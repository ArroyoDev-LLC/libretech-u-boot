// SPDX-License-Identifier: GPL-2.0+
/*
 * USB CDC serial (ACM) function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 * Copyright (C) 2009 by Samsung Electronics
 * Copyright (c) 2021, Linaro Ltd <loic.poulain@linaro.org>
 */

#include <circbuf.h>
#include <console.h>
#include <dfu.h>
#include <errno.h>
#include <g_dnl.h>
#include <malloc.h>
#include <memalign.h>
#include <stdio_dev.h>
#include <time.h>
#include <version.h>
#include <watchdog.h>

#include <linux/string.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>

#define REQ_SIZE_MAX	512

struct f_acm {
	int ctrl_id;
	int data_id;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;
	struct usb_ep *ep_notify;

	struct usb_request *req_in;
	struct usb_request *req_out;
	struct usb_request *req_notify;

	bool connected;
	bool tx_on;
	/* The host left a packet unread past ACM_TX_STALL_MS: stop waiting for it */
	bool tx_stalled;

	circbuf_t rx_buf;
	circbuf_t tx_buf;

	struct usb_function usb_function;

	struct usb_cdc_line_coding line_coding;
	u16 handshake_bits;
#define ACM_CTRL_RTS	BIT(1)	/* unused with full duplex */
#define ACM_CTRL_DTR	BIT(0)	/* host is ready for data r/w */

	struct udevice *udc;

	/*
	 * Bound beside another function in the download gadget
	 * (USB_GADGET_DNL_ACM) rather than owning the gadget: whoever registered
	 * the gadget services the UDC, so this instance never calls
	 * dm_usb_gadget_handle_interrupts() and is drained from its completions.
	 */
	bool attached;
};

static struct f_acm *default_acm_function;
/* The live attached instance, NULL whenever no download gadget carries one */
static struct f_acm *dnl_acm_function;

static void acm_dnl_kick(struct f_acm *f_acm);
static void acm_dnl_unsent(struct usb_request *req);

static inline struct f_acm *func_to_acm(struct usb_function *f)
{
	return container_of(f, struct f_acm, usb_function);
}

static inline struct f_acm *stdio_to_acm(struct stdio_dev *s)
{
	/* stdio dev is cloned on registration, do not use container_of */
	return s->priv;
}

static struct usb_interface_assoc_descriptor
acm_iad_descriptor = {
	.bLength =              sizeof(acm_iad_descriptor),
	.bDescriptorType =      USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface =      0,
	.bInterfaceCount =      2,	// control + data
	.bFunctionClass =       USB_CLASS_COMM,
	.bFunctionSubClass =    USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol =    USB_CDC_ACM_PROTO_AT_V25TER,
};

static struct usb_interface_descriptor acm_control_intf_desc = {
	.bLength =              USB_DT_INTERFACE_SIZE,
	.bDescriptorType =      USB_DT_INTERFACE,
	.bNumEndpoints =        1,
	.bInterfaceClass =      USB_CLASS_COMM,
	.bInterfaceSubClass =   USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =   USB_CDC_ACM_PROTO_AT_V25TER,
};

static struct usb_interface_descriptor acm_data_intf_desc = {
	.bLength =              sizeof(acm_data_intf_desc),
	.bDescriptorType =      USB_DT_INTERFACE,
	.bNumEndpoints =        2,
	.bInterfaceClass =      USB_CLASS_CDC_DATA,
};

static struct usb_cdc_header_desc acm_header_desc = {
	.bLength =		sizeof(acm_header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		__constant_cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor acm_call_mgmt_desc = {
	.bLength =              sizeof(acm_call_mgmt_desc),
	.bDescriptorType =      USB_DT_CS_INTERFACE,
	.bDescriptorSubType =   USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities =       0,
	.bDataInterface =       0x01,
};

static struct usb_cdc_acm_descriptor acm_desc = {
	.bLength =		sizeof(acm_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,
	.bmCapabilities =	USB_CDC_CAP_LINE,
};

static struct usb_cdc_union_desc acm_union_desc = {
	.bLength =		sizeof(acm_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	.bMasterInterface0 =	0x00,
	.bSlaveInterface0 =	0x01,
};

static struct usb_endpoint_descriptor acm_fs_notify_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     3 | USB_DIR_IN,
	.bmAttributes =         USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =       __constant_cpu_to_le16(64),
	.bInterval =            32,
};

static struct usb_endpoint_descriptor acm_fs_in_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_IN,
	.bmAttributes =         USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor acm_fs_out_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bEndpointAddress =     USB_DIR_OUT,
	.bmAttributes =         USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *acm_fs_function[] = {
	(struct usb_descriptor_header *)&acm_iad_descriptor,
	(struct usb_descriptor_header *)&acm_control_intf_desc,
	(struct usb_descriptor_header *)&acm_header_desc,
	(struct usb_descriptor_header *)&acm_call_mgmt_desc,
	(struct usb_descriptor_header *)&acm_desc,
	(struct usb_descriptor_header *)&acm_union_desc,
	(struct usb_descriptor_header *)&acm_fs_notify_desc,
	(struct usb_descriptor_header *)&acm_data_intf_desc,
	(struct usb_descriptor_header *)&acm_fs_in_desc,
	(struct usb_descriptor_header *)&acm_fs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor acm_hs_notify_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bmAttributes =         USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =       __constant_cpu_to_le16(64),
	.bInterval =            11,
};

static struct usb_endpoint_descriptor acm_hs_in_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor acm_hs_out_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= __constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *acm_hs_function[] = {
	(struct usb_descriptor_header *)&acm_iad_descriptor,
	(struct usb_descriptor_header *)&acm_control_intf_desc,
	(struct usb_descriptor_header *)&acm_header_desc,
	(struct usb_descriptor_header *)&acm_call_mgmt_desc,
	(struct usb_descriptor_header *)&acm_desc,
	(struct usb_descriptor_header *)&acm_union_desc,
	(struct usb_descriptor_header *)&acm_hs_notify_desc,
	(struct usb_descriptor_header *)&acm_data_intf_desc,
	(struct usb_descriptor_header *)&acm_hs_in_desc,
	(struct usb_descriptor_header *)&acm_hs_out_desc,
	NULL,
};

/*
 * SuperSpeed: a DWC3 on a USB 3 port (RK3399's Type-C OTG) connects at
 * SuperSpeed, and a host then asks for the SuperSpeed configuration; without
 * one it reads "no configurations" and gives up. Bulk is 1024 B, as in
 * f_fastboot; the companions ask for no bursts or streams.
 */
static struct usb_endpoint_descriptor acm_ss_notify_desc = {
	.bLength =              USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =      USB_DT_ENDPOINT,
	.bmAttributes =         USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =       __constant_cpu_to_le16(64),
	.bInterval =            11,
};

static struct usb_ss_ep_comp_descriptor acm_ss_notify_comp_desc = {
	.bLength =		sizeof(acm_ss_notify_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	.wBytesPerInterval =	__constant_cpu_to_le16(64),
};

static struct usb_endpoint_descriptor acm_ss_in_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= __constant_cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor acm_ss_out_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= __constant_cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor acm_ss_bulk_comp_desc = {
	.bLength =		sizeof(acm_ss_bulk_comp_desc),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *acm_ss_function[] = {
	(struct usb_descriptor_header *)&acm_iad_descriptor,
	(struct usb_descriptor_header *)&acm_control_intf_desc,
	(struct usb_descriptor_header *)&acm_header_desc,
	(struct usb_descriptor_header *)&acm_call_mgmt_desc,
	(struct usb_descriptor_header *)&acm_desc,
	(struct usb_descriptor_header *)&acm_union_desc,
	(struct usb_descriptor_header *)&acm_ss_notify_desc,
	(struct usb_descriptor_header *)&acm_ss_notify_comp_desc,
	(struct usb_descriptor_header *)&acm_data_intf_desc,
	(struct usb_descriptor_header *)&acm_ss_in_desc,
	(struct usb_descriptor_header *)&acm_ss_bulk_comp_desc,
	(struct usb_descriptor_header *)&acm_ss_out_desc,
	(struct usb_descriptor_header *)&acm_ss_bulk_comp_desc,
	NULL,
};

static inline struct usb_endpoint_descriptor *
ep_desc(struct usb_gadget *g, struct usb_endpoint_descriptor *ss,
	struct usb_endpoint_descriptor *hs, struct usb_endpoint_descriptor *fs)
{
	if (gadget_is_superspeed(g) && g->speed >= USB_SPEED_SUPER)
		return ss;
	if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
		return hs;
	return fs;
}

static int acm_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_gadget *gadget = c->cdev->gadget;
	struct f_acm *f_acm = func_to_acm(f);
	struct usb_ep *ep;
	int id;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;

	acm_iad_descriptor.bFirstInterface = id;
	acm_control_intf_desc.bInterfaceNumber = id;
	acm_union_desc.bMasterInterface0 = id;

	f_acm->ctrl_id = id;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;

	acm_data_intf_desc.bInterfaceNumber = id;
	acm_union_desc.bSlaveInterface0 = id;
	acm_call_mgmt_desc.bDataInterface = id;

	f_acm->data_id = id;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(gadget, &acm_fs_in_desc);
	if (!ep)
		return -ENODEV;

	f_acm->ep_in = ep;
	ep->driver_data = c->cdev; /* claim */

	ep = usb_ep_autoconfig(gadget, &acm_fs_out_desc);
	if (!ep)
		return -ENODEV;

	f_acm->ep_out = ep;
	ep->driver_data = c->cdev; /* claim */

	ep = usb_ep_autoconfig(gadget, &acm_fs_notify_desc);
	if (!ep)
		return -ENODEV;

	f_acm->ep_notify = ep;
	ep->driver_data = c->cdev; /* claim */

	if (gadget_is_dualspeed(gadget)) {
		/* Assume endpoint addresses are the same for both speeds */
		acm_hs_in_desc.bEndpointAddress = acm_fs_in_desc.bEndpointAddress;
		acm_hs_out_desc.bEndpointAddress = acm_fs_out_desc.bEndpointAddress;
		acm_hs_notify_desc.bEndpointAddress = acm_fs_notify_desc.bEndpointAddress;
	}

	if (gadget_is_superspeed(gadget)) {
		acm_ss_in_desc.bEndpointAddress = acm_fs_in_desc.bEndpointAddress;
		acm_ss_out_desc.bEndpointAddress = acm_fs_out_desc.bEndpointAddress;
		acm_ss_notify_desc.bEndpointAddress = acm_fs_notify_desc.bEndpointAddress;
		f->ss_descriptors = acm_ss_function;
	}

	return 0;
}

static void acm_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_acm *f_acm = func_to_acm(f);

	if (default_acm_function == f_acm)
		default_acm_function = NULL;
	if (dnl_acm_function == f_acm)
		dnl_acm_function = NULL;

	buf_free(&f_acm->rx_buf);
	buf_free(&f_acm->tx_buf);

	free(f_acm);
}

static void acm_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	/* nothing to do */
}

static void acm_tx_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_acm *f_acm = req->context;

	f_acm->tx_on = true;

	if (!f_acm->attached)
		return;

	if (!req->status)
		acm_dnl_kick(f_acm);
	else if (req->status == -ESHUTDOWN || req->status == -ECONNRESET)
		acm_dnl_unsent(req);
}

static void acm_rx_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_acm *f_acm = req->context;

	/*
	 * The endpoint is being torn down (usb_ep_disable() -> nuke(), or a
	 * disconnect): do not queue the request again. On dwc2 the endpoint is
	 * still enabled while nuke() completes its requests, so a re-queue here
	 * lands back on the list nuke() is draining and the loop never ends --
	 * every acm_disable() of a connected function hung in teardown.
	 */
	if (req->status == -ESHUTDOWN || req->status == -ECONNRESET)
		return;

	if (!req->status)
		buf_push(&f_acm->rx_buf, req->buf, req->actual);

	/* Queue RX req again */
	req->actual = 0;
	usb_ep_queue(ep, req, 0);
}

/*
 * [len] is the request's buffer: REQ_SIZE_MAX, or for the OUT endpoint a whole
 * number of its packets (acm_start_data).
 */
static struct usb_request *acm_start_ep(struct usb_ep *ep, void *complete_cb,
					void *context, unsigned int len)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, 0);
	if (!req)
		return NULL;

	req->length = len;
	req->buf = memalign(CONFIG_SYS_CACHELINE_SIZE, len);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	memset(req->buf, 0, req->length);
	req->complete = complete_cb;
	req->context = context;

	return req;
}

/*
 * Take the data interface down: no output may reach it any more, then disable
 * its endpoints and free their requests. Disabling the IN endpoint completes
 * a queued req_in with -ESHUTDOWN, so the attached instance's unsent bytes go
 * back to the dnlacm ring (acm_tx_complete()) before the request is freed.
 * Safe when the interface never started: the requests are NULL then, and a
 * disable of an endpoint that is not enabled does nothing.
 */
static void acm_stop_data(struct f_acm *f_acm)
{
	f_acm->connected = false;

	usb_ep_disable(f_acm->ep_out);
	usb_ep_disable(f_acm->ep_in);

	if (f_acm->req_out) {
		free(f_acm->req_out->buf);
		usb_ep_free_request(f_acm->ep_out, f_acm->req_out);
		f_acm->req_out = NULL;
	}

	if (f_acm->req_in) {
		free(f_acm->req_in->buf);
		usb_ep_free_request(f_acm->ep_in, f_acm->req_in);
		f_acm->req_in = NULL;
	}
}

static int acm_start_data(struct f_acm *f_acm, struct usb_gadget *gadget)
{
	const struct usb_endpoint_descriptor *d;
	int ret;

	/*
	 * A SET_INTERFACE reaches set_alt with no disable before it
	 * (composite.c), so the endpoints may still be enabled from the last
	 * set_alt; usb_ep_enable() of an enabled dwc2 endpoint fails. Restart
	 * from down, as Linux f_acm does.
	 */
	acm_stop_data(f_acm);

	/* EP IN */
	d = ep_desc(gadget, &acm_ss_in_desc, &acm_hs_in_desc, &acm_fs_in_desc);
	ret = usb_ep_enable(f_acm->ep_in, d);
	if (ret)
		return ret;

	f_acm->req_in = acm_start_ep(f_acm->ep_in, acm_tx_complete, f_acm,
				     REQ_SIZE_MAX);
	if (!f_acm->req_in)
		return -ENOMEM;

	/* EP OUT */
	d = ep_desc(gadget, &acm_ss_out_desc, &acm_hs_out_desc, &acm_fs_out_desc);
	ret = usb_ep_enable(f_acm->ep_out, d);
	if (ret)
		return ret;

	/*
	 * A whole number of packets: an OUT buffer that is not a multiple of
	 * wMaxPacketSize is not allowed on DWC3, and a SuperSpeed bulk packet
	 * (1024) is larger than REQ_SIZE_MAX. U-Boot's usb_ep_enable() does
	 * not copy wMaxPacketSize into ep->maxpacket, so read the descriptor.
	 */
	f_acm->req_out = acm_start_ep(f_acm->ep_out, acm_rx_complete, f_acm,
				      roundup(REQ_SIZE_MAX, usb_endpoint_maxp(d)));
	if (!f_acm->req_out)
		return -ENOMEM;

	/* Start OUT transfer (EP OUT) */
	ret = usb_ep_queue(f_acm->ep_out, f_acm->req_out, 0);
	if (ret)
		return ret;

	return 0;
}

static int acm_start_ctrl(struct f_acm *f_acm, struct usb_gadget *gadget)
{
	const struct usb_endpoint_descriptor *d;

	usb_ep_disable(f_acm->ep_notify);

	d = ep_desc(gadget, &acm_ss_notify_desc, &acm_hs_notify_desc, &acm_fs_notify_desc);
	usb_ep_enable(f_acm->ep_notify, d);

	/*
	 * The request lives from the first set_alt to acm_disable().  set_alt
	 * can run again without a disable in between (SET_INTERFACE), so keep
	 * the one already allocated instead of leaking it.  (req_in/req_out
	 * are freed and allocated again on every data set_alt instead:
	 * acm_start_data() starts with acm_stop_data().)
	 */
	if (!f_acm->req_notify)
		f_acm->req_notify = acm_start_ep(f_acm->ep_notify,
						 acm_notify_complete, f_acm,
						 REQ_SIZE_MAX);

	return 0;
}

static int acm_set_alt(struct usb_function *f, unsigned int intf, unsigned int alt)
{
	struct usb_gadget *gadget = f->config->cdev->gadget;
	struct f_acm *f_acm = func_to_acm(f);
	int ret;

	if (intf == f_acm->ctrl_id) {
		return acm_start_ctrl(f_acm, gadget);
	} else if (intf == f_acm->data_id) {
		ret = acm_start_data(f_acm, gadget);
		if (ret) {
			/*
			 * Say so: a data interface that never started used to
			 * look like a console that prints nothing. Mark it down
			 * first, so this printf does not try to send on it.
			 */
			f_acm->connected = false;
			printf("f_acm: data interface %u failed to start: %d%s\n",
			       intf, ret, f_acm->attached ?
			       " -- ACM console off for this session" : "");
			/*
			 * An error from set_alt makes composite reset the whole
			 * configuration. Beside DFU that would take DFU down
			 * with the console, so the attached instance carries on
			 * without ACM; the stand-alone console has nothing else
			 * to keep and fails the configuration.
			 */
			return f_acm->attached ? 0 : ret;
		}
		f_acm->connected = true;
		f_acm->tx_on = true;
		/* On a restart, send what the old requests handed back */
		if (f_acm->attached)
			acm_dnl_kick(f_acm);
		return 0;
	}

	return -EINVAL;
}

static int acm_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_gadget *gadget =  f->config->cdev->gadget;
	struct usb_request *req = f->config->cdev->req;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);
	struct f_acm *f_acm = func_to_acm(f);
	int value = -1;

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_LINE_CODING:
		/* SET_LINE_CODING */

		if (w_length != sizeof(f_acm->line_coding) || w_index != f_acm->ctrl_id)
			goto invalid;

		value = w_length;

		memcpy(&f_acm->line_coding, req->buf, value);

		break;
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_GET_LINE_CODING:
		/* GET_LINE_CODING */

		if (w_length != sizeof(f_acm->line_coding) || w_index != f_acm->ctrl_id)
			goto invalid;

		value = w_length;

		memcpy(req->buf, &f_acm->line_coding, value);

		break;
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		/* SET_CONTROL_LINE_STATE */

		if (w_index != f_acm->ctrl_id)
			goto invalid;

		value = 0;

		f_acm->handshake_bits = w_value;

		break;
	default:
invalid:
		printf("invalid control req%02x.%02x v%04x i%04x l%d\n",
		       ctrl->bRequestType, ctrl->bRequest, w_value, w_index,
		       w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		req->zero = 0;
		req->length = value;
		usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
	}

	/* The host just raised DTR: send what was buffered before it opened */
	if (f_acm->attached)
		acm_dnl_kick(f_acm);

	return 0;
}

static void acm_disable(struct usb_function *f)
{
	struct f_acm *f_acm = func_to_acm(f);

	/*
	 * Clears `connected` first: a printf between this reset_config and the
	 * next set_alt must not reach the freed req_in.
	 */
	acm_stop_data(f_acm);
	usb_ep_disable(f_acm->ep_notify);

	if (f_acm->req_notify) {
		free(f_acm->req_notify->buf);
		usb_ep_free_request(f_acm->ep_notify, f_acm->req_notify);
		f_acm->req_notify = NULL;
	}
}

/* static strings, in UTF-8 */
static struct usb_string acm_string_defs[] = {
	[0].s = "CDC Abstract Control Model (ACM)",
	[1].s = "CDC ACM Data",
	[2].s = "CDC Serial",
	{  } /* end of list */
};

static struct usb_gadget_strings acm_string_table = {
	.language = 0x0409, /* en-us */
	.strings = acm_string_defs,
};

static struct usb_gadget_strings *acm_strings[] = {
	&acm_string_table,
	NULL,
};

/*
 * The stand-alone `usbacm` console (it owns the gadget and services the UDC)
 * and every stdio device are U-Boot proper's: an xPL phase has no stdio. In
 * xPL only the attached instance is built (SPL_USB_GADGET_DNL_ACM), and its
 * output comes from the console tee in common/console.c.
 */
#ifndef CONFIG_XPL_BUILD
/*
 * How long a console write waits for the host to take the previous packet. A
 * host can raise DTR and then stop reading (the reader exits and leaves DTR
 * up), and the packet then never completes: waiting for it hung U-Boot on the
 * next character, until the watchdog reset the board (MT8188, 2026-10-12).
 * Past this the bytes stay in tx_buf, and later writes do not wait at all
 * until the host takes the packet (acm_tx_complete sets tx_on).
 */
#define ACM_TX_STALL_MS	100

static void __acm_tx(struct f_acm *f_acm)
{
	ulong start = get_timer(0);
	int len, ret;

	do {
		dm_usb_gadget_handle_interrupts(f_acm->udc);

		/* A reset just serviced may have run acm_disable() */
		if (!f_acm->connected)
			break;

		if (!(f_acm->handshake_bits & ACM_CTRL_DTR))
			break;

		if (!f_acm->tx_on) {
			if (f_acm->tx_stalled ||
			    get_timer(start) > ACM_TX_STALL_MS) {
				f_acm->tx_stalled = true;
				break;
			}
			continue;
		}
		f_acm->tx_stalled = false;

		len = buf_pop(&f_acm->tx_buf, f_acm->req_in->buf, REQ_SIZE_MAX);
		if (!len)
			break;

		f_acm->req_in->length = len;

		ret = usb_ep_queue(f_acm->ep_in, f_acm->req_in, 0);
		if (ret)
			break;

		f_acm->tx_on = false;
		start = get_timer(0);
	} while (1);
}

static bool acm_connected(struct stdio_dev *dev)
{
	struct f_acm *f_acm = stdio_to_acm(dev);

	/* give a chance to process udc irq */
	dm_usb_gadget_handle_interrupts(f_acm->udc);

	return f_acm->connected;
}
#endif /* !CONFIG_XPL_BUILD */

static int acm_add_common(struct usb_configuration *c, bool attached)
{
	struct f_acm *f_acm;
	int status;

	f_acm = calloc(1, sizeof(*f_acm));
	if (!f_acm)
		return -ENOMEM;

	f_acm->attached = attached;

	f_acm->usb_function.name = "f_acm";
	f_acm->usb_function.bind = acm_bind;
	f_acm->usb_function.unbind = acm_unbind;
	f_acm->usb_function.set_alt = acm_set_alt;
	f_acm->usb_function.disable = acm_disable;
	f_acm->usb_function.strings = acm_strings;
	f_acm->usb_function.descriptors = acm_fs_function;
	f_acm->usb_function.hs_descriptors = acm_hs_function;
	f_acm->usb_function.setup = acm_setup;

	/*
	 * Only the stand-alone usbacm console services the UDC itself (through
	 * f_acm->udc). An attached instance never touches it, and looking the
	 * UDC up here is not free: without DM_USB_GADGET it runs
	 * board_usb_init() again in the middle of the download gadget's bind,
	 * and it always names controller 0, whichever one that gadget is on.
	 */
	if (!attached) {
		status = udc_device_get_by_index(0, &f_acm->udc);
		if (status)
			goto err;
	}

	status = usb_add_function(c, &f_acm->usb_function);
	if (status)
		goto err;

	buf_init(&f_acm->rx_buf, 2048);
	buf_init(&f_acm->tx_buf, CONFIG_USB_FUNCTION_ACM_TX_BUF_SIZE);

	if (attached)
		dnl_acm_function = f_acm;
	else if (!default_acm_function)
		default_acm_function = f_acm;

	return 0;

err:
	free(f_acm);
	return status;
}

#ifndef CONFIG_XPL_BUILD
static int acm_add(struct usb_configuration *c)
{
	return acm_add_common(c, false);
}

DECLARE_GADGET_BIND_CALLBACK(usb_serial_acm, acm_add);
#endif

#if CONFIG_IS_ENABLED(USB_GADGET_DNL_ACM)
/*
 * Add an ACM function to a download-gadget configuration that already holds
 * another function (g_dnl_do_config(), for usb_dnl_dfu). Its console is the
 * `dnlacm` stdio device below, never `usbacm`, which keeps owning the gadget.
 *
 * If the function cannot bind (no endpoints left on this UDC), the interface
 * numbers it took are handed back so the configuration describes only the
 * functions that are really there, and the caller carries on without it.
 */
int acm_dnl_attach(struct usb_configuration *c)
{
	unsigned int first = c->next_interface_id, i;
	int ret;

	if (dnl_acm_function)
		return -EBUSY;

	ret = acm_add_common(c, true);
	if (ret) {
		for (i = first; i < c->next_interface_id &&
		     i < MAX_CONFIG_INTERFACES; i++)
			c->interface[i] = NULL;
		c->next_interface_id = first;
	}

	return ret;
}

/*
 * The `dnlacm` output buffer outlives the gadget: what is printed between
 * two DFU sessions (the commands a session's script queued, `ums` output)
 * is kept, bounded -- the oldest bytes are overwritten -- and drained into
 * the next session's ttyACM once its host raises DTR.
 *
 * 16 KiB rather than 4: on the GX chain the xPL ring also starts with the
 * boot log of the stages before it (acm_dnl_console_write()), and it must
 * still hold the TPL's first line when the host opens the port.
 */
#define DNL_TX_SIZE	16384
static circbuf_t dnl_tx_buf;

/*
 * Hand bytes that were popped for the IN request but never reached the host
 * back to the FRONT of dnl_tx_buf, in their original order, so the next
 * session sends them first. They are older than everything the ring holds,
 * so when it has no room for all of them the oldest go, exactly as
 * buf_push() overwrites the oldest bytes: only the last `free space` bytes
 * of src are kept.
 */
static void acm_dnl_unpop(const char *src, unsigned int len)
{
	unsigned int room;

	if (!dnl_tx_buf.data || !len)
		return;

	room = dnl_tx_buf.totalsize - dnl_tx_buf.size;
	if (len > room) {
		src += len - room;
		len = room;
	}

	while (len--) {
		if (dnl_tx_buf.top == dnl_tx_buf.data)
			dnl_tx_buf.top = dnl_tx_buf.end;
		*--dnl_tx_buf.top = src[len];
		dnl_tx_buf.size++;
	}
}

/*
 * An IN request completed by a teardown (-ESHUTDOWN from usb_ep_disable(),
 * -ECONNRESET from a dequeue): whatever it did not send goes back into the
 * ring. Without this each session's last line -- the `Gadget ... stopped`
 * that ends it -- was lost instead of reaching the next session's ttyACM.
 */
static void acm_dnl_unsent(struct usb_request *req)
{
	unsigned int sent = min(req->actual, req->length);

	acm_dnl_unpop(req->buf + sent, req->length - sent);
}

/*
 * Send the next chunk of dnl_tx_buf if the host is listening and the IN
 * request is idle. Called from putc/puts, from the IN completion and from
 * SET_CONTROL_LINE_STATE, so the buffer drains while the download gadget's
 * own loop services the UDC; nothing here services it, so a printf from
 * inside a completion (DFU prints from its handlers) cannot re-enter the
 * UDC driver. At most REQ_SIZE_MAX - 1 bytes go per request: a short packet
 * always ends the host's read, so no zero-length packet is needed.
 */
static void acm_dnl_kick(struct f_acm *f_acm)
{
	int len;

	if (!f_acm->connected || !f_acm->req_in || !f_acm->tx_on)
		return;
	if (!(f_acm->handshake_bits & ACM_CTRL_DTR))
		return;
	if (!dnl_tx_buf.data)
		return;

	len = buf_pop(&dnl_tx_buf, f_acm->req_in->buf, REQ_SIZE_MAX - 1);
	if (!len)
		return;

	f_acm->req_in->length = len;
	f_acm->tx_on = false;
	if (usb_ep_queue(f_acm->ep_in, f_acm->req_in, 0)) {
		/* Not sent: keep the bytes for the next kick or session */
		acm_dnl_unpop(f_acm->req_in->buf, len);
		f_acm->tx_on = true;
	}
}

/*
 * U-Boot proper allocates the ring on the first byte, once stdio is up. An
 * xPL phase tees every console byte here from the start of the phase, before
 * any heap may exist (and a failed allocation prints, which would land back
 * here), so there the ring exists only once acm_dnl_console_start() made it.
 */
static bool acm_dnl_buf_ready(void)
{
	if (!dnl_tx_buf.data && !IS_ENABLED(CONFIG_XPL_BUILD))
		buf_init(&dnl_tx_buf, DNL_TX_SIZE);

	return dnl_tx_buf.data;
}

#ifdef CONFIG_XPL_BUILD
/*
 * Set once the ring exists. In .data, not BSS: the console tee asks before
 * crt0 has cleared BSS (dnl_tx_buf may hold the previous stage's leftovers
 * then), and .data comes with the image.
 */
static bool dnl_ring_on __section(".data");

bool acm_dnl_console_on(void)
{
	return dnl_ring_on;
}

/* Start keeping console bytes for the ACM console; call with a heap up. */
int acm_dnl_console_start(void)
{
	if (!dnl_tx_buf.data)
		buf_init(&dnl_tx_buf, DNL_TX_SIZE);
	dnl_ring_on = dnl_tx_buf.data;

	return dnl_tx_buf.data ? 0 : -ENOMEM;
}

/*
 * Bytes that are already a terminal transcript -- an earlier stage's UART
 * output, CR/LF included -- so no CR is added. Oldest-first, like the ring.
 */
void acm_dnl_console_write(const char *s, unsigned int len)
{
	if (!dnl_tx_buf.data || !len)
		return;

	buf_push(&dnl_tx_buf, s, len);
	if (dnl_acm_function)
		acm_dnl_kick(dnl_acm_function);
}
#endif

static void acm_dnl_push(const char c)
{
	if (c == '\n')
		buf_push(&dnl_tx_buf, "\r", 1);
	buf_push(&dnl_tx_buf, &c, 1);
}

/*
 * One console byte / string for the ACM console: into the ring, then sent if
 * the host is listening. The `dnlacm` stdio device of U-Boot proper and the
 * xPL console tee (common/console.c) both come here.
 */
void acm_dnl_console_putc(const char c)
{
	if (!acm_dnl_buf_ready())
		return;

	acm_dnl_push(c);
	if (dnl_acm_function)
		acm_dnl_kick(dnl_acm_function);
}

void acm_dnl_console_puts(const char *str)
{
	if (!acm_dnl_buf_ready())
		return;

	while (*str)
		acm_dnl_push(*str++);
	if (dnl_acm_function)
		acm_dnl_kick(dnl_acm_function);
}

#ifndef CONFIG_XPL_BUILD
static void acm_dnl_stdio_putc(struct stdio_dev *dev, const char c)
{
	acm_dnl_console_putc(c);
}

static void acm_dnl_stdio_puts(struct stdio_dev *dev, const char *str)
{
	acm_dnl_console_puts(str);
}

/* Input only ever arrives while the gadget's loop is servicing the UDC */
static int acm_dnl_stdio_tstc(struct stdio_dev *dev)
{
	struct f_acm *f_acm = dnl_acm_function;

	return f_acm && f_acm->rx_buf.size > 0;
}

static int acm_dnl_stdio_getc(struct stdio_dev *dev)
{
	struct f_acm *f_acm = dnl_acm_function;
	char c;

	if (!acm_dnl_stdio_tstc(dev))
		return 0;

	buf_pop(&f_acm->rx_buf, &c, 1);

	return c;
}

/* Never touches the UDC: the gadget belongs to whoever registered it */
static int acm_dnl_stdio_start(struct stdio_dev *dev)
{
	return 0;
}

static int acm_dnl_stdio_stop(struct stdio_dev *dev)
{
	return 0;
}

static int drv_dnlacm_init(void)
{
	struct stdio_dev stdio;

	memset(&stdio, 0, sizeof(stdio));
	strcpy(stdio.name, "dnlacm");
	stdio.flags = DEV_FLAGS_INPUT | DEV_FLAGS_OUTPUT;
	stdio.tstc = acm_dnl_stdio_tstc;
	stdio.getc = acm_dnl_stdio_getc;
	stdio.putc = acm_dnl_stdio_putc;
	stdio.puts = acm_dnl_stdio_puts;
	stdio.start = acm_dnl_stdio_start;
	stdio.stop = acm_dnl_stdio_stop;

	return stdio_register(&stdio);
}
#endif /* !CONFIG_XPL_BUILD */
#else
static void acm_dnl_kick(struct f_acm *f_acm)
{
}

static void acm_dnl_unsent(struct usb_request *req)
{
}

static int drv_dnlacm_init(void)
{
	return 0;
}
#endif

#ifndef CONFIG_XPL_BUILD
/*
 * The usbacm console while it is started, and whether a download gadget (dfu,
 * fastboot, ums...) has it suspended: there is one UDC, and the composite layer
 * takes one gadget at a time. While suspended dev->priv is NULL and the stdio
 * calls below do nothing; the UART keeps the console.
 */
static struct stdio_dev *acm_console;
static bool acm_console_suspended;

#if IS_ENABLED(CONFIG_USB_FUNCTION_ACM_DFU)
/*
 * USB_FUNCTION_ACM_DFU: one composite gadget from the console's start, the
 * ACM console and a resident DFU function (f_dfu.c) in one configuration, so
 * a host reads and writes the board's flash at the prompt without the board
 * leaving the bus: no `dfu` command, no re-enumeration, and the console stays
 * open through the transfer. DFU requests are served while the console polls
 * the UDC, i.e. while U-Boot waits at its prompt.
 *
 * The DFU core has one entity list. The console builds it from
 * USB_FUNCTION_ACM_DFU_ALT ("<interface> <devstr>", through the board's
 * set_dfu_alt_info() like `dfu <interface> <devstr>`) before it registers,
 * and frees it when its gadget goes. Anyone else building a list first takes
 * it from the console (dfu_console_release(), from dfu_init_env_entities()),
 * and every gadget that suspends the console frees it too. The console then
 * comes back lazily, at its next input poll once no download gadget runs and
 * the list is free -- not at g_dnl_unregister(), where the `dfu` command
 * still holds its list.
 */
static bool acm_console_dfu;		/* its gadget carries DFU, and holds the list */
static bool acm_console_resume_pending;	/* to come back at the next free poll */
static bool acm_console_starting;

static void acm_console_dfu_build(void)
{
	char alt[] = CONFIG_USB_FUNCTION_ACM_DFU_ALT;
	char *dev;

	acm_console_dfu = false;
	if (dfu_entities_in_use())
		return;
	dev = strchr(alt, ' ');
	if (dev) {
		*dev++ = '\0';
		dev = skip_spaces(dev);
	}
	if (dfu_init_env_entities(alt, dev)) {
		dfu_free_entities();
		printf("USB console: no DFU beside it (%s)\n",
		       CONFIG_USB_FUNCTION_ACM_DFU_ALT);
		return;
	}
	acm_console_dfu = true;
}

/* After the console's gadget is unregistered: its DFU function is unbound */
static void acm_console_dfu_free(void)
{
	if (!acm_console_dfu)
		return;
	acm_console_dfu = false;
	dfu_free_entities();
}

bool acm_console_has_dfu(void)
{
	return acm_console_dfu;
}

void dfu_console_release(void)
{
	if (!acm_console_dfu || !acm_console || !acm_console->priv)
		return;
	acm_console_suspend();
	acm_console_resume_pending = true;
}

static int acm_stdio_start(struct stdio_dev *dev);

/* The console's lazy return (above): from its own input poll */
static void acm_console_resume_lazy(void)
{
	int ret;

	if (!acm_console_resume_pending || acm_console_starting ||
	    g_dnl_download_active() || dfu_entities_in_use())
		return;
	acm_console_resume_pending = false;
	acm_console_suspended = false;
	clear_ctrlc();
	acm_console_starting = true;
	ret = acm_stdio_start(acm_console);
	acm_console_starting = false;
	if (ret)
		printf("USB console: not back after the download gadget (%d)\n",
		       ret);
}
#else
static void acm_console_dfu_build(void)
{
}

static void acm_console_dfu_free(void)
{
}

bool acm_console_has_dfu(void)
{
	return false;
}
#endif

/* STDIO */
static int acm_stdio_tstc(struct stdio_dev *dev)
{
	struct f_acm *f_acm;

#if IS_ENABLED(CONFIG_USB_FUNCTION_ACM_DFU)
	if (!dev->priv && dev == acm_console)
		acm_console_resume_lazy();
#endif
	if (!dev->priv)
		return 0;
	f_acm = stdio_to_acm(dev);

	dm_usb_gadget_handle_interrupts(f_acm->udc);

	/*
	 * Bytes left in tx_buf by a stalled write go out only on the next write,
	 * so the tail of a command's output (its last line and the prompt) sat
	 * there until the host typed again. The prompt's input loop polls here:
	 * send them once the host has taken the previous packet.
	 */
	if (f_acm->connected && f_acm->tx_on && f_acm->tx_buf.size > 0)
		__acm_tx(f_acm);

	return (f_acm->rx_buf.size > 0);
}

static int acm_stdio_getc(struct stdio_dev *dev)
{
	struct f_acm *f_acm;
	char c;

	/* Wait for a character to arrive. */
	while (!acm_stdio_tstc(dev))
		schedule();

	f_acm = stdio_to_acm(dev);
	buf_pop(&f_acm->rx_buf, &c, 1);

	return c;
}

static void acm_stdio_putc(struct stdio_dev *dev, const char c)
{
	struct f_acm *f_acm;

	if (!dev->priv)
		return;
	f_acm = stdio_to_acm(dev);

	if (c == '\n')
		buf_push(&f_acm->tx_buf, "\r", 1);

	buf_push(&f_acm->tx_buf, &c, 1);

	if (!f_acm->connected)
		return;

	__acm_tx(f_acm);
}

static void acm_stdio_puts(struct stdio_dev *dev, const char *str)
{
	struct f_acm *f_acm;

	if (!dev->priv)
		return;
	f_acm = stdio_to_acm(dev);

	while (*str) {
		if (*str == '\n')
			buf_push(&f_acm->tx_buf, "\r", 1);

		buf_push(&f_acm->tx_buf, str++, 1);
	}

	if (!f_acm->connected)
		return;

	__acm_tx(f_acm);
}

static int acm_stdio_start(struct stdio_dev *dev)
{
	struct udevice *udc;
	ulong start;
	int ret;

	if (dev->priv) { /* function already exist */
		return 0;
	}

	ret = udc_device_get_by_index(0, &udc);
	if (ret) {
		pr_err("USB init failed: %d\n", ret);
		return ret;
	}

	g_dnl_clear_detach();

	/* USB_FUNCTION_ACM_DFU: the list the gadget's DFU function serves */
	acm_console_dfu_build();

	ret = g_dnl_register("usb_serial_acm");
	if (ret) {
		acm_console_dfu_free();
		return ret;
	}

	if (default_acm_function) {
		dev->priv = default_acm_function;
	} else {
		g_dnl_unregister();
		acm_console_dfu_free();
		return -ENODEV;
	}

	start = get_timer(0);
	while (!acm_connected(dev)) {
		/*
		 * Give up -- on Ctrl-C, or when no host configures the gadget
		 * in time (USB_FUNCTION_ACM_CONNECT_TIMEOUT_MS, so a console
		 * that adds usbacm by itself does not hang a boot with no host
		 * on the port) -- and undo the start either way. A cancel used
		 * to return with the gadget registered and dev->priv set: the
		 * next console output went to a gadget nobody services, and
		 * the board took an SError in acm_stdio_puts().
		 */
		ret = ctrlc() ? -ECANCELED :
		      CONFIG_USB_FUNCTION_ACM_CONNECT_TIMEOUT_MS &&
		      get_timer(start) > CONFIG_USB_FUNCTION_ACM_CONNECT_TIMEOUT_MS ?
		      -ETIMEDOUT : 0;
		if (ret) {
			g_dnl_unregister();
			g_dnl_clear_detach();
			acm_console_dfu_free();
			dev->priv = NULL;
			return ret;
		}

		schedule();
	}

	acm_console = dev;

	return 0;
}

static int acm_stdio_stop(struct stdio_dev *dev)
{
	g_dnl_unregister();
	g_dnl_clear_detach();
	acm_console_dfu_free();
	dev->priv = NULL;
	acm_console = NULL;
	acm_console_suspended = false;
#if IS_ENABLED(CONFIG_USB_FUNCTION_ACM_DFU)
	acm_console_resume_pending = false;
#endif

	return 0;
}

void acm_console_suspend(void)
{
	if (!acm_console || !acm_console->priv)
		return;
	g_dnl_unregister();
	g_dnl_clear_detach();
	acm_console_dfu_free();
	acm_console->priv = NULL;
	acm_console_suspended = true;
}

void acm_console_resume(void)
{
	int ret;

	if (!acm_console || !acm_console_suspended)
		return;
#if IS_ENABLED(CONFIG_USB_FUNCTION_ACM_DFU)
	/* Lazily, from the console's next input poll (above) */
	acm_console_resume_pending = true;
	return;
#endif
	acm_console_suspended = false;
	/*
	 * A Ctrl-C that ended the download gadget (`dfu`, `fastboot`, `ums`) is
	 * still latched (ctrlc() answers it until cleared): it is not a request
	 * to cancel the console's return.
	 */
	clear_ctrlc();
	ret = acm_stdio_start(acm_console);
	if (ret)
		printf("USB console: not back after the download gadget (%d)\n",
		       ret);
}

int drv_usbacm_init(void)
{
	struct stdio_dev stdio;
	int ret;

	strcpy(stdio.name, "usbacm");
	stdio.flags = DEV_FLAGS_INPUT | DEV_FLAGS_OUTPUT;
	stdio.tstc = acm_stdio_tstc;
	stdio.getc = acm_stdio_getc;
	stdio.putc = acm_stdio_putc;
	stdio.puts = acm_stdio_puts;
	stdio.start = acm_stdio_start;
	stdio.stop = acm_stdio_stop;
	stdio.priv = NULL;
	stdio.ext = 0;

	ret = stdio_register(&stdio);
	if (ret)
		return ret;

	return drv_dnlacm_init();
}
#endif /* !CONFIG_XPL_BUILD */
