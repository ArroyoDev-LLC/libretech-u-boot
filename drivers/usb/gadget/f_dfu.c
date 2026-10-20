// SPDX-License-Identifier: GPL-2.0+
/*
 * f_dfu.c -- Device Firmware Update USB function
 *
 * Copyright (C) 2012 Samsung Electronics
 * authors: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *          Lukasz Majewski <l.majewski@samsung.com>
 *
 * Based on OpenMoko u-boot: drivers/usb/usbdfu.c
 * (C) 2007 by OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 *
 * based on existing SAM7DFU code from OpenPCD:
 * (C) Copyright 2006 by Harald Welte <hwelte at hmw-consulting.de>
 */

#include <env.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>

#include <vsprintf.h>
#include <linux/ctype.h>
#include <linux/sizes.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

#include <dfu.h>
#include <g_dnl.h>
#include "f_dfu.h"

struct f_dfu {
	struct usb_function		usb_function;

	struct usb_descriptor_header	**function;
	struct usb_string		*strings;

	/* when configured, we have one config */
	u8				config;
	u8				altsetting;
	enum dfu_state			dfu_state;
	unsigned int			dfu_status;

	/* Send/received block number is handy for data integrity check */
	int                             blk_seq_num;
	unsigned int                    poll_timeout;

	/* The EP0 buffer, while a zero-copy block points the request elsewhere */
	void				*ep0_buf;

#if CONFIG_IS_ENABLED(DFU_BULK)
	/* The bulk data path (DFU_BULK): its interface, endpoints, one request each */
	struct usb_descriptor_header	**function_fs;
	struct usb_descriptor_header	**function_hs;
	struct usb_descriptor_header	**function_ss;
	struct usb_ep			*bulk_in, *bulk_out;
	struct usb_request		*bulk_in_req, *bulk_out_req;
	u64				bulk_left;
	bool				bulk_read;
	bool				bulk_ready;	/* interface bound */
	u8				bulk_intf;
#endif
};

struct dfu_entity *dfu_defer_flush;

typedef int (*dfu_state_fn) (struct f_dfu *,
			     const struct usb_ctrlrequest *,
			     struct usb_gadget *,
			     struct usb_request *);

static inline struct f_dfu *func_to_dfu(struct usb_function *f)
{
	return container_of(f, struct f_dfu, usb_function);
}

static const struct dfu_function_descriptor dfu_func = {
	.bLength =		sizeof dfu_func,
	.bDescriptorType =	DFU_DT_FUNC,
	.bmAttributes =		DFU_BIT_WILL_DETACH |
				DFU_BIT_MANIFESTATION_TOLERANT |
				DFU_BIT_CAN_UPLOAD |
				DFU_BIT_CAN_DNLOAD,
	.wDetachTimeOut =	0,
	.wTransferSize =	DFU_USB_BUFSIZ,
	.bcdDFUVersion =	__constant_cpu_to_le16(0x0110),
};

static struct usb_interface_descriptor dfu_intf_runtime = {
	.bLength =		sizeof dfu_intf_runtime,
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_APP_SPEC,
	.bInterfaceSubClass =	1,
	.bInterfaceProtocol =	1,
	/* .iInterface = DYNAMIC */
};

static struct usb_descriptor_header *dfu_runtime_descs[] = {
	(struct usb_descriptor_header *) &dfu_intf_runtime,
	NULL,
};

#if CONFIG_IS_ENABLED(DFU_BULK)
/*
 * The bulk data path, a vendor extension (DFU_BULK). At high speed the host
 * paces EP0 control data per 64-byte packet, which holds DFU near 12 MiB/s
 * whatever the block size; bulk endpoints on the same link move more than
 * twice that. A host that knows this interface:
 *   1. selects the DFU alternate as usual (SET_INTERFACE), and the bulk
 *      interface (alt 0) to enable its endpoints;
 *   2. in dfuIDLE, sends DFU_BULK_DNLOAD or DFU_BULK_UPLOAD (vendor, to the
 *      DFU interface, no data stage) with wValue = N units of 64 KiB;
 *   3. moves those N * 64 KiB over bulk OUT / IN, DMA'd straight to or from
 *      the alternate's memory (dfu_direct_buf());
 *   4. carries on with ordinary DFU blocks numbered from 0 for any rest, and
 *      ends as DFU does (DNLOAD of 0 bytes, or ABORT after an upload).
 * An alternate with no direct buffer refuses step 2 (STALL); the host then
 * uses control transfers for the whole image.
 */
#define DFU_BULK_REQ_DNLOAD	0x01
#define DFU_BULK_REQ_UPLOAD	0x02
#define DFU_BULK_UNIT		SZ_64K
#define DFU_BULK_CHUNK		SZ_1M

static struct usb_interface_descriptor dfu_bulk_intf = {
	.bLength =		sizeof(dfu_bulk_intf),
	.bDescriptorType =	USB_DT_INTERFACE,
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	/* DFU's own class code, so a host can tell this interface from others */
	.bInterfaceSubClass =	USB_CLASS_APP_SPEC,
	.bInterfaceProtocol =	1,
};

#define DFU_BULK_EP(name, dir, mps)					\
static struct usb_endpoint_descriptor name = {				\
	.bLength =		USB_DT_ENDPOINT_SIZE,			\
	.bDescriptorType =	USB_DT_ENDPOINT,			\
	.bEndpointAddress =	dir,					\
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,			\
	.wMaxPacketSize =	cpu_to_le16(mps),			\
}
DFU_BULK_EP(dfu_bulk_in_fs, USB_DIR_IN, 64);
DFU_BULK_EP(dfu_bulk_out_fs, USB_DIR_OUT, 64);
DFU_BULK_EP(dfu_bulk_in_hs, USB_DIR_IN, 512);
DFU_BULK_EP(dfu_bulk_out_hs, USB_DIR_OUT, 512);
DFU_BULK_EP(dfu_bulk_in_ss, USB_DIR_IN, 1024);
DFU_BULK_EP(dfu_bulk_out_ss, USB_DIR_OUT, 1024);

static struct usb_ss_ep_comp_descriptor dfu_bulk_comp = {
	.bLength =		sizeof(dfu_bulk_comp),
	.bDescriptorType =	USB_DT_SS_ENDPOINT_COMP,
	.bMaxBurst =		15,
};
#endif

static const char dfu_name[] = "Device Firmware Upgrade";

/*
 * static strings, in UTF-8
 *
 * dfu_generic configuration
 */
static struct usb_string strings_dfu_generic[] = {
	[0].s = dfu_name,
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dfu_generic = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dfu_generic,
};

static struct usb_gadget_strings *dfu_generic_strings[] = {
	&stringtab_dfu_generic,
	NULL,
};

/*
 * usb_function specific
 */
static struct usb_gadget_strings stringtab_dfu = {
	.language	= 0x0409,	/* en-us */
	/*
	 * .strings
	 *
	 * assigned during initialization,
	 * depends on number of flash entities
	 *
	 */
};

static struct usb_gadget_strings *dfu_strings[] = {
	&stringtab_dfu,
	NULL,
};

static void dfu_set_poll_timeout(struct dfu_status *dstat, unsigned int ms)
{
	/*
	 * The bwPollTimeout DFU_GETSTATUS request payload provides information
	 * about minimum time, in milliseconds, that the host should wait before
	 * sending a subsequent DFU_GETSTATUS request
	 *
	 * This permits the device to vary the delay depending on its need to
	 * erase or program the memory
	 *
	 */

	unsigned char *p = (unsigned char *)&ms;

	if (!ms || (ms & ~DFU_POLL_TIMEOUT_MASK)) {
		dstat->bwPollTimeout[0] = 0;
		dstat->bwPollTimeout[1] = 0;
		dstat->bwPollTimeout[2] = 0;

		return;
	}

	dstat->bwPollTimeout[0] = *p++;
	dstat->bwPollTimeout[1] = *p++;
	dstat->bwPollTimeout[2] = *p;
}

/*-------------------------------------------------------------------------*/

static void dnload_request_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_dfu *f_dfu = req->context;
	int ret;

	ret = dfu_write(dfu_get_entity(f_dfu->altsetting), req->buf,
			req->actual, f_dfu->blk_seq_num);
	if (ret) {
		/*
		 * -EBADMSG: the board refused the download at its end
		 * (dfu_board_manifest_check()), nothing was written; the
		 * host's next GETSTATUS reads errTARGET in dfuERROR and its
		 * CLRSTATUS returns to dfuIDLE, so the session goes on.
		 */
		f_dfu->dfu_status = ret == -EBADMSG ? DFU_STATUS_errTARGET :
						      DFU_STATUS_errUNKNOWN;
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
	}
}

/*
 * A zero-copy block (dfu_direct_buf()): the controller DMA'd the data stage
 * straight to or from its place. Give the request its EP0 buffer back, then
 * account for the block.
 */
static void dnload_direct_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_dfu *f_dfu = req->context;

	req->buf = f_dfu->ep0_buf;
	if (req->status ||
	    dfu_write_direct(dfu_get_entity(f_dfu->altsetting), req->actual,
			     f_dfu->blk_seq_num)) {
		f_dfu->dfu_status = DFU_STATUS_errUNKNOWN;
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
	}
}

static void upload_direct_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_dfu *f_dfu = req->context;

	req->buf = f_dfu->ep0_buf;
}

static void dnload_request_flush(struct usb_ep *ep, struct usb_request *req)
{
	struct f_dfu *f_dfu = req->context;
	dfu_set_defer_flush(dfu_get_entity(f_dfu->altsetting));
}

static inline int dfu_get_manifest_timeout(struct dfu_entity *dfu)
{
	return dfu->poll_timeout ? dfu->poll_timeout(dfu) :
		DFU_MANIFEST_POLL_TIMEOUT;
}

static int handle_getstatus(struct usb_request *req)
{
	struct dfu_status *dstat = (struct dfu_status *)req->buf;
	struct f_dfu *f_dfu = req->context;
	struct dfu_entity *dfu = dfu_get_entity(f_dfu->altsetting);

	dfu_set_poll_timeout(dstat, 0);

	switch (f_dfu->dfu_state) {
	case DFU_STATE_dfuDNLOAD_SYNC:
	case DFU_STATE_dfuDNBUSY:
		f_dfu->dfu_state = DFU_STATE_dfuDNLOAD_IDLE;
		break;
	case DFU_STATE_dfuMANIFEST_SYNC:
		f_dfu->dfu_state = DFU_STATE_dfuMANIFEST;
		break;
	case DFU_STATE_dfuMANIFEST:
		dfu_set_poll_timeout(dstat, dfu_get_manifest_timeout(dfu));
		break;
	default:
		break;
	}

	if (f_dfu->poll_timeout)
		if (!(f_dfu->blk_seq_num %
		      (dfu_get_buf_size() / DFU_USB_BUFSIZ)))
			dfu_set_poll_timeout(dstat, f_dfu->poll_timeout);

	/* send status response */
	dstat->bStatus = f_dfu->dfu_status;
	dstat->bState = f_dfu->dfu_state;
	dstat->iString = 0;

	return sizeof(struct dfu_status);
}

static int handle_getstate(struct usb_request *req)
{
	struct f_dfu *f_dfu = req->context;

	((u8 *)req->buf)[0] = f_dfu->dfu_state;
	return sizeof(u8);
}

/*
 * DFU has no endpoints of its own: its interface and functional descriptors
 * are the same at every speed, so the SuperSpeed list is the high-speed one.
 * Without it a SuperSpeed link (DWC3 on a USB 3 port) enumerated with no
 * configuration: "no configurations", "can't read configurations, error -22".
 */
static inline void to_dfu_mode(struct f_dfu *f_dfu)
{
	f_dfu->usb_function.strings = dfu_strings;
	f_dfu->usb_function.hs_descriptors = f_dfu->function;
	f_dfu->usb_function.ss_descriptors = f_dfu->function;
	f_dfu->usb_function.descriptors = f_dfu->function;
#if CONFIG_IS_ENABLED(DFU_BULK)
	/* With the bulk interface the lists differ by speed: its endpoints do */
	if (f_dfu->bulk_ready) {
		if (f_dfu->function_fs)
			f_dfu->usb_function.descriptors = f_dfu->function_fs;
		f_dfu->usb_function.hs_descriptors = f_dfu->function_hs;
		f_dfu->usb_function.ss_descriptors = f_dfu->function_ss;
	}
#endif
	f_dfu->dfu_state = DFU_STATE_dfuIDLE;
}

static inline void to_runtime_mode(struct f_dfu *f_dfu)
{
	f_dfu->usb_function.strings = NULL;
	f_dfu->usb_function.hs_descriptors = dfu_runtime_descs;
	f_dfu->usb_function.ss_descriptors = dfu_runtime_descs;
	f_dfu->usb_function.descriptors = dfu_runtime_descs;
}

/*
 * DFU_ABORT out of an UPLOAD or a DNLOAD: drop the entity's transaction too.
 * Leaving it open kept its block counter and read position, so the next
 * transfer, which starts again at block 0, failed dfu_read()'s sequence check
 * and the host saw a STALL.
 */
static void dfu_abort_transaction(struct f_dfu *f_dfu)
{
	struct dfu_entity *dfu = dfu_get_entity(f_dfu->altsetting);

#if CONFIG_IS_ENABLED(DFU_BULK)
	if (f_dfu->bulk_left) {
		f_dfu->bulk_left = 0;
		if (f_dfu->bulk_read)
			usb_ep_dequeue(f_dfu->bulk_in, f_dfu->bulk_in_req);
		else
			usb_ep_dequeue(f_dfu->bulk_out, f_dfu->bulk_out_req);
	}
#endif
	if (dfu)
		dfu_transaction_cleanup(dfu);
}

#if CONFIG_IS_ENABLED(DFU_BULK)
/* The next chunk of an armed bulk transfer, in place: 0, or an error */
static int dfu_bulk_queue(struct f_dfu *f_dfu)
{
	struct dfu_entity *dfu = dfu_get_entity(f_dfu->altsetting);
	bool read = f_dfu->bulk_read;
	struct usb_request *req = read ? f_dfu->bulk_in_req : f_dfu->bulk_out_req;
	int len = min_t(u64, f_dfu->bulk_left, DFU_BULK_CHUNK);

	req->buf = dfu ? dfu_direct_buf(dfu, len, read) : NULL;
	/* an upload never reads past the alternate */
	if (!req->buf || (read && len > dfu->r_left))
		return -EINVAL;
	req->length = len;

	return usb_ep_queue(read ? f_dfu->bulk_in : f_dfu->bulk_out, req, 0);
}

static void dfu_bulk_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct f_dfu *f_dfu = req->context;
	struct dfu_entity *dfu = dfu_get_entity(f_dfu->altsetting);
	int seq = dfu->i_blk_seq_num;
	int ret;

	if (!f_dfu->bulk_left)		/* dequeued by an ABORT */
		return;
	ret = req->status || req->actual != req->length;
	if (!ret)
		ret = f_dfu->bulk_read ? dfu_read_direct(dfu, req->actual, seq) < 0 :
					 dfu_write_direct(dfu, req->actual, seq);
	if (!ret) {
		f_dfu->bulk_left -= req->actual;
#ifdef CONFIG_DFU_TIMEOUT
		/*
		 * Bulk progress is host activity. Not in xPL: an SPL window moves
		 * in seconds, well inside an idle timeout, and the arm that
		 * started it restarted the timeout already.
		 */
		if (!IS_ENABLED(CONFIG_XPL_BUILD))
			dfu_host_active();
#endif
		if (f_dfu->bulk_left) {
			ret = dfu_bulk_queue(f_dfu);
		} else {
			/* the host numbers the DFU blocks after the bulk part from 0 */
			dfu->i_blk_seq_num = 0;
			return;
		}
	}
	if (ret) {
		f_dfu->bulk_left = 0;
		f_dfu->dfu_status = DFU_STATUS_errUNKNOWN;
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
	}
}

/* DFU_BULK_DNLOAD / DFU_BULK_UPLOAD in dfuIDLE: arm the bulk part */
static int dfu_bulk_arm(struct f_dfu *f_dfu, const struct usb_ctrlrequest *ctrl)
{
	struct dfu_entity *dfu = dfu_get_entity(f_dfu->altsetting);
	bool read = ctrl->bRequest == DFU_BULK_REQ_UPLOAD;
	u64 total = (u64)le16_to_cpu(ctrl->wValue) * DFU_BULK_UNIT;
	u64 size;

	if ((!read && ctrl->bRequest != DFU_BULK_REQ_DNLOAD) || ctrl->wLength ||
	    !total || f_dfu->dfu_state != DFU_STATE_dfuIDLE || !dfu)
		return RET_STALL;
	/*
	 * An arm past the alternate's end is refused here, before anything
	 * moves; otherwise it would run to the end and fail there, in dfuERROR.
	 */
	if (dfu->get_medium_size && !dfu->get_medium_size(dfu, &size) &&
	    total > size)
		return RET_STALL;

	/*
	 * An arm starts an image: whatever an earlier transfer left (a client
	 * killed mid-transfer, an alternate switched under it) is dropped here,
	 * or the new data lands at the old offset -- the bulk blocks carry no
	 * sequence number to catch it.
	 */
	dfu_abort_transaction(f_dfu);
	f_dfu->bulk_read = read;
	f_dfu->bulk_left = total;
	if (dfu_bulk_queue(f_dfu)) {
		f_dfu->bulk_left = 0;
		return RET_STALL;
	}
	f_dfu->dfu_state = read ? DFU_STATE_dfuUPLOAD_IDLE :
				  DFU_STATE_dfuDNLOAD_IDLE;

	return RET_ZLP;
}
#endif

/*
 * @len is this request's wLength, bounded by DFU_USB_BUFSIZ.  Not req->length:
 * that is whatever the previous EP0 transfer left (a 6-byte GETSTATUS reply,
 * say), which equals the block size only by accident, and a block read at
 * another size puts the host and dfu_read() a block apart ("Wrong sequence
 * number!").
 */
static int handle_upload(struct usb_request *req, u16 len)
{
	struct f_dfu *f_dfu = req->context;
	struct dfu_entity *dfu = dfu_get_entity(f_dfu->altsetting);
	void *direct = dfu_direct_buf(dfu, len, true);
	int n;

	if (!direct)
		return dfu_read(dfu, req->buf, len, f_dfu->blk_seq_num);

	n = dfu_read_direct(dfu, len, f_dfu->blk_seq_num);
	if (n < 0)
		return n;
	f_dfu->ep0_buf = req->buf;
	req->buf = direct;
	req->complete = upload_direct_complete;

	return n;
}

static int handle_dnload(struct usb_gadget *gadget, u16 len)
{
	struct usb_composite_dev *cdev = get_gadget_data(gadget);
	struct usb_request *req = cdev->req;
	struct f_dfu *f_dfu = req->context;
	void *direct;

	if (len == 0)
		f_dfu->dfu_state = DFU_STATE_dfuMANIFEST_SYNC;

	req->complete = dnload_request_complete;

	direct = len ? dfu_direct_buf(dfu_get_entity(f_dfu->altsetting), len,
				      false) : NULL;
	if (direct) {
		f_dfu->ep0_buf = req->buf;
		req->buf = direct;
		req->complete = dnload_direct_complete;
	}

	return len;
}

/*-------------------------------------------------------------------------*/
/* DFU state machine  */
static int state_app_idle(struct f_dfu *f_dfu,
			  const struct usb_ctrlrequest *ctrl,
			  struct usb_gadget *gadget,
			  struct usb_request *req)
{
	int value = 0;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	case USB_REQ_DFU_DETACH:
		f_dfu->dfu_state = DFU_STATE_appDETACH;
		to_dfu_mode(f_dfu);
		value = RET_ZLP;
		break;
	default:
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_app_detach(struct f_dfu *f_dfu,
			    const struct usb_ctrlrequest *ctrl,
			    struct usb_gadget *gadget,
			    struct usb_request *req)
{
	int value = 0;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_appIDLE;
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_dfu_idle(struct f_dfu *f_dfu,
			  const struct usb_ctrlrequest *ctrl,
			  struct usb_gadget *gadget,
			  struct usb_request *req)
{
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 len = le16_to_cpu(ctrl->wLength);
	int value = 0;

	len = len > DFU_USB_BUFSIZ ? DFU_USB_BUFSIZ : len;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_DNLOAD:
		if (!(ctrl->bRequestType & USB_DIR_IN)) {
			if (len == 0) {
				f_dfu->dfu_state = DFU_STATE_dfuERROR;
				value = RET_STALL;
				break;
			}
			f_dfu->dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			f_dfu->blk_seq_num = w_value;
			value = handle_dnload(gadget, len);
		}
		break;
	case USB_REQ_DFU_UPLOAD:
		if (ctrl->bRequestType & USB_DIR_IN) {
			f_dfu->dfu_state = DFU_STATE_dfuUPLOAD_IDLE;
			f_dfu->blk_seq_num = 0;
			value = handle_upload(req, len);
			if (value >= 0 && value < len)
				f_dfu->dfu_state = DFU_STATE_dfuIDLE;
		}
		break;
	case USB_REQ_DFU_ABORT:
		/* no zlp? */
		value = RET_ZLP;
		break;
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	case USB_REQ_DFU_DETACH:
		/*
		 * Proprietary extension: 'detach' from idle mode and
		 * get back to runtime mode in case of USB Reset.  As
		 * much as I dislike this, we just can't use every USB
		 * bus reset to switch back to runtime mode, since at
		 * least the Linux USB stack likes to send a number of
		 * resets in a row :(
		 */
		f_dfu->dfu_state =
			DFU_STATE_dfuMANIFEST_WAIT_RST;
		to_runtime_mode(f_dfu);
		f_dfu->dfu_state = DFU_STATE_appIDLE;

		g_dnl_trigger_detach();
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_dfu_dnload_sync(struct f_dfu *f_dfu,
				 const struct usb_ctrlrequest *ctrl,
				 struct usb_gadget *gadget,
				 struct usb_request *req)
{
	int value = 0;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_dfu_dnbusy(struct f_dfu *f_dfu,
			    const struct usb_ctrlrequest *ctrl,
			    struct usb_gadget *gadget,
			    struct usb_request *req)
{
	int value = 0;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_dfu_dnload_idle(struct f_dfu *f_dfu,
				 const struct usb_ctrlrequest *ctrl,
				 struct usb_gadget *gadget,
				 struct usb_request *req)
{
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 len = le16_to_cpu(ctrl->wLength);
	int value = 0;

	len = len > DFU_USB_BUFSIZ ? DFU_USB_BUFSIZ : len;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_DNLOAD:
		if (!(ctrl->bRequestType & USB_DIR_IN)) {
			f_dfu->dfu_state = DFU_STATE_dfuDNLOAD_SYNC;
			f_dfu->blk_seq_num = w_value;
			value = handle_dnload(gadget, len);
		}
		break;
	case USB_REQ_DFU_ABORT:
		dfu_abort_transaction(f_dfu);
		f_dfu->dfu_state = DFU_STATE_dfuIDLE;
		value = RET_ZLP;
		break;
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_dfu_manifest_sync(struct f_dfu *f_dfu,
				   const struct usb_ctrlrequest *ctrl,
				   struct usb_gadget *gadget,
				   struct usb_request *req)
{
	int value = 0;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_GETSTATUS:
		/* We're MainfestationTolerant */
		f_dfu->dfu_state = DFU_STATE_dfuMANIFEST;
		value = handle_getstatus(req);
		f_dfu->blk_seq_num = 0;
		req->complete = dnload_request_flush;
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_dfu_manifest(struct f_dfu *f_dfu,
			      const struct usb_ctrlrequest *ctrl,
			      struct usb_gadget *gadget,
			      struct usb_request *req)
{
	int value = 0;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_GETSTATUS:
		/* We're MainfestationTolerant */
		f_dfu->dfu_state = DFU_STATE_dfuIDLE;
		value = handle_getstatus(req);
		f_dfu->blk_seq_num = 0;
		puts("DOWNLOAD ... OK\nCtrl+C to exit ...\n");
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}
	return value;
}

static int state_dfu_upload_idle(struct f_dfu *f_dfu,
				 const struct usb_ctrlrequest *ctrl,
				 struct usb_gadget *gadget,
				 struct usb_request *req)
{
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 len = le16_to_cpu(ctrl->wLength);
	int value = 0;

	len = len > DFU_USB_BUFSIZ ? DFU_USB_BUFSIZ : len;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_UPLOAD:
		if (ctrl->bRequestType & USB_DIR_IN) {
			/* state transition if less data then requested */
			f_dfu->blk_seq_num = w_value;
			value = handle_upload(req, len);
			if (value >= 0 && value < len)
				f_dfu->dfu_state = DFU_STATE_dfuIDLE;
		}
		break;
	case USB_REQ_DFU_ABORT:
		dfu_abort_transaction(f_dfu);
		f_dfu->dfu_state = DFU_STATE_dfuIDLE;
		/* no zlp? */
		value = RET_ZLP;
		break;
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}

	return value;
}

static int state_dfu_error(struct f_dfu *f_dfu,
				 const struct usb_ctrlrequest *ctrl,
				 struct usb_gadget *gadget,
				 struct usb_request *req)
{
	int value = 0;

	switch (ctrl->bRequest) {
	case USB_REQ_DFU_GETSTATUS:
		value = handle_getstatus(req);
		break;
	case USB_REQ_DFU_GETSTATE:
		value = handle_getstate(req);
		break;
	case USB_REQ_DFU_CLRSTATUS:
		f_dfu->dfu_state = DFU_STATE_dfuIDLE;
		f_dfu->dfu_status = DFU_STATUS_OK;
		/* no zlp? */
		value = RET_ZLP;
		break;
	default:
		f_dfu->dfu_state = DFU_STATE_dfuERROR;
		value = RET_STALL;
		break;
	}

	return value;
}

static dfu_state_fn dfu_state[] = {
	state_app_idle,          /* DFU_STATE_appIDLE */
	state_app_detach,        /* DFU_STATE_appDETACH */
	state_dfu_idle,          /* DFU_STATE_dfuIDLE */
	state_dfu_dnload_sync,   /* DFU_STATE_dfuDNLOAD_SYNC */
	state_dfu_dnbusy,        /* DFU_STATE_dfuDNBUSY */
	state_dfu_dnload_idle,   /* DFU_STATE_dfuDNLOAD_IDLE */
	state_dfu_manifest_sync, /* DFU_STATE_dfuMANIFEST_SYNC */
	state_dfu_manifest,	 /* DFU_STATE_dfuMANIFEST */
	NULL,                    /* DFU_STATE_dfuMANIFEST_WAIT_RST */
	state_dfu_upload_idle,   /* DFU_STATE_dfuUPLOAD_IDLE */
	state_dfu_error          /* DFU_STATE_dfuERROR */
};

static int
dfu_handle(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct usb_gadget *gadget = f->config->cdev->gadget;
	struct usb_request *req = f->config->cdev->req;
	struct f_dfu *f_dfu = f->config->cdev->req->context;
	u16 len = le16_to_cpu(ctrl->wLength);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	int value = 0;
	u8 req_type = ctrl->bRequestType & USB_TYPE_MASK;

	len = len > DFU_USB_BUFSIZ ? DFU_USB_BUFSIZ : len;

	debug("w_value: 0x%x len: 0x%x\n", w_value, len);
	debug("req_type: 0x%x ctrl->bRequest: 0x%x f_dfu->dfu_state: 0x%x\n",
	       req_type, ctrl->bRequest, f_dfu->dfu_state);

#ifdef CONFIG_DFU_TIMEOUT
	/*
	 * A host is here: the timeout becomes the idle timeout, restarted by
	 * every request (0 unless the caller set one: no timeout at all).
	 */
	dfu_host_active();
#endif

	if (req_type == USB_TYPE_STANDARD) {
		if (ctrl->bRequest == USB_REQ_GET_DESCRIPTOR &&
		    (w_value >> 8) == DFU_DT_FUNC) {
			value = min(len, (u16) sizeof(dfu_func));
			memcpy(req->buf, &dfu_func, value);
		}
#if CONFIG_IS_ENABLED(DFU_BULK)
	} else if (req_type == USB_TYPE_VENDOR) {
		value = f_dfu->bulk_ready ? dfu_bulk_arm(f_dfu, ctrl) : RET_STALL;
	} else if (f_dfu->bulk_left &&
		   (ctrl->bRequest == USB_REQ_DFU_DNLOAD ||
		    ctrl->bRequest == USB_REQ_DFU_UPLOAD)) {
		/*
		 * A DFU block before the armed bulk part is all in: refused, the
		 * bulk part goes on (an arm or ABORT starts over cleanly).
		 */
		value = RET_STALL;
#endif
	} else /* DFU specific request */
		value = dfu_state[f_dfu->dfu_state] (f_dfu, ctrl, gadget, req);

	if (value >= 0) {
		req->length = value > DFU_USB_BUFSIZ ? DFU_USB_BUFSIZ : value;
		req->zero = value < len;
		value = usb_ep_queue(gadget->ep0, req, 0);
		if (value < 0) {
			debug("ep_queue --> %d\n", value);
			req->status = 0;
		}
	}

	return value;
}

/*-------------------------------------------------------------------------*/

static int
dfu_prepare_strings(struct f_dfu *f_dfu, int n)
{
	struct dfu_entity *de = NULL;
	int i = 0;

	f_dfu->strings = calloc(sizeof(struct usb_string), n + 1);
	if (!f_dfu->strings)
		return -ENOMEM;

	for (i = 0; i < n; ++i) {
		de = dfu_get_entity(i);
		f_dfu->strings[i].s = de->name;
	}

	f_dfu->strings[i].id = 0;
	f_dfu->strings[i].s = NULL;

	return 0;
}

static int dfu_prepare_function(struct f_dfu *f_dfu, int n)
{
	struct usb_interface_descriptor *d;
	int i = 0;

	f_dfu->function = calloc(sizeof(struct usb_descriptor_header *), n + 2);
	if (!f_dfu->function)
		goto enomem;

	for (i = 0; i < n; ++i) {
		d = calloc(sizeof(*d), 1);
		if (!d)
			goto enomem;

		d->bLength =		sizeof(*d);
		d->bDescriptorType =	USB_DT_INTERFACE;
		d->bAlternateSetting =	i;
		d->bNumEndpoints =	0;
		d->bInterfaceClass =	USB_CLASS_APP_SPEC;
		d->bInterfaceSubClass =	1;
		d->bInterfaceProtocol =	2;

		f_dfu->function[i] = (struct usb_descriptor_header *)d;
	}

	/* add DFU Functional Descriptor */
	f_dfu->function[i] = calloc(sizeof(dfu_func), 1);
	if (!f_dfu->function[i])
		goto enomem;
	memcpy(f_dfu->function[i], &dfu_func, sizeof(dfu_func));

	i++;
	f_dfu->function[i] = NULL;

	return 0;

enomem:
	while (i) {
		free(f_dfu->function[--i]);
		f_dfu->function[i] = NULL;
	}
	free(f_dfu->function);

	return -ENOMEM;
}

#if CONFIG_IS_ENABLED(DFU_BULK)
/* One speed's list: the DFU alternates and functional descriptor, then the bulk interface */
static struct usb_descriptor_header **
dfu_bulk_descs(struct f_dfu *f_dfu, int n, void *in, void *out, bool ss)
{
	struct usb_descriptor_header **d = calloc(sizeof(*d), n + 8);
	int i = n + 1;

	if (!d)
		return NULL;
	memcpy(d, f_dfu->function, sizeof(*d) * i);
	d[i++] = (struct usb_descriptor_header *)&dfu_bulk_intf;
	d[i++] = in;
	if (ss)
		d[i++] = (struct usb_descriptor_header *)&dfu_bulk_comp;
	d[i++] = out;
	if (ss)
		d[i++] = (struct usb_descriptor_header *)&dfu_bulk_comp;

	return d;
}

#if IS_ENABLED(CONFIG_XPL_BUILD)
/*
 * xPL: the first unclaimed endpoint named "ep<N>in" / "ep<N>out" (DWC3) or
 * "ep<N>in-bulk" / "ep<N>out-bulk" (DWC2), without usb_ep_autoconfig()'s
 * general matcher and its size. Any other suffix ("-int") is not bulk.
 */
static struct usb_ep *dfu_bulk_ep(struct usb_gadget *g,
				  struct usb_endpoint_descriptor *d)
{
	bool in = d->bEndpointAddress & USB_DIR_IN;
	const char *dir = in ? "in" : "out";
	int dl = in ? 2 : 3;
	struct usb_ep *ep;

	list_for_each_entry(ep, &g->ep_list, ep_list) {
		const char *s = ep->name + 2;
		int num = 0;

		while (isdigit(*s))
			num = num * 10 + *s++ - '0';
		if (ep->driver_data || s == ep->name + 2 || strncmp(s, dir, dl) ||
		    (s[dl] && strcmp(s + dl, "-bulk")))
			continue;
		ep->driver_data = ep;
		d->bEndpointAddress |= num;
		return ep;
	}

	return NULL;
}
#else
#define dfu_bulk_ep usb_ep_autoconfig
#endif

/*
 * The bulk interface and its endpoints. A controller with no two bulk
 * endpoints to spare leaves DFU as it was (control transfers only).
 */
static int dfu_bulk_bind(struct usb_configuration *c, struct f_dfu *f_dfu, int n)
{
	struct usb_gadget *g = c->cdev->gadget;
	struct usb_ep *in, *out;
	int id;

	/*
	 * usb_ep_autoconfig() rewrites a bulk template's wMaxPacketSize to the
	 * full-speed limit, so proper hands it the full-speed descriptors; the
	 * xPL picker writes only the address, so xPL uses the high-speed ones
	 * and builds no full-speed descriptors at all.
	 */
	bool xpl = IS_ENABLED(CONFIG_XPL_BUILD);
	struct usb_endpoint_descriptor *ti = xpl ? &dfu_bulk_in_hs : &dfu_bulk_in_fs;
	struct usb_endpoint_descriptor *to = xpl ? &dfu_bulk_out_hs : &dfu_bulk_out_fs;

	in = dfu_bulk_ep(g, ti);
	out = dfu_bulk_ep(g, to);
	if (!in || !out)
		return 0;
	id = usb_interface_id(c, &f_dfu->usb_function);
	if (id < 0)
		return id;
	dfu_bulk_intf.bInterfaceNumber = id;
	f_dfu->bulk_intf = id;
	dfu_bulk_in_hs.bEndpointAddress = ti->bEndpointAddress;
	dfu_bulk_in_ss.bEndpointAddress = ti->bEndpointAddress;
	dfu_bulk_out_hs.bEndpointAddress = to->bEndpointAddress;
	dfu_bulk_out_ss.bEndpointAddress = to->bEndpointAddress;
	in->maxburst = dfu_bulk_comp.bMaxBurst + 1;
	out->maxburst = dfu_bulk_comp.bMaxBurst + 1;
	f_dfu->bulk_in = in;
	f_dfu->bulk_out = out;

	f_dfu->bulk_in_req = usb_ep_alloc_request(in, 0);
	f_dfu->bulk_out_req = usb_ep_alloc_request(out, 0);
	/* xPL sessions run at high or SuperSpeed: no full-speed bulk list there */
	if (!IS_ENABLED(CONFIG_XPL_BUILD)) {
		f_dfu->function_fs = dfu_bulk_descs(f_dfu, n, &dfu_bulk_in_fs,
						    &dfu_bulk_out_fs, false);
		if (!f_dfu->function_fs)
			return -ENOMEM;
	}
	f_dfu->function_hs = dfu_bulk_descs(f_dfu, n, &dfu_bulk_in_hs,
					    &dfu_bulk_out_hs, false);
	f_dfu->function_ss = dfu_bulk_descs(f_dfu, n, &dfu_bulk_in_ss,
					    &dfu_bulk_out_ss, true);
	if (!f_dfu->bulk_in_req || !f_dfu->bulk_out_req ||
	    !f_dfu->function_hs || !f_dfu->function_ss)
		return -ENOMEM;
	f_dfu->bulk_in_req->complete = dfu_bulk_complete;
	f_dfu->bulk_out_req->complete = dfu_bulk_complete;
	f_dfu->bulk_in_req->context = f_dfu;
	f_dfu->bulk_out_req->context = f_dfu;
	f_dfu->bulk_ready = true;

	return 0;
}

/* SET_INTERFACE on the bulk interface (and SET_CONFIGURATION): its endpoints at this speed */
static int dfu_bulk_set_alt(struct f_dfu *f_dfu, struct usb_gadget *g)
{
	bool ss = g->speed >= USB_SPEED_SUPER, hs = g->speed == USB_SPEED_HIGH;
	int ret;

	/* xPL lists no full-speed bulk interface: nothing to enable */
	if (IS_ENABLED(CONFIG_XPL_BUILD) && !ss && !hs)
		return 0;
	usb_ep_disable(f_dfu->bulk_in);
	usb_ep_disable(f_dfu->bulk_out);
	ret = usb_ep_enable(f_dfu->bulk_in, ss ? &dfu_bulk_in_ss :
				 hs ? &dfu_bulk_in_hs : &dfu_bulk_in_fs);
	if (!ret)
		ret = usb_ep_enable(f_dfu->bulk_out, ss ? &dfu_bulk_out_ss :
					 hs ? &dfu_bulk_out_hs : &dfu_bulk_out_fs);

	return ret;
}
#endif

static int dfu_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_dfu *f_dfu = func_to_dfu(f);
	const char *s;
	int alt_num = dfu_get_alt_number();
	int rv, id, i;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;
	dfu_intf_runtime.bInterfaceNumber = id;

	f_dfu->dfu_state = DFU_STATE_appIDLE;
	f_dfu->dfu_status = DFU_STATUS_OK;

	rv = dfu_prepare_function(f_dfu, alt_num);
	if (rv)
		goto error;
#if CONFIG_IS_ENABLED(DFU_BULK)
	rv = dfu_bulk_bind(c, f_dfu, alt_num);
	if (rv)
		goto error;
#endif

	rv = dfu_prepare_strings(f_dfu, alt_num);
	if (rv)
		goto error;
	for (i = 0; i < alt_num; i++) {
		id = usb_string_id(cdev);
		if (id < 0)
			return id;
		f_dfu->strings[i].id = id;
		((struct usb_interface_descriptor *)f_dfu->function[i])
			->iInterface = id;
	}

	to_dfu_mode(f_dfu);

	stringtab_dfu.strings = f_dfu->strings;

	cdev->req->context = f_dfu;

	/*
	 * No environment in a bare xPL phase (dfu_bare_phase(), see
	 * dfu_env_get() in drivers/dfu/dfu.c): the gadget keeps its
	 * compiled-in serial.  With an environment, serial# is read as before.
	 */
	s = dfu_bare_phase() ? NULL : env_get("serial#");
	if (s)
		g_dnl_set_serialnumber((char *)s);

error:
	return rv;
}

static void dfu_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_dfu *f_dfu = func_to_dfu(f);
	int alt_num = dfu_get_alt_number();
	int i;

	if (f_dfu->strings) {
		i = alt_num;
		while (i)
			f_dfu->strings[--i].s = NULL;

		free(f_dfu->strings);
	}

	if (f_dfu->function) {
		i = alt_num;
		i++; /* free DFU Functional Descriptor */
		while (i) {
			free(f_dfu->function[--i]);
			f_dfu->function[i] = NULL;
		}
		free(f_dfu->function);
	}
#if CONFIG_IS_ENABLED(DFU_BULK) && !IS_ENABLED(CONFIG_XPL_BUILD)
	/*
	 * The per-speed lists only point at descriptors freed above or static.
	 * Not in xPL: its heap is left behind whole when the boot goes on.
	 */
	free(f_dfu->function_fs);
	free(f_dfu->function_hs);
	free(f_dfu->function_ss);
	if (f_dfu->bulk_in_req)
		usb_ep_free_request(f_dfu->bulk_in, f_dfu->bulk_in_req);
	if (f_dfu->bulk_out_req)
		usb_ep_free_request(f_dfu->bulk_out, f_dfu->bulk_out_req);
#endif

	free(f_dfu);
}

static int dfu_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_dfu *f_dfu = func_to_dfu(f);

	debug("%s: intf:%d alt:%d\n", __func__, intf, alt);

#if CONFIG_IS_ENABLED(DFU_BULK)
	if (f_dfu->bulk_ready && intf == f_dfu->bulk_intf)
		return dfu_bulk_set_alt(f_dfu, f->config->cdev->gadget);
#endif
	/* the alternate being left keeps nothing in flight (an armed bulk part) */
	dfu_abort_transaction(f_dfu);
	f_dfu->altsetting = alt;
	f_dfu->dfu_state = DFU_STATE_dfuIDLE;
	f_dfu->dfu_status = DFU_STATUS_OK;

	return 0;
}

static int __dfu_get_alt(struct usb_function *f, unsigned intf)
{
	struct f_dfu *f_dfu = func_to_dfu(f);

	return f_dfu->altsetting;
}

/* TODO: is this really what we need here? */
static void dfu_disable(struct usb_function *f)
{
	struct f_dfu *f_dfu = func_to_dfu(f);
	if (f_dfu->config == 0)
		return;

	debug("%s: reset config\n", __func__);

	f_dfu->config = 0;
}

static int dfu_bind_config(struct usb_configuration *c)
{
	struct f_dfu *f_dfu;
	int status;

	f_dfu = calloc(sizeof(*f_dfu), 1);
	if (!f_dfu)
		return -ENOMEM;
	f_dfu->usb_function.name = "dfu";
	f_dfu->usb_function.hs_descriptors = dfu_runtime_descs;
	f_dfu->usb_function.ss_descriptors = dfu_runtime_descs;
	f_dfu->usb_function.descriptors = dfu_runtime_descs;
	f_dfu->usb_function.bind = dfu_bind;
	f_dfu->usb_function.unbind = dfu_unbind;
	f_dfu->usb_function.set_alt = dfu_set_alt;
	f_dfu->usb_function.get_alt = __dfu_get_alt;
	f_dfu->usb_function.disable = dfu_disable;
	f_dfu->usb_function.strings = dfu_generic_strings;
	f_dfu->usb_function.setup = dfu_handle;
	f_dfu->poll_timeout = DFU_DEFAULT_POLL_TIMEOUT;

	status = usb_add_function(c, &f_dfu->usb_function);
	if (status)
		free(f_dfu);

	return status;
}

int dfu_add(struct usb_configuration *c)
{
	int id;

	id = usb_string_id(c->cdev);
	if (id < 0)
		return id;
	strings_dfu_generic[0].id = id;
	dfu_intf_runtime.iInterface = id;

	debug("%s: cdev: 0x%p gadget:0x%p gadget->ep0: 0x%p\n", __func__,
	       c->cdev, c->cdev->gadget, c->cdev->gadget->ep0);

	return dfu_bind_config(c);
}

DECLARE_GADGET_BIND_CALLBACK(usb_dnl_dfu, dfu_add);
