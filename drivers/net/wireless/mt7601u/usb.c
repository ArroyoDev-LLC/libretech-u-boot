// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * U-Boot background-scan port of the Linux mt7601u USB layer
 * (drivers/net/wireless/mediatek/mt7601u/usb.c and core.c): device-id table,
 * vendor requests, register access, endpoint assignment, register polling.
 * URBs are replaced by synchronous usb_control_msg()/usb_bulk_msg(); the two
 * bulk-IN endpoints are driven by the xHCI async-TD API during a scan.
 */

#include <dm.h>
#include <log.h>
#include <time.h>
#include <usb.h>
#include <usb/xhci.h>
#include <asm/unaligned.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>
#include <linux/delay.h>
#include <linux/errno.h>

#include "mt7601u.h"
#include "mcu.h"

static const struct {
	u16 vid;
	u16 pid;
} mt7601u_device_table[] = {
	{ 0x0b05, 0x17d3 },
	{ 0x0e8d, 0x760a },
	{ 0x0e8d, 0x760b },
	{ 0x13d3, 0x3431 },
	{ 0x13d3, 0x3434 },
	{ 0x148f, 0x7601 },
	{ 0x148f, 0x760a },
	{ 0x148f, 0x760b },
	{ 0x148f, 0x760c },
	{ 0x148f, 0x760d },
	{ 0x2001, 0x3d04 },
	{ 0x2717, 0x4106 },
	{ 0x2955, 0x0001 },
	{ 0x2955, 0x1001 },
	{ 0x2955, 0x1003 },
	{ 0x2a5f, 0x1000 },
	{ 0x7392, 0x7710 },
};

static void mt7601u_xfer_begin(struct mt7601u_dev *dev)
{
	if (!dev->in_cyclic)
		dev->busy++;
}

static void mt7601u_xfer_end(struct mt7601u_dev *dev)
{
	if (!dev->in_cyclic)
		dev->busy--;
}

int mt7601u_vendor_request(struct mt7601u_dev *dev, const u8 req,
			   const u8 direction, const u16 val, const u16 offset,
			   void *buf, const size_t buflen)
{
	struct usb_device *udev = dev->udev;
	const u8 req_type = direction | USB_TYPE_VENDOR | USB_RECIP_DEVICE;
	const unsigned int pipe = (direction == USB_DIR_IN) ?
		usb_rcvctrlpipe(udev, 0) : usb_sndctrlpipe(udev, 0);
	/* inside the cyclic: one try, no 5 ms back-off (stay in budget) */
	int tries = dev->in_cyclic ? 1 : MT_VEND_REQ_MAX_RETRY;
	int i, ret = -EIO;

	mt7601u_xfer_begin(dev);
	/*
	 * xhci_ctrl_tx ignores MT_VEND_REQ_TOUT_MS and waits XHCI_TIMEOUT.
	 * A scan tick's register access goes through here, so one miss is
	 * another 5000 ms stall. Cap this call at 1 ms and clear the
	 * ceiling before returning. Outside the cyclic the wait stays
	 * 5000 ms.
	 */
	if (dev->in_cyclic)
		xhci_set_event_wait_ms(udev, 1);
	for (i = 0; i < tries; i++) {
		ret = usb_control_msg(udev, pipe, req, req_type, val, offset,
				      buf, buflen, MT_VEND_REQ_TOUT_MS);
		if (ret >= 0 || ret == -ENODEV)
			break;
		if (i + 1 < tries)
			mdelay(5);
	}
	if (dev->in_cyclic)
		xhci_set_event_wait_ms(udev, 0);
	mt7601u_xfer_end(dev);

	if (ret < 0) {
		dev->reg_failed = true;
		mt_err(dev, "vendor request req:%02x off:%04x failed:%d\n",
		       req, offset, ret);
	}

	return ret;
}

void mt7601u_vendor_reset(struct mt7601u_dev *dev)
{
	mt7601u_vendor_request(dev, MT_VEND_DEV_MODE, USB_DIR_OUT,
			       MT_VEND_DEV_MODE_RESET, 0, NULL, 0);
}

u32 mt7601u_rr(struct mt7601u_dev *dev, u32 offset)
{
	int ret;
	u32 val = ~0;

	if (offset > 0xffff)
		printf("mt7601u: read high off:%08x\n", offset);

	ret = mt7601u_vendor_request(dev, MT_VEND_MULTI_READ, USB_DIR_IN,
				     0, offset, dev->vend_buf, MT_VEND_BUF);
	if (ret == MT_VEND_BUF) {
		val = get_unaligned_le32(dev->vend_buf);
	} else if (ret >= 0) {
		dev->reg_failed = true;
		mt_err(dev, "wrong size read:%d off:%08x\n", ret, offset);
	}

	return val;
}

int mt7601u_vendor_single_wr(struct mt7601u_dev *dev, const u8 req,
			     const u16 offset, const u32 val)
{
	int ret = mt7601u_vendor_request(dev, req, USB_DIR_OUT,
					 val & 0xffff, offset, NULL, 0);
	if (!ret)
		ret = mt7601u_vendor_request(dev, req, USB_DIR_OUT,
					     val >> 16, offset + 2, NULL, 0);
	return ret;
}

void mt7601u_wr(struct mt7601u_dev *dev, u32 offset, u32 val)
{
	if (offset > 0xffff)
		printf("mt7601u: write high off:%08x\n", offset);

	mt7601u_vendor_single_wr(dev, MT_VEND_WRITE, offset, val);
}

u32 mt7601u_rmw(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val)
{
	val |= mt7601u_rr(dev, offset) & ~mask;
	mt7601u_vendor_single_wr(dev, MT_VEND_WRITE, offset, val);

	return val;
}

u32 mt7601u_rmc(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val)
{
	u32 reg;

	reg = mt7601u_rr(dev, offset);
	val |= reg & ~mask;
	if (reg != val)
		mt7601u_vendor_single_wr(dev, MT_VEND_WRITE, offset, val);

	return val;
}

/*
 * One synchronous bulk transfer. Used for the MCU command OUT and, before
 * the scan arms the async TDs, the init-time MCU-response read. The scan's
 * RX and command-response reads use the xHCI async TDs instead.
 */
int mt7601u_bulk(struct mt7601u_dev *dev, bool in, int ep_idx, void *buf,
		 int len, int *actual, int timeout_ms)
{
	struct usb_device *udev = dev->udev;
	unsigned int pipe;
	int ret;

	if (in)
		pipe = usb_rcvbulkpipe(udev, dev->in_eps[ep_idx] & 0x0f);
	else
		pipe = usb_sndbulkpipe(udev, dev->out_eps[ep_idx] & 0x0f);

	mt7601u_xfer_begin(dev);
	*actual = 0;
	/*
	 * xhci_bulk_tx ignores timeout_ms and waits XHCI_TIMEOUT. Inside the
	 * scan tick that is one 5000 ms stall. Cap this call at 1 ms; a miss
	 * is -ETIMEDOUT and the hop resends on the next tick. Outside the
	 * cyclic the ceiling stays 0, so the wait stays 5000 ms.
	 */
	if (dev->in_cyclic)
		xhci_set_event_wait_ms(udev, 1);
	ret = usb_bulk_msg(udev, pipe, buf, len, actual, timeout_ms);
	if (dev->in_cyclic)
		xhci_set_event_wait_ms(udev, 0);
	if (ret)
		*actual = udev->act_len;
	mt7601u_xfer_end(dev);

	if (!ret)
		return 0;
	if (udev->status & USB_ST_NAK_REC)
		return -ETIMEDOUT;
	return -EIO;
}

/* core.c */
int mt7601u_wait_asic_ready(struct mt7601u_dev *dev)
{
	int i = 100;
	u32 val;

	do {
		val = mt7601u_rr(dev, MT_MAC_CSR0);
		if (val && ~val)
			return 0;

		udelay(10);
	} while (i--);

	return -EIO;
}

/*
 * Inside the cyclic the poll is cut to two reads: a register that is not
 * ready by then fails the access (reg_failed) and the hop stage is retried
 * on the next tick instead of spinning here.
 */
bool mt76_poll(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val,
	       int timeout)
{
	u32 cur;

	timeout = dev->in_cyclic ? 1 : timeout / 10;
	do {
		cur = mt7601u_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		udelay(10);
	} while (timeout-- > 0);

	dev->reg_failed = true;
	mt_err(dev, "Time out with reg %08x\n", offset);

	return false;
}

bool mt76_poll_msec(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val,
		    int timeout)
{
	u32 cur;

	timeout = dev->in_cyclic ? 1 : timeout / 10;
	do {
		cur = mt7601u_rr(dev, offset) & mask;
		if (cur == val)
			return true;

		mdelay(10);
	} while (timeout-- > 0);

	dev->reg_failed = true;
	mt_err(dev, "Time out with reg %08x\n", offset);

	return false;
}

static bool mt7601u_id_match(struct usb_device *udev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt7601u_device_table); i++)
		if (udev->descriptor.idVendor == mt7601u_device_table[i].vid &&
		    udev->descriptor.idProduct == mt7601u_device_table[i].pid)
			return true;

	return false;
}

/* Walk a USB bus or hub; only hubs are descended into. */
static struct usb_device *mt7601u_find_in(struct udevice *parent)
{
	struct udevice *child;
	struct usb_device *udev;

	device_foreach_child(child, parent) {
		if (!device_active(child))
			continue;

		udev = dev_get_parent_priv(child);
		if (!udev)
			continue;
		if (mt7601u_id_match(udev))
			return udev;

		if (device_get_uclass_id(child) == UCLASS_USB_HUB) {
			udev = mt7601u_find_in(child);
			if (udev)
				return udev;
		}
	}

	return NULL;
}

/* mt7601u_assign_pipes() */
static int mt7601u_assign_pipes(struct mt7601u_dev *dev)
{
	struct usb_interface *intf = &dev->udev->config.if_desc[0];
	unsigned int i, ep_i = 0, ep_o = 0;

	for (i = 0; i < intf->no_of_ep; i++) {
		struct usb_endpoint_descriptor *ep = &intf->ep_desc[i];
		u8 addr = ep->bEndpointAddress;
		u16 maxp = get_unaligned_le16(&ep->wMaxPacketSize) & 0x7ff;
		bool bulk = (ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
			USB_ENDPOINT_XFER_BULK;

		if (!bulk)
			continue;

		if ((addr & USB_DIR_IN) && ep_i++ < __MT_EP_IN_MAX) {
			dev->in_eps[ep_i - 1] = (addr & 0x0f) | USB_DIR_IN;
			dev->in_max_packet = maxp;
		} else if (!(addr & USB_DIR_IN) && ep_o++ < __MT_EP_OUT_MAX) {
			dev->out_eps[ep_o - 1] = addr & 0x0f;
			dev->out_max_packet = maxp;
		}
	}

	if (ep_i != __MT_EP_IN_MAX || ep_o != __MT_EP_OUT_MAX) {
		printf("mt7601u: wrong pipe number in:%d out:%d\n", ep_i, ep_o);
		return -EINVAL;
	}

	return 0;
}

int mt7601u_usb_find(struct mt7601u_dev *dev)
{
	struct udevice *bus;
	struct usb_device *udev = NULL;
	int i;

	for (uclass_find_first_device(UCLASS_USB, &bus); bus && !udev;
	     uclass_find_next_device(&bus)) {
		if (!device_active(bus))
			continue;
		udev = mt7601u_find_in(bus);
	}

	if (!udev) {
		printf("mt7601u: no MT7601U dongle found (run 'usb start'?)\n");
		return -ENODEV;
	}

	dev->udev = udev;
	printf("mt7601u: found %04x:%04x on %s, %d endpoints on intf 0\n",
	       udev->descriptor.idVendor, udev->descriptor.idProduct,
	       udev->dev ? udev->dev->name : "?",
	       udev->config.if_desc[0].no_of_ep);

	if (mt7601u_assign_pipes(dev))
		return -EINVAL;

	printf("mt7601u: in eps:");
	for (i = 0; i < __MT_EP_IN_MAX; i++)
		printf(" %02x", dev->in_eps[i]);
	printf(" (maxp %u)  out eps:", dev->in_max_packet);
	for (i = 0; i < __MT_EP_OUT_MAX; i++)
		printf(" %02x", dev->out_eps[i]);
	printf(" (maxp %u)\n", dev->out_max_packet);

	/*
	 * Warm 'usb reset's can leave the dongle enumerated at full speed:
	 * on a Le Potato (GXL) it stayed at 12 Mb/s, maxp 64, through repeated
	 * resets until its power was cycled. It still scans and joins, but at
	 * a fraction of the bulk rate, so say so rather than run slow quietly.
	 */
	if (dev->udev->speed != USB_SPEED_HIGH)
		printf("mt7601u: WARNING: dongle is at %s speed, not high speed (480 Mb/s); transfers will be slow -- power-cycle the dongle (replug) to recover\n",
		       dev->udev->speed == USB_SPEED_FULL ? "full (12 Mb/s)" :
		       "low");

	return 0;
}

static const char *mt7601u_ep_state_name(u32 state)
{
	static const char * const names[] = {
		"Disabled", "Running", "Halted", "Stopped", "Error",
	};

	return state < ARRAY_SIZE(names) ? names[state] : "reserved";
}

static const char * const mt7601u_ep_label[3] = {
	"inband-cmd OUT", "cmd-resp IN", "pkt-rx IN",
};

static unsigned long mt7601u_ep_pipe(struct mt7601u_dev *dev, int which)
{
	switch (which) {
	case 0:
		return usb_sndbulkpipe(dev->udev,
				       dev->out_eps[MT_EP_OUT_INBAND_CMD] & 0xf);
	case 1:
		return usb_rcvbulkpipe(dev->udev,
				       dev->in_eps[MT_EP_IN_CMD_RESP] & 0xf);
	default:
		return usb_rcvbulkpipe(dev->udev,
				       dev->in_eps[MT_EP_IN_PKT_RX] & 0xf);
	}
}

static void mt7601u_diag_eps(struct mt7601u_dev *dev)
{
	int i;

	for (i = 0; i < 3; i++) {
		u32 state = 0;
		u64 deq = 0, enq = 0;
		int ret = xhci_ep_debug_state(dev->udev,
					      mt7601u_ep_pipe(dev, i),
					      &state, &deq, &enq);

		if (ret) {
			printf("  xhci %-15s: unavailable (%d)\n",
			       mt7601u_ep_label[i], ret);
			continue;
		}
		printf("  xhci %-15s: %-8s (%u)  hw deq %010llx  sw enq %010llx%s\n",
		       mt7601u_ep_label[i], mt7601u_ep_state_name(state),
		       state, deq, enq,
		       (deq & ~0xfULL) == (enq & ~0xfULL) ? "  (idle)" :
		       "  (TRBs outstanding)");
	}
}

/*
 * Dongle state through control transfers only, plus the xHCI endpoint
 * contexts. Registers are the ones the Linux driver uses for MCU / FCE /
 * USB DMA / queue status.
 */
void mt7601u_diag(struct mt7601u_dev *dev, const char *why)
{
	static const struct {
		u32 reg;
		const char *name;
	} regs[] = {
		{ MT_ASIC_VERSION,		"ASIC_VERSION" },
		{ MT_MAC_CSR0,			"MAC_CSR0" },
		{ MT_MCU_COM_REG0,		"MCU_COM_REG0" },
		{ MT_MCU_COM_REG1,		"MCU_COM_REG1" },
		{ MT_MCU_COM_REG2,		"MCU_COM_REG2" },
		{ MT_MCU_COM_REG3,		"MCU_COM_REG3" },
		{ MT_USB_DMA_CFG,		"USB_DMA_CFG" },
		{ MT_FCE_PSE_CTRL,		"FCE_PSE_CTRL" },
		{ MT_FCE_PDMA_GLOBAL_CONF,	"FCE_PDMA_GLOBAL_CONF" },
		{ MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, "TX_CPU_FROM_FCE_CPUIDX" },
		{ MT_WPDMA_GLO_CFG,		"WPDMA_GLO_CFG" },
		{ MT_INT_SOURCE_CSR,		"INT_SOURCE_CSR" },
		{ MT_PBF_CFG,			"PBF_CFG" },
		{ MT_TXQ_STA,			"TXQ_STA" },
		{ MT_RXQ_STA,			"RXQ_STA" },
		{ MT_MAC_SYS_CTRL,		"MAC_SYS_CTRL" },
		{ MT_MAC_STATUS,		"MAC_STATUS" },
	};
	u32 val;
	int i;

	printf("mt7601u diag (%s):\n", why);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		dev->reg_failed = false;
		val = mt7601u_rr(dev, regs[i].reg);
		printf("  %04x %-24s %08x%s", regs[i].reg, regs[i].name, val,
		       dev->reg_failed ? "  (read FAILED)" : "");
		if (regs[i].reg == MT_USB_DMA_CFG && !dev->reg_failed)
			printf("  [rx_busy %d tx_busy %d tx_bulk_en %d rx_bulk_en %d]",
			       !!(val & MT_USB_DMA_CFG_RX_BUSY),
			       !!(val & MT_USB_DMA_CFG_TX_BUSY),
			       !!(val & MT_USB_DMA_CFG_TX_BULK_EN),
			       !!(val & MT_USB_DMA_CFG_RX_BULK_EN));
		if (regs[i].reg == MT_WPDMA_GLO_CFG && !dev->reg_failed)
			printf("  [tx_dma_busy %d rx_dma_busy %d]",
			       !!(val & MT_WPDMA_GLO_CFG_TX_DMA_BUSY),
			       !!(val & MT_WPDMA_GLO_CFG_RX_DMA_BUSY));
		printf("\n");
	}
	dev->reg_failed = false;

	mt7601u_diag_eps(dev);
	printf("  usb: speed %d, in maxp %u, out maxp %u, port %d\n",
	       dev->udev->speed, dev->in_max_packet, dev->out_max_packet,
	       dev->udev->portnr);
	printf("  mcu: pending %d seq %u tx_seq %u; late replies %u (last %08x); bad events %u (last %08x); last resp %08x\n",
	       dev->mcu_pending, dev->mcu_pending_seq, dev->mcu_tx_seq,
	       dev->mcu_late_resp, dev->mcu_last_late, dev->mcu_bad_resp,
	       dev->mcu_last_bad, dev->mcu_last_resp);
}
