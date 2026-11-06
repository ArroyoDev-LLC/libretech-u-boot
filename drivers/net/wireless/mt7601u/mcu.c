// SPDX-License-Identifier: GPL-2.0-only
/*
 * (c) Copyright 2002-2010, Ralink Technology, Inc.
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * U-Boot background-scan port of the Linux mt7601u MCU layer
 * (drivers/net/wireless/mediatek/mt7601u/mcu.c): inband commands and the
 * ILM/DLM firmware DMA upload. Command responses are read synchronously at
 * init, and from the xHCI async cmd-response TD during a scan. The firmware
 * is linked in (fw.S) instead of request_firmware().
 */

#include <env.h>
#include <log.h>
#include <time.h>
#include <usb.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>

#include "mt7601u.h"
#include "dma.h"
#include "mcu.h"

extern const u8 mt7601u_fw_start[];
extern const u8 mt7601u_fw_end[];

static inline int firmware_running(struct mt7601u_dev *dev)
{
	return mt7601u_rr(dev, MT_MCU_COM_REG0) == 1;
}

static int mt7601u_resp_submit(struct mt7601u_dev *dev, int h, u8 *buf)
{
	int ret = xhci_async_bulk_submit(h, buf, MCU_RESP_URB_SIZE);

	if (ret == -EBUSY)
		return 0;	/* already posted (or done, not yet polled) */
	if (ret) {
		dev->async_submit_err++;
		dev->async_last_err = ret;
		if (ret == -ENODEV)
			dev->gone = true;
	}
	return ret;
}

/*
 * async mode: post every idle cmd-resp TD. Two are kept posted so that a
 * re-sent command's back-to-back replies (same seq, one per copy) both land;
 * with a single TD the second reply has nowhere to go and the MCU then NAKs
 * every following command OUT ('send MCU cmd N failed:-110').
 */
int mt7601u_async_resp_arm(struct mt7601u_dev *dev)
{
	int ret = mt7601u_resp_submit(dev, dev->async_resp_h, dev->resp_buf);

	if (!ret && dev->async_resp_h2 >= 0)
		ret = mt7601u_resp_submit(dev, dev->async_resp_h2,
					  dev->resp_buf2);
	return ret;
}

/*
 * One response word read from the cmd-resp endpoint.
 * Return: 0 = CMD_DONE for @seq, -EINPROGRESS = nothing arrived,
 * -EALREADY = CMD_DONE for another seq (a late reply to an earlier command,
 * consumed and counted, not an error of this one), -EPROTO = error / unknown
 * event, -EIO = transfer error, -ENODEV = controller gone.
 *
 * Seq scheme as Linux mcu.c: 1..15 (0 = no response wanted), one new seq per
 * command; a re-send of the same command keeps its seq, so a reply to any
 * copy completes it.
 */
static int mt7601u_mcu_read_resp_async(struct mt7601u_dev *dev, u32 *rxfce)
{
	/*
	 * TDs on one endpoint complete in submission order, so the next reply
	 * is always in the TD posted longest ago: poll that one, consume it,
	 * re-post it (it is now the newest) and flip to the other.
	 */
	bool two = dev->async_resp_h2 >= 0;
	int h = (two && dev->resp_next) ? dev->async_resp_h2 : dev->async_resp_h;
	u8 *buf = (two && dev->resp_next) ? dev->resp_buf2 : dev->resp_buf;
	int ret, actual = 0;

	ret = xhci_async_bulk_poll(h, &actual);
	if (ret == -EINPROGRESS)
		return -EINPROGRESS;
	if (ret == -ENODEV) {
		dev->gone = true;
		return -ENODEV;
	}
	if (ret == -ENOENT) {
		/* cancelled by a synchronous transfer: re-queue; the other TD
		 * is now the older one */
		mt7601u_resp_submit(dev, h, buf);
		if (two)
			dev->resp_next ^= 1;
		return -EINPROGRESS;
	}

	if (!ret && actual >= 4)
		*rxfce = get_unaligned_le32(buf);
	mt7601u_resp_submit(dev, h, buf);
	if (two)
		dev->resp_next ^= 1;

	if (ret)
		return -EIO;
	return actual < 4 ? -EPROTO : 0;
}

static int mt7601u_mcu_read_resp(struct mt7601u_dev *dev, u8 seq,
				 unsigned int window_us)
{
	u32 rxfce = 0;
	int ret, actual;

	if (dev->async_resp_h >= 0) {
		ret = mt7601u_mcu_read_resp_async(dev, &rxfce);
		if (ret)
			return ret;
	} else {
		ret = mt7601u_bulk(dev, true, MT_EP_IN_CMD_RESP, dev->resp_buf,
				   MCU_RESP_URB_SIZE, &actual, 300);
		if (ret == -ETIMEDOUT)
			return -EINPROGRESS;
		if (ret)
			return -EIO;
		if (actual < 4)
			return -EPROTO;
		rxfce = get_unaligned_le32(dev->resp_buf);
	}

	dev->mcu_last_resp = rxfce;

	if (FIELD_GET(MT_RXD_CMD_INFO_EVT_TYPE, rxfce) == CMD_DONE) {
		if (FIELD_GET(MT_RXD_CMD_INFO_CMD_SEQ, rxfce) == seq)
			return 0;
		dev->mcu_late_resp++;
		dev->mcu_last_late = rxfce;
		return -EALREADY;
	}

	dev->mcu_bad_resp++;
	dev->mcu_last_bad = rxfce;
	return -EPROTO;
}

/*
 * Packet-RX shares the FCE with command responses. While the scan's
 * standing TD is down, empty that FIFO so a full one cannot stall CMD_DONE.
 * A short timeout on an empty FIFO ends the drain.
 */
#define MT_MCU_DRAIN_TOUT_MS	5
#define MT_MCU_DRAIN_MAX	8

static void mt7601u_mcu_drain_pkt_rx(struct mt7601u_dev *dev)
{
	int i, actual;

	if (dev->async_rx_h >= 0 || !dev->rx_buf)
		return;

	for (i = 0; i < MT_MCU_DRAIN_MAX; i++) {
		if (mt7601u_bulk(dev, true, MT_EP_IN_PKT_RX, dev->rx_buf,
				 MT_RX_URB_SIZE, &actual, MT_MCU_DRAIN_TOUT_MS))
			return;
	}
}

/* mt7601u_mcu_wait_resp(): synchronous, 5 x 300 ms, never from the cyclic */
static int mt7601u_mcu_wait_resp(struct mt7601u_dev *dev, u8 seq)
{
	int ret, i = 5, late = 16;

	mt7601u_mcu_drain_pkt_rx(dev);

	while (i--) {
		ret = mt7601u_mcu_read_resp(dev, seq, 300000);
		if (!ret)
			return 0;
		if (ret == -EALREADY && late--) {
			i++;	/* a stale reply does not use up a wait */
			continue;
		}
		if (ret == -EINPROGRESS) {
			printf("mt7601u: %s retrying\n", __func__);
			mt7601u_mcu_drain_pkt_rx(dev);
		} else if (ret == -EPROTO)
			printf("mt7601u: MCU resp evt:%lx seq:%x-%lx!\n",
			       (ulong)FIELD_GET(MT_RXD_CMD_INFO_EVT_TYPE,
						dev->mcu_last_resp),
			       (unsigned int)seq,
			       (ulong)FIELD_GET(MT_RXD_CMD_INFO_CMD_SEQ,
						dev->mcu_last_resp));
		else
			printf("mt7601u: MCU resp xfer failed:%d\n", ret);
	}

	printf("mt7601u: %s timed out\n", __func__);
	return -ETIMEDOUT;
}

/*
 * The send half of mt7601u_mcu_msg_send() + mt7601u_dma_skb_wrap(); the
 * buffer is
 *	|   4B   | xfer len |      pad       |  4B  |
 *	| TXINFO | pkt/cmd  | zero pad to 4B | zero |
 *
 * @reuse_seq: nonzero = re-send of a command already given that seq.
 */
static int mt7601u_mcu_msg_tx(struct mt7601u_dev *dev, const void *data,
			      int len, enum mcu_cmd cmd, bool want_resp,
			      u8 reuse_seq, u8 *seq_out)
{
	u8 *buf = dev->cmd_buf;
	int plen = round_up(len, 4);
	int xfer = MT_DMA_HDR_LEN + plen + 4;
	int ret, sent;
	u8 seq = 0;
	u32 info;

	if (xfer > MT_CMD_BUF_SIZE)
		return -EINVAL;

	if (want_resp) {
		seq = reuse_seq;
		while (!seq)
			seq = ++dev->mcu_msg_seq & 0xf;
	}
	*seq_out = seq;

	info = FIELD_PREP(MT_TXD_CMD_INFO_SEQ, seq) |
	       FIELD_PREP(MT_TXD_CMD_INFO_TYPE, cmd) |
	       FIELD_PREP(MT_TXD_INFO_LEN, plen) |
	       FIELD_PREP(MT_TXD_INFO_D_PORT, CPU_TX_PORT) |
	       FIELD_PREP(MT_TXD_INFO_TYPE, DMA_COMMAND);

	memset(buf, 0, xfer);
	put_unaligned_le32(info, buf);
	memcpy(buf + MT_DMA_HDR_LEN, data, len);

	ret = mt7601u_bulk(dev, false, MT_EP_OUT_INBAND_CMD, buf, xfer, &sent,
			   500);
	if (ret) {
		mt_err(dev, "send MCU cmd %d failed:%d\n", cmd, ret);
		return ret;
	}
	if (sent != xfer)
		mt_err(dev, "%s sent %d != %d\n", __func__, sent, xfer);

	return 0;
}

static int mt7601u_mcu_msg_send(struct mt7601u_dev *dev, const void *data,
				int len, enum mcu_cmd cmd, bool wait_resp)
{
	int ret;
	u8 seq;

	if (dev->mcu_pending) {
		printf("mt7601u: MCU command while an async one is pending\n");
		return -EBUSY;
	}

	ret = mt7601u_mcu_msg_tx(dev, data, len, cmd, wait_resp, 0, &seq);
	if (ret)
		return ret;

	if (wait_resp) {
		printf("mt7601u: MCU cmd %u seq %u waiting\n",
		       (unsigned int)cmd, seq);
		ret = mt7601u_mcu_wait_resp(dev, seq);
	}

	return ret;
}

/*
 * Async: send with a response requested; the caller polls per tick.
 * @resend: another attempt at the command whose previous OUT failed -- it
 * keeps that seq (dev->mcu_tx_seq).
 */
static int mt7601u_mcu_msg_send_async(struct mt7601u_dev *dev,
				      const void *data, int len,
				      enum mcu_cmd cmd, bool resend)
{
	int ret;
	u8 seq;

	if (dev->mcu_pending)
		return -EBUSY;

	ret = mt7601u_mcu_msg_tx(dev, data, len, cmd, true,
				 resend ? dev->mcu_tx_seq : 0, &seq);
	dev->mcu_tx_seq = seq;
	if (ret)
		return ret;

	dev->mcu_pending = true;
	dev->mcu_pending_seq = seq;
	dev->mcu_sent_at = get_timer(0);
	return 0;
}

/*
 * One short poll for the outstanding async command's response.
 * Return: 0 done, -EINPROGRESS nothing yet, -EPROTO wrong (still pending),
 * -EIO transfer error (still pending), -EALREADY late reply consumed,
 * -ENODEV controller gone, -ENOENT nothing pending.
 */
int mt7601u_mcu_poll_resp(struct mt7601u_dev *dev, unsigned int window_us)
{
	int ret;

	if (!dev->mcu_pending)
		return -ENOENT;

	ret = mt7601u_mcu_read_resp(dev, dev->mcu_pending_seq, window_us);
	if (!ret)
		dev->mcu_pending = false;

	return ret;
}

int mt7601u_write_reg_pairs(struct mt7601u_dev *dev, u32 base,
			    const struct mt76_reg_pair *data, int n)
{
	const int max_vals_per_cmd = INBAND_PACKET_MAX_LEN / 8;
	__le32 msg[INBAND_PACKET_MAX_LEN / 4];
	int cnt, i, ret;

	while (n) {
		cnt = min(max_vals_per_cmd, n);

		for (i = 0; i < cnt; i++) {
			msg[i * 2] = cpu_to_le32(base + data[i].reg);
			msg[i * 2 + 1] = cpu_to_le32(data[i].value);
		}

		ret = mt7601u_mcu_msg_send(dev, msg, cnt * 8, CMD_RANDOM_WRITE,
					   cnt == n);
		if (ret)
			return ret;

		data += cnt;
		n -= cnt;
	}

	return 0;
}

int mt7601u_burst_write_regs(struct mt7601u_dev *dev, u32 offset,
			     const u32 *data, int n)
{
	const int max_regs_per_cmd = INBAND_PACKET_MAX_LEN / 4 - 1;
	__le32 msg[INBAND_PACKET_MAX_LEN / 4];
	int cnt, i, ret;

	while (n) {
		cnt = min(max_regs_per_cmd, n);

		msg[0] = cpu_to_le32(MT_MCU_MEMMAP_WLAN + offset);
		for (i = 0; i < cnt; i++)
			msg[i + 1] = cpu_to_le32(data[i]);

		ret = mt7601u_mcu_msg_send(dev, msg, cnt * 4 + 4,
					   CMD_BURST_WRITE, cnt == n);
		if (ret)
			return ret;

		offset += cnt * 4;
		data += cnt;
		n -= cnt;
	}

	return 0;
}

/* synchronous calibrate, used by the init-time PHY calibrations */
int mt7601u_mcu_calibrate(struct mt7601u_dev *dev, enum mcu_calibrate cal,
			  u32 val)
{
	__le32 msg[2] = { cpu_to_le32(cal), cpu_to_le32(val) };

	if (env_get("mt7601u_skip_cal"))
		return 0;

	return mt7601u_mcu_msg_send(dev, msg, sizeof(msg), CMD_CALIBRATION_OP,
				    true);
}

int mt7601u_mcu_reg_pairs_send(struct mt7601u_dev *dev, u32 base,
			       const struct mt76_reg_pair *data, int n,
			       bool resend)
{
	__le32 msg[INBAND_PACKET_MAX_LEN / 4];
	int i;

	if (n > INBAND_PACKET_MAX_LEN / 8)
		return -EINVAL;

	for (i = 0; i < n; i++) {
		msg[i * 2] = cpu_to_le32(base + data[i].reg);
		msg[i * 2 + 1] = cpu_to_le32(data[i].value);
	}

	return mt7601u_mcu_msg_send_async(dev, msg, n * 8, CMD_RANDOM_WRITE,
					  resend);
}

int mt7601u_mcu_calibrate_send(struct mt7601u_dev *dev, int cal, u32 val,
			       bool resend)
{
	__le32 msg[2] = { cpu_to_le32(cal), cpu_to_le32(val) };

	if (env_get("mt7601u_skip_cal"))
		return 0;

	return mt7601u_mcu_msg_send_async(dev, msg, sizeof(msg),
					  CMD_CALIBRATION_OP, resend);
}

struct mt76_fw_header {
	__le32 ilm_len;
	__le32 dlm_len;
	__le16 build_ver;
	__le16 fw_ver;
	u8 pad[4];
	char build_time[16];
};

struct mt76_fw {
	struct mt76_fw_header hdr;
	u8 ivb[MT_MCU_IVB_SIZE];
	u8 ilm[];
};

static int __mt7601u_dma_fw(struct mt7601u_dev *dev, const void *data,
			    u32 len, u32 dst_addr)
{
	u8 *buf = dev->fw_buf;
	int ret, sent, xfer;
	__le32 reg;
	u32 val;

	reg = cpu_to_le32(FIELD_PREP(MT_TXD_INFO_TYPE, DMA_PACKET) |
			  FIELD_PREP(MT_TXD_INFO_D_PORT, CPU_TX_PORT) |
			  FIELD_PREP(MT_TXD_INFO_LEN, len));
	memcpy(buf, &reg, sizeof(reg));
	memcpy(buf + sizeof(reg), data, len);
	memset(buf + sizeof(reg) + len, 0, 8);

	ret = mt7601u_vendor_single_wr(dev, MT_VEND_WRITE_FCE,
				       MT_FCE_DMA_ADDR, dst_addr);
	if (ret)
		return ret;
	len = roundup(len, 4);
	ret = mt7601u_vendor_single_wr(dev, MT_VEND_WRITE_FCE,
				       MT_FCE_DMA_LEN, len << 16);
	if (ret)
		return ret;

	xfer = MT_DMA_HDR_LEN + len + 4;
	ret = mt7601u_bulk(dev, false, MT_EP_OUT_INBAND_CMD, buf, xfer, &sent,
			   1000);
	if (ret) {
		printf("mt7601u: firmware upload xfer failed:%d\n", ret);
		return ret;
	}

	val = mt7601u_rr(dev, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX);
	val++;
	mt7601u_wr(dev, MT_TX_CPU_FROM_FCE_CPU_DESC_IDX, val);

	return 0;
}

static int mt7601u_dma_fw(struct mt7601u_dev *dev, const u8 *data, int len,
			  u32 dst_addr)
{
	int n, ret;

	while (len > 0) {
		n = min(MCU_FW_URB_MAX_PAYLOAD, len);
		ret = __mt7601u_dma_fw(dev, data, n, dst_addr);
		if (ret)
			return ret;

		if (!mt76_poll_msec(dev, MT_MCU_COM_REG1, BIT(31), BIT(31),
				    500))
			return -ETIMEDOUT;

		data += n;
		len -= n;
		dst_addr += n;
	}

	return 0;
}

static int mt7601u_upload_firmware(struct mt7601u_dev *dev,
				   const struct mt76_fw *fw)
{
	u32 ilm_len, dlm_len;
	int i, ret;

	ilm_len = le32_to_cpu(fw->hdr.ilm_len) - sizeof(fw->ivb);
	debug("mt7601u: loading FW - ILM %u + IVB %zu\n", ilm_len,
	      sizeof(fw->ivb));
	ret = mt7601u_dma_fw(dev, fw->ilm, ilm_len, sizeof(fw->ivb));
	if (ret)
		return ret;

	dlm_len = le32_to_cpu(fw->hdr.dlm_len);
	debug("mt7601u: loading FW - DLM %u\n", dlm_len);
	ret = mt7601u_dma_fw(dev, fw->ilm + ilm_len, dlm_len,
			     MT_MCU_DLM_OFFSET);
	if (ret)
		return ret;

	/* IVB via a DMA-able copy */
	memcpy(dev->fw_buf, fw->ivb, sizeof(fw->ivb));
	ret = mt7601u_vendor_request(dev, MT_VEND_DEV_MODE, USB_DIR_OUT,
				     0x12, 0, dev->fw_buf, sizeof(fw->ivb));
	if (ret < 0)
		return ret;

	for (i = 100; i && !firmware_running(dev); i--)
		mdelay(10);
	if (!i)
		return -ETIMEDOUT;

	debug("mt7601u: Firmware running!\n");
	return 0;
}

static int mt7601u_load_firmware(struct mt7601u_dev *dev)
{
	const u8 *data = mt7601u_fw_start;
	size_t size = mt7601u_fw_end - mt7601u_fw_start;
	const struct mt76_fw_header *hdr;
	int len;
	u32 val;

	if (size < sizeof(*hdr))
		goto err_inv_fw;

	hdr = (const struct mt76_fw_header *)data;

	if (le32_to_cpu(hdr->ilm_len) <= MT_MCU_IVB_SIZE)
		goto err_inv_fw;

	len = sizeof(*hdr);
	len += le32_to_cpu(hdr->ilm_len);
	len += le32_to_cpu(hdr->dlm_len);

	if (size != len)
		goto err_inv_fw;

	val = le16_to_cpu(hdr->fw_ver);
	dev->fw_ver = val;
	dev->fw_build = le16_to_cpu(hdr->build_ver);
	printf("mt7601u: Firmware Version: %d.%d.%02d Build: %x Build time: %.16s\n",
	       (val >> 12) & 0xf, (val >> 8) & 0xf, val & 0xf,
	       le16_to_cpu(hdr->build_ver), hdr->build_time);

	/*
	 * A second init finds the MCU still running. DEV_MODE reset stalls
	 * (req 1, -EPIPE) and the reload does not start. Keep this firmware
	 * and let the rest of hardware init rewrite the MAC.
	 */
	if (firmware_running(dev)) {
		printf("mt7601u: MCU already running\n");
		return 0;
	}

	mt7601u_wr(dev, MT_USB_DMA_CFG, (MT_USB_DMA_CFG_RX_BULK_EN |
					 MT_USB_DMA_CFG_TX_BULK_EN));

	mt7601u_wr(dev, 0x94c, 0);
	mt7601u_wr(dev, MT_FCE_PSE_CTRL, 0);

	mt7601u_vendor_reset(dev);
	mdelay(5);

	mt7601u_wr(dev, 0xa44, 0);
	mt7601u_wr(dev, 0x230, 0x84210);
	mt7601u_wr(dev, 0x400, 0x80c00);
	mt7601u_wr(dev, 0x800, 1);

	mt7601u_rmw(dev, MT_PBF_CFG, 0, (MT_PBF_CFG_TX0Q_EN |
					 MT_PBF_CFG_TX1Q_EN |
					 MT_PBF_CFG_TX2Q_EN |
					 MT_PBF_CFG_TX3Q_EN));

	mt7601u_wr(dev, MT_FCE_PSE_CTRL, 1);

	mt7601u_wr(dev, MT_USB_DMA_CFG, (MT_USB_DMA_CFG_RX_BULK_EN |
					 MT_USB_DMA_CFG_TX_BULK_EN));
	val = mt76_set(dev, MT_USB_DMA_CFG, MT_USB_DMA_CFG_TX_CLR);
	val &= ~MT_USB_DMA_CFG_TX_CLR;
	mt7601u_wr(dev, MT_USB_DMA_CFG, val);

	/* FCE tx_fs_base_ptr */
	mt7601u_wr(dev, MT_TX_CPU_FROM_FCE_BASE_PTR, 0x400230);
	/* FCE tx_fs_max_cnt */
	mt7601u_wr(dev, MT_TX_CPU_FROM_FCE_MAX_COUNT, 1);
	/* FCE pdma enable */
	mt7601u_wr(dev, MT_FCE_PDMA_GLOBAL_CONF, 0x44);
	/* FCE skip_fs_en */
	mt7601u_wr(dev, MT_FCE_SKIP_FS, 3);

	return mt7601u_upload_firmware(dev, (const struct mt76_fw *)data);

err_inv_fw:
	printf("mt7601u: Invalid firmware image (%zu bytes linked from %s)\n",
	       size, CONFIG_WIFI_MT7601U_FIRMWARE_PATH);
	return -ENOENT;
}

int mt7601u_mcu_init(struct mt7601u_dev *dev)
{
	int ret;

	ret = mt7601u_load_firmware(dev);
	if (ret)
		return ret;

	__set_bit(MT7601U_STATE_MCU_RUNNING, &dev->state);

	return 0;
}

int mt7601u_mcu_cmd_init(struct mt7601u_dev *dev)
{
	__le32 msg[2] = { cpu_to_le32(Q_SELECT), cpu_to_le32(1) };

	return mt7601u_mcu_msg_send(dev, msg, sizeof(msg), CMD_FUN_SET_OP,
				    false);
}

void mt7601u_mcu_cmd_deinit(struct mt7601u_dev *dev)
{
}
