// SPDX-License-Identifier: GPL-2.0-only
/*
 * (c) Copyright 2002-2010, Ralink Technology, Inc.
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * U-Boot scan-test port of the Linux mt7601u hardware init
 * (drivers/net/wireless/mediatek/mt7601u/init.c): mt7601u_init_hardware()
 * minus the RX/TX URB queues (mt7601u_dma_init), plus an RX-only variant of
 * mt7601u_mac_start().
 */

#include <log.h>
#include <malloc.h>
#include <linux/delay.h>
#include <linux/errno.h>

#include "mt7601u.h"
#include "eeprom.h"
#include "mcu.h"

#include "initvals.h"

static void
mt7601u_set_wlan_state(struct mt7601u_dev *dev, u32 val, bool enable)
{
	int i;

	if (enable)
		val |= (MT_WLAN_FUN_CTRL_WLAN_EN |
			MT_WLAN_FUN_CTRL_WLAN_CLK_EN);
	else
		val &= ~(MT_WLAN_FUN_CTRL_WLAN_EN);

	mt7601u_wr(dev, MT_WLAN_FUN_CTRL, val);
	udelay(20);

	if (enable) {
		__set_bit(MT7601U_STATE_WLAN_RUNNING, &dev->state);
	} else {
		__clear_bit(MT7601U_STATE_WLAN_RUNNING, &dev->state);
		return;
	}

	for (i = 200; i; i--) {
		val = mt7601u_rr(dev, MT_CMB_CTRL);

		if (val & MT_CMB_CTRL_XTAL_RDY && val & MT_CMB_CTRL_PLL_LD)
			break;

		udelay(20);
	}

	if (!i)
		printf("mt7601u: PLL and XTAL check failed!\n");
}

static void mt7601u_chip_onoff(struct mt7601u_dev *dev, bool enable, bool reset)
{
	u32 val;

	val = mt7601u_rr(dev, MT_WLAN_FUN_CTRL);

	/*
	 * Firmware is already running when WLAN_EN is set. Clearing it
	 * reads back ...0002, the cold-boot value. The WLAN_RESET pulse
	 * still completes its low half and the next vendor OUT times out.
	 * Skip that pulse and let set_wlan_state turn the core back on.
	 */
	if (reset && (val & MT_WLAN_FUN_CTRL_WLAN_EN)) {
		mt7601u_wr(dev, MT_WLAN_FUN_CTRL,
			   val & ~MT_WLAN_FUN_CTRL_WLAN_EN);
		mdelay(10);
		val = mt7601u_rr(dev, MT_WLAN_FUN_CTRL);
		printf("mt7601u: WLAN_FUN_CTRL warm %08x\n", val);
		reset = false;
	}

	if (reset) {
		val |= MT_WLAN_FUN_CTRL_GPIO_OUT_EN;
		val &= ~MT_WLAN_FUN_CTRL_FRC_WL_ANT_SEL;

		/*
		 * As linux-6.18.y-lc (bedf2669d50): always assert WLAN_EN so
		 * the WLAN_RESET | WLAN_RESET_RF pulse fires even on a warm
		 * reboot where the USB bus reset left WLAN_EN clear while the
		 * core kept running from retained VBUS. Harmless on cold boot.
		 */
		val |= MT_WLAN_FUN_CTRL_WLAN_EN;
		val |= (MT_WLAN_FUN_CTRL_WLAN_RESET |
			MT_WLAN_FUN_CTRL_WLAN_RESET_RF);
		mt7601u_wr(dev, MT_WLAN_FUN_CTRL, val);
		udelay(20);

		val &= ~(MT_WLAN_FUN_CTRL_WLAN_RESET |
			 MT_WLAN_FUN_CTRL_WLAN_RESET_RF);
	}

	mt7601u_wr(dev, MT_WLAN_FUN_CTRL, val);
	udelay(20);

	mt7601u_set_wlan_state(dev, val, enable);
}

static void mt7601u_reset_csr_bbp(struct mt7601u_dev *dev)
{
	mt7601u_wr(dev, MT_MAC_SYS_CTRL, (MT_MAC_SYS_CTRL_RESET_CSR |
					  MT_MAC_SYS_CTRL_RESET_BBP));
	mt7601u_wr(dev, MT_USB_DMA_CFG, 0);
	mdelay(1);
	mt7601u_wr(dev, MT_MAC_SYS_CTRL, 0);
}

static void mt7601u_init_usb_dma(struct mt7601u_dev *dev)
{
	u32 val;

	val = FIELD_PREP(MT_USB_DMA_CFG_RX_BULK_AGG_TOUT, MT_USB_AGGR_TIMEOUT) |
	      FIELD_PREP(MT_USB_DMA_CFG_RX_BULK_AGG_LMT,
			 MT_USB_AGGR_SIZE_LIMIT) |
	      MT_USB_DMA_CFG_RX_BULK_EN |
	      MT_USB_DMA_CFG_TX_BULK_EN;
	if (dev->in_max_packet == 512)
		val |= MT_USB_DMA_CFG_RX_BULK_AGG_EN;
	mt7601u_wr(dev, MT_USB_DMA_CFG, val);

	val |= MT_USB_DMA_CFG_UDMA_RX_WL_DROP;
	mt7601u_wr(dev, MT_USB_DMA_CFG, val);
	val &= ~MT_USB_DMA_CFG_UDMA_RX_WL_DROP;
	mt7601u_wr(dev, MT_USB_DMA_CFG, val);
}

static int mt7601u_init_bbp(struct mt7601u_dev *dev)
{
	int ret;

	ret = mt7601u_wait_bbp_ready(dev);
	if (ret)
		return ret;

	/*
	 * Host CSR bridge, same as the Linux driver's mode-3 init tables.
	 * An MCU RANDOM_WRITE of these pairs would be the first waited
	 * command, ahead of the WCID burst that image already answers.
	 */
	mt7601u_write_bbp_pairs(dev, bbp_common_vals,
				ARRAY_SIZE(bbp_common_vals));
	mt7601u_write_bbp_pairs(dev, bbp_chip_vals, ARRAY_SIZE(bbp_chip_vals));
	mt7601u_print_bbp_init_regs(dev);

	return 0;
}

static void
mt76_init_beacon_offsets(struct mt7601u_dev *dev)
{
	u16 base = MT_BEACON_BASE;
	u32 regs[4] = {};
	int i;

	for (i = 0; i < 16; i++) {
		u16 addr = dev->beacon_offsets[i];

		regs[i / 4] |= ((addr - base) / 64) << (8 * (i % 4));
	}

	for (i = 0; i < 4; i++)
		mt7601u_wr(dev, MT_BCN_OFFSET(i), regs[i]);
}

static void mt7601u_write_mac_pairs(struct mt7601u_dev *dev,
				    const struct mt76_reg_pair *data, int n)
{
	int i;

	for (i = 0; i < n; i++)
		mt7601u_wr(dev, data[i].reg, data[i].value);
}

static int mt7601u_write_mac_initvals(struct mt7601u_dev *dev)
{
	/*
	 * Vendor-window writes. CMD_RANDOM_WRITE of this table is a direct
	 * store into the WLAN window, and that wait returned no frame.
	 */
	mt7601u_write_mac_pairs(dev, mac_common_vals,
				ARRAY_SIZE(mac_common_vals));
	mt7601u_write_mac_pairs(dev, mac_chip_vals, ARRAY_SIZE(mac_chip_vals));

	mt76_init_beacon_offsets(dev);

	mt7601u_wr(dev, MT_BCN_BYPASS_MASK, 0xffff);

	mt7601u_wr(dev, MT_AUX_CLK_CFG, 0);

	return 0;
}

static int mt7601u_init_wcid_mem(struct mt7601u_dev *dev)
{
	u32 *vals;
	int i, ret;

	vals = malloc(sizeof(*vals) * N_WCIDS * 2);
	if (!vals)
		return -ENOMEM;

	for (i = 0; i < N_WCIDS; i++)  {
		vals[i * 2] = 0xffffffff;
		vals[i * 2 + 1] = 0x00ffffff;
	}

	ret = mt7601u_burst_write_regs(dev, MT_WCID_ADDR_BASE,
				       vals, N_WCIDS * 2);
	free(vals);

	return ret;
}

static int mt7601u_init_key_mem(struct mt7601u_dev *dev)
{
	u32 vals[4] = {};

	return mt7601u_burst_write_regs(dev, MT_SKEY_MODE_BASE_0,
					vals, ARRAY_SIZE(vals));
}

static int mt7601u_init_wcid_attr_mem(struct mt7601u_dev *dev)
{
	u32 *vals;
	int i, ret;

	vals = malloc(sizeof(*vals) * N_WCIDS * 2);
	if (!vals)
		return -ENOMEM;

	for (i = 0; i < N_WCIDS * 2; i++)
		vals[i] = 1;

	ret = mt7601u_burst_write_regs(dev, MT_WCID_ATTR_BASE,
				       vals, N_WCIDS * 2);
	free(vals);

	return ret;
}

static void mt7601u_reset_counters(struct mt7601u_dev *dev)
{
	mt7601u_rr(dev, MT_RX_STA_CNT0);
	mt7601u_rr(dev, MT_RX_STA_CNT1);
	mt7601u_rr(dev, MT_RX_STA_CNT2);
	mt7601u_rr(dev, MT_TX_STA_CNT0);
	mt7601u_rr(dev, MT_TX_STA_CNT1);
	mt7601u_rr(dev, MT_TX_STA_CNT2);
}

/*
 * mt7601u_mac_start(), RX only: MAC RX enabled, TX left disabled (this port
 * never queues a frame). RX filter = the Linux start filter with the
 * "not-to-me" (PROMISC) and OTHER_BSS drops cleared, as mt76_configure_filter()
 * does for FIF_OTHER_BSS during a scan, so beacons and probe responses of
 * every BSS pass.
 */
int mt7601u_mac_start_rx(struct mt7601u_dev *dev)
{
	if (!mt76_poll(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_TX_DMA_BUSY |
		       MT_WPDMA_GLO_CFG_RX_DMA_BUSY, 0, 200000))
		return -ETIMEDOUT;

	/* CRC errors stay in the queue for this scan. PHY errors stay dropped. */
	dev->rxfilter = MT_RX_FILTR_CFG_PHY_ERR |
		MT_RX_FILTR_CFG_VER_ERR | MT_RX_FILTR_CFG_DUP |
		MT_RX_FILTR_CFG_CFACK | MT_RX_FILTR_CFG_CFEND |
		MT_RX_FILTR_CFG_ACK | MT_RX_FILTR_CFG_CTS |
		MT_RX_FILTR_CFG_RTS | MT_RX_FILTR_CFG_PSPOLL |
		MT_RX_FILTR_CFG_BA | MT_RX_FILTR_CFG_CTRL_RSV;
	mt7601u_wr(dev, MT_RX_FILTR_CFG, dev->rxfilter);
	printf("mt7601u: rx filter %08x\n", dev->rxfilter);

	mt7601u_wr(dev, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX);

	if (!mt76_poll(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_TX_DMA_BUSY |
		       MT_WPDMA_GLO_CFG_RX_DMA_BUSY, 0, 50))
		return -ETIMEDOUT;

	return 0;
}

void mt7601u_stop_hardware(struct mt7601u_dev *dev)
{
	mt76_clear(dev, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX |
					 MT_MAC_SYS_CTRL_ENABLE_TX);
	mt7601u_chip_onoff(dev, false, false);
}

int mt7601u_init_hardware(struct mt7601u_dev *dev)
{
	static const u16 beacon_offsets[16] = {
		/* 512 byte per beacon */
		0xc000,	0xc200,	0xc400,	0xc600,
		0xc800,	0xca00,	0xcc00,	0xce00,
		0xd000,	0xd200,	0xd400,	0xd600,
		0xd800,	0xda00,	0xdc00,	0xde00
	};
	int ret;

	dev->beacon_offsets = beacon_offsets;

	printf("mt7601u: WLAN_FUN_CTRL at init %08x\n",
	       mt7601u_rr(dev, MT_WLAN_FUN_CTRL));
	mt7601u_chip_onoff(dev, true, true);

	ret = mt7601u_wait_asic_ready(dev);
	if (ret)
		goto err;
	ret = mt7601u_mcu_init(dev);
	if (ret)
		goto err;

	if (!mt76_poll_msec(dev, MT_WPDMA_GLO_CFG,
			    MT_WPDMA_GLO_CFG_TX_DMA_BUSY |
			    MT_WPDMA_GLO_CFG_RX_DMA_BUSY, 0, 100)) {
		ret = -EIO;
		goto err;
	}

	/* Wait for ASIC ready after FW load. */
	ret = mt7601u_wait_asic_ready(dev);
	if (ret)
		goto err;

	mt7601u_reset_csr_bbp(dev);
	mt7601u_init_usb_dma(dev);

	ret = mt7601u_mcu_cmd_init(dev);
	if (ret)
		goto err;
	/* mt7601u_dma_init(): no URB queues -- RX is polled by the scan cyclic */
	ret = mt7601u_write_mac_initvals(dev);
	if (ret)
		goto err;

	if (!mt76_poll_msec(dev, MT_MAC_STATUS,
			    MT_MAC_STATUS_TX | MT_MAC_STATUS_RX, 0, 100)) {
		ret = -EIO;
		goto err;
	}

	ret = mt7601u_init_bbp(dev);
	if (ret)
		goto err;
	ret = mt7601u_init_wcid_mem(dev);
	if (ret)
		goto err;
	ret = mt7601u_init_key_mem(dev);
	if (ret)
		goto err;
	ret = mt7601u_init_wcid_attr_mem(dev);
	if (ret)
		goto err;

	mt76_clear(dev, MT_BEACON_TIME_CFG, (MT_BEACON_TIME_CFG_TIMER_EN |
					     MT_BEACON_TIME_CFG_SYNC_MODE |
					     MT_BEACON_TIME_CFG_TBTT_EN |
					     MT_BEACON_TIME_CFG_BEACON_TX));

	mt7601u_reset_counters(dev);

	mt7601u_rmw(dev, MT_US_CYC_CFG, MT_US_CYC_CNT, 0x1e);

	mt7601u_wr(dev, MT_TXOP_CTRL_CFG,
		   FIELD_PREP(MT_TXOP_TRUN_EN, 0x3f) |
		   FIELD_PREP(MT_TXOP_EXT_CCA_DLY, 0x58));

	ret = mt7601u_eeprom_init(dev);
	if (ret)
		goto err;

	ret = mt7601u_phy_init(dev);
	if (ret)
		goto err;

	mt7601u_set_rx_path(dev, 0);
	mt7601u_set_tx_dac(dev, 0);

	mt7601u_mac_set_ctrlch(dev, false);
	mt7601u_bbp_set_ctrlch(dev, false);
	mt7601u_bbp_set_bw(dev, MT_BW_20);

	__set_bit(MT7601U_STATE_INITIALIZED, &dev->state);

	return 0;

err:
	mt7601u_chip_onoff(dev, false, false);
	return ret;
}
