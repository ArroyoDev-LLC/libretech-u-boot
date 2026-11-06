// SPDX-License-Identifier: GPL-2.0-only
/*
 * (c) Copyright 2002-2010, Ralink Technology, Inc.
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * U-Boot scan-test port of the Linux mt7601u PHY layer
 * (drivers/net/wireless/mediatek/mt7601u/phy.c): RF/BBP indirect access,
 * mt7601u_phy_init() with its MCU calibrations, and the 20 MHz channel set.
 * The periodic work (TSSI tracking, temperature compensation re-runs, AGC
 * tuning, frequency calibration) is dropped; mt7601u_phy_set_channel() is
 * also exposed as a sequence of short steps for the scan cyclic.
 */

#include <log.h>
#include <time.h>
#include <vsprintf.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "mt7601u.h"
#include "mcu.h"
#include "eeprom.h"
#include "mac.h"
#include "initvals_phy.h"

static void mt7601u_agc_reset(struct mt7601u_dev *dev);

static int
mt7601u_rf_wr(struct mt7601u_dev *dev, u8 bank, u8 offset, u8 value)
{
	int ret = 0;

	if (!test_bit(MT7601U_STATE_WLAN_RUNNING, &dev->state) || offset > 63)
		return -EINVAL;

	if (!mt76_poll(dev, MT_RF_CSR_CFG, MT_RF_CSR_CFG_KICK, 0, 100)) {
		ret = -ETIMEDOUT;
		goto out;
	}

	mt7601u_wr(dev, MT_RF_CSR_CFG,
		   FIELD_PREP(MT_RF_CSR_CFG_DATA, value) |
		   FIELD_PREP(MT_RF_CSR_CFG_REG_BANK, bank) |
		   FIELD_PREP(MT_RF_CSR_CFG_REG_ID, offset) |
		   MT_RF_CSR_CFG_WR |
		   MT_RF_CSR_CFG_KICK);
out:
	if (ret < 0) {
		dev->reg_failed = true;
		mt_err(dev, "RF write %02x:%02x failed:%d!!\n",
		       (unsigned int)bank, (unsigned int)offset, ret);
	}

	return ret;
}

static int
mt7601u_rf_rr(struct mt7601u_dev *dev, u8 bank, u8 offset)
{
	int ret = -ETIMEDOUT;
	u32 val;

	if (!test_bit(MT7601U_STATE_WLAN_RUNNING, &dev->state) || offset > 63)
		return -EINVAL;

	if (!mt76_poll(dev, MT_RF_CSR_CFG, MT_RF_CSR_CFG_KICK, 0, 100))
		goto out;

	mt7601u_wr(dev, MT_RF_CSR_CFG,
		   FIELD_PREP(MT_RF_CSR_CFG_REG_BANK, bank) |
		   FIELD_PREP(MT_RF_CSR_CFG_REG_ID, offset) |
		   MT_RF_CSR_CFG_KICK);

	if (!mt76_poll(dev, MT_RF_CSR_CFG, MT_RF_CSR_CFG_KICK, 0, 100))
		goto out;

	val = mt7601u_rr(dev, MT_RF_CSR_CFG);
	if (FIELD_GET(MT_RF_CSR_CFG_REG_ID, val) == offset &&
	    FIELD_GET(MT_RF_CSR_CFG_REG_BANK, val) == bank)
		ret = FIELD_GET(MT_RF_CSR_CFG_DATA, val);
out:
	if (ret < 0) {
		dev->reg_failed = true;
		mt_err(dev, "RF read %02x:%02x failed:%d!!\n",
		       (unsigned int)bank, (unsigned int)offset, ret);
	}

	return ret;
}

static int
mt7601u_rf_rmw(struct mt7601u_dev *dev, u8 bank, u8 offset, u8 mask, u8 val)
{
	int ret;

	ret = mt7601u_rf_rr(dev, bank, offset);
	if (ret < 0)
		return ret;
	val |= ret & ~mask;
	ret = mt7601u_rf_wr(dev, bank, offset, val);
	if (ret)
		return ret;

	return val;
}

static int
mt7601u_rf_set(struct mt7601u_dev *dev, u8 bank, u8 offset, u8 val)
{
	return mt7601u_rf_rmw(dev, bank, offset, 0, val);
}

static int
mt7601u_rf_clear(struct mt7601u_dev *dev, u8 bank, u8 offset, u8 mask)
{
	return mt7601u_rf_rmw(dev, bank, offset, mask, 0);
}

static void mt7601u_bbp_wr(struct mt7601u_dev *dev, u8 offset, u8 val)
{
	if (!test_bit(MT7601U_STATE_WLAN_RUNNING, &dev->state))
		return;

	if (!mt76_poll(dev, MT_BBP_CSR_CFG, MT_BBP_CSR_CFG_BUSY, 0, 1000)) {
		mt_err(dev, "BBP write %02x failed!!\n", (unsigned int)offset);
		return;
	}

	mt7601u_wr(dev, MT_BBP_CSR_CFG,
		   FIELD_PREP(MT_BBP_CSR_CFG_VAL, val) |
		   FIELD_PREP(MT_BBP_CSR_CFG_REG_NUM, offset) |
		   MT_BBP_CSR_CFG_RW_MODE | MT_BBP_CSR_CFG_BUSY);
}

void mt7601u_write_bbp_pairs(struct mt7601u_dev *dev,
			     const struct mt76_reg_pair *data, int n)
{
	int i;

	for (i = 0; i < n; i++)
		mt7601u_bbp_wr(dev, data[i].reg, data[i].value);
}

static int mt7601u_bbp_rr(struct mt7601u_dev *dev, u8 offset)
{
	u32 val;
	int ret = -ETIMEDOUT;

	if (!test_bit(MT7601U_STATE_WLAN_RUNNING, &dev->state))
		return -EINVAL;

	if (!mt76_poll(dev, MT_BBP_CSR_CFG, MT_BBP_CSR_CFG_BUSY, 0, 1000))
		goto out;

	mt7601u_wr(dev, MT_BBP_CSR_CFG,
		   FIELD_PREP(MT_BBP_CSR_CFG_REG_NUM, offset) |
		   MT_BBP_CSR_CFG_RW_MODE | MT_BBP_CSR_CFG_BUSY |
		   MT_BBP_CSR_CFG_READ);

	if (!mt76_poll(dev, MT_BBP_CSR_CFG, MT_BBP_CSR_CFG_BUSY, 0, 1000))
		goto out;

	val = mt7601u_rr(dev, MT_BBP_CSR_CFG);
	if (FIELD_GET(MT_BBP_CSR_CFG_REG_NUM, val) == offset)
		ret = FIELD_GET(MT_BBP_CSR_CFG_VAL, val);
out:
	if (ret < 0) {
		dev->reg_failed = true;
		mt_err(dev, "BBP read %02x failed:%d!!\n",
		       (unsigned int)offset, ret);
	}

	return ret;
}

void mt7601u_restore_rf5_0(struct mt7601u_dev *dev)
{
	int before = mt7601u_rf_rr(dev, 5, 0);
	int wr = mt7601u_rf_wr(dev, 5, 0, 0x47);
	int after = mt7601u_rf_rr(dev, 5, 0);

	if (before < 0 || after < 0)
		printf("mt7601u: rf5/0 restore before %d wr %d after %d\n",
		       before, wr, after);
	else
		printf("mt7601u: rf5/0 restore before %02x wr %d after %02x\n",
		       before, wr, after);
}

void mt7601u_print_cal_reg(struct mt7601u_dev *dev)
{
	int v = mt7601u_rf_rr(dev, 5, 0);

	if (v < 0)
		printf("mt7601u: cal-reg ch %u RF5/0=err %d\n", dev->chan, v);
	else
		printf("mt7601u: cal-reg ch %u RF5/0=%02x\n", dev->chan, v);
}

static int mt7601u_bbp_rmw(struct mt7601u_dev *dev, u8 offset, u8 mask, u8 val)
{
	int ret;

	ret = mt7601u_bbp_rr(dev, offset);
	if (ret < 0)
		return ret;
	val |= ret & ~mask;
	mt7601u_bbp_wr(dev, offset, val);

	return val;
}

static u8 mt7601u_bbp_rmc(struct mt7601u_dev *dev, u8 offset, u8 mask, u8 val)
{
	int ret;

	ret = mt7601u_bbp_rr(dev, offset);
	if (ret < 0)
		return ret;
	val |= ret & ~mask;
	if (ret != val)
		mt7601u_bbp_wr(dev, offset, val);

	return val;
}

int mt7601u_wait_bbp_ready(struct mt7601u_dev *dev)
{
	int i = 20;
	int raw;
	u8 val;

	do {
		raw = mt7601u_bbp_rr(dev, MT_BBP_REG_VERSION);
		val = raw;
		if (val && val != 0xff)
			break;
	} while (--i);

	printf("mt7601u: bbp ver raw=%d tries_left=%d\n", raw, i);

	if (!i) {
		printf("mt7601u: BBP is not ready\n");
		return -EIO;
	}

	return 0;
}

void mt7601u_print_bbp_init_regs(struct mt7601u_dev *dev)
{
	printf("mt7601u: bbp init r1=%d r65=%d\n",
	       mt7601u_bbp_rr(dev, 1), mt7601u_bbp_rr(dev, 65));
}

u32 mt7601u_bbp_set_ctrlch(struct mt7601u_dev *dev, bool below)
{
	return mt7601u_bbp_rmc(dev, 3, 0x20, below ? 0x20 : 0);
}

int mt7601u_phy_get_rssi(struct mt7601u_dev *dev,
			 const struct mt7601u_rxwi *rxwi, u16 rate)
{
	static const s8 lna[2][2][3] = {
		/* main LNA */ {
			/* bw20 */ { -2, 15, 33 },
			/* bw40 */ {  0, 16, 34 }
		},
		/*  aux LNA */ {
			/* bw20 */ { -2, 15, 33 },
			/* bw40 */ { -2, 16, 34 }
		}
	};
	int bw = FIELD_GET(MT_RXWI_RATE_BW, rate);
	int aux_lna = FIELD_GET(MT_RXWI_ANT_AUX_LNA, rxwi->ant);
	int lna_id = FIELD_GET(MT_RXWI_GAIN_RSSI_LNA_ID, rxwi->gain);
	int val;

	if (lna_id) /* LNA id can be 0, 2, 3. */
		lna_id--;

	val = 8;
	val -= lna[aux_lna][bw][lna_id];
	val -= FIELD_GET(MT_RXWI_GAIN_RSSI_VAL, rxwi->gain);
	val -= dev->ee->lna_gain;
	val -= dev->ee->rssi_offset[0];

	return val;
}

/*
 * @settle: true = msleep(2) as Linux; false = the caller guarantees >= 2 ms
 * pass before the next RF access (the scan cyclic's next tick).
 */
static void mt7601u_vco_cal(struct mt7601u_dev *dev, bool settle)
{
	mt7601u_rf_wr(dev, 0, 4, 0x0a);
	mt7601u_rf_wr(dev, 0, 5, 0x20);
	mt7601u_rf_set(dev, 0, 4, BIT(7));
	if (settle)
		mdelay(2);
}

static int mt7601u_set_bw_filter_dir(struct mt7601u_dev *dev, bool cal,
				     bool tx)
{
	u32 filter = 0;

	if (!cal)
		filter |= 0x10000;
	if (dev->bw != MT_BW_20)
		filter |= 0x00100;

	return mt7601u_mcu_calibrate(dev, MCU_CAL_BW, filter | (tx ? 1 : 0));
}

static int mt7601u_set_bw_filter(struct mt7601u_dev *dev, bool cal)
{
	int ret;

	/* TX */
	ret = mt7601u_set_bw_filter_dir(dev, cal, true);
	if (ret)
		return ret;
	/* RX */
	return mt7601u_set_bw_filter_dir(dev, cal, false);
}

static int mt7601u_load_bbp_temp_table_bw(struct mt7601u_dev *dev)
{
	const struct reg_table *t;

	if (dev->temp_mode > MT_TEMP_MODE_LOW)
		return -EINVAL;

	t = &bbp_mode_table[dev->temp_mode][dev->bw];

	return mt7601u_write_reg_pairs(dev, MT_MCU_MEMMAP_BBP, t->regs, t->n);
}

static int mt7601u_bbp_temp(struct mt7601u_dev *dev, int mode, const char *name)
{
	const struct reg_table *t;
	int ret;

	if (dev->temp_mode == mode)
		return 0;

	dev->temp_mode = mode;
	debug("mt7601u: temp mode %s\n", name);

	t = bbp_mode_table[dev->temp_mode];
	ret = mt7601u_write_reg_pairs(dev, MT_MCU_MEMMAP_BBP,
				      t[2].regs, t[2].n);
	if (ret)
		return ret;

	return mt7601u_write_reg_pairs(dev, MT_MCU_MEMMAP_BBP,
				       t[dev->bw].regs, t[dev->bw].n);
}

#define FREQ_PLAN_REGS	4
static const u8 freq_plan[14][FREQ_PLAN_REGS] = {
	{ 0x99,	0x99,	0x09,	0x50 },
	{ 0x46,	0x44,	0x0a,	0x50 },
	{ 0xec,	0xee,	0x0a,	0x50 },
	{ 0x99,	0x99,	0x0b,	0x50 },
	{ 0x46,	0x44,	0x08,	0x51 },
	{ 0xec,	0xee,	0x08,	0x51 },
	{ 0x99,	0x99,	0x09,	0x51 },
	{ 0x46,	0x44,	0x0a,	0x51 },
	{ 0xec,	0xee,	0x0a,	0x51 },
	{ 0x99,	0x99,	0x0b,	0x51 },
	{ 0x46,	0x44,	0x08,	0x52 },
	{ 0xec,	0xee,	0x08,	0x52 },
	{ 0x99,	0x99,	0x09,	0x52 },
	{ 0x33,	0x33,	0x0b,	0x52 },
};

/*
 * __mt7601u_phy_set_channel() for a 20 MHz channel as a state machine
 * (enum mt_hop_stage), one stage per call. No stage waits: an MCU command is
 * sent in one stage and its response polled in the next stage(s) with a
 * short xHCI window, and a register access whose busy-poll does not clear at
 * once (reg_failed) makes the stage retry on the next call.
 *
 * The bw/ctrl-channel change branch of the Linux function never runs here:
 * the port is fixed at 20 MHz (dev->bw == MT_BW_20 since init) and
 * chan_ext_below stays false, so mt7601u_bbp_set_bw() reduces to its rmc.
 */
static const char * const hop_stage_names[__HOP_MAX] = {
	[HOP_DONE]		= "rx",
	[HOP_RF_PLAN_SEND]	= "rf-plan send",
	[HOP_RF_PLAN_RESP]	= "rf-plan resp",
	[HOP_TX_ALC]		= "tx-alc rmw",
	[HOP_BBP_SEND]		= "bbp62-64 send",
	[HOP_BBP_RESP]		= "bbp62-64 resp",
	[HOP_VCO_A]		= "vco rf0/4",
	[HOP_VCO_B]		= "vco rf0/5",
	[HOP_VCO_C]		= "vco rf0/4 set",
	[HOP_BW]		= "bbp4 bw rmc",
	[HOP_BWF_TX_SEND]	= "bwf-tx send",
	[HOP_BWF_TX_RESP]	= "bwf-tx resp",
	[HOP_BWF_RX_SEND]	= "bwf-rx send",
	[HOP_BWF_RX_RESP]	= "bwf-rx resp",
	[HOP_CH14_A]		= "ch14 bbp4",
	[HOP_CH14_B]		= "ch14 bbp178",
	[HOP_PWR]		= "tx-pwr-cfg0",
	[HOP_AGC]		= "agc reset",
	[HOP_VFY_0]		= "verify rf0/17",
	[HOP_VFY_1]		= "verify rf0/18",
	[HOP_VFY_2]		= "verify rf0/19",
	[HOP_VFY_3]		= "verify rf0/20",
};

/* "99 99 09 50": the RF 0/17..20 values the freq plan writes for @hw_chan */
const char *mt7601u_freq_plan_str(int hw_chan)
{
	static char buf[16];
	const u8 *p;

	if (hw_chan < 1 || hw_chan > 14)
		return "?";
	p = freq_plan[hw_chan - 1];
	snprintf(buf, sizeof(buf), "%02x %02x %02x %02x", p[0], p[1], p[2],
		 p[3]);
	return buf;
}

const char *mt7601u_hop_stage_name(int stage)
{
	if (stage < 0 || stage >= __HOP_MAX)
		return "?";
	return hop_stage_names[stage];
}

/* grouping of the stages into the Linux function's five steps */
int mt7601u_hop_linux_step(int stage)
{
	if (stage >= HOP_RF_PLAN_SEND && stage <= HOP_TX_ALC)
		return 1;
	if (stage >= HOP_BBP_SEND && stage <= HOP_VCO_C)
		return 2;
	if (stage >= HOP_BW && stage <= HOP_BWF_TX_RESP)
		return 3;
	if (stage >= HOP_BWF_RX_SEND && stage <= HOP_BWF_RX_RESP)
		return 4;
	if (stage >= HOP_CH14_A && stage <= HOP_AGC)
		return 5;
	if (stage >= HOP_VFY_0 && stage <= HOP_VFY_3)
		return 6;	/* readback, not in the Linux function */
	return 0;
}

void mt7601u_hop_begin(struct mt7601u_dev *dev, int hw_chan)
{
	dev->hop_chan = hw_chan;
	dev->hop_stage = HOP_RF_PLAN_SEND;
	dev->hop_retries = 0;
}

/* after a register stage: retry on reg_failed (bounded), else advance */
static int hop_reg_done(struct mt7601u_dev *dev, int next)
{
	if (dev->reg_failed) {
		dev->reg_failed = false;
		if (++dev->hop_retries < MT_HOP_REG_RETRIES)
			return -EINPROGRESS;	/* same stage next call */
		dev->hop_retries = 0;
		dev->hop_stage = next;
		return -ETIMEDOUT;
	}

	dev->hop_retries = 0;
	dev->hop_stage = next;
	return 0;
}

/*
 * After an MCU send attempt. A failed bulk OUT leaves the stage in place:
 * the next call re-sends the same command with the same seq, up to
 * MT_MCU_SEND_ATTEMPTS attempts; then the error is counted and the response
 * stage skipped.
 */
static int hop_mcu_sent(struct mt7601u_dev *dev, int ret, int resp_stage,
			int skip_stage)
{
	if (!ret) {
		dev->mcu_out_fails = 0;
		dev->hop_retries = 0;
		dev->hop_stage = resp_stage;
		return 0;
	}

	/* consecutive failed command OUTs: the scan tick stops the scan once
	 * the MCU has clearly stopped accepting commands */
	dev->mcu_out_fails++;

	if (dev->hop_retries + 1 < MT_MCU_SEND_ATTEMPTS) {
		dev->hop_retries++;
		return -EINPROGRESS;
	}

	dev->hop_retries = 0;
	dev->hop_stage = skip_stage;
	return ret;
}

/* one poll of the response; give up MT_MCU_RESP_DEADLINE_MS after send */
static int hop_mcu_resp(struct mt7601u_dev *dev, int next)
{
	int ret = mt7601u_mcu_poll_resp(dev, MT_MCU_RESP_WINDOW_US);

	if (!ret || ret == -ENOENT) {
		dev->hop_stage = next;
		return 0;
	}

	/* a late reply to an earlier command: consumed, keep waiting */
	if (ret == -EALREADY)
		ret = -EINPROGRESS;

	if (get_timer(dev->mcu_sent_at) < MT_MCU_RESP_DEADLINE_MS)
		return ret;	/* -EINPROGRESS, or -EPROTO/-EIO counted */

	dev->mcu_pending = false;
	dev->hop_stage = next;
	return -ETIMEDOUT;
}

/*
 * Run the current stage once. Return: 0 = stage done (dev->hop_stage
 * advanced), -EINPROGRESS = waiting / retrying, nothing to count, other
 * negative = a poll or send error. The caller counts it only when this
 * call advanced hop_stage. A poll error that leaves the stage unchanged
 * is still pending.
 */
int mt7601u_hop_step(struct mt7601u_dev *dev)
{
	struct mt7601u_rate_power *t = &dev->ee->power_rate_table;
	int hw_chan = dev->hop_chan;
	int chan_idx = hw_chan - 1;
	u32 filter;
	int i, ret;

	if (hw_chan < 1 || hw_chan > 14) {
		dev->hop_stage = HOP_DONE;
		return -EINVAL;
	}

	dev->reg_failed = false;

	switch (dev->hop_stage) {
	case HOP_RF_PLAN_SEND: {
		struct mt76_reg_pair channel_freq_plan[FREQ_PLAN_REGS] = {
			{ 17, 0 }, { 18, 0 }, { 19, 0 }, { 20, 0 },
		};

		for (i = 0; i < FREQ_PLAN_REGS; i++)
			channel_freq_plan[i].value = freq_plan[chan_idx][i];

		ret = mt7601u_mcu_reg_pairs_send(dev, MT_MCU_MEMMAP_RF,
						 channel_freq_plan,
						 FREQ_PLAN_REGS,
						 dev->hop_retries > 0);
		return hop_mcu_sent(dev, ret, HOP_RF_PLAN_RESP, HOP_TX_ALC);
	}
	case HOP_RF_PLAN_RESP:
		return hop_mcu_resp(dev, HOP_TX_ALC);
	case HOP_TX_ALC:
		mt7601u_rmw(dev, MT_TX_ALC_CFG_0, 0x3f3f,
			    dev->ee->chan_pwr[chan_idx] & 0x3f);
		return hop_reg_done(dev, HOP_BBP_SEND);
	case HOP_BBP_SEND: {
		struct mt76_reg_pair bbp_settings[3] = {
			{ 62, 0x37 - dev->ee->lna_gain },
			{ 63, 0x37 - dev->ee->lna_gain },
			{ 64, 0x37 - dev->ee->lna_gain },
		};

		ret = mt7601u_mcu_reg_pairs_send(dev, MT_MCU_MEMMAP_BBP,
						 bbp_settings,
						 ARRAY_SIZE(bbp_settings),
						 dev->hop_retries > 0);
		return hop_mcu_sent(dev, ret, HOP_BBP_RESP, HOP_VCO_A);
	}
	case HOP_BBP_RESP:
		return hop_mcu_resp(dev, HOP_VCO_A);
	/* mt7601u_vco_cal(); its msleep(2) is the gap to the next stage */
	case HOP_VCO_A:
		mt7601u_rf_wr(dev, 0, 4, 0x0a);
		return hop_reg_done(dev, HOP_VCO_B);
	case HOP_VCO_B:
		mt7601u_rf_wr(dev, 0, 5, 0x20);
		return hop_reg_done(dev, HOP_VCO_C);
	case HOP_VCO_C:
		mt7601u_rf_set(dev, 0, 4, BIT(7));
		return hop_reg_done(dev, HOP_BW);
	case HOP_BW:
		mt7601u_bbp_rmc(dev, 4, 0x18, dev->bw == MT_BW_20 ? 0 : 0x10);
		return hop_reg_done(dev, HOP_BWF_TX_SEND);
	/* mt7601u_set_bw_filter(dev, false) */
	case HOP_BWF_TX_SEND:
	case HOP_BWF_RX_SEND:
		filter = 0x10000;
		if (dev->bw != MT_BW_20)
			filter |= 0x00100;
		if (dev->hop_stage == HOP_BWF_TX_SEND) {
			ret = mt7601u_mcu_calibrate_send(dev, MCU_CAL_BW,
							 filter | 1,
							 dev->hop_retries > 0);
			return hop_mcu_sent(dev, ret, HOP_BWF_TX_RESP,
					    HOP_BWF_RX_SEND);
		}
		ret = mt7601u_mcu_calibrate_send(dev, MCU_CAL_BW, filter,
						 dev->hop_retries > 0);
		return hop_mcu_sent(dev, ret, HOP_BWF_RX_RESP, HOP_CH14_A);
	case HOP_BWF_TX_RESP:
		return hop_mcu_resp(dev, HOP_BWF_RX_SEND);
	case HOP_BWF_RX_RESP:
		return hop_mcu_resp(dev, HOP_CH14_A);
	/* mt7601u_apply_ch14_fixup() */
	case HOP_CH14_A:
		if (hw_chan != 14 || dev->bw != MT_BW_20)
			mt7601u_bbp_rmw(dev, 4, 0x20, 0);
		else
			mt7601u_bbp_wr(dev, 4, 0x60);
		return hop_reg_done(dev, HOP_CH14_B);
	case HOP_CH14_B:
		if (hw_chan != 14 || dev->bw != MT_BW_20) {
			mt7601u_bbp_wr(dev, 178, 0xff);
			t->cck[0].bw20 = dev->ee->real_cck_bw20[0];
			t->cck[1].bw20 = dev->ee->real_cck_bw20[1];
		} else {
			mt7601u_bbp_wr(dev, 178, 0);
			/* Note: vendor code is buggy here for negative values */
			t->cck[0].bw20 = dev->ee->real_cck_bw20[0] - 2;
			t->cck[1].bw20 = dev->ee->real_cck_bw20[1] - 2;
		}
		return hop_reg_done(dev, HOP_PWR);
	case HOP_PWR:
		mt7601u_wr(dev, MT_TX_PWR_CFG_0,
			   int_to_s6(t->ofdm[1].bw20) << 24 |
			   int_to_s6(t->ofdm[0].bw20) << 16 |
			   int_to_s6(t->cck[1].bw20) << 8 |
			   int_to_s6(t->cck[0].bw20));
		return hop_reg_done(dev, HOP_AGC);
	case HOP_AGC:
		if (test_bit(MT7601U_STATE_SCANNING, &dev->state))
			mt7601u_agc_reset(dev);
		return hop_reg_done(dev, HOP_VFY_0);
	/* read back the RF registers the freq plan wrote (bank 0, 17..20) */
	case HOP_VFY_0:
	case HOP_VFY_1:
	case HOP_VFY_2:
	case HOP_VFY_3: {
		int k = dev->hop_stage - HOP_VFY_0;
		int next = dev->hop_stage + 1;

		ret = mt7601u_rf_rr(dev, 0, 17 + k);
		dev->hop_rb[k] = ret < 0 ? 0xee : ret;
		if (dev->hop_stage == HOP_VFY_3)
			next = HOP_DONE;
		ret = hop_reg_done(dev, next);
		if (dev->hop_stage == HOP_DONE) {
			dev->hop_verified = !memcmp(dev->hop_rb,
						    freq_plan[chan_idx],
						    FREQ_PLAN_REGS);
			dev->chan = hw_chan;
		}
		return ret;
	}
	}

	dev->hop_stage = HOP_DONE;
	return -EINVAL;
}

/*
 * mt7601u_phy_set_channel(): the whole switch synchronously (outside the
 * cyclic), through the same state machine. Latch the first error from a
 * call that advanced hop_stage. A poll error that leaves the stage
 * unchanged is still pending, same as -EINPROGRESS. HOP_DONE with
 * nothing latched returns 0.
 * Gives up after MT_SCAN_MCU_DEAD_SENDS consecutive failed command OUTs, as
 * the scan tick does: MT_MCU_SEND_ATTEMPTS x a 500 ms OUT per send stage
 * would otherwise hold the console for minutes on a dead MCU.
 */
int mt7601u_phy_set_channel(struct mt7601u_dev *dev, int hw_chan)
{
	int stage, ret, first = 0;

	dev->mcu_out_fails = 0;
	mt7601u_hop_begin(dev, hw_chan);
	while (dev->hop_stage != HOP_DONE) {
		if (dev->mcu_out_fails >= MT_SCAN_MCU_DEAD_SENDS) {
			printf("mt7601u: set channel %d: MCU not accepting commands (%u failed sends, stage %d) -- cold-start the dongle\n",
			       hw_chan, dev->mcu_out_fails, dev->hop_stage);
			dev->hop_retries = 0;
			dev->hop_stage = HOP_DONE;
			return -EIO;
		}
		stage = dev->hop_stage;
		ret = mt7601u_hop_step(dev);
		/* -EPROTO from the response poll leaves hop_stage unchanged.
		 * The reply is still pending. Do not latch that.
		 */
		if (ret && ret != -EINPROGRESS && !first &&
		    dev->hop_stage != stage)
			first = ret;
		if (stage == HOP_VCO_C && dev->hop_stage != stage)
			mdelay(2);	/* mt7601u_vco_cal() msleep(2) */
		else if (ret == -EINPROGRESS ||
			 (ret && dev->hop_stage == stage))
			udelay(100);
	}

	if (first)
		mt_err(dev, "set channel %d: error %d\n", hw_chan, first);
	return first;
}

#define BBP_R47_FLAG		GENMASK(2, 0)
#define BBP_R47_F_TSSI		0
#define BBP_R47_F_PKT_T		1
#define BBP_R47_F_TX_RATE	2
#define BBP_R47_F_TEMP		4

static u8 mt7601u_bbp_r47_get(struct mt7601u_dev *dev, u8 reg, u8 flag)
{
	flag |= reg & ~BBP_R47_FLAG;
	mt7601u_bbp_wr(dev, 47, flag);
	udelay(600);
	return mt7601u_bbp_rr(dev, 49);
}

static s8 mt7601u_read_bootup_temp(struct mt7601u_dev *dev)
{
	u8 bbp_val, temp;
	u32 rf_bp, rf_set;
	int i;

	rf_set = mt7601u_rr(dev, MT_RF_SETTING_0);
	rf_bp = mt7601u_rr(dev, MT_RF_BYPASS_0);

	mt7601u_wr(dev, MT_RF_BYPASS_0, 0);
	mt7601u_wr(dev, MT_RF_SETTING_0, 0x00000010);
	mt7601u_wr(dev, MT_RF_BYPASS_0, 0x00000010);

	bbp_val = mt7601u_bbp_rmw(dev, 47, 0, 0x10);

	mt7601u_bbp_wr(dev, 22, 0x40);

	for (i = 100; i && (bbp_val & 0x10); i--)
		bbp_val = mt7601u_bbp_rr(dev, 47);

	temp = mt7601u_bbp_r47_get(dev, bbp_val, BBP_R47_F_TEMP);

	mt7601u_bbp_wr(dev, 22, 0);

	bbp_val = mt7601u_bbp_rr(dev, 21);
	bbp_val |= 0x02;
	mt7601u_bbp_wr(dev, 21, bbp_val);
	bbp_val &= ~0x02;
	mt7601u_bbp_wr(dev, 21, bbp_val);

	mt7601u_wr(dev, MT_RF_BYPASS_0, 0);
	mt7601u_wr(dev, MT_RF_SETTING_0, rf_set);
	mt7601u_wr(dev, MT_RF_BYPASS_0, rf_bp);

	return temp;
}

static void mt7601u_rxdc_cal(struct mt7601u_dev *dev)
{
	static const struct mt76_reg_pair intro[] = {
		{ 158, 0x8d }, { 159, 0xfc },
		{ 158, 0x8c }, { 159, 0x4c },
	}, outro[] = {
		{ 158, 0x8d }, { 159, 0xe0 },
	};
	u32 mac_ctrl;
	int i, ret;

	mac_ctrl = mt7601u_rr(dev, MT_MAC_SYS_CTRL);
	mt7601u_wr(dev, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX);

	ret = mt7601u_write_reg_pairs(dev, MT_MCU_MEMMAP_BBP,
				      intro, ARRAY_SIZE(intro));
	if (ret)
		printf("mt7601u: %s intro failed:%d\n", __func__, ret);

	for (i = 20; i; i--) {
		udelay(400);

		mt7601u_bbp_wr(dev, 158, 0x8c);
		if (mt7601u_bbp_rr(dev, 159) == 0x0c)
			break;
	}
	if (!i) {
		int r158 = mt7601u_bbp_rr(dev, 158);
		int r159 = mt7601u_bbp_rr(dev, 159);

		printf("mt7601u: %s timed out bbp158=%02x bbp159=%02x\n",
		       __func__, r158, r159);
	}

	mt7601u_wr(dev, MT_MAC_SYS_CTRL, 0);

	ret = mt7601u_write_reg_pairs(dev, MT_MCU_MEMMAP_BBP,
				      outro, ARRAY_SIZE(outro));
	if (ret)
		printf("mt7601u: %s outro failed:%d\n", __func__, ret);

	mt7601u_wr(dev, MT_MAC_SYS_CTRL, mac_ctrl);
}

/* Note: function copied from vendor driver */
static s16 lin2dBd(u16 linear)
{
	short exp = 0;
	unsigned int mantisa;
	int app, dBd;

	if (!linear)
		return -10000;

	mantisa = linear;

	exp = fls(mantisa) - 16;
	if (exp > 0)
		mantisa >>= exp;
	else
		mantisa <<= abs(exp);

	if (mantisa <= 0xb800)
		app = (mantisa + (mantisa >> 3) + (mantisa >> 4) - 0x9600);
	else
		app = (mantisa - (mantisa >> 3) - (mantisa >> 6) - 0x5a00);
	if (app < 0)
		app = 0;

	dBd = ((15 + exp) << 15) + app;
	dBd = (dBd << 2) + (dBd << 1) + (dBd >> 6) + (dBd >> 7);
	dBd = (dBd >> 10);

	return dBd;
}

static void
mt7601u_set_initial_tssi(struct mt7601u_dev *dev, s16 tssi_db, s16 tssi_hvga_db)
{
	struct tssi_data *d = &dev->ee->tssi_data;
	int init_offset;

	init_offset = -((tssi_db * d->slope + d->offset[1]) / 4096) + 10;

	mt76_rmw(dev, MT_TX_ALC_CFG_1, MT_TX_ALC_CFG_1_TEMP_COMP,
		 int_to_s6(init_offset) & MT_TX_ALC_CFG_1_TEMP_COMP);
}

static void mt7601u_tssi_dc_gain_cal(struct mt7601u_dev *dev)
{
	u8 rf_vga, rf_mixer, bbp_r47;
	int i, j;
	s8 res[4];
	s16 tssi_init_db, tssi_init_hvga_db;

	mt7601u_wr(dev, MT_RF_SETTING_0, 0x00000030);
	mt7601u_wr(dev, MT_RF_BYPASS_0, 0x000c0030);
	mt7601u_wr(dev, MT_MAC_SYS_CTRL, 0);

	mt7601u_bbp_wr(dev, 58, 0);
	mt7601u_bbp_wr(dev, 241, 0x2);
	mt7601u_bbp_wr(dev, 23, 0x8);
	printf("mt7601u: bbp wr check r0=%d r1=%d r23=%02x r241=%02x\n",
	       mt7601u_bbp_rr(dev, 0), mt7601u_bbp_rr(dev, 1),
	       mt7601u_bbp_rr(dev, 23), mt7601u_bbp_rr(dev, 241));
	bbp_r47 = mt7601u_bbp_rr(dev, 47);

	/* Set VGA gain */
	rf_vga = mt7601u_rf_rr(dev, 5, 3);
	mt7601u_rf_wr(dev, 5, 3, 8);

	/* Mixer disable */
	rf_mixer = mt7601u_rf_rr(dev, 4, 39);
	mt7601u_rf_wr(dev, 4, 39, 0);

	for (i = 0; i < 4; i++) {
		mt7601u_rf_wr(dev, 4, 39, (i & 1) ? rf_mixer : 0);

		mt7601u_bbp_wr(dev, 23, (i < 2) ? 0x08 : 0x02);
		mt7601u_rf_wr(dev, 5, 3, (i < 2) ? 0x08 : 0x11);

		/* BBP TSSI initial and soft reset */
		mt7601u_bbp_wr(dev, 22, 0);
		mt7601u_bbp_wr(dev, 244, 0);

		mt7601u_bbp_wr(dev, 21, 1);
		udelay(1);
		mt7601u_bbp_wr(dev, 21, 0);

		/* TSSI measurement */
		mt7601u_bbp_wr(dev, 47, 0x50);
		mt7601u_bbp_wr(dev, (i & 1) ? 244 : 22, (i & 1) ? 0x31 : 0x40);

		for (j = 20; j; j--)
			if (!(mt7601u_bbp_rr(dev, 47) & 0x10))
				break;
		if (!j)
			printf("mt7601u: %s pass %d bbp47=%02x timed out\n",
			       __func__, i, mt7601u_bbp_rr(dev, 47));

		/* TSSI read */
		mt7601u_bbp_wr(dev, 47, 0x40);
		res[i] = mt7601u_bbp_rr(dev, 49);
	}
	printf("mt7601u: tssi res %02x %02x %02x %02x\n",
	       (u8)res[0], (u8)res[1], (u8)res[2], (u8)res[3]);

	tssi_init_db = lin2dBd((short)res[1] - res[0]);
	tssi_init_hvga_db = lin2dBd(((short)res[3] - res[2]) * 4);
	dev->tssi_init = res[0];
	dev->tssi_init_hvga = res[2];
	dev->tssi_init_hvga_offset_db = tssi_init_hvga_db - tssi_init_db;

	mt7601u_bbp_wr(dev, 22, 0);
	mt7601u_bbp_wr(dev, 244, 0);

	mt7601u_bbp_wr(dev, 21, 1);
	udelay(1);
	mt7601u_bbp_wr(dev, 21, 0);

	mt7601u_wr(dev, MT_RF_BYPASS_0, 0);
	mt7601u_wr(dev, MT_RF_SETTING_0, 0);

	mt7601u_rf_wr(dev, 5, 3, rf_vga);
	mt7601u_rf_wr(dev, 4, 39, rf_mixer);
	mt7601u_bbp_wr(dev, 47, bbp_r47);

	mt7601u_set_initial_tssi(dev, tssi_init_db, tssi_init_hvga_db);
}

static int mt7601u_temp_comp(struct mt7601u_dev *dev, bool on)
{
	int ret, temp, hi_temp = 400, lo_temp = -200;

	temp = (dev->raw_temp - dev->ee->ref_temp) * MT_EE_TEMPERATURE_SLOPE;
	dev->curr_temp = temp;

	/* DPD Calibration */
	if (temp - dev->dpd_temp > 450 || temp - dev->dpd_temp < -450) {
		dev->dpd_temp = temp;

		ret = mt7601u_mcu_calibrate(dev, MCU_CAL_DPD, dev->dpd_temp);
		if (ret)
			return ret;

		mt7601u_vco_cal(dev, true);
	}

	/* PLL Lock Protect */
	if (temp < -50 && !dev->pll_lock_protect) { /* < 20C */
		dev->pll_lock_protect =  true;

		mt7601u_rf_wr(dev, 4, 4, 6);
		mt7601u_rf_clear(dev, 4, 10, 0x30);
	} else if (temp > 50 && dev->pll_lock_protect) { /* > 30C */
		dev->pll_lock_protect = false;

		mt7601u_rf_wr(dev, 4, 4, 0);
		mt7601u_rf_rmw(dev, 4, 10, 0x30, 0x10);
	}

	if (on) {
		hi_temp -= 50;
		lo_temp -= 50;
	}

	/* BBP CR for H, L, N temperature */
	if (temp > hi_temp)
		return mt7601u_bbp_temp(dev, MT_TEMP_MODE_HIGH, "high");
	else if (temp > lo_temp)
		return mt7601u_bbp_temp(dev, MT_TEMP_MODE_NORMAL, "normal");
	else
		return mt7601u_bbp_temp(dev, MT_TEMP_MODE_LOW, "low");
}

static u8 mt7601u_agc_default(struct mt7601u_dev *dev)
{
	return (dev->ee->lna_gain - 8) * 2 + 0x34;
}

static void mt7601u_agc_reset(struct mt7601u_dev *dev)
{
	u8 agc = mt7601u_agc_default(dev);

	mt7601u_bbp_wr(dev, 66,	agc);
}

void mt7601u_agc_save(struct mt7601u_dev *dev)
{
	dev->agc_save = mt7601u_bbp_rr(dev, 66);
}

void mt7601u_agc_restore(struct mt7601u_dev *dev)
{
	mt7601u_bbp_wr(dev, 66, dev->agc_save);
}

static int mt7601u_init_cal(struct mt7601u_dev *dev)
{
	u32 mac_ctrl;
	int ret;

	dev->raw_temp = mt7601u_read_bootup_temp(dev);
	dev->curr_temp = (dev->raw_temp - dev->ee->ref_temp) *
		MT_EE_TEMPERATURE_SLOPE;
	dev->dpd_temp = dev->curr_temp;

	mac_ctrl = mt7601u_rr(dev, MT_MAC_SYS_CTRL);

	ret = mt7601u_mcu_calibrate(dev, MCU_CAL_R, 0);
	if (ret)
		return ret;

	ret = mt7601u_rf_rr(dev, 0, 4);
	if (ret < 0)
		return ret;
	ret |= 0x80;
	ret = mt7601u_rf_wr(dev, 0, 4, ret);
	if (ret)
		return ret;
	mdelay(2);

	ret = mt7601u_mcu_calibrate(dev, MCU_CAL_TXDCOC, 0);
	if (ret)
		return ret;

	mt7601u_rxdc_cal(dev);

	ret = mt7601u_set_bw_filter(dev, true);
	if (ret)
		return ret;
	ret = mt7601u_mcu_calibrate(dev, MCU_CAL_LOFT, 0);
	if (ret)
		return ret;
	ret = mt7601u_mcu_calibrate(dev, MCU_CAL_TXIQ, 0);
	if (ret)
		return ret;
	ret = mt7601u_mcu_calibrate(dev, MCU_CAL_RXIQ, 0);
	if (ret)
		return ret;
	ret = mt7601u_mcu_calibrate(dev, MCU_CAL_DPD, dev->dpd_temp);
	if (ret)
		return ret;

	mt7601u_rxdc_cal(dev);

	mt7601u_tssi_dc_gain_cal(dev);

	mt7601u_wr(dev, MT_MAC_SYS_CTRL, mac_ctrl);

	mt7601u_temp_comp(dev, true);

	return 0;
}

int mt7601u_bbp_set_bw(struct mt7601u_dev *dev, int bw)
{
	u32 val, old;

	if (bw == dev->bw) {
		/* Vendor driver does the rmc even when no change is needed. */
		mt7601u_bbp_rmc(dev, 4, 0x18, bw == MT_BW_20 ? 0 : 0x10);

		return 0;
	}
	dev->bw = bw;

	/* Stop MAC for the time of bw change */
	old = mt7601u_rr(dev, MT_MAC_SYS_CTRL);
	val = old & ~(MT_MAC_SYS_CTRL_ENABLE_TX | MT_MAC_SYS_CTRL_ENABLE_RX);
	mt7601u_wr(dev, MT_MAC_SYS_CTRL, val);
	mt76_poll(dev, MT_MAC_STATUS, MT_MAC_STATUS_TX | MT_MAC_STATUS_RX,
		  0, 500000);

	mt7601u_bbp_rmc(dev, 4, 0x18, bw == MT_BW_20 ? 0 : 0x10);

	mt7601u_wr(dev, MT_MAC_SYS_CTRL, old);

	return mt7601u_load_bbp_temp_table_bw(dev);
}

void mt7601u_set_rx_path(struct mt7601u_dev *dev, u8 path)
{
	mt7601u_bbp_rmw(dev, 3, 0x18, path << 3);
}

void mt7601u_set_tx_dac(struct mt7601u_dev *dev, u8 dac)
{
	mt7601u_bbp_rmc(dev, 1, 0x18, dac << 3);
}

int mt7601u_phy_init(struct mt7601u_dev *dev)
{
	int ret;

	dev->rf_pa_mode[0] = mt7601u_rr(dev, MT_RF_PA_MODE_CFG0);
	dev->rf_pa_mode[1] = mt7601u_rr(dev, MT_RF_PA_MODE_CFG1);

	ret = mt7601u_rf_wr(dev, 0, 12, dev->ee->rf_freq_off);
	if (ret)
		return ret;
	ret = mt7601u_write_reg_pairs(dev, 0, rf_central,
				      ARRAY_SIZE(rf_central));
	if (ret)
		return ret;
	ret = mt7601u_write_reg_pairs(dev, 0, rf_channel,
				      ARRAY_SIZE(rf_channel));
	if (ret)
		return ret;
	ret = mt7601u_write_reg_pairs(dev, 0, rf_vga, ARRAY_SIZE(rf_vga));
	if (ret)
		return ret;

	ret = mt7601u_init_cal(dev);
	if (ret)
		return ret;

	dev->prev_pwr_diff = 100;

	return 0;
}
