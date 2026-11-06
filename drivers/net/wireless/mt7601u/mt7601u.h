/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * U-Boot background Wi-Fi scan port of the receive half of the Linux
 * mt7601u driver (drivers/net/wireless/mediatek/mt7601u). RX only: the
 * mac80211, TX, power-save, aggregation and workqueue state is dropped.
 * EXPERIMENTAL -- see Kconfig WIFI_MT7601U.
 */

#ifndef MT7601U_H
#define MT7601U_H

#include <cyclic.h>
#include <usb.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "regs.h"
#include "mac.h"

#define MT_BBP_REG_VERSION		0x00

#define MT_USB_AGGR_SIZE_LIMIT		28 /* * 1024B */
#define MT_USB_AGGR_TIMEOUT		0x80 /* * 33ns */
#define MT_RX_URB_SIZE			(4096 << 3)

#define N_WCIDS				128

#define MT_EE_TEMPERATURE_SLOPE		39

#define MT7601U_FIRMWARE		"mt7601u.bin"

#define MT_VEND_REQ_MAX_RETRY		10
#define MT_VEND_REQ_TOUT_MS		300

#define MT_VEND_DEV_MODE_RESET		1

#define MT_VEND_BUF			sizeof(__le32)

enum mt_vendor_req {
	MT_VEND_DEV_MODE = 1,
	MT_VEND_WRITE = 2,
	MT_VEND_MULTI_READ = 7,
	MT_VEND_WRITE_FCE = 0x42,
};

enum mt_usb_ep_in {
	MT_EP_IN_PKT_RX,
	MT_EP_IN_CMD_RESP,
	__MT_EP_IN_MAX,
};

enum mt_usb_ep_out {
	MT_EP_OUT_INBAND_CMD,
	MT_EP_OUT_AC_BK,
	MT_EP_OUT_AC_BE,
	MT_EP_OUT_AC_VI,
	MT_EP_OUT_AC_VO,
	MT_EP_OUT_HCCA,
	__MT_EP_OUT_MAX,
};

enum mt_temp_mode {
	MT_TEMP_MODE_NORMAL,
	MT_TEMP_MODE_HIGH,
	MT_TEMP_MODE_LOW,
};

enum mt_bw {
	MT_BW_20,
	MT_BW_40,
};

enum {
	MT7601U_STATE_INITIALIZED,
	MT7601U_STATE_REMOVED,
	MT7601U_STATE_WLAN_RUNNING,
	MT7601U_STATE_MCU_RUNNING,
	MT7601U_STATE_SCANNING,
};

struct mt76_reg_pair {
	u32 reg;
	u32 value;
};

#include "eeprom.h"

/*
 * A 20 MHz channel switch (__mt7601u_phy_set_channel) as a state machine:
 * one stage per cyclic tick. Each stage is a few control transfers, one MCU
 * command send, or one check of the pending MCU-response TD. VFY_* read the
 * tuned RF registers back. "Step" is the Linux-function grouping, shown by
 * 'wifi scan show -v'.
 */
enum mt_hop_stage {
	HOP_DONE = 0,
	HOP_RF_PLAN_SEND,	/* step 1: RF freq plan, MCU random write */
	HOP_RF_PLAN_RESP,
	HOP_TX_ALC,		/* step 1: TX_ALC_CFG_0 channel power */
	HOP_BBP_SEND,		/* step 2: BBP 62-64, MCU random write */
	HOP_BBP_RESP,
	HOP_VCO_A,		/* step 2: mt7601u_vco_cal() RF 0/4 */
	HOP_VCO_B,		/* step 2: RF 0/5 */
	HOP_VCO_C,		/* step 2: RF 0/4 |= BIT(7) */
	HOP_BW,			/* step 3: mt7601u_bbp_set_bw() rmc BBP 4 */
	HOP_BWF_TX_SEND,	/* step 3: MCU_CAL_BW TX */
	HOP_BWF_TX_RESP,
	HOP_BWF_RX_SEND,	/* step 4: MCU_CAL_BW RX */
	HOP_BWF_RX_RESP,
	HOP_CH14_A,		/* step 5: ch14 fixup BBP 4 */
	HOP_CH14_B,		/* step 5: ch14 fixup BBP 178 */
	HOP_PWR,		/* step 5: TX_PWR_CFG_0 */
	HOP_AGC,		/* step 5: AGC reset (scanning) */
	HOP_VFY_0,		/* verify: read back RF bank 0 reg 17 */
	HOP_VFY_1,		/* reg 18 */
	HOP_VFY_2,		/* reg 19 */
	HOP_VFY_3,		/* reg 20, compare with the freq plan */
	__HOP_MAX,
};

#define MT_FREQ_PLAN_REGS		4

/* an MCU command whose OUT fails is re-sent up to this many ticks */
#define MT_MCU_SEND_ATTEMPTS		100
/*
 * Consecutive failed MCU command OUTs (each a 500 ms timeout) after which the
 * scan declares the MCU dead and stops itself: otherwise every tick blocks in a
 * doomed OUT, the console crawls and a typed 'reset' takes minutes to land.
 */
#define MT_SCAN_MCU_DEAD_SENDS		6
/* one pending-response check per tick: window and give-up deadline */
#define MT_MCU_RESP_WINDOW_US		1000
#define MT_MCU_RESP_DEADLINE_MS		500
/* a register stage whose access did not settle is retried this many ticks */
#define MT_HOP_REG_RETRIES		5

/* per-tick phase: 0 = RX poll+parse, otherwise an enum mt_hop_stage */
#define MT_PHASE_RX			0
#define MT_N_PHASES			__HOP_MAX

struct mt7601u_phase_stat {
	u32 count;
	u64 total_us;
	u64 max_us;
	u32 errors;
	int last_err;
};

/* per tuned channel (index 1..14) */
struct mt7601u_chan_stat {
	u32 switch_ok;		/* RF readback matched the freq plan */
	u32 switch_bad;		/* readback mismatch / unreadable */
	u32 switch_err;		/* switches that counted any error */
	u32 frames_on;		/* beacon/presp, DS param == tuned channel */
	u32 frames_off;		/* DS param != tuned channel */
	u32 frames_nods;	/* no DS param */
	u8 last_rb[MT_FREQ_PLAN_REGS];
};

#define MT_SCAN_MAX_BSS			32

struct mt7601u_bss {
	u8 bssid[6];
	u8 ssid_len;
	char ssid[33];
	u8 chan;
	s8 rssi;		/* last */
	s8 rssi_max;
	u32 beacons;
	u32 probe_resp;
	u8 akm_psk;		/* RSN AKM 00-0F-AC:2 */
	u8 akm_sae;		/* RSN AKM 00-0F-AC:8 */
	u8 pmf_req;		/* RSN capability MFPR */
};

struct mt7601u_scan {
	struct cyclic_info cyclic;
	bool running;
	u32 dwell_ms;
	u8 first_chan;
	u8 last_chan;

	ulong dwell_start;	/* get_timer() ms */
	u64 hop_start_us;

	/* statistics */
	u32 ticks;
	u32 busy_skips;
	u32 rx_xfers;
	u32 rx_empty;
	u32 rx_frames;
	u32 rx_crc;
	u64 rx_bytes;
	u32 rx_mgmt;
	bool mcu_dead;		/* stopped: MCU stopped accepting commands */
	u32 rx_nonpkt_seg;	/* non-packet segment on the packet-RX ep */
	u32 rx_last_nonpkt_fce;
	u32 rx_bad_tail;	/* bytes left that do not parse as a segment */
	u32 hops;
	u32 errors;
	int last_err;
	u64 tick_us_max;
	u64 tick_us_total;
	u64 hop_us_max;
	u64 hop_us_total;
	u32 cur_hop_errs;

	struct mt7601u_phase_stat phase[MT_N_PHASES];
	struct mt7601u_chan_stat chan[15];

	int n_bss;
	struct mt7601u_bss bss[MT_SCAN_MAX_BSS];
};

struct mt7601u_dev {
	struct usb_device *udev;
	unsigned long state;

	/*
	 * Re-entry guard: nonzero while a transfer issued OUTSIDE the scan
	 * cyclic is in flight (or a whole command sequence is running). The
	 * cyclic callback returns at once while it is set. in_cyclic also
	 * bounds register polls / vendor requests to one tick's worth.
	 */
	volatile int busy;
	bool in_cyclic;

	/*
	 * Set by any failed register access / register poll (vendor request
	 * error, mt76_poll timeout, RF/BBP read mismatch). A hop stage that
	 * sees it is retried on the next tick (bounded).
	 */
	bool reg_failed;

	/*
	 * xHCI async bulk TD handles (-1 = none) of the packet-RX and
	 * cmd-resp IN endpoints; gone = the controller forgot them (usb
	 * reset), so the scan stops without touching the usb_device again.
	 */
	int async_rx_h;
	int async_rx_h2;	/* second RX TD on the pkt-rx ep: the eth data
				 * path keeps both posted so the endpoint is
				 * never empty (bulk-RX DMA anti-wedge) */
	int async_resp_h;
	int async_resp_h2;	/* second cmd-resp TD: a re-sent command can
				 * draw back-to-back replies; with one TD the
				 * second has nowhere to land and the MCU then
				 * NAKs every command OUT (-110 flood) */
	u8 resp_next;		/* which cmd-resp TD completes next (0/1) */
	bool gone;
	u32 async_submit_err;
	int async_last_err;
	/* last stats per TD [0] pkt-rx [1] cmd-resp, kept after close */
	struct xhci_async_stats async_st[2];
	bool async_st_valid;

	/* USB_DMA_CFG read by 'wifi scan show' / 'wifi diag' */
	u32 last_usb_dma_cfg;

	/* channel-switch state machine */
	int hop_stage;		/* enum mt_hop_stage, HOP_DONE = idle */
	u8 hop_chan;
	u8 hop_retries;		/* reg-stage retries / send attempts - 1 */
	u8 hop_rb[MT_FREQ_PLAN_REGS];	/* RF 0/17..20 read back */
	bool hop_verified;	/* last switch: readback == freq plan */

	/* one outstanding async MCU command */
	bool mcu_pending;
	u8 mcu_pending_seq;
	u32 mcu_out_fails;	/* consecutive failed MCU command OUTs (hop) */
	u8 mcu_tx_seq;		/* seq of the last command sent (re-sends) */
	ulong mcu_sent_at;	/* get_timer() ms */
	u32 mcu_last_resp;	/* last raw RXFCE word read on CMD_RESP */
	u32 mcu_bad_resp;	/* error / unknown events */
	u32 mcu_last_bad;
	u32 mcu_late_resp;	/* CMD_DONE for a seq not waited on */
	u32 mcu_last_late;

	u8 out_eps[8];
	u8 in_eps[8];
	u16 out_max_packet;
	u16 in_max_packet;

	u8 macaddr[6];
	struct mt7601u_eeprom_params ee_store;
	struct mt7601u_eeprom_params *ee;

	u8 mcu_msg_seq;

	/* DMA-able buffers */
	u8 *vend_buf;
	u8 *cmd_buf;
	u8 *resp_buf;
	u8 *resp_buf2;		/* second cmd-resp TD buffer */
	u8 *fw_buf;
	u8 *rx_buf;
	u8 *rx_buf2;		/* second RX URB buffer: double-buffer the async
				 * bulk-IN so a TD is always posted (connect.c
				 * eth path) -- a gap with no posted buffer wedges
				 * the dongle's USB RX DMA (rx_busy stuck) */
	u8 *tx_buf;		/* one outgoing frame (connect.c), DMA-aligned */

	const u16 *beacon_offsets;

	u32 rxfilter;

	enum mt_temp_mode temp_mode;
	int curr_temp;
	int dpd_temp;
	s8 raw_temp;
	bool pll_lock_protect;

	s8 tssi_init;
	s8 tssi_init_hvga;
	s16 tssi_init_hvga_offset_db;
	int prev_pwr_diff;

	u8 bw;
	bool chan_ext_below;
	u8 chan;		/* current hw channel, 1..14 (0 = none set) */

	u8 agc_save;

	u32 rf_pa_mode[2];

	u32 asic_rev;
	u32 mac_rev;
	u16 fw_ver;
	u16 fw_build;

	struct mt7601u_scan scan;
};

#define MCU_RESP_URB_SIZE		1024
#define MCU_FW_URB_MAX_PAYLOAD		0x3800
#define MCU_FW_URB_SIZE			(MCU_FW_URB_MAX_PAYLOAD + 12)
#define MT_CMD_BUF_SIZE			256

/* usb.c */
int mt7601u_vendor_request(struct mt7601u_dev *dev, const u8 req,
			   const u8 direction, const u16 val, const u16 offset,
			   void *buf, const size_t buflen);
void mt7601u_vendor_reset(struct mt7601u_dev *dev);
int mt7601u_vendor_single_wr(struct mt7601u_dev *dev, const u8 req,
			     const u16 offset, const u32 val);
int mt7601u_bulk(struct mt7601u_dev *dev, bool in, int ep_idx, void *buf,
		 int len, int *actual, int timeout_ms);

/* No console output from inside the cyclic: a 40-char line is ~3.5 ms */
#define mt_err(dev, fmt, ...)						\
	do {								\
		if (!(dev)->in_cyclic)					\
			printf("mt7601u: " fmt, ##__VA_ARGS__);		\
	} while (0)

u32 mt7601u_rr(struct mt7601u_dev *dev, u32 offset);
void mt7601u_wr(struct mt7601u_dev *dev, u32 offset, u32 val);
u32 mt7601u_rmw(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val);
u32 mt7601u_rmc(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val);
int mt7601u_usb_find(struct mt7601u_dev *dev);
void mt7601u_diag(struct mt7601u_dev *dev, const char *why);

/* core.c equivalents (usb.c) */
int mt7601u_wait_asic_ready(struct mt7601u_dev *dev);
bool mt76_poll(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val,
	       int timeout);
bool mt76_poll_msec(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val,
		    int timeout);

static inline u32 mt76_rr(struct mt7601u_dev *dev, u32 offset)
{
	return mt7601u_rr(dev, offset);
}

static inline void mt76_wr(struct mt7601u_dev *dev, u32 offset, u32 val)
{
	mt7601u_wr(dev, offset, val);
}

static inline u32
mt76_rmw(struct mt7601u_dev *dev, u32 offset, u32 mask, u32 val)
{
	return mt7601u_rmw(dev, offset, mask, val);
}

static inline u32 mt76_set(struct mt7601u_dev *dev, u32 offset, u32 val)
{
	return mt76_rmw(dev, offset, 0, val);
}

static inline u32 mt76_clear(struct mt7601u_dev *dev, u32 offset, u32 val)
{
	return mt76_rmw(dev, offset, val, 0);
}

/* mcu.c */
int mt7601u_write_reg_pairs(struct mt7601u_dev *dev, u32 base,
			    const struct mt76_reg_pair *data, int len);
int mt7601u_burst_write_regs(struct mt7601u_dev *dev, u32 offset,
			     const u32 *data, int n);
int mt7601u_async_resp_arm(struct mt7601u_dev *dev);
int mt7601u_mcu_reg_pairs_send(struct mt7601u_dev *dev, u32 base,
			       const struct mt76_reg_pair *data, int n,
			       bool resend);
int mt7601u_mcu_calibrate_send(struct mt7601u_dev *dev, int cal, u32 val,
			       bool resend);
int mt7601u_mcu_poll_resp(struct mt7601u_dev *dev, unsigned int window_us);

/* init.c */
int mt7601u_init_hardware(struct mt7601u_dev *dev);
int mt7601u_mac_start_rx(struct mt7601u_dev *dev);
void mt7601u_stop_hardware(struct mt7601u_dev *dev);

/* phy.c */
int mt7601u_phy_init(struct mt7601u_dev *dev);
int mt7601u_wait_bbp_ready(struct mt7601u_dev *dev);
void mt7601u_print_bbp_init_regs(struct mt7601u_dev *dev);
void mt7601u_write_bbp_pairs(struct mt7601u_dev *dev,
			     const struct mt76_reg_pair *data, int n);
void mt7601u_set_rx_path(struct mt7601u_dev *dev, u8 path);
void mt7601u_set_tx_dac(struct mt7601u_dev *dev, u8 path);
int mt7601u_bbp_set_bw(struct mt7601u_dev *dev, int bw);
u32 mt7601u_bbp_set_ctrlch(struct mt7601u_dev *dev, bool below);
void mt7601u_agc_save(struct mt7601u_dev *dev);
void mt7601u_agc_restore(struct mt7601u_dev *dev);
int mt7601u_phy_set_channel(struct mt7601u_dev *dev, int hw_chan);
void mt7601u_restore_rf5_0(struct mt7601u_dev *dev);
void mt7601u_print_cal_reg(struct mt7601u_dev *dev);
void mt7601u_hop_begin(struct mt7601u_dev *dev, int hw_chan);
int mt7601u_hop_step(struct mt7601u_dev *dev);
int mt7601u_hop_linux_step(int stage);
const char *mt7601u_hop_stage_name(int stage);
const char *mt7601u_freq_plan_str(int hw_chan);
int mt7601u_phy_get_rssi(struct mt7601u_dev *dev,
			 const struct mt7601u_rxwi *rxwi, u16 rate);

static inline u32 mt7601u_mac_set_ctrlch(struct mt7601u_dev *dev, bool below)
{
	return mt7601u_rmc(dev, MT_TX_BAND_CFG, 1, below);
}

/* eeprom.c */
int mt7601u_eeprom_init(struct mt7601u_dev *dev);

#define MT_TX_BUF_SIZE			2048

/* scan.c */
int mt7601u_scan_start(struct mt7601u_dev *dev, u32 dwell_ms);
void mt7601u_scan_stop(struct mt7601u_dev *dev, bool finish);
void mt7601u_scan_show(struct mt7601u_dev *dev, bool verbose);

#endif
