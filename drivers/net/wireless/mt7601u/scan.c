// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * MT7601U background passive scan for U-Boot, driven by CONFIG_CYCLIC.
 *
 * The RX framing (DMA header, RXWI, payload, FCE info) and the segment walk
 * are ported from the Linux mt7601u driver
 * (drivers/net/wireless/mediatek/mt7601u/dma.c: mt7601u_rx_next_seg_len(),
 * mt7601u_rx_process_seg(), mt7601u_rx_skb_from_seg(); mac.c:
 * mt76_mac_process_rx()). Instead of URB queues and mac80211, one cyclic
 * function does, per tick, either ONE short bulk-IN on the packet-RX
 * endpoint or ONE step of a channel switch, and keeps beacons / probe
 * responses in a small BSS table. TEST ONLY.
 */

#include <cyclic.h>
#include <log.h>
#include <time.h>
#include <asm/unaligned.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "mt7601u.h"
#include "connect.h"
#include "dma.h"
#include "mac.h"

#define MT_SCAN_PERIOD_US		20000	/* dwelling: one RX poll */
#define MT_SCAN_HOP_PERIOD_US		5000	/* switching: one stage */
#define MT_SCAN_DEFAULT_DWELL_MS	300
#define MT_SCAN_FIRST_CHAN		1
#define MT_SCAN_LAST_CHAN		11

#define IEEE80211_FCTL_FTYPE		0x000c
#define IEEE80211_FCTL_STYPE		0x00f0
#define IEEE80211_FCTL_ORDER		0x8000
#define IEEE80211_FTYPE_MGMT		0x0000
#define IEEE80211_STYPE_PROBE_RESP	0x0050
#define IEEE80211_STYPE_BEACON		0x0080
#define WLAN_EID_SSID			0
#define WLAN_EID_DS_PARAMS		3
#define WLAN_EID_RSN			48

/* RSN AKM 00-0F-AC:2 is PSK, :8 is SAE. MFPR is capability bit 6. */
static void mt7601u_note_rsn(const u8 *body, u8 len, u8 *psk, u8 *sae, u8 *pmf)
{
	const u8 *p = body;
	const u8 *end = body + len;
	u16 n, cap;
	int i;

	if (len < 8)
		return;
	p += 6;				/* version + group cipher */
	if (p + 2 > end)
		return;
	n = get_unaligned_le16(p);
	p += 2;
	if (n > 8 || p + n * 4 > end)
		return;
	p += n * 4;
	if (p + 2 > end)
		return;
	n = get_unaligned_le16(p);
	p += 2;
	if (n > 8 || p + n * 4 > end)
		return;
	for (i = 0; i < n; i++) {
		if (p[0] == 0x00 && p[1] == 0x0f && p[2] == 0xac && p[3] == 2)
			*psk = 1;
		if (p[0] == 0x00 && p[1] == 0x0f && p[2] == 0xac && p[3] == 8)
			*sae = 1;
		p += 4;
	}
	if (p + 2 <= end) {
		cap = get_unaligned_le16(p);
		if (cap & 0x0040)
			*pmf = 1;
	}
}

static const char *mt7601u_bss_akm(const struct mt7601u_bss *b)
{
	if (b->akm_sae && b->akm_psk)
		return "psk+sae";
	if (b->akm_sae)
		return b->pmf_req ? "sae/pmf" : "sae";
	if (b->akm_psk)
		return "psk";
	return "-";
}

static struct mt7601u_bss *mt7601u_bss_get(struct mt7601u_scan *s,
					   const u8 *bssid)
{
	int i;

	for (i = 0; i < s->n_bss; i++)
		if (!memcmp(s->bss[i].bssid, bssid, 6))
			return &s->bss[i];

	if (s->n_bss >= MT_SCAN_MAX_BSS)
		return NULL;

	memset(&s->bss[s->n_bss], 0, sizeof(s->bss[0]));
	memcpy(s->bss[s->n_bss].bssid, bssid, 6);
	s->bss[s->n_bss].rssi_max = -128;
	return &s->bss[s->n_bss++];
}

/* Drop rows for the channel whose dwell is starting. Other channels stay. */
static void mt7601u_bss_drop_chan(struct mt7601u_scan *s, u8 chan)
{
	int i, w = 0;

	for (i = 0; i < s->n_bss; i++) {
		if (s->bss[i].chan == chan)
			continue;
		if (w != i)
			s->bss[w] = s->bss[i];
		w++;
	}
	if (w < s->n_bss)
		memset(&s->bss[w], 0, (s->n_bss - w) * sizeof(s->bss[0]));
	s->n_bss = w;
}

/* A beacon / probe response: 24 B header, 12 B fixed fields, then IEs */
static void mt7601u_scan_mgmt(struct mt7601u_dev *dev, const u8 *hdr,
			      const u8 *body, u32 body_len, int rssi, u16 stype)
{
	struct mt7601u_scan *s = &dev->scan;
	struct mt7601u_bss *bss;
	const u8 *ie, *end;
	u8 chan = dev->chan;
	const u8 *ssid = NULL;
	u8 ssid_len = 0;
	u8 akm_psk = 0, akm_sae = 0, pmf_req = 0;
	bool have_ds = false;

	if (body_len < 12)
		return;

	ie = body + 12;
	end = body + body_len;
	while (ie + 2 <= end && ie + 2 + ie[1] <= end) {
		if (ie[0] == WLAN_EID_SSID && ie[1] <= 32) {
			ssid = ie + 2;
			ssid_len = ie[1];
		} else if (ie[0] == WLAN_EID_DS_PARAMS && ie[1] >= 1) {
			chan = ie[2];
			have_ds = true;
		} else if (ie[0] == WLAN_EID_RSN) {
			mt7601u_note_rsn(ie + 2, ie[1], &akm_psk, &akm_sae,
					 &pmf_req);
		}
		ie += 2 + ie[1];
	}

	s->rx_mgmt++;
	if (dev->chan >= 1 && dev->chan <= 14) {
		struct mt7601u_chan_stat *cs = &s->chan[dev->chan];

		if (!have_ds)
			cs->frames_nods++;
		else if (chan == dev->chan)
			cs->frames_on++;
		else
			cs->frames_off++;
	}
	bss = mt7601u_bss_get(s, hdr + 16);	/* addr3 = BSSID */
	if (!bss)
		return;

	if (ssid && ssid_len && (ssid[0] || !bss->ssid_len)) {
		memcpy(bss->ssid, ssid, ssid_len);
		bss->ssid[ssid_len] = '\0';
		bss->ssid_len = ssid_len;
	}
	bss->chan = chan;
	bss->akm_psk |= akm_psk;
	bss->akm_sae |= akm_sae;
	bss->pmf_req |= pmf_req;
	bss->rssi = rssi;
	if (rssi > bss->rssi_max)
		bss->rssi_max = rssi;
	if (stype == IEEE80211_STYPE_BEACON)
		bss->beacons++;
	else
		bss->probe_resp++;
}

/* mt7601u_rx_process_seg() + mt7601u_rx_skb_from_seg() + mt76_mac_process_rx() */
static void mt7601u_rx_process_seg(struct mt7601u_dev *dev, u8 *data,
				   u32 seg_len)
{
	struct mt7601u_scan *s = &dev->scan;
	const struct mt7601u_rxwi *rxwi;
	u32 fce_info, ctl, rxinfo, len, hdr_len;
	u16 fc, rate;
	const u8 *body;
	int rssi;

	/* DMA_INFO at the start has only some of the info; FCE is at the end */
	fce_info = get_unaligned_le32(data + seg_len - MT_FCE_INFO_LEN);
	seg_len -= MT_FCE_INFO_LEN;

	data += MT_DMA_HDR_LEN;
	seg_len -= MT_DMA_HDR_LEN;

	rxwi = (const struct mt7601u_rxwi *)data;
	data += sizeof(struct mt7601u_rxwi);
	seg_len -= sizeof(struct mt7601u_rxwi);

	if (FIELD_GET(MT_RXD_INFO_TYPE, fce_info)) {
		/* non-packet segment on the RX ep (an MCU event?) */
		s->rx_nonpkt_seg++;
		s->rx_last_nonpkt_fce = fce_info;
		return;
	}

	s->rx_frames++;

	ctl = le32_to_cpu(rxwi->ctl);
	rxinfo = le32_to_cpu(rxwi->rxinfo);
	rate = le16_to_cpu(rxwi->rate);
	len = FIELD_GET(MT_RXWI_CTL_MPDU_LEN, ctl);
	if (rxinfo & MT_RXINFO_CRCERR)
		s->rx_crc++;
	if (len < 24 || len > seg_len)
		return;
	if (rxinfo & MT_RXINFO_CRCERR)
		return;

	fc = get_unaligned_le16(data);
	if ((fc & IEEE80211_FCTL_FTYPE) != IEEE80211_FTYPE_MGMT)
		return;
	if ((fc & IEEE80211_FCTL_STYPE) != IEEE80211_STYPE_BEACON &&
	    (fc & IEEE80211_FCTL_STYPE) != IEEE80211_STYPE_PROBE_RESP)
		return;

	hdr_len = 24;
	if (fc & IEEE80211_FCTL_ORDER)
		hdr_len += 4;	/* HT control */
	if (hdr_len > len)
		return;

	body = data + hdr_len;
	if (rxinfo & MT_RXINFO_L2PAD) {
		body += 2;
		if (len + 2 > seg_len)
			return;
	}

	rssi = mt7601u_phy_get_rssi(dev, rxwi, rate);

	mt7601u_scan_mgmt(dev, data, body, len - hdr_len, rssi,
			  fc & IEEE80211_FCTL_STYPE);
}

/* mt7601u_rx_next_seg_len() */
static u16 mt7601u_rx_next_seg_len(u8 *data, u32 data_len)
{
	u32 min_seg_len = MT_DMA_HDR_LEN + MT_RX_INFO_LEN +
		sizeof(struct mt7601u_rxwi) + MT_FCE_INFO_LEN;
	u16 dma_len;

	if (data_len < min_seg_len)
		return 0;

	dma_len = get_unaligned_le16(data);
	if (!dma_len || dma_len + MT_DMA_HDRS > data_len ||
	    (dma_len & 0x3) || dma_len < min_seg_len)
		return 0;

	return MT_DMA_HDRS + dma_len;
}

/* mt7601u_rx_process_entry() */
static void mt7601u_rx_process_entry(struct mt7601u_dev *dev, u8 *data,
				     u32 data_len)
{
	u32 seg_len;

	while ((seg_len = mt7601u_rx_next_seg_len(data, data_len))) {
		mt7601u_rx_process_seg(dev, data, seg_len);

		data_len -= seg_len;
		data += seg_len;
	}

	/* Leftover that does not parse as a segment */
	if (data_len)
		dev->scan.rx_bad_tail++;
}

static int mt7601u_async_rx_arm(struct mt7601u_dev *dev)
{
	int ret = xhci_async_bulk_submit(dev->async_rx_h, dev->rx_buf,
					 MT_RX_URB_SIZE);

	if (ret) {
		dev->async_submit_err++;
		dev->async_last_err = ret;
		if (ret == -ENODEV)
			dev->gone = true;
	}
	return ret;
}

/*
 * async mode RX: never waits, never cancels. If the pending packet-RX TD
 * has completed (here or inside any other wait), parse it and queue the
 * next one.
 */
/* Poll one pkt-RX handle; parse its URB, then re-arm THAT handle. The other
 * handle in the pool stays posted across the parse, so the endpoint is never
 * left empty -- an empty endpoint wedges the dongle's USB RX DMA (rx_busy stuck)
 * across a scan sweep, the same failure the eth data path hit. */
static int mt7601u_scan_rx_one(struct mt7601u_dev *dev, int h, u8 *buf)
{
	struct mt7601u_scan *s = &dev->scan;
	int ret, actual = 0;

	ret = xhci_async_bulk_poll(h, &actual);
	if (ret == -EINPROGRESS) {
		s->rx_empty++;
		return 0;
	}
	if (ret == -ENODEV) {
		dev->gone = true;
		return -ENODEV;
	}

	if (ret != -ENOENT) {
		s->rx_xfers++;
		if (!ret) {
			s->rx_bytes += actual;
			mt7601u_rx_process_entry(dev, buf, actual);
		}
	}

	/* parsed (or never queued / cancelled): queue the next TD */
	xhci_async_bulk_submit(h, buf, MT_RX_URB_SIZE);
	return ret == -ENOENT ? 0 : ret;
}

static int mt7601u_scan_rx_async(struct mt7601u_dev *dev)
{
	int ret = mt7601u_scan_rx_one(dev, dev->async_rx_h, dev->rx_buf);

	if (ret == -ENODEV)
		return ret;
	if (dev->async_rx_h2 >= 0) {
		int r2 = mt7601u_scan_rx_one(dev, dev->async_rx_h2,
					     dev->rx_buf2);
		if (r2 == -ENODEV)
			return r2;
	}
	return ret;
}

static void mt7601u_scan_account(struct mt7601u_scan *s, int phase, u64 dt,
				 int err)
{
	struct mt7601u_phase_stat *p = &s->phase[phase];

	p->count++;
	p->total_us += dt;
	if (dt > p->max_us)
		p->max_us = dt;

	if (err && err != -EINPROGRESS) {
		p->errors++;
		p->last_err = err;
		s->errors++;
		s->last_err = err;
		if (phase != MT_PHASE_RX)
			s->cur_hop_errs++;
	}
}

static void mt7601u_async_snapshot(struct mt7601u_dev *dev);

/*
 * One cyclic tick: poll the packet-RX TD (never blocking, never cancelled),
 * then, if a channel switch is in progress, run one stage of it. Both bulk
 * IN endpoints keep one pending xHCI async TD. The MCU command OUTs inside a
 * switch stay synchronous (they complete at once; replies are never lost, so
 * the OUT no longer NAKs). The tick re-arms itself faster while switching.
 */
static void mt7601u_scan_tick(struct cyclic_info *c)
{
	struct mt7601u_scan *s = container_of(c, struct mt7601u_scan, cyclic);
	struct mt7601u_dev *dev = container_of(s, struct mt7601u_dev, scan);
	int phase, ret;
	u64 t0, t1;

	/* A synchronous transfer of ours is in flight: never interleave */
	if (dev->busy) {
		s->busy_skips++;
		return;
	}

	t0 = get_timer_us(0);
	dev->in_cyclic = true;
	s->ticks++;

	/* RX every tick, then at most one switch stage */
	ret = mt7601u_scan_rx_async(dev);
	mt7601u_scan_account(s, MT_PHASE_RX, get_timer_us(0) - t0, ret);
	phase = MT_PHASE_RX;
	if (!dev->gone && dev->hop_stage != HOP_DONE) {
		t1 = get_timer_us(0);
		phase = dev->hop_stage;
		ret = mt7601u_hop_step(dev);
		mt7601u_scan_account(s, phase, get_timer_us(0) - t1, ret);
	}

	dev->in_cyclic = false;
	mt7601u_async_snapshot(dev);	/* kept if the TDs are forgotten */

	t1 = get_timer_us(0) - t0;
	s->tick_us_total += t1;
	if (t1 > s->tick_us_max)
		s->tick_us_max = t1;

	if (dev->gone) {
		/* xHCI forgot the TDs (usb reset): stop, touch nothing */
		cyclic_unregister(c);
		s->running = false;
		dev->async_rx_h = -1;
		dev->async_rx_h2 = -1;
		dev->async_resp_h = -1;
		dev->async_resp_h2 = -1;
		return;
	}

	if (dev->mcu_out_fails >= MT_SCAN_MCU_DEAD_SENDS) {
		/* the MCU stopped accepting commands: stop rather than block
		 * every tick in another 500 ms OUT timeout */
		printf("mt7601u: MCU not accepting commands (%u failed sends); scan stopped -- cold-start the dongle\n",
		       dev->mcu_out_fails);
		s->mcu_dead = true;
		mt7601u_scan_stop(dev, false);
		return;
	}

	if (phase != MT_PHASE_RX && dev->hop_stage == HOP_DONE) {
		/* switch complete: back to dwelling */
		u64 hop_us = get_timer_us(0) - s->hop_start_us;
		struct mt7601u_chan_stat *cs = &s->chan[dev->hop_chan];

		if (dev->hop_verified)
			cs->switch_ok++;
		else
			cs->switch_bad++;
		if (s->cur_hop_errs)
			cs->switch_err++;
		memcpy(cs->last_rb, dev->hop_rb, sizeof(cs->last_rb));

		s->hops++;
		s->hop_us_total += hop_us;
		if (hop_us > s->hop_us_max)
			s->hop_us_max = hop_us;
		s->dwell_start = get_timer(0);
		mt7601u_bss_drop_chan(s, dev->hop_chan);
		c->delay_us = MT_SCAN_PERIOD_US;
	} else if (phase == MT_PHASE_RX &&
		   get_timer(s->dwell_start) >= s->dwell_ms) {
		u8 next = dev->chan + 1;

		if (next < s->first_chan || next > s->last_chan)
			next = s->first_chan;
		mt7601u_hop_begin(dev, next);	/* first stage next tick */
		s->hop_start_us = get_timer_us(0);
		s->cur_hop_errs = 0;
		c->delay_us = MT_SCAN_HOP_PERIOD_US;
	}
}

/* keep a copy of the async TD stats, so they survive teardown / usb reset */
static void mt7601u_async_snapshot(struct mt7601u_dev *dev)
{
	if (dev->async_rx_h >= 0 &&
	    !xhci_async_bulk_stats(dev->async_rx_h, &dev->async_st[0]))
		dev->async_st_valid = true;
	if (dev->async_resp_h >= 0 &&
	    !xhci_async_bulk_stats(dev->async_resp_h, &dev->async_st[1]))
		dev->async_st_valid = true;
}

static void mt7601u_async_close(struct mt7601u_dev *dev)
{
	mt7601u_async_snapshot(dev);
	if (dev->async_rx_h >= 0)
		xhci_async_bulk_close(dev->async_rx_h);
	if (dev->async_rx_h2 >= 0)
		xhci_async_bulk_close(dev->async_rx_h2);
	if (dev->async_resp_h >= 0)
		xhci_async_bulk_close(dev->async_resp_h);
	if (dev->async_resp_h2 >= 0)
		xhci_async_bulk_close(dev->async_resp_h2);
	dev->async_rx_h = -1;
	dev->async_rx_h2 = -1;
	dev->async_resp_h = -1;
	dev->async_resp_h2 = -1;
	dev->resp_next = 0;
}

int mt7601u_scan_start(struct mt7601u_dev *dev, u32 dwell_ms)
{
	struct mt7601u_scan *s = &dev->scan;
	struct usb_device *udev = dev->udev;
	int ret;

	if (!test_bit(MT7601U_STATE_INITIALIZED, &dev->state)) {
		printf("mt7601u: not initialised, run 'wifi init'\n");
		return -ENODEV;
	}

	if (s->running)
		mt7601u_scan_stop(dev, true);

	/* A station join holds the same packet-RX TDs. Channel commands
	 * time out while those stay posted and MAC RX stays on.
	 */
	mt7601u_connect_stop(dev);

	/* Preserve bss[] and n_bss. A new scan drops a channel's rows only
	 * when that channel's dwell starts.
	 */
	memset(s, 0, offsetof(struct mt7601u_scan, n_bss));
	s->dwell_ms = dwell_ms ? dwell_ms : MT_SCAN_DEFAULT_DWELL_MS;
	dev->mcu_out_fails = 0;
	dev->mcu_bad_resp = 0;
	dev->mcu_last_bad = 0;
	dev->mcu_late_resp = 0;
	dev->mcu_last_late = 0;
	s->first_chan = MT_SCAN_FIRST_CHAN;
	s->last_chan = MT_SCAN_LAST_CHAN;

	/* mt7601u_sw_scan() */
	mt7601u_agc_save(dev);
	__set_bit(MT7601U_STATE_SCANNING, &dev->state);

	ret = mt7601u_phy_set_channel(dev, s->first_chan);
	if (ret) {
		printf("mt7601u: set channel %d failed:%d\n", s->first_chan,
		       ret);
		goto err;
	}
	if (dev->hop_verified)
		s->chan[s->first_chan].switch_ok++;
	else
		s->chan[s->first_chan].switch_bad++;
	memcpy(s->chan[s->first_chan].last_rb, dev->hop_rb,
	       sizeof(s->chan[0].last_rb));

	ret = mt7601u_mac_start_rx(dev);
	if (ret) {
		printf("mt7601u: MAC RX start failed:%d\n", ret);
		goto err;
	}

	dev->gone = false;
	dev->async_submit_err = 0;
	dev->async_last_err = 0;
	dev->async_st_valid = false;
	memset(dev->async_st, 0, sizeof(dev->async_st));

	dev->async_rx_h = xhci_async_bulk_open(udev,
		usb_rcvbulkpipe(udev, dev->in_eps[MT_EP_IN_PKT_RX] & 0xf));
	/* a second pkt-RX TD on the same endpoint: the pool keeps one posted
	 * while the other is parsed/re-armed, so the RX DMA never wedges */
	dev->async_rx_h2 = xhci_async_bulk_open(udev,
		usb_rcvbulkpipe(udev, dev->in_eps[MT_EP_IN_PKT_RX] & 0xf));
	dev->async_resp_h = xhci_async_bulk_open(udev,
		usb_rcvbulkpipe(udev, dev->in_eps[MT_EP_IN_CMD_RESP] & 0xf));
	/* two cmd-resp TDs too: back-to-back replies to a re-sent command */
	dev->async_resp_h2 = xhci_async_bulk_open(udev,
		usb_rcvbulkpipe(udev, dev->in_eps[MT_EP_IN_CMD_RESP] & 0xf));
	dev->resp_next = 0;
	if (dev->async_rx_h < 0 || dev->async_rx_h2 < 0 ||
	    dev->async_resp_h < 0 || dev->async_resp_h2 < 0) {
		printf("mt7601u: async TD open failed: rx %d rx2 %d resp %d resp2 %d\n",
		       dev->async_rx_h, dev->async_rx_h2, dev->async_resp_h,
		       dev->async_resp_h2);
		ret = -EIO;
		goto err_async;
	}
	ret = mt7601u_async_rx_arm(dev);
	if (!ret)
		ret = xhci_async_bulk_submit(dev->async_rx_h2, dev->rx_buf2,
					     MT_RX_URB_SIZE);
	if (!ret)
		ret = mt7601u_async_resp_arm(dev);
	if (ret) {
		printf("mt7601u: async TD submit failed: %d\n", ret);
		goto err_async;
	}

	mt7601u_bss_drop_chan(s, s->first_chan);
	s->dwell_start = get_timer(0);
	s->running = true;
	cyclic_register(&s->cyclic, mt7601u_scan_tick, MT_SCAN_PERIOD_US,
			"mt7601u_scan");

	printf("mt7601u: background scan ch %d-%d, %u ms dwell, tick %u us (%u us while switching)\n",
	       s->first_chan, s->last_chan, s->dwell_ms, MT_SCAN_PERIOD_US,
	       MT_SCAN_HOP_PERIOD_US);
	return 0;

err_async:
	mt7601u_async_close(dev);
err:
	__clear_bit(MT7601U_STATE_SCANNING, &dev->state);
	return ret;
}

/*
 * @finish: drain an outstanding MCU response and complete a half-done
 * channel switch synchronously. false = abandon both (the command path is
 * presumed wedged, e.g. the controller went away).
 */
void mt7601u_scan_stop(struct mt7601u_dev *dev, bool finish)
{
	struct mt7601u_scan *s = &dev->scan;

	if (!s->running)
		return;

	cyclic_unregister(&s->cyclic);
	s->running = false;

	/* where the stop landed: the stop-wedge trigger is still unknown */
	if (dev->hop_stage != HOP_DONE || dev->mcu_pending)
		printf("mt7601u: stop mid-hop: chan %u stage %d retries %u pending %d (%lu ms since send) out-fails %u\n",
		       dev->hop_chan, dev->hop_stage, dev->hop_retries,
		       dev->mcu_pending, get_timer(dev->mcu_sent_at),
		       dev->mcu_out_fails);

	/*
	 * An MCU reply in flight lands on its posted cmd-resp TD first:
	 * aborting that TD under it wedges the dongle (rx_busy stuck, every
	 * later command OUT NAKed) -- 'wifi scan stop 2' reproduced it on the
	 * first stop. Bounded at the command's 500 ms.
	 */
	while (finish && dev->mcu_pending &&
	       mt7601u_mcu_poll_resp(dev, 1000) != 0 &&
	       get_timer(dev->mcu_sent_at) < 500)
		;
	dev->mcu_pending = false;

	/*
	 * Same for the pkt-RX TDs: a frame can be mid-transfer on them. MAC RX
	 * off, then keep servicing them until the dongle's USB RX DMA is idle
	 * on two reads in a row (bounded at 50 ms) -- 'wifi scan stop 5' still
	 * wedged 1 in 6 with only the reply taken first.
	 */
	if (finish) {
		ulong t0 = get_timer(0);
		int idle = 0;

		mt76_clear(dev, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX);
		while (idle < 2 && get_timer(t0) < 50) {
			mt7601u_scan_rx_async(dev);
			if (mt7601u_rr(dev, MT_USB_DMA_CFG) &
			    MT_USB_DMA_CFG_RX_BUSY)
				idle = 0;
			else
				idle++;
		}
		if (idle < 2)
			printf("mt7601u: stop: USB RX DMA still busy after 50 ms\n");
	}

	/* then the async TDs, so the rest below runs on synchronous transfers */
	mt7601u_async_close(dev);
	if (finish && dev->hop_stage != HOP_DONE)
		mt7601u_phy_set_channel(dev, dev->hop_chan);
	dev->hop_stage = HOP_DONE;

	/* MAC RX off, mt7601u_sw_scan_complete() */
	mt76_clear(dev, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX);
	mt7601u_agc_restore(dev);
	__clear_bit(MT7601U_STATE_SCANNING, &dev->state);
}

void mt7601u_scan_show(struct mt7601u_dev *dev, bool verbose)
{
	struct mt7601u_scan *s = &dev->scan;
	int i;

	printf("scan: %s%s, chan %u, dwell %u ms\n",
	       s->running ? "running" : "stopped",
	       dev->gone ? " (controller went away: usb reset?)" : "",
	       dev->chan, s->dwell_ms);

	if (!dev->gone) {
		u32 mac, c0, c1, tx0;

		dev->busy++;
		dev->reg_failed = false;
		dev->last_usb_dma_cfg = mt7601u_rr(dev, MT_USB_DMA_CFG);
		/* Clear-on-read. One show consumes the counts since the
		 * last read (init clears them). false_cca is the low half
		 * of CNT1; phy_err / crc_err split CNT0. */
		mac = mt7601u_rr(dev, MT_MAC_SYS_CTRL);
		c0 = mt7601u_rr(dev, MT_RX_STA_CNT0);
		c1 = mt7601u_rr(dev, MT_RX_STA_CNT1);
		tx0 = mt7601u_rr(dev, MT_TX_STA_CNT0);
		dev->busy--;
		printf("USB_DMA_CFG %08x%s  [rx_busy %d tx_busy %d]\n",
		       dev->last_usb_dma_cfg,
		       dev->reg_failed ? " (read FAILED)" : "",
		       !!(dev->last_usb_dma_cfg & MT_USB_DMA_CFG_RX_BUSY),
		       !!(dev->last_usb_dma_cfg & MT_USB_DMA_CFG_TX_BUSY));
		printf("mib: MAC_SYS_CTRL %08x RX_STA_CNT0 %08x RX_STA_CNT1 %08x TX_STA_CNT0 %08x false_cca %u phy_err %u crc_err %u\n",
		       mac, c0, c1, tx0, c1 & 0xffff, c0 >> 16, c0 & 0xffff);
	}
	printf("ticks %u  busy-skips %u  rx-frames %u  rx-crc %u  rx-bytes %llu  mgmt(bcn/presp) %u  hops %u  errors %u (last %d)\n",
	       s->ticks, s->busy_skips, s->rx_frames, s->rx_crc, s->rx_bytes,
	       s->rx_mgmt, s->hops, s->errors, s->last_err);
	if (s->mcu_dead)
		printf("scan: stopped early -- MCU stopped accepting commands; cold-start the dongle\n");
	printf("tick cpu: max %llu us, avg %llu us (CYCLIC_MAX_CPU_TIME_US %d)\n",
	       s->tick_us_max, s->ticks ? s->tick_us_total / s->ticks : 0,
	       CONFIG_CYCLIC_MAX_CPU_TIME_US);
	printf("mcu: late replies %u; bad events %u (last %08x); last resp %08x; pending %d\n",
	       dev->mcu_late_resp, dev->mcu_bad_resp, dev->mcu_last_bad,
	       dev->mcu_last_resp, dev->mcu_pending);

	if (verbose) {
		mt7601u_async_snapshot(dev);
		for (i = 0; i < 2 && dev->async_st_valid; i++) {
			const struct xhci_async_stats st = dev->async_st[i];

			printf("async %-8s TD: submitted %u completed %u (inside other waits %u) polls %u pending %d | queue-errors %u orphans %u last comp %d\n",
			       i ? "cmd-resp" : "pkt-rx", st.submitted,
			       st.completed, st.completed_in_wait, st.polls,
			       st.active, st.errors, st.orphans, st.last_comp);
		}
		printf("async re-arm errors %u (last %d); rx-empty %u  rx-nonpkt-seg %u (last fce %08x)  rx-bad-tail %u\n",
		       dev->async_submit_err, dev->async_last_err, s->rx_empty,
		       s->rx_nonpkt_seg, s->rx_last_nonpkt_fce, s->rx_bad_tail);
		printf("hop wall time: max %llu us, avg %llu us\n",
		       s->hop_us_max, s->hops ? s->hop_us_total / s->hops : 0);

		printf("\n %-4s  %-14s  %7s  %9s  %9s  %8s  %6s  %s\n",
		       "step", "phase", "count", "max us", "avg us",
		       "total ms", "errors", "last err");
		for (i = 0; i < MT_N_PHASES; i++) {
			const struct mt7601u_phase_stat *p = &s->phase[i];
			int st = mt7601u_hop_linux_step(i);

			if (!p->count && !p->errors)
				continue;
			printf(" %-4s  %-14s  %7u  %9llu  %9llu  %8llu  %6u  %d\n",
			       i == MT_PHASE_RX ? "rx" :
			       st == 1 ? "1" : st == 2 ? "2" : st == 3 ? "3" :
			       st == 4 ? "4" : st == 5 ? "5" : "vfy",
			       mt7601u_hop_stage_name(i), p->count, p->max_us,
			       p->count ? p->total_us / p->count : 0,
			       p->total_us / 1000, p->errors, p->last_err);
		}
	}

	/* per tuned channel: did the switch take, and what was heard */
	printf("\n %-4s  %-8s  %-8s  %-8s  %-9s  %-9s  %-8s  %s\n",
	       "chan", "sw-ok", "sw-bad", "sw-err", "frm-on", "frm-off",
	       "frm-nods", "RF 0/17..20 last (plan)");
	for (i = 1; i <= 14; i++) {
		const struct mt7601u_chan_stat *cs = &s->chan[i];

		if (!cs->switch_ok && !cs->switch_bad && !cs->frames_on &&
		    !cs->frames_off && !cs->frames_nods)
			continue;
		printf(" %-4d  %-8u  %-8u  %-8u  %-9u  %-9u  %-8u  %02x %02x %02x %02x (%s)\n",
		       i, cs->switch_ok, cs->switch_bad, cs->switch_err,
		       cs->frames_on, cs->frames_off, cs->frames_nods,
		       cs->last_rb[0], cs->last_rb[1], cs->last_rb[2],
		       cs->last_rb[3],
		       cs->switch_ok || cs->switch_bad ?
		       mt7601u_freq_plan_str(i) : "-");
	}

	printf("\n %-2s  %-17s  %-4s  %-5s  %-5s  %-6s  %-5s  %-8s  %s\n",
	       "#", "BSSID", "chan", "rssi", "max", "beacon", "presp",
	       "akm", "SSID");
	for (i = 0; i < s->n_bss; i++) {
		const struct mt7601u_bss *b = &s->bss[i];

		printf(" %-2d  %pM  %-4u  %-5d  %-5d  %-6u  %-5u  %-8s  %s\n",
		       i, b->bssid, b->chan, b->rssi, b->rssi_max,
		       b->beacons, b->probe_resp, mt7601u_bss_akm(b),
		       (b->ssid_len && b->ssid[0]) ? b->ssid : "<hidden>");
	}
	if (s->n_bss >= MT_SCAN_MAX_BSS)
		printf(" (table full, %d entries)\n", MT_SCAN_MAX_BSS);
}
