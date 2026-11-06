// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * MT7601U station-mode connect for U-Boot (EXPERIMENTAL). This file joins the
 * owner's own AP as a station: open-system authentication first, then (later
 * milestones) association, the WPA2-PSK four-way handshake and hardware key
 * install, then an eth-uclass wrapper so dhcp/wget run over the link.
 *
 * Frame transmit mirrors the Linux mt7601u Tx path (tx.c/dma.c): one 802.11
 * frame wrapped in a TXWI and a Tx DMA header, sent with a single synchronous
 * bulk-OUT on the AC_BE endpoint. Receive reuses the scan's never-cancelled
 * xHCI async bulk-IN TD on the packet-RX endpoint (cancelling an in-flight
 * bulk IN wedges this dongle), polled without blocking. There is no Tx ring,
 * no mac80211 and no retransmit queue -- each request/response step is driven
 * synchronously from the 'wifi join' command.
 */

#include <command.h>
#include <cyclic.h>
#include <dm.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <dm/root.h>
#include <env.h>
#include <log.h>
#include <malloc.h>
#include <net.h>
#include <time.h>
#include <vsprintf.h>
#include <usb.h>
#include <usb/xhci.h>
#include <u-boot/sha1.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "mt7601u.h"
#include "tx.h"
#include "connect.h"
#include "wpa_crypto.h"
#include "sae.h"
#include "dma.h"
#include "mac.h"
#include "regs.h"

/* 802.11 frame-control type/subtype (little-endian frame_control word) */
#define FC_FTYPE		0x000c
#define FC_STYPE		0x00f0
#define FTYPE_MGMT		0x0000
#define FTYPE_DATA		0x0008
#define STYPE_ASSOC_REQ		0x0000
#define STYPE_ASSOC_RESP	0x0010
#define STYPE_PROBE_RESP	0x0050
#define STYPE_AUTH		0x00b0
#define STYPE_DEAUTH		0x00c0
#define STYPE_DISASSOC		0x00a0

#define IEEE80211_FCTL_ORDER	0x8000

static struct mt7601u_connect conn;

/* ------------------------------------------------------------------ Tx */

/*
 * Build [Tx DMA info][TXWI][802.11 frame][pad][trailer] and send it with one
 * synchronous bulk-OUT on AC_BE. wcid 0xff + WIV means "no hardware key": the
 * frame goes out exactly as built (auth/assoc/EAPOL before keys exist).
 */
static int conn_tx(struct mt7601u_dev *dev, const void *frame, int flen,
		   bool encrypt, u8 wcid, u16 rate_ctl, bool no_fallback)
{
	u8 *buf = dev->tx_buf;
	struct mt76_txwi *txwi;
	int wlen = sizeof(*txwi) + flen;
	int plen = round_up(wlen, 4);
	int total = 4 + plen + 4;
	int actual = 0;
	u32 info;

	info = MT_TXD_PKT_INFO_80211 |
	       FIELD_PREP(MT_TXD_PKT_INFO_QSEL, MT_QSEL_EDCA) |
	       FIELD_PREP(MT_TXD_INFO_D_PORT, WLAN_PORT) |
	       FIELD_PREP(MT_TXD_INFO_TYPE, DMA_PACKET) |
	       FIELD_PREP(MT_TXD_INFO_LEN, plen);
	if (!encrypt)
		info |= MT_TXD_PKT_INFO_WIV;
	put_unaligned_le32(info, buf);

	txwi = (struct mt76_txwi *)(buf + 4);
	memset(txwi, 0, sizeof(*txwi));
	txwi->rate_ctl = cpu_to_le16(rate_ctl);
	if (no_fallback)
		txwi->flags = cpu_to_le16(MT_TXWI_FLAGS_NO_RATE_FALLBACK);
	txwi->ack_ctl = MT_TXWI_ACK_CTL_REQ;
	/* let hardware assign the sequence number for encrypted data frames */
	if (encrypt)
		txwi->ack_ctl |= MT_TXWI_ACK_CTL_NSEQ;
	txwi->wcid = wcid;
	txwi->len_ctl = cpu_to_le16(FIELD_PREP(MT_TXWI_LEN_BYTE_CNT, flen));

	memcpy(buf + 4 + sizeof(*txwi), frame, flen);
	memset(buf + 4 + wlen, 0, total - (4 + wlen));

	return mt7601u_bulk(dev, false, MT_EP_OUT_AC_BE, buf, total,
			    &actual, 500);
}

/* OFDM 24 Mbps (MCS index 4) for data frames */
#define MT_RATE_OFDM_24M	(FIELD_PREP(MT_TXWI_RATE_PHY_MODE, \
					    MT_PHY_TYPE_OFDM) | \
				 FIELD_PREP(MT_TXWI_RATE_MCS, 4))

static int build_mgmt_hdr(u8 *f, u16 fc, const u8 *a1, const u8 *a2,
			  const u8 *a3)
{
	put_unaligned_le16(fc, f);
	f[2] = 0;
	f[3] = 0;
	memcpy(f + 4, a1, 6);
	memcpy(f + 10, a2, 6);
	memcpy(f + 16, a3, 6);
	f[22] = 0;
	f[23] = 0;
	return 24;
}

static int build_auth(u8 *f, const u8 *bssid, const u8 *mac)
{
	int n = build_mgmt_hdr(f, FTYPE_MGMT | STYPE_AUTH, bssid, mac, bssid);

	put_unaligned_le16(0, f + n);		/* algorithm: open system */
	put_unaligned_le16(1, f + n + 2);	/* transaction sequence 1 */
	put_unaligned_le16(0, f + n + 4);	/* status */
	return n + 6;
}

/* ------------------------------------------------------------------ Rx */

u32 g_rx_frames;
u32 g_rx_crc_drop;

/* mt7601u_rx_next_seg_len() */
static u16 conn_next_seg_len(u8 *data, u32 data_len)
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

/*
 * Extract one received 802.11 MPDU from a DMA segment (the Rx counterpart of
 * conn_tx's framing). Returns the frame pointer, its logical length and the
 * RXWI rxinfo word; false for a non-packet (MCU) segment or a malformed one.
 */
static bool conn_seg_mpdu(u8 *seg, u16 seg_len, u8 **mpdu, u32 *mlen,
			  u32 *rxinfo)
{
	const struct mt7601u_rxwi *rxwi;
	u32 fce, ctl, len;
	u8 *data = seg;

	fce = get_unaligned_le32(seg + seg_len - MT_FCE_INFO_LEN);
	seg_len -= MT_FCE_INFO_LEN;
	data += MT_DMA_HDR_LEN;
	seg_len -= MT_DMA_HDR_LEN;
	rxwi = (const struct mt7601u_rxwi *)data;
	data += sizeof(struct mt7601u_rxwi);
	seg_len -= sizeof(struct mt7601u_rxwi);

	if (FIELD_GET(MT_RXD_INFO_TYPE, fce))
		return false;

	*rxinfo = le32_to_cpu(rxwi->rxinfo);
	ctl = le32_to_cpu(rxwi->ctl);
	len = FIELD_GET(MT_RXWI_CTL_MPDU_LEN, ctl);
	if (len < 10 || len > seg_len)
		return false;
	/* The flag is counted, not dropped. A false CRCERR on the image
	 * frames leaves the HTTP GET at the header segment. TCP checksums
	 * still reject a frame that is actually corrupt.
	 */
	if (*rxinfo & MT_RXINFO_CRCERR)
		g_rx_crc_drop++;

	*mpdu = data;
	*mlen = len;
	return true;
}

static int conn_rx_arm(struct mt7601u_dev *dev)
{
	return xhci_async_bulk_submit(dev->async_rx_h, dev->rx_buf,
				      MT_RX_URB_SIZE);
}

static int conn_open_rx(struct mt7601u_dev *dev)
{
	struct usb_device *udev = dev->udev;
	int h;

	h = xhci_async_bulk_open(udev,
		usb_rcvbulkpipe(udev, dev->in_eps[MT_EP_IN_PKT_RX] & 0xf));
	if (h < 0) {
		printf("mt7601u: async RX open failed:%d\n", h);
		return h;
	}
	dev->async_rx_h = h;
	return conn_rx_arm(dev);
}

/* take a completed pkt-RX TD (its frame is dropped) and post it again */
static void conn_rx_drain_one(int h, u8 *buf)
{
	int actual = 0;
	int ret = xhci_async_bulk_poll(h, &actual);

	if (ret != -EINPROGRESS && ret != -ENODEV)
		xhci_async_bulk_submit(h, buf, MT_RX_URB_SIZE);
}

/* Idle RX service. Registered only while a join is up. */
static struct cyclic_info conn_rx_cyc;

/*
 * The scan stop's rule (m20) on the join path: aborting a pkt-RX TD while a
 * frame is in flight on it wedges the dongle (rx_busy stuck, every later MCU
 * command OUT -110). A failed WPA2 join used to close the TD with MAC RX on
 * and the AP still sending, and the next 'wifi join' found the MCU dead.
 * MAC RX off, keep the TDs posted and serviced until the USB RX DMA reads
 * idle twice (bounded at 200 ms), then close.
 */
static void conn_close_rx(struct mt7601u_dev *dev)
{
	ulong t0 = get_timer(0);
	int idle = 0;

	cyclic_unregister(&conn_rx_cyc);

	if (dev->async_rx_h >= 0 && !dev->gone) {
		mt76_clear(dev, MT_MAC_SYS_CTRL, MT_MAC_SYS_CTRL_ENABLE_RX);
		while (idle < 2 && get_timer(t0) < 200) {
			conn_rx_drain_one(dev->async_rx_h, dev->rx_buf);
			if (dev->async_rx_h2 >= 0)
				conn_rx_drain_one(dev->async_rx_h2, dev->rx_buf2);
			if (mt7601u_rr(dev, MT_USB_DMA_CFG) &
			    MT_USB_DMA_CFG_RX_BUSY)
				idle = 0;
			else
				idle++;
		}
		if (idle < 2)
			printf("mt7601u: join: USB RX DMA still busy after 200 ms\n");
	}

	if (dev->async_rx_h2 >= 0)
		xhci_async_bulk_close(dev->async_rx_h2);
	dev->async_rx_h2 = -1;
	if (dev->async_rx_h >= 0)
		xhci_async_bulk_close(dev->async_rx_h);
	dev->async_rx_h = -1;
}

/*
 * Management frames that arrive in the same URB as the one this wait is
 * consuming. xhci_async_bulk_poll reports a completion once, so returning
 * on the first match used to drop a confirm that shared the commit's burst.
 * One slot per authentication sequence; a later sequence replaces an older
 * one when the queue is full.
 */
#define CONN_STASH_N	4
#define CONN_STASH_L	768

struct conn_stashed {
	u8 mpdu[CONN_STASH_L];
	u16 len;
	u32 rxinfo;
};

static struct conn_stashed conn_stash[CONN_STASH_N];
static int conn_stash_n;
static u8 conn_match[CONN_STASH_L];

static void conn_stash_flush(void)
{
	conn_stash_n = 0;
}

static void conn_stash_drop(int idx)
{
	int i;

	if (idx < 0 || idx >= conn_stash_n)
		return;
	conn_stash_n--;
	for (i = idx; i < conn_stash_n; i++)
		conn_stash[i] = conn_stash[i + 1];
}

static u16 conn_auth_seq_of(const u8 *m, u32 l, u32 rxi)
{
	int aoff = 24 + ((rxi & MT_RXINFO_L2PAD) ? 2 : 0);

	if ((int)l < aoff + 6)
		return 0xffff;
	return get_unaligned_le16(m + aoff + 2);
}

static void conn_stash_push(const u8 *m, u32 l, u32 rxi)
{
	u16 st, seq;
	int i;

	if (l < 24 || l > CONN_STASH_L)
		return;
	st = get_unaligned_le16(m) & FC_STYPE;
	seq = (st == STYPE_AUTH) ? conn_auth_seq_of(m, l, rxi) : 0;
	if (st == STYPE_AUTH && seq == 0xffff)
		return;
	for (i = 0; i < conn_stash_n; i++) {
		u16 st2 = get_unaligned_le16(conn_stash[i].mpdu) & FC_STYPE;

		if (st2 != st)
			continue;
		if (st != STYPE_AUTH ||
		    conn_auth_seq_of(conn_stash[i].mpdu, conn_stash[i].len,
				     conn_stash[i].rxinfo) == seq)
			return;
	}
	if (conn_stash_n >= CONN_STASH_N) {
		for (i = 0; i < conn_stash_n; i++) {
			u16 st2 = get_unaligned_le16(conn_stash[i].mpdu) &
				  FC_STYPE;

			if (st == STYPE_AUTH && st2 == STYPE_AUTH &&
			    conn_auth_seq_of(conn_stash[i].mpdu,
					     conn_stash[i].len,
					     conn_stash[i].rxinfo) != seq) {
				conn_stash_drop(i);
				break;
			}
		}
	}
	if (conn_stash_n >= CONN_STASH_N)
		return;
	memcpy(conn_stash[conn_stash_n].mpdu, m, l);
	conn_stash[conn_stash_n].len = (u16)l;
	conn_stash[conn_stash_n].rxinfo = rxi;
	conn_stash_n++;
}

/* 1 = copied to conn_match, -ECONNREFUSED = deauth/disassoc, 0 = keep waiting. */
static int conn_mgmt_verdict(struct mt7601u_dev *dev, const u8 *m, u32 l,
			     u32 rxi, u16 stype, int auth_seq,
			     u8 **mpdu, u32 *mlen, u32 *rxinfo)
{
	u16 fc, st;

	if (l < 24 || l > CONN_STASH_L)
		return 0;
	fc = get_unaligned_le16(m);
	if ((fc & FC_FTYPE) != FTYPE_MGMT)
		return 0;
	if (memcmp(m + 4, dev->macaddr, 6) || memcmp(m + 16, conn.bssid, 6))
		return 0;
	st = fc & FC_STYPE;
	if (st == STYPE_DEAUTH || st == STYPE_DISASSOC) {
		printf("mt7601u: AP sent %s\n",
		       st == STYPE_DEAUTH ? "deauth" : "disassoc");
		/*
		 * While waiting for authentication, this is the AP clearing
		 * the old association. The auth response follows it.
		 */
		if (stype == STYPE_AUTH)
			return 0;
		return -ECONNREFUSED;
	}
	if (st != stype)
		return 0;
	if (auth_seq >= 0 && st == STYPE_AUTH &&
	    conn_auth_seq_of(m, l, rxi) != (u16)auth_seq)
		return 0;
	memcpy(conn_match, m, l);
	*mpdu = conn_match;
	*mlen = l;
	*rxinfo = rxi;
	return 1;
}

/*
 * Poll the packet-RX async TD (never blocking, never cancelled) up to ms
 * milliseconds for a management frame of subtype `stype` addressed to us.
 * On a match sets *mpdu to a stable copy. Frames later in that URB stay
 * queued for the next wait, and the TD is re-armed. A deauth/disassoc
 * aborts every wait except authentication. auth_seq >= 0 keeps waiting
 * through authentication frames of another sequence (a retransmitted
 * SAE commit is not the confirm).
 */
static int conn_wait_mgmt(struct mt7601u_dev *dev, u16 stype, u8 **mpdu,
			  u32 *mlen, u32 *rxinfo, int ms, int auth_seq)
{
	ulong start = get_timer(0);
	int i;

	for (i = 0; i < conn_stash_n;) {
		int v = conn_mgmt_verdict(dev, conn_stash[i].mpdu,
					  conn_stash[i].len, conn_stash[i].rxinfo,
					  stype, auth_seq, mpdu, mlen, rxinfo);

		if (v == -ECONNREFUSED) {
			conn_stash_drop(i);
			return -ECONNREFUSED;
		}
		if (v == 1) {
			conn_stash_drop(i);
			return 0;
		}
		i++;
	}

	while (get_timer(start) < (ulong)ms) {
		int actual = 0;
		int ret = xhci_async_bulk_poll(dev->async_rx_h, &actual);
		int have = 0;
		u8 *data;
		u32 dlen;
		u16 sl;

		if (ret == -EINPROGRESS) {
			udelay(300);
			continue;
		}
		if (ret == -ENODEV) {
			dev->gone = true;
			return -ENODEV;
		}

		if (ret == 0 && actual > 0) {
			data = dev->rx_buf;
			dlen = actual;
			while ((sl = conn_next_seg_len(data, dlen))) {
				u8 *m;
				u32 l, rxi;
				u16 st = 0;
				int skip_stash = 0;

				if (conn_seg_mpdu(data, sl, &m, &l, &rxi) &&
				    l >= 24 && l <= CONN_STASH_L &&
				    (get_unaligned_le16(m) & FC_FTYPE) == FTYPE_MGMT &&
				    !memcmp(m + 4, dev->macaddr, 6) &&
				    !memcmp(m + 16, conn.bssid, 6)) {
					st = get_unaligned_le16(m) & FC_STYPE;
					if (!have) {
						int v = conn_mgmt_verdict(dev, m, l, rxi,
									  stype, auth_seq,
									  mpdu, mlen, rxinfo);

						if (v == -ECONNREFUSED) {
							conn_rx_arm(dev);
							return -ECONNREFUSED;
						}
						if (v == 1) {
							have = 1;
							skip_stash = 1;
						}
					}
					if (!skip_stash &&
					    (st == STYPE_AUTH ||
					     st == STYPE_ASSOC_RESP))
						conn_stash_push(m, l, rxi);
				}
				data += sl;
				dlen -= sl;
			}
		}
		conn_rx_arm(dev);
		if (have)
			return 0;
	}
	return -ETIMEDOUT;
}

/* ------------------------------------------------- milestone 1: auth */

static int conn_auth(struct mt7601u_dev *dev, struct mt7601u_connect *c)
{
	u8 frame[32];
	u8 *mpdu;
	u32 mlen = 0, rxi = 0;
	int flen, ret = -ETIMEDOUT, tries;
	u16 alg, seq, status;
	int off;

	conn_stash_flush();
	flen = build_auth(frame, c->bssid, dev->macaddr);

	for (tries = 0; tries < 4; tries++) {
		ret = conn_tx(dev, frame, flen, false, 0xff, MT_RATE_CCK_1M, true);
		if (ret) {
			printf("mt7601u: auth tx failed:%d\n", ret);
			continue;
		}
		ret = conn_wait_mgmt(dev, STYPE_AUTH, &mpdu, &mlen, &rxi,
				     400, 2);
		if (ret == 0 || ret == -ENODEV || ret == -ECONNREFUSED)
			break;
	}
	if (ret) {
		printf("mt7601u: auth: no response from %pM\n", c->bssid);
		return ret;
	}

	off = 24 + ((rxi & MT_RXINFO_L2PAD) ? 2 : 0);
	if (mlen < 24 + 6) {
		printf("mt7601u: auth resp too short (%u)\n", mlen);
		return -EINVAL;
	}
	alg = get_unaligned_le16(mpdu + off);
	seq = get_unaligned_le16(mpdu + off + 2);
	status = get_unaligned_le16(mpdu + off + 4);
	printf("mt7601u: auth resp alg %u seq %u status %u\n", alg, seq, status);
	if (status != 0)
		return -EACCES;

	c->state = MT_CONN_AUTHED;
	return 0;
}

/* -------------------------------------------- milestone 2: associate */

/* Information elements for a WPA2-PSK / CCMP association request */
static const u8 ie_supp_rates[] = {
	1, 8, 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24,
};
static const u8 ie_ext_rates[] = {
	50, 4, 0x30, 0x48, 0x60, 0x6c,
};
static const u8 ie_rsn_ccmp_psk[] = {
	48, 0x14,			/* RSN element, length 20 */
	0x01, 0x00,			/* version 1 */
	0x00, 0x0f, 0xac, 0x04,		/* group cipher suite: CCMP */
	0x01, 0x00,			/* pairwise suite count 1 */
	0x00, 0x0f, 0xac, 0x04,		/* pairwise suite: CCMP */
	0x01, 0x00,			/* AKM suite count 1 */
	0x00, 0x0f, 0xac, 0x02,		/* AKM: PSK */
	0x00, 0x00,			/* RSN capabilities */
};

#define WLAN_CAPABILITY_ESS	0x0001
#define WLAN_CAPABILITY_PRIVACY	0x0010

static int build_assoc(u8 *f, struct mt7601u_connect *c, const u8 *mac)
{
	int n = build_mgmt_hdr(f, FTYPE_MGMT | STYPE_ASSOC_REQ,
			       c->bssid, mac, c->bssid);
	u8 *p = f + n;

	put_unaligned_le16(WLAN_CAPABILITY_ESS |
			   (c->open_net ? 0 : WLAN_CAPABILITY_PRIVACY), p);
	p += 2;
	put_unaligned_le16(10, p);		/* listen interval */
	p += 2;

	*p++ = 0;				/* SSID element */
	*p++ = c->ssid_len;
	memcpy(p, c->ssid, c->ssid_len);
	p += c->ssid_len;

	memcpy(p, ie_supp_rates, sizeof(ie_supp_rates));
	p += sizeof(ie_supp_rates);
	memcpy(p, ie_ext_rates, sizeof(ie_ext_rates));
	p += sizeof(ie_ext_rates);
	if (c->open_net) {
		/* no RSN element */
	} else if (c->rsn_len && c->rsn_len <= sizeof(c->rsn)) {
		memcpy(p, c->rsn, c->rsn_len);
		p += c->rsn_len;
	} else {
		memcpy(p, ie_rsn_ccmp_psk, sizeof(ie_rsn_ccmp_psk));
		p += sizeof(ie_rsn_ccmp_psk);
	}

	return p - f;
}

static int conn_assoc(struct mt7601u_dev *dev, struct mt7601u_connect *c)
{
	u8 frame[256];
	u8 *mpdu;
	u32 mlen = 0, rxi = 0;
	int flen, ret = -ETIMEDOUT, tries, off;
	u16 cap, status, aid;

	conn_stash_flush();
	flen = build_assoc(frame, c, dev->macaddr);

	for (tries = 0; tries < 4; tries++) {
		ret = conn_tx(dev, frame, flen, false, 0xff, MT_RATE_CCK_1M, true);
		if (ret) {
			printf("mt7601u: assoc tx failed:%d\n", ret);
			continue;
		}
		ret = conn_wait_mgmt(dev, STYPE_ASSOC_RESP, &mpdu, &mlen,
				     &rxi, 400, -1);
		if (ret == 0 || ret == -ENODEV || ret == -ECONNREFUSED)
			break;
	}
	if (ret) {
		printf("mt7601u: assoc: no response from %pM\n", c->bssid);
		return ret;
	}

	off = 24 + ((rxi & MT_RXINFO_L2PAD) ? 2 : 0);
	if (mlen < 24 + 6) {
		printf("mt7601u: assoc resp too short (%u)\n", mlen);
		return -EINVAL;
	}
	cap = get_unaligned_le16(mpdu + off);
	status = get_unaligned_le16(mpdu + off + 2);
	aid = get_unaligned_le16(mpdu + off + 4) & 0x3fff;
	printf("mt7601u: assoc resp cap %04x status %u aid %u\n",
	       cap, status, aid);
	if (status != 0)
		return -EACCES;
	if (c->sae)
		printf("mt7601u: associated with PMF\n");

	c->aid = aid;
	c->state = MT_CONN_ASSOCED;
	return 0;
}

/* ----------------------------- milestone 3: WPA2-PSK 4-way handshake */

/* 802.1X / EAPOL-Key (RSN, key descriptor version 2: HMAC-SHA1 + AES wrap) */
static const u8 snap_eapol[8] = {
	0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00, 0x88, 0x8e,
};

#define EAPOL_KEY_TYPE		3
#define KD_DESC_TYPE		2	/* RSN */

/* offsets inside the EAPOL-Key descriptor (after the 4-byte EAPOL header) */
#define KD_INFO		1
#define KD_KEYLEN	3
#define KD_REPLAY	5
#define KD_NONCE	13
#define KD_IV		45
#define KD_RSC		61
#define KD_MIC		77
#define KD_DATALEN	93
#define KD_DATA		95

#define KI_VERSION	0x0007
#define KI_PAIRWISE	0x0008
#define KI_INSTALL	0x0040
#define KI_ACK		0x0080
#define KI_MIC		0x0100
#define KI_SECURE	0x0200
#define KI_ENC		0x1000

#define STYPE_QOS_DATA	0x0080

/* Build 32 bytes of SNonce from timers and our MAC (a fresh per-join value) */
static void conn_make_snonce(struct mt7601u_dev *dev, u8 snonce[32])
{
	static u32 seq;
	sha1_context c;
	u8 seed[32], d[20];
	u64 t = get_ticks();

	memcpy(seed, dev->macaddr, 6);
	put_unaligned_le64(t, seed + 6);
	put_unaligned_le32(get_timer(0), seed + 14);
	put_unaligned_le32(++seq, seed + 18);
	put_unaligned_le64(get_ticks(), seed + 22);

	sha1_starts(&c);
	sha1_update(&c, seed, 30);
	sha1_finish(&c, d);
	memcpy(snonce, d, 20);
	sha1_starts(&c);
	sha1_update(&c, d, 20);
	sha1_update(&c, seed, 30);
	sha1_finish(&c, d);
	memcpy(snonce + 20, d, 12);
}

/*
 * Wait for an EAPOL-Key data frame from the AP addressed to us. Returns a
 * pointer to the EAPOL header (into dev->rx_buf) and the EAPOL byte length.
 */
static int conn_wait_eapol(struct mt7601u_dev *dev, u8 **eapol, u32 *elen,
			   int ms)
{
	ulong start = get_timer(0);

	while (get_timer(start) < (ulong)ms) {
		int actual = 0;
		int ret = xhci_async_bulk_poll(dev->async_rx_h, &actual);
		u8 *data;
		u32 dlen;
		u16 sl;

		if (ret == -EINPROGRESS) {
			udelay(300);
			continue;
		}
		if (ret == -ENODEV) {
			dev->gone = true;
			return -ENODEV;
		}

		if (ret == 0 && actual > 0) {
			data = dev->rx_buf;
			dlen = actual;
			while ((sl = conn_next_seg_len(data, dlen))) {
				u8 *m, *body;
				u32 l, rxi, blen;
				u16 fc, hdrlen;

				if (conn_seg_mpdu(data, sl, &m, &l, &rxi)) {
					fc = get_unaligned_le16(m);
					if ((fc & FC_FTYPE) != FTYPE_DATA ||
					    memcmp(m + 4, dev->macaddr, 6)) {
						data += sl;
						dlen -= sl;
						continue;
					}
					hdrlen = 24;
					if ((fc & FC_STYPE) == STYPE_QOS_DATA)
						hdrlen += 2;
					if (fc & IEEE80211_FCTL_ORDER)
						hdrlen += 4;
					if (l <= hdrlen + 8) {
						data += sl;
						dlen -= sl;
						continue;
					}
					body = m + hdrlen +
					       ((rxi & MT_RXINFO_L2PAD) ? 2 : 0);
					blen = l - hdrlen;
					if (!memcmp(body, snap_eapol, 8)) {
						*eapol = body + 8;
						*elen = blen - 8;
						return 0;
					}
				}
				data += sl;
				dlen -= sl;
			}
		}
		conn_rx_arm(dev);
	}
	return -ETIMEDOUT;
}

/* Wrap an EAPOL payload in LLC/SNAP + an 802.11 data header and transmit it */
static int conn_tx_eapol(struct mt7601u_dev *dev, struct mt7601u_connect *c,
			 const u8 *eapol, int elen)
{
	u8 frame[256];
	int n = 0;

	/* data, ToDS=1: addr1=BSSID(RA) addr2=our MAC(TA) addr3=BSSID(DA) */
	put_unaligned_le16(FTYPE_DATA | 0x0100, frame);
	frame[2] = 0;
	frame[3] = 0;
	memcpy(frame + 4, c->bssid, 6);
	memcpy(frame + 10, dev->macaddr, 6);
	memcpy(frame + 16, c->bssid, 6);
	frame[22] = 0;
	frame[23] = 0;
	n = 24;
	memcpy(frame + n, snap_eapol, 8);
	n += 8;
	memcpy(frame + n, eapol, elen);
	n += elen;

	return conn_tx(dev, frame, n, false, 0xff, MT_RATE_CCK_1M, true);
}

/* Build an EAPOL-Key frame; returns total EAPOL length (header + descriptor) */
static int conn_build_eapol_key(u8 *e, u16 key_info, const u8 *replay,
				const u8 *nonce, u16 key_len,
				const u8 *key_data, int kd_len)
{
	int desc = KD_DATA + kd_len;

	memset(e, 0, 4 + desc);
	e[0] = 2;			/* 802.1X-2004 */
	e[1] = EAPOL_KEY_TYPE;
	put_unaligned_be16(desc, e + 2);

	e[4 + 0] = KD_DESC_TYPE;
	put_unaligned_be16(key_info, e + 4 + KD_INFO);
	put_unaligned_be16(key_len, e + 4 + KD_KEYLEN);
	memcpy(e + 4 + KD_REPLAY, replay, 8);
	if (nonce)
		memcpy(e + 4 + KD_NONCE, nonce, 32);
	put_unaligned_be16(kd_len, e + 4 + KD_DATALEN);
	if (kd_len)
		memcpy(e + 4 + KD_DATA, key_data, kd_len);
	return 4 + desc;
}

/* MIC over the whole EAPOL frame with the MIC field zeroed.
 * WPA2-PSK uses HMAC-SHA1. SAE (AKM 8) uses AES-128-CMAC and key-info version 0.
 */
static void conn_eapol_mic(struct mt7601u_connect *c, u8 *e, int elen)
{
	memset(e + 4 + KD_MIC, 0, 16);
	if (c->sae) {
		sae_aes_cmac(c->ptk, e, elen, e + 4 + KD_MIC);
	} else {
		u8 mac[20];

		wpa_hmac_sha1(c->ptk, 16, e, elen, mac);
		memcpy(e + 4 + KD_MIC, mac, 16);
	}
}

/* ------------------------------------------------- hardware key install */

static void conn_addr_wr(struct mt7601u_dev *dev, u32 offset, const u8 *mac)
{
	mt7601u_wr(dev, offset, get_unaligned_le32(mac));
	mt7601u_wr(dev, offset + 4, mac[4] | mac[5] << 8);
}

static int conn_set_pairwise_key(struct mt7601u_dev *dev, u8 wcid,
				 const u8 *tk)
{
	u8 key_data[32] __aligned(4);
	u8 iv_data[8] __aligned(4);
	u32 val;
	int ret;

	memset(key_data, 0, sizeof(key_data));
	memcpy(key_data, tk, 16);
	ret = mt7601u_burst_write_regs(dev, MT_WCID_KEY(wcid),
				       (const u32 *)key_data, 8);
	if (ret)
		return ret;

	memset(iv_data, 0, sizeof(iv_data));
	iv_data[0] |= 1;		/* PN starts at 1 (CCMP) */
	iv_data[3] = 0x20;		/* ExtIV, key id 0 */
	ret = mt7601u_burst_write_regs(dev, MT_WCID_IV(wcid),
				       (const u32 *)iv_data, 2);
	if (ret)
		return ret;

	val = mt7601u_rr(dev, MT_WCID_ATTR(wcid));
	val &= ~MT_WCID_ATTR_PKEY_MODE & ~MT_WCID_ATTR_PKEY_MODE_EXT;
	val |= FIELD_PREP(MT_WCID_ATTR_PKEY_MODE, MT_CIPHER_AES_CCMP & 7) |
	       FIELD_PREP(MT_WCID_ATTR_PKEY_MODE_EXT, MT_CIPHER_AES_CCMP >> 3);
	val |= MT_WCID_ATTR_PAIRWISE;
	mt7601u_wr(dev, MT_WCID_ATTR(wcid), val);
	return 0;
}

static int conn_set_group_key(struct mt7601u_dev *dev, u8 keyid,
			      const u8 *gtk, int gtk_len)
{
	u8 key_data[32] __aligned(4);
	u32 val;
	int ret;

	memset(key_data, 0, sizeof(key_data));
	memcpy(key_data, gtk, min(gtk_len, 32));
	ret = mt7601u_burst_write_regs(dev, MT_SKEY(0, keyid),
				       (const u32 *)key_data, 8);
	if (ret)
		return ret;

	val = mt7601u_rr(dev, MT_SKEY_MODE(0));
	val &= ~(MT_SKEY_MODE_MASK << MT_SKEY_MODE_SHIFT(0, keyid));
	val |= (u32)MT_CIPHER_AES_CCMP << MT_SKEY_MODE_SHIFT(0, keyid);
	mt7601u_wr(dev, MT_SKEY_MODE(0), val);
	return 0;
}

static int conn_install_keys(struct mt7601u_dev *dev, struct mt7601u_connect *c)
{
	u32 attr;
	int ret;

	/* WCID 1 = the AP, BSS 0 */
	attr = FIELD_PREP(MT_WCID_ATTR_BSS_IDX, 0);
	mt7601u_wr(dev, MT_WCID_ATTR(1), attr);
	conn_addr_wr(dev, MT_WCID_ADDR(1), c->bssid);
	conn_addr_wr(dev, MT_MAC_BSSID_DW0, c->bssid);

	ret = conn_set_pairwise_key(dev, 1, c->ptk + 32);
	if (ret)
		return ret;
	return conn_set_group_key(dev, c->gtk_keyid, c->gtk, c->gtk_len);
}

/* --------------------------------------------------- handshake driver */

static const char *conn_passphrase(void)
{
	const char *p = env_get("wifi_pass");

	return (p && *p) ? p : NULL;
}

static int conn_parse_gtk(struct mt7601u_connect *c, const u8 *kd, int len)
{
	int pos = 0;

	while (pos + 2 <= len) {
		u8 id = kd[pos], l = kd[pos + 1];

		if (pos + 2 + l > len)
			break;
		if (id == 0xdd && l >= 6 &&
		    kd[pos + 2] == 0x00 && kd[pos + 3] == 0x0f &&
		    kd[pos + 4] == 0xac && kd[pos + 5] == 0x01) {
			/* GTK KDE: keyid(2b) + reserved, then GTK */
			c->gtk_keyid = kd[pos + 6] & 0x03;
			c->gtk_len = l - 6;
			if (c->gtk_len > 32)
				c->gtk_len = 32;
			memcpy(c->gtk, kd + pos + 8, c->gtk_len);
			return 0;
		}
		pos += 2 + l;
	}
	return -ENOENT;
}

static int conn_handshake(struct mt7601u_dev *dev, struct mt7601u_connect *c,
			  const char *pass)
{
	u8 eap[256];
	u8 *e;
	u32 elen;
	u16 ki, keylen;
	const u8 *kck, *kek;
	u8 rx_mic[16], calc[20];
	u16 ver;
	const u8 *rsn;
	int rsn_len, ret, len, tries;

	if (!c->sae)
		wpa_pmk(pass, (const u8 *)c->ssid, c->ssid_len, c->pmk);
	conn_make_snonce(dev, c->snonce);
	kck = c->ptk;
	kek = c->ptk + 16;
	ver = c->sae ? 0 : 2;
	rsn = c->rsn_len ? c->rsn : ie_rsn_ccmp_psk;
	rsn_len = c->rsn_len ? c->rsn_len : (int)sizeof(ie_rsn_ccmp_psk);

	/* message 1: ANonce from the AP */
	ret = conn_wait_eapol(dev, &e, &elen, 3000);
	if (ret) {
		printf("mt7601u: handshake: no EAPOL message 1\n");
		return ret;
	}
	ki = get_unaligned_be16(e + 4 + KD_INFO);
	if (!(ki & KI_ACK) || (ki & KI_MIC)) {
		printf("mt7601u: handshake: unexpected key_info %04x for msg1\n",
		       ki);
		return -EPROTO;
	}
	keylen = get_unaligned_be16(e + 4 + KD_KEYLEN);
	memcpy(c->anonce, e + 4 + KD_NONCE, 32);
	memcpy(c->replay, e + 4 + KD_REPLAY, 8);

	if (c->sae)
		sae_ptk_sha256(c->pmk, c->bssid, dev->macaddr, c->anonce,
			       c->snonce, c->ptk, 48);
	else
		wpa_ptk(c->pmk, c->bssid, dev->macaddr, c->anonce, c->snonce,
			c->ptk, 48);

	/*
	 * message 2: SNonce + our RSN element, MIC'd with KCK. The AP
	 * retransmits message 1 (with a fresh replay counter) if our message 2
	 * is lost or arrives late -- common right after association. Reply
	 * msg2 against the latest msg1 and keep waiting for msg3 rather than
	 * treating a retransmitted msg1 as a protocol error.
	 */
	for (tries = 0; ; tries++) {
		len = conn_build_eapol_key(eap, ver | KI_PAIRWISE | KI_MIC,
					   c->replay, c->snonce, keylen,
					   rsn, rsn_len);
		conn_eapol_mic(c, eap, len);
		ret = conn_tx_eapol(dev, c, eap, len);
		if (ret) {
			printf("mt7601u: handshake: msg2 tx failed:%d\n", ret);
			return ret;
		}

		/* message 3: GTK (AES-wrapped), MIC'd */
		ret = conn_wait_eapol(dev, &e, &elen, 2000);
		if (ret) {
			printf("mt7601u: handshake: no EAPOL message 3\n");
			return ret;
		}
		ki = get_unaligned_be16(e + 4 + KD_INFO);
		if ((ki & KI_MIC) && (ki & KI_INSTALL))
			break;			/* message 3 */
		if ((ki & KI_ACK) && !(ki & KI_MIC) && tries < 4) {
			/* a retransmitted message 1: refresh ANonce + replay
			 * counter, recompute the PTK, and resend msg2 */
			keylen = get_unaligned_be16(e + 4 + KD_KEYLEN);
			memcpy(c->anonce, e + 4 + KD_NONCE, 32);
			memcpy(c->replay, e + 4 + KD_REPLAY, 8);
			if (c->sae)
				sae_ptk_sha256(c->pmk, c->bssid, dev->macaddr,
					       c->anonce, c->snonce, c->ptk, 48);
			else
				wpa_ptk(c->pmk, c->bssid, dev->macaddr,
					c->anonce, c->snonce, c->ptk, 48);
			continue;
		}
		printf("mt7601u: handshake: unexpected key_info %04x for msg3\n",
		       ki);
		return -EPROTO;
	}

	/* verify the AP's MIC with our KCK (proves the passphrase matched) */
	memcpy(rx_mic, e + 4 + KD_MIC, 16);
	memset(e + 4 + KD_MIC, 0, 16);
	if (c->sae) {
		sae_aes_cmac(kck, e, elen, calc);
	} else {
		wpa_hmac_sha1(kck, 16, e, elen, calc);
	}
	if (memcmp(rx_mic, calc, 16)) {
		printf("mt7601u: handshake: msg3 MIC mismatch (wrong passphrase?)\n");
		return -EACCES;
	}

	/* decrypt the key data and extract the GTK */
	{
		u16 kdl = get_unaligned_be16(e + 4 + KD_DATALEN);
		u8 plain[256];

		if (ki & KI_ENC) {
			if (kdl < 16 || (kdl & 7) || kdl > sizeof(plain) + 8) {
				printf("mt7601u: handshake: bad key-data len %u\n",
				       kdl);
				return -EINVAL;
			}
			if (wpa_aes_unwrap(kek, kdl / 8 - 1,
					   e + 4 + KD_DATA, plain)) {
				printf("mt7601u: handshake: key-data unwrap failed\n");
				return -EINVAL;
			}
			ret = conn_parse_gtk(c, plain, kdl - 8);
		} else {
			ret = conn_parse_gtk(c, e + 4 + KD_DATA, kdl);
		}
		if (ret) {
			printf("mt7601u: handshake: no GTK in message 3\n");
			return ret;
		}
	}
	memcpy(c->replay, e + 4 + KD_REPLAY, 8);

	/*
	 * message 4: acknowledge. The AP retransmits message 3 when this
	 * frame is lost; answer each one. wifi_eapol_drop_msg4=1 holds the
	 * first message 4 so that retransmit can be measured. Leave it unset.
	 */
	{
		const char *drop = env_get("wifi_eapol_drop_msg4");
		int hold = drop && drop[0] == '1' && !drop[1];
		int held = 0;
		int sent = 0;

		for (;;) {
			if (hold && !held) {
				printf("mt7601u: handshake: holding the first msg4\n");
				held = 1;
			} else {
				len = conn_build_eapol_key(eap,
					ver | KI_PAIRWISE | KI_MIC | KI_SECURE,
					c->replay, NULL, keylen, NULL, 0);
				conn_eapol_mic(c, eap, len);
				ret = conn_tx_eapol(dev, c, eap, len);
				if (ret) {
					printf("mt7601u: handshake: msg4 tx failed:%d\n",
					       ret);
					return ret;
				}
				if (sent)
					printf("mt7601u: handshake: msg4 resent\n");
				sent++;
			}
			if (sent >= 4)
				break;
			conn_rx_arm(dev);
			ret = conn_wait_eapol(dev, &e, &elen,
					      (hold && sent == 0) ? 2000 : 200);
			if (ret) {
				if (sent == 0) {
					hold = 0;
					continue;
				}
				break;
			}
			ki = get_unaligned_be16(e + 4 + KD_INFO);
			if (!((ki & KI_MIC) && (ki & KI_INSTALL) && (ki & KI_ACK)))
				break;
			memcpy(rx_mic, e + 4 + KD_MIC, 16);
			memset(e + 4 + KD_MIC, 0, 16);
			if (c->sae)
				sae_aes_cmac(kck, e, elen, calc);
			else
				wpa_hmac_sha1(kck, 16, e, elen, calc);
			if (memcmp(rx_mic, calc, 16)) {
				printf("mt7601u: handshake: msg3 retransmit MIC mismatch\n");
				return -EACCES;
			}
			memcpy(c->replay, e + 4 + KD_REPLAY, 8);
			printf("mt7601u: handshake: msg3 retransmit\n");
		}
	}

	ret = conn_install_keys(dev, c);
	if (ret) {
		printf("mt7601u: handshake: key install failed:%d\n", ret);
		return ret;
	}

	c->state = MT_CONN_READY;
	printf("mt7601u: %s handshake complete; pairwise + group (id %u, %u B) keys installed\n",
	       c->sae ? "SAE-AKM 4-way" : "WPA2", c->gtk_keyid, c->gtk_len);
	return 0;
}

/* --------------------------- milestone 4: eth-uclass data path */

/*
 * A small RX ring of translated Ethernet frames. eth recv() drains it one
 * frame per call; when it empties, one poll of the packet-RX async TD refills
 * it from whatever the dongle has delivered (hardware has already decrypted).
 */
#define ETHQ		256
#define ETHBUF		1600

static struct mt7601u_dev *g_dev;
static struct mt7601u_connect *g_conn;
static u8 ethq[ETHQ][ETHBUF];
static u16 ethqlen[ETHQ];
static int ethq_head, ethq_tail;
static struct udevice *eth_bound;
static u32 g_tx_frames, g_rx_drop;

/* Translate one received 802.11 data frame to Ethernet and enqueue it */
static void conn_rx_enqueue(struct mt7601u_dev *dev, u8 *m, u32 l, u32 rxi)
{
	static const u8 snap3[3] = { 0xaa, 0xaa, 0x03 };
	u16 fc = get_unaligned_le16(m);
	u16 hdrlen = 24;
	u8 *body, *out;
	u32 blen, paylen;
	int slot;

	if ((fc & FC_FTYPE) == FTYPE_MGMT) {
		u16 st = fc & FC_STYPE;

		/*
		 * After the handshake the data path used to drop these, so
		 * the link stayed "ready" after the AP had kicked it.
		 */
		if ((st == STYPE_DEAUTH || st == STYPE_DISASSOC) &&
		    l >= 24 && g_conn && g_conn->state == MT_CONN_READY &&
		    !memcmp(m + 4, dev->macaddr, 6) &&
		    !memcmp(m + 16, g_conn->bssid, 6)) {
			g_conn->state = MT_CONN_IDLE;
			printf("mt7601u: AP sent %s while associated\n",
			       st == STYPE_DEAUTH ? "deauth" : "disassoc");
		}
		return;
	}
	if ((fc & FC_FTYPE) != FTYPE_DATA)
		return;
	/* unicast to us, or group-addressed (broadcast/multicast) */
	if (memcmp(m + 4, dev->macaddr, 6) && !(m[4] & 0x01))
		return;
	if ((fc & FC_STYPE) == STYPE_QOS_DATA)
		hdrlen += 2;
	if (fc & IEEE80211_FCTL_ORDER)
		hdrlen += 4;
	if (l <= hdrlen + 8)
		return;

	body = m + hdrlen + ((rxi & MT_RXINFO_L2PAD) ? 2 : 0);
	blen = l - hdrlen;
	/* if the CCMP header was not stripped, SNAP is 8 bytes further in */
	if (memcmp(body, snap3, 3) && blen > 8 &&
	    !memcmp(body + 8, snap3, 3)) {
		body += 8;
		blen -= 8;
	}
	if (blen <= 8 || memcmp(body, snap3, 3))
		return;

	paylen = blen - 8;
	if (paylen > ETHBUF - 14)
		return;

	slot = (ethq_head + 1) % ETHQ;
	if (slot == ethq_tail) {
		g_rx_drop++;
		return;			/* ring full, drop */
	}

	g_rx_frames++;
	out = ethq[ethq_head];
	memcpy(out, m + 4, 6);		/* DA = addr1 */
	memcpy(out + 6, m + 16, 6);	/* SA = addr3 (FromDS) */
	out[12] = body[6];		/* EtherType from SNAP */
	out[13] = body[7];
	memcpy(out + 14, body + 8, paylen);
	ethqlen[ethq_head] = 14 + paylen;
	ethq_head = slot;
}

static int mt_eth_start(struct udevice *edev)
{
	ethq_head = 0;
	ethq_tail = 0;
	if (!g_dev || g_dev->gone || !g_conn ||
	    g_conn->state != MT_CONN_READY) {
		printf("mt7601u: link down\n");
		return -ENODEV;
	}
	return 0;
}

static void mt_eth_stop(struct udevice *edev)
{
}

static int mt_eth_send(struct udevice *edev, void *packet, int length)
{
	struct mt7601u_dev *dev = g_dev;
	u8 *p = packet;
	u8 frame[ETHBUF];
	int n;

	if (!dev || dev->gone || !g_conn || g_conn->state != MT_CONN_READY)
		return -ENODEV;
	if (length < 14 || length > 1514)
		return -EINVAL;

	/* data, ToDS=1. Protected only when the BSS uses a pairwise key. */
	put_unaligned_le16(FTYPE_DATA | 0x0100 |
			   (g_conn->open_net ? 0 : 0x4000), frame);
	frame[2] = 0;
	frame[3] = 0;
	memcpy(frame + 4, g_conn->bssid, 6);
	memcpy(frame + 10, dev->macaddr, 6);
	memcpy(frame + 16, p, 6);
	frame[22] = 0;
	frame[23] = 0;
	n = 24;
	frame[n++] = 0xaa;
	frame[n++] = 0xaa;
	frame[n++] = 0x03;
	frame[n++] = 0x00;
	frame[n++] = 0x00;
	frame[n++] = 0x00;
	frame[n++] = p[12];		/* EtherType into SNAP */
	frame[n++] = p[13];
	memcpy(frame + n, p + 14, length - 14);
	n += length - 14;

	g_tx_frames++;
	return conn_tx(dev, frame, n, true, 1, MT_RATE_OFDM_24M, false);
}

/*
 * Poll one RX handle. On completion, parse its URB into the eth ring and
 * re-arm that handle (its buffer is free only after parsing, so re-arm after;
 * the OTHER handle stays posted meanwhile, keeping the endpoint non-empty).
 * Returns -ENODEV if the controller vanished, else 0.
 */
static int conn_rx_pump(struct mt7601u_dev *dev, int h, u8 *buf)
{
	int actual = 0;
	int ret = xhci_async_bulk_poll(h, &actual);

	if (ret == 0 && actual > 0) {
		u8 *data = buf;
		u32 dlen = actual;
		u16 sl;

		while ((sl = conn_next_seg_len(data, dlen))) {
			u8 *mm;
			u32 ll, rr;

			if (conn_seg_mpdu(data, sl, &mm, &ll, &rr))
				conn_rx_enqueue(dev, mm, ll, rr);
			data += sl;
			dlen -= sl;
		}
		xhci_async_bulk_submit(h, buf, MT_RX_URB_SIZE);
	} else if (ret == -ENODEV) {
		return -ENODEV;
	} else if (ret != -EINPROGRESS) {
		xhci_async_bulk_submit(h, buf, MT_RX_URB_SIZE);
	}
	return 0;
}

/* Re-arm packet RX while the prompt is idle. Both posted TDs complete
 * on beacons, and with nobody polling the endpoint goes empty and the
 * dongle's USB RX DMA sticks. dhcp then transmits and hears nothing.
 */
static void conn_rx_tick(struct cyclic_info *c)
{
	struct mt7601u_dev *dev = g_dev;

	(void)c;

	if (!dev || dev->gone || dev->async_rx_h < 0)
		return;
	if (conn_rx_pump(dev, dev->async_rx_h, dev->rx_buf)) {
		dev->gone = true;
		return;
	}
	if (dev->async_rx_h2 >= 0 &&
	    conn_rx_pump(dev, dev->async_rx_h2, dev->rx_buf2))
		dev->gone = true;
}

static int mt_eth_recv(struct udevice *edev, int flags, uchar **packetp)
{
	struct mt7601u_dev *dev = g_dev;

	if (!dev || dev->gone)
		return -EAGAIN;

	/*
	 * Service the RX pool on EVERY call, not only when the ring is empty:
	 * a completed TD must be re-armed promptly or the endpoint goes empty
	 * and the dongle's USB RX DMA wedges (rx_busy stuck). Polling a TD that
	 * is still in flight is a cheap -EINPROGRESS. The ring (large, below)
	 * absorbs the whole-URB bursts; conn_rx_enqueue drops on a full ring,
	 * which TCP recovers from -- a wedge it cannot.
	 */
	if (conn_rx_pump(dev, dev->async_rx_h, dev->rx_buf)) {
		dev->gone = true;
		return -EAGAIN;
	}
	if (dev->async_rx_h2 >= 0 &&
	    conn_rx_pump(dev, dev->async_rx_h2, dev->rx_buf2)) {
		dev->gone = true;
		return -EAGAIN;
	}

	if (ethq_head == ethq_tail)
		return -EAGAIN;

	*packetp = ethq[ethq_tail];
	return ethqlen[ethq_tail];
}

static int mt_eth_free_pkt(struct udevice *edev, uchar *packet, int length)
{
	if (ethq_head != ethq_tail)
		ethq_tail = (ethq_tail + 1) % ETHQ;
	return 0;
}

static int mt_eth_write_hwaddr(struct udevice *edev)
{
	return 0;
}

static const struct eth_ops mt_eth_ops = {
	.start		= mt_eth_start,
	.send		= mt_eth_send,
	.recv		= mt_eth_recv,
	.free_pkt	= mt_eth_free_pkt,
	.stop		= mt_eth_stop,
	.write_hwaddr	= mt_eth_write_hwaddr,
};

U_BOOT_DRIVER(mt7601u_eth) = {
	.name		= "mt7601u_eth",
	.id		= UCLASS_ETH,
	.ops		= &mt_eth_ops,
	.plat_auto	= sizeof(struct eth_pdata),
};

static int conn_bind_eth(struct mt7601u_dev *dev)
{
	struct eth_pdata *pdata;
	int ret;

	if (!eth_bound) {
		ret = device_bind_driver(dm_root(), "mt7601u_eth", "wifi0",
					 &eth_bound);
		if (ret) {
			printf("mt7601u: eth bind failed:%d\n", ret);
			eth_bound = NULL;
			return ret;
		}
	}

	pdata = dev_get_plat(eth_bound);
	memcpy(pdata->enetaddr, dev->macaddr, 6);

	if (!device_active(eth_bound)) {
		ret = device_probe(eth_bound);
		if (ret) {
			printf("mt7601u: eth probe failed:%d\n", ret);
			return ret;
		}
	}
	eth_set_dev(eth_bound);
	env_set("ethact", eth_bound->name);

	/*
	 * Post a SECOND RX TD on the packet-RX endpoint. The data path then
	 * keeps both posted (h1 is already armed from the handshake's last RX),
	 * so the endpoint is never left empty -- which is what wedges the
	 * dongle's USB RX DMA under a sustained bulk download. Degrade to a
	 * single TD if the open fails.
	 */
	if (dev->async_rx_h2 < 0) {
		struct usb_device *udev = dev->udev;
		int h = xhci_async_bulk_open(udev,
			usb_rcvbulkpipe(udev,
					dev->in_eps[MT_EP_IN_PKT_RX] & 0xf));

		if (h >= 0) {
			dev->async_rx_h2 = h;
			xhci_async_bulk_submit(h, dev->rx_buf2, MT_RX_URB_SIZE);
		}
	}
	cyclic_register(&conn_rx_cyc, conn_rx_tick, 2000, "mt7601u_rx");

	printf("mt7601u: net device '%s' ready (%pM); run 'dhcp'\n",
	       eth_bound->name, dev->macaddr);
	return 0;
}

/* ------------------------------------------------------------ SAE */

static void conn_fill_entropy(struct mt7601u_dev *dev, u8 out[32], u32 salt)
{
	sha1_context c;
	u8 seed[32], d[20];

	memcpy(seed, dev->macaddr, 6);
	put_unaligned_le64(get_ticks(), seed + 6);
	put_unaligned_le32(get_timer(0), seed + 14);
	put_unaligned_le32(salt, seed + 18);
	sha1_starts(&c);
	sha1_update(&c, seed, 22);
	sha1_finish(&c, d);
	memcpy(out, d, 20);
	sha1_starts(&c);
	sha1_update(&c, d, 20);
	sha1_update(&c, seed, 22);
	sha1_finish(&c, d);
	memcpy(out + 20, d, 12);
}

/* RSN IE: CCMP, AKM SAE, MFPC|MFPR, one PMKID, BIP-CMAC-128. */
static int conn_build_rsn_sae(u8 *ie, const u8 pmkid[16])
{
	static const u8 tail[] = {
		0x01, 0x00,			/* version */
		0x00, 0x0f, 0xac, 0x04,		/* group CCMP */
		0x01, 0x00,
		0x00, 0x0f, 0xac, 0x04,		/* pairwise CCMP */
		0x01, 0x00,
		0x00, 0x0f, 0xac, 0x08,		/* AKM SAE */
		0xc0, 0x00,			/* MFPC | MFPR */
		0x01, 0x00,			/* one PMKID */
	};
	static const u8 mgmt[] = { 0x00, 0x0f, 0xac, 0x06 };
	u8 *p = ie;

	*p++ = 48;
	*p++ = sizeof(tail) + 16 + sizeof(mgmt);
	memcpy(p, tail, sizeof(tail));
	p += sizeof(tail);
	memcpy(p, pmkid, 16);
	p += 16;
	memcpy(p, mgmt, sizeof(mgmt));
	p += sizeof(mgmt);
	return p - ie;
}

static int conn_sae_tx(struct mt7601u_dev *dev, const u8 *frame, int flen)
{
	int tries, ret = -ETIMEDOUT;

	for (tries = 0; tries < 4; tries++) {
		ret = conn_tx(dev, frame, flen, false, 0xff, MT_RATE_CCK_1M,
			      true);
		if (!ret)
			return 0;
		printf("mt7601u: SAE tx failed:%d\n", ret);
	}
	return ret;
}

/*
 * SAE commit then confirm. Status 0 on both is hunting-and-pecking success.
 * An anti-clogging token (status 76) is echoed once. Status 126 (hash-to-element
 * only) is reported and not treated as a join.
 */
static int conn_sae_mode(struct mt7601u_dev *dev, struct mt7601u_connect *c,
			  const u8 *pass, int pass_len, int h2e, u16 tx_status)
{
	u8 frame[400];
	u8 entropy[32], rand_s[32], mask[32];
	u8 pwe_x[32], pwe_y[32], scalar[32], element[64];
	u8 peer_sc[32], peer_el[64], kck[32], pmkid[16];
	u8 token[128];
	int token_len = 0;
	int round, ret;

	if (!pass || pass_len < 1) {
		printf("mt7601u: SAE needs wifi_pass\n");
		return -EINVAL;
	}
	printf("mt7601u: SAE %s status %u len %d\n",
	       h2e ? "hash-to-element" : "hunting-and-pecking", tx_status,
	       pass_len);
	conn_stash_flush();
	if (h2e)
		ret = sae_derive_pwe_h2e(dev->macaddr, c->bssid, c->ssid,
					 c->ssid_len, pass, pass_len,
					 pwe_x, pwe_y);
	else
		ret = sae_derive_pwe(dev->macaddr, c->bssid, pass, pass_len,
				     pwe_x, pwe_y);
	if (ret) {
		printf("mt7601u: SAE: no PWE\n");
		return -EINVAL;
	}
	conn_fill_entropy(dev, entropy, 1);
	if (sae_random_scalar(entropy, rand_s))
		return -EINVAL;
	conn_fill_entropy(dev, entropy, 2);
	if (sae_random_scalar(entropy, mask))
		return -EINVAL;
	if (sae_build_commit(pwe_x, pwe_y, rand_s, mask, scalar, element)) {
		printf("mt7601u: SAE: commit build failed\n");
		return -EINVAL;
	}

	for (round = 0; round < 3; round++) {
		u8 *mpdu, *body;
		u32 mlen = 0, rxi = 0;

		/* A confirm stashed for a commit the AP rejected is not this round's. */
		conn_stash_flush();
		u16 alg, seq, status, group;
		int n, off, blen;

		n = build_mgmt_hdr(frame, FTYPE_MGMT | STYPE_AUTH,
				   c->bssid, dev->macaddr, c->bssid);
		put_unaligned_le16(3, frame + n);
		put_unaligned_le16(1, frame + n + 2);
		put_unaligned_le16(tx_status, frame + n + 4);
		n += 6;
		put_unaligned_le16(19, frame + n);
		n += 2;
		if (token_len) {
			memcpy(frame + n, token, token_len);
			n += token_len;
		}
		memcpy(frame + n, scalar, 32);
		n += 32;
		memcpy(frame + n, element, 64);
		n += 64;

		ret = conn_sae_tx(dev, frame, n);
		if (ret)
			return ret;
		ret = conn_wait_mgmt(dev, STYPE_AUTH, &mpdu, &mlen, &rxi,
				     800, 1);
		if (ret == -ETIMEDOUT) {
			printf("mt7601u: SAE commit again\n");
			ret = conn_sae_tx(dev, frame, n);
			if (ret)
				return ret;
			ret = conn_wait_mgmt(dev, STYPE_AUTH, &mpdu, &mlen,
					     &rxi, 800, 1);
		}
		if (ret) {
			printf("mt7601u: SAE: no commit response\n");
			return ret;
		}
		off = 24 + ((rxi & MT_RXINFO_L2PAD) ? 2 : 0);
		if ((int)mlen < off + 6) {
			printf("mt7601u: SAE commit too short\n");
			return -EINVAL;
		}
		alg = get_unaligned_le16(mpdu + off);
		seq = get_unaligned_le16(mpdu + off + 2);
		status = get_unaligned_le16(mpdu + off + 4);
		body = mpdu + off + 6;
		blen = (int)mlen - off - 6;
		printf("mt7601u: SAE commit status %u\n", status);
		if (alg != 3 || seq != 1) {
			printf("mt7601u: SAE commit alg %u seq %u\n", alg, seq);
			return -EPROTO;
		}
		if (status == 76 && blen >= 2 && round + 1 < 3) {
			token_len = blen - 2;
			if (token_len > (int)sizeof(token))
				token_len = sizeof(token);
			memcpy(token, body + 2, token_len);
			continue;
		}
		/* 126 is the peer agreeing to hash-to-element. */
		if (status != 0 && !(h2e && status == 126))
			return -EACCES;
		if (blen < 2 + 32 + 64) {
			printf("mt7601u: SAE commit body short\n");
			return -EINVAL;
		}
		group = get_unaligned_le16(body);
		if (group != 19) {
			printf("mt7601u: SAE group %u\n", group);
			return -EPROTONOSUPPORT;
		}
		memcpy(peer_sc, body + 2, 32);
		memcpy(peer_el, body + 34, 64);
		goto have_peer;
	}
	printf("mt7601u: SAE: commit not accepted\n");
	return -EACCES;

have_peer:
	if (sae_process_commit(pwe_x, pwe_y, rand_s, scalar, element,
			       peer_sc, peer_el, kck, c->pmk, pmkid)) {
		printf("mt7601u: SAE: peer commit rejected\n");
		return -EACCES;
	}
	{
		u8 confirm[32];
		u8 *mpdu, *body;
		u32 mlen = 0, rxi = 0;
		u16 alg, seq, status, scnt;
		u8 expect[32];
		int n, off, blen;

		sae_confirm(kck, 0, scalar, element, peer_sc, peer_el, confirm);
		n = build_mgmt_hdr(frame, FTYPE_MGMT | STYPE_AUTH,
				   c->bssid, dev->macaddr, c->bssid);
		put_unaligned_le16(3, frame + n);
		put_unaligned_le16(2, frame + n + 2);
		put_unaligned_le16(0, frame + n + 4);
		n += 6;
		put_unaligned_le16(0, frame + n);	/* send-confirm */
		n += 2;
		memcpy(frame + n, confirm, 32);
		n += 32;
		ret = conn_sae_tx(dev, frame, n);
		if (ret)
			return ret;
		ret = conn_wait_mgmt(dev, STYPE_AUTH, &mpdu, &mlen, &rxi,
				     800, 2);
		if (ret == -ETIMEDOUT) {
			printf("mt7601u: SAE confirm again\n");
			ret = conn_sae_tx(dev, frame, n);
			if (ret)
				return ret;
			ret = conn_wait_mgmt(dev, STYPE_AUTH, &mpdu, &mlen,
					     &rxi, 800, 2);
		}
		if (ret) {
			printf("mt7601u: SAE: no confirm response\n");
			return ret;
		}
		off = 24 + ((rxi & MT_RXINFO_L2PAD) ? 2 : 0);
		if ((int)mlen < off + 6) {
			printf("mt7601u: SAE confirm too short\n");
			return -EINVAL;
		}
		alg = get_unaligned_le16(mpdu + off);
		seq = get_unaligned_le16(mpdu + off + 2);
		status = get_unaligned_le16(mpdu + off + 4);
		body = mpdu + off + 6;
		blen = (int)mlen - off - 6;
		printf("mt7601u: SAE confirm alg %u seq %u status %u\n",
		       alg, seq, status);
		if (status != 0)
			return -EACCES;
		if (alg != 3 || seq != 2)
			return -EPROTO;
		if (blen < 2 + 32) {
			printf("mt7601u: SAE confirm body short\n");
			return -EINVAL;
		}
		scnt = get_unaligned_le16(body);
		sae_confirm(kck, scnt, peer_sc, peer_el, scalar, element,
			    expect);
		if (memcmp(expect, body + 2, 32)) {
			printf("mt7601u: SAE confirm mismatch\n");
			return -EACCES;
		}
	}

	c->rsn_len = conn_build_rsn_sae(c->rsn, pmkid);
	c->state = MT_CONN_AUTHED;
	return 0;
}

static int conn_sae_form(struct mt7601u_dev *dev, struct mt7601u_connect *c,
			  const u8 *pass, int pass_len, const char *form)
{
	int ret;

	printf("mt7601u: SAE form %s\n", form);
	ret = conn_sae_mode(dev, c, pass, pass_len, 0, 0);
	if (!ret)
		return 0;
	return conn_sae_mode(dev, c, pass, pass_len, 1, 0);
}

static int conn_sae(struct mt7601u_dev *dev, struct mt7601u_connect *c)
{
	const char *pass = conn_passphrase();

	if (!pass) {
		printf("mt7601u: SAE needs wifi_pass\n");
		return -EINVAL;
	}
	/* WPA3-Personal uses the configured passphrase only. */
	return conn_sae_form(dev, c, (const u8 *)pass, strlen(pass),
			     "passphrase");
}

/* ------------------------------------------------------------ command */

/*
 * The strongest BSS carrying @ssid (peak RSSI over the scan): a network with
 * several APs or a mesh lists one row per BSSID, and the first one scanned
 * need not be the one in reach.
 */
static struct mt7601u_bss *conn_find_bss(struct mt7601u_dev *dev,
					 const char *ssid)
{
	struct mt7601u_bss *best = NULL;
	int i, n = strlen(ssid), matches = 0;

	for (i = 0; i < dev->scan.n_bss; i++) {
		struct mt7601u_bss *b = &dev->scan.bss[i];

		if (b->ssid_len != n || memcmp(b->ssid, ssid, n))
			continue;
		matches++;
		if (!best || b->rssi_max > best->rssi_max)
			best = b;
	}
	if (matches > 1)
		printf("mt7601u: %d BSSs carry '%s'; taking the strongest (%d dBm peak)\n",
		       matches, ssid, best->rssi_max);
	return best;
}

/* whole auth + assoc + handshake sequences before 'wifi join' gives up */
#define MT_JOIN_ATTEMPTS	3

static struct mt7601u_bss *conn_find_bssid(struct mt7601u_dev *dev,
					   const u8 *bssid)
{
	int i;

	for (i = 0; i < dev->scan.n_bss; i++)
		if (!memcmp(dev->scan.bss[i].bssid, bssid, 6))
			return &dev->scan.bss[i];
	return NULL;
}

/* Scan-table RSN decides the join. No row means the explicit form has no AKM. */
static void conn_apply_akm(struct mt7601u_bss *b)
{
	if (!b) {
		printf("mt7601u: RSN not in the scan table; joining as WPA2\n");
		return;
	}
	if (b->akm_sae && !b->akm_psk) {
		conn.sae = 1;
		printf("mt7601u: BSS RSN is SAE only; WPA3-Personal\n");
	} else if (b->akm_sae && b->akm_psk) {
		printf("mt7601u: BSS advertises PSK and SAE; joining as WPA2\n");
	} else if (b->akm_psk) {
		printf("mt7601u: BSS RSN is PSK; WPA2\n");
	} else {
		conn.open_net = 1;
		printf("mt7601u: BSS has no RSN; joining open\n");
	}
}

static int conn_join(struct mt7601u_dev *dev, const char *ssid,
		     const u8 *bssid, u8 chan)
{
	int ret, attempt;
	struct mt7601u_bss *b = NULL;

	if (dev->scan.running)
		mt7601u_scan_stop(dev, true);

	/*
	 * A missing SSID, or an explicit BSSID that is not in the scan
	 * table, must not drop a live association. Look both up before
	 * packet RX is closed and conn is zeroed. ENOENT leaves the
	 * association that is already up.
	 */
	if (bssid && chan) {
		b = conn_find_bssid(dev, bssid);
		if (!b) {
			printf("mt7601u: BSSID %pM is not in the scan table\n",
			       bssid);
			return -ENOENT;
		}
	} else {
		b = conn_find_bss(dev, ssid);
		if (!b) {
			printf("mt7601u: '%s' not in the scan table; run 'wifi scan start' then 'wifi scan stop' first, or name a bssid and channel: 'wifi join %s <bssid> <chan>'\n",
			       ssid, ssid);
			return -ENOENT;
		}
	}

	/*
	 * A previous join still owns the packet-RX TDs. Forgetting the
	 * handles and switching channel with MAC RX on makes the next MCU
	 * command OUT return -110.
	 */
	if (dev->async_rx_h >= 0)
		conn_close_rx(dev);

	memset(&conn, 0, sizeof(conn));

	if (bssid && chan) {
		/*
		 * Explicit BSSID + channel: join with a single channel set.
		 * The scan row found above supplies the AKM. A miss already
		 * returned without closing the live join or falling back to
		 * WPA2. The channel-hopping background scan stays off (that
		 * rapid MCU channel switching is what intermittently wedges
		 * the dongle's RX DMA).
		 */
		memcpy(conn.bssid, bssid, 6);
		conn.chan = chan;
		conn.ssid_len = min_t(u8, strlen(ssid), 32);
		memcpy(conn.ssid, ssid, conn.ssid_len);
		conn_apply_akm(b);
	} else {
		memcpy(conn.bssid, b->bssid, 6);
		conn.chan = b->chan;
		conn.ssid_len = min_t(u8, b->ssid_len, 32);
		memcpy(conn.ssid, b->ssid, conn.ssid_len);
		conn_apply_akm(b);
	}
	if (!conn.sae && !conn.open_net) {
		memcpy(conn.rsn, ie_rsn_ccmp_psk, sizeof(ie_rsn_ccmp_psk));
		conn.rsn_len = sizeof(ie_rsn_ccmp_psk);
	}

	printf("mt7601u: joining '%s' bssid %pM chan %u\n",
	       conn.ssid, conn.bssid, conn.chan);

	dev->busy++;
	dev->gone = false;
	dev->async_rx_h = -1;
	dev->async_rx_h2 = -1;
	g_dev = dev;
	g_conn = &conn;

	ret = mt7601u_phy_set_channel(dev, conn.chan);
	if (ret) {
		printf("mt7601u: set channel %u failed:%d\n", conn.chan, ret);
		goto out;
	}
	ret = mt7601u_mac_start_rx(dev);
	if (ret) {
		printf("mt7601u: MAC RX start failed:%d\n", ret);
		goto out;
	}
	/* mt7601u_mac_start_rx() enabled RX only; the join path also transmits */
	mt76_set(dev, MT_MAC_SYS_CTRL,
		 MT_MAC_SYS_CTRL_ENABLE_TX | MT_MAC_SYS_CTRL_ENABLE_RX);

	ret = conn_open_rx(dev);
	if (ret)
		goto out;

	/*
	 * Each step retries its own frames, but a failed handshake (the AP
	 * gave up on us, or its message 1 went out before we listened) ends
	 * the whole session. Start over from authentication: the AP drops
	 * the old association when a new authentication arrives.
	 */
	for (attempt = 1; attempt <= MT_JOIN_ATTEMPTS; attempt++) {
		if (attempt > 1) {
			printf("mt7601u: join failed (%d); attempt %d of %d\n",
			       ret, attempt, MT_JOIN_ATTEMPTS);
			memset(&conn.state, 0, sizeof(conn) -
			       offsetof(struct mt7601u_connect, state));
		}
		ret = conn.sae ? conn_sae(dev, &conn) : conn_auth(dev, &conn);
		if (!ret)
			ret = conn_assoc(dev, &conn);
		if (!ret) {
			const char *pass = conn_passphrase();

			if (conn.open_net) {
				conn.state = MT_CONN_READY;
				printf("mt7601u: open association complete\n");
			} else if (pass) {
				ret = conn_handshake(dev, &conn, pass);
			} else {
				printf("mt7601u: associated (open); set 'wifi_pass' for a WPA2 network\n");
			}
		}
		if (!ret || dev->gone)
			break;
	}

	if (!ret && conn.state == MT_CONN_READY) {
		/* keep the RX TD armed for the data path, expose an eth device */
		ethq_head = 0;
		ethq_tail = 0;
		conn_bind_eth(dev);
	} else {
		conn_close_rx(dev);
	}
out:
	dev->busy--;
	return ret;
}

/*
 * Release the data-path RX TD and connection state. Called by 'wifi init'
 * before it tears the device down, so the async bulk-IN slot the eth path
 * held is freed and a fresh scan/join can reopen it.
 */
void mt7601u_connect_stop(struct mt7601u_dev *dev)
{
	if (dev->async_rx_h >= 0)
		conn_close_rx(dev);
	conn.state = MT_CONN_IDLE;
	ethq_head = 0;
	ethq_tail = 0;
	g_dev = NULL;
	g_conn = NULL;
}

int mt7601u_connect_cmd(struct mt7601u_dev *dev, int argc, char *const argv[])
{
	if (!strcmp(argv[0], "status")) {
		static const char * const names[] = {
			"idle", "authenticated", "associated", "ready"
		};
		if (conn.state == MT_CONN_IDLE) {
			printf("mt7601u: not connected\n");
		} else {
			printf("mt7601u: %s  ssid '%s'  bssid %pM  chan %u  aid %u\n",
			       names[conn.state], conn.ssid, conn.bssid,
			       conn.chan, conn.aid);
			printf("mt7601u: data tx %u  rx %u  rx-drop %u\n",
			       g_tx_frames, g_rx_frames, g_rx_drop);
		}
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[0], "join")) {
		if (argc < 2)
			return CMD_RET_USAGE;
		if (argc >= 4) {
			u8 bssid[6];
			unsigned long chan;

			string_to_enetaddr(argv[2], bssid);
			chan = simple_strtoul(argv[3], NULL, 10);
			if (is_zero_ethaddr(bssid) || chan < 1 || chan > 14) {
				printf("mt7601u: join needs 'wifi join <ssid> <bssid> <chan 1-14>'\n");
				return CMD_RET_USAGE;
			}
			return conn_join(dev, argv[1], bssid, (u8)chan) ?
				CMD_RET_FAILURE : CMD_RET_SUCCESS;
		}
		return conn_join(dev, argv[1], NULL, 0) ?
			CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	return CMD_RET_USAGE;
}
