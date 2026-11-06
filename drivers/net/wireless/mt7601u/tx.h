/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * TXWI and Tx DMA descriptor fields from the Linux mt7601u driver
 * (drivers/net/wireless/mediatek/mt7601u/{mac.h,dma.h}). Used by connect.c to
 * build the one frame it transmits at a time: a management frame (probe/auth/
 * assoc) or an EAPOL data frame, wrapped in a TXWI and a Tx DMA header and
 * sent with one synchronous bulk-OUT. There is no Tx ring and no mac80211.
 */

#ifndef __MT7601U_TX_H
#define __MT7601U_TX_H

#include <linux/bitops.h>
#include <linux/types.h>

/* Tx DMA packet info (prepended 4-byte word; shares MT_TXD_INFO_* in dma.h) */
#define MT_TXD_PKT_INFO_NEXT_VLD	BIT(16)
#define MT_TXD_PKT_INFO_TX_BURST	BIT(17)
#define MT_TXD_PKT_INFO_80211		BIT(19)
#define MT_TXD_PKT_INFO_TSO		BIT(20)
#define MT_TXD_PKT_INFO_CSO		BIT(21)
#define MT_TXD_PKT_INFO_WIV		BIT(24)
#define MT_TXD_PKT_INFO_QSEL		GENMASK(26, 25)

enum mt76_qsel {
	MT_QSEL_MGMT,
	MT_QSEL_HCCA,
	MT_QSEL_EDCA,
	MT_QSEL_EDCA_2,
};

/* The transmit wireless information header (precedes the 802.11 frame) */
struct mt76_txwi {
	__le16 flags;
	__le16 rate_ctl;

	u8 ack_ctl;
	u8 wcid;
	__le16 len_ctl;

	__le32 iv;

	__le32 eiv;

	u8 aid;
	u8 txstream;
	__le16 ctl;
} __packed __aligned(4);

#define MT_TXWI_FLAGS_FRAG		BIT(0)
#define MT_TXWI_FLAGS_MMPS		BIT(1)
#define MT_TXWI_FLAGS_CFACK		BIT(2)
#define MT_TXWI_FLAGS_TS		BIT(3)
#define MT_TXWI_FLAGS_AMPDU		BIT(4)
#define MT_TXWI_FLAGS_MPDU_DENSITY	GENMASK(7, 5)
#define MT_TXWI_FLAGS_TXOP		GENMASK(9, 8)
#define MT_TXWI_FLAGS_CWMIN		GENMASK(12, 10)
#define MT_TXWI_FLAGS_NO_RATE_FALLBACK	BIT(13)
#define MT_TXWI_FLAGS_TX_RPT		BIT(14)
#define MT_TXWI_FLAGS_TX_RATE_LUT	BIT(15)

#define MT_TXWI_RATE_MCS		GENMASK(6, 0)
#define MT_TXWI_RATE_BW			BIT(7)
#define MT_TXWI_RATE_SGI		BIT(8)
#define MT_TXWI_RATE_STBC		GENMASK(10, 9)
#define MT_TXWI_RATE_PHY_MODE		GENMASK(15, 14)

#define MT_TXWI_ACK_CTL_REQ		BIT(0)
#define MT_TXWI_ACK_CTL_NSEQ		BIT(1)
#define MT_TXWI_ACK_CTL_BA_WINDOW	GENMASK(7, 2)

#define MT_TXWI_LEN_BYTE_CNT		GENMASK(11, 0)
#define MT_TXWI_LEN_PKTID		GENMASK(15, 12)

/* a CCK 1 Mbps rate_ctl (MCS 0, PHY type CCK): robust, for mgmt/EAPOL */
#define MT_RATE_CCK_1M			0

#endif
