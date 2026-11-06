/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * U-Boot scan-test port: the RXWI layout and field masks from the Linux
 * mt7601u driver (drivers/net/wireless/mediatek/mt7601u/mac.h). TXWI and the
 * mac80211 glue are dropped -- this port never transmits.
 */

#ifndef __MT76_MAC_H
#define __MT76_MAC_H

#include <linux/bitops.h>
#include <linux/types.h>

struct mt7601u_dev;

/* Note: values in original "RSSI" and "SNR" fields are not actually what they
 *	 are called for MT7601U, names used by this driver are educated guesses
 *	 (see vendor mac/ral_omac.c).
 */
struct mt7601u_rxwi {
	__le32 rxinfo;

	__le32 ctl;

	__le16 frag_sn;
	__le16 rate;

	u8 unknown;
	u8 zero[3];

	u8 snr;
	u8 ant;
	u8 gain;
	u8 freq_off;

	__le32 resv2;
	__le32 expert_ant;
} __packed __aligned(4);

#define MT_RXINFO_BA			BIT(0)
#define MT_RXINFO_DATA			BIT(1)
#define MT_RXINFO_NULL			BIT(2)
#define MT_RXINFO_FRAG			BIT(3)
#define MT_RXINFO_U2M			BIT(4)
#define MT_RXINFO_MULTICAST		BIT(5)
#define MT_RXINFO_BROADCAST		BIT(6)
#define MT_RXINFO_MYBSS			BIT(7)
#define MT_RXINFO_CRCERR		BIT(8)
#define MT_RXINFO_ICVERR		BIT(9)
#define MT_RXINFO_MICERR		BIT(10)
#define MT_RXINFO_AMSDU			BIT(11)
#define MT_RXINFO_HTC			BIT(12)
#define MT_RXINFO_RSSI			BIT(13)
#define MT_RXINFO_L2PAD			BIT(14)
#define MT_RXINFO_AMPDU			BIT(15)
#define MT_RXINFO_DECRYPT		BIT(16)

#define MT_RXWI_CTL_WCID		GENMASK(7, 0)
#define MT_RXWI_CTL_KEY_IDX		GENMASK(9, 8)
#define MT_RXWI_CTL_BSS_IDX		GENMASK(12, 10)
#define MT_RXWI_CTL_UDF			GENMASK(15, 13)
#define MT_RXWI_CTL_MPDU_LEN		GENMASK(27, 16)
#define MT_RXWI_CTL_TID			GENMASK(31, 28)

#define MT_RXWI_RATE_MCS		GENMASK(6, 0)
#define MT_RXWI_RATE_BW			BIT(7)
#define MT_RXWI_RATE_SGI		BIT(8)
#define MT_RXWI_RATE_STBC		GENMASK(10, 9)
#define MT_RXWI_RATE_PHY		GENMASK(15, 14)

#define MT_RXWI_GAIN_RSSI_VAL		GENMASK(5, 0)
#define MT_RXWI_GAIN_RSSI_LNA_ID	GENMASK(7, 6)
#define MT_RXWI_ANT_AUX_LNA		BIT(7)

enum mt76_phy_type {
	MT_PHY_TYPE_CCK,
	MT_PHY_TYPE_OFDM,
	MT_PHY_TYPE_HT,
	MT_PHY_TYPE_HT_GF,
};

void mt7601u_set_macaddr(struct mt7601u_dev *dev, const u8 *addr);

#endif
