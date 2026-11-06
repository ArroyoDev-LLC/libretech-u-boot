/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2014 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * U-Boot scan-test port: DMA descriptor field definitions from the Linux
 * mt7601u driver (drivers/net/wireless/mediatek/mt7601u/dma.h); the skb
 * wrapping helpers are replaced by flat-buffer code in mcu.c.
 */

#ifndef __MT7601U_DMA_H
#define __MT7601U_DMA_H

#include <linux/bitops.h>

#define MT_DMA_HDR_LEN			4
#define MT_RX_INFO_LEN			4
#define MT_FCE_INFO_LEN			4
#define MT_DMA_HDRS			(MT_DMA_HDR_LEN + MT_RX_INFO_LEN)

/* Common Tx DMA descriptor fields */
#define MT_TXD_INFO_LEN			GENMASK(15, 0)
#define MT_TXD_INFO_D_PORT		GENMASK(29, 27)
#define MT_TXD_INFO_TYPE		GENMASK(31, 30)

enum mt76_msg_port {
	WLAN_PORT,
	CPU_RX_PORT,
	CPU_TX_PORT,
	HOST_PORT,
	VIRTUAL_CPU_RX_PORT,
	VIRTUAL_CPU_TX_PORT,
	DISCARD,
};

enum mt76_info_type {
	DMA_PACKET,
	DMA_COMMAND,
};

/* Tx DMA MCU command specific flags */
#define MT_TXD_CMD_INFO_SEQ		GENMASK(19, 16)
#define MT_TXD_CMD_INFO_TYPE		GENMASK(26, 20)

/* Common Rx DMA descriptor fields */
#define MT_RXD_INFO_LEN			GENMASK(13, 0)
#define MT_RXD_INFO_PCIE_INTR		BIT(24)
#define MT_RXD_INFO_QSEL		GENMASK(26, 25)
#define MT_RXD_INFO_PORT		GENMASK(29, 27)
#define MT_RXD_INFO_TYPE		GENMASK(31, 30)

/* Rx DMA packet specific flags */
#define MT_RXD_PKT_INFO_PKT_80211	BIT(19)

/* Rx DMA MCU command specific flags */
#define MT_RXD_CMD_INFO_SELF_GEN	BIT(15)
#define MT_RXD_CMD_INFO_CMD_SEQ		GENMASK(19, 16)
#define MT_RXD_CMD_INFO_EVT_TYPE	GENMASK(23, 20)

enum mt76_evt_type {
	CMD_DONE,
	CMD_ERROR,
	CMD_RETRY,
	EVENT_PWR_RSP,
	EVENT_WOW_RSP,
	EVENT_CARRIER_DETECT_RSP,
	EVENT_DFS_DETECT_RSP,
};

#endif
