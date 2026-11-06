/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT7601U station-mode connect (EXPERIMENTAL): take the dongle from a scanned
 * BSS to an associated, WPA2-PSK-keyed link, so U-Boot's net stack can run
 * over it. Built in milestones: open-system authentication, then association,
 * then the WPA2 four-way handshake and hardware key install, then an
 * eth-uclass wrapper. Joins ONE AP (the owner's own network) as a station; it
 * is not an access point and does not inject into networks it is not joining.
 */

#ifndef __MT7601U_CONNECT_H
#define __MT7601U_CONNECT_H

#include <linux/types.h>

struct mt7601u_dev;

enum mt_conn_state {
	MT_CONN_IDLE,
	MT_CONN_AUTHED,		/* open-system auth accepted */
	MT_CONN_ASSOCED,	/* association accepted, AID assigned */
	MT_CONN_READY,		/* 4-way handshake done, keys installed */
};

struct mt7601u_connect {
	u8 bssid[6];
	u8 chan;
	u8 ssid_len;
	char ssid[33];
	u8 sae;			/* BSS RSN is SAE and not PSK */
	u8 open_net;		/* beacon has neither PSK nor SAE */
	u8 rsn_len;
	u8 rsn[80];		/* association / EAPOL RSN element */
	enum mt_conn_state state;
	u16 aid;

	/* WPA2-PSK four-way handshake state */
	u8 pmk[32];
	u8 ptk[48];		/* KCK(16) || KEK(16) || TK(16) */
	u8 anonce[32];
	u8 snonce[32];
	u8 gtk[32];
	u8 gtk_len;
	u8 gtk_keyid;
	u8 replay[8];		/* last replay counter seen, big-endian */
};

/* 'wifi join <ssid>' / 'wifi status' (argv[0] is the subcommand) */
int mt7601u_connect_cmd(struct mt7601u_dev *dev, int argc, char *const argv[]);

/* release the data-path RX TD + connection state (called from 'wifi init') */
void mt7601u_connect_stop(struct mt7601u_dev *dev);

#endif
