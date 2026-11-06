/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * WPA2-PSK key-derivation primitives for the mt7601u station join, built on
 * U-Boot's own SHA-1 and AES-128. Pure functions, no device state:
 *   - PBKDF2-HMAC-SHA1 for the pairwise master key from a passphrase,
 *   - the IEEE 802.11 PRF for the pairwise transient key,
 *   - HMAC-SHA1 for the EAPOL-Key MIC (key descriptor version 2),
 *   - RFC 3394 AES key unwrap for the encrypted key data in message 3.
 * These compute the keys for the owner's own network; they do not attack or
 * recover anyone else's.
 */

#ifndef __MT7601U_WPA_CRYPTO_H
#define __MT7601U_WPA_CRYPTO_H

#include <linux/types.h>

/* HMAC-SHA1(key, data) -> 20-byte out */
void wpa_hmac_sha1(const u8 *key, int key_len, const u8 *data, int data_len,
		   u8 out[20]);

/* PMK = PBKDF2-HMAC-SHA1(passphrase, ssid, 4096, 32) */
void wpa_pmk(const char *passphrase, const u8 *ssid, int ssid_len,
	     u8 pmk[32]);

/*
 * PTK = PRF-(ptk_len*8)(PMK, "Pairwise key expansion",
 *                       min(aa,spa)||max(aa,spa)||min(an,sn)||max(an,sn))
 * aa = authenticator (AP) MAC, spa = our MAC, an/sn = ANonce/SNonce.
 */
void wpa_ptk(const u8 pmk[32], const u8 *aa, const u8 *spa,
	     const u8 *anonce, const u8 *snonce, u8 *ptk, int ptk_len);

/*
 * RFC 3394 AES key unwrap with a 128-bit KEK. `in` is (n+1) 8-byte blocks,
 * `out` receives n*8 bytes. Returns 0 on a valid integrity check, -1 otherwise.
 */
int wpa_aes_unwrap(const u8 kek[16], int n, const u8 *in, u8 *out);

#endif
