/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * WPA3-Personal SAE (group 19) used when a BSS advertises SAE and not PSK.
 */
#ifndef __MT7601U_SAE_H
#define __MT7601U_SAE_H

#ifndef SAE_SELFTEST
#include <linux/types.h>
#endif

int sae_derive_pwe(const u8 *mac_a, const u8 *mac_b,
		   const u8 *password, int pwd_len,
		   u8 pwe_x[32], u8 pwe_y[32]);

int sae_derive_pwe_h2e(const u8 *mac_a, const u8 *mac_b,
		       const u8 *ssid, int ssid_len,
		       const u8 *password, int pwd_len,
		       u8 pwe_x[32], u8 pwe_y[32]);

int sae_random_scalar(const u8 entropy[32], u8 scalar[32]);

int sae_build_commit(const u8 pwe_x[32], const u8 pwe_y[32],
		     const u8 rand_s[32], const u8 mask[32],
		     u8 scalar[32], u8 element[64]);

int sae_process_commit(const u8 pwe_x[32], const u8 pwe_y[32],
		       const u8 rand_s[32],
		       const u8 own_scalar[32], const u8 own_element[64],
		       const u8 peer_scalar[32], const u8 peer_element[64],
		       u8 kck[32], u8 pmk[32], u8 pmkid[16]);

void sae_confirm(const u8 kck[32], u16 send_confirm,
		 const u8 *own_scalar, const u8 *own_element,
		 const u8 *peer_scalar, const u8 *peer_element,
		 u8 confirm[32]);

void sae_aes_cmac(const u8 key[16], const u8 *msg, int len, u8 mic[16]);

void sae_ptk_sha256(const u8 pmk[32], const u8 *aa, const u8 *spa,
		    const u8 *anonce, const u8 *snonce, u8 *ptk, int ptk_len);

#endif
