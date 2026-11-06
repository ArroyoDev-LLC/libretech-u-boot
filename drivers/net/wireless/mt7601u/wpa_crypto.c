// SPDX-License-Identifier: GPL-2.0-only
/*
 * WPA2-PSK key-derivation primitives for the mt7601u station join.
 * Implemented on U-Boot's SHA-1 (lib/sha1.c) and AES-128 (lib/aes.c).
 */

#include <linux/types.h>
#include <linux/string.h>
#include <u-boot/sha1.h>
#include <uboot_aes.h>
#include <asm/unaligned.h>

#include "wpa_crypto.h"

#define SHA1_BLOCK	64
#define SHA1_DIGEST	20

void wpa_hmac_sha1(const u8 *key, int key_len, const u8 *data, int data_len,
		   u8 out[20])
{
	sha1_context ctx;
	u8 k[SHA1_BLOCK], ipad[SHA1_BLOCK], opad[SHA1_BLOCK];
	u8 tmp[SHA1_DIGEST];
	int i;

	memset(k, 0, sizeof(k));
	if (key_len > SHA1_BLOCK) {
		sha1_starts(&ctx);
		sha1_update(&ctx, key, key_len);
		sha1_finish(&ctx, k);
	} else {
		memcpy(k, key, key_len);
	}

	for (i = 0; i < SHA1_BLOCK; i++) {
		ipad[i] = k[i] ^ 0x36;
		opad[i] = k[i] ^ 0x5c;
	}

	sha1_starts(&ctx);
	sha1_update(&ctx, ipad, SHA1_BLOCK);
	sha1_update(&ctx, data, data_len);
	sha1_finish(&ctx, tmp);

	sha1_starts(&ctx);
	sha1_update(&ctx, opad, SHA1_BLOCK);
	sha1_update(&ctx, tmp, SHA1_DIGEST);
	sha1_finish(&ctx, out);
}

/* One PBKDF2 block F(P, S, c, i) accumulated into out[20] */
static void pbkdf2_f(const char *pass, int plen, const u8 *salt, int slen,
		     int count, int idx, u8 out[20])
{
	u8 buf[64], u[20];
	int i, j;

	memcpy(buf, salt, slen);
	buf[slen] = (idx >> 24) & 0xff;
	buf[slen + 1] = (idx >> 16) & 0xff;
	buf[slen + 2] = (idx >> 8) & 0xff;
	buf[slen + 3] = idx & 0xff;

	wpa_hmac_sha1((const u8 *)pass, plen, buf, slen + 4, u);
	memcpy(out, u, 20);

	for (i = 1; i < count; i++) {
		wpa_hmac_sha1((const u8 *)pass, plen, u, 20, u);
		for (j = 0; j < 20; j++)
			out[j] ^= u[j];
	}
}

void wpa_pmk(const char *passphrase, const u8 *ssid, int ssid_len, u8 pmk[32])
{
	u8 blk[20];
	int plen = strlen(passphrase);

	pbkdf2_f(passphrase, plen, ssid, ssid_len, 4096, 1, blk);
	memcpy(pmk, blk, 20);
	pbkdf2_f(passphrase, plen, ssid, ssid_len, 4096, 2, blk);
	memcpy(pmk + 20, blk, 12);
}

void wpa_ptk(const u8 pmk[32], const u8 *aa, const u8 *spa,
	     const u8 *anonce, const u8 *snonce, u8 *ptk, int ptk_len)
{
	static const char label[] = "Pairwise key expansion";
	u8 data[2 + sizeof(label) + 76];	/* label(+NUL) + 0 + B + counter */
	u8 b[76];
	u8 digest[20];
	int pos = 0, i, off = 0;

	/* B = min(aa,spa)||max(aa,spa)||min(an,sn)||max(an,sn) */
	if (memcmp(aa, spa, 6) < 0) {
		memcpy(b + 0, aa, 6);
		memcpy(b + 6, spa, 6);
	} else {
		memcpy(b + 0, spa, 6);
		memcpy(b + 6, aa, 6);
	}
	if (memcmp(anonce, snonce, 32) < 0) {
		memcpy(b + 12, anonce, 32);
		memcpy(b + 44, snonce, 32);
	} else {
		memcpy(b + 12, snonce, 32);
		memcpy(b + 44, anonce, 32);
	}

	/* A(label, no NUL) || 0x00 || B || counter */
	memcpy(data, label, sizeof(label) - 1);
	pos = sizeof(label) - 1;
	data[pos++] = 0x00;
	memcpy(data + pos, b, 76);
	pos += 76;

	for (i = 0; off < ptk_len; i++) {
		data[pos] = i;
		wpa_hmac_sha1(pmk, 32, data, pos + 1, digest);
		if (ptk_len - off >= 20) {
			memcpy(ptk + off, digest, 20);
			off += 20;
		} else {
			memcpy(ptk + off, digest, ptk_len - off);
			off = ptk_len;
		}
	}
}

int wpa_aes_unwrap(const u8 kek[16], int n, const u8 *in, u8 *out)
{
	u8 expkey[AES128_EXPAND_KEY_LENGTH];
	u8 a[8], b[16];
	u8 r[16 * 8];	/* up to 16 blocks of key data */
	int i, j, k;
	u64 t;

	if (n < 1 || n > 16)
		return -1;

	aes_expand_key((u8 *)kek, 128, expkey);

	memcpy(a, in, 8);
	memcpy(r, in + 8, n * 8);

	for (j = 5; j >= 0; j--) {
		for (i = n; i >= 1; i--) {
			t = (u64)n * j + i;
			memcpy(b, a, 8);
			memcpy(b + 8, r + (i - 1) * 8, 8);
			for (k = 0; k < 8; k++)
				b[7 - k] ^= (t >> (8 * k)) & 0xff;
			aes_decrypt(128, b, expkey, b);
			memcpy(a, b, 8);
			memcpy(r + (i - 1) * 8, b + 8, 8);
		}
	}

	for (i = 0; i < 8; i++)
		if (a[i] != 0xa6)
			return -1;

	memcpy(out, r, n * 8);
	return 0;
}
