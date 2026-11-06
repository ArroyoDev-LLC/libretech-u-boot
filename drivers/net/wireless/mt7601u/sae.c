/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * WPA3-Personal (SAE, group 19) for the owner's own AP.
 * Hunting-and-pecking PWE, commit/confirm, and the AKM-8 MIC.
 * A PSK-only BSS does not call this.
 */

#ifndef SAE_SELFTEST
#include <linux/string.h>
#include <linux/types.h>
#include <u-boot/sha256.h>
#include <uboot_aes.h>
#else
#include <stdint.h>
#include <string.h>
#include <stdio.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int64_t s64;
typedef struct {
	uint32_t total[2];
	uint32_t state[8];
	uint8_t buffer[64];
} sha256_context;
void sha256_starts(sha256_context *ctx);
void sha256_update(sha256_context *ctx, const uint8_t *input, uint32_t length);
void sha256_finish(sha256_context *ctx, uint8_t digest[32]);
int sha256_hmac(const unsigned char *key, int keylen,
		const unsigned char *input, unsigned int ilen,
		unsigned char *output);
void aes_expand_key(u8 *key, u32 key_size, u8 *expkey);
void aes_encrypt(u32 key_size, u8 *in, u8 *expkey, u8 *out);
#define AES128_EXPAND_KEY_LENGTH 176
#endif

#include "sae.h"

struct fe {
	u32 w[8];
};

struct pt {
	struct fe x, y;
	int inf;
};

/* P-256 prime, little-endian limbs. */
static const struct fe FE_P = {{
	0xffffffff, 0xffffffff, 0xffffffff, 0x00000000,
	0x00000000, 0x00000000, 0x00000001, 0xffffffff
}};
/* 2^256 mod p. */
static const struct fe FE_R = {{
	0x00000001, 0x00000000, 0x00000000, 0xffffffff,
	0xffffffff, 0xffffffff, 0xfffffffe, 0x00000000
}};
/* Group order n. */
static const struct fe FE_N = {{
	0xfc632551, 0xf3b9cac2, 0xa7179e84, 0xbce6faad,
	0xffffffff, 0xffffffff, 0x00000000, 0xffffffff
}};
/* 2^256 - n, added when a scalar sum overflows 256 bits. */
static const struct fe FE_NRES = {{
	0x039cdaaf, 0x0c46353d, 0x58e8617b, 0x43190552,
	0x00000000, 0x00000000, 0xffffffff, 0x00000000
}};
static const struct fe FE_B = {{
	0x27d2604b, 0x3bce3c3e, 0xcc53b0f6, 0x651d06b0,
	0x769886bc, 0xb3ebbd55, 0xaa3a93e7, 0x5ac635d8
}};
/* (p - 1) / 2 and (p + 1) / 4, little-endian limbs. */
static const struct fe FE_LEG = {{
	0xffffffff, 0xffffffff, 0x7fffffff, 0x00000000,
	0x00000000, 0x80000000, 0x80000000, 0x7fffffff
}};
static const struct fe FE_SQRT = {{
	0x00000000, 0x00000000, 0x40000000, 0x00000000,
	0x00000000, 0x40000000, 0xc0000000, 0x3fffffff
}};

static int fe_cmp(const struct fe *a, const struct fe *b)
{
	int i;

	for (i = 7; i >= 0; i--) {
		if (a->w[i] > b->w[i])
			return 1;
		if (a->w[i] < b->w[i])
			return -1;
	}
	return 0;
}

static int fe_is_zero(const struct fe *a)
{
	int i;

	for (i = 0; i < 8; i++)
		if (a->w[i])
			return 0;
	return 1;
}

static void fe_set_u32(struct fe *r, u32 v)
{
	memset(r, 0, sizeof(*r));
	r->w[0] = v;
}

static void fe_from_be(struct fe *r, const u8 in[32])
{
	int i;

	for (i = 0; i < 8; i++) {
		const u8 *p = in + (7 - i) * 4;

		r->w[i] = ((u32)p[0] << 24) | ((u32)p[1] << 16) |
			  ((u32)p[2] << 8) | p[3];
	}
}

static void fe_to_be(u8 out[32], const struct fe *a)
{
	int i;

	for (i = 0; i < 8; i++) {
		u8 *p = out + (7 - i) * 4;

		p[0] = a->w[i] >> 24;
		p[1] = a->w[i] >> 16;
		p[2] = a->w[i] >> 8;
		p[3] = a->w[i];
	}
}

static void fe_sub_raw(struct fe *r, const struct fe *a, const struct fe *b)
{
	s64 br = 0;
	int i;

	for (i = 0; i < 8; i++) {
		s64 v = (s64)a->w[i] - b->w[i] - br;

		if (v < 0) {
			v += (s64)1 << 32;
			br = 1;
		} else {
			br = 0;
		}
		r->w[i] = (u32)v;
	}
}

static void fe_sub(struct fe *r, const struct fe *a, const struct fe *b)
{
	if (fe_cmp(a, b) >= 0) {
		fe_sub_raw(r, a, b);
	} else {
		struct fe t;

		fe_sub_raw(&t, b, a);
		fe_sub_raw(r, &FE_P, &t);
	}
}

static void fe_mul_wide(u32 w[16], const struct fe *a, const struct fe *b)
{
	unsigned __int128 acc[16];
	unsigned __int128 c = 0;
	int i, j;

	memset(acc, 0, sizeof(acc));
	for (i = 0; i < 8; i++)
		for (j = 0; j < 8; j++)
			acc[i + j] += (unsigned __int128)a->w[i] * b->w[j];
	for (i = 0; i < 16; i++) {
		c += acc[i];
		w[i] = (u32)c;
		c >>= 32;
	}
}

static void fe_red(struct fe *r, const u32 in[16])
{
	u32 w[16];
	int pass, i;

	memcpy(w, in, sizeof(w));
	for (pass = 0; pass < 12; pass++) {
		struct fe hi;
		u32 prod[16];
		u64 c = 0;
		int nz = 0;

		for (i = 8; i < 16; i++)
			if (w[i])
				nz = 1;
		if (!nz)
			break;
		for (i = 0; i < 8; i++)
			hi.w[i] = w[i + 8];
		fe_mul_wide(prod, &hi, &FE_R);
		for (i = 0; i < 8; i++) {
			c += (u64)w[i] + prod[i];
			w[i] = (u32)c;
			c >>= 32;
		}
		for (i = 8; i < 16; i++) {
			c += prod[i];
			w[i] = (u32)c;
			c >>= 32;
		}
		while (c) {
			u64 d = 0;

			for (i = 0; i < 8; i++) {
				d += (u64)w[i] + c * FE_R.w[i];
				w[i] = (u32)d;
				d >>= 32;
			}
			for (i = 8; i < 16; i++) {
				d += w[i];
				w[i] = (u32)d;
				d >>= 32;
			}
			c = d;
		}
	}
	memcpy(r->w, w, 32);
	while (fe_cmp(r, &FE_P) >= 0)
		fe_sub_raw(r, r, &FE_P);
}

static void fe_mul(struct fe *r, const struct fe *a, const struct fe *b)
{
	u32 w[16];

	fe_mul_wide(w, a, b);
	fe_red(r, w);
}

static void fe_add(struct fe *r, const struct fe *a, const struct fe *b)
{
	struct fe t;
	u64 c = 0;
	int i;

	for (i = 0; i < 8; i++) {
		c += (u64)a->w[i] + b->w[i];
		t.w[i] = (u32)c;
		c >>= 32;
	}
	while (c) {
		u64 d = 0;

		for (i = 0; i < 8; i++) {
			d += (u64)t.w[i] + c * (u64)FE_R.w[i];
			t.w[i] = (u32)d;
			d >>= 32;
		}
		c = d;
	}
	while (fe_cmp(&t, &FE_P) >= 0)
		fe_sub_raw(&t, &t, &FE_P);
	*r = t;
}

static void fe_add_n(struct fe *r, const struct fe *a, const struct fe *b)
{
	struct fe t;
	u64 c = 0;
	int i;

	for (i = 0; i < 8; i++) {
		c += (u64)a->w[i] + b->w[i];
		t.w[i] = (u32)c;
		c >>= 32;
	}
	if (c) {
		u64 d = 0;

		for (i = 0; i < 8; i++) {
			d += (u64)t.w[i] + FE_NRES.w[i];
			t.w[i] = (u32)d;
			d >>= 32;
		}
	}
	while (fe_cmp(&t, &FE_N) >= 0)
		fe_sub_raw(&t, &t, &FE_N);
	*r = t;
}

static void fe_pow(struct fe *r, const struct fe *base, const struct fe *exp)
{
	struct fe acc;
	int i;

	fe_set_u32(&acc, 1);
	for (i = 255; i >= 0; i--) {
		fe_mul(&acc, &acc, &acc);
		if ((exp->w[i / 32] >> (i % 32)) & 1)
			fe_mul(&acc, &acc, base);
	}
	*r = acc;
}

static void fe_inv(struct fe *r, const struct fe *a)
{
	struct fe e;

	/* p - 2 */
	e = FE_P;
	e.w[0] -= 2;
	fe_pow(r, a, &e);
}

static void point_set_inf(struct pt *p)
{
	memset(p, 0, sizeof(*p));
	p->inf = 1;
}

static void point_double(struct pt *r, const struct pt *p)
{
	struct fe lam, x2, t, x3, y3, two;

	if (p->inf || fe_is_zero(&p->y)) {
		point_set_inf(r);
		return;
	}
	fe_mul(&x2, &p->x, &p->x);
	fe_add(&t, &x2, &x2);
	fe_add(&t, &t, &x2);		/* 3 x^2 */
	fe_set_u32(&two, 3);
	fe_sub(&t, &t, &two);		/* 3 x^2 - 3 */
	fe_add(&two, &p->y, &p->y);
	fe_inv(&lam, &two);
	fe_mul(&lam, &t, &lam);
	fe_mul(&x3, &lam, &lam);
	fe_sub(&x3, &x3, &p->x);
	fe_sub(&x3, &x3, &p->x);
	fe_sub(&y3, &p->x, &x3);
	fe_mul(&y3, &lam, &y3);
	fe_sub(&y3, &y3, &p->y);
	r->x = x3;
	r->y = y3;
	r->inf = 0;
}

static void point_add(struct pt *r, const struct pt *a, const struct pt *b)
{
	struct pt A, B;
	struct fe lam, t, x3, y3;

	A = *a;
	B = *b;
	if (A.inf) {
		*r = B;
		return;
	}
	if (B.inf) {
		*r = A;
		return;
	}
	if (fe_cmp(&A.x, &B.x) == 0) {
		fe_add(&t, &A.y, &B.y);
		if (fe_is_zero(&t))
			point_set_inf(r);
		else
			point_double(r, &A);
		return;
	}
	fe_sub(&lam, &B.y, &A.y);
	fe_sub(&t, &B.x, &A.x);
	fe_inv(&y3, &t);
	fe_mul(&lam, &lam, &y3);
	fe_mul(&x3, &lam, &lam);
	fe_sub(&x3, &x3, &A.x);
	fe_sub(&x3, &x3, &B.x);
	fe_sub(&y3, &A.x, &x3);
	fe_mul(&y3, &lam, &y3);
	fe_sub(&y3, &y3, &A.y);
	r->x = x3;
	r->y = y3;
	r->inf = 0;
}

static void point_mul(struct pt *r, const struct pt *p, const struct fe *k)
{
	struct pt acc, base;
	int i;

	point_set_inf(&acc);
	base = *p;
	for (i = 255; i >= 0; i--) {
		point_double(&acc, &acc);
		if ((k->w[i / 32] >> (i % 32)) & 1)
			point_add(&acc, &acc, &base);
	}
	*r = acc;
}

static int point_on(const struct pt *p)
{
	struct fe y2, x3, ax, rhs;

	if (p->inf)
		return 0;
	if (fe_cmp(&p->x, &FE_P) >= 0 || fe_cmp(&p->y, &FE_P) >= 0)
		return 0;
	fe_mul(&y2, &p->y, &p->y);
	fe_mul(&x3, &p->x, &p->x);
	fe_mul(&x3, &x3, &p->x);
	fe_add(&ax, &p->x, &p->x);
	fe_add(&ax, &ax, &p->x);
	fe_sub(&rhs, &FE_B, &ax);
	fe_add(&rhs, &rhs, &x3);
	return fe_cmp(&y2, &rhs) == 0;
}

static void point_neg(struct pt *r, const struct pt *p)
{
	*r = *p;
	if (!p->inf && !fe_is_zero(&p->y))
		fe_sub_raw(&r->y, &FE_P, &p->y);
}

static void kdf_sha256(const u8 *key, int key_len, const char *label,
		       const u8 *context, int context_len,
		       u8 *out, int out_bits)
{
	u8 block[2 + 64 + 80 + 2];
	u8 mac[32];
	int lab_len = 0;
	int out_len = (out_bits + 7) / 8;
	int off = 0;
	u16 counter = 1;

	while (label[lab_len])
		lab_len++;
	while (off < out_len) {
		int n, pos = 0;

		/* IEEE 802.11-2012 11.6.1.7.2: counter and length are LE16. */
		block[pos++] = counter & 0xff;
		block[pos++] = counter >> 8;
		memcpy(block + pos, label, lab_len);
		pos += lab_len;
		memcpy(block + pos, context, context_len);
		pos += context_len;
		block[pos++] = out_bits & 0xff;
		block[pos++] = out_bits >> 8;
		sha256_hmac(key, key_len, block, pos, mac);
		n = out_len - off;
		if (n > 32)
			n = 32;
		memcpy(out + off, mac, n);
		off += n;
		counter++;
	}
}

int sae_derive_pwe(const u8 *mac_a, const u8 *mac_b,
		   const u8 *password, int pwd_len,
		   u8 pwe_x[32], u8 pwe_y[32])
{
	u8 addrs[12], prime[32], pwd_seed[32], pwd_value[32];
	u8 msg[64 + 1];
	struct fe x, y2, leg, y, one;
	int counter;
	int found = 0;

	if (pwd_len < 1 || pwd_len > 64)
		return -1;
	if (memcmp(mac_a, mac_b, 6) >= 0) {
		memcpy(addrs, mac_a, 6);
		memcpy(addrs + 6, mac_b, 6);
	} else {
		memcpy(addrs, mac_b, 6);
		memcpy(addrs + 6, mac_a, 6);
	}
	fe_to_be(prime, &FE_P);
	memcpy(msg, password, pwd_len);
	fe_set_u32(&one, 1);
	for (counter = 1; counter <= 40; counter++) {
		struct fe saved;

		msg[pwd_len] = counter;
		sha256_hmac(addrs, 12, msg, pwd_len + 1, pwd_seed);
		kdf_sha256(pwd_seed, 32, "SAE Hunting and Pecking",
			   prime, 32, pwd_value, 256);
		fe_from_be(&x, pwd_value);
		if (fe_cmp(&x, &FE_P) >= 0)
			continue;
		fe_mul(&y2, &x, &x);
		fe_mul(&y2, &y2, &x);
		{
			struct fe ax;

			fe_add(&ax, &x, &x);
			fe_add(&ax, &ax, &x);
			fe_sub(&y2, &y2, &ax);
			fe_add(&y2, &y2, &FE_B);
		}
		if (!fe_is_zero(&y2)) {
			fe_pow(&leg, &y2, &FE_LEG);
			if (fe_cmp(&leg, &one) != 0)
				continue;
		}
		fe_pow(&y, &y2, &FE_SQRT);
		if ((pwd_seed[31] & 1) != (y.w[0] & 1))
			fe_sub_raw(&y, &FE_P, &y);
		if (!found) {
			saved = x;
			fe_to_be(pwe_x, &saved);
			fe_to_be(pwe_y, &y);
			found = 1;
		}
	}
	return found ? 0 : -1;
}

/* RFC 5869 HKDF-Expand, info is the raw string with no trailing NUL. */
static void hkdf_expand(const u8 prk[32], const char *info, u8 *out, int out_len)
{
	u8 t[32];
	u8 block[32 + 80];
	int info_len = 0;
	int tlen = 0;
	int off = 0;
	u8 counter = 1;

	while (info[info_len])
		info_len++;
	while (off < out_len) {
		int n, pos = 0;

		if (tlen) {
			memcpy(block, t, tlen);
			pos = tlen;
		}
		memcpy(block + pos, info, info_len);
		pos += info_len;
		block[pos++] = counter;
		sha256_hmac(prk, 32, block, pos, t);
		tlen = 32;
		n = out_len - off;
		if (n > 32)
			n = 32;
		memcpy(out + off, t, n);
		off += n;
		counter++;
	}
}

/* 48-byte big-endian integer modulo p. 2^256 ≡ FE_R (mod p). */
static void fe_mod_48(struct fe *r, const u8 in[48])
{
	struct fe hi, lo, t;
	u8 hib[32];

	memset(hib, 0, 16);
	memcpy(hib + 16, in, 16);
	fe_from_be(&hi, hib);
	fe_from_be(&lo, in + 16);
	fe_mul(&t, &hi, &FE_R);
	fe_add(r, &t, &lo);
}

/*
 * Simplified SWU for P-256, z = -10. Matches the map used by SAE hash-to-element.
 * u is already reduced modulo p.
 */
static void sswu(struct pt *out, const struct fe *u)
{
	struct fe u2, t1, t2, m, t, x1a, x1b, x1, x2, gx1, gx2, y, z, curve_a;
	struct fe three, ten, one, leg, negb, x, v;
	int m_zero, is_qr;

	fe_set_u32(&three, 3);
	fe_sub(&curve_a, &FE_P, &three);
	fe_set_u32(&ten, 10);
	fe_sub(&z, &FE_P, &ten);
	fe_set_u32(&one, 1);

	fe_mul(&u2, u, u);
	fe_mul(&t1, &z, &u2);
	fe_mul(&t2, &t1, &t1);
	fe_add(&m, &t1, &t2);
	fe_inv(&t, &m);

	fe_mul(&t1, &z, &curve_a);
	fe_inv(&t1, &t1);
	fe_mul(&x1a, &FE_B, &t1);

	fe_sub(&negb, &FE_P, &FE_B);
	fe_inv(&t2, &curve_a);
	fe_mul(&negb, &negb, &t2);
	fe_add(&t2, &one, &t);
	fe_mul(&x1b, &negb, &t2);

	m_zero = fe_is_zero(&m);
	x1 = m_zero ? x1a : x1b;

	fe_mul(&t1, &x1, &x1);
	fe_mul(&t1, &t1, &x1);
	fe_mul(&t2, &curve_a, &x1);
	fe_add(&t1, &t1, &t2);
	fe_add(&gx1, &t1, &FE_B);

	fe_mul(&t1, &z, &u2);
	fe_mul(&x2, &t1, &x1);
	fe_mul(&t1, &x2, &x2);
	fe_mul(&t1, &t1, &x2);
	fe_mul(&t2, &curve_a, &x2);
	fe_add(&t1, &t1, &t2);
	fe_add(&gx2, &t1, &FE_B);

	fe_pow(&leg, &gx1, &FE_LEG);
	is_qr = fe_is_zero(&leg) || fe_cmp(&leg, &one) == 0;
	x = is_qr ? x1 : x2;
	v = is_qr ? gx1 : gx2;
	fe_pow(&y, &v, &FE_SQRT);
	if ((u->w[0] & 1) != (y.w[0] & 1))
		fe_sub(&y, &FE_P, &y);
	out->x = x;
	out->y = y;
	out->inf = 0;
}

int sae_derive_pwe_h2e(const u8 *mac_a, const u8 *mac_b,
		       const u8 *ssid, int ssid_len,
		       const u8 *password, int pwd_len,
		       u8 pwe_x[32], u8 pwe_y[32])
{
	u8 addrs[12], pwd_seed[32], pwd_value[48], valb[32];
	struct fe u, val, one, nm1;
	struct pt p1, p2, pt, pwe;
	static const char *label[2] = {
		"SAE Hash to Element u1 P1",
		"SAE Hash to Element u2 P2",
	};
	int i;

	if (pwd_len < 1 || pwd_len > 64 || ssid_len < 1 || ssid_len > 32)
		return -1;
	sha256_hmac(ssid, ssid_len, password, pwd_len, pwd_seed);
	for (i = 0; i < 2; i++) {
		hkdf_expand(pwd_seed, label[i], pwd_value, 48);
		fe_mod_48(&u, pwd_value);
		if (i == 0)
			sswu(&p1, &u);
		else
			sswu(&p2, &u);
	}
	point_add(&pt, &p1, &p2);
	if (pt.inf || !point_on(&pt))
		return -1;

	if (memcmp(mac_a, mac_b, 6) >= 0) {
		memcpy(addrs, mac_a, 6);
		memcpy(addrs + 6, mac_b, 6);
	} else {
		memcpy(addrs, mac_b, 6);
		memcpy(addrs + 6, mac_a, 6);
	}
	memset(valb, 0, 32);
	sha256_hmac(valb, 32, addrs, 12, valb);
	fe_from_be(&val, valb);
	fe_set_u32(&one, 1);
	fe_sub(&nm1, &FE_N, &one);
	if (fe_cmp(&val, &nm1) < 0) {
		fe_add(&val, &val, &one);
	} else {
		fe_sub(&val, &val, &nm1);
		fe_add(&val, &val, &one);
	}
	point_mul(&pwe, &pt, &val);
	if (pwe.inf || !point_on(&pwe))
		return -1;
	fe_to_be(pwe_x, &pwe.x);
	fe_to_be(pwe_y, &pwe.y);
	return 0;
}

int sae_random_scalar(const u8 entropy[32], u8 scalar[32])
{
	u8 buf[32];
	struct fe one, s;
	int i;

	memcpy(buf, entropy, 32);
	fe_set_u32(&one, 1);
	for (i = 0; i < 16; i++) {
		sha256_context ctx;

		fe_from_be(&s, buf);
		if (fe_cmp(&s, &one) > 0 && fe_cmp(&s, &FE_N) < 0) {
			memcpy(scalar, buf, 32);
			return 0;
		}
		sha256_starts(&ctx);
		sha256_update(&ctx, buf, 32);
		sha256_finish(&ctx, buf);
	}
	return -1;
}

int sae_build_commit(const u8 pwe_x[32], const u8 pwe_y[32],
		     const u8 rand_s[32], const u8 mask[32],
		     u8 scalar[32], u8 element[64])
{
	struct fe rnd, msk, sc, one;
	struct pt pwe, e, neg;

	fe_from_be(&rnd, rand_s);
	fe_from_be(&msk, mask);
	fe_set_u32(&one, 1);
	fe_add_n(&sc, &rnd, &msk);
	if (fe_cmp(&sc, &one) <= 0 || fe_cmp(&sc, &FE_N) >= 0)
		return -1;
	fe_to_be(scalar, &sc);
	fe_from_be(&pwe.x, pwe_x);
	fe_from_be(&pwe.y, pwe_y);
	pwe.inf = 0;
	point_mul(&e, &pwe, &msk);
	if (e.inf)
		return -1;
	point_neg(&neg, &e);
	fe_to_be(element, &neg.x);
	fe_to_be(element + 32, &neg.y);
	return 0;
}

int sae_process_commit(const u8 pwe_x[32], const u8 pwe_y[32],
		       const u8 rand_s[32],
		       const u8 own_scalar[32], const u8 own_element[64],
		       const u8 peer_scalar[32], const u8 peer_element[64],
		       u8 kck[32], u8 pmk[32], u8 pmkid[16])
{
	struct fe one, ps, os, sum, rnd;
	struct pt pwe, pel, t, kpt;
	u8 k[32], keyseed[32], ctx[32], material[64];
	u8 salt[32];

	fe_set_u32(&one, 1);
	fe_from_be(&ps, peer_scalar);
	fe_from_be(&os, own_scalar);
	if (fe_cmp(&ps, &one) <= 0 || fe_cmp(&ps, &FE_N) >= 0)
		return -1;
	if (!memcmp(own_scalar, peer_scalar, 32) &&
	    !memcmp(own_element, peer_element, 64))
		return -1;
	fe_from_be(&pel.x, peer_element);
	fe_from_be(&pel.y, peer_element + 32);
	pel.inf = 0;
	if (!point_on(&pel))
		return -1;
	fe_from_be(&pwe.x, pwe_x);
	fe_from_be(&pwe.y, pwe_y);
	pwe.inf = 0;
	point_mul(&t, &pwe, &ps);
	point_add(&t, &t, &pel);
	if (t.inf)
		return -1;
	fe_from_be(&rnd, rand_s);
	point_mul(&kpt, &t, &rnd);
	if (kpt.inf)
		return -1;
	fe_to_be(k, &kpt.x);
	memset(salt, 0, 32);
	sha256_hmac(salt, 32, k, 32, keyseed);
	fe_add_n(&sum, &os, &ps);
	fe_to_be(ctx, &sum);
	kdf_sha256(keyseed, 32, "SAE KCK and PMK", ctx, 32, material, 512);
	memcpy(kck, material, 32);
	memcpy(pmk, material + 32, 32);
	memcpy(pmkid, ctx, 16);
	return 0;
}

void sae_confirm(const u8 kck[32], u16 send_confirm,
		 const u8 *own_scalar, const u8 *own_element,
		 const u8 *peer_scalar, const u8 *peer_element,
		 u8 confirm[32])
{
	u8 data[2 + 32 + 64 + 32 + 64];

	data[0] = send_confirm & 0xff;
	data[1] = send_confirm >> 8;
	memcpy(data + 2, own_scalar, 32);
	memcpy(data + 34, own_element, 64);
	memcpy(data + 98, peer_scalar, 32);
	memcpy(data + 130, peer_element, 64);
	sha256_hmac(kck, 32, data, sizeof(data), confirm);
}

static void cmac_dbl(u8 *out, const u8 *in)
{
	int i;
	u8 c = 0;

	for (i = 15; i >= 0; i--) {
		u8 n = (u8)((in[i] << 1) | c);

		c = in[i] >> 7;
		out[i] = n;
	}
	if (in[0] & 0x80)
		out[15] ^= 0x87;
}

void sae_aes_cmac(const u8 key[16], const u8 *msg, int len, u8 mic[16])
{
	u8 exp[AES128_EXPAND_KEY_LENGTH];
	u8 kcopy[16], L[16], k1[16], k2[16], x[16], block[16];
	int n, i, complete;

	memcpy(kcopy, key, 16);
	aes_expand_key(kcopy, 128, exp);
	memset(x, 0, 16);
	aes_encrypt(128, x, exp, L);
	cmac_dbl(k1, L);
	cmac_dbl(k2, k1);
	if (len == 0) {
		n = 1;
		complete = 0;
	} else {
		n = (len + 15) / 16;
		complete = (len % 16) == 0;
	}
	memset(x, 0, 16);
	for (i = 0; i < n; i++) {
		int last = (i == n - 1);
		int j;

		memset(block, 0, 16);
		if (!last) {
			memcpy(block, msg + i * 16, 16);
		} else if (complete) {
			memcpy(block, msg + i * 16, 16);
			for (j = 0; j < 16; j++)
				block[j] ^= k1[j];
		} else {
			int take = len - i * 16;

			if (take < 0)
				take = 0;
			if (take > 16)
				take = 16;
			if (take)
				memcpy(block, msg + i * 16, take);
			block[take] = 0x80;
			for (j = 0; j < 16; j++)
				block[j] ^= k2[j];
		}
		for (j = 0; j < 16; j++)
			x[j] ^= block[j];
		aes_encrypt(128, x, exp, x);
	}
	memcpy(mic, x, 16);
}

void sae_ptk_sha256(const u8 pmk[32], const u8 *aa, const u8 *spa,
		    const u8 *anonce, const u8 *snonce, u8 *ptk, int ptk_len)
{
	u8 b[76];

	if (memcmp(aa, spa, 6) < 0) {
		memcpy(b, aa, 6);
		memcpy(b + 6, spa, 6);
	} else {
		memcpy(b, spa, 6);
		memcpy(b + 6, aa, 6);
	}
	if (memcmp(anonce, snonce, 32) < 0) {
		memcpy(b + 12, anonce, 32);
		memcpy(b + 44, snonce, 32);
	} else {
		memcpy(b + 12, snonce, 32);
		memcpy(b + 44, anonce, 32);
	}
	kdf_sha256(pmk, 32, "Pairwise key expansion", b, 76, ptk, ptk_len * 8);
}

#ifdef SAE_SELFTEST
static int unhex(u8 *out, const char *s, int n)
{
	int i;

	for (i = 0; i < n; i++) {
		int hi, lo;
		char c = s[2 * i];
		char d = s[2 * i + 1];

		if (!c || !d)
			return -1;
		hi = (c >= '0' && c <= '9') ? c - '0' :
		     (c >= 'a' && c <= 'f') ? c - 'a' + 10 : -1;
		lo = (d >= '0' && d <= '9') ? d - '0' :
		     (d >= 'a' && d <= 'f') ? d - 'a' + 10 : -1;
		if (hi < 0 || lo < 0)
			return -1;
		out[i] = (u8)((hi << 4) | lo);
	}
	return 0;
}

static int expect_eq(const char *name, const u8 *got, const char *hex, int n)
{
	u8 want[128];
	int i;

	if (n > (int)sizeof(want) || unhex(want, hex, n) || memcmp(got, want, n)) {
		printf("fail %s ", name);
		for (i = 0; i < n; i++)
			printf("%02x", got[i]);
		printf("\n");
		return -1;
	}
	return 0;
}

int sae_selftest(void)
{
	struct fe a, b, r, one, nm1;
	struct pt g, g2;
	u8 px[32], py[32], sc[32], el[64], sc_a[32], el_a[64];
	u8 rand_s[32], mask_s[32], rand_a[32], mask_a[32];
	u8 kck[32], pmk[32], pmkid[16], conf[32], mic[16], ptk[48];
	u8 mac_a[6], mac_b[6];
	static const u8 cmac_key[16] = {
		0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
		0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
	};
	static const u8 cmac_msg[16] = {
		0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
		0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a
	};
	int i;

	/* (p - 1) + 1 == 0 */
	a = FE_P;
	fe_sub_raw(&a, &a, (fe_set_u32(&one, 1), &one));
	fe_add(&r, &a, &one);
	if (!fe_is_zero(&r)) {
		printf("fail fe add wrap\n");
		return 1;
	}
	fe_mul(&r, &a, &a);
	if (fe_cmp(&r, &one) != 0) {
		int k;

		printf("fail fe mul");
		for (k = 7; k >= 0; k--)
			printf(" %08x", r.w[k]);
		printf("\n");
		return 2;
	}
	/* (n - 1) + 2 == 1 (mod n) */
	fe_set_u32(&one, 1);
	fe_sub_raw(&nm1, &FE_N, &one);
	fe_set_u32(&b, 2);
	fe_add_n(&r, &nm1, &b);
	if (fe_cmp(&r, &one) != 0) {
		printf("fail n add\n");
		return 3;
	}

	unhex(px, "6b17d1f2e12c4247f8bce6e563a440f277037d812deb33a0f4a13945d898c296", 32);
	unhex(py, "4fe342e2fe1a7f9b8ee7eb4a7c0f9e162bce33576b315ececbb6406837bf51f5", 32);
	fe_from_be(&g.x, px);
	fe_from_be(&g.y, py);
	g.inf = 0;
	fe_set_u32(&b, 2);
	point_mul(&g2, &g, &b);
	fe_to_be(px, &g2.x);
	fe_to_be(py, &g2.y);
	if (expect_eq("2gx", px, "7cf27b188d034f7e8a52380304b51ac3c08969e277f21b35a60b48fc47669978", 32) ||
	    expect_eq("2gy", py, "07775510db8ed040293d9ac69f7430dbba7dade63ce982299e04b79d227873d1", 32))
		return 4;

	unhex(mac_a, "001122334455", 6);
	unhex(mac_b, "aabbccddeeff", 6);
	if (sae_derive_pwe(mac_a, mac_b, (const u8 *)"sae-test", 8, px, py))
		return 5;
	if (expect_eq("pwex", px, "ad62dfae906a258aaade5e7a0688b4f9275bff9844497e21c6e6fce89d9c49c4", 32) ||
	    expect_eq("pwey", py, "7c74dd80e5a852aa37ab4cfb850d187c97897d0148be0fadd44de4320b2a029c", 32))
		return 6;

	for (i = 0; i < 32; i++) {
		rand_s[i] = 0x11;
		mask_s[i] = 0x22;
		rand_a[i] = 0x33;
		mask_a[i] = 0x44;
	}
	if (sae_build_commit(px, py, rand_s, mask_s, sc, el))
		return 7;
	if (expect_eq("scalar", sc, "3333333333333333333333333333333333333333333333333333333333333333", 32) ||
	    expect_eq("element", el, "518614781129ec47761799f9f7c89d5398a970774e28b1809623a611d8c573e6dd0dc2b63d56cc61cf3161f5c621662bdf9cbe6f1f0bacc92c75e5d1c8ee6d80", 64))
		return 8;
	if (sae_build_commit(px, py, rand_a, mask_a, sc_a, el_a))
		return 9;
	if (sae_process_commit(px, py, rand_s, sc, el, sc_a, el_a, kck, pmk, pmkid))
		return 10;
	if (expect_eq("kck", kck, "e9dc60d88646b1dfef64152beb75ed548ce1c2edc37f627d8b80f3346b6edae0", 32) ||
	    expect_eq("pmk", pmk, "c4f83dd04acd9feef37184167e6a23478a0e8618022f4e57fd185e95272c7470", 32) ||
	    expect_eq("pmkid", pmkid, "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", 16))
		return 11;
	sae_confirm(kck, 0, sc, el, sc_a, el_a, conf);
	if (expect_eq("confirm", conf, "e7dd19d849a771a31ecd4ef597947db949f6f8a82459fe3b7c7d986ee6a80804", 32))
		return 12;
	{
		u8 kck_a[32], pmk_a[32], pmkid_a[16], conf_a[32];

		if (sae_process_commit(px, py, rand_a, sc_a, el_a, sc, el,
				       kck_a, pmk_a, pmkid_a))
			return 13;
		if (memcmp(pmk, pmk_a, 32)) {
			printf("fail pmk agree\n");
			return 14;
		}
		sae_confirm(kck_a, 0, sc_a, el_a, sc, el, conf_a);
		if (expect_eq("confirm_a", conf_a, "c0c7416e724276c1f4c12034e3e6144551150eff3af4f925d2e30e5b1aaf4ef8", 32))
			return 15;
	}
	sae_aes_cmac(cmac_key, cmac_msg, 16, mic);
	if (expect_eq("cmac", mic, "070a16b46b4d4144f79bdd9dd04a287c", 16))
		return 16;
	sae_aes_cmac(cmac_key, cmac_msg, 0, mic);
	if (expect_eq("cmac0", mic, "bb1d6929e95937287fa37d129b756746", 16))
		return 17;
	{
		u8 an[32], sn[32];

		for (i = 0; i < 32; i++) {
			an[i] = i;
			sn[i] = i + 32;
		}
		sae_ptk_sha256(pmk, mac_b, mac_a, an, sn, ptk, 48);
		if (expect_eq("ptk", ptk, "a6e63c53bad4714f38ebeac42ccd3e20ea4da5d19c05f8fe2be2ce6867a9574d3ffa7d8dffbf7accb25c7fe66f70881b", 48))
			return 18;
	}
	if (sae_derive_pwe_h2e(mac_a, mac_b, (const u8 *)"test", 4,
			       (const u8 *)"sae-test", 8, px, py))
		return 19;
	if (expect_eq("h2ex", px, "d6e923472e6c7cd7bb7bc5f2231116a0dd44a7365d060bce9e2e19c241145876", 32) ||
	    expect_eq("h2ey", py, "7215fbc66faf7a1b7b1550bbfc2da3995a928eea7b676b46d3cb6766b5e3403a", 32))
		return 20;
	printf("sae_selftest ok\n");
	return 0;
}

int main(void)
{
	return sae_selftest();
}
#endif
