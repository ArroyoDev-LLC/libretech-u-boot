/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Open transcription of the G12A/SM1 aml_ddr.fw host-side steps that are pure
 * computation over the ddr_set_t (ACS) the TPL already builds:
 *
 *   fffb7f30  a644 step S4 -- PHY static config, 105 CSRs
 *		(phy-static-config-transcription.md, 103/104 on hardware)
 *   fffb7cf8  the LPDDR4 mode-register block inside the timing engine
 *		(message-block-transcription.md, 12/12 MR bytes on silicon)
 *   fffbe244  a644 step S3 -- PMU message-block prepare, 37 field stores
 *		(message-block-transcription.md, 36/37 fields, 124/126 bytes)
 *
 * The timing engine (fffb6a1c) and MID1 (fffb6328) live in the sibling header
 * g12a-ddr-timing.h.
 *
 * Address convention: D-VA is the address printed by
 *   aarch64-linux-gnu-objdump -D -b binary -m aarch64 \
 *       --adjust-vma=0xFFFB4000 aml_ddr.fw
 * on the FILE, i.e. D-VA = payload link VA + 0x60.  A *data* address the code
 * computes is a payload VA; its file offset is VA - 0xFFFB4000 + 0x60.
 *
 * None of this touches MMIO or reads anything from the DRAM device: every
 * function here is a pure function of the ACS, so it is compilable and
 * checkable on the host.  The emitters return {csr, value} / {offset, value}
 * lists; the caller does the writing.
 *
 * SCOPE: LPDDR4 (ctl+440 == 2), pstate 0.  The DDR3/DDR4/LPDDR3 arms and the
 * pstate-1 prologue are NOT transcribed and are refused at runtime.
 */

#ifndef __G12A_DDR_OPEN_H__
#define __G12A_DDR_OPEN_H__

/*
 * Data copied verbatim out of the shipped aml_ddr.fw (md5
 * eddca53f01322e2a7f4a8aca1b89ea3a, 48080 B).  18 bytes for the whole
 * project; these are 14 of them (the timing engine's 0xFFFBF920 is the
 * other 4 and lives in g12a-ddr-timing.h).
 */
#define G12A_OPEN_ROD_F924	1u	/* u32 @ payload 0xFFFBF924, file +0xB984 */
#define G12A_OPEN_ROD_F92C	1u	/* u16 @ payload 0xFFFBF92C, file +0xB98C */
/*
 * u32 @ payload 0xFFFBFB70 -- one word PAST the 0xBB70-byte payload, i.e. BSS
 * the blob never writes.  Read at fffb837c (DbyteDllModeCntrl) and fffb7cf8
 * (MR1 bit 3).  Corroborated 0 on hardware twice: DqsPreambleControl = 0x1CB
 * with DbyteDllModeCntrl = 0, and MR1 = 0x54 with bit 3 clear.
 */
#define G12A_OPEN_BSS_FB70	0u

/* 8-byte rank-config table @ payload VA 0xFFFBF390, file +0xB3F0. */
static const u8 g12a_open_rank_tbl[8] = {
	0x01, 0x01, 0x0f, 0x03, 0x01, 0x03, 0x03, 0x05
};

struct g12a_open_csr {
	u32 csr;
	u16 val;
};

struct g12a_open_mbf {
	u16 off;
	u8 width;
	u16 val;
};

#define G12A_OPEN_PHY_N		112	/* fffb7f30 writes 105 distinct CSRs */
#define G12A_OPEN_MB_N		48	/* 37 fields 1D, 42 fields 2D */

static inline u32 g12a_open_h(const u8 *ds, u32 off)
{
	return (u32)ds[off] | ((u32)ds[off + 1] << 8);
}

/* ------------------------------------------------------------------ leaves */

/**
 * g12a_open_imp() - fffb682c, the impedance / drive-strength encoder
 *
 * DEPARTURE (silent when wrong): the `csinc w5, w5, wzr, eq` at fffb68a0 is
 * "0 if zero, else 1", NOT "1 if zero".  Reading it the natural way adds
 * exactly one LSB to every impedance code -- TxImpedanceCtrl0/1 come out
 * 0x0E79 instead of 0x0E38, which is a structurally valid code and boots.
 * That single polarity accounted for 32 of the 33 first-pass misses when this
 * function was first transcribed.
 */
static inline u32 g12a_open_imp(u32 code, u32 mode)
{
	u32 w0 = 0, w1 = mode & 0xffffu, w2 = code & 0xffffu;

	if (w2)
		w0 = 480u / w2;				/* fffb6840 sdiv */
	if (((w1 & 0xfffffffdu) & 0xffffu) == 1u) {	/* fffb6844-6850 */
		u32 w3 = (w0 >> 2) & 0xffu;		/* fffb6854 ubfx */
		u32 w4 = (w3 <= 3u) ? w3 : 3u;		/* fffb6860 csel */
		u32 w5, w0b, w2b, w4b;

		w3 = w4 & 0xffu;
		w0 &= 0xffu;
		w4 = (w0 - (w4 << 2)) & 0xffu;		/* fffb6870 */
		w4 = (w4 >> 1) & 0x7fu;			/* fffb6874 ubfx */
		w3 = (w4 < 3u) ? w3 : 2u;		/* fffb6880 csel */
		w0 = w0 - (w3 << 2);			/* fffb6884 */
		w5 = (w0 - (w4 << 1)) & 0xffu;		/* fffb6888/688c */
		w4b = (1u << w4) - 1u;			/* fffb6890/689c */
		w5 = (w5 == 0u) ? 0u : 1u;		/* fffb68a0 csinc */
		w0b = (1u << w3) - 1u;			/* fffb6898/68a4 */
		w4b <<= 1;				/* fffb68a8 */
		w2b = (1u << w5) - 1u;			/* fffb68ac/68b4 */
		w0 = ((w4b | (w0b << 3)) | w2b) & 0xffffu;	/* fffb68b0-68bc */
	}
	if (w1 == 2u) {					/* fffb68c0 */
		w0 = (w0 >> 2) & 0xffu;			/* fffb68c8 */
		if (w0 > 6u)
			w0 = 6u;			/* fffb68d4 */
		w0 = (1u << w0) - 1u;			/* fffb68dc/68e0 */
		w0 = (w0 >> 1) & 0xffffu;		/* fffb68e4 */
	}
	return w0 & 0x7fu;				/* fffb68e8 */
}

/** g12a_open_pllctrl2() - fffb67e4, frequency band -> PllCtrl2 code. */
static inline u32 g12a_open_pllctrl2(u32 f)
{
	f &= 0xffffu;
	if (f <= 0x1d5u)
		return 7u;
	if (f <= 0x270u)
		return 6u;
	if (f <= 0x3a9u)
		return 0xbu;
	if (f <= 0x4e1u)
		return 0xau;
	return (f > 0x855u) ? 0x19u : 0x18u;		/* fffb6818-6824 */
}

/**
 * g12a_open_vref() - fffb68f0, the four vref encoders
 *
 * DEPARTURE (measured on silicon, the most expensive place in this work to be
 * wrong): the mode the mode-register block selects is `ds[67] + 3`, and
 * ds[67] is the LPDDR4X flag.  Mode 3 is JEDEC LPDDR4 (10.0 % + 0.4 %/code);
 * mode 4 is the LPDDR4X base (15.0 % + 0.6 %/code).  Solitude takes mode 4 and
 * hardware reads MR12 = 0x10 / MR14 = 0x05; hard-coding JEDEC LPDDR4 gives
 * 0x47 / 0x14 -- both plausible, both wrong, and both would train.
 */
static inline u32 g12a_open_vref(u32 v, u32 mode)
{
	v &= 0xffffu;					/* fffb68f8 */
	if (mode == 1u) {				/* fffb68f4 */
		if (v <= 0x27au)			/* fffb6900 */
			return ((((v / 5u) + 1u) << 3) | 4u) & 0xffffu;
		return (((v - 0x159u) / 5u) << 3) & 0xffffu;
	}
	if (mode == 2u) {				/* fffb6930 */
		if (v <= 0x257u)
			return ((v * 100u - 45000u) / 650u) & 0xffffu;
		return ((v * 100u - 60000u) / 650u) & 0xffffu;
	}
	if (mode == 3u) {				/* fffb6968 */
		if (v <= 0xdbu)
			return ((v * 100u - 10000u) / 400u) & 0xffffu;
		return (((v * 100u - 22000u) / 400u) & 0xffffu) | 0x40u;
	}
	if (mode == 4u) {				/* fffb69ac */
		if (v > 0x274u)				/* fffb69b8 */
			v = 0x275u;
		else if (v <= 0x96u)			/* fffb69c0 */
			v = 0x96u;
		if (v <= 0x1c6u)			/* fffb69c8 */
			return ((v * 100u - 15000u) / 600u) & 0xffffu;
		return (((v * 100u - 32900u) / 600u) & 0xffffu) | 0x40u;
	}
	return 0u;					/* fffb69b0 */
}

/** g12a_open_atxdly() - fffba1e0 mode 1, the ANIB ATxDly encoding. */
static inline u32 g12a_open_atxdly(u32 c)
{
	if (c <= 0x3fu)					/* fffba1ec */
		return (c & 0x1fu) + ((c >> 5) << 6);	/* fffba1f4-1fc */
	return (c & 0x1fu) + 0x60u;			/* fffba204/208 */
}

/** g12a_open_rank_cfg() - fffb62fc, bounds-checked table lookup on ds[18]-1. */
static inline u32 g12a_open_rank_cfg(u32 v)
{
	u32 i = (v - 1u) & 0xffu;			/* fffb6300/6304 */

	if (i > 7u)					/* fffb6308/6310 */
		return 0u;
	return g12a_open_rank_tbl[i];			/* fffb6320 */
}

/* ------------------------------------------------- fffb7f30: PHY static cfg */

static inline int g12a_open_emit(struct g12a_open_csr *t, int n, u32 csr,
				 u32 val)
{
	int i;

	/*
	 * fffb7f30 genuinely writes DfiMode, TristateModeCA, ARdPtrInitVal and
	 * the whole DfiFreqXlat block MORE THAN ONCE, and last-write-wins is
	 * load-bearing: emitting only the first pass leaves DfiFreqXlat2 at
	 * 0x5555 where hardware has 0x4444, and DfiMode at 5 where it is 3.
	 * Replacing in place reproduces that, in order.
	 */
	for (i = 0; i < n; i++)
		if (t[i].csr == csr) {
			t[i].val = (u16)val;
			return n;
		}
	if (n >= G12A_OPEN_PHY_N)
		return n;
	t[n].csr = csr;
	t[n].val = (u16)val;
	return n + 1;
}

/**
 * g12a_open_phy_static() - transcription of aml_ddr.fw fffb7f30 (a644 step S4)
 * @ds:     ddr_set_t (ACS)
 * @pstate: ctl+441; a644 forces 0
 * @out:    G12A_OPEN_PHY_N entries
 *
 * Return: number of distinct CSRs, or -1 if the ACS is outside the arm.
 */
static inline int g12a_open_phy_static(const u8 *ds, u32 pstate,
				       struct g12a_open_csr *out)
{
	u32 f, typ, ps = pstate, P, v30, w21, w21lo, w23, w27, w28, w1;
	u32 atximp, procodt, w0flag = 0, bit64, n8, pat, dq, t2, w25, w26;
	u32 v74, v72, i, b;
	int n = 0;

	if (ps != 0)
		return -1;	/* fffb7fec pstate-1 arm not transcribed */
	typ = ds[14];						/* fffb7f84 */
	if (typ != 2)
		return -1;	/* only the LPDDR4 arm is transcribed */

	P = ps << 20;
	f = g12a_open_h(ds, 56 + 2 * ps);			/* fffb7f60/64 */

	n = g12a_open_emit(out, n, 0xd0000u, 0);		/* fffb7f7c/80 */

	v30 = g12a_open_imp(g12a_open_h(ds, 30), 3);		/* fffb7f68-70 */
	w23 = v30 | ((v30 & 0xffffu) << 6);			/* fffb80e8 */

	w21lo = g12a_open_imp(g12a_open_h(ds, 78), 1);		/* fffb7f8c-94 */
	w28 = g12a_open_imp(g12a_open_h(ds, 80), 1);		/* fffb7f9c-a4 */
	w27 = g12a_open_imp(g12a_open_h(ds, 82), 3);		/* fffb7fa8-b8 */
	w1 = g12a_open_imp(g12a_open_h(ds, 84), 3);		/* fffb7fc0-c8 */

	/*
	 * DEPARTURE (silent): `csel w1, w1, wzr, ne` KEEPS the value when the
	 * compare FAILS.  For LPDDR4 the natural reading zeroes the ds[84]
	 * term and keeps ds[82], giving TxOdtDrvStren = 0x39; hardware has
	 * 0x0E00 -- the other field, shifted 6.  Same magnitude class,
	 * different field, and it boots either way.
	 */
	if ((typ & 0xfffffffdu) == 1u)				/* fffb80cc-d4 */
		w1 = 0;
	if (typ == 2u)						/* fffb80d8/dc */
		w27 = 0;
	w27 |= (w1 << 6);					/* fffb80e0 */
	w21 = w21lo | (w28 << 6);				/* fffb80e4 */

	/* ---- DBYTE loop, d = 0..3 (fffb80fc-fffb82bc) ---- */
	dq = 0x5a1u;						/* typ == 2 */
	for (i = 0; i < 4; i++) {
		u32 base = P | 0x10000u | (i << 12);
		static const u16 lo[5] = { 0x5f, 0x49, 0x41, 0x4b, 0x4d };
		u32 k;

		for (k = 0; k < 5; k++) {
			u32 val = (k == 0) ? g12a_open_h(ds, 98) :
				  (k == 1 || k == 2) ? w21 :
				  (k == 3) ? w23 : w27;

			n = g12a_open_emit(out, n, base | lo[k], val);
			n = g12a_open_emit(out, n, base | (lo[k] + 0x100u),
					   val);
		}
		n = g12a_open_emit(out, n, base | 0x43u, dq);
		n = g12a_open_emit(out, n, base | 0x143u, dq);
	}

	/* ---- DqsPreambleControl / DbyteDllModeCntrl (fffb8364-fffb83f4) ---- */
	{
		u32 g = G12A_OPEN_BSS_FB70;
		u32 w6 = (g & 1u) ? 10u : 11u;			/* fffb83a4-ac */
		u32 w5 = ((G12A_OPEN_ROD_F924 & 0x3ffu) << 6) |
			 (G12A_OPEN_ROD_F92C << 8);		/* fffb83c0/c4 */
		u32 w1b = ((g & 1u) == 0u) ? 0x80u : 0u;	/* fffb83c8/cc */

		n = g12a_open_emit(out, n, 0x2003au,
				   ((g & 0xffffu) & 1u) << 1);	/* fffb838c-9c */
		n = g12a_open_emit(out, n, P | 0x20024u, w1b | w6 | w5);
		procodt = (G12A_OPEN_ROD_F92C != 0u) ? 3u : 0xffffffffu;
	}
	if (procodt == 0xffffffffu) {				/* fffb8400-842c */
		if (f <= 0x3a5u)
			procodt = 10u;
		else if (f <= 0x4b0u)
			procodt = w0flag ? 2u : 6u;
		else
			procodt = w0flag ? 3u : 7u;
	}

	n = g12a_open_emit(out, n, P | 0x20056u, procodt);	/* fffb8430-44c */
	n = g12a_open_emit(out, n, P | 0x200b2u,
			   g12a_open_vref(g12a_open_h(ds, 102), 1));	/* fffb8458-64 */
	n = g12a_open_emit(out, n, P | 0x200fau, 1);		/* fffb8468-7c */
	n = g12a_open_emit(out, n, P | 0x20019u, (u32)ds[16] << 1); /* fffb8480-98 */
	n = g12a_open_emit(out, n, P | 0x2002du, ds[65] & 1u);	/* fffb849c-b4 */
	n = g12a_open_emit(out, n, P | 0x200c5u, g12a_open_pllctrl2(f));
	bit64 = ((u32)ds[64] >> ps) & 1u;			/* fffb84d8-e4 */
	n = g12a_open_emit(out, n, P | 0x2002eu,
			   ((f >= 0x3a5u) ? 2u : 1u) + bit64);	/* fffb84e8-508 */
	n = g12a_open_emit(out, n, P | 0x20008u, f >> 1);	/* fffb850c-20 */

	atximp = g12a_open_imp(g12a_open_h(ds, 76), 2);		/* fffb8524-2c */
	atximp = ((atximp << 5) | atximp) & 0xffffu;		/* fffb8530-3c */

	/* ---- ANIB loop, i = 0..9 (fffb8540-fffb85d0) ---- */
	for (i = 0; i < 10; i++) {
		u32 idx = P | 0x80u | (i << 12);
		u32 ov;

		n = g12a_open_emit(out, n, (i << 12) | 0x55u,
				   g12a_open_h(ds, 96));	/* fffb855c-70 */
		n = g12a_open_emit(out, n, (i << 12) | 0x43u, atximp);
		n = g12a_open_emit(out, n, idx,
				   g12a_open_atxdly(ds[108 + i]));
		ov = ds[ps * 0x1acu + 320u + i];		/* fffb85ac/b0 */
		if (ov)
			n = g12a_open_emit(out, n, idx,
					   g12a_open_atxdly(ov));
	}

	/*
	 * ---- per-ANIB slew-rate overrides (fffb861c-fffb86ac) ----
	 * WHICH four ANIBs are overridden depends on the DRAM type: 0/9 and
	 * 4/5 for DDR3/DDR4, 0/5 and 1/6 for LPDDR4/LPDDR3.
	 *
	 * DEPARTURE (silent): ARdPtrInitVal is written a SECOND time here with
	 * an unrelated, NON-MONOTONE bin -- <=0x4AF -> 1, <=0x513 -> 2,
	 * <=0x63F -> 1, >0x63F -> 3.  The third arm is LOWER than the second.
	 * Implementing only the first write gives 2 where hardware reads 3.
	 */
	{
		u32 w1c;

		if (f <= 0x4afu)
			w1c = 1u;
		else if (f <= 0x513u)
			w1c = typ;
		else
			w1c = (f > 0x63fu) ? 3u : 1u;		/* fffb864c-54 */
		n = g12a_open_emit(out, n, P | 0x2002eu,
				   w1c + (((u32)ds[64] >> ps) & 1u));
	}
	n = g12a_open_emit(out, n, 0x00055u, g12a_open_h(ds, 94)); /* fffb8670- */
	n = g12a_open_emit(out, n, 0x05055u, g12a_open_h(ds, 94));
	n = g12a_open_emit(out, n, 0x01055u, g12a_open_h(ds, 92));
	n = g12a_open_emit(out, n, 0x06055u, g12a_open_h(ds, 92));

	/* ---- ANIB impedance + DfiMode / DfiCAMode (fffb86b0-fffb8874) ---- */
	t2 = (typ - 2u) & 0xffu;
	w25 = (t2 < 2u) ? 6u : 5u;				/* fffb86b4-c8 */
	w26 = (t2 >= 2u) ? 4u : 1u;				/* fffb86d8/dc */
	v74 = g12a_open_imp(g12a_open_h(ds, 74), 2);		/* fffb86d0-e0 */
	v74 = ((v74 << 5) | v74) & 0xffffu;
	n = g12a_open_emit(out, n, 0x00043u, v74);		/* fffb86ec-f8 */
	n = g12a_open_emit(out, n, 0x09043u, v74);		/* fffb86fc-704 */
	v72 = g12a_open_imp(g12a_open_h(ds, 72), 2);		/* fffb8708-10 */
	v72 = ((v72 << 5) | v72) & 0xffffu;
	n = g12a_open_emit(out, n, (w26 << 12) | 0x43u, v72);	/* fffb871c-34 */
	n = g12a_open_emit(out, n, (w25 << 12) | 0x43u, v72);	/* fffb8730-3c */
	n = g12a_open_emit(out, n, 0x20018u, 5);		/* fffb8740-48 */

	/* typ == 2 arm (fffb87e0-fffb8874) */
	n = g12a_open_emit(out, n, P | 0x20019u, 1);		/* fffb87e0/e4 */
	n = g12a_open_emit(out, n, 0x20075u, 4);		/* fffb87e8-f4 */
	n = g12a_open_emit(out, n, P | 0x20204u, 0);		/* fffb87f8-80c */
	n = g12a_open_emit(out, n, 0x20018u, 3);		/* fffb8810-20 */
	if (ds[18] == 7u || ds[18] == 2u)			/* fffb882c-38 */
		n = g12a_open_emit(out, n, 0x20018u, 5);
	if (ds[18] == 4u || ds[18] == 1u)			/* fffb884c-5c */
		n = g12a_open_emit(out, n, 0x20018u, 5);
	n = g12a_open_emit(out, n, P | 0x20019u, 1);		/* fffb8870/74 */

	/* ---- calibration + DfiFreqXlat (fffb8878-fffb8a30) ---- */
	n = g12a_open_emit(out, n, 0x20050u, 0);		/* fffb8878-80 */
	n = g12a_open_emit(out, n, 0x20088u, 9);		/* fffb8884-90 */
	n8 = ds[64] & 1u;					/* fffb8894-9c */
	pat = (n8 << 8) | (n8 << 4) | n8 | (n8 << 12);		/* fffb88a0-ac */
	n = g12a_open_emit(out, n, 0x200f0u,
			   (pat + 0x5000u + 0x555u) & 0xffffu);	/* fffb88b0-c4 */
	for (i = 0xf1u; i < 0xf7u; i++)				/* fffb88c8-e4 */
		n = g12a_open_emit(out, n, 0x20000u | i, 0x5555u);
	n = g12a_open_emit(out, n, 0x200f7u, 0xf000u);		/* fffb88e8-f4 */

	/* the REWRITE -- fffb89a0-fffb8a2c, LPDDR4/LPDDR3 only */
	{
		u32 w3 = 0;					/* fffb895c */
		u32 w6 = (pat + 0x5000u + 0x555u) & 0xffffu;	/* fffb8978-7c */

		if (f <= 0x14cu)				/* fffb8944/60 */
			w3 = (5u << (ps << 2)) & 0xffffu;	/* fffb8964-74 */
		w3 = (w3 + pat) & 0xffffu;			/* fffb8980 */
		for (i = 0; i < 8; i++) {			/* fffb89a0-a2c */
			u32 v;

			if (i == 0)
				v = w3;
			else if (i == 2)
				v = 0x4444u;
			else if (i == 3)
				v = 0x8888u;
			else if (i == 4)
				v = w6;
			else if (i == 7)
				v = 0xf000u;
			else
				v = 0;				/* fffb8a14-20 */
			n = g12a_open_emit(out, n, 0x20000u | (0xf0u + i),
					   v);
		}
	}

	/* ---- disabled-DBYTE parking + tail (fffb890c-fffb8acc) ---- */
	n = g12a_open_emit(out, n, 0x20025u, 0);		/* fffb890c-14 */
	/*
	 * ds[15]'s LOW nibble GATES the parking writes; it is not a
	 * DBYTE-enable mask.  Solitude ships 0xF0, so none of the four fire;
	 * reading the nibble as "enable" and inverting it parks all four.
	 */
	for (b = 0; b < 4; b++)					/* fffb8918/8a38/8a60/8a88 */
		if (((u32)ds[15] >> b) & 1u) {
			n = g12a_open_emit(out, n, 0x10000u | (b << 12), 4);
			n = g12a_open_emit(out, n,
					   0x10000u | (b << 12) | 0x4au,
					   0x7ffu);
		}
	n = g12a_open_emit(out, n, 0x20060u, 3);		/* fffb8aac-b8 */
	n = g12a_open_emit(out, n, 0x20021u, 0);		/* fffb8abc-c4 */
	n = g12a_open_emit(out, n, 0xd0000u, 0);		/* fffb8ac8/cc */
	return n;
}

/* -------------------------------- fffb7cf8: the LPDDR4 mode-register block */

/**
 * g12a_open_bin_code() - the LPDDR4 frequency-bin selector (fffb7464-fffb762c)
 *
 * Returns the register the MR block calls w5 -- the nWR/RL code -- and tm[32],
 * the WL value, through @tm32.
 *
 * TRAP: `fbin` is NOT the DRAM frequency.  It is f when f < cap and
 * (u16)(cap - 12) otherwise, with cap = ds.u16[106] ?: 1600.  Solitude runs
 * 1608 MHz against cap 1600 and therefore bins as 1588, taking the <= 0x63F
 * arm.  Binning on 1608 takes the next arm and changes four DMC registers.
 */
static inline u32 g12a_open_bin_code(const u8 *ds, u32 pstate, u32 *tm32)
{
	u32 f = g12a_open_h(ds, 56 + 2 * pstate);
	u32 cap = g12a_open_h(ds, 106);
	u32 fbin, b65 = ds[65] & 1u, d119 = ds[119];

	if (!cap)
		cap = 1600u;
	fbin = (f < cap) ? f : ((cap - 12u) & 0xffffu);

	if (fbin <= 0x109u) { *tm32 = 4;  return 0; }
	if (fbin <= 0x214u) { *tm32 = 6;  return 1; }
	/*
	 * fffb74fc is `mov w5, w4` -- w4 is ctl[440], the DRAM TYPE, not a
	 * literal 2.  It equals 2 only because LPDDR4 is type 2.
	 */
	if (fbin <= 0x31fu) { *tm32 = 8;  return 2; }
	if (fbin <= 0x429u) { *tm32 = 10; return 3; }
	if (fbin <= 0x534u) { *tm32 = 12; return 4; }
	if (fbin <= 0x63fu) { *tm32 = 14; return 5; }
	if (fbin <= 0x749u) { *tm32 = 16; return 6; }
	(void)b65; (void)d119;
	*tm32 = 18;
	return 7;
}

/**
 * g12a_open_mr_block() - fffb7cf8-fffb7e60, twelve mode registers
 * @mr: output, 12 entries in the order MR1 2 3 4 11 12 13 14 16 17 22 24
 *
 * Twelve, not fifteen: slots 0/5/6/7 of the 16-slot per-pstate array belong to
 * the DDR3/DDR4/LPDDR3 arms and stay untouched on the LPDDR4 path.
 *
 * DEPARTURE (silent): the same `240/x` idiom is treated THREE ways in 91
 * instructions -- MR3 PDDS and MR11 DQ-ODT are CLAMPED to 6, MR11 CA-ODT and
 * MR22 SoC-ODT are MASKED with 7.  They agree only while 240/x <= 6.
 */
static inline int g12a_open_mr_block(const u8 *ds, u32 pstate, u8 *mr)
{
	u32 code, tm32, pdds, dqodt, caodt, soh, vmode;

	if (ds[14] != 2u)
		return -1;
	code = g12a_open_bin_code(ds, pstate, &tm32);

	mr[0] = (u8)(4u | ((G12A_OPEN_BSS_FB70 & 1u) << 3) | (code << 4));
								/* MR1  fffb7d00-7d18 */
	mr[1] = (u8)(code | (((tm32 >> 1) - 2u) << 3));	/* MR2  fffb7d1c-7d48 */

	pdds = 240u / g12a_open_h(ds, 86);		/* fffb7d4c */
	if (pdds > 6u)
		pdds = 6u;				/* fffb7d64/7d70 */
	mr[2] = (u8)((u32)ds[118]			/* fffb6ac4 -> fffb7d3c */
		     | (G12A_OPEN_ROD_F924 << 1)	/* fffb7d38/7d50 */
		     | (((u32)ds[65] & 1u) << 6)	/* fffb7d54/7d60 */
		     | ((((u32)ds[65] >> 1) & 1u) << 7)	/* fffb7d58-7d6c */
		     | (pdds << 3));			/* fffb7d74 */
	mr[3] = 0;					/* MR4  fffb7d8c */

	dqodt = 240u / g12a_open_h(ds, 88);		/* fffb7d90 */
	if (dqodt > 6u)
		dqodt = 6u;				/* fffb7d98/7da0 */
	caodt = (240u / g12a_open_h(ds, 90)) & 7u;	/* fffb7da4 */
	mr[4] = (u8)(dqodt | (caodt << 4));		/* MR11 fffb7da8 */

	vmode = (u32)ds[67] + 3u;			/* fffb7dc8/7de8 */
	mr[5] = (u8)g12a_open_vref(g12a_open_h(ds, 100), vmode);	/* MR12 */
	mr[6] = 0;					/* MR13 fffb7dbc */
	mr[7] = (u8)g12a_open_vref(g12a_open_h(ds, 104), vmode);	/* MR14 */
	mr[8] = 0;					/* MR16 fffb7e0c */
	mr[9] = 0;					/* MR17 fffb7e18 */
	soh = g12a_open_h(ds, 82);			/* fffb7e1c-24 */
	if (!soh)
		soh = g12a_open_h(ds, 84);
	mr[10] = (u8)(((240u / soh) & 7u) | (ds[67] ? 0u : 0x18u));	/* MR22 */
	mr[11] = 0;					/* MR24 fffb7e60 */
	return 0;
}

/* --------------------------------- fffbe244: the PMU message-block prepare */

/**
 * g12a_open_msgblock() - transcription of aml_ddr.fw fffbe244 (a644 step S3)
 * @ds:     ddr_set_t
 * @pstate: ctl+441
 * @is2d:   0 for the 1D block (ctl[234] == 1), 1 for the 2D block
 * @out:    G12A_OPEN_MB_N entries, {byte offset, width, value}
 *
 * Return: number of fields, or -1 outside the transcribed arm.
 *
 * NOTE the 2D arm also writes msg1d[18] = 4 and copies DFIMRLMargin out of the
 * 1D block -- both are side effects on the OTHER buffer and are reported by
 * the caller-visible flag rather than being in this list.
 */
static inline int g12a_open_msgblock(const u8 *ds, u32 pstate, int is2d,
				     struct g12a_open_mbf *out)
{
	u8 mr[12];
	u32 rc, cs_a, cs_b, enq_a = 0x10u, enq_b = 0x10u, seq;
	int n = 0, i;

#define MBF(o, wdt, v) do { out[n].off = (o); out[n].width = (wdt); \
			    out[n].val = (u16)(v); n++; } while (0)

	if (g12a_open_mr_block(ds, pstate, mr))
		return -1;

	rc = g12a_open_rank_cfg(ds[18]) & 0xffu;	/* fffbe270-e27c */
	cs_a = rc & 3u;					/* fffbe290 */
	cs_b = (rc >> 2) & 3u;				/* fffbe298/e30c */

	MBF(0, 1, 0);					/* Reserved00 fffbe2f0 */
	MBF(1, 1, 0);					/* MsgMisc    fffbe2e4 */
	MBF(4, 1, pstate);				/* Pstate     fffbe2c8 */
	MBF(5, 1, ((u32)ds[64] >> pstate) & 1u);	/* PllBypassEn */
	MBF(6, 2, g12a_open_h(ds, 56 + 2 * pstate) * 2u);	/* DRAMFreq */
	MBF(8, 1, 2);					/* DfiFreqRatio */
	MBF(9, 1, 0);					/* BPZNResVal */
	MBF(10, 1, 0);					/* PhyOdtImpedance */
	MBF(11, 1, 0);					/* PhyDrvImpedance */
	MBF(12, 1, (g12a_open_h(ds, 102) << 7) / 1000u);	/* PhyVref sdiv */
	MBF(13, 1, 0);					/* Lp4Misc */

	/*
	 * SequenceCtrl: the 1D arm reads ds.u16[44] and the 2D arm ds.u16[46].
	 * The `& ~0x100` fires only when pstate != 0 (fffbe2d0 branches PAST
	 * the `and`), and a644 forces pstate 0, so it never fires here --
	 * applying it unconditionally would drop RdDeskew from the 1D
	 * sequence.  The 2D arm has no mask at all.
	 */
	if (!is2d) {
		seq = g12a_open_h(ds, 44);		/* fffbe2cc */
		if (pstate != 0)
			seq &= 0xfeffu;			/* fffbe2d4 */
	} else {
		seq = g12a_open_h(ds, 46);		/* fffbe490/498 */
	}
	MBF(16, 2, seq);
	MBF(18, 1, ds[17]);				/* HdtCtrl fffbe2e0 */
	MBF(22, 1, 3);					/* DFIMRLMargin */
	if (!is2d)
		MBF(25, 1, 0);				/* Lp4Quickboot */
	MBF(24, 1, 1);					/* UseBroadcastMR */
	MBF(27, 1, 0);					/* CATrainOpt */
	MBF(28, 1, 0);					/* X8Mode */
	MBF(34, 2, 0);					/* PhyConfigOverride */

	/*
	 * The EnabledDQs override is a single-CHANNEL gate, not a DQ-enable
	 * mask: when it fires it also zeroes EnabledDQsChB and CsPresentChB.
	 * Solitude ships ds[18] = 3, so it does NOT fire and both channels
	 * stay at 0x10; reading it as an enable mask parks channel B.
	 */
	if (!is2d && (ds[18] == 7u || ds[18] == 1u || ds[18] == 2u ||
		      ds[18] == 4u)) {			/* fffbe32c-e348 */
		enq_a = ((u32)ds[15] & 0xfu) ? 0x10u : 0x20u;	/* fffbe34c-64 */
		enq_b = 0;				/* fffbe368 */
		cs_b = 0;				/* fffbe36c */
	}
	MBF(36, 1, enq_a);
	MBF(37, 1, cs_a);				/* CsPresentChA */
	MBF(87, 1, enq_b);
	MBF(88, 1, cs_b);				/* CsPresentChB */
	MBF(74, 1, ds[48] & 0xfu);			/* CATerminatingRankChA */
	MBF(125, 1, ds[49] & 0xfu);			/* CATerminatingRankChB */

	for (i = 0; i < 12; i++)			/* fffbe3a0-e45c */
		MBF(50 + i, 1, mr[i]);

	if (is2d) {					/* fffbe61c-e63c */
		MBF(14, 1, 0);				/* Reserved0E */
		MBF(29, 1, 0);
		MBF(30, 1, 0);
		MBF(31, 1, 0);
		MBF(32, 1, 0x20);			/* Delay_Weight2D */
		MBF(33, 1, 0x80);			/* Voltage_Weight2D */
	}
#undef MBF
	return n;
}

#endif /* __G12A_DDR_OPEN_H__ */
