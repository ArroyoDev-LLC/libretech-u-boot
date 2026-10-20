/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Open transcription of the G12A/SM1 aml_ddr.fw LPDDR4 timing engine
 * (D-VA 0xFFFB6A1C) and the MID1 DMC/PCTL register writer (D-VA 0xFFFB6328).
 *
 * Address convention (identical to g12a-a644-call-sequence.md): D-VA is the
 * address printed by
 *   aarch64-linux-gnu-objdump -D -b binary -m aarch64 \
 *       --adjust-vma=0xFFFB4000 aml_ddr.fw
 * on the FILE, i.e. D-VA = payload link VA + 0x60.  Every step below cites the
 * D-VA of the instruction it transcribes.
 *
 * This header is pure integer arithmetic over the ddr_set_t (ACS) the TPL
 * already builds.  It touches no MMIO and reads nothing from the DRAM device,
 * so it is compilable and testable on the host -- see
 * amlogic/software/tests/g12a-timing-engine-check.py, which drives exactly
 * these functions and compares their output to the MID1 state captured on
 * hardware (boots p2-a / p2-b).
 *
 * SCOPE: LPDDR4 (ctl+440 == 2), pstate 0.  The DDR3/DDR4/LPDDR3 arms of the
 * vendor function are NOT transcribed and are refused at runtime.
 */

#ifndef __G12A_DDR_TIMING_H__
#define __G12A_DDR_TIMING_H__

/* tm[] is addressed by BYTE offset in the vendor code; we index by word. */
#define G12A_TM_WORDS		64
#define G12A_TM(t, byte_off)	((t)[(byte_off) / 4])

/*
 * The ns->clk idiom the vendor uses ~20 times, archetype at D-VA
 * fffb6d68/fffb6d70/fffb6d74:
 *
 *	w9 = f_MHz * t_ps ; w9 = w9 / 1000000 ; w9 = w9 + 1
 *
 * DEPARTURE FROM JEDEC: this is TRUNCATE-then-+1, not ceil().  When
 * t_ps * f_MHz is an exact multiple of 1e6 the vendor yields one clock MORE
 * than ceil() does.  A "cleaned up" DIV_ROUND_UP() reimplementation is wrong
 * on exactly those inputs.
 */
static inline u32 g12a_tm_clk(u32 t_ps, u32 f_mhz)
{
	return (u32)(t_ps * f_mhz) / 1000000u + 1u;
}

/*
 * The blob's own rodata word at payload VA 0xFFFBF920 (file offset 0xB980 of
 * aml_ddr.fw, since file = payload VA - 0xFFFB4000 + 0x60).  Measured = 1 in
 * the shipped image.  It gates one arm of tm[180] at D-VA fffb7ed0 -- and the
 * same word gates 0xFE120024 in fffb5bc4, which g12a-a644-call-sequence.md
 * sec.7 records as an open question.  It is read here, not assumed.
 */
#define G12A_AML_DDR_F920	1u

/*
 * Two call sites use SDIV where every other site uses UDIV (D-VA fffb7800 for
 * tm[4], fffb7880 for tm[128]).  The products stay well inside 2^31 for every
 * frequency this part supports, so the two agree here -- but the distinction
 * is transcribed rather than smoothed, because it is a real difference in the
 * vendor's code and would diverge at f_MHz above ~76 GHz-equivalent products.
 */
static inline u32 g12a_tm_clk_s(u32 t_ps, u32 f_mhz)
{
	return (u32)((s32)(u32)(t_ps * f_mhz) / 1000000) + 1u;
}

static inline u32 g12a_ds_h(const u8 *ds, u32 off)
{
	return (u32)ds[off] | ((u32)ds[off + 1] << 8);
}

/**
 * g12a_timing_engine() - transcription of aml_ddr.fw fffb6a1c, LPDDR4 arm
 * @ds:      ddr_set_t (ACS) image, as the TPL hands it to the blob
 * @pstate:  ctl+441 (a644 forces this to 0 at step E3)
 * @f920:    the blob's own rodata word at payload VA 0xFFFBF920 (file +0xB980).
 *           Measured = 1 in the shipped aml_ddr.fw; gates one tm[180] arm.
 * @tm:      output, G12A_TM_WORDS u32 words, MUST be zeroed by the caller
 *           (the vendor's tim buffer is zeroed at control-block build, and
 *           tm[232] / tm[216] are read but never written by this function)
 *
 * @tm IS `volatile` ON PURPOSE -- DO NOT "CLEAN IT UP".
 *
 * Measured on Solitude 2026-08-19 (t7-dtd-state-diff.md sec.5).  With a plain
 * `u32 *tm` and the TPL's own -Os + LTO build, four of this function's stores
 * -- tm[32], tm[44], tm[48] and tm[64] -- were NOT observable in memory after
 * the function returned, on both a file-scope static and a stack array, on
 * every boot.  The stores are present in the compiled image (verified in the
 * ELF, in tpl/u-boot-tpl.bin and in the signed .usb.bl2 at the same VA) and no
 * instruction between them and the read stores to those slots; the values were
 * nevertheless gone.  Three of the four were harmless because the inlined
 * g12a_mid1_compute() consumed them from registers; the fourth, tm[64], is the
 * ONE word MID1 re-loads from memory (MID1(0xbd0) -> DMC 0xFF638430), so it
 * programmed 0 where the vendor blob programs 0x1e.
 *
 * That single wrong DMC word was the cause of the random transient DRAM write
 * loss this arc chased for weeks: with @tm volatile, 0xFF638430 reads 0x1e and
 * ALL FIVE walks (DTA/DTB/DTC/DTD/DTE, plus a second DTD pass) go to zero
 * failures on two consecutive boots, from 18-32 of 3928 before.
 *
 * A GCC-level root cause is NOT established -- what is established is the
 * measurement above and that volatile fixes it.  Keep the qualifier.
 *
 * Return: 0 on success, -1 if the ACS is outside the transcribed arm.
 */
static inline int g12a_timing_engine(const u8 *ds, u32 pstate, u32 f920,
				     volatile u32 *tm)
{
	u32 f, fbin, cap, w5c, d119, w7, w11, w22, w23, w24, v;

	if (pstate != 0)
		return -1;	/* fffb6a68 pstate-1 prologue not transcribed */

	f = g12a_ds_h(ds, 56 + 2 * pstate);	/* fffb6a48/fffb6a4c */
	w11 = ds[65];				/* fffb6a54 */
	w7 = ds[66];				/* fffb6a58  density/tRFC code */
	d119 = ds[119];				/* fffb74ac etc. */

	/*
	 * fffb6ac8 / fffb6b48 / fffb6bd4: ds[102] / ds[104] / ds[100] are
	 * default-filled only when zero.  Solitude's ACS ships all three
	 * non-zero (180 / 180 / 250), so the fill arms are dead here and are
	 * refused rather than guessed.
	 */
	if (!g12a_ds_h(ds, 100) || !g12a_ds_h(ds, 102) || !g12a_ds_h(ds, 104))
		return -1;

	/* fffb6c60-fffb6c84: bin selector -- NOT simply the DRAM frequency. */
	cap = g12a_ds_h(ds, 106);
	if (!cap)
		cap = 1600;				/* fffb6c64 */
	fbin = (f < cap) ? f : ((cap - 12) & 0xffff);	/* fffb6c80 */

	/* ---- LPDDR4 speed-bin table, fffb7464-fffb762c ---- */
	w5c = 0;
	if (fbin <= 0x109) {				/* fffb7464 */
		G12A_TM(tm, 28) = 6;
		G12A_TM(tm, 64) = 6;
		G12A_TM(tm, 32) = 4;
		G12A_TM(tm, 60) = 8;
	} else {
		w5c = w11 & 1;				/* fffb7490 */
		if (fbin <= 0x214) {			/* fffb748c */
			G12A_TM(tm, 28) = (w5c << 1) + 0xa;
			G12A_TM(tm, 32) = 6;
			G12A_TM(tm, 64) = (d119 << 1) + 0xa;
			G12A_TM(tm, 60) = 8;
		} else if (fbin <= 0x31f) {		/* fffb74cc */
			G12A_TM(tm, 28) = (d119 << 1) + 0xe + (w5c << 1);
			G12A_TM(tm, 32) = 8;
			G12A_TM(tm, 64) = 0x10;
			G12A_TM(tm, 60) = 8;
		} else if (fbin <= 0x429) {		/* fffb7504 */
			G12A_TM(tm, 28) = (d119 << 1) + 0x14 + (w5c << 1);
			G12A_TM(tm, 32) = 0xa;
			G12A_TM(tm, 64) = (d119 << 1) + 0x14;
			G12A_TM(tm, 60) = 8;
		} else if (fbin <= 0x534) {		/* fffb7544 */
			G12A_TM(tm, 28) = (d119 << 1) + 0x18 + (w5c << 2);
			G12A_TM(tm, 32) = 0xc;
			G12A_TM(tm, 64) = (d119 << 2) + 0x18;
			G12A_TM(tm, 60) = 0xa;
		} else if (fbin <= 0x63f) {		/* fffb7584 */
			/* fffb7588 shifts ds[119] left by 2 BEFORE the test */
			G12A_TM(tm, 28) = (d119 << 2) + 0x1c + (w5c << 2);
			G12A_TM(tm, 32) = 0xe;
			G12A_TM(tm, 64) = (d119 << 1) + 0x1e;
			G12A_TM(tm, 60) = 0xc;
		} else if (fbin <= 0x749) {		/* fffb75c4 */
			G12A_TM(tm, 28) = (d119 << 2) + 0x20 + (w5c << 2);
			G12A_TM(tm, 32) = 0x10;
			G12A_TM(tm, 64) = (d119 << 2) + 0x22;
			G12A_TM(tm, 60) = 0xe;
		} else {				/* fffb7600 */
			G12A_TM(tm, 28) = (d119 << 2) + 0x24 + (w5c << 2);
			G12A_TM(tm, 32) = 0x12;
			G12A_TM(tm, 64) = (d119 << 2) + 0x28;
			G12A_TM(tm, 60) = 0x10;
		}
	}

	G12A_TM(tm, 60) += 2;				/* fffb7630 */
	G12A_TM(tm, 36) = g12a_tm_clk(44000, f) + 2;	/* fffb7640 (+3, not +1) */
	G12A_TM(tm, 56) = g12a_tm_clk(40000, f);	/* fffb7658 */
	G12A_TM(tm, 76) = 8;				/* fffb7674 */
	G12A_TM(tm, 80) = 8;				/* fffb767c */
	v = g12a_tm_clk(12000, f);			/* fffb7680 */
	G12A_TM(tm, 48) = (v <= 3) ? 4 : v;		/* fffb7698 */
	G12A_TM(tm, 52) = G12A_TM(tm, 48);		/* fffb76b0 */
	v = g12a_tm_clk(10000, f);			/* fffb76b4 */
	G12A_TM(tm, 68) = (v <= 7) ? 8 : v;		/* fffb76c8 */
	G12A_TM(tm, 72) = G12A_TM(tm, 68);		/* fffb76e0 */

	/* tRFCpb / tRFCab by density code ds[66], fffb76dc-fffb778c */
	if (w7 == 9) {					/* fffb76e4 */
		G12A_TM(tm, 12) = g12a_tm_clk(60000, f);
		G12A_TM(tm, 8) = g12a_tm_clk(130000, f);
	} else if (w7 == 0xa) {				/* fffb7710 */
		G12A_TM(tm, 12) = g12a_tm_clk(90000, f);
		G12A_TM(tm, 8) = g12a_tm_clk(180000, f);
	} else if (w7 == 0xb) {				/* fffb7758 */
		G12A_TM(tm, 12) = g12a_tm_clk(180000, f);
		G12A_TM(tm, 8) = G12A_TM(tm, 12);
	} else {					/* fffb776c */
		G12A_TM(tm, 12) = g12a_tm_clk(280000, f);
		G12A_TM(tm, 8) = G12A_TM(tm, 12);
	}

	G12A_TM(tm, 16) = g12a_tm_clk(21000, f);	/* fffb7790 */
	G12A_TM(tm, 24) = G12A_TM(tm, 68);		/* fffb77b0 */
	G12A_TM(tm, 20) = G12A_TM(tm, 28);		/* fffb77b4 */
	G12A_TM(tm, 248) = g12a_tm_clk(1500, f);	/* fffb77b8 */
	G12A_TM(tm, 252) = g12a_tm_clk(3500, f);	/* fffb77d4 */
	v = g12a_tm_clk_s(14000, f);			/* fffb77f0 SDIV */
	G12A_TM(tm, 4) = (v <= 9) ? 0xa : v;		/* fffb7808 */
	G12A_TM(tm, 84) = 0x1ff;			/* fffb7818 */

	/* ---- common tail, fffb7820-fffb78dc ---- */
	G12A_TM(tm, 40) = G12A_TM(tm, 36) + G12A_TM(tm, 16);	/* fffb7830 */
	G12A_TM(tm, 124) = 300;				/* fffb7838 */
	G12A_TM(tm, 88) = G12A_TM(tm, 84);		/* fffb7844 */
	G12A_TM(tm, 92) = 15;				/* fffb7858 */
	G12A_TM(tm, 104) = 15;				/* fffb785c */
	G12A_TM(tm, 108) = 15;				/* fffb7860 */
	G12A_TM(tm, 112) = 15;				/* fffb7864 */
	G12A_TM(tm, 144) = 15;				/* fffb7868 */
	G12A_TM(tm, 96) = 0x300;			/* fffb7870 */
	G12A_TM(tm, 100) = 0x80;			/* fffb788c */
	G12A_TM(tm, 152) = 0x3f;			/* fffb7890 */
	G12A_TM(tm, 116) = 0x1f;			/* fffb789c */
	G12A_TM(tm, 128) = g12a_tm_clk_s(30000, f);	/* fffb7880 SDIV */
	G12A_TM(tm, 140) = 500;				/* fffb78b0 */
	G12A_TM(tm, 156) = 0x20;			/* fffb78b4 */
	G12A_TM(tm, 44) = G12A_TM(tm, 28);		/* fffb78c0 */
	G12A_TM(tm, 120) = 5;				/* fffb78c4 */
	G12A_TM(tm, 132) = 8;				/* fffb78cc */
	G12A_TM(tm, 136) = 16;				/* fffb78d0 */
	G12A_TM(tm, 160) = 6;				/* fffb78d4 */
	G12A_TM(tm, 164) = 0x98;			/* fffb78d8 */

	w23 = G12A_TM(tm, 232);	/* fffb78dc -- read, never written: 0 */
	w24 = G12A_TM(tm, 32);				/* fffb78e0 */
	w22 = G12A_TM(tm, 28);				/* fffb7884 */

	/*
	 * The LPDDR4 mode-register block (fffb7cf8-fffb7e60) writes
	 * tm[256 + 64*pstate ...] -- the MR1/MR2/MR3/MR11/MR12/MR14/MR22 words
	 * consumed by the message-block prepare (fffbe244) and fffb4bc4.  MID1
	 * reads none of them, so they are outside this header's scope; see
	 * timing-engine-transcription.md sec.5.
	 */
	G12A_TM(tm, 148) = 0x26;			/* fffb7e64 */

	/* ---- MR-latency tail, fffb7e6c-fffb7f10 ---- */
	G12A_TM(tm, 168) = 0xf;				/* fffb7e70 */
	w24 = w23 + w24;				/* fffb7e74 */
	G12A_TM(tm, 172) = 2;				/* fffb7e7c */
	w22 = w23 + w22;				/* fffb7e80 */
	G12A_TM(tm, 176) = w24 - 5;			/* fffb7e84 */
	G12A_TM(tm, 180) = w22 - 5;			/* fffb7e8c */
	if (ds[14] == 3) {				/* fffb7e94 */
		w24 -= 4;
		G12A_TM(tm, 176) = w24;
	}
	if (ds[14] == 2) {				/* fffb7ea8 */
		G12A_TM(tm, 176) += 2;			/* fffb7eb8 */
		G12A_TM(tm, 180) = w22 - 4;		/* fffb7ec0 */
		if (f920 == 0) {			/* fffb7ed0 */
			w22 -= 3;
			G12A_TM(tm, 180) = w22;
		}
	}
	G12A_TM(tm, 184) = 0x22;			/* fffb7ee0 */
	G12A_TM(tm, 188) = 0x18;			/* fffb7ee8 */
	G12A_TM(tm, 192) = 0x30;			/* fffb7ef0 */
	G12A_TM(tm, 196) = 0x1000;			/* fffb7ef8 */
	G12A_TM(tm, 200) = 0;				/* fffb7f08 */
	G12A_TM(tm, 204) = 0;				/* fffb7f0c */
	G12A_TM(tm, 208) = 0x00030f0f;			/* fffb7f10 */
	return 0;
}

/* MID1 register count -- the 0xFF6384xx window only. */
#define G12A_MID1_N	52

struct g12a_mid1_reg {
	u32 addr;
	u32 val;
};

/**
 * g12a_mid1_compute() - transcription of aml_ddr.fw fffb6328 (a644 step S5)
 * @ds:     ddr_set_t
 * @pstate: ctl+441
 * @tm:     output of g12a_timing_engine() (may be mutated -- see tm[120])
 * @out:    G12A_MID1_N entries
 *
 * Only the 0xFF6384xx window is emitted; the fixed 0xFF6386xx stores at
 * fffb6578-fffb67d8 are a separate concern (they are not in the captured
 * comparison window) and are left to the caller.
 *
 * Return: number of entries written.
 */
static inline int g12a_mid1_compute(const u8 *ds, u32 pstate, u32 *tm,
				    struct g12a_mid1_reg *out)
{
	u32 f = g12a_ds_h(ds, 56 + 2 * pstate);		/* fffb6338 */
	u32 base = (u32)((pstate << 8) - 0x9c7000);	/* fffb633c/fffb6364 */
	int n = 0;

	if (f > 0x320 && ds[14] == 3)			/* fffb6340-fffb6358 */
		G12A_TM(tm, 120) = 6;

#define MID1(disp, value) do { out[n].addr = base - (disp); \
			       out[n].val = (value); n++; } while (0)
#define MIN_U(a, b)	 ((a) <= (b) ? (a) : (b))

	MID1(0xbfc, MIN_U(G12A_TM(tm, 8), 0x3ffu));		/* fffb6374 */
	MID1(0xc00, MIN_U(G12A_TM(tm, 4), 0xfu));		/* fffb638c */
	MID1(0xbf8, G12A_TM(tm, 20) | (G12A_TM(tm, 16) << 16));	/* fffb63a0 */
	MID1(0xbec, G12A_TM(tm, 32));				/* fffb63ac */
	MID1(0xbf0, G12A_TM(tm, 28));				/* fffb63b8 */
	MID1(0xbe8, G12A_TM(tm, 36));				/* fffb63c4 */
	MID1(0xbe4, G12A_TM(tm, 40));				/* fffb63d0 */
	MID1(0xbe0, G12A_TM(tm, 44));				/* fffb63dc */
	MID1(0xbdc, G12A_TM(tm, 48) | (G12A_TM(tm, 52) << 16));	/* fffb63f0 */
	MID1(0xbc8, G12A_TM(tm, 76) | (G12A_TM(tm, 80) << 16));	/* fffb6404 */
	MID1(0xbcc, G12A_TM(tm, 68) | (G12A_TM(tm, 72) << 16));	/* fffb6418 */
	MID1(0xbd4, G12A_TM(tm, 60));				/* fffb6424 */
	MID1(0xbd0, G12A_TM(tm, 64));				/* fffb6430 */
	MID1(0xbd8, G12A_TM(tm, 56));				/* fffb643c */
	MID1(0xbc4, 0x3ff);					/* fffb6444 */
	MID1(0xbbc, MIN_U(G12A_TM(tm, 92), 0xfu));		/* fffb6458 */
	MID1(0xba0, G12A_TM(tm, 120));				/* fffb6464 */
	MID1(0xbf4, G12A_TM(tm, 24));				/* fffb6470 */
	MID1(0xbb0, MIN_U(G12A_TM(tm, 104), 0x1fu));		/* fffb6488 */
	MID1(0xbac, MIN_U(G12A_TM(tm, 108), 0x1fu));		/* fffb649c */
	MID1(0xba4, G12A_TM(tm, 116));				/* fffb64a8 */
	MID1(0xba8, G12A_TM(tm, 112));				/* fffb64b4 */
	MID1(0xbb4, MIN_U(G12A_TM(tm, 100), 0xffu));		/* fffb64cc */
	MID1(0xbb8, G12A_TM(tm, 96));				/* fffb64d8 */
	MID1(0xb90, G12A_TM(tm, 136));				/* fffb64e4 */
	MID1(0xbc0, 0x3ff);					/* fffb64ec */
	MID1(0xb84, G12A_TM(tm, 172));				/* fffb64f8 */
	MID1(0xb80, G12A_TM(tm, 176));				/* fffb6504 */
	MID1(0xb7c, G12A_TM(tm, 180));				/* fffb6510 */
	MID1(0xb78, G12A_TM(tm, 184));				/* fffb6544 */
	MID1(0xb88, G12A_TM(tm, 168));				/* fffb6550 */
	MID1(0xb60, G12A_TM(tm, 208));				/* fffb655c */
	MID1(0xb74, G12A_TM(tm, 188));				/* fffb6568 */
	MID1(0xb70, G12A_TM(tm, 192));				/* fffb6574 */
	MID1(0xb50, G12A_TM(tm, 144));				/* fffb665c */
	MID1(0xb4c, 0x20100000u | (G12A_TM(tm, 148) << 8)
		    | ((f / 20u) - 1u));			/* fffb6680 */
	MID1(0xb38, 0xa);					/* fffb668c */
	MID1(0xb34, (f / 200u) * 1000u);			/* fffb66a4 */
	MID1(0xb30, G12A_TM(tm, 44) + 3);			/* fffb66b4 */
	MID1(0xb9c, G12A_TM(tm, 124));				/* fffb66c0 */
	MID1(0xb98, 0x3f);					/* fffb66cc */
	MID1(0xb94, G12A_TM(tm, 132));				/* fffb66d8 */
	MID1(0xb8c, G12A_TM(tm, 140));				/* fffb66e4 */
	MID1(0xb6c, G12A_TM(tm, 196));				/* fffb66f0 */
	MID1(0xb68, G12A_TM(tm, 200));				/* fffb66fc */
	MID1(0xb64, G12A_TM(tm, 204));				/* fffb6708 */
	MID1(0xb58, 0xf);					/* fffb6714 */
	MID1(0xb54, G12A_TM(tm, 216));				/* fffb6720 */
	MID1(0xb48, 0x3f);					/* fffb6728 */
	MID1(0xb44, G12A_TM(tm, 156));				/* fffb6734 */
	MID1(0xb40, G12A_TM(tm, 160));				/* fffb6740 */
	MID1(0xb3c, G12A_TM(tm, 164));				/* fffb674c */

#undef MID1
#undef MIN_U
	return n;
}

#endif /* __G12A_DDR_TIMING_H__ */
