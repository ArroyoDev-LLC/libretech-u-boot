/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2023-2024, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#ifndef DRAM_SETTINGS_GX_H
#define DRAM_SETTINGS_GX_H
#include <linux/bitops.h>
#include <asm/arch/dram-gx.h>

/*
 * These registers are pretty similar to other DRAM registers found in
 * Allwinner A31/sun6i. Some of these registers also exist in some Rockchip
 * SoCs and the TI KeyStone3.
 */
/*
 * DMC_DDR_CTRL.  Field authority is the vendor register header, which
 * documents this register bit by bit and agrees across two SoC families:
 * arch-txlx/secure_apb.h at DMC_DDR_CTRL, and arch-axg/regs.h.
 *
 *	bit 22	DDR4 SDRAM enable -- "use DDR4 protocal"
 *	bit 21	rank 1 enable -- "if 1, rank1 used the address map is as
 *		bit 5:3 defined"
 *	bit 20	BG group 1 enable, DDR4 only
 *	bit 16	"1 only use 16bits data in a 32bits phy data interface"
 *	[5:3]	rank 1 size, [2:0] rank 0 size, both encoded 3'bN = 128 MB << N
 *		(3'b011 = 1 GB, 3'b100 = 2 GB, 3'b101 = 4 GB)
 *
 * So a rank is selected by a RANGE DECODE off those two size fields --
 * rank 0 owns [0, size0), rank 1 owns [size0, size0 + size1) -- and NOT by
 * any ADDRMAP field.  meson_dram_dmc_set_size() builds the word.
 *
 * What stood here before mapped CONFIG_DRAM_TWO_IDENTICAL_RANKS to BIT(22),
 * the DDR4 protocol enable, and CONFIG_DRAM_ONE_RANK to BIT(21), the rank 1
 * enable: "two ranks" put a DDR3 part into DDR4 protocol and "one rank"
 * turned the second one on.  Read back off a live GXL DMC on 2026-08-08 as
 * DMC_DDR_CTRL=0x00200064 -- bit 21 set from a ONE_RANK build.
 */
#define DMC_CTRL_CHANNEL	BIT(6)	/* Channel 0 only */
#define DMC_CTRL_16BIT		BIT(16)	/* 16 of the 32 PHY data bits */
#define DMC_CTRL_BG1_EN		BIT(20)	/* DDR4 bank group 1 */
#define DMC_CTRL_RANK1_EN	BIT(21)	/* rank 1 range decode enabled */
#define DMC_CTRL_DDR4		BIT(22)	/* use the DDR4 protocol */
#define DMC_CTRL_RANK1_SHIFT	3
#if defined(CONFIG_DRAM_DDR4)
#define DMC_CTRL_DDR_TYPE	(DMC_CTRL_DDR4 | DMC_CTRL_BG1_EN)
#else
#define DMC_CTRL_DDR_TYPE	0
#endif

/*
 * GXL DDR4 read DBI (data-bus inversion on reads, MR5 A12).
 *
 * Off on the 32-bit-bus board (AML-S905X-CC-V2): with it on, CC-V2 at 1080 MHz
 * inverts one whole byte when hot (0xe4 -> 0x1b at ~77-79 C, 2 of 2 boots,
 * gxl dram n1231); with it off, 0 errors on 3 hot boots (n1232) and the
 * five-boot landing soak (n1233).  The DBI pin's timing is never trained (it
 * reads zero on every boot on record), the leading suspect.
 *
 * On for the 16-bit-rank board (AML-S805X-AC): it was not tested there, so it
 * keeps the set it shipped with.
 *
 * Read DBI adds 2 tCK of read latency, so every read-side latency below moves
 * with it, as in the vendor BL2 (gxl_bl2_ddr_deep.c adds 2 to cfg_ddr_cl and
 * dfitrddataen when RDBI is on):
 *
 *			read DBI on	read DBI off
 *	MR0 CL code	CL20 (A2 set)	CL18 (A2 clear)
 *	MR5		0x1400		0x400
 *	PCTL_TCL	20		18
 *	DFITRDDATAEN	16		14
 *	DTCR[15:14]	set for S2	not set
 */
#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR4) && \
	!defined(CONFIG_DRAM_16BIT_RANK)
#define GXL_DDR4_READ_DBI	0
#else
#define GXL_DDR4_READ_DBI	1
#endif
#define GXL_DDR4_RDBI_TCK	(GXL_DDR4_READ_DBI ? 2 : 0)

/* Mode Register */
#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR4)
/*
 * A[6:4] = (cl - 9) >> 1 with A2 clear is the JEDEC CL18 code for cl = 18;
 * A2 set makes it CL20, the CL read DBI needs (see GXL_DDR4_READ_DBI).
 */
#define PUB_MR0			(GXL_DDR4_READ_DBI ? 4 : 0) | \
	(((((timings.cl - 9) >> 1) & 7) << 4)) | \
	((((timings.wr - 10) >> 1) & 7) << 9)
#define PUB_MR1			(timings.odt << 8) | (timings.drv << 1) | 0x81
#define PUB_MR2			(((timings.cwl - 6) >> 1) & 7) << 3 | 0xc0
#define PUB_MR3			0
#define PUB_MR4			8
#else
/*
 * MR0 A11-A9 is JEDEC's Write Recovery code, and it is NOT a linear field:
 *
 *	000 = 16   001 = 5   010 = 6   011 = 7
 *	100 = 8    101 = 10  110 = 12  111 = 14
 *
 * JESD79-3F 3.4.2.5 requires WR >= tWR(min), so a value that is not directly
 * encodable must round UP to the next code, never down.  Anything above 16 is
 * not expressible at all and clamps to 16 -- the only case where this macro
 * cannot satisfy the requirement, and it cannot be caught at build time
 * because `timings` is a const object rather than a preprocessor constant.
 *
 * What was here before OR-ed a constant 0x1c00 over the field, presetting A12,
 * A11 and A10, so the `wr` input could only ever reach A9 and the code was
 * pinned to 11x.  Our wr = 16 encodes to 000 -- a zero term -- which the mask
 * then turned into 110 = WR 12, against a tWR(min) of 14 at 912 MHz
 * (15 ns / 1.0965 ns, rounded up).  The parameter handling was already right;
 * the constant was the entire bug, which is why a fix that only checks wr = 16
 * would not be a fix.
 *
 * A12 in that constant is Precharge Power-Down fast exit.  That is legal and
 * consistent with PCTL_TXP = 7, so it stays; only A11/A10 were wrong to force.
 *
 * This diverges from vendor BL2, which programs the same masked constant
 * (gxl_bl2_ddr_deep.c:1912-1914).  The violation is inherited, not introduced
 * here, and correcting it is a deliberate departure from vendor behaviour.
 */
#define PUB_MR0_WR_CODE(w)						\
	((w) <= 5  ? 1 :						\
	 (w) == 6  ? 2 :						\
	 (w) == 7  ? 3 :						\
	 (w) == 8  ? 4 :						\
	 (w) <= 10 ? 5 :						\
	 (w) <= 12 ? 6 :						\
	 (w) <= 14 ? 7 : 0)

#define PUB_MR0			(((timings.cl - 4) & 8) >> 1) | \
	(((timings.cl - 4) & 7) << 4) | \
	((PUB_MR0_WR_CODE(timings.wr) & 7) << 9) | 0x1000
#define PUB_MR1			(timings.drv << 1) | \
	((timings.odt & 1) << 2)        | \
	(((timings.odt >> 1) & 1) << 6) | \
	(((timings.odt >> 2) & 1) << 9) | \
	BIT(7)			        | \
	((timings.al ? ((timings.cl - timings.al) & 3) : 0) << 3)
#define PUB_MR2			BIT(6) | (((timings.cwl - 5) & 7) << 3)
#endif
#define PUB_MR3			0
#if defined(CONFIG_MESON_GXL)
#if defined(CONFIG_DRAM_DDR3)
#define PUB_MR4			0
#define PUB_MR5			0x420
#elif defined(CONFIG_DRAM_DDR4)
/*
 * RUN K (2026-08-10) — match product's LIVE mode-register set.
 *
 * The old note here ("ACS/product final MR5 is not 0x1400") came from a
 * stage-8 dump.  The RUN I full live diff measured product BL2 on this board
 * at MR5 = 0x00001400 against our 0x00000400, so product's final MR5 DOES
 * carry bit 12 (read DBI).  MR5 alone was already tried and is cleanly
 * negative (tip a4d7b715a27, "MARK rdbi-s2 DTCR=8000f0c7 MR5=00001400",
 * memtest FAIL got0=ffffffff imm_fail=6), so this is only meaningful as part
 * of the whole MR set: MR1 bit7 restored + MR11 = 0 (see dram-gxl.c and the
 * VREF block in dram-gx.c).  DTCR |= 0xc000 for S2 is already in the tree.
 *
 * Bit 12 is now per board: GXL_DDR4_READ_DBI.
 */
#define PUB_MR5		(GXL_DDR4_READ_DBI ? 0x1400 : 0x400)
/*
 * ACS t_pub_soc_vref_dram_vref (CC-V2 timing.c):
 *   CONFIG_SOC_VREF=71, CONFIG_DRAM_VREF=72 → pack 0x2612.
 * Vendor gxl_bl2_ddr_deep applies before S5; open used fixed
 * VTCR0 0xf0032000 + PIR VREF only (ACS red row).
 */
/*
 * RUN U.  0x26 is what the ACS pack decodes to and what c99bf7fc494 landed;
 * product BL2 measured live on this board lands DXnGCR5 = 0x09090925 on all
 * four lanes (RUN S, five boots, stable on both sides), i.e. SoC VREF code
 * 0x25 -- one LSB below ours.  This is a seed we chose, not a training
 * outcome, and it sits on the read reference of the data path RUN H put the
 * fault in.  Non-zero either way, so neither the `if (PUB_SOC_VREF_CODE)`
 * gate nor the auto-VREF fallback at dram-gx.c:547 changes behaviour.
 */
#define PUB_SOC_VREF_CODE	0x25
#define PUB_DRAM_VREF_CODE	0x12
/* MR6[12:10]=tCCD_L, MR6[5:0]=DRAM VREF (ACS) */
#define PUB_MR6			(((timings.tccdl - 4) * 0x400) | \
				 PUB_DRAM_VREF_CODE)
#endif
#ifndef PUB_MR6
#define PUB_MR6			0x800
#endif
#endif

/* ODT Configuration Register */
#if defined(CONFIG_MESON_GXBB)
#define PUB_ODTCR	0x210000
#elif defined(CONFIG_MESON_GXL)
#define PUB_ODTCR	0x30000
#endif

/* DDR Timing Parameter */
#if defined(CONFIG_MESON_GXBB)
#define PUB_DTPR0	timings.rtp | \
	(timings.wtr << 4)  | \
	(timings.rp  << 8)  | \
	(timings.ras << 16) | \
	(timings.rrd << 22) | \
	(timings.rcd << 26)
#define PUB_DTPR1	(timings.mod << 2) | \
	(timings.faw << 5)    | \
	(timings.rfc << 11)   | \
	(timings.wlmrd << 20) | \
	(timings.wlo << 26)
#define PUB_DTPR2	timings.xs | \
	(timings.xp << 10)   | \
	(timings.dllk << 19)
#define PUB_DTPR3	0 | \
	(0 << 3)            | \
	(timings.rc << 6)   | \
	(timings.cke << 13) | \
	(timings.mrd << 18) | \
	(0 << 29)
#elif defined(CONFIG_MESON_GXL)
#define PUB_DTPR0	timings.rtp | \
	(timings.rp  << 8)  | \
	(timings.ras << 16) | \
	(timings.rrd << 24)
#define PUB_DTPR1	(timings.wlmrd << 24) | \
	(timings.faw << 16) | \
	timings.mrd
#define PUB_DTPR2	timings.xs | \
	(timings.cke << 16)
#define PUB_DTPR3	(timings.dllk << 16) | (4 << 28)
#define PUB_DTPR4	timings.xp | BIT(11) | (timings.rfc << 0x10)
#define PUB_DTPR5	(timings.rc << 16) | (timings.rcd << 8) | \
	timings.wtr
#endif

#if defined(CONFIG_MESON_GXBB)
#define PUB_PGCR0	0x7D81E3F
#define PUB_PGCR1	0x380C6A0
#define PUB_PGCR2	(0x1F12480 & 0xefffffff)
#define PUB_PGCR3	0xC0AAFE60
#elif defined(CONFIG_MESON_GXL)
#define PUB_PGCR0	0x7d81e3f
#define PUB_PGCR1	0x2004620
#define PUB_PGCR2	(0xf05f97 & 0xefffffff)
/*
 * RUN T.  Vendor boot-lite timing.c has t_pub_pgcr3 = 0xc0aae860 for DDR3 and
 * DDR4, and on that basis e20f62757d5 dropped the DDR4-only `| 0x4000000` this
 * file carried since 17d80a3b326, with the comment "not in vendor ACS".
 *
 * That justification is FALSIFIED by product BL2 measured live on this board:
 * RUN S read PGCR3 0xc883601c = 0xc4aae860 on product against our 0xc0aae860,
 * i.e. product's live value is ours | 0x04000000 (bit 26) exactly.  It is the
 * only row left in the 1534-word register-file diff with that shape, and it is
 * the same shape as the DSGCR row RUN I found and RUN J landed.  The vendor
 * GXL BL2 ORs the bit inside its DDR4 branch only (gxl bl2.c:2656 in the
 * reversing-gxbb-bl2 decompile), with no board, rank-width or ddr_func gate.
 * Every DDR4 boot since 2026-08-31 runs with it set and passes memtest.
 *
 * Read DBI (GXL_DDR4_READ_DBI) does not touch this word, on purpose.  In the
 * SynQuacer DDR4 init for this PUB layout, the read-DBI set changes PGCR3
 * [27:25] (sets bit 27) and [7:5] (RDBICL = 3), next to MR0 CL, MR5 and the
 * DTCR DBI-deskew bits.  The read-DBI CL fields only apply while read DBI is
 * on and their select bit is set (ZynqMP DDR PHY: "When RDBI enabled, if
 * RDBICLSEL is asserted, RDBI CL adjust using this value").  Here bit 27 is
 * clear in both arms, [7:5] = 3 is also what DDR3 runs, which has no DBI, and
 * bit 26 is set by the vendor whether or not ddr_func enables read DBI.  So
 * the read-DBI-off arm (CC-V2) keeps the vendor's PGCR3, and only the fields
 * read DBI owns -- MR0 CL, MR5, PCTL_TCL, DFITRDDATAEN, DTCR[15:14] -- follow
 * the switch.  CC-V2 with read DBI off: 0 errors over the n1233 5-boot soak
 * and n1234 (gxl dram audits in the claude amlogic repo).
 */
#if defined(CONFIG_DRAM_DDR4)
#define PUB_PGCR3	(0xc0aae860 | 0x04000000)
#else
#define PUB_PGCR3	0xc0aae860	/* the vendor DDR3 branch leaves bit 26 clear */
#endif
#endif

#if defined(CONFIG_MESON_GXBB)
#define PUB_DXCCR	0x181884
#define PUB_DTCR	0x4300308f
#define PUB_DSGCR	0x20645A

#define PUB_ZQ0PR	0x69
#define PUB_ZQ1PR	0x69
#define PUB_ZQ2PR	0x69
#define PUB_ZQ3PR	0x69
#elif defined(CONFIG_MESON_GXL)
/*
 * DXCCR: DX Common Configuration Register
 * Vendor boot-lite uses 0x20c01ee4 for all GXL boards.
 * (source: boot-lite board/<board>/firmware/timing.c)
 */
#define PUB_DXCCR	0x20c01ee4

#if defined(CONFIG_DRAM_DDR4)
#define PUB_DTCR	0x80003187 | 0x40
#else
#define PUB_DTCR	0x80003187
#endif

#define PUB_DTCR1	0x00010237 /* XXX: Needed? */
/*
 * DSGCR: DX System General Configuration Register
 * Base value 0x20641b from vendor.  Bits 2 (ZUEN) and 23 (RRMODE)
 * are cleared during training (dram-gxl.c) when present.
 * Vendor boot-lite: CC DDR3=0x020641b; CC-V2 DDR4=0x020641b (no RRMODE);
 * AC DDR4 may OR (1<<2)|(1<<23) — open SPL GXL DDR4 followed CC-V2.
 *
 * RUN J (2026-08-10): that "no RRMODE" note is FALSIFIED by measurement of
 * product BL2 on this exact board.  The RUN I full 1534-word live diff read
 * DSGCR @0xc8836090 as 0x0020441b (open) vs 0x00a0441f (product) — product is
 * ours | 0x800004 EXACTLY, i.e. bit 2 (ZUEN) + bit 23 (RRMODE) are BOTH set on
 * product.  Seed them here so they are live through training, not forced after
 * it (a post-train force of the identical value was already measured, and only
 * a post-train force: tip 0e807bc0c0c, "dsgcr-vendor-live pre=0020441b
 * post=00a0441f", negative).  DDR3 already used this constant.
 */
#define PUB_DSGCR	(0x20641b | 0x800004)

/*
 * ZQ calibration: board/PCB-specific impedance values.
 * Source: vendor boot-lite timing.c per board.
 *
 * DDR3 (CC):     ZQ0PR=0x5d95d  ZQ1PR=0x5d95d  ZQ2PR=0x5d95d
 * DDR4 (CC-V2):  ZQ0PR=0x0995d  ZQ1PR=0x3f95d  ZQ2PR=0x3f95d
 * DDR4 16-bit (AC): ZQ0PR=0x0775d  ZQ1PR=0x6fc5d  ZQ2PR=0x6fc5d
 */
#if defined(CONFIG_DRAM_DDR3)
#define PUB_ZQ0PR	0x5d95d
#define PUB_ZQ1PR	0x5d95d
#define PUB_ZQ2PR	0x5d95d
#define PUB_ZQ3PR	0x1dd1d
#elif defined(CONFIG_DRAM_DDR4) && defined(CONFIG_DRAM_16BIT_RANK)
/* AML-S805X-AC: 2-layer DDR4 16-bit */
#define PUB_ZQ0PR	0x0775d
#define PUB_ZQ1PR	0x6fc5d
#define PUB_ZQ2PR	0x6fc5d
#define PUB_ZQ3PR	0x1dd1d
#elif defined(CONFIG_DRAM_DDR4)
/* AML-S905X-CC-V2: 4-layer DDR4 32-bit */
#define PUB_ZQ0PR	0x0995d
#define PUB_ZQ1PR	0x3f95d
#define PUB_ZQ2PR	0x3f95d
#define PUB_ZQ3PR	0x1dd1d
#endif
#endif

/*
 * DCR: DRAM Configuration Register
 * Vendor boot-lite ACS: DDR3=0x0B, DDR4=0x40C.
 * RE ddr_pre_init (gxl_bl2_ddr_deep.c, ddrtype==1) upgrades DDR4 to
 * 0x1800040c (ACS 0x40c | bits 27+28). Open used plain 0x40c — open-bus
 * residual candidate after ACS DQS-corr bank.
 * 2T mode bit (28) is OR'd separately in dram-gxl.c (no-op if set).
 */
#if defined(CONFIG_DRAM_DDR3)
#define PUB_DCR	0xb
#elif defined(CONFIG_DRAM_DDR4)
#define PUB_DCR	0x1800040c
#endif
#define PUB_DTAR	(0 | (0 << 12) | (0 << 28)) /* Uh? */

#define PCTL0_1US_PCK	0x1C8
#define PCTL0_100NS_PCK	0x2D
#define PCTL0_INIT_US	0x2
#define PCTL0_RSTH_US	0x2

/* Mode Config(?) */
#if defined(CONFIG_MESON_GXBB)
#define PCTL0_MCFG	((((timings.faw + timings.rrd - 1) / timings.rrd) & 3) << 0x12) | \
	(0xa2f21 & 0xfff3ffff)
#define PCTL0_MCFG1	(((timings.rrd - ((timings.faw - (timings.faw / timings.rrd) * \
	timings.rrd) & 0xff)) & 7) << 8) | \
	(0x80200000 & 0xfffffcff)
#elif defined(CONFIG_MESON_GXL)
#if defined(CONFIG_DRAM_DDR3)
#define PCTL0_MCFG_DDRTYPE	0
#elif defined(CONFIG_DRAM_DDR4)
#define PCTL0_MCFG_DDRTYPE	BIT(4)
#endif

#define PCTL0_MCFG	(0xa2f21 & 0xffffff8f) | PCTL0_MCFG_DDRTYPE
/* XXX: What is this?           ↓ ??? */
#define PCTL0_MCFG1	0
#endif

#define PCTL0_SCFG	0xF01

#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_16BIT_RANK)
#define PCTL0_PPCFG	0x1fd
#else
/* RE ddr_pre_init RANK0 forces 0x1e0 (ACS seed 0 banked no residual win). */
#define PCTL0_PPCFG	0x1e0
#endif

#define PCTL0_DFISTCFG0	0x4
#define PCTL0_DFISTCFG1	0x1

#define PCTL0_DFITCTRLDELAY	0x2

#if defined(CONFIG_MESON_GXBB)
#define PCTL0_DFITPHYWRDATA	0x1
#else
#define PCTL0_DFITPHYWRDATA	0x2
#endif

#if defined(CONFIG_MESON_GXBB)
#define PCTL0_DFITPHYWRLTA	(timings.cwl + timings.al - \
	(((timings.cwl + timings.al) % 2) ? 3 : 4)) / 2
#define PCTL0_DFITRDDATAEN	(timings.cl + timings.al - \
	(((timings.cl + timings.al) % 2) ? 3 : 4)) / 2
#define PCTL0_DFITPHYRDLAT	((timings.cl + timings.al) % 2) ? 14 : 16
#elif defined(CONFIG_MESON_GXL)
/*
 * GXL DFI latencies: open formula (cwl+al-2)/(cl+al-4) → WRLAT=10
 * RDDATAEN=14 for CL18/CWL12. Vendor ACS seeds 7/8 hang on first DRAM
 * access (measured tip c84999863ee / 94fce96931f) — worse than formula
 * open-bus got0=ffffffff. Prefer formula; do not force ACS 7/8.
 * PHYRDLAT 0x16 matches ACS (22) and live open dump.
 */
#define PCTL0_DFITPHYWRLTA	((timings.cwl + timings.al) - 2)
/*
 * RUN AA (2026-08-10) -- THE ONE VARIABLE OF THIS BUILD.
 *
 * The RUN I / RUN S full live register diff (1534 words per boot, 0
 * excluded, two boots per firmware) compared the WHOLE PCTL block
 * 0xc8839000..0xc88392fc, which contains every read-latency word.  Of
 * that set only TWO differ open vs product, and both are on the READ
 * side and both by exactly +2 on product:
 *
 *   PCTL_TCL         0xc883902c   ours 0x12 (18)  product 0x14 (20)
 *   PCTL_DFITRDDATAEN 0xc88390b0  ours 0x0e (14)  product 0x10 (16)
 *
 * Everything else in the latency set is byte-IDENTICAL between the two
 * firmwares: PCTL_TAL 0xc8839140 (0), PCTL_TCWL 0xc8839030 (12),
 * PCTL_DFITPHYWRLAT 0xc88390ac (10 = cwl+al-2), PCTL_DFITPHYRDLAT
 * 0xc88390b4 (0x16), and -- decisively -- PUB_MR0 0xc8836180, the
 * DRAM-side CAS-latency encoding, which never appears in the diff.  So
 * the DRAM is told the SAME CL by both firmwares and only the
 * CONTROLLER's read-side expectation differs: product waits 2 tCK
 * longer than we do for the same DRAM CL.
 *
 * Direction and magnitude both match the measured defect.  RUN W
 * measured the read capture opening 2 beats = 8 bytes EARLY
 * (read(A) = mem[A-8], reproduced on the host BROM probe in RUNS X and
 * Y/Z).  Granularity, from code in this tree and not from prose: the
 * GXBB arm of this very #if computes the same register as
 * (cl + al - 4) / 2 -- an explicit tCK -> DFI-clock halving -- while
 * the GXL arm omits the /2.  One DFI clock is therefore 2 tCK, and a
 * read enable programmed 2 tCK short lands exactly ONE DFI clock =
 * 2 beats = 8 bytes early.  That is the observed shift, exactly.
 *
 * Changed here: the read data enable only.  PCTL_TCL is deliberately
 * left at timings.cl so this build has one variable; it is the
 * follow-up if the capture moves but the memtest does not.
 *
 * Prior-work check: `git log -S DFITRDDATAEN` finds exactly one force,
 * c84999863ee (vendor ACS 7/8), which is 6 BELOW the formula and hung
 * the first DRAM access; it was reverted in 684dc59b543.  This value
 * has never been raised, and 16 is product's own measured live value.
 */
/*
 * DDR4-SCOPED, and the scope is measured, not cautious: with this fix live on
 * the DDR3 arm the AML-S905X-CC open SPL trains (PGSR0 0x80000fff) and then
 * HANGS before its size print, 1/1 (2026-08-10, variant E).  The missing /2 is
 * a GXL-wide defect and the right long-term fix is GXL-wide, but DDR3 needs
 * its own bring-up for it -- see ddr4-spl-timing-gaps.md 2026-08-10.
 */
#if defined(CONFIG_DRAM_DDR4)
/* TCL - 4: 16 with read DBI on, 14 with it off (GXL_DDR4_READ_DBI) */
#define PCTL0_DFITRDDATAEN	((timings.cl + timings.al) - 4 + GXL_DDR4_RDBI_TCK)
#else
#define PCTL0_DFITRDDATAEN	((timings.cl + timings.al) - 4)
#endif
#define PCTL0_DFITPHYRDLAT	0x16
#endif

/*
 * RUN AB (2026-08-10) -- THE ONE VARIABLE OF THIS BUILD: PCTL_TCL.
 *
 * 🔴 HISTORY, read with this: the analysis below (and RUN AA's above) is the
 * read-DBI-ON set, then the only one.  Since 2026-10-14 GXL_DDR4_READ_DBI
 * selects it: ON keeps exactly the values below (AML-S805X-AC); OFF
 * (AML-S905X-CC-V2) clears MR0 A2 to CL18 and drops TCL and DFITRDDATAEN by 2
 * (amlogic gxl dram n1231-n1233).  Where the text says "always set" or "20",
 * read "with read DBI on".
 *
 * This is the SECOND and last read-side PCTL word the RUN I / RUN S full
 * live diff found differing open vs product (ours 0x12 = 18, product
 * 0x14 = 20).  It was deliberately left out of RUN AA so that build had
 * one variable.
 *
 * Is a TCL change COHERENT when PUB_MR0 is byte-identical between the two
 * firmwares?  Yes -- and the reason inverts the earlier reading of that
 * fact.  PUB_MR0 is built here as
 *     4 | ((((timings.cl - 9) >> 1) & 7) << 4) | ...
 * so bit 2 (MR0 A2) is always set and A[6:4] = (cl - 9) >> 1.  For
 * timings.cl = 18 that is A[6:4] = 4, A2 = 1, giving the DDR4 MR0 CAS
 * latency code A12,A6,A5,A4,A2 = 0b01001, and the live MR0 read on this
 * board is 0x00000844, which carries exactly that code.  Against the
 * JEDEC DDR4 MR0 CAS-latency table (an EXTERNAL table, not one in this
 * tree -- hence the caveat) code 0b01000 is CL18 and 0b01001 is CL20:
 * the encoding is non-uniform above CL16 and steps 16, 18, 20, 22.  So
 * the DRAM on this board is programmed to CL20, not to CL18 -- the "18"
 * in timings.cl is a nominal tCK number that this macro rounds up into
 * the nearest JEDEC bucket.
 *
 * That makes the disposition consistent rather than contradictory:
 *   DRAM CL (MR0)        20    20     identical, and 20 is the truth
 *   PCTL_TCL             18    20     ours is 2 SHORT of the real CL
 *   PCTL_DFITRDDATAEN    14    16     ours was 2 short -- fixed in RUN AA
 * and product's own relation DFITRDDATAEN = TCL - 4 holds exactly at
 * (20, 16), while ours held at (18, 14).  RUN AA moved one half of the
 * pair to product's value; this moves the other half, and afterwards our
 * controller agrees with the CL the DRAM is actually running.
 *
 * PREDICTION, recorded before the boot: this should NOT fix the residual.
 * The surviving defect is byte lane 0 alone (lanes 1/2/3 byte-exact on
 * every RUN AA boot), and TCL is a global controller-scheduling number
 * that every lane shares -- a per-lane defect cannot be produced by it.
 * Expected: capture stays aligned (IDLE_SLOTS none, SOURCE_DELTA +0
 * dominant), lane0 mismatch stays ~4-5 of 32 with lanes 1/2/3 at 0, and
 * MEMTEST stays FAIL.  `errors` may move within its 272-307 band if any
 * part of it was read-to-write turnaround, which TCL does feed.
 * Run anyway: it is product-exact, it is the last cheap read-side move,
 * and a negative result retires the whole PCTL read-latency set.
 *
 * Deliberately NOT touched: timings.cl itself.  Changing it would move
 * PUB_MR0 (the DRAM-side CL) and PCTL0_DFITRDDATAEN as well, which would
 * be three variables in one build.  Only the PCTL_TCL write site
 * (dram-gx.c) consumes this macro.
 */
#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR4)
/* the CL MR0 programs: 20 with read DBI on, 18 with it off */
#define PCTL0_TCL		(timings.cl + GXL_DDR4_RDBI_TCK)
#else
#define PCTL0_TCL		(timings.cl)
#endif

#define PCTL0_DFITDRAMCLKDIS	1
#define PCTL0_DFITDRAMCLKEN	1
#if defined(CONFIG_MESON_GXBB)
#define PCTL0_DFITPHYUPDTYPE1	0x200
#else
#define PCTL0_DFITPHYUPDTYPE0	16
#define PCTL0_DFITPHYUPDTYPE1	16
#define PCTL0_DFITCTRLUPDMAX	64
#define PCTL0_DFIUPDCFG		3
#endif
#define PCTL0_DFITCTRLUPDMIN	16

#define PCTL0_CMDTSTATEN	1

/*
 * DFI ODT configuration.  The field map is sourced, not guessed:
 * vendor/amlogic/boot/bl33/arch/arm/cpu/armv8/txlx/firmware/bl21/secure_apb.h,
 * the DMC_UPCTL_DFIODTCFG / DMC_UPCTL_DFIODTCFG1 comment blocks.
 *
 *   DFIODTCFG   bit 12 rank1 ODT default    bit 4 rank0 ODT default
 *               bit 11 rank1 write sel      bit 3 rank0 write sel
 *               bit 10 rank1 write nsel     bit 2 rank0 write nsel
 *               bit  9 rank1 read  sel      bit 1 rank0 read  sel
 *               bit  8 rank1 read  nsel     bit 0 rank0 read  nsel
 *   DFIODTCFG1  [27:24] ODT length, BL8 READ    [19:16] ODT length, BL8 WRITE
 *               [12:8]  ODT latency, reads      [4:0]   ODT latency, writes
 *
 * "sel" asserts that rank's ODT for its own traffic, "nsel" for the other
 * rank's.  So one rank wants write-sel only; two ranks want write-sel plus
 * write-nsel on both, which is what termination-on-the-idle-rank means.
 *
 * The values are BL2's RUNTIME values, which are what the silicon is actually
 * programmed with -- read from gxl_bl2_ddr_deep.c, the ddr_pre_init()
 * channelset ladder.  BL2 sets t_pctl0_dfiodtcfg = 8 as the default and then
 * per channelset:
 *
 *   channelset 1 (16-bit)          dfiodtcfg  8      dfiodtcfg1 0x00080000
 *   channelset 2 (RANK0, one rank) dfiodtcfg  8      dfiodtcfg1 0x06080000
 *   channelset 3 (RANK01, 2 ranks) dfiodtcfg  0xc0c  dfiodtcfg1 0x06080000
 *
 * and DDR0_PCTL_027/028 are written from those two fields.  This overrides
 * the board's static timing.c, so matching timing.c is matching the wrong
 * vendor artefact -- BL2 rewrites it before use.
 *
 * What each of our old values got wrong:
 *
 *   DFIODTCFG 0x808 for one rank set bit 11, rank1's write-sel, on a build
 *   that has no rank 1.  BL2 uses 8.
 *
 *   DFIODTCFG1 0x00060000 put the vendor's 6 in [19:16] -- ODT length for
 *   BL8 WRITE -- where BL2 puts 8, and left [27:24] (read) empty where BL2
 *   puts 6.  The 6 was in the wrong field.
 *
 *   DFIODTCFG1 ((6 << 16) | (3 << 25) | (8 << 16)) is worse than it looks:
 *   (3 << 25) and (6 << 24) are the same bits, so the intent was BL2's
 *   0x06080000, but (6 << 16) | (8 << 16) is 0xE << 16, not two fields --
 *   they collide in one 4-bit field and ask for a BL8 write ODT length of
 *   14.  Same collision in the 16-bit arm, which had no (3 << 25) at all and
 *   so asked for length 14 with no read length: 0x000E0000.
 *
 * CAVEAT, and it bounds what this change can be claimed to fix: the read
 * select bits (1 and 9) and read nsel bits (0 and 8) are ZERO in every one of
 * these DFIODTCFG values -- ours and BL2's, one rank and two.  Read ODT is
 * therefore never asserted, and the [27:24] read-length difference is very
 * probably inert.  The field that can matter is [19:16], the write length,
 * and the correctness of moving it 6 -> 8 has NOT been demonstrated here --
 * see the falsifier build, which is the only readable experiment in this
 * area today.
 */
#if defined(CONFIG_DRAM_ONE_RANK) || defined(CONFIG_DRAM_16BIT_RANK)
#define PCTL0_DFIODTCFG	0x8
/*
 * N57: every two-rank flavour takes the vendor's two-rank value 0xc0c.
 * TWO_IDENTICAL_RANKS previously got 0x8, which is the vendor's
 * PRE-channelset default -- the value ddr_pre_init() OVERWRITES once it
 * knows two ranks are present.  We were shipping the value the vendor
 * discards.  Reuses the 0xc0c spelling already here for TWO_DIFF_RANKS
 * rather than minting a second constant for the same number.
 */
#elif defined(CONFIG_DRAM_TWO_DIFF_RANKS) || \
	defined(CONFIG_DRAM_TWO_IDENTICAL_RANKS)
#define PCTL0_DFIODTCFG	0xc0c
#endif

#if defined(CONFIG_MESON_GXBB)
/* GXBB is a different BL2 and is not re-derived here.  Left as inherited. */
#define PCTL0_DFIODTCFG1	(0 | (6 << 16))
#elif defined(CONFIG_MESON_GXL)
#if defined(CONFIG_DRAM_16BIT_RANK)
/* BL2 channelset 1 */
#define PCTL0_DFIODTCFG1	(8 << 16)
#else
/*
 * N57: two-rank GXL.  Vendor timing.c two-rank set is
 *   t_pctl0_dfiodtcfg  = 0xc0c
 *   t_pctl0_dfiodtcfg1 = 0x6080000 = (8 << 16) | (3 << 25)
 * The prior value also OR'd in (6 << 16), giving 0x60e0000 -- the same
 * spurious 6<<16 term that broke DFI ODT latency on the ONE_RANK arm
 * (see the CC-V2 note above, which already retired it there).  Dropped
 * here too; ODT latency now matches the vendor bit for bit.
 */
#define PCTL0_DFIODTCFG1	((8 << 16) | (3 << 25))
#endif
#endif
/* Vendor boot-lite: (1 | (3<<4) | (1<<8) | (13<<12) | (7<<16) | (1<<24) | (3<<28)) */
#define PCTL0_DFILPCFG0		(1 | (3 << 4) | BIT(8) | (13 << 12) | \
	(7 << 16) | BIT(24) | (3 << 28))

#if defined(CONFIG_MESON_GXBB)
#define PUB_ACBDLR0		0x10
#elif defined(CONFIG_MESON_GXL)
/*
 * Address/Command Bit Delay Line: PCB trace-matching compensation.
 * Source: vendor boot-lite timing.c per board.
 *
 * DDR3 (CC):     ACBDLR0=0,    ACLCDLR=0x30, ACBDLR3=0
 * DDR4 16-bit (AC): ACBDLR0=0x3f, ACLCDLR=0x28, ACBDLR3=0x10
 * DDR4 (CC-V2):  ACBDLR0=0,    ACLCDLR=0,    ACBDLR3=0
 */
#if defined(CONFIG_DRAM_DDR3)
#define PUB_ACBDLR0		0
#define PUB_ACBDLR3		0
#define PUB_ACLCDLR		0x30
#elif defined(CONFIG_DRAM_DDR4) && defined(CONFIG_DRAM_16BIT_RANK)
/* AML-S805X-AC: 2-layer board, needs delay compensation */
#define PUB_ACBDLR0		0x3f
#define PUB_ACBDLR3		0x10
#define PUB_ACLCDLR		0x28
#elif defined(CONFIG_DRAM_DDR4)
/* AML-S905X-CC-V2: 4-layer board, no delay needed */
#define PUB_ACBDLR0		0
#define PUB_ACBDLR3		0
#define PUB_ACLCDLR		0
#else
#define PUB_ACBDLR0		0
#define PUB_ACBDLR3		0
#define PUB_ACLCDLR		0x30
#endif
#endif

/*
 * Post-train DQS-correction percentages, consumed by the CONFIG_DRAM_DQS_CORR
 * block in meson_dram_phy_setup_ranks() (dram-gxl.c): entry 0 scales ACLCDLR,
 * entry 1 scales ACBDLR0, and entries 2..5 scale the four DX lanes' LCDLR0
 * (write) / LCDLR2+LCDLR3 (read).
 *
 * Source: that board's OWN vendor boot-lite timing.c .wr_adj_per/.rd_adj_per,
 * non-LPDDR arm.  These are PER-BOARD and must not be copied between boards --
 * the same reason CONFIG_MESON_GXL_VDDEE_PWM_B carries that warning in Kconfig.
 * They lived as literals in dram-gxl.c under a comment reading "CC-V2 / P212",
 * which was accurate while AML-S905X-CC-V2 was the only board that compiled
 * them; libretech-ac_libre_defconfig makes it wrong, because the two boards'
 * READ tables genuinely differ:
 *
 *   DDR4 32-bit (CC-V2, board/aml-s905x-cc-v2/firmware/timing.c):
 *       wr {100,100,95,95,95,95}  rd {100,100,88,95,95,100}
 *   DDR4 16-bit (AC,    board/aml-s805x-ac/firmware/timing.c):
 *       wr {100,100,95,95,95,95}  rd {100,100,95,95,95,95}
 *
 * So La Frite's read-DQS delays would have been scaled to 88 % on DX0 and left
 * at 100 % on DX3 -- another board's per-lane PCB compensation -- when its own
 * vendor table is a uniform 95 %.
 *
 * DDR3 DOES reach here.  This comment used to end "DDR3 does not reach here at
 * all: no DDR3 defconfig sets CONFIG_DRAM_DQS_CORR (libretech-cc_libre does
 * not), so the DDR4 arms below are the complete set of boards that compile
 * this."  That is false as of libretech-cc_libre_defconfig enabling it: the
 * AML-S905X-CC (Le Potato) DDR3 open chain compiles the #else arm below.
 *
 * That is CORRECT for it, and the check is the same one this comment demands of
 * every board -- its own vendor table, not a neighbour's.  Le Potato's is
 * boot-lite board/aml-s905x-cc/firmware/timing.c, non-LPDDR3 (#else) arm:
 *
 *   DDR3 32-bit (CC,    board/aml-s905x-cc/firmware/timing.c):
 *       wr {100,100,95,95,95,95}  rd {100,100,88,95,95,100}
 *
 * -- byte-for-byte the CC-V2 values, so the shared #else arm states both
 * boards' vendor tables rather than lending one board's to the other.  That is
 * a coincidence of two boards' vendor data, NOT a licence to widen an arm to a
 * board whose own table has not been read: the PER-BOARD rule above is
 * unchanged, and La Frite two lines up is the standing counter-example.
 */
#define PUB_WR_ADJ_PER		{ 100, 100, 95, 95, 95, 95 }
#if defined(CONFIG_DRAM_DDR4) && defined(CONFIG_DRAM_16BIT_RANK)
/* AML-S805X-AC (La Frite) */
#define PUB_RD_ADJ_PER		{ 100, 100, 95, 95, 95, 95 }
#else
/* AML-S905X-CC-V2 (Sweet Potato) AND AML-S905X-CC (Le Potato) -- both boards'
 * own vendor timing.c carry these same values; verified per board, not assumed
 */
#define PUB_RD_ADJ_PER		{ 100, 100, 88, 95, 95, 100 }
#endif

#define LPDDR3_CA0	2
#define LPDDR3_CA1	0
#define LPDDR3_REMAP	3
#define LPDDR3_WL	1

/* PLL */
#if defined(CONFIG_MESON_GXBB)
#define DDR_PLL_CNTL1	0x69c80000
#define DDR_PLL_CNTL2	0xca463823
#define DDR_PLL_CNTL3	0xc00023
#define DDR_PLL_CNTL4	0x303500
#define DDR_PLL_CNTL5	0 /* Unused */
#elif defined(CONFIG_MESON_GXL)
#define DDR_PLL_CNTL1	0xaa203
#define DDR_PLL_CNTL2	0x2919a288
#define DDR_PLL_CNTL3	0x3e3b744
#define DDR_PLL_CNTL4	0xc0101
#define DDR_PLL_CNTL5	0xe600001e
#endif

#endif /* DRAM_SETTINGS_GX_H */
