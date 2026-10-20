/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#ifndef DRAM_TIMINGS_GX_H
#define DRAM_TIMINGS_GX_H

/*
 * DRAM timings
 * It looks like those are quite similar in regular boards based on reference
 * designs and not using counterfeit RAM chips. Those are hacked around by lowbin
 * TV box vendors to support lowbin RAM chips, however. Here, we are hardcoding
 * timings, which *will* cause issues on lowbin boards, but should be fine on other
 * boards derived from Amlogic reference designs.
 */

/*
 * TODO:
 * - Add timings for different DRAM clocks
 * - Support overwriting those if board needs different timings (how?)
 * - Other things
 */

#if defined(CONFIG_DRAM_DDR3)
#if defined(CONFIG_MESON_GXL)
/*
 * tFAW -- the four-activate window, in DRAM clocks, derived from
 * CONFIG_DRAM_CLK instead of hardcoded for one clock.  GXL only: the GXBB
 * DDR3 build keeps its literal below.
 *
 * Source: Nanya "DDR3(L) 4Gb SDRAM NT5CB(C)512M8DN / NT5CB(C)256M16DP",
 * v2.3 02/2017 (the AML-S905X-CC BOM part is NT5CB256M16DP-EK, x16, 2 KB
 * page), cross-checked against JEDEC JESD79-3F:
 *   - p.146 / JESD79-3F speed-bin tables, 2 KB page: tFAW(min) = 40 ns at
 *     DDR3-1600, 35 ns at DDR3-1866 and DDR3-2133.
 *   - p.6 "Operating frequency": a rate between two defined bins must adopt
 *     "the comparative loose specifications ... from the neighboring
 *     defined speed bins".  tFAW in ns shrinks as bins get faster, so the
 *     looser neighbour is the next-SLOWER bin.
 * So: data rate >= 1866 MT/s -> 35 ns; 1600..1865 MT/s -> 40 ns.  At the
 * shipping 912 MHz (1824 MT/s, not a defined bin) that is 40 ns = 37 clk;
 * the previous literal 33 clk (36.18 ns) met only the DDR3-1866 reading.
 * Operator decision D5, 2026-09-24.
 *
 * Bins below DDR3-1600 (45/50 ns) are not sourced in this tree, and above
 * DDR3-2133 no DDR3 bin exists, so those clocks refuse to build rather than
 * guess.
 */
#if defined(CONFIG_DRAM_DDR3_ABOVE_2133)
/*
 * Opt-in, off by default: the clock ladder past DDR3-2133.  The datasheet's
 * undefined-frequency clause (p.6) covers a rate "within the highest and the
 * lowest supporting grade", so above DDR3-2133 it gives no basis at all.  The
 * rule here is an extrapolation: every time-bound minimum holds at its
 * DDR3-2133 value (the one neighbouring bin), and the clock count is
 * recomputed from it.  tFAW is then 35 ns, which GXL_DDR3_TFAW_NS already
 * returns for every rate >= 1866 MT/s.
 */
#if CONFIG_DRAM_CLK <= 1066 || CONFIG_DRAM_CLK > 1498
#error "DRAM_DDR3_ABOVE_2133: only for 1067..1498 MHz"
#endif
#elif CONFIG_DRAM_CLK < 800 || CONFIG_DRAM_CLK > 1066
#error "DDR3 tFAW: no sourced speed bin for this CONFIG_DRAM_CLK (sourced: 800..1066 MHz = DDR3-1600..2133)"
#endif
#define GXL_DDR3_TFAW_NS(mhz)	((2 * (mhz)) >= 1866 ? 35 : 40)
/* ceil(ns * MHz / 1000): the smallest clock count that is >= tFAW(min) */
#define GXL_DDR3_TFAW_CLK(mhz)	((GXL_DDR3_TFAW_NS(mhz) * (mhz) + 999) / 1000)
_Static_assert(GXL_DDR3_TFAW_CLK(912) == 37,
	       "DDR3 tFAW at 912 MHz must be 37 clk (40 ns, DDR3-1600 neighbour)");
_Static_assert(GXL_DDR3_TFAW_CLK(933) == 33,
	       "DDR3 tFAW at 933 MHz (DDR3-1866 bin) must be 33 clk (35 ns)");

/*
 * tREFI -- the average periodic refresh interval, written to
 * DMC_REFR_CTRL2[15:8] (tREFI) by meson_dram_finalise_init().  The field
 * counts in units of DMC_REFR_CTRL2[7:0] (t100ns, the 100 ns prescaler), so
 * this value is in 100 ns ticks.  GXL only: the GXBB DDR3 build keeps its
 * literal below.
 *
 * Source: Nanya "DDR3(L) 4Gb SDRAM NT5CB(C)512M8DN / NT5CB(C)256M16DP",
 * v2.3 02/2017, p.1 density table and its NOTE 4: tREFI is 7.8 us at
 * Tc <= 85 C and 3.9 us at 85 C < Tc <= 95 C (Commercial grade, Tc 0..95 C);
 * above 85 C the device must be refreshed externally at 2x rate, with
 * Extended SRT or ASR enabled (ASR is set in MR2 by dram-settings-gx.h).
 *
 * Nothing on GXL can read the DRAM's temperature: DDR3 has no MR4, the
 * AML-S905X-CC has no thermistor, and the on-die sensor is not DRAM Tc.  So
 * the owner of this register writes the value that is legal across the
 * part's whole rated range, 3.9 us, instead of the 7.8 us that is illegal
 * above 85 C.  It is the vendor's 7.8 us setting halved: vendor 78-2 = 76
 * ticks = 7.6 us, halved = 38 ticks = 3.8 us <= 3.9 us, the same 2.6 %
 * margin under the limit.  Operator decision D14 (2026-09-24, "handle it at
 * the correct layer"); ownership argument in the GXL DRAM design doc
 * runtime-refresh-ownership.md.  A run-time relax to 76 when the board is
 * provably cool would belong to the SC, never to this table.
 */
#define GXL_DDR3_REFI_T100NS	38	/* 3.8 us: legal for Tc 0..95 C */
_Static_assert(GXL_DDR3_REFI_T100NS * 100 <= 3900,
	       "DDR3 tREFI must be <= 3.9 us (datasheet 2x refresh, 85..95 C)");
_Static_assert(GXL_DDR3_REFI_T100NS == 38,
	       "DDR3 tREFI must be 38 x 100 ns (half the vendor's 76, D14)");

/*
 * CL / tRP / tRCD and CWL -- the vendor's clock rule, not one fixed triple.
 *
 * The table below is the vendor's ddr3_13_13_13 band.  The product GXL BL2
 * does not stop there: after selecting that band it raises CL, tRP and tRCD
 * to 14 and CWL to 10 for any DDR3 clock above 932 MHz (decompile of the
 * product BL2, reversing-gxbb-bl2 gxl/bl2.c:2536-2539):
 *
 *   if (ddrtype_ == 0) {            DDR3
 *     if (932 < ddrs.ddr_clk) {
 *       cfg_ddr_cl = 14; cfg_ddr_rp = 14; cfg_ddr_rcd = 14; cfg_ddr_cwl = 10;
 *
 * CWL 10 is also the JEDEC CWL for tCK < 1.07 ns (DDR3-2133), which is where
 * every clock above 933 MHz sits; CWL 9 stays correct for 1.07..1.25 ns.
 * Every consumer derives from these fields (MR0 CL code, MR2 CWL code,
 * PCTL_TCL / TCWL / TRP / TRCD, PUB_DTPR0/5, the DFI read-enable and
 * write-latency words), so the rule moves them together.
 *
 * At the shipping 912 MHz the rule selects 13 / 9, the values this table has
 * always carried, so the shipped image does not change.
 */
#define GXL_DDR3_CL_RP_RCD_AT(mhz)	((mhz) > 932 ? 14 : 13)
#define GXL_DDR3_CWL_AT(mhz)		((mhz) > 932 ? 10 : 9)
#define GXL_DDR3_CL_RP_RCD	GXL_DDR3_CL_RP_RCD_AT(CONFIG_DRAM_CLK)
#define GXL_DDR3_CWL		GXL_DDR3_CWL_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_CL_RP_RCD	GXL_DDR3_CL_RP_RCD
#define GX_DDR3_CWL		GXL_DDR3_CWL

/*
 * Fields whose minimum is a TIME -- floored at the clock-derived count.
 *
 * Every field below is held in DRAM clocks, so a fixed count shrinks in ns as
 * CONFIG_DRAM_CLK rises.  Each one's minimum is stated in ns (with an nCK
 * floor the table literal already meets), and that ns minimum is the SAME in
 * every speed bin from DDR3-1600 to DDR3-2133, so no bin reading is involved.
 * Source: Nanya "DDR3(L) 4Gb SDRAM NT5CB(C)512M8DN / NT5CB(C)256M16DP", v2.3
 * 02/2017 (the AML-S905X-CC BOM part is NT5CB256M16DP-EK):
 *
 *   tRTP, tWTR   max(4tCK, 7.5 ns)   p.146
 *   tWR          15 ns               p.146   (the controller's PCTL_TWR; the
 *                                             MR0 WR code clamps at 16, see
 *                                             PUB_MR0_WR_CODE)
 *   tRFC         260 ns (4 Gb)       p.1 density table, NOTE 5
 *   tXP          max(3tCK, 6 ns)     p.147
 *   tCKE         max(3tCK, 5 ns)     p.147
 *   tXPDLL       max(10tCK, 24 ns)   p.147
 *
 * Each field is max(literal, ceil(ns x MHz / 1000)): the table's literal
 * where it is already legal -- which includes the shipping 912 MHz, so the
 * shipped image does not change -- and the smallest legal count where it is
 * not.  Without the floor the literals go illegal inside the range this file
 * builds (800..1066): tRTP/tWTR 7 above 933 MHz, tXPDLL 23 above 958, tCKE 5
 * above 1000.  Operator decision D5 (2026-09-24): "we should scale it if
 * clock-dependent adjustments are more spec compliant".
 */
/* ceil(ns100 / 100 * MHz / 1000): the smallest clock count >= the time */
#define GXL_DDR3_NS100_CLK(ns100, mhz)	(((ns100) * (mhz) + 99999) / 100000)
#define GXL_DDR3_FLOOR_AT(lit, ns100, mhz)				\
	(GXL_DDR3_NS100_CLK(ns100, mhz) > (lit) ?			\
	 GXL_DDR3_NS100_CLK(ns100, mhz) : (lit))
/*
 * Each clock-dependent field is ONE expression of the clock, _AT(mhz): the
 * table below instantiates it at CONFIG_DRAM_CLK, and gxl_ddr3_timings_at()
 * re-instantiates it when dram_init()'s ladder falls back to a lower clock.
 */
#define GXL_DDR3_RTP_AT(m)	GXL_DDR3_FLOOR_AT(0x7, 750, m)	/* 7.5 ns */
#define GXL_DDR3_WTR_AT(m)	GXL_DDR3_FLOOR_AT(0x7, 750, m)	/* 7.5 ns */
#define GXL_DDR3_WR_AT(m)	GXL_DDR3_FLOOR_AT(0x10, 1500, m)	/* 15 ns */
#define GXL_DDR3_RFC_AT(m)	GXL_DDR3_FLOOR_AT(0x118, 26000, m)	/* 260 ns */
#define GXL_DDR3_XP_AT(m)	GXL_DDR3_FLOOR_AT(0x7, 600, m)	/* 6 ns */
#define GXL_DDR3_CKE_AT(m)	GXL_DDR3_FLOOR_AT(0x5, 500, m)	/* 5 ns */
#define GXL_DDR3_XPDLL_AT(m)	GXL_DDR3_FLOOR_AT(0x17, 2400, m)	/* 24 ns */
#define GX_DDR3_RTP		GXL_DDR3_RTP_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_WTR		GXL_DDR3_WTR_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_WR		GXL_DDR3_WR_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_RFC		GXL_DDR3_RFC_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_XP		GXL_DDR3_XP_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_CKE		GXL_DDR3_CKE_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_XPDLL		GXL_DDR3_XPDLL_AT(CONFIG_DRAM_CLK)
/* The shipping clock keeps every literal: the image at 912 MHz is unchanged. */
_Static_assert(GXL_DDR3_NS100_CLK(750, 912) <= 0x7 &&
	       GXL_DDR3_NS100_CLK(1500, 912) <= 0x10 &&
	       GXL_DDR3_NS100_CLK(26000, 912) <= 0x118 &&
	       GXL_DDR3_NS100_CLK(600, 912) <= 0x7 &&
	       GXL_DDR3_NS100_CLK(500, 912) <= 0x5 &&
	       GXL_DDR3_NS100_CLK(2400, 912) <= 0x17,
	       "DDR3 time floors must leave every 912 MHz literal unchanged");
/* ... and they bite where the literal goes short: 936 -> tRTP/tWTR 8 clk. */
_Static_assert(GXL_DDR3_NS100_CLK(750, 936) == 8,
	       "DDR3 tRTP/tWTR at 936 MHz must be 8 clk (7.5 ns)");
#if defined(CONFIG_DRAM_DDR3_ABOVE_2133)
/*
 * The rest of the time-bound fields, at their DDR3-2133 minimums (datasheet
 * v2.3 p.2 core timing, 14-14-14 column; p.146-147).  tRP/tRCD split from
 * CL here: CL cannot follow (MR0 CL 15 and 16 are Reserved, JESD79-3F
 * Figure 9), so it stays at the vendor's 14.
 */
#define GXL_DDR3_RP_RCD_AT(m)	GXL_DDR3_FLOOR_AT(GXL_DDR3_CL_RP_RCD_AT(m), 1309, m)
#define GXL_DDR3_RAS_AT(m)	GXL_DDR3_FLOOR_AT(0x25, 3300, m)	/* 33 ns */
#define GXL_DDR3_RC_AT(m)	GXL_DDR3_FLOOR_AT(0x34, 4609, m)	/* 46.09 ns */
#define GXL_DDR3_RRD_AT(m)	GXL_DDR3_FLOOR_AT(0x7, 600, m)	/* max(4tCK, 6 ns) */
#else
/* Without DRAM_DDR3_ABOVE_2133: the table's literals, tRP/tRCD = CL. */
#define GXL_DDR3_RP_RCD_AT(m)	GXL_DDR3_CL_RP_RCD_AT(m)
#define GXL_DDR3_RAS_AT(m)	0x25
#define GXL_DDR3_RC_AT(m)	0x34
#define GXL_DDR3_RRD_AT(m)	0x7
#endif
#define GX_DDR3_RP_RCD		GXL_DDR3_RP_RCD_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_RAS		GXL_DDR3_RAS_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_RC		GXL_DDR3_RC_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_RRD		GXL_DDR3_RRD_AT(CONFIG_DRAM_CLK)
#if defined(CONFIG_DRAM_DDR3_ABOVE_2133) || defined(CONFIG_DRAM_DDR3_N831_FLOORS)
/*
 * tMOD, tZQCS and tZQoper at their DDR3 minimums (Nanya v2.3 p.146-147).
 * The vendor literals 12 / 64 / 136 are short at the shipping 912 MHz
 * (14 / 73 / 292 there). On by default (gxl:N831); say n to get the literals.
 */
#define GXL_DDR3_MOD_AT(m)	GXL_DDR3_FLOOR_AT(0xc, 1500, m)	/* max(12tCK, 15 ns) */
#define GXL_DDR3_ZQCS_AT(m)	GXL_DDR3_FLOOR_AT(0x40, 8000, m)	/* max(64tCK, 80 ns) */
#define GXL_DDR3_ZQCL_AT(m)	GXL_DDR3_FLOOR_AT(0x100, 32000, m)	/* max(256tCK, 320 ns) */
_Static_assert(GXL_DDR3_NS100_CLK(1500, 912) == 14 &&
	       GXL_DDR3_NS100_CLK(8000, 912) == 73 &&
	       GXL_DDR3_NS100_CLK(32000, 912) == 292,
	       "at 912 MHz tMOD/tZQCS/tZQoper floor to 14/73/292 clk");
#else
/* Without DRAM_DDR3_N831_FLOORS / ABOVE_2133: the vendor literals. */
#define GXL_DDR3_MOD_AT(m)	0xc
#define GXL_DDR3_ZQCS_AT(m)	0x40
#define GXL_DDR3_ZQCL_AT(m)	0x88
#endif
#define GX_DDR3_MOD		GXL_DDR3_MOD_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_ZQCS		GXL_DDR3_ZQCS_AT(CONFIG_DRAM_CLK)
#define GX_DDR3_ZQCL		GXL_DDR3_ZQCL_AT(CONFIG_DRAM_CLK)
#else
/* GXBB DDR3 keeps its fixed 13-13-13 / CWL 9 and its literal timings. */
#define GX_DDR3_CL_RP_RCD	0xd
#define GX_DDR3_CWL		0x9
#define GX_DDR3_RTP		0x7
#define GX_DDR3_WTR		0x7
#define GX_DDR3_WR		0x10
#define GX_DDR3_RFC		0x118
#define GX_DDR3_XP		0x7
#define GX_DDR3_CKE		0x5
#define GX_DDR3_XPDLL		0x17
#endif
#if !defined(GX_DDR3_RP_RCD)
/* Without DRAM_DDR3_ABOVE_2133: the table's literals. */
#define GX_DDR3_RP_RCD		GX_DDR3_CL_RP_RCD
#define GX_DDR3_RAS		0x25
#define GX_DDR3_RC		0x34
#define GX_DDR3_RRD		0x7
#endif
#if !defined(GX_DDR3_MOD)
/* Without DRAM_DDR3_N831_FLOORS / ABOVE_2133: the vendor literals. */
#define GX_DDR3_MOD		0xc
#define GX_DDR3_ZQCS		0x40
#define GX_DDR3_ZQCL		0x88
#endif

/* DDR3: 912 MHz (CL/tRP/tRCD/CWL follow the vendor's clock rule on GXL) */
struct meson_gx_dram_timings timings = {
	.drv        = 0,
	.odt        = 2,

	/* Timings */
	.rtp        = GX_DDR3_RTP, /* 7; >= 7.5 ns on GXL */
	.wtr        = GX_DDR3_WTR, /* 7; >= 7.5 ns on GXL */
	.rp         = GX_DDR3_RP_RCD, /* 13; 14 above 932 MHz on GXL */
	.rcd        = GX_DDR3_RP_RCD, /* 13; 14 above 932 MHz on GXL */
	.ras        = GX_DDR3_RAS, /* 37; ABOVE_2133: >= 33 ns */
	.rrd        = GX_DDR3_RRD, /* 7; ABOVE_2133: >= 6 ns */
	.rc         = GX_DDR3_RC, /* 52; ABOVE_2133: >= 46.09 ns */
	.mrd        = 0x6, /* Should be < 8 */
	.mod        = GX_DDR3_MOD, /* vendor: 12 (tMOD); N831_FLOORS: >= 15 ns */
#if defined(CONFIG_MESON_GXL)
	.faw        = GXL_DDR3_TFAW_CLK(CONFIG_DRAM_CLK), /* 37 @ 912 MHz */
#else
	.faw        = 0x21,
#endif
	.wlmrd      = 0x28,
	.wlo        = 0x7,
	.rfc        = GX_DDR3_RFC, /* 280; >= 260 ns on GXL */
	.xp         = GX_DDR3_XP, /* 7; >= 6 ns on GXL */
	.xs         = 0x200,
	.dllk       = 0x200,
	.cke        = GX_DDR3_CKE, /* 5; >= 5 ns on GXL */
	.rtodt      = 0x0,
	.rtw        = 0x7,
#if defined(CONFIG_MESON_GXL)
	.refi       = GXL_DDR3_REFI_T100NS, /* 3.8 us, legal to Tc 95 C */
#else
	.refi       = 0x4c, /* vendor: 78-2=76 */
#endif
	.refi_mddr3 = 0x4,
	.cl         = GX_DDR3_CL_RP_RCD, /* 13; 14 above 932 MHz on GXL */
	.wr         = GX_DDR3_WR, /* 16; >= 15 ns on GXL */
	.cwl        = GX_DDR3_CWL, /* 9; 10 above 932 MHz on GXL */
	.al         = 0x0,
	.dqs        = 0x17,
	.cksre      = 0xf,
	.cksrx      = 0xf,
	.zqcs       = GX_DDR3_ZQCS, /* 64; N831_FLOORS: >= 80 ns */
	.xpdll      = GX_DDR3_XPDLL, /* 23; >= 24 ns on GXL */
	.exsr       = 0x200, /* Should be < 0x3ff */
	.zqcl       = GX_DDR3_ZQCL, /* 136; N831_FLOORS: >= max(256, 320 ns) */
	.zqcsi      = 0x3e8,
	.rpab       = 0x0,
	.rppb       = 0x0,
	.tdqsck     = 0x0,
	.tdqsckmax  = 0x0,
	.tckesr     = 0x0,
	.tdpd       = 0x0,
	.taond_aofd = 0x0,
	.tccdl      = 0, /* Unused on GXBB */
};

#if defined(CONFIG_MESON_GXL)
/*
 * Re-instantiate every clock-dependent field at @mhz.  dram_init()'s clock
 * ladder calls this before each attempt, so a fallback clock trains with the
 * values the rules above give at THAT clock -- never the CONFIG_DRAM_CLK
 * ones, which can be illegal there (CWL 10 above 932 MHz is not a legal CWL
 * below it).  At CONFIG_DRAM_CLK it rewrites the table's own values.
 */
static void gxl_ddr3_timings_at(u32 mhz)
{
	timings.rtp = GXL_DDR3_RTP_AT(mhz);
	timings.wtr = GXL_DDR3_WTR_AT(mhz);
	timings.rp = GXL_DDR3_RP_RCD_AT(mhz);
	timings.rcd = GXL_DDR3_RP_RCD_AT(mhz);
	timings.ras = GXL_DDR3_RAS_AT(mhz);
	timings.rrd = GXL_DDR3_RRD_AT(mhz);
	timings.rc = GXL_DDR3_RC_AT(mhz);
	timings.mod = GXL_DDR3_MOD_AT(mhz);
	timings.faw = GXL_DDR3_TFAW_CLK(mhz);
	timings.rfc = GXL_DDR3_RFC_AT(mhz);
	timings.xp = GXL_DDR3_XP_AT(mhz);
	timings.cke = GXL_DDR3_CKE_AT(mhz);
	timings.cl = GXL_DDR3_CL_RP_RCD_AT(mhz);
	timings.wr = GXL_DDR3_WR_AT(mhz);
	timings.cwl = GXL_DDR3_CWL_AT(mhz);
	timings.zqcs = GXL_DDR3_ZQCS_AT(mhz);
	timings.xpdll = GXL_DDR3_XPDLL_AT(mhz);
	timings.zqcl = GXL_DDR3_ZQCL_AT(mhz);
}
#endif

#elif defined(CONFIG_DRAM_DDR4)
/*
 * DDR4 @ 1080 MHz — vendor CONFIG_DDR_TIMMING_DDR4_2400 (CC-V2 timing.c).
 * Sticky residual QSGATE (GTR DGSL=0x1f) with cl=16; align cl/rp/rcd/rrd/rc/rtw.
 */

/*
 * Power-down entry / exit floors.  The vendor's constants are sized for the
 * 912 MHz reference operating point; at the 1080 MHz this board ships, tCKE
 * and tXPDLL are already below their JEDEC minima, and tXP falls below its
 * minimum at 1200 MHz.  Floors, and the cycle counts that satisfy them at
 * BOTH clocks (ceil(floor_ns * f_GHz), also satisfying the nCK half):
 *
 *            floor form                 @1080     @1200   chosen
 *   tCKE     max(3 nCK, 5.0 ns)             6         6      6
 *   tXP      max(4 nCK, 6.0 ns)             7         8      8
 *   tXPDLL   max(10 nCK, 24.0 ns)          26        29     29
 *   tZQoper  512 nCK (nCK only)           512       512    512
 *
 * The nCK half of each max() is slack at both clocks (3/4/10 nCK are
 * 2.78/3.70/9.26 ns at 1080), so the first three are time-denominated and a
 * larger cycle count is the correct remedy.  tZQoper is different in kind:
 * it is nCK-denominated with no time half at all, so it is frequency-
 * independent and .zqcl = 256 (the DDR3 tZQoper) is out of spec at 912,
 * 1080 and 1200 alike.
 *
 * Floors cited from DDR4 controller code in this tree:
 *   tCKE     max(3 nCK, SPEED_BIN_TPD = 5000 ps)
 *            drivers/ddr/marvell/a38x/ddr3_training.c:1710
 *            drivers/ddr/marvell/a38x/mv_ddr4_training_db.c:454-456
 *   tXPDLL   max(10 nCK, SPEED_BIN_TXPDLL = 24000 ps)
 *            drivers/ddr/marvell/a38x/ddr3_training.c:1713
 *            drivers/ddr/marvell/a38x/mv_ddr4_training_db.c:519-521
 *   tXP      max(4 nCK, 6 ns), stated verbatim
 *            drivers/ddr/fsl/ctrl_regs.c:321-322
 *   tZQoper  512 nCK for DDR4 (256 for DDR3)
 *            drivers/ddr/fsl/ctrl_regs.c:2181-2188
 *
 * Destination field widths, from the PCTL field-mask table carried by the
 * DRAM fault-class instrument (dram-fault-class.c on the topic/gxl-n313-arms
 * branch; not part of this tree) -- cke 0xf, xp 0x1f, xpdll 0x3f.  All three
 * fit; .xp is the only one that grows a bit (7 -> 8) and it has five.
 *
 * PCTL_TZQCL is absent from that table, so its width is not obtainable
 * from source.  It was measured on silicon instead (round N316, both
 * boots): writing 0xffffffff and reading back gives 0x000003ff, i.e. a
 * 10-bit field, max 1023.  512 fits with room and the feared silent
 * truncation to 0 cannot happen, so .zqcl = 512 rides with the other three.
 *
 * The corrected set is selected PER DRAM POPULATION, on the same symbol
 * dram-settings-gx.h already splits the DDR4 PHY settings on:
 *
 *   CONFIG_DRAM_16BIT_RANK unset -- two x16 devices on a 32-bit bus
 *     (AML-S905X-CC-V2).  The four JEDEC floors, boot-witnessed on CC-V2
 *     unit 1 in rounds n319/n320 (7 evaluable launches): boots, trains,
 *     memtests clean, and the eye figures (trained, W_Max) sit inside the
 *     control arm's own boot-to-boot range.
 *
 *   CONFIG_DRAM_16BIT_RANK set -- one x16 device on a 16-bit bus
 *     (AML-S805X-AC).  The inherited vendor constants are RETAINED, not
 *     endorsed: no unit of that population has booted the corrected set
 *     (the only AC unit is BootROM-dead, N240), and this tree does not
 *     change a DRAM timing on a population it cannot witness.  When an AC
 *     unit boots, the floors above apply to it unchanged -- they are
 *     DDR4-generic and speed-bin-independent for tCKE/tXPDLL -- and this
 *     arm should collapse into the other.
 */
#if defined(CONFIG_DRAM_16BIT_RANK)
#define GXL_DDR4_TCKE	5
#define GXL_DDR4_TXP	7
#define GXL_DDR4_TXPDLL	23
#define GXL_DDR4_TZQCL	256
#else
#define GXL_DDR4_TCKE	6
#define GXL_DDR4_TXP	8
#define GXL_DDR4_TXPDLL	29
#define GXL_DDR4_TZQCL	512
#endif

/*
 * DDR4 tREFI, in 100 ns ticks -- the DDR3 argument above (D14) applied to
 * DDR4, whose 7.8 us is likewise legal only to Tc 85 C and 3.9 us to 95 C.
 * Nothing on GXL reads DDR4 Tc at run time (the MPR page-2 readout turns
 * ordinary reads into MPR reads while it is open), so by policy the die
 * temperature stands in, and the open SC's refresh governor relaxes this
 * boot value to 76 while the die is cool (operator 2026-10-14: adaptive on
 * both DDR4 boards).
 *
 *   two x16 devices (AML-S905X-CC-V2): boot at 38 = 3.8 us, legal to 95 C.
 *   one x16 device (AML-S805X-AC): stays at the vendor's 76 = 7.6 us until
 *     its SC runs the governor -- the same no-unwitnessed-change rule as the
 *     floors above, and the operator's order for that board.
 */
#if defined(CONFIG_DRAM_16BIT_RANK)
#define GXL_DDR4_REFI_T100NS	76	/* 7.6 us: legal to Tc 85 C only */
#else
#define GXL_DDR4_REFI_T100NS	38	/* 3.8 us: legal for Tc 0..95 C */
#endif

/*
 * tCCD_L, written to both PCTL_TCCD and the DRAM's MR6[12:10].  The vendor
 * BL2 on AML-S805X-AC-V2 leaves 4 in both (PCTL_TCCD 0x00040004, MR6 0x0d);
 * at 6 the board's back-to-back stores land shifted within each 32-byte block.
 */
#if defined(CONFIG_DRAM_16BIT_RANK)
#define GXL_DDR4_TCCDL		4
#else
#define GXL_DDR4_TCCDL		6
#endif
#if !defined(CONFIG_DRAM_16BIT_RANK)
_Static_assert(GXL_DDR4_REFI_T100NS * 100 <= 3900,
	       "CC-V2 DDR4 tREFI must be <= 3.9 us (2x refresh, 85..95 C)");
#endif

struct meson_gx_dram_timings timings = {
	.drv        = 0, /* DDR4_DRV_34OHM */
	.odt        = 1, /* DDR4_ODT_60OHM */

	/* Timings */
	.rtp        = 9,
	.wtr        = 9,
	.rp         = 18, /* vendor 15*1.2 */
	.rcd        = 18, /* vendor 15*1.2 */
	.ras        = 35 * 1.2,
	.rrd        = 8,  /* vendor */
	.rc         = 60, /* vendor 50*1.2 */
	.mrd        = 8,
	.mod        = 24,
	.faw        = 35 * 1.2,
	.rfc        = 350 * 1.2,
	.wlmrd      = 40,
	.wlo        = 9.5 * 1.2,
	.xs         = 512,
	.xp         = GXL_DDR4_TXP,
	.cke        = GXL_DDR4_TCKE,
	.dllk       = 1024,
	.rtodt      = 0,
	.rtw        = 9,  /* vendor */
	.refi       = GXL_DDR4_REFI_T100NS, /* 38 on CC-V2, 76 on AC: above */
	.refi_mddr3 = 4,
	.cl         = 18, /* vendor 15*1.2 */
	.wr         = 18, /* vendor 15*1.2 */
	.cwl        = 12,
	.al         = 0,
	.exsr       = 1024,
	.dqs        = 9,
	.cksre      = 15,
	.cksrx      = 15,
	.zqcs       = 128,
	.zqcl       = GXL_DDR4_TZQCL,
	.xpdll      = GXL_DDR4_TXPDLL,
	.zqcsi      = 1000,
	.tccdl      = GXL_DDR4_TCCDL,
};
#endif
#endif
