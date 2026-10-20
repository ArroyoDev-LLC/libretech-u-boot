// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Portions Copyright (C) 2015, Amlogic, Inc. All rights reserved.
 * Copyright (C) 2023-2025, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#include <init.h>
#include <asm/unaligned.h>
#include <linux/libfdt.h>
#include <config.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/dram-gx.h>
#include <asm/arch/gx.h>
#include <asm/arch/clock-gx.h>
#include <asm/arch/dram-settings-gx.h>
#include <asm/arch/gx-handoff.h>
#include <asm/arch/dram-timings-gx.h>
#include <linux/delay.h>
#include <linux/build_bug.h>
#include <cpu_func.h>
#include <asm/system.h>
#include "ddr-detect-gen.h"

/*
 * Meson GX common shared DRAM init code
 *
 * See dram-gxbb.c and dram-gxl.c for gxbb/gxl-specific code
 */

/*
 * meson_dram_pll_init() below writes AM_DDR_PLL_CNTL0 only from inside a
 * per-SoC window -- GXL [399,799] and [800,1498], GXBB [375,749] and
 * [750,1449].  Outside those the if/else-if chain falls through and
 * CNTL0 is NEVER WRITTEN, which is silent in all three of the ways that
 * matter:
 *
 *   - the PLL-lock retry loop cannot catch it.  It spins on
 *     AM_DDR_PLL_STS bit 31, and the PLL locks on whatever CNTL1..5
 *     already command, so "DDR PLL LOCK FAIL" never fires;
 *   - the stage then PRINTS the value it is not running --
 *     printf("DRAM clock: %d MHz\n", CONFIG_DRAM_CLK) is unconditional,
 *     so the console asserts the COMMANDED number;
 *   - the hole is at BOTH ends of each window, not just the top.
 *
 * So a wrong CONFIG_DRAM_CLK yields a booting image that reports a
 * frequency it is not running.  Bound it at build time instead.
 *
 * Why not a Kconfig "range".  Not because it cannot be written per-SoC
 * -- it can (arch/sandbox/Kconfig does exactly this shape) -- but
 * because it does not REFUSE.  sym_validate_range() in
 * scripts/kconfig/symbol.c CLAMPS an out-of-range int to the nearest
 * bound and rewrites the value with no error and no warning, so
 * CONFIG_DRAM_CLK=1500 would silently become 1498 and the operator
 * would get a board running a clock nobody asked for -- the same
 * silent substitution this guard exists to kill, moved one layer
 * earlier.  static_assert refuses.  It is also the tree's existing
 * idiom for exactly this class (asm/arch/dram-timings-gx.h,
 * asm/arch/gx-handoff.h) and costs zero image bytes.
 *
 * Per-SoC, because the two windows differ: a single [399,1498] bound
 * would leave GXBB's 1450..1498 unguarded, where CNTL0 is not written.
 *
 * NOT covered here: CONFIG_MESON_GXM selects MESON_GX (so this file is
 * built) but neither MESON_GXL nor MESON_GXBB, and the chain above has
 * no GXM arm at all -- on GXM, CNTL0 is never written at ANY clock.
 * That is a separate defect and is deliberately not converted into a
 * build failure here.
 */
#define GX_DRAM_CLK_MIN_GXL	399
#define GX_DRAM_CLK_MAX_GXL	1498
#define GX_DRAM_CLK_MIN_GXBB	375
#define GX_DRAM_CLK_MAX_GXBB	1449

#if defined(CONFIG_MESON_GXL)
static_assert(CONFIG_DRAM_CLK >= GX_DRAM_CLK_MIN_GXL &&
	      CONFIG_DRAM_CLK <= GX_DRAM_CLK_MAX_GXL,
	      "CONFIG_DRAM_CLK is outside the GXL window [399,1498]: AM_DDR_PLL_CNTL0 would never be written and the board would run an unknown clock while printing this one");
#elif defined(CONFIG_MESON_GXBB)
static_assert(CONFIG_DRAM_CLK >= GX_DRAM_CLK_MIN_GXBB &&
	      CONFIG_DRAM_CLK <= GX_DRAM_CLK_MAX_GXBB,
	      "CONFIG_DRAM_CLK is outside the GXBB window [375,1449]: AM_DDR_PLL_CNTL0 would never be written and the board would run an unknown clock while printing this one");
#endif

/*
 * Returns 0, or -ETIMEDOUT when the DDR PLL does not lock.  It used to return
 * void after printing "DDR PLL LOCK FAIL", and the attempt went on to train a
 * PHY whose clock (DDR_CLK_CNTL) was never written -- an access to the
 * unclocked PUB may stall the bus, so the ladder never reached its next,
 * lower clock rung (GXL DDR3 FMA round F1 FA4, claude amlogic/software/boot/
 * gxl/audit/2026-10-19-gxl-ddr3-boot-chain-failure-modes.md).
 */
static int meson_dram_pll_init(void)
{
	setbits_32(AM_ANALOG_TOP_REG1, 1);
	setbits_32(GX_HIU_BASE + HHI_MPLL_CNTL5, 1);

	clrbits_32(AM_DDR_PLL_CNTL4, BIT(12));
	setbits_32(AM_DDR_PLL_CNTL4, BIT(12));

	udelay(10);

	int pll_retries = 100;
	do {
		if (IS_ENABLED(CONFIG_MESON_GXBB))
			writel(1 << 29, AM_DDR_PLL_CNTL0);
		writel(DDR_PLL_CNTL1, AM_DDR_PLL_CNTL1);
		writel(DDR_PLL_CNTL2, AM_DDR_PLL_CNTL2);
		writel(DDR_PLL_CNTL3, AM_DDR_PLL_CNTL3);
		writel(DDR_PLL_CNTL4, AM_DDR_PLL_CNTL4);
		if (IS_ENABLED(CONFIG_MESON_GXBB)) {
			if (CONFIG_DRAM_CLK >= 375 && CONFIG_DRAM_CLK <= 749)
				writel((1 << 29) | ((2 << 16) | (1 << 9) |
					(((CONFIG_DRAM_CLK / 6) * 6) / 12)), AM_DDR_PLL_CNTL0);
			else if (CONFIG_DRAM_CLK >= 750 && CONFIG_DRAM_CLK <= 1449)
				writel((1 << 29) | ((1 << 16) | (1 << 9) |
					(((CONFIG_DRAM_CLK / 12) * 12) / 24)), AM_DDR_PLL_CNTL0);
			clrbits_32(AM_DDR_PLL_CNTL0, 1 << 29);
		} else if (IS_ENABLED(CONFIG_MESON_GXL)) {
			/* meson_dram_clk: the ladder's clock for this attempt */
			writel(DDR_PLL_CNTL5, AM_DDR_PLL_CNTL5);
			if (meson_dram_clk >= 399 && meson_dram_clk <= 799)
				writel(((1 << 16) | ((1 << 2) | 1) |
					((meson_dram_clk / 12) << 4)) |
					((1 << 31) | (1 << 29) | (1 << 28)), AM_DDR_PLL_CNTL0);
			else if (meson_dram_clk >= 800 && meson_dram_clk <= 1498)
				writel(((1 << 16) | (1 << 2) |
					((meson_dram_clk / 24) << 4)) |
					((1 << 31) | (1 << 29) | (1 << 28)), AM_DDR_PLL_CNTL0);
		}
		udelay(200);
	} while (!((readl(AM_DDR_PLL_STS) >> 0x1F) & 1) && --pll_retries > 0);

	if (pll_retries <= 0) {
		puts("DDR PLL LOCK FAIL\n");
		return -ETIMEDOUT;
	}

	if (IS_ENABLED(CONFIG_MESON_GXBB)) {
		writel(DDR_CLK_CNTL_CLKGEN_SOFTRESET |
			DDR_CLK_CNTL_PHY_CLK_ENABLE |
			DDR_CLK_CNTL_DDRPLL_ENABLE, DDR_CLK_CNTL);
	} else if (IS_ENABLED(CONFIG_MESON_GXL)) {
		writel(DDR_CLK_CNTL_CLKGEN_SOFTRESET |
			DDR_CLK_CNTL_PHY_CLK_ENABLE |
			DDR_CLK_CNTL_DDRPLL_ENABLE |
			0xa005 /* unknown */, DDR_CLK_CNTL);
	}

	printf("DRAM clock: %d MHz\n", meson_dram_clk);
	return 0;
}

/*
 * ===================================================================
 * N27 -- WHAT A FAILED ATTEMPT LEAVES BEHIND, AND WHAT A RE-ENTRY OWES
 * ===================================================================
 *
 * This function is the only thing dram_init()'s retry ladder runs between two
 * attempts that could plausibly return the controller and PHY to a known
 * state, and until 2026-08-29 it could not: every write here RELEASES a reset
 * and none ASSERTS one.  On a second entry the resets are already released, so
 * all four writes are no-ops and attempt N+1 begins on whatever wreckage
 * attempt N stopped in.  That is not a retry; it is a second attempt on top of
 * the first one's remains, and it is why the observed ladder lands somewhere
 * WORSE than where it started (n27a/n27c, s7-nyc, 2026-08-29):
 *
 *   attempt 1  PCTL kept out of ACCESS -> `PCTL ACCESS timeout! STAT=7001`
 *   attempt 2  a DIFFERENT site, 740 ms later: PGSR0=800000fe, `PGSR0 timeout!`
 *              -- IDONE clear, done bits 8..11 never set, i.e. the PUB's PIR
 *              sequence no longer completes on a PHY nothing reset
 *   attempt 3  wedges with no further output at all
 *
 * THE VENDOR ANSWERS IT DIRECTLY, and this is transcribed from the vendor's
 * own GXL BL2 rather than inferred.  ~/git/amlogic/reversing-gxbb-bl2/gxl/
 * bl2.c:1753-1766 is the FIRST thing inside ddr_init_pctl()'s retry body -- a
 * `do { ... } while (true)` opened at :1752 and closed at :2332, so the vendor
 * re-executes exactly this block on every attempt:
 *
 *     DMC_SOFT_RST(0xc8838004)  = 0;            <- ASSERT
 *     DMC_SOFT_RST1(0xc8838008) = 0;            <- ASSERT
 *     udelay(10);
 *     DMC_SOFT_RST  = 0xffffffff;               <- release
 *     DMC_SOFT_RST1 = 0xffffffff;
 *     udelay(10);
 *     DMC_PCTL_LP_CTRL(0xc8838118) = 0;
 *     DDR0_SOFT_RESET(0xc8837020) = 0;          <- ASSERT
 *     udelay(10);
 *     DDR0_SOFT_RESET = 0xf;                    <- release
 *     udelay(10);
 *
 * The resets are ACTIVE-LOW, which is why writing all-ones a second time can
 * never reset anything.  The vendor's ladder is CONFIG_DDR_INIT_RETRY_TOTAL
 * (10) / CONFIG_DDR_PCTL_RETRY_TOTAL (100) -- arch-gxl/ddr_define.h:57-58 in
 * the A311D-S905D3_linux5.15 BSP, matching the decompiled >10 / >100 bounds --
 * and its give-up is a system reset, not a fall-through.
 *
 * GXL ONLY, and that is not caution, it is what the vendor does.  GXBB's BL2
 * (reversing-gxbb-bl2/gxbb/bl2.c:4113-4115) writes ONLY the release values and
 * never touches 0xc8837020 at all.  The shape this function had is the GXBB
 * shape, applied to both SoCs; giving GXL the assert restores the split rather
 * than inventing one.  GXBB is out of scope for this project's boards and its
 * arm is left byte-for-byte alone.
 *
 * SCOPE.  This makes a retry re-enter from reset.  It does NOT claim the
 * ladder now recovers -- see dram_init(), and the pre-registration in
 * configs/dram-pctl-noaccess-1.config.  That caution was right: measured
 * 2026-08-30, recovery survives this assert being turned off, so the bounded
 * waits are what recover and this is not credited with it.  Detail at the
 * IS_ENABLED below.
 */
void meson_dram_phy_prepare(void)
{
	/*
	 * Vendor ddr_init_pctl: DMC_SOFT_RST HOLD (0) then RELEASE (0xffffffff)
	 * with 10 us between.  Until N27 the assert was DDR4-scoped ("never
	 * measured on a DDR3 board"); the GXL vendor BL2 does it for every
	 * profile, and a DDR3 re-entry that cannot assert cannot retry.
	 */
	/*
	 * The second IS_ENABLED is the N27(b) control and defaults to y, so the
	 * shipping expression folds to exactly what it was.
	 *
	 * MEASURED 2026-08-30, and it is NOT what the SCOPE note below assumes:
	 * with the recovery instrument armed and this assert turned OFF, the
	 * board still comes up `try 2` -- identical to the arm that keeps it.
	 * So on the production lineage the BOUNDED WAITS are what make the
	 * ladder recover, and re-entry from reset is not load-bearing for this
	 * failure mode.  A control arm with the assert off but no induced
	 * failure boots normally on try 1, so that is not an artifact of a
	 * broken first attempt.
	 *
	 * That is a narrow claim: it says nothing about a retry entered from
	 * genuinely wedged state rather than from an abandoned-on-timeout one,
	 * which is the case the assert was restored for and which this
	 * experiment cannot construct.  Keep the assert -- it matches the GXL
	 * vendor BL2 -- but do not credit it with the recovery.
	 */
	if (IS_ENABLED(CONFIG_MESON_GXL) &&
	    IS_ENABLED(CONFIG_MESON_DRAM_PHY_RESET_ASSERT)) {
		/* Assert, so a RE-entry starts from reset and not from wreckage */
		writel(0, DMC_SOFT_RST);
		writel(0, DMC_SOFT_RST1);
		udelay(10);
	}

	/* Release reset of DLL */
	writel(0xffffffff, DMC_SOFT_RST);
	writel(0xffffffff, DMC_SOFT_RST1);
	udelay(10);

	/* Enable UPCTL and PUB clock */
	if (IS_ENABLED(CONFIG_MESON_GXBB))
		writel(0x550620, DMC_PCTL_LP_CTRL);
	else if (IS_ENABLED(CONFIG_MESON_GXL))
		writel(0, DMC_PCTL_LP_CTRL);

	/* Vendor ddr_init_pctl HOLD(0)/RELEASE(0xf); GXL-scoped as above. */
	if (IS_ENABLED(CONFIG_MESON_GXL)) {
		writel(0, DDR0_SOFT_RESET);
		udelay(10);
	}
	writel(0xf, DDR0_SOFT_RESET);
	udelay(10);
}

/*
 * ZQ program-value overrides, 0 = use the board's own value from
 * dram-settings-gx.h.  These exist to answer one question the first ZQ
 * readback opened (N46): ZQ0 and ZQ1 are programmed IDENTICALLY on this board
 * and report differently -- sr0 = 0x30, sr1 = 0.  No field map for ZQnSR
 * exists in any vendor authority we hold (secure_apb.h and cmd_ddr_test.c both
 * give the ADDRESS only), so the words cannot be decoded into ohms; what CAN
 * be established without a decode is whether each SR responds to its PR at
 * all.  Compile-time constants, inert at 0.
 */
#if defined(CONFIG_MESON_DRAM_ZQ0PR_OVERRIDE) && CONFIG_MESON_DRAM_ZQ0PR_OVERRIDE > 0
#define MESON_ZQ0PR	CONFIG_MESON_DRAM_ZQ0PR_OVERRIDE
#else
#define MESON_ZQ0PR	PUB_ZQ0PR
#endif
#if defined(CONFIG_MESON_DRAM_ZQ1PR_OVERRIDE) && CONFIG_MESON_DRAM_ZQ1PR_OVERRIDE > 0
#define MESON_ZQ1PR	CONFIG_MESON_DRAM_ZQ1PR_OVERRIDE
#else
#define MESON_ZQ1PR	PUB_ZQ1PR
#endif

/*
 * PCTL_STATE_TIMEOUT_US (dram-gx.h) bounds every wait here.  The instrument
 * lineage's meson_dram_pctl_wait_state(), which the original N27 commit also
 * touched, is NOT on this branch -- production still waits on PCTL_STAT
 * through WAIT_FOR(), bounded below.
 */

/*
 * The PGSR0 all-done bound, 0.5 s.  Kconfig-settable so N27's failure path is
 * reachable on a healthy board; unset, it is the constant this file has always
 * used and the image does not move.  Defined here rather than beside its use
 * because the use is above the retry ladder.
 */
#if defined(CONFIG_MESON_DRAM_PGSR0_TIMEOUT_US) && \
	CONFIG_MESON_DRAM_PGSR0_TIMEOUT_US > 0
#define MESON_DRAM_PGSR0_TIMEOUT_US	CONFIG_MESON_DRAM_PGSR0_TIMEOUT_US
#else
#define MESON_DRAM_PGSR0_TIMEOUT_US	500000
#endif

/*
 * How many LEADING training attempts get the shortened PGSR0 bound.  0 -- the
 * default -- compiles the whole arm away, counter included, so the shipping
 * stage carries none of it.  Non-zero is the recovery instrument: see the
 * branch in meson_dram_do_init() and MESON_DRAM_FAIL_FIRST_ATTEMPTS in Kconfig.
 */
#if defined(CONFIG_MESON_DRAM_FAIL_FIRST_ATTEMPTS)
#define MESON_DRAM_FAIL_FIRST_ATTEMPTS	CONFIG_MESON_DRAM_FAIL_FIRST_ATTEMPTS
#else
#define MESON_DRAM_FAIL_FIRST_ATTEMPTS	0
#endif

#if MESON_DRAM_FAIL_FIRST_ATTEMPTS > 0
/* 1-based attempt the ladder is on; instrument-only, see above. */
static int dram_attempt;
#endif

/*
 * The bounded body of WAIT_FOR().  The macro it replaces spun forever on
 * `readl(a) & 1` and carried a panic() arm that could not execute (see
 * dram-gx.h).  100 ms is the same bound the PGSR0 waits in this file use
 * and is ~4 orders of magnitude over any transition this hardware makes.
 *
 * It REPORTS and returns rather than panicking, because two of its four GXL
 * call sites sit inside a retry ladder whose whole job is to try again: a
 * panic here would convert a recoverable attempt into a dead board, and the
 * failure this exists to end is a SILENT one, not an unhandled one.
 */
int meson_dram_wait_bit0(unsigned long a, const char *who)
{
	int timeout = PCTL_STATE_TIMEOUT_US;
	u32 v;

	for (;;) {
		v = readl(a);
		if (v & 1)
			return 0;
		if (timeout <= 0) {
			printf("DDR wait %s: bit0 never set, val=%08x (raw)\n",
			       who, v);
			return -ETIMEDOUT;
		}
		udelay(10);
		timeout -= 10;
	}
}

void meson_dram_set_memory_timings(void)
{
	/* Set memory timings */
	writel(timings.rfc, PCTL_TRFC);
	/*
	 * PCTL_TFAW (uPCTL word 0x11): tFAW, the four-activate window, in
	 * DRAM clocks -- the controller issues no fifth ACTIVATE within this
	 * many clocks of the first of four.  GXBB has no such register
	 * (dram-gxbb.h defines it 0).  The same value also reaches the PHY
	 * copy, PUB_DTPR1 at bit 16 (dram-settings-gx.h).  DDR3 on GXL: derived
	 * from CONFIG_DRAM_CLK per the part's speed bins (Nanya NT5CB256M16DP
	 * datasheet v2.3 p.146 tFAW, p.6 undefined-frequency clause), 37 clk
	 * at 912 MHz -- see GXL_DDR3_TFAW_CLK in dram-timings-gx.h.  DDR4:
	 * the literal in its own timings table.
	 */
	if (IS_ENABLED(CONFIG_MESON_GXL))
		writel(timings.faw, PCTL_TFAW);
	writel(timings.refi_mddr3, PCTL_TREFI_MEM_DDR3);
	/*
	 * Vendor ddr_init_pctl clamps field widths (RE gxl_bl2_ddr_deep.c):
	 *   mrd  > 6     → 7   (3-bit field; ACS mrd=8 else reads back 0)
	 *   exsr > 0x3ff → 0x3ff (10-bit; ACS exsr=1024 else 0)
	 *   zqcs as s8 <0 → 0x7f (7-bit; ACS zqcs=128 else 0)
	 * Open residual dump: TMRD/TEXSR/TZQCS live 0 with unclamped ACS.
	 */
#if defined(CONFIG_DRAM_DDR4)
	{
		u32 mrd = timings.mrd;
		u32 exsr = timings.exsr;
		u32 zqcs = timings.zqcs;

		if (mrd > 6)
			mrd = 7;
		if (exsr > 0x3ff)
			exsr = 0x3ff;
		if ((signed char)zqcs < 0)
			zqcs = 0x7f;
		writel(mrd, PCTL_TMRD);
		writel(exsr, PCTL_TEXSR);
		writel(zqcs, PCTL_TZQCS);
	}
#else
	writel(timings.mrd, PCTL_TMRD);
	writel(timings.exsr, PCTL_TEXSR);
	writel(timings.zqcs, PCTL_TZQCS);
#endif
	if (IS_ENABLED(CONFIG_MESON_GXL))
		writel((timings.rp << 16) | timings.rp, PCTL_TRP);
	else /* Meson GXBB */
		writel(timings.rp, PCTL_TRP);
	writel(timings.cke + 1, PCTL_TCKESR);
	writel(timings.al, PCTL_TAL);
	writel(timings.cwl, PCTL_TCWL);
	/* GXL DDR4: cl + 2 = 20 with read DBI on, cl = 18 off (GXL_DDR4_READ_DBI) */
	writel(PCTL0_TCL, PCTL_TCL);
	writel(timings.ras, PCTL_TRAS);
	writel(timings.rc, PCTL_TRC);
	writel(timings.rcd, PCTL_TRCD);
	if (IS_ENABLED(CONFIG_MESON_GXBB)) {
		writel(timings.rrd, PCTL_TRRD);
	} else {
		writel(timings.rrd | ((timings.rrd + 2) * 0x10000), PCTL_TRRD);
		writel((timings.tccdl << 16) | 4, PCTL_TCCD);
	}
	writel(timings.rtp, PCTL_TRTP);
	writel(timings.wr, PCTL_TWR);
	writel(timings.wtr, PCTL_TWTR);
	writel(timings.xp, PCTL_TXP);
	writel(timings.dqs, PCTL_TDQS);
	writel(timings.rtw, PCTL_TRTW);
	writel(timings.cksre, PCTL_TCKSRE);
	writel(timings.cksrx, PCTL_TCKSRX);
	writel(timings.mod, PCTL_TMOD);
	writel(timings.cke, PCTL_TCKE);
	writel(timings.zqcl, PCTL_TZQCL);
	writel(timings.xpdll, PCTL_TXPDLL);
	writel(timings.zqcsi, PCTL_TZQCSI);

	if (IS_ENABLED(CONFIG_MESON_GXBB)) {
		/* GXBB: Enter config state */
		writel(PCTL0_SCFG, PCTL_SCFG);
		writel(PCTL_SCTL_CFG_STATE, PCTL_SCTL);
	}
}

void meson_dram_set_dfi_timings(void)
{
#ifdef CONFIG_MESON_GXL
	writel(0xab0a560a, PCTL_ZQCFG);
#endif
	WAIT_FOR(PCTL_STAT);

	writel(PCTL0_PPCFG, PCTL_PPCFG);
	writel(PCTL0_DFISTCFG0, PCTL_DFISTCFG0);
	writel(PCTL0_DFISTCFG1, PCTL_DFISTCFG1);
	writel(PCTL0_DFITCTRLDELAY, PCTL_DFITCTRLDELAY);
	writel(PCTL0_DFITPHYWRDATA, PCTL_DFITPHYWRDATA);
	writel(PCTL0_DFITPHYWRLTA, PCTL_DFITPHYWRLAT);
	writel(PCTL0_DFITRDDATAEN, PCTL_DFITRDDATAEN);
	writel(PCTL0_DFITPHYRDLAT, PCTL_DFITPHYRDLAT);
	writel(PCTL0_DFITDRAMCLKDIS, PCTL_DFITDRAMCLKDIS);
	writel(PCTL0_DFITDRAMCLKEN, PCTL_DFITDRAMCLKEN);
	writel(PCTL0_DFITCTRLUPDMIN, PCTL_DFITCTRLUPDMIN);
#if defined(CONFIG_MESON_GXL)
	writel(PCTL0_DFITCTRLUPDMAX, PCTL_DFITCTRLUPDMAX);
	writel(PCTL0_DFIUPDCFG, PCTL_DFIUPDCFG);
#endif
	writel(PCTL0_DFILPCFG0, PCTL_DFILPCFG0);
#if defined(CONFIG_MESON_GXL)
	writel(PCTL0_DFITPHYUPDTYPE0, PCTL_DFITPHYUPDTYPE0);
#endif
	writel(PCTL0_DFITPHYUPDTYPE1, PCTL_DFITPHYUPDTYPE1);
	writel(PCTL0_DFIODTCFG, PCTL_DFIODTCFG);
	writel(PCTL0_DFIODTCFG1, PCTL_DFIODTCFG1);
#if defined(CONFIG_MESON_GXBB)
	writel(PCTL0_CMDTSTATEN, PCTL_CMDTSTATEN);
#endif
}

/* Default train error field: ZC/WL/QSG/WLA/RD/WD/RE/WE [27:20] */
#define PUB_PGSR0_ERR_ALL	0x0ff00000u
/* S1 (PIR 0xf3) only produces ZCERR among the error class */
#define PUB_PGSR0_ERR_S1	PUB_PGSR0_ZCERR

/*
 * Wait until (PGSR0 & expected_mask) == expected_mask, then check err_mask.
 * Do not sample errors until done bits assert (stale QSGERR from a prior
 * QSGATE must not abort mid-S1). expected_mask==0 → IDONE.
 */
static int meson_dram_wait_pgsr0_err(u32 expected_mask, u32 err_mask,
				    int timeout_us)
{
	u32 pgsr0;
	u32 need = expected_mask ? expected_mask : PUB_PGSR0_IDONE;
	int i;

	for (i = 0; i < timeout_us; i += 10) {
		pgsr0 = readl(DDR0_PUB_PGSR0);
		if ((pgsr0 & need) == need) {
			if (pgsr0 & err_mask)
				return -1;
			return 0;
		}
		udelay(10);
	}

	return -1;
}

static int meson_dram_wait_pgsr0(u32 expected_mask, int timeout_us)
{
	return meson_dram_wait_pgsr0_err(expected_mask, PUB_PGSR0_ERR_ALL,
					 timeout_us);
}

#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR4)
/* Sticky train breadcrumbs (survive mark_and_return → BROM readMemory).
 * Layout (SRAM, after BL2 runpara @ c000):
 *   0xd900c100  OMOR on rom-return (meson-usb)
 *   0xd900c120  prod_stage 0x5354xxxx (spl.c)
 *   0xd900c128  train step  0x5354d4ss
 *   0xd900c12c  last PGSR0
 *   0xd900c130  DX0GSR0  (vendor RE PUB 0x1f8)
 *   0xd900c134  DX1GSR0
 *   0xd900c138  DX2GSR0
 *   0xd900c13c  DX3GSR0
 *   0xd900c140  DX0GSR2  (gate window / rank status)
 *   0xd900c144  DX1GSR2
 *   0xd900c148  DX2GSR2
 *   0xd900c14c  DX3GSR2
 *   0xd900c150  DX0GTR   (gate train result)
 *   0xd900c154  DX1GTR
 *   0xd900c158  DX2GTR
 *   0xd900c15c  DX3GTR
 *   0xd900c160  magic 0x47535230 "GSR0" marks GSR block valid
 */
/* One definition, shared with the handoff block that forwards these two. */
#define GXL_DDR_STICKY_STEP_ADDR	GX_DDR_STICKY_STEP_ADDR
#define GXL_DDR_STICKY_PGSR_ADDR	GX_DDR_STICKY_PGSR_ADDR
#define GXL_DDR_STICKY_DX0GSR0	0xd900c130UL
#define GXL_DDR_STICKY_DX1GSR0	0xd900c134UL
#define GXL_DDR_STICKY_DX2GSR0	0xd900c138UL
#define GXL_DDR_STICKY_DX3GSR0	0xd900c13cUL
#define GXL_DDR_STICKY_DX0GSR2	0xd900c140UL
#define GXL_DDR_STICKY_DX1GSR2	0xd900c144UL
#define GXL_DDR_STICKY_DX2GSR2	0xd900c148UL
#define GXL_DDR_STICKY_DX3GSR2	0xd900c14cUL
#define GXL_DDR_STICKY_DX0GTR	0xd900c150UL
#define GXL_DDR_STICKY_DX1GTR	0xd900c154UL
#define GXL_DDR_STICKY_DX2GTR	0xd900c158UL
#define GXL_DDR_STICKY_DX3GTR	0xd900c15cUL
#define GXL_DDR_STICKY_GSR_MAGIC_ADDR 0xd900c160UL
#define GXL_DDR_STICKY_GSR_MAGIC	0x47535230u /* 'GSR0' */
#define GXL_DDR_STICKY_MAGIC	0x5354d400u

/*
 * Snapshot per-lane gate status at QSGATE fail for sticky smoke.
 *
 * Sticky layout extension:
 *   c130-c13c  DXnGSR0  (vendor RE: PUB 0x1f8/238/278/2b8 → 0xc88367e0…)
 *   c140-c14c  DXnGCR0  (lane enable — DX2/DX3=0 hypothesis)
 *   c150-c15c  DXnGTR   (gate train result @ PUB 0x1f0/230/270/2b0)
 *   c160       magic 0x47535230 ('GSR0') when dump completed
 *
 * Read order: GCR0/GTR first (known-good headers), then GSR0 (RE train-err).
 */
static void gxl_ddr_sticky_dxgsr(void)
{
	/*
	 * Incremental capture: write magic first so host can see progress if
	 * a later PUB read hangs the bus.
	 */
	writel(0x47535231u, (void *)GXL_DDR_STICKY_GSR_MAGIC_ADDR); /* GSR1 */

	writel(readl(DDR0_PUB_DX0GCR0), (void *)GXL_DDR_STICKY_DX0GSR2);
	writel(readl(DDR0_PUB_DX1GCR0), (void *)GXL_DDR_STICKY_DX1GSR2);
	writel(readl(DDR0_PUB_DX2GCR0), (void *)GXL_DDR_STICKY_DX2GSR2);
	writel(readl(DDR0_PUB_DX3GCR0), (void *)GXL_DDR_STICKY_DX3GSR2);
	writel(0x47535232u, (void *)GXL_DDR_STICKY_GSR_MAGIC_ADDR); /* GSR2=GCR done */

	writel(readl(DDR0_PUB_DX0GTR), (void *)GXL_DDR_STICKY_DX0GTR);
	writel(readl(DDR0_PUB_DX1GTR), (void *)GXL_DDR_STICKY_DX1GTR);
	writel(readl(DDR0_PUB_DX2GTR), (void *)GXL_DDR_STICKY_DX2GTR);
	writel(readl(DDR0_PUB_DX3GTR), (void *)GXL_DDR_STICKY_DX3GTR);
	writel(0x47535233u, (void *)GXL_DDR_STICKY_GSR_MAGIC_ADDR); /* GSR3=GTR done */

	writel(readl(DDR0_PUB_DX0GSR0), (void *)GXL_DDR_STICKY_DX0GSR0);
	writel(readl(DDR0_PUB_DX1GSR0), (void *)GXL_DDR_STICKY_DX1GSR0);
	writel(readl(DDR0_PUB_DX2GSR0), (void *)GXL_DDR_STICKY_DX2GSR0);
	writel(readl(DDR0_PUB_DX3GSR0), (void *)GXL_DDR_STICKY_DX3GSR0);
	writel(GXL_DDR_STICKY_GSR_MAGIC, (void *)GXL_DDR_STICKY_GSR_MAGIC_ADDR);
}

static void gxl_ddr_sticky_step(u32 step)
{
	/* step==0xfe: fail — keep last step code, refresh PGSR0 only */
	if (step != 0xfe)
		writel(GXL_DDR_STICKY_MAGIC | (step & 0xff),
		       (void *)GXL_DDR_STICKY_STEP_ADDR);
	writel(readl(DDR0_PUB_PGSR0), (void *)GXL_DDR_STICKY_PGSR_ADDR);
}

/* Wait helper: on fail stamp sticky PGSR0 and return -1. Always braced. */
static int gxl_ddr_wait_or_fail_err(u32 expected, u32 err_mask, int timeout_us)
{
	if (meson_dram_wait_pgsr0_err(expected, err_mask, timeout_us)) {
		gxl_ddr_sticky_step(0xfe); /* keep step code, refresh PGSR0 */
		return -1;
	}
	/* success: refresh PGSR0 breadcrumb without changing step code */
	writel(readl(DDR0_PUB_PGSR0), (void *)GXL_DDR_STICKY_PGSR_ADDR);
	return 0;
}

static int gxl_ddr_wait_or_fail(u32 expected, int timeout_us)
{
	return gxl_ddr_wait_or_fail_err(expected, PUB_PGSR0_ERR_ALL,
					timeout_us);
}

/*
 * gxl_ddr_clrsr_status() USED TO LIVE HERE, and it was removed because four
 * boots on silicon measured every one of its premises false (N95, N104, N559,
 * N560).  Recorded rather than deleted silently, so nobody re-derives it:
 *
 *   - It wrote PUB_PIR_CLRSR (BIT(27)), which the DDR3 arc measured NOT TO
 *     EXIST on this PHY: writing it reads back 0 and moves nothing.
 *   - Its intent -- have PGSR0[27:20] clear before the next PIR -- cannot be
 *     met by ANY write.  N559 refuted write-1-to-clear two-sided, and PGSR0
 *     absorbed both the positive and the negative write without moving one bit.
 *   - What DOES clear the field is the OWNING STAGE re-running: the error bit
 *     goes at the completion of the PIR that carries the stage owning it
 *     (N559: QSGERR clears at PIR 0xf01, which carries PUB_PIR_QSGATE).
 *   - So both of its call sites were measured INERT -- the bit was still
 *     standing when it returned, at both sites, on every boot.
 *   - Cost, MEASURED on a live clock (N560, arm A, both call sites, two boots):
 *     polls=4000 us=76996 per call.  Two calls = 153992 us per DDR4 boot of
 *     polling that could only ever time out, plus one unrequested PHY INIT per
 *     call, since the "second try" writes PUB_PIR_INIT alongside.
 *
 * Removing it is safe because of what the WAIT does, not because the helper was
 * inert.  meson_dram_wait_pgsr0_err() above samples the error field exactly
 * once, in the same readl() word in which the done condition is first observed
 * satisfied, and at the 0xf01 site `need' is PUB_PGSR0_IDONE alone.  So the
 * risk was a STALE IDONE arriving beside a standing QSGERR.  N560 measured the
 * wait's own first read, at the wait's own code position, on four boots:
 *
 *     f01-pre   PGSR0=8040005f ERRBITS=00400000   (QSGERR standing)
 *     f01-s0    PGSR0=8040004e ... idone=0        (IDONE already clear)
 *     f01-entry min=00400000 clear_at=0 idone_at=-1 samples=64
 *
 * The PHY's entry clear of IDONE beat the CPU's first read every time, and
 * IDONE was never once observed set across 64 back-to-back samples, so the wait
 * cannot reach its error check on stale state.  Two of those four boots ran
 * with both call sites already skipped and trained normally
 * (f01-wait rc=0, FIN 0 80000fff, 2048 MB, memtest OK).
 */

/* S1 done mask: INIT + PLL + DDL cal + ZQ cal (PIR 0xf3 set). */
#define GXL_DDR4_S1_DONE	(PUB_PGSR0_IDONE | PUB_PGSR0_PLDONE | \
				 PUB_PGSR0_DCDONE | PUB_PGSR0_ZCDONE)

static uint meson_dram_ddr4_stepped_training(void)
{
	gxl_ddr_sticky_step(0x00);
	u32 pgsr0;

	/*
	 * DDR4 stepped training from vendor BL2 (gxl_bl2_ddr_deep.c,
	 * ddrtype==1). PIR sequence: 0xf3 → AC delays → 0xf01 → 0x1001
	 * → 0xe001 → 0x20001 + MR6 VREF latch.
	 *
	 * PGSR0 0x8040005f decode (sticky after prior S1 attempt):
	 *   done: IDONE|PLDONE|DCDONE|ZCDONE|DIDONE|QSGDONE
	 *   err:  QSGERR (bit22) — stale from a prior QSGATE, not S1
	 * Brace bug (if wait) sticky; return -1 without braces aborted
	 * every step after S1 wait. Fixed via gxl_ddr_wait_or_fail().
	 */

	/* Re-apply vendor ZQ PR (CC-V2 timing.c: 0x0995d/0x3f95d/0x1dd1d) */
	writel(MESON_ZQ0PR, DDR0_PUB_ZQ0PR);
	writel(MESON_ZQ1PR, DDR0_PUB_ZQ1PR);
	writel(PUB_ZQ2PR, DDR0_PUB_ZQ2PR);
	writel(PUB_ZQ3PR, DDR0_PUB_ZQ3PR);

	/*
	 * DFI ODT (PCTL) — vendor CC-V2 timing.c before/after train:
	 *   dfiodtcfg  = 0x808  ((1<<3)|(1<<11))  rank0 ODT enable
	 *   dfiodtcfg1 = 0x60000 (ODT length/latency)
	 * Re-assert before S2 so gate train sees correct DFI ODT.
	 */
	writel(PCTL0_DFIODTCFG, PCTL_DFIODTCFG);
	writel(PCTL0_DFIODTCFG1, PCTL_DFIODTCFG1);

	/*
	 * Train ODTCR (PHY): vendor RE always writes 0x10000 for DDR4 S2
	 * (mode2 rank0 write-ODT). Static PUB_ODTCR is 0x30000 (vendor).
	 */
	writel(0x10000, DDR0_PUB_ODTCR);

	/*
	 * Vendor only forces MR1 RTT_WR (bit10 / 0x400) when ddr4_odt==0
	 * (DDR4_ODT_0OHM). CC-V2 uses 60Ω — do not force RTT_WR.
	 */
	if (timings.odt == 0)
		writel(readl(DDR0_PUB_MR1) | 0x400, DDR0_PUB_MR1);

	/* Setup VTCR0/VTCR1 for VREF training */
	clrsetbits_32(DDR0_PUB_VTCR0, 0x0fff0000, 0xf0032000);
	writel(0xfc011f2, DDR0_PUB_VTCR1);

	/* Clear DXCCR training-related bits [12:5] (vendor DXCCR &= ~0x1fe0) */
	clrbits_32(DDR0_PUB_DXCCR, 0x1fe0);

	/*
	 * Step 1: Basic PHY init (INIT|ZCAL|PLLINIT|DCAL|PHYRST|DRAMRST)
	 * Vendor PIR = 0xf3.
	 *
	 * No status clear before this PIR: this site's wait masks
	 * PUB_PGSR0_ERR_S1 (ZCERR only), and N559 measured QSGERR standing
	 * across a SUCCESSFUL Step 1, so a standing error bit was already
	 * outside this mask.  See the block above gxl_ddr_wait_or_fail_err().
	 */
	gxl_ddr_sticky_step(0x01);
	writel(PUB_PIR_INIT | PUB_PIR_ZCAL | PUB_PIR_PLLINIT |
	       PUB_PIR_DCAL | PUB_PIR_PHYRST | PUB_PIR_DRAMRST,
	       DDR0_PUB_PIR);
	if (gxl_ddr_wait_or_fail_err(GXL_DDR4_S1_DONE, PUB_PGSR0_ERR_S1,
				     100000))
		return -1;
	/* Extra delay for DDR4 tDLLK */
	udelay(500);

	/* Set ACLCDLR for DDR4 2T command timing (vendor: raw t_pub_aclcdlr) */
	writel(PUB_ACLCDLR, DDR0_PUB_ACLCDLR);
	writel(0x3f003f, DDR0_PUB_ACBDLR2);
	writel(0x3f3f, DDR0_PUB_ACBDLR4);
	writel(0x3f3f, DDR0_PUB_ACBDLR5);
	if (PUB_ACLCDLR < 63) {
		writel(PUB_ACLCDLR | (PUB_ACLCDLR << 16), DDR0_PUB_ACBDLR2);
		writel(PUB_ACLCDLR | (PUB_ACLCDLR << 8), DDR0_PUB_ACBDLR4);
		writel(PUB_ACLCDLR | (PUB_ACLCDLR << 8), DDR0_PUB_ACBDLR5);
	}

	/*
	 * Step 2: vendor DDR4 happy path (gxl_bl2_ddr_deep.c, ddrtype==1):
	 *   DTCR |= 0xc000 if ddr_func RDBI (bit6). Product console prints
	 *   "DDR enable rdbi" — open previously skipped (wrong ACS assumption).
	 *   PIR = 0xf01 = INIT|DRAMINIT|WL|QSGATE|WLADJ (no GTR seed).
	 * Recovery 0xf001 / combined-with-RANKEN already banked QSGATE; this
	 * is the RE gate path without train-RANKEN.
	 *
	 * No status clear before this PIR either, and this is the site the
	 * measurement was about: QSGERR IS inside this wait's mask and IS
	 * standing here.  N560 measured the wait's own first read after the
	 * write on four boots -- IDONE was already clear at sample 0 and was
	 * never observed set across 64 back-to-back samples -- so the wait
	 * cannot reach its error check on the previous stage's state.  See the
	 * block above gxl_ddr_wait_or_fail_err().
	 */
	gxl_ddr_sticky_step(0x20);
	/*
	 * Product RDBI: DTCR[15:14] for S2; cleared before eye below.  Only
	 * where read DBI is on (GXL_DDR4_READ_DBI, dram-settings-gx.h).
	 */
	if (GXL_DDR4_READ_DBI)
		setbits_32(DDR0_PUB_DTCR, 0xc000);
	writel(PUB_PIR_INIT | PUB_PIR_DRAMINIT | PUB_PIR_WL |
	       PUB_PIR_QSGATE | PUB_PIR_WLADJ, DDR0_PUB_PIR); /* 0xf01 */
	if (gxl_ddr_wait_or_fail_err(
			PUB_PGSR0_IDONE,
			PUB_PGSR0_WLERR | PUB_PGSR0_QSGERR | PUB_PGSR0_WLAERR,
			400000)) {
		gxl_ddr_sticky_dxgsr();
		return -1;
	}
	gxl_ddr_sticky_dxgsr();

	/* Clear DXCCR training bits again before deskew (vendor) */
	clrbits_32(DDR0_PUB_DXCCR, 0x1fe0);
	/* Vendor: dtcr &= ~0xc000 before eye — no-op if RANKEN never set */
	clrbits_32(DDR0_PUB_DTCR, 0xc000);

	/*
	 * Step 3: Read bit deskew only — PIR = 0x1001 (INIT|RDDSKW)
	 */
	gxl_ddr_sticky_step(0x03);
	writel(PUB_PIR_INIT | PUB_PIR_RDDSKW, DDR0_PUB_PIR);
	if (gxl_ddr_wait_or_fail(PUB_PGSR0_IDONE | PUB_PGSR0_RDDONE, 100000))
		return -1;

	/*
	 * Check DXnBDLR5 — if any byte lane has zero, wait longer.
	 *
	 * The read is UNCHANGED (word 0x1d6 + 0x40/lane); only the symbol was
	 * renamed, because the vendor names 0x1d6 BDLR5 and not BDLR6.  This
	 * matches vendor BL2, which checks the same four words at the same
	 * point — but which ALSO spends an unconditional udelay(1000) before
	 * this one, where we poll PGSR0 instead.  So our settling budget here
	 * is poll+1 ms against the vendor's ~2 ms; that divergence is recorded,
	 * not corrected, because it is a live question rather than a known bug.
	 *
	 * BDLR5 is the READ-direction DM bit delay, and it is measured zero in
	 * all 1240 lane-boots on record at a strictly later point in the boot —
	 * so this branch is presumptively taken every time and the udelay is
	 * unconditional in practice.  The PHY's read-DM training needs its
	 * RDBI training flags (DTCR[15:14]) off, which is what
	 * clrbits_32(DDR0_PUB_DTCR, 0xc000) above arranges -- a training-step
	 * flag, distinct from the DRAM's read-DBI mode (MR5 A12,
	 * GXL_DDR4_READ_DBI in dram-settings-gx.h).
	 * Left as-is deliberately: changing it moves a 1 ms term between two
	 * training steps.
	 */
	if (!readl(DDR0_PUB_DX0BDLR5) || !readl(DDR0_PUB_DX1BDLR5) ||
	    !readl(DDR0_PUB_DX2BDLR5) || !readl(DDR0_PUB_DX3BDLR5))
		udelay(1000);

	/*
	 * Step 4: Write deskew + read/write eye — PIR = 0xe001
	 */
	clrbits_32(DDR0_PUB_DTCR, 0xc000);
	gxl_ddr_sticky_step(0x04);
	writel(PUB_PIR_INIT | PUB_PIR_WRDSKW | PUB_PIR_RDEYE | PUB_PIR_WREYE,
	       DDR0_PUB_PIR);
	if (gxl_ddr_wait_or_fail(PUB_PGSR0_IDONE | PUB_PGSR0_WDDONE |
				 PUB_PGSR0_REDONE | PUB_PGSR0_WEDONE,
				 200000))
		return -1;

	/*
	 * Step 5: ACS SOC/DRAM VREF encode (gxl_bl2_ddr_deep.c after S4).
	 * CC-V2 pack 0x2612: DRAM→VTCR0 formula, SOC→DXnGCR5 | 0x9090900,
	 * VTCR1 bit1 clear; when both non-zero vendor SKIPS PIR VREF and
	 * latches ACS MR6 via DRAMINIT. Open used fixed VTCR0 0xf0032000 +
	 * always-PIR — ACS red residual for open-bus.
	 */
	gxl_ddr_sticky_step(0x05);
	if (PUB_DRAM_VREF_CODE) {
		u32 dv = PUB_DRAM_VREF_CODE;

		writel(((dv + 1) * 0x1000 | (dv - 1) * 0x40 | dv | 0xf0000000) &
		       0xefffffff, DDR0_PUB_VTCR0);
	}
	if (PUB_SOC_VREF_CODE) {
		u32 gcr5 = PUB_SOC_VREF_CODE | 0x9090900;

		clrbits_32(DDR0_PUB_VTCR1, BIT(1));
		writel(gcr5, DDR0_PUB_DX0GCR5);
		writel(gcr5, DDR0_PUB_DX1GCR5);
		writel(gcr5, DDR0_PUB_DX2GCR5);
		writel(gcr5, DDR0_PUB_DX3GCR5);
	}
	/* Auto path only when SOC or DRAM ACS code is 0 */
	if (!PUB_SOC_VREF_CODE || !PUB_DRAM_VREF_CODE) {
		writel(PUB_PIR_INIT | PUB_PIR_VREF, DDR0_PUB_PIR);
		if (gxl_ddr_wait_or_fail(PUB_PGSR0_IDONE, 200000))
			return -1;
	}

	/* Restore DTCR MPR bit */
	setbits_32(DDR0_PUB_DTCR, BIT(31));

	/* Latch ACS MR6 VREF via DRAMINIT (vendor when DRAM code != 0) */
	if (PUB_DRAM_VREF_CODE) {
		u32 mr6 = PUB_MR6;

		writel(readl(DDR0_PUB_MR1) & ~BIT(7), DDR0_PUB_MR1);
		writel(mr6 | BIT(7), DDR0_PUB_MR6);
		gxl_ddr_sticky_step(0x07);
		writel(PUB_PIR_INIT | PUB_PIR_DRAMINIT, DDR0_PUB_PIR);
		if (gxl_ddr_wait_or_fail(PUB_PGSR0_IDONE, 50000))
			return -1;
		udelay(1);

		writel(mr6 | BIT(7), DDR0_PUB_MR6);
		gxl_ddr_sticky_step(0x08);
		writel(PUB_PIR_INIT | PUB_PIR_DRAMINIT, DDR0_PUB_PIR);
		if (gxl_ddr_wait_or_fail(PUB_PGSR0_IDONE, 50000))
			return -1;
		udelay(1);

		writel(mr6 & ~BIT(7), DDR0_PUB_MR6);
		/* Vendor restores RTT_WR only when ddr4_odt==0 */
		if (timings.odt == 0)
			writel(readl(DDR0_PUB_MR1) | 0x400, DDR0_PUB_MR1);
		gxl_ddr_sticky_step(0x09);
		writel(PUB_PIR_INIT | PUB_PIR_DRAMINIT, DDR0_PUB_PIR);
		if (gxl_ddr_wait_or_fail(PUB_PGSR0_IDONE, 50000))
			return -1;

		/*
		 * RUN K (2026-08-10) — put MR1 bit 7 BACK.
		 *
		 * The clear above is a VREF-training requirement, but it is the
		 * last write to MR1 on our path, so the bit stays clear for the
		 * rest of the boot: RUN I measured our live MR1 = 0x00000101
		 * against product BL2's 0x00000181 on this board, and our own
		 * ACS seed (PUB_MR1 ... | 0x81) has the bit.  Restore it here,
		 * before the remaining training steps, so a later DRAMINIT
		 * latches it into the part the way product's does.
		 */
		writel(readl(DDR0_PUB_MR1) | BIT(7), DDR0_PUB_MR1);
	}

	/* Restore ODTCR to normal operating value */
	writel(PUB_ODTCR, DDR0_PUB_ODTCR);

	/*
	 * Vendor GXL DDR4 stepped path never issues CTLDINIT
	 * (gxl_bl2_ddr_deep.c: PIR ends at DRAMINIT/VREF; handoff is
	 * PCTL SCTL GO only). Open previously ran INIT|CTLDINIT here as
	 * an open-bus hypothesis — train still STEP 0xff but
	 * got0=ffffffff. Drop to match RE (one residual lever).
	 */


	gxl_ddr_sticky_dxgsr();
	gxl_ddr_sticky_step(0xff); /* train OK */
	return 0;
}
#endif

uint meson_dram_phy_finalise_init(void)
{
	writel(MESON_ZQ0PR, DDR0_PUB_ZQ0PR);
	writel(MESON_ZQ1PR, DDR0_PUB_ZQ1PR);
	writel(PUB_ZQ2PR, DDR0_PUB_ZQ2PR);
	/* Vendor timing.c always programs ZQ3PR (CC-V2: 0x1dd1d). */
	writel(PUB_ZQ3PR, DDR0_PUB_ZQ3PR);

	writel(PUB_PIR_INIT | PUB_PIR_ZCAL, DDR0_PUB_PIR);
	WAIT_FOR(DDR0_PUB_PGSR0);
	/*
	 * ZQCR post-ZCAL sequence (vendor gxl_bl2_ddr_deep.c):
	 * pulse bits 2+27, then for DDR4 clear [27:24] and set 0x3000000
	 * (mode bits 25:24). Open previously always cleared 2+27 only —
	 * wrong for DDR4 and a plausible QSGATE impedance confounder.
	 */
	writel(readl(DDR0_PUB_ZQCR) | (1 << 2) | (1 << 27), DDR0_PUB_ZQCR);
	udelay(10);
	{
		u32 zq = readl(DDR0_PUB_ZQCR);

		if (IS_ENABLED(CONFIG_DRAM_DDR4))
			zq = (zq & 0xf0fffffb) | 0x3000000;
		else
			zq = zq & ~((1 << 2) | (1 << 27));
		writel(zq, DDR0_PUB_ZQCR);
	}
	udelay(30);

#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_16BIT_RANK)
	clrsetbits_32(DDR0_PUB_DX2GCR0, 0xb0001, 0xb0000);
	clrsetbits_32(DDR0_PUB_DX3GCR0, 0xb0001, 0xb0000);
#endif

	writel(PUB_ACBDLR0, DDR0_PUB_ACBDLR0);
#if defined(CONFIG_MESON_GXL)
	writel(PUB_ACBDLR3, DDR0_PUB_ACBDLR3);
#endif

#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_2T_MODE) && defined(CONFIG_DRAM_DDR3)
	writel(0x10101010, DDR0_PUB_ACBDLR1);
	writel(0x10101010, DDR0_PUB_ACBDLR7);
	writel(0x20202020, DDR0_PUB_ACBDLR8);
	writel(0x30303030, DDR0_PUB_ACBDLR9);
	writel(0x3f003f, DDR0_PUB_ACBDLR2);
	writel(0, DDR0_PUB_ACBDLR6);
#endif

#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR3)
	/*
	 * DXCCR[12:5] for DDR3 training: 0x1ee0, the value the vendor GXL BL2
	 * writes before gate training on its other memory-type path
	 * (reversing-gxbb-bl2 gxl/bl2.c:2002), not the 0x1200 of its DDR3
	 * path (bl2.c:1974).  No cited source names the field's bits.
	 *
	 * Measured on AML-S905X-CC (claude amlogic n1215, 1392 MHz, interleaved
	 * arms): rank 1's DQS gate training locked about a clock late ("mode
	 * B", the 1200 MHz short size detection) on 4 of 321 trainings with
	 * 0x1200 and 0 of 320 with 0x1ee0; [12:5] = 0 broke training.  n1216
	 * found 0x1ee0 no worse than 0x1200 at 912 and 1200.
	 */
	clrsetbits_32(DDR0_PUB_DXCCR, 0x1fe0, 0x1ee0);
#endif

#if defined(CONFIG_DRAM_DDR4)
	/*
	 * GXL DDR4: vendor multi-step PIR + VREF training (ACS fix).
	 * One-shot PUB_PIR_FINAL_STEP | CTLDINIT did not wait on PGSR0
	 * and fell through without a return (USB identify timeout).
	 * See amlogic/software/boot/gxl/ddr4-spl-timing-gaps.md
	 */
#if defined(CONFIG_MESON_GXL)
	{
		uint ret;

		ret = meson_dram_ddr4_stepped_training();
		printf("FIN %d %x\n", ret, readl(DDR0_PUB_PGSR0));
		if (ret == 0)
			writel(0x5f97, DDR0_PUB_PGCR2);
		return ret;
	}
#else
	printf("PIR=%x\n", PUB_PIR_FINAL_STEP | PUB_PIR_CTLDINIT);
	writel(PUB_PIR_FINAL_STEP | PUB_PIR_CTLDINIT, DDR0_PUB_PIR);
	if (meson_dram_wait_pgsr0(PUB_PGSR0_IDONE, 100000))
		return -1;
	return 0;
#endif
#else
	writel(PUB_PIR_FINAL_STEP, DDR0_PUB_PIR);
	udelay(1000);

	{
		u32 pgsr0;
		/*
		 * 0.5 s, not the "5 seconds" this said until 2026-08-29:
		 * the loop decrements 10 per udelay(10), so the unit is us.
		 * The 740 ms measured between `DDR retry 2/3` and
		 * `PGSR0 timeout!` on n27a/n27c is this bound plus the readl
		 * overhead, which is what showed the comment was wrong.
		 *
		 * Kconfig-settable for the same reason as PCTL_STATE_TIMEOUT_US
		 * (see dram-gx.h): shortened, this wait really does expire, so
		 * `PGSR0 timeout!` and the whole retry ladder behind it print
		 * on a production image without anything being injected.  A
		 * compile-time constant either way, so the default image is
		 * unchanged.
		 */
#if MESON_DRAM_FAIL_FIRST_ATTEMPTS > 0
		/*
		 * The RECOVERY arm: the short bound applies only to the first
		 * N attempts, so attempt N+1 gets the built-in 0.5 s and
		 * succeeds.  That is the only way to reach the `try %d` print
		 * with a number above 1 on a production image -- a single
		 * constant bound fails every attempt identically and can only
		 * ever produce the give-up ladder.
		 *
		 * The whole arm, this branch and the counter it reads, is
		 * compiled out at the default, so it costs the shipping stage
		 * nothing.
		 */
		int timeout = (dram_attempt <= MESON_DRAM_FAIL_FIRST_ATTEMPTS) ?
			      MESON_DRAM_PGSR0_TIMEOUT_US : 500000;
#else
		int timeout = MESON_DRAM_PGSR0_TIMEOUT_US;
#endif

		do {
			pgsr0 = readl(DDR0_PUB_PGSR0);
			if ((pgsr0 & 0xfff) == 0xfff)
				break;	/* all done bits set */
			if (pgsr0 & 0x0ff00000) {
				/* error bits set */
				break;
			}
			udelay(10);
			timeout -= 10;
		} while (timeout > 0);

		printf("PGSR0=%x\n", pgsr0);

		if (pgsr0 & 0x0ff00000) {
			if (meson_dram_wait_pgsr0(0, 0))
				; /* just prints error details */
			return -1;
		}
		if (timeout <= 0) {
			printf("PGSR0 timeout!\n");
			return -1;
		}
	}

	return 0;
#endif
}

void meson_dram_dmc_set_addrmap(void)
{
	if (IS_ENABLED(CONFIG_MESON_GXBB)) {
		/* GXBB address map */
		if (IS_ENABLED(CONFIG_DRAM_TWO_IDENTICAL_RANKS) ||
		    IS_ENABLED(CONFIG_DRAM_ONE_RANK)) {
			writel(11 | 31 << 5 |  0 << 10 | 14 << 15 | 15 << 20 | 16 << 25,
			       DDR0_ADDRMAP_1);
			writel(30 | 12 << 5 | 13 << 10 | 29 << 15 |  0 << 20 |  0 << 25,
			       DDR0_ADDRMAP_4);
		} else if (IS_ENABLED(CONFIG_DRAM_TWO_DIFF_RANKS)) {
			writel(11 | 31 << 5 |  0 << 10 | 14 << 15 | 15 << 20 | 16 << 25,
			       DDR0_ADDRMAP_1);
			writel(0 | 12 << 5 | 13 << 10 | 29 << 15 |  0 << 20 | 30 << 25,
			       DDR0_ADDRMAP_4);
		}
	} else if (IS_ENABLED(CONFIG_MESON_GXL) && IS_ENABLED(CONFIG_DRAM_DDR3)) {
		/*
		 * GXL + DDR3, 32-bit bus.
		 *
		 * This map encodes ONE geometry: 15 row bits, 10 column bits,
		 * 3 bank bits, 1 GB per rank.  It is not "the GXL DDR3 map" --
		 * a part with a different row count needs a different map, and
		 * the boards this branch serves are not known to be
		 * homogeneous.  The reference fit is AML-S905X-CC 2 GB, whose
		 * BOM and schematic give two Nanya NT5CB256M16DP-EK per rank,
		 * two ranks: 4 Gbit 256M x16, BA0-BA2, A0-A14 row, A0-A9
		 * column.  Two x16 devices make the 32-bit bus, and the
		 * 2 KB page is per device.
		 *
		 * Each ADDRMAP field names the AXI bit that drives ONE DRAM
		 * address signal, six 5-bit fields per register:
		 *
		 *   ADDRMAP_0  ca3  ca4   ca5   ca6   ca7   ca8
		 *   ADDRMAP_1  ca9  ca10  ca11  ra0   ra1   ra2
		 *   ADDRMAP_2  ra3  ra4   ra5   ra6   ra7   ra8
		 *   ADDRMAP_3  ra9  ra10  ra11  ra12  ra13  ra14
		 *   ADDRMAP_4  ra15 ba0   ba1   ba2   bg1   ra16
		 *
		 * (arch-txlx/secure_apb.h and arch-axg/regs.h, which agree.
		 * The comment that used to sit here named a different field
		 * order and was wrong; do not restore it.)
		 *
		 * With 15 row, 10 column and 3 bank bits, ca10, ca11, ra15, bg1
		 * and ra16 have no cell behind them and must be parked on an
		 * AXI bit that never varies inside the window: 0 is the byte
		 * lane, 31 is above it.  ca10 in particular drives A10 = the
		 * auto-precharge flag, so mapping it to a live AXI bit changes
		 * the COMMAND, not the location.
		 *
		 * That leaves exactly 1 GB per rank:
		 *
		 *   AXI[1:0]    byte lane
		 *   AXI[4:2]    ca0..ca2, the burst-8 offset (no ADDRMAP field)
		 *   AXI[10:5]   ca3..ca8
		 *   AXI[11]     ca9
		 *   AXI[14:12]  ba0..ba2
		 *   AXI[29:15]  ra0..ra14
		 *   AXI[30]     rank -- a DMC_DDR_CTRL range decode, NOT a
		 *               field here.  See meson_dram_dmc_set_size().
		 *
		 * The map is therefore the SAME for one rank and for two; only
		 * the DMC range decode differs.  The five words below encode
		 * exactly the table above, and their hex is written beside each
		 * one so a live readback can be diffed without re-encoding:
		 *
		 *   0x14941cc5 0x2307800b 0x2f6ad272 0x3bcdeb38 0x3e07359f
		 *
		 * ADDRMAP_2 and ADDRMAP_3 are byte-identical to the vendor GXL
		 * DDR4 words the arm below programs -- the row half of this map
		 * IS the vendor's placement on this controller.  ADDRMAP_1
		 * differs from the vendor DDR4 word by exactly one bit position
		 * (ca9 <- AXI[11] not AXI[12]), which is the shift a 10-column
		 * part makes against an 11-column one.
		 *
		 * HISTORY, so nobody restores it: from e20f62757d5 until this
		 * commit these writes mapped ca10 -> AXI[24], ca11 -> AXI[25],
		 * ra15 -> AXI[29] and tied ba1/ba2 to AXI[0].  That left 2 of 8
		 * banks and a 256 MB store folded eight ways across the 2 GB
		 * window (A and B are the same cell iff (A ^ B) & ~0x23000000
		 * == 0), while the DMC still reported and offered 2 GB.  The
		 * text you are reading described the corrected map for some
		 * time while the code below still wrote the defective one; if
		 * you change one, change the other in the same commit.
		 *
		 * NOT VERIFIED ON HARDWARE at this commit.  The corrected words
		 * were silicon-verified on 2026-08-08 on a DIFFERENT tree
		 * (v2026.07/lc-gxl-dram-alias, a2bd7a4121d / 29440cd1af5); no
		 * DDR3 board has booted this branch with them, because the only
		 * AML-S905X-CC (DDR3) rig is physically detached.  The one live
		 * GXL rig is AML-S905X-CC-V2, which is DDR4 and does not reach
		 * this arm.
		 */
		if (IS_ENABLED(CONFIG_DRAM_TWO_IDENTICAL_RANKS) ||
		    IS_ENABLED(CONFIG_DRAM_ONE_RANK)) {
			writel( 5 |  6 << 5 |  7 << 10 |  8 << 15 |  9 << 20 | 10 << 25,
			       DDR0_ADDRMAP_0);	/* 0x14941cc5 */
			/*
			 * ca10 = 0 IS LOAD-BEARING.  Read this before repointing it.
			 *
			 * DDR3 A10 is the auto-precharge flag on a column command, and
			 * a 4 Gbit x16 part has columns A0-A9 only -- so this field is
			 * PURELY the AP selector with no cell behind it.  Tying it to
			 * request-address bit 0 makes it constant zero at command
			 * granularity, so no read/write-with-auto-precharge is ever
			 * issued and the DRAM never enforces tDAL = WR + tRP.
			 *
			 * The map that shipped before had ca10 -> AXI[24], which would
			 * have asserted auto-precharge on roughly half of all accesses.
			 * Together with the MR0 Write Recovery defect that made a
			 * latent JEDEC violation a live one; a2bd7a4121d fixed the map
			 * for the aliasing fold and, unrecognised at the time, turned
			 * auto-precharge off as a side effect.
			 *
			 * MR0 is now encoded correctly (PUB_MR0_WR_CODE in
			 * dram-settings-gx.h), so this is no longer a correctness
			 * dependency -- it is the explanation for why auto-precharge is
			 * off, which is still worth knowing.  Repointing ca10 at a live
			 * address bit remains a deliberate decision with a tDAL
			 * consequence, not a map detail.
			 */
			writel(11 |  0 << 5 |  0 << 10 | 15 << 15 | 16 << 20 | 17 << 25,
			       DDR0_ADDRMAP_1);	/* 0x2307800b */
			writel(18 | 19 << 5 | 20 << 10 | 21 << 15 | 22 << 20 | 23 << 25,
			       DDR0_ADDRMAP_2);	/* 0x2f6ad272 */
			writel(24 | 25 << 5 | 26 << 10 | 27 << 15 | 28 << 20 | 29 << 25,
			       DDR0_ADDRMAP_3);	/* 0x3bcdeb38 */
			writel(31 | 12 << 5 | 13 << 10 | 14 << 15 |  0 << 20 | 31 << 25,
			       DDR0_ADDRMAP_4);	/* 0x3e07359f */

			writel( 5 |  6 << 5 |  7 << 10 |  8 << 15 |  9 << 20 | 10 << 25,
			       DDR1_ADDRMAP_0);	/* 0x14941cc5 */
			writel(11 |  0 << 5 |  0 << 10 | 15 << 15 | 16 << 20 | 17 << 25,
			       DDR1_ADDRMAP_1);	/* 0x2307800b */
			writel(18 | 19 << 5 | 20 << 10 | 21 << 15 | 22 << 20 | 23 << 25,
			       DDR1_ADDRMAP_2);	/* 0x2f6ad272 */
			writel(24 | 25 << 5 | 26 << 10 | 27 << 15 | 28 << 20 | 29 << 25,
			       DDR1_ADDRMAP_3);	/* 0x3bcdeb38 */
			writel(31 | 12 << 5 | 13 << 10 | 14 << 15 |  0 << 20 | 31 << 25,
			       DDR1_ADDRMAP_4);	/* 0x3e07359f */
		}
	} else if (IS_ENABLED(CONFIG_MESON_GXL) && IS_ENABLED(CONFIG_DRAM_DDR4)) {
		/*
		 * GXL + DDR4, 32-bit bus, ONE_RANK / TWO_IDENTICAL (Sweet Potato).
		 *
		 * Vendor gxl_bl2_ddr_deep.c for ddrchannelset RANK0/RANK01 + DDR4
		 * programs these five words (not the DDR3 0x14941cc5 map):
		 *   0x16a4a0e6 0x2307800c 0x2f6ad272 0x3bcdeb38 0x3e02b9be
		 * Decode: ca3..ca8=AXI[6..11], ca9=12, park ca10/ca11, ra0..ra14=
		 * AXI[15..29], ra15=AXI[30], ba0/1=13/14, ba2=5, park bg1/ra16.
		 * 16 row + 10 column + 3 bank bits on a 32-bit bus = 2 GB, which is
		 * what the board carries: two SK hynix H5AN8G6NAFR-UHC (8 Gbit,
		 * 512M x16, FBGA96) on one chip select, read off the AML-S905X-CC-V2
		 * V2.0-C schematic -- 2 x 8 Gbit = 2 GB, one rank, 32-bit bus.
		 * The map word is read back byte-identical off a product-initialised
		 * board, so this is the geometry vendor BL2 runs too.
		 * 890246's DDR3-shaped map + DMC_SIZE 1 GB left a size residual
		 * after train-green.
		 *
		 * LaFrite 16-bit stays on DRAM_16BIT_RANK below.
		 */
		if (IS_ENABLED(CONFIG_DRAM_TWO_IDENTICAL_RANKS) ||
		    IS_ENABLED(CONFIG_DRAM_ONE_RANK)) {
			writel(0x16a4a0e6, DDR0_ADDRMAP_0);
			writel(0x2307800c, DDR0_ADDRMAP_1);
			writel(0x2f6ad272, DDR0_ADDRMAP_2);
			writel(0x3bcdeb38, DDR0_ADDRMAP_3);
			writel(0x3e02b9be, DDR0_ADDRMAP_4);

			writel(0x16a4a0e6, DDR1_ADDRMAP_0);
			writel(0x2307800c, DDR1_ADDRMAP_1);
			writel(0x2f6ad272, DDR1_ADDRMAP_2);
			writel(0x3bcdeb38, DDR1_ADDRMAP_3);
			writel(0x3e02b9be, DDR1_ADDRMAP_4);
		} else if (IS_ENABLED(CONFIG_DRAM_16BIT_RANK)) {
			writel(0 | 6 << 5  |  7 << 10 |  8 << 15 |  9 << 20 | 10 << 25,
			       DDR0_ADDRMAP_0);
			writel(11 | 0 << 5  |  0 << 10 | 14 << 15 | 15 << 20 | 16 << 25,
			       DDR0_ADDRMAP_1);
			writel(17 | 18 << 5 | 19 << 10 | 20 << 15 | 21 << 20 | 22 << 25,
			       DDR0_ADDRMAP_2);
			writel(23 | 24 << 5 | 25 << 10 | 26 << 15 | 27 << 20 | 28 << 25,
			       DDR0_ADDRMAP_3);
			writel(29 | 12 << 5 | 13 << 10 |  5 << 15 |  0 << 20 | 30 << 25,
			       DDR0_ADDRMAP_4);

			writel(0 |  6 << 5 |  7 << 10 |  8 << 15 |  9 << 20 | 10 << 25,
			       DDR1_ADDRMAP_0);
			writel(11 |  0 << 5 |  0 << 10 | 14 << 15 | 15 << 20 | 16 << 25,
			       DDR1_ADDRMAP_1);
			writel(17 | 18 << 5 | 19 << 10 | 20 << 15 | 21 << 20 | 22 << 25,
			       DDR1_ADDRMAP_2);
			writel(23 | 24 << 5 | 25 << 10 | 26 << 15 | 27 << 20 | 28 << 25,
			       DDR1_ADDRMAP_3);
			writel(29 | 12 << 5 | 13 << 10 |  5 << 15 |  0 << 20 | 30 << 25,
			       DDR1_ADDRMAP_4);
		}
	}
}

/*
 * How much one rank can address under the map meson_dram_dmc_set_addrmap()
 * programs.  This is a property of the MAP, not a guess at the part: the DMC
 * range decode must never offer an address the map cannot reach.
 *
 * The GXL + DDR3 map is 15 row + 10 column + 3 bank bits on a 32-bit bus
 * = 1 GB per rank, two ranks.  The GXL + DDR4 ONE_RANK map is the vendor
 * word set, 16 row + 10 column + 3 bank bits = 2 GB in ONE rank.  Change a
 * map's row count and this must change with it.  The 2048 that used to be
 * hard-coded here was written into BOTH rank fields, so the DMC was told
 * rank 0 and rank 1 were 2 GB each on a part with two 1 GB ranks — and DDR4
 * ONE_RANK still fell through to 2048 after a2bd, leaving detect probing
 * outside the programmed window (128 MB residual on train-green).
 *
 * The DDR4 1024 that replaced it was WRONG, and it cost the upper half of
 * the board's RAM: AML-S905X-CC-V2 fits two H5AN8G6NAFR-UHC (8 Gbit each)
 * = 2 GB, product BL2 prints "Rank0: 2048MB(auto)" and leaves
 * DMC_DDR_CTRL = 0x00500064 (rank 0 size field 4 = 2 GB, rank 1 disabled),
 * and a host-side write/read-back over BROM on a product-initialised board
 * holds distinct data at four addresses and their +1 GB partners plus
 * 0x7ffffff0.  Measured 2026-08-10.
 *
 * The 16-bit arm is FIRST, and the order is the whole point.  AML-S805X-AC
 * (La Frite) is MESON_GXL + DRAM_DDR4 + DRAM_16BIT_RANK, so the generic
 * GXL+DDR4 arm below matches it too -- and being written first it used to
 * win, handing a single-chip 16-bit board the 32-bit board's 2 GB window
 * while the DRAM_16BIT_RANK arm sat unreachable for every DDR4 build.  That
 * was invisible only because no 16-bit DDR4 defconfig existed to compile it;
 * libretech-ac_libre_defconfig is the first, so it is live now.  Per the
 * paragraph above, this constant is a property of the MAP: the 16BIT_RANK
 * map meson_dram_dmc_set_addrmap() programs is one rank on a 16-bit bus,
 * meson_dram_detect_max_mb() is derived from it, so a 2048 here would send
 * detection at an address no rank decodes -- which on this DMC does not read
 * back as garbage, it never completes, and dram_init() dies before its size
 * print (the exact failure the DDR3 arm's comment records).
 */
#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_16BIT_RANK)
#define MESON_DRAM_RANK0_MB	1024
#elif defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR4)
#define MESON_DRAM_RANK0_MB	2048
#elif defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR3)
/*
 * 1 GB, because that is what the corrected map above reaches: 15 row + 10
 * column + 3 bank bits on a 32-bit bus.  This USED to be 2048, deliberately
 * left "as it shipped" on the reasoning that the address map was wrong
 * independently of any size constant, so picking a different number would not
 * be an improvement.  That reasoning expired with the map fix -- 2048 here
 * would offer a 2 GB rank-0 range to a map that decodes 1 GB, folding the
 * upper half onto the lower on AXI[30] and leaving a 2x alias where the old
 * map had an 8x one.
 *
 * The 2026-08-10 measurements that motivated the "left as it shipped" note
 * were all taken under the DEFECTIVE map and do not carry over: "rank0=1024
 * with NO rank 1 boots and still fails 895 of 1535 checks over [1 MB, 1 GB)"
 * is the old map's 128 MB unique window, not this one's, and "rank0=1024 +
 * rank1=1024 trains green then HANGS" is the rank-1 enable below, which
 * ONE_RANK no longer sets.  1 GB + no rank 1 is the combination that was
 * silicon-verified on 2026-08-08 (v2026.07/lc-gxl-dram-alias, 29440cd1af5:
 * every probe 0 bad over 0-1 GB, canary held on all eight segments) -- on a
 * different tree, and NOT re-verified here.
 */
#define MESON_DRAM_RANK0_MB	1024
#elif defined(CONFIG_DRAM_TWO_DIFF_RANKS)
/*
 * GXBB only -- DRAM_16BIT_RANK used to be OR'd in here and is not any more,
 * because Kconfig offers that symbol under MESON_GXL alone, so the GXL 16-bit
 * arm at the top of this chain now takes every build that could have reached
 * this clause.  Leaving it would have been a second answer to the same
 * question, in the file where getting this constant wrong costs the board's
 * upper memory or a hang.
 */
#define MESON_DRAM_RANK0_MB	1024
#else
#define MESON_DRAM_RANK0_MB	2048
#endif

/*
 * Rank 1 is enabled by whether the PHY was told to TRAIN a rank 1, not by what
 * the part carries.  meson_dram_phy_setup_ranks() trains rank 0 only under
 * CONFIG_DRAM_ONE_RANK, so a DMC rank-1 range on a ONE_RANK build offers an
 * untrained rank -- measured as a hang before the size print.  The GXL+DDR3
 * arm used to be forced on here, which is exactly that case: Le Potato's
 * libretech-cc_libre_defconfig is ONE_RANK.  A ONE_RANK build of a two-rank
 * board therefore reports 1 GB of its 2 GB; that is the honest number for what
 * has been trained, not a size regression to chase.
 */
#if defined(CONFIG_DRAM_TWO_IDENTICAL_RANKS)
#define MESON_DRAM_RANK1_MB	MESON_DRAM_RANK0_MB
#elif defined(CONFIG_DRAM_TWO_DIFF_RANKS)
#define MESON_DRAM_RANK1_MB	4096
#else
#define MESON_DRAM_RANK1_MB	0	/* one rank: no second range at all */
#endif

/*
 * How many ranks THIS attempt is training, as opposed to how many the build
 * declares.  One SKU, one config: AML-S905X-CC ships as both a 1 GB one-rank
 * and a 2 GB two-rank board, and a separate config per rank count is the wrong
 * shape -- the board can say which it is, so it should.
 *
 * The build declares the MAXIMUM (MESON_DRAM_RANK1_MB above); this says what
 * the current dram_init() attempt is actually using.  The ladder starts at the
 * maximum and steps down to one rank if training FAILS -- and only if it
 * fails.  A successful two-rank training that then detects 1024 MB is NOT a
 * fallback trigger: detection is already honest about size, and second-guessing
 * it here would re-introduce exactly the guessing that meson_dram_detect_size()
 * refuses to do.
 *
 * Assigned at entry, not merely declared: SPL BSS on this path is not zeroed.
 */
bool meson_dram_rank1_active;

/* MB behind rank 1 for THIS attempt: the declared maximum, or none. */
static inline u32 meson_dram_rank1_mb(void)
{
	return meson_dram_rank1_active ? MESON_DRAM_RANK1_MB : 0;
}

/*
 * Size field, enumerated exactly as the vendor register header does rather
 * than computed, so it is a constant expression and the whole DMC_DDR_CTRL
 * word folds to one immediate.
 */
#define DMC_SIZE_FIELD(mb)	((mb) == 128 ? 0 : (mb) == 256 ? 1 :	\
				 (mb) == 512 ? 2 : (mb) == 1024 ? 3 :	\
				 (mb) == 2048 ? 4 : 5)

/*
 * Rank 1 is selected by a RANGE DECODE, not an address bit: rank 0 owns
 * [0, RANK0_MB) and rank 1 owns [RANK0_MB, RANK0_MB + RANK1_MB).  A one-rank
 * part gets NO rank 1 bits at all -- what made "one rank" enable rank 1 was
 * bit 21 unconditionally set alongside a size in [5:3].
 */
#if MESON_DRAM_RANK1_MB
#define DMC_CTRL_RANK1	(DMC_CTRL_RANK1_EN |				\
	(DMC_SIZE_FIELD(MESON_DRAM_RANK1_MB) << DMC_CTRL_RANK1_SHIFT))
#else
#define DMC_CTRL_RANK1	0
#endif

/*
 * The same word for THIS attempt.  A one-rank attempt must offer no rank-1
 * range at all: a DMC rank-1 range over an untrained rank is the hang this
 * file's rank comment describes, so the range and the DTCR1 training bit have
 * to move together or not at all.
 */
static inline u32 meson_dram_dmc_ctrl_rank1(void)
{
	return meson_dram_rank1_active ? DMC_CTRL_RANK1 : 0;
}

#if defined(CONFIG_DRAM_16BIT_RANK)
#define DMC_CTRL_WIDTH	DMC_CTRL_16BIT
#else
#define DMC_CTRL_WIDTH	0
#endif

/*
 * The highest address meson_dram_detect_size() may touch.
 *
 * This MUST NOT exceed the window meson_dram_dmc_init() programmed.  An
 * address that no rank decodes -- or one inside a rank the PHY has not trained
 * -- does not read back as garbage on this DMC: the transaction does not
 * complete.  dram_init() then dies before its own size print, so the console
 * shows the "DMC version" line and nothing after it, and the SPL watchdog
 * reboots the board into BROM.  Measured on AML-S905X-CC 2026-08-08 with rank 0
 * correctly sized to 1 GB and a hard-coded 2048 MB first probe (0x7ffffffc).
 *
 * Derived from the rank sizes, unconditionally, because that is the SAME
 * expression meson_dram_dmc_init() writes into DMC_DDR_CTRL a few lines below
 * -- DMC_SIZE_FIELD(MESON_DRAM_RANK0_MB) | meson_dram_dmc_ctrl_rank1() -- so
 * the ceiling and the window it must not exceed have one source, not two that
 * can drift.
 *
 * Until N18 (2026-08-29) only the GXL+DDR4 arm derived it and the DDR3 arm
 * kept the flat 2048, which on the shipping libretech-cc_libre_defconfig
 * (ONE_RANK: RANK0_MB 1024, RANK1_MB 0) made the first probe 0x7ffffffc --
 * exactly the measured case the paragraph above records.  The DDR3 window
 * derived here is the same 1 GB the alias instrument booted with on this
 * board (`detect<=1024MB` / `DDR: 1024 MB`, s7-nyc corpus, 2026-08-25).
 *
 * N18 left a non-GXL `#else` holding a flat 2048.  It was DEAD, not merely
 * unexercised: this file is obj-$(CONFIG_MESON_GX) *inside* the Makefile's
 * SPL/TPL guard, and generating .config for all 79 in-tree CONFIG_ARCH_MESON=y
 * defconfigs finds 25 with CONFIG_MESON_GX=y of which exactly three set
 * CONFIG_SPL/CONFIG_TPL -- libretech-{ac,cc,cc_v2}_libre -- and all three are
 * MESON_GXL.  Every GXBB and GXM defconfig (odroid-c2, nanopi-k2, p200, p201,
 * wetek-hub, wetek-play2, videostrong-kii-pro, khadas-vim2, wetek-core2,
 * beelink-gt1-ultimate, libretech-s912-pc) is SPL-less, so dram-gx.c is never
 * compiled for them.  Deleting the arm rather than commenting it is what makes
 * that finding load-bearing: it is bit-identical for all three built images
 * (verified), and a future GXBB SPL then inherits a ceiling that tracks its own
 * DMC window instead of a stale constant -- which for GXBB
 * DRAM_TWO_IDENTICAL_RANKS (RANK0 2048 + RANK1 2048) would have hidden half the
 * board's RAM behind a 2048 ceiling.  An #error here was the other option and is
 * worse: it would convert a future GXBB bring-up into a build failure over a
 * value that is now correct by construction.
 *
 * It is a function and not a macro for the same reason: the ceiling must track
 * the rank count of THIS attempt, not the build's declared maximum.  The ladder
 * in dram_init() steps down to one rank, and probing the rank-1 range after that
 * step is the "address no rank answers" case above -- the access does not
 * complete and the watchdog reboots instead of anything being reported.  A
 * build-time DDR_DETECT_MAX_MB used to sit here alongside it; it was the
 * non-GXL arm's only consumer and is gone with it, so there is now exactly one
 * expression for this ceiling in the file.
 */
static inline u32 meson_dram_detect_max_mb(void)
{
	return MESON_DRAM_RANK0_MB + meson_dram_rank1_mb();
}

void meson_dram_dmc_init(void)
{


	/*
	 * Open exactly the window the address map can reach -- no more, so
	 * detection has no unreachable address to mistake for a real cell,
	 * and no less, so it can see every rank that is fitted.
	 *
	 * GXL+DDR4 RANK0 vendor BL2 seeds DMC_DDR_CTRL with 0x6d|0x500000
	 * (max size fields, channel, DDR4+BG1; no RANK1_EN) then detect
	 * rewrites the size fields. Seed the same; still size RANK0 after.
	 */
#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR4) && \
    defined(CONFIG_DRAM_ONE_RANK)
	/*
	 * Product seeds 0x50006d (max) then rewrites after bist. Post-train
	 * rewrite to 2048 alone banked no residual win (b0ed7bd). NEW:
	 * seed product final Rank0 2048 value 0x500044 through train.
	 */
	writel(0x50006d, DMC_DDR_CTRL);
#else
	writel(DMC_CTRL_CHANNEL | DMC_CTRL_DDR_TYPE | DMC_CTRL_WIDTH |
	       DMC_SIZE_FIELD(MESON_DRAM_RANK0_MB) | meson_dram_dmc_ctrl_rank1(),
	       DMC_DDR_CTRL);
#endif

	meson_dram_dmc_set_addrmap();

	/*
	 * Vendor ddr_init_dmc programs REFR (DMC_REFR_CTRL*) *before* SEC/REQ and
	 * before PCTL GO. Open only wrote REFR in finalise_init after GO — leave
	 * finalise write too (re-assert); first program here.
	 */
#if defined(CONFIG_DRAM_DDR4)
	writel(0x20100000 | ((meson_dram_clk / 20) - 1) |
	       (timings.refi << 8), DMC_REFR_CTRL2);
	writel(0xf08f, DMC_REFR_CTRL1);
#endif

	if (IS_ENABLED(CONFIG_MESON_GXBB)) {
		writel(0x440620, DMC_PCTL_LP_CTRL);
		writel(BIT(13) | BIT(5), DDR0_APD_CTRL);
		writel(0x5, DDR0_CLK_CTRL);

		writel(0x11, DMC_AXI0_QOS_CTRL1);
	} else if (IS_ENABLED(CONFIG_MESON_GXL)) {
		/*
		 * Vendor ddr_init_dmc always writes APD_CTRL=0x2000
		 * (DDR0_APD_CTRL, DDR_MMC_BASE + 0x24 = 0xc8837024) after
		 * the map/SEC path — open had 0.
		 */
#if defined(CONFIG_DRAM_DDR4)
		writel(0x2000, DDR0_APD_CTRL);
#else
		writel(BIT(13), DDR0_APD_CTRL);
#endif
	}

	/*
	 * Full vendor SEC block after map (gxl_bl2_ddr_deep ddr_init_dmc),
	 * register-for-register — residual got0=0xffffffff with map OK /
	 * PCTL ACCESS / REQ=ffff means open-bus; ENABLE_REGION may not hit
	 * the same 11x/12x/13x sequence.  DES keys cleared; DES_CTRL left 0
	 * (DES=2 hung pre-FIN).  REQ last.
	 */
	writel(0x0, DMC_SEC_RANGE_CTRL);		/* SEC_107 */
	/*
	 * DRAM-type independent, deliberately -- see the long note on the value
	 * below.  This used to sit inside the DDR4 arm, and the DDR3 arm wrote
	 * the register nowhere, so a DDR3 board ran with whatever the BootROM
	 * left.  Measured on AML-S905X-CC v1 (DDR3): 0x00000000, i.e. no port
	 * may issue anything, against 0xffffffff on its DDR4 sibling -- and the
	 * v1 board went silent at the ERET to BL33 on a chain whose BL31, being
	 * secure and MMU-on, never touched a denied port.  Who owns the DMC port
	 * permissions has nothing to do with which DRAM type is fitted.
	 */
	writel(0xffffffff, DMC_SEC_AXI_PORT_CTRL);
#if defined(CONFIG_DRAM_DDR4)
	/*
	 * DES off for pure DTDR residual lever (this tip).
	 * Prior: SEC+DES unit e95a97393d0 residual got0=253368af DES class (no win).
	 * DES=2 confounds residual measure; banked as residual class change only.
	 * SEC_CTRL write non-sticky (read 0). Keep DES=0 zero keys for DTDR-alone.
	 */
	/*
	 * DMC_SEC_CTRL (DMC_SEC_REG_BASE + 0x00): bit 31 COMMITS the security
	 * configuration (the RANGEn windows, DMC_SEC_RANGE_CTRL levels and the
	 * per-master grants) -- dram-gx.h, above DMC_SEC_RANGE0_CTRL.  The
	 * DDR3 arm writes BIT(31); this arm wrote 0 since the DDR4 bring-up
	 * base, which never commits, and "reads 0" fits a self-clearing commit
	 * strobe rather than a lost write.  Bits [30:0] stay 0: DES off.
	 */
	writel(BIT(31), DMC_SEC_CTRL);
	/*
	 * DMC_SEC_AXI_PORT_CTRL is a 2-bit-per-port permission field, and the
	 * value here used to be 0x55555555 -- 0b01 in EVERY port, i.e. SECURE
	 * ACCESSES ONLY on all sixteen.  That is not what product programs, and
	 * it is not survivable: it makes DRAM unreadable to any non-secure
	 * access, so the first NS load BL31 issues (aml_scpi_copy_scp_data()
	 * memcpy'ing BL301 out of 0x013CA000, which BL31 maps MT_NS) takes a
	 * synchronous EXTERNAL ABORT -- ESR_EL3 0x96000010, FAR_EL3 0x013CA000,
	 * measured on AML-S905X-CC-V2 with TF-A CRASH_REPORTING=1.  Release TF-A
	 * builds have CRASH_REPORTING=0, so it presented as a silent hang and was
	 * mis-read for two sessions as "the SCP upload wedges the AP".  The SPL's
	 * own DDR memtest never saw it because the SPL runs MMU-off at EL3, where
	 * every access is secure and 0b01 permits it.
	 *
	 * That EXAMPLE is now historical, and the register write is NOT.  Under
	 * CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT the SCP staging buffer moved to
	 * 0x001C0000, inside MAP_SEC_SCP_STAGE, so BL31 reads it MT_SECURE and
	 * that particular abort cannot recur.  Nothing else changes: u-boot proper
	 * runs at EL2 NON-SECURE and Linux at EL1 non-secure, so all-0b01 would
	 * still deny DRAM to every stage after BL31.  The measurement is kept
	 * because it is the causal evidence for what the field means, not because
	 * the SCP memcpy is still the thing that trips it.
	 *
	 * The write itself now happens above, unconditionally, for both DRAM
	 * types; this note stays here because it is where the value's evidence
	 * was recorded.
	 *
	 * 0x55555555 was an RE transcription slip -- it is the value product
	 * writes to the VPU/VDEC port registers a few instructions later.  The
	 * constant product writes to THIS register, read out of this board's own
	 * signed BL2 (vendor/amlogic/blx/aml-s905x-cc-v2/bl2_new.bin, Thumb-2,
	 * literal pool at file offset 0x664c/0x6650):
	 *
	 *   65f8: ldr r2,[pc,#80]   @ 0x664c = 0x1cd51cd5
	 *   65fa: ldr r3,[pc,#84]   @ 0x6650 = 0xda838438  (DMC_SEC_AXI_PORT_CTRL)
	 *   65fe: str r2,[r3,#0]
	 *
	 * is NOT adopted, and the reason is measured, not assumed: 0x1cd51cd5
	 * leaves four fields at 0b01 and four at 0b00, and booting it on this
	 * chain moved the failure EARLIER -- BL31 died before the first SCPI SHA
	 * instead of after it -- so its per-port assignment does not match what
	 * this chain's requestors need.  Product's DMC is configured by a
	 * different sequence than ours and its port map is not established here.
	 *
	 * What IS established, causally, on hardware: all-0b01 denies non-secure
	 * DRAM and all-0b11 permits it (BL31 read 0x013CA000 = 0xb810f000 and
	 * 0x01000000 = 0x1400000a immediately after this register was set to
	 * 0xffffffff, having aborted on the identical read moments before).  The
	 * open chain runs no BL32/TEE and enforces no secure DRAM carve-out, so
	 * "every port may issue both secure and non-secure accesses" is the
	 * policy it actually wants, and 0xffffffff states exactly that.
	 */
	writel(0x55555555, DMC_DEV_SEC_READ_CTRL);
	writel(0x55555555, DMC_DEV_SEC_WRITE_CTRL);
	writel(0x15, DMC_GE2D_SEC_CTRL);
	writel(0x5, DMC_PARSER_SEC_CTRL);
	writel(0xffffffff, DMC_VPU_SEC_CFG);
	writel(0x55555555, DMC_VPU_SEC_WRITE_CTRL);
	writel(0x55555555, DMC_VPU_SEC_READ_CTRL);
	writel(0xffffffff, DMC_VDEC_SEC_CFG);
	writel(0x55555555, DMC_VDEC_SEC_WRITE_CTRL);
	writel(0x55555555, DMC_VDEC_SEC_READ_CTRL);
	writel(0xffffffff, DMC_HCODEC_SEC_CFG);
	writel(0x55555555, DMC_HCODEC_SEC_WRITE_CTRL);
	writel(0x55555555, DMC_HCODEC_SEC_READ_CTRL);
	writel(0xffffffff, DMC_HEVC_SEC_CFG);
	writel(0x55555555, DMC_HEVC_SEC_WRITE_CTRL);
	writel(0x55555555, DMC_HEVC_SEC_READ_CTRL);
	writel(0, DMC_DES_KEY0_H);
	writel(0, DMC_DES_KEY0_L);
	writel(0, DMC_DES_KEY1_H);
	writel(0, DMC_DES_KEY1_L);
	writel(0, DMC_DES_CTRL);
#else
	writel(0x80000000, DMC_SEC_CTRL);
	/*
	 * Grant the DEVICE AXI master port (sd_emmc_a/b/c, NAND, SDIO) both
	 * read and write access to non-secure DRAM.  The DDR4 arm above does
	 * this; the DDR3 arm did not, so on a DDR3 open-chain board the eMMC
	 * controller's first DMA data transfer (CMD8 SEND_EXT_CSD) was denied
	 * and completed with SD_EMMC_STATUS DESC_ERR|DESC_TIMEOUT (0 bytes),
	 * surfacing as "mmc0: error -84 whilst initialising MMC card".
	 * DMC_SEC_AXI_PORT_CTRL (0xffffffff, shared above) covers only the
	 * numbered AXI ports, NOT the aggregated device port -- that is gated
	 * here.  0x55555555 = non-secure read+write enable per port, matching
	 * the vendor ddr_init_dmc value and the DDR4 arm.
	 */
	writel(0x55555555, DMC_DEV_SEC_READ_CTRL);
	writel(0x55555555, DMC_DEV_SEC_WRITE_CTRL);
	/*
	 * Same DDR4-only-grant pattern as DMC_DEV above: the GE2D (2D blit)
	 * and PARSER (video ES/stream parser -- meson-vdec's compressed-stream
	 * input DMA master) ports are granted in the DDR4 arm and the vendor
	 * ddr_init_dmc, but were absent from this DDR3 arm.  Without the PARSER
	 * grant HW video decode's esparser input DMA is denied on a DDR3
	 * open-chain board, exactly as the eMMC's data DMA was.  Values mirror
	 * the DDR4 arm.
	 */
	writel(0x15, DMC_GE2D_SEC_CTRL);
	writel(0x5, DMC_PARSER_SEC_CTRL);
	DMC_ENABLE_REGION(DMC_VPU);
	DMC_ENABLE_REGION(DMC_VDEC);
	DMC_ENABLE_REGION(DMC_HCODEC);
	DMC_ENABLE_REGION(DMC_HEVC);
#endif
	writel(0xffff, DMC_REQ_CTRL);
	dmb();
	isb();

#if IS_ENABLED(CONFIG_MESON_DRAM_ALIAS_TEST) || \
    IS_ENABLED(CONFIG_MESON_DRAM_FAULT_CLASS) || \
    IS_ENABLED(CONFIG_MESON_GX_DMC_DUMP)
	/*
	 * Print the programmed geometry BEFORE the first DRAM access, not
	 * after.  The alias test's register decode and the classifier's S0
	 * dump both run from dram_init() *after* meson_dram_detect_size(), so
	 * a probe that never completes takes the register dump down with it
	 * and the run yields no measurement at all -- which is exactly what
	 * both diagnostic images did on 2026-08-08.  A diagnostic whose output
	 * is downstream of the failure it is meant to characterise is not a
	 * diagnostic.  Costs nothing in a shipping SPL: no diagnostic, no
	 * print.
	 */
	printf("DMC: ctrl=%08x map=%08x %08x %08x %08x %08x detect<=%uMB\n",
	       readl(DMC_DDR_CTRL), readl(DDR0_ADDRMAP_0),
	       readl(DDR0_ADDRMAP_1), readl(DDR0_ADDRMAP_2),
	       readl(DDR0_ADDRMAP_3), readl(DDR0_ADDRMAP_4),
	       meson_dram_detect_max_mb());
#endif

	debug("dram: memory controller init done\n");
}

/*
 * Auto-calibration: retry training with PHY reset on failure.
 * DDR training is sensitive to PVT (process/voltage/temperature) and
 * may need multiple attempts. The closed BL2 also retries.
 */
#define DDR_TRAINING_MAX_RETRIES	3

static int meson_dram_do_init(void)
{
	int ret;

	ret = meson_dram_pll_init();
	if (ret < 0)
		return ret;
	meson_dram_phy_prepare();
	/*
	 * These two used to be `void` and to spin forever inside on a PHY that
	 * did not answer.  They are the two stages dram_init()'s third attempt
	 * wedged in (N27, n27a/n27c) -- so a ladder that cannot hear them
	 * cannot report that it exhausted.  Refuse early instead: an attempt
	 * that could not get the PHY through its PIR sequence has nothing to
	 * tell meson_dram_phy_finalise_init().
	 */
	ret = meson_dram_phy_init();
	if (ret < 0)
		return ret;
	ret = meson_dram_prepare_pctl();
	if (ret < 0)
		return ret;
	meson_dram_set_memory_timings();
	meson_dram_set_dfi_timings();
	ret = meson_dram_phy_finalise_init();
	if (ret < 0)
		return ret;
	meson_dram_phy_setup_ranks();
	/*
	 * Vendor ddr_init_all: ddr_init_pctl (train) then ddr_init_dmc (map/SEC/
	 * REQ + REFR) then detect.  Open ran finalise (PCTL GO + REFR) BEFORE
	 * dmc_init, so ACCESS was entered without the DMC map/ports programmed.
	 * DDR4-scoped: the order change was made for the DDR4 bring-up and has
	 * never been measured on a DDR3 board, which keeps the old order below.
	 */
#if defined(CONFIG_DRAM_DDR4)
	meson_dram_dmc_init();
#endif
	return meson_dram_finalise_init();
}

/*
 * DDR bandwidth benchmark.  Measures sequential read and write throughput
 * using 1MB blocks.  Reports MB/s based on the ARM generic timer.
 */
#define BENCH_SIZE	(1 << 20)	/* 1MB per pass */
#define BENCH_BASE	(CFG_SYS_SDRAM_BASE + 0x2000000)	/* 32MB offset */

static inline u64 read_cntpct(void)
{
	u64 val;

	asm volatile("mrs %0, cntpct_el0" : "=r" (val));
	return val;
}

static inline u32 read_cntfrq(void)
{
	u32 val;

	asm volatile("mrs %0, cntfrq_el0" : "=r" (val));
	/* GXL crystal is 24MHz; CNTFRQ may not be set in SPL context */
	if (!val)
		val = 24000000;
	return val;
}

/*
 * Assembly bandwidth primitives using STP/LDP for maximum throughput.
 * Each iteration moves 64 bytes (one cache line on A53).
 */
void bench_write_64(void *dst, u32 bytes);
void bench_read_64(void *src, u32 bytes);
void bench_copy_64(void *dst, void *src, u32 bytes);
void noinline bench_write_64(void *dst, u32 bytes)
{
	asm volatile(
		"1: stp xzr, xzr, [%0]\n"
		"   stp xzr, xzr, [%0, #16]\n"
		"   stp xzr, xzr, [%0, #32]\n"
		"   stp xzr, xzr, [%0, #48]\n"
		"   add %0, %0, #64\n"
		"   subs %w1, %w1, #64\n"
		"   bgt 1b\n"
		: "+r"(dst), "+r"(bytes) :: "memory"
	);
}

void noinline bench_read_64(void *src, u32 bytes)
{
	asm volatile(
		"1: ldp x2, x3, [%0]\n"
		"   ldp x4, x5, [%0, #16]\n"
		"   ldp x2, x3, [%0, #32]\n"
		"   ldp x4, x5, [%0, #48]\n"
		"   add %0, %0, #64\n"
		"   subs %w1, %w1, #64\n"
		"   bgt 1b\n"
		: "+r"(src), "+r"(bytes) :: "x2", "x3", "x4", "x5", "memory"
	);
}

void noinline bench_copy_64(void *dst, void *src, u32 bytes)
{
	asm volatile(
		"1: ldp x2, x3, [%1]\n"
		"   ldp x4, x5, [%1, #16]\n"
		"   stp x2, x3, [%0]\n"
		"   stp x4, x5, [%0, #16]\n"
		"   ldp x2, x3, [%1, #32]\n"
		"   ldp x4, x5, [%1, #48]\n"
		"   stp x2, x3, [%0, #32]\n"
		"   stp x4, x5, [%0, #48]\n"
		"   add %0, %0, #64\n"
		"   add %1, %1, #64\n"
		"   subs %w2, %w2, #64\n"
		"   bgt 1b\n"
		: "+r"(dst), "+r"(src), "+r"(bytes)
		:: "x2", "x3", "x4", "x5", "memory"
	);
}

static void meson_dram_bench_size(u32 sz, u32 freq)
{
	void *buf1 = (void *)BENCH_BASE;
	void *buf2 = (void *)(BENCH_BASE + sz);
	u64 t0, t1;
	u32 tw, tr, tc;

	/* Write: STP xzr */
	t0 = read_cntpct();
	bench_write_64(buf1, sz);
	dsb();
	t1 = read_cntpct();
	tw = (u32)(t1 - t0);

	/* Read: LDP */
	t0 = read_cntpct();
	bench_read_64(buf1, sz);
	dsb();
	t1 = read_cntpct();
	tr = (u32)(t1 - t0);

	/* Copy: LDP+STP */
	t0 = read_cntpct();
	bench_copy_64(buf2, buf1, sz);
	dsb();
	t1 = read_cntpct();
	tc = (u32)(t1 - t0);

	{
		/* KB/s = sz * freq / ticks / 1024 */
		u32 w = tw ? (u32)(((u64)sz * freq / tw) >> 10) : 0;
		u32 r = tr ? (u32)(((u64)sz * freq / tr) >> 10) : 0;
		u32 c = tc ? (u32)(((u64)sz * freq / tc) >> 10) : 0;

		printf("  %u: W=%u R=%u C=%u KB/s\n", sz, w, r, c);
	}
}

static void meson_dram_benchmark(void)
{
	u32 freq = read_cntfrq();
	static const u32 sizes[] = {
		256, 1024, 4096, 16384, 65536, 262144, 1 << 20, 4 << 20
	};

	/*
	 * Set up minimal MMU at EL3 for cached DRAM access.
	 * Level 1 table with 1GB block entries, 4KB granule.
	 * DRAM (0-3GB) = normal write-back cacheable.
	 * Peripherals (3-4GB) = device-nGnRnE.
	 */
	{
		volatile u64 *pt = (volatile u64 *)(BENCH_BASE + (8 << 20));
		u64 blk_attr_mem, blk_attr_dev;

		/* MAIR: idx0 = WB-RA-WA (0xff), idx1 = device-nGnRnE (0x00) */
		asm volatile("msr mair_el1, %0" :: "r"(0xffULL));

		/* Block descriptor: [1:0]=01, [10]=AF, [9:8]=SH, [7:6]=AP */
		/* Normal memory: AttrIdx=0, Inner Shareable */
		blk_attr_mem = (1ULL << 0)	/* block */
			     | (0ULL << 2)	/* AttrIndx=0 (WB) */
			     | (1ULL << 10)	/* AF */
			     | (3ULL << 8);	/* SH=inner shareable */

		/* Device memory: AttrIdx=1 */
		blk_attr_dev = (1ULL << 0)
			     | (1ULL << 2)	/* AttrIndx=1 (device) */
			     | (1ULL << 10);	/* AF */

		memset((void *)pt, 0, 4096);
		pt[0] = 0x00000000ULL | blk_attr_mem;	/* 0-1GB: DRAM */
		pt[1] = 0x40000000ULL | blk_attr_mem;	/* 1-2GB: DRAM */
		pt[2] = 0x80000000ULL | blk_attr_dev;	/* 2-3GB: periph */
		pt[3] = 0xC0000000ULL | blk_attr_dev;	/* 3-4GB: SRAM+periph (device for UART etc) */
		dsb();

		/* TCR_EL3: T0SZ=32(4GB), TG0=4KB, SH0=inner, ORGN/IRGN=WB-WA */
		asm volatile("msr tcr_el1, %0" :: "r"(
			(32ULL << 0)	/* T0SZ = 32 (4GB VA) */
			| (1ULL << 8)	/* IRGN0 = WB-WA */
			| (1ULL << 10)	/* ORGN0 = WB-WA */
			| (3ULL << 12)	/* SH0 = inner shareable */
			| (0ULL << 14)	/* TG0 = 4KB */
		));
		asm volatile("msr ttbr0_el1, %0" :: "r"((u64)(uintptr_t)pt));
		asm volatile("isb");

		/* Invalidate TLB and caches before enabling MMU */
		asm volatile(
			"tlbi vmalle1\n"
			"dsb sy\n"
			"isb\n"
		);

		/* Enable MMU + dcache + icache */
		asm volatile(
			"mrs x0, sctlr_el1\n"
			"orr x0, x0, #(1 << 0)\n"	/* M = MMU on */
			"orr x0, x0, #(1 << 2)\n"	/* C = dcache on */
			"orr x0, x0, #(1 << 12)\n"	/* I = icache on */
			"msr sctlr_el1, x0\n"
			"isb\n"
			::: "x0"
		);
	}

	/* Verify PLL state after init */
	printf("\nPLL:%x CLK0:%x CLK1:%x R26C:%x\n",
	       readl(GX_HIU_BASE + HHI_SYS_PLL_CNTL),
	       readl(GX_HIU_BASE + HHI_SYS_CPU_CLK_CNTL0),
	       readl(GX_HIU_BASE + HHI_SYS_CPU_CLK_CNTL1),
	       readl(GX_HIU_BASE + 0x26c));

	/* CPU speed test (before MMU -- SRAM needs normal access) */
	{
		u64 t0 = read_cntpct(), t1;
		u32 n = 65536;
		asm volatile(
			"1: subs %w0, %w0, #1\n"
			"   bne 1b\n"
			: "+r"(n) :: "cc"
		);
		t1 = read_cntpct();
		printf("CPU: 64K=%u ticks (%u MHz)\n", (u32)(t1 - t0),
		       (u32)(t1 - t0) > 0 ? 24 * 65536 * 2 / (u32)(t1 - t0) : 0);
	}
	printf("DDR bandwidth (KB/s):\n");
	for (u32 i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
		/* Invalidate cache before each test for cold-start measurement */
		asm volatile("dsb sy");
		meson_dram_bench_size(sizes[i], freq);
	}

	/* Disable MMU + dcache, clean first */
	asm volatile(
		"dsb sy\n"
		"mrs x0, sctlr_el1\n"
		"bic x0, x0, #(1 << 0)\n"	/* M = MMU off */
		"bic x0, x0, #(1 << 2)\n"	/* C = dcache off */
		"msr sctlr_el1, x0\n"
		"isb\n"
		::: "x0"
	);
}

/*
 * Auto-detect DRAM size by writing test patterns at power-of-2 boundaries.
 * GXL address 0 is SRAM, so we use 16MB as the reference address.  16 MB is
 * inside rank 0 for every size this DMC can be programmed with.
 *
 * The offset is a Kconfig symbol only so that the bail path can be reached on
 * healthy silicon (N178): placed ON a marker address, the marker loop clobbers
 * the reference and dram_init() refuses through the shipping compare, with no
 * injected constant anywhere.  The default is the 16 MB above and every
 * shipping board builds it -- see MESON_DRAM_DETECT_REF_OFFSET in Kconfig.
 */
#ifndef CONFIG_MESON_DRAM_DETECT_REF_OFFSET
#define CONFIG_MESON_DRAM_DETECT_REF_OFFSET	0x1000000
#endif
#define DDR_DETECT_REF_ADDR	(CFG_SYS_SDRAM_BASE + \
				 CONFIG_MESON_DRAM_DETECT_REF_OFFSET) /* 16MB */

/*
 * A PER-BOOT nonce, so a marker cell cannot pass by holding LAST boot's value.
 *
 * This workstream lives on watchdog reboots, and DRAM contents survive one.
 * With a constant marker (`*p = size_mb`) a cell whose write was COMPLETELY
 * LOST still read back the right number, because the previous boot had written
 * exactly that number there -- "the write landed" and "the write vanished and
 * the old value is still sitting there" printed the same result.  Mixing a
 * value that changes every boot into the marker makes those two states print
 * different words, which is the whole requirement on the check.
 *
 * The top bit is forced so the nonce, and every marker derived from it, is far
 * outside the range of a size in MB and cannot be confused with one.
 */
static u32 meson_dram_detect_nonce(void)
{
	u64 t;

	asm volatile("isb; mrs %0, cntpct_el0" : "=r" (t));
	return (u32)t | 0x80000000u;
}

/*
 * Why the refusal is a REASON CODE reported by dram_init(), and why it does
 * not carry the words it compared.  Both halves are priced, not assumed --
 * measured on the non-repeat sweep stack, every arm rc=0 and identical
 * otherwise:
 *
 *   34208 B  base
 *   34376 B  this: `printf("... why=%u\n", dram_detect_fail)` in dram_init()
 *   38672 B  the same line plus `ref=%08x nonce=%08x`
 *
 * So the two extra `%08x` conversions cost 4296 B -- 40% of the sweep's whole
 * headroom -- because this TPL's printf has no zero-padded-width path linked
 * in and asking for one drags the full formatter in behind it.  The reason
 * code alone separates the two refusal states, which is what the verdict
 * needs; the compared words are colour, and 4296 B is not what colour costs.
 * A run that needs them can read them off the SRAM breadcrumbs.
 *
 * Assigned at entry, not merely declared: SPL BSS on this path is not zeroed.
 */
static u32 dram_detect_fail;
static u32 dram_detect_ref, dram_detect_nonce;
/* the top marker cell's readback, for dram_init()'s size-mismatch warning */
static u32 dram_detect_top_rb;

/*
 * The smallest marker the detector writes, and therefore the LAST word the
 * marker loop stores.  Named because two things depend on it being the last:
 * the largest-to-smallest walk order, and the bus-echo discrimination below.
 *
 * Kconfig-settable (0 = the built-in 128) for one reason: it is the only way
 * to reach why=2.  Set ABOVE meson_dram_detect_max_mb(), both marker loops
 * have an empty range -- nothing is written and nothing is read back -- so the
 * reference still holds the nonce and the walk finds no cell holding its own
 * marker, which is exactly the state why=2 names.  That is a real misordered
 * detect window, not an injected value, and refusing is the correct response
 * to it.  A compile-time constant either way, so the default image is
 * unchanged.  See MESON_DRAM_DETECT_MIN_MB in Kconfig.
 */
#if defined(CONFIG_MESON_DRAM_DETECT_MIN_MB) && CONFIG_MESON_DRAM_DETECT_MIN_MB > 0
#define DDR_DETECT_MIN_MB	CONFIG_MESON_DRAM_DETECT_MIN_MB
#else
#define DDR_DETECT_MIN_MB	128
#endif

/*
 * The why=3 arm below compares the reference readback against
 * `nonce ^ DDR_DETECT_MIN_MB`.  At DDR_DETECT_MIN_MB == 0 that expression is
 * just `nonce`, i.e. the value the PASS path already required -- the arm would
 * be structurally unfirable, which is precisely the defect the old
 * `ref[0] == 0` alias guard had.  (It would also make both marker loops
 * non-terminating, since `size_mb >>= 1` reaches 0 and 0 >= 0 holds.)
 *
 * So the guard against shipping a second check that cannot fail is a build
 * error, not a comment.  Negative control for this assert is in the commit
 * message: forcing 0 fails the build with this message, reverting rebuilds
 * byte-identical.
 */
static_assert(DDR_DETECT_MIN_MB != 0,
	      "DDR_DETECT_MIN_MB==0 makes the why=3 bus-echo arm unfirable");

static u32 meson_dram_detect_size(void)
{
	volatile u32 *ref = (volatile u32 *)DDR_DETECT_REF_ADDR;
	u32 nonce = meson_dram_detect_nonce();
	u32 size_mb;
	/*
	 * Write-verify accumulator (D2 -- aliasing-test.md "Remaining tests").
	 *
	 * The detector is one write-all pass followed by one read-all pass, so a
	 * mismatch in the read pass has TWO causes it cannot separate: the word
	 * was NEVER STORED (the store went somewhere else, was dropped, or the
	 * address decodes nowhere), or it was STORED AND THEN LOST (a fold from a
	 * later, smaller marker reached it, retention, corruption).  Those are a
	 * write/command-path fault and a map/retention fault respectively, and
	 * they send a rig session in opposite directions.
	 *
	 * Reading a cell back IMMEDIATELY after its own store splits them, and
	 * costs no barrier to do it: a load from the SAME address as the
	 * immediately preceding store is ordered against it by the architecture,
	 * and this stage runs MMU-off at EL3 where every access is
	 * Device-nGnRnE regardless.  A marker's own read-back also cannot be
	 * disturbed by the rest of the loop, because every later iteration
	 * writes a strictly SMALLER address.
	 *
	 * Bit 0: some marker read back its own value at store time.
	 * Bit 1: some marker did not.
	 *
	 * Both bits clear is the EMPTY-RANGE case (DDR_DETECT_MIN_MB set above
	 * meson_dram_detect_max_mb(), the only way to reach why=2 on healthy
	 * silicon), and it must keep reporting why=2 -- nothing was written, so
	 * "never stored" would be a false accusation.  Only bit 1 alone means
	 * markers were attempted and not one of them landed.
	 *
	 * NOT YET OBSERVED ON HARDWARE, and there is no Kconfig knob that makes
	 * either new arm fire on a healthy board the way
	 * MESON_DRAM_DETECT_REF_OFFSET makes why=1/why=3 fire.  Both new arms
	 * are structurally firable -- each compares a real readback against the
	 * word just written, with no injected constant on either side -- but
	 * the state they name (a store that does not land at an address the DMC
	 * decodes) cannot be produced by moving a cell: an address NO rank
	 * answers does not complete on this DMC at all, so the SPL watchdog
	 * reboots and there is no `why=` line to read.  That absence is already
	 * documented as the fourth state in dram_init().  Until a part actually
	 * exhibits it, the codes below are a discrimination the sweep is now
	 * CAPABLE of, not one it has demonstrated.
	 */
	u32 wv_seen = 0;

	dram_detect_fail = 0;
	dram_detect_nonce = nonce;
	dram_detect_ref = 0;
	dram_detect_top_rb = 0;

	/*
	 * The reference cell is a SIGNATURE, not a zero.
	 *
	 * It used to be `ref[0] = 0` guarded by `ref[0] == 0`, described as an
	 * alias guard.  That guard was structurally unfirable: `ref` is at
	 * offset 0x000000 within its MB and every marker is at 0xffffc within
	 * its own, so no power-of-2 address fold can map a marker onto it --
	 * the condition was true whether or not the map aliased, and it was
	 * being read as evidence that it did not.
	 *
	 * A signature CAN fail, on a state the old form could not see: a bus
	 * that returns the last-written word regardless of address, or DRAM
	 * that reads back a constant.  In either case the markers all "match"
	 * and the reference does not, so the detector refuses instead of
	 * reporting meson_dram_detect_max_mb().
	 */
	ref[0] = nonce;

	/*
	 * why=4: the reference did not read back its own signature at store
	 * time, before the marker loop has written anything at all.  Nothing
	 * can have overwritten it yet, so this is not a fold and not a
	 * retention loss -- the store never landed.  That is the half of the
	 * old why=1 ("it did not store, or a fold reached it") which no later
	 * evidence can recover, and separating it here is the whole point of
	 * the write-verify pass: with this arm in place why=1 below means
	 * STORED AND THEN LOST, unambiguously.
	 *
	 * Bailing here rather than continuing is deliberate.  The marker loop
	 * has no meaning once the reference cell cannot hold a word, and not
	 * running it leaves the sweep's own writes out of whatever state the
	 * rig is about to be inspected in.
	 */
	if (ref[0] != nonce) {
		dram_detect_ref = ref[0];
		dram_detect_fail = 4;
		return 0;
	}

	/* Markers, largest to smallest, so a fold leaves the SMALLEST intact. */
	for (size_mb = meson_dram_detect_max_mb(); size_mb >= DDR_DETECT_MIN_MB;
	     size_mb >>= 1) {
		volatile u32 *p = (volatile u32 *)(CFG_SYS_SDRAM_BASE +
						   ((ulong)size_mb << 20) - 4);
		*p = nonce ^ size_mb;
		/*
		 * Not a failure on its own: every marker ABOVE the real size is
		 * expected to fail this, and that is the normal outcome on a
		 * healthy part.  It is only read at the why=2 bail below, where
		 * the walk has already found nothing.
		 */
		wv_seen |= (*p == (nonce ^ size_mb)) ? 1 : 2;
	}
	dsb();

	dram_detect_ref = ref[0];
	if (dram_detect_ref != nonce) {
		/*
		 * A bail is only better than a false size if the reason is
		 * legible, and these two are DIFFERENT FAULTS that the single
		 * "ref signature failed" code could not tell apart:
		 *
		 *   why=3  the reference read back the LAST word the marker
		 *          loop wrote -- `nonce ^ DDR_DETECT_MIN_MB`.  That is
		 *          a bus (or a DMC window) returning the last-written
		 *          word regardless of address.  It is NOT a size fault
		 *          at all, and chasing rank sizes for it wastes rig
		 *          time; the check above is the only place this is
		 *          visible, because such a bus makes every marker
		 *          "match" too.
		 *   why=1  the reference read back something else.  It DID
		 *          store -- the write-verify arm above already proved
		 *          that -- so the word was stored and then lost: a fold
		 *          from the marker loop reached it, or the cell did not
		 *          hold.  A size/map fault.  (The "did not store" half
		 *          of this code's old meaning is now why=4.)
		 *
		 * One compare and one constant.  Priced in the commit message
		 * against a paired control, not assumed.
		 */
		dram_detect_fail =
			(dram_detect_ref == (nonce ^ DDR_DETECT_MIN_MB)) ? 3 : 1;
		return 0;
	}

	/*
	 * First cell that still holds THIS boot's own marker is the size.  Each
	 * cell is read ONCE -- the top one's value is kept for the size-mismatch
	 * warning in dram_init(), without a second access that could disagree
	 * with the one the size was decided on.
	 */
	for (size_mb = meson_dram_detect_max_mb(); size_mb >= DDR_DETECT_MIN_MB;
	     size_mb >>= 1) {
		volatile u32 *p = (volatile u32 *)(CFG_SYS_SDRAM_BASE +
						   ((ulong)size_mb << 20) - 4);
		u32 v = *p;

		if (size_mb == meson_dram_detect_max_mb())
			dram_detect_top_rb = v;
		if (v == (nonce ^ size_mb))
			return size_mb;
	}

	/*
	 * Not a size: every marker cell disagreed with what was just written
	 * to it.  Returning 128 here would hand the rest of the boot a number
	 * nothing measured, and every window END derived from it would inherit
	 * that guess silently.
	 *
	 * Which of the two faults it is comes from the write-verify pass:
	 *
	 *   why=5  bit 1 set and bit 0 clear -- markers were attempted and NOT
	 *          ONE of them read back its own value at store time.  Nothing
	 *          was ever stored anywhere in the swept range; the write or
	 *          command path is dead, or no address in it decodes.  Chasing
	 *          rank sizes or retention for this wastes rig time.
	 *   why=2  anything else -- at least one marker did store and the walk
	 *          then found none of them (stored and lost), or the range was
	 *          empty and nothing was written at all.
	 */
	dram_detect_fail = (wv_seen == 2) ? 5 : 2;
	return 0;
}

/*
 * The witness for everything above: the signature reference cell, the per-boot
 * nonce, and the `return 0` refusal that makes dram_init() FAIL rather than
 * publish 128 MB.  Any consumer whose verdicts are read against a window END
 * derived from this function requires it by generation -- see
 * ddr-detect-gen.h.  Absolute symbol; zero image bytes; present on the product
 * build too, where nothing requires it.
 */
DDR_DETECT_PROVIDE(DDR_DETECT_GEN);

/*
 * Memtest word for a key.  0x9e3779b9 is odd, so k -> k * 0x9e3779b9 is a
 * bijection on u32: distinct keys give distinct words (aliasing still shows),
 * and consecutive keys differ in ALL FOUR byte lanes rather than only the low
 * one.  The XOR is cosmetic -- it just keeps key 0 off the all-zero word.
 *
 * Used by BOTH DRAM types.  It was DDR4-scoped on a size argument -- "the DDR3
 * SPL has 29 B of headroom against CONFIG_SPL_MAX_SIZE and this costs 304 B"
 * -- which described the pre-split SINGLE-SPL geometry.  On the TPL/SPL chain
 * this code lives in the TPL: libretech-cc_libre_defconfig links 19144 B
 * against CONFIG_TPL_SIZE_LIMIT=0xb000 (45056 B), i.e. 25912 B free, and the
 * full-entropy memtest costs 288 B of it.  See the memtest block below.
 */
static inline u32 meson_dram_pat(u32 k)
{
	return (k * 0x9e3779b9u) ^ 0xa5a5a5a5u;
}

/*
 * The one megabyte the memtest must NOT write.
 *
 * ramoops exists to survive a reset, so a write+read test over it is
 * self-defeating: it erases pstore on every boot, destroying the log of the
 * crash it was meant to preserve.  The stride pass below writes ONE word per
 * megabyte, so exactly one of its iterations lands in the region; skipping
 * that iteration leaves every other byte of DRAM covered, and the dense blocks
 * (lowest and highest megabyte) are unaffected.
 *
 * Scoped to the libre layout because that is what puts ramoops at
 * GX_LIBRE_RAMOOPS_BASE.  With the symbol off this is the constant 0 and the
 * generated code is unchanged.
 *
 * amlogic/software/boot/design/dram-reservations.md sections 8c A6 and 10b.
 */
#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT)
#define MESON_DRAM_NOTEST(off)						\
	((off) >= GX_LIBRE_RAMOOPS_BASE &&				\
	 (off) < GX_LIBRE_RAMOOPS_BASE + GX_LIBRE_RAMOOPS_SIZE)
#else
#define MESON_DRAM_NOTEST(off)		0
#endif

/*
 * Size dram_init() proved, in MB; 0 until it has.  The TPL handoff block needs
 * the trained size, and dram_init()'s int return cannot carry it.
 */
u32 meson_dram_trained_mb;

/* The clock this attempt trains at; dram_init() sets it per ladder rung. */
u32 meson_dram_clk;

/* Watchdog pet between ladder attempts; spl.c supplies the real one. */
__weak void meson_spl_wdt_pet(void)
{
}

#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR3)
/*
 * The lanes whose rank-1 DXnGTR0 byte 2 differs from rank 0's, after a
 * two-rank training (0: none).  Per-rank words are read through RANKIDR,
 * rank in [3:0] and [19:16], as the vendor's cmd_ddr_test.c selects them.
 *
 * n1212 (1392 MHz, 150 trainings): byte 2 read 2 on every lane of rank 0
 * in all 150, and on rank 1 in all 147 trainings that tested clean.  In the
 * 3 trainings whose rank 1 lane 2 then failed every probe ("mode B"), rank
 * 1's byte 2 was 4 on lane 2, and on other lanes in 2 of them.  n1209's
 * short size detection and n1211's 1200 MHz rank-1 lane-2 misreads are the
 * same mode.  A mismatch rejects the rung, so the ladder retrains.
 */
static u32 meson_dram_rank_gtr_mismatch(u32 *r0, u32 *r1)
{
	static const ulong gtr[] = {
		DDR0_PUB_DX0GTR, DDR0_PUB_DX1GTR, DDR0_PUB_DX2GTR, DDR0_PUB_DX3GTR,
	};
	u32 i, m = 0;

	for (i = 0; i < 4; i++) {
		writel(0, DDR0_PUB_RANKIDR);
		r0[i] = readl(gtr[i]);
		writel(1 | (1 << 16), DDR0_PUB_RANKIDR);
		r1[i] = readl(gtr[i]);
		if (((r0[i] >> 16) & 0xff) != ((r1[i] >> 16) & 0xff))
			m |= BIT(i);
	}
	writel(0, DDR0_PUB_RANKIDR);
	return m;
}
#endif

/*
 * One rung of dram_init()'s ladder, at meson_dram_clk with
 * meson_dram_rank1_active ranks: train, then size-detect, then memtest.
 *
 *   < 0  training failed (do_init's error)
 *   0    trained, detected and memtested; *size_mb is the detected size
 *   1    trained, but detection refused or the memtest failed
 *
 * Every attempt re-enters from reset (meson_dram_phy_prepare() asserts the
 * DMC and PHY resets, N27), including after a rung that trained cleanly.
 */
static int meson_dram_attempt(int attempt, u32 *size_mb)
{
	int ret;
	u32 detected_size;

	*size_mb = 0;
#if MESON_DRAM_FAIL_FIRST_ATTEMPTS > 0
	dram_attempt = attempt;
#endif
	ret = meson_dram_do_init();
	if (ret < 0)
		return ret;
#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR3)
	if (meson_dram_rank1_active) {
		u32 r0[4], r1[4], m = meson_dram_rank_gtr_mismatch(r0, r1);

		if (m) {
			u32 l = __builtin_ctz(m);

			printf("DDR: attempt %d rank 1 lane latency differs from rank 0 (lanes %x; dx%u GTR0 rank 0 %08x rank 1 %08x) -- rung rejected\n",
			       attempt, m, l, r0[l], r1[l]);
			return 1;
		}
	}
#endif

	/*
	 * dmc_init already ran inside do_init before finalise (vendor order).
	 * Re-assert AXI after GO.
	 *
	 * Same product value as the dmc_init write above -- this re-assert is
	 * the one that actually survives into BL31, so a permissive dmc_init
	 * write alone is not enough. Unconditional for the same reason the
	 * dmc_init write is: the DDR3 board is the one that was left at 0.
	 */
	writel(0xffffffff, DMC_SEC_AXI_PORT_CTRL);

#if defined(CONFIG_DRAM_DDR4)
	/* REQ re-assert for the DDR4 detect/memtest path. */
	writel(0xffff, DMC_REQ_CTRL);
	dmb();
	isb();

	/* Force M|C|I off for uncached DRAM probe. */
	{
		unsigned long sctlr0 = get_sctlr();
		unsigned int el = current_el();

		set_sctlr(sctlr0 & ~(CR_M | CR_C | CR_I));
		if (el == 1)
			asm volatile("msr ttbr0_el1, %0" : : "r" (0UL));
		else if (el == 2)
			asm volatile("msr ttbr0_el2, %0" : : "r" (0UL));
		else
			asm volatile("msr ttbr0_el3, %0" : : "r" (0UL));
		if (el == 1)
			asm volatile("tlbi vmalle1\n dsb sy\n isb" ::: "memory");
		else if (el == 2)
			asm volatile("tlbi alle2\n dsb sy\n isb" ::: "memory");
		else
			asm volatile("tlbi alle3\n dsb sy\n isb" ::: "memory");
	}
#else
	meson_dram_dmc_init();
#endif

	/*
	 * Name the stage before the first access that can hang.  A detect or
	 * memtest access no trained rank answers does not complete, and the
	 * last line used to be the final-PIR "PGSR0=80000fff", which says
	 * training PASSED (GXL DDR3 FMA round F1 FA5, claude amlogic/software/
	 * boot/gxl/audit/2026-10-19-gxl-ddr3-boot-chain-failure-modes.md).
	 * No colon: "DDR: <n> MB" lines are what scorers grep.
	 */
	puts("DDR detect\n");
	detected_size = meson_dram_detect_size();
	if (!detected_size) {
		/*
		 * Detection refused.  Failing dram_init() is the honest outcome:
		 * the size is the window END every later stage inherits, and a
		 * substituted default would propagate into the AO register, the
		 * DMC range decode and every probe window as if it had been
		 * measured.  spl.c prints "DF" and the host's pass-2 gate stops.
		 *
		 * why= key, for a console log read without the source at hand.
		 * Codes 4 and 5 are the write-verify pass (D2): each is the
		 * NEVER-STORED half of the code above it, which now means
		 * stored-and-then-lost.
		 *   1  reference stored the signature and later did not hold it
		 *      -- a fold reached it, or the cell lost it.  size/map
		 *   2  reference stored, markers stored, but NO marker survived
		 *      the walk -- stored then lost.  size/map
		 *   3  reference read back the last-written marker -- the bus
		 *      echoes; not a size fault, do not chase rank sizes
		 *   4  reference never stored: it did not read back its own
		 *      signature at store time, before any marker was written.
		 *      A write/command-path fault, not a size fault
		 *   5  no marker ever stored: not one read back its own value at
		 *      store time, and the walk then found none.  Same class as
		 *      4, one layer out
		 * NO `why=` line at all, and nothing after "DMC version", means
		 * detection could not RUN: an address no rank decodes does not
		 * complete on this DMC, so the SPL watchdog reboots instead.
		 * That absence is the fourth state and it is not a reason code.
		 */
		printf("DDR: size detect REFUSED why=%u\n", dram_detect_fail);
		return 1;
	}

	/*
	 * Detection is the only step on this path that has looked at the part.
	 * Hand its answer back to the DMC so the range decode stops offering
	 * addresses no rank answers -- vendor ddr_detect_size rewrites
	 * DMC_SEC_1DA size fields the same way.  DDR4-scoped: DDR3 keeps the
	 * seeded window it shipped with (untested there).
	 */
#if defined(CONFIG_DRAM_DDR4)
	writel(DMC_CTRL_CHANNEL | DMC_CTRL_DDR_TYPE | DMC_CTRL_WIDTH |
	       DMC_SIZE_FIELD(detected_size) | meson_dram_dmc_ctrl_rank1(),
	       DMC_DDR_CTRL);
#endif

	/*
	 * Memory test across the WHOLE detected window, not just the first MB.
	 * A test confined to low memory says nothing about the upper half, and
	 * the upper half is exactly where a wrong rank size or a folded address
	 * map shows up.  Two parts, both with MMU and dcache off:
	 *
	 *  - a stride pass writing one address-derived word per MB over
	 *    [1 MB, size), then reading every one back.  Distinct values a
	 *    megabyte apart is what catches aliasing anywhere in the window;
	 *  - a dense 256-word block in the LOWEST and in the HIGHEST megabyte,
	 *    so both ends get a real data pattern rather than a single word.
	 *
	 * Address 0 is SRAM on GXL, so the walk starts at 1 MB.
	 *
	 * The words are a MULTIPLICATIVE HASH of each word's key rather than
	 * the key XOR a constant, and that is the whole point.  The original
	 * patterns left byte lanes constant, so those lanes could not fail:
	 *
	 *   stride  (u32)off ^ 0xa5a5a5a5, off a multiple of 1 MB
	 *           -> byte 0 and byte 1 are ALWAYS 0xa5.  Lanes 0 and 1 blind.
	 *   dense   i ^ 0xaa55aa55, i = 0..255
	 *           -> byte 1 always 0xaa, byte 2 always 0x55, byte 3 always
	 *              0xaa.  Only lane 0 carries data.
	 *
	 * Between them lane 1 was never varied by ANY word of the test, and
	 * lanes 2/3 only by the coarse stride pass.  meson_dram_pat() multiplies
	 * the key by the odd constant 0x9e3779b9, which is a bijection on u32,
	 * so consecutive keys differ in every byte lane and each lane sweeps all
	 * 256 values.  Keys: the megabyte index for the stride pass, the word
	 * index for the dense blocks -- disjoint ranges, so no two addresses in
	 * the test hold the same word and aliasing still shows up as a mismatch.
	 *
	 * Errors are counted PER BYTE LANE as well as in total, so a failure
	 * names the lane instead of just the word (same reason the eye-margin
	 * shmoo counts that way).  `lanevar` is the OR of (expected ^ first
	 * expected) over every word written: a byte that reads back as zero
	 * there is a lane the test never varied, i.e. a lane it cannot fail on.
	 * It is printed so the pass line carries its own coverage proof.
	 *
	 * BOTH DRAM TYPES run this.  It used to be DDR4-scoped by a #if, on a
	 * size argument: "the DDR3 target links at 45027 B against a 45056 B
	 * limit, i.e. 29 B free, so it does not fit".  That budget was the
	 * SINGLE-SPL SRAM geometry.  Since the TPL/SPL split this code links
	 * into the TPL, and libretech-cc_libre_defconfig builds a 19144 B TPL
	 * against CONFIG_TPL_SIZE_LIMIT=0xb000 (45056 B) -- 25912 B free, of
	 * which the full-entropy pattern plus the histogram costs 288 B.  The
	 * blocker the #if named does not exist on this geometry, and the same
	 * comment already said to delete the #if once there was room.
	 */
	{
		const ulong end = (ulong)detected_size << 20;
		u32 i, errors = 0;
		ulong off;
		u32 e, e0 = 0, d, lanevar = 0;
		u32 lanerr[4] = { 0, 0, 0, 0 };

		for (off = 0x100000; off < end; off += 0x100000) {
			if (MESON_DRAM_NOTEST(off))
				continue;
			e = meson_dram_pat((u32)(off >> 20));
			if (off == 0x100000)
				e0 = e;
			lanevar |= e ^ e0;
			*(volatile u32 *)(CFG_SYS_SDRAM_BASE + off) = e;
		}
		/*
		 * Every word the test checks is then cleared to 0.  The TPL's
		 * jump refuses an SPL entry word of 0 or 0xffffffff as "nobody
		 * wrote here" (spl.c gx_tpl_jump_to_spl()), and the stride
		 * pass's word at the DRAM SPL entry (0x08000000) used to stay
		 * behind as 0xbe197925: with no DRAM SPL staged the TPL printed
		 * JMP and branched into it, with the watchdog stopped (FMA round
		 * F1 FC1, same audit as the walk below).  Clearing, rather than
		 * restoring what was there, also keeps a previous boot's image
		 * from surviving at a load address.
		 */
		for (off = 0x100000; off < end; off += 0x100000) {
			volatile u32 *w;

			if (MESON_DRAM_NOTEST(off))
				continue;
			w = (volatile u32 *)(CFG_SYS_SDRAM_BASE + off);
			d = *w ^ meson_dram_pat((u32)(off >> 20));
			*w = 0;
			if (d) {
				errors++;
				for (i = 0; i < 4; i++)
					if (d & (0xffu << (i * 8)))
						lanerr[i]++;
			}
		}

		/*
		 * Dense blocks, plus an address-line walk over AXI[19:10].
		 *
		 * The stride pass only varies AXI[30:20] (one word per MB) and
		 * the dense block only AXI[9:2], so no two words of the test
		 * differ in AXI[19:10] -- which the DDR3 map uses for ca8/ca9,
		 * the three bank bits and the low five row bits.  A map or a
		 * bank/address net that ignores one of those bits passed with
		 * a clean "memtest OK" (measured: ba1 tied off, FMA round F1
		 * FA1, claude amlogic/software/boot/gxl/audit/
		 * 2026-10-19-gxl-ddr3-boot-chain-failure-modes.md).  So inside
		 * each dense block's megabyte, one more word goes at
		 * base + (1 << b) for b = 10..19, keyed by its own word index
		 * like the dense block (disjoint from the dense keys and from
		 * the stride pass's MB keys).  Everything is written before
		 * anything is read, so a bit that does not select its own
		 * location makes one write land on another, and the read-back
		 * of the overwritten word mismatches.  Both ranks are covered:
		 * one block sits at +1 MB, the other at the top megabyte.
		 */
		for (off = 0x100000;; off = end - 0x100000) {
			volatile u32 *p =
				(volatile u32 *)(CFG_SYS_SDRAM_BASE + off);
			const u32 k = (u32)(off >> 2);
			u32 b;

			for (i = 0; i < 256; i++) {
				e = meson_dram_pat(k + i);
				lanevar |= e ^ e0;
				p[i] = e;
			}
			for (b = 10; b < 20; b++)
				p[1u << (b - 2)] = meson_dram_pat(k + (1u << (b - 2)));
			for (i = 0; i < 256 + 10; i++) {
				const u32 w = i < 256 ? i : 1u << (i - 256 + 10 - 2);

				d = p[w] ^ meson_dram_pat(k + w);
				p[w] = 0;	/* cleared, see the stride pass */
				if (d) {
					errors++;
					for (e = 0; e < 4; e++)
						if (d & (0xffu << (e * 8)))
							lanerr[e]++;
				}
			}
			if (off != 0x100000)
				break;
		}

		/* Write detected size for later stages / kernel */
		clrsetbits_32(GX_SEC_AO_SEC_GP_CFG0, GX_AO_MEM_SIZE_MASK,
			      detected_size << GX_AO_MEM_SIZE_SHIFT);

		/*
		 * Only the TOP of the covered range is printed -- the range
		 * starts at SDRAM_BASE + 1 MB always, so a second bound would
		 * restate a constant.  Printing the top is what makes
		 * "memtest OK" mean the whole part rather than its first
		 * megabyte.
		 */
		/*
		 * FATAL on BOTH DRAM types.  This used to abort only on DDR4,
		 * because AML-S905X-CC's DDR3 address map decoded only ~128 MB
		 * uniquely and the test could not pass there -- aborting would
		 * have turned a board that had always booted-with-aliased-RAM
		 * into a board that does not boot, without fixing the map.
		 * That map was fixed (see the DDR3 arm of MESON_DRAM_RANK0_MB
		 * above), and the fix is measured: the stride pass writes one
		 * address-derived word per MB across the whole 2047 MB window
		 * and reads every one back with errors == 0, which 8x aliasing
		 * cannot produce.  35 boots on one byte-identical TPL binary,
		 * all `memtest OK to 0x7fffffff, try 1`.  The premise for the
		 * split is gone, so the split is gone: a memtest failure is a
		 * refusal to hand DRAM to the next stage, on either type.
		 *
		 * Both types also run the SAME patterns now (see the block
		 * above).  While the DDR3 arm ran the low-entropy ones, byte
		 * lane 1 was never varied by any word of the test, so it could
		 * not fail there and no `lanevar` was compiled in to say so --
		 * a DDR3 pass could not state its own coverage.  It can now:
		 * the line below prints on either type, and a lane that reads
		 * back zero in `lanevar` is a lane the test never varied.
		 */
		printf("DDR: lanevar %08x lanerr %u %u %u %u\n", lanevar,
		       lanerr[0], lanerr[1], lanerr[2], lanerr[3]);
		if (errors) {
			printf("DDR: %u MB, MEMTEST FAIL (%u errors)\n",
			       detected_size, errors);
			return 1;
		} else {
			/*
			 * `try` is the 1-based ladder attempt that printed it.
			 * A rung that detects short prints this line too, and
			 * the ladder goes on, so the LAST such line of a boot
			 * is the one it booted on.  It belongs on THIS line and
			 * not only on the pre-attempt "DDR retry %d/%d" line:
			 * that line is not printed at all when the first attempt
			 * works, so a scorer that greps one line per boot cannot
			 * tell "trained first time" from "trained on attempt 3",
			 * and a board that needs three attempts prints a line
			 * byte-identical to a healthy one.  Whether try > 1
			 * counts as a failure is the reader's policy, not the
			 * SPL's -- the SPL still boots and says what happened.
			 * Encoding the policy here would mean refusing to boot a
			 * board that trained on attempt 2, which is a different
			 * and much larger decision.
			 *
			 * The suffix goes AFTER the address bound, so the stable
			 * prefix every consumer matches on -- "memtest OK" --
			 * keeps its position, and both existing spellings stay
			 * prefixes of this one.
			 */
			printf("DDR: %u MB, %s %u MHz, memtest OK to 0x%08lx, try %d\n",
			       detected_size,
			       IS_ENABLED(CONFIG_DRAM_DDR4) ? "DDR4" : "DDR3",
			       meson_dram_clk, (ulong)CFG_SYS_SDRAM_BASE + end - 1,
			       attempt);
		}
	}

	*size_mb = detected_size;
	return 0;
}

#if defined(CONFIG_DRAM_CLK_FALLBACK1)
#define MESON_DRAM_CLK_FB1	CONFIG_DRAM_CLK_FALLBACK1
#define MESON_DRAM_CLK_FB2	\
	(CONFIG_DRAM_CLK_FALLBACK1 ? CONFIG_DRAM_CLK_FALLBACK2 : 0)
#else
#define MESON_DRAM_CLK_FB1	0
#define MESON_DRAM_CLK_FB2	0
#endif
/*
 * A fallback must be a clock the timing rules are sourced for (>= 800, the
 * DDR3-1600 bin; dram-timings-gx.h) and one the DDR PLL makes exactly (a
 * multiple of 24 above 800 MHz), and each rung must be below the one above.
 */
#define MESON_DRAM_CLK_FB_OK(f, above)					\
	((f) == 0 || ((f) >= 800 && (f) < (above) && (f) % 24 == 0))
static_assert(MESON_DRAM_CLK_FB_OK(MESON_DRAM_CLK_FB1, CONFIG_DRAM_CLK),
	      "DRAM_CLK_FALLBACK1: must be >= 800, below DRAM_CLK, a multiple of 24");
static_assert(MESON_DRAM_CLK_FB_OK(MESON_DRAM_CLK_FB2, MESON_DRAM_CLK_FB1),
	      "DRAM_CLK_FALLBACK2: must be >= 800, below FALLBACK1, a multiple of 24");

/*
 * The ladder, memory before clock: every clock with both ranks (when the
 * build declares two), then every clock with one.  Without fallbacks it is
 * the ladder this file always had: two ranks, then one rank twice.
 */
static void meson_dram_ladder_rung(int a, int nclk)
{
	static const u16 clk[] = {
		CONFIG_DRAM_CLK, MESON_DRAM_CLK_FB1, MESON_DRAM_CLK_FB2,
	};
	const bool two = MESON_DRAM_RANK1_MB != 0;

	if (nclk == 1) {
		meson_dram_rank1_active = two && a == 0;
		meson_dram_clk = CONFIG_DRAM_CLK;
	} else {
		meson_dram_rank1_active = two && a < nclk;
		meson_dram_clk = clk[a % nclk];
	}
#if defined(CONFIG_MESON_GXL) && defined(CONFIG_DRAM_DDR3)
	gxl_ddr3_timings_at(meson_dram_clk);
#endif
}

/*
 * dram_init() -- train, detect and memtest, walking a ladder until one rung
 * gives the full size for its rank count.
 *
 * Operator policy, 2026-10-09: "more total memory is preferred over
 * frequency", and a boot that ends below what the board should run says so
 * and still boots (soft degradation, as n1206).  So a rung is ACCEPTED when
 * it trains, detects the full size of its rank count and memtests clean, and
 * the order decides what is preferred:
 *
 *   two ranks at CONFIG_DRAM_CLK, FALLBACK1, FALLBACK2   (2 GB, fastest first)
 *   one rank  at CONFIG_DRAM_CLK, FALLBACK1, FALLBACK2   (1 GB)
 *
 * A rung that fails training, detects short, or fails the memtest moves on.
 * A short detection is a per-boot training state, not a weak cell (n1207:
 * 0 misreads in 450000 replays of the detector's pattern), so a retrain can
 * cure it, and a lower clock gives that retrain more margin.  If no rung is
 * accepted, the rung with the most memory that did train and test is
 * retrained and booted.
 *
 * Warnings, both soft:
 *   - the clock ended below CONFIG_DRAM_CLK;
 *   - the size ended below the two-rank size although rank 1 TRAINED on some
 *     rung, or below the accepted rung's own size.  A 1 GB board fails
 *     two-rank training at every clock and is accepted at one rank, full
 *     size, at CONFIG_DRAM_CLK: no warning, at the cost of one failed
 *     training per clock (each about 0.5 s).
 */
int dram_init(void)
{
	const int nclk = 1 + (MESON_DRAM_CLK_FB1 != 0) + (MESON_DRAM_CLK_FB2 != 0);
	const bool two = MESON_DRAM_RANK1_MB != 0;
	const int n = nclk == 1 ? DDR_TRAINING_MAX_RETRIES : (two ? 2 : 1) * nclk;
	int a, ret = -1, best = -1, last_ok = -1;
	u32 mb = 0, best_mb = 0;
	bool rank1_trained = false, accepted = false;
	u32 short_max = 0, short_rb = 0, short_want = 0;

	debug("DDR init: %s %d MHz\n",
	      IS_ENABLED(CONFIG_DRAM_DDR4) ? "DDR4" : "DDR3", CONFIG_DRAM_CLK);

	for (a = 0; a < n; a++) {
		const bool r1_before = meson_dram_rank1_active;

		/*
		 * Assigned here, not at a definition: SPL BSS on this path is
		 * not zeroed, so a file-scope initialiser would not have run.
		 */
		meson_dram_ladder_rung(a, nclk);
		if (a > 0) {
			if (r1_before && !meson_dram_rank1_active)
				printf("DDR: rank 1 dropped -- retrying as one rank\n");
			printf("DDR retry %d/%d at %u MHz\n", a + 1, n,
			       meson_dram_clk);
		}
		meson_spl_wdt_pet();

		ret = meson_dram_attempt(a + 1, &mb);
		if (ret < 0) {
			/*
			 * Carries the errno and stops claiming a retry on the
			 * LAST attempt, where there is none.  N27's ladder
			 * printed "retrying..." three times and then went
			 * silent, so the console could not distinguish "gave
			 * up" from "wedged".
			 */
			printf("DDR attempt %d/%d failed, err=%d%s\n",
			       a + 1, n, ret, a + 1 < n ? ", retrying..." : "");
			udelay(10000);
			continue;
		}
		if (meson_dram_rank1_active)
			rank1_trained = true;
		if (ret > 0) {
			printf("DDR attempt %d/%d trained but did not test clean%s\n",
			       a + 1, n, a + 1 < n ? ", retrying..." : "");
			continue;
		}
		last_ok = a;
		if (mb == meson_dram_detect_max_mb()) {
			accepted = true;
			break;
		}
		if (!short_max) {
			short_max = meson_dram_detect_max_mb();
			short_rb = dram_detect_top_rb;
			short_want = dram_detect_nonce ^ short_max;
		}
		if (mb > best_mb) {
			best_mb = mb;
			best = a;
		}
		printf("DDR attempt %d/%d short: %u of %u MB%s\n", a + 1, n, mb,
		       meson_dram_detect_max_mb(), a + 1 < n ? ", retrying..." : "");
	}

	if (!accepted) {
		if (best < 0) {
			printf("DDR init FAILED after %d attempts\n", n);
			return ret < 0 ? ret : -1;
		}
		/*
		 * Nothing gave its full size.  Boot the rung that gave the
		 * most, retraining it unless it was the last one run.
		 */
		if (last_ok != a - 1 || best != last_ok) {
			meson_dram_ladder_rung(best, nclk);
			printf("DDR: no rung gave its full size -- retraining attempt %d at %u MHz\n",
			       best + 1, meson_dram_clk);
			meson_spl_wdt_pet();
			ret = meson_dram_attempt(n + 1, &mb);
			if (ret != 0) {
				printf("DDR init FAILED after %d attempts\n", n + 1);
				return ret < 0 ? ret : -1;
			}
		}
	}

	if (meson_dram_clk < CONFIG_DRAM_CLK)
		printf("DDR: WARNING clock fallback: running %u MHz, configured %u MHz; "
		       "booting degraded\n", meson_dram_clk, CONFIG_DRAM_CLK);

	/*
	 * Size mismatch: memory that TRAINED is missing from what booted.
	 * Either the booted rung itself detected short (its top marker did not
	 * read back), or rank 1 trained on some rung and the boot still ended
	 * on one rank.  A 1 GB board never prints this: rank 1 never trains.
	 * The marker shown is the first short rung's, the cell that decided it.
	 */
	if (mb < meson_dram_detect_max_mb() ||
	    (rank1_trained && !meson_dram_rank1_active)) {
		u32 want_mb = rank1_trained ?
			MESON_DRAM_RANK0_MB + MESON_DRAM_RANK1_MB :
			meson_dram_detect_max_mb();

		printf("DDR: WARNING size mismatch: trained %u MB, detected %u MB -- ",
		       want_mb, mb);
		if (short_max)
			printf("top marker @0x%08lx read %08x want %08x; booting degraded\n",
			       (ulong)CFG_SYS_SDRAM_BASE + ((ulong)short_max << 20) - 4,
			       short_rb, short_want);
		else
			printf("rank 1 trained but did not test clean; booting degraded\n");
	}

#if IS_ENABLED(CONFIG_MESON_DRAM_ZQ_REPORT)
	/*
	 * What the PHY's ZQ calibration actually converged to, as opposed to
	 * what PUB_ZQ0PR/ZQ1PR asked for.  Read AFTER training and memtest so
	 * the value reported is the one the board ran the test with.
	 *
	 * This is the first read of these registers in this tree: the PHY
	 * drive/ODT/ZQ settings are on record as never measured (N46), and the
	 * programmed side alone cannot say whether calibration converged, hit a
	 * rail, or was overridden.  Both the PROGRAM and the RESULT are printed
	 * so the pair can be compared without the source at hand.
	 *
	 * Uses %x, not %08x: this stage's printf has no zero-padded-width path
	 * linked in and asking for one drags the full formatter in behind it
	 * (measured at +4296 B elsewhere in this file).
	 */
	printf("ZQ: cr=%x pr0=%x sr0=%x pr1=%x sr1=%x\n",
	       readl(DDR0_PUB_ZQCR), readl(DDR0_PUB_ZQ0PR),
	       readl(DDR0_PUB_ZQ0SR), readl(DDR0_PUB_ZQ1PR),
	       readl(DDR0_PUB_ZQ1SR));
#endif

	/*
	 * Published only after the memtest, so the value in the handoff block
	 * is a size that was proven usable, not merely detected.
	 */
	meson_dram_trained_mb = mb;


#ifndef CONFIG_XPL_BUILD
	meson_dram_benchmark();
#endif

	return 0;
}
