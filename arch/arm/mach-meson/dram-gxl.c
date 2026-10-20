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
#include <linux/delay.h>

/* Meson GXL specific DRAM init */

/*
 * Returns int since 2026-08-29 (N27).  It was void, and each of its three
 * waits was either unbounded or discarded, so an attempt that never got the
 * controller into CONFIG carried on regardless and an attempt that never got
 * the PHY through PIR 0x581 hung here with the console dark.  Both are
 * failures dram_init()'s ladder is entitled to hear about; it cannot report
 * that it exhausted if its stages cannot tell it that they failed.
 */
int meson_dram_prepare_pctl(void)
{
	int ret;

	writel(IS_ENABLED(CONFIG_DRAM_2T_MODE) ? 8 : 0, PCTL_MCFG);
	setbits_32(PCTL_MCFG, PCTL0_MCFG);
	/*
	 * GXL never wrote PCTL_MCFG1 (GXBB does).  Vendor ACS
	 * t_pctl0_mcfg1 = 0 (tfaw_cfg_offset); program it explicitly rather
	 * than leaving the reset value.  DDR4-scoped: added during the DDR4
	 * bring-up, never measured on a DDR3 board.
	 */
#if defined(CONFIG_DRAM_DDR4)
	writel(PCTL0_MCFG1, PCTL_MCFG1);
#endif
	udelay(500);

	ret = WAIT_FOR(PCTL_DFISTSTAT0);
	if (ret)
		return ret;

	/*
	 * Enter config state.  Production waits on bit 0 of PCTL_STAT here
	 * (init_mem is 0, config is 1, so on this upward path bit 0 rising
	 * does mean config was reached); the wait is now bounded and its
	 * return honoured, so an attempt that never reached CONFIG stops
	 * instead of programming a controller in the wrong state.
	 */
	writel(PCTL0_SCFG, PCTL_SCFG);
	writel(PCTL_SCTL_CFG_STATE, PCTL_SCTL);
	ret = WAIT_FOR(PCTL_STAT);
	if (ret)
		return ret;

	writel(0x581, DDR0_PUB_PIR);
	return WAIT_FOR(DDR0_PUB_PGSR0);
}

int meson_dram_phy_init(void)
{
	/* Some unknown magic done by bl2 */
	writel(0x190c3500, DDR0_PUB_PTR3);
	writel(0x12c493e0, DDR0_PUB_PTR4);

	writel(0x1f090909, DDR0_PUB_IOVCR0);
	writel(0x109, DDR0_PUB_IOVCR1);

	/*
	 * RUN L (2026-08-10) — DXnGCR4 bit 28.
	 *
	 * The earlier "product sets DXnGCR0 |= 0x10000000" port (EXP-A) wrote
	 * GCR0 and measured the bit as read-only there.  RUN I's full live diff
	 * shows the bit is set on product in **GCR4**, not GCR0: 0x1e09093c
	 * (product) vs 0x0e09093c (ours) on all four lanes, and GCR4 is a word
	 * we already seed right here.  So the register, not the silicon, was
	 * the mistake — seed the product value.
	 */
#if defined(CONFIG_DRAM_DDR4)
	writel(0x1e09093c, DDR0_PUB_DX0GCR4);
	writel(0x1e09093c, DDR0_PUB_DX1GCR4);
	writel(0x1e09093c, DDR0_PUB_DX2GCR4);
	writel(0x1e09093c, DDR0_PUB_DX3GCR4);
#else
	writel(0xe09093c, DDR0_PUB_DX0GCR4);
	writel(0xe09093c, DDR0_PUB_DX1GCR4);
	writel(0xe09093c, DDR0_PUB_DX2GCR4);
	writel(0xe09093c, DDR0_PUB_DX3GCR4);
#endif

	writel(PUB_ODTCR, DDR0_PUB_ODTCR);

	writel(PUB_MR0, DDR0_PUB_MR0);
	writel(PUB_MR1, DDR0_PUB_MR1);
	writel(PUB_MR2, DDR0_PUB_MR2);
	writel(PUB_MR3, DDR0_PUB_MR3);
	writel(PUB_MR4, DDR0_PUB_MR4);
	writel(PUB_MR5, DDR0_PUB_MR5);
	writel(PUB_MR6, DDR0_PUB_MR6);

	/*
	 * RUN K (2026-08-10) — MR11 (DDR4 CA ODT).  RUN I measured product BL2
	 * live on this board at MR11 = 0 against our 5 (timings.odt | BIT(2)).
	 * Part of the product MR-set match; DDR3 keeps the computed value.
	 */
#if defined(CONFIG_DRAM_DDR4)
	writel(0, DDR0_PUB_MR11);
#else
	writel(timings.odt | (1 << 2), DDR0_PUB_MR11);
#endif
	/* Configure DRAM timing parameters (DTPR) */
	writel(PUB_DTPR0, DDR0_PUB_DTPR0);
	writel(PUB_DTPR1, DDR0_PUB_DTPR1);
	writel(PUB_DTPR2, DDR0_PUB_DTPR2);
	writel(PUB_DTPR3, DDR0_PUB_DTPR3);
	writel(PUB_DTPR4, DDR0_PUB_DTPR4);
	writel(PUB_DTPR5, DDR0_PUB_DTPR5);

	/*
	 * N58 -- PGCR2 is written UNMODIFIED, both rank counts.  The vendor's
	 * ddr_pre_init() never adjusts t_pub_pgcr2 by ddrchannelset; PGCR2 is
	 * not one of the registers it makes rank-dependent (unlike DTCR1,
	 * whose |= 0x20000 below IS vendored).  The old two-rank
	 * `| (1 << 28)` was ours, not the vendor's, and this tree does not
	 * record what bit 28 selects -- an unnamed PHY config bit live on
	 * only one arm is a confound in every DRAM measurement taken on the
	 * two-rank build, which is exactly what this branch exists to take.
	 * Dropped.
	 */
	writel(PUB_PGCR2, DDR0_PUB_PGCR2);

	writel(PUB_PGCR3, DDR0_PUB_PGCR3);
	writel(PUB_DXCCR, DDR0_PUB_DXCCR);

	writel(PUB_DTCR, DDR0_PUB_DTCR);
	/*
	 * DTCR1[19:16] is the DATA TRAINING RANK SET -- one bit per rank the
	 * PHY is allowed to train.  Our static PUB_DTCR1 (0x00010237,
	 * dram-settings-gx.h) is byte-identical to the vendor's table entry,
	 * but the vendor does not write the table entry as-is: ddr_pre_init()
	 * adjusts the value before it reaches the PUB, setting bit 17 when
	 * ddrchannelset says two ranks (t_pub_dtcr1 |= 0x20000) and clearing
	 * it for one.  Writing the table value unconditionally leaves
	 * [19:16] = 0b0001 -- rank 0 only -- even with two ranks declared to
	 * the DMC, so rank 1 is never trained and its first access never
	 * completes.
	 *
	 * Gated on the same symbol as the PGCR2 conditional above, so a
	 * ONE_RANK / 16BIT_RANK build writes exactly the value it wrote
	 * before.
	 */
	/*
	 * Runtime, not build-time: the DTCR1 rank-set bit and the DMC rank-1
	 * range must move together, and dram_init()'s ladder steps both down
	 * when a two-rank attempt fails to train.  A build that declares one
	 * rank never sets meson_dram_rank1_active, so it writes exactly the
	 * value it wrote before.
	 */
	if (meson_dram_rank1_active)
		writel(PUB_DTCR1 | (1 << 17), DDR0_PUB_DTCR1);
	else
		writel(PUB_DTCR1, DDR0_PUB_DTCR1);
	/*
	 * Do NOT seed ACS PUB_PGCR0 here. Tip 072865811fb phy_init write
	 * hung at prod_stage 0x05 after mcfg1 (no FIN ZQ). Product loads
	 * ACS PGCR0 during its train path; open re-asserts post-train only
	 * (dram-gx.c MARK pgcr0-acs-product). Hang-class at phy_init.
	 */
	writel(PUB_PGCR1, DDR0_PUB_PGCR1);

	writel(0, DDR0_PUB_ACIOCR1);
	writel(0, DDR0_PUB_ACIOCR2);
	writel(0, DDR0_PUB_ACIOCR3);
	writel(0, DDR0_PUB_ACIOCR4);
	writel(0, DDR0_PUB_ACIOCR5);

	writel(0, DDR0_PUB_DX0GCR1);
	writel(0, DDR0_PUB_DX0GCR2);
	writel(0, DDR0_PUB_DX1GCR1);
	writel(0, DDR0_PUB_DX1GCR2);
	writel(0, DDR0_PUB_DX2GCR1);
	writel(0, DDR0_PUB_DX2GCR2);
	writel(0, DDR0_PUB_DX3GCR1);
	writel(0, DDR0_PUB_DX3GCR2);

	if (IS_ENABLED(CONFIG_DRAM_16BIT_RANK)) {
		writel(0, DDR0_PUB_DX2GCR0);
		writel(0, DDR0_PUB_DX3GCR0);
	}
	writel(0x73, DDR0_PUB_PIR);
	if (WAIT_FOR(DDR0_PUB_PGSR0))
		return -ETIMEDOUT;

	writel(PUB_DCR | (IS_ENABLED(CONFIG_DRAM_2T_MODE) ? 1 << 28 : 0), DDR0_PUB_DCR);

	writel(0xfc00172, DDR0_PUB_VTCR1);

	/*
	 * RUN J (2026-08-10) — bits 2 (ZUEN) + 23 (RRMODE) must be LIVE.
	 *
	 * This mask is why the RUN J header change alone was a no-op: whatever
	 * PUB_DSGCR carries, 0x800004 was stripped here before it ever reached
	 * the PHY, and DSGCR then read 0x0020441b for the rest of the boot.
	 * RUN I measured product BL2 live on this exact board at 0x00a0441f =
	 * ours | 0x800004, so the mask is what our live value is missing.
	 * Scoped to DDR4 so the DDR3 (Le Potato) path is untouched.
	 */
#if defined(CONFIG_DRAM_DDR4)
	writel(PUB_DSGCR, DDR0_PUB_DSGCR);
#else
	writel(PUB_DSGCR & ~(0x800004), DDR0_PUB_DSGCR);
#endif

	/* Wait for the SDRAM to initialise */
	return WAIT_FOR(DDR0_PUB_PGSR0);
}

void meson_dram_phy_setup_ranks(void)
{
	u32 ac = PUB_ACLCDLR;

	/*
	 * Vendor post-train 2T (gxl_bl2_ddr_deep.c, non-LPDDR): always
	 * ACLCDLR = (raw - 24) + (ACMDLR0 & 0x1ff), even when raw==0.
	 * Open previously forced ac=0 when raw==0 to avoid "underflow" —
	 * that diverges from vendor (unsigned wrap → ACMDLR-24) and is an
	 * untested open-bus residual after soft-path/DSGCR banks.
	 */
	if (IS_ENABLED(CONFIG_DRAM_2T_MODE)) {
#if defined(CONFIG_DRAM_DDR4)
		ac = (PUB_ACLCDLR - 24) + (readl(DDR0_PUB_ACMDLR0) & 0x1ff);
		writel(0x3f003f, DDR0_PUB_ACBDLR2);
		if (ac < 0x3f)
			writel(ac | (ac << 16), DDR0_PUB_ACBDLR2);
#else
		/*
		 * Same block, same BL2 source, same three arithmetic
		 * corrections as the DDR4 arm above -- they were applied
		 * there and not here, so the shipping DDR3 board (Le Potato,
		 * libretech-cc_libre_defconfig: CONFIG_DRAM_2T_MODE=y, no
		 * CONFIG_DRAM_DDR4) still takes the wrong arithmetic:
		 *
		 *   - ACMDLR0 was masked `& ~0xe00`, which clears bits 9-11
		 *     but admits everything at bit 12 and above into a 9-bit
		 *     delay field.  BL2 masks `& 0x1ff`.
		 *   - ACBDLR2 was `ac | (ac * 0xffff)`.  BL2 multiplies by
		 *     65536.  0xffff is not 0x10000, and the two differ by
		 *     2*ac for every ac -- ac here is 0x18 + (ACMDLR0 & mask),
		 *     so it is never zero and the write is always wrong.
		 *   - the `< 0x3f` guard tested PUB_ACLCDLR, the compile-time
		 *     constant, and against 62 rather than 63.  BL2 tests the
		 *     ADJUSTED value, which is the one that can be out of
		 *     range.  With PUB_ACLCDLR 0x30 (dram-settings-gx.h, the
		 *     CONFIG_DRAM_DDR3 arm) `0x30 <= 62` folds true at compile
		 *     time, so the clamp was not a clamp.
		 *
		 * KEPT, and it is why this arm is not simply the DDR4 one:
		 * PUB_ACLCDLR == 0 skips the adjustment, because
		 * (0 - 0x18) + ACMDLR0 underflows for any ACMDLR0 below 0x18
		 * and BL2 has no guard against that.  It is dead code at the
		 * shipped DDR3 value of 0x30 and costs nothing; it is left in
		 * so a future DDR3 profile with a raw 0 cannot wrap silently.
		 */
		if (PUB_ACLCDLR == 0)
			ac = 0;
		else
			ac = (PUB_ACLCDLR - 24) +
			     (readl(DDR0_PUB_ACMDLR0) & 0x1ff);
		writel(0x3f003f, DDR0_PUB_ACBDLR2);
		if (PUB_ACLCDLR != 0 && ac < 0x3f)
			writel(ac | (ac << 16), DDR0_PUB_ACBDLR2);
#endif
	}
	writel(ac, DDR0_PUB_ACLCDLR);

	if (IS_ENABLED(CONFIG_DRAM_DQS_CORR)) {
		/*
		 * Vendor post-train DQS corr (gxl_bl2_ddr_deep.c + ACS
		 * wr_adj_per/rd_adj_per).
		 * ACLCDLR *= wr[0]/100; ACBDLR0 *= wr[1]/100;
		 * per-DX LCDLR0 *= wr[2+i]/100; LCDLR2/3 *= rd[2+i]/100.
		 *
		 * The tables themselves are PER-BOARD and now live beside the
		 * other per-board PCB-compensation constants in
		 * dram-settings-gx.h (PUB_WR_ADJ_PER / PUB_RD_ADJ_PER), which
		 * carries each board's vendor values and where they came from.
		 * They were literals here, correct for the one board that
		 * compiled them; La Frite's read table is not the same.
		 *
		 * Prior open DQSCORR_DX packed 95/88 into LCDLR0 bitfields
		 * and never touched LCDLR2/3 — wrong vs vendor shape and an
		 * untested open-bus residual after ACS VREF bank.
		 */
		static const u8 wr_adj[6] = PUB_WR_ADJ_PER;
		static const u8 rd_adj[6] = PUB_RD_ADJ_PER;
		static const u32 dx_lcdlr0[4] = {
			DDR0_PUB_DX0LCDLR0, DDR0_PUB_DX1LCDLR0,
			DDR0_PUB_DX2LCDLR0, DDR0_PUB_DX3LCDLR0,
		};
		u32 v, r, r3, i;

		v = readl(DDR0_PUB_ACLCDLR) & 0x1ff;
		if (!v)
			v = 1;
		writel((v * wr_adj[0]) / 100 & 0x1ff, DDR0_PUB_ACLCDLR);

		v = readl(DDR0_PUB_ACBDLR0) & 0x1ff;
		if (!v)
			v = 1;
		writel((v * wr_adj[1]) / 100 & 0x3f, DDR0_PUB_ACBDLR0);

		for (i = 0; i < 4; i++) {
			v = readl(dx_lcdlr0[i]) & 0x1ff;
			if (!v)
				v = 1;
			/* LCDLR2 @ +8 B, LCDLR3 @ +12 B (PUB 0x1e2/0x1e3 +0x40*i) */
			r = readl(dx_lcdlr0[i] + 8) & 0x1ff;
			if (!r)
				r = 1;
			/*
			 * RUN AD: LCDLR3 (RDQSD, read-DQS data-capture delay)
			 * must be scaled from its OWN trained value.  Until now
			 * the scaled LCDLR2 (DQSGD, read-DQS GATING delay) was
			 * written into both, so the capture delay was set from
			 * the gating delay — a category error that discarded
			 * QSGATE-independent read-DQS training.  Measured effect
			 * (RUN AC): LCDLR3 == LCDLR2 on all four lanes
			 * (0x0b/0x13/0x10/0x15) where product has LCDLR3 ==
			 * LCDLR4_RDQSND (0x25/0x28/0x29/0x27).  Vendor shape in
			 * the comment above already says "LCDLR2/3 *= rd[2+i]",
			 * i.e. each from itself.
			 */
			r3 = readl(dx_lcdlr0[i] + 12) & 0x1ff;
			if (!r3)
				r3 = 1;
			writel((v * wr_adj[2 + i]) / 100, dx_lcdlr0[i]);
			writel((r * rd_adj[2 + i]) / 100, dx_lcdlr0[i] + 8);
			writel((r3 * rd_adj[2 + i]) / 100, dx_lcdlr0[i] + 12);
		}
	}
}

/*
 * The GO->ACCESS wait's own bound.  Compile-time constant, inert at 0.
 *
 * Two things were measured here on 2026-08-30 and both are worth keeping,
 * because together they say this timeout cannot be exercised by timing:
 *
 *   - shortening the SHARED PCTL_STATE_TIMEOUT_US never reaches this wait --
 *     the bit-0 waits earlier in the sequence take the same bound and fail
 *     first, so `PCTL ACCESS timeout!` is masked by `bit0 never set`;
 *   - and once separated, this bound at 100 us AND at 1 us still boots
 *     normally (1024 MB, try 1), because the controller is already in ACCESS
 *     on the first read and the loop never iterates.
 *
 * So the message stays unprinted on a healthy board by construction, not for
 * want of trying.  It guards the wedge that used to hold the console dark.
 */
#if defined(CONFIG_MESON_DRAM_PCTL_ACCESS_TIMEOUT_US) && \
	CONFIG_MESON_DRAM_PCTL_ACCESS_TIMEOUT_US > 0
#define MESON_DRAM_PCTL_ACCESS_TIMEOUT_US \
	CONFIG_MESON_DRAM_PCTL_ACCESS_TIMEOUT_US
#else
#define MESON_DRAM_PCTL_ACCESS_TIMEOUT_US	PCTL_STATE_TIMEOUT_US
#endif

int meson_dram_finalise_init(void)
{
	int timeout = MESON_DRAM_PCTL_ACCESS_TIMEOUT_US;
	u32 stat;

	writel((0x3f << 12) | 0xf8, DDR0_PUB_PGCR6);
	/*
	 * BDONE critical path: GO + ACCESS + lane GCR3/RANKIDR/REFR only.
	 * Residual pgcr45/re-GO/DSGCR forces (banked open-bus no-win) deferred
	 * until after product PUB bist — they ran before bist and left BISTS=0.
	 *
	 * The GO -> ACCESS wait was the one UNBOUNDED spin left on the DDR3
	 * path after WAIT_FOR() was bounded (N27): a hand-rolled
	 * `while ((readl(PCTL_STAT) & 7) != ACCESS);` that no macro change
	 * reached.  It sits in the silent span after `PGSR0=` where the plain
	 * libretech-cc_libre_defconfig boot of c742d38c12a wedges (2 of 2,
	 * s7-nyc, 2026-08-29), so a controller that never reaches ACCESS must
	 * say so rather than hold the console dark.  Bounded to the same 100 ms
	 * as the other waits; prints the raw STAT and fails the attempt.
	 */
	writel(PCTL_SCTL_GO_STATE, PCTL_SCTL);
	for (;;) {
		stat = readl(PCTL_STAT);
		if ((stat & 7) == PCTL_STAT_ACCESS)
			break;
		if (timeout <= 0) {
			printf("PCTL ACCESS timeout! STAT=%08x\n", stat);
			return -ETIMEDOUT;
		}
		udelay(10);
		timeout -= 10;
	}

#if defined(CONFIG_DRAM_DDR4)
	writel(0, DMC_PCTL_LP_CTRL);
#endif
	writel(0xfffc << 16, DDR0_PUB_DX0GCR3);
	writel(0xfffc << 16, DDR0_PUB_DX1GCR3);
	writel(0xfffc << 16, DDR0_PUB_DX2GCR3);
	writel(0xfffc << 16, DDR0_PUB_DX3GCR3);
	writel(0, DDR0_PUB_RANKIDR);

	/*
	 * The permanent DSGCR rewrite stays DDR3-only: it restores the bits
	 * stripped for training in meson_dram_phy_init(), which is what the
	 * vendor BL2 also does (bl2.c:1844 strips, bl2.c:2321 restores).  The
	 * DDR4 arm deliberately differs, which is a separate RUN-era lever.
	 */
#if !defined(CONFIG_DRAM_DDR4)
	writel(PUB_DSGCR, DDR0_PUB_DSGCR);
#endif
	/*
	 * ZQCR bit 2 is DDR_FUNC_ZQ_PD -- "ddr zq power down", 0:disable,
	 * 1:enable.  Vendor-named in the GXL BSP header
	 * (bl33/v2015 arch/arm/include/asm/arch-gxl/ddr_define.h:116) and
	 * glossed in board/amlogic/configs/txhd2_be319.h:341.  The GXL product
	 * BL2 drives it as a LEVEL from ddr_func bit 2 at the head of
	 * ddr_init_dmc(), immediately before the same DMC_REFR_CTRL2/CTRL1 pair
	 * written just below (reversing-gxbb-bl2/gxl/bl2.c:1551-1562), so this
	 * write is that site's counterpart and not the ZQ pulse's.
	 *
	 * On the decompiled AML-S905X-CC product image ddr_func is 0x80000000,
	 * so bit 2 is 0 and the vendor CLEARS it; the decompile's own comment
	 * on that branch reads "DDR_FUNC_ZQ_PD (disabled on lepotato)".  No BSP
	 * board configuration enables the feature: of the CONFIG_DDR_ZQ_PD
	 * lines outside the headers in V2603 V2.5, 72 are active and every one
	 * is 0; across all three BSP releases 140 are active and every one is
	 * 0.  The GXL header default is 0.
	 *
	 * DDR3 used to SET it.  That was original to the first SPL DRAM-init
	 * commit (17d80a3b326:162), was never derived from vendor behaviour,
	 * and it silently re-enabled the feature after the ZQ pulse in
	 * meson_dram_phy_finalise_init() (dram-gx.c:884) had just cleared the
	 * same bit.  Clear it on both arms, as GXBB already does
	 * (dram-gxbb.c) and as the DDR4 arm already did.
	 */
	clrbits_32(DDR0_PUB_ZQCR, 4);
	/*
	 * DMC_REFR_CTRL2 (0xc8838090), the DMC refresh generator.  Field map,
	 * vendor-named (BSP A311D_Linux2603V2.5 arch-axg/regs.h:20912-20917,
	 * same layout in 14 vendor headers):
	 *   [31:24] tZQCI  = 0x20
	 *   [23:16] tPVTI  = 0x10
	 *   [15:8]  tREFI  = timings.refi, in t100ns ticks
	 *   [7:0]   t100ns = meson_dram_clk / 20 - 1, the 100 ns prescaler
	 *           (the attempt's clock: a fallback clock keeps tREFI in ns)
	 * On GXL DDR3 (AML-S905X-CC) and on the CC-V2's DDR4,
	 * timings.refi is 38 (3.8 us), legal across the part's whole Tc
	 * 0..95 C range; see dram-timings-gx.h (AML-S805X-AC's DDR4 keeps
	 * 76 until its own governor lands).  This owns refresh correctness on
	 * the open chain (operator decision D14), and the open SC governor
	 * relaxes it while cool; on the vendor chain, where BL2 trains, the
	 * AML-S905X-CC and AML-S905X-CC-V2 board code lowers tREFI to 38.
	 */
	writel(0x20100000 | ((meson_dram_clk / 20) - 1) |
		(timings.refi << 8), DMC_REFR_CTRL2);
	writel(0xf08f, DMC_REFR_CTRL1);

	return 0;
}

