// SPDX-License-Identifier: GPL-2.0+
/*
 * Portions Copyright (C) 2015, Amlogic, Inc. All rights reserved.
 * Copyright (C) 2023, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#include <hang.h>
#include <image.h>
#include <spl.h>
#include <vsprintf.h>
#include <asm/io.h>
#include <asm/arch/boot.h>
#include <asm/arch/clock-gx.h>
#include <asm/arch/dram-gx.h>
#include <asm/arch/gx-bootlog.h>
#include <asm/arch/gx.h>
#include <linux/delay.h>

/* Meson GX SPL code */

/*
 * GXL AO UART: WFIFO 0xc81004c0, STATUS 0xc81004cc (base + 0x0c), TX-full at
 * bit 21.  Fourth copy of this helper in the tree; the other three -- spl.c,
 * spl-dram-stage-gx.c, dram-g12.c -- all read STATUS at base + 0x0c and all
 * bound the spin.
 *
 * 🔴 BOTH OF THOSE WERE WRONG HERE.  This read 0xc81004dc, which is base +
 * 0x1c, past REG5 at 0x14 and OUTSIDE the AO UART's register block, and it spun
 * on it forever.  spl.c:1142-1143 states why the bound is not optional: "Never
 * infinite-wait: if AO UART is unclocked/stuck, USB boot still needs to reach
 * meson_usb_reenum_serve()."
 *
 * ⚪ CONSEQUENCE-LIMITED TODAY, DELIBERATELY FIXED ANYWAY.  early_puthex() has
 * no caller in this file and early_putc() has none outside it, so both helpers
 * are dead code and neither defect has ever executed.  It is a landmine rather
 * than a live bug: the first debug call added to spl-gx.c would have got an
 * out-of-block poll with no timeout, and whether that reads 0 (characters
 * silently dropped) or all-ones (SPL hangs before the USB stack) needs
 * hardware to say.
 */
#define GX_EARLY_UART_WFIFO	0xc81004c0UL
#define GX_EARLY_UART_STATUS	0xc81004ccUL

static void early_putc(char c)
{
	int timeout = 100000;

	while ((readl((void *)GX_EARLY_UART_STATUS) & (1 << 21)) && --timeout)
		;
	if (timeout)
		writel(c, (void *)GX_EARLY_UART_WFIFO);
	gx_bootlog_putc(c);
}

static void early_puthex(u32 val)
{
	int i;

	for (i = 28; i >= 0; i -= 4) {
		u8 nib = (val >> i) & 0xf;
		early_putc(nib < 10 ? '0' + nib : 'a' + nib - 10);
	}
	early_putc('\r');
	early_putc('\n');
}

void cpu_pll_switch_to(int mode)
{
	u32 reg;

	reg = readl(GX_HIU_BASE + HHI_SYS_CPU_CLK_CNTL0);

	while (reg & HHI_SCC_CNTL0_BUSY)
		reg = readl(GX_HIU_BASE + HHI_SYS_CPU_CLK_CNTL0);

	reg |= HHI_SCC_CNTL0_DYN_ENABLE;

	if (mode == 1) {
		/* Switch to System PLL */
		reg |= HHI_SCC_CNTL0_FINAL_MUX_SEL;
	} else {
		if (reg & HHI_SCC_CNTL0_FINAL_DYN_MUX_SEL) {
			reg = (reg & ~(HHI_SCC_CNTL0_FINAL_DYN_MUX_SEL |
				HHI_SCC_CNTL0_MUX0_DIVN_TCNT |
				HHI_SCC_CNTL0_POSTMUX0 | HHI_SCC_CNTL0_PREMUX0));
		} else {
			reg = (reg & ~(HHI_SCC_CNTL0_FINAL_DYN_MUX_SEL |
				HHI_SCC_CNTL0_MUX1_DIVN_TCNT |
				(HHI_SCC_CNTL0_POSTMUX1 | HHI_SCC_CNTL0_PREMUX1))) |
				HHI_SCC_CNTL0_FINAL_DYN_MUX_SEL;
		}
		/* Select dynamic mux */
		reg = reg & ~(HHI_SCC_CNTL0_FINAL_MUX_SEL) /*final_mux_sel*/;
	}
	writel(reg, GX_HIU_BASE + HHI_SYS_CPU_CLK_CNTL0);
}

/*
 * If BROM left SYS PLL locked, mux CPU onto it without reconfiguring MPLL
 * (full meson_pll_init kills worldcup USB — p47). Used before receive_fit.
 * Return: 1 switched, 0 not locked.
 */
int meson_cpu_try_sys_pll(void)
{
	u32 pll = readl(GX_HIU_BASE + HHI_SYS_PLL_CNTL);

	if (!(pll & BIT(31)))
		return 0;
	cpu_pll_switch_to(1);
	return 1;
}

/*
 * Force SYS PLL to 1.2 GHz and mux CPU onto it.  Touches ONLY
 * HHI_SYS_PLL_CNTL* + CPU mux — never MPLL / MPEG (those kill worldcup
 * USB, p47).  Busy-wait lock; no timer dependency.
 * Return: 1 locked+muxed, 0 lock fail (CPU left on dyn mux).
 */
int meson_cpu_sys_pll_force_1g2(void)
{
	int retries = 200;
	u32 cntl;

	/* Dyn/backup clock while reprogramming SYS PLL */
	cpu_pll_switch_to(0);

	/* GXL SYS PLL 1.2 GHz (same values as meson_pll_init, no MPLL) */
	writel(0xc4258100, GX_HIU_BASE + HHI_SYS_PLL_CNTL1);
	writel(0xb7400000, GX_HIU_BASE + HHI_SYS_PLL_CNTL2);
	writel(0xa59a288, GX_HIU_BASE + HHI_SYS_PLL_CNTL3);
	writel(0x40002d, GX_HIU_BASE + HHI_SYS_PLL_CNTL4);
	writel(0x7c700007, GX_HIU_BASE + HHI_SYS_PLL_CNTL5);
	/* enable + N=50 (1200/24); leave OD=0 */
	writel((1u << 30) | ((1u << 9) | (1200 / 24)),
	       GX_HIU_BASE + HHI_SYS_PLL_CNTL);

	do {
		unsigned int i;
		/* ~20 us busy at unknown CPU rate */
		for (i = 0; i < 2000; i++)
			asm volatile("nop");
		cntl = readl(GX_HIU_BASE + HHI_SYS_PLL_CNTL);
		if (cntl & BIT(31)) {
			cpu_pll_switch_to(1);
			return 1;
		}
	} while (--retries > 0);

	return 0;
}

/*
 * Enable the fixed-PLL /3 output (fclk_div3).
 *
 * The BootROM leaves HHI_MPLL_CNTL10 == 0, and in that state the fixed
 * PLL's /3 tap produces NOTHING while /2, /4, /5 and /7 all run -- even
 * though fclk_div3's documented gate (HHI_MPLL_CNTL6 bit 28) is already
 * set.  Every consumer that selects fclk_div3 then gets 0 Hz, and an
 * unclocked slave on a bus with no timeout stalls the access forever
 * instead of aborting -- the meson VPU's first VCBUS read is the one that
 * bit us.
 *
 * 0x1003 is the vendor value: product BL2 for this SoC stores it with
 * "movw r2, #0x1003 / str r2, [HHI_MPLL_CNTL10]", and meson_pll_init()
 * below writes the same constant on its GXL arm.  The GXL USB-SPL path
 * cannot call meson_pll_init() -- the full MPLL re-init kills the worldcup
 * USB gadget the DRAM-boot loop depends on -- so this one register, the
 * only part of that sequence fclk_div3 needs, is written on its own.
 *
 * Measured on AML-S905X-CC-V2 with the SoC's own clock-measure block
 * (channel 46 "vpu", HHI_VPU_CLK_CNTL mux 1 = fclk_div3, /1):
 *   HHI_MPLL_CNTL10 = 0      ->         0 Hz, VCBUS 0xd0109e40 stalls
 *   HHI_MPLL_CNTL10 = 0x1003 -> 666 664 000 Hz, 0xd0109e40 = 0x00210000
 * reproduced on/off/on in a single boot.  Linux needs it too: meson-gxl.dtsi
 * assigns vpu_0_sel = CLKID_FCLK_DIV3 and gxbb.c marks fclk_div3
 * CLK_IS_CRITICAL, so CCF assumes it is already running and never enables it.
 */
void meson_fixed_pll_enable_div3(void)
{
	writel(0x1003, GX_HIU_BASE + HHI_MPLL_CNTL10);
}

/*
 * Read back the two registers this stage writes but cannot observe.
 *
 * Three defects on this chain were invisible to the stage that caused
 * them and were found only by a LATER stage failing:
 *
 *  - DMC_SEC_AXI_PORT_CTRL left every AXI port secure-only.  The SPL runs
 *    at EL3 with the MMU off, so every access it makes is secure and
 *    succeeds; the first NON-secure DRAM read, in BL31, took an external
 *    abort that a release TF-A reports as silence.
 *  - HHI_MPLL_CNTL10 was never written, so fclk_div3 produced 0 Hz.
 *    Nothing in the boot path reads it back, and the consumer that cared
 *    (the VPU's first VCBUS access) stalled a bus with no timeout -- no
 *    abort, no console output.
 *
 * Neither can be turned into a self-TEST here: the SPL cannot issue a
 * non-secure access, and it has no fclk_div3 consumer of its own.  What it
 * CAN do is state the value, so the record carries what was programmed
 * rather than what was intended.  Printed, not enforced -- a stage that
 * refuses to continue on a value it cannot itself validate would be worse
 * than one that reports it.
 *
 * NOT DDR4-scoped any more.  It was, on a headroom argument, and that scoping
 * removed the instrument from the only board whose SECAXI value is in doubt:
 * the DDR4 path WRITES 0xffffffff to DMC_SEC_AXI_PORT_CTRL, the DDR3 path
 * writes it nowhere, and it was the DDR3 board (AML-S905X-CC v1) that stopped
 * dead at the ERET to BL33 while its DDR4 sibling reached the boot menu on the
 * same chain.  An instrument that only runs where the answer is already known
 * measures nothing.  Headroom is not the constraint it was: SPL_MAX_SIZE is
 * 0x1fe00 against a 37 KiB image.
 */
void meson_spl_selfcheck(void)
{
	u32 secaxi = readl(DMC_SEC_AXI_PORT_CTRL);
	u32 mpll10 = readl(GX_HIU_BASE + HHI_MPLL_CNTL10);
	bool bad = secaxi != 0xffffffff || mpll10 != 0x1003;

	/*
	 * Expected: SECAXI ffffffff (all ports secure+non-secure),
	 * MPLL10 00001003 (fclk_div3 enabled). On a good boot the words are a
	 * bring-up diagnostic (MESON_GX_XPL_VERBOSE); a BAD one always prints.
	 */
	if (!bad && !IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
		return;
	printf("SELFCHK: SECAXI %08x%s MPLL10 %08x%s\n",
	       secaxi, secaxi == 0xffffffff ? "" : " BAD",
	       mpll10, mpll10 == 0x1003 ? "" : " BAD");

	/*
	 * The DMC secure-range bank, printed for the same reason: it is not
	 * readable from non-secure U-Boot at all (the bank itself refuses),
	 * so EL3 here is the only stage that can state what the BootROM left
	 * programmed.  A non-secure read below 0x00100000 on this board takes
	 * a synchronous external abort (esr 0x96000210, DFSC 0x10) while
	 * 0x00100000 reads fine, and one of these six windows is that wall.
	 *
	 * Each RANGEn word is [15:0] start >> 16, [31:16] end >> 16.  Whether
	 * the end field is inclusive is NOT documented for this register (the
	 * ">= start && <= end" note in the vendor header belongs to
	 * DMC_PROT0_RANGE, a different bank) -- reading the live value against
	 * the known 1 MiB wall is what settles it.
	 */
	printf("SELFCHK: SECCTRL %08x RANGE0 %08x RANGE1 %08x RANGE2 %08x\n",
	       readl(DMC_SEC_CTRL), readl(DMC_SEC_RANGE0_CTRL),
	       readl(DMC_SEC_RANGE1_CTRL), readl(DMC_SEC_RANGE2_CTRL));
	printf("SELFCHK: RANGE3 %08x RANGE4 %08x RANGE5 %08x RANGECTL %08x\n",
	       readl(DMC_SEC_RANGE3_CTRL), readl(DMC_SEC_RANGE4_CTRL),
	       readl(DMC_SEC_RANGE5_CTRL), readl(DMC_SEC_RANGE_CTRL));
}

/*
 * Publish the DOS capability word the vendor's BL2 publishes and ours did not.
 *
 * The vendor GXL boot chain leaves DOS_SCRATCH31 = 0x00000004 and VP9 decodes;
 * our open chain left it 0x00000000 and VP9 returned zero frames with zero
 * decoder interrupts, its AMRISC parked for ever on the BIT(2) spin at word
 * 0x92f.  Everything else about the two chains' VDEC_HEVC register state is
 * byte-identical (N95).  This is the one-word parity gap, closed in the stage
 * that replaces the vendor's BL2 rather than in the decoder driver, where
 * masking it would be a fix at the wrong layer.
 *
 * Read-modify-write, never a bare store: bit 3 of the same word is the S805X
 * H.264 1080p cap, and any other bit a later stage or a future SoC uses is
 * none of this function's business.
 *
 * The DOS block must be clocked for the store to land -- an MMIO access into an
 * unclocked DOS block resets the SoC.  HHI_GCLK_MPEG0 bit 1 is that gate; it is
 * measured 0xffffffff at this point in xPL (status.md, "xPL clock question
 * settled"), so the enable below is belt-and-braces rather than the normal
 * path, and it is printed so a boot that needed it says so.
 */
void meson_gx_dos_publish_caps(void)
{
	u32 gclk = readl(GX_HIU_BASE + HHI_GCLK_MPEG0);
	u32 before, after;

	if (!(gclk & BIT(1))) {
		setbits_le32(GX_HIU_BASE + HHI_GCLK_MPEG0, BIT(1));
		readl(GX_HIU_BASE + HHI_GCLK_MPEG0);
	}

	before = readl(GX_DOS_SCRATCH31);
	writel(before | GX_DOS_SCRATCH31_VP9_CAP, GX_DOS_SCRATCH31);
	after = readl(GX_DOS_SCRATCH31);

	/* A diagnostic when it took (MESON_GX_XPL_VERBOSE); BAD always prints. */
	if (!(after & GX_DOS_SCRATCH31_VP9_CAP) || IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
		printf("DOSCAP: gclk0 %08x SCRATCH31 %08x -> %08x%s\n",
		       gclk, before, after,
		       (after & GX_DOS_SCRATCH31_VP9_CAP) ? "" : " BAD");
}

int meson_pll_init(void)
{
	int pll_retries = 100;

	/* Skip MPEG disable + cpu_pll_switch_to(0) for S805X debugging */

	setbits_32(GX_HIU_BASE + HHI_MPLL_CNTL6, 1 << 26);
	udelay(100);

	/*
	 * Always reconfigure SYS_PLL -- even if already locked from ROM.
	 * cpu_pll_switch_to(0) puts CPU on a slow backup clock;
	 * we must set up and lock the PLL before switching back.
	 */
	do {
		if (IS_ENABLED(CONFIG_MESON_GXBB)) {
			setbits_32(GX_HIU_BASE + HHI_SYS_PLL_CNTL, 1 << 29);
			writel(0x5ac80000, GX_HIU_BASE + HHI_SYS_PLL_CNTL2);
			writel(0x8e452015, GX_HIU_BASE + HHI_SYS_PLL_CNTL3);
			writel(0x401d40c, GX_HIU_BASE + HHI_SYS_PLL_CNTL4);
			writel(0x870, GX_HIU_BASE + HHI_SYS_PLL_CNTL5);
			writel((1 << 30) | (1 << 29) |
				((0 << 16) | (1 << 9) |
				(1536 / 24)), /* 1.5 GHz */
				GX_HIU_BASE + HHI_SYS_PLL_CNTL);
			clrbits_32(GX_HIU_BASE + HHI_SYS_PLL_CNTL, 1 << 29);
		} else if (IS_ENABLED(CONFIG_MESON_GXL)) {
			writel(0xc4258100, GX_HIU_BASE + HHI_SYS_PLL_CNTL1);
			writel(0xb7400000, GX_HIU_BASE + HHI_SYS_PLL_CNTL2);
			writel(0xa59a288, GX_HIU_BASE + HHI_SYS_PLL_CNTL3);
			writel(0x40002d, GX_HIU_BASE + HHI_SYS_PLL_CNTL4);
			writel(0x7c700007, GX_HIU_BASE + HHI_SYS_PLL_CNTL5);
			writel((1 << 30) | ((1 << 9) |
				(1200 / 24)), /* 1.2 GHz */
				GX_HIU_BASE + HHI_SYS_PLL_CNTL);
		}
		udelay(20);
	} while (!((readl(GX_HIU_BASE + HHI_SYS_PLL_CNTL) >> 31) & 1) &&
		 --pll_retries > 0);

	if (pll_retries <= 0) {
		puts("SYS PLL LOCK FAIL\n");
		return -1;
	}

	cpu_pll_switch_to(1);

	if (IS_ENABLED(CONFIG_MESON_GXBB))
		writel(0x10007, GX_HIU_BASE + HHI_MPLL_CNTL4);
	else if (IS_ENABLED(CONFIG_MESON_GXL))
		writel(0x10006, GX_HIU_BASE + HHI_MPLL_CNTL4);

	setbits_32(GX_HIU_BASE + HHI_MPLL_CNTL, 1 << 29);
	udelay(200);

	writel(0x59C80000, GX_HIU_BASE + HHI_MPLL_CNTL2);
	writel(0xCA753822, GX_HIU_BASE + HHI_MPLL_CNTL3); /* vendor bl2.c:1065 */

	if (IS_ENABLED(CONFIG_MESON_GXBB))
		writel(0xB5500E1A, GX_HIU_BASE + HHI_MPLL_CNTL5);
	else if (IS_ENABLED(CONFIG_MESON_GXL))
		writel(0x95520E1A, GX_HIU_BASE + HHI_MPLL_CNTL5);

	writel(0xFC454545, GX_HIU_BASE + HHI_MPLL_CNTL6);

	if (IS_ENABLED(CONFIG_MESON_GXBB)) {
		writel((1 << 30) | (1 << 29) | (3 << 9) | (250 << 0), GX_HIU_BASE + HHI_MPLL_CNTL);
		clrbits_32(GX_HIU_BASE + HHI_MPLL_CNTL, 1 << 29);
	} else if (IS_ENABLED(CONFIG_MESON_GXL)) {
		writel((1 << 30) | (3 << 9) | (250 << 0), GX_HIU_BASE + HHI_MPLL_CNTL);
	}
	udelay(800);

	setbits_32(GX_HIU_BASE + HHI_MPLL_CNTL4, 1 << 14);

	while (!((readl(GX_HIU_BASE + HHI_MPLL_CNTL) >> 31) & 1)) {
		if ((readl(GX_HIU_BASE + HHI_MPLL_CNTL) & (1 << 31)) != 0)
			break;
		setbits_32(GX_HIU_BASE + HHI_MPLL_CNTL, 1 << 29);
		udelay(1000);
		clrbits_32(GX_HIU_BASE + HHI_MPLL_CNTL, 1 << 29);
		udelay(1000);
	}

	if (IS_ENABLED(CONFIG_MESON_GXBB)) {
		writel(0xFFF << 16, GX_HIU_BASE + HHI_MPLL_CNTL10);
		writel(((7 << 16) | (1 << 15) | (1 << 14) | (4681 << 0)),
		       GX_HIU_BASE + HHI_MPLL_CNTL7);
		writel(((readl(GX_HIU_BASE + HHI_MPEG_CLK_CNTL) & (~((0x7 << 12) | (1 << 7) |
			(0x7F << 0)))) | ((5 << 12) | (1 << 7)	| (2 << 0))),
			GX_HIU_BASE + HHI_MPEG_CLK_CNTL);
		setbits_32(GX_HIU_BASE + HHI_MPEG_CLK_CNTL, 1 << 8);
		writel(((5 << 16) | (1 << 15) | (1 << 14) | (12524 << 0)),
		       GX_HIU_BASE + HHI_MPLL_CNTL8);
	} else if (IS_ENABLED(CONFIG_MESON_GXL)) {
		/*
		 * FIX_PLL re-lock (vendor bl2.c:1079-1086).
		 * MPLL_CNTL is the FIX_PLL control register on GXL.
		 * Toggle reset (bit 29) if not already locked (bit 31).
		 */
		while (!((readl(GX_HIU_BASE + HHI_MPLL_CNTL) >> 31) & 1)) {
			setbits_32(GX_HIU_BASE + HHI_MPLL_CNTL, 1 << 29);
			udelay(1000);
			clrbits_32(GX_HIU_BASE + HHI_MPLL_CNTL, 1 << 29);
			udelay(1000);
		}

		writel(0x1003, GX_HIU_BASE + HHI_MPLL_CNTL10);
		writel(0x5edb7, GX_HIU_BASE + HHI_MPLL_CNTL7);
		/* Vendor bl2.c:1094: (val & 0xffff8f00) | 0x5182 */
		clrsetbits_32(GX_HIU_BASE + HHI_MPEG_CLK_CNTL,
			      0x70ff, 0x5182);
		writel(0x4e15a, GX_HIU_BASE + HHI_MPLL_CNTL8);
	}

	udelay(200);

	/* TODO: Some error handling and timeouts... */
	return 0;
}

#if CONFIG_IS_ENABLED(MMC) && !CONFIG_IS_ENABLED(DM_MMC)
int board_mmc_init(struct bd_info *bis)
{
	int mmc_device;

	switch (meson_get_boot_device()) {
	case BOOT_DEVICE_SD:
		mmc_device = 0;
		break;
	case BOOT_DEVICE_EMMC:
		mmc_device = 1;
		break;
	default:
		return -1;
	}

	if (!meson_mmc_init(mmc_device))
		return -1;

	return 0;
}
#endif
