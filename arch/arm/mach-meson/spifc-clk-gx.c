// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX: the SPI flash controller's input rate, without a clock tree.
 *
 * drivers/spi/meson_spifc.c computes its divider as a function of the parent
 * rate: n = parent / speed - 1.  In an xPL phase built without the CLK uclass
 * -- which is every phase under the 0xb000 amlimage ceiling, because
 * drivers/clk/meson/gxbb.c alone overflows .sram -- <clk.h>'s stub returns
 * -ENOSYS and that arithmetic runs on 0xffff...ffdc.  Tolerating the -ENOSYS
 * the way meson_gx_mmc.c does would therefore be strictly worse than the
 * previous refusal: it would bind the controller and then write a garbage
 * divider.
 *
 * The controller does not need a clock TREE, it needs a NUMBER, and the number
 * is already in two registers the ROM has finished with.  So this file answers
 * the one question the driver asks, by reading:
 *
 *   fixed PLL  <- HHI_MPLL_CNTL  (M/N/OD, the same fields
 *                 drivers/clk/meson/gxbb.c's meson_pll_get_rate() reads)
 *   clk81      <- HHI_MPEG_CLK_CNTL  (mux at [14:12], zero-based divider at
 *                 [6:0], the same fields meson_clk81_get_rate() reads)
 *
 * and nothing else.  No uclass, no regmap, no syscon, no gxbb.c.
 *
 * Measured on AML-S905X-CC-V2 2026-08-12, read from that board's own u-boot
 * prompt on the shipping single-stage image:
 *
 *   HHI_MPLL_CNTL     0xc883c280 = 0xc00006fa
 *                     -> m 250, n 3, od 0 -> 24 MHz * 250 / 3 = 2000 MHz
 *   HHI_MPEG_CLK_CNTL 0xc883c174 = 0x00005182
 *                     -> mux 5 = fclk_div4 = 500 MHz, divider field 2 -> /3
 *                     -> clk81 = 166 666 666 Hz
 *   SPIFC REG_CLOCK   0xc1108c98 = 0x00001001
 *                     -> n = 1, which is exactly max(166666666/83333333 - 1, 1)
 *
 * The last line is the cross-check that matters: it is what the FULL u-boot,
 * with the whole CLK uclass behind it, programmed into the controller.  This
 * file has to reproduce that value from the registers alone, and it does.
 */

#include <errno.h>
#include <asm/io.h>
#include <asm/arch/gx.h>
#include <asm/arch/clock-gx.h>

#define GX_XTAL_RATE		24000000UL

/* HHI_MPLL_CNTL, the fixed PLL: gxbb.c meson_fixed_pll_parm[]. */
#define GX_MPLL_M_SHIFT		0
#define GX_MPLL_M_MASK		0x1ff
#define GX_MPLL_N_SHIFT		9
#define GX_MPLL_N_MASK		0x1f
#define GX_MPLL_OD_SHIFT	16
#define GX_MPLL_OD_MASK		0x3

/* HHI_MPEG_CLK_CNTL, clk81: gxbb.c meson_clk81_get_rate(). */
#define GX_MPEG_MUX_SHIFT	12
#define GX_MPEG_MUX_MASK	0x7
#define GX_MPEG_DIV_MASK	0x7f

static ulong gx_fixed_pll_rate(void)
{
	u32 reg = readl((void *)(ulong)(GX_HIU_BASE + HHI_MPLL_CNTL));
	u32 m = (reg >> GX_MPLL_M_SHIFT) & GX_MPLL_M_MASK;
	u32 n = (reg >> GX_MPLL_N_SHIFT) & GX_MPLL_N_MASK;
	u32 od = (reg >> GX_MPLL_OD_SHIFT) & GX_MPLL_OD_MASK;

	if (!n || !m)
		return 0;

	/* MHz first, exactly as gxbb.c does, so the rounding matches. */
	return (((GX_XTAL_RATE / 1000000) * m / n) >> od) * 1000000UL;
}

/**
 * meson_spifc_parent_rate() - clk81, read out of the HIU registers
 *
 * Return: rate in Hz, or 0 when the mux is on a source this file will not
 *         derive (MPLL1/MPLL2, which would need the MPLL SDM math, and which
 *         no GX board has been seen to use for clk81).
 *
 * Overrides the __weak stub in drivers/spi/meson_spifc.c.  Returning 0 makes
 * the driver refuse the transfer rather than divide by a value it invented --
 * a stage that stops is worth more than one that silently clocks the part
 * wrong.
 */
ulong meson_spifc_parent_rate(void)
{
	u32 reg = readl((void *)(ulong)(GX_HIU_BASE + HHI_MPEG_CLK_CNTL));
	u32 mux = (reg >> GX_MPEG_MUX_SHIFT) & GX_MPEG_MUX_MASK;
	u32 div = reg & GX_MPEG_DIV_MASK;
	ulong parent;

	switch (mux) {
	case 0:
		parent = GX_XTAL_RATE;
		break;
	case 2:
		parent = gx_fixed_pll_rate() / 7;
		break;
	case 5:
		parent = gx_fixed_pll_rate() / 4;
		break;
	case 6:
		parent = gx_fixed_pll_rate() / 3;
		break;
	case 7:
		parent = gx_fixed_pll_rate() / 5;
		break;
	default:
		/* 1 is reserved; 3 and 4 are MPLL1/MPLL2. */
		return 0;
	}

	if (!parent)
		return 0;

	/* The clk81 divider field is zero-based. */
	return parent / (div + 1);
}
