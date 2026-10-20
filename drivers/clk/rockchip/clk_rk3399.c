// SPDX-License-Identifier: GPL-2.0
/*
 * (C) Copyright 2015 Google, Inc
 * (C) 2017 Theobroma Systems Design und Consulting GmbH
 */

#include <clk-uclass.h>
#include <dm.h>
#include <dt-structs.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <mapmem.h>
#include <syscon.h>
#include <bitfield.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/cru.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/global_data.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <dt-bindings/clock/rk3399-cru.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/printk.h>

DECLARE_GLOBAL_DATA_PTR;

#if CONFIG_IS_ENABLED(OF_PLATDATA)
struct rk3399_clk_plat {
	struct dtd_rockchip_rk3399_cru dtd;
};

struct rk3399_pmuclk_plat {
	struct dtd_rockchip_rk3399_pmucru dtd;
};
#endif

struct pll_div {
	u32 refdiv;
	u32 fbdiv;
	u32 postdiv1;
	u32 postdiv2;
	u32 frac;
};

#define RATE_TO_DIV(input_rate, output_rate) \
	((input_rate) / (output_rate) - 1)
#define DIV_TO_RATE(input_rate, div)		((input_rate) / ((div) + 1))

#define PLL_DIVISORS(hz, _refdiv, _postdiv1, _postdiv2) {\
	.refdiv = _refdiv,\
	.fbdiv = (u32)((u64)hz * _refdiv * _postdiv1 * _postdiv2 / OSC_HZ),\
	.postdiv1 = _postdiv1, .postdiv2 = _postdiv2};

static const struct pll_div gpll_init_cfg = PLL_DIVISORS(GPLL_HZ, 2, 2, 1);
static const struct pll_div cpll_init_cfg = PLL_DIVISORS(CPLL_HZ, 1, 2, 2);
/*
 * NPLL 1200 MHz as VCO 2400 / 2 (refdiv 1, fbdiv 100): the same VCO Linux
 * runs CPLL at for its 800 MHz card-clock source (rk3399_pll_rates: 800 MHz
 * = 1 / 100 / 3 / 1), so the SD eye measured on the two is compared at one
 * VCO, not two.
 */
static const struct pll_div npll_init_cfg = PLL_DIVISORS(NPLL_HZ, 1, 2, 1);
#if !defined(CONFIG_XPL_BUILD)
static const struct pll_div ppll_init_cfg = PLL_DIVISORS(PPLL_HZ, 2, 2, 1);
#endif

static const struct pll_div apll_l_1600_cfg = PLL_DIVISORS(1600 * MHz, 3, 1, 1);
static const struct pll_div apll_l_600_cfg = PLL_DIVISORS(600 * MHz, 1, 2, 1);

/* VCO = the rate (postdiv 1, 1): 1008 / 1200 / 1416 MHz, inside 800..3200 */
static const struct pll_div apll_l_1008_cfg = PLL_DIVISORS(1008 * MHz, 1, 1, 1);
static const struct pll_div apll_l_1200_cfg = PLL_DIVISORS(1200 * MHz, 1, 1, 1);
static const struct pll_div apll_l_1416_cfg = PLL_DIVISORS(1416 * MHz, 1, 1, 1);

static const struct pll_div *apll_l_cfgs[] = {
	[APLL_L_1600_MHZ] = &apll_l_1600_cfg,
	[APLL_L_600_MHZ] = &apll_l_600_cfg,
	[APLL_L_1008_MHZ] = &apll_l_1008_cfg,
	[APLL_L_1200_MHZ] = &apll_l_1200_cfg,
	[APLL_L_1416_MHZ] = &apll_l_1416_cfg,
};

static const struct pll_div apll_b_600_cfg = PLL_DIVISORS(600 * MHz, 1, 2, 1);
static const struct pll_div *apll_b_cfgs[] = {
	[APLL_B_600_MHZ] = &apll_b_600_cfg,
};

enum {
	/* PLL_CON0 */
	PLL_FBDIV_MASK			= 0xfff,
	PLL_FBDIV_SHIFT			= 0,

	/* PLL_CON1 */
	PLL_POSTDIV2_SHIFT		= 12,
	PLL_POSTDIV2_MASK		= 0x7 << PLL_POSTDIV2_SHIFT,
	PLL_POSTDIV1_SHIFT		= 8,
	PLL_POSTDIV1_MASK		= 0x7 << PLL_POSTDIV1_SHIFT,
	PLL_REFDIV_MASK			= 0x3f,
	PLL_REFDIV_SHIFT		= 0,

	/* PLL_CON2 */
	PLL_LOCK_STATUS_SHIFT		= 31,
	PLL_LOCK_STATUS_MASK		= 1 << PLL_LOCK_STATUS_SHIFT,
	PLL_FRACDIV_MASK		= 0xffffff,
	PLL_FRACDIV_SHIFT		= 0,

	/* PLL_CON3 */
	/* CLKSEL_CON32 */
	SPDIF_REC_DPTX_PLL_SEL_SHIFT	= 15,
	SPDIF_REC_DPTX_PLL_SEL_MASK	= 1 << SPDIF_REC_DPTX_PLL_SEL_SHIFT,
	SPDIF_REC_DPTX_PLL_SEL_CPLL	= 0,
	SPDIF_REC_DPTX_PLL_SEL_GPLL	= 1,
	SPDIF_REC_DPTX_DIV_CON_SHIFT	= 8,
	SPDIF_REC_DPTX_DIV_CON_MASK	= 0x1f << SPDIF_REC_DPTX_DIV_CON_SHIFT,

	/* CLKSEL_CON46 */
	DP_CORE_PLL_SEL_SHIFT		= 6,
	DP_CORE_PLL_SEL_MASK		= 0x3 << DP_CORE_PLL_SEL_SHIFT,
	DP_CORE_PLL_SEL_NPLL		= 0,
	DP_CORE_PLL_SEL_CPLL		= 1,
	DP_CORE_PLL_SEL_GPLL		= 2,
	DP_CORE_DIV_CON_SHIFT		= 0,
	DP_CORE_DIV_CON_MASK		= 0x1f << DP_CORE_DIV_CON_SHIFT,

	PLL_MODE_SHIFT			= 8,
	PLL_MODE_MASK			= 3 << PLL_MODE_SHIFT,
	PLL_MODE_SLOW			= 0,
	PLL_MODE_NORM,
	PLL_MODE_DEEP,
	PLL_DSMPD_SHIFT			= 3,
	PLL_DSMPD_MASK			= 1 << PLL_DSMPD_SHIFT,
	PLL_INTEGER_MODE		= 1,

	/* PMUCRU_CLKSEL_CON0 */
	PMU_PCLK_DIV_CON_MASK		= 0x1f,
	PMU_PCLK_DIV_CON_SHIFT		= 0,

	/* PMUCRU_CLKSEL_CON1 */
	SPI3_PLL_SEL_SHIFT		= 7,
	SPI3_PLL_SEL_MASK		= 1 << SPI3_PLL_SEL_SHIFT,
	SPI3_PLL_SEL_24M		= 0,
	SPI3_PLL_SEL_PPLL		= 1,
	SPI3_DIV_CON_SHIFT		= 0x0,
	SPI3_DIV_CON_MASK		= 0x7f,

	/* PMUCRU_CLKSEL_CON2 */
	I2C_DIV_CON_MASK		= 0x7f,
	CLK_I2C8_DIV_CON_SHIFT		= 8,
	CLK_I2C0_DIV_CON_SHIFT		= 0,

	/* PMUCRU_CLKSEL_CON3 */
	CLK_I2C4_DIV_CON_SHIFT		= 0,

	/* CLKSEL_CON0 */
	ACLKM_CORE_L_DIV_CON_SHIFT	= 8,
	ACLKM_CORE_L_DIV_CON_MASK	= 0x1f << ACLKM_CORE_L_DIV_CON_SHIFT,
	CLK_CORE_L_PLL_SEL_SHIFT	= 6,
	CLK_CORE_L_PLL_SEL_MASK		= 3 << CLK_CORE_L_PLL_SEL_SHIFT,
	CLK_CORE_L_PLL_SEL_ALPLL	= 0x0,
	CLK_CORE_L_PLL_SEL_ABPLL	= 0x1,
	CLK_CORE_L_PLL_SEL_DPLL		= 0x10,
	CLK_CORE_L_PLL_SEL_GPLL		= 0x11,
	CLK_CORE_L_DIV_MASK		= 0x1f,
	CLK_CORE_L_DIV_SHIFT		= 0,

	/* CLKSEL_CON1 */
	PCLK_DBG_L_DIV_SHIFT		= 0x8,
	PCLK_DBG_L_DIV_MASK		= 0x1f << PCLK_DBG_L_DIV_SHIFT,
	ATCLK_CORE_L_DIV_SHIFT		= 0,
	ATCLK_CORE_L_DIV_MASK		= 0x1f << ATCLK_CORE_L_DIV_SHIFT,

	/* CLKSEL_CON2 */
	ACLKM_CORE_B_DIV_CON_SHIFT	= 8,
	ACLKM_CORE_B_DIV_CON_MASK	= 0x1f << ACLKM_CORE_B_DIV_CON_SHIFT,
	CLK_CORE_B_PLL_SEL_SHIFT	= 6,
	CLK_CORE_B_PLL_SEL_MASK		= 3 << CLK_CORE_B_PLL_SEL_SHIFT,
	CLK_CORE_B_PLL_SEL_ALPLL	= 0x0,
	CLK_CORE_B_PLL_SEL_ABPLL	= 0x1,
	CLK_CORE_B_PLL_SEL_DPLL		= 0x10,
	CLK_CORE_B_PLL_SEL_GPLL		= 0x11,
	CLK_CORE_B_DIV_MASK		= 0x1f,
	CLK_CORE_B_DIV_SHIFT		= 0,

	/* CLKSEL_CON3 */
	PCLK_DBG_B_DIV_SHIFT		= 0x8,
	PCLK_DBG_B_DIV_MASK		= 0x1f << PCLK_DBG_B_DIV_SHIFT,
	ATCLK_CORE_B_DIV_SHIFT		= 0,
	ATCLK_CORE_B_DIV_MASK		= 0x1f << ATCLK_CORE_B_DIV_SHIFT,

	/* CLKSEL_CON14 */
	PCLK_PERIHP_DIV_CON_SHIFT	= 12,
	PCLK_PERIHP_DIV_CON_MASK	= 0x7 << PCLK_PERIHP_DIV_CON_SHIFT,
	HCLK_PERIHP_DIV_CON_SHIFT	= 8,
	HCLK_PERIHP_DIV_CON_MASK	= 3 << HCLK_PERIHP_DIV_CON_SHIFT,
	ACLK_PERIHP_PLL_SEL_SHIFT	= 7,
	ACLK_PERIHP_PLL_SEL_MASK	= 1 << ACLK_PERIHP_PLL_SEL_SHIFT,
	ACLK_PERIHP_PLL_SEL_CPLL	= 0,
	ACLK_PERIHP_PLL_SEL_GPLL	= 1,
	ACLK_PERIHP_DIV_CON_SHIFT	= 0,
	ACLK_PERIHP_DIV_CON_MASK	= 0x1f,

	/* CLKSEL_CON21 */
	ACLK_EMMC_PLL_SEL_SHIFT         = 7,
	ACLK_EMMC_PLL_SEL_MASK          = 0x1 << ACLK_EMMC_PLL_SEL_SHIFT,
	ACLK_EMMC_PLL_SEL_GPLL          = 0x1,
	ACLK_EMMC_DIV_CON_SHIFT         = 0,
	ACLK_EMMC_DIV_CON_MASK          = 0x1f,

	/* CLKSEL_CON22 */
	CLK_EMMC_PLL_SHIFT              = 8,
	CLK_EMMC_PLL_MASK               = 0x7 << CLK_EMMC_PLL_SHIFT,
	CLK_EMMC_PLL_SEL_CPLL           = 0x0,
	CLK_EMMC_PLL_SEL_GPLL           = 0x1,
	/* CLKSEL_CON15/16 (sdio / sdmmc) [10:8]: cpll gpll npll ppll usb480m 24m */
	CLK_EMMC_PLL_SEL_NPLL           = 0x2,
	CLK_EMMC_PLL_SEL_24M            = 0x5,
	CLK_EMMC_DIV_CON_SHIFT          = 0,
	CLK_EMMC_DIV_CON_MASK           = 0x7f << CLK_EMMC_DIV_CON_SHIFT,

	/* CLKSEL_CON23 */
	PCLK_PERILP0_DIV_CON_SHIFT	= 12,
	PCLK_PERILP0_DIV_CON_MASK	= 0x7 << PCLK_PERILP0_DIV_CON_SHIFT,
	HCLK_PERILP0_DIV_CON_SHIFT	= 8,
	HCLK_PERILP0_DIV_CON_MASK	= 3 << HCLK_PERILP0_DIV_CON_SHIFT,
	ACLK_PERILP0_PLL_SEL_SHIFT	= 7,
	ACLK_PERILP0_PLL_SEL_MASK	= 1 << ACLK_PERILP0_PLL_SEL_SHIFT,
	ACLK_PERILP0_PLL_SEL_CPLL	= 0,
	ACLK_PERILP0_PLL_SEL_GPLL	= 1,
	ACLK_PERILP0_DIV_CON_SHIFT	= 0,
	ACLK_PERILP0_DIV_CON_MASK	= 0x1f,

	/* CLKSEL_CON25 */
	PCLK_PERILP1_DIV_CON_SHIFT	= 8,
	PCLK_PERILP1_DIV_CON_MASK	= 0x7 << PCLK_PERILP1_DIV_CON_SHIFT,
	HCLK_PERILP1_PLL_SEL_SHIFT	= 7,
	HCLK_PERILP1_PLL_SEL_MASK	= 1 << HCLK_PERILP1_PLL_SEL_SHIFT,
	HCLK_PERILP1_PLL_SEL_CPLL	= 0,
	HCLK_PERILP1_PLL_SEL_GPLL	= 1,
	HCLK_PERILP1_DIV_CON_SHIFT	= 0,
	HCLK_PERILP1_DIV_CON_MASK	= 0x1f,

	/* CLKSEL_CON26 */
	CLK_SARADC_DIV_CON_SHIFT	= 8,
	CLK_SARADC_DIV_CON_MASK		= GENMASK(15, 8),
	CLK_SARADC_DIV_CON_WIDTH	= 8,

	/* CLKSEL_CON27 */
	CLK_TSADC_SEL_X24M		= 0x0,
	CLK_TSADC_SEL_SHIFT		= 15,
	CLK_TSADC_SEL_MASK		= 1 << CLK_TSADC_SEL_SHIFT,
	CLK_TSADC_DIV_CON_SHIFT		= 0,
	CLK_TSADC_DIV_CON_MASK		= 0x3ff,

	/* CLKSEL_CON47 & CLKSEL_CON48 */
	ACLK_VOP_PLL_SEL_SHIFT		= 6,
	ACLK_VOP_PLL_SEL_MASK		= 0x3 << ACLK_VOP_PLL_SEL_SHIFT,
	ACLK_VOP_PLL_SEL_CPLL		= 0x1,
	ACLK_VOP_DIV_CON_SHIFT		= 0,
	ACLK_VOP_DIV_CON_MASK		= 0x1f << ACLK_VOP_DIV_CON_SHIFT,

	/* CLKSEL_CON49 & CLKSEL_CON50 */
	DCLK_VOP_DCLK_SEL_SHIFT         = 11,
	DCLK_VOP_DCLK_SEL_MASK          = 1 << DCLK_VOP_DCLK_SEL_SHIFT,
	DCLK_VOP_DCLK_SEL_DIVOUT        = 0,
	DCLK_VOP_PLL_SEL_SHIFT          = 8,
	DCLK_VOP_PLL_SEL_MASK           = 3 << DCLK_VOP_PLL_SEL_SHIFT,
	DCLK_VOP_PLL_SEL_VPLL           = 0,
	DCLK_VOP_DIV_CON_MASK           = 0xff,
	DCLK_VOP_DIV_CON_SHIFT          = 0,

	/* CLKSEL_CON57 */
	PCLK_ALIVE_DIV_CON_SHIFT        = 0,
	PCLK_ALIVE_DIV_CON_MASK         = 0x1f << PCLK_ALIVE_DIV_CON_SHIFT,

	/* CLKSEL_CON58 */
	CLK_SPI_PLL_SEL_WIDTH = 1,
	CLK_SPI_PLL_SEL_MASK = ((1 < CLK_SPI_PLL_SEL_WIDTH) - 1),
	CLK_SPI_PLL_SEL_CPLL = 0,
	CLK_SPI_PLL_SEL_GPLL = 1,
	CLK_SPI_PLL_DIV_CON_WIDTH = 7,
	CLK_SPI_PLL_DIV_CON_MASK = ((1 << CLK_SPI_PLL_DIV_CON_WIDTH) - 1),

	CLK_SPI5_PLL_DIV_CON_SHIFT      = 8,
	CLK_SPI5_PLL_SEL_SHIFT	        = 15,

	/* CLKSEL_CON59 */
	CLK_SPI1_PLL_SEL_SHIFT		= 15,
	CLK_SPI1_PLL_DIV_CON_SHIFT	= 8,
	CLK_SPI0_PLL_SEL_SHIFT		= 7,
	CLK_SPI0_PLL_DIV_CON_SHIFT	= 0,

	/* CLKSEL_CON60 */
	CLK_SPI4_PLL_SEL_SHIFT		= 15,
	CLK_SPI4_PLL_DIV_CON_SHIFT	= 8,
	CLK_SPI2_PLL_SEL_SHIFT		= 7,
	CLK_SPI2_PLL_DIV_CON_SHIFT	= 0,

	/* CLKSEL_CON61 */
	CLK_I2C_PLL_SEL_MASK		= 1,
	CLK_I2C_PLL_SEL_CPLL		= 0,
	CLK_I2C_PLL_SEL_GPLL		= 1,
	CLK_I2C5_PLL_SEL_SHIFT		= 15,
	CLK_I2C5_DIV_CON_SHIFT		= 8,
	CLK_I2C1_PLL_SEL_SHIFT		= 7,
	CLK_I2C1_DIV_CON_SHIFT		= 0,

	/* CLKSEL_CON62 */
	CLK_I2C6_PLL_SEL_SHIFT		= 15,
	CLK_I2C6_DIV_CON_SHIFT		= 8,
	CLK_I2C2_PLL_SEL_SHIFT		= 7,
	CLK_I2C2_DIV_CON_SHIFT		= 0,

	/* CLKSEL_CON63 */
	CLK_I2C7_PLL_SEL_SHIFT		= 15,
	CLK_I2C7_DIV_CON_SHIFT		= 8,
	CLK_I2C3_PLL_SEL_SHIFT		= 7,
	CLK_I2C3_DIV_CON_SHIFT		= 0,

	/* CRU_SOFTRST_CON4 */
	RESETN_DDR0_REQ_SHIFT		= 8,
	RESETN_DDR0_REQ_MASK		= 1 << RESETN_DDR0_REQ_SHIFT,
	RESETN_DDRPHY0_REQ_SHIFT	= 9,
	RESETN_DDRPHY0_REQ_MASK		= 1 << RESETN_DDRPHY0_REQ_SHIFT,
	RESETN_DDR1_REQ_SHIFT		= 12,
	RESETN_DDR1_REQ_MASK		= 1 << RESETN_DDR1_REQ_SHIFT,
	RESETN_DDRPHY1_REQ_SHIFT	= 13,
	RESETN_DDRPHY1_REQ_MASK		= 1 << RESETN_DDRPHY1_REQ_SHIFT,
};

#define VCO_MAX_KHZ	(3200 * (MHz / KHz))
#define VCO_MIN_KHZ	(800 * (MHz / KHz))
#define OUTPUT_MAX_KHZ	(3200 * (MHz / KHz))
#define OUTPUT_MIN_KHZ	(16 * (MHz / KHz))

/*
 *  the div restructions of pll in integer mode, these are defined in
 *  * CRU_*PLL_CON0 or PMUCRU_*PLL_CON0
 */
#define PLL_DIV_MIN	16
#define PLL_DIV_MAX	3200

/*
 * How to calculate the PLL(from TRM V0.3 Part 1 Page 63):
 * Formulas also embedded within the Fractional PLL Verilog model:
 * If DSMPD = 1 (DSM is disabled, "integer mode")
 * FOUTVCO = FREF / REFDIV * FBDIV
 * FOUTPOSTDIV = FOUTVCO / POSTDIV1 / POSTDIV2
 * Where:
 * FOUTVCO = Fractional PLL non-divided output frequency
 * FOUTPOSTDIV = Fractional PLL divided output frequency
 *               (output of second post divider)
 * FREF = Fractional PLL input reference frequency, (the OSC_HZ 24MHz input)
 * REFDIV = Fractional PLL input reference clock divider
 * FBDIV = Integer value programmed into feedback divide
 *
 */
/*
 * Datasheet lock time is typical 250 / max 500 input clock cycles at 24 MHz.
 * 1000 us is ~24000 cycles -- generous, and only ever reached on a PLL that
 * is not going to lock at all.
 */
#define PLL_LOCK_TIMEOUT_US	1000

/*
 * Returns 0, or -ETIMEDOUT with the PLL left parked in SLOW mode.
 *
 * It returns a value rather than void because "left in slow mode" is not
 * uniformly survivable, and the caller is the only one who knows which case
 * it is in. For GPLL/CPLL/NPLL/VPLL a 24 MHz output is merely wrong-and-slow.
 * For DPLL it is not: DRAM whose timing and PHY delay lines were computed for
 * 800 MHz, clocked at 24 MHz, has no defined behaviour at all, and reporting
 * the requested rate as achieved sends the caller straight into
 * sdram_init()/data_training() with every parameter mismatched.
 */
static int rkclk_set_pll(u32 *pll_con, const struct pll_div *div)
{
	/* All 8 PLLs have same VCO and output frequency range restrictions. */
	u32 vco_khz = OSC_HZ / 1000 * div->fbdiv / div->refdiv;
	u32 output_khz = vco_khz / div->postdiv1 / div->postdiv2;
	uint i;

	debug("PLL at %p: fbdiv=%d, refdiv=%d, postdiv1=%d, "
			   "postdiv2=%d, vco=%u khz, output=%u khz\n",
			   pll_con, div->fbdiv, div->refdiv, div->postdiv1,
			   div->postdiv2, vco_khz, output_khz);
	assert(vco_khz >= VCO_MIN_KHZ && vco_khz <= VCO_MAX_KHZ &&
	       output_khz >= OUTPUT_MIN_KHZ && output_khz <= OUTPUT_MAX_KHZ &&
	       div->fbdiv >= PLL_DIV_MIN && div->fbdiv <= PLL_DIV_MAX);

	/*
	 * When power on or changing PLL setting,
	 * we must force PLL into slow mode to ensure output stable clock.
	 */
	rk_clrsetreg(&pll_con[3], PLL_MODE_MASK,
		     PLL_MODE_SLOW << PLL_MODE_SHIFT);

	/* use integer mode */
	rk_clrsetreg(&pll_con[3], PLL_DSMPD_MASK,
		     PLL_INTEGER_MODE << PLL_DSMPD_SHIFT);

	rk_clrsetreg(&pll_con[0], PLL_FBDIV_MASK,
		     div->fbdiv << PLL_FBDIV_SHIFT);
	/*
	 * The clear mask used to carry `| PLL_REFDIV_SHIFT`, which is 0 --
	 * a no-op that read as a fourth field. Dropping it is bit-identical.
	 */
	rk_clrsetreg(&pll_con[1],
		     PLL_POSTDIV2_MASK | PLL_POSTDIV1_MASK |
		     PLL_REFDIV_MASK,
		     (div->postdiv2 << PLL_POSTDIV2_SHIFT) |
		     (div->postdiv1 << PLL_POSTDIV1_SHIFT) |
		     (div->refdiv << PLL_REFDIV_SHIFT));

	/*
	 * Wait for lock, BOUNDED. Datasheet: typical 250, max 500 input clock
	 * cycles at 24 MHz, so 1000 us is ~24000 cycles -- the same very safe
	 * bound Linux uses in rockchip_rk3399_pll_wait_lock(). Unbounded, a
	 * PLL that never locks hung the boot inside udelay() with nothing on
	 * the console and no way to tell it from a hang anywhere else.
	 *
	 * A counted udelay loop rather than readl_poll_timeout(): the latter
	 * is built on timer_get_us(), and this runs from rkclk_init() in early
	 * SPL where that is a dependency worth not taking.
	 */
	for (i = 0; i < PLL_LOCK_TIMEOUT_US; i++) {
		if (readl(&pll_con[2]) & PLL_LOCK_STATUS_MASK)
			break;
		udelay(1);
	}

	if (!(readl(&pll_con[2]) & PLL_LOCK_STATUS_MASK)) {
		/*
		 * Leave the mode mux in SLOW. Going to NORM here would put
		 * every consumer on an unlocked VCO. Staying on the 24 MHz
		 * oscillator is not "defined" in any useful sense -- every
		 * divider downstream was computed for the target rate, so the
		 * board runs at roughly 1/25 speed with peripheral clocks no
		 * driver's divider math expects -- but it is at least
		 * observable, which an unlocked VCO is not, and it gets a
		 * message onto the console instead of hanging in udelay().
		 * The caller decides whether that is survivable.
		 */
		printf("PLL at %p: no lock in %d us, left in slow mode (24 MHz)\n",
		       pll_con, PLL_LOCK_TIMEOUT_US);
		return -ETIMEDOUT;
	}

	/* pll enter normal mode */
	rk_clrsetreg(&pll_con[3], PLL_MODE_MASK,
		     PLL_MODE_NORM << PLL_MODE_SHIFT);

	return 0;
}

static int pll_para_config(u32 freq_hz, struct pll_div *div)
{
	u32 ref_khz = OSC_HZ / KHz, refdiv, fbdiv = 0;
	u32 postdiv1, postdiv2 = 1;
	u32 fref_khz;
	u32 diff_khz, best_diff_khz;
	const u32 max_refdiv = 63, max_fbdiv = 3200, min_fbdiv = 16;
	const u32 max_postdiv1 = 7, max_postdiv2 = 7;
	u32 vco_khz;
	u32 freq_khz = freq_hz / KHz;

	if (!freq_hz) {
		printf("%s: the frequency can't be 0 Hz\n", __func__);
		return -1;
	}

	postdiv1 = DIV_ROUND_UP(VCO_MIN_KHZ, freq_khz);
	if (postdiv1 > max_postdiv1) {
		postdiv2 = DIV_ROUND_UP(postdiv1, max_postdiv1);
		postdiv1 = DIV_ROUND_UP(postdiv1, postdiv2);
	}

	vco_khz = freq_khz * postdiv1 * postdiv2;

	if (vco_khz < VCO_MIN_KHZ || vco_khz > VCO_MAX_KHZ ||
	    postdiv2 > max_postdiv2) {
		printf("%s: Cannot find out a supported VCO"
		       " for Frequency (%uHz).\n", __func__, freq_hz);
		return -1;
	}

	div->postdiv1 = postdiv1;
	div->postdiv2 = postdiv2;

	best_diff_khz = vco_khz;
	for (refdiv = 1; refdiv < max_refdiv && best_diff_khz; refdiv++) {
		fref_khz = ref_khz / refdiv;

		fbdiv = vco_khz / fref_khz;
		if (fbdiv >= max_fbdiv || fbdiv <= min_fbdiv)
			continue;
		diff_khz = vco_khz - fbdiv * fref_khz;
		if (fbdiv + 1 < max_fbdiv && diff_khz > fref_khz / 2) {
			fbdiv++;
			diff_khz = fref_khz - diff_khz;
		}

		if (diff_khz >= best_diff_khz)
			continue;

		best_diff_khz = diff_khz;
		div->refdiv = refdiv;
		div->fbdiv = fbdiv;
	}

	if (best_diff_khz > 4 * (MHz / KHz)) {
		printf("%s: Failed to match output frequency %u, "
		       "difference is %u Hz,exceed 4MHZ\n", __func__, freq_hz,
		       best_diff_khz * KHz);
		return -1;
	}
	return 0;
}

void rk3399_configure_cpu_l(struct rockchip_cru *cru,
			    enum apll_l_frequencies apll_l_freq)
{
	u32 aclkm_div;
	u32 pclk_dbg_div;
	u32 atclk_div;

	/*
	 * The OPP rates take Linux's dividers (clk-rk3399.c rk3399_cpuclkl_rates
	 * for 1008/1200/1416 MHz: aclkm 1, pclk_dbg 5, atclk 5), set BEFORE the
	 * PLL rises so no bus is ever above its rate; the legacy entries keep
	 * the dividers computed for LPLL_HZ.
	 */
	if (apll_l_freq == APLL_L_1008_MHZ || apll_l_freq == APLL_L_1200_MHZ ||
	    apll_l_freq == APLL_L_1416_MHZ) {
		rk_clrsetreg(&cru->clksel_con[0],
			     ACLKM_CORE_L_DIV_CON_MASK | CLK_CORE_L_DIV_MASK,
			     1 << ACLKM_CORE_L_DIV_CON_SHIFT |
			     0 << CLK_CORE_L_DIV_SHIFT);
		rk_clrsetreg(&cru->clksel_con[1],
			     PCLK_DBG_L_DIV_MASK | ATCLK_CORE_L_DIV_MASK,
			     5 << PCLK_DBG_L_DIV_SHIFT |
			     5 << ATCLK_CORE_L_DIV_SHIFT);
		rkclk_set_pll(&cru->apll_l_con[0], apll_l_cfgs[apll_l_freq]);
		rk_clrsetreg(&cru->clksel_con[0], CLK_CORE_L_PLL_SEL_MASK,
			     CLK_CORE_L_PLL_SEL_ALPLL << CLK_CORE_L_PLL_SEL_SHIFT);
		return;
	}

	/* Setup cluster L */
	rkclk_set_pll(&cru->apll_l_con[0], apll_l_cfgs[apll_l_freq]);

	aclkm_div = LPLL_HZ / ACLKM_CORE_L_HZ - 1;
	assert((aclkm_div + 1) * ACLKM_CORE_L_HZ == LPLL_HZ &&
	       aclkm_div < 0x1f);

	pclk_dbg_div = LPLL_HZ / PCLK_DBG_L_HZ - 1;
	assert((pclk_dbg_div + 1) * PCLK_DBG_L_HZ == LPLL_HZ &&
	       pclk_dbg_div < 0x1f);

	atclk_div = LPLL_HZ / ATCLK_CORE_L_HZ - 1;
	assert((atclk_div + 1) * ATCLK_CORE_L_HZ == LPLL_HZ &&
	       atclk_div < 0x1f);

	rk_clrsetreg(&cru->clksel_con[0],
		     ACLKM_CORE_L_DIV_CON_MASK | CLK_CORE_L_PLL_SEL_MASK |
		     CLK_CORE_L_DIV_MASK,
		     aclkm_div << ACLKM_CORE_L_DIV_CON_SHIFT |
		     CLK_CORE_L_PLL_SEL_ALPLL << CLK_CORE_L_PLL_SEL_SHIFT |
		     0 << CLK_CORE_L_DIV_SHIFT);

	rk_clrsetreg(&cru->clksel_con[1],
		     PCLK_DBG_L_DIV_MASK | ATCLK_CORE_L_DIV_MASK,
		     pclk_dbg_div << PCLK_DBG_L_DIV_SHIFT |
		     atclk_div << ATCLK_CORE_L_DIV_SHIFT);
}

void rk3399_configure_cpu_b(struct rockchip_cru *cru,
			    enum apll_b_frequencies apll_b_freq)
{
	u32 aclkm_div;
	u32 pclk_dbg_div;
	u32 atclk_div;

	/* Setup cluster B */
	rkclk_set_pll(&cru->apll_b_con[0], apll_b_cfgs[apll_b_freq]);

	aclkm_div = BPLL_HZ / ACLKM_CORE_B_HZ - 1;
	assert((aclkm_div + 1) * ACLKM_CORE_B_HZ == BPLL_HZ &&
	       aclkm_div < 0x1f);

	pclk_dbg_div = BPLL_HZ / PCLK_DBG_B_HZ - 1;
	assert((pclk_dbg_div + 1) * PCLK_DBG_B_HZ == BPLL_HZ &&
	       pclk_dbg_div < 0x1f);

	atclk_div = BPLL_HZ / ATCLK_CORE_B_HZ - 1;
	assert((atclk_div + 1) * ATCLK_CORE_B_HZ == BPLL_HZ &&
	       atclk_div < 0x1f);

	rk_clrsetreg(&cru->clksel_con[2],
		     ACLKM_CORE_B_DIV_CON_MASK | CLK_CORE_B_PLL_SEL_MASK |
		     CLK_CORE_B_DIV_MASK,
		     aclkm_div << ACLKM_CORE_B_DIV_CON_SHIFT |
		     CLK_CORE_B_PLL_SEL_ABPLL << CLK_CORE_B_PLL_SEL_SHIFT |
		     0 << CLK_CORE_B_DIV_SHIFT);

	rk_clrsetreg(&cru->clksel_con[3],
		     PCLK_DBG_B_DIV_MASK | ATCLK_CORE_B_DIV_MASK,
		     pclk_dbg_div << PCLK_DBG_B_DIV_SHIFT |
		     atclk_div << ATCLK_CORE_B_DIV_SHIFT);
}

#define I2C_CLK_REG_MASK(bus) \
	(I2C_DIV_CON_MASK << CLK_I2C ##bus## _DIV_CON_SHIFT | \
	 CLK_I2C_PLL_SEL_MASK << CLK_I2C ##bus## _PLL_SEL_SHIFT)

#define I2C_CLK_REG_VALUE(bus, clk_div) \
	((clk_div - 1) << CLK_I2C ##bus## _DIV_CON_SHIFT | \
	 CLK_I2C_PLL_SEL_GPLL << CLK_I2C ##bus## _PLL_SEL_SHIFT)

#define I2C_CLK_DIV_VALUE(con, bus) \
	((con >> CLK_I2C ##bus## _DIV_CON_SHIFT) & I2C_DIV_CON_MASK)

#define I2C_PMUCLK_REG_MASK(bus) \
	(I2C_DIV_CON_MASK << CLK_I2C ##bus## _DIV_CON_SHIFT)

#define I2C_PMUCLK_REG_VALUE(bus, clk_div) \
	((clk_div - 1) << CLK_I2C ##bus## _DIV_CON_SHIFT)

static ulong rk3399_i2c_get_clk(struct rockchip_cru *cru, ulong clk_id)
{
	u32 div, con;

	switch (clk_id) {
	case SCLK_I2C1:
		con = readl(&cru->clksel_con[61]);
		div = I2C_CLK_DIV_VALUE(con, 1);
		break;
	case SCLK_I2C2:
		con = readl(&cru->clksel_con[62]);
		div = I2C_CLK_DIV_VALUE(con, 2);
		break;
	case SCLK_I2C3:
		con = readl(&cru->clksel_con[63]);
		div = I2C_CLK_DIV_VALUE(con, 3);
		break;
	case SCLK_I2C5:
		con = readl(&cru->clksel_con[61]);
		div = I2C_CLK_DIV_VALUE(con, 5);
		break;
	case SCLK_I2C6:
		con = readl(&cru->clksel_con[62]);
		div = I2C_CLK_DIV_VALUE(con, 6);
		break;
	case SCLK_I2C7:
		con = readl(&cru->clksel_con[63]);
		div = I2C_CLK_DIV_VALUE(con, 7);
		break;
	default:
		printf("do not support this i2c bus\n");
		return -EINVAL;
	}

	return DIV_TO_RATE(GPLL_HZ, div);
}

static ulong rk3399_i2c_set_clk(struct rockchip_cru *cru, ulong clk_id, uint hz)
{
	int src_clk_div;

	/* i2c0,4,8 src clock from ppll, i2c1,2,3,5,6,7 src clock from gpll*/
	src_clk_div = GPLL_HZ / hz;
	assert(src_clk_div - 1 < 127);

	switch (clk_id) {
	case SCLK_I2C1:
		rk_clrsetreg(&cru->clksel_con[61], I2C_CLK_REG_MASK(1),
			     I2C_CLK_REG_VALUE(1, src_clk_div));
		break;
	case SCLK_I2C2:
		rk_clrsetreg(&cru->clksel_con[62], I2C_CLK_REG_MASK(2),
			     I2C_CLK_REG_VALUE(2, src_clk_div));
		break;
	case SCLK_I2C3:
		rk_clrsetreg(&cru->clksel_con[63], I2C_CLK_REG_MASK(3),
			     I2C_CLK_REG_VALUE(3, src_clk_div));
		break;
	case SCLK_I2C5:
		rk_clrsetreg(&cru->clksel_con[61], I2C_CLK_REG_MASK(5),
			     I2C_CLK_REG_VALUE(5, src_clk_div));
		break;
	case SCLK_I2C6:
		rk_clrsetreg(&cru->clksel_con[62], I2C_CLK_REG_MASK(6),
			     I2C_CLK_REG_VALUE(6, src_clk_div));
		break;
	case SCLK_I2C7:
		rk_clrsetreg(&cru->clksel_con[63], I2C_CLK_REG_MASK(7),
			     I2C_CLK_REG_VALUE(7, src_clk_div));
		break;
	default:
		printf("do not support this i2c bus\n");
		return -EINVAL;
	}

	return rk3399_i2c_get_clk(cru, clk_id);
}

/*
 * RK3399 SPI clocks have a common divider-width (7 bits) and a single bit
 * to select either CPLL or GPLL as the clock-parent. The location within
 * the enclosing CLKSEL_CON (i.e. div_shift and sel_shift) are variable.
 */

struct spi_clkreg {
	u8 reg;  /* CLKSEL_CON[reg] register in CRU */
	u8 div_shift;
	u8 sel_shift;
};

/*
 * The entries are numbered relative to their offset from SCLK_SPI0.
 *
 * Note that SCLK_SPI3 (which is configured via PMUCRU and requires different
 * logic is not supported).
 */
static const struct spi_clkreg spi_clkregs[] = {
	[0] = { .reg = 59,
		.div_shift = CLK_SPI0_PLL_DIV_CON_SHIFT,
		.sel_shift = CLK_SPI0_PLL_SEL_SHIFT, },
	[1] = { .reg = 59,
		.div_shift = CLK_SPI1_PLL_DIV_CON_SHIFT,
		.sel_shift = CLK_SPI1_PLL_SEL_SHIFT, },
	[2] = { .reg = 60,
		.div_shift = CLK_SPI2_PLL_DIV_CON_SHIFT,
		.sel_shift = CLK_SPI2_PLL_SEL_SHIFT, },
	[3] = { .reg = 60,
		.div_shift = CLK_SPI4_PLL_DIV_CON_SHIFT,
		.sel_shift = CLK_SPI4_PLL_SEL_SHIFT, },
	[4] = { .reg = 58,
		.div_shift = CLK_SPI5_PLL_DIV_CON_SHIFT,
		.sel_shift = CLK_SPI5_PLL_SEL_SHIFT, },
};

static ulong rk3399_spi_get_clk(struct rockchip_cru *cru, ulong clk_id)
{
	const struct spi_clkreg *spiclk = NULL;
	u32 div, val;

	switch (clk_id) {
	case SCLK_SPI0 ... SCLK_SPI5:
		spiclk = &spi_clkregs[clk_id - SCLK_SPI0];
		break;

	default:
		pr_err("%s: SPI clk-id %ld not supported\n", __func__, clk_id);
		return -EINVAL;
	}

	val = readl(&cru->clksel_con[spiclk->reg]);
	div = bitfield_extract(val, spiclk->div_shift,
			       CLK_SPI_PLL_DIV_CON_WIDTH);

	return DIV_TO_RATE(GPLL_HZ, div);
}

static ulong rk3399_spi_set_clk(struct rockchip_cru *cru, ulong clk_id, uint hz)
{
	const struct spi_clkreg *spiclk = NULL;
	int src_clk_div;

	src_clk_div = DIV_ROUND_UP(GPLL_HZ, hz) - 1;
	assert(src_clk_div < 128);

	switch (clk_id) {
	case SCLK_SPI1 ... SCLK_SPI5:
		spiclk = &spi_clkregs[clk_id - SCLK_SPI0];
		break;

	default:
		pr_err("%s: SPI clk-id %ld not supported\n", __func__, clk_id);
		return -EINVAL;
	}

	rk_clrsetreg(&cru->clksel_con[spiclk->reg],
		     ((CLK_SPI_PLL_DIV_CON_MASK << spiclk->div_shift) |
		       (CLK_SPI_PLL_SEL_GPLL << spiclk->sel_shift)),
		     ((src_clk_div << spiclk->div_shift) |
		      (CLK_SPI_PLL_SEL_GPLL << spiclk->sel_shift)));

	return rk3399_spi_get_clk(cru, clk_id);
}

static ulong rk3399_pll_get_rate(u32 *pll_con);

/**
 * rkclk_pll_rate_of() - what these dividers would produce, in Hz
 *
 * pll_para_config() picks the closest reachable dividers rather than the
 * requested rate, so "is this PLL already where I want it" can only be asked
 * against what the dividers actually yield.
 */
static ulong rkclk_pll_rate_of(const struct pll_div *div)
{
	/*
	 * Same guard as rk3399_pll_get_rate(), whose result this must match
	 * exactly. Both current callers pass either a PLL_DIVISORS() constant
	 * or a struct that has already survived pll_para_config(), so it
	 * cannot fire today -- but the pair is documented as having to agree,
	 * and a zeroed struct reaching only this half is a data abort.
	 */
	if (!div->refdiv || !div->postdiv1 || !div->postdiv2)
		return 0;

	return (ulong)((u64)OSC_HZ * div->fbdiv / div->refdiv /
		       div->postdiv1 / div->postdiv2);
}

static ulong rk3399_vop_set_clk(struct rockchip_cru *cru, ulong clk_id, u32 hz)
{
	struct pll_div vpll_config = {0};
	int aclk_vop = 198 * MHz;
	void *aclkreg_addr, *dclkreg_addr;
	u32 div;

	switch (clk_id) {
	case DCLK_VOP0:
		aclkreg_addr = &cru->clksel_con[47];
		dclkreg_addr = &cru->clksel_con[49];
		break;
	case DCLK_VOP1:
		aclkreg_addr = &cru->clksel_con[48];
		dclkreg_addr = &cru->clksel_con[50];
		break;
	default:
		return -EINVAL;
	}
	/*
	 * Move aclk_vop to CPLL BEFORE touching VPLL below. aclk_vop's mux
	 * resets to VPLL (sel 0), and rkclk_set_pll() parks a PLL on the
	 * 24 MHz oscillator for the whole reprogram-and-relock window. These
	 * two blocks must not be reordered: the second would otherwise glitch
	 * the VOP's own bus clock while the VOP is being brought up.
	 */
	/* vop aclk source clk: cpll */
	div = CPLL_HZ / aclk_vop;
	assert(div - 1 < 32);

	rk_clrsetreg(aclkreg_addr,
		     ACLK_VOP_PLL_SEL_MASK | ACLK_VOP_DIV_CON_MASK,
		     ACLK_VOP_PLL_SEL_CPLL << ACLK_VOP_PLL_SEL_SHIFT |
		     (div - 1) << ACLK_VOP_DIV_CON_SHIFT);

	/* vop dclk source from vpll, and equals to vpll(means div == 1) */
	if (pll_para_config(hz, &vpll_config))
		return -1;

	/*
	 * Only reprogram VPLL if it is not already there. rkclk_set_pll()
	 * forces the mode mux to SLOW, rewrites the dividers, waits for lock,
	 * and only then returns to NORM -- so every consumer sees the 24 MHz
	 * oscillator for that whole window and then a step back up. On a
	 * pixel clock with a display already scanning out, that is a dropped
	 * link, not a no-op. Free on the first call, which is the common one:
	 * VPLL comes out of reset in slow mode and every target differs.
	 *
	 * Compared against the achieved rate rather than the mode bits, for
	 * the same reason rk3399_dp_set_clk() does: NORM says the PLL is
	 * running, not that it is running where these dividers assume.
	 */
	if (rk3399_pll_get_rate(&cru->vpll_con[0]) !=
	    rkclk_pll_rate_of(&vpll_config))
		rkclk_set_pll(&cru->vpll_con[0], &vpll_config);

	rk_clrsetreg(dclkreg_addr,
		     DCLK_VOP_DCLK_SEL_MASK | DCLK_VOP_PLL_SEL_MASK |
		     DCLK_VOP_DIV_CON_MASK,
		     DCLK_VOP_DCLK_SEL_DIVOUT << DCLK_VOP_DCLK_SEL_SHIFT |
		     DCLK_VOP_PLL_SEL_VPLL << DCLK_VOP_PLL_SEL_SHIFT |
		     (1 - 1) << DCLK_VOP_DIV_CON_SHIFT);

	return hz;
}

/*
 * Bring NPLL to NPLL_HZ if it is not there already. rkclk_init() programs
 * GPLL and CPLL only, and does not run at all in U-Boot proper when SPL
 * handed off, so NPLL is brought up here, by whichever consumer first needs
 * it (the DP core clock, the SD / SDIO card clock). Left alone it sits in
 * slow mode on the 24 MHz oscillator, which would give a consumer 24 / n
 * while it was told NPLL_HZ / n -- a silent wrong-frequency, not a failure.
 *
 * Checked against the RATE, not the mode bits: normal mode only says the
 * PLL is running, not that it runs at the rate the divider assumes; TPL,
 * SPL or a prior boot could have left it anywhere. rkclk_set_pll() parks
 * the PLL on the oscillator for the reprogram-and-relock window, so a
 * consumer already on NPLL at a different rate would glitch -- which is
 * why every NPLL consumer comes through this one function and the rate is
 * one constant.
 */
static void rk3399_npll_ensure(struct rockchip_cru *cru)
{
	if (rk3399_pll_get_rate(&cru->npll_con[0]) !=
	    rkclk_pll_rate_of(&npll_init_cfg))
		rkclk_set_pll(&cru->npll_con[0], &npll_init_cfg);
}

static ulong rk3399_mmc_get_clk(struct rockchip_cru *cru, uint clk_id)
{
	u32 div, con;

	switch (clk_id) {
	case HCLK_SDIO:
	case SCLK_SDIO:
		con = readl(&cru->clksel_con[15]);
		/* dwmmc controller have internal div 2 */
		div = 2;
		break;
	case HCLK_SDMMC:
	case SCLK_SDMMC:
		con = readl(&cru->clksel_con[16]);
		/* dwmmc controller have internal div 2 */
		div = 2;
		break;
	case SCLK_EMMC:
		con = readl(&cru->clksel_con[22]);
		div = 1;
		break;
	default:
		return -EINVAL;
	}

	/*
	 * The CRU field is (divider - 1); the dw-mshc's own halving is the
	 * "internal div 2" above, so the card clock is src / (field + 1) / div.
	 * (The previous `div *= field` read 594/3 = 198 MHz for a CRU divider
	 * of 2, whose real card clock is 594/2/2 = 148.5 MHz.)
	 */
	div *= ((con & CLK_EMMC_DIV_CON_MASK) >> CLK_EMMC_DIV_CON_SHIFT) + 1;
	/*
	 * The PLL rates are read from the CRU, not assumed: a PLL that another
	 * stage left elsewhere, or NPLL before rk3399_npll_ensure() has run,
	 * would otherwise be reported at the rate this driver wants rather
	 * than the one the card is clocked at.
	 */
	switch ((con & CLK_EMMC_PLL_MASK) >> CLK_EMMC_PLL_SHIFT) {
	case CLK_EMMC_PLL_SEL_24M:
		return OSC_HZ / div;
	case CLK_EMMC_PLL_SEL_CPLL:
		return rk3399_pll_get_rate(&cru->cpll_con[0]) / div;
	case CLK_EMMC_PLL_SEL_NPLL:
		return rk3399_pll_get_rate(&cru->npll_con[0]) / div;
	default:
		return rk3399_pll_get_rate(&cru->gpll_con[0]) / div;
	}
}

/*
 * clk_sdmmc / clk_sdio: the CRU divides a PLL and the dw-mshc halves that
 * again (its CLKDIV stays 0), so the card clock is PLL / 2 / n. The parents
 * on offer are the two PLLs rkclk_init() programs -- GPLL 594 (297 / 148.5
 * / 99 / 74.25 ...) and CPLL 384 (192 / 96 / 64 ...) -- and NPLL, brought up
 * on demand at the 1200 MHz that divides to every SD band exactly (200 / 150
 * / 100 / 50 = 1200 / 2 / 3, 4, 6, 12). Pick the source and divider that
 * give the largest card clock not above the request, the way Linux's CCF
 * resolves the same composite over the same parents: a request of 200 lands
 * on 200 (it used to land on 192, the CPLL step), 150 on 150 (was 148.5),
 * 100 on 100 (was 99), 50 on 50 (was 49.5). On a tie the parent listed
 * first wins, so a request no better served by NPLL never brings it up.
 */
static void rk3399_dwmmc_set_clk(struct rockchip_cru *cru,
				 unsigned int con, ulong set_rate)
{
	static const struct {
		ulong hz;
		u8 sel;
	} parents[] = {
		{ GPLL_HZ, CLK_EMMC_PLL_SEL_GPLL },
		{ CPLL_HZ, CLK_EMMC_PLL_SEL_CPLL },
		{ NPLL_HZ, CLK_EMMC_PLL_SEL_NPLL },
	};
	/* mmc clock defaulg div 2 internal, provide double in cru */
	int src_clk_div = DIV_ROUND_UP(GPLL_HZ / 2, set_rate);
	int src = CLK_EMMC_PLL_SEL_GPLL;
	ulong best = 0;
	int i;

	if (src_clk_div > 128) {
		/* use 24MHz source for 400KHz clock */
		src_clk_div = DIV_ROUND_UP(OSC_HZ / 2, set_rate);
		assert(src_clk_div - 1 < 128);
		rk_clrsetreg(&cru->clksel_con[con],
			     CLK_EMMC_PLL_MASK | CLK_EMMC_DIV_CON_MASK,
			     CLK_EMMC_PLL_SEL_24M << CLK_EMMC_PLL_SHIFT |
			     (src_clk_div - 1) << CLK_EMMC_DIV_CON_SHIFT);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(parents); i++) {
		int div = DIV_ROUND_UP(parents[i].hz / 2, set_rate);
		ulong card = parents[i].hz / 2 / div;

		if (div > 128 || card <= best)
			continue;
		best = card;
		src_clk_div = div;
		src = parents[i].sel;
	}

	if (src == CLK_EMMC_PLL_SEL_NPLL)
		rk3399_npll_ensure(cru);

	rk_clrsetreg(&cru->clksel_con[con],
		     CLK_EMMC_PLL_MASK | CLK_EMMC_DIV_CON_MASK,
		     src << CLK_EMMC_PLL_SHIFT |
		     (src_clk_div - 1) << CLK_EMMC_DIV_CON_SHIFT);
}

static ulong rk3399_mmc_set_clk(struct rockchip_cru *cru,
				ulong clk_id, ulong set_rate)
{
	switch (clk_id) {
	case HCLK_SDIO:
	case SCLK_SDIO:
		rk3399_dwmmc_set_clk(cru, 15, set_rate);
		break;
	case HCLK_SDMMC:
	case SCLK_SDMMC:
		rk3399_dwmmc_set_clk(cru, 16, set_rate);
		break;
	case SCLK_EMMC: {
		int aclk_emmc = 198 * MHz;
		/* Select aclk_emmc source from GPLL */
		int src_clk_div = DIV_ROUND_UP(GPLL_HZ, aclk_emmc);

		assert(src_clk_div - 1 < 32);

		rk_clrsetreg(&cru->clksel_con[21],
			     ACLK_EMMC_PLL_SEL_MASK | ACLK_EMMC_DIV_CON_MASK,
			     ACLK_EMMC_PLL_SEL_GPLL << ACLK_EMMC_PLL_SEL_SHIFT |
			     (src_clk_div - 1) << ACLK_EMMC_DIV_CON_SHIFT);

		/* Select clk_emmc source from GPLL too */
		src_clk_div = DIV_ROUND_UP(GPLL_HZ, set_rate);
		assert(src_clk_div - 1 < 128);

		rk_clrsetreg(&cru->clksel_con[22],
			     CLK_EMMC_PLL_MASK | CLK_EMMC_DIV_CON_MASK,
			     CLK_EMMC_PLL_SEL_GPLL << CLK_EMMC_PLL_SHIFT |
			     (src_clk_div - 1) << CLK_EMMC_DIV_CON_SHIFT);
		break;
	}
	default:
		return -EINVAL;
	}
	return rk3399_mmc_get_clk(cru, clk_id);
}

static ulong rk3399_gmac_set_clk(struct rockchip_cru *cru, ulong rate)
{
	ulong ret;

	/*
	 * The RGMII CLK can be derived either from an external "clkin"
	 * or can be generated from internally by a divider from SCLK_MAC.
	 */
	if (readl(&cru->clksel_con[19]) & BIT(4)) {
		/* An external clock will always generate the right rate... */
		ret = rate;
	} else {
		/*
		 * No platform uses an internal clock to date.
		 * Implement this once it becomes necessary and print an error
		 * if someone tries to use it (while it remains unimplemented).
		 */
		pr_err("%s: internal clock is UNIMPLEMENTED\n", __func__);
		ret = 0;
	}

	return ret;
}

#define PMUSGRF_DDR_RGN_CON16 0xff330040
static ulong rk3399_ddr_set_clk(struct rockchip_cru *cru,
				ulong set_rate)
{
	struct pll_div dpll_cfg;
	int ret;

	/*  IC ECO bug, need to set this register */
	writel(0xc000c000, PMUSGRF_DDR_RGN_CON16);

	/*  clk_ddrc == DPLL = 24MHz / refdiv * fbdiv / postdiv1 / postdiv2 */
	switch (set_rate) {
	case 50 * MHz:
		dpll_cfg = (struct pll_div)
		{.refdiv = 2, .fbdiv = 75, .postdiv1 = 3, .postdiv2 = 6};
		break;
	case 200 * MHz:
		dpll_cfg = (struct pll_div)
		{.refdiv = 1, .fbdiv = 50, .postdiv1 = 6, .postdiv2 = 1};
		break;
	case 300 * MHz:
		dpll_cfg = (struct pll_div)
		{.refdiv = 2, .fbdiv = 100, .postdiv1 = 4, .postdiv2 = 1};
		break;
	case 400 * MHz:
		dpll_cfg = (struct pll_div)
		{.refdiv = 1, .fbdiv = 50, .postdiv1 = 3, .postdiv2 = 1};
		break;
	case 666 * MHz:
		dpll_cfg = (struct pll_div)
		{.refdiv = 2, .fbdiv = 111, .postdiv1 = 2, .postdiv2 = 1};
		break;
	case 800 * MHz:
		dpll_cfg = (struct pll_div)
		{.refdiv = 1, .fbdiv = 100, .postdiv1 = 3, .postdiv2 = 1};
		break;
	case 933 * MHz:
		dpll_cfg = (struct pll_div)
		{.refdiv = 1, .fbdiv = 116, .postdiv1 = 3, .postdiv2 = 1};
		break;
	default:
		/*
		 * Return, do not fall through. dpll_cfg is an uninitialised
		 * stack local, so continuing here programmed the DDR PLL from
		 * whatever happened to be on the stack -- and then reported
		 * the requested rate as if it had been set. The assert() in
		 * rkclk_set_pll() is the only thing that stood in front of it,
		 * and asserts are compiled out unless debug is on.
		 */
		pr_err("Unsupported SDRAM frequency!,%ld\n", set_rate);
		return -EINVAL;
	}
	/*
	 * Propagated, unlike the other seven call sites. A DPLL that does not
	 * lock leaves the DRAM on the 24 MHz oscillator; returning set_rate
	 * here would report the requested frequency as achieved and the
	 * caller would train the PHY for it.
	 */
	ret = rkclk_set_pll(&cru->dpll_con[0], &dpll_cfg);
	if (ret)
		return ret;

	return set_rate;
}

static ulong rk3399_alive_get_clk(struct rockchip_cru *cru)
{
        u32 div, val;

        val = readl(&cru->clksel_con[57]);
        div = (val & PCLK_ALIVE_DIV_CON_MASK) >>
	       PCLK_ALIVE_DIV_CON_SHIFT;

        return DIV_TO_RATE(GPLL_HZ, div);
}

static ulong rk3399_saradc_get_clk(struct rockchip_cru *cru)
{
	u32 div, val;

	val = readl(&cru->clksel_con[26]);
	div = bitfield_extract(val, CLK_SARADC_DIV_CON_SHIFT,
			       CLK_SARADC_DIV_CON_WIDTH);

	return DIV_TO_RATE(OSC_HZ, div);
}

static ulong rk3399_saradc_set_clk(struct rockchip_cru *cru, uint hz)
{
	int src_clk_div;

	src_clk_div = DIV_ROUND_UP(OSC_HZ, hz) - 1;
	assert(src_clk_div < 128);

	rk_clrsetreg(&cru->clksel_con[26],
		     CLK_SARADC_DIV_CON_MASK,
		     src_clk_div << CLK_SARADC_DIV_CON_SHIFT);

	return rk3399_saradc_get_clk(cru);
}

static ulong rk3399_pciephy_get_clk(struct rockchip_cru *cru)
{
	if (readl(&cru->clksel_con[18]) & BIT(10))
		return 100 * MHz;
	else
		return OSC_HZ;
}

static ulong rk3399_pciephy_set_clk(struct rockchip_cru *cru, uint hz)
{
	if (hz == 100 * MHz)
		rk_setreg(&cru->clksel_con[18], BIT(10));
	else if (hz == OSC_HZ)
		rk_clrreg(&cru->clksel_con[18], BIT(10));
	else
		return -EINVAL;

	return rk3399_pciephy_get_clk(cru);
}

/**
 * rk3399_pll_get_rate() - read a PLL's actual output rate from its registers
 *
 * Derived rather than assumed, because the DisplayPort core clock's value is
 * written into the DP controller's SW_CLK_H register and the firmware scales
 * its AUX bit timing and link-training timers by it. A wrong value is not
 * rejected there -- only zero is checked -- so it produces AUX deferrals and
 * flaky training rather than an error. A rate taken from a #define would be a
 * claim about what we programmed; this is a reading of what the PLL is doing.
 */
static ulong rk3399_pll_get_rate(u32 *pll_con)
{
	u32 refdiv, fbdiv, postdiv1, postdiv2, con1, con3;

	con3 = readl(&pll_con[3]);

	switch ((con3 & PLL_MODE_MASK) >> PLL_MODE_SHIFT) {
	case PLL_MODE_SLOW:
		return OSC_HZ;
	case PLL_MODE_DEEP:
		return 32768;
	}

	/* Fractional mode is not used by anything here. */
	if (!(con3 & PLL_DSMPD_MASK))
		return 0;

	fbdiv    = (readl(&pll_con[0]) & PLL_FBDIV_MASK) >> PLL_FBDIV_SHIFT;
	con1     = readl(&pll_con[1]);
	refdiv   = (con1 & PLL_REFDIV_MASK) >> PLL_REFDIV_SHIFT;
	postdiv1 = (con1 & PLL_POSTDIV1_MASK) >> PLL_POSTDIV1_SHIFT;
	postdiv2 = (con1 & PLL_POSTDIV2_MASK) >> PLL_POSTDIV2_SHIFT;

	if (!refdiv || !postdiv1 || !postdiv2)
		return 0;

	return (ulong)((u64)OSC_HZ * fbdiv / refdiv / postdiv1 / postdiv2);
}

static ulong rk3399_dp_get_clk(struct rockchip_cru *cru)
{
	u32 con, div, sel;
	ulong parent;

	con = readl(&cru->clksel_con[46]);
	sel = (con & DP_CORE_PLL_SEL_MASK) >> DP_CORE_PLL_SEL_SHIFT;
	div = (con & DP_CORE_DIV_CON_MASK) >> DP_CORE_DIV_CON_SHIFT;

	switch (sel) {
	case DP_CORE_PLL_SEL_NPLL:
		parent = rk3399_pll_get_rate(&cru->npll_con[0]);
		break;
	case DP_CORE_PLL_SEL_CPLL:
		parent = CPLL_HZ;
		break;
	case DP_CORE_PLL_SEL_GPLL:
		parent = GPLL_HZ;
		break;
	default:
		return -EINVAL;
	}

	return DIV_TO_RATE(parent, div);
}

static ulong rk3399_dp_set_clk(struct rockchip_cru *cru, ulong hz)
{
	u32 div;

	if (!hz)
		return -EINVAL;

	div = NPLL_HZ / hz;
	if (!div || div > 32 || div * hz != NPLL_HZ) {
		log_debug("Cannot derive %lu Hz from NPLL\n", hz);
		return -EINVAL;
	}

	/*
	 * NPLL is the only parent that divides to 100 MHz exactly (1200 / 12);
	 * brought up on demand, rate-checked -- see rk3399_npll_ensure().
	 */
	rk3399_npll_ensure(cru);

	rk_clrsetreg(&cru->clksel_con[46],
		     DP_CORE_PLL_SEL_MASK | DP_CORE_DIV_CON_MASK,
		     DP_CORE_PLL_SEL_NPLL << DP_CORE_PLL_SEL_SHIFT |
		     (div - 1) << DP_CORE_DIV_CON_SHIFT);

	return rk3399_dp_get_clk(cru);
}

static ulong rk3399_spdif_rec_dptx_get_clk(struct rockchip_cru *cru)
{
	u32 con = readl(&cru->clksel_con[32]);
	u32 div = (con & SPDIF_REC_DPTX_DIV_CON_MASK) >>
		  SPDIF_REC_DPTX_DIV_CON_SHIFT;
	ulong parent = (con & SPDIF_REC_DPTX_PLL_SEL_MASK) ? GPLL_HZ : CPLL_HZ;

	return DIV_TO_RATE(parent, div);
}

static ulong rk3399_spdif_rec_dptx_set_clk(struct rockchip_cru *cru, ulong hz)
{
	u32 div = DIV_ROUND_UP(GPLL_HZ, hz);

	/*
	 * cdn-dp asks for 200 MHz, which is exact from the 800 MHz CPLL Linux
	 * runs but unreachable from U-Boot's 384 MHz CPLL and 594 MHz GPLL --
	 * the closest is GPLL/3 = 198 MHz. Program that and RETURN IT, rather
	 * than reporting the requested rate: this clock feeds only the SPDIF
	 * audio path (cdn_dp_audio_config()), never EDID, DPCD, HPD or link
	 * training, so 1% off is harmless -- but a get_rate that lies about it
	 * would not be.
	 *
	 * CPLL is deliberately not moved to 800 MHz for this: it also feeds
	 * perihp, perilp0 and the VOP aclk arithmetic.
	 */
	if (!div || div > 32)
		return -EINVAL;

	rk_clrsetreg(&cru->clksel_con[32],
		     SPDIF_REC_DPTX_PLL_SEL_MASK | SPDIF_REC_DPTX_DIV_CON_MASK,
		     SPDIF_REC_DPTX_PLL_SEL_GPLL <<
		     SPDIF_REC_DPTX_PLL_SEL_SHIFT |
		     (div - 1) << SPDIF_REC_DPTX_DIV_CON_SHIFT);

	return rk3399_spdif_rec_dptx_get_clk(cru);
}

static ulong rk3399_clk_get_rate(struct clk *clk)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate = 0;

	switch (clk->id) {
	/*
	 * The PLLs, read rather than asserted. These used to fall into a
	 * `case 0 ... 63: return 0;` arm that answered ZERO for every PLL --
	 * so anything asking what GPLL or VPLL was running at got a plausible-
	 * looking 0 instead of an error, and the one caller that needed a real
	 * answer (the DisplayPort core clock) had to read the registers
	 * itself. Reading is nearly free and it is the only answer that
	 * survives another stage having programmed the PLL differently.
	 */
	case PLL_APLLL:
		return rk3399_pll_get_rate(priv->cru->apll_l_con);
	case PLL_APLLB:
		return rk3399_pll_get_rate(priv->cru->apll_b_con);
	case PLL_DPLL:
		return rk3399_pll_get_rate(priv->cru->dpll_con);
	case PLL_CPLL:
		return rk3399_pll_get_rate(priv->cru->cpll_con);
	case PLL_GPLL:
		return rk3399_pll_get_rate(priv->cru->gpll_con);
	case PLL_NPLL:
		return rk3399_pll_get_rate(priv->cru->npll_con);
	case PLL_VPLL:
		return rk3399_pll_get_rate(priv->cru->vpll_con);
	/* 0 and ARMCLKL/ARMCLKB (8, 9) upward -- the PLLs are handled above. */
	case 0:
	case ARMCLKL ... 63:
		return 0;
	case HCLK_SDIO:
	case SCLK_SDIO:
	case HCLK_SDMMC:
	case SCLK_SDMMC:
	case SCLK_EMMC:
		rate = rk3399_mmc_get_clk(priv->cru, clk->id);
		break;
	case SCLK_SDMMC_DRV:
	case SCLK_SDMMC_SAMPLE:
		rate = rk3399_mmc_get_clk(priv->cru, SCLK_SDMMC);
		break;
	case SCLK_SDIO_DRV:
	case SCLK_SDIO_SAMPLE:
		rate = rk3399_mmc_get_clk(priv->cru, SCLK_SDIO);
		break;
	case SCLK_I2C1:
	case SCLK_I2C2:
	case SCLK_I2C3:
	case SCLK_I2C5:
	case SCLK_I2C6:
	case SCLK_I2C7:
		rate = rk3399_i2c_get_clk(priv->cru, clk->id);
		break;
	case SCLK_SPI0...SCLK_SPI5:
		rate = rk3399_spi_get_clk(priv->cru, clk->id);
		break;
	case SCLK_UART0:
	case SCLK_UART1:
	case SCLK_UART2:
	case SCLK_UART3:
	case SCLK_USB3OTG0_REF:
	case SCLK_USB3OTG1_REF:
		return OSC_HZ;
	case PCLK_HDMI_CTRL:
		break;
	case DCLK_VOP0:
	case DCLK_VOP1:
		break;
	case PCLK_EFUSE1024NS:
		break;
	case SCLK_SARADC:
		rate = rk3399_saradc_get_clk(priv->cru);
		break;
	case SCLK_PCIEPHY_REF:
		rate = rk3399_pciephy_get_clk(priv->cru);
		break;
	case SCLK_DP_CORE:
		rate = rk3399_dp_get_clk(priv->cru);
		break;
	case SCLK_SPDIF_REC_DPTX:
		rate = rk3399_spdif_rec_dptx_get_clk(priv->cru);
		break;
	case PCLK_DP_CTRL:
		/*
		 * A gate with no divider, whose parent (pclk_hdcp) this
		 * driver never programs -- so the rate is genuinely unknown
		 * rather than zero. Nothing asks for it: cdn-dp calls
		 * clk_get_rate() only on its core clock.
		 */
		break;
	case ACLK_VIO:
	case ACLK_HDCP:
	case ACLK_GIC_PRE:
	case PCLK_DDR:
	case ACLK_VDU:
		break;
	case PCLK_ALIVE:
	case PCLK_WDT:
		rate = rk3399_alive_get_clk(priv->cru);
		break;
	default:
		log_debug("Unknown clock %lu\n", clk->id);
		return -ENOENT;
	}

	return rate;
}

static ulong rk3399_clk_set_rate(struct clk *clk, ulong rate)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);
	ulong ret = 0;

	switch (clk->id) {
	/*
	 * Accept the request and report where the PLL actually is. We do not
	 * move it -- see below -- but the answer must be truthful, and it
	 * must not be an error.
	 *
	 * 🔴 RETURNING AN ERROR HERE BREAKS THE BOARD, and not through any
	 * clock path. rk3399_clk_bind() calls rockchip_reset_bind(), which
	 * does device_bind_driver_to_node(..., "rockchip_reset", "reset",
	 * dev_ofnode(pdev), ...) -- so a SECOND udevice, in UCLASS_RESET,
	 * shares this CRU node, and with it the node's 17
	 * assigned-clock-rates. When that reset device probes,
	 * device_probe() runs clk_set_defaults(dev, CLK_DEFAULTS_PRE) on it
	 * and any failure is fatal. The "skip a provider's own clocks" guard
	 * does not save us: it tests `clk.dev == dev`, and here clk.dev is
	 * the CRU while dev is the reset device, so the rates really are
	 * applied -- by the reset device's probe.
	 *
	 * An error here therefore kills the reset provider, and every &cru
	 * reset consumer dies with it. Measured, returning -ENOSYS:
	 *     rk3399_vop vop@ff8f0000: failed to get ahb reset (ret=-38)
	 *     Bus usb@fe900000: probe failed, error -38
	 * those being merely the two that print -- dwc3_glue_reset_init()
	 * tolerates only -ENOTSUPP and -ENOENT, and -ENOSYS is neither.
	 *
	 * The identical trap is already documented one function down, where
	 * PLL_PPLL returns PPLL_HZ rather than an error, for a pmucru node
	 * with the same bind-a-reset-onto-my-own-node structure.
	 *
	 * Returning the MEASURED rate rather than a constant is what makes
	 * this honest. clk_set_rate()'s contract is "the rate you ended up
	 * at", so this says "I did not move it, here is where it is", and a
	 * caller can compare that against what it asked for. Which matters:
	 * the DT asks for CPLL at 800 MHz while this driver runs it at
	 * 384 MHz with CPLL_HZ hardcoded into every DIV_TO_RATE() consumer,
	 * and the old `return 0` made that divergence undetectable.
	 *
	 * Actually honouring the assignment is a bigger change than wiring up
	 * rkclk_set_pll(): those DIV_TO_RATE() consumers must read the PLL
	 * first, or re-rating CPLL silently re-rates i2c, spi and mmc.
	 */
	case PLL_APLLL:
		return rk3399_pll_get_rate(priv->cru->apll_l_con);
	case PLL_APLLB:
		return rk3399_pll_get_rate(priv->cru->apll_b_con);
	case PLL_DPLL:
		return rk3399_pll_get_rate(priv->cru->dpll_con);
	case PLL_CPLL:
		return rk3399_pll_get_rate(priv->cru->cpll_con);
	case PLL_GPLL:
		return rk3399_pll_get_rate(priv->cru->gpll_con);
	case PLL_NPLL:
		return rk3399_pll_get_rate(priv->cru->npll_con);
	case PLL_VPLL:
		return rk3399_pll_get_rate(priv->cru->vpll_con);
	/* 0 and ARMCLKL/ARMCLKB (8, 9) upward -- the PLLs are handled above. */
	case 0:
	case ARMCLKL ... 63:
		return 0;

	case ACLK_PERIHP:
	case HCLK_PERIHP:
	case PCLK_PERIHP:
		return 0;

	case ACLK_PERILP0:
	case HCLK_PERILP0:
	case PCLK_PERILP0:
		return 0;

	case ACLK_CCI:
		return 0;

	case HCLK_PERILP1:
	case PCLK_PERILP1:
		return 0;

	case HCLK_SDIO:
	case SCLK_SDIO:
	case HCLK_SDMMC:
	case SCLK_SDMMC:
	case SCLK_EMMC:
		ret = rk3399_mmc_set_clk(priv->cru, clk->id, rate);
		break;
	case SCLK_MAC:
		ret = rk3399_gmac_set_clk(priv->cru, rate);
		break;
	case SCLK_I2C1:
	case SCLK_I2C2:
	case SCLK_I2C3:
	case SCLK_I2C5:
	case SCLK_I2C6:
	case SCLK_I2C7:
		ret = rk3399_i2c_set_clk(priv->cru, clk->id, rate);
		break;
	case SCLK_SPI0...SCLK_SPI5:
		ret = rk3399_spi_set_clk(priv->cru, clk->id, rate);
		break;
	case PCLK_HDMI_CTRL:
	case PCLK_VIO_GRF:
		/* the PCLK gates for video are enabled by default */
		break;
	case SCLK_DP_CORE:
		ret = rk3399_dp_set_clk(priv->cru, rate);
		break;
	case SCLK_SPDIF_REC_DPTX:
		ret = rk3399_spdif_rec_dptx_set_clk(priv->cru, rate);
		break;
	case DCLK_VOP0:
	case DCLK_VOP1:
		ret = rk3399_vop_set_clk(priv->cru, clk->id, rate);
		break;
	case ACLK_VOP0:
	case HCLK_VOP0:
	case ACLK_VOP1:
	case HCLK_VOP1:
	case HCLK_SD:
	case SCLK_UPHY0_TCPDCORE:
	case SCLK_UPHY1_TCPDCORE:
		/**
		 * assigned-clocks handling is not required for EITHER VOP, so
		 * return 0 to satisfy clk_set_defaults during device probe.
		 *
		 * The VOP0 pair was missing, and its absence stopped VOPB
		 * probing at all. Both VOP nodes carry assigned-clocks, and
		 * clk_set_defaults(CLK_DEFAULTS_PRE) runs at the very end of
		 * device_probe()'s pre-phase -- so ACLK_VOP0/HCLK_VOP0 fell to
		 * the default arm, returned -ENOENT, and device_probe() failed
		 * before drv->probe() was ever called. rk_vop_probe() never
		 * ran, which is why not one of its own error paths printed.
		 *
		 * It was silent as well as fatal: the single dev_warn() on that
		 * path is LOGL_WARNING, and with CONFIG_LOG off and
		 * CONFIG_LOGLEVEL=4 the `level < LOGLEVEL` test makes 4 < 4
		 * false. Every error inside rk_vop_probe() uses dev_err
		 * (LOGL_ERR, 3) and WOULD have printed -- so the total absence
		 * of output was itself the evidence that the failure was
		 * upstream of the driver.
		 *
		 * Returning 0 is honest rather than a paper-over:
		 * rk3399_vop_set_clk() already programs aclk_vop's mux and
		 * divider (clksel_con[47] for VOP0, [48] for VOP1) on every
		 * DCLK_VOP* set, so the DT's 400 MHz aclk assignment is
		 * redundant at bring-up, and hclk_vop* is a plain gate handled
		 * in rk3399_clk_enable(). rk3399_clk_get_rate() has no case for
		 * any of these four either, so reporting a measured rate is not
		 * available without new code -- matching VOP1's long-standing
		 * form is the correct minimal fix.
		 *
		 * Upstream gap, not a local regression: only VOPL was ever
		 * driven on RK3399 upstream.
		 */
		return 0;
	case SCLK_DDRC:
		ret = rk3399_ddr_set_clk(priv->cru, rate);
		break;
	case PCLK_EFUSE1024NS:
		break;
	case SCLK_SARADC:
		ret = rk3399_saradc_set_clk(priv->cru, rate);
		break;
	case SCLK_PCIEPHY_REF:
		ret = rk3399_pciephy_set_clk(priv->cru, rate);
		break;
	case ACLK_VIO:
	case ACLK_HDCP:
	case ACLK_GIC_PRE:
	case PCLK_DDR:
	case ACLK_VDU:
		return 0;
	default:
		log_debug("Unknown clock %lu\n", clk->id);
		return -ENOENT;
	}

	return ret;
}

static int __maybe_unused rk3399_gmac_set_parent(struct clk *clk,
						 struct clk *parent)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);
	const char *clock_output_name;
	int ret;

	/*
	 * If the requested parent is in the same clock-controller and
	 * the id is SCLK_MAC ("clk_gmac"), switch to the internal clock.
	 */
	if (parent->dev == clk->dev && parent->id == SCLK_MAC) {
		debug("%s: switching RGMII to SCLK_MAC\n", __func__);
		rk_clrreg(&priv->cru->clksel_con[19], BIT(4));
		return 0;
	}

	/*
	 * Otherwise, we need to check the clock-output-names of the
	 * requested parent to see if the requested id is "clkin_gmac".
	 */
	ret = dev_read_string_index(parent->dev, "clock-output-names",
				    parent->id, &clock_output_name);
	if (ret < 0)
		return -ENODATA;

	/* If this is "clkin_gmac", switch to the external clock input */
	if (!strcmp(clock_output_name, "clkin_gmac")) {
		debug("%s: switching RGMII to CLKIN\n", __func__);
		rk_setreg(&priv->cru->clksel_con[19], BIT(4));
		return 0;
	}

	return -EINVAL;
}

static int __maybe_unused rk3399_pciephy_set_parent(struct clk *clk,
						    struct clk *parent)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);
	const char *clock_output_name;
	int ret;

	if (parent->dev == clk->dev && parent->id == SCLK_PCIEPHY_REF100M) {
		rk_setreg(&priv->cru->clksel_con[18], BIT(10));
		return 0;
	}

	ret = dev_read_string_index(parent->dev, "clock-output-names",
				    parent->id, &clock_output_name);
	if (ret < 0)
		return -ENODATA;

	if (!strcmp(clock_output_name, "xin24m")) {
		rk_clrreg(&priv->cru->clksel_con[18], BIT(10));
		return 0;
	}

	return -EINVAL;
}

static int __maybe_unused rk3399_clk_set_parent(struct clk *clk,
						struct clk *parent)
{
	switch (clk->id) {
	case SCLK_RMII_SRC:
		return rk3399_gmac_set_parent(clk, parent);
	case SCLK_PCIEPHY_REF:
		return rk3399_pciephy_set_parent(clk, parent);
	}

	debug("%s: unsupported clk %ld\n", __func__, clk->id);
	return -ENOENT;
}

static int rk3399_clk_enable(struct clk *clk)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);

	switch (clk->id) {
	case SCLK_MAC:
		rk_clrreg(&priv->cru->clkgate_con[5], BIT(5));
		break;
	case SCLK_MAC_RX:
		rk_clrreg(&priv->cru->clkgate_con[5], BIT(8));
		break;
	case SCLK_MAC_TX:
		rk_clrreg(&priv->cru->clkgate_con[5], BIT(9));
		break;
	case SCLK_MACREF:
		rk_clrreg(&priv->cru->clkgate_con[5], BIT(7));
		break;
	case SCLK_MACREF_OUT:
		rk_clrreg(&priv->cru->clkgate_con[5], BIT(6));
		break;
	case SCLK_USB2PHY0_REF:
		rk_clrreg(&priv->cru->clkgate_con[6], BIT(5));
		break;
	case SCLK_USB2PHY1_REF:
		rk_clrreg(&priv->cru->clkgate_con[6], BIT(6));
		break;
	case ACLK_GMAC:
		rk_clrreg(&priv->cru->clkgate_con[32], BIT(0));
		break;
	case PCLK_GMAC:
		rk_clrreg(&priv->cru->clkgate_con[32], BIT(2));
		break;
	case SCLK_USB3OTG0_REF:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(1));
		break;
	case SCLK_USB3OTG1_REF:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(2));
		break;
	case SCLK_USB3OTG0_SUSPEND:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(3));
		break;
	case SCLK_USB3OTG1_SUSPEND:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(4));
		break;
	case ACLK_USB3OTG0:
		rk_clrreg(&priv->cru->clkgate_con[30], BIT(1));
		break;
	case ACLK_USB3OTG1:
		rk_clrreg(&priv->cru->clkgate_con[30], BIT(2));
		break;
	case ACLK_USB3_RKSOC_AXI_PERF:
		rk_clrreg(&priv->cru->clkgate_con[30], BIT(3));
		break;
	case ACLK_USB3:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(0));
		break;
	case ACLK_USB3_GRF:
		rk_clrreg(&priv->cru->clkgate_con[30], BIT(4));
		break;
	case HCLK_HOST0:
		rk_clrreg(&priv->cru->clksel_con[20], BIT(5));
		break;
	case HCLK_HOST0_ARB:
		rk_clrreg(&priv->cru->clksel_con[20], BIT(6));
		break;
	case HCLK_HOST1:
		rk_clrreg(&priv->cru->clksel_con[20], BIT(7));
		break;
	case HCLK_HOST1_ARB:
		rk_clrreg(&priv->cru->clksel_con[20], BIT(8));
		break;
	case SCLK_UPHY0_TCPDPHY_REF:
		rk_clrreg(&priv->cru->clkgate_con[13], BIT(4));
		break;
	case SCLK_UPHY0_TCPDCORE:
		rk_clrreg(&priv->cru->clkgate_con[13], BIT(5));
		break;
	case SCLK_UPHY1_TCPDPHY_REF:
		rk_clrreg(&priv->cru->clkgate_con[13], BIT(6));
		break;
	case SCLK_UPHY1_TCPDCORE:
		rk_clrreg(&priv->cru->clkgate_con[13], BIT(7));
		break;
	case SCLK_PCIEPHY_REF:
		if (readl(&priv->cru->clksel_con[18]) & BIT(10))
			rk_clrreg(&priv->cru->clkgate_con[12], BIT(6));
		break;
	/*
	 * The DisplayPort controller's four clocks. All are ungated at reset
	 * (measured: CLKGATE_CON11 and CON29 both read 0), so these writes are
	 * idempotent -- but "enabled by default" is a claim about reset, not
	 * about the state U-Boot inherits from TPL, SPL or a prior boot, and
	 * every other case here does a real write.
	 */
	case SCLK_DP_CORE:
		rk_clrreg(&priv->cru->clkgate_con[11], BIT(8));
		break;
	case SCLK_SPDIF_REC_DPTX:
		rk_clrreg(&priv->cru->clkgate_con[10], BIT(6));
		break;
	case PCLK_DP_CTRL:
		rk_clrreg(&priv->cru->clkgate_con[29], BIT(7));
		break;
	case PCLK_VIO_GRF:
		rk_clrreg(&priv->cru->clkgate_con[29], BIT(12));
		break;
	/*
	 * The power-domain clocks. rk3399-base.dtsi's power-controller names
	 * 34 distinct clocks across its power-domain@N nodes, and the NoC
	 * bus-idle handshake a power-domain driver has to drive is itself
	 * CLOCKED -- assert BUS_IDLE_REQ on a domain whose bus clock is gated
	 * and the ACK never arrives. So every one of them has to be
	 * ungatable, or the domain cannot be brought up at all.
	 *
	 * Register and bit for each come from Linux's clk-rk3399.c branch
	 * table. Nothing in this tree calls clk_enable() with any of these
	 * ids yet, so all of it is unreachable until the power-domain driver
	 * lands -- which is the point: it is a prerequisite, and it changes
	 * no register on any current boot path.
	 */
	case ACLK_IEP:
		rk_clrreg(&priv->cru->clkgate_con[16], BIT(0));
		break;
	case HCLK_IEP:
		rk_clrreg(&priv->cru->clkgate_con[16], BIT(2));
		break;
	case ACLK_RGA:
		rk_clrreg(&priv->cru->clkgate_con[16], BIT(8));
		break;
	case HCLK_RGA:
		rk_clrreg(&priv->cru->clkgate_con[16], BIT(10));
		break;
	case ACLK_VCODEC:
		rk_clrreg(&priv->cru->clkgate_con[17], BIT(0));
		break;
	case HCLK_VCODEC:
		rk_clrreg(&priv->cru->clkgate_con[17], BIT(2));
		break;
	case ACLK_VDU:
		rk_clrreg(&priv->cru->clkgate_con[17], BIT(8));
		break;
	case HCLK_VDU:
		rk_clrreg(&priv->cru->clkgate_con[17], BIT(10));
		break;
	case SCLK_VDU_CORE:
		rk_clrreg(&priv->cru->clkgate_con[4], BIT(4));
		break;
	case SCLK_VDU_CA:
		rk_clrreg(&priv->cru->clkgate_con[4], BIT(5));
		break;
	case ACLK_GPU:
		rk_clrreg(&priv->cru->clkgate_con[30], BIT(8));
		break;
	case PCLK_EDP_CTRL:
		rk_clrreg(&priv->cru->clkgate_con[32], BIT(13));
		break;
	/*
	 * aclk_emmc is COMPOSITE_NOGATE upstream -- a mux and a divider with
	 * no gate bit behind them, so there is nothing to write. PD_EMMC
	 * names it anyway, and clk_enable_bulk() forgives only -ENOSYS, so
	 * falling through to default's -ENOENT would fail that domain over a
	 * clock that is structurally always on.
	 */
	case ACLK_EMMC:
		break;
	/*
	 * pclk_wdt's gate is SECURE_GRF_SOC_CON3[8] (Linux models it as an
	 * SGRF_GATE), not a CRU bit, and the non-secure world cannot write it;
	 * it is on out of reset. Without a case here designware_wdt_probe()
	 * fails on -ENOENT, initr_watchdog() drops the device with a debug
	 * print, and no watchdog is ever started or stopped on RK3399.
	 */
	case PCLK_WDT:
		break;
	case HCLK_SDMMC:
		rk_clrreg(&priv->cru->clkgate_con[33], BIT(8));
		break;
	case SCLK_SDMMC:
		rk_clrreg(&priv->cru->clkgate_con[6], BIT(1));
		break;
	case HCLK_SDIO:
		rk_clrreg(&priv->cru->clkgate_con[34], BIT(4));
		break;
	case HCLK_HDCP:
		rk_clrreg(&priv->cru->clkgate_con[11], BIT(3));
		break;
	case PCLK_HDCP:
		rk_clrreg(&priv->cru->clkgate_con[11], BIT(10));
		break;
	case ACLK_HDCP:
		rk_clrreg(&priv->cru->clkgate_con[11], BIT(12));
		break;
	case ACLK_ISP0:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(8));
		break;
	case HCLK_ISP0:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(9));
		break;
	case ACLK_ISP1:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(10));
		break;
	case HCLK_ISP1:
		rk_clrreg(&priv->cru->clkgate_con[12], BIT(11));
		break;
	case HCLK_VOP0:
		rk_clrreg(&priv->cru->clkgate_con[28], BIT(2));
		break;
	case ACLK_VOP0:
		rk_clrreg(&priv->cru->clkgate_con[28], BIT(3));
		break;
	case HCLK_VOP1:
		rk_clrreg(&priv->cru->clkgate_con[28], BIT(6));
		break;
	case ACLK_VOP1:
		rk_clrreg(&priv->cru->clkgate_con[28], BIT(7));
		break;
	default:
		debug("%s: unsupported clk %ld\n", __func__, clk->id);
		return -ENOENT;
	}

	return 0;
}

static int rk3399_clk_disable(struct clk *clk)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);

	switch (clk->id) {
	case SCLK_MAC:
		rk_setreg(&priv->cru->clkgate_con[5], BIT(5));
		break;
	case SCLK_MAC_RX:
		rk_setreg(&priv->cru->clkgate_con[5], BIT(8));
		break;
	case SCLK_MAC_TX:
		rk_setreg(&priv->cru->clkgate_con[5], BIT(9));
		break;
	case SCLK_MACREF:
		rk_setreg(&priv->cru->clkgate_con[5], BIT(7));
		break;
	case SCLK_MACREF_OUT:
		rk_setreg(&priv->cru->clkgate_con[5], BIT(6));
		break;
	case SCLK_USB2PHY0_REF:
		rk_setreg(&priv->cru->clkgate_con[6], BIT(5));
		break;
	case SCLK_USB2PHY1_REF:
		rk_setreg(&priv->cru->clkgate_con[6], BIT(6));
		break;
	case ACLK_GMAC:
		rk_setreg(&priv->cru->clkgate_con[32], BIT(0));
		break;
	case PCLK_GMAC:
		rk_setreg(&priv->cru->clkgate_con[32], BIT(2));
		break;
	case SCLK_USB3OTG0_REF:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(1));
		break;
	case SCLK_USB3OTG1_REF:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(2));
		break;
	case SCLK_USB3OTG0_SUSPEND:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(3));
		break;
	case SCLK_USB3OTG1_SUSPEND:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(4));
		break;
	case ACLK_USB3OTG0:
		rk_setreg(&priv->cru->clkgate_con[30], BIT(1));
		break;
	case ACLK_USB3OTG1:
		rk_setreg(&priv->cru->clkgate_con[30], BIT(2));
		break;
	case ACLK_USB3_RKSOC_AXI_PERF:
		rk_setreg(&priv->cru->clkgate_con[30], BIT(3));
		break;
	case ACLK_USB3:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(0));
		break;
	case ACLK_USB3_GRF:
		rk_setreg(&priv->cru->clkgate_con[30], BIT(4));
		break;
	case HCLK_HOST0:
		rk_setreg(&priv->cru->clksel_con[20], BIT(5));
		break;
	case HCLK_HOST0_ARB:
		rk_setreg(&priv->cru->clksel_con[20], BIT(6));
		break;
	case HCLK_HOST1:
		rk_setreg(&priv->cru->clksel_con[20], BIT(7));
		break;
	case HCLK_HOST1_ARB:
		rk_setreg(&priv->cru->clksel_con[20], BIT(8));
		break;
	case SCLK_UPHY0_TCPDPHY_REF:
		rk_setreg(&priv->cru->clkgate_con[13], BIT(4));
		break;
	case SCLK_UPHY0_TCPDCORE:
		rk_setreg(&priv->cru->clkgate_con[13], BIT(5));
		break;
	case SCLK_UPHY1_TCPDPHY_REF:
		rk_setreg(&priv->cru->clkgate_con[13], BIT(6));
		break;
	case SCLK_UPHY1_TCPDCORE:
		rk_setreg(&priv->cru->clkgate_con[13], BIT(7));
		break;
	case SCLK_PCIEPHY_REF:
		if (readl(&priv->cru->clksel_con[18]) & BIT(10))
			rk_setreg(&priv->cru->clkgate_con[12], BIT(6));
		break;
	/*
	 * The DisplayPort clocks' disable half. Without these a driver's
	 * probe-failure unwind hits -ENOENT out of clk_disable_bulk(), which
	 * forgives only -ENOSYS -- the same failure the enable side was added
	 * to fix, one direction over. PCLK_VIO_GRF is deliberately absent:
	 * the TRM annotates it "suggest always on".
	 */
	case SCLK_DP_CORE:
		rk_setreg(&priv->cru->clkgate_con[11], BIT(8));
		break;
	case SCLK_SPDIF_REC_DPTX:
		rk_setreg(&priv->cru->clkgate_con[10], BIT(6));
		break;
	case PCLK_DP_CTRL:
		rk_setreg(&priv->cru->clkgate_con[29], BIT(7));
		break;
	/*
	 * The power-domain clocks' disable half. FOUR of the enable-side ids
	 * above are deliberately absent here, for the same reason
	 * PCLK_VIO_GRF is:
	 *
	 * ACLK_HDCP, HCLK_HDCP and PCLK_HDCP are the PARENTS of
	 * aclk_hdcp_noc / hclk_hdcp_noc / pclk_hdcp_noc, which upstream lists
	 * in rk3399_cru_critical_clocks[]. Those NoC clocks have their own
	 * gate bits in CLKGATE_CON(29) and would stay "enabled" while their
	 * source disappeared -- gating the parent takes the VIO fabric down
	 * with it, and nothing would bring it back. ACLK_EMMC is absent
	 * because it has no gate bit to write.
	 *
	 * The cost of each omission is the one the enable side exists to fix,
	 * one direction over: a consumer that bulk-disables one of them gets
	 * -ENOENT, which clk_disable_bulk() does not forgive. That is the
	 * deliberate trade -- a failed unwind beats a hung fabric.
	 */
	case ACLK_IEP:
		rk_setreg(&priv->cru->clkgate_con[16], BIT(0));
		break;
	case HCLK_IEP:
		rk_setreg(&priv->cru->clkgate_con[16], BIT(2));
		break;
	case ACLK_RGA:
		rk_setreg(&priv->cru->clkgate_con[16], BIT(8));
		break;
	case HCLK_RGA:
		rk_setreg(&priv->cru->clkgate_con[16], BIT(10));
		break;
	case ACLK_VCODEC:
		rk_setreg(&priv->cru->clkgate_con[17], BIT(0));
		break;
	case HCLK_VCODEC:
		rk_setreg(&priv->cru->clkgate_con[17], BIT(2));
		break;
	case ACLK_VDU:
		rk_setreg(&priv->cru->clkgate_con[17], BIT(8));
		break;
	case HCLK_VDU:
		rk_setreg(&priv->cru->clkgate_con[17], BIT(10));
		break;
	case SCLK_VDU_CORE:
		rk_setreg(&priv->cru->clkgate_con[4], BIT(4));
		break;
	case SCLK_VDU_CA:
		rk_setreg(&priv->cru->clkgate_con[4], BIT(5));
		break;
	case ACLK_GPU:
		rk_setreg(&priv->cru->clkgate_con[30], BIT(8));
		break;
	case PCLK_EDP_CTRL:
		rk_setreg(&priv->cru->clkgate_con[32], BIT(13));
		break;
	case HCLK_SDMMC:
		rk_setreg(&priv->cru->clkgate_con[33], BIT(8));
		break;
	/*
	 * SCLK_SDMMC is the SD card clock, and most RK3399 boards boot from
	 * SD. Nothing in this tree disables it, and this case only fires on an
	 * explicit request -- but a driver that gates it mid-boot loses its
	 * boot device, so it is worth saying out loud.
	 */
	case SCLK_SDMMC:
		rk_setreg(&priv->cru->clkgate_con[6], BIT(1));
		break;
	case HCLK_SDIO:
		rk_setreg(&priv->cru->clkgate_con[34], BIT(4));
		break;
	case ACLK_ISP0:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(8));
		break;
	case HCLK_ISP0:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(9));
		break;
	case ACLK_ISP1:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(10));
		break;
	case HCLK_ISP1:
		rk_setreg(&priv->cru->clkgate_con[12], BIT(11));
		break;
	case HCLK_VOP0:
		rk_setreg(&priv->cru->clkgate_con[28], BIT(2));
		break;
	case ACLK_VOP0:
		rk_setreg(&priv->cru->clkgate_con[28], BIT(3));
		break;
	case HCLK_VOP1:
		rk_setreg(&priv->cru->clkgate_con[28], BIT(6));
		break;
	case ACLK_VOP1:
		rk_setreg(&priv->cru->clkgate_con[28], BIT(7));
		break;
	default:
		debug("%s: unsupported clk %ld\n", __func__, clk->id);
		return -ENOENT;
	}

	return 0;
}

/*
 * MMC drive / sample phase clocks (SDMMC_CON0/1, SDIO_CON0/1), the same
 * delay line Linux drives through clk-mmc-phase.c: a 90-degree coarse
 * select plus up to 255 fine delay elements of ~60 ps each, both relative to
 * the card clock (the CRU source divided by the controller's fixed 2).
 */
#define RK3399_MMC_PHASE_SHIFT		1
#define RK3399_MMC_PHASE_MASK		0x07ff
#define RK3399_MMC_DELAY_SEL		BIT(10)
#define RK3399_MMC_DEGREE_MASK		0x3
#define RK3399_MMC_DELAYNUM_OFFSET	2
#define RK3399_MMC_DELAYNUM_MASK	(0xff << RK3399_MMC_DELAYNUM_OFFSET)
#define RK3399_MMC_DELAY_ELEMENT_PSEC	60

static u32 *rk3399_mmc_phase_reg(struct rockchip_cru *cru, ulong clk_id,
				 ulong *rate_id)
{
	switch (clk_id) {
	case SCLK_SDMMC_DRV:
		*rate_id = SCLK_SDMMC;
		return &cru->sdmmc_con[0];
	case SCLK_SDMMC_SAMPLE:
		*rate_id = SCLK_SDMMC;
		return &cru->sdmmc_con[1];
	case SCLK_SDIO_DRV:
		*rate_id = SCLK_SDIO;
		return &cru->sdio0_con[0];
	case SCLK_SDIO_SAMPLE:
		*rate_id = SCLK_SDIO;
		return &cru->sdio0_con[1];
	default:
		return NULL;
	}
}

static int rk3399_clk_get_phase(struct clk *clk)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate_id, rate;
	u32 raw, delay_num;
	u32 *reg;
	int degrees;

	reg = rk3399_mmc_phase_reg(priv->cru, clk->id, &rate_id);
	if (!reg)
		return -ENOSYS;

	rate = rk3399_mmc_get_clk(priv->cru, rate_id);
	if (!rate)
		return 0;

	raw = readl(reg) >> RK3399_MMC_PHASE_SHIFT;
	degrees = (raw & RK3399_MMC_DEGREE_MASK) * 90;

	if (raw & RK3399_MMC_DELAY_SEL) {
		/* degrees per delay element, scaled by 1000000 */
		ulong factor = (RK3399_MMC_DELAY_ELEMENT_PSEC / 10) *
			       36 * (rate / 10000);

		delay_num = (raw & RK3399_MMC_DELAYNUM_MASK) >>
			    RK3399_MMC_DELAYNUM_OFFSET;
		degrees += DIV_ROUND_CLOSEST(delay_num * factor, 1000000);
	}

	return degrees % 360;
}

static int rk3399_clk_set_phase(struct clk *clk, int degrees)
{
	struct rk3399_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate_id, rate;
	u32 nineties, remainder, delay, delay_num, raw;
	u32 *reg;

	reg = rk3399_mmc_phase_reg(priv->cru, clk->id, &rate_id);
	if (!reg)
		return -ENOSYS;

	if (degrees < 0 || degrees >= 360)
		return -EINVAL;

	rate = rk3399_mmc_get_clk(priv->cru, rate_id);
	if (!rate)
		return -EINVAL;

	nineties = degrees / 90;
	remainder = degrees % 90;

	/*
	 * Fine delay elements for the remainder, at the current card clock;
	 * each element is 44..77 ps, so the result can be up to ~25 degrees
	 * off -- the tuning sweep tolerates that (the same approximation
	 * Linux makes).
	 */
	delay = 10000000; /* PSECS_PER_SEC / 10000 / 10 */
	delay *= remainder;
	delay = DIV_ROUND_CLOSEST(delay,
				  (rate / 1000) * 36 *
				  (RK3399_MMC_DELAY_ELEMENT_PSEC / 10));
	delay_num = min_t(u32, delay, 255);

	raw = delay_num ? RK3399_MMC_DELAY_SEL : 0;
	raw |= delay_num << RK3399_MMC_DELAYNUM_OFFSET;
	raw |= nineties;

	rk_clrsetreg(reg, RK3399_MMC_PHASE_MASK << RK3399_MMC_PHASE_SHIFT,
		     raw << RK3399_MMC_PHASE_SHIFT);

	debug("%s: clk %ld set_phase(%d) delay_num=%u reg=0x%08x actual=%d\n",
	      __func__, clk->id, degrees, delay_num, readl(reg),
	      rk3399_clk_get_phase(clk));

	return 0;
}

static struct clk_ops rk3399_clk_ops = {
	.get_rate = rk3399_clk_get_rate,
	.set_rate = rk3399_clk_set_rate,
#if CONFIG_IS_ENABLED(OF_REAL)
	.set_parent = rk3399_clk_set_parent,
#endif
	.get_phase = rk3399_clk_get_phase,
	.set_phase = rk3399_clk_set_phase,
	.enable = rk3399_clk_enable,
	.disable = rk3399_clk_disable,
};

static void rkclk_init(struct rockchip_cru *cru)
{
	u32 aclk_div;
	u32 hclk_div;
	u32 pclk_div;

	rk3399_configure_cpu_l(cru, APLL_L_600_MHZ);
	rk3399_configure_cpu_b(cru, APLL_B_600_MHZ);
	/*
	 * some cru registers changed by bootrom, we'd better reset them to
	 * reset/default values described in TRM to avoid confusion in kernel.
	 * Please consider these three lines as a fix of bootrom bug.
	 */
	rk_clrsetreg(&cru->clksel_con[12], 0xffff, 0x4101);
	rk_clrsetreg(&cru->clksel_con[19], 0xffff, 0x033f);
	rk_clrsetreg(&cru->clksel_con[56], 0x0003, 0x0003);

	/* configure gpll cpll */
	rkclk_set_pll(&cru->gpll_con[0], &gpll_init_cfg);
	rkclk_set_pll(&cru->cpll_con[0], &cpll_init_cfg);

	/* configure perihp aclk, hclk, pclk */
	aclk_div = GPLL_HZ / PERIHP_ACLK_HZ - 1;
	assert((aclk_div + 1) * PERIHP_ACLK_HZ == GPLL_HZ && aclk_div < 0x1f);

	hclk_div = PERIHP_ACLK_HZ / PERIHP_HCLK_HZ - 1;
	assert((hclk_div + 1) * PERIHP_HCLK_HZ ==
	       PERIHP_ACLK_HZ && (hclk_div < 0x4));

	pclk_div = PERIHP_ACLK_HZ / PERIHP_PCLK_HZ - 1;
	assert((pclk_div + 1) * PERIHP_PCLK_HZ ==
	       PERIHP_ACLK_HZ && (pclk_div < 0x7));

	rk_clrsetreg(&cru->clksel_con[14],
		     PCLK_PERIHP_DIV_CON_MASK | HCLK_PERIHP_DIV_CON_MASK |
		     ACLK_PERIHP_PLL_SEL_MASK | ACLK_PERIHP_DIV_CON_MASK,
		     pclk_div << PCLK_PERIHP_DIV_CON_SHIFT |
		     hclk_div << HCLK_PERIHP_DIV_CON_SHIFT |
		     ACLK_PERIHP_PLL_SEL_GPLL << ACLK_PERIHP_PLL_SEL_SHIFT |
		     aclk_div << ACLK_PERIHP_DIV_CON_SHIFT);

	/* configure perilp0 aclk, hclk, pclk */
	aclk_div = GPLL_HZ / PERILP0_ACLK_HZ - 1;
	assert((aclk_div + 1) * PERILP0_ACLK_HZ == GPLL_HZ && aclk_div < 0x1f);

	hclk_div = PERILP0_ACLK_HZ / PERILP0_HCLK_HZ - 1;
	assert((hclk_div + 1) * PERILP0_HCLK_HZ ==
	       PERILP0_ACLK_HZ && (hclk_div < 0x4));

	pclk_div = PERILP0_ACLK_HZ / PERILP0_PCLK_HZ - 1;
	assert((pclk_div + 1) * PERILP0_PCLK_HZ ==
	       PERILP0_ACLK_HZ && (pclk_div < 0x7));

	rk_clrsetreg(&cru->clksel_con[23],
		     PCLK_PERILP0_DIV_CON_MASK | HCLK_PERILP0_DIV_CON_MASK |
		     ACLK_PERILP0_PLL_SEL_MASK | ACLK_PERILP0_DIV_CON_MASK,
		     pclk_div << PCLK_PERILP0_DIV_CON_SHIFT |
		     hclk_div << HCLK_PERILP0_DIV_CON_SHIFT |
		     ACLK_PERILP0_PLL_SEL_GPLL << ACLK_PERILP0_PLL_SEL_SHIFT |
		     aclk_div << ACLK_PERILP0_DIV_CON_SHIFT);

	/* perilp1 hclk select gpll as source */
	hclk_div = GPLL_HZ / PERILP1_HCLK_HZ - 1;
	assert((hclk_div + 1) * PERILP1_HCLK_HZ ==
	       GPLL_HZ && (hclk_div < 0x1f));

	pclk_div = PERILP1_HCLK_HZ / PERILP1_HCLK_HZ - 1;
	assert((pclk_div + 1) * PERILP1_HCLK_HZ ==
	       PERILP1_HCLK_HZ && (hclk_div < 0x7));

	rk_clrsetreg(&cru->clksel_con[25],
		     PCLK_PERILP1_DIV_CON_MASK | HCLK_PERILP1_DIV_CON_MASK |
		     HCLK_PERILP1_PLL_SEL_MASK,
		     pclk_div << PCLK_PERILP1_DIV_CON_SHIFT |
		     hclk_div << HCLK_PERILP1_DIV_CON_SHIFT |
		     HCLK_PERILP1_PLL_SEL_GPLL << HCLK_PERILP1_PLL_SEL_SHIFT);
}

static int rk3399_clk_probe(struct udevice *dev)
{
	struct rk3399_clk_priv *priv = dev_get_priv(dev);
	bool init_clocks = false;

#if CONFIG_IS_ENABLED(OF_PLATDATA)
	struct rk3399_clk_plat *plat = dev_get_plat(dev);

	priv->cru = map_sysmem(plat->dtd.reg[0], plat->dtd.reg[1]);
#endif

#if defined(CONFIG_XPL_BUILD)
	init_clocks = true;
#elif CONFIG_IS_ENABLED(HANDOFF)
	if (!(gd->flags & GD_FLG_RELOC)) {
		if (!(gd->spl_handoff))
			init_clocks = true;
	}
#endif

	if (init_clocks)
		rkclk_init(priv->cru);

	return 0;
}

static int rk3399_clk_of_to_plat(struct udevice *dev)
{
	if (CONFIG_IS_ENABLED(OF_REAL)) {
		struct rk3399_clk_priv *priv = dev_get_priv(dev);

		priv->cru = dev_read_addr_ptr(dev);
	}

	return 0;
}

static int rk3399_clk_bind(struct udevice *dev)
{
	int ret;
	struct udevice *sys_child;
	struct sysreset_reg *priv;

	/* The reset driver does not have a device node, so bind it here */
	ret = device_bind_driver(dev, "rockchip_sysreset", "sysreset",
				 &sys_child);
	if (ret) {
		debug("Warning: No sysreset driver: ret=%d\n", ret);
	} else {
		priv = malloc(sizeof(struct sysreset_reg));
		priv->glb_srst_fst_value = offsetof(struct rockchip_cru,
						    glb_srst_fst_value);
		priv->glb_srst_snd_value = offsetof(struct rockchip_cru,
						    glb_srst_snd_value);
		dev_set_priv(sys_child, priv);
	}

#if CONFIG_IS_ENABLED(RESET_ROCKCHIP)
	ret = offsetof(struct rockchip_cru, softrst_con[0]);
	ret = rockchip_reset_bind(dev, ret, 21);
	if (ret)
		debug("Warning: software reset driver bind failed\n");
#endif

	return 0;
}

static const struct udevice_id rk3399_clk_ids[] = {
	{ .compatible = "rockchip,rk3399-cru" },
	{ }
};

U_BOOT_DRIVER(clk_rk3399) = {
	.name		= "rockchip_rk3399_cru",
	.id		= UCLASS_CLK,
	.of_match	= rk3399_clk_ids,
	.priv_auto	= sizeof(struct rk3399_clk_priv),
	.of_to_plat = rk3399_clk_of_to_plat,
	.ops		= &rk3399_clk_ops,
	.bind		= rk3399_clk_bind,
	.probe		= rk3399_clk_probe,
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	.plat_auto	= sizeof(struct rk3399_clk_plat),
#endif
};

static ulong rk3399_i2c_get_pmuclk(struct rk3399_pmucru *pmucru, ulong clk_id)
{
	u32 div, con;

	switch (clk_id) {
	case SCLK_I2C0_PMU:
		con = readl(&pmucru->pmucru_clksel[2]);
		div = I2C_CLK_DIV_VALUE(con, 0);
		break;
	case SCLK_I2C4_PMU:
		con = readl(&pmucru->pmucru_clksel[3]);
		div = I2C_CLK_DIV_VALUE(con, 4);
		break;
	case SCLK_I2C8_PMU:
		con = readl(&pmucru->pmucru_clksel[2]);
		div = I2C_CLK_DIV_VALUE(con, 8);
		break;
	default:
		printf("do not support this i2c bus\n");
		return -EINVAL;
	}

	return DIV_TO_RATE(PPLL_HZ, div);
}

static ulong rk3399_i2c_set_pmuclk(struct rk3399_pmucru *pmucru, ulong clk_id,
				   uint hz)
{
	int src_clk_div;

	src_clk_div = PPLL_HZ / hz;
	assert(src_clk_div - 1 < 127);

	switch (clk_id) {
	case SCLK_I2C0_PMU:
		rk_clrsetreg(&pmucru->pmucru_clksel[2], I2C_PMUCLK_REG_MASK(0),
			     I2C_PMUCLK_REG_VALUE(0, src_clk_div));
		break;
	case SCLK_I2C4_PMU:
		rk_clrsetreg(&pmucru->pmucru_clksel[3], I2C_PMUCLK_REG_MASK(4),
			     I2C_PMUCLK_REG_VALUE(4, src_clk_div));
		break;
	case SCLK_I2C8_PMU:
		rk_clrsetreg(&pmucru->pmucru_clksel[2], I2C_PMUCLK_REG_MASK(8),
			     I2C_PMUCLK_REG_VALUE(8, src_clk_div));
		break;
	default:
		printf("do not support this i2c bus\n");
		return -EINVAL;
	}

	return DIV_TO_RATE(PPLL_HZ, src_clk_div);
}

static ulong rk3399_pwm_get_clk(struct rk3399_pmucru *pmucru)
{
	u32 div, con;

	/* PWM closk rate is same as pclk_pmu */
	con = readl(&pmucru->pmucru_clksel[0]);
	div = con & PMU_PCLK_DIV_CON_MASK;

	return DIV_TO_RATE(PPLL_HZ, div);
}

static ulong rk3399_pmuclk_get_rate(struct clk *clk)
{
	struct rk3399_pmuclk_priv *priv = dev_get_priv(clk->dev);
	ulong rate = 0;

	switch (clk->id) {
	case PLL_PPLL:
		return PPLL_HZ;
	case PCLK_RKPWM_PMU:
	case PCLK_WDT_M0_PMU:
		rate = rk3399_pwm_get_clk(priv->pmucru);
		break;
	case SCLK_I2C0_PMU:
	case SCLK_I2C4_PMU:
	case SCLK_I2C8_PMU:
		rate = rk3399_i2c_get_pmuclk(priv->pmucru, clk->id);
		break;
	default:
		return -ENOENT;
	}

	return rate;
}

static ulong rk3399_pmuclk_set_rate(struct clk *clk, ulong rate)
{
	struct rk3399_pmuclk_priv *priv = dev_get_priv(clk->dev);
	ulong ret = 0;

	switch (clk->id) {
	case PLL_PPLL:
		/*
		 * This has already been set up and we don't want/need
		 * to change it here.  Accept the request though, as the
		 * device-tree has this in an 'assigned-clocks' list.
		 */
		return PPLL_HZ;
	case SCLK_I2C0_PMU:
	case SCLK_I2C4_PMU:
	case SCLK_I2C8_PMU:
		ret = rk3399_i2c_set_pmuclk(priv->pmucru, clk->id, rate);
		break;
	default:
		return -ENOENT;
	}

	return ret;
}

static struct clk_ops rk3399_pmuclk_ops = {
	.get_rate = rk3399_pmuclk_get_rate,
	.set_rate = rk3399_pmuclk_set_rate,
};

#ifndef CONFIG_XPL_BUILD
static void pmuclk_init(struct rk3399_pmucru *pmucru)
{
	u32 pclk_div;

	/*  configure pmu pll(ppll) */
	rkclk_set_pll(&pmucru->ppll_con[0], &ppll_init_cfg);

	/*  configure pmu pclk */
	pclk_div = PPLL_HZ / PMU_PCLK_HZ - 1;
	rk_clrsetreg(&pmucru->pmucru_clksel[0],
		     PMU_PCLK_DIV_CON_MASK,
		     pclk_div << PMU_PCLK_DIV_CON_SHIFT);
}
#endif

static int rk3399_pmuclk_probe(struct udevice *dev)
{
#if CONFIG_IS_ENABLED(OF_PLATDATA) || !defined(CONFIG_XPL_BUILD)
	struct rk3399_pmuclk_priv *priv = dev_get_priv(dev);
#endif

#if CONFIG_IS_ENABLED(OF_PLATDATA)
	struct rk3399_pmuclk_plat *plat = dev_get_plat(dev);

	priv->pmucru = map_sysmem(plat->dtd.reg[0], plat->dtd.reg[1]);
#endif

#ifndef CONFIG_XPL_BUILD
	pmuclk_init(priv->pmucru);
#endif
	return 0;
}

static int rk3399_pmuclk_of_to_plat(struct udevice *dev)
{
	if (CONFIG_IS_ENABLED(OF_REAL)) {
		struct rk3399_pmuclk_priv *priv = dev_get_priv(dev);

		priv->pmucru = dev_read_addr_ptr(dev);
	}

	return 0;
}

static int rk3399_pmuclk_bind(struct udevice *dev)
{
#if CONFIG_IS_ENABLED(RESET_ROCKCHIP)
	int ret;

	ret = offsetof(struct rk3399_pmucru, pmucru_softrst_con[0]);
	ret = rockchip_reset_bind(dev, ret, 2);
	if (ret)
		debug("Warning: software reset driver bind failed\n");
#endif
	return 0;
}

static const struct udevice_id rk3399_pmuclk_ids[] = {
	{ .compatible = "rockchip,rk3399-pmucru" },
	{ }
};

U_BOOT_DRIVER(rockchip_rk3399_pmuclk) = {
	.name		= "rockchip_rk3399_pmucru",
	.id		= UCLASS_CLK,
	.of_match	= rk3399_pmuclk_ids,
	.priv_auto	= sizeof(struct rk3399_pmuclk_priv),
	.of_to_plat = rk3399_pmuclk_of_to_plat,
	.ops		= &rk3399_pmuclk_ops,
	.probe		= rk3399_pmuclk_probe,
	.bind		= rk3399_pmuclk_bind,
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	.plat_auto	= sizeof(struct rk3399_pmuclk_plat),
#endif
};
