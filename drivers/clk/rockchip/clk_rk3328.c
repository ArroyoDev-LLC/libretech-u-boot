// SPDX-License-Identifier: GPL-2.0
/*
 * (C) Copyright 2017 Rockchip Electronics Co., Ltd
 */

#include <bitfield.h>
#include <clk-uclass.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <syscon.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/cru_rk3328.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/arch-rockchip/grf_rk3328.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <dt-bindings/clock/rk3328-cru.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/kernel.h>

struct pll_div {
	u32 refdiv;
	u32 fbdiv;
	u32 postdiv1;
	u32 postdiv2;
	u32 frac;
};

#define RATE_TO_DIV(input_rate, output_rate) \
	((input_rate) / (output_rate) - 1);
#define DIV_TO_RATE(input_rate, div)    ((input_rate) / ((div) + 1))

#define PLL_DIVISORS(hz, _refdiv, _postdiv1, _postdiv2) {\
	.refdiv = _refdiv,\
	.fbdiv = (u32)((u64)hz * _refdiv * _postdiv1 * _postdiv2 / OSC_HZ),\
	.postdiv1 = _postdiv1, .postdiv2 = _postdiv2};

static const struct pll_div gpll_init_cfg = PLL_DIVISORS(GPLL_HZ, 1, 4, 1);
static const struct pll_div cpll_init_cfg = PLL_DIVISORS(CPLL_HZ, 1, 1, 1);

static const struct pll_div apll_816_cfg = PLL_DIVISORS(816 * MHz, 1, 2, 1);
static const struct pll_div apll_600_cfg = PLL_DIVISORS(600 * MHz, 1, 3, 1);

static const struct pll_div *apll_cfgs[] = {
	[APLL_816_MHZ] = &apll_816_cfg,
	[APLL_600_MHZ] = &apll_600_cfg,
};

enum {
	/* PLL_CON0 */
	PLL_POSTDIV1_SHIFT		= 12,
	PLL_POSTDIV1_MASK		= 0x7 << PLL_POSTDIV1_SHIFT,
	PLL_FBDIV_SHIFT			= 0,
	PLL_FBDIV_MASK			= 0xfff,

	/* PLL_CON1 */
	PLL_DSMPD_SHIFT			= 12,
	PLL_DSMPD_MASK			= 1 << PLL_DSMPD_SHIFT,
	PLL_INTEGER_MODE		= 1,
	PLL_LOCK_STATUS_SHIFT		= 10,
	PLL_LOCK_STATUS_MASK		= 1 << PLL_LOCK_STATUS_SHIFT,
	PLL_POSTDIV2_SHIFT		= 6,
	PLL_POSTDIV2_MASK		= 0x7 << PLL_POSTDIV2_SHIFT,
	PLL_REFDIV_SHIFT		= 0,
	PLL_REFDIV_MASK			= 0x3f,

	/* PLL_CON2 */
	PLL_FRACDIV_SHIFT		= 0,
	PLL_FRACDIV_MASK		= 0xffffff,

	/* MODE_CON */
	APLL_MODE_SHIFT			= 0,
	NPLL_MODE_SHIFT			= 1,
	DPLL_MODE_SHIFT			= 4,
	CPLL_MODE_SHIFT			= 8,
	GPLL_MODE_SHIFT			= 12,
	PLL_MODE_SLOW			= 0,
	PLL_MODE_NORM,

	/* CLKSEL_CON0 */
	CLK_CORE_PLL_SEL_APLL		= 0,
	CLK_CORE_PLL_SEL_GPLL,
	CLK_CORE_PLL_SEL_DPLL,
	CLK_CORE_PLL_SEL_NPLL,
	CLK_CORE_PLL_SEL_SHIFT		= 6,
	CLK_CORE_PLL_SEL_MASK		= 3 << CLK_CORE_PLL_SEL_SHIFT,
	CLK_CORE_DIV_SHIFT		= 0,
	CLK_CORE_DIV_MASK		= 0x1f,

	/* CLKSEL_CON1 */
	ACLKM_CORE_DIV_SHIFT		= 4,
	ACLKM_CORE_DIV_MASK		= 0x7 << ACLKM_CORE_DIV_SHIFT,
	PCLK_DBG_DIV_SHIFT		= 0,
	PCLK_DBG_DIV_MASK		= 0xF << PCLK_DBG_DIV_SHIFT,

	/* CLKSEL_CON27 */
	GMAC2IO_PLL_SEL_SHIFT		= 7,
	GMAC2IO_PLL_SEL_MASK		= 1 << GMAC2IO_PLL_SEL_SHIFT,
	GMAC2IO_PLL_SEL_CPLL		= 0,
	GMAC2IO_PLL_SEL_GPLL		= 1,
	GMAC2IO_CLK_DIV_MASK		= 0x1f,
	GMAC2IO_CLK_DIV_SHIFT		= 0,

	/* CLKSEL_CON28 */
	ACLK_PERIHP_PLL_SEL_CPLL	= 0,
	ACLK_PERIHP_PLL_SEL_GPLL,
	ACLK_PERIHP_PLL_SEL_HDMIPHY,
	ACLK_PERIHP_PLL_SEL_SHIFT	= 6,
	ACLK_PERIHP_PLL_SEL_MASK	= 3 << ACLK_PERIHP_PLL_SEL_SHIFT,
	ACLK_PERIHP_DIV_CON_SHIFT	= 0,
	ACLK_PERIHP_DIV_CON_MASK	= 0x1f,

	/* CLKSEL_CON29 */
	PCLK_PERIHP_DIV_CON_SHIFT	= 4,
	PCLK_PERIHP_DIV_CON_MASK	= 0x7 << PCLK_PERIHP_DIV_CON_SHIFT,
	HCLK_PERIHP_DIV_CON_SHIFT	= 0,
	HCLK_PERIHP_DIV_CON_MASK	= 3 << HCLK_PERIHP_DIV_CON_SHIFT,

	/* CLKSEL_CON22 */
	CLK_TSADC_DIV_CON_SHIFT		= 0,
	CLK_TSADC_DIV_CON_MASK		= 0x3ff,

	/* CLKSEL_CON23 */
	CLK_SARADC_DIV_CON_SHIFT	= 0,
	CLK_SARADC_DIV_CON_MASK		= GENMASK(9, 0),
	CLK_SARADC_DIV_CON_WIDTH	= 10,

	/* CLKSEL_CON24 */
	CLK_PWM_PLL_SEL_CPLL		= 0,
	CLK_PWM_PLL_SEL_GPLL,
	CLK_PWM_PLL_SEL_SHIFT		= 15,
	CLK_PWM_PLL_SEL_MASK		= 1 << CLK_PWM_PLL_SEL_SHIFT,
	CLK_PWM_DIV_CON_SHIFT		= 8,
	CLK_PWM_DIV_CON_MASK		= 0x7f << CLK_PWM_DIV_CON_SHIFT,

	CLK_SPI_PLL_SEL_CPLL		= 0,
	CLK_SPI_PLL_SEL_GPLL,
	CLK_SPI_PLL_SEL_SHIFT		= 7,
	CLK_SPI_PLL_SEL_MASK		= 1 << CLK_SPI_PLL_SEL_SHIFT,
	CLK_SPI_DIV_CON_SHIFT		= 0,
	CLK_SPI_DIV_CON_MASK		= 0x7f << CLK_SPI_DIV_CON_SHIFT,

	/* CLKSEL_CON30 */
	CLK_SDMMC_PLL_SEL_CPLL		= 0,
	CLK_SDMMC_PLL_SEL_GPLL,
	CLK_SDMMC_PLL_SEL_24M,
	CLK_SDMMC_PLL_SEL_USBPHY,
	CLK_SDMMC_PLL_SHIFT		= 8,
	CLK_SDMMC_PLL_MASK		= 0x3 << CLK_SDMMC_PLL_SHIFT,
	CLK_SDMMC_DIV_CON_SHIFT          = 0,
	CLK_SDMMC_DIV_CON_MASK           = 0xff << CLK_SDMMC_DIV_CON_SHIFT,

	/* CLKSEL_CON32 */
	CLK_EMMC_PLL_SEL_CPLL		= 0,
	CLK_EMMC_PLL_SEL_GPLL,
	CLK_EMMC_PLL_SEL_24M,
	CLK_EMMC_PLL_SEL_USBPHY,
	CLK_EMMC_PLL_SHIFT		= 8,
	CLK_EMMC_PLL_MASK		= 0x3 << CLK_EMMC_PLL_SHIFT,
	CLK_EMMC_DIV_CON_SHIFT          = 0,
	CLK_EMMC_DIV_CON_MASK           = 0xff << CLK_EMMC_DIV_CON_SHIFT,

	/* CLKSEL_CON34 */
	CLK_I2C_PLL_SEL_CPLL		= 0,
	CLK_I2C_PLL_SEL_GPLL,
	CLK_I2C_DIV_CON_MASK		= 0x7f,
	CLK_I2C_PLL_SEL_MASK		= 1,
	CLK_I2C1_PLL_SEL_SHIFT		= 15,
	CLK_I2C1_DIV_CON_SHIFT		= 8,
	CLK_I2C0_PLL_SEL_SHIFT		= 7,
	CLK_I2C0_DIV_CON_SHIFT		= 0,

	/* CLKSEL_CON35 */
	CLK_I2C3_PLL_SEL_SHIFT		= 15,
	CLK_I2C3_DIV_CON_SHIFT		= 8,
	CLK_I2C2_PLL_SEL_SHIFT		= 7,
	CLK_I2C2_DIV_CON_SHIFT		= 0,

	/* CLKSEL_CON40 */
	CLK_HDMIPHY_DIV_CON_SHIFT	= 3,
	CLK_HDMIPHY_DIV_CON_MASK	= 0x7 << CLK_HDMIPHY_DIV_CON_SHIFT,
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
static void rkclk_set_pll(struct rk3328_cru *cru, enum rk_clk_id clk_id,
			const struct pll_div *div)
{
	u32 *pll_con;
	u32 mode_shift, mode_mask;

	pll_con = NULL;
	mode_shift = 0;
	switch (clk_id) {
	case CLK_ARM:
		pll_con = cru->apll_con;
		mode_shift = APLL_MODE_SHIFT;
		break;
	case CLK_DDR:
		pll_con = cru->dpll_con;
		mode_shift = DPLL_MODE_SHIFT;
		break;
	case CLK_CODEC:
		pll_con = cru->cpll_con;
		mode_shift = CPLL_MODE_SHIFT;
		break;
	case CLK_GENERAL:
		pll_con = cru->gpll_con;
		mode_shift = GPLL_MODE_SHIFT;
		break;
	case CLK_NEW:
		pll_con = cru->npll_con;
		mode_shift = NPLL_MODE_SHIFT;
		break;
	default:
		break;
	}
	mode_mask = 1 << mode_shift;

	/* All 8 PLLs have same VCO and output frequency range restrictions. */
	u32 vco_khz = OSC_HZ / 1000 * div->fbdiv / div->refdiv;
	u32 output_khz = vco_khz / div->postdiv1 / div->postdiv2;

	debug("PLL at %p: fbdiv=%d, refdiv=%d, postdiv1=%d, \
	      postdiv2=%d, vco=%u khz, output=%u khz\n",
	      pll_con, div->fbdiv, div->refdiv, div->postdiv1,
	      div->postdiv2, vco_khz, output_khz);
	assert(vco_khz >= VCO_MIN_KHZ && vco_khz <= VCO_MAX_KHZ &&
	       output_khz >= OUTPUT_MIN_KHZ && output_khz <= OUTPUT_MAX_KHZ &&
	       div->fbdiv >= PLL_DIV_MIN && div->fbdiv <= PLL_DIV_MAX);

	/*
	 * When power on or changing PLL setting,
	 * we must force PLL into slow mode to ensure output stable clock.
	 */
	rk_clrsetreg(&cru->mode_con, mode_mask, PLL_MODE_SLOW << mode_shift);

	/* use integer mode */
	rk_clrsetreg(&pll_con[1], PLL_DSMPD_MASK,
		     PLL_INTEGER_MODE << PLL_DSMPD_SHIFT);

	rk_clrsetreg(&pll_con[0],
		     PLL_FBDIV_MASK | PLL_POSTDIV1_MASK,
		     (div->fbdiv << PLL_FBDIV_SHIFT) |
		     (div->postdiv1 << PLL_POSTDIV1_SHIFT));
	rk_clrsetreg(&pll_con[1],
		     PLL_POSTDIV2_MASK | PLL_REFDIV_MASK,
		     (div->postdiv2 << PLL_POSTDIV2_SHIFT) |
		     (div->refdiv << PLL_REFDIV_SHIFT));

	/* waiting for pll lock */
	while (!(readl(&pll_con[1]) & (1 << PLL_LOCK_STATUS_SHIFT)))
		udelay(1);

	/* pll enter normal mode */
	rk_clrsetreg(&cru->mode_con, mode_mask, PLL_MODE_NORM << mode_shift);
}

/*
 * Every CLKSEL composite that can take CPLL, from Linux's clk-rk3328.c tables
 * (COMPOSITE entries with "cpll" -- or "cpll_peri" for aclk_peri_pre -- among
 * their parents): the register, the mux field and CPLL's value in it, the
 * divider field.
 */
struct rk3328_cpll_child {
	u8 con;
	u8 mux_shift, mux_width, cpll_sel;
	u8 div_shift, div_width;
};

static const struct rk3328_cpll_child rk3328_cpll_children[] = {
	{ 38, 14, 2, 0,  0, 14 },	/* clk_rtc32k */
	{ 44,  6, 2, 0,  0,  5 },	/* aclk_gpu_pre */
	/* clk_ddr (CON3) is left out: the DRAM clock is TPL's, never retimed here */
	{  4, 13, 2, 0,  8,  3 },	/* pclk_ddr */
	{  0, 13, 2, 0,  8,  5 },	/* aclk_bus_pre */
	{ 21, 15, 1, 0,  8,  5 },	/* clk_tsp */
	{  6, 15, 1, 0,  0,  7 },	/* clk_i2s0_div */
	{  8, 15, 1, 0,  0,  7 },	/* clk_i2s1_div */
	{ 10, 15, 1, 0,  0,  7 },	/* clk_i2s2_div */
	{ 12, 15, 1, 0,  0,  7 },	/* clk_spdif_div */
	{ 14, 12, 2, 0,  0,  7 },	/* clk_uart0_div */
	{ 16, 12, 2, 0,  0,  7 },	/* clk_uart1_div */
	{ 18, 12, 2, 0,  0,  7 },	/* clk_uart2_div */
	{ 34,  7, 1, 0,  0,  7 },	/* clk_i2c0 */
	{ 34, 15, 1, 0,  8,  7 },	/* clk_i2c1 */
	{ 35,  7, 1, 0,  0,  7 },	/* clk_i2c2 */
	{ 35, 15, 1, 0,  8,  7 },	/* clk_i2c3 */
	{ 20,  7, 1, 0,  0,  5 },	/* clk_crypto */
	{ 24,  7, 1, 0,  0,  7 },	/* clk_spi */
	{ 24, 15, 1, 0,  8,  7 },	/* clk_pwm */
	{  4,  6, 2, 0,  0,  6 },	/* clk_otp */
	{  5, 14, 2, 0,  8,  5 },	/* clk_efuse */
	{ 20, 14, 2, 0,  8,  5 },	/* clk_pdm */
	{ 52,  6, 2, 0,  0,  6 },	/* clk_wifi */
	{ 48,  6, 2, 0,  0,  5 },	/* aclk_rkvdec_pre */
	{ 48, 14, 2, 0,  8,  5 },	/* sclk_vdec_cabac */
	{ 49,  6, 2, 0,  0,  5 },	/* sclk_vdec_core */
	{ 50,  6, 2, 0,  0,  5 },	/* aclk_vpu_pre */
	{ 51,  6, 2, 0,  0,  5 },	/* aclk_rkvenc */
	{ 51, 14, 2, 0,  8,  5 },	/* sclk_venc_core */
	{ 52, 14, 2, 0,  8,  5 },	/* sclk_venc_dsp */
	{ 37,  6, 2, 0,  0,  5 },	/* aclk_vio_pre */
	{ 36, 14, 2, 0,  8,  5 },	/* aclk_rga_pre */
	{ 36,  6, 2, 0,  0,  5 },	/* clk_rga */
	{ 39,  6, 2, 0,  0,  5 },	/* aclk_vop_pre */
	{ 40,  0, 1, 1,  8,  8 },	/* dclk_lcdc_src */
	{ 30,  8, 2, 0,  0,  8 },	/* clk_sdmmc */
	{ 31,  8, 2, 0,  0,  8 },	/* clk_sdio */
	{ 32,  8, 2, 0,  0,  8 },	/* clk_emmc */
	{ 43,  8, 2, 0,  0,  8 },	/* clk_sdmmc_ext */
	{ 45,  7, 1, 0,  0,  7 },	/* clk_ref_usb3otg_src */
	{ 25,  6, 2, 0,  0,  5 },	/* aclk_gmac */
	{ 27,  7, 1, 0,  0,  5 },	/* clk_mac2io_src */
	{ 27, 15, 1, 0,  8,  5 },	/* clk_mac2io_out */
	{ 26,  7, 1, 0,  0,  5 },	/* clk_mac2phy_src */
	{ 28,  6, 2, 0,  0,  5 },	/* aclk_peri_pre (cpll_peri) */
};

/* CPLL's rate now: its output in normal mode, 24 MHz in slow mode */
static ulong rk3328_cpll_rate(struct rk3328_cru *cru)
{
	u32 con0 = readl(&cru->cpll_con[0]), con1 = readl(&cru->cpll_con[1]);
	u32 fbdiv = (con0 & PLL_FBDIV_MASK) >> PLL_FBDIV_SHIFT;
	u32 postdiv1 = (con0 & PLL_POSTDIV1_MASK) >> PLL_POSTDIV1_SHIFT;
	u32 refdiv = (con1 & PLL_REFDIV_MASK) >> PLL_REFDIV_SHIFT;
	u32 postdiv2 = (con1 & PLL_POSTDIV2_MASK) >> PLL_POSTDIV2_SHIFT;

	if (((readl(&cru->mode_con) >> CPLL_MODE_SHIFT) & 1) != PLL_MODE_NORM ||
	    !refdiv || !postdiv1 || !postdiv2)
		return OSC_HZ;

	return (ulong)((u64)OSC_HZ * fbdiv / refdiv / postdiv1 / postdiv2);
}

/*
 * Before CPLL moves to CPLL_HZ, give every child running from it the divider
 * that keeps its rate: the rate it has now, or -- with CPLL still at its reset
 * or slow-mode state, as in TPL -- the rate it had at U-Boot's former
 * CPLL_FORMER_HZ. Linux does the same before it raises CPLL (rk3328.dtsi's
 * assigned-clocks: "CPLL should run at 1200, but that is to high for the
 * initial dividers of most of its children"). A divider that would overflow
 * its field is clamped there.
 */
static void rk3328_cpll_children_keep_rate(struct rk3328_cru *cru)
{
	ulong from = rk3328_cpll_rate(cru);
	int i;

	if (from < 100 * MHz)
		from = CPLL_FORMER_HZ;
	if (from == CPLL_HZ)
		return;

	for (i = 0; i < ARRAY_SIZE(rk3328_cpll_children); i++) {
		const struct rk3328_cpll_child *c = &rk3328_cpll_children[i];
		u32 con = readl(&cru->clksel_con[c->con]);
		u32 div_max = 1U << c->div_width;
		u32 div, keep;

		if (((con >> c->mux_shift) & ((1U << c->mux_width) - 1)) !=
		    c->cpll_sel)
			continue;
		div = ((con >> c->div_shift) & (div_max - 1)) + 1;
		keep = (u32)DIV_ROUND_UP((u64)div * CPLL_HZ, from);
		if (keep > div_max)
			keep = div_max;
		rk_clrsetreg(&cru->clksel_con[c->con],
			     (div_max - 1) << c->div_shift,
			     (keep - 1) << c->div_shift);
	}
}

static void rkclk_init(struct rk3328_cru *cru)
{
	u32 aclk_div;
	u32 hclk_div;
	u32 pclk_div;

	rk3328_configure_cpu(cru, APLL_600_MHZ);

	/* configure gpll cpll */
	rkclk_set_pll(cru, CLK_GENERAL, &gpll_init_cfg);
	rk3328_cpll_children_keep_rate(cru);
	rkclk_set_pll(cru, CLK_CODEC, &cpll_init_cfg);

	/* configure perihp aclk, hclk, pclk */
	aclk_div = GPLL_HZ / PERIHP_ACLK_HZ - 1;
	hclk_div = PERIHP_ACLK_HZ / PERIHP_HCLK_HZ - 1;
	pclk_div = PERIHP_ACLK_HZ / PERIHP_PCLK_HZ - 1;

	rk_clrsetreg(&cru->clksel_con[28],
		     ACLK_PERIHP_PLL_SEL_MASK | ACLK_PERIHP_DIV_CON_MASK,
		     ACLK_PERIHP_PLL_SEL_GPLL << ACLK_PERIHP_PLL_SEL_SHIFT |
		     aclk_div << ACLK_PERIHP_DIV_CON_SHIFT);
	rk_clrsetreg(&cru->clksel_con[29],
		     PCLK_PERIHP_DIV_CON_MASK | HCLK_PERIHP_DIV_CON_MASK,
		     pclk_div << PCLK_PERIHP_DIV_CON_SHIFT |
		     hclk_div << HCLK_PERIHP_DIV_CON_SHIFT);
}

void rk3328_configure_cpu(struct rk3328_cru *cru,
			  enum apll_frequencies apll_freq)
{
	u32 clk_core_div;
	u32 aclkm_div;
	u32 pclk_dbg_div;

	rkclk_set_pll(cru, CLK_ARM, apll_cfgs[apll_freq]);

	clk_core_div = APLL_HZ / CLK_CORE_HZ - 1;
	aclkm_div = APLL_HZ / ACLKM_CORE_HZ / (clk_core_div + 1) - 1;
	pclk_dbg_div = APLL_HZ / PCLK_DBG_HZ / (clk_core_div + 1) - 1;

	rk_clrsetreg(&cru->clksel_con[0],
		     CLK_CORE_PLL_SEL_MASK | CLK_CORE_DIV_MASK,
		     CLK_CORE_PLL_SEL_APLL << CLK_CORE_PLL_SEL_SHIFT |
		     clk_core_div << CLK_CORE_DIV_SHIFT);

	rk_clrsetreg(&cru->clksel_con[1],
		     PCLK_DBG_DIV_MASK | ACLKM_CORE_DIV_MASK,
		     pclk_dbg_div << PCLK_DBG_DIV_SHIFT |
		     aclkm_div << ACLKM_CORE_DIV_SHIFT);
}

/* CPLL or GPLL, by a one-bit mux that reads 0 for CPLL */
static ulong rk3328_cpll_gpll_hz(u32 sel)
{
	return sel ? GPLL_HZ : CPLL_HZ;
}

static ulong rk3328_i2c_get_clk(struct rk3328_cru *cru, ulong clk_id)
{
	u32 div, con, sel;

	switch (clk_id) {
	case SCLK_I2C0:
		con = readl(&cru->clksel_con[34]);
		div = con >> CLK_I2C0_DIV_CON_SHIFT & CLK_I2C_DIV_CON_MASK;
		sel = con >> CLK_I2C0_PLL_SEL_SHIFT & CLK_I2C_PLL_SEL_MASK;
		break;
	case SCLK_I2C1:
		con = readl(&cru->clksel_con[34]);
		div = con >> CLK_I2C1_DIV_CON_SHIFT & CLK_I2C_DIV_CON_MASK;
		sel = con >> CLK_I2C1_PLL_SEL_SHIFT & CLK_I2C_PLL_SEL_MASK;
		break;
	case SCLK_I2C2:
		con = readl(&cru->clksel_con[35]);
		div = con >> CLK_I2C2_DIV_CON_SHIFT & CLK_I2C_DIV_CON_MASK;
		sel = con >> CLK_I2C2_PLL_SEL_SHIFT & CLK_I2C_PLL_SEL_MASK;
		break;
	case SCLK_I2C3:
		con = readl(&cru->clksel_con[35]);
		div = con >> CLK_I2C3_DIV_CON_SHIFT & CLK_I2C_DIV_CON_MASK;
		sel = con >> CLK_I2C3_PLL_SEL_SHIFT & CLK_I2C_PLL_SEL_MASK;
		break;
	default:
		printf("do not support this i2c bus\n");
		return -EINVAL;
	}

	/* the reset mux is CPLL, and nothing moves it unless the rate is set */
	return DIV_TO_RATE(rk3328_cpll_gpll_hz(sel), div);
}

static ulong rk3328_i2c_set_clk(struct rk3328_cru *cru, ulong clk_id, uint hz)
{
	int src_clk_div;

	src_clk_div = GPLL_HZ / hz;
	assert(src_clk_div - 1 < 127);

	switch (clk_id) {
	case SCLK_I2C0:
		rk_clrsetreg(&cru->clksel_con[34],
			     CLK_I2C_DIV_CON_MASK << CLK_I2C0_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_MASK << CLK_I2C0_PLL_SEL_SHIFT,
			     (src_clk_div - 1) << CLK_I2C0_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_GPLL << CLK_I2C0_PLL_SEL_SHIFT);
		break;
	case SCLK_I2C1:
		rk_clrsetreg(&cru->clksel_con[34],
			     CLK_I2C_DIV_CON_MASK << CLK_I2C1_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_MASK << CLK_I2C1_PLL_SEL_SHIFT,
			     (src_clk_div - 1) << CLK_I2C1_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_GPLL << CLK_I2C1_PLL_SEL_SHIFT);
		break;
	case SCLK_I2C2:
		rk_clrsetreg(&cru->clksel_con[35],
			     CLK_I2C_DIV_CON_MASK << CLK_I2C2_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_MASK << CLK_I2C2_PLL_SEL_SHIFT,
			     (src_clk_div - 1) << CLK_I2C2_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_GPLL << CLK_I2C2_PLL_SEL_SHIFT);
		break;
	case SCLK_I2C3:
		rk_clrsetreg(&cru->clksel_con[35],
			     CLK_I2C_DIV_CON_MASK << CLK_I2C3_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_MASK << CLK_I2C3_PLL_SEL_SHIFT,
			     (src_clk_div - 1) << CLK_I2C3_DIV_CON_SHIFT |
			     CLK_I2C_PLL_SEL_GPLL << CLK_I2C3_PLL_SEL_SHIFT);
		break;
	default:
		printf("do not support this i2c bus\n");
		return -EINVAL;
	}

	return DIV_TO_RATE(GPLL_HZ, src_clk_div);
}

static ulong rk3328_gmac2io_set_clk(struct rk3328_cru *cru, ulong rate)
{
	struct rk3328_grf_regs *grf;
	ulong ret;

	grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);

	/*
	 * The RGMII CLK can be derived either from an external "clkin"
	 * or can be generated from internally by a divider from SCLK_MAC.
	 */
	if (readl(&grf->mac_con[1]) & BIT(10) &&
	    readl(&grf->soc_con[4]) & BIT(14)) {
		/* An external clock will always generate the right rate... */
		ret = rate;
	} else {
		u32 con = readl(&cru->clksel_con[27]);
		ulong pll_rate;
		u8 div;

		if ((con >> GMAC2IO_PLL_SEL_SHIFT) & GMAC2IO_PLL_SEL_GPLL)
			pll_rate = GPLL_HZ;
		else
			pll_rate = CPLL_HZ;

		div = DIV_ROUND_UP(pll_rate, rate) - 1;
		if (div <= 0x1f)
			rk_clrsetreg(&cru->clksel_con[27], GMAC2IO_CLK_DIV_MASK,
				     div << GMAC2IO_CLK_DIV_SHIFT);
		else
			debug("Unsupported div for gmac:%d\n", div);

		return DIV_TO_RATE(pll_rate, div);
	}

	return ret;
}

static ulong rk3328_mmc_get_clk(struct rk3328_cru *cru, uint clk_id)
{
	u32 div, con, con_id;

	switch (clk_id) {
	case HCLK_SDMMC:
	case SCLK_SDMMC:
		con_id = 30;
		break;
	case HCLK_EMMC:
	case SCLK_EMMC:
		con_id = 32;
		break;
	default:
		return -EINVAL;
	}
	con = readl(&cru->clksel_con[con_id]);
	div = (con & CLK_EMMC_DIV_CON_MASK) >> CLK_EMMC_DIV_CON_SHIFT;

	/* the card clock: the selected source over the divider and the controller's 2 */
	switch ((con & CLK_EMMC_PLL_MASK) >> CLK_EMMC_PLL_SHIFT) {
	case CLK_EMMC_PLL_SEL_CPLL:
		return DIV_TO_RATE(CPLL_HZ, div) / 2;
	case CLK_EMMC_PLL_SEL_GPLL:
		return DIV_TO_RATE(GPLL_HZ, div) / 2;
	case CLK_EMMC_PLL_SEL_24M:
		return DIV_TO_RATE(OSC_HZ, div) / 2;
	default:
		return DIV_TO_RATE(480 * MHz, div) / 2;
	}
}

static ulong rk3328_mmc_set_clk(struct rk3328_cru *cru,
				ulong clk_id, ulong set_rate)
{
	int src_clk_div;
	u32 con_id;

	switch (clk_id) {
	case HCLK_SDMMC:
	case SCLK_SDMMC:
		con_id = 30;
		break;
	case HCLK_EMMC:
	case SCLK_EMMC:
		con_id = 32;
		break;
	default:
		return -EINVAL;
	}
	/* mmc clock defaulg div 2 internal, need provide double in cru */
	src_clk_div = DIV_ROUND_UP(GPLL_HZ / 2, set_rate);

	if (src_clk_div > 127) {
		/* use 24MHz source for 400KHz clock */
		src_clk_div = DIV_ROUND_UP(OSC_HZ / 2, set_rate);
		rk_clrsetreg(&cru->clksel_con[con_id],
			     CLK_EMMC_PLL_MASK | CLK_EMMC_DIV_CON_MASK,
			     CLK_EMMC_PLL_SEL_24M << CLK_EMMC_PLL_SHIFT |
			     (src_clk_div - 1) << CLK_EMMC_DIV_CON_SHIFT);
	} else {
		/*
		 * GPLL or CPLL, whichever lands closer at or below the request:
		 * the tuned-mode ladder asks for 200 / 150 / 100 / 50 MHz, and
		 * GPLL alone (576 MHz) gives 144 / 144 / 96 / 48 where CPLL
		 * (1200 MHz) gives them exactly.
		 */
		int cpll_div = DIV_ROUND_UP(CPLL_HZ / 2, set_rate);
		uint sel = CLK_EMMC_PLL_SEL_GPLL;

		if (cpll_div <= 127 &&
		    CPLL_HZ / 2 / cpll_div > GPLL_HZ / 2 / src_clk_div) {
			sel = CLK_EMMC_PLL_SEL_CPLL;
			src_clk_div = cpll_div;
		}
		rk_clrsetreg(&cru->clksel_con[con_id],
			     CLK_EMMC_PLL_MASK | CLK_EMMC_DIV_CON_MASK,
			     sel << CLK_EMMC_PLL_SHIFT |
			     (src_clk_div - 1) << CLK_EMMC_DIV_CON_SHIFT);
	}

	return rk3328_mmc_get_clk(cru, clk_id);
}

static ulong rk3328_pwm_get_clk(struct rk3328_cru *cru)
{
	u32 div, con;

	con = readl(&cru->clksel_con[24]);
	div = (con & CLK_PWM_DIV_CON_MASK) >> CLK_PWM_DIV_CON_SHIFT;

	return DIV_TO_RATE(rk3328_cpll_gpll_hz((con & CLK_PWM_PLL_SEL_MASK) >>
					       CLK_PWM_PLL_SEL_SHIFT), div);
}

static ulong rk3328_pwm_set_clk(struct rk3328_cru *cru, uint hz)
{
	u32 div = GPLL_HZ / hz;

	rk_clrsetreg(&cru->clksel_con[24],
		     CLK_PWM_PLL_SEL_MASK | CLK_PWM_DIV_CON_MASK,
		     CLK_PWM_PLL_SEL_GPLL << CLK_PWM_PLL_SEL_SHIFT |
		     (div - 1) << CLK_PWM_DIV_CON_SHIFT);

	return DIV_TO_RATE(GPLL_HZ, div);
}

static ulong rk3328_saradc_get_clk(struct rk3328_cru *cru)
{
	u32 div, val;

	val = readl(&cru->clksel_con[23]);
	div = bitfield_extract(val, CLK_SARADC_DIV_CON_SHIFT,
			       CLK_SARADC_DIV_CON_WIDTH);

	return DIV_TO_RATE(OSC_HZ, div);
}

static ulong rk3328_saradc_set_clk(struct rk3328_cru *cru, uint hz)
{
	int src_clk_div;

	src_clk_div = DIV_ROUND_UP(OSC_HZ, hz) - 1;
	assert(src_clk_div < 128);

	rk_clrsetreg(&cru->clksel_con[23],
		     CLK_SARADC_DIV_CON_MASK,
		     src_clk_div << CLK_SARADC_DIV_CON_SHIFT);

	return rk3328_saradc_get_clk(cru);
}

static ulong rk3328_spi_get_clk(struct rk3328_cru *cru)
{
	u32 div, val;

	val = readl(&cru->clksel_con[24]);
	div = (val & CLK_SPI_DIV_CON_MASK) >> CLK_SPI_DIV_CON_SHIFT;

	return DIV_TO_RATE(OSC_HZ, div);
}

static ulong rk3328_spi_set_clk(struct rk3328_cru *cru, uint hz)
{
	u32 src_clk_div;

	src_clk_div = GPLL_HZ / hz;
	assert(src_clk_div < 128);

	rk_clrsetreg(&cru->clksel_con[24],
		     CLK_PWM_PLL_SEL_MASK | CLK_PWM_DIV_CON_MASK,
		     CLK_PWM_PLL_SEL_GPLL << CLK_PWM_PLL_SEL_SHIFT |
		     (src_clk_div - 1) << CLK_PWM_DIV_CON_SHIFT);

	return rk3328_spi_get_clk(cru);
}

#ifndef CONFIG_XPL_BUILD
static ulong rk3328_vop_get_clk(struct rk3328_clk_priv *priv, ulong clk_id)
{
	struct rk3328_cru *cru = priv->cru;
	u32 div, con, parent;

	switch (clk_id) {
	case ACLK_VOP_PRE:
		con = readl(&cru->clksel_con[39]);
		div = (con & ACLK_VOP_DIV_CON_MASK) >> ACLK_VOP_DIV_CON_SHIFT;
		con = (con & ACLK_VOP_PLL_SEL_MASK) >> ACLK_VOP_PLL_SEL_SHIFT;
		parent = con == ACLK_VOP_PLL_SEL_CPLL ? CPLL_HZ :
			 con == ACLK_VOP_PLL_SEL_USB480M ? 480 * MHz : GPLL_HZ;
		break;
	case ACLK_VIO_PRE:
		con = readl(&cru->clksel_con[37]);
		div = (con & ACLK_VIO_DIV_CON_MASK) >> ACLK_VIO_DIV_CON_SHIFT;
		con = (con & ACLK_VIO_PLL_SEL_MASK) >> ACLK_VIO_PLL_SEL_SHIFT;
		parent = con == ACLK_VIO_PLL_SEL_CPLL ? CPLL_HZ :
			 con == ACLK_VIO_PLL_SEL_USB480M ? 480 * MHz : GPLL_HZ;
		break;
	case DCLK_LCDC:
		con = readl(&cru->clksel_con[40]);
		div = (con & DCLK_LCDC_DIV_CON_MASK) >> DCLK_LCDC_DIV_CON_SHIFT;
		parent = ((con & DCLK_LCDC_PLL_SEL_MASK) >> DCLK_LCDC_PLL_SEL_SHIFT) ==
			 DCLK_LCDC_PLL_SEL_CPLL ? CPLL_HZ : GPLL_HZ;
		break;
	default:
		printf("%s: Unsupported vop get clk#%ld\n", __func__, clk_id);
		return -ENOENT;
	}

	return DIV_TO_RATE(parent, div);
}

static ulong rk3328_vop_set_clk(struct rk3328_clk_priv *priv,
				ulong clk_id, uint hz)
{
	struct rk3328_cru *cru = priv->cru;
	int src_clk_div;
	u32 con, parent;

	/* ACLK_VOP_PRE and ACLK_VIO_PRE are put on CPLL: size from it */
	src_clk_div = DIV_ROUND_UP(CPLL_HZ, hz);
	assert(src_clk_div - 1 < 31);

	switch (clk_id) {
	case ACLK_VOP_PRE:
		rk_clrsetreg(&cru->clksel_con[39],
			     ACLK_VOP_PLL_SEL_MASK | ACLK_VOP_DIV_CON_MASK,
			     ACLK_VOP_PLL_SEL_CPLL << ACLK_VOP_PLL_SEL_SHIFT |
			     (src_clk_div - 1) << ACLK_VOP_DIV_CON_SHIFT);
		break;
	case ACLK_VIO_PRE:
		rk_clrsetreg(&cru->clksel_con[37],
			     ACLK_VIO_PLL_SEL_MASK | ACLK_VIO_DIV_CON_MASK,
			     ACLK_VIO_PLL_SEL_CPLL << ACLK_VIO_PLL_SEL_SHIFT |
			     (src_clk_div - 1) << ACLK_VIO_DIV_CON_SHIFT);
		break;
	case DCLK_LCDC:
		con = readl(&cru->clksel_con[40]);
		con = (con & DCLK_LCDC_SEL_MASK) >> DCLK_LCDC_SEL_SHIFT;
		if (con) {
			parent = readl(&cru->clksel_con[40]);
			parent = (parent & DCLK_LCDC_PLL_SEL_MASK) >>
				 DCLK_LCDC_PLL_SEL_SHIFT;
			if (parent == DCLK_LCDC_PLL_SEL_CPLL)
				src_clk_div = DIV_ROUND_UP(CPLL_HZ, hz);
			else
				src_clk_div = DIV_ROUND_UP(GPLL_HZ, hz);

			rk_clrsetreg(&cru->clksel_con[40],
				     DCLK_LCDC_DIV_CON_MASK,
				     (src_clk_div - 1) <<
				     DCLK_LCDC_DIV_CON_SHIFT);
		}
		break;
	default:
		printf("%s: Unable to set vop clk#%ld\n", __func__, clk_id);
		return -EINVAL;
	}

	return rk3328_vop_get_clk(priv, clk_id);
}
#endif

static ulong rk3328_hdmiphy_get_clk(struct rk3328_cru *cru)
{
	u32 div, con;

	con = readl(&cru->clksel_con[40]);
	div = (con & CLK_HDMIPHY_DIV_CON_MASK) >> CLK_HDMIPHY_DIV_CON_SHIFT;

	return DIV_TO_RATE(GPLL_HZ, div);
}

/*
 * pclk_bus, the parent of pclk_wdt (Linux clk-rk3328.c:
 * SGRF_GATE(PCLK_WDT, "pclk_wdt", "pclk_bus")), read from the registers as
 * they are: aclk_bus_pre = CPLL or GPLL (CLKSEL_CON0[14:13]) /
 * (CLKSEL_CON0[12:8] + 1), pclk_bus = aclk_bus_pre / (CLKSEL_CON1[14:12] + 1)
 * (RK3328 TRM part 1, CRU_CLKSEL_CON0 / CON1). U-Boot never sets these; TPL
 * and the CPLL move (rk3328_cpll_children_keep_rate) leave them.
 */
static ulong rk3328_pclk_bus_get_clk(struct rk3328_cru *cru)
{
	u32 con0 = readl(&cru->clksel_con[0]);
	u32 con1 = readl(&cru->clksel_con[1]);
	u32 sel = (con0 >> 13) & 0x3;
	u32 adiv = (con0 >> 8) & 0x1f;
	u32 pdiv = (con1 >> 12) & 0x7;
	ulong src;

	if (sel == 0)
		src = rk3328_cpll_rate(cru);
	else if (sel == 1)
		src = GPLL_HZ;
	else
		return 0;	/* the HDMI PHY PLL: not a bus parent we set */

	return src / (adiv + 1) / (pdiv + 1);
}

static ulong rk3328_clk_get_rate(struct clk *clk)
{
	struct rk3328_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate = 0;

	switch (clk->id) {
	case 0 ... 29:
		return 0;
	case HCLK_SDMMC:
	case HCLK_EMMC:
	case SCLK_SDMMC:
	case SCLK_EMMC:
		rate = rk3328_mmc_get_clk(priv->cru, clk->id);
		break;
	case SCLK_SDMMC_DRV:
	case SCLK_SDMMC_SAMPLE:
		rate = rk3328_mmc_get_clk(priv->cru, SCLK_SDMMC);
		break;
	case SCLK_EMMC_DRV:
	case SCLK_EMMC_SAMPLE:
		rate = rk3328_mmc_get_clk(priv->cru, SCLK_EMMC);
		break;
	case SCLK_I2C0:
	case SCLK_I2C1:
	case SCLK_I2C2:
	case SCLK_I2C3:
		rate = rk3328_i2c_get_clk(priv->cru, clk->id);
		break;
	case SCLK_PWM:
		rate = rk3328_pwm_get_clk(priv->cru);
		break;
	case SCLK_SARADC:
		rate = rk3328_saradc_get_clk(priv->cru);
		break;
	case SCLK_SPI:
		rate = rk3328_spi_get_clk(priv->cru);
		break;
	case PCLK_HDMIPHY:
		rate = rk3328_hdmiphy_get_clk(priv->cru);
		break;
	case SCLK_USB3OTG_REF:
		rate = OSC_HZ;
		break;
	/*
	 * pclk_wdt's gate is in the secure GRF (Linux: SGRF_GATE), on out of
	 * reset (rk3328_clk_enable() accepts it). Without a rate here
	 * designware_wdt_probe() fails, initr_watchdog() drops the device with
	 * a debug print, and no watchdog is started or stopped on RK3328
	 * (WATCHDOG_AUTOSTART notwithstanding) -- as on RK3399 before its case.
	 */
	case PCLK_WDT:
		rate = rk3328_pclk_bus_get_clk(priv->cru);
		break;
	default:
		return -ENOENT;
	}

	return rate;
}

static ulong rk3328_clk_set_rate(struct clk *clk, ulong rate)
{
	struct rk3328_clk_priv *priv = dev_get_priv(clk->dev);
	ulong ret = 0;

	switch (clk->id) {
	case 0 ... 29:
		return 0;
	case HCLK_SDMMC:
	case HCLK_EMMC:
	case SCLK_SDMMC:
	case SCLK_EMMC:
		ret = rk3328_mmc_set_clk(priv->cru, clk->id, rate);
		break;
	case SCLK_I2C0:
	case SCLK_I2C1:
	case SCLK_I2C2:
	case SCLK_I2C3:
		ret = rk3328_i2c_set_clk(priv->cru, clk->id, rate);
		break;
	case SCLK_MAC2IO:
		ret = rk3328_gmac2io_set_clk(priv->cru, rate);
		break;
	case SCLK_PWM:
		ret = rk3328_pwm_set_clk(priv->cru, rate);
		break;
	case SCLK_SARADC:
		ret = rk3328_saradc_set_clk(priv->cru, rate);
		break;
	case SCLK_SPI:
		ret = rk3328_spi_set_clk(priv->cru, rate);
		break;
#ifndef CONFIG_XPL_BUILD
	case DCLK_LCDC:
	case ACLK_VOP_PRE:
	case ACLK_VIO_PRE:
		rate = rk3328_vop_set_clk(priv, clk->id, rate);
		break;
#endif
	case SCLK_PDM:
	case SCLK_RTC32K:
	case SCLK_UART0:
	case SCLK_UART1:
	case SCLK_UART2:
	case SCLK_SDIO:
	case SCLK_TSP:
	case SCLK_WIFI:
	case ACLK_BUS_PRE:
	case HCLK_BUS_PRE:
	case PCLK_BUS_PRE:
	case ACLK_PERI_PRE:
	case HCLK_PERI:
	case PCLK_PERI:
	case HCLK_VIO_PRE:
	case ACLK_RGA_PRE:
	case SCLK_RGA:
	case ACLK_RKVDEC_PRE:
	case ACLK_RKVENC:
	case ACLK_VPU_PRE:
	case SCLK_VDEC_CABAC:
	case SCLK_VDEC_CORE:
	case SCLK_VENC_CORE:
	case SCLK_VENC_DSP:
	case SCLK_EFUSE:
	case PCLK_DDR:
	case ACLK_GMAC:
	case PCLK_GMAC:
	case SCLK_USB3OTG_REF:
	case SCLK_USB3OTG_SUSPEND:
	case USB480M:
		return 0;
	default:
		return -ENOENT;
	}

	return ret;
}

static int rk3328_gmac2io_set_parent(struct clk *clk, struct clk *parent)
{
	struct rk3328_grf_regs *grf;
	const char *clock_output_name;
	int ret;

	grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);

	/*
	 * If the requested parent is in the same clock-controller and the id
	 * is SCLK_MAC2IO_SRC ("clk_mac2io_src"), switch to the internal clock.
	 */
	if ((parent->dev == clk->dev) && (parent->id == SCLK_MAC2IO_SRC)) {
		debug("%s: switching RGMII to SCLK_MAC2IO_SRC\n", __func__);
		rk_clrreg(&grf->mac_con[1], BIT(10));
		return 0;
	}

	/*
	 * Otherwise, we need to check the clock-output-names of the
	 * requested parent to see if the requested id is "gmac_clkin".
	 */
	ret = dev_read_string_index(parent->dev, "clock-output-names",
				    parent->id, &clock_output_name);
	if (ret < 0)
		return -ENODATA;

	/* If this is "gmac_clkin", switch to the external clock input */
	if (!strcmp(clock_output_name, "gmac_clkin")) {
		debug("%s: switching RGMII to CLKIN\n", __func__);
		rk_setreg(&grf->mac_con[1], BIT(10));
		return 0;
	}

	return -EINVAL;
}

static int rk3328_gmac2io_ext_set_parent(struct clk *clk, struct clk *parent)
{
	struct rk3328_grf_regs *grf;
	const char *clock_output_name;
	int ret;

	grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);

	/*
	 * If the requested parent is in the same clock-controller and the id
	 * is SCLK_MAC2IO ("clk_mac2io"), switch to the internal clock.
	 */
	if ((parent->dev == clk->dev) && (parent->id == SCLK_MAC2IO)) {
		debug("%s: switching RGMII to SCLK_MAC2IO\n", __func__);
		rk_clrreg(&grf->soc_con[4], BIT(14));
		return 0;
	}

	/*
	 * Otherwise, we need to check the clock-output-names of the
	 * requested parent to see if the requested id is "gmac_clkin".
	 */
	ret = dev_read_string_index(parent->dev, "clock-output-names",
				    parent->id, &clock_output_name);
	if (ret < 0)
		return -ENODATA;

	/* If this is "gmac_clkin", switch to the external clock input */
	if (!strcmp(clock_output_name, "gmac_clkin")) {
		debug("%s: switching RGMII to CLKIN\n", __func__);
		rk_setreg(&grf->soc_con[4], BIT(14));
		return 0;
	}

	return -EINVAL;
}

static int rk3328_clk_set_parent(struct clk *clk, struct clk *parent)
{
	switch (clk->id) {
	case SCLK_MAC2IO:
		return rk3328_gmac2io_set_parent(clk, parent);
	case SCLK_MAC2IO_EXT:
		return rk3328_gmac2io_ext_set_parent(clk, parent);
	case DCLK_LCDC:
	case USB480M:
	case SCLK_PDM:
	case SCLK_RTC32K:
	case SCLK_UART0:
	case SCLK_UART1:
	case SCLK_UART2:
		return 0;
	}

	debug("%s: unsupported clk %ld\n", __func__, clk->id);
	return -ENOENT;
}

/*
 * MMC drive / sample phase clocks (SDMMC_CON0/1, EMMC_CON0/1), the delay
 * line Linux drives through clk-mmc-phase.c: a 90-degree coarse select plus
 * up to 255 fine delay elements of ~60 ps each, both relative to the card
 * clock (the CRU source divided by the controller's fixed 2). The same math
 * as clk_rk3399.c.
 */
#define RK3328_MMC_PHASE_SHIFT		1
#define RK3328_MMC_PHASE_MASK		0x07ff
#define RK3328_MMC_DELAY_SEL		BIT(10)
#define RK3328_MMC_DEGREE_MASK		0x3
#define RK3328_MMC_DELAYNUM_OFFSET	2
#define RK3328_MMC_DELAYNUM_MASK	(0xff << RK3328_MMC_DELAYNUM_OFFSET)
#define RK3328_MMC_DELAY_ELEMENT_PSEC	60

static u32 *rk3328_mmc_phase_reg(struct rk3328_cru *cru, ulong clk_id,
				 ulong *rate_id)
{
	switch (clk_id) {
	case SCLK_SDMMC_DRV:
		*rate_id = SCLK_SDMMC;
		return &cru->sdmmc_con[0];
	case SCLK_SDMMC_SAMPLE:
		*rate_id = SCLK_SDMMC;
		return &cru->sdmmc_con[1];
	case SCLK_EMMC_DRV:
		*rate_id = SCLK_EMMC;
		return &cru->emmc_con[0];
	case SCLK_EMMC_SAMPLE:
		*rate_id = SCLK_EMMC;
		return &cru->emmc_con[1];
	default:
		return NULL;
	}
}

static int rk3328_clk_get_phase(struct clk *clk)
{
	struct rk3328_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate_id, rate;
	u32 raw, delay_num;
	u32 *reg;
	int degrees;

	reg = rk3328_mmc_phase_reg(priv->cru, clk->id, &rate_id);
	if (!reg)
		return -ENOSYS;

	rate = rk3328_mmc_get_clk(priv->cru, rate_id);
	if (!rate || IS_ERR_VALUE(rate))
		return 0;

	raw = readl(reg) >> RK3328_MMC_PHASE_SHIFT;
	degrees = (raw & RK3328_MMC_DEGREE_MASK) * 90;

	if (raw & RK3328_MMC_DELAY_SEL) {
		/* degrees per delay element, scaled by 1000000 */
		ulong factor = (RK3328_MMC_DELAY_ELEMENT_PSEC / 10) *
			       36 * (rate / 10000);

		delay_num = (raw & RK3328_MMC_DELAYNUM_MASK) >>
			    RK3328_MMC_DELAYNUM_OFFSET;
		degrees += DIV_ROUND_CLOSEST(delay_num * factor, 1000000);
	}

	return degrees % 360;
}

static int rk3328_clk_set_phase(struct clk *clk, int degrees)
{
	struct rk3328_clk_priv *priv = dev_get_priv(clk->dev);
	ulong rate_id, rate;
	u32 nineties, remainder, delay, delay_num, raw;
	u32 *reg;

	reg = rk3328_mmc_phase_reg(priv->cru, clk->id, &rate_id);
	if (!reg)
		return -ENOSYS;

	if (degrees < 0 || degrees >= 360)
		return -EINVAL;

	rate = rk3328_mmc_get_clk(priv->cru, rate_id);
	if (!rate || IS_ERR_VALUE(rate))
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
				  (RK3328_MMC_DELAY_ELEMENT_PSEC / 10));
	delay_num = min_t(u32, delay, 255);

	raw = delay_num ? RK3328_MMC_DELAY_SEL : 0;
	raw |= delay_num << RK3328_MMC_DELAYNUM_OFFSET;
	raw |= nineties;

	rk_clrsetreg(reg, RK3328_MMC_PHASE_MASK << RK3328_MMC_PHASE_SHIFT,
		     raw << RK3328_MMC_PHASE_SHIFT);

	debug("%s: clk %ld set_phase(%d) delay_num=%u reg=0x%08x actual=%d\n",
	      __func__, clk->id, degrees, delay_num, readl(reg),
	      rk3328_clk_get_phase(clk));

	return 0;
}

/*
 * Only pclk_wdt: its gate is in the secure GRF and on out of reset, but
 * clk_enable() without an enable op is -ENOSYS, which designware_wdt_probe()
 * treats as fatal. Every other id keeps the -ENOSYS it had before this op
 * existed, so the drivers that tolerate -ENOSYS see no change.
 */
static int rk3328_clk_enable(struct clk *clk)
{
	switch (clk->id) {
	case PCLK_WDT:
		return 0;
	default:
		return -ENOSYS;
	}
}

static struct clk_ops rk3328_clk_ops = {
	.enable = rk3328_clk_enable,
	.get_rate = rk3328_clk_get_rate,
	.set_rate = rk3328_clk_set_rate,
	.set_parent = rk3328_clk_set_parent,
	.get_phase = rk3328_clk_get_phase,
	.set_phase = rk3328_clk_set_phase,
};

static int rk3328_clk_probe(struct udevice *dev)
{
	struct rk3328_clk_priv *priv = dev_get_priv(dev);

	rkclk_init(priv->cru);

	return 0;
}

static int rk3328_clk_of_to_plat(struct udevice *dev)
{
	struct rk3328_clk_priv *priv = dev_get_priv(dev);

	priv->cru = dev_read_addr_ptr(dev);

	return 0;
}

static int rk3328_clk_bind(struct udevice *dev)
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
		priv->glb_srst_fst_value = offsetof(struct rk3328_cru,
						    glb_srst_fst_value);
		priv->glb_srst_snd_value = offsetof(struct rk3328_cru,
						    glb_srst_snd_value);
		dev_set_priv(sys_child, priv);
	}

#if CONFIG_IS_ENABLED(RESET_ROCKCHIP)
	ret = offsetof(struct rk3328_cru, softrst_con[0]);
	ret = rockchip_reset_bind(dev, ret, 12);
	if (ret)
		debug("Warning: software reset driver bind failed\n");
#endif

	return ret;
}

static const struct udevice_id rk3328_clk_ids[] = {
	{ .compatible = "rockchip,rk3328-cru" },
	{ }
};

U_BOOT_DRIVER(rockchip_rk3328_cru) = {
	.name		= "rockchip_rk3328_cru",
	.id		= UCLASS_CLK,
	.of_match	= rk3328_clk_ids,
	.priv_auto	= sizeof(struct rk3328_clk_priv),
	.of_to_plat = rk3328_clk_of_to_plat,
	.ops		= &rk3328_clk_ops,
	.bind		= rk3328_clk_bind,
	.probe		= rk3328_clk_probe,
};
