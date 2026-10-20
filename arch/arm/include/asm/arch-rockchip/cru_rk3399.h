/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 */

#ifndef __ASM_ARCH_CRU_RK3399_H_
#define __ASM_ARCH_CRU_RK3399_H_

/* Private data for the clock driver - used by rockchip_get_cru() */
struct rk3399_clk_priv {
	struct rockchip_cru *cru;
};

struct rk3399_pmuclk_priv {
	struct rk3399_pmucru *pmucru;
};

struct rk3399_pmucru {
	u32 ppll_con[6];
	u32 reserved[0x1a];
	u32 pmucru_clksel[6];
	u32 pmucru_clkfrac_con[2];
	u32 reserved2[0x18];
	u32 pmucru_clkgate_con[3];
	u32 reserved3;
	u32 pmucru_softrst_con[2];
	u32 reserved4[2];
	u32 pmucru_rstnhold_con[2];
	u32 reserved5[2];
	u32 pmucru_gatedis_con[2];
};
check_member(rk3399_pmucru, pmucru_gatedis_con[1], 0x134);

struct rockchip_cru {
	u32 apll_l_con[6];
	u32 reserved[2];
	u32 apll_b_con[6];
	u32 reserved1[2];
	u32 dpll_con[6];
	u32 reserved2[2];
	u32 cpll_con[6];
	u32 reserved3[2];
	u32 gpll_con[6];
	u32 reserved4[2];
	u32 npll_con[6];
	u32 reserved5[2];
	u32 vpll_con[6];
	u32 reserved6[0x0a];
	u32 clksel_con[108];
	u32 reserved7[0x14];
	u32 clkgate_con[35];
	u32 reserved8[0x1d];
	u32 softrst_con[21];
	u32 reserved9[0x2b];
	u32 glb_srst_fst_value;
	u32 glb_srst_snd_value;
	u32 glb_cnt_th;
	u32 misc_con;
	u32 glb_rst_con;
	u32 glb_rst_st;
	u32 reserved10[0x1a];
	u32 sdmmc_con[2];
	u32 sdio0_con[2];
	u32 sdio1_con[2];
};
check_member(rockchip_cru, sdio1_con[1], 0x594);
#define KHz		1000
#define OSC_HZ		(24*MHz)
#define LPLL_HZ		(600*MHz)
#define BPLL_HZ		(600*MHz)
#define GPLL_HZ		(594*MHz)
#define CPLL_HZ		(384*MHz)
/*
 * NPLL is not brought up by rkclk_init(), which programs only GPLL and CPLL.
 * It powers on in slow mode -- clocked from the 24 MHz oscillator -- with its
 * reset dividers encoding 1000 MHz (NPLL_CON0 fbdiv 250, NPLL_CON1 refdiv 3 /
 * postdiv1 2 / postdiv2 1), and rk3399_npll_ensure() brings it to THIS rate
 * on demand, from whichever consumer first needs it.
 *
 * 1200 MHz rather than the reset 1000 because NPLL is the one PLL in the
 * clk_sdmmc / clk_sdio parent set that divides to every SD card-clock band
 * exactly: the dw-mshc halves the CRU output, so a 200 / 150 / 100 / 50 MHz
 * card clock needs 400 / 300 / 200 / 100 MHz at the CRU, which are 1200 / 3,
 * 4, 6, 12. GPLL (594) gives 148.5 / 99 / 49.5, CPLL (384) 192 / 96 / 48,
 * and 1000 divides to none of them (1000 / 5 = 200 at the CRU is a 100 MHz
 * card clock). The other NPLL consumer, dp_core, stays exact: 1200 / 12 =
 * 100 MHz. Linux re-assigns NPLL to its own 1000 at boot (rk3399-base.dtsi
 * assigned-clock-rates) and takes the card clock from CPLL 800, so nothing
 * on the Linux side reads this value.
 *
 * What else is on NPLL, read from the CRU at the U-Boot prompt of a
 * ROC-RK3399-PC with this value (every CLKSEL mux that lists NPLL as a
 * parent, decoded against the Linux clk-rk3399.c branch table): the four
 * muxes whose TRM RESET value selects NPLL -- aclk_gpu (CLKSEL_CON13
 * 0x8361, /2), clk_pcie_core (CON18 0x4a87, /8), clk_gmac (CON20 0x9303,
 * /20) and clk_isp0/1 (CON55 0x8181, /2) -- plus clk_pciephy_ref100m
 * (CON18, /10, and CON18[10] = 0 selects the 24 MHz reference instead)
 * and dp_core (CON46 0x0b, /12). U-Boot writes none of the first four:
 * they have been riding NPLL from the oscillator (24 / n) to 1000 / n at
 * the moment the DP walk brought the PLL up, and now ride it to 1200 / n.
 * Their IPs have no U-Boot driver activity (GPU, ISP), pcie0 is disabled,
 * and the GMAC's RGMII clock is the external clkin (CLKSEL_CON19[4] = 1),
 * so no rate a driver depends on moves; Linux re-parents and re-rates all
 * of them at boot.
 */
#define NPLL_HZ		(1200*MHz)
#define PPLL_HZ		(676*MHz)

#define PMU_PCLK_HZ	(48*MHz)

#define ACLKM_CORE_L_HZ	(300*MHz)
#define ATCLK_CORE_L_HZ	(300*MHz)
#define PCLK_DBG_L_HZ	(100*MHz)

#define ACLKM_CORE_B_HZ	(300*MHz)
#define ATCLK_CORE_B_HZ	(300*MHz)
#define PCLK_DBG_B_HZ	(100*MHz)

#define PERIHP_ACLK_HZ	(148500*KHz)
#define PERIHP_HCLK_HZ	(148500*KHz)
#define PERIHP_PCLK_HZ	(37125*KHz)

#define PERILP0_ACLK_HZ	(99000*KHz)
#define PERILP0_HCLK_HZ	(99000*KHz)
#define PERILP0_PCLK_HZ	(49500*KHz)

#define PERILP1_HCLK_HZ	(99000*KHz)
#define PERILP1_PCLK_HZ	(49500*KHz)

#define PWM_CLOCK_HZ    PMU_PCLK_HZ

enum apll_l_frequencies {
	APLL_L_1600_MHZ,
	APLL_L_600_MHZ,
	/* The RK3399 cluster-0 OPPs above 600 MHz (Linux rk3399.dtsi) */
	APLL_L_1008_MHZ,
	APLL_L_1200_MHZ,
	APLL_L_1416_MHZ,
};

enum apll_b_frequencies {
	APLL_B_600_MHZ,
};

void rk3399_configure_cpu_l(struct rockchip_cru *cru,
			    enum apll_l_frequencies apll_l_freq);
void rk3399_configure_cpu_b(struct rockchip_cru *cru,
			    enum apll_b_frequencies apll_b_freq);

#endif	/* __ASM_ARCH_CRU_RK3399_H_ */
