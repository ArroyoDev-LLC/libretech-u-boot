// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/*
 * (C) Copyright 2017 Rockchip Electronics Co., Ltd.
 */
#include <config.h>
#include <clk.h>
#include <debug_uart.h>
#include <dm.h>
#include <dt-structs.h>
#include <init.h>
#include <log.h>
#include <ram.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/io.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/cru_rk3328.h>
#include <asm/arch-rockchip/grf_rk3328.h>
#include <asm/arch-rockchip/sdram.h>
#include <asm/arch-rockchip/sdram_rk3328.h>
#include <asm/arch-rockchip/uart.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

struct dram_info {
#ifdef CONFIG_TPL_BUILD
	struct ddr_pctl_regs *pctl;
	struct ddr_phy_regs *phy;
	struct clk ddr_clk;
	struct rk3328_cru *cru;
	struct msch_regs *msch;
	struct rk3328_ddr_grf_regs *ddr_grf;
#endif
	struct ram_info info;
	struct rk3328_grf_regs *grf;
};

#ifdef CONFIG_TPL_BUILD

struct rk3328_sdram_channel sdram_ch;

#if IS_ENABLED(CONFIG_RAM_ROCKCHIP_RK3328_FALLBACK)
#include <asm/arch-rockchip/sdram_rk3328_ddr4_666.h>

/*
 * The step-down rung, from the same cells rk3328-sdram-ddr4-666.dtsi uses.
 * Writable: a rung's geometry pass rewrites its own pctl/cap fields.
 */
#define RK3328_SDRAM_C_CELL(c) c,
static u32 rk3328_sdram_fallback_cells[] = {
	RK3328_SDRAM_DDR4_666(RK3328_SDRAM_C_CELL)
};
_Static_assert(sizeof(rk3328_sdram_fallback_cells) ==
	       sizeof(struct rk3328_sdram_params),
	       "the DDR4-666 cells are not a whole struct rk3328_sdram_params");
#endif

struct rockchip_dmc_plat {
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	struct dtd_rockchip_rk3328_dmc dtplat;
#else
	struct rk3328_sdram_params sdram_params;
#endif
	struct regmap *map;
};

#if CONFIG_IS_ENABLED(OF_PLATDATA)
static int conv_of_plat(struct udevice *dev)
{
	struct rockchip_dmc_plat *plat = dev_get_plat(dev);
	struct dtd_rockchip_rk3328_dmc *dtplat = &plat->dtplat;
	int ret;

	ret = regmap_init_mem_plat(dev, dtplat->reg, sizeof(dtplat->reg[0]),
				   ARRAY_SIZE(dtplat->reg) / 2, &plat->map);
	if (ret)
		return ret;

	return 0;
}
#endif

static void rkclk_ddr_reset(struct dram_info *dram,
			    u32 ctl_srstn, u32 ctl_psrstn,
			    u32 phy_srstn, u32 phy_psrstn)
{
	writel(ddrctrl_srstn_req(ctl_srstn) | ddrctrl_psrstn_req(ctl_psrstn) |
		ddrphy_srstn_req(phy_srstn) | ddrphy_psrstn_req(phy_psrstn),
		&dram->cru->softrst_con[5]);
	writel(ddrctrl_asrstn_req(ctl_srstn), &dram->cru->softrst_con[9]);
}

static void rkclk_set_dpll(struct dram_info *dram, unsigned int hz)
{
	unsigned int refdiv, postdiv1, postdiv2, fbdiv;
	int delay = 1000;
	u32 mhz = hz / MHZ;

	refdiv = 1;
	if (mhz <= 300) {
		postdiv1 = 4;
		postdiv2 = 2;
	} else if (mhz <= 400) {
		postdiv1 = 6;
		postdiv2 = 1;
	} else if (mhz <= 600) {
		postdiv1 = 4;
		postdiv2 = 1;
	} else if (mhz <= 800) {
		postdiv1 = 3;
		postdiv2 = 1;
	} else if (mhz <= 1600) {
		postdiv1 = 2;
		postdiv2 = 1;
	} else {
		postdiv1 = 1;
		postdiv2 = 1;
	}
	fbdiv = (mhz * refdiv * postdiv1 * postdiv2) / 24;

	writel(((0x1 << 4) << 16) | (0 << 4), &dram->cru->mode_con);
	writel(POSTDIV1(postdiv1) | FBDIV(fbdiv), &dram->cru->dpll_con[0]);
	writel(DSMPD(1) | POSTDIV2(postdiv2) | REFDIV(refdiv),
	       &dram->cru->dpll_con[1]);

	while (delay > 0) {
		udelay(1);
		if (LOCK(readl(&dram->cru->dpll_con[1])))
			break;
		delay--;
	}

	writel(((0x1 << 4) << 16) | (1 << 4), &dram->cru->mode_con);
}

static void rkclk_configure_ddr(struct dram_info *dram,
				struct rk3328_sdram_params *sdram_params)
{
	void __iomem *phy_base = dram->phy;

	/* choose DPLL for ddr clk source */
	clrbits_le32(PHY_REG(phy_base, 0xef), 1 << 7);

	/* for inno ddr phy need 2*freq */
	rkclk_set_dpll(dram,  sdram_params->base.ddr_freq * MHZ * 2);
}

/* return ddrconfig value
 *       (-1), find ddrconfig fail
 *       other, the ddrconfig value
 * only support cs0_row >= cs1_row
 */
static u32 calculate_ddrconfig(struct rk3328_sdram_params *sdram_params)
{
	struct sdram_cap_info *cap_info = &sdram_params->ch.cap_info;
	u32 cs, bw, die_bw, col, row, bank;
	u32 cs1_row;
	u32 i, tmp;
	u32 ddrconf = -1;

	cs = cap_info->rank;
	bw = cap_info->bw;
	die_bw = cap_info->dbw;
	col = cap_info->col;
	row = cap_info->cs0_row;
	cs1_row = cap_info->cs1_row;
	bank = cap_info->bk;

	if (sdram_params->base.dramtype == DDR4) {
		/* when DDR_TEST, CS always at MSB position for easy test */
		if (cs == 2 && row == cs1_row) {
			/* include 2cs cap both 2^n  or both (2^n - 2^(n-2)) */
			tmp = ((row - 13) << 3) | (1 << 2) | (bw & 0x2) |
			      die_bw;
			for (i = 17; i < 21; i++) {
				if (((tmp & 0x7) ==
				     (ddr4_cfg_2_rbc[i - 10] & 0x7)) &&
				    ((tmp & 0x3c) <=
				     (ddr4_cfg_2_rbc[i - 10] & 0x3c))) {
					ddrconf = i;
					goto out;
				}
			}
		}

		tmp = ((cs - 1) << 6) | ((row - 13) << 3) | (bw & 0x2) | die_bw;
		for (i = 10; i < 17; i++) {
			if (((tmp & 0x7) == (ddr4_cfg_2_rbc[i - 10] & 0x7)) &&
			    ((tmp & 0x3c) <= (ddr4_cfg_2_rbc[i - 10] & 0x3c)) &&
			    ((tmp & 0x40) <= (ddr4_cfg_2_rbc[i - 10] & 0x40))) {
				ddrconf = i;
				goto out;
			}
		}
	} else {
		if (bank == 2) {
			ddrconf = 8;
			goto out;
		}

		/* when DDR_TEST, CS always at MSB position for easy test */
		if (cs == 2 && row == cs1_row) {
			/* include 2cs cap both 2^n  or both (2^n - 2^(n-2)) */
			for (i = 5; i < 8; i++) {
				if ((bw + col - 11) == (ddr_cfg_2_rbc[i] &
							0x3)) {
					ddrconf = i;
					goto out;
				}
			}
		}

		tmp = ((row - 13) << 4) | (1 << 2) | ((bw + col - 11) << 0);
		for (i = 0; i < 5; i++)
			if (((tmp & 0xf) == (ddr_cfg_2_rbc[i] & 0xf)) &&
			    ((tmp & 0x30) <= (ddr_cfg_2_rbc[i] & 0x30))) {
				ddrconf = i;
				goto out;
			}
	}

out:
	if (ddrconf > 20)
		printf("calculate ddrconfig error\n");

	return ddrconf;
}

/*******
 * calculate controller dram address map, and setting to register.
 * argument sdram_ch.ddrconf must be right value before
 * call this function.
 *******/
static void set_ctl_address_map(struct dram_info *dram,
				struct rk3328_sdram_params *sdram_params)
{
	struct sdram_cap_info *cap_info = &sdram_params->ch.cap_info;
	void __iomem *pctl_base = dram->pctl;

	sdram_copy_to_reg((u32 *)(pctl_base + DDR_PCTL2_ADDRMAP0),
			  &addrmap[cap_info->ddrconfig][0], 9 * 4);
	if (sdram_params->base.dramtype == LPDDR3 && cap_info->row_3_4)
		setbits_le32(pctl_base + DDR_PCTL2_ADDRMAP6, 1 << 31);
	if (sdram_params->base.dramtype == DDR4 && cap_info->bw == 0x1)
		setbits_le32(pctl_base + DDR_PCTL2_PCCFG, 1 << 8);

	if (cap_info->rank == 1)
		clrsetbits_le32(pctl_base + DDR_PCTL2_ADDRMAP0, 0x1f, 0x1f);
}

static int data_training(struct dram_info *dram, u32 cs, u32 dramtype)
{
	void __iomem *pctl_base = dram->pctl;
	u32 dis_auto_zq = 0;
	u32 pwrctl;
	u32 ret;

	/* disable auto low-power */
	pwrctl = readl(pctl_base + DDR_PCTL2_PWRCTL);
	writel(0, pctl_base + DDR_PCTL2_PWRCTL);

	dis_auto_zq = pctl_dis_zqcs_aref(dram->pctl);

	ret = phy_data_training(dram->phy, cs, dramtype);

	pctl_rest_zqcs_aref(dram->pctl, dis_auto_zq);

	/* restore auto low-power */
	writel(pwrctl, pctl_base + DDR_PCTL2_PWRCTL);

	return ret;
}

static void rx_deskew_switch_adjust(struct dram_info *dram)
{
	u32 i, deskew_val;
	u32 gate_val = 0;
	void __iomem *phy_base = dram->phy;

	for (i = 0; i < 4; i++)
		gate_val = MAX(readl(PHY_REG(phy_base, 0xfb + i)), gate_val);

	deskew_val = (gate_val >> 3) + 1;
	deskew_val = (deskew_val > 0x1f) ? 0x1f : deskew_val;
	clrsetbits_le32(PHY_REG(phy_base, 0x6e), 0xc, (deskew_val & 0x3) << 2);
	clrsetbits_le32(PHY_REG(phy_base, 0x6f), 0x7 << 4,
			(deskew_val & 0x1c) << 2);
}

static void tx_deskew_switch_adjust(struct dram_info *dram)
{
	void __iomem *phy_base = dram->phy;

	clrsetbits_le32(PHY_REG(phy_base, 0x6e), 0x3, 1);
}

static void set_ddrconfig(struct dram_info *dram, u32 ddrconfig)
{
	writel(ddrconfig, &dram->msch->ddrconf);
}

static void sdram_msch_config(struct msch_regs *msch,
			      struct sdram_msch_timings *noc_timings)
{
	writel(noc_timings->ddrtiming.d32, &msch->ddrtiming);

	writel(noc_timings->ddrmode.d32, &msch->ddrmode);
	writel(noc_timings->readlatency, &msch->readlatency);

	writel(noc_timings->activate.d32, &msch->activate);
	writel(noc_timings->devtodev.d32, &msch->devtodev);
	writel(noc_timings->ddr4timing.d32, &msch->ddr4_timing);
	writel(noc_timings->agingx0, &msch->aging0);
	writel(noc_timings->agingx0, &msch->aging1);
	writel(noc_timings->agingx0, &msch->aging2);
	writel(noc_timings->agingx0, &msch->aging3);
	writel(noc_timings->agingx0, &msch->aging4);
	writel(noc_timings->agingx0, &msch->aging5);
}

static void dram_all_config(struct dram_info *dram,
			    struct rk3328_sdram_params *sdram_params)
{
	struct sdram_cap_info *cap_info = &sdram_params->ch.cap_info;
	u32 sys_reg2 = 0;
	u32 sys_reg3 = 0;

	set_ddrconfig(dram, cap_info->ddrconfig);
	sdram_org_config(cap_info, &sdram_params->base, &sys_reg2,
			 &sys_reg3, 0);
	writel(sys_reg2, &dram->grf->os_reg[2]);
	writel(sys_reg3, &dram->grf->os_reg[3]);

	sdram_msch_config(dram->msch, &sdram_ch.noc_timings);
}

static void enable_low_power(struct dram_info *dram,
			     struct rk3328_sdram_params *sdram_params)
{
	void __iomem *pctl_base = dram->pctl;

	/* enable upctl2 axi clock auto gating */
	writel(0x00800000, &dram->ddr_grf->ddr_grf_con[0]);
	writel(0x20012001, &dram->ddr_grf->ddr_grf_con[2]);
	/* enable upctl2 core clock auto gating */
	writel(0x001e001a, &dram->ddr_grf->ddr_grf_con[2]);
	/* enable sr, pd */
	if (PD_IDLE == 0)
		clrbits_le32(pctl_base + DDR_PCTL2_PWRCTL, (1 << 1));
	else
		setbits_le32(pctl_base + DDR_PCTL2_PWRCTL, (1 << 1));
	if (SR_IDLE == 0)
		clrbits_le32(pctl_base + DDR_PCTL2_PWRCTL,	1);
	else
		setbits_le32(pctl_base + DDR_PCTL2_PWRCTL, 1);
	setbits_le32(pctl_base + DDR_PCTL2_PWRCTL, (1 << 3));
}

static int sdram_init(struct dram_info *dram,
		      struct rk3328_sdram_params *sdram_params, u32 pre_init)
{
	struct sdram_cap_info *cap_info = &sdram_params->ch.cap_info;
	void __iomem *pctl_base = dram->pctl;

	rkclk_ddr_reset(dram, 1, 1, 1, 1);
	udelay(10);
	/*
	 * dereset ddr phy psrstn to config pll,
	 * if using phy pll psrstn must be dereset
	 * before config pll
	 */
	rkclk_ddr_reset(dram, 1, 1, 1, 0);
	rkclk_configure_ddr(dram, sdram_params);

	/* release phy srst to provide clk to ctrl */
	rkclk_ddr_reset(dram, 1, 1, 0, 0);
	udelay(10);
	phy_soft_reset(dram->phy);
	/* release ctrl presetn, and config ctl registers */
	rkclk_ddr_reset(dram, 1, 0, 0, 0);
	pctl_cfg(dram->pctl, &sdram_params->pctl_regs, SR_IDLE, PD_IDLE);
	cap_info->ddrconfig = calculate_ddrconfig(sdram_params);
	set_ctl_address_map(dram, sdram_params);
	phy_cfg(dram->phy, &sdram_params->phy_regs, &sdram_params->skew,
		&sdram_params->base, cap_info->bw);

	/* enable dfi_init_start to init phy after ctl srstn deassert */
	setbits_le32(pctl_base + DDR_PCTL2_DFIMISC, (1 << 5) | (1 << 4));
	rkclk_ddr_reset(dram, 0, 0, 0, 0);
	/* wait for dfi_init_done and dram init complete */
	while ((readl(pctl_base + DDR_PCTL2_STAT) & 0x7) == 0)
		continue;

	/* do ddr gate training */
	if (data_training(dram, 0, sdram_params->base.dramtype) != 0) {
		printf("data training error\n");
		return -1;
	}

	if (sdram_params->base.dramtype == DDR4)
		pctl_write_vrefdq(dram->pctl, 0x3, 5670,
				  sdram_params->base.dramtype);

	if (pre_init != 0) {
		rx_deskew_switch_adjust(dram);
		tx_deskew_switch_adjust(dram);
	}

	dram_all_config(dram, sdram_params);
	enable_low_power(dram, sdram_params);

	return 0;
}

static u64 dram_detect_cap(struct dram_info *dram,
			   struct rk3328_sdram_params *sdram_params,
			   unsigned char channel)
{
	struct sdram_cap_info *cap_info = &sdram_params->ch.cap_info;

	/*
	 * for ddr3: ddrconf = 3
	 * for ddr4: ddrconf = 12
	 * for lpddr3: ddrconf = 3
	 * default bw = 1
	 */
	u32 bk, bktmp;
	u32 col, coltmp;
	u32 rowtmp;
	u32 cs;
	u32 bw = 1;
	u32 dram_type = sdram_params->base.dramtype;

	if (dram_type != DDR4) {
		/* detect col and bk for ddr3/lpddr3 */
		coltmp = 12;
		bktmp = 3;
		rowtmp = 16;

		if (sdram_detect_col(cap_info, coltmp) != 0)
			goto cap_err;
		sdram_detect_bank(cap_info, coltmp, bktmp);
		sdram_detect_dbw(cap_info, dram_type);
	} else {
		/* detect bg for ddr4 */
		coltmp = 10;
		bktmp = 4;
		rowtmp = 17;

		col = 10;
		bk = 2;
		cap_info->col = col;
		cap_info->bk = bk;
		sdram_detect_bg(cap_info, coltmp);
	}

	/* detect row */
	if (sdram_detect_row(cap_info, coltmp, bktmp, rowtmp) != 0)
		goto cap_err;

	/* detect row_3_4 */
	sdram_detect_row_3_4(cap_info, coltmp, bktmp);

	/* bw and cs detect using data training */
	if (data_training(dram, 1, dram_type) == 0)
		cs = 1;
	else
		cs = 0;
	cap_info->rank = cs + 1;

	bw = 2;
	cap_info->bw = bw;

	cap_info->cs0_high16bit_row = cap_info->cs0_row;
	if (cs) {
		cap_info->cs1_row = cap_info->cs0_row;
		cap_info->cs1_high16bit_row = cap_info->cs0_row;
	} else {
		cap_info->cs1_row = 0;
		cap_info->cs1_high16bit_row = 0;
	}

	return 0;
cap_err:
	return -1;
}

/*
 * One rung: train the parameter set, detect the geometry, retrain with it.
 * A training failure in either pass is returned, not ignored (the passes'
 * results used to be dropped, so a failed training went on to detect and
 * boot on an untrained controller).
 */
static int sdram_try(struct dram_info *dram,
		     struct rk3328_sdram_params *sdram_params)
{
	u32 sys_reg = 0;
	u32 sys_reg3 = 0;
	struct sdram_cap_info *cap_info = &sdram_params->ch.cap_info;
	int ret;

	debug("Starting SDRAM initialization...\n");

	memcpy(&sdram_ch, &sdram_params->ch,
	       sizeof(struct rk3328_sdram_channel));

	ret = sdram_init(dram, sdram_params, 0);
	if (ret)
		return ret;
	dram_detect_cap(dram, sdram_params, 0);

	/* modify bw, cs related timing */
	pctl_remodify_sdram_params(&sdram_params->pctl_regs, cap_info,
				   sdram_params->base.dramtype);

	if (cap_info->bw == 2)
		sdram_ch.noc_timings.ddrtiming.b.bwratio = 0;
	else
		sdram_ch.noc_timings.ddrtiming.b.bwratio = 1;

	/* reinit sdram by real dram cap */
	ret = sdram_init(dram, sdram_params, 1);
	if (ret)
		return ret;

	/* redetect cs1 row */
	sdram_detect_cs1_row(cap_info, sdram_params->base.dramtype);
	if (cap_info->cs1_row) {
		sys_reg = readl(&dram->grf->os_reg[2]);
		sys_reg3 = readl(&dram->grf->os_reg[3]);
		SYS_REG_ENC_CS1_ROW(cap_info->cs1_row,
				    sys_reg, sys_reg3, 0);
		writel(sys_reg, &dram->grf->os_reg[2]);
		writel(sys_reg3, &dram->grf->os_reg[3]);
	}

	return 0;
}

/* A bijection on u32: consecutive keys differ in every byte lane. */
static inline u32 rk3328_dram_pat(u32 k)
{
	return (k * 0x9e3779b9u) ^ 0xa5a5a5a5u;
}

#define RK3328_MEMTEST_DENSE	256
/* Block keys start here; stride keys (megabyte indices) stay below 0x1000. */
#define RK3328_MEMTEST_BLOCK_KEY	0x40000000u

/*
 * Write the dense block and the address-line walk of one megabyte, or check
 * and clear them. Keys are RK3328_MEMTEST_BLOCK_KEY plus the word index, so
 * no word of the test holds the same value as another and an alias always
 * reads back wrong.
 */
static u32 rk3328_dram_block(ulong off, bool check, u32 *lanes)
{
	volatile u32 *p = (volatile u32 *)(CFG_SYS_SDRAM_BASE + off);
	const u32 k = RK3328_MEMTEST_BLOCK_KEY + (u32)(off >> 2);
	u32 i, b, d, errors = 0;
	volatile u32 *w;

	for (i = 0; i < RK3328_MEMTEST_DENSE + 10; i++) {
		b = i - RK3328_MEMTEST_DENSE + 10;
		w = i < RK3328_MEMTEST_DENSE ? &p[i] : &p[(1u << b) >> 2];
		if (!check) {
			*w = rk3328_dram_pat(k + i);
			continue;
		}
		d = *w ^ rk3328_dram_pat(k + i);
		*w = 0;
		if (d) {
			errors++;
			*lanes |= d;
		}
	}

	return errors;
}

/*
 * The memtest a rung must pass before it is accepted (P16 M2; bootloader
 * policy "DRAM capacity & geometry are detected at runtime": a rung is
 * accepted when it trains, detects its full size and memtests clean). The
 * shape is the Amlogic TPL's (arch/arm/mach-meson/dram-gx.c):
 *
 *  - one hashed word per megabyte over the whole usable window, all written
 *    before any is read, so an aliased map, a wrong rank size or a dead CS1
 *    reads back another megabyte's word (AXI bits 20 and up);
 *  - in the lowest and the highest megabyte, a dense block of 256 words
 *    (bits 2..9, every byte lane through all 256 values) and one word at
 *    1 << b for b = 10..19, the bits the stride never varies (bank, column
 *    and low row bits).
 *
 * Every checked word is then cleared to 0, so nothing of the test is left
 * where the BootROM loads the SPL (SPL_TEXT_BASE 0). The window stops at
 * SDRAM_MAX_SIZE, where the peripherals start. Returns the error count;
 * *lanes is the OR of every mismatch, so a failure names its byte lanes.
 */
static u32 rk3328_dram_memtest(struct rk3328_sdram_params *sdram_params,
			       u32 *lanes)
{
	struct sdram_cap_info *cap_info = &sdram_params->ch.cap_info;
	u64 cap = sdram_get_cs_cap(cap_info, 3, sdram_params->base.dramtype);
	ulong end, off;
	u32 errors = 0, d;

	if (cap_info->row_3_4)
		cap = cap * 3 / 4;
	end = min_t(u64, cap, SDRAM_MAX_SIZE) & ~(ulong)(SZ_1M - 1);
	*lanes = 0;
	if (end < 2 * SZ_1M)
		return 1;

	for (off = 0; off < end; off += SZ_1M)
		writel(rk3328_dram_pat(off >> 20), CFG_SYS_SDRAM_BASE + off);
	rk3328_dram_block(0, false, NULL);
	rk3328_dram_block(end - SZ_1M, false, NULL);

	errors += rk3328_dram_block(0, true, lanes);
	errors += rk3328_dram_block(end - SZ_1M, true, lanes);
	for (off = 0; off < end; off += SZ_1M) {
		/* The two blocks' first words sit on these; already checked. */
		if (off == 0 || off == end - SZ_1M)
			continue;
		d = readl(CFG_SYS_SDRAM_BASE + off) ^ rk3328_dram_pat(off >> 20);
		writel(0, CFG_SYS_SDRAM_BASE + off);
		if (d) {
			errors++;
			*lanes |= d;
		}
	}

	return errors;
}

/*
 * One rung: train, then memtest. A memtest failure is a rung failure, so it
 * steps down like a training failure.
 */
static int sdram_rung(struct dram_info *dram,
		      struct rk3328_sdram_params *sdram_params)
{
	u32 errors, lanes;
	int ret;

	ret = sdram_try(dram, sdram_params);
	if (ret) {
		printf("DDR: training failed at %u MHz (%d)\n",
		       sdram_params->base.ddr_freq, ret);
		return ret;
	}
	errors = rk3328_dram_memtest(sdram_params, &lanes);
	if (errors) {
		printf("DDR: memtest FAIL at %u MHz (%u errors, lanes %08x)\n",
		       sdram_params->base.ddr_freq, errors, lanes);
		return -EIO;
	}

	return 0;
}

/*
 * The DRAM step-down (bootloader policy, "DRAM capacity & geometry are
 * detected at runtime"; boot-chain-policy §3a): train and memtest the
 * configured set; on a failure of either, the fallback set when the board
 * has one (RAM_ROCKCHIP_RK3328_FALLBACK), and a degraded boot says so.
 */
static int sdram_init_detect(struct dram_info *dram,
			     struct rk3328_sdram_params *sdram_params,
			     struct rk3328_sdram_params *fallback)
{
	bool fell = false;
	int ret;

	ret = sdram_rung(dram, sdram_params);
	if (ret && fallback) {
		printf("DDR: trying %u MHz\n", fallback->base.ddr_freq);
		sdram_params = fallback;
		ret = sdram_rung(dram, sdram_params);
		fell = !ret;
	}
	if (ret)
		return ret;

	sdram_print_ddr_info(&sdram_params->ch.cap_info, &sdram_params->base, 0);
	sdram_print_ddr_summary(&sdram_params->ch.cap_info, &sdram_params->base, 0);
	if (fell)
		printf("DDR: WARNING clock fallback\n");

	return 0;
}

static int rk3328_dmc_init(struct udevice *dev)
{
	struct dram_info *priv = dev_get_priv(dev);
	struct rockchip_dmc_plat *plat = dev_get_plat(dev);
	struct rk3328_sdram_params *fallback = NULL;
	int ret;

#if IS_ENABLED(CONFIG_RAM_ROCKCHIP_RK3328_FALLBACK)
	fallback = (void *)rk3328_sdram_fallback_cells;
#endif
#if CONFIG_IS_ENABLED(OF_REAL)
	struct rk3328_sdram_params *params = &plat->sdram_params;
#else
	struct dtd_rockchip_rk3328_dmc *dtplat = &plat->dtplat;
	struct rk3328_sdram_params *params =
					(void *)dtplat->rockchip_sdram_params;

	ret = conv_of_plat(dev);
	if (ret)
		return ret;
#endif
	priv->phy = regmap_get_range(plat->map, 0);
	priv->pctl = regmap_get_range(plat->map, 1);
	priv->grf = regmap_get_range(plat->map, 2);
	priv->cru = regmap_get_range(plat->map, 3);
	priv->msch = regmap_get_range(plat->map, 4);
	priv->ddr_grf = regmap_get_range(plat->map, 5);

	debug("%s phy %p pctrl %p grf %p cru %p msch %p ddr_grf %p\n",
	      __func__, priv->phy, priv->pctl, priv->grf, priv->cru,
	      priv->msch, priv->ddr_grf);
	ret = sdram_init_detect(priv, params, fallback);
	if (ret < 0) {
		printf("%s DRAM init failed%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int rk3328_dmc_of_to_plat(struct udevice *dev)
{
#if CONFIG_IS_ENABLED(OF_REAL)
	struct rockchip_dmc_plat *plat = dev_get_plat(dev);
	int ret;

	ret = dev_read_u32_array(dev, "rockchip,sdram-params",
				 (u32 *)&plat->sdram_params,
				 sizeof(plat->sdram_params) / sizeof(u32));
	if (ret) {
		printf("%s: Cannot read rockchip,sdram-params %d\n",
		       __func__, ret);
		return ret;
	}
	ret = regmap_init_mem(dev, &plat->map);
	if (ret)
		printf("%s: regmap failed %d\n", __func__, ret);
#endif
	return 0;
}

#endif

static int rk3328_dmc_probe(struct udevice *dev)
{
#ifdef CONFIG_TPL_BUILD
	if (rk3328_dmc_init(dev))
		return 0;
#else
	struct dram_info *priv = dev_get_priv(dev);

	priv->grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);
	debug("%s: grf=%p\n", __func__, priv->grf);
	priv->info.base = CFG_SYS_SDRAM_BASE;
	priv->info.size = rockchip_sdram_size(
				(phys_addr_t)&priv->grf->os_reg[2]);
#endif
	return 0;
}

static int rk3328_dmc_get_info(struct udevice *dev, struct ram_info *info)
{
	struct dram_info *priv = dev_get_priv(dev);

	*info = priv->info;

	return 0;
}

static struct ram_ops rk3328_dmc_ops = {
	.get_info = rk3328_dmc_get_info,
};

static const struct udevice_id rk3328_dmc_ids[] = {
	{ .compatible = "rockchip,rk3328-dmc" },
	{ }
};

U_BOOT_DRIVER(rockchip_rk3328_dmc) = {
	.name = "rockchip_rk3328_dmc",
	.id = UCLASS_RAM,
	.of_match = rk3328_dmc_ids,
	.ops = &rk3328_dmc_ops,
#ifdef CONFIG_TPL_BUILD
	.of_to_plat = rk3328_dmc_of_to_plat,
#endif
	.probe = rk3328_dmc_probe,
	.priv_auto	= sizeof(struct dram_info),
#ifdef CONFIG_TPL_BUILD
	.plat_auto	= sizeof(struct rockchip_dmc_plat),
#endif
};
