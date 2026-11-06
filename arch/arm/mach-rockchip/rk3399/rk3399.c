// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2016 Rockchip Electronics Co., Ltd
 */

#include <fdt_support.h>
#include <init.h>
#include <log.h>
#include <spl.h>
#include <spl_gpio.h>
#include <syscon.h>
#include <asm/armv8/mmu.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/cru.h>
#include <asm/arch-rockchip/gpio.h>
#include <asm/arch-rockchip/grf_rk3399.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/arch-rockchip/usbflash-alts.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/printk.h>
#include <power/regulator.h>

#define GRF_EMMCCORE_CON11 0xff77f02c
#define GRF_BASE	0xff770000

const char * const boot_devices[BROM_LAST_BOOTSOURCE + 1] = {
	[BROM_BOOTSOURCE_EMMC] = "/mmc@fe330000",
	[BROM_BOOTSOURCE_SPINOR] = "/spi@ff1d0000/flash@0",
	[BROM_BOOTSOURCE_SD] = "/mmc@fe320000",
};

static struct mm_region rk3399_mem_map[] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0xf8000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0xf8000000UL,
		.phys = 0xf8000000UL,
		.size = 0x08000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = rk3399_mem_map;

#ifdef CONFIG_XPL_BUILD

#define TIMER_END_COUNT_L	0x00
#define TIMER_END_COUNT_H	0x04
#define TIMER_INIT_COUNT_L	0x10
#define TIMER_INIT_COUNT_H	0x14
#define TIMER_CONTROL_REG	0x1c

#define TIMER_EN	0x1
#define TIMER_FMODE	(0 << 1)
#define TIMER_RMODE	(1 << 1)

void rockchip_stimer_init(void)
{
	/* If Timer already enabled, don't re-init it */
	u32 reg = readl(CONFIG_ROCKCHIP_STIMER_BASE + TIMER_CONTROL_REG);

	if (reg & TIMER_EN)
		return;

	writel(0xffffffff, CONFIG_ROCKCHIP_STIMER_BASE + TIMER_END_COUNT_L);
	writel(0xffffffff, CONFIG_ROCKCHIP_STIMER_BASE + TIMER_END_COUNT_H);
	writel(0, CONFIG_ROCKCHIP_STIMER_BASE + TIMER_INIT_COUNT_L);
	writel(0, CONFIG_ROCKCHIP_STIMER_BASE + TIMER_INIT_COUNT_H);
	writel(TIMER_EN | TIMER_FMODE, CONFIG_ROCKCHIP_STIMER_BASE + \
	       TIMER_CONTROL_REG);
}
#endif

int arch_cpu_init(void)
{

#ifdef CONFIG_XPL_BUILD
	struct rk3399_pmusgrf_regs *sgrf;
	struct rk3399_grf_regs *grf;

	/*
	 * Disable DDR and SRAM security regions.
	 *
	 * As we are entered from the BootROM, the region from
	 * 0x0 through 0xfffff (i.e. the first MB of memory) will
	 * be protected. This will cause issues with the DW_MMC
	 * driver, which tries to DMA from/to the stack (likely)
	 * located in this range.
	 */
	sgrf = syscon_get_first_range(ROCKCHIP_SYSCON_PMUSGRF);
	rk_clrsetreg(&sgrf->ddr_rgn_con[16], 0x1ff, 0);
	rk_clrreg(&sgrf->slv_secure_con4, 0x2000);

	/*  eMMC clock generator: disable the clock multipilier */
	grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);
	rk_clrreg(&grf->emmccore_con[11], 0x0ff);
#endif

	return 0;
}

#ifdef CONFIG_DEBUG_UART_BOARD_INIT
void board_debug_uart_init(void)
{
#define GRF_BASE	0xff770000
#define GPIO0_BASE	0xff720000
#define PMUGRF_BASE	0xff320000
	struct rk3399_grf_regs * const grf = (void *)GRF_BASE;

#if defined(CONFIG_DEBUG_UART_BASE) && (CONFIG_DEBUG_UART_BASE == 0xff180000)
	/* Enable early UART0 on the RK3399 */
	rk_clrsetreg(&grf->gpio2c_iomux,
		     GRF_GPIO2C0_SEL_MASK,
		     GRF_UART0BT_SIN << GRF_GPIO2C0_SEL_SHIFT);
	rk_clrsetreg(&grf->gpio2c_iomux,
		     GRF_GPIO2C1_SEL_MASK,
		     GRF_UART0BT_SOUT << GRF_GPIO2C1_SEL_SHIFT);
#elif defined(CONFIG_DEBUG_UART_BASE) && (CONFIG_DEBUG_UART_BASE == 0xff1B0000)
	/* Enable early UART3 on the RK3399 */
	rk_clrsetreg(&grf->gpio3b_iomux,
		     GRF_GPIO3B6_SEL_MASK,
		     GRF_UART3_SIN << GRF_GPIO3B6_SEL_SHIFT);
	rk_clrsetreg(&grf->gpio3b_iomux,
		     GRF_GPIO3B7_SEL_MASK,
		     GRF_UART3_SOUT << GRF_GPIO3B7_SEL_SHIFT);
#else
	struct rk3399_pmugrf_regs * const pmugrf = (void *)PMUGRF_BASE;
	struct rockchip_gpio_regs * const gpio = (void *)GPIO0_BASE;

	if (IS_ENABLED(CONFIG_XPL_BUILD) &&
	    (IS_ENABLED(CONFIG_TARGET_CHROMEBOOK_BOB) ||
	     IS_ENABLED(CONFIG_TARGET_CHROMEBOOK_KEVIN))) {
		rk_setreg(&grf->io_vsel, 1 << 0);

		/*
		 * Let's enable these power rails here, we are already running
		 * the SPI-Flash-based code.
		 */
		spl_gpio_output(gpio, GPIO(BANK_B, 2), 1);  /* PP1500_EN */
		spl_gpio_set_pull(&pmugrf->gpio0_p, GPIO(BANK_B, 2),
				  GPIO_PULL_NORMAL);

		spl_gpio_output(gpio, GPIO(BANK_B, 4), 1);  /* PP3000_EN */
		spl_gpio_set_pull(&pmugrf->gpio0_p, GPIO(BANK_B, 4),
				  GPIO_PULL_NORMAL);
	}

	/* Enable early UART2 channel C on the RK3399 */
	rk_clrsetreg(&grf->gpio4c_iomux,
		     GRF_GPIO4C3_SEL_MASK,
		     GRF_UART2DBGC_SIN << GRF_GPIO4C3_SEL_SHIFT);
	rk_clrsetreg(&grf->gpio4c_iomux,
		     GRF_GPIO4C4_SEL_MASK,
		     GRF_UART2DBGC_SOUT << GRF_GPIO4C4_SEL_SHIFT);
	/* Set channel C as UART2 input */
	rk_clrsetreg(&grf->soc_con7,
		     GRF_UART_DBG_SEL_MASK,
		     GRF_UART_DBG_SEL_C << GRF_UART_DBG_SEL_SHIFT);
#endif
}
#endif

#if defined(CONFIG_XPL_BUILD)
#if defined(CONFIG_TPL_BUILD)
/*
 * Without GPIO support in TPL, follow a watchdog or TSADC reset with one
 * global soft reset, here, before DRAM init. Such a reset fires with
 * whatever power domains the OS had switched off, and their logic misses
 * it. So power those domains on and release their buses first, as BL31's
 * pmu_power_domains_on() does before its own soft reset (FST_GLB_RST_ST
 * alone, which needs nothing more), and then reset again to reach them.
 */
#define RK3399_PMU_BASE		0xff310000
#define PMU_PWRDN_CON		(RK3399_PMU_BASE + 0x14)
#define PMU_PWRDN_ST		(RK3399_PMU_BASE + 0x18)
#define PMU_BUS_IDLE_REQ	(RK3399_PMU_BASE + 0x60)
#define PMU_BUS_IDLE_ST		(RK3399_PMU_BASE + 0x64)
#define PMU_BUS_IDLE_ACK	(RK3399_PMU_BASE + 0x68)
/*
 * BL31's list: TCPD0/1 (8, 9), GPU 15, VCODEC 16, VDU 17, RGA 18, IEP 19,
 * VO 20, ISP0 22, ISP1 23, HDCP 24, GMAC 25, EDP 28, SDIOAUDIO 31
 */
#define PMU_PD_MASK		0x93df8300
/*
 * Their buses: GPU 0, VCODEC 3, VDU 4, RGA 5, IEP 6, VOPB 7, VOPL 8,
 * ISP0 9, ISP1 10, HDCP 11, EDP 22, GMAC 23, SDIOAUDIO 29
 */
#define PMU_BUS_MASK		0x20c00ff9

static void rk3399_pmu_domains_on(void)
{
	u32 con = readl(PMU_PWRDN_CON), pst = readl(PMU_PWRDN_ST);
	u32 req = readl(PMU_BUS_IDLE_REQ), ist = readl(PMU_BUS_IDLE_ST);
	int i;

	writel(con & ~PMU_PD_MASK, PMU_PWRDN_CON);
	for (i = 0; i < 10000 && (readl(PMU_PWRDN_ST) & PMU_PD_MASK); i++)
		udelay(1);
	writel(req & ~PMU_BUS_MASK, PMU_BUS_IDLE_REQ);
	for (i = 0; i < 10000 &&
	     ((readl(PMU_BUS_IDLE_ST) | readl(PMU_BUS_IDLE_ACK)) & PMU_BUS_MASK); i++)
		udelay(1);
	printf("PMU before: PWRDN_CON 0x%x ST 0x%x BUS_IDLE_REQ 0x%x ST 0x%x; after: ST 0x%x IDLE_ST 0x%x\n",
	       con, pst, req, ist, readl(PMU_PWRDN_ST), readl(PMU_BUS_IDLE_ST));
}

static void rk3399_reset_again_after_wdt_tsadc(struct rockchip_cru *cru)
{
	u32 st = readl(&cru->glb_rst_st);

	if (!(st & (FST_GLB_WDT_RST_ST | SND_GLB_WDT_RST_ST |
		    FST_GLB_TSADC_RST_ST | SND_GLB_TSADC_RST_ST)))
		return;

	printf("reset cause CRU_GLB_RST_ST=0x%x: global reset again, every power domain on\n",
	       st);
	rk3399_pmu_domains_on();
	/* the next pass reads this cause, not its own soft reset (F21) */
	writel(RK3399_RST_CAUSE_MAGIC | RK3399_RST_CAUSE_PENDING |
	       (st & RK3399_RST_CAUSE_MASK), RK3399_RST_CAUSE_REG);
	udelay(2000);	/* let the UART drain */
	/* W1C: the next boot sees only this reset's flag and goes on */
	writel(st, &cru->glb_rst_st);
	writel(0xfdb9, &cru->glb_srst_fst_value);
	while (1)
		;
}

static void rk3399_force_power_on_reset(void)
{
	const struct rockchip_cru *cru = rockchip_get_cru();
	ofnode node;
	struct gpio_desc sysreset_gpio;

	/*
	 * The RK3399 resets only 'almost all logic' (see also in the
	 * TRM "3.9.4 Global software reset"), when issuing a software
	 * reset. This may cause issues during boot-up for some
	 * configurations of the application software stack.
	 *
	 * To work around this, we test whether the last reset reason
	 * was a power-on reset and (if not) issue an overtemp-reset to
	 * reset the entire module.
	 *
	 * While this was previously fixed by modifying the various
	 * places that could generate a software reset (e.g. U-Boot's
	 * sysreset driver, the ATF or Linux), we now have it here to
	 * ensure that we no longer have to track this through the
	 * various components.
	 */
	if (cru->glb_rst_st == 0)
		return;

	if (!IS_ENABLED(CONFIG_TPL_GPIO)) {
		debug("%s: trying to force a power-on reset but no GPIO "
		      "support in TPL!\n", __func__);
		rk3399_reset_again_after_wdt_tsadc((struct rockchip_cru *)cru);
		return;
	}

	debug("%s: trying to force a power-on reset\n", __func__);

	node = ofnode_path("/config");
	if (!ofnode_valid(node)) {
		debug("%s: no /config node?\n", __func__);
		return;
	}

	if (gpio_request_by_name_nodev(node, "sysreset-gpio", 0,
				       &sysreset_gpio, GPIOD_IS_OUT)) {
		debug("%s: could not find a /config/sysreset-gpio\n", __func__);
		return;
	}

	dm_gpio_set_value(&sysreset_gpio, 1);
}

/*
 * TSADC hardware over-temperature shutdown from TPL on (rockchip B186):
 * before this, nothing protected the SoC until BL31 armed it, about a
 * second into each boot (F12), and a board reset by the TSADC came straight
 * back up hot (F11). Arm it to the CRU at 95 C with the sequence BL31's
 * soc_tsadc_tshut_arm_once() uses (Linux's probe, traced), then, if either
 * channel reads above TSADC_HOLD_CODE, wait here, idle, until it is below,
 * for at most TSADC_HOLD_MAX_S. BL31 arms it again later.
 */
#define RK3399_TSADC_BASE	0xff260000
#define TSADC_USER_CON		(RK3399_TSADC_BASE + 0x00)
#define TSADC_AUTO_CON		(RK3399_TSADC_BASE + 0x04)
#define TSADC_INT_EN		(RK3399_TSADC_BASE + 0x08)
#define TSADC_DATA(chn)		(RK3399_TSADC_BASE + 0x20 + (chn) * 4)
#define TSADC_COMP_SHUT(chn)	(RK3399_TSADC_BASE + 0x40 + (chn) * 4)
#define TSADC_HT_INT_DEBOUNCE	(RK3399_TSADC_BASE + 0x60)
#define TSADC_HT_TSHUT_DEBOUNCE	(RK3399_TSADC_BASE + 0x64)
#define TSADC_AUTO_PERIOD	(RK3399_TSADC_BASE + 0x68)
#define TSADC_AUTO_PERIOD_HT	(RK3399_TSADC_BASE + 0x6c)
#define TSADC_CODE_95C		0x279	/* rk3399_code_table */
#ifndef TSADC_HOLD_CODE
#define TSADC_HOLD_CODE		0x268	/* 85 C; a test build may lower it */
#endif
#ifndef TSADC_HOLD_MAX_S
#define TSADC_HOLD_MAX_S	600
#endif
#define TSADC_ARM_TRIES		3
/* channels further apart than this read falsely (the M0's F3 threshold) */
#ifndef TSADC_SPREAD_CODES
#define TSADC_SPREAD_CODES	64
#endif
#define GRF_SARADC_TESTBIT	(GRF_BASE + 0xe644)
#define GRF_TSADC_TESTBIT_L	(GRF_BASE + 0xe648)
#define GRF_TSADC_TESTBIT_H	(GRF_BASE + 0xe64c)

/*
 * The early arm (below) runs before the timer is set up, so its waits are
 * reads of a CRU register, which is always clocked. Each read through the
 * interconnect takes well over 40 ns, so 25 reads are at least 1 us; a
 * slower read only makes the wait longer, which the sequence tolerates.
 */
#define RK3399_CRU_BASE		0xff760000
#define TSADC_SPIN_READS_PER_US	25

static void rk3399_tsadc_wait_us(struct rockchip_cru *cru, unsigned int us,
				 bool early)
{
	unsigned int i;

	if (!early) {
		udelay(us);
		return;
	}
	for (i = 0; i < us * TSADC_SPIN_READS_PER_US; i++)
		readl(&cru->glb_rst_st);
}

/*
 * The CRU routes for the channels' first readings (B186 F31): both, unless
 * the two are more than TSADC_SPREAD_CODES apart. Both channels sit on one
 * die (healthy spread within 25 codes under GPU load), so such a pair has
 * one channel reading falsely; a false hot one at 95 C or more would trip at
 * every arm and loop the board through resets for ever. Then only the cooler
 * channel shuts down. A channel with no reading leaves both routed. The same
 * rule is in BL31, the M0 and Linux's driver.
 */
static u32 rk3399_tsadc_routes(void)
{
	u32 d0 = readl(TSADC_DATA(0)) & 0xfff, d1 = readl(TSADC_DATA(1)) & 0xfff;

	if (!d0 || !d1 || (d0 > d1 ? d0 - d1 : d1 - d0) <= TSADC_SPREAD_CODES)
		return BIT(8) | BIT(9);

	return d0 > d1 ? BIT(9) : BIT(8);
}

static void rk3399_tsadc_arm_once(struct rockchip_cru *cru, bool analog_off_first,
				  bool early, u32 shut_code)
{
	int chn;

	if (analog_off_first) {
		writel(0x10000 << 2, GRF_TSADC_TESTBIT_H);
		writel(0x10000 << 2, GRF_SARADC_TESTBIT);
		writel(0x10000 << 7, GRF_TSADC_TESTBIT_H);
		writel(0x10000 << 7, GRF_TSADC_TESTBIT_L);
		rk3399_tsadc_wait_us(cru, 100, early);
	}
	/* pclk_tsadc CLKGATE_CON(22)[13], clk_tsadc CLKGATE_CON(9)[10] open */
	writel(BIT(13 + 16), &cru->clkgate_con[22]);
	writel(BIT(10 + 16), &cru->clkgate_con[9]);
	/* clk_tsadc = xin24M / 32 = 750 kHz */
	writel((0x83ff << 16) | 0x1f, &cru->clksel_con[27]);
	/* SRST_TSADC (SOFTRST_CON14[8]) */
	writel(BIT(8 + 16) | BIT(8), &cru->softrst_con[14]);
	rk3399_tsadc_wait_us(cru, 20, early);
	writel(BIT(8 + 16), &cru->softrst_con[14]);
	/* the analog power sequence */
	writel(0x10001 << 7, GRF_TSADC_TESTBIT_L);
	writel(0x10001 << 7, GRF_TSADC_TESTBIT_H);
	rk3399_tsadc_wait_us(cru, 15, early);
	writel(0x10001 << 2, GRF_SARADC_TESTBIT);
	writel(0x10001 << 2, GRF_TSADC_TESTBIT_H);
	rk3399_tsadc_wait_us(cru, 90, early);
	writel(1875, TSADC_AUTO_PERIOD);	/* 2.5 ms at 750 kHz */
	writel(4, TSADC_HT_INT_DEBOUNCE);
	writel(1875, TSADC_AUTO_PERIOD_HT);
	writel(4, TSADC_HT_TSHUT_DEBOUNCE);
	writel(BIT(8), TSADC_AUTO_CON);		/* tshut polarity high, alone */
	writel(0, TSADC_INT_EN);		/* routes once there is a reading */
	for (chn = 0; chn < 2; chn++) {
		writel(shut_code, TSADC_COMP_SHUT(chn));
		setbits_le32(TSADC_AUTO_CON, BIT(4 + chn));
	}
	setbits_le32(TSADC_AUTO_CON, BIT(0) | BIT(1));	/* enable, Q_SEL */
	rk3399_tsadc_wait_us(cru, 10000, early);
	writel(rk3399_tsadc_routes(), TSADC_INT_EN);
}

/* Code the early arm shuts down at; a test build may lower it */
#ifndef TSADC_CODE_SHUT_EARLY
#define TSADC_CODE_SHUT_EARLY	TSADC_CODE_95C
#endif

/*
 * Both channels converting (B186 F28): the shutdown watches the CPU and the
 * GPU channel, and the hold reads the hotter of the two, so an arm is
 * retried until both have a reading, not channel 0 alone.
 */
static bool rk3399_tsadc_converting(void)
{
	return readl(TSADC_DATA(0)) && readl(TSADC_DATA(1));
}

/*
 * B186 F25: a TPL that finds the BootROM download flag hands the board back
 * to the BootROM before tpl_board_init() runs, so the BootROM's USB download
 * mode waited unarmed for as long as nobody came. Arm the TSADC first, with
 * the same sequence and the same three tries, timer-free and silent: this
 * runs from save_boot_params(), before the console, the timer and DM.
 * Nothing in the BootROM's download mode touches the TSADC.
 *
 * Route the TSADC (and the watchdog) to the first global reset, the value
 * DRAM init writes on a normal boot (sdram_rk3399.c), so a trip here resets
 * the PMU and its M0 too (F19), with the barrier F20 needs before the next
 * CRU write.
 */
void rk3399_tsadc_arm_early(void)
{
	struct rockchip_cru *cru = (struct rockchip_cru *)RK3399_CRU_BASE;
	unsigned int tries = 0;

	clrsetbits_le32(&cru->glb_rst_con, 0x3, 0x3);
	dsb();
	do {
		rk3399_tsadc_arm_once(cru, tries != 0, true, TSADC_CODE_SHUT_EARLY);
		tries++;
	} while (!rk3399_tsadc_converting() && tries < TSADC_ARM_TRIES);
}

static void rk3399_tsadc_tpl(bool thermal)
{
	struct rockchip_cru *cru = (struct rockchip_cru *)rockchip_get_cru();
	unsigned int code, code1, tries = 0, s;

	do {
		rk3399_tsadc_arm_once(cru, tries != 0, false, TSADC_CODE_95C);
		tries++;
	} while (!rk3399_tsadc_converting() && tries < TSADC_ARM_TRIES);
	if (!rk3399_tsadc_converting())
		printf("TSADC: a channel is not converting after %u tries (code 0x%x/0x%x): the shutdown may not fire and the hold cannot see heat\n",
		       tries, readl(TSADC_DATA(0)), readl(TSADC_DATA(1)));
	if ((readl(TSADC_INT_EN) & (BIT(8) | BIT(9))) != (BIT(8) | BIT(9)))
		printf("TSADC: channels 0x%x/0x%x are more than %u codes apart: only channel %u shuts down (B186 F31)\n",
		       readl(TSADC_DATA(0)), readl(TSADC_DATA(1)), TSADC_SPREAD_CODES,
		       (readl(TSADC_INT_EN) & BIT(8)) ? 0 : 1);

	/*
	 * After a thermal reset U-Boot proper powers the board off (B186
	 * F22), and off, every rail is down: no hold, which would only keep
	 * the board on, at the CPU and GPU voltages Linux last set, for up to
	 * TSADC_HOLD_MAX_S (F32). A SoC still at 95 C trips the armed
	 * shutdown again and cools, idle, until the boot gets through.
	 */
	if (thermal && IS_ENABLED(CONFIG_ROCKCHIP_THERMAL_RESET_POWER_OFF)) {
		printf("TSADC: shutdown armed at 95 C, try %u, code 0x%x/0x%x, thermal reset: no hold, U-Boot powers off\n",
		       tries, readl(TSADC_DATA(0)), readl(TSADC_DATA(1)));
		return;
	}

	for (s = 0; ; s++) {
		/*
		 * The hotter channel (a higher code is hotter on the RK3399),
		 * unless the two are more than TSADC_SPREAD_CODES apart: both
		 * sit on one die (healthy spread measured within 25 codes under
		 * GPU load), so such a pair has one channel reading falsely, and
		 * holding on a false hot one would hold every boot for
		 * TSADC_HOLD_MAX_S (B186 F31). Then hold on the cooler one.
		 */
		code = readl(TSADC_DATA(0)) & 0xfff;
		code1 = readl(TSADC_DATA(1)) & 0xfff;
		if ((code > code1 ? code - code1 : code1 - code) > TSADC_SPREAD_CODES) {
			if (s == 0)
				printf("TSADC: channels 0x%x/0x%x are more than %u codes apart: holding on the cooler one\n",
				       code, code1, TSADC_SPREAD_CODES);
			if (code1 < code)
				code = code1;
		} else if (code1 > code) {
			code = code1;
		}
		if (code <= TSADC_HOLD_CODE || s >= TSADC_HOLD_MAX_S)
			break;
		if (s % 10 == 0)
			printf("TSADC: code 0x%x is above 85 C; holding until it cools (%u s)\n",
			       code, s);
		mdelay(1000);
	}
	printf("TSADC: shutdown armed at 95 C, try %u, code 0x%x/0x%x%s\n", tries,
	       readl(TSADC_DATA(0)), readl(TSADC_DATA(1)),
	       s ? (s >= TSADC_HOLD_MAX_S ? ", hold timed out" : ", cooled") : "");
}

/*
 * B186 F21: keep the reset cause of this boot in RK3399_RST_CAUSE_REG for
 * BL31 and U-Boot proper. A first global soft reset right after TPL marked
 * its own reset-again as pending is that reset-again, so the cause it
 * recorded stands; anything else is this boot's cause.
 */
static u32 rk3399_rst_cause_keep(void)
{
	const struct rockchip_cru *cru = rockchip_get_cru();
	u32 st = cru->glb_rst_st & RK3399_RST_CAUSE_MASK;
	u32 w = readl(RK3399_RST_CAUSE_REG);

	if ((w & RK3399_RST_CAUSE_MAGIC_MASK) == RK3399_RST_CAUSE_MAGIC &&
	    (w & RK3399_RST_CAUSE_PENDING) && st == FST_GLB_RST_ST)
		st = w & RK3399_RST_CAUSE_MASK;
	writel(RK3399_RST_CAUSE_MAGIC | st, RK3399_RST_CAUSE_REG);
	printf("reset cause kept: 0x%x\n", st);

	return st;
}

void tpl_board_init(void)
{
	u32 st = rk3399_rst_cause_keep();

	rk3399_tsadc_tpl(st & (FST_GLB_TSADC_RST_ST | SND_GLB_TSADC_RST_ST));
	rk3399_force_power_on_reset();
}
# else
void __weak led_setup(void)
{
}

void spl_board_init(void)
{
	led_setup();
	/* On a MaskROM boot: the USB flash session, then the boot goes on */
	if (CONFIG_IS_ENABLED(USBFLASH))
		rockchip_spl_usbflash();
}
#endif
#endif

#if !defined(CONFIG_XPL_BUILD) && IS_ENABLED(CONFIG_DISPLAY_CPUINFO)
/*
 * B186 F21: pass the reset cause TPL kept to the OS, as /chosen
 * "u-boot,reset-cause" (POR, RST, THERMAL, WDOG), the string U-Boot prints.
 * A failure here leaves the property out and does not stop the boot.
 */
int ft_system_setup(void *blob, struct bd_info *bd)
{
	const char *cause = get_reset_cause();
	int node;

	if (!cause)
		return 0;
	node = fdt_find_or_add_subnode(blob, 0, "chosen");
	if (node >= 0)
		fdt_setprop_string(blob, node, "u-boot,reset-cause", cause);
	return 0;
}
#endif
