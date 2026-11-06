// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * (C) Copyright 2019 Amarula Solutions(India)
 * Author: Jagan Teki <jagan@amarulasolutions.com>
 */

#include <env.h>
#include <init.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/cru.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/io.h>
#include <linux/err.h>

char *get_reset_cause(void)
{
	struct rockchip_cru *cru = rockchip_get_cru();
	u32 st;

	if (IS_ERR(cru))
		return NULL;

	st = cru->glb_rst_st;
#if defined(CONFIG_ROCKCHIP_RK3399)
	{
		/* the cause TPL kept; the CRU's own copy is cleared by now */
		u32 w = readl(RK3399_RST_CAUSE_REG);

		if ((w & RK3399_RST_CAUSE_MAGIC_MASK) == RK3399_RST_CAUSE_MAGIC)
			st = w & RK3399_RST_CAUSE_MASK;
	}
#endif

	/*
	 * Several bits can be set at once (a TSADC reset on top of a soft
	 * one reads 0x5): the thermal and watchdog causes come first.
	 */
	if (st & (FST_GLB_TSADC_RST_ST | SND_GLB_TSADC_RST_ST))
		return "THERMAL";
	if (st & (FST_GLB_WDT_RST_ST | SND_GLB_WDT_RST_ST))
		return "WDOG";
	if (st & (FST_GLB_RST_ST | SND_GLB_RST_ST))
		return "RST";
	if (st == GLB_POR_RST)
		return "POR";
	return "unknown reset";
}

#if IS_ENABLED(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo(void)
{
	char *cause = get_reset_cause();

	printf("SoC: Rockchip %s\n", CONFIG_SYS_SOC);
	printf("Reset cause: %s\n", cause);

	/**
	 * reset_reason env is used by rk3288, due to special use case
	 * to figure it the boot behavior. so keep this as it is.
	 */
	env_set("reset_reason", cause);

	/* TODO print operating temparature and clock */

	return 0;
}
#endif
