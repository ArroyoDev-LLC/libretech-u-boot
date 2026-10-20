// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 MediaTek Inc.
 * Author: Macpaul Lin <macpaul.lin@mediatek.com>
 *
 * Forward-ported from the MediaTek 2022.10 BSP onto v2026.04/lc-master.
 * The BSP board file carried the vendor-only A/B boot machinery (iot_ab.h),
 * the EFI capsule-update image table, mediatek_capsule_update_board_setup()
 * and an SCP-SRAM enable poke -- none of which exist in / are required by
 * our LBS boot chain. Stripped to a minimal bringup board: fixed
 * model/hostname and an empty board_init(), mirroring the in-tree mt8365
 * (Genio 700) sibling and the genio-720/mt8391 port. The mm_region map
 * lives in arch/arm/mach-mediatek/mt8188/init.c.
 */

#include <dm.h>
#include <env.h>
#include <fdt_support.h>
#include <log.h>
#include <linux/sizes.h>

static const char *const board_full_name = "MediaTek Genio 510 EVK";

#if (IS_ENABLED(CONFIG_BOARD_LATE_INIT))
int board_late_init(void)
{
	env_set("hostname", "genio-510-evk");
	return 0;
}
#endif

#if (IS_ENABLED(CONFIG_OF_BOARD_SETUP))
int ft_board_setup(void *blob, struct bd_info *bd)
{
	do_fixup_by_path_string(blob, "/", "model", board_full_name);
	return 0;
}
#endif

int board_init(void)
{
	return 0;
}
