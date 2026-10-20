// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2026 Libre Computer Project
 *
 * Libre Computer MED-MT88-MX, both SKUs of the one board: G5 (MT8370) and G7
 * (MT8390), the same PCB on the MT8188 die with a different efuse. One binary
 * serves both: the control DT is chosen at runtime from the efuse part ID
 * (MTK_SIP_PLAT_BINFO, answered by the open and the vendor BL31), and the
 * hostname and the kernel DT name follow it. Not the MediaTek Genio 510 / 700
 * EVK -- do not compile board/mediatek/genio-510-evk/mt8188_evk.c for this
 * product. No BSP A/B, capsule, or SCP-SRAM poke.
 */

#include <dm.h>
#include <env.h>
#include <fdt_support.h>
#include <log.h>
#include <linux/errno.h>
#include <linux/string.h>

#include "../../../arch/arm/mach-mediatek/cpu.h"

#define MED_MT88_MX_PART_G7	0x8390

/* The G7 only on an efuse that says MT8390; anything else, read failure
 * included, is the G5 -- the SKU every BL31 build before the part-ID SMC
 * already booted with this binary. */
static bool med_mt88_mx_is_g7(void)
{
	return mediatek_sip_part_name() == MED_MT88_MX_PART_G7;
}

int board_fit_config_name_match(const char *name)
{
	const char *want = med_mt88_mx_is_g7() ? "mt8390-med-mt88-mx-g7" :
						 "mt8370-med-mt88-mx-g5";

	return strstr(name, want) ? 0 : -EINVAL;
}

int board_init(void)
{
	return 0;
}

#if (IS_ENABLED(CONFIG_BOARD_LATE_INIT))
int board_late_init(void)
{
	bool g7 = med_mt88_mx_is_g7();
	int ret;

	printf("Board: MED-MT88-MX-%s (part 0x%x)\n", g7 ? "G7" : "G5",
	       mediatek_sip_part_name());
	ret = env_set("hostname", g7 ? "med-mt88-mx-g7" : "med-mt88-mx-g5");
	if (ret)
		return ret;
	/* A saved environment keeps its own choice. */
	if (g7 && !strcmp(env_get("fdtfile") ?: "", CONFIG_DEFAULT_FDT_FILE ".dtb"))
		ret = env_set("fdtfile", "mt8390-med-mt88-mx-g7.dtb");
	return ret;
}
#endif

#if (IS_ENABLED(CONFIG_OF_BOARD_SETUP))
int ft_board_setup(void *blob, struct bd_info *bd)
{
	do_fixup_by_path_string(blob, "/", "model",
				med_mt88_mx_is_g7() ?
				"Libre Computer MED-MT88-MX-G7" :
				"Libre Computer MED-MT88-MX-G5");
	return 0;
}
#endif
