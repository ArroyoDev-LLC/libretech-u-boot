// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2025 Da Xue <da@libre.computer>
 *
 * Sunxi-specific Libre Computer board support.
 * Provides EVT_SETTINGS_R handler for boot device detection,
 * splash partition setup, and boot.ini loading.
 * Provides EFI capsule update_info for on-disk firmware update.
 *
 * Note: mmc_get_env_dev() is handled by modifying board/sunxi/board.c
 * directly since sunxi board code is monolithic and cannot be replaced
 * by a vendor board directory without losing essential platform functions.
 */

#include <bootsource.h>
#include <dm.h>
#include <env.h>
#include <ini.h>
#include <linux/ctype.h>
#include <mmc.h>
#include <event.h>
#include <spl.h>
#include <vsprintf.h>
#include <asm/arch/spl.h>
#include "board.h"

#if IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT)
#include <efi_loader.h>
#include <malloc.h>
#include <dm/ofnode.h>

static u16 *sunxi_fw_name;

struct efi_fw_image fw_images[] = {
	{
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "mmc 0=u-boot-bin raw 0x10 0x7f0 mmcpart 0",
	.num_images = ARRAY_SIZE(fw_images),
	.images = fw_images,
};

/*
 * Build fw_name from DT compatible: strip vendor prefix, uppercase,
 * replace hyphens/commas with underscores, append _BOOT.
 * e.g. "libretech,all-h3-cc-h5" -> "ALL_H3_CC_H5_BOOT"
 */
static void sunxi_capsule_init(void)
{
	const char *compat, *name;
	char buf[64];
	int i, len;

	compat = ofnode_read_string(ofnode_root(), "compatible");
	if (!compat)
		return;

	/* skip vendor prefix (everything up to and including comma) */
	name = strchr(compat, ',');
	name = name ? name + 1 : compat;

	len = strlen(name);
	if (len + 6 > sizeof(buf)) /* _BOOT\0 */
		return;

	for (i = 0; i < len; i++)
		buf[i] = (name[i] == '-') ? '_' : toupper(name[i]);
	memcpy(buf + len, "_BOOT", 6);

	sunxi_fw_name = calloc(strlen(buf) + 1, sizeof(u16));
	if (!sunxi_fw_name)
		return;

	for (i = 0; buf[i]; i++)
		sunxi_fw_name[i] = buf[i];

	fw_images[0].fw_name = sunxi_fw_name;
}
#endif

/*
 * Where the environment and boot.ini live, by boot source (bootloader policy
 * §4 "Boot source variables"): SD -> mmc1, eMMC -> mmc0 (policy §4 "MMC
 * numbering": eMMC first), SPI NOR -> the first MMC with a medium, USB (FEL)
 * -> no boot.ini: a stage `sunxi-fel` sent over the cable runs on its
 * compiled-in defaults, not on a card it did not boot from. `bootdevice` is
 * still set on every source (splash + the block commands' default device).
 */
static int settings_r(void)
{
	enum bootsource src = sunxi_bootsource();
	int bootdevice_num = 0;
	bool ini_from_mmc = false;
	char *bootdevice;

#if IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT)
	sunxi_capsule_init();
#endif

	switch (src) {
	case BOOTSOURCE_SD:
		bootdevice_num = 1;
		ini_from_mmc = true;
		break;
	case BOOTSOURCE_EMMC:
		bootdevice_num = 0;
		ini_from_mmc = true;
		break;
	case BOOTSOURCE_SPI:
		if (!mmc_is_present(bootdevice_num))
			bootdevice_num = mmc_is_present(!bootdevice_num);
		ini_from_mmc = true;
		break;
	default:
		break;
	}

	bootdevice = simple_itoa(bootdevice_num);

	bootsource_publish(src, bootdevice_num);
#ifdef CONFIG_SPLASH_SOURCE
	env_set("splashdevpart", bootdevice);
#endif
#ifdef CONFIG_CMD_INI
	if (ini_from_mmc)
		env_ini_load("mmc", bootdevice);
#endif
	return 0;
}
EVENT_SPY_SIMPLE(EVT_SETTINGS_R, settings_r);
