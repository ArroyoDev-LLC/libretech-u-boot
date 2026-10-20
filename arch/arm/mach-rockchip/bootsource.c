// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * The BootROM's boot-source identifier, mapped onto the platform vocabulary.
 *
 * The BootROM leaves the medium it loaded the first stage from in
 * BROM_BOOTSOURCE_ID_ADDR (IRAM + 0x10). SPL reads it once (that word is
 * overwritten as soon as a BL31 SRAM segment lands on IRAM), prints it,
 * hands it to U-Boot proper as /chosen/libre-computer,boot-source, and
 * U-Boot proper publishes it as the `bootsource` environment variable.
 * The spelling -- banner name and lower-case token -- is boot/bootsource.c's,
 * shared with every other SoC family; this file only maps the Rockchip
 * identifier onto it, so the SPL banner, the DT property and the env
 * variable can never disagree with each other or with another board.
 */

#include <bootsource.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <asm/arch-rockchip/bootrom.h>

static const enum bootsource rockchip_bootsources[BROM_LAST_BOOTSOURCE + 1] = {
	[BROM_BOOTSOURCE_UNKNOWN]	= BOOTSOURCE_UNKNOWN,
	[BROM_BOOTSOURCE_NAND]		= BOOTSOURCE_NAND,
	[BROM_BOOTSOURCE_EMMC]		= BOOTSOURCE_EMMC,
	[BROM_BOOTSOURCE_SPINOR]	= BOOTSOURCE_SPI,
	[BROM_BOOTSOURCE_SPINAND]	= BOOTSOURCE_NAND,
	[BROM_BOOTSOURCE_SD]		= BOOTSOURCE_SD,
	[BROM_BOOTSOURCE_UFS]		= BOOTSOURCE_UFS,
	[BROM_BOOTSOURCE_I2C]		= BOOTSOURCE_UNKNOWN,
	[BROM_BOOTSOURCE_SPI]		= BOOTSOURCE_SPI,
	[BROM_BOOTSOURCE_USB]		= BOOTSOURCE_USB,
};

enum bootsource rockchip_bootsource(u32 id)
{
	if (id < ARRAY_SIZE(rockchip_bootsources))
		return rockchip_bootsources[id];

	return BOOTSOURCE_UNKNOWN;
}

const char *rockchip_bootsource_name(u32 id)
{
	return bootsource_name(rockchip_bootsource(id));
}

const char *rockchip_bootsource_token(u32 id)
{
	if (rockchip_bootsource(id) == BOOTSOURCE_UNKNOWN)
		return NULL;

	return bootsource_token(rockchip_bootsource(id));
}
