// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 *
 * AML-S905X-CC-V3 (Das Potato). The board boots the open chain only, whose
 * TPL trains the DDR4 at the tREFI the board needs, so unlike the CC-V2 there
 * is no vendor-chain refresh fix-up here.
 */

#include <fdt_support.h>
#include <init.h>
#include <linux/libfdt.h>
#include <asm/arch/boot.h>
#include <asm/arch/gx.h>
#include "../common/board-meson.h"

#if IS_ENABLED(CONFIG_SET_DFU_ALT_INFO)
void set_dfu_alt_info(char *interface, char *devstr){
	meson_set_dfu_alt_info(interface, devstr);
}
#endif

int misc_init_r(void)
{
	meson_gxl_misc_init_r();

	return 0;
}

#if IS_ENABLED(CONFIG_OF_BOARD_SETUP)
/* &sd_emmc_c and &spifc in meson-gx.dtsi */
#define CC_V3_EMMC_NODE	"/soc/apb@d0000000/mmc@74000"
#define CC_V3_NOR_NODE	"/soc/bus@c1100000/spi@8c80"

/*
 * A V3 carries the SPI NOR or the eMMC, never both, and its device tree
 * describes both (meson-gxl-s905x-libretech-cc-v3.dtsi). The boot source names
 * the build: a board that booted from NOR has no eMMC and the reverse, so the
 * OS is not left to probe a part that is absent. Booted from SD or USB, the
 * build is not known and both stay as described.
 */
int meson_ft_board_setup(void *blob, struct bd_info *bd)
{
	const char *absent, *build;
	int node, ret;

	switch (meson_get_boot_device()) {
	case BOOT_DEVICE_SPI:
		absent = CC_V3_EMMC_NODE;
		build = "NOR";
		break;
	case BOOT_DEVICE_EMMC:
		absent = CC_V3_NOR_NODE;
		build = "eMMC";
		break;
	default:
		return 0;
	}

	node = fdt_path_offset(blob, absent);
	if (node < 0) {
		printf("cc-v3: %s build, %s not in the device tree: %s\n", build,
		       absent, fdt_strerror(node));
		return 0;
	}
	ret = fdt_set_node_status(blob, node, FDT_STATUS_DISABLED);
	if (ret)
		printf("cc-v3: %s build, %s not disabled: %s\n", build, absent,
		       fdt_strerror(ret));

	return 0;
}
#endif
