// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <dm.h>
#include <env.h>
#include <init.h>
#include <net.h>
#include <efi_loader.h>
#include <asm/io.h>
#include <asm/arch/eth.h>
#include "../common/board-meson.h"

#if IS_ENABLED(CONFIG_SET_DFU_ALT_INFO)
void set_dfu_alt_info(char *interface, char *devstr){
	meson_set_dfu_alt_info(interface, devstr);
}
#endif
int misc_init_r(void)
{
	meson_generate_serial_ethaddr();
	meson_setup_serial_env();
	/* USB recovery boot: compiled-in env only, no stored NOR env overrides */
	if (!IS_ENABLED(CONFIG_BOARD_CONFIG_NOR) || !meson_usb_env_nowhere())
		board_config_env_import();

	return 0;
}
