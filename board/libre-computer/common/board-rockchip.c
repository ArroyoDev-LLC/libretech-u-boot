// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2025 Da Xue <da@libre.computer>
 *
 * Rockchip-specific Libre Computer board support.
 * Provides EVT_SETTINGS_R handler for boot device detection,
 * splash partition setup, and boot.ini loading.
 */

#include <bootsource.h>
#include <dm.h>
#include <env.h>
#include <env_internal.h>
#include <ini.h>
#include <mmc.h>
#include <event.h>
#include <vsprintf.h>
#include <asm/arch-rockchip/bootrom.h>
#include "board.h"

/*
 * The BootROM's boot source, as SPL wrote it into /chosen (the platform
 * token from include/bootsource.h: sd, emmc, spi, usb, ...). NULL when SPL
 * did not say -- an older SPL, or a DT handed over by something else.
 */
static const char *rockchip_boot_source(void)
{
	return ofnode_read_chosen_string(BOOTSOURCE_PROP);
}

/*
 * Where the environment lives, by boot source.
 *
 * SD / eMMC / SPI NOR: the default ladder -- a FAT file on the boot MMC
 * device (bootloader policy §4 "Environment storage"), which
 * mmc_get_env_dev() resolves to the device the FIT came from, or on a NOR
 * boot to the first MMC that has a medium.
 *
 * USB: the stage was sent over the cable by a host (MaskROM download); it
 * is a recovery or test boot that must run on its own compiled-in defaults,
 * not on whatever `uboot.env` sits on a card that happens to be inserted.
 * ENVL_NOWHERE also makes `saveenv` refuse, so a test session cannot leave
 * its variables behind on the medium.
 */
#if IS_ENABLED(CONFIG_ENV_IS_NOWHERE)
enum env_location env_get_location(enum env_operation op, int prio)
{
	const char *src = rockchip_boot_source();

	if (src && !strcmp(src, "usb"))
		return prio ? ENVL_UNKNOWN : ENVL_NOWHERE;

	return arch_env_get_location(op, prio);
}
#endif

static int settings_r(void)
{
	const char *boot_device;
	const char *boot_source = rockchip_boot_source();
	struct udevice *dev;
	/*
	 * env/Kconfig only defines CONFIG_ENV_MMC_DEVICE_INDEX when the
	 * environment lives somewhere that needs an MMC device index
	 * (ENV_IS_IN_MMC / _FAT / _EXT4, or one of the four unrelated symbols
	 * that reuse it). roc-rk3399-pc-mezzanine_defconfig keeps its
	 * environment in SPI NOR, so on that board the symbol does not exist
	 * and this was an undeclared identifier -- a hard build break, not a
	 * warning. Guard it exactly as board-meson.c already does.
	 */
#ifdef CONFIG_ENV_MMC_DEVICE_INDEX
	int bootdevice_num = CONFIG_ENV_MMC_DEVICE_INDEX;
#else
	int bootdevice_num = 0;
#endif
	char *bootdevice;
	/* Is there an MMC device whose boot.ini we obey? See the table below. */
	bool booted_from_mmc = false;

	/*
	 * Whether a boot.ini is read, and from which MMC device, by boot
	 * source (bootloader policy §4 "`boot.ini` on SPI-NOR boot",
	 * §4 "Environment storage"):
	 *
	 *   SD / eMMC  the FIT's own MMC device (u-boot,spl-boot-device);
	 *   SPI NOR    the first MMC device with a medium -- the NOR has no
	 *              filesystem to carry a boot.ini, and policy says a
	 *              NOR-booted board still reads one, from its eMMC (or SD)
	 *              EFI partition; mmc_get_env_dev() resolves that device
	 *              and the FAT env follows it, so env and boot.ini agree;
	 *   USB        none -- a stage sent over the cable takes no boot.ini
	 *              off a card it did not boot from, the rule
	 *              env_get_location() applies to the environment;
	 *   unknown    (no SPL hand-over at all) the historical default:
	 *              the default index, boot.ini from it.
	 *
	 * Until 2026-09-17 the NOR case loaded no boot.ini ("no filesystem in
	 * the SPI environment"), which was this file disagreeing with the
	 * policy and with board-meson.c, whose BOOT_DEVICE_SPI arm has always
	 * picked the first present MMC.
	 */
	boot_device = ofnode_read_chosen_string("u-boot,spl-boot-device");
	if (boot_device) {
		if (!uclass_get_device_by_ofnode(UCLASS_MMC,
				ofnode_path(boot_device), &dev)) {
			bootdevice_num = dev->seq_;
			booted_from_mmc = true;
		} else if (boot_source && !strcmp(boot_source, "spi")) {
			bootdevice_num = mmc_get_env_dev();
			booted_from_mmc = true;
		}
	}

	bootdevice = simple_itoa(bootdevice_num);

	/*
	 * Publish the BootROM's boot source (boot/bootsource.c): `bootsource`
	 * is the token SPL wrote (sd / emmc / spi / usb -- "unknown" when SPL
	 * did not say), `bootdevice` the MMC index the environment and
	 * boot.ini are read from, one "Boot source:" banner line, and the
	 * same token into the kernel's /chosen at boot. Printed here rather
	 * than in the board banner because the value is not known until the
	 * SPL-handed DT is read.
	 */
	bootsource_publish(bootsource_from_token(boot_source), bootdevice_num);
#ifdef CONFIG_SPLASH_SOURCE
	env_set("splashdevpart", bootdevice);
#endif
#ifdef CONFIG_CMD_INI
	if (booted_from_mmc || (!boot_device && !boot_source))
		env_ini_load("mmc", bootdevice);
#endif
	return 0;
}
EVENT_SPY_SIMPLE(EVT_SETTINGS_R, settings_r);
