// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2025 Da Xue <da@libre.computer>
 *
 * Rockchip-specific Libre Computer board support.
 * Provides EVT_SETTINGS_R handler for boot device detection,
 * splash partition setup, and boot.ini loading, and the DFU alternates
 * by interface (set_dfu_alt_info()).
 */

#include <bootsource.h>
#include <dm.h>
#include <env.h>
#include <env_internal.h>
#include <ini.h>
#include <mmc.h>
#include <event.h>
#include <vsprintf.h>
#include <malloc.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/usbflash-alts.h>
#include <linux/sizes.h>
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
 *
 * Every other boot source keeps the default ladder with ENVL_NOWHERE taken
 * out, as board-meson.c does: left in, NOWHERE is the last entry of
 * env_locations[], loads "OK" when the FAT load fails, and leaves
 * env_load_prio where `saveenv` is "not possible".
 */
#if IS_ENABLED(CONFIG_ENV_IS_NOWHERE)
enum env_location env_get_location(enum env_operation op, int prio)
{
	const char *src = rockchip_boot_source();
	enum env_location loc;

	if (src && !strcmp(src, "usb"))
		return prio ? ENVL_UNKNOWN : ENVL_NOWHERE;

	loc = arch_env_get_location(op, prio);

	return loc == ENVL_NOWHERE ? ENVL_UNKNOWN : loc;
}
#endif

/*
 * The OTG controller as a USB device in U-Boot on a USB boot.
 *
 * A USB boot came over the OTG port from a host, and the flash session, the
 * ACM console and the app's commands need a gadget on that port. A board DTS
 * that sets the controller to `host` (ROC-RK3328-CC: rk3328-roc.dtsi, its OTG
 * controller is wired to a USB-A port) makes dwc2_udc_otg_bind() refuse it,
 * and DM then binds the host driver. So the role is set here, in the live
 * tree only, before DM binds anything: the flat control FDT the kernel may be
 * given (fdtcontroladdr) keeps the board's `host`, and env usb_otg still sets
 * the kernel's role (boot/usb_otg_role.c).
 *
 * Nothing changes on a boot from any medium, or where the controller is not
 * `host` (ROC-RK3399-PC: its Type-C DWC3 is `otg` and binds a gadget as it
 * is). The saved environment is not loaded yet at this point (initr_env runs
 * after initr_dm), so the boot source alone decides.
 */
static int rockchip_otg_usb_boot_role(void)
{
	const char *src = rockchip_boot_source();
	const char *mode;
	ofnode node;

	if (!src || strcmp(src, "usb"))
		return 0;

	node = ofnode_get_aliases_node("usb-otg");
	if (!ofnode_valid(node))
		return 0;

	mode = ofnode_read_string(node, "dr_mode");
	if (!mode || strcmp(mode, "host"))
		return 0;

	return ofnode_write_string(node, "dr_mode", "peripheral");
}
EVENT_SPY_SIMPLE(EVT_OF_LIVE_BUILT, rockchip_otg_usb_boot_role);

#if IS_ENABLED(CONFIG_SET_DFU_ALT_INFO)
/*
 * U-Boot's DFU alternates, under the names the SPL flash session and the Meson
 * boards use (<asm/arch-rockchip/usbflash-alts.h>), so `dfu` and the boot
 * menu's "USB DFU Mode" offer a fixed list instead of whatever dfu_alt_info
 * holds: the eMMC (mmc 0) and SD (mmc 1) alternates of the media that answer,
 * and the RAM slots where the SoC config carries them (ROCKCHIP_DFU_RAM_ALTS).
 * The console gadget's resident DFU (USB_FUNCTION_ACM_DFU_ALT, e.g.
 * "virt 0&virt 1") gets its lists here too:
 *   virt 0   status  (lc-status.c, ROCKCHIP_LC_STATUS)
 *   virt 1   bootrom (the USB reset-to-MaskROM request, f_dfu.c, CMD_BOOTROM)
 *   sf       u-boot-bin, config, fit: the canonical NOR layout on a 16 MiB
 *            part (the SPL session sizes it from the part it probes)
 *
 * `dfu 0 <interface> [<dev>]` gets that interface's list from the hook. A
 * call with no interface leaves dfu_alt_info alone: the EFI capsule writer
 * (efi_firmware.c) sets dfu_alt_info to its update_info.dfu_string and then
 * writes by alt number through dfu_write_by_alt(..., NULL, NULL), which calls
 * this hook, and a list rebuilt here would move its alt 0 to another medium.
 * The whole list is instead the default of dfu_alt_info, set at boot
 * (rockchip_dfu_env()), so a bare `dfu 0` and the menu use it.
 */
#define ROCKCHIP_DFU_ALT_LEN	1024

#ifdef ROCKCHIP_DFU_RAM_ALTS
#define ROCKCHIP_DFU_RAM	"ram 0=" ROCKCHIP_DFU_RAM_ALTS
#else
#define ROCKCHIP_DFU_RAM	""
#endif

/* The alternates of every interface, '&'-separated, into @buf */
static void rockchip_dfu_alts(char *buf, int len, bool ram)
{
	*buf = '\0';
	if (mmc_is_present(0))
		strlcat(buf, "mmc 0=" ROCKCHIP_DFU_MMC_EMMC_ALTS "&", len);
	if (mmc_is_present(1))
		strlcat(buf, "mmc 1=" ROCKCHIP_DFU_MMC_SD_ALTS "&", len);
	if (ram)
		strlcat(buf, ROCKCHIP_DFU_RAM, len);
	/* a list ending in the separator names an empty interface */
	if (*buf && buf[strlen(buf) - 1] == '&')
		buf[strlen(buf) - 1] = '\0';
}

void set_dfu_alt_info(char *interface, char *devstr)
{
	char buf[128];
	char *alts;

	if (!interface)
		return;
	if (!strcmp(interface, "virt")) {
		if (IS_ENABLED(CONFIG_CMD_BOOTROM) && devstr && !strcmp(devstr, "1"))
			env_set("dfu_alt_info", "bootrom");
		else if (IS_ENABLED(CONFIG_ROCKCHIP_LC_STATUS))
			env_set("dfu_alt_info", "status");
		return;
	}
	if (!strcmp(interface, "sf")) {
		snprintf(buf, sizeof(buf),
			 "u-boot-bin raw 0x0 0x%x;config raw 0x%x 0x%x;fit raw 0x%x 0x%x",
			 ROCKCHIP_DFU_SF_UBOOT_SIZE, SZ_16M - ROCKCHIP_DFU_SF_CONFIG_SIZE,
			 ROCKCHIP_DFU_SF_CONFIG_SIZE, ROCKCHIP_DFU_SF_FIT_OFFSET,
			 ROCKCHIP_DFU_SF_FIT_SIZE);
		env_set("dfu_alt_info", buf);
		return;
	}
	if (!strcmp(interface, "ram")) {
#ifdef ROCKCHIP_DFU_RAM_ALTS
		env_set("dfu_alt_info", ROCKCHIP_DFU_RAM_ALTS);
#endif
		return;
	}
	if (strcmp(interface, "mmc"))
		return;
	if (devstr) {
		if (!strcmp(devstr, "0") && mmc_is_present(0))
			env_set("dfu_alt_info", ROCKCHIP_DFU_MMC_EMMC_ALTS);
		else if (!strcmp(devstr, "1") && mmc_is_present(1))
			env_set("dfu_alt_info", ROCKCHIP_DFU_MMC_SD_ALTS);
		return;
	}
	alts = calloc(1, ROCKCHIP_DFU_ALT_LEN);
	if (!alts)
		return;
	rockchip_dfu_alts(alts, ROCKCHIP_DFU_ALT_LEN, false);
	if (*alts)
		env_set("dfu_alt_info", alts);
	free(alts);
}

/* dfu_alt_info's default, where no saved environment set one */
static void rockchip_dfu_env(void)
{
	char *alts;

	if (env_get("dfu_alt_info"))
		return;
	alts = calloc(1, ROCKCHIP_DFU_ALT_LEN);
	if (!alts)
		return;
	rockchip_dfu_alts(alts, ROCKCHIP_DFU_ALT_LEN, true);
	if (*alts)
		env_set("dfu_alt_info", alts);
	free(alts);
}
#else
static void rockchip_dfu_env(void)
{
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
	rockchip_dfu_env();
	return 0;
}
EVENT_SPY_SIMPLE(EVT_SETTINGS_R, settings_r);
