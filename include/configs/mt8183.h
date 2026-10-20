/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8183 based boards
 *
 * Copyright (C) 2021 BayLibre, SAS
 * Author: Fabien Parent <fparent@baylibre.com
 */

#ifndef __MT8183_H
#define __MT8183_H

#include <linux/sizes.h>

#define CFG_SYS_NS16550_COM1		0x11005200
#define CFG_SYS_NS16550_CLK		26000000

/* Environment settings */
#include <config_distro_bootcmd.h>

#define BOOT_TARGET_DEVICES(func) \
	func(MMC, mmc, 0)

/*
 * fdtfile is what scan_dev_for_efi() probes for under efi_dtb_prefixes before
 * handing control to the EFI payload. Leave it unset and the lookup degrades to
 * a bare prefix, no OS DTB is ever loaded, and bootefi falls back to u-boot's
 * own control FDT -- see arch/arm/dts/mt8183-pumpkin-u-boot.dtsi.
 *
 * The nine variables are the MT8183 row of the libre DRAM load map, the
 * shared offsets plus the 0x40000000 DRAM base:
 * design/rules/bootloader/dram-load-map.md. They are the seven slots, the
 * loadaddr alias, and kernel_comp_size. loadaddr is restated here because
 * env_default.h emits it from CONFIG_SYS_LOAD_ADDR (the text base) earlier in
 * default_environment and the later entry wins. The secure TZRAM is linked
 * at 0x40000000, leaving the slots at offset 0x04000000 to share the map's
 * load addresses. initrd_high=~0 boots the initramfs in its slot instead of
 * copying it below U-Boot.
 */
#define CFG_EXTRA_ENV_SETTINGS \
	"scriptaddr=0x44000000\0" \
	"pxefile_addr_r=0x44100000\0" \
	"fdtoverlay_addr_r=0x44200000\0" \
	"fdt_addr_r=0x44300000\0" \
	"kernel_addr_r=0x44600000\0" \
	"loadaddr=0x44600000\0" \
	"kernel_comp_addr_r=0x48600000\0" \
	"kernel_comp_size=0x4000000\0" \
	"ramdisk_addr_r=0x4d600000\0" \
	"initrd_high=0xffffffffffffffff\0" \
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0" \
	BOOTENV

#endif
