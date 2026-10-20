/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8188 (Genio 510, die MT8370) based boards
 *
 * Copyright (C) 2025 MediaTek Inc.
 * Author: Macpaul Lin <macpaul.lin@mediatek.com>
 */

#ifndef __MT8188_H
#define __MT8188_H

#include <linux/sizes.h>

/*
 * UART is the DM mtk-serial driver (CONFIG_MTK_SERIAL=y) bound via the
 * device tree (mediatek,mt8195-uart / mediatek,hsuart). The 2022.10 BSP
 * legacy CONFIG_SYS_NS16550_* platform defines are dropped -- they no
 * longer exist in current u-boot (renamed CFG_SYS_NS16550_*) and would
 * force-compile the legacy ns16550 path, mirroring the in-tree mt8365 and
 * the genio-720/mt8391 sibling port.
 */

/* Environment settings */
#include <config_distro_bootcmd.h>

#ifdef CONFIG_CMD_MMC
#define BOOT_TARGET_MMC(func) \
	func(MMC, mmc, 0) \
	func(MMC, mmc, 1)
#else
#define BOOT_TARGET_MMC(func)
#endif

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_USB(func)
#endif

#ifdef CONFIG_CMD_SCSI
#define BOOT_TARGET_SCSI(func) func(SCSI, scsi, 2)
#else
#define BOOT_TARGET_SCSI(func)
#endif

#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_MMC(func) \
	BOOT_TARGET_USB(func) \
	BOOT_TARGET_SCSI(func)

/*
 * env_default.h consumes CFG_EXTRA_ENV_SETTINGS (CONFIG_EXTRA_ENV_SETTINGS
 * is ignored). APU/VPU CMA 20+20 MiB now occupy 0x41800000–0x43FFFFFF
 * (abut SCP). scriptaddr was 0x41800000 and would smash APU. After CMA
 * the first NS scratch is 0x44000000 (FIP already consumed by BL2; KEEP
 * FIP ~2 MiB). fdt_addr_r must not share scriptaddr (`source` still live
 * while loading FDT).
 *
 * The nine variables are the MT8188 row of the libre DRAM load map, the
 * shared offsets plus the 0x40000000 DRAM base:
 * design/rules/bootloader/dram-load-map.md. They are the seven slots, the
 * loadaddr alias, and kernel_comp_size. loadaddr is restated here because
 * env_default.h emits it from CONFIG_SYS_LOAD_ADDR (the text base) earlier
 * in default_environment and the later entry wins; the map's alias is the
 * kernel slot. splashimage keeps CONFIG_SYS_LOAD_ADDR because the splash
 * file stays at the text base and is not one of the slots. initrd_high=~0
 * boots the initramfs in its slot instead of copying it below U-Boot.
 */
#if !defined(CFG_EXTRA_ENV_SETTINGS)
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
	"fdt_resize=0x3000\0" \
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE ".dtb\0" \
	"splashimage=" __stringify(CONFIG_SYS_LOAD_ADDR) "\0" \
	"splashsource=mmc_fs\0" \
	"splashfile=logo.bmp\0" \
	"splashdevpart=0#bootassets\0" \
	"splashpos=m,m\0" \
	BOOTENV
#endif

#ifdef CONFIG_ARM64
#define MTK_SIP_PLAT_BINFO		0xC2000529
#endif

/* DRAM */
#define CONFIG_SYS_SDRAM_BASE		0x40000000

#endif
