/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 */

#ifndef __CONFIG_RK3328_COMMON_H
#define __CONFIG_RK3328_COMMON_H

#include "rockchip-common.h"
#include <linux/stringify.h>

#define CFG_IRAM_BASE			0xff090000

#define CFG_SYS_SDRAM_BASE		0
#define SDRAM_MAX_SIZE			0xff000000

/*
 * The nine variables are the RK3328 row of the libre DRAM load map, the
 * shared offsets on top of the 0 DRAM base:
 * design/rules/bootloader/dram-load-map.md. They are the seven slots, the
 * loadaddr alias, and kernel_comp_size; script_offset_f and script_size_f
 * are FAT script-load knobs, not slots. sgrf_init zeroes SGRF DDR region 0
 * and leaves the other firewalls non-secure, so no secure region covers a
 * slot. loadaddr is restated here because env_default.h emits it from
 * CONFIG_SYS_LOAD_ADDR (the text base) earlier in default_environment and
 * the later entry wins; the map's alias is the kernel slot. initrd_high=~0
 * boots the initramfs in its slot instead of copying it below U-Boot.
 *
 * The *_SLOT_SIZE values are the budgets of the same map, the ones the GXL
 * libre layout carries (meson64.h), and the U-Boot DFU RAM alternates
 * (ROCKCHIP_DFU_RAM_ALTS, board-rockchip.c) take their sizes from them.
 */
#define SCRIPT_ADDR_R		__stringify(0x04000000)
#define PXEFILE_ADDR_R		__stringify(0x04100000)
#define FDTOVERLAY_ADDR_R	__stringify(0x04200000)
#define FDT_ADDR_R		__stringify(0x04300000)
#define FDT_SLOT_SIZE		__stringify(0x200000)
#define KERNEL_ADDR_R		__stringify(0x04600000)
#define KERNEL_SLOT_SIZE	__stringify(0x9000000)
#define KERNEL_COMP_ADDR_R	__stringify(0x08600000)
#define KERNEL_COMP_SIZE	__stringify(0x4000000)
#define RAMDISK_ADDR_R		__stringify(0x0d600000)
#define RAMDISK_SLOT_SIZE	__stringify(0x9a00000)

#define ENV_MEM_LAYOUT_SETTINGS		\
	"scriptaddr=" SCRIPT_ADDR_R "\0"	\
	"script_offset_f=0xffe000\0"	\
	"script_size_f=0x2000\0"	\
	"pxefile_addr_r=" PXEFILE_ADDR_R "\0"	\
	"fdtoverlay_addr_r=" FDTOVERLAY_ADDR_R "\0"	\
	"fdt_addr_r=" FDT_ADDR_R "\0"	\
	"kernel_addr_r=" KERNEL_ADDR_R "\0"	\
	"loadaddr=" KERNEL_ADDR_R "\0"	\
	"kernel_comp_addr_r=" KERNEL_COMP_ADDR_R "\0"	\
	"kernel_comp_size=" KERNEL_COMP_SIZE "\0"	\
	"ramdisk_addr_r=" RAMDISK_ADDR_R "\0"	\
	"initrd_high=0xffffffffffffffff\0"

/* U-Boot's DFU RAM alternates, under the Meson names (board-meson.c) */
#define ROCKCHIP_DFU_RAM_ALTS		\
	"script script " SCRIPT_ADDR_R " 0x8000;"	\
	"fdt ram " FDT_ADDR_R " " FDT_SLOT_SIZE ";"	\
	"kernel ram " KERNEL_ADDR_R " " KERNEL_SLOT_SIZE ";"	\
	"ramdisk ram " RAMDISK_ADDR_R " " RAMDISK_SLOT_SIZE

#define CFG_EXTRA_ENV_SETTINGS		\
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0"	\
	"partitions=" PARTS_DEFAULT	\
	ENV_MEM_LAYOUT_SETTINGS		\
	ROCKCHIP_DEVICE_SETTINGS	\
	"boot_targets=" BOOT_TARGETS "\0"

#endif /* __CONFIG_RK3328_COMMON_H */
