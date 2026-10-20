/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 */

#ifndef __CONFIG_RK3328_COMMON_H
#define __CONFIG_RK3328_COMMON_H

#include "rockchip-common.h"

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
 */
#define ENV_MEM_LAYOUT_SETTINGS		\
	"scriptaddr=0x04000000\0"	\
	"script_offset_f=0xffe000\0"	\
	"script_size_f=0x2000\0"	\
	"pxefile_addr_r=0x04100000\0"	\
	"fdtoverlay_addr_r=0x04200000\0"	\
	"fdt_addr_r=0x04300000\0"	\
	"kernel_addr_r=0x04600000\0"	\
	"loadaddr=0x04600000\0"		\
	"kernel_comp_addr_r=0x08600000\0"	\
	"kernel_comp_size=0x4000000\0"	\
	"ramdisk_addr_r=0x0d600000\0"	\
	"initrd_high=0xffffffffffffffff\0"

#define CFG_EXTRA_ENV_SETTINGS		\
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0"	\
	"partitions=" PARTS_DEFAULT	\
	ENV_MEM_LAYOUT_SETTINGS		\
	ROCKCHIP_DEVICE_SETTINGS	\
	"boot_targets=" BOOT_TARGETS "\0"

#endif /* __CONFIG_RK3328_COMMON_H */
