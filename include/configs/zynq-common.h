/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2012 Michal Simek <monstr@monstr.eu>
 * (C) Copyright 2013 - 2018 Xilinx, Inc.
 *
 * Common configuration options for all Zynq boards.
 */

#ifndef __CONFIG_ZYNQ_COMMON_H
#define __CONFIG_ZYNQ_COMMON_H

/* Cache options */
#ifndef CONFIG_SYS_L2CACHE_OFF
# define CFG_SYS_PL310_BASE		0xf8f02000
#endif

#define ZYNQ_SCUTIMER_BASEADDR		0xF8F00600
#define CFG_SYS_TIMERBASE		ZYNQ_SCUTIMER_BASEADDR
#define CFG_SYS_TIMER_COUNTER	(CFG_SYS_TIMERBASE + 0x4)

/* GUIDs for capsule updatable firmware images */
#define XILINX_BOOT_IMAGE_GUID \
	EFI_GUID(0x1ba29a15, 0x9969, 0x40aa, 0xb4, 0x24, \
		 0xe8, 0x61, 0x21, 0x61, 0x86, 0x64)

#define XILINX_UBOOT_IMAGE_GUID \
	EFI_GUID(0x1a5178f0, 0x87d3, 0x4f36, 0xac, 0x63, \
		 0x3b, 0x31, 0xa2, 0x3b, 0xe3, 0x05)

/* Serial drivers */
/* The following table includes the supported baudrates */
#define CFG_SYS_BAUDRATE_TABLE  \
	{300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400}

/* Ethernet driver */

/* NOR */

#ifdef CONFIG_USB_EHCI_ZYNQ
# define DFU_DEFAULT_POLL_TIMEOUT	300
#endif

/* enable preboot to be loaded before CONFIG_BOOTDELAY */

/* Boot configuration */

#ifdef CONFIG_XPL_BUILD
#define BOOTENV
#endif

/* Only use this section if no BOOTENV has been configured yet */
#ifndef BOOTENV

#ifdef CONFIG_CMD_MMC
#define BOOT_TARGET_DEVICES_MMC(func) func(MMC, mmc, 0) func(MMC, mmc, 1)
#else
#define BOOT_TARGET_DEVICES_MMC(func)
#endif

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_DEVICES_USB(func) func(USB, usb, 0) func(USB, usb, 1)
#else
#define BOOT_TARGET_DEVICES_USB(func)
#endif

#if defined(CONFIG_CMD_PXE) && defined(CONFIG_CMD_DHCP)
#define BOOT_TARGET_DEVICES_PXE(func) func(PXE, pxe, na)
#else
#define BOOT_TARGET_DEVICES_PXE(func)
#endif

#if defined(CONFIG_CMD_DHCP)
#define BOOT_TARGET_DEVICES_DHCP(func) func(DHCP, dhcp, na)
#else
#define BOOT_TARGET_DEVICES_DHCP(func)
#endif

/*
 * The QSPI boot target sources a boot SCRIPT out of SPI NOR (BOOTENV_DEV_QSPI
 * below: sf read ${script_offset_f} -> source). That is a valid Xilinx layout,
 * but it is NOT a sanctioned consumer of the Libre Computer NOR map.
 *
 * Board-bootloader policy §4 gives SPI NOR exactly three regions -- the
 * bootloader blob at 0x0, the 64 KiB config sector at 0x1F0000, and the DT
 * overlay FIT at 0x200000 (which on OP-TEE boards may also carry BL32). There
 * is no boot-script region, and the sanctioned per-unit override surface is
 * the config sector: plain key=value pairs imported by board_config_env_import() (which
 * can set bootcmd/boot_targets/anything else) plus an "overlays=" list applied
 * by nor_apply_overlays(). A NOR boot script is redundant with that, and every
 * offset it could use is inside a region something else owns -- the QSPI slot
 * was pointed at 0x200000, the overlay-FIT region, before this.
 *
 * So a board that has adopted the policy map -- which is exactly what
 * CONFIG_BOARD_CONFIG marks -- does not get the qspi boot target at all,
 * and config_distro_bootcmd.h then emits no bootcmd_qspi and no "qspi" token
 * in the compiled-in boot_targets. CONFIG_ZYNQ_QSPI stays on: it is the driver
 * the config sector, the overlay FIT and `sf` themselves need.
 *
 * Every other Zynq board in this tree leaves BOARD_CONFIG unset and is
 * unaffected -- only zynq_adrv9009 and zynq_e310 select it.
 *
 * The runtime half of this is board_late_init() in board/xilinx/zynq/board.c,
 * which prepends the DETECTED boot mode: on a QSPI-strapped board it must not
 * re-add a "qspi" token that has no bootcmd_qspi behind it. See the comment
 * there.
 */
#if defined(CONFIG_ZYNQ_QSPI) && !defined(CONFIG_BOARD_CONFIG)
# define BOOT_TARGET_DEVICES_QSPI(func)	func(QSPI, qspi, na)
#else
# define BOOT_TARGET_DEVICES_QSPI(func)
#endif

#if defined(CONFIG_NAND_ZYNQ)
# define BOOT_TARGET_DEVICES_NAND(func)	func(NAND, nand, na)
#else
# define BOOT_TARGET_DEVICES_NAND(func)
#endif

#if defined(CONFIG_MTD_NOR_FLASH)
# define BOOT_TARGET_DEVICES_NOR(func)	func(NOR, nor, na)
#else
# define BOOT_TARGET_DEVICES_NOR(func)
#endif

#define BOOTENV_DEV_QSPI(devtypeu, devtypel, instance) \
	"bootcmd_qspi=sf probe 0 0 0 && " \
		      "sf read ${scriptaddr} ${script_offset_f} ${script_size_f} && " \
		      "echo QSPI: Trying to boot script at ${scriptaddr} && " \
		      "source ${scriptaddr}; echo QSPI: SCRIPT FAILED: continuing...;\0"

#define BOOTENV_DEV_NAME_QSPI(devtypeu, devtypel, instance) \
	"qspi "

#define BOOTENV_DEV_NAND(devtypeu, devtypel, instance) \
	"bootcmd_nand=nand info && " \
		      "nand read ${scriptaddr} ${script_offset_f} ${script_size_f} && " \
		      "echo NAND: Trying to boot script at ${scriptaddr} && " \
		      "source ${scriptaddr}; echo NAND: SCRIPT FAILED: continuing...;\0"

#define BOOTENV_DEV_NAME_NAND(devtypeu, devtypel, instance) \
	"nand "

#define BOOTENV_DEV_NOR(devtypeu, devtypel, instance) \
	"script_offset_nor=0xE2FC0000\0"        \
	"bootcmd_nor=cp.b ${script_offset_nor} ${scriptaddr} ${script_size_f} && " \
		     "echo NOR: Trying to boot script at ${scriptaddr} && " \
		     "source ${scriptaddr}; echo NOR: SCRIPT FAILED: continuing...;\0"

#define BOOTENV_DEV_NAME_NOR(devtypeu, devtypel, instance) \
	"nor "

#define BOOT_TARGET_DEVICES_JTAG(func)  func(JTAG, jtag, na)

#define BOOTENV_DEV_JTAG(devtypeu, devtypel, instance) \
	"bootcmd_jtag=echo JTAG: Trying to boot script at ${scriptaddr} && " \
		"source ${scriptaddr}; echo JTAG: SCRIPT FAILED: continuing...;\0"

#define BOOTENV_DEV_NAME_JTAG(devtypeu, devtypel, instance) \
	"jtag "

#define BOOT_TARGET_DEVICES_USB_DFU(func) \
	func(USB_DFU, usb_dfu, 0) func(USB_DFU, usb_dfu, 1)

#define BOOTENV_DEV_USB_DFU(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel #instance "=setenv dfu_alt_info boot.scr ram " \
	"$scriptaddr $script_size_f && " \
	"dfu " #instance " ram " #instance " 60 && " \
	"echo DFU" #instance ": Trying to boot script at ${scriptaddr} && " \
	"source ${scriptaddr}; " \
	"echo DFU" #instance ": SCRIPT FAILED: continuing...;\0"

#define BOOTENV_DEV_NAME_USB_DFU(devtypeu, devtypel, instance) \
	""

#define BOOT_TARGET_DEVICES_USB_THOR(func) \
	func(USB_THOR, usb_thor, 0) func(USB_THOR, usb_thor, 1)

#define BOOTENV_DEV_USB_THOR(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel #instance "=setenv dfu_alt_info boot.scr ram " \
	"$scriptaddr $script_size_f && " \
	"thordown " #instance " ram " #instance " && " \
	"echo THOR" #instance ": Trying to boot script at ${scriptaddr} && " \
	"source ${scriptaddr}; " \
	"echo THOR" #instance ": SCRIPT FAILED: continuing...;\0"

#define BOOTENV_DEV_NAME_USB_THOR(devtypeu, devtypel, instance) \
	""

/*
 * Board-bootloader policy §2 "Default boot order": local media first, and
 * network/PXE is NOT a default target -- it is enabled by overriding
 * boot_targets from the NOR config sector. An unattended board whose media is
 * absent or unbootable must NOT fall through to BOOTP/DHCP and run whatever a
 * LAN PXE server happens to offer.
 *
 * So PXE and DHCP are dropped from the compiled-in list, and JTAG is demoted
 * below every storage target -- it was ahead of all of them, which is the same
 * "unattended board takes external input first" defect.
 *
 * `setenv boot_targets "... pxe dhcp"` still works, but NOT via bootcmd_pxe /
 * bootcmd_dhcp: config_distro_bootcmd.h only emits a bootcmd_<dev> for devices
 * that appear in BOOT_TARGET_DEVICES, so on a build with these two dropped
 * there is no bootcmd_pxe in the default environment at all (verified: 0 hits
 * for "bootcmd_pxe" in the built u-boot.bin). What makes the override work is
 * BOOTSTD: boot/bootdev-uclass.c label_to_uclass() special-cases the labels
 * "pxe" and "dhcp" to UCLASS_ETH with BOOTFLOW_METHF_PXE_ONLY / _DHCP_ONLY, so
 * `bootflow scan` walks boot_targets and picks them up directly. CMD_PXE and
 * CMD_DHCP stay enabled for the manual commands.
 *
 * The runtime half of this list is board_late_init() in
 * board/xilinx/zynq/board.c, which prepends the DETECTED boot mode; its JTAG
 * arm re-added "pxe dhcp" until it was fixed to prepend "jtag" alone.
 *
 * Same change rockchip-common.h took. Applies to every Zynq board in this tree.
 */
#define BOOT_TARGET_DEVICES(func) \
	BOOT_TARGET_DEVICES_MMC(func) \
	BOOT_TARGET_DEVICES_QSPI(func) \
	BOOT_TARGET_DEVICES_NAND(func) \
	BOOT_TARGET_DEVICES_NOR(func) \
	BOOT_TARGET_DEVICES_USB_DFU(func) \
	BOOT_TARGET_DEVICES_USB_THOR(func) \
	BOOT_TARGET_DEVICES_USB(func) \
	BOOT_TARGET_DEVICES_JTAG(func)

#include <config_distro_bootcmd.h>

#endif /* BOOTENV */

/* Default environment */
#ifndef CFG_EXTRA_ENV_SETTINGS
#define CFG_EXTRA_ENV_SETTINGS	\
	"script_size_f=0x40000\0"	\
	"fdt_addr_r=0x1f00000\0"        \
	"pxefile_addr_r=0x2000000\0"    \
	"kernel_addr_r=0x2000000\0"     \
	"ramdisk_addr_r=0x3100000\0"    \
	BOOTENV
#endif

/* Miscellaneous configurable options */

#define CFG_SYS_INIT_RAM_ADDR	0xFFFF0000
#define CFG_SYS_INIT_RAM_SIZE	0x2000

/* Extend size of kernel image for uncompression */

/* Address in RAM where the parameters must be copied by SPL. */

/* Not using MMC raw mode - just for compilation purpose */

/* SP location before relocation, must use scratch RAM */

/* 3 * 64kB blocks of OCM - one is on the top because of bootrom */

/* On the top of OCM space */

/*
 * SPL stack position - and stack goes down
 * 0xfffffe00 is used for putting wfi loop.
 * Set it up as limit for now.
 */

#endif /* __CONFIG_ZYNQ_COMMON_H */
