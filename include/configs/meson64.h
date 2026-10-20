/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for Amlogic Meson 64bits SoCs
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 */

#ifndef __MESON64_CONFIG_H
#define __MESON64_CONFIG_H

/* Generic Interrupt Controller Definitions */
#if (defined(CONFIG_MESON_AXG) || defined(CONFIG_MESON_G12A))
#define GICD_BASE			0xffc01000
#define GICC_BASE			0xffc02000
#elif defined(CONFIG_MESON_A1)
#define GICD_BASE			0xff901000
#define GICC_BASE			0xff902000
#else /* MESON GXL and GXBB */
#define GICD_BASE			0xc4301000
#define GICC_BASE			0xc4302000
#endif

/* Serial drivers */
/* The following table includes the supported baudrates */
#define CFG_SYS_BAUDRATE_TABLE  \
	{300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, \
		230400, 250000, 460800, 500000, 1000000, 2000000, 4000000, \
		8000000 }

/* For splashscreen */
#ifdef CONFIG_VIDEO
#define STDOUT_CFG "vidconsole,serial"
#else
#define STDOUT_CFG "serial"
#endif

#ifdef CONFIG_USB_KEYBOARD
#define STDIN_CFG "usbkbd,serial"
#else
#define STDIN_CFG "serial"
#endif

#define CFG_SYS_SDRAM_BASE		0

/* ROM USB boot support, auto-execute boot.scr at scriptaddr */
#define BOOTENV_DEV_ROMUSB(devtypeu, devtypel, instance) \
	"bootcmd_romusb=" \
		"if test \"${boot_source}\" = \"usb\" && " \
				"test -n \"${scriptaddr}\"; then " \
			"echo '(ROM USB boot)'; " \
			"source ${scriptaddr}; " \
		"fi\0"

#define BOOTENV_DEV_NAME_ROMUSB(devtypeu, devtypel, instance)	\
		"" /* add to preboot, remove for bootstd */

/*
 * Fallback to "USB DFU" boot if script is not at scriptaddr
 *
 * DFU will expose the kernel_addr_r memory range as DFU entry,
 * then with `dfu-util --detach`, booting the uploaded image
 * will be attempted:
 * $ dfu-util -a 0 -D fitImage
 * $ dfu-util -a 0 -e
 */
#if CONFIG_IS_ENABLED(USB_GADGET) && CONFIG_IS_ENABLED(DFU_OVER_USB)
	#define BOOTENV_DEV_USB_DFU(devtypeu, devtypel, instance) \
		"bootcmd_usbdfu=" \
			"if test \"${boot_source}\" = \"usb\"; then " \
				"dfu 0 60;" \
			"fi\0"

	#define BOOTENV_DEV_NAME_USB_DFU(devtypeu, devtypel, instance) \
		"" /* add to preboot, remove for bootstd */
#else
	#define BOOTENV_DEV_USB_DFU(devtypeu, devtypel, instance)
	#define BOOTENV_DEV_NAME_USB_DFU(devtypeu, devtypel, instance)
#endif

#ifdef CONFIG_CMD_MMC
	#define BOOT_TARGET_MMC(func) \
		func(MMC, mmc, 0) \
		func(MMC, mmc, 1) \
		func(MMC, mmc, 2)
#else
	#define BOOT_TARGET_MMC(func)
#endif

#ifdef CONFIG_CMD_USB
#define BOOT_TARGET_DEVICES_USB(func) func(USB, usb, 0)
#else
#define BOOT_TARGET_DEVICES_USB(func)
#endif

#ifdef CONFIG_CMD_NVME
	#define BOOT_TARGET_NVME(func) func(NVME, nvme, 0)
#else
	#define BOOT_TARGET_NVME(func)
#endif

#ifdef CONFIG_CMD_SCSI
	#define BOOT_TARGET_SCSI(func) func(SCSI, scsi, 0)
#else
	#define BOOT_TARGET_SCSI(func)
#endif

#if defined(CONFIG_CMD_DHCP) && defined(CONFIG_CMD_PXE)
	#define BOOT_TARGET_PXE(func) func(PXE, pxe, na)
	#define BOOT_TARGET_DHCP(func) func(DHCP, dhcp, na)
#elif defined(CONFIG_CMD_DHCP)
	#define BOOT_TARGET_PXE(func)
	#define BOOT_TARGET_DHCP(func) func(DHCP, dhcp, na)
#else
	#define BOOT_TARGET_PXE(func)
	#define BOOT_TARGET_DHCP(func)
#endif

#ifndef BOOT_TARGET_DEVICES
#define BOOT_TARGET_DEVICES(func) \
	func(ROMUSB, romusb, na)  \
	func(USB_DFU, usbdfu, na)  \
	BOOT_TARGET_MMC(func) \
	BOOT_TARGET_DEVICES_USB(func) \
	BOOT_TARGET_NVME(func) \
	BOOT_TARGET_SCSI(func)
#endif

#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT) || \
    defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT)
/*
 * The libre DRAM load map: design/rules/bootloader/dram-load-map.md.  One set of
 * offsets for every libre board, from offset 0x04000000 -- above the 16 MiB
 * secure block both families now reserve from 0 (ramoops at 0x00F00000) and the
 * headroom GXL moved BL33 to 0x04000000 to buy.  Script, PXE file and overlay are
 * written only after U-Boot relocates to the top of DRAM, so sharing GXL's text
 * base with scriptaddr is safe.  loadaddr is restated because env_default.h
 * emits it from CONFIG_SYS_LOAD_ADDR earlier in default_environment and the
 * later entry wins; the map's alias is the kernel slot.
 *
 * Only the two LIBRE layouts take it.  Every other meson64 build keeps the
 * values below the #else: a vendor chain still runs BL31 at 0x05100000, which
 * sits inside this map's kernel slot, and AXG/A1 were never measured against it.
 *
 * The kernel slot is 2 MiB-aligned so booti runs an arm64 Image in place
 * instead of copying it up to the next 2 MiB boundary, and it is 144 MiB
 * because booti leaves the DECOMPRESSED kernel there: a 6.18 Image declares
 * 70 MiB and a kernel with a built-in initramfs 130 MiB.  The decompress
 * buffer is nested in the slot's upper 80 MiB -- a compressed file (at most
 * KERNEL_COMP_SIZE) sits at the bottom, decompresses above it and is copied
 * back down -- so it costs the ramdisk nothing.  The *_SLOT_SIZE values are
 * the budgets, used by the DFU RAM alternates (board-meson.c).
 *
 * initrd_high=~0 boots the initramfs where it was loaded.  Unset, booti copies
 * it into a second free region below U-Boot's top reservation, so one initramfs
 * holds two copies of its size; on a 512 MiB S805X that halves the ramdisk room.
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
#define LOADADDR_ENV		"loadaddr=" KERNEL_ADDR_R "\0" \
				"initrd_high=0xffffffffffffffff\0"
#else
#define KERNEL_ADDR_R		__stringify(0x08080000)
#define KERNEL_SLOT_SIZE	__stringify(0x7f80000)
#define KERNEL_COMP_ADDR_R	__stringify(0x0d080000)
#define KERNEL_COMP_SIZE	__stringify(0x2000000)
#define FDT_ADDR_R		__stringify(0x08008000)
#define FDT_SLOT_SIZE		__stringify(0x78000)
#define SCRIPT_ADDR_R		__stringify(0x08000000)
#define PXEFILE_ADDR_R		__stringify(0x01080000)
#define FDTOVERLAY_ADDR_R	__stringify(0x01000000)
#define RAMDISK_ADDR_R		__stringify(0x13000000)
#define RAMDISK_SLOT_SIZE	__stringify(0x4000000)
#define LOADADDR_ENV
#endif

#include <config_distro_bootcmd.h>

#ifdef CONFIG_OF_UPSTREAM
#define FDTFILE_NAME		CONFIG_DEFAULT_DEVICE_TREE ".dtb"
#else
#define FDTFILE_NAME		"amlogic/" CONFIG_DEFAULT_DEVICE_TREE ".dtb"
#endif

#ifndef CFG_EXTRA_ENV_SETTINGS
#define CFG_EXTRA_ENV_SETTINGS \
	"stdin=" STDIN_CFG "\0" \
	"stdout=" STDOUT_CFG "\0" \
	"stderr=" STDOUT_CFG "\0" \
	"kernel_comp_addr_r=" KERNEL_COMP_ADDR_R "\0" \
	"kernel_comp_size=" KERNEL_COMP_SIZE "\0" \
	"fdt_addr_r=" FDT_ADDR_R "\0" \
	"scriptaddr=" SCRIPT_ADDR_R "\0" \
	"kernel_addr_r=" KERNEL_ADDR_R "\0" \
	LOADADDR_ENV \
	"pxefile_addr_r=" PXEFILE_ADDR_R "\0" \
	"fdtoverlay_addr_r=" FDTOVERLAY_ADDR_R "\0" \
	"ramdisk_addr_r=" RAMDISK_ADDR_R "\0" \
	"fdtfile=" FDTFILE_NAME "\0" \
	"dfu_alt_info=fitimage ram " KERNEL_ADDR_R " 0x4000000 \0" \
	BOOTENV
#endif

#endif /* __MESON64_CONFIG_H */
