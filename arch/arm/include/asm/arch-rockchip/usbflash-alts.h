/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The Libre Computer Rockchip DFU alternate names and storage extents of the
 * SPL USB flash session (arch/arm/mach-rockchip/spl-usbflash.c), under the
 * names the Meson family uses (<asm/arch-meson/usbflash-alts.h>), so a host
 * flashes either family the same way.
 *
 * The bootloader region is the Rockchip one: the BootROM reads the idblock
 * from sector 64 of the eMMC or SD user area, and the bootloader (idblock and
 * FIT, compact or unified layout) lies below sector 0x4000 (8 MiB).  The
 * RK3399 BootROM does not boot from the eMMC boot partitions, so they have no
 * u-boot-bin alternate; the whole-part alternates are listed for reading.
 */

#ifndef __ROCKCHIP_USBFLASH_ALTS_H__
#define __ROCKCHIP_USBFLASH_ALTS_H__

/* mmc 0 (eMMC): `name raw <lba> <blocks> mmcpart <n>` */
#define ROCKCHIP_DFU_MMC_EMMC_ALTS \
	"u-boot-bin-emmc raw 0x40 0x3fc0 mmcpart 0;" \
	"emmc raw 0 0 mmcpart 0;" \
	"emmc-boot0 raw 0 0 mmcpart 1;" \
	"emmc-boot1 raw 0 0 mmcpart 2"

/* mmc 1 (SD) */
#define ROCKCHIP_DFU_MMC_SD_ALTS \
	"u-boot-bin-sd raw 0x40 0x3fc0 mmcpart 0;" \
	"sd raw 0 0 mmcpart 0"

/* SPL: run the flash session on a MaskROM (USB) boot (spl-usbflash.c) */
void rockchip_spl_usbflash(void);
/* The board's setup before that session (weak, empty by default) */
void rockchip_spl_usbflash_board(void);

#endif /* __ROCKCHIP_USBFLASH_ALTS_H__ */
