/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The Libre Computer Rockchip DFU alternate names and storage extents of the
 * SPL USB flash session (arch/arm/mach-rockchip/spl-usbflash.c), under the
 * names the Meson family uses (<asm/arch-meson/usbflash-alts.h>), so a host
 * flashes either family the same way.
 *
 * The bootloader region is the Rockchip one: the BootROM reads the idblock
 * from sector 64 of the eMMC or SD user area, and the u-boot-bin alternates
 * span what the image's size gate (LBS lib/target-size.sh) lets a bootloader
 * there occupy, so a write cannot reach partition 1:
 *   RK3399  sectors 0x40..0xfbf: the unified image (idblock, then the FIT at
 *           the SPI offset) is written there too, under the 0x1f0000 NOR
 *           budget, with 2 MiB ahead of partition 1;
 *   RK3328  sectors 0x40..0x7ff: the compact image only, under the 1 MiB
 *           default (1 MiB - 64 * 512 B).
 * The RK3399 BootROM does not boot from the eMMC boot partitions, so they
 * have no u-boot-bin alternate; the whole-part alternates are listed for
 * reading.
 */

#ifndef __ROCKCHIP_USBFLASH_ALTS_H__
#define __ROCKCHIP_USBFLASH_ALTS_H__

#if IS_ENABLED(CONFIG_ROCKCHIP_RK3328)
#define ROCKCHIP_DFU_UBOOT_BLOCKS	"0x7c0"
#else
#define ROCKCHIP_DFU_UBOOT_BLOCKS	"0xf80"
#endif

/* mmc 0 (eMMC): `name raw <lba> <blocks> mmcpart <n>` */
#define ROCKCHIP_DFU_MMC_EMMC_ALTS \
	"u-boot-bin-emmc raw 0x40 " ROCKCHIP_DFU_UBOOT_BLOCKS " mmcpart 0;" \
	"emmc raw 0 0 mmcpart 0;" \
	"emmc-boot0 raw 0 0 mmcpart 1;" \
	"emmc-boot1 raw 0 0 mmcpart 2"

/* mmc 1 (SD) */
#define ROCKCHIP_DFU_MMC_SD_ALTS \
	"u-boot-bin-sd raw 0x40 " ROCKCHIP_DFU_UBOOT_BLOCKS " mmcpart 0;" \
	"sd raw 0 0 mmcpart 0"

/*
 * SPI NOR (SPL_ROCKCHIP_USBFLASH_NOR): the canonical NOR layout's regions,
 * as on Meson.  The config block's offset depends on the part's size and is
 * computed at run time (spl-usbflash.c usbflash_probe_nor()).
 */
#define ROCKCHIP_DFU_SF_UBOOT_SIZE	0x1f0000
#define ROCKCHIP_DFU_SF_CONFIG_SIZE	0x10000
#define ROCKCHIP_DFU_SF_FIT_OFFSET	0x200000
#define ROCKCHIP_DFU_SF_FIT_SIZE	0x200000

/* SPL: run the flash session on a MaskROM (USB) boot (spl-usbflash.c) */
void rockchip_spl_usbflash(void);
/* The board's setup before that session (weak, empty by default) */
void rockchip_spl_usbflash_board(void);

#endif /* __ROCKCHIP_USBFLASH_ALTS_H__ */
