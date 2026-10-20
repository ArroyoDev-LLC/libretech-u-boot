// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * MediaTek BootROM storage strap → platform bootsource vocabulary.
 *
 * Unlike Rockchip (BROM_BOOTSOURCE_ID in IRAM) or Amlogic (AO_SEC_GP_CFG0),
 * MediaTek BL2 does not hand BL33 a boot-device argument. The BootROM's
 * storage choice is latched in GPIO_TRAPPING_REG (GPIO block, offset 0x56f0
 * from IO_PHYS = 0x10000000) and is still readable from NS U-Boot — the same
 * register U-Boot already reads for DRAM channel count on MT8188
 * (mt8188/init.c MT8188_GPIO_TRAPPING_REG).
 *
 * Provenance for the address and the boot-dev bits:
 *   MT8370 AP datasheet Table 3-209 / MT8390 Table 3-209 (AUD_SYNC_MOSI:
 *   0 = eMMC, 1 = SPI NOR);
 *   TF-A plat/mediatek/mt8188/bl2_plat_setup.c mt_get_storage_type()
 *   (bit 10, 1-bit);
 *   TF-A plat/mediatek/mt8189/include/platform_def.h GPIO_TRAPPING_REG
 *   + plat/mediatek/mt8391/bl2_plat_setup.c mt_get_storage_type()
 *   (bits [11:10], 0 = UFS, 1 = SPI NOR, 2 = eMMC).
 *
 * This is the storage strap, not the BROM USB-download entry (KPCOL0 /
 * WDT NONRST2). The actual load path for a USB/UART DRAM-boot is handed
 * over in BL33 ep_info.args.arg2: BL2 writes MTK_BL33_ARG2_USB ('LCUS')
 * when usb_dl_active. lowlevel_init.S captures x2 before BSS clear.
 * GPIO_TRAPPING_REG remains the storage-boot map. SoCs with no
 * documented trap (MT8183, MT8195 sequential eMMC→UFS→SPI NOR + eFuse
 * skip, MT8365) stay unknown unless that arg2 tag is present.
 */

#include <asm/io.h>
#include <bootsource.h>
#include <linux/bitops.h>

/*
 * GPIO_TRAPPING_REG — IO_PHYS + 0x56f0. Same window on MT8188/MT8189.
 * Vendor: plat/mediatek/mt8189/include/platform_def.h GPIO_TRAPPING_REG
 * and u-boot arch/arm/mach-mediatek/mt8188/init.c MT8188_GPIO_TRAPPING_REG.
 */
#define MTK_IO_PHYS			0x10000000
#define MTK_GPIO_TRAPPING_REG		(MTK_IO_PHYS + 0x000056f0)
#define MTK_GPIO_TRAP_BOOTDEV_SHIFT	10

/*
 * BL33 x2 tag. TF-A plat/mediatek/mt8188/bl2_plat_setup.c
 * mtk_bl33_note_usb_load() writes this into ep_info.args.arg2
 * when the FIP came from USB/UART (usb_dl_active). 'LCUS' =
 * Libre Computer USB Source. arg0/arg1 are fw_config/hw_config
 * in common/desc_image_load.c — do not reuse them.
 */
#define MTK_BL33_ARG2_USB		0x4c435553UL

extern unsigned long mtk_bl33_arg2;

enum bootsource mtk_bootsource(void)
{
	if (mtk_bl33_arg2 == MTK_BL33_ARG2_USB)
		return BOOTSOURCE_USB;

#if CONFIG_IS_ENABLED(TARGET_MT8188)
	u32 trap = readl(MTK_GPIO_TRAPPING_REG);

	/* AUD_SYNC_MOSI, 1 bit. Datasheet Table 3-209. */
	if (trap & BIT(MTK_GPIO_TRAP_BOOTDEV_SHIFT))
		return BOOTSOURCE_SPI;
	return BOOTSOURCE_EMMC;
#elif CONFIG_IS_ENABLED(TARGET_MT8189)
	u32 bits = (readl(MTK_GPIO_TRAPPING_REG) >>
		    MTK_GPIO_TRAP_BOOTDEV_SHIFT) & 0x3;

	/* TF-A plat/mediatek/mt8391/bl2_plat_setup.c mt_get_storage_type(). */
	switch (bits) {
	case 0:
		return BOOTSOURCE_UFS;
	case 1:
		return BOOTSOURCE_SPI;
	case 2:
		return BOOTSOURCE_EMMC;
	default:
		return BOOTSOURCE_UNKNOWN;
	}
#else
	return BOOTSOURCE_UNKNOWN;
#endif
}
