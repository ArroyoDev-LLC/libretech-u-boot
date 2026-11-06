// SPDX-License-Identifier: GPL-2.0+
/*
 * The bootsettings command's Rockchip decode: RK3328 and RK3399.
 *
 * Neither SoC has boot strap pins, and the BootROM's medium order is fixed (TRM §1.2; claude
 * tools/libre-computer-app/boot-settings-registers.md): RK3328 eMMC, SPI NOR, SPI NAND, SD, then
 * USB MaskROM; RK3399 SPI NOR, SPI NAND, eMMC, SD, then USB. No password or fuse in that flow
 * skips USB, and the TRMs give no field map for a secure-boot, USB-boot or JTAG fuse, so
 * boot_efuse is unknown. The BootROM's own boot record (IRAM + 0x10) is gone by U-Boot proper
 * (BL31 loads over it); boot_source comes from the bootsource SPL handed over. Reads only.
 */

#include <bootsettings.h>
#include <asm/io.h>

static const char *rockchip_boot_order(void)
{
	if (IS_ENABLED(CONFIG_ROCKCHIP_RK3328))
		return "emmc,spi,nand,sd,usb";
	if (IS_ENABLED(CONFIG_ROCKCHIP_RK3399))
		return "spi,nand,emmc,sd,usb";
	return "unknown";
}

void bootsettings_soc(void)
{
	bootsettings_set("order", rockchip_boot_order());
	bootsettings_set("efuse", "unknown");
#if CONFIG_ROCKCHIP_BOOT_MODE_REG
	/* The reboot-mode word: BOOT_BROM_DOWNLOAD (0xef08a53c) sends the next boot to MaskROM. */
	bootsettings_set_raw("BOOT_MODE_REG", readl((void *)CONFIG_ROCKCHIP_BOOT_MODE_REG));
#endif
}
