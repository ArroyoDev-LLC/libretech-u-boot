// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019 Rockchip Electronics Co., Ltd
 */

#include <binman_sym.h>
#include <blk.h>
#include <cpu_func.h>
#include <debug_uart.h>
#include <dm.h>
#include <hang.h>
#include <image.h>
#include <init.h>
#include <log.h>
#include <mapmem.h>
#include <memalign.h>
#include <mmc.h>
#include <ram.h>
#include <spl.h>
#include <sysreset.h>
#include <asm/arch-rockchip/boot_mode.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/timer.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/bitops.h>

DECLARE_GLOBAL_DATA_PTR;

int board_return_to_bootrom(struct spl_image_info *spl_image,
			    struct spl_boot_device *bootdev)
{
	/*
	 * Two reasons to be here, told apart by how SPL was configured:
	 *
	 *  - SPL_ROCKCHIP_BACK_TO_BROM: BOOT_DEVICE_BOOTROM is the whole boot
	 *    list and the BootROM loads the next stage itself. Tell it to
	 *    continue (BROM_BOOT_NEXTSTAGE).
	 *
	 *  - otherwise BOOT_DEVICE_BOOTROM is the LAST entry that
	 *    board_boot_order() appends, reached only after every medium in
	 *    u-boot,spl-boot-order failed to yield a bootable FIT. There is
	 *    no next stage for the BootROM to load -- it already loaded this
	 *    SPL from the medium that just failed -- so the board must end
	 *    up in the BootROM's download mode, enumerated on USB as the
	 *    MaskROM device, where a bootloader can be sent over the bad
	 *    one.
	 *
	 * The second case does NOT longjmp into the context save_boot_params()
	 * saved. Measured on ROC-RK3399-PC (2026-09-17): returning
	 * BROM_BOOT_ENTER_DNL through that context after a full SPL run (DM
	 * up, MMU on, MMC/SPI probed, caches flushed and disabled again
	 * first) leaves the BootROM's USB device enumerated but answering
	 * nothing -- the host reads `can't set config #1, error -71`, and no
	 * download goes through. The saved context is only good at ENTRY,
	 * which is exactly where save_boot_params() uses it for the
	 * BOOT_BROM_DOWNLOAD flag. So take that path, the one `reboot
	 * loader` has always taken: leave the flag in the boot-mode register
	 * (PMU domain, survives a global soft reset) and reset. The BootROM
	 * reloads TPL from whatever medium it finds; TPL's save_boot_params()
	 * sees the flag before it has run anything and hands the BootROM to
	 * download mode -- and a medium the BootROM rejects outright lands
	 * there by itself.
	 */
	if (CONFIG_IS_ENABLED(ROCKCHIP_BACK_TO_BROM)) {
		back_to_bootrom(BROM_BOOT_NEXTSTAGE);
		return 0;
	}

	if (!CONFIG_ROCKCHIP_BOOT_MODE_REG)
		return -ENODEV;	/* no flag the next TPL could read */

	puts("Resetting into BootROM download mode...\n");
	writel(BOOT_BROM_DOWNLOAD, CONFIG_ROCKCHIP_BOOT_MODE_REG);
	sysreset_walk_halt(SYSRESET_COLD);

	return -ENODEV;
}

__weak const char * const boot_devices[BROM_LAST_BOOTSOURCE + 1] = {
};

__weak u32 read_brom_bootsource_id(void)
{
	u32 bootsource_id = readl(BROM_BOOTSOURCE_ID_ADDR);

	/* Re-map the raw value read from reg to an existing BROM_BOOTSOURCE
	 * enum value to avoid having to create a larger boot_devices table.
	 */
	if (bootsource_id == 0x81)
		return BROM_BOOTSOURCE_USB;
	else if (bootsource_id > BROM_LAST_BOOTSOURCE)
		log_debug("Unknown bootsource %x\n", bootsource_id);

	return bootsource_id;
}

/*
 * Cache the BootROM's boot source on first use. The old cache lived inside
 * board_spl_was_booted_from() and was filled only when the id mapped to an
 * ofpath -- so a USB (MaskROM) boot, which has no ofpath, re-read IRAM on
 * every call, and any call after a BL31 SRAM segment had landed on IRAM
 * (atf-4 at 0xff8c0000 on RK3399 covers IRAM + 0x10) read that segment's
 * bytes instead. The id is the thing to cache, whatever it maps to.
 * Forced into .data: SPL's BSS is not valid yet when board_boot_order()
 * first asks, and a BSS-resident cache would be zeroed (UNKNOWN) again.
 */
static u32 brom_bootsource_id_cache __section(".data") = BROM_BOOTSOURCE_UNKNOWN;

u32 rockchip_brom_bootsource(void)
{
	if (brom_bootsource_id_cache == BROM_BOOTSOURCE_UNKNOWN)
		brom_bootsource_id_cache = read_brom_bootsource_id();

	return brom_bootsource_id_cache;
}

const char *board_spl_was_booted_from(void)
{
	u32 bootdevice_brom_id = rockchip_brom_bootsource();
	const char *bootdevice_ofpath = NULL;

	if (bootdevice_brom_id < ARRAY_SIZE(boot_devices))
		bootdevice_ofpath = boot_devices[bootdevice_brom_id];

	if (bootdevice_ofpath)
		debug("%s: brom_bootdevice_id %x maps to '%s'\n",
		      __func__, bootdevice_brom_id, bootdevice_ofpath);
	else
		debug("%s: failed to resolve brom_bootdevice_id %x\n",
		      __func__, bootdevice_brom_id);

	return bootdevice_ofpath;
}

#if IS_ENABLED(CONFIG_ROCKCHIP_UNIFIED_IMAGE) && CONFIG_IS_ENABLED(MMC)
/*
 * Where the MMC BootROM reads the idblock from, in 512-byte sectors: the
 * unified image is written there as-is, so its FIT is at that sector plus
 * the byte offset the image itself puts the FIT at (the SPI offset).
 */
#define RK_MMC_IDBLOCK_SECTOR	64

/*
 * One image, two media, one FIT offset: the unified image carries its FIT at
 * CONFIG_SYS_SPI_U_BOOT_OFFS whether it was written to NOR or to MMC, while
 * the compact SD image keeps it at CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR.
 * Look for a FIT header at the compact sector first (the shipped SD layout,
 * one 512-byte read it was about to do anyway), then at the unified one, and
 * say which was taken. Neither present: return the compact sector and let
 * spl_mmc report the failure the way it always has.
 */
unsigned long arch_spl_mmc_get_uboot_raw_sector(struct mmc *mmc,
						unsigned long raw_sect)
{
	const unsigned long unified = RK_MMC_IDBLOCK_SECTOR +
				      CONFIG_SYS_SPI_U_BOOT_OFFS / 512;
	struct blk_desc *bd = mmc_get_blk_desc(mmc);
	ALLOC_CACHE_ALIGN_BUFFER(u8, hdr, 512);

	if (!bd || unified == raw_sect || bd->blksz != 512)
		return raw_sect;

	if (blk_dread(bd, raw_sect, 1, hdr) == 1 &&
	    image_get_magic((struct legacy_img_hdr *)hdr) == FDT_MAGIC)
		return raw_sect;

	if (blk_dread(bd, unified, 1, hdr) == 1 &&
	    image_get_magic((struct legacy_img_hdr *)hdr) == FDT_MAGIC) {
		printf("Unified image: FIT at sector %lu\n", unified);
		return unified;
	}

	return raw_sect;
}
#endif

u32 spl_boot_device(void)
{
	u32 boot_device = BOOT_DEVICE_MMC1;

#if defined(CONFIG_TARGET_CHROMEBOOK_JERRY) || \
		defined(CONFIG_TARGET_CHROMEBIT_MICKEY) || \
		defined(CONFIG_TARGET_CHROMEBOOK_MINNIE) || \
		defined(CONFIG_TARGET_CHROMEBOOK_SPEEDY) || \
		defined(CONFIG_TARGET_CHROMEBOOK_BOB) || \
		defined(CONFIG_TARGET_CHROMEBOOK_KEVIN)
	return BOOT_DEVICE_SPI;
#endif
	if (CONFIG_IS_ENABLED(ROCKCHIP_BACK_TO_BROM))
		return BOOT_DEVICE_BOOTROM;

	if (CONFIG_IS_ENABLED(RAM_DEVICE) &&
	    rockchip_brom_bootsource() == BROM_BOOTSOURCE_USB)
		return BOOT_DEVICE_RAM;

	return boot_device;
}

u32 spl_mmc_boot_mode(struct mmc *mmc, const u32 boot_device)
{
	return MMCSD_MODE_RAW;
}

__weak int board_early_init_f(void)
{
	return 0;
}

__weak int arch_cpu_init(void)
{
	return 0;
}

void board_init_f(ulong dummy)
{
	int ret;

	board_early_init_f();

	ret = spl_early_init();
	if (ret) {
		printf("spl_early_init() failed: %d\n", ret);
		hang();
	}
	arch_cpu_init();

	rockchip_stimer_init();

#ifdef CONFIG_SYS_ARCH_TIMER
	/* Init ARM arch timer in arch/arm/cpu/armv7/arch_timer.c */
	timer_init();
#endif
#if !defined(CONFIG_TPL) || defined(CONFIG_SPL_RAM)
	debug("\nspl:init dram\n");
	ret = dram_init();
	if (ret) {
		printf("DRAM init failed: %d\n", ret);
		return;
	}
	gd->ram_top = gd->ram_base + get_effective_memsize();
	gd->ram_top = board_get_usable_ram_top(gd->ram_size);

	if (IS_ENABLED(CONFIG_ARM64) && !CONFIG_IS_ENABLED(SYS_DCACHE_OFF)) {
		gd->relocaddr = gd->ram_top;
		arch_reserve_mmu();
		enable_caches();
	}
#endif
	preloader_console_init();
}

void spl_board_prepare_for_boot(void)
{
	if (!IS_ENABLED(CONFIG_ARM64) || CONFIG_IS_ENABLED(SYS_DCACHE_OFF))
		return;

	cleanup_before_linux();
}

#if CONFIG_IS_ENABLED(RAM_DEVICE) && IS_ENABLED(CONFIG_SPL_LOAD_FIT)
/* defined in spl-binman.c, outside LTO */
binman_sym_extern(ulong, payload, image_pos);
binman_sym_extern(ulong, payload, size);

static ulong ramboot_load_read(struct spl_load_info *load, ulong sector,
			       ulong count, void *buf)
{
	ulong addr = IF_ENABLED_INT(CONFIG_SPL_LOAD_FIT,
				    CONFIG_SPL_LOAD_FIT_ADDRESS);

	memcpy(buf, map_sysmem(addr + sector, 0), count);
	return count;
}

static int ramboot_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	struct legacy_img_hdr *header;
	ulong addr = IF_ENABLED_INT(CONFIG_SPL_LOAD_FIT,
				    CONFIG_SPL_LOAD_FIT_ADDRESS);
	ulong image_pos = binman_sym(ulong, payload, image_pos);
	ulong size = binman_sym(ulong, payload, size);

	if (addr == CFG_SYS_SDRAM_BASE || addr == CONFIG_SPL_TEXT_BASE)
		return -ENODEV;

	if (image_pos != BINMAN_SYM_MISSING && size != BINMAN_SYM_MISSING) {
		header = map_sysmem(image_pos, 0);
		if (image_get_magic(header) == FDT_MAGIC) {
			memmove(map_sysmem(addr, 0), header, size);
			memset(header, 0, sizeof(*header));
		}
	}

	header = map_sysmem(addr, 0);
	if (image_get_magic(header) == FDT_MAGIC) {
		struct spl_load_info load;

		spl_load_init(&load, ramboot_load_read, NULL, 1);
		return spl_load_simple_fit(spl_image, &load, 0, header);
	}

	return -ENODEV;
}

/* Use priority and method name that sort before default spl_ram_load_image */
SPL_LOAD_IMAGE_METHOD("RAM", 0, BOOT_DEVICE_RAM, ramboot_load_image);
#endif
