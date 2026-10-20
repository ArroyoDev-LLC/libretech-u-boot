// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX TPL -> SPL -> FIT chain: fetching SPL from SD/eMMC (T3).
 *
 * TPL is the stage the BootROM loads, and in the chain it is also the stage
 * that has to go and GET the next one.  Over USB that is a push and needs no
 * loader; from a card it is a pull, and this file is the pull.
 *
 * What it deliberately is not: a call into common/spl/spl_mmc.c.  That file is
 * built around an image FORMAT -- it parses a FIT or a legacy header into a
 * struct spl_image_info -- and around the generic board_init_r() flow.  The
 * DRAM stage is a raw blob at a fixed offset, fetched by a stage that never
 * returns from board_init_f(), so both assumptions would have to be worked
 * around rather than used.  blk_dread() plus blk_dselect_hwpart() is the part
 * of the block layer that is actually wanted, and it is the same call
 * spl_mmc.c itself makes underneath.
 *
 * The layout it reads is the one binman wrote, and it is the SAME layout on
 * every medium: the combined image starts at MESON_GX_MMC_IMAGE_OFFSET (512 on
 * a card, 0 in a boot partition) and SPL sits MESON_GX_SPL_OFFSET into it.
 * Nothing here is medium-specific except the offset of the image itself.
 *
 * eMMC boot0/boot1 needs no register code: u-boot's mmc_startup() has already
 * read EXT_CSD and left BOOT_PARTITION_ENABLE in mmc->part_config, and
 * blk_dselect_hwpart() is the switch.  Asking the card where the ROM was told
 * to boot from is better than a build-time constant, because it is the same
 * answer the ROM acted on.
 *
 * NOTHING IN THIS FILE HAS RUN ON HARDWARE.  It is build-verified only.
 *
 * Design: amlogic/software/boot/gxl/tpl-spl-chain-design.md.
 */

#include <blk.h>
#include <dm.h>
#include <errno.h>
#include <mmc.h>
#include <part.h>
#include <asm/arch/gx-handoff.h>

/**
 * gx_tpl_mmc_hwpart() - which hardware partition holds the image.
 * @mmc:	an initialised device
 *
 * Return: 0 user area, 1 boot0, 2 boot1.
 *
 * MESON_GX_MMC_HWPART pins it; -1 (the default) asks the card.  An SD card has
 * no boot partitions and reports MMCPART_NOAVAILABLE, which extracts to
 * something outside 1..2 and therefore lands on the user area -- so the SD case
 * needs no separate branch, and does not get one.
 */
static int gx_tpl_mmc_hwpart(struct mmc *mmc)
{
	int part;

	if (CONFIG_MESON_GX_MMC_HWPART >= 0)
		return CONFIG_MESON_GX_MMC_HWPART;

	part = EXT_CSD_EXTRACT_BOOT_PART(mmc->part_config);
	if (part == 1 || part == 2)
		return part;

	return 0;
}

/**
 * gx_tpl_load_spl_mmc() - read the DRAM stage off SD/eMMC.
 * @devnum:	u-boot MMC device index (0 = SD, 1 = eMMC on GXL)
 * @dst:	where to put it -- MESON_GX_SPL_ENTRY
 * @len:	how many bytes to read; must be a multiple of the block size
 * @err:	set to a GX_TPL_LOAD_* code on failure, untouched on success
 *
 * Return: bytes read, or 0 on failure (with @err set).
 *
 * @len is a window, not a parsed size: the raw stage carries no header, so TPL
 * reads a fixed span and reports exactly how many bytes it read.  It does not
 * claim to know the image's true length, and the handoff block says so.  The
 * window is bounded by MESON_GX_SPL_MAX_SIZE, which defaults to the whole
 * 0xc000..0x1fe00 padding gap, so it can never reach into the FIT.
 */
int gx_tpl_load_spl_mmc(int devnum, void *dst, u32 len, u32 *err)
{
	struct blk_desc *bd;
	struct mmc *mmc;
	unsigned long image_off;
	lbaint_t sector, count;
	int part, ret;

	ret = mmc_init_device(devnum);
	if (ret) {
		*err = GX_TPL_LOAD_NO_DEV;
		return 0;
	}

	mmc = find_mmc_device(devnum);
	if (!mmc) {
		*err = GX_TPL_LOAD_NO_DEV;
		return 0;
	}

	ret = mmc_init(mmc);
	if (ret) {
		*err = GX_TPL_LOAD_INIT;
		return 0;
	}

	bd = mmc_get_blk_desc(mmc);
	if (!bd) {
		*err = GX_TPL_LOAD_NO_DEV;
		return 0;
	}

	part = gx_tpl_mmc_hwpart(mmc);
	if (blk_dselect_hwpart(bd, part)) {
		*err = GX_TPL_LOAD_HWPART;
		return 0;
	}

	image_off = part ? CONFIG_MESON_GX_MMC_BOOT_IMAGE_OFFSET :
			   CONFIG_MESON_GX_MMC_IMAGE_OFFSET;

	/*
	 * Byte offsets to sectors.  Both constants are Kconfig-documented as
	 * multiples of 512 and the sum is checked here rather than trusted:
	 * an unaligned offset would silently read the wrong bytes, which on a
	 * loader is a hang with no evidence.
	 */
	if ((image_off + CONFIG_MESON_GX_SPL_OFFSET) & (bd->blksz - 1) ||
	    len & (bd->blksz - 1)) {
		*err = GX_TPL_LOAD_ALIGN;
		return 0;
	}

	sector = (image_off + CONFIG_MESON_GX_SPL_OFFSET) >> bd->log2blksz;
	count = len >> bd->log2blksz;

	if (blk_dread(bd, sector, count, dst) != count) {
		*err = GX_TPL_LOAD_READ;
		return 0;
	}

	return len;
}
