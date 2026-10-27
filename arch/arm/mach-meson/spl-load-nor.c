// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX DRAM stage: reading the FIT from SPI NOR.
 *
 * The second half of a NOR boot (tpl-load-nor.c is the first): the TPL read
 * this stage from NOR, so this stage reads the FIT from NOR.  It is the
 * BOOT_DEVICE_SPI load method of the generic SPL flow -- spl_load() parses the
 * FIT, exactly as spl_mmc and spl_ram do on the other media -- with the bare
 * SPIFC reader (board/libre-computer/common/meson-spifc-nor.c, RDID and READ
 * only) as its read callback.
 *
 * Not common/spl/spl_spi.c: that brings the SPI flash stack, and with it a
 * write path, into the stage, and the SPL flash session requires there be
 * none (spl-usbflash.c refuses to build with one).  NOR stays read-only in
 * every xPL build.
 *
 * The FIT sits MESON_GX_FIT_OFFSET into the combined image, which starts at
 * MESON_GX_NOR_IMAGE_OFFSET.  The clock, gate and pin function are as the TPL
 * left them; the divider is set again so this stage does not depend on that.
 *
 * Design: amlogic/software/boot/gxl/tpl-spl-chain-design.md.
 */

#include <board-config.h>
#include <errno.h>
#include <spl.h>
#include <spl_load.h>
#include <asm/arch/gx-handoff.h>
#include <asm/arch/usbflash-alts.h>

#if defined(MESON_GX_DRAM_STAGE)

#define GX_NOR_FIT_OFFSET \
	(CONFIG_MESON_GX_NOR_IMAGE_OFFSET + CONFIG_MESON_GX_FIT_OFFSET)

/* clk81 / (7 + 1), as tpl-load-nor.c. */
#define GX_NOR_DIV		7

static ulong gx_spl_nor_read(struct spl_load_info *load, ulong off,
			     ulong count, void *buf)
{
	return board_nor_read(off, buf, count) ? 0 : count;
}

static int gx_spl_load_nor(struct spl_image_info *spl_image,
			   struct spl_boot_device *bootdev)
{
	struct spl_load_info load;

	if (!board_nor_clk_enabled() || !board_nor_init()) {
		puts("NOR: controller not up\n");
		return -ENODEV;
	}
	board_nor_set_div(GX_NOR_DIV);

	spl_load_init(&load, gx_spl_nor_read, NULL, 1);

	return spl_load(spl_image, bootdev, &load, 0, GX_NOR_FIT_OFFSET);
}
SPL_LOAD_IMAGE_METHOD("NOR", 0, BOOT_DEVICE_SPI, gx_spl_load_nor);

#endif /* MESON_GX_DRAM_STAGE */
