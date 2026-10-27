// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX TPL -> SPL -> FIT chain: fetching SPL from SPI NOR.
 *
 * The BootROM loaded this stage from NOR, so the next stage comes from NOR
 * too: a board reads every stage from the medium it booted from.  The DRAM
 * stage then reads the FIT from the same part (spl-load-nor.c).
 *
 * The layout is the one binman wrote and the one every other medium carries:
 * u-boot-meson-with-spl.bin starts at MESON_GX_NOR_IMAGE_OFFSET (0, where the
 * BootROM reads it, as in an eMMC boot partition) and SPL sits
 * MESON_GX_SPL_OFFSET into it.
 *
 * Through the bare SPIFC reader (board/libre-computer/common/meson-spifc-nor.c,
 * RDID and READ only): no driver model, no SPI uclass, no write opcode in the
 * stage.  What the reader leaves to its caller:
 *  - the clock gate and the NOR pin function, which the BootROM set to read
 *    this stage and nothing in the TPL has touched since;
 *  - the divider, which the BootROM set for ITS clk81.  spl-gx.c moves clk81
 *    to the run rate before this runs, so the divider is programmed again:
 *    n = 7, clk81 / 8, about 21 MHz at the 166.67 MHz clk81 measured on GX --
 *    the rate the SPL flash session reads NOR at (spl-usbflash.c).
 *
 * Design: amlogic/software/boot/gxl/tpl-spl-chain-design.md.
 */

#include <board-config.h>
#include <linux/build_bug.h>
#include <asm/arch/gx-handoff.h>
#include <asm/arch/usbflash-alts.h>

#if defined(CONFIG_TPL_BUILD)

#define GX_NOR_SPL_OFFSET \
	(CONFIG_MESON_GX_NOR_IMAGE_OFFSET + CONFIG_MESON_GX_SPL_OFFSET)

/* clk81 / (7 + 1): see the file header. */
#define GX_NOR_DIV		7

/**
 * gx_tpl_load_spl_nor() - read the DRAM stage off SPI NOR.
 * @dst:	where to put it -- MESON_GX_SPL_ENTRY
 * @len:	how many bytes to read
 * @err:	set to a GX_TPL_LOAD_* code on failure, untouched on success
 *
 * Return: bytes read, or 0 on failure (with @err set).
 *
 * @len is a window, not a parsed size, exactly as on MMC: the raw stage
 * carries no header.
 */
int gx_tpl_load_spl_nor(void *dst, u32 len, u32 *err)
{
	/* The reader sends a 3-byte address, which reaches 16 MiB. */
	BUILD_BUG_ON(GX_NOR_SPL_OFFSET + CONFIG_MESON_GX_SPL_MAX_SIZE >
		     0x1000000);

	if (!board_nor_clk_enabled() || !board_nor_init()) {
		*err = GX_TPL_LOAD_NOR_BUS;
		return 0;
	}
	board_nor_set_div(GX_NOR_DIV);

	if (board_nor_read(GX_NOR_SPL_OFFSET, dst, len)) {
		*err = GX_TPL_LOAD_NOR_XFER;
		return 0;
	}

	return len;
}

#endif /* CONFIG_TPL_BUILD */
