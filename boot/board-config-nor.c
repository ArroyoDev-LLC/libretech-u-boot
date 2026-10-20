// SPDX-License-Identifier: GPL-2.0+
/*
 * Where the board config block sits on the NOR transport.
 *
 * Split out of boot/board-config.c so that every phase which needs the
 * offset computes it with this one function: U-Boot proper's config-block
 * reader and its DFU `config` alternate (board-meson.c), and the G12A/SM1 SPL
 * flash session's read-only `config` alternate (arch/arm/mach-meson/
 * spl-usbflash.c).  Built in proper with CONFIG_BOARD_CONFIG_NOR, and in an
 * xPL phase only when that phase's DFU has the virtual back end
 * (CONFIG_$(PHASE_)DFU_VIRT) it serves NOR through.
 */

#include <board-config.h>
#include <linux/compiler.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
#define NOR_SECTOR_SIZE		CONFIG_BOARD_CONFIG_NOR_SIZE

/*
 * One past the highest byte a 3-byte (24-bit) NOR address can name. This is
 * not a property of any particular part: it is the width of the address phase
 * a transport has before the spi-nor layer exists to negotiate 4-byte mode,
 * and the NOR transport's first read happens at initcall #2. A part larger
 * than this is not smaller, but the top of it is unreachable, so "the end of
 * the device" means the end of THIS window.
 */
#define BOARD_CONFIG_NOR_ADDR_LIMIT	0x1000000

/*
 * 0 means "this board cannot tell you", which is deliberately distinct from
 * any size. A board that does not know must not be handed a plausible-looking
 * number here: board_config_nor_offset() is the one place allowed to decide
 * what a board that cannot say gets, and it must be able to see that the
 * answer was absent rather than small.
 */
__weak u32 board_nor_size(void)
{
	return 0;
}

/* See the kerneldoc in include/board-config.h. */
u32 board_config_nor_offset(void)
{
#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR_OFFSET_PIN)
	/*
	 * The board pinned it, and a pin always wins. Either it cannot report
	 * a capacity at the point the block is read, or its flash layout puts
	 * something else where the computed offset would land.
	 */
	return CONFIG_BOARD_CONFIG_NOR_OFFSET;
#else
	u32 size = board_nor_size();

	/*
	 * Either the part is bigger than 3-byte addressing can reach, or it did
	 * not report a size at all, or it reported one too small to hold the
	 * block. All three collapse to the same answer: the LAST BLOCK OF THE
	 * ADDRESSABLE WINDOW, 0xFF0000 with the default 64 KiB block.
	 *
	 * The unknown case takes the ceiling rather than declining, because the
	 * ceiling is the one offset that is right for every part large enough
	 * for the question to arise -- 16 MiB and up all compute to it. A part
	 * SMALLER than the window that cannot be measured is the only case this
	 * gets wrong, and such a part has nowhere correct to put the block
	 * anyway; a board in that position pins.
	 *
	 * "size too small" is folded in on purpose. A density code that yields
	 * a part with no room for the block is not a smaller answer, it is an
	 * implausible one -- the byte came off the wire -- so it is treated as
	 * no answer rather than as a measurement to obey.
	 */
	if (!size || size > BOARD_CONFIG_NOR_ADDR_LIMIT ||
	    size <= NOR_SECTOR_SIZE)
		size = BOARD_CONFIG_NOR_ADDR_LIMIT;

	return size - NOR_SECTOR_SIZE;
#endif
}
#endif /* CONFIG_BOARD_CONFIG_NOR */
