/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Board configuration block -- platform-neutral interface.
 *
 * See boot/board-config.c and the Libre Computer board-bootloader policy
 * section "SPI NOR config sector".
 *
 * (C) Copyright 2025 Da Xue <da@libre.computer>
 */

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include <linux/types.h>

/*
 * NOR-transport platform hooks. All four have __weak no-op defaults in
 * boot/board-config.c, so a board opts in to only what it needs. They stay
 * board_nor_*() because they are specific to the NOR transport -- the DRAM
 * transport needs no board code at all, the boot host having already placed
 * the block in memory.
 *
 *   board_nor_clk_enabled() -- is the NOR controller clock running?
 *                              default: true
 *   board_nor_init()        -- bring the controller up for raw reads.
 *                              default: true
 *   board_nor_probe()       -- is a flash actually present (JEDEC ID)?
 *                              default: true
 *   board_nor_read()        -- read @len bytes from NOR @offset into @buf,
 *                              0 on success, negative errno otherwise.
 *                              default: -ENOSYS, which makes the NOR
 *                              transport silently no-op.
 *   board_nor_size()        -- capacity of the fitted flash in bytes.
 *                              default: 0, meaning "this board cannot say",
 *                              which is NOT a size and is never guessed at.
 *                              What a board that cannot say gets is decided in
 *                              exactly one place, board_config_nor_offset().
 *                              Valid only once board_nor_probe() has run.
 */
bool board_nor_clk_enabled(void);
bool board_nor_init(void);
bool board_nor_probe(void);
int board_nor_read(u32 offset, void *buf, u32 len);
u32 board_nor_size(void);

/**
 * board_config_nor_skip() - must this boot ignore the stored NOR config?
 *
 * A boot POLICY, not a hardware question: the four hooks above say whether
 * the NOR can be read; this one says whether its config block may be acted
 * on. True makes board_config_nor_overlays() apply no stored overlay, before
 * it touches the controller. __weak default: false, which is exactly the
 * behaviour without the hook.
 *
 * Kept apart from board_nor_clk_enabled() on purpose: that one also decides
 * whether a board's DFU alt map offers the config block for WRITING, and a
 * flash session is precisely the boot that must still be able to write it.
 *
 * Scope: the overlay half only, which the generic fdtdec_board_setup()
 * calls with no board code in between. The env half,
 * board_config_env_import(), is called from a board's misc_init_r(), which
 * gates it there.
 */
bool board_config_nor_skip(void);

#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
/**
 * board_config_nor_offset() - where the config block sits on the NOR transport
 *
 * The single answer to that question, for readers and writers alike. The read
 * path (boot/board-config.c) and the write path (a board's DFU alt map, its
 * capsule FMP target) must name the same bytes, and the only way to guarantee
 * that is for both to ask here rather than to each carry a constant that
 * someone has to remember to update in two places.
 *
 * Either the board pinned it (CONFIG_BOARD_CONFIG_NOR_OFFSET_PIN), or it is
 * the last block-sized region (CONFIG_BOARD_CONFIG_NOR_SIZE, 64 KiB by
 * default) of the device that 3-byte addressing can reach:
 *
 *	offset = min(board_nor_size(), 16 MiB) - CONFIG_BOARD_CONFIG_NOR_SIZE
 *
 * A board that cannot report a capacity -- board_nor_size() is __weak 0, and a
 * backend that only exists after driver-model init has nothing to answer with
 * at the pre-relocation overlay phase -- takes the 16 MiB term, i.e. the
 * default is the last block of the addressable window (0xFF0000 with the
 * default block size). Detection failing is NOT a refusal to place the block;
 * it is the same placement every part large enough to raise the question gets.
 * A board for which that is wrong states so with a pin, which always wins.
 *
 * Call only after board_nor_probe() has succeeded -- that is what makes
 * board_nor_size() valid.
 *
 * Return: the byte offset. Never 0: offset 0 is where the boot image starts on
 *	   every board in tree, so it is not a location this can legitimately
 *	   name, and a caller never has to handle a "no answer" case.
 */
u32 board_config_nor_offset(void);
#endif

#if IS_ENABLED(CONFIG_BOARD_CONFIG)
/**
 * board_config_env_import() - import key=value overrides from the board config
 *
 * Runs every enabled transport: the NOR config sector first (when
 * CONFIG_BOARD_CONFIG_NOR is set), then a host-staged DRAM block on top
 * (when CONFIG_BOARD_CONFIG_DRAM is set), so a USB boot's
 * overrides win over what NOR says.
 *
 * Call from misc_init_r(), i.e. after the environment proper has been loaded:
 * values found here override what is already set. board_r.c runs
 * initr_env() -> misc_init_r() -> board_late_init(); every in-tree caller uses
 * misc_init_r(), because board_late_init() on some platforms belongs to shared
 * SoC code that returns early on several paths.
 *
 * Return: number of variables imported, 0 if none.
 */
int board_config_env_import(void);
#else
static inline int board_config_env_import(void)
{
	return 0;
}
#endif

#endif /* __BOARD_CONFIG_H */
