/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic G12A/SM1 TPL -> SPL handoff block.
 *
 * Pattern: gx-handoff.h / gxl tpl-spl-chain-design.md.  Addresses are G12 AHB
 * (0xFFFA....), not GXL 0xD900....  Layout:
 *
 *   TPL TEXT  0xFFFA1000  size <= 0xf000  -> ends by 0xFFFB0000
 *   TPL STACK 0xFFFB3000  (grows down)
 *   HANDOFF   0xFFFB3C00  64 B  (gap before aml_ddr scratch @ 0xFFFB4000)
 *   PROG/RPT  0xFFFB3C40 / 0xFFFB3C80
 *   aml_ddr   0xFFFB4000  48 KiB scratch (Strategy A; TPL owns)
 *
 * Design: amlogic/software/boot/g12/s905d3-cc-libre-plan.md L1.2b
 * Pattern: amlogic/software/boot/gxl/tpl-spl-chain-design.md
 */

#ifndef __G12A_HANDOFF_H__
#define __G12A_HANDOFF_H__

#define G12A_XPL_PROG_ADDR		0xFFFB3C40UL
#define G12A_XPL_PROG_HANDOFF		0x0002
#define G12A_XPL_PROG_JUMP		0x0004
#define G12A_XPL_PROG_DRAM_ENTRY	0x0008
#define G12A_XPL_PROG_DRAM_OK		0x0040
#define G12A_XPL_PROG_DRAM_BAD		0x0080

#define G12A_DRAM_STAGE_ADDR		0xFFFB3C80UL
#define G12A_DRAM_FAULT_ADDR		0xFFFB3C84UL
#define G12A_DRAM_STAGE_MAGIC		0x53540000u
#define G12A_DRAM_STAGE_ENTRY		0x80
#define G12A_DRAM_STAGE_OK		0x81
#define G12A_DRAM_STAGE_FIT		0x82
#define G12A_DRAM_STAGE_NO_DM		0x89
#define G12A_DRAM_STAGE_BAD_MAGIC	0x8e
#define G12A_DRAM_STAGE_BAD_VER		0x8d
#define G12A_DRAM_STAGE_BAD_DDR		0x8c
#define G12A_DRAM_STAGE_BAD_SIZE	0x8b
#define G12A_DRAM_STAGE_BAD_ADDR	0x8a

#define G12A_TPL_LOAD_NO_SRC		0x90
#define G12A_TPL_LOAD_NO_MMC		0x91
#define G12A_TPL_LOAD_NO_NOR		0x92
#define G12A_TPL_LOAD_USB_NONE		0x98

/*
 * MESON_G12A_DRAM_STAGE - true only in the SPL phase of a chain build.
 * CONFIG_MESON_G12A_XPL_CHAIN is global (both phases); CONFIG_SPL_BUILD is
 * SPL-phase only.
 */
#if defined(CONFIG_MESON_G12A_XPL_CHAIN) && defined(CONFIG_SPL_BUILD)
#define MESON_G12A_DRAM_STAGE	1
#endif

#define G12A_HANDOFF_ADDR		0xFFFB3C00UL
#define G12A_HANDOFF_MAGIC		0x31484731u	/* '1HG1' LE: G12a HandofF Gen1 */
#define G12A_HANDOFF_VERSION		1

#define G12A_HANDOFF_F_DDR_OK		(1u << 0)
#define G12A_HANDOFF_F_PAYLOAD		(1u << 1)
#define G12A_HANDOFF_F_USB		(1u << 2)
#define G12A_HANDOFF_F_SPL		(1u << 3)
/*
 * The TPL put the boot FIT in DRAM (at CONFIG_SPL_LOAD_FIT_ADDRESS), from USB
 * or from eMMC: the DRAM stage boots it from RAM whatever the BootROM's boot
 * device was.  Set only under CONFIG_MESON_G12A_RESIDENT (g12-resident.c).
 */
#define G12A_HANDOFF_F_FIT_RAM		(1u << 4)

#define G12A_HANDOFF_DRAM_MIN_MB	256u
#define G12A_HANDOFF_DRAM_MAX_MB	8192u

#ifndef __ASSEMBLY__

#include <linux/build_bug.h>
#include <linux/types.h>

/**
 * struct g12a_xpl_handoff - SRAM stage -> DRAM stage (exactly 64 bytes).
 */
struct g12a_xpl_handoff {
	u32 magic;
	u32 version;
	u32 flags;
	u32 dram_size_mb;
	u32 dram_clk_mhz;
	u32 ddr_flags;
	u32 boot_device;
	u32 payload_addr;
	u32 payload_len;
	u32 run_addr;
	u32 spl_addr;
	u32 spl_len;
	u32 train_status;
	u32 pgsr0;
	u32 tune_params;
	u32 progress;
};

/*
 * IMAGE LAYOUT: how much room the DRAM stage actually has, and the check that
 * enforces it.  Same question gx-handoff.h answers for GXL, a different map,
 * and one more bound -- so it is written out here rather than shared.
 *
 * Under MESON_G12A_XPL_CHAIN the combined image is
 *
 *   [ signed SRAM stage ][ SPL @ MESON_G12A_SPL_OFFSET ][ FIT @ MESON_G12A_FIT_OFFSET ]
 *
 * so the SPL's room is FIT_OFFSET - SPL_OFFSET.  As on GXL those are not three
 * independent numbers: MESON_G12A_FIT_OFFSET defaults to SPL_PAD_TO, and
 * SPL_PAD_TO defaults to SPL_MAX_SIZE (common/spl/Kconfig:97), so unless a
 * board overrides them
 *
 *     FIT_OFFSET == SPL_PAD_TO == SPL_MAX_SIZE
 *
 * and the room is SPL_MAX_SIZE - MESON_G12A_SPL_OFFSET.  Read out of a
 * generated .config for aml-s905d3-cc_libre_open_defconfig rather than assumed:
 * SPL_MAX_SIZE 0x40000, SPL_PAD_TO 0x40000, MESON_G12A_FIT_OFFSET 0x40000,
 * MESON_G12A_SPL_OFFSET 0x10000 -- room 0x30000, 196608 B.
 *
 * G12A then has a SECOND bound GXL's assert has no counterpart for, and it is
 * the one that decides what actually reaches DRAM: MESON_G12A_SPL_MAX_SIZE is
 * a FIXED byte count the TPL fetches from SPL_OFFSET, not a parsed header, so
 * an SPL larger than it is truncated at load whatever the image layout allows.
 * Both bounds are 0x30000 today and neither implies the other -- moving
 * FIT_OFFSET without moving the fetch window, or the reverse, breaks exactly
 * one of them -- so both are asserted.
 *
 * The symbol asserted is SPL_SIZE_LIMIT, the post-link size_check on the built
 * binary (top-level Makefile, ignored when 0).  NOT SPL_MAX_SIZE: that is the
 * linker region only and it excludes BSS, and comparing it against
 * FIT_OFFSET - SPL_OFFSET is comparing it against itself minus a constant,
 * which is unsatisfiable for every value.  Same trap, spelled out in
 * gx-handoff.h, where it was measured.
 *
 * SCOPED TO MESON_G12A_XPL_CHAIN because the layout asserted above only exists
 * under the chain: MESON_G12A_SPL_OFFSET and MESON_G12A_SPL_MAX_SIZE are both
 * `depends on MESON_G12A_XPL_CHAIN` (arch/arm/mach-meson/Kconfig), and a
 * single-stage G12A SPL -- which also compiles this header, via spl.c -- has no
 * signed SRAM-stage prefix for those offsets to describe.
 *
 * NOT because a GXL build would otherwise evaluate this.  The earlier text here
 * said arch/arm/mach-meson/spl.c "includes BOTH this header and gx-handoff.h",
 * so a GXL build would see these constants.  It does not: spl.c's two includes
 * are in EXCLUSIVE arms of one conditional -- `#if defined(CONFIG_MESON_G12A)`
 * takes this header, the `#else` takes gx-handoff.h -- so a GXL build never
 * includes this file, and the only other includer, spl-dram-stage.c, is
 * Makefile-gated on MESON_G12A_XPL_CHAIN.  There is no cross-family exposure in
 * either direction; the mirror claim in gx-handoff.h was wrong the same way, and
 * is corrected there.
 */
#if defined(CONFIG_MESON_G12A_XPL_CHAIN) && \
	defined(CONFIG_MESON_G12A_SPL_OFFSET) && \
	defined(CONFIG_SPL_SIZE_LIMIT) && CONFIG_SPL_SIZE_LIMIT > 0
static_assert(CONFIG_SPL_SIZE_LIMIT <=
	      CONFIG_SPL_MAX_SIZE - CONFIG_MESON_G12A_SPL_OFFSET,
	      "image layout: SPL_SIZE_LIMIT claims more room than the image leaves between the SRAM stage and the FIT");
static_assert(CONFIG_SPL_SIZE_LIMIT <= CONFIG_MESON_G12A_SPL_MAX_SIZE,
	      "image layout: SPL_SIZE_LIMIT exceeds the fixed byte count the TPL fetches (MESON_G12A_SPL_MAX_SIZE)");
#endif

#endif /* __ASSEMBLY__ */
#endif /* __G12A_HANDOFF_H__ */
