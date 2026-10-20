/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic GX (GXBB/GXL) TPL -> SPL handoff block.
 *
 * The BootROM-loaded stage runs from SRAM under a hard 0xb000 payload ceiling;
 * everything after it runs from DRAM and has no comparable limit.  TPL owns DDR
 * init and the DWC2 receive path, then hands the next stage what it learned --
 * as a struct in SRAM, not in registers, because SRAM survives the jump and a
 * register ABI cannot be versioned.
 *
 * Region: the SRAM window the bring-up breadcrumbs already live in.  WHICH
 * RANGES ARE CLAIMED IS NO LONGER A LIST IN THIS COMMENT -- it is the
 * GX_SRAM_*_ADDR/LEN pairs below, which are the definitions the code uses AND
 * the thing the adjacency asserts check.  A hand-maintained list is what
 * drifted; see "THE SRAM BREADCRUMB MAP" below.  Individual WORDS inside a
 * named range are still documented at their own definitions:
 *
 *   0xd900c100         'OMOR' marker            (meson-usb.c)
 *   0xd900c104         progress word            (meson-usb.c, see below)
 *   0xd900c108-c124    DWC2 register snapshots  (meson-usb.c)
 *   0xd900c114         N8 request               (spl.c GXL_N8_REQ_ADDR)
 *   0xd900c120         stage byte 0x5354xxxx    (spl.c prod_stage)
 *   0xd900c128-c160    DDR train step / PGSR0 / DXnGSR0 / GTR / magic
 *   0xd900c168-c16c    bulk rate report
 *   0xd900c170-c174    THIS file: DRAM-stage report word + faulting value (T2)
 *   0xd900c180         EP info / bulk DMA scratch (spl-boot-debug.S, meson-usb.c)
 *   0xd900c1a4         FIFO rung request/echo   (meson-usb.c)
 *
 * Progress word (0xd900c104) -- audited 2026-08-07.  The rule this comment used
 * to state was "breadcrumb flags MUST stay in the LOW 16 bits, because ep0_tx()
 * stashes a GNPTXSTS snapshot in bits 31:16".  That is half right, and both
 * halves of the error have cost design decisions, so it is written out in full:
 *
 *   - The clobber is CONDITIONAL, not unconditional.  ep0_tx() (meson-usb.c)
 *     replaces bits 31:16 only on its slave-PIO path; once bulk_dma_enter() has
 *     set bulk_dma_mode, ep0_tx() takes an early return and never writes the
 *     word again.  So every stage that runs after the DWC2 session -- JUMP,
 *     DRAM_ENTRY, DRAM_OK, DRAM_BAD, HANDOFF -- has all 16 high bits available.
 *     "There is no free bit" is false: there are 16, for post-USB stages.
 *
 *   - What the high half is NOT is CLEAN.  The last PIO ep0_tx() leaves a live
 *     GNPTXSTS snapshot there (low 12 bits = NP TX FIFO free dwords), so a flag
 *     OR'd into 31:16 afterwards cannot be told from a stale snapshot bit.
 *     U1P_DMA_ON/_STALL sit in bits 16 and 18 and have exactly this defect.
 *     Fixing it properly means giving the snapshot its own SRAM word.
 *
 *   - The LOW half is not durable either, which the old rule implied it was.
 *     meson_usb_reenum_serve(), meson_usb_acm_serve() and
 *     meson_usb_receive_fit() each do a full-word
 *     writel(U1P_START | U1P_WORLDCUP, ...) that wipes ALL 32 bits.  A flag
 *     survives because it was written after the last whole-word store, not
 *     because of which half it is in.  The invariant is TEMPORAL, not spatial.
 *
 * GX_XPL_PROG_* stay in the low half -- not because the high half is forbidden,
 * but because they are already allocated there and moving them buys nothing.
 * Only the low half is copied into ::progress, which is why the DMA flags never
 * reach a consumer in this tree at all.
 *
 * Design: amlogic/software/boot/gxl/tpl-spl-chain-design.md, "Handoff block".
 */

#ifndef __GX_HANDOFF_H__
#define __GX_HANDOFF_H__

/* Host-visible progress word, shared with meson-usb.c's U1P_* bits. */
#define GX_XPL_PROG_ADDR	0xd900c104UL
#define GX_XPL_PROG_HANDOFF	0x0002		/* see the progress-word note above */
#define GX_XPL_PROG_JUMP	0x0004		/* SRAM stage is about to branch */
#define GX_XPL_PROG_DRAM_ENTRY	0x0008		/* DRAM stage reached _start */
#define GX_XPL_PROG_DRAM_OK	0x0040		/* DRAM stage accepted the block */
#define GX_XPL_PROG_DRAM_BAD	0x0080		/* DRAM stage REFUSED the block */

/*
 * DRAM-stage report, in SRAM because that is the only memory both stages and a
 * later BootROM read agree on.  The high half is the same 0x5354 'ST' tag the
 * SRAM stage's own stage byte @0xd900c120 uses, so a host that reads garbage
 * (or reads before anything ran) can tell.  The second word carries the value
 * that failed, because "version mismatch" without the version is a bug report
 * nobody can act on.
 */
#define GX_DRAM_STAGE_ADDR	0xd900c170UL
#define GX_DRAM_FAULT_ADDR	0xd900c174UL
#define GX_DRAM_STAGE_MAGIC	0x53540000u
#define GX_DRAM_STAGE_ENTRY	0x80	/* running from DRAM */
#define GX_DRAM_STAGE_OK	0x81	/* handoff block accepted */
/*
 * 0x82: the handoff was accepted AND the stage has joined the generic xPL
 * flow -- spl_early_init(), spl_init(), console -- so board_init_r() is next
 * and the FIT is being loaded by common/spl/, exactly as the single-stage SPL
 * does it.  Distinct from OK on purpose: OK means "the block validated", 0x82
 * means "everything after the block is now somebody else's code", and a board
 * that dies with 0x82 in the word died in the shared loader, not in the seam.
 */
#define GX_DRAM_STAGE_FIT	0x82	/* entering the shared FIT/ATF path */
#define GX_DRAM_STAGE_NO_DM	0x89	/* spl_early_init() failed; fault = its return */
#define GX_DRAM_STAGE_BAD_MAGIC	0x8e	/* fault word = magic as read */
#define GX_DRAM_STAGE_BAD_VER	0x8d	/* fault word = version as read */
#define GX_DRAM_STAGE_BAD_DDR	0x8c	/* fault word = flags as read */
#define GX_DRAM_STAGE_BAD_SIZE	0x8b	/* fault word = dram_size_mb as read */
#define GX_DRAM_STAGE_BAD_ADDR	0x8a	/* fault word = spl_addr as read */

/*
 * SPI NOR read-only probe (tpl-nor-probe.c), in the two words above the
 * DRAM-stage report and below the DWC2 DMA scratch at 0xd900c180.  Its own
 * pair rather than the DRAM stage's, because the probe is a DIAGNOSTIC that
 * runs before the boot proceeds: overwriting the stage verdict with it would
 * destroy the one word a host reads after a dead boot.
 *
 * GX_NOR_PROBE_ID_ADDR carries the 24-bit JEDEC ID on success, and the failing
 * return value otherwise -- an ID nobody can read is not evidence.
 */
#define GX_NOR_PROBE_ADDR	0xd900c178UL
#define GX_NOR_PROBE_ID_ADDR	0xd900c17cUL
#define GX_NOR_PROBE_MAGIC	0x53540000u
#define GX_NOR_PROBE_START	0xa0	/* probe entered */
#define GX_NOR_PROBE_OK		0xa1	/* ID read AND content read */
#define GX_NOR_PROBE_NOBUS	0xae	/* no SPI bus 0 / no chip select 0 */
#define GX_NOR_PROBE_NOCLK	0xad	/* claim failed -- set_speed refused */
#define GX_NOR_PROBE_XFER	0xac	/* a transfer returned an error */

/*
 * Why TPL could not fetch SPL (T3).  Reported through the same
 * GX_DRAM_STAGE_ADDR/GX_DRAM_FAULT_ADDR pair as the DRAM stage's own verdict,
 * because there is exactly one question a host asks after a dead boot -- "how
 * far did it get" -- and one word to read is better than two.  The ranges do
 * not overlap: 0x8x is the DRAM stage speaking, 0x9x is TPL failing to reach
 * it.  The fault word carries the boot device id, so a report names both the
 * failure and the medium it happened on.
 */
#define GX_TPL_LOAD_NO_SRC	0x90	/* boot device has no loader at all */
#define GX_TPL_LOAD_NO_MMC	0x91	/* booted SD/eMMC, TPL_MESON_GX_LOAD_MMC off */
/*
 * 0x92: booted SPI NOR.  Not a link failure -- meson_spifc DOES build in TPL
 * (measured 2026-08-07, 38475 B).  The blocker is meson_spifc_set_speed()
 * needing clk_get_rate(), which has no meaning without the CLK uclass.  See
 * TPL_MESON_GX_LOAD_MMC in arch/arm/mach-meson/Kconfig.
 */
#define GX_TPL_LOAD_NO_NOR	0x92	/* booted SPI NOR, no loader for it */
#define GX_TPL_LOAD_NO_DEV	0x93	/* the block device did not appear */
#define GX_TPL_LOAD_INIT	0x94	/* mmc_init() failed */
#define GX_TPL_LOAD_HWPART	0x95	/* boot0/boot1 switch refused */
#define GX_TPL_LOAD_ALIGN	0x96	/* offset or length not a whole sector */
#define GX_TPL_LOAD_READ	0x97	/* short read */
#define GX_TPL_LOAD_USB_NONE	0x98	/* USB session delivered nothing to SPL_ENTRY */

/* Sticky DDR breadcrumbs the block forwards (written by dram-gx.c). */
#define GX_DDR_STICKY_STEP_ADDR	0xd900c128UL
#define GX_DDR_STICKY_PGSR_ADDR	0xd900c12cUL

/* Pre-DDR knob block the host may write before RUN; 0 in the block if absent. */
#define GX_TUNE_PARAMS_ADDR	0xd900c040UL

/*
 * ======================================================================
 * THE SRAM BREADCRUMB MAP -- one owner, machine-checked.
 * ======================================================================
 *
 * The comment block at the top of this file used to BE the map: a
 * hand-maintained list, in ONE file, of addresses #defined in four OTHERS --
 * meson-usb.c, spl.c, dram-gx.c and spl-boot-debug.S.  A list that names
 * addresses it does not define drifts, and this one has: it still says
 * "0xd900c180-c240  DWC2 DMA scratch + EP0 DMA buffers, bulk/FIFO requests"
 * as though that were one region with one owner, and inside it
 * FIFO_REQ_ADDR (0xd900c1a4, meson-usb.c) lies INSIDE EP0_DMA_SETUP
 * (0xd900c1a0, 24 B, meson-usb.c) -- at the wIndex/wLength of the first
 * staged SETUP packet, so one control transfer in buffer-DMA mode would
 * overwrite the FIFO rung request.  That is latent only because
 * bulk_dma_enter() is unreachable, which is luck, not a bound.
 *
 * WHY THIS IS NOT THREE #defines.  The obvious fix -- a _Static_assert that
 * the ranges do not overlap -- CANNOT BE WRITTEN AT THE .c SITES.  The
 * claims live in four translation units with no shared header, so no assert
 * in any one of them can see two of them; and the collision that matters
 * most on a chain build is between a USB buffer and a DRAM-stage sticky,
 * which are in different files by construction.  Moving an address makes the
 * assert TRUE without making it EXPRESSIBLE.
 *
 * So the map lives HERE, as GX_SRAM_<NAME>_ADDR/LEN pairs -- the definitions
 * the code uses AND the thing the asserts check -- and this header carries
 * one adjacency assert per adjacent pair.  Each .c keeps its own struct or
 * buffer and asserts only `sizeof(x) <= GX_SRAM_<X>_LEN`, the half that has
 * to stay local because the object is local.  That split is what makes the
 * check total: a range cannot overlap its neighbour (checked here) and an
 * object cannot outgrow its range (checked there).
 *
 * NOT a description of the hardware: SRAM does not enforce any of this.  It
 * is the allocation, and the asserts are what make it one.
 *
 * THE BOUNDS, and why the top one is a VENDOR constant rather than a Kconfig
 * symbol.  The bottom is ours: CONFIG_SPL_TEXT_BASE + CONFIG_SPL_MAX_SIZE =
 * 0xd9001000 + 0xb000 = 0xd900c000, below which is the loaded image itself.
 * The top is the vendor's, and it is 0xd900c400, not 0xd900d000.  The GXL
 * BROM's own SRAM map (Amlogic BSP v2015 arch-gxl/cpu_config.h) reads:
 *
 *     0xD900_C000 - 0xD900_C3FF    1K    Reserved              (:33)
 *     0xD900_C400 - 0xD900_C7FF    1K    NAND, USB Buffer      (:32)
 *     0xD900_C800 - 0xD901_2FFF   26K    BL1 Stack, BSS        (:31)
 *
 * with RAM_START 0xD9000000 (:36), BL1_RAM_START = RAM_START + 0xC000 (:38),
 * BL1_NAND_BUFF = BL1_RAM_START + 1024 = 0xD900C400 (:50) and BSS_START =
 * BL1_RAM_START + 2048 = 0xD900C800 (:41).  The whole window sits in the AHB
 * SRAM block, 0xD9000000-0xD9013FFF, 80K (s805x datasheet ADDRSPACE line).
 * ONE kilobyte of that -- 0xd900c000-0xd900c3ff -- is the only part the
 * vendor BL1 does not name for itself, and a stage that re-enters the BROM
 * for USB is running beside a ROM whose buffer and stack are the next two
 * kilobytes up.  So the map is bounded by what the vendor left free, not by
 * where our own .bss happens to start on one profile.
 *
 * (The chain/instrument lineage places its EP0 DMA trio at 0xd900cb00
 * because six DRAM-instrument stickies claim 0xd900c200-0xd900ca00 there.
 * Those stickies do not exist on this branch, so the trio goes in the
 * vendor-Reserved kilobyte instead and the map stays inside it.  When the
 * two maps are reconciled, GX_SRAM_EP0_DMA_ADDR is the one line that
 * differs, and it differs for a reason each side can state.)
 *
 * Individual WORDS inside a named range stay documented at their own
 * definitions -- GX_DRAM_STAGE_ADDR, GX_XPL_PROG_ADDR, FIFO_REQ_ADDR and the
 * DDR sticky block are words, not ranges, and the map does not restate them.
 * A range is minted when two different FILES need to be kept off each other.
 */
#define GX_SRAM_BROM_PARAM_ADDR		0xd900c000UL	/* ROM ritual: host zeroes it before RUN (tools/meson-usb-boot.py) */
#define GX_SRAM_BROM_PARAM_LEN		0x40UL
#define GX_SRAM_TUNE_ADDR		0xd900c040UL	/* GX_TUNE_PARAMS_ADDR -- struct spl_tune_params, declared, unimplemented */
#define GX_SRAM_TUNE_LEN		0x80UL
#define GX_SRAM_HANDOFF_ADDR		0xd900c0c0UL	/* GX_HANDOFF_ADDR -- struct gx_xpl_handoff */
#define GX_SRAM_HANDOFF_LEN		0x40UL
#define GX_SRAM_BREADCRUMB_ADDR		0xd900c100UL	/* 'OMOR', progress word, DWC2 + DDR snapshots, bulk rate, stage report pair */
#define GX_SRAM_BREADCRUMB_LEN		0x78UL
/*
 * The SPI NOR probe's own two words.  This pair is why the map could not be
 * copied from the instrument lineage unchanged: tpl-nor-probe.c exists only
 * on this branch, and its report pair is the LAST two words of what the
 * instrument's map calls one 0x80-long breadcrumb range.  Left unnamed they
 * read as free, and the map's whole guarantee is that a new claim has to
 * displace an existing one to fit.  So BREADCRUMB ends at 0xd900c178 and the
 * probe owns the rest.
 */
#define GX_SRAM_NOR_PROBE_ADDR		0xd900c178UL	/* GX_NOR_PROBE_ADDR + GX_NOR_PROBE_ID_ADDR */
#define GX_SRAM_NOR_PROBE_LEN		0x08UL
#define GX_SRAM_USB_REQ_ADDR		0xd900c180UL	/* BULK_DMA_EP0_SCRATCH, GXL_SPL_EP_INFO, FIFO rung request/echo */
#define GX_SRAM_USB_REQ_LEN		0x80UL
/*
 * The EP0 DMA staging trio, moved here from 0xd900c1a0/0xd900c1c0/0xd900c200.
 * Moving it is what resolves the FIFO_REQ_ADDR collision: the overlap was a
 * consequence of where EP0_DMA_SETUP sat, not of where FIFO_REQ_ADDR sat, so
 * the rung word does not move and no host tool has to be re-pinned.  This
 * region survives nothing and needs to survive nothing -- it is live only
 * inside one DWC2 session.
 */
#define GX_SRAM_EP0_DMA_ADDR		0xd900c200UL
#define GX_SRAM_EP0_DMA_LEN		0xa0UL
#define GX_SRAM_EP0_DMA_SETUP_ADDR	0xd900c200UL	/* 3 back-to-back SETUPs */
#define GX_SRAM_EP0_DMA_SETUP_LEN	0x20UL
#define GX_SRAM_EP0_DMA_OUT_ADDR	0xd900c220UL	/* control data stage OUT */
#define GX_SRAM_EP0_DMA_OUT_LEN		0x40UL
#define GX_SRAM_EP0_DMA_IN_ADDR		0xd900c260UL	/* control data/status IN */
#define GX_SRAM_EP0_DMA_IN_LEN		0x40UL
/*
 * The boot log's first home (gx-bootlog.h): the TPL's console bytes from its
 * first line until DDR is up, when the log moves to DRAM and this header is
 * left saying where it went.  The DRAM stage reads that header to find the
 * log, so it must survive the TPL -> SPL jump, which SRAM does.
 */
#define GX_SRAM_BOOTLOG_ADDR		0xd900c2a0UL
#define GX_SRAM_BOOTLOG_LEN		0x160UL
/* End of the vendor-Reserved kilobyte; 0xd900c400 is BL1_NAND_BUFF. */
#define GX_SRAM_WINDOW_END		0xd900c400UL

/*
 * MESON_GX_DRAM_STAGE - true only in the SPL phase of a chain build.
 *
 * CONFIG_MESON_GX_XPL_CHAIN is a global symbol, so it is set in BOTH phases of
 * a chain build; CONFIG_SPL_BUILD is what scripts/Makefile.xpl adds for the SPL
 * phase alone (CONFIG_XPL_BUILD covers any xPL stage and would match TPL too).
 * The pair is the only way to say "the stage that runs from DRAM" -- and it
 * must stay false for the shipping single-stage SPL, which is the same phase
 * built without the chain and still owns DDR init and the DWC2 server.
 */
#if defined(CONFIG_MESON_GX_XPL_CHAIN) && defined(CONFIG_SPL_BUILD)
#define MESON_GX_DRAM_STAGE	1
#endif

#define GX_HANDOFF_ADDR		0xd900c0c0UL
#define GX_HANDOFF_MAGIC	0x48544c47u	/* 'GLTH' LE: GxL Tpl Handoff */
#define GX_HANDOFF_VERSION	1

/* ::flags -- what this stage actually completed */
#define GX_HANDOFF_F_DDR_OK	(1u << 0)	/* dram_init() returned 0 */
#define GX_HANDOFF_F_PAYLOAD	(1u << 1)	/* payload_addr/len are valid */
#define GX_HANDOFF_F_USB	(1u << 2)	/* payload arrived over DWC2 */
#define GX_HANDOFF_F_SPL	(1u << 3)	/* spl_addr is where TPL branched */

/*
 * What the DRAM stage will accept in ::dram_size_mb.  Not a probe and not a
 * guess at the board: it is the range outside which the word cannot be a
 * trained size at all, so a zeroed or stale block is rejected instead of being
 * carried forward as fact.  GXL parts ship 512 MB to 4 GB.
 */
#define GX_HANDOFF_DRAM_MIN_MB	256u
#define GX_HANDOFF_DRAM_MAX_MB	4096u

/* ::ddr_flags -- the Kconfig DDR geometry TPL trained with */
#define GX_HANDOFF_DDR_2T	(1u << 0)
#define GX_HANDOFF_DDR_DDR4	(1u << 1)
#define GX_HANDOFF_DDR_DQS_CORR	(1u << 2)
#define GX_HANDOFF_DDR_1RANK	(1u << 3)
#define GX_HANDOFF_DDR_2RANK_ID	(1u << 4)	/* two identical ranks */
#define GX_HANDOFF_DDR_2RANK_DF	(1u << 5)	/* two different ranks */
#define GX_HANDOFF_DDR_16BIT	(1u << 6)	/* one 16-bit rank */

#ifndef __ASSEMBLY__

#include <linux/build_bug.h>
#include <linux/types.h>

/**
 * struct gx_xpl_handoff - what the SRAM stage tells the DRAM stage.
 *
 * Exactly 64 bytes, all u32: the block sits between two already-claimed SRAM
 * windows, and every consumer (SPL, and the host over BROM readMemory) reads it
 * as a flat little-endian word array.  Grow it by bumping @version and moving
 * the block, never by spilling past 0xd900c0ff.
 *
 * @magic:	GX_HANDOFF_MAGIC.  Written LAST, so a torn block never validates.
 * @version:	GX_HANDOFF_VERSION.  SPL refuses a mismatch rather than guessing.
 *		There is no separate size word: at 64 B the block is exactly the
 *		gap it sits in, so a reader that needs more is reading a
 *		different version at a different address anyway.
 * @flags:	GX_HANDOFF_F_*.
 * @dram_size_mb: what TPL's size detection measured, in MB.
 * @dram_clk_mhz: CONFIG_DRAM_CLK it trained at.
 * @ddr_flags:	GX_HANDOFF_DDR_*.
 * @boot_device: meson_get_boot_device() -- the ROM boot id, which picks the
 *		load path if SPL ever has to fetch something itself.
 * @payload_addr: where the received image landed, 0 if none.
 * @payload_len: its length in bytes, 0 if none.
 * @run_addr:	address the host named in REQ_RUN_IN_ADDR, 0 if it never did.
 * @spl_addr:	address TPL branched to, 0 if it did not branch.  The DRAM stage
 *		checks this against its own link address: a TPL and an SPL built
 *		for different addresses is the one mismatch that would otherwise
 *		present as a silent hang.
 * @spl_len:	its length in bytes, or 0 for "TPL did not load it and does not
 *		know".  0 is the honest value while the host writes the DRAM
 *		stage itself (T2); it becomes real when TPL fetches SPL (T3).
 * @train_status: DDR sticky step word @0xd900c128 (0x5354d4ss), for provenance
 *		in a failure report -- SPL forwards it, it does not act on it.
 * @pgsr0:	last PUB PGSR0 @0xd900c12c, same purpose.
 * @tune_params: address of the pre-DDR knob block, 0 if none was supplied.
 * @progress:	low 16 bits of the USB progress word.  The high half is dropped
 *		because ep0_tx() may have left a GNPTXSTS snapshot there; that
 *		also means U1P_DMA_ON/_STALL never reach this struct.
 */
struct gx_xpl_handoff {
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
 * The gap between the tune-param block and the 'OMOR' marker is exactly this
 * big.  Both checks are here so a field added without moving the block fails
 * the build instead of silently overwriting the first breadcrumb.
 */
static_assert(sizeof(struct gx_xpl_handoff) == 64,
	      "gx_xpl_handoff must stay 64 B");
static_assert(GX_HANDOFF_ADDR + 64 <= 0xd900c100UL,
	      "gx_xpl_handoff must not reach the 0xd900c100 breadcrumbs");

/*
 * The map, checked.  One assert per ADJACENT PAIR, in address order, plus the
 * two bounds at the ends.  Written as a chain rather than as an all-pairs
 * matrix because the ranges are ordered: if every range ends at or before the
 * next one starts, no two can overlap, and a chain says WHICH PAIR broke
 * instead of which of N(N-1)/2 combinations did.
 *
 * Adding a range means inserting its pair of macros AND its link in this
 * chain.  A range added without its link is the one thing this does not
 * catch -- which is why the chain covers the whole span with no holes: every
 * byte from BROM_PARAM to GX_SRAM_WINDOW_END is inside some named range, so a
 * new claim has to displace an existing one to fit.
 */
#define GX_SRAM_ADJACENT(a, b)						\
	static_assert(GX_SRAM_##a##_ADDR + GX_SRAM_##a##_LEN <=		\
		      GX_SRAM_##b##_ADDR,				\
		      "SRAM map: " #a " overlaps " #b			\
		      " -- see the breadcrumb map in gx-handoff.h")

GX_SRAM_ADJACENT(BROM_PARAM, TUNE);
GX_SRAM_ADJACENT(TUNE, HANDOFF);
GX_SRAM_ADJACENT(HANDOFF, BREADCRUMB);
GX_SRAM_ADJACENT(BREADCRUMB, NOR_PROBE);
GX_SRAM_ADJACENT(NOR_PROBE, USB_REQ);
GX_SRAM_ADJACENT(USB_REQ, EP0_DMA);
GX_SRAM_ADJACENT(EP0_DMA, BOOTLOG);

/* The EP0 DMA trio must tile its own block, in order. */
GX_SRAM_ADJACENT(EP0_DMA_SETUP, EP0_DMA_OUT);
GX_SRAM_ADJACENT(EP0_DMA_OUT, EP0_DMA_IN);
static_assert(GX_SRAM_EP0_DMA_SETUP_ADDR == GX_SRAM_EP0_DMA_ADDR &&
	      GX_SRAM_EP0_DMA_IN_ADDR + GX_SRAM_EP0_DMA_IN_LEN <=
	      GX_SRAM_EP0_DMA_ADDR + GX_SRAM_EP0_DMA_LEN,
	      "SRAM map: the EP0 DMA trio does not fit its own block");

/*
 * The words this header already defines must land in the range that claims
 * them.  These are the links between the map and the definitions it is a map
 * OF -- without them the map is a second, parallel set of numbers, which is
 * the defect it exists to remove.
 */
static_assert(GX_TUNE_PARAMS_ADDR == GX_SRAM_TUNE_ADDR,
	      "SRAM map: GX_TUNE_PARAMS_ADDR is not where TUNE says it is");
static_assert(GX_HANDOFF_ADDR == GX_SRAM_HANDOFF_ADDR,
	      "SRAM map: GX_HANDOFF_ADDR is not where HANDOFF says it is");
static_assert(GX_XPL_PROG_ADDR >= GX_SRAM_BREADCRUMB_ADDR &&
	      GX_XPL_PROG_ADDR < GX_SRAM_BREADCRUMB_ADDR +
				 GX_SRAM_BREADCRUMB_LEN,
	      "SRAM map: the progress word is outside BREADCRUMB");
static_assert(GX_DRAM_FAULT_ADDR + 4 <= GX_SRAM_BREADCRUMB_ADDR +
					GX_SRAM_BREADCRUMB_LEN,
	      "SRAM map: the stage report pair runs past BREADCRUMB");
static_assert(GX_NOR_PROBE_ADDR == GX_SRAM_NOR_PROBE_ADDR &&
	      GX_NOR_PROBE_ID_ADDR + 4 <= GX_SRAM_NOR_PROBE_ADDR +
					  GX_SRAM_NOR_PROBE_LEN,
	      "SRAM map: the NOR probe pair does not fit NOR_PROBE");

/*
 * The two bounds.  The bottom is Kconfig and therefore preprocessor-visible;
 * the top is the vendor's BL1 map, quoted with its citations in the map
 * comment above.  0xd900c400 is BL1_NAND_BUFF and 0xd900c800 is the BROM's
 * own BSS, so a claim past the end of this window is a claim on the ROM.
 *
 * The bottom bound belongs to whichever stage is RESIDENT IN SRAM, and the
 * chain moves that.  Single-stage, it is the SPL at SPL_TEXT_BASE.  Under
 * MESON_GX_XPL_CHAIN it is the TPL, and SPL_TEXT_BASE is a DRAM address --
 * 0x08000000, which trivially satisfies any comparison against a 0xd9xxxxxx
 * window.  Both CONFIG_SPL_* symbols are unprefixed and so carry the SPL
 * phase's values in every phase, which is exactly how this assert came to
 * pass without being able to fail.  Select the pair, do not assume it.
 */
#if defined(CONFIG_MESON_GX_XPL_CHAIN)
#if defined(CONFIG_TPL_TEXT_BASE) && defined(CONFIG_TPL_MAX_SIZE)
static_assert(GX_SRAM_BROM_PARAM_ADDR >=
	      (unsigned long)CONFIG_TPL_TEXT_BASE + CONFIG_TPL_MAX_SIZE,
	      "SRAM map: the breadcrumb window starts inside the TPL image");
#endif
#else
#if defined(CONFIG_SPL_TEXT_BASE) && defined(CONFIG_SPL_MAX_SIZE)
static_assert(GX_SRAM_BROM_PARAM_ADDR >=
	      (unsigned long)CONFIG_SPL_TEXT_BASE + CONFIG_SPL_MAX_SIZE,
	      "SRAM map: the breadcrumb window starts inside the loaded image");
#endif
#endif
static_assert(GX_SRAM_BOOTLOG_ADDR + GX_SRAM_BOOTLOG_LEN == GX_SRAM_WINDOW_END,
	      "SRAM map: the chain does not reach the end of the window");

/*
 * ...and the OTHER end of the window, which nothing guarded until 2026-08-31.
 *
 * The asserts above bound the window against the loaded IMAGE, i.e. from
 * BELOW. Nothing bound it against SPL's BSS, which sits ABOVE the image at an
 * address the linker takes on trust from Kconfig. So an SPL_BSS_START_ADDR
 * inside the window compiled, linked and ran, silently overwriting the
 * breadcrumb + handoff block that is the whole point of the window -- the one
 * region that must SURVIVE the stage boundary.
 *
 * It was not hypothetical. Four in-tree Amlogic SPL fragments carried
 * SPL_BSS_START_ADDR=0xd900c000 with SPL_BSS_MAX_SIZE=0x1000, i.e. BSS spanning
 * 0xd900c000-0xd900d000: the ENTIRE named window (BROM_PARAM .. WINDOW_END
 * 0xd900c400) plus BL1_NAND_BUFF beyond it. Measured __bss_end = 0xd900c650.
 * They linked clean, and `git grep SPL_BSS_START_ADDR` over arch/ include/
 * common/spl/ found no assert relating it to GX_SRAM_* anywhere. Every SHIPPING
 * profile used 0xd900d000, so the divergence lived only in the fragments the
 * published doc/board/amlogic/spl.rst recipe builds -- the path with no CI.
 *
 * Bound to WINDOW_END rather than to BL1's own extent on purpose: BL1's stack
 * and BSS (0xD900_C800-0xD901_2FFF) are SEQUENTIALLY safe -- BL1 is finished
 * before this stage's BSS is live, the same argument that lets TPL and SPL
 * share a BSS address. The breadcrumb window is the part that must survive, so
 * it is the part worth asserting.
 *
 * SCOPED TO MESON_GX -- but NOT for the reason this comment used to give, and
 * not for the reason 23da3dea134 gave when it added the scope.
 *
 * Both said arch/arm/mach-meson/spl.c "includes BOTH this header and
 * g12a-handoff.h", so a G12A/SM1 build would evaluate whatever is written here.
 * It does not. spl.c's two includes sit in EXCLUSIVE arms of the same
 * conditional -- `#if defined(CONFIG_MESON_G12A)` pulls in g12a-handoff.h, the
 * `#else` pulls in this one -- so a G12A/SM1 build never sees this header and a
 * GXL build never sees g12a-handoff.h. (23da3dea134's commit message spelled the
 * same premise a third way, "g12a-handoff.h INCLUDES gx-handoff.h"; that
 * #include does not exist either. No header in the tree includes either handoff
 * header, and no translation unit sees both: every other includer -- dram-gx.c,
 * meson-usb.c, spl-dram-stage-gx.c, tpl-load-mmc.c, tpl-nor-probe.c here, and
 * spl-dram-stage.c for G12A -- is Makefile-gated to its own family.)
 *
 * So this assert was never evaluated in an SM1 build at all, vacuously or
 * otherwise; it was absent there. The three-arm evidence in 23da3dea134 is
 * consistent with that -- arm C observed only rc=0 and "no MESON_GX in the
 * generated config", which absence explains as well as scoping does.
 *
 * The scope is KEPT, for a real if currently inert reason: the `#else` arm in
 * spl.c is "ARCH_MESON and not G12A", which is wider than MESON_GX -- it also
 * covers AXG and A1. Neither selects SUPPORT_SPL/SUPPORT_TPL today
 * (arch/arm/mach-meson/Kconfig), so neither reaches an SPL/TPL build of spl.c
 * and CONFIG_MESON_GX is in fact set wherever this header compiles. The guard
 * bounds a future AXG/A1 SPL, not a present G12A one.
 *
 * What survives unchanged is the numeric argument, which is why the scope is
 * worth keeping rather than deleting: an SPL BSS address from the 0xfffb0000
 * G12 SRAM map compared against a 0xd900c400 GX window constant is a
 * coincidence, not a check.
 */
#if defined(CONFIG_MESON_GX) && defined(CONFIG_SPL_BSS_START_ADDR)
static_assert((unsigned long)CONFIG_SPL_BSS_START_ADDR >= GX_SRAM_WINDOW_END,
	      "SRAM map: SPL BSS starts inside the breadcrumb/handoff window");
#endif

/*
 * The SPL's pre-relocation arena is the CONFIG_SPL_SYS_MALLOC_F_LEN bytes
 * directly under CONFIG_SPL_STACK.  With the stack in SRAM at 0xd900ff00, a
 * 0x8000 arena (what SPL_LZMA needs) starts at 0xd9007f00 and grows up through
 * this window and the SPL BSS: the TPL->SPL handoff at 0xd900c0c0 is reached
 * after ~16.5 KB of early allocations.  That is the N24 "SPL link delta"
 * (2026-09-13).  An SRAM stack must keep the whole arena above the BSS.
 */
#if defined(CONFIG_MESON_GX) && defined(CONFIG_SPL_BSS_START_ADDR) && \
	defined(CONFIG_SPL_BSS_MAX_SIZE) && defined(CONFIG_SPL_STACK) && \
	defined(CONFIG_SPL_SYS_MALLOC_F_LEN)
static_assert((unsigned long)CONFIG_SPL_STACK < 0xd9000000UL ||
	      (unsigned long)CONFIG_SPL_STACK - CONFIG_SPL_SYS_MALLOC_F_LEN >=
	      (unsigned long)CONFIG_SPL_BSS_START_ADDR + CONFIG_SPL_BSS_MAX_SIZE,
	      "SRAM map: the SPL early-malloc arena under SPL_STACK overlaps the SPL BSS or the handoff window");
#endif

/*
 * IMAGE LAYOUT: how much room the DRAM stage actually has, and the check that
 * enforces it.
 *
 * Under MESON_GX_XPL_CHAIN the combined image is
 *
 *     [ signed SRAM stage ][ SPL @ MESON_GX_SPL_OFFSET ][ FIT @ MESON_GX_FIT_OFFSET ]
 *
 * so the SPL's room is FIT_OFFSET - SPL_OFFSET.  What is NOT obvious, and is
 * the trap this comment exists for: those are not three independent numbers.
 * MESON_GX_FIT_OFFSET defaults to SPL_PAD_TO, and SPL_PAD_TO defaults to
 * SPL_MAX_SIZE (common/spl/Kconfig:97).  So unless a board overrides them,
 *
 *     FIT_OFFSET == SPL_PAD_TO == SPL_MAX_SIZE
 *
 * and the room is SPL_MAX_SIZE - MESON_GX_SPL_OFFSET.  The FIT moves to
 * accommodate a padded SPL; it is not a fixed hole the SPL can overrun.
 *
 * The consequence for anyone writing a check here: comparing SPL_MAX_SIZE
 * against FIT_OFFSET - SPL_OFFSET is comparing SPL_MAX_SIZE against itself
 * minus a constant, which is unsatisfiable for every value.  An assert in that
 * shape fires on every clean build -- measured, not theorised.
 *
 * The bound that CAN be violated is on the SPL's ACTUAL size, and the symbol
 * that expresses it is SPL_SIZE_LIMIT: the post-link size_check on the built
 * binary (top-level Makefile), which is ignored when 0.  SPL_MAX_SIZE is only
 * the linker region (scripts/Makefile.xpl -> u-boot-spl.lds), and it excludes
 * BSS, so it cannot stand in for it.
 *
 * This assert therefore keeps SPL_SIZE_LIMIT honest against the layout: the
 * post-link limit must not claim more room than the image leaves.
 */
#if defined(CONFIG_MESON_GX_XPL_CHAIN) && defined(CONFIG_MESON_GX_SPL_OFFSET) && \
	defined(CONFIG_SPL_SIZE_LIMIT) && CONFIG_SPL_SIZE_LIMIT > 0
static_assert(CONFIG_SPL_SIZE_LIMIT <=
	      CONFIG_SPL_MAX_SIZE - CONFIG_MESON_GX_SPL_OFFSET,
	      "image layout: SPL_SIZE_LIMIT claims more room than the image leaves between the SRAM stage and the FIT");
#endif

/*
 * MESON_GX_SPL_MAX_SIZE is how many bytes TPL's storage loaders read from
 * MESON_GX_SPL_OFFSET.  It must cover every SPL the size check lets through
 * (else a storage boot runs a truncated SPL), and must stop at the FIT.  Its
 * Kconfig default (0x13e00) is the window of the original layout, so a board
 * that widens the window and forgets this symbol fails here, not on media.
 */
#if defined(CONFIG_MESON_GX_XPL_CHAIN) && defined(CONFIG_MESON_GX_SPL_MAX_SIZE) && \
	defined(CONFIG_SPL_SIZE_LIMIT) && CONFIG_SPL_SIZE_LIMIT > 0
static_assert(CONFIG_MESON_GX_SPL_MAX_SIZE >= CONFIG_SPL_SIZE_LIMIT &&
	      CONFIG_MESON_GX_SPL_MAX_SIZE <=
	      CONFIG_SPL_MAX_SIZE - CONFIG_MESON_GX_SPL_OFFSET,
	      "image layout: MESON_GX_SPL_MAX_SIZE (TPL's read of the SPL) must cover SPL_SIZE_LIMIT and stop at the FIT");
#endif

#endif /* __ASSEMBLY__ */
#endif /* __GX_HANDOFF_H__ */
