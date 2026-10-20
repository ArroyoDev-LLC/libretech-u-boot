/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * DDR_DETECT_GEN -- the SIZE DETECTOR's generation counter, and the cross-file
 * half of DFC_INSTRUMENT_GEN.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * dram-fault-class.c carries DFC_INSTRUMENT_GEN, a monotone counter bumped by
 * every fix a printed verdict depends on, with `#if ... #error` guards in the
 * arms that depend on it.  It works because the counter and the arm live in
 * THE SAME FILE: cut an arm onto an older dram-fault-class.c and it brings the
 * older number with it.  That was a deliberate, stated scope, and it leaves a
 * hole one file over:
 *
 *   meson_dram_detect_size() lives in dram-gx.c, and it is the source of the
 *   size_mb that becomes the window END of every probe the instrument runs.
 *   Before 27720951c62 that function's alias guard (`ref[0] == 0`) was
 *   structurally unfirable and a total detection failure DEFAULTED TO 128 MB.
 *   So an instrument arm built against a dram-gx.c lacking the bail compiles
 *   clean at DFC_INSTRUMENT_GEN 4 while every window END it prints descends
 *   from a number nothing measured -- with no build error and no
 *   wrong-looking output.
 *
 * WHAT THE COUNTER MEANS -- unchanged from DFC_INSTRUMENT_GEN's rule
 * -----------------------------------------------------------------
 * "A printed verdict means something different now", NOT "something changed".
 * Bump it only for a detector change that alters what a size, a refusal or a
 * window END MEANS.  A refactor, a comment, a byte saved: no bump.
 *
 *   1  baseline -- the unfirable `ref[0] == 0` alias guard, constant markers
 *      that a watchdog reboot could satisfy off the PREVIOUS boot, and a
 *      total detection failure returning 128 MB.  NO HEADER EXISTED at this
 *      generation, which is why a tree at 1 fails the #include rather than a
 *      comparison.
 *   2  27720951c62 (audit S5/S6) -- the reference cell is a SIGNATURE that can
 *      fail, the markers carry a per-boot CNTPCT nonce so a completely lost
 *      write can no longer read back correct, and a refusal returns 0 so
 *      dram_init() FAILS instead of publishing a size nothing measured.
 *
 * The D3 product split (u-boot topic/gxl-detect-bail 502fe35a5b9 +
 * 022f7608ec8: `why=3` bus-echo discrimination and DDR_DETECT_MIN_MB) takes
 * generation 3 when it lands, because it splits one refusal state into two and
 * so changes what a printed `why=` means.  It is NOT numbered here in advance
 * -- a generation that no dram-gx.c implements is exactly the version-without-
 * a-verdict-change this counter must never be spent on.
 *
 * HOW IT FAILS -- two layers, because one is not enough
 * ----------------------------------------------------
 * (a) PREPROCESSOR.  This header travels with dram-gx.c, so an older tree
 *     brings an older DDR_DETECT_GEN -- or no header at all.  The instrument's
 *     `#if DDR_DETECT_GEN < N` / missing-include #error fires at build time on
 *     the exact fragment stack, before any board time.
 *
 * (b) LINK WITNESS.  A header is a SHARED file, so (a) alone is satisfied by a
 *     cherry-pick that takes this header and the instrument commit but not the
 *     dram-gx.c commit -- which is the same motion the guard exists to refuse.
 *     So dram-gx.c PROVIDES an absolute symbol stamped with the generation it
 *     was compiled against, and dram-fault-class.c REQUIRES it through a
 *     relocation that emits no bytes.  A dram-gx.c that never saw this header
 *     defines no such symbol and the link fails naming it.
 *
 *     Measured with /usr/bin/aarch64-linux-gnu-gcc 14.2.0 + GNU ld 2.44, the
 *     toolchain both the product and the bench targets select: R_AARCH64_NONE
 *     costs 0 bytes of .text (20 B with and without, same object), and the
 *     absent-provider arm fails rc=1 with
 *     `undefined reference to '__ddr_detect_gen_2__stale_dram_gx_c'`.
 *
 * WHAT IT DOES NOT COVER -- the residual, stated rather than left implicit
 * ----------------------------------------------------------------------
 * Bumping the number in this header WITHOUT making the matching change in
 * dram-gx.c passes both layers, because dram-gx.c provides whatever the header
 * says.  That is the SAME residual DFC_INSTRUMENT_GEN already has (bump the
 * counter without the fix) and it is the bound on this whole class: the guard
 * proves the number travelled with the code, never that the number is true.
 * The remedy for that one is the rule above and a reviewer, not a check.
 *
 * The PRODUCT branch has no dram-fault-class.c at all.  There, dram-gx.c
 * provides the witness and nothing requires it: an absolute symbol costs zero
 * image bytes, so the product carries the statement and none of the cost.
 */

#ifndef __MESON_DDR_DETECT_GEN_H
#define __MESON_DDR_DETECT_GEN_H

#define DDR_DETECT_GEN		2

#define __DDR_DETECT_SYM2(n)	__ddr_detect_gen_##n##__stale_dram_gx_c
#define __DDR_DETECT_SYM(n)	__DDR_DETECT_SYM2(n)
#define __DDR_DETECT_STR2(x)	#x
#define __DDR_DETECT_STR(x)	__DDR_DETECT_STR2(x)
#define __DDR_DETECT_NAME(n)	__DDR_DETECT_STR(__DDR_DETECT_SYM(n))

/*
 * PROVIDE -- file scope, in dram-gx.c, beside the detector it witnesses.
 * An ABSOLUTE symbol (`.set`), so it occupies no section and no image bytes.
 */
#define DDR_DETECT_PROVIDE(n)						\
	asm(".globl " __DDR_DETECT_NAME(n) "\n"				\
	    ".set " __DDR_DETECT_NAME(n) ", " __DDR_DETECT_STR(n) "\n")

/*
 * REQUIRE -- statement scope, inside a function that is certainly linked, so
 * --gc-sections cannot discard the reference and quietly disarm the check.
 * R_AARCH64_NONE resolves the symbol and emits nothing.
 */
#define DDR_DETECT_REQUIRE(n)						\
	asm volatile(".reloc ., R_AARCH64_NONE, " __DDR_DETECT_NAME(n))

#endif /* __MESON_DDR_DETECT_GEN_H */
