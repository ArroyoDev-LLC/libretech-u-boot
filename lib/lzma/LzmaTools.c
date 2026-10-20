// SPDX-License-Identifier: GPL-2.0+
/*
 * Usefuls routines based on the LzmaTest.c file from LZMA SDK 4.65
 *
 * Copyright (C) 2007-2009 Industrie Dial Face S.p.A.
 * Luigi 'Comio' Mantellini (luigi.mantellini@idf-hit.com)
 *
 * Copyright (C) 1999-2005 Igor Pavlov
 */

/*
 * LZMA_Alone stream format:
 *
 * uchar   Properties[5]
 * uint64  Uncompressed size
 * uchar   data[*]
 *
 */

#include <config.h>
#include <log.h>
#include <watchdog.h>

#ifdef CONFIG_LZMA

#define LZMA_PROPERTIES_OFFSET 0
#define LZMA_SIZE_OFFSET       LZMA_PROPS_SIZE
#define LZMA_DATA_OFFSET       LZMA_SIZE_OFFSET+sizeof(uint64_t)

#include "LzmaTools.h"
#include "LzmaDec.h"

#include <asm/global_data.h>
#include <linux/build_bug.h>
#include <linux/string.h>
#include <malloc.h>

DECLARE_GLOBAL_DATA_PTR;

static void *SzAlloc(ISzAllocPtr p, size_t size) { return malloc(size); }
static void SzFree(ISzAllocPtr p, void *address) { free(address); }

/*
 * SzAlloc() above is the decoder's only allocator, and LzmaDecode()
 * (lib/lzma/LzmaDec.c) reaches exactly one allocation through it:
 * LzmaDec_AllocateProbs().  It then points p.dic at the CALLER's output
 * buffer, so LzmaDec_Allocate()'s dictionary allocation is never on this path
 * and the stream header's dicSize does not bind.  What does bind is
 *
 *   numProbs * sizeof(CLzmaProb)
 *     = (1984 + (0x300 << (lc + lp))) * 4
 *
 * -- LzmaProps_GetNumProbs(), with CLzmaProb four bytes wide because
 * lib/lzma/Makefile adds -DZ7_LZMA_PROB32.  lc and lp are STREAM properties,
 * read out of the LZMA_Alone header's first byte, so the requirement is a
 * property of the encoder, not of the decoder or the payload size.  binman
 * pins them: tools/binman/btool/lzma_alone.py compresses with
 * '-lc1 -lp0 -pb0 -d8', giving 4 * (1984 + (0x300 << 1)) = 14080 bytes.
 *
 * When SYS_MALLOC_SIMPLE is in effect, malloc() is the bump allocator in
 * common/malloc_simple.c and that 14080 bytes has to come out of the fixed
 * SYS_MALLOC_F_LEN arena, on top of whatever the phase already allocated for
 * driver model.  A GXL SPL was measured using 0x1a8 before the decoder ran --
 * it died at "alloc space exhausted ptr 38a8 limit 1000", i.e. 0x3700 of
 * probabilities behind 0x1a8 of driver model against a 0x1000 pool -- so
 * require the decoder's own 0x3700 plus a 0x400 allowance for the phase.
 *
 * LZ4's decoder is streaming and allocates nothing, which is why swapping LZ4
 * for LZMA can turn a build-clean, correctly-packed, hash-verified image into
 * one that dies on its first compressed node with no image-side symptom.  This
 * assert is the build-time half of that: a phase that selects LZMA with a heap
 * it cannot decompress out of must fail here rather than on the board.
 */
#define LZMA_BINMAN_LC_PLUS_LP	1
#define LZMA_NUM_PROBS		(1984 + (0x300 << LZMA_BINMAN_LC_PLUS_LP))
#define LZMA_PROBS_BYTES	(LZMA_NUM_PROBS * (int)sizeof(CLzmaProb))
#define LZMA_PHASE_HEAP_OTHER	0x400
#define LZMA_MIN_MALLOC_F_LEN	(LZMA_PROBS_BYTES + LZMA_PHASE_HEAP_OTHER)

#if CONFIG_IS_ENABLED(SYS_MALLOC_SIMPLE)
static_assert(CONFIG_VAL(SYS_MALLOC_F_LEN) >= LZMA_MIN_MALLOC_F_LEN,
	      "SYS_MALLOC_F_LEN is too small for the LZMA decoder. "
	      "With SYS_MALLOC_SIMPLE the decoder's probability table "
	      "(14080 bytes for a binman -lc1 -lp0 stream) is taken from the "
	      "SYS_MALLOC_F_LEN arena, so this phase would die at "
	      "'alloc space exhausted' on its first compressed node. "
	      "Raise CONFIG_SPL_SYS_MALLOC_F_LEN to at least 0x3b00 "
	      "(0x8000 is the value proven on GXL silicon), or drop "
	      "CONFIG_SPL_LZMA.");
#endif

/*
 * ...and the RUNTIME half, which is the half that can be right.
 *
 * The static_assert above has to pick an lc+lp at COMPILE time, and the only
 * value it can pick is a guess about a build step that has not run yet: binman
 * compresses the FIT's nodes AFTER this translation unit is compiled, with
 * flags that live in a Python bintool (tools/binman/btool/lzma_alone.py).  That
 * tool's own default is -lc3, which needs 32512 bytes -- 168 bytes MORE than
 * even a 0x8000 arena has left after the phase's own allocations.  So the
 * build-time threshold is a floor against configurations that are broken for
 * EVERY stream; it is not, and cannot be, a proof that THIS stream fits.
 *
 * Nothing at build time can be, because an xPL decompresses whatever FIT it is
 * handed at boot, which need not be the FIT binman built here: a hand-lzma'd
 * payload, an `xz --format=lzma` round-trip, another vendor's packer and a
 * future edit to that bintool all produce a legal stream with a different
 * lc/lp, and none of them is visible to any check in this tree.
 *
 * The stream's own first byte IS visible, here, one call before the allocation
 * it determines -- and when the phase is on the bump allocator, exactly how
 * many bytes remain is visible too (gd->malloc_limit - gd->malloc_ptr, the same
 * two words common/malloc_simple.c compares).  So compute the real requirement
 * from the real stream and refuse with the numbers in hand, rather than letting
 * alloc_simple() report a bare "alloc space exhausted ptr/limit" that names
 * neither the decoder nor the reason.
 *
 * The predicate is gd->flags, not a Kconfig symbol, because SYS_MALLOC_SIMPLE
 * is not the only way to end up on the bump allocator: common/dlmalloc.c routes
 * malloc() to malloc_simple() whenever GD_FLG_FULL_MALLOC_INIT is clear, which
 * also covers a phase that sets NEITHER SYS_MALLOC_SIMPLE NOR SYS_MALLOC (as
 * xilinx_zynqmp_kria's SPL does).  Testing the flag covers every such phase and
 * costs nothing in U-Boot proper, where it is set and the check falls through.
 */
#if CONFIG_IS_ENABLED(SYS_MALLOC_F)
static int lzma_probs_fit(const unsigned char *props)
{
	unsigned int d = props[LZMA_PROPERTIES_OFFSET];
	unsigned int lc, lp, need, avail;

	if (gd->flags & GD_FLG_FULL_MALLOC_INIT)
		return SZ_OK;

	/* LzmaProps_Decode()'s own arithmetic; >= 9*5*5 is not a valid byte */
	if (d >= (9 * 5 * 5))
		return SZ_ERROR_UNSUPPORTED;
	lc = d % 9;
	lp = (d / 9) % 5;

	need = (1984 + (0x300u << (lc + lp))) * sizeof(CLzmaProb);
	avail = gd->malloc_limit - gd->malloc_ptr;
	if (need <= avail)
		return SZ_OK;

	log_err("LZMA: stream needs %u B of probability table (lc=%u lp=%u) but only %u B of the %u B early-malloc arena are left; raise SYS_MALLOC_F_LEN for this phase or re-compress with a lower -lc/-lp\n",
		need, lc, lp, avail,
		(unsigned int)CONFIG_VAL(SYS_MALLOC_F_LEN));

	return SZ_ERROR_MEM;
}
#else
static int lzma_probs_fit(const unsigned char *props) { return SZ_OK; }
#endif

int lzmaBuffToBuffDecompress(unsigned char *outStream, SizeT *uncompressedSize,
			     const unsigned char *inStream, SizeT length)
{
    int res = SZ_ERROR_DATA;
    int i;
    ISzAlloc g_Alloc;

    SizeT outSizeFull = 0xFFFFFFFF; /* 4GBytes limit */
    SizeT outProcessed;
    SizeT outSize;
    SizeT outSizeHigh;
    ELzmaStatus state;
    SizeT compressedSize = (SizeT)(length - (LZMA_PROPS_SIZE + sizeof(uint64_t)));

    debug ("LZMA: Image address............... 0x%p\n", inStream);
    debug ("LZMA: Properties address.......... 0x%p\n", inStream + LZMA_PROPERTIES_OFFSET);
    debug ("LZMA: Uncompressed size address... 0x%p\n", inStream + LZMA_SIZE_OFFSET);
    debug ("LZMA: Compressed data address..... 0x%p\n", inStream + LZMA_DATA_OFFSET);
    debug ("LZMA: Destination address......... 0x%p\n", outStream);

    memset(&state, 0, sizeof(state));

    outSize = 0;
    outSizeHigh = 0;
    /* Read the uncompressed size */
    for (i = 0; i < 8; i++) {
        unsigned char b = inStream[LZMA_SIZE_OFFSET + i];
            if (i < 4) {
                outSize     += (UInt32)(b) << (i * 8);
        } else {
                outSizeHigh += (UInt32)(b) << ((i - 4) * 8);
        }
    }

    outSizeFull = (SizeT)outSize;
    if (sizeof(SizeT) >= 8) {
        /*
         * SizeT is a 64 bit uint => We can manage files larger than 4GB!
         *
         */
            outSizeFull |= (((SizeT)outSizeHigh << 16) << 16);
    } else if (outSizeHigh != 0 || (UInt32)(SizeT)outSize != outSize) {
        /*
         * SizeT is a 32 bit uint => We cannot manage files larger than
         * 4GB!  Assume however that all 0xf values is "unknown size" and
         * not actually a file of 2^64 bits.
         *
         */
        if (outSizeHigh != (SizeT)-1 || outSize != (SizeT)-1) {
            debug ("LZMA: 64bit support not enabled.\n");
            return SZ_ERROR_DATA;
        }
    }

    debug("LZMA: Uncompresed size............ 0x%zx\n", outSizeFull);
    debug("LZMA: Compresed size.............. 0x%zx\n", compressedSize);

    g_Alloc.Alloc = SzAlloc;
    g_Alloc.Free = SzFree;

    /* Refuse a stream this phase's arena cannot hold, while we can say why */
    res = lzma_probs_fit(inStream);
    if (res != SZ_OK)
        return res;

    /* Short-circuit early if we know the buffer can't hold the results. */
    if (outSizeFull != (SizeT)-1 && *uncompressedSize < outSizeFull)
        return SZ_ERROR_OUTPUT_EOF;

    /* Decompress */
    outProcessed = min(outSizeFull, *uncompressedSize);

    schedule();

    res = LzmaDecode(
        outStream, &outProcessed,
        inStream + LZMA_DATA_OFFSET, &compressedSize,
        inStream, LZMA_PROPS_SIZE, LZMA_FINISH_END, &state, &g_Alloc);
    *uncompressedSize = outProcessed;

    debug("LZMA: Uncompressed ............... 0x%zx\n", outProcessed);

    if (res != SZ_OK)  {
        return res;
    }

    return res;
}

#endif
