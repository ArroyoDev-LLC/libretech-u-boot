// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2013
 * Afzal Mohammed <afzal.mohd.ma@gmail.com>
 *
 * Reference: dfu_mmc.c
 * Copyright (C) 2012 Samsung Electronics
 * author: Lukasz Majewski <l.majewski@samsung.com>
 */

#include <malloc.h>
#include <mapmem.h>
#include <errno.h>
#include <dfu.h>
#include <command.h>
#include <time.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <lzma/LzmaTypes.h>
#include <lzma/LzmaDec.h>

/*
 * DFU_RAM_LZMA: the stream is unpacked WHILE it arrives. Each block lands in
 * the staging window; dfu_background(), called from the gadget loop between
 * USB events, decodes the staged bytes in small slices straight into the
 * alt's window (the window is the decoder's dictionary), and the flush at the
 * end of the download decodes what is left. Unpacking then overlaps the
 * transfer instead of following it. One LZMA download at a time.
 */
#define DFU_LZMA_HDR		(LZMA_PROPS_SIZE + 8)
/* Input per background slice: a few hundred us of decoding at most */
#define DFU_LZMA_SLICE		2048

static struct {
	struct dfu_entity *dfu;		/* the download being unpacked, or NULL */
	CLzmaDec dec;
	bool started;			/* header parsed, decoder up */
	bool done;			/* end of stream reached */
	int err;			/* first decode error, 0 if none */
	u64 consumed;			/* staged bytes the decoder has taken */
	u64 out_size;			/* from the header; (u64)-1 if unknown */
	ulong t_last_block;		/* when the flush came, for the report */
} dfu_lzma;

static void *dfu_lzma_alloc(ISzAllocPtr p, size_t size) { return malloc(size); }
static void dfu_lzma_free(ISzAllocPtr p, void *address) { free(address); }
static const ISzAlloc dfu_lzma_allocator = { dfu_lzma_alloc, dfu_lzma_free };

static void dfu_ram_lzma_start(struct dfu_entity *dfu)
{
	/*
	 * Unreachable without DFU_RAM_LZMA (dfu_fill_entity_ram() refuses the
	 * layout), and said so here: the call below would otherwise still be
	 * linked, and a phase built without the decoder fails on
	 * LzmaDec_FreeProbs.
	 */
	if (!CONFIG_IS_ENABLED(DFU_RAM_LZMA))
		return;
	if (dfu_lzma.started)
		LzmaDec_FreeProbs(&dfu_lzma.dec, &dfu_lzma_allocator);
	memset(&dfu_lzma, 0, sizeof(dfu_lzma));
	LzmaDec_Construct(&dfu_lzma.dec);
	dfu_lzma.dfu = dfu;
	dfu->data.ram.staged = 0;
}

/* Decode up to @budget staged input bytes; 0, or a negative errno. */
static int dfu_ram_lzma_step(SizeT budget)
{
	struct dfu_entity *dfu = dfu_lzma.dfu;
	const Byte *in = map_sysmem(dfu->data.ram.staging, 0);
	ELzmaStatus status;
	SizeT len, limit;
	SRes res;
	int i;

	if (dfu_lzma.err)
		return dfu_lzma.err;
	if (!dfu_lzma.started) {
		if (dfu->data.ram.staged < DFU_LZMA_HDR)
			return 0;
		dfu_lzma.out_size = 0;
		for (i = 0; i < 8; i++)
			dfu_lzma.out_size |= (u64)in[LZMA_PROPS_SIZE + i] << (8 * i);
		if (dfu_lzma.out_size != (u64)-1 &&
		    dfu_lzma.out_size > dfu->data.ram.size) {
			pr_err("DFU %s: lzma stream unpacks to %lu bytes, window %u\n",
			       dfu->name, (ulong)dfu_lzma.out_size, dfu->data.ram.size);
			return dfu_lzma.err = -EFBIG;
		}
		if (LzmaDec_AllocateProbs(&dfu_lzma.dec, in, LZMA_PROPS_SIZE,
					  &dfu_lzma_allocator) != SZ_OK)
			return dfu_lzma.err = -ENOMEM;
		dfu_lzma.dec.dic = map_sysmem(dfu->data.ram.start, 0);
		dfu_lzma.dec.dicBufSize = dfu->data.ram.size;
		LzmaDec_Init(&dfu_lzma.dec);
		dfu_lzma.consumed = DFU_LZMA_HDR;
		dfu_lzma.started = true;
	}

	if (dfu_lzma.done)
		return 0;
	len = min_t(SizeT, budget, dfu->data.ram.staged - dfu_lzma.consumed);
	if (!len)
		return 0;
	limit = dfu_lzma.out_size == (u64)-1 ? dfu->data.ram.size :
		dfu_lzma.out_size;
	/*
	 * At the output limit the decoder stops short of the end marker; only
	 * LZMA_FINISH_END makes it take (and check) the bytes that remain.
	 */
	res = LzmaDec_DecodeToDic(&dfu_lzma.dec, limit, in + dfu_lzma.consumed,
				  &len, dfu_lzma.dec.dicPos >= limit ?
				  LZMA_FINISH_END : LZMA_FINISH_ANY, &status);
	dfu_lzma.consumed += len;
	if (res != SZ_OK) {
		pr_err("DFU %s: lzma decode error %d at input %lu\n", dfu->name,
		       res, (ulong)dfu_lzma.consumed);
		return dfu_lzma.err = -EIO;
	}
	if (status == LZMA_STATUS_FINISHED_WITH_MARK ||
	    (dfu_lzma.dec.dicPos >= limit && dfu_lzma.out_size != (u64)-1))
		dfu_lzma.done = true;
	else if (!len && status != LZMA_STATUS_NEEDS_MORE_INPUT)
		return dfu_lzma.err = -EIO;	/* no progress: a stuck stream */

	return 0;
}

/* The gadget loop's idle work (common/dfu.c run_usb_dnl_gadget()). */
void dfu_background(void)
{
	if (CONFIG_IS_ENABLED(DFU_RAM_LZMA) && dfu_lzma.dfu)
		dfu_ram_lzma_step(DFU_LZMA_SLICE);
}

static int dfu_transfer_medium_ram(enum dfu_op op, struct dfu_entity *dfu,
				   u64 offset, void *buf, long *len)
{
	int ret = 1;
	switch (dfu->layout){
		case DFU_RAM_ADDR:
			/*
			 * The whole span, not only its start: checking the
			 * offset alone let a write run up to one DFU buffer
			 * past the end of the alt's RAM window.
			 */
			if (offset > dfu->data.ram.size || *len < 0 ||
			    (u64)*len > dfu->data.ram.size - offset) {
				pr_err("request exceeds allowed area\n");
				return -EINVAL;
			}

			if (op == DFU_OP_WRITE)
				memcpy(map_sysmem(dfu->data.ram.start + offset, 0), buf, *len);
			else
				memcpy(buf, map_sysmem(dfu->data.ram.start + offset, 0), *len);
			ret = 0;
			break;
		case DFU_RAM_LZMA:
			/*
			 * The download is an LZMA ("alone") stream: it lands in
			 * the staging window and dfu_flush_medium_ram() unpacks
			 * it into the alt's window. An upload reads that window,
			 * the unpacked bytes.
			 */
			if (offset > dfu->data.ram.size || *len < 0 ||
			    (u64)*len > dfu->data.ram.size - offset) {
				pr_err("request exceeds allowed area\n");
				return -EINVAL;
			}
			if (op == DFU_OP_WRITE) {
				if (!offset)
					dfu_ram_lzma_start(dfu);
				memcpy(map_sysmem(dfu->data.ram.staging + offset, 0),
				       buf, *len);
				if (offset + *len > dfu->data.ram.staged)
					dfu->data.ram.staged = offset + *len;
			} else {
				memcpy(buf, map_sysmem(dfu->data.ram.start + offset, 0),
				       *len);
			}
			ret = 0;
			break;
		case DFU_SCRIPT:
			/* a bare xPL phase has no shell; fill refuses it */
			if (dfu_bare_phase())
				return -EOPNOTSUPP;
			/*
			 * Commands run on a download only. On an upload buf is
			 * the read buffer, so running it re-executed whatever
			 * it last held; as for dfu_mmc's script entity, the
			 * medium has size 0 and there is nothing to read.
			 */
			if (op != DFU_OP_WRITE)
				return -EOPNOTSUPP;
			ret = run_command_list(buf, *len, 0);
			ret = 0;
			break;
		default:
			pr_err("unsupported layout: %s\n", dfu_get_layout(dfu->layout));
			return  -EINVAL;
	}
	return ret;
}

static int dfu_write_medium_ram(struct dfu_entity *dfu, u64 offset,
				void *buf, long *len)
{
	return dfu_transfer_medium_ram(DFU_OP_WRITE, dfu, offset, buf, len);
}

int dfu_get_medium_size_ram(struct dfu_entity *dfu, u64 *size)
{
	/* a script has nothing to upload (dfu_mmc's script entity is 0 too) */
	*size = dfu->layout == DFU_SCRIPT ? 0 : dfu->data.ram.size;

	return 0;
}

static int dfu_read_medium_ram(struct dfu_entity *dfu, u64 offset,
			       void *buf, long *len)
{
	return dfu_transfer_medium_ram(DFU_OP_READ, dfu, offset, buf, len);
}

/*
 * The plain layout is memory a USB controller can DMA to and from: hand out
 * the span itself (dfu_direct_buf()). Bounded as dfu_transfer_medium_ram()
 * bounds a copy. The LZMA layout keeps the copying path: its blocks feed the
 * decoder's bookkeeping as they arrive.
 */
static void *dfu_direct_buf_ram(struct dfu_entity *dfu, u64 offset, long len)
{
	if (dfu->layout != DFU_RAM_ADDR || len < 0 ||
	    offset > dfu->data.ram.size || (u64)len > dfu->data.ram.size - offset)
		return NULL;

	return map_sysmem(dfu->data.ram.start + offset, len);
}

/*
 * DFU_RAM_LZMA, at the end of a download: decode what the background has not
 * yet, and check the stream ended where it said. A stream that does not
 * decode, or would not fit, fails the manifest; nothing past the window is
 * written. The report gives the time spent here, after the last block: the
 * part of the unpacking the transfer did not hide.
 */
static int dfu_flush_medium_ram(struct dfu_entity *dfu)
{
	ulong t0 = get_timer(0);
	u64 out, in;
	int ret = 0;

	if (!CONFIG_IS_ENABLED(DFU_RAM_LZMA) || dfu->layout != DFU_RAM_LZMA ||
	    dfu_lzma.dfu != dfu || !dfu->data.ram.staged)
		return 0;

	while (!ret && !dfu_lzma.done) {
		u64 before = dfu_lzma.consumed;

		ret = dfu_ram_lzma_step(64 * 1024);
		if (!ret && (!dfu_lzma.started || dfu_lzma.consumed == before))
			ret = -EIO;	/* short of a header, or of the stream's end */
	}
	out = dfu_lzma.started ? dfu_lzma.dec.dicPos : 0;
	in = dfu->data.ram.staged;
	if (!ret && dfu_lzma.out_size != (u64)-1 && out != dfu_lzma.out_size)
		ret = -EIO;
	printf("DFU %s: lzma %lu -> %lu bytes, %lu ms after the last block (%d)\n",
	       dfu->name, (ulong)in, (ulong)out, get_timer(t0), ret);

	if (dfu_lzma.started)
		LzmaDec_FreeProbs(&dfu_lzma.dec, &dfu_lzma_allocator);
	memset(&dfu_lzma, 0, sizeof(dfu_lzma));
	dfu->data.ram.staged = 0;

	return ret;
}

int dfu_fill_entity_ram(struct dfu_entity *dfu, char *devstr, char **argv, int argc)
{
	char *s;
	bool lzma = argc && !strcmp(argv[0], "lzma");

	/* `<name> ram|script <addr> <size>`, `<name> lzma <addr> <size> <staging>` */
	if (argc != (lzma ? 4 : 3)) {
		pr_err("Invalid number of arguments.\n");
		return -EINVAL;
	}

	dfu->dev_type = DFU_DEV_RAM;
	if (strcmp(argv[0], "ram") && strcmp(argv[0], "script") &&
	    !(lzma && CONFIG_IS_ENABLED(DFU_RAM_LZMA))) {
		pr_err("unsupported device: %s\n", argv[0]);
		return -ENODEV;
	}

	if (lzma) {
		dfu->layout = DFU_RAM_LZMA;
		dfu->data.ram.staging = hextoul(argv[3], &s);
		if (*s)
			return -EINVAL;
		dfu->data.ram.staged = 0;
		dfu->flush_medium = dfu_flush_medium_ram;
		/*
		 * Each block reaches the staging window (and so the background
		 * decoder) as it arrives, not after an 8 MiB buffer fills; at
		 * least one 64 KiB block fits.
		 */
		dfu->max_buf_size = SZ_128K;
	} else if (!strcmp(argv[0], "script")) {
		if (dfu_bare_phase()) {
			pr_err("script entity not available in this phase\n");
			return -EOPNOTSUPP;
		}
		dfu->layout = DFU_SCRIPT;
	} else
		dfu->layout = DFU_RAM_ADDR;
	dfu->data.ram.start = hextoul(argv[1], &s);
	if (*s)
		return -EINVAL;
	dfu->data.ram.size = hextoul(argv[2], &s);
	if (*s)
		return -EINVAL;

	dfu->write_medium = dfu_write_medium_ram;
	dfu->get_medium_size = dfu_get_medium_size_ram;
	dfu->read_medium = dfu_read_medium_ram;
	dfu->direct_buf = dfu_direct_buf_ram;

	dfu->inited = 0;

	return 0;
}
