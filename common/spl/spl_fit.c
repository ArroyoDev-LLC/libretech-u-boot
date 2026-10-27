// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <errno.h>
#include <fpga.h>
#include <gzip.h>
#include <image.h>
#include <u-boot/lz4.h>
#include <log.h>
#include <memalign.h>
#include <mapmem.h>
#include <spl.h>
#include <upl.h>
#include <sysinfo.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <linux/libfdt.h>
#include <linux/printk.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * This file is also built for TPL/VPL, whose LOAD_FIT symbols are separate
 * from SPL's -- so a build can compile it with CONFIG_SPL_LOAD_FIT unset and
 * the Kconfig symbol below therefore absent. Zero is the "unset" value and
 * means "use CONFIG_SYS_LOAD_ADDR", so defaulting it here is exactly the
 * behaviour a board that never set it asks for.
 */
#ifndef CONFIG_SPL_FIT_COMP_STAGING_ADDR
#define CONFIG_SPL_FIT_COMP_STAGING_ADDR 0
#endif
#ifndef CONFIG_SPL_FIT_COMP_STAGING_MAX
#define CONFIG_SPL_FIT_COMP_STAGING_MAX 0
#endif
#ifndef CONFIG_SPL_FIT_BUF_MAX
#define CONFIG_SPL_FIT_BUF_MAX 0
#endif

/*
 * A hand-picked staging address is a memory-map CLAIM, and until now nothing
 * tied it to the symbols that can invalidate it. A later CONFIG_TEXT_BASE bump,
 * a bigger media budget, or a variant with a different SPL layout would leave
 * the constant sitting where it is and SPL would read compressed bytes over
 * live .text -- after the hash that would have caught it has already passed.
 * That failure is silent by construction, so it has to be caught at BUILD time.
 *
 * The two board-declared bounds are what make it checkable:
 *
 *   SPL_FIT_COMP_STAGING_MAX  the largest COMPRESSED payload staged, so the
 *                             window above the staging address is big enough.
 *                             It must exceed the compressed size of the biggest
 *                             node (U-Boot proper, in practice), which moves
 *                             build to build -- declare it with headroom.
 *   SPL_FIT_BUF_MAX           the largest FIT the media can deliver into the
 *                             buffer at CONFIG_SPL_LOAD_FIT_ADDRESS, i.e. the
 *                             image budget. This is the bound the original
 *                             safety argument did not use: it measured
 *                             clearance against the buffer's CURRENT size, and
 *                             the buffer's CEILING is the tighter side.
 *
 * Each check is skipped when its bound is 0 (undeclared) so no existing board
 * changes behaviour; a board that names a staging address is strongly
 * encouraged to declare both.
 *
 * NOT covered, deliberately, because SPL's own extent is not derivable from
 * Kconfig alone: an overlap with SPL text/BSS/stack, or with a payload whose
 * load address is neither U-Boot's nor the FIT buffer's. Those stay a board
 * memory-map review.
 */
#if CONFIG_SPL_FIT_COMP_STAGING_ADDR
#if CONFIG_SPL_FIT_COMP_STAGING_MAX
_Static_assert(CONFIG_SPL_FIT_COMP_STAGING_ADDR +
	       CONFIG_SPL_FIT_COMP_STAGING_MAX <= CONFIG_TEXT_BASE,
	       "SPL_FIT_COMP_STAGING_ADDR + SPL_FIT_COMP_STAGING_MAX runs into CONFIG_TEXT_BASE: the staging buffer would be read over U-Boot proper's .text, silently, after its hash has passed");
#endif
#if CONFIG_SPL_FIT_BUF_MAX
_Static_assert(CONFIG_SPL_LOAD_FIT_ADDRESS + CONFIG_SPL_FIT_BUF_MAX <=
	       CONFIG_SPL_FIT_COMP_STAGING_ADDR,
	       "SPL_FIT_COMP_STAGING_ADDR is inside the FIT read buffer's bound (SPL_LOAD_FIT_ADDRESS + SPL_FIT_BUF_MAX): a full-budget image would overlap the staging buffer");
#endif
#endif

struct spl_fit_info {
	const void *fit;	/* Pointer to a valid FIT blob */
	size_t ext_data_offset;	/* Offset to FIT external data (end of FIT) */
	int images_node;	/* FDT offset to "/images" node */
	int conf_node;		/* FDT offset to selected configuration node */
};

__weak ulong board_spl_fit_size_align(ulong size)
{
	return size;
}

static int find_node_from_desc(const void *fit, int node, const char *str)
{
	int child;

	if (node < 0)
		return -EINVAL;

	/* iterate the FIT nodes and find a matching description */
	for (child = fdt_first_subnode(fit, node); child >= 0;
	     child = fdt_next_subnode(fit, child)) {
		int len;
		const char *desc = fdt_getprop(fit, child, FIT_DESC_PROP, &len);

		if (!desc)
			continue;

		if (!strcmp(desc, str))
			return child;
	}

	return -ENOENT;
}

/**
 * spl_fit_get_image_name(): By using the matching configuration subnode,
 * retrieve the name of an image, specified by a property name and an index
 * into that.
 * @fit:	Pointer to the FDT blob.
 * @images:	Offset of the /images subnode.
 * @type:	Name of the property within the configuration subnode.
 * @index:	Index into the list of strings in this property.
 * @outname:	Name of the image
 *
 * Return:	0 on success, or a negative error number
 */
static int spl_fit_get_image_name(const struct spl_fit_info *ctx,
				  const char *type, int index,
				  const char **outname)
{
	struct udevice *sysinfo;
	const char *name, *str, *end;
	__maybe_unused int node;
	int len, i;
	bool found = true;

	name = fdt_getprop(ctx->fit, ctx->conf_node, type, &len);
	if (!name) {
		debug("cannot find property '%s': %d\n", type, len);
		return -EINVAL;
	}
	/* A string property should be NUL terminated */
	end = name + len - 1;
	if (!len || *end) {
		debug("malformed property '%s'\n", type);
		return -EINVAL;
	}

	str = name;
	for (i = 0; i < index; i++) {
		str = strchr(str, '\0') + 1;
		if (str > end) {
			found = false;
			break;
		}
	}

	if (!found && CONFIG_IS_ENABLED(SYSINFO) && !sysinfo_get(&sysinfo)) {
		int rc;
		/*
		 * no string in the property for this index. Check if the
		 * sysinfo-level code can supply one.
		 */
		rc = sysinfo_detect(sysinfo);
		if (rc)
			return rc;

		rc = sysinfo_get_fit_loadable(sysinfo, index - i - 1, type,
					      &str);
		if (rc && rc != -ENOENT)
			return rc;

		if (!rc) {
			/*
			 * The sysinfo provided a name for a loadable.
			 * Try to match it against the description properties
			 * first. If no matching node is found, use it as a
			 * node name.
			 */
			int node;
			int images = fdt_path_offset(ctx->fit, FIT_IMAGES_PATH);

			node = find_node_from_desc(ctx->fit, images, str);
			if (node > 0)
				str = fdt_get_name(ctx->fit, node, NULL);

			found = true;
		}
	}

	if (!found) {
		debug("no string for index %d\n", index);
		return -E2BIG;
	}

	*outname = str;
	return 0;
}

/**
 * spl_fit_get_image_node(): By using the matching configuration subnode,
 * retrieve the name of an image, specified by a property name and an index
 * into that.
 * @fit:	Pointer to the FDT blob.
 * @images:	Offset of the /images subnode.
 * @type:	Name of the property within the configuration subnode.
 * @index:	Index into the list of strings in this property.
 *
 * Return:	the node offset of the respective image node or a negative
 *		error number.
 */
static int spl_fit_get_image_node(const struct spl_fit_info *ctx,
				  const char *type, int index)
{
	const char *str;
	int err;
	int node;

	err = spl_fit_get_image_name(ctx, type, index, &str);
	if (err)
		return err;

	debug("%s: '%s'\n", type, str);

	node = fdt_subnode_offset(ctx->fit, ctx->images_node, str);
	if (node < 0) {
		pr_err("cannot find image node '%s': %d\n", str, node);
		return -EINVAL;
	}

	return node;
}

static int get_aligned_image_offset(struct spl_load_info *info, int offset)
{
	return ALIGN_DOWN(offset, spl_get_bl_len(info));
}

static int get_aligned_image_overhead(struct spl_load_info *info, int offset)
{
	return offset & (spl_get_bl_len(info) - 1);
}

static int get_aligned_image_size(struct spl_load_info *info, int data_size,
				  int offset)
{
	data_size = data_size + get_aligned_image_overhead(info, offset);

	return ALIGN(data_size, spl_get_bl_len(info));
}

/**
 * load_simple_fit(): load the image described in a certain FIT node
 * @info:	points to information about the device to load data from
 * @fit_offset:	the offset of the FIT image on the device
 * @ctx:	points to the FIT context structure
 * @node:	offset of the DT node describing the image to load (relative
 *		to @fit)
 * @image_info:	will be filled with information about the loaded image
 *		If the FIT node does not contain a "load" (address) property,
 *		the image gets loaded to the address pointed to by the
 *		load_addr member in this struct, if load_addr is not 0
 *
 * Return:	0 on success, -EBADSLT if this image is not the correct phase
 * (for CONFIG_BOOTMETH_VBE_SIMPLE_FW), or another negative error number on
 * other error.
 */
/**
 * spl_decomp_size() - exact decompressed length of a compressed image
 *
 * Reads the decompressed size recorded in the compressed stream itself.
 * This is needed to decide whether the compressed source overlaps the
 * decompression destination: the *compressed* length says nothing about
 * how far the decompressor will write.
 *
 * @comp:	IH_COMP_* of the stream
 * @src:	compressed data (already in memory)
 * @len:	compressed length
 * Return: decompressed length, or 0 if this format does not record one
 *	   (LZ4, or an LZMA stream marked "unknown size")
 */
static ulong spl_decomp_size(int comp, const void *src, ulong len)
{
	const u8 *p = src;
	u64 size;
	int i;

	switch (comp) {
	case IH_COMP_LZMA:
		/* .lzma "alone" header: props[1], dict[4], size[8], LE */
		if (len < 13)
			return 0;
		for (size = 0, i = 7; i >= 0; i--)
			size = (size << 8) | p[5 + i];
		/* all-ones is the "size unknown" marker */
		if (size == ~0ULL || size > (u64)(ulong)~0UL)
			return 0;
		return (ulong)size;
	case IH_COMP_GZIP:
		/* gzip trailer: CRC32[4], ISIZE[4], LE (ISIZE is mod 2^32) */
		if (len < 18)
			return 0;
		return p[len - 4] | ((ulong)p[len - 3] << 8) |
		       ((ulong)p[len - 2] << 16) | ((ulong)p[len - 1] << 24);
	default:
		return 0;
	}
}

static int load_simple_fit(struct spl_load_info *info, ulong fit_offset,
			   const struct spl_fit_info *ctx, int node,
			   struct spl_image_info *image_info)
{
	int offset;
	size_t length;
	int len;
	ulong size;
	ulong load_addr;
	void *load_ptr;
	void *src;
	ulong overhead;
	uint8_t image_comp = -1, type = -1;
	const void *data;
	const void *fit = ctx->fit;
	bool external_data = false;

	log_debug("starting\n");
	if (CONFIG_IS_ENABLED(BOOTMETH_VBE) &&
	    xpl_get_phase(info) != IH_PHASE_NONE) {
		enum image_phase_t phase;
		int ret;

		ret = fit_image_get_phase(fit, node, &phase);
		/* if the image is for any phase, let's use it */
		if (ret == -ENOENT || phase == xpl_get_phase(info)) {
			log_debug("found\n");
		} else if (ret < 0) {
			log_debug("err=%d\n", ret);
			return ret;
		} else {
			log_debug("- phase mismatch, skipping this image\n");
			return -EBADSLT;
		}
	}

	/*
	 * The type is also what image_decomp() names in its per-node line
	 * ("Uncompressing Firmware to 40000"), so read it whenever a node can
	 * be decompressed, not only for FPGA/OS payloads. Left at -1 it
	 * prints "Invalid Image" for every compressed node -- which is what
	 * the LZMA path below did until it started passing @type through.
	 */
	if (IS_ENABLED(CONFIG_SPL_FPGA) || spl_decompression_enabled()) {
		if (fit_image_get_type(fit, node, &type))
			puts("Cannot get image type.\n");
		else
			debug("%s ", genimg_get_type_name(type));
	}

	if (spl_decompression_enabled()) {
		fit_image_get_comp(fit, node, &image_comp);
		debug("%s ", genimg_get_comp_name(image_comp));
	}

	if (fit_image_get_load(fit, node, &load_addr)) {
		if (!image_info->load_addr) {
			printf("Can't load %s: No load address and no buffer\n",
			       fit_get_name(fit, node, NULL));
			return -ENOBUFS;
		}
		load_addr = image_info->load_addr;
	}

	if (!fit_image_get_data_position(fit, node, &offset)) {
		external_data = true;
	} else if (!fit_image_get_data_offset(fit, node, &offset)) {
		log_debug("read offset %x = offset from fit %lx\n",
			  offset, (ulong)offset + ctx->ext_data_offset);
		offset += ctx->ext_data_offset;
		external_data = true;
	}

	if (external_data) {
		ulong read_offset;
		void *src_ptr;

		/* External data */
		if (fit_image_get_data_size(fit, node, &len))
			return -ENOENT;

		/* Dont bother to copy 0 byte data, but warn, though */
		if (!len) {
			log_warning("%s: Skip load '%s': image size is 0!\n",
				    __func__, fit_get_name(fit, node, NULL));
			return 0;
		}

		if (spl_decompression_enabled() &&
		    (image_comp == IH_COMP_GZIP || image_comp == IH_COMP_LZMA ||
		     image_comp == IH_COMP_LZ4)) {
			/*
			 * Staging buffer for compressed data.
			 *
			 * The historical choice is CONFIG_SYS_LOAD_ADDR, and
			 * it is not a memory-map statement at all: it is
			 * U-Boot-proper's `loadaddr`, the address a human
			 * types `tftp` to once U-Boot has relocated itself to
			 * the top of DRAM. It says nothing about which parts
			 * of DRAM SPL has already filled. Where it happens to
			 * land inside an image loaded EARLIER from this same
			 * FIT, the read below destroys that image after its
			 * own hash has already passed -- and nothing hashes it
			 * again, so the failure is silent.
			 *
			 * That is not hypothetical on rk3399, where
			 * SYS_LOAD_ADDR is TEXT_BASE + 0x800: 2 KiB into
			 * decompressed U-Boot. Every compressed node loaded
			 * after U-Boot writes over live .text -- and the
			 * control FDT is always such a node, because
			 * spl_fit_append_fdt() runs immediately after the
			 * U-Boot image is in memory.
			 *
			 * CONFIG_SPL_FIT_COMP_STAGING_ADDR lets a board name a
			 * window it knows is free instead. Zero, the default,
			 * keeps the old behaviour for every board that has not
			 * made that statement.
			 *
			 * Either way this is only a first guess: if it
			 * overlaps the decompression destination, place the
			 * staging buffer past the *compressed* extent. The
			 * decompressed extent is not known yet -- nothing has
			 * been read -- so it is re-checked exactly, and the
			 * buffer moved again if needed, once the data is in
			 * memory (below).
			 */
			ulong staging = CONFIG_SPL_FIT_COMP_STAGING_ADDR ?
					CONFIG_SPL_FIT_COMP_STAGING_ADDR :
					CONFIG_SYS_LOAD_ADDR;

			if (staging < load_addr + len &&
			    load_addr < staging + len)
				staging = ALIGN(load_addr + len,
						ARCH_DMA_MINALIGN);
			src_ptr = map_sysmem(ALIGN(staging,
						   ARCH_DMA_MINALIGN), len);
		} else
			src_ptr = map_sysmem(ALIGN(load_addr, ARCH_DMA_MINALIGN), len);
		length = len;

		overhead = get_aligned_image_overhead(info, offset);
		size = get_aligned_image_size(info, length, offset);
		read_offset = fit_offset + get_aligned_image_offset(info,
							    offset);
		log_debug("reading from offset %x / %lx size %lx to %p: ",
			  offset, read_offset, size, src_ptr);

		if (info->read(info, read_offset, size, src_ptr) < length)
			return -EIO;

		debug("External data: dst=%p, offset=%x, size=%lx\n",
		      src_ptr, offset, (unsigned long)length);
		src = src_ptr + overhead;
	} else {
		/* Embedded data */
		if (fit_image_get_emb_data(fit, node, &data, &length)) {
			puts("Cannot get image data/size\n");
			return -ENOENT;
		}
		debug("Embedded data: dst=%lx, size=%lx\n", load_addr,
		      (unsigned long)length);
		src = (void *)data;	/* cast away const */
	}

	if (CONFIG_IS_ENABLED(FIT_SIGNATURE)) {
		const char *name = fit_get_name(fit, node, NULL);

		if (!CONFIG_IS_ENABLED(FIT_QUIET))
			printf("## Checking hash(es) for Image %s ... ", name);
		if (!fit_image_verify_with_data(fit, node, gd_fdt_blob(), src,
						length)) {
			if (CONFIG_IS_ENABLED(FIT_QUIET))
				printf("\n## Image %s: hash check FAILED\n", name);
			return -EPERM;
		}
		if (!CONFIG_IS_ENABLED(FIT_QUIET))
			puts("OK\n");
	}

	if (CONFIG_IS_ENABLED(FIT_IMAGE_POST_PROCESS))
		board_fit_image_post_process(fit, node, &src, &length);

	/*
	 * Decompression reads from @src and writes to @load_addr. Where the
	 * two ranges overlap the result is only correct while the read
	 * pointer stays ahead of the write pointer, which depends on the
	 * compression ratio -- so an image that grows by a few percent can
	 * start corrupting itself with no diagnostic at all. The FIT hash
	 * cannot catch it: it is verified above, over the *compressed*
	 * bytes, before any of this runs.
	 *
	 * The compressed data is in memory now, so the decompressed extent
	 * is exactly known. Move the compressed copy clear of the
	 * destination if -- and only if -- they really overlap. Formats that
	 * do not record their decompressed size (LZ4) are left as they were.
	 */
	if ((IS_ENABLED(CONFIG_SPL_GZIP) && image_comp == IH_COMP_GZIP) ||
	    (IS_ENABLED(CONFIG_SPL_LZMA) && image_comp == IH_COMP_LZMA)) {
		ulong dst_len = spl_decomp_size(image_comp, src, length);
		ulong src_addr = map_to_sysmem(src);

		if (dst_len > CONFIG_SYS_BOOTM_LEN) {
			printf("Decompressed size %lx exceeds SYS_BOOTM_LEN %x\n",
			       dst_len, (uint)CONFIG_SYS_BOOTM_LEN);
			return -EFBIG;
		}

		if (dst_len && src_addr < load_addr + dst_len &&
		    load_addr < src_addr + length) {
			ulong safe = ALIGN(load_addr + dst_len,
					   ARCH_DMA_MINALIGN);

			if (safe < load_addr || safe + length < safe) {
				puts("No safe decompression staging buffer\n");
				return -EFBIG;
			}

			/*
			 * KNOWN LIMITATION, stated because the failure it
			 * leaves is the same silent class this code exists to
			 * remove.
			 *
			 * The only rejection above is arithmetic wrap. Nothing
			 * here checks that [safe, safe+length) is actually
			 * free: not against the end of DRAM, not against the
			 * SPL heap/stack/BSS, and -- the one that matters --
			 * not against an image ALREADY LOADED from this same
			 * FIT. spl_load_simple_fit() loads `firmware` first and
			 * then each loadable in order, so on a FIT whose
			 * earlier loadable happens to sit in
			 * [load_addr+dst_len, +length) this memmove writes
			 * compressed bytes over it AFTER that image's own hash
			 * has already passed.
			 *
			 * SPL has no memory map to check against, which is why
			 * this is documented rather than guarded. It does not
			 * bite the board this was written for -- its BL31
			 * segments are at 0x40000 and 0xff8c0000, nowhere near
			 * -- but a FIT carrying OP-TEE, an FPGA blob or a
			 * second firmware in that window would be corrupted
			 * silently.
			 */
			debug("Restaging compressed data %lx -> %lx (dst %lx+%lx)\n",
			      src_addr, safe, load_addr, dst_len);
			memmove(map_sysmem(safe, length), src, length);
			src = map_sysmem(safe, length);
		}
	}

	load_ptr = map_sysmem(load_addr, length);
	if (IS_ENABLED(CONFIG_SPL_GZIP) && image_comp == IH_COMP_GZIP) {
		size = length;
		if (gunzip(load_ptr, CONFIG_SYS_BOOTM_LEN, src, &size)) {
			puts("Uncompressing error\n");
			return -EIO;
		}
		length = size;
	} else if (IS_ENABLED(CONFIG_SPL_LZMA) && image_comp == IH_COMP_LZMA) {
		size = CONFIG_SYS_BOOTM_LEN;
		ulong loadEnd;

		/*
		 * @load and @type are only used for the message and for
		 * *load_end = load + decompressed length: hand over the
		 * node's real load address and type, so the line names the
		 * image that is being placed and where. The old call passed
		 * CONFIG_SYS_LOAD_ADDR and 0 (IH_TYPE_INVALID), which printed
		 * "Uncompressing Invalid Image to 800800" for every node on
		 * a board whose SYS_LOAD_ADDR is 0x800800 -- an address no
		 * SPL-loaded node on that board is ever written to.
		 */
		if (image_decomp(IH_COMP_LZMA, load_addr, 0, type,
				 load_ptr, src, length, size, &loadEnd)) {
			puts("Uncompressing error\n");
			return -EIO;
		}
		length = loadEnd - load_addr;
	} else if (IS_ENABLED(CONFIG_SPL_LZ4) && image_comp == IH_COMP_LZ4) {
		size_t lz4_size = CONFIG_SYS_BOOTM_LEN;

		if (ulz4fn(src, length, load_ptr, &lz4_size)) {
			puts("Uncompressing error\n");
			return -EIO;
		}
		length = lz4_size;
	} else {
		memmove(load_ptr, src, length);
	}

	if (image_info) {
		ulong entry_point;

		image_info->load_addr = load_addr;
		image_info->size = length;

		if (!fit_image_get_entry(fit, node, &entry_point))
			image_info->entry_point = entry_point;
		else
			image_info->entry_point = FDT_ERROR;
	}
	log_debug("- done loading\n");

	upl_add_image(fit, node, load_addr, length);

	return 0;
}

static bool os_takes_devicetree(uint8_t os)
{
	switch (os) {
	case IH_OS_U_BOOT:
		return true;
	case IH_OS_LINUX:
		return IS_ENABLED(CONFIG_SPL_OS_BOOT) ||
		       IS_ENABLED(CONFIG_SPL_OPENSBI);
	default:
		return false;
	}
}

__weak int board_spl_fit_append_fdt_skip(const char *name)
{
	return 0;	/* Do not skip */
}

static int spl_fit_append_fdt(struct spl_image_info *spl_image,
			      struct spl_load_info *info, ulong offset,
			      const struct spl_fit_info *ctx)
{
	struct spl_image_info image_info;
	int node, ret = 0, index = 0;

	/*
	 * Use the address following the image as target address for the
	 * device tree.
	 */
	image_info.load_addr = ALIGN(spl_image->load_addr + spl_image->size, 8);

	/* Figure out which device tree the board wants to use */
	node = spl_fit_get_image_node(ctx, FIT_FDT_PROP, index++);
	if (node < 0) {
		size_t size;

		debug("%s: cannot find FDT node\n", __func__);

		/*
		 * U-Boot did not find a device tree inside the FIT image. Use
		 * the U-Boot device tree instead.
		 */
		if (!gd->fdt_blob)
			return node;

		/*
		 * Make the load-address of the FDT available for the SPL
		 * framework
		 */
		size = fdt_totalsize(gd->fdt_blob);
		spl_image->fdt_addr = map_sysmem(image_info.load_addr, size);
		memcpy(spl_image->fdt_addr, gd->fdt_blob, size);
	} else {
		ret = load_simple_fit(info, offset, ctx, node, &image_info);
		if (ret < 0)
			return ret;

		spl_image->fdt_addr = phys_to_virt(image_info.load_addr);
	}

	if (CONFIG_IS_ENABLED(FIT_IMAGE_TINY))
		return 0;

#if CONFIG_IS_ENABLED(LOAD_FIT_APPLY_OVERLAY)
		void *tmpbuffer = NULL;

		for (; ; index++) {
			const char *str;

			ret = spl_fit_get_image_name(ctx, FIT_FDT_PROP, index, &str);
			if (ret == -E2BIG) {
				debug("%s: No additional FDT node\n", __func__);
				ret = 0;
				break;
			} else if (ret) {
				break;
			}

			ret = board_spl_fit_append_fdt_skip(str);
			if (ret)
				continue;

			node = fdt_subnode_offset(ctx->fit, ctx->images_node, str);
			if (node < 0) {
				debug("%s: unable to find FDT node %d\n",
				      __func__, index);
				continue;
			}

			if (!tmpbuffer) {
				/*
				 * allocate memory to store the DT overlay
				 * before it is applied. It may not be used
				 * depending on how the overlay is stored, so
				 * don't fail yet if the allocation failed.
				 */
				size_t size = CONFIG_SPL_LOAD_FIT_APPLY_OVERLAY_BUF_SZ;

				tmpbuffer = malloc_cache_aligned(size);
				if (!tmpbuffer)
					debug("%s: unable to allocate space for overlays\n",
					      __func__);
			}
			image_info.load_addr = (ulong)tmpbuffer;
			ret = load_simple_fit(info, offset, ctx, node,
					      &image_info);
			if (ret == -EBADSLT)
				continue;
			else if (ret < 0)
				break;

			/* Make room in FDT for changes from the overlay */
			ret = fdt_increase_size(spl_image->fdt_addr,
						image_info.size);
			if (ret < 0)
				break;

			ret = fdt_overlay_apply_verbose(spl_image->fdt_addr,
							(void *)image_info.load_addr);
			if (ret) {
				pr_err("failed to apply DT overlay %s\n",
				       fit_get_name(ctx->fit, node, NULL));
				break;
			}

			debug("%s: DT overlay %s applied\n", __func__,
			      fit_get_name(ctx->fit, node, NULL));
		}
		free(tmpbuffer);
		if (ret)
			return ret;
#endif
	/* Try to make space, so we can inject details on the loadables */
	ret = fdt_shrink_to_minimum(spl_image->fdt_addr, 8192);
	if (ret < 0)
		return ret;

	return ret;
}

static int spl_fit_record_loadable(const struct spl_fit_info *ctx, int index,
				   void *blob, struct spl_image_info *image)
{
	int ret = 0;
	const char *name;
	int node;

	ret = spl_fit_get_image_name(ctx, "loadables", index, &name);
	if (ret < 0)
		return ret;

	node = spl_fit_get_image_node(ctx, "loadables", index);

	ret = fdt_record_loadable(blob, index, name, image->load_addr,
			image->size, image->entry_point,
			fdt_getprop(ctx->fit, node, FIT_TYPE_PROP, NULL),
			fdt_getprop(ctx->fit, node, FIT_OS_PROP, NULL),
			fdt_getprop(ctx->fit, node, FIT_ARCH_PROP, NULL));

	return ret;
}

static int spl_fit_image_is_fpga(const void *fit, int node)
{
	const char *type;

	if (!IS_ENABLED(CONFIG_SPL_FPGA))
		return 0;

	type = fdt_getprop(fit, node, FIT_TYPE_PROP, NULL);
	if (!type)
		return 0;

	return !strcmp(type, "fpga");
}

static void spl_fit_image_record_arm32_optee(const void *fit, int node,
					     struct spl_image_info *spl_image,
					     struct spl_image_info *image_info)
{
#if defined(CONFIG_BOOTM_OPTEE) && defined(CONFIG_ARM) && !defined(CONFIG_ARM64)
	const char *type = fdt_getprop(fit, node, FIT_TYPE_PROP, NULL);

	if (!type)
		return;

	if (strcmp(type, "tee"))
		return;

	spl_image->optee_addr = image_info->load_addr;
#endif
}

static int spl_fit_image_get_os(const void *fit, int noffset, uint8_t *os)
{
	if (!CONFIG_IS_ENABLED(FIT_IMAGE_TINY) || CONFIG_IS_ENABLED(OS_BOOT))
		return fit_image_get_os(fit, noffset, os);

	const char *name = fdt_getprop(fit, noffset, FIT_OS_PROP, NULL);
	if (!name)
		return -ENOENT;

	/*
	 * We don't care what the type of the image actually is,
	 * only whether or not it is U-Boot. This saves some
	 * space by omitting the large table of OS types.
	 */
	if (!strcmp(name, "u-boot"))
		*os = IH_OS_U_BOOT;
	else
		*os = IH_OS_INVALID;

	return 0;
}

/*
 * The purpose of the FIT load buffer is to provide a memory location that is
 * independent of the load address of any FIT component.
 */
__weak void *board_spl_fit_buffer_addr(ulong fit_size, int sectors, int bl_len)
{
	void *buf;
	size_t size = sectors * bl_len;

	buf = malloc_cache_aligned(size);
	if (!buf) {
		pr_err("Could not get FIT buffer of %lu bytes\n", (ulong)size);

		if (IS_ENABLED(CONFIG_SPL_SYS_MALLOC))
			pr_err("\tcheck CONFIG_SPL_SYS_MALLOC_SIZE\n");
		else
			pr_err("\tcheck CONFIG_SPL_SYS_MALLOC_F_LEN\n");

		buf = spl_get_load_buffer(0, size);
	}
	return buf;
}

/*
 * Weak default function to allow customizing SPL fit loading for load-only
 * use cases by allowing to skip the parsing/processing of the FIT contents
 * (so that this can be done separately in a more customized fashion)
 */
__weak bool spl_load_simple_fit_skip_processing(void)
{
	return false;
}

/*
 * Weak default function to allow fixes after fit header
 * is loaded.
 */
__weak void *spl_load_simple_fit_fix_load(const void *fit)
{
	return (void *)fit;
}

static void warn_deprecated(const char *msg)
{
	printf("DEPRECATED: %s\n", msg);
	printf("\tSee https://fitspec.osfw.foundation/\n");
}

static int spl_fit_upload_fpga(struct spl_fit_info *ctx, int node,
			       struct spl_image_info *fpga_image)
{
	const char *compatible;
	int ret;
	int devnum = 0;
	int flags = 0;

	debug("FPGA bitstream at: %x, size: %x\n",
	      (u32)fpga_image->load_addr, fpga_image->size);

	compatible = fdt_getprop(ctx->fit, node, "compatible", NULL);
	if (!compatible) {
		warn_deprecated("'fpga' image without 'compatible' property");
	} else {
		if (CONFIG_IS_ENABLED(FPGA_LOAD_SECURE))
			flags = fpga_compatible2flag(devnum, compatible);
		if (strcmp(compatible, "u-boot,fpga-legacy"))
			debug("Ignoring compatible = %s property\n",
			      compatible);
	}

	ret = fpga_load(devnum, (void *)fpga_image->load_addr,
			fpga_image->size, BIT_FULL, flags);
	if (ret) {
		printf("%s: Cannot load the image to the FPGA\n", __func__);
		return ret;
	}

	puts("FPGA image loaded from FIT\n");

	return 0;
}

static int spl_fit_load_fpga(struct spl_fit_info *ctx,
			     struct spl_load_info *info, ulong offset)
{
	int node, ret;

	struct spl_image_info fpga_image = {
		.load_addr = 0,
	};

	node = spl_fit_get_image_node(ctx, "fpga", 0);
	if (node < 0)
		return node;

	warn_deprecated("'fpga' property in config node. Use 'loadables'");

	/* Load the image and set up the fpga_image structure */
	ret = load_simple_fit(info, offset, ctx, node, &fpga_image);
	if (ret) {
		printf("%s: Cannot load the FPGA: %i\n", __func__, ret);
		return ret;
	}

	return spl_fit_upload_fpga(ctx, node, &fpga_image);
}

static int spl_simple_fit_read(struct spl_fit_info *ctx,
			       struct spl_load_info *info, ulong offset,
			       const void *fit_header)
{
	unsigned long count, size;
	void *buf;

	/*
	 * For FIT with external data, figure out where the external images
	 * start. This is the base for the data-offset properties in each
	 * image.
	 */
	size = ALIGN(fdt_totalsize(fit_header), 4);
	size = board_spl_fit_size_align(size);
	ctx->ext_data_offset = ALIGN(size, 4);

	/*
	 * So far we only have one block of data from the FIT. Read the entire
	 * thing, including that first block.
	 *
	 * For FIT with data embedded, data is loaded as part of FIT image.
	 * For FIT with external data, data is not loaded in this step.
	 */
	size = get_aligned_image_size(info, size, 0);
	buf = board_spl_fit_buffer_addr(size, size, 1);
	if (!buf) {
		/*
		 * We assume that none of the board will ever use 0x0 as a
		 * valid load address. Theoretically some board could use it,
		 * but this is extremely unlikely.
		 */
		return -EIO;
	}

	count = info->read(info, offset, size, buf);
	if (!count) {
		/*
		 * FIT could not be read. This means we should free the
		 * memory allocated by board_spl_fit_buffer_addr().
		 * Unfortunately, we don't know what memory allocation
		 * mechanism was used:
		 *   - For the SPL_SYS_MALLOC_SIMPLE case nothing could
		 *     be done. The memory just could not be freed.
		 *   - For statically allocated memory buffer we can try
		 *     to reuse previously allocated memory (example:
		 *     board_spl_fit_buffer_addr() function from the
		 *     file test/image/spl_load.c).
		 *   - For normall malloc() -- memory leak can't be easily
		 *     avoided. To somehow reduce memory consumption the
		 *     next calls of board_spl_fit_buffer_addr() could
		 *     reallocate previously allocated buffer and use
		 *     them again. This is somethat similar to the approach
		 *     used for statically allocated buffer.
		 *
		 * Please note:
		 *   - FIT images with data placed outside of the FIT
		 *     structure will cause small memory leak (several
		 *     kilobytes),
		 *   - FIT images with data placed inside to the FIT
		 *     structure may cause huge memory leak (up to
		 *     several megabytes). Do NOT use such images!
		 */
		return -EIO;
	}

	ctx->fit = buf;
	debug("fit read offset %lx, size=%lu, dst=%p, count=%lu\n",
	      offset, size, buf, count);

	return 0;
}

static int spl_simple_fit_parse(struct spl_fit_info *ctx)
{
	/* Find the correct subnode under "/configurations" */
	ctx->conf_node = fit_find_config_node(ctx->fit);
	if (ctx->conf_node < 0)
		return -EINVAL;

	if (IS_ENABLED(CONFIG_SPL_FIT_SIGNATURE)) {
		const char *name = fit_get_name(ctx->fit, ctx->conf_node, NULL);

		if (!CONFIG_IS_ENABLED(FIT_QUIET))
			printf("## Checking hash(es) for config %s ... ", name);
		if (fit_config_verify(ctx->fit, ctx->conf_node)) {
			if (CONFIG_IS_ENABLED(FIT_QUIET))
				printf("\n## Config %s: hash check FAILED\n", name);
			return -EPERM;
		}
		if (!CONFIG_IS_ENABLED(FIT_QUIET))
			puts("OK\n");
	}

	/* find the node holding the images information */
	ctx->images_node = fdt_path_offset(ctx->fit, FIT_IMAGES_PATH);
	if (ctx->images_node < 0) {
		debug("%s: Cannot find /images node: %d\n", __func__,
		      ctx->images_node);
		return -EINVAL;
	}

	return 0;
}

int spl_load_simple_fit(struct spl_image_info *spl_image,
			struct spl_load_info *info, ulong offset, void *fit)
{
	struct spl_image_info image_info;
	struct spl_fit_info ctx;
	int node = -1;
	int ret;
	int index = 0;
	int firmware_node;

	ret = spl_simple_fit_read(&ctx, info, offset, fit);
	if (ret < 0)
		return ret;

	/* skip further processing if requested to enable load-only use cases */
	if (spl_load_simple_fit_skip_processing())
		return 0;

	ctx.fit = spl_load_simple_fit_fix_load(ctx.fit);

	ret = spl_simple_fit_parse(&ctx);
	if (ret < 0)
		return ret;

	if (IS_ENABLED(CONFIG_SPL_FPGA))
		spl_fit_load_fpga(&ctx, info, offset);

	/*
	 * Find the U-Boot image using the following search order:
	 *   - start at 'firmware' (e.g. an ARM Trusted Firmware)
	 *   - fall back 'kernel' (e.g. a Falcon-mode OS boot
	 *   - fall back to using the first 'loadables' entry
	 */
	if (node < 0)
		node = spl_fit_get_image_node(&ctx, FIT_FIRMWARE_PROP, 0);

	if (node < 0 && IS_ENABLED(CONFIG_SPL_OS_BOOT))
		node = spl_fit_get_image_node(&ctx, FIT_KERNEL_PROP, 0);

	if (node < 0) {
		debug("could not find firmware image, trying loadables...\n");
		node = spl_fit_get_image_node(&ctx, "loadables", 0);
		/*
		 * If we pick the U-Boot image from "loadables", start at
		 * the second image when later loading additional images.
		 */
		index = 1;
	}
	if (node < 0) {
		debug("%s: Cannot find u-boot image node: %d\n",
		      __func__, node);
		return -1;
	}

	/* Load the image and set up the spl_image structure */
	ret = load_simple_fit(info, offset, &ctx, node, spl_image);
	if (ret)
		return ret;

	/*
	 * For backward compatibility, we treat the first node that is
	 * as a U-Boot image, if no OS-type has been declared.
	 */
	if (!spl_fit_image_get_os(ctx.fit, node, &spl_image->os))
		debug("Image OS is %s\n", genimg_get_os_name(spl_image->os));
	else if (!IS_ENABLED(CONFIG_SPL_OS_BOOT))
		spl_image->os = IH_OS_U_BOOT;

	/*
	 * Booting a next-stage U-Boot may require us to append the FDT.
	 * We allow this to fail, as the U-Boot image might embed its FDT --
	 * but not when the FDT is there and FAILED its hash check (-EPERM,
	 * from spl_load_fit_image()): that is a corrupt or tampered image,
	 * not a missing one, and booting on it hands U-Boot a bad DTB.
	 */
	if (os_takes_devicetree(spl_image->os)) {
		ret = spl_fit_append_fdt(spl_image, info, offset, &ctx);
		if (ret < 0 && (spl_image->os != IH_OS_U_BOOT || ret == -EPERM))
			return ret;
	}

	firmware_node = node;
	/* Now check if there are more images for us to load */
	for (; ; index++) {
		uint8_t os_type = IH_OS_INVALID;

		node = spl_fit_get_image_node(&ctx, "loadables", index);
		if (node < 0)
			break;

		/*
		 * if the firmware is also a loadable, skip it because
		 * it already has been loaded. This is typically the case with
		 * u-boot.img generated by mkimage.
		 */
		if (firmware_node == node)
			continue;

		image_info.load_addr = 0;
		ret = load_simple_fit(info, offset, &ctx, node, &image_info);
		if (ret < 0 && ret != -EBADSLT) {
			printf("%s: can't load image loadables index %d (ret = %d)\n",
			       __func__, index, ret);
			return ret;
		}

		if (spl_fit_image_is_fpga(ctx.fit, node))
			spl_fit_upload_fpga(&ctx, node, &image_info);

		if (!spl_fit_image_get_os(ctx.fit, node, &os_type))
			debug("Loadable is %s\n", genimg_get_os_name(os_type));

		if (os_takes_devicetree(os_type)) {
			/*
			 * Same rule as for the firmware image above: a missing
			 * FDT is tolerated, one that failed its hash check is
			 * not.  This return used to be ignored, so a loadable
			 * U-Boot (BL31 as the firmware, as on Meson GX) booted
			 * on a DTB the SPL had just reported "sha256 error".
			 */
			ret = spl_fit_append_fdt(&image_info, info, offset,
						 &ctx);
			if (ret == -EPERM)
				return ret;
			spl_image->fdt_addr = image_info.fdt_addr;
		}

		/*
		 * If the "firmware" image did not provide an entry point,
		 * use the first valid entry point from the loadables.
		 */
		if (spl_image->entry_point == FDT_ERROR &&
		    image_info.entry_point != FDT_ERROR)
			spl_image->entry_point = image_info.entry_point;

		spl_fit_image_record_arm32_optee(ctx.fit, node, spl_image,
						 &image_info);

		/* Record our loadables into the FDT */
		if (!CONFIG_IS_ENABLED(FIT_IMAGE_TINY) &&
		    xpl_get_fdt_update(info) && spl_image->fdt_addr)
			spl_fit_record_loadable(&ctx, index,
						spl_image->fdt_addr,
						&image_info);
	}

	/*
	 * If a platform does not provide CONFIG_SYS_UBOOT_START, U-Boot's
	 * Makefile will set it to 0 and it will end up as the entry point
	 * here. What it actually means is: use the load address.
	 */
	if (spl_image->entry_point == FDT_ERROR || spl_image->entry_point == 0)
		spl_image->entry_point = spl_image->load_addr;

	spl_image->flags |= SPL_FIT_FOUND;
	upl_set_fit_info(map_to_sysmem(ctx.fit), ctx.conf_node,
			 spl_image->entry_point);

	return 0;
}

/* Parse and load full fitImage in SPL */
int spl_load_fit_image(struct spl_image_info *spl_image,
		       const struct legacy_img_hdr *header)
{
	struct bootm_headers images;
	const char *fit_uname_config = NULL;
	ulong fdt_hack;
	const char *uname;
	ulong fw_data = 0, dt_data = 0, img_data = 0;
	ulong fw_len = 0, dt_len = 0, img_len = 0;
	int idx, conf_noffset;
	int ret;

#ifdef CONFIG_SPL_FIT_SIGNATURE
	images.verify = 1;
#endif
	ret = fit_image_load(&images, virt_to_phys((void *)header),
			     NULL, &fit_uname_config,
			     IH_ARCH_DEFAULT, IH_TYPE_STANDALONE, -1,
			     FIT_LOAD_OPTIONAL, &fw_data, &fw_len);
	if (ret >= 0) {
		printf("DEPRECATED: 'standalone = ' property.");
		printf("Please use either 'firmware =' or 'kernel ='\n");
	} else {
		ret = fit_image_load(&images, virt_to_phys((void *)header),
				     NULL, &fit_uname_config, IH_ARCH_DEFAULT,
				     IH_TYPE_FIRMWARE, -1, FIT_LOAD_OPTIONAL,
				     &fw_data, &fw_len);
	}

	if (ret < 0) {
		ret = fit_image_load(&images, virt_to_phys((void *)header),
				     NULL, &fit_uname_config, IH_ARCH_DEFAULT,
				     IH_TYPE_KERNEL, -1, FIT_LOAD_OPTIONAL,
				     &fw_data, &fw_len);
	}

	if (ret < 0)
		return ret;

	spl_image->size = fw_len;
	spl_image->load_addr = fw_data;
	if (fit_image_get_entry(header, ret, &spl_image->entry_point))
		spl_image->entry_point = fw_data;
	if (fit_image_get_os(header, ret, &spl_image->os))
		spl_image->os = IH_OS_INVALID;
	spl_image->name = genimg_get_os_name(spl_image->os);

	debug(PHASE_PROMPT "payload image: %32s load addr: 0x%lx size: %d\n",
	      spl_image->name, spl_image->load_addr, spl_image->size);

#ifdef CONFIG_SPL_FIT_SIGNATURE
	images.verify = 1;
#endif
	ret = fit_image_load(&images, virt_to_phys((void *)header), NULL,
			     &fit_uname_config, IH_ARCH_DEFAULT, IH_TYPE_FLATDT,
			     -1, FIT_LOAD_OPTIONAL, &dt_data, &dt_len);
	if (ret >= 0) {
		spl_image->fdt_addr = (void *)dt_data;

		if (spl_image->os == IH_OS_U_BOOT) {
			/* HACK: U-Boot expects FDT at a specific address */
			fdt_hack = spl_image->load_addr + spl_image->size;
			fdt_hack = (fdt_hack + 3) & ~3;
			debug("Relocating FDT to %p\n", spl_image->fdt_addr);
			memcpy((void *)fdt_hack, spl_image->fdt_addr, dt_len);
		}
	}

	conf_noffset = fit_conf_get_node((const void *)header,
					 fit_uname_config);
	if (conf_noffset < 0)
		return 0;

	for (idx = 0;
	     uname = fdt_stringlist_get((const void *)header, conf_noffset,
					FIT_LOADABLE_PROP, idx,
				NULL), uname;
	     idx++) {
#ifdef CONFIG_SPL_FIT_SIGNATURE
		images.verify = 1;
#endif
		ret = fit_image_load(&images, (ulong)header,
				     &uname, &fit_uname_config,
				     IH_ARCH_DEFAULT, IH_TYPE_LOADABLE, -1,
				     FIT_LOAD_OPTIONAL_NON_ZERO,
				     &img_data, &img_len);
		if (ret < 0)
			return ret;
	}
	spl_image->flags |= SPL_FIT_FOUND;

	upl_set_fit_info(map_to_sysmem(header), conf_noffset,
			 spl_image->entry_point);

	return 0;
}
