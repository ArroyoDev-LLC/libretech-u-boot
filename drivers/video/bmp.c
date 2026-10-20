// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2002
 * Detlev Zundel, DENX Software Engineering, dzu@denx.de.
 */

/*
 * BMP handling routines
 */

#include <bmp_layout.h>
#include <command.h>
#include <dm.h>
#include <gzip.h>
#include <lzma/LzmaDec.h>
#include <log.h>
#include <malloc.h>
#include <mapmem.h>
#include <splash.h>
#include <video.h>
#include <asm/byteorder.h>

/*
 * Allocate and decompress a BMP image using gunzip() or lzma.
 *
 * Returns a pointer to the decompressed image data. This pointer is
 * aligned to 32-bit-aligned-address + 2.
 * See doc/README.displaying-bmps for explanation.
 *
 * The allocation address is passed to 'alloc_addr' and must be freed
 * by the caller after use.
 *
 * Returns NULL if decompression failed, or if the decompressed data
 * didn't contain a valid BMP signature or decompression is not enabled in
 * Kconfig.
 */
struct bmp_image *decompress_bmp(unsigned long addr, unsigned long *lenp,
				void **alloc_addr)
{
	void *dst;
	unsigned long len;
	struct bmp_image *bmp;

	if (!CONFIG_IS_ENABLED(VIDEO_BMP_GZIP))
		return NULL;

	/*
	 * Decompress bmp image
	 */
	len = CONFIG_VAL(VIDEO_LOGO_MAX_SIZE);
	/* allocate extra 3 bytes for 32-bit-aligned-address + 2 alignment */
	dst = malloc(CONFIG_VAL(VIDEO_LOGO_MAX_SIZE) + 3);
	if (!dst) {
		puts("Error: malloc in decompress failed!\n");
		return NULL;
	}

	/* align to 32-bit-aligned-address + 2 */
	bmp = dst + 2;

#ifdef CONFIG_VIDEO_BMP_GZIP
	if (CONFIG_IS_ENABLED(VIDEO_BMP_GZIP)) {
		if (gunzip(bmp, CONFIG_VAL(VIDEO_LOGO_MAX_SIZE), map_sysmem(addr, 0),
			   &len) == 0) {
			goto check_bmp;
		}
	}
#endif

	free(dst);
	return NULL;

check_bmp:
	if (len == CONFIG_VAL(VIDEO_LOGO_MAX_SIZE))
		puts("Image could be truncated (increase CONFIG_VIDEO_LOGO_MAX_SIZE)!\n");

	/*
	 * Check for bmp mark 'BM'
	 */
	if (!((bmp->header.signature[0] == 'B') &&
	      (bmp->header.signature[1] == 'M'))) {
		free(dst);
		return NULL;
	}

	debug("Decompressed BMP image detected!\n");

	*alloc_addr = dst;
	return bmp;
}

int bmp_info(ulong addr)
{
	struct bmp_image *bmp = (struct bmp_image *)map_sysmem(addr, 0);
	void *bmp_alloc_addr = NULL;
	unsigned long len;

	if (!((bmp->header.signature[0] == 'B') &&
	      (bmp->header.signature[1] == 'M')))
		bmp = decompress_bmp(addr, &len, &bmp_alloc_addr);

	if (!bmp) {
		printf("There is no valid bmp file at the given address\n");
		return 1;
	}

	printf("Image size    : %d x %d\n", le32_to_cpu(bmp->header.width),
	       le32_to_cpu(bmp->header.height));
	printf("Bits per pixel: %d\n", le16_to_cpu(bmp->header.bit_count));
	printf("Compression   : %d\n", le32_to_cpu(bmp->header.compression));

	if (bmp_alloc_addr)
		free(bmp_alloc_addr);

	return 0;
}

int bmp_display(ulong addr, int x, int y)
{
	struct udevice *dev;
	int ret, r;
	struct bmp_image *bmp = map_sysmem(addr, 0);
	void *bmp_alloc_addr = NULL;
	unsigned long len;

	if (!((bmp->header.signature[0] == 'B') &&
	      (bmp->header.signature[1] == 'M')))
		bmp = decompress_bmp(addr, &len, &bmp_alloc_addr);

	if (!bmp) {
		printf("There is no valid bmp file at the given address\n");
		return 1;
	}
	addr = map_to_sysmem(bmp);

	/*
	 * Draw on a video device that is actually driving a display.
	 *
	 * uclass_first_device_err() takes the FIRST BOUND device and never
	 * advances, and bind order is DT order (uclass.c). On RK3399 that is
	 * rk3399-base.dtsi's vopl@ff8f0000, ahead of vopb@ff900000, with no
	 * alias to order them -- so the splash was aimed at the little VOP,
	 * which is precisely the one rk_vop_probe_ranked() leaves idle when the
	 * big VOP has not been probed yet. And because uclass_first_device_err()
	 * reports that VOP's probe error rather than moving on, the result was
	 * not "the splash went to the wrong screen": it was no splash at all.
	 *
	 * Measured on ROC-RK3399-PC, unattended cold boot, no typed commands:
	 * splash_screen_prepare() succeeded (a valid 332x394 BMP sat at
	 * splashimage=0x800800), and a pixel the logo paints RGB(52,44,42) read
	 * back 0x00000000 in BOTH framebuffers -- 0xf6000000 (VOPL) and
	 * 0xf4000000 (VOPB). Nothing was ever drawn. That readback is the
	 * load-bearing evidence and it stands.
	 *
	 * The same run had `vop list` reporting BOTH VOPs ACTIVE with different
	 * modes (VOPL/HDMI 1920x1080, VOPB/DP 3840x2160), which is a state the
	 * hardware cannot actually hold: both DCLKs hang off VPLL, so the second
	 * VOP programmed took the first one's pixel clock. At most one of those
	 * two lines described a picture. rk_vop_probe() now refuses to bring up
	 * a second VOP while one is scanning out (rk_vop_find_lit()), so the
	 * ambiguity is gone as well as the double-programming that caused it.
	 *
	 * So iterate. _check() is the variant that keeps going past a device
	 * whose probe failed (uclass_first_device() stops dead there, which
	 * would reproduce the bug); a non-zero per-device ret means that one did
	 * not come up, and xsize/ysize are the uclass's own record of a mode it
	 * can scan out -- an active device with a 0x0 mode has no display to
	 * draw on. Boards with exactly one video device are unaffected: the
	 * first device is the only device.
	 *
	 * There is deliberately no RANKING here, and this loop takes the FIRST
	 * device with a non-zero mode, which on RK3399 is still bind-order
	 * first. That is sufficient rather than lucky: with the single-live-VOP
	 * rule enforced in rk_vop.c at most one video device can have a mode at
	 * all, so "the first one with a mode" and "the one driving the display"
	 * are the same device. If a future SoC ever scans out two at once, this
	 * loop needs a rank, not just a filter.
	 */
	ret = -ENODEV;
	for (r = uclass_first_device_check(UCLASS_VIDEO, &dev);
	     dev;
	     r = uclass_next_device_check(&dev)) {
		struct video_priv *uc_priv;

		if (r)
			continue;

		uc_priv = dev_get_uclass_priv(dev);
		if (!uc_priv->xsize || !uc_priv->ysize)
			continue;

		ret = 0;
		break;
	}

	if (!ret) {
		bool align = false;

		if (x == BMP_ALIGN_CENTER || y == BMP_ALIGN_CENTER)
			align = true;

		ret = video_bmp_display(dev, addr, x, y, align);
	} else {
		log_debug("bmp: no video device is driving a display\n");
	}

	if (bmp_alloc_addr)
		free(bmp_alloc_addr);

	return ret ? CMD_RET_FAILURE : 0;
}
