// SPDX-License-Identifier: GPL-2.0
/*
 * Amlogic Meson Video Processing Unit driver
 *
 * Copyright (c) 2018 BayLibre, SAS.
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <display.h>
#include <dm.h>
#include <asm/io.h>
#include <efi_loader.h>
#include <fdt_support.h>
#include <log.h>
#include <part.h>
#include <linux/sizes.h>
#include <asm/arch/mem.h>
#include <asm/global_data.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>

#include "meson_vpu.h"
#include "meson_registers.h"
#include "simplefb_common.h"

#define MESON_VPU_OVERSCAN SZ_64K

/*
 * HHI clock registers, byte offsets into the vpu node's second reg range.
 * Datasheet word indices 0x6F and 0x7D.
 */
#define MESON_HHI_VPU_CLK_CNTL		0x1bc
#define MESON_HHI_VAPBCLK_CNTL		0x1f4

/* vpu_clk_0: mux [10:9] (0 = fclk_div4), divider [6:0] (N-1), gate BIT(8) */
#define MESON_VPU_CLK_SEL_FCLK_DIV4	(0 << 9)
#define MESON_VPU_CLK_GATE		BIT(8)
/* vapb_clk_0 = fclk_div4 / 2; path gate BIT(8) and final gate BIT(30) */
#define MESON_VAPB_CLK_ENABLE		(BIT(30) | BIT(8) | 1)

/* Static variable for use in meson_vpu_rsv_fb() */
static struct meson_framebuffer {
	u64 base;
	u64 fb_size;
	unsigned int xsize;
	unsigned int ysize;
	bool is_cvbs;
} meson_fb = { 0 };

bool meson_vpu_is_compatible(struct meson_vpu_priv *priv,
			     enum vpu_compatible family)
{
	enum vpu_compatible compat = dev_get_driver_data(priv->dev);

	return compat == family;
}

static int meson_vpu_setup_mode(struct udevice *dev, struct udevice *disp)
{
	struct video_uc_plat *uc_plat = dev_get_uclass_plat(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct display_timing timing;
	bool is_cvbs = false;
	int ret = 0;

	if (disp) {
		ret = display_read_timing(disp, &timing);
		if (ret) {
			debug("%s: Failed to read timings\n", __func__);
			goto cvbs;
		}

		uc_priv->xsize = timing.hactive.typ;
		uc_priv->ysize = timing.vactive.typ;

		ret = display_enable(disp, 0, &timing);
		if (ret)
			goto cvbs;
	} else {
cvbs:
		/* CVBS has a fixed 720x480i (NTSC) and 720x576i (PAL) */
		is_cvbs = true;
		timing.flags = DISPLAY_FLAGS_INTERLACED;
		uc_priv->xsize = 720;
		uc_priv->ysize = 576;
	}

	uc_priv->bpix = VPU_MAX_LOG2_BPP;

	meson_fb.is_cvbs = is_cvbs;
	meson_fb.xsize = uc_priv->xsize;
	meson_fb.ysize = uc_priv->ysize;

	/* Move the framebuffer to the end of addressable ram */
	meson_fb.fb_size = ALIGN(meson_fb.xsize * meson_fb.ysize *
				 ((1 << VPU_MAX_LOG2_BPP) / 8) +
				 MESON_VPU_OVERSCAN, EFI_PAGE_SIZE);
	meson_fb.base = gd->bd->bi_dram[0].start +
			gd->bd->bi_dram[0].size - meson_fb.fb_size;

	/* Override the framebuffer address */
	uc_plat->base = meson_fb.base;

	meson_vpu_setup_plane(dev, timing.flags & DISPLAY_FLAGS_INTERLACED);
	meson_vpu_setup_venc(dev, &timing, is_cvbs);
	meson_vpu_setup_vclk(dev, &timing, is_cvbs);

	video_set_flush_dcache(dev, 1);

	return 0;
}

static const struct udevice_id meson_vpu_ids[] = {
	{ .compatible = "amlogic,meson-gxbb-vpu", .data = VPU_COMPATIBLE_GXBB },
	{ .compatible = "amlogic,meson-gxl-vpu", .data = VPU_COMPATIBLE_GXL },
	{ .compatible = "amlogic,meson-gxm-vpu", .data = VPU_COMPATIBLE_GXM },
	{ .compatible = "amlogic,meson-g12a-vpu", .data = VPU_COMPATIBLE_G12A },
	{ }
};

static int meson_vpu_probe(struct udevice *dev)
{
	struct meson_vpu_priv *priv = dev_get_priv(dev);
	struct udevice *disp;
	int ret;

	/* Before relocation we don't need to do anything */
	if (!(gd->flags & GD_FLG_RELOC))
		return 0;

	priv->dev = dev;

	priv->io_base = dev_remap_addr_index(dev, 0);
	if (!priv->io_base)
		return -EINVAL;

	priv->hhi_base = dev_remap_addr_index(dev, 1);
	if (!priv->hhi_base)
		return -EINVAL;

	priv->dmc_base = dev_remap_addr_index(dev, 2);
	if (!priv->dmc_base)
		return -EINVAL;

	/*
	 * Give the VPU a clock that is actually RUNNING before touching VCBUS.
	 *
	 * The meson clk driver models vpu_clk as a GATE only
	 * (drivers/clk/meson/gxbb.c: MESON_GATE(CLKID_VPU_0, HHI_VPU_CLK_CNTL,
	 * 8)); it never programs the mux or the divider.  So the clk_enable_bulk()
	 * the power-domain driver issues gates whatever selection the earlier
	 * firmware left behind, and never checks that the selection produces a
	 * clock.  Linux does not have this problem because meson-gxl.dtsi drives
	 * CLKID_VPU_0_SEL/_DIV through assigned-clocks; nothing in u-boot does.
	 *
	 * On the GXL open boot chain that inherited selection is mux 1
	 * (fclk_div3), and that mux position delivers NO CLOCK on S905X.
	 * Measured on AML-S905X-CC-V2 from EL3 with the SoC's own clock-measure
	 * block (CBUS 0xc1108758, channel 46 "vpu"): 0 Hz at mux 1, 0 Hz at
	 * mux 2, 62 500 000 Hz at mux 0 with the same /8 divider -- while
	 * fclk_div5 measures 399 984 375 Hz on channel 43, so the source clocks
	 * are alive and it is the selection, not the source, that is dead.
	 *
	 * An unclocked VPU never completes a bus transfer and this bus has no
	 * timeout, so the FIRST VCBUS access below (meson_vpu_init()'s
	 * VPU_RDARB_MODE_L1C1 at 0xd0109e40) never retires: a bus STALL, not an
	 * abort, with no exception to report it and no console output.  It
	 * stalls identically from EL2 non-secure and from EL3 secure, which is
	 * what ruled out a security filter.
	 *
	 * Program it here, in the order Amlogic's own GXL VPU driver uses -- mux
	 * and divider first with the gate closed, gate afterwards:
	 * vpu_clk = fclk_div4 / 1 = 500 MHz, vapb_clk = fclk_div4 / 2 = 250 MHz.
	 * G12A and later are excluded: their clock tree is different and they are
	 * not affected.
	 */
	if (!meson_vpu_is_compatible(priv, VPU_COMPATIBLE_G12A)) {
		writel(MESON_VPU_CLK_SEL_FCLK_DIV4,
		       priv->hhi_base + MESON_HHI_VPU_CLK_CNTL);
		writel(MESON_VPU_CLK_SEL_FCLK_DIV4 | MESON_VPU_CLK_GATE,
		       priv->hhi_base + MESON_HHI_VPU_CLK_CNTL);
		writel(MESON_VAPB_CLK_ENABLE,
		       priv->hhi_base + MESON_HHI_VAPBCLK_CNTL);
	}

	meson_vpu_init(dev);

	/* probe the display */
	ret = uclass_get_device(UCLASS_DISPLAY, 0, &disp);

	return meson_vpu_setup_mode(dev, ret ? NULL : disp);
}

static int meson_vpu_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	plat->size = VPU_MAX_WIDTH * VPU_MAX_HEIGHT *
		(1 << VPU_MAX_LOG2_BPP) / 8;

	return 0;
}

#if defined(CONFIG_VIDEO_DT_SIMPLEFB)
static void meson_vpu_setup_simplefb(void *fdt)
{
	const char *pipeline = NULL;
	u64 mem_start, mem_size;
	int offset, ret;

	if (meson_fb.is_cvbs)
		pipeline = "vpu-cvbs";
	else
		pipeline = "vpu-hdmi";

	offset = meson_simplefb_fdt_match(fdt, pipeline);
	if (offset < 0) {
		eprintf("Cannot setup simplefb: node not found\n");

		/* If simplefb is missing, add it as reserved memory */
		meson_board_add_reserved_memory(fdt, meson_fb.base,
						meson_fb.fb_size);

		return;
	}

	/*
	 * SimpleFB will try to iomap the framebuffer, so we can't use
	 * fdt_add_mem_rsv on the memory area. Instead, the FB is stored
	 * at the end of the RAM and we strip this portion from the kernel
	 * allowed region
	 */
	mem_start = gd->bd->bi_dram[0].start;
	mem_size = gd->bd->bi_dram[0].size - meson_fb.fb_size;
	ret = fdt_fixup_memory_banks(fdt, &mem_start, &mem_size, 1);
	if (ret) {
		eprintf("Cannot setup simplefb: Error reserving memory\n");
		return;
	}

	ret = fdt_setup_simplefb_node(fdt, offset, meson_fb.base,
				      meson_fb.xsize, meson_fb.ysize,
				      meson_fb.xsize * 4, "x8r8g8b8");
	if (ret)
		eprintf("Cannot setup simplefb: Error setting properties\n");
}
#endif

void meson_vpu_rsv_fb(void *fdt)
{
	if (!meson_fb.base || !meson_fb.xsize || !meson_fb.ysize)
		return;

#if defined(CONFIG_EFI_LOADER)
	efi_add_memory_map(meson_fb.base, meson_fb.fb_size,
			   EFI_RESERVED_MEMORY_TYPE);
#endif
#if defined(CONFIG_VIDEO_DT_SIMPLEFB)
	meson_vpu_setup_simplefb(fdt);
#endif
}

U_BOOT_DRIVER(meson_vpu) = {
	.name = "meson_vpu",
	.id = UCLASS_VIDEO,
	.of_match = meson_vpu_ids,
	.probe = meson_vpu_probe,
	.bind = meson_vpu_bind,
	.priv_auto	= sizeof(struct meson_vpu_priv),
	.flags  = DM_FLAG_PRE_RELOC | DM_FLAG_LEAVE_PD_ON,
};
