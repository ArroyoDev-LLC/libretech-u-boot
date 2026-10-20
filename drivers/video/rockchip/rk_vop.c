// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015 Google, Inc
 * Copyright 2014 Rockchip Inc.
 */

#include <clk.h>
#include <command.h>
#include <display.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <edid.h>
#include <log.h>
#include <regmap.h>
#include <reset.h>
#include <syscon.h>
#include <video.h>
#include <asm/global_data.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/edp_rk3288.h>
#include <asm/arch-rockchip/vop_rk3288.h>
#include <dm/device-internal.h>
#include <dm/uclass-internal.h>
#include <efi.h>
#include <efi_loader.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/string.h>
#include <power/regulator.h>
#include <vsprintf.h>
#include "rk_vop.h"

DECLARE_GLOBAL_DATA_PTR;

/*
 * True only for the duration of a device_probe() issued by the `vop` command
 * below.  The port walk is normally silent because it runs from
 * stdio/console init on every boot and a board with no monitor attached would
 * otherwise print an error per port, per VOP, every time.  That silence is
 * exactly what makes a hand-driven bring-up attempt useless, so the command
 * turns the same messages into printf() for its own call and back off
 * afterwards.  The boot path is byte-for-byte unchanged: with the flag clear
 * vop_msg() is debug(), which is what these sites already were.
 */
static bool vop_cmd_verbose;

#define vop_msg(fmt, ...)						\
	do {								\
		if (vop_cmd_verbose)					\
			printf(fmt, ##__VA_ARGS__);			\
		else							\
			debug(fmt, ##__VA_ARGS__);			\
	} while (0)

static const char *vop_mode_name(enum vop_modes mode)
{
	switch (mode) {
	case VOP_MODE_EDP:
		return "eDP";
	case VOP_MODE_MIPI:
		return "MIPI";
	case VOP_MODE_HDMI:
		return "HDMI";
	case VOP_MODE_LVDS:
		return "LVDS";
	case VOP_MODE_DP:
		return "DP";
	}

	return "?";
}

enum vop_pol {
	HSYNC_POSITIVE = 0,
	VSYNC_POSITIVE = 1,
	DEN_NEGATIVE   = 2,
	DCLK_INVERT    = 3
};

static void rkvop_enable(struct udevice *dev, ulong fbbase,
			 int fb_bits_per_pixel,
			 const struct display_timing *edid,
			 struct reset_ctl *dclk_rst)
{
	struct rk_vop_priv *priv = dev_get_priv(dev);
	struct rk3288_vop *regs = priv->regs;
	struct rk3288_vop *win_regs = priv->regs + priv->win_offset;
	u32 lb_mode;
	u32 rgb_mode;
	u32 hactive = edid->hactive.typ;
	u32 vactive = edid->vactive.typ;
	int ret;

	writel(V_ACT_WIDTH(hactive - 1) | V_ACT_HEIGHT(vactive - 1),
	       &win_regs->win0_act_info);

	writel(V_DSP_XST(edid->hsync_len.typ + edid->hback_porch.typ) |
	       V_DSP_YST(edid->vsync_len.typ + edid->vback_porch.typ),
	       &win_regs->win0_dsp_st);

	writel(V_DSP_WIDTH(hactive - 1) |
		V_DSP_HEIGHT(vactive - 1),
		&win_regs->win0_dsp_info);

	clrsetbits_le32(&win_regs->win0_color_key, M_WIN0_KEY_EN | M_WIN0_KEY_COLOR,
			V_WIN0_KEY_EN(0) | V_WIN0_KEY_COLOR(0));

	switch (fb_bits_per_pixel) {
	case 16:
		rgb_mode = RGB565;
		writel(V_RGB565_VIRWIDTH(hactive), &win_regs->win0_vir);
		break;
	case 24:
		rgb_mode = RGB888;
		writel(V_RGB888_VIRWIDTH(hactive), &win_regs->win0_vir);
		break;
	case 32:
	default:
		rgb_mode = ARGB8888;
		writel(V_ARGB888_VIRWIDTH(hactive), &win_regs->win0_vir);
		break;
	}

	if (hactive > 2560)
		lb_mode = LB_RGB_3840X2;
	else if (hactive > 1920)
		lb_mode = LB_RGB_2560X4;
	else if (hactive > 1280)
		lb_mode = LB_RGB_1920X5;
	else
		lb_mode = LB_RGB_1280X8;

	/*
	 * M_WIN0_RB_SWAP is in the clear mask so the format we just chose does
	 * not depend on the register's prior contents. Linux writes this bit
	 * explicitly for every window (rb_swap = has_rb_swapped(format), which
	 * is 0 for XRGB8888 -- what ARGB8888 here means); leaving it out made
	 * the pixel order rely on the AHB reset default. Measured as already 0
	 * on this board (WIN0_CTRL0 at 0xff900030 read 0x3a000041), so this is
	 * a no-op today and exists so it stays one.
	 */
	clrsetbits_le32(&win_regs->win0_ctrl0,
			M_WIN0_LB_MODE | M_WIN0_DATA_FMT | M_WIN0_RB_SWAP |
			M_WIN0_EN,
			V_WIN0_LB_MODE(lb_mode) | V_WIN0_DATA_FMT(rgb_mode) |
			V_WIN0_EN(1));

	writel(fbbase, &win_regs->win0_yrgb_mst);
	writel(0x01, &regs->reg_cfg_done); /* enable reg config */

	ret = reset_assert(dclk_rst);
	if (ret) {
		dev_warn(dev, "failed to assert dclk reset (ret=%d)\n", ret);
		return;
	}
	udelay(20);

	ret = reset_deassert(dclk_rst);
	if (ret)
		dev_warn(dev, "failed to deassert dclk reset (ret=%d)\n", ret);

}

static void rkvop_set_pin_polarity(struct udevice *dev,
				   enum vop_modes mode, u32 polarity)
{
	struct rkvop_driverdata *ops =
		(struct rkvop_driverdata *)dev_get_driver_data(dev);

	if (ops->set_pin_polarity)
		ops->set_pin_polarity(dev, mode, polarity);
}

static void rkvop_enable_output(struct udevice *dev, enum vop_modes mode)
{
	struct rk_vop_priv *priv = dev_get_priv(dev);
	struct rk3288_vop *regs = priv->regs;
	struct rkvop_driverdata *data =
		(struct rkvop_driverdata *)dev_get_driver_data(dev);
	/*
	 * Which SYS_CTRL bits count as "an output enable" is SoC-specific:
	 * RK3399 has a DisplayPort enable at bit 11 that rk3288/rk3328 do not
	 * (and whose meaning there is unknown), so the mask comes from the
	 * per-SoC driverdata.  A driverdata that leaves it 0 keeps exactly the
	 * pre-DP behaviour -- this must not silently start clearing an extra
	 * bit on rk3288/rk3328.
	 */
	u32 out_en_mask = data->out_en_mask ? data->out_en_mask : M_ALL_OUT_EN;

	/* remove from standby */
	clrbits_le32(&regs->sys_ctrl, V_STANDBY_EN(1));

	switch (mode) {
	case VOP_MODE_HDMI:
		clrsetbits_le32(&regs->sys_ctrl, out_en_mask,
				V_HDMI_OUT_EN(1));
		break;

	case VOP_MODE_EDP:
		clrsetbits_le32(&regs->sys_ctrl, out_en_mask,
				V_EDP_OUT_EN(1));
		break;

#if defined(CONFIG_ROCKCHIP_RK3288)
	case VOP_MODE_LVDS:
		clrsetbits_le32(&regs->sys_ctrl, out_en_mask,
				V_RGB_OUT_EN(1));
		break;
#endif

	case VOP_MODE_MIPI:
		clrsetbits_le32(&regs->sys_ctrl, out_en_mask,
				V_MIPI_OUT_EN(1));
		break;

	case VOP_MODE_DP:
		/*
		 * Without this the DP output enable is never set at all: the
		 * VOP leaves standby, scans out, and drives nothing towards
		 * the cdn-dp encoder -- a black screen with a perfectly happy
		 * link on the DP side.  Only RK3399 provides a driverdata with
		 * M_RK3399_DP_OUT_EN in out_en_mask, so on any other SoC this
		 * case would clear a bit it never set; guard on that rather
		 * than on a CONFIG symbol.
		 *
		 * NOT yet verified on hardware.
		 */
		if (!(out_en_mask & M_RK3399_DP_OUT_EN)) {
			debug("%s: DP output not supported on this VOP\n",
			      __func__);
			break;
		}
		clrsetbits_le32(&regs->sys_ctrl, out_en_mask,
				V_RK3399_DP_OUT_EN(1));
		break;

	default:
		debug("%s: unsupported output mode %x\n", __func__, mode);
	}
}

static void rkvop_mode_set(struct udevice *dev,
			   const struct display_timing *edid,
			   enum vop_modes mode)
{
	struct rk_vop_priv *priv = dev_get_priv(dev);
	struct rk3288_vop *regs = priv->regs;
	struct rk3288_vop *dsp_regs = priv->regs + priv->dsp_offset;
	struct rkvop_driverdata *data =
		(struct rkvop_driverdata *)dev_get_driver_data(dev);

	u32 hactive = edid->hactive.typ;
	u32 vactive = edid->vactive.typ;
	u32 hsync_len = edid->hsync_len.typ;
	u32 hback_porch = edid->hback_porch.typ;
	u32 vsync_len = edid->vsync_len.typ;
	u32 vback_porch = edid->vback_porch.typ;
	u32 hfront_porch = edid->hfront_porch.typ;
	u32 vfront_porch = edid->vfront_porch.typ;
	int mode_flags;
	u32 pin_polarity;

	/*
	 * Every other output wants the dclk inverted (Linux sets
	 * {rgb,edp,hdmi,mipi}_dclk_pol = 1 in vop_crtc_atomic_enable()), but
	 * DisplayPort is the one case where it sets dp_dclk_pol = 0.  The
	 * blanket invert here predates any DP support; leaving it set means DP
	 * pixel data is presented on the wrong dclk edge, which at a 533 MHz
	 * 3840x2160 pixel clock is a plausible cause of a black or badly
	 * corrupt screen.  Clear it for DP only -- other modes are untouched,
	 * so rk3288/rk3328 behaviour is bit-identical.
	 *
	 * NOT yet verified on hardware.
	 */
	if (mode == VOP_MODE_DP)
		pin_polarity = 0;
	else
		pin_polarity = BIT(DCLK_INVERT);
	if (edid->flags & DISPLAY_FLAGS_HSYNC_HIGH)
		pin_polarity |= BIT(HSYNC_POSITIVE);
	if (edid->flags & DISPLAY_FLAGS_VSYNC_HIGH)
		pin_polarity |= BIT(VSYNC_POSITIVE);

	rkvop_set_pin_polarity(dev, mode, pin_polarity);
	rkvop_enable_output(dev, mode);

	/*
	 * DisplayPort belongs in the 10-bit arm with HDMI and eDP, and leaving
	 * it out is a colour bug rather than a conservative choice.
	 *
	 * out_mode sizes the VOP -> encoder INTERNAL bus; the DP framer's own
	 * pixel depth is a separate thing that cdn-dp sets from the EDID. They
	 * are not required to agree, and Linux proves it: cdn_dp_bridge sets
	 * ROCKCHIP_OUT_MODE_AAAA unconditionally while running color_depth = 8
	 * on ordinary 8-bpc monitors. An earlier reading here had DP at P888 to
	 * "match the framer's 8 bpc"; that is not how the split works.
	 *
	 * What P888 actually does on this path: the framer samples the top 8
	 * bits of each 10-bit lane, so with out[29:24] left at zero red
	 * collapses to a quarter scale and green and blue each shift down a
	 * channel. Full white (0x00ffffff) reaches the panel as (0x3f,0xff,0xff)
	 * -- pale cyan. Measured on the board before this change: DSP_CTRL0 at
	 * 0xff900010 read 0x00000000, i.e. out_mode 0.
	 *
	 * Self-scoping: rk3399_lit_driverdata has no VOP_FEATURE_OUTPUT_10BIT,
	 * so a VOPL-driven DP still gets P888 -- which is exactly Linux's
	 * "no VOP_FEATURE_OUTPUT_RGB10 -> force P888" fallback. rk3288 and
	 * rk3328 carry no 10-bit feature bit and are untouched.
	 */
	mode_flags = 0;  /* RGB888 */
	if ((data->features & VOP_FEATURE_OUTPUT_10BIT) &&
	    (mode == VOP_MODE_HDMI || mode == VOP_MODE_EDP ||
	     mode == VOP_MODE_DP))
		mode_flags = 15;  /* RGBaaa */

	clrsetbits_le32(&regs->dsp_ctrl0, M_DSP_OUT_MODE,
			V_DSP_OUT_MODE(mode_flags));

	writel(V_HSYNC(hsync_len) |
	       V_HORPRD(hsync_len + hback_porch + hactive + hfront_porch),
			&dsp_regs->dsp_htotal_hs_end);

	writel(V_HEAP(hsync_len + hback_porch + hactive) |
	       V_HASP(hsync_len + hback_porch),
	       &dsp_regs->dsp_hact_st_end);

	writel(V_VSYNC(vsync_len) |
	       V_VERPRD(vsync_len + vback_porch + vactive + vfront_porch),
	       &dsp_regs->dsp_vtotal_vs_end);

	writel(V_VAEP(vsync_len + vback_porch + vactive)|
	       V_VASP(vsync_len + vback_porch),
	       &dsp_regs->dsp_vact_st_end);

	writel(V_HEAP(hsync_len + hback_porch + hactive) |
	       V_HASP(hsync_len + hback_porch),
	       &dsp_regs->post_dsp_hact_info);

	writel(V_VAEP(vsync_len + vback_porch + vactive)|
	       V_VASP(vsync_len + vback_porch),
	       &dsp_regs->post_dsp_vact_info);

	writel(0x01, &regs->reg_cfg_done); /* enable reg config */
}

/**
 * rk_vop_endpoint_disp() - Resolve a VOP endpoint to the encoder behind it
 *
 * The remote-endpoint references into a subnode of the encoder (HDMI, MIPI,
 * DP, ...) with the DTS looking something like the following (assume
 * 'hdmi_in_vopl' to be referenced):
 *
 * hdmi: hdmi@ff940000 {
 *   ports {
 *     hdmi_in: port {
 *       hdmi_in_vopb: endpoint@0 { ... };
 *       hdmi_in_vopl: endpoint@1 { ... };
 *     }
 *   }
 * }
 *
 * The original code had 3 steps of "walking the parent", but a much better
 * (as in: less likely to break if the DTS changes) way of doing this is to
 * "find the enclosing device of UCLASS_DISPLAY".
 *
 * This is split out of rk_display_init() so the ranking pre-pass can ask
 * "which encoder is on this port, and what does its sink want?" without
 * committing to driving it.
 *
 * @ep_node:		endpoint node within the VOP's 'port' list
 * @dispp:		returns the UCLASS_DISPLAY device behind @ep_node
 * @remote_vop_idp:	returns the encoder's 'reg' for this endpoint, i.e.
 *			which of the encoder's inputs this VOP is
 * Return: 0 if OK, -ve if something went wrong
 */
static int rk_vop_endpoint_disp(ofnode ep_node, struct udevice **dispp,
				int *remote_vop_idp)
{
	struct udevice *disp = NULL;
	u32 remote_phandle;
	ofnode remote;
	int ret;

	ret = ofnode_read_u32(ep_node, "remote-endpoint", &remote_phandle);
	if (ret)
		return ret;

	remote = ofnode_get_by_phandle(remote_phandle);
	if (!ofnode_valid(remote))
		return -EINVAL;
	*remote_vop_idp = ofnode_read_u32_default(remote, "reg", -1);

	while (ofnode_valid(remote)) {
		remote = ofnode_get_parent(remote);
		if (!ofnode_valid(remote))
			return -EINVAL;

		uclass_find_device_by_ofnode(UCLASS_DISPLAY, remote, &disp);
		if (disp)
			break;
	}
	if (!disp)
		return -EINVAL;

	*dispp = disp;

	return 0;
}

/*
 * How many VOPs on this SoC can scan out strictly more than @dev can.
 *
 * This is the whole of the assignment rule: the most capable VOP takes the
 * highest-resolution sink, the next takes the next, and so on.  A VOP that has
 * @n more capable siblings is therefore not entitled to the @n largest sinks --
 * they are spoken for -- and starts at rank @n of its own ranked candidate
 * list.  With RK3399's two VOPs this reduces to "VOPL (2560x1600) takes the
 * smaller monitor, VOPB (4096x2160) takes the larger", which is the point:
 * DT order decided it before, and DT order has nothing to do with capability.
 *
 * Deliberately does NOT probe the siblings -- uclass_find_* walks bound
 * devices only.  max_output_* is static per-compatible driver data, so it is
 * readable without probing, and a VOP that is never probed cannot claim
 * anything anyway.
 *
 * The compatible-string test is what makes reading another UCLASS_VIDEO
 * device's driver data as a rkvop_driverdata safe; a non-VOP video device
 * (and there can be one, e.g. a simple framebuffer) is skipped rather than
 * type-punned.
 */
static int rk_vop_more_capable_count(struct udevice *dev)
{
	struct rkvop_driverdata *self =
		(struct rkvop_driverdata *)dev_get_driver_data(dev);
	struct udevice *d;
	int count = 0;

	if (!self || !self->max_output_width)
		return 0;

	for (uclass_find_first_device(UCLASS_VIDEO, &d);
	     d;
	     uclass_find_next_device(&d)) {
		struct rkvop_driverdata *other;
		const char *compat;

		if (d == dev)
			continue;

		/*
		 * A sibling that has already been probed has already taken its
		 * output, so it is owed nothing and must not be reserved for.
		 *
		 * Without this the count is static and the reservation outlives
		 * the need for it: probe VOPB first -- which is what video0 =
		 * &vopb and `vop probe vop@ff900000` both do -- and it claims
		 * the 4K DP sink; VOPL then sees that encoder in_use, drops it
		 * to tier 3, still counts VOPB as more capable, and reserves
		 * away the 1080p HDMI monitor it could have driven. Nobody
		 * drives it, where plain DT order would have. Reserving is only
		 * correct against a sibling that can still act on it.
		 *
		 * device_active() is the right test because a VOP activates
		 * only on a successful probe: rk_vop_probe() seeds its result
		 * -ENODEV, so a VOP that drove nothing returns an error and is
		 * never marked active.
		 *
		 * This skip is about ENTITLEMENT only, and must not be read as
		 * "so a second VOP can now be lit". It cannot be: both DCLKs
		 * hang off one PLL (see rk_vop_probe_ranked()'s note), so the
		 * second VOP programmed drops the first. That invariant is
		 * enforced where it belongs, at the top of rk_vop_probe() --
		 * see rk_vop_find_lit(). Deriving it from a reservation count
		 * instead is what broke: with VOPB active on 4K DP the count
		 * here reaches 0, the last-resort pass then accepted a 1080p
		 * guess for VOPL, and retuning VPLL to 148.5 MHz took VOPB's
		 * pixel clock away.
		 */
		if (device_active(d))
			continue;

		/*
		 * Skip a device with no ofnode before asking for a property:
		 * under OF_LIVE that reaches of_find_property(NULL, ...).
		 */
		if (!ofnode_valid(dev_ofnode(d)))
			continue;

		compat = ofnode_get_property(dev_ofnode(d), "compatible", NULL);
		if (!compat || !strstr(compat, "vop"))
			continue;

		other = (struct rkvop_driverdata *)dev_get_driver_data(d);
		if (!other || !other->max_output_width)
			continue;

		if (other->max_output_width > self->max_output_width ||
		    (other->max_output_width == self->max_output_width &&
		     other->max_output_height > self->max_output_height))
			count++;
	}

	return count;
}

/**
 * rk_vop_find_lit() - is another VOP already scanning out a picture?
 *
 * The one hard constraint this driver has, and until now nothing enforced it:
 * a Rockchip SoC's VOPs share a single pixel PLL. clk_rk3399.c's
 * rockchip_vop_set_clk() points BOTH DCLK_VOP0 and DCLK_VOP1 at VPLL with
 * div == 1 and retunes VPLL itself to the requested pixel rate, so programming
 * a second VOP does not add a second display -- it moves the first one's
 * pixel clock out from under it and kills the link that was already up.
 * rk_vop_probe_ranked() has said so in a comment since it was written; what
 * was missing was anybody refusing to do it.
 *
 * The failure that motivated this: video0 = &vopb, so VOPB is probed first,
 * takes DP at 3840x2160 and retunes VPLL to 533 MHz. The splash path then
 * walks every bound video device (bmp_display(), and `vop probe` likewise) and
 * probes VOPL. VOPL's reservation count skips VOPB -- correctly, VOPB has
 * already taken its output -- so nothing was left to stop it, the last-resort
 * pass accepted the 1920x1080 fallback (inside VOPL's 2560x1600 ceiling), and
 * clk_set_rate(DCLK_VOP1, 148500000) dropped VOPB's 4K output.
 *
 * "Lit" is deliberately stricter than device_active(): a VOP whose probe
 * activated it but which configured no mode (xsize/ysize still 0) owns no
 * pixel clock and must not block anybody. That is the same test
 * rkvop_cmd_is_lit() applies, kept here so it is available outside CMD_DM.
 *
 * Like rk_vop_more_capable_count(), this walks BOUND devices only and screens
 * on the compatible string, so a non-VOP UCLASS_VIDEO device (a simple
 * framebuffer, say) is skipped rather than having its private data type-punned.
 *
 * @self: the VOP asking; never reported against itself
 * Return: the VOP that is scanning out, or NULL if none is
 */
static struct udevice *rk_vop_find_lit(struct udevice *self)
{
	struct udevice *d;

	for (uclass_find_first_device(UCLASS_VIDEO, &d);
	     d;
	     uclass_find_next_device(&d)) {
		struct video_priv *uc_priv;
		const char *compat;

		if (d == self || !device_active(d))
			continue;

		if (!ofnode_valid(dev_ofnode(d)))
			continue;

		compat = ofnode_get_property(dev_ofnode(d), "compatible", NULL);
		if (!compat || !strstr(compat, "vop"))
			continue;

		uc_priv = dev_get_uclass_priv(d);
		if (uc_priv && uc_priv->xsize && uc_priv->ysize)
			return d;
	}

	return NULL;
}

/*
 * CEA-861 video ID 16: 1920x1080p60, 148.5 MHz, positive syncs.
 *
 * The mode used when a sink is attached but its EDID cannot be read.  The
 * numbers are the CEA timings verbatim rather than a CVT/GTF computation,
 * because a fixed-function TV/monitor input is far likelier to lock a CEA mode
 * than a computed one, and there is no EDID here to tell us which it prefers.
 *
 * hdmi_monitor is deliberately left false: with no EDID we do not know the sink
 * speaks HDMI, so dw_hdmi_enable() configures DVI signalling -- the subset every
 * HDMI sink also accepts.  Claiming HDMI on a guess would add infoframes and
 * audio packets a DVI sink must ignore, for no gain on the one thing that
 * matters, which is that a picture appears.
 */
static const struct display_timing rk_vop_fallback_timing = {
	.pixelclock	= { 148500000, 148500000, 148500000 },
	.hactive	= { 1920, 1920, 1920 },
	.hfront_porch	= { 88, 88, 88 },
	.hback_porch	= { 148, 148, 148 },
	.hsync_len	= { 44, 44, 44 },
	.vactive	= { 1080, 1080, 1080 },
	.vfront_porch	= { 4, 4, 4 },
	.vback_porch	= { 36, 36, 36 },
	.vsync_len	= { 5, 5, 5 },
	.flags		= DISPLAY_FLAGS_HSYNC_HIGH | DISPLAY_FLAGS_VSYNC_HIGH,
	.hdmi_monitor	= false,
};

/**
 * rk_display_init() - Try to enable the given display device
 *
 * This function performs many steps:
 * - Finds the display device being referenced by @ep_node
 * - Puts the VOP's ID into its uclass platform data
 * - Probes the device to set it up
 * - Reads the EDID timing information
 * - Sets up the VOP clocks, etc. for the selected pixel clock and display mode
 * - Enables the display (the display device handles this and will do different
 *     things depending on the display type)
 * - Tells the uclass about the display resolution so that the console will
 *     appear correctly
 *
 * @dev:	VOP device that we want to connect to the display
 * @fbbase:	Frame buffer address
 * @ep_node:	Device tree node to process - this is the offset of an endpoint
 *		node within the VOP's 'port' list.
 * @allow_fallback: use the 1080p fallback timing when a sink is attached but
 *		its EDID cannot be read. Only the LAST-RESORT pass of the port
 *		walk sets this -- see rk_vop_probe_ranked().
 * Return: 0 if OK, -ve if something went wrong
 */
static int rk_display_init(struct udevice *dev, ulong fbbase, ofnode ep_node,
			   bool allow_fallback)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct rk_vop_priv *priv = dev_get_priv(dev);
	int vop_id, remote_vop_id;
	struct display_timing timing;
	struct udevice *disp;
	int ret;
	struct display_plat *disp_uc_plat;
	struct clk clk;
	enum video_log2_bpp l2bpp;
	const char *compat;
	struct reset_ctl dclk_rst;
	struct rkvop_driverdata *data;

	debug("%s(%s, 0x%lx, %s)\n", __func__,
	      dev_read_name(dev), fbbase, ofnode_get_name(ep_node));

	ret = rk_vop_endpoint_disp(ep_node, &disp, &remote_vop_id);
	if (ret) {
		debug("%s(%s): no UCLASS_DISPLAY for remote-endpoint\n",
		      __func__, dev_read_name(dev));
		return ret;
	}
	debug("remote vop_id=%d\n", remote_vop_id);

	compat = ofnode_get_property(dev_ofnode(disp), "compatible", NULL);
	if (!compat) {
		debug("%s(%s): Failed to find compatible property\n",
		      __func__, dev_read_name(dev));
		return -EINVAL;
	}
	if (strstr(compat, "edp") ||
	    strstr(compat, "rk3288-dp")) {
		vop_id = VOP_MODE_EDP;
	} else if (strstr(compat, "mipi")) {
		vop_id = VOP_MODE_MIPI;
	} else if (strstr(compat, "hdmi")) {
		vop_id = VOP_MODE_HDMI;
	} else if (strstr(compat, "cdn-dp")) {
		vop_id = VOP_MODE_DP;
	} else if (strstr(compat, "lvds")) {
		vop_id = VOP_MODE_LVDS;
	} else {
		debug("%s(%s): Failed to find vop mode for %s\n",
		      __func__, dev_read_name(dev), compat);
		return -EINVAL;
	}
	debug("vop_id=%d\n", vop_id);

	disp_uc_plat = dev_get_uclass_plat(disp);
	debug("Found device '%s', disp_uc_priv=%p\n", disp->name, disp_uc_plat);
	if (display_in_use(disp)) {
		vop_msg("%s: %s already claimed by another VOP\n",
			dev_read_name(dev), disp->name);
		return -EBUSY;
	}

	disp_uc_plat->source_id = remote_vop_id;
	disp_uc_plat->src_dev = dev;

	ret = device_probe(disp);
	if (ret) {
		vop_msg("%s: encoder %s won't probe (ret=%d)\n",
			dev_read_name(dev), disp->name, ret);
		return ret;
	}

	ret = display_read_timing(disp, &timing);
	if (ret) {
		/*
		 * The overwhelmingly common reason at boot: nothing is plugged
		 * in, or the sink has not finished powering up, so there is no
		 * EDID to read.  Naming the encoder matters because a VOP walks
		 * several of them and only one of these lines is about the
		 * output the operator is actually trying to bring up.
		 */
		vop_msg("%s: %s: no timing (no sink / EDID read failed, ret=%d)\n",
			dev_read_name(dev), disp->name, ret);

		/*
		 * "No EDID" and "no monitor" are different faults and only one
		 * of them justifies guessing a mode.  Ask the encoder's hotplug
		 * detect, which is the only signal that separates them.
		 *
		 * A sink that is present but unreadable is exactly the state
		 * this board's HDMI connector is in -- a real monitor whose DDC
		 * EEPROM NAKs -- and before this, that produced no output at
		 * all: not a wrong mode, nothing.  That is the worst possible
		 * outcome, because the splash, the boot menu and the failure
		 * text (see console_silent_defeat()) all become invisible for a
		 * fault that has nothing to do with any of them, and a board
		 * with a blank screen is indistinguishable from a dead one.
		 *
		 * 1080p60 is the fallback because it is the mode every digital
		 * sink made in the last twenty years accepts, and because the
		 * only job here is to be SEEN, not to be optimal.  A sink that
		 * cannot take it was never going to be driven correctly by a
		 * guess anyway.
		 *
		 * detect() returning 0 (nothing attached) or -ENOSYS (the
		 * encoder cannot tell) both keep the old behaviour: return the
		 * error and light nothing.  -ENOSYS deliberately does NOT
		 * enable the fallback -- an encoder with no HPD has given us no
		 * evidence, and "no evidence" must not become "assume a
		 * monitor", or every diskless port on every Rockchip board
		 * starts scanning out a picture nobody asked for.
		 *
		 * @allow_fallback is why this is not simply always on, and the
		 * reason is measured, not theoretical.  rk_vop_cand_cmp() ranks
		 * a present-but-unreadable sink ABOVE a known one deliberately
		 * (see the tier comment there), on the assumption that this
		 * function FAILS for it and the walk moves on.  Make the guess
		 * succeed unconditionally and that assumption inverts: on this
		 * board VOPB took the EDID-NAKing HDMI port at 1920x1080 and
		 * never reached DisplayPort, turning a working 3840x2160 output
		 * into a 1080p one.  So the guess is a LAST RESORT: every port
		 * is offered a real EDID first, and only a walk that enabled
		 * nothing at all comes back with this set.
		 */
		if (!allow_fallback || display_detect(disp) != 1)
			return ret;

		timing = rk_vop_fallback_timing;
		vop_msg("%s: %s: sink present but EDID unreadable -- falling back to %dx%d\n",
			dev_read_name(dev), disp->name,
			timing.hactive.typ, timing.vactive.typ);
	}

	/*
	 * Refuse a mode this VOP physically cannot scan out, mirroring Linux's
	 * vop_crtc_mode_valid() against vop_data.max_output.
	 *
	 * This matters because RK3399 has two VOPs of different capability
	 * (VOPB 4096x2160, VOPL 2560x1600) and DT can legitimately wire both
	 * to the same encoder: rk3399-base.dtsi lists vopl@ff8f0000 before
	 * vopb@ff900000 and there is no alias to order them, so VOPL is bound
	 * first and reaches the DP encoder first.  Before this check U-Boot
	 * had no max_output notion at all, so VOPL would happily accept a
	 * monitor's 3840x2160 EDID and program a mode it cannot produce --
	 * failing silently, with the walk never reaching VOPB.
	 *
	 * The check lives here, and specifically at this point in
	 * rk_display_init(), for three reasons:
	 *   - the timing is only known after display_read_timing(), so it
	 *     cannot go any earlier;
	 *   - it is before clk_set_rate() and rkvop_mode_set(), so a rejected
	 *     mode leaves no side effects: NPLL is not retuned and no VOP
	 *     register is written;
	 *   - it is before display_enable(), which is the only thing that sets
	 *     disp_uc_plat->in_use (display-uclass.c).  Bailing out here
	 *     therefore leaves the encoder unclaimed, so returning an error
	 *     makes rk_vop_probe()'s port walk move to VOPL's next port and,
	 *     more importantly, lets VOPB claim the same DP encoder when it is
	 *     probed afterwards.  Failing any later would mark the encoder in
	 *     use and permanently deny it to the VOP that can drive it.
	 *
	 * SoC impact: purely data-driven.  rk3288/rk3328 driverdata leave
	 * max_output_* at 0 and are not affected.
	 */
	data = (struct rkvop_driverdata *)dev_get_driver_data(dev);
	if ((data->max_output_width &&
	     timing.hactive.typ > data->max_output_width) ||
	    (data->max_output_height &&
	     timing.vactive.typ > data->max_output_height)) {
		vop_msg("%s: %s wants %dx%d, this VOP tops out at %dx%d\n",
			dev_read_name(dev), disp->name, timing.hactive.typ,
			timing.vactive.typ, data->max_output_width,
			data->max_output_height);
		return -ERANGE;
	}

	/*
	 * Set bitwidth for vop display according to vop mode.
	 *
	 * Computed HERE, ahead of clk_set_rate(), only because the framebuffer
	 * bound below needs it and that check must stay side-effect free for
	 * the same reasons the max_output check above does.  It depends on
	 * vop_id alone, which has been known since the top of this function.
	 */
	switch (vop_id) {
	case VOP_MODE_EDP:
#if defined(CONFIG_ROCKCHIP_RK3288)
	case VOP_MODE_LVDS:
#endif
		l2bpp = VIDEO_BPP16;
		break;
	case VOP_MODE_HDMI:
	case VOP_MODE_MIPI:
	/*
	 * DisplayPort used to land in the default case below and come out as
	 * VIDEO_BPP16, i.e. an RGB565 framebuffer on a link that was
	 * negotiated for 8bpc -- banded output at best.  It belongs with the
	 * other full-colour outputs.
	 */
	case VOP_MODE_DP:
		l2bpp = VIDEO_BPP32;
		break;
	default:
		l2bpp = VIDEO_BPP16;
	}

	/*
	 * Refuse a mode whose framebuffer will not fit the reservation.
	 *
	 * rk_vop_bind() sizes plat->size from CONFIG_VIDEO_ROCKCHIP_MAX_XRES x
	 * MAX_YRES, and rk_vop_probe() hands exactly that range to
	 * efi_add_memory_map() as EFI_RESERVED_MEMORY_TYPE.  The VOP's own
	 * max_output_* is an INDEPENDENT limit and on RK3399 the wrong one is
	 * larger: VOPB's max_output_width is 4096 while MAX_XRES is 3840, so a
	 * DCI-4K sink (4096x2160 preferred -- ordinary on cinema monitors and
	 * projectors) passes the check above and then makes rkvop_enable()
	 * program a 35,389,440-byte scanout over a 33,177,600-byte reservation.
	 * The VOP would DMA 2.1 MB past it, into memory this driver has already
	 * told the OS is free.
	 *
	 * The bound is the reservation, not a recomputed rectangle: EFI has
	 * been told about plat->size and nothing else, whatever the Kconfig
	 * dimensions were.  Refusing costs a mode nobody can display safely and
	 * lets the port walk try the next candidate; there is no correct way to
	 * scan out more pixels than were reserved.
	 */
	if ((u64)timing.hactive.typ * timing.vactive.typ * (1 << l2bpp) / 8 >
	    (u64)plat->size) {
		vop_msg("%s: %s wants %dx%d at %d bpp, over the %u-byte framebuffer reservation\n",
			dev_read_name(dev), disp->name, timing.hactive.typ,
			timing.vactive.typ, 1 << l2bpp, plat->size);
		return -ENOSPC;
	}

	ret = clk_get_by_index(dev, 1, &clk);
	if (!ret)
		ret = clk_set_rate(&clk, timing.pixelclock.typ);
	if (IS_ERR_VALUE(ret)) {
		debug("%s: Failed to set pixel clock: ret=%d\n", __func__, ret);
		return ret;
	}

	rkvop_mode_set(dev, &timing, vop_id);

	ret = reset_get_by_name(dev, "dclk", &dclk_rst);
	if (ret) {
		dev_err(dev, "failed to get dclk reset (ret=%d)\n", ret);
		return ret;
	}

	rkvop_enable(dev, fbbase, 1 << l2bpp, &timing, &dclk_rst);

	ret = display_enable(disp, 1 << l2bpp, &timing);
	if (ret) {
		/*
		 * This is the one failure worth shouting about: the VOP is now
		 * out of standby and scanning out (rkvop_enable() above ran),
		 * but the encoder refused to light the link.  Returning here
		 * without a word is how a board ends up with a configured VOP,
		 * a dark panel, and nothing on the console to say which side
		 * gave up -- the exact ambiguity the cdn-dp ops were written to
		 * avoid, so do not reintroduce it on this side.
		 */
		vop_msg("%s: %s (%s) refused to enable (ret=%d)\n",
			dev_read_name(dev), disp->name, vop_mode_name(vop_id),
			ret);
		return ret;
	}

	uc_priv->xsize = timing.hactive.typ;
	uc_priv->ysize = timing.vactive.typ;
	uc_priv->bpix = l2bpp;

	/* Attribution only -- see the comment on these fields in rk_vop.h. */
	priv->active_ep = ofnode_get_name(ep_node);
	priv->active_encoder = disp->name;
	priv->active_mode = vop_id;

	debug("fb=%lx, size=%d %d\n", fbbase, uc_priv->xsize, uc_priv->ysize);

	return 0;
}

void rk_vop_probe_regulators(struct udevice *dev,
			     const char * const *names, int cnt)
{
	int i, ret;
	const char *name;
	struct udevice *reg;

	for (i = 0; i < cnt; ++i) {
		name = names[i];
		debug("%s: probing regulator '%s'\n", dev->name, name);

		ret = regulator_autoset_by_name(name, &reg);
		if (!ret)
			ret = regulator_set_enable(reg, true);
	}
}

/*
 * A VOP has at most a handful of endpoints (RK3399's have five: eDP, MIPI,
 * HDMI, MIPI1, DP).  Anything past this is DROPPED, not merely left unranked --
 * there is no second pass over the remainder -- so the collection loop says so
 * out loud rather than letting an output silently never come up.
 */
#define RK_VOP_MAX_ENDPOINTS	8

/**
 * struct rk_vop_cand - one candidate output port, with its sink's size
 *
 * @ep:		the endpoint node
 * @w:		sink's active width, valid only when @known
 * @h:		sink's active height, valid only when @known
 * @present:	an encoder is behind this endpoint and it probed
 * @known:	the sink's timing was read (implies @present)
 */
struct rk_vop_cand {
	ofnode ep;
	u32 w, h;
	bool present;
	bool known;
};

/*
 * Sort key.  Three tiers, in this order:
 *
 *   1. an encoder is present but its EDID could not be read  (rank infinity)
 *   2. an encoder is present and its sink's size is known    (largest first)
 *   3. no encoder is there at all                            (last)
 *
 * Tier 1 above tier 2 looks backwards and is deliberate: an unreadable sink
 * might be anything, including the largest monitor attached, and the two
 * mistakes are not symmetric.  Handing an unknown sink to the big VOP and
 * having it fail costs one idle VOP for this boot -- a blank screen that was
 * blank anyway.  Handing it to the small VOP costs the ability to ever drive
 * it, because the small VOP will take it, discover it wants 4K, and by then
 * the big VOP has moved on.  On this board the DP sink's EDID is intermittent
 * (the AUX channel is marginal), so "present but unknown" is a live case on
 * every boot, not a theoretical one.
 *
 * Tier 3 is separate from tier 1 for the same reason in reverse.  An RK3399
 * VOP has five endpoints and most boards populate two, so the majority are
 * dangling phandles with no encoder behind them.  Letting those share the
 * infinity rank would fill the reserved slots with nothing, and the reserved
 * slots are exactly what protects the big VOP's claim -- the small VOP would
 * "reserve" a handful of absent encoders and then help itself to the real
 * monitor.  An endpoint that resolves to no device is not a candidate for
 * anybody and sorts to the bottom, where it costs a failed rk_display_init()
 * and nothing else.
 *
 * Return: <0 if @a sorts before @b, >0 if after.
 */
static int rk_vop_cand_cmp(const struct rk_vop_cand *a,
			   const struct rk_vop_cand *b)
{
	if (a->present != b->present)
		return a->present ? -1 : 1;
	if (a->known != b->known)
		return a->known ? 1 : -1;
	if (a->w != b->w)
		return a->w < b->w ? 1 : -1;
	if (a->h != b->h)
		return a->h < b->h ? 1 : -1;

	return 0;
}

/*
 * Ask an endpoint's sink how big it is, without committing to driving it.
 *
 * This is safe to call speculatively because display_enable() is the only
 * thing that sets disp_uc_plat->in_use (display-uclass.c), and we do not call
 * it.  A probe and an EDID read leave the encoder unclaimed, so a candidate we
 * rank and then decline stays available to the VOP that should have it.
 *
 * source_id/src_dev are set exactly as rk_display_init() sets them, because an
 * encoder is entitled to look at them during probe; whichever VOP finally
 * claims the encoder overwrites them before enabling it.
 */
static void rk_vop_peek_sink(struct udevice *dev, struct rk_vop_cand *cand)
{
	struct display_plat *disp_uc_plat;
	struct display_timing timing;
	struct udevice *disp;
	int remote_vop_id;

	cand->present = false;
	cand->known = false;
	cand->w = 0;
	cand->h = 0;

	if (rk_vop_endpoint_disp(cand->ep, &disp, &remote_vop_id))
		return;

	/* Already driven by another VOP -- not a candidate at all. */
	if (display_in_use(disp))
		return;

	disp_uc_plat = dev_get_uclass_plat(disp);
	disp_uc_plat->source_id = remote_vop_id;
	disp_uc_plat->src_dev = dev;

	if (device_probe(disp))
		return;

	cand->present = true;

	if (display_read_timing(disp, &timing))
		return;

	cand->w = timing.hactive.typ;
	cand->h = timing.vactive.typ;
	cand->known = true;
}

/**
 * rk_vop_probe_ranked() - walk this VOP's output ports largest sink first
 *
 * Assign the most capable VOP to the highest-resolution sink, the next most
 * capable to the next, and so on, instead of letting DT order decide.  DT
 * order is arbitrary with respect to capability: rk3399-base.dtsi lists
 * vopl@ff8f0000 before vopb@ff900000 with no alias to order them, and within
 * each VOP's port list HDMI (endpoint@2) precedes DP (endpoint@4).  So the
 * little VOP is offered every encoder first, and takes the first one it can
 * drive -- which, with two monitors both inside its 2560x1600 ceiling, is
 * whichever the DTS happened to mention earlier.
 *
 * The rule is implemented by RESERVATION rather than by negotiation: this VOP
 * skips the first rk_vop_more_capable_count() candidates, because those belong
 * to VOPs that can scan out more than it can.  No cross-probe state is needed
 * beyond what the driver model already tracks, and the VOPs may probe in any
 * order -- that last part is true only because the count excludes siblings that
 * have already been probed and so already hold an output; see the comment
 * there.
 *
 * There is deliberately NO fallback to the reserved candidates when the
 * entitled ones all fail.  A small VOP that grabs the only monitor because the
 * big VOP had not been probed yet would satisfy this boot and violate the
 * rule; leaving it alone lets the big VOP take it moments later.  A VOP with
 * more capable siblings and only one sink attached is therefore expected to
 * come up idle -- that is the rule working, not a failure.
 *
 * Note this selects WHICH single output each VOP drives.  It does not make two
 * of them run at once: clk_rk3399.c points both DCLK_VOP0 and DCLK_VOP1 at
 * VPLL and retunes VPLL to the pixel clock, so whichever VOP is programmed
 * second drops the first.  Making two work at once would be a change to the
 * dclk parent, not to this assignment; until then rk_vop_find_lit() refuses
 * the second one at rk_vop_probe() rather than letting it silently take the
 * first one's pixel clock.
 *
 * @dev:	the VOP
 * @port:	its 'port' subnode
 * @headless:	set true when the walk found no sink on any port at all (no
 *		EDID, no hotplug detect); left untouched otherwise
 * Return: 0 if an output was enabled, -ve otherwise
 */
static int rk_vop_probe_ranked(struct udevice *dev, ofnode port,
			       bool *headless)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct rk_vop_cand cand[RK_VOP_MAX_ENDPOINTS];
	int n = 0, reserved, skipped = 0, i, j, ret = -ENODEV;
	bool sink_seen = false;
	ofnode node;

	for (node = ofnode_first_subnode(port);
	     ofnode_valid(node);
	     node = dev_read_next_subnode(node)) {
		if (n == RK_VOP_MAX_ENDPOINTS) {
			/*
			 * Say so rather than dropping them quietly. There is no
			 * second pass over the unranked remainder, so an
			 * endpoint past this point is never offered to
			 * rk_display_init() at all -- if a future SoC has more
			 * ports than this, the symptom is an output that simply
			 * never comes up.
			 */
			vop_msg("%s: more than %d output ports; the rest are ignored\n",
				dev_read_name(dev), RK_VOP_MAX_ENDPOINTS);
			break;
		}
		cand[n].ep = node;
		rk_vop_peek_sink(dev, &cand[n]);
		if (cand[n].known)
			sink_seen = true;
		n++;
	}

	/* Insertion sort: n is at most RK_VOP_MAX_ENDPOINTS. */
	for (i = 1; i < n; i++) {
		struct rk_vop_cand key = cand[i];

		for (j = i - 1; j >= 0 && rk_vop_cand_cmp(&cand[j], &key) > 0;
		     j--)
			cand[j + 1] = cand[j];
		cand[j + 1] = key;
	}

	reserved = rk_vop_more_capable_count(dev);

	for (i = 0; i < n; i++) {
		const char *name = ofnode_get_name(cand[i].ep);

		/*
		 * Only a real encoder can consume a reservation.  Counting an
		 * absent one would spend the slot that protects the more
		 * capable VOP's claim on nothing at all.
		 */
		if (cand[i].present && reserved > 0) {
			reserved--;
			skipped++;
			if (cand[i].known)
				vop_msg("%s: port '%s' (%ux%u) reserved for a more capable VOP\n",
					dev_read_name(dev), name,
					cand[i].w, cand[i].h);
			else
				vop_msg("%s: port '%s' (size unknown) reserved for a more capable VOP\n",
					dev_read_name(dev), name);
			continue;
		}

		ret = rk_display_init(dev, plat->base, cand[i].ep, false);
		if (!ret)
			return 0;

		vop_msg("%s: port '%s' unusable (ret=%d)\n",
			dev_read_name(dev), name, ret);
	}

	/*
	 * Last resort: nothing above produced a picture from a real EDID, so
	 * re-offer the same candidates in the same order, this time letting
	 * rk_display_init() guess 1080p for a port that has a sink attached
	 * but would not tell us its timing.
	 *
	 * It is a SECOND pass rather than a flag on the first because the
	 * ranking deliberately puts an unreadable sink ahead of a readable
	 * one: guessing inside the first pass makes the unreadable port win
	 * outright and starves the readable one, which is exactly the
	 * regression this pass exists to avoid (measured on ROC-RK3399-PC:
	 * HDMI NAKs its EDID, DP reads 3840x2160, and a single-pass guess cost
	 * the 4K output).
	 *
	 * Reservations are re-applied identically, so a VOP that came up idle
	 * by rank stays idle by rank: this pass changes what counts as usable,
	 * never who is entitled to it.
	 */
	reserved = rk_vop_more_capable_count(dev);
	for (i = 0; i < n; i++) {
		const char *name = ofnode_get_name(cand[i].ep);

		if (cand[i].present && reserved > 0) {
			reserved--;
			continue;
		}

		/*
		 * An encoder that would not PROBE cannot be helped by a
		 * fallback timing: rk_display_init() returns at device_probe()
		 * and never reaches the timing logic, so this pass would run
		 * the identical code to the identical failure.  cand[].present
		 * is exactly that fact, recorded by rk_vop_peek_sink().
		 *
		 * It is not free to find out again.  rk_hdmi_probe() gates on
		 * dw_hdmi_phy_wait_for_hpd(), which polls for a full 300 ms
		 * before giving up, and driver model clears DM_FLAG_ACTIVATED
		 * on a failed probe -- so every re-attempt pays the wait again.
		 * On a board with nothing in the HDMI socket that is 300 ms per
		 * VOP per pass; dropping this one costs 600 ms of the 900 ms
		 * two VOPs used to spend proving the same absent monitor absent
		 * three times over.
		 *
		 * The peek and the first pass are deliberately kept.  Two
		 * attempts is a genuine retry window for an encoder that is
		 * still powering up; the third was pure duplication.
		 */
		if (!cand[i].present) {
			vop_msg("%s: port '%s' skipped -- its encoder did not probe, and a fallback mode cannot change that\n",
				dev_read_name(dev), name);
			continue;
		}

		/*
		 * Same argument one step further, for an encoder that probed
		 * but gave no timing in the peek AND in pass 1: the only thing
		 * this pass can add is the fallback mode, and rk_display_init()
		 * grants that only when display_detect() answers 1.  So ask
		 * that FIRST.  It is a single HPD read (or -ENOSYS), while the
		 * EDID read it used to precede is the expensive part -- on
		 * cdn-dp every .read_edid re-runs the firmware load and a
		 * ~1.2 s DisplayPort alt-mode request to the Type-C port
		 * manager, measured 1.42 s per call on ROC-RK3399-PC with
		 * nothing on the connector.  A sink that detect() cannot see
		 * gets no fallback, so re-reading its EDID here could only have
		 * served as a THIRD plain attempt, which the comment above
		 * already calls pure duplication.  A candidate that DID read a
		 * timing (known) failed pass 1 for some other reason and keeps
		 * the old path.
		 */
		if (!cand[i].known) {
			struct udevice *disp;
			int remote_vop_id;

			if (rk_vop_endpoint_disp(cand[i].ep, &disp,
						 &remote_vop_id) ||
			    display_detect(disp) != 1) {
				vop_msg("%s: port '%s' skipped -- no timing and no sink detected, so no fallback mode applies\n",
					dev_read_name(dev), name);
				continue;
			}
			sink_seen = true;
		}

		ret = rk_display_init(dev, plat->base, cand[i].ep, true);
		if (!ret)
			return 0;

		vop_msg("%s: port '%s' unusable even with a fallback mode (ret=%d)\n",
			dev_read_name(dev), name, ret);
	}

	/*
	 * Nothing on any port answered: no EDID from any encoder and no
	 * hotplug detect either.  Tell the caller, so it can decline to walk
	 * this VOP again this boot -- see rk_vop_probe().
	 */
	if (ret && !sink_seen && headless)
		*headless = true;

	/*
	 * Say so when this VOP came up idle because it GAVE something away,
	 * since "no output port could be enabled" on its own reads like a fault
	 * and this is the rule working as intended.
	 *
	 * The test is what we actually skipped, not what is left of the
	 * reservation budget. Testing `reserved > 0` was exactly backwards: it
	 * is what remains UNSPENT, so in the one-monitor case this message
	 * exists for -- one present encoder, one reservation, both consumed --
	 * it reaches 0 and the message stayed silent, while a VOP with no
	 * encoder present at all kept its budget and announced that "every
	 * encoder present is reserved" when none was.
	 */
	if (ret && skipped > 0)
		vop_msg("%s: idle by rank -- %d encoder(s) reserved for a more capable VOP\n",
			dev_read_name(dev), skipped);

	return ret;
}

/*
 * VOPs whose ranked walk found no sink on any port this boot.
 *
 * Driver model does not latch a failed probe, and the `vop` command's own
 * comment below records the assumption that this did not matter because
 * "U-Boot probes video exactly once, early in board_init_r()".  Measured on
 * ROC-RK3399-PC (bootstage, 2026-09-16), it probes it FOUR times: once from
 * stdio_add_devices(), twice from console_init_r() (the stdout and stderr
 * "vidconsole" searches each land on the video0 alias), and once more from
 * efi_gop_register() at the first bootflow scan.  Every one of those re-runs
 * the whole port walk, and with nothing attached the walk costs 4.9 s on VOPB
 * (three cdn-dp DisplayPort alt-mode requests at 1.42 s each, two dw_hdmi HPD
 * waits at 300 ms) and 2.05 s on VOPL -- 21.8 s of U-Boot init on a headless
 * board, 16.8 s of it this, plus 2.05 s inside `bootflow scan`.
 *
 * So remember, per VOP, that the walk came up empty, and answer -ENODEV to the
 * next probe instead of walking again.  The latch is deliberately NARROW: it is
 * set only when NO port produced an EDID and NO encoder's hotplug detect saw a
 * sink (rk_vop_probe_ranked()'s @headless).  A sink that was read but could
 * not be driven -- a link that would not train, a mode outside this VOP's
 * ceiling, an encoder that is reserved for a sibling -- leaves the latch
 * clear, so those keep every retry they have today.  What changes is only the
 * board with nothing plugged in, which is exactly the one that paid for the
 * retries and could never benefit from them.
 *
 * A display that is attached AFTER the first walk is the documented job of
 * the `vop` command, which clears the latch before it re-probes; it always was
 * -- the extra boot-time walks were never designed as a retry window and no
 * caller of theirs asked for one.
 *
 * Keyed on the udevice pointer because the driver's priv is freed with every
 * failed probe while the udevice itself lives from bind to unbind.  Two VOPs
 * per SoC today; the array has room for a few more.
 */
#define RK_VOP_MAX_VOPS	4
static struct udevice *rk_vop_headless[RK_VOP_MAX_VOPS];

static bool rk_vop_is_headless(struct udevice *dev)
{
	int i;

	for (i = 0; i < RK_VOP_MAX_VOPS; i++)
		if (rk_vop_headless[i] == dev)
			return true;

	return false;
}

static void rk_vop_set_headless(struct udevice *dev, bool headless)
{
	int i, free = -1;

	for (i = 0; i < RK_VOP_MAX_VOPS; i++) {
		if (rk_vop_headless[i] == dev) {
			if (!headless)
				rk_vop_headless[i] = NULL;
			return;
		}
		if (!rk_vop_headless[i] && free < 0)
			free = i;
	}
	if (headless && free >= 0)
		rk_vop_headless[free] = dev;
}

int rk_vop_probe(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct rk_vop_priv *priv = dev_get_priv(dev);
	struct rkvop_driverdata *ops =
		(struct rkvop_driverdata *)dev_get_driver_data(dev);
	struct udevice *lit;
	int ret = 0;
	ofnode port, node;
	struct reset_ctl ahb_rst;
	bool headless = false;

	/* Before relocation we don't need to do anything */
	if (!(gd->flags & GD_FLG_RELOC))
		return 0;

	/* Walked already this boot and found nothing attached -- see above. */
	if (rk_vop_is_headless(dev)) {
		vop_msg("%s: no sink found on any port earlier this boot -- not walking again (`vop probe` to retry)\n",
			dev_read_name(dev));
		return -ENODEV;
	}

	/*
	 * Refuse outright while a sibling VOP is scanning out. This is the
	 * shared-VPLL invariant, and it is checked HERE -- before the AHB
	 * reset, before the EFI reservation, before any encoder is touched --
	 * precisely because everything below it has side effects on hardware
	 * that is currently displaying a picture. Failing later would already
	 * have retuned the PLL.
	 *
	 * -EBUSY rather than 0: this VOP configured no output, and
	 * rk_vop_probe() returning success for a VOP that drove nothing is the
	 * lie the -ENODEV seeding below exists to prevent. Driver model does
	 * not latch a failed probe, so a later `vop probe` retries this after
	 * the picture is gone if that is ever what the operator wants.
	 *
	 * Cheap side benefit: on this board it also skips a second VOP's full
	 * port walk, and with it dw_hdmi_phy_wait_for_hpd()'s 300 ms poll.
	 */
	lit = rk_vop_find_lit(dev);
	if (lit) {
		vop_msg("%s: %s is already driving a display -- both DCLKs share one PLL, so lighting this one would drop it\n",
			dev_read_name(dev), lit->name);
		return -EBUSY;
	}

	ret = reset_get_by_name(dev, "ahb", &ahb_rst);
	if (ret) {
		dev_err(dev, "failed to get ahb reset (ret=%d)\n", ret);
		return ret;
	}

	ret = reset_assert(&ahb_rst);
	if (ret) {
		dev_err(dev, "failed to assert ahb reset (ret=%d)\n", ret);
		return ret;
	}
	udelay(20);

	ret = reset_deassert(&ahb_rst);
	if (ret) {
		dev_err(dev, "failed to deassert ahb reset (ret=%d)\n", ret);
		return ret;
	}

#if defined(CONFIG_EFI_LOADER)
	debug("Adding to EFI map %d @ %lx\n", plat->size, plat->base);
	efi_add_memory_map(plat->base, plat->size, EFI_RESERVED_MEMORY_TYPE);
#endif

	priv->regs = dev_read_addr_ptr(dev);
	priv->win_offset = ops->win_offset;
	priv->dsp_offset = ops->dsp_offset;

	/*
	 * Try the ports until we find one that works.  Which order that is
	 * depends on whether this SoC declares max_output_* -- see the ranked
	 * walk below.  Without it the order is DT order, which in practice
	 * tries EDP first if available, then HDMI.
	 *
	 * Note that rockchip_vop_set_clk() always uses NPLL as the source
	 * clock so it is currently not possible to use more than one display
	 * device simultaneously.
	 */
	port = dev_read_subnode(dev, "port");
	if (!ofnode_valid(port)) {
		vop_msg("%s: no 'port' subnode in DT\n", dev_read_name(dev));
		return -EINVAL;
	}

	/*
	 * Seed the result as a failure rather than inheriting the 0 left by the
	 * reset_deassert() above.  If 'port' has no endpoint subnodes the loop
	 * body never runs, and with ret still 0 this function used to return
	 * success for a VOP that had configured no output at all -- leaving
	 * video_post_probe() to build a console on a 0x0 framebuffer.  That is
	 * the same class of lie as an encoder .enable that returns 0 without a
	 * link, so it fails loudly instead.
	 */
	ret = -ENODEV;

	if (ops->max_output_width) {
		ret = rk_vop_probe_ranked(dev, port, &headless);
		if (ret && headless) {
			vop_msg("%s: no sink on any port -- later probes this boot are refused without a walk\n",
				dev_read_name(dev));
			rk_vop_set_headless(dev, true);
		}
	} else {
		for (node = ofnode_first_subnode(port);
		     ofnode_valid(node);
		     node = dev_read_next_subnode(node)) {
			/*
			 * No fallback on the unranked path.  It is used by the
			 * SoCs with no max_output_width (rk3288, rk3328), none
			 * of which is exercised here, and none of whose
			 * encoders implements .detect -- so enabling it would
			 * be inert today and an untested behaviour change the
			 * moment one did.  Wire .detect for such an SoC and
			 * give this loop the same two-pass shape as
			 * rk_vop_probe_ranked() when there is a board to test
			 * it on.
			 */
			ret = rk_display_init(dev, plat->base, node, false);
			if (!ret)
				break;
			vop_msg("%s: port '%s' unusable (ret=%d)\n",
				dev_read_name(dev), ofnode_get_name(node), ret);
		}
	}

	if (ret)
		vop_msg("%s: no output port could be enabled (last ret=%d)\n",
			dev_read_name(dev), ret);

	video_set_flush_dcache(dev, 1);

	return ret;
}

int rk_vop_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	plat->size = 4 * (CONFIG_VIDEO_ROCKCHIP_MAX_XRES *
			  CONFIG_VIDEO_ROCKCHIP_MAX_YRES);

	return 0;
}

/*
 * `vop` command -- probe a VOP on demand, after boot.
 *
 * Why this exists at all: U-Boot treats a display as an always-present
 * resource probed during board_init_r().  On a board whose only output is a
 * DisplayPort sink that is not ready for a long time after reset, every port
 * of every VOP legitimately fails at that moment, rk_vop_probe() returns an
 * error, and device_probe() clears DM_FLAG_ACTIVATED and frees the device.
 * The boot path does probe again -- console_init_r() and efi_gop_register()
 * each land on the VOP once more (see rk_vop_headless above) -- but those are
 * incidental lookups seconds apart, not a retry window anyone asked for, and
 * with the headless latch a VOP that found nothing at all is not walked by
 * them again.  So after boot the VOP stays bound-but-inactive for the rest of
 * the session even though the hardware became usable later.  Driver model
 * itself has no objection to a retry -- a failed probe is explicitly not
 * latched -- there is simply no caller.  This command is that caller, and it
 * clears the headless latch first so the walk really runs.
 *
 * It deliberately does NOT reimplement any part of the bring-up: it calls
 * device_probe() and therefore runs exactly the same regulator / AHB-reset /
 * port-walk path as boot.  A picture obtained this way is evidence about the
 * real path, not about a command-only shortcut.
 *
 * Guarded on CONFIG_CMD_DM only as a stand-in: this wants its own Kconfig
 * symbol, but Kconfig is outside the scope of this change.  CMD_DM is the
 * existing "driver-model poking commands are wanted on this build" switch, so
 * size-constrained boards that turn it off do not pay for this either.
 */
#if IS_ENABLED(CONFIG_CMD_DM)

/*
 * Only devices bound by one of the rk*_vop drivers.  The uclass may in
 * principle hold other video devices, and printing their dev_get_priv() as a
 * struct rk_vop_priv would be reading a foreign driver's private data.
 */
static bool rkvop_is_vop(struct udevice *dev)
{
	return dev->driver && strstr(dev->driver->name, "vop");
}

/* Print what an ACTIVE VOP is actually driving.  Callers check active first. */
static void rkvop_cmd_show_active(struct udevice *dev)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);
	struct rk_vop_priv *priv = dev_get_priv(dev);

	if (!priv->active_encoder || !uc_priv->xsize || !uc_priv->ysize) {
		/*
		 * Active, but with nothing recorded and/or a zero-size mode.
		 * Say so rather than dressing it up: a VOP in this state has
		 * not brought an output up, whatever its probe returned.
		 */
		printf("active but NO OUTPUT recorded (%ux%u) -- treat as failure",
		       uc_priv->xsize, uc_priv->ysize);
		return;
	}

	printf("%s via %s (%s), %ux%u, %u bpp, fb 0x%lx",
	       vop_mode_name(priv->active_mode), priv->active_encoder,
	       priv->active_ep, uc_priv->xsize, uc_priv->ysize,
	       1 << uc_priv->bpix, (ulong)plat->base);
}

/* True only if the device is active AND has a mode it can actually scan out. */
static bool rkvop_cmd_is_lit(struct udevice *dev)
{
	struct video_priv *uc_priv;
	struct rk_vop_priv *priv;

	if (!device_active(dev))
		return false;

	uc_priv = dev_get_uclass_priv(dev);
	priv = dev_get_priv(dev);

	return priv->active_encoder && uc_priv->xsize && uc_priv->ysize;
}

static bool rkvop_cmd_matches(struct udevice *dev, const char *sel)
{
	char *end;
	ulong seq;

	if (!sel)
		return true;

	if (!strcmp(dev->name, sel))
		return true;

	seq = simple_strtoul(sel, &end, 10);

	return *sel && !*end && dev_seq(dev) == (int)seq;
}

static int do_vop_list(void)
{
	struct udevice *dev;
	int found = 0;

	for (uclass_find_first_device(UCLASS_VIDEO, &dev);
	     dev;
	     uclass_find_next_device(&dev)) {
		if (!rkvop_is_vop(dev))
			continue;

		found++;
		printf("%d  %-16s %-8s ", dev_seq(dev), dev->name,
		       device_active(dev) ? "ACTIVE" : "inactive");
		if (device_active(dev))
			rkvop_cmd_show_active(dev);
		else
			printf("-");
		printf("\n");
	}

	if (!found) {
		printf("vop: no VOP devices bound\n");
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

static int do_vop_probe(const char *sel)
{
	struct udevice *dev;
	int targeted = 0, lit = 0;
	int ret;

	for (uclass_find_first_device(UCLASS_VIDEO, &dev);
	     dev;
	     uclass_find_next_device(&dev)) {
		if (!rkvop_is_vop(dev) || !rkvop_cmd_matches(dev, sel))
			continue;

		targeted++;

		if (device_active(dev)) {
			/*
			 * device_probe() short-circuits to 0 on an already
			 * active device, which would look like this command
			 * had just brought the output up.  Never claim that.
			 * There is no 'vop remove' counterpart on purpose: an
			 * active VOP owns a bound-and-probed vidconsole that
			 * may be registered in stdout, and tearing that down
			 * from a command is a far larger blast radius than the
			 * problem being solved here.
			 */
			printf("vop: %s: already active, NOT re-probed -- ",
			       dev->name);
			rkvop_cmd_show_active(dev);
			printf("\n");
			if (rkvop_cmd_is_lit(dev))
				lit++;
			continue;
		}

		printf("vop: %s: probing (walking output ports)...\n",
		       dev->name);

		/*
		 * Make the walk speak.  Cleared unconditionally afterwards so
		 * that a later boot-path probe (console re-init, say) is as
		 * quiet as it has always been.
		 */
		vop_cmd_verbose = true;
		rk_vop_set_headless(dev, false);
		ret = device_probe(dev);
		vop_cmd_verbose = false;

		if (ret) {
			printf("vop: %s: FAILED (ret=%d) -- no output enabled\n",
			       dev->name, ret);
			continue;
		}

		/*
		 * A zero return is not by itself the answer.  rk_vop_probe()
		 * only returns 0 after rk_display_init() ran display_enable()
		 * to completion, but this command's whole purpose is to make a
		 * failure impossible to mistake for a picture, so re-check the
		 * observable state instead of trusting the return code.
		 */
		printf("vop: %s: ", dev->name);
		rkvop_cmd_show_active(dev);
		printf("\n");

		if (rkvop_cmd_is_lit(dev)) {
			lit++;
		} else {
			printf("vop: %s: probe returned 0 but no output is configured\n",
			       dev->name);
		}
	}

	if (!targeted) {
		printf("vop: no VOP matches '%s'\n", sel ? sel : "*");
		return CMD_RET_FAILURE;
	}

	printf("vop: %d of %d VOP(s) driving an output\n", lit, targeted);

	return lit ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

static int do_vop(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc < 2)
		return CMD_RET_USAGE;

	if (!strcmp(argv[1], "list"))
		return argc == 2 ? do_vop_list() : CMD_RET_USAGE;

	if (!strcmp(argv[1], "probe")) {
		if (argc > 3)
			return CMD_RET_USAGE;
		return do_vop_probe(argc == 3 ? argv[2] : NULL);
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(vop, 3, 0, do_vop,
	   "Rockchip VOP display controllers",
	   "list                - list bound VOPs and what each is driving\n"
	   "vop probe [<vop>]       - probe a VOP now, re-walking its output\n"
	   "                          ports; <vop> is a device name or seq\n"
	   "                          number, default all.  Boot probes video0\n"
	   "                          only, once; use this to retry a sink that\n"
	   "                          was not ready then, or to probe another\n"
	   "                          VOP.");

#endif /* CONFIG_CMD_DM */
