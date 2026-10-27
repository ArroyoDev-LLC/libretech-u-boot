/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2017 Theobroma Systems Design und Consulting GmbH
 */

#ifndef __RK_VOP_H__
#define __RK_VOP_H__

#include <asm/arch-rockchip/vop_rk3288.h>

struct rk_vop_priv {
	void *grf;
	void *regs;
	int win_offset;
	int dsp_offset;
	/*
	 * Attribution only: which output actually came up, recorded by
	 * rk_display_init() the moment display_enable() returns 0 and read back
	 * by the `vop` command.  Nothing in the display path consults these.
	 *
	 * They exist because "the screen is on" is not by itself an answer when
	 * a VOP has four ports and a board has two VOPs: without them a human
	 * cannot tell whether the picture came from the port they were trying
	 * to bring up or from an unrelated one that happened to work.  Both
	 * pointers are into the live FDT / a bound udevice's name and so stay
	 * valid for as long as the device is active.
	 */
	const char *active_ep;
	const char *active_encoder;
	enum vop_modes active_mode;
};

enum vop_features {
	VOP_FEATURE_OUTPUT_10BIT = (1 << 0),
};

struct rkvop_driverdata {
	int win_offset;
	int dsp_offset;
	/* configuration */
	u32 features;
	/*
	 * SYS_CTRL bits that select which single output the VOP drives, i.e.
	 * the mask rkvop_enable_output() clears before setting the one it
	 * wants.  Leave at 0 to get M_ALL_OUT_EN (MIPI/eDP/HDMI/RGB); RK3399
	 * overrides it with M_RK3399_ALL_OUT_EN so the DP enable at bit 11 is
	 * also cleared when another output is selected.  It is a per-SoC datum
	 * rather than a wider shared macro because bit 11's function outside
	 * RK3399 is unknown -- see vop_rk3288.h.
	 */
	u32 out_en_mask;
	/*
	 * Largest display the VOP's internal path can actually scan out,
	 * mirroring Linux's vop_data.max_output (rockchip_vop_reg.c).  On
	 * RK3399 the two VOPs differ: VOPB does 4096x2160, VOPL only
	 * 2560x1600, yet both can be wired to the same encoder in DT.  With
	 * these zero no limit is enforced (previous behaviour, kept for the
	 * SoCs whose figures we have not confirmed).
	 */
	u32 max_output_width;
	u32 max_output_height;
	/* block-specific setters/getters */
	void (*set_pin_polarity)(struct udevice *, enum vop_modes, u32);
};

/**
 * rk_vop_probe() - common probe implementation
 *
 * Performs the rk_display_init on each port-subnode until finding a
 * working port (or returning an error if none of the ports could be
 * successfully initialised).
 *
 * @dev:	device
 * Return: 0 if OK, -ve if something went wrong
 */
int rk_vop_probe(struct udevice *dev);

/**
 * rk_vop_bind() - common bind implementation
 *
 * Sets the plat->size field to the amount of memory to be reserved for
 * the framebuffer: this is always
 *     (32 BPP) x VIDEO_ROCKCHIP_MAX_XRES x VIDEO_ROCKCHIP_MAX_YRES
 *
 * @dev:	device
 * Return: 0 (always OK)
 */
int rk_vop_bind(struct udevice *dev);

/**
 * rk_vop_remove() - put the VOP in standby, so it stops fetching
 *
 * For drivers flagged DM_FLAG_OS_PREPARE: removed before the OS starts.
 *
 * @dev:	device
 * Return: 0 (always OK)
 */
int rk_vop_remove(struct udevice *dev);

/**
 * rk_vop_probe_regulators() - probe (autoset + enable) regulators
 *
 * Probes a list of regulators by performing autoset and enable
 * operations on them.  The list of regulators is an array of string
 * pointers and any individual regulator-probe may fail without
 * counting as an error.
 *
 * @dev:	device
 * @names:	array of string-pointers to regulator names to probe
 * @cnt:	number of elements in the 'names' array
 */
void rk_vop_probe_regulators(struct udevice *dev,
			     const char * const *names, int cnt);

#endif
