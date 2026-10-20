/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2014 Google Inc.
 */

#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <linux/types.h>

struct udevice;
struct display_timing;

/**
 * Display uclass platform data for each device
 *
 * @source_id:	ID for the source of the display data, typically a video
 * controller
 * @src_dev:	Source device providing the video
 * @in_use:	Display is being used
 */
struct display_plat {
	int source_id;
	struct udevice *src_dev;
	bool in_use;
};

/**
 * display_read_edid() - Read edid from display
 *
 * @dev:	Device to read from
 * @buf:	Buffer to read into (should be EDID_SIZE bytes)
 * @buf_size:	Buffer size (should be EDID_SIZE)
 * Return number of bytes read, <= 0 for error
 */
int display_read_edid(struct udevice *dev, u8 *buf, int buf_size);

/**
 * display_read_timing() - Read timing information
 *
 * @dev:	Device to read from
 * Return: 0 if OK, -ve on error
 */
int display_read_timing(struct udevice *dev, struct display_timing *timing);

/**
 * display_port_enable() - Enable a display port device
 *
 * @dev:	Device to enable
 * @panel_bpp:	Number of bits per pixel for panel
 * @timing:	Display timings
 * Return: 0 if OK, -ve on error
 */
int display_enable(struct udevice *dev, int panel_bpp,
		   const struct display_timing *timing);

/**
 * display_in_use() - Check if a display is in use by any device
 *
 * Return: true if the device is in use (display_enable() has been called
 * successfully), else false
 */
bool display_in_use(struct udevice *dev);

/**
 * display_detect() - Is a sink physically attached to this output?
 *
 * This answers the question an EDID read CANNOT answer. A failed
 * display_read_timing() has two completely different causes -- nothing is
 * plugged in, or something is plugged in whose EDID we could not read (dead
 * DDC, a NAKing EEPROM, a corrupt block) -- and only the second one justifies
 * bringing the output up on a guessed mode. Hotplug detect is the signal that
 * separates them, and it lives in the encoder, which is why this is an op and
 * not something the VOP can work out for itself.
 *
 * @dev:	Display (encoder) device
 * Return: 1 if a sink is present, 0 if not, -ve on error -- including
 *	-ENOSYS when the driver has no way to tell, which callers must NOT
 *	read as "no sink"
 */
int display_detect(struct udevice *dev);

struct dm_display_ops {
	/**
	 * read_timing() - Read information directly
	 *
	 * @dev:	Device to read from
	 * @timing:	Display timings
	 * @return 0 if OK, -ve on error
	 */
	int (*read_timing)(struct udevice *dev, struct display_timing *timing);

	/**
	 * read_edid() - Read information from EDID
	 *
	 * @dev:	Device to read from
	 * @buf:	Buffer to read into (should be EDID_SIZE bytes)
	 * @buf_size:	Buffer size (should be EDID_SIZE)
	 * @return number of bytes read, <=0 for error
	 */
	int (*read_edid)(struct udevice *dev, u8 *buf, int buf_size);

	/**
	 * enable() - Enable the display port device
	 *
	 * @dev:	Device to enable
	 * @panel_bpp:	Number of bits per pixel for panel
	 * @timing:	Display timings
	 * @return 0 if OK, -ve on error
	 */
	int (*enable)(struct udevice *dev, int panel_bpp,
		      const struct display_timing *timing);

	/**
	 * mode_valid() - Check if mode is supported
	 *
	 * @dev:	Device to enable
	 * @timing:	Display timings
	 * @return true if supported, false if not
	 */
	bool (*mode_valid)(struct udevice *dev,
			   const struct display_timing *timing);

	/**
	 * detect() - Report whether a sink is physically attached
	 *
	 * Optional. Implement it wherever the encoder has a hotplug-detect
	 * signal; leaving it NULL means "cannot tell", which is distinct from
	 * "nothing attached" and is reported as -ENOSYS.
	 *
	 * @dev:	Device to query
	 * @return 1 if a sink is present, 0 if not, -ve on error
	 */
	int (*detect)(struct udevice *dev);
};

#define display_get_ops(dev)	((struct dm_display_ops *)(dev)->driver->ops)

#endif
