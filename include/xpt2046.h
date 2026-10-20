/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * XPT2046 / ADS7846 resistive touch controller -- shared sampling and
 * zone-decode helpers.
 *
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * Two consumers share this: cmd/touchtest.c (the diagnostic, which prints what
 * a touch WOULD emit) and drivers/input/touch_kbd.c (the keyboard driver, which
 * actually emits it). They must agree exactly -- the five-zone mapping below is
 * the one a human verified on an MHS3528 hat, so it lives in one place rather
 * than being written twice and drifting.
 *
 * The control-byte sequence is taken verbatim from the userspace reader
 * amlogic/software/tests/spi/xpt2046-read.c, which is validated against this
 * exact hardware, rather than re-derived from the datasheet.
 */

#ifndef __XPT2046_H
#define __XPT2046_H

#include <errno.h>
#include <spi.h>
/*
 * The dm/ sub-headers rather than <dm.h>: including the wrapper from a header
 * trips its own "Suspect dm.h is included from a header file" warning.
 */
#include <dm/device.h>
#include <dm/ofnode.h>
#include <dm/read.h>
#include <dm/uclass.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/types.h>

/*
 * Control bytes: S=1, A2:0 = channel, MODE=0 (12-bit), SER/DFR=0,
 * PD1:0 = 00 (power down between conversions, PENIRQ enabled -- which is what
 * keeps the pendown GPIO meaningful).
 */
#define XPT_CMD_X		0xD0	/* A=101, X position */
#define XPT_CMD_Y		0x90	/* A=001, Y position */
#define XPT_CMD_Z1		0xB0	/* A=011, Z1 (touch pressure) */
#define XPT_CMD_Z2		0xC0	/* A=100, Z2 (touch pressure) */

#define XPT_ADC_MAX		4095

/*
 * Panel geometry, from the spi-cc-2cs-mhs3528 overlay's touchscreen@1 node.
 * Used ONLY to normalise raw ADC into thirds for zone detection -- this is not
 * a calibration and makes no claim about absolute coordinates.
 */
#define XPT_MIN_X		180
#define XPT_SIZE_X		3880
#define XPT_MIN_Y		240
#define XPT_SIZE_Y		3940
#define XPT_XPLATE_OHMS		60	/* ti,x-plate-ohms */

#define XPT_NORM_FULL		1000	/* normalised axis range */
#define XPT_ZONE_LOW		(XPT_NORM_FULL / 3)
#define XPT_ZONE_HIGH		(XPT_NORM_FULL - XPT_NORM_FULL / 3)

enum xpt_zone {
	XPT_ZONE_NONE = 0,
	XPT_ZONE_CENTRE,
	XPT_ZONE_TOP,
	XPT_ZONE_BOTTOM,
	XPT_ZONE_LEFT,
	XPT_ZONE_RIGHT,
	XPT_ZONE_COUNT
};

/**
 * struct xpt_zone_desc - what a zone is called, emits and looks like
 *
 * @name:	zone name, as printed by touchtest
 * @key:	name of the key this zone stands for
 * @keycode:	Linux keycode the keyboard driver emits for this zone (0 = none)
 * @r: @g: @b:	colour touchtest paints the panel when this zone is hit
 */
struct xpt_zone_desc {
	const char *name;
	const char *key;
	int keycode;
	u8 r, g, b;
};

/*
 * The keycodes are ordinary Linux keycodes, which is what makes this work with
 * no change to the bootmenu: drivers/input/input.c already turns KEY_UP/DOWN/
 * LEFT/RIGHT into the ECMA-48 escape sequences the CLI decodes (\e[A etc), and
 * KEY_ENTER into '\r' through kbd_plain_xlate[].
 */
static const struct xpt_zone_desc xpt_zones[XPT_ZONE_COUNT] = {
	[XPT_ZONE_NONE]   = { "none",   "-",     0,         0x00, 0x00, 0x00 },
	[XPT_ZONE_CENTRE] = { "centre", "enter", KEY_ENTER, 0xff, 0xff, 0x00 },
	[XPT_ZONE_TOP]    = { "top",    "up",    KEY_UP,    0x00, 0xff, 0x00 },
	[XPT_ZONE_BOTTOM] = { "bottom", "down",  KEY_DOWN,  0x00, 0x00, 0xff },
	[XPT_ZONE_LEFT]   = { "left",   "left",  KEY_LEFT,  0xff, 0x00, 0x00 },
	[XPT_ZONE_RIGHT]  = { "right",  "right", KEY_RIGHT, 0xff, 0x00, 0xff },
};

/*
 * One 12-bit conversion: 3 bytes (command + two result bytes), which is short
 * enough to take the driver's PIO path. The result is left-aligned in the two
 * bytes FOLLOWING the command byte.
 */
static inline int xpt_read(struct spi_slave *slave, u8 cmd, unsigned int *out)
{
	u8 tx[3] = { cmd, 0x00, 0x00 };
	u8 rx[3] = { 0x00, 0x00, 0x00 };
	int ret;

	ret = spi_xfer(slave, sizeof(tx) * 8, tx, rx,
		       SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret < 0)
		return ret;

	*out = (((unsigned int)rx[1] << 8) | rx[2]) >> 3;

	return 0;
}

/**
 * xpt_sample() - claim the bus, read X/Y/Z1/Z2, release it
 *
 * The panel and the touch controller share the SPI controller, and the driver
 * rejects a re-entrant claim with -EBUSY (the display's cyclic sync, reached
 * from video_idle() inside schedule(), can be holding it). -EBUSY is NOT a
 * failure: the caller should skip this sample and try again.
 *
 * Return: 0 if OK, -EBUSY if another slave holds the bus, other -ve on error
 */
static inline int xpt_sample(struct spi_slave *slave, unsigned int *x,
			     unsigned int *y, unsigned int *z1,
			     unsigned int *z2)
{
	int ret;

	ret = spi_claim_bus(slave);
	if (ret)
		return ret;

	ret = xpt_read(slave, XPT_CMD_X, x);
	if (!ret)
		ret = xpt_read(slave, XPT_CMD_Y, y);
	if (!ret)
		ret = xpt_read(slave, XPT_CMD_Z1, z1);
	if (!ret)
		ret = xpt_read(slave, XPT_CMD_Z2, z2);

	spi_release_bus(slave);

	return ret;
}

static inline unsigned int xpt_norm(unsigned int raw, unsigned int min,
				    unsigned int size)
{
	if (raw <= min)
		return 0;
	raw -= min;
	if (raw >= size)
		return XPT_NORM_FULL;

	return raw * XPT_NORM_FULL / size;
}

/*
 * Raw ADC -> screen-oriented normalised coordinates, applying the same two
 * transforms the DT asks Linux for: touchscreen-inverted-x, then
 * touchscreen-swapped-x-y (Linux applies inversion before the swap in
 * touchscreen_report_pos()). NOT calibration -- the zone thresholds are
 * thirds of the raw span.
 */
static inline void xpt_to_screen(unsigned int rawx, unsigned int rawy,
				 unsigned int *sx, unsigned int *sy)
{
	unsigned int nx = xpt_norm(rawx, XPT_MIN_X, XPT_SIZE_X);
	unsigned int ny = xpt_norm(rawy, XPT_MIN_Y, XPT_SIZE_Y);

	nx = XPT_NORM_FULL - nx;	/* touchscreen-inverted-x */

	*sx = ny;			/* touchscreen-swapped-x-y */
	*sy = nx;
}

static inline enum xpt_zone xpt_zone_of(unsigned int sx, unsigned int sy)
{
	unsigned int dx = sx > XPT_NORM_FULL / 2 ?
			  sx - XPT_NORM_FULL / 2 : XPT_NORM_FULL / 2 - sx;
	unsigned int dy = sy > XPT_NORM_FULL / 2 ?
			  sy - XPT_NORM_FULL / 2 : XPT_NORM_FULL / 2 - sy;

	if (sx >= XPT_ZONE_LOW && sx <= XPT_ZONE_HIGH &&
	    sy >= XPT_ZONE_LOW && sy <= XPT_ZONE_HIGH)
		return XPT_ZONE_CENTRE;

	/* Outside the middle square: the axis furthest from centre wins */
	if (dx >= dy)
		return sx < XPT_NORM_FULL / 2 ? XPT_ZONE_LEFT : XPT_ZONE_RIGHT;

	return sy < XPT_NORM_FULL / 2 ? XPT_ZONE_TOP : XPT_ZONE_BOTTOM;
}

/*
 * ADS7846 pressure: Rtouch = Rx_plate * (X / 4096) * (Z2 / Z1 - 1), in ohms.
 * Lower is a harder press. Z1 == 0 means no contact (or no reply).
 */
static inline unsigned int xpt_pressure(unsigned int x, unsigned int z1,
					unsigned int z2)
{
	if (!z1 || z2 < z1)
		return 0;

	return (unsigned int)div_u64((u64)XPT_XPLATE_OHMS * x * (z2 - z1),
				     (u64)z1 * (XPT_ADC_MAX + 1));
}

/**
 * xpt_arm() - make the pendown GPIO meaningful, by taking one conversion
 *
 * PENIRQ is an OUTPUT OF A STATE the controller only enters as a RESULT of a
 * conversion. Every control byte above carries PD1:0 = 00, so each conversion
 * ends by returning the controller to power-down with PENIRQ enabled -- but
 * until at least one has been issued, that precondition has never been
 * established, the PENIRQ switch stays open, and the line sits at whatever the
 * board's external pull-up says. Which is "pen up", forever, however hard the
 * panel is pressed.
 *
 * That is a deadlock for any consumer that gates its SPI access on the pendown
 * GPIO -- and both consumers here do, deliberately, because an SPI transaction
 * per console poll is unaffordable: no conversion until a pen-down, and no
 * pen-down until a conversion. Such a consumer MUST arm the controller once,
 * up front, on a path that does not consult the pen.
 *
 * The sampled values are discarded: with no finger down they are the rails,
 * and the point is the state transition, not the numbers.
 *
 * Return: 0 if OK, -EBUSY if another slave holds the bus, other -ve on error
 */
static inline int xpt_arm(struct spi_slave *slave)
{
	unsigned int x, y, z1, z2;

	return xpt_sample(slave, &x, &y, &z1, &z2);
}

/**
 * xpt_find() - locate the bound XPT2046 device, whoever claimed it
 *
 * The DT node ("ti,ads7846") is claimed by exactly one driver: the touch
 * keyboard (UCLASS_KEYBOARD) when CONFIG_TOUCH_KEYBOARD is on, otherwise
 * touchtest's do-nothing SPI_GENERIC stub. Look it up by compatible so both
 * consumers work in either configuration.
 *
 * Return: 0 if found, -ENODEV if there is no such node (overlay not applied)
 */
static inline int xpt_find(struct udevice **devp)
{
	static const enum uclass_id ids[] = {
		UCLASS_KEYBOARD, UCLASS_SPI_GENERIC,
	};
	struct udevice *dev;
	int i;

	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		uclass_foreach_dev_probe(ids[i], dev) {
			ofnode node = dev_ofnode(dev);

			if (!ofnode_valid(node))
				continue;
			if (ofnode_device_is_compatible(node, "ti,ads7846") ||
			    ofnode_device_is_compatible(node, "ti,tsc2046")) {
				*devp = dev;
				return 0;
			}
		}
	}

	return -ENODEV;
}

#endif /* __XPT2046_H */
