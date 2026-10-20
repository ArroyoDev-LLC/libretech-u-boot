// SPDX-License-Identifier: GPL-2.0+
/*
 * touchtest -- XPT2046 / ADS7846 resistive touch panel test
 *
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * Companion to vidtest (cmd/video.c): where vidtest proves the display half of
 * an SPI TFT hat, this proves the touch half, on the SAME bus. On the MHS3528
 * hat the ILI9486 panel is CS0 and the XPT2046 touch controller is CS1, so a
 * run that samples touch AND paints the result also exercises per-slave chip
 * select, per-slave speed (41.67 MHz panel vs 1.3 MHz touch) and per-slave word
 * length -- a regression in any of those shows up here as garbage samples or a
 * corrupted panel rather than silently.
 *
 * The sampling and the five-zone decode live in <xpt2046.h>, shared with
 * drivers/input/touch_kbd.c: what this command PRINTS a touch would emit and
 * what that driver actually emits must be the same thing, and the zone mapping
 * has been verified by a human against this hardware.
 */

#include <command.h>
#include <console.h>
#include <dm.h>
#include <errno.h>
#include <spi.h>
#include <time.h>
#include <video.h>
#include <video_console.h>
#include <vsprintf.h>
#include <xpt2046.h>
#include <asm/gpio.h>
#include <dm/device-internal.h>
#include <linux/delay.h>
#include <linux/string.h>

#define TOUCH_POLL_MS		20	/* pen-up poll: GPIO only, no SPI */
#define TOUCH_SAMPLE_MS		60	/* pen-down: rate-limit the SPI + print */

#if !IS_ENABLED(CONFIG_TOUCH_KEYBOARD)
/*
 * The DT node carries compatible "ti,ads7846" and no U-Boot driver claimed it,
 * so the SPI uclass never bound a child for CS1 and the slave was unreachable.
 * Bind a driver with no behaviour of its own: it exists purely so the node
 * becomes a udevice, which gives this command the slave (speed, mode, cs) and
 * the pendown-gpio phandle straight out of DT instead of hardcoding them.
 *
 * When CONFIG_TOUCH_KEYBOARD is on, drivers/input/touch_kbd.c claims the same
 * node as a real keyboard, so this stub steps aside: exactly one driver must
 * match the compatible, and xpt_find() locates the device either way.
 */
static const struct udevice_id xpt2046_ids[] = {
	{ .compatible = "ti,ads7846" },
	{ .compatible = "ti,tsc2046" },
	{ }
};

U_BOOT_DRIVER(xpt2046_touch) = {
	.name		= "xpt2046_touch",
	.id		= UCLASS_SPI_GENERIC,
	.of_match	= xpt2046_ids,
};
#endif

static u32 touch_colour(struct video_priv *priv, enum xpt_zone z)
{
	u8 r = xpt_zones[z].r, g = xpt_zones[z].g, b = xpt_zones[z].b;

	switch (priv->bpix) {
	case VIDEO_BPP16:
		return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
	case VIDEO_BPP32:
		return ((u32)r << 16) | ((u32)g << 8) | b;
	default:
		return (r | g | b) ? ~0U : 0;
	}
}

/*
 * Paint the whole panel in the zone's colour and label it. Called ONLY on a
 * zone change: a full-screen push costs ~65 ms on this hat, and doing it per
 * sample would make the poll rate the display's rather than the touch's.
 *
 * Never called while the touch claim is held -- the panel and the touch
 * controller share the controller, and the driver rejects a re-entrant claim
 * with -EBUSY.
 */
static void touch_paint(struct udevice *vid, struct udevice *vc,
			enum xpt_zone z, const char *line2)
{
	struct video_priv *priv;

	if (!vid)
		return;

	priv = dev_get_uclass_priv(vid);
	if (video_fill(vid, touch_colour(priv, z)))
		return;

	if (vc) {
		vidconsole_position_cursor(vc, 0, 0);
		vidconsole_put_string(vc, xpt_zones[z].name);
		if (line2) {
			vidconsole_position_cursor(vc, 0, 1);
			vidconsole_put_string(vc, line2);
		}
	}
	video_sync(vid, true);
}

static int do_touchtest(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	unsigned int secs = 30;
	bool force = false, have_pen = false, was_down = false;
	unsigned int samples = 0, busy = 0, pendowns = 0;
	unsigned int xmin = ~0U, xmax = 0, ymin = ~0U, ymax = 0;
	unsigned int same = 0, railed_hi = 0, railed_lo = 0;
	enum xpt_zone shown = XPT_ZONE_NONE;
	struct udevice *dev, *vid = NULL, *vc = NULL;
	struct gpio_desc pen;
	struct spi_slave *slave;
	ulong start, last = 0;
	int ret;

	if (argc > 3)
		return CMD_RET_USAGE;
	if (argc >= 2) {
		secs = dectoul(argv[1], NULL);
		if (!secs)
			return CMD_RET_USAGE;
	}
	if (argc == 3) {
		if (strcmp(argv[2], "force"))
			return CMD_RET_USAGE;
		force = true;
	}

	ret = xpt_find(&dev);
	if (ret) {
		printf("touchtest: no ti,ads7846 node (%d) -- is the overlay (spi-cc-2cs + spi-cc-2cs-mhs3528) applied?\n",
		       ret);
		return CMD_RET_FAILURE;
	}
	slave = dev_get_parent_priv(dev);

	/*
	 * The touch controller is 8-bit; the panel on the other chip select
	 * uses 16 (and 64 for pixels). Word length is stored per slave, so ask
	 * explicitly rather than inheriting whatever ran last.
	 */
	ret = dm_spi_set_wordlen(dev, 8);
	if (ret && ret != -ENOSYS) {
		printf("touchtest: cannot set 8-bit words: %d\n", ret);
		return CMD_RET_FAILURE;
	}

	/*
	 * With CONFIG_TOUCH_KEYBOARD the keyboard driver owns this GPIO for the
	 * lifetime of the device, so the request is refused and this falls back
	 * to forced sampling. Use 'touchkbd watch' for a pen-driven zone report
	 * in that configuration -- it reads the pen through the owner.
	 */
	ret = gpio_request_by_name(dev, "pendown-gpio", 0, &pen, GPIOD_IS_IN);
	if (ret) {
		printf("touchtest: no usable pendown-gpio (%d) -- sampling every poll%s\n",
		       ret, IS_ENABLED(CONFIG_TOUCH_KEYBOARD) ?
		       " (the touch keyboard owns it; try 'touchkbd watch')" : "");
		force = true;
	} else {
		have_pen = true;
	}

	/*
	 * Arm PENIRQ before a pen-gated run. Without a conversion the pendown
	 * GPIO reads "up" forever (see xpt_arm()), so a run that waits for
	 * pen-down before it will ever sample waits for something it has itself
	 * made impossible. 'force' does not need this -- it samples regardless,
	 * and its first sample is the arming one -- but doing it up front costs
	 * one conversion and makes the two modes behave the same.
	 */
	ret = xpt_arm(slave);
	if (ret)
		printf("touchtest: could not arm PENIRQ (%d) -- pen-down may not be seen\n",
		       ret);

	uclass_first_device(UCLASS_VIDEO, &vid);
	uclass_first_device(UCLASS_VIDEO_CONSOLE, &vc);

	printf("touchtest: XPT2046 on %s, %u Hz, %u s%s\n",
	       dev->name, slave->max_hz, secs, force ? " (forced sampling)" : "");
	printf("touchtest: touch the panel; zones map top/bottom/left/right/centre to up/down/left/right/enter\n");
	printf("touchtest: ctrl-C to stop\n");

	touch_paint(vid, vc, XPT_ZONE_NONE, "touchtest: touch me");

	start = get_timer(0);
	while (get_timer(start) < (ulong)secs * 1000) {
		unsigned int x = 0, y = 0, z1 = 0, z2 = 0, sx, sy, rt;
		enum xpt_zone zone;
		bool down;

		if (ctrlc()) {
			puts("touchtest: interrupted\n");
			break;
		}

		/*
		 * Pen state first: a GPIO read is free, an SPI transaction is
		 * not, and hammering the bus while nothing is touching it would
		 * also starve the display's cyclic sync.
		 */
		down = have_pen ? !!dm_gpio_get_value(&pen) : false;
		if (down && !was_down)
			pendowns++;
		was_down = down;

		if (!down && !force) {
			if (shown != XPT_ZONE_NONE) {
				touch_paint(vid, vc, XPT_ZONE_NONE, "pen up");
				shown = XPT_ZONE_NONE;
			}
			mdelay(TOUCH_POLL_MS);
			continue;
		}

		if (last && get_timer(last) < TOUCH_SAMPLE_MS) {
			mdelay(TOUCH_POLL_MS);
			continue;
		}

		/*
		 * -EBUSY here is the display's cyclic sync holding the bus (or
		 * this claim landing inside one). It is not a failure: skip the
		 * sample and try again rather than aborting the run.
		 */
		ret = xpt_sample(slave, &x, &y, &z1, &z2);
		last = get_timer(0);

		if (ret == -EBUSY) {
			busy++;
			mdelay(TOUCH_POLL_MS);
			continue;
		}

		if (ret) {
			printf("touchtest: SPI transfer failed: %d\n", ret);
			break;
		}

		samples++;
		if (x < xmin)
			xmin = x;
		if (x > xmax)
			xmax = x;
		if (y < ymin)
			ymin = y;
		if (y > ymax)
			ymax = y;
		if (x == y)
			same++;
		if (x == XPT_ADC_MAX && y == XPT_ADC_MAX)
			railed_hi++;
		if (!x && !y)
			railed_lo++;

		xpt_to_screen(x, y, &sx, &sy);
		zone = down ? xpt_zone_of(sx, sy) : XPT_ZONE_NONE;
		rt = xpt_pressure(x, z1, z2);

		printf("touchtest: %s x=%4u y=%4u z1=%4u z2=%4u rt=%6u ohm  screen=%4u,%4u/1000  zone=%-6s key=%s\n",
		       down ? "DOWN" : "up  ", x, y, z1, z2, rt, sx, sy,
		       xpt_zones[zone].name, xpt_zones[zone].key);

		if (zone != shown) {
			touch_paint(vid, vc, zone, xpt_zones[zone].key);
			shown = zone;
		}
	}

	if (have_pen)
		dm_gpio_free(dev, &pen);

	printf("touchtest: %u samples, %u pen-down edges, %u bus-busy skips\n",
	       samples, pendowns, busy);
	if (!samples) {
		printf("touchtest: NO SAMPLES -- pen never reported down; re-run as 'touchtest %u force'\n",
		       secs);
		return CMD_RET_FAILURE;
	}

	printf("touchtest: x %u..%u, y %u..%u\n", xmin, xmax, ymin, ymax);

	/*
	 * Same verdicts the validated userspace reader applies. X and Y use
	 * different command bytes, so differing replies prove the slave DECODED
	 * what was sent rather than the bus idling at a rail.
	 */
	if (railed_hi == samples) {
		printf("touchtest: FAIL -- every read 0x%03x: MISO stuck high (CS not reaching the device, or pinmux)\n",
		       XPT_ADC_MAX);
		return CMD_RET_FAILURE;
	}
	if (railed_lo == samples) {
		puts("touchtest: FAIL -- every read 0: MISO stuck low\n");
		return CMD_RET_FAILURE;
	}
	if (same == samples) {
		puts("touchtest: FAIL -- X and Y identical in every sample: slave is not decoding commands (MOSI/CS path)\n");
		return CMD_RET_FAILURE;
	}

	puts("touchtest: PASS -- slave decodes commands (per-channel replies differ)\n");
	puts("touchtest: zone mapping is NOT self-verifying; a human must touch each of the five regions\n");

	return 0;
}

U_BOOT_CMD(
	touchtest, 3,	1,	do_touchtest,
	"sample an XPT2046/ADS7846 touch panel and report zones",
	"    [seconds] [force]\n"
	"      seconds - run for this long (default 30); ctrl-C also exits\n"
	"      force   - sample even when the pendown GPIO says pen-up.\n"
	"                An untouched resistive panel rails X and Y to\n"
	"                OPPOSITE ends, which is enough to prove the wire\n"
	"                without anyone touching it.\n"
	"      e.g. touchtest          - 30 s, sampling only while touched\n"
	"           touchtest 10 force - 10 s wire check, no human needed"
);
