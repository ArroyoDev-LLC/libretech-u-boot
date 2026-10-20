// SPDX-License-Identifier: GPL-2.0+
/*
 * Touchscreen zones as a keyboard -- XPT2046/ADS7846 on SPI
 *
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * A resistive touch panel presented through the keyboard uclass, so that any
 * consumer of stdin -- the bootmenu in particular -- becomes touch-navigable
 * with no change of its own. The panel is divided into a five-zone cross and
 * each zone stands for one key:
 *
 *	top    -> KEY_UP	bottom -> KEY_DOWN
 *	left   -> KEY_LEFT	right  -> KEY_RIGHT
 *	centre -> KEY_ENTER
 *
 * Nothing here knows what a bootmenu is. The keycodes go into the input layer,
 * which already turns KEY_UP/DOWN/LEFT/RIGHT into ECMA-48 escape sequences
 * (\e[A, \e[B, \e[D, \e[C) and KEY_ENTER into '\r' -- exactly the bytes the CLI
 * escape decoder (cli_ch_process()) converts into the ^P/^N/'\n' that
 * bootmenu_conv_key() consumes. So the bootmenu needs zero changes, and the
 * same touches also work at the command prompt.
 *
 * Structure follows drivers/input/button_kbd.c (a non-keyboard input source
 * presented as a keyboard) for the input plumbing, and
 * drivers/input/apple_spi_kbd.c for binding a UCLASS_KEYBOARD driver to a node
 * on an SPI bus. The sampling and zone decode are shared with cmd/touchtest.c
 * through <xpt2046.h> so the two cannot disagree about where a zone is.
 *
 * Two properties matter for a device whose read_keys() is called from the
 * console's poll loop:
 *
 *  - A poll must be nearly free. tstc() runs continuously, and the touch
 *    controller is on the same SPI controller as the panel at 1.3 MHz, so an
 *    SPI transaction per poll would both cost more than the console can afford
 *    and starve the display's cyclic sync. The pendown GPIO is read first and
 *    the bus is touched only while a finger is actually down, rate-limited.
 *
 *  - A key is emitted on pen-UP, not pen-down. That debounces for free (a
 *    resistive panel chatters on contact) and means a finger held on a zone
 *    produces exactly one keypress instead of an auto-repeat storm through a
 *    menu.
 */

#define LOG_CATEGORY UCLASS_KEYBOARD

#include <command.h>
#include <console.h>
#include <dm.h>
#include <env.h>
#include <errno.h>
#include <input.h>
#include <keyboard.h>
#include <log.h>
#include <spi.h>
#include <time.h>
#include <vsprintf.h>
#include <xpt2046.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/string.h>

#define TOUCH_KBD_SAMPLE_MS	40	/* min gap between SPI samples */

/**
 * struct touch_kbd_priv - driver private data
 *
 * @slave:	SPI slave for the touch controller (from the parent bus)
 * @pen:	pendown GPIO, asserted while a finger is down
 * @have_pen:	@pen is usable; without it nothing can be emitted
 * @armed:	a conversion has been taken, so PENIRQ is enabled (see xpt_arm())
 * @was_down:	pen state at the previous poll, for edge detection
 * @zone:	zone of the most recent valid sample of the current touch
 * @last_ms:	timestamp of the last SPI sample, for rate limiting
 * @presses:	keys emitted since probe
 * @samples:	SPI samples taken since probe
 * @busy:	samples skipped because another slave held the bus
 */
struct touch_kbd_priv {
	struct spi_slave *slave;
	struct gpio_desc pen;
	bool have_pen;
	bool armed;
	bool was_down;
	enum xpt_zone zone;
	ulong last_ms;
	unsigned int presses;
	unsigned int samples;
	unsigned int busy;
};

/**
 * touch_kbd_arm() - take the one conversion that enables PENIRQ
 *
 * read_keys() reaches the bus only while the pen is down, which is what keeps a
 * console poll free -- and which means it can never be the thing that arms the
 * controller. Something on a path that does NOT consult the pen has to do it,
 * exactly once. See xpt_arm() for why the pendown GPIO is meaningless until it
 * happens.
 *
 * Return: 0 if OK, -ve if the sample failed (the controller stays unarmed)
 */
static int touch_kbd_arm(struct touch_kbd_priv *priv)
{
	int ret;

	ret = xpt_arm(priv->slave);
	if (ret)
		return ret;

	priv->armed = true;

	return 0;
}

/**
 * touch_kbd_emit() - push one keypress into the input FIFO
 *
 * A press followed immediately by a release: input_add_keycode() only queues
 * characters for the press, and the release clears the key from the input
 * layer's "previously pressed" set so the same zone touched twice in a row
 * registers twice.
 */
static void touch_kbd_emit(struct input_config *input, enum xpt_zone zone)
{
	int code = xpt_zones[zone].keycode;

	if (!code)
		return;

	input_add_keycode(input, code, false);
	input_add_keycode(input, code, true);
}

/**
 * touch_kbd_read_keys() - input-layer callback: sample and queue keycodes
 *
 * Called from input_tstc()/input_getc(), i.e. from the console's poll loop.
 *
 * Return: 0 always, which is what the input layer wants from a device that may
 *	have nothing to say. input_getc() treats a non-zero return as "give up
 *	and return -1", so a driver must not report "I queued something" the way
 *	the keyboard_ops tstc() method does; input_tstc() picks the queued
 *	character up on its next call, having found the FIFO non-empty. This is
 *	the same convention button_kbd.c uses.
 */
static int touch_kbd_read_keys(struct input_config *input)
{
	struct touch_kbd_priv *priv = dev_get_priv(input->dev);
	unsigned int x = 0, y = 0, z1 = 0, z2 = 0, sx, sy;
	bool down;
	int ret;

	if (!priv->have_pen)
		return 0;

	/*
	 * A GPIO read is free; an SPI transaction is not. Poll the pen first
	 * so that the overwhelmingly common case -- nobody is touching the
	 * panel -- costs one register read and never reaches the bus.
	 */
	down = !!dm_gpio_get_value(&priv->pen);

	if (!down) {
		if (priv->was_down) {
			/*
			 * Release: emit whatever zone the touch settled on.
			 * A touch that came and went between two polls has no
			 * sampled zone and correctly emits nothing.
			 */
			priv->was_down = false;
			if (priv->zone != XPT_ZONE_NONE) {
				log_debug("release in zone %s -> %s\n",
					  xpt_zones[priv->zone].name,
					  xpt_zones[priv->zone].key);
				touch_kbd_emit(input, priv->zone);
				priv->presses++;
				priv->zone = XPT_ZONE_NONE;
			}
		}
		return 0;
	}

	if (!priv->was_down) {
		priv->was_down = true;
		priv->zone = XPT_ZONE_NONE;
		priv->last_ms = 0;
	}

	/* Rate-limit the bus while the finger stays down */
	if (priv->last_ms && get_timer(priv->last_ms) < TOUCH_KBD_SAMPLE_MS)
		return 0;

	/*
	 * -EBUSY is the display's cyclic sync holding the controller (it is
	 * reached from video_idle() inside schedule(), which SPI poll loops
	 * enter through udelay(), so it can land on top of a console poll).
	 * That is not an error: skip this sample, keep the pen state, and try
	 * again on the next poll. Never block and never fail the console.
	 */
	ret = xpt_sample(priv->slave, &x, &y, &z1, &z2);
	priv->last_ms = get_timer(0);
	if (ret) {
		if (ret == -EBUSY)
			priv->busy++;
		else
			log_debug("sample failed: %d\n", ret);
		return 0;
	}

	priv->samples++;
	xpt_to_screen(x, y, &sx, &sy);
	priv->zone = xpt_zone_of(sx, sy);

	return 0;
}

static int touch_kbd_probe(struct udevice *dev)
{
	struct touch_kbd_priv *priv = dev_get_priv(dev);
	struct keyboard_priv *uc_priv = dev_get_uclass_priv(dev);
	struct stdio_dev *sdev = &uc_priv->sdev;
	struct input_config *input = &uc_priv->input;
	int ret;

	priv->slave = dev_get_parent_priv(dev);
	if (!priv->slave)
		return -ENODEV;

	/*
	 * The touch controller is 8-bit; the panel on the other chip select
	 * uses 16 (and 64 for pixels). Word length is per slave, so set ours
	 * once here rather than inheriting whatever ran last.
	 */
	ret = dm_spi_set_wordlen(dev, 8);
	if (ret && ret != -ENOSYS) {
		log_err("cannot set 8-bit words: %d\n", ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "pendown-gpio", 0, &priv->pen,
				   GPIOD_IS_IN);
	if (ret) {
		/*
		 * Without the pen GPIO every poll would have to hit the bus,
		 * which the console poll loop cannot afford. Refuse to probe
		 * rather than register a keyboard that would cripple stdin.
		 */
		log_err("no usable pendown-gpio (%d), refusing to probe\n", ret);
		return ret;
	}
	priv->have_pen = true;
	priv->zone = XPT_ZONE_NONE;

	/*
	 * Arm PENIRQ before the console can start polling us. A failure here is
	 * not fatal -- -EBUSY is the display holding the bus at the wrong
	 * moment, and 'touchkbd arm' (and the first 'touchkbd watch') retries --
	 * but it does mean the pen reads "up" until something succeeds, so say
	 * so rather than leaving it to be rediscovered on the bench.
	 */
	ret = touch_kbd_arm(priv);
	if (ret)
		log_warning("could not arm PENIRQ at probe (%d) -- pen-down will not be seen until 'touchkbd arm' succeeds\n",
			    ret);

	input_init(input, false);
	input_add_tables(input, false);
	input->dev = dev;
	input->read_keys = touch_kbd_read_keys;

	strcpy(sdev->name, "touchkbd");
	ret = input_stdio_register(sdev);
	if (ret) {
		log_err("input_stdio_register() failed: %d\n", ret);
		dm_gpio_free(dev, &priv->pen);
		priv->have_pen = false;
		return ret;
	}

	log_debug("%s: touch keyboard on %s, %u Hz\n", dev->name,
		  dev->parent->name, priv->slave->max_hz);

	return 0;
}

static const struct keyboard_ops touch_kbd_ops = {
};

static const struct udevice_id touch_kbd_ids[] = {
	{ .compatible = "ti,ads7846" },
	{ .compatible = "ti,tsc2046" },
	{ }
};

U_BOOT_DRIVER(touch_kbd) = {
	.name		= "touch_kbd",
	.id		= UCLASS_KEYBOARD,
	.of_match	= touch_kbd_ids,
	.ops		= &touch_kbd_ops,
	.priv_auto	= sizeof(struct touch_kbd_priv),
	.probe		= touch_kbd_probe,
};

#if IS_ENABLED(CONFIG_CMDLINE)
/*
 * 'touchkbd' -- inspect and exercise the driver without a finger.
 *
 * 'emit' is a SYNTHETIC injection: it proves the keycode -> input-layer ->
 * FIFO path, and nothing about the touch hardware. Only a human touching the
 * panel can confirm the zone mapping.
 */
static int touch_kbd_cmd_dev(struct udevice **devp,
			     struct touch_kbd_priv **privp,
			     struct input_config **inputp)
{
	struct keyboard_priv *uc_priv;
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_KEYBOARD,
					  DM_DRIVER_GET(touch_kbd), &dev);
	if (ret) {
		printf("touchkbd: no touch keyboard (%d) -- is the overlay (spi-cc-2cs + spi-cc-2cs-mhs3528) applied?\n",
		       ret);
		return ret;
	}

	uc_priv = dev_get_uclass_priv(dev);
	if (devp)
		*devp = dev;
	if (privp)
		*privp = dev_get_priv(dev);
	if (inputp)
		*inputp = &uc_priv->input;

	return 0;
}

static int do_touchkbd(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	struct input_config *input;
	struct touch_kbd_priv *priv;
	struct udevice *dev;
	const char *cmd;
	int i, ret;

	if (argc < 2)
		return CMD_RET_USAGE;
	cmd = argv[1];

	ret = touch_kbd_cmd_dev(&dev, &priv, &input);
	if (ret)
		return CMD_RET_FAILURE;

	if (!strcmp(cmd, "status")) {
		printf("touchkbd: %s on %s, %u Hz, stdin=%s\n", dev->name,
		       dev->parent->name, priv->slave->max_hz,
		       env_get("stdin") ? env_get("stdin") : "(unset)");
		printf("touchkbd: PENIRQ %s\n", priv->armed ?
		       "armed (a conversion has been taken)" :
		       "NOT ARMED -- pen-down cannot be seen; run 'touchkbd arm'");
		printf("touchkbd: pen %s, %u samples, %u keys emitted, %u bus-busy skips\n",
		       priv->have_pen ?
			(dm_gpio_get_value(&priv->pen) ? "DOWN" : "up") :
			"unavailable",
		       priv->samples, priv->presses, priv->busy);
		printf("touchkbd: zones");
		for (i = XPT_ZONE_CENTRE; i < XPT_ZONE_COUNT; i++)
			printf(" %s=%s", xpt_zones[i].name, xpt_zones[i].key);
		printf("\n");

		return 0;
	}

	if (!strcmp(cmd, "arm")) {
		ret = touch_kbd_arm(priv);
		if (ret) {
			printf("touchkbd: arm failed (%d)%s\n", ret,
			       ret == -EBUSY ? " -- bus held, try again" : "");
			return CMD_RET_FAILURE;
		}
		printf("touchkbd: armed -- PENIRQ enabled, pen %s\n",
		       dm_gpio_get_value(&priv->pen) ? "DOWN" : "up");

		return 0;
	}

	if (!strcmp(cmd, "emit")) {
		if (argc != 3)
			return CMD_RET_USAGE;
		for (i = XPT_ZONE_CENTRE; i < XPT_ZONE_COUNT; i++) {
			if (!strcmp(argv[2], xpt_zones[i].name) ||
			    !strcmp(argv[2], xpt_zones[i].key))
				break;
		}
		if (i == XPT_ZONE_COUNT) {
			printf("touchkbd: unknown zone '%s'\n", argv[2]);
			return CMD_RET_USAGE;
		}
		printf("touchkbd: SYNTHETIC -- injecting %s (%s, keycode %d); this tests the keycode path, NOT the panel\n",
		       xpt_zones[i].name, xpt_zones[i].key,
		       xpt_zones[i].keycode);
		touch_kbd_emit(input, i);

		return 0;
	}

	if (!strcmp(cmd, "drain")) {
		int count = 0;

		printf("touchkbd: FIFO:");
		while (input_tstc(input)) {
			int ch = input_getc(input);

			if (ch < 0)
				break;
			printf(" %02x", ch);
			count++;
		}
		printf("%s (%d bytes)\n", count ? "" : " (empty)", count);

		return 0;
	}

	if (!strcmp(cmd, "watch")) {
		unsigned int secs = argc == 3 ? dectoul(argv[2], NULL) : 30;
		enum xpt_zone shown = XPT_ZONE_NONE;
		ulong start;

		if (!secs)
			return CMD_RET_USAGE;

		/*
		 * Re-arm: this is the diagnostic a human reaches for when the
		 * panel appears dead, so it must not itself depend on the pen
		 * having been meaningful up to now.
		 */
		ret = touch_kbd_arm(priv);
		if (ret)
			printf("touchkbd: could not arm PENIRQ (%d) -- pen-down may not be seen\n",
			       ret);

		printf("touchkbd: reporting zones for %u s WITHOUT emitting keys; ctrl-C to stop\n",
		       secs);

		start = get_timer(0);
		while (get_timer(start) < (ulong)secs * 1000) {
			unsigned int x, y, z1, z2, sx, sy;
			enum xpt_zone zone;

			if (ctrlc()) {
				puts("touchkbd: interrupted\n");
				break;
			}
			if (!dm_gpio_get_value(&priv->pen)) {
				if (shown != XPT_ZONE_NONE) {
					puts("touchkbd: pen up\n");
					shown = XPT_ZONE_NONE;
				}
				mdelay(20);
				continue;
			}
			if (xpt_sample(priv->slave, &x, &y, &z1, &z2)) {
				mdelay(20);
				continue;
			}
			xpt_to_screen(x, y, &sx, &sy);
			zone = xpt_zone_of(sx, sy);
			if (zone != shown) {
				printf("touchkbd: DOWN x=%4u y=%4u screen=%4u,%4u/1000 zone=%-6s would emit %s\n",
				       x, y, sx, sy, xpt_zones[zone].name,
				       xpt_zones[zone].key);
				shown = zone;
			}
			mdelay(40);
		}

		return 0;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(
	touchkbd, 3,	0,	do_touchkbd,
	"inspect the touchscreen keyboard",
	"status            - show the device, PENIRQ arm state, pen state and counters\n"
	"touchkbd arm               - take one conversion, which is what enables PENIRQ.\n"
	"                             Until this has happened the pendown GPIO reads\n"
	"                             'up' no matter what the panel does. Done at probe\n"
	"                             and by 'watch'; this is the manual retry.\n"
	"touchkbd watch [secs]      - report the zone under the finger without emitting\n"
	"touchkbd emit <zone>       - SYNTHETIC: inject a zone's keycode into the input\n"
	"                             FIFO. Proves the keycode path only; it says\n"
	"                             nothing about the touch panel.\n"
	"                             zone: centre|top|bottom|left|right (or the key name)\n"
	"touchkbd drain             - pop and hexdump whatever is queued in the FIFO"
);
#endif /* CONFIG_CMDLINE */
