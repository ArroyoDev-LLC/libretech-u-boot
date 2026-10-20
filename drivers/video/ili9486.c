// SPDX-License-Identifier: GPL-2.0+
/*
 * ILI9486 SPI TFT display driver for U-Boot
 *
 * Copyright (C) 2024 Da Xue <da@libre.computer>
 *
 * Supports 480x320 RGB565 displays using the ILI9486 controller
 * behind a SPI-to-16-bit-parallel converter (Waveshare/PiScreen).
 *
 * Commands and parameters are zero-padded to 16 bits and sent as 8-bit
 * words; pixel data is sent as native 16-bit words. See the wordlen note
 * below -- the two paths deliberately use different widths, as Linux does.
 *
 * Init sequence follows Linux DRM drivers/gpu/drm/tiny/ili9486.c.
 */

#include <asm/byteorder.h>
#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <malloc.h>
#include <memalign.h>
#include <spi.h>
#include <video.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <linux/delay.h>

#define ILI9486_WIDTH		480
#define ILI9486_HEIGHT		320

/* ILI9486 commands */
#define ILI9486_SLPOUT		0x11
#define ILI9486_DISPON		0x29
#define ILI9486_CASET		0x2A
#define ILI9486_PASET		0x2B
#define ILI9486_RAMWR		0x2C
#define ILI9486_MADCTL		0x36
#define ILI9486_PIXFMT		0x3A
#define ILI9486_ITFCTR1		0xB0
#define ILI9486_PWCTRL1		0xC2
#define ILI9486_VMCTRL1		0xC5
#define ILI9486_PGAMCTRL	0xE0
#define ILI9486_NGAMCTRL	0xE1
#define ILI9486_DGAMCTRL	0xE2

/* MADCTL bits */
#define MADCTL_MV	BIT(5)
#define MADCTL_MX	BIT(6)
#define MADCTL_MY	BIT(7)
#define MADCTL_BGR	BIT(3)

/*
 * Word length is chosen PER TRANSFER, exactly as Linux's ili9486 does in
 * waveshare_command() (drivers/gpu/drm/tiny/ili9486.c):
 *
 *   commands + parameters -- bpw 8, each 8-bit value padded to 16 bits
 *                            big-endian in software ({ 0x00, val }).
 *   pixel data (RAMWR)    -- bpw 16, native u16, NOT byte-swapped.
 *
 * Linux picks between the two pixel forms with
 *     if (!mipi->swap_bytes) bpw = 16;
 * i.e. byte-swapping and 16-bit words are ALTERNATIVES, never both. An
 * 8-bit-only controller swaps and sends at 8; a controller that does 16 --
 * the Meson SPICC -- skips the swap and sends at 16.
 *
 * Sending COMMANDS at 16 is what blanked this panel on the bench 2026-08-04:
 * the { 0x00, cmd } byte pair is reinterpreted as a little-endian u16 and
 * shifted out MSB-first as (cmd, 0x00), so every init command was corrupted
 * while display_init still returned 0. Linux never sends a command at 16.
 */
#define ILI9486_CMD_WORDLEN	8
#define ILI9486_PIXEL_WORDLEN	16

struct ili9486_priv {
	struct gpio_desc reset_gpio;
	struct gpio_desc dc_gpio;
	struct udevice *dev;
	u8 *tx_buf;
};

/*
 * Send a 16-bit-padded command (DC=0).
 * The SPI-to-parallel converter latches 16 clocks per word.
 */
static int ili9486_write_cmd(struct udevice *dev, u8 cmd)
{
	struct ili9486_priv *priv = dev_get_priv(dev);
	u8 buf[2] = { 0x00, cmd };

	dm_gpio_set_value(&priv->dc_gpio, 0);
	return dm_spi_xfer(dev, 16, buf, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
}

/*
 * Send command + parameters, each 16-bit padded, in one CS transaction.
 */
static int ili9486_cmd_data(struct udevice *dev, u8 cmd, const u8 *data,
			    size_t len)
{
	struct ili9486_priv *priv = dev_get_priv(dev);
	u8 buf[64];
	int ret;
	size_t i;

	buf[0] = 0x00;
	buf[1] = cmd;
	dm_gpio_set_value(&priv->dc_gpio, 0);

	if (len > 0) {
		/*
		 * Each chunk gets its OWN CS frame, and DC only changes while
		 * CS is high. Holding CS across the command->parameter DC
		 * transition (BEGIN ... END) leaves the converter mid-frame
		 * when D/CX flips, and it never recovers: the panel stays
		 * blank. Replaying this same byte stream from Linux, where
		 * every chunk is separately CS-framed, drives the panel
		 * correctly (verified on the bench S905X, 2026-08-04).
		 */
		ret = dm_spi_xfer(dev, 16, buf, NULL,
				  SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret)
			return ret;

		for (i = 0; i < len; i++) {
			buf[i * 2] = 0x00;
			buf[i * 2 + 1] = data[i];
		}
		dm_gpio_set_value(&priv->dc_gpio, 1);
		ret = dm_spi_xfer(dev, len * 16, buf, NULL,
				  SPI_XFER_BEGIN | SPI_XFER_END);
	} else {
		ret = dm_spi_xfer(dev, 16, buf, NULL,
				  SPI_XFER_BEGIN | SPI_XFER_END);
	}

	return ret;
}

static int ili9486_set_window(struct udevice *dev, u16 x0, u16 y0,
			      u16 x1, u16 y1)
{
	u8 data[4];
	int ret;

	data[0] = x0 >> 8;
	data[1] = x0 & 0xff;
	data[2] = x1 >> 8;
	data[3] = x1 & 0xff;
	ret = ili9486_cmd_data(dev, ILI9486_CASET, data, 4);
	if (ret)
		return ret;

	data[0] = y0 >> 8;
	data[1] = y0 & 0xff;
	data[2] = y1 >> 8;
	data[3] = y1 & 0xff;
	return ili9486_cmd_data(dev, ILI9486_PASET, data, 4);
}

/*
 * Init sequence from Linux DRM drivers/gpu/drm/tiny/ili9486.c
 */
static int ili9486_display_init(struct udevice *dev)
{
	int ret;

	/* Interface Mode Control - no params */
	ret = ili9486_write_cmd(dev, ILI9486_ITFCTR1);
	if (ret)
		return ret;

	/* Sleep Out */
	ret = ili9486_write_cmd(dev, ILI9486_SLPOUT);
	if (ret)
		return ret;
	mdelay(250);

	/* Pixel format: 16bpp (RGB565) */
	ret = ili9486_cmd_data(dev, ILI9486_PIXFMT, (u8[]){0x55}, 1);
	if (ret)
		return ret;

	/* Power Control 3 */
	ret = ili9486_cmd_data(dev, ILI9486_PWCTRL1, (u8[]){0x44}, 1);
	if (ret)
		return ret;

	/* VCOM Control */
	ret = ili9486_cmd_data(dev, ILI9486_VMCTRL1,
			       (u8[]){0x00, 0x00, 0x00, 0x00}, 4);
	if (ret)
		return ret;

	/* Positive Gamma Correction */
	ret = ili9486_cmd_data(dev, ILI9486_PGAMCTRL,
		(u8[]){0x0F, 0x1F, 0x1C, 0x0C, 0x0F, 0x08, 0x48, 0x98,
		       0x37, 0x0A, 0x13, 0x04, 0x11, 0x0D, 0x00}, 15);
	if (ret)
		return ret;

	/* Negative Gamma Correction */
	ret = ili9486_cmd_data(dev, ILI9486_NGAMCTRL,
		(u8[]){0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75,
		       0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00}, 15);
	if (ret)
		return ret;

	/* Digital Gamma Control */
	ret = ili9486_cmd_data(dev, ILI9486_DGAMCTRL,
		(u8[]){0x0F, 0x32, 0x2E, 0x0B, 0x0D, 0x05, 0x47, 0x75,
		       0x37, 0x06, 0x10, 0x03, 0x24, 0x20, 0x00}, 15);
	if (ret)
		return ret;

	/* Display On */
	ret = ili9486_write_cmd(dev, ILI9486_DISPON);
	if (ret)
		return ret;
	mdelay(100);

	/* Memory Access Control - landscape, BGR (after DISPON per Linux) */
	ret = ili9486_cmd_data(dev, ILI9486_MADCTL,
			       (u8[]){MADCTL_MV | MADCTL_MY | MADCTL_MX |
				      MADCTL_BGR}, 1);
	if (ret)
		return ret;

	return 0;
}

static int ili9486_sync(struct udevice *vid)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(vid);
	struct ili9486_priv *priv = dev_get_priv(vid);
	struct udevice *dev = priv->dev;
	u16 *fb = (u16 *)uc_priv->fb;
	u8 cmd_buf[2] = { 0x00, ILI9486_RAMWR };
	u16 *tx = (u16 *)priv->tx_buf;
	int x0, y0, x1, y1, w, y;
	unsigned int pixel_wordlen = ILI9486_PIXEL_WORDLEN;
	size_t tx_size, npix, i;
	int ret;

	if (IS_ENABLED(CONFIG_VIDEO_DAMAGE)) {
		x0 = uc_priv->damage.xstart;
		y0 = uc_priv->damage.ystart;
		x1 = uc_priv->damage.xend;
		y1 = uc_priv->damage.yend;

		if (x0 >= x1 || y0 >= y1) {
			return 0;
		}
	} else {
		x0 = 0;
		y0 = 0;
		x1 = uc_priv->xsize;
		y1 = uc_priv->ysize;
	}

	/*
	 * Printed on EVERY sync, not once: the first sync happens during boot,
	 * before the usbacm gadget console exists, so a one-shot report is
	 * only ever seen on the UART.
	 */


	w = x1 - x0;
	npix = (size_t)w * (y1 - y0);
	tx_size = npix * 2;

	for (y = y0; y < y1; y++) {
		u16 *src = &fb[y * uc_priv->xsize + x0];
		u16 *dst = &tx[(y - y0) * w];
		int x;

		/*
		 * No __swab16(): the controller shifts the native u16 out
		 * MSB-first, which IS the big-endian order the panel wants.
		 * Pre-swapping here would double-transform it.
		 */
		for (x = 0; x < w; x++)
			dst[x] = src[x];
	}

	/*
	 * Widen the pixel stream to 64-bit words when it divides evenly.
	 *
	 * GXL terminates a DMA session after CONREG.BURSTLENGTH+1 words and
	 * the driver caps that at 128, so the session count is set by the WORD
	 * count, not the byte count: at bpw 16 a full frame is ~1200 sessions
	 * of 256 bytes, and per-session setup dominates the transfer. Four
	 * pixels per 64-bit word cuts that to ~300 sessions of 1024 bytes.
	 *
	 * The word is transmitted MSB-first, so the first pixel on the wire
	 * must occupy the HIGH half: V = P0<<48 | P1<<32 | P2<<16 | P3. Packed
	 * in place -- each iteration reads the four u16 it then overwrites.
	 */
	if (!(npix % 4)) {
		u64 *q = (u64 *)tx;

		for (i = 0; i < npix / 4; i++) {
			u16 *g = &tx[i * 4];

			q[i] = ((u64)g[0] << 48) | ((u64)g[1] << 32) |
			       ((u64)g[2] << 16) | (u64)g[3];
		}
		pixel_wordlen = 64;
	}

	/*
	 * Propagate claim failure (e.g. -EBUSY from a re-entrant cyclic
	 * sync during another transfer). Returning 0 here would let
	 * video_sync() reset the damage rect with nothing transferred,
	 * silently dropping the pending pixels.
	 */
	ret = dm_spi_claim_bus(dev);
	if (ret)
		return ret;

	ret = ili9486_set_window(dev, x0, y0, x1 - 1, y1 - 1);
	if (ret)
		goto out;

	dm_gpio_set_value(&priv->dc_gpio, 0);
	ret = dm_spi_xfer(dev, 16, cmd_buf, NULL,
			  SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret)
		goto out;

	dm_gpio_set_value(&priv->dc_gpio, 1);
	flush_dcache_range((ulong)priv->tx_buf,
			   (ulong)priv->tx_buf + tx_size);

	/*
	 * Pixels only: 16-bit words, native byte order. Restored to the
	 * command width afterwards (including on the error path) so the next
	 * command -- and any other slave on this bus -- is not left at 16.
	 */
	ret = dm_spi_set_wordlen(dev, pixel_wordlen);
	if (ret)
		goto out;

	ret = dm_spi_xfer(dev, tx_size * 8, priv->tx_buf, NULL,
			  SPI_XFER_BEGIN | SPI_XFER_END);

	dm_spi_set_wordlen(dev, ILI9486_CMD_WORDLEN);


out:
	dm_spi_release_bus(dev);
	return ret;
}

static void ili9486_hw_reset(struct ili9486_priv *priv)
{
	/* Match Linux mipi_dbi_hw_reset(): assert low, release high */
	dm_gpio_set_value(&priv->reset_gpio, 0);
	mdelay(1);
	dm_gpio_set_value(&priv->reset_gpio, 1);
	mdelay(120);
}

static int ili9486_probe(struct udevice *dev)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct ili9486_priv *priv = dev_get_priv(dev);
	int ret;

	ret = gpio_request_by_name(dev, "reset-gpios", 0,
				   &priv->reset_gpio, GPIOD_IS_OUT);
	if (ret) {
		dev_err(dev, "missing reset GPIO: %d\n", ret);
		return ret;
	}

	ret = gpio_request_by_name(dev, "dc-gpios", 0,
				   &priv->dc_gpio, GPIOD_IS_OUT);
	if (ret) {
		dev_err(dev, "missing dc GPIO: %d\n", ret);
		return ret;
	}

	uc_priv->bpix = VIDEO_BPP16;
	uc_priv->xsize = ILI9486_WIDTH;
	uc_priv->ysize = ILI9486_HEIGHT;
	uc_priv->rot = 0;

	priv->dev = dev;

	priv->tx_buf = memalign(64, ILI9486_WIDTH * ILI9486_HEIGHT * 2);
	if (!priv->tx_buf) {
		dev_err(dev, "Failed to allocate TX buffer\n");
		return -ENOMEM;
	}

	/*
	 * SPI mode comes from the DT (mode 0 for this panel) and is NOT
	 * overridden here. Mode 0 with 8-bit commands is the only pairing ever
	 * observed to drive this panel -- it matches the userspace spidev
	 * replay in amlogic/software/tests/spi/ili9486-uboot-replay.c, which
	 * turned the screen red, and it is what rendered text in U-Boot on
	 * 2026-08-04. Do not force SPI_CPOL|SPI_CPHA here: mode 3 was an
	 * untested SCLK-pull-up theory, and with it in place the panel is
	 * blank.
	 *
	 * Commands go out at 8 bits per word. The { 0x00, cmd } byte pair is a
	 * BYTE STREAM; at bpw 16 the SPI framework treats the buffer as
	 * CPU-native u16s, so it reaches the wire as (cmd, 0x00) and every
	 * init command is corrupted while display_init still returns 0.
	 * Pixels are switched to 16 (or 64) per transfer in ili9486_sync().
	 */
	ret = dm_spi_set_wordlen(dev, ILI9486_CMD_WORDLEN);
	if (ret && ret != -ENOSYS)
		dev_warn(dev, "%d-bit words unavailable (%d); framing may be wrong\n",
			 ILI9486_CMD_WORDLEN, ret);

	/*
	 * Claim (and thereby fully configure) the SPI controller BEFORE the
	 * panel reset: the hat's SPI-to-16-bit converter counts SCLK edges
	 * across CS frames with no CS-based reframe, so any clock glitch
	 * emitted by controller setup after the reset permanently misaligns
	 * its 16-clock framing. Validated on the bench S905X: a userspace
	 * driver with reset-after-config works; reset-before-config does not.
	 */
	ret = dm_spi_claim_bus(dev);
	if (ret) {
		dev_err(dev, "Failed to claim SPI bus: %d\n", ret);
		return ret;
	}

	ili9486_hw_reset(priv);

	ret = ili9486_display_init(dev);
	if (ret) {
		dev_err(dev, "Display init failed: %d\n", ret);
		dm_spi_release_bus(dev);
		return ret;
	}

	/*
	 * Release the bus after init and re-claim per message, matching the
	 * working Linux path (drivers/gpu/drm/tiny/ili9486.c goes through
	 * spi_sync(), which claims and releases around every message).
	 *
	 * Holding one claim for the panel's life was the previous behaviour;
	 * it was tried on the bench and the panel stayed blank (2026-08-04).
	 */
	dm_spi_release_bus(dev);

	printf("ILI9486: %dx%d SPI display initialized\n",
	       ILI9486_WIDTH, ILI9486_HEIGHT);

	return 0;
}

static int ili9486_bind(struct udevice *dev)
{
	struct video_uc_plat *plat = dev_get_uclass_plat(dev);

	plat->size = ILI9486_WIDTH * ILI9486_HEIGHT * 2;

	/*
	 * SPI display devices bind after relocation, so video_reserve()
	 * never sees them. Self-allocate the framebuffer so video_post_bind()
	 * skips the reserved-memory check.
	 */
	if (gd->flags & GD_FLG_RELOC) {
		void *fb = memalign(64, plat->size);

		if (!fb)
			return -ENOMEM;
		plat->base = (ulong)fb;
	}

	return 0;
}

static const struct video_ops ili9486_ops = {
	.video_sync = ili9486_sync,
};

static const struct udevice_id ili9486_ids[] = {
	{ .compatible = "ilitek,ili9486" },
	{ .compatible = "waveshare,rpi-lcd-35" },
	{ }
};

U_BOOT_DRIVER(ili9486_video) = {
	.name		= "ili9486_video",
	.id		= UCLASS_VIDEO,
	.of_match	= ili9486_ids,
	.ops		= &ili9486_ops,
	.plat_auto	= sizeof(struct video_uc_plat),
	.bind		= ili9486_bind,
	.probe		= ili9486_probe,
	.priv_auto	= sizeof(struct ili9486_priv),
};
