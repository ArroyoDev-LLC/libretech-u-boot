// SPDX-License-Identifier: GPL-2.0+
/*
 * video commands
 *
 * Copyright 2022 Google LLC
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <command.h>
#include <console.h>
#include <vsprintf.h>
#include <dm.h>
#include <linux/delay.h>
#include <time.h>
#include <video.h>
#include <video_console.h>

static int do_video_setcursor(struct cmd_tbl *cmdtp, int flag, int argc,
			      char *const argv[])
{
	unsigned int col, row;
	struct udevice *dev;

	if (argc != 3)
		return CMD_RET_USAGE;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev))
		return CMD_RET_FAILURE;
	col = dectoul(argv[1], NULL);
	row = dectoul(argv[2], NULL);
	vidconsole_position_cursor(dev, col, row);

	return 0;
}

static int do_video_puts(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	struct udevice *dev;
	int ret;

	if (argc != 2)
		return CMD_RET_USAGE;

	if (uclass_first_device_err(UCLASS_VIDEO_CONSOLE, &dev))
		return CMD_RET_FAILURE;
	ret = vidconsole_put_string(dev, argv[1]);
	if (!ret)
		ret = video_sync(dev->parent, false);

	return ret ? CMD_RET_FAILURE : 0;
}

U_BOOT_CMD(
	setcurs, 3,	1,	do_video_setcursor,
	"set cursor position within screen",
	"    <col> <row> in character"
);

U_BOOT_CMD(
	lcdputs, 2,	1,	do_video_puts,
	"print string on video framebuffer",
	"    <string>"
);

/*
 * Display test: cycle full-screen colours and report the frame rate.
 *
 * Useful on slow serial-attached panels (SPI TFTs), where the cost of a frame
 * is dominated by the transport rather than by drawing: a full-screen fill is
 * the cheapest possible thing to draw, so the rate reported here is an upper
 * bound on what the display pipeline can deliver.
 */
struct vidtest_pattern {
	const char *name;
	u8 r, g, b;
};

static const struct vidtest_pattern vidtest_patterns[] = {
	{ "red",     0xff, 0x00, 0x00 },
	{ "green",   0x00, 0xff, 0x00 },
	{ "blue",    0x00, 0x00, 0xff },
	{ "yellow",  0xff, 0xff, 0x00 },
	{ "cyan",    0x00, 0xff, 0xff },
	{ "magenta", 0xff, 0x00, 0xff },
	{ "white",   0xff, 0xff, 0xff },
	{ "black",   0x00, 0x00, 0x00 },
};

#define VIDTEST_MAX_REPORT	16

static u32 vidtest_colour(struct video_priv *priv,
			  const struct vidtest_pattern *p)
{
	switch (priv->bpix) {
	case VIDEO_BPP16:
		return ((p->r >> 3) << 11) | ((p->g >> 2) << 5) | (p->b >> 3);
	case VIDEO_BPP32:
		return ((u32)p->r << 16) | ((u32)p->g << 8) | p->b;
	default:
		return (p->r | p->g | p->b) ? ~0U : 0;
	}
}

static int do_video_test(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	ulong times[VIDTEST_MAX_REPORT];
	unsigned int frames = ARRAY_SIZE(vidtest_patterns);
	unsigned int dwell_ms = 1000;
	unsigned int i, reported;
	struct video_priv *priv;
	struct udevice *dev, *vc;
	ulong total = 0;
	size_t frame_bytes;
	int ret;

	if (argc > 3)
		return CMD_RET_USAGE;
	if (argc >= 2) {
		frames = dectoul(argv[1], NULL);
		if (!frames)
			return CMD_RET_USAGE;
	}
	if (argc == 3)
		dwell_ms = dectoul(argv[2], NULL);

	if (uclass_first_device_err(UCLASS_VIDEO, &dev))
		return CMD_RET_FAILURE;
	priv = dev_get_uclass_priv(dev);

	/* Optional: without a console the test still runs, just unlabelled. */
	uclass_first_device(UCLASS_VIDEO_CONSOLE, &vc);

	frame_bytes = (size_t)priv->xsize * priv->ysize *
		      VNBITS(priv->bpix) / 8;

	/*
	 * With a dwell, each colour is held long enough to actually be
	 * identified -- at full rate this panel changes colour every ~137 ms,
	 * which is useless for spotting a wrong channel order. The name is
	 * announced BEFORE the timed region (so a vidconsole stdout cannot
	 * charge its own draw to the frame) and the delay is applied after it,
	 * so neither affects the reported rate.
	 *
	 * Pass a dwell of 0 to benchmark: vidtest 240 0
	 */
	for (i = 0; i < frames; i++) {
		const struct vidtest_pattern *pat;
		ulong t0, dt;

		pat = &vidtest_patterns[i % ARRAY_SIZE(vidtest_patterns)];

		/*
		 * video_fill() ends in video_sync(dev, false), which the
		 * uclass GATES on VIDEO_SYNC_MS -- so on its own it may only
		 * touch memory and never reach the panel, reporting a memset
		 * rate rather than a frame rate. Force the push.
		 */
		t0 = timer_get_us();
		ret = video_fill(dev, vidtest_colour(priv, pat));
		if (!ret)
			ret = video_sync(dev, true);
		dt = timer_get_us() - t0;
		if (ret) {
			printf("vidtest: frame %u (%s) failed: %d\n",
			       i, pat->name, ret);
			return CMD_RET_FAILURE;
		}

		if (i < VIDTEST_MAX_REPORT)
			times[i] = dt;
		total += dt;

		/*
		 * Label the frame ON THE PANEL, not just on the console -- the
		 * whole point of the dwell is to identify a colour by looking
		 * at the display, and a console line does not tell you which
		 * colour is in front of you. Drawn AFTER the timed region and
		 * synced separately, so it never charges its own draw to the
		 * measured frame. The next frame's full-screen fill wipes it.
		 */
		if (dwell_ms) {
			if (vc) {
				char label[32];

				snprintf(label, sizeof(label), "%s  %u/%u",
					 pat->name, i + 1, frames);
				vidconsole_position_cursor(vc, 0, 0);
				vidconsole_put_string(vc, label);
				video_sync(dev, true);
			}
			printf("vidtest: showing %s\n", pat->name);

			if (ctrlc()) {
				puts("vidtest: interrupted\n");
				return CMD_RET_FAILURE;
			}
			mdelay(dwell_ms);
		}
	}

	reported = min(frames, (unsigned int)VIDTEST_MAX_REPORT);
	for (i = 0; i < reported; i++) {
		const struct vidtest_pattern *pat;

		pat = &vidtest_patterns[i % ARRAY_SIZE(vidtest_patterns)];
		printf("vidtest: %-8s %lu us  %lu.%02lu fps  %lu kB/s\n",
		       pat->name, times[i],
		       times[i] ? 1000000 / times[i] : 0,
		       times[i] ? (100000000 / times[i]) % 100 : 0,
		       times[i] ? (ulong)frame_bytes * 1000 / times[i] : 0);
	}
	if (frames > reported)
		printf("vidtest: ... %u more frames not listed\n",
		       frames - reported);

	printf("vidtest: %ux%u %ubpp, %u frames, avg %lu us, %lu.%02lu fps, %lu kB/s\n",
	       priv->xsize, priv->ysize, VNBITS(priv->bpix), frames,
	       total / frames,
	       total ? (ulong)frames * 1000000 / total : 0,
	       total ? ((ulong)frames * 100000000 / total) % 100 : 0,
	       total ? (ulong)frame_bytes * frames * 1000 / total : 0);

	return 0;
}

U_BOOT_CMD(
	vidtest, 3,	1,	do_video_test,
	"cycle full-screen colours and report the frame rate",
	"    [frames] [dwell_ms]\n"
	"      frames   - default 8, one pass over the built-in colours\n"
	"      dwell_ms - hold each colour this long (default 1000) so it can\n"
	"                 be identified; 0 = free-run, for benchmarking\n"
	"      e.g. vidtest        - 8 colours, 1s each, named on the console\n"
	"           vidtest 240 0  - 240-frame benchmark"
);
