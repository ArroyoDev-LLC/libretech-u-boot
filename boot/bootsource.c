// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * The boot source, spelled once for every SoC family.
 *
 * See include/bootsource.h. This file owns the vocabulary; the SoC families
 * own the detection (arch/arm/mach-<soc>) and the board code owns the
 * decision of which MMC device the environment follows (board/...).
 *
 * Built in every phase: SPL calls bootsource_announce(); U-Boot proper calls
 * bootsource_publish() and carries the EVT_FT_FIXUP spy that writes the
 * kernel's /chosen.
 */

#include <bootsource.h>
#include <env.h>
#include <event.h>
#include <stdio.h>
#include <vsprintf.h>
#include <fdt_support.h>
#include <dm/ofnode.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/string.h>

static const struct {
	const char *name;
	const char *token;
} bootsources[BOOTSOURCE_COUNT] = {
	[BOOTSOURCE_UNKNOWN]	= { "unknown",	"unknown" },
	[BOOTSOURCE_SD]		= { "SD",	"sd" },
	[BOOTSOURCE_EMMC]	= { "eMMC",	"emmc" },
	[BOOTSOURCE_UFS]	= { "UFS",	"ufs" },
	[BOOTSOURCE_SPI]	= { "SPI NOR",	"spi" },
	[BOOTSOURCE_NAND]	= { "NAND",	"nand" },
	[BOOTSOURCE_USB]	= { "USB",	"usb" },
	[BOOTSOURCE_NET]	= { "network",	"net" },
	[BOOTSOURCE_RAM]	= { "RAM",	"ram" },
};

const char *bootsource_token(enum bootsource src)
{
	if (src < 0 || src >= BOOTSOURCE_COUNT)
		src = BOOTSOURCE_UNKNOWN;

	return bootsources[src].token;
}

const char *bootsource_name(enum bootsource src)
{
	if (src < 0 || src >= BOOTSOURCE_COUNT)
		src = BOOTSOURCE_UNKNOWN;

	return bootsources[src].name;
}

enum bootsource bootsource_from_token(const char *token)
{
	int i;

	if (!token)
		return BOOTSOURCE_UNKNOWN;

	for (i = 0; i < BOOTSOURCE_COUNT; i++)
		if (!strcmp(bootsources[i].token, token))
			return i;

	return BOOTSOURCE_UNKNOWN;
}

void bootsource_announce(enum bootsource src)
{
	/*
	 * .data, not .bss: an SPL calls this from board_init_f(), before
	 * .bss is usable on several of these SoCs.
	 */
	static bool announced __section(".data");

	if (announced)
		return;
	announced = true;

	printf("Boot source: %s\n", bootsource_name(src));
}

#ifndef CONFIG_XPL_BUILD
static enum bootsource published_source;

void bootsource_publish(enum bootsource src, int mmc_dev)
{
	published_source = src;

	env_set(BOOTSOURCE_ENV, bootsource_token(src));
	if (mmc_dev >= 0)
		env_set_ulong(BOOTSOURCE_DEV_ENV, mmc_dev);

	bootsource_announce(src);
}

enum bootsource bootsource_get(void)
{
	return published_source;
}

/*
 * The same fact for the kernel: /chosen/libre-computer,boot-source in the
 * FDT being booted, whatever supplied that FDT (built-in, /lbs override,
 * PXE, a distro), so an initramfs or flash-kernel can read where the
 * firmware came from without parsing a U-Boot environment.
 *
 * Two routes to the same write.  image_setup_libfdt() notifies EVT_FT_FIXUP
 * only when U-Boot is NOT on a live tree, so the spy below covers the
 * flat-tree boards; a board on a live tree (OF_LIVE: Meson, RK3399) calls
 * bootsource_fdt_fixup() from its ft_board_setup().  The two never both run.
 */
int bootsource_fdt_fixup(void *blob)
{
	int off;

	off = fdt_find_or_add_subnode(blob, 0, "chosen");
	if (off < 0)
		return off;

	return fdt_setprop_string(blob, off, BOOTSOURCE_PROP,
				  bootsource_token(published_source));
}

#if CONFIG_IS_ENABLED(EVENT)
static int bootsource_ft_fixup(void *ctx, struct event *event)
{
	oftree tree = event->data.ft_fixup.tree;
	ofnode chosen;
	int ret;

	chosen = oftree_path(tree, "/chosen");
	if (!ofnode_valid(chosen)) {
		ret = ofnode_add_subnode(oftree_root(tree), "chosen", &chosen);
		if (ret)
			return log_msg_ret("chosen", ret);
	}

	ret = ofnode_write_string(chosen, BOOTSOURCE_PROP,
				  bootsource_token(published_source));
	if (ret)
		return log_msg_ret("prop", ret);

	return 0;
}
EVENT_SPY_FULL(EVT_FT_FIXUP, bootsource_ft_fixup);
#endif
#endif /* !CONFIG_XPL_BUILD */
