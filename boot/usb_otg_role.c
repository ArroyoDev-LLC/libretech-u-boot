// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * The OTG port's role, chosen in the environment and handed to the kernel.
 *
 * `usb_otg=device|host|otg` (unset: the device tree's own dr_mode stands).
 * The board's control FDT names its OTG controller with the alias
 * `usb-otg`; usb_otg_fdt_fixup() writes the matching dr_mode on the node
 * at the same path in the FDT being booted, whatever supplied it
 * (built-in, /lbs override, PXE, a distro). boot.ini, the NOR config
 * sector or a saved environment set it with a plain setenv.
 *
 * It is called from the board's ft_board_setup(), which image_setup_libfdt()
 * runs on every board (EVT_FT_FIXUP is not notified when U-Boot runs on a
 * live tree, which these boards do).
 *
 * Failure modes: an unknown value, a board with no `usb-otg` alias, or a
 * kernel FDT without that node is reported and 0 is returned, so the boot
 * goes on with the device tree as it was; a failed write into the FDT is
 * returned as the libfdt error, for the board's ft_board_setup() to treat
 * as it treats its own fixups.
 */

#include <env.h>
#include <stdio.h>
#include <usb_otg_role.h>
#include <dm/ofnode.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/string.h>

#define USB_OTG_ALIAS	"usb-otg"

static const struct {
	const char *env;
	const char *dr_mode;
} usb_otg_roles[] = {
	{ "device",	"peripheral" },
	{ "host",	"host" },
	{ "otg",	"otg" },
};

static const char *usb_otg_dr_mode(const char *val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(usb_otg_roles); i++)
		if (!strcmp(val, usb_otg_roles[i].env))
			return usb_otg_roles[i].dr_mode;

	return NULL;
}

int usb_otg_fdt_fixup(void *blob)
{
	const char *val, *dr_mode;
	char path[128];
	ofnode node;
	int off, ret;

	val = env_get(USB_OTG_ENV);
	if (!val || !*val)
		return 0;

	dr_mode = usb_otg_dr_mode(val);
	if (!dr_mode) {
		printf("usb_otg: '%s' is not device, host or otg; device tree left as it is\n",
		       val);
		return 0;
	}

	node = ofnode_get_aliases_node(USB_OTG_ALIAS);
	if (!ofnode_valid(node)) {
		printf("usb_otg: this board names no OTG controller (alias %s); device tree left as it is\n",
		       USB_OTG_ALIAS);
		return 0;
	}

	ret = ofnode_get_path(node, path, sizeof(path));
	if (ret) {
		printf("usb_otg: OTG controller path unreadable (%d); device tree left as it is\n",
		       ret);
		return 0;
	}

	off = fdt_path_offset(blob, path);
	if (off < 0) {
		printf("usb_otg: %s is not in the kernel device tree; left as it is\n",
		       path);
		return 0;
	}

	ret = fdt_setprop_string(blob, off, "dr_mode", dr_mode);
	if (ret) {
		printf("usb_otg: %s dr_mode not written: %s\n", path,
		       fdt_strerror(ret));
		return ret;
	}

	printf("usb_otg: %s dr_mode = %s\n", path, dr_mode);

	return 0;
}
