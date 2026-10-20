/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for the OLogic Pumpkin i350 (MT8365).
 *
 * Separate from mt8365.h because that header is shared with the upstream
 * mt8365-evk, a board we do not build and must not drift. Everything here is
 * additive to the shared base.
 */

#ifndef __MT8365_PUMPKIN_H
#define __MT8365_PUMPKIN_H

#include <configs/mt8365.h>

/*
 * fdtfile is the ONLY input to efi_get_distro_fdt_name() (lib/efi_loader/efi_fdt.c):
 * it builds "<prefix>/<fdtfile>" for prefix in /dtb, "", /dtb/current, /dtbs. Unset
 * on arm64 it returns -ENOENT for every prefix, so bootmeth_efi sets
 * BOOTFLOWF_USE_BUILTIN_FDT and hands Linux u-boot's CONTROL FDT -- which carries the
 * mt8365-pumpkin-u-boot.dtsi USB surgery (dr_mode = "peripheral", no usb-role-switch,
 * no nested usb_host). Defining it lets seq 0 resolve the deployed kernel's own DTB
 * at the ESP path /dtb/mt8365-pumpkin.dtb, so the OS device tree tracks the deployed
 * kernel. If that file is absent the -ENOENT fallback to the control FDT is unchanged,
 * so this cannot make the board unbootable.
 *
 * CONFIG_DEFAULT_FDT_FILE is the bare basename ("mt8365-pumpkin.dtb") because
 * efi_fdt.c supplies the "/dtb" prefix itself.
 */
#undef CFG_EXTRA_ENV_SETTINGS
#define CFG_EXTRA_ENV_SETTINGS \
	CFG_MT8365_BASE_ENV \
	"fdtfile=" CONFIG_DEFAULT_FDT_FILE "\0"

#endif
