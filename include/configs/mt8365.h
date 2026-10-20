/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuration for MT8365 (Genio 350) based boards
 *
 * Modelled on mt8183.h (single-cluster Genio sibling). Adds the bootflow
 * load addresses so BOOTSTD's EFI / extlinux bootmeths can stage grub +
 * kernel from the eMMC ESP/rootfs at the shared libre DRAM load map
 * offsets (DRAM base 0x40000000).
 */

#ifndef __MT8365_H
#define __MT8365_H

#include <linux/sizes.h>

#define CFG_SYS_NS16550_COM1		0x11002000
#define CFG_SYS_NS16550_CLK		26000000

/*
 * Boot via BOOTSTD bootflow (the EFI + extlinux bootmeths). Do NOT include
 * <config_distro_bootcmd.h>: its for-loop distro_bootcmd needs the hush
 * parser (not enabled) and overrides the BOOTSTD default bootcmd, so autoboot
 * failed with "Unknown command 'for'". bootflow is the boot path; these env
 * vars just give the bootmeths their staging addresses.
 */
/*
 * Base env, shared with the upstream mt8365-evk. Board headers that need to
 * extend it (mt8365_pumpkin.h adds fdtfile) reuse CFG_MT8365_BASE_ENV so the
 * EVK's expansion stays byte-identical.
 *
 * The nine variables are the MT8365 row of the libre DRAM load map, the
 * shared offsets plus the 0x40000000 DRAM base:
 * design/rules/bootloader/dram-load-map.md. They are the seven slots, the
 * loadaddr alias, and kernel_comp_size. EMI-MPU closes only 0x43000000
 * (192 KiB) and 0x43200000 (10 MiB), so the slots start above the secure
 * pack at 0x44000000. loadaddr is restated here because env_default.h emits
 * it from CONFIG_SYS_LOAD_ADDR (the text base) earlier in
 * default_environment and the later entry wins; the map's alias is the
 * kernel slot. initrd_high=~0 boots the initramfs in its slot instead of
 * copying it below U-Boot.
 */
#define CFG_MT8365_BASE_ENV \
	"scriptaddr=0x44000000\0" \
	"pxefile_addr_r=0x44100000\0" \
	"fdtoverlay_addr_r=0x44200000\0" \
	"fdt_addr_r=0x44300000\0" \
	"kernel_addr_r=0x44600000\0" \
	"loadaddr=0x44600000\0" \
	"kernel_comp_addr_r=0x48600000\0" \
	"kernel_comp_size=0x4000000\0" \
	"ramdisk_addr_r=0x4d600000\0" \
	"initrd_high=0xffffffffffffffff\0"

#define CFG_EXTRA_ENV_SETTINGS \
	CFG_MT8365_BASE_ENV

#endif
