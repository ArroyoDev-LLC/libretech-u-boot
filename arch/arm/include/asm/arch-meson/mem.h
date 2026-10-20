/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#ifndef __MESON_MEM_H__
#define __MESON_MEM_H__

#include <linux/types.h>

/* Configure the reserved memory zones exported by the secure registers
 * into EFI and DTB reserved memory entries.
 */
void meson_board_add_reserved_memory(void *fdt, u64 start, u64 size);
void meson_init_reserved_memory(void *fdt);

/*
 * CONFIG_MESON_GX_LIBRE_SCMI_FIXUP: flip the shipping DTB's disabled SCMI
 * package on for the open SC.  Runs from ft_board_setup() on the blob handed
 * to Linux.  Never fails the boot: returns 0 on a blob without the carrier.
 */
int meson_gx_libre_scmi_fixup(void *fdt);

/*
 * The &sram child that hands Linux the SC's PUB-telemetry block.  Two places
 * touch it and they have to agree: board-common.c shuts it on every meson
 * build, as the first write to the blob, and the SCMI fixup above is the only
 * thing allowed to open it again -- on a live PUBMON_GET_BLOCK answer, never on
 * anything stored in SRAM.  One name, because a node that is shut by a
 * different string than the one the gate opens is a node that is never open.
 */
#define MESON_PUBMON_COMPAT		"amlogic,gxl-pubmon-sc-shmem"

/*
 * The same guard, applied to U-Boot's OWN control FDT from board_fix_fdt() in
 * board_init_f -- before DM binds U-Boot's SCMI agent, whose probe reads the
 * SC's shared-memory window and aborts if no SC ever opened it.  With no SC it
 * removes /firmware/scmi so the agent is never bound; with an SC it changes
 * nothing.  Never fails the boot: returns 0 on a blob without the node.
 */
int meson_gx_libre_scmi_self_guard(void *fdt);

/*
 * CONFIG_MESON_G12A_LIBRE_SCMI_FIXUP: the SM1 twins of the two above, for
 * the -libre DTB's disabled meson-sm1-libre-scmi.dtsi package (board-g12a.c).
 */
int meson_g12a_libre_scmi_fixup(void *fdt);
int meson_g12a_libre_scmi_self_guard(void *fdt);

#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT) || \
    defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT)
/*
 * /reserved-memory surgery for the open TF-A's DRAM-start secure block,
 * implemented once in board-common.c and used by board-gx.c (GXL/GXM) and
 * board-g12a.c (G12A/G12B/SM1).  Both families use the same 16 MiB map with
 * the mailboxes at 0x00200000 and ramoops at 0x00300000; only the node set
 * differs between the two.
 */
void meson_del_node(void *fdt, const char *path);
void meson_del_ramoops_nodes(void *fdt);
int meson_add_rsv_node(void *fdt, const char *name, u32 base, u32 size,
		       bool nomap, const char *compat);

/*
 * The optional OP-TEE band at the top of the block: described to the OS only
 * when OP-TEE answers OPTEE_SMC_CALLS_UID on this boot (board-common.c).
 * U-Boot proper only; SPL hands no DT to an OS.
 */
#if defined(CONFIG_XPL_BUILD)
static inline void meson_libre_optee_fixup(void *fdt) {}
#else
void meson_libre_optee_fixup(void *fdt);
/* Hand an OP-TEE image (v2 header + image) in memory to TF-A's opteed. */
int meson_libre_optee_load(ulong addr, ulong size);
#endif
#endif

#endif /* __MESON_MEM_H__ */
