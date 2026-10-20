/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#ifndef __G12A_H__
#define __G12A_H__

#ifndef __ASSEMBLY__
#include <linux/bitops.h>
#endif

/*
 * CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT: the open TF-A's secure block at DRAM
 * start on G12A/G12B/SM1.  It is the GXL block of arch-meson/gx.h EXACTLY --
 * same 16 MiB, same addresses -- decided by the operator 2026-09-26 and
 * reordered 2026-10-17 to put the optional OP-TEE band at the top.  An
 * earlier revision moved ramoops to the BOTTOM megabyte and handed the top
 * one back (measured 2026-08-21, A311D
 * and S905D3: this family has no ROM mirror at 0x00000000, so the megabyte
 * GXL must reserve for hardware this family could actually use).  That saved
 * one megabyte per board and cost decision A5's property -- ramoops had one
 * address across all families -- forcing every pstore-aware tool to become
 * family-aware.  The map is uniform again.
 *
 *   0x00000000 +  1 MiB  NS reserved    empty -- NOT a ROM mirror here; held
 *                                        only to keep the low map GXL's
 *   0x00100000 +  1 MiB  S-only, EL3    BL31
 *   0x00200000 +  1 MiB  S<->NS shared  SMC shared mem + storage mailboxes
 *   0x00300000 +  1 MiB  NS reserved    ramoops
 *   0x00400000 + 12 MiB  OP-TEE band    added at boot only when OP-TEE
 *                                        answers; else Linux (operator,
 *                                        2026-10-17: libre-secure-block-and-
 *                                        optee.md)
 *   0x01000000 ->        NS free        BL33 (CONFIG_TEXT_BASE), then Linux
 *
 * The cost of uniformity, stated rather than hidden: on GXL the first
 * megabyte is unusable anyway (hwrom@0) so its reservation is free; here it
 * is ordinary allocator-owned DRAM and the block reserves it for nothing.
 * See amlogic/software/boot/design/dram-reservations.md section 3 for the
 * measurement and section 10i for the trade as it stood before this decision.
 *
 * These MUST track plat/amlogic/{g12a,g12b}/{g12a,g12b}_def.h in our TF-A:
 *   BL31_BASE                 0x00100000  (inside the block)
 *   AML_NSDRAM0_BASE          0x00200000  (== _SHARED_BASE)
 *   AML_SHARE_STORAGE_IN_BASE 0x00200000 .. AML_SHARE_MEM_OUTPUT_BASE 0x00207000
 *
 * There is NO SCP staging constant here, unlike GXL.  On this family SPL, not
 * BL31, starts the SC (arch/arm/mach-meson/spl-g12a-scp.c copies the FIT "scp"
 * image into AHB SRAM), so the FIT scp load address is an SPL-lifetime scratch
 * buffer that needs no reservation and no TF-A counterpart -- and it stays at
 * 0x013C0000, outside the block.
 */
#define G12A_LIBRE_BLOCK_BASE		0x00000000
#define G12A_LIBRE_RAMOOPS_BASE		0x00300000
#define G12A_LIBRE_RAMOOPS_SIZE		0x00100000
#define G12A_LIBRE_SEC_BASE		0x00100000	/* BL31 */
#define G12A_LIBRE_SHARED_BASE		0x00200000	/* SMC mailboxes, NON-secure */
#define G12A_LIBRE_SHARED_SIZE		0x00100000
#define G12A_LIBRE_OPTEE_BASE		0x00400000	/* OP-TEE TZDRAM, optional */
#define G12A_LIBRE_OPTEE_SIZE		0x00C00000
#define G12A_LIBRE_BLOCK_LIMIT		0x01000000	/* first free byte above */

/*
 * Derived extents -- never re-spell these as literals.
 *
 *  _SECURE_SIZE  the no-map node from DRAM start up to the mailboxes, 2 MiB:
 *                BL31 and -- unlike GXL, where that space is the ROM mirror --
 *                one megabyte of nothing.
 *  _FW_RSV_SIZE  the part FIRMWARE always owns: secure + the shared
 *                mailboxes, i.e. everything below ramoops, which is the OS's
 *                own storage and is described by its own node.  It derives
 *                from _BLOCK_BASE exactly as GX_LIBRE_FW_RSV_SIZE does, so the
 *                two families' reservations are the same expression.
 */
#define G12A_LIBRE_SECURE_SIZE	(G12A_LIBRE_SHARED_BASE - G12A_LIBRE_BLOCK_BASE)
#define G12A_LIBRE_FW_RSV_BASE	G12A_LIBRE_BLOCK_BASE
#define G12A_LIBRE_FW_RSV_SIZE	(G12A_LIBRE_RAMOOPS_BASE - G12A_LIBRE_BLOCK_BASE)

/*
 * ramoops zone sizing inside the 1 MiB.  Same values as the kernel's
 * arch/arm64/boot/dts/amlogic/meson-libretech-ramoops.dtsi and as GXL's
 * GX_LIBRE_RAMOOPS_* -- deliberately NOT re-derived, because the zone split is
 * a property of the megabyte, not of where the megabyte sits.
 */
#define G12A_LIBRE_RAMOOPS_CONSOLE_SIZE	0x00080000
#define G12A_LIBRE_RAMOOPS_RECORD_SIZE	0x00020000
#define G12A_LIBRE_RAMOOPS_PMSG_SIZE	0x00001000
#define G12A_LIBRE_RAMOOPS_ECC_SIZE	16

#define G12A_AOBUS_BASE			0xff800000
#define G12A_PERIPHS_BASE		0xff634400
#define G12A_HIU_BASE			0xff63c000
#define G12A_ETH_PHY_BASE		0xff64c000
#define G12A_ETH_BASE			0xff3f0000

/* Always-On Peripherals registers */
#define G12A_AO_ADDR(off)	(G12A_AOBUS_BASE + ((off) << 2))

#define G12A_AO_SEC_GP_CFG0		G12A_AO_ADDR(0x90)
#define G12A_AO_SEC_GP_CFG3		G12A_AO_ADDR(0x93)
#define G12A_AO_SEC_GP_CFG4		G12A_AO_ADDR(0x94)
#define G12A_AO_SEC_GP_CFG5		G12A_AO_ADDR(0x95)

#define G12A_AO_BOOT_DEVICE		0xF
#define G12A_AO_MEM_SIZE_MASK		0xFFF80000
#define G12A_AO_MEM_SIZE_SHIFT		16
#define G12A_AO_BL31_RSVMEM_SIZE_MASK	0xFFFF0000
#define G12A_AO_BL31_RSVMEM_SIZE_SHIFT	16
#define G12A_AO_BL32_RSVMEM_SIZE_MASK	0xFFFF

#endif /* __G12A_H__ */
