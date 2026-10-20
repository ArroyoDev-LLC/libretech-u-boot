/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2016 - Beniamino Galvani <b.galvani@gmail.com>
 */

#ifndef __GX_H__
#define __GX_H__

#ifndef __ASSEMBLY__
#include <linux/bitops.h>
#endif

/*
 * DRAM below BL33 that is handed to Linux as an FDT /memreserve/.
 *
 * The historical value is 16 MiB, which is simply everything below
 * CONFIG_TEXT_BASE (0x01000000) -- a catch-all, not a measured extent. On
 * GXL/GXM the secure range BL31 installs was measured to end at 1 MiB: from
 * u-boot proper a read below 0x00100000 takes a synchronous EXTERNAL abort
 * (esr 0x96000210, DFSC 0x10) while 0x00100000 .. 0x00FFFFFF reads and writes
 * normally, and BL31 + the SCP still answer PSCI after that whole 15 MiB has
 * been overwritten.
 *
 * Measurement + method: amlogic/software/boot/design/dram-reservations.md
 */
#define GX_FIRMWARE_MEM_SIZE	0x100000

/*
 * CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT: the open TF-A's 16 MiB block at DRAM
 * start, with the OPTIONAL OP-TEE band at its top so Linux keeps one
 * contiguous run whether OP-TEE runs or not
 * (amlogic/software/boot/design/libre-secure-block-and-optee.md):
 *
 *   0x00000000 +  1 MiB  ROM mirror      hwrom@0, the SoC .dtsi's own node
 *   0x00100000 +  1 MiB  BL31 + SCP stage secure-firmware@100000, no-map
 *   0x00200000 +  1 MiB  SMC mailboxes   secure-monitor-shmem@200000, no-map
 *   0x00300000 +  1 MiB  ramoops         ramoops@300000, mapped
 *   0x00400000 + 12 MiB  OP-TEE band     optee-core@400000, added at boot
 *                                        only when OP-TEE answers; else Linux
 *
 * These MUST track plat/amlogic/gxl/gxl_def.h in our TF-A fork:
 *   BL31_BASE                 0x00100000  (== _BL31_BASE)
 *   AML_NSDRAM0_BASE          0x00200000  (== _SHARED_BASE)
 *   AML_SHARE_STORAGE_IN_BASE 0x00200000 .. AML_SHARE_MEM_OUTPUT_BASE 0x00207000
 */
#define GX_LIBRE_SEC_BLOCK_BASE		0x00000000
#define GX_LIBRE_SEC_BLOCK_SIZE		0x01000000	/* 16 MiB */
#define GX_LIBRE_BL31_BASE		0x00100000	/* BL31 + SCP staging */
#define GX_LIBRE_BL31_SIZE		0x00100000
#define GX_LIBRE_SHARED_BASE		0x00200000	/* SMC mailboxes, NON-secure */
#define GX_LIBRE_SHARED_SIZE		0x00100000
#define GX_LIBRE_RAMOOPS_BASE		0x00300000
#define GX_LIBRE_RAMOOPS_SIZE		0x00100000
#define GX_LIBRE_OPTEE_BASE		0x00400000	/* OP-TEE TZDRAM, optional */
#define GX_LIBRE_OPTEE_SIZE		0x00C00000

/*
 * Derived extent -- never re-spell it as a literal.
 *
 *  _FW_RSV_SIZE  the part FIRMWARE always owns: ROM mirror, BL31 and the
 *                shared mailboxes, i.e. everything below ramoops, which is the
 *                OS's own storage and is described by its own node.
 */
#define GX_LIBRE_FW_RSV_SIZE	(GX_LIBRE_RAMOOPS_BASE - GX_LIBRE_SEC_BLOCK_BASE)

/*
 * ramoops zone sizing inside the 1 MiB.  Same values as the kernel's
 * arch/arm64/boot/dts/amlogic/meson-libretech-ramoops.dtsi, which sizes the
 * same 1 MiB for the same boards -- not re-derived here.  console is the
 * load-bearing zone (a watchdog bite produces no kmsg_dump).  ecc-size 16 is
 * the kernel's own default (ecc_info.ecc_size ?: 16).
 *
 * The binding requires at least one of record-/console-/ftrace-/pmsg-size to
 * be non-zero -- they all default to 0, so a bare node does not bind.
 *
 * pmsg-size: WITHOUT it pstore/pmsg never registers and /dev/pmsg0 does not
 * exist (fs/pstore/ram.c ramoops_probe(): the pmsg zone is created only "if
 * (cxt->pmsg_size)", and fs/pstore/pmsg.c is driven from there), so userspace
 * cannot write its own persistent record even though the console zone works.
 *
 * The VALUE is not re-derived here either, and it is not taken from the
 * kernel dtsi: meson-libretech-ramoops.dtsi does NOT carry a pmsg-size, and
 * Documentation/devicetree/bindings/reserved-memory/ramoops.yaml gives
 * "default: 0" and no recommended size.  So it is the kernel's own default
 * for this zone -- MIN_MEM_SIZE, fs/pstore/ram.c:
 *
 *   #define MIN_MEM_SIZE 4096UL
 *   static ulong ramoops_pmsg_size = MIN_MEM_SIZE;
 *   module_param_named(pmsg_size, ramoops_pmsg_size, ulong, 0400);
 *
 * -- the same provenance rule ecc-size 16 already follows.  It is a power of
 * two, so ramoops_probe()'s rounddown_pow_of_two() leaves it alone.
 *
 * Arithmetic, in the 1 MiB (fs/pstore/ram.c: dump_mem_sz = size - console -
 * ftrace - pmsg, then max_dump_cnt = dump_mem_sz / record_size):
 *
 *   total                    0x100000  (1 MiB)
 *   - console-size           0x080000  (512 KiB)
 *   - ftrace-size            0x000000  (not set)
 *   - pmsg-size              0x001000  (4 KiB)
 *   = dump area              0x07F000  (508 KiB)
 *   / record-size 0x20000  =        3  dump records, 0x01F000 slack
 *
 * It fits.  The cost is honest and worth stating: the dump-record count goes
 * 4 -> 3, because ANY non-zero pmsg-size smaller than a record still consumes
 * a whole 128 KiB record slot to rounding.  If 4 KiB of userspace log ever
 * proves too small, the free move is 0x20000 (fs/pstore/ram.c's own DT
 * fallback sets pmsg_size = record_size) -- it lands on the SAME 3 records
 * with zero slack, not on 2.
 */
#define GX_LIBRE_RAMOOPS_CONSOLE_SIZE	0x00080000
#define GX_LIBRE_RAMOOPS_RECORD_SIZE	0x00020000
#define GX_LIBRE_RAMOOPS_PMSG_SIZE	0x00001000
#define GX_LIBRE_RAMOOPS_ECC_SIZE	16

#define GX_MB_SRAM_BASE		0xd9013800
#define GX_AOBUS_BASE		0xc8100000
#define GX_SEC_HIU_MB_BASE	0xda83c400
#define GX_SEC_AOBUS_BASE	0xda100000
#define GX_PERIPHS_BASE	0xc8834400
#define GX_HIU_BASE		0xc883c000
#define GX_ETH_BASE		0xc9410000
#define GX_DOS_BASE		0xc8820000

/*
 * DOS_SCRATCH31 -- the DOS-block capability word the video firmware reads.
 *
 * Byte offset 0xFCBC off the DOS base = AMRISC control register 0x3f2f << 2.
 *
 * The NAME is cited to a VENDOR-authored map, not to our own tree: BSP
 * A311D_Linux2603V2.5 media_modules drivers/include/regs/dos_registers_v1.h:421
 * "#define DOS_SCRATCH31  0x3f2f" (and dos_registers_v2.h:1810, same value).
 * `software/tools/bsp-reverse-index.py C882FCBC` names no register at this
 * ADDRESS -- the vendor address->name maps it indexes cover the AOBUS/HIU/DMC
 * blocks, not the DOS AMRISC control-register file -- and reports only the
 * enclosing block, "in block DOS (s805x-reference.txt:35)", which is what
 * pins GX_DOS_BASE.  So the address is the datasheet's, the name is the BSP
 * header's, and the CR<<2 mapping is the one dos_regs.h:186 states.
 *
 * Linux drivers/staging/media/meson/vdec/dos_regs.h (linux-6.18.y-lc
 * 6b61da19ad0c) :183 calls it "GXL-family capability register written by
 * U-Boot" -- which is what this stage is doing -- :189 defines the same
 * 0xFCBC, and :190 names BIT(3) the S805X H.264 1080p cap, which is why the
 * store below is a read-modify-write.
 *
 * BIT(2) gates VP9: gxl_vp9_mmu.bin parks at word 0x92f on
 * "mfcr r43, 0x3f2f; bnandi r43, 4, -1", i.e. spin until BIT(2) is SET, and
 * never reads a byte of bitstream until it is.  gxl_hevc_mmu.bin reads the
 * same word at 0x7c8 and spins until BIT(0) CLEARS, which an all-zero word
 * satisfies at once -- so a chain that leaves the word zero decodes HEVC and
 * starves VP9 on the same engine, in the same boot.  The vendor's own BL2
 * leaves 0x00000004 here; ours left 0x00000000 until this write.
 * Measured: amlogic/software/boot/gxl/audit/2026-09-06-gxl-n95-vp9-dos-scratch31.md
 */
#define GX_DOS_SCRATCH31		(GX_DOS_BASE + 0xfcbc)
#define GX_DOS_SCRATCH31_VP9_CAP	BIT(2)

/* Always-On Peripherals registers */
#define GX_AO_ADDR(off)	(GX_AOBUS_BASE + ((off) << 2))

#define GX_AO_SEC_GP_CFG0	GX_AO_ADDR(0x90)
#define GX_AO_SEC_GP_CFG3	GX_AO_ADDR(0x93)
#define GX_AO_SEC_GP_CFG4	GX_AO_ADDR(0x94)
#define GX_AO_SEC_GP_CFG5	GX_AO_ADDR(0x95)
#define GX_AO_SEC_SD_CFG15	GX_AO_ADDR(0x8f)

#define GX_SEC_AO_ADDR(off)	(GX_SEC_AOBUS_BASE + ((off) << 2))
#define GX_SEC_AO_SEC_GP_CFG0	GX_SEC_AO_ADDR(0x90)

#define GX_AO_BOOT_DEVICE	0xF
#define GX_AO_MEM_SIZE_MASK	0xFFFF0000
#define GX_AO_MEM_SIZE_SHIFT	16
#define GX_AO_BL31_RSVMEM_SIZE_MASK	0xFFFF0000
#define GX_AO_BL31_RSVMEM_SIZE_SHIFT	16
#define GX_AO_BL32_RSVMEM_SIZE_MASK	0xFFFF

/* Peripherals registers */
#define GX_PERIPHS_ADDR(off)	(GX_PERIPHS_BASE + ((off) << 2))

/* GPIO registers 0 to 6 */
#define _GX_GPIO_OFF(n)	((n) == 6 ? 0x08 : 0x0c + 3 * (n))
#define GX_GPIO_EN(n)		GX_PERIPHS_ADDR(_GX_GPIO_OFF(n) + 0)
#define GX_GPIO_IN(n)		GX_PERIPHS_ADDR(_GX_GPIO_OFF(n) + 1)
#define GX_GPIO_OUT(n)	GX_PERIPHS_ADDR(_GX_GPIO_OFF(n) + 2)

/* Mailbox registers */
#define GX_SEC_HIU_MB_ADDR(off)	(GX_SEC_HIU_MB_BASE + ((off) << 2))
#define GX_SEC_HIU_MAILBOX_SET_0	GX_SEC_HIU_MB_ADDR(0x01)
#define GX_SEC_HIU_MAILBOX_STAT_0	GX_SEC_HIU_MB_ADDR(0x02)
#define GX_SEC_HIU_MAILBOX_CLR_0	GX_SEC_HIU_MB_ADDR(0x03)

/* Mailbox commands */
#define GX_MB_CMD_SHA	0xc0de0001
#define GX_MB_CMD_DATA	0xc0dec0de
#define GX_MB_CMD_END	0xe00de00d
#define GX_MB_CMD_OP_SHA	0xc0de0002
#define GX_MB_CMD_DATA_LEN	0xc0dec0d0

/* PIN_MUX registers */
#define GX_PIN_MUX_REG1		(0xda834400 + (0x2d << 2))
#define GX_PIN_MUX_REG2		(0xda834400 + (0x2e << 2))
#define GX_PIN_MUX_REG3		(0xda834400 + (0x2f << 2))
#define GX_PIN_MUX_REG7		(0xda834400 + (0x33 << 2))

/* PWM registers */
#define GX_PWM_PWM_B	(0xc1100000 + (0x2155 << 2))
#define GX_PWM_PWM_D	(0xc1100000 + (0x2195 << 2))
#define GX_PWM_MISC_REG_CD	(0xc1100000 + (0x2196 << 2))
#define GX_PWM_MISC_REG_AB	(0xc1100000 + (0x2156 << 2))

/* Non-DM MMC init */
#if CONFIG_IS_ENABLED(MMC) && !CONFIG_IS_ENABLED(DM_MMC)
struct mmc *meson_mmc_init(int mmc_no);
#endif

#if !CONFIG_IS_ENABLED(WDT_MESON_GXBB) && defined(CONFIG_XPL_BUILD)
#define GX_WDT_CTRL_REG		0xc11098d0
#endif

#endif /* __GX_H__ */
