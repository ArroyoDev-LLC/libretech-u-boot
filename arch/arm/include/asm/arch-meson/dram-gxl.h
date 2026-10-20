/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015, Amlogic, Inc. All rights reserved.
 * Copyright (C) 2023-2025, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#ifndef DRAM_GXL_H
#define DRAM_GXL_H

/* PUB (not much documented) */
#define DDR0_PUB_REG_BASE	0xc8836000

#define DDR0_PUB_PIR	(DDR0_PUB_REG_BASE + (0x01 << 2))

/* PGCR0 @ PUB 0x04 (before PGCR1@0x05); ACS t_pub_pgcr0=0x07d81e3f */
#define DDR0_PUB_PGCR0	(DDR0_PUB_REG_BASE + (0x04 << 2))
#define DDR0_PUB_PGCR1	(DDR0_PUB_REG_BASE + (0x05 << 2))
#define DDR0_PUB_PGCR2	(DDR0_PUB_REG_BASE + (0x06 << 2))
#define DDR0_PUB_PGCR3	(DDR0_PUB_REG_BASE + (0x07 << 2))
#define DDR0_PUB_PGCR4	(DDR0_PUB_REG_BASE + (0x08 << 2))
#define DDR0_PUB_PGCR5	(DDR0_PUB_REG_BASE + (0x09 << 2))
#define DDR0_PUB_PGCR6	(DDR0_PUB_REG_BASE + (0x0a << 2))
#define DDR0_PUB_PGSR0	(DDR0_PUB_REG_BASE + (0x0d << 2))

#define DDR0_PUB_ACLCDLR	(DDR0_PUB_REG_BASE + (0x160 << 2))
#define DDR0_PUB_ACMDLR0	(DDR0_PUB_REG_BASE + (0x168 << 2))
#define DDR0_PUB_ACBDLR0	(DDR0_PUB_REG_BASE + (0x150 << 2))
#define DDR0_PUB_ACBDLR1	(DDR0_PUB_REG_BASE + (0x151 << 2))
#define DDR0_PUB_ACBDLR2	(DDR0_PUB_REG_BASE + (0x152 << 2))
#define DDR0_PUB_ACBDLR3	(DDR0_PUB_REG_BASE + (0x153 << 2))
#define DDR0_PUB_ACBDLR4	(DDR0_PUB_REG_BASE + (0x154 << 2))
#define DDR0_PUB_ACBDLR5	(DDR0_PUB_REG_BASE + (0x155 << 2))
#define DDR0_PUB_ACBDLR6	(DDR0_PUB_REG_BASE + (0x156 << 2))
#define DDR0_PUB_ACBDLR7	(DDR0_PUB_REG_BASE + (0x157 << 2))
#define DDR0_PUB_ACBDLR8	(DDR0_PUB_REG_BASE + (0x158 << 2))
#define DDR0_PUB_ACBDLR9	(DDR0_PUB_REG_BASE + (0x159 << 2))
#define DDR0_PUB_ACIOCR1	(DDR0_PUB_REG_BASE + (0x141 << 2))
#define DDR0_PUB_ACIOCR2	(DDR0_PUB_REG_BASE + (0x142 << 2))
#define DDR0_PUB_ACIOCR3	(DDR0_PUB_REG_BASE + (0x143 << 2))
#define DDR0_PUB_ACIOCR4	(DDR0_PUB_REG_BASE + (0x144 << 2))
#define DDR0_PUB_ACIOCR5	(DDR0_PUB_REG_BASE + (0x145 << 2))

#define DDR0_PUB_PTR3	(DDR0_PUB_REG_BASE + (0x13 << 2))
#define DDR0_PUB_PTR4	(DDR0_PUB_REG_BASE + (0x14 << 2))

#define DDR0_PUB_DXCCR	(DDR0_PUB_REG_BASE + (0x22 << 2))
#define DDR0_PUB_DSGCR	(DDR0_PUB_REG_BASE + (0x24 << 2))
#define DDR0_PUB_DCR	(DDR0_PUB_REG_BASE + (0x40 << 2))

#define DDR0_PUB_DTPR0	(DDR0_PUB_REG_BASE + (0x44 << 2))
#define DDR0_PUB_DTPR1	(DDR0_PUB_REG_BASE + (0x45 << 2))
#define DDR0_PUB_DTPR2	(DDR0_PUB_REG_BASE + (0x46 << 2))
#define DDR0_PUB_DTPR3	(DDR0_PUB_REG_BASE + (0x47 << 2))
#define DDR0_PUB_DTPR4	(DDR0_PUB_REG_BASE + (0x48 << 2))
#define DDR0_PUB_DTPR5	(DDR0_PUB_REG_BASE + (0x49 << 2))

#define DDR0_PUB_MR0	(DDR0_PUB_REG_BASE + (0x60 << 2))
#define DDR0_PUB_MR1	(DDR0_PUB_REG_BASE + (0x61 << 2))
#define DDR0_PUB_MR2	(DDR0_PUB_REG_BASE + (0x62 << 2))
#define DDR0_PUB_MR3	(DDR0_PUB_REG_BASE + (0x63 << 2))
#define DDR0_PUB_MR4	(DDR0_PUB_REG_BASE + (0x64 << 2))
#define DDR0_PUB_MR5	(DDR0_PUB_REG_BASE + (0x65 << 2))
#define DDR0_PUB_MR6	(DDR0_PUB_REG_BASE + (0x66 << 2))
#define DDR0_PUB_MR11	(DDR0_PUB_REG_BASE + (0x6b << 2))

#define DDR0_PUB_ODTCR	(DDR0_PUB_REG_BASE + (0x26 << 2))

#define DDR0_PUB_DTCR	(DDR0_PUB_REG_BASE + (0x80 << 2))
#define DDR0_PUB_DTCR1	(DDR0_PUB_REG_BASE + (0x81 << 2))

#define DDR0_PUB_DTAR0	(DDR0_PUB_REG_BASE + (0x82 << 2))
#define DDR0_PUB_DTAR1	(DDR0_PUB_REG_BASE + (0x83 << 2))
#define DDR0_PUB_DTAR2	(DDR0_PUB_REG_BASE + (0x84 << 2))
#define DDR0_PUB_DTAR3	(DDR0_PUB_REG_BASE + (0x85 << 2))
/*
 * Product bist post @0x48cc writes PUB 0x86..0x89 (after DTAR).  No vendor
 * file names these four words, so they carry no name here.
 */
#define DDR0_PUB_UNNAMED_0X86	(DDR0_PUB_REG_BASE + (0x86 << 2))
#define DDR0_PUB_UNNAMED_0X87	(DDR0_PUB_REG_BASE + (0x87 << 2))
#define DDR0_PUB_UNNAMED_0X88	(DDR0_PUB_REG_BASE + (0x88 << 2))
#define DDR0_PUB_UNNAMED_0X89	(DDR0_PUB_REG_BASE + (0x89 << 2))
/*
 * Data Training Eye Data Registers, per the vendor's cmd_ddr_test.c
 * (A311D_Linux2603V2.5 bl33/v2019/cmd/amlogic/cmd_ddr_test.c:1297-1299):
 * DTEDR0 0x08c, DTEDR1 0x08d, DTEDR2 0x08e.  They used to be named at
 * 0x88/0x89 here; under a DTCR0 rank/lane select it is 0x8c..0x8e that
 * change (claude amlogic n1212).
 */
#define DDR0_PUB_DTEDR0	(DDR0_PUB_REG_BASE + (0x8c << 2))
#define DDR0_PUB_DTEDR1	(DDR0_PUB_REG_BASE + (0x8d << 2))
#define DDR0_PUB_DTEDR2	(DDR0_PUB_REG_BASE + (0x8e << 2))

#define DDR0_PUB_RANKIDR	(DDR0_PUB_REG_BASE + (0x137 << 2))

#define DDR0_PUB_IOVCR0	(DDR0_PUB_REG_BASE + (0x148 << 2))
#define DDR0_PUB_IOVCR1	(DDR0_PUB_REG_BASE + (0x149 << 2))

#define DDR0_PUB_VTCR0	(DDR0_PUB_REG_BASE + (0x14a << 2))
#define DDR0_PUB_VTCR1	(DDR0_PUB_REG_BASE + (0x14b << 2))

#define DDR0_PUB_ZQCR	(DDR0_PUB_REG_BASE + (0x1a0 << 2))

#define DDR0_PUB_ZQ0PR	(DDR0_PUB_REG_BASE + (0x1a1 << 2))
#define DDR0_PUB_ZQ1PR	(DDR0_PUB_REG_BASE + (0x1a5 << 2))

/*
 * ZQ calibration RESULT registers, i.e. what the PHY actually converged to as
 * opposed to what PUB_ZQ0PR/ZQ1PR asked for.  Nothing in this tree has ever
 * read them, which is why the PHY drive/ODT/ZQ settings are on record as
 * "never measured" (N46).
 *
 * Provenance, per the register-name rule -- the ADDRESS is the fact and it was
 * resolved in a vendor-authored map, not inferred from the name:
 *
 *   0xC883668C  DDR0_PUB_ZQ0SR   amlogic/cmd_ddr_test.c:898  [g12a, g12b]
 *   0xC883669C  DDR0_PUB_ZQ1SR   amlogic/cmd_ddr_test.c:902  [g12a, g12b]
 *
 * The BSP names them for g12a/g12b rather than GXL, and the reason they carry
 * here is structural rather than assumed: GXL shares DDR0_PUB_REG_BASE with
 * those SoCs and this header already places ZQ0PR/ZQ1PR/ZQCR at exactly the
 * block indices that arithmetic implies (0x1a1/0x1a5/0x1a0), so the ZQ block
 * layout is the same PUB IP.  GXBB's is NOT -- it puts ZQCR at 0x90 -- which is
 * the case that would have made a blind copy wrong.
 */
#define DDR0_PUB_ZQ0SR	(DDR0_PUB_REG_BASE + (0x1a3 << 2))
#define DDR0_PUB_ZQ1SR	(DDR0_PUB_REG_BASE + (0x1a7 << 2))
#define DDR0_PUB_ZQ2PR	(DDR0_PUB_REG_BASE + (0x1a9 << 2))
#define DDR0_PUB_ZQ3PR	(DDR0_PUB_REG_BASE + (0x1ad << 2))

#define DDR0_PUB_DX0GCR0	(DDR0_PUB_REG_BASE + (0x1c0 << 2))
#define DDR0_PUB_DX0GCR1	(DDR0_PUB_REG_BASE + (0x1c1 << 2))
#define DDR0_PUB_DX0GCR2	(DDR0_PUB_REG_BASE + (0x1c2 << 2))
#define DDR0_PUB_DX0GCR3	(DDR0_PUB_REG_BASE + (0x1c3 << 2))
#define DDR0_PUB_DX0GCR4	(DDR0_PUB_REG_BASE + (0x1c4 << 2))
#define DDR0_PUB_DX0GCR5	(DDR0_PUB_REG_BASE + (0x1c5 << 2))
/*
 * Per-bit delay lines.  The vendor names word 0x1d6 BDLR5, NOT BDLR6:
 *   BDLR0 0x1d0  BDLR1 0x1d1  BDLR2 0x1d2  [hole 0x1d3]
 *   BDLR3 0x1d4  BDLR4 0x1d5  BDLR5 0x1d6  [hole 0x1d7]  BDLR6 0x1d8
 * -- BSP A311D_Linux2603V2.5 uboot/bl33/v2019/cmd/amlogic/cmd_ddr_test.c:1085-1091,
 * inside the GXL guard at :1062 whose DDR0_PUB_REG_BASE (:880) is our 0xc8836000.
 * Corroborated by the gen-2 map in drivers/ram/k3-am654-ddrss.h:372-378, whose
 * byte offsets give the identical layout including BOTH holes:
 * (0x758 - 0x700) / 4 = 0x16 = BDLR5 and (0x760 - 0x700) / 4 = 0x18 = BDLR6.
 * These four symbols were named BDLR6 until 2026-09-01; the ADDRESS is unchanged
 * and no read moves, only the name.  BDLR6 (0x1d8, +0x40/lane) is deliberately
 * NOT defined here -- nothing reads it, and defining it would silently redirect
 * the existing reader in dram-gx.c.
 */
#define DDR0_PUB_DX0BDLR5	(DDR0_PUB_REG_BASE + (0x1d6 << 2))
#define DDR0_PUB_DX0LCDLR0	(DDR0_PUB_REG_BASE + (0x1e0 << 2))
#define DDR0_PUB_DX0LCDLR2	(DDR0_PUB_REG_BASE + (0x1e2 << 2))

#define DDR0_PUB_DX0GTR		(DDR0_PUB_REG_BASE + (0x1f0 << 2))
#define DDR0_PUB_DX1GTR		(DDR0_PUB_REG_BASE + (0x230 << 2))
#define DDR0_PUB_DX2GTR		(DDR0_PUB_REG_BASE + (0x270 << 2))
#define DDR0_PUB_DX3GTR		(DDR0_PUB_REG_BASE + (0x2b0 << 2))

/*
 * DXnGSR* — per-byte-lane PHY general status (gate/WL window, rank state).
 * Addresses from vendor BL2 RE (gxl_bl2_ddr_deep.c serial_print of train err):
 *   DX0GSR0 @ PUB 0x1f8 → 0xc88367e0, DX1 @ 0x238, DX2 @ 0x278, DX3 @ 0x2b8
 *   DX0GSR2 @ 0x1fa, DX0GSR3 @ 0x1fb (+0x40 per lane)
 */
#define DDR0_PUB_DX0GSR0	(DDR0_PUB_REG_BASE + (0x1f8 << 2))
#define DDR0_PUB_DX0GSR2	(DDR0_PUB_REG_BASE + (0x1fa << 2))
#define DDR0_PUB_DX0GSR3	(DDR0_PUB_REG_BASE + (0x1fb << 2))
#define DDR0_PUB_DX1GSR0	(DDR0_PUB_REG_BASE + (0x238 << 2))
#define DDR0_PUB_DX1GSR2	(DDR0_PUB_REG_BASE + (0x23a << 2))
#define DDR0_PUB_DX1GSR3	(DDR0_PUB_REG_BASE + (0x23b << 2))
#define DDR0_PUB_DX2GSR0	(DDR0_PUB_REG_BASE + (0x278 << 2))
#define DDR0_PUB_DX2GSR2	(DDR0_PUB_REG_BASE + (0x27a << 2))
#define DDR0_PUB_DX2GSR3	(DDR0_PUB_REG_BASE + (0x27b << 2))
#define DDR0_PUB_DX3GSR0	(DDR0_PUB_REG_BASE + (0x2b8 << 2))
#define DDR0_PUB_DX3GSR2	(DDR0_PUB_REG_BASE + (0x2ba << 2))
#define DDR0_PUB_DX3GSR3	(DDR0_PUB_REG_BASE + (0x2bb << 2))

#define DDR0_PUB_DX1GCR0	(DDR0_PUB_REG_BASE + (0x200 << 2))
#define DDR0_PUB_DX1GCR1	(DDR0_PUB_REG_BASE + (0x201 << 2))
#define DDR0_PUB_DX1GCR2	(DDR0_PUB_REG_BASE + (0x202 << 2))
#define DDR0_PUB_DX1GCR3	(DDR0_PUB_REG_BASE + (0x203 << 2))
#define DDR0_PUB_DX1GCR4	(DDR0_PUB_REG_BASE + (0x204 << 2))
#define DDR0_PUB_DX1GCR5	(DDR0_PUB_REG_BASE + (0x205 << 2))
#define DDR0_PUB_DX1BDLR5	(DDR0_PUB_REG_BASE + (0x216 << 2))
#define DDR0_PUB_DX1LCDLR0	(DDR0_PUB_REG_BASE + (0x220 << 2))
#define DDR0_PUB_DX1LCDLR2	(DDR0_PUB_REG_BASE + (0x222 << 2))

#define DDR0_PUB_DX2GCR0	(DDR0_PUB_REG_BASE + (0x240 << 2))
#define DDR0_PUB_DX2GCR1	(DDR0_PUB_REG_BASE + (0x241 << 2))
#define DDR0_PUB_DX2GCR2	(DDR0_PUB_REG_BASE + (0x242 << 2))
#define DDR0_PUB_DX2GCR3	(DDR0_PUB_REG_BASE + (0x243 << 2))
#define DDR0_PUB_DX2GCR4	(DDR0_PUB_REG_BASE + (0x244 << 2))
#define DDR0_PUB_DX2GCR5	(DDR0_PUB_REG_BASE + (0x245 << 2))
#define DDR0_PUB_DX2BDLR5	(DDR0_PUB_REG_BASE + (0x256 << 2))
#define DDR0_PUB_DX2LCDLR0	(DDR0_PUB_REG_BASE + (0x260 << 2))
#define DDR0_PUB_DX2LCDLR2	(DDR0_PUB_REG_BASE + (0x262 << 2))

#define DDR0_PUB_DX3GCR0	(DDR0_PUB_REG_BASE + (0x280 << 2))
#define DDR0_PUB_DX3GCR1	(DDR0_PUB_REG_BASE + (0x281 << 2))
#define DDR0_PUB_DX3GCR2	(DDR0_PUB_REG_BASE + (0x282 << 2))
#define DDR0_PUB_DX3GCR3	(DDR0_PUB_REG_BASE + (0x283 << 2))
#define DDR0_PUB_DX3GCR4	(DDR0_PUB_REG_BASE + (0x284 << 2))
#define DDR0_PUB_DX3GCR5	(DDR0_PUB_REG_BASE + (0x285 << 2))
#define DDR0_PUB_DX3BDLR5	(DDR0_PUB_REG_BASE + (0x296 << 2))
#define DDR0_PUB_DX3LCDLR0	(DDR0_PUB_REG_BASE + (0x2a0 << 2))
#define DDR0_PUB_DX3LCDLR2	(DDR0_PUB_REG_BASE + (0x2a2 << 2))

/* PCTL */
#define DMC_PCTL_BASE	0xc8839000

#define PCTL_SCFG	(DDR0_PCTL_BASE + 0x000)
#define PCTL_SCTL	(DDR0_PCTL_BASE + (0x1  << 2))
#define PCTL_STAT	(DDR0_PCTL_BASE + (0x48 << 2))

#define PCTL_CMDTSTAT	(DDR0_PCTL_BASE + (0x48 << 2))

#define PCTL_PPCFG	(DDR0_PCTL_BASE + (0x43 << 2))
#define PCTL_ZQCFG	(DDR0_PCTL_BASE + (0x44 << 2))

#define PCTL_MCFG	(DDR0_PCTL_BASE + (0x41 << 2))
#define PCTL_MCFG1	(DDR0_PCTL_BASE + (0x42 << 2))

#define PCTL_TCKSRE	(DDR0_PCTL_BASE + (0x1a << 2))
#define PCTL_TZQCSI	(DDR0_PCTL_BASE + (0x19 << 2))
#define PCTL_TCKE		(DDR0_PCTL_BASE + (0x1c << 2))
#define PCTL_TMOD		(DDR0_PCTL_BASE + (0x1d << 2))
#define PCTL_TEXSR		(DDR0_PCTL_BASE + (0x15 << 2))
#define PCTL_TAL		(DDR0_PCTL_BASE + (0x50 << 2))
#define PCTL_TCCD		(DDR0_PCTL_BASE + (0x52 << 2))
#define PCTL_TRTP		(DDR0_PCTL_BASE + (0x12 << 2))
#define PCTL_TFAW		(DDR0_PCTL_BASE + (0x11 << 2))
#define PCTL_TCKSRX	(DDR0_PCTL_BASE + (0x1b << 2))
#define PCTL_TRTW		(DDR0_PCTL_BASE + (0x9  << 2))
#define PCTL_TCWL		(DDR0_PCTL_BASE + (0xc  << 2))
#define PCTL_TWR		(DDR0_PCTL_BASE + (0x13 << 2))
#define PCTL_TCL		(DDR0_PCTL_BASE + (0xb  << 2))
#define PCTL_TDQS		(DDR0_PCTL_BASE + (0x1e << 2))
#define PCTL_TRCD		(DDR0_PCTL_BASE + (0xf  << 2))
#define PCTL_TXP		(DDR0_PCTL_BASE + (0x16 << 2))
#define PCTL_TMRD		(DDR0_PCTL_BASE + (0x6  << 2))
#define PCTL_TRAS		(DDR0_PCTL_BASE + (0xd  << 2))
#define PCTL_TREFI_MEM_DDR3	(DDR0_PCTL_BASE + (0x24 << 2)) // replaced by TREFI
#define PCTL_TWTR		(DDR0_PCTL_BASE + (0x14 << 2))
#define PCTL_TRC		(DDR0_PCTL_BASE + (0xe  << 2))
#define PCTL_TRFC		(DDR0_PCTL_BASE + (0x7  << 2))
#define PCTL_TCKESR	(DDR0_PCTL_BASE + (0x22 << 2))
#define PCTL_TZQCL		(DDR0_PCTL_BASE + (0x20 << 2))
#define PCTL_TRRD		(DDR0_PCTL_BASE + (0x10 << 2))
#define PCTL_TRP		(DDR0_PCTL_BASE + (0x8  << 2))
#define PCTL_TZQCS		(DDR0_PCTL_BASE + (0x18 << 2))
#define PCTL_TXPDLL	(DDR0_PCTL_BASE + (0x17 << 2))

#define PCTL_DFIODTCFG	(DDR0_PCTL_BASE + (0x27 << 2))
#define PCTL_DFIODTCFG1	(DDR0_PCTL_BASE + (0x28 << 2))
#define PCTL_DFITCTRLDELAY	(DDR0_PCTL_BASE + (0x26 << 2))
#define PCTL_DFITPHYWRLAT	(DDR0_PCTL_BASE + (0x2b << 2))
#define PCTL_DFITPHYWRDATA	(DDR0_PCTL_BASE + (0x2a << 2))
#define PCTL_DFITRDDATAEN	(DDR0_PCTL_BASE + (0x2c << 2))
#define PCTL_DFITPHYRDLAT	(DDR0_PCTL_BASE + (0x2d << 2))
#define PCTL_DFITPHYUPDTYPE0	(DDR0_PCTL_BASE + (0x2e << 2))
#define PCTL_DFITPHYUPDTYPE1	(DDR0_PCTL_BASE + (0x2f << 2))
#define PCTL_DFIUPDCFG	(DDR0_PCTL_BASE + (0x35 << 2))
#define PCTL_DFISTCFG0	(DDR0_PCTL_BASE + (0x3c << 2))
#define PCTL_DFISTCFG1	(DDR0_PCTL_BASE + (0x3d << 2))
#define PCTL_DFISTSTAT0	(DDR0_PCTL_BASE + (0x46 << 2))
#define PCTL_DFILPCFG0	(DDR0_PCTL_BASE + (0x40 << 2))
#define PCTL_DFITCTRLUPDMIN	(DDR0_PCTL_BASE + (0x32 << 2))
#define PCTL_DFITCTRLUPDMAX	(DDR0_PCTL_BASE + (0x33 << 2))
#define PCTL_DFITDRAMCLKEN	(DDR0_PCTL_BASE + (0x3e << 2))
#define PCTL_DFITDRAMCLKDIS	(DDR0_PCTL_BASE + (0x3f << 2))

/* DDR MMC (see dram-gx.h for more details) */
#define DDR_MMC_BASE	0xc8837000

#define DDR0_SOFT_RESET	(DDR_MMC_BASE + 0x20)

#define AM_DDR_PLL_STS	(DDR_MMC_BASE + 0x18)
/*
 * The vendor's P_DDR0_CLK_CTRL on GXL: cmd/amlogic/cmd_ddr_test.c's
 * P_DDR_PHY_905X arm has DDR_CLK_CNTL = MMC_REG_BASE + (0x7 << 2), then
 * "#define P_DDR0_CLK_CTRL DDR_CLK_CNTL".  One register, two vendor names.
 */
#define DDR_CLK_CNTL	(DDR_MMC_BASE + 0x1c)
#define DDR0_APD_CTRL	(DDR_MMC_BASE + 0x24)

/* GXBB-only, and this will get optimized out by the compiler */
#define DDR0_CLK_CTRL	0
#endif
