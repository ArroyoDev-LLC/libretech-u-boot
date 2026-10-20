/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * G12A/SM1 Strategy A train-FW bases (RE-locked).
 *
 * AUTHORITATIVE register-name sources — use THESE, in this order, before
 * inventing a name from observed behaviour.  Every name in this file was
 * re-derived from them on 2026-08-11; see
 * amlogic/software/boot/g12/ddr-fw-header-audit.md for the per-constant table.
 *   PHY CSRs   libretech-builder-simple
 *              atf/drivers/st/ddr/phy/phyinit/include/ddrphy_csr_all_cdefines.h
 *              (Synopsys/ST generated map; PA = 0xFE000000 + (csr << 1),
 *              16-bit accesses; csr = pstate<<20 | block | inst<<12 | off),
 *              and the DWC LPDDR4x multiPHY PUB databook at
 *              amlogic/software/ref/datasheets/DesignWare_Cores_LPDDR4x_*
 *   AO / DMC   vendor BSP uboot/bl33/v2019/arch/arm/include/asm/arch-g12a/
 *              {secure_apb.h,regs.h} (and the scp_task/ copy), plus the
 *              S905D3 datasheet register tables
 *
 * RE background (NOT name-authoritative — g12a_bl2_regs.h in particular names
 * registers by decompiler usage pattern and has AO_SEC_GP_CFG0 / AO_SEC_SD_CFG0
 * swapped, which is where this file's 0xFF800200 defect came from):
 *   amlogic software/boot/blx-analysis/src/g12a-bl2/g12a_bl2_regs.h
 *   amlogic software/boot/audit/g12b-ddr-init-analysis.md
 *   amlogic software/boot/g12/ddr-phy-firmware-interface.md
 *   amlogic software/boot/g12/s905d3-cc-libre-plan.md (L1.3 / L3.A.6)
 *
 * Vendor BL2 never embeds the full train set in the BROM stage
 * (aml_ddr+piei+1d+2d ≈ 156 KiB ≫ 0xf000). Open SPL is the same:
 * orchestrate load into free AHB SRAM, then kick mailbox training.
 *
 * Open SPL layout (configs/aml-s905d3-cc_libre_*_defconfig) vs these bases:
 *   TEXT  0xFFFA1000  size ≤ 0xf000  → ends by 0xFFFB0000
 *   STACK 0xFFFB3000  (grows down; 4 KiB gap before aml_ddr)
 *   BSS   0xFFFB0000  — those defconfigs set
 *                       CONFIG_SPL_BSS_START_ADDR=0xfffb0000 in BOTH phases.
 *                       A bare 0xFFFBC000 lands INSIDE the aml_ddr scratch,
 *                       so the scratch clear wipes BSS; do not reintroduce it.
 *   aml_ddr.fw @ 0xFFFB4000 (48 KiB clear)
 *   piei.fw    @ 0xFFFCC000
 *   hold park  @ 0xFFFD0000 / 0xFFFE0000 (host residual before FIP keys)
 */

#ifndef __G12A_DDR_FW_H__
#define __G12A_DDR_FW_H__

/*
 * AHB SRAM window.  S905D3 datasheet address map: 0xFFFA0000..0xFFFE7FFF,
 * 288 KiB, "ahb sram" (0xFFFE8000..0xFFFEFFFF is the reserved 32 KiB above
 * it).  Was G12A_TZRAM_* @ 256 KiB, inherited from g12a_bl2_regs.h's
 * AML_TZRAM_TOTAL_SIZE — that stopped at 0xFFFE0000 and so made
 * G12A_FW_HOLD_PIEI look out of window.  Unused; kept for the map.
 */
#define G12A_AHB_SRAM_BASE			0xFFFA0000UL
#define G12A_AHB_SRAM_SIZE			0x00048000UL

/* aml_ddr.fw scratch — RE: memset(0xFFFB4000, 0, 0xC000) before load */
#define G12A_AML_DDR_FW_BASE		0xFFFB4000UL
#define G12A_AML_DDR_FW_SCRATCH_SIZE	0x0000C000UL	/* 48 KiB */
#define G12A_AML_DDR_FW_BLOB_SIZE	48080UL		/* measured file size */

/* PHY Init Engine Image */
#define G12A_PIEI_FW_BASE		0xFFFCC000UL
#define G12A_PIEI_FW_BLOB_SIZE		10088UL	/* whole @DFM file */
/* payload only — the TOC describes payload, not the container */
#define G12A_PIEI_FW_PAYLOAD_SIZE	(G12A_PIEI_FW_BLOB_SIZE - \
					 G12A_DDRFW_HDR_SIZE)

/* Synopsys PMU train FW sizes (loaded via PHY APB, not stuffed in BROM) */
#define G12A_LPDDR4_1D_FW_BLOB_SIZE	49248UL
#define G12A_LPDDR4_2D_FW_BLOB_SIZE	49248UL
/* @DFM container: 32 B enc + 64 B @DFM hdr = 96; payload after +0x60 */
#define G12A_DDRFW_HDR_SIZE		0x60UL

/*
 * Host residual park (AHB free after piei dest, before FIP root-key ~0xFFFE7C20).
 * boot-g12 AMLC may write blobs here; SPL copies to dest after scratch clear.
 * Host may also write directly to G12A_*_FW_BASE after the 'C' breadcrumb.
 *
 * HOLD1 @ 0xFFFE0000 bus-hangs on residual write (measured) — do not use.
 * 1d hold = vendor PHY staging window 0xFFFC0000..0xFFFCC000 (abuts piei).
 */
#define G12A_FW_HOLD_AML_DDR		0xFFFD0000UL
#define G12A_FW_HOLD_PIEI		0xFFFE0000UL	/* hang — legacy only */
#define G12A_FW_HOLD_1D			0xFFFC0000UL	/* vendor IMEM staging */
/*
 * L21 blob-arm DMEM-tail measurement (2026-09-12): the DELIVERED span, not
 * the destination window.  IMEM 0x8000 + the 0x1000 of DMEM either loader
 * copies into the PHY; payload [0x9000,0xC000) is all-zero in both vendor
 * blobs and never staged.  Lockstep with HOLD_1D_MAX in meson-usb-g12.c.
 */
#define G12A_FW_HOLD_1D_MAX		0x00009000UL	/* delivered; window ends 0xFFFCC000 */

/* DWC PHY APB + UCT mailbox (16-bit APB: addr = reg_id*2 + base) */
#define G12A_DDR_PHY_APB_BASE		0xFE000000UL
/*
 * PUB names next to G12 Amlogic 0xFE… PHY windows.
 * PA = 0xFE000000 + (csr << 1).  Citations: DWC LPDDR4x multiPHY PUB 2.20a
 * OCR (amlogic/.../DesignWare_Cores_LPDDR4x_*_PUB_unlimited-ocr.md)
 * §7.8.2 Table 7-2 (ICCM/DCCM), §11.3.9 MicroContMuxSel, §11.3.13
 * UctWriteOnlyShadow, §11.3.227 PPTTrainSetup; and
 * amlogic/software/boot/g12/ddr-phy-firmware-interface.md §2 (mailbox)
 * + §4 (PPT CSR snapshot in training_delay_set_ps_t).
 *
 *   0xFE1A0000  APBONLY0 MicroContMuxSel           CSR 0xD0000
 *   0xFE1A0064  APBONLY0 UctWriteOnlyShadow        CSR 0xD0032
 *   0xFE0A0000  ICCM (IMEM)                        CSR 0x50000
 *   0xFE0A8000  DCCM (DMEM / message block)        CSR 0x54000
 *   0xFE040020  MASTER0 PPTTrainSetup (PState 0)   CSR 0x20010
 *   0xFE240020  MASTER0 PPTTrainSetup (PState 1)   CSR 0x120010
 *   0xFE020154  DBYTE0  PptCtlStatic               CSR 0x0100AA
 */
/*
 * MASTER0 MemResetL, CSR 0x20060 (ST/Synopsys CSR_MEMRESETL_ADDR = 0x60U,
 * TMASTER = 0x20000U).  Was misnamed G12A_DDR_PHY_MEM_ACCESS: it is the
 * DRAM reset_n pin control (bits [1:0]), not an aperture/access control.
 * The vendor loader 2280 writes 2 here (0xFFFB61E8 strh w4,[0xFE0400C0]).
 */
#define G12A_DDR_PHY_MEMRESETL		0xFE0400C0UL
#define G12A_DDR_PHY_CSR_MEMRESETL	0x20060U
/*
 * MASTER0 PPTTrainSetup, CSR 0x20010 (CSR_PPTTRAINSETUP_ADDR = 0x10U,
 * TMASTER = 0x20000U).  PA = 0xFE000000 + (0x20010 << 1) = 0xFE040020.
 * PUB §11.3.227 Table 11-235: PhyMstrTrainInterval[3:0],
 * PhyMstrMaxReqToAck[6:4]; 0 disables DFI PHY Master (no LPDDR4 DDC).
 * Replicated per PState (PState 1 CSR 0x120010 → 0xFE240020).
 * Mission-mode; not a mailbox CSR.  Residual TPL dumps these after T7
 * (G12-C1); pre-kick interval is 0.
 */
#define G12A_DDR_PHY_PPTTRAINSETUP	0xFE040020UL
#define G12A_DDR_PHY_PPTTRAINSETUP_PS1	0xFE240020UL
#define G12A_DDR_PHY_CSR_PPTTRAINSETUP	0x20010U
/*
 * Packed PPTTrainSetup for opt-in DDC (G12-C3).  Encoding is PUB 2.20a
 * §11.3.227 Table 11-235, not invented:
 *   [3:0] PhyMstrTrainInterval  LSB 0
 *         (ST ddrphy_csr_all_cdefines.h:1647 CSR_PHYMSTRTRAININTERVAL_LSB)
 *         0x0 disable PHY Master; 0x1=524288 MEMCLKs … 0xA=268435456;
 *         0xB..0xF undefined.
 *   [6:4] PhyMstrMaxReqToAck    LSB 4
 *         (ST :1649 CSR_PHYMSTRMAXREQTOACK_LSB)
 *         0x0 disable PHY Master; 0x1=512 … 0x5=8192 MEMCLKs;
 *         0x6=32768; 0x7 undefined.
 * Either field 0 disables the interface.  ST phyinit default
 * (ddrphy_phyinit_struct.h:296 interval 0xa, :315 max-req 0x5) packs
 * as (0xA << 0) | (0x5 << 4) = 0x005A — loadpieimage.c:169-175.
 * Closed aml_ddr.fw T7 writes 0 (G12-C1); this is not a copy of closed.
 */
#define G12A_DDR_PHY_PPTTRAINSETUP_ST_DDC	0x005AU
/*
 * MASTER0 PPTTrainSetup2, CSR 0x20011 (CSR_PPTTRAINSETUP2_ADDR = 0x11U,
 * ST ddrphy_csr_all_cdefines.h:126).  PA = 0xFE000000 + (0x20011 << 1)
 * = 0xFE040022.  PUB map 020011 (Table 11-7).  Replicated per PState
 * (0x120011 → 0xFE240022).  T7 writes 3 (aml_ddr.fw +0x1DB8 / ST
 * loadpieimage.c:176-178).  Do not invent the [2:0] encoding here.
 */
#define G12A_DDR_PHY_PPTTRAINSETUP2	0xFE040022UL
#define G12A_DDR_PHY_PPTTRAINSETUP2_PS1	0xFE240022UL
#define G12A_DDR_PHY_CSR_PPTTRAINSETUP2	0x20011U
/*
 * DBYTE0 PptCtlStatic, CSR 0x0100AA (PUB map 0100AA).
 * PA = 0xFE000000 + (0x0100AA << 1) = 0xFE020154.  Instances 0..3 at
 * stride 0x2000 (DBYTE instance field [15:12] of the CSR).
 * Snapshot field csr_pptctlstatic[4] in training_delay_set_ps_t
 * (ddr-phy-firmware-interface.md §4).
 */
#define G12A_DDR_PHY_PPTCTLSTATIC		0xFE020154UL
#define G12A_DDR_PHY_PPTCTLSTATIC_STRIDE	0x2000UL
/*
 * ddr_set_t / aml_ddr ds[] byte offset of csr_pptctlstatic[0].
 * Class-11 save/restore: ds+250+2i → csr (i<<12)|0x100AA
 * (class-restore-transcription.md §5.2 / ddr-capture-replay-procedure.md).
 */
#define G12A_DS_OFF_PPTCTLSTATIC	250U
/* APBONLY0 MicroContMuxSel CSR 0xd0000 — same PA as G12A_DDR_PHY_MICROCONTMUXSEL */
#define G12A_DDR_PHY_CSR_MUXSEL		0xd0000U
/*
 * Microcontroller memory windows.  PUB databook §"Memory Type / PHY CSR
 * Address (16-bit word)": ICCM 0x50000 (32 kB), DCCM 0x54000 (16 kB); the
 * generated map calls the block TUCTL_MEM = 0x50000U.
 * DMEM_LOAD_SIZE is the COPY LENGTH the vendor loader uses (2280:
 * mov x1,#0x1000), not the DCCM aperture — the aperture is 16 kB.
 */
#define G12A_DDR_PHY_IMEM_BASE		0xFE0A0000UL	/* PUB ICCM; CSR 0x50000 *2 */
/*
 * NOT a register and not a second IMEM aperture: this is the discarded
 * "no *2" address-arithmetic hypothesis from the sticky-I hunt, reachable
 * only via G12A_IMEM_MAP_OFF, which g12a_imem_map (dram-g12a.c, static
 * = G12A_IMEM_MAP_CSR2, never assigned) never selects.  It decodes to CSR
 * 0x028000 = MASTER0 instance 8, and MASTER has only instance 0 — i.e. it
 * addresses nothing.  Kept as a labelled falsified hypothesis, not a name.
 */
#define G12A_DDR_PHY_IMEM_ALT		0xFE050000UL	/* base+0x50000 no *2 */
#define G12A_DDR_PHY_DMEM_BASE		0xFE0A8000UL	/* PUB DCCM; CSR 0x54000 */
#define G12A_DDR_PHY_IMEM_SIZE		0x8000UL	/* 32 KiB (PUB: ICCM) */
#define G12A_DDR_PHY_DMEM_LOAD_SIZE	0x1000UL	/* copy len, not DCCM sz */
/*
 * APBONLY0 MicroContMuxSel, CSR 0xd0000 (CSR_MICROCONTMUXSEL_ADDR = 0x0U,
 * TAPBONLY = 0xD0000U).  0 = APB owns the CSR/mem space, 1 = the micro does.
 * Was misnamed G12A_DDR_PHY_UCT_WRITE.
 */
#define G12A_DDR_PHY_MICROCONTMUXSEL	0xFE1A0000UL
/*
 * The PMU mailbox, all four in APBONLY0.  Names and bit meanings from the PUB
 * databook register tables (§11.3.10/.12/.13/.14) and CSR ids from
 * ddrphy_csr_all_cdefines.h:996-1000.  These carried a "UCT_*" prefix that
 * matched none of them — DctWriteProt is a Dct register, and the two message
 * ports are read-only shadows, not "MSG_LO/HI" write ports.
 *
 *   UctShadowRegs         CSR 0xD0004  R    bit0 UctWriteProtShadow:
 *                                           0 = the PMU has a message
 *   DctWriteProt          CSR 0xD0031  R/W  write 0 = acknowledge receipt
 *                                           (default 1)
 *   UctWriteOnlyShadow    CSR 0xD0032  R    major-message ID; low 16 bits of
 *                                           a streaming message
 *   UctDatWriteOnlyShadow CSR 0xD0034  R    upper 16 bits of a streaming
 *                                           message; unused for major msgs
 */
#define G12A_DDR_PHY_UCTSHADOWREGS	0xFE1A0008UL	/* CSR 0xd0004 */
#define G12A_DDR_PHY_DCTWRITEPROT	0xFE1A0062UL	/* CSR 0xd0031 */
#define G12A_DDR_PHY_UCTWRITEONLYSHADOW	0xFE1A0064UL	/* CSR 0xd0032 */
#define G12A_DDR_PHY_UCTDATWRITEONLYSHADOW 0xFE1A0068UL	/* CSR 0xd0034 */
/*
 * APBONLY0 MicroReset, CSR 0xd0099 (CSR_MICRORESET_ADDR = 0x99U,
 * TAPBONLY = 0xD0000U), 4-bit field: stall/reset control for the ARCv2.
 * Was misnamed G12A_DDR_PHY_UCT_DONE — it is not a completion status.
 * The 9/1/0 sequence in the micro start is reset+stall / reset / run.
 */
#define G12A_DDR_PHY_MICRORESET		0xFE1A0132UL
/*
 * DRTUB0 UcclkHclkEnables CSR 0xc0080 (PA = csr*2 + base).
 *
 * The "DMEM readback needs HclkEn" theory this once carried is DEAD: measured
 * 0003 already set after 2280, forcing it changed nothing, and the vendor
 * writes it in exactly one place (0xFFFB60E4, inside 0xFFFB5BC4) which a644
 * reaches only AFTER training. It is not part of the micro-start sequence —
 * 4dc0 is four writes: MuxSel=1 then MicroReset 9/1/0.
 */
#define G12A_DDR_PHY_UCCLKHCLKENABLES	0xFE180100UL	/* CSR 0xc0080 */
#define G12A_DDR_PHY_UCCLKHCLKENABLES_BOTH 0x3U		/* UcclkEn | HclkEn */
/* IMEM halfword addr maps (Solitude sticky-I hunt) */
#define G12A_IMEM_MAP_CSR2		0	/* vendor: (0x50000+i)*2 + base */
#define G12A_IMEM_MAP_OFF		1	/* base + 0x50000 + i*2 */
#define G12A_IMEM_MAP_DWORD		2	/* FE0A0000 + i*4 */
#define G12A_IMEM_MAP_32B		3	/* MAP_CSR2 with writel pairs */

/*
 * DMC / PLL (orchestration context; full train lives in aml_ddr.fw).
 *
 * Two distinct register blocks inside the datasheet's 8 KiB "dmc" space
 * 0xFF638000..0xFF639FFF, and the datasheet's own extraction lists BOTH as
 * offsets from 0xFF638000, which is a trap:
 *   0xFF638000  DMC_REQ_CTRL / DMC_SOFT_RST{,1,2} / DMC_RST_STS1 / ...
 *               (BSP arch-g12a/regs.h:13 "DMC_REG_BASE (0xFF638000L)")
 *   0xFF638C00  AM_DDR_PLL_CNTL0..6 / _STS / DDR_CLK_CNTL / DDR_PHY_CTRL
 *               (BSP cmd/amlogic/cmd_ddr_test.c:648
 *                "#define G12_AM_DDR_PLL_CNTL0 0xff638c00")
 * NOT the DMC secure/scramble cluster that g12a_bl2_regs.h calls
 * AML_DMC_SEC_CTRL / AML_DMC_SCRAMBLE_CTRL at the same two addresses.
 */
#define G12A_DMC_BASE			0xFF638000UL
#define G12A_DDR_PLL_BASE		0xFF638C00UL
#define G12A_DDR_PLL_CNTL0		(G12A_DDR_PLL_BASE + 0x00UL)
#define G12A_DDR_PLL_CNTL1		(G12A_DDR_PLL_BASE + 0x04UL)
#define G12A_DDR_PLL_CNTL2		(G12A_DDR_PLL_BASE + 0x08UL)
#define G12A_DDR_PLL_CNTL3		(G12A_DDR_PLL_BASE + 0x0cUL)
#define G12A_DDR_PLL_CNTL4		(G12A_DDR_PLL_BASE + 0x10UL)
#define G12A_DDR_PLL_CNTL5		(G12A_DDR_PLL_BASE + 0x14UL)
#define G12A_DDR_PLL_CNTL6		(G12A_DDR_PLL_BASE + 0x18UL)
/*
 * WATCHDOG_CNTL, EE "isa" block: BSP arch-g12a/regs.h:13304-13305
 * "#define WATCHDOG_CNTL (0x3c34)" / (0x3c34 << 2) + 0xffd00000.
 * aml_ddr FUN_fffd00ec pets it the same way.  _CLR knocks down bits 25 and 18.
 * (g12a_bl2_regs.h calls this G12A_BL2_ISA_TIMER "timer/scratch" — it is not.)
 */
#define G12A_WATCHDOG_CNTL		0xFFD0F0D0UL
#define G12A_WATCHDOG_CNTL_CLR		0xFDFBFFFFUL

/*
 * Solitude LPDDR4 primary pstate (boot-lite timing.c first entry).
 * FUN_fffd4f14(1608, PllBypass=0): M=1608/12=134, OD=0, N=2 → 0x00100486
 * CNTL1..6 defaults from aml_ddr FUN_fffd504c when ACS PLL block empty.
 *
 * G12A_SOLITUDE_DRAM_FREQ_MHZ is ddr_set_t.DRAMFreq[0] in BOTH ACS arms:
 * aml_ddr fp_pll re-programs the DDR PLL from it ("ddr clk to %uMHz") and
 * S3 writes msgblock DRAMFreq = it * 2, so it is the training clock --
 * g12a_ddr_pll_init()'s 1608 lock is overwritten by fp_pll.  Set to the
 * product BL2's lowest ACS row (1200) for the off-table LPDDR4X, which
 * rejects 1D at 1608.
 */
#ifndef G12A_SOLITUDE_DRAM_FREQ_MHZ
#define G12A_SOLITUDE_DRAM_FREQ_MHZ	1200U
#endif

/*
 * G12A_DS_MAX_CORE_TIMING_MHZ is ddr_set +0x6a max_core_timmming_frequency
 * (G12A_DS_OFF_MAX_CORE_FREQ): the timing-engine speed-bin cap, NOT a
 * hardware limit.  g12a-ddr-timing.h takes cap = it ?: 1600 and
 * fbin = f < cap ? f : cap - 12, so every f >= 1600 bins as 1588 while it
 * reads 0.  0U keeps the product ACS byte pair 00 00 (vendor default 1600).
 */
#ifndef G12A_DS_MAX_CORE_TIMING_MHZ
#define G12A_DS_MAX_CORE_TIMING_MHZ	0U
#endif

#define G12A_DDR_PLL_VAL_1608		0x00100486UL
#define G12A_DDR_PLL_CNTL1_DEF		0x00000004UL
#define G12A_DDR_PLL_CNTL2_DEF		0x00000000UL
#define G12A_DDR_PLL_CNTL3_DEF		0x08691c00UL
#define G12A_DDR_PLL_CNTL4_DEF		0x88770290UL
#define G12A_DDR_PLL_CNTL5_DEF		0x39272000UL
#define G12A_DDR_PLL_CNTL6_DEF		0x50540000UL
#define G12A_DDR_PLL_RST_ASSERT		0x1c000000UL
#define G12A_DDR_PLL_RST_RELEASE	0x3c000000UL
#define G12A_DDR_PLL_LOCK_BIT		(1U << 31)
/*
 * PHY/DMC soft-reset + clock enable after PLL (BL2 FUN_fffa7e58).
 * Required before MemResetL / IMEM APB — Solitude hung on v write without this.
 * DMC_SOFT_RST{,1,2} and DDR_CLK_CNTL / DDR_PHY_CTRL are the S905D3
 * datasheet's own names for these five (see the two-block note at
 * G12A_DMC_BASE).  DDR_PHY_CTRL was G12A_DDR_CLK_CNTL2, a name that exists
 * in no map; the identification was already made in
 * amlogic/software/boot/g12/g12a-ddr-clock-tree.md and never applied here.
 * The vendor writes both, same addresses, same values: fffa8080 C20 <-
 * 0x10000000 then 0xb0000007; fffa81f0 C24 <- 0x10 then 0x1f.
 */
#define G12A_DMC_SOFT_RST		(G12A_DMC_BASE + 0x04UL)	/* 0xFF638004 */
#define G12A_DMC_SOFT_RST1		(G12A_DMC_BASE + 0x08UL)
#define G12A_DMC_SOFT_RST2		(G12A_DMC_BASE + 0x0cUL)
#define G12A_DDR_CLK_CNTL		(G12A_DDR_PLL_BASE + 0x20UL) /* 0xFF638C20 */
#define G12A_DDR_PHY_CTRL		(G12A_DDR_PLL_BASE + 0x24UL) /* 0xFF638C24 */
#define G12A_DDR_CLK_CNTL_VAL		0xb0000007UL
#define G12A_DDR_PHY_CTRL_VAL		0x1fUL

/*
 * LPDDR4 1D ACS (PMU_SMB_LPDDR4_1D_t) — Solitude timing.c.
 * SequenceCtrl 0x131f = DevInit|WrLvl|RxEn|RdDQS1D|WrDQ1D|RdDeskew|MxRdLat|LPCA.
 * HdtCtrl: product timing.c = 0x0a (coarse). Open briefly used 0x05 (detailed)
 * for stage text, but 0x05 (≤9) enables 4dc0 prologue puts of
 * "c_vref_reg_value 0x" then main() UCT hang — red herring. Match product 0x0a.
 */
/*
 * Product full 1D = 0x131f (DevInit|WrLvl|RxEn|RdDQS1D|WrDQ1D|RdDeskew|
 * MxRdLat|LPCA). DevInit-only 0x0001 measured 04b744: still printed
 * "0001 rx_clk_window:" and no K — a644 train path does not honor a
 * reduced SequenceCtrl alone (or re-prepare ignores ACS). Keep FULL.
 */
#define G12A_ACS_SEQCTRL_1D_FULL	0x131fU
#define G12A_ACS_SEQCTRL_1D_DEVINIT	0x0001U
/* DevInit|WrLvl — exp(a) on split a644 (no RxEn). Monolithic a644 overwrote
 * PHY SequenceCtrl to 0x10f so DevInit-only never stuck; re-measure on split. */
#define G12A_ACS_SEQCTRL_1D_WRLVL	0x0003U
#define G12A_ACS_SEQCTRL_1D		G12A_ACS_SEQCTRL_1D_FULL
/* Product staged after 2280 (a644): 0x1001 LPCA pass, then 0x10f data-eye. */
#define G12A_ACS_SEQCTRL_PHY_LPCA	0x1001U
#define G12A_ACS_SEQCTRL_PHY_DATA	0x010fU
#define G12A_ACS_SEQCTRL_2D		0x0061U
#define G12A_ACS_HDTCTRL	0x0aU	/* product timing.c coarse */
#define G12A_ACS_HDTCTRL_PRODUCT	0x0aU
#define G12A_ACS_PHYVREF_025		0x20U	/* ~250 permil of VDDQ */
#define G12A_ACS_BPZN_240OHM		0xf0U
#define G12A_ACS_DFIFREQ_RATIO		0x02U	/* 1:2 DfiCtl:memclk (LPDDR4) */
#define G12A_ACS_DFIMRL_MARGIN		0x01U

/* Solitude ddr_set_t constants (boot-lite timing.c first pstate) */
#define G12A_DS_MAGIC			0x2e676663UL	/* DRAM_CFG_MAGIC */
#define G12A_DS_DDR_FUNC		0x80000000UL	/* (1u<<31) default bits */
#define G12A_DS_BOARD_ID		0xffU		/* CONFIG_BOARD_ID_MASK */
#define G12A_DS_VERSION			1U
#define G12A_DS_PLL_SSC_MODE		0x00100120UL	/* center 1000 ppm */
#define G12A_DS_SLEW_CLK		0x3ffU
#define G12A_DS_SLEW_CS			0x100U
#define G12A_DS_SLEW_AC			0x100U
#define G12A_DS_SLEW_DATA		0x1ffU
/* ddr_dmc_remap[5] from Solitude timing.c */
#define G12A_DS_DMC_REMAP0		0x14941cc5UL
#define G12A_DS_DMC_REMAP1		0x2307800bUL
#define G12A_DS_DMC_REMAP2		0x2f6ad272UL
#define G12A_DS_DMC_REMAP3		0x3bcdeb38UL
#define G12A_DS_DMC_REMAP4		0x0007359eUL
/* DMEM / msg-block scratch (BL2 FUN_fffa64d4) */
#define G12A_MSG_BLOCK_BASE		0xFFFC8000UL

/* AO UART (BROM leaves up) — same as spl.c early path */
#define G12A_AO_UART_WFIFO		0xFF803000UL
#define G12A_AO_UART_STATUS		0xFF80300cUL

/*
 * @DFM blob header: magic string sits at +0x34 (measured 2026-08-09 on
 * blx/init/g12a/{aml_ddr,piei}.fw — +0x30 is date/version 0x00201709;
 * +0x34 is AML0/eaea). LE words:
 *   AML0 = 0x304C4D41, eaea = 0x61656165, dl44 = 0x34346C64, dl40 = 0x30346C64
 */
#define G12A_DDRFW_MAGIC_OFF		0x34U
#define G12A_DDRFW_MAGIC_AML0		0x304C4D41UL
#define G12A_DDRFW_MAGIC_PIEI		0x61656165UL
#define G12A_DDRFW_MAGIC_LP4_1D		0x34346C64UL
#define G12A_DDRFW_MAGIC_LP4_2D		0x30346C64UL

/*
 * PMU major-message IDs — the vendor main() switch in aml_ddr.fw at D-VA
 * 0xFFFB483C, and the string each arm prints, read verbatim out of the blob
 * (md5 eddca53f01322e2a7f4a8aca1b89ea3a; file offset = D-VA - 0xFFFB4000 +
 * 0x60).  The previous names in this block were guessed from the stage order
 * a training run "ought" to have and were wrong for eleven of the sixteen
 * ids — most consequentially 0x0b, which was labelled "Training has run
 * successfully" (that is 0x07) and is really the LCDL offset calibration
 * progress message.  See amlogic/software/boot/g12/uct-poll-phase-n15.md.
 *
 * Only TWO ids are terminal in the vendor loop: 0x07 (main() sets its return
 * to 0 at 0xFFFB4970 and leaves) and 0xff (returns 0xff at 0xFFFB4A18).
 * Every other id is printed and the loop continues — none of them is a
 * success signal, and there is no register-based completion test anywhere.
 */
#define G12A_DDR_MSG_END_INIT		0x00U	/* "End of initialization" */
#define G12A_DDR_MSG_FINE_WRLVL		0x01U	/* "End of fine write leveling" */
#define G12A_DDR_MSG_RDEN_TRAIN		0x02U	/* "End of read enable training" */
#define G12A_DDR_MSG_RD_DLY_CENTER	0x03U	/* "End of read delay center optimization" */
#define G12A_DDR_MSG_WR_DLY_CENTER	0x04U	/* "End of write delay center optimization" */
#define G12A_DDR_MSG_RD_2D		0x05U	/* "End of 2D read delay Voltage center optimization" */
#define G12A_DDR_MSG_WR_2D		0x06U	/* "End of 2D write delay Voltage center optimization" */
#define G12A_DDR_MSG_TRAIN_DONE		0x07U	/* "Training has run successfully!" — main() returns 0 */
#define G12A_DDR_MSG_RESULTS		0x08U	/* streaming block; handled, no string */
#define G12A_DDR_MSG_MAX_RDLAT		0x09U	/* "End of max read latency training" */
#define G12A_DDR_MSG_RD_DQ_DESKEW	0x0aU	/* "End of read dq deskew training" */
#define G12A_DDR_MSG_LCDL_OFFSET_CAL	0x0bU	/* "End of LCDL offset calibration" */
#define G12A_DDR_MSG_LRDIMM		0x0cU	/* "End of LRDIMM Specific training" */
#define G12A_DDR_MSG_CA_TRAIN		0x0dU	/* "End of CA training" */
#define G12A_DDR_MSG_MPR_RD_CENTER	0xfdU	/* "End of MPR read delay center optimization" */
#define G12A_DDR_MSG_WRLVL_COARSE	0xfeU	/* "End of Write leveling coarse delay" */
#define G12A_DDR_MSG_FAIL		0xffU	/* "ERROR : Training has failed!" — main() returns 0xff */

/* Open train HdtCtrl — product 0x0a (skip 4dc0 c_vref puts; match timing.c) */
#define G12A_DDR_HDTCTRL_DEFAULT	G12A_ACS_HDTCTRL

/*
 * ddr_fw control block + min ACS — AHB below stack top 0xFFFB3000
 * (vendor BL2 uses ~0xFFFB1290). Must not overlap TEXT (≤0xFFFB0000) or
 * aml_ddr (0xFFFB4000). Stack grows down from 0xFFFB3000 — keep ≥4 KiB.
 *
 * RE (g12b bl2.c FUN_fffa6a60 / aml_ddr_load; amlddr-fw-function-map.md;
 *     g12a aml_ddr.fw export table @ file+0x60, STP = export+0x60):
 *   ctl is array of u64 slots (AArch64):
 *     [0] +0x00  pll_init          FUN_fffd504c  (export[7]+0x60)
 *     [3] +0x18  driver_init       LPDDR4: FUN_fffda644 (export[3]+0x60)
 *     [5] +0x28  main / memtest    FUN_fffd5df4  (export[5]+0x60)
 *     [6] +0x30  post_init         FUN_fffd9578  (export[6]+0x60)
 *     [7] +0x38  mid (BL2-side)    FUN_fffa81b8 — not in aml_ddr
 *   Type-select installs [3] from export[type]; [0]/[5]/[6] common.
 *   FUN_fffd4dc0 (TRAIN) is mailbox-only (UCT start + main()) — NOT [3].
 *   a644 path: topology → timing → prepare → pinmux → FUN_fffd2280
 *   (IMEM 0xFFFC0000→PHY + DMEM 0xFFFC8000→PHY) → FUN_fffd4dc0.
 *     [0x14] +0xa0  → ddr_set_t *
 *     [0x15] +0xa8  → timing_t *
 *     [0x16] +0xb0  → pll_param *
 *     +0xe8  need_2d (u8)
 *     +0xea  phase (0xf=aml,0=piei,1=1d,2=2d)
 *     +0x1b8 type, +0x1b9 ch, +0x1ba silent
 *     +0x205 HdtCtrl / verbosity
 *     +0x264 init mode (0=train; 0xff=fastboot restore, skips 2280/4dc0)
 * aml_ddr.fw linked for load base 0xFFFB4000; Ghidra used 0xFFFD0000
 * (delta -0x1C000). STP prologue is often +0x14 past the Ghidra label
 * (epilogue of previous fn sits at ghidra_va - 0x1C000).
 *   real_STP = ghidra_va - 0x1C000 + 0x14
 * Verified on g12a aml_ddr.fw (48080 B, May 2020). Wrong low-16-bit
 * maps (0xFFFB0000|lo16) land mid-function — PREPARE 0xFFFBA644 was
 * CMP not STP and hung after crumb G once ACS/DMEM got richer.
 * Hang after E with [3]=TRAIN: 4dc0 polls UCT forever without IMEM load.
 */
#define G12A_DDR_CTL_BASE		0xFFFB1000UL
#define G12A_DDR_CTL_SIZE		0x00000280UL
#define G12A_DDR_SET_BASE		0xFFFB1280UL
#define G12A_DDR_SET_SIZE		0x00000498UL	/* vendor ACS entry size */
#define G12A_DDR_TIM_BASE		0xFFFB1780UL
#define G12A_DDR_TIM_SIZE		0x00000200UL
#define G12A_DDR_PLL_PAR_BASE		0xFFFB1980UL
#define G12A_DDR_PLL_PAR_SIZE		0x00000060UL

/*
 * Black-box training probe (CONFIG_MESON_G12A_DRAM_PROBE): the host
 * uploads a cfg overlay here each run so a geometry sweep is a
 * re-upload, not a rebuild. 0xFFFB2000 sits in the OCRAM gap between
 * the PLL-par block (ends 0xFFFB19E0) and the aml_ddr blob window
 * (starts 0xFFFB4000); g12a_ddr_ctl_build() zeroes only ctl/ds/tim/pll,
 * so this window survives the pre-train boot untouched.
 *
 * Layout — little-endian u32 per field, 0xFFFFFFFF = "keep built-in".
 * A non-matching magic disables the whole overlay (built-in cfg stands).
 */
#define G12A_CFG_PROBE_BASE		0xFFFB2000UL
#define G12A_CFG_PROBE_MAGIC		0x314E4750UL	/* 'PGN1' LE */
#define G12A_CFG_PROBE_NONE		0xFFFFFFFFUL
#define G12A_CFG_PROBE_RANK_CFG		0x04U	/* -> ds[RANK_CFG]      (rank/width) */
#define G12A_CFG_PROBE_DDR_FUNC		0x08U	/* -> ds[DDR_FUNC]      (density/geom) */
#define G12A_CFG_PROBE_RFC_TYPE		0x0cU	/* -> ds[RFC_TYPE]      (tRFC class) */
#define G12A_CFG_PROBE_FREQ_MHZ		0x10U	/* -> ds[DRAM_FREQ] f0  (freq point) */
#define G12A_CFG_PROBE_LPDDR4X		0x14U	/* -> ds[LPDDR4X]       (vref class) */
#define G12A_CFG_PROBE_CS0_MB		0x18U	/* -> ds[CS0_MB]        (size/density) */
#define G12A_CFG_PROBE_CS1_MB		0x1cU	/* -> ds[CS1_MB]        (rank1 present) */
#define G12A_CFG_PROBE_DISABLED_DBYTE	0x20U	/* -> ds[DISABLED_DBYTE](bus width) */

/* bound on the open UCT mailbox poll (a runaway micro must not spin) */
#define G12A_PROBE_MAILBOX_MAX_MSG	4000U

/*
 * aml_ddr.fw entries — PAYLOAD (file +0x60) loaded at 0xFFFB4000.
 *
 * The image's own export table (payload +0x00 = file +0x60) holds
 * 0xFFFBE5F8 for LPDDR4/a644, 0xFFFB9DA8 for main, 0xFFFB9000 for pll —
 * i.e. the linker base IS 0xFFFB4000 and it addresses the PAYLOAD.
 * Loading the whole FILE there instead put every byte 0x60 high; code
 * still ran (PC-relative) but every in-image ADRP data reference resolved
 * 0x60 early, which is what manufactured the "prepare bugs" below and the
 * mis-resolved UART strings. Entries are now the table values verbatim.
 */
#define G12A_AML_FN_PLL			0xFFFB9000UL	/* export[7] — ctl[0] */
#define G12A_AML_FN_PINMUX		0xFFFB7ED0UL
#define G12A_AML_FN_TRAIN		0xFFFB8D74UL	/* 4dc0 mailbox only */
#define G12A_AML_FN_LOAD_IMEM		0xFFFB6234UL	/* 2280 IMEM+DMEM → PHY */
#define G12A_AML_FN_LOAD_2280	G12A_AML_FN_LOAD_IMEM/* alias: vendor IMEM+DMEM load */
/*
 * The halfword copy routine that 2280 calls twice (IMEM 0x8000 B from
 * 0xFFFC0000 to CSR 0x50000, then DMEM 0x1000 B from 0xFFFC8000 to CSR
 * 0x54000), and the store inside its loop.  Runtime VAs; the objdump of
 * aml_ddr.fw with --adjust-vma=0xFFFB4000 prints payload VA + 0x60, so the
 * loop store printed at 0xFFFB6278 lives at 0xFFFB6218 at run time.
 *
 *   fffb623c: mov  x3,#0xc0 / movk #0xfe04     MemResetL
 *   fffb6248: strh w4,[x3]                     MemResetL = 2
 *   fffb6250: strh wzr,[x3=0xfe1a0000]         MicroContMuxSel = 0
 *   fffb6254: lsr  x1,x1,#1                    x1 = halfword count
 *   fffb6260: and  x4,x3,#0xffffffff           <- loop top
 *   fffb6264: cmp  x4,x1
 *   fffb6268: b.cs 0xfffb6284                  <- exit
 *   fffb626c: ldrh w6,[x0,x3,lsl #1]
 *   fffb6270: add  x4,x4,x2
 *   fffb6274: lsl  x4,x4,#1
 *   fffb6278: strh w6,[x4,x5]                  <- THE STORE (x5=0xfe000000)
 *   fffb627c: add  x3,x3,#0x1
 *   fffb6280: b    0xfffb6260
 *   fffb628c: strh w1,[0xfe1a0000]             MicroContMuxSel = 1
 */
#define G12A_AML_LOOP_COPY		0xFFFB61DCUL	/* copy routine entry */
#define G12A_AML_LOOP_STORE		0xFFFB6218UL	/* strh w6,[x4,x5] */
#define G12A_AML_LOOP_STORE_INSN	0x78256886u
#define G12A_AML_LOOP_NEXT		0xFFFB621CUL	/* add x3,x3,#1 */
#define G12A_AML_FN_PREPARE		0xFFFBE1E4UL	/* a230 */
#define G12A_AML_FN_LPDDR4		0xFFFBE5F8UL	/* export[3] — ctl[3] LPDDR4 */
#define G12A_AML_FN_TOPOLOGY		0xFFFB92BCUL	/* a644 step: topology */
#define G12A_AML_FN_TIMING		0xFFFB69BCUL	/* a644 step: timing */
#define G12A_AML_FN_MID1		0xFFFB62C8UL	/* a644 step after pinmux */
#define G12A_AML_FN_MID2		0xFFFB4E70UL
#define G12A_AML_FN_MID3		0xFFFB4E04UL
/*
 * a644 STAGE 2 (T1..T14) — the steps that run AFTER 1D training and that
 * actually bring the DDR controller up.  Addresses are PAYLOAD VAs, i.e.
 * the D-VA printed by objdump --adjust-vma=0xFFFB4000 on the FILE minus
 * 0x60, the same convention every other G12A_AML_FN_* here uses.
 *
 * Transcribed from the a644 tail, disassembled this session:
 *   fffbea90: bl 0xfffb5bc4   T7  PHY post-train program (PIE window, PPT)
 *   fffbea9c: bl 0xfffb5a54   T8  save/apply 10 delay classes (w1 = 1)
 *   fffbeaa4: bl 0xfffbbbb0   T9  apply <=3 user CSR overrides
 *   fffbeaac: bl 0xfffb8af0   T10 DMC <-> PHY handshake state machine
 *   fffbeab0: bl 0xfffb61c8   T11 clear 8 DMC status regs (no x0 setup)
 *   fffbeab8: bl 0xfffb8d2c   T12 DMC "go"
 *   fffbeb04: bl 0xfffb4bc4   T14 vref average / report (w1 = 2)
 */
#define G12A_AML_FN_T7_PHY_POST		0xFFFB5B64UL	/* D-VA fffb5bc4 */
#define G12A_AML_FN_T8_DELAY_SAVE	0xFFFB59F4UL	/* D-VA fffb5a54 */
#define G12A_AML_FN_T9_CSR_OVERRIDE	0xFFFBBB50UL	/* D-VA fffbbbb0 */
#define G12A_AML_FN_T10_DMC_PHY		0xFFFB8A90UL	/* D-VA fffb8af0 */
#define G12A_AML_FN_T11_DMC_CLR		0xFFFB6168UL	/* D-VA fffb61c8 */
#define G12A_AML_FN_T12_DMC_GO		0xFFFB8CCCUL	/* D-VA fffb8d2c */
#define G12A_AML_FN_T14_VREF		0xFFFB4B64UL	/* D-VA fffb4bc4 */
/*
 * a644 T5b — the 2D pass, between T2 and T7.  Transcribed from the same
 * disassembly (D-VA, subtract 0x60 for the payload VA used here):
 *   fffbea3c: cmp ctl[612], #0xff ; b.eq fffbea80  -> fastboot skips 2D
 *   fffbea48: bl 0xfffbe244   prepare, phase==2 branch -> 2D message block
 *   fffbea50: bl 0xfffb6294   IMEM/DMEM -> PHY from FIXED 0xFFFC0000/8000
 *   fffbea58: bl 0xfffb8dd4   train (w20 = result; cbnz w20 skips T7..T14)
 *   fffbea68: bl 0xfffb4bc4   vref, w1 = 1  (T14 with a DIFFERENT arg)
 *   fffbea70: bl 0xfffb94d0   2D post A
 *   fffbea78: bl 0xfffba544   2D post B
 * The first three are PREPARE / LOAD_IMEM / TRAIN, i.e. literally the same
 * functions the 1D pass uses; only ctl[234] (PHASE) and the image staged at
 * 0xFFFC0000 differ.  fffb6294 takes NO source argument — it is
 *   mov x0,#0xfffc0000 ; mov x1,#0x8000  ; mov x2,#0x50000 ; bl fffb623c
 *   mov x0,#0xfffc8000 ; mov x1,#0x1000  ; mov x2,#0x54000 ; bl fffb623c
 * so the caller MUST have replaced the 1D image at 0xFFFC0000 with the 2D
 * image before it runs.  That is the whole of the "2D delivery" problem.
 */
#define G12A_AML_FN_T5B_2D_POST_A	0xFFFB9470UL	/* D-VA fffb94d0 */
#define G12A_AML_FN_T5B_2D_POST_B	0xFFFBA4E4UL	/* D-VA fffba544 */
/*
 * T13 is inline MMIO in a644, fffbeabc..fffbeaf8: read PS0, set bit 5, write
 * PS0; read PS0 again, set bit 5, write PS1.  Both are MASTER0
 * DqsPreambleControl — the PUB register table lists its four PState copies
 * explicitly as "020024 120024 220024 320024", so csr bits [21:20] are the
 * PSTATE, not a channel.  (dram-g12a.c's own MID2 RE sees the same field:
 * fffb4f20 `lsl w6,w6,#20` on ctl+0x1b9 before `orr #0x20000` = TMASTER.)
 */
#define G12A_DDR_PHY_DQSPREAMBLECONTROL_PS0	0xFE040048UL	/* CSR 0x020024 */
#define G12A_DDR_PHY_DQSPREAMBLECONTROL_PS1	0xFE240048UL	/* CSR 0x120024 */
/*
 * Payload load: skip the 0x60 @DFM container header. Self-check reads the
 * export table's LPDDR4 slot back out of the destination.
 */
#define G12A_AML_FW_PAYLOAD_SIZE	(G12A_AML_DDR_FW_BLOB_SIZE - \
					 G12A_DDRFW_HDR_SIZE)
#define G12A_AML_FW_EXPORT_LPDDR4_OFF	0x18UL	/* export[3] in payload */
#define G12A_AML_FW_EXPORT_LPDDR4_VAL	0xFFFBE5F8UL
/*
 * RETIRED — both "prepare bugs" below were artifacts of the 0x60 load-base
 * error, not vendor defects. With the payload at 0xFFFB4000 the rank helper
 * resolves the real map at payload +0xb390 (it was reading the string 0x60
 * early), and the DRAMFreq `lsl #1` is the correct MHz→MT/s conversion:
 * ACS DRAMFreq is the DRAM clock in MHz (1608), the PMU message block wants
 * MT/s (3216). NOPing it shipped a half-rate DRAMFreq, which scales every
 * CA delay step. Kept only as a record; the patcher no longer runs.
 *
 * Historical text:
 * prepare (FUN_fffda230) RE bugs in g12a aml_ddr.fw (May 2020):
 * 1) DRAMFreq: ldrh ACS+0x38 then lsl #1 → msg+0x06. ACS stores MT/s
 *    (1608); PMU wants MT/s (mnPmuSramMsgBlock: 0x640 = DDR1600). The
 *    shift yields 3216. NOP both 1D (0xa294) and 2D-class (0x9f7c) sites.
 * 2) CsPresent: rank helper @ 0xFFFB62FC does table[rank_cfg-1] via
 *    ADRP 0xfffbf000 + #0x390 → lands in "non-sec scramble" string
 *    (rc=3 → 0x6e → ChA=0x02 ChB=0x03). Correct map is at +0x3f0:
 *    {0x01,0x01,0x0f,0x03,0x01,0x03,0x03,0x05} (rc=3 → 0x0f = 0x03/0x03).
 * Open patches AHB image after load so a644 re-prepare is correct.
 */
#define G12A_AML_OFF_PREPARE_FREQ_LSL1	0xa294UL	/* 1D prepare lsl w1,#1 */
#define G12A_AML_OFF_PREPARE_FREQ_LSL2	0x9f7cUL	/* 2D-class prepare lsl */
#define G12A_AML_OFF_RANK_MAP		0xb390UL	/* helper ADRP+#0x390 table */
#define G12A_AML_A64_NOP			0xd503201fUL
#define G12A_AML_RANK_MAP0		0x01U
#define G12A_AML_RANK_MAP1		0x01U
#define G12A_AML_RANK_MAP2		0x0fU	/* RANK01_CH01 → ChA/B=0x03 */
#define G12A_AML_RANK_MAP3		0x03U
#define G12A_AML_RANK_MAP4		0x01U
#define G12A_AML_RANK_MAP5		0x03U
#define G12A_AML_RANK_MAP6		0x03U
#define G12A_AML_RANK_MAP7		0x05U
/*
 * These two were MISSED by the 0x60 load-base correction (e49f0d4ceb7) — they
 * sit apart from the main entry block and kept their old D-VA values, 0x60
 * past the real export-table entries (slot 5 = 0xFFFB9DA8, slot 6 =
 * 0xFFFBD52C, read from the blob at payload +0x28 / +0x30). Latent while
 * nothing calls them; fatal the moment a replay path invokes fp_main/fp_post.
 */
#define G12A_AML_FN_MAIN		0xFFFB9DA8UL	/* export[5] — ctl[5] memtest */
#define G12A_AML_FN_POST		0xFFFBD52CUL	/* export[6] — ctl[6] */
/* Image export trampoline near PLL (table +0x98) — B to PLL body */
#define G12A_AML_IMG_PTR_TABLE		0x60UL

/* ddr_set_t packed offsets (boot-lite g12 timing.h, packed) */
#define G12A_DS_OFF_MAGIC		0x00U
#define G12A_DS_OFF_FAST_BOOT		0x04U
#define G12A_DS_OFF_DDR_FUNC		0x08U
#define G12A_DS_OFF_BOARD_ID		0x0cU
#define G12A_DS_OFF_VERSION		0x0dU
#define G12A_DS_OFF_DRAM_TYPE		0x0eU
#define G12A_DS_OFF_DISABLED_DBYTE	0x0fU
#define G12A_DS_OFF_IS_2T		0x10U
#define G12A_DS_OFF_HDTCTRL		0x11U
#define G12A_DS_OFF_RANK_CFG		0x12U
#define G12A_DS_OFF_DIAGNOSE		0x13U
/* 0x14..0x1d: ps1 ohm shorts (leave 0); 0x1e dmem_load_size */
#define G12A_DS_OFF_DMEM_LOAD_SZ	0x1eU	/* u16 — product comment 0x1000 */
#define G12A_DS_OFF_BASE_ADDR		0x20U
#define G12A_DS_OFF_START_OFF		0x24U
#define G12A_DS_OFF_CS0_MB		0x28U
#define G12A_DS_OFF_CS1_MB		0x2aU
#define G12A_DS_OFF_SEQCTRL		0x2cU	/* u16[2] */
#define G12A_DS_OFF_PHY_ODT_RANK	0x30U	/* u8[2] */
#define G12A_DS_OFF_RANK1_CA_VREF	0x32U
#define G12A_DS_OFF_DFI_ODT		0x34U	/* u32 */
#define G12A_DS_OFF_DRAM_FREQ		0x38U	/* u16[4] */
#define G12A_DS_OFF_PLL_BYPASS		0x40U
#define G12A_DS_OFF_RDBI_WR		0x41U
#define G12A_DS_OFF_RFC_TYPE		0x42U
#define G12A_DS_OFF_LPDDR4X		0x43U
#define G12A_DS_OFF_PLL_SSC		0x44U
#define G12A_DS_OFF_CLK_DRV		0x48U
#define G12A_DS_OFF_CS_DRV		0x4aU
#define G12A_DS_OFF_AC_DRV		0x4cU
#define G12A_DS_OFF_SOC_DRV_P		0x4eU
#define G12A_DS_OFF_SOC_DRV_N		0x50U
#define G12A_DS_OFF_SOC_ODT_P		0x52U
#define G12A_DS_OFF_SOC_ODT_N		0x54U
#define G12A_DS_OFF_DRAM_DRV		0x56U
#define G12A_DS_OFF_DRAM_ODT		0x58U
#define G12A_DS_OFF_DRAM_AC_ODT		0x5aU
#define G12A_DS_OFF_SLEW_CLK		0x5cU
#define G12A_DS_OFF_SLEW_CS		0x5eU
#define G12A_DS_OFF_SLEW_AC		0x60U
#define G12A_DS_OFF_SLEW_DATA		0x62U
#define G12A_DS_OFF_VREF_OUT		0x64U
#define G12A_DS_OFF_VREF_RX		0x66U
#define G12A_DS_OFF_VREF_DRAM		0x68U
#define G12A_DS_OFF_MAX_CORE_FREQ	0x6aU	/* u16; timing.c leaves 0 */
#define G12A_DS_OFF_AC_TRACE		0x6cU	/* u8[10] */
#define G12A_DS_OFF_LPDDR4_VOUT		0x76U
#define G12A_DS_OFF_LPDDR4_X8		0x77U
#define G12A_DS_OFF_BITTIME_2D		0x7dU
#define G12A_DS_OFF_DMC_REMAP		0x80U	/* u32[5] */
#define G12A_DS_OFF_RTT_NOM		0x94U	/* u32[2] */
#define G12A_DS_OFF_CA_REMAP		0x9cU	/* u8[4] */
#define G12A_DS_OFF_DQ_REMAP		0xa0U	/* u8[32] */
/*
 * MID2 (0xFFFB4ED0) picks ds+0xc0 if ds[0xc0..0xc2] is nonzero, else the
 * pointer at ctl+0x1a0; it then walks G12A_AC_PINMUX_N bytes into PHY CSRs.
 * Product ACS is only 0xc0 bytes, so ds+0xc0 is off the end and zero.
 */
#define G12A_CTL_OFF_AC_PINMUX_PTR	0x1a0U	/* MID2 fallback table ptr */
/* BL2 fills this from its rodata at 0xFFFAE2B8; MID3 prefers ds+0xa0 */
#define G12A_CTL_OFF_DQ_REMAP_PTR	0x1b0U
#define G12A_CTL_OFF_DRAM_TYPE2		0x1b8U	/* MID2: 2 = LPDDR4 (20 entries) */
/*
 * ctl+0x1b9 has two aliases here (this one and G12A_CTL_OFF_CHAN below).
 * MID2 reads it and does `lsl w6,w6,#20` then `orr #0x20000` (TMASTER) to
 * build a CSR index — and csr bits [21:20] are the PUB's PSTATE field, not a
 * channel (PUB MASTER table: DqsPreambleControl at 020024/120024/220024/
 * 320024).  So the "_CHAN" alias is probably a misnomer for a pstate index;
 * NOT renamed, because nothing in the vendor BL2 has been read that says what
 * it writes there.  Do not use "channel" as a fact until that is settled.
 */
#define G12A_CTL_OFF_CSR_SHIFT		0x1b9U	/* MID2: shifted <<20 into CSR idx */
#define G12A_AC_PINMUX_N		20U	/* dram type 2 count (mov #0x14) */
#define G12A_DS_OFF_AC_PINMUX		0xc0U	/* u8[28] zeros on Solitude */
#define G12A_DS_OFF_DFI_PINMUX		0xdcU	/* u8[26] zeros on Solitude */
/* product timing.c dmem_load_size comment class */
#define G12A_DS_DMEM_LOAD_SIZE		0x1000U

/* Solitude board (timing.c first pstate) */
#define G12A_DS_DRAM_TYPE_LPDDR4	2U
#define G12A_DS_RANK_32B_R01_CH01	0x3U	/* CONFIG_DDR0_32BIT_RANK01_CH01 */
#define G12A_DS_RANK_32B_R0_CH01	0x8U	/* CONFIG_DDR0_32BIT_RANK0_CH01, "only for
	 * lpddr4,dram total bus width 32bit  use channel a cs0  channel b cs0"
	 * -- BSP A311D-S905D3_linux5.15/aml-comp/uboot/bl33/v2019/arch/arm/
	 * include/asm/arch-g12a/timing.h:232; value also ddr_define.h:28 */
/* ds[0x12] dram_rank_config; rank_cfg() maps it into msgblock CsPresentChA/B.
 * Default dual (product 0x03); single-rank parts are DETECTED at train time
 * (qi:N1294: CA assert id 0xa1 -> R0_CH01 retry in the OPEN_LIVE block). */
#define G12A_SOLITUDE_RANK_CFG		G12A_DS_RANK_32B_R01_CH01
/*
 * DDR_RFC_TYPE_LPDDR4_8Gbx1 = 13, per the vendor's own enum:
 *   BSP A311D_Linux2603V2.5/yocto-4.0-bsp/aml-comp/uboot/bl33/v2019/
 *   arch/arm/include/asm/arch-g12a/ddr_define.h:104-106
 *       LPDDR4_4Gbx1 11 | LPDDR4_6Gbx1 12 | LPDDR4_8Gbx1 13
 * This was 0xb -- the vendor's code for a FOUR-gigabit die under an
 * eight-gigabit name -- while the product ACS pstate-0 that this board's
 * own BL2 carries has 0x0d at ds+0x42.  Found by T7's isolation of
 * G12A_USE_PRODUCT_ACS, which reconstructed both arms byte-exactly and
 * found them differing in exactly two bytes, this being one.
 *
 * Consumed ONLY by the G12A_USE_PRODUCT_ACS=0 arm, which is not the
 * shipping arm, so the correction is inert in the built image (verified:
 * TPL byte-identical).  It must NOT be read as making that arm usable --
 * measured, =0 fails DTE with ~1700 of 2048 bad points in the top 8 MiB
 * and the chain withholds F_DDR_OK and refuses to hand off.
 * See amlogic/software/boot/g12/t7-knob-isolation.md.
 */
#define G12A_DS_RFC_LPDDR4_8GB		0xdU	/* DDR_RFC_TYPE_LPDDR4_8Gbx1 */
#define G12A_DS_CS0_MB			0xffffU	/* auto-size class (timing.c) */
#define G12A_DS_CS1_MB			0xffffU
/*
 * The density slot5 DISCOVERS on this part, stated instead of asked for.
 * Its own console output on boot stage2-2 is the measurement: "DDR cs0 size:
 * 2048MB / DDR cs1 size: 2048MB / DMC_DDR_CTRL: 00e00024"
 * (amlogic/software/boot/g12/dram-access-bringup.md).  0x0800 = 2048 MB is
 * g12a_open_density_code()'s 0x004 case, and S1's arithmetic with both
 * channels at 0x004 reproduces 0x00E00024 exactly.
 *
 * Consumed only under G12A_DS_SIZE_EXPLICIT=1, which pokes them AFTER the
 * product-ACS memcpy — unlike G12A_DS_CS0_MB / G12A_DS_CS1_MB above, which the
 * shipping G12A_USE_PRODUCT_ACS=1 arm never reads.
 */
#define G12A_DS_CS0_MB_EXPLICIT		0x0800U	/* 2048 MB — slot5's cs0 */
#define G12A_DS_CS1_MB_EXPLICIT		0x0800U	/* 2048 MB — slot5's cs1 */
#define G12A_DS_DISABLED_DBYTE		0xf0U
#define G12A_DS_AC_PINMUX_N		28U
#define G12A_DS_DFI_PINMUX_N		26U
#define G12A_DS_DQ_REMAP_N		32U

/* ctl byte offsets — RE g12a-aml-ddr-control-block.md */
#define G12A_CTL_OFF_FN0		0x00U
#define G12A_CTL_OFF_FN3		0x18U
#define G12A_CTL_OFF_FN5		0x28U
#define G12A_CTL_OFF_FN6		0x30U
#define G12A_CTL_OFF_FN7		0x38U
#define G12A_CTL_OFF_DMEM0		0x48U	/* six u64 → msg block */
#define G12A_CTL_OFF_DMEM_N		6U
#define G12A_CTL_OFF_PIEI_D333		0x78U
#define G12A_CTL_OFF_PIEI_D444		0x80U
#define G12A_CTL_OFF_PIEI_DL33		0x88U
#define G12A_CTL_OFF_PIEI_DL44		0x90U
/*
 * Vendor probe (bl2_new.bin 0xFFFA6D44) copies the type-selected PIEI pointer
 * here. Nothing in BL2 reads it back — the reader is aml_ddr.fw, which
 * dereferences it at fffb5bc4 (ldr x0,[x19,#152]) and then fffb5008
 * (ldr w1,[x0]) with NO null check. Left zero, that is a read of VA 0.
 */
#define G12A_CTL_OFF_PIEI_SEL		0x98U
#define G12A_CTL_OFF_DDR_SET		0xa0U
#define G12A_CTL_OFF_TIMING		0xa8U
#define G12A_CTL_OFF_PLL_PAR		0xb0U
#define G12A_CTL_OFF_NEED_2D		0xe8U
#define G12A_CTL_OFF_PHASE		0xeaU	/* load step: 0xf aml,0 piei,1 1d,2 2d */
#define G12A_CTL_OFF_TYPE		0x1b8U
#define G12A_CTL_OFF_CHAN		0x1b9U
#define G12A_CTL_OFF_SILENT		0x1baU
#define G12A_CTL_OFF_HDT		0x205U
#define G12A_CTL_OFF_LAST_STEP		0x25dU	/* last loaded 1d/2d step echo */
#define G12A_CTL_OFF_INIT_MODE		0x264U
/*
 * TOC cluster (16 B each: off u32, size u32, entry* u64) — RE control-block.md
 * load_ddr_fw copies → dest; open path stages entry* already at dest.
 */
#define G12A_CTL_OFF_TOC_AML0		0x0f0U
#define G12A_CTL_OFF_TOC_PIEI		0x100U
#define G12A_CTL_OFF_TOC_D333		0x110U
#define G12A_CTL_OFF_TOC_D444		0x120U
#define G12A_CTL_OFF_TOC_D422		0x130U
#define G12A_CTL_OFF_TOC_DL33		0x140U
#define G12A_CTL_OFF_TOC_DL44		0x150U	/* lpddr4_1d — entry @ 0xFFFC0000 */
#define G12A_CTL_OFF_TOC_DL40		0x160U	/* lpddr4_2d */
#define G12A_CTL_OFF_TOC_SEL_1D		0x190U	/* param_1[0x32] → TOC 1d */
#define G12A_CTL_OFF_TOC_SEL_2D		0x198U	/* param_1[0x33] → TOC 2d */

/* 2d hold = residual park at HOLD0 once aml is written direct to dest */
#define G12A_FW_HOLD_2D			G12A_FW_HOLD_AML_DDR
/* L21 (2026-09-12): delivered span, lockstep with HOLD_2D_MAX in
 * meson-usb-g12.c — see G12A_FW_HOLD_1D_MAX above. */
#define G12A_FW_HOLD_2D_MAX		0x00009000UL
/* Alt residual 1d park (after aml@HOLD0 without bounce) */
#define G12A_FW_HOLD_1D_ALT		0xFFFDC000UL

/*
 * Sticky train_status word — NOT in SPL BSS (0xFFFBC000 sits inside aml_ddr
 * scratch 0xFFFB4000..0xFFFC0000). Gap between stage/fault @ 0xFFFB3C80 and
 * aml_ddr @ 0xFFFB4000.
 */
#define G12A_TRAIN_STATUS_STICKY	0xFFFB3CF0UL

/*
 * Sticky train_status bits (handoff train_status field + getter).
 * F_DDR_OK only when G12A_TRAIN_ST_COMPLETE is set.
 */
#define G12A_TRAIN_ST_NONE		0x00000000U
#define G12A_TRAIN_ST_NO_FW		0x00000001U	/* no aml_ddr magic */
#define G12A_TRAIN_ST_FW_PRESENT	0x00000002U
#define G12A_TRAIN_ST_1D_LOADED		0x00000004U	/* 1d at HOLD_1D (+ IMEM if APB) */
#define G12A_TRAIN_ST_1D_MISSING	0x00000008U	/* no 1d at hold */
#define G12A_TRAIN_ST_KICK_1D		0x00000010U
#define G12A_TRAIN_ST_KICK_2D		0x00000020U
#define G12A_TRAIN_ST_PLL_OK		0x00000040U	/* DDR PLL lock */
#define G12A_TRAIN_ST_PLL_FAIL		0x00000080U
#define G12A_TRAIN_ST_1D_OK		0x00000100U
#define G12A_TRAIN_ST_2D_OK		0x00000200U
#define G12A_TRAIN_ST_IMEM_OK		0x00000400U	/* APB IMEM fill done */
#define G12A_TRAIN_ST_ACS_OK		0x00000800U	/* minimal DMEM ACS written */
#define G12A_TRAIN_ST_FAIL		0x00001000U
#define G12A_TRAIN_ST_TIMEOUT		0x00002000U
#define G12A_TRAIN_ST_CTL_OK		0x00004000U	/* control block built */
#define G12A_TRAIN_ST_ENTRY_RET		0x00010000U	/* aml_ddr entry returned */
#define G12A_TRAIN_ST_COMPLETE		0x00008000U	/* mailbox success */
#define G12A_TRAIN_ST_IMEM_STICKY	0x00020000U	/* IMEM first/last match src */
#define G12A_TRAIN_ST_IMEM_RETRY	0x00040000U	/* IMEM re-fill needed */
#define G12A_TRAIN_ST_2D_LOADED		0x00080000U	/* 2d staged / IMEM filled */
#define G12A_TRAIN_ST_ACS_TABLES	0x00100000U	/* full pinmux/dq/dmc tables */
#define G12A_TRAIN_ST_DRIVER_INIT	0x00200000U	/* fp_driver_init (+0x18) called */
#define G12A_TRAIN_ST_1D_STAGED		0x00400000U	/* 1d image at 0xFFFC0000 */

/*
 * SM1/G12A stage_init ACS table (product bl2_stage_init). Solitude power:
 * VCCK = AO_PWM_D, VDDEE = AO_PWM_B. Stages match product UART:
 *   0x01 → 0x81 → (hw_id/pwm_id) → 0xc0|pwm → 0x02
 * RE: g12b bl2 stage_init__ns + boot-lite aml-s905d3-cc timing.c __bl2_reg.
 */
#define G12A_BL2_STAGE_1		0x01U
#define G12A_BL2_STAGE_2		0x02U
#define G12A_BL2_STAGE_PWM_PRE		0x81U
#define G12A_BL2_STAGE_PWM_CHK_HW	0x82U
#define G12A_BL2_STAGE_PWM_CFG_GROUP	0x83U
#define G12A_BL2_STAGE_PWM_INIT		0xC0U
#define G12A_PWM_CFG0			0U
#define G12A_PWM_CFG1			1U

/*
 * AO PWM block bases.  The base register of each block IS a PWM register:
 * BSP secure_apb.h AO_PWM_PWM_C = 0xFF802000, AO_PWM_PWM_A = 0xFF807000,
 * so +0x04 / +0x08 below resolve exactly to AO_PWM_PWM_D / AO_PWM_PWM_B and
 * AO_PWM_MISC_REG_CD / AO_PWM_MISC_REG_AB.
 */
#define G12A_AO_PWM_CD_BASE		0xFF802000UL
#define G12A_AO_PWM_AB_BASE		0xFF807000UL
#define G12A_AO_PWM_PWM_D		(G12A_AO_PWM_CD_BASE + 0x04UL) /* VCCK */
#define G12A_AO_PWM_MISC_CD		(G12A_AO_PWM_CD_BASE + 0x08UL)
#define G12A_AO_PWM_PWM_B		(G12A_AO_PWM_AB_BASE + 0x04UL) /* VDDEE */
#define G12A_AO_PWM_MISC_AB		(G12A_AO_PWM_AB_BASE + 0x08UL)
/* All three are BSP secure_apb.h names, AO + (off << 2) */
#define G12A_AO_RTI_PIN_MUX_REG1	0xFF800018UL	/* AO_RTI_PIN_MUX_REG1 */
#define G12A_AO_SEC_REG0		0xFF800140UL	/* AO_SEC_REG0, (0x50<<2) */
#define G12A_AO_GPIO_O			0xFF800034UL	/* AO_GPIO_O, (0x0d<<2) */
/* G12B Alta dual-cluster enables.  BSP scp_task/secure_apb.h. */
#define G12A_AO_GPIO_O_EN_N		0xFF800024UL	/* AO_GPIO_O_EN_N, scp_task/secure_apb.h:2821 */
#define G12A_AO_RTI_PIN_MUX_REG		0xFF800014UL	/* AO_RTI_PIN_MUX_REG, scp_task/secure_apb.h:2808 */
/*
 * AO_SEC_GP_CFG0 = 0xFF800000 + (0x090 << 2) = 0xFF800240.  Vendor BSP
 * secure_apb.h (uboot/bl33/v2019/arch/arm/mach-meson/g12a/firmware/
 * scp_task/secure_apb.h:3184); it is absent from the S905D3 datasheet.
 * Layout, from the product BL2's own writes:
 *   [31:19] populated DRAM size / 8 MB   [18:16] dram_type (2 = LPDDR4)
 *   [15:8]  board ID                     [7:4] rsvd   [3:0] boot device
 * This was 0xFF800200 until 2026-08-11 — that is AO_SEC_SD_CFG0 (same
 * header :3136), which spl-g12a-scp.c already carries correctly named as
 * G12A_AO_SCP_HANDSHAKE.  The only user was the "sib" board-ID crumb,
 * which was therefore reading the SC handshake register.
 */
#ifndef G12A_AO_SEC_GP_CFG0
#define G12A_AO_SEC_GP_CFG0		0xFF800240UL	/* AO + (0x90<<2) */
#endif
/* BSP arch-g12a/regs.h:15328 ASSIST_POR_CONFIG (0x2055), P_ = <<2 + 0xffd00000 */
#define G12A_ASSIST_POR_CONFIG		0xFFD08154UL

/*
 * Solitude VCCK (VDDCPU_B, AO_PWM_PWM_D): 810 mV, the boot-lite board table's
 * VCCK_VAL == 810 entry (board/aml-s905d3-cc/firmware/timing.c:326-327,
 * VCCK = 1040 - 10 * high over a 34-count period).  The vendor SM1 OPP asks
 * 810 mV at 1.2 GHz on every bin (mesonsm1.dtsi:2570-2571); the board's
 * configured 800 mV (0x0018000a, configs/aml-s905d3-cc.h:14) is 10 mV under
 * it.  No later stage writes this register (claude amlogic
 * software/boot/g12/audit/2026-10-07-sm1-vddcpu-setpoint.md), so this is the
 * setpoint for both cpufreq OPPs.  VDDEE 840 mV (CFG1 path) as boot-lite.
 */
#define G12A_SOLITUDE_VCCK_PWM		0x0017000bU	/* 810 mV, timing.c:326-327 */
#define G12A_SOLITUDE_VDDEE_PWM_CFG0	0x000c0010U	/* 840 mV 0.67–0.97 table */
#define G12A_SOLITUDE_VDDEE_PWM_CFG1	0x0004000eU	/* 840 mV 0.69–0.89 table */
/*
 * Alta (AML-A311D-CC) boot-lite configs/aml-a311d-cc.h: VCCK 800 mV,
 * VDDEE 840 mV.  PWM_AO_D 36-step revB table is VCCK_VAL_REG1 at 800 mV
 * (board/aml-a311d-cc/firmware/timing.c).  Same duty words as Solitude.
 * Dual cluster: VCCK_A = GPIOAO_4 (A73), VCCK_B = TEST_N AO_GPIO_O[31]
 * (A53) — pwr_ctrl.c power_on_at_24M, not a second PWM channel.
 */
#define G12B_ALTA_VCCK_PWM		0x0018000aU	/* 800 mV REG1, timing.c VCCK_VAL==800 */
#define G12B_ALTA_VDDEE_PWM_CFG0	0x000c0010U	/* 840 mV 0.67–0.97 table */
#define G12B_ALTA_VDDEE_PWM_CFG1	0x0004000eU	/* 840 mV 0.69–0.89 table */
/* Product Solitude: Board ID = 1 (SARADC); ACS board_id mask 0xff */
#define G12A_SOLITUDE_BOARD_ID		1U
#define G12A_AO_SEC_BOARD_ID_SHIFT	8U
#define G12A_AO_SEC_BOARD_ID_MASK	0x0000FF00U

#ifndef __ASSEMBLY__

/**
 * g12a_stage_init_sm1 - product-like stage_init / PWM power for Solitude
 *
 * Applies ACS-equivalent register ops for stages 0x01 / 0x81 / 0xc0|pwm / 0x02
 * and sticks Board ID into AO_SEC_GP_CFG0[15:8]. Call before a644 train.
 * UART crumbs: si01 / si81 / sihw / sicN / si02 / sib (N = pwm id nibble).
 */
void g12a_stage_init_sm1(void);

/**
 * meson_g12_ddr_pll_init - Strategy B Phase 1 DDR PLL lock (FUN_fffd504c)
 *
 * Program CNTL1..6 defaults, CNTL0 reset sequence, poll bit 31.
 * Return: 0 if G12A_DDR_PLL_LOCK_BIT set, -1 on timeout.
 */
int meson_g12_ddr_pll_init(void);

/**
 * g12a_ddr_fw_stub_init - Strategy A L1.3 + L3.A.6 mailbox kick skeleton
 *
 * Clears aml_ddr scratch, optionally copies from host hold, checks magics,
 * attempts aml_ddr entry if present, then mailbox 1D/2D kick poll (skeleton —
 * real 1D/2D needs ACS/timing + PHY IMEM load, L3.A.7).
 *
 * Return: 0 always for TPL banner / residual continuity. Train success is
 *         NOT implied — use g12a_ddr_fw_train_ok() / train_status sticky for
 *         handoff F_DDR_OK (honest: no false green without mailbox complete).
 */
int g12a_ddr_fw_stub_init(void);

/**
 * g12a_ddr_fw_get_train_status - sticky train_status for handoff block
 */
unsigned int g12a_ddr_fw_get_train_status(void);

/**
 * g12a_ddr_fw_train_ok - non-zero only if mailbox reported train complete
 */
int g12a_ddr_fw_train_ok(void);

#endif /* __ASSEMBLY__ */

#endif /* __G12A_DDR_FW_H__ */
