/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2015, Amlogic, Inc. All rights reserved.
 * Copyright (C) 2023, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#ifndef DRAM_GX_H
#define DRAM_GX_H

/*
 * Registers
 */

/* PCTL */
#define DDR0_PCTL_BASE	0xc8839000
/* DDR1_PCTL_BASE is DDR0_PCTL_BASE + 0x400 */

/* DMC */
#define DMC_REG_BASE	0xc8838000

#define DMC_REQ_CTRL	(DMC_REG_BASE + (0x00 << 2))
#define DMC_SOFT_RST	(DMC_REG_BASE + (0x01 << 2))
#define DMC_SOFT_RST1	(DMC_REG_BASE + (0x02 << 2))
#define DMC_RST_STS	(DMC_REG_BASE + (0x03 << 2))
#define DMC_VERSION	(DMC_REG_BASE + (0x05 << 2))

#define DMC_REFR_CTRL1	(DMC_REG_BASE + (0x23 << 2))
#define DMC_REFR_CTRL2	(DMC_REG_BASE + (0x24 << 2))

#define DMC_PCTL_LP_CTRL	(DMC_REG_BASE + (0x46 << 2))

#define DMC_AM0_QOS_INC		(DMC_REG_BASE + (0x62 << 2))
#define DMC_AM0_QOS_DEC		(DMC_REG_BASE + (0x64 << 2))
#define DMC_AM0_QOS_DIS		(DMC_REG_BASE + (0x66 << 2))

#define DMC_AM1_QOS_INC		(DMC_REG_BASE + (0x6c << 2))
#define DMC_AM1_QOS_DEC		(DMC_REG_BASE + (0x6e << 2))
#define DMC_AM1_QOS_DIS		(DMC_REG_BASE + (0x70 << 2))

#define DMC_AM2_QOS_INC		(DMC_REG_BASE + (0x76 << 2))
#define DMC_AM2_QOS_DEC		(DMC_REG_BASE + (0x78 << 2))
#define DMC_AM2_QOS_DIS		(DMC_REG_BASE + (0x7a << 2))

#define DMC_AM3_QOS_INC		(DMC_REG_BASE + (0x80 << 2))
#define DMC_AM3_QOS_DEC		(DMC_REG_BASE + (0x82 << 2))
#define DMC_AM3_QOS_DIS		(DMC_REG_BASE + (0x84 << 2))

#define DMC_AM4_QOS_INC		(DMC_REG_BASE + (0x8a << 2))
#define DMC_AM4_QOS_DEC		(DMC_REG_BASE + (0x8c << 2))
#define DMC_AM4_QOS_DIS		(DMC_REG_BASE + (0x8e << 2))

#define DMC_AM5_QOS_INC		(DMC_REG_BASE + (0x94 << 2))
#define DMC_AM5_QOS_DEC		(DMC_REG_BASE + (0x96 << 2))
#define DMC_AM5_QOS_DIS		(DMC_REG_BASE + (0x98 << 2))

#define DMC_AM6_QOS_INC		(DMC_REG_BASE + (0x9e << 2))
#define DMC_AM6_QOS_DEC		(DMC_REG_BASE + (0xa0 << 2))
#define DMC_AM6_QOS_DIS		(DMC_REG_BASE + (0xa2 << 2))

#define DMC_AM7_QOS_INC		(DMC_REG_BASE + (0xa8 << 2))
#define DMC_AM7_QOS_DEC		(DMC_REG_BASE + (0xaa << 2))
#define DMC_AM7_QOS_DIS		(DMC_REG_BASE + (0xac << 2))

#define DMC_AXI0_QOS_INC	(DMC_REG_BASE + (0xb2 << 2))
#define DMC_AXI0_QOS_DEC	(DMC_REG_BASE + (0xb4 << 2))
#define DMC_AXI0_QOS_DIS	(DMC_REG_BASE + (0xb6 << 2))
#define DMC_AXI0_QOS_CTRL1	(DMC_REG_BASE + (0xb9 << 2))

#define DMC_AXI1_QOS_INC	(DMC_REG_BASE + (0xbc << 2))
#define DMC_AXI1_QOS_DEC	(DMC_REG_BASE + (0xbe << 2))
#define DMC_AXI1_QOS_DIS	(DMC_REG_BASE + (0xc0 << 2))

#define DMC_AXI2_QOS_INC	(DMC_REG_BASE + (0xc6 << 2))
#define DMC_AXI2_QOS_DEC	(DMC_REG_BASE + (0xc8 << 2))
#define DMC_AXI2_QOS_DIS	(DMC_REG_BASE + (0xca << 2))

#define DMC_AXI3_QOS_INC	(DMC_REG_BASE + (0xd0 << 2))
#define DMC_AXI3_QOS_DEC	(DMC_REG_BASE + (0xd2 << 2))
#define DMC_AXI3_QOS_DIS	(DMC_REG_BASE + (0xd4 << 2))

#define DMC_AXI4_QOS_INC	(DMC_REG_BASE + (0xda << 2))
#define DMC_AXI4_QOS_DEC	(DMC_REG_BASE + (0xdc << 2))
#define DMC_AXI4_QOS_DIS	(DMC_REG_BASE + (0xde << 2))

#define DMC_AXI5_QOS_INC	(DMC_REG_BASE + (0xe4 << 2))
#define DMC_AXI5_QOS_DEC	(DMC_REG_BASE + (0xe6 << 2))
#define DMC_AXI5_QOS_DIS	(DMC_REG_BASE + (0xe8 << 2))

#define DMC_AXI6_QOS_INC	(DMC_REG_BASE + (0xee << 2))
#define DMC_AXI6_QOS_DEC	(DMC_REG_BASE + (0xf0 << 2))
#define DMC_AXI6_QOS_DIS	(DMC_REG_BASE + (0xf2 << 2))

#define DMC_AXI7_QOS_INC	(DMC_REG_BASE + (0xf8 << 2))
#define DMC_AXI7_QOS_DEC	(DMC_REG_BASE + (0xfa << 2))
#define DMC_AXI7_QOS_DIS	(DMC_REG_BASE + (0xfc << 2))

/* DDR MMC */
#define AM_DDR_PLL_CNTL0	(DDR_MMC_BASE + 0x00)
#define AM_DDR_PLL_CNTL1	(DDR_MMC_BASE + 0x04)
#define AM_DDR_PLL_CNTL2	(DDR_MMC_BASE + 0x08)
#define AM_DDR_PLL_CNTL3	(DDR_MMC_BASE + 0x0c)
#define AM_DDR_PLL_CNTL4	(DDR_MMC_BASE + 0x10)
#if defined(CONFIG_MESON_GXBB)
#define AM_DDR_PLL_STS	(DDR_MMC_BASE + 0x14)
#else
#define AM_DDR_PLL_CNTL5	(DDR_MMC_BASE + 0x14)
#endif

/*
 * DDR0_CLK_CTRL is GXBB-only and lives in dram-gxbb.h.  On GXL the vendor
 * moved it: cmd/amlogic/cmd_ddr_test.c's P_DDR_PHY_905X arm defines
 * DDR_CLK_CNTL as MMC_REG_BASE + (0x7 << 2) and then
 * "#define P_DDR0_CLK_CTRL DDR_CLK_CNTL" -- one register, two vendor names,
 * at DDR_MMC_BASE + 0x1c, which dram-gxl.h already carries as DDR_CLK_CNTL.
 * dram-gxl.h defines DDR0_CLK_CTRL to 0 so the IS_ENABLED(CONFIG_MESON_GXBB)
 * arm of dram-gx.c still compiles there; that arm never runs on GXL.
 */

/* DMC SEC */
#define DMC_SEC_REG_BASE	0xda838400

#define DMC_SEC_CTRL		(DMC_SEC_REG_BASE + (0x00 << 2))
/*
 * Six secure-range windows.  Each word is [15:0] = start >> 16,
 * [31:16] = end >> 16 -- a 64 KiB granule.  DMC_SEC_CTRL bit 31 commits
 * them; DMC_SEC_RANGE_CTRL bits 0-5 carry each range's secure level and
 * bit 6 the default-region level.
 */
#define DMC_SEC_RANGE0_CTRL	(DMC_SEC_REG_BASE + (0x01 << 2))
#define DMC_SEC_RANGE1_CTRL	(DMC_SEC_REG_BASE + (0x02 << 2))
#define DMC_SEC_RANGE2_CTRL	(DMC_SEC_REG_BASE + (0x03 << 2))
#define DMC_SEC_RANGE3_CTRL	(DMC_SEC_REG_BASE + (0x04 << 2))
#define DMC_SEC_RANGE4_CTRL	(DMC_SEC_REG_BASE + (0x05 << 2))
#define DMC_SEC_RANGE5_CTRL	(DMC_SEC_REG_BASE + (0x06 << 2))
#define DMC_SEC_RANGE_CTRL	(DMC_SEC_REG_BASE + (0x07 << 2))
#define DMC_SEC_AXI_PORT_CTRL	(DMC_SEC_REG_BASE + (0x0e << 2))

#define DMC_VDEC_SEC_READ_CTRL		(DMC_SEC_REG_BASE + (0x10 << 2))
#define DMC_VDEC_SEC_WRITE_CTRL		(DMC_SEC_REG_BASE + (0x11 << 2))
#define DMC_VDEC_SEC_CFG		(DMC_SEC_REG_BASE + (0x12 << 2))

#define DMC_HCODEC_SEC_READ_CTRL	(DMC_SEC_REG_BASE + (0x17 << 2))
#define DMC_HCODEC_SEC_WRITE_CTRL	(DMC_SEC_REG_BASE + (0x18 << 2))
#define DMC_HCODEC_SEC_CFG		(DMC_SEC_REG_BASE + (0x19 << 2))

#define DMC_HEVC_SEC_READ_CTRL		(DMC_SEC_REG_BASE + (0x1e << 2))
#define DMC_HEVC_SEC_WRITE_CTRL		(DMC_SEC_REG_BASE + (0x1f << 2))
#define DMC_HEVC_SEC_CFG		(DMC_SEC_REG_BASE + (0x20 << 2))

#define DMC_VPU_SEC_READ_CTRL	(DMC_SEC_REG_BASE + (0x32 << 2))
#define DMC_VPU_SEC_WRITE_CTRL	(DMC_SEC_REG_BASE + (0x33 << 2))
#define DMC_VPU_SEC_CFG		(DMC_SEC_REG_BASE + (0x25 << 2))

#define DMC_GE2D_SEC_CTRL	(DMC_SEC_REG_BASE + (0x34 << 2))
#define DMC_PARSER_SEC_CTRL	(DMC_SEC_REG_BASE + (0x35 << 2))
#define DMC_DEV_SEC_READ_CTRL	(DMC_SEC_REG_BASE + (0x36 << 2))
#define DMC_DEV_SEC_WRITE_CTRL	(DMC_SEC_REG_BASE + (0x37 << 2))

#define DMC_WTCH0_CTRL	(DMC_SEC_REG_BASE + (0xa9 << 2))
#define DMC_WTCH1_CTRL	(DMC_SEC_REG_BASE + (0xb0 << 2))

#define DDR0_ADDRMAP_0	(DMC_SEC_REG_BASE + (0xd0 << 2))
#define DDR0_ADDRMAP_1	(DMC_SEC_REG_BASE + (0xd1 << 2))
#define DDR0_ADDRMAP_2	(DMC_SEC_REG_BASE + (0xd2 << 2))
#define DDR0_ADDRMAP_3	(DMC_SEC_REG_BASE + (0xd3 << 2))
#define DDR0_ADDRMAP_4	(DMC_SEC_REG_BASE + (0xd4 << 2))

#define DDR1_ADDRMAP_0	(DMC_SEC_REG_BASE + (0xd5 << 2))
#define DDR1_ADDRMAP_1	(DMC_SEC_REG_BASE + (0xd6 << 2))
#define DDR1_ADDRMAP_2	(DMC_SEC_REG_BASE + (0xd7 << 2))
#define DDR1_ADDRMAP_3	(DMC_SEC_REG_BASE + (0xd8 << 2))
#define DDR1_ADDRMAP_4	(DMC_SEC_REG_BASE + (0xd9 << 2))

#if defined(CONFIG_MESON_GXL)
#define DMC_DES_KEY0_H	(DMC_SEC_REG_BASE + (0x90 << 2))
#define DMC_DES_KEY0_L	(DMC_SEC_REG_BASE + (0x91 << 2))
#define DMC_DES_KEY1_H	(DMC_SEC_REG_BASE + (0x92 << 2))
#define DMC_DES_KEY1_L	(DMC_SEC_REG_BASE + (0x93 << 2))

#define DMC_DES_CTRL	(DMC_SEC_REG_BASE + (0x9d << 2))
#endif

#define DMC_DDR_CTRL	(DMC_SEC_REG_BASE + (0xda << 2))

#define AM_ANALOG_TOP_REG1	(0xc8834400 + (0x6f << 2))

/* Macros */
#define DQSCORR_DX(dx)							\
	if ((readl(dx) & ~(0xe00)) && ((readl(dx) >> 8) & ~(0xe00)))	\
		writel((((readl(dx) & ~(0xe00)) * 95) / 100) |		\
			(((((readl(dx) >> 8) & ~(0xe00)) * 88) / 100) << 8) | \
			(((((readl(dx) >> 8) & ~(0xe00)) * 88) / 100) << 16), \
			dx);						\
	else if (((readl(dx) >> 8) & ~(0xe00)))				\
		writel((95 / 100) |					\
			(((((readl(dx) >> 8) & ~(0xe00)) * 88) / 100) << 8) | \
			(((((readl(dx) >> 8) & ~(0xe00)) * 88) / 100) << 16), \
			dx);						\
	else if (((readl(dx)) & ~(0xe00)))				\
		writel((((readl(dx) & ~(0xe00)) * 95) / 100) |		\
			(((88) / 100) << 8) | (((88) / 100) << 16),	\
			dx);						\
	else								\
		writel((95 / 100) |					\
			((88 / 100) << 8) | ((88 / 100) << 16), dx)

#define DMC_ENABLE_REGION(REGION)					\
	writel(0xffffffff, REGION## _SEC_CFG);				\
	writel(0x55555555, REGION## _SEC_WRITE_CTRL);			\
	writel(0x55555555, REGION## _SEC_READ_CTRL)

/*
 * WAIT_FOR() was an UNBOUNDED spin whose only refusal could not fire.  The
 * loop `while (!(readl(a) & 1));` exits only when bit 0 is set, so the
 * `if (!(readl(a) & 1)) panic(...)` beneath it is dead code -- the same class
 * as a negative control whose verdict cannot execute, which this arc has now
 * found four times.  Its `TODO: Timeout` had been standing since the file was
 * written, and N27 measured what that costs: with the retry ladder driven into
 * a second attempt, dram_init()'s third attempt WEDGES inside one of these
 * spins, printing nothing for a full 45 s drain -- no `DDR init FAILED after
 * 3 attempts`, no SPL refusal.  A ladder that cannot say it exhausted is
 * indistinguishable from a hang anywhere else in the stage.
 *
 * Bounded, named, report-and-continue: the callers that CAN refuse do so
 * through meson_dram_wait_bit0()'s return value; the ones that cannot (the
 * GXBB arm, which this board family does not ship) at least leave a line on
 * the console instead of a dark board.
 */
#define WAIT_FOR(a)	meson_dram_wait_bit0((a), #a)

/**
 * Register values
 **/

/*
 * PLL
 */
#define DDR_CLK_CNTL_CLKGEN_SOFTRESET	BIT(28)
#define DDR_CLK_CNTL_PHY_CLK_ENABLE	BIT(29)
#define DDR_CLK_CNTL_DDRPLL_ENABLE	BIT(31)

/*
 * PCTL
 */

/* PCTL_SCTL: state control register (S905X datasheet p.451) */
#define PCTL_SCTL_CFG_STATE	BIT(0)
#define PCTL_SCTL_GO_STATE	BIT(1)

/* PCTL_STAT */
#define PCTL_STAT_ACCESS	(BIT(1) | BIT(0))

/*
 * Bound shared by every DRAM-init wait: 100 ms, ~4 orders of magnitude over
 * any transition this hardware makes.
 *
 * It is a Kconfig symbol only so that N27's failure path can be reached on
 * healthy silicon: shortened, these waits genuinely time out, and
 * `PCTL ACCESS timeout!` / `DDR wait <who>: bit0 never set` -- neither of
 * which a working board can produce -- print through the shipping code with
 * nothing injected.  Both spellings are compile-time constants, so at the
 * default the generated image is unchanged.  See
 * MESON_DRAM_PCTL_STATE_TIMEOUT_US in arch/arm/mach-meson/Kconfig.
 */
#if defined(CONFIG_MESON_DRAM_PCTL_STATE_TIMEOUT_US) && \
	CONFIG_MESON_DRAM_PCTL_STATE_TIMEOUT_US > 0
#define PCTL_STATE_TIMEOUT_US	CONFIG_MESON_DRAM_PCTL_STATE_TIMEOUT_US
#else
#define PCTL_STATE_TIMEOUT_US	100000
#endif

/* Bounded bit-0 wait behind WAIT_FOR().  dram-gx.c. */
int meson_dram_wait_bit0(unsigned long a, const char *who);

/*
 * Whether THIS dram_init() attempt is training and decoding a rank 1.  Set by
 * the ladder in dram-gx.c, read by the DTCR1 rank-set write in dram-gxl.c so
 * the training bit and the DMC range can never disagree.  One SKU, one config:
 * the build declares the maximum rank count and the board settles the rest.
 */
extern bool meson_dram_rank1_active;

/* PCTL_POWCTL: power control */
#define PCTL_POWCTL_POWERON	BIT(0)

/*
 * PUB
 */

/* PUB_PGSR0: PHY General Status Register 0 */
#define PUB_PGSR0_IDONE		BIT(0)	/* Initialization Done */
#define PUB_PGSR0_PLDONE	BIT(1)	/* PLL Lock Done */
#define PUB_PGSR0_DCDONE	BIT(2)	/* DDL Calibration Done */
#define PUB_PGSR0_ZCDONE	BIT(3)	/* Impedance Calibration Done */
#define PUB_PGSR0_DIDONE	BIT(4)	/* DRAM Initialization Done */
#define PUB_PGSR0_WLDONE	BIT(5)	/* Write Leveling Done */
#define PUB_PGSR0_QSGDONE	BIT(6)	/* DQS Gate Training Done */
#define PUB_PGSR0_WLADONE	BIT(7)	/* Write Leveling Adjust Done */
#define PUB_PGSR0_RDDONE	BIT(8)	/* Read Bit Deskew Done */
#define PUB_PGSR0_WDDONE	BIT(9)	/* Write Bit Deskew Done */
#define PUB_PGSR0_REDONE	BIT(10)	/* Read Eye Training Done */
#define PUB_PGSR0_WEDONE	BIT(11)	/* Write Eye Training Done */
#define PUB_PGSR0_ZCERR		BIT(20)	/* Impedance Calib Error */
#define PUB_PGSR0_WLERR		BIT(21)	/* Write Leveling Error */
#define PUB_PGSR0_QSGERR	BIT(22)	/* DQS Gate Training Error */
#define PUB_PGSR0_WLAERR	BIT(23)	/* Write Leveling Adj Error */
#define PUB_PGSR0_RDERR		BIT(24)	/* Read Bit Deskew Error */
#define PUB_PGSR0_WDERR		BIT(25)	/* Write Bit Deskew Error */
#define PUB_PGSR0_REERR		BIT(26)	/* Read Eye Training Error */
#define PUB_PGSR0_WEERR		BIT(27)	/* Write Eye Training Error */

/* PUB_PIR: PHY init register */
#define PUB_PIR_INIT		BIT(0)
#define PUB_PIR_ZCAL		BIT(1)
#define PUB_PIR_CA		BIT(2)

#define PUB_PIR_PLLINIT		BIT(4)
#define PUB_PIR_DCAL		BIT(5)
#define PUB_PIR_PHYRST		BIT(6)
#define PUB_PIR_DRAMRST		BIT(7)
#define PUB_PIR_DRAMINIT	BIT(8)
#define PUB_PIR_WL		BIT(9)
#define PUB_PIR_QSGATE		BIT(10)
#define PUB_PIR_WLADJ		BIT(11)
#define PUB_PIR_RDDSKW		BIT(12)
#define PUB_PIR_WRDSKW		BIT(13)
#define PUB_PIR_RDEYE		BIT(14)
#define PUB_PIR_WREYE		BIT(15)
#define PUB_PIR_ICPC		BIT(16)
#define PUB_PIR_VREF		BIT(17)	/* DDR4 VREF training (shared with PLLBYP) */
#define PUB_PIR_PLLBYP		BIT(17)
#define PUB_PIR_CTLDINIT	BIT(18)
#define PUB_PIR_RDIMMINIT	BIT(19)
/*
 * PUB_PIR_CLRSR is documented by the vendor but is MEASURED NOT TO EXIST on
 * this PHY: writing BIT(27) alone, or with PUB_PIR_INIT, reads back 0 and moves
 * no PGSR0 bit (DDR3 arc; N95/N104/N559).  It has NO user in this tree -- the
 * one function that wrote it was removed in the N560 landing, see the block
 * where it used to live in dram-gx.c.  Kept here because the header documents
 * the register, not our use of it; do not write it.
 */
#define PUB_PIR_CLRSR		BIT(27)
#define PUB_PIR_LOCKBYP		BIT(28)
#define PUB_PIR_DCALBYP		BIT(29)
#define PUB_PIR_ZCALBYP		BIT(30)
#define PUB_PIR_INITBYP		BIT(31)

#define PUB_PIR_FINAL_STEP	(PUB_PIR_INIT | PUB_PIR_ZCAL |			\
	PUB_PIR_PLLINIT | PUB_PIR_DCAL | PUB_PIR_PHYRST | PUB_PIR_DRAMRST |	\
	PUB_PIR_DRAMINIT | PUB_PIR_WL | PUB_PIR_QSGATE | PUB_PIR_WLADJ |	\
	PUB_PIR_RDDSKW | PUB_PIR_WRDSKW | PUB_PIR_RDEYE | PUB_PIR_WREYE)

/* Struct which holds timings (see dram-settings-gx.h) */
struct meson_gx_dram_timings {
	u8 drv;
	u8 odt;
	u8 rtp;
	u8 wtr;
	u8 rp;
	u8 rcd;
	u8 ras;
	u8 rrd;
	u8 rc;
	u8 mrd;
	u8 mod;
	u8 faw;
	u8 wlmrd;
	u8 wlo;
	ushort rfc;
	u8 xp;
	ushort xs;
	ushort dllk;
	u8 cke;
	u8 rtodt;
	u8 rtw;
	u8 refi;
	u8 refi_mddr3;
	u8 cl;
	u8 wr;
	u8 cwl;
	u8 al;
	u8 dqs;
	u8 cksre;
	u8 cksrx;
	u8 zqcs;
	u8 xpdll;
	ushort exsr;
	ushort zqcl;
	ushort zqcsi;
	u8 rpab;
	u8 rppb;
	u8 tccdl;
	u8 tdqsck;
	u8 tdqsckmax;
	u8 tckesr;
	u8 tdpd;
	u8 taond_aofd;
};

#if defined(CONFIG_MESON_GXBB)
# include <asm/arch/dram-gxbb.h>
#elif defined(CONFIG_MESON_GXL)
# include <asm/arch/dram-gxl.h>
#endif

/* Functions */
int dram_init(void);
int meson_dram_prepare_pctl(void);
int meson_dram_phy_init(void);
void meson_dram_phy_setup_ranks(void);
int meson_dram_finalise_init(void);
/*
 * Not const: on GXL DDR3 dram_init()'s clock ladder re-derives the
 * clock-dependent fields before each attempt (gxl_ddr3_timings_at()).
 */
extern struct meson_gx_dram_timings timings;
/* The clock the current or last dram_init() attempt trained at, in MHz. */
extern u32 meson_dram_clk;
#endif
