// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic G12A/SM1 TPL USB residual — minimal DWC2 slave skeleton (Option 2).
 *
 * After train stubs, soft-disconnect and soft-connect so the host can re-enum
 * as 1b8e:c003 and push DRAM SPL (+ optional train holds) via worldcup
 * writeLargeMemory.  Not product Stage 1.8 / AMLC; not GXL meson-usb.c.
 *
 * Address map (G12 / SM1 ONLY — do not copy GXL 0xc910 / 0xd007 / 0xd900):
 *
 *   USB glue (meson-g12a-usb-ctrl)  0xFFE09000  size 0xa0
 *     U2P port i                   base + i*0x20
 *     USB_R0..R5                   base + 0x80 + off
 *   DWC2 OTG (peripheral)          0xFF400000  size 0x40000
 *   DWC3 (host; leave alone)       0xFF500000
 *   USB2 PHY0 (host A)             0xFF636000  (DTS phy@36000 under 0xff60)
 *   USB2 PHY1 (OTG B / residual)   0xFF63A000  (DTS phy@3a000; DWC2 phys)
 *   HIU                            0xFF63C000
 *   HHI_GCLK_MPEG1                 HIU + 0x144
 *
 * Sources: meson-g12-common.dtsi, dwc3-meson-g12a.c, g12a_bl2_usbboot.c RE
 * (AML_USBPHY20/21), open-tpl-usb-residual.md Option 2.
 *
 * UART crumbs (AO UART 0xff803000, same as spl.c early path):
 *   U  — worldcup soft-connect window open (id1 expected)
 *   I  — first identify (02 23) completed
 *   M  — mid-session reclaim start (post-id, no SFTDISCON)
 *   F  — FIFO map rewritten (no CSFTRST; CSFTRST killed host bulk)
 *   C  — mid-session reclaim done (FIFO+EP0+EP2 armed for bulk)
 *   B  — bulk-mode arm marker (legacy / dump)
 *   r  — first EP2 OUTRX byte landed (any bulk)
 *   R  — allowlisted bulk payload started/complete (WR_LARGE or G12R)
 *   +  — payload complete; followed by 8-hex first word of dest
 *   u  — residual window ended without usable SPL head at 0x08000000
 *   f  — RXFLVL sticky while polling
 *   y  — DOEPTSIZ2 shrunk vs arm
 *   X  — DOEPINT2.XFERCOMPL
 *
 * Mid-session residual (2026-08-09, SFTDISCON re-attach BLOCKED):
 *   worldcup id1 (02 23 / sSixOE806) → FIFO-only reclaim (NO CSFTRST —
 *   CSFTRST mid-session killed host bulk USBError5; NO SFTDISCON; NO PHY
 *   POR) → GRXFSIZ=512 + EP2 arm → crumb MFC → host bulk-only G12R → R/+.
 *   Multi-frame: host may send aml_ddr + piei + SPL; R per completed frame;
 *   EP2 soft re-arm between frames (no EPDIS); short packet ends G12R but
 *   EP2 stays live; residual exits after ~3 s quiet post last frame.
 *   HOLD0 concat split: one G12R dest=HOLD0 len=aml+piei[+1d]
 *   payload=aml||piei[||1d] → HOLD0 + PIEI [+ HOLD_1D]; one R.
 *   SFTDISCON re-session path is parked (NO_REENUM on phys4).
 *   Call order (spl.c): residual BEFORE dram_init so holds feed train.
 *
 * Size: keep under TPL 0xf000 ceiling.  Full GXL ACM/WDT product modes are
 * intentionally absent.  Gate: CONFIG_TPL_MESON_G12A_USB_RESIDUAL.
 */

#include <chip_serial.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/types.h>

/* ---------- G12 bases (NOT GXL) ---------- */

#define G12_DWC2_BASE		0xFF400000UL
#define G12_USB_GLUE_BASE	0xFFE09000UL
#define G12_USB2_PHY1_BASE	0xFF63A000UL	/* OTG PHY for DWC2 */
#define G12_HIU_BASE		0xFF63C000UL
/* meson-gxbb-wdt @ cbus 0xffd00000 + 0xf0d0 (meson-g12-common.dtsi) */
#define G12_WDT_BASE		0xFFD0F0D0UL
/*
 * reset-controller@1004 under 0xffd00000 → 0xFFD01004
 * (amlogic,meson-axg-reset / meson8b pulse bank).  Do NOT pulse RESET_USB
 * (id 34) — GXL measured soft-connect death.  PHY21 pulse is linux phy_init.
 */
#define G12_RESET_BASE		0xFFD01004UL
#define G12_RESET_USB_PHY21	49u	/* RESET1 bit17; dt-bindings */

#define HHI_GCLK_MPEG1		0x144
/* Same MPEG1 USB gate bits as GXL family HIU (BROM leaves USB up after pass-1) */
#define HHI_MPEG1_USB0		BIT(21)
#define HHI_MPEG1_USB1		BIT(22)
#define HHI_MPEG1_USB		BIT(26)

/* Glue: U2P per-port + USB_R* at +0x80 (dwc3-meson-g12a.c) */
#define U2P_REG_SIZE		0x20
#define U2P_R0			0x00
#define  U2P_R0_HOST_DEVICE	BIT(0)
#define  U2P_R0_POWER_OK	BIT(1)
#define  U2P_R0_POWER_ON_RESET	BIT(3)
#define  U2P_R0_ID_PULLUP	BIT(4)
#define  U2P_R0_DRV_VBUS	BIT(5)
#define U2P_R1			0x04
#define  U2P_R1_PHY_READY	BIT(0)
#define  U2P_R1_ID_DIG		BIT(1)
#define  U2P_R1_OTG_SESSION_VALID BIT(2)
#define  U2P_R1_VBUS_VALID	BIT(3)

#define USB_R0			0x80
#define  USB_R0_U2D_ACT		BIT(31)
#define  USB_R0_U2D_SS_SCALEDOWN_MASK	GENMASK(30, 29)
#define USB_R1			0x84
#define  USB_R1_U3H_FLADJ_30MHZ_MASK	GENMASK(24, 19)
#define USB_R4			0x90
#define  USB_R4_P21_PORT_RESET_0 BIT(0)
#define  USB_R4_P21_SLEEP_M0	BIT(1)
#define USB_R5			0x94
#define  USB_R5_ID_DIG_EN_0	BIT(4)
#define  USB_R5_ID_DIG_EN_1	BIT(5)
#define  USB_R5_ID_DIG_TH_MASK	GENMASK(15, 8)

/* OTG PHY is port 1 (usb2-phy1); DWC2 phys = <&usb2_phy1> */
#define U2P1_R0			(U2P_REG_SIZE * 1 + U2P_R0)
#define U2P1_R1			(U2P_REG_SIZE * 1 + U2P_R1)

/*
 * Synopsys UTMI+ USB2 PHY (phy-meson-g12a-usb2.c) — NOT GXL U2P_R0 layout.
 * GXL residual POR touches Amlogic U2P at 0xd0078020; G12 must re-init
 * PHY_CTRL at 0xFF63A000 after glue U2P POR for D+/pullup to re-assert.
 */
#define PHY_CTRL_R3		0x0c
#define PHY_CTRL_R4		0x10
#define PHY_CTRL_R13		0x34
#define PHY_CTRL_R14		0x38
#define PHY_CTRL_R16		0x40
#define PHY_CTRL_R17		0x44
#define PHY_CTRL_R18		0x48
#define PHY_CTRL_R20		0x50
#define PHY_CTRL_R21		0x54
#define  PHY_CTRL_R21_USB2_OTG_ACA_EN BIT(2)
#define  PHY_CTRL_R16_MPLL_M_MASK	GENMASK(8, 0)
#define  PHY_CTRL_R16_MPLL_N_MASK	GENMASK(14, 10)
#define  PHY_CTRL_R16_MPLL_LOAD		BIT(22)
#define  PHY_CTRL_R16_MPLL_LOCK_LONG_MASK GENMASK(25, 24)
#define  PHY_CTRL_R16_MPLL_FAST_LOCK	BIT(27)
#define  PHY_CTRL_R16_MPLL_EN		BIT(28)
#define  PHY_CTRL_R16_MPLL_RESET	BIT(29)
#define  PHY_CTRL_R13_UPDATE_PMA_SIGNALS BIT(15)
#define  PHY_CTRL_R13_MIN_COUNT_FOR_SYNC_DET_MASK GENMASK(20, 16)
#define  PHY_CTRL_R20_USB2_OTG_VBUSDET_EN BIT(4)
#define  PHY_CTRL_R20_USB2_EDGE_DRV_EN BIT(13)
#define  PHY_CTRL_R4_TEST_BYPASS_MODE_EN BIT(27)

/* EE WDT (same layout as GXL meson_gxbb_wdt; base differs) */
#define G12_WDT_CTRL		0x0
#define G12_WDT_TCNT		0x8
#define G12_WDT_RSET		0xc
#define G12_WDT_EN		BIT(18)
#define G12_WDT_CLK_EN		BIT(24)
#define G12_WDT_CLKDIV_EN	BIT(25)
#define G12_WDT_DIV_1MS		24000u

#ifndef GENMASK
#define GENMASK(h, l) (((~0UL) << (l)) & (~0UL >> (sizeof(long) * 8 - 1 - (h))))
#endif

/* DWC2 core */
#define GOTGCTL		0x000
#define GAHBCFG		0x008
#define GUSBCFG		0x00c
#define GRSTCTL		0x010
#define GINTSTS		0x014
#define GINTMSK		0x018
#define GRXSTSR		0x01c	/* peek RX status (no pop) */
#define GRXSTSP		0x020
#define GRXFSIZ		0x024
#define GNPTXFSIZ	0x028
#define GNPTXSTS	0x02c
#define DIEPTXF1	0x104
#define DCFG		0x800
#define DCTL		0x804
#define DSTS		0x808
#define DIEPMSK		0x810
#define DOEPMSK		0x814
#define DAINT		0x818
#define DAINTMSK	0x81c
#define DIEPCTL0	0x900
#define DIEPINT0	0x908
#define DIEPTSIZ0	0x910
#define DIEPDMA0	0x914
#define DOEPCTL0	0xb00
#define DOEPINT0	0xb08
#define DOEPTSIZ0	0xb10
#define DOEPDMA0	0xb14
#define DOEPCTL2	0xb40
#define DOEPINT2	0xb48
#define DOEPTSIZ2	0xb50
#define DOEPDMA2	0xb54
#define GHWCFG2		0x048
#define EP0_FIFO	0x1000
#define EP2_FIFO	0x3000

/* GAHBCFG: DMA_EN (cleared for slave PIO; linux dwc2) */
#define GAHBCFG_DMA_EN		BIT(5)

/* One HS bulk packet: EP2 OUT arm size and bulk_stage bound. */
#define DMA_BULK_BYTES	512u

/* GNPTXSTS: [15:0] NP TX FIFO free dwords; [19:16] NP TX queue slots */
#define GNPTXSTS_NPTXQ_SHIFT	16
#define GNPTXSTS_NPTXQ_MASK	(0xfu << GNPTXSTS_NPTXQ_SHIFT)

#define GOTGCTL_VBVALOEN	BIT(2)
#define GOTGCTL_VBVALOVAL	BIT(3)
#define GOTGCTL_BVALOEN		BIT(6)
#define GOTGCTL_BVALOVAL	BIT(7)

#define CSFTRST		BIT(0)
#define RXFFLSH		BIT(4)
#define TXFFLSH		BIT(5)
#define TXFNUM_ALL	(0x10 << 6)
#define AHBIDLE		BIT(31)

#define FORCEDEVMODE	BIT(30)
#define USBTRDTIM_5	(5 << 10)

#define INT_RXFLVL	BIT(4)
#define INT_GOUTNAKEFF	BIT(7)
#define INT_USBRST	BIT(12)
#define INT_ENUMDONE	BIT(13)
#define INT_IEPINT	BIT(18)
#define INT_OEPINT	BIT(19)

#define SFTDISCON	BIT(1)
#define DCTL_CGOUTNAK	BIT(10)	/* clear global OUT NAK */
#define DCTL_SGOUTNAK	BIT(9)
#define DEVSPD_HS	0

#define DOEPINT_XFERCOMPL	BIT(0)
#define DOEPINT_SETUP		BIT(3)
#define DOEPINT_STUPPKTRCVD	BIT(15)
#define DIEPINT_XFERCOMPL	BIT(0)

#define EPENA		BIT(31)
#define EPDIS		BIT(30)
#define SETD0PID	BIT(28)	/* DxEPCTL: next data PID DATA0 (bulk) */
#define SNAK		BIT(27)
#define CNAK		BIT(26)
#define STALL		BIT(21)
#define USBACTEP	BIT(15)
#define EPTYPE_BULK	(2 << 18)
#define SUPCNT3		(3 << 29)
#define PKTCNT1		BIT(19)
#define DIEPINT_EPDISBLD	BIT(1)

#define PKTSTS(s)	(((s) >> 17) & 0xf)
#define EPNUM(s)	((s) & 0xf)
#define BCNT(s)		(((s) >> 4) & 0x7ff)
#define PKTSTS_OUTRX	2
#define PKTSTS_OUTDONE	3
#define PKTSTS_SETUPDONE 4
#define PKTSTS_SETUPRX	6

#define FIFOSIZE(start, depth)	(((depth) << 16) | (start))

#define R(off)		readl((void __iomem *)(G12_DWC2_BASE + (off)))
#define W(off, v)	writel((v), (void __iomem *)(G12_DWC2_BASE + (off)))
#define SET(off, m)	W((off), R(off) | (m))
#define CLR(off, m)	W((off), R(off) & ~(u32)(m))

#define USB_VID		0x1b8e
#define USB_PID		0xc003
#define REQ_WR_LARGE_MEM	0x11
#define REQ_RUN_IN_ADDR		0x05
#define REQ_IDENTIFY_HOST	0x20
#define REQ_READ_MEM		0x02

/*
 * Identify: distinct from BROM (02 04 …) and GXL SPL (02 22 …).
 * Host scaffold looks for 02 23 as G12 TPL residual fingerprint.
 */
static const u8 identify_rsp[8] = { 0x02, 0x23, 0x00, 0x20, 0, 0, 0, 0 };

/*
 * TPL_MESON_USB_REENUM (the *_libre_dfu_defconfig targets only): the TPL
 * re-enumerates as its own device, and its iSerialNumber is the chip id.
 */
#define USB_REENUM	CONFIG_IS_ENABLED(MESON_USB_REENUM)
#define STR_IDX_SERIAL	3

static const u8 dev_desc[18] = {
	18, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
	(USB_VID & 0xff), (USB_VID >> 8),
	(USB_PID & 0xff), (USB_PID >> 8),
	0x00, 0x01, 0x00, 0x00, USB_REENUM ? STR_IDX_SERIAL : 0, 0x01,
};

#if USB_REENUM
/*
 * The chip id as a UTF-16LE string descriptor -- 24 hex digits, 50 bytes,
 * one EP0 packet; the same value every later stage reports
 * (design/software/device-identity.md).
 */
static const u8 str_desc_langid[4] = { 4, 0x03, 0x09, 0x04 };	/* en-US */
static u8 str_desc_serial[2 + 2 * 24];

static int str_serial_build(void)
{
	char id[25];
	int n;

	if (chip_serial_get(id, sizeof(id)))
		return 0;
	for (n = 0; n < 24 && id[n]; n++) {
		str_desc_serial[2 + 2 * n] = id[n];
		str_desc_serial[3 + 2 * n] = 0;
	}
	str_desc_serial[1] = 0x03;		/* STRING */
	str_desc_serial[0] = 2 + 2 * n;		/* bLength */
	return str_desc_serial[0];
}
#endif

#define CFG_LEN 32
static const u8 cfg_desc[CFG_LEN] = {
	9, 0x02, CFG_LEN, 0, 1, 1, 0, 0x80, 250,
	9, 0x04, 0, 0, 2, 0xff, 0x00, 0x00, 0,
	7, 0x05, 0x02, 0x02, 0x00, 0x02, 0,	/* EP2 OUT bulk 512 */
	7, 0x05, 0x81, 0x02, 0x00, 0x02, 0,	/* EP1 IN bulk 512 */
};

/* Fixed dest allowlist — train parks + DRAM SPL + FIT stage */
#define DEST_SPL	0x08000000u
/*
 * Must equal CONFIG_SPL_LOAD_FIT_ADDRESS.  NOT 0x05000000: the FIT is
 * ~1.4 MB and 0x05000000 puts it straight through BL31_BASE 0x05100000,
 * whose load then corrupts the u-boot loadable that has not been copied
 * out yet.  CONFIG_SPL_LOAD_FIT_ADDRESS is set to 0x02000000 by the
 * configs/aml-s905d3-cc_libre_*_defconfig open-chain targets.
 */
#define DEST_FIT	0x02000000u
#define DEST_HOLD0	0xFFFD0000u
#define DEST_HOLD1	0xFFFE0000u
#define DEST_AML_DDR	0xFFFB4000u
#define DEST_PIEI	0xFFFCC000u
/* Vendor PHY staging; abuts piei @ 0xFFFCC000 — max 0xC000 */
#define DEST_HOLD_1D	0xFFFC0000u
/* After aml @ HOLD0 (48080): free from ~0xFFFDBB90 — park 1d here */
#define DEST_HOLD_1D_ALT	0xFFFDC000u

/* Bulk-only residual magic: 'G12R' little-endian */
#define G12R_MAGIC	0x52323147u

/*
 * HOLD0 concat split (bypass dual-frame R2 wall): one G12R to DEST_HOLD0
 * with len = aml + piei [+ 1d [+ 2d]]:
 *   no 2d:  [0,AML)→HOLD0, piei→DEST_PIEI, 1d→HOLD0 (aml bounce→HOLD_1D)
 *   with 2d (len > piei+1d): aml→DEST_AML_DDR (frees HOLD0), piei→DEST_PIEI,
 *     1d→HOLD_1D, 2d→HOLD0. Max after-aml = piei+1d+2d (~108 KiB).
 */
/*
 * Second-window split.  DEST_SPL + len > SPL_FIT_SPLIT means the payload is
 * the binman image from CONFIG_MESON_G12A_SPL_OFFSET onward, i.e. exactly
 * out/<board>.usb.payload:
 *   [0, 0x30000)  DRAM stage  -> DEST_SPL 0x08000000
 *   [0x30000, …)  boot FIT    -> DEST_FIT 0x02000000
 * 0x30000 is FIT_OFFSET - SPL_OFFSET, the same delta the on-media layout
 * uses, so the wire format is the media format with the header cut off.
 * The FIT cannot be length-derived from its own FDT header: binman writes
 * it with EXTERNAL data, so fdt totalsize is 0x800 while the real region
 * is ~1.3 MB.
 */
#define SPL_FIT_SPLIT		0x30000u

#define AML_DDR_SPLIT_SIZE	48080u
#define PIEI_SPLIT_SIZE		10088u
/*
 * L21 blob-arm DMEM-tail measurement (2026-09-12).  The shipped section is
 * IMEM 0x8000 + the 0x1000 of DMEM either loader ever copies into the PHY;
 * payload [0x9000,0xC000) is all-zero in both vendor training blobs and is
 * never staged.  The 0xC000 DESTINATION window is unchanged -- only what
 * arrives on the wire shrinks.  Must move in lockstep with
 * G12A_FW_HOLD_{1D,2D}_MAX in g12a-ddr-fw.h and with HOLD_{1D,2D}_MAX in
 * amlogic/software/tools/mk-g12a-residual-concat.py.
 */
#define HOLD_1D_MAX		0x9000u
#define HOLD_2D_MAX		0x9000u
#define CONCAT_AFTER_AML_MAX	(PIEI_SPLIT_SIZE + HOLD_1D_MAX + HOLD_2D_MAX)

/* AO UART crumbs (same as spl.c G12 path) */
#define EARLY_UART_WFIFO	0xff803000UL
#define EARLY_UART_STATUS	0xff80300cUL

static void g12_putc(char c)
{
	int t = 100000;

	while ((readl((void *)EARLY_UART_STATUS) & (1 << 21)) && --t)
		;
	if (t)
		writel(c, (void *)EARLY_UART_WFIFO);
}

static void g12_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			g12_putc('\r');
		g12_putc(*s++);
	}
}

/* Compact 8-hex UART line: "TAG=xxxxxxxx\n" (post-id dump). */
static void g12_hex8(u32 v)
{
	static const char hx[] = "0123456789abcdef";
	int i;

	for (i = 7; i >= 0; i--)
		g12_putc(hx[(v >> (i * 4)) & 15u]);
}

static void g12_dump_reg(const char *tag, u32 v)
{
	g12_puts(tag);
	g12_putc('=');
	g12_hex8(v);
	g12_putc('\n');
}

/*
 * BROM GAHBCFG / FIFO map at residual entry (before any residual write).
 * Hypothesis 1 (closed): BROM used buffer-DMA and clearing DMA_EN for id1 left
 * bulk silent.  The measured cause was the BROM RX FIFO map (GRXFSIZ=0x100),
 * not DMA-vs-slave, so this is now snapshot-and-dump only — nothing restores
 * GAHBCFG from it.
 */
static u32 snap_gahbcfg;
static u32 snap_gusbcfg;
static u32 snap_grxfsiz;
static u32 snap_gnptxfsiz;
static u32 snap_dieptxf1;
static u32 snap_ghwcfg2;
static int snap_taken;

static void g12_snapshot_entry(void)
{
	if (snap_taken)
		return;
	snap_gahbcfg = R(GAHBCFG);
	snap_gusbcfg = R(GUSBCFG);
	snap_grxfsiz = R(GRXFSIZ);
	snap_gnptxfsiz = R(GNPTXFSIZ);
	snap_dieptxf1 = R(DIEPTXF1);
	snap_ghwcfg2 = R(GHWCFG2);
	snap_taken = 1;
}

static void g12_dump_entry_snap(void)
{
	g12_puts("\n#entry\n");
	g12_dump_reg("GAHBCFG", snap_gahbcfg);
	g12_dump_reg("GUSBCFG", snap_gusbcfg);
	g12_dump_reg("GRXFSIZ", snap_grxfsiz);
	g12_dump_reg("GNPTXFSIZ", snap_gnptxfsiz);
	g12_dump_reg("DIEPTXF1", snap_dieptxf1);
	g12_dump_reg("GHWCFG2", snap_ghwcfg2);
	g12_puts("#eend\n");
}

/*
 * After first identify completes (post-OE / I): dump core + EP0 state before
 * waiting on id2 / WR_LARGE.  One line per reg — host captures via fleet log.
 */
static void g12_dump_post_id(void)
{
	g12_puts("\n#regs\n");
	g12_dump_reg("DSTS", R(DSTS));
	g12_dump_reg("DCTL", R(DCTL));
	g12_dump_reg("GINTSTS", R(GINTSTS));
	g12_dump_reg("GINTMSK", R(GINTMSK));
	g12_dump_reg("GAHBCFG", R(GAHBCFG));
	g12_dump_reg("GUSBCFG", R(GUSBCFG));
	g12_dump_reg("DOEPCTL0", R(DOEPCTL0));
	g12_dump_reg("DOEPTSIZ0", R(DOEPTSIZ0));
	g12_dump_reg("DOEPINT0", R(DOEPINT0));
	g12_dump_reg("DOEPDMA0", R(DOEPDMA0));
	g12_dump_reg("DIEPCTL0", R(DIEPCTL0));
	g12_dump_reg("DIEPINT0", R(DIEPINT0));
	g12_dump_reg("DIEPDMA0", R(DIEPDMA0));
	/* GRXSTSR is a non-destructive peek; safe in slave mode. */
	g12_dump_reg("GRXSTSR", R(GRXSTSR));
	g12_dump_reg("DAINT", R(DAINT));
	g12_dump_reg("DAINTMSK", R(DAINTMSK));
	g12_dump_reg("GNPTXSTS", R(GNPTXSTS));
	g12_dump_reg("GRXFSIZ", R(GRXFSIZ));
	g12_dump_reg("GOTGCTL", R(GOTGCTL));
	g12_puts("#end\n");
}

/*
 * Crude ms delay.  TPL has no timer driver yet.  ~1e6 nops ≈ 1 ms class on
 * A55 @ ~1–1.8 GHz (volatile prevents elision).  Residual only needs
 * order-of-magnitude accuracy for disconnect windows + poll budget.
 */
static void crude_mdelay(unsigned int ms)
{
	while (ms--) {
		unsigned int i;

		for (i = 0; i < 1000000; i++)
			asm volatile("nop");
	}
}

#if defined(CONFIG_MESON_G12A_RESIDENT) && defined(CONFIG_TPL_BUILD)
/*
 * Resident chain: once the TPL has armed the USB fallback it owns the EE
 * watchdog, and this file pets it instead of stopping or re-timing it
 * (g12-resident.c).  The eMMC fetch feeds window 1 through
 * meson_g12_residual_feed_begin(), so this is on the eMMC path too.
 */
bool g12a_resident_wdt_keep(void);
#define G12_WDT_KEEP()	g12a_resident_wdt_keep()
#else
#define G12_WDT_KEEP()	false
#endif

/* BROM/EE WDT ~6–8 s kills residual before host re-enums — stop it. */
static void g12_wdt_stop(void)
{
	void __iomem *w = (void __iomem *)G12_WDT_BASE;
	u32 c;

	if (G12_WDT_KEEP())
		return;
	c = readl(w + G12_WDT_CTRL);
	writel(c & ~G12_WDT_EN, w + G12_WDT_CTRL);
	writel(0, w + G12_WDT_RSET);
}

static void g12_wdt_pet(void)
{
	writel(0, (void __iomem *)(G12_WDT_BASE + G12_WDT_RSET));
}

/*
 * Stretch TCNT and re-enable only if something re-armed EN.  Prefer stop;
 * pet keeps a live timer from firing mid-poll if stop was ignored (secure).
 */
static void g12_wdt_hold_off(void)
{
	void __iomem *w = (void __iomem *)G12_WDT_BASE;
	u32 c;

	if (G12_WDT_KEEP())
		return;
	c = readl(w + G12_WDT_CTRL);

	/* 1 ms tick if DIV already programmed; else program DIV + stop EN */
	if (!(c & G12_WDT_CLKDIV_EN)) {
		c = G12_WDT_DIV_1MS | G12_WDT_CLK_EN | G12_WDT_CLKDIV_EN;
		writel(c, w + G12_WDT_CTRL);
	}
	writel(0xffff, w + G12_WDT_TCNT); /* max ~65 s @ 1 ms */
	writel(0, w + G12_WDT_RSET);
	/* Clear EN — residual window must not reboot to BROM */
	writel(readl(w + G12_WDT_CTRL) & ~G12_WDT_EN, w + G12_WDT_CTRL);
}

/*
 * 0 while window 1 runs (before dram_init), 1 from window 2 on.  The two DRAM
 * destinations (DEST_SPL, DEST_FIT) are unbacked until training has finished,
 * and a store to one of them in window 1 hangs the CPU inside handle_rxfifo
 * (an early-queued frame 2: console ends R1 / + .. / H, no F1).
 */
static int bulk_dram_ready;
/* 1 = EP2 deliberately not re-armed (window 1, after its first frame). */
static int bulk_hold;

static int dest_is_dram(u32 addr)
{
	return addr == DEST_SPL || addr == DEST_FIT;
}

static int dest_allowed(u32 addr)
{
	if (!bulk_dram_ready && dest_is_dram(addr))
		return 0;
	return addr == DEST_SPL || addr == DEST_FIT ||
	       addr == DEST_HOLD0 || addr == DEST_HOLD1 ||
	       addr == DEST_AML_DDR || addr == DEST_PIEI ||
	       addr == DEST_HOLD_1D || addr == DEST_HOLD_1D_ALT;
}

/* ---------- protocol state ---------- */

enum ep0_state {
	EP0_IDLE,
	EP0_DATA_OUT,
	EP0_STATUS_IN,	/* IN data queued; DIEPINT → arm status OUT */
	EP0_STATUS_OUT,	/* status OUT ZLP armed; await XFERCOMPL/OUTDONE */
};

static enum ep0_state ep0_state;
static u8 setup_buf[8];
static u8 ep0_data_buf[64];
static int ep0_data_len;
static u8 pending_addr;

static u32 bulk_dst_addr;
static u32 bulk_total_len;
static u32 bulk_recv_len;
static u32 usb_spl_len;
static int usb_done;
static int identify_hits;
static int saw_enum;
static int crumb_r_sent;
/* Prevent SETUPDONE + DOEPINT.SETUP double-dispatch of one SETUP packet. */
static u8 setup_dispatch_gen;
static u8 setup_handled_gen;

/*
 * Post-id bulk-only mode (multi-SETUP wall bypass).  RX always lands in
 * bulk_stage first — never write dest during fifo_read (a bad dest must not
 * hang before the 'r'/'R' crumb).
 */
static int bulk_mode;
static int bulk_saw_outrx;
static u8 bulk_stage[512] __aligned(64);
static u8 bulk_hdr[16];
static u32 bulk_hdr_have;
static u32 bulk_pay_dest;
static u32 bulk_pay_len;
static u32 bulk_pay_have;
static u32 bulk_arm_bytes;
/* Multi-frame G12R: completed allowlisted payloads this residual window. */
static u32 bulk_frames_done;
/* 1 = frame finished; wait OUTDONE then ep2_bulk_frame_reinit. */
static int bulk_need_frame_reinit;
static u32 bulk_bytes_total;
/* Deferred R/+ crumbs — UART mid-stream stalls host bulk (FIFO overflow). */
static u32 bulk_pending_r;
static u32 bulk_last_dest;
static u32 bulk_last_word;
static int bulk_last_is_ahb;
/* 1 = aml CPU-bounced HOLD0→HOLD_1D so 1d parks at HOLD0. */
static int bulk_aml_bounced_to_1d;
/* 1 = aml already CPU-promoted HOLD_1D→DEST_AML_DDR (4-way concat). */
static int bulk_aml_promoted;
/* 1 = this window carried a 2d section (4-way concat). */
static int bulk_2d_seen;

enum bulk_phase {
	BULK_PH_HDR = 0,
	BULK_PH_PAY = 1,
};
static enum bulk_phase bulk_ph;
/*
 * 1 = the host reset the bus during bulk: the frame in flight is dropped and
 * EP0 is served until the next frame completes, so the host can re-enumerate
 * and resend it.  The bulk paths otherwise never look at EP0 or USBRST.
 */
static int bulk_reenum;

static void fifo_read(u32 fifo_off, u8 *buf, int len)
{
	while (len >= 4) {
		u32 w = R(fifo_off);

		buf[0] = w;
		buf[1] = w >> 8;
		buf[2] = w >> 16;
		buf[3] = w >> 24;
		buf += 4;
		len -= 4;
	}
	if (len > 0) {
		u32 w = R(fifo_off);

		if (len > 2)
			buf[2] = w >> 16;
		if (len > 1)
			buf[1] = w >> 8;
		buf[0] = w;
	}
}

static void fifo_write(u32 fifo_off, const u8 *buf, int len)
{
	while (len >= 4) {
		u32 w = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);

		W(fifo_off, w);
		buf += 4;
		len -= 4;
	}
	if (len > 0) {
		u32 w = buf[0];

		if (len > 1)
			w |= buf[1] << 8;
		if (len > 2)
			w |= buf[2] << 16;
		W(fifo_off, w);
	}
}

static void g12_clear_goutnak(void)
{
	int i;

	SET(DCTL, DCTL_CGOUTNAK);
	for (i = 0; i < 10000; i++) {
		if (!(R(GINTSTS) & INT_GOUTNAKEFF))
			break;
	}
	if (R(GINTSTS) & INT_GOUTNAKEFF)
		W(GINTSTS, INT_GOUTNAKEFF);
}

static void ep0_arm_setup(void)
{
	g12_clear_goutnak();
	/* SUPCNT=3, PKTCNT=1, xfersize=24 (GXL / Linux dwc2) */
	W(DOEPTSIZ0, SUPCNT3 | PKTCNT1 | 24);
	/*
	 * Absolute program (not RMW): EP0 is control, MPS encoding 0 = 64 B.
	 * RMW of NAKSTS(bit27) as SNAK was a known trap; absolute write also
	 * drops any stale STALL/EPDIS/EPTYPE residue from BROM.
	 */
	W(DOEPCTL0, USBACTEP | EPENA | CNAK);
	ep0_state = EP0_IDLE;
}

static void ep0_arm_data(int len)
{
	u32 c;

	W(DOEPTSIZ0, PKTCNT1 | (len & 0x7ffff));
	c = R(DOEPCTL0);
	c &= ~((u32)STALL | (u32)SNAK);
	c |= USBACTEP | EPENA | CNAK;
	W(DOEPCTL0, c);
}

static void ep0_zlp(void)
{
	u32 c;
	int i;

	/* Wait NP TX slot (GXL: bare SET without wait left host ETIMEDOUT). */
	for (i = 0; i < 100000; i++) {
		u32 nptx = R(GNPTXSTS);

		if ((nptx & GNPTXSTS_NPTXQ_MASK) && (nptx & 0xffff) >= 1)
			break;
	}
	W(DIEPTSIZ0, PKTCNT1 | 0);
	c = R(DIEPCTL0);
	c &= ~((u32)STALL | (u32)SNAK);
	c |= USBACTEP | EPENA | CNAK;
	if ((c & 0x3) == 0)
		c |= 0; /* EP0 MPS encoded 0 = 64 B */
	W(DIEPCTL0, c);
	ep0_state = EP0_STATUS_IN;
}

/*
 * EP0 IN data (identify / GET_DESCRIPTOR / READ_MEM).
 * Match GXL slave-PIO order: wait GNPTX → DIEPTSIZ → EPENA|CNAK → push DFIFO.
 * No prior EPENA-stop / NPTX flush (measured: not required for id1).
 */
static void ep0_tx(const u8 *data, int len)
{
	u32 need = (u32)((len + 3) / 4) + 1;
	u32 diep;
	int i;

	for (i = 0; i < 100000; i++) {
		u32 nptx = R(GNPTXSTS);

		if ((nptx & GNPTXSTS_NPTXQ_MASK) && (nptx & 0xffff) >= need)
			break;
	}

	W(DIEPTSIZ0, PKTCNT1 | (len & 0x7ffff));

	diep = R(DIEPCTL0);
	diep &= ~((u32)STALL | (u32)SNAK | (u32)EPDIS);
	diep |= USBACTEP | EPENA | CNAK;
	if ((diep & 0x3) == 0)
		diep |= 0; /* EP0 MPS 64 encoded 0 */
	W(DIEPCTL0, diep);

	fifo_write(EP0_FIFO, data, len);
	/* Wait DIEPINT then arm status OUT (do not arm here — races IN). */
	ep0_state = EP0_STATUS_IN;
}

/*
 * Kill BROM's leftover EP2 transfer.  DWC2 EPENA is set-only from SW; writing
 * a DOEPCTL without EPENA does NOT clear it.  Measured 2026-08-09: host
 * bulk OUT ACKs (usbmon C Bo 0 80) while our DOEPTSIZ2 stays full and no
 * RXFLVL/XFERCOMPL — bulk still completed into BROM's old DOEPDMA target.
 * EP0 EPDIS wedges this core (ban); EP2 EPDIS is the required reclaim.
 */
static void handle_rxfifo(void);

static void ep2_bulk_out_kill(void)
{
	int i;

	W(DOEPINT2, 0xffffffffu);
	/* SNAK first so the core stops accepting while we disable. */
	W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK | 512);
	if (R(DOEPCTL2) & EPENA) {
		W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK | EPDIS | 512);
		for (i = 0; i < 100000; i++) {
			if (R(DOEPINT2) & DIEPINT_EPDISBLD)
				break;
			if (!(R(DOEPCTL2) & EPENA))
				break;
		}
		W(DOEPINT2, 0xffffffffu);
	}
	W(DOEPDMA2, 0);
	W(DOEPTSIZ2, 0);
}

static void ep2_bulk_out_arm(void)
{
	u32 chunk;

	if (bulk_hold) {
		/*
		 * Window 1 after its first frame: leave EP2 disabled and NAKing.
		 * Anything the host queues now (frame 2) waits on the wire until
		 * window 2 clears bulk_hold and arms EP2 (serve2 entry).
		 */
		W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK | 512);
		return;
	}
	if (bulk_mode) {
		/* Continuous: one HS bulk packet (≤512).  Host short-packet OK. */
		chunk = DMA_BULK_BYTES;
	} else {
		u32 left = bulk_total_len - bulk_recv_len;

		chunk = left > DMA_BULK_BYTES ? DMA_BULK_BYTES : left;
		if (!chunk)
			return;
	}
	bulk_arm_bytes = chunk;
	W(DOEPTSIZ2, (1u << 19) | chunk);

	/* Slave PIO */
	W(DOEPDMA2, 0);
	W(DOEPINT2, 0xffffffffu);
	W(DOEPCTL2, USBACTEP | EPTYPE_BULK | EPENA | CNAK | 512);
}

static void bulk_copy_to(u32 dest, u32 off, const u8 *src, int n)
{
	u8 *d = (u8 *)(uintptr_t)(dest + off);
	int i;

	for (i = 0; i < n; i++)
		d[i] = src[i];
}

/*
 * Payload write.  HOLD0 + len > AML_DDR_SPLIT_SIZE → split:
 *   aml → HOLD0 (or DEST_AML_DDR when 2d present so HOLD0 free for 2d)
 *   piei → DEST_PIEI — NOT HOLD1 (hangs)
 *   1d → HOLD0 after aml bounce to HOLD_1D (USB HOLD_1D hangs)
 *   2d → DEST_HOLD0 (1d stays HOLD_1D when has_2d)
 * HOLD1 @ 0xFFFE0000 bus-hangs on write (measured 2026-08-09).
 */
static void bulk_copy_pay(u32 off, const u8 *src, int n)
{
	u32 dest = bulk_pay_dest;
	u32 aml = AML_DDR_SPLIT_SIZE;
	u32 piei_end = aml + PIEI_SPLIT_SIZE;
	u32 d1_end = piei_end + HOLD_1D_MAX;
	u32 d2_end = d1_end + HOLD_2D_MAX;
	int has_2d;

	if (!dest)
		return;	/* refused DRAM frame: bytes are consumed and dropped */

	if (dest == DEST_SPL && bulk_pay_len > SPL_FIT_SPLIT) {
		/* DRAM stage then boot FIT, in one frame (see SPL_FIT_SPLIT). */
		while (n > 0) {
			int chunk;

			if (off < SPL_FIT_SPLIT) {
				chunk = (int)(SPL_FIT_SPLIT - off);
				if (chunk > n)
					chunk = n;
				bulk_copy_to(DEST_SPL, off, src, chunk);
			} else {
				chunk = n;
				bulk_copy_to(DEST_FIT, off - SPL_FIT_SPLIT,
					     src, chunk);
			}
			off += chunk;
			src += chunk;
			n -= chunk;
		}
		return;
	}

	if (dest != DEST_HOLD0 || bulk_pay_len <= aml) {
		bulk_copy_to(dest, off, src, n);
		return;
	}

	has_2d = (bulk_pay_len > d1_end);

	while (n > 0) {
		u32 d, d_off, region_end;
		int chunk;

		if (off < aml) {
			d = DEST_HOLD0;
			d_off = off;
			region_end = aml;
		} else if (off < piei_end) {
			d = DEST_PIEI;
			d_off = off - aml;
			region_end = piei_end;
		} else if (off < d1_end) {
			/*
			 * Solitude: USB writes into HOLD_1D hang ~8 KiB in
			 * (concat 107320 dies at host off=66048). Bounce aml
			 * HOLD0→HOLD_1D once, park 1d at HOLD0. Train skips
			 * AML0 at HOLD_1D and uses HOLD0 1d (h0); residual end
			 * copies aml bounce → DEST_AML_DDR.
			 *
			 * With a 2d section the same trick simply runs twice
			 * (see the d2 arm below) — every USB write still lands
			 * on HOLD0/DEST_PIEI, the only two destinations this
			 * board has ever accepted from the wire.
			 */
			if (!bulk_aml_bounced_to_1d) {
				u32 i;
				volatile u32 *s =
					(volatile u32 *)(uintptr_t)DEST_HOLD0;
				volatile u32 *d32 =
					(volatile u32 *)(uintptr_t)DEST_HOLD_1D;

				for (i = 0; i < (AML_DDR_SPLIT_SIZE / 4); i++)
					d32[i] = s[i];
				bulk_aml_bounced_to_1d = 1;
				g12_putc('b');
			}
			d = DEST_HOLD0;
			d_off = off - piei_end;
			region_end = d1_end;
		} else if (has_2d && off < d2_end) {
			/*
			 * 2d arm.  HOLD_1D holds the aml bounce and HOLD0 the
			 * complete 1d payload, so free HOLD0 with two CPU
			 * copies before the first 2d byte:
			 *   aml  HOLD_1D  -> DEST_AML_DDR (the promote the
			 *                    3-way path defers to window end)
			 *   1d   HOLD0    -> DEST_HOLD_1D (the vendor staging
			 *                    window g12a_stage_1d_vendor would
			 *                    otherwise fill at train time)
			 * 2d then parks at HOLD0 and the train stage copies it
			 * over DEST_HOLD_1D between the 1D and 2D passes.
			 */
			if (!bulk_aml_promoted) {
				u32 i;
				volatile u32 *s =
					(volatile u32 *)(uintptr_t)DEST_HOLD_1D;
				volatile u32 *d32 =
					(volatile u32 *)(uintptr_t)DEST_AML_DDR;

				g12_putc('q');
				for (i = 0; i < (AML_DDR_SPLIT_SIZE / 4); i++)
					d32[i] = s[i];
				s = (volatile u32 *)(uintptr_t)DEST_HOLD0;
				d32 = (volatile u32 *)(uintptr_t)DEST_HOLD_1D;
				for (i = 0; i < (HOLD_1D_MAX / 4); i++)
					d32[i] = s[i];
				bulk_aml_promoted = 1;
				bulk_2d_seen = 1;
				g12_putc('Q');
			}
			d = DEST_HOLD0;
			d_off = off - d1_end;
			region_end = d2_end;
		} else {
			return;
		}
		if (region_end > bulk_pay_len)
			region_end = bulk_pay_len;
		chunk = (int)(region_end - off);
		if (chunk <= 0)
			return;
		if (chunk > n)
			chunk = n;
		bulk_copy_to(d, d_off, src, chunk);
		off += chunk;
		src += chunk;
		n -= chunk;
	}
}

/* Flush deferred R/+ crumbs when host is idle (safe for UART). */
static void bulk_flush_r_crumbs(void)
{
	u32 n = bulk_pending_r;
	static const char hx[] = "0123456789abcdef";

	if (!n)
		return;
	/* One R line per flush; nibble = frame count (cap 15). */
	g12_puts("\nR");
	g12_putc(hx[n > 15 ? 15 : n]);
	g12_puts("\n+");
	if (bulk_last_is_ahb) {
		g12_putc(' ');
		g12_hex8(bulk_last_word);
	} else {
		g12_putc('D');
	}
	g12_putc('\n');
	bulk_pending_r = 0;
}

/*
 * After a completed G12R frame: dump EP2 + force clean re-init before the
 * next frame.  Dual64 usbmon both C 0 but device R1-only: host wire OK,
 * SW missed frame2 OUTRX — suspect DOEPTSIZ rewrite while EPENA still set
 * (DWC2 DOEPTSIZn only writable when EPENA clear).  EP2 EPDIS reclaim is
 * OK (unlike EP0).  Host sequential --gap 100–500 ms covers dump+reinit.
 */
static void ep2_bulk_frame_reinit(void)
{
	/* OUTDONE-only.  Soft re-arm ONLY — never EPDIS between frames. */
	g12_putc('f');
	W(DOEPINT2, 0xffffffffu);
	if (!(R(DOEPCTL2) & EPENA))
		ep2_bulk_out_arm();
	else if (bulk_hold)
		/* Held: an armed EP2 would take one more packet; disable it. */
		W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK | EPDIS | 512);
	else
		/* EPENA still set: re-assert CNAK, leave live. */
		W(DOEPCTL2, USBACTEP | EPTYPE_BULK | EPENA | CNAK | 512);
	g12_putc((R(DOEPCTL2) & EPENA) ? 'A' : 'a');
	bulk_need_frame_reinit = 0;
	/* Host gap: flush deferred R crumb for the completed frame. */
	bulk_flush_r_crumbs();
}





static void bulk_finish_payload(void)
{
	u32 finished_len = bulk_pay_len;
	u32 finished_dest = bulk_pay_dest;

	if (!finished_dest) {
		/*
		 * A DRAM-destination frame refused in window 1 (see the 'N'
		 * crumb in bulk_handle_pkt): all its bytes were consumed and
		 * dropped.  Not a received frame -- do not count it.
		 */
		bulk_ph = BULK_PH_HDR;
		bulk_hdr_have = 0;
		bulk_pay_have = 0;
		bulk_pay_len = 0;
		return;
	}
	if (finished_dest == DEST_SPL)
		usb_spl_len = finished_len;

	/*
	 * NO mid-stream R/+ UART — 115200 blocked RX drain (aml OK, rest
	 * timeout).  Between frames: dump + EPDIS re-init (host --gap).
	 */
	bulk_ph = BULK_PH_HDR;
	bulk_hdr_have = 0;
	bulk_pay_have = 0;
	bulk_pay_len = 0;
	bulk_total_len = 0;
	bulk_recv_len = 0;
	crumb_r_sent = 1;
	bulk_reenum = 0;	/* a frame arrived: the host is back on bulk */
	if (bulk_frames_done < 15) {
		bulk_frames_done++;
		bulk_bytes_total += finished_len;
		bulk_pending_r++;
	}
	bulk_last_dest = finished_dest;
	bulk_last_is_ahb = ((finished_dest & 0xff000000u) == 0xff000000u);
	if (bulk_last_is_ahb)
		bulk_last_word = readl((void __iomem *)(uintptr_t)finished_dest);

#ifdef CONFIG_TPL_MESON_G12A_USB_W1_HOLD
	/*
	 * First frame of window 1 is in: stop taking EP2 data until window 2.
	 * Whatever the host queues next (frame 2) NAKs on the wire instead of
	 * being parsed here, before DRAM exists.  Single-frame recipes only
	 * (a host that sends aml/piei/1d/2d as separate frames must not set
	 * this symbol).
	 */
	if (!bulk_dram_ready)
		bulk_hold = 1;
#endif
	/* Prefer OUTDONE reinit; soft re-arm now if EPENA already clear. */
	bulk_need_frame_reinit = 1;
	g12_putc('F'); /* frame SW complete; reinit pending */
	if (!(R(DOEPCTL2) & EPENA)) {
		ep2_bulk_out_arm();
		g12_putc('A');
		bulk_need_frame_reinit = 0;
		bulk_flush_r_crumbs();
	}
}

/*
 * Assemble G12R stream (or raw-64 HOLD0 probe) from one OUTRX packet.
 * pkt is already drained from EP2 DFIFO into bulk_stage.
 */
static void bulk_handle_pkt(const u8 *pkt, int bcnt)
{
	int off = 0;

	if (bcnt <= 0)
		return;
	/* Ignore bulk until id1 complete + mid-session reclaim armed EP2. */
	if (!bulk_mode || !identify_hits)
		return;

	if (!bulk_saw_outrx) {
		g12_putc('r');
		bulk_saw_outrx = 1;
	}

	while (off < bcnt) {
		if (bulk_ph == BULK_PH_HDR) {
			int n = 16 - (int)bulk_hdr_have;
			int i;

			if (n > bcnt - off)
				n = bcnt - off;
			for (i = 0; i < n; i++)
				bulk_hdr[bulk_hdr_have + i] = pkt[off + i];
			bulk_hdr_have += n;
			off += n;
			if (bulk_hdr_have < 16)
				continue;

			{
				u32 magic = bulk_hdr[0] | ((u32)bulk_hdr[1] << 8) |
					    ((u32)bulk_hdr[2] << 16) |
					    ((u32)bulk_hdr[3] << 24);
				u32 dest = bulk_hdr[4] | ((u32)bulk_hdr[5] << 8) |
					   ((u32)bulk_hdr[6] << 16) |
					   ((u32)bulk_hdr[7] << 24);
				u32 len = bulk_hdr[8] | ((u32)bulk_hdr[9] << 8) |
					  ((u32)bulk_hdr[10] << 16) |
					  ((u32)bulk_hdr[11] << 24);

				if (magic == G12R_MAGIC) {
					if (!bulk_dram_ready && dest_is_dram(dest) &&
					    len != 0 && len <= 0x400000u) {
						/*
						 * DRAM destination before dram_init: refuse.
						 * Swallow the payload (bulk_pay_dest 0 =
						 * discard) so the stream stays in sync and
						 * the host's write completes instead of
						 * hanging the CPU on an unbacked store.
						 * One crumb: 'N' (no DRAM yet).
						 */
						g12_putc('N');
						bulk_pay_dest = 0;
						bulk_pay_len = len;
						bulk_pay_have = 0;
						bulk_ph = BULK_PH_PAY;
						bulk_hdr_have = 0;
						continue;
					}
					if (!dest_allowed(dest) || len == 0 ||
					    len > 0x400000u) {
						g12_putc('!');
						bulk_hdr_have = 0;
						continue;
					}
					/* HOLD0 concat: aml||piei[||1d[||2d]]; after-aml ≤ piei+1d+2d. */
					if (dest == DEST_HOLD0 && len > AML_DDR_SPLIT_SIZE &&
					    (len - AML_DDR_SPLIT_SIZE) > CONCAT_AFTER_AML_MAX) {
						g12_putc('!');
						bulk_hdr_have = 0;
						continue;
					}
					bulk_pay_dest = dest;
					bulk_pay_len = len;
					bulk_pay_have = 0;
					bulk_ph = BULK_PH_PAY;
					/* One-char only — no multi-char mid-stream. */
					g12_putc('H');
					/* no mid-stream 'S' — 2nd UART char stalls multi-packet RX */
				} else if (bulk_frames_done == 0) {
					/*
					 * Raw 64 B HOLD0 probe — only before any
					 * G12R.  After first G12R, non-magic
					 * headers are desync (do NOT treat as
					 * raw — that flooded R and stomped holds).
					 */
					bulk_pay_dest = DEST_HOLD0;
					bulk_pay_len = 64;
					bulk_pay_have = 0;
					bulk_ph = BULK_PH_PAY;
					bulk_copy_to(DEST_HOLD0, 0, bulk_hdr, 16);
					bulk_pay_have = 16;
					if (bulk_pay_have >= bulk_pay_len) {
						bulk_finish_payload();
						continue;
					}
				} else {
					/* Desync after G12R: drop 1 B and resync. */
					g12_putc('?');
					bulk_hdr_have = 0;
					/* shift stream: keep looking (off already advanced) */
					continue;
				}
			}
		} else {
			/* BULK_PH_PAY */
			int n = (int)(bulk_pay_len - bulk_pay_have);

			if (n > bcnt - off)
				n = bcnt - off;
			if (n > 0) {
				bulk_copy_pay(bulk_pay_have, pkt + off, n);
				bulk_pay_have += n;
				off += n;
			}
			if (bulk_pay_have >= bulk_pay_len)
				bulk_finish_payload();
		}
	}
}

static void g12_clk_enable(void)
{
	u32 v = readl((void __iomem *)(G12_HIU_BASE + HHI_GCLK_MPEG1));

	v |= HHI_MPEG1_USB0 | HHI_MPEG1_USB1 | HHI_MPEG1_USB;
	writel(v, (void __iomem *)(G12_HIU_BASE + HHI_GCLK_MPEG1));
	/* Do NOT pulse RESET_USB — GXL measured soft-connect death after reset. */
}

static void g12_glue_device_mode(void)
{
	void __iomem *glue = (void __iomem *)G12_USB_GLUE_BASE;
	u32 r0, r1, r4, r5, u2p;

	/* OTG PHY port → peripheral (HOST_DEVICE=0); linux OTG phy init */
	u2p = readl(glue + U2P1_R0);
	u2p &= ~U2P_R0_HOST_DEVICE;
	u2p |= U2P_R0_ID_PULLUP | U2P_R0_DRV_VBUS;
	writel(u2p, glue + U2P1_R0);

	/* FLADJ 0x20 — dwc3_meson_g12a_usb_init_glue */
	r1 = readl(glue + USB_R1);
	r1 &= ~USB_R1_U3H_FLADJ_30MHZ_MASK;
	r1 |= (0x20u << 19);
	writel(r1, glue + USB_R1);

	/* Peripheral: U2D_ACT + P21 sleep (linux usb_otg_apply_mode DEVICE) */
	r0 = readl(glue + USB_R0);
	r0 |= USB_R0_U2D_ACT;
	r0 &= ~USB_R0_U2D_SS_SCALEDOWN_MASK;
	writel(r0, glue + USB_R0);

	r4 = readl(glue + USB_R4);
	r4 |= USB_R4_P21_SLEEP_M0;
	/* clear P21_PORT_RESET so the device port is out of reset */
	r4 &= ~USB_R4_P21_PORT_RESET_0;
	writel(r4, glue + USB_R4);

	r5 = readl(glue + USB_R5);
	r5 |= USB_R5_ID_DIG_EN_0 | USB_R5_ID_DIG_EN_1;
	r5 &= ~USB_R5_ID_DIG_TH_MASK;
	r5 |= (0xffu << 8);
	writel(r5, glue + USB_R5);
}

/*
 * PHY re-attach stack (PHY21 + U2P POR + PHY_CTRL + glue dump) parked:
 * SFTDISCON re-enum is BLOCKED on phys4 (tip a8b6adb0f4d).  Mid-session
 * residual never calls these.  Restore from git history if re-attach resumes.
 */

/* Parked: mid-session CSFTRST killed host bulk (USBError 5).  Keep for RE. */
static void __maybe_unused dwc2_core_soft_reset(void)
{
	int t;

	SET(GRSTCTL, CSFTRST);
	for (t = 0; t < 10000; t++) {
		if (!(R(GRSTCTL) & CSFTRST))
			break;
	}
	for (t = 0; t < 10000; t++) {
		if (R(GRSTCTL) & AHBIDLE)
			break;
	}
}

static void dwc2_flush_fifos(void)
{
	int t;

	W(GRSTCTL, TXFFLSH | TXFNUM_ALL);
	for (t = 0; t < 10000; t++) {
		if (!(R(GRSTCTL) & TXFFLSH))
			break;
	}
	W(GRSTCTL, RXFFLSH);
	for (t = 0; t < 10000; t++) {
		if (!(R(GRSTCTL) & RXFFLSH))
			break;
	}
}

static void dwc2_handle_reset(void)
{
	W(DCFG, DEVSPD_HS);
	pending_addr = 0;
	ep0_arm_setup();
	W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK | 512);
}

/*
 * A bus reset while a bulk frame is in flight (a host port reset, a hub
 * glitch): the host drops the transfer, re-enumerates, and sends the frame
 * again from its header.  Forget the partial frame so the parser expects a
 * header, and stay on EP0 until the next frame lands (bulk_reenum).  A
 * non-continuous WR_LARGE ends the same way.  Before this, both bulk paths
 * ignored USBRST and SETUP, so the host's GET_DESCRIPTOR after the reset
 * went unanswered and the board sat in the TPL, deaf, until power-off.
 *
 * Only the parser is reset: frame 1's concat routing flags
 * (bulk_aml_*, bulk_2d_seen) are not unwound, so an interrupted frame 1
 * is not covered.
 */
static void bulk_abort_frame(void)
{
	bulk_ph = BULK_PH_HDR;
	bulk_hdr_have = 0;
	bulk_pay_have = 0;
	bulk_pay_len = 0;
	bulk_need_frame_reinit = 0;
	bulk_total_len = 0;
	bulk_recv_len = 0;
	bulk_reenum = 1;
}

static int handle_ep0_data_done(void);

static void handle_setup(void)
{
	u8 bmRequestType = setup_buf[0];
	u8 bRequest = setup_buf[1];
	u16 wValue = (u16)(setup_buf[2] | (setup_buf[3] << 8));
	u16 wLength = (u16)(setup_buf[6] | (setup_buf[7] << 8));

	if (setup_handled_gen == setup_dispatch_gen)
		return; /* already handled this SETUP packet */
	setup_handled_gen = setup_dispatch_gen;

	/* Per-SETUP crumb: proves EP0 SETUP still lands after first identify. */
	g12_putc('S');

	if ((bmRequestType & 0x60) == 0x00) {
		switch (bRequest) {
		case 0x05: /* SET_ADDRESS */
			/*
			 * DCFG.DevAddr is written on the SETUP, as Linux dwc2
			 * and the databook do: the core holds the new address
			 * back until the status stage completes.  Deferring
			 * it to an OUT completion never applies it -- a
			 * no-data SET_ADDRESS has an IN status stage -- so a
			 * fresh attach stayed at address 0 and the host's next
			 * request got no handshake (xHCI -71).  The GXL twin
			 * in meson-usb.c was measured; the BootROM's session
			 * arrives already addressed, so only a re-enumeration
			 * or a host bus reset reaches this.
			 */
			{
				u32 dcfg = R(DCFG);

				dcfg &= ~(0x7f << 4);
				dcfg |= (u32)(wValue & 0x7f) << 4;
				W(DCFG, dcfg);
			}
			pending_addr = 0;
			ep0_zlp();
			ep0_arm_setup();
			break;
		case 0x06: /* GET_DESCRIPTOR */
			switch (wValue >> 8) {
			case 0x01: {
				int len = 18;

				if (wLength < len)
					len = wLength;
				ep0_tx(dev_desc, len);
				break;
			}
			case 0x02: {
				int len = CFG_LEN;

				if (wLength < len)
					len = wLength;
				ep0_tx(cfg_desc, len);
				break;
			}
#if USB_REENUM
			case 0x03: {	/* STRING: langid, and the chip-id serial */
				const u8 *sd = NULL;
				int len = 0;

				if ((wValue & 0xff) == 0) {
					sd = str_desc_langid;
					len = sizeof(str_desc_langid);
				} else if ((wValue & 0xff) == STR_IDX_SERIAL) {
					len = str_serial_build();
					sd = str_desc_serial;
				}
				if (!len) {
					SET(DIEPCTL0, STALL);
					ep0_arm_setup();
					break;
				}
				if (wLength < len)
					len = wLength;
				ep0_tx(sd, len);
				break;
			}
#endif
			default:
				SET(DIEPCTL0, STALL);
				ep0_arm_setup();
				break;
			}
			break;
		case 0x09: /* SET_CONFIGURATION */
			/*
			 * Configuring resets the host's bulk data toggle to DATA0
			 * (USB 2.0 9.1.1.5); reset ours with it.  A continued
			 * BootROM session never sends this request; after a
			 * re-enumeration EP2 still held the BootROM's toggle, and
			 * the core ACKed and dropped every DATA0 packet as a
			 * retransmission (host bulk write OK, nothing received).
			 */
			/*
			 * Mid-bulk re-enumeration (bulk_reenum): EP2 may still be
			 * enabled from before the reset, or armed by window 2's loop
			 * since, and SNAK on an enabled EP2 NAKs the resent frame
			 * forever.  Disable it first, then re-arm it for the frame.
			 */
			if (bulk_reenum)
				ep2_bulk_out_kill();
			W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK | SETD0PID | 512);
			ep0_zlp();
			ep0_arm_setup();
			if (bulk_reenum && bulk_mode)
				ep2_bulk_out_arm();
			break;
		default:
			SET(DIEPCTL0, STALL);
			ep0_arm_setup();
			break;
		}
	} else if ((bmRequestType & 0x60) == 0x40) {
		switch (bRequest) {
		case REQ_IDENTIFY_HOST:
			identify_hits++;
			g12_putc('i'); /* every identify, not just first */
			ep0_tx(identify_rsp, sizeof(identify_rsp));
			break;
		case REQ_WR_LARGE_MEM:
			/* Crumb W: SETUP seen (before DATA-OUT completes → R). */
			g12_puts("\nW\n");
			ep0_state = EP0_DATA_OUT;
			ep0_data_len = 0;
			ep0_arm_data(16);
			break;
		case REQ_READ_MEM: {
			u16 wIndex = (u16)(setup_buf[4] | (setup_buf[5] << 8));
			u32 addr = ((u32)wValue << 16) | (u32)wIndex;
			int len = (int)wLength;
			u8 buf[64];
			int i;

			if (len > 64)
				len = 64;
			if (len <= 0) {
				ep0_zlp();
				break;
			}
			for (i = 0; i < len; i++)
				buf[i] = readb((void __iomem *)(uintptr_t)(addr + i));
			ep0_tx(buf, len);
			break;
		}
		case REQ_RUN_IN_ADDR:
			ep0_state = EP0_DATA_OUT;
			ep0_arm_data(4);
			break;
		default:
			SET(DIEPCTL0, STALL);
			ep0_arm_setup();
			break;
		}
	} else {
		SET(DIEPCTL0, STALL);
		ep0_arm_setup();
	}
}

static int handle_ep0_data_done(void)
{
	u8 bRequest = setup_buf[1];

	switch (bRequest) {
	case REQ_WR_LARGE_MEM:
		bulk_dst_addr = ep0_data_buf[0] | ((u32)ep0_data_buf[1] << 8) |
			((u32)ep0_data_buf[2] << 16) | ((u32)ep0_data_buf[3] << 24);
		bulk_total_len = ep0_data_buf[4] | ((u32)ep0_data_buf[5] << 8) |
			((u32)ep0_data_buf[6] << 16) | ((u32)ep0_data_buf[7] << 24);
		bulk_recv_len = 0;
		if (!dest_allowed(bulk_dst_addr) || bulk_total_len == 0 ||
		    bulk_total_len > 0x400000) {
			bulk_total_len = 0;
			SET(DIEPCTL0, STALL);
			ep0_arm_setup();
			return 0;
		}
		if (!crumb_r_sent) {
			g12_puts("\nR\n");
			crumb_r_sent = 1;
		}
		ep0_zlp();
		ep0_arm_setup();
		ep2_bulk_out_arm();
		break;
	case REQ_RUN_IN_ADDR:
		ep0_zlp();
		ep0_arm_setup();
		usb_done = 1;
		break;
	default:
		ep0_arm_setup();
		break;
	}
	return 0;
}

static void handle_rxfifo(void)
{
	u32 st = R(GRXSTSP);
	int ep = EPNUM(st);
	int bcnt = BCNT(st);
	int pktsts = PKTSTS(st);

	if (pktsts == PKTSTS_SETUPRX && ep == 0 && bcnt > 0) {
		fifo_read(EP0_FIFO, setup_buf, bcnt > 8 ? 8 : bcnt);
		setup_dispatch_gen++;
		g12_putc('s'); /* raw SETUPRX (before dispatch) */
	} else if (pktsts == PKTSTS_OUTRX && ep == 0 && bcnt > 0) {
		int n = bcnt > (int)sizeof(ep0_data_buf) ? (int)sizeof(ep0_data_buf) : bcnt;

		fifo_read(EP0_FIFO, ep0_data_buf, n);
		ep0_data_len = n;
	} else if (pktsts == PKTSTS_OUTRX && ep == 2 && bcnt > 0) {
		int n = bcnt > 512 ? 512 : bcnt;

		/* Always drain DFIFO into stage first (never dest-during-read). */
		fifo_read(EP2_FIFO, bulk_stage, n);
		if (bulk_mode) {
			bulk_handle_pkt(bulk_stage, n);
			/*
			 * Mid-frame: re-arm if EPENA dropped.  Frame boundary reinit
			 * is OUTDONE-only (kill/reinit from OUTRX hung on Solitude).
			 */
			if (!bulk_need_frame_reinit && !(R(DOEPCTL2) & EPENA))
				ep2_bulk_out_arm();
		} else if (bulk_total_len > 0 && bulk_recv_len < bulk_total_len) {
			int i;
			u8 *dst = (u8 *)(uintptr_t)(bulk_dst_addr + bulk_recv_len);

			for (i = 0; i < n && bulk_recv_len < bulk_total_len; i++) {
				dst[i] = bulk_stage[i];
				bulk_recv_len++;
			}
			if (!crumb_r_sent) {
				g12_puts("\nR\n");
				crumb_r_sent = 1;
			}
			if (bulk_recv_len >= bulk_total_len) {
				g12_putc('+');
				if (bulk_dst_addr == DEST_SPL)
					usb_spl_len = bulk_recv_len;
			}
		}
		/* else: unexpected bulk — stage discarded */
	} else if (pktsts == PKTSTS_OUTRX || pktsts == PKTSTS_SETUPRX) {
		int w = (bcnt + 3) >> 2;

		while (w-- > 0)
			(void)R(EP0_FIFO + (ep << 12));
	} else if (pktsts == PKTSTS_SETUPDONE && ep == 0) {
		/*
		 * Only IDLE.  Do NOT dispatch from STATUS_IN here: DOEPINT.SETUP
		 * and SETUPDONE both fire for one SETUP; after identify ep0_tx
		 * sets STATUS_IN, a second dispatch re-entered handle_setup and
		 * double-fired ep0_tx → NP TX wedged (UART SiSi, host id2/WR
		 * timeout; Solitude 2026-08-09).  GXL: IDLE-only.
		 */
		if (ep0_state == EP0_IDLE)
			handle_setup();
	} else if (pktsts == PKTSTS_OUTDONE && ep == 0) {
		if (ep0_state == EP0_DATA_OUT) {
			handle_ep0_data_done();
		} else if (ep0_state == EP0_STATUS_OUT ||
			   ep0_state == EP0_STATUS_IN) {
			/*
			 * Host OUT ZLP status for our IN data (GXL path).
			 * DOEPINT.XFERCOMPL may also fire — ep0_arm_setup is
			 * idempotent.
			 */
			if (pending_addr) {
				u32 dcfg = R(DCFG);

				dcfg &= ~(0x7f << 4);
				dcfg |= (u32)pending_addr << 4;
				W(DCFG, dcfg);
				pending_addr = 0;
			}
			g12_putc('O');
			ep0_arm_setup();
			g12_putc((R(DOEPCTL0) & EPENA) ? 'E' : 'e');
			/* post-rearm nibbles: DOEPCTL[31:28], [27:24], DOEPTSIZ[31:28] */
			g12_putc("0123456789abcdef"[(R(DOEPCTL0) >> 28) & 15u]);
			g12_putc("0123456789abcdef"[(R(DOEPCTL0) >> 24) & 15u]);
			g12_putc("0123456789abcdef"[(R(DOEPTSIZ0) >> 28) & 15u]);
		}
	} else if (pktsts == PKTSTS_OUTDONE && ep == 2) {
		if (bulk_need_frame_reinit) {
			ep2_bulk_frame_reinit();
		} else if ((bulk_mode ||
			    (bulk_total_len > 0 &&
			     bulk_recv_len < bulk_total_len)) &&
			   !(R(DOEPCTL2) & EPENA)) {
			ep2_bulk_out_arm();
		}
	}
}

/*
 * Worldcup takeover (GXL phy_por=0 class) — default residual path.
 *
 * Measured 2026-08-09: after pass-1 RUN the host still sees 1b8e:c003 and
 * BROM identify 02 04 while TPL residual is live (U printed, no ENUMDONE).
 * Full PHY-POR + CSFTRST + soft-connect did NOT produce a new enum / 02 23.
 * Vendor/GXL residual keeps the same bus address and only swaps the identify
 * handler — do that first: leave BROM GUSBCFG/FIFOs alone, re-arm EP0 + masks.
 */
static void g12_device_start_worldcup(void)
{
	ep0_state = EP0_IDLE;
	usb_done = 0;
	bulk_total_len = 0;
	bulk_recv_len = 0;
	usb_spl_len = 0;
	identify_hits = 0;
	saw_enum = 0;
	crumb_r_sent = 0;
	pending_addr = 0;
	setup_dispatch_gen = 0;
	setup_handled_gen = 0;
	bulk_mode = 0;
	bulk_saw_outrx = 0;
	bulk_hdr_have = 0;
	bulk_pay_have = 0;
	bulk_pay_len = 0;
	bulk_ph = BULK_PH_HDR;

	g12_wdt_hold_off();
	g12_clk_enable();
	g12_glue_device_mode();

	/*
	 * Snapshot BROM GAHBCFG/FIFO/GHWCFG2 BEFORE any residual change.
	 * Then worldcup: leave BROM FIFO map; clear DMA_EN only (keep
	 * HBSTLEN — full GAHBCFG=BIT(0) zeroed burst limited EP0 control).
	 */
	g12_snapshot_entry();
	g12_dump_entry_snap();
	{
		u32 ahb = R(GAHBCFG);

		ahb &= ~GAHBCFG_DMA_EN;
		ahb |= BIT(0);  /* GBLINTRMSK */
		W(GAHBCFG, ahb);
	}
	W(DOEPMSK, R(DOEPMSK) | DOEPINT_SETUP | DOEPINT_XFERCOMPL |
	  DOEPINT_STUPPKTRCVD);
	W(DIEPMSK, R(DIEPMSK) | DIEPINT_XFERCOMPL);
	/* OUT EP0, OUT EP2 (bulk), IN EP0 — not IN EP2 (bit18 was wrong). */
	W(DAINTMSK, R(DAINTMSK) | (1u << 0) | (1u << 2) | (1u << 16));
	W(GINTMSK, R(GINTMSK) | INT_USBRST | INT_ENUMDONE | INT_RXFLVL |
	  INT_OEPINT | INT_IEPINT);
	W(GUSBCFG, R(GUSBCFG) | FORCEDEVMODE | USBTRDTIM_5);
	W(GOTGCTL, R(GOTGCTL) | GOTGCTL_VBVALOEN | GOTGCTL_VBVALOVAL |
	  GOTGCTL_BVALOEN | GOTGCTL_BVALOVAL);
#if USB_REENUM
	/*
	 * Re-enumerate as our own device: a 10 ms host-visible disconnect
	 * before the soft-connect below, keeping the BootROM's configuration.
	 * Needs SET_ADDRESS applied on the SETUP and the bulk toggle reset on
	 * SET_CONFIGURATION (both in this file).  The host follows the device
	 * by its port.
	 */
	SET(DCTL, SFTDISCON);
	crude_mdelay(10);
#endif
	{
		u32 dctl = R(DCTL);

		dctl &= ~SFTDISCON;
		W(DCTL, dctl);
		SET(DCTL, DCTL_CGOUTNAK);
	}
	/* Do NOT drain RXFIFO here (GXL: that discarded the host SETUP). */
	ep0_arm_setup();
	/* Reclaim EP2 from BROM (EPDIS); leave SNAK until post-id bulk arm. */
	ep2_bulk_out_kill();
	W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK | 512);
}

/*
 * Mid-session FIFO+EP reclaim after worldcup id1 (NO SFTDISCON, NO CSFTRST).
 *
 * Measured iter1 (CSFTRST mid-session): UART MsC + GRXFSIZ=512 green, but host
 * bulk USBError(5) — core soft-reset kills the host pipe even with DCTL
 * soft-connected.  Park CSFTRST; hyp1 only: rewrite FIFO map + re-arm EP2.
 *
 * Crumbs: M (start) F (FIFO rewritten, no CSFTRST) C (EP2 armed).  #C dump.
 */
static void g12_mid_session_core_reclaim(void)
{
	u32 dctl;
	int i;

	/* Soft-connected for the whole reclaim — never write SFTDISCON. */
	dctl = R(DCTL);
	dctl &= ~SFTDISCON;
	W(DCTL, dctl);

	g12_putc('M');

	/* Slave PIO; keep link.  Do not rewrite GUSBCFG / DCFG / CSFTRST. */
	{
		u32 ahb = R(GAHBCFG);

		ahb &= ~GAHBCFG_DMA_EN;
		ahb |= BIT(0);
		W(GAHBCFG, ahb);
	}

	/*
	 * Drain stale RX before rewriting FIFO map (GXL mid-session habit).
	 * SGOUTNAK briefly so host cannot deliver mid-rewrite, then clear.
	 */
	SET(DCTL, DCTL_SGOUTNAK);
	for (i = 0; i < 256 && (R(GINTSTS) & INT_RXFLVL); i++)
		(void)R(GRXSTSP);

	/*
	 * Full FIFO map for 512 B HS bulk (GXL rung-3 class).
	 * BROM left GRXFSIZ=0x100 (256 words).  Grow RX to 512 words;
	 * NPTX 128 @ 512; DIEPTXF1 64 @ 640.  Total 704 words.
	 */
	W(GRXFSIZ, 512);
	W(GNPTXFSIZ, FIFOSIZE(512, 128));
	W(DIEPTXF1, FIFOSIZE(640, 64));
	dwc2_flush_fifos();
	g12_putc('F'); /* FIFO-only (no CSFTRST) */

	SET(DCTL, DCTL_CGOUTNAK);
	for (i = 0; i < 10000; i++) {
		if (!(R(GINTSTS) & INT_GOUTNAKEFF))
			break;
	}
	if (R(GINTSTS) & INT_GOUTNAKEFF)
		W(GINTSTS, INT_GOUTNAKEFF);

	W(DOEPMSK, R(DOEPMSK) | DOEPINT_SETUP | DOEPINT_XFERCOMPL |
	  DOEPINT_STUPPKTRCVD);
	W(DIEPMSK, R(DIEPMSK) | DIEPINT_XFERCOMPL);
	W(DAINTMSK, R(DAINTMSK) | (1u << 0) | (1u << 2) | (1u << 16));
	W(GINTMSK, R(GINTMSK) | INT_USBRST | INT_ENUMDONE | INT_RXFLVL |
	  INT_OEPINT | INT_IEPINT);

	ep0_state = EP0_IDLE;
	ep0_arm_setup();

	/* Continuous bulk-only (multi-SETUP wall still blocks WR_LARGE). */
	bulk_mode = 1;
	bulk_ph = BULK_PH_HDR;
	bulk_hdr_have = 0;
	bulk_pay_have = 0;
	bulk_pay_len = 0;
	bulk_saw_outrx = 0;
	bulk_total_len = 0;
	bulk_recv_len = 0;
	bulk_arm_bytes = 0;
	bulk_frames_done = 0;
	bulk_need_frame_reinit = 0;
		bulk_bytes_total = 0;
	bulk_pending_r = 0;
	bulk_last_dest = 0;
	bulk_last_word = 0;
	bulk_last_is_ahb = 0;
	bulk_aml_bounced_to_1d = 0;
	bulk_aml_promoted = 0;
	bulk_2d_seen = 0;
	crumb_r_sent = 0;

	ep2_bulk_out_kill();
	ep2_bulk_out_arm();

	g12_putc('C');
	g12_puts("\n#C\n");
	g12_dump_reg("DCFG", R(DCFG));
	g12_dump_reg("DCTL", R(DCTL));
	g12_dump_reg("GAHBCFG", R(GAHBCFG));
	g12_dump_reg("GRXFSIZ", R(GRXFSIZ));
	g12_dump_reg("GNPTXFSIZ", R(GNPTXFSIZ));
	g12_dump_reg("DIEPTXF1", R(DIEPTXF1));
	g12_dump_reg("DOEPCTL0", R(DOEPCTL0));
	g12_dump_reg("DOEPCTL2", R(DOEPCTL2));
	g12_dump_reg("DOEPTSIZ2", R(DOEPTSIZ2));
	g12_dump_reg("GINTSTS", R(GINTSTS));
	g12_puts("#Cend\n");
}

static void g12_poll_ep0(u32 intsts);

static void g12_poll_once(void)
{
	u32 intsts = R(GINTSTS);
	int n;

	/*
	 * Bulk-hot (WR_LARGE stream) OR continuous bulk-only mode after id1:
	 * drain RXFIFO hard and keep EP2 EPENA.  Multi-SETUP is dead; skip
	 * EP0 work while bulk_mode to maximize OUT catch rate -- except on a
	 * bus reset and the re-enumeration after it (bulk_abort_frame()).
	 */
	if (!(intsts & INT_USBRST) && !bulk_reenum &&
	    (bulk_mode ||
	     (bulk_total_len > 0 && bulk_recv_len < bulk_total_len))) {
		for (n = 0; n < 2048 && (R(GINTSTS) & INT_RXFLVL); n++)
			handle_rxfifo();
		{
			u32 di2 = R(DOEPINT2);

			if (di2 & DOEPINT_XFERCOMPL) {
				g12_putc('X');
				W(DOEPINT2, di2);
				for (n = 0; n < 64 && (R(GINTSTS) & INT_RXFLVL);
				     n++)
					handle_rxfifo();
			} else if (di2) {
				W(DOEPINT2, di2);
			}
		}
		if (!(R(DOEPCTL2) & EPENA))
			ep2_bulk_out_arm();
		return;
	}

	g12_poll_ep0(intsts);
}

/* EP0 and bus events; also window 2's way back after a bus reset. */
static void g12_poll_ep0(u32 intsts)
{
	int n;

	if (intsts & INT_USBRST) {
		W(GINTSTS, INT_USBRST);
		if (bulk_mode || bulk_total_len > 0) {
			g12_putc('u');
			bulk_abort_frame();
		}
		dwc2_handle_reset();
	}
	if (intsts & INT_ENUMDONE) {
		W(GINTSTS, INT_ENUMDONE);
		saw_enum = 1;
	}

	for (n = 0; n < 64 && (R(GINTSTS) & INT_RXFLVL); n++)
		handle_rxfifo();

	{
		u32 doep = R(DOEPINT0);

		if (doep & (DOEPINT_SETUP | DOEPINT_STUPPKTRCVD)) {
			W(DOEPINT0, doep);
			/*
			 * Dispatch only when this SETUPRX gen is unhandled.
			 * Same-SETUP double IRQ (SETUPDONE+DOEPINT) is latched
			 * out.  A *new* SETUP (gen advanced) aborts prior xfer.
			 */
			if (setup_handled_gen != setup_dispatch_gen) {
				if (ep0_state != EP0_IDLE)
					g12_putc('D');
				ep0_state = EP0_IDLE;
				handle_setup();
			}
		} else if (doep & DOEPINT_XFERCOMPL) {
			W(DOEPINT0, DOEPINT_XFERCOMPL);
			if (ep0_state == EP0_DATA_OUT) {
				/*
				 * Worldcup residual (BROM FIFOs kept) often
				 * signals EP0 DATA-OUT complete via DOEPINT
				 * XFERCOMPL without a GRXSTSP OUTDONE pop.
				 */
				handle_ep0_data_done();
			} else if (ep0_state == EP0_STATUS_OUT ||
				   ep0_state == EP0_STATUS_IN) {
				u32 d, c, n;

				/*
				 * Status OUT complete (identify IN done).
				 * Match GXL: only ep0_arm_setup — no RXFFLSH /
				 * NPTX flush / mask rewrite.  Those left EPENA
				 * set but the core NAKed every further SETUP
				 * (usbmon S, no 2nd SETUPRX; tip 0fb38cf5d79).
				 */
				g12_putc('o');
				if (pending_addr) {
					u32 dcfg = R(DCFG);

					dcfg &= ~(0x7f << 4);
					dcfg |= (u32)pending_addr << 4;
					W(DCFG, dcfg);
					pending_addr = 0;
				}
				ep0_arm_setup();
				g12_putc((R(DOEPCTL0) & EPENA) ? 'E' : 'e');
				/* diag nibble: DSTS[2:0] | (NAKSTS<<3) */
				d = R(DSTS);
				c = R(DOEPCTL0);
				n = (d & 7u) | ((c & SNAK) ? 8u : 0u);
				g12_putc("0123456789abcdef"[n & 15u]);
			}
		} else if (doep) {
			W(DOEPINT0, doep);
		}
	}
	{
		u32 diep = R(DIEPINT0);

		if (diep & DIEPINT_XFERCOMPL) {
			u32 c;

			W(DIEPINT0, DIEPINT_XFERCOMPL);
			if (ep0_state == EP0_STATUS_IN) {
				/*
				 * IN data delivered — arm status OUT as ZLP
				 * only once (xfersize=0).  Re-entering while
				 * already STATUS_OUT rewrote DOEPTSIZ mid-
				 * handshake (UART sSixx, no oE; Solitude).
				 */
				g12_putc('x');
				W(DOEPTSIZ0, PKTCNT1 | 0);
				c = R(DOEPCTL0);
				c &= ~((u32)STALL | (u32)SNAK | (u32)EPDIS);
				c |= USBACTEP | EPENA | CNAK;
				W(DOEPCTL0, c);
				ep0_state = EP0_STATUS_OUT;
			}
		} else if (diep) {
			W(DIEPINT0, diep);
		}
	}

	/*
	 * Missed status-OUT IRQ recovery: EPENA dropped while waiting for
	 * status → treat as complete and re-arm SETUP (host already ACKed).
	 */
	if (ep0_state == EP0_STATUS_OUT && !(R(DOEPCTL0) & EPENA)) {
		g12_putc('q');
		if (pending_addr) {
			u32 dcfg = R(DCFG);

			dcfg &= ~(0x7f << 4);
			dcfg |= (u32)pending_addr << 4;
			W(DCFG, dcfg);
			pending_addr = 0;
		}
		ep0_arm_setup();
		g12_putc((R(DOEPCTL0) & EPENA) ? 'E' : 'e');
	}
}

/* Post-id EP0 ZY refresh parked — mid-session CSFTRST re-arms EP0 in reclaim. */

/**
 * meson_g12_usb_device_stop() - tear DWC2 before TPL→SPL jump.
 *
 * Never leave a live soft-connected link across the stage boundary (GXL rule).
 */
void meson_g12_usb_device_stop(void)
{
	W(GINTMSK, 0);
	W(DOEPMSK, 0);
	W(DIEPMSK, 0);
	W(DAINTMSK, 0);
	SET(DCTL, SFTDISCON);
	crude_mdelay(10);
}

u32 meson_g12_usb_spl_payload_len(void)
{
	return usb_spl_len;
}

/**
 * meson_g12_usb_has_2d() - did this window deliver an lpddr4_2d section?
 *
 * The train stage cannot tell 1d from 2d by inspecting HOLD0 — both are
 * headerless 0xC000 payloads — so the splitter reports it instead of the
 * consumer guessing.
 */
int meson_g12_usb_has_2d(void)
{
	return bulk_2d_seen;
}

/**
 * meson_g12_usb_residual_serve2() - SECOND residual window, after dram_init.
 * @timeout_ms: poll budget, ~1 ms per spin while idle.
 *
 * The first window runs BEFORE dram_init because the DDR blobs have to be in
 * SRAM before training.  The DRAM stage and the boot FIT cannot ride it: they
 * live at 0x08000000 and 0x02000000, which are DRAM and are not backed by
 * anything until training has finished.  That — not a packaging omission — is
 * why every boot so far ended at NOSPL.
 *
 * This window reuses the SAME live DWC2 session: no device_start, no core
 * reclaim, no SFTDISCON (all three are measured to kill the host link on
 * Solitude).  EP2 was left armed and soft-connected when window 1 returned,
 * so the only thing needed is to re-arm and drain again.
 *
 * Crumb W2 on entry, w2 on exit with the frame count.
 */
int meson_g12_usb_residual_serve2(unsigned int timeout_ms)
{
	unsigned int spins;
	unsigned int idle_ms = 0;
	unsigned int last_frames = 0;
	unsigned int stall = 0;
	u32 last_have = 0;
#ifdef CONFIG_TPL_MESON_G12A_USB_W1_HOLD
	int was_held = bulk_hold;
#endif

	/* dram_init has run: DRAM destinations are valid, release the EP2 hold. */
	bulk_dram_ready = 1;
	bulk_hold = 0;
	bulk_mode = 1;
	bulk_reenum = 0;
	bulk_ph = BULK_PH_HDR;
	bulk_hdr_have = 0;
	bulk_pay_have = 0;
	bulk_pay_len = 0;
	bulk_frames_done = 0;
	bulk_need_frame_reinit = 0;
	bulk_pending_r = 0;
	bulk_saw_outrx = 0;
	usb_spl_len = 0;
	crumb_r_sent = 0;

	g12_puts("\nW2\n");
#ifdef CONFIG_TPL_MESON_G12A_USB_W1_HOLD
	/*
	 * The held branch of ep2_bulk_frame_reinit() requests EPDIS but does
	 * not wait for it.  If EPENA is still set here, EP2 sits in SNAK with
	 * nothing left to write CNAK: the arm below and the loop's re-arm both
	 * key on EPENA clear, so frame 2 would NAK until the window times out.
	 * Reclaim it (bounded EPDIS wait) and arm it unconditionally.  Crumb K.
	 */
	if (was_held && (R(DOEPCTL2) & EPENA)) {
		g12_putc('K');
		ep2_bulk_out_kill();
		ep2_bulk_out_arm();
	}
#endif
	if (!(R(DOEPCTL2) & EPENA))
		ep2_bulk_out_arm();

	/*
	 * timeout_ms is an IDLE budget, not a spin count.  Mid-frame spins do
	 * not mdelay (that starves the EP2 re-arm), so counting them against
	 * the budget spends it in microseconds while data is streaming — which
	 * is exactly what truncated the 1577472 B payload on boot spl-2: the
	 * device left the window mid-frame (crumb w20, no R) and the host's
	 * bulk write then timed out.  Mid-frame progress is bounded by a
	 * separate stall counter instead.
	 */
	for (spins = 0;; spins++) {
		int n;

		if ((spins % 1024) == 0) {
			g12_wdt_stop();
			g12_wdt_pet();
		}
		/*
		 * This loop serves only the RX FIFO, so a host bus reset used to
		 * leave the board deaf here (no USBRST handling, no SETUP
		 * dispatch).  On a reset, and until the resent frame completes,
		 * run the EP0 path too; a fresh idle budget covers the host's
		 * re-enumeration.
		 */
		{
			u32 gi = R(GINTSTS);

			if (gi & INT_USBRST) {
				idle_ms = 0;
				stall = 0;
				last_have = 0;
			}
			if ((gi & INT_USBRST) || bulk_reenum)
				g12_poll_ep0(gi);
		}
		if (bulk_need_frame_reinit && !(R(DOEPCTL2) & EPENA))
			ep2_bulk_frame_reinit();
		else if (!(R(DOEPCTL2) & EPENA))
			ep2_bulk_out_arm();
		for (n = 0; n < 256 && (R(GINTSTS) & INT_RXFLVL); n++)
			handle_rxfifo();

		if (bulk_frames_done > last_frames) {
			last_frames = bulk_frames_done;
			idle_ms = 0;
			stall = 0;
			continue;
		}
		if (bulk_ph != BULK_PH_HDR || bulk_hdr_have) {
			/* Mid-frame: tight poll, never mdelay. */
			idle_ms = 0;
			if (bulk_pay_have != last_have) {
				last_have = bulk_pay_have;
				stall = 0;
			} else if (++stall > 20000000u) {
				g12_putc('!');
				break;
			}
			continue;
		}
		crude_mdelay(1);
		idle_ms++;
		if ((idle_ms % 500) == 0)
			g12_putc(':');
		if (idle_ms >= timeout_ms)
			break;
		if (bulk_frames_done > 0 && idle_ms >= 2000)
			break;
	}

	bulk_flush_r_crumbs();
	g12_puts("\nw2");
	{
		static const char hx[] = "0123456789abcdef";
		u32 f = bulk_frames_done;

		g12_putc(hx[f > 15 ? 15 : f]);
	}
	g12_putc('\n');
	return bulk_frames_done > 0 ? 0 : -1;
}

/**
 * meson_g12_usb_residual_serve() - mid-session residual (no SFTDISCON).
 * @timeout_ms: total poll budget (worldcup + bulk window)
 *
 * Flow:
 *   1. Worldcup takeover → id1 02 23 (crumb U / I)
 *   2. Mid-session FIFO reclaim: GRXFSIZ=512 + EP0/EP2 (crumb MFC)
 *      — DCTL stays soft-connected; no CSFTRST; no PHY POR; no SFTDISCON
 *   3. Host bulk-only multi-frame G12R (no 2nd SETUP): holds then SPL
 *   4. Window ends on timeout OR non-empty word at DEST_SPL after a frame
 *
 * Return: 0 if ≥1 allowlisted G12R frame completed OR SPL head present;
 *         -1 on timeout with nothing.  Tears USB (SFTDISCON) on exit.
 */
int meson_g12_usb_residual_serve(unsigned int timeout_ms)
{
	unsigned int spins;
	unsigned int max_spins = timeout_ms ? timeout_ms : 1;
	unsigned int idle_ms = 0;
	unsigned int last_frames = 0;
	int enum_crumb = 0;
	int id_crumb = 0;
	int mid_done = 0;

	/* Explicit zero — do not trust BSS alone for residual statics. */
	bulk_dram_ready = 0;
	bulk_hold = 0;
	bulk_mode = 0;
	bulk_reenum = 0;
	bulk_frames_done = 0;
	bulk_need_frame_reinit = 0;
	bulk_bytes_total = 0;
	bulk_pending_r = 0;
	bulk_saw_outrx = 0;
	bulk_ph = BULK_PH_HDR;
	bulk_hdr_have = 0;
	bulk_pay_have = 0;
	bulk_pay_len = 0;
	usb_spl_len = 0;
	usb_done = 0;
	identify_hits = 0;
	crumb_r_sent = 0;
	saw_enum = 0;

	/* Stage 1: worldcup — keep BROM link, answer 02 23 on EP0 */
	g12_device_start_worldcup();
	g12_puts("U\n");

	for (spins = 0; spins < max_spins; spins++) {
		int k;

		if ((spins % 100) == 0) {
			g12_wdt_stop();
			g12_wdt_pet();
		}

		/*
		 * Bulk-hot: more polls per spin to keep EP2 re-armed across
		 * multi-packet frames (aml_ddr ~94×512).  Pre-bulk: denser
		 * EP0 polling for id1.
		 */
		for (k = 0; k < (bulk_mode ? 80 : 200); k++)
			g12_poll_once();

		if (saw_enum && !enum_crumb) {
			g12_puts("E");
			enum_crumb = 1;
		}

		/*
		 * After first identify completes (EP0 IDLE post status):
		 * mid-session FIFO reclaim once.  NO SFTDISCON.
		 */
		if (identify_hits && !id_crumb && ep0_state == EP0_IDLE &&
		    !mid_done) {
			g12_puts("\nI\n");
			id_crumb = 1;
			g12_dump_post_id();
			/* Host id1 URB done; brief settle before reclaim. */
			crude_mdelay(50);
			g12_mid_session_core_reclaim();
			mid_done = 1;
			g12_puts("\nB\n"); /* bulk window open for host */
		}

		/*
		 * Bulk-only after C: heartbeat + DOEPTSIZ watch.  Success
		 * crumbs: r/H/R/+/y/f/X.  #t dump if still silent.
		 * Quiet the '.' spam once first OUTRX lands (multi-packet).
		 */
		if (mid_done && bulk_mode) {
			u32 ts, di2, gint, ctl2;

			if ((spins % 10) == 0) {
				ts = R(DOEPTSIZ2);
				di2 = R(DOEPINT2);
				gint = R(GINTSTS);
				ctl2 = R(DOEPCTL2);
				if (!bulk_saw_outrx)
					g12_putc('.');
				if (gint & INT_RXFLVL) {
					int n;

					if (!bulk_saw_outrx)
						g12_putc('f');
					for (n = 0; n < 256 &&
					     (R(GINTSTS) & INT_RXFLVL); n++)
						handle_rxfifo();
				}
				if (di2 & DOEPINT_XFERCOMPL) {
					if (!bulk_saw_outrx)
						g12_putc('X');
					W(DOEPINT2, di2);
				}
				if (bulk_arm_bytes &&
				    (ts & 0x7ffff) != bulk_arm_bytes) {
					if (!bulk_saw_outrx) {
						g12_putc('y');
						g12_dump_reg("TS", ts);
					}
				}
				if (!(ctl2 & EPENA)) {
					if (!bulk_saw_outrx)
						g12_putc('e');
					ep2_bulk_out_arm();
				}
			}
			if (!bulk_saw_outrx && (spins % 100) == 0) {
				g12_puts("\n#t\n");
				g12_dump_reg("DOEPTSIZ2", R(DOEPTSIZ2));
				g12_dump_reg("DOEPINT2", R(DOEPINT2));
				g12_dump_reg("DOEPCTL2", R(DOEPCTL2));
				g12_dump_reg("DOEPDMA2", R(DOEPDMA2));
				g12_dump_reg("GINTSTS", R(GINTSTS));
				g12_dump_reg("GRXSTSR", R(GRXSTSR));
				g12_dump_reg("GRXFSIZ", R(GRXFSIZ));
				g12_puts("#te\n");
			}
		}

		/*
		 * Multi-frame: stay in residual until full timeout_ms OR SPL
		 * head present.  Do NOT idle-exit after first R — that plus
		 * device_stop SFTDISCON killed piei.  Do NOT crude_mdelay in
		 * the bulk window (starves EP2 re-arm; usbmon: piei pkt1 ACK
		 * then pkt2 -2 timeout).  Host sequential gaps are fine while
		 * we tight-poll; R crumbs flush only at end / every new frame
		 * after a short spin quiet (no mdelay).
		 */
		if (usb_spl_len > 0 && bulk_ph == BULK_PH_HDR)
			break;
		if (usb_done && usb_spl_len > 0)
			break;
		if (bulk_frames_done > last_frames) {
			last_frames = bulk_frames_done;
			idle_ms = 0;
		} else if (bulk_frames_done > 0 && bulk_ph == BULK_PH_HDR &&
			   bulk_hdr_have == 0) {
			/*
			 * Wall-clock after first frame: without mdelay, remaining
			 * max_spins burn in <<1 s and residual exits (F1 +
			 * device_stop) before host piei.  1 ms/spin keeps the
			 * window open; drain RX every spin (not every 20 ms) so
			 * frame2 OUTRX is not left sticky in the FIFO.
			 */
			{
				int n;

				if (bulk_need_frame_reinit && !(R(DOEPCTL2) & EPENA))
					ep2_bulk_frame_reinit();
				else if (!(R(DOEPCTL2) & EPENA))
					ep2_bulk_out_arm();
				for (n = 0; n < 128 && (R(GINTSTS) & INT_RXFLVL); n++)
					handle_rxfifo();
			}
			crude_mdelay(1);
			idle_ms++;
			if ((idle_ms % 100) == 0)
				g12_putc('.');
			/* Early exit ~3 s quiet after last frame (host done). */
			if (idle_ms >= 3000)
				break;
		} else {
			idle_ms = 0;
			/* Mid-PAY: tight poll (no mdelay). */
		}
		/* Pre-id worldcup settle only (before bulk window). */
		if (!mid_done)
			crude_mdelay(1);
	}

	/*
	 * Second quiet only if first-loop idle was short (host may still be
	 * writing).  Main loop already waits ~3 s idle_ms after last frame —
	 * skip duplicate 3 s when idle_ms already >= 3000 (avoids hang in
	 * EP2 re-arm thrash before promote/train).
	 */
	if (bulk_frames_done > 0 && idle_ms < 3000) {
		unsigned int quiet = 0;
		unsigned int last = bulk_frames_done;

		while (quiet < 3000) {
			int n;

			if (!(R(DOEPCTL2) & EPENA))
				ep2_bulk_out_arm();
			for (n = 0; n < 128 && (R(GINTSTS) & INT_RXFLVL); n++)
				handle_rxfifo();
			if (bulk_frames_done > last) {
				last = bulk_frames_done;
				quiet = 0;
				continue;
			}
			crude_mdelay(1);
			quiet++;
			if ((quiet % 100) == 0)
				g12_putc('.');
		}
	}

	bulk_flush_r_crumbs();
	/*
	 * Concat 1d at HOLD0: aml bounce is at HOLD_1D. Promote aml to
	 * DEST before train. 1d stays at HOLD0 — train-time
	 * g12a_stage_1d_vendor() CPU-copies HOLD0 → 0xFFFC0000 (vendor
	 * staging). Crumb p/P around promote (measured hang risk).
	 */
	if (bulk_aml_bounced_to_1d && !bulk_aml_promoted) {
		u32 i;
		volatile u32 *s = (volatile u32 *)(uintptr_t)DEST_HOLD_1D;
		volatile u32 *d = (volatile u32 *)(uintptr_t)DEST_AML_DDR;

		g12_putc('p');
		for (i = 0; i < (AML_DDR_SPLIT_SIZE / 4); i++) {
			d[i] = s[i];
			if ((i & 0x3ff) == 0)
				g12_wdt_pet();
		}
		g12_putc('P');
		g12_putc('B');
	}
	/*
	 * Print F1 BEFORE any stop. SFTDISCON (device_stop) banned mid
	 * residual on Solitude — host re-enum dies; leave DWC2 soft-connected
	 * idle and hand off to train (train does not need USB).
	 */
	if (bulk_frames_done > 0) {
		g12_puts("\nF");
		{
			static const char hx[] = "0123456789abcdef";
			u32 n = bulk_frames_done;

			if (n > 15)
				n = 15;
			g12_putc(hx[n]);
		}
		g12_putc('\n');
	}
	g12_putc('z'); /* residual end (no SFTDISCON) */
	g12_putc('Z');

	if (bulk_frames_done > 0)
		return 0;

	g12_puts("u\n");
	return -1;
}

#if CONFIG_IS_ENABLED(MESON_G12A_EMMC_FETCH)
/*
 * Storage source for the two residual windows (TPL_MESON_G12A_EMMC_FETCH).
 * The bytes come from the eMMC (tpl-emmc-g12.c) instead of EP2, and are fed
 * into the same splitter, so every SRAM destination, the aml bounce/promote
 * and has_2d come out exactly as a USB delivery of the same concat leaves
 * them.  The on-media format is the wire format without the G12R header.
 */

/**
 * meson_g12_residual_feed_begin() - open window 1 for a storage source.
 * @total: concat length, the G12R header's len on the USB path.
 *
 * Does what the USB window does to the watchdog before its first byte:
 * g12_wdt_hold_off() (g12_device_start_worldcup()) then g12_wdt_stop()
 * (the serve loop).
 */
void meson_g12_residual_feed_begin(u32 total)
{
	g12_wdt_hold_off();
	g12_wdt_stop();
	bulk_dram_ready = 0;
	bulk_aml_bounced_to_1d = 0;
	bulk_aml_promoted = 0;
	bulk_2d_seen = 0;
	usb_spl_len = 0;
	bulk_pay_dest = DEST_HOLD0;
	bulk_pay_len = total;
}

/** meson_g12_residual_feed() - @n bytes of window 1 at concat offset @off. */
void meson_g12_residual_feed(u32 off, const u8 *src, int n)
{
	bulk_copy_pay(off, src, n);
}

/**
 * meson_g12_residual_feed_end() - close window 1: the end-of-window promote
 * meson_g12_usb_residual_serve() runs (aml bounce HOLD_1D -> DEST_AML_DDR
 * when no 2d section did it already).
 */
void meson_g12_residual_feed_end(void)
{
	if (bulk_aml_bounced_to_1d && !bulk_aml_promoted) {
		u32 i;
		volatile u32 *s = (volatile u32 *)(uintptr_t)DEST_HOLD_1D;
		volatile u32 *d = (volatile u32 *)(uintptr_t)DEST_AML_DDR;

		for (i = 0; i < (AML_DDR_SPLIT_SIZE / 4); i++)
			d[i] = s[i];
	}
	bulk_pay_dest = 0;
	bulk_pay_len = 0;
}

/**
 * meson_g12_residual_set_spl_len() - window 2 delivered @len bytes at
 * DEST_SPL (split at SPL_FIT_SPLIT); what bulk_finish_payload() records.
 */
void meson_g12_residual_set_spl_len(u32 len)
{
	bulk_dram_ready = 1;
	usb_spl_len = len;
}

/** meson_g12_residual_spl_fit_split() - the window-2 split this TPL uses. */
u32 meson_g12_residual_spl_fit_split(void)
{
	return SPL_FIT_SPLIT;
}

/* Window-2 destinations, for the reader. */
u32 meson_g12_residual_dest_spl(void)
{
	return DEST_SPL;
}

u32 meson_g12_residual_dest_fit(void)
{
	return DEST_FIT;
}
#endif
