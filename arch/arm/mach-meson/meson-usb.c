// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GXL SPL USB device mode -- minimal DWC2 slave-mode implementation.
 *
 * After DDR init, SPL re-initializes the DWC2 OTG controller (base
 * 0xc9100000) as a USB device, enumerates as the Amlogic ROM USB device
 * (VID=1b8e PID=c003), and receives a FIT image from the host via the
 * Amlogic USB protocol before returning for normal SPL boot.
 *
 * Protocol (from pyamlboot pyamlboot.py):
 *   REQ_IDENTIFY_HOST (0x20): IN, returns 8-byte version string
 *   REQ_WR_LARGE_MEM  (0x11): OUT, 16-byte header {addr,len,0,0}, then
 *                              bulk OUT on EP1 for the data
 *   REQ_RUN_IN_ADDR   (0x05): OUT, 4-byte address -- signals transfer done
 *
 * References:
 *   drivers/usb/common/dwc2_core.h -- register layout and bit definitions
 *   bl2_new.bin (0xd9000800): vendor BL2 DWC2 reinit + USB command server
 */

#include <chip_serial.h>
#include <asm/io.h>
#include <asm/arch/gx-handoff.h>
#include <linux/delay.h>

/* U1/U2 host-visible progress (readable after ROM-return). */
#define SPL_U1_MARKER_ADDR	0xd900c100UL
#define SPL_U1_MARKER_VAL	0x524f4d4fu	/* 'OMOR' */
/* One definition of the word, shared with the stage that sets the handoff bit */
#define SPL_U1_PROG_ADDR	GX_XPL_PROG_ADDR
#define U1P_START		0x0001
/* 0x0002 = GX_XPL_PROG_HANDOFF (gx-handoff.h), set by the TPL stage */
#define U1P_WORLDCUP		0x0010
#define U1P_PHYPOR		0x0020
#define U1P_USBRST		0x0100
#define U1P_ENUMDONE		0x0200
#define U1P_RXFLVL		0x0400
#define U1P_SETUPRX		0x0800
#define U1P_SETUPDONE		0x1000
#define U1P_IDENTIFY_TX		0x2000
#define U1P_IDENTIFY_OK		0x4000
/*
 * Bits 16..18 -- AMBIGUOUS BY CONSTRUCTION, do not read them as booleans.
 *
 * ep0_tx()'s slave-PIO path replaces the whole high half with a GNPTXSTS
 * snapshot whose low 12 bits are the NP TX FIFO free-dword count, so word bits
 * 16/17/18 are also free-dword count bits 0/1/2.  Both flags below are OR'd in
 * AFTER the last PIO ep0_tx() (bulk_dma_enter() switches ep0_tx to its DMA
 * early-return), so they are never erased -- but a SET bit cannot be told apart
 * from a stale snapshot bit.  Only a CLEAR bit carries information.
 *
 * Making these trustworthy means moving the snapshot out of this word; see the
 * "Progress word" note in gx-handoff.h.
 */
#define U1P_DMA_ON		0x10000	/* buffer-DMA bulk OUT armed */
#define U1P_DMA_STALL		0x40000	/* DMA watchdog forced the transfer done */

/*
 * U1P_STOP AND U1P_IN_XFERCOMPL ARE SEPARATE BITS NOW (audit S14).
 *
 * They used to be the same bit, 0x8000, with U1P_STOP the loser:
 * meson_usb_poll_once() sets U1P_IN_XFERCOMPL on every EP0 IN transfer that
 * completes -- many times during enumeration -- while U1P_STOP is set once, at
 * the very end of meson_usb_device_stop().  The bit therefore read 1 long
 * before teardown ran, so "did the stage tear down?" and "did any control-IN
 * ever complete?" printed the same string.  Two states, one output.
 *
 * The blocker on record was "the low half is full and the high half is
 * overwritten wholesale by ep0_tx()'s GNPTXSTS snapshot, so separating them
 * needs a progress-word redesign".  The first clause is true -- all sixteen
 * low bits are assigned, here and in gx-handoff.h.  The second is too coarse:
 * the high half is not uniformly dirty.  ep0_tx() writes
 *
 *     snap = (nptx & 0xffff) | (((nptx >> NPTXQ_SHIFT) & 0xf) << 12)
 *
 * into bits 31:16, so word bits 27:16 carry NPTxFSpcAvail[11:0] -- free DWORDS
 * in a partition of a DFIFO this part reports as 712 words total
 * (GHWCFG3.DfifoDepth, read 2026-08-06; the FIFO-ladder budget below puts NPTX
 * at 128, tighter still).  712 < 1024, so NPTxFSpcAvail can never set its bits
 * 10 or 11, and word bits 26 and 27 are ALWAYS ZERO after every ep0_tx().
 * ep0_tx() also masks them off explicitly (see the snapshot store), so this is
 * true by construction and not only by the datasheet bound.
 *
 * That is a stronger property than "free", and it is exactly what
 * U1P_DMA_ON/_STALL at bits 16/18 lack: those sit inside the SpcAvail range,
 * where only a CLEAR bit carries information.
 *
 * The temporal rule permits it too.  u1_prog_or(U1P_STOP) is the LAST
 * statement of meson_usb_device_stop(); no ep0_tx() and no whole-word
 * writel(U1P_START | U1P_WORLDCUP, ...) runs after it in that session, so a
 * bit OR'd there survives.
 *
 * WHY NOT DELETE THE STATE INSTEAD.  Blast radius is zero either way -- the
 * word is read over EP0 and never printed to the UART, so it appears in no
 * recorded log, and the one host decoder (tools/spl-u1-verify.py) had no STOP
 * entry at all.  But "did meson_usb_device_stop() run?" is directly diagnostic
 * for N145/N149, whose open question is where the stage stops after the pass-2
 * bulk.  Making that observable for the first time is a better trade than
 * removing the question.
 *
 * 🔴 Unrun on silicon.
 */
#define U1P_STOP		0x04000000	/* word bit 26: device_stop ran */
#define U1P_IN_XFERCOMPL	0x8000		/* EP0 IN XferCompl */

static void u1_prog_or(u32 bit)
{
	u32 v = readl((void __iomem *)SPL_U1_PROG_ADDR);

	writel(v | bit, (void __iomem *)SPL_U1_PROG_ADDR);
}

/* Busy-loop ms delay for use before timer/SMC init (USB-first path). */
static void crude_mdelay(unsigned int ms)
{
	/* ~1 GHz A53 rough busy-loop — safe before timer init */
	while (ms--) {
		unsigned int i;
		for (i = 0; i < 100000; i++)
			asm volatile("nop");
	}
}

static void crude_udelay(unsigned int us)
{
	while (us--) {
		unsigned int i;
		for (i = 0; i < 100; i++)
			asm volatile("nop");
	}
}

/* GXL DWC2 OTG controller MMIO base */
#define DWC2_BASE	0xc9100000UL

/* HHI gates + RESET1 for OTG (linux gxbb-clkc / Amlogic GXL) */
#define HHI_BASE		0xc883c000UL
#define HHI_GCLK_MPEG1		0x144
#define HHI_MPEG1_USB0		BIT(21)
#define HHI_MPEG1_USB1		BIT(22)
#define HHI_MPEG1_USB		BIT(26)
#define RESET1_REGISTER		0xc1104408UL
#define RESET1_USB_OTG		BIT(2)

/* GXL USB glue (dwc3-meson-gxl.c) — required for device soft-connect */
#define USB_GLUE_BASE		0xd0078080UL
#define USB_R0			0x00
#define USB_R0_U2D_ACT		BIT(31)
#define USB_R0_U2D_SS_SCALEDOWN_MASK GENMASK(30, 29)
#define USB_R4			0x10
#define USB_R4_P21_SLEEP_M0	BIT(1)
#define USB_R5			0x14
#define USB_R5_ID_DIG_EN_0	BIT(4)
#define USB_R5_ID_DIG_EN_1	BIT(5)

#ifndef GENMASK
#define GENMASK(h, l) (((~0UL) << (l)) & (~0UL >> (sizeof(long) * 8 - 1 - (h))))
#endif

static void meson_usb_glue_device_mode(void)
{
	u32 r0, r4, r5;

	/* Peripheral: U2D_ACT + P21 sleep (linux dwc3_meson_gxl_usb2_set_mode) */
	r0 = readl((void __iomem *)(USB_GLUE_BASE + USB_R0));
	r0 |= USB_R0_U2D_ACT;
	r0 &= ~USB_R0_U2D_SS_SCALEDOWN_MASK;
	writel(r0, (void __iomem *)(USB_GLUE_BASE + USB_R0));

	r4 = readl((void __iomem *)(USB_GLUE_BASE + USB_R4));
	r4 |= USB_R4_P21_SLEEP_M0;
	writel(r4, (void __iomem *)(USB_GLUE_BASE + USB_R4));

	r5 = readl((void __iomem *)(USB_GLUE_BASE + USB_R5));
	r5 |= USB_R5_ID_DIG_EN_0 | USB_R5_ID_DIG_EN_1;
	writel(r5, (void __iomem *)(USB_GLUE_BASE + USB_R5));
}

static void meson_usb_clk_enable(void)
{
	u32 v;

	/* ungated USB clocks (BROM leaves them on; after long idle re-open) */
	v = readl((void __iomem *)(HHI_BASE + HHI_GCLK_MPEG1));
	v |= HHI_MPEG1_USB0 | HHI_MPEG1_USB1 | HHI_MPEG1_USB;
	writel(v, (void __iomem *)(HHI_BASE + HHI_GCLK_MPEG1));
	/* Do NOT pulse RESET1_USB_OTG here — measured to leave soft-connect
	 * unattached and BROM reappearing ~20–40s later (PPPS 2026-08-04). */
}


/*
 * GXL USB2 OTG PHY (PHY1 = OTG PHY per USB2_OTG_PHY1 enum in
 * drivers/usb/dwc3/dwc3-meson-gxl.c).
 * APB base 0xd0000000 + 0x78020 = 0xd0078020.
 * drivers/phy/meson-gxl-usb2.c defines the register layout.
 */
#define USB2_PHY_BASE		0xd0078020UL
#define U2P_R0			0x00
#define  U2P_R0_DM_PULLDOWN	BIT(5)
#define  U2P_R0_DP_PULLDOWN	BIT(6)
#define  U2P_R0_ID_PULLUP	BIT(13)
#define  U2P_R0_POWER_ON_RESET	BIT(22)

/* Global register offsets */
#define GOTGCTL		0x000
#define GAHBCFG		0x008
#define GUSBCFG		0x00c
#define GRSTCTL		0x010
#define GINTSTS		0x014
#define GINTMSK		0x018
#define GRXSTSR		0x01c	/* peek RX status (no pop) */
#define GRXSTSP		0x020	/* pop RX status */
#define GRXFSIZ		0x024
#define GNPTXFSIZ	0x028
/*
 * Core capability registers. Read-only; never written.
 * GHWCFG2[4:3] ARCHITECTURE decides whether buffer-DMA is even possible
 * (Linux dwc2/hw.h GHWCFG2_ARCHITECTURE_MASK = 0x3 << 3; [2:0] is OP_MODE):
 *   0 = Slave-Only  -> GAHBCFG.DMA_EN is a no-op, PIO is the only path
 *   1 = External DMA
 *   2 = Internal DMA -> what DOEPDMA2 buffer-DMA needs
 * Dumped to scratch because three DMA attempts were made without ever
 * establishing the core supports it.
 */
#define GSNPSID		0x040
#define GHWCFG1		0x044
#define GHWCFG2		0x048
#define GHWCFG3		0x04c
#define GHWCFG4		0x050
#define DIEPTXF2	0x108
#define DIEPTXF1	0x104	/* dptxfsizn[0] */
/* GOTGCTL: force B-peripheral session (dwc2_udc force-b-session-valid) */
#define GOTGCTL_VBVALOEN	BIT(2)
#define GOTGCTL_VBVALOVAL	BIT(3)
#define GOTGCTL_BVALOEN		BIT(6)
#define GOTGCTL_BVALOVAL	BIT(7)
#define GOTGCTL_BSESVLD		BIT(19)
#define GOTGCTL_CONID_B		BIT(16)

/* Device register offsets */
#define DCFG		0x800
#define DCTL		0x804
#define DSTS		0x808
#define DIEPCTL0	0x900
#define DIEPINT0	0x908
#define DIEPTSIZ0	0x910
#define DOEPCTL0	0xb00
#define DOEPINT0	0xb08
#define DOEPTSIZ0	0xb10
#define DIEPCTL1	0x920
#define DIEPINT1	0x928
#define DIEPCTL2	0x940
#define DIEPTSIZ2	0x950
#define DIEPTSIZ1	0x930
#define DTXFSTS1	0x938
#define DOEPCTL1	0xb20
#define DOEPTSIZ1	0xb30
#define DOEPCTL2	0xb40
#define DOEPINT2	0xb48
#define DOEPTSIZ2	0xb50
#define DOEPDMA2	0xb54
#define DAINT		0x818
#define DAINTMSK	0x81c
#define DOEPMSK		0x814
#define DIEPMSK		0x810
/* DOEPINT bits */
#define DOEPINT_XFERCOMPL	BIT(0)
#define DOEPINT_EPDISBLD	BIT(1)
#define DOEPCTL_EPDIS		BIT(30)
#define DOEPINT_SETUP		BIT(3)
#define DOEPINT_STUPPKTRCVD	BIT(15)
#define GAHBCFG_DMA_EN		BIT(5)
/* DIEPINT bits */
#define DIEPINT_XFERCOMPL	BIT(0)
#define DIEPINT_TIMEOUT		BIT(3)

/* EP FIFO base: EP n at 0x1000 + n*0x1000 */
#define EP0_FIFO	0x1000
#define EP2_FIFO	0x3000
#define EP1_FIFO	0x2000

/* Register helpers */
#define R(off)		readl((void __iomem *)(DWC2_BASE + (off)))
#define W(off, v)	writel((v), (void __iomem *)(DWC2_BASE + (off)))
#define SET(off, m)	W((off), R(off) | (m))
#define CLR(off, m)	W((off), R(off) & ~(u32)(m))

/* GRSTCTL bits */
#define CSFTRST		BIT(0)
#define RXFFLSH		BIT(4)
#define TXFFLSH		BIT(5)
#define TXFNUM_ALL	(0x10 << 6)
#define AHBIDLE		BIT(31)

/* GUSBCFG bits */
#define FORCEDEVMODE	BIT(30)
/*
 * USBTRDTIM=5: correct value for GXL 8-bit UTMI PHY.
 * (bl2_new.bin:0x73f2 sets USBTRDTIM=5; 9 is for 16-bit UTMI only)
 */
#define USBTRDTIM_5	(5 << 10)

/* GINTSTS / GINTMSK bits */
#define INT_USBRST	BIT(12)
#define INT_ENUMDONE	BIT(13)
#define INT_RXFLVL	BIT(4)
#define INT_IEPINT	BIT(18)
#define INT_OEPINT	BIT(19)

/* GRXSTSP pktsts field: bits [20:17] */
#define PKTSTS(s)	(((s) >> 17) & 0xf)
#define EPNUM(s)	((s) & 0xf)
#define BCNT(s)		(((s) >> 4) & 0x7ff)

#define PKTSTS_GLOBALOUTNAK	1
#define PKTSTS_OUTRX		2
#define PKTSTS_OUTDONE		3
#define PKTSTS_SETUPDONE	4
#define PKTSTS_SETUPRX		6

/* DCFG bits */
#define DEVSPD_HS	0	/* High speed */

/* DCTL bits */
#define SFTDISCON	BIT(1)
#define DCTL_SGOUTNAK	BIT(9)	/* set global OUT NAK */
#define DCTL_CGOUTNAK	BIT(10)	/* clear global OUT NAK */

/* GINTSTS: global OUT NAK effective (quiesce handshake for DMA switch) */
#define INT_GOUTNAKEFF	BIT(7)

/* EP0 DMA buffer pointers — must be valid before GAHBCFG.DMA_EN goes on */
#define DOEPDMA0	0xb14
#define DIEPDMA0	0x914

/* Common EP control bits */
#define EPENA		BIT(31)
#define SETD0PID	BIT(28)	/* next data PID DATA0 (bulk) */
#define SNAK		BIT(27)
#define CNAK		BIT(26)
#define STALL		BIT(21)
#define USBACTEP	BIT(15)
#define EPTYPE_CTRL	(0 << 18)
#define EPTYPE_INT	(3 << 18)
#define EPTYPE_BULK	(2 << 18)
#define TXFNUM2		(2 << 22)
#define TXFNUM1		(1 << 22)	/* IN EP1 uses FIFO 1 */

/* FIFO size register format: [31:16]=depth [15:0]=start */
#define FIFOSIZE(start, depth)	(((depth) << 16) | (start))

/* DOEPTSIZ0 */
#define SUPCNT3		(3 << 29)	/* accept up to 3 back-to-back SETUPs */
#define PKTCNT1		BIT(19)

/* USB VID/PID matching Amlogic ROM device */
#define USB_VID		0x1b8e
#define USB_PID		0xc003

/* Amlogic vendor command codes */
#define REQ_WR_LARGE_MEM	0x11
#define REQ_RUN_IN_ADDR		0x05
#define REQ_IDENTIFY_HOST	0x20
#define REQ_READ_MEM		0x02

/*
 * Identify response -- deliberately distinct from GXL BROM (measured
 * 02 04 00 00 … on s7-apple). Host harness classifies SPL vs BROM by
 * these bytes (see tools/spl-loop.py classify_identify).
 * Layout: [0]=2 [1]=0x22 (SPL mark) [2]=0 [3]=0x20 (USB server stage).
 */
static const u8 identify_rsp[8] = { 0x02, 0x22, 0x00, 0x20, 0, 0, 0, 0 };

/*
 * Dual descriptors (S2):
 *  pre-id: BROM-like vendor so Linux does not bind cdc_acm before 02 22
 *  post soft-reenum: CDC-ACM 55 B (≤64, single EP0 packet; 75 B IAD timed out)
 */
/*
 * TPL_MESON_USB_REENUM (the *_libre_dfu_defconfig targets only): the TPL
 * re-enumerates as its own device, and its iSerialNumber is the chip id.
 */
#define USB_REENUM	CONFIG_IS_ENABLED(MESON_USB_REENUM)
#define STR_IDX_SERIAL	3
#define USB_ISERIAL	(USB_REENUM ? STR_IDX_SERIAL : 0)
/*
 * GUSBCFG on the PHY-POR start.  The re-enumeration uses U-Boot's
 * reconfig_usbd() value, 0x540f: TxFIFO rewind (14), turnaround 5, 16-bit
 * UTMI (3), HS/FS timeout calibration 7 -- the shape measured to answer a
 * fresh HS attach (E4b-E6, amlogic gxl/tpl-usb-stage-bus-reset-plan.md).
 */
#define USB_GUSBCFG_START	(USB_REENUM ? 0x540f : USBTRDTIM_5)

static const u8 dev_desc_vendor[18] = {
	18, 0x01, 0x00, 0x02,
	0x00, 0x00, 0x00, 0x40,
	(USB_VID & 0xff), (USB_VID >> 8),
	(USB_PID & 0xff), (USB_PID >> 8),
	0x00, 0x01, 0x00, 0x00, USB_ISERIAL, 0x01,
};
static const u8 dev_desc_cdc[18] = {
	18, 0x01, 0x00, 0x02,
	0x02, 0x00, 0x00, 0x40,
	(USB_VID & 0xff), (USB_VID >> 8),
	(USB_PID & 0xff), (USB_PID >> 8),
	0x00, 0x01, 0x00, 0x00, USB_ISERIAL, 0x01,
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

/* The BootROM's GRXFSIZ, GNPTXFSIZ, DIEPTXF1, DIEPTXF2, for the restart */
static u32 reenum_fifo[4];
static int reenum_fifo_valid;
#endif
/* Vendor bulk: EP2 OUT (WR_LARGE/BROM) + EP1 IN */
#define VENDOR_CFG_LEN 32
static const u8 cfg_desc_vendor[VENDOR_CFG_LEN] = {
	9, 0x02, VENDOR_CFG_LEN, 0, 1, 1, 0, 0x80, 250,
	9, 0x04, 0, 0, 2, 0xff, 0x00, 0x00, 0,
	7, 0x05, 0x02, 0x02, 0x00, 0x02, 0,	/* EP2 OUT bulk 512 (BROM WR_LARGE) */
	7, 0x05, 0x81, 0x02, 0x00, 0x02, 0,	/* EP1 IN bulk 512 */
};
#define ACM_CFG_LEN 55
static const u8 cfg_desc_cdc[ACM_CFG_LEN] = {
	9, 0x02, ACM_CFG_LEN, 0, 2, 1, 0, 0x80, 50,
	9, 0x04, 0, 0, 0, 0x02, 0x02, 0x01, 0,
	5, 0x24, 0x00, 0x10, 0x01,
	4, 0x24, 0x02, 0x02,
	5, 0x24, 0x06, 0x00, 0x01,
	9, 0x04, 1, 0, 2, 0x0a, 0x00, 0x00, 0,
	7, 0x05, 0x01, 0x02, 0x40, 0x00, 0,
	7, 0x05, 0x81, 0x02, 0x40, 0x00, 0,
};
static int acm_cdc_live;

/* EP0 control transfer state */
enum ep0_state {
	EP0_IDLE,
	EP0_DATA_OUT,	/* data stage in progress (OUT) */
	EP0_STATUS_IN,	/* sent IN data/ZLP, awaiting host OUT ZLP status */
};

static enum ep0_state ep0_state;
static u8  setup_buf[8];
static u8  ep0_data_buf[64];
static int ep0_data_len;

/* Bulk OUT transfer tracking */
static u32 bulk_dst_addr;
static u32 bulk_total_len;
static u32 bulk_recv_len;
static u32 bulk_arm_bytes;
static int bulk_dma_mode;
#if defined(CONFIG_MESON_GX_SPL_ENTRY)
/*
 * Bytes a COMPLETED transfer delivered to the chain's SPL entry address, so the
 * TPL can put a real spl_len in the handoff block instead of 0.  USB is a push:
 * TPL cannot ask for the DRAM stage, it can only notice that it arrived.  Only
 * the chain build defines SPL_ENTRY, so the shipping single-stage SPL compiles
 * none of this and stays byte-identical.
 */
static u32 usb_spl_len;

static void bulk_note_spl(void)
{
	if (bulk_total_len && bulk_recv_len >= bulk_total_len &&
	    bulk_dst_addr == CONFIG_MESON_GX_SPL_ENTRY)
		usb_spl_len = bulk_recv_len;
}

u32 meson_usb_spl_payload_len(void)
{
	return usb_spl_len;
}
#else
static inline void bulk_note_spl(void) { }
#endif
/*
 * Bulk path selector.  PIO and DMA are one image; which one runs is
 * bulk_dma_want.
 *
 * 🟢 BULK_DMA_REQ_ADDR (0xd900c194, magic 'DMA'<digit>) IS DELETED.  It was
 * documented as the host's A/B selector -- BROM writeMemory before RUN -- and
 * it had no reader, ever.  A selector with no reader is not a weak control, it
 * is an absent one, and the comment asserting it worked is the defect.  It also
 * sat inside BULK_DMA_EP0_SCRATCH, so re-pointing it would have been a second
 * wrong answer.
 *
 * 🟡 One correction to the row that named it while deleting it: it said the
 * word was "never read, never written".  The read half is right and is the
 * defect; the write half is NOT -- tools/spl-bulk-bench.py:36,469 writes it
 * from the host under --bulk-dma.  So that flag has been selecting nothing,
 * silently, for as long as it has existed.  The host writer is left in place
 * deliberately: it is a no-op today and removing it belongs with whoever
 * revives the DMA path and gives the choice a reader.
 *
 * Reviving buffer DMA should reintroduce the choice as a Kconfig default with
 * the stage ECHOING the arm it resolved, so a lost request degrades to
 * "changed nothing" and which arm ran is a fact the host READS rather than one
 * it assumes.
 *
 * 🟢 Separately fixed already: `bulk_dma_want` was never ASSIGNED, and SPL BSS
 * in this tree is not zeroed (measured: an uninitialised `mp_patblind` read,
 * 7ca00ff5e69), so the two tests that consume it (>= 3 grows GRXFSIZ, >= 2
 * selects INCR16) read whatever SRAM held.  It is now zeroed in
 * meson_usb_device_start() with the other session state, so re-arming
 * bulk_dma_enter() starts from the documented default rather than from
 * yesterday's SRAM.
 */
/*
 * Variant selects the AHB/FIFO shape so the burst + RX-depth ladder is one
 * image, A/B'd per run instead of per rebuild:
 *   1 = INCR4  bursts, BROM FIFO map (GRXFSIZ 256 words)
 *   2 = INCR16 bursts, BROM FIFO map
 *   3 = INCR16 bursts, RX grown to 512 words inside the quiesced window
 * GHWCFG3.DfifoDepth measured 712 words total on this part — that is the
 * whole budget, and why the old GRXFSIZ=768 attempts corrupted the map.
 */
static int bulk_dma_want;
/*
 * DMA completion watchdog.  A missed XFERCOMPL would otherwise spin the
 * SPL forever with EP0 unserviced — the host then loses the device with
 * no way to read back what happened (measured 2026-08-06 run 1/2).
 */
static u32 bulk_dma_stall;
#define BULK_DMA_STALL_MAX	600000u
static int usb_done;
/* Address from REQ_RUN_IN_ADDR; 0 if the host never sent one. */
static u32 usb_run_addr;
/* Count of REQ_IDENTIFY_HOST responses (SPL fingerprint 02 22 …). */
static int identify_hits;
/* 0 = EP0-only (pre-DDR re-enum); 1 = allow bulk to DRAM (post-DDR FIT). */
static int usb_allow_bulk;
static int usb_acm_mode;
static int acm_configured;
static int acm_dtr;
static u8 acm_line_coding[7] = {0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08};
#define ACM_RX_SZ 256
#define ACM_TX_SZ 256
static u8 acm_rx[ACM_RX_SZ], acm_tx[ACM_TX_SZ];
static unsigned int acm_rx_h, acm_rx_t, acm_tx_h, acm_tx_t;
static int acm_tx_busy;

/* SET_ADDRESS: apply after status phase */
static u8 pending_addr;

/* ------------------------------------------------------------------ */
/* FIFO read/write (slave mode, 32-bit FIFO port)                     */
/* ------------------------------------------------------------------ */

static void fifo_read(u32 fifo_off, u8 *buf, int len)
{
	/* Word path when dest is 4-aligned (DRAM FIT bulk) — less PIO churn */
	if (!((uintptr_t)buf & 3)) {
		u32 *p = (u32 *)(uintptr_t)buf;

		while (len >= 4) {
			*p++ = R(fifo_off);
			len -= 4;
		}
		buf = (u8 *)p;
	} else {
		while (len >= 4) {
			u32 w = R(fifo_off);

			buf[0] = w;
			buf[1] = w >> 8;
			buf[2] = w >> 16;
			buf[3] = w >> 24;
			buf += 4;
			len -= 4;
		}
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
		buf += 4; len -= 4;
	}
	if (len > 0) {
		u32 w = buf[0];
		if (len > 1) w |= buf[1] << 8;
		if (len > 2) w |= buf[2] << 16;
		W(fifo_off, w);
	}
}

/* ------------------------------------------------------------------ */
/* EP0 helpers                                                         */
/* ------------------------------------------------------------------ */

/*
 * EP0 DMA staging buffers.  GAHBCFG.DMA_EN is global — once the bulk path
 * turns it on, EP0 stops using the RX FIFO too, so control transfers need
 * their own DMA addresses or they simply stop working (measured: identify
 * dead for 12 s after a DMA payload, 2026-08-06 run 6).  SRAM, above the
 * 0xc000-byte SPL image and the DDR sticky block; D-cache is off.
 */
/*
 * 🟢 THE COLLISION THESE ADDRESSES USED TO CARRY IS RESOLVED.  It was:
 *
 *     FIFO_REQ_ADDR 0xd900c1a4 lay INSIDE EP0_DMA_SETUP (0xd900c1a0, 24 B)
 *     -- offset +4, i.e. wIndex/wLength of the first staged SETUP packet.
 *     A single control transfer would have overwritten the FIFO rung word.
 *
 * It was latent only because bulk_dma_enter() is unreachable today (buffer DMA
 * measured slower than PIO and wedged EP0 control IN -- see REQ_WR_LARGE_MEM),
 * which is luck rather than a bound.  The trio now lives at
 * GX_SRAM_EP0_DMA_ADDR, allocated in gx-handoff.h where an adjacency
 * _Static_assert can SEE both owners -- which is the reason this could not be
 * fixed here.  FIFO_REQ_ADDR does NOT move: the overlap was a consequence of
 * where EP0_DMA_SETUP sat, so moving the trio resolves it and no host tool has
 * to be re-pinned.
 *
 * 🟡 The chain/instrument lineage carries a SECOND collision at these
 * addresses -- EP0_DMA_IN 0xd900c200 is exactly DFC_EYE_STICKY there, and on a
 * TPL->SPL chain the DRAM instrument and this USB stage are the same image.
 * dram-fault-class.c does not exist on this branch, so that one is not present
 * here; it is named because the two lineages' GX_SRAM_EP0_DMA_ADDR differ for
 * exactly this reason (see the map comment in gx-handoff.h).
 *
 * 🔴 Unrun on silicon.  Every use below is still gated on bulk_dma_mode, which
 * nothing sets, so this is a build-verified relocation of dead code -- the
 * point of it is that the assert now exists, not that the buffers moved.
 */
#define EP0_DMA_SETUP	GX_SRAM_EP0_DMA_SETUP_ADDR	/* 3 back-to-back SETUPs = 24 B */
#define EP0_DMA_OUT	GX_SRAM_EP0_DMA_OUT_ADDR	/* data stage OUT, 64 B */
#define EP0_DMA_IN	GX_SRAM_EP0_DMA_IN_ADDR		/* data/status IN, 64 B */

_Static_assert(GX_SRAM_EP0_DMA_SETUP_LEN >= 3 * 8,
	       "EP0_DMA_SETUP must hold three 8-byte SETUP packets");
_Static_assert(GX_SRAM_EP0_DMA_OUT_LEN >= 64 && GX_SRAM_EP0_DMA_IN_LEN >= 64,
	       "EP0 DMA data buffers must hold a 64-byte control packet");

/*
 * The DWC2 DMA scratch and the FIFO rung word are the two live claims inside
 * GX_SRAM_USB_REQ, and this is the local half of the map's check: the range
 * has to hold them.  BULK_DMA_EP0_SCRATCH and FIFO_REQ_ADDR are #defined
 * further down, so the assert lives with them, not here.
 */

/* Arm EP0 OUT to receive the next SETUP (or status OUT ZLP). */
static void ep0_arm_setup(void)
{
	if (bulk_dma_mode)
		W(DOEPDMA0, EP0_DMA_SETUP);
	/* SUPCNT=3, PKTCNT=1, xfersize=24 (3 * 8 bytes for back-to-back setup) */
	W(DOEPTSIZ0, SUPCNT3 | PKTCNT1 | 24);
	SET(DOEPCTL0, EPENA | CNAK);
	ep0_state = EP0_IDLE;
}

/* Arm EP0 OUT to receive data stage bytes (not SETUP — no SUPCNT). */
static void ep0_arm_data(int len)
{
	if (bulk_dma_mode)
		W(DOEPDMA0, EP0_DMA_OUT);
	W(DOEPTSIZ0, PKTCNT1 | (u32)len);
	SET(DOEPCTL0, EPENA | CNAK);
}

/* GNPTXSTS: [15:0] NP TX FIFO free dwords; [19:16] NP TX queue slots */
#define GNPTXSTS		0x02c
#define GNPTXSTS_NPTXQ_SHIFT	16
#define GNPTXSTS_NPTXQ_MASK	(0xfu << GNPTXSTS_NPTXQ_SHIFT)

/* Send data on EP0 IN (PIO / non-DMA). Match Linux dwc2_hsotg_start_req. */
static void ep0_tx(const u8 *data, int len)
{
	int i;
	u32 need = (u32)((len + 3) / 4) + 1;
	u32 nptx = 0;
	u32 diep;

	if (bulk_dma_mode) {
		/* Buffer DMA: the core fetches the packet, no DFIFO push. */
		u8 *dst = (u8 *)EP0_DMA_IN;

		for (i = 0; i < len; i++)
			dst[i] = data[i];
		W(DIEPDMA0, EP0_DMA_IN);
		W(DIEPTSIZ0, (1u << 19) | (u32)len);
		diep = R(DIEPCTL0);
		diep &= ~((u32)STALL | (u32)SNAK);
		diep |= USBACTEP | EPENA | CNAK;
		W(DIEPCTL0, diep);
		ep0_state = EP0_STATUS_IN;
		return;
	}

	/*
	 * Linux order (gadget.c start_req, non-DMA IN):
	 *   1) wait NP TX queue slot + FIFO free space
	 *   2) DIEPTSIZ
	 *   3) DIEPCTL EPENA|CNAK
	 *   4) push packet to DFIFO
	 * Earlier FIFO-before-EPENA logged ID_OK but host ETIMEDOUT.
	 */
	for (i = 0; i < 100000; i++) {
		nptx = R(GNPTXSTS);
		if ((nptx & GNPTXSTS_NPTXQ_MASK) &&
		    (nptx & 0xffff) >= need)
			break;
		crude_udelay(1);
	}

	/*
	 * High half of progress = full GNPTXSTS low 16 + queue nibble.
	 *
	 * This is a destructive REPLACE of bits 31:16, not an OR, and it is
	 * reached only here -- on the slave-PIO path, below the bulk_dma_mode
	 * early return above.  So the clobber is conditional: it happens on
	 * every control-IN carrying data while PIO is live, and never again
	 * once bulk_dma_enter() has run.  Two consequences, both real:
	 *
	 *   - a breadcrumb in 31:16 set BEFORE this point is erased;
	 *   - the snapshot LEFT here afterwards is indistinguishable from a
	 *     breadcrumb OR'd in later (see U1P_DMA_ON/_STALL above).
	 *
	 * The word this belongs in is its own; parking a register capture in
	 * the middle of a flag bitmap is what costs the 16 bits.
	 */
	{
		u32 p = readl((void __iomem *)SPL_U1_PROG_ADDR);
		/*
		 * The free-dword count is taken as 10 bits, not 16.
		 *
		 * It used to be `nptx & 0xffff`, i.e. NPTxFSpcAvail[15:0], and
		 * nothing above bit 9 of that can ever be set: the whole DFIFO
		 * is 712 words (GHWCFG3.DfifoDepth) and the count is bounded by
		 * its own partition of it.  Taking only [9:0] therefore loses
		 * no information, and it buys two things that a bound read once
		 * from a register does not:
		 *
		 *   - word bits 26 and 27 are zero BY CONSTRUCTION rather than
		 *     by that bound, which is what lets U1P_STOP live at bit 26
		 *     as a flag instead of as an inference;
		 *   - bits 31:28 stop being "the queue nibble OR'd with
		 *     NPTxFSpcAvail[15:12]" and become the queue nibble, so the
		 *     host can read it without a caveat.
		 *
		 * PRICE, MEASURED AND NOT AS DESIGNED.  The design estimated
		 * "~4 B" for a `snap &= ~0x0c00u` after the fact.  Both
		 * spellings were built on the N136 gate stack and both cost
		 * EIGHT bytes: 34376 -> 34384 either way, against 34376 for the
		 * U1P_STOP #define alone.  The mov+and encoding story that
		 * would explain it for `&= ~0x0c00u` does NOT explain it for
		 * this one, which only narrows an immediate that was already
		 * there, so the cause is not established and is not guessed at
		 * here.  8 B of 10680 B spare buys a flag bit that is true by
		 * construction instead of by a register bound read once -- the
		 * kind of premise this workstream keeps finding to be false --
		 * so it is taken, with the price stated rather than estimated.
		 */
		u32 snap = (nptx & 0x3ff) |
			   (((nptx >> GNPTXSTS_NPTXQ_SHIFT) & 0xf) << 12);

		writel((p & 0xffff) | (snap << 16),
		       (void __iomem *)SPL_U1_PROG_ADDR);
	}

	W(DIEPTSIZ0, (1u << 19) | (u32)len);

	diep = R(DIEPCTL0);
	diep &= ~((u32)STALL | (u32)SNAK);
	diep |= USBACTEP | EPENA | CNAK;
	if ((diep & 0x3ff) == 0)
		diep |= 64;
	W(DIEPCTL0, diep);

	fifo_write(EP0_FIFO, data, len);

	/*
	 * Do NOT arm status OUT here — that races the IN data stage and was
	 * leaving host ETIMEDOUT despite ID_TX/ID_OK (2026-08-04).  Status
	 * OUT is armed on DIEPINT0 XferCompl below.
	 */
	ep0_state = EP0_STATUS_IN;
}

/* Send ZLP on EP0 IN (status for OUT control transfers). */
static void ep0_zlp(void)
{
	if (bulk_dma_mode)
		W(DIEPDMA0, EP0_DMA_IN);
	W(DIEPTSIZ0, (1 << 19) | 0);
	SET(DIEPCTL0, EPENA | CNAK);
	ep0_state = EP0_STATUS_IN;
}

/* ------------------------------------------------------------------ */
/* EP1 helpers                                                         */
/* ------------------------------------------------------------------ */

/*
 * RX FIFO budget (words): see GRXFSIZ in meson_usb_receive_fit().
 * Each HS bulk packet is 512 B = 128 words; leave headroom for status entries.
 * Arm multiple packets so the host can stream without waiting for re-arm each
 * 512 B -- required to approach USB2 HS bulk rates for multi-MiB FIT loads.
 */
/*
 * Max HS bulk packets per arm (×512 B). DWC2 DOEPTSIZ.PKTCNT is 10 bits
 * (≤1023). Measured ladder (CRC green, PPPS): 16→19.8, 32→20.7, 64→22.0.
 * Cap at 512 (=256 KiB) — large arms cut rearm gaps toward stretch ≥35.
 */
/* 1023 = DWC2 PKTCNT max; asm spin also rearms at EP2_RX_PKT_MAX */
#define EP2_RX_PKT_MAX	512

/*
 * BROM bulk OUT is EP2 (0x02), bulk IN EP1 (0x81) — measured p31 on PPPS.
 * Mode-5 ACM uses EP1 both ways; FIT WR_LARGE must arm EP2 OUT.
 */

/*
 * Expand RX FIFO after worldcup identify so multi-pkt EP2 OUT has room.
 * BROM leaves a small GRXFSIZ; multi-pkt then stalls (~2 MiB/s).
 * Use GRXFSIZ=768 words (same as phy_por path).
 */
static int bulk_fifo_ready;

static void bulk_prepare_fifos(void)
{
	/* Disabled: GRXFSIZ-only broke CRC; GRX+TX at start killed worldcup;
	 * GRX+GNPT on WR_LARGE → EP0 EIO on READ_MEM. Revisit after DMA path.
	 */
	bulk_fifo_ready = 1;
}

/* GAHBCFG HBSTLEN (linux dwc2): 3 = INCR4, 7 = INCR16 */
#define GAHBCFG_HBSTLEN_INCR4	(3u << 1)
#define GAHBCFG_HBSTLEN_INCR16	(7u << 1)

/* Saved BROM FIFO map, restored on the way back to slave PIO. */
static u32 bulk_dma_grxfsiz, bulk_dma_gnptxfsiz;

/*
 * SRAM parking slot for EP0's DMA pointers while GAHBCFG.DMA_EN is on.
 * Once the core is in buffer-DMA mode EVERY endpoint uses DMA, including
 * EP0 — a SETUP that arrives mid-bulk would otherwise be written through
 * whatever DOEPDMA0 happens to hold.  32 B, above the DDR sticky block
 * (dram-gx.c ends at 0xd900c160) and above the 0xc000-byte SPL image.
 */
#define BULK_DMA_EP0_SCRATCH	0xd900c180UL
_Static_assert(BULK_DMA_EP0_SCRATCH >= GX_SRAM_USB_REQ_ADDR &&
	       BULK_DMA_EP0_SCRATCH + 32 <= GX_SRAM_USB_REQ_ADDR +
					    GX_SRAM_USB_REQ_LEN,
	       "SRAM map: the EP0 DMA scratch is outside GX_SRAM_USB_REQ");

/*
 * Clean mid-session DWC2 mode reinit (slave PIO -> buffer DMA).
 *
 * A "clean reinit" here CANNOT mean CSFTRST or PHY POR: DCTL.SFTDISCON
 * resets to 1, so a core reset drops the terminations, and every
 * disconnect-based re-enum path measured on this PPPS rig never
 * reattaches (design doc Phase 0e / S2).  The link must stay up.
 *
 * What is left is a quiesce that the host tolerates: global OUT NAK.
 * The core NAKs OUT tokens (host just retries), which is the only state
 * in which switching GAHBCFG.DMA_EN and flushing the RX FIFO is defined.
 * The three earlier DMA attempts all flipped DMA_EN with OUT traffic
 * live and an RX FIFO holding slave-mode data — hence the hangs.
 */
static void bulk_dma_quiesce(void)
{
	int i;

	SET(DCTL, DCTL_SGOUTNAK);
	for (i = 0; i < 100000; i++) {
		if (R(GINTSTS) & INT_GOUTNAKEFF)
			break;
		crude_udelay(1);
	}
}

static void handle_setup(void);
static void handle_ep0_data_done(void);

/*
 * RX FIFO depth ladder.  BROM leaves GRXFSIZ = 256 words = 1 KiB = TWO 512 B
 * packets, so the host can only ever have two in flight before the core NYETs
 * while the CPU drains.  Every earlier attempt to grow it asked for 768 words
 * alone — more than the part's whole 712-word DFIFO (GHWCFG3.DfifoDepth, read
 * 2026-08-06) — so the map overlapped and "growing RX corrupts CRC" was never
 * a test of a legal map.  Budget here: RX + NPTX 128 + DIEPTXF1 64 <= 712.
 *
 * Applied in a global-OUT-NAK quiesced window, which the earlier bare
 * GRX+GNPT-on-WR_LARGE attempt did not have.
 */
#define FIFO_REQ_ADDR	0xd900c1a4UL
#define FIFO_REQ_MAGIC	0x00464946u	/* 'FIF' in bits 23:0 */
/*
 * The word this file and tools/spl-bulk-bench.py both name.  It is the claim
 * the EP0 DMA trio used to overlap, and the assert is the reason the trio
 * moved instead of this word: a host tool holds this address too, so moving it
 * would have re-pinned a tool to fix a stage-internal allocation.
 */
_Static_assert(FIFO_REQ_ADDR >= GX_SRAM_USB_REQ_ADDR &&
	       FIFO_REQ_ADDR + 4 <= GX_SRAM_USB_REQ_ADDR +
				    GX_SRAM_USB_REQ_LEN,
	       "SRAM map: the FIFO rung word is outside GX_SRAM_USB_REQ");
/*
 * THE DEFAULT IS THE NO-OP, AND THAT IS THE WHOLE POINT.
 *
 * This was 3 — the arm that GROWS the RX FIFO to 512 words, i.e. the thing
 * under test — while the CONTROL (keep the BROM 256-word map) was the arm that
 * required a host write to land.  That is backwards: the write is a BROM
 * writeMemory into SRAM the ROM itself uses, and on 2026-08-26 it did not
 * stick — `FIF0` read back as 0000c700.  A failed control write therefore
 * handed the run the TREATMENT arm, silently, and indistinguishably on the
 * wire: the run reported a control and executed the experiment (n145h).
 *
 * Rung 0 means fifo_apply() returns before touching GRXFSIZ/GNPTXFSIZ/
 * DIEPTXF1, so no magic, a corrupted magic, or an out-of-range digit all
 * leave the map exactly as the BROM left it.  Growing it now takes an
 * explicit, in-range 'FIF'<1..6>.
 *
 * The price is bulk write rate on a run that does not ask (23.2 vs 37.4
 * MiB/s, measured).  The price of the old default is an experiment whose two
 * arms cannot be told apart afterwards, which is not a trade.
 *
 * Generally: a failed experiment write must fail toward "no change", never
 * toward the thing under test.
 */
#define FIFO_RUNG_DEFAULT	0	/* no-op: keep the BROM map */
#define FIFO_RUNG_MAX		6
static int fifo_want, fifo_done;
static void dwc2_flush_fifos(void);

/*
 * {RX, NPTX, DIEPTXF1} in words.  Rungs 1-3 leave the TX side at the BROM-ish
 * 128/64 and total 576/640/704 of the 712-word DFIFO; rung 4 spends the last
 * 8 words on RX; rungs 5-6 buy RX depth from TX, which is affordable because
 * NPTX only has to hold an EP0 IN packet (64 B = 16 words) and DIEPTXF1 only
 * the tiny GXL-SPL banner — 128/64 is far more than either needs.
 */
/*
 * BROM FIFO map, captured before the first rung so stop() can restore it.
 *
 * SPL BSS in this tree is NOT zeroed on this path: crt0_64.S calls
 * board_init_f() at :98 and clears BSS at :161-166, i.e. AFTER, and
 * board_init_f() never returns here -- it contains dram_init(),
 * meson_usb_receive_fit() and gx_tpl_jump_to_spl().  So these three started as
 * whatever SRAM held, and BOTH tests on them were unsound in the same
 * direction: `if (!saved_grxfsiz)` (garbage-nonzero => never capture) and
 * `if (saved_grxfsiz)` in stop() (garbage-nonzero => write three garbage words
 * into GRXFSIZ/GNPTXFSIZ/DIEPTXF1 before handing the core on).  The restore
 * half of the very FIFO ladder the rung-0 default above exists to control
 * could not restore what it never captured.
 *
 * Two changes, and the second is the one that generalises: capture happens
 * unconditionally at device start -- so it also covers rung 0, where
 * fifo_apply() returns early and used to leave nothing saved -- and the
 * "have I captured?" flag is a MAGIC, established by the capture itself, not a
 * zero asserted by a declaration that this path never executes.
 */
#define FIFO_SAVED_MAGIC	0x46494653u	/* 'FIFS' */
static u32 fifo_saved_magic;
static u32 saved_grxfsiz, saved_gnptxfsiz, saved_dieptxf1;

static void fifo_capture(void)
{
	saved_grxfsiz   = R(GRXFSIZ);
	saved_gnptxfsiz = R(GNPTXFSIZ);
	saved_dieptxf1  = R(DIEPTXF1);
	fifo_saved_magic = FIFO_SAVED_MAGIC;
}

static void fifo_apply(void)
{
	static const u16 map[FIFO_RUNG_MAX][3] = {
		{ 384, 128, 64 }, { 448, 128, 64 }, { 512, 128, 64 },
		{ 520, 128, 64 }, { 584,  88, 40 }, { 648,  32, 32 },
	};
	const u16 *m;

	if (fifo_want < 1 || fifo_want > FIFO_RUNG_MAX)
		return;
	m = map[fifo_want - 1];
	if (fifo_saved_magic != FIFO_SAVED_MAGIC)
		fifo_capture();		/* belt and braces; start() already did */
	W(GRXFSIZ, m[0]);
	W(GNPTXFSIZ, FIFOSIZE(m[0], m[1]));
	W(DIEPTXF1, FIFOSIZE(m[0] + m[1], m[2]));
	dwc2_flush_fifos();
}


static void bulk_dma_enter(void)
{
	int i;

	/*
	 * Idempotent: the host issues a small WR_LARGE probe before the real
	 * payload, so this runs more than once per session.  Re-quiescing an
	 * already-DMA core STALLed the next bulk OUT ("Pipe error").
	 */
	if (bulk_dma_mode)
		return;

	/* Let the WR_LARGE status ZLP leave the TX FIFO before the switch. */
	for (i = 0; i < 2000; i++) {
		if (R(DIEPINT0) & DIEPINT_XFERCOMPL)
			break;
		crude_udelay(1);
	}

	bulk_dma_quiesce();

	/*
	 * Give EP0 valid DMA addresses BEFORE DMA_EN, but do NOT re-arm it:
	 * the pointers alone are mandatory (with DOEPDMA0 left stale the core
	 * STALLs the first bulk OUT — measured "Pipe error"), while arming a
	 * second OUT endpoint inside this window stops EP2 streaming outright.
	 */
	W(DOEPDMA0, EP0_DMA_SETUP);
	W(DIEPDMA0, EP0_DMA_IN);

	/* RX FIFO must be empty: in DMA mode the core drains it over AHB. */
	W(GRSTCTL, RXFFLSH);
	for (i = 0; i < 100000; i++) {
		if (!(R(GRSTCTL) & RXFFLSH))
			break;
		crude_udelay(1);
	}

	/* RXFLVL carries no OUT data in DMA mode — stop the poll loop popping. */
	/*
	 * RX depth only matters as staging before the AHB master drains it,
	 * but 256 words (BROM) is two packets — one NYET away from stalling
	 * the endpoint.  The quiesced window is the only safe place to move
	 * the map, and 512+128 stays inside the 712-word DFIFO.
	 */
	if (bulk_dma_want >= 3) {
		bulk_dma_grxfsiz = R(GRXFSIZ);
		bulk_dma_gnptxfsiz = R(GNPTXFSIZ);
		W(GRXFSIZ, 512);
		W(GNPTXFSIZ, FIFOSIZE(512, 128));
	}

	W(GINTMSK, R(GINTMSK) & ~INT_RXFLVL);
	W(GAHBCFG, GAHBCFG_DMA_EN | BIT(0) |
		   (bulk_dma_want >= 2 ? GAHBCFG_HBSTLEN_INCR16 :
					 GAHBCFG_HBSTLEN_INCR4));
	bulk_dma_mode = 1;
	bulk_dma_stall = 0;
	/*
	 * EP0 is re-armed with its DMA address only AFTER the payload, in
	 * meson_usb_receive_fit(): arming a second OUT endpoint inside this
	 * window stopped EP2 from streaming at all (measured — the 4 MiB
	 * write timed out instead of running at 21 MiB/s).
	 */
	u1_prog_or(U1P_DMA_ON);
}

static void ep2_bulk_out_arm(void)
{
	u32 remain = bulk_total_len - bulk_recv_len;
	u32 npkt;
	u32 xfer;

	if (remain == 0)
		return;

	npkt = (remain + 511) / 512;
	if (npkt > EP2_RX_PKT_MAX)
		npkt = EP2_RX_PKT_MAX;
	if (npkt < 1)
		npkt = 1;
	xfer = npkt * 512;
	if (xfer > remain)
		xfer = remain;

	W(DAINTMSK, R(DAINTMSK) | (1u << (16 + 2)));
	W(DOEPTSIZ2, (npkt << 19) | xfer);
	bulk_arm_bytes = xfer;

	if (bulk_dma_mode) {
		/*
		 * Buffer-DMA bulk OUT.  Mode is already live (bulk_dma_enter);
		 * per-arm work is only the destination + size.  D-cache is off
		 * in SPL, so the AHB master's writes need no maintenance.
		 */
		W(DOEPDMA2, bulk_dst_addr + bulk_recv_len);
		W(DOEPINT2, R(DOEPINT2));
		W(DOEPCTL2, USBACTEP | EPTYPE_BULK | EPENA | CNAK | 512);
		return;
	}

	/* Slave PIO: force EPENA|CNAK|USBACTEP — no SNAK dance (was slower). */
	W(DOEPCTL2, USBACTEP | EPTYPE_BULK | EPENA | CNAK | 512);
}

/*
 * Buffer-DMA bulk OUT (EP2): tried mid-worldcup with DOEPDMA set at arm
 * (see ep2_bulk_out_arm). Prior hang was p40/p41 without addr-first order.
 * bulk_slave_pump (p49–p51) stays disabled.
 */

/* ------------------------------------------------------------------ */
/* Control transfer dispatch                                           */
/* ------------------------------------------------------------------ */


static void acm_rx_push(const u8 *data, int len)
{
	int i;
	for (i = 0; i < len; i++) {
		unsigned int n = (acm_rx_h + 1) % ACM_RX_SZ;
		if (n == acm_rx_t)
			break;
		acm_rx[acm_rx_h] = data[i];
		acm_rx_h = n;
	}
}

static void acm_ep1_out_arm(void)
{
	W(DOEPTSIZ1, (1u << 19) | 64);
	SET(DOEPCTL1, EPENA | CNAK);
}

/*
 * After 02 22 only: dedicate TX FIFO 1 for EP1 bulk IN.
 * Sharing NP (TXFNUM0) with EP0 shipped identify residue on bulk 0x81
 * (measured S2). Never program DIEPTXF* before identify.
 * Layout [31:16]=depth [15:0]=start (Amlogic FIFOSIZE).
 */
static void acm_arm_bulk_in(void)
{
	u32 nptx, np_start, np_depth, d1;
	int fi;
	u32 mps = acm_cdc_live ? 64u : 512u;

	nptx = R(GNPTXFSIZ);
	np_start = nptx & 0xffff;
	np_depth = nptx >> 16;
	d1 = np_start + np_depth;
	/* 128 dwords = 512 B — one HS bulk max packet */
	W(DIEPTXF1, FIFOSIZE(d1, 128));
	W(GRSTCTL, TXFFLSH | (1u << 6));
	for (fi = 0; fi < 10000; fi++) {
		if (!(R(GRSTCTL) & TXFFLSH))
			break;
		crude_udelay(1);
	}
	W(DIEPCTL1, USBACTEP | EPTYPE_BULK | TXFNUM1 | SNAK | mps);
	W(DOEPCTL1, USBACTEP | EPTYPE_BULK | SNAK | mps);
	acm_ep1_out_arm();
	acm_configured = 1;
	acm_tx_busy = 0;
}

static void acm_try_tx(void)
{
	u8 buf[64];
	unsigned int n = 0;
	unsigned int save_t;
	u32 dtx, need, diep;
	int i;
	u32 mps = acm_cdc_live ? 64u : 512u;

	/* Never TX while EP0 control is in flight — corrupts NP / host view */
	if (ep0_state != EP0_IDLE)
		return;
	if (acm_tx_busy || !acm_configured || acm_tx_h == acm_tx_t)
		return;
	save_t = acm_tx_t;
	while (n < 64 && acm_tx_t != acm_tx_h) {
		buf[n++] = acm_tx[acm_tx_t];
		acm_tx_t = (acm_tx_t + 1) % ACM_TX_SZ;
	}
	need = (n + 3) / 4;
	for (i = 0; i < 100000; i++) {
		dtx = R(DTXFSTS1) & 0xffff;
		if (dtx >= need)
			break;
		crude_udelay(1);
	}
	if (i >= 100000) {
		acm_tx_t = save_t;
		return;
	}
	W(DIEPTSIZ1, (1u << 19) | n);
	diep = R(DIEPCTL1);
	diep &= ~((u32)STALL | (u32)SNAK | (0xfu << 22) | 0x7ffu);
	diep |= USBACTEP | EPENA | CNAK | EPTYPE_BULK | TXFNUM1 | mps;
	W(DIEPCTL1, diep);
	fifo_write(EP1_FIFO, buf, (int)n);
	acm_tx_busy = 1;
}

void meson_usb_acm_putc(char c)
{
	unsigned int n;
	if (c == '\n')
		meson_usb_acm_putc('\r');
	n = (acm_tx_h + 1) % ACM_TX_SZ;
	if (n == acm_tx_t)
		return;
	acm_tx[acm_tx_h] = (u8)c;
	acm_tx_h = n;
	/* TX only from poll loop when EP0 idle */
}

void meson_usb_acm_puts(const char *s)
{
	while (*s)
		meson_usb_acm_putc(*s++);
}

int meson_usb_acm_getc(void)
{
	int c;
	if (acm_rx_t == acm_rx_h)
		return -1;
	c = acm_rx[acm_rx_t];
	acm_rx_t = (acm_rx_t + 1) % ACM_RX_SZ;
	return c;
}

int meson_usb_acm_configured(void)
{
	return acm_configured && acm_dtr;
}

static void handle_setup(void)
{
	u8  bmRequestType = setup_buf[0];
	u8  bRequest      = setup_buf[1];
	u16 wValue  = (u16)(setup_buf[2] | (setup_buf[3] << 8));
	u16 wIndex  = (u16)(setup_buf[4] | (setup_buf[5] << 8));
	u16 wLength = (u16)(setup_buf[6] | (setup_buf[7] << 8));

	if ((bmRequestType & 0x60) == 0x00) {
		/* Standard request */
		switch (bRequest) {
		case 0x05: /* SET_ADDRESS */
			/*
			 * DCFG.DevAddr is written on the SETUP, as Linux dwc2
			 * and the databook do: the core holds the new address
			 * back until the status stage completes.  Deferring
			 * the write to an OUT completion never applies it --
			 * a no-data SET_ADDRESS has an IN status stage -- so
			 * the device stayed at address 0 and the host's next
			 * request got no handshake (xHCI -71).  Only a fresh
			 * attach reaches this; the BootROM's session arrives
			 * already addressed.
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
				const u8 *dd = acm_cdc_live ? dev_desc_cdc :
							      dev_desc_vendor;
				int len = 18;

				if (wLength < len)
					len = wLength;
				ep0_tx(dd, len);
				break;
			}
			case 0x02: {
				const u8 *cd = acm_cdc_live ? cfg_desc_cdc :
							      cfg_desc_vendor;
				int len = acm_cdc_live ? ACM_CFG_LEN :
							 VENDOR_CFG_LEN;

				if (wLength < len)
					len = wLength;
				ep0_tx(cd, len);
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
			if (usb_acm_mode && acm_cdc_live) {
				acm_arm_bulk_in(); /* DIEPTXF1 + EP1 bulk 64 */
				acm_ep1_out_arm();
			} else if (!usb_acm_mode) {
				/*
				 * BROM: EP2 OUT bulk + EP1 IN bulk.  Configuring
				 * resets the host's bulk data toggles to DATA0
				 * (USB 2.0 9.1.1.5), so reset ours: these
				 * endpoints may hold the BootROM's toggle, and a
				 * mismatched OUT toggle makes the core ACK and
				 * drop every packet (measured on the G12 twin).
				 */
				W(DOEPCTL2, USBACTEP | EPTYPE_BULK | SNAK |
				  SETD0PID | 512);
				W(DIEPCTL1, USBACTEP | EPTYPE_BULK | TXFNUM1 |
				  SNAK | SETD0PID | 512);
			}
			ep0_zlp();
			ep0_arm_setup();
			break;
		default:
			SET(DIEPCTL0, STALL);
			ep0_arm_setup();
			break;
		}
	} else if ((bmRequestType & 0x60) == 0x40) {
		/* Vendor request (Amlogic ROM protocol) */
		switch (bRequest) {
		case REQ_IDENTIFY_HOST:
			identify_hits++;
			u1_prog_or(U1P_IDENTIFY_TX);
			ep0_tx(identify_rsp, sizeof(identify_rsp));
			u1_prog_or(U1P_IDENTIFY_OK);
			/*
			 * After first 02 22: ensure EP2 bulk OUT is active.
			 * Do NOT touch DIEPCTL1/DIEPTXF* here (mode-5: TXFNUM1
			 * without dedicated DIEPTXF1 wedges bulk — p26 still
			 * timed out when we set both).
			 */
			if (usb_allow_bulk && !usb_acm_mode) {
				u32 c = R(DOEPCTL2);

				if (!(c & USBACTEP))
					W(DOEPCTL2, USBACTEP | EPTYPE_BULK |
						  SNAK | 512);
			}
			break;
		case REQ_WR_LARGE_MEM:
			if (!usb_allow_bulk) {
				SET(DIEPCTL0, STALL);
				ep0_arm_setup();
				break;
			}
			/* breadcrumb: saw WR_LARGE SETUP */
			writel(0x5354B200u, (void __iomem *)0xd900c120UL);
			ep0_state = EP0_DATA_OUT;
			ep0_arm_data(16);
			break;
		case REQ_READ_MEM: {
			/* pyamlboot readSimpleMemory: max 64 B, addr in wValue:wIndex */
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
			/* Word path when aligned — better for DRAM */
			if (!(addr & 3) && !(len & 3)) {
				for (i = 0; i < len; i += 4) {
					u32 w = readl((void __iomem *)(uintptr_t)(addr + i));

					buf[i] = w;
					buf[i + 1] = w >> 8;
					buf[i + 2] = w >> 16;
					buf[i + 3] = w >> 24;
				}
			} else {
				for (i = 0; i < len; i++)
					buf[i] = readb((void __iomem *)(uintptr_t)(addr + i));
			}
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
	} else if ((bmRequestType & 0x60) == 0x20) {
		/* Class (CDC-ACM) */
		switch (bRequest) {
		case 0x20: /* SET_LINE_CODING */
			ep0_state = EP0_DATA_OUT;
			ep0_arm_data(7);
			break;
		case 0x21: { /* GET_LINE_CODING */
			int len = 7;

			if (wLength < len)
				len = wLength;
			ep0_tx(acm_line_coding, len);
			break;
		}
		case 0x22: /* SET_CONTROL_LINE_STATE */
			acm_dtr = wValue & 0x1;
			ep0_zlp();
			ep0_arm_setup();
			if (acm_dtr && acm_configured)
				meson_usb_acm_puts("GXL-SPL\n");
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

static void handle_ep0_data_done(void)
{
	u8 bRequest = setup_buf[1];

	switch (bRequest) {
	case REQ_WR_LARGE_MEM:
		/* Close out the previous transfer before its state is lost. */
		bulk_note_spl();
		/*
		 * REFUSE the request if its 16-byte data stage never arrived,
		 * and refuse it the way a USB device is supposed to: a
		 * protocol STALL, then re-arm for the next SETUP.
		 *
		 * handle_setup() has two call sites -- the GRXSTSP SETUPDONE
		 * entry inside meson_usb_poll_once()'s drain loop, and the
		 * latched DOEPINT0.SETUP, which runs AFTER that loop -- and
		 * which one wins is decided by the HOST.  Measured
		 * deterministic per host: the latch in 25 of 25 arms behind an
		 * AMD xHCI plus two cascaded VIA hubs, the SETUPDONE entry in
		 * 24 of 25 behind an Intel one plus a single hub.
		 *
		 * When the latch wins, ep0_state is already EP0_DATA_OUT by
		 * the time the core's ep=0 BCnt=0 PktSts=3 entry for the
		 * SETUP's OWN armed EP0 OUT transfer is popped, so
		 * handle_rxfifo() calls this function BEFORE the payload
		 * exists.  Parsing then produced a wild bulk destination out
		 * of BSS -- 0x9c5ea102 on a 2 GB board, outside DRAM and
		 * unaligned -- and a 512-byte PIO store to it.
		 *
		 * ep0_data_len is ESTABLISHED at session start below, so this
		 * is a real test rather than a test against inherited SRAM.
		 *
		 * The recovery shape is upstream's.  Linux answers a control
		 * request it cannot honour with dwc2_hsotg_stall_ep0() --
		 * set STALL on the EP0 control register and then "complete
		 * won't be called, so we enqueue setup request here",
		 * dwc2_hsotg_enqueue_setup() (drivers/usb/dwc2/gadget.c:1893,
		 * :1908, :1919 @ linux-6.18.y-lc).  A bare break with no
		 * re-arm is NOT that: it strands the endpoint, the host never
		 * sees a status stage, and a LATER request times out.  This is
		 * also this file's own established idiom for a refused control
		 * request (:1255, :1274, :1302, :1346, :1373, :1378).
		 */
		if (ep0_data_len < 16) {
			SET(DIEPCTL0, STALL);
			ep0_arm_setup();
			break;
		}
		bulk_dst_addr  = ep0_data_buf[0] | ((u32)ep0_data_buf[1] << 8) |
			((u32)ep0_data_buf[2] << 16) | ((u32)ep0_data_buf[3] << 24);
		bulk_total_len = ep0_data_buf[4] | ((u32)ep0_data_buf[5] << 8) |
			((u32)ep0_data_buf[6] << 16) | ((u32)ep0_data_buf[7] << 24);
		bulk_recv_len  = 0;
		/*
		 * CONSUME the data stage, so a LATER REQ_WR_LARGE_MEM whose
		 * own data stage is missed hits the STALL above instead of
		 * silently replaying this header.  Without it the guard
		 * protects only the first request of a session.
		 */
		ep0_data_len = 0;
		/*
		 * FIFO map FIRST, before the status ZLP is queued.
		 *
		 * fifo_apply() flushes the TX FIFOs (GRSTCTL.TXFFLSH|TXFNUM_ALL).
		 * Done after ep0_zlp(), that flush can destroy the not-yet-sent
		 * status ZLP of this very WR_LARGE, and the host then fails the
		 * control transfer -- measured as a ~1-in-5 flake on the FIRST
		 * WR_LARGE of a session, hitting a 64 B probe as readily as a
		 * 64 KiB burst, and as both ETIMEDOUT and EIO.  Waiting for the
		 * ZLP to drain first did NOT fix it (5/8); doing the FIFO work
		 * before there is a ZLP to lose removes the race instead of
		 * narrowing it.  With the block disabled entirely the flake is
		 * absent 8/8, which is what pinned it here.
		 *
		 * It cannot move earlier still: growing GRXFSIZ at identify time
		 * kills the 02 22 (7c3280d6ac0).  The host tolerates a NAKed /
		 * delayed status stage; it cannot tolerate a lost one.
		 */
		if (!fifo_done) {
			fifo_done = 1;
			bulk_dma_quiesce();
			fifo_apply();
			SET(DCTL, DCTL_CGOUTNAK);
		}
		ep0_zlp();
		ep0_arm_setup();
		bulk_prepare_fifos();
		/*
		 * EP0 is idle here (ZLP queued, next SETUP armed) and no bulk
		 * data is in flight yet — the one point in the session where
		 * the slave->DMA switch is safe.  Global OUT NAK covers the
		 * window; the host sees NAKs on its first OUT tokens and
		 * retries.  bulk_dma_want=0 keeps the proven PIO asm path.
		 */
		/*
		 * Buffer DMA is not reachable any more: it measured slower
		 * than PIO (21.1 vs 37.x) and permanently wedged EP0 control
		 * IN, which is exactly the lifecycle breakage this path now
		 * has to avoid.  Kept in history (9a6ba3c46a1) + design doc.
		 */
		ep2_bulk_out_arm();
		break;
	case REQ_RUN_IN_ADDR:
		/*
		 * The address the host names here is the only statement it makes
		 * about what it just wrote; keep it for the handoff block rather
		 * than throwing it away with the request.
		 */
		if (ep0_data_len >= 4)
			usb_run_addr = ep0_data_buf[0] |
				((u32)ep0_data_buf[1] << 8) |
				((u32)ep0_data_buf[2] << 16) |
				((u32)ep0_data_buf[3] << 24);
		ep0_zlp();
		ep0_arm_setup();
		usb_done = 1;
		break;
	case 0x20: /* SET_LINE_CODING */
		if (ep0_data_len >= 7) {
			int i;

			for (i = 0; i < 7; i++)
				acm_line_coding[i] = ep0_data_buf[i];
		}
		ep0_zlp();
		ep0_arm_setup();
		break;
	default:
		ep0_arm_setup();
		break;
	}
}

/* ------------------------------------------------------------------ */
/* Main RXFIFO event handler                                           */
/* ------------------------------------------------------------------ */

static void handle_rxfifo(void)
{
	u32 status = R(GRXSTSP);
	int ep     = EPNUM(status);
	int bcnt   = BCNT(status);
	int pktsts = PKTSTS(status);
	int bulk_hot = (bulk_total_len > 0 && bulk_recv_len < bulk_total_len);

	/* Skip prog breadcrumbs during bulk — SRAM writes cost PIO rate */
	if (!bulk_hot)
		u1_prog_or(U1P_RXFLVL);

	switch (pktsts) {
	case PKTSTS_SETUPRX:
		/* Read 8-byte SETUP packet; dispatch on DOEPINT.SETUP. */
		fifo_read(EP0_FIFO, setup_buf, 8);
		if (!bulk_hot)
			u1_prog_or(U1P_SETUPRX);
		break;

	case PKTSTS_SETUPDONE:
		if (!bulk_hot)
			u1_prog_or(U1P_SETUPDONE);
		if (ep0_state == EP0_IDLE)
			handle_setup();
		break;

	case PKTSTS_OUTRX:
		if (ep == 0) {
			if (ep0_state == EP0_DATA_OUT && bcnt > 0) {
				if (bcnt > (int)sizeof(ep0_data_buf))
					bcnt = sizeof(ep0_data_buf);
				fifo_read(EP0_FIFO, ep0_data_buf, bcnt);
				ep0_data_len = bcnt;
			}
			/* else: status OUT ZLP for IN transfer -- ignore */
		} else if (ep == 1 && bcnt > 0 && usb_acm_mode) {
			u8 tmp[64];
			int n = bcnt;
			if (n > 64)
				n = 64;
			fifo_read(EP1_FIFO, tmp, n);
			acm_rx_push(tmp, n);
			acm_ep1_out_arm();
		} else if (ep == 2 && bcnt > 0 && !usb_acm_mode) {
			if (bulk_dma_mode) {
				/* DMA owns the payload; pop status only if still RXFLVL */
				break;
			}
			/* BROM protocol bulk OUT → DRAM (fast 512 B path) */
			u8 *dst = (u8 *)(uintptr_t)(bulk_dst_addr +
						 bulk_recv_len);

			if (!((uintptr_t)dst & 3) && bcnt == 512) {
				u32 *p = (u32 *)(uintptr_t)dst;
				int i;

				/* 8-wide unroll: A53 fills write buffer better */
				for (i = 0; i < 128; i += 8) {
					p[i + 0] = R(EP2_FIFO);
					p[i + 1] = R(EP2_FIFO);
					p[i + 2] = R(EP2_FIFO);
					p[i + 3] = R(EP2_FIFO);
					p[i + 4] = R(EP2_FIFO);
					p[i + 5] = R(EP2_FIFO);
					p[i + 6] = R(EP2_FIFO);
					p[i + 7] = R(EP2_FIFO);
				}
			} else {
				fifo_read(EP2_FIFO, dst, bcnt);
			}
			bulk_recv_len += bcnt;
			/* Rearm on OUTDONE only (OUTRX+OUTDONE double-arm corrupted CRC). */
		}
		break;

	case PKTSTS_OUTDONE:
		if (ep == 0) {
			if (ep0_state == EP0_DATA_OUT) {
				/*
				 * OUT data stage complete.  Process header
				 * and send IN ZLP status.
				 */
				handle_ep0_data_done();
			} else if (ep0_state == EP0_STATUS_IN) {
				/*
				 * Host's OUT ZLP acknowledged our IN data.
				 * Apply deferred SET_ADDRESS if pending.
				 */
				if (pending_addr) {
					u32 dcfg = R(DCFG);

					dcfg &= ~(0x7f << 4);
					dcfg |= (u32)pending_addr << 4;
					W(DCFG, dcfg);
					pending_addr = 0;
				}
				ep0_arm_setup();
			}
		} else if (ep == 2 && !usb_acm_mode) {
			if (bulk_recv_len < bulk_total_len &&
			    !(R(DOEPCTL2) & EPENA))
				ep2_bulk_out_arm();
		} else if (ep == 1 && usb_acm_mode) {
			/* ACM path re-arms in OUTRX */
		}
		break;

	case PKTSTS_GLOBALOUTNAK:
	default:
		break;
	}
}

/* ------------------------------------------------------------------ */
/* DWC2 initialisation                                                 */
/* ------------------------------------------------------------------ */

static void dwc2_flush_fifos(void)
{
	int i;

	W(GRSTCTL, TXFFLSH | TXFNUM_ALL);
	for (i = 0; i < 10000; i++) {
		if (!(R(GRSTCTL) & TXFFLSH))
			break;
		crude_udelay(1);
	}

	W(GRSTCTL, RXFFLSH);
	for (i = 0; i < 10000; i++) {
		if (!(R(GRSTCTL) & RXFFLSH))
			break;
		crude_udelay(1);
	}
}

static void dwc2_core_soft_reset(void)
{
	int i;

	/* Wait for AHB idle before resetting */
	for (i = 0; i < 100000; i++) {
		if (R(GRSTCTL) & AHBIDLE)
			break;
		crude_udelay(1);
	}

	W(GRSTCTL, CSFTRST);
	for (i = 0; i < 100000; i++) {
		if (!(R(GRSTCTL) & CSFTRST))
			break;
		crude_udelay(1);
	}
	crude_udelay(3);	/* phy clock stable */
}

static void dwc2_handle_reset(void)
{
	/* Clear device address */
	CLR(DCFG, 0x7f << 4);
	pending_addr = 0;

	/* NAK all OUT endpoints */
	SET(DOEPCTL0, SNAK);
	SET(DOEPCTL1, SNAK);

	dwc2_flush_fifos();
	ep0_arm_setup();
}

/* ------------------------------------------------------------------ */
/* Public entry point                                                  */
/* ------------------------------------------------------------------ */

/* Soft-disconnect: PHY POR so host sees a real disconnect. */
static void meson_usb_phy_disconnect(void)
{
	void __iomem *phy = (void __iomem *)USB2_PHY_BASE;
	u32 u2p = readl(phy + U2P_R0);

	u2p &= ~(U2P_R0_DM_PULLDOWN | U2P_R0_DP_PULLDOWN);
	u2p |= U2P_R0_ID_PULLUP | U2P_R0_POWER_ON_RESET;
	writel(u2p, phy + U2P_R0);
	SET(DCTL, SFTDISCON);
	crude_mdelay(50);
}

/**
 * meson_usb_device_start() - PHY POR disconnect + DWC2 HS device connect.
 * @allow_bulk: 0 = identify/control only (pre-DDR); 1 = bulk FIT to DRAM
 */
static void meson_usb_device_start(int allow_bulk, int phy_por)
{
	/*
	 * Core capability breadcrumbs, read BEFORE any PHY/FIFO work so they
	 * reflect the core as BROM left it. Host reads them live with
	 * pyamlboot readSimpleMemory() -- no watchdog round trip needed.
	 * 0xd900c11c / 0xd900c124 are the only unclaimed words in the window.
	 */
	writel(R(GSNPSID), (void __iomem *)0xd900c11cUL);
	writel(R(GHWCFG2), (void __iomem *)0xd900c124UL);

	/*
	 * Capture the BROM FIFO map HERE, before any rung and regardless of
	 * which one is requested -- rung 0 returns from fifo_apply() before
	 * touching a register, so the capture cannot live there.
	 */
	fifo_capture();

	ep0_state      = EP0_IDLE;
	usb_done       = 0;
	pending_addr   = 0;
	/*
	 * ESTABLISH the documented defaults instead of asserting them in a
	 * comment.  BSS is not zeroed on this path (see fifo_capture()), so
	 * "0 if the host never sent one" and "0 still means TPL did not load
	 * this" were claims about a value nothing had written.  bulk_dma_want
	 * is here for the same reason: its only consumers live in
	 * bulk_dma_enter(), which is unreachable today, so this is latent --
	 * but a selector that is never assigned must not be left to read
	 * whatever SRAM held if that path is ever re-armed.
	 */
	usb_run_addr   = 0;
	bulk_dma_want  = 0;
#if defined(CONFIG_MESON_GX_SPL_ENTRY)
	usb_spl_len    = 0;
#endif
	/*
	 * ESTABLISH the EP0 data stage and the bulk destination.
	 *
	 * BSS is not zeroed on this path (see fifo_capture()): crt0_64.S
	 * clears it after board_init_f(), which never returns here.  So
	 * ep0_data_len, ep0_data_buf and bulk_dst_addr were previously
	 * whatever the mask ROM's own download session left in SRAM, and
	 * handle_ep0_data_done() parsed ep0_data_buf unconditionally -- a
	 * missed data stage did not fail, it "succeeded" on garbage and
	 * armed EP2 at a plausible-looking wild address instead of an
	 * obvious zero.  The ep0_data_len < 16 STALL above is only a real
	 * test because of this block.
	 */
	{
		int i;

		for (i = 0; i < (int)sizeof(ep0_data_buf); i++)
			ep0_data_buf[i] = 0;
	}
	ep0_data_len   = 0;
	bulk_dst_addr  = 0;
	bulk_recv_len  = 0;
	bulk_total_len = 0;
	bulk_arm_bytes = 0;
	bulk_dma_mode = 0;
	bulk_fifo_ready = 0;
	identify_hits  = 0;
	usb_allow_bulk = allow_bulk;
	usb_acm_mode = 0;
	acm_cdc_live = 0;
	acm_configured = 0;
	acm_dtr = 0;
	acm_tx_busy = 0;
	acm_rx_h = acm_rx_t = acm_tx_h = acm_tx_t = 0;

	/*
	 * phy_por=1: full PHY POWER_ON_RESET disconnect (design U1 re-enum).
	 * phy_por=0: vendor-style "worldcup" — soft SFTDISCON only, keep
	 *            link alive so host can re-identify without re-plug.
	 * Measured: vendor BL2 keeps same bus/addr and only changes identify
	 * (02 04 → 02 02 00 08); open SPL PHY-POR path was leaving a zombie.
	 */
	if (phy_por) {
		void __iomem *phy = (void __iomem *)USB2_PHY_BASE;
		u32 u2p;

		meson_usb_clk_enable();

		u2p = readl(phy + U2P_R0);
		/* Device mode: no DM/DP pulldown, ID pullup (meson-gxl-usb2) */
		u2p &= ~(U2P_R0_DM_PULLDOWN | U2P_R0_DP_PULLDOWN);
		u2p |= U2P_R0_ID_PULLUP | U2P_R0_POWER_ON_RESET;
		writel(u2p, phy + U2P_R0);
		crude_mdelay(5);
		u2p &= ~U2P_R0_POWER_ON_RESET;
		writel(u2p, phy + U2P_R0);
		crude_mdelay(5);
		/* second POR pulse */
		u2p |= U2P_R0_POWER_ON_RESET;
		writel(u2p, phy + U2P_R0);
		crude_mdelay(5);
		u2p &= ~U2P_R0_POWER_ON_RESET;
		writel(u2p, phy + U2P_R0);
		crude_mdelay(1);

		dwc2_core_soft_reset();
		W(GUSBCFG, FORCEDEVMODE | USB_GUSBCFG_START);
		/*
		 * Force B-session + VBUS valid — without this, post-DDR soft-
		 * connect sits in USBSUSP/SESSREQINT (p18 dump: DSTS.SUSPSTS,
		 * GINT SESSREQINT|USBSUSP, no USBRST; GUSBCFG lost host view).
		 */
		W(GOTGCTL, GOTGCTL_VBVALOEN | GOTGCTL_VBVALOVAL |
			   GOTGCTL_BVALOEN | GOTGCTL_BVALOVAL);
		crude_mdelay(25);

		SET(DCTL, SFTDISCON);
		crude_mdelay(10);
	} else {
		/*
		 * Worldcup (minimal): leave BROM GUSBCFG/GAHBCFG/FIFOs alone.
		 * Do NOT drain RXFIFO here — that discarded the host's identify
		 * SETUP (freestanding stub: only non-0x20 SETUP left).  Re-arm
		 * SETUP and process pending RX in the poll loop instead.
		 */
		W(DOEPMSK, R(DOEPMSK) | DOEPINT_SETUP | DOEPINT_XFERCOMPL |
			   DOEPINT_STUPPKTRCVD);
		W(DIEPMSK, R(DIEPMSK) | DIEPINT_XFERCOMPL | DIEPINT_TIMEOUT);
		W(DAINTMSK, R(DAINTMSK) | (1u << 0) | (1u << 1) |
			   (1u << 16) | (1u << 17));
		W(GINTMSK, R(GINTMSK) | INT_USBRST | INT_ENUMDONE | INT_RXFLVL |
			   INT_OEPINT | INT_IEPINT);
		writel(R(DIEPCTL0), (void __iomem *)0xd900c10cUL);
		writel(R(DCFG), (void __iomem *)0xd900c110UL);
		writel(R(GAHBCFG), (void __iomem *)0xd900c114UL);
		ep0_arm_setup();
		return;
	}

#if USB_REENUM
	/*
	 * The BootROM's own FIFO map, captured by meson_usb_receive_fit()
	 * before the disconnect.  The map below sums 912 words against GXL's
	 * 712-word DFIFO and put the EP0 TX FIFO past the end (-110 on every
	 * SETUP, E3e); the BootROM's is the one measured to work (E4g-E6).
	 */
	if (reenum_fifo_valid) {
		W(GRXFSIZ,   reenum_fifo[0]);
		W(GNPTXFSIZ, reenum_fifo[1]);
		W(DIEPTXF1,  reenum_fifo[2]);
		W(DIEPTXF2,  reenum_fifo[3]);
	} else
#endif
	{
		/* RX 3 KB for multi-packet bulk; TX 256 B each for EP0/EP1 */
		W(GRXFSIZ,   768);
		W(GNPTXFSIZ, FIFOSIZE(768, 64));
		W(DIEPTXF1,  FIFOSIZE(832, 64));
		W(DIEPTXF2,  FIFOSIZE(896, 16));
	}

	dwc2_flush_fifos();
	W(DCFG, DEVSPD_HS);
	/* Full EP masks — bare GINTMSK left host with zombie 1b8e (2026-08-04) */
	W(DOEPMSK, DOEPINT_SETUP | DOEPINT_XFERCOMPL | DOEPINT_STUPPKTRCVD);
	W(DIEPMSK, DIEPINT_XFERCOMPL | DIEPINT_TIMEOUT);
	/* EP0 + EP1 IN/OUT (bulk FIT / console) */
	W(DAINTMSK, (1u << 0) | (1u << 1) | (1u << 16) | (1u << 17));
	W(GINTMSK, INT_USBRST | INT_ENUMDONE | INT_RXFLVL |
		   INT_OEPINT | INT_IEPINT);
	W(GAHBCFG, BIT(0)); /* GBLINTRMSK */
	ep0_arm_setup();
	meson_usb_glue_device_mode(); /* U2D_ACT after PHY/core ready */
	/* Re-assert device mode after FIFO/glue (FORCEDEVMODE can drop) */
	W(GUSBCFG, R(GUSBCFG) | FORCEDEVMODE | USB_GUSBCFG_START);
	W(GOTGCTL, R(GOTGCTL) | GOTGCTL_VBVALOEN | GOTGCTL_VBVALOVAL |
		   GOTGCTL_BVALOEN | GOTGCTL_BVALOVAL);
	u1_prog_or(U1P_PHYPOR);
	/* soft-connect: clear SFTDISCON (linux dwc2 gadget connect) */
	{
		u32 dctl = R(DCTL);

		dctl &= ~SFTDISCON;
		W(DCTL, dctl);
	}
	crude_mdelay(50);
	/* Snapshot CSR for postmortem (overwritten each start) */
	writel(R(DSTS), (void __iomem *)0xd900c108UL);
	writel(R(DCTL), (void __iomem *)0xd900c10cUL);
	writel(R(GUSBCFG), (void __iomem *)0xd900c114UL);
	writel(R(GOTGCTL), (void __iomem *)0xd900c118UL);
}


void meson_usb_poll_once(void)
{
	u32 intsts = R(GINTSTS);

	/*
	 * Bulk-hot: only drain RXFIFO + re-arm EP2.  Skipping EP0/EP1 work
	 * during WR_LARGE keeps multi-packet OUT from stalling (P4 rate).
	 * A bus reset leaves this path: it is handled below, where the
	 * transfer is abandoned.
	 */
	if (bulk_total_len > 0 && bulk_recv_len < bulk_total_len &&
	    !(intsts & INT_USBRST)) {
		int n;
		u32 di2;

		if (bulk_dma_mode) {
			/*
			 * DMA: completion via DOEPINT2.XFERCOMPL, never a
			 * RXFIFO pop — the core moves OUT data straight to
			 * DRAM.  Do NOT touch GRXSTSP here: popping a status
			 * the core never pushed is what wedged the earlier
			 * mid-WR_LARGE attempts.
			 */
			(void)n;
			di2 = R(DOEPINT2);
			if (di2 & DOEPINT_XFERCOMPL) {
				u32 left = R(DOEPTSIZ2) & 0x7ffff;
				u32 got = bulk_arm_bytes - left;

				bulk_dma_stall = 0;
				W(DOEPINT2, di2);
				if (got > 0 && got <= bulk_arm_bytes)
					bulk_recv_len += got;
				else
					/* fallback: assume full arm */
					bulk_recv_len += bulk_arm_bytes;
				if (bulk_recv_len > bulk_total_len)
					bulk_recv_len = bulk_total_len;
				if (bulk_recv_len < bulk_total_len)
					ep2_bulk_out_arm();
			} else if (di2) {
				W(DOEPINT2, di2);
			} else if (++bulk_dma_stall > BULK_DMA_STALL_MAX) {
				u1_prog_or(U1P_DMA_STALL);
				bulk_recv_len = bulk_total_len;
			}
			return;
		}

		/*
		 * Bulk-hot fast path: one GRXSTSP pop (no GRXSTSR double-read).
		 * EP2 OUTRX 512 B → DRAM word copy; OUTDONE ignored here and
		 * rearmed below when !EPENA. Rare non-EP2 falls to handle_rxfifo
		 * — but GRXSTSP was already popped, so only call it when the
		 * status was NOT consumed (i.e. never for EP2 paths above).
		 * Non-EP2: re-process via status-driven helper not available —
		 * discard remaining data words and ignore (WR_LARGE is EP2-only
		 * after header; control is idle for bulk duration).
		 */
		for (n = 0; n < 4096 && (R(GINTSTS) & INT_RXFLVL); n++) {
			u32 st = R(GRXSTSP);
			int ep = EPNUM(st);
			int bcnt = BCNT(st);
			int pktsts = PKTSTS(st);

			if (ep == 2 && pktsts == PKTSTS_OUTRX && bcnt == 512 &&
			    !((bulk_dst_addr + bulk_recv_len) & 3)) {
				u32 *p = (u32 *)(uintptr_t)(bulk_dst_addr +
							    bulk_recv_len);
				int i;

				for (i = 0; i < 128; i += 8) {
					p[i + 0] = R(EP2_FIFO);
					p[i + 1] = R(EP2_FIFO);
					p[i + 2] = R(EP2_FIFO);
					p[i + 3] = R(EP2_FIFO);
					p[i + 4] = R(EP2_FIFO);
					p[i + 5] = R(EP2_FIFO);
					p[i + 6] = R(EP2_FIFO);
					p[i + 7] = R(EP2_FIFO);
				}
				bulk_recv_len += 512;
			} else if (ep == 2 && pktsts == PKTSTS_OUTRX &&
				   bcnt > 0) {
				u8 *dst = (u8 *)(uintptr_t)(bulk_dst_addr +
							    bulk_recv_len);

				fifo_read(EP2_FIFO, dst, bcnt);
				bulk_recv_len += bcnt;
			} else if (ep == 2 && pktsts == PKTSTS_OUTDONE) {
				/* rearm below */
			} else if (pktsts == PKTSTS_OUTRX ||
				   pktsts == PKTSTS_SETUPRX) {
				/* drain unexpected payload so FIFO stays sane */
				int w = (bcnt + 3) >> 2;

				while (w-- > 0)
					(void)R(EP0_FIFO + (ep << 12));
			}
		}
		if (bulk_recv_len < bulk_total_len &&
		    !(R(DOEPCTL2) & EPENA))
			ep2_bulk_out_arm();
		return;
	}

	if (intsts & INT_USBRST) {
		W(GINTSTS, INT_USBRST);
		u1_prog_or(U1P_USBRST);
		/*
		 * A host reset in the middle of a WR_LARGE ends that transfer:
		 * the host re-enumerates and sends the request again from its
		 * start.  Left in place, the bulk-hot path above kept the stage
		 * deaf to the reset (no address clear, no EP0 re-arm), the
		 * host's enumeration failed and its hub cut the port, which on
		 * a port-powered board is a power cycle back to the BootROM.
		 */
		if (bulk_total_len > 0 && bulk_recv_len < bulk_total_len) {
			bulk_total_len = 0;
			bulk_recv_len = 0;
			if (R(DOEPCTL2) & EPENA)
				W(DOEPCTL2, R(DOEPCTL2) | SNAK | DOEPCTL_EPDIS);
		}
		dwc2_handle_reset();
	}
	if (intsts & INT_ENUMDONE) {
		W(GINTSTS, INT_ENUMDONE);
		u1_prog_or(U1P_ENUMDONE);
	}
	/*
	 * Drain RXFIFO hard during bulk FIT — host multi-packet URBs need
	 * the device to empty packets before the next microframe batch.
	 */
	{
		int n;
		int lim = (bulk_total_len > 0 &&
			   bulk_recv_len < bulk_total_len) ? 2048 : 64;

		for (n = 0; n < lim && (R(GINTSTS) & INT_RXFLVL); n++)
			handle_rxfifo();
	}

	/*
	 * Endpoint interrupts: W1C latched DOEPINT0 SETUP bits so they do
	 * not stick.  handle_setup() runs only from GRXSTSP SETUPDONE in
	 * handle_rxfifo() — dispatching from this latch is C15.
	 */
	{
		u32 doep = R(DOEPINT0);

		if (doep & (DOEPINT_SETUP | DOEPINT_STUPPKTRCVD)) {
			W(DOEPINT0, doep);
			u1_prog_or(U1P_SETUPDONE);
		} else if (doep & DOEPINT_XFERCOMPL) {
			W(DOEPINT0, DOEPINT_XFERCOMPL);
			if (ep0_state == EP0_STATUS_IN && pending_addr) {
				u32 dcfg = R(DCFG);

				dcfg &= ~(0x7f << 4);
				dcfg |= (u32)pending_addr << 4;
				W(DCFG, dcfg);
				pending_addr = 0;
			}
			if (ep0_state == EP0_STATUS_IN || ep0_state == EP0_DATA_OUT)
				ep0_arm_setup();
		} else if (doep) {
			W(DOEPINT0, doep);
		}
	}
	{
		u32 diep = R(DIEPINT0);

		if (diep & DIEPINT_XFERCOMPL) {
			W(DIEPINT0, DIEPINT_XFERCOMPL);
			u1_prog_or(U1P_IN_XFERCOMPL); /* bit 15; U1P_STOP is bit 26 now */
			if (ep0_state == EP0_STATUS_IN) {
				/*
				 * IN data delivered — arm status OUT ZLP so the
				 * control transfer can complete on the host.
				 */
				W(DOEPTSIZ0, PKTCNT1 | 0);
				SET(DOEPCTL0, EPENA | CNAK);
			}
		} else if (diep) {
			W(DIEPINT0, diep);
		}
	}
	{
		u32 diep1 = R(DIEPINT1);

		if (diep1 & DIEPINT_XFERCOMPL) {
			W(DIEPINT1, DIEPINT_XFERCOMPL);
			if (usb_acm_mode) {
				acm_tx_busy = 0;
				acm_try_tx();
			}
		} else if (diep1) {
			W(DIEPINT1, diep1);
		}
		/* Recover stuck busy if hardware cleared EPENA */
		if (usb_acm_mode && acm_tx_busy &&
		    !(R(DIEPCTL1) & EPENA)) {
			acm_tx_busy = 0;
			acm_try_tx();
		}
	}
}

/*
 * Vendor check_handler / BL1 USB continue — N8 USB return-to-ROM.
 *
 * RE: gxl-blx-standouts.md #1 — path is SMC 0xC0000000 (not WDT).
 * Model: gxl_bl2_usb_deep.c check_handler → ep_info → smc #0.
 * Table base 0xD9040000 | stepping offset from SOC_ID @ 0xD904050C.
 * ep_info lives at fixed SRAM 0xD9001880 (vendor BL2).
 */
#define BL1_HANDLER_TABLE	0xd9040000UL
#define BL1_HANDLER_SOC_ID	0xd904050cUL
#define BL1_ROM_OFF_CONT_DEFAULT	0x4518u
#define BL1_ROM_OFF_CONT_AFE		0x454cu
#define BL1_ROM_OFF_CONT_18CC		0x4e20u
#define BL1_ROM_OFF_CONT_0022		0x4640u
#define BL1_ROM_STEP_AFE		0x94000afeu
#define BL1_ROM_STEP_18CC		0x940018ccu
#define BL1_ROM_STEP_0022		0x52800022u
#define BL1_ROM_EP_SPSR			0x2cdu
#define BL1_SMC_RUN_IMAGE		0xc0000000UL
#define BL1_EP_INFO_ADDR		0xd9001880UL
/* Host-visible stage if SMC returns (should not). 0x5354N8E0 */
#define BL1_N8_FAIL_STAGE		0x5354e800u
#define BL1_N8_FAIL_STAGE_U32		0x5354e800u

/**
 * meson_bl1_usb_return() - Hand USB back to BL1 (vendor check_handler).
 *
 * Builds ep_info {pc = ROM_TABLE|off, spsr = 0x2cd, arg0 = 1} and issues
 * SMC #0 with x0 = 0xC0000000 (Amlogic run-image FID).  Does not return
 * on success: BL1 re-enters USB download (1b8e:c003, identify 02 04).
 *
 * Measured (2026-08-04): freestanding DWC2 re-enum dies; pure spin dies;
 * this SMC path keeps BROM USB so the host can read @SPL_U1_MARKER_ADDR.
 * Not a watchdog reset (standouts #1).
 */
void meson_bl1_usb_return(void)
{
	u32 soc_id = readl((void __iomem *)BL1_HANDLER_SOC_ID);
	u32 off = BL1_ROM_OFF_CONT_DEFAULT;
	u64 entry;
	/* Match vendor ep_info layout at fixed SRAM (bl2 uses 0xd9001880). */
	volatile u64 *ep = (volatile u64 *)BL1_EP_INFO_ADDR;

	if (soc_id == BL1_ROM_STEP_AFE)
		off = BL1_ROM_OFF_CONT_AFE;
	else if (soc_id == BL1_ROM_STEP_18CC)
		off = BL1_ROM_OFF_CONT_18CC;
	else if (soc_id == BL1_ROM_STEP_0022)
		off = BL1_ROM_OFF_CONT_0022;

	entry = BL1_HANDLER_TABLE | off;

	ep[0] = 0;			/* h0 + attrs (clear bit0) */
	ep[1] = entry;			/* pc */
	ep[2] = BL1_ROM_EP_SPSR;	/* spsr in low 32 */
	ep[3] = 1;			/* arg0 */

	/* Make ep_info + I-cache state visible to BL1 before SMC. */
	asm volatile("dsb sy\nisb" ::: "memory");

	asm volatile(
		"mov x0, %0\n"
		"mov x1, %1\n"
		"mov x2, xzr\n"
		"mov x3, xzr\n"
		"mov x4, xzr\n"
		"mov x5, xzr\n"
		"mov x6, xzr\n"
		"mov x7, xzr\n"
		"smc #0\n"
		:
		: "r"((u64)BL1_SMC_RUN_IMAGE), "r"((u64)BL1_EP_INFO_ADDR)
		: "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7",
		  "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
		  "x16", "x17", "memory"
	);

	/*
	 * Error path (should not run): SMC returned — BL1 rejected handoff.
	 * Leave a stage crumb for host BROM readMemory; spin (no WDT reset).
	 */
	writel(0x5354e800u, (void __iomem *)0xd900c120UL);
	for (;;)
		;
}

/**
 * meson_u1_mark_and_return() - Write U1 proof marker then return USB to BL1.
 */
void meson_u1_mark_and_return(void)
{
	writel(SPL_U1_MARKER_VAL, (void __iomem *)SPL_U1_MARKER_ADDR);
	meson_bl1_usb_return();
}

/**
 * meson_usb_reenum_serve() - Pre-DDR worldcup identify (02 22) for U1/U2.
 *
 * Takes over BROM's live DWC2 without PHY POR (worldcup).  Masks DAIF so
 * stray IRQs cannot kill us before VBAR is installed.  On identify success
 * stays on the link.  Returns 0 if identify served, -1 on timeout.
 */
int meson_usb_reenum_serve(unsigned int timeout_ms)
{
	unsigned int t;

	/* Mask IRQ/FIQ/SError — no SPL VBAR yet on U1-only path. */
	asm volatile("msr daifset, #0xf" ::: "memory");

	/*
	 * Make SRAM-written SPL text visible as instructions (BROM bulk
	 * write is DMA-coherent for data; I-cache may still hold old lines).
	 */
	asm volatile(
		"ic iallu\n"
		"dsb sy\n"
		"isb\n"
		::: "memory"
	);

	writel(SPL_U1_MARKER_VAL, (void __iomem *)SPL_U1_MARKER_ADDR);
	writel(U1P_START | U1P_WORLDCUP, (void __iomem *)SPL_U1_PROG_ADDR);

	meson_usb_device_start(0, 0); /* worldcup */	/* no bulk */

	for (t = 0; t < timeout_ms; t++) {
		meson_usb_poll_once();
		if (identify_hits > 0) {
			/*
			 * Host saw 02 22.  Long post-id poll was dying within
			 * ~256 ms (stage stuck 0x5354AA00, then BROM). Finish
			 * EP0 with a short bounded drain, stage BB01, return.
			 * Mode-5 forever-serve remains the long-lived console.
			 */
			writel(0x5354AA01u,
			       (void __iomem *)0xd900c120UL);
			for (t = 0; t < 50; t++) { /* ~50 ms status drain */
				meson_usb_poll_once();
				crude_udelay(1000);
			}
			writel(0x5354BB01u,
			       (void __iomem *)0xd900c120UL);
			return 0;
		}
		crude_udelay(1000);
	}

	return -1;
}

/**
 * meson_usb_reenum_phy_por() - Full PHY POR disconnect + HS re-enum (U2).
 *
 * Design U1 wanted a real host-visible disconnect.  Worldcup (phy_por=0)
 * has not served 02 22 on the PPPS hub; this path forces re-attach.
 *
 * Return: 0 if identify served, -1 on timeout.
 */
int meson_usb_reenum_phy_por(unsigned int timeout_ms)
{
	unsigned int t;

	asm volatile("msr daifset, #0xf" ::: "memory");
	asm volatile("ic iallu\ndsb sy\nisb\n" ::: "memory");
	writel(SPL_U1_MARKER_VAL, (void __iomem *)SPL_U1_MARKER_ADDR);
	u1_prog_or(U1P_PHYPOR);

	meson_usb_device_start(0, 1); /* PHY POR + soft-connect */

	for (t = 0; t < timeout_ms; t++) {
		meson_usb_poll_once();
		if ((t % 1000) == 0) {
			/* host may read via BROM after WD if we never ID */
			writel(R(DSTS), (void __iomem *)0xd900c108UL);
			writel(R(DCTL), (void __iomem *)0xd900c10cUL);
			writel(R(GINTSTS), (void __iomem *)0xd900c110UL);
		}
		if (identify_hits > 0) {
			unsigned int s;

			/* status-OUT + host retries after first 02 22 */
			for (s = 0; s < 8000; s++) {
				meson_usb_poll_once();
				crude_udelay(1000);
			}
			return 0;
		}
		crude_udelay(1000);
	}
	/* leave soft-connected for post-timeout host; park polling 10s more */
	{
		unsigned int s;

		for (s = 0; s < 10000; s++) {
			meson_usb_poll_once();
			crude_udelay(1000);
		}
	}
	return -1;
}

/**
 * meson_u1_try_identify_or_return() - Serve 02 22 worldcup or fall back OMOR.
 *
 * Measured freestanding id-try.S (2026-08-04): host saw 02220020 at +0.01s
 * when the device stayed on the wire after EP0 IN (no immediate ROM-return).
 * On timeout, ROM-return + OMOR keeps the session recoverable.
 */
void meson_u1_try_identify_or_return(unsigned int timeout_ms)
{
	/* Do not cap to 3s — CRT0 + host PPPS latency needs tens of seconds. */
	if (timeout_ms < 1000)
		timeout_ms = 1000;

	if (meson_usb_reenum_serve(timeout_ms) == 0) {
		/* Host-visible 02 22 path: keep serving forever. */
		for (;;)
			meson_usb_poll_once();
	}
	/* No identify hit — recover via BL1 USB + OMOR marker. */
	meson_u1_mark_and_return();
}

/**
 * meson_usb_receive_fit() - Receive FIT image via USB after DDR init.
 *
 * Reinitializes DWC2 as a USB device (Amlogic ROM protocol, VID=1b8e
 * PID=c003), waits for the host to deliver a FIT image via
 * REQ_WR_LARGE_MEM + bulk OUT to CONFIG_SPL_LOAD_FIT_ADDRESS, then
 * returns.  The host tool (meson-usb-boot.py) signals completion with
 * REQ_RUN_IN_ADDR.
 */
/**
 * meson_usb_acm_serve() - Identify then vendor bulk console (OTG “UART”).
 *
 * Soft SFTDISCON→CDC re-enum never reattached (1b8e gone; 2026-08-04). Until
 * that is fixed, stay on vendor bulk EP1 IN/OUT: host bulk-reads banner and
 * can bulk-write for getc. cdc_acm/ttyACM remains follow-up.
 */
void meson_usb_acm_serve(unsigned int id_timeout_ms)
{
	unsigned int i;

	asm volatile("msr daifset, #0xf" ::: "memory");
	asm volatile("ic iallu\ndsb sy\nisb\n" ::: "memory");
	writel(SPL_U1_MARKER_VAL, (void __iomem *)SPL_U1_MARKER_ADDR);
	writel(U1P_START | U1P_WORLDCUP, (void __iomem *)SPL_U1_PROG_ADDR);
	meson_usb_device_start(0, 0);
	usb_acm_mode = 1;
	acm_cdc_live = 0;
	for (i = 0; i < id_timeout_ms; i++) {
		meson_usb_poll_once();
		if (identify_hits > 0)
			break;
		crude_udelay(1000);
	}
	if (identify_hits > 0) {
		for (i = 0; i < 50; i++) {
			meson_usb_poll_once();
			if (ep0_state == EP0_IDLE)
				break;
			crude_udelay(1000);
		}
		acm_arm_bulk_in();
		meson_usb_acm_puts("GXL-SPL\n");
	}
	{
		unsigned int n = 0;

		for (;;) {
			meson_usb_poll_once();
			if (acm_configured) {
				acm_try_tx();
				/* re-nudge banner every ~2 s */
				if ((n++ % 2000) == 0)
					meson_usb_acm_puts("GXL-SPL\n");
			}
			crude_udelay(1000);
		}
	}
}

/**
 * meson_usb_soft_disconnect() - SFTDISCON after early identify, before SMC.
 * Leaving worldcup live into ROM SMC rebooted to BROM (~8 s, 2026-08-04).
 */
void meson_usb_soft_disconnect(void)
{
	u32 dctl = R(DCTL);

	W(DCTL, dctl | SFTDISCON);
	crude_mdelay(10);
}

/**
 * meson_usb_device_stop() - hand the DWC2 back in a state the next stage owns.
 *
 * Returning from receive_fit() used to jump into the FIT payload with the core
 * exactly as the bulk session left it: still soft-connected at its assigned
 * address, EP2 possibly still armed against a DOEPTSIZ2/DOEPDMA2 aimed at DRAM
 * the payload now owns, every EP/global interrupt unmasked, GAHBCFG.DMA_EN
 * sticky if the DMA path ran, and a non-BROM FIFO map.  The host kept a live
 * 1b8e handle straight across the handoff, which is what "sticky BROM state"
 * looks like from the outside.
 *
 * Order is the databook's, and the same one the DMA work had to learn: mask
 * first, global OUT NAK before touching an OUT endpoint, then disable, flush,
 * restore, and only then drop the pull-up so the host sees a real disconnect
 * rather than a device that stops answering.
 */
static void dwc2_hw_stop(void);

static void meson_usb_device_stop(void)
{
	/* The session's LAST transfer has no successor to close it out. */
	bulk_note_spl();
	dwc2_hw_stop();
	u1_prog_or(U1P_STOP);
}

/**
 * meson_usb_disconnect_for_reset() - quiesce the DWC2 before a stage resets
 * the chip, when no session of ours ran.
 *
 * The TPL's DRAM-init failure stop ("DF") is reached with the controller
 * exactly as the BootROM left it when it jumped -- soft-connected at its
 * address, mid-way through the host's REQ_RUN -- and then the watchdog
 * resets the chip.  The BootROM after that warm reset stalled on the host's
 * first write in 5 of 6 re-stages, while a watchdog reset reached with the
 * DWC2 stopped by meson_usb_device_stop() (NOSPL) re-staged 4 of 4 (GXL DDR3
 * FMA round F3, claude amlogic/software/boot/gxl/audit/
 * 2026-10-19-gxl-ddr3-boot-chain-failure-modes.md).  The hardware half of
 * device_stop() only: no session bookkeeping, since none ran.
 */
void meson_usb_disconnect_for_reset(void)
{
	dwc2_hw_stop();
}

static void dwc2_hw_stop(void)
{
	int i;

	W(GINTMSK, 0);
	W(DOEPMSK, 0);
	W(DIEPMSK, 0);
	W(DAINTMSK, 0);

	bulk_dma_quiesce();
	for (i = 1; i <= 2; i++) {
		u32 ctl = DOEPCTL0 + i * 0x20;

		if (R(ctl) & EPENA)
			W(ctl, R(ctl) | SNAK | DOEPCTL_EPDIS);
	}
	SET(DCTL, DCTL_CGOUTNAK);

	for (i = 1; i <= 2; i++) {
		u32 ctl = DIEPCTL0 + i * 0x20;

		if (R(ctl) & EPENA)
			W(ctl, R(ctl) | SNAK | DOEPCTL_EPDIS);
	}

	dwc2_flush_fifos();
	if (fifo_saved_magic == FIFO_SAVED_MAGIC) {
		W(GRXFSIZ, saved_grxfsiz);
		W(GNPTXFSIZ, saved_gnptxfsiz);
		W(DIEPTXF1, saved_dieptxf1);
	}
	W(GAHBCFG, BIT(0));		/* drop DMA_EN if the DMA path ran */
	CLR(DCFG, 0x7f << 4);		/* address back to 0 */
	SET(DCTL, SFTDISCON);		/* real disconnect, not a silent device */
	crude_mdelay(5);
}

/* Asm bulk spin rearm trampoline (ep2_bulk_out_arm is static). */
void meson_bulk_rearm(void)
{
	ep2_bulk_out_arm();
}

void meson_bulk_spin_pio(u32 *dst, u32 *total, u32 *recv);

void meson_usb_receive_fit(void)
{
	/*
	 * Worldcup + allow_bulk + multi-pkt EP2 + host --urb.
	 * Spin bulk-hot poll during WR_LARGE for HS bulk rate.
	 */
	asm volatile("msr daifset, #0xf" ::: "memory");
	asm volatile("ic iallu\ndsb sy\nisb\n" ::: "memory");
	writel(SPL_U1_MARKER_VAL, (void __iomem *)SPL_U1_MARKER_ADDR);
	writel(U1P_START | U1P_WORLDCUP, (void __iomem *)SPL_U1_PROG_ADDR);
	{
		u32 req;

		fifo_done = 0;

		/*
		 * Default is the NO-OP (rung 0, BROM 256-word map) — see
		 * FIFO_RUNG_DEFAULT.  'FIF'<1..6> explicitly grows RX; 512
		 * words takes the bulk write from 23.2 to 37.4 MiB/s.
		 */
		req = readl((void __iomem *)FIFO_REQ_ADDR);
		fifo_want = ((req & 0x00ffffffu) == FIFO_REQ_MAGIC) ?
			    (int)((req >> 24) - '0') : FIFO_RUNG_DEFAULT;
		if (fifo_want < 0 || fifo_want > FIFO_RUNG_MAX)
			fifo_want = FIFO_RUNG_DEFAULT;

		/*
		 * Echo the RESOLVED rung back into the same word, so which arm
		 * ran is a fact the host can READ (readSimpleMemory over EP0,
		 * which works before the bulk) rather than one it assumes from
		 * the results.  A host that asked for rung N and reads back
		 * anything else knows its write did not take, at the time.
		 * Idempotent: re-reading this word resolves to the same rung.
		 */
		writel(FIFO_REQ_MAGIC | ((u32)('0' + fifo_want) << 24),
		       (void __iomem *)FIFO_REQ_ADDR);
	}
#if USB_REENUM
	/*
	 * Re-enumerate as our own device: a 10 ms host-visible disconnect,
	 * then the PHY-POR start, and straight into the receive loop -- the
	 * host starts pass 2 as soon as identify answers, so bulk must be
	 * served at full rate from here (a slower poll overran the RX FIFO,
	 * E5).  The host follows the device by its port.
	 */
	reenum_fifo[0] = R(GRXFSIZ);
	reenum_fifo[1] = R(GNPTXFSIZ);
	reenum_fifo[2] = R(DIEPTXF1);
	reenum_fifo[3] = R(DIEPTXF2);
	reenum_fifo_valid = 1;
	SET(DCTL, SFTDISCON);
	crude_mdelay(10);
	meson_usb_device_start(1, 1);
#else
	meson_usb_device_start(1, 0);
#endif

	while (!usb_done) {
		if (bulk_total_len > 0 && bulk_recv_len < bulk_total_len) {
			if (bulk_dma_mode) {
				while (bulk_recv_len < bulk_total_len)
					meson_usb_poll_once();
				/* Payload done — EP0 now needs a DMA arm. */
				W(DOEPINT0, R(DOEPINT0));
				W(DIEPINT0, R(DIEPINT0));
				ep0_arm_setup();
				continue;
			}
			meson_bulk_spin_pio(&bulk_dst_addr, &bulk_total_len,
					    &bulk_recv_len);
			/* Short only on a bus reset: poll_once abandons the transfer. */
			if (bulk_recv_len < bulk_total_len)
				meson_usb_poll_once();
			continue;
		}
		meson_usb_poll_once();
	}
	meson_usb_device_stop();
}

/**
 * meson_usb_last_payload() - what the finished session actually delivered.
 * @addr:	destination of the last REQ_WR_LARGE_MEM, or 0
 * @len:	its length in bytes, or 0
 * @run:	address from REQ_RUN_IN_ADDR, or 0
 *
 * The transfer state is file-static and reset per session; this is the one
 * reader outside it, so the TPL can record in the handoff block what arrived
 * instead of the next stage having to rediscover it.  Last transfer, not a
 * total: the closed-loop session pushes one image.
 */
void meson_usb_last_payload(u32 *addr, u32 *len, u32 *run)
{
	if (addr)
		*addr = bulk_dst_addr;
	if (len)
		*len = bulk_recv_len;
	if (run)
		*run = usb_run_addr;
}
