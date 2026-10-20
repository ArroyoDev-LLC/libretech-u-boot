// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic G12A/SM1 TPL: register-level eMMC reader, READ ONLY.
 *
 * The SRAM stage has a 61440 B ceiling and the driver-model reader
 * (TPL_DM + OF_CONTROL + MMC/BLK + meson_gx_mmc) overflows it by 8208 B on
 * the shipping AML-S905D3-CC TPL.  This file is the other shape: no driver
 * model, no DTB, no block layer.  It drives SD_EMMC_C through its registers
 * and moves each 512 B block through the controller's own buffer by CPU,
 * which is how the SM1 BootROM reads eMMC (bl1.bin sha256 94322c13...,
 * FUN_ffff4a6c: CMD17 per block with the data address at base+0x400 /
 * base+0x600, then a CPU copy).  It never asks the controller to DMA to
 * memory, so it works with no DRAM and does not depend on whether the
 * controller can master AHB SRAM.
 *
 * WHAT IT REUSES FROM THE BOOTROM (read, never written here): the BOOT pad
 * mux (PERIPHS_PIN_MUX_0/1), the pull-ups the ROM sets on those pads, and the
 * SD_EMMC_C clock gate.  The ROM programs all of them on every power-up
 * before it scans eMMC (FUN_ffff51b0 for base 0xFFE07000), and on this unit
 * it scans eMMC on every boot (POC:F), USB boots included.
 *
 * WHAT IT RE-INITIALISES: the controller's CLOCK and CFG words (written with
 * the ROM's own values, FUN_ffff5370), and the card: CMD0 / CMD1 / CMD2 /
 * CMD3 (RCA 1, the ROM's) / CMD7, the ROM's sequence (FUN_ffff4ea8), in
 * 1-bit mode.  The ROM leaves the card selected with PARTITION_ACCESS on
 * boot1 (its last CMD6 writes 0x12); CMD0 returns the access field to the
 * user area, so user reads need no CMD6 at all.
 *
 * WHAT IT CANNOT DO: change the medium.  No write command (CMD24/25), no
 * erase, and no CMD6 that names a non-volatile EXT_CSD field.  Selecting a
 * boot area is CMD6 in SET_BITS / CLEAR_BITS mode on PARTITION_CONFIG
 * (EXT_CSD 179) with a mask of bits [2:0] only -- PARTITION_ACCESS, a
 * volatile field that CMD0 and power loss clear -- so BOOT_PARTITION_ENABLE
 * (bits [5:3], non-volatile) and BOOT_ACK are not written at all.  (The ROM
 * itself uses WRITE_BYTE p|p<<3, which does rewrite BOOT_PARTITION_ENABLE,
 * three times per power-up; this reader does less than that.)
 *
 * Register offsets and fields: vendor BSP A311D_Linux2603V2.5 u-boot bl33,
 * cited per line below by path from bl33/ (the BSP ships dozens of
 * disagreeing copies of each header); addresses resolved with
 * amlogic/software/tools/bsp-reverse-index.py.
 *
 * Plan: amlogic/software/boot/g12/plan/sm1-resident-chain.md (step 4).
 */

#include <mmc.h>
#include <stdio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/types.h>

/* SD_EMMC_BASE_C, v2019/arch/arm/include/asm/arch-g12a/cpu_sdio.h:11: the eMMC port the ROM scans */
#define G12E_BASE		0xFFE07000UL

/* MESON_SD_EMMC_CLOCK, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:18 */
#define G12E_CLOCK		0x00
#define   G12E_CLK_DIV(n)	((n) << 0)	/* Cfg_div [5:0], v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:21 */
#define   G12E_CLK_SRC_XTAL	(0 << 6)	/* Cfg_src [7:6] = 0: 24 MHz, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:15,22 */
#define   G12E_CLK_CO_180	(2 << 8)	/* Cfg_co_phase [9:8], v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:23 */
#define   G12E_CLK_ALWAYS_ON	BIT(28)		/* Cfg_always_on, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:29 */
/* MESON_SD_EMMC_CFG, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:74 */
#define G12E_CFG		0x44
#define   G12E_CFG_BUS_1	(0 << 0)	/* CFG_BUS_WIDTH_1, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:76 */
#define   G12E_CFG_BL_LEN_512	(9 << 4)	/* CFG_BL_LEN_512, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:82 */
#define   G12E_CFG_RESP_TO(n)	((n) << 8)	/* CFG_RESP_TIMEOUT [11:8], v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:83 */
#define   G12E_CFG_RC_CC(n)	((n) << 12)	/* CFG_RC_CC [15:12], v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:85 */
/* MESON_SD_EMMC_STATUS, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:90 (write-one-to-clear) */
#define G12E_STATUS		0x48
#define   G12E_ST_MASK		GENMASK(15, 0)	/* STATUS_MASK, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:91 */
#define   G12E_ST_ERR		GENMASK(12, 0)	/* STATUS_ERR_MASK, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:92 */
#define   G12E_ST_EOC		BIT(13)		/* STATUS_END_OF_CHAIN, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:99 */
/* MESON_SD_EMMC_IRQ_EN, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:101 */
#define G12E_IRQ_EN		0x4c
/* MESON_SD_EMMC_CMD_CFG, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:103 */
#define G12E_CMD_CFG		0x50
#define   G12E_CC_LEN(n)	((n) << 0)	/* CMD_CFG_LENGTH [8:0], v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:104 */
#define   G12E_CC_BLOCK_MODE	BIT(9)		/* v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:105 */
#define   G12E_CC_R1B		BIT(10)		/* v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:106 */
#define   G12E_CC_EOC		BIT(11)		/* CMD_CFG_END_OF_CHAIN, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:107 */
#define   G12E_CC_TIMEOUT_4S	(12 << 12)	/* CMD_CFG_TIMEOUT_4S, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:108 */
#define   G12E_CC_NO_RESP	BIT(16)		/* v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:109 */
#define   G12E_CC_DATA_IO	BIT(18)		/* v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:111 */
#define   G12E_CC_RESP_NOCRC	BIT(20)		/* v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:113 */
#define   G12E_CC_RESP_128	BIT(21)		/* v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:114 */
#define   G12E_CC_IDX(n)	((n) << 24)	/* CMD_CFG_CMD_INDEX_SHIFT, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:118 */
#define   G12E_CC_OWNER		BIT(31)		/* CMD_CFG_OWNER, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:120 */
#define G12E_CMD_ARG		0x54		/* MESON_SD_EMMC_CMD_ARG, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:122 */
#define G12E_CMD_DAT		0x58		/* MESON_SD_EMMC_CMD_DAT, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:123 */
#define G12E_CMD_RSP		0x5c		/* MESON_SD_EMMC_CMD_RSP, v2019/arch/arm/include/asm/arch-g12a/sd_emmc.h:124 */
/*
 * The controller's internal buffer.  No vendor header names it; the address
 * is the ROM's: FUN_ffff4a6c writes base+0x400 (and base+0x600) to CMD_DAT
 * and copies 512 B out of it after each CMD17.
 */
#define G12E_BUF		0x400

/* Reported only, never written here. */
/* PERIPHS_PIN_MUX_0, v2019/arch/arm/mach-meson/g12a/firmware/scp_task/secure_apb.h:580 */
#define G12E_PIN_MUX_0		0xFF6346C0UL
/* HHI_GCLK_MPEG0, v2019/arch/arm/mach-meson/g12a/firmware/scp_task/secure_apb.h:2197 */
#define G12E_GCLK_MPEG0		0xFF63C140UL
/* P_ISA_TIMERE, v2019/arch/arm/include/asm/arch-g12a/regs.h:13360: the 1 MHz counter dram-g12a.c polls */
#define G12E_TIMERE		0xFFD0F188UL

/*
 * The ROM's clock and config words (FUN_ffff5370): identification at
 * 24 MHz / 60 = 400 kHz, data at 24 MHz / 4 = 6 MHz, both 180-degree core
 * phase with the clock always on; CFG 1-bit, 512 B blocks, response timeout
 * field 7, command gap field 4.
 */
#define G12E_CLK_ID	(G12E_CLK_ALWAYS_ON | G12E_CLK_CO_180 | \
			 G12E_CLK_SRC_XTAL | G12E_CLK_DIV(60))
#define G12E_CLK_DATA	(G12E_CLK_ALWAYS_ON | G12E_CLK_CO_180 | \
			 G12E_CLK_SRC_XTAL | G12E_CLK_DIV(4))
#define G12E_CFG_WORD	(G12E_CFG_RC_CC(4) | G12E_CFG_RESP_TO(7) | \
			 G12E_CFG_BL_LEN_512 | G12E_CFG_BUS_1)

/* The ROM's R1 error mask (0xffff4ae0, tested at 0xffff4b3c). */
#define G12E_R1_ERR		0xfcf80000u
/* The ROM's CMD1 argument (0xffff4ef8): sector mode + 2.7-3.6 V window. */
#define G12E_OCR_ARG		0x40ff8000u
/* The ROM's relative card address (0xffff4fd8). */
#define G12E_RCA		1u
/* Every wait is bounded well under the TPL's 8 s watchdog. */
#define G12E_CMD_US		1000000u
#define G12E_OCR_US		1000000u

static u32 g12e_sector_mode;	/* OCR bit 30: LBA arguments, not bytes */

static inline u32 g12e_us(void)
{
	return readl((void __iomem *)G12E_TIMERE);
}

static inline void __iomem *g12e_reg(u32 off)
{
	return (void __iomem *)(G12E_BASE + off);
}

/*
 * One command.  Returns 0, or the STATUS word with bit 31 set on a timeout of
 * our own poll (STATUS never sets bit 31).  *rsp gets CMD_RSP either way.
 */
static u32 g12e_cmd(u32 idx, u32 flags, u32 arg, u32 *rsp)
{
	u32 st, t0;

	/* STATUS: clear every latched bit [15:0] (W1C) before the command. */
	writel(G12E_ST_MASK, g12e_reg(G12E_STATUS));
	/* CMD_CFG: index, response/data shape, 4 s controller timeout,
	 * single-command chain owned by the controller (mainline layout). */
	writel(G12E_CC_IDX(idx) | flags | G12E_CC_TIMEOUT_4S | G12E_CC_OWNER |
	       G12E_CC_EOC, g12e_reg(G12E_CMD_CFG));
	/* CMD_DAT: the controller's own buffer for a data command, else 0. */
	writel((flags & G12E_CC_DATA_IO) ? G12E_BASE + G12E_BUF : 0,
	       g12e_reg(G12E_CMD_DAT));
	/* CMD_ARG: the argument; this write starts the command. */
	writel(arg, g12e_reg(G12E_CMD_ARG));

	t0 = g12e_us();
	do {
		st = readl(g12e_reg(G12E_STATUS));
	} while (!(st & G12E_ST_EOC) && g12e_us() - t0 < G12E_CMD_US);

	*rsp = readl(g12e_reg(G12E_CMD_RSP));
	/* STATUS: clear what this command latched. */
	writel(G12E_ST_MASK, g12e_reg(G12E_STATUS));

	if (!(st & G12E_ST_EOC))
		return st | BIT(31);
	return st & G12E_ST_ERR;
}

static u32 g12e_crc32(u32 crc, u32 w)
{
	int k;

	crc ^= w;	/* four bytes, little-endian, as they sit on the card */
	for (k = 0; k < 32; k++)
		crc = (crc >> 1) ^ (0xedb88320u & -(crc & 1));
	return crc;
}

static int g12e_fail(u32 idx, u32 st, u32 rsp)
{
	printf("EM: FAIL cmd%u st %08x rsp %08x\n", idx, st, rsp);
	return -1;
}

/*
 * Identify and select the card, then raise the clock.  Leaves the card in
 * transfer state on the user area.
 */
static int g12e_init(void)
{
	u32 st, rsp, t0;

	/* IRQ_EN: none; this reader polls. */
	writel(0, g12e_reg(G12E_IRQ_EN));
	/* CLOCK: 400 kHz identification clock, the ROM's word. */
	writel(G12E_CLK_ID, g12e_reg(G12E_CLOCK));
	/* CFG: 1-bit, 512 B blocks, the ROM's timeouts. */
	writel(G12E_CFG_WORD, g12e_reg(G12E_CFG));

	st = g12e_cmd(MMC_CMD_GO_IDLE_STATE, G12E_CC_NO_RESP, 0, &rsp);
	if (st)
		return g12e_fail(MMC_CMD_GO_IDLE_STATE, st, rsp);

	t0 = g12e_us();
	do {
		st = g12e_cmd(MMC_CMD_SEND_OP_COND, G12E_CC_RESP_NOCRC,
			      G12E_OCR_ARG, &rsp);
		if (st)
			return g12e_fail(MMC_CMD_SEND_OP_COND, st, rsp);
	} while (!(rsp & OCR_BUSY) && g12e_us() - t0 < G12E_OCR_US);
	if (!(rsp & OCR_BUSY))
		return g12e_fail(MMC_CMD_SEND_OP_COND, 0, rsp);
	g12e_sector_mode = rsp & OCR_HCS;
	printf("EM: ocr %08x\n", rsp);

	st = g12e_cmd(MMC_CMD_ALL_SEND_CID, G12E_CC_RESP_128, 0, &rsp);
	if (st)
		return g12e_fail(MMC_CMD_ALL_SEND_CID, st, rsp);
	st = g12e_cmd(MMC_CMD_SET_RELATIVE_ADDR, 0, G12E_RCA << 16, &rsp);
	if (st)
		return g12e_fail(MMC_CMD_SET_RELATIVE_ADDR, st, rsp);
	st = g12e_cmd(MMC_CMD_SELECT_CARD, G12E_CC_R1B, G12E_RCA << 16, &rsp);
	if (st || (rsp & G12E_R1_ERR))
		return g12e_fail(MMC_CMD_SELECT_CARD, st, rsp);

	/* CLOCK: 6 MHz data clock, the ROM's word. */
	writel(G12E_CLK_DATA, g12e_reg(G12E_CLOCK));
	return 0;
}

/*
 * Read @count blocks from @lba of the selected partition.  Each block lands
 * in the controller buffer and is moved by CPU to @dst (if not NULL) and
 * folded into *@crc (if not NULL; caller seeds ~0 and finalises with ~).
 */
static int g12e_read(u32 lba, u32 count, u32 *dst, u32 *crc)
{
	const u32 __iomem *buf = g12e_reg(G12E_BUF);
	u32 st, rsp, i, w;

	for (; count; count--, lba++) {
		st = g12e_cmd(MMC_CMD_READ_SINGLE_BLOCK,
			      G12E_CC_DATA_IO | G12E_CC_BLOCK_MODE |
			      G12E_CC_LEN(1),
			      g12e_sector_mode ? lba : lba << 9, &rsp);
		if (st || (rsp & G12E_R1_ERR))
			return g12e_fail(MMC_CMD_READ_SINGLE_BLOCK, st, rsp);
		for (i = 0; i < 128; i++) {
			w = readl(buf + i);
			if (dst)
				*dst++ = w;
			if (crc)
				*crc = g12e_crc32(*crc, w);
		}
	}
	return 0;
}

/*
 * Select a hardware partition (0 user, 1 boot0, 2 boot1) by PARTITION_ACCESS
 * only: CLEAR_BITS 0x07, then SET_BITS @part.  Confirms with CMD13.
 */
static int g12e_access(u32 part)
{
	u32 st, rsp;

	st = g12e_cmd(MMC_CMD_SWITCH, G12E_CC_R1B,
		      (MMC_SWITCH_MODE_CLEAR_BITS << 24) |
		      (EXT_CSD_PART_CONF << 16) | (0x07 << 8), &rsp);
	if (st || (rsp & G12E_R1_ERR))
		return g12e_fail(MMC_CMD_SWITCH, st, rsp);
	if (part) {
		st = g12e_cmd(MMC_CMD_SWITCH, G12E_CC_R1B,
			      (MMC_SWITCH_MODE_SET_BITS << 24) |
			      (EXT_CSD_PART_CONF << 16) | (part << 8), &rsp);
		if (st || (rsp & G12E_R1_ERR))
			return g12e_fail(MMC_CMD_SWITCH, st, rsp);
	}
	st = g12e_cmd(MMC_CMD_SEND_STATUS, 0, G12E_RCA << 16, &rsp);
	if (st || (rsp & (G12E_R1_ERR | MMC_STATUS_SWITCH_ERROR)) ||
	    (rsp & MMC_STATUS_CURR_STATE) != MMC_STATE_TRANS)
		return g12e_fail(MMC_CMD_SEND_STATUS, st, rsp);
	return 0;
}

/**
 * g12_tpl_emmc_selftest() - read three known regions and print their crc32
 *
 * user sector 0, user LBA 1..128 (the 64 KiB the BootROM reads), boot0
 * sector 0; then the user area again.  zlib CRC-32, the value U-Boot's
 * `crc32` command prints for the same bytes.  Never fatal.
 */
void g12_tpl_emmc_selftest(void)
{
	u32 t0, t1, t2, crc;

	printf("EM: mux0 %08x gclk0 %08x clk %08x cfg %08x\n",
	       readl((void __iomem *)G12E_PIN_MUX_0),
	       readl((void __iomem *)G12E_GCLK_MPEG0),
	       readl(g12e_reg(G12E_CLOCK)), readl(g12e_reg(G12E_CFG)));

	t0 = g12e_us();
	if (g12e_init())
		return;
	t1 = g12e_us();

	crc = ~0u;
	if (g12e_read(0, 1, NULL, &crc))
		return;
	printf("EM: user s0 crc32 %08x\n", ~crc);

	t2 = g12e_us();
	crc = ~0u;
	if (g12e_read(1, 128, NULL, &crc))
		return;
	printf("EM: user lba1+128 crc32 %08x us %u\n", ~crc, g12e_us() - t2);

	if (g12e_access(1))
		return;
	crc = ~0u;
	if (g12e_read(0, 1, NULL, &crc))
		return;
	printf("EM: boot0 s0 crc32 %08x\n", ~crc);
	if (g12e_access(0))
		return;
	printf("EM: OK init us %u total us %u\n", t1 - t0, g12e_us() - t0);
}

#if CONFIG_IS_ENABLED(MESON_G12A_EMMC_FETCH)
/*
 * Fetch the two residual windows from a staged layout (resident-chain step 5).
 *
 * The layout header is one 512 B block written by
 * amlogic/software/tools/mk-g12a-emmc-stage.py, little-endian u32 words:
 *   [0] magic 'G12L'  [1] version 1  [2] hardware partition  [3] header LBA
 *   [4] w1 LBA  [5] w1 length  [6] w1 crc32
 *   [7] w2 LBA  [8] w2 length  [9] w2 crc32  [10] w2 split (SPL | FIT)
 *   [11] crc32 of words [0..10]
 * Window crcs are zlib CRC-32 over whole blocks, ceil(length / 512) of them
 * (the builder zero-pads each window to a block).  Every offset and length
 * the TPL uses comes from this header; the only constants are where the
 * header is (Kconfig) and the window-2 split, which the header must repeat.
 */
#define G12E_LAYOUT_MAGIC	0x4c323147u	/* "G12L" */
#define G12E_LAYOUT_VERSION	1u
#define G12E_LAYOUT_WORDS	12

void meson_g12_residual_feed_begin(u32 total);
void meson_g12_residual_feed(u32 off, const u8 *src, int n);
void meson_g12_residual_feed_end(void);
void meson_g12_residual_set_spl_len(u32 len);
u32 meson_g12_residual_spl_fit_split(void);
u32 meson_g12_residual_dest_spl(void);
u32 meson_g12_residual_dest_fit(void);

static u32 g12e_hdr[G12E_LAYOUT_WORDS];
static int g12e_hdr_ok;
/* One block between the controller buffer and the splitter (BSS, sized at link). */
static u32 g12e_blk[128];

static u32 g12e_blocks(u32 len)
{
	return (len + 511) / 512;
}

static int g12e_check_crc(const char *w, u32 crc, u32 want)
{
	if (crc == want)
		return 0;
	printf("EF: %s crc32 %08x want %08x FAIL\n", w, crc, want);
	return -1;
}

/**
 * g12_tpl_emmc_fetch_w1() - window 1 from the eMMC, before dram_init().
 *
 * Initialises the card, selects the staged partition, reads and checks the
 * header, then reads the residual concat one block at a time through the
 * controller buffer into the USB splitter.  Return 0, or -1 (printed).
 */
int g12_tpl_emmc_fetch_w1(void)
{
	u32 *blk = g12e_blk;
	u32 t0, crc, off, n, i, len;

	t0 = g12e_us();
	if (g12e_init())
		return -1;
	if (CONFIG_TPL_MESON_G12A_EMMC_FETCH_PART &&
	    g12e_access(CONFIG_TPL_MESON_G12A_EMMC_FETCH_PART))
		return -1;

	if (g12e_read(CONFIG_TPL_MESON_G12A_EMMC_FETCH_LBA, 1, blk, NULL))
		return -1;
	for (i = 0; i < G12E_LAYOUT_WORDS; i++)
		g12e_hdr[i] = blk[i];
	crc = ~0u;
	for (i = 0; i < G12E_LAYOUT_WORDS - 1; i++)
		crc = g12e_crc32(crc, g12e_hdr[i]);
	printf("EF: hdr p%u lba %x magic %08x v%u w1 %x+%x %08x w2 %x+%x %08x split %x\n",
	       CONFIG_TPL_MESON_G12A_EMMC_FETCH_PART,
	       CONFIG_TPL_MESON_G12A_EMMC_FETCH_LBA, g12e_hdr[0], g12e_hdr[1],
	       g12e_hdr[4], g12e_hdr[5], g12e_hdr[6], g12e_hdr[7], g12e_hdr[8],
	       g12e_hdr[9], g12e_hdr[10]);
	if (g12e_hdr[0] != G12E_LAYOUT_MAGIC ||
	    g12e_hdr[1] != G12E_LAYOUT_VERSION ||
	    g12e_hdr[2] != CONFIG_TPL_MESON_G12A_EMMC_FETCH_PART ||
	    g12e_hdr[3] != CONFIG_TPL_MESON_G12A_EMMC_FETCH_LBA ||
	    g12e_hdr[10] != meson_g12_residual_spl_fit_split() ||
	    g12e_hdr[8] <= g12e_hdr[10] ||
	    g12e_check_crc("hdr", ~crc, g12e_hdr[11])) {
		printf("EF: hdr FAIL\n");
		return -1;
	}
	g12e_hdr_ok = 1;

	len = g12e_hdr[5];
	meson_g12_residual_feed_begin(len);
	crc = ~0u;
	for (off = 0, i = 0; i < g12e_blocks(len); i++, off += 512) {
		if (g12e_read(g12e_hdr[4] + i, 1, blk, &crc))
			return -1;
		n = len - off < 512 ? len - off : 512;
		meson_g12_residual_feed(off, (const u8 *)blk, n);
	}
	meson_g12_residual_feed_end();
	printf("EF: w1 %u B crc32 %08x us %u\n", len, ~crc, g12e_us() - t0);
	return g12e_check_crc("w1", ~crc, g12e_hdr[6]);
}

/**
 * g12_tpl_emmc_fetch_w2() - window 2 from the eMMC, after dram_init().
 *
 * [0, split) to the DRAM stage address, the rest to the boot FIT address,
 * straight out of the controller buffer.  On a good crc the length is handed
 * to the jump as the USB window would; on any failure it is not, so the TPL
 * stops at NOSPL.  Leaves the card on the user area.
 */
int g12_tpl_emmc_fetch_w2(void)
{
	u32 t0, crc, nb, split_blk;
	u32 len = g12e_hdr[8];

	if (!g12e_hdr_ok)
		return -1;
	t0 = g12e_us();
	nb = g12e_blocks(len);
	split_blk = g12e_hdr[10] / 512;
	crc = ~0u;
	if (g12e_read(g12e_hdr[7], split_blk,
		      (u32 *)(uintptr_t)meson_g12_residual_dest_spl(), &crc) ||
	    g12e_read(g12e_hdr[7] + split_blk, nb - split_blk,
		      (u32 *)(uintptr_t)meson_g12_residual_dest_fit(), &crc))
		return -1;
	printf("EF: w2 %u B crc32 %08x us %u\n", len, ~crc, g12e_us() - t0);
	if (CONFIG_TPL_MESON_G12A_EMMC_FETCH_PART && g12e_access(0))
		return -1;
	if (g12e_check_crc("w2", ~crc, g12e_hdr[9]))
		return -1;
	meson_g12_residual_set_spl_len(len);
	return 0;
}
#endif
