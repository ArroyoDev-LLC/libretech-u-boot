// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic SPIFC NOR read backend for NOR overlay loader
 *
 * Bare-metal SPIFC register access for pre-DM NOR reads.
 * BL2 has already configured SPIFC clocks and pinmux for NOR boot.
 *
 * Provides board_nor_init() and board_nor_read() called by board-config.c.
 *
 * (C) Copyright 2025 Da Xue <da@libre.computer>
 */

#include <linux/types.h>
#include <asm/io.h>
#include <asm/arch/usbflash-alts.h>
#include <string.h>

/*
 * SoC-specific register bases.
 * GXL: CBUS @ 0xc1100000, HIU @ 0xc883c000
 * G12/SM1: CBUS @ 0xffd00000, HIU @ 0xff63c000
 */
#ifdef CONFIG_MESON_GXL
#define SPIFC_BASE		0xc1108c80
#else /* G12A / G12B / SM1 */
#define SPIFC_BASE		0xffd14000
#endif

#define REG_CMD			0x00
#define REG_CTRL		0x08
#define REG_CLOCK		0x18
#define REG_USER		0x1c
#define REG_USER1		0x20
#define REG_USER4		0x2c
#define REG_SLAVE		0x30
#define REG_C0			0x40

#define CMD_USER		BIT(18)
#define CTRL_ENABLE_AHB		BIT(17)
#define USER_DIN_EN_MS		BIT(0)
#define USER_CMP_MODE		BIT(2)
#define USER_UC_DOUT_SEL	BIT(27)
#define USER_UC_MASK		((BIT(5) - 1) << 27)
#define USER4_CS_ACT		BIT(30)
#define SLAVE_TRST_DONE		BIT(4)
#define SLAVE_OP_MODE		BIT(30)
#define SLAVE_SW_RST		BIT(31)

#define SPIFC_BUF_SIZE		64
#define SPIFC_TIMEOUT_US	100000	/* 100ms per chunk */

/* HHI clock gate register -- SPIFC clock is bit 30 of GCLK_MPEG0 */
#ifdef CONFIG_MESON_GXL
#define HIU_BASE		0xc883c000
#else
#define HIU_BASE		0xff63c000
#endif
#define HHI_GCLK_MPEG0		(HIU_BASE + 0x140)

/* SPI NOR commands */
#define SPI_NOR_CMD_READ	0x03
#define SPI_NOR_CMD_RDID	0x9f

static inline u32 spifc_read(u32 reg)
{
	return readl(SPIFC_BASE + reg);
}

static inline void spifc_write(u32 reg, u32 val)
{
	writel(val, SPIFC_BASE + reg);
}

static inline void spifc_set(u32 reg, u32 mask)
{
	spifc_write(reg, spifc_read(reg) | mask);
}

static inline void spifc_clr(u32 reg, u32 mask)
{
	spifc_write(reg, spifc_read(reg) & ~mask);
}

static inline void spifc_update(u32 reg, u32 mask, u32 val)
{
	spifc_write(reg, (spifc_read(reg) & ~mask) | (val & mask));
}

/*
 * Transfer one chunk (up to 64 bytes) via SPIFC user mode.
 * dout/din may be NULL independently. keep_cs keeps CS asserted after transfer.
 */
static int spifc_chunk(const u8 *dout, u8 *din, int len, bool keep_cs)
{
	u32 data;
	int i, timeout;

	/* fill TX buffer */
	if (dout) {
		for (i = 0; i < len; i += 4) {
			if (len - i >= 4)
				data = *(const u32 *)(dout + i);
			else {
				data = 0;
				memcpy(&data, dout + i, len - i);
			}
			spifc_write(REG_C0 + i, data);
		}
	}

	/* enable DOUT stage */
	spifc_update(REG_USER, USER_UC_MASK, USER_UC_DOUT_SEL);
	/* set bit count */
	spifc_write(REG_USER1, (8 * len - 1) << 17);
	/* enable simultaneous RX */
	spifc_set(REG_USER, USER_DIN_EN_MS);

	/* CS control */
	spifc_update(REG_USER4, USER4_CS_ACT, keep_cs ? USER4_CS_ACT : 0);

	/* clear done, start */
	spifc_clr(REG_SLAVE, SLAVE_TRST_DONE);
	spifc_set(REG_CMD, CMD_USER);

	/* poll completion */
	timeout = SPIFC_TIMEOUT_US;
	while (!(spifc_read(REG_SLAVE) & SLAVE_TRST_DONE)) {
		if (--timeout <= 0)
			return -1;
		/* ~1us per iteration on typical Cortex-A53 */
	}

	/* drain RX buffer */
	if (din) {
		for (i = 0; i < len; i += 4) {
			data = spifc_read(REG_C0 + i);
			if (len - i >= 4)
				*(u32 *)(din + i) = data;
			else
				memcpy(din + i, &data, len - i);
		}
	}

	return 0;
}

/*
 * Check if SPIFC clock gate is enabled.
 * BL2 enables this when booting from SPI NOR but skips it on USB boot.
 */
bool board_nor_clk_enabled(void)
{
	return !!(readl(HHI_GCLK_MPEG0) & BIT(30));
}

/*
 * Program the SPI clock divider: clk81 / (n + 1), with
 * drivers/spi/meson_spifc.c meson_spifc_set_speed()'s field formula -- DIV n
 * [17:12], CNT_HIGH (n + 1) / 2 - 1 [11:6], CNT_LOW n [5:0].
 *
 * For a stage that has changed clk81 since the BootROM set the divider: the
 * GX TPL moves clk81 to its run rate before it reads NOR, so the BootROM's
 * divider no longer names the clock the part sees.
 */
void board_nor_set_div(u32 n)
{
	spifc_write(REG_CLOCK, (n << 12) | (((n + 1) / 2 - 1) << 6) | n);
}

/*
 * Initialize SPIFC for user-mode transfers.
 * Mirrors meson_spifc_hw_init() from drivers/spi/meson_spifc.c.
 * Returns false if SPIFC fails to initialize (clock gated, reset stuck).
 */
bool board_nor_init(void)
{
	int timeout = 10000;

	/* software reset */
	spifc_set(REG_SLAVE, SLAVE_SW_RST);

	/* wait for reset to clear -- spins forever if clock is gated */
	while (spifc_read(REG_SLAVE) & SLAVE_SW_RST) {
		if (--timeout <= 0)
			return false;
	}

	/* disable compatible mode */
	spifc_clr(REG_USER, USER_CMP_MODE);
	/* set master mode */
	spifc_clr(REG_SLAVE, SLAVE_OP_MODE);

	return true;
}

/*
 * Capacity of the fitted part, in bytes, as reported by its own JEDEC ID.
 * 0 until board_nor_probe() has seen a flash answer -- and 0 means exactly
 * "not known", never a default.
 */
static u32 nor_size;

/* The last JEDEC ID board_nor_probe() read, whatever it was (0 = none yet). */
static u32 nor_jedec;

/*
 * The three RDID bytes of the last probe, id[0] in bits 23:16.  For the
 * caller's log only: it lets a reader tell "no part" from "a part whose
 * density byte was not understood".
 */
u32 board_nor_jedec_id(void)
{
	return nor_jedec;
}

/*
 * JEDEC ID byte 3 (id[2]) is the density code: for every SPI NOR that follows
 * the JEDEC convention, capacity is 1 << id[2] bytes -- 0x15 is 4 MiB, 0x19 is
 * 32 MiB. Bound the shift rather than trusting the byte: the ID comes off the
 * wire, and a garbled or non-conforming code must produce "unknown", not a
 * wildly wrong size that then decides where the config block is read from.
 *
 * The lower bound is 64 KiB (the config block itself); the upper is 2 GiB,
 * past which the shift would not fit a u32.
 */
#define NOR_DENSITY_CODE_MIN	16
#define NOR_DENSITY_CODE_MAX	31

/*
 * Capacity in bytes, or 0 if unknown. Consumed by board_config_nor_offset()
 * (boot/board-config-nor.c), which places the config block in the last 64 KiB of
 * the addressable device.
 */
u32 board_nor_size(void)
{
	return nor_size;
}

/*
 * Probe SPI NOR by reading JEDEC ID (command 0x9F).
 * Returns true if a valid flash is detected, false otherwise.
 */
bool board_nor_probe(void)
{
	u8 cmd = SPI_NOR_CMD_RDID;
	u8 id[3] = {0, 0, 0};
	int ret;

	nor_size = 0;

	/* disable AHB for user-mode access */
	spifc_clr(REG_CTRL, CTRL_ENABLE_AHB);

	/* send RDID command, keep CS asserted */
	ret = spifc_chunk(&cmd, NULL, 1, true);
	if (ret)
		goto out;

	/* read 3-byte JEDEC ID, release CS */
	ret = spifc_chunk(NULL, id, 3, false);

out:
	/* re-enable AHB */
	spifc_set(REG_CTRL, CTRL_ENABLE_AHB);

	if (ret)
		return false;

	nor_jedec = id[0] << 16 | id[1] << 8 | id[2];

	/* 00,00,00 or FF,FF,FF means no flash responding */
	if ((id[0] == 0x00 && id[1] == 0x00 && id[2] == 0x00) ||
	    (id[0] == 0xff && id[1] == 0xff && id[2] == 0xff))
		return false;

	/*
	 * The ID was already being read and thrown away. Keep the density
	 * byte: it is the only thing on this board that knows how big the
	 * part is, since there is no spi-nor layer here to look one up.
	 *
	 * An out-of-range code leaves nor_size at 0 and the probe still
	 * succeeds -- a flash did answer, and the reads that do not need a
	 * size (the FIT at its own fixed offset) are unaffected. The
	 * end-of-device computation then falls back to the last block of the
	 * addressable window (board_config_nor_offset(), boot/board-config-nor.c),
	 * which is where every part of 16 MiB or more computes to anyway.
	 */
	if (id[2] >= NOR_DENSITY_CODE_MIN && id[2] <= NOR_DENSITY_CODE_MAX)
		nor_size = 1U << id[2];

	return true;
}

/*
 * Read 'len' bytes from SPI NOR at 'offset' into 'buf'.
 * Issues standard READ (0x03) command with 24-bit address.
 */
int board_nor_read(u32 offset, void *buf, u32 len)
{
	u8 cmd[4];
	u8 *p = buf;
	u32 done = 0;
	int chunk, ret;

	/* disable AHB for user-mode access */
	spifc_clr(REG_CTRL, CTRL_ENABLE_AHB);

	/* send READ command + 3-byte address, keep CS */
	cmd[0] = SPI_NOR_CMD_READ;
	cmd[1] = (offset >> 16) & 0xff;
	cmd[2] = (offset >> 8) & 0xff;
	cmd[3] = offset & 0xff;

	ret = spifc_chunk(cmd, NULL, 4, true);
	if (ret)
		goto out;

	/* read data in 64-byte chunks */
	while (done < len) {
		chunk = len - done;
		if (chunk > SPIFC_BUF_SIZE)
			chunk = SPIFC_BUF_SIZE;

		ret = spifc_chunk(NULL, p + done, chunk,
				  done + chunk < len);
		if (ret)
			goto out;

		done += chunk;
	}

out:
	/* re-enable AHB */
	spifc_set(REG_CTRL, CTRL_ENABLE_AHB);
	return ret;
}
