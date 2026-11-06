// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2014 Beniamino Galvani <b.galvani@gmail.com>
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 *
 * Amlogic Meson SPI Flash Controller driver
 */

#include <log.h>
#include <spi.h>
#include <spi-mem.h>
#include <clk.h>
#include <dm.h>
#include <regmap.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>

/* register map */
#define REG_CMD			0x00
#define REG_ADDR		0x04
#define REG_CTRL		0x08
#define REG_CTRL1		0x0c
#define REG_STATUS		0x10
#define REG_CTRL2		0x14
#define REG_CLOCK		0x18
#define REG_USER		0x1c
#define REG_USER1		0x20
#define REG_USER2		0x24
#define REG_USER3		0x28
#define REG_USER4		0x2c
#define REG_SLAVE		0x30
#define REG_SLAVE1		0x34
#define REG_SLAVE2		0x38
#define REG_SLAVE3		0x3c
#define REG_C0			0x40
#define REG_B8			0x60
#define REG_MAX			0x7c

/* register fields */
#define CMD_USER		BIT(18)
/*
 * The data stage of a read takes two lines (D and Q). Named FAST_READ_DUAL_OUT
 * in Amlogic's U-Boot (drivers/spi/spifc.c, which sets it for the DIN stage of
 * a 0x3B read); the datasheets name the register (SPIFC_CONTROL) but not its
 * bits.
 */
#define CTRL_FAST_READ_DUAL_OUT	BIT(14)
#define CTRL_ENABLE_AHB		BIT(17)
#define CLOCK_SOURCE		BIT(31)
#define CLOCK_DIV_SHIFT		12
#define CLOCK_DIV_MASK		(0x3f << CLOCK_DIV_SHIFT)
#define CLOCK_CNT_HIGH_SHIFT	6
#define CLOCK_CNT_HIGH_MASK	(0x3f << CLOCK_CNT_HIGH_SHIFT)
#define CLOCK_CNT_LOW_SHIFT	0
#define CLOCK_CNT_LOW_MASK	(0x3f << CLOCK_CNT_LOW_SHIFT)
#define USER_DIN_EN_MS		BIT(0)
#define USER_CMP_MODE		BIT(2)
#define USER_CLK_NOT_INV	BIT(7)
#define USER_UC_DOUT_SEL	BIT(27)
#define USER_UC_DIN_SEL		BIT(28)
#define USER_UC_MASK		((BIT(5) - 1) << 27)
#define USER1_BN_UC_DOUT_SHIFT	17
#define USER1_BN_UC_DOUT_MASK	(0xff << 16)
#define USER1_BN_UC_DIN_SHIFT	8
#define USER1_BN_UC_DIN_MASK	(0xff << 8)
#define USER4_CS_POL_HIGH	BIT(23)
#define USER4_IDLE_CLK_HIGH	BIT(29)
#define USER4_CS_ACT		BIT(30)
#define SLAVE_TRST_DONE		BIT(4)
#define SLAVE_OP_MODE		BIT(30)
#define SLAVE_SW_RST		BIT(31)

#define SPIFC_BUFFER_SIZE	64

/* spi-mem (spi-mem.o) is linked only in a phase with DM SPI */
#define MESON_SPIFC_MEM_OPS \
	(CONFIG_IS_ENABLED(DM_SPI) && IS_ENABLED(CONFIG_SPI_MEM))

struct meson_spifc_priv {
	struct regmap			*regmap;
	struct clk			clk;
};

/**
 * meson_spifc_drain_buffer() - copy data from device buffer to memory
 * @spifc:	the Meson SPI device
 * @buf:	the destination buffer
 * @len:	number of bytes to copy
 */
static void meson_spifc_drain_buffer(struct meson_spifc_priv *spifc,
				     u8 *buf, int len)
{
	u32 data;
	int i = 0;

	while (i < len) {
		regmap_read(spifc->regmap, REG_C0 + i, &data);

		if (len - i >= 4) {
			*((u32 *)buf) = data;
			buf += 4;
		} else {
			memcpy(buf, &data, len - i);
			break;
		}
		i += 4;
	}
}

/**
 * meson_spifc_fill_buffer() - copy data from memory to device buffer
 * @spifc:	the Meson SPI device
 * @buf:	the source buffer
 * @len:	number of bytes to copy
 */
static void meson_spifc_fill_buffer(struct meson_spifc_priv *spifc,
				    const u8 *buf, int len)
{
	u32 data = 0;
	int i = 0;

	while (i < len) {
		if (len - i >= 4)
			data = *(u32 *)buf;
		else
			memcpy(&data, buf, len - i);

		regmap_write(spifc->regmap, REG_C0 + i, data);

		buf += 4;
		i += 4;
	}
}

/**
 * meson_spifc_txrx() - transfer a chunk of data
 * @spifc:	the Meson SPI device
 * @dout:	data buffer for TX
 * @din:	data buffer for RX
 * @offset:	offset of the data to transfer
 * @len:	length of the data to transfer
 * @last_xfer:	whether this is the last transfer of the message
 * @last_chunk:	whether this is the last chunk of the transfer
 * Return:	0 on success, a negative value on error
 */
static int meson_spifc_txrx(struct meson_spifc_priv *spifc,
			    const u8 *dout, u8 *din, int offset,
			    int len, bool last_xfer, bool last_chunk)
{
	bool keep_cs = true;
	u32 data;
	int ret;

	if (dout)
		meson_spifc_fill_buffer(spifc, dout + offset, len);

	/* enable DOUT stage */
	regmap_update_bits(spifc->regmap, REG_USER, USER_UC_MASK,
			   USER_UC_DOUT_SEL);
	regmap_write(spifc->regmap, REG_USER1,
		     (8 * len - 1) << USER1_BN_UC_DOUT_SHIFT);

	/* enable data input during DOUT */
	regmap_update_bits(spifc->regmap, REG_USER, USER_DIN_EN_MS,
			   USER_DIN_EN_MS);

	if (last_chunk && last_xfer)
		keep_cs = false;

	regmap_update_bits(spifc->regmap, REG_USER4, USER4_CS_ACT,
			   keep_cs ? USER4_CS_ACT : 0);

	/* clear transition done bit */
	regmap_update_bits(spifc->regmap, REG_SLAVE, SLAVE_TRST_DONE, 0);
	/* start transfer */
	regmap_update_bits(spifc->regmap, REG_CMD, CMD_USER, CMD_USER);

	/* wait for the current operation to terminate */
	ret = regmap_read_poll_timeout(spifc->regmap, REG_SLAVE, data,
				       (data & SLAVE_TRST_DONE),
				       0, 5 * CONFIG_SYS_HZ);

	if (!ret && din)
		meson_spifc_drain_buffer(spifc, din + offset, len);

	return ret;
}

/**
 * meson_spifc_xfer() - perform a single transfer
 * @dev:	the SPI controller device
 * @bitlen:	length of the transfer
 * @dout:	data buffer for TX
 * @din:	data buffer for RX
 * @flags:	transfer flags
 * Return:	0 on success, a negative value on error
 */
static int meson_spifc_xfer(struct udevice *slave, unsigned int bitlen,
			    const void *dout, void *din, unsigned long flags)
{
	struct meson_spifc_priv *spifc = dev_get_priv(slave->parent);
	int blen = bitlen / 8;
	int len, done = 0, ret = 0;

	if (bitlen % 8)
		return -EINVAL;

	debug("xfer len %d (%d) dout %p din %p\n", bitlen, blen, dout, din);

	regmap_update_bits(spifc->regmap, REG_CTRL, CTRL_ENABLE_AHB, 0);

	while (done < blen && !ret) {
		len = min_t(int, blen - done, SPIFC_BUFFER_SIZE);
		ret = meson_spifc_txrx(spifc, dout, din, done, len,
				       flags & SPI_XFER_END,
				       done + len >= blen);
		done += len;
	}

	regmap_update_bits(spifc->regmap, REG_CTRL, CTRL_ENABLE_AHB,
			   CTRL_ENABLE_AHB);

	return ret;
}

#if MESON_SPIFC_MEM_OPS
/**
 * meson_spifc_din_stream() - receive a read's data stage in the DIN stage
 * @spifc:	the Meson SPI device
 * @buf:	the destination buffer
 * @nbytes:	number of bytes
 * Return:	0 on success, a negative value on error
 *
 * Nothing is driven on the data lines, so the data can arrive on two lines
 * (CTRL_FAST_READ_DUAL_OUT, set by the caller); the DOUT stage of
 * meson_spifc_txrx() drives D while it samples Q, which is one line only.
 * Chip select stays active from the caller's header until the last chunk.
 *
 * The stage is configured once and each 64-byte chunk is only started,
 * awaited and copied out. Per chunk, the regmap read-modify-writes of
 * meson_spifc_txrx() cost about as much as the chunk's 3.07 us on two lines
 * at 83.33 MHz.
 */
static int meson_spifc_din_stream(struct meson_spifc_priv *spifc, u8 *buf,
				  unsigned int nbytes)
{
	void __iomem *base = regmap_get_range(spifc->regmap, 0);
	unsigned int done, n, len = 0, i;
	u32 user4, slave, data;
	ulong start;

	clrsetbits_le32(base + REG_USER, USER_UC_MASK | USER_DIN_EN_MS,
			USER_UC_DIN_SEL);
	user4 = readl(base + REG_USER4) | USER4_CS_ACT;
	writel(user4, base + REG_USER4);
	slave = readl(base + REG_SLAVE) & ~SLAVE_TRST_DONE;

	for (done = 0; done < nbytes; done += n) {
		n = min_t(unsigned int, nbytes - done, SPIFC_BUFFER_SIZE);
		if (n != len) {
			writel((8 * n - 1) << USER1_BN_UC_DIN_SHIFT,
			       base + REG_USER1);
			len = n;
		}
		if (done + n >= nbytes)
			writel(user4 & ~USER4_CS_ACT, base + REG_USER4);

		writel(slave, base + REG_SLAVE);
		writel(CMD_USER, base + REG_CMD);
		start = get_timer(0);
		while (!(readl(base + REG_SLAVE) & SLAVE_TRST_DONE)) {
			if (get_timer(start) > 5 * CONFIG_SYS_HZ)
				return -ETIMEDOUT;
		}

		for (i = 0; i < n; i += 4) {
			data = readl(base + REG_C0 + i);
			memcpy(buf + done + i, &data, min_t(unsigned int, 4,
							    n - i));
		}
	}

	return 0;
}

/*
 * One line for the command, address and dummy cycles; one or, for a read, two
 * lines for the data. A GXL board wires D and Q only (no IO2/IO3), so two is
 * the most it can use.
 */
#define MESON_SPIFC_HDR_MAX	(1 + 4 + 16)

static bool meson_spifc_supports_op(struct spi_slave *slave,
				    const struct spi_mem_op *op)
{
	if (op->cmd.nbytes != 1 || op->cmd.buswidth != 1)
		return false;
	if (op->addr.nbytes && (op->addr.buswidth != 1 || op->addr.nbytes > 4))
		return false;
	if (op->dummy.nbytes &&
	    (op->dummy.buswidth != 1 || op->dummy.nbytes > 16))
		return false;
	if (op->data.nbytes && op->data.buswidth > 1 &&
	    (op->data.buswidth != 2 || op->data.dir != SPI_MEM_DATA_IN))
		return false;

	return spi_mem_default_supports_op(slave, op);
}

static int meson_spifc_exec_op(struct spi_slave *slave,
			       const struct spi_mem_op *op)
{
	struct meson_spifc_priv *spifc = dev_get_priv(slave->dev->parent);
	unsigned int nbytes = op->data.nbytes;
	bool dual = nbytes && op->data.buswidth == 2;
	u8 hdr[MESON_SPIFC_HDR_MAX];
	int i, n = 0, len, ret;
	unsigned int done;

	if (!meson_spifc_supports_op(slave, op))
		return -EOPNOTSUPP;

	hdr[n++] = op->cmd.opcode;
	for (i = op->addr.nbytes - 1; i >= 0; i--)
		hdr[n++] = op->addr.val >> (8 * i);
	memset(hdr + n, 0xff, op->dummy.nbytes);
	n += op->dummy.nbytes;

	regmap_update_bits(spifc->regmap, REG_CTRL, CTRL_ENABLE_AHB, 0);

	/* Chip select stays active into the data stage, if there is one */
	ret = meson_spifc_txrx(spifc, hdr, NULL, 0, n, !nbytes, true);

	if (!ret && nbytes && op->data.dir == SPI_MEM_DATA_IN) {
		if (dual) {
			regmap_update_bits(spifc->regmap, REG_CTRL,
					   CTRL_FAST_READ_DUAL_OUT,
					   CTRL_FAST_READ_DUAL_OUT);
			ret = meson_spifc_din_stream(spifc, op->data.buf.in,
						     nbytes);
			regmap_update_bits(spifc->regmap, REG_CTRL,
					   CTRL_FAST_READ_DUAL_OUT, 0);
		}
		for (done = 0; !dual && done < nbytes && !ret; done += len) {
			len = min_t(int, nbytes - done, SPIFC_BUFFER_SIZE);
			ret = meson_spifc_txrx(spifc, NULL, op->data.buf.in,
					       done, len, true,
					       done + len >= nbytes);
		}
	} else if (!ret && nbytes) {
		for (done = 0; done < nbytes && !ret; done += len) {
			len = min_t(int, nbytes - done, SPIFC_BUFFER_SIZE);
			ret = meson_spifc_txrx(spifc, op->data.buf.out, NULL,
					       done, len, true,
					       done + len >= nbytes);
		}
	}

	regmap_update_bits(spifc->regmap, REG_CTRL, CTRL_ENABLE_AHB,
			   CTRL_ENABLE_AHB);

	return ret;
}

static const struct spi_controller_mem_ops meson_spifc_mem_ops = {
	.supports_op	= meson_spifc_supports_op,
	.exec_op	= meson_spifc_exec_op,
};
#endif

/**
 * meson_spifc_parent_rate() - the controller's input rate without the CLK uclass
 *
 * Return: rate in Hz, or 0 if this platform cannot say.
 *
 * The controller needs a RATE, not a clock tree.  In an xPL phase built without
 * the CLK uclass, <clk.h>'s stub makes clk_get_rate() return -ENOSYS cast to
 * ulong, and the divider below is a computed function of that value -- so the
 * driver would bind and then immediately write a garbage divider.  The SoC
 * layer overrides this to read the rate out of the registers the ROM left,
 * which is the same argument meson_gx_mmc.c makes for SD_EMMC_CLOCK.
 *
 * Weak, and returning 0, so a platform that has not implemented it makes the
 * driver refuse rather than mis-program the clock.
 */
__weak ulong meson_spifc_parent_rate(void)
{
	return 0;
}

/**
 * meson_spifc_set_speed() - program the clock divider
 * @dev:	the SPI controller device
 * @speed:	desired speed in Hz
 */
static int meson_spifc_set_speed(struct udevice *dev, uint speed)
{
	struct meson_spifc_priv *spifc = dev_get_priv(dev);
	unsigned long parent, value;
	int n;

	parent = clk_get_rate(&spifc->clk);
	if (IS_ERR_VALUE(parent)) {
		parent = meson_spifc_parent_rate();
		if (!parent)
			return -ENOSYS;
	}

	n = max_t(int, parent / speed - 1, 1);

	debug("parent %lu, speed %u, n %d\n", parent, speed, n);

	value = (n << CLOCK_DIV_SHIFT) & CLOCK_DIV_MASK;
	value |= (n << CLOCK_CNT_LOW_SHIFT) & CLOCK_CNT_LOW_MASK;
	value |= (((n + 1) / 2 - 1) << CLOCK_CNT_HIGH_SHIFT) &
		CLOCK_CNT_HIGH_MASK;

	regmap_write(spifc->regmap, REG_CLOCK, value);

	return 0;
}

/**
 * meson_spifc_set_mode() - setups the SPI bus mode
 * @dev:	the SPI controller device
 * @mode:	desired mode bitfield
 * Return:	0 on success, -ENODEV on error
 */
static int meson_spifc_set_mode(struct udevice *dev, uint mode)
{
	struct meson_spifc_priv *spifc = dev_get_priv(dev);

	/* Two-line reads only through the spi-mem ops (meson_spifc_exec_op()) */
	if (mode & (SPI_CPHA | SPI_RX_QUAD | SPI_TX_QUAD | SPI_TX_DUAL))
		return -ENODEV;
	if ((mode & SPI_RX_DUAL) && !MESON_SPIFC_MEM_OPS)
		return -ENODEV;

	regmap_update_bits(spifc->regmap, REG_USER, USER_CLK_NOT_INV,
			   mode & SPI_CPOL ? USER_CLK_NOT_INV : 0);

	regmap_update_bits(spifc->regmap, REG_USER4, USER4_CS_POL_HIGH,
			   mode & SPI_CS_HIGH ? USER4_CS_POL_HIGH : 0);

	return 0;
}

/**
 * meson_spifc_hw_init() - reset and initialize the SPI controller
 * @spifc:	the Meson SPI device
 */
static void meson_spifc_hw_init(struct meson_spifc_priv *spifc)
{
	/* reset device */
	regmap_update_bits(spifc->regmap, REG_SLAVE, SLAVE_SW_RST,
			   SLAVE_SW_RST);
	/* disable compatible mode */
	regmap_update_bits(spifc->regmap, REG_USER, USER_CMP_MODE, 0);
	/* set master mode */
	regmap_update_bits(spifc->regmap, REG_SLAVE, SLAVE_OP_MODE, 0);
}

static const struct dm_spi_ops meson_spifc_ops = {
	.xfer		= meson_spifc_xfer,
	.set_speed	= meson_spifc_set_speed,
	.set_mode	= meson_spifc_set_mode,
#if MESON_SPIFC_MEM_OPS
	.mem_ops	= &meson_spifc_mem_ops,
#endif
};

static int meson_spifc_probe(struct udevice *dev)
{
	struct meson_spifc_priv *priv = dev_get_priv(dev);
	int ret;

	ret = regmap_init_mem(dev_ofnode(dev), &priv->regmap);
	if (ret)
		return ret;

	ret = clk_get_by_index(dev, 0, &priv->clk);
	if (ret == -ENOSYS) {
		/*
		 * No CLK uclass in this phase.  Not fatal: the gate this clock
		 * describes is already on when a stage gets here (the ROM used
		 * this controller, or clk81 is simply running), and
		 * meson_spifc_set_speed() has a rate source that does not need
		 * the framework.  priv->clk stays zeroed; clk_get_rate() on it
		 * returns the same -ENOSYS and set_speed takes the fallback.
		 */
	} else if (ret) {
		return ret;
	} else {
		ret = clk_enable(&priv->clk);
		if (ret)
			return ret;
	}

	meson_spifc_hw_init(priv);

	return 0;
}

static const struct udevice_id meson_spifc_ids[] = {
	{ .compatible = "amlogic,meson-gxbb-spifc", },
	{ }
};

U_BOOT_DRIVER(meson_spifc) = {
	.name		= "meson_spifc",
	.id		= UCLASS_SPI,
	.of_match	= meson_spifc_ids,
	.ops		= &meson_spifc_ops,
	.probe		= meson_spifc_probe,
	.priv_auto	= sizeof(struct meson_spifc_priv),
};
