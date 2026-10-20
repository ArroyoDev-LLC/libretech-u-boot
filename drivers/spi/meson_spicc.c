// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson SPICC controller driver
 *
 * Copyright (C) 2024 Da Xue <da@libre.computer>
 *
 * Ported from Linux driver by Neil Armstrong <narmstrong@baylibre.com>
 * and Da Xue.
 *
 * Transfer modes following the Linux driver:
 * - PIO: For small transfers (< fifo_size * 2 words). Uses XCH bit to
 *   trigger bursts of up to fifo_size words. Polls TC for completion.
 * - DMA: For large transfers. The SPICC DMA engine reads/writes 64-bit
 *   words from memory. Data is padded to 64-bit alignment. Uses SMC bit,
 *   LD_CNTL counters for burst management. Remainder handled via PIO.
 */

#include <clk.h>
#include <dm/pinctrl.h>
#include <cpu_func.h>
#include <dm.h>
#include <malloc.h>
#include <memalign.h>
#include <spi.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/log2.h>

/*
 * HHI_SPICC_CLK_CNTL register for direct clock setup when the U-Boot
 * clock driver doesn't support SPICC_SCLK clocks (IDs >= 256).
 *
 * SPICC1 bit fields (bits 16-31):
 *   [21:16] - divider (div = N+1)
 *   [22]    - gate enable
 *   [25:23] - mux select: 0=xtal, 1=clk81, 2=fclk_div4, 3=fclk_div3,
 *                          4=fclk_div2, 5=fclk_div5, 6=fclk_div7
 */
#define G12A_HHI_SPICC_CLK_CNTL	0xff63c3dcUL
#define SPICC1_SCLK_MUX_SHIFT		23
#define SPICC1_SCLK_MUX_MASK		(0x7 << 23)
#define SPICC1_SCLK_DIV_SHIFT		16
#define SPICC1_SCLK_DIV_MASK		(0x3f << 16)
#define SPICC1_SCLK_GATE		BIT(22)
#define SPICC1_SCLK_MUX_CLK81		1
#define SPICC1_SCLK_MUX_FCLK_DIV4	2
#define SPICC1_SCLK_FCLK_DIV4_HZ	500000000UL

/* Register offsets */
#define SPICC_RXDATA		0x00
#define SPICC_TXDATA		0x04
#define SPICC_CONREG		0x08
#define SPICC_INTREG		0x0c
#define SPICC_DMAREG		0x10
#define SPICC_STATREG		0x14
#define SPICC_PERIODREG		0x18
#define SPICC_TESTREG		0x1c
#define SPICC_DRADDR		0x20
#define SPICC_DWADDR		0x24
#define SPICC_LD_CNTL0		0x28
#define SPICC_LD_CNTL1		0x2c
#define SPICC_ENH_CTL0		0x38
#define SPICC_ENH_CTL1		0x3c
#define SPICC_ENH_CTL2		0x40

/* SPICC_CONREG bits */
#define SPICC_ENABLE		BIT(0)
#define SPICC_MODE_MASTER	BIT(1)
#define SPICC_XCH		BIT(2)
#define SPICC_SMC		BIT(3)
#define SPICC_POL		BIT(4)
#define SPICC_PHA		BIT(5)
#define SPICC_SSCTL		BIT(6)
#define SPICC_SSPOL		BIT(7)
#define SPICC_DRCTL_MASK	GENMASK(9, 8)
#define SPICC_CS_MASK		GENMASK(13, 12)
#define SPICC_DATARATE_MASK	GENMASK(18, 16)
#define SPICC_BITLENGTH_MASK	GENMASK(24, 19)
#define SPICC_BURSTLENGTH_MASK	GENMASK(31, 25)

/* SPICC_DMAREG bits */
#define SPICC_DMA_ENABLE		BIT(0)
#define SPICC_TXFIFO_THRESHOLD_MASK	GENMASK(5, 1)
#define SPICC_RXFIFO_THRESHOLD_MASK	GENMASK(10, 6)
#define SPICC_READ_BURST_MASK		GENMASK(14, 11)
#define SPICC_WRITE_BURST_MASK		GENMASK(18, 15)
#define SPICC_DMA_URGENT		BIT(19)

/* SPICC_STATREG bits */
#define SPICC_TE		BIT(0)	/* TX FIFO Empty */
#define SPICC_TH		BIT(1)	/* TX FIFO Half-Full */
#define SPICC_TF		BIT(2)	/* TX FIFO Full */
#define SPICC_RR		BIT(3)	/* RX FIFO Ready */
#define SPICC_RH		BIT(4)	/* RX FIFO Half-Full */
#define SPICC_RF		BIT(5)	/* RX FIFO Full */
#define SPICC_RO		BIT(6)	/* RX FIFO Overflow */
#define SPICC_TC		BIT(7)	/* Transfer Complete */

/*
 * SPICC_TESTREG layout (characterized on G12A/SM1):
 *
 * Bits 0-4:   TXCNT (read-only) — TX FIFO word count
 * Bits 5-9:   RXCNT (read-only) — RX FIFO word count (reads N-1 after N-word xfer)
 * Bits 10-13: read-only / reserved
 * Bit 14:     LBC — loopback control (regular R/W, NOT W1 toggle)
 * Bit 15:     writable, purpose unknown (does not affect loopback)
 * Bits 16-17: MO_DELAY — master output delay
 * Bits 18-19: MI_DELAY — master input delay
 * Bits 20-21: MI_CAP_DELAY — master input capture delay
 * Bits 22-23: FIFORST — FIFO soft-reset (write-1-to-trigger, auto-clears)
 *
 * FIFO geometry: 15 entries, each 64 bits wide. For bpw <= 32, one
 * TXDATA/RXDATA access per entry. For bpw > 32, two accesses (MSB first,
 * then LSB) are paired into a single 64-bit entry. TXCNT/RXCNT always
 * count logical words (entries), not individual 32-bit accesses.
 * BURSTLENGTH likewise counts logical words.
 *
 * The LBC bit position may vary across SoC variants and is detected at
 * probe time. Use RMW via spicc_setbits for all TESTREG writes to
 * preserve the LBC state.
 */
#define SPICC_TXCNT_MASK	GENMASK(4, 0)
#define SPICC_RXCNT_MASK	GENMASK(9, 5)
#define SPICC_LBC		BIT(14)	/* Loopback control (default position) */
#define SPICC_MO_DELAY_MASK	GENMASK(17, 16)
#define SPICC_MI_DELAY_MASK	GENMASK(19, 18)
#define SPICC_MI_CAP_DELAY_MASK	GENMASK(21, 20)
#define SPICC_FIFORST_W1_MASK	GENMASK(23, 22)

/* MI_DELAY values */
#define SPICC_MI_NO_DELAY	0
#define SPICC_MI_DELAY_1_CYCLE	1
#define SPICC_MI_DELAY_2_CYCLE	2
#define SPICC_MI_DELAY_3_CYCLE	3

/* MI_CAP_DELAY values */
#define SPICC_CAP_AHEAD_2_CYCLE	0
#define SPICC_CAP_AHEAD_1_CYCLE	1
#define SPICC_CAP_NO_DELAY	2
#define SPICC_CAP_DELAY_1_CYCLE	3

/* SPICC_LD_CNTL0 bits */
#define DMA_READ_COUNTER_EN	BIT(4)
#define DMA_WRITE_COUNTER_EN	BIT(5)

/* SPICC_LD_CNTL1 bits */
#define DMA_READ_COUNTER_MASK	GENMASK(15, 0)
#define DMA_WRITE_COUNTER_MASK	GENMASK(31, 16)

/* SPICC_ENH_CTL0 bits */
#define SPICC_ENH_DATARATE_MASK		GENMASK(23, 16)
#define SPICC_ENH_DATARATE_EN		BIT(24)
#define SPICC_ENH_MOSI_OEN		BIT(25)
#define SPICC_ENH_CLK_OEN		BIT(26)
#define SPICC_ENH_CS_OEN		BIT(27)
#define SPICC_ENH_MAIN_CLK_AO		BIT(29)

/*
 * DMA constants following the Linux driver.
 * The SPICC DMA engine always transfers 64-bit words from memory.
 * For 8-bit bpw, each byte is padded to an 8-byte DMA word.
 */
#define SPICC_DMA_WIDTH_BYTES	8
#define SPICC_DMA_MIN_FIFOS	2
#define DMA_BURST_WORDS_DEFAULT	8

#define SPICC_TIMEOUT_MS	100

/*
 * Maximum actual data bytes per DMA setup.
 * Padded DMA buffer is 8x this size.
 * 320KB covers a 320x480x2 framebuffer (307,200 bytes).
 */
#define SPICC_DMA_MAX_BYTES	(320 * 1024)

/* Default frequency-switch settling delay in clock cycles */
#define FREQ_SWITCH_DELAY_CYCLES	2

/*
 * Fallback when a slave names no speed. Matches SPI_DEFAULT_SPEED_HZ in
 * drivers/spi/spi-uclass.c, which is file-local there.
 */
#define SPICC_DEFAULT_SPEED_HZ		100000

/*
 * CONREG.CS is a 2-bit field, so the controller can address at most four chip
 * selects; a DT cs-gpios list longer than that cannot be reached anyway.
 */
#define SPICC_MAX_CS		4

struct meson_spicc_data {
	unsigned int max_speed_hz;
	unsigned int min_speed_hz;
	unsigned int fifo_size;
	unsigned int dma_bursts_max;
	bool has_oen;
	bool has_enhance_clk_div;
	bool has_pclk;
};

struct meson_spicc_priv {
	void __iomem *base;
	struct clk core_clk;
	struct clk pclk;
	bool pclk_enabled;
	unsigned long pclk_rate;
	const struct meson_spicc_data *data;
	unsigned int speed_hz;
	unsigned int mode;
	unsigned int xfer_bpw;		/* current BITLENGTH in CONREG */
	unsigned int clk_ns;		/* nanoseconds per SPI clock cycle */
	unsigned long last_speed_hz;	/* for freq-switch delay */
	u32 lbc_bit;			/* detected LBC bit position */
	bool bus_claimed;		/* bus in use, reject re-entrant claims */
	struct udevice *claim_dev;	/* slave holding the claim, or NULL */
	/*
	 * Chip select is per SLAVE, not per bus. One descriptor for the whole
	 * controller meant every transfer asserted cs-gpios[0] no matter which
	 * device was addressed, so a second slave (CS1) was never selected while
	 * the first one's line was pulled low under it. Keep the whole list and
	 * point active_cs at the claiming slave's entry, mirroring Linux, where
	 * host->use_gpio_descriptors = true has the SPI core drive per slave.
	 */
	struct gpio_desc cs_gpios[SPICC_MAX_CS];
	unsigned int num_cs_gpios;	/* entries populated from DT */
	struct gpio_desc *active_cs;	/* claiming slave's line, or NULL */
	/* DMA padded buffers (cache-line aligned, 8x actual data) */
	void *dma_tx_buf;
	void *dma_rx_buf;
	size_t dma_buf_size;
};

/* --- Low-level register helpers --- */

static inline u32 spicc_read(struct meson_spicc_priv *priv, u32 reg)
{
	return readl(priv->base + reg);
}

static inline void spicc_write(struct meson_spicc_priv *priv, u32 reg,
			       u32 val)
{
	writel(val, priv->base + reg);
}

/* Read-modify-write helper — safe for CONREG, ENH_CTL0, etc. */
static inline void spicc_setbits(struct meson_spicc_priv *priv, u32 reg,
				 u32 mask, u32 val)
{
	u32 tmp = spicc_read(priv, reg);

	tmp = (tmp & ~mask) | (val & mask);
	spicc_write(priv, reg, tmp);
}

/* --- FIFO reset --- */

/*
 * Reset TX and RX FIFOs. Uses direct writes to TESTREG to avoid
 * read-modify-write hazards with W1 bits at shifted positions.
 */
static void spicc_reset_fifo(struct meson_spicc_priv *priv)
{
	if (priv->data->has_oen)
		spicc_setbits(priv, SPICC_ENH_CTL0,
			      SPICC_ENH_MAIN_CLK_AO, SPICC_ENH_MAIN_CLK_AO);

	/* RMW to set FIFORST bits while preserving LBC and delay settings */
	spicc_setbits(priv, SPICC_TESTREG,
		      SPICC_FIFORST_W1_MASK, SPICC_FIFORST_W1_MASK);

	while (spicc_read(priv, SPICC_STATREG) & SPICC_RR)
		spicc_read(priv, SPICC_RXDATA);

	if (priv->data->has_oen)
		spicc_setbits(priv, SPICC_ENH_CTL0,
			      SPICC_ENH_MAIN_CLK_AO, 0);
}

/*
 * Detect which TESTREG bit controls loopback on this SoC.
 *
 * The LBC bit is a regular R/W bit (not W1 toggle). We detect its position
 * by writing candidate bits and checking which one reads back as set.
 * Tested candidates: BIT(14) (G12A/SM1), BIT(13) (possible on other SoCs).
 */
static void spicc_detect_lbc(struct udevice *bus)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	static const u32 candidates[] = { BIT(14), BIT(13), BIT(15) };
	u32 val;
	int i;

	/* Enable controller temporarily for TESTREG access */
	spicc_write(priv, SPICC_CONREG, SPICC_ENABLE | SPICC_MODE_MASTER);
	spicc_write(priv, SPICC_TESTREG, 0);

	for (i = 0; i < ARRAY_SIZE(candidates); i++) {
		/* Write the candidate bit */
		spicc_write(priv, SPICC_TESTREG, candidates[i]);
		val = spicc_read(priv, SPICC_TESTREG);

		if (val & candidates[i]) {
			priv->lbc_bit = candidates[i];

			/* Clear it back */
			spicc_write(priv, SPICC_TESTREG, 0);

			printf("spicc: LBC detected at bit %d\n",
			       ffs(priv->lbc_bit) - 1);
			goto done;
		}

		/* Clear before trying next candidate */
		spicc_write(priv, SPICC_TESTREG, 0);
	}

	/* Fallback */
	priv->lbc_bit = SPICC_LBC;
	printf("spicc: LBC detection failed, using default bit 14\n");

done:
	spicc_write(priv, SPICC_CONREG, 0);
}

/*
 * Descriptor for one chip select, or NULL when this bus drives CS in hardware
 * (no cs-gpios at all) or the DT list does not cover the requested index.
 */
static struct gpio_desc *spicc_cs_desc(struct meson_spicc_priv *priv,
				       unsigned int cs)
{
	if (cs >= priv->num_cs_gpios)
		return NULL;
	if (!dm_gpio_is_valid(&priv->cs_gpios[cs]))
		return NULL;

	return &priv->cs_gpios[cs];
}

static void spicc_oen_enable(struct meson_spicc_priv *priv)
{
	u32 conf;

	if (!priv->data->has_oen)
		return;

	conf = spicc_read(priv, SPICC_ENH_CTL0);
	conf |= SPICC_ENH_MOSI_OEN | SPICC_ENH_CLK_OEN;
	if (!priv->num_cs_gpios)
		conf |= SPICC_ENH_CS_OEN;
	else
		conf &= ~SPICC_ENH_CS_OEN;
	spicc_write(priv, SPICC_ENH_CTL0, conf);
}

/* --- Auto IO delay — following Linux meson_spicc_auto_io_delay --- */

static void spicc_auto_io_delay(struct meson_spicc_priv *priv)
{
	unsigned int speed_hz = priv->speed_hz;
	u32 mi_delay, cap_delay;
	u32 div, conf;

	if (priv->data->has_enhance_clk_div) {
		div = FIELD_GET(SPICC_ENH_DATARATE_MASK,
				spicc_read(priv, SPICC_ENH_CTL0));
		div++;
		div <<= 1;
	} else {
		div = FIELD_GET(SPICC_DATARATE_MASK,
				spicc_read(priv, SPICC_CONREG));
		div += 2;
		div = 1 << div;
	}

	mi_delay = SPICC_MI_NO_DELAY;
	cap_delay = SPICC_CAP_AHEAD_2_CYCLE;

	/*
	 * Delay table from Linux driver (LBC-validated values).
	 * These empirical settings ensure correct MISO capture timing.
	 */
	if (speed_hz >= 166666664)
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE;
	else if (speed_hz >= 124999998)
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE;
	else if (speed_hz >= 99999999)
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE;
	else if (speed_hz >= 83333332)
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE;
	else if (speed_hz >= 33333333)
		cap_delay = SPICC_CAP_AHEAD_1_CYCLE;
	else if (speed_hz <= 200000)
		mi_delay = SPICC_MI_NO_DELAY;
	else if (speed_hz <= 8000000 && div == 2)
		mi_delay = SPICC_MI_DELAY_3_CYCLE;
	else if (div >= 16)
		mi_delay = SPICC_MI_DELAY_3_CYCLE;
	else if (div >= 8)
		mi_delay = SPICC_MI_DELAY_2_CYCLE;
	else if (div >= 6)
		mi_delay = SPICC_MI_DELAY_1_CYCLE;

apply:
	/* RMW to update delay fields while preserving LBC state */
	conf = FIELD_PREP(SPICC_MI_DELAY_MASK, mi_delay) |
	       FIELD_PREP(SPICC_MI_CAP_DELAY_MASK, cap_delay);
	spicc_setbits(priv, SPICC_TESTREG,
		      SPICC_MI_DELAY_MASK | SPICC_MI_CAP_DELAY_MASK, conf);
}

/* --- DMA helpers --- */

/*
 * Each logical SPI word occupies one 64-bit DMA lane; the shift engine takes
 * the low CONREG.BITLENGTH bits of it. On this little-endian host a plain
 * copy of the word's Bpw bytes to the base of the lane therefore lands it in
 * the low bits, for any width up to 64 -- so the same code serves bpw 8 and
 * bpw 16 (the ILI9486 pixel stream) without a special case.
 */
static void spicc_pad_data(const u8 *src, u8 *dst, unsigned int words,
			   unsigned int Bpw)
{
	unsigned int i;

	memset(dst, 0, (size_t)words * SPICC_DMA_WIDTH_BYTES);
	for (i = 0; i < words; i++)
		memcpy(&dst[i * SPICC_DMA_WIDTH_BYTES], &src[i * Bpw], Bpw);
}

static void spicc_unpad_data(const u8 *src, u8 *dst, unsigned int words,
			     unsigned int Bpw)
{
	unsigned int i;

	for (i = 0; i < words; i++)
		memcpy(&dst[i * Bpw], &src[i * SPICC_DMA_WIDTH_BYTES], Bpw);
}

/* --- Clock configuration --- */

/*
 * Program the divider. Always writes hardware -- the ops->set_speed wrapper
 * below owns the "is the bus busy?" question.
 */
static int meson_spicc_apply_speed(struct udevice *bus, uint speed)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	unsigned long parent_rate;
	u32 div;

	if (speed > priv->data->max_speed_hz)
		speed = priv->data->max_speed_hz;
	if (speed < priv->data->min_speed_hz)
		speed = priv->data->min_speed_hz;

	priv->speed_hz = speed;

	parent_rate = priv->pclk_rate ? priv->pclk_rate :
		      clk_get_rate(&priv->core_clk);

	if (priv->data->has_enhance_clk_div) {
		div = DIV_ROUND_UP(parent_rate, speed * 2) - 1;
		if (div > 255)
			div = 255;

		spicc_setbits(priv, SPICC_ENH_CTL0,
			      SPICC_ENH_DATARATE_MASK | SPICC_ENH_DATARATE_EN,
			      FIELD_PREP(SPICC_ENH_DATARATE_MASK, div) |
			      SPICC_ENH_DATARATE_EN);

		debug("spicc: parent %lu, speed %u, enh div %u, actual %lu\n",
		      parent_rate, speed, div, parent_rate / 2 / (div + 1));
	} else {
		/*
		 * Legacy divider is a power of two: rate = parent / 4 / 2^div.
		 * Pick the smallest div whose resulting rate does not exceed
		 * the request.
		 *
		 * The previous DIV_ROUND_UP() + ilog2() form rounded ANY
		 * shortfall up to the next power of two, so a request of
		 * parent/4 lost a factor of TWO to integer truncation: with
		 * clk81 at 166666667, parent/4 is 41666666.75 and the ILI9486
		 * panel's DT asks for 41666666 -- three hertz short -- giving
		 * DIV_ROUND_UP(166666667, 166666664) = 2, ilog2() = 1, and a
		 * 20.83 MHz clock where 41.67 MHz was both requested and
		 * achievable. Measured effect on the MHS3528 panel: 2448 ->
		 * 4716 kB/s (125.4 -> 65.1 ms per 480x320 frame).
		 */
		for (div = 0; div < 7; div++)
			if (parent_rate / 4 / (1 << div) <= speed)
				break;

		spicc_setbits(priv, SPICC_CONREG, SPICC_DATARATE_MASK,
			      FIELD_PREP(SPICC_DATARATE_MASK, div));

		debug("spicc: parent %lu, speed %u, div %u, actual %lu\n",
		      parent_rate, speed, div,
		      parent_rate / 4 / (1 << div));
	}

	/*
	 * Frequency-switch settling delay following Linux driver.
	 * The IP needs old-frequency clock cycles to propagate the change.
	 */
	if (priv->last_speed_hz && speed != priv->last_speed_hz) {
		unsigned long min_hz = min((unsigned long)speed,
					   priv->last_speed_hz);
		unsigned int delay_ns = DIV_ROUND_UP(
			(u64)FREQ_SWITCH_DELAY_CYCLES * 1000000000ULL, min_hz);
		ndelay(delay_ns);
	}
	priv->last_speed_hz = speed;

	/* Compute nanoseconds per clock cycle for PIO delays */
	priv->clk_ns = speed ? DIV_ROUND_UP(1000000000U, speed) : 1000;

	return 0;
}

/*
 * ops->set_speed.
 *
 * The SPI uclass programs speed and mode from dm_spi_claim_bus() BEFORE it
 * calls ops->claim_bus() -- so a claim this driver is about to reject with
 * -EBUSY (the cyclic video sync re-entering through udelay()/schedule()
 * mid-transfer) has ALREADY rewritten the divider under the transfer that is
 * on the wire, and the rejection comes too late to undo it.
 *
 * Ignore any speed request that arrives while the bus is held. Nothing is
 * lost: claim_bus() below programs the incoming slave's own speed itself, so
 * the value is re-derived from the slave at the next legitimate claim rather
 * than inherited from whoever asked last.
 */
static int meson_spicc_set_speed(struct udevice *bus, uint speed)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);

	if (priv->bus_claimed)
		return 0;

	return meson_spicc_apply_speed(bus, speed);
}

/*
 * Set SPI mode (CPOL, CPHA, CS_HIGH, LOOP).
 *
 * LBC (loopback) is a regular R/W bit in TESTREG. Position detected at probe.
 *
 * Always writes hardware; ops->set_mode below owns the bus-busy question.
 */
static int meson_spicc_apply_mode(struct udevice *bus, uint mode)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	u32 conf = 0;

	priv->mode = mode;

	if (mode & SPI_CPOL)
		conf |= SPICC_POL;
	if (mode & SPI_CPHA)
		conf |= SPICC_PHA;
	if (mode & SPI_CS_HIGH)
		conf |= SPICC_SSPOL;

	spicc_setbits(priv, SPICC_CONREG,
		      SPICC_POL | SPICC_PHA | SPICC_SSPOL, conf);

	/* Set or clear LBC bit via RMW */
	spicc_setbits(priv, SPICC_TESTREG, priv->lbc_bit,
		      (mode & SPI_LOOP) ? priv->lbc_bit : 0);

	return 0;
}

/* ops->set_mode. Deferred while the bus is held -- see set_speed above. */
static int meson_spicc_set_mode(struct udevice *bus, uint mode)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);

	if (priv->bus_claimed)
		return 0;

	return meson_spicc_apply_mode(bus, mode);
}

/*
 * SCLK pad parking. The SPICC parks SCLK HIGH between bursts (measured on
 * GXL pads), which mode-0 slaves tolerate but clock-counting SPI-to-parallel
 * display converters (MHS3528/PiScreen 16-bit latch, no CS reframe) do not:
 * every idle-level edge shifts their free-running frame counter, garbling
 * all subsequent words until the panel's next hardware reset. Validated on
 * the bench S905X: with SCLK force-parked low between frames the panel
 * initializes; with the controller's native parking it never does.
 *
 * Park by selecting the DT "idle-low"/"idle-high" pinctrl state (per CPOL)
 * while the bus is idle -- the same mechanism the Linux LC driver uses --
 * and restore "default" around transfers. Boards whose DT carries no idle
 * states silently keep the old behavior.
 */
static void spicc_pins_idle(struct udevice *bus)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);

	pinctrl_select_state(bus, (priv->mode & SPI_CPOL) ?
			     "idle-high" : "idle-low");
}

static void spicc_pins_active(struct udevice *bus)
{
	pinctrl_select_state(bus, "default");
}

/* --- Bus claim/release --- */

static int meson_spicc_claim_bus(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	struct dm_spi_slave_plat *slave = dev_get_parent_plat(dev);
	struct spi_slave *slave_priv = dev_get_parent_priv(dev);
	struct dm_spi_bus *dm_bus = dev_get_uclass_priv(bus);
	unsigned int speed, mode;
	u32 conf;

	/*
	 * Reject re-entrant claims. The cyclic video sync can trigger
	 * dm_spi_claim_bus via udelay/get_timer during another transfer,
	 * which would reconfigure the controller mid-transfer.
	 */
	if (priv->bus_claimed)
		return -EBUSY;

	priv->bus_claimed = true;
	priv->claim_dev = dev;

	/*
	 * Latch the CS line belonging to THIS slave for the life of the claim.
	 * SPI_XFER_BEGIN/END below drive only this one, so a transfer to CS1 no
	 * longer asserts CS0's line.
	 */
	priv->active_cs = spicc_cs_desc(priv, slave->cs[0]);
	if (priv->num_cs_gpios && !priv->active_cs)
		dev_warn(bus, "no cs-gpio for cs %u; leaving CS to hardware\n",
			 slave->cs[0]);

	/*
	 * Word length is a property of the SLAVE, not the bus. Caching it in
	 * the bus priv let one slave's width leak to every other device on
	 * the controller (nothing re-applies it per claim -- the SPI uclass
	 * re-applies speed and mode only), so a display asking for 16 left a
	 * touch controller or flash transferring at 16 too.
	 */
	priv->xfer_bpw = slave->wordlen ? slave->wordlen : 8;

	/*
	 * Build CONREG preserving DATARATE field (matches Linux driver).
	 */
	conf = spicc_read(priv, SPICC_CONREG) & SPICC_DATARATE_MASK;
	conf |= SPICC_ENABLE | SPICC_MODE_MASTER;
	conf |= FIELD_PREP(SPICC_CS_MASK, slave->cs[0] & 0x3);
	conf |= FIELD_PREP(SPICC_BITLENGTH_MASK, priv->xfer_bpw - 1);
	spicc_write(priv, SPICC_CONREG, conf);

	/* Reset FIFOs */
	spicc_reset_fifo(priv);

	/*
	 * Speed and mode belong to the SLAVE, not the bus -- the same disease
	 * word length had before f76949fd601. Re-applying the bus-wide
	 * priv->speed_hz/priv->mode here meant "whatever was programmed last,
	 * by whichever device", and the SPI uclass cannot be relied on to have
	 * corrected it first: dm_spi_claim_bus() only calls set_speed/set_mode
	 * when the requested values differ from its own dm_spi_bus cache, and
	 * _spi_get_bus_and_cs() (cmd/spi.c "sspi") programs the controller
	 * directly WITHOUT updating that cache. After `sspi 0:0` the cache
	 * still read 41666666 while the divider was at sspi's 1 MHz request,
	 * so the panel's next claim was skipped by the uclass and then pinned
	 * to 1 MHz here -- silently, with no error and no DMA timeout.
	 *
	 * Derive both from the claiming slave every time, mirroring the
	 * uclass's own clamping, so the value on the wire depends only on the
	 * device being addressed.
	 */
	speed = slave_priv ? slave_priv->max_hz : 0;
	mode = slave_priv ? slave_priv->mode : 0;
	if (dm_bus && dm_bus->max_hz)
		speed = speed ? min(speed, dm_bus->max_hz) : dm_bus->max_hz;
	if (!speed)
		speed = SPICC_DEFAULT_SPEED_HZ;

	meson_spicc_apply_mode(bus, mode);
	meson_spicc_apply_speed(bus, speed);

	/* Configure IO delays for correct capture timing */
	spicc_auto_io_delay(priv);

	/* Disable interrupts - we poll */
	spicc_write(priv, SPICC_INTREG, 0);
	spicc_write(priv, SPICC_PERIODREG, 0);
	spicc_write(priv, SPICC_DMAREG, 0);

	spicc_pins_idle(bus);

	return 0;
}

static int meson_spicc_release_bus(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct meson_spicc_priv *priv = dev_get_priv(bus);

	/*
	 * Only the holder may release. dm_spi_release_bus() has no return
	 * value and the uclass does not track whether ops->claim_bus()
	 * succeeded, so a caller whose claim we rejected with -EBUSY can still
	 * reach here (cmd/spi.c's do_spi_xfer() does exactly that: it jumps to
	 * its `done:` label and releases unconditionally). Tearing the bus
	 * down there would deassert the real holder's chip select and clear
	 * bus_claimed in the middle of its transfer.
	 */
	if (!priv->bus_claimed || priv->claim_dev != dev)
		return 0;

	if (priv->active_cs)
		dm_gpio_set_value(priv->active_cs, 0);
	priv->active_cs = NULL;

	spicc_write(priv, SPICC_INTREG, 0);
	spicc_write(priv, SPICC_DMAREG, 0);

	/* Disable controller, preserving DATARATE */
	spicc_write(priv, SPICC_CONREG,
		    spicc_read(priv, SPICC_CONREG) & SPICC_DATARATE_MASK);

	priv->bus_claimed = false;
	priv->claim_dev = NULL;

	return 0;
}

/* --- PIO TX/RX following Linux driver bpw ranges --- */

/*
 * Fill TX FIFO with @count words from @buf.
 *
 * bpw 1-8:   1 byte per word, lower bpw bits used
 * bpw 9-16:  2 bytes per word (u16 little-endian)
 * bpw 17-24: 3 bytes per word (assembled byte-by-byte)
 * bpw 25-32: 4 bytes per word (u32 little-endian)
 * bpw 33-56: 4+N bytes per word, two TXDATA writes (MSB first, LSB second)
 * bpw 57-64: 8 bytes per word, two TXDATA writes (MSB first, LSB second)
 */
static void spicc_pio_fill_tx(struct meson_spicc_priv *priv,
			      const u8 **txp, unsigned int count,
			      unsigned int bpw, unsigned int Bpw)
{
	unsigned int i, j;
	u32 data, data2;

	if (bpw <= 8) {
		for (i = 0; i < count; i++) {
			data = *txp ? *(*txp)++ : 0;
			spicc_write(priv, SPICC_TXDATA, data);
		}
	} else if (bpw <= 16) {
		for (i = 0; i < count; i++) {
			if (*txp) {
				data = *(const u16 *)*txp;
				*txp += 2;
			} else {
				data = 0;
			}
			spicc_write(priv, SPICC_TXDATA, data);
		}
	} else if (bpw <= 24) {
		/* 17-24 bpw: assemble from bytes (Linux approach) */
		for (i = 0; i < count; i++) {
			data = 0;
			if (*txp) {
				for (j = 0; j < Bpw; j++)
					data |= (u32)(*txp)[j] << (j * 8);
				*txp += Bpw;
			}
			spicc_write(priv, SPICC_TXDATA, data);
		}
	} else if (bpw <= 32) {
		for (i = 0; i < count; i++) {
			if (*txp) {
				data = *(const u32 *)*txp;
				*txp += 4;
			} else {
				data = 0;
			}
			spicc_write(priv, SPICC_TXDATA, data);
		}
	} else if (bpw <= 56) {
		/* 33-56 bpw: LSB u32 + MSB bytes, write MSB first */
		for (i = 0; i < count; i++) {
			if (*txp) {
				data2 = *(const u32 *)*txp;	/* LSB */
				data = 0;			/* MSB */
				for (j = 0; j < Bpw - 4; j++)
					data |= (u32)(*txp)[4 + j] << (j * 8);
				*txp += Bpw;
			} else {
				data = data2 = 0;
			}
			spicc_write(priv, SPICC_TXDATA, data);  /* MSB */
			spicc_write(priv, SPICC_TXDATA, data2); /* LSB */
		}
	} else {
		/* 57-64 bpw: two u32, write MSB first */
		for (i = 0; i < count; i++) {
			if (*txp) {
				data2 = *(const u32 *)*txp;	 /* LSB */
				data = *(const u32 *)(*txp + 4); /* MSB */
				*txp += 8;
			} else {
				data = data2 = 0;
			}
			spicc_write(priv, SPICC_TXDATA, data);  /* MSB */
			spicc_write(priv, SPICC_TXDATA, data2); /* LSB */
		}
	}
}

/*
 * Drain RX FIFO of @count words into @buf.
 * Mirror of spicc_pio_fill_tx byte layout.
 */
static void spicc_pio_drain_rx(struct meson_spicc_priv *priv,
			       u8 **rxp, unsigned int count,
			       unsigned int bpw, unsigned int Bpw)
{
	unsigned int i, j;
	u32 data, data2;
	u32 mask = GENMASK(min(bpw, 32U) - 1, 0);
	u32 msb_mask = (bpw > 32) ? GENMASK(bpw - 32 - 1, 0) : 0;

	if (bpw <= 8) {
		for (i = 0; i < count; i++) {
			data = spicc_read(priv, SPICC_RXDATA) & mask;
			if (*rxp)
				*(*rxp)++ = data;
		}
	} else if (bpw <= 16) {
		for (i = 0; i < count; i++) {
			data = spicc_read(priv, SPICC_RXDATA) & mask;
			if (*rxp) {
				*(u16 *)*rxp = data;
				*rxp += 2;
			}
		}
	} else if (bpw <= 24) {
		for (i = 0; i < count; i++) {
			data = spicc_read(priv, SPICC_RXDATA) & mask;
			if (*rxp) {
				for (j = 0; j < Bpw; j++)
					(*rxp)[j] = data >> (j * 8);
				*rxp += Bpw;
			}
		}
	} else if (bpw <= 32) {
		for (i = 0; i < count; i++) {
			data = spicc_read(priv, SPICC_RXDATA) & mask;
			if (*rxp) {
				*(u32 *)*rxp = data;
				*rxp += 4;
			}
		}
	} else if (bpw <= 56) {
		for (i = 0; i < count; i++) {
			data = spicc_read(priv, SPICC_RXDATA) & msb_mask; /* MSB */
			data2 = spicc_read(priv, SPICC_RXDATA);  /* LSB */
			if (*rxp) {
				*(u32 *)*rxp = data2;		  /* LSB */
				for (j = 0; j < Bpw - 4; j++)
					(*rxp)[4 + j] = data >> (j * 8);
				*rxp += Bpw;
			}
		}
	} else {
		for (i = 0; i < count; i++) {
			data = spicc_read(priv, SPICC_RXDATA) & msb_mask; /* MSB */
			data2 = spicc_read(priv, SPICC_RXDATA);  /* LSB */
			if (*rxp) {
				*(u32 *)*rxp = data2;		  /* LSB */
				*(u32 *)(*rxp + 4) = data;	  /* MSB */
				*rxp += 8;
			}
		}
	}
}

/*
 * PIO transfer following the Linux driver.
 * Transfers up to fifo_size words per burst using XCH, polls TC.
 */
static int meson_spicc_xfer_pio(struct udevice *dev, const void *tx, void *rx,
				unsigned int words, unsigned int bpw,
				unsigned int Bpw)
{
	struct udevice *bus = dev->parent;
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	const u8 *txp = tx;
	u8 *rxp = rx;
	unsigned int burst, fifo_active, retry;
	unsigned long start;

	/*
	 * The FIFO is 64 bits wide. For bpw > 32, two TXDATA/RXDATA
	 * accesses (MSB + LSB) are paired into one 64-bit FIFO entry.
	 * TXCNT counts logical words, so fifo_active is the same
	 * regardless of bpw.
	 */
	fifo_active = priv->data->fifo_size;

	while (words) {
		burst = min_t(unsigned int, words, fifo_active);

		/* Set burst length (Linux: meson_spicc_setup_pio) */
		spicc_setbits(priv, SPICC_CONREG, SPICC_BURSTLENGTH_MASK,
			      FIELD_PREP(SPICC_BURSTLENGTH_MASK, burst - 1));

		/* Wait one SPI clock cycle for CONREG to take effect */
		ndelay(priv->clk_ns);

		/* Fill TX FIFO */
		spicc_pio_fill_tx(priv, &txp, burst, bpw, Bpw);

		/* Wait for TX FIFO to settle before XCH (Linux: ndelay after tx) */
		ndelay(priv->clk_ns);

		/* Clear TC from any previous transfer before starting */
		spicc_write(priv, SPICC_STATREG, SPICC_TC);

		/* Start burst via XCH */
		spicc_setbits(priv, SPICC_CONREG, SPICC_XCH, SPICC_XCH);

		/* Poll for Transfer Complete */
		start = get_timer(0);
		for (;;) {
			if (spicc_read(priv, SPICC_STATREG) & SPICC_TC)
				break;
			if (get_timer(start) > SPICC_TIMEOUT_MS) {
				dev_err(bus, "PIO timeout (%u words)\n", words);
				return -ETIMEDOUT;
			}
			udelay(1);
		}

		/*
		 * XCH quirk: TC may fire while TX FIFO still has data
		 * (Linux driver: "XCH occasionally fails to do TX, returns
		 * TC but fifo still loaded"). The Linux IRQ handler
		 * re-checks on every interrupt; do the same here with a
		 * bounded retry loop instead of a single-shot retry.
		 * TC must be cleared (W1C) before each re-trigger --
		 * otherwise the stale TC satisfies the poll immediately
		 * and the retriggered burst is treated as complete while
		 * still shifting, dropping its tail.
		 */
		for (retry = 0;
		     FIELD_GET(SPICC_TXCNT_MASK,
			       spicc_read(priv, SPICC_TESTREG));
		     retry++) {
			if (retry >= 8) {
				dev_err(bus, "PIO XCH retry: TX FIFO stuck\n");
				return -EIO;
			}
			spicc_write(priv, SPICC_STATREG, SPICC_TC);
			spicc_setbits(priv, SPICC_CONREG,
				      SPICC_XCH, SPICC_XCH);
			start = get_timer(0);
			while (!(spicc_read(priv, SPICC_STATREG) & SPICC_TC)) {
				if (get_timer(start) > SPICC_TIMEOUT_MS) {
					dev_err(bus, "PIO XCH retry timeout\n");
					return -ETIMEDOUT;
				}
				udelay(1);
			}
		}

		/* Wait for RX to be ready (Linux: ndelay before rx) */
		ndelay(priv->clk_ns);

		/* Drain RX FIFO */
		spicc_pio_drain_rx(priv, &rxp, burst, bpw, Bpw);

		words -= burst;
	}

	return 0;
}

/* --- DMA transfer --- */

static u32 spicc_calc_dma_burst(struct meson_spicc_priv *priv,
				u32 words, u32 *burst_words)
{
	u32 dma_words_max;
	u32 burst_count;
	unsigned int i;

	if (words <= priv->data->fifo_size) {
		*burst_words = words;
		return 1;
	}

	*burst_words = DMA_BURST_WORDS_DEFAULT;

	dma_words_max = priv->data->dma_bursts_max * DMA_BURST_WORDS_DEFAULT;
	if (words >= dma_words_max)
		return priv->data->dma_bursts_max;

	for (i = DMA_BURST_WORDS_DEFAULT; i > 1; i--) {
		if ((words % i) == 0) {
			*burst_words = i;
			burst_count = words / i;
			if (burst_count <= priv->data->dma_bursts_max)
				return burst_count;
			return priv->data->dma_bursts_max;
		}
	}

	return words / DMA_BURST_WORDS_DEFAULT;
}

static int meson_spicc_xfer_dma(struct udevice *dev, const u8 *tx, u8 *rx,
				unsigned int words, unsigned int Bpw)
{
	struct udevice *bus = dev->parent;
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	u32 dma_burst_words, dma_burst_count, xfers;
	u32 txfifo_thres, rxfifo_thres;
	size_t padded_len;
	ulong tx_phys, rx_phys;
	unsigned long start;
	u32 dmareg;

	unsigned int done = 0;

	padded_len = (size_t)words * SPICC_DMA_WIDTH_BYTES;
	if (padded_len > priv->dma_buf_size) {
		dev_err(bus, "DMA buf too small: need %zu, have %zu\n",
			padded_len, priv->dma_buf_size);
		return -EINVAL;
	}

	if (tx)
		spicc_pad_data(tx, priv->dma_tx_buf, words, Bpw);
	else
		memset(priv->dma_tx_buf, 0, padded_len);

	tx_phys = (ulong)priv->dma_tx_buf;
	rx_phys = (ulong)priv->dma_rx_buf;

	flush_dcache_range(tx_phys, tx_phys + padded_len);

	/*
	 * RX cache maintenance is only needed if someone will READ the
	 * received data. A tx-only client (a display push is the common case)
	 * passes rx == NULL, and the unpad below is already skipped for it --
	 * so the two invalidates over the full padded length are pure
	 * overhead on exactly the shape that moves the most bytes.
	 *
	 * The DMA WRITE channel stays fully armed either way: DWADDR still
	 * points at dma_rx_buf and DMA_WRITE_COUNTER_EN stays set, because the
	 * engine drains the RX FIFO to memory regardless and stalls if it
	 * cannot. Only the CPU-side cache work is skipped. Nothing reads
	 * dma_rx_buf while rx == NULL, and the next rx-bearing transfer
	 * re-invalidates it here before use, so stale lines cannot leak out.
	 */
	if (rx)
		invalidate_dcache_range(rx_phys, rx_phys + padded_len);

	/*
	 * On GXL (!has_oen) the shift engine honors CONREG.BURSTLENGTH even
	 * in SMC/DMA mode: a session terminates after BURSTLENGTH+1 words no
	 * matter what the LD counters say, stranding the remainder in the
	 * DMA engine (DMA_ENABLE never clears -> timeout). Validated on
	 * silicon (S905X, internal loopback): 128 words pass, 129 stall.
	 * Split the transfer into <=128-word sessions, advancing the DMA
	 * addresses per session. On G12/SM1 (has_oen) BURSTLENGTH is not
	 * used in DMA mode and one session covers the whole transfer.
	 *
	 * Do NOT replace the per-session DRADDR/DWADDR writes with the
	 * LD_RADDR/LD_WADDR "address reload" path (LD_CNTL0 bit 8). Measured
	 * on S905X, 2026-08-05: with bit 8 set the engine sources from the LD
	 * registers, but it does NOT post-increment them -- across a
	 * 300-session frame every session re-used the same address (only the
	 * first 1 KB destination block was ever written; the DRADDR/DWADDR
	 * path wrote all of them). LD_CNTL0 bits 6 and 7 do not change that,
	 * alone or together. So continuation still needs two address writes
	 * per session -- exactly what this loop already does -- and the
	 * session boundary itself is mandatory anyway because it is set by
	 * the 7-bit CONREG.BURSTLENGTH shift-engine limit, not by the
	 * addresses. Even a free reload would be worth ~60 us of a 65 ms
	 * frame (2 writes x 300 sessions x ~0.10 us), below run-to-run noise.
	 */
	while (done < words) {
		u32 chunk = words - done;

		if (!priv->data->has_oen && chunk > 128)
			chunk = 128;

		/* DMA needs at least two FIFO-fulls; PIO handles the tail */
		if (chunk < priv->data->fifo_size * SPICC_DMA_MIN_FIFOS)
			break;

		dma_burst_count = spicc_calc_dma_burst(priv, chunk,
						       &dma_burst_words);
		xfers = dma_burst_count * dma_burst_words;

		/* Reset FIFO before DMA setup (matches Linux driver) */
		spicc_reset_fifo(priv);

		/* Write DMA addresses (advanced by the padded stride) */
		spicc_write(priv, SPICC_DRADDR,
			    tx_phys + (ulong)done * SPICC_DMA_WIDTH_BYTES);
		spicc_write(priv, SPICC_DWADDR,
			    rx_phys + (ulong)done * SPICC_DMA_WIDTH_BYTES);

		dma_burst_words--;

		if (!priv->data->has_oen)
			spicc_setbits(priv, SPICC_CONREG,
				      SPICC_BURSTLENGTH_MASK,
				      FIELD_PREP(SPICC_BURSTLENGTH_MASK,
						 xfers - 1));

		spicc_write(priv, SPICC_LD_CNTL0,
			    DMA_READ_COUNTER_EN | DMA_WRITE_COUNTER_EN);
		spicc_write(priv, SPICC_LD_CNTL1,
			    FIELD_PREP(DMA_READ_COUNTER_MASK, dma_burst_count) |
			    FIELD_PREP(DMA_WRITE_COUNTER_MASK, dma_burst_count));

		txfifo_thres = priv->data->fifo_size - dma_burst_words;
		rxfifo_thres = dma_burst_words;

		dmareg = SPICC_DMA_ENABLE | SPICC_DMA_URGENT |
			 FIELD_PREP(SPICC_TXFIFO_THRESHOLD_MASK, txfifo_thres) |
			 FIELD_PREP(SPICC_RXFIFO_THRESHOLD_MASK, rxfifo_thres) |
			 FIELD_PREP(SPICC_READ_BURST_MASK, dma_burst_words) |
			 FIELD_PREP(SPICC_WRITE_BURST_MASK, dma_burst_words);

		spicc_write(priv, SPICC_DMAREG, dmareg);

		spicc_setbits(priv, SPICC_CONREG, SPICC_SMC, SPICC_SMC);

		/*
		 * Silicon quirk (Linux driver: "SMC occasionally fails to do
		 * TX"): verify SMC actually latched and re-set it once if
		 * not. Without this a silent SMC failure burns the full
		 * transfer timeout.
		 */
		ndelay(priv->clk_ns);
		if (!(spicc_read(priv, SPICC_CONREG) & SPICC_SMC)) {
			dev_warn(bus, "SMC failed to latch, retrying\n");
			spicc_setbits(priv, SPICC_CONREG, SPICC_SMC,
				      SPICC_SMC);
		}

		start = get_timer(0);
		while (spicc_read(priv, SPICC_DMAREG) & SPICC_DMA_ENABLE) {
			if (get_timer(start) > SPICC_TIMEOUT_MS) {
				dev_err(bus,
					"DMA timeout (%u words, %u bursts, %u done)\n",
					xfers, dma_burst_count, done);
				spicc_setbits(priv, SPICC_CONREG,
					      SPICC_SMC, 0);
				spicc_write(priv, SPICC_DMAREG, 0);
				return -ETIMEDOUT;
			}
			udelay(1);
		}

		spicc_setbits(priv, SPICC_CONREG, SPICC_SMC, 0);
		spicc_write(priv, SPICC_DMAREG, 0);

		done += xfers;

		/* calc could not cover the chunk: leave rest to PIO */
		if (xfers < chunk)
			break;
	}

	if (rx) {
		invalidate_dcache_range(rx_phys, rx_phys + padded_len);
		spicc_unpad_data(priv->dma_rx_buf, rx, done, Bpw);
	}

	return done;
}

/* --- Main transfer function --- */

static int meson_spicc_xfer(struct udevice *dev, unsigned int bitlen,
			    const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	struct dm_spi_slave_plat *slave = dev_get_parent_plat(dev);
	/*
	 * Per TRANSFER, from the slave -- mirroring Linux, which programs
	 * CONREG.BITLENGTH from xfer->bits_per_word on every transfer. A
	 * device may legitimately change width between transfers: the ILI9486
	 * sends commands at 8 bits and pixel data at 16.
	 */
	unsigned int bpw = slave->wordlen ? slave->wordlen : 8;
	unsigned int words, Bpw;
	int ret;

	if (!bitlen)
		return 0;

	/*
	 * A transfer from a device that is not the claim holder would run on
	 * the holder's chip select and the holder's speed/mode. Refuse it
	 * rather than put another slave's bytes on the wire. (A transfer with
	 * no claim at all is left alone: the U-Boot SPI contract requires a
	 * claim, but rejecting it here would be a behaviour change for
	 * consumers this driver cannot test.)
	 */
	if (priv->bus_claimed && priv->claim_dev != dev) {
		dev_err(bus, "xfer from '%s' while '%s' holds the bus\n",
			dev->name, priv->claim_dev ? priv->claim_dev->name : "?");
		return -EBUSY;
	}

	if (bitlen < bpw) {
		bpw = bitlen;
	} else if (bitlen % bpw) {
		dev_err(bus, "bitlen %u not aligned to bpw %u\n", bitlen, bpw);
		return -EINVAL;
	}

	words = bitlen / bpw;
	Bpw = DIV_ROUND_UP(bpw, 8);

	/* Update BITLENGTH in CONREG if changed */
	if (bpw != priv->xfer_bpw) {
		spicc_setbits(priv, SPICC_CONREG, SPICC_BITLENGTH_MASK,
			      FIELD_PREP(SPICC_BITLENGTH_MASK, bpw - 1));
		priv->xfer_bpw = bpw;
	}

	if (flags & SPI_XFER_BEGIN) {
		spicc_pins_active(bus);
		if (priv->active_cs)
			dm_gpio_set_value(priv->active_cs, 1);
	}

	/*
	 * Use DMA for 8-bit transfers >= 2*fifo_size words when buffers
	 * are available. DMA requires minimum 2 FIFO-fulls of data.
	 */
	if (Bpw * 8 == bpw && Bpw <= SPICC_DMA_WIDTH_BYTES &&
	    words >= priv->data->fifo_size * SPICC_DMA_MIN_FIFOS &&
	    priv->dma_tx_buf) {
		ret = meson_spicc_xfer_dma(dev, dout, din, words, Bpw);
		if (ret < 0) {
			/*
			 * Do NOT fall back to PIO with a full resend: part
			 * of the stream may already be on the wire, and a
			 * duplicate corrupts non-idempotent slaves (e.g. a
			 * display's RAMWR stream -- every byte after the
			 * splice point lands displaced). Reset the FIFO and
			 * report the error; the caller owns the retry.
			 */
			dev_err(bus, "DMA failed (%d), aborting transfer\n",
				ret);
			spicc_reset_fifo(priv);
		} else if (ret < (int)words) {
			/* DMA transferred partial, finish remainder via PIO */
			unsigned int done = ret;
			const u8 *tx_rem = dout ? (const u8 *)dout + (size_t)done * Bpw : NULL;
			u8 *rx_rem = din ? (u8 *)din + (size_t)done * Bpw : NULL;

			spicc_reset_fifo(priv);
			ret = meson_spicc_xfer_pio(dev, tx_rem, rx_rem,
						   words - done, bpw, Bpw);
		} else {
			ret = 0;
		}
	} else {
		ret = meson_spicc_xfer_pio(dev, dout, din, words, bpw, Bpw);
	}

	if (flags & SPI_XFER_END) {
		if (priv->active_cs)
			dm_gpio_set_value(priv->active_cs, 0);
		spicc_pins_idle(bus);
	}

	return ret;
}

/* --- Probe and platform data --- */

static int meson_spicc_probe(struct udevice *bus)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);
	int ret;

	priv->base = dev_read_addr_ptr(bus);
	if (!priv->base)
		return -EINVAL;

	priv->data = (const struct meson_spicc_data *)dev_get_driver_data(bus);
	if (!priv->data)
		return -EINVAL;

	ret = clk_get_by_name(bus, "core", &priv->core_clk);
	if (ret) {
		dev_err(bus, "failed to get core clock\n");
		return ret;
	}

	ret = clk_enable(&priv->core_clk);
	if (ret) {
		dev_err(bus, "failed to enable core clock\n");
		return ret;
	}

	priv->pclk_enabled = false;
	priv->pclk_rate = 0;
	if (priv->data->has_pclk) {
		ret = clk_get_by_name(bus, "pclk", &priv->pclk);
		if (!ret) {
			ret = clk_enable(&priv->pclk);
			if (!ret) {
				priv->pclk_enabled = true;
				priv->pclk_rate = clk_get_rate(&priv->pclk);
			}
		}

		if (!priv->pclk_enabled) {
			void __iomem *hhi = (void __iomem *)G12A_HHI_SPICC_CLK_CNTL;
			u32 val;

			val = readl(hhi);
			val &= ~(SPICC1_SCLK_MUX_MASK | SPICC1_SCLK_DIV_MASK |
				  SPICC1_SCLK_GATE);
			val |= (SPICC1_SCLK_MUX_FCLK_DIV4 << SPICC1_SCLK_MUX_SHIFT) |
			       SPICC1_SCLK_GATE;
			writel(val, hhi);
			priv->pclk_rate = SPICC1_SCLK_FCLK_DIV4_HZ;
			dev_dbg(bus, "pclk: direct fclk_div4 setup, rate %lu\n",
				priv->pclk_rate);
		}
	}

	/* Allocate DMA padded buffers (8 bytes per word) */
	priv->dma_buf_size = SPICC_DMA_MAX_BYTES * SPICC_DMA_WIDTH_BYTES;
	priv->dma_tx_buf = memalign(64, priv->dma_buf_size);
	priv->dma_rx_buf = memalign(64, priv->dma_buf_size);
	if (!priv->dma_tx_buf || !priv->dma_rx_buf) {
		dev_warn(bus, "DMA buffer alloc failed, DMA disabled\n");
		free(priv->dma_tx_buf);
		free(priv->dma_rx_buf);
		priv->dma_tx_buf = NULL;
		priv->dma_rx_buf = NULL;
		priv->dma_buf_size = 0;
	}

	/*
	 * Request EVERY chip select the DT lists, not just index 0. The count
	 * bounds how many slaves this bus can address with a GPIO CS; a board
	 * with no cs-gpios keeps num_cs_gpios == 0 and the hardware SS logic,
	 * exactly as before.
	 */
	ret = gpio_get_list_count(bus, "cs-gpios");
	if (ret > 0) {
		unsigned int i, count = min_t(unsigned int, ret, SPICC_MAX_CS);

		if ((unsigned int)ret > SPICC_MAX_CS)
			dev_warn(bus, "cs-gpios lists %d entries, using %u\n",
				 ret, count);

		for (i = 0; i < count; i++) {
			int gret = gpio_request_by_name(bus, "cs-gpios", i,
							&priv->cs_gpios[i],
							GPIOD_IS_OUT);

			/* A hole in the list (<0>) is legal: leave it invalid */
			if (gret && gret != -ENOENT) {
				dev_err(bus, "failed to get cs-gpios[%u]: %d\n",
					i, gret);
				return gret;
			}
		}
		priv->num_cs_gpios = count;
	} else if (ret < 0 && ret != -ENOENT && ret != -EINVAL) {
		dev_err(bus, "failed to count cs-gpios: %d\n", ret);
		return ret;
	}

	/* Detect LBC bit positions before clearing registers */
	spicc_detect_lbc(bus);

	/* Clear all registers to known state */
	spicc_write(priv, SPICC_CONREG, 0);
	spicc_write(priv, SPICC_INTREG, 0);
	spicc_write(priv, SPICC_DMAREG, 0);
	spicc_write(priv, SPICC_TESTREG, 0);

	spicc_oen_enable(priv);

	priv->mode = SPI_MODE_0;
	priv->speed_hz = 1000000;
	priv->last_speed_hz = 0;
	priv->clk_ns = 1000;

	return 0;
}

static int meson_spicc_remove(struct udevice *bus)
{
	struct meson_spicc_priv *priv = dev_get_priv(bus);

	free(priv->dma_tx_buf);
	free(priv->dma_rx_buf);

	return 0;
}

/*
 * Honour the framework's word-length request instead of forcing 8 bits.
 *
 * The SPICC releases SCLK at every WORD boundary, so the word width the
 * controller is programmed with is visible on the wire: a 16-bit unit sent
 * as two 8-bit words has an extra intra-unit release that the same unit
 * sent as one 16-bit word does not. Slaves that count SCLK edges without
 * re-framing on CS -- the MHS3528 SPI-to-16-bit-parallel converter behind
 * the ILI9486 -- are sensitive to exactly that. Linux drives this panel
 * correctly using bpw 16 for pixel data (drivers/gpu/drm/tiny/ili9486.c),
 * while this driver previously hardcoded 8.
 */
static int meson_spicc_set_wordlen(struct udevice *dev, unsigned int wordlen)
{
	struct dm_spi_slave_plat *slave = dev_get_parent_plat(dev);

	/* CONREG.BITLENGTH is 6 bits: 1..64 */
	if (wordlen < 1 || wordlen > 64)
		return -EINVAL;

	/*
	 * Stored per slave; meson_spicc_xfer() programs CONREG from it on
	 * each transfer. The SPI uclass does not keep this for us (unlike
	 * speed/mode, which it re-applies on every claim).
	 */
	slave->wordlen = wordlen;

	return 0;
}

static const struct dm_spi_ops meson_spicc_ops = {
	.claim_bus	= meson_spicc_claim_bus,
	.release_bus	= meson_spicc_release_bus,
	.xfer		= meson_spicc_xfer,
	.set_speed	= meson_spicc_set_speed,
	.set_mode	= meson_spicc_set_mode,
	.set_wordlen	= meson_spicc_set_wordlen,
};

static const struct meson_spicc_data meson_spicc_gx_data = {
	.max_speed_hz		= 41666666,
	.min_speed_hz		= 325521,
	.fifo_size		= 16,
	.dma_bursts_max		= 64,
	.has_oen		= false,
	.has_enhance_clk_div	= false,
	.has_pclk		= false,
};

static const struct meson_spicc_data meson_spicc_axg_data = {
	.max_speed_hz		= 83333332,
	.min_speed_hz		= 325000,
	.fifo_size		= 16,
	.dma_bursts_max		= 65535,
	.has_oen		= true,
	.has_enhance_clk_div	= true,
	.has_pclk		= false,
};

static const struct meson_spicc_data meson_spicc_g12a_data = {
	.max_speed_hz		= 166666664,
	.min_speed_hz		= 50000,
	.fifo_size		= 15,	/* 15 x 64-bit entries; 16 causes stalls */
	.dma_bursts_max		= 65535,
	.has_oen		= true,
	.has_enhance_clk_div	= true,
	.has_pclk		= true,
};

static const struct udevice_id meson_spicc_ids[] = {
	{ .compatible = "amlogic,meson-gx-spicc",
	  .data = (ulong)&meson_spicc_gx_data },
	{ .compatible = "amlogic,meson-axg-spicc",
	  .data = (ulong)&meson_spicc_axg_data },
	{ .compatible = "amlogic,meson-g12a-spicc",
	  .data = (ulong)&meson_spicc_g12a_data },
	{ }
};

U_BOOT_DRIVER(meson_spicc) = {
	.name		= "meson_spicc",
	.id		= UCLASS_SPI,
	.of_match	= meson_spicc_ids,
	.ops		= &meson_spicc_ops,
	.probe		= meson_spicc_probe,
	.remove		= meson_spicc_remove,
	.priv_auto	= sizeof(struct meson_spicc_priv),
};
