// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Carlo Caione <carlo@caione.org>
 */

#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <fdtdec.h>
#include <log.h>
#include <malloc.h>
#include <pwrseq.h>
#include <mmc.h>
#if CONFIG_IS_ENABLED(DM_REGULATOR)
#include <power/regulator.h>
#endif
#include <spl.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/log2.h>
#include "meson_gx_mmc.h"

struct meson_gx_mmc_version_data meson_gx_mmc_version[] = {
	[MMC_COMPATIBLE_V2] = {
		.clk_always_on = BIT(24),
		.adjust = MESON_SD_EMMC_ADJUST,
	},
	[MMC_COMPATIBLE_V3] = {
		.clk_always_on = BIT(28),
		.adjust = MESON_SD_EMMC_V3_ADJUST,
	},
};

static inline void *get_regbase(const struct mmc *mmc)
{
	struct meson_mmc_plat *pdata = mmc->priv;

	return pdata->regbase;
}

static inline uint32_t meson_read(struct mmc *mmc, int offset)
{
	return readl(get_regbase(mmc) + offset);
}

static inline void meson_write(struct mmc *mmc, uint32_t val, int offset)
{
	writel(val, get_regbase(mmc) + offset);
}

/*
 * The first, SRAM-resident stage runs at 6 MHz (see probe) and has no room to
 * spare; a compile-time constant, so what it guards drops out of that stage.
 */
static inline bool meson_mmc_sram_stage(void)
{
	return xpl_phase() == PHASE_TPL ||
	       (xpl_phase() == PHASE_SPL && !IS_ENABLED(CONFIG_TPL));
}

/* Modes whose sample point the tuning sets; every other mode runs untuned. */
static bool meson_mmc_mode_is_tuned(enum bus_mode mode)
{
	return mode == MMC_HS_200 || mode == MMC_HS_400 ||
	       mode == MMC_HS_400_ES || mode == UHS_SDR104 || mode == UHS_SDR50;
}

static void meson_mmc_config_clock(struct mmc *mmc)
{
	struct meson_mmc_plat *pdata = mmc->priv;
	uint32_t meson_mmc_clk = 0;
	unsigned int clk, clk_src, clk_div;

	if (!mmc->clock)
		return;

	/*
	 * A board whose untuned sample point fails below the DT ceiling sets
	 * "amlogic,untuned-max-frequency": the ceiling then serves the tuned
	 * modes, and an untuned mode (HS52, SD HS, ...) stays at the cap, so a
	 * card that does not tune falls back to HS52 at the cap rather than
	 * failing HS52 and dropping to HS 26 MHz. mmc->clock follows, so
	 * "mmc info" reports the clock the bus runs at.
	 */
	if (!meson_mmc_sram_stage() && pdata->untuned_max &&
	    mmc->clock > pdata->untuned_max &&
	    !meson_mmc_mode_is_tuned(mmc->selected_mode))
		mmc->clock = pdata->untuned_max;

	/*
	 * An untuned mode samples at the controller's default point. Tuning
	 * sets ADJ_EN and a delay in ADJUST, and nothing else clears them, so
	 * a card stepped down from HS200 -- by this stage, or by an earlier
	 * one that tuned on the same controller (the SPL flash session) --
	 * would be sampled at the HS200 point: AML-S905D3-CC proper then read
	 * its eMMC wrong, silently, and found no EFI system partition.
	 */
	if (!meson_mmc_sram_stage() &&
	    !meson_mmc_mode_is_tuned(mmc->selected_mode)) {
		uint32_t adj = meson_read(mmc, pdata->version->adjust);

		if (adj & (ADJUST_ADJ_EN | ADJUST_ADJ_DELAY_MASK))
			meson_write(mmc, adj & ~(ADJUST_ADJ_EN | ADJUST_ADJ_DELAY_MASK),
				    pdata->version->adjust);
	}

	/*
	 * DDR samples on both edges, so the module clock is twice the bus
	 * clock. CFG_DDR (set by set_ios before this runs) then halves what
	 * the card sees. Proper only: the GXL SPL is at its size ceiling.
	 */
#ifndef CONFIG_XPL_BUILD
	if (mmc->ddr_mode)
		mmc->clock <<= 1;
#endif

	/* Clk always on */
	meson_mmc_clk |= pdata->version->clk_always_on;
	meson_mmc_clk |= CLK_CO_PHASE_180;
	meson_mmc_clk |= CLK_TX_PHASE_000;

	/* 1GHz / CLK_MAX_DIV = 15,9 MHz */
	if (mmc->clock > 16000000) {
		clk = SD_EMMC_CLKSRC_DIV2;
		clk_src = CLK_SRC_DIV2;
	} else {
		clk = SD_EMMC_CLKSRC_24M;
		clk_src = CLK_SRC_24M;
	}
	clk_div = DIV_ROUND_UP(clk, mmc->clock);

#ifndef CONFIG_XPL_BUILD
	if (mmc->ddr_mode)
		mmc->clock >>= 1;
#endif

	/* clock settings */
	meson_mmc_clk |= clk_src;
	meson_mmc_clk |= clk_div;

	meson_write(mmc, meson_mmc_clk, MESON_SD_EMMC_CLOCK);
}

/*
 * Any stage with the regulator framework, a DRAM-stage SPL included (its flash
 * session runs SD UHS); a stage without SPL_DM_REGULATOR compiles this out.
 */
#if CONFIG_IS_ENABLED(DM_MMC) && CONFIG_IS_ENABLED(DM_REGULATOR)
static int meson_mmc_set_vqmmc(struct mmc *mmc)
{
	int uv, cur, ret;

	if (!mmc->vqmmc_supply)
		return 0;

	switch (mmc->signal_voltage) {
	case MMC_SIGNAL_VOLTAGE_330:
		uv = 3300000;
		break;
	case MMC_SIGNAL_VOLTAGE_180:
		uv = 1800000;
		break;
	default:
		return 0;
	}

	cur = regulator_get_value(mmc->vqmmc_supply);
	ret = regulator_set_value(mmc->vqmmc_supply, uv);
	/*
	 * Fixed eMMC vqmmc has no set_value (-ENOSYS). A gpio rail returns
	 * -EINVAL for a step it does not have. Either one means leave the
	 * rail where it is, which is what set_ios used to do.
	 */
	if (ret == -EINVAL || ret == -ENOSYS)
		return 0;
	if (ret)
		return ret;

	/* vddio_card settling: 200 us up to 1.8 V, 50 ms down to 3.3 V. */
	if (cur != uv) {
		if (uv == 3300000)
			udelay(50000);
		else
			udelay(200);
	}

	return 0;
}
#endif

#if CONFIG_IS_ENABLED(DM_MMC)
static int meson_dm_mmc_set_ios(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else /* U-Boot SPL */
static int meson_legacy_mmc_set_ios(struct mmc *mmc)
{
#endif
	uint32_t meson_mmc_cfg;

#ifndef CONFIG_XPL_BUILD
	/*
	 * Program CFG_DDR before the clock divider. The later bus-width
	 * update preserves bit 2. A zero clock leaves the bit alone, matching
	 * the clock function's early return.
	 */
	if (mmc->clock) {
		/*
		 * SD_EMMC_CFG (0x44) cannot be accessed while SD_EMMC_CLOCK
		 * (0x00) is 0 -- no source, no divider (BSP v2019
		 * arch/arm/include/asm/arch-g12a/sd_emmc.h:18, :74): the read
		 * faults (measured on GXL, see meson_mmc_probe()).  On SM1
		 * with eMMC as the boot device U-Boot proper stopped inside
		 * initr_mmc(), before any mmc_init(), and the watchdog reset
		 * it; this read is the first register access of that probe
		 * after the clock gates.  SD_EMMC_CLOCK is 0 on a controller
		 * no earlier stage has clocked, e.g. the SD slot on an eMMC
		 * boot, where neither the BootROM nor the TPL touch it and the
		 * SPL flash session that clocks both controllers on a USB boot
		 * does not run.  Clock it first.  With no clock the card has seen no edge, so the
		 * CFG_DDR-before-divider ordering below protects nothing yet.
		 * SD_EMMC_CLOCK itself reads 0 without a fault.
		 */
		if (!meson_read(mmc, MESON_SD_EMMC_CLOCK))
			meson_mmc_config_clock(mmc);
		meson_mmc_cfg = meson_read(mmc, MESON_SD_EMMC_CFG);
		if (mmc->ddr_mode)
			meson_mmc_cfg |= CFG_DDR;
		else
			meson_mmc_cfg &= ~CFG_DDR;
		meson_write(mmc, meson_mmc_cfg, MESON_SD_EMMC_CFG);
	}
#endif

	meson_mmc_config_clock(mmc);

	meson_mmc_cfg = meson_read(mmc, MESON_SD_EMMC_CFG);

	meson_mmc_cfg &= ~CFG_BUS_WIDTH_MASK;
	if (mmc->bus_width == 1)
		meson_mmc_cfg |= CFG_BUS_WIDTH_1;
	else if (mmc->bus_width == 4)
		meson_mmc_cfg |= CFG_BUS_WIDTH_4;
	else if (mmc->bus_width == 8)
		meson_mmc_cfg |= CFG_BUS_WIDTH_8;
	else
		return -EINVAL;

	/* 512 bytes block length */
	meson_mmc_cfg &= ~CFG_BL_LEN_MASK;
	meson_mmc_cfg |= CFG_BL_LEN_512;

	/* Response timeout 256 clk */
	meson_mmc_cfg &= ~CFG_RESP_TIMEOUT_MASK;
	meson_mmc_cfg |= CFG_RESP_TIMEOUT_256;

	/* Command-command gap 16 clk */
	meson_mmc_cfg &= ~CFG_RC_CC_MASK;
	meson_mmc_cfg |= CFG_RC_CC_16;

	meson_write(mmc, meson_mmc_cfg, MESON_SD_EMMC_CFG);

#if CONFIG_IS_ENABLED(DM_MMC) && CONFIG_IS_ENABLED(DM_REGULATOR)
	{
		int ret = meson_mmc_set_vqmmc(mmc);

		if (ret)
			return ret;
	}
#endif

	return 0;
}

#if CONFIG_IS_ENABLED(MMC_SUPPORTS_TUNING)
static int meson_mmc_execute_tuning(struct udevice *dev, uint opcode)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct meson_mmc_plat *pdata = mmc->priv;
	unsigned int max_dly, i, clk_src, best, best_len;
	uint32_t orig, val, pass, adj = pdata->version->adjust;
	int ret;

	if (!mmc->clock)
		return -EINVAL;

	clk_src = (mmc->clock > 16000000) ? SD_EMMC_CLKSRC_DIV2 :
		  SD_EMMC_CLKSRC_24M;
	max_dly = DIV_ROUND_UP(clk_src, mmc->clock);
	if (!max_dly)
		max_dly = 1;

	/* The delays span one clock period; at most 32 are scanned. */
	if (max_dly > 32)
		max_dly = 32;

	orig = meson_read(mmc, adj);
	val = orig | ADJUST_ADJ_EN;
	meson_write(mmc, val, adj);

	/*
	 * Try every delay and keep the middle of the widest run that passes,
	 * not the first delay that passes: that one sits at the edge of the
	 * window, where reads that pass the tuning block can still fail
	 * (AML-S905D3-CC SD at SDR104: the first pass was delay 0, and the
	 * session's reads were slow and then hung). The run may wrap from the
	 * last delay to the first.
	 */
	pass = 0;
	for (i = 0; i < max_dly; i++) {
		val &= ~ADJUST_ADJ_DELAY_MASK;
		val |= (i << ADJUST_ADJ_DELAY_SHIFT) & ADJUST_ADJ_DELAY_MASK;
		meson_write(mmc, val, adj);

		ret = mmc_send_tuning(mmc, opcode);
		if (!ret)
			pass |= BIT(i);
	}

	if (!pass) {
		printf("MMCTUNE op=%u clk=%u none of %u\n", opcode, mmc->clock,
		       max_dly);
		/*
		 * No delay passed the tuning block: restore the pre-tuning
		 * ADJUST word and fail, so the MMC core reverts and tries the
		 * next mode/width row. A tuned mode is never kept on an
		 * untuned sample point.
		 */
		meson_write(mmc, orig, adj);
		return -EIO;
	}

	/*
	 * Every delay passing shows no edge, so there is no middle to find:
	 * keep delay 0, as before. (AML-S905D3-CC eMMC at HS200 4-bit passes
	 * all five; reads were right at delay 0 and failed at delay 2.)
	 * Otherwise: the middle of the widest passing run, which may wrap.
	 */
	if (hweight32(pass) == max_dly) {
		i = 0;
	} else {
		best = 0;
		best_len = 0;
		for (i = 0; i < max_dly; i++) {
			unsigned int len = 0;

			/* a run starts where the delay before it failed */
			if (!(pass & BIT(i)) ||
			    (pass & BIT((i + max_dly - 1) % max_dly)))
				continue;
			while (len < max_dly && (pass & BIT((i + len) % max_dly)))
				len++;
			if (len > best_len) {
				best_len = len;
				best = i;
			}
		}
		i = (best + (best_len - 1) / 2) % max_dly;
	}

	val &= ~ADJUST_ADJ_DELAY_MASK;
	val |= (i << ADJUST_ADJ_DELAY_SHIFT) & ADJUST_ADJ_DELAY_MASK;
	meson_write(mmc, val, adj);
	/* no '#' flag: the SPL's tiny printf does not take it */
	printf("MMCTUNE op=%u clk=%u pass=0x%x of %u dly=%u\n", opcode,
	       mmc->clock, pass, max_dly, i);
	return 0;
}
#endif

static void meson_mmc_setup_cmd(struct mmc *mmc, struct mmc_data *data,
				struct mmc_cmd *cmd)
{
	uint32_t meson_mmc_cmd = 0, cfg;

	meson_mmc_cmd |= cmd->cmdidx << CMD_CFG_CMD_INDEX_SHIFT;

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136)
			meson_mmc_cmd |= CMD_CFG_RESP_128;

		if (cmd->resp_type & MMC_RSP_BUSY)
			meson_mmc_cmd |= CMD_CFG_R1B;

		if (!(cmd->resp_type & MMC_RSP_CRC))
			meson_mmc_cmd |= CMD_CFG_RESP_NOCRC;
	} else {
		meson_mmc_cmd |= CMD_CFG_NO_RESP;
	}

	if (data) {
		cfg = meson_read(mmc, MESON_SD_EMMC_CFG);
		cfg &= ~CFG_BL_LEN_MASK;
		cfg |= ilog2(data->blocksize) << CFG_BL_LEN_SHIFT;
		meson_write(mmc, cfg, MESON_SD_EMMC_CFG);

		if (data->flags == MMC_DATA_WRITE)
			meson_mmc_cmd |= CMD_CFG_DATA_WR;

		meson_mmc_cmd |= CMD_CFG_DATA_IO | CMD_CFG_BLOCK_MODE |
				 data->blocks;
	}

	meson_mmc_cmd |= CMD_CFG_TIMEOUT_4S | CMD_CFG_OWNER |
			 CMD_CFG_END_OF_CHAIN;

	meson_write(mmc, meson_mmc_cmd, MESON_SD_EMMC_CMD_CFG);
}

static void meson_mmc_setup_addr(struct mmc *mmc, struct mmc_data *data)
{
	struct meson_mmc_plat *pdata = mmc->priv;
	unsigned int data_size;
	uint32_t data_addr = 0;

	if (data) {
		data_size = data->blocks * data->blocksize;

		if (data->flags == MMC_DATA_READ) {
			data_addr = (ulong) data->dest;
			invalidate_dcache_range(data_addr,
						data_addr + data_size);
		} else {
			pdata->w_buf = calloc(data_size, sizeof(char));
			data_addr = (ulong) pdata->w_buf;
			memcpy(pdata->w_buf, data->src, data_size);
			flush_dcache_range(data_addr, data_addr + data_size);
		}
	}

	meson_write(mmc, data_addr, MESON_SD_EMMC_CMD_DAT);
}

static void meson_mmc_read_response(struct mmc *mmc, struct mmc_cmd *cmd)
{
	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[0] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP3);
		cmd->response[1] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP2);
		cmd->response[2] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP1);
		cmd->response[3] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP);
	} else {
		cmd->response[0] = meson_read(mmc, MESON_SD_EMMC_CMD_RSP);
	}
}

#if CONFIG_IS_ENABLED(DM_MMC)
static int meson_dm_mmc_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
				 struct mmc_data *data)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else /* U-Boot SPL */
static int meson_legacy_mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
				     struct mmc_data *data)
{
#endif
	struct meson_mmc_plat *pdata = mmc->priv;
	uint32_t status;
	ulong start;
	int ret = 0;

	/* max block size supported by chip is 512 byte */
	if (data && data->blocksize > 512)
		return -EINVAL;

	meson_mmc_setup_cmd(mmc, data, cmd);
	meson_mmc_setup_addr(mmc, data);

	meson_write(mmc, cmd->cmdarg, MESON_SD_EMMC_CMD_ARG);

	/* use 10s timeout */
	start = get_timer(0);
	do {
		status = meson_read(mmc, MESON_SD_EMMC_STATUS);
	} while(!(status & STATUS_END_OF_CHAIN) && get_timer(start) < 10000);

	if (!(status & STATUS_END_OF_CHAIN))
		ret = -ETIMEDOUT;
	else if (status & STATUS_RESP_TIMEOUT)
		ret = -ETIMEDOUT;
	else if (status & STATUS_ERR_MASK)
		ret = -EIO;

	meson_mmc_read_response(mmc, cmd);

	if (data && data->flags == MMC_DATA_WRITE)
		free(pdata->w_buf);

	/* reset status bits */
	meson_write(mmc, STATUS_MASK, MESON_SD_EMMC_STATUS);

	return ret;
}

#if !CONFIG_IS_ENABLED(DM_MMC) /* Non-DM MMC driver for use in U-Boot SPL */
struct meson_mmc_plat mmc_plat[2];

static int meson_legacy_mmc_init(struct mmc *mmc)
{
	/* reset all status bits */
	meson_write(mmc, STATUS_MASK, MESON_SD_EMMC_STATUS);

	/* disable interrupts */
	meson_write(mmc, 0, MESON_SD_EMMC_IRQ_EN);

	return 0;
}

static const struct mmc_ops meson_mmc_ops = {
	.send_cmd	= meson_legacy_mmc_send_cmd,
	.set_ios	= meson_legacy_mmc_set_ios,
	.init		= meson_legacy_mmc_init,
};

struct mmc *meson_mmc_init(int mmc_no)
{
	struct meson_mmc_plat *pdata = &mmc_plat[mmc_no];
	struct mmc_config *cfg = &pdata->cfg;

	cfg->voltages = MMC_VDD_33_34 | MMC_VDD_32_33 |
			MMC_VDD_31_32 | MMC_VDD_165_195;
	cfg->host_caps = MMC_MODE_8BIT | MMC_MODE_4BIT;
	cfg->f_min = DIV_ROUND_UP(SD_EMMC_CLKSRC_24M, CLK_MAX_DIV);
	cfg->f_max = 6000000; /* 6 MHz */
	cfg->b_max = 127; /* max 128 - 1 block */
	cfg->name = "Meson SD/eMMC";
	cfg->ops = &meson_mmc_ops;

	if (mmc_no == 0) /* MMC1: SD card */
		pdata->regbase = (void *)0xd0072000;
	else if (mmc_no == 1) /* MMC2: eMMC */
		pdata->regbase = (void *)0xd0074000;

#if CONFIG_IS_ENABLED(MMC_PWRSEQ)
	/* Enable power if needed */
	ret = mmc_pwrseq_get_power(dev, cfg);
	if (!ret) {
		ret = pwrseq_set_power(cfg->pwr_dev, true);
		if (ret)
			return ret;
	}
#endif

	return mmc_create(cfg, pdata);
}

#else /* DM-based driver for use in U-Boot proper */
static const struct dm_mmc_ops meson_dm_mmc_ops = {
	.send_cmd = meson_dm_mmc_send_cmd,
	.set_ios = meson_dm_mmc_set_ios,
#if CONFIG_IS_ENABLED(MMC_SUPPORTS_TUNING)
	.execute_tuning = meson_mmc_execute_tuning,
#endif
};

static int meson_mmc_of_to_plat(struct udevice *dev)
{
	enum meson_gx_mmc_compatible compat = dev_get_driver_data(dev);
	struct meson_mmc_plat *pdata = dev_get_plat(dev);
	fdt_addr_t addr;

	addr = dev_read_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	pdata->regbase = (void *)addr;
	pdata->version = &meson_gx_mmc_version[compat];

	return 0;
}

static int meson_mmc_probe(struct udevice *dev)
{
	struct meson_mmc_plat *pdata = dev_get_plat(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct mmc *mmc = &pdata->mmc;
	struct mmc_config *cfg = &pdata->cfg;
	struct clk_bulk clocks;
	uint32_t val;
	int ret;

	/*
	 * Enable the clocks feeding the MMC controller.
	 *
	 * A phase built without the CLK uclass does not get a failed lookup,
	 * it gets -ENOSYS from the <clk.h> stub (clk_get_bulk() there is a
	 * static inline returning it unconditionally).  In TPL that is
	 * survivable and deliberate: the BootROM has already run this very
	 * controller to fetch TPL itself, so every gate above it is on, and
	 * meson_mmc_config_clock() below programs SD_EMMC_CLOCK with a plain
	 * meson_write() -- no clock framework is involved in the one register
	 * that actually has to move.  Measured on AML-S905X-CC 2026-08-07 with
	 * the board sitting in BROM: HHI_GCLK_MPEG0 = 0xffffffff (SD_EMMC_A/B/C
	 * gates set), HHI_NAND_CLK_CNTL = 0x80, HHI_SD_EMMC_CLK_CNTL =
	 * 0x00800080, while the controller's own SD_EMMC_CLOCK read 0 and
	 * SD_EMMC_CFG faulted -- and writing SD_EMMC_CLOCK directly, with
	 * nothing else, made SD_EMMC_CFG read 0x00004890.
	 *
	 * clk_enable_bulk() already takes exactly this view of -ENOSYS from
	 * clk_enable(), so the policy is not new; what is new is applying it
	 * to the phase where the uclass is absent altogether.
	 *
	 * Scoped to TPL rather than to "any phase without CLK" on purpose.
	 * The generic form is what one would want upstream, but it un-deads
	 * the rest of this probe in the SPL phase too, and the GXL open-chain
	 * single-stage SPL has 21 bytes of headroom under its 0xb000 amlimage
	 * ceiling: measured, it overflows by 0xc3.  A phase that has no CLK
	 * and no prior stage to have left the gates on would also be trusting
	 * something nobody set up.
	 */
	ret = clk_get_bulk(dev, &clocks);
	if (!ret) {
		ret = clk_enable_bulk(&clocks);
		if (ret)
			return ret;
	} else if (!(ret == -ENOSYS && xpl_phase() == PHASE_TPL)) {
		return ret;
	}

	cfg->voltages = MMC_VDD_33_34 | MMC_VDD_32_33 |
			MMC_VDD_31_32 | MMC_VDD_165_195;
	/* bus width comes from the DT "bus-width" via mmc_of_parse() only */
	cfg->host_caps = SD_HS | MMC_MODE_HS_52MHz | MMC_MODE_HS;
	cfg->f_min = DIV_ROUND_UP(SD_EMMC_CLKSRC_24M, CLK_MAX_DIV);
	cfg->f_max = 40000000; /* default if DT has no max-frequency */
	cfg->b_max = 511; /* max 512 - 1 blocks */
	cfg->name = dev->name;

	ret = mmc_of_parse(dev, cfg);
	if (ret)
		return ret;

	if (!meson_mmc_sram_stage())
		pdata->untuned_max = dev_read_u32_default(dev, "amlogic,untuned-max-frequency", 0);

	/*
	 * GXL (adjust @ 0x8): offering HS200 at probe made the first mmc_init
	 * fail and the eMMC disappear, so the cap is masked here. It is not the
	 * last word: mmc_startup() (23ab756c4fc) puts HS200 back at mode
	 * select whenever MMC_HS200_SUPPORT is built, and AML-S905X-CC-V2
	 * proper then tunes its eMMC at HS200 200 MHz. A GXL build without
	 * MMC_HS200_SUPPORT stays at HS52.
	 */
	if (pdata->version->adjust == MESON_SD_EMMC_ADJUST)
		cfg->host_caps &= ~MMC_CAP(MMC_HS_200);

	/*
	 * The 6 MHz legacy limit is for the first, SRAM-resident boot stage,
	 * which runs on the clocks the BootROM left. A DRAM-stage SPL behind a
	 * TPL has the clock driver (SPL_CLK) and takes the DT's speed caps like
	 * proper: under the limit it reads eMMC at about 5 MB/s.
	 */
	if (xpl_phase() == PHASE_TPL ||
	    (xpl_phase() == PHASE_SPL && !IS_ENABLED(CONFIG_TPL))) {
		cfg->host_caps &= ~(MMC_MODE_HS_52MHz | MMC_MODE_HS);
		cfg->f_max = 6000000; /* 6 MHz */
		cfg->b_max = 127; /* max 128 - 1 block */
	}

	mmc->priv = pdata;
	upriv->mmc = mmc;

	mmc_set_clock(mmc, cfg->f_min, MMC_CLK_ENABLE);

#if CONFIG_IS_ENABLED(MMC_PWRSEQ)
	/* Enable power if needed */
	ret = mmc_pwrseq_get_power(dev, cfg);
	if (!ret) {
		ret = pwrseq_set_power(cfg->pwr_dev, true);
		if (ret)
			return ret;
	}
#endif

	/* reset all status bits */
	meson_write(mmc, STATUS_MASK, MESON_SD_EMMC_STATUS);

	/* disable interrupts */
	meson_write(mmc, 0, MESON_SD_EMMC_IRQ_EN);

	/* enable auto clock mode */
	val = meson_read(mmc, MESON_SD_EMMC_CFG);
	val &= ~CFG_SDCLK_ALWAYS_ON;
	val |= CFG_AUTO_CLK;
	meson_write(mmc, val, MESON_SD_EMMC_CFG);

	return 0;
}

int meson_mmc_bind(struct udevice *dev)
{
	struct meson_mmc_plat *pdata = dev_get_plat(dev);

	return mmc_bind(dev, &pdata->mmc, &pdata->cfg);
}

static const struct udevice_id meson_mmc_match[] = {
	{ .compatible = "amlogic,meson-gx-mmc", .data = MMC_COMPATIBLE_V2 },
	{ .compatible = "amlogic,meson-axg-mmc", .data = MMC_COMPATIBLE_V3 },
	{ .compatible = "amlogic,meson-sm1-mmc", .data = MMC_COMPATIBLE_V3 },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(meson_mmc) = {
	.name = "meson_gx_mmc",
	.id = UCLASS_MMC,
	.of_match = meson_mmc_match,
	.ops = &meson_dm_mmc_ops,
	.probe = meson_mmc_probe,
	.bind = meson_mmc_bind,
	.of_to_plat = meson_mmc_of_to_plat,
	.plat_auto	= sizeof(struct meson_mmc_plat),
};
#endif /* CONFIG_IS_ENABLED(DM_MMC) */
