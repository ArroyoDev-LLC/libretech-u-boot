// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2013 Google, Inc
 */

#include <clk.h>
#include <dm.h>
#include <dt-structs.h>
#include <dwmmc.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <mapmem.h>
#include <pwrseq.h>
#include <stdio.h>
#include <syscon.h>
#include <asm/gpio.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/io-domain.h>
#include <asm/arch-rockchip/periph.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/string.h>

struct rockchip_mmc_plat {
#if CONFIG_IS_ENABLED(OF_PLATDATA)
	struct dtd_rockchip_rk3288_dw_mshc dtplat;
#endif
	struct mmc_config cfg;
	struct mmc mmc;
};

struct rockchip_dwmmc_priv {
	struct clk clk;
	/*
	 * The CRU's per-controller drive and sample delay lines (Linux:
	 * "ciu-drive" / "ciu-sample"); optional -- without them the host
	 * stays at the modes that need no tuning.
	 */
	struct clk drv_clk;
	struct clk sample_clk;
	struct gpio_desc cd_gpio;
	struct dwmci_host host;
	int fifo_depth;
	bool fifo_mode;
	u32 minmax[2];
	int default_sample_phase;
	int num_phases;
	bool phase_capable;
	/*
	 * The card-clock fallback ladder's state for this card session: the
	 * rung it settled on (0 = none, the DT ceiling applies) and how many
	 * bus errors the block path has taken at that rung. Both go back to
	 * zero at the next re-init, which re-measures the eye from the top.
	 */
	uint clk_ceiling;
	int bus_errs;
};

/* The dw-mshc's CLKDIV must be 1, not 0, for eMMC DDR52 on an 8-bit bus */
static bool rockchip_dwmmc_doubled_clk(const struct mmc *mmc)
{
	return mmc->selected_mode == MMC_DDR_52 && mmc->bus_width == 8;
}

static uint rockchip_dwmmc_get_mmc_clk(struct dwmci_host *host, uint freq)
{
	struct udevice *dev = host->priv;
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	int ret;

	/*
	 * The clock frequency chosen here affects CLKDIV in the dw_mmc core.
	 * That can be either 0 or 1, but it must be set to 1 for eMMC DDR52
	 * 8-bit mode.  It will be set to 0 for all other modes.
	 */
	if (rockchip_dwmmc_doubled_clk(host->mmc))
		freq *= 2;

	ret = clk_set_rate(&priv->clk, freq);
	if (ret < 0) {
		debug("%s: err=%d\n", __func__, ret);
		return 0;
	}

	/*
	 * Always the request: the core computes CLKDIV from the difference
	 * between what it asked for and what this returns, so reporting a
	 * shortfall here would halve the card clock on top of it. The rate
	 * the CRU actually settled on is written back to mmc->clock by
	 * set_ios below, where `mmc info` reads it.
	 */
	return freq;
}

/*
 * The core's set_ios, with the ladder's ceiling applied to the request
 * before it and the achieved rate written back after it: mmc->clock is what
 * `mmc info` prints as Bus Speed and what the ladder measures its rungs
 * against, so it carries the card clock the CRU settled on (Linux's
 * actual_clock), not the number the core asked for -- a 200 MHz request that
 * the tree can only serve at 192 reads 192.
 */
static int rockchip_dwmmc_set_ios(struct udevice *dev)
{
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = priv->host.mmc;
	ulong achieved;
	int ret;

	if (priv->clk_ceiling && mmc->clock > priv->clk_ceiling)
		mmc->clock = priv->clk_ceiling;

	ret = dm_dwmci_ops.set_ios(dev);
	if (ret || !mmc->clock || mmc->clk_disable)
		return ret;

	achieved = clk_get_rate(&priv->clk);
	if (IS_ERR_VALUE(achieved) || !achieved)
		return 0;
	if (rockchip_dwmmc_doubled_clk(mmc))
		achieved /= 2;
	if (achieved != mmc->clock) {
		debug("%s: requested %u Hz, card clock %lu Hz\n", dev->name,
		      mmc->clock, achieved);
		mmc->clock = achieved;
	}

	return 0;
}

/*
 * Drive and sample phase per bus mode, the same choices as Linux's
 * dw_mci_rk3288_set_ios(): the drive phase is a hold-time offset that
 * depends only on the mode (90 degrees, 180 where the clock is fastest or
 * doubled), and the sample phase is the enumeration default for every mode
 * that is not tuned -- the tuned modes leave whatever execute_tuning() set.
 */
static int rockchip_dwmmc_clksel(struct dwmci_host *host)
{
	struct udevice *dev = host->priv;
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = host->mmc;
	int phase = 90;

	if (!priv->phase_capable || !mmc->clock)
		return 0;

	switch (mmc->selected_mode) {
	case MMC_DDR_52:
		/*
		 * The clock in rate is doubled for 8-bit DDR52, so double the
		 * phase offset to get the same timing.
		 */
		if (mmc->bus_width == 8)
			phase = 180;
		break;
	case UHS_SDR104:
	case MMC_HS_200:
		/*
		 * At the 150 MHz these run, a 90 degree offset is 1.67 ns of
		 * hold; 180 leaves margin over the .8 ns minimum.
		 */
		phase = 180;
		break;
	default:
		break;
	}

	if (clk_valid(&priv->drv_clk))
		clk_set_phase(&priv->drv_clk, phase);

	if (mmc->selected_mode != UHS_SDR104 &&
	    mmc->selected_mode != MMC_HS_200)
		clk_set_phase(&priv->sample_clk, priv->default_sample_phase);

	return 0;
}

#if CONFIG_IS_ENABLED(DM_REGULATOR)
static int rockchip_dwmmc_io_voltage(struct dwmci_host *host, int uV,
				     bool pre)
{
	struct mmc *mmc = host->mmc;

	return rockchip_iodomain_set_supply(mmc->vqmmc_supply, uV);
}
#endif

#if CONFIG_IS_ENABLED(MMC_SUPPORTS_TUNING)
#define TUNING_ITERATION_TO_PHASE(i, num_phases) \
		(DIV_ROUND_UP((i) * 360, num_phases))

/*
 * Card-clock fallback ladder for the tuned modes. The top rung is whatever
 * the core asks for (the DT max-frequency clamps it through f_max); the
 * rungs below are the SD spec's own bands -- 200 MHz (exact from CPLL on
 * RK3399), the 150 MHz rk3399-base.dtsi ceiling, SDR50's 100 and SDR25's
 * 50 -- as fixed rates rather than ratios, because the clock driver picks
 * the parent + divider at or below a request and a ratio lands wherever
 * that rounds, while these are rates the card is specified at. A sweep
 * whose widest window is narrower than the configured floor (or finds no
 * window at all) steps the clock down one rung and sweeps again; the
 * bottom rung accepts any open window. Each re-init starts from the top,
 * so a re-init re-measures the eye rather than remembering a rung. Linux's
 * dw_mmc-rockchip carries the same ladder plus a runtime CRC-storm step;
 * U-Boot has no re-tune path, so the tuning-time gate is the whole ladder.
 */
static const uint rockchip_dwmmc_clk_rungs[] = {
	200000000, 150000000, 100000000, 50000000,
};

/* The largest rung strictly below @clock, or 0 when the ladder is exhausted */
static uint rockchip_dwmmc_next_rung(uint clock)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rockchip_dwmmc_clk_rungs); i++)
		if (rockchip_dwmmc_clk_rungs[i] < clock)
			return rockchip_dwmmc_clk_rungs[i];
	return 0;
}

/*
 * Step the card clock down to @rung for the rest of this card session: the
 * ceiling keeps a later set_ios (a bus-width change, the core re-applying
 * tran_speed) from climbing back, and the error count starts afresh at the
 * new rung. mmc_set_clock() re-rates the CRU divider through set_ios.
 */
static int rockchip_dwmmc_set_rung(struct udevice *dev, uint rung)
{
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);

	priv->clk_ceiling = rung;
	priv->bus_errs = 0;

	return mmc_set_clock(priv->host.mmc, rung, MMC_CLK_ENABLE);
}

/*
 * Sweep the sample phase over a full turn, sending the tuning block at
 * each step, and settle on the middle of the longest passing window --
 * Linux's dw_mci_rk3288_execute_tuning(). @window receives that window's
 * width in degrees (360 when every phase passes, 0 when none does).
 */
static int rockchip_dwmmc_tuning_sweep(struct udevice *dev, uint opcode,
				       int *window)
{
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct dwmci_host *host = &priv->host;
	struct mmc *mmc = host->mmc;
	struct range_t {
		int start;
		int end; /* inclusive */
	} *ranges;
	int num_phases = priv->num_phases;
	int longest_range_len = -1, longest_range = -1;
	unsigned int range_count = 0;
	bool v, prev_v = false, first_v = false;
	int i, middle_phase, phase, ret = 0;

	*window = 0;

	if (!priv->phase_capable)
		return -ENOSYS;

	ranges = malloc((num_phases / 2 + 1) * sizeof(*ranges));
	if (!ranges)
		return -ENOMEM;

	/* Try each phase and extract good ranges */
	for (i = 0; i < num_phases; ) {
		clk_set_phase(&priv->sample_clk,
			      TUNING_ITERATION_TO_PHASE(i, num_phases));

		v = !mmc_send_tuning(mmc, opcode);

		if (i == 0)
			first_v = v;

		if (!prev_v && v) {
			range_count++;
			ranges[range_count - 1].start = i;
		}
		if (v) {
			ranges[range_count - 1].end = i;
			i++;
		} else if (i == num_phases - 1) {
			/* No extra skipping rules if we're at the end */
			i++;
		} else {
			/*
			 * No need to check too close to an invalid one since
			 * testing bad phases is slow.  Skip 20 degrees.
			 */
			i += DIV_ROUND_UP(20 * num_phases, 360);

			/* Always test the last one */
			if (i >= num_phases)
				i = num_phases - 1;
		}

		prev_v = v;
	}

	if (range_count == 0) {
		printf("%s: tuning: all sample phases bad\n", dev->name);
		ret = -EIO;
		goto free;
	}

	/* wrap around case, merge the end points */
	if (range_count > 1 && first_v && v) {
		ranges[0].start = ranges[range_count - 1].start;
		range_count--;
	}

	if (ranges[0].start == 0 && ranges[0].end == num_phases - 1) {
		clk_set_phase(&priv->sample_clk, priv->default_sample_phase);
		printf("%s: tuning: all phases work, using default phase %d\n",
		       dev->name, priv->default_sample_phase);
		*window = 360;
		goto free;
	}

	/* Find the longest range */
	for (i = 0; i < range_count; i++) {
		int len = ranges[i].end - ranges[i].start + 1;

		if (len < 0)
			len += num_phases;

		if (longest_range_len < len) {
			longest_range_len = len;
			longest_range = i;
		}

		debug("%s: good phase range %d-%d (%d len)\n", dev->name,
		      TUNING_ITERATION_TO_PHASE(ranges[i].start, num_phases),
		      TUNING_ITERATION_TO_PHASE(ranges[i].end, num_phases),
		      len);
	}

	middle_phase = ranges[longest_range].start + longest_range_len / 2;
	middle_phase %= num_phases;
	phase = TUNING_ITERATION_TO_PHASE(middle_phase, num_phases);

	printf("%s: tuned sample phase to %d (window %d-%d, %d of %d)\n",
	       dev->name, phase,
	       TUNING_ITERATION_TO_PHASE(ranges[longest_range].start,
					 num_phases),
	       TUNING_ITERATION_TO_PHASE(ranges[longest_range].end,
					 num_phases),
	       longest_range_len, num_phases);

	clk_set_phase(&priv->sample_clk, phase);
	*window = TUNING_ITERATION_TO_PHASE(longest_range_len, num_phases);

free:
	free(ranges);
	return ret;
}

/*
 * Tuning with the clock-fallback ladder: sweep at the clock the core set;
 * a closed eye or one narrower than the configured floor lowers the clock
 * one rung (mmc_set_clock() re-rates the CRU divider through set_ios) and
 * sweeps again; the bottom rung accepts any open eye. The clock the ladder
 * settles on is what `mmc info` reports as Bus Speed.
 */
static int rockchip_dwmmc_execute_tuning(struct udevice *dev, uint opcode)
{
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = priv->host.mmc;
	int min_window = CONFIG_MMC_DW_ROCKCHIP_TUNING_MIN_WINDOW;
	int window, ret;
	uint next;

	for (;;) {
		ret = rockchip_dwmmc_tuning_sweep(dev, opcode, &window);
		if (ret && ret != -EIO)
			return ret;

		next = rockchip_dwmmc_next_rung(mmc->clock);
		if (!next) {
			if (!ret && window < min_window)
				printf("%s: tuning window %d < %d at %u MHz, no lower rung, accepting\n",
				       dev->name, window, min_window,
				       mmc->clock / 1000000);
			return ret;
		}
		if (!ret && window >= min_window)
			return 0;

		if (ret)
			printf("%s: tuning: all phases bad at %u MHz, falling back to %u MHz\n",
			       dev->name, mmc->clock / 1000000, next / 1000000);
		else
			printf("%s: tuning window %d < %d at %u MHz, falling back to %u MHz\n",
			       dev->name, window, min_window,
			       mmc->clock / 1000000, next / 1000000);

		ret = rockchip_dwmmc_set_rung(dev, next);
		if (ret)
			return ret;
	}
}

static const char *rockchip_dwmmc_bus_error_name(u32 rintsts)
{
	if (rintsts & DWMCI_INTMSK_DCRC)
		return "data CRC";
	if (rintsts & DWMCI_INTMSK_DRTO)
		return "data timeout";
	if (rintsts & (DWMCI_INTMSK_SBE | DWMCI_INTMSK_EBE))
		return "data start/end bit";
	if (rintsts & DWMCI_INTMSK_RCRC)
		return "response CRC";
	if (rintsts & DWMCI_INTMSK_RTO)
		return "response timeout";
	return "response";
}

/*
 * The ladder's runtime arm, Linux's crc_storm -> retune -> lower_bus_clock
 * in U-Boot's terms: a block read / write that failed on the bus in a tuned
 * mode is re-tuned once at the clock it failed at (the sweep re-finds the
 * eye, and a phase that had drifted to its edge is re-centred), and a
 * second failure at the same rung steps the clock down one rung and re-tunes
 * there; the command is then re-issued. A multi-block transfer is stopped
 * first so the card is back in its transfer state for the tuning block.
 * Untuned modes, and a ladder with no rung left, hand the error back as it
 * was -- the core then fails the read exactly as it did before.
 */
static int rockchip_dwmmc_bus_error(struct dwmci_host *host,
				    struct mmc_cmd *cmd, struct mmc_data *data,
				    u32 rintsts)
{
	struct udevice *dev = host->priv;
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = host->mmc;
	uint opcode, next;
	int ret;

	if (!priv->phase_capable)
		return -EIO;

	switch (mmc->selected_mode) {
	case UHS_SDR50:
	case UHS_SDR104:
	case MMC_HS_200:
		break;
	default:
		return -EIO;
	}

	if (cmd->cmdidx == MMC_CMD_READ_MULTIPLE_BLOCK ||
	    cmd->cmdidx == MMC_CMD_WRITE_MULTIPLE_BLOCK) {
		struct mmc_cmd stop = {
			.cmdidx = MMC_CMD_STOP_TRANSMISSION,
			.resp_type = MMC_RSP_R1b,
		};

		/* The card may already be back in tran; either answer is fine */
		mmc_send_cmd(mmc, &stop, NULL);
	}

	opcode = IS_SD(mmc) ? MMC_CMD_SEND_TUNING_BLOCK :
			      MMC_CMD_SEND_TUNING_BLOCK_HS200;

	if (++priv->bus_errs == 1) {
		printf("%s: CMD%d %s error (RINTSTS 0x%x) at %u MHz, re-tuning\n",
		       dev->name, cmd->cmdidx,
		       rockchip_dwmmc_bus_error_name(rintsts), rintsts,
		       mmc->clock / 1000000);
		return rockchip_dwmmc_execute_tuning(dev, opcode);
	}

	next = rockchip_dwmmc_next_rung(mmc->clock);
	if (!next) {
		printf("%s: CMD%d %s error #%d at %u MHz, no lower rung\n",
		       dev->name, cmd->cmdidx,
		       rockchip_dwmmc_bus_error_name(rintsts), priv->bus_errs,
		       mmc->clock / 1000000);
		return -EIO;
	}

	printf("%s: CMD%d %s error #%d at %u MHz, falling back to %u MHz\n",
	       dev->name, cmd->cmdidx, rockchip_dwmmc_bus_error_name(rintsts),
	       priv->bus_errs, mmc->clock / 1000000, next / 1000000);

	ret = rockchip_dwmmc_set_rung(dev, next);
	if (ret)
		return ret;

	return rockchip_dwmmc_execute_tuning(dev, opcode);
}
#endif

/*
 * The core re-inits at `mmc dev`/`mmc rescan` and after a card power cycle;
 * a new session re-measures the eye from the DT ceiling, so the ladder's
 * rung and error count do not carry across (Linux: the clock ceiling is
 * cleared on MMC_POWER_OFF).
 */
static int rockchip_dwmmc_reinit(struct udevice *dev)
{
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);

	priv->clk_ceiling = 0;
	priv->bus_errs = 0;

	return dm_dwmci_ops.reinit(dev);
}

static int rockchip_dwmmc_of_to_plat(struct udevice *dev)
{
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct dwmci_host *host = &priv->host;

	if (!CONFIG_IS_ENABLED(OF_REAL))
		return 0;

	host->name = dev->name;
	host->ioaddr = dev_read_addr_ptr(dev);
	host->buswidth = dev_read_u32_default(dev, "bus-width", 4);
	host->get_mmc_clk = rockchip_dwmmc_get_mmc_clk;
	host->priv = dev;

	/* use non-removeable as sdcard and emmc as judgement */
	if (dev_read_bool(dev, "non-removable"))
		host->dev_index = 0;
	else
		host->dev_index = 1;

	priv->fifo_depth = dev_read_u32_default(dev, "fifo-depth", 0);

	if (priv->fifo_depth < 0)
		return log_msg_ret("rkp", -EINVAL);
	priv->fifo_mode = dev_read_bool(dev, "fifo-mode");

#ifdef CONFIG_XPL_BUILD
	if (!priv->fifo_mode)
		priv->fifo_mode = dev_read_bool(dev, "u-boot,spl-fifo-mode");
#endif

	/*
	 * 'clock-freq-min-max' is deprecated
	 * (see https://github.com/torvalds/linux/commit/b023030f10573de738bbe8df63d43acab64c9f7b)
	 */
	if (dev_read_u32_array(dev, "clock-freq-min-max", priv->minmax, 2)) {
		int val = dev_read_u32_default(dev, "max-frequency", -EINVAL);

		if (val < 0)
			return log_msg_ret("rkc", val);

		priv->minmax[0] = 400000;  /* 400 kHz */
		priv->minmax[1] = val;
	} else {
		debug("%s: 'clock-freq-min-max' property was deprecated.\n",
		      __func__);
	}

	if (CONFIG_IS_ENABLED(DM_GPIO))
		gpio_request_by_name(dev, "cd-gpios", 0, &priv->cd_gpio,
				     GPIOD_IS_IN);

	priv->num_phases = dev_read_u32_default(dev,
						"rockchip,desired-num-phases",
						360);
	priv->default_sample_phase =
		dev_read_u32_default(dev, "rockchip,default-sample-phase", 0);

	return 0;
}

/*
 * Take the host capabilities from the DT through the generic mmc_of_parse()
 * (bus-width, max-frequency, cap-*-highspeed, sd-uhs-*, mmc-hs*, the
 * card-detect flags), then drop every mode that needs a tuning sweep unless
 * this host can actually tune (a clock provider with set_phase on
 * ciu-sample) AND has a vqmmc rail for the 1.8 V switch -- so a board whose
 * clock driver cannot tune never leaves SD High Speed however the DT is
 * decorated.
 */
static int rockchip_dwmmc_parse_caps(struct udevice *dev,
				     struct rockchip_dwmmc_priv *priv,
				     struct mmc_config *cfg)
{
#if CONFIG_IS_ENABLED(OF_REAL)
	int ret;

	ret = mmc_of_parse(dev, cfg);
	if (ret)
		return ret;

	if (!priv->phase_capable || !dev_read_bool(dev, "vqmmc-supply")) {
		debug("%s: %s, tuned modes off\n", dev->name,
		      priv->phase_capable ? "no vqmmc-supply" :
					     "no tunable sample phase");
		cfg->host_caps &= ~(UHS_CAPS | MMC_MODE_HS200 | MMC_MODE_HS400 |
				    MMC_MODE_HS400_ES);
	}
#endif
	return 0;
}

static int rockchip_dwmmc_get_cd(struct udevice *dev)
{
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct mmc *mmc = priv->host.mmc;

	/* Soldered-down, or told to poll: always present */
	if (mmc->cfg->host_caps & (MMC_CAP_NONREMOVABLE | MMC_CAP_NEEDS_POLL))
		return 1;

#if CONFIG_IS_ENABLED(DM_GPIO)
	if (dm_gpio_is_valid(&priv->cd_gpio)) {
		int value = dm_gpio_get_value(&priv->cd_gpio);

		if (value >= 0)
			return (mmc->cfg->host_caps & MMC_CAP_CD_ACTIVE_HIGH) ?
				!value : value;
	}
#endif
	/*
	 * No cd-gpios: the controller's own CDETECT pin is only meaningful
	 * when the pad is muxed to it, which the DT does not say -- assume
	 * present and let the init timeout decide, as before.
	 */
	return 1;
}

static void rockchip_dwmmc_get_phase_clks(struct udevice *dev,
					  struct rockchip_dwmmc_priv *priv)
{
#if CONFIG_IS_ENABLED(OF_REAL)
	int ret;

	ret = clk_get_by_name(dev, "ciu-drive", &priv->drv_clk);
	if (ret)
		debug("%s: ciu-drive not available (%d)\n", dev->name, ret);

	ret = clk_get_by_name(dev, "ciu-sample", &priv->sample_clk);
	if (ret) {
		debug("%s: ciu-sample not available (%d)\n", dev->name, ret);
		return;
	}

	/* A provider without set_phase answers -ENOSYS here */
	ret = clk_get_phase(&priv->sample_clk);
	if (ret < 0) {
		debug("%s: ciu-sample has no phase control (%d)\n", dev->name,
		      ret);
		return;
	}

	priv->phase_capable = true;
#endif
}

static struct dm_mmc_ops rockchip_dwmmc_ops;

static int rockchip_dwmmc_probe(struct udevice *dev)
{
	struct rockchip_mmc_plat *plat = dev_get_plat(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct rockchip_dwmmc_priv *priv = dev_get_priv(dev);
	struct dwmci_host *host = &priv->host;
	int ret;

	/* Extend the generic ops with tuning + card detect; see exynos_dw_mmc.c */
	memcpy(&rockchip_dwmmc_ops, &dm_dwmci_ops, sizeof(rockchip_dwmmc_ops));
#if CONFIG_IS_ENABLED(MMC_SUPPORTS_TUNING)
	rockchip_dwmmc_ops.execute_tuning = rockchip_dwmmc_execute_tuning;
	host->bus_error = rockchip_dwmmc_bus_error;
#endif
	rockchip_dwmmc_ops.get_cd = rockchip_dwmmc_get_cd;
	rockchip_dwmmc_ops.set_ios = rockchip_dwmmc_set_ios;
	rockchip_dwmmc_ops.reinit = rockchip_dwmmc_reinit;

#if CONFIG_IS_ENABLED(OF_PLATDATA)
	struct dtd_rockchip_rk3288_dw_mshc *dtplat = &plat->dtplat;

	host->name = dev->name;
	host->ioaddr = map_sysmem(dtplat->reg[0], dtplat->reg[1]);
	host->buswidth = dtplat->bus_width;
	host->get_mmc_clk = rockchip_dwmmc_get_mmc_clk;
	host->priv = dev;
	host->dev_index = 0;
	priv->fifo_depth = dtplat->fifo_depth;
	priv->fifo_mode = dtplat->u_boot_spl_fifo_mode;
	priv->minmax[0] = 400000;  /*  400 kHz */
	priv->minmax[1] = dtplat->max_frequency;

	ret = clk_get_by_phandle(dev, &dtplat->clocks[1], &priv->clk);
#else
	ret = clk_get_by_index(dev, 1, &priv->clk);
#endif
	if (ret < 0 && ret != -ENOSYS)
		return log_msg_ret("clk", ret);
	host->fifo_depth = priv->fifo_depth;
	host->fifo_mode = priv->fifo_mode;

	rockchip_dwmmc_get_phase_clks(dev, priv);
	host->clksel = rockchip_dwmmc_clksel;
#if CONFIG_IS_ENABLED(DM_REGULATOR)
	host->io_voltage = rockchip_dwmmc_io_voltage;
#endif

#if CONFIG_IS_ENABLED(MMC_PWRSEQ)
	/* Enable power if needed */
	ret = mmc_pwrseq_get_power(dev, &plat->cfg);
	if (!ret) {
		ret = pwrseq_set_power(plat->cfg.pwr_dev, true);
		if (ret)
			return ret;
	}
#endif
	dwmci_setup_cfg(&plat->cfg, host, priv->minmax[1], priv->minmax[0]);
	ret = rockchip_dwmmc_parse_caps(dev, priv, &plat->cfg);
	if (ret)
		return log_msg_ret("caps", ret);
	host->mmc = &plat->mmc;
	host->mmc->priv = &priv->host;
	host->mmc->dev = dev;
	upriv->mmc = host->mmc;

	/* Hosts capable of 8-bit can also do 4 bits */
	if (host->buswidth == 8)
		plat->cfg.host_caps |= MMC_MODE_4BIT;

	return dwmci_probe(dev);
}

static int rockchip_dwmmc_bind(struct udevice *dev)
{
	struct rockchip_mmc_plat *plat = dev_get_plat(dev);

	return dwmci_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id rockchip_dwmmc_ids[] = {
	{ .compatible = "rockchip,rk2928-dw-mshc" },
	{ .compatible = "rockchip,rk3288-dw-mshc" },
	{ .compatible = "rockchip,rk3576-dw-mshc" },
	{ }
};

U_BOOT_DRIVER(rockchip_rk3288_dw_mshc) = {
	.name		= "rockchip_rk3288_dw_mshc",
	.id		= UCLASS_MMC,
	.of_match	= rockchip_dwmmc_ids,
	.of_to_plat = rockchip_dwmmc_of_to_plat,
	.ops		= &rockchip_dwmmc_ops,
	.bind		= rockchip_dwmmc_bind,
	.probe		= rockchip_dwmmc_probe,
	.priv_auto	= sizeof(struct rockchip_dwmmc_priv),
	.plat_auto	= sizeof(struct rockchip_mmc_plat),
};

DM_DRIVER_ALIAS(rockchip_rk3288_dw_mshc, rockchip_rk2928_dw_mshc)
DM_DRIVER_ALIAS(rockchip_rk3288_dw_mshc, rockchip_rk3328_dw_mshc)
DM_DRIVER_ALIAS(rockchip_rk3288_dw_mshc, rockchip_rk3368_dw_mshc)
