// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2017 Theobroma Systems Design und Consulting GmbH
 */

#include <clk.h>
#include <display.h>
#include <dm.h>
#include <dw_hdmi.h>
#include <edid.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/gpio.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/hardware.h>
#include <asm/arch-rockchip/grf_rk3399.h>
#include <power/regulator.h>
#include "rk_hdmi.h"

static int rk3399_hdmi_enable(struct udevice *dev, int panel_bpp,
			      const struct display_timing *edid)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct display_plat *uc_plat = dev_get_uclass_plat(dev);
	int vop_id = uc_plat->source_id;
	struct rk3399_grf_regs *grf = priv->grf;

	/* select the hdmi encoder input data from our source_id */
	rk_clrsetreg(&grf->soc_con20, GRF_RK3399_HDMI_VOP_SEL_MASK,
		     (vop_id == 1) ? GRF_RK3399_HDMI_VOP_SEL_L : 0);

	return dw_hdmi_enable(&priv->hdmi, edid);
}

static int rk3399_hdmi_of_to_plat(struct udevice *dev)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct dw_hdmi *hdmi = &priv->hdmi;

	hdmi->i2c_clk_high = 0x7a;
	hdmi->i2c_clk_low = 0x8d;

	return rk_hdmi_of_to_plat(dev);
}

/*
 * Both spellings, because the name is the only thing matched and boards
 * disagree on it. regulator_autoset_by_name() matches `regulator-name`
 * literally, and ROC-RK3399-PC calls its HDMI analog rails vcca1v8_hdmi
 * (LDO_REG2) and vcca0v9_hdmi (LDO_REG7) -- with the extra 'a' -- so the two
 * names below missed entirely and the rails were never enabled here.
 *
 * Nothing else turns them on: regulators_enable_boot_on() has no caller
 * anywhere in this tree, so regulator-always-on / regulator-boot-on in the DT
 * are NOT honoured by U-Boot. rk_vop_probe_regulators() ignores a per-name
 * failure by design, so the miss was silent.
 *
 * What this does NOT fix, measured: the suspicion was that unpowered analog
 * rails would make dw_hdmi_detect_hpd() time out and surface as -ENODEV. On
 * roc-rk3399-pc HPD_PHY_STAT0 (0xff94c010) reads 0x02 -- HPD ASSERTED -- so
 * that path was never the failure here, and this correction stands on its own
 * merits rather than as a repair for it. Whether LDO2/LDO7 happened to be on at
 * power-on reset is still not established (CONFIG_CMD_REGULATOR is off, so it
 * cannot be read from the prompt); the point is only that nothing about the
 * observed HDMI failure is explained by these names. Keeping the original two
 * costs nothing and other RK3399 boards use them.
 */
static const char * const rk3399_regulator_names[] = {
	"vcc1v8_hdmi",
	"vcc0v9_hdmi",
	"vcca1v8_hdmi",
	"vcca0v9_hdmi"
};

static int rk3399_hdmi_probe(struct udevice *dev)
{
	/* Enable regulators required for HDMI */
	rk_hdmi_probe_regulators(dev, rk3399_regulator_names,
				 ARRAY_SIZE(rk3399_regulator_names));

	return rk_hdmi_probe(dev);
}

static const struct dm_display_ops rk3399_hdmi_ops = {
	.read_edid = rk_hdmi_read_edid,
	.detect = rk_hdmi_detect,
	.enable = rk3399_hdmi_enable,
};

static const struct udevice_id rk3399_hdmi_ids[] = {
	{ .compatible = "rockchip,rk3399-dw-hdmi" },
	{ }
};

U_BOOT_DRIVER(rk3399_hdmi_rockchip) = {
	.name = "rk3399_hdmi_rockchip",
	.id = UCLASS_DISPLAY,
	.of_match = rk3399_hdmi_ids,
	.ops = &rk3399_hdmi_ops,
	.of_to_plat = rk3399_hdmi_of_to_plat,
	.probe = rk3399_hdmi_probe,
	.priv_auto	= sizeof(struct rk_hdmi_priv),
};
