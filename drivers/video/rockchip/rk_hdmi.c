// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2017 Theobroma Systems Design und Consulting GmbH
 * Copyright (c) 2015 Google, Inc
 * Copyright 2014 Rockchip Inc.
 */

#include <clk.h>
#include <display.h>
#include <dm.h>
#include <dw_hdmi.h>
#include <edid.h>
#include <log.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/gpio.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/hardware.h>
#include <dm/device_compat.h>
#include "rk_hdmi.h"
#include "rk_vop.h" /* for rk_vop_probe_regulators */

static const struct hdmi_phy_config rockchip_phy_config[] = {
	{
		.mpixelclock = 74250000,
		.sym_ctr = 0x8009, .term = 0x0004, .vlev_ctr = 0x0272,
	}, {
		.mpixelclock = 148500000,
		.sym_ctr = 0x802b, .term = 0x0004, .vlev_ctr = 0x028d,
	}, {
		.mpixelclock = 297000000,
		.sym_ctr = 0x8039, .term = 0x0005, .vlev_ctr = 0x028d,
	}, {
		.mpixelclock = 584000000,
		.sym_ctr = 0x8039, .term = 0x0000, .vlev_ctr = 0x019d,
	}, {
		.mpixelclock = ~0ul,
		.sym_ctr = 0x0000, .term = 0x0000, .vlev_ctr = 0x0000,
	}
};

static const struct hdmi_mpll_config rockchip_mpll_cfg[] = {
	{
		.mpixelclock = 40000000,
		.cpce = 0x00b3, .gmp = 0x0000, .curr = 0x0018,
	}, {
		.mpixelclock = 65000000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0028,
	}, {
		.mpixelclock = 66000000,
		.cpce = 0x013e, .gmp = 0x0003, .curr = 0x0038,
	}, {
		.mpixelclock = 83500000,
		.cpce = 0x0072, .gmp = 0x0001, .curr = 0x0028,
	}, {
		.mpixelclock = 146250000,
		.cpce = 0x0051, .gmp = 0x0002, .curr = 0x0038,
	}, {
		.mpixelclock = 148500000,
		.cpce = 0x0051, .gmp = 0x0003, .curr = 0x0000,
	}, {
		.mpixelclock = 272000000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0000,
	}, {
		.mpixelclock = 340000000,
		.cpce = 0x0040, .gmp = 0x0003, .curr = 0x0000,
	}, {
		.mpixelclock = ~0ul,
		.cpce = 0x0051, .gmp = 0x0003, .curr = 0x0000,
	}
};

int rk_hdmi_read_edid(struct udevice *dev, u8 *buf, int buf_size)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);

	return dw_hdmi_read_edid(&priv->hdmi, buf, buf_size);
}

/*
 * HPD is a pin, and it is the only thing on this connector that answers when
 * the EDID does not. On a board whose DDC EEPROM NAKs, HPD asserted is what
 * distinguishes "a monitor is plugged into this port and we could not read it"
 * from "this port is empty" -- the two cases the display-uclass caller has to
 * tell apart before it decides whether guessing a mode is justified.
 *
 * dw_hdmi_phy_wait_for_hpd() polls HDMI_PHY_STAT0 for up to 300 ms, which is
 * also what rk_hdmi_probe() already uses, so this reports exactly the signal
 * the probe gated on.
 *
 * struct dm_display_ops .detect documents "-ve on error", and this
 * implementation never returns one. That is not a lost error code: the only
 * thing underneath is dw_hdmi_phy_wait_for_hpd(), whose entire body is a
 * register read in a 300 ms loop returning 0 or -1, and hdmi_get_plug_in_status()
 * cannot fail -- HPD is a pin, read over a bus that is already up because the
 * caller probed this device. There is no I/O error to distinguish "we could not
 * read HPD" from "HPD is low", so inventing one here would report a fault the
 * hardware path cannot produce. If a future PHY grows a readback that CAN fail,
 * that error belongs in dw_hdmi_phy_wait_for_hpd() and this function should
 * forward it; until then 0/1 is the whole truth.
 */
int rk_hdmi_detect(struct udevice *dev)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);

	return dw_hdmi_phy_wait_for_hpd(&priv->hdmi) < 0 ? 0 : 1;
}

int rk_hdmi_of_to_plat(struct udevice *dev)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct dw_hdmi *hdmi = &priv->hdmi;
	int ret;

	hdmi->ioaddr = (ulong)dev_read_addr(dev);
	hdmi->mpll_cfg = rockchip_mpll_cfg;
	hdmi->phy_cfg = rockchip_phy_config;

	/* hdmi->i2c_clk_{high,low} are set up by the SoC driver */

	hdmi->reg_io_width = 4;

	priv->grf = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);

	/*
	 * Say so when the DT names a DDC bus we cannot resolve. The return was
	 * discarded, so a phandle that failed to probe left ddc_bus NULL and
	 * hdmi_read_edid() silently took its internal-I2CM fallback instead --
	 * a completely different transport, chosen without a word. That turns
	 * "i2c3 did not come up" into "the monitor did not answer", which are
	 * not the same bug and do not have the same fix.
	 */
	ret = uclass_get_device_by_phandle(UCLASS_I2C, dev, "ddc-i2c-bus",
					   &hdmi->ddc_bus);
	if (ret && ofnode_read_prop(dev_ofnode(dev), "ddc-i2c-bus", NULL))
		dev_err(dev, "ddc-i2c-bus named in DT but unresolvable (%d) -- falling back to internal I2CM\n",
			ret);

	return 0;
}

void rk_hdmi_probe_regulators(struct udevice *dev,
			      const char * const *names, int cnt)
{
	rk_vop_probe_regulators(dev, names, cnt);
}

int rk_hdmi_probe(struct udevice *dev)
{
	struct rk_hdmi_priv *priv = dev_get_priv(dev);
	struct dw_hdmi *hdmi = &priv->hdmi;
	int ret;

	dw_hdmi_init(hdmi);
	dw_hdmi_phy_init(hdmi);

	ret = dw_hdmi_detect_hpd(hdmi);
	if (ret < 0)
		return ret;

	return 0;
}
