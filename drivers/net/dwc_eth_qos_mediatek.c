// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek DWMAC-5.10a glue for the Synopsys DWC Ethernet QoS core.
 *
 * Ported from Linux drivers/net/ethernet/stmicro/stmmac/dwmac-mediatek.c
 * (Copyright (c) 2018 MediaTek Inc., Biao Huang <biao.huang@mediatek.com>)
 * and modelled on the U-Boot rockchip/stm32 glue in this directory.
 *
 * Targets the "mediatek,mt8195-gmac" binding used by MT8188 / MT8370
 * (Genio 510 / 700). The interface mode (RGMII/RMII/MII) and the TX/RX
 * clock delay lines live in the pericfg (infracfg_ao) syscon reached via
 * the "mediatek,pericfg" phandle; the core registers are the standard
 * DWMAC-5.10a block.
 *
 * The code is intentionally kept close to the Linux counterpart so future
 * fixes port easily. Only the pieces needed for link + MDIO + RGMII/RMII
 * are implemented (no PTP/WoL/TBS).
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <malloc.h>
#include <net.h>
#include <phy.h>
#include <regmap.h>
#include <syscon.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>

#include "dwc_eth_qos.h"

/* Peri (pericfg/infracfg_ao) configuration registers for mt8195 family */
#define MT8195_PERI_ETH_CTRL0		0xFD0
#define MT8195_RMII_CLK_SRC_INTERNAL	BIT(28)
#define MT8195_RMII_CLK_SRC_RXC		BIT(27)
#define MT8195_ETH_INTF_SEL		GENMASK(26, 24)
#define MT8195_RGMII_TXC_PHASE_CTRL	BIT(22)
#define MT8195_EXT_PHY_MODE		BIT(21)
#define MT8195_DLY_GTXC_INV		BIT(12)
#define MT8195_DLY_GTXC_ENABLE		BIT(5)
#define MT8195_DLY_GTXC_STAGES		GENMASK(4, 0)

#define MT8195_PERI_ETH_CTRL1		0xFD4
#define MT8195_DLY_RXC_INV		BIT(25)
#define MT8195_DLY_RXC_ENABLE		BIT(18)
#define MT8195_DLY_RXC_STAGES		GENMASK(17, 13)
#define MT8195_DLY_TXC_INV		BIT(12)
#define MT8195_DLY_TXC_ENABLE		BIT(5)
#define MT8195_DLY_TXC_STAGES		GENMASK(4, 0)

#define MT8195_PERI_ETH_CTRL2		0xFD8
#define MT8195_DLY_RMII_RXC_INV		BIT(25)
#define MT8195_DLY_RMII_RXC_ENABLE	BIT(18)
#define MT8195_DLY_RMII_RXC_STAGES	GENMASK(17, 13)
#define MT8195_DLY_RMII_TXC_INV		BIT(12)
#define MT8195_DLY_RMII_TXC_ENABLE	BIT(5)
#define MT8195_DLY_RMII_TXC_STAGES	GENMASK(4, 0)

#define PHY_INTF_MII			0
#define PHY_INTF_RGMII			1
#define PHY_INTF_RMII			4

/* mt8195 delay line: 290ps per stage, 9280ps max (0..31 stages) */
#define MT8195_DELAY_PS_PER_STAGE	290
#define MT8195_DELAY_PS_MAX		9280

/* Clocks requested by name from the DT "clock-names" property. */
static const char * const mtk_dwmac_clk_l[] = {
	"axi", "apb", "mac_cg", "mac_main", "ptp_ref"
};

#define MTK_DWMAC_NUM_CLKS	ARRAY_SIZE(mtk_dwmac_clk_l)

struct mtk_dwmac_data {
	struct regmap *peri_regmap;
	struct clk clks[MTK_DWMAC_NUM_CLKS];
	struct clk rmii_internal_clk;
	bool has_rmii_internal;
	phy_interface_t phy_mode;
	u32 tx_delay;		/* in stages */
	u32 rx_delay;		/* in stages */
	bool tx_inv;
	bool rx_inv;
	bool rmii_rxc;
	bool rmii_clk_from_mac;
};

static int mt8195_set_interface(struct udevice *dev, struct mtk_dwmac_data *data)
{
	int rmii_clk_from_mac = data->rmii_clk_from_mac ?
				MT8195_RMII_CLK_SRC_INTERNAL : 0;
	int rmii_rxc = data->rmii_rxc ? MT8195_RMII_CLK_SRC_RXC : 0;
	u32 intf_val = 0;

	/* select phy interface in top control domain */
	switch (data->phy_mode) {
	case PHY_INTERFACE_MODE_MII:
		intf_val |= FIELD_PREP(MT8195_ETH_INTF_SEL, PHY_INTF_MII);
		break;
	case PHY_INTERFACE_MODE_RMII:
		intf_val |= (rmii_rxc | rmii_clk_from_mac);
		intf_val |= FIELD_PREP(MT8195_ETH_INTF_SEL, PHY_INTF_RMII);
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		intf_val |= FIELD_PREP(MT8195_ETH_INTF_SEL, PHY_INTF_RGMII);
		break;
	default:
		dev_err(dev, "phy interface not supported\n");
		return -EINVAL;
	}

	/* MT8195 family only supports external PHY */
	intf_val |= MT8195_EXT_PHY_MODE;

	regmap_write(data->peri_regmap, MT8195_PERI_ETH_CTRL0, intf_val);

	return 0;
}

static int mt8195_set_delay(struct udevice *dev, struct mtk_dwmac_data *data)
{
	u32 gtxc_delay_val = 0, delay_val = 0, rmii_delay_val = 0;
	u32 tx_delay = data->tx_delay;
	u32 rx_delay = data->rx_delay;

	switch (data->phy_mode) {
	case PHY_INTERFACE_MODE_MII:
		delay_val |= FIELD_PREP(MT8195_DLY_TXC_ENABLE, !!tx_delay);
		delay_val |= FIELD_PREP(MT8195_DLY_TXC_STAGES, tx_delay);
		delay_val |= FIELD_PREP(MT8195_DLY_TXC_INV, data->tx_inv);

		delay_val |= FIELD_PREP(MT8195_DLY_RXC_ENABLE, !!rx_delay);
		delay_val |= FIELD_PREP(MT8195_DLY_RXC_STAGES, rx_delay);
		delay_val |= FIELD_PREP(MT8195_DLY_RXC_INV, data->rx_inv);
		break;
	case PHY_INTERFACE_MODE_RMII:
		if (data->rmii_clk_from_mac) {
			/*
			 * case 1: mac provides the rmii reference clock, output
			 * to the TXC pin. Egress timing via RMII_TXC delay,
			 * ingress timing via RMII_RXC delay.
			 */
			rmii_delay_val |= FIELD_PREP(MT8195_DLY_RMII_TXC_ENABLE,
						     !!tx_delay);
			rmii_delay_val |= FIELD_PREP(MT8195_DLY_RMII_TXC_STAGES,
						     tx_delay);
			rmii_delay_val |= FIELD_PREP(MT8195_DLY_RMII_TXC_INV,
						     data->tx_inv);

			rmii_delay_val |= FIELD_PREP(MT8195_DLY_RMII_RXC_ENABLE,
						     !!rx_delay);
			rmii_delay_val |= FIELD_PREP(MT8195_DLY_RMII_RXC_STAGES,
						     rx_delay);
			rmii_delay_val |= FIELD_PREP(MT8195_DLY_RMII_RXC_INV,
						     data->rx_inv);
		} else {
			/*
			 * case 2: the rmii reference clock is from the external
			 * phy; "mediatek,rmii-rxc" selects TXC/RXC pin. The
			 * reference is a received signal so rx_delay/rx_inv
			 * drive the adjustment.
			 */
			if (data->rmii_rxc) {
				delay_val |= FIELD_PREP(MT8195_DLY_RXC_ENABLE,
							!!rx_delay);
				delay_val |= FIELD_PREP(MT8195_DLY_RXC_STAGES,
							rx_delay);
				delay_val |= FIELD_PREP(MT8195_DLY_RXC_INV,
							data->rx_inv);
			} else {
				delay_val |= FIELD_PREP(MT8195_DLY_TXC_ENABLE,
							!!rx_delay);
				delay_val |= FIELD_PREP(MT8195_DLY_TXC_STAGES,
							rx_delay);
				delay_val |= FIELD_PREP(MT8195_DLY_TXC_INV,
							data->rx_inv);
			}
		}
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		gtxc_delay_val |= FIELD_PREP(MT8195_DLY_GTXC_ENABLE, !!tx_delay);
		gtxc_delay_val |= FIELD_PREP(MT8195_DLY_GTXC_STAGES, tx_delay);
		gtxc_delay_val |= FIELD_PREP(MT8195_DLY_GTXC_INV, data->tx_inv);

		delay_val |= FIELD_PREP(MT8195_DLY_RXC_ENABLE, !!rx_delay);
		delay_val |= FIELD_PREP(MT8195_DLY_RXC_STAGES, rx_delay);
		delay_val |= FIELD_PREP(MT8195_DLY_RXC_INV, data->rx_inv);
		break;
	default:
		dev_err(dev, "phy interface not supported\n");
		return -EINVAL;
	}

	regmap_update_bits(data->peri_regmap, MT8195_PERI_ETH_CTRL0,
			   MT8195_RGMII_TXC_PHASE_CTRL |
			   MT8195_DLY_GTXC_INV |
			   MT8195_DLY_GTXC_ENABLE |
			   MT8195_DLY_GTXC_STAGES,
			   gtxc_delay_val);
	regmap_write(data->peri_regmap, MT8195_PERI_ETH_CTRL1, delay_val);
	regmap_write(data->peri_regmap, MT8195_PERI_ETH_CTRL2, rmii_delay_val);

	return 0;
}

static u32 mtk_delay_ps_to_stage(struct udevice *dev, u32 ps)
{
	if (ps >= MT8195_DELAY_PS_MAX) {
		dev_warn(dev, "clock delay %ups exceeds max %dps, clamping\n",
			 ps, MT8195_DELAY_PS_MAX);
		ps = MT8195_DELAY_PS_MAX - MT8195_DELAY_PS_PER_STAGE;
	}

	return ps / MT8195_DELAY_PS_PER_STAGE;
}

static int eqos_probe_resources_mediatek(struct udevice *dev)
{
	struct eqos_priv *eqos = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mtk_dwmac_data *data;
	u32 tx_delay_ps, rx_delay_ps;
	int i, ret;

	ret = eqos_get_base_addr_dt(dev);
	if (ret) {
		dev_err(dev, "eqos_get_base_addr_dt failed: %d\n", ret);
		return ret;
	}

	data = calloc(1, sizeof(*data));
	if (!data)
		return -ENOMEM;

	pdata->priv_pdata = data;
	pdata->phy_interface = eqos->config->interface(dev);
	pdata->max_speed = eqos->max_speed;
	data->phy_mode = pdata->phy_interface;

	if (data->phy_mode == PHY_INTERFACE_MODE_NA) {
		dev_err(dev, "Invalid PHY interface\n");
		ret = -EINVAL;
		goto err_free;
	}

	data->peri_regmap = syscon_regmap_lookup_by_phandle(dev,
							    "mediatek,pericfg");
	if (IS_ERR(data->peri_regmap)) {
		dev_err(dev, "Failed to get mediatek,pericfg syscon\n");
		ret = PTR_ERR(data->peri_regmap);
		goto err_free;
	}

	data->rmii_rxc = dev_read_bool(dev, "mediatek,rmii-rxc");
	data->rmii_clk_from_mac = dev_read_bool(dev, "mediatek,rmii-clk-from-mac");
	data->tx_inv = dev_read_bool(dev, "mediatek,txc-inverse");
	data->rx_inv = dev_read_bool(dev, "mediatek,rxc-inverse");

	tx_delay_ps = dev_read_u32_default(dev, "mediatek,tx-delay-ps", 0);
	rx_delay_ps = dev_read_u32_default(dev, "mediatek,rx-delay-ps", 0);
	data->tx_delay = mtk_delay_ps_to_stage(dev, tx_delay_ps);
	data->rx_delay = mtk_delay_ps_to_stage(dev, rx_delay_ps);

	for (i = 0; i < MTK_DWMAC_NUM_CLKS; i++) {
		ret = clk_get_by_name(dev, mtk_dwmac_clk_l[i], &data->clks[i]);
		if (ret) {
			dev_err(dev, "clk_get_by_name(%s) failed: %d\n",
				mtk_dwmac_clk_l[i], ret);
			goto err_free;
		}
	}

	/*
	 * The "rmii_internal" clock is only needed in RMII when the MAC
	 * provides the reference clock; optional otherwise.
	 */
	if (data->rmii_clk_from_mac) {
		ret = clk_get_by_name(dev, "rmii_internal",
				      &data->rmii_internal_clk);
		if (ret) {
			dev_err(dev, "clk_get_by_name(rmii_internal) failed: %d\n",
				ret);
			goto err_free;
		}
		data->has_rmii_internal = true;
	}

	return 0;

err_free:
	free(data);
	pdata->priv_pdata = NULL;

	return ret;
}

static int eqos_remove_resources_mediatek(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mtk_dwmac_data *data = pdata->priv_pdata;

	if (data) {
		free(data);
		pdata->priv_pdata = NULL;
	}

	return 0;
}

static int eqos_start_clks_mediatek(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mtk_dwmac_data *data = pdata->priv_pdata;
	int i, ret;

	for (i = 0; i < MTK_DWMAC_NUM_CLKS; i++) {
		ret = clk_enable(&data->clks[i]);
		if (ret < 0) {
			dev_err(dev, "clk_enable(%s) failed: %d\n",
				mtk_dwmac_clk_l[i], ret);
			goto err_disable;
		}
	}

	if (data->has_rmii_internal) {
		ret = clk_enable(&data->rmii_internal_clk);
		if (ret < 0) {
			dev_err(dev, "clk_enable(rmii_internal) failed: %d\n",
				ret);
			goto err_disable;
		}
	}

	/*
	 * Program the interface mode + TX/RX delays in the pericfg syscon.
	 * These live outside the MAC and survive the core's SWR reset, so
	 * doing it once here (mirroring the rockchip glue) is sufficient.
	 */
	ret = mt8195_set_interface(dev, data);
	if (ret)
		goto err_disable;

	ret = mt8195_set_delay(dev, data);
	if (ret)
		goto err_disable;

	return 0;

err_disable:
	if (data->has_rmii_internal)
		clk_disable(&data->rmii_internal_clk);
	while (--i >= 0)
		clk_disable(&data->clks[i]);

	return ret;
}

static int eqos_stop_clks_mediatek(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mtk_dwmac_data *data = pdata->priv_pdata;
	int i;

	if (data->has_rmii_internal)
		clk_disable(&data->rmii_internal_clk);

	for (i = MTK_DWMAC_NUM_CLKS - 1; i >= 0; i--)
		clk_disable(&data->clks[i]);

	return 0;
}

static ulong eqos_get_tick_clk_rate_mediatek(struct udevice *dev)
{
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct mtk_dwmac_data *data = pdata->priv_pdata;

	/* clks[0] == "axi", the CSR/register-access clock feeding us_tic. */
	return clk_get_rate(&data->clks[0]);
}

static struct eqos_ops eqos_mediatek_ops = {
	.eqos_inval_desc = eqos_inval_desc_generic,
	.eqos_flush_desc = eqos_flush_desc_generic,
	.eqos_inval_buffer = eqos_inval_buffer_generic,
	.eqos_flush_buffer = eqos_flush_buffer_generic,
	.eqos_probe_resources = eqos_probe_resources_mediatek,
	.eqos_remove_resources = eqos_remove_resources_mediatek,
	.eqos_stop_resets = eqos_null_ops,
	.eqos_start_resets = eqos_null_ops,
	.eqos_stop_clks = eqos_stop_clks_mediatek,
	.eqos_start_clks = eqos_start_clks_mediatek,
	.eqos_calibrate_pads = eqos_null_ops,
	.eqos_disable_calibration = eqos_null_ops,
	.eqos_set_tx_clk_speed = eqos_null_ops,
	.eqos_get_enetaddr = eqos_null_ops,
	.eqos_get_tick_clk_rate = eqos_get_tick_clk_rate_mediatek,
};

struct eqos_config eqos_mediatek_config = {
	.reg_access_always_ok = false,
	.mdio_wait = 20,
	.swr_wait = 50,
	.config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB,
	.config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_250_300,
	.axi_bus_width = EQOS_AXI_WIDTH_64,
	.interface = dev_read_phy_mode,
	.ops = &eqos_mediatek_ops,
};
