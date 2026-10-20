// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MT8365 SCPSYS power-domain driver (focused on the MM domain).
 *
 * Ported from the BayLibre MT8365 BSP u-boot (2022.10,
 * drivers/power/domain/mtk-power-domain.c, scp_domain_mt8365[]) to the
 * 2026.04 fork. The existing fork mtk-power-domain.c only supports the older
 * mt762x SoCs whose SPM/INFRA register layout differs from the MT8365
 * (different ctl_offs, pwr_sta offsets and a multi-step bus-protection
 * sequence touching both infracfg and infracfg-nao), so MT8365 cannot be
 * folded into it as a simple data-table addition.
 *
 * This file implements only what the HDMI boot-splash path needs: powering on
 * the MM (display/SMI) power domain referenced as
 * "power-domains = <&spm MT8365_POWER_DOMAIN_MM>" by the mmsys / mutex / smi /
 * larb0 / rdma1 / dpi0 nodes. The DM core calls dev_power_domain_on() on each
 * of those nodes before probing their drivers; without this provider the
 * power_domain_get() fails and every display node refuses to probe.
 *
 * The full mt8365 SCPSYS has 9 domains; only MM is implemented here because it
 * is the only one on the display data path and the only one any display node
 * references.
 */

#include <clk.h>
#include <dm.h>
#include <mapmem.h>
#include <power-domain-uclass.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/iopoll.h>

#include <dt-bindings/power/mediatek,mt8365-power.h>

/* SPM (scpsys) registers, base 0x10006000 */
#define SPM_EN			(0xb16 << 16 | 0x1)

#define PWR_RST_B_BIT		BIT(0)
#define PWR_ISO_BIT		BIT(1)
#define PWR_ON_BIT		BIT(2)
#define PWR_ON_2ND_BIT		BIT(3)
#define PWR_CLK_DIS_BIT		BIT(4)

#define PWR_STATUS_DISP		BIT(3)

/* MT8365 MM domain SPM offsets (from BSP scp_domain_mt8365[MM]) */
#define MT8365_MM_CTL_OFFS	0x30c
#define MT8365_MM_PWR_STA_OFFS	0x0180
#define MT8365_MM_PWR_STA2ND_OFFS 0x0184
#define MT8365_MM_SRAM_PDN_BITS	GENMASK(8, 8)
#define MT8365_MM_SRAM_PDN_ACK	GENMASK(12, 12)

/* infracfg DCM */
#define INFRA_TOPDCM_CTRL	0x10
#define DCM_TOP_EN		BIT(0)

/* INFRA_TOPAXI protection bits used by the MM domain (from the BSP) */
#define MT8365_INFRA_PROT_MM_M0			BIT(1)
#define MT8365_INFRA_PROT_MDMCU_M1		BIT(2)
#define MT8365_INFRA_PROT_MMAPB_S		BIT(6)
#define MT8365_INFRA_PROT_MM2INFRA_SLV_0	BIT(10)
#define MT8365_INFRA_PROT_MM2INFRA_SLV_1	BIT(11)
#define MT8365_INFRA_PROT_1_MM2INFRA_MST_0	BIT(16)
#define MT8365_INFRA_PROT_1_MM2INFRA_MST_1	BIT(17)
#define MT8365_INFRA_NAO_SI0_CTRL_UPDATED	BIT(24)
#define MT8365_INFRA_SI0_WAY_EN_MMAPB_S		BIT(6)
#define MT8365_INFRA_SI2_WAY_EN_PERI_M1		BIT(5)
#define MT8365_INFRA_NAO_SI2_CTRL_UPDATED	BIT(14)

/*
 * One bus-protection step. A "write" step toggles a mask in a set/clr register
 * and waits for the matching status; a "way-enable" step (wayen) writes a
 * way-enable bit and waits for the ack in the infracfg-nao status register.
 */
struct mt8365_bus_prot {
	u32 mask;
	u32 sta_mask;
	u32 set_ofs;
	u32 clr_ofs;
	u32 sta_ofs;
	bool wayen;
};

#define BUS_PROT_WR(_mask, _set, _clr, _sta) {		\
		.mask = (_mask), .sta_mask = (_mask),	\
		.set_ofs = (_set), .clr_ofs = (_clr),	\
		.sta_ofs = (_sta), .wayen = false,	\
	}

#define BUS_PROT_WAYEN(_en_mask, _sta_mask, _set, _sta) {	\
		.mask = (_en_mask), .sta_mask = (_sta_mask),	\
		.set_ofs = (_set), .clr_ofs = (_set),		\
		.sta_ofs = (_sta), .wayen = true,		\
	}

/* The MM domain bus-protection sequence (5 steps), ordered for power-on. */
static const struct mt8365_bus_prot mt8365_mm_bus_prot[] = {
	BUS_PROT_WR(MT8365_INFRA_PROT_1_MM2INFRA_MST_0 |
		    MT8365_INFRA_PROT_1_MM2INFRA_MST_1,
		    0x2a8, 0x2ac, 0x258),
	BUS_PROT_WR(MT8365_INFRA_PROT_MM_M0 |
		    MT8365_INFRA_PROT_MDMCU_M1 |
		    MT8365_INFRA_PROT_MM2INFRA_SLV_0 |
		    MT8365_INFRA_PROT_MM2INFRA_SLV_1,
		    0x2a0, 0x2a4, 0x228),
	BUS_PROT_WAYEN(MT8365_INFRA_SI0_WAY_EN_MMAPB_S,
		       MT8365_INFRA_NAO_SI0_CTRL_UPDATED,
		       0x200, 0x0),
	BUS_PROT_WAYEN(MT8365_INFRA_SI2_WAY_EN_PERI_M1,
		       MT8365_INFRA_NAO_SI2_CTRL_UPDATED,
		       0x234, 0x28),
	BUS_PROT_WR(MT8365_INFRA_PROT_MMAPB_S,
		    0x2a0, 0x2a4, 0x228),
};

#define MT8365_MM_MAX_CLKS	8

struct mt8365_scp {
	void __iomem *base;		/* SPM */
	void __iomem *infracfg;
	void __iomem *infracfg_nao;
	struct clk clks[MT8365_MM_MAX_CLKS];	/* MM subnode clocks */
	int num_clks;
};

static int mt8365_clks_enable(struct mt8365_scp *scp)
{
	int i, ret;

	for (i = 0; i < scp->num_clks; i++) {
		ret = clk_enable(&scp->clks[i]);
		if (ret && ret != -ENOSYS && ret != -ENOTSUPP)
			return ret;
	}
	return 0;
}

static void mt8365_clks_disable(struct mt8365_scp *scp)
{
	int i;

	for (i = scp->num_clks - 1; i >= 0; i--)
		clk_disable(&scp->clks[i]);
}

static int mt8365_mm_is_on(struct mt8365_scp *scp)
{
	u32 sta = readl(scp->base + MT8365_MM_PWR_STA_OFFS) & PWR_STATUS_DISP;
	u32 sta2 = readl(scp->base + MT8365_MM_PWR_STA2ND_OFFS) & PWR_STATUS_DISP;

	if (sta && sta2)
		return true;
	if (!sta && !sta2)
		return false;
	return -EINVAL;
}

/* Disable (clear) one bus-protection step -- used on power-ON. */
static int mt8365_bus_prot_disable(struct mt8365_scp *scp,
				   const struct mt8365_bus_prot *bp)
{
	u32 val;

	if (bp->wayen) {
		if (!scp->infracfg_nao)
			return -ENODEV;
		/* way-enable: set the enable bit, wait ack clears in NAO */
		clrbits_le32(scp->infracfg + bp->clr_ofs, bp->mask);
		return readl_poll_timeout(scp->infracfg_nao + bp->sta_ofs, val,
					  (val & bp->sta_mask) == bp->sta_mask,
					  1000);
	}

	writel(bp->mask, scp->infracfg + bp->clr_ofs);
	return readl_poll_timeout(scp->infracfg + bp->sta_ofs, val,
				  !(val & bp->sta_mask), 1000);
}

/* Enable (set) one bus-protection step -- used on power-OFF. */
static int mt8365_bus_prot_enable(struct mt8365_scp *scp,
				  const struct mt8365_bus_prot *bp)
{
	u32 val;

	if (bp->wayen) {
		if (!scp->infracfg_nao)
			return -ENODEV;
		setbits_le32(scp->infracfg + bp->set_ofs, bp->mask);
		return readl_poll_timeout(scp->infracfg_nao + bp->sta_ofs, val,
					  !(val & bp->sta_mask), 1000);
	}

	writel(bp->mask, scp->infracfg + bp->set_ofs);
	return readl_poll_timeout(scp->infracfg + bp->sta_ofs, val,
				  (val & bp->sta_mask) == bp->sta_mask, 1000);
}

static int mt8365_mm_power_on(struct power_domain *pd)
{
	struct mt8365_scp *scp = dev_get_priv(pd->dev);
	void __iomem *ctl = scp->base + MT8365_MM_CTL_OFFS;
	u32 val;
	int ret, tmp, i;

	ret = mt8365_clks_enable(scp);
	if (ret)
		return ret;

	writel(SPM_EN, scp->base);

	val = readl(ctl);
	val |= PWR_ON_BIT;
	writel(val, ctl);
	val |= PWR_ON_2ND_BIT;
	writel(val, ctl);

	ret = readx_poll_timeout(mt8365_mm_is_on, scp, tmp, tmp > 0, 100);
	if (ret < 0)
		return ret;

	val &= ~PWR_CLK_DIS_BIT;
	writel(val, ctl);
	val &= ~PWR_ISO_BIT;
	writel(val, ctl);
	val |= PWR_RST_B_BIT;
	writel(val, ctl);

	val &= ~MT8365_MM_SRAM_PDN_BITS;
	writel(val, ctl);

	ret = readl_poll_timeout(ctl, tmp, !(tmp & MT8365_MM_SRAM_PDN_ACK), 100);
	if (ret < 0)
		return ret;

	/* Release bus protection (reverse-order on power-on). */
	for (i = ARRAY_SIZE(mt8365_mm_bus_prot) - 1; i >= 0; i--) {
		ret = mt8365_bus_prot_disable(scp, &mt8365_mm_bus_prot[i]);
		if (ret)
			return ret;
	}

	return 0;
}

static int mt8365_mm_power_off(struct power_domain *pd)
{
	struct mt8365_scp *scp = dev_get_priv(pd->dev);
	void __iomem *ctl = scp->base + MT8365_MM_CTL_OFFS;
	u32 val;
	int ret, tmp, i;

	/* Engage bus protection (forward order on power-off). */
	for (i = 0; i < ARRAY_SIZE(mt8365_mm_bus_prot); i++) {
		ret = mt8365_bus_prot_enable(scp, &mt8365_mm_bus_prot[i]);
		if (ret)
			return ret;
	}

	val = readl(ctl);
	val |= MT8365_MM_SRAM_PDN_BITS;
	writel(val, ctl);

	ret = readl_poll_timeout(ctl, tmp,
				 (tmp & MT8365_MM_SRAM_PDN_ACK) == MT8365_MM_SRAM_PDN_ACK,
				 100);
	if (ret < 0)
		return ret;

	val |= PWR_ISO_BIT;
	writel(val, ctl);
	val &= ~PWR_RST_B_BIT;
	writel(val, ctl);
	val |= PWR_CLK_DIS_BIT;
	writel(val, ctl);
	val &= ~PWR_ON_BIT;
	writel(val, ctl);
	val &= ~PWR_ON_2ND_BIT;
	writel(val, ctl);

	ret = readx_poll_timeout(mt8365_mm_is_on, scp, tmp, !tmp, 100);
	if (ret < 0)
		return ret;

	mt8365_clks_disable(scp);

	return 0;
}

static int mt8365_power_request(struct power_domain *pd)
{
	/* Only MM (id 0) is implemented. */
	if (pd->id != MT8365_POWER_DOMAIN_MM)
		return -ENOTSUPP;
	return 0;
}

static void *mt8365_map_syscon(ofnode node, const char *prop)
{
	struct ofnode_phandle_args args;
	struct regmap *regmap;
	int ret;

	ret = ofnode_parse_phandle_with_args(node, prop, NULL, 0, 0, &args);
	if (ret)
		return NULL;

	regmap = syscon_node_to_regmap(args.node);
	if (IS_ERR(regmap))
		return NULL;

	return regmap_get_range(regmap, 0);
}

static int mt8365_power_domain_probe(struct udevice *dev)
{
	struct mt8365_scp *scp = dev_get_priv(dev);
	ofnode subnode;
	fdt_addr_t base;
	int ret;

	/*
	 * The spm "power-controller" node (mediatek,mt8365-power-controller)
	 * has no reg of its own -- the SPM register window (0x10006000) is the
	 * reg of its parent scpsys syscon node.
	 */
	base = dev_read_addr(dev);
	if (base == FDT_ADDR_T_NONE)
		base = ofnode_get_addr(ofnode_get_parent(dev_ofnode(dev)));
	if (base == FDT_ADDR_T_NONE)
		return -ENOENT;
	scp->base = map_sysmem(base, 0x1000);

	/*
	 * The MM power-domain subnode (power-domain@0) carries the infracfg /
	 * infracfg-nao phandles (upstream: "mediatek,infracfg[-nao]"; BSP flat
	 * shape uses the un-prefixed names on the scpsys node) and the MM gate
	 * clocks (mm, mm-0..mm-3 -> CLK_TOP_MM_SEL + the four MM SMI clocks).
	 * The first subnode that has a "clocks" property is the MM domain.
	 */
	ofnode_for_each_subnode(subnode, dev_ofnode(dev)) {
		int i;

		if (!ofnode_read_prop(subnode, "clocks", NULL))
			continue;

		scp->infracfg = mt8365_map_syscon(subnode, "mediatek,infracfg");
		if (!scp->infracfg)
			scp->infracfg = mt8365_map_syscon(subnode, "infracfg");
		scp->infracfg_nao = mt8365_map_syscon(subnode,
						      "mediatek,infracfg-nao");
		if (!scp->infracfg_nao)
			scp->infracfg_nao = mt8365_map_syscon(subnode,
							      "infracfg-nao");

		for (i = 0; i < MT8365_MM_MAX_CLKS; i++) {
			ret = clk_get_by_index_nodev(subnode, i, &scp->clks[i]);
			if (ret)
				break;
		}
		scp->num_clks = i;
		break;
	}

	/* Fall back to the scpsys-node infracfg phandles (BSP flat shape). */
	if (!scp->infracfg)
		scp->infracfg = mt8365_map_syscon(dev_ofnode(dev), "infracfg");
	if (!scp->infracfg)
		return -ENOENT;
	if (!scp->infracfg_nao)
		scp->infracfg_nao = mt8365_map_syscon(dev_ofnode(dev),
						      "infracfg-nao");

	/* enable Infra DCM (matches BSP) */
	setbits_le32(scp->infracfg + INFRA_TOPDCM_CTRL, DCM_TOP_EN);

	return 0;
}

static const struct power_domain_ops mt8365_power_domain_ops = {
	.on = mt8365_mm_power_on,
	.off = mt8365_mm_power_off,
	.request = mt8365_power_request,
};

static const struct udevice_id mt8365_power_domain_ids[] = {
	{ .compatible = "mediatek,mt8365-power-controller" },
	{ }
};

U_BOOT_DRIVER(mt8365_power_domain) = {
	.name = "mt8365_power_domain",
	.id = UCLASS_POWER_DOMAIN,
	.of_match = mt8365_power_domain_ids,
	.probe = mt8365_power_domain_probe,
	.priv_auto = sizeof(struct mt8365_scp),
	.ops = &mt8365_power_domain_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
