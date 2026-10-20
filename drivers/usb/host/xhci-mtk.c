// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2019 MediaTek, Inc.
 * Authors: Chunfeng Yun <chunfeng.yun@mediatek.com>
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <generic-phy.h>
#include <malloc.h>
#include <power/regulator.h>
#include <usb.h>
#include <usb/xhci.h>
#include <linux/bitfield.h>
#include <linux/compat.h>
#include <linux/errno.h>
#include <linux/iopoll.h>

/* IPPC (IP Port Control) registers */
#define IPPC_IP_PW_CTRL0		0x00
#define CTRL0_IP_SW_RST			BIT(0)

#define IPPC_IP_PW_CTRL1		0x04
#define CTRL1_IP_HOST_PDN		BIT(0)

#define IPPC_IP_PW_STS1			0x10
#define STS1_IP_SLEEP_STS		BIT(30)
#define STS1_U3_MAC_RST			BIT(16)
#define STS1_XHCI_RST			BIT(11)
#define STS1_SYS125_RST			BIT(10)
#define STS1_REF_RST			BIT(8)
#define STS1_SYSPLL_STABLE		BIT(0)

#define IPPC_IP_XHCI_CAP		0x24
#define CAP_U3_PORT_NUM(p)		((p) & 0xff)
#define CAP_U2_PORT_NUM(p)		(((p) >> 8) & 0xff)

#define IPPC_U3_CTRL_0P			0x30
#define CTRL_U3_PORT_HOST_SEL		BIT(2)
#define CTRL_U3_PORT_PDN		BIT(1)
#define CTRL_U3_PORT_DIS		BIT(0)

#define IPPC_U2_CTRL_0P			0x50
#define CTRL_U2_PORT_RG_IDDIG		BIT(12)
#define CTRL_U2_PORT_FORCE_IDDIG	BIT(11)
#define CTRL_U2_PORT_HOST_SEL		BIT(2)
#define CTRL_U2_PORT_PDN		BIT(1)
#define CTRL_U2_PORT_DIS		BIT(0)

#define IPPC_U3_CTRL(p)	(IPPC_U3_CTRL_0P + ((p) * 0x08))
#define IPPC_U2_CTRL(p)	(IPPC_U2_CTRL_0P + ((p) * 0x08))

/* xHCI CSR */
#define LS_EOF_CFG		0x930
#define LSEOF_OFFSET		0x89

#define FS_EOF_CFG		0x934
#define FSEOF_OFFSET		0x2e

#define SS_GEN1_EOF_CFG		0x93c
#define SSG1EOF_OFFSET		0x78

#define HFCNTR_CFG		0x944
#define ITP_DELTA_CLK_MASK	GENMASK(5, 1)
#define FRMCNT_LEV1_RANG_MASK	GENMASK(19, 8)

#define SS_GEN2_EOF_CFG		0x990
#define SSG2EOF_OFFSET		0x3c

#define XSEOF_OFFSET_MASK	GENMASK(11, 0)

struct mtk_xhci {
	struct xhci_ctrl ctrl;	/* Needs to come first in this struct! */
	struct xhci_hccr *hcd;
	void __iomem *ippc;
	struct udevice *dev;
	struct udevice *vusb33_supply;
	struct udevice *vbus_supply;
	struct clk_bulk clks;
	struct phy_bulk phys;
	int num_u2ports;
	int num_u3ports;
	u32 u3p_dis_msk;
	u32 u2p_dis_msk;
	bool glue_child;	/* bound under the mtu3 glue, which owns clks+vusb33 */
};

/*
 * workaround for mt8195:
 * MT8195 has 4 controllers, the controller1~3's default SOF/ITP interval
 * is calculated from the frame counter clock 24M, but in fact, the clock
 * is 48M.
 */
static void xhci_mtk_set_frame_interval(struct mtk_xhci *mtk)
{
	void __iomem *mac = (void __iomem *)mtk->hcd;

	if (!ofnode_device_is_compatible(dev_ofnode(mtk->dev), "mediatek,mt8195-xhci"))
		return;

	clrsetbits_le32(mac + HFCNTR_CFG,
			ITP_DELTA_CLK_MASK | FRMCNT_LEV1_RANG_MASK,
			FIELD_PREP(ITP_DELTA_CLK_MASK, 0xa) |
			FIELD_PREP(FRMCNT_LEV1_RANG_MASK, 0x12b));

	clrsetbits_le32(mac + LS_EOF_CFG, XSEOF_OFFSET_MASK, LSEOF_OFFSET);
	clrsetbits_le32(mac + FS_EOF_CFG, XSEOF_OFFSET_MASK, FSEOF_OFFSET);
	clrsetbits_le32(mac + SS_GEN1_EOF_CFG, XSEOF_OFFSET_MASK, SSG1EOF_OFFSET);
	clrsetbits_le32(mac + SS_GEN2_EOF_CFG, XSEOF_OFFSET_MASK, SSG2EOF_OFFSET);
}

static int xhci_mtk_host_enable(struct mtk_xhci *mtk)
{
	int u3_ports_disabed = 0;
	u32 value;
	u32 check_val;
	int ret;
	int i;

	/*
	 * Force the OTG role to HOST on the OTG port (U2 port 0): set
	 * FORCE_IDDIG, clear RG_IDDIG. The single SSUSB controller otherwise
	 * follows the cable/VBUS IDDIG comparator -- which reads "device"
	 * whenever the Type-C is plugged (VBUS present) -- and the mtu3 gadget
	 * actively forces device (ssusb_set_force_mode(FORCE_DEVICE) sets both
	 * IDDIG bits). Either way the host IP can't power up (HOST_PDN won't
	 * clear, SYSPLL never stabilises) so it never owns the port the
	 * directly-wired on-board LAN9512 sits on. Override it here (mirrors
	 * mtu3 ssusb_set_force_mode(FORCE_HOST)) so the LAN9512 enumerates with
	 * the Type-C cable still plugged in -- there is no mux on the i350; the
	 * blocker was role, not routing.
	 */
	clrsetbits_le32(mtk->ippc + IPPC_U2_CTRL(0),
			CTRL_U2_PORT_RG_IDDIG, CTRL_U2_PORT_FORCE_IDDIG);

	/* power on host ip */
	clrbits_le32(mtk->ippc + IPPC_IP_PW_CTRL1, CTRL1_IP_HOST_PDN);

	/* power on and enable u3 ports except skipped ones */
	for (i = 0; i < mtk->num_u3ports; i++) {
		if (BIT(i) & mtk->u3p_dis_msk) {
			u3_ports_disabed++;
			continue;
		}

		clrsetbits_le32(mtk->ippc + IPPC_U3_CTRL(i),
				CTRL_U3_PORT_PDN | CTRL_U3_PORT_DIS,
				CTRL_U3_PORT_HOST_SEL);
	}

	/* power on and enable u2 ports except skipped ones */
	for (i = 0; i < mtk->num_u2ports; i++) {
		if (BIT(i) & mtk->u2p_dis_msk)
			continue;

		clrsetbits_le32(mtk->ippc + IPPC_U2_CTRL(i),
				CTRL_U2_PORT_PDN | CTRL_U2_PORT_DIS,
				CTRL_U2_PORT_HOST_SEL);
	}

	/*
	 * wait for clocks to be stable, and clock domains reset to
	 * be inactive after power on and enable ports
	 */
	check_val = STS1_SYSPLL_STABLE | STS1_REF_RST |
			STS1_SYS125_RST | STS1_XHCI_RST;

	if (mtk->num_u3ports > u3_ports_disabed)
		check_val |= STS1_U3_MAC_RST;

	ret = readl_poll_timeout(mtk->ippc + IPPC_IP_PW_STS1, value,
				 (check_val == (value & check_val)), 20000);
	if (ret)
		dev_err(mtk->dev, "clocks are not stable 0x%x!\n", value);

	return ret;
}

static int xhci_mtk_host_disable(struct mtk_xhci *mtk)
{
	int i;

	/* power down all u3 ports */
	for (i = 0; i < mtk->num_u3ports; i++)
		setbits_le32(mtk->ippc + IPPC_U3_CTRL(i),
			     CTRL_U3_PORT_PDN | CTRL_U3_PORT_DIS);

	/* power down all u2 ports */
	for (i = 0; i < mtk->num_u2ports; i++)
		setbits_le32(mtk->ippc + IPPC_U2_CTRL(i),
			     CTRL_U2_PORT_PDN | CTRL_U2_PORT_DIS);

	/* power down host ip */
	setbits_le32(mtk->ippc + IPPC_IP_PW_CTRL1, CTRL1_IP_HOST_PDN);

	return 0;
}

static int xhci_mtk_ssusb_init(struct mtk_xhci *mtk)
{
	u32 value;

	/* reset whole ip */
	setbits_le32(mtk->ippc + IPPC_IP_PW_CTRL0, CTRL0_IP_SW_RST);
	udelay(1);
	clrbits_le32(mtk->ippc + IPPC_IP_PW_CTRL0, CTRL0_IP_SW_RST);

	value = readl(mtk->ippc + IPPC_IP_XHCI_CAP);
	mtk->num_u3ports = CAP_U3_PORT_NUM(value);
	mtk->num_u2ports = CAP_U2_PORT_NUM(value);
	dev_info(mtk->dev, "u2p:%d, u3p:%d\n",
		 mtk->num_u2ports, mtk->num_u3ports);

	return xhci_mtk_host_enable(mtk);
}

static int xhci_mtk_ofdata_get(struct mtk_xhci *mtk)
{
	struct udevice *dev = mtk->dev;
	int ret = 0;

	mtk->hcd = devfdt_remap_addr_name(dev, "mac");
	if (!mtk->hcd) {
		dev_err(dev, "failed to get xHCI base address\n");
		return -ENXIO;
	}

	/*
	 * In the standalone binding the xhci node carries "ippc" itself. In the
	 * upstream nested binding (mt8183: usb_host under the "mediatek,mtu3"
	 * glue) the IPPC window and the port-disable masks live on the glue
	 * parent instead, and only "mac" is on the child. Fall back to the
	 * parent, exactly as generic_phy_get_bulk() already does for "phys".
	 */
	mtk->ippc = devfdt_remap_addr_name(dev, "ippc");
	if (!mtk->ippc && dev->parent)
		mtk->ippc = devfdt_remap_addr_name(dev->parent, "ippc");
	if (!mtk->ippc) {
		dev_err(dev, "failed to get IPPC base address\n");
		return -ENXIO;
	}

	dev_info(dev, "hcd: 0x%p, ippc: 0x%p\n", mtk->hcd, mtk->ippc);

	ret = clk_get_bulk(dev, &mtk->clks);
	if (ret) {
		dev_err(dev, "failed to get clocks %d!\n", ret);
		return ret;
	}

	ret = device_get_supply_regulator(dev, "vusb33-supply",
					  &mtk->vusb33_supply);
	if (ret)
		debug("can't get vusb33 regulator %d!\n", ret);

	ret = device_get_supply_regulator(dev, "vbus-supply",
					  &mtk->vbus_supply);
	if (ret)
		debug("can't get vbus regulator %d!\n", ret);

	/* optional properties to disable ports, ignore the error */
	if (dev_read_u32(dev, "mediatek,u3p-dis-msk", &mtk->u3p_dis_msk) && dev->parent)
		dev_read_u32(dev->parent, "mediatek,u3p-dis-msk", &mtk->u3p_dis_msk);
	if (dev_read_u32(dev, "mediatek,u2p-dis-msk", &mtk->u2p_dis_msk) && dev->parent)
		dev_read_u32(dev->parent, "mediatek,u2p-dis-msk", &mtk->u2p_dis_msk);
	dev_info(dev, "ports disabled mask: u3p-0x%x, u2p-0x%x\n",
		 mtk->u3p_dis_msk, mtk->u2p_dis_msk);

	return 0;
}

static int xhci_mtk_ldos_enable(struct mtk_xhci *mtk)
{
	int ret;

	ret = regulator_set_enable(mtk->vusb33_supply, true);
	if (ret < 0 && ret != -ENOSYS && ret != -EALREADY) {
		dev_err(mtk->dev, "failed to enable vusb33 %d!\n", ret);
		return ret;
	}

	ret = regulator_set_enable(mtk->vbus_supply, true);
	if (ret < 0 && ret != -ENOSYS && ret != -EALREADY) {
		dev_err(mtk->dev, "failed to enable vbus %d!\n", ret);
		regulator_set_enable(mtk->vusb33_supply, false);
		return ret;
	}

	return 0;
}

static void xhci_mtk_ldos_disable(struct mtk_xhci *mtk)
{
	regulator_set_enable(mtk->vbus_supply, false);
	regulator_set_enable(mtk->vusb33_supply, false);
}

static int xhci_mtk_phy_setup(struct mtk_xhci *mtk)
{
	struct udevice *dev = mtk->dev;
	struct phy_bulk *phys = &mtk->phys;
	int ret, i;

	ret = generic_phy_get_bulk(dev, phys);
	if (ret) {
		dev_err(dev, "phy get_bulk failed %d\n", ret);
		return ret;
	}

	ret = generic_phy_init_bulk(phys);
	if (ret) {
		dev_err(dev, "phy init_bulk failed %d\n", ret);
		return ret;
	}

	ret = generic_phy_power_on_bulk(phys);
	if (ret) {
		dev_err(dev, "phy power_on_bulk failed %d\n", ret);
		generic_phy_exit_bulk(phys);
		return ret;
	}

	/*
	 * Force every U2 PHY to host OTG role. Without this the PHY IDDIG
	 * comparator reads "device" when no Type-C cable is present, so the
	 * controller never enumerates the on-board host devices (the LAN9512
	 * RJ45). Must follow power_on (which rewrites DTM1). Mirrors the kernel
	 * HCD's usb_phy_roothub_set_mode(PHY_MODE_USB_HOST).
	 */
	for (i = 0; i < phys->count; i++)
		generic_phy_set_mode(&phys->phys[i], PHY_MODE_USB_HOST, 0);

	return 0;
}

static void xhci_mtk_phy_shutdown(struct mtk_xhci *mtk)
{
	generic_phy_power_off_bulk(&mtk->phys);
	generic_phy_exit_bulk(&mtk->phys);
}

static int xhci_mtk_probe(struct udevice *dev)
{
	struct mtk_xhci *mtk = dev_get_priv(dev);
	struct xhci_hcor *hcor;
	int ret;

	mtk->dev = dev;

	/*
	 * Bound as the mtu3 glue's child (CONFIG_USB_MTU3_XHCI_CHILD)? Then the
	 * glue owns the SSUSB clocks and the vusb33 LDO: DM probes the parent
	 * first, and the glue stays probed after `usb stop` removes us. Remember
	 * that, so .remove can leave the shared resources alone.
	 *
	 * Gating those clocks is what breaks the gadget. MediaTek's gate ops are
	 * raw register writes -- CLK_CCF is off, so clk_enable/clk_disable are NOT
	 * refcounted -- and dropping them takes the whole SSUSB IP down with them
	 * (the IPPC window then reads back all-zero). Critically it also drops the
	 * SSUSB PLL: U3D_SSUSB_IP_PW_STS1.SSUSB_SYSPLL_STABLE (BIT(0)) goes 0, and
	 * `usb stop` has just set IP_PW_CTRL1.SSUSB_IP_HOST_PDN, which nothing on
	 * the device path ever clears. So the PLL never re-locks: merely turning
	 * the clocks back on restores register access and SYS125/REF_RST_B, but not
	 * SYSPLL_STABLE. The next ssusb_gadget_init() then spins in
	 * ssusb_check_clocks() -- which polls SYS125|SYSPLL_STABLE|REF -- and
	 * returns -110, killing fastboot, ums and dfu until the board is reset.
	 *
	 * Measured on a Pumpkin i350 (mt8365): host up STS1=0x03003f0f; after a
	 * clock-gating `usb stop` + gadget probe STS1=0x0100350e (SYSPLL_STABLE
	 * clear); with the clocks left alone STS1=0x00003d0f (stable) even though
	 * CTRL1=1, i.e. HOST_PDN set. Isolated by ablation: gating only the four
	 * clocks the glue also owns reproduces the -110; gating only xhci_ck does
	 * not; disabling the LDO alone does not. vusb33 is skipped anyway because
	 * it is the glue's to hold, not because it triggers this.
	 */
	mtk->glue_child = dev->parent &&
			  ofnode_device_is_compatible(dev_ofnode(dev->parent),
						      "mediatek,mtu3");

	ret = xhci_mtk_ofdata_get(mtk);
	if (ret)
		return ret;

	ret = xhci_mtk_ldos_enable(mtk);
	if (ret)
		goto ldos_err;

	ret = clk_enable_bulk(&mtk->clks);
	if (ret)
		goto clks_err;

	/*
	 * Order matters: wake the SSUSB host IP (clear HOST_PDN, enable the
	 * U2/U3 ports, wait for SYSPLL) BEFORE touching the T-PHY. The U2-PHY
	 * register banks (0x11cc_xxxx) are only writable once the IP is out of
	 * sleep -- if phy_setup() runs first its power_on/set_mode writes to
	 * U3P_U2PHYDTM1 are silently dropped (verified live: a raw mw reads
	 * back 0 while IP_SLEEP=1), so the port never leaves device mode and
	 * the on-board LAN9512 never enumerates. ssusb_init() also IP_SW_RSTs
	 * the whole IP, overriding any device-mode state the mtu3 gadget left.
	 */
	ret = xhci_mtk_ssusb_init(mtk);
	if (ret)
		goto ssusb_init_err;

	ret = xhci_mtk_phy_setup(mtk);
	if (ret)
		goto phys_err;

	xhci_mtk_set_frame_interval(mtk);

	mtk->ctrl.quirks = XHCI_MTK_HOST;
	hcor = (struct xhci_hcor *)((uintptr_t)mtk->hcd +
			HC_LENGTH(xhci_readl(&mtk->hcd->cr_capbase)));

	ret = xhci_register(dev, mtk->hcd, hcor);
	return ret;

phys_err:
	xhci_mtk_phy_shutdown(mtk);
ssusb_init_err:
	xhci_mtk_host_disable(mtk);
	clk_disable_bulk(&mtk->clks);
clks_err:
	xhci_mtk_ldos_disable(mtk);
ldos_err:
	return ret;
}

static int xhci_mtk_remove(struct udevice *dev)
{
	struct mtk_xhci *mtk = dev_get_priv(dev);

	xhci_deregister(dev);

	/*
	 * Always give the port back: HOST_PDN + clearing PORT_HOST_SEL is exactly
	 * what `usb stop` must do so the mtu3 gadget can claim the OTG port.
	 */
	xhci_mtk_host_disable(mtk);

	/*
	 * But the clocks and vusb33 belong to the mtu3 glue when we are its child
	 * (see xhci_mtk_probe). Neither is refcounted, so releasing them here
	 * would gate them under a parent that is still probed and still expects
	 * them on -- and gating them costs the SSUSB PLL, which HOST_PDN then
	 * prevents from re-locking. Standalone (top-level xhci node, no glue
	 * parent) we own them; note that a standalone xhci sharing the IP with an
	 * mtu3 glue hits the same PLL trap, because a later glue probe restores
	 * the clocks but cannot restore SYSPLL_STABLE.
	 */
	if (!mtk->glue_child) {
		xhci_mtk_ldos_disable(mtk);
		clk_disable_bulk(&mtk->clks);
	}

	return 0;
}

static const struct udevice_id xhci_mtk_ids[] = {
	{ .compatible = "mediatek,mtk-xhci" },
	{ .compatible = "mediatek,mt8195-xhci" },
	{ }
};

U_BOOT_DRIVER(xhci_mtk) = {
	.name = "xhci-mtk",
	.id = UCLASS_USB,
	.of_match = xhci_mtk_ids,
	.probe = xhci_mtk_probe,
	.remove = xhci_mtk_remove,
	.ops = &xhci_usb_ops,
	.bind = dm_scan_fdt_dev,
	.priv_auto	= sizeof(struct mtk_xhci),
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};
