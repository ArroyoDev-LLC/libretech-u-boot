// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2015 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 */

#include <bootsource.h>
#include <dm.h>
#include <dm/lists.h>
#include <bitfield.h>
#include <env.h>
#include <errno.h>
#include <log.h>
#include <linux/bitfield.h>
#include <power/rk8xx_pmic.h>
#include <power/pmic.h>
#include <spi.h>
#include <sysreset.h>

static int rk8xx_sysreset_request(struct udevice *dev, enum sysreset_t type)
{
	struct rk8xx_priv *priv = dev_get_priv(dev->parent);

	if (type != SYSRESET_POWER_OFF)
		return -EPROTONOSUPPORT;

	switch (priv->variant) {
	case RK805_ID:
	case RK808_ID:
	case RK816_ID:
	case RK818_ID:
		pmic_clrsetbits(dev->parent, REG_DEVCTRL, 0, BIT(0));
		break;
	case RK809_ID:
	case RK817_ID:
		pmic_clrsetbits(dev->parent, RK817_REG_SYS_CFG3, 0,
				BIT(0));
		break;
	case RK806_ID:
		pmic_clrsetbits(dev->parent, RK806_REG_SYS_CFG3, 0,
				BIT(0));
		break;
	default:
		printf("Unknown PMIC RK%x: Cannot shutdown\n",
		       priv->variant);
		return -EPROTONOSUPPORT;
	};

	return -EINPROGRESS;
}

static struct sysreset_ops rk8xx_sysreset_ops = {
	.request	= rk8xx_sysreset_request,
};

U_BOOT_DRIVER(rk8xx_sysreset) = {
	.name		= "rk8xx_sysreset",
	.id		= UCLASS_SYSRESET,
	.ops		= &rk8xx_sysreset_ops,
};

#define RK808_INT_PLUG_IN	BIT(0)	/* INT_STS2, INT_STS_MSK2 */
#define RK808_INT_PLUG_OUT	BIT(1)

/*
 * The RK808 has no power-on reason register. Its PLUG_IN interrupt status
 * latches when power is applied, but also on a supply dip while the board
 * runs, and it survives a SoC reset. The interrupt masks tell the two apart:
 * they read 0 after the PMIC powers up and keep what was written while it
 * stays powered (Linux masks the PLUG interrupts, and so does this, on every
 * boot). So power was just applied only if PLUG_IN is set while the PLUG_IN
 * mask is still 0. The masks are set on every boot, and the status cleared
 * after a power-up, so a supply glitch while the board is off waiting for the
 * button cannot make the press read as a plug-in. Runs once, at probe.
 * Returns 1 power applied, 0 not, -EIO when a register could not be read.
 */
static int rk808_power_applied(struct udevice *dev)
{
	int sts = pmic_reg_read(dev, REG_INT_STS2);
	int msk = pmic_reg_read(dev, REG_INT_STS_MSK2);
	bool cold;

	if (sts < 0 || msk < 0)
		return -EIO;

	cold = !(msk & RK808_INT_PLUG_IN) && (sts & RK808_INT_PLUG_IN);
	pmic_reg_write(dev, REG_INT_STS_MSK2,
		       msk | RK808_INT_PLUG_IN | RK808_INT_PLUG_OUT);
	if (cold)
		pmic_reg_write(dev, REG_INT_STS2, sts);	/* write-1-to-clear */

	return cold;
}

static void rk808_dev_off(struct udevice *dev, const char *who)
{
	printf("%s: power attached, off until the power button\n", who);
	if (pmic_clrsetbits(dev, REG_DEVCTRL, 0, BIT(0)))
		printf("%s: DEV_OFF failed, booting\n", who);
}

static void rk808_off_for_plugin(struct udevice *dev)
{
	struct rk8xx_priv *priv = dev_get_priv(dev);

	if (priv->power_applied == 1)
		rk808_dev_off(dev, "RK808");
}

#if IS_ENABLED(CONFIG_ROCKCHIP_RK8XX_POWER_ON_ATTACH)
void rk8xx_power_on_attach(void)
{
	const char *val = env_get("power_on_attach");
	struct rk8xx_priv *priv;
	struct udevice *dev;

	/* An on/off platform key takes on|1 and off|0 (board-config.md) */
	if (!val || !strcmp(val, "on") || !strcmp(val, "1"))
		return;
	if (strcmp(val, "off") && strcmp(val, "0")) {
		printf("power_on_attach: unknown value '%s', booting\n", val);
		return;
	}
	/* A MaskROM USB boot is the recovery path: never refuse it */
	if (IS_ENABLED(CONFIG_BOOTSOURCE) && bootsource_get() == BOOTSOURCE_USB)
		return;
	if (uclass_get_device_by_driver(UCLASS_PMIC,
					DM_DRIVER_GET(rockchip_rk805), &dev)) {
		printf("power_on_attach: no RK8xx PMIC, booting\n");
		return;
	}
	priv = dev_get_priv(dev);
	if (priv->variant != RK808_ID) {
		printf("power_on_attach: not supported on RK%x, booting\n",
		       priv->variant);
		return;
	}
	if (priv->power_applied < 0) {
		printf("power_on_attach: cannot read the PMIC, booting\n");
		return;
	}
	if (priv->power_applied)
		rk808_dev_off(dev, "power_on_attach");
}
#endif

/* In the event of a plug-in and the appropriate option has been
 * selected, we simply shutdown instead of continue the normal boot
 * process.
 */
void rk8xx_off_for_plugin(struct udevice *dev)
{
	struct rk8xx_priv *priv = dev_get_priv(dev);

	switch (priv->variant) {
	case RK808_ID:
		rk808_off_for_plugin(dev);
		break;
	case RK805_ID:
	case RK816_ID:
	case RK818_ID:
		if (pmic_reg_read(dev, RK8XX_ON_SOURCE) & RK8XX_ON_PLUG_IN) {
			printf("Power Off due to plug-in event\n");
			pmic_clrsetbits(dev, REG_DEVCTRL, 0, BIT(0));
		}
		break;
	case RK809_ID:
	case RK817_ID:
		if (pmic_reg_read(dev, RK817_ON_SOURCE) & RK8XX_ON_PLUG_IN) {
			printf("Power Off due to plug-in event\n");
			pmic_clrsetbits(dev, RK817_REG_SYS_CFG3, 0,
					BIT(0));
		}
		break;
	default:
		printf("PMIC RK%x: Cannot read boot reason.\n",
		       priv->variant);
	}
}

static struct reg_data rk817_init_reg[] = {
/* enable the under-voltage protection,
 * the under-voltage protection will shutdown the LDO3 and reset the PMIC
 */
	{ RK817_BUCK4_CMIN, 0x60, 0x60},
};

static const struct pmic_child_info pmic_children_info[] = {
	{ .prefix = "DCDC_REG", .driver = "rk8xx_buck"},
	{ .prefix = "dcdc-reg", .driver = "rk8xx_buck"},
	{ .prefix = "LDO_REG", .driver = "rk8xx_ldo"},
	{ .prefix = "nldo-reg", .driver = "rk8xx_nldo"},
	{ .prefix = "pldo-reg", .driver = "rk8xx_pldo"},
	{ .prefix = "SWITCH_REG", .driver = "rk8xx_switch"},
	{ },
};

static int rk8xx_reg_count(struct udevice *dev)
{
	return RK808_NUM_OF_REGS;
}

#if CONFIG_IS_ENABLED(SPI) && CONFIG_IS_ENABLED(DM_SPI)
struct rk806_cmd {
	uint8_t	len: 4; /* Payload size in bytes - 1 */
	uint8_t	reserved: 2;
	uint8_t	crc_en: 1;
	uint8_t	op: 1; /* READ=0; WRITE=1; */
	uint8_t	reg_l;
#define REG_L_MASK	GENMASK(7, 0)
	uint8_t	reg_h;
#define REG_H_MASK	GENMASK(15, 8)
};
#endif

static int rk8xx_write(struct udevice *dev, uint reg, const uint8_t *buff,
			  int len)
{
	int ret;

#if CONFIG_IS_ENABLED(SPI) && CONFIG_IS_ENABLED(DM_SPI)
	if (device_get_uclass_id(dev->parent) == UCLASS_SPI) {
		struct spi_slave *spi = dev_get_parent_priv(dev);
		struct rk806_cmd cmd = {
			.op = 1,
			.len = len - 1,
			.reg_l = FIELD_GET(REG_L_MASK, reg),
			.reg_h = FIELD_GET(REG_H_MASK, reg),
		};

		ret = dm_spi_claim_bus(dev);
		if (ret) {
			debug("Couldn't claim bus for device: %p!\n", dev);
			return ret;
		}

		ret = spi_write_then_read(spi, (u8 *)&cmd, sizeof(cmd), buff, NULL, len);
		if (ret)
			debug("write error to device: %p register: %#x!\n",
			      dev, reg);

		dm_spi_release_bus(dev);

		return ret;
	}
#endif

	ret = dm_i2c_write(dev, reg, buff, len);
	if (ret) {
		debug("write error to device: %p register: %#x!\n", dev, reg);
		return ret;
	}

	return 0;
}

static int rk8xx_read(struct udevice *dev, uint reg, uint8_t *buff, int len)
{
	int ret;

#if CONFIG_IS_ENABLED(SPI) && CONFIG_IS_ENABLED(DM_SPI)
	if (device_get_uclass_id(dev->parent) == UCLASS_SPI) {
		struct spi_slave *spi = dev_get_parent_priv(dev);
		struct rk806_cmd cmd = {
			.op = 0,
			.len = len - 1,
			.reg_l = FIELD_GET(REG_L_MASK, reg),
			.reg_h = FIELD_GET(REG_H_MASK, reg),
		};

		ret = dm_spi_claim_bus(dev);
		if (ret) {
			debug("Couldn't claim bus for device: %p!\n", dev);
			return ret;
		}

		ret = spi_write_then_read(spi, (u8 *)&cmd, sizeof(cmd), NULL, buff, len);
		if (ret)
			debug("read error to device: %p register: %#x!\n",
			      dev, reg);

		dm_spi_release_bus(dev);

		return ret;
	}
#endif

	ret = dm_i2c_read(dev, reg, buff, len);
	if (ret) {
		debug("read error from device: %p register: %#x!\n", dev, reg);
		return ret;
	}

	return 0;
}

#if CONFIG_IS_ENABLED(PMIC_CHILDREN)
static int rk8xx_bind(struct udevice *dev)
{
	ofnode regulators_node;
	int children, ret;

	regulators_node = dev_read_subnode(dev, "regulators");
	if (!ofnode_valid(regulators_node)) {
		debug("%s: %s regulators subnode not found!\n", __func__,
		      dev->name);
		return -ENXIO;
	}

	debug("%s: '%s' - found regulators subnode\n", __func__, dev->name);

	if (CONFIG_IS_ENABLED(SYSRESET) &&
	    (dev_read_bool(dev, "rockchip,system-power-controller") ||
	     dev_read_bool(dev, "system-power-controller"))) {
		ret = device_bind_driver_to_node(dev, "rk8xx_sysreset",
						 "rk8xx_sysreset",
						 dev_ofnode(dev), NULL);
		if (ret)
			return ret;
	}

	children = pmic_bind_children(dev, regulators_node, pmic_children_info);
	if (!children)
		debug("%s: %s - no child found\n", __func__, dev->name);

	/* The RK805's OUT1/OUT2, on the PMIC's own gpio-controller node */
	if (CONFIG_IS_ENABLED(RK805_GPIO) &&
	    device_is_compatible(dev, "rockchip,rk805") &&
	    dev_read_bool(dev, "gpio-controller")) {
		ret = device_bind_driver_to_node(dev, "rk805_gpio", "gpio",
						 dev_ofnode(dev), NULL);
		if (ret)
			return ret;
	}

	if (IS_ENABLED(CONFIG_XPL_BUILD) &&
	    IS_ENABLED(CONFIG_ROCKCHIP_RK8XX_DISABLE_BOOT_ON_POWERON))
		dev_or_flags(dev, DM_FLAG_PROBE_AFTER_BIND);

	/* Always return success for this device */
	return 0;
}
#endif

static int rk8xx_probe(struct udevice *dev)
{
	struct rk8xx_priv *priv = dev_get_priv(dev);
	struct reg_data *init_data = NULL;
	int init_data_num = 0;
	int ret = 0, i, show_variant;
	u8 msb, lsb, id_msb, id_lsb;
	u8 on_source = 0, off_source = 0;
	u8 power_en0, power_en1, power_en2, power_en3;
	u8 value;

	/* read Chip variant */
	if (device_is_compatible(dev, "rockchip,rk817") ||
	    device_is_compatible(dev, "rockchip,rk809")) {
		id_msb = RK817_ID_MSB;
		id_lsb = RK817_ID_LSB;
	} else if (device_is_compatible(dev, "rockchip,rk806")) {
		id_msb = RK806_ID_MSB;
		id_lsb = RK806_ID_LSB;
	} else {
		id_msb = ID_MSB;
		id_lsb = ID_LSB;
	}

	ret = rk8xx_read(dev, id_msb, &msb, 1);
	if (ret)
		return ret;
	ret = rk8xx_read(dev, id_lsb, &lsb, 1);
	if (ret)
		return ret;

	priv->variant = ((msb << 8) | lsb) & RK8XX_ID_MSK;
	show_variant = bitfield_extract_by_mask(priv->variant, RK8XX_ID_MSK);
	switch (priv->variant) {
	case RK808_ID:
		/* RK808 ID is 0x0000, so fix show_variant for that PMIC */
		show_variant = 0x808;
		break;
	case RK805_ID:
	case RK816_ID:
	case RK818_ID:
		on_source = RK8XX_ON_SOURCE;
		off_source = RK8XX_OFF_SOURCE;
		break;
	case RK809_ID:
	case RK817_ID:
		on_source = RK817_ON_SOURCE;
		off_source = RK817_OFF_SOURCE;
		init_data = rk817_init_reg;
		init_data_num = ARRAY_SIZE(rk817_init_reg);
		power_en0 = pmic_reg_read(dev, RK817_POWER_EN0);
		power_en1 = pmic_reg_read(dev, RK817_POWER_EN1);
		power_en2 = pmic_reg_read(dev, RK817_POWER_EN2);
		power_en3 = pmic_reg_read(dev, RK817_POWER_EN3);

		value = (power_en0 & 0x0f) | ((power_en1 & 0x0f) << 4);
		pmic_reg_write(dev, RK817_POWER_EN_SAVE0, value);
		value = (power_en2 & 0x0f) | ((power_en3 & 0x0f) << 4);
		pmic_reg_write(dev, RK817_POWER_EN_SAVE1, value);
		break;
	case RK806_ID: {
		u32 rst_fun = 2;

		on_source = RK806_ON_SOURCE;
		off_source = RK806_OFF_SOURCE;

		ret = dev_read_u32(dev, "rockchip,reset-mode", &rst_fun);
		if (ret)
			debug("rockchip,reset-mode property missing, defaulting to %d\n",
			      rst_fun);

		pmic_clrsetbits(dev, RK806_REG_SYS_CFG3, RK806_RST_FUN_MSK,
				FIELD_PREP(RK806_RST_FUN_MSK, rst_fun));
		break; }
	default:
		printf("Unknown PMIC: RK%x!!\n", show_variant);
		return -EINVAL;
	}

	for (i = 0; i < init_data_num; i++) {
		ret = pmic_clrsetbits(dev,
				      init_data[i].reg,
				      init_data[i].mask,
				      init_data[i].val);
		if (ret < 0) {
			printf("%s: i2c set reg 0x%x failed, ret=%d\n",
			       __func__, init_data[i].reg, ret);
		}

		debug("%s: reg[0x%x] = 0x%x\n", __func__, init_data[i].reg,
		      pmic_reg_read(dev, init_data[i].reg));
	}

	if (!IS_ENABLED(CONFIG_XPL_BUILD)) {
		printf("PMIC:  RK%x ", show_variant);
		if (on_source && off_source)
			printf("(on=0x%02x, off=0x%02x)",
			       pmic_reg_read(dev, on_source),
			       pmic_reg_read(dev, off_source));
		printf("\n");
	}

	priv->power_applied = -ENODATA;
	if (priv->variant == RK808_ID &&
	    (IS_ENABLED(CONFIG_ROCKCHIP_RK8XX_DISABLE_BOOT_ON_POWERON) ||
	     (IS_ENABLED(CONFIG_ROCKCHIP_RK8XX_POWER_ON_ATTACH) &&
	      !IS_ENABLED(CONFIG_XPL_BUILD))))
		priv->power_applied = rk808_power_applied(dev);

	if (IS_ENABLED(CONFIG_ROCKCHIP_RK8XX_DISABLE_BOOT_ON_POWERON))
		rk8xx_off_for_plugin(dev);

	return 0;
}

static struct dm_pmic_ops rk8xx_ops = {
	.reg_count = rk8xx_reg_count,
	.read = rk8xx_read,
	.write = rk8xx_write,
};

static const struct udevice_id rk8xx_ids[] = {
	{ .compatible = "rockchip,rk805" },
	{ .compatible = "rockchip,rk806" },
	{ .compatible = "rockchip,rk808" },
	{ .compatible = "rockchip,rk809" },
	{ .compatible = "rockchip,rk816" },
	{ .compatible = "rockchip,rk817" },
	{ .compatible = "rockchip,rk818" },
	{ }
};

U_BOOT_DRIVER(rockchip_rk805) = {
	.name = "rockchip_rk805",
	.id = UCLASS_PMIC,
	.of_match = rk8xx_ids,
#if CONFIG_IS_ENABLED(PMIC_CHILDREN)
	.bind = rk8xx_bind,
#endif
	.priv_auto	  = sizeof(struct rk8xx_priv),
	.probe = rk8xx_probe,
	.ops = &rk8xx_ops,
};

DM_DRIVER_ALIAS(rockchip_rk805, rockchip_rk808)
