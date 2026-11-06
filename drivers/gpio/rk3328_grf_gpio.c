// SPDX-License-Identifier: GPL-2.0+
/*
 * RK3328 GRF GPIO_MUTE: one output-only line at GRF_SOC_CON10[1]
 * ("rockchip,rk3328-grf-gpio", a child of the GRF syscon), the same bit
 * Linux drives through gpio-syscon. Boards use it as the SD card's IO
 * voltage select (ROC-RK3328-CC vcc_sdio: high is 1.8 V, low 3.3 V) or as
 * an audio mute.
 */

#include <dm.h>
#include <regmap.h>
#include <syscon.h>
#include <asm/gpio.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/errno.h>

#define RK3328_GRF_SOC_CON10		0x0428
#define RK3328_GRF_GPIO_MUTE_BIT	1

struct rk3328_grf_gpio_priv {
	struct regmap *grf;
};

static int rk3328_grf_gpio_get_value(struct udevice *dev, unsigned int offset)
{
	struct rk3328_grf_gpio_priv *priv = dev_get_priv(dev);
	uint val;
	int ret;

	ret = regmap_read(priv->grf, RK3328_GRF_SOC_CON10, &val);
	if (ret)
		return ret;

	return !!(val & BIT(RK3328_GRF_GPIO_MUTE_BIT));
}

static int rk3328_grf_gpio_set_value(struct udevice *dev, unsigned int offset,
				     int value)
{
	struct rk3328_grf_gpio_priv *priv = dev_get_priv(dev);

	/* the GRF's upper half is the write mask */
	return regmap_write(priv->grf, RK3328_GRF_SOC_CON10,
			    (value ? BIT(RK3328_GRF_GPIO_MUTE_BIT) : 0) |
			    BIT(RK3328_GRF_GPIO_MUTE_BIT + 16));
}

static int rk3328_grf_gpio_direction_input(struct udevice *dev,
					   unsigned int offset)
{
	return -ENOSYS;
}

static int rk3328_grf_gpio_direction_output(struct udevice *dev,
					    unsigned int offset, int value)
{
	return rk3328_grf_gpio_set_value(dev, offset, value);
}

static int rk3328_grf_gpio_get_function(struct udevice *dev,
					unsigned int offset)
{
	return GPIOF_OUTPUT;
}

static const struct dm_gpio_ops rk3328_grf_gpio_ops = {
	.direction_input	= rk3328_grf_gpio_direction_input,
	.direction_output	= rk3328_grf_gpio_direction_output,
	.get_value		= rk3328_grf_gpio_get_value,
	.set_value		= rk3328_grf_gpio_set_value,
	.get_function		= rk3328_grf_gpio_get_function,
};

static int rk3328_grf_gpio_probe(struct udevice *dev)
{
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);
	struct rk3328_grf_gpio_priv *priv = dev_get_priv(dev);

	priv->grf = syscon_get_regmap(dev_get_parent(dev));
	if (IS_ERR(priv->grf))
		return PTR_ERR(priv->grf);

	uc_priv->gpio_count = 1;
	uc_priv->bank_name = "grf_gpio";

	return 0;
}

static const struct udevice_id rk3328_grf_gpio_ids[] = {
	{ .compatible = "rockchip,rk3328-grf-gpio" },
	{ }
};

U_BOOT_DRIVER(rk3328_grf_gpio) = {
	.name		= "rk3328_grf_gpio",
	.id		= UCLASS_GPIO,
	.of_match	= rk3328_grf_gpio_ids,
	.ops		= &rk3328_grf_gpio_ops,
	.probe		= rk3328_grf_gpio_probe,
	.priv_auto	= sizeof(struct rk3328_grf_gpio_priv),
};
