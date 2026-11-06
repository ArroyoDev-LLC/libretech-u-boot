// SPDX-License-Identifier: GPL-2.0+
/*
 * The two output-only GPIOs of the Rockchip RK805 PMIC (OUT1, OUT2).
 *
 * Linux drivers/pinctrl/pinctrl-rk805.c is the reference: both pins are
 * output only, and their levels are bits 0 and 1 of RK805_OUT_REG (0x52), a
 * set bit driving the pin high. The RK805 datasheet: OUTn is a CMOS output
 * whose high level is the FB4 rail. Bound by rk8xx.c on the PMIC's own node,
 * which carries gpio-controller, so <&rk805 N flags> resolves to this device.
 */

#include <dm.h>
#include <errno.h>
#include <asm/gpio.h>
#include <linux/bitops.h>
#include <power/pmic.h>
#include <power/rk8xx_pmic.h>

#define RK805_OUT_REG		0x52
#define RK805_GPIO_COUNT	2

static int rk805_gpio_direction_input(struct udevice *dev, unsigned int offset)
{
	return -ENOSYS;		/* output only */
}

static int rk805_gpio_set_value(struct udevice *dev, unsigned int offset,
				int value)
{
	if (offset >= RK805_GPIO_COUNT)
		return -EINVAL;

	return pmic_clrsetbits(dev->parent, RK805_OUT_REG, BIT(offset),
			       value ? BIT(offset) : 0);
}

static int rk805_gpio_direction_output(struct udevice *dev,
				       unsigned int offset, int value)
{
	return rk805_gpio_set_value(dev, offset, value);
}

static int rk805_gpio_get_value(struct udevice *dev, unsigned int offset)
{
	int ret;

	if (offset >= RK805_GPIO_COUNT)
		return -EINVAL;
	ret = pmic_reg_read(dev->parent, RK805_OUT_REG);
	if (ret < 0)
		return ret;

	return !!(ret & BIT(offset));
}

static int rk805_gpio_get_function(struct udevice *dev, unsigned int offset)
{
	return GPIOF_OUTPUT;
}

static const struct dm_gpio_ops rk805_gpio_ops = {
	.direction_input	= rk805_gpio_direction_input,
	.direction_output	= rk805_gpio_direction_output,
	.get_value		= rk805_gpio_get_value,
	.set_value		= rk805_gpio_set_value,
	.get_function		= rk805_gpio_get_function,
};

static int rk805_gpio_probe(struct udevice *dev)
{
	struct gpio_dev_priv *uc_priv = dev_get_uclass_priv(dev);

	uc_priv->gpio_count = RK805_GPIO_COUNT;
	uc_priv->bank_name = "rk805-out";

	return 0;
}

U_BOOT_DRIVER(rk805_gpio) = {
	.name	= "rk805_gpio",
	.id	= UCLASS_GPIO,
	.probe	= rk805_gpio_probe,
	.ops	= &rk805_gpio_ops,
};
