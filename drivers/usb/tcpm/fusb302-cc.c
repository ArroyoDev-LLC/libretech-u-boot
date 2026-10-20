// SPDX-License-Identifier: GPL-2.0+
/*
 * FUSB302 plug orientation for a phase without the port manager (SPL).
 *
 * A sink's CC pull-downs are on from reset; the source's pull-up sits on
 * whichever CC line the plug connected, so measuring each line once tells
 * which way round the plug is. U-Boot proper learns the same from the full
 * driver's toggle state machine (fusb302.c); this is the few register
 * accesses an SPL can afford, for a Type-C PHY that has to map its
 * SuperSpeed lanes before the host trains the link. The registers it touches
 * are put back, so proper's driver finds the chip as reset left it.
 */

#include <dm.h>
#include <i2c.h>
#include <linux/delay.h>
#include <usb/tcpm.h>
#include "fusb302_reg.h"

/* BC_LVL comparator settling after the measure switch moves, generous */
#define FUSB302_CC_SETTLE_US	1000

static int fusb302_cc_level(struct udevice *chip, u8 switches0, u8 meas)
{
	int ret;

	ret = dm_i2c_reg_write(chip, FUSB_REG_SWITCHES0, switches0 | meas);
	if (ret)
		return ret;
	udelay(FUSB302_CC_SETTLE_US);

	ret = dm_i2c_reg_read(chip, FUSB_REG_STATUS0);
	if (ret < 0)
		return ret;

	return ret & FUSB_REG_STATUS0_BC_LVL_MASK;
}

int fusb302_cc_orientation(struct udevice *chip)
{
	int power, switches0, cc1, cc2, ret;

	power = dm_i2c_reg_read(chip, FUSB_REG_POWER);
	switches0 = dm_i2c_reg_read(chip, FUSB_REG_SWITCHES0);
	if (power < 0 || switches0 < 0)
		return power < 0 ? power : switches0;

	/* The measure block, and nothing measuring until we say which line */
	ret = dm_i2c_reg_write(chip, FUSB_REG_POWER, FUSB_REG_POWER_PWR_HIGH);
	if (ret)
		return ret;
	switches0 &= ~(FUSB_REG_SWITCHES0_MEAS_CC1 | FUSB_REG_SWITCHES0_MEAS_CC2);

	cc1 = fusb302_cc_level(chip, switches0, FUSB_REG_SWITCHES0_MEAS_CC1);
	cc2 = fusb302_cc_level(chip, switches0, FUSB_REG_SWITCHES0_MEAS_CC2);

	dm_i2c_reg_write(chip, FUSB_REG_SWITCHES0, switches0);
	dm_i2c_reg_write(chip, FUSB_REG_POWER, power);

	if (cc1 < 0 || cc2 < 0)
		return cc1 < 0 ? cc1 : cc2;
	/* Exactly one line carries the source's pull-up */
	if (cc1 && !cc2)
		return TYPEC_ORIENTATION_NORMAL;
	if (cc2 && !cc1)
		return TYPEC_ORIENTATION_REVERSE;

	return TYPEC_ORIENTATION_NONE;
}
