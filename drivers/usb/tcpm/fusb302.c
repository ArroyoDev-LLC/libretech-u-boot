// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2016-2017 Google, Inc
 *
 * Fairchild FUSB302 Type-C Chip Driver
 */

#include <dm.h>
#include <env.h>
#include <i2c.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <dm/device_compat.h>
#include <power/regulator.h>
#include <usb/tcpm.h>
#include "fusb302_reg.h"

#define FUSB302_MAX_MSG_LEN 0x1F

enum toggling_mode {
	TOGGLING_MODE_OFF,
	TOGGLING_MODE_DRP,
	TOGGLING_MODE_SNK,
	TOGGLING_MODE_SRC,
};

enum src_current_status {
	SRC_CURRENT_DEFAULT,
	SRC_CURRENT_MEDIUM,
	SRC_CURRENT_HIGH,
};

static const u8 ra_mda_value[] = {
	[SRC_CURRENT_DEFAULT] = 4,	/* 210mV */
	[SRC_CURRENT_MEDIUM] = 9,	/* 420mV */
	[SRC_CURRENT_HIGH] = 18,	/* 798mV */
};

static const u8 rd_mda_value[] = {
	[SRC_CURRENT_DEFAULT] = 38,	/* 1638mV */
	[SRC_CURRENT_MEDIUM] = 38,	/* 1638mV */
	[SRC_CURRENT_HIGH] = 61,	/* 2604mV */
};

struct fusb302_chip {
	enum toggling_mode toggling_mode;
	enum src_current_status src_current_status;
	bool intr_togdone;
	bool intr_bc_lvl;
	bool intr_comp_chng;

	/*
	 * Whether the port manager currently wants PD messages reported.
	 * Shadowed for the same reason as intr_*: a wake has to restore the
	 * configuration the port is IN, and HARD_RESET_START deliberately
	 * turns this off for the duration of the reset window.
	 */
	bool intr_pd;
	/* set_pd_rx() has run once, so intr_pd is the chip's real PD RX state */
	bool pd_rx_on_known;
	/* a BC_LVL read was deferred because PD traffic was on the line */
	bool bc_lvl_pending;
	/* COMP read high as a source: a detach waiting for confirmation */
	bool comp_pending;
	ulong comp_since;

	/* The port's VBUS rail, from vbus-supply. NULL if the board has none */
	struct udevice *vbus;

	/* INT_N as a GPIO input, from the DT interrupt; invalid if unrouted */
	struct gpio_desc int_n;

	/* SWITCHES1 as the previous session left it, read before the reset */
	u8 prior_switches1;

	/* port status */
	bool vconn_on;
	bool vbus_present;
	enum typec_cc_polarity cc_polarity;
	enum typec_cc_status cc1;
	enum typec_cc_status cc2;
};

static int fusb302_i2c_write(struct udevice *dev, u8 address, u8 data)
{
	int ret;

	ret = dm_i2c_write(dev, address, &data, 1);
	if (ret)
		dev_err(dev, "cannot write 0x%02x to 0x%02x, ret=%d\n",
			data, address, ret);

	return ret;
}

static int fusb302_i2c_block_write(struct udevice *dev, u8 address,
				   u8 length, const u8 *data)
{
	int ret;

	if (!length)
		return 0;

	ret = dm_i2c_write(dev, address, data, length);
	if (ret)
		dev_err(dev, "cannot block write 0x%02x, len=%d, ret=%d\n",
			address, length, ret);

	return ret;
}

static int fusb302_i2c_read(struct udevice *dev, u8 address, u8 *data)
{
	int ret, retries;

	for (retries = 0; retries < 3; retries++) {
		ret = dm_i2c_read(dev, address, data, 1);
		if (ret == 0)
			return ret;
		dev_err(dev, "cannot read %02x, ret=%d\n", address, ret);
	}

	return ret;
}

static int fusb302_i2c_block_read(struct udevice *dev, u8 address,
				  u8 length, u8 *data)
{
	int ret;

	if (!length)
		return 0;

	ret = dm_i2c_read(dev, address, data, length);
	if (ret)
		dev_err(dev, "cannot block read 0x%02x, len=%d, ret=%d\n",
			address, length, ret);
	return ret;
}

static int fusb302_i2c_mask_write(struct udevice *dev, u8 address,
				  u8 mask, u8 value)
{
	int ret;
	u8 data;

	ret = fusb302_i2c_read(dev, address, &data);
	if (ret)
		return ret;
	data &= ~mask;
	data |= value;
	ret = fusb302_i2c_write(dev, address, data);
	if (ret)
		return ret;

	return ret;
}

static int fusb302_i2c_set_bits(struct udevice *dev, u8 address, u8 set_bits)
{
	return fusb302_i2c_mask_write(dev, address, 0x00, set_bits);
}

static int fusb302_i2c_clear_bits(struct udevice *dev, u8 address, u8 clear_bits)
{
	return fusb302_i2c_mask_write(dev, address, clear_bits, 0x00);
}

static int fusb302_sw_reset(struct udevice *dev)
{
	int ret = fusb302_i2c_write(dev, FUSB_REG_RESET, FUSB_REG_RESET_SW_RESET);

	if (ret)
		dev_err(dev, "cannot sw reset the fusb302: %d\n", ret);

	return ret;
}

static int fusb302_enable_tx_auto_retries(struct udevice *dev, u8 retry_count)
{
	int ret;

	ret = fusb302_i2c_set_bits(dev, FUSB_REG_CONTROL3, retry_count |
				   FUSB_REG_CONTROL3_AUTO_RETRY);

	return ret;
}

/*
 * mask all interrupt on the chip
 */
static int fusb302_mask_interrupt(struct udevice *dev)
{
	int ret;

	ret = fusb302_i2c_write(dev, FUSB_REG_MASK, 0xFF);
	if (ret)
		return ret;
	ret = fusb302_i2c_write(dev, FUSB_REG_MASKA, 0xFF);
	if (ret)
		return ret;
	ret = fusb302_i2c_write(dev, FUSB_REG_MASKB, 0xFF);
	if (ret)
		return ret;
	ret = fusb302_i2c_set_bits(dev, FUSB_REG_CONTROL0,
				   FUSB_REG_CONTROL0_INT_MASK);
		return ret;
}

/*
 * initialize interrupt on the chip
 * - unmasked interrupt: VBUS_OK
 */
static int fusb302_init_interrupt(struct udevice *dev)
{
	int ret;

	ret = fusb302_i2c_write(dev, FUSB_REG_MASK,
				0xFF & ~FUSB_REG_MASK_VBUSOK);
	if (ret)
		return ret;
	ret = fusb302_i2c_write(dev, FUSB_REG_MASKA, 0xFF);
	if (ret)
		return ret;
	ret = fusb302_i2c_write(dev, FUSB_REG_MASKB, 0xFF);
	if (ret)
		return ret;
	ret = fusb302_i2c_clear_bits(dev, FUSB_REG_CONTROL0,
				     FUSB_REG_CONTROL0_INT_MASK);
	return ret;
}

/*
 * INT_N goes low while any unmasked interrupt is pending and high once the
 * handler has read them all, so its level says whether a poll has anything
 * to find. The DT routes it as an interrupt on a GPIO bank (interrupt-parent
 * + interrupts = <pin type>); read that pin as an input. Without DM_GPIO or
 * such a route the port is polled on every tick, as before.
 */
static void fusb302_get_int_n(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	ofnode bank;
	u32 pin, type;

	if (!CONFIG_IS_ENABLED(DM_GPIO) || dm_gpio_is_valid(&chip->int_n))
		return;

	bank = ofnode_parse_phandle(dev_ofnode(dev), "interrupt-parent", 0);
	if (!ofnode_valid(bank) ||
	    dev_read_u32_index(dev, "interrupts", 0, &pin) ||
	    dev_read_u32_index(dev, "interrupts", 1, &type) ||
	    uclass_get_device_by_ofnode(UCLASS_GPIO, bank, &chip->int_n.dev))
		goto none;

	chip->int_n.offset = pin;
	chip->int_n.flags = 0;
	if (dm_gpio_request(&chip->int_n, "fusb302-int-n"))
		goto none;
	/* IRQ_TYPE_LEVEL_LOW / EDGE_FALLING: the pin is low when asserted */
	if (dm_gpio_set_dir_flags(&chip->int_n, GPIOD_IS_IN |
				  ((type & 0xa) ? GPIOD_ACTIVE_LOW : 0))) {
		dm_gpio_free(dev, &chip->int_n);
		goto none;
	}
	return;

none:
	chip->int_n.dev = NULL;
	dev_dbg(dev, "INT_N not readable; polling every tick\n");
}

static int fusb302_get_prior_roles(struct udevice *dev, enum typec_role *role,
				   enum typec_data_role *data)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	u8 sw1 = chip->prior_switches1;

	if (!(sw1 & FUSB_REG_SWITCHES1_AUTO_GCRC))
		return -ENOENT;

	*role = sw1 & FUSB_REG_SWITCHES1_POWERROLE ? TYPEC_SOURCE : TYPEC_SINK;
	*data = sw1 & FUSB_REG_SWITCHES1_DATAROLE ? TYPEC_HOST : TYPEC_DEVICE;

	return 0;
}

static bool fusb302_event_pending(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);

	if (!CONFIG_IS_ENABLED(DM_GPIO) || !dm_gpio_is_valid(&chip->int_n) ||
	    chip->bc_lvl_pending || chip->comp_pending)
		return true;

	return dm_gpio_get_value(&chip->int_n) != 0;
}

static int fusb302_set_power_mode(struct udevice *dev, u8 power_mode)
{
	int ret;

	ret = fusb302_i2c_write(dev, FUSB_REG_POWER, power_mode);

	return ret;
}

static int fusb302_init(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;
	u8 data;

	if (CONFIG_IS_ENABLED(DM_REGULATOR)) {
		/*
		 * Optional: a sink-only port sources no VBUS and needs no
		 * rail. Resolve it here rather than at set_vbus() time so a
		 * missing regulator is a boot-time diagnostic and not a
		 * failure in the middle of a state transition.
		 */
		ret = device_get_supply_regulator(dev, "vbus-supply",
						  &chip->vbus);
		if (ret) {
			dev_dbg(dev, "no vbus-supply: %d\n", ret);
			chip->vbus = NULL;
		}
	}

	/*
	 * The chip is not reset with the SoC. SWITCHES1 still holds the roles
	 * the last session put in its GoodCRC headers, and AUTO_GCRC says
	 * whether that session spoke PD at all; the reset below clears both.
	 */
	if (fusb302_i2c_read(dev, FUSB_REG_SWITCHES1, &chip->prior_switches1))
		chip->prior_switches1 = 0;

	ret = fusb302_sw_reset(dev);
	if (ret)
		return ret;
	ret = fusb302_enable_tx_auto_retries(dev, FUSB_REG_CONTROL3_N_RETRIES_3);
	if (ret)
		return ret;
	ret = fusb302_init_interrupt(dev);
	if (ret)
		return ret;
	fusb302_get_int_n(dev);
	ret = fusb302_set_power_mode(dev, FUSB_REG_POWER_PWR_ALL);
	if (ret)
		return ret;
	ret = fusb302_i2c_read(dev, FUSB_REG_STATUS0, &data);
	if (ret)
		return ret;
	chip->vbus_present = !!(data & FUSB_REG_STATUS0_VBUSOK);
	ret = fusb302_i2c_read(dev, FUSB_REG_DEVICE_ID, &data);
	if (ret)
		return ret;
	dev_info(dev, "fusb302 device ID: 0x%02x\n", data);

	return ret;
}

static int fusb302_get_vbus(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);

	return chip->vbus_present ? 1 : 0;
}

static int fusb302_set_src_current(struct udevice *dev,
				   enum src_current_status status)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;

	chip->src_current_status = status;
	switch (status) {
	case SRC_CURRENT_DEFAULT:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_CONTROL0,
					     FUSB_REG_CONTROL0_HOST_CUR_MASK,
					     FUSB_REG_CONTROL0_HOST_CUR_DEF);
		break;
	case SRC_CURRENT_MEDIUM:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_CONTROL0,
					     FUSB_REG_CONTROL0_HOST_CUR_MASK,
					     FUSB_REG_CONTROL0_HOST_CUR_MED);
		break;
	case SRC_CURRENT_HIGH:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_CONTROL0,
					     FUSB_REG_CONTROL0_HOST_CUR_MASK,
					     FUSB_REG_CONTROL0_HOST_CUR_HIGH);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int fusb302_set_toggling(struct udevice *dev,
				enum toggling_mode mode)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;

	chip->comp_pending = false;

	/* first disable toggling */
	ret = fusb302_i2c_clear_bits(dev, FUSB_REG_CONTROL2,
				     FUSB_REG_CONTROL2_TOGGLE);
	if (ret)
		return ret;
	/* mask interrupts for SRC or SNK */
	ret = fusb302_i2c_set_bits(dev, FUSB_REG_MASK,
				   FUSB_REG_MASK_BC_LVL |
				   FUSB_REG_MASK_COMP_CHNG);
	if (ret)
		return ret;
	chip->intr_bc_lvl = false;
	chip->intr_comp_chng = false;
	/* configure toggling mode: none/snk/src/drp */
	switch (mode) {
	case TOGGLING_MODE_OFF:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_NONE);
		break;
	case TOGGLING_MODE_SNK:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_UFP);
		break;
	case TOGGLING_MODE_SRC:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_DFP);
		break;
	case TOGGLING_MODE_DRP:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_CONTROL2,
					     FUSB_REG_CONTROL2_MODE_MASK,
					     FUSB_REG_CONTROL2_MODE_DRP);
		break;
	default:
		break;
	}

	if (ret)
		return ret;

	if (mode == TOGGLING_MODE_OFF) {
		/* mask TOGDONE interrupt */
		ret = fusb302_i2c_set_bits(dev, FUSB_REG_MASKA,
					   FUSB_REG_MASKA_TOGDONE);
		if (ret)
			return ret;
		chip->intr_togdone = false;
	} else {
		/* Datasheet says vconn MUST be off when toggling */
		if (chip->vconn_on)
			dev_warn(dev, "Vconn is on during toggle start\n");
		/* unmask TOGDONE interrupt */
		ret = fusb302_i2c_clear_bits(dev, FUSB_REG_MASKA,
					     FUSB_REG_MASKA_TOGDONE);
		if (ret)
			return ret;
		chip->intr_togdone = true;
		/* start toggling */
		ret = fusb302_i2c_set_bits(dev, FUSB_REG_CONTROL2,
					   FUSB_REG_CONTROL2_TOGGLE);
		if (ret)
			return ret;
		/* during toggling, consider cc as Open */
		chip->cc1 = TYPEC_CC_OPEN;
		chip->cc2 = TYPEC_CC_OPEN;
	}
	chip->toggling_mode = mode;

	return ret;
}

static const enum src_current_status cc_src_current[] = {
	[TYPEC_CC_OPEN]		= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RA]		= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RD]		= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RP_DEF]	= SRC_CURRENT_DEFAULT,
	[TYPEC_CC_RP_1_5]	= SRC_CURRENT_MEDIUM,
	[TYPEC_CC_RP_3_0]	= SRC_CURRENT_HIGH,
};

static int fusb302_set_cc(struct udevice *dev, enum typec_cc_status cc)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	const u8 switches0_mask = FUSB_REG_SWITCHES0_CC1_PU_EN |
				  FUSB_REG_SWITCHES0_CC2_PU_EN |
				  FUSB_REG_SWITCHES0_CC1_PD_EN |
				  FUSB_REG_SWITCHES0_CC2_PD_EN;
	u8 rd_mda, switches0_data = 0x00;
	int ret;

	/* a pending COMP detach belongs to the configuration being replaced */
	chip->comp_pending = false;

	switch (cc) {
	case TYPEC_CC_OPEN:
		break;
	case TYPEC_CC_RD:
		switches0_data |= FUSB_REG_SWITCHES0_CC1_PD_EN |
				  FUSB_REG_SWITCHES0_CC2_PD_EN;
		break;
	case TYPEC_CC_RP_DEF:
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		switches0_data |= (chip->cc_polarity == TYPEC_POLARITY_CC1) ?
				  FUSB_REG_SWITCHES0_CC1_PU_EN :
				  FUSB_REG_SWITCHES0_CC2_PU_EN;
		break;
	default:
		dev_err(dev, "unsupported CC value: %s\n",
			typec_cc_status_name[cc]);
		ret = -EINVAL;
		goto done;
	}

	ret = fusb302_set_toggling(dev, TOGGLING_MODE_OFF);
	if (ret) {
		dev_err(dev, "cannot set toggling mode: %d\n", ret);
		goto done;
	}

	ret = fusb302_i2c_mask_write(dev, FUSB_REG_SWITCHES0,
				     switches0_mask, switches0_data);
	if (ret) {
		dev_err(dev, "cannot set pull-up/-down: %d\n", ret);
		goto done;
	}
	/* reset the cc status */
	chip->cc1 = TYPEC_CC_OPEN;
	chip->cc2 = TYPEC_CC_OPEN;

	/* adjust current for SRC */
	ret = fusb302_set_src_current(dev, cc_src_current[cc]);
	if (ret) {
		dev_err(dev, "cannot set src current %s: %d\n",
			typec_cc_status_name[cc], ret);
		goto done;
	}

	/* enable/disable interrupts, BC_LVL for SNK and COMP_CHNG for SRC */
	switch (cc) {
	case TYPEC_CC_RP_DEF:
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		rd_mda = rd_mda_value[cc_src_current[cc]];
		ret = fusb302_i2c_write(dev, FUSB_REG_MEASURE, rd_mda);
		if (ret) {
			dev_err(dev, "cannot set SRC measure value: %d\n", ret);
			goto done;
		}
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_MASK,
					     FUSB_REG_MASK_BC_LVL |
					     FUSB_REG_MASK_COMP_CHNG,
					     FUSB_REG_MASK_BC_LVL);
		if (ret) {
			dev_err(dev, "cannot set SRC irq: %d\n", ret);
			goto done;
		}
		chip->intr_comp_chng = true;
		break;
	case TYPEC_CC_RD:
		ret = fusb302_i2c_mask_write(dev, FUSB_REG_MASK,
					     FUSB_REG_MASK_BC_LVL |
					     FUSB_REG_MASK_COMP_CHNG,
					     FUSB_REG_MASK_COMP_CHNG);
		if (ret) {
			dev_err(dev, "cannot set SRC irq: %d\n", ret);
			goto done;
		}
		chip->intr_bc_lvl = true;
		break;
	default:
		break;
	}
done:
	return ret;
}

static int fusb302_get_cc(struct udevice *dev, enum typec_cc_status *cc1,
			  enum typec_cc_status *cc2)
{
	struct fusb302_chip *chip = dev_get_priv(dev);

	*cc1 = chip->cc1;
	*cc2 = chip->cc2;
	dev_dbg(dev, "get cc1 = %s, cc2 = %s\n", typec_cc_status_name[*cc1],
		typec_cc_status_name[*cc2]);

	return 0;
}

static int fusb302_set_vconn(struct udevice *dev, bool on)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;
	u8 switches0_data = 0x00;
	u8 switches0_mask = FUSB_REG_SWITCHES0_VCONN_CC1 |
			    FUSB_REG_SWITCHES0_VCONN_CC2;

	if (chip->vconn_on == on) {
		ret = 0;
		dev_dbg(dev, "vconn is already %s\n", on ? "on" : "off");
		goto done;
	}
	if (on) {
		switches0_data = (chip->cc_polarity == TYPEC_POLARITY_CC1) ?
				 FUSB_REG_SWITCHES0_VCONN_CC2 :
				 FUSB_REG_SWITCHES0_VCONN_CC1;
	}
	ret = fusb302_i2c_mask_write(dev, FUSB_REG_SWITCHES0,
				     switches0_mask, switches0_data);
	if (ret)
		goto done;
	/*
	 * The shadow every caller tests. It was never written, so every
	 * set_vconn(false) returned "already off" and VCONN, once on, stayed
	 * on through detach and the next toggle (Linux sets it here).
	 */
	chip->vconn_on = on;
	dev_dbg(dev, "set vconn = %s\n", on ? "on" : "off");
done:
	return ret;
}

/*
 * Source VBUS on or off.
 *
 * This was a stub returning success, which makes a source-role port a
 * contradiction: the port manager believes it has raised VBUS, the partner
 * never sees it, and so -- per the Type-C state machine -- the partner stays
 * in AttachWait.SNK and answers no PD message at all. The visible symptom is
 * not "no power", it is every PD transmission timing out against a device
 * that is plainly attached.
 *
 * The FUSB302 has no VBUS switch of its own; the rail is a board regulator
 * named by the chip node's vbus-supply. A board without one keeps the old
 * behaviour -- a sink-only port never asks for VBUS, so its absence is not
 * an error to report at init.
 */
static int fusb302_set_vbus(struct udevice *dev, bool on, bool charge)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;

	if (!CONFIG_IS_ENABLED(DM_REGULATOR) || !chip->vbus)
		return 0;

	ret = regulator_set_enable_if_allowed(chip->vbus, on);
	if (ret) {
		dev_err(dev, "cannot turn VBUS %s: %d\n", on ? "on" : "off",
			ret);
		return ret;
	}

	dev_dbg(dev, "set vbus = %s\n", on ? "on" : "off");

	return 0;
}

static int fusb302_pd_tx_flush(struct udevice *dev)
{
	return fusb302_i2c_set_bits(dev, FUSB_REG_CONTROL0,
				    FUSB_REG_CONTROL0_TX_FLUSH);
}

static int fusb302_pd_rx_flush(struct udevice *dev)
{
	return fusb302_i2c_set_bits(dev, FUSB_REG_CONTROL1,
				    FUSB_REG_CONTROL1_RX_FLUSH);
}

static int fusb302_pd_set_auto_goodcrc(struct udevice *dev, bool on)
{
	if (on)
		return fusb302_i2c_set_bits(dev, FUSB_REG_SWITCHES1,
					    FUSB_REG_SWITCHES1_AUTO_GCRC);
	return fusb302_i2c_clear_bits(dev, FUSB_REG_SWITCHES1,
				      FUSB_REG_SWITCHES1_AUTO_GCRC);
}

static int fusb302_pd_set_interrupts(struct udevice *dev, bool on)
{
	int ret;
	u8 mask_interrupts = FUSB_REG_MASK_COLLISION;
	u8 maska_interrupts = FUSB_REG_MASKA_RETRYFAIL |
			      FUSB_REG_MASKA_HARDSENT |
			      FUSB_REG_MASKA_TX_SUCCESS |
			      FUSB_REG_MASKA_HARDRESET;
	u8 maskb_interrupts = FUSB_REG_MASKB_GCRCSENT;

	ret = on ?
		fusb302_i2c_clear_bits(dev, FUSB_REG_MASK, mask_interrupts) :
		fusb302_i2c_set_bits(dev, FUSB_REG_MASK, mask_interrupts);
	if (ret)
		return ret;
	ret = on ?
		fusb302_i2c_clear_bits(dev, FUSB_REG_MASKA, maska_interrupts) :
		fusb302_i2c_set_bits(dev, FUSB_REG_MASKA, maska_interrupts);
	if (ret)
		return ret;
	ret = on ?
		fusb302_i2c_clear_bits(dev, FUSB_REG_MASKB, maskb_interrupts) :
		fusb302_i2c_set_bits(dev, FUSB_REG_MASKB, maskb_interrupts);
	return ret;
}

static int fusb302_set_pd_rx(struct udevice *dev, bool on)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;

	/*
	 * Already in the state asked for: do nothing. Flushing here dropped a
	 * message the chip had already GoodCRC'd -- SNK_WAIT_CAPABILITIES is
	 * re-entered after a Soft Reset or a failed Request, and a
	 * Source_Capabilities sitting in the FIFO was thrown away while the
	 * partner believed it delivered. Linux returns early the same way.
	 */
	if (chip->intr_pd == on && chip->pd_rx_on_known)
		return 0;

	ret = fusb302_pd_rx_flush(dev);
	if (ret) {
		dev_err(dev, "cannot flush pd rx buffer: %d\n", ret);
		goto done;
	}
	ret = fusb302_pd_tx_flush(dev);
	if (ret) {
		dev_err(dev, "cannot flush pd tx buffer: %d\n", ret);
		goto done;
	}
	ret = fusb302_pd_set_auto_goodcrc(dev, on);
	if (ret) {
		dev_err(dev, "cannot turn %s auto GoodCRC: %d\n",
			on ? "on" : "off", ret);
		goto done;
	}
	ret = fusb302_pd_set_interrupts(dev, on);
	if (ret) {
		dev_err(dev, "cannot turn %s pd interrupts: %d\n",
			on ? "on" : "off", ret);
		goto done;
	}
	chip->intr_pd = on;
	chip->pd_rx_on_known = true;
	dev_dbg(dev, "set pd RX %s\n", on ? "on" : "off");
done:
	return ret;
}

static int fusb302_set_roles(struct udevice *dev, bool attached,
			     enum typec_role pwr, enum typec_data_role data)
{
	int ret;
	u8 switches1_mask = FUSB_REG_SWITCHES1_POWERROLE |
			    FUSB_REG_SWITCHES1_DATAROLE;
	u8 switches1_data = 0x00;

	if (pwr == TYPEC_SOURCE)
		switches1_data |= FUSB_REG_SWITCHES1_POWERROLE;
	if (data == TYPEC_HOST)
		switches1_data |= FUSB_REG_SWITCHES1_DATAROLE;
	ret = fusb302_i2c_mask_write(dev, FUSB_REG_SWITCHES1,
				     switches1_mask, switches1_data);
	if (ret) {
		dev_err(dev, "unable to set pd header %s, %s, ret=%d\n",
			typec_role_name[pwr], typec_data_role_name[data], ret);
		goto done;
	}
	dev_dbg(dev, "pd header : %s, %s\n", typec_role_name[pwr],
		typec_data_role_name[data]);
done:

	return ret;
}

static int fusb302_start_toggling(struct udevice *dev,
				  enum typec_port_type port_type,
				  enum typec_cc_status cc)
{
	enum toggling_mode mode = TOGGLING_MODE_OFF;
	int ret;

	switch (port_type) {
	case TYPEC_PORT_SRC:
		mode = TOGGLING_MODE_SRC;
		break;
	case TYPEC_PORT_SNK:
		mode = TOGGLING_MODE_SNK;
		break;
	case TYPEC_PORT_DRP:
		mode = TOGGLING_MODE_DRP;
		break;
	}

	ret = fusb302_set_src_current(dev, cc_src_current[cc]);
	if (ret) {
		dev_err(dev, "unable to set src current %s, ret=%d",
			typec_cc_status_name[cc], ret);
		goto done;
	}
	ret = fusb302_set_toggling(dev, mode);
	if (ret) {
		dev_err(dev, "unable to start drp toggling: %d\n", ret);
		goto done;
	}
	dev_info(dev, "fusb302 start drp toggling\n");
done:

	return ret;
}

static int fusb302_pd_send_message(struct udevice *dev,
				   const struct pd_message *msg)
{
	int ret;
	/* SOP tokens */
	u8 buf[40] = {FUSB302_TKN_SYNC1, FUSB302_TKN_SYNC1, FUSB302_TKN_SYNC1,
		      FUSB302_TKN_SYNC2};
	u8 pos = 4;
	int len;

	len = pd_header_cnt_le(msg->header) * 4;
	/* plug 2 for header */
	len += 2;
	if (len > FUSB302_MAX_MSG_LEN) {
		dev_err(dev, "PD message too long %d (incl. header)", len);
		return -EINVAL;
	}
	/* packsym tells the FUSB302 chip that the next X bytes are payload */
	buf[pos++] = FUSB302_TKN_PACKSYM | (len & FUSB302_MAX_MSG_LEN);
	memcpy(&buf[pos], &msg->header, sizeof(msg->header));
	pos += sizeof(msg->header);

	len -= 2;
	memcpy(&buf[pos], msg->payload, len);
	pos += len;

	/* CRC */
	buf[pos++] = FUSB302_TKN_JAMCRC;
	/* EOP */
	buf[pos++] = FUSB302_TKN_EOP;
	/* turn tx off after sending message */
	buf[pos++] = FUSB302_TKN_TXOFF;
	/* start transmission */
	buf[pos++] = FUSB302_TKN_TXON;

	ret = fusb302_i2c_block_write(dev, FUSB_REG_FIFOS, pos, buf);
	if (ret)
		return ret;
	dev_dbg(dev, "Send PD message (header=0x%x len=%d)\n", msg->header, len);

	return ret;
}

static int fusb302_pd_send_hardreset(struct udevice *dev)
{
	return fusb302_i2c_set_bits(dev, FUSB_REG_CONTROL3,
				    FUSB_REG_CONTROL3_SEND_HARDRESET);
}

static const char * const transmit_type_name[] = {
	[TCPC_TX_SOP]			= "SOP",
	[TCPC_TX_SOP_PRIME]		= "SOP'",
	[TCPC_TX_SOP_PRIME_PRIME]	= "SOP''",
	[TCPC_TX_SOP_DEBUG_PRIME]	= "DEBUG'",
	[TCPC_TX_SOP_DEBUG_PRIME_PRIME]	= "DEBUG''",
	[TCPC_TX_HARD_RESET]		= "HARD_RESET",
	[TCPC_TX_CABLE_RESET]		= "CABLE_RESET",
	[TCPC_TX_BIST_MODE_2]		= "BIST_MODE_2",
};

static int fusb302_pd_transmit(struct udevice *dev, enum tcpm_transmit_type type,
			       const struct pd_message *msg, unsigned int negotiated_rev)
{
	int ret;

	switch (type) {
	case TCPC_TX_SOP:
		/* nRetryCount 3 in P2.0 spec, whereas 2 in PD3.0 spec */
		ret = fusb302_enable_tx_auto_retries(dev, negotiated_rev > PD_REV20 ?
						     FUSB_REG_CONTROL3_N_RETRIES_2 :
						     FUSB_REG_CONTROL3_N_RETRIES_3);
		if (ret)
			dev_err(dev, "cannot update retry count: %d\n", ret);

		ret = fusb302_pd_send_message(dev, msg);
		if (ret)
			dev_err(dev, "cannot send PD message: %d\n", ret);
		break;
	case TCPC_TX_HARD_RESET:
		ret = fusb302_pd_send_hardreset(dev);
		if (ret)
			dev_err(dev, "cannot send hardreset: %d\n", ret);
		break;
	default:
		dev_err(dev, "type %s not supported", transmit_type_name[type]);
		ret = -EINVAL;
	}

	return ret;
}

static enum typec_cc_status fusb302_bc_lvl_to_cc(u8 bc_lvl)
{
	if (bc_lvl == FUSB_REG_STATUS0_BC_LVL_1230_MAX)
		return TYPEC_CC_RP_3_0;
	if (bc_lvl == FUSB_REG_STATUS0_BC_LVL_600_1230)
		return TYPEC_CC_RP_1_5;
	if (bc_lvl == FUSB_REG_STATUS0_BC_LVL_200_600)
		return TYPEC_CC_RP_DEF;
	return TYPEC_CC_OPEN;
}

static void fusb302_bc_lvl_handler(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	enum typec_cc_status cc_status;
	u8 status0, bc_lvl;
	int ret;

	if (!chip->intr_bc_lvl) {
		dev_err(dev, "BC_LVL interrupt is turned off, abort\n");
		goto done;
	}
	ret = fusb302_i2c_read(dev, FUSB_REG_STATUS0, &status0);
	if (ret)
		goto done;

	dev_dbg(dev, "BC_LVL handler, status0 = 0x%02x\n", status0);
	/*
	 * A BC_LVL read during PD traffic can read Open mid-message, which
	 * the port manager takes for a detach (tcpm_reset_port(), PD RX and
	 * auto GoodCRC off). Linux re-schedules the read; this driver is
	 * polled, so the next poll reads again.
	 */
	chip->bc_lvl_pending = false;
	if (status0 & FUSB_REG_STATUS0_ACTIVITY) {
		chip->bc_lvl_pending = true;
		goto done;
	}
	bc_lvl = status0 & FUSB_REG_STATUS0_BC_LVL_MASK;
	cc_status = fusb302_bc_lvl_to_cc(bc_lvl);
	if (chip->cc_polarity == TYPEC_POLARITY_CC1) {
		if (chip->cc1 != cc_status) {
			dev_dbg(dev, "cc1: %s -> %s\n",
				typec_cc_status_name[chip->cc1],
				typec_cc_status_name[cc_status]);
			chip->cc1 = cc_status;
			tcpm_cc_change(dev);
		}
	} else {
		if (chip->cc2 != cc_status) {
			dev_dbg(dev, "cc2: %s -> %s\n",
				typec_cc_status_name[chip->cc2],
				typec_cc_status_name[cc_status]);
			chip->cc2 = cc_status;
			tcpm_cc_change(dev);
		}
	}

done:
	return;
}

static int fusb302_enter_low_power_mode(struct udevice *dev,
					bool attached, bool pd_capable)
{
	unsigned int reg;
	int ret;

	ret = fusb302_mask_interrupt(dev);
	if (ret)
		return ret;

	/*
	 * An attached port stays fully powered. PWR_MEDIUM clears PWR3, the
	 * internal oscillator, which unclocks the BMC transmitter -- so the
	 * chip's automatic GoodCRC stops -- and PWR_LOW also drops the receiver
	 * and the measure block. Either way the partner's next PD message goes
	 * unacknowledged from here until an OS driver re-initialises the chip:
	 * it retries, Soft Resets, then Hard Resets, and its VBUS goes through
	 * vSafe0V. On ROC-RK3399-PC's power inlet (Type-C0) that is a power-on
	 * reset a few seconds into GRUB; on Type-C1 a monitor sees VBUS cycle
	 * and PD fail on every boot. Linux's fusb302 never leaves PWR_ALL.
	 * Masking the interrupts is harmless: this stack polls, and GoodCRC is
	 * the chip's own.
	 *
	 * Not before attach either: POWER = 0 also took the bandgap, receiver
	 * and measure block from a port still in TOGGLING or SNK_ATTACH_WAIT,
	 * so a partner arriving afterwards saw a chip that could not detect it
	 * until a consumer woke it. Nothing here needs the few mW back.
	 */
	reg = FUSB_REG_POWER_PWR_ALL;

	return fusb302_set_power_mode(dev, reg);
}

/*
 * The inverse. Low power drops the chip to PWR_MEDIUM, which clears PWR3 --
 * the internal oscillator -- so the BMC transmitter is unclocked, and masks
 * every interrupt, so even a transmit that did happen would report nothing.
 * Restore both before the port manager tries to speak again.
 */
static int fusb302_exit_low_power_mode(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	u8 mask = 0xFF & ~FUSB_REG_MASK_VBUSOK;
	u8 maska = 0xFF;
	int ret;

	ret = fusb302_set_power_mode(dev, FUSB_REG_POWER_PWR_ALL);
	if (ret)
		return ret;

	/*
	 * Rebuild the masks from the port's CURRENT configuration rather than
	 * calling fusb302_init_interrupt(), which writes MASK/MASKA/MASKB
	 * wholesale and would drop the CC interrupts an attached port depends
	 * on -- BC_LVL for a sink, COMP_CHNG for a source, TOGDONE while
	 * toggling. Those are armed by set_cc()/set_toggling(), and nothing
	 * re-runs those after a wake.
	 *
	 * Worse than losing them: this driver keeps shadow copies
	 * (chip->intr_*) that the handler ANDs against the hardware bits, and
	 * a wholesale init updates none of them. The shadows would keep saying
	 * "armed" while the hardware said "off", so a detach after the first
	 * consumer command would simply never be reported and the port would
	 * go on believing it was attached.
	 */
	if (chip->intr_bc_lvl)
		mask &= ~FUSB_REG_MASK_BC_LVL;
	if (chip->intr_comp_chng)
		mask &= ~FUSB_REG_MASK_COMP_CHNG;
	if (chip->intr_togdone)
		maska &= ~FUSB_REG_MASKA_TOGDONE;

	ret = fusb302_i2c_write(dev, FUSB_REG_MASK, mask);
	if (ret)
		return ret;

	ret = fusb302_i2c_write(dev, FUSB_REG_MASKA, maska);
	if (ret)
		return ret;

	ret = fusb302_i2c_write(dev, FUSB_REG_MASKB, 0xFF);
	if (ret)
		return ret;

	ret = fusb302_i2c_clear_bits(dev, FUSB_REG_CONTROL0,
				     FUSB_REG_CONTROL0_INT_MASK);
	if (ret)
		return ret;

	/*
	 * And the PD interrupts to whatever set_pd_rx() last asked for, NOT
	 * unconditionally on. HARD_RESET_START turns them off on purpose, to
	 * silence GCRCSENT and TX_SUCCESS for the duration of the reset
	 * window; a wake landing inside that window would otherwise re-enable
	 * them behind the port manager's back.
	 */
	return fusb302_pd_set_interrupts(dev, chip->intr_pd);
}

static const char * const cc_polarity_name[] = {
	[TYPEC_POLARITY_CC1]	= "Polarity_CC1",
	[TYPEC_POLARITY_CC2]	= "Polarity_CC2",
};

static int fusb302_set_cc_polarity_and_pull(struct udevice *dev,
					    enum typec_cc_polarity cc_polarity,
					    bool pull_up, bool pull_down)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;
	u8 switches0_data = 0x00;
	u8 switches1_mask = FUSB_REG_SWITCHES1_TXCC1_EN |
			    FUSB_REG_SWITCHES1_TXCC2_EN;
	u8 switches1_data = 0x00;

	if (pull_down)
		switches0_data |= FUSB_REG_SWITCHES0_CC1_PD_EN |
				  FUSB_REG_SWITCHES0_CC2_PD_EN;

	if (cc_polarity == TYPEC_POLARITY_CC1) {
		switches0_data |= FUSB_REG_SWITCHES0_MEAS_CC1;
		if (chip->vconn_on)
			switches0_data |= FUSB_REG_SWITCHES0_VCONN_CC2;
		if (pull_up)
			switches0_data |= FUSB_REG_SWITCHES0_CC1_PU_EN;
		switches1_data = FUSB_REG_SWITCHES1_TXCC1_EN;
	} else {
		switches0_data |= FUSB_REG_SWITCHES0_MEAS_CC2;
		if (chip->vconn_on)
			switches0_data |= FUSB_REG_SWITCHES0_VCONN_CC1;
		if (pull_up)
			switches0_data |= FUSB_REG_SWITCHES0_CC2_PU_EN;
		switches1_data = FUSB_REG_SWITCHES1_TXCC2_EN;
	}
	ret = fusb302_i2c_write(dev, FUSB_REG_SWITCHES0, switches0_data);
	if (ret)
		return ret;
	ret = fusb302_i2c_mask_write(dev, FUSB_REG_SWITCHES1,
				     switches1_mask, switches1_data);
	if (ret)
		return ret;
	chip->cc_polarity = cc_polarity;

	return ret;
}

static int fusb302_handle_togdone_snk(struct udevice *dev,
				      u8 togdone_result)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	int ret;
	u8 status0;
	u8 bc_lvl;
	enum typec_cc_polarity cc_polarity;
	enum typec_cc_status cc_status_active, cc1, cc2;

	/* set polarity and pull_up, pull_down */
	cc_polarity = (togdone_result == FUSB_REG_STATUS1A_TOGSS_SNK1) ?
		      TYPEC_POLARITY_CC1 : TYPEC_POLARITY_CC2;
	ret = fusb302_set_cc_polarity_and_pull(dev, cc_polarity, false, true);
	if (ret) {
		dev_err(dev, "cannot set cc polarity %s, ret = %d\n",
			cc_polarity_name[cc_polarity], ret);
		return ret;
	}
	/* fusb302_set_cc_polarity() has set the correct measure block */
	ret = fusb302_i2c_read(dev, FUSB_REG_STATUS0, &status0);
	if (ret < 0)
		return ret;
	bc_lvl = status0 & FUSB_REG_STATUS0_BC_LVL_MASK;
	cc_status_active = fusb302_bc_lvl_to_cc(bc_lvl);
	/* restart toggling if the cc status on the active line is OPEN */
	if (cc_status_active == TYPEC_CC_OPEN) {
		dev_info(dev, "restart toggling as CC_OPEN detected\n");
		ret = fusb302_set_toggling(dev, chip->toggling_mode);
		return ret;
	}
	/* update tcpm with the new cc value */
	cc1 = (cc_polarity == TYPEC_POLARITY_CC1) ?
	      cc_status_active : TYPEC_CC_OPEN;
	cc2 = (cc_polarity == TYPEC_POLARITY_CC2) ?
	      cc_status_active : TYPEC_CC_OPEN;
	if (chip->cc1 != cc1 || chip->cc2 != cc2) {
		chip->cc1 = cc1;
		chip->cc2 = cc2;
		tcpm_cc_change(dev);
	}
	/* turn off toggling */
	ret = fusb302_set_toggling(dev, TOGGLING_MODE_OFF);
	if (ret) {
		dev_err(dev, "cannot set toggling mode off, ret=%d\n", ret);
		return ret;
	}
	/* unmask bc_lvl interrupt */
	ret = fusb302_i2c_clear_bits(dev, FUSB_REG_MASK, FUSB_REG_MASK_BC_LVL);
	if (ret) {
		dev_err(dev, "cannot unmask bc_lcl irq, ret=%d\n", ret);
		return ret;
	}
	chip->intr_bc_lvl = true;
	dev_dbg(dev, "detected cc1=%s, cc2=%s\n",
		typec_cc_status_name[cc1],
		typec_cc_status_name[cc2]);

	return ret;
}

/* On error returns < 0, otherwise a typec_cc_status value */
static int fusb302_get_src_cc_status(struct udevice *dev,
				     enum typec_cc_polarity cc_polarity,
				     enum typec_cc_status *cc)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	u8 ra_mda = ra_mda_value[chip->src_current_status];
	u8 rd_mda = rd_mda_value[chip->src_current_status];
	u8 switches0_data, status0;
	int ret;

	/* Step 1: Set switches so that we measure the right CC pin */
	switches0_data = (cc_polarity == TYPEC_POLARITY_CC1) ?
		FUSB_REG_SWITCHES0_CC1_PU_EN | FUSB_REG_SWITCHES0_MEAS_CC1 :
		FUSB_REG_SWITCHES0_CC2_PU_EN | FUSB_REG_SWITCHES0_MEAS_CC2;
	ret = fusb302_i2c_write(dev, FUSB_REG_SWITCHES0, switches0_data);
	if (ret < 0)
		return ret;

	fusb302_i2c_read(dev, FUSB_REG_SWITCHES0, &status0);
	dev_dbg(dev, "get_src_cc_status switches: 0x%0x", status0);

	/* Step 2: Set compararator volt to differentiate between Open and Rd */
	ret = fusb302_i2c_write(dev, FUSB_REG_MEASURE, rd_mda);
	if (ret)
		return ret;

	udelay(100);
	ret = fusb302_i2c_read(dev, FUSB_REG_STATUS0, &status0);
	if (ret)
		return ret;

	dev_dbg(dev, "get_src_cc_status rd_mda status0: 0x%0x", status0);
	if (status0 & FUSB_REG_STATUS0_COMP) {
		*cc = TYPEC_CC_OPEN;
		return 0;
	}

	/* Step 3: Set compararator input to differentiate between Rd and Ra. */
	ret = fusb302_i2c_write(dev, FUSB_REG_MEASURE, ra_mda);
	if (ret)
		return ret;

	udelay(100);
	ret = fusb302_i2c_read(dev, FUSB_REG_STATUS0, &status0);
	if (ret)
		return ret;

	dev_dbg(dev, "get_src_cc_status ra_mda status0: 0x%0x", status0);
	if (status0 & FUSB_REG_STATUS0_COMP)
		*cc = TYPEC_CC_RD;
	else
		*cc = TYPEC_CC_RA;

	return 0;
}

static int fusb302_handle_togdone_src(struct udevice *dev,
				      u8 togdone_result)
{
	/*
	 * - set polarity (measure cc, vconn, tx)
	 * - set pull_up, pull_down
	 * - set cc1, cc2, and update to tcpm state machine
	 * - set I_COMP interrupt on
	 */
	struct fusb302_chip *chip = dev_get_priv(dev);
	u8 rd_mda = rd_mda_value[chip->src_current_status];
	enum toggling_mode toggling_mode = chip->toggling_mode;
	enum typec_cc_polarity cc_polarity;
	enum typec_cc_status cc1, cc2;
	int ret;

	/*
	 * The toggle-engine will stop in a src state if it sees either Ra or
	 * Rd. Determine the status for both CC pins, starting with the one
	 * where toggling stopped, as that is where the switches point now.
	 */
	if (togdone_result == FUSB_REG_STATUS1A_TOGSS_SRC1)
		ret = fusb302_get_src_cc_status(dev, TYPEC_POLARITY_CC1, &cc1);
	else
		ret = fusb302_get_src_cc_status(dev, TYPEC_POLARITY_CC2, &cc2);
	if (ret)
		return ret;
	/* we must turn off toggling before we can measure the other pin */
	ret = fusb302_set_toggling(dev, TOGGLING_MODE_OFF);
	if (ret) {
		dev_err(dev, "cannot set toggling mode off, ret=%d\n", ret);
		return ret;
	}
	/* get the status of the other pin */
	if (togdone_result == FUSB_REG_STATUS1A_TOGSS_SRC1)
		ret = fusb302_get_src_cc_status(dev, TYPEC_POLARITY_CC2, &cc2);
	else
		ret = fusb302_get_src_cc_status(dev, TYPEC_POLARITY_CC1, &cc1);
	if (ret)
		return ret;

	/* determine polarity based on the status of both pins */
	if (cc1 == TYPEC_CC_RD && (cc2 == TYPEC_CC_OPEN || cc2 == TYPEC_CC_RA)) {
		cc_polarity = TYPEC_POLARITY_CC1;
	} else if (cc2 == TYPEC_CC_RD &&
		    (cc1 == TYPEC_CC_OPEN || cc1 == TYPEC_CC_RA)) {
		cc_polarity = TYPEC_POLARITY_CC2;
	} else {
		dev_err(dev, "unexpected CC status cc1=%s, cc2=%s, restarting toggling\n",
			typec_cc_status_name[cc1],
			typec_cc_status_name[cc2]);
		return fusb302_set_toggling(dev, toggling_mode);
	}
	/* set polarity and pull_up, pull_down */
	ret = fusb302_set_cc_polarity_and_pull(dev, cc_polarity, true, false);
	if (ret < 0) {
		dev_err(dev, "cannot set cc polarity %s, ret=%d\n",
			cc_polarity_name[cc_polarity], ret);
		return ret;
	}
	/* update tcpm with the new cc value */
	if (chip->cc1 != cc1 || chip->cc2 != cc2) {
		chip->cc1 = cc1;
		chip->cc2 = cc2;
		tcpm_cc_change(dev);
	}
	/* set MDAC to Rd threshold, and unmask I_COMP for unplug detection */
	ret = fusb302_i2c_write(dev, FUSB_REG_MEASURE, rd_mda);
	if (ret)
		return ret;
	/* unmask comp_chng interrupt */
	ret = fusb302_i2c_clear_bits(dev, FUSB_REG_MASK,
				     FUSB_REG_MASK_COMP_CHNG);
	if (ret) {
		dev_err(dev, "cannot unmask comp_chng irq, ret=%d\n", ret);
		return ret;
	}
	chip->intr_comp_chng = true;
	dev_dbg(dev, "detected cc1=%s, cc2=%s\n",
		typec_cc_status_name[cc1],
		typec_cc_status_name[cc2]);

	return ret;
}

static int fusb302_handle_togdone(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	u8 togdone_result, status1a;
	int ret;

	ret = fusb302_i2c_read(dev, FUSB_REG_STATUS1A, &status1a);
	if (ret < 0)
		return ret;
	togdone_result = (status1a >> FUSB_REG_STATUS1A_TOGSS_POS) &
			 FUSB_REG_STATUS1A_TOGSS_MASK;
	switch (togdone_result) {
	case FUSB_REG_STATUS1A_TOGSS_SNK1:
	case FUSB_REG_STATUS1A_TOGSS_SNK2:
		return fusb302_handle_togdone_snk(dev, togdone_result);
	case FUSB_REG_STATUS1A_TOGSS_SRC1:
	case FUSB_REG_STATUS1A_TOGSS_SRC2:
		return fusb302_handle_togdone_src(dev, togdone_result);
	case FUSB_REG_STATUS1A_TOGSS_AA:
		/* doesn't support */
		dev_err(dev, "AudioAccessory not supported\n");
		fusb302_set_toggling(dev, chip->toggling_mode);
		break;
	default:
		dev_err(dev, "TOGDONE with an invalid state: %d\n",
			togdone_result);
		fusb302_set_toggling(dev, chip->toggling_mode);
		break;
	}
	return ret;
}

static int fusb302_pd_reset(struct udevice *dev)
{
	return fusb302_i2c_set_bits(dev, FUSB_REG_RESET,
				    FUSB_REG_RESET_PD_RESET);
}

/**
 * fusb302_rx_token_class() - name the packet class in an RX FIFO token
 *
 * Bits [7:5] of the token, per the datasheet table quoted in fusb302_reg.h.
 * A byte that is not a token at all still lands in one of these classes --
 * naming the class is a description, never on its own a proof that a real
 * frame of that class arrived.
 */
static const char *fusb302_rx_token_class(u8 token)
{
	switch (token & FUSB302_RX_TOKEN_CLASS_MASK) {
	case FUSB302_RX_TOKEN_SOP:
		return "SOP";
	case FUSB302_RX_TOKEN_SOP1:
		return "SOP'";
	case FUSB302_RX_TOKEN_SOP2:
		return "SOP''";
	default:
		return "no token class";
	}
}

static int fusb302_pd_read_message(struct udevice *dev,
				   struct pd_message *msg)
{
	int len, ret;
	u8 crc[4];
	u8 token;

	/* first SOP token */
	ret = fusb302_i2c_read(dev, FUSB_REG_FIFOS, &token);
	if (ret)
		return ret;
	ret = fusb302_i2c_block_read(dev, FUSB_REG_FIFOS, 2,
				     (u8 *)&msg->header);
	if (ret)
		return ret;
	len = pd_header_cnt_le(msg->header) * 4;
	/* add 4 to length to include the CRC */
	if (len > PD_MAX_PAYLOAD * 4) {
		dev_err(dev, "PD message too long %d\n", len);
		return -EINVAL;
	}
	if (len > 0) {
		ret = fusb302_i2c_block_read(dev, FUSB_REG_FIFOS, len,
					     (u8 *)msg->payload);
		if (ret)
			return ret;
	}
	/* another 4 bytes to read CRC out */
	ret = fusb302_i2c_block_read(dev, FUSB_REG_FIFOS, 4, crc);
	if (ret)
		return ret;
	dev_dbg(dev, "Received PD message (header=0x%x len=%d)\n", msg->header, len);

	/*
	 * DROP the frame. The RX token is the only thing that says where a frame
	 * starts, so a token that is not SOP means the bytes just read are not a
	 * header and a payload -- and a frame whose framing has been disproved
	 * must not be acted on.
	 *
	 * On a FUSB302 a received SOP frame is prefixed 0b111xxxxx. The other
	 * two classes are REAL tokens -- 0b110xxxxx is SOP' and 0b101xxxxx is
	 * SOP'' -- so the token is now CLASSIFIED rather than declared a
	 * desync, which the earlier version of this message did assert. The
	 * observed 0xc7 lands exactly on the SOP' class, and a mid-payload byte
	 * hitting a neighbouring class boundary is a coincidence worth ruling
	 * out before believing the desync story.
	 *
	 * It is ruled out HERE, not asserted: SOP'/SOP'' are delivered only
	 * with CONTROL1.ENSOP1/ENSOP2 set, this driver never sets either, and
	 * fusb302_init() opens with a full software reset (FUSB_REG_RESET_SW_RESET)
	 * that returns CONTROL1 to its 0x00 default -- so no SOP' frame can be
	 * queued for us at all. CONTROL1 is READ BACK into the message rather
	 * than trusted, because that reasoning is the only thing standing
	 * between "mid-frame byte" and "a cable-plug frame we asked for".
	 *
	 * With SOP'/SOP'' excluded, a non-SOP token is a partial frame or a
	 * desync. A drain that terminates on STATUS1.RX_EMPTY cannot tell
	 * "another complete message" from "a message still arriving", and a pop
	 * that lands mid-frame desynchronises the FIFO: every subsequent
	 * "header" is payload bytes, so its data-role bit is arbitrary and
	 * ~half of them trip tcpm_pd_rx_handler()'s mismatch check.
	 *
	 * This was previously a diagnostic that printed and then fell through to
	 * tcpm_pd_receive() anyway, on the grounds that the token encoding is
	 * not in fusb302_reg.h. That reasoning does not survive what the frame
	 * is USED for: a bogus data-role bit does not produce a bogus reading,
	 * it initiates a full PD ERROR RECOVERY -- the port detaches, re-attaches
	 * and renegotiates -- and when that lands inside an alt-mode VDM
	 * sequence it destroys a DisplayPort entry. Measured on this bench: a
	 * boot carrying "RX token 0xc7 is not SOP (header=0xe0be len=24)" went on
	 * to fail DisplayPort Status Update at -110. Not knowing the exact
	 * encoding of a token is a reason to distrust the frame, not a reason to
	 * trust it.
	 *
	 * Dropping is still the conservative half of the trade, but NOT for the
	 * reason first written here: "a PD-layer retransmission, which the
	 * protocol is built for" is not what this layer does. tcpm_vdm_xfer()
	 * retries only on CMDT_RSP_BUSY; tcpm_vdm_wait() returning -ETIMEDOUT
	 * goes straight out, and a dropped GoodCRC expires into
	 * `TCPM: PD transmit data failed` -- the same line the boot-time DP
	 * failure signature is made of. That claim is WITHDRAWN.
	 *
	 * What survives is the comparison: dropping costs one failed exchange,
	 * which the port survives, while accepting a frame whose framing has
	 * been disproved costs a spurious PD error recovery mid-alt-mode, which
	 * it does not. Still logged, because both a desync and a lost message
	 * are real faults and a silent drop would hide either.
	 */
	if ((token & FUSB302_RX_TOKEN_CLASS_MASK) != FUSB302_RX_TOKEN_SOP) {
		u8 control1;

		if (fusb302_i2c_read(dev, FUSB_REG_CONTROL1, &control1))
			control1 = 0xff;

		dev_err(dev, "RX token %#04x is not SOP (class %s, header=%#06x len=%d, CONTROL1=%#04x) -- frame DROPPED\n",
			token, fusb302_rx_token_class(token), msg->header, len,
			control1);
		return -EPROTO;
	}

	/*
	 * Check if we've read off a GoodCRC message. If so then indicate to
	 * TCPM that the previous transmission has completed. Otherwise we pass
	 * the received message over to TCPM for processing.
	 *
	 * We make this check here instead of basing the reporting decision on
	 * the IRQ event type, as it's possible for the chip to report the
	 * TX_SUCCESS and GCRCSENT events out of order on occasion, so we need
	 * to check the message type to ensure correct reporting to TCPM.
	 */
	if (!len && (pd_header_type_le(msg->header) == PD_CTRL_GOOD_CRC))
		tcpm_pd_transmit_complete(dev, TCPC_TX_SUCCESS);
	else
		tcpm_pd_receive(dev, msg);

	return ret;
}

/*
 * Empty the RX FIFO, rather than popping one message per interrupt bit.
 *
 * The FIFO is a QUEUE and the interrupts are EDGES, so the two do not have a
 * fixed ratio. INTERRUPTA/B are read-to-clear at the top of the handler, and
 * the only two places that ever popped a message were the TX_SUCCESS and
 * GCRCSENT branches -- so one pass drained at most two, and a third message
 * queued in the same window was stranded with no interrupt left to retrieve
 * it. CRC_CHK and ACTIVITY are permanently masked and have no handler branch,
 * so nothing else would ever come back for it.
 *
 * That is the whole of the intermittent DisplayPort failure. Entering alt
 * mode makes the monitor start sending unsolicited Attention VDMs; when one
 * lands between our GoodCRC and our response the FIFO holds three, two are
 * popped, and OUR ANSWER is the one left behind. tcpm_vdm_wait() then polls
 * for 300 ms against interrupt registers reading all-zero while the reply
 * sits unread inside the chip, and reports -ETIMEDOUT. The queue drains by
 * exactly one only when the partner next speaks, which is why a boot that
 * fails once keeps failing: the pump runs permanently one message behind.
 *
 * Measured over three instrumented boots: every failing attempt, and only a
 * failing attempt, begins with the TX_SUCCESS branch popping something that
 * is not a GoodCRC. Zero anomalies in the boot that passed.
 *
 * STATUS1.RX_EMPTY is the level status that answers "is a message waiting",
 * it has been defined in fusb302_reg.h all along, and nothing read it.
 * Reading it is also what makes the GoodCRC-vs-message decision inside
 * fusb302_pd_read_message() sound: that check is already order-independent,
 * but it can only be right about messages that are actually read out.
 */
static int fusb302_pd_drain_rx(struct udevice *dev)
{
	struct pd_message msg;
	u8 status1;
	int i, ret;

	/*
	 * Bounded because a wedged chip that never asserts RX_EMPTY would
	 * otherwise spin forever. The FIFO holds far fewer than this; the cap
	 * is a backstop, not a policy.
	 */
	for (i = 0; i < 16; i++) {
		ret = fusb302_i2c_read(dev, FUSB_REG_STATUS1, &status1);
		if (ret)
			return ret;

		if (status1 & FUSB_REG_STATUS1_RX_EMPTY)
			return 0;

		ret = fusb302_pd_read_message(dev, &msg);

		/*
		 * -EPROTO is "that frame was not SOP-framed and was dropped",
		 * not "the chip is broken" -- so KEEP DRAINING. Returning here
		 * would leave the rest of the FIFO unread and recreate, exactly,
		 * the one-message-behind pathology this drain was written to
		 * cure: the reply we are waiting for stays inside the chip while
		 * tcpm_vdm_wait() polls interrupt registers that read all-zero.
		 *
		 * Continuing EMPTIES the FIFO. It does NOT resynchronise it --
		 * an earlier version of this comment claimed it did, and that
		 * claim is WITHDRAWN. Nothing here re-establishes framing:
		 * every following iteration reads `1 + 2 + len + 4` bytes with
		 * @len taken from a header this function has just proved bogus,
		 * so the pops walk the FIFO in the wrong stride until it
		 * happens to run dry.
		 *
		 * FUSB_REG_CONTROL1_RX_FLUSH is the primitive that would
		 * actually reset framing, and it is deliberately NOT used here:
		 * it discards the whole FIFO, including the reply the caller is
		 * waiting for, which recreates by hand the one-message-behind
		 * pathology this drain exists to cure. Draining to RX_EMPTY
		 * reaches the same empty FIFO while still delivering every
		 * frame that IS correctly framed. Choosing flush over drain is
		 * a behavioural change and needs a bench, which this has not
		 * had. Only a real I2C failure is worth abandoning the drain
		 * for.
		 */
		if (ret == -EPROTO)
			continue;
		if (ret)
			return ret;
	}

	dev_warn(dev, "RX FIFO still not empty after %d messages\n", i);

	return 0;
}

static void fusb302_interrupt_handle(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);
	u8 interrupt;
	u8 interrupta;
	u8 interruptb;
	u8 status0;
	bool vbus_present;
	bool comp_result;
	bool intr_togdone;
	bool intr_bc_lvl;
	bool intr_comp_chng;
	int ret;

	/* grab a snapshot of intr flags */
	intr_togdone = chip->intr_togdone;
	intr_bc_lvl = chip->intr_bc_lvl;
	intr_comp_chng = chip->intr_comp_chng;

	ret = fusb302_i2c_read(dev, FUSB_REG_INTERRUPT, &interrupt);
	if (ret)
		return;
	ret = fusb302_i2c_read(dev, FUSB_REG_INTERRUPTA, &interrupta);
	if (ret)
		return;
	ret = fusb302_i2c_read(dev, FUSB_REG_INTERRUPTB, &interruptb);
	if (ret)
		return;
	ret = fusb302_i2c_read(dev, FUSB_REG_STATUS0, &status0);
	if (ret)
		return;

	/*
	 * Since we are polling the IRQs, avoid printing messages when there
	 * no interrupts at all to avoid spamming the log.
	 */
	if (interrupt != 0 || interrupta != 0 || interruptb != 0)
		dev_dbg(dev, "IRQ: 0x%02x, a: 0x%02x, b: 0x%02x, status0: 0x%02x\n",
			interrupt, interrupta, interruptb, status0);
	/* TEST/diagnostic (B175): `setenv fusb302_trace 1` prints the same */
	if ((interrupt | interrupta | interruptb) &&
	    env_get_yesno("fusb302_trace") == 1)
		printf("%s: IRQ %02x a %02x b %02x status0 %02x\n", dev->name,
		       interrupt, interrupta, interruptb, status0);

	if (interrupt & FUSB_REG_INTERRUPT_VBUSOK) {
		vbus_present = !!(status0 & FUSB_REG_STATUS0_VBUSOK);
		dev_dbg(dev, "IRQ: VBUS_OK, vbus=%s\n",
			vbus_present ? "On" : "Off");
		if (vbus_present != chip->vbus_present) {
			chip->vbus_present = vbus_present;
			tcpm_vbus_change(dev);
		}
	}

	if ((interrupta & FUSB_REG_INTERRUPTA_TOGDONE) && intr_togdone) {
		dev_dbg(dev, "IRQ: TOGDONE\n");
		ret = fusb302_handle_togdone(dev);
		if (ret) {
			dev_err(dev, "handle togdone error: %d\n", ret);
			return;
		}
	}

	if ((interrupt & FUSB_REG_INTERRUPT_BC_LVL) && intr_bc_lvl) {
		dev_dbg(dev, "IRQ: BC_LVL, handler pending\n");
		fusb302_bc_lvl_handler(dev);
	}

	if ((interrupt & FUSB_REG_INTERRUPT_COMP_CHNG) && intr_comp_chng) {
		comp_result = !!(status0 & FUSB_REG_STATUS0_COMP);
		dev_dbg(dev, "IRQ: COMP_CHNG, comp=%s\n",
			comp_result ? "true" : "false");
		/*
		 * cc level > Rd threshold: a detach, if it lasts. One edge is not
		 * enough -- our own BMC transmit, or the partner's, moves the
		 * line, and a source that detaches on it switches VBUS off at
		 * the partner (measured on ROC-RK3399-PC Type-C1: unattached 11
		 * ms after each Source_Capabilities). Confirm after tPDDebounce,
		 * on the poll path.
		 */
		if (comp_result && !chip->comp_pending) {
			chip->comp_pending = true;
			chip->comp_since = get_timer(0);
		}
	}

	if (interrupt & FUSB_REG_INTERRUPT_COLLISION) {
		dev_dbg(dev, "IRQ: PD collision\n");
		tcpm_pd_transmit_complete(dev, TCPC_TX_FAILED);
	}

	if (interrupta & FUSB_REG_INTERRUPTA_RETRYFAIL) {
		dev_dbg(dev, "IRQ: PD retry failed\n");
		tcpm_pd_transmit_complete(dev, TCPC_TX_FAILED);
	}

	if (interrupta & FUSB_REG_INTERRUPTA_HARDSENT) {
		dev_dbg(dev, "IRQ: PD hardreset sent\n");
		ret = fusb302_pd_reset(dev);
		if (ret) {
			dev_err(dev, "cannot PD reset, ret=%d\n", ret);
			return;
		}
		tcpm_pd_transmit_complete(dev, TCPC_TX_SUCCESS);
	}

	if (interrupta & FUSB_REG_INTERRUPTA_TX_SUCCESS) {
		dev_dbg(dev, "IRQ: PD tx success\n");
		ret = fusb302_pd_drain_rx(dev);
		if (ret) {
			dev_err(dev, "cannot read in PD message, ret=%d\n", ret);
			return;
		}
	}

	if (interrupta & FUSB_REG_INTERRUPTA_HARDRESET) {
		dev_dbg(dev, "IRQ: PD received hardreset\n");
		ret = fusb302_pd_reset(dev);
		if (ret) {
			dev_err(dev, "cannot PD reset, ret=%d\n", ret);
			return;
		}
		tcpm_pd_hard_reset(dev);
	}

	if (interruptb & FUSB_REG_INTERRUPTB_GCRCSENT) {
		dev_dbg(dev, "IRQ: PD sent good CRC\n");
		ret = fusb302_pd_drain_rx(dev);
		if (ret) {
			dev_err(dev, "cannot read in PD message, ret=%d\n", ret);
			return;
		}
	}
}

static void fusb302_poll_event(struct udevice *dev)
{
	struct fusb302_chip *chip = dev_get_priv(dev);

	if (chip->bc_lvl_pending)
		fusb302_bc_lvl_handler(dev);

	/* A source detach seen on COMP, confirmed after tPDDebounce */
	if (chip->comp_pending && get_timer(chip->comp_since) >= 15) {
		u8 status0;

		if (!fusb302_i2c_read(dev, FUSB_REG_STATUS0, &status0) &&
		    !(status0 & FUSB_REG_STATUS0_ACTIVITY)) {
			chip->comp_pending = false;
			if (status0 & FUSB_REG_STATUS0_COMP) {
				chip->cc1 = TYPEC_CC_OPEN;
				chip->cc2 = TYPEC_CC_OPEN;
				tcpm_cc_change(dev);
			}
		}
	}

	fusb302_interrupt_handle(dev);
}

static int fusb302_get_connector_node(struct udevice *dev, ofnode *connector_node)
{
	*connector_node = dev_read_subnode(dev, "connector");
	if (!ofnode_valid(*connector_node)) {
		dev_err(dev, "'connector' node is not found\n");
		return -ENODEV;
	}

	return 0;
}

static struct dm_tcpm_ops fusb302_ops = {
	.get_connector_node = fusb302_get_connector_node,
	.init = fusb302_init,
	.get_vbus = fusb302_get_vbus,
	.set_cc = fusb302_set_cc,
	.get_cc = fusb302_get_cc,
	.set_vconn = fusb302_set_vconn,
	.set_vbus = fusb302_set_vbus,
	.set_pd_rx = fusb302_set_pd_rx,
	.set_roles = fusb302_set_roles,
	.start_toggling = fusb302_start_toggling,
	.pd_transmit = fusb302_pd_transmit,
	.poll_event = fusb302_poll_event,
	.enter_low_power_mode = fusb302_enter_low_power_mode,
	.exit_low_power_mode = fusb302_exit_low_power_mode,
	.event_pending = fusb302_event_pending,
	.get_prior_roles = fusb302_get_prior_roles,
};

static const struct udevice_id fusb302_ids[] = {
	{ .compatible = "fcs,fusb302" },
	{ }
};

U_BOOT_DRIVER(fusb302) = {
	.name = "fusb302",
	.id = UCLASS_TCPM,
	.of_match = fusb302_ids,
	.ops = &fusb302_ops,
	.priv_auto = sizeof(struct fusb302_chip),
};
