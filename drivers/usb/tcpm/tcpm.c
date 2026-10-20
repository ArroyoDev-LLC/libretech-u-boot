// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2015-2017 Google, Inc
 *
 * USB Power Delivery protocol stack.
 */

#include <asm/gpio.h>
#include <asm/io.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/devres.h>
#include <i2c.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <time.h>
#include <usb/tcpm.h>
#include <usb/typec_dp.h>
#include "tcpm-internal.h"

const char * const tcpm_states[] = {
	FOREACH_TCPM_STATE(GENERATE_TCPM_STRING)
};

const char * const typec_pd_rev_name[] = {
	[PD_REV10]		= "rev1",
	[PD_REV20]		= "rev2",
	[PD_REV30]		= "rev3",
};

const char * const typec_role_name[] = {
	[TYPEC_SINK]		= "sink",
	[TYPEC_SOURCE]		= "source",
};

const char * const typec_data_role_name[] = {
	[TYPEC_DEVICE]		= "device",
	[TYPEC_HOST]		= "host",
};

const char * const typec_orientation_name[] = {
	[TYPEC_ORIENTATION_NONE]	= "none",
	[TYPEC_ORIENTATION_NORMAL]	= "normal",
	[TYPEC_ORIENTATION_REVERSE]	= "reverse",
};

const char * const typec_cc_status_name[] = {
	[TYPEC_CC_OPEN]		= "open",
	[TYPEC_CC_RA]		= "ra",
	[TYPEC_CC_RD]		= "rd",
	[TYPEC_CC_RP_DEF]	= "rp-def",
	[TYPEC_CC_RP_1_5]	= "rp-1.5",
	[TYPEC_CC_RP_3_0]	= "rp-3.0",
};

static inline bool tcpm_cc_is_sink(enum typec_cc_status cc)
{
	return cc == TYPEC_CC_RP_DEF ||
	       cc == TYPEC_CC_RP_1_5 ||
	       cc == TYPEC_CC_RP_3_0;
}

static inline bool tcpm_port_is_sink(struct tcpm_port *port)
{
	bool cc1_is_snk = tcpm_cc_is_sink(port->cc1);
	bool cc2_is_snk = tcpm_cc_is_sink(port->cc2);

	return (cc1_is_snk && !cc2_is_snk) ||
	       (cc2_is_snk && !cc1_is_snk);
}

static inline bool tcpm_cc_is_source(enum typec_cc_status cc)
{
	return cc == TYPEC_CC_RD;
}

static inline bool tcpm_port_is_source(struct tcpm_port *port)
{
	bool cc1_is_src = tcpm_cc_is_source(port->cc1);
	bool cc2_is_src = tcpm_cc_is_source(port->cc2);

	return (cc1_is_src && !cc2_is_src) ||
	       (cc2_is_src && !cc1_is_src);
}

static inline bool tcpm_try_src(struct tcpm_port *port)
{
	return port->try_role == TYPEC_SOURCE &&
	       port->port_type == TYPEC_PORT_DRP;
}

/*
 * Re-arm the probe-time pump's quiet-time budget. Every event the port sees
 * (a CC or VBUS change, a message in or out, a detach) calls this, so the
 * pump gives up TCPM_POLL_EVENT_TIME_OUT ms after the LAST event, not after
 * a fixed count of polls: it used to count 2000 polls, which at one ~0.5 ms
 * poll plus a 500 us sleep each was the same 2 s of silence -- but a count
 * only measures time while something polls continuously, and the pump no
 * longer does (see tcpm_post_probe()).
 */
static inline void tcpm_reset_event_cnt(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	port->poll_event_deadline = timer_get_us() +
				    TCPM_POLL_EVENT_TIME_OUT * 1000ULL;
}

static enum tcpm_state tcpm_default_state(struct tcpm_port *port)
{
	if (port->port_type == TYPEC_PORT_DRP) {
		if (port->try_role == TYPEC_SINK)
			return SNK_UNATTACHED;
		else if (port->try_role == TYPEC_SOURCE)
			return SRC_UNATTACHED;
	} else if (port->port_type == TYPEC_PORT_SNK) {
		return SNK_UNATTACHED;
	}
	return SRC_UNATTACHED;
}

static bool tcpm_port_is_disconnected(struct tcpm_port *port)
{
	return (!port->attached && port->cc1 == TYPEC_CC_OPEN &&
		port->cc2 == TYPEC_CC_OPEN) ||
	       (port->attached && ((port->polarity == TYPEC_POLARITY_CC1 &&
				    port->cc1 == TYPEC_CC_OPEN) ||
				   (port->polarity == TYPEC_POLARITY_CC2 &&
				    port->cc2 == TYPEC_CC_OPEN)));
}

static void tcpm_set_cc(struct udevice *dev, enum typec_cc_status cc)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	dev_dbg(dev, "TCPM: set cc = %d\n", cc);
	port->cc_req = cc;
	drvops->set_cc(dev, cc);
}

/*
 * Determine RP value to set based on maximum current supported
 * by a port if configured as source.
 * Returns CC value to report to link partner.
 */
static enum typec_cc_status tcpm_rp_cc(struct tcpm_port *port)
{
	const u32 *src_pdo = port->src_pdo;
	int nr_pdo = port->nr_src_pdo;
	int i;

	/*
	 * Search for first entry with matching voltage.
	 * It should report the maximum supported current.
	 */
	for (i = 0; i < nr_pdo; i++) {
		const u32 pdo = src_pdo[i];

		if (pdo_type(pdo) == PDO_TYPE_FIXED &&
		    pdo_fixed_voltage(pdo) == 5000) {
			unsigned int curr = pdo_max_current(pdo);

			if (curr >= 3000)
				return TYPEC_CC_RP_3_0;
			else if (curr >= 1500)
				return TYPEC_CC_RP_1_5;
			return TYPEC_CC_RP_DEF;
		}
	}

	return TYPEC_CC_RP_DEF;
}

static void tcpm_check_and_run_delayed_work(struct udevice *dev);
static void tcpm_ams_start(struct udevice *dev);
static void tcpm_ams_finish(struct udevice *dev);

static bool tcpm_transmit_helper(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	drvops->poll_event(dev);
	udelay(500);
	tcpm_check_and_run_delayed_work(dev);
	return port->tx_complete;
}

/**
 * tcpm_wake() - undo the low-power mode the port manager asked for
 *
 * tcpm_poll_event() ends by calling enter_low_power_mode(), which on the
 * FUSB302 drops the internal oscillator and masks every interrupt. That is
 * correct for the lifecycle it was written for -- run the state machine at
 * probe, then idle -- but this stack now exposes the port to consumers, and
 * anything done after the port went idle reached a chip that could not
 * respond. The symptom was not an error: transmits timed out with NO status
 * at all, since an unclocked transmitter reports neither success nor
 * retry-fail nor collision.
 *
 * The port manager decided to sleep it, so the port manager owes the wake.
 * Here rather than in the driver's transmit, because "the chip must be awake"
 * is a fact about the port, and a TCPC with no low-power mode should not have
 * to know about it.
 *
 * Idempotent, and needed by anything that touches the wire OR waits on the
 * chip to observe something -- polling a sleeping TCPC observes nothing, for
 * as long as you care to poll.
 */
static int tcpm_wake(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	if (!port->low_power)
		return 0;

	if (!drvops->exit_low_power_mode) {
		/*
		 * Clear it anyway. There is nothing to call and nothing will
		 * ever change that, so leaving the flag set only re-prints
		 * this on every transmit for the life of the port.
		 */
		port->low_power = false;
		dev_warn(dev, "TCPM: TCPC sleeps but cannot be woken\n");
		return 0;
	}

	ret = drvops->exit_low_power_mode(dev);
	if (ret < 0) {
		dev_err(dev, "TCPM: failed to leave low power: %d\n", ret);
		return ret;
	}

	port->low_power = false;

	return 0;
}

static int tcpm_pd_transmit(struct udevice *dev,
			    enum tcpm_transmit_type type,
			    const struct pd_message *msg)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	u32 timeout_us = PD_T_TCPC_TX_TIMEOUT * 1000;
	bool tx_complete;
	int ret;

	if (msg)
		dev_dbg(dev, "TCPM: PD TX, header: %#x\n",
			le16_to_cpu(msg->header));
	else
		dev_dbg(dev, "TCPM: PD TX, type: %#x\n", type);

	ret = tcpm_wake(dev);
	if (ret < 0)
		return ret;

	port->tx_complete = false;
	ret = drvops->pd_transmit(dev, type, msg, port->negotiated_rev);
	if (ret < 0)
		return ret;

	/*
	 * At this point we basically need to block until the TCPM controller
	 * returns successful transmission. Since this is usually done using
	 * the generic interrupt status bits, we poll for any events. That
	 * will clear the interrupt status, so we also need to process any
	 * of the incoming events. This means we will do more processing and
	 * thus let's give everything a bit more time.
	 */
	timeout_us *= 5;
	ret = read_poll_timeout(tcpm_transmit_helper, tx_complete,
				tx_complete, false, timeout_us, dev);
	if (ret < 0) {
		dev_err(dev, "TCPM: PD transmit data failed: %d\n", ret);
		return ret;
	}

	switch (port->tx_status) {
	case TCPC_TX_SUCCESS:
		port->message_id = (port->message_id + 1) & PD_HEADER_ID_MASK;
		break;
	case TCPC_TX_DISCARDED:
		ret = -EAGAIN;
		break;
	case TCPC_TX_FAILED:
	default:
		ret = -EIO;
		break;
	}

	return ret;
}

void tcpm_pd_transmit_complete(struct udevice *dev,
			       enum tcpm_transmit_status status)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	dev_dbg(dev, "TCPM: PD TX complete, status: %u\n", status);
	tcpm_reset_event_cnt(dev);
	port->tx_status = status;
	port->tx_complete = true;
}

static int tcpm_set_polarity(struct udevice *dev,
			     enum typec_cc_polarity polarity)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	int ret;

	dev_dbg(dev, "TCPM: set polarity = %d\n", polarity);

	if (drvops->set_polarity) {
		ret = drvops->set_polarity(dev, polarity);
		if (ret < 0)
			return ret;
	}

	port->polarity = polarity;

	return 0;
}

static int tcpm_set_vconn(struct udevice *dev, bool enable)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	dev_dbg(dev, "TCPM: set vconn = %d\n", enable);

	ret = drvops->set_vconn(dev, enable);
	if (!ret)
		port->vconn_role = enable ? TYPEC_SOURCE : TYPEC_SINK;

	return ret;
}

static inline u32 tcpm_get_current_limit(struct tcpm_port *port)
{
	switch (port->polarity ? port->cc2 : port->cc1) {
	case TYPEC_CC_RP_1_5:
		return 1500;
	case TYPEC_CC_RP_3_0:
		return 3000;
	case TYPEC_CC_RP_DEF:
	default:
		return 0;
	}
}

static int tcpm_set_current_limit(struct udevice *dev, u32 max_ma, u32 mv)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret = -EOPNOTSUPP;

	dev_info(dev, "TCPM: set voltage limit = %u mV\n", mv);
	dev_info(dev, "TCPM: set current limit = %u mA\n", max_ma);

	port->supply_voltage = mv;
	port->current_limit = max_ma;

	return ret;
}

static int tcpm_set_attached_state(struct udevice *dev, bool attached)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	return drvops->set_roles(dev, attached, port->pwr_role,
				 port->data_role);
}

static int tcpm_set_roles(struct udevice *dev, bool attached,
			  enum typec_role role, enum typec_data_role data)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	ret = drvops->set_roles(dev, attached, role, data);
	if (ret < 0)
		return ret;

	port->pwr_role = role;
	port->data_role = data;

	return 0;
}

static int tcpm_pd_send_source_caps(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	struct pd_message msg;
	int i;

	memset(&msg, 0, sizeof(msg));

	if (!port->nr_src_pdo) {
		/* No source capabilities defined, sink only */
		msg.header = PD_HEADER_LE(PD_CTRL_REJECT,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id, 0);
	} else {
		msg.header = PD_HEADER_LE(PD_DATA_SOURCE_CAP,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id,
					  port->nr_src_pdo);
	}

	for (i = 0; i < port->nr_src_pdo; i++)
		msg.payload[i] = cpu_to_le32(port->src_pdo[i]);

	return tcpm_pd_transmit(dev, TCPC_TX_SOP, &msg);
}

static int tcpm_pd_send_sink_caps(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	struct pd_message msg;
	unsigned int i;

	memset(&msg, 0, sizeof(msg));

	if (!port->nr_snk_pdo) {
		/* No sink capabilities defined, source only */
		msg.header = PD_HEADER_LE(PD_CTRL_REJECT,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id, 0);
	} else {
		msg.header = PD_HEADER_LE(PD_DATA_SINK_CAP,
					  port->pwr_role,
					  port->data_role,
					  port->negotiated_rev,
					  port->message_id,
					  port->nr_snk_pdo);
	}

	for (i = 0; i < port->nr_snk_pdo; i++)
		msg.payload[i] = cpu_to_le32(port->snk_pdo[i]);

	return tcpm_pd_transmit(dev, TCPC_TX_SOP, &msg);
}

static void tcpm_state_machine(struct udevice *dev);

static inline void tcpm_timer_uninit(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	port->delay_target = 0;
}

static void tcpm_timer_init(struct udevice *dev, uint32_t ms)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	unsigned long time_us = ms * 1000;

	port->delay_target = timer_get_us() + time_us;
}

static void tcpm_check_and_run_delayed_work(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	/* no delayed state changes scheduled */
	if (port->delay_target == 0)
		return;

	/* it's not yet time */
	if (timer_get_us() < port->delay_target)
		return;

	tcpm_timer_uninit(dev);
	tcpm_state_machine(dev);
}

static void mod_tcpm_delayed_work(struct udevice *dev, unsigned int delay_ms)
{
	if (delay_ms) {
		tcpm_timer_init(dev, delay_ms);
	} else {
		tcpm_timer_uninit(dev);
		tcpm_state_machine(dev);
	}
}

static void tcpm_set_state(struct udevice *dev, enum tcpm_state state,
			   unsigned int delay_ms)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (delay_ms) {
		dev_dbg(dev, "TCPM: pending state change %s -> %s @ %u ms [%s]\n",
			tcpm_states[port->state], tcpm_states[state], delay_ms,
			typec_pd_rev_name[port->negotiated_rev]);
		port->delayed_state = state;
		mod_tcpm_delayed_work(dev, delay_ms);
		port->delay_ms = delay_ms;
	} else {
		dev_dbg(dev, "TCPM: state change %s -> %s\n",
			tcpm_states[port->state], tcpm_states[state]);
		port->delayed_state = INVALID_STATE;
		port->prev_state = port->state;
		port->state = state;
		/*
		 * Don't re-queue the state machine work item if we're currently
		 * in the state machine and we're immediately changing states.
		 * tcpm_state_machine_work() will continue running the state
		 * machine.
		 */
		if (!port->state_machine_running)
			mod_tcpm_delayed_work(dev, 0);
	}
}

static void tcpm_set_state_cond(struct udevice *dev, enum tcpm_state state,
				unsigned int delay_ms)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (port->enter_state == port->state)
		tcpm_set_state(dev, state, delay_ms);
	else
		dev_dbg(dev, "TCPM: skipped %sstate change %s -> %s [%u ms], context state %s [%s]\n",
			delay_ms ? "delayed " : "",
			tcpm_states[port->state], tcpm_states[state],
			delay_ms, tcpm_states[port->enter_state],
			typec_pd_rev_name[port->negotiated_rev]);
}

static void tcpm_queue_message(struct udevice *dev,
			       enum pd_msg_request message)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	port->queued_message = message;
	mod_tcpm_delayed_work(dev, 0);
}

enum pdo_err {
	PDO_NO_ERR,
	PDO_ERR_NO_VSAFE5V,
	PDO_ERR_VSAFE5V_NOT_FIRST,
	PDO_ERR_PDO_TYPE_NOT_IN_ORDER,
	PDO_ERR_FIXED_NOT_SORTED,
	PDO_ERR_VARIABLE_BATT_NOT_SORTED,
	PDO_ERR_DUPE_PDO,
	PDO_ERR_PPS_APDO_NOT_SORTED,
	PDO_ERR_DUPE_PPS_APDO,
};

static const char * const pdo_err_msg[] = {
	[PDO_ERR_NO_VSAFE5V] =
	" err: source/sink caps should at least have vSafe5V",
	[PDO_ERR_VSAFE5V_NOT_FIRST] =
	" err: vSafe5V Fixed Supply Object Shall always be the first object",
	[PDO_ERR_PDO_TYPE_NOT_IN_ORDER] =
	" err: PDOs should be in the following order: Fixed; Battery; Variable",
	[PDO_ERR_FIXED_NOT_SORTED] =
	" err: Fixed supply pdos should be in increasing order of their fixed voltage",
	[PDO_ERR_VARIABLE_BATT_NOT_SORTED] =
	" err: Variable/Battery supply pdos should be in increasing order of their minimum voltage",
	[PDO_ERR_DUPE_PDO] =
	" err: Variable/Batt supply pdos cannot have same min/max voltage",
	[PDO_ERR_PPS_APDO_NOT_SORTED] =
	" err: Programmable power supply apdos should be in increasing order of their maximum voltage",
	[PDO_ERR_DUPE_PPS_APDO] =
	" err: Programmable power supply apdos cannot have same min/max voltage and max current",
};

static enum pdo_err tcpm_caps_err(struct udevice *dev, const u32 *pdo,
				  unsigned int nr_pdo)
{
	unsigned int i;

	/* Should at least contain vSafe5v */
	if (nr_pdo < 1)
		return PDO_ERR_NO_VSAFE5V;

	/* The vSafe5V Fixed Supply Object Shall always be the first object */
	if (pdo_type(pdo[0]) != PDO_TYPE_FIXED ||
	    pdo_fixed_voltage(pdo[0]) != VSAFE5V)
		return PDO_ERR_VSAFE5V_NOT_FIRST;

	for (i = 1; i < nr_pdo; i++) {
		if (pdo_type(pdo[i]) < pdo_type(pdo[i - 1])) {
			return PDO_ERR_PDO_TYPE_NOT_IN_ORDER;
		} else if (pdo_type(pdo[i]) == pdo_type(pdo[i - 1])) {
			enum pd_pdo_type type = pdo_type(pdo[i]);

			switch (type) {
			/*
			 * The remaining Fixed Supply Objects, if
			 * present, shall be sent in voltage order;
			 * lowest to highest.
			 */
			case PDO_TYPE_FIXED:
				if (pdo_fixed_voltage(pdo[i]) <=
				    pdo_fixed_voltage(pdo[i - 1]))
					return PDO_ERR_FIXED_NOT_SORTED;
				break;
			/*
			 * The Battery Supply Objects and Variable
			 * supply, if present shall be sent in Minimum
			 * Voltage order; lowest to highest.
			 */
			case PDO_TYPE_VAR:
			case PDO_TYPE_BATT:
				if (pdo_min_voltage(pdo[i]) <
				    pdo_min_voltage(pdo[i - 1]))
					return PDO_ERR_VARIABLE_BATT_NOT_SORTED;
				else if ((pdo_min_voltage(pdo[i]) ==
					  pdo_min_voltage(pdo[i - 1])) &&
					 (pdo_max_voltage(pdo[i]) ==
					  pdo_max_voltage(pdo[i - 1])))
					return PDO_ERR_DUPE_PDO;
				break;
			/*
			 * The Programmable Power Supply APDOs, if present,
			 * shall be sent in Maximum Voltage order;
			 * lowest to highest.
			 */
			case PDO_TYPE_APDO:
				if (pdo_apdo_type(pdo[i]) != APDO_TYPE_PPS)
					break;

				if (pdo_pps_apdo_max_voltage(pdo[i]) <
				    pdo_pps_apdo_max_voltage(pdo[i - 1]))
					return PDO_ERR_PPS_APDO_NOT_SORTED;
				else if (pdo_pps_apdo_min_voltage(pdo[i]) ==
					  pdo_pps_apdo_min_voltage(pdo[i - 1]) &&
					 pdo_pps_apdo_max_voltage(pdo[i]) ==
					  pdo_pps_apdo_max_voltage(pdo[i - 1]) &&
					 pdo_pps_apdo_max_current(pdo[i]) ==
					  pdo_pps_apdo_max_current(pdo[i - 1]))
					return PDO_ERR_DUPE_PPS_APDO;
				break;
			default:
				dev_err(dev, "TCPM: Unknown pdo type\n");
			}
		}
	}

	return PDO_NO_ERR;
}

static int tcpm_validate_caps(struct udevice *dev, const u32 *pdo,
			      unsigned int nr_pdo)
{
	enum pdo_err err_index = tcpm_caps_err(dev, pdo, nr_pdo);

	if (err_index != PDO_NO_ERR) {
		dev_err(dev, "TCPM:%s\n", pdo_err_msg[err_index]);
		return -EINVAL;
	}

	return 0;
}

/*
 * PD (data, control) command handling functions
 */
static inline enum tcpm_state ready_state(struct tcpm_port *port)
{
	if (port->pwr_role == TYPEC_SOURCE)
		return SRC_READY;
	else
		return SNK_READY;
}

static void tcpm_pd_data_request(struct udevice *dev,
				 const struct pd_message *msg)
{
	enum pd_data_msg_type type = pd_header_type_le(msg->header);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	unsigned int cnt = pd_header_cnt_le(msg->header);
	unsigned int rev = pd_header_rev_le(msg->header);
	unsigned int i;

	switch (type) {
	case PD_DATA_SOURCE_CAP:
		for (i = 0; i < cnt; i++)
			port->source_caps[i] = le32_to_cpu(msg->payload[i]);

		port->nr_source_caps = cnt;

		tcpm_validate_caps(dev, port->source_caps,
				   port->nr_source_caps);

		/*
		 * Adjust revision in subsequent message headers, as required,
		 * to comply with 6.2.1.1.5 of the USB PD 3.0 spec. We don't
		 * support Rev 1.0 so just do nothing in that scenario.
		 */
		if (rev == PD_REV10)
			break;

		if (rev < PD_MAX_REV)
			port->negotiated_rev = rev;

		if ((pdo_type(port->source_caps[0]) == PDO_TYPE_FIXED) &&
		    (port->source_caps[0] & PDO_FIXED_DUAL_ROLE) &&
		    (port->source_caps[0] & PDO_FIXED_DATA_SWAP)) {
			/* Dual role power and data, eg: self-powered Type-C */
			port->wait_dr_swap_message = true;
		} else {
			/* Non-Dual role power, eg: adapter */
			port->wait_dr_swap_message = false;
		}

		/*
		 * This message may be received even if VBUS is not
		 * present. This is quite unexpected; see USB PD
		 * specification, sections 8.3.3.6.3.1 and 8.3.3.6.3.2.
		 * However, at the same time, we must be ready to
		 * receive this message and respond to it 15ms after
		 * receiving PS_RDY during power swap operations, no matter
		 * if VBUS is available or not (USB PD specification,
		 * section 6.5.9.2).
		 * So we need to accept the message either way,
		 * but be prepared to keep waiting for VBUS after it was
		 * handled.
		 */
		tcpm_set_state(dev, SNK_NEGOTIATE_CAPABILITIES, 0);
		break;
	case PD_DATA_REQUEST:
		/*
		 * Adjust revision in subsequent message headers, as required,
		 * to comply with 6.2.1.1.5 of the USB PD 3.0 spec. We don't
		 * support Rev 1.0 so just reject in that scenario.
		 */
		if (rev == PD_REV10) {
			tcpm_queue_message(dev, PD_MSG_CTRL_REJECT);
			break;
		}

		if (rev < PD_MAX_REV)
			port->negotiated_rev = rev;

		port->sink_request = le32_to_cpu(msg->payload[0]);

		tcpm_set_state(dev, SRC_NEGOTIATE_CAPABILITIES, 0);
		break;
	case PD_DATA_SINK_CAP:
		/* We don't do anything with this at the moment... */
		for (i = 0; i < cnt; i++)
			port->sink_caps[i] = le32_to_cpu(msg->payload[i]);

		port->nr_sink_caps = cnt;
		break;
	case PD_DATA_VENDOR_DEF:
		/*
		 * Park the VDM where the initiator's wait loop can see it.
		 * Nothing is interpreted here: this runs inside the receive
		 * path that tcpm_pd_transmit() and tcpm_vdm_wait() are both
		 * pumping, and deciding what a VDM means is the caller's job.
		 *
		 * An unsolicited VDM -- an Attention from the partner, most
		 * often a DisplayPort HPD change -- lands here too. When an
		 * exchange is in flight it is DROPPED HERE rather than parked,
		 * because the slot is one deep and the receive path can now
		 * deliver several messages between two looks at it: park the
		 * Attention and it overwrites the ACK that arrived just before
		 * it, and no amount of filtering in the wait loop can recover
		 * a message that is already gone. Attention takes no response,
		 * so dropping it costs nothing but a hotplug we were not
		 * watching for.
		 */
		if (cnt > VDO_MAX_SIZE)
			cnt = VDO_MAX_SIZE;

		if (port->vdm_waiting && cnt) {
			u32 hdr = le32_to_cpu(msg->payload[0]);

			if (!PD_VDO_SVDM(hdr) ||
			    PD_VDO_VID(hdr) != port->vdm_wait_svid ||
			    PD_VDO_CMD(hdr) != port->vdm_wait_cmd ||
			    PD_VDO_CMDT(hdr) == CMDT_INIT)
				break;
		}

		for (i = 0; i < cnt; i++)
			port->vdm_rx[i] = le32_to_cpu(msg->payload[i]);

		port->vdm_rx_cnt = cnt;
		break;
	default:
		break;
	}
}

static void tcpm_pd_ctrl_request(struct udevice *dev,
				 const struct pd_message *msg)
{
	enum pd_ctrl_msg_type type = pd_header_type_le(msg->header);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	enum tcpm_state next_state;

	switch (type) {
	case PD_CTRL_GOOD_CRC:
	case PD_CTRL_PING:
		break;
	case PD_CTRL_GET_SOURCE_CAP:
		switch (port->state) {
		case SRC_READY:
		case SNK_READY:
			tcpm_queue_message(dev, PD_MSG_DATA_SOURCE_CAP);
			break;
		default:
			tcpm_queue_message(dev, PD_MSG_CTRL_REJECT);
			break;
		}
		break;
	case PD_CTRL_GET_SINK_CAP:
		switch (port->state) {
		case SRC_READY:
		case SNK_READY:
			tcpm_queue_message(dev, PD_MSG_DATA_SINK_CAP);
			break;
		default:
			tcpm_queue_message(dev, PD_MSG_CTRL_REJECT);
			break;
		}
		break;
	case PD_CTRL_GOTO_MIN:
		break;
	case PD_CTRL_PS_RDY:
		switch (port->state) {
		case SNK_TRANSITION_SINK:
			if (port->vbus_present) {
				tcpm_set_current_limit(dev,
						       port->req_current_limit,
						       port->req_supply_voltage);
				port->explicit_contract = true;
				tcpm_set_state(dev, SNK_READY, 0);
			} else {
				/*
				 * Seen after power swap. Keep waiting for VBUS
				 * in a transitional state.
				 */
				tcpm_set_state(dev,
					       SNK_TRANSITION_SINK_VBUS, 0);
			}
			break;
		default:
			break;
		}
		break;
	case PD_CTRL_REJECT:
	case PD_CTRL_WAIT:
	case PD_CTRL_NOT_SUPP:
		switch (port->state) {
		case SNK_NEGOTIATE_CAPABILITIES:
			/* USB PD specification, Figure 8-43 */
			if (port->explicit_contract)
				next_state = SNK_READY;
			else
				next_state = SNK_WAIT_CAPABILITIES;

			tcpm_set_state(dev, next_state, 0);
			break;
		default:
			break;
		}
		break;
	case PD_CTRL_ACCEPT:
		switch (port->state) {
		case SNK_NEGOTIATE_CAPABILITIES:
			tcpm_set_state(dev, SNK_TRANSITION_SINK, 0);
			break;
		case SOFT_RESET_SEND:
			port->message_id = 0;
			port->rx_msgid = -1;
			if (port->pwr_role == TYPEC_SOURCE)
				next_state = SRC_SEND_CAPABILITIES;
			else
				next_state = SNK_WAIT_CAPABILITIES;
			tcpm_set_state(dev, next_state, 0);
			break;
		default:
			break;
		}
		break;
	case PD_CTRL_SOFT_RESET:
		tcpm_set_state(dev, SOFT_RESET, 0);
		break;
	case PD_CTRL_DR_SWAP:
		if (port->port_type != TYPEC_PORT_DRP) {
			tcpm_queue_message(dev, PD_MSG_CTRL_REJECT);
			break;
		}
		/*
		 * 6.3.9: If an alternate mode is active, a request to swap
		 * alternate modes shall trigger a port reset.
		 */
		switch (port->state) {
		case SRC_READY:
		case SNK_READY:
			tcpm_set_state(dev, DR_SWAP_ACCEPT, 0);
			break;
		default:
			tcpm_queue_message(dev, PD_MSG_CTRL_WAIT);
			break;
		}
		break;
	case PD_CTRL_PR_SWAP:
	case PD_CTRL_VCONN_SWAP:
	case PD_CTRL_GET_SOURCE_CAP_EXT:
	case PD_CTRL_GET_STATUS:
	case PD_CTRL_FR_SWAP:
	case PD_CTRL_GET_PPS_STATUS:
	case PD_CTRL_GET_COUNTRY_CODES:
		/* Currently not supported */
		dev_err(dev, "TCPM: Currently not supported type %#x\n", type);
		tcpm_queue_message(dev, PD_MSG_CTRL_NOT_SUPP);
		break;
	default:
		dev_err(dev, "TCPM: Unrecognized ctrl message type %#x\n", type);
		break;
	}
}

static void tcpm_pd_rx_handler(struct udevice *dev,
			       const struct pd_message *msg)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	unsigned int cnt = pd_header_cnt_le(msg->header);
	bool remote_is_host, local_is_host;

	dev_dbg(dev, "TCPM: PD RX, header: %#x [%d]\n",
		le16_to_cpu(msg->header), port->attached);

	if (port->attached) {
		enum pd_ctrl_msg_type type = pd_header_type_le(msg->header);
		unsigned int msgid = pd_header_msgid_le(msg->header);

		/*
		 * USB PD standard, 6.6.1.2:
		 * "... if MessageID value in a received Message is the
		 * same as the stored value, the receiver shall return a
		 * GoodCRC Message with that MessageID value and drop
		 * the Message (this is a retry of an already received
		 * Message). Note: this shall not apply to the Soft_Reset
		 * Message which always has a MessageID value of zero."
		 */
		if (msgid == port->rx_msgid && type != PD_CTRL_SOFT_RESET)
			return;
		port->rx_msgid = msgid;

		/*
		 * If both ends believe to be DFP/host, we have a data role
		 * mismatch.
		 */
		remote_is_host = !!(le16_to_cpu(msg->header) & PD_HEADER_DATA_ROLE);
		local_is_host = port->data_role == TYPEC_HOST;
		if (remote_is_host == local_is_host) {
			dev_err(dev, "TCPM: data role mismatch, initiating error recovery (hdr=%04x type=%u cnt=%u msgid=%u remote=%s local=%s state=%s)\n",
				le16_to_cpu(msg->header), type, cnt, msgid,
				remote_is_host ? "host" : "device",
				local_is_host ? "host" : "device",
				tcpm_states[port->state]);
			tcpm_set_state(dev, ERROR_RECOVERY, 0);
		} else {
			if (cnt)
				tcpm_pd_data_request(dev, msg);
			else
				tcpm_pd_ctrl_request(dev, msg);
		}
	}
}

void tcpm_pd_receive(struct udevice *dev, const struct pd_message *msg)
{
	tcpm_reset_event_cnt(dev);
	tcpm_pd_rx_handler(dev, msg);
}

static int tcpm_pd_send_control(struct udevice *dev,
				enum pd_ctrl_msg_type type)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	struct pd_message msg;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(type, port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 0);

	return tcpm_pd_transmit(dev, TCPC_TX_SOP, &msg);
}

/*
 * Send queued message without affecting state.
 * Return true if state machine should go back to sleep,
 * false otherwise.
 */
static bool tcpm_send_queued_message(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	enum pd_msg_request queued_message;
	int max_messages = 100;

	do {
		queued_message = port->queued_message;
		port->queued_message = PD_MSG_NONE;
		max_messages--;

		switch (queued_message) {
		case PD_MSG_CTRL_WAIT:
			tcpm_pd_send_control(dev, PD_CTRL_WAIT);
			break;
		case PD_MSG_CTRL_REJECT:
			tcpm_pd_send_control(dev, PD_CTRL_REJECT);
			break;
		case PD_MSG_CTRL_NOT_SUPP:
			tcpm_pd_send_control(dev, PD_CTRL_NOT_SUPP);
			break;
		case PD_MSG_DATA_SINK_CAP:
			tcpm_pd_send_sink_caps(dev);
			break;
		case PD_MSG_DATA_SOURCE_CAP:
			tcpm_pd_send_source_caps(dev);
			break;
		default:
			break;
		}
	} while (max_messages > 0 && port->queued_message != PD_MSG_NONE);

	if (!max_messages)
		dev_err(dev, "Aborted sending of too many queued messages\n");

	return false;
}

static int tcpm_pd_check_request(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	u32 pdo, rdo = port->sink_request;
	unsigned int max, op, pdo_max, index;
	enum pd_pdo_type type;

	index = rdo_index(rdo);
	if (!index || index > port->nr_src_pdo)
		return -EINVAL;

	pdo = port->src_pdo[index - 1];
	type = pdo_type(pdo);
	switch (type) {
	case PDO_TYPE_FIXED:
	case PDO_TYPE_VAR:
		max = rdo_max_current(rdo);
		op = rdo_op_current(rdo);
		pdo_max = pdo_max_current(pdo);

		if (op > pdo_max)
			return -EINVAL;
		if (max > pdo_max && !(rdo & RDO_CAP_MISMATCH))
			return -EINVAL;

		if (type == PDO_TYPE_FIXED)
			dev_dbg(dev, "TCPM: Requested %u mV, %u mA for %u / %u mA\n",
				pdo_fixed_voltage(pdo), pdo_max, op, max);
		else
			dev_dbg(dev, "TCPM: Requested %u -> %u mV, %u mA for %u / %u mA\n",
				pdo_min_voltage(pdo), pdo_max_voltage(pdo),
				pdo_max, op, max);
		break;
	case PDO_TYPE_BATT:
		max = rdo_max_power(rdo);
		op = rdo_op_power(rdo);
		pdo_max = pdo_max_power(pdo);

		if (op > pdo_max)
			return -EINVAL;
		if (max > pdo_max && !(rdo & RDO_CAP_MISMATCH))
			return -EINVAL;
		dev_info(dev, "TCPM: Requested %u -> %u mV, %u mW for %u / %u mW\n",
			 pdo_min_voltage(pdo), pdo_max_voltage(pdo),
			 pdo_max, op, max);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define min_power(x, y) min(pdo_max_power(x), pdo_max_power(y))
#define min_current(x, y) min(pdo_max_current(x), pdo_max_current(y))

static int tcpm_pd_select_pdo(struct udevice *dev, int *sink_pdo,
			      int *src_pdo)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	unsigned int i, j, max_src_mv = 0, min_src_mv = 0, max_mw = 0,
		     max_mv = 0, src_mw = 0, src_ma = 0, max_snk_mv = 0,
		     min_snk_mv = 0;
	int ret = -EINVAL;

	/*
	 * Select the source PDO providing the most power which has a
	 * matchig sink cap.
	 */
	for (i = 0; i < port->nr_source_caps; i++) {
		u32 pdo = port->source_caps[i];
		enum pd_pdo_type type = pdo_type(pdo);

		switch (type) {
		case PDO_TYPE_FIXED:
			max_src_mv = pdo_fixed_voltage(pdo);
			min_src_mv = max_src_mv;
			break;
		case PDO_TYPE_BATT:
		case PDO_TYPE_VAR:
			max_src_mv = pdo_max_voltage(pdo);
			min_src_mv = pdo_min_voltage(pdo);
			break;
		case PDO_TYPE_APDO:
			continue;
		default:
			dev_err(dev, "TCPM: Invalid source PDO type, ignoring\n");
			continue;
		}

		switch (type) {
		case PDO_TYPE_FIXED:
		case PDO_TYPE_VAR:
			src_ma = pdo_max_current(pdo);
			src_mw = src_ma * min_src_mv / 1000;
			break;
		case PDO_TYPE_BATT:
			src_mw = pdo_max_power(pdo);
			break;
		case PDO_TYPE_APDO:
			continue;
		default:
			dev_err(dev, "TCPM: Invalid source PDO type, ignoring\n");
			continue;
		}

		for (j = 0; j < port->nr_snk_pdo; j++) {
			pdo = port->snk_pdo[j];

			switch (pdo_type(pdo)) {
			case PDO_TYPE_FIXED:
				max_snk_mv = pdo_fixed_voltage(pdo);
				min_snk_mv = max_snk_mv;
				break;
			case PDO_TYPE_BATT:
			case PDO_TYPE_VAR:
				max_snk_mv = pdo_max_voltage(pdo);
				min_snk_mv = pdo_min_voltage(pdo);
				break;
			case PDO_TYPE_APDO:
				continue;
			default:
				dev_err(dev, "TCPM: Invalid sink PDO type, ignoring\n");
				continue;
			}

			if (max_src_mv <= max_snk_mv && min_src_mv >= min_snk_mv) {
				/* Prefer higher voltages if available */
				if ((src_mw == max_mw && min_src_mv > max_mv) ||
				    src_mw > max_mw) {
					*src_pdo = i;
					*sink_pdo = j;
					max_mw = src_mw;
					max_mv = min_src_mv;
					ret = 0;
				}
			}
		}
	}

	return ret;
}

static int tcpm_pd_build_request(struct udevice *dev, u32 *rdo)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	unsigned int mv, ma, mw, flags;
	unsigned int max_ma, max_mw;
	enum pd_pdo_type type;
	u32 pdo, matching_snk_pdo;
	int src_pdo_index = 0;
	int snk_pdo_index = 0;
	int ret;

	ret = tcpm_pd_select_pdo(dev, &snk_pdo_index, &src_pdo_index);
	if (ret < 0)
		return ret;

	pdo = port->source_caps[src_pdo_index];
	matching_snk_pdo = port->snk_pdo[snk_pdo_index];
	type = pdo_type(pdo);

	switch (type) {
	case PDO_TYPE_FIXED:
		mv = pdo_fixed_voltage(pdo);
		break;
	case PDO_TYPE_BATT:
	case PDO_TYPE_VAR:
		mv = pdo_min_voltage(pdo);
		break;
	default:
		dev_err(dev, "TCPM: Invalid PDO selected!\n");
		return -EINVAL;
	}

	/* Select maximum available current within the sink pdo's limit */
	if (type == PDO_TYPE_BATT) {
		mw = min_power(pdo, matching_snk_pdo);
		ma = 1000 * mw / mv;
	} else {
		ma = min_current(pdo, matching_snk_pdo);
		mw = ma * mv / 1000;
	}

	flags = RDO_USB_COMM | RDO_NO_SUSPEND;

	/* Set mismatch bit if offered power is less than operating power */
	max_ma = ma;
	max_mw = mw;
	if (mw < port->operating_snk_mw) {
		flags |= RDO_CAP_MISMATCH;
		if (type == PDO_TYPE_BATT &&
		    (pdo_max_power(matching_snk_pdo) > pdo_max_power(pdo)))
			max_mw = pdo_max_power(matching_snk_pdo);
		else if (pdo_max_current(matching_snk_pdo) >
			 pdo_max_current(pdo))
			max_ma = pdo_max_current(matching_snk_pdo);
	}

	dev_dbg(dev, "TCPM: cc=%d cc1=%d cc2=%d vbus=%d vconn=%s polarity=%d\n",
		port->cc_req, port->cc1, port->cc2, port->vbus_source,
		port->vconn_role == TYPEC_SOURCE ? "source" : "sink",
		port->polarity);

	if (type == PDO_TYPE_BATT) {
		*rdo = RDO_BATT(src_pdo_index + 1, mw, max_mw, flags);

		dev_info(dev, "TCPM: requesting PDO %d: %u mV, %u mW%s\n",
			 src_pdo_index, mv, mw,
			 flags & RDO_CAP_MISMATCH ? " [mismatch]" : "");
	} else {
		*rdo = RDO_FIXED(src_pdo_index + 1, ma, max_ma, flags);

		dev_info(dev, "TCPM: requesting PDO %d: %u mV, %u mA%s\n",
			 src_pdo_index, mv, ma,
			 flags & RDO_CAP_MISMATCH ? " [mismatch]" : "");
	}

	port->req_current_limit = ma;
	port->req_supply_voltage = mv;

	return 0;
}

static int tcpm_pd_send_request(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	struct pd_message msg;
	int ret;
	u32 rdo;

	ret = tcpm_pd_build_request(dev, &rdo);
	if (ret < 0)
		return ret;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(PD_DATA_REQUEST,
				  port->pwr_role,
				  port->data_role,
				  port->negotiated_rev,
				  port->message_id, 1);
	msg.payload[0] = cpu_to_le32(rdo);

	return tcpm_pd_transmit(dev, TCPC_TX_SOP, &msg);
}

static int tcpm_set_vbus(struct udevice *dev, bool enable)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	int ret;

	if (enable && port->vbus_charge)
		return -EINVAL;

	dev_dbg(dev, "TCPM: set vbus = %d charge = %d\n",
		enable, port->vbus_charge);

	ret = drvops->set_vbus(dev, enable, port->vbus_charge);
	if (ret < 0)
		return ret;

	port->vbus_source = enable;
	return 0;
}

static int tcpm_set_charge(struct udevice *dev, bool charge)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	if (charge && port->vbus_source)
		return -EINVAL;

	if (charge != port->vbus_charge) {
		dev_dbg(dev, "TCPM: set vbus = %d charge = %d\n",
			port->vbus_source, charge);
		ret = drvops->set_vbus(dev, port->vbus_source,
					   charge);
		if (ret < 0)
			return ret;
	}
	port->vbus_charge = charge;
	return 0;
}

static bool tcpm_start_toggling(struct udevice *dev, enum typec_cc_status cc)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	if (!drvops->start_toggling)
		return false;

	dev_dbg(dev, "TCPM: Start toggling\n");
	ret = drvops->start_toggling(dev, port->port_type, cc);
	return ret == 0;
}

static int tcpm_init_vbus(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	ret = drvops->set_vbus(dev, false, false);
	port->vbus_source = false;
	port->vbus_charge = false;
	return ret;
}

static int tcpm_init_vconn(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	ret = drvops->set_vconn(dev, false);
	port->vconn_role = TYPEC_SINK;
	return ret;
}

static inline void tcpm_typec_connect(struct tcpm_port *port)
{
	if (!port->connected)
		port->connected = true;
}

static int tcpm_src_attach(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	enum typec_cc_polarity polarity =
				port->cc2 == TYPEC_CC_RD ? TYPEC_POLARITY_CC2
							 : TYPEC_POLARITY_CC1;
	int ret;

	if (port->attached)
		return 0;

	ret = tcpm_set_polarity(dev, polarity);
	if (ret < 0)
		return ret;

	ret = tcpm_set_roles(dev, true, TYPEC_SOURCE,
			     port->typec_data == TYPEC_PORT_DATA_UFP ?
			     TYPEC_DEVICE : TYPEC_HOST);
	if (ret < 0)
		return ret;

	ret = drvops->set_pd_rx(dev, true);
	if (ret < 0)
		goto out_disable_mux;

	/*
	 * USB Type-C specification, version 1.2,
	 * chapter 4.5.2.2.8.1 (Attached.SRC Requirements)
	 * Enable VCONN only if the non-RD port is set to RA.
	 */
	if ((polarity == TYPEC_POLARITY_CC1 && port->cc2 == TYPEC_CC_RA) ||
	    (polarity == TYPEC_POLARITY_CC2 && port->cc1 == TYPEC_CC_RA)) {
		ret = tcpm_set_vconn(dev, true);
		if (ret < 0)
			goto out_disable_pd;
	}

	ret = tcpm_set_vbus(dev, true);
	if (ret < 0)
		goto out_disable_vconn;

	port->pd_capable = false;

	port->partner = NULL;

	port->attached = true;

	return 0;

out_disable_vconn:
	tcpm_set_vconn(dev, false);
out_disable_pd:
	drvops->set_pd_rx(dev, false);
out_disable_mux:
	dev_err(dev, "TCPM: CC connected in %s as DFP\n",
		polarity ? "CC2" : "CC1");
	return 0;
}

static inline void tcpm_typec_disconnect(struct tcpm_port *port)
{
	if (port->connected) {
		port->partner = NULL;
		port->connected = false;
	}
}

static void tcpm_reset_port(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	tcpm_timer_uninit(dev);
	tcpm_typec_disconnect(port);
	tcpm_reset_event_cnt(dev);
	port->wait_dr_swap_message = false;
	port->attached = false;
	port->pd_capable = false;
	memset(&port->dp, 0, sizeof(port->dp));

	/*
	 * First Rx ID should be 0; set this to a sentinel of -1 so that
	 * we can check tcpm_pd_rx_handler() if we had seen it before.
	 */
	port->rx_msgid = -1;

	drvops->set_pd_rx(dev, false);
	tcpm_init_vbus(dev);	/* also disables charging */
	tcpm_init_vconn(dev);
	tcpm_set_current_limit(dev, 0, 0);
	tcpm_set_polarity(dev, TYPEC_POLARITY_CC1);
	tcpm_set_attached_state(dev, false);
	port->nr_sink_caps = 0;
}

static void tcpm_detach(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (tcpm_port_is_disconnected(port))
		port->hard_reset_count = 0;

	if (!port->attached)
		return;

	tcpm_reset_port(dev);
}

static void tcpm_src_detach(struct udevice *dev)
{
	tcpm_detach(dev);
}

static int tcpm_snk_attach(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	if (port->attached)
		return 0;

	ret = tcpm_set_polarity(dev, port->cc2 != TYPEC_CC_OPEN ?
				TYPEC_POLARITY_CC2 : TYPEC_POLARITY_CC1);
	if (ret < 0)
		return ret;

	/*
	 * The INITIAL data role is fixed by CC, not by the device tree. The Rd
	 * side of a Type-C connection is the UFP; the Rp side is the DFP. A
	 * sink that asserts DFP at attach is asserting a role the partner
	 * already holds, and the partner is right to refuse it.
	 *
	 * data-role = "host" says what this port may BECOME, not what it may
	 * claim before anyone has agreed. The way a power sink legitimately
	 * becomes the data host is the DR_Swap the partner offers -- which this
	 * stack already implements end to end (the Source_Capabilities
	 * DUAL_ROLE|DATA_SWAP probe that arms ->wait_dr_swap_message, the
	 * SNK_READY hold that waits for it, DR_SWAP_ACCEPT and
	 * DR_SWAP_CHANGE_DR) and which the monitor on this board actively
	 * sends.
	 *
	 * Measured on ROC-RK3399-PC Type-C1 against a 4K Samsung: attaching as
	 * TYPEC_HOST here collides with the partner's own DFP and the first
	 * Source_Capabilities is destroyed by tcpm_pd_rx_handler()'s data-role
	 * check, before any contract exists -- so the DR_Swap that would have
	 * granted the role never gets dispatched either:
	 *
	 *   hdr=1161 type=1 cnt=1 msgid=0 remote=host local=host
	 *                                       state=SNK_WAIT_CAPABILITIES
	 *   hdr=0769 type=9 cnt=0 msgid=3 remote=host local=host   (DR_Swap)
	 *
	 * tcpm_src_attach() is the other half and stays as it is: there the
	 * port presents Rp, so it genuinely IS the DFP and data-role = "host"
	 * is describing the attach, not overriding it.
	 */
	ret = tcpm_set_roles(dev, true, TYPEC_SINK, TYPEC_DEVICE);
	if (ret < 0)
		return ret;

	port->pd_capable = false;

	port->partner = NULL;

	port->attached = true;
	dev_info(dev, "TCPM: CC connected in %s as UFP\n",
		 port->cc1 != TYPEC_CC_OPEN ? "CC1" : "CC2");

	return 0;
}

static void tcpm_snk_detach(struct udevice *dev)
{
	tcpm_detach(dev);
}

static inline enum tcpm_state hard_reset_state(struct tcpm_port *port)
{
	/*
	 * A sink-only port that is not self-powered draws its own supply
	 * through this VBUS, and a Hard Reset makes the source cycle VBUS
	 * through vSafe0V -- i.e. it powers the board off. Before any PD
	 * contract exists on this attach there is nothing a Hard Reset could
	 * restore that Type-C current does not already give, so settle there.
	 * Same rule as the kernel's tcpm.c (linux-6.18.y-lc), so neither side
	 * of the boot hand-off resets a source the other one left running.
	 */
	if (port->port_type == TYPEC_PORT_SNK && !port->self_powered &&
	    !port->pd_capable && port->vbus_present)
		return SNK_READY;
	if (port->hard_reset_count < PD_N_HARD_RESET_COUNT)
		return HARD_RESET_SEND;
	if (port->pd_capable)
		return ERROR_RECOVERY;
	if (port->pwr_role == TYPEC_SOURCE)
		return SRC_UNATTACHED;
	if (port->state == SNK_WAIT_CAPABILITIES)
		return SNK_READY;
	return SNK_UNATTACHED;
}

static inline enum tcpm_state unattached_state(struct tcpm_port *port)
{
	if (port->port_type == TYPEC_PORT_DRP) {
		if (port->pwr_role == TYPEC_SOURCE)
			return SRC_UNATTACHED;
		else
			return SNK_UNATTACHED;
	} else if (port->port_type == TYPEC_PORT_SRC) {
		return SRC_UNATTACHED;
	}

	return SNK_UNATTACHED;
}

static void run_state_machine(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	port->enter_state = port->state;
	switch (port->state) {
	case TOGGLING:
		break;
	/* SRC states */
	case SRC_UNATTACHED:
		tcpm_src_detach(dev);
		if (tcpm_start_toggling(dev, tcpm_rp_cc(port))) {
			tcpm_set_state(dev, TOGGLING, 0);
			break;
		}
		tcpm_set_cc(dev, tcpm_rp_cc(port));
		if (port->port_type == TYPEC_PORT_DRP)
			tcpm_set_state(dev, SNK_UNATTACHED, PD_T_DRP_SNK);
		break;
	case SRC_ATTACH_WAIT:
		if (tcpm_port_is_source(port))
			tcpm_set_state(dev, SRC_ATTACHED, PD_T_CC_DEBOUNCE);
		break;

	case SRC_ATTACHED:
		ret = tcpm_src_attach(dev);
		if (ret < 0) {
			tcpm_set_state(dev, SRC_UNATTACHED, 0);
			break;
		}

		/*
		 * This used to jump straight to SRC_READY, with the comment
		 * "vbus control is not implemented, and the SRC detection
		 * process cannot be fully implemented". The shortcut skips
		 * SRC_STARTUP -- which is where negotiated_rev, message_id and
		 * rx_msgid are initialised -- and SRC_SEND_CAPABILITIES, so a
		 * source-role port arrived in SRC_READY having sent nothing,
		 * with negotiated_rev still 0. Every subsequent transmission
		 * then went out as a PD 1.0 message that no modern partner
		 * answers, and the visible symptom was every PD transmit
		 * timing out against a device that was plainly attached.
		 *
		 * VBUS control is implemented now (the TCPC driver's
		 * set_vbus op drives the port's vbus-supply), so the normal
		 * path works: _tcpm_pd_vbus_on() advances SRC_ATTACHED to
		 * SRC_STARTUP when the TCPC reports the rail up, and this
		 * arms the specified give-up timeout in case it never does.
		 *
		 * A board whose TCPC has no VBUS control still cannot be a
		 * source, and now says so by falling back to SRC_UNATTACHED
		 * instead of sitting in a SRC_READY it cannot transmit from.
		 *
		 * The already-on case is handled explicitly: if VBUS is up
		 * before the attach -- an always-on rail, or a port that was
		 * already sourcing -- there is no edge coming and waiting for
		 * one would time out for no reason.
		 */
		if (port->vbus_present)
			tcpm_set_state(dev, SRC_STARTUP, 0);
		else if (port->port_type == TYPEC_PORT_DRP)
			tcpm_set_state(dev, SRC_UNATTACHED, PD_T_PS_SOURCE_ON);
		else
			/*
			 * A source-ONLY port has no sink role to fall back on,
			 * so SRC_UNATTACHED would put it in a loop rather than
			 * a resting state. Keep the old destination for it --
			 * the VBUS-on event still overrides this delayed state
			 * if the rail does come up, so a port with working
			 * VBUS control is unaffected, and one without lands
			 * exactly where it used to.
			 */
			tcpm_set_state(dev, SRC_READY, PD_T_PS_SOURCE_ON);
		break;
	case SRC_STARTUP:
		port->caps_count = 0;
		/*
		 * Advertise PD 2.0, not PD_MAX_REV, when WE are the source.
		 *
		 * PD 3.0 puts obligations on a source that this stack does
		 * not meet: collision avoidance -- the Rp level signalling
		 * SinkTxOk/SinkTxNG that tells the sink when it may start an
		 * AMS -- and extended-message chunking. There is no
		 * tcpm_ams_finish() here and no SINK_TX handling anywhere, so
		 * a rev-3 header is a claim about behaviour that does not
		 * exist.
		 *
		 * Measured against a DisplayPort monitor: source caps went
		 * out with a rev-3 header and were GoodCRC'd, no Request ever
		 * came back, and the sink issued a Hard Reset ~150 ms later.
		 * That reset returns the port through ERROR_RECOVERY and
		 * PORT_RESET to a fresh attach, where SRC_STARTUP set
		 * PD_MAX_REV again -- so the existing fallback in
		 * SRC_SEND_CAPABILITIES_TIMEOUT, which steps the revision
		 * down after PD_N_HARD_RESET_COUNT attempts, was never
		 * reached and the port looped indefinitely.
		 *
		 * The sink path is untouched: as a sink we only have to
		 * understand rev 3, which this stack does.
		 *
		 * RETRACTED, and left here because the retraction is the
		 * useful part. Capping the source to PD 2.0 made things
		 * WORSE against the same monitor, measured: at rev 3 the
		 * Source_Capabilities were GoodCRC'd and the partner merely
		 * declined to Request; at rev 2 they were not acknowledged at
		 * all -- "IRQ: PD retry failed", TCPC_TX_FAILED -- so the
		 * message reached the wire and the partner's protocol layer
		 * refused it after three auto-retries.
		 *
		 * The real defect was never the revision. It was that this
		 * stack claimed PD 3.0 while presenting an Rp that expresses
		 * neither SinkTxOk nor SinkTxNG; that is now fixed in
		 * SRC_SEND_CAPABILITIES and SRC_READY, so advertise the
		 * revision we actually implement again.
		 */
		port->negotiated_rev = PD_MAX_REV;
		port->message_id = 0;
		port->rx_msgid = -1;
		port->explicit_contract = false;

		/*
		 * Power negotiation is a source-initiated AMS and runs until
		 * SRC_READY, so claim the channel for it the same way every
		 * other source-initiated sequence does.
		 */
		tcpm_ams_start(dev);
		tcpm_set_state(dev, SRC_SEND_CAPABILITIES, 0);
		break;
	case SRC_SEND_CAPABILITIES:
		port->caps_count++;
		if (port->caps_count > PD_N_CAPS_COUNT) {
			tcpm_set_state(dev, SRC_READY, 0);
			break;
		}
		ret = tcpm_pd_send_source_caps(dev);
		if (ret < 0) {
			tcpm_set_state(dev, SRC_SEND_CAPABILITIES,
				       PD_T_SEND_SOURCE_CAP);
		} else {
			/*
			 * Per standard, we should clear the reset counter here.
			 * However, that can result in state machine hang-ups.
			 * Reset it only in READY state to improve stability.
			 */
			/* port->hard_reset_count = 0; */
			port->caps_count = 0;
			port->pd_capable = true;
			tcpm_set_state_cond(dev, SRC_SEND_CAPABILITIES_TIMEOUT,
					    PD_T_SEND_SOURCE_CAP);
		}
		break;
	case SRC_SEND_CAPABILITIES_TIMEOUT:
		/*
		 * Error recovery for a PD_DATA_SOURCE_CAP reply timeout.
		 *
		 * PD 2.0 sinks are supposed to accept src-capabilities with a
		 * 3.0 header and simply ignore any src PDOs which the sink does
		 * not understand such as PPS but some 2.0 sinks instead ignore
		 * the entire PD_DATA_SOURCE_CAP message, causing contract
		 * negotiation to fail.
		 *
		 * After PD_N_HARD_RESET_COUNT hard-reset attempts, we try
		 * sending src-capabilities with a lower PD revision to
		 * make these broken sinks work.
		 */
		if (port->hard_reset_count < PD_N_HARD_RESET_COUNT) {
			tcpm_set_state(dev, HARD_RESET_SEND, 0);
		} else if (port->negotiated_rev > PD_REV20) {
			port->negotiated_rev--;
			port->hard_reset_count = 0;
			tcpm_set_state(dev, SRC_SEND_CAPABILITIES, 0);
		} else {
			tcpm_set_state(dev, hard_reset_state(port), 0);
		}
		break;
	case SRC_NEGOTIATE_CAPABILITIES:
		ret = tcpm_pd_check_request(dev);
		if (ret < 0) {
			tcpm_pd_send_control(dev, PD_CTRL_REJECT);
			if (!port->explicit_contract) {
				tcpm_set_state(dev,
					       SRC_WAIT_NEW_CAPABILITIES, 0);
			} else {
				tcpm_set_state(dev, SRC_READY, 0);
			}
		} else {
			tcpm_pd_send_control(dev, PD_CTRL_ACCEPT);
			tcpm_set_state(dev, SRC_TRANSITION_SUPPLY,
				       PD_T_SRC_TRANSITION);
		}
		break;
	case SRC_TRANSITION_SUPPLY:
		/* XXX: regulator_set_voltage(vbus, ...) */
		tcpm_pd_send_control(dev, PD_CTRL_PS_RDY);
		port->explicit_contract = true;
		tcpm_set_state_cond(dev, SRC_READY, 0);
		break;
	case SRC_READY:
		port->hard_reset_count = 0;

		/*
		 * The power-negotiation AMS ends here, whether it produced a
		 * contract or timed out into this state. Releasing the channel
		 * is what raises the Rp to SinkTxOk and tells the sink it may
		 * speak.
		 */
		tcpm_ams_finish(dev);
		tcpm_typec_connect(port);
		break;
	case SRC_WAIT_NEW_CAPABILITIES:
		/* Nothing to do... */
		break;

	/* SNK states */
	case SNK_UNATTACHED:
		tcpm_snk_detach(dev);
		if (tcpm_start_toggling(dev, TYPEC_CC_RD)) {
			tcpm_set_state(dev, TOGGLING, 0);
			break;
		}
		tcpm_set_cc(dev, TYPEC_CC_RD);
		if (port->port_type == TYPEC_PORT_DRP)
			tcpm_set_state(dev, SRC_UNATTACHED, PD_T_DRP_SRC);
		break;
	case SNK_ATTACH_WAIT:
		if ((port->cc1 == TYPEC_CC_OPEN &&
		     port->cc2 != TYPEC_CC_OPEN) ||
		    (port->cc1 != TYPEC_CC_OPEN &&
		     port->cc2 == TYPEC_CC_OPEN))
			tcpm_set_state(dev, SNK_DEBOUNCED,
				       PD_T_CC_DEBOUNCE);
		else if (tcpm_port_is_disconnected(port))
			tcpm_set_state(dev, SNK_UNATTACHED,
				       PD_T_CC_DEBOUNCE);
		break;
	case SNK_DEBOUNCED:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(dev, SNK_UNATTACHED, PD_T_PD_DEBOUNCE);
		else if (port->vbus_present)
			tcpm_set_state(dev, SNK_ATTACHED, 0);
		else
			/* Wait for VBUS, but not forever */
			tcpm_set_state(dev, PORT_RESET, PD_T_PS_SOURCE_ON);
		break;
	case SNK_ATTACHED:
		ret = tcpm_snk_attach(dev);
		if (ret < 0)
			tcpm_set_state(dev, SNK_UNATTACHED, 0);
		else
			tcpm_set_state(dev, SNK_STARTUP, 0);
		break;
	case SNK_STARTUP:
		port->negotiated_rev = PD_MAX_REV;
		port->message_id = 0;
		port->rx_msgid = -1;
		port->explicit_contract = false;
		tcpm_set_state(dev, SNK_DISCOVERY, 0);
		break;
	case SNK_DISCOVERY:
		if (port->vbus_present) {
			tcpm_set_current_limit(dev,
					       tcpm_get_current_limit(port),
					       5000);
			tcpm_set_charge(dev, true);
			tcpm_set_state(dev, SNK_WAIT_CAPABILITIES, 0);
			break;
		}
		/*
		 * For DRP, timeouts differ. Also, handling is supposed to be
		 * different and much more complex (dead battery detection;
		 * see USB power delivery specification, section 8.3.3.6.1.5.1).
		 */
		tcpm_set_state(dev, hard_reset_state(port),
			       port->port_type == TYPEC_PORT_DRP ?
					PD_T_DB_DETECT : PD_T_NO_RESPONSE);
		break;
	case SNK_DISCOVERY_DEBOUNCE:
		tcpm_set_state(dev, SNK_DISCOVERY_DEBOUNCE_DONE,
			       PD_T_CC_DEBOUNCE);
		break;
	case SNK_DISCOVERY_DEBOUNCE_DONE:
		tcpm_set_state(dev, unattached_state(port), 0);
		break;
	case SNK_WAIT_CAPABILITIES:
		ret = drvops->set_pd_rx(dev, true);
		if (ret < 0) {
			tcpm_set_state(dev, SNK_READY, 0);
			break;
		}
		/*
		 * If VBUS has never been low, and we time out waiting
		 * for source cap, try a soft reset first, in case we
		 * were already in a stable contract before this boot.
		 * Do this only once.
		 */
		if (port->vbus_never_low) {
			port->vbus_never_low = false;
			tcpm_set_state(dev, SOFT_RESET_SEND,
				       PD_T_SINK_WAIT_CAP);
		} else {
			tcpm_set_state(dev, hard_reset_state(port),
				       PD_T_SINK_WAIT_CAP);
		}
		break;
	case SNK_NEGOTIATE_CAPABILITIES:
		port->pd_capable = true;
		port->hard_reset_count = 0;
		ret = tcpm_pd_send_request(dev);
		if (ret < 0) {
			/* Let the Source send capabilities again. */
			tcpm_set_state(dev, SNK_WAIT_CAPABILITIES, 0);
		} else {
			tcpm_set_state_cond(dev, hard_reset_state(port),
					    PD_T_SENDER_RESPONSE);
		}
		break;
	case SNK_TRANSITION_SINK:
	case SNK_TRANSITION_SINK_VBUS:
		tcpm_set_state(dev, hard_reset_state(port),
			       PD_T_PS_TRANSITION);
		break;
	case SNK_READY:
		port->update_sink_caps = false;
		tcpm_typec_connect(port);
		/*
		 * Re-arm the pump's quiet-time budget: a self-powered Type-C
		 * device gets TCPM_POLL_EVENT_TIME_OUT ms from here to send its
		 * DR_Swap before the probe-time pump stops waiting for it.
		 */
		if (port->wait_dr_swap_message)
			tcpm_reset_event_cnt(dev);

		break;

	/* Hard_Reset states */
	case HARD_RESET_SEND:
		tcpm_pd_transmit(dev, TCPC_TX_HARD_RESET, NULL);
		tcpm_set_state(dev, HARD_RESET_START, 0);
		port->wait_dr_swap_message = false;
		break;
	case HARD_RESET_START:
		port->hard_reset_count++;
		drvops->set_pd_rx(dev, false);
		port->nr_sink_caps = 0;
		/*
		 * A hard reset takes the partner out of every alt mode, and
		 * this state leaves ->attached true and never reaches
		 * tcpm_reset_port(), so it is its own invalidation site. This
		 * monitor exercises it ~150 ms after Source_Capabilities.
		 */
		memset(&port->dp, 0, sizeof(port->dp));
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_state(dev, SRC_HARD_RESET_VBUS_OFF,
				       PD_T_PS_HARD_RESET);
		else
			tcpm_set_state(dev, SNK_HARD_RESET_SINK_OFF, 0);
		break;
	case SRC_HARD_RESET_VBUS_OFF:
		tcpm_set_vconn(dev, true);
		tcpm_set_vbus(dev, false);
		tcpm_set_roles(dev, port->self_powered, TYPEC_SOURCE,
			       TYPEC_HOST);
		tcpm_set_state(dev, SRC_HARD_RESET_VBUS_ON, PD_T_SRC_RECOVER);
		break;
	case SRC_HARD_RESET_VBUS_ON:
		tcpm_set_vconn(dev, true);
		tcpm_set_vbus(dev, true);
		drvops->set_pd_rx(dev, true);
		tcpm_set_attached_state(dev, true);
		tcpm_set_state(dev, SRC_UNATTACHED, PD_T_PS_SOURCE_ON);
		break;
	case SNK_HARD_RESET_SINK_OFF:
		tcpm_set_vconn(dev, false);
		if (port->pd_capable)
			tcpm_set_charge(dev, false);
		tcpm_set_roles(dev, port->self_powered, TYPEC_SINK,
			       TYPEC_DEVICE);
		/*
		 * VBUS may or may not toggle, depending on the adapter.
		 * If it doesn't toggle, transition to SNK_HARD_RESET_SINK_ON
		 * directly after timeout.
		 */
		tcpm_set_state(dev, SNK_HARD_RESET_SINK_ON, PD_T_SAFE_0V);
		break;
	case SNK_HARD_RESET_WAIT_VBUS:
		/* Assume we're disconnected if VBUS doesn't come back. */
		tcpm_set_state(dev, SNK_UNATTACHED,
			       PD_T_SRC_RECOVER_MAX + PD_T_SRC_TURN_ON);
		break;
	case SNK_HARD_RESET_SINK_ON:
		/* Note: There is no guarantee that VBUS is on in this state */
		/*
		 * XXX:
		 * The specification suggests that dual mode ports in sink
		 * mode should transition to state PE_SRC_Transition_to_default.
		 * See USB power delivery specification chapter 8.3.3.6.1.3.
		 * This would mean to
		 * - turn off VCONN, reset power supply
		 * - request hardware reset
		 * - turn on VCONN
		 * - Transition to state PE_Src_Startup
		 * SNK only ports shall transition to state Snk_Startup
		 * (see chapter 8.3.3.3.8).
		 * Similar, dual-mode ports in source mode should transition
		 * to PE_SNK_Transition_to_default.
		 */
		if (port->pd_capable) {
			tcpm_set_current_limit(dev,
					       tcpm_get_current_limit(port),
					       5000);
			tcpm_set_charge(dev, true);
		}
		tcpm_set_attached_state(dev, true);
		tcpm_set_state(dev, SNK_STARTUP, 0);
		break;

	/* Soft_Reset states */
	case SOFT_RESET:
		port->message_id = 0;
		port->rx_msgid = -1;
		tcpm_pd_send_control(dev, PD_CTRL_ACCEPT);
		if (port->pwr_role == TYPEC_SOURCE)
			tcpm_set_state(dev, SRC_SEND_CAPABILITIES, 0);
		else
			tcpm_set_state(dev, SNK_WAIT_CAPABILITIES, 0);
		break;
	case SOFT_RESET_SEND:
		port->message_id = 0;
		port->rx_msgid = -1;
		if (tcpm_pd_send_control(dev, PD_CTRL_SOFT_RESET))
			tcpm_set_state_cond(dev, hard_reset_state(port), 0);
		else
			tcpm_set_state_cond(dev, hard_reset_state(port),
					    PD_T_SENDER_RESPONSE);
		break;

	/* DR_Swap states */
	case DR_SWAP_ACCEPT:
		tcpm_pd_send_control(dev, PD_CTRL_ACCEPT);
		tcpm_set_state_cond(dev, DR_SWAP_CHANGE_DR, 0);
		break;
	case DR_SWAP_CHANGE_DR:
		if (port->data_role == TYPEC_HOST) {
			tcpm_set_roles(dev, true, port->pwr_role,
				       TYPEC_DEVICE);
		} else {
			tcpm_set_roles(dev, true, port->pwr_role,
				       TYPEC_HOST);
		}
		/*
		 * The DFP is who a DisplayPort alt mode was entered as, so a
		 * data-role swap invalidates it even though the connection
		 * survives.
		 */
		memset(&port->dp, 0, sizeof(port->dp));
		/* DR_swap process complete, wait_dr_swap_message is cleared */
		port->wait_dr_swap_message = false;
		tcpm_set_state(dev, ready_state(port), 0);
		break;
	case ERROR_RECOVERY:
		tcpm_set_state(dev, PORT_RESET, 0);
		break;
	case PORT_RESET:
		tcpm_reset_port(dev);
		if (port->self_powered)
			tcpm_set_cc(dev, TYPEC_CC_OPEN);
		else
			tcpm_set_cc(dev, tcpm_default_state(port) == SNK_UNATTACHED ?
				    TYPEC_CC_RD : tcpm_rp_cc(port));
		tcpm_set_state(dev, PORT_RESET_WAIT_OFF,
			       PD_T_ERROR_RECOVERY);
		break;
	case PORT_RESET_WAIT_OFF:
		tcpm_set_state(dev,
			       tcpm_default_state(port),
			       port->vbus_present ? PD_T_PS_SOURCE_OFF : 0);
		break;
	default:
		dev_err(dev, "TCPM: Unexpected port state %d\n", port->state);
		break;
	}
}

static void tcpm_state_machine(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	enum tcpm_state prev_state;

	mutex_lock(&port->lock);
	port->state_machine_running = true;

	if (port->queued_message && tcpm_send_queued_message(dev))
		goto done;

	/* If we were queued due to a delayed state change, update it now */
	if (port->delayed_state) {
		dev_dbg(dev, "TCPM: state change %s -> %s [delayed %ld ms]\n",
			tcpm_states[port->state],
			tcpm_states[port->delayed_state], port->delay_ms);
		port->prev_state = port->state;
		port->state = port->delayed_state;
		port->delayed_state = INVALID_STATE;
	}

	/*
	 * Continue running as long as we have (non-delayed) state changes
	 * to make.
	 */
	do {
		prev_state = port->state;
		run_state_machine(dev);
		if (port->queued_message)
			tcpm_send_queued_message(dev);
	} while (port->state != prev_state && !port->delayed_state);

done:
	port->state_machine_running = false;
	mutex_unlock(&port->lock);
}

static void _tcpm_cc_change(struct udevice *dev, enum typec_cc_status cc1,
			    enum typec_cc_status cc2)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	enum typec_cc_status old_cc1, old_cc2;
	enum tcpm_state new_state;

	old_cc1 = port->cc1;
	old_cc2 = port->cc2;
	port->cc1 = cc1;
	port->cc2 = cc2;

	dev_dbg(dev, "TCPM: CC1: %u -> %u, CC2: %u -> %u [state %s, polarity %d, %s]\n",
		old_cc1, cc1, old_cc2, cc2, tcpm_states[port->state],
		port->polarity,
		tcpm_port_is_disconnected(port) ? "disconnected" : "connected");

	switch (port->state) {
	case TOGGLING:
		if (tcpm_port_is_source(port))
			tcpm_set_state(dev, SRC_ATTACH_WAIT, 0);
		else if (tcpm_port_is_sink(port))
			tcpm_set_state(dev, SNK_ATTACH_WAIT, 0);
		break;
	case SRC_UNATTACHED:
	case SRC_ATTACH_WAIT:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(dev, SRC_UNATTACHED, 0);
		else if (cc1 != old_cc1 || cc2 != old_cc2)
			tcpm_set_state(dev, SRC_ATTACH_WAIT, 0);
		break;
	case SRC_ATTACHED:
	case SRC_SEND_CAPABILITIES:
	case SRC_READY:
		if (tcpm_port_is_disconnected(port) ||
		    !tcpm_port_is_source(port))
			tcpm_set_state(dev, SRC_UNATTACHED, 0);
		break;
	case SNK_UNATTACHED:
		if (tcpm_port_is_sink(port))
			tcpm_set_state(dev, SNK_ATTACH_WAIT, 0);
		break;
	case SNK_ATTACH_WAIT:
		if ((port->cc1 == TYPEC_CC_OPEN &&
		     port->cc2 != TYPEC_CC_OPEN) ||
		    (port->cc1 != TYPEC_CC_OPEN &&
		     port->cc2 == TYPEC_CC_OPEN))
			new_state = SNK_DEBOUNCED;
		else if (tcpm_port_is_disconnected(port))
			new_state = SNK_UNATTACHED;
		else
			break;
		if (new_state != port->delayed_state)
			tcpm_set_state(dev, SNK_ATTACH_WAIT, 0);
		break;
	case SNK_DEBOUNCED:
		if (tcpm_port_is_disconnected(port))
			new_state = SNK_UNATTACHED;
		else if (port->vbus_present)
			/*
			 * Linux branches to SRC_TRY here when the port prefers
			 * the source role. This stack has no Try.SRC cluster,
			 * and the placeholder was INVALID_STATE -- so a port
			 * declaring try-power-role = "source" was driven into
			 * a state with no handler rather than simply attaching
			 * as a sink. Attach; a preference we cannot act on is
			 * not a reason to wedge.
			 */
			new_state = SNK_ATTACHED;
		else
			new_state = SNK_UNATTACHED;
		if (new_state != port->delayed_state)
			tcpm_set_state(dev, SNK_DEBOUNCED, 0);
		break;
	case SNK_READY:
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(dev, unattached_state(port), 0);
		else if (!port->pd_capable &&
			 (cc1 != old_cc1 || cc2 != old_cc2))
			tcpm_set_current_limit(dev,
					       tcpm_get_current_limit(port),
					       5000);
		break;

	case SNK_DISCOVERY:
		/* CC line is unstable, wait for debounce */
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(dev, SNK_DISCOVERY_DEBOUNCE, 0);
		break;
	case SNK_DISCOVERY_DEBOUNCE:
		break;

	case PORT_RESET:
	case PORT_RESET_WAIT_OFF:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore CC changes here.
		 */
		break;
	default:
		/*
		 * While acting as sink and auto vbus discharge is enabled, Allow disconnect
		 * to be driven by vbus disconnect.
		 */
		if (tcpm_port_is_disconnected(port))
			tcpm_set_state(dev, unattached_state(port), 0);
		break;
	}
}

static void _tcpm_pd_vbus_on(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	dev_dbg(dev, "TCPM: VBUS on event\n");
	port->vbus_present = true;
	/*
	 * When vbus_present is true i.e. Voltage at VBUS is greater than VSAFE5V implicitly
	 * states that vbus is not at VSAFE0V, hence clear the vbus_vsafe0v flag here.
	 */
	port->vbus_vsafe0v = false;

	switch (port->state) {
	case SNK_TRANSITION_SINK_VBUS:
		port->explicit_contract = true;
		tcpm_set_state(dev, SNK_READY, 0);
		break;
	case SNK_DISCOVERY:
		tcpm_set_state(dev, SNK_DISCOVERY, 0);
		break;
	case SNK_DEBOUNCED:
		tcpm_set_state(dev, SNK_ATTACHED, 0);
		break;
	case SNK_HARD_RESET_WAIT_VBUS:
		tcpm_set_state(dev, SNK_HARD_RESET_SINK_ON, 0);
		break;
	case SRC_ATTACHED:
		tcpm_set_state(dev, SRC_STARTUP, 0);
		break;
	case SRC_HARD_RESET_VBUS_ON:
		tcpm_set_state(dev, SRC_STARTUP, 0);
		break;

	case PORT_RESET:
	case PORT_RESET_WAIT_OFF:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore vbus changes here.
		 */
		break;

	default:
		break;
	}
}

static void _tcpm_pd_vbus_off(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	dev_dbg(dev, "TCPM: VBUS off event\n");
	port->vbus_present = false;
	port->vbus_never_low = false;
	switch (port->state) {
	case SNK_HARD_RESET_SINK_OFF:
		tcpm_set_state(dev, SNK_HARD_RESET_WAIT_VBUS, 0);
		break;
	case HARD_RESET_SEND:
		break;
	case SNK_ATTACH_WAIT:
		tcpm_set_state(dev, SNK_UNATTACHED, 0);
		break;

	case SNK_NEGOTIATE_CAPABILITIES:
		break;

	case PORT_RESET_WAIT_OFF:
		tcpm_set_state(dev, tcpm_default_state(port), 0);
		break;

	case PORT_RESET:
		/*
		 * State set back to default mode once the timer completes.
		 * Ignore vbus changes here.
		 */
		break;

	default:
		if (port->pwr_role == TYPEC_SINK && port->attached)
			tcpm_set_state(dev, SNK_UNATTACHED, 0);
		break;
	}
}

void tcpm_cc_change(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	enum typec_cc_status cc1, cc2;

	tcpm_reset_event_cnt(dev);
	if (drvops->get_cc(dev, &cc1, &cc2) == 0)
		_tcpm_cc_change(dev, cc1, cc2);
}

void tcpm_vbus_change(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	bool vbus;

	tcpm_reset_event_cnt(dev);
	vbus = drvops->get_vbus(dev);
	if (vbus)
		_tcpm_pd_vbus_on(dev);
	else
		_tcpm_pd_vbus_off(dev);
}

void tcpm_pd_hard_reset(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	tcpm_reset_event_cnt(dev);
	dev_dbg(dev, "TCPM: Received hard reset\n");

	/* If a hard reset message is received during the port reset process,
	 * we should ignore it, that is, do not set port->state to HARD_RESET_START.
	 */
	if (port->state == PORT_RESET || port->state == PORT_RESET_WAIT_OFF)
		return;

	/*
	 * If we keep receiving hard reset requests, executing the hard reset
	 * must have failed. Revert to error recovery if that happens.
	 */
	tcpm_set_state(dev,
		       port->hard_reset_count < PD_N_HARD_RESET_COUNT ?
				HARD_RESET_START : ERROR_RECOVERY,
		       0);
}

static void tcpm_init(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	enum typec_cc_status cc1, cc2;

	drvops->init(dev);

	tcpm_reset_port(dev);

	/*
	 * XXX
	 * Should possibly wait for VBUS to settle if it was enabled locally
	 * since tcpm_reset_port() will disable VBUS.
	 */
	port->vbus_present = drvops->get_vbus(dev);
	if (port->vbus_present)
		port->vbus_never_low = true;

	/*
	 * 1. When vbus_present is true, voltage on VBUS is already at VSAFE5V.
	 * So implicitly vbus_vsafe0v = false.
	 *
	 * 2. When vbus_present is false and TCPC does NOT support querying
	 * vsafe0v status, then, it's best to assume vbus is at VSAFE0V i.e.
	 * vbus_vsafe0v is true.
	 *
	 * 3. When vbus_present is false and TCPC does support querying vsafe0v,
	 * then, query tcpc for vsafe0v status.
	 */
	if (port->vbus_present)
		port->vbus_vsafe0v = false;
	else
		port->vbus_vsafe0v = true;

	tcpm_set_state(dev, tcpm_default_state(port), 0);

	if (drvops->get_cc(dev, &cc1, &cc2) == 0)
		_tcpm_cc_change(dev, cc1, cc2);
}

/*
 * Structured Vendor Defined Messages.
 *
 * Linux runs VDMs through an asynchronous state machine because its port
 * manager is interrupt-driven and a VDM exchange must not block the thread
 * that services everything else. U-Boot has the opposite problem: there is
 * no other thread, and tcpm_pd_transmit() already blocks -- it polls the
 * TCPC for tx-complete while running the receive path inline. So a VDM
 * exchange here is "send, then keep pumping that same loop until the reply
 * lands", which is a few dozen lines instead of a few hundred, and has
 * exactly one transaction in flight by construction.
 *
 * What is deliberately NOT implemented: we never respond to a VDM. A partner
 * may send Attention (a DisplayPort HPD change, most often) and it is
 * accepted and dropped, which is legal -- Attention takes no response. A
 * partner that initiates Discover Identity against us gets silence rather
 * than a NAK. That is a real gap, and it is the right one to have first:
 * everything below is about being a DisplayPort SOURCE, which is entirely
 * an initiator role.
 */

/**
 * tcpm_ams_start() - claim the channel for a source-initiated message sequence
 * tcpm_ams_finish() - release it
 *
 * In PD 3.0 the Rp level a source presents IS the collision-avoidance flag:
 * Rp-3.0A means SinkTxOk, the sink may start its own AMS; Rp-1.5A means
 * SinkTxNG, it may not. A source that is about to own the channel lowers the
 * level first and waits tSinkTx for the sink to see it.
 *
 * This is a property of "a source is starting an atomic message sequence",
 * not of any particular message, which is why it lives here rather than at
 * the call sites. It was previously open-coded in SRC_STARTUP and SRC_READY,
 * which covered Source_Capabilities and nothing else -- so every VDM this
 * stack sends went out of SRC_READY with the Rp still saying SinkTxOk while
 * the source was in fact talking. That is exactly the case collision
 * avoidance exists to prevent.
 *
 * Both are no-ops below PD 3.0, and for a sink, which has no Rp to speak
 * with.
 */
static bool tcpm_ams_applies(struct tcpm_port *port)
{
	/*
	 * One predicate for both halves. Below PD 3.0 neither touches the CC
	 * at all: there is no collision avoidance to signal, and the Rp is
	 * tcpm_rp_cc()'s to own as the current advertisement a legacy sink
	 * reads.
	 *
	 * It is evaluated at two different times over mutable state, so a
	 * hard reset or role change between them can still leave the level
	 * where the other half would not have put it. That is why both halves
	 * ASSIGN an absolute level rather than push and pop one.
	 *
	 * @attached is part of the predicate because tcpm_vdm_wait() runs the
	 * state machine for the whole timeout, and the partner can leave
	 * during it. SRC_UNATTACHED and SNK_UNATTACHED set the CC themselves
	 * -- to Rp-default and to Rd respectively -- and neither touches
	 * pwr_role or negotiated_rev, so without this test a VDM retry would
	 * install an Rp on a detached port that the state machine believes is
	 * presenting Rd and looking for a source, and re-attaching as a sink
	 * would stay broken until something drove SNK_UNATTACHED again. When
	 * the partner is gone there is no channel to arbitrate for, and the
	 * unattached states already own the level.
	 */
	return port->attached &&
	       port->pwr_role == TYPEC_SOURCE &&
	       port->negotiated_rev >= PD_REV30;
}

/**
 * tcpm_ams_start() - claim the channel for a source-initiated sequence
 * tcpm_ams_finish() - release it
 *
 * Both are IDEMPOTENT ABSOLUTE ASSIGNMENTS, not a matched push/pop pair, and
 * that is load bearing. A nesting counter was tried and was worse than what
 * it replaced: SRC_STARTUP starts a sequence that only SRC_READY finishes,
 * but the hard-reset loop re-enters SRC_STARTUP without ever passing through
 * SRC_READY -- so the count only ever grew, the Rp latched at SinkTxNG
 * permanently over a 1.4 A rail, and every later VDM no-oped because the
 * count never returned to zero. That is the bug this code exists to prevent,
 * made permanent instead of transient.
 *
 * Assigning instead means an unbalanced call is self-correcting: whichever
 * runs last states the level the port should be presenting right now.
 */
static void tcpm_ams_start(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (!tcpm_ams_applies(port))
		return;

	/* Already claimed -- do not pay tSinkTx again. */
	if (port->cc_req == TYPEC_CC_RP_1_5)
		return;

	tcpm_set_cc(dev, TYPEC_CC_RP_1_5);
	mdelay(PD_T_SINK_TX);
}

static void tcpm_ams_finish(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (!tcpm_ams_applies(port))
		return;

	/*
	 * SinkTxOk only once a contract exists. Before that the Rp is still
	 * the advertisement a legacy sink reads for its current limit, and
	 * claiming 3.0 A to a device that will believe it is not ours to do.
	 *
	 * Without a contract the level to return to is tcpm_rp_cc() -- what
	 * this port advertises for its own rail -- and NOT SinkTxNG. Those
	 * are the same thing only on a 1.5 A source. This board declares
	 * PDO_FIXED(5000, 1400), so tcpm_rp_cc() is Rp-default, and releasing
	 * to Rp-1.5A would latch an over-advertisement permanently: a non-PD
	 * device attaches, SRC_STARTUP claims the channel for the power
	 * negotiation that the device never answers, SRC_READY releases it,
	 * and nothing between here and the next detach lowers it again.
	 */
	tcpm_set_cc(dev, port->explicit_contract ? TYPEC_CC_RP_3_0
						 : tcpm_rp_cc(port));
}

static u32 tcpm_svdm_header(u16 svid, u8 opos, u8 cmd)
{
	/*
	 * SVDM version 1.0 rather than the negotiated version: DPAM 1.0b is
	 * what this SoC's DP path implements, every sink accepts 1.0, and
	 * advertising 2.0 would oblige us to handle the 2.0 response shapes.
	 */
	return VDO(svid, 1, SVDM_VER_1_0, VDO_OPOS(opos) | (cmd));
}

static int tcpm_pd_send_vdm(struct udevice *dev, const u32 *vdo,
			    unsigned int cnt)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	struct pd_message msg;
	unsigned int i;

	if (!cnt || cnt > VDO_MAX_SIZE)
		return -EINVAL;

	memset(&msg, 0, sizeof(msg));
	msg.header = PD_HEADER_LE(PD_DATA_VENDOR_DEF, port->pwr_role,
				  port->data_role, port->negotiated_rev,
				  port->message_id, cnt);

	for (i = 0; i < cnt; i++)
		msg.payload[i] = cpu_to_le32(vdo[i]);

	/*
	 * Discard any parked VDM before the request goes out, so the wait
	 * below cannot match a response to the PREVIOUS command. Doing it
	 * here rather than in the wait is what makes that guarantee hold
	 * even though tcpm_pd_transmit() itself pumps the receive path.
	 *
	 * Arm the receive filter in the same breath, and for the same reason:
	 * tcpm_pd_transmit() pumps the receive path, so the response can be
	 * delivered before this function has even returned.
	 */
	port->vdm_rx_cnt = 0;
	port->vdm_wait_svid = PD_VDO_VID(vdo[0]);
	port->vdm_wait_cmd = PD_VDO_CMD(vdo[0]);
	port->vdm_waiting = true;

	return tcpm_pd_transmit(dev, TCPC_TX_SOP, &msg);
}

static int tcpm_vdm_wait(struct udevice *dev, u16 svid, u8 cmd,
			 unsigned int timeout_ms)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	ulong start = get_timer(0);

	do {
		if (port->vdm_rx_cnt) {
			u32 hdr = port->vdm_rx[0];

			if (PD_VDO_SVDM(hdr) && PD_VDO_VID(hdr) == svid &&
			    PD_VDO_CMD(hdr) == cmd &&
			    PD_VDO_CMDT(hdr) != CMDT_INIT)
				return PD_VDO_CMDT(hdr);

			/*
			 * Somebody else's message -- an Attention, or a
			 * command aimed at us. Drop it and keep waiting; a
			 * VDM that is not the response we asked for must not
			 * end the wait, or a monitor that sends HPD mid-
			 * discovery would abort the discovery.
			 */
			port->vdm_rx_cnt = 0;
		}

		drvops->poll_event(dev);
		udelay(500);
		tcpm_check_and_run_delayed_work(dev);
	} while (get_timer(start) < timeout_ms);

	return -ETIMEDOUT;
}

/**
 * tcpm_vdm_xfer() - send one structured VDM and wait for its response
 *
 * Returns the response's command type (CMDT_RSP_ACK / CMDT_RSP_NAK) on
 * success, or a negative errno. BUSY is retried rather than returned: it
 * means "ask again later", and PD_T_VDM_BUSY is how much later.
 */
static int tcpm_vdm_xfer(struct udevice *dev, const u32 *vdo, unsigned int cnt,
			 unsigned int timeout_ms)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	u16 svid = PD_VDO_VID(vdo[0]);
	u8 cmd = PD_VDO_CMD(vdo[0]);
	int tries, ret;

	for (tries = 0; tries < 3; tries++) {
		/*
		 * Per attempt, not once around the loop: tcpm_vdm_wait() runs
		 * the state machine, which can reach SRC_READY and release the
		 * channel underneath us. Re-asserting is a no-op when the level
		 * is already SinkTxNG, so this costs nothing in the common case
		 * and repairs the one where it matters.
		 */
		tcpm_ams_start(dev);

		ret = tcpm_pd_send_vdm(dev, vdo, cnt);
		if (ret < 0)
			goto out;

		ret = tcpm_vdm_wait(dev, svid, cmd, timeout_ms);
		if (ret != CMDT_RSP_BUSY)
			goto out;

		mdelay(PD_T_VDM_BUSY);
	}

	ret = -EBUSY;
out:
	/*
	 * Disarm, so an Attention arriving outside an exchange is parked as
	 * before rather than silently discarded. Nothing reads it today, but
	 * a hotplug consumer would, and a filter left armed would be a trap
	 * for whoever writes one.
	 */
	port->vdm_waiting = false;
	tcpm_ams_finish(dev);

	return ret;
}

static int tcpm_vdm_ack(struct udevice *dev, const u32 *vdo, unsigned int cnt,
			unsigned int timeout_ms)
{
	int ret = tcpm_vdm_xfer(dev, vdo, cnt, timeout_ms);

	if (ret < 0)
		return ret;
	if (ret != CMDT_RSP_ACK)
		return -EPROTO;

	return 0;
}

/**
 * tcpm_wait_ready() - run the port until it can carry a message
 *
 * The state machine is pumped once, at probe, and then the port idles. That
 * is fine when the partner is already up -- but a display powered BY this
 * board comes up after it, so at probe time there is nothing to attach to,
 * and the port is left in SNK_WAIT_CAPABILITIES with no one to advance it.
 * A consumer then found a port that was permanently not-ready and had no way
 * to ask it to try.
 *
 * So a consumer says what it needs -- a port that can carry a message -- and
 * this runs the machine until that is true or the time is up, rather than
 * reporting the state and giving up.
 */
int tcpm_wait_ready(struct udevice *dev, unsigned int timeout_ms)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	ulong start;
	int ret;

	/*
	 * A consumer's wait starts where the probe-time pump ended, and the
	 * pump may still be running from its tick: finish it first, and
	 * outside this call's budget, since it was never charged to one.
	 */
	tcpm_settle(dev);
	start = get_timer(0);

	/*
	 * Wake FIRST. Polling a sleeping TCPC observes nothing -- its CC
	 * measurement block is off and every interrupt is masked -- so
	 * without this the loop below can only ever advance a state change
	 * that was already armed when the port idled. That happens to cover
	 * the port-stuck-in-SNK_WAIT_CAPABILITIES case, which is why this
	 * appeared to work, but the case the function was written for -- a
	 * display that powers up after the board, leaving nothing attached at
	 * probe -- is exactly the one where the chip is powered down hardest
	 * and no amount of polling can see the attach.
	 */
	ret = tcpm_wake(dev);
	if (ret < 0)
		return ret;

	do {
		if (port->attached &&
		    (port->state == SRC_READY || port->state == SNK_READY))
			goto ready;

		drvops->poll_event(dev);
		udelay(500);
		tcpm_check_and_run_delayed_work(dev);
	} while (get_timer(start) < timeout_ms);

	dev_dbg(dev, "TCPM: not ready after %u ms, state %s\n", timeout_ms,
		tcpm_states[port->state]);

	return -EAGAIN;

ready:
	/*
	 * Reaching SNK_READY is not the same as being able to INITIATE. Under
	 * PD 2.0 -- which is what this monitor negotiates -- structured VDMs
	 * are the DFP's to start, and a port that declared data-role = "host"
	 * attaches as the UFP and acquires the host role from the partner's
	 * DR_Swap. That swap arrives a few messages after the contract, so a
	 * consumer that started discovery the instant SNK_READY appeared would
	 * send Discover Identity as a UFP and be ignored.
	 *
	 * So spend what is left of the caller's own budget waiting for the role
	 * we were told this port can hold -- and then proceed regardless, since
	 * a partner that offers the swap late but does offer it is still worth
	 * talking to, and refusing here would trade a working link for a tidier
	 * state machine.
	 *
	 * ->wait_dr_swap_message is the gate that keeps this from costing time
	 * it cannot buy anything with. It is set from the partner's first
	 * Source_Capabilities -- true only for a DUAL_ROLE|DATA_SWAP source,
	 * i.e. one that can hand the data role over at all -- and cleared again
	 * once the swap completes, on hard reset, and on detach. A plain
	 * charger, or a port that reached SNK_READY without PD at all (the
	 * set_pd_rx failure arm of SNK_WAIT_CAPABILITIES), therefore leaves it
	 * false and skips the wait entirely rather than burning the caller's
	 * whole timeout on a swap that is not coming. tcpm_poll_event() already
	 * uses this exact flag for the same "is a swap still owed to us?"
	 * question, so the two pumps agree on when the port has settled.
	 *
	 * Honesty about the cost that remains: a partner that ADVERTISES
	 * dual-role data and then never sends DR_Swap still burns the full
	 * timeout_ms here -- 960 ms at every call site (PD_T_SINK_WAIT_CAP * 4)
	 * -- where before this wait existed the function returned the instant
	 * SNK_READY appeared, and it compounds across the two or three calls an
	 * alt-mode entry makes. The outcome is unchanged, only the latency. No
	 * cheaper positive signal for "this partner will never swap" exists in
	 * the protocol: an advertised capability the partner declines to
	 * exercise is indistinguishable from one it has not exercised YET, so
	 * anything shorter would be a guess at the partner's timing, and this
	 * comment is not going to pretend otherwise.
	 *
	 * The test is "not UFP-only", NOT "is DFP". A connector declaring
	 * data-role = "dual" can hold the host role every bit as much as one
	 * declaring "host" -- DRD is the SUPERSET -- so an == DFP test drops
	 * this wait for exactly the ports that also need it, and does so
	 * silently: alt-mode entry then starts as a UFP and the partner
	 * ignores the VDMs. Only TYPEC_PORT_DATA_UFP is a real "this port can
	 * never be the host". Nothing is spent on a port that will not swap:
	 * ->wait_dr_swap_message is false unless the partner's own
	 * Source_Capabilities advertised DUAL_ROLE|DATA_SWAP, so widening this
	 * condition costs a DRD port with a plain charger nothing at all.
	 */
	while (port->typec_data != TYPEC_PORT_DATA_UFP &&
	       port->data_role != TYPEC_HOST &&
	       port->wait_dr_swap_message &&
	       get_timer(start) < timeout_ms) {
		drvops->poll_event(dev);
		udelay(500);
		tcpm_check_and_run_delayed_work(dev);

		/*
		 * The loop above is pumping a live state machine, so the port
		 * can go away underneath it: a disconnect, a hard reset, or the
		 * ERROR_RECOVERY -> PORT_RESET path all run from these very
		 * calls, and PORT_RESET clears ->attached while a sink hard
		 * reset puts ->data_role back to TYPEC_DEVICE. Neither of those
		 * ends the loop on its own -- the exit condition only watches
		 * for the role we WANT -- so without this check the wait would
		 * spin out the remaining budget on a dead port and then fall
		 * through to `return 0`, telling the caller it may send
		 * Discover Identity to a partner that is no longer there.
		 *
		 * -EAGAIN is what the timeout arm above already returns for
		 * "not usable, ask again later", and every caller treats any
		 * non-zero as fatal to this attempt, which is correct: the
		 * contract this function promises is a port that can carry a
		 * message right now.
		 */
		if (!port->attached ||
		    (port->state != SRC_READY && port->state != SNK_READY)) {
			dev_dbg(dev, "TCPM: left ready while awaiting DR_Swap, state %s attached %d\n",
				tcpm_states[port->state], port->attached);
			return -EAGAIN;
		}
	}

	/*
	 * Say WHICH of the two no-swap cases this was: one of them cost the
	 * caller its whole timeout and the other cost nothing, and a log line
	 * that cannot tell them apart is the one you want when alt-mode entry
	 * is slow rather than broken.
	 *
	 * Same "not UFP-only" test as the wait above, and for the same reason:
	 * a diagnostic that goes quiet for exactly the port shape the wait was
	 * just widened to cover would report a link as fine while it came up
	 * as a UFP.
	 */
	if (port->typec_data != TYPEC_PORT_DATA_UFP &&
	    port->data_role != TYPEC_HOST)
		dev_dbg(dev, "TCPM: proceeding as UFP -- partner %s\n",
			port->wait_dr_swap_message ?
			"advertised a data swap but did not send one" :
			"cannot swap the data role");

	return 0;
}

/**
 * tcpm_discover_svids() - run generic alt-mode discovery on the partner
 *
 * Discover Identity for the Modal Operation bit, then Discover SVIDs. Leaves
 * the partner's SVID list in the port, where tcpm_get_partner_svids() and
 * tcpm_discover_modes() can reach it.
 *
 * Nothing here knows what any SVID means. That is the point: alt modes are
 * consumers of this, not part of it.
 */
int tcpm_discover_svids(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	u32 vdo[VDO_MAX_SIZE];
	unsigned int i, half;
	bool done = false;
	int ret, round;

	port->nr_partner_svids = 0;

	ret = tcpm_wait_ready(dev, PD_T_SINK_WAIT_CAP * 4);
	if (ret)
		return ret;

	/*
	 * Discover Identity is not asked for its identity. It is asked for
	 * one bit: Modal Operation Supported in the ID header. A partner
	 * without it has no alt modes to discover and asking further is a
	 * protocol error, not merely futile.
	 */
	vdo[0] = tcpm_svdm_header(USB_SID_PD, 0, CMD_DISCOVER_IDENT);
	ret = tcpm_vdm_ack(dev, vdo, 1, PD_T_VDM_SNDR_RSP * 10);
	if (ret)
		return ret;

	if (port->vdm_rx_cnt < 2 || !PD_IDH_MODAL_SUPP(port->vdm_rx[1])) {
		dev_dbg(dev, "TCPM: partner does not support modal operation\n");
		return -EOPNOTSUPP;
	}

	/*
	 * Discover SVIDs, repeated while the response could have more behind
	 * it. Two SVIDs per VDO, terminated by a zero -- but a response that
	 * is not completely full also ends the list, because partners exist
	 * that stop short of 12 SVIDs without writing the terminator, and
	 * looping purely on "no zero seen" hangs against them.
	 */
	for (round = 0; round < 4 && !done; round++) {
		vdo[0] = tcpm_svdm_header(USB_SID_PD, 0, CMD_DISCOVER_SVID);
		ret = tcpm_vdm_ack(dev, vdo, 1, PD_T_VDM_SNDR_RSP * 10);
		if (ret)
			return ret;

		for (i = 1; i < port->vdm_rx_cnt && !done; i++) {
			u16 svid[2];

			svid[0] = PD_VDO_SVID_SVID0(port->vdm_rx[i]);
			svid[1] = PD_VDO_SVID_SVID1(port->vdm_rx[i]);

			for (half = 0; half < 2; half++) {
				if (!svid[half] ||
				    port->nr_partner_svids >= SVID_DISCOVERY_MAX) {
					done = true;
					break;
				}

				port->partner_svids[port->nr_partner_svids++] =
					svid[half];
			}
		}

		if (port->vdm_rx_cnt != VDO_MAX_SIZE)
			break;
	}

	return 0;
}

/**
 * tcpm_discover_modes() - Discover Modes for one SVID
 * @modes: filled with the mode VDOs, in response order
 * @max: capacity of @modes
 *
 * Returns the number of modes discovered, or a negative errno.
 *
 * The INDEX of a mode in @modes, one-based, is its object position -- that is
 * what Enter Mode has to carry, and it is not a constant. A partner is free to
 * return the mode a caller wants second.
 */
int tcpm_discover_modes(struct udevice *dev, u16 svid, u32 *modes,
			unsigned int max)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	u32 vdo[VDO_MAX_SIZE];
	unsigned int i, n;
	int ret;

	ret = tcpm_wait_ready(dev, PD_T_SINK_WAIT_CAP * 4);
	if (ret)
		return ret;

	vdo[0] = tcpm_svdm_header(svid, 0, CMD_DISCOVER_MODES);
	ret = tcpm_vdm_ack(dev, vdo, 1, PD_T_VDM_SNDR_RSP * 10);
	if (ret)
		return ret;

	if (port->vdm_rx_cnt < 2)
		return -ENODEV;

	n = min(max, port->vdm_rx_cnt - 1);
	for (i = 0; i < n; i++)
		modes[i] = port->vdm_rx[i + 1];

	return n;
}

int tcpm_vdm_exchange(struct udevice *dev, const u32 *vdo, unsigned int cnt,
		      u32 *rx, unsigned int max, unsigned int *rx_cnt,
		      unsigned int timeout_ms)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	unsigned int i, n;
	int ret;

	if (rx_cnt)
		*rx_cnt = 0;

	ret = tcpm_vdm_xfer(dev, vdo, cnt, timeout_ms);
	if (ret < 0)
		return ret;

	/*
	 * Report how many objects the caller may READ, which is how many were
	 * copied -- not how many arrived on the wire. A caller reading rx[1]
	 * has to know it exists: a partner may ACK with a header and no data
	 * object, and without a count the caller reads whatever was on its
	 * stack and feeds it into the next message it sends. Reporting the
	 * wire count when the buffer was smaller would recreate that same
	 * defect one step further along.
	 */
	if (rx && max) {
		n = min(max, port->vdm_rx_cnt);
		for (i = 0; i < n; i++)
			rx[i] = port->vdm_rx[i];

		if (rx_cnt)
			*rx_cnt = n;
	}

	return ret;
}

u32 tcpm_svdm_build_header(u16 svid, u8 opos, u8 cmd)
{
	return tcpm_svdm_header(svid, opos, cmd);
}

struct typec_dp_state *tcpm_dp_state(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	tcpm_settle(dev);

	return &port->dp;
}

int tcpm_get_connector(struct udevice *dev, ofnode *node)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);

	if (!drvops || !drvops->get_connector_node)
		return -ENOSYS;

	return drvops->get_connector_node(dev, node);
}


unsigned int tcpm_get_partner_svids(struct udevice *dev, u16 *svids,
				    unsigned int max)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	unsigned int i, n;

	tcpm_settle(dev);
	n = min(max, port->nr_partner_svids);

	for (i = 0; i < n; i++)
		svids[i] = port->partner_svids[i];

	return n;
}

static int tcpm_fw_get_caps(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	ofnode node;
	const char *cap_str;
	int ret;
	u32 mw;

	ret = drvops->get_connector_node(dev, &node);
	if (ret)
		return ret;

	/*
	 * The DATA role capability, which is not the power role. A port can be
	 * a power sink and a data host at once -- that is what a laptop
	 * driving a self-powered monitor is, and what DisplayPort alt mode
	 * needs from us. Without this the sink attach below hardcoded UFP, so
	 * a connector declaring data-role = "host" still came up as a device
	 * and could never be the DisplayPort source.
	 *
	 * Absent property keeps the old assumption rather than failing: a
	 * board that never declared one was getting DFP-when-source and
	 * UFP-when-sink, which is what "dual" means.
	 */
	port->typec_data = TYPEC_PORT_DATA_DRD;
	cap_str = ofnode_read_string(node, "data-role");
	if (cap_str) {
		if (!strcmp("host", cap_str))
			port->typec_data = TYPEC_PORT_DATA_DFP;
		else if (!strcmp("device", cap_str))
			port->typec_data = TYPEC_PORT_DATA_UFP;
		else if (strcmp("dual", cap_str))
			return -EINVAL;
	}

	cap_str = ofnode_read_string(node, "power-role");
	if (!cap_str)
		return -EINVAL;

	if (!strcmp("dual", cap_str))
		port->typec_type = TYPEC_PORT_DRP;
	else if (!strcmp("source", cap_str))
		port->typec_type = TYPEC_PORT_SRC;
	else if (!strcmp("sink", cap_str))
		port->typec_type = TYPEC_PORT_SNK;
	else
		return -EINVAL;

	port->port_type = port->typec_type;

	if (port->port_type == TYPEC_PORT_SNK)
		goto sink;

	/* Get source pdos */
	ret = ofnode_read_size(node, "source-pdos") / sizeof(u32);
	if (ret <= 0)
		return -EINVAL;

	port->nr_src_pdo = min(ret, PDO_MAX_OBJECTS);
	ret = ofnode_read_u32_array(node, "source-pdos",
				    port->src_pdo, port->nr_src_pdo);
	if (ret || tcpm_validate_caps(dev, port->src_pdo, port->nr_src_pdo))
		return -EINVAL;

	if (port->port_type == TYPEC_PORT_SRC)
		return 0;

	/* Get the preferred power role for DRP */
	cap_str = ofnode_read_string(node, "try-power-role");
	if (!cap_str)
		return -EINVAL;

	if (!strcmp("sink", cap_str))
		port->typec_prefer_role = TYPEC_SINK;
	else if (!strcmp("source", cap_str))
		port->typec_prefer_role = TYPEC_SOURCE;
	else
		return -EINVAL;

	if (port->typec_prefer_role < 0)
		return -EINVAL;
sink:
	/* Get sink pdos */
	ret = ofnode_read_size(node, "sink-pdos") / sizeof(u32);
	if (ret <= 0)
		return -EINVAL;

	port->nr_snk_pdo = min(ret, PDO_MAX_OBJECTS);
	ret = ofnode_read_u32_array(node, "sink-pdos",
				    port->snk_pdo, port->nr_snk_pdo);
	if (ret || tcpm_validate_caps(dev, port->snk_pdo, port->nr_snk_pdo))
		return -EINVAL;

	if (ofnode_read_u32_array(node, "op-sink-microwatt", &mw, 1))
		return -EINVAL;
	port->operating_snk_mw = mw / 1000;

	port->self_powered = ofnode_read_bool(node, "self-powered");

	return 0;
}

static int tcpm_port_init(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int err;

	err = tcpm_fw_get_caps(dev);
	if (err < 0) {
		dev_err(dev, "TCPM: please check the dts config: %d\n", err);
		return err;
	}

	port->try_role = port->typec_prefer_role;
	port->port_type = port->typec_type;

	tcpm_init(dev);

	dev_info(dev, "TCPM: init finished\n");

	return 0;
}

/**
 * tcpm_enter_dp_altmode() - enter DisplayPort alt mode, on request
 *
 * A port whose connector node carries altmodes/displayport is a DisplayPort
 * source receptacle; the board said so, and there is nowhere else for it to say
 * so. For a long time the only thing that acted on that declaration was the
 * `tcpm dp enter` command, which means a board could describe a DisplayPort port
 * completely and correctly and still come up with a blank monitor unless a
 * human typed a command -- so the display worked on the bench and never at
 * boot, which is the one place it has to work.
 *
 * Entering is the port manager's business for the same reason the power
 * contract is: it is negotiated over the connector, it must happen while the
 * transceiver is awake, and every consumer downstream (the Type-C PHY's lane
 * mux, then cdn-dp) can only READ the result. A consumer that sent Enter Mode
 * on its own behalf would be a second entity driving the partner's mode state
 * with no way to agree with this one about who is in it.
 *
 * 🔴 But it must follow INTENT, and the first version of this did not: it ran
 * unconditionally from tcpm_poll_event(), i.e. on EVERY probe of the port. The
 * USB3 PHY reaches a probe -- rockchip_usb3_phy_power_on() ->
 * rockchip_tcphy_set_mode() -> rockchip_tcphy_get_orientation(), which has
 * always probed -- so `usb start` alone would send Enter Mode to a USB-C dock
 * and configure pin assignment C, four DisplayPort lanes with no SuperSpeed,
 * while that same call finished configuring the PHY for USB3. Nothing exits the
 * mode afterwards; on this hardware only a physical unplug recovers it. So the
 * declaration says the port CAN do DisplayPort, and a caller asking says it
 * SHOULD -- the two are not the same and only the second belongs here.
 *
 * Failure is a warning to the caller, not a probe failure. A partner that
 * refuses DisplayPort, or a cable carrying only USB, is an ordinary thing to
 * find on a Type-C port; it must not take the port manager -- and with it the
 * power contract -- down with it.
 *
 * Return: as documented in include/usb/tcpm.h.
 */
int tcpm_enter_dp_altmode(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret;

	/*
	 * Enter Mode is the DFP's to send, so a port that can never hold the host
	 * data role can never carry DisplayPort, whatever its connector declares.
	 * A sink-only port is one: it attaches as the device, and this stack
	 * rejects a DR_Swap on any port that is not dual-power-role (the
	 * PD_CTRL_DR_SWAP case above). Asking anyway is not harmless: it starts
	 * PD traffic and alt-mode discovery on the link, and on ROC-RK3399-PC
	 * that link is the board's power inlet -- Type-C0, the same port its
	 * USB gadget talks to the host on.
	 */
	if (port->port_type == TYPEC_PORT_SNK && port->data_role != TYPEC_HOST) {
		dev_dbg(dev, "TCPM: sink-only port, never the host: no DisplayPort\n");
		return -ENOENT;
	}

	/*
	 * There is deliberately NO "is anything attached?" gate here, and the
	 * first version of this function had one -- `if (!drvops->get_vbus(dev))
	 * return -ENODEV;` -- which was wrong twice over.
	 *
	 * It read a STALE value. fusb302_get_vbus() returns chip->vbus_present,
	 * a shadow written only in the chip's init and in its poll_event/IRQ
	 * handler. Nothing refreshes it between the port's probe and a request
	 * arriving here, and by then enter_low_power_mode() has masked every
	 * interrupt and dropped power mode -- so the shadow is stale AND the
	 * chip is asleep, which is the state in which it observes nothing.
	 *
	 * And it short-circuited ahead of the machinery written for exactly
	 * this case. Returning here skips typec_dp_autoenter() entirely, so
	 * tcpm_wait_ready() -- and the tcpm_wake() inside it -- never run, even
	 * though tcpm_wake()'s own contract says it is needed by anything that
	 * touches the wire OR waits on the chip to observe something. On a
	 * board whose display is powered BY the board, the panel attaches after
	 * U-Boot does, and this gate turned that into a hard -ENODEV.
	 *
	 * That gate was, precisely, a second worse answer to a question
	 * tcpm_wait_ready() owns -- the same mistake the readiness pre-check
	 * described below was removed for. Leaving the question to that
	 * function costs a wait when nothing is attached and is correct when
	 * something attaches late; the reverse trade is not worth making on a
	 * board that always attaches late.
	 */

	/*
	 * No "is the port ready?" test here, deliberately, and the first
	 * version of this function had one -- it returned unless the port was
	 * attached and in SNK_READY/SRC_READY, and that made boot-time entry
	 * fail on this board every time. Measured: the port is still cycling
	 * through ERROR_RECOVERY with the monitor when the poll loop above
	 * gives up its 1 s budget, so the guard read "not ready" and gave up on
	 * a port that settles about a second later.
	 *
	 * tcpm_wait_ready() -- which every VDM exchange underneath
	 * typec_dp_enter() already calls -- exists precisely for this, and its
	 * own comment names this board's case: "a display powered BY this board
	 * comes up after it, so at probe time there is nothing to attach to".
	 * It wakes the transceiver and pumps the machine until the port can
	 * carry a message. A pre-check here is a second, worse answer to a
	 * question that function owns.
	 *
	 * There is no surviving gate to point at: the sentence that used to sit
	 * here named "the get_vbus() check above", which the comment twenty
	 * lines up explains was deleted for the same reason this one was never
	 * added. Two comments in one function disagreeing about whether a gate
	 * exists is worse than either answer.
	 */
	ret = typec_dp_autoenter(dev);

	/* The board declared no DisplayPort here. Not a failure, not news. */
	if (ret == -ENOENT)
		return ret;

	/* Already in the mode before we asked. Success, but nothing happened. */
	if (ret == -EALREADY)
		return 0;

	if (ret)
		dev_warn(dev, "TCPM: DisplayPort alt mode not entered: %d\n",
			 ret);
	else
		dev_info(dev, "TCPM: entered DisplayPort alt mode\n");

	return ret;
}

/*
 * The probe-time pump, in three pieces so that it can run either inline
 * (the loop every consumer used to pay for at probe) or one step at a time
 * from a cyclic tick while the rest of the boot goes on.
 */

/* Has the pump reached the state the synchronous loop stopped at? */
static bool tcpm_poll_event_settled(struct tcpm_port *port)
{
	if (!port->wait_dr_swap_message &&
	    (port->state == SNK_READY || port->state == SRC_READY))
		return true;

	return time_after_eq64((u64)timer_get_us(), port->poll_event_deadline);
}

/* One turn of the pump: read the chip's events, run any timer that expired */
static void tcpm_poll_event_step(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);

	drvops->poll_event(dev);
	tcpm_check_and_run_delayed_work(dev);
}

/* What the pump does once it has settled: report, then let the chip sleep */
static void tcpm_poll_event_finish(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (port->state != SNK_READY && port->state != SRC_READY)
		dev_warn(dev, "TCPM: exit in state %s\n",
			 tcpm_states[port->state]);

	/*
	 * Alt-mode entry deliberately does NOT happen here.
	 *
	 * It did, and it made `usb start` drive USB-C partners into
	 * DisplayPort -- see tcpm_enter_dp_altmode(). A probe is not a request:
	 * this function runs for whoever probed the port, and most of them want
	 * USB. The DisplayPort consumer asks for the mode itself, which is also
	 * the only point at which "somebody wants a display" is actually known.
	 *
	 * The cost of moving it is that a later request arrives after the
	 * low-power block below has masked the transceiver's interrupts and
	 * stopped its oscillator. That is survivable and already handled:
	 * every VDM exchange underneath typec_dp_enter() calls
	 * tcpm_wait_ready(), which calls tcpm_wake() first.
	 */

	/*
	 * At this time, call the callback function of the respective pd chip
	 * to enter the low-power mode. In order to reduce the time spent on
	 * the PD chip driver as much as possible, the tcpm framework does not
	 * fully process the communication initiated by the device,so it should
	 * be noted that we can disable the internal oscillator, etc., but do
	 * not turn off the power of the transceiver module, otherwise the
	 * self-powered Type-C device will initiate a Message(eg: self-powered
	 * Type-C hub initiates a SINK capability request(PD_CTRL_GET_SINK_CAP))
	 * and the pd chip cannot reply to GoodCRC, causing the self-powered Type-C
	 * device to switch vbus to vSafe5v, or even turn off vbus.
	 */
	if (!drvops->enter_low_power_mode)
		return;

	/*
	 * Set BEFORE the call, and leave it set on failure. The FUSB302 masks
	 * every interrupt first and only then changes power mode, so a
	 * half-completed sleep leaves a chip that reports nothing while the
	 * port believes it is awake -- which reproduces the very symptom the
	 * wake exists to cure, with no path back. "May be asleep" is the
	 * honest state after an attempt, whatever the attempt returned.
	 */
	port->low_power = true;

	if (drvops->enter_low_power_mode(dev, port->attached, port->pd_capable))
		dev_err(dev, "TCPM: failed to enter low power\n");
	else
		dev_info(dev, "TCPM: PD chip enter low power mode\n");
}

/* The pump run inline, to its end: the loop the probe used to block on */
static void tcpm_poll_event(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	while (!tcpm_poll_event_settled(port)) {
		tcpm_poll_event_step(dev);
		udelay(500);
	}

	tcpm_poll_event_finish(dev);
}

/*
 * How often the cyclic tick steps the pump. The inline loop polled every
 * ~1 ms (a 500 us sleep plus the poll itself); a tick every 4 ms keeps the
 * port's own reaction time well inside the PD protocol's tightest budget
 * for a sink (tSenderResponse, 24-30 ms: the Request that must answer a
 * Source_Capabilities) while costing the boot a few percent of one core
 * instead of all of it, since one poll is a few I2C register reads.
 */
#define TCPM_PROBE_PUMP_PERIOD_US	4000

static void tcpm_probe_pump_tick(struct cyclic_info *c)
{
	struct tcpm_port *port = container_of(c, struct tcpm_port, cyclic);
	struct udevice *dev = port->dev;

	if (!tcpm_poll_event_settled(port)) {
		/*
		 * A tick lands wherever the boot last called udelay(), and
		 * that includes the inside of another device's transfer on
		 * this port's I2C bus (the bus driver polls its controller
		 * with udelay()). Stepping the pump there would put a second
		 * transaction into the middle of the first. Step aside; the
		 * next tick is 4 ms away and the state machine keeps wall
		 * time, so nothing is lost but the one poll.
		 */
		if (device_get_uclass_id(dev->parent) == UCLASS_I2C &&
		    dm_i2c_xfer_in_progress(dev->parent))
			return;

		tcpm_poll_event_step(dev);
		if (!tcpm_poll_event_settled(port))
			return;
	}

	cyclic_unregister(c);
	port->probe_pump_pending = false;
	tcpm_poll_event_finish(dev);
}

void tcpm_settle(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (!port->probe_pump_pending)
		return;

	cyclic_unregister(&port->cyclic);
	port->probe_pump_pending = false;
	tcpm_poll_event(dev);
}

int tcpm_pre_remove(struct udevice *dev)
{
	struct tcpm_port *port = dev_get_uclass_plat(dev);

	if (port->probe_pump_pending) {
		cyclic_unregister(&port->cyclic);
		port->probe_pump_pending = false;
	}

	return 0;
}

int tcpm_post_probe(struct udevice *dev)
{
	const struct dm_tcpm_ops *drvops = dev_get_driver_ops(dev);
	struct tcpm_port *port = dev_get_uclass_plat(dev);
	int ret = tcpm_port_init(dev);

	if (ret < 0) {
		dev_err(dev, "failed to tcpm port init\n");
		return ret;
	}

	port->dev = dev;

	/*
	 * Nothing to pump on a port with no VBUS: a source-capable port
	 * whose partner has not powered anything yet waits for a consumer
	 * to ask (tcpm_wait_ready()), and a sink port with no supply has no
	 * partner at all. Exactly the early return the inline pump had.
	 */
	if (!drvops->get_vbus(dev))
		return 0;

	tcpm_reset_event_cnt(dev);

	/*
	 * The pump runs at probe for a reason tcpm_post_bind() spells out: a
	 * board powered through this port must talk PD inside the source's
	 * window or lose its supply to a hard reset later. But the state
	 * machine is a series of wall-clock waits -- tCCDebounce, then
	 * tSinkWaitCap, and on a supply that speaks no PD at all the whole
	 * soft-reset / hard-reset walk to SNK_READY-without-a-contract, two
	 * seconds of it -- and the inline loop made the entire boot stand
	 * still for them. So the negotiation starts here and now, as before,
	 * but it is stepped from a cyclic tick while initr_env, the video
	 * walk and everything after them run. Whoever needs the port's
	 * settled state (the Type-C PHY for its orientation, cdn-dp for alt
	 * mode, `tcpm info`) settles it first, inline, and sees what it
	 * always saw; a boot that never asks pays nothing.
	 *
	 * Without CYCLIC there is no tick to step from, so the loop runs
	 * inline as it always did.
	 */
	if (!CONFIG_IS_ENABLED(CYCLIC)) {
		tcpm_poll_event(dev);
		return 0;
	}

	port->probe_pump_pending = true;
	cyclic_register(&port->cyclic, tcpm_probe_pump_tick,
			TCPM_PROBE_PUMP_PERIOD_US, dev->name);

	return 0;
}
