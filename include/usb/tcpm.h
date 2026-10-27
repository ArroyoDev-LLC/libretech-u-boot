/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2017 Google, Inc
 * Copyright 2024 Collabora
 */

#ifndef __LINUX_USB_TCPM_H
#define __LINUX_USB_TCPM_H

#include <dm/of.h>
#include <linux/bitops.h>
#include "pd.h"

enum typec_orientation {
	TYPEC_ORIENTATION_NONE,
	TYPEC_ORIENTATION_NORMAL,
	TYPEC_ORIENTATION_REVERSE,
};

enum typec_cc_status {
	TYPEC_CC_OPEN,
	TYPEC_CC_RA,
	TYPEC_CC_RD,
	TYPEC_CC_RP_DEF,
	TYPEC_CC_RP_1_5,
	TYPEC_CC_RP_3_0,
};

enum typec_cc_polarity {
	TYPEC_POLARITY_CC1,
	TYPEC_POLARITY_CC2,
};

enum tcpm_transmit_status {
	TCPC_TX_SUCCESS = 0,
	TCPC_TX_DISCARDED = 1,
	TCPC_TX_FAILED = 2,
};

enum tcpm_transmit_type {
	TCPC_TX_SOP = 0,
	TCPC_TX_SOP_PRIME = 1,
	TCPC_TX_SOP_PRIME_PRIME = 2,
	TCPC_TX_SOP_DEBUG_PRIME = 3,
	TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
	TCPC_TX_HARD_RESET = 5,
	TCPC_TX_CABLE_RESET = 6,
	TCPC_TX_BIST_MODE_2 = 7
};

struct dm_tcpm_ops {
	int (*get_connector_node)(struct udevice *dev, ofnode *connector_node);
	int (*init)(struct udevice *dev);
	int (*get_vbus)(struct udevice *dev);
	int (*set_cc)(struct udevice *dev, enum typec_cc_status cc);
	int (*get_cc)(struct udevice *dev, enum typec_cc_status *cc1,
		      enum typec_cc_status *cc2);
	int (*set_polarity)(struct udevice *dev,
			    enum typec_cc_polarity polarity);
	int (*set_vconn)(struct udevice *dev, bool on);
	int (*set_vbus)(struct udevice *dev, bool on, bool charge);
	int (*set_pd_rx)(struct udevice *dev, bool on);
	int (*set_roles)(struct udevice *dev, bool attached,
			 enum typec_role role, enum typec_data_role data);
	int (*start_toggling)(struct udevice *dev,
			      enum typec_port_type port_type,
			      enum typec_cc_status cc);
	int (*pd_transmit)(struct udevice *dev, enum tcpm_transmit_type type,
			   const struct pd_message *msg, unsigned int negotiated_rev);
	void (*poll_event)(struct udevice *dev);
	int (*enter_low_power_mode)(struct udevice *dev, bool attached, bool pd_capable);
	/**
	 * @exit_low_power_mode: undo enter_low_power_mode()
	 *
	 * Required of any TCPC whose low-power mode disables the transmitter
	 * or its interrupt reporting. The port manager calls this before it
	 * transmits anything after the port has gone idle -- which it does
	 * whenever a consumer initiates an exchange, rather than only during
	 * the probe-time poll loop the low-power call was written for.
	 */
	int (*exit_low_power_mode)(struct udevice *dev);
	/**
	 * @event_pending: optional; whether poll_event() has anything to do
	 *
	 * The port manager keeps polling an attached port from a cyclic
	 * function, and a poll is several bus transfers. A TCPC that can tell
	 * cheaply that nothing happened -- its interrupt line is idle and no
	 * deferred work is due -- returns false here and the poll is skipped.
	 * Without this op every tick polls.
	 */
	bool (*event_pending)(struct udevice *dev);
	/**
	 * @get_prior_roles: optional; the roles a previous session left in the
	 * TCPC, read before init() reset it
	 *
	 * A warm reset of the SoC does not reset the TCPC or the partner. A PD
	 * contract and any data-role swap survive it, and the partner then
	 * answers a message carrying the default data role with Type-C error
	 * recovery: VBUS goes away, which on a power inlet is a power-on
	 * reset. Return -ENOENT when the TCPC held no PD session.
	 */
	int (*get_prior_roles)(struct udevice *dev, enum typec_role *role,
			       enum typec_data_role *data);
};

/* API for drivers */
void tcpm_vbus_change(struct udevice *dev);
void tcpm_cc_change(struct udevice *dev);
void tcpm_pd_receive(struct udevice *dev, const struct pd_message *msg);
void tcpm_pd_transmit_complete(struct udevice *dev,
			       enum tcpm_transmit_status status);
void tcpm_pd_hard_reset(struct udevice *dev);

/* API for boards */
extern const char * const typec_pd_rev_name[];
extern const char * const typec_orientation_name[];
extern const char * const typec_role_name[];
extern const char * const typec_data_role_name[];
extern const char * const typec_cc_status_name[];

int tcpm_get(int index, struct udevice **devp);
int tcpm_get_pd_rev(struct udevice *dev);
int tcpm_get_current(struct udevice *dev);
int tcpm_get_voltage(struct udevice *dev);
enum typec_orientation tcpm_get_orientation(struct udevice *dev);
enum typec_role tcpm_get_pwr_role(struct udevice *dev);
enum typec_data_role tcpm_get_data_role(struct udevice *dev);
/**
 * tcpm_get_source_caps() - the partner's last Source_Capabilities
 * @dev: TCPM device
 * @caps: set to the PDOs; valid until the next message on the port
 * @explicit_contract: set to whether a PD contract is in force
 *
 * Return: the number of PDOs, 0 when the partner sent none.
 */
unsigned int tcpm_get_source_caps(struct udevice *dev, const u32 **caps,
				  bool *explicit_contract);
bool tcpm_is_connected(struct udevice *dev);
const char *tcpm_get_state(struct udevice *dev);

/**
 * fusb302_cc_orientation() - plug orientation from one FUSB302 CC measurement
 * @chip: the FUSB302's I2C chip device
 *
 * For a phase without the port manager (CONFIG_SPL_TYPEC_FUSB302_CC).
 *
 * Return: an enum typec_orientation (TYPEC_ORIENTATION_NONE when neither or
 * both CC lines carry a pull-up), or -errno from the I2C access
 */
int fusb302_cc_orientation(struct udevice *chip);

/*
 * Alt-mode support. The port manager provides VDM transport and generic
 * discovery; it knows nothing about what any SVID means. An alt mode is a
 * consumer of these -- see usb/typec_dp.h for DisplayPort.
 */

/* How many partner SVIDs are worth keeping. Not a protocol limit. */
#define SVID_DISCOVERY_MAX	16

/**
 * tcpm_wait_ready() - run the port until it can carry a message
 *
 * The state machine is pumped at probe and the port then idles, so a partner
 * that comes up later -- a display powered by this board does -- would leave
 * the port permanently not-ready with nothing to advance it. The discovery
 * calls below do this themselves; it is exposed for a consumer that wants to
 * wait before doing something else.
 *
 * On a port that declared data-role = "host" this also waits out the partner's
 * DR_Swap before returning, because a UFP cannot initiate the structured VDMs
 * every caller below is about to send. That wait is bounded by the same
 * @timeout_ms and is skipped entirely when the partner cannot swap.
 *
 * Return: 0 once attached and in a ready state, -EAGAIN on timeout or if the
 * port left that ready state again while the swap was being waited for -- a 0
 * return is a promise that the port can carry a message NOW, so a disconnect or
 * error recovery mid-wait must not be reported as success.
 */
int tcpm_wait_ready(struct udevice *dev, unsigned int timeout_ms);

enum tcpm_attach_role {
	TCPM_ATTACH_DRP,	/* whatever the port's connector allows */
	TCPM_ATTACH_SINK,
	TCPM_ATTACH_SOURCE,
};

/**
 * tcpm_force_attach() - run a fresh Type-C attach and pump it to the end
 *
 * Wakes the TCPC, then restarts the port's attach: through ErrorRecovery (CC
 * open, so the partner sees a detach) on a port that does not power the
 * board, or by restarting the sink attach with Rd held on the power inlet
 * (sink-only, not self-powered), which must never drop CC. The state machine
 * is then stepped continuously -- not left to the probe-time pump or to
 * interrupts that may be masked -- until the port is ready and, as a sink of a
 * swap-capable partner, holds or has stopped waiting for the host role. Every
 * state change is printed with a timestamp.
 *
 * @role forces the attach role for this attempt only (the connector's
 * capability is restored afterwards); it must be one the connector allows.
 *
 * Return: 0 attached and ready, -ETIMEDOUT not within @timeout_ms,
 * -EOPNOTSUPP a role the connector cannot take, or tcpm_wake()'s error.
 */
int tcpm_force_attach(struct udevice *dev, enum tcpm_attach_role role,
		      unsigned int timeout_ms);

/**
 * tcpm_request_host_role() - send DR_Swap when still the UFP
 *
 * Either end may ask for a data-role swap (USB PD 6.3.9). A port that waits
 * for the partner to offer it stays the UFP for the whole boot when that
 * offer is lost -- a sink hard reset or port reset clears the wait, and a
 * partner need not offer again -- and a UFP cannot start the structured VDMs
 * alt-mode discovery needs.
 *
 * Asks only from SNK_READY with an explicit contract, on a connector that may
 * be the host, to a partner whose Source_Capabilities advertise DATA_SWAP;
 * otherwise nothing goes on the wire. Runs the state machine until Accept,
 * Reject, Wait, Not_Supported or tSenderResponse ends the exchange.
 *
 * A successful swap clears the port's DisplayPort state, so call this BEFORE
 * discovery reads anything into it, never from inside an exchange.
 * A failure is only reported; the caller proceeds as UFP.
 */
void tcpm_request_host_role(struct udevice *dev);

/**
 * tcpm_discover_svids() - run generic alt-mode discovery on the partner
 *
 * Discover Identity for the Modal Operation bit, then Discover SVIDs.
 *
 * Return: 0 on success, -EOPNOTSUPP if the partner does not support modal
 * operation, -EAGAIN if the port is not in a state where VDMs may be sent.
 */
int tcpm_discover_svids(struct udevice *dev);

/**
 * tcpm_get_partner_svids() - copy out the discovered SVIDs
 *
 * Return: how many were copied, at most @max.
 */
unsigned int tcpm_get_partner_svids(struct udevice *dev, u16 *svids,
				    unsigned int max);

/**
 * tcpm_discover_modes() - Discover Modes for one SVID
 * @modes: filled with the mode VDOs, in response order
 * @max: capacity of @modes
 *
 * The one-based INDEX of a mode in @modes is its object position, which is
 * what Enter Mode must carry. It is not a constant.
 *
 * Return: how many modes were discovered, or a negative errno.
 */
int tcpm_discover_modes(struct udevice *dev, u16 svid, u32 *modes,
			unsigned int max);

/**
 * tcpm_svdm_build_header() - build a structured VDM header
 */
u32 tcpm_svdm_build_header(u16 svid, u8 opos, u8 cmd);

/**
 * tcpm_vdm_exchange() - send one structured VDM and collect the response
 * @rx: receives the response objects including its header; may be NULL
 * @max: capacity of @rx
 *
 * Handles the PD 3.0 collision-avoidance handshake around the exchange, so a
 * caller never has to think about the Rp level.
 *
 * Return: the response's command type (CMDT_RSP_ACK / CMDT_RSP_NAK), or a
 * negative errno.
 */
int tcpm_vdm_exchange(struct udevice *dev, const u32 *vdo, unsigned int cnt,
		      u32 *rx, unsigned int max, unsigned int *rx_cnt,
		      unsigned int timeout_ms);

/**
 * tcpm_get_connector() - the port's connector node
 *
 * So a consumer can read its own properties out of the device tree -- an alt
 * mode's capability describes the board, and belongs to the alt mode rather
 * than to the port manager.
 */
int tcpm_get_connector(struct udevice *dev, ofnode *node);

struct typec_dp_state;

/**
 * tcpm_dp_state() - the port's DisplayPort alt-mode state
 *
 * One per port, owned by the port, never NULL. This is the single answer to
 * "is DisplayPort active on this connector, and with which pin assignment",
 * and it is how the negotiated link reaches the Type-C PHY: the PHY finds its
 * own TCPM through the connector graph and reads it here, at its own probe,
 * long after the command that negotiated it has returned.
 *
 * A caller may write through this pointer -- typec_dp_enter() does -- but the
 * port owns invalidation. Do not cache the CONTENTS across anything that can
 * detach, hard-reset or data-role-swap the port; cache the pointer if you
 * like, it is stable for the life of the device.
 */
struct typec_dp_state *tcpm_dp_state(struct udevice *dev);

/**
 * tcpm_enter_dp_altmode() - enter DisplayPort alt mode, on request
 *
 * Entering stays the port manager's business -- it is negotiated over the
 * connector, it has to happen while the transceiver is awake, and a consumer
 * that sent Enter Mode itself would be a second entity driving the partner's
 * mode state. But it must follow INTENT, which is why this is a request and not
 * something the port does to every partner it meets.
 *
 * It used to be the latter, and that was a bug with reach well past the board
 * it was written for. Alt-mode entry ran from tcpm_poll_event() on every probe,
 * and the USB3 PHY reaches a probe: rockchip_usb3_phy_power_on() ->
 * rockchip_tcphy_set_mode() -> rockchip_tcphy_get_orientation(), which has
 * always probed. So `usb start` alone would send Enter Mode to a USB-C dock and
 * configure pin assignment C -- four DisplayPort lanes and no SuperSpeed --
 * while the very same call finished configuring the PHY for USB3. Nothing exits
 * the mode; on this hardware only a physical unplug recovers it.
 *
 * Call this when something actually wants a DisplayPort link. No VBUS, or a
 * connector the board never declared for DisplayPort, is an ordinary finding
 * and returns quietly.
 *
 * Return: 0 if the mode is entered (or was already), -ENOENT if this connector
 * declares no DisplayPort or can never be the host (a sink-only port, which
 * cannot send Enter Mode), -ENODEV if there is nothing attached, or a negative
 * error from the VDM exchange.
 */
int tcpm_enter_dp_altmode(struct udevice *dev);

#endif /* __LINUX_USB_TCPM_H */
