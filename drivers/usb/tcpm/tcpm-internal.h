/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2017 Google, Inc
 * Copyright 2024 Collabora
 */

#ifndef __LINUX_USB_TCPM_INTERNAL_H
#define __LINUX_USB_TCPM_INTERNAL_H

#include <cyclic.h>
#include <usb/pd_vdo.h>
#include <usb/tcpm.h>
#include <usb/typec_dp.h>

#define FOREACH_TCPM_STATE(S)			\
	S(INVALID_STATE),			\
	S(TOGGLING),			\
	S(SRC_UNATTACHED),			\
	S(SRC_ATTACH_WAIT),			\
	S(SRC_ATTACHED),			\
	S(SRC_STARTUP),				\
	S(SRC_SEND_CAPABILITIES),		\
	S(SRC_SEND_CAPABILITIES_TIMEOUT),	\
	S(SRC_NEGOTIATE_CAPABILITIES),		\
	S(SRC_TRANSITION_SUPPLY),		\
	S(SRC_READY),				\
	S(SRC_WAIT_NEW_CAPABILITIES),		\
						\
	S(SNK_UNATTACHED),			\
	S(SNK_ATTACH_WAIT),			\
	S(SNK_DEBOUNCED),			\
	S(SNK_ATTACHED),			\
	S(SNK_STARTUP),				\
	S(SNK_DISCOVERY),			\
	S(SNK_DISCOVERY_DEBOUNCE),		\
	S(SNK_DISCOVERY_DEBOUNCE_DONE),		\
	S(SNK_WAIT_CAPABILITIES),		\
	S(SNK_NEGOTIATE_CAPABILITIES),		\
	S(SNK_TRANSITION_SINK),			\
	S(SNK_TRANSITION_SINK_VBUS),		\
	S(SNK_READY),				\
						\
	S(HARD_RESET_SEND),			\
	S(HARD_RESET_START),			\
	S(SRC_HARD_RESET_VBUS_OFF),		\
	S(SRC_HARD_RESET_VBUS_ON),		\
	S(SNK_HARD_RESET_SINK_OFF),		\
	S(SNK_HARD_RESET_WAIT_VBUS),		\
	S(SNK_HARD_RESET_SINK_ON),		\
						\
	S(SOFT_RESET),				\
	S(SOFT_RESET_SEND),			\
						\
	S(DR_SWAP_ACCEPT),			\
	S(DR_SWAP_SEND),			\
	S(DR_SWAP_SEND_TIMEOUT),		\
	S(DR_SWAP_CHANGE_DR),			\
						\
	S(ERROR_RECOVERY),			\
	S(PORT_RESET),				\
	S(PORT_RESET_WAIT_OFF)

#define GENERATE_TCPM_ENUM(e)		e
#define GENERATE_TCPM_STRING(s)		#s
#define TCPM_POLL_EVENT_TIME_OUT	2000
/*
 * Absolute bound on the probe-time pump, however many events keep re-arming
 * TCPM_POLL_EVENT_TIME_OUT. A partner that never stops talking (or a swap
 * that never comes) must not hold the pump -- and with it any `tcpm` getter,
 * which settles it inline -- forever.
 */
#define TCPM_POLL_EVENT_HARD_MAX	6000
/*
 * Longest a sink may sit in SNK_ATTACH_WAIT without debouncing. A DRP partner
 * whose Rp blinks faster than PD_T_CC_DEBOUNCE cancels both the attach and the
 * detach transition on every edge, so without a bound the port waits forever.
 */
#define PD_T_ATTACH_WAIT_MAX		1000

enum tcpm_state {
	FOREACH_TCPM_STATE(GENERATE_TCPM_ENUM)
};

/*
 * What DATA roles this port can take, from the connector's data-role
 * property. Distinct from the power role: a port can be a power sink and a
 * data host at once, which is exactly what a laptop driving a self-powered
 * DisplayPort monitor is, and what DisplayPort alt mode needs from us.
 */
enum typec_port_data {
	TYPEC_PORT_DATA_UFP,
	TYPEC_PORT_DATA_DFP,
	TYPEC_PORT_DATA_DRD,
};

enum pd_msg_request {
	PD_MSG_NONE = 0,
	PD_MSG_CTRL_REJECT,
	PD_MSG_CTRL_WAIT,
	PD_MSG_CTRL_NOT_SUPP,
	PD_MSG_DATA_SINK_CAP,
	PD_MSG_DATA_SOURCE_CAP,
};

struct tcpm_port {
	enum typec_port_type typec_type;
	enum typec_port_data typec_data;
	int typec_prefer_role;

	enum typec_role vconn_role;
	enum typec_role pwr_role;
	enum typec_data_role data_role;

	struct typec_partner *partner;

	enum typec_cc_status cc_req;
	enum typec_cc_status cc1;
	enum typec_cc_status cc2;
	enum typec_cc_polarity polarity;

	bool attached;
	bool connected;
	/* timer_get_us() at which the probe-time pump stops waiting for an event */
	u64 poll_event_deadline;
	/* timer_get_us() past which the pump stops regardless of events */
	u64 poll_event_hard_deadline;
	/* events that re-armed the pump's deadline since the pump started */
	unsigned int poll_event_count;
	/* get_timer() when the port last entered SNK_ATTACH_WAIT afresh */
	ulong attach_wait_start;
	/* get_timer() deadline of a source-only toggle round; 0 = none */
	ulong src_toggle_until;
	/* the power inlet has used its one Soft Reset for this attach */
	bool inlet_soft_reset_done;
	/* print every state change (tcpm_force_attach()), relative to trace_t0 */
	bool attach_trace;
	ulong trace_t0;
	enum typec_port_type port_type;

	/*
	 * Set to true when vbus is greater than VSAFE5V min.
	 * Set to false when vbus falls below vSinkDisconnect max threshold.
	 */
	bool vbus_present;

	/*
	 * Set to true when vbus is less than VSAFE0V max.
	 * Set to false when vbus is greater than VSAFE0V max.
	 */
	bool vbus_vsafe0v;

	bool vbus_never_low;
	/*
	 * VBUS never dropped and the TCPC still held a PD session in which
	 * this sink was the data host: the partner is the device, and the
	 * first message of this boot must say so (get_prior_roles).
	 */
	bool prior_data_host;
	bool vbus_source;
	bool vbus_charge;

	int try_role;

	enum pd_msg_request queued_message;

	enum tcpm_state enter_state;
	enum tcpm_state prev_state;
	enum tcpm_state state;
	enum tcpm_state delayed_state;
	unsigned long delay_ms;

	bool state_machine_running;

	bool tx_complete;
	enum tcpm_transmit_status tx_status;

	unsigned int negotiated_rev;
	unsigned int message_id;
	unsigned int caps_count;
	unsigned int hard_reset_count;
	bool pd_capable;
	bool explicit_contract;
	unsigned int rx_msgid;

	/* Partner capabilities/requests */
	u32 sink_request;
	u32 source_caps[PDO_MAX_OBJECTS];
	unsigned int nr_source_caps;
	u32 sink_caps[PDO_MAX_OBJECTS];
	unsigned int nr_sink_caps;

	/*
	 * whether to wait for the Type-C device to send the DR_SWAP Message flag
	 * For Type-C device with Dual-Role Power and Dual-Role Data, the port side
	 * is used as sink + ufp, then the tcpm framework needs to wait for Type-C
	 * device to initiate DR_swap Message.
	 */
	bool wait_dr_swap_message;

	/* Local capabilities */
	u32 src_pdo[PDO_MAX_OBJECTS];
	unsigned int nr_src_pdo;
	u32 snk_pdo[PDO_MAX_OBJECTS];
	unsigned int nr_snk_pdo;

	unsigned int operating_snk_mw;
	bool update_sink_caps;

	/* Requested current / voltage to the port partner */
	u32 req_current_limit;
	u32 req_supply_voltage;
	/* Actual current / voltage limit of the local port */
	u32 current_limit;
	u32 supply_voltage;

	/* port belongs to a self powered device */
	bool self_powered;

	unsigned long delay_target;

	/*
	 * Structured VDM exchange state.
	 *
	 * U-Boot does not need Linux's asynchronous VDM state machine.
	 * tcpm_pd_transmit() already blocks until the TCPC reports the
	 * message gone, running the receive path inline while it waits, so a
	 * VDM exchange is just "send, then keep pumping the same poll loop
	 * until the response lands". One transaction is in flight at a time
	 * by construction.
	 *
	 * vdm_rx_cnt is the object count of the last received VDM including
	 * its header, and is zeroed by the sender before each request so a
	 * stale response cannot be mistaken for a fresh one.
	 */
	u32 vdm_rx[VDO_MAX_SIZE];
	unsigned int vdm_rx_cnt;

	/*
	 * What the in-flight exchange is waiting for, so the receive path can
	 * REFUSE to park anything else.
	 *
	 * The slot above holds one VDM, and since the FUSB302 driver started
	 * draining its whole RX FIFO per interrupt, more than one can be
	 * delivered between two looks at it. A monitor in DisplayPort alt
	 * mode sends unsolicited Attention VDMs, so the losing order is real:
	 * our ACK is parked, the Attention immediately overwrites it, and the
	 * response is gone before the wait loop ever runs. Filtering in the
	 * wait loop cannot fix that -- by then the message is already lost.
	 */
	bool vdm_waiting;
	u16 vdm_wait_svid;
	u8 vdm_wait_cmd;

	/*
	 * The negotiated DisplayPort alt mode, if any.
	 *
	 * It lives on the PORT because the port is what owns the connector,
	 * and because the answer has to outlive the command that produced it:
	 * the Type-C PHY needs the negotiated pin assignment to mux its lanes,
	 * and it asks later, from its own probe. A stack local in cmd/tcpm.c
	 * could not be asked at all -- which is why `tcpm dp exit` used to
	 * FABRICATE an alt-mode state to force an exit against a port whose
	 * real state it could not see.
	 *
	 * There is exactly one of these per port, so "is DP active here" has
	 * one answer. Invalidating it is the part that has to be got right:
	 * tcpm_reset_port() covers detach, error recovery, port reset and
	 * init, but HARD_RESET_START leaves ->attached true and never calls
	 * it, and DR_SWAP_CHANGE_DR changes who the DFP is. SOFT_RESET and
	 * the generic cc-change path must NOT clear it -- the first resets
	 * the protocol layer without leaving alt modes, and the second also
	 * fires on a plain Rp current-limit update, so clearing there would
	 * drop a live display every time the source re-advertised.
	 */
	struct typec_dp_state dp;

	/*
	 * The TCPC has been put in low power and cannot transmit until it is
	 * woken. Tracked here because the port manager is what decided to
	 * sleep it, so the port manager owes the wake.
	 */
	bool low_power;

	/* Partner alt modes, discovered. Generic -- no SVID is special here. */
	u16 partner_svids[SVID_DISCOVERY_MAX];
	unsigned int nr_partner_svids;

	/*
	 * The probe-time pump, when it runs off the critical path. See
	 * tcpm_post_probe(): the state machine is stepped from a cyclic tick
	 * while the rest of the boot goes on, and ->probe_pump_pending is
	 * true until the pump has reached the state the old synchronous loop
	 * stopped at. Every consumer settles it first (tcpm_settle()), so
	 * what a consumer observes is exactly what it observed before.
	 */
	struct cyclic_info cyclic;
	struct udevice *dev;
	bool probe_pump_pending;
	/* ->cyclic now runs tcpm_service_tick(): an attached port, after the pump */
	bool service_on;
};

extern const char * const tcpm_states[];

int tcpm_post_probe(struct udevice *dev);
int tcpm_pre_remove(struct udevice *dev);

/**
 * tcpm_settle() - finish the probe-time pump before reading the port
 *
 * A no-op once the pump has run to its end (or never started: a port with
 * no VBUS, or a build without CYCLIC). Otherwise runs the remaining pump
 * synchronously, exactly as the probe used to.
 */
void tcpm_settle(struct udevice *dev);

#endif
