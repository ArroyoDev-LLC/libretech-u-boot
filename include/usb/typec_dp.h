/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * DisplayPort Alt Mode on USB Type-C, DPAM 1.0b.
 *
 * Trimmed from the Linux header of the same name to the fields a
 * DFP_D (source) needs to discover a sink, enter the mode, and commit a
 * pin assignment.
 */

#ifndef __USB_TYPEC_DP_H
#define __USB_TYPEC_DP_H

#include <errno.h>
#include <linux/bitops.h>
#include "pd_vdo.h"

#define USB_TYPEC_DP_SID	0xff01

/*
 * The DP-specific commands are the first two vendor commands. Unlike
 * Enter Mode -- whose object position is the 1-based index of the mode VDO
 * in the Discover Modes response -- these always carry the object position
 * of the DP mode itself.
 */
#define DP_CMD_STATUS_UPDATE	VDO_CMD_VENDOR(0)	/* 0x10 */
#define DP_CMD_CONFIGURE	VDO_CMD_VENDOR(1)	/* 0x11 */

/* Pin assignments, as bit positions in the capability/configure bytes */
#define DP_PIN_ASSIGN_A		0
#define DP_PIN_ASSIGN_B		1
#define DP_PIN_ASSIGN_C		2
#define DP_PIN_ASSIGN_D		3
#define DP_PIN_ASSIGN_E		4
#define DP_PIN_ASSIGN_F		5

/* Pin assignments that carry DP on all four lanes */
#define DP_PIN_ASSIGN_DP_ONLY_MASK	(BIT(DP_PIN_ASSIGN_A) | \
					 BIT(DP_PIN_ASSIGN_C) | \
					 BIT(DP_PIN_ASSIGN_E))
/* Pin assignments that leave one lane pair for USB 3 */
#define DP_PIN_ASSIGN_MULTI_FUNC_MASK	(BIT(DP_PIN_ASSIGN_B) | \
					 BIT(DP_PIN_ASSIGN_D) | \
					 BIT(DP_PIN_ASSIGN_F))

/* DisplayPort Capabilities VDO, returned by Discover Modes for this SVID */
#define DP_CAP_CAPABILITY(cap)		((cap) & 3)
#define   DP_CAP_UFP_D			1
#define   DP_CAP_DFP_D			2
#define   DP_CAP_DFP_D_AND_UFP_D	3
#define DP_CAP_DP_SIGNALLING(cap)	(((cap) >> 2) & 0xf)
#define DP_CAP_RECEPTACLE		BIT(6)
#define DP_CAP_USB			BIT(7)
#define DP_CAP_DFP_D_PIN_ASSIGN(cap)	(((cap) >> 8) & 0xff)
#define DP_CAP_UFP_D_PIN_ASSIGN(cap)	(((cap) >> 16) & 0xff)

/*
 * The two pin-assignment bytes are labelled for a device that presents a
 * RECEPTACLE. On a captive plug they mean the opposite, so every
 * interpretation of a partner's VDO has to go through these, not through the
 * raw accessors above.
 */
#define DP_CAP_PIN_ASSIGN_UFP_D(cap)	((cap) & DP_CAP_RECEPTACLE ?	\
					 DP_CAP_UFP_D_PIN_ASSIGN(cap) :	\
					 DP_CAP_DFP_D_PIN_ASSIGN(cap))
#define DP_CAP_PIN_ASSIGN_DFP_D(cap)	((cap) & DP_CAP_RECEPTACLE ?	\
					 DP_CAP_DFP_D_PIN_ASSIGN(cap) :	\
					 DP_CAP_UFP_D_PIN_ASSIGN(cap))

/* DisplayPort Status Update VDO */
#define DP_STATUS_CONNECTION(st)	((st) & 3)
#define   DP_STATUS_CON_DISABLED	0
#define   DP_STATUS_CON_DFP_D		1
#define   DP_STATUS_CON_UFP_D		2
#define   DP_STATUS_CON_BOTH		3
#define DP_STATUS_POWER_LOW		BIT(2)
#define DP_STATUS_ENABLED		BIT(3)
#define DP_STATUS_PREFER_MULTI_FUNC	BIT(4)
#define DP_STATUS_SWITCH_TO_USB		BIT(5)
#define DP_STATUS_EXIT_DP_MODE		BIT(6)
#define DP_STATUS_HPD_STATE		BIT(7)	/* 0 = HPD_Low, 1 = HPD_High */
#define DP_STATUS_IRQ_HPD		BIT(8)

/*
 * DisplayPort Configure VDO. The two-bit select-configuration field always
 * describes what the UFP_U (the partner) is to become -- so a source driving
 * a monitor sets UFP_U_AS_UFP_D, which reads backwards until you notice the
 * field is about the other end of the cable.
 */
#define DP_CONF_CURRENTLY(conf)		((conf) & 3)
#define DP_CONF_USB			0
#define DP_CONF_UFP_U_AS_DFP_D		BIT(0)
#define DP_CONF_UFP_U_AS_UFP_D		BIT(1)
#define DP_CONF_SIGNALLING_SHIFT	2
#define DP_CONF_SIGNALLING_MASK		(0xf << DP_CONF_SIGNALLING_SHIFT)
#define DP_CONF_SET_PIN_ASSIGN(a)	((a) << 8)
#define DP_CONF_GET_PIN_ASSIGN(conf)	(((conf) >> 8) & 0xff)

/* A partner may expose several modes under this SVID; keep a few. */
#define TYPEC_DP_MAX_MODES	6

/**
 * struct typec_dp_state - one port's DisplayPort alt mode
 * @port_vdo:    our own capability, from the connector's altmodes node
 * @partner_vdo: the partner's, from Discover Modes
 * @mode_pos:    object position of that mode, one-based; 0 = not discovered
 * @status:      the partner's last DP Status VDO
 * @conf:        the DP Configure VDO accepted; pin assignment in bits 15:8
 * @active:      Enter Mode succeeded
 *
 * Owned by the caller, not by the port manager -- DisplayPort is a consumer
 * of a Type-C port, and a port may have other consumers.
 *
 * Purely an OUTPUT: both calls below fill it in and neither reads it, so
 * there is no initialisation contract for a caller to get wrong.
 */
struct typec_dp_state {
	u32 port_vdo;
	u32 partner_vdo;
	u8 mode_pos;
	u32 status;
	u32 conf;
	bool active;
};

#if CONFIG_IS_ENABLED(TYPEC_DISPLAYPORT)

/**
 * typec_dp_discover() - find the partner's DisplayPort mode
 *
 * Return: 0 on success, -EOPNOTSUPP if this port declares no DisplayPort
 * capability, -ENODEV if the partner has none, or an errno from the port
 * manager.
 */
int typec_dp_discover(struct udevice *tcpm, struct typec_dp_state *dp);

/**
 * typec_dp_enter() - enter DisplayPort alt mode as a source
 *
 * Enter Mode, Status Update, Configure -- in that order, which is not a style
 * choice: Configure's direction bit and pin assignment are both derived from
 * the connection field the partner reports in its Status Update, so
 * configuring before asking picks a direction blind.
 *
 * Runs typec_dp_discover() itself; @dp need not be primed. On success the
 * partner has switched its connector to DisplayPort and @dp->conf carries the
 * negotiated pin assignment, which the caller owes to the Type-C PHY.
 */
int typec_dp_enter(struct udevice *tcpm, struct typec_dp_state *dp);

/**
 * typec_dp_exit() - leave DisplayPort alt mode
 *
 * Also run automatically when typec_dp_enter() fails after Enter Mode was
 * accepted, and when Enter Mode is NAK'd (which usually means the partner is
 * already in the mode). A partner left in DisplayPort alt mode by an
 * abandoned entry stays there, and nothing on this side remembers across a
 * reboot -- the only other way back is unplugging the cable.
 *
 * Leaves @dp->active set if the partner did not confirm the exit, because it
 * probably has not left.
 */
int typec_dp_exit(struct udevice *tcpm, struct typec_dp_state *dp);

/**
 * typec_dp_autoenter() - enter DisplayPort alt mode if the BOARD asked for it
 *
 * The connector's altmodes/displayport node is the board saying "this
 * receptacle's lanes are routed to a DisplayPort source". Nothing else in the
 * device tree says it, and a board that says it has no second way to ask for
 * the mode to be entered -- so a port that carries the node and is sitting in a
 * settled contract should be IN the mode, not waiting for someone to type
 * `tcpm dp enter`.
 *
 * Separate from typec_dp_enter() so that the port manager can run this without
 * knowing what DisplayPort is: the policy question ("did the board declare it?")
 * is answered here, out of the same node typec_dp_read_port_vdo() already
 * reads, rather than by open-coding an altmodes walk in tcpm.c.
 *
 * Return: 0 if the mode was entered by this call, -EALREADY if it was already
 * active (also success -- the caller must not report an entry that did not
 * happen), -ENOENT if this connector declares no DisplayPort alt mode (not an
 * error -- the ordinary case for a port that is only ever USB), or an errno
 * from reading a malformed declaration or from the entry itself.
 */
int typec_dp_autoenter(struct udevice *tcpm);

#else
static inline int typec_dp_exit(struct udevice *tcpm,
				struct typec_dp_state *dp)
{
	return -ENOSYS;
}

/*
 * Stubs, because cmd/tcpm.c offers `tcpm dp` whenever CMD_TCPM is set and
 * CMD_TCPM depends only on TYPEC_TCPM. Without these, a board that enables
 * the command without the alt mode -- rock5b-rk3588 does -- fails to LINK,
 * not to compile, which is a long way from where the mistake was made.
 */
static inline int typec_dp_discover(struct udevice *tcpm,
				    struct typec_dp_state *dp)
{
	return -ENOSYS;
}

static inline int typec_dp_enter(struct udevice *tcpm,
				 struct typec_dp_state *dp)
{
	return -ENOSYS;
}

static inline int typec_dp_autoenter(struct udevice *tcpm)
{
	return -ENOENT;
}
#endif

#endif /* __USB_TYPEC_DP_H */
