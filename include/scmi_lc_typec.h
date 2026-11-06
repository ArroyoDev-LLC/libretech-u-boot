/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Libre Computer "lc-typec" SCMI vendor protocol 0x80.
 *
 * Served by the RK3399 Cortex-M0 SCMI server ("open SC") when it owns the
 * rear Type-C0 port: the M0 is the sole I2C7 master, runs the sink policy
 * engine against the FUSB302 at 0x22, and answers every message of this
 * protocol from RAM. Base reports vendor "libre-computer", sub-vendor
 * "rk3399-m0".
 *
 * Design: claude:rockchip/rk3399/plan/m0-typec-pd-owner.md, "Vendor protocol
 * 0x80" and "Device tree and fallback".
 */
#ifndef _SCMI_LC_TYPEC_H
#define _SCMI_LC_TYPEC_H

#include <stdbool.h>
#include <asm/types.h>
#include <dm/ofnode.h>
#include <linux/errno.h>

struct udevice;

/* Base DISCOVER_VENDOR string of the only server that speaks this layout. */
#define SCMI_LC_TYPEC_VENDOR		"libre-computer"

/*
 * The I2C7 controller the M0 owns. The TCPC on this bus is the rear port's
 * FUSB302 (&fusb0); the front port's (&fusb1) is on I2C4 and never M0's.
 */
#define SCMI_LC_TYPEC_M0_I2C_BASE	0xff160000

enum scmi_lc_typec_msg {
	SCMI_LC_TYPEC_PROTOCOL_VERSION		= 0x0,
	SCMI_LC_TYPEC_PROTOCOL_ATTRIBUTES	= 0x1,
	SCMI_LC_TYPEC_PROTOCOL_MESSAGE_ATTRIBUTES = 0x2,
	SCMI_LC_TYPEC_UCSI_READ			= 0x3,
	SCMI_LC_TYPEC_UCSI_WRITE		= 0x4,
	SCMI_LC_TYPEC_CONNECTOR_EXT_STATUS	= 0x5,
	SCMI_LC_TYPEC_NOTIFY_ENABLE		= 0x6,
};

#define SCMI_LC_TYPEC_ORIENT_NONE	0
#define SCMI_LC_TYPEC_ORIENT_NORMAL	1
#define SCMI_LC_TYPEC_ORIENT_REVERSE	2

#define SCMI_LC_TYPEC_OWNER_M0		0
#define SCMI_LC_TYPEC_OWNER_YIELDED	1

/* The rear Type-C0 port is connector 0 of this protocol. */
#define SCMI_LC_TYPEC_CONN_REAR		0

struct scmi_lc_typec_ext_status_in {
	u32 connector;
};

struct scmi_lc_typec_ext_status_out {
	s32 status;
	u32 orientation;
	u32 contract_mv;
	u32 contract_ma;
	u32 pd_rev;
	u32 owner;
};

/**
 * struct scmi_lc_typec_ext_status - CONNECTOR_EXT_STATUS, decoded
 * @orientation:	SCMI_LC_TYPEC_ORIENT_*
 * @contract_mv:	negotiated contract voltage, mV (0 = none)
 * @contract_ma:	negotiated contract current, mA (0 = none)
 * @pd_rev:		PD revision as the M0 reports it (raw)
 * @owner:		SCMI_LC_TYPEC_OWNER_*
 */
struct scmi_lc_typec_ext_status {
	u32 orientation;
	u32 contract_mv;
	u32 contract_ma;
	u32 pd_rev;
	u32 owner;
};

#if CONFIG_IS_ENABLED(SCMI_LC_TYPEC)
/**
 * scmi_lc_typec_ext_status() - send CONNECTOR_EXT_STATUS on an SCMI device
 * @dev:	any SCMI protocol device (the Base device or the 0x80 client)
 * @connector:	connector index
 * @st:		decoded response, filled on success only
 * Return: 0, a transport errno, or the SCMI status as an errno
 */
int scmi_lc_typec_ext_status(struct udevice *dev, u32 connector,
			     struct scmi_lc_typec_ext_status *st);

/**
 * scmi_m0_owns_typec() - the one bind-time decision, cached
 *
 * True only if Base listed protocol 0x80 from the "libre-computer" server AND
 * CONNECTOR_EXT_STATUS on connector 0 read status 0 and owner == M0, at the
 * moment the SCMI agent bound. Never re-queried: every consumer (the fusb302
 * refusal, the Type-C PHY, the kernel DT fixup) reads this one answer.
 */
bool scmi_m0_owns_typec(void);

/**
 * scmi_m0_typec_query() - a FRESH CONNECTOR_EXT_STATUS on the Base device
 * @st:	decoded response, filled on success only
 *
 * Independent of the 0x80 client driver. Used by the kernel DT fixup to
 * confirm the M0 has not yielded since bind time.
 * Return: 0 or a negative errno (-ENODEV when there is no usable agent)
 */
int scmi_m0_typec_query(struct scmi_lc_typec_ext_status *st);

/**
 * scmi_m0_owns_tcpc() - is this TCPC node the port the M0 owns?
 * @tcpc:	a Type-C port controller node (e.g. the fusb302)
 *
 * True only when scmi_m0_owns_typec() and @tcpc sits on the M0's I2C7.
 */
bool scmi_m0_owns_tcpc(ofnode tcpc);

/**
 * scmi_lc_typec_rear_status() - the rear connector, via the 0x80 client
 * @st:	decoded response, filled on success only
 * Return: 0, -ENODEV when the 0x80 client is not bound, or an errno
 */
int scmi_lc_typec_rear_status(struct scmi_lc_typec_ext_status *st);
#else
static inline bool scmi_m0_owns_typec(void)
{
	return false;
}

static inline bool scmi_m0_owns_tcpc(ofnode tcpc)
{
	return false;
}

static inline int scmi_m0_typec_query(struct scmi_lc_typec_ext_status *st)
{
	return -ENOSYS;
}

static inline int scmi_lc_typec_rear_status(struct scmi_lc_typec_ext_status *st)
{
	return -ENOSYS;
}
#endif

#endif /* _SCMI_LC_TYPEC_H */
