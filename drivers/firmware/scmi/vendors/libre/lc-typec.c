// SPDX-License-Identifier: GPL-2.0+
/*
 * Libre Computer lc-typec SCMI vendor protocol 0x80: the client.
 *
 * Binds to the static scmi/protocol@80 node when Base lists 0x80. It is the
 * live source of the rear Type-C0 connector state for U-Boot consumers -- the
 * Type-C PHY asks it for the plug orientation instead of the fusb302 TCPM
 * the M0 has displaced -- and backs `scmi typec status`.
 *
 * It does NOT take the ownership decision; that is scmi_m0_owns_typec(),
 * taken once in the agent's post_bind on the Base device, so it holds even
 * if this driver fails to probe.
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <scmi_agent.h>
#include <scmi_agent-uclass.h>
#include <scmi_lc_typec.h>
#include <scmi_protocols.h>

static int scmi_lc_typec_probe(struct udevice *dev)
{
	int ret;

	ret = devm_scmi_of_get_channel(dev);
	if (ret) {
		dev_err(dev, "failed to get channel (%d)\n", ret);
		return ret;
	}

	return 0;
}

int scmi_lc_typec_rear_status(struct scmi_lc_typec_ext_status *st)
{
	struct udevice *agent, *proto;

	if (uclass_first_device_err(UCLASS_SCMI_AGENT, &agent))
		return -ENODEV;

	/* NULL, not an error print, when the server did not list 0x80. */
	proto = scmi_get_protocol(agent, SCMI_PROTOCOL_ID_VENDOR_80);
	if (!proto || !device_active(proto))
		return -ENODEV;

	return scmi_lc_typec_ext_status(proto, SCMI_LC_TYPEC_CONN_REAR, st);
}

U_BOOT_DRIVER(scmi_lc_typec) = {
	.name = "scmi_lc_typec",
	.id = UCLASS_SCMI_BASE,
	.probe = scmi_lc_typec_probe,
};

static struct scmi_proto_match match[] = {
	{ .proto_id = SCMI_PROTOCOL_ID_VENDOR_80 },
	{ /* Sentinel */ }
};

U_BOOT_SCMI_PROTO_DRIVER(scmi_lc_typec, match);
