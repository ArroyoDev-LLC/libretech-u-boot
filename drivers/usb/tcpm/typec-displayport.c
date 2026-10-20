// SPDX-License-Identifier: GPL-2.0+
/*
 * DisplayPort Alt Mode on USB Type-C, source (DFP_D) side.
 *
 * A consumer of the Type-C port manager, not a part of it. TCPM owns the
 * connector, the power contract and the VDM transport; everything here is
 * DisplayPort's own protocol on top of that transport, and the port manager
 * contains no knowledge of SVID 0xff01.
 *
 * That split is not tidiness. The pin assignment negotiated here has to reach
 * the Type-C PHY, the port's own capability comes from the connector's
 * altmodes node, and a second alt mode would want the same transport -- none
 * of which is the port manager's business.
 */

#include <dm.h>
#include <dm/device_compat.h>
#include <linux/err.h>
#include <usb/tcpm.h>
#include <usb/typec_dp.h>

/**
 * typec_dp_read_port_vdo() - this port's own DisplayPort capability
 *
 * From the connector's altmodes/displayport node, because it describes the
 * BOARD's lane routing -- which pin assignments this connector can actually
 * honour -- and not the SoC's. A port that declares none is refusing to enter
 * DisplayPort, which is different from a port that would accept anything.
 */
static int typec_dp_read_port_vdo(struct udevice *tcpm, u32 *vdo)
{
	ofnode connector, altmodes, dp;
	int ret;

	ret = tcpm_get_connector(tcpm, &connector);
	if (ret)
		return ret;

	altmodes = ofnode_find_subnode(connector, "altmodes");
	if (!ofnode_valid(altmodes))
		return -ENOENT;

	dp = ofnode_find_subnode(altmodes, "displayport");
	if (!ofnode_valid(dp))
		return -ENOENT;

	return ofnode_read_u32(dp, "vdo", vdo);
}

int typec_dp_discover(struct udevice *tcpm, struct typec_dp_state *dp)
{
	u32 modes[TYPEC_DP_MAX_MODES];
	u16 svids[SVID_DISCOVERY_MAX];
	unsigned int n, i;
	int ret;

	/*
	 * Clear only what discovery itself produces. This used to be a
	 * `memset(dp, 0, sizeof(*dp))`, which was harmless while @dp was a
	 * stack local in cmd/tcpm.c and became a bug the moment it became the
	 * PORT's state: a plain `tcpm dp` -- the read-only-looking one --
	 * would wipe ->active and ->conf while the partner was still in
	 * DisplayPort alt mode. The PHY would then see MODE_DFP_USB on a live
	 * link, and `tcpm dp exit` would answer "not in alt mode" and refuse,
	 * leaving the partner stuck somewhere only a physical unplug reaches.
	 *
	 * ->active and ->conf describe the MODE, not the discovery, and only
	 * an Enter or an Exit may change them. typec_dp_enter() calls this
	 * function on its way in, so the same rule protects a re-enter whose
	 * discovery fails.
	 */
	dp->port_vdo = 0;
	dp->partner_vdo = 0;
	dp->mode_pos = 0;
	dp->status = 0;

	ret = typec_dp_read_port_vdo(tcpm, &dp->port_vdo);
	if (ret) {
		dev_dbg(tcpm, "port declares no DisplayPort alt mode\n");
		return -EOPNOTSUPP;
	}

	ret = tcpm_discover_svids(tcpm);
	if (ret)
		return ret;

	n = tcpm_get_partner_svids(tcpm, svids, ARRAY_SIZE(svids));
	for (i = 0; i < n; i++)
		if (svids[i] == USB_TYPEC_DP_SID)
			break;

	if (i == n) {
		dev_dbg(tcpm, "partner has no DisplayPort SVID\n");
		return -ENODEV;
	}

	ret = tcpm_discover_modes(tcpm, USB_TYPEC_DP_SID, modes,
				  ARRAY_SIZE(modes));
	if (ret < 0)
		return ret;
	if (!ret)
		return -ENODEV;

	/*
	 * Every mode VDO under this SVID is a DisplayPort capability VDO, so
	 * the first is the one to use -- but record WHICH, because the object
	 * position Enter Mode carries is the one-based index in the Discover
	 * Modes response and a partner may return DisplayPort second.
	 */
	dp->partner_vdo = modes[0];
	dp->mode_pos = 1;

	return 0;
}

/**
 * typec_dp_pin_assign() - pick exactly one pin assignment
 *
 * The intersection of what the partner offers with what this port declared,
 * narrowed to a SINGLE bit. Linux can leave two bits set here; its consumer
 * then takes get_count_order() of the pair, lands past the last DisplayPort
 * state, and falls back to USB with no error reported anywhere. Collapsing
 * here is cheaper than diagnosing that.
 */
static u8 typec_dp_pin_assign(const struct typec_dp_state *dp, u8 con)
{
	u8 pin;
	int i;

	/*
	 * Always the PARTNER's UFP_D byte against OUR DFP_D byte. The operands
	 * do not swap with the connection direction -- only which end is which
	 * does, and that is already encoded in the two field names.
	 *
	 * Getting this backwards yields an empty intersection against a real
	 * monitor and reads as "no pin assignment in common", which sounds
	 * like an incompatibility rather than an arithmetic error. Measured:
	 * a partner offering 0x00000405 (plug, DFP_D byte 0x04 = C) against
	 * our 0x00001c46 (receptacle, DFP_D byte 0x1c = C|D|E) gives C -- but
	 * with the operands swapped it is 0x00 & 0x00 = nothing.
	 *
	 * Receptacle-aware in BOTH directions. Linux uses the raw accessors on
	 * the CON_DFP_D branch, which misreads a captive-plug partner acting
	 * as a source; there is no reason to reproduce that here.
	 */
	if (con == DP_STATUS_CON_DFP_D)
		pin = DP_CAP_PIN_ASSIGN_DFP_D(dp->partner_vdo) &
		      DP_CAP_PIN_ASSIGN_UFP_D(dp->port_vdo);
	else
		pin = DP_CAP_PIN_ASSIGN_UFP_D(dp->partner_vdo) &
		      DP_CAP_PIN_ASSIGN_DFP_D(dp->port_vdo);

	if (dp->status & DP_STATUS_PREFER_MULTI_FUNC &&
	    pin & DP_PIN_ASSIGN_MULTI_FUNC_MASK)
		pin &= DP_PIN_ASSIGN_MULTI_FUNC_MASK;
	else if (pin & DP_PIN_ASSIGN_DP_ONLY_MASK)
		pin &= DP_PIN_ASSIGN_DP_ONLY_MASK;

	/* C first if it survived -- four DP lanes and no multiplexing. */
	if (pin & BIT(DP_PIN_ASSIGN_C))
		return BIT(DP_PIN_ASSIGN_C);

	for (i = 0; i <= DP_PIN_ASSIGN_F; i++)
		if (pin & BIT(i))
			return BIT(i);

	return 0;
}

/**
 * typec_dp_exit() - leave DisplayPort alt mode
 *
 * Also the failure path of typec_dp_enter(). A partner that has been told to
 * ENTER and then abandoned stays in DisplayPort alt mode, and since nothing
 * on this side remembers that across a reboot, the only way back is to
 * unplug the cable -- which is not available on a board at the end of a
 * lease. Measured the hard way: an entry that failed at the pin assignment
 * left a monitor unusable for every subsequent attempt until it was
 * physically re-plugged.
 */
static int dp_exit(struct udevice *tcpm, struct typec_dp_state *dp,
		   bool speculative)
{
	u32 vdo[1];
	int ret;

	if (!dp->active)
		return 0;

	/*
	 * Exit Mode is the recovery command, so it has to work on a port
	 * nothing has driven yet -- `tcpm dp exit` after a reset, with no
	 * discovery in front of it. Transmitting into a port that has not
	 * reached SRC_READY/SNK_READY costs three timeouts and reports
	 * -ETIMEDOUT for a link that was fine. The discovery path reaches
	 * this through tcpm_discover_svids(); do it here so the exit does
	 * not depend on having been preceded by one.
	 */
	ret = tcpm_wait_ready(tcpm, PD_T_SINK_WAIT_CAP * 4);
	if (ret)
		return ret;

	vdo[0] = tcpm_svdm_build_header(USB_TYPEC_DP_SID, dp->mode_pos,
					CMD_EXIT_MODE);
	ret = tcpm_vdm_exchange(tcpm, vdo, 1, NULL, 0, NULL,
				PD_T_VDM_WAIT_MODE_E * 5);
	if (ret < 0 || ret != CMDT_RSP_ACK) {
		/*
		 * Do NOT clear ->active here. The partner did not confirm it
		 * left the mode, so it is probably still in it -- which is the
		 * state that needs a physical unplug to clear. Saying
		 * otherwise would hide exactly the condition worth reporting,
		 * and this is the likely case, since Exit Mode is usually sent
		 * because the link is already unhealthy.
		 *
		 * Unless the exit was SPECULATIVE. typec_dp_enter() sends one
		 * on a NAK'd Enter, on the theory that the partner is already
		 * in the mode; when that theory is wrong -- the partner was
		 * busy, or does not do DP sink -- the NAK we get back means
		 * the mode was never entered, and announcing that a device may
		 * be stuck needing a physical unplug is false on the failure
		 * path users hit most. Report it as the debug detail it is
		 * and let the caller's own error stand.
		 */
		if (speculative)
			dev_dbg(tcpm, "DisplayPort speculative Exit Mode not accepted: %d\n",
				ret);
		else
			dev_err(tcpm, "DisplayPort Exit Mode failed: %d -- partner may still be in alt mode\n",
				ret);
		return ret < 0 ? ret : -EPROTO;
	}

	dp->active = false;
	dp->conf = 0;

	return 0;
}

int typec_dp_exit(struct udevice *tcpm, struct typec_dp_state *dp)
{
	return dp_exit(tcpm, dp, false);
}

int typec_dp_enter(struct udevice *tcpm, struct typec_dp_state *dp)
{
	u32 vdo[2], rx[VDO_MAX_SIZE];
	unsigned int rx_cnt;
	u8 con, pin;
	u32 conf;
	int ret;

	/*
	 * Always discover, and never look at @dp on the way in.
	 *
	 * This used to skip discovery when dp->mode_pos was already set,
	 * which made the struct an INPUT as well as an output and put a
	 * zero-initialise contract on every caller. cmd/tcpm.c did not honour
	 * it, and the failure was silent and confusing rather than a crash:
	 * uninitialised stack made mode_pos non-zero, discovery was skipped,
	 * and the command printed a port capability of 00000003 and a pin
	 * assignment of "multiple" out of a configure word that was never
	 * computed.
	 *
	 * Discovery is three VDMs against a partner we are about to drive a
	 * display through. It is not worth an API that can be held wrong.
	 */
	ret = typec_dp_discover(tcpm, dp);
	if (ret)
		return ret;

	vdo[0] = tcpm_svdm_build_header(USB_TYPEC_DP_SID, dp->mode_pos,
					CMD_ENTER_MODE);
	ret = tcpm_vdm_exchange(tcpm, vdo, 1, NULL, 0, NULL,
				PD_T_VDM_WAIT_MODE_E * 5);
	if (ret < 0) {
		dev_err(tcpm, "DisplayPort Enter Mode failed: %d\n", ret);
		return ret;
	}

	if (ret != CMDT_RSP_ACK) {
		/*
		 * A NAK most often means the partner is ALREADY in the mode --
		 * from a previous entry that nothing exited. Assume so and try
		 * to take it back out, because the alternative is leaving it
		 * stuck somewhere only a physical unplug can reach. Sending
		 * Exit Mode to a partner that is not in the mode costs a NAK.
		 */
		dev_err(tcpm, "DisplayPort Enter Mode refused: %d -- assuming already entered, exiting\n",
			ret);

		/*
		 * ->active is set here only to get past dp_exit()'s "are we in
		 * the mode" guard. It is an ASSUMPTION, not an observation, and
		 * it must not survive this block.
		 *
		 * Leaving it set was harmless while @dp was a stack local and
		 * is not now that it is the PORT's state. A partner that simply
		 * does not do DisplayPort sink NAKs the Enter and then NAKs the
		 * speculative Exit too -- the ordinary case, not a corner --
		 * and the port was left claiming an active alt mode for the
		 * rest of the boot. Since typec_dp_discover() deliberately does
		 * not clear ->conf, a refused re-entry after one good entry left
		 * ->conf holding a stale 4-lane pin assignment beside it, and
		 * the PHY's get_mode() answers MODE_DFP_DP off exactly that
		 * pair -- for a partner sitting in USB.
		 *
		 * The two outcomes of the speculative exit do NOT mean the same
		 * thing, and an earlier version of this comment claimed they
		 * did by attributing to dp_exit() a reading it does not have.
		 * ACK: the partner was in the mode and has left. NAK: dp_exit()
		 * says the opposite of "never entered" -- that a partner which
		 * does not confirm leaving is probably still in it, which is
		 * the entire premise for sending this Exit at all.
		 *
		 * So on a NAK we genuinely cannot tell, and there is no state
		 * here that expresses "maybe". Clearing is the deliberate
		 * choice of the two available wrong answers: the far more
		 * common cause of a NAK'd Enter is a partner that does no
		 * DisplayPort sink at all, and clearing makes the port converge
		 * on the state a fresh attach produces. The cost when the guess
		 * is wrong -- a partner stuck in DP that we treat as USB -- is
		 * a link that does not come up until the cable is unplugged,
		 * which is also the cost of guessing the other way.
		 */
		dp->active = true;
		dp_exit(tcpm, dp, true);
		dp->active = false;
		dp->conf = 0;

		return -EPROTO;
	}

	dp->active = true;

	/*
	 * The previous configuration, if there was one, stopped being true the
	 * moment the partner accepted this Enter -- so clear it here rather
	 * than overwriting it at the end.
	 *
	 * Every `goto exit_mode` below reaches typec_dp_exit(), which by
	 * design does NOT clear ->active or ->conf when the Exit is not ACKed
	 * (see dp_exit()). Without this line, one good entry followed by a
	 * re-entry that ACKs Enter but fails at Status Update or Configure --
	 * a monitor that renegotiates, a re-plug mid-session -- leaves
	 * ->active set beside a STALE 4-lane pin assignment, and
	 * rockchip_tcphy_get_mode() reads exactly that pair and answers
	 * MODE_DFP_DP for a link that was never configured. ->conf is now
	 * non-zero only between a successful DP_Configure and the next
	 * attempt.
	 */
	dp->conf = 0;

	/* "I have a DFP_D connected", and nothing else. */
	vdo[0] = tcpm_svdm_build_header(USB_TYPEC_DP_SID, dp->mode_pos,
					DP_CMD_STATUS_UPDATE);
	vdo[1] = DP_STATUS_CON_DFP_D;
	ret = tcpm_vdm_exchange(tcpm, vdo, 2, rx, ARRAY_SIZE(rx), &rx_cnt,
				PD_T_VDM_SNDR_RSP * 10);
	if (ret < 0 || ret != CMDT_RSP_ACK) {
		dev_err(tcpm, "DisplayPort Status Update failed: %d\n", ret);
		ret = ret < 0 ? ret : -EPROTO;
		goto exit_mode;
	}

	/*
	 * An ACK is not a status VDO. A partner may acknowledge with a header
	 * and nothing after it, and rx[1] would then be whatever was on the
	 * stack -- which this code would decode into a connection direction
	 * and a multi-function preference, and then transmit back to the
	 * partner inside DP_Configure.
	 */
	if (rx_cnt < 2) {
		dev_err(tcpm, "DisplayPort Status Update returned no status\n");
		ret = -EPROTO;
		goto exit_mode;
	}

	dp->status = rx[1];
	con = DP_STATUS_CONNECTION(dp->status);

	if (con == DP_STATUS_CON_DISABLED) {
		dev_err(tcpm, "partner reports no DisplayPort connection\n");
		ret = -ENOTCONN;
		goto exit_mode;
	}

	pin = typec_dp_pin_assign(dp, con);
	if (!pin) {
		dev_err(tcpm, "no pin assignment in common (port %08x, partner %08x)\n",
			dp->port_vdo, dp->partner_vdo);
		ret = -EINVAL;
		goto exit_mode;
	}

	/*
	 * The select-configuration field says what the UFP_U -- the partner --
	 * is to become, which reads backwards until you notice it describes
	 * the far end of the cable. Driving a monitor, we are the DFP_D and it
	 * becomes the UFP_D.
	 */
	conf = (con == DP_STATUS_CON_DFP_D) ? DP_CONF_UFP_U_AS_DFP_D
					    : DP_CONF_UFP_U_AS_UFP_D;
	conf |= (DP_CAP_DP_SIGNALLING(dp->port_vdo) &
		 DP_CAP_DP_SIGNALLING(dp->partner_vdo)) <<
		DP_CONF_SIGNALLING_SHIFT;
	conf |= DP_CONF_SET_PIN_ASSIGN(pin);

	vdo[0] = tcpm_svdm_build_header(USB_TYPEC_DP_SID, dp->mode_pos,
					DP_CMD_CONFIGURE);
	vdo[1] = conf;
	ret = tcpm_vdm_exchange(tcpm, vdo, 2, NULL, 0, NULL,
				PD_T_VDM_SNDR_RSP * 10);
	if (ret < 0 || ret != CMDT_RSP_ACK) {
		dev_err(tcpm, "DisplayPort Configure refused: %d\n", ret);
		ret = ret < 0 ? ret : -EPROTO;
		goto exit_mode;
	}

	dp->conf = conf;

	return 0;

exit_mode:
	/*
	 * We told the partner to enter DisplayPort and then could not finish.
	 * Take it back out, or it stays there -- nothing on this side
	 * remembers across a reboot, so the only other way back is unplugging
	 * the cable.
	 */
	typec_dp_exit(tcpm, dp);

	return ret;
}

int typec_dp_autoenter(struct udevice *tcpm)
{
	struct typec_dp_state *dp = tcpm_dp_state(tcpm);
	u32 vdo;
	int ret;

	/*
	 * The board's declaration is the whole policy. typec_dp_read_port_vdo()
	 * already distinguishes "this connector has an altmodes/displayport
	 * node" from "it has not", and that distinction is exactly the question
	 * being asked here -- a connector with the node is a DisplayPort source
	 * receptacle and a connector without one is a USB port that must not be
	 * sent an Enter Mode. Re-deriving that from anything else would be a
	 * second answer to a question the device tree already answers.
	 *
	 * Pass the errno through rather than flattening it to -ENOENT, which
	 * this used to do and which threw away the distinction the comment
	 * above claims to rely on. -ENOENT means "no altmodes/displayport node"
	 * and is the quiet, expected answer for a plain USB port; every other
	 * error means the node IS there and something about it is wrong -- most
	 * usefully a missing or malformed `vdo`. Collapsing them made a board
	 * that declared DisplayPort and fluffed one property produce no
	 * diagnostic at all and a blank monitor, which is precisely the failure
	 * this path exists to eliminate.
	 */

	ret = typec_dp_read_port_vdo(tcpm, &vdo);
	if (ret)
		return ret;

	/*
	 * Already in it. Re-entering costs a NAK and then a speculative Exit
	 * (the "assuming already entered" arm of typec_dp_enter()), which would
	 * take a working link back DOWN -- the one outcome worse than not
	 * trying.
	 *
	 * -EALREADY rather than 0, so the caller can tell "I entered the mode"
	 * from "it was entered before I got here" and not log the first when it
	 * means the second. Both are success; only one is news.
	 */
	if (dp->active)
		return -EALREADY;

	return typec_dp_enter(tcpm, dp);
}
