/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Rockchip Type-C PHY -- the negotiated DisplayPort link, for consumers.
 *
 * Modelled on Linux's include/linux/phy/rockchip-typec-phy.h, which exists for
 * the same consumer: cdn-dp needs the lane count and the plug orientation of a
 * link that only the Type-C port manager knows about.
 *
 * ONE deliberate divergence from Linux, and it is in the error contract rather
 * than the data. Linux's struct carries a @dp_connected flag and returns 0 with
 * lanes == 0 when the port is not in DisplayPort, so a caller that forgets to
 * test the flag hands a zero lane count to the DP firmware and gets a link that
 * fails for a reason nothing reports. Here "not in DisplayPort" is a negative
 * errno and there is no flag to forget.
 */

#ifndef __PHY_ROCKCHIP_TYPEC_H
#define __PHY_ROCKCHIP_TYPEC_H

#include <linux/errno.h>
#include <linux/types.h>

struct phy;

/**
 * struct rockchip_typec_phy_dp_state - the DisplayPort link on one connector
 * @flip: the plug is reversed (CC2), so the DP lanes are mirrored
 * @lanes: DisplayPort lanes carried by this connector -- 4 or 2
 *
 * Purely an OUTPUT. Only ever filled in on success, so there is no
 * initialisation contract for a caller to get wrong, and a caller that ignores
 * the return value reads its own uninitialised stack rather than a plausible
 * zero.
 */
struct rockchip_typec_phy_dp_state {
	bool flip;
	u8 lanes;
};

#if IS_ENABLED(CONFIG_PHY_ROCKCHIP_TYPEC)

/**
 * rockchip_typec_phy_get_dp_state() - the DP link this PHY is muxed for
 * @phy: the "dp-port" phy of a rockchip,rk3399-typec-phy
 * @state: filled in on success only
 *
 * Return: 0, or a negative errno -- in particular -ENODEV when the connector
 * is not in a DisplayPort alt mode that this PHY can carry, which is a real
 * answer and not an error to paper over.
 */
int rockchip_typec_phy_get_dp_state(struct phy *phy,
				    struct rockchip_typec_phy_dp_state *state);

#else

static inline int
rockchip_typec_phy_get_dp_state(struct phy *phy,
				struct rockchip_typec_phy_dp_state *state)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_PHY_ROCKCHIP_TYPEC */

#endif /* __PHY_ROCKCHIP_TYPEC_H */
