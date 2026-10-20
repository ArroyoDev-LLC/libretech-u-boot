/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Rockchip IO voltage domain -- runtime hook for drivers that move a rail
 * the domain follows (an SD host switching vqmmc between 3.3 V and 1.8 V).
 */

#ifndef _ASM_ARCH_IO_DOMAIN_H
#define _ASM_ARCH_IO_DOMAIN_H

struct udevice;

/**
 * rockchip_iodomain_set_supply() - Re-program the IO domain(s) fed by a rail
 * @supply:	The regulator device the domain lists as its supply
 * @uV:		The voltage the rail is at (or is about to be at)
 *
 * The io-domain driver configures each pad bank once at probe from the
 * regulator's value; it has no notifier, so a driver that changes the rail
 * afterwards must call this. Mirror Linux's ordering: when the rail RISES,
 * call this with the new voltage BEFORE the regulator changes; when it
 * FALLS, call it AFTER -- so the pad bank is never in 1.8 V mode while the
 * rail is above it.
 *
 * Return: 0 on success, -ENOENT if no io-domain lists @supply, or -ve error
 */
#if CONFIG_IS_ENABLED(ROCKCHIP_IODOMAIN)
int rockchip_iodomain_set_supply(struct udevice *supply, int uV);
#else
static inline int rockchip_iodomain_set_supply(struct udevice *supply, int uV)
{
	return -ENOSYS;
}
#endif

#endif
