/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef __MESON_WDT_RESET_H
#define __MESON_WDT_RESET_H

/* How long a terminal failure waits before the watchdog resets the board */
#define MESON_HANG_RESET_MS	10000

/**
 * meson_wdt_reset_after() - arm the EE watchdog and wait for it
 * @ms: delay before the reset, at most 65535
 *
 * Never returns.  arch/arm/mach-meson/wdt-reset.c.
 */
void meson_wdt_reset_after(unsigned int ms) __noreturn;

#endif
