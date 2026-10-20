// SPDX-License-Identifier: GPL-2.0+
/*
 * A terminal failure in a Meson first stage resets the board instead of
 * spinning: the EE watchdog fires after a delay, the BootROM runs again and,
 * with no bootable medium (or the USB-boot key held), offers USB download.
 * A bare spin left a board with no console wedged until its power was cut
 * (amlogic/software/boot/audit/2026-10-12-tpl-spl-usb-gadget-review.md).
 *
 * EE watchdog, meson_gxbb_wdt layout: CTRL +0x0, TCNT +0x8 (16-bit count),
 * RSET +0xc; the divider gives a 1 ms tick from the 24 MHz crystal.
 *   GXL   0xc11098d0  (arch/arm/mach-meson/spl.c GXL_WDT_CTRL)
 *   G12A  0xffd0f0d0  (meson-g12-common.dtsi watchdog@f0d0 under cbus
 *                      0xffd00000; meson-usb-g12.c G12_WDT_BASE)
 */

#include <hang.h>
#include <spl.h>
#include <stdio.h>
#include <asm/io.h>
#include <asm/arch/wdt-reset.h>
#include <linux/bitops.h>

#if IS_ENABLED(CONFIG_MESON_G12A)
#define MESON_WDT_BASE		0xffd0f0d0UL
#else
#define MESON_WDT_BASE		0xc11098d0UL
#endif
#define MESON_WDT_CTRL		0x0
#define MESON_WDT_TCNT		0x8
#define MESON_WDT_RSET		0xc
#define MESON_WDT_EN		BIT(18)
#define MESON_WDT_EE_RST	BIT(21)
#define MESON_WDT_CLK_EN	BIT(24)
#define MESON_WDT_CLKDIV_EN	BIT(25)
#define MESON_WDT_DIV_1MS	24000u

void meson_wdt_reset_after(unsigned int ms)
{
	void __iomem *w = (void __iomem *)MESON_WDT_BASE;

	if (ms > 0xffff)
		ms = 0xffff;
	writel(0, w + MESON_WDT_CTRL);
	writel(MESON_WDT_DIV_1MS | MESON_WDT_EE_RST | MESON_WDT_CLK_EN |
	       MESON_WDT_CLKDIV_EN, w + MESON_WDT_CTRL);
	writel(ms, w + MESON_WDT_TCNT);
	writel(0, w + MESON_WDT_RSET);
	writel(readl(w + MESON_WDT_CTRL) | MESON_WDT_EN, w + MESON_WDT_CTRL);
	for (;;)
		;
}

#if IS_ENABLED(CONFIG_XPL_BUILD) && !IS_ENABLED(CONFIG_TPL_BUILD)
/* The DRAM stage found no bootable FIT: reset rather than hang() */
void spl_boot_failed(void)
{
	printf("SPL: nothing to boot; reset in %u ms\n", MESON_HANG_RESET_MS);
	meson_wdt_reset_after(MESON_HANG_RESET_MS);
}
#endif
