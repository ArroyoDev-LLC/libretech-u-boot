// SPDX-License-Identifier: GPL-2.0+
/*
 * Is a USB host still there?  g_dnl's cable check, for the Meson DWC2 OTG
 * port in device mode.
 *
 * The weak default answers -EOPNOTSUPP, so a UMS session whose host left
 * without ejecting never ended (f_mass_storage sleep_thread), and the DFU loop
 * had no way to tell at all (amlogic/software/boot/audit/
 * 2026-10-12-tpl-spl-usb-gadget-review.md).  A host sends a start-of-frame
 * every 1 ms (125 us on high speed), and the core counts them in
 * DSTS.SOFFN[21:8]: a counter that has moved and then stands still for
 * MESON_USB_HOST_GONE_MS means the cable is out or the host has gone.
 *
 * Until the counter has been seen moving this answers "connected", so a
 * session waiting for its host to arrive is not cut short; a "gone" answer
 * re-arms that, for the next session.  A host that suspends the port also
 * stops SOFs and reads as gone after the same delay.
 *
 * DWC2 bases: GXL usb@c9100000, G12A usb@ff400000 (meson-gxl.dtsi,
 * meson-g12-common.dtsi).  DSTS is +0x808.
 */

#include <g_dnl.h>
#include <time.h>
#include <asm/io.h>

#if IS_ENABLED(CONFIG_MESON_G12A)
#define MESON_DWC2_BASE		0xff400000UL
#else
#define MESON_DWC2_BASE		0xc9100000UL
#endif
#define DWC2_DSTS		0x808
#define DWC2_DSTS_SOFFN(v)	(((v) >> 8) & 0x3fff)

#define MESON_USB_HOST_GONE_MS	10000

int g_dnl_board_usb_cable_connected(void)
{
	static u32 last_fn = ~0u;
	static ulong since;
	static bool seen;
	u32 fn = DWC2_DSTS_SOFFN(readl((void __iomem *)(MESON_DWC2_BASE +
						      DWC2_DSTS)));

	if (fn != last_fn) {
		if (last_fn != ~0u)
			seen = true;
		last_fn = fn;
		since = get_timer(0);
		return 1;
	}
	if (!seen || get_timer(since) < MESON_USB_HOST_GONE_MS)
		return 1;

	seen = false;
	last_fn = ~0u;
	return 0;
}
