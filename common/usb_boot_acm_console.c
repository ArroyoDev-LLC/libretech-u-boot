// SPDX-License-Identifier: GPL-2.0+
/*
 * A USB boot has a host on the download port: give it the console.
 *
 * When the published boot source (bootsource_get(), boot/bootsource.c) is
 * USB, the usbacm stdio device (drivers/usb/gadget/f_acm.c) is started once
 * after preboot and added to stdin, stdout and stderr beside whatever they
 * already name, so the host that booted the board over USB reads and drives
 * the console with no serial cable.  After preboot, not at the end of init:
 * U-Boot services the gadget only while it polls the console, and preboot's
 * work (`usb start` takes seconds) would leave a just-configured gadget
 * unanswered; a host request in that window (GET_DESCRIPTOR as the host opens
 * the port) wedged EP0 for the rest of the boot on the S905D3-CC.  Started
 * once, a host that never configures the gadget costs one
 * USB_FUNCTION_ACM_CONNECT_TIMEOUT_MS rather than one per stream.  With
 * CONSOLE_RECORD the record of what U-Boot printed since relocation, preboot's
 * output included, goes out first, so the host reads the boot in order; f_acm
 * holds it until the host raises DTR.
 *
 * Any SoC whose board code publishes its boot source can use it (MediaTek's
 * BL2 tag, the Rockchip BootROM's record).
 */

#include <asm/global_data.h>
#include <bootsource.h>
#include <env.h>
#include <event.h>
#include <membuf.h>
#include <stdio.h>
#include <stdio_dev.h>
#include <linux/kernel.h>
#include <linux/string.h>

DECLARE_GLOBAL_DATA_PTR;

static int usb_boot_acm_console(void)
{
	static const char *const streams[] = { "stdin", "stdout", "stderr" };
	struct stdio_dev *acm;
	char buf[64];
	int i, ret;

	if (bootsource_get() != BOOTSOURCE_USB)
		return 0;

	acm = stdio_get_by_name("usbacm");
	if (!acm || !acm->start)
		return 0;
	ret = acm->start(acm);
	if (ret) {
		printf("USB console: no host took the ACM port (%d)\n", ret);
		return 0;
	}

#if IS_ENABLED(CONFIG_CONSOLE_RECORD)
	if (gd->flags & GD_FLG_RECORD) {
		char *data;
		int n;

		/* membuf_getraw() hands out the contiguous part; loop over a wrap. */
		while ((n = membuf_getraw((struct membuf *)&gd->console_out,
					  4096, true, &data)) > 0) {
			char chunk[257];

			for (i = 0; i < n; i += 256) {
				int k = min(256, n - i);

				memcpy(chunk, data + i, k);
				chunk[k] = '\0';
				acm->puts(acm, chunk);
			}
		}
	}
#endif

	for (i = 0; i < ARRAY_SIZE(streams); i++) {
		const char *cur = env_get(streams[i]) ?: "serial";

		if (strstr(cur, "usbacm"))
			continue;
		snprintf(buf, sizeof(buf), "%s,usbacm", cur);
		env_set(streams[i], buf);
	}
	printf("USB console: usbacm beside the UART\n");

	return 0;
}
EVENT_SPY_SIMPLE(EVT_POST_PREBOOT, usb_boot_acm_console);
