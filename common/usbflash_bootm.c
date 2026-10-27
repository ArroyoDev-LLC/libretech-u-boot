// SPDX-License-Identifier: GPL-2.0+
/*
 * U-Boot proper's half of `usbflash boot kernel` and `usbflash boot prompt`
 * (include/usbflash-proto.h).  For `boot prompt` the SPL marks
 * /chosen/u-boot,spl-usbflash-prompt, and here the autoboot is turned off
 * (bootdelay -1), so the host reaches the prompt on the USB console.
 *
 * The SPL's USB flash session ends on that line with the host's FIT still in
 * the session's `kernel` alt, and marks the control DTB it hands on with that
 * address (/chosen/u-boot,spl-usbflash-bootm, spl_usbflash_fdt_fixup()).
 * Here, before the command line starts, the mark makes the boot command
 * `bootm` that FIT, with no autoboot delay.  The FIT carries the kernel and
 * the initramfs only, the same for every board; the device tree Linux gets
 * is U-Boot's own control DT (${fdtcontroladdr}), which policy keeps in sync
 * with Linux.  The same for every family that runs the session (Meson,
 * Rockchip); the DTB comes with the boot, so the mark cannot outlive it.
 */

#include <env.h>
#include <event.h>
#include <stdio.h>
#include <usb_otg_role.h>
#include <usbflash-proto.h>
#include <dm/ofnode.h>

static int usbflash_bootm(void)
{
	ofnode chosen = ofnode_path("/chosen");
	char cmd[64];
	u64 addr;

	/*
	 * `usbflash boot prompt`: the host stays on the line to drive U-Boot
	 * (its USB console), so there is no autoboot to race it to.
	 */
	if (ofnode_read_bool(chosen, USBFLASH_FDT_PROMPT)) {
		env_set("bootdelay", "-1");
		printf("usbflash: boot prompt: no autoboot\n");
		return 0;
	}
	if (ofnode_read_u64(chosen, USBFLASH_FDT_BOOTM, &addr))
		return 0;
	/* kernel and initramfs from the FIT, the device tree U-Boot runs on */
	snprintf(cmd, sizeof(cmd), "bootm 0x%llx 0x%llx ${fdtcontroladdr}",
		 (unsigned long long)addr, (unsigned long long)addr);
	env_set("bootcmd", cmd);
	env_set("bootdelay", "0");
	/*
	 * The host that sent the FIT is on the OTG port and the kernel's
	 * console is a gadget on it: the port stays a device unless the
	 * environment already chose (boot/usb_otg_role.c, applied by the
	 * board's ft_board_setup()).
	 */
	if (IS_ENABLED(CONFIG_USB_OTG_ROLE) && !env_get(USB_OTG_ENV))
		env_set(USB_OTG_ENV, "device");
	printf("usbflash: boot kernel: %s\n", cmd);

	return 0;
}
EVENT_SPY_SIMPLE(EVT_MAIN_LOOP, usbflash_bootm);
