// SPDX-License-Identifier: GPL-2.0+
/*
 * bootrom - reset to the SoC BootROM's USB download mode.
 *
 * Not `reset` plus the boot order: a plain reset reaches the BootROM's USB
 * mode only when no medium boots first. This arms the family's own
 * mechanism (an override word, a boot-mode flag), reads it back, and resets
 * through SYSRESET. One parseable line, `bootrom: <word>`:
 *
 *   forced        armed and read back; the board is resetting
 *   unsupported   no mechanism, no reset device, or the unit refuses it;
 *                 nothing changed
 *   reset-failed  armed, but the reset returned; the arm is undone
 *
 * The first stage of ours a host then loads over USB clears the request, so
 * it is spent by one boot.
 *
 * Policy: claude design/rules/bootloader/policy.md section 6 "Reset to USB boot
 * mode (bootrom)"; design design/rules/bootloader/reset-to-usb-boot.md R1.
 */

#include <bootrom.h>
#include <command.h>
#include <console.h>
#include <dm.h>
#include <g_dnl.h>
#include <hang.h>
#include <stdio.h>
#include <linux/string.h>
#include <sysreset.h>
#include <dm/device-internal.h>
#include <dm/root.h>
#include <linux/delay.h>
#include <linux/errno.h>

int __weak bootrom_arch_check(void)
{
	return -ENOSYS;
}

int __weak bootrom_arch_arm(void)
{
	return -ENOSYS;
}

void __weak bootrom_arch_disarm(void)
{
}

const char * __weak bootrom_arch_reason(void)
{
	return NULL;
}

int bootrom_check(void)
{
	struct udevice *dev;
	int ret;

	ret = bootrom_arch_check();
	if (ret)
		return ret;
	/* Armed with nothing to reset through would hang armed. */
	if (uclass_first_device_err(UCLASS_SYSRESET, &dev))
		return -ENODEV;
	return 0;
}

int bootrom_run(void)
{
	int ret;

	/* The word first, so a host parses it; the reason after it. */
	ret = bootrom_check();
	if (ret) {
		const char *why = bootrom_arch_reason();

		printf("bootrom: unsupported (%s, %d)\n",
		       ret == -ENODEV ? "no reset device" : why ? why : "no mechanism", ret);
		return ret;
	}

	ret = bootrom_arch_arm();
	if (ret) {
		bootrom_arch_disarm();
		printf("bootrom: unsupported (not armed on read-back, %d)\n", ret);
		return ret;
	}

	printf("bootrom: forced\n");
	/* The word a host reads first: out of every console before the reset. */
	if (IS_ENABLED(CONFIG_CONSOLE_FLUSH_SUPPORT))
		flush();
	mdelay(100);

	/* As do_reset: cards and rails back in their power-on state. */
	dm_remove_devices_active();

	/*
	 * sysreset_walk(), not sysreset_walk_halt(): the latter hang()s when
	 * the reset does not happen, which would leave the mechanism armed.
	 */
	ret = sysreset_walk(SYSRESET_COLD);
	if (ret == -EINPROGRESS)
		mdelay(100);

	bootrom_arch_disarm();
	printf("bootrom: reset-failed\n");
	return -EIO;
}

/* The USB request (design R4) and the wait U-Boot is in, if any. */
static bool bootrom_pending;
static enum bootrom_wait bootrom_waiting;

void bootrom_request(void)
{
	bootrom_pending = true;
}

bool bootrom_requested(void)
{
	return bootrom_pending;
}

__weak void board_wait_for_person(enum bootrom_wait w)
{
}

enum bootrom_wait bootrom_wait_set(enum bootrom_wait w)
{
	enum bootrom_wait prev = bootrom_waiting;

	bootrom_waiting = w;
	if (w != BOOTROM_WAIT_NONE)
		board_wait_for_person(w);
	return prev;
}

enum bootrom_wait bootrom_wait_get(void)
{
	return bootrom_waiting;
}

void bootrom_act(void)
{
	if (!bootrom_pending)
		return;
	bootrom_pending = false;
	printf("\nbootrom: USB request\n");
	bootrom_run();
}

void bootrom_act_handoff(void)
{
	if (!bootrom_pending)
		return;
	bootrom_pending = false;
	printf("\nbootrom: USB request, instead of the OS\n");
	bootrom_run();
	/*
	 * Not into the OS the request replaced.  An `unsupported` here would
	 * have been refused at the DETACH; a reset that returned has already
	 * quiesced the devices, so there is no console to go back to.
	 */
	hang();
}

static int do_bootrom(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	if (argc > 2 || (argc == 2 && strcmp(argv[1], "-d")))
		return CMD_RET_USAGE;
	/*
	 * -d: a flash session's `usbflash bootrom`, run after its dfu returned.
	 * Only when that dfu ended with the host's DETACH: one that timed out
	 * means the host is gone, and an armed BootROM would wait for it with
	 * no timeout.  Dropped, the boot continues.
	 */
	if (argc == 2) {
		bool detached = false;

#if CONFIG_IS_ENABLED(DFU_OVER_USB)
		detached = usb_dnl_last_detached();
#endif
		if (!detached) {
			printf("bootrom: dropped (the session did not end with a DETACH)\n");
			return CMD_RET_SUCCESS;
		}
	}
	return bootrom_run() ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(bootrom, 2, 0, do_bootrom,
	   "reset to the SoC BootROM's USB download mode",
	   "[-d]\n"
	   "    - arm the BootROM's USB mode, then reset; prints\n"
	   "      'bootrom: forced', 'bootrom: unsupported' (nothing changed)\n"
	   "      or 'bootrom: reset-failed' (the arm is undone)\n"
	   "    -d: only if the last DFU session ended with the host's DETACH\n"
	   "      (a flash session's `usbflash bootrom`); otherwise dropped");
