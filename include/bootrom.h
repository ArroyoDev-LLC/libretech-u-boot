/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Reset to the SoC BootROM's USB download mode, whatever media is present.
 *
 * Policy: claude design/rules/bootloader/policy.md section 6 "Reset to USB boot
 * mode (bootrom)"; design design/rules/bootloader/reset-to-usb-boot.md R1.
 */

#ifndef __BOOTROM_H
#define __BOOTROM_H

#include <linux/errno.h>
#include <linux/kconfig.h>
#include <linux/types.h>

/**
 * bootrom_arch_check() - does this SoC and build have a mechanism?
 *
 * Return: 0 when bootrom_arch_arm() can force the next boot to USB, -ENOSYS
 * when the family has none (or its word is unconfirmed), another negative
 * errno when the mechanism exists but cannot be used on this unit.
 */
int bootrom_arch_check(void);

/**
 * bootrom_arch_arm() - arm the mechanism and read it back.
 *
 * Return: 0 when the read-back says the next boot is USB, a negative errno
 * otherwise (the caller then disarms).
 */
int bootrom_arch_arm(void);

/** bootrom_arch_disarm() - undo bootrom_arch_arm(). */
void bootrom_arch_disarm(void);

/**
 * bootrom_arch_reason() - why the last bootrom_arch_check() refused, in words
 *
 * Return: a short phrase naming the cause (e.g. the medium that would boot
 * instead), or NULL to let the errno speak.  Weak, NULL by default.
 */
const char *bootrom_arch_reason(void);

/*
 * With CONFIG_CMD_BOOTROM (cmd/bootrom.c):
 *
 * bootrom_check() - what `bootrom` would answer here, without arming.
 *   Return: 0 for "forced", a negative errno for "unsupported".
 *
 * bootrom_run() - check, arm, read back, print, drain, quiesce, reset.
 *   Prints exactly one `bootrom: <word>` line. Returns only when the board did
 *   not reset: -ENOSYS (or the check's errno) for `unsupported`, -EIO for
 *   `reset-failed`, with the mechanism disarmed in both cases.
 */

/*
 * The USB request (design R4): a DETACH to the console gadget's `bootrom` DFU
 * alternate sets a pending flag; U-Boot acts on it only where it waits for a
 * person, or at the last step before an OS -- never inside a running command.
 */
enum bootrom_wait {
	BOOTROM_WAIT_NONE,
	BOOTROM_WAIT_PROMPT,	/* the top-level prompt's readline */
	BOOTROM_WAIT_MENU,
	BOOTROM_WAIT_COUNTDOWN,
	BOOTROM_WAIT_PAUSE,	/* `pause`, confirm_yesno() */
};

/*
 * dfu_resident_hold() - the CLI read a line while a host transfer runs on the
 * console gadget's resident DFU: keep it until the transfer ends (f_dfu.c).
 * Return: 0 to run the line, -EINTR when a Ctrl-C dropped it.
 */
#if CONFIG_IS_ENABLED(USB_FUNCTION_ACM_DFU)
int dfu_resident_hold(void);
#else
static inline int dfu_resident_hold(void)
{
	return 0;
}
#endif

#if CONFIG_IS_ENABLED(CMD_BOOTROM)
int bootrom_check(void);
int bootrom_run(void);
/** bootrom_request() - the USB request arrived (DETACH status stage done) */
void bootrom_request(void);
bool bootrom_requested(void);
/** bootrom_wait_set() - enter (or leave, NONE) a wait; returns the previous */
enum bootrom_wait bootrom_wait_set(enum bootrom_wait w);
/**
 * board_wait_for_person() - a board hook: U-Boot entered wait @w (not NONE).
 *
 * Weak, empty by default.  The G12A/SM1 resident chain clears its boot-hang
 * count at the prompt and the menu (design R11).
 */
void board_wait_for_person(enum bootrom_wait w);
enum bootrom_wait bootrom_wait_get(void);
/** bootrom_act() - at a wait: run bootrom if requested (returns if it did not reset) */
void bootrom_act(void);
/** bootrom_act_handoff() - before an OS: the same, but never returns into the OS */
void bootrom_act_handoff(void);
#else
static inline int bootrom_check(void)
{
	return -ENOSYS;
}

static inline void bootrom_request(void) {}
static inline bool bootrom_requested(void) { return false; }
static inline enum bootrom_wait bootrom_wait_set(enum bootrom_wait w)
{
	return BOOTROM_WAIT_NONE;
}

/* Without the command, nothing gates the resident DFU: always "at the prompt". */
static inline enum bootrom_wait bootrom_wait_get(void)
{
	return BOOTROM_WAIT_PROMPT;
}

static inline void bootrom_act(void) {}
static inline void bootrom_act_handoff(void) {}
#endif

#endif
