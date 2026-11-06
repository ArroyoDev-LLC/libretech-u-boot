/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The Libre Computer stage word: what a gadget of ours is, in its device
 * descriptor's bcdDevice, so a host names the stage from what its OS already
 * cached (sysfs, the IORegistry, Windows' REV_ hardware id, Android's
 * UsbDevice.getVersion()) without asking the device anything.
 *
 * bcdDevice = 0x4c00 | phase << 4 | function
 *
 *   phase     0  our stage on the BootROM's id (the TPL answering in its place)
 *             1  SPL (xPL)
 *             2  U-Boot proper
 *   function  0  the console (CDC ACM, with or without a resident DFU)
 *             1  DFU (the SPL flash session; proper's `dfu` command)
 *             2  mass storage (`ums`)
 *             3  fastboot
 *             4  the BootROM's protocol (the TPL's vendor interface, which
 *                answers the BootROM's identify and download requests)
 *             f  any other gadget
 *
 * The high byte 0x4c ('L') says the low byte is ours; a host reads the word
 * only on our gadget ids, never on another device that carries 0x4cXX. The
 * word names the stage the gadget enumerated in. The table is shared with the
 * Libre Computer app's core (core.LcStage*); the policy is the claude tree's
 * tools/libre-computer-app/matching/policy.md, "The stage word".
 */

#ifndef __LC_STAGE_H
#define __LC_STAGE_H

#define LC_STAGE_MAGIC		0x4c00

#define LC_STAGE_PHASE_ROM	0x0
#define LC_STAGE_PHASE_SPL	0x1
#define LC_STAGE_PHASE_PROPER	0x2

#define LC_STAGE_FN_CONSOLE	0x0
#define LC_STAGE_FN_DFU		0x1
#define LC_STAGE_FN_UMS		0x2
#define LC_STAGE_FN_FASTBOOT	0x3
#define LC_STAGE_FN_ROMPROTO	0x4
#define LC_STAGE_FN_OTHER	0xf

#define LC_STAGE_WORD(phase, fn)	(LC_STAGE_MAGIC | (phase) << 4 | (fn))

#endif /* __LC_STAGE_H */
