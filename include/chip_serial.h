/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The SoC's unique id as the bootloader's serial string: the one string that
 * serial#, SMBIOS Type 1, the devicetree's /serial-number and every USB
 * gadget's iSerialNumber carry (claude docs repo
 * design/rules/bootloader/policy.md "Network MAC & serial number").
 *
 * It is read from the hardware in every phase that has a source for it, TPL
 * onwards; nothing stored on the board (a saved or imported environment)
 * feeds it.
 */

#ifndef __CHIP_SERIAL_H
#define __CHIP_SERIAL_H

#include <linux/types.h>

/* 32 hex digits and the NUL: Rockchip needs 32, Amlogic 24, the other vendors 16 */
#define CHIP_SERIAL_MAX		33

/**
 * chip_serial_get() - the chip id as lowercase hex, read from the hardware
 *
 * Implemented per SoC family; the weak default has no source.
 *
 * @buf:	output, NUL-terminated
 * @size:	size of @buf (CHIP_SERIAL_MAX is always enough)
 * Return:	0, -ENOSYS if this phase has no source, or another -errno if
 *		the read failed or the chip is unburned
 */
int chip_serial_get(char *buf, size_t size);

/**
 * chip_serial_hex() - lowercase hex of @len bytes, no separators
 *
 * @buf:	output, NUL-terminated
 * @size:	size of @buf, at least 2 * @len + 1
 * @bytes:	the bytes, most significant first as printed
 * @len:	number of bytes
 * Return:	0, or -ENOSPC if @buf is too small
 */
int chip_serial_hex(char *buf, size_t size, const u8 *bytes, size_t len);

/**
 * chip_serial_announce() - print "Chip id: <hex>" on the console
 *
 * Silent when this phase has no source; says "unreadable" when the read
 * failed, which is the policy's FAIL signal.
 */
void chip_serial_announce(void);

/**
 * chip_serial_env() - set serial# to this chip's id (U-Boot proper)
 *
 * Runs after the environment is loaded and before anything reads serial#
 * (SMBIOS, the USB gadgets, the devicetree fixup).  A serial# already present
 * (a saved environment from another board, an imported config) is replaced
 * past its write-once flag.  When the chip id cannot be read, a present
 * serial# is deleted rather than left claiming another board's identity.
 *
 * Return:	0, or the chip_serial_get() error
 */
int chip_serial_env(void);

/**
 * chip_serial_ethaddr() - ethaddr and eth1addr from serial# (U-Boot proper)
 *
 * For a SoC with no programmed MAC: SHA-256 of the serial# string, its first
 * six bytes, unicast and locally administered; eth1addr is that address with
 * the last byte's low bit flipped. A stored ethaddr, the operator's override,
 * is kept. Run after chip_serial_env().
 *
 * Return:	0, -ENODATA with no serial#, -ENOSYS without HASH/SHA256/NET
 */
int chip_serial_ethaddr(void);

#endif /* __CHIP_SERIAL_H */
