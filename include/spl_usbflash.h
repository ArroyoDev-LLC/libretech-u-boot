/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The SPL USB flash session (common/spl/spl_usbflash.c): DFU with the RAM
 * and storage alternates that answer, the `script` alternate and its line
 * protocol (<usbflash-proto.h>), UMS of an MMC device.  A SoC family runs it
 * from its SPL on a USB boot and supplies what differs per family in a
 * struct spl_usbflash_family.
 */

#ifndef __SPL_USBFLASH_H__
#define __SPL_USBFLASH_H__

#include <linux/types.h>

/**
 * struct spl_usbflash_family - what a SoC family supplies to the session
 * @emmc_alts:	the `;`-separated DFU MMC specs of mmc 0 (the eMMC); specs
 *		naming `mmcpart 1` or `mmcpart 2` are listed only if the part
 *		has boot partitions
 * @sd_alts:	the specs of mmc 1 (the SD slot)
 * @kernel_addr: the RAM alternate `kernel`: start
 * @kernel_size: and length
 * @script_addr: the RAM alternate `script`: start
 * @script_size: and length
 * @timeout_ms:	a DFU session with no request for this long ends
 * @lzma_staging_addr: optional: with SPL_LZMA, a RAM alternate `kernel-lzma`
 *		takes an LZMA stream here (@kernel_size bytes of room) and unpacks
 *		it into `kernel` at the end of the download
 * @probe_more:	optional: add the family's other alternates (spl_usbflash_add())
 *		after the MMC ones
 * @restore:	optional: undo what @probe_more changed, before the boot
 *		continues or the board resets
 * @replay:	optional: write the boot so far (earlier stages' console) to @out,
 *		the ACM console, before the session's first line
 * @reset:	`usbflash reset`: reset the board; does not return
 */
struct spl_usbflash_family {
	const char *emmc_alts;
	const char *sd_alts;
	ulong kernel_addr;
	ulong kernel_size;
	ulong script_addr;
	ulong script_size;
	ulong timeout_ms;
	ulong lzma_staging_addr;
	void (*probe_more)(void);
	void (*restore)(void);
	void (*replay)(void (*out)(const char *s, unsigned int len));
	void (*reset)(void);
};

/**
 * spl_usbflash_session() - run the flash session, then return
 * @fam: the family
 *
 * Returns when the session ends and the boot is to continue (nothing left
 * in `script`, `usbflash boot`, or no alternate list); `usbflash reset` does
 * not return.
 */
void spl_usbflash_session(const struct spl_usbflash_family *fam);

/**
 * spl_usbflash_add() - add one alternate, for a family's @probe_more
 * @interface:	dfu_alt_add() interface ("mmc", "ram", "virt", ...)
 * @devstr:	its device string
 * @spec:	its spec ("<name> <args>")
 * Return: 0, or a negative errno
 */
int spl_usbflash_add(const char *interface, const char *devstr,
		     const char *spec);

#endif /* __SPL_USBFLASH_H__ */
