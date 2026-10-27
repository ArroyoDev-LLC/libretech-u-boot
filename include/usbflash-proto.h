/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The flash-session line protocol (amlogic/software/boot/spl-usb-flasher-plan.md
 * section 4), parsed in C by both layers that serve a session: the SPL
 * session (common/spl/spl_usbflash.c) and U-Boot proper's `usbflash` command
 * (board/libre-computer/common/board-meson.c).  One parser, so the two layers
 * accept and refuse the same bytes.
 *
 * A line is ASCII, single spaces between words, one optional trailing "\n" or
 * "\r\n":
 *   usbflash ums mmc <n>       UMS of mmc <n>'s user area, read-write
 *   usbflash ums mmc <n> ro    the same, read-only
 *   usbflash dfu               DFU again
 *   usbflash boot              leave the session, continue the boot
 *   usbflash boot kernel       the same, and U-Boot proper runs `bootm` on the
 *                              FIT the host loaded into the `kernel` alt
 *   usbflash boot prompt       the same, and U-Boot proper does not autoboot:
 *                              it waits at its prompt for the host's commands
 *                              (on the USB console, USB_BOOT_ACM_CONSOLE)
 *   usbflash reset             reset the board
 * <n> is one decimal digit, U-Boot's mmc numbering (0 = eMMC, 1 = SD).
 * Anything else is refused.  Which accepted lines a layer can carry out is
 * the layer's own rule (the SPL has no write path, so it refuses a
 * read-write UMS).
 */

#ifndef __USBFLASH_PROTO_H__
#define __USBFLASH_PROTO_H__

#include <errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/types.h>

#define USBFLASH_LINE_MAX	64

/*
 * The SPL's mark in U-Boot proper's control DTB that it served this boot's
 * session (an empty property of /chosen; plan section 5).
 */
#define USBFLASH_FDT_PROP	"u-boot,spl-usbflash"
/*
 * After `usbflash boot kernel`: the address of the `kernel` alt (a u64), for
 * U-Boot proper to `bootm` (common/usbflash_bootm.c).
 */
#define USBFLASH_FDT_BOOTM	"u-boot,spl-usbflash-bootm"
/*
 * After `usbflash boot prompt`: an empty property, for U-Boot proper to skip
 * its autoboot (common/usbflash_bootm.c).
 */
#define USBFLASH_FDT_PROMPT	"u-boot,spl-usbflash-prompt"

/* SPL: set USBFLASH_FDT_PROP in @blob if the session ran (spl_usbflash.c) */
void spl_usbflash_fdt_fixup(void *blob);

enum usbflash_verb {
	USBFLASH_UMS,
	USBFLASH_DFU,
	USBFLASH_BOOT,
	USBFLASH_BOOT_KERNEL,
	USBFLASH_BOOT_PROMPT,
	USBFLASH_RESET,
};

struct usbflash_cmd {
	enum usbflash_verb verb;
	int mmc;		/* USBFLASH_UMS only */
	bool ro;		/* USBFLASH_UMS only */
};

/* Is the word at @p (length @n) exactly @w? */
static inline bool usbflash_word(const char *p, size_t n, const char *w)
{
	return n == strlen(w) && !memcmp(p, w, n);
}

/*
 * usbflash_parse() - parse one protocol line.
 * @buf: the bytes (need not be NUL-terminated)
 * @len: their count
 * @cmd: filled on success
 * Return: 0, or -EINVAL for anything that is not exactly one protocol line.
 */
static inline int usbflash_parse(const char *buf, size_t len,
				       struct usbflash_cmd *cmd)
{
	const char *w[6];
	size_t wl[6];
	size_t i, n = 0, start = 0;

	if (len && buf[len - 1] == '\n')
		len--;
	if (len && buf[len - 1] == '\r')
		len--;
	if (!len || len >= USBFLASH_LINE_MAX)
		return -EINVAL;

	for (i = 0; i <= len; i++) {
		if (i < len && buf[i] != ' ') {
			if (buf[i] < 0x21 || buf[i] > 0x7e)
				return -EINVAL;	/* printable ASCII only */
			continue;
		}
		if (i == start || n == ARRAY_SIZE(w))
			return -EINVAL;		/* empty word, or too many */
		w[n] = buf + start;
		wl[n] = i - start;
		n++;
		start = i + 1;
	}

	if (n < 2 || !usbflash_word(w[0], wl[0], "usbflash"))
		return -EINVAL;

	if (usbflash_word(w[1], wl[1], "ums")) {
		if ((n != 4 && n != 5) || !usbflash_word(w[2], wl[2], "mmc") ||
		    wl[3] != 1 || w[3][0] < '0' || w[3][0] > '9')
			return -EINVAL;
		if (n == 5 && !usbflash_word(w[4], wl[4], "ro"))
			return -EINVAL;
		cmd->verb = USBFLASH_UMS;
		cmd->mmc = w[3][0] - '0';
		cmd->ro = n == 5;
		return 0;
	}
	if (n == 3 && usbflash_word(w[1], wl[1], "boot") &&
	    usbflash_word(w[2], wl[2], "kernel")) {
		cmd->verb = USBFLASH_BOOT_KERNEL;
		return 0;
	}
	if (n == 3 && usbflash_word(w[1], wl[1], "boot") &&
	    usbflash_word(w[2], wl[2], "prompt")) {
		cmd->verb = USBFLASH_BOOT_PROMPT;
		return 0;
	}
	if (n != 2)
		return -EINVAL;
	if (usbflash_word(w[1], wl[1], "dfu"))
		cmd->verb = USBFLASH_DFU;
	else if (usbflash_word(w[1], wl[1], "boot"))
		cmd->verb = USBFLASH_BOOT;
	else if (usbflash_word(w[1], wl[1], "reset"))
		cmd->verb = USBFLASH_RESET;
	else
		return -EINVAL;

	return 0;
}

#endif /* __USBFLASH_PROTO_H__ */
