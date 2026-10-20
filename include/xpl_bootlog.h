/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * xPL boot log: every byte the TPL and SPL write to the UART, kept in DRAM so
 * that a later phase can hand the boot so far to a console that starts late
 * -- the SPL USB flash session's ACM console (struct spl_usbflash_family
 * .replay), on a board with no serial cable attached.
 *
 * The TPL keeps its bytes in its own .data until DRAM is up
 * (xpl_bootlog_dram_ready()), then moves them to XPL_BOOTLOG_ADDR and goes on
 * there; the SPL, which runs with DRAM up, appends to the log this boot's TPL
 * started. Bytes are recorded as the UART sent them (CR/LF included), at the
 * driver: the TPL's banner goes straight to the debug UART.
 */
#ifndef __XPL_BOOTLOG_H
#define __XPL_BOOTLOG_H

#include <linux/kconfig.h>
#include <linux/types.h>

#define XPL_BOOTLOG_MAGIC	0x474f4c42	/* "BLOG" */

struct xpl_bootlog {
	u32 magic;
	u32 len;
	u32 cap;
	u32 lost;
	char data[];
};

#if CONFIG_IS_ENABLED(XPL_BOOTLOG)
void xpl_bootlog_putc(char c);
void xpl_bootlog_dram_ready(void);
void xpl_bootlog_replay(void (*out)(const char *s, unsigned int len));
#else
static inline void xpl_bootlog_putc(char c) {}
static inline void xpl_bootlog_dram_ready(void) {}
#endif

#endif /* __XPL_BOOTLOG_H */
