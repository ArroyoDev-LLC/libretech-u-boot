/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic GX TPL -> SPL boot log: every byte the chain writes to the AO UART,
 * kept in memory so the USB console can show it.
 *
 * The UART is the only console the TPL and the early SPL have, and a board
 * booted over USB from a phone has nothing attached to it.  The log is a copy
 * of the UART transcript, taken at the byte that goes into the UART FIFO (the
 * debug UART, the serial driver and spl.c's early_putc()), so it holds exactly
 * what a serial console would have shown, CR/LF included.
 *
 * Two homes, one format.  The TPL starts it in the SRAM window
 * (GX_SRAM_BOOTLOG_ADDR, gx-handoff.h) because DRAM is not trained yet; once
 * dram_init() succeeds it moves the log to CONFIG_MESON_GX_BOOTLOG_ADDR and
 * leaves the SRAM header saying so (GX_BOOTLOG_MOVED).  The DRAM stage finds
 * the log through that header and keeps appending.  Its USB flash session
 * replays the log into the CDC ACM console when the console starts, so the
 * host sees the whole boot from the TPL's first line.
 *
 * A host can also read the log directly over the TPL's USB protocol
 * (REQ_READ_MEM) at either address: the header says how many bytes are valid.
 * The log keeps the FIRST bytes of the boot; once full, further bytes are only
 * counted in @lost.
 *
 * Design: amlogic/software/boot/gxl/tpl-spl-chain-design.md, "Boot log".
 */

#ifndef __GX_BOOTLOG_H__
#define __GX_BOOTLOG_H__

#include <linux/types.h>

#define GX_BOOTLOG_LIVE		0x4c425847u	/* "GXBL": this log is being written */
#define GX_BOOTLOG_MOVED	0x4d425847u	/* "GXBM": SRAM only -- moved to DRAM */

/**
 * struct gx_bootlog - boot log header, followed by its data
 * @magic:	GX_BOOTLOG_LIVE, or GX_BOOTLOG_MOVED in the SRAM header once the
 *		log lives in DRAM
 * @len:	bytes of @data that are valid
 * @cap:	bytes @data can hold
 * @lost:	bytes that arrived after @data was full
 * @data:	the UART transcript
 */
struct gx_bootlog {
	u32 magic;
	u32 len;
	u32 cap;
	u32 lost;
	char data[];
};

#if CONFIG_IS_ENABLED(MESON_GX_BOOTLOG)
void gx_bootlog_putc(char c);
void gx_bootlog_start(void);
void gx_bootlog_dram_ready(void);
void gx_bootlog_replay(void (*out)(const char *s, unsigned int len));
#else
static inline void gx_bootlog_putc(char c) {}
static inline void gx_bootlog_start(void) {}
static inline void gx_bootlog_dram_ready(void) {}
static inline void gx_bootlog_replay(void (*out)(const char *s,
						 unsigned int len)) {}
#endif

#endif /* __GX_BOOTLOG_H__ */
