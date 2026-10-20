// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX TPL -> SPL boot log.  What it is and why: gx-bootlog.h.
 *
 * Which log a phase writes is held in gd, never in .bss: crt0 clears .bss only
 * after board_init_f(), and the TPL prints from debug_uart_init() on.  gd is
 * set up (and zeroed) before the first C call, so gd->arch.gx_bootlog is NULL
 * until this phase has decided where its log is.
 *
 * The SRAM header is NOT trusted to say where the log is until this boot's TPL
 * wrote it: after a warm reset it still holds the previous boot's
 * GX_BOOTLOG_MOVED, and following that into DRAM before DDR is trained would
 * be a write to an unconfigured controller.  So the TPL always starts a fresh
 * SRAM log in gx_bootlog_start(), and only the DRAM stage -- which runs from
 * DRAM, so DDR is up by construction, and which only this boot's TPL can have
 * reached -- follows the SRAM header to DRAM.
 */

#include <asm/global_data.h>
#include <asm/arch/gx-bootlog.h>
#include <asm/arch/gx-handoff.h>
#include <linux/string.h>

DECLARE_GLOBAL_DATA_PTR;

#define GX_BOOTLOG_SRAM	((struct gx_bootlog *)GX_SRAM_BOOTLOG_ADDR)
#define GX_BOOTLOG_DRAM	((struct gx_bootlog *)CONFIG_MESON_GX_BOOTLOG_ADDR)

static void gx_bootlog_init(struct gx_bootlog *l, unsigned long size)
{
	l->len = 0;
	l->lost = 0;
	l->cap = size - sizeof(*l);
	l->magic = GX_BOOTLOG_LIVE;
}

static struct gx_bootlog *gx_bootlog_get(void)
{
	struct gx_bootlog *l;

	if (!gd)
		return NULL;
	l = gd->arch.gx_bootlog;
	if (l)
		return l;
#if !defined(CONFIG_TPL_BUILD)
	/* The DRAM stage: adopt the log this boot's TPL moved to DRAM */
	if (GX_BOOTLOG_SRAM->magic == GX_BOOTLOG_MOVED &&
	    GX_BOOTLOG_DRAM->magic == GX_BOOTLOG_LIVE &&
	    GX_BOOTLOG_DRAM->cap == CONFIG_MESON_GX_BOOTLOG_SIZE -
				    sizeof(struct gx_bootlog) &&
	    GX_BOOTLOG_DRAM->len <= GX_BOOTLOG_DRAM->cap) {
		gd->arch.gx_bootlog = GX_BOOTLOG_DRAM;
		return GX_BOOTLOG_DRAM;
	}
#endif

	return NULL;
}

/* One byte that went into the UART FIFO */
void gx_bootlog_putc(char c)
{
	struct gx_bootlog *l = gx_bootlog_get();

	if (!l)
		return;
	if (l->len < l->cap)
		l->data[l->len++] = c;
	else
		l->lost++;
}

#if defined(CONFIG_TPL_BUILD)
/* TPL, before its first byte: a fresh log in SRAM, whatever SRAM held */
void gx_bootlog_start(void)
{
	gx_bootlog_init(GX_BOOTLOG_SRAM, GX_SRAM_BOOTLOG_LEN);
	gd->arch.gx_bootlog = GX_BOOTLOG_SRAM;
}

/* TPL, once dram_init() succeeded: move the log to DRAM and say so in SRAM */
void gx_bootlog_dram_ready(void)
{
	struct gx_bootlog *s = GX_BOOTLOG_SRAM, *d = GX_BOOTLOG_DRAM;

	if (gd->arch.gx_bootlog != s)
		return;
	gx_bootlog_init(d, CONFIG_MESON_GX_BOOTLOG_SIZE);
	memcpy(d->data, s->data, s->len);
	d->len = s->len;
	d->lost = s->lost;
	s->magic = GX_BOOTLOG_MOVED;
	gd->arch.gx_bootlog = d;
}
#else
void gx_bootlog_start(void)
{
}

void gx_bootlog_dram_ready(void)
{
}
#endif

/*
 * Hand the log so far to @out, in order.  Bytes printed while @out runs are
 * not included: the caller's own console tee carries them.
 */
void gx_bootlog_replay(void (*out)(const char *s, unsigned int len))
{
	struct gx_bootlog *l = gx_bootlog_get();
	unsigned int len;

	if (!l)
		return;
	len = l->len;
	out(l->data, len);
}
