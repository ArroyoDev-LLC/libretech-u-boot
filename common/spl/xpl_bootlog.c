// SPDX-License-Identifier: GPL-2.0+
/*
 * xPL boot log; what it is and why: <xpl_bootlog.h>.
 *
 * No state lives in .bss: the UART is written before crt0 clears it. The TPL
 * keeps its state in .data, which comes with the image; the SPL keeps none
 * and reads the header in DRAM, which only a TPL of this boot can have made
 * live (it rewrites the header when DRAM comes up, whatever was there).
 */

#include <xpl_bootlog.h>
#include <linux/string.h>

#define XPL_BOOTLOG_DRAM	((struct xpl_bootlog *)CONFIG_XPL_BOOTLOG_ADDR)

static void bootlog_append(struct xpl_bootlog *l, char c)
{
	if (l->len < l->cap)
		l->data[l->len++] = c;
	else
		l->lost++;
}

#if defined(CONFIG_TPL_BUILD)
static char tpl_early[CONFIG_TPL_XPL_BOOTLOG_EARLY_SIZE] __section(".data");
static u32 tpl_early_len __section(".data");
static bool tpl_in_dram __section(".data");

void xpl_bootlog_putc(char c)
{
	if (tpl_in_dram)
		bootlog_append(XPL_BOOTLOG_DRAM, c);
	else if (tpl_early_len < sizeof(tpl_early))
		tpl_early[tpl_early_len++] = c;
}

/* TPL, once DRAM init succeeded: a fresh log in DRAM, the bytes so far first */
void xpl_bootlog_dram_ready(void)
{
	struct xpl_bootlog *l = XPL_BOOTLOG_DRAM;

	l->len = tpl_early_len;
	l->lost = 0;
	l->cap = CONFIG_XPL_BOOTLOG_SIZE - sizeof(*l);
	memcpy(l->data, tpl_early, tpl_early_len);
	l->magic = XPL_BOOTLOG_MAGIC;
	tpl_in_dram = true;
}
#else
static struct xpl_bootlog *bootlog_live(void)
{
	struct xpl_bootlog *l = XPL_BOOTLOG_DRAM;

	if (l->magic != XPL_BOOTLOG_MAGIC ||
	    l->cap != CONFIG_XPL_BOOTLOG_SIZE - sizeof(*l) || l->len > l->cap)
		return NULL;

	return l;
}

void xpl_bootlog_putc(char c)
{
	struct xpl_bootlog *l = bootlog_live();

	if (l)
		bootlog_append(l, c);
}

void xpl_bootlog_dram_ready(void)
{
}

/*
 * Hand the log so far to @out, then retire it: a later boot whose TPL keeps
 * no log must not find this one. Bytes printed while @out runs are not
 * included; the caller's console carries them.
 */
void xpl_bootlog_replay(void (*out)(const char *s, unsigned int len))
{
	struct xpl_bootlog *l = bootlog_live();
	unsigned int len;

	if (!l)
		return;
	len = l->len;
	l->magic = 0;
	out(l->data, len);
}
#endif
