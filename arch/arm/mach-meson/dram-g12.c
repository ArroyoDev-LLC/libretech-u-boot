// SPDX-License-Identifier: GPL-2.0+
/*
 * G12B/SM1 Strategy B Phase 1: open DDR PLL lock (no PHY train).
 *
 * Register map is g12a-ddr-fw.h (G12-T1) — do not fork dram-g12.h.
 * Sequence transcribed from aml_ddr FUN_fffd504c (g12b-ddr-init-analysis.md).
 * PLL lock bit is AM_DDR_PLL_CNTL0[31] (same header).  This file is linked
 * only when CONFIG_MESON_G12_OPEN_PLL=y; SM1 residual-opensc keeps dram-g12a.c.
 */

#include <init.h>
#include <config.h>
#include <asm/io.h>
#include <asm/arch/g12a-ddr-fw.h>
#include <linux/types.h>

static void g12_spin(ulong n)
{
	while (n--)
		asm volatile("nop");
}

/* AO UART — BROM left it up.  Same addresses as spl.c G12A early_puts. */
static void g12_uart_putc(char c)
{
	int timeout = 100000;

	while ((readl((void *)0xff80300cUL) & (1 << 21)) && --timeout)
		;
	if (timeout)
		writel(c, (void *)0xff803000UL);
}

static void g12_uart_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			g12_uart_putc('\r');
		g12_uart_putc(*s++);
	}
}

static void g12_uart_puthex(u32 v)
{
	int i;

	for (i = 28; i >= 0; i -= 4)
		g12_uart_putc("0123456789abcdef"[(v >> i) & 0xf]);
}

/**
 * meson_g12_ddr_pll_init - FUN_fffd504c for LPDDR4 1608 MT/s
 *
 * CNTL1..6 defaults when ACS PLL block empty, then
 * CNTL0 = pll_val | 0x1c000000 → delay → | 0x3c000000 → delay → | 0x1c000000,
 * poll bit 31.  Returns 0 on lock, -1 on failure.
 */
int meson_g12_ddr_pll_init(void)
{
	u32 pll_val = G12A_DDR_PLL_VAL_1608;
	int retries = 100;
	u32 sts;

	writel(readl((void *)G12A_WATCHDOG_CNTL) & G12A_WATCHDOG_CNTL_CLR,
	       (void *)G12A_WATCHDOG_CNTL);

	writel(G12A_DDR_PLL_CNTL6_DEF, (void *)G12A_DDR_PLL_CNTL6);
	writel(G12A_DDR_PLL_CNTL5_DEF, (void *)G12A_DDR_PLL_CNTL5);
	writel(G12A_DDR_PLL_CNTL4_DEF, (void *)G12A_DDR_PLL_CNTL4);
	writel(G12A_DDR_PLL_CNTL3_DEF, (void *)G12A_DDR_PLL_CNTL3);
	writel(G12A_DDR_PLL_CNTL2_DEF, (void *)G12A_DDR_PLL_CNTL2);
	writel(G12A_DDR_PLL_CNTL1_DEF, (void *)G12A_DDR_PLL_CNTL1);
	g12_spin(1000);

	do {
		writel(pll_val | G12A_DDR_PLL_RST_ASSERT,
		       (void *)G12A_DDR_PLL_CNTL0);
		g12_spin(10000);
		writel(pll_val | G12A_DDR_PLL_RST_RELEASE,
		       (void *)G12A_DDR_PLL_CNTL0);
		g12_spin(10000);
		writel(pll_val | G12A_DDR_PLL_RST_ASSERT,
		       (void *)G12A_DDR_PLL_CNTL0);
		g12_spin(10000);

		sts = readl((void *)G12A_DDR_PLL_CNTL0);
		if (sts & G12A_DDR_PLL_LOCK_BIT)
			return 0;
	} while (--retries > 0);

	return -1;
}

/*
 * Phase 1 terminal: print CNTL0 (lock-status register) then hang.
 * Do not claim DRAM training.  spl.c must not proceed to FIT/spl_init
 * without DRAM.
 */
int dram_init(void)
{
	int locked;
	u32 sts;

	g12_uart_puts("P\n");
	locked = meson_g12_ddr_pll_init();
	sts = readl((void *)G12A_DDR_PLL_CNTL0);
	g12_uart_puts("CNTL0 ");
	g12_uart_puthex(sts);
	if (locked == 0 && (sts & G12A_DDR_PLL_LOCK_BIT))
		g12_uart_puts(" L\n");
	else
		g12_uart_puts(" F\n");

	for (;;)
		;
	return locked;
}
