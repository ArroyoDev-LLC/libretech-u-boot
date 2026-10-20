// SPDX-License-Identifier: GPL-2.0+
/*
 * H5 DRAM parameter sweep
 *
 * When CONFIG_DRAM_SWEEP is enabled, sunxi_dram_init() is replaced
 * by a version that loops through a table of (clk, zq, odt)
 * combinations, re-initializing DRAM each time and running a quick
 * memory test. Results are printed over UART as CSV.
 *
 * After the sweep, calls return_to_fel() so the board re-enters
 * FEL mode without a power cycle.
 *
 * Build:
 *   Add CONFIG_DRAM_SWEEP=y to defconfig (or use a separate sweep defconfig)
 *
 * Run:
 *   sunxi-fel spl spl/sunxi-spl.bin
 *   (results appear on UART, board returns to FEL when done)
 */

#ifdef CONFIG_DRAM_SWEEP

#include <init.h>
#include <log.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include <asm/arch/dram.h>
#include <asm/arch/cpu.h>
#include <linux/delay.h>

/* Functions from dram_sunxi_dw.c (made non-static for sweep) */
extern void mctl_sys_init(uint16_t socid, struct dram_para *para);
extern int mctl_channel_init(uint16_t socid, struct dram_para *para);
extern void mctl_set_cr(uint16_t socid, struct dram_para *para);
extern void mctl_auto_detect_dram_size(uint16_t socid, struct dram_para *para);
extern unsigned long mctl_calc_rank_size(struct rank_para *rank);

/*
 * Sweep parameter table.
 * Each entry is tested in order. Edit this table to add/remove test points.
 */
struct sweep_entry {
	u16 clk;
	u32 zq;
	u8  odt;
};

static const struct sweep_entry sweep_table[] = {
	/* Freq sweep: ODT off, OrangePi ZQ (most likely to work) */
	{ 408, 3881977, 0 },
	{ 480, 3881977, 0 },
	{ 528, 3881977, 0 },
	{ 576, 3881977, 0 },
	{ 600, 3881977, 0 },
	{ 624, 3881977, 0 },
	{ 648, 3881977, 0 },
	{ 672, 3881977, 0 },

	/* Freq sweep: ODT on, OrangePi ZQ */
	{ 408, 3881977, 1 },
	{ 528, 3881977, 1 },
	{ 624, 3881977, 1 },
	{ 672, 3881977, 1 },

	/* Freq sweep: ODT off, default ZQ */
	{ 408, 3881979, 0 },
	{ 528, 3881979, 0 },
	{ 624, 3881979, 0 },
	{ 672, 3881979, 0 },

	/* Freq sweep: ODT on, default ZQ (original failing config) */
	{ 408, 3881979, 1 },
	{ 528, 3881979, 1 },
	{ 624, 3881979, 1 },
	{ 672, 3881979, 1 },

	/* A64 ZQ */
	{ 528, 3881915, 0 },
	{ 672, 3881915, 0 },
	{ 528, 3881915, 1 },
	{ 672, 3881915, 1 },
};

#define SWEEP_COUNT ARRAY_SIZE(sweep_table)

#define SUN8I_H5_DX_READ_DELAYS				\
	{{ 14, 15, 17, 17, 17, 17, 17, 18, 17,  3,  3 },	\
	 { 21, 21, 12, 22, 21, 21, 21, 21, 21,  3,  3 },	\
	 { 16, 19, 19, 17, 22, 22, 21, 22, 19,  3,  3 },	\
	 { 21, 21, 22, 22, 20, 21, 19, 19, 19,  3,  3 } }
#define SUN8I_H5_DX_WRITE_DELAYS				\
	{{  1,  2,  3,  4,  3,  4,  4,  4,  6,  6,  6 },	\
	 {  6,  6,  6,  5,  5,  5,  5,  5,  6,  6,  6 },	\
	 {  0,  2,  4,  2,  6,  5,  5,  5,  6,  6,  6 },	\
	 {  3,  3,  3,  2,  2,  1,  1,  1,  4,  4,  4 } }
#define SUN8I_H5_AC_DELAYS					\
	{  0,  0,  5,  5,  0,  0,  0,  0,			\
	   0,  0,  0,  0,  3,  3,  3,  3,			\
	   3,  3,  3,  3,  3,  3,  3,  3,			\
	   3,  3,  3,  3,  2,  0,  0      }

/* Simple walking-ones memory test -- returns error count */
static u32 memtest_quick(u32 start, u32 size)
{
	volatile u32 *base = (volatile u32 *)start;
	u32 words = size / 4;
	u32 errors = 0;
	u32 i;
	static const u32 patterns[] = {
		0x00000000, 0xFFFFFFFF, 0x55555555, 0xAAAAAAAA,
		0x01234567, 0x89ABCDEF, 0xDEADBEEF, 0xA5A5A5A5,
	};

	for (int p = 0; p < 8; p++) {
		u32 pat = patterns[p];

		/* Write */
		for (i = 0; i < words; i += 256)
			base[i] = pat ^ i;

		/* Verify */
		for (i = 0; i < words; i += 256) {
			if (base[i] != (pat ^ i))
				errors++;
		}
	}

	return errors;
}

/* External: FEL return (from board.c) */
extern struct {
	u32 sp, lr, cpsr, sctlr, vbar, sp_irq;
} fel_stash;
extern void return_to_fel(u32 sp, u32 lr);

unsigned long sunxi_dram_init(void)
{
	struct sunxi_mctl_com_reg * const mctl_com =
			(struct sunxi_mctl_com_reg *)SUNXI_DRAM_COM_BASE;
	struct sunxi_mctl_ctl_reg * const mctl_ctl =
			(struct sunxi_mctl_ctl_reg *)SUNXI_DRAM_CTL0_BASE;
	unsigned long size = 0;
	int i;

	printf("\n\n=== H5 DRAM PARAMETER SWEEP ===\n");
	printf("entries: %d\n", (int)SWEEP_COUNT);
	printf("CLK,ZQ,ODT,INIT,SIZE_MB,ERRORS\n");

	for (i = 0; i < SWEEP_COUNT; i++) {
		const struct sweep_entry *e = &sweep_table[i];
		u32 errors;

		struct dram_para para = {
			.clk = e->clk,
			.zq = e->zq,
			.odt_en = e->odt,
			.dual_rank = 1,
			.bus_full_width = 1,
			.ranks = {
				{ .row_bits = 15, .bank_bits = 3,
				  .page_size = 4096 },
				{ .row_bits = 15, .bank_bits = 3,
				  .page_size = 4096 },
			},
			.dx_read_delays  = SUN8I_H5_DX_READ_DELAYS,
			.dx_write_delays = SUN8I_H5_DX_WRITE_DELAYS,
			.ac_delays       = SUN8I_H5_AC_DELAYS,
		};

		mctl_sys_init(SOCID_H5, &para);

		if (mctl_channel_init(SOCID_H5, &para)) {
			printf("%d,0x%x,%d,FAIL,0,0\n",
			       e->clk, e->zq, e->odt);
			continue;
		}

		/* Post-init config (from sunxi_dram_init) */
		writel(0x00000303, &mctl_ctl->odtmap);
		udelay(1);
		setbits_le32(&mctl_ctl->vtfcr, 3 << 8);
		clrbits_le32(&mctl_ctl->pgcr[2], (1 << 13));
		setbits_le32(&mctl_com->cccr, 1 << 31);
		udelay(10);

		mctl_auto_detect_dram_size(SOCID_H5, &para);
		mctl_set_cr(SOCID_H5, &para);

		size = mctl_calc_rank_size(&para.ranks[0]);
		if (para.dual_rank)
			size *= 2;

		/* Quick memory test: 64MB region, sparse pattern */
		errors = memtest_quick(CFG_SYS_SDRAM_BASE, 0x4000000);

		printf("%d,0x%x,%d,OK,%lu,%u\n",
		       e->clk, e->zq, e->odt,
		       size >> 20, errors);
	}

	printf("=== SWEEP COMPLETE ===\n");

	/* Return to FEL for another run or data collection */
	printf("Returning to FEL...\n");
	return_to_fel(fel_stash.sp, fel_stash.lr);

	/* Not reached, but return last size for compilation */
	return size;
}

#endif /* CONFIG_DRAM_SWEEP */
