// SPDX-License-Identifier: GPL-2.0+
/*
 * G12-C5 first slice — BL33 command: dump the already-trained G12A/SM1
 * PHY centre and reuse the C6 width walker (VrefDAC0 + TxDqsDlyTg0).
 *
 * Same algorithm as g12a_dram_width() in arch/arm/mach-meson/dram-g12a.c
 * (TPL-only object; not linked into U-Boot proper).  Not hooked from
 * board_init() or PREBOOT.  Does not reload 1D/2D firmware.
 */

#include <command.h>
#include <log.h>
#include <string.h>
#include <u-boot/schedule.h>
#include <asm/io.h>
#include <asm/arch/g12a-ddr-fw.h>
#include <linux/types.h>

#define G12A_MRG_PA(csr)	(G12A_DDR_PHY_APB_BASE + ((ulong)(csr) << 1))
#define G12A_MRG_DB(d, l, off)	(0x10000u | ((u32)(d) << 12) | \
				 ((u32)(l) << 8) | (u32)(off))

#define G12A_MRG_VREFDAC0	0x40	/* [6:0] class 9 */
#define G12A_MRG_TXDQSDLY_TG0	0xD0	/* class 2, per nibble */

#define G12A_MRG_PROBE_BASE	0x30000000UL
#define G12A_MRG_PROBE_WORDS	0x00020000UL	/* 512 KiB */
#define G12A_MRG_SWEEP_MAX	48

static u16 g12a_cmd_vref0[4][9];
static u16 g12a_cmd_vref1[4][9];
static u16 g12a_cmd_txdqs[4][2][2];
static u32 g12a_cmd_rb_csr;
static u32 g12a_cmd_xor;

static void g12a_cmd_write16(ulong addr, u16 v)
{
	writew(v, (void *)addr);
	dsb();
}

static u16 g12a_cmd_read16(ulong addr)
{
	u16 v = readw((void *)addr);

	dsb();
	return v;
}

static u16 g12a_cmd_mux_enter(void)
{
	u16 old = g12a_cmd_read16(G12A_DDR_PHY_MICROCONTMUXSEL);

	g12a_cmd_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
	return old;
}

static void g12a_cmd_mux_exit(u16 old)
{
	g12a_cmd_write16(G12A_DDR_PHY_MICROCONTMUXSEL, old);
}

static u32 g12a_cmd_dec4(u16 raw)
{
	return (raw & 0x3Fu) + (((raw >> 6) & 7u) << 5);
}

static u16 g12a_cmd_enc4(u32 v)
{
	return (u16)((v & 0x1Fu) | ((v >> 5) << 6));
}

static u32 g12a_cmd_val(u32 pat, u32 i, ulong a)
{
	u32 h;

	if (pat == 0)
		return (i & 1) ? 0xFFFFFFFFu : 0x00000000u;
	h = i * 2654435761u;
	h ^= h >> 15;
	h *= 2246822519u;
	h ^= h >> 13;
	return h;
}

static u32 g12a_cmd_run(u32 pat, ulong base, u32 words)
{
	u32 i, bad = 0;

	for (i = 0; i < words; i++) {
		ulong a = base + (ulong)i * 4;

		writel(g12a_cmd_val(pat, i, a), (void *)a);
		if ((i & 0xFFFF) == 0)
			schedule();
	}
	for (i = 0; i < words; i++) {
		ulong a = base + (ulong)i * 4;
		u32 e = g12a_cmd_val(pat, i, a);
		u32 g = readl((void *)a);

		if (g != e) {
			g12a_cmd_xor |= g ^ e;
			bad++;
		}
		if ((i & 0xFFFF) == 0)
			schedule();
	}
	return bad;
}

static u32 g12a_cmd_probe(void)
{
	u32 bad;

	g12a_cmd_xor = 0;
	bad = g12a_cmd_run(0, G12A_MRG_PROBE_BASE, G12A_MRG_PROBE_WORDS);
	bad += g12a_cmd_run(7, G12A_MRG_PROBE_BASE, G12A_MRG_PROBE_WORDS);
	return bad;
}

static void g12a_cmd_dump(void)
{
	u32 d, l, n;
	u16 mux, ppt0, ppt1, v0, q0;

	mux = g12a_cmd_read16(G12A_DDR_PHY_MICROCONTMUXSEL);
	ppt0 = g12a_cmd_read16(G12A_DDR_PHY_PPTTRAINSETUP);
	ppt1 = g12a_cmd_read16(G12A_DDR_PHY_PPTTRAINSETUP_PS1);
	v0 = g12a_cmd_read16(G12A_MRG_PA(G12A_MRG_DB(0, 0, G12A_MRG_VREFDAC0)));
	q0 = g12a_cmd_read16(G12A_MRG_PA(G12A_MRG_DB(0, 0,
						    G12A_MRG_TXDQSDLY_TG0)));

	printf("MGW mux %04x ppt %04x/%04x v %04x q %04x\n",
	       mux, ppt0, ppt1, v0, q0);

	for (d = 0; d < 4; d++) {
		printf("MGCv %02x", d);
		for (l = 0; l < 9; l++) {
			g12a_cmd_vref0[d][l] = g12a_cmd_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, l,
							G12A_MRG_VREFDAC0)));
			printf(" %04x", g12a_cmd_vref0[d][l]);
		}
		printf("\n");
	}
	for (d = 0; d < 4; d++) {
		printf("MGCV %02x", d);
		for (l = 0; l < 9; l++) {
			g12a_cmd_vref1[d][l] = g12a_cmd_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, l,
							0x30))); /* VrefDAC1 */
			printf(" %04x", g12a_cmd_vref1[d][l]);
		}
		printf("\n");
	}
	for (d = 0; d < 4; d++) {
		printf("MGCq %02x", d);
		for (n = 0; n < 2; n++) {
			g12a_cmd_txdqs[d][n][0] = g12a_cmd_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
							G12A_MRG_TXDQSDLY_TG0)));
			g12a_cmd_txdqs[d][n][1] = g12a_cmd_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
							G12A_MRG_TXDQSDLY_TG0 +
							1)));
			printf(" %04x %04x", g12a_cmd_txdqs[d][n][0],
			       g12a_cmd_txdqs[d][n][1]);
		}
		printf("\n");
	}
}

static void g12a_cmd_vref_set(int delta)
{
	u32 d, l;

	for (d = 0; d < 4; d++) {
		for (l = 0; l < 9; l++) {
			int v = (int)(g12a_cmd_vref0[d][l] & 0x7Fu) + delta;
			u16 keep;

			if (v < 0)
				v = 0;
			if (v > 0x7F)
				v = 0x7F;
			keep = g12a_cmd_vref0[d][l] & (u16)~0x7Fu;
			g12a_cmd_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
								 G12A_MRG_VREFDAC0)),
					 (u16)(keep | (u16)v));
			keep = g12a_cmd_vref1[d][l] & (u16)~0x7Fu;
			g12a_cmd_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, 0x30)),
					 (u16)(keep | (u16)v));
		}
	}
}

static void g12a_cmd_vref_restore(void)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 9; l++) {
			g12a_cmd_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
								 G12A_MRG_VREFDAC0)),
					 g12a_cmd_vref0[d][l]);
			g12a_cmd_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, 0x30)),
					 g12a_cmd_vref1[d][l]);
		}
}

static void g12a_cmd_txdqs_set(int delta)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++) {
				int v = (int)g12a_cmd_dec4(
						g12a_cmd_txdqs[d][n][t]) +
					delta;

				if (v < 0)
					v = 0;
				if (v > 255)
					v = 255;
				g12a_cmd_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
							G12A_MRG_TXDQSDLY_TG0 + t)),
						 g12a_cmd_enc4((u32)v));
			}
}

static void g12a_cmd_txdqs_restore(void)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++)
				g12a_cmd_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
							G12A_MRG_TXDQSDLY_TG0 + t)),
						 g12a_cmd_txdqs[d][n][t]);
}

static int g12a_cmd_sweep(const char *tag, void (*set)(int),
			  void (*restore)(void), int dir, int limit)
{
	int k;
	int last = 0;

	for (k = 1; k <= limit; k++) {
		u32 bad;

		set(dir * k);
		bad = g12a_cmd_probe();
		printf("%s %c%02x %08x %04x %08x\n", tag,
		       dir < 0 ? '-' : '+', k, bad,
		       g12a_cmd_read16(G12A_MRG_PA(g12a_cmd_rb_csr)),
		       g12a_cmd_xor);
		if (bad)
			break;
		last = k;
	}
	restore();
	return last;
}

static void g12a_cmd_width_line(char axis, int lo, int hi, u32 csr)
{
	int w = lo + hi + 1;

	printf("MGW %c -%02x +%02x %02x %08x %04x\n", axis, lo, hi, w, csr,
	       g12a_cmd_read16(G12A_MRG_PA(csr)));
}

static int g12a_cmd_width(void)
{
	u16 mux;
	u32 bad;
	int hi, lo;
	u32 csr_v, csr_q;

	printf("MGW begin\n");
	mux = g12a_cmd_mux_enter();
	g12a_cmd_dump();

	bad = g12a_cmd_probe();
	printf("MGW centre %08x\n", bad);
	if (bad) {
		printf("MGW abort: centre fails\n");
		g12a_cmd_mux_exit(mux);
		printf("MGW end\n");
		return CMD_RET_FAILURE;
	}

	csr_v = G12A_MRG_DB(0, 0, G12A_MRG_VREFDAC0);
	g12a_cmd_rb_csr = csr_v;
	hi = g12a_cmd_sweep("MGSv", g12a_cmd_vref_set, g12a_cmd_vref_restore,
			    +1, G12A_MRG_SWEEP_MAX);
	lo = g12a_cmd_sweep("MGSv", g12a_cmd_vref_set, g12a_cmd_vref_restore,
			    -1, G12A_MRG_SWEEP_MAX);
	g12a_cmd_width_line('v', lo, hi, csr_v);

	csr_q = G12A_MRG_DB(0, 0, G12A_MRG_TXDQSDLY_TG0);
	g12a_cmd_rb_csr = csr_q;
	hi = g12a_cmd_sweep("MGSq", g12a_cmd_txdqs_set, g12a_cmd_txdqs_restore,
			    +1, G12A_MRG_SWEEP_MAX);
	lo = g12a_cmd_sweep("MGSq", g12a_cmd_txdqs_set, g12a_cmd_txdqs_restore,
			    -1, G12A_MRG_SWEEP_MAX);
	g12a_cmd_width_line('q', lo, hi, csr_q);

	g12a_cmd_mux_exit(mux);
	printf("MGW end\n");
	return CMD_RET_SUCCESS;
}

static int do_g12a_dram(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	const char *sub = (argc < 2) ? "width" : argv[1];
	u16 mux;

	if (!strcmp(sub, "dump")) {
		mux = g12a_cmd_mux_enter();
		g12a_cmd_dump();
		g12a_cmd_mux_exit(mux);
		return CMD_RET_SUCCESS;
	}
	if (!strcmp(sub, "width"))
		return g12a_cmd_width();

	return CMD_RET_USAGE;
}

U_BOOT_CMD(
	g12a_dram, 2, 0, do_g12a_dram,
	"dump G12A/SM1 trained DRAM centre / eye width (no 1D/2D retrain)",
	"[dump|width]\n"
	"    dump  - mux + trained VrefDAC0 / TxDqsDlyTg0 (already-trained centre)\n"
	"    width - dump, then C6 walker (stop at first fail); default\n"
	"  Opt-in CLI. Not board_init/PREBOOT. Does not reload 1D/2D."
);
