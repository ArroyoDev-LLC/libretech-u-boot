// SPDX-License-Identifier: GPL-2.0+
/*
 * G12A/SM1 DRAM stage of the TPL -> SPL -> FIT chain (L1.2b).
 *
 * Built in both phases; compiled to nothing in TPL (gates on
 * MESON_G12A_DRAM_STAGE).  Pattern: gxl-tpl spl-dram-stage.c -- without
 * GXL DWC2 BROM-return (G12 USB residual still open).
 *
 * Plan: amlogic/software/boot/g12/s905d3-cc-libre-plan.md L1.2b
 */

#include <spl.h>
#include <bootsource.h>
#include <asm/arch/boot.h>
#include <asm/arch/wdt-reset.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/delay.h>
#include <asm/arch/clock-g12a.h>
#include <asm/arch/g12a.h>
#include <asm/arch/g12a-handoff.h>
#include <usbflash-proto.h>

#ifdef MESON_G12A_DRAM_STAGE

#define AO_UART_WFIFO	0xff803000UL
#define AO_UART_STATUS	0xff80300cUL
#define AO_UART_TX_FULL	BIT(21)

static void dram_putc(char c)
{
	int timeout = 100000;

	while ((readl((void *)AO_UART_STATUS) & AO_UART_TX_FULL) && --timeout)
		;
	if (timeout)
		writel(c, (void *)AO_UART_WFIFO);
}

static void dram_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			dram_putc('\r');
		dram_putc(*s++);
	}
}

static void dram_puthex(u32 v)
{
	int i;

	dram_puts("0x");
	for (i = 28; i >= 0; i -= 4) {
		u32 nib = (v >> i) & 0xf;

		dram_putc(nib < 10 ? '0' + nib : 'a' + nib - 10);
	}
}

static void dram_stage_note(u32 code, u32 fault)
{
	u32 prog = (code == G12A_DRAM_STAGE_OK || code == G12A_DRAM_STAGE_FIT) ?
		   G12A_XPL_PROG_DRAM_OK : G12A_XPL_PROG_DRAM_BAD;

	writel(fault, (void *)G12A_DRAM_FAULT_ADDR);
	writel(G12A_DRAM_STAGE_MAGIC | code, (void *)G12A_DRAM_STAGE_ADDR);
	writel(readl((void *)G12A_XPL_PROG_ADDR) | prog,
	       (void *)G12A_XPL_PROG_ADDR);
	asm volatile("dsb sy" ::: "memory");

	if (code == G12A_DRAM_STAGE_OK)
		dram_puts("S2 OK\n");
	else if (code == G12A_DRAM_STAGE_FIT)
		dram_puts("S2 FIT\n");
	else {
		dram_puts("S2 REFUSE ");
		dram_puthex(code);
		dram_puts(" got ");
		dram_puthex(fault);
		dram_puts("\n");
	}
}

/*
 * Refused: the note above records why, then the watchdog resets the board
 * rather than leaving it wedged (arch/arm/mach-meson/wdt-reset.c).
 */
static void __noreturn dram_stage_refuse(u32 code, u32 fault)
{
	dram_stage_note(code, fault);
	meson_wdt_reset_after(MESON_HANG_RESET_MS);
}

/*
 * SYS PLL bring-up -- the vendor BL2's job, done by the stage that replaces
 * it.
 *
 * On the open G12A/SM1 chain nothing programmed HHI_SYS_PLL_CNTL0..CNTL6, so
 * Linux met the block at power-on reset (CNTL0 = 0x20000000: lock=0, rst=1,
 * en=0, M=0, N=0; CNTL1..CNTL6 all zero).  Mainline's g12a_sys_pll_dco has
 * no .init_regs and no NOINIT_ENABLED; meson_clk_pll_enable() writes only
 * rst/en, never M/N/OD or the analog block, so with M=0 and N=0 wait_lock()
 * cannot succeed, sys_pll_dco is CLK_IS_CRITICAL, -EIO aborts the g12a-clkc
 * probe and every EE-clock consumer defers -- the eMMC among them
 * (amlogic/software/boot/g12/audit/2026-09-12-syspll.md).  Every Meson family
 * leaves this PLL to the bootloader: GXBB/GXL and AXG make it read-only to
 * Linux outright, and CLK_MESON_PLL_NOINIT_ENABLED exists solely to skip
 * init on a PLL "already initialized ... by lower levels"
 * (2026-09-13-syspll-layer.md D2).
 *
 * Here, and not in U-Boot proper: this stage is what replaces the vendor's
 * BL2, whose sys_pll_init() is where the vendor chain does this; BL33's
 * board_late_init() is the last hook of the last stage and leaves four
 * stages running on an unlocked CPU PLL (syspll-layer D5).  The GXL twin is
 * meson_cpu_sys_pll_force_1g2() in spl-gx.c, called from the SRAM stage;
 * on G12A the SRAM stage is the TPL, which jumps here before the GXL call
 * site is reached, so the DRAM stage is the first G12A-phase C that runs
 * with the SoC fully addressable and the handoff validated.
 *
 * Values and write ORDER are the vendor BL33's sys_pll_init(), the only
 * source-level authority for this block in the BSP (bl2 ships as a binary):
 *   sys_pll_init()             .../uboot/bl33/v2019/arch/arm/mach-meson/
 *                              g12a/pll.c:299-315
 *   sys_pll_cfg.sys_pll[0]     same file :149-159
 *   CNTL0, 750 < MHz <= 1500   same file :362-364, :370
 * Proven on AML-S905D3-CC from the BL33 placement first: the same seven
 * words locked on the first attempt and took the kernel's EE-clock
 * deferrals from 16 to 0 (2026-09-12-syspll-init.md D1/D3).
 *
 * Register offsets are the in-tree vendor-cited ones:
 *   HHI_SYS_PLL_CNTL0..CNTL6  0x2f4..0x30c  arch-meson/clock-g12a.h:88-94
 *   HHI_SYS_CPU_CLK_CNTL0     0x19c         arch-meson/clock-g12a.h:58
 *   G12A_HIU_BASE             0xff63c000    arch-meson/g12a.h:16
 *
 * The CPU mux is deliberately NOT moved onto the PLL: handing Linux a CPU
 * already on sys_pll was measured to change nothing for the better
 * (2026-09-12-syspll1200.md), and DVFS hand-off is a separate contract.
 */

/* HHI_SYS_PLL_CNTL0 fields, per drivers/clk/meson/g12a.c g12a_sys_pll_dco */
#define SYS_PLL_CNTL0_LOCK		BIT(31)	/* [31] l   -- read-only status */
#define SYS_PLL_CNTL0_VENDOR_EN		BIT(30)	/* [30] the vendor's enable write */
#define SYS_PLL_CNTL0_RST		BIT(29)	/* [29] rst -- analog reset */
/*
 * 1200 MHz by the vendor's formula for 750 < MHz <= 1500:
 *   0x38000000 | (OD << 16) | (N << 10) | M,  OD = 2, N = 1, M = 1200/6 = 200
 * m[7:0] = 200 (inside g12a_sys_pll_mult_range {128..250}), n[14:10] = 1,
 * od[18:16] = 2  =>  24 MHz * 200 / 1 = 4800 MHz DCO, >> 2 = 1200 MHz.
 */
#define SYS_PLL_CNTL0_1G2		0x380204c8
/* Analog block.  CNTL1/CNTL2 are zero in the vendor cfg. */
#define SYS_PLL_CNTL3_VAL		0x48681c00
#define SYS_PLL_CNTL4_VAL		0x88770290
#define SYS_PLL_CNTL5_VAL		0x39272000
/*
 * CNTL6 is an addition to sys_pll_init(), which writes CNTL0-CNTL5 only: the
 * value is the vendor's own for this PLL (pll.c:157) and what mainline writes
 * for every G12A PLL that carries an .init_regs.  The vendor routine can omit
 * it because it runs after BL2 has configured the block; our chain arrives
 * with CNTL6 at POR 0.  Whether it is REQUIRED is not adjudicated
 * (syspll-init D9); it is known to be harmless.
 */
#define SYS_PLL_CNTL6_VAL		0x56540000

#define SYS_PLL_LOCK_RETRIES		500

/* HHI_SYS_CPU_CLK_CNTL0[11] final_mux_sel: 1 = CPU is fed from sys_pll */
#define SYS_CPU_CLK_CNTL0_FINAL_MUX_SEL	BIT(11)

/* G12A_HIU_BASE is a plain integer literal on a 64-bit build. */
static inline u32 hiu_rd(u32 off)
{
	return readl((void *)(ulong)(G12A_HIU_BASE + off));
}

static inline void hiu_wr(u32 off, u32 val)
{
	writel(val, (void *)(ulong)(G12A_HIU_BASE + off));
}

/*
 * No timer dependency at this point of the stage (same reason as the GXL
 * twin's busy-wait): ~30 us of nops at the ~666 MHz dynamic-mux rate the CPU
 * runs at here.  The vendor waits 20 us between reset release and the lock
 * test.
 */
static void sys_pll_settle(void)
{
	unsigned int i;

	for (i = 0; i < 20000; i++)
		asm volatile("nop");
}

/*
 * Configure and lock the SYS PLL at 1.2 GHz; leave the CPU on the dynamic mux.
 * Crumb: "S2 pll 0x<CNTL0>" (bit 31 set = locked), "S2 pll FAIL 0x<CNTL0>",
 * or "S2 pll skip 0x<CPU_CLK_CNTL0>" when the CPU is already running off the
 * PLL -- reprogramming it would pull the clock out from under this code.
 */
static void g12a_spl_sys_pll_init(void)
{
	u32 cpu = hiu_rd(HHI_SYS_CPU_CLK_CNTL0);
	int retries = SYS_PLL_LOCK_RETRIES;
	u32 cntl0;

	if (cpu & SYS_CPU_CLK_CNTL0_FINAL_MUX_SEL) {
		dram_puts("S2 pll skip ");
		dram_puthex(cpu);
		dram_puts("\n");
		return;
	}

	do {
		/* 1. hold the analog block in reset with the target M/N/OD */
		hiu_wr(HHI_SYS_PLL_CNTL0, SYS_PLL_CNTL0_1G2 | SYS_PLL_CNTL0_RST);
		/* 2-3. vendor cfg has both of these at zero */
		hiu_wr(HHI_SYS_PLL_CNTL1, 0x00000000);
		hiu_wr(HHI_SYS_PLL_CNTL2, 0x00000000);
		/* 4-6. analog block */
		hiu_wr(HHI_SYS_PLL_CNTL3, SYS_PLL_CNTL3_VAL);
		hiu_wr(HHI_SYS_PLL_CNTL4, SYS_PLL_CNTL4_VAL);
		hiu_wr(HHI_SYS_PLL_CNTL5, SYS_PLL_CNTL5_VAL);
		/* 6b. addition to sys_pll_init() -- see SYS_PLL_CNTL6_VAL */
		hiu_wr(HHI_SYS_PLL_CNTL6, SYS_PLL_CNTL6_VAL);
		/* 7. enable, still in reset */
		hiu_wr(HHI_SYS_PLL_CNTL0, SYS_PLL_CNTL0_VENDOR_EN |
					  SYS_PLL_CNTL0_RST | SYS_PLL_CNTL0_1G2);
		/* 8. release reset */
		hiu_wr(HHI_SYS_PLL_CNTL0,
		       hiu_rd(HHI_SYS_PLL_CNTL0) & ~SYS_PLL_CNTL0_RST);
		/* 9. settle, then test the lock bit */
		sys_pll_settle();
		cntl0 = hiu_rd(HHI_SYS_PLL_CNTL0);
	} while (!(cntl0 & SYS_PLL_CNTL0_LOCK) && --retries);

	dram_puts((cntl0 & SYS_PLL_CNTL0_LOCK) ? "S2 pll " : "S2 pll FAIL ");
	dram_puthex(cntl0);
	dram_puts("\n");
}

/*
 * GP1 PLL bring-up -- the SYS PLL's twin, for the PLL that feeds the DSU.
 *
 * The second instance of the same omission: on the open SM1 chain nothing
 * programmed HHI_GP1_PLL_CNTL0..CNTL6 either, so Linux meets that block at
 * power-on reset too (CNTL0 = 0x20000000).  Mainline's sm1_gp1_pll_dco is
 * meson_clk_pll_ro_ops -- no .enable, no .set_rate, no .init_regs -- and
 * CLK_IS_CRITICAL ("This clock feeds the DSU"), so Linux has no writable path
 * to it at all: a bootloader-owned PLL by construction
 * (amlogic/software/boot/g12/audit/2026-09-13-syspll-layer.md D6).  Same
 * stage, same mechanism as the SYS PLL above.
 *
 * Values, every one vendor-authored:
 *   rate 1200 MHz = M 200 / N 1 / OD 2 -- the vendor kernel's
 *     sm1_gp1_pll_table[0] "DCO=4800M OD=1200M"
 *     (A311D-S905D3_linux5.15 .../common_drivers/drivers/clk/meson/
 *     g12a.c:1814-1818) and the vendor BL2's own "DSU/CPU 1200 MHz" banner
 *   analog block CNTL1..CNTL6 -- the vendor kernel's sm1_gp1_init_regs
 *     (same file :1826-1837; the vendor kernel does not apply the set because
 *     its BL2 already has, exactly as sys_pll_init() omits CNTL6).  It is
 *     byte-identical to mainline's g12a_gp0_pll_init_regs (g12a.c:401-408):
 *     CNTL4 = 0x33771290 is the GP-class value, the one word that differs
 *     from the SYS PLL's 0x88770290
 *   CNTL0 word -- the vendor BL33 formula for 750 < MHz <= 1500
 *     (g12a/pll.c:362-370), the same base the vendor's own GP0 write uses
 *     (pll.c:591); byte-identical to SYS_PLL_CNTL0_1G2
 *   offsets -- clock-g12a.h HHI_GP1_PLL_CNTL0..6 (0x060..0x078), cited there
 *
 * The DSU is NOT moved onto the PLL: HHI_SYS_CPU_CLK_CNTL5 (dyn legs on
 * xtal at POR) and CNTL6 (dsu_clk from cpu_clk) are never written here, so
 * nothing runs off gp1_pll and a wrong write cannot take the cluster down.
 * CNTL5 is read once, as the guard.
 */

/* HHI_GP1_PLL_CNTL0 fields, per drivers/clk/meson/g12a.c sm1_gp1_pll_dco */
#define GP1_PLL_CNTL0_LOCK		BIT(31)	/* [31] l   -- read-only status */
#define GP1_PLL_CNTL0_VENDOR_EN		BIT(30)	/* [30] the vendor's enable write */
#define GP1_PLL_CNTL0_RST		BIT(29)	/* [29] rst -- analog reset */
/*
 * 1200 MHz: 0x38000000 | (OD << 16) | (N << 10) | M, OD = 2, N = 1, M = 200
 * m[7:0] = 200, n[14:10] = 1, od[18:16] = 2 => 24 MHz * 200 / 1 = 4800 MHz
 * DCO, >> 2 = 1200 MHz.
 */
#define GP1_PLL_CNTL0_1G2		0x380204c8
/* Analog block, vendor sm1_gp1_init_regs.  CNTL1/CNTL2 are zero there. */
#define GP1_PLL_CNTL3_VAL		0x48681c00
#define GP1_PLL_CNTL4_VAL		0x33771290
#define GP1_PLL_CNTL5_VAL		0x39272000
#define GP1_PLL_CNTL6_VAL		0x56540000

#define GP1_PLL_LOCK_RETRIES		500

/*
 * HHI_SYS_CPU_CLK_CNTL5 DSU dynamic-leg source selects, per
 * drivers/clk/meson/g12a.c sm1_dsu_clk_dyn0_sel / dyn1_sel: [1:0] and
 * [17:16], parent index 3 = gp1_pll.  READ ONLY here.
 */
#define SYS_CPU_CLK_CNTL5_DYN0_SEL_MASK	GENMASK(1, 0)
#define SYS_CPU_CLK_CNTL5_DYN1_SEL_MASK	GENMASK(17, 16)
#define SYS_CPU_CLK_CNTL5_DYN0_SEL_GP1	0x3
#define SYS_CPU_CLK_CNTL5_DYN1_SEL_GP1	(0x3 << 16)

/*
 * Configure and lock the GP1 PLL at 1.2 GHz; leave the DSU where it is.
 * Crumb: "S2 gp1 0x<CNTL0>" (bit 31 set = locked), "S2 gp1 FAIL 0x<CNTL0>",
 * or "S2 gp1 skip 0x<CPU_CLK_CNTL5>" when a DSU dynamic leg already selects
 * gp1_pll -- reprogramming a PLL the cluster runs off would pull the clock
 * out from under it.
 */
static void g12a_spl_gp1_pll_init(void)
{
	u32 cntl5 = hiu_rd(HHI_SYS_CPU_CLK_CNTL5);
	int retries = GP1_PLL_LOCK_RETRIES;
	u32 cntl0;

	if ((cntl5 & SYS_CPU_CLK_CNTL5_DYN0_SEL_MASK) ==
	    SYS_CPU_CLK_CNTL5_DYN0_SEL_GP1 ||
	    (cntl5 & SYS_CPU_CLK_CNTL5_DYN1_SEL_MASK) ==
	    SYS_CPU_CLK_CNTL5_DYN1_SEL_GP1) {
		dram_puts("S2 gp1 skip ");
		dram_puthex(cntl5);
		dram_puts("\n");
		return;
	}

	do {
		/* 1. hold the analog block in reset with the target M/N/OD */
		hiu_wr(HHI_GP1_PLL_CNTL0, GP1_PLL_CNTL0_1G2 | GP1_PLL_CNTL0_RST);
		/* 2-3. vendor init_regs have both of these at zero */
		hiu_wr(HHI_GP1_PLL_CNTL1, 0x00000000);
		hiu_wr(HHI_GP1_PLL_CNTL2, 0x00000000);
		/* 4-7. analog block */
		hiu_wr(HHI_GP1_PLL_CNTL3, GP1_PLL_CNTL3_VAL);
		hiu_wr(HHI_GP1_PLL_CNTL4, GP1_PLL_CNTL4_VAL);
		hiu_wr(HHI_GP1_PLL_CNTL5, GP1_PLL_CNTL5_VAL);
		hiu_wr(HHI_GP1_PLL_CNTL6, GP1_PLL_CNTL6_VAL);
		/* 8. enable, still in reset */
		hiu_wr(HHI_GP1_PLL_CNTL0, GP1_PLL_CNTL0_VENDOR_EN |
					  GP1_PLL_CNTL0_RST | GP1_PLL_CNTL0_1G2);
		/* 9. release reset */
		hiu_wr(HHI_GP1_PLL_CNTL0,
		       hiu_rd(HHI_GP1_PLL_CNTL0) & ~GP1_PLL_CNTL0_RST);
		/* 10. settle, then test the lock bit */
		sys_pll_settle();
		cntl0 = hiu_rd(HHI_GP1_PLL_CNTL0);
	} while (!(cntl0 & GP1_PLL_CNTL0_LOCK) && --retries);

	dram_puts((cntl0 & GP1_PLL_CNTL0_LOCK) ? "S2 gp1 " : "S2 gp1 FAIL ");
	dram_puthex(cntl0);
	dram_puts("\n");
}

/**
 * board_init_f() - DRAM stage entry (overrides SRAM stage's).
 *
 * Validates the handoff block, then joins the generic xPL FIT/ATF path.
 */
void board_init_f(ulong dummy)
{
	struct g12a_xpl_handoff *h = (struct g12a_xpl_handoff *)G12A_HANDOFF_ADDR;
	u32 magic, version, flags, size_mb, spl_addr;

	writel(G12A_DRAM_STAGE_MAGIC | G12A_DRAM_STAGE_ENTRY,
	       (void *)G12A_DRAM_STAGE_ADDR);
	writel(readl((void *)G12A_XPL_PROG_ADDR) | G12A_XPL_PROG_DRAM_ENTRY,
	       (void *)G12A_XPL_PROG_ADDR);
	dram_puts("S2\n");

	magic = readl(&h->magic);
	if (magic != G12A_HANDOFF_MAGIC)
		dram_stage_refuse(G12A_DRAM_STAGE_BAD_MAGIC, magic);

	version = readl(&h->version);
	if (version != G12A_HANDOFF_VERSION)
		dram_stage_refuse(G12A_DRAM_STAGE_BAD_VER, version);

	flags = readl(&h->flags);
	if (!(flags & G12A_HANDOFF_F_DDR_OK))
		dram_stage_refuse(G12A_DRAM_STAGE_BAD_DDR, flags);

	size_mb = readl(&h->dram_size_mb);
	if (size_mb < G12A_HANDOFF_DRAM_MIN_MB ||
	    size_mb > G12A_HANDOFF_DRAM_MAX_MB)
		dram_stage_refuse(G12A_DRAM_STAGE_BAD_SIZE, size_mb);

	spl_addr = readl(&h->spl_addr);
	if (!(flags & G12A_HANDOFF_F_SPL) || spl_addr != CONFIG_SPL_TEXT_BASE)
		dram_stage_refuse(G12A_DRAM_STAGE_BAD_ADDR, spl_addr);

	dram_puts("S2 dram ");
	dram_puthex(size_mb);
	dram_puts(" MB @ ");
	dram_puthex(spl_addr);
	dram_puts("\n");

	dram_stage_note(G12A_DRAM_STAGE_OK, 0);

	/*
	 * After the handoff is validated (a refused boot never touches a
	 * PLL) and before driver model, the console and the FIT load: the
	 * vendor's BL2 has the PLL locked before it loads anything either.
	 */
	g12a_spl_sys_pll_init();
	/* Its twin for the PLL that feeds the DSU; the DSU itself is left parked. */
	g12a_spl_gp1_pll_init();

	if (CONFIG_IS_ENABLED(OF_CONTROL)) {
		int ret = spl_early_init();

		if (ret)
			dram_stage_refuse(G12A_DRAM_STAGE_NO_DM, (u32)ret);
	}

	spl_init();
	icache_enable();
	preloader_console_init();

	/* First console line after the banner: where the BootROM found us. */
	if (CONFIG_IS_ENABLED(BOOTSOURCE))
		bootsource_announce(meson_bootsource());

	dram_stage_note(G12A_DRAM_STAGE_FIT, 0);
}

/**
 * spl_perform_board_fixups() - crumb the loaded image on the way out.
 *
 * "Trying to boot from RAM" with NOTHING after it (boots spl-3, spl-4, both
 * with a complete 1577472 B payload and SHOW_ERRORS + FIT_PRINT on) is
 * ambiguous: the RAM/FIT loader could have wedged, or it could have loaded
 * cleanly and jumped into a BL31 that never opens a console.  boot_from_devices
 * failing would have printed; this says whether it succeeded.
 *
 * Crumb: "S2 img os=<n> entry=<a> size=<n> fdt=<a>"
 */
void spl_perform_board_fixups(struct spl_image_info *spl_image)
{
	dram_puts("S2 img os=");
	dram_puthex(spl_image->os);
	dram_puts(" entry=");
	dram_puthex((u32)spl_image->entry_point);
	dram_puts(" load=");
	dram_puthex((u32)spl_image->load_addr);
	dram_puts(" size=");
	dram_puthex(spl_image->size);
	dram_puts("\n");

	/* The USB flash session's mark for U-Boot proper (spl-usbflash.c) */
	if (IS_ENABLED(CONFIG_SPL_USBFLASH))
		spl_usbflash_fdt_fixup(spl_image_fdt_addr(spl_image));
}

/**
 * spl_board_prepare_for_boot() - last host-visible instant before the jump.
 */
void spl_board_prepare_for_boot(void)
{
	dram_puts("S2 GO\n");
}

#endif /* MESON_G12A_DRAM_STAGE */
