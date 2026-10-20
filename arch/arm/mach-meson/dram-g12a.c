// SPDX-License-Identifier: GPL-2.0+
/*
 * G12A/SM1 open TPL DDR bring-up: stage the vendor DDR firmware set, drive
 * its entry points, bring the DMC up, and hand DRAM to the SPL.
 *
 * This is NOT blob-free.  The vendor firmware set stays closed and is what
 * actually trains the PHY; only the orchestration around it is open code:
 *     aml_ddr.fw   (48080 B) → 0xFFFB4000   driver / entry blob
 *     piei.fw      (10088 B) → 0xFFFCC000
 *     lpddr4_1d.fw (49248 B) → 0xFFFC0000, staged into PHY IMEM/DMEM
 *     lpddr4_2d.fw (49248 B) → 0xFFFD0000, staged over the same window
 * Delivered as one concatenated G12R residual frame, aml_ddr‖piei‖1d[‖2d].
 * Replacing them is tracked as P2/P3 in the arc plan cited below.
 *
 * UART breadcrumbs go out the AO UART (0xFF803000) with no DM and no
 * console.  Every phase prints, so silence is always a failure and never a
 * quiet success.  Single-character crumbs, in rough boot order:
 *
 *   D dram_init entered            C scratch clear done
 *   H aml_ddr payload at dest      h no aml_ddr at dest or hold
 *   M aml_ddr payload confirmed    m no payload -- no-train path
 *   P piei magic present           p piei missing
 *   0 DDR PLL locked               o DDR PLL lock fail
 *   n PHY/DMC soft-rst + clk enable (FUN_fffa7e58) done
 *   L 1d staged at 0xFFFC0000      s 1d staged after a copy
 *   h0 1d found at HOLD0           hA 1d found at HOLD_1D_ALT
 *   l no 1d anywhere               J product ACS p0 copied into ddr_set
 *   B control block built          b build refused (no payload at dest)
 *   e ctl build failed, entry skipped
 *   N no-train path (handoff must not set F_DDR_OK)
 *
 * Both cases of a pair can appear: 0/n are also printed by the
 * G12A_OPEN_PLL_INIT=0 / G12A_OPEN_PHY_REG_ACCESS=0 arms (both default to
 * G12A_OPEN_PLL_SOFT_RST) to mark the skipped PLL and skipped soft-reset.
 * A crumb stream therefore cannot tell those arms apart -- identify an
 * image by content.
 *
 * Multi-character families, each documented where it is emitted:
 *   si.. sihw.. sib..   product-order stage_init, POR hw/pwm, and the
 *                       board-ID sticky written to AO_SEC_GP_CFG0[15:8]
 *   MPD0 MPD1           HHI_MEM_PD_REG0 before / after clearing bits [17:16]
 *                       -- the PHY microcode SRAM power-up
 *   MPDE MPDF           the same register read at the earliest DDR instant
 *                       and at the last instant before the SRAM fill
 *   c ..                ACS readback: freq seq hdt rank type
 *   os ..               piei TOC scan result (dl44 pointer)
 *   cb ..               CalBusy / MemResetL sampled before fp_pll
 *   >bp bp              the vendor fp_pll export slot, entered / returned
 *   >N <N N=1..9        the vendor a644 body walked one step at a time:
 *                       1 topology 2 timing 3 prepare 4 pinmux 5 MID1
 *                       6 MID2 7 MID3 8 2280 (IMEM+DMEM load) 9 4dc0 (train)
 *   sb db               loader source / destination head halfwords, either
 *                       side of step 8
 *   pa st uc SD         DRTUB vs APBONLY shadow, MicroReset, UcclkHclk, and
 *                       the PHY DMEM readback, all read-only around step 8
 *   IFI IFN IFP IFX     in-fill gate probe: installed, per-record samples,
 *                       image checksum, and its refusals (sum|site|range).
 *                       NOT PRINTED by default -- G12A_INFILL_PROBE is 0
 *   PCSR BEGIN|SRAM|END|SKIP    PHY CSR state capture
 *   S2 S2 skip S2 end   stage 2: entered on a zero return from step 9,
 *                       skipped otherwise, and its tail
 *   >tN <tN <td         stage-2 export-slot calls (DMC bring-up, size
 *                       detect, DataBus/AddrBus); <td is the DMC "go" tail
 *   >2c <2c no2d        2D staging swap in / out (1D leaves 0108/0000), and
 *                       the no-2D-delivered path
 *   dm ms               DMC register dump, and the size published to
 *                       AO_SEC_GP_CFG0
 *   DT begin|end DTA..DTE   the DRAM decode tests (A coarse aliasing,
 *                       B stride, C pattern, D full span, E the clamped tail)
 *   dt cb2 >ba ba       G12A_BL2_ORDER_STEPWISE=0 only: the DRTUB probe and
 *                       the monolithic a644 call
 *   MG MGC MGE MGS MGT MGV      CONFIG_MESON_G12A_DRAM_MARGIN only: trained
 *                       CSR dump, environment (temperature / VDDEE), shmoo,
 *                       stress, and voltage
 *
 * Arc state and evidence:
 *   amlogic/software/boot/g12/s905d3-open-boot-status.md   (state of the chain)
 *   amlogic/software/boot/g12/g12a-ddr-blob-independence-plan.md (live plan)
 * RE:   amlogic software/boot/audit/g12b-ddr-init-analysis.md §1.2 §9.2
 *       amlogic reversing amlddr-fw-function-map.md (context layout)
 */

#include <init.h>
#include <config.h>
#include <asm/io.h>
#include <asm/arch/g12a.h>
#include <asm/arch/g12a-ddr-fw.h>
#include <asm/arch/g12a-ddr-timing.h>
#include <asm/arch/g12a-ddr-open.h>
#include <linux/types.h>

/*
 * G12A_OPEN_REPLAY -- mode-A snapshot replay: bring the DRAM up with NO PMU
 * training microcode anywhere in the path.
 *
 * a644's mode A (ctl+612 == 0xFF) is the vendor's own no-training route: it
 * skips fffb6294 (PHY IMEM/DMEM stage) and fffb8dd4 (micro kick) entirely and
 * instead RESTORES the ten delay classes out of a previously captured
 * ddr_set_t (fffbea84 bl fffb5a54, w1 = 0), programs the PIE (T7), captures
 * them back (fffbea9c, w1 = 1) and runs the ordinary DMC tail.  On this arm
 * there is no a644 to call -- topic/g12a-slot5-replaced retired all five
 * aml_ddr.fw call sites -- so the open code takes that sequence itself.
 *
 * With this gate on:
 *   - g12a_ddr_ctl_build() overwrites ds with the captured 1176 B snapshot,
 *     VERBATIM (asm/arch/g12a-ddr-snapshot.h, generated by
 *     amlogic/software/tools/mk-g12a-ds-snapshot.py from
 *     amlogic/software/boot/g12/ds-snapshot-trained.md);
 *   - g12a_open_stage1() stops after S6/S7 -- no g12a_open_s9_load(), no
 *     g12a_open_train_kick() at all;
 *   - stage 2 runs T1/T2, T5a restore, T7, T8 capture, T10 (+ the host-side
 *     DRAM mode-register init the PMU would otherwise have issued), T11..T14;
 *   - the residual concat ships the lpddr4_1d / lpddr4_2d windows ZERO-FILLED
 *     (mk-g12a-residual-concat.py --replay).
 *
 * THE SNAPSHOT IS PER UNIT, per part, per temperature and per voltage.  This
 * is a bring-up vehicle and a blob-boundary result -- it is NOT training and
 * NOT a shipping configuration.
 *
 * amlogic/software/boot/g12/audit/2026-08-30-open-mode-a-replay.md
 * amlogic/software/boot/g12/ddr-replay-implementation.md
 * amlogic/software/boot/g12/class-restore-transcription.md
 * amlogic/software/boot/g12/mr-vref-transcription.md
 */
#ifndef G12A_OPEN_REPLAY
#define G12A_OPEN_REPLAY	0
#endif

/*
 * G12A_OPEN_MR0 -- emit the blob's MR0 write in the host MR init.
 *
 * fffbb638/fffbb644 write MR0 | 0x100 unconditionally for every DRAM type.
 * mr-vref-transcription.md sec.7 D2: on LPDDR4 that is a DDR3-ism -- MR0 is
 * read-only device information and bit 8 is DDR3's "DLL reset".  Off by
 * default; the knob exists so the departure is testable rather than assumed.
 */
#ifndef G12A_OPEN_MR0
#define G12A_OPEN_MR0		0
#endif

#if G12A_OPEN_REPLAY
#include <asm/arch/g12a-ddr-snapshot.h>
#endif

/*
 * Sticky train state lives at G12A_TRAIN_STATUS_STICKY (AHB gap), not BSS.
 * BSS must stay OUT of the aml_ddr window (0xFFFB4000..0xFFFBFBD0): the
 * residual chain sets CONFIG_SPL_BSS_START_ADDR=0xfffb0000 for both phases
 * precisely because the earlier 0xfffbc000 was inside it, and anything in
 * there is wiped on clear or clobbered when the host places aml_ddr.fw.
 */
static void g12a_train_status_write(u32 v)
{
	writel(v, (void *)G12A_TRAIN_STATUS_STICKY);
}

static u32 g12a_train_status_read(void)
{
	return readl((void *)G12A_TRAIN_STATUS_STICKY);
}

static void g12a_train_status_or(u32 bits)
{
	g12a_train_status_write(g12a_train_status_read() | bits);
}

/* Bare-metal AO UART — BROM left it up; no DM yet. */
static void g12a_uart_putc(char c)
{
	int timeout = 100000;

	while ((readl((void *)G12A_AO_UART_STATUS) & (1 << 21)) && --timeout)
		;
	if (timeout)
		writel(c, (void *)G12A_AO_UART_WFIFO);
}

static void g12a_uart_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			g12a_uart_putc('\r');
		g12a_uart_putc(*s++);
	}
}

static void g12a_uart_puthex8(u8 v)
{
	static const char hex[] = "0123456789abcdef";

	g12a_uart_putc(hex[(v >> 4) & 0xf]);
	g12a_uart_putc(hex[v & 0xf]);
}

/* Product stage_init crumb: "si" + 2-digit hex stage + '\n' */
static void g12a_stage_crumb(u8 stage)
{
	g12a_uart_puts("si");
	g12a_uart_puthex8(stage);
	g12a_uart_putc('\n');
}

/*
 * stage_init__ns body (RE g12b bl2.c): *reg = (*reg & ~mask) | value
 * ACS bl2_reg flag byte selects stage.
 */
static void g12a_reg_clrset(ulong addr, u32 mask, u32 value)
{
	u32 v = readl((void *)addr);

	writel((v & ~mask) | value, (void *)addr);
}

/**
 * g12a_stage_init_sm1 - Solitude product-like PWM power + board-id sticky
 *
 * Product sequence (UART): bl2_stage_init 0x01 / 0x81 /
 *   hw id: 0x0000 - pwm id 0x01 / 0xc1 / 0x02; Board ID = 1.
 * Open hardcodes Solitude ACS table from boot-lite aml-s905d3-cc timing.c
 * (VDDEE 840 mV AO_PWM_B CFG0/CFG1), with VCCK on AO_PWM_D at the same
 * table's 810 mV entry rather than the board's configured 800 mV (see
 * G12A_SOLITUDE_VCCK_PWM).
 * Both Solitude CFG_GROUP rows map to PWM_CFG1 (product pwm id 0x01).
 */
void g12a_stage_init_sm1(void)
{
	u32 por, hw_id, pwm_id, vddee;
	u32 gp;

	/* --- stage 0x01: VCCK PWM_AO_D + enable TEST_N / VCCK --- */
	g12a_reg_clrset(G12A_AO_PWM_PWM_D, 0xffffffffU,
			G12A_SOLITUDE_VCCK_PWM);
	/* enable PWM_D: bits [22:16] clear, set clk+en pattern */
	g12a_reg_clrset(G12A_AO_PWM_MISC_CD, (0x7fU << 16),
			(1U << 23) | (1U << 1));
	/* AO_PIN_MUX_REG1[23:20] = 3 → PWM_D */
	g12a_reg_clrset(G12A_AO_RTI_PIN_MUX_REG1, (0xfU << 20), (3U << 20));
	/* VCCK enable: AO_SEC_REG0 bit0 + AO_GPIO_O bit31 (TEST_N) */
	g12a_reg_clrset(G12A_AO_SEC_REG0, 0xffffffffU, (1U << 0));
	g12a_reg_clrset(G12A_AO_GPIO_O, 0xffffffffU, (1U << 31));
	g12a_stage_crumb(G12A_BL2_STAGE_1);

	/* --- stage 0x81: PWM pre (empty ACS table on Solitude) --- */
	g12a_stage_crumb(G12A_BL2_STAGE_PWM_PRE);

	/*
	 * Product FUN_fffa5ee4: stage 0x82 reads ASSIST_POR bit7 → hw_id;
	 * stage 0x83 matches hw_id → pwm_id. Solitude table maps 0 and 1
	 * both to PWM_CFG1; product UART: hw=0 pwm=1 → stage 0xc1.
	 */
	por = readl((void *)G12A_ASSIST_POR_CONFIG);
	hw_id = (por >> 7) & 1U;
	/* Solitude CFG_GROUP: {0x1→CFG1, 0x0→CFG1} */
	pwm_id = G12A_PWM_CFG1;
	(void)hw_id;
	g12a_uart_puts("sihw");
	g12a_uart_puthex8((u8)hw_id);
	g12a_uart_putc('p');
	g12a_uart_puthex8((u8)pwm_id);
	g12a_uart_putc('\n');

	/* --- stage 0xc0|pwm: VDDEE PWM_AO_B --- */
	vddee = (pwm_id == G12A_PWM_CFG0) ?
		G12A_SOLITUDE_VDDEE_PWM_CFG0 : G12A_SOLITUDE_VDDEE_PWM_CFG1;
	g12a_reg_clrset(G12A_AO_PWM_PWM_B, 0xffffffffU, vddee);
	g12a_reg_clrset(G12A_AO_PWM_MISC_AB, (0x7fU << 16),
			(1U << 23) | (1U << 1));
	/* AO_PIN_MUX_REG1[19:16] = 3 → PWM_B */
	g12a_reg_clrset(G12A_AO_RTI_PIN_MUX_REG1, (0xfU << 16), (3U << 16));
	g12a_stage_crumb((u8)(G12A_BL2_STAGE_PWM_INIT | pwm_id));

	/* --- stage 0x02: post-power (empty on Solitude ACS) --- */
	g12a_stage_crumb(G12A_BL2_STAGE_2);

	/*
	 * Board ID: product SARADC writes 1 into AO_SEC_GP_CFG0[15:8].
	 * Open only READs + crumbs — a full writel of that SEC reg rebooted
	 * the board mid meson_power_init (measured: si02 then BROM, no "2").
	 * Product ACS board_id is 0xff (mask) so aml_ddr still matches.
	 */
	gp = readl((void *)G12A_AO_SEC_GP_CFG0);
	g12a_uart_puts("sib");
	g12a_uart_puthex8((u8)((gp >> G12A_AO_SEC_BOARD_ID_SHIFT) & 0xffU));
	g12a_uart_putc('\n');
	(void)G12A_SOLITUDE_BOARD_ID;
}

/**
 * meson_power_init - G12A/SM1 strong override (spl.c weak is empty)
 *
 * Product BL2 runs stage_init in bl2_early_platform_setup before DDR.
 * Open TPL must raise VCCK/VDDEE before a644 train.
 *
 * G12B (Alta) is dual-cluster and supplies its own strong override from
 * spl-g12b.c, so compile this one out there rather than link two.
 * g12a_stage_init_sm1() stays built and externally visible either way.
 */
#if !IS_ENABLED(CONFIG_MESON_G12B_POWER_INIT)
void meson_power_init(void)
{
	g12a_stage_init_sm1();
}
#endif

/*
 * Synopsys DWC DDR PHY APB: PA = csr*2 + 0xFE000000 (vendor FUN_fffd2228).
 * Control CSRs (MemResetL / MicroContMuxSel) are 16-bit strh/ldrh.
 * IMEM fill may need a different map — Solitude tip 9318987: csr*2 strh
 * poke/fill readback 0 (d0000 j / J j). Probe picks first sticky map.
 *
 * RE bl2 before IMEM (g12b bl2.c / amlddr.fw.c):
 *   FUN_fffa7e58 (CLK+SOFT_RST pulse, leave RST=1s, CNTL2=0x1f)
 *   → load_ddr_fw → aml_ddr driver_init (pinmux/PHY cfg)
 *   → FUN_fffd2280: MemResetL=2, MicroContMuxSel=0,
 *     FUN_fffd2228(src,0x8000,0x50000) ends MicroContMuxSel=1,
 *     same for DMEM @ 0x54000, MicroContMuxSel=0.
 * No extra APB enable between 7e58 and 2228 beyond aml_ddr PHY cfg.
 */
static void g12a_wdt_pet_safe(void);
static void g12a_spin(ulong n);
static void g12a_phy_reg_access_enable(void);

/*
 * The vendor BL2's EE watchdog clear (FUN_fffd00ec): G12A_WATCHDOG_CNTL_CLR
 * drops EN (bit 18) and CLKDIV_EN (bit 25), i.e. it STOPS the watchdog.  On a
 * resident TPL that armed the USB fallback (g12-resident.c) the watchdog must
 * keep counting through training, so G12A_WDT_KEEP() pets it and the clear is
 * skipped.  Without MESON_G12A_RESIDENT it is a constant false and the code is
 * what it was.
 */
#if defined(CONFIG_MESON_G12A_RESIDENT) && defined(CONFIG_TPL_BUILD)
bool g12a_resident_wdt_keep(void);
#define G12A_WDT_KEEP()	g12a_resident_wdt_keep()
#else
#define G12A_WDT_KEEP()	false
#endif

static void g12a_phy_write16(ulong addr, u16 v)
{
	writew(v, (void *)addr);
	dsb();
}

static u16 g12a_phy_read16(ulong addr)
{
	u16 v = readw((void *)addr);

	dsb();
	return v;
}

static void g12a_uart_hex16(u16 v)
{
	static const char *hex = "0123456789abcdef";

	g12a_uart_putc(hex[(v >> 12) & 15]);
	g12a_uart_putc(hex[(v >> 8) & 15]);
	g12a_uart_putc(hex[(v >> 4) & 15]);
	g12a_uart_putc(hex[v & 15]);
}

static void g12a_uart_hex8(u8 v)
{
	static const char *hex = "0123456789abcdef";

	g12a_uart_putc(hex[(v >> 4) & 15]);
	g12a_uart_putc(hex[v & 15]);
}

static void g12a_uart_hex32(u32 v)
{
	g12a_uart_hex16((u16)(v >> 16));
	g12a_uart_hex16((u16)v);
}

/*
 * Measurement only: G12A_STAGE_TS=1 (KCFLAGS) prints "TSM <tag> <cntpct>" at
 * each DRAM bring-up stage boundary, the 64-bit system counter (24 MHz) as
 * two hex words, so a boot's time can be split by stage.  Default 0, and then
 * g12a_tsm() compiles to nothing.
 */
#ifndef G12A_STAGE_TS
#define G12A_STAGE_TS		0
#endif
#if G12A_STAGE_TS
static void g12a_tsm(const char *tag)
{
	u64 v;

	asm volatile("isb; mrs %0, cntpct_el0" : "=r" (v));
	g12a_uart_puts("TSM ");
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex32((u32)(v >> 32));
	g12a_uart_hex32((u32)v);
	g12a_uart_puts("\n");
}
#else
#define g12a_tsm(tag)	do { } while (0)
#endif

/*
 * ---- HHI_MEM_PD_REG0 read probe ------------------------------------------
 *
 * 0xFF63C100 is HHI_MEM_PD_REG0 (hiu base 0xFF63C000 + word offset 0x40 << 2),
 * a MEMORY POWER-DOWN register: a set bit powers a block's SRAM down.  The
 * product BL2 read-modify-writes it at 0xFFFA7AC8-0xFFFA7AE0 on the USB DDR
 * path, ANDing with 0xFFFCFFFF -- i.e. clearing bits [17:16] -- before the
 * DDR blob is staged.  Our TPL never writes the register.
 *
 * This probe READS it and prints it.  It writes nothing, so it cannot
 * contaminate its own measurement -- the failure mode of three earlier probes
 * in this arc.  Two call sites: MPDE (early, before any DDR work) and MPDF
 * (immediately before the vendor loader's SRAM fill).
 *
 * See amlogic/software/boot/g12/non-phy-clock-diff.md §1.
 */
#define G12A_HHI_MEM_PD_REG0	0xFF63C100UL

static void g12a_mem_pd_report(const char *tag)
{
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)G12A_HHI_MEM_PD_REG0));
	g12a_uart_puts("\n");
}

/*
 * Measured 2026-08-11 on Solitude: our TPL reads 0x3FFFFFFF here at both the
 * early and the pre-fill instant; the product BL2 reads 0x3FFCFFFF at the
 * equivalent instant in a boot whose fill succeeds.  The two differ in bits
 * [17:16] and nowhere else -- exactly the AND mask BL2 applies.
 *
 * Mirror BL2's read-modify-write instruction for instruction (ldr / and
 * #0xfffcffff / str at 0xFFFA7AC8-0xFFFA7AE0): one 32-bit load, one AND, one
 * 32-bit store.  No other bit is touched, and the register is not
 * re-purposed as a status word -- the reads either side are printed
 * separately so the measurement, the write and the outcome stay distinct.
 */
static void g12a_mem_pd_power_up(void)
{
	u32 v;

	/* pre-write value: the measurement */
	g12a_uart_puts("MPD0 ");
	g12a_uart_hex32(readl((void *)G12A_HHI_MEM_PD_REG0));
	g12a_uart_puts("\n");

	/* the fix */
	v = readl((void *)G12A_HHI_MEM_PD_REG0);
	writel(v & 0xFFFCFFFFu, (void *)G12A_HHI_MEM_PD_REG0);

	/* post-write value: proof the store landed */
	g12a_uart_puts("MPD1 ");
	g12a_uart_hex32(readl((void *)G12A_HHI_MEM_PD_REG0));
	g12a_uart_puts("\n");
}

/*
 * ---- in-fill gate probe --------------------------------------------------
 *
 * Every gate comparison so far was taken OUTSIDE the vendor loader's copy
 * loop -- before 2280 and at the train kick -- and all seven candidate gates
 * came back byte-identical between a product boot that loads its microcode
 * and ours that does not.  That walk therefore cannot hold the cause.  This
 * samples the same gates from INSIDE the loop, on the iteration that actually
 * writes the SRAM.
 *
 * Mechanism: a trampoline over one instruction of the RAM copy of the blob,
 * at G12A_AML_LOOP_STORE (0xFFFB6218, the loop's `strh w6,[x4,x5]`).  The
 * stub samples, executes that same store verbatim, and branches to
 * G12A_AML_LOOP_NEXT.  It is the same vendor function at the same VA in both
 * worlds, so the two captures are comparable by construction the same way the
 * PCSR dumper's are; the product side installs the identical word array from
 * the BL2 pad-A installer at 0xFFFAF600, hooked at 0xFFFA90A8 (the state
 * 0x2002 USB-path 1D dispatch -- the media-path site 0xFFFA790C never fires
 * on a USB boot).
 *
 * WHAT THIS PROBE WRITES, exhaustively: its own 400-byte record buffer, the
 * one patched instruction word in the staged blob, and -- inside the stub --
 * the loader's original store, unmodified.  It writes NO PHY CSR, no clock,
 * no reset, and no UART from inside the fill (records go to RAM; the UART
 * print happens after 2280 returns).  Every register it reports is read-only
 * to it.  Three earlier probes in this arc contaminated their own measurement
 * by writing the thing they were measuring; this one cannot.
 *
 * NOTHING HERE CAN FAIL SILENTLY.  Install prints IFI (site word read, stub
 * checksum, stub VA) then either IFP (patched word read back) or IFX with a
 * reason.  The report prints IFN (iterations seen, records filled) even when
 * both are zero, followed by all four record slots regardless of whether they
 * were filled.  "The stub never ran" and "the patch never landed" and "the
 * loop ran but wrote nothing" are three distinct printed outcomes.
 *
 * Stub source: amlogic/software/boot/g12/in-fill-gate-probe.md (assembled
 * with aarch64-linux-gnu-as; word 41 is 0x78256886, byte-identical to the
 * instruction it replaces, which is the cheapest possible check that the
 * store was reproduced and not paraphrased).
 *
 * Registers: at the patch site x0-x6 and x30 are live; the stub touches only
 * x7-x13, which are AAPCS temps dead across the bl into the copy routine.
 * NZCV is dead there -- the next flag write is the cmp at 0xFFFB6204.
 *
 * DEFAULT 0 -- the experiment is concluded and this is a boot path, not a
 * bring-up instrument.  The probe survived the merge at 1 only because
 * flipping it there would have changed the TPL at the final step and
 * discarded the byte-identical-TPL evidence chain (merge-plan D5, follow-up
 * N21).  At 0 the whole block is preprocessed out, so a future bring-up
 * flips one macro and gets the identical instrument back.
 *
 * What its IFP line identified was NOT the image.  IFP <word> is the branch
 * word written over the loop store -- 0x14000000 | (((&g12a_ifp_stub -
 * 0xFFFB6218) / 4) & 0x03ffffff) -- i.e. the link address of ONE .data
 * symbol, which the IFI line prints in full anyway.  It is identical across
 * every build whose layout does not move that symbol (measured: 17ffc44e on
 * boots merge2-1, vpupd-3 and vpupd-8, three different trees) and identical
 * across a replay of the same image, so it is neither an image hash nor an
 * anti-replay instrument.  The SPL/U-Boot build stamp discriminates all three
 * of those boots, and the per-boot console START mark is what bounds replay.
 */
#ifndef G12A_INFILL_PROBE
#define G12A_INFILL_PROBE	0
#endif
#if G12A_INFILL_PROBE

#define G12A_IFP_WORDS		100	/* 400 B: 0xB4 code, data from 0x100 */
#define G12A_IFP_CODE_WORDS	45
#define G12A_IFP_CODE_SUM	0x8cb07914u
#define G12A_IFP_HDR		(0x100 / 4)	/* [0] iterations [1] slot */
#define G12A_IFP_RECS		(0x110 / 4)
#define G12A_IFP_NREC		4
#define G12A_IFP_RECW		8		/* 32-byte records */

/* Writable: the stub keeps its records in its own tail. */
static u32 g12a_ifp_stub[G12A_IFP_WORDS] = {
	0x10000007, 0xb94100e8, 0x11000508, 0xb90100e8,
	0xb4000083, 0xd1000429, 0xeb09007f, 0x54000441,
	0xb94104e9, 0x7100113f, 0x540003e2, 0x1100052a,
	0xb90104ea, 0x910440ea, 0x8b09154a, 0xb9000143,
	0x8b05008b, 0xb900054b, 0xb9000946, 0xd280000c,
	0xf2bfc34c, 0x7940018d, 0x7900194d, 0xd280264c,
	0xf2bfc34c, 0x7940018d, 0x79001d4d, 0xd280200c,
	0xf2bfc30c, 0x7940018d, 0x7900214d, 0xd280180c,
	0xf2bfc08c, 0x7940018d, 0x7900254d, 0xd28023cc,
	0xf2bfc08c, 0x7940018d, 0x7900294d, 0xb9001948,
	0xb9001d42, 0x78256886, 0xd28c4387, 0xf2bfff67,
	0xd61f00e0,
};

/**
 * g12a_ifp_install - patch the trampoline over the copy loop's store
 *
 * Must run after the blob is staged at 0xFFFB4000 and before 2280 is called.
 *
 * Return: 1 if the site was patched, 0 if it was left untouched (and why).
 */
static int g12a_ifp_install(void)
{
	void *site = (void *)G12A_AML_LOOP_STORE;
	u32 orig, patch, sum = 0;
	long off;
	int i;

	for (i = 0; i < G12A_IFP_CODE_WORDS; i++)
		sum += g12a_ifp_stub[i];
	for (i = G12A_IFP_HDR; i < G12A_IFP_WORDS; i++)
		g12a_ifp_stub[i] = 0;

	orig = readl(site);
	off = ((long)(ulong)g12a_ifp_stub - (long)G12A_AML_LOOP_STORE) / 4;
	patch = 0x14000000u | ((u32)off & 0x03ffffffu);

	g12a_uart_puts("IFI ");
	g12a_uart_hex32(orig);
	g12a_uart_putc(' ');
	g12a_uart_hex32(sum);
	g12a_uart_putc(' ');
	g12a_uart_hex32((u32)(ulong)g12a_ifp_stub);
	g12a_uart_puts("\n");

	if (sum != G12A_IFP_CODE_SUM) {
		g12a_uart_puts("IFX sum\n");
		return 0;
	}
	if (orig != G12A_AML_LOOP_STORE_INSN) {
		g12a_uart_puts("IFX site\n");
		return 0;
	}
	if (off > 0x1ffffffL || off < -0x2000000L) {
		g12a_uart_puts("IFX range\n");
		return 0;
	}

	writel(patch, site);
	dsb();
	asm volatile("ic iallu" ::: "memory");
	asm volatile("dsb sy" ::: "memory");
	asm volatile("isb" ::: "memory");

	g12a_uart_puts("IFP ");
	g12a_uart_hex32(readl(site));
	g12a_uart_puts("\n");
	return 1;
}

/**
 * g12a_ifp_report - print what the stub recorded
 *
 * Prints unconditionally, including the all-zero case, so silence is never a
 * possible outcome of this probe.
 *
 *   IFN <iterations> <records>
 *   IF<n> <idx> <destPA> <val> <mux>/<mrst>/<ucclk>/<memrstl>/<ucmem> \
 *         <seq> <dstcsrbase>
 */
static void g12a_ifp_report(void)
{
	int r;

	g12a_uart_puts("IFN ");
	g12a_uart_hex32(g12a_ifp_stub[G12A_IFP_HDR]);
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_ifp_stub[G12A_IFP_HDR + 1]);
	g12a_uart_puts("\n");

	for (r = 0; r < G12A_IFP_NREC; r++) {
		const u32 *p = &g12a_ifp_stub[G12A_IFP_RECS + r * G12A_IFP_RECW];

		g12a_uart_puts("IF");
		g12a_uart_putc((char)('0' + r));
		g12a_uart_putc(' ');
		g12a_uart_hex32(p[0]);		/* iteration index */
		g12a_uart_putc(' ');
		g12a_uart_hex32(p[1]);		/* store's own dest PA */
		g12a_uart_putc(' ');
		g12a_uart_hex16((u16)p[2]);	/* halfword being stored */
		g12a_uart_putc(' ');
		g12a_uart_hex16((u16)p[3]);		/* MicroContMuxSel */
		g12a_uart_putc('/');
		g12a_uart_hex16((u16)(p[3] >> 16));	/* MicroReset */
		g12a_uart_putc('/');
		g12a_uart_hex16((u16)p[4]);		/* UcclkHclkEnables */
		g12a_uart_putc('/');
		g12a_uart_hex16((u16)(p[4] >> 16));	/* MemResetL */
		g12a_uart_putc('/');
		g12a_uart_hex16((u16)p[5]);		/* UcMemResetControl */
		g12a_uart_putc(' ');
		g12a_uart_hex32(p[6]);		/* iteration counter at capture */
		g12a_uart_putc(' ');
		g12a_uart_hex32(p[7]);		/* dst CSR base 0x50000/0x54000 */
		g12a_uart_puts("\n");
	}
}
#else
static inline int g12a_ifp_install(void) { return 0; }
static inline void g12a_ifp_report(void) { }
#endif

/*
 * ---- PCSR capture (open side), v3 ----------------------------------------
 *
 * The PRODUCT half of this experiment lives in a BL2 pad-A patch
 * (csrdump.S); see amlogic/software/boot/g12/csr-state-diff-v3.md.
 * Everything below must stay byte-for-byte equivalent to it: same generated
 * sequence, same order, same CRC-32 definition, same halfword sum, same text.
 *
 * v3: the run list is GENERATED, not literal.  31 literal head words (the
 * whole v2 list, positions 0..30 unchanged) followed by group/range expansion
 * over MASTER (PState 0/1), DBYTE (instances 0..3, PState 0/1), ANIB
 * (instances 0..9, PState 0/1), ACSM and INITENG -- n = 1949.  `tbl=` is
 * CRC-32/ISO-HDLC over the GENERATED sequence, so a generator divergence
 * between the two worlds fails at the header instead of producing a
 * fictitious diff.
 *
 * TWO capture sites, matched pairwise to two product instruction addresses:
 *
 *   site "e6b0" -- product blob VA 0xFFFBE6B0, the `mov x0, x19` immediately
 *                  before `bl 0xFFFB6234` (the vendor IMEM+DMEM loader,
 *                  "2280") at 0xFFFBE6B4.  Open side: immediately before
 *                  `fn = step[7]`, which IS G12A_AML_FN_LOAD_IMEM.
 *
 *   site "8d7c" -- product blob VA 0xFFFB8D7C, inside 0xFFFB8D74 after
 *                  `stp x29,x30` / `mov x29,sp` and before
 *                  `MicroContMuxSel = 1`.  Open side: immediately before
 *                  `fn = step[8]`, which IS G12A_AML_FN_TRAIN.
 *
 * READS ONLY.  No CSR is written anywhere in this block.
 */
/* GENERATED by emit.py from gen.py -- do not edit. Mirrors csrtab.inc. */
static const u32 g12a_pcsr_head[] = {
	0x0C00EEu,
	0x0C00EDu,
	0x0200EEu,
	0x0200EDu,
	0x020004u,
	0x0D0037u,
	0x0D0000u,
	0x0D0004u,
	0x0D0030u,
	0x0D0031u,
	0x0D0032u,
	0x0D0034u,
	0x0D0035u,
	0x0D0037u,
	0x0D0099u,
	0x0D00E7u,
	0x0D00FAu,
	0x0C0004u,
	0x0C0030u,
	0x0C0032u,
	0x0C0033u,
	0x0C0034u,
	0x0C0035u,
	0x0C0036u,
	0x0C0080u,
	0x0C0081u,
	0x0C0095u,
	0x0C0096u,
	0x0C00EDu,
	0x0C00EEu,
	0x020060u,
};

struct g12a_pcsr_grp { u32 base; u16 ninst; u16 nrange; };
static const struct g12a_pcsr_grp g12a_pcsr_grp[] = {
	{ 0x020000u, 1, 34 },	/* MASTER ps0 */
	{ 0x120000u, 1, 17 },	/* MASTER ps1 */
	{ 0x010000u, 4, 90 },	/* DBYTE ps0 i0-3 */
	{ 0x110000u, 4, 29 },	/* DBYTE ps1 i0-3 */
	{ 0x000000u, 10, 6 },	/* ANIB ps0 i0-9 */
	{ 0x100000u, 10, 1 },	/* ANIB ps1 i0-9 */
	{ 0x040000u, 1, 5 },	/* ACSM ps0 */
	{ 0x090000u, 1, 21 },	/* INITENG ps0 */
};

struct g12a_pcsr_rng { u16 off; u16 cnt; };
static const struct g12a_pcsr_rng g12a_pcsr_rng[] = {
	/* MASTER ps0 */
	{ 0x000u, 6 },
	{ 0x007u, 2 },
	{ 0x00Au, 9 },
	{ 0x014u, 11 },
	{ 0x021u, 8 },
	{ 0x02Cu, 3 },
	{ 0x030u, 12 },
	{ 0x03Fu, 1 },
	{ 0x045u, 1 },
	{ 0x047u, 1 },
	{ 0x049u, 3 },
	{ 0x050u, 1 },
	{ 0x055u, 2 },
	{ 0x05Bu, 2 },
	{ 0x060u, 1 },
	{ 0x06Eu, 18 },
	{ 0x081u, 2 },
	{ 0x084u, 2 },
	{ 0x088u, 2 },
	{ 0x08Bu, 3 },
	{ 0x08Fu, 2 },
	{ 0x097u, 2 },
	{ 0x09Au, 5 },
	{ 0x0A8u, 1 },
	{ 0x0AEu, 1 },
	{ 0x0B0u, 1 },
	{ 0x0B2u, 1 },
	{ 0x0B4u, 10 },
	{ 0x0C3u, 34 },
	{ 0x0EDu, 15 },
	{ 0x0FFu, 11 },
	{ 0x110u, 10 },
	{ 0x11Bu, 32 },
	{ 0x13Cu, 2 },
	/* MASTER ps1 */
	{ 0x008u, 1 },
	{ 0x00Bu, 4 },
	{ 0x010u, 2 },
	{ 0x019u, 1 },
	{ 0x024u, 1 },
	{ 0x02Du, 2 },
	{ 0x056u, 1 },
	{ 0x07Cu, 2 },
	{ 0x0B0u, 1 },
	{ 0x0B2u, 1 },
	{ 0x0B4u, 1 },
	{ 0x0C5u, 1 },
	{ 0x0C7u, 1 },
	{ 0x0CAu, 1 },
	{ 0x0CCu, 1 },
	{ 0x0D2u, 1 },
	{ 0x0FAu, 1 },
	/* DBYTE ps0 i0-3 */
	{ 0x000u, 4 },
	{ 0x010u, 5 },
	{ 0x017u, 2 },
	{ 0x01Au, 1 },
	{ 0x01Fu, 2 },
	{ 0x024u, 1 },
	{ 0x026u, 1 },
	{ 0x028u, 1 },
	{ 0x02Au, 1 },
	{ 0x030u, 1 },
	{ 0x032u, 1 },
	{ 0x040u, 2 },
	{ 0x043u, 1 },
	{ 0x048u, 6 },
	{ 0x056u, 4 },
	{ 0x05Fu, 1 },
	{ 0x062u, 1 },
	{ 0x068u, 4 },
	{ 0x080u, 4 },
	{ 0x08Cu, 8 },
	{ 0x0A0u, 8 },
	{ 0x0AAu, 6 },
	{ 0x0B1u, 11 },
	{ 0x0C0u, 2 },
	{ 0x0D1u, 2 },
	{ 0x0E4u, 1 },
	{ 0x130u, 1 },
	{ 0x132u, 1 },
	{ 0x140u, 2 },
	{ 0x143u, 1 },
	{ 0x149u, 1 },
	{ 0x14Bu, 1 },
	{ 0x14Du, 1 },
	{ 0x15Fu, 1 },
	{ 0x162u, 1 },
	{ 0x168u, 4 },
	{ 0x180u, 4 },
	{ 0x18Cu, 8 },
	{ 0x1B2u, 3 },
	{ 0x1C1u, 1 },
	{ 0x1D1u, 2 },
	{ 0x230u, 1 },
	{ 0x232u, 1 },
	{ 0x240u, 1 },
	{ 0x262u, 1 },
	{ 0x268u, 4 },
	{ 0x2B2u, 3 },
	{ 0x2C1u, 1 },
	{ 0x330u, 1 },
	{ 0x332u, 1 },
	{ 0x340u, 1 },
	{ 0x362u, 1 },
	{ 0x368u, 4 },
	{ 0x3B2u, 3 },
	{ 0x3C1u, 1 },
	{ 0x430u, 1 },
	{ 0x432u, 1 },
	{ 0x440u, 1 },
	{ 0x462u, 1 },
	{ 0x468u, 4 },
	{ 0x4B2u, 3 },
	{ 0x4C1u, 1 },
	{ 0x530u, 1 },
	{ 0x532u, 1 },
	{ 0x540u, 1 },
	{ 0x562u, 1 },
	{ 0x568u, 4 },
	{ 0x5B2u, 3 },
	{ 0x5C1u, 1 },
	{ 0x630u, 1 },
	{ 0x632u, 1 },
	{ 0x640u, 1 },
	{ 0x662u, 1 },
	{ 0x668u, 4 },
	{ 0x6B2u, 3 },
	{ 0x6C1u, 1 },
	{ 0x730u, 1 },
	{ 0x732u, 1 },
	{ 0x740u, 1 },
	{ 0x762u, 1 },
	{ 0x768u, 4 },
	{ 0x7B2u, 3 },
	{ 0x7C1u, 1 },
	{ 0x830u, 1 },
	{ 0x832u, 1 },
	{ 0x840u, 1 },
	{ 0x862u, 1 },
	{ 0x868u, 4 },
	{ 0x8B2u, 3 },
	{ 0x8C1u, 1 },
	/* DBYTE ps1 i0-3 */
	{ 0x003u, 1 },
	{ 0x020u, 1 },
	{ 0x041u, 1 },
	{ 0x043u, 1 },
	{ 0x048u, 2 },
	{ 0x04Bu, 3 },
	{ 0x05Fu, 1 },
	{ 0x080u, 4 },
	{ 0x08Cu, 8 },
	{ 0x0AEu, 2 },
	{ 0x0C1u, 1 },
	{ 0x0D1u, 2 },
	{ 0x141u, 1 },
	{ 0x143u, 1 },
	{ 0x149u, 1 },
	{ 0x14Bu, 1 },
	{ 0x14Du, 1 },
	{ 0x15Fu, 1 },
	{ 0x180u, 4 },
	{ 0x18Cu, 8 },
	{ 0x1C1u, 1 },
	{ 0x1D1u, 1 },
	{ 0x2C1u, 1 },
	{ 0x3C1u, 1 },
	{ 0x4C1u, 1 },
	{ 0x5C1u, 1 },
	{ 0x6C1u, 1 },
	{ 0x7C1u, 1 },
	{ 0x8C1u, 1 },
	/* ANIB ps0 i0-9 */
	{ 0x01Au, 1 },
	{ 0x027u, 2 },
	{ 0x043u, 1 },
	{ 0x053u, 1 },
	{ 0x055u, 2 },
	{ 0x080u, 1 },
	/* ANIB ps1 i0-9 */
	{ 0x080u, 1 },
	/* ACSM ps0 */
	{ 0x000u, 146 },
	{ 0x0C0u, 1 },
	{ 0x0C2u, 1 },
	{ 0x0C8u, 1 },
	{ 0x0D0u, 48 },
	/* INITENG ps0 */
	{ 0x000u, 20 },
	{ 0x017u, 80 },
	{ 0x06Au, 8 },
	{ 0x073u, 1 },
	{ 0x077u, 16 },
	{ 0x088u, 1 },
	{ 0x08Au, 5 },
	{ 0x090u, 2 },
	{ 0x093u, 27 },
	{ 0x0AFu, 155 },
	{ 0x14Bu, 1 },
	{ 0x14Du, 1 },
	{ 0x14Fu, 23 },
	{ 0x169u, 8 },
	{ 0x172u, 1 },
	{ 0x176u, 1 },
	{ 0x178u, 2 },
	{ 0x17Cu, 8 },
	{ 0x185u, 18 },
	{ 0x201u, 8 },
	{ 0x2FFu, 1 },
};

/*
 * Sites 0/1 must match .Lhsite1 / .Lhsite2 in csrdump.S byte for byte.
 *
 * Sites 2/3 are new and have NO product-BL2 counterpart, deliberately: the
 * two questions they answer are both WITHIN-TPL differentials, so the
 * matched control is another arm of this same image rather than the product
 * BL2.  Both sites live in g12a_stage2_finalize(), which the compare arm and
 * the live arm both execute, so the instant is the same instruction in both.
 *
 *   site "ea94" -- immediately after T7 (fffb5bc4) returns and before T8/
 *                  T9/T10.  Blob D-VA fffbea94.  Diffing our-T7 against
 *                  blob-T7 here isolates T7 and nothing else: every other
 *                  step in the boot is byte-identical code.
 *
 *   site "eab0" -- immediately after T10 (fffb8af0) returns and before T11.
 *                  Blob D-VA fffbeab0.  This is the complete trained PHY
 *                  state, and it is BEFORE T12 puts the DMC in mission
 *                  mode, so a wide APB sweep here is still inside PUB
 *                  section 7.2.1 rule 1.  Diffing live against compare here
 *                  is the DTD question.
 */
#ifdef CONFIG_MESON_G12A_DRAM_OPEN
#define G12A_PCSR_NSITE		4
#else
#define G12A_PCSR_NSITE		2	/* keep the default arm byte-identical */
#endif

static const char *const g12a_pcsr_site[G12A_PCSR_NSITE] = {
	" site=8d7c call=",	/* 0: pre-kick,   product 0xFFFB8D7C */
	" site=e6b0 call=",	/* 1: pre-loader, product 0xFFFBE6B0 */
#ifdef CONFIG_MESON_G12A_DRAM_OPEN
	" site=ea94 call=",	/* 2: post-T7,    blob fffbea94 */
	" site=eab0 call=",	/* 3: post-T10,   blob fffbeab0 */
#endif
};

/* Walker state -- the C mirror of csrdump.S's x19/x20/x21/x23. */
struct g12a_pcsr_walk {
	u32 mode;		/* 0 = CRC only (no bus access), 1 = read+print */
	u32 crc;		/* raw CRC-32 accumulator */
	u32 sum;		/* running sum of read halfwords */
	u32 idx;		/* addresses emitted */
#ifdef CONFIG_MESON_G12A_DRAM_PROBE
	char line[136];		/* STACK: no writable BSS in TPL here */
	u32 llen;
#endif
};

/*
 * mode 2 = read into this RAM buffer instead of the uart: the boot-burst
 * console path silently drops ~50% of a 12 KB emission (measured on
 * nyc-slot5 n1294b-resm: BEGIN printed, every body chunk lost, boot
 * survived -- the death was capture loss, not a bus hang, proven by the
 * PX direct-read boot n1294b-resn). The body is replayed line-by-line,
 * each line twice, paced on CNTPCT, after the failure prints.
 */
/* CRC-32/ISO-HDLC, one little-endian u32 at a time (design F2). */
static void g12a_pcsr_emit(struct g12a_pcsr_walk *w, u32 csr)
{
	u32 b, i;

	if (!w->mode) {
		for (i = 0; i < 4; i++) {
			w->crc ^= (csr >> (i * 8)) & 0xFFu;
			for (b = 0; b < 8; b++)
				w->crc = (w->crc >> 1) ^
					 (0xEDB88320u & (~(w->crc & 1u) + 1u));
		}
		w->idx++;
		return;
	}
	{
#ifdef CONFIG_MESON_G12A_DRAM_PROBE
		static const char *hx = "0123456789abcdef";
		u16 v = 0;

		if (w->mode != 1u)
			goto crc_only;
		if ((w->idx & 255u) == 0) {
			g12a_uart_puts("CH CH ");
			g12a_uart_hex16(w->idx);
			g12a_uart_puts(" 0000\n");
		}
		v = g12a_phy_read16(G12A_DDR_PHY_APB_BASE + ((ulong)csr << 1));
		if (w->idx < 16u) {
			g12a_uart_puts("Pe Pe ");
			g12a_uart_hex16(v);
			g12a_uart_putc(' ');
			g12a_uart_hex16(w->idx);
			g12a_uart_puts(" 0\n");
		}
		w->sum += v;
		if (w->llen + 4 < sizeof(w->line) - 2) {
			w->line[w->llen++] = hx[(v >> 12) & 15];
			w->line[w->llen++] = hx[(v >> 8) & 15];
			w->line[w->llen++] = hx[(v >> 4) & 15];
			w->line[w->llen++] = hx[v & 15];
		}
		w->idx++;
		if ((w->idx & 31u) == 0) {
			int rep;
			u32 k, sg;

			for (rep = 0; rep < 2; rep++) {
				g12a_uart_puts("CK ");
				g12a_uart_hex16(w->idx >> 5);
				g12a_uart_putc(' ');
				for (k = 0; k < w->llen; k++)
					g12a_uart_putc(w->line[k]);
				g12a_uart_putc('\n');
				for (sg = 0; sg < 60000u; sg++)
					asm volatile("" : : : "memory");
			}
			w->llen = 0;
		}
		return;
crc_only:
		w->idx++;
		return;
#else
		u16 v = g12a_phy_read16(G12A_DDR_PHY_APB_BASE +
				       ((ulong)csr << 1));

		w->sum += v;
		g12a_uart_hex16(v);
		w->idx++;
		if ((w->idx & 31u) == 0)
			g12a_uart_puts("\n");
#endif
	}
}

/*
 * The generator.  Identical sequence to walk/emit in csrdump.S:
 *   31 head words, then for each group, for each instance, for each range,
 *   csr = base | (instance << 12) | (off + k).
 * base carries PState (22:20) and BlockType (19:16); no group has ninst > 10
 * and no base has PState > 1, so the Instance = 0xF and PState = 0b111
 * broadcast aliases are unreachable by construction.
 */
static void g12a_pcsr_walk(struct g12a_pcsr_walk *w)
{
	u32 g, inst, r, k, ri = 0;

	for (g = 0; g < sizeof(g12a_pcsr_head) / sizeof(g12a_pcsr_head[0]); g++)
		g12a_pcsr_emit(w, g12a_pcsr_head[g]);

	for (g = 0; g < sizeof(g12a_pcsr_grp) / sizeof(g12a_pcsr_grp[0]); g++) {
		const struct g12a_pcsr_grp *gr = &g12a_pcsr_grp[g];

		for (inst = 0; inst < gr->ninst; inst++) {
			for (r = 0; r < gr->nrange; r++) {
				const struct g12a_pcsr_rng *rg =
					&g12a_pcsr_rng[ri + r];

				for (k = 0; k < rg->cnt; k++)
					g12a_pcsr_emit(w, gr->base |
						       (inst << 12) |
						       (u32)(rg->off + k));
			}
			if (w->mode)
				g12a_wdt_pet_safe();
		}
		ri += gr->nrange;
	}
}

/* 32-bit sum of `n` halfwords at PA `pa` -- same algorithm as sum16 in .S */
static u32 g12a_pcsr_sum16(ulong pa, u32 n)
{
	u32 s = 0;
	u32 i;

	for (i = 0; i < n; i++) {
		s += g12a_phy_read16(pa + (ulong)i * 2);
		if ((i & 0xFFFu) == 0xFFFu)
			g12a_wdt_pet_safe();	/* product cannot pet; harmless */
	}
	return s;
}

static void g12a_pcsr_hexrun(ulong pa, u32 n)
{
	u32 i;

	for (i = 0; i < n; i++)
		g12a_uart_hex16(g12a_phy_read16(pa + (ulong)i * 2));
}

#ifdef CONFIG_MESON_G12A_DRAM_OPEN
/*
 * The DMC/PCTL side of the state diff.  The 1949-register PHY sweep covers
 * PA = 0xFE000000 + (csr << 1) and NOTHING else, so if it comes back
 * equivalent the remaining candidates are by definition outside it -- and
 * the largest of those is the DMC program.
 *
 * Windows, all of them ones this file already reads elsewhere, so no
 * undecoded offset is probed blind:
 *   0xFF638000  the T12 enable word and its neighbour
 *   0xFF638400  the MID1 window -- exactly what g12a_mid1_compute() emits
 *               (G12A_MID1_N = 52 registers live here)
 *   0xFF638600  the T10 handshake window (open-host-path.md sec.8.1)
 *   0xFF639340  T10's ds->dmc_remap copy (10 words) plus the geometry /
 *               size-detect window that follows it at 0xFF639368
 * Crumb: DMCW <pa> <w0> <w1> <w2> <w3>
 */
static void g12a_dmc_dump(void)
{
	static const struct { u32 base; u32 words; } win[] = {
		{ 0xFF638000u, 4 }, { 0xFF638400u, 64 },
		{ 0xFF638600u, 32 }, { 0xFF639340u, 24 },
	};
	u32 w, i, k;

	for (w = 0; w < sizeof(win) / sizeof(win[0]); w++) {
		for (i = 0; i < win[w].words; i += 4) {
			g12a_uart_puts("DMCW ");
			g12a_uart_hex32(win[w].base + i * 4);
			for (k = 0; k < 4; k++) {
				g12a_uart_putc(' ');
				g12a_uart_hex32(readl((void *)(ulong)
					(win[w].base + (i + k) * 4)));
			}
			g12a_uart_puts("\n");
		}
		g12a_wdt_pet_safe();
	}
}
#endif

#ifdef CONFIG_MESON_G12A_DRAM_OPEN
/*
 * 0xFF638430 is MID1's tm[64] entry (MID1(0xbd0) in g12a-ddr-timing.h, base
 * 0xFF639000).  The host harness computes 0x1e for the Solitude ACS and the
 * blob programs 0x1e, but the LIVE arm reads 0x00000000 there at the post-T10
 * instant while the compare arm reads 0x1e -- the ONE word that differs in
 * the whole DMC dump.  This crumb says WHERE it goes to zero.
 * Crumb: D430 <tag> <value>
 */
static void g12a_d430(const char *tag)
{
	g12a_uart_puts("D430 ");
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)0xFF638430UL));
	g12a_uart_puts("\n");
}
#endif

static void g12a_pcsr_capture(ulong ctl, int site)
{
	/*
	 * 'call' and its guard live in BSS, which used to sit at 0xFFFBC000 --
	 * INSIDE the 0xFFFB4000 + 0xC000 window g12a_clear_aml_ddr_scratch()
	 * wipes. (BSS is 0xfffb0000 on the residual chain now, but the guard
	 * stays: it is cheap and the hazard is one config line away.)
	 * Measured with BSS still inside the window: the first run
	 * printed "PCSR SKIP call=bf", i.e. the counter came up as garbage and
	 * gated the capture off. Self-heal by validating a magic before
	 * trusting the counter; garbage matching the magic is not credible.
	 * The product side needs no equivalent: its counters live in pad A,
	 * which nothing clears.
	 */
#ifdef CONFIG_MESON_G12A_DRAM_PROBE
	/*
	 * NO WRITABLE BSS at this point on the TPL residual chain:
	 * writable statics live at 0xFFFB0000 = DDR scratch, not backed
	 * until DDR trains (measured n1294b-res2g..res2s: the `call[]`
	 * self-heal store after `CB` = silent data abort, board frozen
	 * mid-capture every time). All capture state is therefore
	 * stack-local in PROBE builds.
	 */
	struct g12a_pcsr_walk w;
	u32 n, c, st;

	g12a_uart_puts("CB CB 00000 00\n");	/* entry, stack-safe */
#else
	static u32 call[G12A_PCSR_NSITE], guard;
	struct g12a_pcsr_walk w;
	u32 n, c, st;

	if (guard != 0x50435352UL) {	/* 'PCSR' */
		u32 k;

		guard = 0x50435352UL;
		for (k = 0; k < G12A_PCSR_NSITE; k++)
			call[k] = 0;
	}

	if (call[site]++) {
		g12a_uart_puts("PCSR SKIP");
		g12a_uart_puts(g12a_pcsr_site[site]);
		g12a_uart_hex8((u8)(call[site] - 1));
		g12a_uart_puts("\n");
		return;
	}
#endif

	/* Pass 0 -- CRC + count, no bus access at all. */
	g12a_wdt_pet_safe();
	w.mode = 0; w.crc = 0xFFFFFFFFu; w.sum = 0; w.idx = 0;
	g12a_pcsr_walk(&w);
	c = ~w.crc;
	n = w.idx;
	g12a_wdt_pet_safe();
	g12a_uart_puts("P0 P0 ");
	g12a_uart_hex16((u16)(c >> 16));
	g12a_uart_putc(' ');
	g12a_uart_hex16((u16)n);
	g12a_uart_puts(" 0\n");

	g12a_uart_puts("PCSR BEGIN v3 tbl=");
	g12a_uart_hex16((u16)(c >> 16));
	g12a_uart_hex16((u16)(c & 0xffff));
	g12a_uart_puts(" n=");
	g12a_uart_hex16((u16)n);
	g12a_uart_puts(g12a_pcsr_site[site]);
#ifdef CONFIG_MESON_G12A_DRAM_PROBE
	g12a_uart_hex8(0);
#else
	g12a_uart_hex8((u8)(call[site] - 1));
#endif
	g12a_uart_puts(" world=o ds4=EE c612=EE\n");
	(void)ctl;

	/* Pass 1 -- the same walk, buffered to RAM (mode 2) for the slow
	 * replay, or streamed (legacy arm).
	 */
#ifdef CONFIG_MESON_G12A_DRAM_PROBE
	w.llen = 0;
#endif
	w.mode = 1;
	w.sum = 0; w.idx = 0;
	g12a_pcsr_walk(&w);
#ifdef CONFIG_MESON_G12A_DRAM_PROBE
	g12a_uart_puts("PN ");
	g12a_uart_hex16(w.idx);
	g12a_uart_puts(" 000000\n");
	g12a_uart_puts("PN ");
	g12a_uart_hex16(w.idx);
	g12a_uart_puts(" 000000\n");
#endif
	if (w.idx & 31u) {
#ifdef CONFIG_MESON_G12A_DRAM_PROBE
		int rep;
		u32 k, sg;

		for (rep = 0; rep < 2; rep++) {
			g12a_uart_puts("CK ");
			g12a_uart_hex16(w.idx >> 5);
			g12a_uart_putc(' ');
			for (k = 0; k < w.llen; k++)
				g12a_uart_putc(w.line[k]);
			g12a_uart_putc('\n');
			for (sg = 0; sg < 60000u; sg++)
				asm volatile("" : : : "memory");
		}
#else
		g12a_uart_puts("\n");
#endif
	}
	g12a_wdt_pet_safe();

	g12a_uart_puts("PCSR SRAM st=PCSR SRAM st=");
	st = g12a_phy_read16(0xFE1A0132UL);	/* MicroReset, CSR 0xD0099 */
	g12a_uart_hex16((u16)st);
	g12a_uart_puts(" gated");
	if (0 && (st & 1u)) {
		u32 ic = g12a_pcsr_sum16(0xFE0A0000UL, 16384);
		u32 dc = g12a_pcsr_sum16(0xFE0A8000UL, 2048);

		g12a_uart_puts(" icsum=");
		g12a_uart_hex16((u16)(ic >> 16));
		g12a_uart_hex16((u16)(ic & 0xffff));
		g12a_uart_puts(" dcsum=");
		g12a_uart_hex16((u16)(dc >> 16));
		g12a_uart_hex16((u16)(dc & 0xffff));
		g12a_uart_puts(" i=");
		g12a_pcsr_hexrun(0xFE0A0000UL, 8);
		g12a_uart_puts(" iz=");
		g12a_pcsr_hexrun(0xFE0A7FF0UL, 8);
		g12a_uart_puts(" d=");
		g12a_pcsr_hexrun(0xFE0A8000UL, 16);
	} else {
		g12a_uart_puts(" gated");
	}
	g12a_uart_puts("\nPCSR END sum=");
	g12a_uart_hex16((u16)(w.sum >> 16));
	g12a_uart_hex16((u16)(w.sum & 0xffff));
	g12a_uart_puts("\n");
	g12a_wdt_pet_safe();
}
/* ---- end PCSR capture ---------------------------------------------------- */

#ifdef CONFIG_TPL_MESON_G12A_USB_RESIDUAL
/*
 * G12-C1 — residual-only PPT/DDC snapshot.  Same capture class as the
 * PCSR sites e6b0/8d7c: g12a_phy_read16 only, no CSR write, no mux steal.
 * Call after 1D+2D success / blob T7.  Mux=1 means the ARC owns the APB
 * (PUB 7.2.1); still read and report that as the result.
 *
 * Crumb: PPT <tag> mux <m> ps <ps0>/<ps1> st <d0> <d1> <d2> <d3>
 */
static void g12a_ppt_crumb(const char *tag)
{
	int i;

	g12a_uart_puts("PPT ");
	g12a_uart_puts(tag);
	g12a_uart_puts(" mux ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICROCONTMUXSEL));
	g12a_uart_puts(" ps ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_PPTTRAINSETUP));
	g12a_uart_putc('/');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_PPTTRAINSETUP_PS1));
	g12a_uart_puts(" st ");
	for (i = 0; i < 4; i++) {
		if (i)
			g12a_uart_putc(' ');
		g12a_uart_hex16(g12a_phy_read16(
			G12A_DDR_PHY_PPTCTLSTATIC +
			(ulong)i * G12A_DDR_PHY_PPTCTLSTATIC_STRIDE));
	}
	g12a_uart_puts("\n");
}
#endif

#ifdef CONFIG_MESON_G12A_PPT_DDC
/*
 * G12-C3 — opt-in DDC after 1D/2D.  T7 (blob or open) writes
 * PPTTrainSetup = 0; T10 already programmed DMC DFI_CTRL so the MC
 * acks dfi_phymstr in hardware.  Restore PptCtlStatic from the T8
 * ds[] snapshot (class 11), then write the ST phyinit default packing
 * to both PStates.  No SW ack loop.
 */
static void g12a_ppt_ddc_arm(void)
{
	int i;
	u8 *ds = (u8 *)G12A_DDR_SET_BASE;
	u16 v;

	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);

	for (i = 0; i < 4; i++) {
		v = readw((void *)(ds + G12A_DS_OFF_PPTCTLSTATIC + i * 2));
		if (v)
			g12a_phy_write16(G12A_DDR_PHY_PPTCTLSTATIC +
					 (ulong)i *
					 G12A_DDR_PHY_PPTCTLSTATIC_STRIDE, v);
	}

	v = G12A_DDR_PHY_PPTTRAINSETUP_ST_DDC;
	g12a_phy_write16(G12A_DDR_PHY_PPTTRAINSETUP, v);
	g12a_phy_write16(G12A_DDR_PHY_PPTTRAINSETUP_PS1, v);
	/* T7 / ST loadpieimage.c:176-178 write 3; keep that. */
	g12a_phy_write16(G12A_DDR_PHY_PPTTRAINSETUP2, 3);
	g12a_phy_write16(G12A_DDR_PHY_PPTTRAINSETUP2_PS1, 3);

#ifdef CONFIG_TPL_MESON_G12A_USB_RESIDUAL
	g12a_ppt_crumb("c3");
#endif
}
#endif

static u32 g12a_fw_magic_at(ulong base)
{
	return readl((void *)(base + G12A_DDRFW_MAGIC_OFF));
}

static int g12a_fw_magic_is(ulong base, u32 expect)
{
	return g12a_fw_magic_at(base) == expect;
}

/*
 * Word copy — no libc memcpy dependency in early SPL; keep it tiny.
 * len rounded up to 4; source/dest must be 4-byte aligned (AHB holds are).
 */
static void g12a_copy32(ulong dst, ulong src, ulong len)
{
	volatile u32 *d = (volatile u32 *)dst;
	volatile u32 *s = (volatile u32 *)src;
	ulong n = (len + 3) / 4;
	ulong i;

	for (i = 0; i < n; i++)
		d[i] = s[i];
}

static void g12a_clear_aml_ddr_scratch(void)
{
	volatile u32 *p = (volatile u32 *)G12A_AML_DDR_FW_BASE;
	ulong n = G12A_AML_DDR_FW_SCRATCH_SIZE / 4;
	ulong i;

	for (i = 0; i < n; i++)
		p[i] = 0;
}

/*
 * Is the aml_ddr PAYLOAD already at the linker base? The @DFM magic lives
 * in the 0x60 header we deliberately do not copy, so the check is the
 * image's own export table: payload +0x18 = LPDDR4/a644 entry.
 */
static int g12a_aml_payload_is_placed(void)
{
	u32 v = readl((void *)(G12A_AML_DDR_FW_BASE +
			       G12A_AML_FW_EXPORT_LPDDR4_OFF));

	return v == (u32)G12A_AML_FW_EXPORT_LPDDR4_VAL;
}

/*
 * If dest lacks magic but hold has it, copy hold → dest.
 * Returns 1 if dest has expected magic after path, else 0.
 */
static int g12a_load_blob(ulong dest, ulong hold, ulong blob_size, u32 magic)
{
	if (g12a_fw_magic_is(dest, magic))
		return 1;

	if (g12a_fw_magic_is(hold, magic)) {
		g12a_copy32(dest, hold, blob_size);
		return g12a_fw_magic_is(dest, magic);
	}

	return 0;
}

/*
 * Zero a region — no libc; AHB targets only.
 */
static void g12a_zero32(ulong dst, ulong len)
{
	volatile u32 *p = (volatile u32 *)dst;
	ulong n = (len + 3) / 4;
	ulong i;

	for (i = 0; i < n; i++)
		p[i] = 0;
}

static void g12a_poke8(ulong base, u32 off, u8 v)
{
	writeb(v, (void *)(base + off));
}

static void g12a_poke16(ulong base, u32 off, u16 v)
{
	writew(v, (void *)(base + off));
}

static void g12a_poke32(ulong base, u32 off, u32 v)
{
	writel(v, (void *)(base + off));
}

static void g12a_poke64(ulong base, u32 off, u64 v)
{
	writel((u32)v, (void *)(base + off));
	writel((u32)(v >> 32), (void *)(base + off + 4));
}

/*
 * Resolve aml_ddr function address: prefer image export table entry if it
 * lands inside the loaded blob; else fall back to RE-locked STP address.
 *
 * Image +0x60 is an 8-slot absolute ptr table linked for 0xFFFB4000
 * (measured g12a aml_ddr.fw). Slot indices are NOT identical to ctl[0..7]
 * (BL2 PTR_DAT_fffadcb0 is a separate rodata table); use known offsets.
 */
static ulong g12a_aml_fn_resolve(ulong fallback, u32 img_table_off)
{
	u32 p;

	/*
	 * Payload (not file) is at G12A_AML_DDR_FW_BASE, so the export table
	 * starts at +0x00 — callers pass file-relative offsets, rebase them.
	 */
	if (img_table_off >= G12A_DDRFW_HDR_SIZE && img_table_off < 0x100) {
		img_table_off -= (u32)G12A_DDRFW_HDR_SIZE;
		p = readl((void *)(G12A_AML_DDR_FW_BASE + img_table_off));
		if (p >= (u32)G12A_AML_DDR_FW_BASE &&
		    p < (u32)(G12A_AML_DDR_FW_BASE + G12A_AML_DDR_FW_BLOB_SIZE))
			return (ulong)p;
	}
	return fallback;
}

/*
 * LPDDR4 AC/CA pin-mux table MID2 programs into the PHY CA CSRs.
 * Source: product BL2 rodata, aml-s905d3-cc/bl2_new.bin file +0xd298
 * (D-VA 0xFFFAE298), stored to ctl+0x1a0 by the DramType dispatch at
 * 0xFFFA6ECC. Byte-identical at aml-a311d-cc/bl2_new.bin +0xced8.
 * Two groups of 10: a permutation of {0..5} then an identity tail.
 */
static const u8 g12a_ac_pinmux_lpddr4[G12A_AC_PINMUX_N] = {
	0x02, 0x03, 0x01, 0x00, 0x05, 0x04, 0x06, 0x07, 0x08, 0x09,
	0x01, 0x03, 0x05, 0x02, 0x04, 0x00, 0x06, 0x07, 0x08, 0x09,
};

/* Solitude LPDDR4 dq remap (timing.c) — 32 lanes */
static const u8 g12a_solitude_dq_remap[G12A_DS_DQ_REMAP_N] = {
	3, 2, 0, 1, 7, 6, 5, 4,
	10, 9, 14, 11, 8, 12, 13, 15,
	20, 21, 23, 22, 18, 17, 19, 16,
	28, 26, 25, 24, 31, 30, 27, 29,
};

/* Solitude ddr_dmc_remap[5] (timing.c) */
static const u32 g12a_solitude_dmc_remap[5] = {
	G12A_DS_DMC_REMAP0, G12A_DS_DMC_REMAP1, G12A_DS_DMC_REMAP2,
	G12A_DS_DMC_REMAP3, G12A_DS_DMC_REMAP4,
};

/*
 * Product ACS pstate0 from blx aml-s905d3-cc/bl2_new.bin @ +0xe2ec.
 * Product pad stride 0xc0 (3 pstates); open G12A_DDR_SET_SIZE is 0x498
 * (extra room for ac/dfi pinmux zeros). Offsets 0..0xbf layout-match open
 * poke path; dmc/dq byte-identical to timing.c tables. dmem_load_size=0
 * in product (open used to force 0x1000).
 */
#define G12A_PRODUCT_ACS_SIZE	0xc0U
static const u8 g12a_product_acs_p0[G12A_PRODUCT_ACS_SIZE] = {
	0x63, 0x66, 0x67, 0x2e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x01, 0x02, 0xf0,
	/* +0x12 dram_rank_config (product 0x03 = RANK01_CH01): rank_cfg() maps
	 * it into msgblock CsPresentChA/B; knob see g12a-ddr-fw.h */
	0x00, 0x0a, G12A_SOLITUDE_RANK_CFG, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x13, 0x61, 0x00,
	/* +0x38 DRAMFreq[0] (product 0x0648 = 1608): the one knob both fp_pll
	 * (PLL) and S3 (msgblock ds*2) read, so it must come from the macro */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	G12A_SOLITUDE_DRAM_FREQ_MHZ & 0xffu, G12A_SOLITUDE_DRAM_FREQ_MHZ >> 8,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0d, 0x01, 0x20, 0x01, 0x10, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00,
	0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x28, 0x00, 0x3c, 0x00, 0x50, 0x00, 0xff, 0x03, 0x00, 0x01,
	0x00, 0x01, 0xff, 0x01, 0xfa, 0x00, 0xb4, 0x00, 0xb4, 0x00,
	/* +0x6a max_core_timmming_frequency (product 00 00 = bin cap 1600):
	 * the timing-engine speed-bin cap, from the macro like +0x38 above */
	G12A_DS_MAX_CORE_TIMING_MHZ & 0xffu, G12A_DS_MAX_CORE_TIMING_MHZ >> 8,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0xc5, 0x1c, 0x94, 0x14, 0x0b, 0x80, 0x07, 0x23, 0x72, 0xd2, 0x6a, 0x2f, 0x38, 0xeb, 0xcd, 0x3b,
	0x9e, 0x35, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x02, 0x00, 0x01, 0x07, 0x06, 0x05, 0x04, 0x0a, 0x09, 0x0e, 0x0b, 0x08, 0x0c, 0x0d, 0x0f,
	0x14, 0x15, 0x17, 0x16, 0x12, 0x11, 0x13, 0x10, 0x1c, 0x1a, 0x19, 0x18, 0x1f, 0x1e, 0x1b, 0x1d,
};

/* Exp knobs — residual default OFF at link; these only change train path */
#ifndef G12A_BL2_ORDER_STEPWISE
/* 1 = walk a644's body once with crumbs instead of calling a644 */
#define G12A_BL2_ORDER_STEPWISE	1
#endif
#ifndef G12A_L13_RESERVED00
/*
 * Row L13 — the control on the per-bit commit bias.  0 = leave whatever the
 * blob's own prepare wrote into message-block byte 0, which is the production
 * default and is measured 0x00 on this part (message-block-transcription.md
 * §7.3, authored AND captured, "0 Reserved00 1 00 00 MATCH").
 *
 * lpddr4_1d commits the per-bit read delay as
 *
 *     RXPBDLYTG<sel>[dev][lane] = min(latched[dev][lane] + bias, 0x3f)  1d:0x5302
 *     bias = (msgblk[0].Reserved00 & 0xf) + 1                           1d:0x5294
 *
 * (pmu-fn5030-transcription.md §6.3), so on this part the additive term sits at
 * its minimum of 1 and every committed value is ODD — measured 96 of 96 across
 * three boots, with no fit (audit/2026-09-06-l12-perbit-commit-bias.md F8).
 * That decode has been READ but never CONTROLLED.  Setting this to 3 makes the
 * term 4, which must flip all 32 lanes to EVEN whatever the vendor's own
 * ±4-step per-lane latch noise does.  Bits [7:4] are left alone: 1d:0x3bbe
 * reads only bit 7, and the commit masks with & 0xf.
 *
 * Criterion and verdict table: audit/2026-09-06-l13-reserved00-control-prereg.md,
 * committed before this knob's first image existed.  Scorer:
 * scripts/l13-reserved00-score.py.
 */
#define G12A_L13_RESERVED00	0
#endif
#ifndef G12A_OPEN_PLL_SOFT_RST
/* 1 = open PLL+soft-rst (measured needed before topology; skip regressed at c)
 * 0 = residual+ctl only (hang after ACS dump; no E)
 *
 * T7 measured this at 0 and got no training at all, so the PAIR is
 * load-bearing -- but it gates TWO independent operations, so which half
 * carries that is a separate question.  The two halves below default to
 * this knob's value, so the bundled setting still selects both and the
 * default build is unchanged; set either half explicitly to run one arm.
 */
#define G12A_OPEN_PLL_SOFT_RST	1
#endif
#ifndef G12A_OPEN_PLL_INIT
/* 1 = run g12a_ddr_pll_init() (open DDR PLL programming + lock poll).
 * MEASURED NECESSARY (T7B, boots t7b-regonly/-regonlyb): at 0 with the other
 * half at 1 the chain HANGS INSIDE g12a_phy_reg_access_enable() -- crumb n
 * never prints, and it dies EARLIER than with both halves off.  A locked DDR
 * PLL is a precondition of the soft-reset half, which is what the `if
 * (pll_ok)` guard at the call site has always encoded.  a644's own fp_pll is
 * NOT a substitute: it runs later, inside the blob. */
#define G12A_OPEN_PLL_INIT	G12A_OPEN_PLL_SOFT_RST
#endif
#ifndef G12A_OPEN_PHY_REG_ACCESS
/* 1 = run g12a_phy_reg_access_enable() (DDR_CLK_CNTL, DMC soft-reset pulse,
 *     DDR_PHY_CTRL -- BL2 FUN_fffa7e58).
 * MEASURED NECESSARY (T7B, boot t7b-pllonly): at 0 with the PLL still locked,
 * the chain reaches the control block and stops at the ACS readback, the same
 * wall the both-off arm hits -- so this half is what the bundled knob was
 * buying. */
#define G12A_OPEN_PHY_REG_ACCESS	G12A_OPEN_PLL_SOFT_RST
#endif
#ifndef G12A_USE_PRODUCT_ACS
/* 1 = memcpy product ACS p0 into ddr_set (layout 0xc0 matches open poke) */
#define G12A_USE_PRODUCT_ACS	1
#endif

/*
 * ---------------------------------------------------------------------
 * aml_ddr.fw call-site retirement knobs.
 *
 * Each defaults to TODAY'S BEHAVIOUR and each is INDEPENDENT, so a failing
 * boot names ONE site.  Do NOT combine them into a single gate.
 *
 * G12A_KEEP_FP_PLL -- the fp_pll call (export[7], 0xFFFB9000) made
 *   unconditionally by g12a_try_aml_ddr_entry() before the open stage-1 walk.
 *   fp_pll's COMPLETE MMIO footprint is AM_DDR_PLL_CNTL0..CNTL6
 *   (0xFF638C00..0xFF638C18), a watchdog RMW at 0xFFD0F0D0 gated on
 *   HdtCtrl <= 9, a timer read and a console-gate read (g12a-fp-pll-re.md
 *   sec.2.6).  On this board every value it writes is already in the register:
 *
 *     - HdtCtrl is 0x0A (ds+0x11; product ACS byte 0x11 = 0x0a, re-poked
 *       below), so the watchdog RMW branch is NOT taken;
 *     - the CNTL1..6 image lives at ctl+0xB0 = G12A_DDR_PLL_PAR_BASE, poked
 *       below at +0x44..+0x58 with exactly G12A_DDR_PLL_CNTL1_DEF..CNTL6_DEF.
 *       fp_pll's selector is `blk[+68] != 0 || blk[+76] != 0`, and both are
 *       non-zero (0x00000004 / 0x08691c00), so it takes the use-the-image-
 *       as-is branch and writes back the six literals g12a_ddr_pll_init()
 *       already wrote;
 *     - CNTL0 is computed from DRAMFreq[ctl[0x1B9]] and PllBypassEn.  We poke
 *       ctl+0x1B9 = 0 below, ds+0x38 = 1608 and ds+0x40 = 0, which yields
 *       0x00100486 = G12A_DDR_PLL_VAL_1608 -- the literal
 *       g12a_ddr_pll_init() already asserted.
 *
 *   What fp_pll ALSO does is re-run the CNTL0 assert/release/assert lock pulse
 *   AFTER g12a_phy_reg_access_enable() pulsed the DMC soft-resets.  Whether
 *   the PLL needs that second cycle is NOT decidable from the source; that is
 *   what the =0 boot measures.  The skip arm reads the seven registers back so
 *   the verdict is a register comparison, not an outcome inference.
 *
 * G12A_KEEP_SLOT5 -- the slot5 call (export[5], 0xFFFB9DA8,
 *   G12A_AML_FN_MAIN) made by g12a_stage2_finalize().  It is the SIZE DETECT,
 *   not a memtest: it resolves the 0xFFFF density sentinels, walks the
 *   aliasing boundary and rewrites the DMC geometry word 0xFF639368.
 *   Setting it to 0 is only sound together with G12A_DS_SIZE_EXPLICIT=1.
 */
/*
 * G12A_OPEN_REPLAY forces all three off/open: mode A ships aml_ddr.fw as a
 * 156 B stub, so a blob call would branch into a zero-filled window.  The
 * replay arm therefore inherits `allblobsoff` by construction rather than by
 * command line -- and its ds+0x28/0x2a come from the snapshot, which carries
 * the measured 0x0800/0x0800 rather than the auto-size sentinels, so
 * G12A_DS_SIZE_EXPLICIT is redundant here (it is also overwritten by the
 * snapshot copy, which lands last).
 */
#ifndef G12A_KEEP_FP_PLL
#define G12A_KEEP_FP_PLL	(G12A_OPEN_REPLAY ? 0 : 1)
#endif
#ifndef G12A_KEEP_SLOT5
#define G12A_KEEP_SLOT5		1
#endif
#if G12A_OPEN_REPLAY && !defined(G12A_OPEN_SLOT5)
#define G12A_OPEN_SLOT5		1
#endif
#if G12A_OPEN_REPLAY && !defined(G12A_OPEN_SLOT6)
#define G12A_OPEN_SLOT6		1
#endif

/*
 * G12A_DS_SIZE_EXPLICIT -- state the DRAM density instead of asking slot5.
 *
 * 0 (default, today): leave the product ACS's auto-size sentinels
 *   ds+0x28 = ds+0x2a = 0xFFFF in place.  g12a_open_density_code() has no case
 *   for 0xFFFF, so BOTH channels take its fall-through arm, which returns
 *   0x005 -- the code for 0x1000 = 4096 MB per rank -- and g12a_open_s1()
 *   programs 0xFF639368 = 0x00E0002D, twice each rank's real size.  slot5 is
 *   then what corrects it.
 *
 * 1: poke the measured per-rank density after the ACS copy.  slot5's own
 *   console output on boot stage2-2 is the measurement --
 *     "auto size-- 65535 / DDR cs0 size: 2048MB / DDR cs1 size: 2048MB /
 *      DMC_DDR_CTRL: 00e00024"
 *   (dram-access-bringup.md).  2048 MB = 0x0800 -> density code 0x004, and
 *   re-running g12a_open_s1()'s five ADDs by hand with cs0 = cs1 = 0x0800 and
 *   this board's ds[14]=2 / ds[15]=0xF0 / ds[18]=3 gives exactly 0x00E00024 --
 *   the value the hardware is measured to end at.  So S1 alone programs the
 *   geometry slot5 would have discovered.
 *
 * NOTE THE POKE SITE.  G12A_USE_PRODUCT_ACS defaults to 1, so ds is a memcpy
 * of g12a_product_acs_p0[], whose bytes 0x28..0x2B are ff ff ff ff.  The
 * G12A_DS_CS0_MB / G12A_DS_CS1_MB constants in g12a-ddr-fw.h are consumed ONLY
 * by the G12A_USE_PRODUCT_ACS=0 arm and are INERT in the shipping image;
 * editing them changes nothing.  This knob pokes ds after both arms.
 */
#ifndef G12A_DS_SIZE_EXPLICIT
#define G12A_DS_SIZE_EXPLICIT	0
#endif
/**
 * g12a_piei_scan_ptrs - fill ctl +0x78..+0x90 from piei.fw FEED+magic slices
 *
 * RE: PIEI_prepare_ walks halfword 0xFEED adjacent to d333/d444/dl33/dl44.
 * On miss leave 0 (pinmux may still run from ACS alone).
 */
static void g12a_piei_scan_ptrs(ulong ctl)
{
	ulong base = G12A_PIEI_FW_BASE;
	ulong end = base + G12A_PIEI_FW_BLOB_SIZE - 8;
	ulong p;
	u32 mag;
	u16 hw0, hw1;

	if (!g12a_fw_magic_is(base, G12A_DDRFW_MAGIC_PIEI))
		return;

	/*
	 * Match vendor PIEI_prepare (bl2_new.bin D-VA 0xFFFA7568) exactly.
	 *
	 * It walks p from base+8 upward in EIGHT-byte steps and tests the tag
	 * at p-8 and 0xFEED at p-4, storing p — the payload start AFTER the
	 * 8-byte {tag,FEED} preamble. Our version walked 2-byte steps testing
	 * FEED FIRST and building the magic from p+2, i.e. the opposite byte
	 * order: the blob is [magic][FEED], not [FEED][magic]. It therefore
	 * matched nothing on every boot and fell through to a fallback that
	 * pointed dl44 at the container base.
	 *
	 * Why this was not fixed earlier: no consumer of ctl+0x78..0x90 could
	 * be found. The consumer is now identified — BL2's probe at
	 * 0xFFFA6D44 selects one of these four by DRAM type and stores it to
	 * ctl+0x98, and the blob dereferences ctl+0x98 (fffb5bc4:
	 * ldr x0,[x19,#152] then fffb5008: ldr w1,[x0], no null check).
	 * Leaving the scan broken leaves that pointer zero.
	 *
	 * Steps stay 8-byte aligned so readl is never issued at 2-mod-4
	 * (AHB unaligned hang on AArch64), which the old 2-byte walk risked.
	 */
	for (p = base + 8; p < end; p += 8) {
		if (readw((void *)(p - 4)) != 0xFEED)
			continue;
		hw0 = readw((void *)(p - 8));
		hw1 = readw((void *)(p - 6));
		mag = (u32)hw0 | ((u32)hw1 << 16);
		switch (mag) {
		case 0x33333364UL: /* 'd333' */
			g12a_poke64(ctl, G12A_CTL_OFF_PIEI_D333, p);
			break;
		case 0x34343464UL: /* 'd444' */
			g12a_poke64(ctl, G12A_CTL_OFF_PIEI_D444, p);
			break;
		case 0x33336c64UL: /* 'dl33' */
			g12a_poke64(ctl, G12A_CTL_OFF_PIEI_DL33, p);
			break;
		case 0x34346c64UL: /* 'dl44' LPDDR4 */
			g12a_poke64(ctl, G12A_CTL_OFF_PIEI_DL44, p);
			break;
		default:
			break;
		}
	}

	/*
	 * Vendor "probe" (0xFFFA6D44) then selects by DRAM type and stores the
	 * chosen pointer to ctl+0x98. Nothing in BL2 ever reads ctl+0x98 — the
	 * reader is the blob. LPDDR4 is dl44 here.
	 *
	 * Crumb: os <dl44 hi> <lo> — 0000c068-class means the scan found the
	 * real slice, 0000c000 means it fell back to the container base, and
	 * 00000000 means it found nothing at all.
	 */
	{
		u32 lo = readl((void *)(ctl + G12A_CTL_OFF_PIEI_DL44));
		u32 hi = readl((void *)(ctl + G12A_CTL_OFF_PIEI_DL44 + 4));

		if (lo || hi)
			g12a_poke64(ctl, G12A_CTL_OFF_PIEI_SEL,
				    ((u64)hi << 32) | lo);

		g12a_uart_puts("os ");
		g12a_uart_hex16((u16)(lo >> 16));
		g12a_uart_hex16((u16)(lo & 0xffff));
		g12a_uart_puts("\n");
	}
}

/*
 * Rank detection demoted the part: the cold re-pass must BE a cold pass
 * built as RANK0_CH01. g12a_ddr_ctl_build consults this after its ACS
 * memcpy/pokes and the cfg overlay. .data: set and read before
 * dram_init's .bss clear_loop, like g12a_mid1_live.
 */
static u8 g12a_rank_override __section(".data");

#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
/*
 * (a) cfg overlay. Overlay only the fields the host actually set
 * (non-G12A_CFG_PROBE_NONE). This does NOT touch the PLL frequency: the
 * DDR PLL runs G12A_SOLITUDE_DRAM_FREQ_MHZ regardless of the ds
 * advertised-frequency, so a freq-point sweep needs the PLL constant
 * moved too, not just this overlay.
 */
static void g12a_probe_cfg_overlay(ulong ds)
{
	ulong b = G12A_CFG_PROBE_BASE;
	u32 v;

	if (readl((void *)b) != G12A_CFG_PROBE_MAGIC) {
		g12a_uart_puts("cfgovh off\n");
		return;
	}
	g12a_uart_puts("cfgovh");
#define PROBE_OV(off, ds_off, w)					\
	do {								\
		v = readl((void *)(b + (off)));				\
		if (v != G12A_CFG_PROBE_NONE) {				\
			if ((w) == 1)					\
				writeb(v & 0xffu, (void *)(ds + (ds_off))); \
			else if ((w) == 2)				\
				writew(v & 0xffffu, (void *)(ds + (ds_off))); \
			else						\
				writel(v, (void *)(ds + (ds_off)));	\
			g12a_uart_putc('/');				\
			g12a_uart_hex32(v);				\
		}							\
	} while (0)
	if (g12a_rank_override)
		g12a_uart_puts("/ro");	/* rank/cs1: detection's answer, not the overlay's */
	else
		PROBE_OV(G12A_CFG_PROBE_RANK_CFG, G12A_DS_OFF_RANK_CFG, 1);
	PROBE_OV(G12A_CFG_PROBE_DDR_FUNC, G12A_DS_OFF_DDR_FUNC, 4);
	PROBE_OV(G12A_CFG_PROBE_RFC_TYPE, G12A_DS_OFF_RFC_TYPE, 1);
	PROBE_OV(G12A_CFG_PROBE_FREQ_MHZ, G12A_DS_OFF_DRAM_FREQ, 2);
	PROBE_OV(G12A_CFG_PROBE_LPDDR4X, G12A_DS_OFF_LPDDR4X, 1);
	PROBE_OV(G12A_CFG_PROBE_CS0_MB, G12A_DS_OFF_CS0_MB, 2);
	if (!g12a_rank_override)
		PROBE_OV(G12A_CFG_PROBE_CS1_MB, G12A_DS_OFF_CS1_MB, 2);
	PROBE_OV(G12A_CFG_PROBE_DISABLED_DBYTE, G12A_DS_OFF_DISABLED_DBYTE, 1);
#undef PROBE_OV
	g12a_uart_puts("\n");
}
#endif

/**
 * g12a_ddr_ctl_build - build vendor-style ddr_fw control block in AHB
 *
 * RE: bl2 FUN_fffa6488 + FUN_fffa6a60 (LPDDR4 branch) + aml_ddr_load.
 * Full Solitude ACS: ohms, odt, dmc_remap, dq_remap, ac/dfi pinmux (zeros
 * on Solitude), SequenceCtrl, need_2d, DMEM/PIEI TOC ptrs.
 *
 * Return: 1 on success (crumb B), 0 on missing magic (crumb b).
 */
static int g12a_ddr_ctl_build(void)
{
	ulong ctl = G12A_DDR_CTL_BASE;
	ulong ds = G12A_DDR_SET_BASE;
	ulong tim = G12A_DDR_TIM_BASE;
	ulong pll = G12A_DDR_PLL_PAR_BASE;
	ulong fn_pll, fn_train, fn_main, fn_post;
	u32 i;

	/*
	 * Payload form: the @DFM magic lives in the 0x60 header we do not
	 * copy, so presence is the export table's own LPDDR4 slot.
	 */
	if (!g12a_aml_payload_is_placed()) {
		g12a_uart_puts("b\n");
		return 0;
	}

	g12a_zero32(ctl, G12A_DDR_CTL_SIZE);
	g12a_zero32(ds, G12A_DDR_SET_SIZE);
	g12a_zero32(tim, G12A_DDR_TIM_SIZE);
	g12a_zero32(pll, G12A_DDR_PLL_PAR_SIZE);

	/*
	 * Solitude ddr_set_t: product bl2 ACS pad is 0xc0/entry (3 pstates);
	 * open buffer is 0x498 (pinmux tail zeros). Layout 0..0xbf matches.
	 */
#if G12A_USE_PRODUCT_ACS
	{
		volatile u8 *d = (volatile u8 *)ds;
		u32 j;

		for (j = 0; j < G12A_PRODUCT_ACS_SIZE; j++)
			d[j] = g12a_product_acs_p0[j];
		/* pinmux tail 0xc0.. already zeroed with ds */
		g12a_uart_puts("J\n"); /* product ACS p0 memcpy */
	}
	/* Hdt may be exp-overridden after product ACS */
	g12a_poke8(ds, G12A_DS_OFF_HDTCTRL, G12A_ACS_HDTCTRL);
	g12a_poke16(ds, G12A_DS_OFF_SEQCTRL, G12A_ACS_SEQCTRL_1D);
	g12a_poke16(ds, G12A_DS_OFF_SEQCTRL + 2, G12A_ACS_SEQCTRL_2D);
#else
	/* --- full Solitude ddr_set_t (timing.c first pstate) --- */
	g12a_poke32(ds, G12A_DS_OFF_MAGIC, G12A_DS_MAGIC);
	g12a_poke8(ds, G12A_DS_OFF_FAST_BOOT, 0); /* full train */
	g12a_poke32(ds, G12A_DS_OFF_DDR_FUNC, G12A_DS_DDR_FUNC);
	g12a_poke8(ds, G12A_DS_OFF_BOARD_ID, G12A_DS_BOARD_ID);
	g12a_poke8(ds, G12A_DS_OFF_VERSION, G12A_DS_VERSION);
	g12a_poke8(ds, G12A_DS_OFF_DRAM_TYPE, G12A_DS_DRAM_TYPE_LPDDR4);
	g12a_poke8(ds, G12A_DS_OFF_DISABLED_DBYTE, G12A_DS_DISABLED_DBYTE);
	g12a_poke8(ds, G12A_DS_OFF_IS_2T, 0);
	g12a_poke8(ds, G12A_DS_OFF_HDTCTRL, G12A_ACS_HDTCTRL);
	g12a_poke8(ds, G12A_DS_OFF_RANK_CFG, G12A_SOLITUDE_RANK_CFG);
	/* timing.h: dmem_load_size remains after #if0 imem fields — 4 KiB msg block */
	g12a_poke16(ds, G12A_DS_OFF_DMEM_LOAD_SZ, G12A_DS_DMEM_LOAD_SIZE);
	g12a_poke32(ds, G12A_DS_OFF_BASE_ADDR, 0);
	g12a_poke32(ds, G12A_DS_OFF_START_OFF, 0); /* G12 CFG_DDR_START_OFFSET=0 */
	g12a_poke16(ds, G12A_DS_OFF_CS0_MB, G12A_DS_CS0_MB);
	g12a_poke16(ds, G12A_DS_OFF_CS1_MB, G12A_DS_CS1_MB);
	g12a_poke16(ds, G12A_DS_OFF_SEQCTRL, G12A_ACS_SEQCTRL_1D);
	g12a_poke16(ds, G12A_DS_OFF_SEQCTRL + 2, G12A_ACS_SEQCTRL_2D);
	/* phy_odt_config_rank {0,0}; dfi_odt_config 0 — Solitude timing.c */
	g12a_poke8(ds, G12A_DS_OFF_PHY_ODT_RANK, 0);
	g12a_poke8(ds, G12A_DS_OFF_PHY_ODT_RANK + 1, 0);
	g12a_poke32(ds, G12A_DS_OFF_DFI_ODT, 0);
	g12a_poke16(ds, G12A_DS_OFF_DRAM_FREQ, (u16)G12A_SOLITUDE_DRAM_FREQ_MHZ);
	g12a_poke8(ds, G12A_DS_OFF_PLL_BYPASS, 0);
	g12a_poke8(ds, G12A_DS_OFF_RDBI_WR, 0);
	g12a_poke8(ds, G12A_DS_OFF_RFC_TYPE, G12A_DS_RFC_LPDDR4_8GB);
	g12a_poke8(ds, G12A_DS_OFF_LPDDR4X, 1);
	g12a_poke32(ds, G12A_DS_OFF_PLL_SSC, G12A_DS_PLL_SSC_MODE);
	/* 40Ω class SI from timing.c */
	g12a_poke16(ds, G12A_DS_OFF_CLK_DRV, 40);
	g12a_poke16(ds, G12A_DS_OFF_CS_DRV, 40);
	g12a_poke16(ds, G12A_DS_OFF_AC_DRV, 40);
	g12a_poke16(ds, G12A_DS_OFF_SOC_DRV_P, 40);
	g12a_poke16(ds, G12A_DS_OFF_SOC_DRV_N, 40);
	g12a_poke16(ds, G12A_DS_OFF_SOC_ODT_P, 40);
	g12a_poke16(ds, G12A_DS_OFF_SOC_ODT_N, 40);
	g12a_poke16(ds, G12A_DS_OFF_DRAM_DRV, 40);
	g12a_poke16(ds, G12A_DS_OFF_DRAM_ODT, 60);
	g12a_poke16(ds, G12A_DS_OFF_DRAM_AC_ODT, 80);
	g12a_poke16(ds, G12A_DS_OFF_SLEW_CLK, G12A_DS_SLEW_CLK);
	g12a_poke16(ds, G12A_DS_OFF_SLEW_CS, G12A_DS_SLEW_CS);
	g12a_poke16(ds, G12A_DS_OFF_SLEW_AC, G12A_DS_SLEW_AC);
	g12a_poke16(ds, G12A_DS_OFF_SLEW_DATA, G12A_DS_SLEW_DATA);
	g12a_poke16(ds, G12A_DS_OFF_VREF_OUT, 250);
	g12a_poke16(ds, G12A_DS_OFF_VREF_RX, 180);
	g12a_poke16(ds, G12A_DS_OFF_VREF_DRAM, 180);
	/* ac_trace_delay zeros; lpddr4 vout 1/3; bitTimeControl_2d=1 */
	g12a_poke8(ds, G12A_DS_OFF_LPDDR4_VOUT, 1);
	g12a_poke8(ds, G12A_DS_OFF_BITTIME_2D, 1);
	for (i = 0; i < 5; i++)
		g12a_poke32(ds, G12A_DS_OFF_DMC_REMAP + i * 4,
			    g12a_solitude_dmc_remap[i]);
	/* ca_remap zeros; dq_remap full table; ac/dfi pinmux zeros (timing.c) */
	for (i = 0; i < G12A_DS_DQ_REMAP_N; i++)
		g12a_poke8(ds, G12A_DS_OFF_DQ_REMAP + i,
			   g12a_solitude_dq_remap[i]);
	/* ac_pinmux[28] + dfi_pinmux[26] left 0 — Solitude tables are zero */
#endif

#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	/* (a) black-box cfg overlay: host uploads a cfg block each run. */
	g12a_probe_cfg_overlay(ds);
#endif

	if (g12a_rank_override) {
		g12a_poke8(ds, G12A_DS_OFF_RANK_CFG, G12A_DS_RANK_32B_R0_CH01);
		g12a_poke16(ds, G12A_DS_OFF_CS1_MB, 0);
	}

#if G12A_DS_SIZE_EXPLICIT
	/*
	 * State the density rather than shipping the 0xFFFF auto-size
	 * sentinels.  Deliberately AFTER both ACS arms — the product-ACS memcpy
	 * carries ff ff ff ff at 0x28..0x2b and would overwrite an earlier
	 * poke — and before g12a_open_s1() reads ds+0x28 / ds+0x2a.
	 */
	g12a_poke16(ds, G12A_DS_OFF_CS0_MB, G12A_DS_CS0_MB_EXPLICIT);
	g12a_poke16(ds, G12A_DS_OFF_CS1_MB, G12A_DS_CS1_MB_EXPLICIT);
	g12a_uart_puts("dsz\n");
#endif

#if G12A_OPEN_REPLAY
	/*
	 * MODE A.  The captured ddr_set_t replaces ds WHOLE and LAST.
	 *
	 * Last, because the snapshot is valid only paired with the exact head
	 * that produced it (ddr-capture-replay-procedure.md sec.7.3): every ds
	 * write above -- the product-ACS memcpy, the HdtCtrl/SequenceCtrl
	 * pokes, G12A_DS_SIZE_EXPLICIT -- is therefore overwritten rather than
	 * left standing beside it.
	 *
	 * Whole, byte for byte, never field-by-field: ds[248..1175] is an
	 * EXTENT WITH HOLES (330-331, 664-696, 726-727 + their idx-1 mirrors)
	 * that fffb5a54 never writes, and dram_bit_vref[32] at ds+664 sits in
	 * one of those gaps yet IS read in mode A.
	 *
	 * ds+4 (fast_boot[0]) is copied, not written: the byte is part of the
	 * struct and four sites branch on it.  It is PRINTED below.
	 */
	{
		volatile u8 *d = (volatile u8 *)ds;
		u32 j, bs = 0, ws = 0;

		for (j = 0; j < G12A_DS_SNAPSHOT_SIZE; j++)
			d[j] = g12a_ds_snapshot[j];
		for (j = 0; j < G12A_DS_SNAPSHOT_SIZE; j++)
			bs += readb((void *)(ds + j));
		for (j = 0; j < G12A_DS_SNAPSHOT_SIZE; j += 4)
			ws += readl((void *)(ds + j));

		g12a_uart_puts("RPds ");
		g12a_uart_hex32(bs);
		g12a_uart_putc(' ');
		g12a_uart_hex32(ws);
		g12a_uart_putc(' ');
		g12a_uart_hex8(readb((void *)(ds + G12A_DS_OFF_FAST_BOOT)));
		g12a_uart_putc(' ');
		g12a_uart_hex16(readw((void *)(ds + G12A_DS_OFF_CS0_MB)));
		g12a_uart_putc(' ');
		g12a_uart_hex16(readw((void *)(ds + G12A_DS_OFF_CS1_MB)));
		g12a_uart_putc(' ');
		g12a_uart_hex8(readb((void *)(ds + G12A_DS_OFF_DRAM_TYPE)));
		g12a_uart_puts("\n");
	}
#endif	/* G12A_OPEN_REPLAY */

	/* --- PLL param defaults (FUN_fffd504c empty-ACS path) --- */
	g12a_poke32(pll, 0x40, 0x18110400);
	g12a_poke32(pll, 0x44, 0x00000004);
	g12a_poke32(pll, 0x48, 0x00000000);
	g12a_poke32(pll, 0x4c, 0x08691c00);
	g12a_poke32(pll, 0x50, 0x88770290);
	g12a_poke32(pll, 0x54, 0x39272000);
	g12a_poke32(pll, 0x58, 0x50540000);

	/*
	 * Fn ptrs: RE STP addresses in loaded image (export table +0x60).
	 * LPDDR4: [3]=a644 (full train), [5]=5df4 (memtest) — NOT 4dc0.
	 * Table +0x98 is a B trampoline into PLL. Never +0x40 (sentinel).
	 */
	fn_pll = g12a_aml_fn_resolve(G12A_AML_FN_PLL, 0x98);
	fn_train = G12A_AML_FN_LPDDR4;	/* driver_init: FUN_fffda644 */
	fn_main = G12A_AML_FN_MAIN;	/* post-train mem detect/test */
	fn_post = G12A_AML_FN_POST;

	g12a_poke64(ctl, G12A_CTL_OFF_FN0, fn_pll);
	g12a_poke64(ctl, G12A_CTL_OFF_FN3, fn_train);	/* driver_init */
	g12a_poke64(ctl, G12A_CTL_OFF_FN5, fn_main);
	g12a_poke64(ctl, G12A_CTL_OFF_FN6, fn_post);
	/* [7] mid is BL2 FUN_fffa81b8 — leave 0; open path does not call it */

	/* DMEM / PMU msg block ptrs (BL2: all six → 0xFFFC8000) */
	for (i = 0; i < G12A_CTL_OFF_DMEM_N; i++)
		g12a_poke64(ctl, G12A_CTL_OFF_DMEM0 + i * 8, G12A_MSG_BLOCK_BASE);

	g12a_piei_scan_ptrs(ctl);

	g12a_poke64(ctl, G12A_CTL_OFF_DDR_SET, ds);

	/*
	 * Point ctl+0x1A0 / ctl+0x1B0 at our in-image tables, the way BL2 does
	 * (it fills them from its own rodata at 0xFFFAE298 / 0xFFFAE2B8).
	 *
	 * MID2 takes the AC/CA table from ds+0xC0 only when its first three
	 * bytes are non-zero, else from ctl+0x1A0 — and with neither set it
	 * walks physical address 0 and hangs. We had been papering over that
	 * by poking the table into ds+0xC0 late in the old entry path; the
	 * BL2-order path returns before that poke, so MID2 hung again. Setting
	 * the pointer here fixes it for every path and matches the vendor.
	 *
	 * The DQ table stays sourced from ds+0xa0, which MID3 prefers: ours is
	 * byte-identical to this board's own product ACS pstate-0 map, whereas
	 * BL2's 0xFFFAE2B8 is the SoC FALLBACK for boards that leave theirs
	 * zero. Pointing at the fallback would silently mis-train 15 of 32
	 * lanes. ctl+0x1B0 is set only as null-pointer hygiene.
	 */
	g12a_poke64(ctl, G12A_CTL_OFF_AC_PINMUX_PTR,
		    (ulong)&g12a_ac_pinmux_lpddr4[0]);
	g12a_poke64(ctl, G12A_CTL_OFF_DQ_REMAP_PTR,
		    (ulong)&g12a_solitude_dq_remap[0]);
	g12a_poke64(ctl, G12A_CTL_OFF_TIMING, tim);
	g12a_poke64(ctl, G12A_CTL_OFF_PLL_PAR, pll);

	g12a_poke8(ctl, G12A_CTL_OFF_NEED_2D, 1);	/* LPDDR4 2D */
	g12a_poke8(ctl, G12A_CTL_OFF_PHASE, 1);		/* load step = 1d */
	g12a_poke8(ctl, G12A_CTL_OFF_TYPE, G12A_DS_DRAM_TYPE_LPDDR4);
	g12a_poke8(ctl, G12A_CTL_OFF_CHAN, 0);
	g12a_poke8(ctl, G12A_CTL_OFF_SILENT, 0);
	g12a_poke8(ctl, G12A_CTL_OFF_HDT, G12A_ACS_HDTCTRL);
	g12a_poke8(ctl, G12A_CTL_OFF_LAST_STEP, 1);	/* last loaded = 1d */
	/*
	 * init_mode: 0 = normal train (a644 runs FUN_fffd2280 + 4dc0);
	 * 0xff = fastboot restore — a644 skips 2280/4dc0 and reloads margins.
	 */
	g12a_poke8(ctl, G12A_CTL_OFF_INIT_MODE, 0);

	/* TOC slots: open residual already placed blobs at RE dests */
	g12a_poke32(ctl, G12A_CTL_OFF_TOC_AML0, 0);
	g12a_poke32(ctl, G12A_CTL_OFF_TOC_AML0 + 4, (u32)G12A_AML_DDR_FW_BLOB_SIZE);
	g12a_poke64(ctl, G12A_CTL_OFF_TOC_AML0 + 8, G12A_AML_DDR_FW_BASE);

	g12a_poke32(ctl, G12A_CTL_OFF_TOC_PIEI, 0);
	/* container at the base, so the TOC size is the container size */
	g12a_poke32(ctl, G12A_CTL_OFF_TOC_PIEI + 4,
		    (u32)G12A_PIEI_FW_BLOB_SIZE);
	g12a_poke64(ctl, G12A_CTL_OFF_TOC_PIEI + 8, G12A_PIEI_FW_BASE);

	/* 1d vendor staging — driver_init copies 0xFFFC0000 → PHY IMEM */
	g12a_poke32(ctl, G12A_CTL_OFF_TOC_DL44, 0);
	g12a_poke32(ctl, G12A_CTL_OFF_TOC_DL44 + 4, (u32)G12A_LPDDR4_1D_FW_BLOB_SIZE);
	g12a_poke64(ctl, G12A_CTL_OFF_TOC_DL44 + 8, G12A_FW_HOLD_1D);

	g12a_poke32(ctl, G12A_CTL_OFF_TOC_DL40, 0);
	g12a_poke32(ctl, G12A_CTL_OFF_TOC_DL40 + 4, (u32)G12A_LPDDR4_2D_FW_BLOB_SIZE);
	g12a_poke64(ctl, G12A_CTL_OFF_TOC_DL40 + 8, G12A_FW_HOLD_1D);

	/* Type-select TOC pointers (LPDDR4: 1d=dl44, 2d=dl40) */
	g12a_poke64(ctl, G12A_CTL_OFF_TOC_SEL_1D, ctl + G12A_CTL_OFF_TOC_DL44);
	g12a_poke64(ctl, G12A_CTL_OFF_TOC_SEL_2D, ctl + G12A_CTL_OFF_TOC_DL40);

#if G12A_OPEN_REPLAY
	/*
	 * The three ctl bytes mode A owns.  No code we run reads +0x264 -- there
	 * is no a644 on this arm -- but g12a_open_t10() gates its host MR-init
	 * arm on the same field the blob did (fffb8c0c), so it is the field, not
	 * a private flag.
	 *   +0x264 (612) = 0xFF   MODE A            (fffbe690/fffbe694)
	 *   +0x267 (615) = 0      run the JEDEC power-up reset window in the MR
	 *                         init (fffb8c5c -> fffbb394)
	 *   +0x1CD (461) = 0      empty user-CSR-override list, so T9 (fffbbbb0)
	 *                         has nothing to do and is omitted
	 */
	g12a_poke8(ctl, G12A_CTL_OFF_INIT_MODE, 0xff);
	g12a_poke8(ctl, 615, 0);
	g12a_poke8(ctl, 461, 0);
	g12a_uart_puts("RPv ");
	g12a_uart_hex8(readb((void *)(ctl + G12A_CTL_OFF_INIT_MODE)));
	g12a_uart_putc(' ');
	g12a_uart_hex8(readb((void *)(ctl + 615)));
	g12a_uart_putc(' ');
	g12a_uart_hex8(readb((void *)(ctl + 461)));
	g12a_uart_puts("\n");
#endif

	g12a_train_status_or(G12A_TRAIN_ST_CTL_OK | G12A_TRAIN_ST_ACS_TABLES);
	g12a_uart_puts("B\n");
	/* ACS readback: freq seq hdt rank type — catches poke-offset bugs */
	{
		u16 f = readw((void *)(ds + G12A_DS_OFF_DRAM_FREQ));
		u16 s = readw((void *)(ds + G12A_DS_OFF_SEQCTRL));
		u8 h = readb((void *)(ds + G12A_DS_OFF_HDTCTRL));
		u8 r = readb((void *)(ds + G12A_DS_OFF_RANK_CFG));
		u8 t = readb((void *)(ds + G12A_DS_OFF_DRAM_TYPE));

		g12a_uart_puts("c ");
		g12a_uart_hex16(f);
		g12a_uart_puts(" ");
		g12a_uart_hex16(s);
		g12a_uart_puts(" ");
		g12a_uart_hex8(h);
		g12a_uart_puts(" ");
		g12a_uart_hex8(r);
		g12a_uart_puts(" ");
		g12a_uart_hex8(t);
		g12a_uart_puts("\n");
	}
	return 1;
}

/**
 * g12a_phy_dmem_dump - read the message block back out of PHY DMEM
 *
 * Sourced from the PHY APB window (PA = 0xFE0A8000 + byte_off), not from the
 * AHB staging copy.
 * Tag + 7 halfwords: Freq Dfi Seq Hdt DQa/CSa ChB56 ChB58.
 */
static void g12a_phy_dmem_dump(const char *tag)
{
	static const u16 off[7] = {
		0x06, 0x08, 0x10, 0x12, 0x24, 0x56, 0x58
	};
	int i;

	g12a_uart_puts(tag);
	for (i = 0; i < 7; i++) {
		g12a_uart_puts(" ");
		g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_DMEM_BASE +
						off[i]));
	}
	g12a_uart_puts("\n");
}

/*
 * Removed with the DDR bring-up arc (see the arc status doc): the CA-only
 * SequenceCtrl experiment (G12A_EXP_SEQ_CA_ONLY / g12a_patch_a644_ca_only)
 * and the Solitude message-block force (g12a_msg_block_force_solitude).  The
 * force was RETIRED by the 0x60 load-base fix -- every value it wrote was
 * chosen to paper over data the vendor prepare was reading 0x60 early, so it
 * undid correct work (prepare writes DRAMFreq 0x0c90 = 3216 MT/s; the force
 * put back 0x0648 = 1608, and PhyVref 0x20 over the vendor-computed 0x17).
 * Both knobs were 0 at link, so nothing the board runs changes.
 */

/*
 * ---------------------------------------------------------------------
 * DRAM access test — runs ONLY after the DMC "go" step (T12).  Before
 * that the address space is not decoded and any access there is
 * meaningless.
 *
 * Three tests, each with a defect class it is the only one to catch:
 *
 *  A  coarse aliasing walk — 64 MiB stride, 62 points, 0 .. 3904 MiB.
 *     Every point carries a value derived from its OWN address, and ALL
 *     points are written before ANY is read.  A wrong address decode
 *     (two addresses reaching the same cell) therefore shows up as a
 *     mismatch, not as a value that happens to read back.  The span is
 *     what makes a size/rank misconfiguration visible: the product
 *     reports 3928 MB on this unit, so a part configured as one rank or
 *     half the density fails in the upper half of this walk.
 *  B  fine aliasing walk — 4 KiB stride over the first 16 MiB, same
 *     write-all-then-read-all rule, at row/column granularity.
 *  C  contiguous 1 MiB at 0x10000000 filled from a non-repeating LCG —
 *     catches stuck bits and data-bus shorts that an address-derived
 *     pattern can alias through.
 *
 * Nothing here writes or compares zeros: a zero-vs-zero result cannot
 * distinguish working DRAM from an undecoded bus that returns 0.
 */
#define G12A_DRAM_TEST_STRIDE_A		0x04000000UL
#define G12A_DRAM_TEST_N_A		62
#define G12A_DRAM_TEST_STRIDE_B		0x00001000UL
#define G12A_DRAM_TEST_N_B		4096
#define G12A_DRAM_TEST_C_BASE		0x10000000UL
#define G12A_DRAM_TEST_C_WORDS		0x00040000UL	/* 1 MiB */

/*
 * The one megabyte the memtest must NOT write -- G12A/G12B/SM1's counterpart
 * of dram-gx.c's MESON_DRAM_NOTEST (decision A6).
 *
 * ramoops exists to survive a reset, so a write+read test over it is
 * self-defeating: it erases pstore on every boot, destroying the log of the
 * crash it was meant to preserve.  Under CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT
 * ramoops is the TOP megabyte of the reserved block, 0x00F00000..0x00FFFFFF --
 * GXL's address exactly -- and every one of the walks below starts at base 0,
 * so the fine 4 KiB walk puts its last 256 iterations inside it.  Exact
 * arithmetic, all four walks:
 *
 *   DTA  base 0 stride 0x04000000 n=62    -> none          0 skipped
 *   DTB  base 0 stride 0x00001000 n=4096  -> i=3840..4095 256 skipped
 *   DTC  base 0x10000000, 1 MiB LCG       -> disjoint       0 skipped
 *   DTD  base 0 stride 0x00100000 n=3928  -> i=15 only      1 skipped
 *   DTD2 as DTD (CONFIG_MESON_G12A_DRAM_OPEN)               1 skipped
 *   DTE  base 0xF5000000 stride 0x1000    -> disjoint       0 skipped
 *
 * i.e. 258 of 12 174 test addresses, 8 bytes each.  Against the earlier
 * bottom-megabyte placement the shape flips rather than shifts: DTA now TESTS
 * address 0 (ordinary DRAM on this family, unlike GXL's ROM mirror), and DTB
 * covers 3840 points over 0x00000000..0x00EFF000 -- the first 15 MiB entire --
 * losing its tail instead of its head.  The coarse walks lose only the stride
 * point at 0x00F00000, whose megabyte DTB covers densely anyway.
 *
 * The skip is applied inside g12a_dram_walk() rather than at each call site so
 * a walk added later inherits it, and it guards the READ loop as well as the
 * WRITE loop -- reading is harmless but comparing against a pattern that was
 * never written would report the whole region as bad.
 *
 * The address is tested, not the index, because the two words a walk touches
 * are `a` and `a + 4`: every stride here is a multiple of 0x1000, so `a` in
 * the region implies `a + 4` in it, and the skip is exact rather than
 * conservative.
 *
 * Scoped to the libre layout because that is what puts ramoops at
 * G12A_LIBRE_RAMOOPS_BASE.  With the symbol off this is the constant 0 and the
 * generated code is unchanged -- the TPL links against a 0xf000 amlimage
 * payload ceiling and cannot afford dead weight.
 *
 * NOT covered, deliberately and stated rather than hidden:
 *   - the vendor size-detect (slot5, reached through G12A_AML_FN_MAIN) walks
 *     address bits 2..29 writing 0xAABBCCDD.  It is inside aml_ddr.fw and
 *     cannot be bounded from source -- but with ramoops at 0x00F00000 no
 *     2^b point can reach it (2^23 = 0x800000 falls short, 2^24 = 0x1000000 is
 *     the block limit), which retires the 18-points-inside-pstore exposure the
 *     bottom-megabyte placement had put the reboot log at.  The
 *     pstore-survives-reboot test is what measures the residue.
 *   - CONFIG_MESON_G12A_DRAM_MARGIN / _WIDTH (the -libre-margin and
 *     -libre-width diagnostic targets) run their own stress spans and are not
 *     part of the -libre boot chain.
 *
 * amlogic/software/boot/design/dram-reservations.md sections 8c A6, 10b, 10i.
 */
#if defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT)
#define G12A_DRAM_NOTEST(a)						\
	((ulong)(a) >= (ulong)G12A_LIBRE_RAMOOPS_BASE &&		\
	 (ulong)(a) < (ulong)G12A_LIBRE_RAMOOPS_BASE +			\
		      (ulong)G12A_LIBRE_RAMOOPS_SIZE)
#else
#define G12A_DRAM_NOTEST(a)		0
#endif

static u32 g12a_dram_pat(ulong a)
{
	return (u32)a ^ 0x5A5A5A5Au;
}

static void g12a_dram_fail(const char *tag, ulong a, u32 exp, u32 got)
{
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex32((u32)a);
	g12a_uart_putc(' ');
	g12a_uart_hex32(exp);
	g12a_uart_putc(' ');
	g12a_uart_hex32(got);
	g12a_uart_puts("\n");
}

static u32 g12a_dram_walk(const char *tag, ulong base, ulong stride, u32 n)
{
	u32 i, bad = 0, skipped = 0;

	for (i = 0; i < n; i++) {
		ulong a = base + (ulong)i * stride;

		if (G12A_DRAM_NOTEST(a)) {
			skipped++;
			continue;
		}
		writel(g12a_dram_pat(a), (void *)a);
		writel(~g12a_dram_pat(a), (void *)(a + 4));
		if ((i & 0xFF) == 0)
			g12a_wdt_pet_safe();
	}
	g12a_wdt_pet_safe();

	/*
	 * Report the skip on the wire, not just in the source.  This is the
	 * only runtime witness that the ramoops bound is compiled in -- the
	 * constants fold, so a stripped SPL carries no immediate to grep for.
	 * Crumb: "dns <tag> <skipped>".  Absent with the gate off.
	 */
	if (skipped) {
		g12a_uart_puts("dns ");
		g12a_uart_puts(tag);
		g12a_uart_putc(' ');
		g12a_uart_hex32(skipped);
		g12a_uart_puts("\n");
	}

	for (i = 0; i < n; i++) {
		ulong a = base + (ulong)i * stride;
		u32 e0, g0, g1;

		if (G12A_DRAM_NOTEST(a))
			continue;

		e0 = g12a_dram_pat(a);
		g0 = readl((void *)a);
		g1 = readl((void *)(a + 4));

		if (g0 != e0 || g1 != ~e0) {
			if (bad < 8)
				g12a_dram_fail(tag, a, e0, g0);
#ifdef CONFIG_MESON_G12A_DRAM_OPEN
			/*
			 * Every failure, compactly, as the two per-word
			 * error vectors got^exp.  The first-8 verbose form
			 * above prints word 0 even when only word 1 failed,
			 * and caps at 8 -- neither of which can answer
			 * "do the failures cluster by lane, rank or bank".
			 * A zero vector means that word was correct.
			 *
			 * r0/r1 are a SECOND read of the same two words,
			 * taken immediately.  A persistent error (r == x)
			 * means the cell holds the wrong data -- the write
			 * was lost or landed elsewhere.  A vanishing error
			 * (r == 0) means the first read itself was wrong,
			 * which is a read-path/timing signature and a
			 * completely different fault class.
			 * Crumb: dx <tag> <addr> <x0> <x1> <r0> <r1>
			 */
			if (bad < 256) {
				u32 r0 = readl((void *)a) ^ e0;
				u32 r1 = readl((void *)(a + 4)) ^ ~e0;
				u32 y0, y1;

				/*
				 * y0/y1 -- re-WRITE the same two words and
				 * read them again.  If the retry lands
				 * (y == 0) the cell is functional and the
				 * original write was lost in flight, i.e. a
				 * marginal command/address cycle.  If it
				 * still fails, the address itself is broken.
				 */
				writel(e0, (void *)a);
				writel(~e0, (void *)(a + 4));
				y0 = readl((void *)a) ^ e0;
				y1 = readl((void *)(a + 4)) ^ ~e0;

				g12a_uart_puts("dx ");
				g12a_uart_puts(tag);
				g12a_uart_putc(' ');
				g12a_uart_hex32((u32)a);
				g12a_uart_putc(' ');
				g12a_uart_hex32(g0 ^ e0);
				g12a_uart_putc(' ');
				g12a_uart_hex32(g1 ^ ~e0);
				g12a_uart_putc(' ');
				g12a_uart_hex32(r0);
				g12a_uart_putc(' ');
				g12a_uart_hex32(r1);
				g12a_uart_putc(' ');
				g12a_uart_hex32(y0);
				g12a_uart_putc(' ');
				g12a_uart_hex32(y1);
				g12a_uart_puts("\n");
			}
#endif
			bad++;
		}
		if ((i & 0xFF) == 0)
			g12a_wdt_pet_safe();
	}
	return bad;
}

static u32 g12a_dram_lcg(void)
{
	ulong a = G12A_DRAM_TEST_C_BASE;
	u32 i, bad = 0, v = 0x13579BDFu;

	for (i = 0; i < G12A_DRAM_TEST_C_WORDS; i++) {
		v = v * 1103515245u + 12345u;
		writel(v, (void *)(a + (ulong)i * 4));
		if ((i & 0x3FFF) == 0)
			g12a_wdt_pet_safe();
	}
	v = 0x13579BDFu;
	for (i = 0; i < G12A_DRAM_TEST_C_WORDS; i++) {
		u32 g;

		v = v * 1103515245u + 12345u;
		g = readl((void *)(a + (ulong)i * 4));
		if (g != v) {
			if (bad < 8)
				g12a_dram_fail("dcf", a + (ulong)i * 4, v, g);
			bad++;
		}
		if ((i & 0x3FFF) == 0)
			g12a_wdt_pet_safe();
	}
	return bad;
}

/*
 * ---------------------------------------------------------------------
 * CAPACITY ACCOUNTING probe (DTD / DTE)
 *
 * DTA/DTB/DTC walk 0 .. DETECTED span (the DTS bound in g12a_dram_test();
 * their fixed tables top out at 3904 MiB), i.e. only what slot5 REPORTED.
 * By construction they cannot say anything about memory that exists but was
 * never enumerated, and the part is 2048 + 2048 = 4096 MB while slot5
 * prints 3928 MB.  These two answer the remaining questions:
 *
 *  D  full-span coarse walk, 1 MiB stride, 3928 points, 0 .. 0xF5700000
 *     on SM1 (3920 points, 0 .. 0xF4F00000 on G12B).  Same write-ALL-
 *     then-read-ALL rule as DTA, so it is aliasing-sensitive over the
 *     WHOLE enumerated space, not just its lower 3904 MiB.
 *  E  the 8 MiB tail 0xF5000000 .. 0xF57FF000 at 4 KiB stride.  U-Boot
 *     proper's get_effective_memsize() clamps to 0xF5000000 and its
 *     mem_map stops there, so this window is enumerated by the DDR
 *     firmware and thrown away by the bootloader.  Does it work?
 *     SM1 only -- on G12B that window is A73_DBG, not DRAM, and the
 *     clamp is exactly right.  See G12A_DRAM_APERTURE_BASE below.
 *
 * A third walk, DTF, asked whether any DRAM is reachable above the 32-bit
 * window.  It was run once (boot cap-1) and the question is CLOSED: the
 * 64-bit read at 0x1_0000_0000 never returned and did not wrap to physical
 * 0, so bit 32 is undecoded -- there is no high alias and no remap window,
 * and the shadowed 168 MB is structurally unrecoverable.  The probe is
 * removed with its conclusion recorded in
 * amlogic/software/boot/g12/dram-capacity-accounting.md §4; it hung the
 * board every time it ran, which is the other reason not to leave it here.
 */
/*
 * G12A_DRAM_APERTURE_BASE -- the lowest address the SoC does NOT decode as
 * DRAM, and therefore the exclusive top of every walk below.  It is a
 * per-SoC number, and it is 8 MiB apart on the two families:
 *
 *   A311D (G12B), amlogic/software/ref/datasheets/a311d-reference.txt:
 *	ADDRSPACE 0xF5000000 0xF57FFFFF  8M  A73_DBG	<-- lowest non-DRAM row
 *	ADDRSPACE 0xF5800000 0xF5FFFFFF  8M  A53_DBG
 *	... FLASH 64M, reserved 32M, PCIE_AXI 32M, DDR_CTRL 16M, periph 16M
 *	0x1_0000_0000 - 0xF5000000 = 0x0B000000 = 176 MiB, so a 4096 MB part
 *	enumerates 4096 - 176 = 3920 MB.
 *   S905D3 (G12A/SM1): quad-A55, no A73 cluster and therefore no A73_DBG.
 *	Its lowest non-DRAM row is A53_DBG at 0xF5800000: 168 MiB, 3928 MB
 *	(amlogic/software/boot/g12/dram-capacity-accounting.md sec.2).
 *
 * aml_ddr.fw states the same split in ONE code byte: the g12a blob's size-
 * detect csel arm is `mov x0, #0xf58` (3928) and the g12b blob's is
 * `mov x0, #0xf50` (3920) -- file offset 0x60f9, 0xeb vs 0xea
 * (g12b-transfer-assessment.md sec.1.3).  That blob byte is what prints
 * "DDR size:"; this constant is what OUR walks must not run past.
 *
 * Keyed on CONFIG_MESON_G12B_POWER_INIT, which is this file's existing
 * G12A/G12B discriminator (see the meson_power_init() guard above) and the
 * only symbol either -libre defconfig uses to say "this is Alta".  A future
 * G12B target that wants the SM1 orchestrator without spl-g12b.c's power
 * sequence would need a symbol of its own; none exists today.
 */
#if IS_ENABLED(CONFIG_MESON_G12B_POWER_INIT)
#define G12A_DRAM_APERTURE_BASE		0xF5000000UL	/* G12B: A73_DBG */
#else
#define G12A_DRAM_APERTURE_BASE		0xF5800000UL	/* G12A/SM1: A53_DBG */
#endif

#define G12A_DRAM_TEST_STRIDE_D		0x00100000UL
/* 0 .. aperture-1 MiB: 3928 points on SM1, 3920 on G12B. */
#define G12A_DRAM_TEST_N_D		(G12A_DRAM_APERTURE_BASE >> 20)
#define G12A_DRAM_TEST_E_BASE		0xF5000000UL
#define G12A_DRAM_TEST_STRIDE_E		0x00001000UL
/*
 * The tail between U-Boot proper's clamp and the real aperture: 2048 points
 * (8 MiB) on SM1, and EMPTY on G12B, where the clamp IS the aperture base --
 * there the whole window is A73_DBG, so walking it would write CPU debug
 * registers and feed the SPL's accept/refuse gate with their readback.
 */
#define G12A_DRAM_TEST_N_E \
	((G12A_DRAM_APERTURE_BASE - G12A_DRAM_TEST_E_BASE) / \
	 G12A_DRAM_TEST_STRIDE_E)

/**
 * g12a_publish_mem_size - republish the POPULATED size into AO_SEC_GP_CFG0
 * @ctl: aml_ddr control block base (ctl[0xa0] = ddr_set pointer)
 *
 * The product BL2 does exactly this immediately after its own slot5 call,
 * at bl2_new.bin fffa7a18-fffa7a50 (blr slot5 is at fffa797c):
 *
 *	mov  x1, #0x240 ; movk x1, #0xff80, lsl #16
 *	ldr  w2, [x1]			; AO_SEC_GP_CFG0
 *	ldrb w1, [x0, #14]		; ds.dram_type
 *	and  w2, w2, #0xffff		; keep board ID + boot device
 *	orr  w2, w2, w1, lsl #16
 *	ldrh w1, [x0, #40] ; ldrh w0, [x0, #42]	; cs0 MB, cs1 MB
 *	add  w1, w1, w0 ; lsr w1, w1, #3
 *	orr  w1, w2, w1, lsl #19
 *	str  w1, [AO_SEC_GP_CFG0]
 *
 * slot5 has already written its OWN figure into [31:16] (aml_ddr.fw
 * fffba120-fffba13c) — and that figure is aperture-CLAMPED: for a part
 * whose populated total does not fit a 32-bit byte address it substitutes
 * the literal 0xF58 = 3928 MB = the SM1 device-aperture base (fffba0f0-
 * fffba0fc), and its "orr w20, w0, w20, lsl #16" also zeroes [18:16].
 * The vendor overwrites both fields with the raw cs0+cs1 total and the
 * DRAM type; that write is the last word on a product board.
 *
 * Our chain called slot5 and never made this correction, so it published
 * 3928 where the vendor publishes 4096: measured 0x0F5800F5 here versus
 * 0x100201B3 on the production AML-S905D3-CC (192.168.0.140).  Nothing
 * downstream diverged — U-Boot's get_effective_memsize() applies its own
 * min(..., 0xF5000000) clamp, which both values exceed — but the register's
 * contract is POPULATED size (the consumer clamps, the producer does not;
 * dram-gx.c:1168 publishes meson_dram_detect_size() for the same reason),
 * so publishing a clamped value destroys inventory no other register
 * carries and drops the DRAM type with it.
 *
 * Crumb: "ms <value>".
 */
static void g12a_publish_mem_size(ulong ctl)
{
	ulong ds = (ulong)readq((void *)(ctl + G12A_CTL_OFF_DDR_SET));
	u32 v, mb;

	if (!ds)
		return;

	v = readl((void *)G12A_AO_SEC_GP_CFG0) & 0xFFFFu;
	v |= ((u32)readb((void *)(ds + G12A_DS_OFF_DRAM_TYPE)) & 0x7u) << 16;

	mb = (u32)readw((void *)(ds + G12A_DS_OFF_CS0_MB)) +
	     (u32)readw((void *)(ds + G12A_DS_OFF_CS1_MB));
	v |= (mb >> 3) << 19;

	writel(v, (void *)G12A_AO_SEC_GP_CFG0);

	g12a_uart_puts("ms ");
	g12a_uart_hex32(v);
	g12a_uart_puts("\n");
}

static void g12a_dram_report(const char *tag, u32 bad, u32 n)
{
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex32(bad);
	g12a_uart_putc('/');
	g12a_uart_hex32(n);
	g12a_uart_puts("\n");
}

/*
 * ---------------------------------------------------------------------
 * BUS TEST (BTD / BTA) — our own DataBus and AddrBus verification
 *
 * WHY THIS EXISTS.  Skipping slot5 (G12A_KEEP_SLOT5=0, on topic/g12a-
 * fppll-slot5) removes the `cs0/cs1 DataBus test pass` and `cs0/cs1
 * AddrBus test pass` lines, because those are slot5's OWN output — it
 * calls both tests from its tail at aml_ddr.fw fffba0e4 / fffba0f0 and
 * returns their summed verdict.  Boot bootB-noslot5 (audit
 * amlogic 3efc4a656) therefore carries no evidence the bus is good.
 * slot5 discovers geometry AND verifies; this block replaces the
 * verification half so the discovery half can be retired on its own.
 *
 * WHAT slot5 ACTUALLY DOES, re-derived from the blob this session:
 * aml_ddr.fw md5 eddca53f01322e2a7f4a8aca1b89ea3a, 48080 B, 0x60 header
 * stripped, payload based at 0xFFFB4000 (the convention of
 * amlogic/software/boot/design/g12-slot5-lowmem-writes.md, re-confirmed
 * here: the documented slot5 prologue `stp x29,x30,[sp,#-112]!` decodes
 * at exactly file +0x5e08 under it and nowhere else).
 *
 *   DataBus core, fffb9970(addr) — ONE address per chip-select, the rank
 *     base.  34 write-then-immediately-read-back pairs: 0xAAAAAAAA,
 *     0x55555555, then 1<<0 .. 1<<31 (fffb9980, fffb99c8, and the
 *     `mov w22,#0x20` loop at fffb9a14).  On a mismatch fffb97d4 prints
 *     "fail D" and enumerates the differing DQ bit indices.
 *   DataBus wrapper, fffb9bb0(ctl) — base = ds[0x20]; runs cs0 at base,
 *     then, only if ds[0x2a] != 0, cs1 at base + (ds[0x28] << 20).
 *     Prints "cs0/cs1 DataBus test " + "pass"/"failed".
 *   AddrBus core, fffb9a6c(base, size, hdt) — word mask = size/4 - 1.
 *     Ph1: write 0xAAAAAAAA at EVERY power-of-two word offset within the
 *          mask (fffb9a90).
 *     Ph2: write 0x55555555 at offset 0, then re-read each power-of-two
 *          offset expecting 0xAAAAAAAA — an address line that fails to
 *          decode aliases onto 0.  Crumb "ADDR" (fffb9aa8).
 *     Ph3: write 0xAAAAAAAA at offset 0; for each power-of-two offset w,
 *          write 0x55555555 there, re-read offset 0 ("ADDR2" if it
 *          moved), then re-read every OTHER power-of-two offset
 *          expecting 0xAAAAAAAA ("ADDR3" = two address lines shorted),
 *          then restore w (fffb9af8..fffb9ba0).
 *     Returns the failing ADDRESS, which fffb9844 maps to a named DDR
 *     address pin via the DMC address-map registers and a blob rodata
 *     name table at fffbf438.
 *   AddrBus wrapper, fffb9c80(ctl) — clamps ds[0x28] to 0xF58 if it
 *     exceeds 0xFFF (and WRITES that back), then runs cs0 over
 *     ds[0x28]<<20 and, if ds[0x2a] != 0, cs1 at base + cs0 size.
 *
 * So slot5's bus verification is a single-address data-bus walk plus a
 * power-of-two address-decode walk, per rank.  It touches on the order
 * of 900 addresses per rank.  It is a BUS test, not a memory test — that
 * is exactly why DTA..DTE below are complementary and not redundant.
 *
 * WHAT THIS BLOCK COVERS, AND WHAT IT DOES NOT.  Measured claim: none —
 * this has never executed.  By construction:
 *   COVERED  — the DataBus walk, pattern-for-pattern and in slot5's
 *              order (0xAAAAAAAA, 0x55555555, 32 walking ones), at the
 *              rank base, per chip-select, reported per chip-select.
 *   COVERED  — all three AddrBus phases, same patterns, same
 *              power-of-two offset set, same pairwise-short inner loop,
 *              per chip-select, reported per chip-select.
 *   NOT      — the DDR address-PIN name mapping.  slot5 prints which pin
 *              failed; we print the failing ADDRESS and the got^exp bit
 *              vector, and the reader maps it by hand.
 *   NOT      — DISCOVERY.  slot5 derives the per-rank density from the
 *              part; ours is the compile-time G12A_BUS_TEST_CS*_MB pair.
 *              On a board populated differently from those constants
 *              this tests the wrong span while slot5 would self-adapt.
 *              The "BT cfg" crumb prints ours AND ds[0x28]/ds[0x2a] side
 *              by side so a disagreement is visible rather than hidden.
 *   NOT      — anything about margin, retention or thermal stability,
 *              for the same reason slot5's pass line does not: a bus
 *              test is a boot-time fingerprint at one operating point.
 *
 * Crumbs, deliberately NOT slot5's strings, so a log is never ambiguous
 * about which arm ran:
 *   BT cfg <base> <our cs0 MB> <our cs1 MB> <ds[0x28]> <ds[0x2a]>
 *   BTD0/BTD1 <bad>/<34>     DataBus, per chip-select
 *   BTA0/BTA1 <bad>/<points> AddrBus, per chip-select
 *   bd  <addr> <exp> <got^exp>        a DataBus miscompare
 *   ba1/ba2/ba3 <addr> <exp> <got^exp> an AddrBus miscompare, by phase
 *
 * This writes DRAM and nothing else.  It touches no delay register.
 */
#ifndef G12A_BUS_TEST
#define G12A_BUS_TEST			0
#endif

#if G12A_BUS_TEST
/*
 * Populated density per chip-select, MB.  These are OURS, not
 * discovered: 2048 + 2048 is what G12A_DS_SIZE_EXPLICIT states for this
 * part and what slot5's own walk finds (it then publishes the
 * aperture-CLAMPED 3928 instead — see g12a_publish_mem_size()).  The
 * clamp is a publishing artefact; the ranks really are 2 GiB each, and
 * the AddrBus walk must span the rank, not the aperture.
 */
#ifndef G12A_BUS_TEST_CS0_MB
#define G12A_BUS_TEST_CS0_MB		2048U
#endif
#ifndef G12A_BUS_TEST_CS1_MB
#define G12A_BUS_TEST_CS1_MB		2048U
#endif

#define G12A_BUS_PAT_A			0xAAAAAAAAu
#define G12A_BUS_PAT_5			0x55555555u
#define G12A_BUS_DATA_N			34u	/* AA, 55, 32 walking ones */

/*
 * SELF-TEST — the negative control for this instrument.
 *
 * A zero from a comparator that has never been shown to FIRE is not
 * evidence of a good bus; it is equally consistent with a comparator that
 * cannot fail.  This project has already paid for that confusion once: the
 * G1 RxEnDly sweep reported 4080 points / 0 non-passing, and the null was
 * a consequence of writing dead registers, not of a clean margin
 * (software/convention/instrument-liveness.md).
 *
 * Building with -DG12A_BUS_TEST_SELFTEST=1 XORs the EXPECTED side of every
 * comparison — and ONLY the expected side — with G12A_BUS_SELFTEST_XOR, so
 * every read-back must miscompare.  What is WRITTEN to DRAM is unchanged:
 * the patterns, the addresses and the ordering are bit-for-bit the ones the
 * default arm uses, so the self-test arm stresses the hardware identically
 * and can never damage it or leave DRAM in a state the default arm would
 * not.  Only the arithmetic that decides pass/fail moves.
 *
 * A boot of that arm must therefore report BTD 0x22/0x22 and BTA
 * 0x366/0x366 — every point bad.  Anything less, and in particular a zero,
 * says the corruption never reached the comparator and the calibration
 * zero means nothing.
 *
 * Default 0 makes the XOR the identity, which folds away at compile time,
 * so the default build stays byte-identical to one without this knob.
 */
#ifndef G12A_BUS_TEST_SELFTEST
#define G12A_BUS_TEST_SELFTEST		0
#endif
#if G12A_BUS_TEST_SELFTEST
#define G12A_BUS_SELFTEST_XOR		0xFFFFFFFFu
#else
#define G12A_BUS_SELFTEST_XOR		0u
#endif
/* Expected value of a read-back.  Identity unless the self-test is on. */
#define G12A_BUS_EXP(v)			((v) ^ G12A_BUS_SELFTEST_XOR)

static void g12a_bus_fail(const char *tag, ulong a, u32 exp, u32 got)
{
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex32((u32)a);
	g12a_uart_putc(' ');
	g12a_uart_hex32(exp);
	g12a_uart_putc(' ');
	g12a_uart_hex32(got ^ exp);	/* the failing DQ lanes, directly */
	g12a_uart_puts("\n");
}

/*
 * DataBus — 34 patterns at ONE address, each written and read straight
 * back, exactly as slot5's fffb9970 does.  Walking ones is what makes
 * this a DQ-lane test rather than a memory test: pattern i drives lane i
 * high with every other lane low, so a stuck, open or shorted lane
 * cannot be masked by a neighbour.  Read-back is immediate and
 * deliberately so -- this asks whether the bus carried the word, not
 * whether the cell retained it.
 */
static u32 g12a_bus_data(ulong a)
{
	u32 i, bad = 0;

	for (i = 0; i < G12A_BUS_DATA_N; i++) {
		u32 pat, exp, got;

		if (i == 0)
			pat = G12A_BUS_PAT_A;
		else if (i == 1)
			pat = G12A_BUS_PAT_5;
		else
			pat = 1u << (i - 2);

		/* @pat is written; @exp is compared.  They differ only in
		 * the self-test arm -- see G12A_BUS_TEST_SELFTEST. */
		exp = G12A_BUS_EXP(pat);

		writel(pat, (void *)a);
		got = readl((void *)a);
		if (got != exp) {
			if (bad < 8)
				g12a_bus_fail("bd", a, exp, got);
			bad++;
		}
	}
	g12a_wdt_pet_safe();
	return bad;
}

/*
 * AddrBus — the three-phase power-of-two address-decode walk of slot5's
 * fffb9a6c.  @size is the rank's span in BYTES; the walk is over word
 * offsets, so the usable mask is size/4 - 1.
 *
 * @pts returns the number of read-back COMPARISONS made, which is the
 * denominator the BTA crumb reports.  Counting rule: one per Ph2 read,
 * plus one per Ph3 offset-0 read, plus one per Ph3 inner read.  Ph1 is
 * write-only and contributes none.
 */
static u32 g12a_bus_addr(ulong base, ulong size, u32 *pts)
{
	ulong mask = (size >> 2) - 1UL;
	ulong w, v;
	u32 bad = 0, n = 0, got;
	/*
	 * What a read-back must equal.  Every WRITE below still uses
	 * G12A_BUS_PAT_A / G12A_BUS_PAT_5 unchanged; only this comparison
	 * value moves, and only in the self-test arm.
	 */
	const u32 exp_a = G12A_BUS_EXP(G12A_BUS_PAT_A);

	/* Ph1 — seed every power-of-two word offset. */
	for (w = 1; (w & mask) != 0UL; w <<= 1)
		writel(G12A_BUS_PAT_A, (void *)(base + (w << 2)));
	g12a_wdt_pet_safe();

	/*
	 * Ph2 — anti-pattern at offset 0.  Any address line that does not
	 * decode makes its power-of-two offset alias onto offset 0, and the
	 * read below then returns 0x55555555 instead of 0xAAAAAAAA.
	 */
	writel(G12A_BUS_PAT_5, (void *)base);
	for (w = 1; (w & mask) != 0UL; w <<= 1) {
		got = readl((void *)(base + (w << 2)));
		n++;
		if (got != exp_a) {
			if (bad < 8)
				g12a_bus_fail("ba1", base + (w << 2),
					      exp_a, got);
			bad++;
		}
	}
	g12a_wdt_pet_safe();

	/*
	 * Ph3 — walk the anti-pattern instead.  Writing offset w and then
	 * finding offset 0 disturbed means w aliases onto 0 (ba2); finding
	 * some OTHER offset v disturbed means the two address lines are
	 * shorted together (ba3).  Restore w before advancing so each step
	 * starts from the Ph1 seed.
	 */
	writel(G12A_BUS_PAT_A, (void *)base);
	for (w = 1; (w & mask) != 0UL; w <<= 1) {
		ulong wa = base + (w << 2);

		writel(G12A_BUS_PAT_5, (void *)wa);

		got = readl((void *)base);
		n++;
		if (got != exp_a) {
			if (bad < 8)
				g12a_bus_fail("ba2", wa, exp_a, got);
			bad++;
		}

		for (v = 1; (v & mask) != 0UL; v <<= 1) {
			if (v == w)
				continue;
			got = readl((void *)(base + (v << 2)));
			n++;
			if (got != exp_a) {
				if (bad < 8)
					g12a_bus_fail("ba3",
						      base + (v << 2),
						      exp_a, got);
				bad++;
			}
		}

		writel(G12A_BUS_PAT_A, (void *)wa);
		g12a_wdt_pet_safe();
	}

	*pts = n;
	return bad;
}

/*
 * Per-chip-select driver.  slot5 reports cs0 and cs1 separately and a
 * single-CS test is not equivalent to it, so both ranks are walked and
 * both are reported under their own tag.
 */
static u32 g12a_bus_test(void)
{
	ulong ds = (ulong)readq((void *)(G12A_DDR_CTL_BASE +
					 G12A_CTL_OFF_DDR_SET));
	ulong base = 0;
	u32 bad, tot = 0, pts = 0;
	u32 cs0_mb = G12A_BUS_TEST_CS0_MB, cs1_mb = G12A_BUS_TEST_CS1_MB;

	if (ds)
		base = (ulong)readl((void *)(ds + G12A_DS_OFF_BASE_ADDR));

	/*
	 * Per-rank span from the ddr_set_t the chain actually ran on, when it
	 * states one: N1294's rank path pokes ds+0x2a = 0 on a single-rank
	 * part (g12a_rank_override), slot5 (blob or G12A_OPEN_SLOT5) resolves
	 * the 0xFFFF sentinels in place, and a replayed snapshot carries its
	 * capture's own sizes.  The compile-time pair stays the fallback for
	 * a ds still at the sentinel.  cs1 == 0 skips the rank-1 tests, so a
	 * single-rank part is never walked above its populated span.
	 */
	if (ds) {
		u16 v0 = readw((void *)(ds + G12A_DS_OFF_CS0_MB));
		u16 v1 = readw((void *)(ds + G12A_DS_OFF_CS1_MB));

		if (v0 != 0xFFFFu && v0 != 0u)
			cs0_mb = v0;
		if (v1 != 0xFFFFu)
			cs1_mb = v1;
	}

	/*
	 * Ours vs slot5's, side by side.  The last two words are what slot5
	 * discovered; with slot5 skipped they stand at the 0xFFFF sentinel
	 * (or at whatever G12A_DS_SIZE_EXPLICIT wrote), and 0xEEEE means
	 * there was no ddr_set to read at all.
	 */
	g12a_uart_puts("BT cfg ");
	g12a_uart_hex32((u32)base);
	g12a_uart_putc(' ');
	g12a_uart_hex16((u16)cs0_mb);
	g12a_uart_putc(' ');
	g12a_uart_hex16((u16)cs1_mb);
	g12a_uart_putc(' ');
	g12a_uart_hex16(ds ? readw((void *)(ds + G12A_DS_OFF_CS0_MB))
			   : (u16)0xEEEE);
	g12a_uart_putc(' ');
	g12a_uart_hex16(ds ? readw((void *)(ds + G12A_DS_OFF_CS1_MB))
			   : (u16)0xEEEE);
	g12a_uart_puts("\n");

	bad = g12a_bus_data(base);
	g12a_dram_report("BTD0", bad, G12A_BUS_DATA_N);
	tot += bad;

	bad = g12a_bus_addr(base, (ulong)cs0_mb << 20, &pts);
	g12a_dram_report("BTA0", bad, pts);
	tot += bad;

	if (cs1_mb) {
		ulong b1 = base + ((ulong)cs0_mb << 20);

		bad = g12a_bus_data(b1);
		g12a_dram_report("BTD1", bad, G12A_BUS_DATA_N);
		tot += bad;

		bad = g12a_bus_addr(b1, (ulong)cs1_mb << 20, &pts);
		g12a_dram_report("BTA1", bad, pts);
		tot += bad;
	}

	return tot;
}
#endif	/* G12A_BUS_TEST */

static u32 g12a_dram_test(void)
{
	u32 bad, tot = 0;
	u32 dmb, n_a, n_b, n_d;
	ulong span, end;

	/*
	 * DMC state first, so a failing test can be read against the
	 * controller's own registers rather than guessed at.
	 * dm <0xFF638000 T12 enable> <0xFF639368 geometry> <0xFF800240 density>
	 */
	g12a_uart_puts("dm ");
	g12a_uart_hex32(readl((void *)0xFF638000UL));
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)0xFF639368UL));
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)0xFF800240UL));
	g12a_uart_puts("\n");

#if G12A_BUS_TEST
	/*
	 * Bus before span.  A bad DQ lane or a mis-decoded address line
	 * makes every DTA..DTE result uninterpretable, so ask that question
	 * first -- which is also where slot5 asks it, from its own tail
	 * before g12a_dram_test() is ever reached.  Counts fold into `tot`,
	 * so the SPL's accept/refuse gate sees a bus failure.
	 */
	tot += g12a_bus_test();
#endif

	g12a_uart_puts("DT begin\n");

	/*
	 * Bound every walk by the DETECTED size: slot5/aml_ddr's published
	 * figure in AO_SEC_GP_CFG0[31:19], which counts 8 MB units (<< 23 =
	 * bytes), clamped to the 3928 MB = 0xF5800000 aperture of
	 * dram-capacity-accounting.md.  Boot n1294r8a: [31:19] = 0x100 ->
	 * span = 2 GiB, and the fixed N_A/N_D walks were the defect -- they
	 * poked above populated DRAM (DTA failed only at 0x80000000+).
	 */
	dmb = readl((void *)G12A_AO_SEC_GP_CFG0) >> 19;	/* [31:19], 8 MB units */
	span = (ulong)dmb << 23;
	if (span > 0xF5800000UL)
		span = 0xF5800000UL;
	g12a_uart_puts("DTS ");
	g12a_uart_hex32((u32)span);
	g12a_uart_puts("\n");
	if (span < 0x10000000UL) {
		/*
		 * Below 256 MiB the DTC window (0x10000000 + 1 MiB) and the
		 * SPL entry at 0x08000000 cannot both be tested, and every
		 * walk count would round to 0 -- tot == 0 on UNTESTED DRAM
		 * is exactly what a zero/garbage density field must NOT be
		 * allowed to produce.  Fail loudly, as before the span
		 * bound existed.
		 */
		g12a_uart_puts("DTS bad\n");
		return tot + 1;
	}
	n_a = (u32)(span / G12A_DRAM_TEST_STRIDE_A);
	if (n_a > G12A_DRAM_TEST_N_A)
		n_a = G12A_DRAM_TEST_N_A;
	n_b = (u32)(span / G12A_DRAM_TEST_STRIDE_B);
	if (n_b > G12A_DRAM_TEST_N_B)
		n_b = G12A_DRAM_TEST_N_B;
	n_d = (u32)(span / G12A_DRAM_TEST_STRIDE_D);
	if (n_d > G12A_DRAM_TEST_N_D)
		n_d = G12A_DRAM_TEST_N_D;

	bad = g12a_dram_walk("daf", 0, G12A_DRAM_TEST_STRIDE_A, n_a);
	g12a_dram_report("DTA", bad, n_a);
	tot += bad;

	bad = g12a_dram_walk("dbf", 0, G12A_DRAM_TEST_STRIDE_B, n_b);
	g12a_dram_report("DTB", bad, n_b);
	tot += bad;

	end = G12A_DRAM_TEST_C_BASE + (ulong)G12A_DRAM_TEST_C_WORDS * 4u;
	if (end <= span) {
		bad = g12a_dram_lcg();
		g12a_dram_report("DTC", bad, G12A_DRAM_TEST_C_WORDS);
		tot += bad;
	} else {
		g12a_uart_puts("DTC skip\n");
	}

	/* Bounded by the detected span (DTS), 1 MiB stride -- capped at N_D. */
	bad = g12a_dram_walk("ddf", 0, G12A_DRAM_TEST_STRIDE_D, n_d);
	g12a_dram_report("DTD", bad, n_d);
	tot += bad;

#ifdef CONFIG_MESON_G12A_DRAM_OPEN
	/*
	 * DTD again, immediately, in the SAME boot with the SAME PHY and DMC
	 * state.  Across boots the failing address sets are DISJOINT
	 * (t7-dtd-state-diff.md sec.3), which already rules out a fixed
	 * address-decode defect; this separates the two remaining readings.
	 * A second pass failing at DIFFERENT addresses means the fault is
	 * random per access.  A second pass failing at the SAME addresses
	 * means it is bound to this boot's trained state.
	 * The count is not added to `tot` -- this is a diagnostic, and the
	 * SPL's accept/refuse gate must judge one walk, not two.
	 */
	bad = g12a_dram_walk("de2", 0, G12A_DRAM_TEST_STRIDE_D, n_d);
	g12a_dram_report("DTD2", bad, n_d);
#endif

#if G12A_DRAM_TEST_N_E
	/* The 8 MiB tail U-Boot proper's 0xF5000000 clamp discards. */
	end = G12A_DRAM_TEST_E_BASE +
	      (ulong)G12A_DRAM_TEST_N_E * G12A_DRAM_TEST_STRIDE_E;
	if (end <= span) {
		bad = g12a_dram_walk("def", G12A_DRAM_TEST_E_BASE,
				    G12A_DRAM_TEST_STRIDE_E,
				    G12A_DRAM_TEST_N_E);
		g12a_dram_report("DTE", bad, G12A_DRAM_TEST_N_E);
		tot += bad;
	} else {
		g12a_uart_puts("DTE skip\n");
	}
#endif

	/*
	 * The coarse walk's third point IS the SPL load address
	 * (CONFIG_MESON_G12A_SPL_ENTRY = 0x08000000), and the TPL handoff
	 * decides "an SPL is present" by reading that first word.  Leaving
	 * our test pattern there makes a residual boot that delivered NO
	 * SPL jump into garbage (measured: boot stage2-1 and stage2-2 both
	 * printed JMP).  Clear the window so the handoff sees the truth.
	 */
	g12a_zero32(CONFIG_MESON_G12A_SPL_ENTRY, 16);

	g12a_uart_puts("DT end\n");
	return tot;
}

/*
 * G12A_DIAG_NO_DRAM_TEST -- DIAGNOSTIC-ONLY.  Default 0; no defconfig, no
 * board config and no product build sets it.  Passed through KCFLAGS by a
 * diagnostic build only, the same way G12A_OPEN_DEVINIT_WB is.
 *
 * Compiles out the boot-time DRAM self-tests DTA..DTE (g12a_dram_test(), and
 * with it g12a_dram_walk(), g12a_dram_lcg(), g12a_dram_report() and their
 * strings) so a diagnostic arm that overflows .sram can link.  It exists for
 * the L14 HOLD arm (G12A_OPEN_DEVINIT_WB=2), which is 160 B over
 * CONFIG_TPL_MAX_SIZE.
 *
 * Why the tests are independent of what such an arm measures: they run after
 * "S2 end", i.e. after training, the DevInit replay and its DMEM writes, the
 * stage-2 DMC bring-up and the size publish have all completed; they only
 * write and read DRAM.  Nothing before them reads their result.
 *
 * With the gate on, the stub prints "DT off", clears the SPL entry window as
 * the real test does on its way out (the handoff reads that word), and returns
 * 0 -- so the SPL accept/refuse gate sees NO DRAM verdict at all.  A build with
 * this set is not a DRAM-acceptance build and must never be shipped.
 */
#ifndef G12A_DIAG_NO_DRAM_TEST
#define G12A_DIAG_NO_DRAM_TEST	0
#endif

#if G12A_DIAG_NO_DRAM_TEST
static u32 g12a_dram_test_off(void)
{
	(void)g12a_dram_test;
	g12a_zero32(CONFIG_MESON_G12A_SPL_ENTRY, 16);
	g12a_uart_puts("DT off\n");
	return 0;
}
#define g12a_dram_test	g12a_dram_test_off
#endif

/*
 * ---------------------------------------------------------------------
 * MID1 scratch -- ONE buffer, shared by both builders
 *
 * Two call sites build a MID1 image with g12a_mid1_compute():
 * g12a_p2_mid1_compare() (the MARGIN instrument, compare-only) and
 * g12a_open_s2s5_timing_mid1() (the OPEN path's S2+S5 / T1+T2 step).  They
 * used to own 416 B each.  aml-s905d3-cc_libre_open_margin is the only
 * target that compiles both, and those 832 B are what pushed its TPL .bss
 * 328 B past `.sdram' -- a region whose 4 KiB cannot grow, being pinned
 * below by TPL_TEXT_BASE + the BootROM's 0xf000 payload cap and above by
 * G12A_DDR_CTL_BASE (0xFFFB1000), the vendor ddr_fw control block that
 * aml_ddr.fw reads at a fixed address.  De-duplicating the buffer is not a
 * relaxed bound: `.sdram' stays 0x1000.  Derivation:
 * amlogic/software/boot/g12/audit/2026-08-31-sm1-tpl-sdram-bss-overflow.md
 *
 * Sharing is sound because NEITHER user keeps the buffer live across its
 * own return.  Each fills it with g12a_mid1_compute() and has consumed the
 * last entry it reads before the function exits -- no other function in
 * this file touches either object -- and neither user is reachable from
 * the other: g12a_p2_mid1_compare() has exactly one caller,
 * g12a_p2_open_train(), which runs from g12a_dram_margin() after bring-up,
 * while g12a_open_s2s5_timing_mid1() is called only from the open S2/S5
 * and T1/T2 steps.  The TPL is single-threaded at EL3 with interrupts
 * masked, so there is no concurrent reader either.
 *
 * That leaves exactly one way a future edit can break it: nesting the two
 * users, directly or through a new caller.  g12a_mid1_claim() makes that a
 * printed refusal instead of silent corruption, so the invariant does not
 * rest on this comment.
 *
 * The outer guard is the union of the two users' own guards, so a target
 * that compiles neither (residual, residual_opensc) gets no object and no
 * unused-function warning.  G12A_MID1_SHARED is the INTERSECTION: the
 * nesting hazard exists only where both users are compiled, which today is
 * open_margin alone, so that is the only image that carries the runtime
 * check.  A target compiling exactly one user had one 416 B buffer before
 * and has one now, and its TPL stays byte-for-byte what it was.
 */
#if defined(CONFIG_MESON_G12A_DRAM_OPEN) || \
    defined(CONFIG_MESON_G12A_DRAM_MARGIN) || \
    defined(CONFIG_MESON_G12A_DRAM_WIDTH)

#if defined(CONFIG_MESON_G12A_DRAM_OPEN) && \
    (defined(CONFIG_MESON_G12A_DRAM_MARGIN) || \
     defined(CONFIG_MESON_G12A_DRAM_WIDTH))
#define G12A_MID1_SHARED	1
#else
#define G12A_MID1_SHARED	0
#endif

static struct g12a_mid1_reg g12a_mid1_buf[G12A_MID1_N];
#if G12A_MID1_SHARED
/* dram_init() runs inside board_init_f(), before the TPL clears .bss
 * (arch/arm/lib/crt0_64.S clear_loop), so this pre-.bss state must live in .data. */
static u8 g12a_mid1_live __section(".data");
#endif

/*
 * The compile-time half: both users hand this buffer to
 * g12a_mid1_compute(), whose contract is "@out: G12A_MID1_N entries".  A
 * future edit that changes the count on one side only fails the build
 * instead of overrunning the shared object.
 */
_Static_assert(sizeof(g12a_mid1_buf) / sizeof(g12a_mid1_buf[0]) ==
	       G12A_MID1_N,
	       "MID1 scratch must hold exactly G12A_MID1_N entries");

/**
 * g12a_mid1_claim() - take the shared MID1 scratch
 * @who: short tag printed if the claim is refused
 *
 * Return: the buffer, or NULL if it is already live.  NULL can only happen
 * where both users are compiled and a later edit has nested them; refusing
 * is the same shape as the g12a_timing_engine() refusal both callers
 * already handle.  Where only one user is compiled the check is compiled
 * out and this is just the buffer's address.
 */
static struct g12a_mid1_reg *g12a_mid1_claim(const char *who)
{
#if G12A_MID1_SHARED
	if (g12a_mid1_live) {
		g12a_uart_puts("MID1 busy ");
		g12a_uart_puts(who);
		g12a_uart_puts("\n");
		return NULL;
	}
	g12a_mid1_live = 1;
#else
	(void)who;
#endif
	return g12a_mid1_buf;
}

static void g12a_mid1_release(void)
{
#if G12A_MID1_SHARED
	g12a_mid1_live = 0;
#endif
}

#endif /* DRAM_OPEN || DRAM_MARGIN || DRAM_WIDTH */

/*
 * ---------------------------------------------------------------------
 * DRAM MARGIN measurement (CONFIG_MESON_G12A_DRAM_MARGIN)
 *
 * DTA/DTB/DTC above are decode and integrity tests: they answer "is the
 * address decode right and is any bit stuck", and they were already at
 * zero failures.  Neither they nor anything in U-Boot proper can answer
 * "how much timing headroom is there".  Two facts fix the layer this has
 * to run at:
 *
 *  - MEASURED from an interactive U-Boot proper on this board (boot
 *    margin-1): every 16-bit read in the DDR PHY APB window 0xFE000000
 *    returns 0000 — including MemResetL (0xFE0400C0), which the TPL
 *    measures as 0002 — and DMC 0xFF6393xx (the BIST engine) reads 0
 *    as well, while DMC 0xFF638000 / 0xFF800240 read their true values.
 *    So the PHY and the BIST block are firewalled from EL2 after BL31.
 *  - The TPL runs from SRAM at EL3.  Code and stack survive DRAM that
 *    has been deliberately pushed out of its trained window, which is
 *    exactly what a shmoo does.
 *
 * Three measurements, in increasing risk order so a hang still leaves
 * the earlier ones on the wire:
 *
 *   MGC  the trained state itself — per-lane VrefDAC, TxDqDly, RxPBDly,
 *        RxEnDly, RxClkDly, DFIMRL, ANIB ATxDly.  Costs nothing and is
 *        directly comparable to the product BL2's captured ddr_set_t
 *        (amlogic/software/boot/g12/ds-snapshot-trained.md).
 *   MGS  shmoo — step VrefDAC and TxDqDly/RxClkDly away from the trained
 *        centre until a data test fails.  This measures headroom rather
 *        than inferring it from a pass.
 *   MGT  stress — high-toggle patterns (max-SSO 0/~0, checkerboard,
 *        walking-one-with-inversion, hash) over a large span.
 *
 * CSR addressing: PA = 0xFE000000 + (csr << 1), 16-bit
 * (g12a-phy-init-sequence.md §; vendor strh at fffb4c08).
 * CSR = (block << 16) | (instance << 12) | (lane << 8) | offset;
 * DBYTE block = 0x1, ANIB block = 0x0.  Offsets from the Synopsys
 * cdefines and cross-checked against the ten delay classes fffb502c
 * sweeps (host-driven-training-feasibility.md §2.1).
 */
#if defined(CONFIG_MESON_G12A_DRAM_MARGIN) || \
    defined(CONFIG_MESON_G12A_DRAM_WIDTH) || \
    defined(CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE)

#if defined(CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE) && \
    !defined(CONFIG_MESON_G12A_DRAM_MARGIN) && \
    !defined(CONFIG_MESON_G12A_DRAM_WIDTH)
/*
 * The centring pass shares the margin helpers it needs (CSR codec, data
 * probe, G2's window search, P6's apply-and-revert); the rest of this
 * block is compiled but never called, and the linker drops it.
 */
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/* P6: keep G2's centres.  On for the product centring pass. */
#ifndef G12A_MRG_P6
#ifdef CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE
#define G12A_MRG_P6		1
#else
#define G12A_MRG_P6		0
#endif
#endif

#define G12A_MRG_PA(csr)	(G12A_DDR_PHY_APB_BASE + ((ulong)(csr) << 1))
#define G12A_MRG_DB(d, l, off)	(0x10000u | ((u32)(d) << 12) | \
				 ((u32)(l) << 8) | (u32)(off))
#define G12A_MRG_ANIB(i)	(((u32)(i) << 12) | 0x80u)

#define G12A_MRG_VREFDAC1	0x30	/* [6:0] */
#define G12A_MRG_VREFDAC0	0x40	/* [6:0]  class 9 */
#define G12A_MRG_RXPBDLY_TG0	0x68	/* class 5, per lane */
#define G12A_MRG_TXDQSDLY_TG0	0xD0	/* class 2, per nibble */
#define G12A_MRG_RXENDLY_TG0	0x80	/* class 6, per nibble */
#define G12A_MRG_RXCLKDLY_TG0	0x8C	/* class 3, per nibble */
#define G12A_MRG_RXCLKCDLY_TG0	0x90	/* class 3 "+4" */
#define G12A_MRG_TXDQDLY_TG0	0xC0	/* class 4, per lane */
#define G12A_MRG_DFIMRL		0x20	/* class 8, per DBYTE */

/*
 * RxEnDly (class 6) is ELEVEN bits, not nine: coarse [10:6] (PUB 11.3, one
 * unit = 1 UI), fine [4:0], and bit [5] RESERVED.  So the decoded range is
 * 32 coarse x 32 fine - 1, and g12a_mrg_enc4() never sets bit 5 -- which is
 * what makes a decoded-unit step across a UI boundary land on 0x340 rather
 * than 0x320 (rxen-dtsm-boot-plan.md section 2.2).
 */
#define G12A_MRG_RXEN_MAX	1023
#define G12A_MRG_RXEN_RSVD5	0x0020u	/* PUB reserved bit; must read 0 */

/*
 * G1's OWN sweep bound, deliberately NOT the shared G12A_MRG_SWEEP_MAX.
 *
 * Boots dtsm-a and dtsm-b swept +-G12A_MRG_SWEEP_MAX (48) about each nibble's
 * trained point -- 96 points per nibble, 768 in total -- and every one of them
 * PASSED, so no edge was recorded on any of the eight nibbles.  That is a lower
 * bound, not a window: the read gate is wider than +-48 decoded units (+-1.5 UI,
 * the fine field being 5 bits) on 8/8 nibbles, reproduced across two boots.
 * Closing the acceptance item needs a wider sweep, and the shared constant is
 * the wrong place to widen it -- it also bounds the Vref, RxClkDly, RxPBDly and
 * the two Tx axes, whose windows are already two-sided and must not move.
 *
 * 255 is chosen by two limits that happen to agree, not by rounding:
 *
 *   1. The PRINT is the tighter one.  g12a_mrg_rxen_step() and _arm() emit the
 *      offset through g12a_uart_hex8(), so an offset past 255 aliases mod 256
 *      and the log stops being readable without analysis -- which is the one
 *      property the MGEs line exists to have.  255 is the largest bound the
 *      existing format represents, and the format is G1's output, not something
 *      to change while widening its range.
 *   2. The REGISTER is the looser one, and it must still be reachable.  The
 *      decoded axis is 0..G12A_MRG_RXEN_MAX; the trained points measured on
 *      those two boots run 399..411, so the smallest headroom is 399 DOWN (d2)
 *      and 612 UP (d0).  255 < 399, so every arm can actually reach its bound
 *      in both directions on all eight nibbles: an 'L' verdict then means THE
 *      BOUND, unambiguously, and never the register end.
 *
 * That sweeps 511 of the 1024 encodable codes, 49.9% of the axis, +-8.0 UI.
 * Stepping stays in DECODED units through dec4()/enc4(), so the walk still goes
 * AROUND reserved bit 5 exactly as before -- widening the bound changes how far
 * G1 steps and nothing about how it steps, what it writes, or what it probes.
 *
 * The cost is bounded by the first-failure break in g12a_mrg_rxen_arm(), which
 * is unchanged: a direction that finds an edge stops there, so the extra points
 * are walked ONLY where the axis is genuinely open -- i.e. only in the case the
 * previous boots actually hit.  Worst case (nothing fails anywhere) is 8 x 2 x
 * 255 = 4080 points against 768, and the MGEs line is a fixed 45 bytes, so the
 * UART alone goes from 3.0 s to 15.9 s at 115200 8N1.
 */
#define G12A_MRG_RXEN_SWEEP_MAX	255

/*
 * DTSM -- the PHY's per-lane data-training state machine, the one instrument
 * in this file that is NOT the CPU probe.  Offsets and bit fields are the ST
 * PHY header's, cited by line: libretech-builder-simple
 * atf/drivers/st/ddr/phy/phyinit/include/ddrphy_csr_all_cdefines.h.
 * All are DBYTE-space offsets; the per-lane replicated ones take (lane << 8).
 */
#define G12A_MRG_TRAININGPARAM	0x02	/* :23   TrainingParam, DBYTE-wide */
#define G12A_MRG_TSMBYTE3	0x13	/* :28   TsmByte3, DBYTE-wide */
#define G12A_MRG_INCDECDTSMEN	0x62	/* :58   TrainingIncDecDtsmEn [8:0] */
#define G12A_MRG_DTSMBLANK	0xB1	/* :89   DtsmBlankingCtrl [9:0] */
#define G12A_MRG_TSM0		0xB2	/* :90   per lane */
#define G12A_MRG_TSM1		0xB3	/* :91   per lane, DtsmErrCnt */
#define G12A_MRG_TSM3		0xB5	/* :93   DBYTE-wide (lane-0 copy) */
#define G12A_MRG_DTSMINHIB	0xB9	/* :97   DbyteAllDtsmCtrl0 */

/* TrainingParam fields.  Vendor value is 0x6209 (init-items-transcription.md
 * section 4.3) = EnDynRateReduction | TrainEnRxEn | RollIntoCoarse | rate 3. */
#define G12A_MRG_TP_TRAINENRXPBD  0x0100u /* :1100 BIT(8)  arm RxPBD training */
#define G12A_MRG_TP_ROLLINTOCOARSE 0x0200u /* :1102 BIT(9) fine carries into
					    *       coarse */
#define G12A_MRG_TP_INCDECRATE	  0xE000u /* :1110 [15:13] step rate */
#define G12A_MRG_TP_INCDECRATE_3  0x6000u /* the vendor's own measured rate */

/* TsmByte3.  ONLY the two comparator bits are ever written here: BIT(2)
 * EnblRxSampFlops, BIT(3) SelRxSampFlops and BIT(4) SelRxByBass are receiver
 * MUX controls with no measured restore path (rxen-dtsm-boot-plan.md 1.3). */
#define G12A_MRG_B3_STATICCMPR	  0x0080u /* :1147 BIT(7) compare vs a constant */
#define G12A_MRG_B3_STATICCMPRVAL 0x0100u /* :1149 BIT(8) that constant */

#define G12A_MRG_TSM0_ENB	  0x0001u /* :1459 BIT(0) DtsmEnb */
#define G12A_MRG_TSM3_CLRERRCNT	  0x0200u /* :1492 BIT(9) DtsmClrErrCnt */
#define G12A_MRG_LANE_MSK	  0x01FFu /* [8:0] -- 8 DQ + DBI */

/*
 * The DTSM arm sweeps ONE lane of ONE DBYTE; every other lane stays trained
 * and inhibited.  DBYTE 0 lane 0 is the same instance g12a_mrg_rb_csr uses,
 * so its trained RxPBDly is already in the MGCr row of every boot.
 */
#define G12A_MRG_DTSM_D		0
#define G12A_MRG_DTSM_L		0

/* Probe window for the shmoo — small enough to run ~100 times. */
/*
 * The probe window.  The default sits in rank 0.  Ranks map linearly on
 * this part (slot 5, CS0 only, tests clean below 0x80000000 and nowhere
 * above), so a KCFLAGS override such as 0xB0000000UL puts every margin
 * axis's probe in rank 1 on a dual-rank unit.  Measurement images only.
 */
#ifndef G12A_MRG_PROBE_BASE
#define G12A_MRG_PROBE_BASE	0x30000000UL
#endif
#define G12A_MRG_PROBE_RANK0	0x30000000UL

/*
 * With P6 the base is a variable, so P6's centring can search on rank 0
 * while every margin axis after it probes G12A_MRG_PROBE_BASE.  Without P6
 * it stays the constant, and those images are unchanged.
 */
#if G12A_MRG_P6
static unsigned long g12a_mrg_pbase = G12A_MRG_PROBE_BASE;
#define G12A_MRG_PBASE		g12a_mrg_pbase
#else
#define G12A_MRG_PBASE		G12A_MRG_PROBE_BASE
#endif
#define G12A_MRG_PROBE_WORDS	0x00020000UL	/* 512 KiB */

/* Stress spans. */
#define G12A_MRG_FULL_BASE	0x01000000UL
#define G12A_MRG_FULL_WORDS	0x3BC00000UL	/* to 0xF0000000, 3824 MiB */
#define G12A_MRG_WIN_BASE	0x40000000UL
#define G12A_MRG_WIN_WORDS	0x02000000UL	/* 128 MiB */
#define G12A_MRG_WIN_REPEAT	2

#define G12A_MRG_NPAT		8
#define G12A_MRG_SWEEP_MAX	48

/*
 * Phase set.  1 = T19 (per-lane eye + the DMC BIST bring-up), 0 = the
 * environmental set (soak + VDDEE) that dram-margin-environmental.md
 * already measured.  They are mutually exclusive because the whole
 * margin run happens inside dram_init(), i.e. BEFORE the SPL image's
 * residual window opens, so a 65-minute phase set and a per-lane pass
 * cannot both fit in one boot.
 */
#ifndef G12A_MRG_T19
#define G12A_MRG_T19		1
#endif

static u16 g12a_mrg_vref0[4][9];
static u16 g12a_mrg_vref1[4][9];
static u16 g12a_mrg_txdq[4][9][2];
static u16 g12a_mrg_rxclk[4][2][2];
static u16 g12a_mrg_rxclkc[4][2][2];
static u16 g12a_mrg_txdqs[4][2][2];
static u16 g12a_mrg_rxpb[4][9];
/*
 * RxEnDly, per DBYTE / nibble / timing group.  Until now this register was
 * read inline for one field of the MGCk row and never saved, because nothing
 * swept it.  Capturing it costs four extra reads and gives G1 the vendor-side
 * reference value it has never had -- the PMU demonstrably writes this CSR
 * (1d:0x75a8 -> DBYTE.RXENDLYTG0..3, delay-write-site-5302.md section 7.3),
 * so the trained raw is a point to compare a measured window's centre against
 * (rxen-dtsm-boot-plan.md section 5.1.1).
 */
static u16 g12a_mrg_rxen[4][2][2];

/*
 * T19 (per-lane eye).  Bad-word counts alone cannot say WHICH DQ bit
 * failed, so a per-lane sweep reported only as pass/fail could not
 * distinguish "lane (d,l) lost its eye" from "the whole bus moved".
 * g12a_mrg_xor is the OR of (got ^ expected) over every failing word of
 * the last probe: when one lane's delay is stepped out of its window it
 * must be exactly that lane's DQ bit that goes bad, which makes the
 * per-lane CSR addressing self-checking rather than assumed.
 */
static u32 g12a_mrg_xor;

/*
 * Probe size selector.  The 512 KiB two-pattern probe costs ~134 ms; a
 * per-lane shmoo is 36 lanes x 2 axes x two directions of it and does
 * not fit in the residual window's budget.  G12A_MRG_FAST_WORDS is the
 * short probe used for the per-lane pass; the SAME aggregate sweeps are
 * run with BOTH probes in the same boot (tags MGS* and MGF*) so the
 * short probe's sensitivity is calibrated against the long one here,
 * not assumed.
 */
#define G12A_MRG_FAST_WORDS	0x00004000UL	/* 64 KiB */
static int g12a_mrg_fast;

static u64 g12a_mrg_ticks(void)
{
	u64 v;

	asm volatile("isb; mrs %0, cntpct_el0" : "=r" (v));
	return v;
}

/*
 * Host CSR access needs the APB muxed to the host, not the micro
 * (fffb502c writes MicroContMuxSel = 0 before touching any delay CSR).
 * The micro is halted by this point; the value is restored on exit.
 */
static u16 g12a_mrg_mux_enter(void)
{
	u16 old = g12a_phy_read16(G12A_DDR_PHY_MICROCONTMUXSEL);

	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
	return old;
}

static void g12a_mrg_mux_exit(u16 old)
{
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, old);
}

/* The delay codec, defined below with its measured derivation. */
static u32 g12a_mrg_dec4(u16 raw);

static void g12a_mrg_row(const char *tag, u32 d, const u16 *v, u32 n)
{
	u32 i;

	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)d);
	for (i = 0; i < n; i++) {
		g12a_uart_putc(' ');
		g12a_uart_hex16(v[i]);
	}
	g12a_uart_puts("\n");
}

/* Read the trained state into the save arrays and print it. */
static void g12a_mrg_dump(void)
{
	u16 buf[10];
	u32 d, l, n, i;

	g12a_uart_puts("MGC begin\n");

	for (d = 0; d < 4; d++) {
		for (l = 0; l < 9; l++) {
			g12a_mrg_vref0[d][l] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_VREFDAC0)));
			g12a_mrg_vref1[d][l] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_VREFDAC1)));
			g12a_mrg_txdq[d][l][0] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_TXDQDLY_TG0)));
			g12a_mrg_txdq[d][l][1] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_TXDQDLY_TG0 + 1)));
		}
		for (l = 0; l < 9; l++)
			g12a_mrg_rxpb[d][l] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_RXPBDLY_TG0)));
		for (n = 0; n < 2; n++) {
			g12a_mrg_txdqs[d][n][0] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_TXDQSDLY_TG0)));
			g12a_mrg_txdqs[d][n][1] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_TXDQSDLY_TG0 + 1)));
			g12a_mrg_rxclk[d][n][0] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKDLY_TG0)));
			g12a_mrg_rxclk[d][n][1] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKDLY_TG0 + 1)));
			g12a_mrg_rxclkc[d][n][0] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKCDLY_TG0)));
			g12a_mrg_rxclkc[d][n][1] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKCDLY_TG0 + 1)));
			g12a_mrg_rxen[d][n][0] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXENDLY_TG0)));
			g12a_mrg_rxen[d][n][1] = g12a_phy_read16(
				G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXENDLY_TG0 + 1)));
		}
	}

	for (d = 0; d < 4; d++)
		g12a_mrg_row("MGCv", d, g12a_mrg_vref0[d], 9);
	for (d = 0; d < 4; d++)
		g12a_mrg_row("MGCV", d, g12a_mrg_vref1[d], 9);
	for (d = 0; d < 4; d++) {
		for (l = 0; l < 9; l++)
			buf[l] = g12a_mrg_txdq[d][l][0];
		g12a_mrg_row("MGCt", d, buf, 9);
	}
	for (d = 0; d < 4; d++) {
		for (l = 0; l < 9; l++)
			buf[l] = g12a_mrg_txdq[d][l][1];
		g12a_mrg_row("MGCT", d, buf, 9);
	}
	for (d = 0; d < 4; d++)
		g12a_mrg_row("MGCr", d, g12a_mrg_rxpb[d], 9);
	for (d = 0; d < 4; d++) {
		buf[0] = g12a_mrg_txdqs[d][0][0];
		buf[1] = g12a_mrg_txdqs[d][1][0];
		buf[2] = g12a_mrg_txdqs[d][0][1];
		buf[3] = g12a_mrg_txdqs[d][1][1];
		g12a_mrg_row("MGCq", d, buf, 4);
	}
	for (d = 0; d < 4; d++) {
		buf[0] = g12a_mrg_rxclk[d][0][0];
		buf[1] = g12a_mrg_rxclk[d][1][0];
		buf[2] = g12a_mrg_rxclk[d][0][1];
		buf[3] = g12a_mrg_rxclk[d][1][1];
		buf[4] = g12a_mrg_rxclkc[d][0][0];
		buf[5] = g12a_mrg_rxclkc[d][1][0];
		buf[6] = g12a_mrg_rxen[d][0][0];
		buf[7] = g12a_mrg_rxen[d][1][0];
		buf[8] = g12a_phy_read16(G12A_MRG_PA(
			G12A_MRG_DB(d, 0, G12A_MRG_DFIMRL)));
		g12a_mrg_row("MGCk", d, buf, 9);
	}
	/*
	 * The full RxEnDly state, raw and decoded, as its own row: MGCk only
	 * ever carried Tg0 of the two nibbles, and G1 compares its measured
	 * window centre against all four instances per DBYTE.  Decoded is
	 * printed beside raw because the two differ by the coarse/fine codec
	 * and a reader must not have to run it by hand (section 2.2).
	 */
	for (d = 0; d < 4; d++) {
		buf[0] = g12a_mrg_rxen[d][0][0];
		buf[1] = g12a_mrg_rxen[d][1][0];
		buf[2] = g12a_mrg_rxen[d][0][1];
		buf[3] = g12a_mrg_rxen[d][1][1];
		for (i = 0; i < 4; i++)
			buf[4 + i] = (u16)g12a_mrg_dec4(buf[i]);
		g12a_mrg_row("MGCe", d, buf, 8);
	}
	for (i = 0; i < 10; i++)
		buf[i] = g12a_phy_read16(G12A_MRG_PA(G12A_MRG_ANIB(i)));
	g12a_mrg_row("MGCa", 0, buf, 10);

	g12a_uart_puts("MGC end\n");
}

/*
 * Pattern generator.  Every pattern is a pure function of the word
 * index so the verify pass needs no stored state, and every pattern
 * except 5 changes most of the bus between consecutive words — that
 * transition density is the point (SSO / crosstalk), and it is what
 * DTA/DTB/DTC do not have.
 */
static u32 g12a_mrg_val(u32 pat, u32 i, ulong a)
{
	u32 h;

	switch (pat) {
	case 0:				/* max SSO: all bits toggle */
		return (i & 1) ? 0xFFFFFFFFu : 0x00000000u;
	case 1:				/* checkerboard, inverted each word */
		return (i & 1) ? 0xAAAAAAAAu : 0x55555555u;
	case 2:				/* half-bus inversion */
		return (i & 1) ? 0x0000FFFFu : 0xFFFF0000u;
	case 3:				/* nibble-pair inversion */
		return (i & 1) ? 0x33CCCC33u : 0xCC3333CCu;
	case 4:				/* walking one, inverted each word */
		h = 1u << (i & 31);
		return (i & 1) ? ~h : h;
	case 5:				/* address-derived (DTA's rule) */
		return (u32)a ^ 0x5A5A5A5Au;
	case 6:				/* single-bit-apart extremes */
		return (i & 1) ? 0x7FFFFFFFu : 0x80000000u;
	default:			/* pseudo-random, high density */
		h = i * 2654435761u;
		h ^= h >> 15;
		h *= 2246822519u;
		h ^= h >> 13;
		return h;
	}
}

static u32 g12a_mrg_run(u32 pat, ulong base, ulong words, int quiet)
{
	ulong i;
	u32 bad = 0;

	for (i = 0; i < words; i++) {
		ulong a = base + i * 4;

		writel(g12a_mrg_val(pat, (u32)i, a), (void *)a);
		if ((i & 0xFFFF) == 0)
			g12a_wdt_pet_safe();
	}
	g12a_wdt_pet_safe();
	for (i = 0; i < words; i++) {
		ulong a = base + i * 4;
		u32 e = g12a_mrg_val(pat, (u32)i, a);
		u32 g = readl((void *)a);

		if (g != e) {
			if (!quiet && bad < 4)
				g12a_dram_fail("mgf", a, e, g);
			g12a_mrg_xor |= g ^ e;
			bad++;
		}
		if ((i & 0xFFFF) == 0)
			g12a_wdt_pet_safe();
	}
	return bad;
}

/* Short, quiet test used inside a sweep step. */
static u32 g12a_mrg_probe(void)
{
	u32 bad;

	g12a_mrg_xor = 0;
	bad = g12a_mrg_run(0, G12A_MRG_PBASE, G12A_MRG_PROBE_WORDS, 1);
	bad += g12a_mrg_run(7, G12A_MRG_PBASE, G12A_MRG_PROBE_WORDS, 1);
	return bad;
}

/* Same two patterns, 64 KiB — the per-lane probe.  See G12A_MRG_FAST_WORDS. */
static u32 g12a_mrg_probe_fast(void)
{
	u32 bad;

	g12a_mrg_xor = 0;
	bad = g12a_mrg_run(0, G12A_MRG_PBASE, G12A_MRG_FAST_WORDS, 1);
	bad += g12a_mrg_run(7, G12A_MRG_PBASE, G12A_MRG_FAST_WORDS, 1);
	return bad;
}

static u32 g12a_mrg_probe_cur(void)
{
	return g12a_mrg_fast ? g12a_mrg_probe_fast() : g12a_mrg_probe();
}

/*
 * Delay codec, transcribed from fffba174 / fffba1e0
 * (g12a-a644-call-sequence.md §4.6).  Class 4 (TxDqDly) is a non-linear
 * coarse/fine split: stepping the RAW CSR skips and repeats codes at
 * every 32-step boundary, so a sweep must step in DECODED units and
 * re-encode.  Class 3 (RxClkDly) is the identity in its low 6 bits.
 *
 * The coarse field is FIVE bits, not three, and bit [5] is RESERVED.  The
 * original `(raw & 0x3Fu) + (((raw >> 6) & 7u) << 5)` is right only for the
 * two 9-bit classes (TxDqDly/TxDqsDly, coarse [8:6]); it truncates [10:9] of
 * RxEnDly (class 6, coarse [10:6], PUB 11.3), which is 11 bits wide.  This
 * board trains RxEnDly to raw 0x31B: the 3-bit form decodes it to 155 instead
 * of 411, so a +0 step re-encodes to 0x011B and moves the gate -256 fine
 * units (-8 UI).  Host audit (mrg_codec_check.py, amlogic
 * software/boot/g12/delay-codec-verification.md): over the canonical raw
 * domain (reserved bit 5 clear) the 3-bit form fails 768/1024 on RxEnDly and
 * the form below fails 0/1024 on all three fields, while decoding all 25
 * transcribed trained Tx raws bit-identically to the old form.  enc4() is
 * unchanged -- it is already the general form and never sets bit 5.
 */
static u32 g12a_mrg_dec4(u16 raw)
{
	return (raw & 0x1Fu) + (((raw >> 6) & 0x1Fu) << 5);
}

/*
 * The encode side is the GENERAL form, not the doc's `v > 0x3F ->
 * (v & 0x1F) + 0x60` branch.  Measured: TxDqsDly trains to raw 0x111
 * on this board, which decodes to 145 — and only (v & 0x1F) |
 * ((v >> 5) << 6) re-encodes 145 back to 0x111.  The doc's saturating
 * branch returns 0x71, which decodes to 81, and writing it collapsed
 * the DQS delay: boot margin-2d-v3 failed the very first ±1 TxDqsDly
 * step with all 0x40000 words bad and readback 007f.  The general form
 * also reproduces every trained TxDqDly raw exactly (73 -> 0x89).
 */
static u16 g12a_mrg_enc4(u32 v)
{
	return (u16)((v & 0x1Fu) | ((v >> 5) << 6));
}

static void g12a_mrg_vref_set(int delta)
{
	u32 d, l;

	for (d = 0; d < 4; d++) {
		for (l = 0; l < 9; l++) {
			int v = (int)(g12a_mrg_vref0[d][l] & 0x7Fu) + delta;
			u16 keep;

			if (v < 0)
				v = 0;
			if (v > 0x7F)
				v = 0x7F;
			keep = g12a_mrg_vref0[d][l] & (u16)~0x7Fu;
			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					G12A_MRG_VREFDAC0)),
					 (u16)(keep | (u16)v));
			keep = g12a_mrg_vref1[d][l] & (u16)~0x7Fu;
			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					G12A_MRG_VREFDAC1)),
					 (u16)(keep | (u16)v));
		}
	}
}

static void g12a_mrg_vref_restore(void)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 9; l++) {
			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					G12A_MRG_VREFDAC0)),
					 g12a_mrg_vref0[d][l]);
			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					G12A_MRG_VREFDAC1)),
					 g12a_mrg_vref1[d][l]);
		}
}

static void g12a_mrg_txdq_set(int delta)
{
	u32 d, l, t;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 9; l++)
			for (t = 0; t < 2; t++) {
				int v = (int)g12a_mrg_dec4(
						g12a_mrg_txdq[d][l][t]) + delta;

				if (v < 0)
					v = 0;
				if (v > 255)
					v = 255;
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_TXDQDLY_TG0 + t)),
						 g12a_mrg_enc4((u32)v));
			}
}

static void g12a_mrg_txdq_restore(void)
{
	u32 d, l, t;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 9; l++)
			for (t = 0; t < 2; t++)
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_TXDQDLY_TG0 + t)),
						 g12a_mrg_txdq[d][l][t]);
}

/* RxClkDly and its complement move together — they are the DQS pair. */
static void g12a_mrg_rxclk_set(int delta)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++) {
				int v = (int)(g12a_mrg_rxclk[d][n][t] & 0x3Fu)
					+ delta;
				int c = (int)(g12a_mrg_rxclkc[d][n][t] & 0x3Fu)
					+ delta;
				u16 keep;

				if (v < 0)
					v = 0;
				if (v > 0x3F)
					v = 0x3F;
				if (c < 0)
					c = 0;
				if (c > 0x3F)
					c = 0x3F;
				keep = g12a_mrg_rxclk[d][n][t] & (u16)~0x3Fu;
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKDLY_TG0 + t)),
						 (u16)(keep | (u16)v));
				keep = g12a_mrg_rxclkc[d][n][t] & (u16)~0x3Fu;
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKCDLY_TG0 + t)),
						 (u16)(keep | (u16)c));
			}
}

static void g12a_mrg_rxclk_restore(void)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++) {
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKDLY_TG0 + t)),
						 g12a_mrg_rxclk[d][n][t]);
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_RXCLKCDLY_TG0 + t)),
						 g12a_mrg_rxclkc[d][n][t]);
			}
}

/*
 * A sweep step that reports only pass/fail cannot tell "this knob has
 * headroom" from "this knob is not live" — a write that never reaches
 * the register, or is reloaded behind us, also never fails.  Every step
 * therefore carries the READBACK of one representative CSR of the class
 * (DBYTE0 lane0 / nibble0, Tg0), taken after the write.
 */
static u32 g12a_mrg_rb_csr;

/*
 * TxDqsDly (class 2) — the DQS transmit delay.  This is the knob the
 * write eye actually hangs off: TxDqDly is per-DQ-bit deskew, and this
 * is the strobe the DRAM samples against.  Same coarse/fine codec as
 * class 4.
 */
static void g12a_mrg_txdqs_set(int delta)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++) {
				int v = (int)g12a_mrg_dec4(
						g12a_mrg_txdqs[d][n][t])
					+ delta;

				if (v < 0)
					v = 0;
				if (v > 255)
					v = 255;
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_TXDQSDLY_TG0 + t)),
						 g12a_mrg_enc4((u32)v));
			}
}

static void g12a_mrg_txdqs_restore(void)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++)
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_TXDQSDLY_TG0 + t)),
						 g12a_mrg_txdqs[d][n][t]);
}

/* RxPBDly (class 5) — per-DQ-bit read deskew, linear in its low 6 bits. */
static void g12a_mrg_rxpb_set(int delta)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 9; l++) {
			int v = (int)(g12a_mrg_rxpb[d][l] & 0x3Fu) + delta;
			u16 keep;

			if (v < 0)
				v = 0;
			if (v > 0x3F)
				v = 0x3F;
			keep = g12a_mrg_rxpb[d][l] & (u16)~0x3Fu;
			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					G12A_MRG_RXPBDLY_TG0)),
					 (u16)(keep | (u16)v));
		}
}

static void g12a_mrg_rxpb_restore(void)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 9; l++)
			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					G12A_MRG_RXPBDLY_TG0)),
					 g12a_mrg_rxpb[d][l]);
}

static void g12a_mrg_step_report(const char *tag, int delta, u32 bad)
{
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_putc(delta < 0 ? '-' : '+');
	g12a_uart_hex8((u8)(delta < 0 ? -delta : delta));
	g12a_uart_putc(' ');
	g12a_uart_hex32(bad);
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_MRG_PA(g12a_mrg_rb_csr)));
	/*
	 * Trailing field, appended rather than inserted so the existing
	 * MGS/MGX parsers keep working: the OR of the failing words' error
	 * bits.  On an aggregate sweep this says how much of the bus went
	 * with the knob; on a per-lane sweep it is the addressing check.
	 */
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_puts("\n");
}

/*
 * One shmoo arm: step away from the trained centre in one direction
 * until the probe fails, then restore.  The reported edge is the last
 * offset that still passed, so the pass/fail boundary is measured, not
 * inferred from a single pass at the centre.
 */
static int g12a_mrg_sweep(const char *tag, void (*set)(int),
			  void (*restore)(void), int dir, int limit)
{
	int k;
	int last = 0;

	for (k = 1; k <= limit; k++) {
		u32 bad;

		set(dir * k);
		bad = g12a_mrg_probe_cur();
		g12a_mrg_step_report(tag, dir * k, bad);
		if (bad)
			break;
		last = k;
	}
	restore();
	return last;
}

/*
 * Liveness check.  A sweep that reaches its limit without failing is
 * ambiguous: either the knob has that much headroom, or writing it does
 * nothing to the data path.  Slamming the class to both ends of its own
 * range separates them — a delay line driven to 0 and to full scale
 * cannot leave a 3216 MT/s eye intact if the PHY is using it.
 */
static void g12a_mrg_extreme(const char *tag, void (*set)(int),
			     void (*restore)(void), int lo, int hi)
{
	set(lo);
	g12a_mrg_step_report(tag, lo, g12a_mrg_probe_cur());
	set(hi);
	g12a_mrg_step_report(tag, hi, g12a_mrg_probe_cur());
	restore();
	g12a_mrg_step_report(tag, 0, g12a_mrg_probe_cur());
}

/*
 * ---------------------------------------------------------------------
 * Environmental axes: die temperature and the VDDEE rail.
 *
 * Everything above is measured at one temperature and one voltage, which
 * is the stated limit of amlogic/software/boot/g12/dram-margin.md.  Both
 * axes are reachable from this stage, and both have to be measured HERE
 * rather than from U-Boot proper: the PHY window is firewalled at EL2
 * after BL31, so the eye can only be re-measured where the shmoo lives.
 *
 * TEMPERATURE.  The SoC's own TSENSOR blocks answer over plain MMIO with
 * no SCPI client and no driver: CPU sensor at 0xFF634800, DDR sensor at
 * 0xFF634C00, both clocked by the `ts` gate in HHI_TS_CLK_CNTL and both
 * trimmed by a per-chip efuse word in the AO-secure block.  Register
 * layout, calibration constants and the code->millicelsius formula are
 * transcribed from drivers/thermal/amlogic_thermal.c (A/B/m/n for
 * "amlogic,g12a-thermal"), so a reading here is directly comparable to
 * what Linux would report for the same silicon.  There is no chamber on
 * this rig: the only reachable delta is self-heating under our own load,
 * so the honest measurement is idle temperature at entry, temperature
 * sampled through the soak, and the eye re-measured at the top of
 * whatever range that reaches.
 *
 * VOLTAGE.  VDDEE is the SoC digital-core rail that feeds the DDR PHY,
 * driven by the AO PWM_B duty cycle at 0xFF807004 -- the register
 * g12a_stage_init_sm1() above already programs to the board's 840 mV
 * operating point.  Its encoding is (high << 16) | low with high + low
 * held at 18, and the board's table is `pwm_voltage_table_ee_new`,
 * 700-880 mV in 10 mV steps over 19 entries with duty INVERTED (more
 * high time = lower voltage), so
 *
 *     VDDEE_mV = 880 - 10 * high,  high = 0..18
 *
 * which returns exactly the 840 mV the header comment records for the
 * board's own 0x0004000e, and matches the VCCK table on the same board
 * (its 800 mV word 0x0018000a, duty 24/34, against the DT's 730-1011 mV
 * inverted range, gives 813 mV against the table's 800 mV).  Every value
 * written here is an entry of that vendor table, so no rail can be driven
 * outside its documented range in either direction even if the polarity inference
 * were backwards; the sweep is further bounded to high = 0..8, which is
 * 880..800 mV under that mapping and never below the vendor's own
 * 770 mV suspend point under the inverted one.  The nominal value is
 * rewritten at the end of the sweep and re-probed.
 */
#define G12A_HHI_TS_CLK_CNTL	0xFF63C190UL	/* hiu 0xFF63C000 + 0x64<<2 */
#define G12A_TS_CPU_BASE	0xFF634800UL
#define G12A_TS_DDR_BASE	0xFF634C00UL
#define G12A_TS_CFG_REG1	0x04UL
#define G12A_TS_STAT0		0x40UL
/* FILTER_EN | VCM_EN | VBG_EN | DEM_EN | CH_SEL=3 (amlogic_thermal.c) */
#define G12A_TS_CFG_REG1_EN	0x0000062BU
/* aobus 0xFF800000 + sec_AO@0x140 + per-sensor efuse offset */
#define G12A_TS_TRIM_CPU	0xFF800268UL	/* 0x140 + 0x128 */
#define G12A_TS_TRIM_DDR	0xFF800230UL	/* 0x140 + 0x0F0 */
#define G12A_TS_CAL_A		9411
#define G12A_TS_CAL_B		3159
#define G12A_TS_CAL_M		424
#define G12A_TS_CAL_N		324

#define G12A_MRG_VDDEE_NOM_HIGH	4U		/* 0x0004000e = 840 mV */
#define G12A_MRG_VDDEE_MAX_HIGH	8U		/* 800 mV, sweep floor */
#define G12A_MRG_SOAK_CYCLES	3

static u32 g12a_ts_trim_cpu;
static u32 g12a_ts_trim_ddr;

static void g12a_uart_dec(int v)
{
	char b[12];
	int i = 0;
	unsigned int u;

	if (v < 0) {
		g12a_uart_putc('-');
		u = (unsigned int)(-v);
	} else {
		u = (unsigned int)v;
	}
	if (!u) {
		g12a_uart_putc('0');
		return;
	}
	while (u) {
		b[i++] = (char)('0' + (u % 10u));
		u /= 10u;
	}
	while (i--)
		g12a_uart_putc(b[i]);
}

/* Transcribed from amlogic_thermal_code_to_millicelsius(), same order. */
static int g12a_ts_mc(u32 trim, u32 code)
{
	s64 factor, uptat, uefuse, t;

	uefuse = (trim & 0x8000U) ?
		 (s64)(u32)(~(trim & 0x7FFFU) + 1U) : (s64)(trim & 0x7FFFU);

	factor = (s64)G12A_TS_CAL_N * (s64)code;
	factor = factor / 100;

	uptat = (s64)code * (s64)G12A_TS_CAL_M;
	uptat = uptat / 100;
	uptat = uptat * 65536;
	uptat = uptat / (65536 + factor);

	t = (uptat + uefuse) * (s64)G12A_TS_CAL_A;
	t = t / 65536;
	t = (t - G12A_TS_CAL_B) * 100;

	return (int)t;
}

static void g12a_ts_init(void)
{
	u32 old = readl((void *)G12A_HHI_TS_CLK_CNTL);

	g12a_ts_trim_cpu = readl((void *)G12A_TS_TRIM_CPU);
	g12a_ts_trim_ddr = readl((void *)G12A_TS_TRIM_DDR);

	/* xtal/48 = 500 kHz, gate on.  Divider field is (value + 1). */
	writel(0x0000012FU, (void *)G12A_HHI_TS_CLK_CNTL);

	writel(G12A_TS_CFG_REG1_EN, (void *)(G12A_TS_CPU_BASE +
					     G12A_TS_CFG_REG1));
	writel(G12A_TS_CFG_REG1_EN, (void *)(G12A_TS_DDR_BASE +
					     G12A_TS_CFG_REG1));

	g12a_uart_puts("MGE init tsclk ");
	g12a_uart_hex32(old);
	g12a_uart_puts(" -> ");
	g12a_uart_hex32(readl((void *)G12A_HHI_TS_CLK_CNTL));
	g12a_uart_puts(" trim ");
	g12a_uart_hex32(g12a_ts_trim_cpu);
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_ts_trim_ddr);
	g12a_uart_puts("\n");
}

/*
 * Both sensors are printed with their RAW code beside the converted
 * value: the two have independent efuse trims, so their agreement is an
 * internal control on the conversion, and a stuck raw code cannot hide
 * behind a plausible-looking temperature.
 */
static void g12a_mrg_temp(const char *tag)
{
	u32 cc = readl((void *)(G12A_TS_CPU_BASE + G12A_TS_STAT0)) & 0xFFFFU;
	u32 dc = readl((void *)(G12A_TS_DDR_BASE + G12A_TS_STAT0)) & 0xFFFFU;

	g12a_uart_puts("MGE ");
	g12a_uart_puts(tag);
	g12a_uart_puts(" cpu ");
	g12a_uart_dec(g12a_ts_mc(g12a_ts_trim_cpu, cc));
	g12a_uart_puts(" ddr ");
	g12a_uart_dec(g12a_ts_mc(g12a_ts_trim_ddr, dc));
	g12a_uart_puts(" raw ");
	g12a_uart_hex16((u16)cc);
	g12a_uart_putc(' ');
	g12a_uart_hex16((u16)dc);
	g12a_uart_puts("\n");
}

/*
 * udelay() does not delay in this xPL phase (measured: a nominal 20 s
 * loop completed in under 4 ms, scp-bringup.md §4.1), so every wait here
 * counts CNTPCT_EL0 directly.  CNTFRQ_EL0 reads 0 on this part, so the
 * budget is expressed in ticks and deliberately generous: 2e6 ticks is
 * >= 40 ms at any plausible rate for this counter.
 */
static void g12a_mrg_wait_ticks(u64 n)
{
	u64 t0 = g12a_mrg_ticks();

	while (g12a_mrg_ticks() - t0 < n)
		g12a_wdt_pet_safe();
}

static void g12a_mrg_vddee_set(u32 high)
{
	if (high > 18U)
		high = G12A_MRG_VDDEE_NOM_HIGH;
	writel((high << 16) | (18U - high), (void *)G12A_AO_PWM_PWM_B);
	g12a_mrg_wait_ticks(2000000ULL);
}

static void g12a_mrg_vddee_report(const char *tag, u32 high)
{
	g12a_uart_puts("MGV ");
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_dec((int)(880U - 10U * high));
	g12a_uart_puts("mV pwm ");
	g12a_uart_hex32(readl((void *)G12A_AO_PWM_PWM_B));
	g12a_uart_puts("\n");
}

/*
 * ---------------------------------------------------------------------
 * T19 — the PER-LANE eye.
 *
 * Everything above moves a whole delay CLASS at once, so every number it
 * produces is an all-lanes-together eye: the reported edge is the first
 * offset at which the WORST lane fails, and the distribution behind it
 * is invisible.  P2.2 (open read/write DQ centring) has to land per
 * lane, so "equivalent to the vendor" cannot be judged against an
 * aggregate — which is the reason this pass exists.
 *
 * WHICH AXES CAN BE PER-LANE AT ALL is a hardware question, answered
 * before writing the sweep rather than after:
 *
 *   TxDqDlyTg0/Tg1 (0xC0, class 4, 9 per DBYTE)   per-lane, and INERT.
 *      Driven to raw 0000 and 01DF on every lane, in our TPL and inside
 *      the vendor's own product BL2, the probe stays at zero errors
 *      while TxDqsDly and RxPBDly controls fire in the same run
 *      (amlogic/software/boot/g12/txdqdly-deskew.md).  Not swept here.
 *   RxPBDlyTg0 (0x68, class 5, 9 per DBYTE)       per-lane and LIVE.
 *   VrefDAC0/1 (0x40/0x30, class 9, 8+1 per DBYTE) per-lane and LIVE
 *      after 2D; inert at 1D-only (dram-margin.md §6).
 *   TxDqsDly (0xD0, class 2), RxClkDly (0x8C, class 3) and RxEnDly
 *      (0x80, class 6) are per NIBBLE, and ATxDly (ANIB 0x80, class 1)
 *      is per ANIB.  They cannot produce a per-lane number at all.
 *
 * So the live per-lane axes are BOTH on the read side, and there is no
 * live per-lane WRITE axis on this part.  The instance counts are the
 * Synopsys PUB's (TxDqDlyTg0 "0x0C0 controls the lane0 write DQ timing
 * ... 0x8C0 the lane{DMDBI}", 36 instances) and the vendor blob's own
 * class sweeper fffb502c, which builds exactly (lane << 8) | off for
 * classes 4 and 5 and (lane << 8) for class 9.
 *
 * Method per lane: step THAT lane's CSR away from its own trained value
 * with every other lane left trained, probe, stop at the first failure,
 * restore the lane and re-probe.  Both directions.  A sweep that runs
 * into the register's own floor/ceiling without failing is reported as
 * a LIMIT, not as an edge — the same distinction §4 of dram-margin.md
 * had to make for the aggregate read-side sweeps.
 */
/*
 * Abort budget, in CNTPCT_EL0 ticks.  MEASURED on boot t19-a: the first
 * value tried here, 0x40000000, stopped the pass 25 lanes in — the tick
 * rate on this part is ~107 MHz (calibrated in dram-margin.md §7 against
 * the host clock, since CNTFRQ_EL0 reads 0), not the ~24 MHz the round
 * number silently assumed, and a 64 KiB probe costs ~7.8 ms.  The full
 * pass measures at ~30 s; 0x600000000 is ~240 s, which is a runaway guard
 * rather than a schedule, and still inside the host's RS2 wait.
 */
#define G12A_MRG_LANE_TIME_LIMIT	0x600000000ULL

static u64 g12a_mrg_lane_t0;

static void g12a_mrg_lane_edge(const char *tag, u32 d, u32 l, int dir,
			       u32 edge, u32 bad, int limited)
{
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)d);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)l);
	g12a_uart_putc(' ');
	g12a_uart_putc(dir < 0 ? '-' : '+');
	g12a_uart_hex8((u8)edge);
	g12a_uart_putc(' ');
	g12a_uart_hex32(bad);
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_putc(' ');
	g12a_uart_putc(limited ? 'L' : 'F');
	g12a_uart_puts("\n");
}

/* RxPBDly (class 5) for ONE lane, everything else left trained. */
static void g12a_mrg_rxpb_lane_set(u32 d, u32 l, int v)
{
	u16 keep = g12a_mrg_rxpb[d][l] & (u16)~0x3Fu;

	g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, G12A_MRG_RXPBDLY_TG0)),
			 (u16)(keep | (u16)v));
}

static void g12a_mrg_rxpb_lane_restore(u32 d, u32 l)
{
	g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, G12A_MRG_RXPBDLY_TG0)),
			 g12a_mrg_rxpb[d][l]);
}

/* VrefDAC0 and VrefDAC1 are the two timing groups of the same lane. */
static void g12a_mrg_vref_lane_set(u32 d, u32 l, int v)
{
	u16 keep0 = g12a_mrg_vref0[d][l] & (u16)~0x7Fu;
	u16 keep1 = g12a_mrg_vref1[d][l] & (u16)~0x7Fu;

	g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, G12A_MRG_VREFDAC0)),
			 (u16)(keep0 | (u16)v));
	g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, G12A_MRG_VREFDAC1)),
			 (u16)(keep1 | (u16)v));
}

static void g12a_mrg_vref_lane_restore(u32 d, u32 l)
{
	g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, G12A_MRG_VREFDAC0)),
			 g12a_mrg_vref0[d][l]);
	g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l, G12A_MRG_VREFDAC1)),
			 g12a_mrg_vref1[d][l]);
}

/*
 * One direction of one lane.  `trained` and `mask` are in raw CSR units:
 * both live per-lane axes are linear in their low bits — class 5 is
 * 6-bit and class 9 is 7-bit, and neither goes through the coarse/fine
 * delay codec that classes 2 and 4 need.
 */
static void g12a_mrg_lane_arm(const char *tag, u32 d, u32 l, int dir,
			      int trained, int mask,
			      void (*set)(u32, u32, int),
			      void (*rst)(u32, u32))
{
	int k;
	u32 bad = 0;
	int last = 0;
	int limited = 1;

	for (k = 1; k <= mask; k++) {
		int v = trained + dir * k;

		if (v < 0 || v > mask)
			break;
		set(d, l, v);
		bad = g12a_mrg_probe_fast();
		if (bad) {
			limited = 0;
			break;
		}
		last = k;
	}
	rst(d, l);
	g12a_mrg_lane_edge(tag, d, l, dir, (u32)last, bad, limited);
}

static void g12a_mrg_lane_axis(const char *tag, u32 mask,
			       void (*set)(u32, u32, int),
			       void (*rst)(u32, u32),
			       const u16 *trained, u32 stride)
{
	u32 d, l;

	for (d = 0; d < 4; d++) {
		for (l = 0; l < 9; l++) {
			int t = (int)(trained[d * stride + l] & (u16)mask);
			u32 bad;

			if (g12a_mrg_ticks() - g12a_mrg_lane_t0 >
			    G12A_MRG_LANE_TIME_LIMIT) {
				g12a_uart_puts("MGL abort\n");
				return;
			}
			/*
			 * The trained value is printed beside the lane so a
			 * per-lane edge is readable as an absolute CSR
			 * value and not only as an offset — an open
			 * training implementation is judged on where it
			 * LANDS, which is the whole point of this pass.
			 */
			g12a_uart_puts(tag);
			g12a_uart_puts(" t ");
			g12a_uart_hex8((u8)d);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)l);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)t);
			g12a_uart_puts("\n");

			g12a_mrg_lane_arm(tag, d, l, +1, t, (int)mask,
					  set, rst);
			g12a_mrg_lane_arm(tag, d, l, -1, t, (int)mask,
					  set, rst);
			/*
			 * Restore-and-verify.  A lane left broken by its own
			 * sweep would poison every lane measured after it,
			 * and the damage would read as a narrow eye rather
			 * than as the bug it is.
			 */
			bad = g12a_mrg_probe_fast();
			if (bad) {
				g12a_uart_puts(tag);
				g12a_uart_puts(" z ");
				g12a_uart_hex8((u8)d);
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)l);
				g12a_uart_putc(' ');
				g12a_uart_hex32(bad);
				g12a_uart_puts("\n");
			}
		}
	}
}

static void g12a_mrg_per_lane(void)
{
	u32 bad;

	g12a_uart_puts("MGL begin\n");
	g12a_mrg_lane_t0 = g12a_mrg_ticks();
	g12a_mrg_fast = 1;

	bad = g12a_mrg_probe_fast();
	g12a_uart_puts("MGL centre ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (bad) {
		g12a_uart_puts("MGL abort: centre fails\n");
		g12a_mrg_fast = 0;
		return;
	}

	g12a_mrg_lane_axis("MGLp", 0x3F, g12a_mrg_rxpb_lane_set,
			   g12a_mrg_rxpb_lane_restore,
			   &g12a_mrg_rxpb[0][0], 9);
	g12a_mrg_lane_axis("MGLv", 0x7F, g12a_mrg_vref_lane_set,
			   g12a_mrg_vref_lane_restore,
			   &g12a_mrg_vref0[0][0], 9);

	g12a_mrg_fast = 0;
	g12a_uart_puts("MGL end ");
	g12a_uart_hex32(g12a_mrg_probe());
	g12a_uart_puts("\n");
}

/*
 * ---------------------------------------------------------------------
 * P2 — OPEN TRAINING.  amlogic/software/boot/g12/p2-open-training.md.
 *
 * Everything above measures what aml_ddr.fw's training produced.  This
 * block is the first code in the arc that tries to PRODUCE it: an open
 * per-lane read-side centring that starts from a de-trained bus, plus
 * the two pin-mux steps (MID2/MID3) an open staging half needs.
 *
 * Scope, and the two things it deliberately does not do:
 *
 *   - MID1 (fffb6328, the DMC/PCTL program) is NOT implemented.  Its ~45
 *     register values come from `tm`, the output of the timing engine
 *     fffb6a1c, which g12a-a644-call-sequence.md §7 records as sampled
 *     and not transcribed.  There is no open `tm`, so there can be no
 *     open MID1.  g12a_p2_dmc_dump() captures MID1's OUTPUT instead, so
 *     a future timing engine has a target to check itself against.
 *   - CA training (the plan's P2.4) is not attempted at all.  Host MRR
 *     is unestablished and that is the arc's identified research risk.
 *
 * The oracle is g12a_mrg_xor — the OR of (got ^ expected) over the
 * failing words of a probe — and NOT the DMC BIST, which does not answer
 * from the TPL and, more decisively, returns only a scalar error count
 * with no per-lane field (dmc-bist-transcription.md §6).  That
 * substitution is what makes g12a_p2_sweep() possible: because a failing
 * word names WHICH DQ bits are wrong, all 32 lanes can be de-trained at
 * once and swept in lockstep, so one probe per code yields the whole
 * bus's per-lane window map.  With a scalar oracle a single bad lane
 * blackens every code for every lane and this loop cannot exist.
 *
 * Nothing here ships: every value written is restored before return, so
 * the chain continues on the vendor's trained state and the six
 * acceptance items are unaffected.
 */
/*
 * Phase selector, like G12A_MRG_T19 before it: 1 makes P2 the live phase
 * set.  This is NOT a product default in any sense — the whole file
 * section is inside CONFIG_MESON_G12A_DRAM_MARGIN, which no product
 * config sets, so the shipping chain neither compiles nor runs a line of
 * it.  Keeping the default at the live phase is what makes the image
 * that gets booted the same one that is committed.
 */
#ifndef G12A_MRG_P2
#define G12A_MRG_P2		1
#endif

/*
 * P6 -- write-DQS centring on the vendor-trained bus, in the T19 phase set
 * (G12A_MRG_P2=0).  It reuses G2's per-nibble TxDqsDly window search and
 * KEEPS the centres, shifting both timing groups of a nibble by the same
 * delta so the vendor's TG0/TG1 relation stands; the shmoo that follows then
 * measures the write eye around the applied point.  Needs P2's helpers (the
 * lane->DQ-bit map), so the P2 block compiles under either gate.  Its
 * default is set at the top of this block (1 with
 * CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE, else 0).
 */

#if G12A_MRG_P2 || G12A_MRG_P6

#define G12A_P2_RXPB_MASK	0x3Fu	/* class 5, 6-bit linear */
#define G12A_P2_VREF_MASK	0x7Fu	/* class 9, 7-bit linear */
#define G12A_P2_REFINE_SPAN	8	/* +-codes around the converged value */

/* One xor word per swept code; 128 is the wider of the two axes. */
static u32 g12a_p2_map[G12A_P2_VREF_MASK + 1];
/* Converged per-lane value, indexed [dbyte][lane]; lane 8 (DM) unused. */
static u8 g12a_p2_open[4][8];

/*
 * DQ bit of a (dbyte, lane).  g12a_solitude_dq_remap is indexed BY DQ
 * bit and holds the global lane number, so this is its inverse — the
 * direction the oracle needs.  The forward direction was validated at
 * runtime 64 of 64 by the per-lane pass (per-lane-eye-reference.md
 * §4.1), which is why this table is trusted here.
 */
static int g12a_p2_dqbit(u32 d, u32 l)
{
	u32 b;
	u8 want = (u8)(d * 8u + l);

	for (b = 0; b < G12A_DS_DQ_REMAP_N; b++)
		if (g12a_solitude_dq_remap[b] == want)
			return (int)b;
	return -1;
}

/*
 * P2.1a/P2.1b — open MID2 (AC pin-mux) and MID3 (DQ pin-mux).
 *
 * Both are run COMPARE-ONLY first: the open writer computes the
 * {csr, value} pairs from the transcribed formulas and the CSR is READ
 * to see what aml_ddr.fw actually left there.  A read cannot perturb the
 * boot, so a formula error is a result rather than a hazard.  The values
 * are only re-applied if every entry already matched, in which case the
 * write is a no-op by construction and the probe afterwards proves it.
 *
 * MID3 (a644 spec §3.2 S7): csr = 0x10000 | ((i>>3)<<12) | (0xA0+(i&7)),
 *                           val = byte & 7, i = 0..31.
 * MID2 (S6):                csr = (pstate<<20) | 0x20000 | (base+i),
 *                           base = 0x100, 0x106 for i>9 when the type
 *                           byte is 2 or 3; count 20 for type 2.
 * pstate is 0 (a644 forces ctl[441]=0 at three points).
 */
static u32 g12a_p2_pinmux(int apply)
{
	u32 i, bad = 0;

	for (i = 0; i < G12A_DS_DQ_REMAP_N; i++) {
		u32 csr = 0x10000u | ((i >> 3) << 12) | (0xA0u + (i & 7u));
		u16 want = (u16)(g12a_solitude_dq_remap[i] & 7u);
		u16 got = g12a_phy_read16(G12A_MRG_PA(csr));

		if ((got & 7u) != want)
			bad++;
		g12a_uart_puts("P2Mq ");
		g12a_uart_hex8((u8)i);
		g12a_uart_putc(' ');
		g12a_uart_hex32(csr);
		g12a_uart_putc(' ');
		g12a_uart_hex16(want);
		g12a_uart_putc(' ');
		g12a_uart_hex16(got);
		g12a_uart_puts("\n");
	}

	for (i = 0; i < G12A_AC_PINMUX_N; i++) {
		u32 base = (i > 9u) ? 0x106u : 0x100u;
		u32 csr = 0x20000u | (base + i);
		u16 want = (u16)g12a_ac_pinmux_lpddr4[i];
		u16 got = g12a_phy_read16(G12A_MRG_PA(csr));

		if (got != want)
			bad++;
		g12a_uart_puts("P2Ma ");
		g12a_uart_hex8((u8)i);
		g12a_uart_putc(' ');
		g12a_uart_hex32(csr);
		g12a_uart_putc(' ');
		g12a_uart_hex16(want);
		g12a_uart_putc(' ');
		g12a_uart_hex16(got);
		g12a_uart_puts("\n");
	}

	g12a_uart_puts("P2M bad ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");

	/*
	 * Re-apply only on a clean compare.  Writing values that differ
	 * from what the blob programmed would be an untested change to a
	 * trained PHY, which is not what this probe is for.
	 */
	if (apply && !bad) {
		for (i = 0; i < G12A_DS_DQ_REMAP_N; i++)
			g12a_phy_write16(G12A_MRG_PA(0x10000u |
					 ((i >> 3) << 12) |
					 (0xA0u + (i & 7u))),
					 (u16)(g12a_solitude_dq_remap[i] & 7u));
		for (i = 0; i < G12A_AC_PINMUX_N; i++)
			g12a_phy_write16(G12A_MRG_PA(0x20000u |
					 ((i > 9u ? 0x106u : 0x100u) + i)),
					 (u16)g12a_ac_pinmux_lpddr4[i]);
		g12a_uart_puts("P2M reapply ");
		g12a_uart_hex32(g12a_mrg_probe_fast());
		g12a_uart_puts("\n");
	}
	return bad;
}

/*
 * P2.1c — the delay codec is already open (g12a_mrg_dec4/enc4, corrected
 * once by measurement: dram-margin.md §3).  Assert rather than re-derive:
 * every trained raw of the two coarse/fine classes must round-trip.
 *
 * The two Tx classes are 9 bits (coarse [8:6]); RxEnDly (class 6) is 11
 * (coarse [10:6]).  Covering only the Tx classes is what let the 3-bit
 * coarse mask in g12a_mrg_dec4() survive: every trained Tx raw round-trips
 * under both the broken and the corrected form, so P2K read 0 while RxEnDly
 * 0x31B was decoding to 155 instead of 411.  RxEnDly is not captured into a
 * static array (it is read inline for the MGCk row), so read it here.
 * Failures print P2Ke, alongside P2Kd/P2Ks.
 */
static void g12a_p2_codec_assert(void)
{
	u32 d, l, t, bad = 0;

	for (d = 0; d < 4; d++) {
		for (l = 0; l < 9; l++)
			for (t = 0; t < 2; t++) {
				u16 raw = g12a_mrg_txdq[d][l][t];

				if (g12a_mrg_enc4(g12a_mrg_dec4(raw)) != raw) {
					bad++;
					g12a_uart_puts("P2Kd ");
					g12a_uart_hex16(raw);
					g12a_uart_puts("\n");
				}
			}
		for (l = 0; l < 2; l++)
			for (t = 0; t < 2; t++) {
				u16 raw = g12a_mrg_txdqs[d][l][t];

				if (g12a_mrg_enc4(g12a_mrg_dec4(raw)) != raw) {
					bad++;
					g12a_uart_puts("P2Ks ");
					g12a_uart_hex16(raw);
					g12a_uart_puts("\n");
				}
			}
		for (l = 0; l < 2; l++)
			for (t = 0; t < 2; t++) {
				u16 raw = g12a_phy_read16(
					G12A_MRG_PA(G12A_MRG_DB(d, l,
						G12A_MRG_RXENDLY_TG0 + t)));

				if (g12a_mrg_enc4(g12a_mrg_dec4(raw)) != raw) {
					bad++;
					g12a_uart_puts("P2Ke ");
					g12a_uart_hex16(raw);
					g12a_uart_puts("\n");
				}
			}
	}
	g12a_uart_puts("P2K bad ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
}

/*
 * P2.1d — MID1's OUTPUT, captured.  Not an implementation: it turns the
 * DMC/PCTL program from "unknown target, unknown derivation" into "known
 * target, unknown derivation", which is what a future open timing engine
 * needs to check itself against.  Read-only.
 */
static void g12a_p2_dmc_dump(void)
{
	ulong pa;

	for (pa = 0xFF638400UL; pa < 0xFF638500UL; pa += 16) {
		u32 i;

		g12a_uart_puts("P2D ");
		g12a_uart_hex32((u32)pa);
		for (i = 0; i < 4; i++) {
			g12a_uart_putc(' ');
			g12a_uart_hex32(readl((void *)(pa + i * 4)));
		}
		g12a_uart_puts("\n");
	}
	for (pa = 0xFF639000UL; pa < 0xFF639040UL; pa += 16) {
		u32 i;

		g12a_uart_puts("P2D ");
		g12a_uart_hex32((u32)pa);
		for (i = 0; i < 4; i++) {
			g12a_uart_putc(' ');
			g12a_uart_hex32(readl((void *)(pa + i * 4)));
		}
		g12a_uart_puts("\n");
	}
	g12a_uart_puts("P2D geo ");
	g12a_uart_hex32(readl((void *)0xFF638000UL));
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)0xFF639368UL));
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)0xFF800240UL));
	g12a_uart_puts("\n");
}

/*
 * P2.1e — OPEN MID1, compare-only.
 *
 * g12a_timing_engine() + g12a_mid1_compute() (arch-meson/g12a-ddr-timing.h)
 * are the transcription of aml_ddr.fw fffb6a1c + fffb6328.  They are pure
 * arithmetic over the ddr_set_t this TPL already built -- no MMIO, no mode
 * registers -- so this step only READS the DMC window and prints any register
 * whose open value differs from what the blob left.  It cannot perturb the
 * boot, which is why it runs before any write, exactly as P2.1a/b did.
 *
 * The byte-exact verdict does NOT rest on this call site: the same two
 * functions are compiled and run on the host by
 * amlogic/software/tests/g12a-timing-engine-check.py against the identical
 * captured target, 52 of 52.  This is on-target corroboration, not the proof.
 *
 * Format: "P2T <addr> <open> <blob>" per differing register, then
 *         "P2T bad <count>".  0xFF638464 is expected to differ: MID1 stores
 *         tm[124] = 0x12C and that register is an 8-bit field reading 0x2C.
 */
static void g12a_p2_mid1_compare(void)
{
	/*
	 * tm is LOCAL, as the OPEN path's already is (see the note in
	 * g12a_open_s2s5_timing_mid1()): it is written before it is read on
	 * every path, so the .bss zeroing bought nothing.  mid1 is the
	 * shared MID1 scratch, live only between the claim and the release.
	 */
	u32 tm[G12A_TM_WORDS];
	struct g12a_mid1_reg *mid1 = g12a_mid1_claim("P2T");
	const u8 *ds = (const u8 *)G12A_DDR_SET_BASE;
	u32 bad = 0;
	int i, n;

	if (!mid1)
		return;

	for (i = 0; i < G12A_TM_WORDS; i++)
		tm[i] = 0;

	if (g12a_timing_engine(ds, 0, G12A_AML_DDR_F920, tm)) {
		g12a_uart_puts("P2T refused\n");
		g12a_mid1_release();
		return;
	}
	n = g12a_mid1_compute(ds, 0, tm, mid1);

	for (i = 0; i < n; i++) {
		u32 live = readl((void *)(ulong)mid1[i].addr);

		if (live == mid1[i].val)
			continue;
		bad++;
		g12a_uart_puts("P2T ");
		g12a_uart_hex32(mid1[i].addr);
		g12a_uart_putc(' ');
		g12a_uart_hex32(mid1[i].val);
		g12a_uart_putc(' ');
		g12a_uart_hex32(live);
		g12a_uart_puts("\n");
	}
	g12a_uart_puts("P2T bad ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	g12a_mid1_release();
}

/* ---- P2.2: the two live per-lane read axes, set on ALL 32 DQ lanes ---- */

static void g12a_p2_rxpb_all(int v)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 8; l++) {
			u16 keep = g12a_mrg_rxpb[d][l] & (u16)~G12A_P2_RXPB_MASK;

			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					 G12A_MRG_RXPBDLY_TG0)),
					 (u16)(keep | (u16)v));
		}
}

static void g12a_p2_vref_all(int v)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 8; l++) {
			u16 k0 = g12a_mrg_vref0[d][l] & (u16)~G12A_P2_VREF_MASK;
			u16 k1 = g12a_mrg_vref1[d][l] & (u16)~G12A_P2_VREF_MASK;

			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					 G12A_MRG_VREFDAC0)),
					 (u16)(k0 | (u16)v));
			g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, l,
					 G12A_MRG_VREFDAC1)),
					 (u16)(k1 | (u16)v));
		}
}

/* Per-lane setters used once the bus is at its own converged values. */
static void g12a_p2_rxpb_one(u32 d, u32 l, int v)
{
	g12a_mrg_rxpb_lane_set(d, l, v);
}

static void g12a_p2_vref_one(u32 d, u32 l, int v)
{
	g12a_mrg_vref_lane_set(d, l, v);
}

static void g12a_p2_rxpb_open(void)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 8; l++)
			g12a_p2_rxpb_one(d, l, (int)g12a_p2_open[d][l]);
}

static void g12a_p2_vref_open(void)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 8; l++)
			g12a_p2_vref_one(d, l, (int)g12a_p2_open[d][l]);
}

/*
 * The lockstep sweep.  Every DQ lane is driven to the SAME code and one
 * probe is taken; the xor word is that code's verdict for all 32 lanes
 * at once.  mask+1 probes give the complete per-lane window map of a bus
 * on which no lane carries a vendor-trained value — which is the whole
 * point: an open implementation may not be handed 31 trained lanes to
 * find the 32nd.
 *
 * The map is only valid if a lane's failure stays local to its own DQ
 * bit while every other lane is also out of position.  That is
 * prediction P2-3 and it is checked off-board from these lines, because
 * the raw xor per code is printed rather than a reduction of it.
 */
static void g12a_p2_sweep(const char *tag, u32 mask, void (*setall)(int))
{
	int v;

	for (v = 0; v <= (int)mask; v++) {
		u32 bad;

		setall(v);
		bad = g12a_mrg_probe_fast();
		g12a_p2_map[v] = g12a_mrg_xor;
		g12a_uart_puts(tag);
		g12a_uart_putc(' ');
		g12a_uart_hex8((u8)v);
		g12a_uart_putc(' ');
		g12a_uart_hex32(g12a_mrg_xor);
		g12a_uart_putc(' ');
		g12a_uart_hex32(bad);
		g12a_uart_puts("\n");
	}
}

/*
 * Reduce the map to one window per lane and choose the open value.
 *
 * The estimator is the centre of the longest run of codes at which the
 * lane's own DQ bit is clean.  It is deliberately the simplest thing
 * that could work, because the question this engagement asks is whether
 * an open implementation can re-derive the vendor's state at all — not
 * whether a cleverer estimator can beat it.  Where the run reaches the
 * register limit that is reported (`L`), because a window bounded by the
 * CSR rather than by data has no centre to speak of.
 */
static void g12a_p2_converge(const char *tag, u32 mask, const u16 *trained,
			     u32 stride, u32 tmask)
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 8; l++) {
			int bit = g12a_p2_dqbit(d, l);
			int v, run = 0, best = 0, blo = -1, bhi = -1;
			int lo = -1;
			int cen;

			for (v = 0; v <= (int)mask; v++) {
				int clean = bit >= 0 &&
					!((g12a_p2_map[v] >> bit) & 1u);

				if (clean) {
					if (lo < 0)
						lo = v;
					run++;
					if (run > best) {
						best = run;
						blo = lo;
						bhi = v;
					}
				} else {
					run = 0;
					lo = -1;
				}
			}
			cen = best ? (blo + bhi) / 2 : -1;
			if (cen >= 0)
				g12a_p2_open[d][l] = (u8)cen;

			g12a_uart_puts(tag);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)d);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)l);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)(bit < 0 ? 0xFF : bit));
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)(trained[d * stride + l] & tmask));
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)(blo < 0 ? 0xFF : blo));
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)(bhi < 0 ? 0xFF : bhi));
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)(cen < 0 ? 0xFF : cen));
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)best);
			g12a_uart_putc(' ');
			g12a_uart_putc(blo == 0 ? 'l' : '-');
			g12a_uart_putc(bhi == (int)mask ? 'h' : '-');
			g12a_uart_puts("\n");
		}
}

/*
 * The refinement pass, which is also the control on the lockstep map.
 * With every lane at its OWN converged value, one lane at a time is
 * walked +-G12A_P2_REFINE_SPAN and judged on its own DQ bit.  If the
 * lockstep windows were distorted by mutual de-train, the window
 * measured here will not agree with the one measured there — and the
 * disagreement is the measurement, not an error to be smoothed away.
 */
static void g12a_p2_refine(const char *tag, u32 mask,
			   void (*one)(u32, u32, int))
{
	u32 d, l;

	for (d = 0; d < 4; d++)
		for (l = 0; l < 8; l++) {
			int bit = g12a_p2_dqbit(d, l);
			int c = (int)g12a_p2_open[d][l];
			int k, lo = c, hi = c;

			for (k = 1; k <= G12A_P2_REFINE_SPAN; k++) {
				int v = c - k;

				if (v < 0)
					break;
				one(d, l, v);
				g12a_mrg_probe_fast();
				if (bit >= 0 && ((g12a_mrg_xor >> bit) & 1u))
					break;
				lo = v;
			}
			for (k = 1; k <= G12A_P2_REFINE_SPAN; k++) {
				int v = c + k;

				if (v > (int)mask)
					break;
				one(d, l, v);
				g12a_mrg_probe_fast();
				if (bit >= 0 && ((g12a_mrg_xor >> bit) & 1u))
					break;
				hi = v;
			}
			one(d, l, c);

			g12a_uart_puts(tag);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)d);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)l);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)c);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)lo);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)hi);
			g12a_uart_puts("\n");
		}
}

/*
 * P2.2c — is the missing RxPBDly lower edge a strobe-placement artefact?
 *
 * The reference records the per-lane read-per-bit low edge as BELOW the
 * register floor on all 32 lanes (per-lane-eye-reference.md §4.2), so
 * the eye is one-sided in this parameterisation and its true centre is
 * not observable.  Prediction P2-5 says that is exactly why an open
 * centring must miss the vendor's values.  The cheap test of the
 * explanation is to move the PER-NIBBLE read strobe down and re-run the
 * lockstep sweep: if low edges appear, a co-optimising open algorithm is
 * the route past the wall; if they do not, the vendor's per-lane
 * criterion is not observable from the data bus at all, which is the
 * harder answer and has to be reported as one.
 *
 * 2026-09-06 (row L11): the three offsets above are ALL NEGATIVE, and the
 * table this probe produced proves lowering the strobe moves every lane's
 * UPPER edge down by roughly twice the offset -- i.e. it translates the
 * whole window FURTHER BELOW the register floor, which is the direction
 * that cannot expose a low edge.  The conclusion drawn from them ("no low
 * edge exists at any strobe offset") therefore generalises from three
 * samples on the non-exposing sign.  The positive rungs below walk the
 * other sign to the read-strobe knob's own measured ceiling (dram-margin.md
 * section 4: last passing +8, first failing +9); +9 probes past that bound
 * deliberately, and it is the must-fire arm rather than a rung expected to
 * pass.  The negatives are kept, first and in the same boot, because they
 * are what makes the positive arm's reading a DIFFERENCE.
 */
static void g12a_p2_strobe_probe(void)
{
	static const int offs[] = { -4, -8, -16, 2, 4, 6, 8, 9 };
	u32 i, d, n;

	/*
	 * The trained strobe every offset below is relative to.  Without it a
	 * clamp at the 6-bit rail -- g12a_mrg_rxclk_set() saturates at 0x3F --
	 * reads exactly like a landed write, and a rung that received less
	 * lift than it asked for would be scored as if it had received all of
	 * it.  One representative CSR per DBYTE and nibble, Tg0, which is the
	 * same read-back shape the MGS sweeps already use.
	 */
	g12a_uart_puts("P2Xt");
	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++) {
			g12a_uart_putc(' ');
			g12a_uart_hex16(g12a_phy_read16(G12A_MRG_PA(
				G12A_MRG_DB(d, n, G12A_MRG_RXCLKDLY_TG0))));
		}
	g12a_uart_puts("\n");

	for (i = 0; i < ARRAY_SIZE(offs); i++) {
		u32 bad;

		g12a_uart_puts("P2X off ");
		g12a_uart_putc(offs[i] < 0 ? '-' : '+');
		g12a_uart_hex8((u8)(offs[i] < 0 ? -offs[i] : offs[i]));
		g12a_uart_puts("\n");
		g12a_mrg_rxclk_set(offs[i]);
		/*
		 * Liveness, both arms on one line, printed BEFORE the sweep:
		 * the eight read-backs say the write landed (and by how much,
		 * after the clamp), and `bad`/`xor` are the CPU probe at the
		 * VENDOR's trained RxPBDly with only the strobe moved -- which
		 * must be NON-ZERO at +9, or the strobe is reaching the CSR
		 * and not the data path.
		 */
		g12a_uart_puts("P2Xk");
		for (d = 0; d < 4; d++)
			for (n = 0; n < 2; n++) {
				g12a_uart_putc(' ');
				g12a_uart_hex16(g12a_phy_read16(G12A_MRG_PA(
					G12A_MRG_DB(d, n,
					G12A_MRG_RXCLKDLY_TG0))));
			}
		bad = g12a_mrg_probe_fast();
		g12a_uart_putc(' ');
		g12a_uart_hex32(bad);
		g12a_uart_putc(' ');
		g12a_uart_hex32(g12a_mrg_xor);
		g12a_uart_puts("\n");
		g12a_p2_sweep("P2Xp", G12A_P2_RXPB_MASK, g12a_p2_rxpb_all);
		g12a_mrg_rxclk_restore();
		g12a_mrg_rxpb_restore();
		g12a_uart_puts("P2X back ");
		g12a_uart_hex32(g12a_mrg_probe_fast());
		g12a_uart_puts("\n");
	}
}

/*
 * ---------------------------------------------------------------------
 * G1 -- the READ GATE.  RxEnDlyTg0/Tg1, per NIBBLE, one nibble at a time.
 * amlogic/software/boot/g12/rxen-dtsm-boot-plan.md sections 2.2, 5.1, 8.
 *
 * Every other delay axis in this file has been swept; the gate has not, on
 * any boot of this arc.  It is the only axis whose codec is genuinely wide:
 * the two Tx classes are 9 bits and RxClkDly/RxPBDly are linear in their low
 * bits, but RxEnDly is 11 with a RESERVED bit inside the field, so a raw +-1
 * walk sets bit 5 and moves the gate BACKWARDS by ~1 UI at every 32-step
 * boundary.  Stepping is therefore in DECODED units through
 * g12a_mrg_dec4()/enc4(), exactly as the two Tx classes already do, and the
 * read-back raw is printed at every step so a wrong step is visible in the
 * log without analysis: the correct sequence steps 0x31E 0x31F 0x340, a +33
 * jump, and no printed raw ever has bit 5 set.
 *
 * Both timing groups of the swept nibble move together, by the same DELTA
 * from each one's own trained value -- the same shape g12a_mrg_rxclk_set()
 * uses for the DQS pair.  Stepping only Tg0 would measure nothing whenever
 * the live group is not 0, and the live group is chosen by a CS->destination
 * map register (MASTER.DfiRdDataCsDestMap) rather than being fixed.
 */
static u32 g12a_mrg_rxen_bit5;
static u32 g12a_mrg_rxen_xor;

static void g12a_mrg_rxen_nib_step(u32 d, u32 n, int delta)
{
	u32 t;

	for (t = 0; t < 2; t++) {
		int v = (int)g12a_mrg_dec4(g12a_mrg_rxen[d][n][t]) + delta;

		if (v < 0)
			v = 0;
		if (v > G12A_MRG_RXEN_MAX)
			v = G12A_MRG_RXEN_MAX;
		g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
				 G12A_MRG_RXENDLY_TG0 + t)),
				 g12a_mrg_enc4((u32)v));
	}
}

static void g12a_mrg_rxen_nib_restore(u32 d, u32 n)
{
	u32 t;

	for (t = 0; t < 2; t++)
		g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
				 G12A_MRG_RXENDLY_TG0 + t)),
				 g12a_mrg_rxen[d][n][t]);
}

/*
 * One step: the read-back RAW is the first field after the offset, because
 * section 2.2's abort test is a property of the raw and nothing else.  'B'
 * marks a raw with PUB reserved bit 5 set -- a landed static result says
 * enc4() produces none in 1024 codes, so one here contradicts it (section 8).
 */
static void g12a_mrg_rxen_step(u32 d, u32 n, int delta, u32 bad)
{
	u16 rb = g12a_phy_read16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						       G12A_MRG_RXENDLY_TG0)));

	if (rb & G12A_MRG_RXEN_RSVD5)
		g12a_mrg_rxen_bit5++;
	g12a_mrg_rxen_xor |= g12a_mrg_xor;

	g12a_uart_puts("MGEs ");
	g12a_uart_hex8((u8)d);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)n);
	g12a_uart_putc(' ');
	g12a_uart_putc(delta < 0 ? '-' : '+');
	g12a_uart_hex8((u8)(delta < 0 ? -delta : delta));
	g12a_uart_putc(' ');
	g12a_uart_hex16(rb);
	g12a_uart_putc(' ');
	g12a_uart_hex16((u16)g12a_mrg_dec4(rb));
	g12a_uart_putc(' ');
	g12a_uart_hex32(bad);
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_putc(' ');
	g12a_uart_putc((rb & G12A_MRG_RXEN_RSVD5) ? 'B' : '.');
	g12a_uart_puts("\n");
}

/*
 * One direction of one nibble: step until the probe fails, then restore.
 * A run that reaches its end without failing is reported as a LIMIT ('L'),
 * not as an edge -- the distinction section 4 of dram-margin.md had to make
 * for every other axis.  With G12A_MRG_RXEN_SWEEP_MAX = 255 below the smallest
 * measured trained headroom (399), that end is the SWEEP BOUND on every nibble
 * in both directions; the register-end break is kept as a guard and is not the
 * limit any arm is expected to hit.
 */
static int g12a_mrg_rxen_arm(u32 d, u32 n, int dir)
{
	int base = (int)g12a_mrg_dec4(g12a_mrg_rxen[d][n][0]);
	int k, last = 0, limited = 1;
	u32 bad = 0;

	for (k = 1; k <= G12A_MRG_RXEN_SWEEP_MAX; k++) {
		int v = base + dir * k;

		if (v < 0 || v > G12A_MRG_RXEN_MAX)
			break;
		g12a_mrg_rxen_nib_step(d, n, dir * k);
		bad = g12a_mrg_probe_cur();
		g12a_mrg_rxen_step(d, n, dir * k, bad);
		if (bad) {
			limited = 0;
			break;
		}
		last = k;
	}
	g12a_mrg_rxen_nib_restore(d, n);

	g12a_uart_puts("MGEe ");
	g12a_uart_hex8((u8)d);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)n);
	g12a_uart_putc(' ');
	g12a_uart_putc(dir < 0 ? '-' : '+');
	g12a_uart_hex8((u8)last);
	g12a_uart_putc(' ');
	g12a_uart_hex32(bad);
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_putc(' ');
	g12a_uart_putc(limited ? 'L' : 'F');
	g12a_uart_puts("\n");
	return last;
}

static void g12a_mrg_rxen_axis(void)
{
	u32 d, n;

	g12a_uart_puts("MGE begin\n");
	g12a_mrg_rxen_bit5 = 0;

	if (g12a_mrg_probe_cur()) {
		g12a_uart_puts("MGE abort: centre fails\n");
		return;
	}

	for (d = 0; d < 4; d++) {
		for (n = 0; n < 2; n++) {
			int hi, lo;
			u32 bad;

			g12a_mrg_rxen_xor = 0;

			/*
			 * The vendor's own trained point, printed before
			 * anything moves: this is the reference G1 measures
			 * its window centre against (section 5.1.1).
			 */
			g12a_uart_puts("MGEt ");
			g12a_uart_hex8((u8)d);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)n);
			g12a_uart_putc(' ');
			g12a_uart_hex16(g12a_mrg_rxen[d][n][0]);
			g12a_uart_putc(' ');
			g12a_uart_hex16((u16)g12a_mrg_dec4(g12a_mrg_rxen[d][n][0]));
			g12a_uart_putc(' ');
			g12a_uart_hex16(g12a_mrg_rxen[d][n][1]);
			g12a_uart_putc(' ');
			g12a_uart_hex16((u16)g12a_mrg_dec4(g12a_mrg_rxen[d][n][1]));
			g12a_uart_puts("\n");

			hi = g12a_mrg_rxen_arm(d, n, +1);
			lo = g12a_mrg_rxen_arm(d, n, -1);

			/* <dbyte> <nib> <trained> <lo> <hi> <xor> */
			g12a_uart_puts("MGEw ");
			g12a_uart_hex8((u8)d);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)n);
			g12a_uart_putc(' ');
			g12a_uart_hex16((u16)g12a_mrg_dec4(g12a_mrg_rxen[d][n][0]));
			g12a_uart_puts(" -");
			g12a_uart_hex8((u8)lo);
			g12a_uart_puts(" +");
			g12a_uart_hex8((u8)hi);
			g12a_uart_putc(' ');
			g12a_uart_hex32(g12a_mrg_rxen_xor);
			g12a_uart_puts("\n");

			/*
			 * Restore-and-re-probe, per nibble.  A nibble left
			 * broken by its own arm would poison every nibble
			 * after it and read as a narrow eye rather than as
			 * the bug it is, so this STOPS rather than continuing
			 * on a bus that is no longer trained (section 8).
			 */
			bad = g12a_mrg_probe_cur();
			if (bad) {
				g12a_uart_puts("MGEz ");
				g12a_uart_hex8((u8)d);
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)n);
				g12a_uart_putc(' ');
				g12a_uart_hex32(bad);
				g12a_uart_puts("\nMGE abort: restore\n");
				return;
			}
		}
	}

	g12a_uart_puts("MGE bit5 ");
	g12a_uart_hex32(g12a_mrg_rxen_bit5);
	g12a_uart_puts("\nMGE end ");
	g12a_uart_hex32(g12a_mrg_probe_cur());
	g12a_uart_puts("\n");
}

/*
 * ---------------------------------------------------------------------
 * The DTSM arm -- rxen-dtsm-boot-plan.md sections 4, 5.3, 6, 8.
 *
 * SECOND IMAGE, deliberately.  This is the only NEW instrument in the set:
 * it touches TsmByte3, which also carries the receiver mux controls, and
 * this arc has no measured restore-and-verify for a receiver mux.  Putting
 * it in the boot that carries G1's validated result would risk all of it, so
 * it is compiled in only when G12A_MRG_DTSM is defined to 1 out of tree --
 * the same shape G12A_MRG_T19 / G12A_MRG_P2 / G12A_OPEN_BLOB_T7 already use.
 *
 * TWO SUB-ARMS, ONE IMAGE.  They differ in REGISTER VALUES, not code paths,
 * so they cost no extra boot:
 *
 *   A  TrainingIncDecDtsmEn (0x62) and TrainingParam.TrainEnRxPBD CLEAR for
 *      the swept lane.  The axis holds still, so DtsmErrCnt across codes is
 *      a window measurement (D-1..D-4).  Its RxPBDly read-back is an
 *      INSTRUMENT SELF-CHECK that must read equal; a mismatch here is D-5",
 *      an arm defect and an abort, never a D-5' result.
 *   B  0x62 SET for the swept lane, with the vendor's own measured RxEn
 *      recipe (IncDecRate 3, RollIntoCoarse) copied onto RxPBD.  Asks only
 *      whether the engine WRITES the delay CSR (D-5 / D-5').  It forfeits
 *      its own DtsmErrCnt readings by design -- while the engine is armed
 *      the code is unknown, so an error count is uninterpretable.
 *
 * 0x62 is NOT the lane selector and NOT the error-count enable; the
 * selectors are DbyteAllDtsmCtrl0.DtsmInhibDtsm (0xB9) and
 * TSM3.DtsmClrErrCntMsk (0xB5), and counting is gated by TSM0.DtsmEnb.
 * DtsmSoeLaneMode is two bits and cannot name nine lanes at all.
 */
#ifndef G12A_MRG_DTSM
#define G12A_MRG_DTSM		0
#endif

#if G12A_MRG_DTSM

/* DBYTE-wide CSR of the swept DBYTE (the lane field is 0 for these). */
#define G12A_MRG_DTSM_CSR(off)	G12A_MRG_DB(G12A_MRG_DTSM_D, 0, (off))
/* Per-lane replicated CSR of the swept lane. */
#define G12A_MRG_DTSM_LCSR(off)	G12A_MRG_DB(G12A_MRG_DTSM_D, \
					    G12A_MRG_DTSM_L, (off))
#define G12A_MRG_DTSM_LBIT	(1u << G12A_MRG_DTSM_L)

static struct {
	u16 param;
	u16 byte3;
	u16 incdec;
	u16 inhib;
	u16 tsm3;
	u16 tsm0;
	u16 rxpb;
} g12a_mrg_dtsm_sv;

static u16 g12a_mrg_dtsm_r(u32 csr)
{
	return g12a_phy_read16(G12A_MRG_PA(csr));
}

static void g12a_mrg_dtsm_w(u32 csr, u16 v)
{
	g12a_phy_write16(G12A_MRG_PA(csr), v);
}

/*
 * Every register the arm touches, printed as one line.  Called before the
 * arm, inside each sub-arm, and after the restore, so a wedge (D-4) leaves a
 * readable post-mortem and the restore is checkable rather than assumed.
 * Order: TrainingParam IncDecDtsmEn TsmByte3 DtsmInhib TSM3 TSM0 DtsmBlank.
 */
static void g12a_mrg_dtsm_cfg(const char *tag)
{
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TRAININGPARAM)));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_INCDECDTSMEN)));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TSMBYTE3)));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_DTSMINHIB)));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TSM3)));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_mrg_dtsm_r(G12A_MRG_DTSM_LCSR(G12A_MRG_TSM0)));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_DTSMBLANK)));
	g12a_uart_puts("\n");
}

static void g12a_mrg_dtsm_save(void)
{
	g12a_mrg_dtsm_sv.param =
		g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TRAININGPARAM));
	g12a_mrg_dtsm_sv.byte3 =
		g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TSMBYTE3));
	g12a_mrg_dtsm_sv.incdec =
		g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_INCDECDTSMEN));
	g12a_mrg_dtsm_sv.inhib =
		g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_DTSMINHIB));
	g12a_mrg_dtsm_sv.tsm3 =
		g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TSM3));
	g12a_mrg_dtsm_sv.tsm0 =
		g12a_mrg_dtsm_r(G12A_MRG_DTSM_LCSR(G12A_MRG_TSM0));
	g12a_mrg_dtsm_sv.rxpb = g12a_mrg_rxpb[G12A_MRG_DTSM_D][G12A_MRG_DTSM_L];
}

static void g12a_mrg_dtsm_restore(void)
{
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0),
			g12a_mrg_dtsm_sv.rxpb);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_LCSR(G12A_MRG_TSM0),
			g12a_mrg_dtsm_sv.tsm0);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_TSM3),
			g12a_mrg_dtsm_sv.tsm3);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_DTSMINHIB),
			g12a_mrg_dtsm_sv.inhib);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_INCDECDTSMEN),
			g12a_mrg_dtsm_sv.incdec);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_TSMBYTE3),
			g12a_mrg_dtsm_sv.byte3);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_TRAININGPARAM),
			g12a_mrg_dtsm_sv.param);
}

/*
 * @engine: 0 = sub-arm A, 1 = sub-arm B.  Everything except the engine
 * enable and its TrainingParam companion is identical between the two, which
 * is exactly why they share an image.
 */
static void g12a_mrg_dtsm_setup(int engine)
{
	u16 v;

	/*
	 * Static comparator: the expected value is a register constant, so no
	 * pattern generator is needed and PPGC stays out of csrtab.inc
	 * (section 3.1).  Read-modify-write of ONLY bits 7 and 8 -- the
	 * receiver mux bits in this same register are left as training left
	 * them, because there is no measured restore for them.
	 */
	v = g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TSMBYTE3));
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_TSMBYTE3),
			(u16)(v | G12A_MRG_B3_STATICCMPR |
			      G12A_MRG_B3_STATICCMPRVAL));

	/* Watch the swept lane ONLY.  DtsmInhibDtsm is an INHIBIT mask, so
	 * the swept lane's bit is the one that must be CLEAR. */
	v = g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_DTSMINHIB));
	v = (u16)((v & (u16)~G12A_MRG_LANE_MSK) |
		  (u16)(G12A_MRG_LANE_MSK & ~G12A_MRG_DTSM_LBIT));
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_DTSMINHIB), v);

	/* The independent variable of the two sub-arms. */
	v = g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_INCDECDTSMEN));
	if (engine)
		v = (u16)(v | G12A_MRG_DTSM_LBIT);
	else
		v = (u16)(v & ~G12A_MRG_DTSM_LBIT);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_INCDECDTSMEN), v);

	/* TrainEnRxPBD tracks the engine.  Sub-arm B additionally copies the
	 * vendor's own measured arming recipe for RxEn -- TrainingParam
	 * 0x6209 carries RollIntoCoarse and IncDecRate = 3 -- onto RxPBD,
	 * which is the closest thing to a known-good arming we hold. */
	v = g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_TRAININGPARAM));
	if (engine)
		v = (u16)(((v | G12A_MRG_TP_TRAINENRXPBD |
			    G12A_MRG_TP_ROLLINTOCOARSE) &
			   (u16)~G12A_MRG_TP_INCDECRATE) |
			  G12A_MRG_TP_INCDECRATE_3);
	else
		v = (u16)(v & ~G12A_MRG_TP_TRAINENRXPBD);
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_TRAININGPARAM), v);

	/* Error counting is gated HERE, not by 0x62. */
	v = g12a_mrg_dtsm_r(G12A_MRG_DTSM_LCSR(G12A_MRG_TSM0));
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_LCSR(G12A_MRG_TSM0),
			(u16)(v | G12A_MRG_TSM0_ENB));
}

/* Mask-driven clear from the lane-0 copy; see the section 4 table note. */
static void g12a_mrg_dtsm_clr(void)
{
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_TSM3),
			(u16)(G12A_MRG_DTSM_LBIT | G12A_MRG_TSM3_CLRERRCNT));
	g12a_mrg_dtsm_w(G12A_MRG_DTSM_CSR(G12A_MRG_TSM3),
			(u16)G12A_MRG_DTSM_LBIT);
}

/*
 * The codes swept, in RxPBDly raw units (class 5 is linear in [5:0]).  Nine
 * codes spanning the whole register: the acceptance item asks for at least
 * three including a known-good and a known-bad, and the measured per-lane
 * window on this axis is [0, hi] with hi well inside the range, so the top
 * of this list is known-bad by a landed measurement rather than by hope.
 */
static const u8 g12a_mrg_dtsm_code[] = { 0, 8, 16, 24, 32, 40, 48, 56, 63 };

/*
 * Returns 1 if the engine bit read back as the sub-arm requires.  Without
 * this line neither sub-arm carries information: an equal read-back from a
 * disabled engine is the exact non-result the plan was corrected for.
 */
static int g12a_mrg_dtsm_engine_check(const char *tag, int want)
{
	u16 v = g12a_mrg_dtsm_r(G12A_MRG_DTSM_CSR(G12A_MRG_INCDECDTSMEN));
	int got = (v & G12A_MRG_DTSM_LBIT) ? 1 : 0;

	g12a_uart_puts(tag);
	g12a_uart_puts(" en ");
	g12a_uart_hex16(v);
	g12a_uart_putc(' ');
	g12a_uart_putc((char)('0' + got));
	g12a_uart_putc(' ');
	g12a_uart_putc((char)('0' + want));
	g12a_uart_puts("\n");
	return got == want;
}

/* Sub-arm A -- the window sweep.  D-1..D-4. */
static void g12a_mrg_dtsm_sub_a(void)
{
	u16 keep = (u16)(g12a_mrg_dtsm_sv.rxpb & (u16)~0x3Fu);
	u32 i;

	g12a_uart_puts("MGDa begin\n");
	g12a_mrg_dtsm_setup(0);
	g12a_mrg_dtsm_cfg("MGDa cfg");
	if (!g12a_mrg_dtsm_engine_check("MGDa", 0)) {
		g12a_uart_puts("MGDa abort: engine set\n");
		return;
	}

	for (i = 0; i < sizeof(g12a_mrg_dtsm_code); i++) {
		u16 want = (u16)(keep | g12a_mrg_dtsm_code[i]);
		u16 rb, cnt;
		u32 bad;

		g12a_mrg_dtsm_clr();
		g12a_mrg_dtsm_w(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0), want);
		bad = g12a_mrg_probe_cur();
		rb = g12a_mrg_dtsm_r(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0));
		cnt = g12a_mrg_dtsm_r(G12A_MRG_DTSM_LCSR(G12A_MRG_TSM1));

		/* <code> <written> <readback> <cpu bad> <cpu xor> <errcnt>
		 * <S|X>.  'X' is D-5": the engine is CLEAR in this sub-arm, so
		 * a moving register is an ARM DEFECT, not a D-5' result. */
		g12a_uart_puts("MGDa ");
		g12a_uart_hex8(g12a_mrg_dtsm_code[i]);
		g12a_uart_putc(' ');
		g12a_uart_hex16(want);
		g12a_uart_putc(' ');
		g12a_uart_hex16(rb);
		g12a_uart_putc(' ');
		g12a_uart_hex32(bad);
		g12a_uart_putc(' ');
		g12a_uart_hex32(g12a_mrg_xor);
		g12a_uart_putc(' ');
		g12a_uart_hex16(cnt);
		g12a_uart_putc(' ');
		g12a_uart_putc(rb == want ? 'S' : 'X');
		g12a_uart_puts("\n");
		if (rb != want) {
			g12a_uart_puts("MGDa abort: self-check\n");
			break;
		}
	}

	g12a_mrg_dtsm_w(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0),
			g12a_mrg_dtsm_sv.rxpb);
	g12a_uart_puts("MGDa end ");
	g12a_uart_hex32(g12a_mrg_probe_cur());
	g12a_uart_puts("\n");
}

/*
 * Sub-arm B -- the engine probe.  D-5 / D-5'.  Runs LAST, after sub-arm A
 * has finished and printed: it arms an engine that may walk the axis, and
 * running it first would contaminate A's window and cost the boot's only
 * interpretable window data.
 */
static void g12a_mrg_dtsm_sub_b(void)
{
	u16 keep = (u16)(g12a_mrg_dtsm_sv.rxpb & (u16)~0x3Fu);
	u32 i;

	g12a_uart_puts("MGDb begin\n");
	g12a_mrg_dtsm_setup(1);
	g12a_mrg_dtsm_cfg("MGDb cfg");
	if (!g12a_mrg_dtsm_engine_check("MGDb", 1)) {
		g12a_uart_puts("MGDb abort: engine not set\n");
		return;
	}

	for (i = 0; i < sizeof(g12a_mrg_dtsm_code); i++) {
		u16 want = (u16)(keep | g12a_mrg_dtsm_code[i]);
		u16 rb0, rb1;
		u32 bad;

		g12a_mrg_dtsm_w(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0), want);
		rb0 = g12a_mrg_dtsm_r(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0));
		/* The engine needs DRAM traffic to act on, so the second
		 * read-back is taken AFTER a probe, not instead of one. */
		bad = g12a_mrg_probe_cur();
		rb1 = g12a_mrg_dtsm_r(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0));

		/* <code> <written> <rb immediate> <rb post-traffic> <cpu bad>
		 * <EQ|NE>.  No DtsmErrCnt: while the engine is armed the code
		 * is unknown, so an error count here is uninterpretable and is
		 * deliberately not printed at all. */
		g12a_uart_puts("MGDb ");
		g12a_uart_hex8(g12a_mrg_dtsm_code[i]);
		g12a_uart_putc(' ');
		g12a_uart_hex16(want);
		g12a_uart_putc(' ');
		g12a_uart_hex16(rb0);
		g12a_uart_putc(' ');
		g12a_uart_hex16(rb1);
		g12a_uart_putc(' ');
		g12a_uart_hex32(bad);
		g12a_uart_putc(' ');
		g12a_uart_puts((rb0 == want && rb1 == want) ? "EQ" : "NE");
		g12a_uart_puts("\n");
	}

	g12a_mrg_dtsm_w(G12A_MRG_DTSM_LCSR(G12A_MRG_RXPBDLY_TG0),
			g12a_mrg_dtsm_sv.rxpb);
	g12a_uart_puts("MGDb end ");
	g12a_uart_hex32(g12a_mrg_probe_cur());
	g12a_uart_puts("\n");
}

static void g12a_mrg_dtsm(void)
{
	g12a_uart_puts("MGD begin ");
	g12a_uart_hex8((u8)G12A_MRG_DTSM_D);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)G12A_MRG_DTSM_L);
	g12a_uart_puts("\n");

	g12a_mrg_dtsm_save();
	g12a_mrg_dtsm_cfg("MGD pre");
	g12a_mrg_dtsm_sub_a();
	g12a_mrg_dtsm_sub_b();
	g12a_mrg_dtsm_restore();
	g12a_mrg_dtsm_cfg("MGD post");

	g12a_uart_puts("MGD end ");
	g12a_uart_hex32(g12a_mrg_probe_cur());
	g12a_uart_puts("\n");
}
#endif /* G12A_MRG_DTSM */

/*
 * ---------------------------------------------------------------------
 * G2 -- OPEN PER-NIBBLE TxDqsDly CENTRING.  TxDqsDlyTg0/Tg1, from a
 * DE-TRAINED seed, converged per nibble and refined per nibble.
 * amlogic/software/boot/g12/rxen-dtsm-boot-plan.md sections 1.4, 5.2, 8.
 *
 * WHAT THIS IS NOT.  g12a_mrg_txdqs_set() / the "MGSq" sweep applies ONE
 * COMMON delta to all 16 instances relative to each one's TRAINED value and
 * walks to the first AGGREGATE failure.  That is a headroom instrument on the
 * vendor's state.  This is a different instrument: no instance keeps a trained
 * value, each nibble is judged on its OWN four DQ bits, and the estimator is
 * the centre of that nibble's longest clean run.  Neither can substitute for
 * the other and this arm does not touch MGSq.
 *
 * MODELLED ON g12a_p2_sweep/converge/refine (the RxPBDly and VrefDAC passes
 * above), which is the per-instance converge/refine family this file already
 * has, NOT on g12a_mrg_sweep.  Three differences, all forced by the axis:
 *
 *   1. The instance is a NIBBLE, not a lane, so the oracle is the OR of that
 *      nibble's four DQ bits in g12a_mrg_xor rather than one bit.  The DWC
 *      DBYTE nibble field is the same [11:8] field G12A_MRG_DB() already
 *      writes for RxEnDly/RxClkDly, and nibble n covers dbyte lanes 4n..4n+3;
 *      the composed mask is PRINTED per instance so the assumption is
 *      checkable off-board rather than assumed here.
 *   2. The class-2 codec is the coarse/fine one, so every step goes through
 *      g12a_mrg_dec4()/enc4() exactly as g12a_mrg_txdqs_set() does.  The
 *      swept quantity is the DECODED code.
 *   3. Both timing groups of a nibble move TOGETHER to the same absolute
 *      code -- the same reason g12a_mrg_rxen_nib_step() gives: the live group
 *      is selected by a CS->destination map, so moving one alone measures
 *      nothing whenever the live group is the other.  So there are 16
 *      REGISTERS but 8 observable WINDOWS.  The Delta-to-vendor is still 16
 *      numbers, one per register, because Tg0 and Tg1 have different trained
 *      values; the window columns simply repeat across a nibble's two rows.
 *      Section 5.2 says "16 instances"; what is measurable is 8 windows and 16
 *      deltas, and that is what is printed.
 *
 * ORDER.  Section 1.4 makes this item 4 and G1 item 3: G1 needs seven nibbles
 * TRAINED while the eighth moves, and this arm de-trains all of them.  The
 * call site is therefore after g12a_mrg_rxen_axis() and it restores and
 * re-probes at the end (section 8's per-arm requirement).
 *
 * RISK, stated rather than assumed.  The full 0..255 lockstep sweep drives the
 * strobe far outside any working window, so for most codes the bus is
 * expected to be broadly wrong (the reference records the first failure at
 * +5 from trained as 261116/262144 words bad).  The probe reads and compares;
 * it does not execute from DRAM.  The extremes of THIS axis have never been
 * run: "MGXq" exists but shares MGSq's unreachability, so this arm is the
 * first thing to slam TxDqsDly, and a hang here is a result about the axis,
 * not a defect in the estimator.  G1 has already printed by then.
 */
#ifndef G12A_MRG_G2
#define G12A_MRG_G2		0
#endif

#if G12A_MRG_G2 || G12A_MRG_P6

/*
 * G2's OWN bound, deliberately NOT the shared G12A_MRG_SWEEP_MAX (48), which
 * 13 call sites read: widening that would silently move Vref, RxClkDly,
 * RxPBDly and both Tx axes.  255 is the whole DECODED domain of the 9-bit
 * class-2 field (coarse [8:6] x 32 + fine [4:0]), which is what a de-trained
 * seed requires -- a bound smaller than the domain would presuppose where the
 * window is, and section 5.2's whole question is whether we can find it
 * without being told.
 */
#define G12A_G2_MAX		255
#define G12A_G2_REFINE_SPAN	8

/* Converged decoded centre, window bounds, run length, run COUNT, per nibble. */
static u8 g12a_g2_open[4][2];
static u8 g12a_g2_blo[4][2];
static u8 g12a_g2_bhi[4][2];
static u8 g12a_g2_best[4][2];
static u8 g12a_g2_runs[4][2];
/* 1 = do not print the per-code "G2Sq" lines (the product centring pass). */
static int g12a_g2_quiet;
/*
 * The decoded codes the sweep visits, [g12a_g2_vlo, g12a_g2_vhi].  The whole
 * domain unless the product pass narrows it to the trained values +- a span
 * (CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE_SPAN).
 */
static int g12a_g2_vlo;
static int g12a_g2_vhi = G12A_G2_MAX;

/*
 * The nibble oracle.  g12a_p2_dqbit() is the validated lane->DQ-bit inverse
 * the per-lane pass already trusts (64 of 64); a nibble is four consecutive
 * lanes of its DBYTE.  A lane with no DQ bit contributes nothing, and a mask
 * of 0 is printed as such -- an instance with no oracle has no window and is
 * reported, never silently centred.
 */
static u32 g12a_g2_nibmask(u32 d, u32 n)
{
	u32 m = 0;
	u32 i;

	for (i = 0; i < 4; i++) {
		int b = g12a_p2_dqbit(d, n * 4u + i);

		if (b >= 0)
			m |= 1u << b;
	}
	return m;
}

/* Drive ONE nibble's two timing groups to the same absolute decoded code. */
static void g12a_g2_one(u32 d, u32 n, int v)
{
	u32 t;

	for (t = 0; t < 2; t++)
		g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
				 G12A_MRG_TXDQSDLY_TG0 + t)),
				 g12a_mrg_enc4((u32)v));
}

/* The de-trained seed: every one of the 16 instances at the same code. */
static void g12a_g2_all(int v)
{
	u32 d, n;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			g12a_g2_one(d, n, v);
}

static void g12a_g2_open_apply(void)
{
	u32 d, n;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			g12a_g2_one(d, n, (int)g12a_g2_open[d][n]);
}

/*
 * The lockstep sweep, and the window reduction folded into it.
 *
 * g12a_p2_sweep() stores an xor word per code in g12a_p2_map[] and
 * g12a_p2_converge() walks that map afterwards.  Here the map would be 256
 * words of .bss for no gain, because every code's xor is PRINTED anyway and
 * the reduction is a single forward pass: the run state is carried in locals
 * and only the RESULT is kept.  The raw per-code xor on the wire is still the
 * primary record, exactly as "P2Sp" is.
 *
 * Section 5.2's G2-3 asks whether each window is a SINGLE CONTIGUOUS RUN, so
 * the number of distinct clean runs is counted, not just the longest.  A
 * count above 1 is the fragmentation section 8 says to report instead of
 * taking a centre.
 *
 * Format: "G2Sq <code> <xor> <bad>" per code.
 */
static void g12a_g2_sweep(void)
{
	int run[4][2], lo[4][2];
	u32 mask[4][2];
	u32 d, n;
	int v;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++) {
			run[d][n] = 0;
			lo[d][n] = -1;
			mask[d][n] = g12a_g2_nibmask(d, n);
			g12a_g2_open[d][n] = 0;
			g12a_g2_blo[d][n] = 0xFF;
			g12a_g2_bhi[d][n] = 0xFF;
			g12a_g2_best[d][n] = 0;
			g12a_g2_runs[d][n] = 0;
		}

	for (v = g12a_g2_vlo; v <= g12a_g2_vhi; v++) {
		u32 bad;

		g12a_g2_all(v);
		bad = g12a_mrg_probe_fast();

		if (!g12a_g2_quiet) {
			g12a_uart_puts("G2Sq ");
			g12a_uart_hex8((u8)v);
			g12a_uart_putc(' ');
			g12a_uart_hex32(g12a_mrg_xor);
			g12a_uart_putc(' ');
			g12a_uart_hex32(bad);
			g12a_uart_puts("\n");
		}

		for (d = 0; d < 4; d++)
			for (n = 0; n < 2; n++) {
				int clean = mask[d][n] &&
					    !(g12a_mrg_xor & mask[d][n]);

				if (!clean) {
					run[d][n] = 0;
					lo[d][n] = -1;
					continue;
				}
				if (lo[d][n] < 0) {
					lo[d][n] = v;
					if (g12a_g2_runs[d][n] < 0xFF)
						g12a_g2_runs[d][n]++;
				}
				run[d][n]++;
				if (run[d][n] > (int)g12a_g2_best[d][n]) {
					g12a_g2_best[d][n] = (u8)run[d][n];
					g12a_g2_blo[d][n] = (u8)lo[d][n];
					g12a_g2_bhi[d][n] = (u8)v;
				}
			}
	}

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			if (g12a_g2_best[d][n])
				g12a_g2_open[d][n] =
					(u8)(((int)g12a_g2_blo[d][n] +
					      (int)g12a_g2_bhi[d][n]) / 2);
}

/*
 * Per-REGISTER report: 16 rows, because the Delta-to-vendor is per register.
 *
 * Format:
 *   "G2Cq <d> <n> <t> <mask> <trained> <blo> <bhi> <cen> <best> <runs> <sd><delta> <lh>"
 *
 * <trained> is the vendor's own decoded TxDqsDly for THIS (d,n,t), read by
 * g12a_mrg_capture() before anything moved.  <delta> is (cen - trained) with
 * an explicit sign character, which is the quantity section 5.2's G2-1
 * prediction is about; the ~3.5-step figure is NOT computed here and no
 * verdict against it is printed.  <runs> is the contiguity count for G2-3.
 * 'l'/'h' mark a window that touched the register floor/ceiling rather than
 * data, the same distinction g12a_p2_converge() makes.  A row with <best> 00
 * has no window at all and its <cen> reads FF.
 */
static void g12a_g2_report(void)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++) {
				int cen = g12a_g2_best[d][n] ?
					  (int)g12a_g2_open[d][n] : -1;
				int tr = (int)g12a_mrg_dec4(
						g12a_mrg_txdqs[d][n][t]);
				int dl = cen < 0 ? 0 : cen - tr;

				g12a_uart_puts("G2Cq ");
				g12a_uart_hex8((u8)d);
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)n);
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)t);
				g12a_uart_putc(' ');
				g12a_uart_hex32(g12a_g2_nibmask(d, n));
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)tr);
				g12a_uart_putc(' ');
				g12a_uart_hex8(g12a_g2_blo[d][n]);
				g12a_uart_putc(' ');
				g12a_uart_hex8(g12a_g2_bhi[d][n]);
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)(cen < 0 ? 0xFF : cen));
				g12a_uart_putc(' ');
				g12a_uart_hex8(g12a_g2_best[d][n]);
				g12a_uart_putc(' ');
				g12a_uart_hex8(g12a_g2_runs[d][n]);
				g12a_uart_putc(' ');
				g12a_uart_putc(cen < 0 ? '?' :
					       (dl < 0 ? '-' : '+'));
				g12a_uart_hex8((u8)(dl < 0 ? -dl : dl));
				g12a_uart_putc(' ');
				g12a_uart_putc(cen >= 0 &&
					       g12a_g2_blo[d][n] == 0 ?
					       'l' : '-');
				g12a_uart_putc(cen >= 0 &&
					       g12a_g2_bhi[d][n] ==
					       G12A_G2_MAX ? 'h' : '-');
				g12a_uart_puts("\n");
			}
}

/*
 * The refinement pass, and the control on the lockstep map -- the same role
 * g12a_p2_refine() plays for RxPBDly.  With every nibble at its OWN converged
 * value, one nibble at a time is walked +-G12A_G2_REFINE_SPAN and judged on
 * its own four DQ bits.  If the de-trained lockstep windows were distorted by
 * the other nibbles also being out of position, the window measured here will
 * not agree with the one measured there, and the DISAGREEMENT is the
 * measurement.
 *
 * Format: "G2Rq <d> <n> <c> <lo> <hi>".
 */
static void g12a_g2_refine(void)
{
	u32 d, n;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++) {
			u32 mask = g12a_g2_nibmask(d, n);
			int c = (int)g12a_g2_open[d][n];
			int k, lo = c, hi = c;

			for (k = 1; k <= G12A_G2_REFINE_SPAN; k++) {
				int v = c - k;

				if (v < 0)
					break;
				g12a_g2_one(d, n, v);
				g12a_mrg_probe_fast();
				if (!mask || (g12a_mrg_xor & mask))
					break;
				lo = v;
			}
			for (k = 1; k <= G12A_G2_REFINE_SPAN; k++) {
				int v = c + k;

				if (v > G12A_G2_MAX)
					break;
				g12a_g2_one(d, n, v);
				g12a_mrg_probe_fast();
				if (!mask || (g12a_mrg_xor & mask))
					break;
				hi = v;
			}
			g12a_g2_one(d, n, c);

			g12a_uart_puts("G2Rq ");
			g12a_uart_hex8((u8)d);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)n);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)c);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)lo);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)hi);
			g12a_uart_puts("\n");
		}
}

/*
 * "G2 begin" / "G2 end" bracket the arm so its PRESENCE in an image is
 * decidable from a string literal with a control, not from nm -- a static at
 * -O2 inlines away and leaves no symbol even where it demonstrably ran.
 * "G2 pre" is the trained-state probe taken before anything moves: it is the
 * positive control that the bus was clean going in, so a nonzero "G2 rst"
 * afterwards is attributable to this arm and not to the state it inherited.
 */
static void g12a_g2_axis(void)
{
	u32 bad;

	g12a_uart_puts("G2 begin\n");

	bad = g12a_mrg_probe_fast();
	g12a_uart_puts("G2 pre ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");

	g12a_g2_sweep();
	g12a_g2_report();
	g12a_g2_open_apply();

	g12a_uart_puts("G2Aq ");
	g12a_uart_hex32(g12a_mrg_probe_fast());
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_puts("\n");

	g12a_g2_refine();

	/*
	 * Section 8: an arm whose restore does not re-probe to 0 bad has left
	 * the bus de-trained.  The value is printed rather than acted on --
	 * nothing downstream in this image depends on it, and the operator
	 * needs the number to decide whether the boot is interpretable.
	 */
	g12a_mrg_txdqs_restore();
	bad = g12a_mrg_probe_fast();
	g12a_uart_puts("G2 rst ");
	g12a_uart_hex32(bad);
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_puts("\n");

	g12a_uart_puts("G2 end\n");
}
#endif /* G12A_MRG_G2 || G12A_MRG_P6 */

#if G12A_MRG_P6
/*
 * Format: "P6 pre <bad>", G2's "G2Sq"/"G2Cq" lines, then per nibble
 * "P6Cq <d> <n> <+|-><delta> <tg0 old> <tg0 new> <tg1 old> <tg1 new>" (decoded
 * codes; delta 0 where G2 found no window), "P6 post <bad> <xor>", and
 * "P6 revert" if the applied state probes bad (the vendor values are then
 * put back).  The TxDqsDly snapshot is updated to the applied values, so the
 * shmoo's q axis and its restore are centred on them.
 */
static u16 g12a_p6_orig[4][2][2];

static void g12a_p6_centre(void)
{
	u32 d, n, t, bad;

	g12a_uart_puts("P6 begin\n");
	g12a_mrg_pbase = G12A_MRG_PROBE_RANK0;	/* centre on rank 0, as P6 ran */
	bad = g12a_mrg_probe_fast();
	g12a_uart_puts("P6 pre ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++)
				g12a_p6_orig[d][n][t] = g12a_mrg_txdqs[d][n][t];

	g12a_g2_sweep();
	if (g12a_g2_vlo > 0 || g12a_g2_vhi < G12A_G2_MAX) {
		/*
		 * A narrowed sweep: a window cut by the range, or a nibble with
		 * no window, would bias a centre.  Sweep the whole domain then.
		 */
		int redo = 0;

		for (d = 0; d < 4; d++)
			for (n = 0; n < 2; n++)
				if (!g12a_g2_best[d][n] ||
				    (g12a_g2_vlo > 0 && g12a_g2_blo[d][n] == g12a_g2_vlo) ||
				    (g12a_g2_vhi < G12A_G2_MAX &&
				     g12a_g2_bhi[d][n] == g12a_g2_vhi))
					redo = 1;
		g12a_uart_puts("TXS ");
		g12a_uart_hex8((u8)g12a_g2_vlo);
		g12a_uart_putc(' ');
		g12a_uart_hex8((u8)g12a_g2_vhi);
		g12a_uart_puts(redo ? " full\n" : " ok\n");
		if (redo) {
			g12a_g2_vlo = 0;
			g12a_g2_vhi = G12A_G2_MAX;
			g12a_g2_sweep();
		}
	}
	if (!g12a_g2_quiet)
		g12a_g2_report();

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++) {
			int delta = g12a_g2_best[d][n] ?
				    (int)g12a_g2_open[d][n] -
				    (int)g12a_mrg_dec4(g12a_p6_orig[d][n][0]) : 0;
			int v[2];

			for (t = 0; t < 2; t++) {
				v[t] = (int)g12a_mrg_dec4(g12a_p6_orig[d][n][t]) + delta;
				if (v[t] < 0)
					v[t] = 0;
				if (v[t] > 255)
					v[t] = 255;
				g12a_mrg_txdqs[d][n][t] = (u16)g12a_mrg_enc4((u32)v[t]);
				g12a_phy_write16(G12A_MRG_PA(G12A_MRG_DB(d, n,
						 G12A_MRG_TXDQSDLY_TG0 + t)),
						 g12a_mrg_txdqs[d][n][t]);
			}
			g12a_uart_puts("P6Cq ");
			g12a_uart_hex8((u8)d);
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)n);
			g12a_uart_putc(' ');
			g12a_uart_putc(delta < 0 ? '-' : '+');
			g12a_uart_hex8((u8)(delta < 0 ? -delta : delta));
			for (t = 0; t < 2; t++) {
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)g12a_mrg_dec4(g12a_p6_orig[d][n][t]));
				g12a_uart_putc(' ');
				g12a_uart_hex8((u8)v[t]);
			}
			g12a_uart_puts("\n");
		}

	bad = g12a_mrg_probe_fast();
	g12a_uart_puts("P6 post ");
	g12a_uart_hex32(bad);
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_puts("\n");
	if (bad) {
		for (d = 0; d < 4; d++)
			for (n = 0; n < 2; n++)
				for (t = 0; t < 2; t++)
					g12a_mrg_txdqs[d][n][t] = g12a_p6_orig[d][n][t];
		g12a_mrg_txdqs_restore();
		g12a_uart_puts("P6 revert\n");
	}
	g12a_mrg_pbase = G12A_MRG_PROBE_BASE;	/* the margin axes probe here */
	bad = g12a_mrg_probe_fast();
	g12a_uart_puts("P6 base ");
	g12a_uart_hex32((u32)g12a_mrg_pbase);
	g12a_uart_putc(' ');
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	g12a_uart_puts("P6 end\n");
}
#endif /* G12A_MRG_P6 */

#if G12A_MRG_P2
static void g12a_p2_open_train(void)
{
	u32 bad;

	g12a_uart_puts("P2 begin\n");
	g12a_mrg_fast = 1;

	bad = g12a_mrg_probe_fast();
	g12a_uart_puts("P2 centre ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (bad) {
		g12a_uart_puts("P2 abort: centre fails\n");
		g12a_mrg_fast = 0;
		return;
	}

	/* P2.1 — read-only first, then the no-op re-apply. */
	g12a_p2_pinmux(1);
	g12a_p2_codec_assert();
	g12a_p2_dmc_dump();
	g12a_p2_mid1_compare();

	/* P2.2 — read per-bit delay. */
	g12a_p2_sweep("P2Sp", G12A_P2_RXPB_MASK, g12a_p2_rxpb_all);
	g12a_p2_converge("P2Cp", G12A_P2_RXPB_MASK, &g12a_mrg_rxpb[0][0], 9,
			 G12A_P2_RXPB_MASK);
	g12a_p2_rxpb_open();
	g12a_uart_puts("P2Ap ");
	g12a_uart_hex32(g12a_mrg_probe_fast());
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_puts("\n");
	g12a_p2_refine("P2Rp", G12A_P2_RXPB_MASK, g12a_p2_rxpb_one);
	g12a_mrg_rxpb_restore();
	g12a_uart_puts("P2Zp ");
	g12a_uart_hex32(g12a_mrg_probe_fast());
	g12a_uart_puts("\n");

	/* P2.2 — per-lane RX vref.  Live only because 2D ran. */
	g12a_p2_sweep("P2Sv", G12A_P2_VREF_MASK, g12a_p2_vref_all);
	g12a_p2_converge("P2Cv", G12A_P2_VREF_MASK, &g12a_mrg_vref0[0][0], 9,
			 G12A_P2_VREF_MASK);
	g12a_p2_vref_open();
	g12a_uart_puts("P2Av ");
	g12a_uart_hex32(g12a_mrg_probe_fast());
	g12a_uart_putc(' ');
	g12a_uart_hex32(g12a_mrg_xor);
	g12a_uart_puts("\n");
	g12a_p2_refine("P2Rv", G12A_P2_VREF_MASK, g12a_p2_vref_one);
	g12a_mrg_vref_restore();
	g12a_uart_puts("P2Zv ");
	g12a_uart_hex32(g12a_mrg_probe_fast());
	g12a_uart_puts("\n");

	/* P2.2c — the strobe question, last, because it moves a nibble. */
	g12a_p2_strobe_probe();

	/*
	 * Everything is restored: the chain continues on the VENDOR's
	 * trained state, so the open values are measured and reported and
	 * never shipped.  That is what "default OFF until it meets the
	 * reference" means here.
	 */
	g12a_mrg_rxpb_restore();
	g12a_mrg_vref_restore();
	g12a_mrg_rxclk_restore();

	/*
	 * G1 -- the read gate, last, and only from here: every arm needs the
	 * seven other nibbles TRAINED while the eighth moves, so it must run
	 * after the P2 passes above have put the vendor's state back.  Its
	 * data is readable only if g12a_p2_codec_assert() printed
	 * "P2K bad 00000000" with no P2Ke line earlier in this same boot --
	 * that gate is section 2.1 of the plan and it is not re-checked here.
	 */
	g12a_mrg_rxen_axis();

	/*
	 * G2 -- the open per-nibble TxDqsDly centring, after G1 and only after
	 * it: section 1.4 forces the order, because G2 seeds every nibble from
	 * a DE-TRAINED code and so destroys the trained TxDqsDly state G1's
	 * "seven trained while the eighth moves" shape requires.  Its own
	 * image, default OFF, for that reason.
	 */
#if G12A_MRG_G2
	g12a_g2_axis();
#endif

	/*
	 * The DTSM arm is a SEPARATE IMAGE (section 1.3) and runs last of all
	 * when it is compiled in, so everything upstream has already printed
	 * before an instrument with no measured restore path touches TsmByte3.
	 */
#if G12A_MRG_DTSM
	g12a_mrg_dtsm();
#endif

	g12a_mrg_fast = 0;
	g12a_uart_puts("P2 end ");
	g12a_uart_hex32(g12a_mrg_probe());
	g12a_uart_puts("\n");
}
#endif /* G12A_MRG_P2 */
#endif /* G12A_MRG_P2 || G12A_MRG_P6 */

/*
 * ---------------------------------------------------------------------
 * The DMC BIST engine, from the TPL.
 *
 * host-driven-training-feasibility.md §2.5 calls this the single most
 * important finding for P2: a pattern generator, comparator and error
 * counter inside the DMC at 0xFF639380-0xFF6393D8, programmed and polled
 * entirely by the CPU with no PMU involvement — i.e. the pass/fail
 * oracle an open training loop needs.  It has never been exercised.  It
 * is unreadable from U-Boot proper (0xFF6393xx returns 0 at EL2 after
 * BL31, dram-margin.md §2), so the TPL is the only place to try it.
 *
 * This is a bring-up probe, not a measurement: it programs the engine as
 * fffba6ac does, arms it, triggers it, and polls the done bits with a
 * tick bound.  Every register it writes is saved and restored.  It
 * deliberately does NOT touch 0xFF639368 (which carries the DMC's
 * discovered geometry) or 0xFF638004 (DMC control) — fffba6ac writes
 * those only on its own failure path, and a wrong write there would take
 * DRAM down with it.  It runs LAST for the same reason.
 */
#define G12A_DMC_BIST_ADDR	0xFF639380UL
#define G12A_DMC_BIST_LEN	0xFF639384UL
#define G12A_DMC_BIST_CTRL	0xFF639388UL
#define G12A_DMC_BIST_PAT0	0xFF639390UL
#define G12A_DMC_BIST_MASK	0xFF63937CUL
#define G12A_DMC_BIST_ERRCNT	0xFF6393D4UL
#define G12A_DMC_BIST_STAT	0xFF6393D8UL

static void g12a_mrg_bist_reg(const char *tag, ulong pa)
{
	g12a_uart_puts("MGB ");
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)pa));
	g12a_uart_puts("\n");
}

static void g12a_mrg_bist(void)
{
	static const u32 bpat[8] = {
		0x55AAAA55u, 0xAA5555AAu, 0xCC3333CCu, 0x0000FFFFu,
		0xFFFF0000u, 0x33CCCC33u, 0x00000000u, 0xFFFFFFFFu,
	};
	u32 save[8], sa, sl, sc, sm;
	u64 t0;
	u32 st = 0, i;

	g12a_uart_puts("MGB begin\n");
	g12a_mrg_bist_reg("geo", 0xFF639368UL);
	g12a_mrg_bist_reg("stat0", G12A_DMC_BIST_STAT);
	g12a_mrg_bist_reg("err0", G12A_DMC_BIST_ERRCNT);
	/*
	 * MEASURED on boot t19-a: programmed exactly as the fffba6ac
	 * register table records, the engine never asserts its done bits
	 * (stat stays 00000000).  Whatever the missing precondition is, it
	 * is in registers this probe does not know about, so dump the whole
	 * block either side of the run rather than guessing at a mode bit —
	 * the next engagement should not have to spend a boot to see it.
	 */
	for (i = 0; i < 32; i++) {
		if ((i & 3) == 0) {
			g12a_uart_puts("\nMGB d ");
			g12a_uart_hex32((u32)(0xFF639360UL + i * 4));
			g12a_uart_putc(':');
		}
		g12a_uart_putc(' ');
		g12a_uart_hex32(readl((void *)(0xFF639360UL + i * 4)));
	}
	g12a_uart_puts("\n");

	sa = readl((void *)G12A_DMC_BIST_ADDR);
	sl = readl((void *)G12A_DMC_BIST_LEN);
	sc = readl((void *)G12A_DMC_BIST_CTRL);
	sm = readl((void *)G12A_DMC_BIST_MASK);
	for (i = 0; i < 8; i++)
		save[i] = readl((void *)(G12A_DMC_BIST_PAT0 + i * 4));

	writel((u32)G12A_MRG_PROBE_BASE, (void *)G12A_DMC_BIST_ADDR);
	writel((u32)(G12A_MRG_FAST_WORDS * 4 - 4), (void *)G12A_DMC_BIST_LEN);
	for (i = 0; i < 8; i++)
		writel(bpat[i], (void *)(G12A_DMC_BIST_PAT0 + i * 4));
	writel(0xF000F000u, (void *)G12A_DMC_BIST_MASK);
	writel(0x8000001Fu, (void *)G12A_DMC_BIST_STAT);	/* arm/clear */
	writel(0x81800000u, (void *)G12A_DMC_BIST_CTRL);	/* go */

	t0 = g12a_mrg_ticks();
	while (g12a_mrg_ticks() - t0 < 0x02000000ULL) {
		st = readl((void *)G12A_DMC_BIST_STAT);
		if (st & 0xC0000000u)
			break;
		g12a_wdt_pet_safe();
	}

	g12a_uart_puts("MGB run stat ");
	g12a_uart_hex32(st);
	g12a_uart_puts(" err ");
	g12a_uart_hex32(readl((void *)G12A_DMC_BIST_ERRCNT));
	g12a_uart_puts(" done ");
	g12a_uart_hex8((u8)((st & 0xC0000000u) ? 1 : 0));
	g12a_uart_puts("\n");

	writel(0, (void *)G12A_DMC_BIST_CTRL);			/* stop */
	writel(0x8000001Fu, (void *)G12A_DMC_BIST_STAT);
	writel(sa, (void *)G12A_DMC_BIST_ADDR);
	writel(sl, (void *)G12A_DMC_BIST_LEN);
	writel(sm, (void *)G12A_DMC_BIST_MASK);
	for (i = 0; i < 8; i++)
		writel(save[i], (void *)(G12A_DMC_BIST_PAT0 + i * 4));
	writel(sc, (void *)G12A_DMC_BIST_CTRL);

	for (i = 0; i < 32; i++) {
		if ((i & 3) == 0) {
			g12a_uart_puts("\nMGB e ");
			g12a_uart_hex32((u32)(0xFF639360UL + i * 4));
			g12a_uart_putc(':');
		}
		g12a_uart_putc(' ');
		g12a_uart_hex32(readl((void *)(0xFF639360UL + i * 4)));
	}
	g12a_uart_puts("\n");

	g12a_uart_puts("MGB end ");
	g12a_uart_hex32(g12a_mrg_probe());
	g12a_uart_puts("\n");
}

static void g12a_mrg_shmoo(int extremes, const char *phase)
{
	u32 bad;

	g12a_uart_puts("MGS begin ");
	g12a_uart_puts(phase);
	g12a_uart_puts("\n");

	bad = g12a_mrg_probe_cur();
	g12a_uart_puts("MGS centre ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (bad) {
		g12a_uart_puts("MGS abort: centre fails\n");
		return;
	}

	g12a_mrg_rb_csr = G12A_MRG_DB(0, 0, G12A_MRG_VREFDAC0);
	g12a_mrg_sweep("MGSv", g12a_mrg_vref_set, g12a_mrg_vref_restore,
		       +1, G12A_MRG_SWEEP_MAX);
	g12a_mrg_sweep("MGSv", g12a_mrg_vref_set, g12a_mrg_vref_restore,
		       -1, G12A_MRG_SWEEP_MAX);
	bad = g12a_mrg_probe_cur();
	g12a_uart_puts("MGSv restored ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (extremes)
		g12a_mrg_extreme("MGXv", g12a_mrg_vref_set,
				 g12a_mrg_vref_restore, -127, 127);

	/*
	 * TxDqDly is swept only at its extremes.  MEASURED on boot
	 * margin-2d-v2: driving every lane to decoded 0 and to 95
	 * (readback 0000 / 007f, so the writes land) leaves the probe at
	 * ZERO errors, i.e. the per-DQ-bit transmit deskew is not applied
	 * to the data path in this post-training state.  A step sweep of
	 * it therefore measures nothing — it ran to ±48 without failing,
	 * which would have read as enormous write margin and been wrong.
	 */
	if (extremes) {
		g12a_mrg_rb_csr = G12A_MRG_DB(0, 0, G12A_MRG_TXDQDLY_TG0);
		g12a_mrg_extreme("MGXt", g12a_mrg_txdq_set,
				 g12a_mrg_txdq_restore, -255, 255);
	}

	g12a_mrg_rb_csr = G12A_MRG_DB(0, 0, G12A_MRG_TXDQSDLY_TG0);
	g12a_mrg_sweep("MGSq", g12a_mrg_txdqs_set, g12a_mrg_txdqs_restore,
		       +1, G12A_MRG_SWEEP_MAX);
	g12a_mrg_sweep("MGSq", g12a_mrg_txdqs_set, g12a_mrg_txdqs_restore,
		       -1, G12A_MRG_SWEEP_MAX);
	bad = g12a_mrg_probe_cur();
	g12a_uart_puts("MGSq restored ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (extremes)
		g12a_mrg_extreme("MGXq", g12a_mrg_txdqs_set,
				 g12a_mrg_txdqs_restore, -255, 255);

	g12a_mrg_rb_csr = G12A_MRG_DB(0, 0, G12A_MRG_RXPBDLY_TG0);
	g12a_mrg_sweep("MGSp", g12a_mrg_rxpb_set, g12a_mrg_rxpb_restore,
		       +1, G12A_MRG_SWEEP_MAX);
	g12a_mrg_sweep("MGSp", g12a_mrg_rxpb_set, g12a_mrg_rxpb_restore,
		       -1, G12A_MRG_SWEEP_MAX);
	bad = g12a_mrg_probe_cur();
	g12a_uart_puts("MGSp restored ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (extremes)
		g12a_mrg_extreme("MGXp", g12a_mrg_rxpb_set,
				 g12a_mrg_rxpb_restore, -63, 63);

	g12a_mrg_rb_csr = G12A_MRG_DB(0, 0, G12A_MRG_RXCLKDLY_TG0);
	g12a_mrg_sweep("MGSk", g12a_mrg_rxclk_set, g12a_mrg_rxclk_restore,
		       +1, G12A_MRG_SWEEP_MAX);
	g12a_mrg_sweep("MGSk", g12a_mrg_rxclk_set, g12a_mrg_rxclk_restore,
		       -1, G12A_MRG_SWEEP_MAX);
	bad = g12a_mrg_probe_cur();
	g12a_uart_puts("MGSk restored ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (extremes)
		g12a_mrg_extreme("MGXk", g12a_mrg_rxclk_set,
				 g12a_mrg_rxclk_restore, -63, 63);

	g12a_uart_puts("MGS end\n");
}

static void g12a_mrg_stress_one(const char *tag, u32 pat, ulong base,
				ulong words)
{
	u64 t0 = g12a_mrg_ticks();
	u32 bad = g12a_mrg_run(pat, base, words, 0);
	u64 dt = g12a_mrg_ticks() - t0;

	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)pat);
	g12a_uart_putc(' ');
	g12a_uart_hex32(bad);
	g12a_uart_puts(" t ");
	g12a_uart_hex32((u32)(dt >> 32));
	g12a_uart_hex32((u32)dt);
	g12a_uart_puts("\n");
	/*
	 * Errors are reported AGAINST time and temperature, not as a
	 * total: a soak that only reports a final count cannot say at
	 * which temperature it was clean.
	 */
	g12a_mrg_temp("soak");
}

/*
 * The soak is also the heat source.  There is no chamber here, so the
 * only temperature delta this rig can produce is the one this loop makes
 * itself, and the hot shmoo runs immediately after it — never after an
 * idle gap that would let the die fall back.
 */
static void g12a_mrg_stress(void)
{
	u32 pat, rep, cyc;
	u64 f;

	asm volatile("mrs %0, cntfrq_el0" : "=r" (f));
	g12a_uart_puts("MGT begin cntfrq ");
	g12a_uart_hex32((u32)f);
	g12a_uart_puts("\n");

	for (cyc = 0; cyc < G12A_MRG_SOAK_CYCLES; cyc++) {
		g12a_uart_puts("MGT cycle ");
		g12a_uart_hex8((u8)cyc);
		g12a_uart_puts("\n");

		/* Full span, the two highest-toggle patterns. */
		for (pat = 0; pat < 2; pat++)
			g12a_mrg_stress_one("MGTf", pat, G12A_MRG_FULL_BASE,
					    G12A_MRG_FULL_WORDS);

		/* Every pattern over a 128 MiB window, repeated. */
		for (rep = 0; rep < G12A_MRG_WIN_REPEAT; rep++)
			for (pat = 0; pat < G12A_MRG_NPAT; pat++)
				g12a_mrg_stress_one("MGTw", pat,
						    G12A_MRG_WIN_BASE,
						    G12A_MRG_WIN_WORDS);
	}

	g12a_uart_puts("MGT end\n");
}

/*
 * G12-C6 — compact WIDTH at the trained centre.  Reuses g12a_mrg_sweep
 * (stop at first failing probe, bound G12A_MRG_SWEEP_MAX).  Axes:
 *
 *   v  VrefDAC0/1  csr 0x10040  PA 0xFE020080  (SoC RX vref, class 9)
 *   q  TxDqsDlyTg0 csr 0x100D0  PA 0xFE0201A0  (write DQS LCDL, class 2)
 *
 * Width is inclusive of the trained centre: last-pass+ + last-pass- + 1.
 * No soak, no extremes, no DDC.
 */
#ifdef CONFIG_MESON_G12A_DRAM_WIDTH
static void g12a_mrg_width_line(char axis, int lo, int hi, u32 csr)
{
	int w = lo + hi + 1;

	g12a_uart_puts("MGW ");
	g12a_uart_putc(axis);
	g12a_uart_putc(' ');
	g12a_uart_putc('-');
	g12a_uart_hex8((u8)lo);
	g12a_uart_putc(' ');
	g12a_uart_putc('+');
	g12a_uart_hex8((u8)hi);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)w);
	g12a_uart_putc(' ');
	g12a_uart_hex32(csr);
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_MRG_PA(csr)));
	g12a_uart_puts("\n");
}

static void g12a_dram_width(void)
{
	u16 mux;
	u32 bad;
	int hi, lo;
	u32 csr_v, csr_q;

	g12a_uart_puts("MGW begin\n");
	mux = g12a_mrg_mux_enter();
	g12a_mrg_dump();

	bad = g12a_mrg_probe();
	g12a_uart_puts("MGW centre ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	if (bad) {
		g12a_uart_puts("MGW abort: centre fails\n");
		g12a_mrg_mux_exit(mux);
		g12a_zero32(CONFIG_MESON_G12A_SPL_ENTRY, 16);
		g12a_uart_puts("MGW end\n");
		return;
	}

	csr_v = G12A_MRG_DB(0, 0, G12A_MRG_VREFDAC0);
	g12a_mrg_rb_csr = csr_v;
	hi = g12a_mrg_sweep("MGSv", g12a_mrg_vref_set, g12a_mrg_vref_restore,
			    +1, G12A_MRG_SWEEP_MAX);
	lo = g12a_mrg_sweep("MGSv", g12a_mrg_vref_set, g12a_mrg_vref_restore,
			    -1, G12A_MRG_SWEEP_MAX);
	g12a_mrg_width_line('v', lo, hi, csr_v);

	csr_q = G12A_MRG_DB(0, 0, G12A_MRG_TXDQSDLY_TG0);
	g12a_mrg_rb_csr = csr_q;
	hi = g12a_mrg_sweep("MGSq", g12a_mrg_txdqs_set, g12a_mrg_txdqs_restore,
			    +1, G12A_MRG_SWEEP_MAX);
	lo = g12a_mrg_sweep("MGSq", g12a_mrg_txdqs_set, g12a_mrg_txdqs_restore,
			    -1, G12A_MRG_SWEEP_MAX);
	g12a_mrg_width_line('q', lo, hi, csr_q);

	g12a_mrg_mux_exit(mux);
	g12a_zero32(CONFIG_MESON_G12A_SPL_ENTRY, 16);
	g12a_uart_puts("MGW end\n");
}
#endif

/*
 * Ordered by risk, so a hang still leaves the earlier phases on the wire:
 * the trained-state dump costs nothing, the cold shmoo repeats a
 * measurement already reproduced three times, the soak is long but
 * passive, the hot shmoo is the soak's payoff, and only then does
 * anything touch a power rail.
 */
#if defined(CONFIG_MESON_G12A_DRAM_MARGIN) || \
    defined(CONFIG_MESON_G12A_DRAM_WIDTH)
static void g12a_dram_margin(void)
{
#ifdef CONFIG_MESON_G12A_DRAM_WIDTH
	g12a_dram_width();
	return;
#else
	u16 mux;
	u32 high;

	g12a_uart_puts("MG begin\n");
	g12a_ts_init();
	g12a_mrg_temp("boot");
	g12a_mrg_wait_ticks(2000000ULL);
	g12a_mrg_temp("boot");
	g12a_mrg_vddee_report("nominal", G12A_MRG_VDDEE_NOM_HIGH);

	mux = g12a_mrg_mux_enter();
	g12a_uart_puts("MG mux ");
	g12a_uart_hex16(mux);
	g12a_uart_puts("\n");

	g12a_mrg_dump();

#if G12A_MRG_P2
	/*
	 * P2 phase set — open training.  Mutually exclusive with T19 and
	 * the environmental set for the same reason they are mutually
	 * exclusive with each other: the whole run happens inside
	 * dram_init(), before the SPL image's residual window opens.
	 */
	g12a_mrg_temp("p2-pre");
	g12a_p2_open_train();
	g12a_mrg_temp("p2-post");
	g12a_mrg_mux_exit(mux);
	g12a_zero32(CONFIG_MESON_G12A_SPL_ENTRY, 16);
	g12a_uart_puts("MG end\n");
	return;
#endif

#if G12A_MRG_T19
	/*
	 * T19 phase set.  The soak and the VDDEE sweep below are ~65
	 * minutes and are already measured (dram-margin-environmental.md);
	 * running them here would push the per-lane pass past the residual
	 * window the SPL image arrives in.  Order is by risk, as always:
	 * the aggregate shmoo reproduces a figure measured on three prior
	 * boots, the same sweeps then repeat on the SHORT probe so its
	 * sensitivity is calibrated rather than assumed, the per-lane pass
	 * is the new measurement, and the never-exercised DMC BIST is last.
	 */
	g12a_mrg_temp("t19-pre");
#if G12A_MRG_P6
	g12a_p6_centre();
#endif
	g12a_mrg_shmoo(0, "agg-full");
	g12a_mrg_fast = 1;
	g12a_mrg_shmoo(0, "agg-fast");
	g12a_mrg_fast = 0;
	g12a_mrg_per_lane();
	g12a_mrg_temp("t19-post");
	g12a_mrg_mux_exit(mux);
	g12a_mrg_bist();
	g12a_zero32(CONFIG_MESON_G12A_SPL_ENTRY, 16);
	g12a_uart_puts("MG end\n");
	return;
#endif

	/* Phase 1 — the eye as cold as this rig ever sees it. */
	g12a_mrg_temp("cold-pre");
	g12a_mrg_shmoo(1, "cold");
	g12a_mrg_temp("cold-post");
	g12a_mrg_mux_exit(mux);

	/* Phase 2 — soak, which is also the only heater available. */
	g12a_mrg_stress();

	/* Phase 3 — the same eye at the top of the reached range. */
	mux = g12a_mrg_mux_enter();
	g12a_mrg_temp("hot-pre");
	g12a_mrg_shmoo(0, "hot");
	g12a_mrg_temp("hot-post");

	/* Phase 4 — VDDEE, in vendor-table steps, restored at the end. */
	for (high = 0; high <= G12A_MRG_VDDEE_MAX_HIGH; high += 2) {
		g12a_mrg_vddee_set(high);
		g12a_mrg_vddee_report("set", high);
		g12a_mrg_temp("vddee");
		g12a_mrg_shmoo(0, "vddee");
	}
	g12a_mrg_vddee_set(G12A_MRG_VDDEE_NOM_HIGH);
	g12a_mrg_vddee_report("restored", G12A_MRG_VDDEE_NOM_HIGH);
	g12a_uart_puts("MGV probe ");
	g12a_uart_hex32(g12a_mrg_probe());
	g12a_uart_puts("\n");
	g12a_mrg_temp("end");
	g12a_mrg_mux_exit(mux);

	/*
	 * The stress spans cover CONFIG_MESON_G12A_SPL_ENTRY, and the
	 * handoff decides "an SPL is present" from the first word there.
	 */
	g12a_zero32(CONFIG_MESON_G12A_SPL_ENTRY, 16);
	g12a_uart_puts("MG end\n");
#endif /* !CONFIG_MESON_G12A_DRAM_WIDTH */
}
#else
static inline void g12a_dram_margin(void) { }
#endif /* CONFIG_MESON_G12A_DRAM_MARGIN || CONFIG_MESON_G12A_DRAM_WIDTH */

#ifdef CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE
/*
 * Product write-DQS centring (CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE), the slow
 * optimisation of plan/dram-bringup-quick-slow.md, entered two ways:
 *
 *   B, the quick check (g12a_dram_txdqs_quick): after training, move every
 *     nibble's TxDqsDly by +T and probe, by -T and probe, and restore.  A
 *     failing probe means the worst side is below T steps: centre.  T is
 *     CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE_THRESH; 0 centres unconditionally.
 *   A, the failure fallback (g12a_dram_txdqs_rescue): the DRAM test found a
 *     bad word and the centring has not run this boot: centre, then the
 *     caller tests again.
 *
 * Centring is P6's pass: G2's window search, each nibble's centre applied
 * to both timing groups by the same delta, the vendor values put back if
 * the centred state probes bad.  Prints "TXQ <T> <bad+> <bad->" for the quick
 * check, "TXA <bad>" when A fires, then P6's "P6 pre" / eight "P6Cq" /
 * "P6 post" / "P6 base" if it centres.
 */
static int g12a_txdqs_centred;

static void g12a_txdqs_snap(void)
{
	u32 d, n, t;

	for (d = 0; d < 4; d++)
		for (n = 0; n < 2; n++)
			for (t = 0; t < 2; t++)
				g12a_mrg_txdqs[d][n][t] = g12a_phy_read16(
					G12A_MRG_PA(G12A_MRG_DB(d, n,
						G12A_MRG_TXDQSDLY_TG0 + t)));
}

/* Measurement only: override the Kconfig span (0 = the full 256-code sweep). */
#ifdef G12A_TXDQS_SPAN_OVERRIDE
#define G12A_TXDQS_SPAN		G12A_TXDQS_SPAN_OVERRIDE
#else
#define G12A_TXDQS_SPAN		CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE_SPAN
#endif

static void g12a_txdqs_centre_now(void)
{
	const int span = G12A_TXDQS_SPAN;
	u32 d, n, t;

	if (span > 0) {
		int lo = G12A_G2_MAX, hi = 0;

		for (d = 0; d < 4; d++)
			for (n = 0; n < 2; n++)
				for (t = 0; t < 2; t++) {
					int v = (int)g12a_mrg_dec4(
							g12a_mrg_txdqs[d][n][t]);

					if (v < lo)
						lo = v;
					if (v > hi)
						hi = v;
				}
		g12a_g2_vlo = lo - span < 0 ? 0 : lo - span;
		g12a_g2_vhi = hi + span > G12A_G2_MAX ? G12A_G2_MAX : hi + span;
	}
	g12a_g2_quiet = 1;
	g12a_p6_centre();
	g12a_g2_quiet = 0;
	g12a_g2_vlo = 0;
	g12a_g2_vhi = G12A_G2_MAX;
	g12a_txdqs_centred = 1;
}

static void g12a_dram_txdqs_quick(void)
{
	const int thr = CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE_THRESH;
	u16 mux = g12a_mrg_mux_enter();
	u32 up = 0, dn = 0;

	g12a_txdqs_snap();
	if (thr > 0) {
		g12a_mrg_txdqs_set(thr);
		up = g12a_mrg_probe_fast();
		g12a_mrg_txdqs_set(-thr);
		dn = g12a_mrg_probe_fast();
		g12a_mrg_txdqs_restore();
		g12a_uart_puts("TXQ ");
		g12a_uart_hex8((u8)thr);
		g12a_uart_putc(' ');
		g12a_uart_hex32(up);
		g12a_uart_putc(' ');
		g12a_uart_hex32(dn);
		g12a_uart_puts("\n");
	}
	if (thr == 0 || up || dn)
		g12a_txdqs_centre_now();
	g12a_mrg_mux_exit(mux);
}

/* Returns 1 when it centred, so the caller tests again. */
static int g12a_dram_txdqs_rescue(u32 bad)
{
	u16 mux;

	if (!bad || g12a_txdqs_centred)
		return 0;
	g12a_uart_puts("TXA ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	mux = g12a_mrg_mux_enter();
	g12a_txdqs_snap();
	g12a_txdqs_centre_now();
	g12a_mrg_mux_exit(mux);
	return 1;
}
#endif
#else
static inline void g12a_dram_margin(void) { }
#endif /* MARGIN || WIDTH || TXDQS_CENTRE */

#ifndef CONFIG_MESON_G12A_DRAM_TXDQS_CENTRE
static inline void g12a_dram_txdqs_quick(void) { }
static inline int g12a_dram_txdqs_rescue(u32 bad) { return 0; }
#endif

/*
 * Measurement only: report the first DRAM test as failed once, so fallback A
 * can be exercised on a healthy bus.  Never set in a product image.
 */
#ifndef G12A_TXDQS_FORCE_A
#define G12A_TXDQS_FORCE_A	0
#endif

/*
 * ---------------------------------------------------------------------
 * THE OPEN HOST-SIDE DDR INIT PATH  (CONFIG_MESON_G12A_DRAM_OPEN)
 *
 * Everything below performs, in the vendor's own order, the a644 steps
 * that have been transcribed instruction-by-instruction and verified
 * against captured hardware state.  The vendor PMU firmware images stay
 * -- they are staged into PHY IMEM/DMEM and run -- and the host drives
 * them over the UCT mailbox.  Nothing else on the host side calls into
 * aml_ddr.fw on this path except the two steps that are NOT transcribed:
 *
 *   S1  fffb931c  PLL / DMC address-map + geometry   -- still the blob
 *   T10 fffb8af0  DMC<->PHY handshake state machine  -- still the blob
 *
 * plus fp_pll and the two post-a644 exports (slot5 size detect, slot6),
 * which were never inside a644 to begin with.
 *
 * Sources, one per step (each carries its own N-of-M verification
 * against a hardware capture):
 *   S2/T1  fffb6a1c  timing engine    -> g12a-ddr-timing.h   52/52
 *   S3/T5b fffbe244  message block    -> g12a-ddr-open.h     36/37
 *   S4     fffb7f30  PHY static cfg   -> g12a-ddr-open.h    103/104
 *   S5/T2  fffb6328  MID1 DMC program -> g12a-ddr-timing.h   52/52
 *   S6/S7  fffb4ed0/fffb4e64 pin-mux  -> P2.1 compare pass   52/52
 *   S9     fffb6294  IMEM/DMEM load, train kick, UCT mailbox, S11/S12,
 *   T7/T11/T12/T14   S17a, loadPIEImage, DMC go, vref  -> below
 *
 * Omitted deliberately, each with a measured reason:
 *   S14/S15/T8  fffb502c/fffb5a54 delay SAVE.  dir==1 is CSR->memory and
 *      in mode B nothing in the blob reads the saved buffers back
 *      (class-restore-transcription.md), so an always-train path can omit
 *      the whole 650-instruction function.
 *   T5b's fffb94d0 DFI nudge: gated on ds[8] bit 29 (fffb94e8 tbz) and the
 *      shipped ACS has ds[8] = 0x80000000, so it never executes.
 *   T5b's fffba544 post-train shift: gated on ds[127], shipped 0.
 *   T9 fffbbbb0: up to three user CSR overrides from ctl+461, never set.
 *
 * The default is OFF: CONFIG_MESON_G12A_DRAM_OPEN is in no product
 * config, so the shipping TPL neither compiles nor runs a line of this.
 */
#ifdef CONFIG_MESON_G12A_DRAM_OPEN

/*
 * Two arms, and the SAFE one is the default.
 *
 * G12A_OPEN_COMPARE=1 leaves the blob's own a644 walk in place and runs
 * every open computation READ-ONLY against the state it left, at the
 * instant a644 has finished S2..S7 and nothing has trained.  A read
 * cannot perturb the boot, so a formula error is a result.
 *
 * G12A_OPEN_LIVE=1 replaces the walk: the open path performs S2..T14
 * itself and calls the blob only for the two untranscribed steps.
 */
#ifdef CONFIG_MESON_G12A_DRAM_OPEN_LIVE
#define G12A_OPEN_COMPARE	0
#define G12A_OPEN_LIVE		1
#else
#define G12A_OPEN_COMPARE	1
#define G12A_OPEN_LIVE		0
#endif

/*
 * Restore T8/T9 as BLOB calls inside the otherwise-open stage 2.  See the
 * call site: this is the discriminator for the open-live-1 T10 hang.
 */
#ifndef G12A_OPEN_KEEP_T8T9
#if G12A_OPEN_REPLAY
/*
 * Mode A has no aml_ddr.fw to call, and T8 is ours here anyway
 * (g12a_open_class_rw(ds, 1) is fffb5a54 with dir = 1).  T9 (fffbbbb0) reads
 * the user-CSR-override list at ctl+0x1CD, which replay sets to 0, so it has
 * nothing to do.  Forced off rather than left to the command line.
 */
#define G12A_OPEN_KEEP_T8T9	0
#else
#define G12A_OPEN_KEEP_T8T9	1
#endif
#endif

/*
 * Bisection knob for the open-live T10 hang: 1 = call the BLOB's T7
 * instead of ours.
 *
 * DEFAULT NOW 0 -- OURS.  The hang was our T7 skipping item 4, the PIE
 * image + ACSM sequence load; with it implemented the blob's T10 returns
 * (`<ta 01`), the DMC comes up at 3928 MB and both ranks pass DataBus and
 * AddrBus, and a 1949-register PHY CSR sweep taken between T7's return and
 * T10 shows ACSM 80 -> 0 and INITENG 302 -> 0 differences against the
 * blob's T7 -- an EMPTY difference set once the trained/counter classes
 * csr-state-diff-v3.md sec.5.2 pre-registers as DRIFTING are excluded.
 * Measured over five boots (t7-dtd-state-diff.md sec.3.4).  The knob stays
 * so the blob's T7 remains one #define away as a control.
 */
#ifndef G12A_OPEN_BLOB_T7
#define G12A_OPEN_BLOB_T7	0
#endif

/*
 * T7 item 4 -- where the PIE prod-code image comes from.
 *
 * 1 (default) = our own table, generated by
 *     amlogic/software/boot/g12/scripts/piei_gen.py --slice dl44 --emit c
 * from piei.fw's 'dl44' (LPDDR4) slice into g12a-piei-dl44.h.  This is the
 * last thing on the open path that read piei.fw, so with it set the open arm
 * consumes no vendor DDR blob at all.
 *
 * 0 = walk the staged blob's own dl44 slice through ctl+0x98
 * (G12A_CTL_OFF_PIEI_SEL), which is what the vendor consumer at blob VA
 * fffb5008 does and what this function did before.  Kept as the control:
 * the two paths write the same 487 (csr, data) pairs in the same order by
 * construction, so a boot difference between them is a defect in the table
 * or in its emission, not in the surrounding sequence.
 *
 * The table is UNVALIDATED ON SILICON.  piei-open-writer.md sec.2 ran the only
 * host-side check available and it was negative for a sequencing reason: the
 * archived capture predates the PIE load.  The site-2 PCSR capture immediately
 * after this function (g12a_pcsr_capture(ctl, 2)) is what settles it -- with
 * this knob at 1 the capture reads back OUR writes, at 0 the blob's.
 */
#ifndef G12A_OPEN_PIEI_TABLE
#define G12A_OPEN_PIEI_TABLE	1
#endif

#if G12A_OPEN_PIEI_TABLE
#include <asm/arch/g12a-piei-dl44.h>
#endif

/* Per-message UCT stream trace; see the harvest. */
#ifndef G12A_OPEN_TRACE_STREAM
#define G12A_OPEN_TRACE_STREAM	0
#endif

/*
 * G12A_OPEN_ASSERT_ID -- name the assert that aborted a failing pass.
 * Ported from topic/open-replay (audit 2026-09-26-1d-fail-reporting... sec.
 * Findings (d)).  fn 0x0012c publishes the assert's id at 1d:0x0164 as a
 * major-8 header (id << 16) | nargs -- bypassing fn 0x00ff8's level compare,
 * skipped only at HdtCtrl == 0xff -- and 1d:0x0170 posts the fixed sentinel
 * 0x04020000 immediately after; so the header preceding the sentinel is the
 * abort's own.  Latch it and print after OPT FAIL (0xFFFFFFFF = never
 * latched).  An assert record is emitted only on a boot already failing
 * (fn 0x0012c ends in brk_s), so this cannot perturb a passing arm.
 * DEFAULT 1 here -- unlike open-replay's 0 -- this tree is the diagnostic
 * vehicle; costs one compare per harvested message plus the print.
 */
#ifndef G12A_OPEN_ASSERT_ID
#define G12A_OPEN_ASSERT_ID	1
#endif

/*
 * Bisection knob for the residual DRAM errors: 1 = call the BLOB's T14
 * (fffb4bc4 mode 2) instead of ours.  Ours is the only step on this path
 * that issues an MRS to the DRAM itself, and a malformed one lands on
 * MR14 (VREF-DQ) -- which is exactly the shape of "trains, sizes, passes
 * DataBus, then fails scattered words".
 */
#ifndef G12A_OPEN_BLOB_T14
#define G12A_OPEN_BLOB_T14	0
#endif

/*
 * The last two untranscribed a644 steps, now transcribed.  Both knobs
 * default to 0 = OURS; set either to 1 to put the blob's back as a control.
 *
 * S1  fffb931c -- the DMC scratch-register writability probe (fffb929c),
 *		   the ds+40 / ds+42 density -> field-code conversion
 *		   (fffb8e60), the DMC geometry word 0xFF639368 and the
 *		   AO_SEC_GP_CFG0 density field 0xFF800240[31:16].
 * T10 fffb8af0 -- the DMC<->PHY handshake state machine on
 *		   0xFF638624/8628/8648/8650/8654, then a 10-word copy of
 *		   ds->dmc_remap[5], twice, into 0xFF639340..0xFF639364.
 */
#ifndef G12A_OPEN_BLOB_S1
#define G12A_OPEN_BLOB_S1	0
#endif
#ifndef G12A_OPEN_BLOB_T10
#define G12A_OPEN_BLOB_T10	0
#endif

/*
 * G12A_OPEN_SLOT6 -- slot6 (export[6], G12A_AML_FN_POST, real VA
 * 0xFFFBD52C).  1 = run g12a_open_slot6(), our transcription; 0 = run the
 * blob through g12a_t_call('p', ...), i.e. today's behaviour.
 *
 * This knob is an A/B between TWO IMPLEMENTATIONS OF THE SAME STEP, not
 * between the step and nothing: both arms leave the same register state if
 * the transcription is right, so a boot can compare ours against the blob's
 * rather than against an ablation.  Default 0 -- the blob stays current
 * behaviour until a boot says otherwise.
 *
 * ⚠️ A green five-walk Layer-1 fingerprint on the 1 arm proves NOTHING about
 * the one thing slot6 durably changes (0xFF639254 bit 1) -- see the comment
 * on g12a_open_slot6() below.
 */
#ifndef G12A_OPEN_SLOT6
#define G12A_OPEN_SLOT6		0
#endif

/*
 * G12A_OPEN_SLOT5 -- slot5 (export[5], G12A_AML_FN_MAIN, real VA
 * 0xFFFB9DA8), the SIZE DETECT.  1 = run g12a_open_slot5(), our
 * transcription of the DISCOVERY half; 0 = the blob through
 * g12a_t_call('m', ...), i.e. today's behaviour.  Consulted only when
 * G12A_KEEP_SLOT5 is 1: G12A_KEEP_SLOT5=0 still skips the step outright on
 * both arms, exactly as before.
 *
 * Same A/B shape as G12A_OPEN_SLOT6 -- two implementations of one step,
 * both meant to leave ds+0x28 / ds+0x2a, 0xFF639368 and AO_SEC_GP_CFG0 in
 * the same state -- so a boot compares ours against the blob's by the
 * "OP5" crumbs rather than by outcome.  The blob's tail (DataBus/AddrBus)
 * is NOT part of this arm: that half is G12A_BUS_TEST, already transcribed
 * and negative-controlled, which g12a_dram_test() runs.
 *
 * Default 0.  Transcription and the zero-boot check of its arithmetic:
 * amlogic/software/boot/g12/aml-ddr-residue-transcription.md.
 */
#ifndef G12A_OPEN_SLOT5
#define G12A_OPEN_SLOT5		0
#endif

/* The blob-call wrappers live below this block; the open path uses them
 * for the two untranscribed steps and for the bisection knobs above. */
static int g12a_t_call(char tag, ulong fn, ulong ctl);
static int g12a_t_call2(char tag, ulong fn, ulong ctl, int arg);

/* The 1 MHz free-running counter fffb4134 polls (fffb4134/fffb413c). */
#define G12A_OPEN_TIMER			0xFFD0F188UL
/*
 * ST bounds both mailbox waits at PHYINIT_TIMEOUT_US_1S; the blob spins
 * forever (fffb4304 / fffb4354), which is the shape of the observed msg08
 * stall -- a PMU that stopped posting is indistinguishable from a slow
 * one.  Bound ours.
 */
#define G12A_OPEN_UCT_TIMEOUT_US	1000000u

static u32 g12a_open_us(void)
{
	return readl((void *)G12A_OPEN_TIMER);
}

/* fffb4134 -- a MICROsecond delay, pinned by the JEDEC tINIT1/tINIT3 pair. */
static void g12a_open_delay_us(u32 n)
{
	u32 t0 = g12a_open_us();

	while ((g12a_open_us() - t0) <= n)
		;
}

/*
 * The delay codec, fffba174 / fffba1e0.  Type 1 is the only ENCODE arm
 * carrying the saturating `v > 0x3F -> (v & 0x1F) + 0x60` branch
 * (fffba1ec); types 2/4/6 reach fffba1f4 directly and use the general
 * form.  Getting that backwards collapses a trained DQS delay -- measured,
 * dram-margin.md sec.3.
 */
static u32 g12a_open_dly_dec(u32 type, u32 raw)
{
	switch (type) {
	case 1:
		return (raw & 0x1fu) +
		       ((((raw >> 5) & 1u) + ((raw >> 6) & 1u)) << 5);
	case 2:
	case 4:
		return (raw & 0x3fu) + (((raw >> 6) & 7u) << 5);
	case 6:
		return (raw & 0x1fu) +
		       ((((raw >> 5) & 1u) + ((raw >> 6) & 0x1fu)) << 5);
	default:
		return (type > 0x10u) ? raw : (raw & 0x3fu);
	}
}

static u32 g12a_open_dly_enc(u32 type, u32 v)
{
	if (type == 1u)					/* fffba1ec */
		return (v > 0x3fu) ? ((v & 0x1fu) + 0x60u)
				   : ((v & 0x1fu) + ((v >> 5) << 6));
	if (type == 2u || type == 4u || type == 6u)	/* -> fffba1f4 */
		return (v & 0x1fu) + ((v >> 5) << 6);
	if (type == 3u || type == 5u)			/* fffba240 */
		return v & 0x3fu;
	return (type > 5u) ? v : 0u;			/* fffba254 */
}

/* PA = 0xFE000000 + (csr << 1) -- the rule every access site confirms. */
#define G12A_OPEN_CSR_PA(c)	(G12A_DDR_PHY_APB_BASE + ((ulong)(c) << 1))

static void g12a_open_csr_w(u32 csr, u32 val)
{
	g12a_phy_write16(G12A_OPEN_CSR_PA(csr), (u16)val);
}

static u16 g12a_open_csr_r(u32 csr)
{
	return g12a_phy_read16(G12A_OPEN_CSR_PA(csr));
}

/* Scratch shared by the emitters; sized by the headers' own maxima. */
static struct g12a_open_csr g12a_open_phy[G12A_OPEN_PHY_N];
static struct g12a_open_mbf g12a_open_mb[G12A_OPEN_MB_N];
/*
 * file-scope tm retired -- see g12a_open_s2s5_timing_mid1().
 * file-scope MID1 retired too: it is now the shared g12a_mid1_buf, taken
 * with g12a_mid1_claim() inside that same function.
 */

/* ---------------------------------------------------------------- S4 */

/**
 * g12a_open_s4_phy_static - a644 S4, fffb7f30, 105 CSRs
 * @apply: 0 = compare against what the blob left, 1 = write
 *
 * Return: count of CSRs whose live value differs from ours (compare
 * mode), 0 after a write, ~0 if the ACS is outside the transcribed arm.
 */
static u32 g12a_open_s4_phy_static(const u8 *ds, int apply)
{
	int n, i;
	u32 bad = 0;

	n = g12a_open_phy_static(ds, 0, g12a_open_phy);
	if (n <= 0) {
		g12a_uart_puts("OP4 refused\n");
		return 0xffffffffu;
	}
	if (apply) {
		for (i = 0; i < n; i++)
			g12a_open_csr_w(g12a_open_phy[i].csr,
					g12a_open_phy[i].val);
		g12a_uart_puts("OP4 wrote ");
		g12a_uart_hex32((u32)n);
		g12a_uart_puts("\n");
		return 0;
	}
	for (i = 0; i < n; i++) {
		u16 live = g12a_open_csr_r(g12a_open_phy[i].csr);

		if (live == g12a_open_phy[i].val)
			continue;
		bad++;
		g12a_uart_puts("OP4 ");
		g12a_uart_hex32(g12a_open_phy[i].csr);
		g12a_uart_putc(' ');
		g12a_uart_hex16(g12a_open_phy[i].val);
		g12a_uart_putc(' ');
		g12a_uart_hex16(live);
		g12a_uart_puts("\n");
	}
	g12a_uart_puts("OP4 n ");
	g12a_uart_hex32((u32)n);
	g12a_uart_puts(" bad ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	return bad;
}

/* ---------------------------------------------------------------- S3 */

/**
 * g12a_open_s3_msgblock - a644 S3 (and T5b's prepare), fffbe244
 *
 * The 2D arm's cross-buffer side effect is reproduced: after its own
 * fields it sets the 1D block's HdtCtrl to 4 (fffbe4ac).  On this chain
 * ctl+104 and ctl+112 are the SAME address, so that last store is what
 * the buffer ends up holding -- as it does under the blob today.
 */
static u32 g12a_open_s3_msgblock(const u8 *ds, int is2d, int apply)
{
	volatile u8 *mb = (volatile u8 *)G12A_MSG_BLOCK_BASE;
	int n, i;
	u32 bad = 0;

	n = g12a_open_msgblock(ds, 0, is2d, g12a_open_mb);
	if (n <= 0) {
		g12a_uart_puts("OP3 refused\n");
		return 0xffffffffu;
	}
	for (i = 0; i < n; i++) {
		u32 off = g12a_open_mb[i].off;
		u32 w = g12a_open_mb[i].width;
		u32 v = g12a_open_mb[i].val;

		if (apply) {
			mb[off] = (u8)v;
			if (w == 2)
				mb[off + 1] = (u8)(v >> 8);
			continue;
		}
		{
			u32 live = mb[off];

			if (w == 2)
				live |= (u32)mb[off + 1] << 8;
			if (live == v)
				continue;
			bad++;
			g12a_uart_puts("OP3 ");
			g12a_uart_hex8((u8)off);
			g12a_uart_putc(' ');
			g12a_uart_hex16((u16)v);
			g12a_uart_putc(' ');
			g12a_uart_hex16((u16)live);
			g12a_uart_puts("\n");
		}
	}
	if (apply && is2d)
		mb[18] = 4;				/* fffbe4a8/4ac */
	g12a_uart_puts(apply ? "OP3 wrote " : "OP3 n ");
	g12a_uart_hex32((u32)n);
	if (!apply) {
		g12a_uart_puts(" bad ");
		g12a_uart_hex32(bad);
	}
	g12a_uart_puts("\n");
	return bad;
}

/* ----------------------------------------------------------- S2 + S5 */

/*
 * The eight fixed 0xFF6386xx stores fffb6328 makes alongside the
 * 0xFF6384xx window (fffb6578-fffb67d8).  They lie outside
 * g12a_mid1_compute()'s captured comparison window, so they are
 * transcribed here rather than in the header.
 *
 * The one store dropped on purpose is fffb6600-18, which copies
 * 0xFF638614 back into tm[384 + 4*(pstate & 1)] -- past the end of the
 * 64-word tm this TPL carries, and read by nothing on this path.
 */
static void g12a_open_mid1_600(const u8 *ds, u32 *tm)
{
	u32 v;
	u32 d52 = g12a_open_h(ds, 52) | ((u32)ds[54] << 16) |
		  ((u32)ds[55] << 24);

	writel(0x1e, (void *)0xFF638600UL);		/* fffb6580/84 */
	writel(d52, (void *)0xFF638610UL);		/* fffb6588-94 */
	v = 0x08080000u;				/* fffb65a0 */
	if (ds[14] == 3u)				/* fffb65a8-b0 */
		v = (G12A_TM(tm, 32) - 2u) | 0x08080000u;
	writel(v, (void *)0xFF638614UL);
	if ((d52 & 0x101u) == 0x101u) {			/* fffb65cc-fc */
		v = 0x08080000u |
		    ((G12A_TM(tm, 28) - G12A_TM(tm, 32)) << 8);
		writel(v, (void *)0xFF638614UL);
	}
	v = g12a_open_h(ds, 42) ? 4u : 1u;		/* fffb661c-34 csinc */
	if (ds[14] <= 1u)				/* fffb6638-50 */
		v = 2u;
	writel(v, (void *)0xFF63862CUL);
	writel(0x30, (void *)0xFF638604UL);		/* fffb6750-5c */
	writel(0, (void *)0xFF638608UL);		/* fffb6760-68 */
	writel(0, (void *)0xFF63860CUL);		/* fffb676c-74 */
	if (ds[14] == 2u)				/* fffb67a4-d8 */
		v = (((u32)ds[65] & 1u) << 11) |
		    (((u32)ds[65] & 2u) << 12) |
		    (((u32)ds[16] | 2u) & 0xffu);
	else						/* fffb6778-a0 */
		v = ((((u32)ds[65] & 1u) << 12) |
		     (((u32)ds[65] & 1u) << 11)) |
		    (((u32)ds[16] | 2u) & 0xffu);
	writel(v, (void *)0xFF638618UL);
}

/**
 * g12a_open_s2s5_timing_mid1 - a644 S2+S5 (and T1+T2)
 * @apply: 0 = compare, 1 = write
 */
static u32 g12a_open_s2s5_timing_mid1(const u8 *ds, int apply)
{
	/*
	 * tm is LOCAL.  It was a file-scope static; that was changed to rule
	 * out the storage as the cause of the tm[64] loss, and it is not --
	 * the loss reproduced identically on both (see the volatile note on
	 * g12a_timing_engine()'s @tm in g12a-ddr-timing.h, and
	 * t7-dtd-state-diff.md sec.5).
	 */
	u32 tmbuf[G12A_TM_WORDS];
	u32 *g12a_open_tm = tmbuf;
	/*
	 * MID1 is the shared g12a_mid1_buf.  It was a file-scope static of
	 * its own; the MARGIN instrument kept a second identical one, and on
	 * open_margin (the only target compiling both) those 832 B overran
	 * `.sdram'.  Live only between this claim and the releases below.
	 */
	struct g12a_mid1_reg *g12a_open_mid1 = g12a_mid1_claim("OP5");
	u32 bad = 0;
	u32 lat7, lat8, lat15, lat16, lat17;
	int i, n;

	if (!g12a_open_mid1)
		return 0xffffffffu;

	for (i = 0; i < G12A_TM_WORDS; i++)
		g12a_open_tm[i] = 0;
	if (g12a_timing_engine(ds, 0, G12A_AML_DDR_F920, g12a_open_tm)) {
		g12a_uart_puts("OP2 refused\n");
		g12a_mid1_release();
		return 0xffffffffu;
	}
	/*
	 * The instrument that found the tm[64] loss, kept as the regression
	 * witness.  Latch five words -- four of them written by the speed-bin
	 * arm's own basic block -- through volatile, so the loads are pinned
	 * HERE, before any UART call can run, and dump the whole array so no
	 * offset argument is taken on trust.
	 *
	 * With a plain (non-volatile) @tm the run of 2026-08-19 read
	 *   OP5p lat 0000001c 00000000 0000000e 00000000 00000011
	 * i.e. tm[32], tm[44], tm[48] and tm[64] were all 0 in memory while
	 * the first three still reached MID1 correctly from registers.  With
	 * the volatile @tm it reads
	 *   OP5p lat 0000001c 0000000e 0000000e 0000001e 00000011
	 * and 0xFF638430 programs 0x1e, matching the blob.
	 */
	lat7 = *(volatile u32 *)&g12a_open_tm[7];	/* tm byte 28 */
	lat8 = *(volatile u32 *)&g12a_open_tm[8];	/* tm byte 32 */
	lat15 = *(volatile u32 *)&g12a_open_tm[15];	/* tm byte 60 */
	lat16 = *(volatile u32 *)&g12a_open_tm[16];	/* tm byte 64 <- */
	lat17 = *(volatile u32 *)&g12a_open_tm[17];	/* tm byte 68 */
	g12a_uart_puts("OP5p lat ");
	g12a_uart_hex32(lat7);
	g12a_uart_putc(' ');
	g12a_uart_hex32(lat8);
	g12a_uart_putc(' ');
	g12a_uart_hex32(lat15);
	g12a_uart_putc(' ');
	g12a_uart_hex32(lat16);
	g12a_uart_putc(' ');
	g12a_uart_hex32(lat17);
	g12a_uart_puts(" base ");
	g12a_uart_hex32((u32)(ulong)g12a_open_tm);
	g12a_uart_puts("\n");
	/* The whole array, so no offset argument is taken on trust. */
	for (i = 0; i < G12A_TM_WORDS; i += 8) {
		int k;

		g12a_uart_puts("OP5p w");
		g12a_uart_hex8((u8)i);
		for (k = 0; k < 8; k++) {
			g12a_uart_putc(' ');
			g12a_uart_hex32(g12a_open_tm[i + k]);
		}
		g12a_uart_puts("\n");
	}
	/*
	 * tm[16] = G12A_TM(tm, 64) read the instant g12a_timing_engine()
	 * returns, BEFORE g12a_mid1_compute() runs -- through the ordinary
	 * lvalue MID1 itself uses, so it witnesses what MID1 will see.
	 * Crumb: OP5e tm16 <v> at <&tm[16]>
	 */
	g12a_uart_puts("OP5e tm16 ");
	g12a_uart_hex32(G12A_TM(g12a_open_tm, 64));
	g12a_uart_puts(" at ");
	g12a_uart_hex32((u32)(ulong)&g12a_open_tm[16]);
	g12a_uart_puts("\n");
	n = g12a_mid1_compute(ds, 0, g12a_open_tm, g12a_open_mid1);
	{
		int k;

		for (k = 0; k < n; k++)
			if (g12a_open_mid1[k].addr == 0xFF638430u) {
				g12a_uart_puts("OP5e ent ");
				g12a_uart_hex32((u32)k);
				g12a_uart_putc(' ');
				g12a_uart_hex32(g12a_open_mid1[k].val);
				g12a_uart_puts("\n");
			}
	}
	/*
	 * 0xFF638430 = MID1(0xbd0) = G12A_TM(tm, 64) = tm[16] is the ONE of
	 * 52 MID1 words the live arm gets wrong (live 0, blob 0x1e), and a
	 * host + aarch64 build of the SAME g12a_timing_engine() over the same
	 * ACS computes 0x1e on both.  So the divergence is in the runtime ds
	 * or in tm, and these are the only inputs the tm[64] arm reads.
	 * Crumb: OP5i tm16 <tm[16]> f <ds56> cap <ds106> x8 <ds119> t <ds14>
	 *              v <ds100> <ds102> <ds104>
	 */
	g12a_uart_puts("OP5i tm16 ");
	g12a_uart_hex32(G12A_TM(g12a_open_tm, 64));
	g12a_uart_puts(" f ");
	g12a_uart_hex16((u16)(ds[56] | (ds[57] << 8)));
	g12a_uart_puts(" cap ");
	g12a_uart_hex16((u16)(ds[106] | (ds[107] << 8)));
	g12a_uart_puts(" x8 ");
	g12a_uart_hex8(ds[119]);
	g12a_uart_puts(" t ");
	g12a_uart_hex8(ds[14]);
	g12a_uart_puts(" v ");
	g12a_uart_hex16((u16)(ds[100] | (ds[101] << 8)));
	g12a_uart_putc('/');
	g12a_uart_hex16((u16)(ds[102] | (ds[103] << 8)));
	g12a_uart_putc('/');
	g12a_uart_hex16((u16)(ds[104] | (ds[105] << 8)));
	g12a_uart_puts("\n");
	if (apply) {
		for (i = 0; i < n; i++)
			writel(g12a_open_mid1[i].val,
			       (void *)(ulong)g12a_open_mid1[i].addr);
		/*
		 * Read every MID1 register straight back.  0xFF638430 (the
		 * tm[64] entry, MID1(0xbd0)) is the ONE word of the whole DMC
		 * dump that differs between the live and compare arms -- live
		 * 0x00000000, blob 0x0000001e -- and the host harness over the
		 * same g12a_mid1_compute() with the same ACS computes 0x1e.
		 * So either the write does not stick or the value is not what
		 * the harness says.  This prints both.
		 * Crumb: OP5v <addr> <wrote> <readback>
		 */
		for (i = 0; i < n; i++) {
			u32 rb = readl((void *)(ulong)g12a_open_mid1[i].addr);

			if (rb == g12a_open_mid1[i].val)
				continue;
			g12a_uart_puts("OP5v ");
			g12a_uart_hex32(g12a_open_mid1[i].addr);
			g12a_uart_putc(' ');
			g12a_uart_hex32(g12a_open_mid1[i].val);
			g12a_uart_putc(' ');
			g12a_uart_hex32(rb);
			g12a_uart_puts("\n");
		}
		g12a_open_mid1_600(ds, g12a_open_tm);
		g12a_uart_puts("OP5 wrote ");
		g12a_uart_hex32((u32)n);
		g12a_uart_puts("\n");
		g12a_mid1_release();
		return 0;
	}
	for (i = 0; i < n; i++) {
		u32 live = readl((void *)(ulong)g12a_open_mid1[i].addr);

		if (live == g12a_open_mid1[i].val)
			continue;
		bad++;
		g12a_uart_puts("OP5 ");
		g12a_uart_hex32(g12a_open_mid1[i].addr);
		g12a_uart_putc(' ');
		g12a_uart_hex32(g12a_open_mid1[i].val);
		g12a_uart_putc(' ');
		g12a_uart_hex32(live);
		g12a_uart_puts("\n");
	}
	g12a_uart_puts("OP5 n ");
	g12a_uart_hex32((u32)n);
	g12a_uart_puts(" bad ");
	g12a_uart_hex32(bad);
	g12a_uart_puts("\n");
	g12a_mid1_release();
	return bad;
}

/* -------------------------------------------------------------- S6/S7 */

/*
 * MID3 (S7, fffb4e64) then MID2 (S6, fffb4ed0).  Both formulas were
 * validated 52 of 52 against the blob's own programmed state by the
 * compare-only P2.1 pass, which is why they are written here directly.
 */
static void g12a_open_s6s7_pinmux(const u8 *ds)
{
	u32 i;

	for (i = 0; i < G12A_DS_DQ_REMAP_N; i++)
		g12a_open_csr_w(0x10000u | ((i >> 3) << 12) |
				(0xa0u + (i & 7u)),
				ds[G12A_DS_OFF_DQ_REMAP + i] & 7u);
	for (i = 0; i < G12A_AC_PINMUX_N; i++)
		g12a_open_csr_w(0x20000u |
				((i > 9u ? 0x106u : 0x100u) + i),
				g12a_ac_pinmux_lpddr4[i]);
	g12a_uart_puts("OP67 done\n");
}

/* ----------------------------------------------------------------- S9 */

/*
 * fffb623c: MemResetL = 2, MicroContMuxSel = 0, halfword copy, then
 * MicroContMuxSel = 1.  The `2` here is the write the PHY-static page's
 * single MemResetL non-match is attributable to.
 */
static void g12a_open_copy16(ulong src, u32 bytes, u32 csr_base)
{
	u32 i, n = bytes >> 1;

	g12a_phy_write16(G12A_DDR_PHY_MEMRESETL, 2);	/* fffb6244/48 */
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
	/* DctWriteProt (CSR 0xd0031, g12a-ddr-fw.h:211): the IMEM/DCCM
	 * aperture only accepts the fill while write-protect is at 0, so the
	 * whole-window write must sit inside this leg -- the identical bracket
	 * the s9_load msgblock raw fill verifies bad==0 with. Left out, an
	 * off-table part reads the ICCM/DCCM back blank (qi:reszj icd bad).
	 */
	g12a_open_csr_w(0x000d0031u, 1u);
	g12a_open_csr_w(0x000d0031u, 0u);
	for (i = 0; i < n; i++)
		g12a_open_csr_w(csr_base + i,
				readw((void *)(src + 2 * i)));
	g12a_open_csr_w(0x000d0031u, 1u);
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 1);	/* fffb628c */
}

/* fffb6294 */
static void g12a_open_s9_load(void)
{
	/* v3-diff decisive row: MicroReset(bit0=StallToMicro) reads 1 at
	 * the vendor kick entry (site=8d7c 0c0099/0d0099=0001) and 0 on
	 * this board = the product STALLS the micro across the load; we
	 * never did = the micro executed while IMEM/DCCM were still
	 * being written, fetched partially-loaded code and died with no
	 * message. Stall BEFORE the copies; the existing kick pulse
	 * (9->1->0, fffb8df8/8e04/8e10) releases it after the bell,
	 * exactly as the product does.
	 */
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 1);
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_uart_puts("MS1 MS1 ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICRORESET));
	g12a_uart_puts(" 0\n");
#endif
	g12a_phy_write16(G12A_DDR_PHY_MEMRESETL, 2);	/* fffb62b0 */
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
	g12a_open_copy16(G12A_FW_HOLD_1D, 0x8000, 0x50000);
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 1);
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	/* Is the SOURCE window (where S3 built the msgblock) alive at
	 * all when copy16 reads it? CS <w0> <w4> <w8> <w12> <window w0>
	 */
	g12a_uart_puts("CS CS ");
	g12a_uart_hex32(readl((void *)G12A_MSG_BLOCK_BASE));
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)(G12A_MSG_BLOCK_BASE + 4)));
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)(G12A_MSG_BLOCK_BASE + 8)));
	g12a_uart_putc(' ');
	g12a_uart_hex32(readl((void *)(G12A_MSG_BLOCK_BASE + 12)));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x00054000u));
	g12a_uart_puts("\n");
#endif
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	/* CS proved the source alive; MZ proved the 0x54000 window
	 * stayed zero through a plain csr_w copy = raw aperture writes
	 * to the DCT data block are dropped. Bracket the copy in the
	 * DCT write protocol the IMX loader uses (ddrphy_train.c:
	 * DctWriteProt 1 -> 0, writes, 1) and verify.
	 */
	g12a_open_csr_w(0x000d0031u, 1u);
	g12a_open_csr_w(0x000d0031u, 0u);
	{
		u32 i;

		/* 0x54000 dropped both raw and protocol writes with the
		 * micro stalled; 0x50000 (IMEM, same copy) lands. Try
		 * the third window for the DCCM/msgblock half.
		 */
		for (i = 0; i < 0x800u; i++)
			g12a_open_csr_w(0x00058000u + i,
					readw((void *)(G12A_MSG_BLOCK_BASE + 2 * i)));
	}
	g12a_open_csr_w(0x000d0031u, 1u);
	g12a_uart_puts("CV CV ");
	g12a_uart_hex16(g12a_open_csr_r(0x00058000u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x00058001u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x00058002u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x00058003u));
	g12a_uart_puts(" 0\n");
#else
	/* fffb62d8-62e8: the msgblock goes to DCCM at CSR 0x54000, exactly
	 * as the vendor loads it.  0x58000 is not DCCM: staging there left
	 * DCCM unwritten and blanked ICCM word 0, and the micro never posted.
	 */
	g12a_open_copy16(G12A_MSG_BLOCK_BASE, 0x1000, 0x54000);
#endif
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
	/* ICCM word 0, then DCCM msgblock +0x06 DRAMFreq (MT/s, never 0).
	 * DCCM word 0 is Reserved00|MsgMisc = 0 on a GOOD fill, so it cannot
	 * tell a landed msgblock from a dropped one. */
	g12a_uart_puts("OP9 ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_IMEM_BASE));
	g12a_uart_putc('/');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_DMEM_BASE + 6));
	g12a_uart_puts("\n");
}

/* -------------------------------------------------- the UCT mailbox */

/*
 * fffb42fc, with the one change the blob cannot have: a bound.  Return
 * -1 on timeout rather than spinning.
 */
static int g12a_open_uct_read(u32 wide, u32 *out)
{
	u32 t0, v;

	t0 = g12a_open_us();
	while (g12a_open_csr_r(0x0d0004u) & 1u)		/* fffb4304-4318 */
		if (g12a_open_us() - t0 > G12A_OPEN_UCT_TIMEOUT_US) {
			g12a_uart_puts("OPU t1\n");
			return -1;
		}
	v = g12a_open_csr_r(0x0d0032u);			/* fffb4324 */
	if (wide == 1u)					/* fffb4328 */
		v |= (u32)g12a_open_csr_r(0x0d0034u) << 16;	/* fffb433c */
	g12a_open_csr_w(0x0d0031u, 0);			/* fffb434c  ack */
	t0 = g12a_open_us();
	while (!(g12a_open_csr_r(0x0d0004u) & 1u))	/* fffb4354-4368 */
		if (g12a_open_us() - t0 > G12A_OPEN_UCT_TIMEOUT_US) {
			g12a_uart_puts("OPU t2\n");
			return -1;
		}
	g12a_open_csr_w(0x0d0031u, 1);			/* fffb4378 */
	*out = v;
	return 0;
}

/*
 * fffb4448 -- the harvest.  This is where ds[628...] and ds[664...], the
 * two per-DQ-bit vref banks fffb4bc4 consumes, are produced: the PMU
 * streams them as message 0x0014001F with 31 arguments, twice per slot,
 * and the host reduces the pair.
 *
 * TRAPS, both silent:
 *  - the mean is over slots where BOTH members are non-zero AND both are
 *    below the per-bank ceiling, and the divisor is the number of VALUES
 *    (cnt << 1), not of pairs (fffb46cc-4710).
 *  - the FIRST lap (slot index < 32) stores the mean RAW; only the second
 *    lap averages it with what is there (fffb4794 jumps straight to the
 *    strb with no preceding load).  A port that always averages folds lap
 *    0 into stale contents -- and on the second main() call that is the
 *    previous pass's value, so the error is invisible and plausible.
 */
static u8 g12a_open_stg_a[31];
static u8 g12a_open_stg_b[31];
static u8 g12a_open_pair_idx;
static u8 g12a_open_msgc;
static u8 g12a_open_bank;
static u8 g12a_open_marker;
#if G12A_OPEN_ASSERT_ID
static u32 g12a_open_assert_hdr;	/* the header before the 0x04020000 sentinel */
static u32 g12a_open_last_hdr;		/* the last header of any kind */
#endif

static int g12a_open_harvest(u8 *ds)
{
	static u32 args[64];
	u32 hdr, na, i;
	u32 ceil, cnt = 0, sum = 0, avg, idx, w19;

	if (g12a_open_uct_read(1, &hdr))		/* fffb44b4-bc */
		return -1;
	na = hdr & 0xffu;				/* fffb44fc  8-bit */
	for (i = 0; i < na && i < 64; i++)		/* fffb4510-4524 */
		if (g12a_open_uct_read(1, &args[i]))
			return -1;

#if G12A_OPEN_ASSERT_ID
	/*
	 * 1d:0x0170 posts 0x04020000 immediately after 1d:0x0164 has posted the
	 * assert's own (id << 16) | nargs.  Latch the predecessor.
	 */
	if (hdr == 0x04020000u)
		g12a_open_assert_hdr = g12a_open_last_hdr;
	g12a_open_last_hdr = hdr;
#endif
	if (hdr == 0x001e0004u || hdr == 0x00200004u) {	/* fffb456c-458c */
		if (g12a_open_bank) {
			g12a_open_pair_idx = 0;
			g12a_open_bank = 0;
			g12a_open_marker = 0;
		}
		g12a_open_marker++;
	}
	if (hdr == 0x00230004u || hdr == 0x001c0004u) {	/* fffb45bc-45d8 */
		if (!g12a_open_bank) {
			g12a_open_pair_idx = 0;
			g12a_open_bank = 1;
			g12a_open_marker = 0;
		}
		g12a_open_marker++;
	}
#if G12A_OPEN_TRACE_STREAM
	/*
	 * Every stream header the PMU posts, with the harvest state it lands
	 * in.  This is the instrument that separates "the reduction is wrong"
	 * from "the routing is wrong": the vendor's own banks are known
	 * (OPV blob, boot open-cmp-2), so a per-message trace says which.
	 */
	g12a_uart_puts("OPH ");
	g12a_uart_hex32(hdr);
	g12a_uart_putc(' ');
	g12a_uart_hex8(g12a_open_bank);
	g12a_uart_hex8(g12a_open_msgc);
	g12a_uart_hex8(g12a_open_pair_idx);
	g12a_uart_hex8(g12a_open_marker);
	g12a_uart_puts("\n");
#endif
	/*
	 * DEPARTURE, silent when wrong.  `b.ne 0xfffb4820` at fffb464c is a
	 * branch to the function EPILOGUE (fffb4820 is `ldp x19,x20,[sp,#16]`
	 * ... `ret`), so a non-payload header returns WITHOUT touching the
	 * message counter.  The counter therefore numbers ONLY the
	 * 31-argument 0x0014001F messages -- and its low bit is what selects
	 * staging array A vs B.
	 *
	 * uct-mailbox-transcription.md sec.5.3 writes this correctly as
	 * `goto out`; what it does not say is that `out` is the RETURN and
	 * `bump` is not, which is exactly the distinction this depends on.
	 * The first implementation here read it as `goto bump` and was wrong.
	 *
	 * Measured on boot open-live-5: the 2D pass interleaves 128
	 * `0x00150004` messages among the 256 payload ones, exactly one per
	 * pair, so counting every message inverts the A/B parity on every
	 * second reduction and the mean is taken across two different lanes.
	 * That produced ds[628...] uniformly ~11 codes high and ds[664...]
	 * bimodal (+20 on odd slots) against the blob's own banks.
	 */
	if (hdr != 0x0014001fu)				/* fffb4640-464c */
		return 0;

	for (i = 0; i < 31; i++)			/* fffb4674-4698 */
		((g12a_open_msgc & 1u) ? g12a_open_stg_b :
					 g12a_open_stg_a)[i] = (u8)args[i];
	if (!(g12a_open_msgc & 1u))			/* fffb469c */
		goto bump;

	ceil = 0x7fu;					/* fffb4614 */
	if (g12a_open_bank)
		ceil = (ds[14] == 2u) ? 0x50u :
		       (ds[14] == 1u) ? 0x49u : 0x7fu;
	idx = g12a_open_pair_idx++;			/* fffb46a4-b0 */
	for (i = 0; i < 31; i++) {			/* fffb46cc-4704 */
		u32 a = g12a_open_stg_a[i], b = g12a_open_stg_b[i];

		if (!a || !b || a >= ceil || b >= ceil)
			continue;
		cnt = (cnt + 1u) & 0xffu;
		sum += a + b;
	}
	avg = cnt ? (sum / (cnt << 1)) : 0u;		/* fffb470c-4710 */

	w19 = ((u32)ds[15] & 0xfu) ? 16u : 32u;		/* fffb4488-4494 */
	/* fffb4750-4778 -- the production index remap, live when ranks == 2 */
	if (ds[14] == 2u && !(ds[65] & 1u)) {
		if (g12a_open_marker == 2u)
			idx += 0x10u;
		else if (g12a_open_marker == 3u)
			idx -= 0x10u;
		idx &= 0xffu;
	}
	if (idx < w19) {				/* fffb4780-84  RAW */
		ds[(g12a_open_bank ? 664u : 628u) + (idx % w19)] = (u8)avg;
	} else if (idx < 2u * w19) {			/* fffb479c-47a0 */
		u32 o = (g12a_open_bank ? 664u : 628u) + (idx % w19);

		ds[o] = (u8)((ds[o] + avg) >> 1);
	}
bump:
	g12a_open_msgc++;				/* fffb47d8-dc */
	return 0;
}

/*
 * fffb483c -- main().  The 16-id string table is presentation only; the
 * two terminal ids are 0x07 (success) and 0xFF (failure), and 0x08 is
 * the only arm that reads more from the mailbox.
 */
static int g12a_open_mailbox(u8 *ds)
{
	u32 major;
	int i;
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	unsigned long msgs = 0;
#endif

	g12a_open_pair_idx = 0;				/* fffb4864 */
	g12a_open_msgc = 0;				/* fffb4850 */
	g12a_open_bank = 0;				/* fffb4878 */
	g12a_open_marker = 0;				/* fffb4858 */
#if G12A_OPEN_ASSERT_ID
	g12a_open_assert_hdr = 0xffffffffu;		/* never latched */
	g12a_open_last_hdr = 0xffffffffu;
#endif
	for (i = 0; i < 31; i++) {			/* fffb4890/94 */
		g12a_open_stg_a[i] = 0;
		g12a_open_stg_b[i] = 0;
	}

	for (;;) {
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
		if (++msgs > G12A_PROBE_MAILBOX_MAX_MSG) {
			/* part never posts ok/fail: bound + report state */
			g12a_uart_puts("OPT capped\n");
			g12a_uart_puts("PDUMP msgc=");
			g12a_uart_hex16((u16)g12a_open_msgc);
			g12a_uart_puts(" bank=");
			g12a_uart_hex16((u16)g12a_open_bank);
			g12a_uart_puts(" mark=");
			g12a_uart_hex16((u16)g12a_open_marker);
			g12a_uart_puts("\n");
			return -3;
		}
#endif
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
		/* The bound was the bug: uct_read waits 1 s for the
		 * first message; real 1D pass-A training takes minutes
		 * (resc: OPEN pa fe while the shadow held the pending
		 * message seconds later: MB pa 0d0004=0004 bit0=0).
		 * Wait 180 s for bit0 to clear, petting the watchdog,
		 * then uct_read's own bound only guards the handshake.
		 */
		{
			u32 w0 = g12a_open_us();

			while (g12a_open_csr_r(0x0d0004u) & 1u) {
				if ((s32)(g12a_open_us() - w0) < 0 ||
				    g12a_open_us() - w0 > 1200000000u) {
					g12a_uart_puts("MZ t\n");
					return -2;
				}
				if (((g12a_open_us() - w0) % 5000000u) < 2000u) {
					g12a_uart_puts("MZ ");
					g12a_uart_hex32((g12a_open_us() - w0) / 1000000u);
					g12a_uart_putc(' ');
					g12a_uart_hex16(g12a_open_csr_r(0x000c0009u));
					g12a_uart_putc(' ');
					g12a_uart_hex16(g12a_open_csr_r(0x000d0004u));
					g12a_uart_putc(' ');
					g12a_uart_hex32(readl((void *)0xFFFB2210UL));
					g12a_uart_putc(' ');
					g12a_uart_hex16(g12a_open_csr_r(0x000c0032u));
					g12a_uart_putc(' ');
					g12a_uart_hex16(g12a_open_csr_r(0x000c0033u));
					g12a_uart_putc(' ');
					g12a_uart_hex32(readl((void *)(G12A_MSG_BLOCK_BASE + 0x10)));
					g12a_uart_putc(' ');
					g12a_uart_hex16(g12a_open_csr_r(0x000d0000u));
					g12a_uart_putc(' ');
					g12a_uart_hex16(g12a_open_csr_r(0x000d0099u));
					g12a_uart_puts("\n");
				}
				g12a_wdt_pet_safe();
			}
		}
#endif
		if (g12a_open_uct_read(0, &major))
			return -2;
		major &= 0xffffu;
		if (major == 0x07u) {			/* fffb4958 */
			g12a_uart_puts("OPT ok\n");
			return 0;
		}
		if (major == 0xffu) {			/* fffb4a04 */
			g12a_uart_puts("OPT FAIL\n");
#if G12A_OPEN_ASSERT_ID
			g12a_uart_puts("OPT AID ");
			g12a_uart_hex32(g12a_open_assert_hdr);
			g12a_uart_putc(' ');
			g12a_uart_hex32(g12a_open_last_hdr);
			g12a_uart_puts("\n");
#endif
			return 0xff;
		}
		if (major == 0x08u) {			/* fffb4a20 */
			if (g12a_open_harvest(ds))
				return -2;
			continue;
		}
		g12a_uart_puts("OPT ");			/* stage marker */
		g12a_uart_hex16((u16)major);
		g12a_uart_puts("\n");
	}
}

/* fffb8dd4 -- release the micro and run the mailbox. */
static int g12a_open_train_kick(u8 *ds)
{
	int r;

#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	/* APB owns the PHY SRAMs while MuxSel=0: self-test the host write
	 * path and read IMEM/msgblock/DMEM back. A zero msgblock word HERE
	 * (after s9's copy16) means the load never landed; a nonzero that
	 * later reads zero at MuxSel=1 is simple ownership, not a dead
	 * clock -- that distinction is dwc-micro-start-contract G1.
	 */
	writel(0x5A5A0001u, (void *)0xFFFB2210UL);
	writel(0x5A5A0002u, (void *)0xFFFB2214UL);
	writel(0x5A5A0003u, (void *)0xFFFB2218UL);
	writel(0x5A5A0004u, (void *)0xFFFB221CUL);
	/* Load-integrity (Implication 3, measured on silicon): sum every
	 * halfword of the staged 1d image in host SRAM and of the same
	 * region as it now lives in the PHY, ICCM csr 0x50000+ and DCCM
	 * csr 0x54000+. A mismatch names a corrupted/misplaced load --
	 * the last host-side bug class left. ~0x4800 csr reads ~= 50 ms.
	 */
	{
		u32 hi = 0, hd = 0, si = 0, sd = 0, j;

		for (j = 0; j < 0x4000u; j++) {
			hi += g12a_open_csr_r(0x50000u + j);
			si += (u32)readw((void *)(G12A_FW_HOLD_1D + 2u * j));
		}
		for (j = 0; j < 0x800u; j++) {
			hd += g12a_open_csr_r(0x54000u + j);
			sd += (u32)readw((void *)(G12A_MSG_BLOCK_BASE + 2u * j));
		}
		for (j = 0; j < 3; j++) {
			g12a_uart_puts("KI ");
			g12a_uart_hex32(si);
			g12a_uart_putc(' ');
			g12a_uart_hex32(hi);
			g12a_uart_puts("\n");
			g12a_uart_puts("KD ");
			g12a_uart_hex32(sd);
			g12a_uart_putc(' ');
			g12a_uart_hex32(hd);
			g12a_uart_puts("\n");
		}
		/* localize: first/last differing halfword, count, and the
		 * 3 words around the FIRST one (DCCM window).
		 */
		{
			u32 cnt = 0, first = 0xFFFFFFFFu, last = 0;

			for (j = 0; j < 0x800u; j++) {
				u32 a = g12a_open_csr_r(0x54000u + j);
				u32 b = (u32)readw((void *)(G12A_MSG_BLOCK_BASE +
							     2u * j));

				if (a != b) {
					if (first == 0xFFFFFFFFu)
						first = j;
					last = j;
					cnt++;
				}
			}
			for (j = 0; j < 2; j++) {
				g12a_uart_puts("KF ");
				g12a_uart_hex32(first * 2u);
				g12a_uart_putc(' ');
				g12a_uart_hex32(last * 2u);
				g12a_uart_putc(' ');
				g12a_uart_hex32(cnt);
				g12a_uart_puts("\n");
				g12a_uart_puts("KW ");
				if (first != 0xFFFFFFFFu) {
					u32 k;

					for (k = (first ? first - 1u : 0u); k < first + 3u; k++) {
						g12a_uart_hex32(g12a_open_csr_r(0x54000u + k));
						g12a_uart_putc(' ');
					}
					g12a_uart_puts("| ");
					for (k = (first ? first - 1u : 0u); k < first + 3u; k++) {
						g12a_uart_hex32((u32)readw((void *)(G12A_MSG_BLOCK_BASE +
										     2u * k)));
						g12a_uart_putc(' ');
					}
				}
				g12a_uart_puts("\n");
			}
		}
	}
	for (u32 k = 0; k < 3; k++) {
		u32 stop = g12a_open_us() + 3000u;

		g12a_uart_puts("ZA ");
		g12a_uart_hex32(readl((void *)0xFFFB2210UL));
		g12a_uart_putc(' ');
		g12a_uart_hex32(g12a_open_csr_r(0x50000u));
		g12a_uart_putc(' ');
		g12a_uart_hex32(g12a_open_csr_r(0x54000u));
		g12a_uart_putc(' ');
		g12a_uart_hex32(g12a_open_csr_r(0x58000u));
		g12a_uart_puts("\n");
		while ((s32)(g12a_open_us() - stop) < 0)
			;
	}
#endif
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 1);	/* fffb8de8 */
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 9);		/* fffb8df8 */
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_uart_puts("Qk ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICRORESET));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MEMRESETL));
	g12a_uart_puts("\n");
#endif
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 1);		/* fffb8e04 */
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 0);		/* fffb8e10 */
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	/* ring the doorbell AFTER the release -- if the product rings
	 * after (8d7c shows the stall held at kick ENTRY, bell status
	 * unknown), ours rung the bell into a held-in-reset micro whose
	 * reset pulse then cleared it. Crumb: ZK
	 */
	g12a_open_csr_w(0x054008u, 0x1001u);
	g12a_uart_puts("ZK ");
	g12a_uart_hex32(g12a_open_csr_r(0x054008u));
	g12a_uart_puts("\n");
#endif
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	/* qi:N1294: read the release state back before the first wait --
	 * the transcript's CSR capture tagged MICROCONTMUXSEL write=dead.
	 */
	g12a_uart_puts("Q ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICROCONTMUXSEL));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICRORESET));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MEMRESETL));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x0d0004u));
	g12a_uart_puts("\n");
	/* Console drops ~half of a boot-burst's lines; stash the same
	 * readings + the micro-start contract quad in proven-free SRAM
	 * for the host to readSimpleMemory after the boot (same window
	 * the --cfg-blob overlay uses, slot +0x200).
	 */
	writel(0xD05E0001u, (void *)0xFFFB2200UL);
	writel(g12a_phy_read16(G12A_DDR_PHY_MICROCONTMUXSEL),
	       (void *)0xFFFB2204UL);
	writel(g12a_phy_read16(G12A_DDR_PHY_MICRORESET),
	       (void *)0xFFFB2208UL);
	writel(g12a_phy_read16(G12A_DDR_PHY_MEMRESETL),
	       (void *)0xFFFB220CUL);
	writel(g12a_open_csr_r(0x0d0004u), (void *)0xFFFB2210UL);
	writel(g12a_open_csr_r(0x50000u), (void *)0xFFFB2214UL);
	writel(g12a_open_csr_r(0x54000u), (void *)0xFFFB2218UL);
	writel(g12a_open_csr_r(0x58000u), (void *)0xFFFB221CUL);
	/* Console eats ~half of a burst; print the 4-word quad five times
	 * with ~3 ms spacing so any surviving copy answers the G1 question.
	 */
	for (u32 i = 0; i < 5; i++) {
		u32 stop = g12a_open_us() + 3000u;

		g12a_uart_puts("Z ");
		g12a_uart_hex32(readl((void *)0xFFFB2210UL));
		g12a_uart_putc(' ');
		g12a_uart_hex32(readl((void *)0xFFFB2214UL));
		g12a_uart_putc(' ');
		g12a_uart_hex32(readl((void *)0xFFFB2218UL));
		g12a_uart_putc(' ');
		g12a_uart_hex32(readl((void *)0xFFFB221CUL));
		g12a_uart_puts("\n");
		while ((s32)(g12a_open_us() - stop) < 0)
			;
	}
#endif
	r = g12a_open_mailbox(ds);
	if (r == 0) {						/* fffb8e3c */
		g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 1);
		g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
	}
	g12a_wdt_pet_safe();
	return r;
}

#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
/*
 * (b)+(c) bounded open 1D train + a register dump on every non-OK
 * terminal, so a boot on an off-table part returns a SIGNAL instead of
 * the silent dead-stop the blob's unbounded mailbox poll produces.
 * Reads only the addresses this file already uses; no new register.
 */
static void g12a_probe_dump(const char *why)
{
	ulong ds = G12A_DDR_SET_BASE;
	u32 i;

	g12a_uart_puts("PDUMP ");
	g12a_uart_puts(why);
	g12a_uart_puts("\n");

	g12a_uart_puts("pd dmc  ");
	for (i = 0; i < 8; i++) {
		g12a_uart_hex32(readl((void *)(0xFF638000UL + i * 4)));
		g12a_uart_putc(' ');
	}
	g12a_uart_puts("\n");

	g12a_uart_puts("pd uct  ");
	g12a_uart_hex32(g12a_open_csr_r(0x0d0004u));
	g12a_uart_putc('/');
	g12a_uart_hex32(g12a_open_csr_r(0x0d0032u));
	g12a_uart_putc('/');
	g12a_uart_hex32(g12a_open_csr_r(0x0d0034u));
	g12a_uart_puts("\n");

	g12a_uart_puts("pd phy  ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICRORESET));
	g12a_uart_putc('/');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICROCONTMUXSEL));
	g12a_uart_putc('/');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_UCCLKHCLKENABLES));
	g12a_uart_puts("\n");

	/* (c) the geometry the driver actually applied */
	g12a_uart_puts("pd ds   ");
	g12a_uart_hex8(readb((void *)(ds + G12A_DS_OFF_DRAM_TYPE)));
	g12a_uart_putc('/');
	g12a_uart_hex8(readb((void *)(ds + G12A_DS_OFF_RANK_CFG)));
	g12a_uart_putc('/');
	g12a_uart_hex8(readb((void *)(ds + G12A_DS_OFF_RFC_TYPE)));
	g12a_uart_putc('/');
	g12a_uart_hex8(readb((void *)(ds + G12A_DS_OFF_LPDDR4X)));
	g12a_uart_putc('/');
	g12a_uart_hex32(readl((void *)(ds + G12A_DS_OFF_DDR_FUNC)));
	g12a_uart_puts("\n");

	/* per-lane harvested vref bank (PIEI result, host-side) */
	g12a_uart_puts("pd vref ");
	for (i = 0; i < 16; i++) {
		g12a_uart_hex8(readb((void *)(ds + 628u + i)));
		g12a_uart_putc(' ');
	}
	g12a_uart_puts("\n");

	g12a_uart_puts("pd msg  ");
	for (i = 0; i < 4; i++) {
		g12a_uart_hex32(readl((void *)(G12A_MSG_BLOCK_BASE + i * 4)));
		g12a_uart_putc(' ');
	}
	g12a_uart_puts("\n");
}

static int g12a_probe_train(u8 *ds)
{
	u32 i, major;
	int r;

	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 1);
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 9);
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 1);
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 0);

	for (i = 0; i < G12A_PROBE_MAILBOX_MAX_MSG; i++) {
		r = g12a_open_uct_read(0, &major);
		if (r) {
			g12a_uart_puts("PT opu\n");
			g12a_probe_dump("opu");
			return -2;
		}
		major &= 0xffffu;
		if (major == 0x07u) {
			g12a_uart_puts("PT ok\n");
			g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 1);
			g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
			return 0;
		}
		if (major == 0xffu) {
			g12a_uart_puts("PT fail\n");
			g12a_probe_dump("fail");
			return 0xff;
		}
		if (i < 16 || !(i & 0xff)) {
			g12a_uart_puts("pt ");
			g12a_uart_hex16((u16)major);
			g12a_uart_putc(' ');
			g12a_uart_hex32(i);
			g12a_uart_puts("\n");
		}
		if (major == 0x08u)
			g12a_open_harvest(ds);
		g12a_wdt_pet_safe();
	}
	g12a_uart_puts("PT capped\n");
	g12a_probe_dump("capped");
	return -3;
}
#endif /* CONFIG_MESON_G12A_DRAM_PROBE */

#if G12A_OPEN_REPLAY
/* fffbb238 -- one MRS through the DMC.  Defined with the T14 block below. */
static void g12a_open_mrs(ulong ctl, const u8 *ds, u32 rank, u32 mr, u32 val);

/*
 * ==================== T5a / T8: the delay-class save/restore ====================
 *
 * Transcription of aml_ddr.fw fffb502c (650 instructions, one ret) and its
 * driver fffb5a54 (92 instructions).  Everything below is from
 * amlogic/software/boot/g12/class-restore-transcription.md sec.3-sec.6; that page
 * records ZERO bytes to copy verbatim -- every CSR address and every ds offset
 * is an instruction-stream immediate, there is no rodata, no table and no
 * arithmetic that can round differently.
 *
 * dir == 0 is RESTORE (memory -> CSR), dir == 1 is SAVE (CSR -> memory)
 * (fffb5350 / fffb545c / fffb54f8 / fffb57dc all `cmp w23, #1`).  a644 mode A
 * calls restore at fffbea84 and save at fffbea9c.
 *
 * PHY CSR address rule: PA = 0xFE000000 + (csr << 1), all accesses 16-bit,
 * csr = (pstate << 20) | (block << 16) | (inst << 12) | off, and for DBYTE
 * per-lane registers off = base + lane * 0x100.  g12a_open_csr_{r,w} apply it.
 *
 * SIX OF THE ELEVEN DEPARTURES ARE SILENT WHEN WRONG.  Each is implemented as
 * a departure and marked with its number; do not "symmetrise" any of them.
 */

/*
 * Classes 1..6: the csr for element i.  sec.3.2, corrected table -- all
 * formulas in CSR units (the previously published class-1 row was in PA-byte
 * units, departure D6).
 */
static u32 g12a_open_cls_csr(u32 cls, u32 ps, u32 i)
{
	switch (cls) {
	case 1:		/* ANIB ATxDly, 10 -- fffb5068 / fffb50e4 / fffb50e8 */
		return ps | (i << 12) | 0x080u;
	case 2:		/* DBYTE 0x0D0 lane-paired, 16 -- fffb50f8-5118 */
		return ps | 0x10000u | (((i >> 1) & 3u) << 12) |
		       (0xD0u + (i >> 3) + ((i & 1u) << 8));
	case 3:		/* DBYTE RxClkDlyTg0, 16 -- fffb51a8-51cc */
		return ps | 0x10000u | (((i >> 1) & 3u) << 12) |
		       (0x8Cu + (i >> 3) + ((i & 1u) << 8));
	case 4:		/* DBYTE TxDqDly, 72 = 2 rank x 4 DBYTE x 9 lane */
		return ps | 0x10000u | (((i % 36u) / 9u) << 12) |
		       (0xC0u + i / 36u + ((i % 9u) << 8));	/* fffb51dc-5210 */
	case 5:		/* DBYTE RxPBDly, 72 -- fffb5124-5158 */
		return ps | 0x10000u | (((i % 36u) / 9u) << 12) |
		       (0x68u + i / 36u + ((i % 9u) << 8));
	default:	/* 6 -- DBYTE RxEnDlyTg0, 16 -- fffb5174-5198 */
		return ps | 0x10000u | (((i >> 1) & 3u) << 12) |
		       (0x80u + (i >> 3) + ((i & 1u) << 8));
	}
}

/**
 * g12a_open_cls - fffb502c(ctl, cls, dir, idx, buf)
 * @ds:   the ddr_set_t (fffb502c reads it as *(ctl + 0xA0))
 * @cls:  delay class 1..6, 8, 9, 11, 12
 * @dir:  0 = RESTORE (memory -> CSR), 1 = SAVE
 * @idx:  pstate; also the CSR address's pstate field (fffb504c lsl #20)
 * @buf:  backing store for classes 1..6 and 9; NULL for 8, 11, 12, which
 *        address ds directly
 */
static void g12a_open_cls(u8 *ds, u32 cls, u32 dir, u32 idx, u8 *buf)
{
	/* D8: element widths.  cls1 u8, cls2 u16, cls3 u8, cls4 u16 (NOT u8 --
	 * getting this wrong shifts the whole ds+452..595 array and misaligns
	 * class 6 behind it), cls5 u8, cls6 u16. */
	static const u8 cls_cnt[7] = { 0, 10, 16, 16, 72, 72, 16 };
	static const u8 cls_wid[7] = { 0,  1,  2,  1,  2,  1,  2 };
	u32 ps = idx << 20;
	u32 i, csr, v, raw;
	u8 *p = buf;
	u8 *d;

	/*
	 * D9: fffb5058-505c writes APBONLY MicroContMuxSel (csr 0xD0000, PA
	 * 0xFE1A0000) = 0 on entry, and NOTHING in the 650 instructions
	 * re-arms it.  That is intended -- txdqdly-deskew.md sec.4 relies on it
	 * -- so it is not "fixed" here either.
	 */
	g12a_open_csr_w(0xD0000u, 0);

	if (cls >= 1u && cls <= 6u) {
		u32 n = cls_cnt[cls], w = cls_wid[cls];

		for (i = 0; i < n; i++) {
			csr = g12a_open_cls_csr(cls, ps, i);
			if (dir == 1u) {
				/* decode on save -- fffb5268 bl fffba174 */
				v = g12a_open_dly_dec(cls,
						      g12a_open_csr_r(csr));
				p[0] = (u8)v;
				if (w == 2u)
					p[1] = (u8)(v >> 8);
			} else {
				v = (w == 1u) ? (u32)p[0]
					      : ((u32)p[0] | ((u32)p[1] << 8));
				/* encode on restore -- fffb52e0 bl fffba1e0 */
				raw = g12a_open_dly_enc(cls, v);
				g12a_open_csr_w(csr, raw);
				/*
				 * Class 3 restore ALSO writes csr + 4 (Tg1),
				 * fffb530c-5314 gated by fffb52f0 cmp w21,#3.
				 * The save side reads Tg0 only.
				 */
				if (cls == 3u)
					g12a_open_csr_w(csr + 4u, raw);
			}
			p += w;
		}
		return;
	}

	if (cls == 8u) {			/* fffb5348-542c */
		d = ds + idx * 0x1ACu;
		if (dir == 1u) {
			/*
			 * D2: fffb5364's `bfi w0, w23, #16, #16` builds the
			 * block selector out of `dir`, and is correct only
			 * because dir == 1 is the only way to reach it.
			 * Transcribed as the CONSTANT 1 (DBYTE), not as dir.
			 */
			d[697] = (u8)g12a_open_csr_r(ps | 0x10020u);
			d[698] = (u8)g12a_open_csr_r(ps | 0x20020u);
		} else {
			/*
			 * D1: 2 reads, 5 writes.  The PHY requires DFIMRL to
			 * be identical across DBYTEs, so the ONE saved byte
			 * fans out to all four.  A symmetric restore leaves
			 * DBYTE1-3 at their reset DFIMRL -- which still reads
			 * at nominal margin and fails on a corner.
			 */
			for (i = 0; i < 4u; i++)
				g12a_open_csr_w(ps | 0x10020u | (i << 12),
						d[697]);
			g12a_open_csr_w(ps | 0x20020u, d[698]);
		}
		return;
	}

	if (cls == 9u) {			/* fffb5430-54e8, VrefDAC0 */
		if (dir == 1u) {
			for (i = 0; i < 32u; i++)
				p[i] = (u8)g12a_open_csr_r(ps | 0x10040u |
							   ((i >> 3) << 12) |
							   ((i & 7u) << 8));
			/*
			 * D5: four lane-8 reads with NO restore counterpart --
			 * ds+660..663 is write-only and lane 8's true value is
			 * discarded.  fffb54b4-54d8, guarded for dir != 1.
			 */
			for (i = 0; i < 4u; i++)
				p[32 + i] = (u8)g12a_open_csr_r(ps | 0x10840u |
								(i << 12));
		} else {
			for (i = 0; i < 32u; i++) {
				csr = ps | 0x10040u | ((i >> 3) << 12) |
				      ((i & 7u) << 8);
				g12a_open_csr_w(csr, p[i]);
				/* D5: lane 8 is DERIVED from lane 7, not
				 * restored from p[32..35].  fffb5488-549c. */
				if ((i & 7u) == 7u)
					g12a_open_csr_w(csr + 0x100u, p[i]);
			}
		}
		return;
	}

	if (cls == 11u) {			/* fffb54ec-57cc, symmetric */
		/*
		 * D11: class 11 takes ds as *(ctl+0xA0) with NO idx*0x1AC
		 * stride (fffb5500), and the driver only ever calls it with
		 * idx = 0 (fffb5bb8), so ps = 0 too.  Looping it over pstates
		 * would alias every pstate onto ds+248.
		 */
		static const u32 s_csr[6] = { 0x200CBu, 0x20077u, 0x20072u,
					      0x20073u, 0x400FDu, 0x400C0u };
		static const u16 s_off[6] = { 248u, 274u, 276u, 278u,
					      280u, 282u };
		static const u32 t_csr[3] = { 0x100AAu, 0x10062u, 0x10001u };
		static const u16 t_off[3] = { 250u, 258u, 266u };
		u32 k;

		for (k = 0; k < 6u; k++) {		/* the six scalars */
			d = ds + s_off[k];
			if (dir == 1u) {
				v = g12a_open_csr_r(ps | s_csr[k]);
				d[0] = (u8)v;
				d[1] = (u8)(v >> 8);
			} else {
				g12a_open_csr_w(ps | s_csr[k],
						(u32)d[0] | ((u32)d[1] << 8));
			}
		}
		for (k = 0; k < 3u; k++)		/* PptCtlStatic /
							 * TrainingIncDecDtsmEn /
							 * TsmByte0, i = 0..3 */
			for (i = 0; i < 4u; i++) {
				d = ds + t_off[k] + 2u * i;
				csr = ps | t_csr[k] | (i << 12);
				if (dir == 1u) {
					v = g12a_open_csr_r(csr);
					d[0] = (u8)v;
					d[1] = (u8)(v >> 8);
				} else {
					g12a_open_csr_w(csr, (u32)d[0] |
							     ((u32)d[1] << 8));
				}
			}
		for (i = 0; i < 32u; i++) {		/* VrefDAC1 lanes 0-7 */
			csr = ps | 0x10030u | ((i >> 3) << 12) |
			      ((i & 7u) << 8);
			if (dir == 1u)
				ds[284 + i] = (u8)g12a_open_csr_r(csr);
			else
				g12a_open_csr_w(csr, ds[284 + i]);
		}
		for (i = 0; i < 4u; i++) {		/* VrefDAC1 lane 8 */
			csr = ps | 0x10830u | (i << 12);
			if (dir == 1u)
				ds[316 + i] = (u8)g12a_open_csr_r(csr);
			else
				g12a_open_csr_w(csr, ds[316 + i]);
		}
		return;
	}

	if (cls == 12u) {			/* fffb57d0-5a34 */
		static const u32 q_csr[3] = { 0x10043u, 0x100AEu, 0x100AFu };
		static const u16 q_off[3] = { 702u, 710u, 718u };
		u32 k;

		d = ds + idx * 0x1ACu;
		/*
		 * D4: ARdPtrInitVal is saved/restored through a u8 while its
		 * three ds neighbours are u16, and MASTER 0x2E is a 16-bit CSR
		 * (fffb5800 strb / fffb5918 ldrb).  Kept as the blob has it.
		 */
		if (dir == 1u)
			d[699] = (u8)g12a_open_csr_r(ps | 0x2002Eu);
		else
			g12a_open_csr_w(ps | 0x2002Eu, d[699]);
		{
			static const u32 m_csr[3] = { 0x200B2u, 0x2007Cu,
						      0x2007Du };
			static const u16 m_off[3] = { 700u, 744u, 746u };

			for (k = 0; k < 3u; k++) {
				u8 *e = d + m_off[k];

				if (dir == 1u) {
					v = g12a_open_csr_r(ps | m_csr[k]);
					e[0] = (u8)v;
					e[1] = (u8)(v >> 8);
				} else {
					g12a_open_csr_w(ps | m_csr[k],
						(u32)e[0] | ((u32)e[1] << 8));
				}
			}
		}
		for (k = 0; k < 3u; k++)
			for (i = 0; i < 4u; i++) {
				u8 *e = d + q_off[k] + 2u * i;

				csr = ps | q_csr[k] | (i << 12);
				if (dir == 1u) {
					v = g12a_open_csr_r(csr);
					e[0] = (u8)v;
					e[1] = (u8)(v >> 8);
					continue;
				}
				v = (u32)e[0] | ((u32)e[1] << 8);
				g12a_open_csr_w(csr, v);
				/*
				 * D3: the RESTORE also writes the lane-1 alias
				 * 0x10143 from the single saved u16; the SAVE
				 * reads lane 0 only.  fffb59a0-59b4, four extra
				 * writes with no read counterpart.  Dropping
				 * them leaves lane 1's receiver control stale.
				 */
				if (k == 0u)
					g12a_open_csr_w(ps | 0x10143u |
							(i << 12), v);
			}
		for (i = 0; i < 8u; i++) {		/* Seq0BGPR1..8 */
			u8 *e = d + 728u + 2u * i;

			csr = ps | 0x90000u | (0x201u + i);
			if (dir == 1u) {
				v = g12a_open_csr_r(csr);
				e[0] = (u8)v;
				e[1] = (u8)(v >> 8);
			} else {
				g12a_open_csr_w(csr,
						(u32)e[0] | ((u32)e[1] << 8));
			}
		}
		return;
	}
}

/**
 * g12a_open_class_rw - fffb5a54(ctl, dir), the driver over fffb502c
 * @dir: 0 = T5a RESTORE (fffbea84), 1 = T8 CAPTURE (fffbea9c)
 *
 * sec.3.3: for n = 0, 1 it calls fffb502c(ctl, cls, dir, n, ds + n*0x1AC + off)
 * for classes 1, 2, 3, 4, 5, 6, 8 (NULL), 9, 12 (NULL), then tail-calls
 * fffb502c(ctl, 11, dir, 0, NULL).
 *
 * The ds tiling is the independent check on the counts: with 72 for classes 4
 * and 5 the n = 0 slice runs 320..746 with no overlap and no gap, the stride is
 * 0x1AC = 428, and 428 + 748 = 1176 -- the struct is EXACTLY two pstates with
 * no slack.  With 36 entries class 4 would end at 523 and leave a 72-byte hole.
 */
static void g12a_open_class_rw(u8 *ds, u32 dir)
{
	static const u8 drv_cls[9] = { 1u, 2u, 3u, 4u, 5u, 6u, 8u, 9u, 12u };
	static const u16 drv_off[9] = { 0x140u, 0x1A4u, 0x14Cu, 0x1C4u, 0x15Cu,
					0x254u, 0xFFFFu, 0x274u, 0xFFFFu };
	u32 n, k;

	for (n = 0; n < 2u; n++) {
		for (k = 0; k < 9u; k++) {
			u8 *buf = (drv_off[k] == 0xFFFFu) ? NULL :
				  (ds + n * 0x1ACu + drv_off[k]);

			g12a_open_cls(ds, drv_cls[k], dir, n, buf);
		}
		g12a_wdt_pet_safe();
	}
	g12a_open_cls(ds, 11u, dir, 0u, NULL);		/* fffb5bb4-5bc0 */
}

/* Word-sum of ds[248..1175], the delay/vref tail -- the replay discriminator. */
static u32 g12a_open_ds_tailsum(const u8 *ds)
{
	u32 i, s = 0;

	for (i = 248; i < G12A_DDR_SET_SIZE; i += 4)
		s += (u32)ds[i] | ((u32)ds[i + 1] << 8) |
		     ((u32)ds[i + 2] << 16) | ((u32)ds[i + 3] << 24);
	return s;
}

/**
 * g12a_open_mr_init - fffbb37c, the host-side DRAM mode-register init
 *
 * The mode-A substitute for the PMU: when training runs, the micro issues the
 * MRs out of the message block, so the host never does.  In mode A nothing
 * loads the micro, so the host must.  Called from inside T10 at fffb8c6c, the
 * function's ONE caller in the whole blob, on the ctl[612] == 0xFF arm only.
 *
 * amlogic/software/boot/g12/mr-vref-transcription.md sec.3.
 */
static void g12a_open_mr_init(ulong ctl, const u8 *ds)
{
	u8 mr[12];
	u32 rank, ranks, n = 0;

	/*
	 * fffbb394 tbnz w21,#0: arg1 bit 0 SKIPS the reset window.  arg1 comes
	 * from fffb8c5c (ctl[615]); ours is 0, so the window runs -- and in
	 * mode A it is the ONLY DRAM reset, because fffb6294's own
	 * MemResetL = 2 (fffb62a8/fffb62b0, departure D5) is on the mode-B
	 * loader path that never executes here.
	 *
	 * 200 and 2010 are JEDEC LPDDR4 tINIT1 (>= 200 us, RESET_n low) and
	 * tINIT3 (>= 2 ms after it rises), to the digit -- which is what pins
	 * fffb4134 as a MICROsecond delay.
	 */
	if (!readb((void *)(ctl + 615))) {
		g12a_open_csr_w(0x020060u, 2);	/* fffbb3a4 MemResetL := 2 */
		g12a_open_delay_us(200);	/* fffbb3a8 tINIT1 */
		g12a_open_csr_w(0x020060u, 3);	/* fffbb3bc MemResetL := 3 */
		g12a_open_delay_us(2010);	/* fffbb3c0 tINIT3 */
	}

	/*
	 * fffbb3c8-fffbb3fc: the DFI-ODT poke, taken only when BOTH bit 0 and
	 * bit 8 of ds[52] are set.  This part's captured ds[52] is 0x00, so it
	 * is not taken -- and its source word tm[4*(0x60 + ctl[441])] lies past
	 * the 64-word tm this TPL carries, so the arm is REFUSED loudly rather
	 * than approximated.
	 */
	if ((ds[52] & 0x101u) == 0x101u) {
		g12a_uart_puts("RPm odt-refused\n");
		return;
	}

	/*
	 * The MR values.  The blob reads a packed u32 mr[15] at
	 * tm + 256 + 64*pstate; g12a_open_mr_block() is the transcription of
	 * fffb7cf8-fffb7e60, the code that FILLS those slots, so it is the same
	 * values one hop upstream -- and it is already the source the 37/37
	 * message-block prepare uses.  Our timing engine deliberately does not
	 * build tm[256..], so there is nothing there to read.
	 */
	if (g12a_open_mr_block(ds, 0, mr)) {
		g12a_uart_puts("RPm mr-refused\n");
		return;
	}
	/* fffbb400-410 csinc: 2 ranks iff ds+42 is non-zero. */
	ranks = g12a_open_h(ds, 42) ? 2u : 1u;

	for (rank = 0; rank < ranks; rank++) {
		/*
		 * LPDDR4 emission order, ctl[440] == 2 arm:
		 * MR13 (bit 3 forced, fffbb510/fffbb51c), MR3, MR4, MR11,
		 * MR12, MR16, MR17, MR14, MR22 -- then the unconditional tail
		 * MR0, MR1, MR2.
		 * g12a_open_mr_block() order: mr[0]=MR1 mr[1]=MR2 mr[2]=MR3
		 * mr[3]=MR4 mr[4]=MR11 mr[5]=MR12 mr[6]=MR13 mr[7]=MR14
		 * mr[8]=MR16 mr[9]=MR17 mr[10]=MR22 mr[11]=MR24.
		 */
		g12a_open_mrs(ctl, ds, rank, 13, mr[6] | 0x8u);	n++;
		g12a_open_mrs(ctl, ds, rank,  3, mr[2]);	n++;
		g12a_open_mrs(ctl, ds, rank,  4, mr[3]);	n++;
		g12a_open_mrs(ctl, ds, rank, 11, mr[4]);	n++;
		g12a_open_mrs(ctl, ds, rank, 12, mr[5]);	n++;
		g12a_open_mrs(ctl, ds, rank, 16, mr[8]);	n++;
		g12a_open_mrs(ctl, ds, rank, 17, mr[9]);	n++;
		g12a_open_mrs(ctl, ds, rank, 14, mr[7]);	n++;
		g12a_open_mrs(ctl, ds, rank, 22, mr[10]);	n++;
#if G12A_OPEN_MR0
		/*
		 * Departure D2, OFF by default: fffbb638/fffbb644 write
		 * MR0 | 0x100 for every DRAM type, but LPDDR4's MR0 is
		 * READ-ONLY device information and bit 8 is DDR3's DLL-reset.
		 * The tm[256] slot our transcription does not build would be
		 * the only other operand, so 0x100 is the forced bit alone.
		 */
		g12a_open_mrs(ctl, ds, rank, 0, 0x100u);	n++;
#endif
		g12a_open_mrs(ctl, ds, rank,  1, mr[0]);	n++;
		g12a_open_mrs(ctl, ds, rank,  2, mr[1]);	n++;
		g12a_wdt_pet_safe();
	}

	g12a_uart_puts("RPm ");
	g12a_uart_hex8((u8)n);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)ranks);
	g12a_uart_putc(' ');
	g12a_uart_hex8(mr[6] | 0x8u);
	g12a_uart_putc(' ');
	g12a_uart_hex8(mr[7]);
	g12a_uart_puts("\n");
}
#endif	/* G12A_OPEN_REPLAY */

/* ---------------------------------------------------------- S11 / S12 */

static void g12a_open_s11s12_acdelay(const u8 *ds)
{
	u32 i, j;

	for (i = 0; i < 10; i++)			/* fffbe748-e77c */
		if (ds[108 + i])
			g12a_open_csr_w((i << 12) | 0x80u,
					g12a_open_dly_enc(1, ds[108 + i]));
	for (i = 1; i < 11u; i += 5) {			/* fffbe788-e7fc, i in {1,6} */
		if (g12a_open_csr_r((i << 12) | 0x80u))
			continue;
		for (j = 0; j < 5; j++) {
			u32 c = ((i - 1u + j) << 12) | 0x80u;
			u32 v = g12a_open_dly_dec(1, g12a_open_csr_r(c));

			g12a_open_csr_w(c, g12a_open_dly_enc(1, v + 32u));
		}
	}
}

/* --------------------------------------------------------------- S17a */

/*
 * fffbe8b8-fffbe9dc, gated on ds[8] bit 26 == 0 (fffbe888 tbz).  The
 * shipped ACS has ds[8] = 0x80000000, so this DOES run.  The `dec == 32`
 * clamp and the `base + fine - coarse` recentre are real logic, and the
 * uxth truncations at fffbe994 / fffbe99c are load-bearing.
 */
static void g12a_open_s17a_dbyte_fixup(void)
{
	u32 dec, base, n, k;

	dec = g12a_open_dly_dec(1, g12a_open_csr_r(0x001080u));	/* fffbe868-74 */
	base = (u32)g12a_open_csr_r(0x0100aeu);			/* fffbe85c */
	base = (base + 0x10u) & 0xffffu;			/* fffbe8cc */

	for (n = 0; n < 16; n++) {
		u32 inst = (n >> 1) & 3u;			/* fffbe8d0 */
		u32 hi = n >> 3;				/* fffbe8d4 */
		u32 blk = (inst << 12) | 0x10000u;
		u32 c0 = blk | (hi + 0xd0u + ((n & 1u) << 8));
		u32 raw = g12a_open_csr_r(c0);
		u32 coarse;

		if (dec == 32u) {				/* fffbe910 */
			u32 v = g12a_open_dly_dec(2, raw) & 0x3fu;

			if (v >= 0x3du)				/* fffbe920 csel */
				v = 0x20u;
			g12a_open_csr_w(c0,
					g12a_open_dly_enc(2, v + 0x80u));
		}
		raw = g12a_open_csr_r(c0);			/* fffbe944 */
		coarse = raw & 0xffc0u;				/* fffbe950 */
		for (k = 0; k < 9; k++) {			/* fffbe954-e9c8 */
			u32 c1 = blk | (hi + (k << 8) + 0xc0u);
			u32 v = (base + g12a_open_dly_dec(2, raw)) & 0xffffu;

			v = (v - g12a_open_dly_dec(2, coarse)) & 0xffffu;
			g12a_open_csr_w(c1, g12a_open_dly_enc(4, v));
		}
	}
}

/* ----------------------------------------------------------------- T7 */

/*
 * fffb5bc4 -- Amlogic's port of Synopsys
 * dwc_ddrphy_phyinit_I_loadPIEImage.  All 17 items; the sub-arms that
 * ctl[440] == 2 makes dead are named rather than transcribed.
 */
static void g12a_open_t7(ulong ctl, const u8 *ds)
{
	u32 i, m, mx;
	u32 d68 = g12a_open_h(ds, 68) | ((u32)ds[70] << 16) |
		  ((u32)ds[71] << 24);

	if (d68) {						/* item 1 */
		writel(0x0a691c00u, (void *)0xFF638C0CUL);	/* fffb5bf0 */
		writel(d68, (void *)0xFF638C08UL);		/* fffb5c00 */
	}
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);	/* item 2 */

	/*
	 * item 3 -- fffba4bc, gated on (s8)ctl[456] < 0 (fffba4d4 ldrsb /
	 * fffba4d8 tbz w0,#31).  This TPL leaves ctl[456] zero, so the gate
	 * is not taken.  The crumb makes that an observation rather than an
	 * assumption.
	 */
	g12a_uart_puts("OP7 c456 ");
	g12a_uart_hex8(readb((void *)(ctl + 456)));

	/*
	 * item 4 -- fffb5008, THE PIE IMAGE LOAD, and the reason this
	 * function is named loadPIEImage.
	 *
	 * This was previously dismissed as "an override list this TPL never
	 * populates, so a no-op rather than an omission".  That was wrong on
	 * both halves.  ctl+0x98 IS populated -- g12a_ddr_ctl_build() stores
	 * the piei blob's type-selected dl44 slice there, exactly as the
	 * vendor probe at bl2_new.bin 0xFFFA6D44 does -- and the list it
	 * points at is not an "override list", it is the PIE image plus the
	 * ACSM command sequence.
	 *
	 * MEASURED (t7-dtd-state-diff.md sec.3): with this omitted, 302 of
	 * 387 INITENG registers read 0x0000 where the blob's T7 leaves real
	 * content (PreSequenceReg0b0s0 = 0x0010, s1 = 0x0400, s2 = 0x010e),
	 * and 80 of 197 ACSM registers -- including the ACSM sequence RAM at
	 * 0x040000.. -- differ.  T10 (fffb8af0) runs that sequence to hand
	 * the DRAM to the DMC, so an empty PIE image is why T10's first spin
	 * on 0xFF638654 never completes.
	 *
	 * fffb5008 verbatim: x0 = ctl[152]; while ((csr = *(u32 *)x0))
	 * { write16(0xFE000000 + (csr << 1), *(u16 *)(x0 + 4)); x0 += 8; }
	 * -- it has NO null check on x0; ours does, because a zero there is
	 * a read of VA 0 and we would rather see the crumb than the fault.
	 *
	 * G12A_OPEN_PIEI_TABLE (default 1) replaces the blob's list with our
	 * own generated copy of the same 487 records.  Same addresses, same
	 * data, same order, same 16-bit store to 0xFE000000 + (csr << 1) --
	 * the only difference is where the bytes came from.  The crumb keeps
	 * the same shape so the two arms are directly comparable in a log:
	 * `pie 01e7` from the blob, `pieo 01e7` from the table.
	 */
#if G12A_OPEN_PIEI_TABLE
	{
		u32 n;

		for (n = 0; n < ARRAY_SIZE(g12a_piei_dl44_addr); n++)
			g12a_open_csr_w(g12a_piei_dl44_addr[n],
					g12a_piei_dl44_data[n]);

		g12a_uart_puts(" pieo ");
		g12a_uart_hex16((u16)n);
		g12a_uart_puts("\n");
	}
#else
	{
		ulong p = (ulong)readq((void *)(ctl + G12A_CTL_OFF_PIEI_SEL));
		u32 n = 0;

		while (p) {
			u32 csr = readl((void *)p);

			if (!csr)
				break;
			g12a_open_csr_w(csr, readw((void *)(p + 4)));
			p += 8;
			n++;
			if (n > 4096u)	/* a runaway list is a bug, not a PIE */
				break;
		}
		g12a_uart_puts(" pie ");
		g12a_uart_hex16((u16)n);
		g12a_uart_puts("\n");
	}
#endif

	/* item 5 -- per-channel refresh / ZQ from ds[56] / ds[58] */
	g12a_open_csr_w(0x02000bu, g12a_open_h(ds, 56) >> 4);	/* fffb5c30 */
	g12a_open_csr_w(0x12000bu, g12a_open_h(ds, 58) >> 4);	/* fffb5c44 */
	g12a_open_csr_w(0x02000cu, g12a_open_h(ds, 56) >> 3);	/* fffb5c58 */
	g12a_open_csr_w(0x12000cu, g12a_open_h(ds, 58) >> 3);	/* fffb5c6c */
	g12a_open_csr_w(0x02000du,
			((g12a_open_h(ds, 56) * 10u) >> 3) & 0xffffu);
	g12a_open_csr_w(0x12000du,
			((g12a_open_h(ds, 58) * 10u) >> 3) & 0xffffu);
	g12a_open_csr_w(0x02000eu, 0x2c);			/* fffb5cb4 */
	g12a_open_csr_w(0x12000eu, 0x2c);			/* fffb5cc0 */

	/* item 6 -- INITENG0 Seq0BDisableFlag0..7 */
	g12a_open_csr_w(0x09000cu, 0x0000);			/* fffb5ccc */
	g12a_open_csr_w(0x09000du, 0x0173);			/* fffb5cdc */
	g12a_open_csr_w(0x09000eu, 0x0060);			/* fffb5cec */
	g12a_open_csr_w(0x09000fu, 0x6110);			/* fffb5cf8 */
	g12a_open_csr_w(0x090010u, 0x2152);			/* fffb5d08 */
	g12a_open_csr_w(0x090011u, 0xdfbd);			/* fffb5d18 */
	g12a_open_csr_w(0x090012u, 0xffff);			/* fffb5d28 */
	g12a_open_csr_w(0x090013u, 0x6152);			/* fffb5d38 */

	/*
	 * item 7 -- Seq0BDisableFlag6 is 0x0060, NOT the 0x6152 our own
	 * published note carried (that is w4, flag 7's value; w2 still holds
	 * 0x60 from the flag-2 write).  The retraining gate at payload
	 * 0xFFFBF920 is a SHIPPED CONSTANT 1, so only this arm exists.
	 * Amlogic also differs from ST by one bit here: ST writes 0x2060.
	 */
	g12a_open_csr_w(0x090012u, 0x0060);			/* fffb5d68 */
	g12a_open_csr_w(0x020010u, 0);				/* fffb5d74 */
	g12a_open_csr_w(0x120010u, 0);				/* fffb5d80 */
	g12a_open_csr_w(0x020011u, 3);				/* fffb5db8 */
	g12a_open_csr_w(0x120011u, 3);				/* fffb5dc4 */

	/*
	 * item 8 -- ACSM retraining playback.  The trip count is THREE, not
	 * four (fffb5dec mov w2,#3; the 4 at fffb5e54 is ctl[440]==3 only),
	 * and BOTH pstates are written.  Every tm-sourced alternate is
	 * ctl[440]==3-only and therefore dead on LPDDR4.
	 */
	for (i = 0; i < 3; i++) {
		g12a_open_csr_w(0x040080u + 2u * i, 0x00e0);
		g12a_open_csr_w(0x040081u + 2u * i, 0x0012);
		g12a_open_csr_w(0x140080u + 2u * i, 0x00e0);
		g12a_open_csr_w(0x140081u + 2u * i, 0x0012);
	}

	g12a_open_csr_w(0x0400fdu, 0x000f);			/* item 9 */

	/* item 10 -- the per-DBYTE DTSM comparators, 4 instances x 14 CSRs */
	for (i = 0; i < 4; i++) {
		u32 b = i << 12;

		g12a_open_csr_w(0x10011u | b, 0x0001);		/* TsmByte1 */
		g12a_open_csr_w(0x10012u | b, 0x0001);		/* TsmByte2 */
		g12a_open_csr_w(0x10013u | b, 0x0180);		/* TsmByte3 */
		g12a_open_csr_w(0x10018u | b, 0x0001);		/* TsmByte5 */
		g12a_open_csr_w(0x10002u | b, 0x6209);		/* TrainingParam */
		g12a_open_csr_w(0x100b2u | b, 0x0001);		/* Tsm0 lane 0 */
		for (m = 1; m <= 8; m++)			/* Tsm2 lanes 1-8 */
			g12a_open_csr_w(0x100b4u | b | (m << 8), 0x0001);
	}

	if (ds[14] == 2u)					/* item 11 */
		g12a_open_csr_w(0x02003au, 2);			/* 0xFE040074 */
	g12a_open_csr_w(0x020089u, 1);				/* 0xFE040112 */
	g12a_open_csr_w(0x020088u, 0x11);			/* 0xFE040110 */
	if (ds[14] == 1u || ds[14] == 2u) {			/* item 12 */
		g12a_open_csr_w(0x0200b0u, 1);			/* 0xFE040160 */
		if (ds[14] == 2u)
			g12a_open_csr_w(0x0200b0u, 4);
	}
	/* item 13 is three level-gated log calls */
	g12a_phy_write16(G12A_DDR_PHY_UCCLKHCLKENABLES, 3);	/* item 14 */
	if (g12a_open_h(ds, 30)) {				/* item 15 */
		g12a_open_csr_w(0x010048u, 1);
		g12a_open_csr_w(0x011048u, 1);
		g12a_open_csr_w(0x012048u, 1);
		g12a_open_csr_w(0x013048u, 1);
	}

	/*
	 * item 16 -- equalise DFIMRL across the four DBYTEs.  A FOLD, not
	 * min(max(reads), 7): the uxtb at fffb6162 sits INSIDE the loop and
	 * BEFORE the clamp, so a DBYTE reading >= 0x100 collapses the running
	 * value and the answer is DBYTE-order dependent.
	 */
	mx = 0;							/* fffb6134 */
	for (i = 0; i < 4; i++) {
		u32 r = g12a_open_csr_r(0x10020u | (i << 12));

		if (r > mx)					/* fffb6160-64 */
			mx = r;
		mx &= 0xffu;					/* fffb6168 uxtb */
		if (mx > 7u)					/* fffb616c-70 */
			mx = 7u;
		mx &= 0xffu;					/* fffb6178 */
	}
	for (i = 0; i < 4; i++)					/* fffb6180-61a4 */
		g12a_open_csr_w(0x10020u | (i << 12), mx);

	g12a_open_csr_w(0x0200e3u, 0x0c00);			/* item 17 */
	g12a_uart_puts("OP7 dfimrl ");
	g12a_uart_hex16((u16)mx);
	g12a_uart_puts("\n");
}

/* --------------------------------------------------------------- S1 */

/*
 * fffb8e60 -- per-channel density (MB, ds+40 / ds+42) to the packed
 * (row/col/bank) field code the DMC geometry word carries.  A pure
 * switch with no memory and no MMIO.  The fall-through arm returns
 * 0x005 rather than 0: w1 still holds the 5 loaded at fffb8e98 before
 * the comparison at fffb8e9c failed, and fffb8ed0 only clears w0.
 *
 * 0xFFFF -- what Solitude's ACS ships in BOTH slots, because size is
 * discovered later by slot5 -- takes exactly that arm.
 */
static u32 g12a_open_density_code(u16 mb)
{
	switch (mb) {
	case 0x0060: return 0x000;	/* fffb8ea0 -> fffb8f18 -> fffb8ed0 */
	case 0x0080: return 0x000;	/* fffb8e60 -> fffb8f18 -> fffb8ed0 */
	case 0x0100: return 0x001;	/* fffb8e68 -> fffb8f20 -> fffb8ed0 */
	case 0x0200: return 0x002;	/* fffb8e70, w1 = 2  -> fffb8ed0 */
	case 0x0400: return 0x003;	/* fffb8e7c, w1 = 3 */
	case 0x0800: return 0x004;	/* fffb8e88, w1 = 4 */
	case 0x1000: return 0x005;	/* fffb8e94, w1 = 5 */
	case 0x00c0: return 0x008;	/* fffb8ea8 -> fffb8ed8: 0x008 | 0 */
	case 0x0180: return 0x109;	/* fffb8eb0 -> fffb8ee4: 0x108 | 1 */
	case 0x0300: return 0x20a;	/* fffb8eb8 -> fffb8ef0: 0x208 | 2 */
	case 0x0600: return 0x40b;	/* fffb8ec0 -> fffb8efc: 0x408 | 3 */
	case 0x0c00: return 0x80c;	/* fffb8ec8 -> fffb8f08: 0x808 | 4 */
	default:     return 0x005;	/* fffb8ecc falls to fffb8ed0, w1 = 5 */
	}
}

/*
 * fffb929c -- prove the DMC register file is writable before anything
 * depends on it.  0xFF638668 is the DMC scratch word T12 also writes
 * (fffb8d44).  The verdict lands in ctl+610, which g12a_open_mrs() reads
 * to choose between the DMC MRS engine (0xFF6394E0) and the legacy
 * 0xFF638664 doorbell -- so S1 is a precondition of T14, not decoration.
 * The blob prints "\ndmc_version <n>" here; ours prints its own crumb.
 */
static int g12a_open_s1_probe(ulong ctl)
{
	writeb(0, (void *)(ctl + 610));				/* fffb92ac */
	writel(0x1234u, (void *)0xFF638668UL);			/* fffb92bc */
	if ((readl((void *)0xFF638668UL) & 0xffffu) == 0x1234u)	/* fffb92cc */
		writeb(1, (void *)(ctl + 610));			/* fffb92d8 */
	g12a_uart_puts("OP1 dmcv ");
	g12a_uart_hex8(readb((void *)(ctl + 610)));
	g12a_uart_puts("\n");
	return readb((void *)(ctl + 610));
}

/*
 * fffb931c (S1) -- the DMC address map and geometry.  Despite the arc's
 * long-standing "PLL / DMC address map" label this function programs NO
 * PLL: the DDR PLL is fp_pll's (export[7], fffb9000), which a644's caller
 * runs before it.  S1 is three MMIO writes plus the probe above.
 */
static int g12a_open_s1(ulong ctl, u8 *ds)
{
	u16 cs0, cs1;
	u32 c0, c1, g;

	g12a_open_s1_probe(ctl);				/* fffb9330 */

	if (!readb((void *)(ctl + 610)))			/* fffb9338 */
		ds[65] = 0;		/* fffb9340 -- ds+0x41 rdbi_wr */

	cs0 = (u16)((u16)ds[40] | ((u16)ds[41] << 8));		/* fffb9348 */
	cs1 = (u16)((u16)ds[42] | ((u16)ds[43] << 8));		/* fffb9354 */
	c0 = g12a_open_density_code(cs0);			/* fffb9350 */
	c1 = g12a_open_density_code(cs1);			/* fffb9360 */

	/* fffb9364..fffb938c -- five ADDs of disjoint fields, in order. */
	g  = (c1 & 0x7u) << 3;					/* fffb9368 */
	g += (c0 & 0xf00u) << 20;				/* fffb9364/6c */
	g += c0 & 0x7u;						/* fffb9370/74 */
	g += (c0 & 0x8u) << 5;					/* fffb937c/80 */
	g += (c1 & 0x8u) << 6;					/* fffb9388/8c */

	if (ds[14] == 1u) {					/* fffb9384 */
		g |= 0x00500000u;				/* fffb9394 */
	} else if (ds[14] == 3u) {				/* fffb93a0 */
		g |= 0x00800000u;				/* fffb93a8 */
	} else if (ds[14] == 2u) {				/* fffb93b0 */
		/* fffb93b8-c8: rank_cfg in {5,6} keeps the high nibbles. */
		if ((u8)(ds[18] - 5u) > 1u)
			g &= 0x0f3fffffu;			/* fffb93cc */
		g |= 0x00c00000u;				/* fffb93d4 */
	}
	if (cs1 != 0u)						/* fffb93dc/e0 */
		g |= 0x00200000u;
	if ((ds[15] & 0xfu) != 0u)				/* fffb93d8/e8/f0 */
		g |= 0x00010000u;

	/* fffb93f8..fffb9410 -- clear bit 1 of the DMC control word. */
	writel(readl((void *)0xFF639254UL) & ~2u, (void *)0xFF639254UL);
	writel(g, (void *)0xFF639368UL);			/* fffb941c */
	/* fffb9420..fffb9440 -- AO_SEC_GP_CFG0[31:16] = total density. */
	writel((readl((void *)0xFF800240UL) & 0xffffu) |
	       ((u32)(u16)(cs0 + cs1) << 16), (void *)0xFF800240UL);

	g12a_uart_puts("OP1 geo ");
	g12a_uart_hex32(g);
	g12a_uart_puts("\n");
	return 0;						/* fffb9448 */
}

/* ------------------------------------------------------------ slot6 */

#if G12A_OPEN_SLOT6
/*
 * ⚠️ ADDRESS CONVENTION.  Every hex address in the slot6 block below is a
 * REAL VA -- the payload VA, i.e. what
 *
 *   aarch64-linux-gnu-objdump -D -b binary -m aarch64 \
 *       --adjust-vma=0xFFFB3FA0 aml_ddr.fw
 *
 * prints.  This is the SAME convention G12A_AML_FN_* in g12a-ddr-fw.h uses
 * (that header's values are real VAs and its "D-VA" comments are +0x60), and
 * it is the OPPOSITE of the in-blob citations in g12a_open_s1() above, which
 * are D-VA (+0x60, the --adjust-vma=0xFFFB4000 form): s1's
 * "fffb93f8..fffb9410 -- clear bit 1" is at real 0xFFFB9398..0xFFFB93B0, and
 * real 0xFFFB93F8 is a DIFFERENT function -- the one slot6 calls for its
 * delay figure.  Both conventions live in this file; do not mix them.
 *
 * Blob: vendor/amlogic/blx/init/g12a/aml_ddr.fw in
 * ~/git/libretech-builder-simple (NOTE: the LBS repo root, NOT under
 * u-boot/), 48080 B, sha256
 * c15c3715744030590c87edbda8bc5185a68df3837d094f8684f22400407940ad.
 * Entry read from the export table at file +0x90 -> 0xFFFBD52C, matching
 * G12A_AML_FN_POST (g12a-ddr-fw.h:601).
 *
 * ---- REGISTER NAMES -------------------------------------------------
 * amlogic/software/CLAUDE.md "A Register Name Needs a Citation" binds: a
 * name without a vendor header:line is an invention.  Exactly one address
 * here resolves.
 *
 *   0xFFD0F0D0  G12A_WATCHDOG_CNTL -- g12a-ddr-fw.h:270, cited there to BSP
 *               arch-g12a/regs.h:13304.  DELIBERATELY NOT TRANSCRIBED, see
 *               "WHAT IS DROPPED" below.
 *   0xFF639254  NO NAME in any map we hold.  dram-g12a.c:6217 already uses
 *               it as a bare literal (the &= ~2 site in g12a_open_s1()).
 *   0xFF63864C  NO NAME.  It is adjacent to the DMC<->PHY handshake pair
 *               0xFF638650/0xFF638654 that g12a_open_t10() drives, but
 *               adjacency is not a citation.
 *   0xFE0401C6 / 0xFE0401C8  the BLOCK is named -- g12a-ddr-fw.h:92 gives
 *               PA = 0xFE000000 + (CSR << 1) -- so these are DWC PHY CSR
 *               0x0000E3 and 0x0000E4.  The INDIVIDUAL CSRs are not named.
 *   0xFFFBFB80..0xFFFBFB88  not registers: SRAM scratch immediately past
 *               the payload (0xFFFB3FA0 + 0xBBD0 = 0xFFFBFB70).
 *
 * ⚠️ The amlogic_bsp_grep MCP returns nothing for any of these and THAT
 * NEGATIVE IS WORTHLESS -- it indexes media_modules/drivers/ (video codec),
 * the wrong corpus for a DDR register.  Recorded so nobody re-runs it and
 * reads the empty result as evidence of absence.
 */
#define G12A_UNNAMED_0XFF639254		0xFF639254UL
#define G12A_UNNAMED_0XFF63864C		0xFF63864CUL
#define G12A_UNNAMED_PHY_CSR_0X0E3	0xFE0401C6UL	/* CSR 0x0000E3 PA */
#define G12A_UNNAMED_PHY_CSR_0X0E4	0xFE0401C8UL	/* CSR 0x0000E4 PA */
#define G12A_UNNAMED_SLOT6_SCRATCH	0xFFFBFB80UL

/* ctl fields.  Named ones come from g12a-ddr-fw.h; the five gates and the
 * 0x258 slot have no vendor name, so they get _UNNAMED_0X<off> labels. */
#define G12A_CTL_OFF_UNNAMED_0X1C8	0x1c8U	/* gate -> 0xFFFBCEC4 */
#define G12A_CTL_OFF_UNNAMED_0X1CC	0x1ccU	/* gate -> 0xFFFBAAD0 */
#define G12A_CTL_OFF_UNNAMED_0X1F0	0x1f0U	/* gate -> 0xFFFBABBC / 0xFFFB9668 */
#define G12A_CTL_OFF_UNNAMED_0X208	0x208U	/* gate -> 0xFFFBD0C0 */
#define G12A_CTL_OFF_UNNAMED_0X236	0x236U	/* inverts the 0xFF639254 |= 2 */
#define G12A_CTL_OFF_UNNAMED_0X258	0x258U	/* 0x55AA55AA sentinel */
#define G12A_DS_OFF_UNNAMED_0X78	0x78U	/* bit 0 -> 0xFFFBABBC */

/* The five blob sub-handlers the gates select.  Real VAs, taken from the
 * bl targets at 0xFFFBD5E4 / 0xFFFBD604 / 0xFFFBD618 / 0xFFFBD628 /
 * 0xFFFBD638.  Not transcribed -- see the gate comment in g12a_open_slot6(). */
#define G12A_AML_FN_SLOT6_G1CC		0xFFFBAAD0UL
#define G12A_AML_FN_SLOT6_G1F0A		0xFFFBABBCUL
#define G12A_AML_FN_SLOT6_G1F0B		0xFFFB9668UL
#define G12A_AML_FN_SLOT6_G1C8		0xFFFBCEC4UL
#define G12A_AML_FN_SLOT6_G208		0xFFFBD0C0UL

/*
 * 0xFFFB93F8 -- the delay figure slot6 stores to scratch.  22 instructions,
 * 0xFFFB93F8..0xFFFB944C, transcribed whole.
 *
 * 100000000 / (2 * DRAMFreq[ch]) is a half-period in hundredths of a
 * picosecond; dividing by the 10-bit field of PHY CSR 0x0E4 makes the result
 * a delay-line step count per half-UI.  It is computed and then stored ONLY
 * to SRAM scratch (see g12a_open_slot6()).
 *
 * 🟡 The channel index here is 0xFF63864C bit 0, NOT ctl+0x1B9 -- slot6's own
 * DRAMFreq store two lines later uses ctl+0x1B9.  The two CAN disagree; on
 * this board both are 0.  Transcribed as the blob has it, disagreement and
 * all.
 *
 * Both divisions are SDIV in the blob.  Every operand is provably
 * non-negative here (DRAMFreq and the CSR field are u16-derived), so the
 * unsigned form below is bit-identical.  AArch64 SDIV/UDIV by zero returns 0
 * without trapping, so the explicit zero guards reproduce the hardware rather
 * than diverging from it -- in C the unguarded form would be UB.
 */
static u16 g12a_open_slot6_dly(ulong ctl)
{
	ulong ds = (ulong)readq((void *)(ctl + G12A_CTL_OFF_DDR_SET));
	u32 ch = readl((void *)G12A_UNNAMED_0XFF63864C) & 1u;	/* fffb9400 */
	u32 f, s, q;

	/* fffb941c -- PHY CSR 0x0E3 = 0x0C00.  The SAME constant is written to
	 * the SAME CSR at 0xFFFB6154, so this write is idempotent and, at
	 * worst, a repeat: it is NOT evidence that slot6 ran. */
	g12a_phy_write16(G12A_UNNAMED_PHY_CSR_0X0E3, 0x0c00u);

	f = readw((void *)(ds + G12A_DS_OFF_DRAM_FREQ + ch * 2u));  /* fffb942c */
	s = g12a_phy_read16(G12A_UNNAMED_PHY_CSR_0X0E4);	   /* fffb9430 */

	f = 2u * f;						/* fffb9438 */
	q = f ? (100000000u / f) : 0u;				/* fffb9440 */
	s &= 0x3ffu;						/* fffb9444 */
	return (u16)(s ? (q / s) : 0u);				/* fffb9448 */
}

/*
 * 0xFFFBD52C (slot6, export[6]) -- 72 instructions, 0xFFFBD52C..0xFFFBD648,
 * transcribed whole except the tail call.  x19 = ctl.
 *
 * ---- WHAT IT ACTUALLY DOES ON THE OPEN ARM --------------------------
 * All five conditional post-steps are gated on ctl+0x1C8 / 0x1CC / 0x1F0 /
 * 0x208 and ds+0x78 bit 0, and every one of those is ZERO here:
 * g12a_ddr_ctl_build() zeroes the whole 0x280-byte ctl block (:1523, size
 * g12a-ddr-fw.h:444) and never writes any of them, and the blob writes none
 * of them either -- the only in-blob store to any is `strb wzr,[x19,#456]`
 * at 0xFFFBD0A8, which writes ZERO.  ds is likewise zeroed (:1524) and
 * ds[0x78] is 0x00 in the product ACS (g12a_product_acs_p0, dram-g12a.c:1363,
 * the 0x70 row, byte 8); no g12a_poke*(ds, ...) in ctl_build touches 0x78
 * (nearest are G12A_DS_OFF_LPDDR4_VOUT 0x76 and G12A_DS_OFF_BITTIME_2D 0x7d).
 *
 * Control for those negatives: the same store regex finds FOUR writers of
 * ctl+0x205 (0xFFFBABE0, 0xFFFBCF8C, 0xFFFBD114, 0xFFFBD4FC), so a zero
 * result is a real absence, not a broken pattern.  Bound: this excludes
 * immediate-offset stores; it cannot exclude a register-computed base.
 *
 * The gates are kept as explicit guards rather than deleted, and when one
 * fires we CALL THE BLOB'S OWN HANDLER instead of silently skipping -- so a
 * future ctl-builder change makes the path live rather than quietly wrong.
 *
 * ---- WHAT A BOOT WOULD AND WOULD NOT ESTABLISH ----------------------
 * slot6's ONE durable, externally visible effect is 0xFF639254 bit 1, which
 * g12a_open_s1() CLEARS at :6217 before rewriting the DMC geometry word
 * 0xFF639368, and slot6 SETS again afterwards -- a matched close/reopen
 * around a DMC reconfiguration.  Enumerated over the whole blob, the only
 * three 0xFF639254 sites are 0xFFFB9398/93A4 (&= ~2, already ours),
 * 0xFFFBCEE0/CEEC (|= 2, inside the ctl+0x1C8-gated handler, dead here) and
 * 0xFFFBD540/D55C (|= 2, here).  NOTHING IN THE BLOB READS IT BACK.
 *
 * Therefore, before any boot:
 *   - ctl+0x258 = 0x55AA55AA has exactly one reader, 0xFFFBAAE8, inside the
 *     ctl+0x1CC-gated handler -- dead on this arm.
 *   - the 0xFFFBFB80..0x88 record has NO reader on the open arm at all.
 *   - PHY CSR 0x0E3 = 0x0C00 is a constant also written at 0xFFFB6154.
 *   => a green five-walk Layer-1 fingerprint PROVES NOTHING about bit 1, and
 *      the Layer-1 chain cannot see the difference either: DTA..DTE, <ta 01,
 *      the size figure and the DataBus/AddrBus walks are all produced before
 *      slot6 runs, or by g12a_dram_test()/g12a_dram_margin(), which read DRAM
 *      through a DMC that bit 1 does not gate the boot on.
 * The test that settles it is the LATE read-back of 0xFF639254 -- from
 * U-Boot proper or the booted kernel -- on a stock product boot versus this
 * one.  It rides on any boot that already reaches Linux.
 *
 * ---- WHAT IS DELIBERATELY DROPPED -----------------------------------
 * The blob TAIL-CALLS wdt_arm(5000) at 0xFFFBD644/D648 -> 0xFFFB4040 (19
 * instructions: G12A_WATCHDOG_CNTL = 0x03605DBF, +0x08 = 5000, +0x0C = 0,
 * then |= 0x40000 to arm).  WE DO NOT REPRODUCE IT.  g12a_t_call() -- the
 * wrapper the open path invokes slot6 through -- calls g12a_wdt_pet_safe()
 * immediately after, which writes G12A_WATCHDOG_CNTL &
 * G12A_WATCHDOG_CNTL_CLR, i.e. a DISABLE.  Copying the arm would be a
 * faithful copy of an operation the surrounding code exists to undo.
 * Transcribe behaviour that matters, not instructions.
 *
 * Consequence: the blob's return value is JUNK -- 0xF0D0, the low half of the
 * last register address left in x0 by wdt_arm, not a status -- so the blob
 * arm's crumb reads "<tp d0".  Ours returns 0.  A differing crumb byte
 * between the arms is EXPECTED and is not a failure on either side.
 *
 * Counting rule (so it can be re-run): one instruction = one 4-byte AArch64
 * word; a function's count is (last_va - entry_va) / 4 + 1, entry stp through
 * terminating ret or tail-call b inclusive.  slot6 72, 0xFFFB93F8 22,
 * wdt_arm 19 -> 94 read, 75 transcribed, 19 dropped.
 */
static int g12a_open_slot6(ulong ctl)
{
	ulong ds = (ulong)readq((void *)(ctl + G12A_CTL_OFF_DDR_SET));
	u32 before, after;
	u16 r, f;
	u8 g;

	g12a_uart_puts(">tp6\n");

	before = readl((void *)G12A_UNNAMED_0XFF639254);

	/*
	 * fffbd53c..fffbd564 -- 0xFF639254 |= (ctl[0x236] ? 0 : 2).
	 * ctl+0x236 is zero (above), so this is the |= 2 that reopens what
	 * g12a_open_s1() closed at :6217.
	 */
	g = readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X236));	/* fffbd53c */
	writel(before | (g ? 0u : 2u), (void *)G12A_UNNAMED_0XFF639254);

	after = readl((void *)G12A_UNNAMED_0XFF639254);

	/*
	 * fffbd568..fffbd578 -- write the sentinel only if the slot is clear.
	 * Its one reader is dead on this arm (see above).
	 */
	if (readl((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X258)) == 0u)
		writel(0x55aa55aau, (void *)(ctl + G12A_CTL_OFF_UNNAMED_0X258));

	/*
	 * fffbd57c..fffbd584 -- ctl[0x205] (G12A_CTL_OFF_HDT) is loaded into
	 * w1 and the callee never reads it: a DEAD ARGUMENT.  Not reproduced;
	 * loading a register the callee ignores has no observable effect.
	 */
	r = g12a_open_slot6_dly(ctl);

	/* fffbd588..fffbd5b8 -- the same halfword four times, in the blob's
	 * order: +4, +8, +0, +2.  Order preserved so a scratch dump taken
	 * mid-sequence compares. */
	writew(r, (void *)(G12A_UNNAMED_SLOT6_SCRATCH + 4));
	writew(r, (void *)(G12A_UNNAMED_SLOT6_SCRATCH + 8));
	writew(r, (void *)(G12A_UNNAMED_SLOT6_SCRATCH + 0));
	writew(r, (void *)(G12A_UNNAMED_SLOT6_SCRATCH + 2));

	/* fffbd5bc..fffbd5d4 -- DRAMFreq[ctl+0x1B9] into the fifth halfword.
	 * NOTE the channel index differs from the one dly() used. */
	f = readw((void *)(ds + G12A_DS_OFF_DRAM_FREQ +
			   (u32)readb((void *)(ctl + G12A_CTL_OFF_CHAN)) * 2u));
	writew(f, (void *)(G12A_UNNAMED_SLOT6_SCRATCH + 6));

	/* fffbd5d8..fffbd638 -- the five conditional post-steps.  All five
	 * gates are zero on this configuration; the crumb below makes that an
	 * observation rather than an assumption. */
	if (readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X1CC)))	/* fffbd5d8 */
		((int (*)(ulong))G12A_AML_FN_SLOT6_G1CC)(ctl);
	if (readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X1F0)) == 1u ||
	    (readb((void *)(ds + G12A_DS_OFF_UNNAMED_0X78)) & 1u))
		((int (*)(ulong))G12A_AML_FN_SLOT6_G1F0A)(ctl);
	if (readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X1F0)) == 2u)
		((int (*)(ulong))G12A_AML_FN_SLOT6_G1F0B)(ctl);
	if (readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X1C8)))
		((int (*)(ulong))G12A_AML_FN_SLOT6_G1C8)(ctl);
	if (readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X208)))
		((int (*)(ulong))G12A_AML_FN_SLOT6_G208)(ctl);

	/*
	 * fffbd644/fffbd648 -- tail call wdt_arm(5000): DROPPED, see above.
	 *
	 * The read-back pair below IS the experiment: it turns the
	 * transcription's correctness into a byte comparison rather than an
	 * outcome inference.  Expect before with bit 1 CLEAR (g12a_open_s1()
	 * cleared it at :6217 and nothing sets it in between) and after ==
	 * before | 2, differing in NO other bit.
	 */
	g12a_uart_puts("OP6 9254 ");
	g12a_uart_hex32(before);
	g12a_uart_putc(' ');
	g12a_uart_hex32(after);
	g12a_uart_puts("\nOP6 dly ");
	g12a_uart_hex16(r);
	g12a_uart_puts(" freq ");
	g12a_uart_hex16(f);
	g12a_uart_puts("\nOP6 gates ");
	g12a_uart_hex8(readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X1CC)));
	g12a_uart_hex8(readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X1F0)));
	g12a_uart_hex8(readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X1C8)));
	g12a_uart_hex8(readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X208)));
	g12a_uart_hex8(readb((void *)(ctl + G12A_CTL_OFF_UNNAMED_0X236)));
	g12a_uart_hex8(readb((void *)(ds + G12A_DS_OFF_UNNAMED_0X78)));
	g12a_uart_puts("\n<tp6 00\n");

	return 0;
}
#endif /* G12A_OPEN_SLOT6 */

/* ------------------------------------------------------------ slot5 */

#if G12A_OPEN_SLOT5
/*
 * slot5 (export[5], real VA 0xFFFB9DA8..0xFFFBA110, 219 insns) -- the size
 * detect.  Transcribed from the blob; 0 bytes copied.  Every address in
 * this block is a REAL VA (objdump --adjust-vma=0xFFFB3FA0), the same
 * convention the slot6 block above uses; the comment at the KEEP_SLOT5
 * call site quotes the older +0x60 "D-VA" form (fffb9e08 == this 9da8).
 * Page: amlogic/software/boot/g12/aml-ddr-residue-transcription.md §4.
 *
 * In the blob's order:
 *   1. fffb9db4..fffb9e24  resolve the 0xFFFF auto-size sentinels in
 *      ds+0x28 / ds+0x2a to 0x1000 (4096 MB); nrank = 2 if cs1 was a
 *      sentinel, else 1; if NEITHER was a sentinel, skip to step 5;
 *   2. fffb9e28..fffb9e50  seed 0xFF639368[5:0] = 0x2d -- density code 5
 *      (4096 MB) for both chip-selects, so the walk runs against the
 *      largest decode;
 *   3. fffb9e60..fffb9f7c  per rank: 30-point write/read-back walk over
 *      base + (4 << i), i = 0..29 (a "\nwrite-error" per miss, no other
 *      effect); then the coarse alias search from i = 20 (4 MiB) up; then
 *      the 3/4-vs-4/4 refinement -> rank size in MB;
 *   4. fffb9f80..fffb9ffc  store it (cs1 CLAMPED to 0x800 = 2048 MB),
 *      recode it through fffb8e00 (= g12a_open_density_code()) and RMW
 *      that rank's fields of 0xFF639368; rank 1's base is cs0_mb << 20;
 *   5. fffba01c..fffba0dc  report cs0 / cs1 / DMC_DDR_CTRL, then
 *      "DDR size" = total, or the literal 0xF58 = 3928 when total << 20
 *      does not fit 32 bits, rounded to 8 MB, into AO_SEC_GP_CFG0[31:16];
 *   6. fffba0e0..fffba0f4  DataBus + AddrBus (fffb9bb0 / fffb9c80) --
 *      G12A_BUS_TEST's territory, NOT reproduced here.
 *
 * ⚠️ ALL ADDRESS ARITHMETIC IS 32-BIT, deliberately.  The blob computes
 * every point in w-registers (fffb9e7c "add w0, w0, w21", fffb9f18/f24),
 * so on the rank-1 pass (base 0x80000000 for a 2 GiB cs0) the i = 29 point
 * and the whole refinement WRAP to 0x00000000 / 0x40000000 -- rank-0
 * addresses -- and the refinement then reports 4096 MB, which is what the
 * "csel ... ls" clamp at fffb9f8c turns into the measured "DDR cs1 size:
 * 2048MB".  A 64-bit rewrite would walk 0x1_0000_0000 instead and hang on
 * the aperture (dram-capacity-accounting.md §4).  Ours keeps the u32 so
 * it lands the same figure by the same route, and prints the pre-clamp
 * value ("raw") so the wrap is visible in the log rather than assumed.
 *
 * Consequence for low memory: the rank-1 pass leaves address 0 holding
 * G12A_SLOT5_PAT_B (its refinement's last store), not PAT_A -- the
 * design page g12-slot5-lowmem-writes.md is corrected on this point.
 */
#define G12A_SLOT5_PAT_A		0xAABBCCDDu	/* fffb9e60/68 */
#define G12A_SLOT5_PAT_B		0x55443322u	/* fffb9eb8/c0 */
#define G12A_SLOT5_SENTINEL_MB		0x1000u		/* fffb9df4 */
#define G12A_SLOT5_CS1_CLAMP_MB		0x800u		/* fffb9f88 */
#define G12A_SLOT5_APERTURE_MB		0xF58u		/* fffba098 */

/*
 * fffb9e60..fffb9f7c -- discover one rank's size in MB.  @base is the rank
 * base as the blob holds it: a 32-bit byte address (w21).  Returns the
 * UNCLAMPED figure (4 << sh or 3 << sh, sh = i - 20); the caller clamps
 * cs1.  @werr counts the step-3 read-back misses the blob prints
 * "\nwrite-error" for.
 */
static u32 g12a_open_slot5_walk(u32 base, u32 *werr)
{
	u32 i, a, a2, a3, sh;

	writel(G12A_SLOT5_PAT_A, (void *)(ulong)base);	/* fffb9e6c */

	/* fffb9e70..fffb9eac -- 30 points, each written then read back. */
	for (i = 0; i < 30u; i++) {
		a = base + (4u << i);				/* fffb9e74/7c */
		writel(G12A_SLOT5_PAT_A, (void *)(ulong)a);	/* fffb9e84 */
		if (readl((void *)(ulong)a) != G12A_SLOT5_PAT_A)	/* fffb9e88/94 */
			(*werr)++;				/* fffb9ea0 */
	}

	/*
	 * fffb9eb0..fffb9f00 -- coarse: the first i in 20..29 whose point
	 * either does not hold PAT_B or clobbers base.  Falls out at i = 30
	 * if none does.
	 */
	for (i = 20u; i < 30u; i++) {
		a = base + (4u << i);				/* fffb9eb4/bc */
		writel(G12A_SLOT5_PAT_B, (void *)(ulong)a);	/* fffb9ec4 */
		g12a_open_delay_us(1);				/* fffb9ecc */
		if (readl((void *)(ulong)a) != G12A_SLOT5_PAT_B)	/* fffb9ed0/dc */
			break;
		if (readl((void *)(ulong)base) != G12A_SLOT5_PAT_A)	/* fffb9ee4/f0 */
			break;
	}

	/*
	 * fffb9f04..fffb9f7c -- refine between 3 << i and 4 << i: write PAT_A
	 * at both 2<<i and 3<<i, then PAT_B at 2<<i; if 3<<i still reads
	 * PAT_A the two are distinct and the rank is 4 << sh MB, else 3 << sh.
	 */
	sh = i - 20u;						/* fffb9f10 */
	a2 = base + (2u << i);					/* fffb9f0c/18 */
	a3 = a2 + (1u << i);					/* fffb9f14/20/24 */
	writel(G12A_SLOT5_PAT_A, (void *)(ulong)a2);		/* fffb9f30 */
	writel(G12A_SLOT5_PAT_A, (void *)(ulong)a3);		/* fffb9f38 */
	writel(G12A_SLOT5_PAT_B, (void *)(ulong)a2);		/* fffb9f44 */
	g12a_open_delay_us(1);					/* fffb9f50 */
	if (readl((void *)(ulong)a3) == G12A_SLOT5_PAT_A)	/* fffb9f54/60 */
		return 4u << sh;				/* fffb9f6c */
	return 3u << sh;					/* fffb9f74/78 */
}

/*
 * fffb9fa4..fffb9ffc -- place one rank's density code into 0xFF639368.
 * Same field placement g12a_open_s1() builds from scratch (fffb9364..8c);
 * here it is a read-modify-write of the live word, one rank at a time.
 * 0xFF639368 resolves in no vendor map ("in block DMC",
 * a311d-reference.txt:81) -- an _UNNAMED_ address, as in S1.
 */
static void g12a_open_slot5_geo(u32 rank, u32 mb)
{
	u32 code = g12a_open_density_code((u16)mb);		/* fffb9fa0 */
	u32 lo = code & 0x7u;					/* fffb9fa8 */
	u32 hi = code & 0x8u;					/* fffb9fb0 */
	u32 v = readl((void *)0xFF639368UL);

	if (rank == 0u)						/* fffb9fb4 */
		v = (v & 0xFFFFFEF8u) | lo | (hi << 5);		/* fffb9fe0..fffb9ffc */
	else
		v = (v & 0xFFFFFDC7u) | (lo << 3) | (hi << 6);	/* fffb9fb8..fffb9fd8 */
	writel(v, (void *)0xFF639368UL);
}

/*
 * The whole export, steps 1-5.  Crumbs (ours, not the blob's strings):
 *   OP5 auto <cs0 before> <cs1 before>   the sentinel state on entry
 *   OP5 r<n> raw <mb> werr <count>       per rank walked, pre-clamp
 *   OP5 cs 0800 0800 geo 00e00024 size 0f58 ms <AO_SEC_GP_CFG0>
 * The last line is the paired-control against slot5's own
 *   "DDR cs0 size: 2048MB / DDR cs1 size: 2048MB / DMC_DDR_CTRL: 00e00024
 *    / DDR size: 3928MB" (boot stage2-2, dram-access-bringup.md).
 */
static int g12a_open_slot5(ulong ctl)
{
	ulong ds = (ulong)readq((void *)(ctl + G12A_CTL_OFF_DDR_SET));	/* fffb9db4 */
	u32 cs0 = readw((void *)(ds + G12A_DS_OFF_CS0_MB));		/* fffb9dbc */
	u32 cs1 = readw((void *)(ds + G12A_DS_OFF_CS1_MB));		/* fffb9e00 */
	u32 nrank = 0, rank, base = 0, mb, raw, werr, total, size, v;

	g12a_uart_puts(">tp5\nOP5 auto ");
	g12a_uart_hex16((u16)cs0);
	g12a_uart_putc(' ');
	g12a_uart_hex16((u16)cs1);
	g12a_uart_puts("\n");

	/* 1. fffb9db8..fffb9e24 -- sentinels, and how many ranks to walk. */
	if (cs0 == 0xFFFFu) {					/* fffb9dd0 */
		cs0 = G12A_SLOT5_SENTINEL_MB;			/* fffb9df4/f8 */
		writew((u16)cs0, (void *)(ds + G12A_DS_OFF_CS0_MB));
		nrank = 1u;					/* fffb9dfc */
	}
	if (cs1 == 0xFFFFu) {					/* fffb9e08 */
		cs1 = G12A_SLOT5_SENTINEL_MB;			/* fffb9e10/14 */
		writew((u16)cs1, (void *)(ds + G12A_DS_OFF_CS1_MB));
		nrank = 2u;					/* fffb9e18 */
	}
	/* fffb9e20: neither a sentinel -> no walk, straight to the report. */

	if (nrank) {
		/* 2. fffb9e28..fffb9e50 -- seed both codes to 5 (4096 MB). */
		writel((readl((void *)0xFF639368UL) & 0xFFFFFFC0u) | 0x2du,
		       (void *)0xFF639368UL);

		/* 3.+4. fffb9e60..fffba018 -- per rank. */
		for (rank = 0; rank < nrank; rank++) {
			werr = 0;
			raw = g12a_open_slot5_walk(base, &werr);
			mb = raw;
			if (rank == 0u) {				/* fffb9f80 */
				cs0 = mb;
				writew((u16)mb, (void *)(ds + G12A_DS_OFF_CS0_MB));	/* fffb9f98 */
			} else {
				if (mb > G12A_SLOT5_CS1_CLAMP_MB)	/* fffb9f84/8c */
					mb = G12A_SLOT5_CS1_CLAMP_MB;
				cs1 = mb;
				writew((u16)mb, (void *)(ds + G12A_DS_OFF_CS1_MB));	/* fffb9f90 */
			}
			g12a_open_slot5_geo(rank, mb);			/* fffb9fa0..fffb9ffc */

			g12a_uart_puts("OP5 r");
			g12a_uart_hex8((u8)rank);
			g12a_uart_puts(" raw ");
			g12a_uart_hex16((u16)raw);
			g12a_uart_puts(" werr ");
			g12a_uart_hex8((u8)werr);
			g12a_uart_puts("\n");

			base = cs0 << 20;				/* fffba00c/14 -- w21 */
		}
	}

	/* 5. fffba01c..fffba0dc -- report and publish. */
	cs0 = readw((void *)(ds + G12A_DS_OFF_CS0_MB));		/* fffba020 */
	cs1 = readw((void *)(ds + G12A_DS_OFF_CS1_MB));		/* fffba01c */
	total = cs0 + cs1;					/* fffba028 */
	/* fffba090..fffba09c: total << 20 must fit a 32-bit byte address. */
	size = ((u64)total << 20) <= 0xFFFFFFFFull ? total
						   : G12A_SLOT5_APERTURE_MB;
	size &= 0xFF8u;						/* fffba0ac */
	/* fffba0c0..fffba0dc -- AO_SEC_GP_CFG0[31:16] = size, [15:0] kept.
	 * AO_SEC_GP_CFG0 = 0xFF800240, scp_task/secure_apb.h:3184. */
	v = (readl((void *)G12A_AO_SEC_GP_CFG0) & 0xFFFFu) | (size << 16);
	writel(v, (void *)G12A_AO_SEC_GP_CFG0);

	g12a_uart_puts("OP5 cs ");
	g12a_uart_hex16((u16)cs0);
	g12a_uart_putc(' ');
	g12a_uart_hex16((u16)cs1);
	g12a_uart_puts(" geo ");
	g12a_uart_hex32(readl((void *)0xFF639368UL));
	g12a_uart_puts(" size ");
	g12a_uart_hex16((u16)size);
	g12a_uart_puts(" ms ");
	g12a_uart_hex32(v);
	g12a_uart_puts("\n<tp5 00\n");

	/* 6. fffba0e0..fffba0f4 -- the bus tests: G12A_BUS_TEST, not here. */
	return 0;
}
#endif /* G12A_OPEN_SLOT5 */

/* -------------------------------------------------------------- T10 */

/*
 * fffb8af0 (T10) -- the DMC<->PHY handshake.  0xFF638650 is the command
 * register and 0xFF638654 the status whose low nibble echoes it; the
 * sequence walks 1 -> 2 (wait 2) -> DFI update via 0xFF638628 -> 4
 * (wait 4) -> 0xFF638648 = 0x31, then copies the ACS dmc_remap table
 * into the DMC's own address-remap window.
 *
 * No PHY APB access, no DRAM access, no mode-register read: T10 cannot
 * change the host-MRR-gates-CA-training position.
 */
/*
 * T10's three waits are UNBOUNDED in the blob and were unbounded here, which
 * makes "the DMC never answered" indistinguishable from "the DMC was slow" --
 * both are silence after `>ta`.  Boot replay-2 (2026-08-30) stopped in exactly
 * that silence, with the pre-T10 DMC window (`OPD ff638600..ff638670`) and the
 * post-T7 PHY state (`PPT t7 mux 0000 ps 0000/0000 st 0b0f 0b03 0b03 0b0f`)
 * BYTE-IDENTICAL to the known-good `allblobsoff` boot -- so the readable state
 * did not name the step.
 *
 * Bound each wait and print which one stopped, with the register it was
 * watching: `RPt <id> <ok> <spins> <value>`.  A bound cannot break anything a
 * spin was protecting -- it only converts an invisible hang into a reading --
 * and continuing past it lets the rest of the tail report whether it is
 * reachable at all.
 */
#if G12A_OPEN_REPLAY
#define G12A_T10_SPIN_MAX	200000u		/* microseconds */
#define G12A_T10_SPIN(id, pa, mask, want)				\
	do {								\
		u32 _n = 0, _v;						\
									\
		while ((_v = readl((void *)(pa))) != 0xffffffffu &&	\
		       (_v & (mask)) != (want) && _n < G12A_T10_SPIN_MAX) { \
			g12a_open_delay_us(1);				\
			_n++;						\
		}							\
		g12a_uart_puts("RPt ");					\
		g12a_uart_putc(id);					\
		g12a_uart_putc(' ');					\
		g12a_uart_hex8((u8)(((_v & (mask)) == (want)) ? 1 : 0));\
		g12a_uart_putc(' ');					\
		g12a_uart_hex32(_n);					\
		g12a_uart_putc(' ');					\
		g12a_uart_hex32(_v);					\
		g12a_uart_puts("\n");					\
	} while (0)
#endif

static int g12a_open_t10(ulong ctl, const u8 *ds)
{
	u32 i;

#if !G12A_OPEN_REPLAY
	(void)ctl;
#endif
	if (ds[14] == 2u && (ds[18] == 3u || ds[18] == 8u))	/* fffb8b04 */
		writel(0x0018db6bu, (void *)0xFF638624UL);	/* fffb8b34 */

	writel(1, (void *)0xFF638650UL);			/* fffb8b44 */
	writel(2, (void *)0xFF638650UL);			/* fffb8b54 */
#if G12A_OPEN_REPLAY
	G12A_T10_SPIN('1', 0xFF638654UL, 0xfu, 2u);		/* fffb8b58 */
#else
	while ((readl((void *)0xFF638654UL) & 0xfu) != 2u)	/* fffb8b58 */
		g12a_open_delay_us(1);				/* fffb8b74 */
#endif

	writel(0x4140u, (void *)0xFF638628UL);			/* fffb8b88 */
	g12a_open_delay_us(1);					/* fffb8b90 */
	writel(0x4141u, (void *)0xFF638628UL);			/* fffb8ba0 */
	g12a_open_delay_us(1);					/* fffb8ba8 */
#if G12A_OPEN_REPLAY
	G12A_T10_SPIN('2', 0xFF638628UL, 0x80000000u, 0x80000000u); /* fffb8bb8 */
#else
	while (!(readl((void *)0xFF638628UL) & 0x80000000u))	/* fffb8bb8 */
		;
#endif
	writel(0x4140u, (void *)0xFF638628UL);			/* fffb8bc8 */
	g12a_open_delay_us(0x21);				/* fffb8bd0 */

	writel(4, (void *)0xFF638650UL);			/* fffb8be0 */
#if G12A_OPEN_REPLAY
	G12A_T10_SPIN('3', 0xFF638654UL, 0xfu, 4u);		/* fffb8be4 */
#else
	while ((readl((void *)0xFF638654UL) & 0xfu) != 4u)	/* fffb8be4 */
		;
#endif
	writel(0x31u, (void *)0xFF638648UL);			/* fffb8c08 */

#if G12A_OPEN_REPLAY
	/*
	 * THE MODE-A ARM, fffb8c34-fffb8ce8, now TRANSCRIBED rather than
	 * approximated.  fffb8c14's b.eq takes the ctl[612] == 0xFF path to
	 * fffb8c34; it is not a bare call to fffbb37c, it is a DMC handshake
	 * BRACKET with the call inside it.  Nothing else on this boot issues an
	 * MRS: the PMU would have done it out of the message block, and the PMU
	 * never runs.
	 *
	 * amlogic/software/boot/g12/audit/2026-08-30-open-mode-a-replay.md
	 * sec.6.1 carries the disassembly, per instruction.
	 *
	 * WHY THIS IS THE FIX.  Boot replay-3 issued all 22 MRs here with the
	 * command register 0xFF638650 still in state 4 -- the state T10's own
	 * spin 3 leaves, and (see below) the state this bracket ENDS in -- so
	 * every one of them went into a CLOSED window and every completion poll
	 * on 0xFF6394E0 bit 31 timed out at the 200 ms bound.  That bit is
	 * polled by g12a_open_mrs() itself (fffbb330/fffbb334); the bracket
	 * never touches it.  The bracket does not do the waiting -- it puts the
	 * DMC into the state in which an MRS is legal and can complete.
	 *
	 * NOT the same bracket as g12a_open_mr_write()'s (fffbb6ac).  That one
	 * polls 0xFF638654[7:4] and runs 1 -> wait 1 ... 4 -> wait 2.  This one
	 * polls the LOW nibble [3:0] (fffb8c50 `and w1, w1, #0xf`) and runs
	 * five waits: 1 -> wait 1, the call, then 4 -> wait 4, 2 -> wait 2,
	 * 4 -> wait 4.  0xFF638654 carries both nibbles independently and boot
	 * replay-3 measured them together (RPt 3 ... 00000024: [3:0]=4,
	 * [7:4]=2), which is what made the two look interchangeable.
	 *
	 * Bounded and crumbed for the reason every wait on this path is: an
	 * unbounded spin and an absent step produce identical silence, and that
	 * cost two boots in this arc.  Crumb ids a/b/c/d do not collide with
	 * T10's 1/2/3, the MRS m/M, or fffbb6ac's w/W.
	 */
	writel(1, (void *)0xFF638650UL);			/* fffb8c3c/40 */
	G12A_T10_SPIN('a', 0xFF638654UL, 0xfu, 1u);		/* fffb8c44-58 */

	/*
	 * fffb8c5c-68: arg1 = (ctl[615] != 0).  fffb8c64's cbnz skips the
	 * `mov w1, w2`, leaving w1 at the poll loop's exit value, which
	 * fffb8c50/54 fix at 1.  g12a_open_mr_init() reads ctl[615] itself,
	 * so there is no argument to pass.
	 */
	g12a_open_mr_init(ctl, ds);				/* fffb8c6c */

	writel(4, (void *)0xFF638650UL);			/* fffb8c78/7c */
	G12A_T10_SPIN('b', 0xFF638654UL, 0xfu, 4u);		/* fffb8c80-94 */
	writel(2, (void *)0xFF638650UL);			/* fffb8ca0/a4 */
	G12A_T10_SPIN('c', 0xFF638654UL, 0xfu, 2u);		/* fffb8ca8-bc */
	writel(4, (void *)0xFF638650UL);			/* fffb8cc8/cc */
	G12A_T10_SPIN('d', 0xFF638654UL, 0xfu, 4u);		/* fffb8cd0-e4 */
	/* fffb8ce8 b fffb8c18 -- rejoin the common tail, below. */
#endif

	/*
	 * fffb8c18-24, the COMMON tail: 0xFF638648 = 0x31 a SECOND time.  Both
	 * arms reach it -- mode B by falling through fffb8c14, mode A by
	 * fffb8ce8's branch -- so on the mode-A path it lands AFTER the
	 * bracket, not back-to-back with the fffb8c08 write.  In a non-replay
	 * build the block above is empty and the two stores stay adjacent, as
	 * they were.
	 */
	writel(0x31u, (void *)0xFF638648UL);			/* fffb8c24 */

	/* fffb8cec..fffb8d18 -- ds->dmc_remap[5] twice, ten words. */
	for (i = 0; i < 10u; i++) {
		u32 k = 0x80u + (i % 5u) * 4u;

		writel((u32)ds[k] | ((u32)ds[k + 1] << 8) |
		       ((u32)ds[k + 2] << 16) | ((u32)ds[k + 3] << 24),
		       (void *)(0xFF639340UL + i * 4u));
	}
	return 1;						/* fffb8d1c */
}

/* ---------------------------------------------------------- T11 / T12 */

static void g12a_open_t11(void)
{
	writel(0xffffffffu, (void *)0xFF639130UL);
	writel(0xffffffffu, (void *)0xFF639134UL);
	writel(0xffffffffu, (void *)0xFF639138UL);
	writel(0xffffffffu, (void *)0xFF63913CUL);
	writel(0xffffffffu, (void *)0xFF639230UL);
	writel(0xffffffffu, (void *)0xFF639234UL);
	writel(0xffffffffu, (void *)0xFF639238UL);
	writel(0xffffffffu, (void *)0xFF63923CUL);
	writel(0, (void *)0xFF639000UL);
}

static void g12a_open_t12(const u8 *ds)
{
	writel(1, (void *)0xFF638668UL);			/* fffb8d44 */
	writel(0x31030820u, (void *)0xFF63810CUL);		/* fffb8d58 */
	if (ds[14] == 2u) {					/* fffb8d5c-64 */
		writel(0xc10003e8u, (void *)0xFF63865CUL);	/* fffb8d78 */
		g12a_open_delay_us(1000);			/* fffb8d80 */
		writel(0x010186a0u, (void *)0xFF63865CUL);	/* fffb8d94 */
	}
	writel(0x00ffffffu, (void *)0xFF638000UL);		/* fffb8da4 */
	writel(0xbaadf00du, (void *)0xFFD08140UL);		/* fffb8db8 */
	asm volatile("isb; dmb sy");				/* fffb8dc0/c4 */
}

/* ---------------------------------------------------------------- T14 */

/* fffbb238 -- one MRS through the DMC. */
static void g12a_open_mrs(ulong ctl, const u8 *ds, u32 rank, u32 mr, u32 val)
{
	u32 w4, w1;

	writel(0x000a000au, (void *)0xFF6394E8UL);		/* fffbb238 */
	if (ds[18] == 4u)					/* fffbb2d0- */
		w4 = 2;
	else if (ds[18] != 1u)
		w4 = 4;
	else
		w4 = 2;
	if (readb((void *)(ctl + 610))) {			/* fffbb300 */
		w1 = val | ((rank + 1u) << 24) | (mr << 16) |
		     ((w4 - 1u) << 26);
		writel(w1, (void *)0xFF6394E4UL);		/* fffbb320 */
		writel(1, (void *)0xFF6394E0UL);		/* fffbb330 */
#if G12A_OPEN_REPLAY
		/* Bounded for the same reason T10's waits are -- see the
		 * G12A_T10_SPIN comment.  In mode A this is the FIRST traffic
		 * ever driven at the DRAM, so a silent spin here is the most
		 * expensive thing the arm can do. */
		G12A_T10_SPIN('m', 0xFF6394E0UL, 0x80000000u, 0x80000000u);
#else
		while (!(readl((void *)0xFF6394E0UL) & 0x80000000u))
			;					/* fffbb334 */
#endif
	} else {						/* fffbb348 */
		writel(val | 0x80000000u | (mr << 16),
		       (void *)0xFF638664UL);
#if G12A_OPEN_REPLAY
		G12A_T10_SPIN('M', 0xFF638664UL, 0x80000000u, 0u);
#else
		while (readl((void *)0xFF638664UL) & 0x80000000u)
			;					/* fffbb35c */
#endif
	}
}

/* fffbb6ac, LPDDR4 arm -- the quasi-dynamic bracket an MRS needs. */
static void g12a_open_mr_write(ulong ctl, const u8 *ds, u32 hi, u32 lo)
{
	u32 v = lo | (hi << 6);					/* fffbb6d8 */

	writel(1, (void *)0xFF638650UL);			/* fffbb70c */
	g12a_open_delay_us(1);					/* fffbb71c */
#if G12A_OPEN_REPLAY
	G12A_T10_SPIN('w', 0xFF638654UL, 0xf0u, 0x10u);		/* fffbb720 */
#else
	while (((readl((void *)0xFF638654UL) >> 4) & 0xfu) != 1u)
		;						/* fffbb720 */
#endif
	g12a_open_mrs(ctl, ds, 0, 14, v & 0xffffu);		/* fffbb7e8 */
	if (g12a_open_h(ds, 42))				/* fffbb7fc */
		g12a_open_mrs(ctl, ds, 1, 14, v & 0xffffu);
	writel(4, (void *)0xFF638650UL);			/* fffbb81c */
	g12a_open_delay_us(1);
#if G12A_OPEN_REPLAY
	G12A_T10_SPIN('W', 0xFF638654UL, 0xf0u, 0x20u);		/* fffbb834 */
#else
	while (((readl((void *)0xFF638654UL) >> 4) & 0xfu) != 2u)
		;						/* fffbb834 */
#endif
	g12a_open_delay_us(1);
}

/*
 * fffb4bc4 -- mode 1 programs the 72 per-lane VrefDAC CSRs from the
 * harvested ds[628...]; mode 2 averages ds[664...] into MR14.
 *
 * The `v == 0` skip in mode 1 is load-bearing: a lane whose harvested
 * vref is zero is LEFT at whatever the PHY holds, not forced to zero.
 * The mode-2 mean is over non-zero entries only (both counter and
 * accumulator sit inside the cbz), so dividing by 32 gives a different
 * MR14 on any part with a masked bit.
 *
 * The one store dropped on purpose is tm[300] (fffb4d6c-7c), the MR14
 * slot: it lies past the 64-word tm this TPL carries and its only reader
 * in the blob is fffbb37c, which is unreachable in mode B.
 */
static void g12a_open_t14_vref(ulong ctl, u8 *ds, u32 mode)
{
	u32 i, sum = 0, n = 0, avg, mr = 0;

	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);	/* fffb4be0 */
	if (mode == 1u) {
		u32 live = 0;

		for (i = 0; i < 32; i++) {			/* fffb4c10-cb0 */
			u32 v = ds[628 + i];
			u32 lane, inst, c;

			if (!v)					/* fffb4c20 */
				continue;
			live++;
			lane = i & 7u;
			inst = i >> 3;
			c = 0x010040u | (inst << 12) | (lane << 8);
			g12a_open_csr_w(c, v);			/* VrefDAC0 */
			g12a_open_csr_w(c - 0x10u, v);		/* VrefDAC1 */
			if (lane == 7u) {			/* fffb4c5c */
				g12a_open_csr_w(c + 0x100u, v);
				g12a_open_csr_w(c + 0x0f0u, v);
			}
		}
		g12a_uart_puts("OPv1 lanes ");
		g12a_uart_hex8((u8)live);
		g12a_uart_puts("\n");
		return;
	}

	for (i = 0; i < 32; i++) {				/* fffb4cd0-cf0 */
		u32 v = ds[664 + i];

		if (v) {
			n = (n + 1u) & 0xffu;
			sum += v;
		}
	}
	avg = n ? (sum / n) : 0u;				/* fffb4cf4 */
	if (avg && ds[14] == 2u) {				/* fffb4cf8/4d48 */
		mr = (avg <= 0x1du) ? avg
				    : (((avg - 0x1eu) | 0x40u) & 0xffu);
		g12a_open_mr_write(ctl, ds, mr >> 6, mr & 0x3fu);
	}
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);	/* fffb4d9c */
	g12a_open_csr_w(0x010082u, avg);			/* fffb4da4-db0 */
	g12a_uart_puts("OPv2 n ");
	g12a_uart_hex8((u8)n);
	g12a_uart_puts(" avg ");
	g12a_uart_hex8((u8)avg);
	g12a_uart_puts(" mr14 ");
	g12a_uart_hex8((u8)mr);
	g12a_uart_puts("\n");
}

/* ------------------------------------------------- the two drivers */

/*
 * The two per-DQ-bit vref banks the UCT harvest fills: ds[628...659] (the
 * SoC/PHY receive vref, consumed by fffb4bc4 mode 1 -> 72 VrefDAC CSRs) and
 * ds[664...695] (the DRAM-side VREF-DQ, averaged into MR14 by mode 2).
 * Dumped identically from both arms so the open harvest can be judged
 * against the blob's own, byte for byte.
 */
/*
 * C-MRL -- the four DBYTEs' DFIMRL as 1D/2D training left them, read BEFORE
 * T7 item 16 folds them to one value.  Four reads, no write, and NOT a
 * csrtab.inc entry: tbl= is a CRC over the generated address sequence, so a
 * capture SITE does not move it and every landed v3 capture stays
 * position-comparable (rxen-dtsm-boot-plan.md sections 1.2, 5.4).
 *
 * What the four values decide: C-MRL-1 all 0x0007 (the PMU already
 * equalises, so item 16 is a no-op in production too); C-MRL-2 they differ
 * with max 7 (the fold is load-bearing); C-MRL-3 any read >= 8 (the clamp is
 * BITING on the reference board); C-MRL-4 any read >= 0x100 (the fold's
 * in-loop uxtb makes the answer DBYTE-order dependent).  Reading 0xFFFF, or
 * failing to decode, is the abort -- the capture is on the wrong window.
 */
static void g12a_open_dfimrl_dump(const char *tag)
{
	u32 i;

	g12a_uart_puts("OPM ");
	g12a_uart_puts(tag);
	for (i = 0; i < 4; i++) {
		g12a_uart_putc(' ');
		g12a_uart_hex16(g12a_open_csr_r(0x10020u | (i << 12)));
	}
	g12a_uart_puts("\n");
}

#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
/* ZK proved the micro CONSUMES the doorbell (mailbox reads back 0 right
 * after the post-release ring) yet never posts. Dump both mailbox views
 * and the shadow regs at every poll exit. Crumb: MB <tag> <95> <96> <32> <04>
 */
static void g12a_open_mbox_dump(const char *tag)
{
	g12a_uart_puts("MB ");
	g12a_uart_puts(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x000c0095u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x000c0096u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x000d0032u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x000d0004u));
	g12a_uart_puts("\n");
}
#endif

static void g12a_open_vref_dump(const char *tag)
{
	const u8 *ds = (const u8 *)G12A_DDR_SET_BASE;
	u32 b, i;

	for (b = 0; b < 2; b++) {
		g12a_uart_puts("OPV ");
		g12a_uart_puts(tag);
		g12a_uart_putc(' ');
		g12a_uart_hex16((u16)(b ? 664 : 628));
		for (i = 0; i < 32; i++) {
			g12a_uart_putc(' ');
			g12a_uart_hex8(ds[(b ? 664 : 628) + i]);
		}
		g12a_uart_puts("\n");
	}
}

/**
 * g12a_open_compare - read-only equivalence pass against the blob's output
 *
 * Runs AFTER the blob's own a644 walk, so every open computation is
 * judged against the state the vendor code actually left.  A pure read;
 * it cannot perturb the boot, which is why a formula error here is a
 * result rather than a hazard.
 */
static void g12a_open_compare(ulong ctl)
{
	const u8 *ds = (const u8 *)G12A_DDR_SET_BASE;

	(void)ctl;
	g12a_uart_puts("OPCMP begin\n");
	g12a_open_s4_phy_static(ds, 0);
	g12a_open_s2s5_timing_mid1(ds, 0);
	g12a_open_s3_msgblock(ds, 0, 0);
	g12a_uart_puts("OPCMP end\n");
}

/**
 * g12a_open_stage1 - a644 stage 1, S2..S19, with no call into the blob
 *
 * Return: the 1D train's own return code (0 = the micro reported success).
 */
#if G12A_OPEN_ASSERT_ID
/*
 * Which pass failed and the assert it published, latched AT the failure
 * (see g12a_open_fail_pass store in stage 1). .data: the retry decision
 * reads them before dram_init's .bss clear_loop, like g12a_mid1_live.
 */
static u8 g12a_open_fail_pass __section(".data");
static u32 g12a_open_fail_aid __section(".data");
#endif

static int g12a_open_stage1(ulong ctl)
{
	u8 *ds = (u8 *)G12A_DDR_SET_BASE;
	int r;

#if G12A_OPEN_ASSERT_ID
	g12a_open_fail_pass = 0;
	g12a_open_fail_aid = 0;
#endif

	g12a_tsm("s1");
	g12a_uart_puts("OPEN s1\n");
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	/* qi:N1294 black-box: one crumb per step so a silent death names
	 * its step. Arms below (overlay/caps/dumps) are the same probe. */
	g12a_probe_cfg_overlay((ulong)ds);		/* cfg-from-upload */
#endif
	g12a_open_s2s5_timing_mid1(ds, 1);		/* S2 + S5 */
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_uart_puts("PB s25 done\n");
#endif
	g12a_open_s3_msgblock(ds, 0, 1);		/* S3 */
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_uart_puts("PB s3\n");
	g12a_open_s4_phy_static(ds, 1);			/* S4 */
	g12a_uart_puts("PB s4\n");
#else
	g12a_open_s4_phy_static(ds, 1);			/* S4 */
#endif
	g12a_open_s6s7_pinmux(ds);			/* S6 + S7 */
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_uart_puts("PB s67\n");
	/* The decisive diff vs production world=p site=8d7c (a working
	 * S905D3 capture). MUST run while MicroContMuxSel=0 -- the kick
	 * entry hangs the APB (mux is 1 there, sweep out of spec: that
	 * death was measured on n1294b-resj2: ZW printed, then silence).
	 * State here misses only S9's load + doorbell, both independently
	 * verified; the SRAM line of this capture re-verifies the load.
	 */
	/* The decisive diff vs production world=p site=8d7c. NOT in TPL:
	 * stage1 runs this same train at 24 MHz before DDR is live, where
	 * a 1949-read walk hangs the SoC into a boot loop (n1294b-resv,
	 * -resw, -resx: two `OPEN s1` + TPL banner, no stage2). Stage2
	 * (SPL) captures with muxsel=0, buffers, replays slowly.
	 */
	/* v3-diff first divergence on this SKU: the Ucclk/Hclk enable
	 * (ST restore_sequence.c:51 writes 0x3 at TDRTUB|0x80) was NEVER
	 * set by the OPEN arm -- vendor site=8d7c reads 0d0004=0003, we
	 * read 0000. All earlier KQ attempts wrote abs 0xFE0C0080, off
	 * the csr<<1 aperture = the experiment never ran. Do it in the
	 * proven csr-id world, BEFORE the capture records the state:
	 */
	/* v3-diff: 0d0004=0 here = a UCT message from the STAGE-1 kick
	 * is still undrained in the shadow regs. C9: the micro blocks
	 * FOREVER on its next message post until the host reads this
	 * one = a silent second-kick death that matches every symptom.
	 * Drain all pending messages (C9 sequence = g12a_open_uct_read)
	 * BEFORE ringing the stage-2 doorbell:
	 */
	{
		u32 mv;
		int k;

		for (k = 0; k < 8; k++) {
			if (g12a_open_uct_read(1u, &mv))
				break;
			g12a_uart_puts("DR DR ");
			g12a_uart_hex32(mv);
			g12a_uart_puts(" 0\n");
		}
	}
	/* v3-diff decisive: UctWriteProt(0c0033)/UctDatWriteProt(0c0035)
	 * read 0 here, vendor 1 -- stage-1's message loop died between
	 * the ack write and the final re-arm (OPU t2 path returns with
	 * protection disarmed), leaving the mailbox wedged for every
	 * later post. Re-arm all three write-protect copies, then KQ:
	 */
	g12a_open_csr_w(0x000c0033u, 1u);
	g12a_open_csr_w(0x000c0035u, 1u);
	g12a_open_csr_w(0x000d0031u, 1u);
	g12a_uart_puts("WA WA ");
	g12a_uart_hex16(g12a_open_csr_r(0x000c0033u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x000c0035u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x000d0031u));
	g12a_uart_puts(" 0\n");
	g12a_open_csr_w(0x000c0080u, 0x3u);
	g12a_uart_puts("KQ KQ ");
	g12a_uart_hex16(g12a_open_csr_r(0x000c0080u));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_open_csr_r(0x000d0004u));
	g12a_uart_puts(" 0\n");
	g12a_uart_puts("PB x1\n");
	g12a_pcsr_capture((ulong)ds, 0);
	g12a_uart_puts("PB x2\n");
#endif

#if G12A_OPEN_REPLAY
	/*
	 * MODE A.  fffbe700-fffbe70c reads ctl[612], compares 0xFF and branches
	 * straight to fffbea04 -- the stage-2 head -- so S9 through S19 are
	 * SKIPPED WHOLE: no fffb6294 (PHY IMEM/DMEM stage), no fffb8dd4 (micro
	 * kick), no mailbox, and none of the three train passes.  No byte of
	 * lpddr4_1d.fw or lpddr4_2d.fw is read, and the micro is never started.
	 */
	g12a_uart_puts("RPs1\n");
	(void)r;
	return 0;
#else
	g12a_open_s9_load();				/* S9 */
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_uart_puts("PB s9\nQ9 ");
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MICROCONTMUXSEL));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MEMRESETL));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_IMEM_BASE));
	g12a_uart_putc(' ');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_DMEM_BASE));
	g12a_uart_puts("\n");
#endif

	/* S10 -- pass A, gated on ACS SequenceCtrl bit 12 (fffbe718) */
	if (g12a_open_h(ds, 44) & (1u << 12)) {
		g12a_open_csr_w(0x054008u, 0x1001);	/* 0xFE0A8010 */
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
		g12a_uart_puts("ZW ");
		g12a_uart_hex32(g12a_open_csr_r(0x054008u));
		g12a_uart_puts("\n");
		if (!(g12a_open_csr_r(0x054008u) & 0x1000u)) {
			/* the copy16 recipe: the micro owns the msgblock
			 * bus; flip ownership to APB around the doorbell.
			 */
			g12a_uart_puts("ZX\n");
			g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
			g12a_open_csr_w(0x054008u, 0x1001);
			g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 1);
			g12a_uart_puts("ZX ");
			g12a_uart_hex32(g12a_open_csr_r(0x054008u));
			g12a_uart_puts("\n");
		}
#endif
		r = g12a_open_train_kick(ds);
		#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_open_mbox_dump("pa");
#endif
	g12a_uart_puts("OPEN pa ");
		g12a_uart_hex8((u8)r);
		g12a_uart_puts("\n");
#if G12A_OPEN_ASSERT_ID
		/* Latch pass A's identity AT the failure, before the PROBE
		 * re-kick diagnostic below drains the mailbox and moves
		 * g12a_open_assert_hdr on: the retry decision must never
		 * read the live latch -- it would name the diagnostic's
		 * assert, not the train's.
		 */
		if (r) {
			g12a_open_fail_pass = 'a';
			g12a_open_fail_aid = g12a_open_assert_hdr;
		}
#endif
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
		/* Diagnostic: is the micro responsive to ANY sequence?
		 * Pass B doorbell (0x10f, fffbe80c) after a failed pass A.
		 */
		if (r) {
			int r2;

			g12a_open_csr_w(0x054008u, 0x010f);
			r2 = g12a_open_train_kick(ds);
			#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_open_mbox_dump("pbp");
#endif
	g12a_uart_puts("OPEN pbp ");
			g12a_uart_hex8((u8)r2);
			g12a_uart_puts("\n");
		}
#endif
		if (r)
			return r;
	}

	g12a_open_s11s12_acdelay(ds);			/* S11 + S12 */

	g12a_open_csr_w(0x054008u, 0x010f);		/* S13 */
	r = g12a_open_train_kick(ds);
	#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_open_mbox_dump("pb");
#endif
	g12a_uart_puts("OPEN pb ");
	g12a_uart_hex8((u8)r);
	g12a_uart_puts("\n");
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	if (r)
#endif
	if (r)
		return r;

	/* S14/S15 are class-6 SAVEs -- omitted, see the file header. */

	/* S16/S17/S17a -- gated on ds[8] bit 26 == 0, which the ACS sets */
	if (!(((u32)ds[8] | ((u32)ds[9] << 8) | ((u32)ds[10] << 16) |
	       ((u32)ds[11] << 24)) & (1u << 26)))
		g12a_open_s17a_dbyte_fixup();

	/* S18 + S19 -- pass C, mandatory in mode B */
	g12a_open_csr_w(0x054008u, 0x0ef9u & g12a_open_h(ds, 44));
	r = g12a_open_train_kick(ds);
	#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	g12a_open_mbox_dump("pc");
#endif
	g12a_uart_puts("OPEN pc ");
	g12a_uart_hex8((u8)r);
	g12a_uart_puts("\n");
	return r;
#endif	/* G12A_OPEN_REPLAY */
}

/**
 * g12a_open_stage2 - a644 stage 2, T1..T14, minus T10
 * @has2d: whether the 2D PMU image is on the wire
 *
 * T10 (fffb8af0) is NOT transcribed and is still the blob; the caller
 * supplies it.  Everything else here is ours.
 */
static int g12a_open_stage2_pre(ulong ctl)
{
	u8 *ds = (u8 *)G12A_DDR_SET_BASE;

	g12a_tsm("s2");
	g12a_uart_puts("OPEN s2\n");
	g12a_open_s2s5_timing_mid1(ds, 1);		/* T1 + T2 */
	return 0;
}

static int g12a_open_stage2_2d(ulong ctl)
{
	u8 *ds = (u8 *)G12A_DDR_SET_BASE;
	int r;

	g12a_tsm("2d");
	g12a_uart_puts("OPEN 2d\n");
	g12a_copy32(G12A_FW_HOLD_1D, G12A_FW_HOLD_2D, G12A_FW_HOLD_2D_MAX);
	g12a_open_s3_msgblock(ds, 1, 1);		/* T5b prepare */
	g12a_open_s9_load();				/* T5b reload */
	r = g12a_open_train_kick(ds);			/* T5b train */
	g12a_uart_puts("OPEN 2d ");
	g12a_uart_hex8((u8)r);
	g12a_uart_puts("\n");
	if (r)
		return r;
	g12a_open_t14_vref(ctl, ds, 1);			/* T5b fffb4bc4(1) */
	/* fffb94d0 and fffba544 are gated off by ds[8] bit 29 / ds[127] */
	return 0;
}

/*
 * T11..T14.  The caller runs T7 and then the blob's T10 between them,
 * because fffb8af0 is the one stage-2 step that is not transcribed.
 */
static void g12a_open_stage2_post_tail(ulong ctl)
{
	u8 *ds = (u8 *)G12A_DDR_SET_BASE;
	u16 v;

	g12a_open_t11();				/* T11 */
	g12a_open_t12(ds);				/* T12 */

	/* T13 -- inline in a644; two reads, two stores, no helper. */
	v = (u16)(g12a_phy_read16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS0) |
		  0x20u);
	g12a_phy_write16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS0, v);
	v = (u16)(g12a_phy_read16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS0) |
		  0x20u);
	g12a_phy_write16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS1, v);

#if G12A_OPEN_BLOB_T14
	g12a_t_call2('e', G12A_AML_FN_T14_VREF, ctl, 2);
#else
	g12a_open_t14_vref(ctl, ds, 2);			/* T14 */
#endif
	g12a_tsm("2dend");
	g12a_uart_puts("OPEN s2 end\n");
}
#endif /* CONFIG_MESON_G12A_DRAM_OPEN */

/*
 * ---------------------------------------------------------------------
 * a644 stage 2, walked step by step — the part that brings the DDR
 * controller up.  Our stepwise stage-1 walk stops at the 1D train kick,
 * so nothing had ever programmed the DMC or touched a byte of DRAM.
 *
 * Sequence and argument registers are transcribed from the a644 tail
 * (aml_ddr.fw, md5 eddca53f01322e2a7f4a8aca1b89ea3a, disassembled with
 * --adjust-vma=0xFFFB4000):
 *
 *   fffbea04  mov x0,x19 / strb wzr,[x19,#441] / bl 0xfffb6a1c   T1
 *   fffbea14  bl 0xfffb6328                                      T2
 *   fffbea90  bl 0xfffb5bc4                                      T7
 *   fffbea94  mov w1,#1 / bl 0xfffb5a54                          T8
 *   fffbeaa4  bl 0xfffbbbb0                                      T9
 *   fffbeaac  bl 0xfffb8af0                                      T10
 *   fffbeab0  bl 0xfffb61c8                (no x0 setup)         T11
 *   fffbeab8  bl 0xfffb8d2c                                      T12
 *   fffbeabc..fffbeaf8  0xFE040048 |= 0x20, then that value      T13
 *                       stored to 0xFE240048
 *   fffbeb00  mov w1,#2 / bl 0xfffb4bc4                          T14
 *
 * T5b (the 2D pass — fffbe244, fffb6294, fffb8dd4, fffb4bc4(1),
 * fffb94d0, fffba544) is NOT run here and is NOT claimed: it reloads
 * PHY IMEM/DMEM from 0xFFFC0000 / 0xFFFC8000, which on this delivery
 * still hold the 1D image.  The 2D blob is not on the wire — the
 * residual carries one G12R frame of aml_ddr||piei||1d.  Running T5b
 * against the 1D image would re-run 1D under a 2D message block and
 * report a "2D pass" that never happened.
 *
 * Crumb form: ">tX" before each step, "<tX rr" after, so the last ">tX"
 * without a matching "<tX" names the step that stops.
 */
static int g12a_t_call(char tag, ulong fn, ulong ctl)
{
	typedef int (*aml_ctx_fn)(ulong ctl);
	int r;

	g12a_uart_puts(">t");
	g12a_uart_putc(tag);
	g12a_uart_puts("\n");
	r = ((aml_ctx_fn)fn)(ctl);
	g12a_uart_puts("<t");
	g12a_uart_putc(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)(r & 0xff));
	g12a_uart_puts("\n");
	g12a_wdt_pet_safe();
	return r;
}

static int g12a_t_call2(char tag, ulong fn, ulong ctl, int arg)
{
	typedef int (*aml_ctx2_fn)(ulong ctl, int arg);
	int r;

	g12a_uart_puts(">t");
	g12a_uart_putc(tag);
	g12a_uart_puts("\n");
	r = ((aml_ctx2_fn)fn)(ctl, arg);
	g12a_uart_puts("<t");
	g12a_uart_putc(tag);
	g12a_uart_putc(' ');
	g12a_uart_hex8((u8)(r & 0xff));
	g12a_uart_puts("\n");
	g12a_wdt_pet_safe();
	return r;
}

#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
int meson_g12_usb_has_2d(void);
#else
static inline int meson_g12_usb_has_2d(void) { return 0; }
#endif

/**
 * g12a_stage2_2d - a644's T5b, the 2D training pass
 *
 * Runs between T2 and T7, exactly where a644 runs it (fffbea48..fffbea78).
 * PREPARE takes its 2D branch because ctl[PHASE] is already 2; LOAD_IMEM
 * reads FIXED 0xFFFC0000/0xFFFC8000, so the 2D image has to be moved there
 * first — the residual splitter parks it at HOLD0 (0xFFFD0000) for exactly
 * this copy.
 *
 * Crumbs: >2c/<2c staging copy (prints the first IMEM and DMEM halfwords
 * at 0xFFFC0000 / 0xFFFC8000 AFTER the copy, so a no-op copy is visible),
 * then tP tL tT tv tw tx for the six vendor calls.
 *
 * Return: the 2D train's own return code (0 = the micro reported success).
 */
static int g12a_stage2_2d(ulong ctl)
{
	int ret;

	g12a_uart_puts(">2c\n");
	g12a_copy32(G12A_FW_HOLD_1D, G12A_FW_HOLD_2D, G12A_FW_HOLD_2D_MAX);
	g12a_uart_puts("<2c ");
	g12a_uart_hex16(readw((void *)G12A_FW_HOLD_1D));
	g12a_uart_putc('/');
	g12a_uart_hex16(readw((void *)G12A_MSG_BLOCK_BASE));
	g12a_uart_puts("\n");

	g12a_t_call('P', G12A_AML_FN_PREPARE, ctl);
	g12a_t_call('L', G12A_AML_FN_LOAD_IMEM, ctl);
	ret = g12a_t_call('T', G12A_AML_FN_TRAIN, ctl);
	if (ret)
		return ret;

	g12a_t_call2('v', G12A_AML_FN_T14_VREF, ctl, 1);
	g12a_t_call('w', G12A_AML_FN_T5B_2D_POST_A, ctl);
	g12a_t_call('x', G12A_AML_FN_T5B_2D_POST_B, ctl);
	return 0;
}

static u32 g12a_stage2_finalize(ulong ctl)
{
	u16 v;
	u32 tot;

	g12a_uart_puts("S2\n");

	/*
	 * Mirror the state a644 is in when it reaches T1 on its second
	 * (mode-B) call: pstate 0, stage selector = 2.  We do not re-enter
	 * a644 itself because its stage-2 head would run T5b.
	 */
	g12a_poke8(ctl, 441, 0);
	g12a_poke8(ctl, G12A_CTL_OFF_PHASE, 2);

#if defined(CONFIG_MESON_G12A_DRAM_OPEN) && G12A_OPEN_LIVE
	g12a_open_stage2_pre(ctl);			/* T1 + T2 */
	g12a_d430("t2");
#else
	g12a_t_call('1', G12A_AML_FN_TIMING, ctl);
	g12a_t_call('2', G12A_AML_FN_MID1, ctl);
#endif

	/*
	 * T5b.  a644 gates T7..T14 on the 2D train's return (fffbea88:
	 * cbnz w20 -> exit), so a failing 2D pass must NOT be followed by the
	 * DMC bring-up: it would claim a controller brought up on margins the
	 * micro said it could not find.
	 */
#if defined(CONFIG_MESON_G12A_DRAM_OPEN) && G12A_OPEN_LIVE
#if G12A_OPEN_REPLAY
	/*
	 * MODE A: fffbea3c-fffbea44 compares ctl[612] with 0xFF and branches to
	 * fffbea80, skipping the ENTIRE 2D block (fffbea48 msg-block prepare,
	 * fffbea50 IMEM/DMEM stage, fffbea58 micro kick, fffbea68 vref average,
	 * fffbea70 DFI nudge, fffbea78 post-train shift) and landing on
	 * fffbea84 bl fffb5a54 with w1 = 0 -- the RESTORE.
	 */
	g12a_uart_puts("REPLAY2D\n");
#else
	if (meson_g12_usb_has_2d()) {
		if (g12a_open_stage2_2d(ctl) != 0) {
			g12a_uart_puts("2D fail\n");
			return 1;
		}
	} else {
		g12a_uart_puts("no2d\n");
	}
#endif	/* G12A_OPEN_REPLAY */
	g12a_d430("2d");
	g12a_open_vref_dump("open");
	g12a_open_dfimrl_dump("open");	/* C-MRL -- pre-fold, open arm */

	/*
	 * T7, T11, T12, T13 and T14 are ours; T8 and T9 are omitted with a
	 * measured reason (see the open-path header).  T10 -- fffb8af0, the
	 * DMC<->PHY handshake state machine -- is NOT transcribed and is the
	 * second and last blob call on this path.
	 */
#if G12A_OPEN_REPLAY
	/*
	 * T5a -- fffbea80/fffbea84: mov w1,#0 / bl fffb5a54.  THE REPLAY.
	 * The ten delay classes go from the captured ddr_set_t back into the
	 * PHY, through the same codec that produced them.
	 */
	{
		u8 *rds = (u8 *)G12A_DDR_SET_BASE;
		u32 s_pre;

		s_pre = g12a_open_ds_tailsum(rds);
		g12a_open_class_rw(rds, 0);
		g12a_uart_puts("RPr ");
		g12a_uart_hex32(s_pre);
		g12a_uart_puts("\n");
	}
#endif
#if G12A_OPEN_BLOB_T7
	g12a_t_call('7', G12A_AML_FN_T7_PHY_POST, ctl);
#else
	g12a_open_t7(ctl, (const u8 *)G12A_DDR_SET_BASE);
#endif
#if G12A_OPEN_REPLAY
	/*
	 * T8 -- fffbea94/fffbea9c: mov w1,#1 / bl fffb5a54.  The capture half,
	 * run in mode A exactly as in mode B, and here it is also the PROOF:
	 * restore(0) -> T7 -> capture(1) means the after-sum is a genuine
	 * read-back of the PHY through the codec that wrote it.  Two similar,
	 * non-trivial, unequal sums = the PHY answered.  All-zero or all-ones =
	 * the APB read path is dead and the writes went nowhere.
	 */
	{
		u8 *rds = (u8 *)G12A_DDR_SET_BASE;

		g12a_open_class_rw(rds, 1);
		g12a_uart_puts("RPc ");
		g12a_uart_hex32(g12a_open_ds_tailsum(rds));
		g12a_uart_puts("\n");
	}
#endif
	/*
	 * Site 2 -- T7 has returned, T8/T9/T10 have not run.  MicroContMuxSel
	 * was set to 0 by T7 item 2 in BOTH implementations, so the APB owns
	 * the CSR bus and the sweep is legal (PUB 7.2.1 rule 2).
	 */
	g12a_d430("t7");
	g12a_pcsr_capture(ctl, 2);
#ifdef CONFIG_TPL_MESON_G12A_USB_RESIDUAL
	g12a_ppt_crumb("t7");	/* G12-C1: after 1D+2D + T7 (open-live) */
#endif
	/*
	 * The DMC window T10 hands-hakes against, read immediately before it.
	 * 0xFF638650 is the command register and 0xFF638654 the status whose
	 * low nibble T10's first spin waits to read as 2 (fffb8b58-fffb8b78).
	 */
	{
		ulong pa;

		for (pa = 0xFF638600UL; pa < 0xFF638680UL; pa += 16) {
			u32 k;

			g12a_uart_puts("OPD ");
			g12a_uart_hex32((u32)pa);
			for (k = 0; k < 4; k++) {
				g12a_uart_putc(' ');
				g12a_uart_hex32(readl((void *)(pa + k * 4)));
			}
			g12a_uart_puts("\n");
		}
	}
#if G12A_OPEN_KEEP_T8T9
	/*
	 * Boot open-live-1 stopped inside the blob's T10 with no `<ta`.
	 * The only stage-2 deltas from the vendor at that point are our T7
	 * and the omission of T8/T9, so these two are restored as blob
	 * calls to separate those hypotheses in ONE boot.  T8's omission
	 * rested on class-restore-transcription.md sec.7.2 ("SAVE is CSR
	 * reads plus memory writes into ds, and nothing reads those bytes")
	 * -- if restoring it clears the hang, that reading is incomplete.
	 */
	g12a_t_call2('8', G12A_AML_FN_T8_DELAY_SAVE, ctl, 1);
	g12a_t_call('9', G12A_AML_FN_T9_CSR_OVERRIDE, ctl);
#endif
#if G12A_OPEN_BLOB_T10
	g12a_t_call('a', G12A_AML_FN_T10_DMC_PHY, ctl);
#else
	{
		int tr;

		g12a_uart_puts(">ta\n");
		tr = g12a_open_t10(ctl, (const u8 *)G12A_DDR_SET_BASE);
		g12a_uart_puts("<ta ");
		g12a_uart_hex8((u8)tr);
		g12a_uart_puts("\n");
	}
#endif
	g12a_d430("ta");
	g12a_pcsr_capture(ctl, 3);	/* site 3 -- post-T10, pre-DMC-go */
	g12a_dmc_dump();
#ifdef CONFIG_MESON_G12A_PPT_DDC
	g12a_ppt_ddc_arm();	/* G12-C3: after T8 save + T10 DMC ack */
#endif
	g12a_open_stage2_post_tail(ctl);
	v = 0;
	(void)v;
#else
	if (meson_g12_usb_has_2d()) {
		if (g12a_stage2_2d(ctl) != 0) {
			g12a_uart_puts("2D fail\n");
			return 1;
		}
	} else {
		g12a_uart_puts("no2d\n");
	}
#ifdef CONFIG_MESON_G12A_DRAM_OPEN
	g12a_open_vref_dump("blob");
	g12a_open_dfimrl_dump("blob");	/* C-MRL -- pre-fold, blob arm */
#endif

	g12a_t_call('7', G12A_AML_FN_T7_PHY_POST, ctl);
#ifdef CONFIG_TPL_MESON_G12A_USB_RESIDUAL
	g12a_ppt_crumb("t7");	/* G12-C1: after 1D+2D + PIE T7 */
#endif
#ifdef CONFIG_MESON_G12A_DRAM_OPEN
	g12a_pcsr_capture(ctl, 2);	/* site 2 -- post-T7, blob world */
#endif
	g12a_t_call2('8', G12A_AML_FN_T8_DELAY_SAVE, ctl, 1);
	g12a_t_call('9', G12A_AML_FN_T9_CSR_OVERRIDE, ctl);
	g12a_t_call('a', G12A_AML_FN_T10_DMC_PHY, ctl);
#ifdef CONFIG_MESON_G12A_DRAM_OPEN
	g12a_pcsr_capture(ctl, 3);	/* site 3 -- post-T10, blob world */
	g12a_dmc_dump();
#endif
#ifdef CONFIG_MESON_G12A_PPT_DDC
	g12a_ppt_ddc_arm();	/* G12-C3: after T8 save + T10 DMC ack */
#endif
	g12a_t_call('b', G12A_AML_FN_T11_DMC_CLR, ctl);
	g12a_t_call('c', G12A_AML_FN_T12_DMC_GO, ctl);

	/* T13 — inline in a644; two reads, two stores, no helper. */
	v = (u16)(g12a_phy_read16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS0) | 0x20u);
	g12a_phy_write16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS0, v);
	v = (u16)(g12a_phy_read16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS0) | 0x20u);
	g12a_phy_write16(G12A_DDR_PHY_DQSPREAMBLECONTROL_PS1, v);
	g12a_uart_puts("<td ");
	g12a_uart_hex16(v);
	g12a_uart_puts("\n");

	g12a_t_call2('e', G12A_AML_FN_T14_VREF, ctl, 2);
#endif

	/*
	 * slot5 / slot6 — the two calls BL2 makes AFTER a644 and that our
	 * path had never made.  BL2's cold LPDDR4 entry is
	 * fp_pll -> a644(1D) -> a644(2D) -> slot5 -> slot6.
	 *
	 * slot5 (export[5], payload VA 0xFFFB9DA8, D-VA 0xFFFB9E08) is NOT
	 * merely a memtest: it is the SIZE DETECT.  Its first act is
	 *   ds = ctl[160]; if (ds[40] == 0xFFFF) { print; ds[40] = 0x1000; }
	 *   if (ds[42] == 0xFFFF) { ds[42] = 0x1000; nrank = 2; }
	 * (fffb9e14..fffb9e78), and it then walks address bits 2..29 writing
	 * 0xAABBCCDD and reading back, finds the aliasing boundary, writes
	 * the discovered geometry into DMC 0xFF639368 (fffba030/fffba05c)
	 * and prints "DDR cs0 size:" / "DDR cs1 size:" / "DDR size:".
	 *
	 * Measured consequence of skipping it (boot stage2-1): the coarse
	 * aliasing walk failed on exactly 30 of 62 points with the readback
	 * equal to the value written 2 GiB higher — address bit 31 not
	 * decoded, i.e. the DMC left at half the part's size.
	 *
	 * G12A_KEEP_SLOT5=0 retires it.  Pair it with G12A_DS_SIZE_EXPLICIT=1:
	 * with the sentinels still in ds, S1 has programmed 0x00E0002D (4096 MB
	 * per rank) and nothing else corrects it, which is that same stage2-1
	 * failure.  With the density stated, S1 programs 0x00E00024 — slot5's
	 * own measured answer — and there is nothing left for it to discover.
	 * slot6 (export[6]) has its own independent gate, immediately below.
	 *
	 * G12A_OPEN_SLOT5 picks the IMPLEMENTATION when the step runs:
	 * 1 = g12a_open_slot5(), our transcription of the discovery half
	 * (crumbs ">tp5" / "OP5 ..." / "<tp5 00"); 0 = the blob (crumbs
	 * ">tm" / "<tm ..").  Both arms are meant to leave ds+0x28/0x2a,
	 * 0xFF639368 and AO_SEC_GP_CFG0 identical; ours does NOT run the
	 * blob's DataBus/AddrBus tail -- G12A_BUS_TEST is that half.
	 */
#if G12A_KEEP_SLOT5
#if G12A_OPEN_SLOT5
	g12a_open_slot5(ctl);
#else
	g12a_t_call('m', G12A_AML_FN_MAIN, ctl);
#endif
#else
	g12a_uart_puts("no-m\n");
#endif

	/*
	 * slot6 -- G12A_OPEN_SLOT6 picks the implementation, not whether the
	 * step happens: 1 = our transcription (crumbs ">tp6" / "<tp6 00" plus
	 * the "OP6 9254 <before> <after>" read-back), 0 = the blob (crumbs
	 * ">tp" / "<tp d0").  Default 0.  The two arms are meant to leave the
	 * same register state; a differing "<tp" crumb byte is expected,
	 * because ours drops the blob's watchdog arm and therefore does not
	 * inherit its junk return.
	 */
#if G12A_OPEN_SLOT6
	g12a_open_slot6(ctl);
#else
	g12a_t_call('p', G12A_AML_FN_POST, ctl);
#endif

	g12a_publish_mem_size(ctl);

	g12a_uart_puts("S2 end\n");

	g12a_tsm("quick");
	g12a_dram_txdqs_quick();
	g12a_tsm("dt");
	tot = g12a_dram_test();
	if (G12A_TXDQS_FORCE_A && !tot) {
		g12a_uart_puts("TXA forced\n");
		tot = 1;
	}
	if (g12a_dram_txdqs_rescue(tot))
		tot = g12a_dram_test();
	g12a_tsm("dtend");
	g12a_dram_margin();

	/*
	 * BL2's post-train DMC tail, the APD half: enable DMC auto power-down.
	 *
	 * DMC_DRAM_APD_CTRL is 0xFF638630 (BSP bl33/v2019/cmd/amlogic/cmd_ddr_test_g12.c:134,
	 * G12A row: ddr_dmc_apd_address = (0x008c << 2) + 0xff638400).  Bit
	 * layout from the same DMC block on A5 (BSP
	 * bl33/v2019/arch/arm/include/asm/arch-a5/register.h:1049):
	 * [9:0] idle DMC cycles before auto power-down (0 = APD off), [10]
	 * keep DFI_clk on in APD, [11] raise DFI_LP_REQ in APD, [12] slow exit
	 * (wait PLL lock), [19:16] DFI_LP_WAKEUP.  0x900 = APD after 0x100
	 * idle cycles, with DFI_LP_REQ.
	 *
	 * Vendor BL2 (bl2_new.bin md5 d552c567, VMA 0xFFFA1000) stores 0x900
	 * here at 0xFFFA7998, after slot5/slot6, and 0x1D00 instead at
	 * 0xFFFA79B4 only for DRAM type 0 (ds[14] == 0).  This TPL builds the
	 * ds as LPDDR4 (type 2) only, so that arm is dead.  Nothing else in
	 * our chain writes this word: it reads 0 after T10
	 * (s1-t10-transcription.md sec 8.2, DMCW ff638630 00000000).
	 *
	 * Why it matters: with APD off an idle DMC sits in DRAM_STAT[7:4] = 2
	 * (DRAM_ACCESS, same register.h:1113), and the vendor BL30 SR
	 * entry (bl30.bin 0x10003754) waits for 0 (IDLE) or 4 (APD) before
	 * it requests self-refresh.  Our SC carries the same wait.
	 *
	 * Placed last, after every PHY CSR access of this stage: the vendor
	 * DDR tool closes APD and ASR around PHY register reads and writes
	 * (bl33/v2019/cmd/amlogic/cmd_ddr_test_g12.c:9324, :9371).
	 */
	writel(0x900u, (void *)0xFF638630UL);	/* DMC_DRAM_APD_CTRL, fffa7998 */
	g12a_uart_puts("S2 apd ");
	g12a_uart_hex32(readl((void *)0xFF638630UL));
	g12a_uart_puts("\n");
	return tot;
}

/**
 * g12a_try_aml_ddr_entry - pinmux → prepare → force → full a644
 *
 * Vendor BL2 LPDDR4: PLL → soft-rst → pure 1d IMEM @ 0xFFFC0000 →
 * ddr_run_driver_init → ctl[3]=FUN_fffda644 (NOT open dual 4dc0).
 * a644 does: topology → timing → prepare → pinmux → mids → 2280 →
 * dual 4dc0 (0x1001 then 0x10f) with product UCT start.
 *
 * Open pre-calls nothing: ctl is built, then the vendor fp_pll, then a644 --
 * either walked step by step (G12A_BL2_ORDER_STEPWISE=1, the live path) or
 * called whole.
 * Crumbs: cb bp, then >1..>9 / <N per step, then S2.
 */
static int g12a_try_aml_ddr_entry(void)
{
	typedef int (*aml_ctx_fn)(ulong ctl);
	aml_ctx_fn fn;
	ulong ctl = G12A_DDR_CTL_BASE;
	int ret;

	if (!g12a_ddr_ctl_build()) {
		g12a_uart_puts("e\n");
		return -1;
	}

	/*
	 * Do what BL2 does, and nothing else.
	 *
	 * The product bootloader builds the control block completely and then
	 * calls fp_pll, a644 (1D), a644 (2D), slot5, slot6 — that is the whole
	 * cold LPDDR4 entry. We instead pre-call topology, timing, prepare and
	 * 2280 ourselves and THEN call a644, which runs all four again. Every
	 * one of those pre-calls is a chance to burn a one-shot or leave state
	 * a644 does not expect; fp_pll already showed the shape of that class
	 * of bug by hanging when called against an unbuilt ctl.
	 *
	 * This path is the control: ctl is fully built above, then fp_pll,
	 * then ONE a644. If the micro starts here, our pre-calls were the
	 * problem and the rest of this function is the bug. If it still does
	 * not, the pre-calls are exonerated and the fault is in the ctl/ACS
	 * contents rather than the call sequence — which is worth knowing
	 * either way, and is cheaper than arguing about it.
	 *
	 * Crumbs: bp = fp_pll returned, ba = a644 returned.
	 */
	/*
	 * CalBusy (CSR 0x20097 -> PA 0xFE04012E), which i.MX uses as a
	 * completion gate. Spec-derived test, not vendor mimicry: the
	 * Synopsys PhyInit reference states UcclkHclkEnables=0x3 is what lets
	 * DRTUB-domain reads complete. We measure the gates OPEN, APB and
	 * MASTER CSRs alive, and PHY SRAM dead — and a gate cannot manufacture
	 * a clock. That signature fits a missing SOURCE clock behind
	 * ucclk/hclk, which would equally stop the micro fetching.
	 *
	 * A value that never changes across init is the answer. Sampled either
	 * side of fp_pll, since fp_pll is what programs the DDR PLL.
	 * Also reads MemResetL, where the vendor's config step writes 3 —
	 * seeing 3 proves PhyInit step C ran, in a register we read fine.
	 */
	g12a_uart_puts("cb ");
	g12a_uart_hex16(g12a_phy_read16(0xFE04012EUL));
	g12a_uart_putc('/');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MEMRESETL));
	g12a_uart_puts("\n");

#if G12A_KEEP_FP_PLL
	fn = (aml_ctx_fn)G12A_AML_FN_PLL;
	g12a_uart_puts(">bp\n");
	ret = fn(ctl);
	g12a_uart_puts("bp ");
	g12a_uart_hex8((u8)(ret & 0xff));
	g12a_uart_puts("\n");
#else
	/*
	 * fp_pll skipped.  Read back its ENTIRE register footprint — the seven
	 * AM_DDR_PLL_CNTL words are all it writes on this board — so the
	 * verdict is a register comparison and not an outcome inference.
	 * Expected: CNTL0 = G12A_DDR_PLL_VAL_1608 | G12A_DDR_PLL_RST_ASSERT
	 * (0x1C100486) with bit 31 (lock) set, then CNTL1..6 equal to
	 * G12A_DDR_PLL_CNTL1_DEF .. CNTL6_DEF: 00000004 00000000 08691c00
	 * 88770290 39272000 50540000.
	 */
	{
		int k;

		g12a_uart_puts("nopll");
		for (k = 0; k < 7; k++) {
			g12a_uart_putc(' ');
			g12a_uart_hex32(readl((void *)(G12A_DDR_PLL_BASE +
						       (ulong)k * 4)));
		}
		g12a_uart_puts("\n");
	}
#endif

#if defined(CONFIG_MESON_G12A_DRAM_OPEN) && G12A_OPEN_LIVE
	/*
	 * THE OPEN PATH.  a644 is not called at all; its S2..S19 are
	 * performed here.  The ONE blob call left in stage 1 is S1
	 * (fffb931c, PLL / DMC address map + geometry), which is not
	 * transcribed -- it is also what sets ctl+610, the "DMC register
	 * access proved writable" flag the MRS path reads.
	 */
	{
		int oret;

#if G12A_OPEN_BLOB_S1
		g12a_t_call('t', G12A_AML_FN_TOPOLOGY, ctl);
#else
		g12a_open_s1(ctl, (u8 *)G12A_DDR_SET_BASE);
#endif
		oret = g12a_open_stage1(ctl);
		g12a_tsm("s1ret");
		g12a_uart_puts("OPEN s1 ret ");
		g12a_uart_hex8((u8)oret);
		g12a_uart_puts("\n");
#if G12A_OPEN_ASSERT_ID
		/* First-pass success: the 0x03 (RANK01_CH01) default trained
		 * clean and no phantom showed -- detection had nothing to do.
		 * A post-demotion success is reported by the caller's cold
		 * loop (RKD 1), not twice more here.
		 */
		if (oret == 0 && !g12a_rank_override)
			g12a_uart_puts("RKD 2\n");
#endif
		if (oret == 0) {
			if (g12a_stage2_finalize(ctl) == 0) {
				g12a_tsm("qed2");
				g12a_uart_puts("QED2\n");	/* sweep marker */
				g12a_train_status_or(G12A_TRAIN_ST_COMPLETE);
			}
		} else {
			g12a_uart_puts("S2 skip\n");
		}
		return oret;
	}
#endif

#if G12A_BL2_ORDER_STEPWISE
	/*
	 * Same work a644 does, once, but observable. This is NOT the old
	 * pre-call pattern — nothing runs twice; we simply walk a644's body
	 * instead of calling it, so the last ">N" without a matching "<N"
	 * names the step that stops. The product reaches 4dc0 and prints
	 * "Check phy result"; we do not, and everything before that is now
	 * byte-matched to the product's output.
	 * 1 topology · 2 timing · 3 prepare · 4 pinmux · 5 MID1 · 6 MID2
	 * 7 MID3 · 8 2280 · 9 4dc0
	 */
	{
		static const ulong step[9] = {
			G12A_AML_FN_TOPOLOGY, G12A_AML_FN_TIMING,
			G12A_AML_FN_PREPARE,  G12A_AML_FN_PINMUX,
			G12A_AML_FN_MID1,     G12A_AML_FN_MID2,
			G12A_AML_FN_MID3,     G12A_AML_FN_LOAD_IMEM,
			G12A_AML_FN_TRAIN,
		};
		int i;

		for (i = 0; i < 9; i++) {
			if (i == 7) {
				/*
				 * Bracket the loader. Source is only known
				 * good from a crumb printed far earlier; the
				 * destination is empty by the time we capture
				 * at step 9. Read BOTH at the copy, so a
				 * clobbered source and a dead write are
				 * distinguishable instead of inferred.
				 * sb = source IMEM/DMEM first halfwords,
				 * db = destination IMEM/DMEM first halfwords.
				 */
#if defined(CONFIG_MESON_G12A_DRAM_OPEN) && G12A_OPEN_COMPARE
				/*
				 * The reference instant: a644's S2..S7 have
				 * just run and nothing has trained, so every
				 * CSR and DMEM byte the open code computes is
				 * comparable against the vendor's own output.
				 * Read-only.
				 */
				g12a_open_compare(ctl);
#endif
#if G12A_L13_RESERVED00
				/*
				 * Row L13.  This is the ONLY instant at which
				 * the blob arm's message block is host-
				 * writable: a644 step 3 (prepare, fffbe244)
				 * has authored it and step[7] (2280) has not
				 * yet copied it into PHY DMEM.  The block is
				 * the head of the 1D firmware's own DMEM
				 * image and prepare overwrites only its first
				 * ~126 bytes in place, so byte 0 here IS the
				 * Reserved00 the micro will read.
				 *
				 * Print both sides of the write.  A poke that
				 * never landed and a mechanism that does not
				 * exist produce the same P2Cp table, so the
				 * pre value (must be 00, the blob's) and the
				 * post value (must be our knob) are the first
				 * of three receipts -- `sb` below reads the
				 * staging copy back at the loader bracket,
				 * and `db` after step[7] reads PHY DMEM, the
				 * byte the PMU actually consumes.
				 */
				{
					volatile u8 *mb =
						(volatile u8 *)G12A_MSG_BLOCK_BASE;
					u8 pre = mb[0];

					mb[0] = (u8)G12A_L13_RESERVED00;
					g12a_uart_puts("L13 mb0 ");
					g12a_uart_hex8(pre);
					g12a_uart_putc('/');
					g12a_uart_hex8(mb[0]);
					g12a_uart_puts("\n");
				}
#endif
				g12a_uart_puts("sb ");
				g12a_uart_hex16(readw((void *)G12A_FW_HOLD_1D));
				g12a_uart_putc('/');
				g12a_uart_hex16(readw((void *)
						G12A_MSG_BLOCK_BASE));
				g12a_uart_putc(' ');
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_IMEM_BASE));
				g12a_uart_putc('/');
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_DMEM_BASE));
				g12a_uart_puts("\n");
			}
			if (i == 8) {
				g12a_uart_puts("db ");
				g12a_uart_hex16(readw((void *)G12A_FW_HOLD_1D));
				g12a_uart_putc('/');
				g12a_uart_hex16(readw((void *)
						G12A_MSG_BLOCK_BASE));
				g12a_uart_putc(' ');
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_IMEM_BASE));
				g12a_uart_putc('/');
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_DMEM_BASE));
				g12a_uart_puts("\n");
			}
			if (i == 7) {
				/*
				 * PUB:3664 — ICCM/DCCM are readable AND
				 * writable over APB ONLY while ARCv2 is
				 * stalled. Our 16-combination sweep varied
				 * gates, reset polarity and width but never
				 * StallToMicro, so "the window is write-only"
				 * was never actually tested in the one state
				 * the spec requires.
				 *
				 * MicroReset (0xD0099 -> 0xFE1A0132) bit0 is
				 * StallToMicro. Read it as the loader finds
				 * it, force the stall, then let step 8 (2280)
				 * copy and read the window back.
				 * Crumb: st <as-found>/<forced>
				 */
				/*
				 * Probe A — the free read, relocated. It was
				 * first written into the monolithic #else
				 * branch, which G12A_BL2_ORDER_STEPWISE=1
				 * compiles out, so it never executed and its
				 * absence was NOT a result.
				 *
				 * DRTUB UctWriteProt (CSR 0xC0033 ->
				 * 0xFE180066) vs the APBONLY status shadow
				 * (0xD0004 -> 0xFE1A0008), read at the same
				 * moment. No write, no stall sequence, no
				 * preconditions. A DIFFERENCE means the micro
				 * posted and the shadow path is broken;
				 * agreement means the mailbox is idle.
				 * Crumb: pa <drtub>/<apbonly>
				 */
				g12a_uart_puts("pa ");
				g12a_uart_hex16(g12a_phy_read16(0xFE180066UL));
				g12a_uart_putc('/');
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_UCTSHADOWREGS));
				g12a_uart_puts("\n");

				g12a_uart_puts("st ");
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_MICRORESET));
				/*
				 * F4/PCSR: the write that used to be here
				 * (MicroReset = 1, CSR 0xD0099) is REMOVED.
				 * The product writes MicroReset for the
				 * first time inside 0xFFFB8D74, AFTER the
				 * capture point, so a host write here makes
				 * the G1 value and the G9 gate ours, not the
				 * PHY's.  Read as found only.
				 */
				g12a_uart_putc('/');
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_MICRORESET));
				g12a_uart_puts("\n");
			}
			if (i == 8) {
				/* stalled: the window should answer now */
				g12a_phy_dmem_dump("SD");
			}
			if (i == 8) {
				/*
				 * T0: ctl+517 is a HOST print level, not
				 * HdtCtrl. Zero it so 4dc0's "Check phy
				 * result" fires — if the string appears we
				 * have PROVEN we reach 0xFFFB8DC4 and the
				 * wall is inside main(); if not, the hang is
				 * in the four writes, a different class.
				 *
				 * And re-arm UcclkHclkEnables. It is written
				 * once in the whole image (PHY init), never
				 * by 4dc0, and we measured it 0003 after 2280
				 * then 0000 at kick time. Forcing it failed
				 * before, but that was the old pre-call path.
				 */
				g12a_uart_puts("uc ");
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_UCCLKHCLKENABLES));
				/*
				 * F4: do NOT force this. Writing
				 * UcclkHclkEnables here perturbs the exact
				 * state the product/open CSR state-diff is
				 * meant to compare, and would mask the
				 * strongest available finding. Read only.
				 */
				g12a_uart_putc('/');
				g12a_uart_hex16(g12a_phy_read16(
						G12A_DDR_PHY_UCCLKHCLKENABLES));
				g12a_uart_puts("\n");
			}
			g12a_uart_putc('>');
			g12a_uart_putc((char)('1' + i));
			g12a_uart_puts("\n");
			if (i == 7) {
				/*
				 * Capture #2 -- the instant that matters for
				 * the SRAM write path.  step[7] IS
				 * G12A_AML_FN_LOAD_IMEM (0xFFFB6234, "2280"),
				 * the vendor IMEM+DMEM loader.  The product's
				 * matching site is 0xFFFBE6B0, the `mov x0,x19`
				 * immediately before `bl 0xFFFB6234` at
				 * 0xFFFBE6B4.  Reads only.
				 */
				g12a_pcsr_capture(ctl, 1);
				/*
				 * Install LAST, so the pcsr walk above sees
				 * the blob exactly as the product's does.
				 * step[7] is about to call 2280, which calls
				 * the copy routine this patches.
				 */
				g12a_ifp_install();
				/*
				 * Last host-visible instant before step[7]
				 * (2280) performs the SRAM fill.  Read only.
				 */
				g12a_mem_pd_report("MPDF");
			}
			if (i == 8) {
				/*
				 * step[7] (2280) has returned; drain the
				 * in-fill records before the train kick.
				 */
				g12a_ifp_report();
				/* THE capture -- last host-visible instant */
				g12a_pcsr_capture(ctl, 0);
				/*
				 * Host print level, moved BELOW the capture so
				 * the snapshot sees ctl as the product leaves
				 * it.  Not a PHY CSR; 0xFFFB8D74 reads it only
				 * at 0xFFFB8DB4, after the micro is released.
				 */
				g12a_poke8(ctl, 517, 0);
			}
			fn = (aml_ctx_fn)step[i];
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
			/* (b) replace the blob's unbounded train poll */
			if (i == 8)
				ret = g12a_probe_train(
					(u8 *)G12A_DDR_SET_BASE);
			else
				ret = fn(ctl);
#else
			ret = fn(ctl);
#endif
			g12a_uart_putc('<');
			g12a_uart_putc((char)('1' + i));
			g12a_uart_putc(' ');
			g12a_uart_hex8((u8)(ret & 0xff));
			g12a_uart_puts("\n");
			g12a_wdt_pet_safe();
		}

		/*
		 * `ret` here is step[8]'s (the 1D train kick's) return.  The
		 * vendor gates stage 2 on exactly this at S20 (fffbea00:
		 * cbnz w20 -> return), so a non-zero return means the micro
		 * did not report success and stage 2 must not run.
		 */
		if (ret == 0) {
			/*
			 * F_DDR_OK is claimed on the DRAM TEST, not on the
			 * training pass.  A micro that reports success and a
			 * controller that returns the bytes you wrote are two
			 * different claims.
			 */
			if (g12a_stage2_finalize(ctl) == 0)
				g12a_train_status_or(G12A_TRAIN_ST_COMPLETE);
		} else {
			g12a_uart_puts("S2 skip\n");
		}
	}
	return -1;
#else
	/*
	 * DRTUB probe — the discriminator. CurPstate0b (CSR 0x0C0081 -> PA
	 * 0xFE180102) and UctWriteOnly (0x0C0032 -> 0xFE180064) are R/W CSRs
	 * in the SAME block as the PHY SRAM, unlike the MASTER regs we have
	 * been reading successfully. PUB documents UcclkHclk as running at
	 * DFICLK or gated, so an open gate cannot manufacture the clock and
	 * APB-domain liveness proves nothing about this domain.
	 *
	 * DRTUB dead while MASTER works  => the domain is unclocked.
	 * DRTUB alive while SRAM reads 0 => the fault is the SRAM path.
	 * Crumb: dt <cur> <uct>
	 */
	g12a_phy_write16(0xFE180102UL, 0x5A5Au);
	g12a_phy_write16(0xFE180064UL, 0xA5A5u);
	g12a_spin(2000);
	g12a_uart_puts("dt ");
	g12a_uart_hex16(g12a_phy_read16(0xFE180102UL));
	g12a_uart_putc('/');
	g12a_uart_hex16(g12a_phy_read16(0xFE180064UL));
	g12a_uart_puts("\n");

	g12a_uart_puts("cb2 ");
	g12a_uart_hex16(g12a_phy_read16(0xFE04012EUL));
	g12a_uart_putc('/');
	g12a_uart_hex16(g12a_phy_read16(G12A_DDR_PHY_MEMRESETL));
	g12a_uart_puts("\n");

	fn = (aml_ctx_fn)G12A_AML_FN_LPDDR4;
	g12a_uart_puts(">ba\n");
	ret = fn(ctl);
	g12a_uart_puts("ba ");
	g12a_uart_hex8((u8)(ret & 0xff));
	g12a_uart_puts("\n");
	return -1;
#endif
}

/*
 * Coarse spin — no timer yet. Order ~µs–ms at 24 MHz-class early clocks.
 * Decompiled aml_ddr FUN_fffd0120 is a delay (ghidra shows infinite-loop stub).
 */
static void g12a_spin(ulong n)
{
	while (n--)
		asm volatile("nop");
}

/**
 * g12a_ddr_pll_init - minimal FUN_fffd504c for Solitude 1608 MT/s
 *
 * RE (g12b amlddr.fw.c): CNTL1..6 defaults when ACS PLL block empty, then
 * CNTL0 = pll_val | 0x1c000000 → delay → | 0x3c000000 → delay → | 0x1c000000,
 * poll bit31. Returns 0 on lock, -1 on fail. Crumb '0' / 'o'.
 */
static int g12a_ddr_pll_init(void)
{
	u32 pll_val = G12A_DDR_PLL_VAL_1608;
	int retries = 100;
	u32 sts;

	/* FUN_fffd00ec: clear EE WDT/timer sticky bits that can cut train */
	if (!G12A_WDT_KEEP())
		writel(readl((void *)G12A_WATCHDOG_CNTL) & G12A_WATCHDOG_CNTL_CLR,
		       (void *)G12A_WATCHDOG_CNTL);

	writel(G12A_DDR_PLL_CNTL6_DEF, (void *)G12A_DDR_PLL_CNTL6);
	writel(G12A_DDR_PLL_CNTL5_DEF, (void *)G12A_DDR_PLL_CNTL5);
	writel(G12A_DDR_PLL_CNTL4_DEF, (void *)G12A_DDR_PLL_CNTL4);
	writel(G12A_DDR_PLL_CNTL3_DEF, (void *)G12A_DDR_PLL_CNTL3);
	writel(G12A_DDR_PLL_CNTL2_DEF, (void *)G12A_DDR_PLL_CNTL2);
	writel(G12A_DDR_PLL_CNTL1_DEF, (void *)G12A_DDR_PLL_CNTL1);
	g12a_spin(1000);

	do {
		writel(pll_val | G12A_DDR_PLL_RST_ASSERT,
		       (void *)G12A_DDR_PLL_CNTL0);
		g12a_spin(10000);
		writel(pll_val | G12A_DDR_PLL_RST_RELEASE,
		       (void *)G12A_DDR_PLL_CNTL0);
		g12a_spin(10000);
		writel(pll_val | G12A_DDR_PLL_RST_ASSERT,
		       (void *)G12A_DDR_PLL_CNTL0);
		g12a_spin(10000);

		sts = readl((void *)G12A_DDR_PLL_CNTL0);
		if (sts & G12A_DDR_PLL_LOCK_BIT) {
			g12a_train_status_or(G12A_TRAIN_ST_PLL_OK);
			g12a_uart_puts("0\n");
			return 0;
		}
	} while (--retries > 0);

	g12a_train_status_or(G12A_TRAIN_ST_PLL_FAIL);
	g12a_uart_puts("o\n");
	return -1;
}

/**
 * g12a_phy_reg_access_enable - BL2 FUN_fffa7e58 after PLL, before IMEM
 *
 * RE (g12b bl2.c): after ddr_init_pll success, before load_ddr_fw + train:
 *   DDR_CLK_CNTL=0xb0000007; SOFT_RST*=0xffffffff; pulse 0; restore;
 *   DDR_CLK_CNTL2=0x1f. Without this, Solitude hung on MEM_ACCESS write
 *   (crumb v, no w) even after PLL lock (tip 23a0017).
 * Crumb n = done.
 */
static void g12a_phy_reg_access_enable(void)
{
	writel(G12A_DDR_CLK_CNTL_VAL, (void *)G12A_DDR_CLK_CNTL);
	writel(0xffffffff, (void *)G12A_DMC_SOFT_RST);
	writel(0xffffffff, (void *)G12A_DMC_SOFT_RST1);
	writel(0xffffffff, (void *)G12A_DMC_SOFT_RST2);
	g12a_spin(10000);
	writel(0, (void *)G12A_DMC_SOFT_RST);
	writel(0, (void *)G12A_DMC_SOFT_RST1);
	writel(0, (void *)G12A_DMC_SOFT_RST2);
	g12a_spin(10000);
	writel(0xffffffff, (void *)G12A_DMC_SOFT_RST);
	writel(0xffffffff, (void *)G12A_DMC_SOFT_RST1);
	writel(0xffffffff, (void *)G12A_DMC_SOFT_RST2);
	writel(G12A_DDR_PHY_CTRL_VAL, (void *)G12A_DDR_PHY_CTRL);
	g12a_spin(5000);
	g12a_uart_puts("n\n");
}

/* WDT pet during long IMEM fill — re-clear EE sticky (FUN_fffd00ec class) */
static void g12a_wdt_pet_safe(void)
{
	if (G12A_WDT_KEEP())
		return;
	writel(readl((void *)G12A_WATCHDOG_CNTL) & G12A_WATCHDOG_CNTL_CLR,
	       (void *)G12A_WATCHDOG_CNTL);
}

/**
 * g12a_stage_1d_imem - place pure IMEM (32 KiB) at 0xFFFC0000
 *
 * FUN_fffd2280 copies 0x8000 B from 0xFFFC0000 → PHY IMEM. Vendor
 * load_ddr_fw/check_fw leaves the *payload* there (header stripped).
 * Residual parks full @DFM 1d.fw (header 0x60 + payload) — strip to
 * IMEM-only so 2280 does not feed the @DFM header into the PMU.
 * Do not span into msg-block @ 0xFFFC8000 (0x8000 only).
 *
 * @src: full blob base (may include @DFM hdr)
 * Return: 1 if non-zero IMEM head written.
 */
static int g12a_stage_1d_imem(ulong src)
{
	ulong dst = G12A_FW_HOLD_1D;
	ulong payload = src;
	u16 hw0;

	if (g12a_fw_magic_is(src, G12A_DDRFW_MAGIC_AML0))
		return 0;
	/* @DFM / dl44 container: IMEM payload after 0x60 hdr */
	if (g12a_fw_magic_is(src, G12A_DDRFW_MAGIC_LP4_1D) ||
	    g12a_fw_magic_is(src, G12A_DDRFW_MAGIC_LP4_2D))
		payload = src + G12A_DDRFW_HDR_SIZE;
	else {
		/* Already stripped? non-zero at base, no dl44 at +0x34 */
		hw0 = readw((void *)src);
		if (hw0 == 0) {
			payload = src + G12A_DDRFW_HDR_SIZE;
			hw0 = readw((void *)payload);
			if (hw0 == 0)
				return 0;
		}
	}

	hw0 = readw((void *)payload);
	if (hw0 == 0)
		return 0;

	/*
	 * Stage the WHOLE 0xC000 payload, not just IMEM. lpddr4_1d.fw is
	 * 49248 = 0xC060 bytes: a 0x60 header then 0x8000 IMEM immediately
	 * followed by 0x4000 DMEM. 2280 copies IMEM from 0xFFFC0000 and DMEM
	 * from 0xFFFC8000 — which is exactly where that DMEM section lands
	 * when the payload is staged contiguously.
	 *
	 * The old "IMEM only, leave 0xFFFC8000 for the msg block" read the
	 * overlap as a collision. It is the design: the message block IS the
	 * head of the firmware's DMEM image, and vendor prepare overwrites
	 * only its first ~126 bytes in place. Staging IMEM alone (and then
	 * zero-filling) handed the micro a good message block on top of zeros
	 * where its own constants belong — per-rank blocks at 0x90, tables
	 * out to 0x67E.
	 */
	g12a_copy32(dst, payload, G12A_FW_HOLD_1D_MAX);
	return 1;
}

/**
 * g12a_stage_1d_vendor - deliver 1d IMEM at 0xFFFC0000 (vendor staging)
 *
 * Residual parks 1d at HOLD0 because USB into HOLD_1D hangs (~8 KiB).
 * After residual promotes aml bounce → DEST, HOLD_1D may still hold AML0
 * residue — CPU-copy stripped IMEM from HOLD0 (or ALT) → 0xFFFC0000.
 * a644 → FUN_fffd2280 then copies FFFC0000 → PHY IMEM.
 *
 * Crumb s = staged (copy); L = already present pure IMEM; l = missing.
 * Return: 1 if non-zero IMEM image at 0xFFFC0000.
 */
static int g12a_stage_1d_vendor(void)
{
	ulong dst = G12A_FW_HOLD_1D;
	ulong src;
	u16 hw0;
	int have = 0;
	int copied = 0;

	/*
	 * Already pure IMEM at staging? non-AML0, no @DFM magic, hw0 != 0.
	 * If still a full 1d container, strip in place via stage_1d_imem.
	 */
	if (!g12a_fw_magic_is(dst, G12A_DDRFW_MAGIC_AML0)) {
		if (g12a_fw_magic_is(dst, G12A_DDRFW_MAGIC_LP4_1D) ||
		    g12a_fw_magic_is(dst, G12A_DDRFW_MAGIC_LP4_2D)) {
			if (g12a_stage_1d_imem(dst)) {
				have = 1;
				copied = 1;
			}
		} else {
			hw0 = readw((void *)dst);
			if (hw0 != 0)
				have = 1;
		}
	}

	if (!have) {
		/* Residual no-2d path: 1d at HOLD0 after aml bounce */
		src = G12A_FW_HOLD_AML_DDR;
		if (g12a_stage_1d_imem(src)) {
			have = 1;
			copied = 1;
			g12a_uart_puts("h0\n");
		}
	}

	if (!have) {
		src = G12A_FW_HOLD_1D_ALT;
		if (g12a_stage_1d_imem(src)) {
			have = 1;
			copied = 1;
			g12a_uart_puts("hA\n");
		}
	}

	if (!have) {
		g12a_uart_puts("l\n");
		return 0;
	}

	/* Confirm IMEM head at FFFC0000 */
	hw0 = readw((void *)dst);
	if (hw0 == 0 || g12a_fw_magic_is(dst, G12A_DDRFW_MAGIC_AML0) ||
	    g12a_fw_magic_is(dst, G12A_DDRFW_MAGIC_LP4_1D)) {
		g12a_uart_puts("l\n");
		return 0;
	}

	g12a_uart_puts(copied ? "s\n" : "L\n");
	g12a_train_status_or(G12A_TRAIN_ST_1D_LOADED | G12A_TRAIN_ST_1D_STAGED);
	return 1;
}

/*
 * Brief spin for host AMLC after 'C'. No timer yet — loop count is a
 * coarse wait (order ~tens of ms at 24 MHz-class early clocks). Host can
 * write dest or hold during this window.
 */
static int g12a_poll_magic(ulong base, u32 magic, ulong spins)
{
	ulong i;

	for (i = 0; i < spins; i++) {
		if (g12a_fw_magic_is(base, magic))
			return 1;
	}
	return 0;
}

unsigned int g12a_ddr_fw_get_train_status(void)
{
	return g12a_train_status_read();
}

int g12a_ddr_fw_train_ok(void)
{
	return (g12a_train_status_read() & G12A_TRAIN_ST_COMPLETE) != 0;
}

#if defined(CONFIG_MESON_G12A_DRAM_OPEN) && G12A_OPEN_LIVE && G12A_OPEN_ASSERT_ID
/*
 * qi:N1294 rank is DETECTED. The 0x03 (RANK01_CH01) default lets the slot5
 * size walk alias rank-0 cells as a phantom rank 1 (cs1 reads 2048 MB, DTS
 * clamps). A pass-A CA-training assert (id 0xa1, nargs 0) is the evidence
 * there is no rank 1: halt the failed micro (vendor StallToMicro, then the
 * msgblock bus back to APB), mark the part single-rank, and let the caller
 * re-run the whole DRAM-init path COLD -- built, PLL'd, soft-reset, loaded
 * and trained as RANK0_CH01, not patched in place. A dual-rank part whose
 * CA training fails for ANOTHER reason is indistinguishable here and would
 * be demoted to single-rank; RKD 1 on the console is the only trace. A
 * live host cfg overlay that names a rank outranks detection: the
 * operator's cfg row said what the part is (RKD cfg, no retry).
 */
static int g12a_rank_detect_cold(int oret)
{
	_Static_assert(G12A_OPEN_ASSERT_ID == 1,
		       "rank detection reads the latched assert id");

	if (oret != 0xff || g12a_open_fail_pass != 'a' ||
	    g12a_open_fail_aid != 0x00a10000u || g12a_rank_override)
		return 0;
#if IS_ENABLED(CONFIG_MESON_G12A_DRAM_PROBE)
	if (readl((void *)G12A_CFG_PROBE_BASE) == G12A_CFG_PROBE_MAGIC &&
	    readl((void *)(G12A_CFG_PROBE_BASE + G12A_CFG_PROBE_RANK_CFG)) !=
	    G12A_CFG_PROBE_NONE) {
		g12a_uart_puts("RKD cfg\n");
		return 0;
	}
#endif
	g12a_uart_puts("RKD 1\n");
	g12a_phy_write16(G12A_DDR_PHY_MICRORESET, 1);	/* StallToMicro */
	g12a_phy_write16(G12A_DDR_PHY_MICROCONTMUXSEL, 0);
	g12a_rank_override = 1;
	return 1;
}
#endif

int g12a_ddr_fw_stub_init(void)
{
	int aml_ok, piei_ok;
	int had_dest;
	int pll_ok = 0;

	/*
	 * Reset sticky before any path. BSS may have been wiped by a prior
	 * scratch clear on a previous boot attempt; always start clean.
	 * The rank override is sticky-in-.data for the same cold-pass reason
	 * it lives there -- an SRAM re-run of the TPL must not start
	 * pre-demoted.
	 */
	g12a_train_status_write(G12A_TRAIN_ST_NONE);
	g12a_rank_override = 0;

	g12a_uart_puts("D\n");

	/* Earliest DDR-path instant: before blob staging, before ctl build. */
	g12a_mem_pd_report("MPDE");

	/*
	 * BL2 powers this memory domain up at the same point in its own DDR
	 * entry -- before the blob is staged.  Do the same, here.
	 */
	g12a_mem_pd_power_up();

	/*
	 * Clear aml_ddr scratch only when dest does not already hold a valid
	 * blob (host may have pre-placed via residual before RUN). Clearing
	 * always would race and wipe a good payload.
	 *
	 * Note: CONFIG_SPL_BSS_START_ADDR (0xfffbc000) sits inside the 48 KiB
	 * window — after clear, BSS globals are invalid. Only stack locals +
	 * .text/.rodata (below 0xFFFB0000) remain safe. train sticky is
	 * re-written below after clear.
	 */
	/*
	 * "Already delivered" is EITHER the rebased payload OR the raw @DFM
	 * file the residual receiver drops straight at the dest. Checking only
	 * the payload form makes this clear() wipe a good residual blob.
	 */
	had_dest = g12a_aml_payload_is_placed() ||
		   g12a_fw_magic_is(G12A_AML_DDR_FW_BASE, G12A_DDRFW_MAGIC_AML0);
	if (!had_dest)
		g12a_clear_aml_ddr_scratch();
	g12a_train_status_write(G12A_TRAIN_ST_NONE);
	g12a_uart_puts("C\n");

	/*
	 * Place the PAYLOAD (file +0x60) at the linker base, not the whole
	 * @DFM file — see g12a-ddr-fw.h. The dest therefore carries no AML0
	 * magic; the export table's own LPDDR4 slot is the integrity check.
	 */
	aml_ok = g12a_aml_payload_is_placed();
	if (!aml_ok) {
		/*
		 * Common case: the residual receiver already wrote the whole
		 * @DFM file to the dest, so rebase IN PLACE (dest < src and
		 * g12a_copy32 walks ascending, so the overlap is safe).
		 */
		if (!g12a_fw_magic_is(G12A_AML_DDR_FW_BASE,
				      G12A_DDRFW_MAGIC_AML0))
			g12a_poll_magic(G12A_AML_DDR_FW_BASE,
					G12A_DDRFW_MAGIC_AML0, 2000000UL);

		if (g12a_fw_magic_is(G12A_AML_DDR_FW_BASE,
				     G12A_DDRFW_MAGIC_AML0)) {
			g12a_copy32(G12A_AML_DDR_FW_BASE,
				    G12A_AML_DDR_FW_BASE + G12A_DDRFW_HDR_SIZE,
				    G12A_AML_FW_PAYLOAD_SIZE);
			aml_ok = g12a_aml_payload_is_placed();
		} else if (g12a_fw_magic_is(G12A_FW_HOLD_AML_DDR,
					    G12A_DDRFW_MAGIC_AML0)) {
			g12a_copy32(G12A_AML_DDR_FW_BASE,
				    G12A_FW_HOLD_AML_DDR + G12A_DDRFW_HDR_SIZE,
				    G12A_AML_FW_PAYLOAD_SIZE);
			aml_ok = g12a_aml_payload_is_placed();
		}
	}
	if (aml_ok)
		g12a_uart_puts("H\n");
	else
		g12a_uart_puts("h\n");

	/*
	 * REVERTED (was c006cd31f3b). piei is NOT the aml_ddr case: the base
	 * legitimately holds the CONTAINER, and the consumer gate at the scan
	 * reads base+0x34, where 'eaea' lives inside the 0x60 header. Staging
	 * the payload here would break that gate, not fix anything — piei has
	 * no absolute self-references, so the 0x60 class of corruption is
	 * structurally impossible for it.
	 *
	 * The replacement also regressed: it sourced from G12A_FW_HOLD_PIEI
	 * (0xFFFE0000), which nothing ever writes, so the presence test failed
	 * every boot and paid a 500k-iteration poll on a dead address. The
	 * residual splitter writes the container straight to the dest, which
	 * is what the original dest-first check detects.
	 */
	piei_ok = g12a_load_blob(G12A_PIEI_FW_BASE, G12A_FW_HOLD_PIEI,
				 G12A_PIEI_FW_BLOB_SIZE,
				 G12A_DDRFW_MAGIC_PIEI);
	if (!piei_ok)
		piei_ok = g12a_poll_magic(G12A_PIEI_FW_BASE,
					  G12A_DDRFW_MAGIC_PIEI, 500000UL);
	if (piei_ok)
		g12a_uart_puts("P\n");
	else
		g12a_uart_puts("p\n");

	/* Payload form, not @DFM header — the header is no longer at dest */
	if (g12a_aml_payload_is_placed()) {
		g12a_uart_puts("M\n");
		g12a_train_status_or(G12A_TRAIN_ST_FW_PRESENT);

		/*
		 * L3.A.7 pre-init: open DDR PLL + FUN_fffa7e58. Optional —
		 * product does PLL via aml_ddr fp_pll before a644; a644 itself
		 * does topology/timing clocks, not open-coded PLL. Mid-pass-A
		 * hang after 0x08: measure with open PLL/soft-rst OFF so a644
		 * path is not pre-stomped (G12A_OPEN_PLL_SOFT_RST).
		 */
		/*
		 * Cold-pass loop: at most 2 iterations, the second ever only
		 * for rank demotion (g12a_rank_detect_cold). The re-pass runs
		 * this SAME path from its top -- PLL, DMC soft reset, ctl
		 * build, fp_pll, S1, stage 1 -- with the ds built as
		 * RANK0_CH01, not a partial re-run against the failed micro.
		 */
		{
			int iter = 0;
			int oret = -1;

			for (;;) {
#if G12A_OPEN_PLL_INIT
				pll_ok = (g12a_ddr_pll_init() == 0);
#else
				pll_ok = 1; /* residual+ctl only; no open 0 */
				g12a_uart_puts("0\n"); /* skipped open PLL */
#endif
#if G12A_OPEN_PHY_REG_ACCESS
				if (pll_ok)
					g12a_phy_reg_access_enable();
#else
				g12a_uart_puts("n\n"); /* skipped open soft-rst */
#endif
				/*
				 * Vendor order: stage the pure 1D IMEM at
				 * 0xFFFC0000, then let the vendor sequence
				 * run it (topology .. 2280 .. 4dc0).
				 * Crumbs: (0 n optional) cb bp >1..>9 S2
				 */
#if G12A_OPEN_REPLAY
				/*
				 * MODE A NEVER READS 0xFFFC0000.  a644's
				 * fffbe70c b.eq jumps past fffb6294 (the only
				 * consumer of the staged IMEM/DMEM), and the
				 * open replay arm skips g12a_open_s9_load()
				 * outright, so staging is dead work here --
				 * and worse than dead.
				 *
				 * MEASURED, boot replay-1 (2026-08-30): with
				 * the lpddr4_1d and lpddr4_2d windows
				 * zero-filled, g12a_stage_1d_vendor() finds
				 * nothing at 0xFFFC0000 (hw0 == 0) and nothing
				 * at HOLD0, then falls through to
				 * G12A_FW_HOLD_1D_ALT = 0xFFFDC000 -- PAST the
				 * window the concat delivers -- and copied
				 * 0x9000 bytes over the SC window at
				 * 0xFFFE0000.  Not staging is the fix;
				 * bounding the fallback would be a workaround
				 * for a step that must not run in this mode.
				 */
				g12a_uart_puts("RPno1d\n");
#else
				if (!g12a_stage_1d_vendor())
					g12a_train_status_or(G12A_TRAIN_ST_1D_MISSING);
#endif

				oret = g12a_try_aml_ddr_entry();
#if defined(CONFIG_MESON_G12A_DRAM_OPEN) && G12A_OPEN_LIVE && G12A_OPEN_ASSERT_ID
				if (iter == 0 && g12a_rank_detect_cold(oret)) {
					iter++;
					continue;
				}
#endif
				break;
			}
			(void)iter;
			(void)oret;
		}

		(void)pll_ok;
		/*
		 * Return 0 for TPL continuity (banner / HO / residual).
		 * F_DDR_OK is gated on g12a_ddr_fw_train_ok() only.
		 */
		return 0;
	}

	g12a_uart_puts("m\n");
	g12a_train_status_or(G12A_TRAIN_ST_NO_FW);
	g12a_uart_puts("N\n"); /* no train — handoff must leave F_DDR_OK clear */
	return 0;
}

int dram_init(void)
{
	return g12a_ddr_fw_stub_init();
}
