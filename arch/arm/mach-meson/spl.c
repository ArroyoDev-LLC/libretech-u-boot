// SPDX-License-Identifier: GPL-2.0+
/*
 * Portions Copyright (C) 2015, Amlogic, Inc. All rights reserved.
 * Copyright (C) 2023, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#include <spl.h>
#include <bootsource.h>
#include <chip_serial.h>
#include <hang.h>
#include <linux/string.h>
#include <asm/io.h>
#include <asm/sections.h>
#include <asm/spl.h>
#include <asm/arch/boot.h>
#include <asm/arch/wdt-reset.h>
#include <timestamp.h>
#include <version.h>
#include <vsprintf.h>
#include <asm/ptrace.h>
#include <asm/system.h>
#include <atf_common.h>
#include <image.h>
#include <linux/delay.h>
#if defined(CONFIG_MESON_G12A)
#include <asm/arch/g12a.h>
#include <asm/arch/clock-g12a.h>
#include <asm/arch/g12a-ddr-fw.h>
#include <asm/arch/g12a-handoff.h>
/* Strategy A: dram_init() in dram-g12a.c clears 0xFFFB4000, load stubs. */
#else
#include <asm/arch/gx.h>
#include <asm/arch/clock-gx.h>
#include <asm/arch/gx-handoff.h>
#endif
#include <asm/arch/gx-bootlog.h>

#if defined(CONFIG_MESON_GXL)
/*
 * meson-usb.o and meson-u1-asm.o are Makefile-gated on
 * CONFIG_$(PHASE_)MESON_USB_BOOT, so their declarations are gated the same
 * way.  CONFIG_MESON_GXL alone is TRUE for the SD/eMMC SPL fragments
 * (board/amlogic/p212/spl-libretech-cc-{1,2}gb.config), which link neither
 * object -- and a declaration with no definition turns a missing guard into
 * a LINK error naming an address rather than a compile error naming the
 * function.  That is exactly how the two call sites below went unnoticed.
 */
#if CONFIG_IS_ENABLED(MESON_USB_BOOT)
void meson_usb_receive_fit(void);
int meson_usb_reenum_serve(unsigned int timeout_ms);
int meson_usb_reenum_phy_por(unsigned int timeout_ms);
void meson_usb_poll_once(void);
void meson_bl1_usb_return(void);
void meson_u1_mark_and_return(void);
void meson_u1_try_identify_or_return(unsigned int timeout_ms);
void meson_u1_asm_identify(void);
void meson_usb_acm_serve(unsigned int id_timeout_ms);
void meson_usb_soft_disconnect(void);
void meson_usb_last_payload(u32 *addr, u32 *len, u32 *run);
#endif
int meson_cpu_try_sys_pll(void);
int meson_cpu_sys_pll_force_1g2(void);
void meson_fixed_pll_enable_div3(void);
void meson_spl_selfcheck(void);
void meson_gx_dos_publish_caps(void);
/*
 * Stage breadcrumb @ 0xd900c120 (not c108 — that is DSTS debug dump).
 * Magic high half 0x5354 'ST' so host rejects garbage.
 */
#define SPL_U1_STAGE_ADDR	0xd900c120UL
#define SPL_U1_STAGE_MAGIC	0x53540000u
static void prod_stage(u32 s)
{
	writel(SPL_U1_STAGE_MAGIC | (s & 0xffff),
	       (void __iomem *)SPL_U1_STAGE_ADDR);
}
/*
 * GXL EE watchdog (meson_gxbb_wdt): CTRL@0xc11098d0, TCNT@+8, RSET@+c.
 * TCNT is 16-bit milliseconds after DIV setup (1 ms tick). Old code used
 * wrong TCNT/RSET offsets (0xc1109894/98) so WDT never armed.
 */
#define GXL_WDT_CTRL	0xc11098d0UL
#define GXL_WDT_TCNT	0xc11098d8UL
#define GXL_WDT_RSET	0xc11098dcUL
#define GXL_WDT_EN	BIT(18)
#define GXL_WDT_EE_RST	BIT(21)
#define GXL_WDT_CLK_EN	BIT(24)
#define GXL_WDT_CLKDIV	BIT(25)
/* Assume 24 MHz xtal → div=24000 for 1 ms tick */
#define GXL_WDT_DIV_1MS	24000u

static void wdt_start_ms(u32 ms)
{
#if !defined(CONFIG_MESON_G12A)
	if (ms > 0xffff)
		ms = 0xffff;
	writel(GXL_WDT_DIV_1MS | GXL_WDT_EE_RST | GXL_WDT_CLK_EN |
	       GXL_WDT_CLKDIV, (void *)GXL_WDT_CTRL);
	writel(ms, (void *)GXL_WDT_TCNT);
	writel(0, (void *)GXL_WDT_RSET);
	writel(readl((void *)GXL_WDT_CTRL) | GXL_WDT_EN, (void *)GXL_WDT_CTRL);
#endif
}

static void wdt_pet(void)
{
#if !defined(CONFIG_MESON_G12A)
	writel(0, (void *)GXL_WDT_RSET);
#endif
}

static void wdt_stop(void)
{
#if !defined(CONFIG_MESON_G12A)
	writel(readl((void *)GXL_WDT_CTRL) & ~GXL_WDT_EN, (void *)GXL_WDT_CTRL);
#endif
}

/*
 * meson_gx_wdt_arm_ms() - arm the GXL EE watchdog from a stage that does not
 * own these helpers (the DRAM SPL's failure stop, spl-dram-stage-gx.c).
 */
void meson_gx_wdt_arm_ms(u32 ms)
{
	wdt_start_ms(ms);
}

/*
 * dram_init()'s clock ladder pets between attempts: board_init_f() gives it
 * one 8 s window, and up to six trainings (a failed one waits 0.5 s on PGSR0)
 * plus their memtests must not be cut off by it.  Overrides the weak no-op in
 * dram-gx.c.
 */
void meson_spl_wdt_pet(void)
{
	wdt_pet();
}
/*
 * A terminal stop.  The watchdog is DELIBERATELY LEFT AS IT IS -- read the
 * contradiction below before changing that, because both sides of it are
 * measured and they want opposite things.
 *
 * The DRAM-init failure stop (prod_hang(0xDE), "DF") is a third ARMED site
 * since the GXL DDR3 FMA round F3: its BootROM hand-back was measured mute
 * on both GXL boards, so it now relies on this reset, like NOSPL.
 *
 * 🔴 TWO COMMITTED AUDITS DISAGREE ABOUT WHETHER prod_hang() MAY DISARM.
 *
 * FOR disarming -- amlogic/software/boot/audit/2026-09-07-open-boot-product-
 * source-reading.md finding 4, and gx_tpl_jump_to_spl()'s own docstring below:
 * "The watchdog is STOPPED. [...] leaving it armed means a reset lands
 * somewhere inside the next stage at a time nobody chose -- destroying the SRAM
 * evidence that is the whole point of the handoff block."  Two of the seven
 * prod_hang() call sites in this file are reached with the watchdog ARMED, both
 * on the GXL TPL path where board_init_f() arms wdt_start_ms(120000) -- clamped
 * to 65,535 ms by wdt_start_ms()'s own `if (ms > 0xffff)`:
 *
 *   :599  prod_hang(0x7e)  NOSPL: nothing loaded at SPL_ENTRY   ARMED
 *   :1108 prod_hang(0x79)  DM init failed                       ARMED
 *
 * gx_tpl_jump_to_spl()'s wdt_stop() sits AFTER the check that can take the :599
 * branch, so the success path is guarded and the failure path is not.  About
 * 65.5 s later the EE watchdog resets the board and erases the report.
 *
 * AGAINST -- the DRAM bring-up rig DEPENDS on that reset, measured a day
 * earlier and used as an instrument:
 *   gxl/dram/audit/2026-09-06-n480-bit26-read-back-and-the-jmp-wedge.md:158
 *     "the one unrecoverable state now takes NOSPL, which is upstream of
 *      wdt_stop() -- the watchdog is still armed, prod_hang() does not disarm
 *      it, and BL1 is re-entered on its own, which is the resume substrate n474
 *      measured."  The launcher waits up to 180 s for exactly that.
 *   gxl/dram/audit/2026-09-06-n485-jmp-accept-and-the-unproducible-stop0.md:217
 *     uses the endpoint's fate as a SECOND, INDEPENDENT WITNESS of which branch
 *     was taken: "NOSPL reaches prod_hang() with the watchdog still ARMED, so
 *     BL1 is re-entered on its own and 3-5.2 comes back [...] JMP runs
 *     wdt_stop() first, so nothing brings the board back."
 *
 * Disarming here therefore does not merely change a failure path: it removes
 * the USB rig's only unattended recovery AND retires a two-sided discriminator
 * that a whole round of DRAM results rests on.  A disarm landed in
 * 0f174b0a68f and is reverted by this commit for that reason.
 *
 * 🟡 THIS IS AN OPEN OWNER DECISION, NOT A CLOSED ONE.  The two readings are
 * not reconcilable by reading more code -- they weigh a boot-time SRAM report
 * against an unattended-rig recovery, on the same line, and which matters
 * depends on which build is running.  The options, none of them taken here:
 *
 *   (a) leave as-is (this commit).  The rig keeps its recovery and its witness;
 *       a failing product boot still loses its SRAM report 65.5 s later.
 *   (b) disarm at the :1108 DM-init site only.  n480/n485 name the :599 NOSPL
 *       path exclusively, so this may cost the rig nothing -- but nothing in
 *       either audit says the rig never reaches :1108, so that is an untested
 *       assumption, not a finding.
 *   (c) make it explicit per build -- a symbol the DRAM-eye targets set to keep
 *       the reset and the product targets clear.  Costs a new Kconfig, which
 *       the Amlogic boot-chain policy constrains (§3b: one defconfig per
 *       target, no fragments).
 *   (d) give the rig a different recovery (host-side power cycle) and then
 *       disarm unconditionally.  Largest change, removes the conflict.
 *
 * ⚪ Note for whoever takes it: the G12A pair (:729, :755) is not in scope
 * either way.  wdt_start_ms(), wdt_pet() and wdt_stop() are ALL
 * `#if !defined(CONFIG_MESON_G12A)`, so on G12A/SM1 this file arms no watchdog
 * at all -- a textual walk reports a third armed hang there and is wrong.
 */
static void prod_hang(u32 s)
{
	prod_stage(s);
	for (;;)
		;
}
#else
/* Non-GXL (G12A/SM1 open first stage): no GXL WDT / stage breadcrumb yet */
static void prod_stage(u32 s) { (void)s; }
static void wdt_start_ms(u32 ms) { (void)ms; }
static void wdt_pet(void) { }
static void wdt_stop(void) { }
/*
 * A terminal stop on G12A/SM1.  This first stage arms no watchdog of its own
 * and the USB residual stops the BootROM's, so a bare spin here left the board
 * wedged until its power was cut -- with the DWC2 still soft-connected, which
 * a host sees as a dead 1b8e:c003 (amlogic/software/boot/audit/
 * 2026-10-12-tpl-spl-usb-gadget-review.md).  Drop the link, say so, and let
 * the EE watchdog reset the board: the BootROM runs again and, with no
 * bootable medium or the USB-boot key, offers USB download.  The GXL
 * prod_hang() above stays a spin -- its reset is an open owner decision.
 */
#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
void meson_g12_usb_device_stop(void);
#endif
static void early_puts(const char *s);

/*
 * Measurement only: G12A_STAGE_TS=1 (KCFLAGS) prints "TSM <tag> <cntpct>"
 * around the stage-2 request, matching dram-g12a.c's stage marks.
 */
#ifndef G12A_STAGE_TS
#define G12A_STAGE_TS		0
#endif
#if G12A_STAGE_TS
static void spl_tsm(const char *tag)
{
	static const char hx[] = "0123456789abcdef";
	char buf[17];
	u64 v;
	int i;

	asm volatile("isb; mrs %0, cntpct_el0" : "=r" (v));
	for (i = 15; i >= 0; i--, v >>= 4)
		buf[i] = hx[v & 0xf];
	buf[16] = '\0';
	early_puts("TSM ");
	early_puts(tag);
	early_puts(" ");
	early_puts(buf);
	early_puts("\n");
}
#else
#define spl_tsm(tag)	do { } while (0)
#endif

static void prod_hang(u32 s)
{
#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
	meson_g12_usb_device_stop();
#endif
	early_puts("HANG: reset by watchdog\n");
	(void)s;
	meson_wdt_reset_after(MESON_HANG_RESET_MS);
}
#endif
#include <linux/delay.h>
#include <debug_uart.h>
#include <asm/global_data.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Give udelay() a tick rate before anything can call it.
 *
 * udelay() -> usec_to_tick() -> get_tbclk(), and get_tbclk() reads CNTFRQ_EL0.
 * That register reads 0 on this stage: CONFIG_COUNTER_FREQUENCY is 0, and the
 * msr in arch/arm/cpu/armv8/start.S is behind branch_if_not_highest_el while
 * we run at EL1, where CNTFRQ_EL0 is architecturally read-only -- so it cannot
 * be fixed from here either.  usec * 0 / 1000000 is 0, the target becomes
 * start + 1 tick, and the first comparison in __udelay()'s loop already
 * passes.  Every udelay() on this path therefore takes the same ~4.09 us
 * whatever it was asked for.
 *
 * CONFIG_ARMV8_CNTFRQ_BROKEN makes get_tbclk() prefer gd->arch.timer_rate_hz
 * when it is non-zero, so publishing the rate here is the whole fix.  gd is
 * live: crt0_64.S sets x18 and calls board_init_f_init_reserve() before it
 * calls this function.
 *
 * First statement of board_init_f() deliberately.  meson_power_init() and
 * dram_init() both delay, and there is no earlier C on this path.  The GXL
 * DRAM stage has its own board_init_f() (spl-dram-stage-gx.c), which calls
 * this too: without it every get_timer() deadline in that stage stands still,
 * the SPL flash session's DFU inactivity timeout among them.
 */
#if defined(CONFIG_MESON_TIMER_RATE_FIXUP)
/*
 * The rate is calibrated, not assumed.  CONFIG_MESON_TIMER_RATE_HZ's
 * 333.33 MHz was measured by the dram-bench stage on AML-S905X-CC, but the
 * DRAM SPL counts 600 MHz on both AML-S805X-AC-V2 (599999738 ticks per
 * 1000000 ISA_TIMERE us) and AML-S905X-CC-V2 (599999921), so the constant
 * made SPL deadlines 1.8x short (the AC-V2's 60 s flash-session timeout
 * lasted 33 s).  The
 * reference is ISA_TIMERE, the BootROM's 1 MHz counter (P_ISA_TIMERE, BSP
 * v2015 arch/arm/include/asm/arch-gxl/regs.h:164), which a host read at
 * 999995.5 ticks per second over the USB stage.  CONFIG_MESON_TIMER_RATE_HZ
 * is the fallback when ISA_TIMERE does not move.
 */
#define GX_ISA_TIMERE		0xc1109988UL
#define GX_TIMER_CAL_US		10000
/* Give up on ISA_TIMERE after this many CNTPCT ticks (~1.8 s at 600 MHz) */
#define GX_TIMER_CAL_GIVEUP	0x40000000ULL

static u64 gx_cntpct(void)
{
	u64 c;

	asm volatile("isb; mrs %0, cntpct_el0" : "=r" (c));
	return c;
}

void meson_timer_rate_init(void)
{
	u32 e0, e;
	u64 c0, c;

	e0 = readl((void *)GX_ISA_TIMERE);
	c0 = gx_cntpct();
	do {
		e = readl((void *)GX_ISA_TIMERE);
		c = gx_cntpct();
		if (c - c0 > GX_TIMER_CAL_GIVEUP) {
			gd->arch.timer_rate_hz = CONFIG_MESON_TIMER_RATE_HZ;
			return;
		}
	} while (e - e0 < GX_TIMER_CAL_US);
	gd->arch.timer_rate_hz = (c - c0) * 1000000ULL / (e - e0);
}
#else
void meson_timer_rate_init(void)
{
}
#endif

/*
 * The u-boot SPL boot device enum (asm/spl.h) assigns BOOT_DEVICE_SPI = 8.
 * Meson's boot.h #defines BOOT_DEVICE_SPI = 3 (the ROM's boot device ID),
 * which shadows the enum value.  Use this constant for the SPL return value.
 */
#define UBOOT_SPL_BOOT_DEVICE_SPI	8

#if defined(CONFIG_MESON_G12A_RESIDENT) && defined(MESON_G12A_DRAM_STAGE)
bool g12a_resident_fit_in_ram(void);
#endif

u32 spl_boot_device(void)
{
	int boot_device = meson_get_boot_device();

#if defined(CONFIG_MESON_G12A_RESIDENT) && defined(MESON_G12A_DRAM_STAGE)
	/*
	 * A TPL the BootROM loaded from eMMC has already read the boot FIT to
	 * DRAM; MMC2 would make this stage read raw sector
	 * SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR (the GXL value) instead.
	 */
	if (g12a_resident_fit_in_ram())
		return BOOT_DEVICE_RAM;
#endif

	switch (boot_device) {
	case BOOT_DEVICE_EMMC:
		return BOOT_DEVICE_MMC2;
	case BOOT_DEVICE_SD:
		return BOOT_DEVICE_MMC1;
	case BOOT_DEVICE_SPI:
		return UBOOT_SPL_BOOT_DEVICE_SPI;
	/*
	 * USB boot uses the RAM boot device.  The host tool writes FIT
	 * to DRAM via ROM protocol, then SPL loads it from RAM.
	 */
	case BOOT_DEVICE_USB:
		if (CONFIG_IS_ENABLED(YMODEM_SUPPORT))
			return BOOT_DEVICE_UART;
		else
			return BOOT_DEVICE_RAM;
	}

	panic("Unknown device %d\n", boot_device);
	return BOOT_DEVICE_NONE; /* Never reached */
}

/*
 * The fixed FIT staging buffer.  Spelled CONFIG_TEXT_BASE + 0x4000000 because
 * every meson defconfig had CONFIG_TEXT_BASE 0x01000000, which made it
 * 0x05000000 -- but the derivation was a coincidence, not a relationship.
 *
 * CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT moves BL33 to 0x04000000, and the old
 * expression would have silently dragged this buffer to 0x08000000, which is
 * CONFIG_MESON_GX_SPL_ENTRY -- the DRAM stage would stage a FIT on top of
 * itself.  So pin it: 0x05000000 is now FREE on the libre layout precisely
 * because BL31 vacated 0x05100000 for 0x00100000.
 */
#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT)
#define MESON_SPL_FIT_BUFFER_ADDR	0x05000000UL
#else
#define MESON_SPL_FIT_BUFFER_ADDR	(CONFIG_TEXT_BASE + 0x4000000UL)
#endif

__weak struct legacy_img_hdr *spl_get_load_buffer(ssize_t offset, size_t size)
{
	return (void *)MESON_SPL_FIT_BUFFER_ADDR;
}

/* Bare-metal AO-UART writers, defined below with the rest of the early debug. */
static void early_puts(const char *s);
static void early_puthex(ulong v);

/*
 * The distance from the fixed FIT load buffer to the next FIT load
 * destination above it.
 *
 * spl_get_load_buffer() above is not an allocator: it returns the CONSTANT
 * MESON_SPL_FIT_BUFFER_ADDR, 0x05000000 on every meson defconfig in this tree.
 * The FIT's own 'atf' image loads at 0x05100000
 * (arch/arm/dts/meson-gxl-u-boot.dtsi and meson-g12-common-u-boot.dtsi),
 * 1 MiB above.  A FIT larger than that gap therefore CONTAINS BL31's own
 * destination, and the atf copy shreds the FIT mid-parse.
 *
 * Under CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT the atf image loads at 0x00100000
 * instead, so the nearest load destination ABOVE the buffer becomes
 * CONFIG_MESON_GX_SPL_ENTRY 0x08000000, 48 MiB up.  The bound below is
 * therefore conservative rather than binding there; it is deliberately NOT
 * raised, because a bigger FIT has not been measured on that path.
 *
 * Measured on AML-S905X-CC-V2 (2026-08-12): the external-data FIT copies
 * 2048 bytes here and boots; the embedded-data form copies 1206008 and dies
 * with "cannot find image node 'fdt-1': -11" -- a corruption three stages
 * away from its cause.
 *
 * This constant is the distance to that destination, not a free-standing
 * number: if the buffer address moves, this moves with it.
 */
#define MESON_SPL_FIT_BUFFER_SIZE	0x00100000UL

__weak void *board_spl_fit_buffer_addr(ulong fit_size, int sectors, int bl_len)
{
	/* HACK: use same fit load buffer address as for mmc raw */
	void *buf = spl_get_load_buffer(0, fit_size);

	/*
	 * Honour fit_size -- it was accepted and discarded, so nothing
	 * anywhere checked that the FIT fits.  spl_simple_fit_read() maps a
	 * NULL return to -EIO, so an over-large FIT becomes a NAMED refusal
	 * here, at the point of overflow, instead of a silent overwrite
	 * diagnosed stages later.
	 */
	if (fit_size > MESON_SPL_FIT_BUFFER_SIZE) {
		early_puts("SPL: FIT 0x");
		early_puthex(fit_size);
		early_puts(" B exceeds the 0x");
		early_puthex(MESON_SPL_FIT_BUFFER_SIZE);
		early_puts(" B fixed FIT buffer at 0x");
		early_puthex((ulong)(uintptr_t)buf);
		early_puts(" - refusing\n");
		return NULL;
	}

	return buf;
}

__weak bool spl_load_simple_fit_skip_processing(void)
{
	return false;
}

/* To be defined in dram-${GENERATION}.c */
__weak int dram_init(void)
{
	return 0;
}

/* Placeholder functions to be defined in SoC-specific spl-... file */
__weak void meson_power_init(void)
{
}

__weak int meson_pll_init(void)
{
	return 0;
}

/*
 * Bare-metal UART putc for very early debug.
 * Writes directly to AO UART WFIFO, no init needed (BROM leaves it up).
 * GXL AO UART TX = 0xc81004c0; G12/SM1 AO UART = 0xff803000.
 */
#if defined(CONFIG_MESON_G12A)
#define EARLY_UART_WFIFO	0xff803000UL
#define EARLY_UART_STATUS	0xff80300cUL
#else
#define EARLY_UART_WFIFO	0xc81004c0UL
#define EARLY_UART_STATUS	0xc81004ccUL
#endif
static void __used early_putc(char c)
{
	/* Bit 21 = TX full on both generations. */
	int timeout = 100000;

	while ((readl((void *)EARLY_UART_STATUS) & (1 << 21)) && --timeout)
		;
	if (timeout)
		writel(c, (void *)EARLY_UART_WFIFO);
	gx_bootlog_putc(c);
}
static void __used early_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			early_putc('\r');
		early_putc(*s++);
	}
}
/*
 * A bring-up diagnostic: on GX only with MESON_GX_XPL_VERBOSE (board+bootloader
 * policy "Boot console output"); G12 keeps printing it.
 */
static void __used early_vputs(const char *s)
{
	if (!IS_ENABLED(CONFIG_MESON_GX) || IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
		early_puts(s);
}
/* Fixed-width 8 nibbles: a length is only useful next to another length. */
static void __used early_puthex(ulong v)
{
	int i;

	for (i = 28; i >= 0; i -= 4)
		early_putc("0123456789abcdef"[(v >> i) & 0xf]);
}

#if defined(CONFIG_TPL_BUILD) && defined(CONFIG_MESON_GXL)
extern u32 meson_dram_trained_mb;
extern u32 meson_dram_clk;

/**
 * gx_tpl_write_handoff() - publish what this stage learned, for the next one.
 * @ddr_ok:	dram_init() succeeded
 *
 * Written after DDR and after the receive session has been torn down, which is
 * the first moment every field is final.  @magic goes last and behind a barrier:
 * a reader that finds it has, by construction, the fifteen words before it too,
 * so a stage that dies mid-fill leaves an invalid block rather than a plausible
 * one.  The block is SRAM, not DRAM -- it survives the jump whatever the next
 * stage does to memory.
 *
 * The progress word is read back masked to 16 bits on purpose: ep0_tx() parks a
 * GNPTXSTS snapshot in bits 31:16, so the high half is not a breadcrumb.
 */
static void gx_tpl_write_handoff(int ddr_ok)
{
	struct gx_xpl_handoff *h = (struct gx_xpl_handoff *)GX_HANDOFF_ADDR;
	u32 payload_addr = 0, payload_len = 0, run_addr = 0;
	u32 flags = 0, ddr_flags = 0;
	u32 tune;

	if (ddr_ok)
		flags |= GX_HANDOFF_F_DDR_OK;

#if CONFIG_IS_ENABLED(MESON_USB_BOOT)
	meson_usb_last_payload(&payload_addr, &payload_len, &run_addr);
	if (payload_len)
		flags |= GX_HANDOFF_F_PAYLOAD | GX_HANDOFF_F_USB;
#endif

	if (IS_ENABLED(CONFIG_DRAM_2T_MODE))
		ddr_flags |= GX_HANDOFF_DDR_2T;
	if (IS_ENABLED(CONFIG_DRAM_DDR4))
		ddr_flags |= GX_HANDOFF_DDR_DDR4;
	if (IS_ENABLED(CONFIG_DRAM_DQS_CORR))
		ddr_flags |= GX_HANDOFF_DDR_DQS_CORR;
	if (IS_ENABLED(CONFIG_DRAM_ONE_RANK))
		ddr_flags |= GX_HANDOFF_DDR_1RANK;
	if (IS_ENABLED(CONFIG_DRAM_TWO_IDENTICAL_RANKS))
		ddr_flags |= GX_HANDOFF_DDR_2RANK_ID;
	if (IS_ENABLED(CONFIG_DRAM_TWO_DIFF_RANKS))
		ddr_flags |= GX_HANDOFF_DDR_2RANK_DF;
	if (IS_ENABLED(CONFIG_DRAM_16BIT_RANK))
		ddr_flags |= GX_HANDOFF_DDR_16BIT;

	writel(0, &h->magic);

	writel(GX_HANDOFF_VERSION, &h->version);
	writel(flags, &h->flags);
	writel(meson_dram_trained_mb, &h->dram_size_mb);
	writel(meson_dram_clk, &h->dram_clk_mhz);
	writel(ddr_flags, &h->ddr_flags);
	writel((u32)meson_get_boot_device(), &h->boot_device);
	writel(payload_addr, &h->payload_addr);
	writel(payload_len, &h->payload_len);
	writel(run_addr, &h->run_addr);
	/* No DRAM SPL yet (T2); the pair stays 0 rather than guessing one. */
	writel(0, &h->spl_addr);
	writel(0, &h->spl_len);
	writel(readl((void *)GX_DDR_STICKY_STEP_ADDR), &h->train_status);
	writel(readl((void *)GX_DDR_STICKY_PGSR_ADDR), &h->pgsr0);
	/*
	 * Forward the knob block by address: it is SRAM the next stage can read
	 * directly, and copying an as-yet-unwritten 128 B into a 64 B block
	 * would cost image space to move nothing.  A word of zero means the
	 * host wrote no params before RUN.
	 */
	tune = readl((void *)GX_TUNE_PARAMS_ADDR);
	writel(tune ? (u32)GX_TUNE_PARAMS_ADDR : 0, &h->tune_params);
	writel(readl((void *)GX_XPL_PROG_ADDR) & 0xffff, &h->progress);

	asm volatile("dsb sy" ::: "memory");
	writel(GX_HANDOFF_MAGIC, &h->magic);

	/*
	 * OR into the low half, where GX_XPL_PROG_* are allocated.  The high
	 * half is not "not ours" -- by this point the DWC2 session is over and
	 * nothing writes it again -- but it still holds ep0_tx()'s last GNPTXSTS
	 * snapshot, so a flag there would be unreadable.  See gx-handoff.h.
	 */
	writel(readl((void *)GX_XPL_PROG_ADDR) | GX_XPL_PROG_HANDOFF,
	       (void *)GX_XPL_PROG_ADDR);
}

#if defined(CONFIG_MESON_GX_XPL_CHAIN)
int gx_tpl_load_spl_mmc(int devnum, void *dst, u32 len, u32 *err);
int gx_tpl_load_spl_nor(void *dst, u32 len, u32 *err);
u32 meson_usb_spl_payload_len(void);
void gx_tpl_nor_probe(void);
void gx_tpl_mmc_probe(void);

/**
 * gx_tpl_dm_init() - bring up driver model exactly once
 *
 * Two things in the TPL want DM -- the NOR probe (a diagnostic, early) and the
 * MMC loader (a boot source, late) -- and spl_early_init() is not safe to call
 * twice: it re-runs spl_common_init(), which re-initialises malloc and
 * re-scans the device tree.  GD_FLG_SPL_EARLY_INIT is the flag spl_init()
 * itself tests for the same reason.
 */
static int gx_tpl_dm_init(void)
{
	if (gd->flags & GD_FLG_SPL_EARLY_INIT)
		return 0;

	return spl_early_init();
}

/**
 * gx_tpl_fetch_spl() - get the DRAM stage from the medium this board booted.
 * @err:	set to a GX_TPL_LOAD_* code when the return is 0
 *
 * Return: bytes placed at CONFIG_MESON_GX_SPL_ENTRY, or 0 on failure.
 *
 * One dispatch, one layout.  binman puts SPL at MESON_GX_SPL_OFFSET inside
 * u-boot-meson-with-spl.bin on every medium, so what differs between boot
 * sources is only HOW the bytes are moved, never where they are:
 *
 *  - USB is a PUSH.  The host wrote the image into DRAM during the session
 *    TPL just finished; there is nothing to fetch, only a length to recover.
 *    meson-usb.c latches the length of any completed transfer addressed to
 *    SPL_ENTRY, so this reports what actually arrived rather than assuming.
 *
 *  - SD/eMMC is a PULL, and is the product path.  tpl-load-mmc.c.
 *
 *  - SPI NOR is a PULL too, tpl-load-nor.c, and the DRAM stage then reads
 *    the FIT from the same part (spl-load-nor.c): a board that booted from
 *    NOR reads every stage from NOR.  A build without MESON_GX_LOAD_NOR says
 *    so with its own code instead of falling through to a generic "no".
 *
 * A stage that guesses here is worse than one that stops: every failure below
 * hands back a code and the boot device id, and the caller refuses.
 */
static u32 gx_tpl_fetch_spl(u32 *err)
{
	void *dst = (void *)(uintptr_t)CONFIG_MESON_GX_SPL_ENTRY;
	int bd = meson_get_boot_device();

	switch (bd) {
	case BOOT_DEVICE_USB:
#if CONFIG_IS_ENABLED(MESON_USB_BOOT)
		{
			u32 got = meson_usb_spl_payload_len();

			if (got)
				return got;
		}
#endif
		/*
		 * Not an error yet, and not treated as one: the entry-word
		 * check below is the real test.  A host that wrote SPL in a
		 * way this stage could not attribute -- a single transfer
		 * spanning SPL and the FIT, say -- still leaves runnable code
		 * at SPL_ENTRY, and refusing it would break the working T2
		 * bring-up path for a bookkeeping reason.  What is lost is the
		 * length, and spl_len says 0 rather than inventing one.
		 */
		*err = GX_TPL_LOAD_USB_NONE;
		return 0;

	case BOOT_DEVICE_SD:
	case BOOT_DEVICE_EMMC:
		if (!IS_ENABLED(CONFIG_TPL_MESON_GX_LOAD_MMC)) {
			*err = GX_TPL_LOAD_NO_MMC;
			return 0;
		}
#if IS_ENABLED(CONFIG_TPL_MESON_GX_LOAD_MMC)
		/*
		 * The devnum is a BOARD fact and comes from Kconfig, not from
		 * a literal here.  This used to read `? 1 : 0`, which is
		 * inverted on both GXL boards: meson-gx.dtsi calls
		 * `mmc0 = &sd_emmc_c` the SD card, but AML-S905X-CC and
		 * CC-V2 both put SD on sd_emmc_b and eMMC on sd_emmc_c, so
		 * mmc0 is eMMC and mmc1 is SD.  Unobservable until the
		 * bootph-pre-sram marks landed, because no mmc node bound and
		 * the wrong devnum was never reached.  See
		 * MESON_GX_MMC_SD_DEVNUM.
		 */
		return gx_tpl_load_spl_mmc(bd == BOOT_DEVICE_EMMC ?
					   CONFIG_MESON_GX_MMC_EMMC_DEVNUM :
					   CONFIG_MESON_GX_MMC_SD_DEVNUM, dst,
					   CONFIG_MESON_GX_SPL_MAX_SIZE, err);
#else
		return 0;
#endif

	case BOOT_DEVICE_SPI:
		if (!IS_ENABLED(CONFIG_MESON_GX_LOAD_NOR)) {
			*err = GX_TPL_LOAD_NO_NOR;
			return 0;
		}
#if IS_ENABLED(CONFIG_MESON_GX_LOAD_NOR)
		return gx_tpl_load_spl_nor(dst, CONFIG_MESON_GX_SPL_MAX_SIZE,
					   err);
#else
		return 0;
#endif
	}

	*err = GX_TPL_LOAD_NO_SRC;
	return 0;
}

/**
 * gx_tpl_jump_to_spl() - hand control to the DRAM-resident stage.  Never returns.
 *
 * The last thing the SRAM stage does.  Three things happen first, in this order,
 * and each of them is load-bearing:
 *
 *  - The watchdog is STOPPED.  The DRAM stage cannot pet it and does not know it
 *    exists, so leaving it armed means a reset lands somewhere inside the next
 *    stage at a time nobody chose -- destroying the SRAM evidence that is the
 *    whole point of the handoff block.  A stage that hangs visibly is worth more
 *    than one that reboots mysteriously.
 *
 *  - The entry word is checked.  0x00000000 and 0xffffffff are what an address
 *    nobody wrote reads back as, and branching into either is an undefined
 *    instruction fault with no breadcrumb.  Refusing is a reported failure; the
 *    branch is not.  This is a sanity check on the ADDRESS, not a validation of
 *    the image -- validating the handoff is the DRAM stage's own job, and it
 *    refuses rather than recovers.
 *
 *  - The I-cache is invalidated.  TPL enabled it before the bulk transfer, and
 *    the SPL image arrived at this address afterwards; a stale line there would
 *    execute whatever was cached from the same physical address earlier.
 */
static void gx_tpl_jump_to_spl(void)
{
	struct gx_xpl_handoff *h = (struct gx_xpl_handoff *)GX_HANDOFF_ADDR;
	u32 entry = CONFIG_MESON_GX_SPL_ENTRY;
	u32 err = 0, len, first;

	/*
	 * Fetch first, then look.  On a pull medium the entry word is whatever
	 * the last boot left in DRAM until this runs, so checking before
	 * fetching would test the wrong thing.  A failed fetch is NOT fatal by
	 * itself: on the USB path the bytes may already be there from the
	 * session, and only the length is unknown.  The entry word is the
	 * single test that decides, below.
	 */
	len = gx_tpl_fetch_spl(&err);
	first = readl((void *)(uintptr_t)entry);

	if (first == 0 || first == 0xffffffff) {
		/*
		 * Nothing was loaded here.  Say so and stop.
		 *
		 * NOT because there is no way back -- there is, and this same
		 * file already uses it: meson_u1_mark_and_return()'s SMC
		 * 0xC0000000 re-enters BL1's USB service in place, and the DRAM
		 * stage now ends in a copy of it (spl-dram-stage.c) precisely so
		 * its report can be read.  This path is deliberately NOT
		 * converted in the same change: TPL got here with the DWC2 torn
		 * down by meson_usb_device_stop(), including its SFTDISCON
		 * assert, and whether BL1 recovers from that is unmeasured.  The
		 * DRAM stage is where that question gets answered first, on one
		 * path, with one mitigation.  Convert this one after a rig run
		 * says which mitigation is right -- not before.
		 */
		early_puts("NOSPL\n");
		/*
		 * Report why the FETCH failed, not the generic bad-address
		 * code: "TPL booted from NOR and has no NOR loader" and "the
		 * card read came up short" are different bugs, and after this
		 * hang the SRAM word is the only thing anyone gets to read.
		 * The fault word carries the boot device so the report names
		 * the medium too.
		 */
		writel(GX_DRAM_STAGE_MAGIC |
		       (err ? err : (u32)GX_DRAM_STAGE_BAD_ADDR),
		       (void *)GX_DRAM_STAGE_ADDR);
		writel(err ? (u32)meson_get_boot_device() : first,
		       (void *)GX_DRAM_FAULT_ADDR);
		prod_hang(0x7e);
	}

	/*
	 * Publish the pair before branching, and re-stamp magic: the DRAM stage
	 * reads spl_addr back to prove it is running where TPL aimed.  spl_len
	 * is what TPL actually moved -- 0 still means "TPL did not load this and
	 * does not know how long it is", which stays the truthful answer when
	 * the host pushed the image over USB.
	 */
	writel(0, &h->magic);
	writel(entry, &h->spl_addr);
	writel(len, &h->spl_len);
	writel(readl(&h->flags) | GX_HANDOFF_F_SPL, &h->flags);
	asm volatile("dsb sy" ::: "memory");
	writel(GX_HANDOFF_MAGIC, &h->magic);

	wdt_stop();
	early_vputs("JMP\n");
	prod_stage(0x7b);
	writel(readl((void *)GX_XPL_PROG_ADDR) | GX_XPL_PROG_JUMP,
	       (void *)GX_XPL_PROG_ADDR);

	asm volatile("dsb sy\nic iallu\ndsb sy\nisb" ::: "memory");
	((void (*)(void))(uintptr_t)entry)();

	/* A return from the next stage is as much a failure as never reaching it. */
	prod_hang(0x7f);
}
#endif /* CONFIG_MESON_GX_XPL_CHAIN */
#endif

/*
 * ep_info buffer passed as x1 to the initial ROM SMC.
 * Vendor BL2 (bl2_new.bin:0x800) uses 0xD9001880 for this; we allocate
 * an aligned BSS buffer in the same SRAM region.  Passing null (xzr) was
 * wrong: the ROM reads and writes [x1] during its init handler.
 */

#if defined(CONFIG_MESON_G12A) && defined(CONFIG_MESON_G12A_XPL_CHAIN)
/**
 * g12a_tpl_write_handoff() - publish what this stage learned for the DRAM stage.
 * @ddr_ok: non-zero only when mailbox train completed (g12a_ddr_fw_train_ok).
 *          Not dram_init()==0 — stub returns 0 for banner continuity without
 *          claiming DRAM ready (L3.A.6 honest F_DDR_OK).
 */
static void g12a_tpl_write_handoff(int ddr_ok)
{
	struct g12a_xpl_handoff *h = (struct g12a_xpl_handoff *)G12A_HANDOFF_ADDR;
	u32 flags = 0;
	u32 train_st = g12a_ddr_fw_get_train_status();

	if (ddr_ok)
		flags |= G12A_HANDOFF_F_DDR_OK;

	writel(0, &h->magic);
	writel(G12A_HANDOFF_VERSION, &h->version);
	writel(flags, &h->flags);
	/*
	 * Size not yet probed on G12A open train.  Use board default 2048 MB
	 * for Solitude LPDDR4 class so the DRAM stage range-check can pass once
	 * F_DDR_OK is real; without train complete DRAM stage REFUSE BAD_DDR.
	 */
	writel(2048, &h->dram_size_mb);
	writel(0, &h->dram_clk_mhz);
	writel(0, &h->ddr_flags);
	writel((u32)meson_get_boot_device(), &h->boot_device);
	writel(0, &h->payload_addr);
	writel(0, &h->payload_len);
	writel(0, &h->run_addr);
	writel(0, &h->spl_addr);
	writel(0, &h->spl_len);
	writel(train_st, &h->train_status);
	writel(0, &h->pgsr0);
	writel(0, &h->tune_params);
	writel(readl((void *)G12A_XPL_PROG_ADDR) & 0xffff, &h->progress);

	asm volatile("dsb sy" ::: "memory");
	writel(G12A_HANDOFF_MAGIC, &h->magic);
	writel(readl((void *)G12A_XPL_PROG_ADDR) | G12A_XPL_PROG_HANDOFF,
	       (void *)G12A_XPL_PROG_ADDR);
}

#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
int meson_g12_usb_residual_serve(unsigned int timeout_ms);
int meson_g12_usb_residual_serve2(unsigned int timeout_ms);
u32 meson_g12_usb_spl_payload_len(void);
void meson_g12_usb_device_stop(void);
#endif

/**
 * g12a_tpl_jump_to_spl() - hand control to the DRAM-resident stage.
 *
 * When residual is ON, bulk window already ran *before* dram_init (AHB
 * holds feed train; optional SPL @ 0x08000000).  Here: entry-word check
 * empty head → NOSPL; jump if word present.
 */
static void g12a_tpl_jump_to_spl(void)
{
	struct g12a_xpl_handoff *h = (struct g12a_xpl_handoff *)G12A_HANDOFF_ADDR;
	u32 entry = CONFIG_MESON_G12A_SPL_ENTRY;
	u32 first, len = 0;

#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
	len = meson_g12_usb_spl_payload_len();
#endif

	first = readl((void *)(uintptr_t)entry);
#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
	/*
	 * A non-zero entry word is NOT proof of a complete image.  Boot spl-2
	 * jumped into a payload the residual had only partly received (crumb
	 * w20 = zero completed frames) because the first 0x30000 of it had
	 * landed; the DRAM stage then ran and stopped at "Trying to boot from
	 * RAM" with no FIT behind it.  With residual on, the delivered length
	 * is the authority.
	 */
	if (first != 0 && first != 0xffffffffu && len == 0)
		early_puts("PARTSPL\n");
#endif
	if (first == 0 || first == 0xffffffff
#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
	    || len == 0
#endif
	   ) {
		early_puts("NOSPL\n");
		writel(G12A_DRAM_STAGE_MAGIC | G12A_TPL_LOAD_USB_NONE,
		       (void *)G12A_DRAM_STAGE_ADDR);
		writel((u32)meson_get_boot_device(),
		       (void *)G12A_DRAM_FAULT_ADDR);
		prod_hang(0x7e);
	}

	writel(0, &h->magic);
	writel(entry, &h->spl_addr);
	writel(len, &h->spl_len);
	{
		u32 flags = readl(&h->flags) | G12A_HANDOFF_F_SPL;

#if CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
		if (len)
			flags |= G12A_HANDOFF_F_USB;
#endif
#if defined(CONFIG_MESON_G12A_RESIDENT)
		if (len)
			flags |= G12A_HANDOFF_F_FIT_RAM;
#endif
		writel(flags, &h->flags);
	}
	asm volatile("dsb sy" ::: "memory");
	writel(G12A_HANDOFF_MAGIC, &h->magic);

	wdt_stop();
	early_puts("JMP\n");
	prod_stage(0x7b);
	writel(readl((void *)G12A_XPL_PROG_ADDR) | G12A_XPL_PROG_JUMP,
	       (void *)G12A_XPL_PROG_ADDR);

	asm volatile("dsb sy\nic iallu\ndsb sy\nisb" ::: "memory");
	((void (*)(void))(uintptr_t)entry)();
	prod_hang(0x7f);
}
#endif /* CONFIG_MESON_G12A_XPL_CHAIN */

/*
 * The DRAM stage has its own board_init_f in spl-dram-stage.c (G12A/SM1)
 * or spl-dram-stage-gx.c (GXL).  Each SoC's DRAM-stage build defines its
 * MESON_*_DRAM_STAGE and must not compile this SRAM-stage board_init_f.
 */
#if !defined(MESON_G12A_DRAM_STAGE) && !defined(MESON_GX_DRAM_STAGE)
/*
 * Zero .bss.  crt0 clears it only when board_init_f() returns
 * (arch/arm/lib/crt0_64.S), and this SRAM stage's never does: it jumps to
 * the DRAM stage or hangs.  So every static below started as whatever SRAM
 * held -- BL1's stack, or the previous boot's values (a TPL static read 0x67
 * where the code assumed 0, 2026-10-14).  Nothing live sits in the range at
 * entry: the host's and BootROM's SRAM data are in the breadcrumb window
 * below it (GXL, static_assert in gx-handoff.h) or in the hold buffers above
 * it (G12, g12a-ddr-fw.h), and gd, the early malloc arena and this frame are
 * under TPL_STACK, above __bss_end.
 */
static void meson_sram_stage_bss_zero(void)
{
	memset(__bss_start, 0, __bss_end - __bss_start);
}

void board_init_f(ulong dummy)
{
	int ret;

	meson_sram_stage_bss_zero();
#if defined(CONFIG_MESON_G12A_RESIDENT)
	u32 g12a_resident_arm(void);
	u32 g12a_resident_word(void);
#if defined(CONFIG_MESON_G12A_RESIDENT_M20_PROBE)
	void g12a_resident_m20_ee(u32 *v);
	void g12a_resident_m20_sample(u32 *v);
	u32 m20_ee[2];

	/* Before the arm rewrites it: the EE watchdog as the reset left it. */
	g12a_resident_m20_ee(m20_ee);
#endif
#if defined(CONFIG_MESON_G12A_RESIDENT_M21_AO_SPIN)
	{
		/*
		 * TEST ONLY (plan M2.1): stop the EE watchdog, BootROM arm
		 * included, and spin with no SC loaded; only the AO watchdog can
		 * reset the chip from here.  The board never boots past this.
		 */
		void g12a_resident_m21_ee_off(u32 *v);
		u32 m21[3];
		int i;

		g12a_resident_m21_ee_off(m21);
		early_puts("M21 ao/ee/ee-off");
		for (i = 0; i < 3; i++) {
			early_puts(" ");
			early_puthex(m21[i]);
		}
		early_puts(" spin\n");
		for (;;)
			;
	}
#endif
	/*
	 * First act of a TPL the BootROM may have loaded from eMMC: arm the
	 * USB fallback (g12-resident.c) before anything that can hang.  Only
	 * register writes, so it goes ahead of meson_timer_rate_init(); what
	 * runs before it is start.S and crt0_64.S (no I/O, no wait).
	 */
	u32 armed = g12a_resident_arm();
#endif

	meson_timer_rate_init();

#if defined(CONFIG_MESON_G12A_RESIDENT)
	{
		/*
		 * "UF arm" + the EE watchdog CNTL read-back when armed, then the
		 * override word ("rig" mode) or "sa" ("standalone" mode, no word).
		 */
		early_puts(armed ? "UF arm " : "UF usb\n");
		if (armed) {
			early_puthex(armed);
			early_puts(IS_ENABLED(CONFIG_MESON_G12A_RESIDENT_OVERRIDE) ?
				   " ovd " : " sa\n");
			if (IS_ENABLED(CONFIG_MESON_G12A_RESIDENT_OVERRIDE)) {
				early_puthex(g12a_resident_word());
				early_puts("\n");
			}
		}
	}
#endif
#if defined(CONFIG_MESON_G12A_RESIDENT_M20_PROBE)
	{
		/*
		 * TEST ONLY (plan M2.0): no SC has been loaded on this boot yet
		 * (the SPL loads it), so a heartbeat tick or pet count that
		 * advances across 0.5 s is an SC that survived the reset.
		 */
		u32 s[2][7];
		int i, j;

		g12a_resident_m20_sample(s[0]);
		udelay(500000);
		g12a_resident_m20_sample(s[1]);
		early_puts("M20 ee ");
		early_puthex(m20_ee[0]);
		early_puts(" ");
		early_puthex(m20_ee[1]);
		early_puts("\n");
		for (i = 0; i < 2; i++) {
			early_puts(i ? "M20 s2" : "M20 s1");
			for (j = 0; j < 7; j++) {
				early_puts(" ");
				early_puthex(s[i][j]);
			}
			early_puts("\n");
		}
	}
#endif

/*
 * U1/U2 closed-loop bring-up (U1_ONLY_MINIMAL):
 *   0 = product: power/DDR -> optional N8 ROM-return -> FIT USB
 *   1 = N8 early: OMOR + BL1 USB return via SMC 0xC0000000 (no DDR)
 *   2 = C worldcup reenum forever
 *   3 = asm 02 22 then hang (isolation; host-only)
 *   4 = PHY POR soft-connect isolation
 *   5 = S2 isolation: ACM/bulk console forever (no DDR)
 *   6 = N8 post-DDR force: power/DDR then meson_bl1_usb_return (no FIT)
 *
 * Product path also honors host SRAM request at 0xd900c114 =
 * GXL_N8_REQ_MAGIC after successful DDR (tools/spl-n8-verify.py).
 * RE: gxl-blx-standouts.md #1 (SMC 0xC0000000, not WDT).
 */
#define U1_ONLY_MINIMAL 0	/* product default */
/* Force post-DDR USB return-to-ROM (N8); 0 = only host magic / mode 6. */
#ifndef GXL_N8_USB_ROM
#define GXL_N8_USB_ROM 0
#endif
/* Host writes this LE u32 @ 0xd900c114 before RUN to request N8 after DDR. */
#define GXL_N8_REQ_ADDR	0xd900c114UL
#define GXL_N8_REQ_MAGIC	0x4e380001u /* N8 + 1 */
#if U1_ONLY_MINIMAL == 1
	/* N8 early: return-to-ROM without power/DDR (standouts #1 SMC path). */
	meson_u1_mark_and_return();
#elif U1_ONLY_MINIMAL == 2
	/* C worldcup forever (poll after ID — host keep-handle 02 22) */
	meson_u1_try_identify_or_return(30000);
#elif U1_ONLY_MINIMAL == 3
	meson_u1_asm_identify();
	while (1)
		;
#elif U1_ONLY_MINIMAL == 4
	/*
	 * Isolation: PHY POR soft-connect ASAP (no SMC/power/DDR).
	 * Host: after RUN, wait re-attach, identify 02 22.
	 */
	early_puts("4\n");
	if (meson_usb_reenum_phy_por(60000) == 0)
		early_puts("u\n");
	else
		early_puts("ut\n");
	for (;;)
		meson_usb_poll_once();
#elif U1_ONLY_MINIMAL == 5
	early_puts("A\n");
	meson_usb_acm_serve(15000);
#endif

#if defined(CONFIG_MESON_GXL) && (U1_ONLY_MINIMAL == 0)
	/*
	 * Product bring-up with stop-point isolation.
	 * PROD_STOP_AT: hang after that stage (binary-search reboot).
	 *   0 = after stage0 (before id)
	 *   1 = after early 02 22, USB still live (poll hang)
	 *   2 = after soft-disconnect (before SMC)
	 *   3 = after SMC
	 *   4 = after power
	 *   5 = after DDR
	 *   0 = full product (default)
	 * Stage @ 0xd900c120 = 0x5354xxxx
	 */
/*
	 * PROD_STOP_AT isolation (set in this file for bring-up):
	 *  0 = full product
	 *  1 = hang after early 02 22 (USB live)
	 *  9 = no early USB; skip SMC; power+DDR+FIT only
	 */
/*
	 * Vendor order: SMC before long USB serve. Early worldcup 02 22 then
	 * hang dies ~15 s later with stage=1 — likely BROM WDT (not yet
	 * disabled; our WDT enable is after this block). Skip early USB;
	 * identify after DDR via receive_fit (02 22 + bulk).
	 */
#ifndef PROD_STOP_AT
#define PROD_STOP_AT 0
#endif
	prod_stage(0);
#if PROD_STOP_AT == 1
	/* Debug only: early id + hang (will BROM-WDT reboot ~15 s) */
	early_puts("i\n");
	if (meson_usb_reenum_serve(15000) == 0)
		early_puts("I\n");
	prod_stage(1);
	for (;;)
		meson_usb_poll_once();
#endif
#endif

	/*
	 * Product path (measured 2026-08-04):
	 *  - ROM SMC hangs (stage 0x10) — skip; no VBAR/SCR (SError trap).
	 *  - Worldcup receive_fit after DDR → 02 22 green; PHY POR no attach.
	 *  - Bulk WR_LARGE still open (timeout after 02 22).
	 */
	wdt_start_ms(8000);
	prod_stage(0x5C); /* smc_skipped */
	wdt_pet();
	early_vputs("1\n");

	/*
	 * The banner and chip id under the TPL's own early console
	 * (include/chip_serial.h): this stage jumps on before
	 * preloader_console_init(), which says both for the stages that reach
	 * it, in the same form.
	 */
	if (IS_ENABLED(CONFIG_TPL_BUILD)) {
		char id[CHIP_SERIAL_MAX];

		if (IS_ENABLED(CONFIG_MESON_GX))
			early_puts("\nU-Boot TPL " PLAIN_VERSION " (" U_BOOT_DATE " - "
				   U_BOOT_TIME " " U_BOOT_TZ ")\n");

		if (chip_serial_get(id, sizeof(id))) {
			early_puts("Chip id: unreadable\n");
		} else {
			early_puts("Chip id: ");
			early_puts(id);
			early_puts("\n");
		}
	}

	prod_stage(0x53);
	meson_power_init();
	early_vputs("2\n");
	prod_stage(4);
	wdt_pet();

	early_vputs("4\n");
	prod_stage(5);
	wdt_pet();

#if defined(CONFIG_MESON_G12A) && defined(CONFIG_MESON_G12A_XPL_CHAIN) && \
	CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
	/*
	 * Residual BEFORE train so host bulk can park Strategy A holds on
	 * AHB (aml_ddr @ HOLD0/dest, piei @ HOLD1/dest) and optional SPL.
	 * Single mid-session window (no SFTDISCON); 20 s host budget.
	 * Order: residual → dram_init/train → handoff → JMP/NOSPL.
	 */
#if CONFIG_IS_ENABLED(MESON_G12A_EMMC_FETCH)
	/*
	 * Resident-chain step 5: window 1 from the staged eMMC layout, into
	 * the same splitter the USB window feeds.  The source is chosen by
	 * the option, not by meson_get_boot_device(): this TPL still arrives
	 * over USB.  A failure is printed; training then runs on whatever the
	 * holds contain and reports for itself.
	 */
#if defined(CONFIG_MESON_G12A_RESIDENT)
	/*
	 * Resident build: the BootROM's boot device picks the source.  eMMC
	 * reads the staged layout; any other device (USB, the rig's recovery
	 * path) keeps today's USB window.
	 */
	{
		int g12a_resident_from_emmc(void);
		int g12_tpl_emmc_fetch_w1(void);

		if (g12a_resident_from_emmc()) {
			early_puts("EF1\n");
			if (g12_tpl_emmc_fetch_w1())
				early_puts("EF1 FAIL\n");
		} else {
			early_puts("RS\n");
			(void)meson_g12_usb_residual_serve(30000);
		}
	}
#else
	{
		int g12_tpl_emmc_fetch_w1(void);

		early_puts("EF1\n");
		if (g12_tpl_emmc_fetch_w1())
			early_puts("EF1 FAIL\n");
	}
#endif
#else
	early_puts("RS\n");
	(void)meson_g12_usb_residual_serve(30000);
#endif
	wdt_pet();
#endif

	ret = dram_init();
#if defined(CONFIG_MESON_G12A_RESIDENT)
	{
		/* The vendor DDR firmware re-arms the watchdog at 5 s; undo it. */
		void g12a_resident_rearm(void);

		g12a_resident_rearm();
	}
#endif
	if (ret) {
		early_puts("DF\n");
		prod_stage(0xDF);
		/*
		 * Stop with the watchdog ARMED, so the board resets into a fresh
		 * BootROM that a host can stage again.  The report is the console
		 * (every attempt and its failure is printed above) plus the sticky
		 * train step/PGSR0 @ 0xd900c128/c12c.
		 *
		 * This used to stop the watchdog and hand USB back to BL1
		 * (meson_u1_mark_and_return()) so the host could read the sticky
		 * words over the BootROM.  The re-entered BL1 answers no EP0
		 * request on either GXL board today -- v1 (N42, and again here
		 * with the DWC2 untouched by this stage) and CC-V2 (n1204) -- so
		 * the hand-back bought nothing and left a board that only a power
		 * cycle recovered.  GXL DDR3 FMA round F3 FA3: claude amlogic/
		 * software/boot/gxl/audit/2026-10-19-gxl-ddr3-boot-chain-failure-
		 * modes.md.
		 *
		 * The DWC2 is stopped first: left soft-connected as the BootROM
		 * left it, the BootROM after the watchdog reset stalled on the
		 * host's first write (see meson_usb_disconnect_for_reset()).
		 */
#if defined(CONFIG_MESON_GXL) && CONFIG_IS_ENABLED(MESON_USB_BOOT)
		{
			void meson_usb_disconnect_for_reset(void);

			meson_usb_disconnect_for_reset();
		}
#endif
		prod_hang(0xDE);
	}
	/* DDR is trained: the boot log moves from SRAM to DRAM */
	gx_bootlog_dram_ready();
	early_vputs("5\n");
	prod_stage(6);

#if defined(CONFIG_MESON_G12A) && defined(CONFIG_MESON_G12A_XPL_CHAIN) && \
	CONFIG_IS_ENABLED(MESON_G12A_USB_RESIDUAL)
	/*
	 * Second residual window.  0x08000000 (DRAM stage) and 0x02000000
	 * (boot FIT) only exist now — before dram_init they are unbacked, so
	 * the pre-train window CANNOT carry them and every boot to date has
	 * correctly stopped at NOSPL for want of an image, not for want of a
	 * loader.  Host sends ONE G12R frame dest=0x08000000 carrying
	 * out/<board>.usb.payload; the splitter cuts it at SPL_FIT_SPLIT.
	 */
#if CONFIG_IS_ENABLED(MESON_G12A_EMMC_FETCH)
	/* Window 2 from the staged eMMC layout; no length on failure → NOSPL. */
#if defined(CONFIG_MESON_G12A_RESIDENT)
	{
		int g12a_resident_from_emmc(void);
		int g12_tpl_emmc_fetch_w2(void);

		if (g12a_resident_from_emmc()) {
			early_puts("EF2\n");
			if (g12_tpl_emmc_fetch_w2())
				early_puts("EF2 FAIL\n");
		} else {
			spl_tsm("rs2");
			early_puts("RS2\n");
			(void)meson_g12_usb_residual_serve2(60000);
			spl_tsm("rs2got");
		}
	}
#else
	{
		int g12_tpl_emmc_fetch_w2(void);

		early_puts("EF2\n");
		if (g12_tpl_emmc_fetch_w2())
			early_puts("EF2 FAIL\n");
	}
#endif
#else
	spl_tsm("rs2");
	early_puts("RS2\n");
	(void)meson_g12_usb_residual_serve2(60000);
	spl_tsm("rs2got");
#endif
	wdt_pet();
#endif

#if defined(CONFIG_MESON_G12A) && defined(CONFIG_MESON_G12A_XPL_CHAIN) && \
	CONFIG_IS_ENABLED(MESON_G12A_EMMC_REG) && \
	!CONFIG_IS_ENABLED(MESON_G12A_EMMC_FETCH)
	/*
	 * Register-level eMMC reader self-test (resident-chain step 4).  After
	 * both USB windows, so its time cannot stall a bulk transfer; before
	 * the handoff, so a failed read still reaches the DRAM stage.  It uses
	 * no DRAM: blocks are folded into a crc straight out of the
	 * controller's buffer.  Every wait inside is bounded at 1 s.
	 */
	{
		void g12_tpl_emmc_selftest(void);

		wdt_pet();
		g12_tpl_emmc_selftest();
		wdt_pet();
	}
#endif

#if defined(CONFIG_MESON_G12A) && defined(CONFIG_MESON_G12A_XPL_CHAIN)
	/*
	 * T1: TPL owns DDR stubs + handoff + jump.  FIT/ATF is the DRAM
	 * stage.  With residual ON, holds/SPL already bulk-received above.
	 * Without residual, host must pre-place SPL or TPL stops with NOSPL.
	 */
	/*
	 * Honest F_DDR_OK: only when mailbox reported complete. Without FW or
	 * without ACS/PHY IMEM (L3.A.7) train_ok is 0 → DRAM stage BAD_DDR.
	 */
	g12a_tpl_write_handoff(g12a_ddr_fw_train_ok());
	prod_stage(0x7a);
	early_puts("HO\n");
	g12a_tpl_jump_to_spl();
	/* not reached */
#endif

	/*
	 * Full meson_pll_init() before receive_fit kills worldcup (p47).
	 * Only mux CPU to SYS PLL if already locked — no MPLL touch.
	 */
#if defined(CONFIG_MESON_GXL)
	/* Sticky flag @ c130 (c10c clobbered by worldcup DIEPCTL dump)
	 * 0x53540061 = try_sys mux OK; 0x53540062 = SYS PLL not locked.
	 */
	if (meson_cpu_sys_pll_force_1g2())
		writel(0x53540063u, (void *)0xd900c130UL);
	else if (meson_cpu_try_sys_pll())
		writel(0x53540061u, (void *)0xd900c130UL);
	else
		writel(0x53540062u, (void *)0xd900c130UL);
	/*
	 * ...and the one piece of the skipped MPLL sequence the platform
	 * cannot do without: without it fclk_div3 is dead, which is the
	 * clock meson-gxl.dtsi hands the VPU.  Safe here — it touches only
	 * HHI_MPLL_CNTL10, never the fixed PLL's own control register.
	 */
	meson_fixed_pll_enable_div3();
	/*
	 * Both values this stage writes and cannot itself validate, stated
	 * once here -- after the DMC re-assert in dram_init() and after the
	 * fclk_div3 write above, so both are final.
	 *
	 * Runs on DDR3 too, deliberately: see meson_spl_selfcheck().
	 */
	meson_spl_selfcheck();
	/*
	 * The DOS capability word the vendor's BL2 publishes and ours did not
	 * -- BIT(2) of DOS_SCRATCH31, without which the GXL VP9 firmware never
	 * reads a byte of bitstream (N95).  Here rather than in U-Boot proper
	 * because this stage is what replaces the vendor's BL2, and after
	 * dram_init() only for ordering hygiene: it needs no DRAM, only the
	 * DOS clock gate, which is already on.
	 */
	meson_gx_dos_publish_caps();
#if defined(CONFIG_TPL_MESON_GX_NOR_PROBE)
	/*
	 * SPI NOR, read only, BEFORE the USB session rather than after.
	 *
	 * Both orders were available and this one is not arbitrary: the dev
	 * loop's second bulk transfer is a known intermittent (design doc,
	 * "a second WR_LARGE in one TPL session"), so a probe placed after it
	 * would inherit that flakiness and its result would be missing exactly
	 * on the runs that fail.  Here it is independent of USB entirely.
	 *
	 * It needs driver model, which the MMC arm otherwise brings up much
	 * later.  spl_early_init() is NOT idempotent -- it re-runs
	 * spl_common_init(), which re-scans DM -- so both call sites go through
	 * gx_tpl_dm_init(), which does it once.
	 */
	if (!gx_tpl_dm_init())
		gx_tpl_nor_probe();
	else
		early_puts("NORDM\n");
#endif
#if defined(CONFIG_TPL_MESON_GX_MMC_PROBE)
	/*
	 * SD/eMMC, read only, in the same place and for the same reasons as
	 * the NOR probe above: before the USB session so its result does not
	 * inherit the second-bulk-transfer intermittent, and behind
	 * gx_tpl_dm_init() because spl_early_init() is not idempotent.
	 *
	 * This is NOT the boot path.  gx_tpl_fetch_spl() still reaches
	 * tpl-load-mmc.c only when the ROM booted from a card; the probe is
	 * what makes that path observable on a rig that boots over USB.
	 */
	if (!gx_tpl_dm_init())
		gx_tpl_mmc_probe();
	else
		early_puts("MMCDM\n");
#endif
	/*
	 * N8 USB return-to-ROM after DDR (gxl-blx-standouts.md #1).
	 * Trigger: GXL_N8_USB_ROM=1, U1_ONLY_MINIMAL==6, or host magic @
	 * 0xd900c114 before RUN.  SMC 0xC0000000 / check_handler — not WDT.
	 *
	 * Gated on the object that defines the SMC helper, not on the SoC: a
	 * return-to-ROM has no meaning in a build with no USB stack to return.
	 */
#if CONFIG_IS_ENABLED(MESON_USB_BOOT)
	{
		u32 n8_req = readl((void *)GXL_N8_REQ_ADDR);
		int n8 = (GXL_N8_USB_ROM != 0) || (U1_ONLY_MINIMAL == 6) ||
			(n8_req == GXL_N8_REQ_MAGIC);

		if (n8) {
			if (n8_req == GXL_N8_REQ_MAGIC)
				writel(0, (void *)GXL_N8_REQ_ADDR);
			prod_stage(0xe8); /* N8 about to SMC */
			early_puts("N8\n");
			wdt_stop();
			/* OMOR marker then check_handler SMC into BL1 USB. */
			meson_u1_mark_and_return();
			/* not reached on success; fail stage left by meson_bl1_usb_return */
			prod_hang(0xe9);
		}
	}
#endif /* MESON_USB_BOOT */
#endif
	wdt_start_ms(120000); /* FIT window (clamped 65535 ms) */
	wdt_pet();

	/*
	 * USB boot flow: SPL acts as a USB device after DDR init.
	 *
	 * After DDR init succeeds, SPL re-initializes the DWC2 OTG controller
	 * and enumerates as the Amlogic ROM USB device (VID=1b8e PID=c003).
	 * The host (meson-usb-boot.py) writes a FIT image to DRAM via the
	 * Amlogic USB protocol (REQ_WR_LARGE_MEM + bulk OUT), then sends
	 * REQ_RUN_IN_ADDR to signal completion.  SPL then continues to
	 * spl_init() which loads the FIT from RAM.
	 *
	 * This replaces the broken two-pass watchdog-reset approach: a GXL
	 * watchdog reset also resets the DDR controller, making DRAM
	 * inaccessible via the ROM USB protocol after reset.
	 */
#if CONFIG_IS_ENABLED(MESON_USB_BOOT)
	/*
	 * Re-enumerate for the next stage after DDR, unless the BootROM booted
	 * this stage from a medium the chain pulls the next stage from itself:
	 * SPI NOR (MESON_GX_LOAD_NOR) or SD/eMMC (TPL_MESON_GX_LOAD_MMC).  Then
	 * every stage comes from that medium (gx_tpl_fetch_spl()), and a USB
	 * session here would wait for a host that is not there.
	 *
	 * Every other boot-id -- USB, and anything unexpected -- still takes
	 * the USB session, as this always did: an AO boot-id that disagrees
	 * with "we were loaded via BROM USB" must not skip receive_fit.
	 */
	{
		u32 bd = (u32)meson_get_boot_device();
		bool pull = false;

#if defined(CONFIG_MESON_GX_XPL_CHAIN)
		pull = (bd == BOOT_DEVICE_SPI && IS_ENABLED(CONFIG_MESON_GX_LOAD_NOR)) ||
		       ((bd == BOOT_DEVICE_SD || bd == BOOT_DEVICE_EMMC) &&
			IS_ENABLED(CONFIG_TPL_MESON_GX_LOAD_MMC));
#endif
		prod_stage(0xB000 | (bd & 0xff));
		if (!IS_ENABLED(CONFIG_MESON_GX) || IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE)) {
			early_puts("BOOT ");
			early_puthex(bd);
			early_puts(pull ? " pull\n" : "\n");
		}
		/* I-cache before bulk PIO (was after receive_fit), or before the pull */
		icache_enable();
		if (!pull) {
			early_vputs("USB\n");
			meson_usb_receive_fit();
			early_vputs("FIT\n");
			prod_stage(7);
		}
	}
#endif

#if defined(CONFIG_TPL_BUILD) && defined(CONFIG_MESON_GXL)
	/*
	 * T1: TPL = DDR init + USB receive + handoff block.  Filled here, after
	 * meson_usb_receive_fit() has run device_stop() -- the receive session
	 * is over, so payload_addr/len are final and the DWC2 is not going to
	 * write anything else.  ddr_ok is unconditionally 1: a dram_init()
	 * failure never reaches this point, it goes to prod_stage(0xDF) and
	 * hands USB back to the ROM so the host can read the breadcrumbs.
	 */
	gx_tpl_write_handoff(1);
	prod_stage(0x7a); /* handoff block published */
#if defined(CONFIG_MESON_GX_XPL_CHAIN)
	/*
	 * T2: the SRAM stage ends here.  Everything below this point -- the
	 * FIT, the ATF handoff -- is the DRAM stage's, which is the entire
	 * reason for the split.  gx_tpl_jump_to_spl() does not return.
	 *
	 * T3 pulls ONE thing back across that line: driver model, and only when
	 * TPL has to read a block device.  Fetching SPL is the SRAM stage's job
	 * by definition -- it is the stage that knows which medium the ROM used
	 * -- and a block read needs the MMC controller bound.  A TPL with no
	 * pull medium (the USB path) never runs this and pays nothing for it.
	 */
	if (IS_ENABLED(CONFIG_TPL_MESON_GX_LOAD_MMC)) {
		ret = gx_tpl_dm_init();
		if (ret) {
			early_puts("DM\n");
			prod_hang(0x79);
		}
	}
	gx_tpl_jump_to_spl();
#endif
#endif

	if (CONFIG_IS_ENABLED(OF_CONTROL)) {
		ret = spl_early_init();
		if (ret) {
			debug("spl_early_init() failed: %d\n", ret);
			while (1)
				;
		}
	}

	spl_init();
	icache_enable();
	preloader_console_init();

	/*
	 * Where the BootROM found us. Compiled out of the TPL phase
	 * (TPL_BOOTSOURCE defaults off) so a TPL -> SPL chain prints it once,
	 * from the DRAM stage; a single-stage SPL prints it here.
	 */
	if (CONFIG_IS_ENABLED(BOOTSOURCE))
		bootsource_announce(meson_bootsource());
}
#endif /* !MESON_G12A_DRAM_STAGE && !MESON_GX_DRAM_STAGE */

/* Bare-metal UART for early debug - uses same method as spl-boot-debug.S */
static void __attribute__((used)) spl_uart_putc(char c)
{
#if defined(CONFIG_MESON_G12A)
	volatile u32 *wfifo = (volatile u32 *)0xff803000;
	volatile u32 *status = (volatile u32 *)0xff80300c;
#else
	volatile u32 *wfifo = (volatile u32 *)0xc81004c0;
	volatile u32 *status = (volatile u32 *)0xc81004cc;
#endif
	int timeout = 100000;

	/* Never infinite-wait: if AO UART is unclocked/stuck, USB boot
	 * still needs to reach meson_usb_reenum_serve(). */
	while ((*status & (1 << 21)) && --timeout)
		;
	if (timeout)
		*wfifo = c;
	gx_bootlog_putc(c);
}
static void __attribute__((used)) spl_uart_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			spl_uart_putc('\r');
		spl_uart_putc(*s++);
	}
}
