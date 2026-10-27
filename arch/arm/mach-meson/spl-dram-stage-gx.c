// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX TPL -> SPL -> FIT chain: the DRAM-resident stage (T2).
 *
 * The BootROM-loaded stage lives in SRAM under a hard 0xb000 amlimage payload
 * ceiling.  This one does not: TPL trains DDR, fetches this image, publishes a
 * handoff block and branches here, at CONFIG_MESON_GX_SPL_ENTRY in DRAM, where
 * the only limits are the ones the memory map imposes.
 *
 * This stage validates the handoff block and, if it is good, CONTINUES: it
 * falls through into the generic xPL flow (spl_early_init / spl_init /
 * board_init_r), which is the same code the shipping single-stage SPL uses to
 * parse the FIT and invoke BL31.  It gains no loader of its own -- see the
 * comment at the end of board_init_f() for why that is the point rather than a
 * shortcut.  T2 stopped here; T3 item 4 is this fall-through.
 *
 * Two rules shape the whole file:
 *
 *  - It REFUSES a bad handoff.  It does not re-probe DDR, re-read a size, or
 *    fall back to a default and continue.  The block exists to catch a
 *    mismatched TPL/SPL pair; a stage that quietly recovers from one has
 *    deleted the only evidence that there was a bug.
 *
 *  - It does not bring the DWC2 up.  TPL ran meson_usb_device_stop() before
 *    branching, and re-attaching a link across a stage boundary is the one
 *    thing this stack has broken itself on three times.  The report goes to
 *    SRAM (readable over the BootROM afterwards) and to the AO UART.
 *
 *    The single exception, and it is one register: on the way out this stage
 *    hands the board back to BL1 over SMC 0xC0000000 so the SRAM report is
 *    readable at all, and clears DCTL.SFTDISCON immediately before doing so.
 *    That is a register poke on a path that has already failed, not a driver
 *    and not a re-attach -- see dram_return_to_brom() below.
 *
 * Design: amlogic/software/boot/gxl/tpl-spl-chain-design.md.
 */

#include <spl.h>
#include <bootsource.h>
#include <cpu_func.h>
#include <asm/global_data.h>
#include <asm/arch/boot.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/arch/gx-bootlog.h>
#include <asm/arch/gx-handoff.h>
#include <usbflash-proto.h>

#ifdef MESON_GX_DRAM_STAGE

/*
 * Bare-metal AO UART TX, as spl.c and spl-gx.c each already carry: WFIFO
 * @0xc81004c0, STATUS @0xc81004cc, bit 21 = TX full.  A copy rather than a
 * shared symbol on purpose -- this stage links separately from the SRAM one and
 * must be able to say something before any driver, clock or DM exists, without
 * a dependency that could drag SRAM-stage code into a DRAM binary.
 */
#define AO_UART_WFIFO	0xc81004c0UL
#define AO_UART_STATUS	0xc81004ccUL
#define AO_UART_TX_FULL	BIT(21)

static void dram_putc(char c)
{
	int timeout = 100000;

	while ((readl((void *)AO_UART_STATUS) & AO_UART_TX_FULL) && --timeout)
		;
	if (timeout)
		writel(c, (void *)AO_UART_WFIFO);
	gx_bootlog_putc(c);
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

/*
 * Return-to-BootROM, so the verdict below is READABLE.
 *
 * Writing a code into SRAM and then spinning leaves the board inert: the
 * watchdog is stopped, TPL tore the DWC2 down, and nothing on the host can
 * reach those two words.  The vendor's own answer to this is not a reset --
 * meson_bl1_usb_return() (arch/arm/mach-meson/meson-usb.c:1769, modelled on
 * BL2's check_handler) builds an ep_info at the fixed SRAM address BL1 expects
 * and issues SMC 0xC0000000, the Amlogic run-image FID.  BL1 re-enters its own
 * USB download loop IN PLACE: no reset, DRAM stays trained, SRAM survives, and
 * the host talks to a normal 1b8e:c003 BROM again and reads the breadcrumbs
 * with REQ_READ_MEM.  Measured on AML-S905X-CC-V2 2026-08-05 from the
 * dram_init() failure path: four SRAM words came back over the BROM afterwards.
 *
 * This is a COPY of that function rather than a call to it, for the same
 * reason dram_putc() above is a copy: meson-usb.o is built only into the phase
 * that sets $(PHASE_)MESON_USB_BOOT, i.e. TPL.  Unlike dram_putc() this one
 * must carry the BL1 stepping table with it.
 */
#define BL1_HANDLER_TABLE		0xd9040000UL
#define BL1_HANDLER_SOC_ID		0xd904050cUL
#define BL1_ROM_OFF_CONT_DEFAULT	0x4518u
#define BL1_ROM_OFF_CONT_AFE		0x454cu
#define BL1_ROM_OFF_CONT_18CC		0x4e20u
#define BL1_ROM_OFF_CONT_0022		0x4640u
#define BL1_ROM_STEP_AFE		0x94000afeu
#define BL1_ROM_STEP_18CC		0x940018ccu
#define BL1_ROM_STEP_0022		0x52800022u
#define BL1_ROM_EP_SPSR			0x2cdu
#define BL1_SMC_RUN_IMAGE		0xc0000000UL
#define BL1_EP_INFO_ADDR		0xd9001880UL

/* 'OMOR' at the marker the host already looks for after a BROM return. */
#define U1_MARKER_ADDR			0xd900c100UL
#define U1_MARKER_VAL			0x524f4d4fu

/* Stage crumb if the SMC comes back, which it should not.  0xd900c120 is
 * spl.c's prod_stage word, deliberately NOT the report pair at 0xd900c170.
 */
#define PROD_STAGE_ADDR			0xd900c120UL
#define DRAM_STAGE_SMC_FAILED		0x5354e8d2u

/*
 * The one thing this stage touches in the DWC2, and it is one register.
 *
 * TPL's meson_usb_device_stop() ends by asserting DCTL.SFTDISCON
 * (meson-usb.c:2106) -- a deliberate, host-visible disconnect.  The proven
 * return-to-ROM path never ran that teardown, so BL1 was re-entered with the
 * pull-up still up.  Whether BL1's re-entered service re-asserts the pull-up
 * itself is UNKNOWN, and meson-usb.c:647-650 records that no disconnect-based
 * re-enumeration has ever reattached on this rig -- so leaving it asserted is
 * the failure mode that would make the whole channel silent.
 *
 * Clearing it here, immediately before the SMC, is chosen over the other
 * candidate fix (TPL skipping the assert under MESON_GX_XPL_CHAIN) because it
 * costs nothing on the success path and changes no behaviour TPL has already
 * been measured with: this code runs only when the stage is about to hand the
 * board back anyway.  It is stated rather than smuggled: yes, this breaches
 * "the DRAM stage does not touch the DWC2", for exactly one read-modify-write.
 *
 * NEITHER mitigation is verified.  Choosing correctly between them needs a
 * hardware run -- see the design doc, "The one real risk, and it is specific".
 */
#define DWC2_BASE			0xc9100000UL
#define DWC2_DCTL			0x804
#define DWC2_DCTL_SFTDISCON		BIT(1)

static void __maybe_unused dram_return_to_brom(void)
{
	u32 soc_id = readl((void *)BL1_HANDLER_SOC_ID);
	u32 off = BL1_ROM_OFF_CONT_DEFAULT;
	volatile u64 *ep = (volatile u64 *)BL1_EP_INFO_ADDR;
	u32 dctl;
	u64 entry;

	dctl = readl((void *)(DWC2_BASE + DWC2_DCTL));
	writel(dctl & ~DWC2_DCTL_SFTDISCON, (void *)(DWC2_BASE + DWC2_DCTL));

	writel(U1_MARKER_VAL, (void *)U1_MARKER_ADDR);

	if (soc_id == BL1_ROM_STEP_AFE)
		off = BL1_ROM_OFF_CONT_AFE;
	else if (soc_id == BL1_ROM_STEP_18CC)
		off = BL1_ROM_OFF_CONT_18CC;
	else if (soc_id == BL1_ROM_STEP_0022)
		off = BL1_ROM_OFF_CONT_0022;

	entry = BL1_HANDLER_TABLE | off;

	ep[0] = 0;			/* h0 + attrs (clear bit0) */
	ep[1] = entry;			/* pc */
	ep[2] = BL1_ROM_EP_SPSR;	/* spsr in low 32 */
	ep[3] = 1;			/* arg0 */

	asm volatile("dsb sy\nisb" ::: "memory");

	asm volatile(
		"mov x0, %0\n"
		"mov x1, %1\n"
		"mov x2, xzr\n"
		"mov x3, xzr\n"
		"mov x4, xzr\n"
		"mov x5, xzr\n"
		"mov x6, xzr\n"
		"mov x7, xzr\n"
		"smc #0\n"
		:
		: "r"((u64)BL1_SMC_RUN_IMAGE), "r"((u64)BL1_EP_INFO_ADDR)
		: "x0", "x1", "x2", "x3", "x4", "x5", "x6", "x7",
		  "x8", "x9", "x10", "x11", "x12", "x13", "x14", "x15",
		  "x16", "x17", "memory"
	);

	/* BL1 rejected the handoff.  Say so somewhere that is not the report. */
	writel(DRAM_STAGE_SMC_FAILED, (void *)PROD_STAGE_ADDR);
}

/**
 * dram_stage_note() - leave the verdict where a host can find it.
 * @code:	GX_DRAM_STAGE_*
 * @fault:	the value that failed the check, or 0 when nothing did
 *
 * SRAM, not DRAM: the BootROM can read SRAM after this stage is gone, and DDR
 * is the very thing under suspicion when the block is wrong.  The progress-word
 * bits go in the LOW half of 0xd900c104 because that is where GX_XPL_PROG_* are
 * allocated -- NOT because the high half is unusable here.  This stage runs long
 * after the DWC2 session ends, and ep0_tx() only writes bits 31:16 from its
 * slave-PIO path, so nothing can clobber a flag set at this point.  What the
 * high half does still hold is a stale GNPTXSTS snapshot, which is why a flag
 * OR'd up there would be unreadable rather than overwritten.  See the
 * progress-word note in gx-handoff.h.
 *
 * RETURNS, deliberately.  A note is not a verdict about whether to continue:
 * the accept path writes one and carries on into the FIT, and the refuse path
 * writes one and then stops (dram_stage_refuse() below).  Only the low-half
 * progress bit distinguishes them, and it is derived from @code rather than
 * passed, so the two can never disagree.
 */
static void dram_stage_note(u32 code, u32 fault)
{
	u32 prog = (code == GX_DRAM_STAGE_OK || code == GX_DRAM_STAGE_FIT) ?
		   GX_XPL_PROG_DRAM_OK : GX_XPL_PROG_DRAM_BAD;

	writel(fault, (void *)GX_DRAM_FAULT_ADDR);
	writel(GX_DRAM_STAGE_MAGIC | code, (void *)GX_DRAM_STAGE_ADDR);
	writel(readl((void *)GX_XPL_PROG_ADDR) | prog,
	       (void *)GX_XPL_PROG_ADDR);
	asm volatile("dsb sy" ::: "memory");

	/* The handoff's marks are diagnostics (MESON_GX_XPL_VERBOSE); a refusal always prints. */
	if (code == GX_DRAM_STAGE_OK) {
		if (IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
			dram_puts("S2 OK\n");
	} else if (code == GX_DRAM_STAGE_FIT) {
		if (IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
			dram_puts("S2 FIT\n");
	} else {
		dram_puts("S2 REFUSE ");
		dram_puthex(code);
		dram_puts(" got ");
		dram_puthex(fault);
		dram_puts("\n");
	}
}

/**
 * dram_stage_refuse() - record why the handoff was rejected, then stop.
 *
 * Does not return, and must not: continuing past a rejected handoff is exactly
 * the behaviour the block exists to prevent.  The note is written and printed
 * first, then the EE watchdog resets the board into the BootROM.
 */
/*
 * The D-cache (and so the MMU) for this stage's loading.  Without it every
 * data access the FIT load makes -- the copy out of NOR or RAM, the SHA-256
 * of each image, LZMA's decode of U-Boot -- goes to DRAM uncached: on
 * AML-S805X-AC-V2 the decode alone took 2.44 s of the BootROM watchdog's
 * 5000 ms.  The page tables are gx_mem_map's (board-gx.c: DRAM normal, MMIO
 * device), at GX_SPL_TLB_ADDR: DRAM between this stage's stack top
 * (CONFIG_SPL_STACK, 0x09000000) and its malloc arena (0x0a000000), which
 * nothing else in this stage or the next uses.  bl31_entry() (spl_atf.c)
 * cleans and disables the cache before BL31; the one reader past the cache
 * before that, the SC-ROM's fetch of the SC image, is cleaned for in
 * spl-gxl-scp.c.
 */
#define GX_SPL_TLB_ADDR		0x09800000UL
#define GX_SPL_TLB_SIZE		0x00080000UL

DECLARE_GLOBAL_DATA_PTR;

static void gx_dram_stage_dcache_enable(void)
{
	if (CONFIG_IS_ENABLED(SYS_DCACHE_OFF))
		return;

	gd->arch.tlb_addr = GX_SPL_TLB_ADDR;
	gd->arch.tlb_size = GX_SPL_TLB_SIZE;
	dcache_enable();
}

static void __noreturn dram_stage_refuse(u32 code, u32 fault)
{
	void meson_gx_wdt_arm_ms(u32 ms);

	dram_stage_note(code, fault);

	/*
	 * Stop with the EE watchdog armed, so the board resets into a fresh
	 * BootROM a host can stage again.  The report is the console line
	 * printed above.
	 *
	 * This used to hand USB back to BL1 (dram_return_to_brom()) so the
	 * host could read the two SRAM words, with the watchdog stopped by the
	 * TPL.  The re-entered BL1 answers no EP0 request on either GXL board
	 * (v1: N42; and the TPL's own hand-back in the GXL DDR3 FMA round F1
	 * FA3), so that left a board only a power cycle recovered.  The TPL's
	 * DRAM-init failure stop made the same change in round F3, and the
	 * stager now stops polling a board that resets (meson-usb-boot.py
	 * BOARD RESET): claude amlogic/software/boot/gxl/audit/
	 * 2026-10-19-gxl-ddr3-boot-chain-failure-modes.md.  G12A/SM1's
	 * terminal stop already ends the same way (spl.c).
	 *
	 * The TPL already ran meson_usb_device_stop() before its jump, so the
	 * DWC2 is disconnected; 2 s leaves the UART time to drain the note.
	 */
	meson_gx_wdt_arm_ms(2000);
	for (;;)
		;
}

/**
 * board_init_f() - the DRAM stage's entire job in T2.
 *
 * Overrides the SRAM stage's board_init_f, which is compiled out of this phase
 * (see spl.c).  Checks are ordered cheapest-and-most-fundamental first, so the
 * reported code names the first thing that is actually wrong rather than a
 * downstream symptom of it.
 */
void board_init_f(ulong dummy)
{
	struct gx_xpl_handoff *h = (struct gx_xpl_handoff *)GX_HANDOFF_ADDR;
	u32 magic, version, flags, size_mb, spl_addr;

	/* Before anything delays: CNTFRQ_EL0 reads 0 here too (spl.c). */
	meson_timer_rate_init();
	writel(GX_DRAM_STAGE_MAGIC | GX_DRAM_STAGE_ENTRY,
	       (void *)GX_DRAM_STAGE_ADDR);
	writel(readl((void *)GX_XPL_PROG_ADDR) | GX_XPL_PROG_DRAM_ENTRY,
	       (void *)GX_XPL_PROG_ADDR);
	if (IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
		dram_puts("S2\n");

	/*
	 * Magic first: TPL writes it LAST, behind a barrier, so finding it means
	 * the fifteen words before it are final.  Any other value -- zero, a
	 * torn block, whatever the last boot left -- is not a handoff.
	 */
	magic = readl(&h->magic);
	if (magic != GX_HANDOFF_MAGIC)
		dram_stage_refuse(GX_DRAM_STAGE_BAD_MAGIC, magic);

	/*
	 * Version: the block has no size word, so a different version is a
	 * different layout at, potentially, a different address.  There is
	 * nothing to interpret and no compatible subset to fall back on.
	 */
	version = readl(&h->version);
	if (version != GX_HANDOFF_VERSION)
		dram_stage_refuse(GX_DRAM_STAGE_BAD_VER, version);

	/* TPL says it trained DDR.  This stage is executing out of it. */
	flags = readl(&h->flags);
	if (!(flags & GX_HANDOFF_F_DDR_OK))
		dram_stage_refuse(GX_DRAM_STAGE_BAD_DDR, flags);

	/*
	 * The size TPL measured, taken on trust and range-checked -- not
	 * re-probed.  Re-probing would make the handoff decorative and would
	 * hide precisely the failure it is here to surface.
	 */
	size_mb = readl(&h->dram_size_mb);
	if (size_mb < GX_HANDOFF_DRAM_MIN_MB || size_mb > GX_HANDOFF_DRAM_MAX_MB)
		dram_stage_refuse(GX_DRAM_STAGE_BAD_SIZE, size_mb);

	/*
	 * And that TPL branched where this image was linked.  A TPL and an SPL
	 * built with different addresses is the mismatch that would otherwise
	 * present as a dead board with no output at all.
	 */
	spl_addr = readl(&h->spl_addr);
	if (!(flags & GX_HANDOFF_F_SPL) || spl_addr != CONFIG_SPL_TEXT_BASE)
		dram_stage_refuse(GX_DRAM_STAGE_BAD_ADDR, spl_addr);

	if (IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE)) {
		dram_puts("S2 dram ");
		dram_puthex(size_mb);
		dram_puts(" MB @ ");
		dram_puthex(spl_addr);
		dram_puts("\n");
	}

	dram_stage_note(GX_DRAM_STAGE_OK, 0);

	/*
	 * The handoff is accepted, and this is where T2 used to stop.
	 *
	 * What follows is not a second FIT loader.  It is the TAIL OF THE
	 * SINGLE-STAGE SPL's board_init_f() (spl.c, below the
	 * CONFIG_MESON_GX_XPL_CHAIN block) with one deviation, noted at it --
	 * and then a plain return, which is the whole mechanism.  crt0_64.S
	 * calls board_init_r() after
	 * board_init_f() whenever CONFIG_IS_ENABLED(FRAMEWORK), and
	 * common/spl/spl.c's board_init_r() is what parses the FIT and invokes
	 * BL31: spl_boot_device() -> boot_from_devices() -> spl_ram_load_image()
	 * (BOOT_DEVICE_RAM, FIT at CONFIG_SPL_LOAD_FIT_ADDRESS -- 0x02000000 on
	 * libretech-cc_libre_defconfig, 0x0c000000 on the v2 and ac boards --
	 * where TPL's USB session left it) or spl_mmc_load_image()
	 * (SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR 0x100 = 0x20000 = the binman FIT
	 * offset 0x1fe00 plus the 0x200 image offset) -> spl_load_simple_fit()
	 * -> jump_to_image() -> spl_invoke_atf().
	 *
	 * So the DRAM stage reaches the FIT by NOT overriding the generic flow
	 * any further, rather than by gaining a loader of its own.  That is
	 * deliberate: a second implementation is how the chain build and the
	 * shipping single-stage build would silently drift apart, and the two
	 * are supposed to reach BL31 the same way from the same sources.  The
	 * only thing this file overrides is board_init_f -- the part that is
	 * genuinely different, because DDR and the DWC2 are already done.
	 *
	 * Note what is NOT needed here: h->spl_len.  It describes THIS image,
	 * which is already running; nothing on the path above consumes it.  The
	 * FIT is found by address, not by walking past the SPL.  So parsing the
	 * FIT does not force the "should the raw SPL stage carry a length
	 * header" question either way, and none is added -- see the design doc's
	 * open item 2.
	 */
	if (CONFIG_IS_ENABLED(OF_CONTROL)) {
		int ret = spl_early_init();

		/*
		 * REFUSE rather than hang.  The single stage's equivalent is a
		 * bare while(1) with a debug() nobody can see; here the report
		 * channel exists and the errno is worth having, because "no
		 * FDT" and "malloc too small" are different bugs with the same
		 * symptom.
		 */
		if (ret)
			dram_stage_refuse(GX_DRAM_STAGE_NO_DM, (u32)ret);
	}

	spl_init();
	icache_enable();
	gx_dram_stage_dcache_enable();
	preloader_console_init();

	/* First console line after the banner: where the BootROM found us. */
	if (CONFIG_IS_ENABLED(BOOTSOURCE))
		bootsource_announce(meson_bootsource());

	/*
	 * Last word this file writes.  Past this point the board is executing
	 * common/spl/ code that knows nothing about the handoff block, so a
	 * board found with 0x82 in the report word died in the shared loader --
	 * a FIT that did not parse, a BL31 that did not run -- not in the seam.
	 */
	dram_stage_note(GX_DRAM_STAGE_FIT, 0);
}

/*
 * spl_perform_board_fixups() - the USB flash session's mark for U-Boot proper.
 *
 * When the session ran (spl-usbflash.c), the control DTB the FIT loader placed
 * after U-Boot proper gets /chosen/u-boot,spl-usbflash, so proper runs no
 * second session.  Same hook as the G12A/SM1 DRAM stage (spl-dram-stage.c).
 */
void spl_perform_board_fixups(struct spl_image_info *spl_image)
{
	if (IS_ENABLED(CONFIG_SPL_USBFLASH))
		spl_usbflash_fdt_fixup(spl_image_fdt_addr(spl_image));
}

#endif /* MESON_GX_DRAM_STAGE */
