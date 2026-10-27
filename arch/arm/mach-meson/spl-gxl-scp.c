// SPDX-License-Identifier: GPL-2.0+
/*
 * GXL DRAM SPL: open SC load via SC-ROM send=0 of one FIT "scp".
 *
 * RE (aml_scpi.c / vendor BL31): AP cannot write M3 TCM. SC-ROM is
 * already up and DMA-fetches a DRAM pointer (send=0: 0xd3/0xd2 + SHA
 * 0xd0/0xd1/0xd5). After 0xd5 it replaces itself with that image.
 * Ready bits [21:20] are painted by the *payload* (G12 open firmware
 * does; vendor BL30 does), not by SC-ROM.
 *
 * Open product: one amlogic-gxl.bin at LAYOUT 0x001C0000, size the FIT
 * node (≤ 0xC000, the measured SC-ROM window). Packing matches TF-A aml_scpi_cmd(cmd, size):
 * SET_FW_ADDR size=8 → 0x800000d3; FW_SIZE size=firmware_len (not 4).
 * Two-window BL30 0xa000 + BL301 COPY_FW is vendor packaging, not this.
 *
 * Not: --bl30sig, bl30_new.bin, TCM dest, AP 0x10000000, 0x013C0000,
 * G12 0xFF…, GXBB send_scp, TF-A upload_scp_fw.
 *
 * CURATED for the production branch (N54, 2026-09-03).  Two bring-up
 * instruments that lived here on v2026.07/lc-gxl-spl-scrom-fetch are NOT
 * carried over, and neither are the Kconfig symbols that gated them
 * (MESON_GXL_SCP_DIAG_MBOX_POISON, MESON_GXL_SCP_DIAG_EL3_SCMI):
 *
 *   - the OSPM mailbox poison, which MUTATES the exact bytes an SCMI channel
 *     header occupies.  Its result is banked (the M3 reaches transport_start()
 *     and decodes the non-secure alias, 2026-08-26) and the non-secure
 *     aperture is now open, so a real agent drives this channel;
 *   - the EL3 SCMI probe, which still carried the INERT CNTFRQ_EL0-based
 *     deadline N55 removed from every wait below (CNTFRQ_EL0 reads 0 in this
 *     stage, so its real bound was 40,000,000 iterations -- the shape that
 *     reset the board before BL31).
 *
 * Both remain on that topic branch for anyone who needs to re-take the
 * measurement.  What is left here MUTATES nothing the SC owns: the loader
 * plus read-only reporting.
 *
 * G5/D3 (N17 round) adds ONE decision to this flow: scp_board_pairing_check()
 * reads the staged slice's trailer word and refuses a payload compiled for a
 * different board BEFORE the first mailbox send. See that function.
 */

#include <cpu_func.h>
#include <image.h>
#include <spl.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <u-boot/sha256.h>

#if CONFIG_IS_ENABLED(FIT_IMAGE_POST_PROCESS)

/*
 * 🟢 MEASURED 2026-08-29 -- this cap IS the hardware limit.  The SC ROM
 * accepts a single window up to 0xC000 (48 KiB) and REFUSES 0xC100 and above
 * with "sha fail.", never starting the SC (ten boots, bisected to 256 B).
 * Method + full sweep:
 * amlogic/software/boot/gxl/audit/2026-08-29-gxl-rom-copy-ceiling.md
 *
 * This is the ENFORCED twin of scp-firmware's GXL_SCP_CODE_SRAM_SIZE
 * (product/amlogic/gxl/include/gxl_mmap.h), which moved from 0x8400 to
 * 0xC000 in the same change as its link region SCP_RAM_SIZE (0xA000 ->
 * 0xE000).  Never raise one without the other: an image the ROM copies past
 * the SC's link region is erased by the SC's own .bss zeroing and stack.
 */
/*
 * The upload's detail -- budgets, fetches, digests, mailbox words -- is a
 * bring-up diagnostic (MESON_GX_XPL_VERBOSE; board+bootloader policy "Boot
 * console output"). A good boot says "SCP: started"; a refusal or a stall
 * always prints in full.
 */
#define scp_vprintf(...)						\
	do {								\
		if (IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))		\
			printf(__VA_ARGS__);				\
	} while (0)

#define GXL_SCP_CODE_SIZE		0x0000C000UL
/* LAYOUT FIT "scp" staging (AML_SEC_SCP_STAGE_BASE). Not 0x013C0000. */
#define GXL_SCP_STAGE_BASE		0x001C0000UL

#define GXL_HIU_MAILBOX_SET_3		0xC883C428UL
#define GXL_HIU_MAILBOX_STAT_3		0xC883C42CUL
#define GXL_HIU_MAILBOX_STAT_0		0xC883C408UL
#define GXL_HIU_MAILBOX_CLR_0		0xC883C40CUL
#define GXL_HIU_MAILBOX_CLR_3		0xC883C430UL

#define GXL_MHU_AP_TO_SCP_PAYLOAD	0xD9013A00UL
#define GXL_MHU_SCP_TO_AP_PAYLOAD	0xD9013800UL

#define GXL_AO_RTI_SCP_STAT		0xC810023CUL

/*
 * The BootROM's always-on watchdog -- the deadline this whole file runs
 * against, and now the clock its waits are bounded on.
 *
 * A ROM stage arms it with a 5000 ms period before our first instruction and
 * hands the board over with it counting; nothing of ours arms it, and nothing
 * pets it until U-Boot proper's misc_init_r (CONFIG_MESON_GXL_AO_WDT_HANDOFF).
 * Register layout is the kernel's (drivers/watchdog/meson_gxbb_wdt.c): base
 * 0xC8100120, CTRL +0x0, TCNT +0x8, RESET +0xc.  TCNT[31:16] is the ELAPSED
 * count and TCNT[15:0] the period, both in milliseconds -- CTRL reads
 * 0x03245DBF here, i.e. EN | CLK_EN | CLKDIV_EN with divider 23999 off the
 * 24 MHz crystal, a 1 kHz tick.
 *
 * 🔴 0xC810012C must NEVER be READ (a read stalls both masters -- see
 * amlogic/software/boot/gxl/design/open-sc-architecture.md section 5), and
 * this file never writes it either: bounding a wait is this stage's job,
 * petting is not.  0xC8100128 is a DIFFERENT register, and reading it is
 * exactly what the kernel's own .get_timeleft does on this same block every
 * time userspace asks.
 */
#define GXL_AO_WDT_CTRL			0xC8100120UL
#define GXL_AO_WDT_CTRL_EN		(1U << 18)
#define GXL_AO_WDT_TCNT			0xC8100128UL
#define GXL_AO_WDT_TCNT_CNT_SHIFT	16
#define GXL_AO_WDT_TCNT_SETUP_MASK	0xFFFFU

static u32 ao_wdt_elapsed_ms(void)
{
	return readl((void *)GXL_AO_WDT_TCNT) >> GXL_AO_WDT_TCNT_CNT_SHIFT;
}

static u32 ao_wdt_period_ms(void)
{
	return readl((void *)GXL_AO_WDT_TCNT) & GXL_AO_WDT_TCNT_SETUP_MASK;
}

/*
 * Is there a deadline to protect at all?  If the block is not enabled, or its
 * period reads 0, nothing will reset the board and a caller keeps whatever
 * bound it had.  Fail-safe in both directions: no watchdog, no clamp.
 */
static bool ao_wdt_armed(void)
{
	return (readl((void *)GXL_AO_WDT_CTRL) & GXL_AO_WDT_CTRL_EN) &&
	       ao_wdt_period_ms() != 0;
}

/*
 * Has `limit` ms elapsed since the counter read `t0`?  A counter that has gone
 * BACKWARDS means someone pet the watchdog under us; there is no such petter
 * before U-Boot proper, but treat it as "not expired" rather than wrapping a
 * u32 subtraction into a huge value that would end every wait instantly.
 */
static bool ao_wdt_elapsed_since(u32 t0, u32 limit)
{
	u32 now = ao_wdt_elapsed_ms();

	return now >= t0 && (now - t0) >= limit;
}

/*
 * NS alias of the OSPM SCMI mailbox that the open SC's transport module
 * initialises: scp-firmware product/amlogic/gxl/scp_ramfw/config_transport.c
 * gives the OSPM channel out_band_mailbox_address = GXL_SCMI_PAYLOAD_NS_HP
 * (gxl_mmap.h, 0xC8000000 + 0x13400).  Secure 0xD9000000 and NS 0xC8000000
 * are two aliases of ONE 80 KiB AHB SRAM -- TF-A plat/amlogic/gxl/gxl_def.h
 * :132-133 (AML_TZRAM_BASE / AML_TZRAM_SIZE 0xD9000000 / 0x14000) and Linux
 * arch/arm64/boot/dts/amlogic/meson-gx.dtsi:490-492 (sram@c8000000, reg
 * 0xC8000000 / 0x14000) -- so offset 0x13400 is one physical location either
 * way, and this stage reads it at the non-secure spelling because that is the
 * one the SC's own transport module is configured with.
 */
#define GXL_SCMI_PAYLOAD_NS_HP		0xC8013400UL

#define SCPI_CMD_FW_HASH_0		0xd0
#define SCPI_CMD_FW_HASH_1		0xd1
#define SCPI_CMD_FW_SIZE		0xd2
#define SCPI_CMD_SET_FW_ADDR		0xd3
#define SCPI_CMD_FW_HASH_VERIFY		0xd5
#define SCPI_SIZE_SHIFT			20
#define SCPI_CMD(cmd, sz)		((u32)(cmd) | ((u32)(sz) << SCPI_SIZE_SHIFT))

static u64 arch_cnt(void)
{
	u64 v;

	asm volatile("mrs %0, cntpct_el0" : "=r" (v));
	return v;
}

static bool scp_running(void)
{
	return ((readl((void *)GXL_AO_RTI_SCP_STAT) >> 20) & 0x3U) == 0x3U;
}

static int mhu_wait_stat3_clear(void)
{
	int t;

	for (t = 1000000; t > 0; t--) {
		if (!readl((void *)GXL_HIU_MAILBOX_STAT_3))
			return 0;
	}
	return -ETIMEDOUT;
}

static int mhu_wait_stat0(void)
{
	int t;

	for (t = 1000000; t > 0; t--) {
		if (readl((void *)GXL_HIU_MAILBOX_STAT_0))
			return 0;
	}
	return -ETIMEDOUT;
}

static int scp_send_cmd(u32 cmd)
{
	int ret;

	ret = mhu_wait_stat3_clear();
	if (ret)
		goto fail;
	writel(cmd, (void *)GXL_HIU_MAILBOX_SET_3);
	ret = mhu_wait_stat3_clear();
	if (ret)
		goto fail;
	return 0;
fail:
	printf("SCP: cmd 0x%x not taken (mb3 0x%x)\n",
	       cmd, readl((void *)GXL_HIU_MAILBOX_STAT_3));
	return ret;
}

static int scp_send_and_wait(u32 cmd)
{
	int ret;

	ret = scp_send_cmd(cmd);
	if (ret)
		return ret;
	ret = mhu_wait_stat0();
	if (ret) {
		printf("SCP: cmd 0x%x no rsp (mb0 0x%x rsp 0x%x)\n",
		       cmd, readl((void *)GXL_HIU_MAILBOX_STAT_0),
		       readl((void *)GXL_MHU_SCP_TO_AP_PAYLOAD));
	}
	return ret;
}

static void scp_mhu_clear(void)
{
	writel(0xffffffff, (void *)GXL_HIU_MAILBOX_CLR_3);
	writel(0xffffffff, (void *)GXL_HIU_MAILBOX_CLR_0);
}

static void scp_print_sha(const char *tag, uintptr_t addr, size_t size,
			  const u8 *digest)
{
	int i;

	if (!IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
		return;
	printf("SCP: sha256 %s 0x%x 0x%x ", tag, (u32)addr, (u32)size);
	for (i = 0; i < SHA256_SUM_LEN; i++)
		printf("%02x", digest[i]);
	printf("\n");
}

static int scp_send_hash(const void *data, size_t size, const char *tag,
			 uintptr_t addr)
{
	u8 digest[SHA256_SUM_LEN];

	sha256_csum_wd(data, size, digest, CHUNKSZ_SHA256);
	scp_print_sha(tag, addr, size, digest);
	memcpy((void *)GXL_MHU_AP_TO_SCP_PAYLOAD, digest, SHA256_SUM_LEN);
	if (scp_send_cmd(SCPI_CMD_FW_HASH_0))
		return -ETIMEDOUT;
	if (scp_send_cmd(SCPI_CMD_FW_HASH_1))
		return -ETIMEDOUT;
	if (scp_send_cmd(SCPI_CMD_FW_HASH_VERIFY))
		return -ETIMEDOUT;
	writel(0xffffffff, (void *)GXL_HIU_MAILBOX_CLR_0);
	return 0;
}

/* send=0: DRAM pointer, SC-ROM DMA of one open image. */
static int scp_fetch_window(uintptr_t addr, size_t size, const char *tag)
{
	scp_vprintf("SCP: fetch %s 0x%x B from 0x%x src0=0x%x\n",
	       tag, (u32)size, (u32)addr, readl((void *)addr));

	scp_mhu_clear();
	asm volatile("dsb sy" ::: "memory");

	writel((u32)addr, (void *)GXL_MHU_AP_TO_SCP_PAYLOAD);
	writel((u32)(addr >> 32), (void *)(GXL_MHU_AP_TO_SCP_PAYLOAD + 4));
	if (scp_send_and_wait(SCPI_CMD(SCPI_CMD_SET_FW_ADDR, 8)))
		return -ETIMEDOUT;

	/* TF-A aml_scpi_set_scp_addr: mailbox = len, cmd = FW_SIZE|len<<20 */
	writel((u32)size, (void *)GXL_MHU_AP_TO_SCP_PAYLOAD);
	if (scp_send_and_wait(SCPI_CMD(SCPI_CMD_FW_SIZE, size)))
		return -ETIMEDOUT;

	return scp_send_hash((const void *)addr, size, tag, addr);
}

/*
 * How long a HEALTHY payload is given to paint the ready bits.  Measured
 * 2026-08-30 over six boots of the shipping 34 412 B payload
 * (scp-firmware 56269baa): ready after 75 or 76 poll iterations, 6268-6319
 * CNTPCT_EL0 ticks, and 0 ms on the watchdog's own 1 kHz tick EVERY time -- so
 * on the clock that matters here the healthy case does not consume a single
 * unit of the budget, and 250 ms is beyond generous.  It is also the figure
 * this file already uses for GXL_SCP_MBOX_PROBE_MS.
 *
 * ⚠️ Do NOT convert those CNTPCT counts to a duration.  CNTFRQ_EL0 reads 0
 * here, and calibrating CNTPCT against the watchdog's milliseconds in one boot
 * gives ~599 MHz over the 250 ms mailbox poll and ~617 MHz over the 533 ms
 * stage -- neither the 24 MHz crystal nor a single self-consistent rate.  The
 * counts are comparable with each other and with nothing else, which is
 * precisely why no bound in this file is built on them.
 */
#define GXL_SCP_READY_TIMEOUT_MS	250

/*
 * What must be left of the watchdog's budget when this stage returns, for BL31
 * and U-Boot proper to reach misc_init_r's first pet.  Measured 2026-08-30 on
 * the same boots: this function is entered at 0 ms of 5000, the whole SC upload
 * leaves at 32 ms, BL31 is running by 197 ms, and the first pet lands at
 * 2894 ms -- so BL31 plus U-Boot proper need ~2.9 s of the 5 s.  3500 ms is
 * that with ~600 ms of headroom.
 */
#define GXL_SCP_WDT_RESERVE_MS		3500

/*
 * ...and the floor the reserve may not clamp below.  A payload needs well under
 * a millisecond; a boot that arrives here so late that the reserve leaves it
 * nothing is not saved by refusing to look, and skipping the wait entirely
 * would break a HEALTHY SC on any medium slower than this rig's USB.
 */
#define GXL_SCP_READY_FLOOR_MS		20

/*
 * Wait for the payload to paint the ready bits -- bounded against the ROM's
 * watchdog, and reporting rather than resetting when it does not.
 *
 * 🔴 THIS BOUND USED TO BE INERT, AND THAT COST THE BOARD.  The loop carried a
 * `frq * 3` deadline off CNTFRQ_EL0, but CNTFRQ_EL0 reads 0 in this stage
 * (measured 2026-08-26), so `if (frq && ...)` never fired and the real bound
 * was the ITERATION COUNT -- about 3 s of spinning.  Measured 2026-08-30, a
 * payload that never becomes ready spends all of it and the board RESETS
 * BEFORE BL31 IS ENTERED (OTG identify 02 04 00 00, the BootROM answering), so
 * a bad or truncated SC image presented as a reset loop with no diagnostic at
 * all -- and the DT fixup's own SC guard, which is what makes a mute SC a
 * degraded boot rather than a crash, never got to run.
 *
 * The fix is to bound the wait on a clock that exists here.  CNTPCT_EL0 does
 * count (it is still read below, to REPORT the wait), but only the AO
 * watchdog's counter is denominated in the same units as the deadline being
 * protected, so no frequency constant and no conversion enters the decision.
 *
 * The bound is budget-relative on purpose: min(TIMEOUT, remaining - RESERVE),
 * floored.  However long the stages before this one took -- and USB, MMC and
 * SPI NOR do not take the same time to get here -- the wait cannot be the
 * thing that pushes the board past the deadline.
 */
static void scp_wait_ready(size_t size, u32 stat_found)
{
	u32 t0 = ao_wdt_elapsed_ms();
	u32 period = ao_wdt_period_ms();
	u32 limit = 0;
	u64 c0 = arch_cnt();
	u32 stat, mb0, rsp;
	int t;

	if (ao_wdt_armed()) {
		u32 left = (period > t0) ? period - t0 : 0;

		limit = (left > GXL_SCP_WDT_RESERVE_MS)
			? left - GXL_SCP_WDT_RESERVE_MS : 0;
		if (limit > GXL_SCP_READY_TIMEOUT_MS)
			limit = GXL_SCP_READY_TIMEOUT_MS;
		if (limit < GXL_SCP_READY_FLOOR_MS)
			limit = GXL_SCP_READY_FLOOR_MS;
	}

	/* The iteration count stays as a backstop for an unarmed watchdog. */
	for (t = 30000000; t > 0; t--) {
		if (scp_running()) {
			if (IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
				printf("SCP: started, 0x%x B (stat 0x%x) after %u cnt, at %u ms of %u\n",
				       (u32)size,
				       readl((void *)GXL_AO_RTI_SCP_STAT),
				       (u32)(arch_cnt() - c0),
				       ao_wdt_elapsed_ms(), period);
			else
				printf("SCP: started\n");
			return;
		}
		if (limit && ao_wdt_elapsed_since(t0, limit))
			break;
	}

	/*
	 * 🔴 REPORT THE READINGS AND WHAT THEY DISCRIMINATE -- NEVER A VERDICT
	 * THE FIELDS CANNOT CARRY.
	 *
	 * This used to print two conclusions it had no reading for -- "the SC
	 * did not start" and "SCMI will be unavailable" -- on boots where the SC
	 * was demonstrably alive.  Measured 2026-09-21 on AML-S905X-CC v1
	 * (rig nyc-a), verbatim, one boot:
	 *
	 *   SCP: the SC did not start -- ready bits never painted after 250 ms (limit 250 ms, wdt 250 of 5000); continuing without the SC, SCMI will be unavailable
	 *   SCP: never became ready (stat 0x3a00000 mb0 0x0 rsp 0x8658bdd4)
	 *   SCP: ospm 00000000 00000001 00000000 00000000
	 *   ...
	 *   NOTICE:  BL31: SCMI GET Base ver rc=0 ver=0x20000 spins=110; proto=0x10
	 *
	 * so both sentences were false on that boot: the SC had already run
	 * transport_mailbox_init() (ospm[1] = 1) and BL31 then talked SCMI to
	 * it.  The ready bits are painted by the PAYLOAD (this file's header
	 * says so), never by the SC-ROM, and the shipped payload's
	 * gxl_paint_scp_ready() is not in the image at all -- the sole access to
	 * 0xC810023C is the retraction (whole-image disassembly,
	 * amlogic/software/boot/gxl/audit/
	 * 2026-09-21-instruments-that-speak-when-they-cannot-see.md §5).  On
	 * such a payload this register can only ever read as-found no matter how
	 * healthy the SC is.
	 *
	 * What THIS stage can honestly discriminate:
	 *
	 *   stat == as-found     nobody ever painted the bits.  Says NOTHING
	 *                        about whether the SC is running
	 *   stat moved, not 3    a partial or retracted paint
	 *
	 * and the "ROM still waiting for firmware" state -- the third one the
	 * G12A twin reads off its AO handshake register -- is already excluded
	 * by CONTROL FLOW here, not by a register: reaching this function means
	 * scp_fetch_window() returned 0, i.e. the ROM consumed every command
	 * (mb3 cleared after each) and answered 0xd3 and 0xd2 (mb0 set).  🔴 Do
	 * NOT "improve" this by adding a read of GXL's spelling of G12A's
	 * 0xFF800200 handshake: no measurement on this chain says such a
	 * register decodes here, and an undecoded AO read STALLS this core
	 * forever (scp_probe_sram_security() above, three boots).
	 *
	 * The 250 ms bound is NOT the defect -- measured `limit 250 ms, wdt 250
	 * of 5000`, i.e. the full nominal bound spent with 4.7 s of watchdog
	 * left -- and is deliberately unchanged.  Every register is sampled ONCE
	 * into a local so the two lines describe one sample.
	 */
	stat = readl((void *)GXL_AO_RTI_SCP_STAT);
	mb0 = readl((void *)GXL_HIU_MAILBOX_STAT_0);
	rsp = readl((void *)GXL_MHU_SCP_TO_AP_PAYLOAD);

	printf("SCP: ready bits %d of stat 0x%x (want 3, as-found 0x%x) mb0 0x%x rsp 0x%x after %u ms (limit %u ms, wdt %u of %u)\n",
	       (stat >> 20) & 0x3U, stat, stat_found, mb0, rsp,
	       ao_wdt_elapsed_ms() - t0, limit, ao_wdt_elapsed_ms(), period);

	if (stat == stat_found)
		printf("SCP: the ROM consumed and answered every fetch command, and the ready bits are UNCHANGED from as-found -- nobody painted them%s. The paint is the payload's job, so this line says NOTHING about whether the SC is running; confirm with SCMI, not with this bit\n",
		       mb0 ? " (an SC->AP doorbell was raised)"
		           : " (and no SC->AP doorbell was raised either, so nothing here witnesses the SC)");
	else
		printf("SCP: the ready bits MOVED (as-found 0x%x) but never reached 3 -- a partial or retracted paint; confirm with SCMI, not with this bit\n",
		       stat_found);
}

/*
 * Did the SC actually write its OSPM mailbox, and on which alias?
 *
 * This has to happen HERE, in the SPL, and not from U-Boot proper.  Measured
 * 2026-08-26: a non-secure `md.l 0xc8013400` in U-Boot proper takes a
 * synchronous external abort (esr 0x96000210, DFSC 0x10 -- external abort, not
 * on a translation-table walk) with far = 0xc8013400, whose handler then
 * PSCI_SYSTEM_RESETs and panics BL31.  The whole 80 KiB AHB share SRAM is
 * secure on this chain, so EL3 is the only exec state that can read it.
 *
 * transport_mailbox_init() (scp-firmware module/transport/src/mod_transport.c
 * :806-810) runs from transport_start(), i.e. AFTER the ready bits are painted
 * in hardware_init_hook(), so a read taken the instant scp_wait_ready() returns
 * is too early.  Poll, bounded, for the pattern it writes -- word 1 = 1, every
 * other word 0 -- and then state both windows verbatim either way.
 *
 * The PSCI channel is the control: it is the same module, the same policy and
 * the same code path, but its mailbox is the SECURE spelling 0xD9013A00
 * (config_transport.c).  If PSCI reads initialised while OSPM does not, the M3
 * reached transport_start() and could not write 0xC8...; if neither is
 * initialised, it never got there.
 */
#define GXL_SCP_MBOX_PROBE_MS		250

static void scp_probe_mailboxes(void)
{
	u32 ospm[8], psci[8];
	u32 w0 = ao_wdt_elapsed_ms();
	u64 c0 = arch_cnt();
	u32 ms;
	int i, spins;

	/*
	 * 🔴 THE SAME INERT BOUND scp_wait_ready() carried, and on the failure
	 * path it is the LARGER of the two.  CNTFRQ_EL0 reads 0 in this stage
	 * (measured 2026-08-26: both poll loops here reported "0 ms" on a boot
	 * where the work plainly took longer), so the `frq / 4` deadline never
	 * fired and 40 000 000 iterations were the real bound -- several
	 * seconds, spent precisely when the SC has NOT written this window,
	 * i.e. on the boot that can least afford it.  Bounding scp_wait_ready()
	 * alone would have moved the reset here rather than removed it.
	 *
	 * Bound it on the watchdog counter instead, at the GXL_SCP_MBOX_PROBE_MS
	 * this function always meant.  The iteration count stays as the backstop
	 * for a board whose watchdog is not armed.
	 */
	for (i = 40000000; i > 0; i--) {
		if (readl((void *)(GXL_SCMI_PAYLOAD_NS_HP + 4)) == 1)
			break;
		if (ao_wdt_armed() &&
		    ao_wdt_elapsed_since(w0, GXL_SCP_MBOX_PROBE_MS))
			break;
	}
	spins = 40000000 - i;
	ms = ao_wdt_elapsed_ms() - w0;

	/* Read everything before printing: the UART races long printfs. */
	for (i = 0; i < 8; i++) {
		ospm[i] = readl((void *)(GXL_SCMI_PAYLOAD_NS_HP + (i * 4)));
		psci[i] = readl((void *)(GXL_MHU_AP_TO_SCP_PAYLOAD + (i * 4)));
	}

	scp_vprintf("SCP: mbox waited %u ms (%u cnt, spins %d)\n", ms,
		    (u32)(arch_cnt() - c0), spins);
	scp_vprintf("SCP: ospm %08x %08x %08x %08x\n",
		    ospm[0], ospm[1], ospm[2], ospm[3]);
	scp_vprintf("SCP: ospm+ %08x %08x %08x %08x\n",
		    ospm[4], ospm[5], ospm[6], ospm[7]);
	scp_vprintf("SCP: psci %08x %08x %08x %08x\n",
		    psci[0], psci[1], psci[2], psci[3]);
	scp_vprintf("SCP: psci+ %08x %08x %08x %08x\n",
		    psci[4], psci[5], psci[6], psci[7]);
}

/*
 * ⚪ The EL3 SCMI probe that used to sit here -- one BASE/PROTOCOL_VERSION
 * message framed into the OSPM channel and a doorbell rung from the secure
 * side -- is DELIBERATELY not carried onto the production branch.  It was
 * built because the non-secure agent could not address this shared memory at
 * all; that wall is gone, U-Boot proper's own agent is the right instrument,
 * and driving the channel from EL3 would CONTEND with it in the same boot.
 * It also still carried the inert CNTFRQ_EL0 deadline N55 removed from every
 * wait above.  It survives on v2026.07/lc-gxl-spl-scrom-fetch.
 */

/*
 * Read-only dump of the AHB share-SRAM secure/non-secure filter.
 *
 * ADDRESSES (vendor-authored, not inferred): the twelve
 * AO_SEC_SHARED_AHB_SRAM_REG{0..3}_{0,1,2} words are
 * 0xc8100000 + (0x70..0x7b << 2) = 0xC81001C0..0xC81001EC, and the four
 * AO_SEC_AO_AHB_SRAM_REG{0,1}_{0,1} are 0x7c..0x7f << 2 =
 * 0xC81001F0..0xC81001FC -- BSP v2015
 * arch/arm/include/asm/arch-gxl/secure_apb.h:1538-1585, identically in
 * arch/arm/cpu/armv8/gxl/firmware/scp_task/secure_apb.h:1715-1762.  The same
 * header gives a SECOND spelling of every one of them at base 0xda100000
 * (SEC_AO_SEC_*), so both are read here and printed side by side: whether the
 * two AO aliases behave alike at EL3 is an assumption nothing on this chain has
 * ever measured.
 *
 * WHAT THE FIELDS MEAN IS NOT KNOWN AND IS DELIBERATELY NOT GUESSED.  The GXL
 * datasheet documents the AHB SRAM only as a block extent (ADDRSPACE 0xC8000000
 * -- 0xC8013FFF, 80K; the alias at 0xD9000000 likewise) and carries no
 * REGISTER line for any of these sixteen words, and the BSP headers are
 * address-only.  So this prints raw values and names nothing beyond what the
 * vendor header names.
 *
 * 🔴 RESULT, AND WHY THIS IS NOT CALLED: reading the FIRST word of the block
 * HANGS THE AP.  Measured 2026-08-26 across three boots (E, F, G) with nothing
 * else changed.  Boot G printed, verbatim:
 *
 *	Trying to boot from RAM
 *	SCP: sramsec try ns 0xc81001c0
 *
 * and then nothing -- no matching "got" line, console dead, and the OTG port
 * never returned to the BootROM (each of the three needed a PPPS cycle).  So
 * 0xC81001C0 does not abort, it STALLS: an unclocked or undecoded slave on a
 * bus with no timeout, the same failure mode documented for the VPU's first
 * VCBUS read in arch/arm/mach-meson/dram-gx.c.  Boot F's orphaned half-line
 * flushed out at boot G's reset, so F reached the identical address.
 *
 * That makes the whole block unreadable, hence unprogrammable, from the AP on
 * this chain as it stands -- which is consistent with the only vendor stage
 * that touches it being BL30, i.e. code running on the AO M3, not on the AP.
 * The routine is kept, wired to nothing, because it is the instrument that
 * measured this; calling it wedges the board.
 *
 * WHY IT WOULD RUN HERE, FIRST: vendor BL30 -- not BL2, not BL31 -- is the only
 * vendor stage that writes this block (measured: the signed
 * vendor/amlogic/blx/aml-s905x-cc-v2/bl30_new.bin carries the five register
 * literals at file offset 0x3080-0x3094 and the code that stores through them
 * at 0x303e-0x307e; bl2_new.bin and bl31.img contain none of the sixteen
 * addresses at either alias).  Our chain runs no vendor BL30, so this dump --
 * taken BEFORE the open SC is started and before anything of ours touches the
 * window -- states what the BootROM/BL1 left, which is the state that is
 * currently refusing every non-secure access to 0xC8013400.
 */
#define GXL_AO_SEC_SRAM_SEC_NS_BASE	0xC81001C0UL
#define GXL_AO_SEC_SRAM_SEC_S_BASE	0xDA1001C0UL
#define GXL_AO_SEC_SRAM_SEC_WORDS	16

/*
 * MEASURED 2026-08-26, boot E: a version of this routine that read all sixteen
 * NS-alias words AND all sixteen 0xDA100000 secure-alias words into arrays
 * before printing anything WEDGED THE AP -- console dead immediately after
 * "Trying to boot from RAM", no first line, and the OTG port never came back to
 * the BootROM (a PPPS cycle was needed).  An unclocked or undecoded slave on
 * these buses stalls the access forever rather than aborting; the same failure
 * mode is documented for the VPU's first VCBUS read in
 * arch/arm/mach-meson/dram-gx.c.  So the address that stalls has to be named,
 * not assumed: print the address FIRST, then read it, one word at a time.  The
 * last address printed with no value after it is the one that hangs.
 */
static void scp_probe_sram_security(void) __attribute__((unused));

static void scp_probe_sram_security(void)
{
	int i;

	/*
	 * Boot F printed NOTHING either, including the half-line that preceded
	 * the read -- so a marker without a newline is not evidence a read was
	 * reached (the console log only carries completed lines).  Every marker
	 * below is a COMPLETE line, so "try" with no matching "got" names the
	 * stalling address unambiguously.
	 */
	for (i = 0; i < GXL_AO_SEC_SRAM_SEC_WORDS; i++) {
		u32 a = (u32)(GXL_AO_SEC_SRAM_SEC_NS_BASE + (i * 4));

		printf("SCP: sramsec try ns 0x%x\n", a);
		printf("SCP: sramsec got ns 0x%x %08x\n", a,
		       readl((void *)(uintptr_t)a));
	}

	for (i = 0; i < GXL_AO_SEC_SRAM_SEC_WORDS; i++) {
		u32 a = (u32)(GXL_AO_SEC_SRAM_SEC_S_BASE + (i * 4));

		printf("SCP: sramsec try se 0x%x\n", a);
		printf("SCP: sramsec got se 0x%x %08x\n", a,
		       readl((void *)(uintptr_t)a));
	}
}

/*
 * Read back what the SC measured about the AHB share-SRAM secure filter.
 *
 * The AP cannot read that register block: 0xC81001C0 STALLS this core at EL3
 * (scp_probe_sram_security() above, three boots).  Vendor BL30 -- code on the
 * AO M3 -- is the only vendor stage that programs it.  So the SC reads it and
 * leaves the values in AHB share SRAM at 0xD9011000, and this stage, which CAN
 * address that SRAM at EL3, states them.
 *
 * Layout is scp-firmware product/amlogic/gxl/include/gxl_sramsec.h:
 *   [0]      progress word, stamped BEFORE each access.  0x5C5A00FF = the probe
 *            ran to completion; anything else names the access that never
 *            returned (0x5C5A0010+i = NS word i, +0x30 secure word i,
 *            +0x50 write word i, +0x60 post-write read i).  0 = the SC never
 *            entered the probe at all.
 *   [1..16]  the sixteen words at 0xC81001C0, before any write
 *   [17..32] the same sixteen at the 0xDA1001C0 spelling (alias control)
 *   [33]     write plan: region written, or 0xFFFFFFFF for a read-only probe
 *   [34..36] the three values written
 *   [37..52] the sixteen words at 0xC81001C0 after the write
 *
 * Word order within each sixteen is REG0_0, REG0_1, REG0_2, REG1_0 ... REG3_2,
 * then AO_SEC_AO_AHB_SRAM_REG0_0, REG0_1, REG1_0, REG1_1 -- a DIFFERENT SRAM
 * (the AO AHB SRAM), printed only because it shares the register block.
 */
#define GXL_SRAMSEC_RESULT_BASE		0xD9012800UL
#define GXL_SRAMSEC_RESULT_WORDS	53

static void scp_report_sram_security(void)
{
	u32 r[GXL_SRAMSEC_RESULT_WORDS];
	int i;

	/* A probe readback: bring-up only. */
	if (!IS_ENABLED(CONFIG_MESON_GX_XPL_VERBOSE))
		return;
	for (i = 0; i < GXL_SRAMSEC_RESULT_WORDS; i++)
		r[i] = readl((void *)(GXL_SRAMSEC_RESULT_BASE + (i * 4)));

	/*
	 * A payload built WITHOUT SCP_GXL_SRAMSEC_PROBE -- the product default --
	 * never writes this block, and at rung 4 the address is transport's own
	 * OSPM staging slot, so an unguarded dump prints SCMI strings and stale
	 * pointers formatted as register values.  Measured on boot P6
	 * (2026-08-26), where it produced sixteen plausible-looking words that
	 * were nothing of the kind.  The magic is written LAST by the probe, so
	 * its absence means "no probe in this payload", not "probe failed".
	 */
	if ((r[0] & 0xFFFFFF00) != 0x5C5A0000) {
		printf("SCP: sramsec absent (no probe in payload, w0 %08x)\n",
		       r[0]);
		return;
	}

	/*
	 * r[33] is the experiment number (0 = read-only).  r[34..36] are that
	 * experiment's own result words; for experiment 1 they are, in order,
	 * REG1_2 read back after writing 0x3000 to it, REG1_2 read back after
	 * writing 0 to it again (a value that returns to 0 cannot be OTP), and
	 * the aperture value written to REG3_2.
	 */
	printf("SCP: sramsec prog %08x exp %08x res %08x %08x %08x\n",
	       r[0], r[33], r[34], r[35], r[36]);
	for (i = 0; i < 4; i++)
		printf("SCP: sramsec ns%d %08x %08x %08x %08x\n", i,
		       r[1 + i * 4], r[2 + i * 4], r[3 + i * 4], r[4 + i * 4]);
	for (i = 0; i < 4; i++)
		printf("SCP: sramsec se%d %08x %08x %08x %08x\n", i,
		       r[17 + i * 4], r[18 + i * 4], r[19 + i * 4],
		       r[20 + i * 4]);
	if (r[33] != 0)
		for (i = 0; i < 4; i++)
			printf("SCP: sramsec po%d %08x %08x %08x %08x\n", i,
			       r[37 + i * 4], r[38 + i * 4], r[39 + i * 4],
			       r[40 + i * 4]);
}

/*
 * G5/D3 PAIRING GUARD (ledger N339) -- refuse a mis-pinned SC payload BEFORE
 * the 0xd3 send.
 *
 * In a per-board-binary world (scmi-plan.md G5: one contract struct compiled
 * per SKU, scp-firmware boards/<board>.c) mispairing the slice is otherwise
 * SILENT.  The ROM does not verify BL30, and the FIT's SHA-256 -- which SPL
 * recomputes and hands over at 0xd0/0xd1/0xd5 -- covers a CORRUPT payload,
 * not the RIGHT payload for the WRONG board: both hash cleanly.  So the check
 * is of an IDENTITY the payload carries: the LAST word of the staged image,
 * 0xB0A4 << 16 | contract-version << 8 | board-id (registry: scp-firmware
 * product/amlogic/gxl/include/gxl_board.h; the packaging side appends and
 * read-back-verifies it).  LAST, never first: the M3 vector table occupies
 * the head of the 0x001C0000 staging window.
 *
 * The expected id is CONFIG_MESON_GXL_SCP_BOARD_ID, the number this build's
 * own defconfig declared and that binman stamped into the same FIT node's
 * `board-id` property (which the host-side gate
 * amlogic/software/boot/gxl/tests/ asserts against the decompiled FIT).  On
 * any mismatch -- missing signature, foreign contract version, or a foreign
 * board -- SPL refuses the FETCH and degrades exactly the way a mute SC is
 * already degraded (return without touching the mailbox; BL31 reports
 * "SCP never became ready" and the DT fixup leaves SCMI off -- the chain
 * boots, no SC).  A defconfig with the symbol at 0 skips the check out loud.
 */
#define GXL_SCP_BOARD_TRAILER_SIG	0xB0A4U
#define GXL_SCP_BOARD_CONTRACT_VER	1U

static int scp_board_pairing_check(uintptr_t addr, size_t size)
{
#ifdef CONFIG_MESON_GXL_SCP_BOARD_ID
	u32 expect = CONFIG_MESON_GXL_SCP_BOARD_ID;
#else
	u32 expect = 0;
#endif
	u32 trailer;
	size_t off;

	if (!expect) {
		scp_vprintf("SCP: board-id unset (CONFIG_MESON_GXL_SCP_BOARD_ID=0), skipping pairing check\n");
		return 0;
	}

	if (size < 8) {
		printf("SCP: board mismatch, refusing fetch (0x%x B carries no room for a trailer; board expects id %u)\n",
		       (u32)size, expect);
		return -EINVAL;
	}

	/* Last COMPLETE word of the staged slice -- 4-aligned by construction
	 * (the payload linker pads the image, and the trailer itself is one
	 * word), but round down here rather than assume it.
	 */
	off = (size - 4) & ~(size_t)0x3;
	trailer = readl((void *)(addr + off));

	if ((trailer >> 16) != GXL_SCP_BOARD_TRAILER_SIG ||
	    ((trailer >> 8) & 0xFFU) != GXL_SCP_BOARD_CONTRACT_VER) {
		printf("SCP: board mismatch, refusing fetch (payload has no readable board trailer, word 0x%08x at 0x%x+0x%x; board expects id %u contract v%u)\n",
		       trailer, (u32)addr, (u32)off, expect,
		       GXL_SCP_BOARD_CONTRACT_VER);
		return -EINVAL;
	}

	if ((trailer & 0xFFU) != expect) {
		printf("SCP: board mismatch, refusing fetch (payload board id %u, this board is %u)\n",
		       trailer & 0xFFU, expect);
		/* -EXDEV: a FIT for a DIFFERENT board, which stops the boot */
		return -EXDEV;
	}

	scp_vprintf("SCP: board id %u paired (trailer 0x%08x)\n", expect, trailer);
	return 0;
}

static void meson_gxl_scp_upload(const void *fw, size_t size)
{
	uintptr_t addr = (uintptr_t)fw;
	u64 c0 = arch_cnt();
	/*
	 * The AS-FOUND value of the ready register, sampled before this stage
	 * touches anything.  scp_wait_ready() compares against it: that is the
	 * only way an AP-side reader can tell "these bits were never written by
	 * anybody" from "the SC started painting and stopped".
	 */
	u32 stat_found = readl((void *)GXL_AO_RTI_SCP_STAT);

	/* scp_probe_sram_security() is DELIBERATELY not called -- it hangs. */

	/*
	 * Both budget lines are cheap and are the only place a reader can see
	 * what this stage costs of the ROM watchdog's period -- the number every
	 * bound in this file is chosen against, and the one that would silently
	 * rot if the stages before us got slower.
	 */
	scp_vprintf("SCP: enter at %u ms of %u (AO wdt ctrl 0x%x)\n",
		    ao_wdt_elapsed_ms(), ao_wdt_period_ms(),
		    readl((void *)GXL_AO_WDT_CTRL));

	if (scp_running()) {
		printf("SCP: already running (stat 0x%x)\n", stat_found);
		return;
	}

	if (!fw || !size || size > GXL_SCP_CODE_SIZE) {
		printf("SCP: size 0x%x out of range (max 0x%x)\n",
		       (u32)size, (u32)GXL_SCP_CODE_SIZE);
		return;
	}

	/*
	 * The SC-ROM reads the image from DRAM itself, past the AP's caches:
	 * with the DRAM stage's D-cache on (spl-dram-stage-gx.c), what the FIT
	 * loader just wrote is still in it.  Clean it to DRAM first, or the ROM
	 * hashes stale memory and refuses the image.
	 */
	if (dcache_status()) {
		uintptr_t start = addr & ~(uintptr_t)(CONFIG_SYS_CACHELINE_SIZE - 1);

		flush_dcache_range(start, ALIGN(addr + size, CONFIG_SYS_CACHELINE_SIZE));
	}

	if (addr != GXL_SCP_STAGE_BASE) {
		printf("SCP: refuse, scp at 0x%x not LAYOUT 0x%x\n",
		       (u32)addr, (u32)GXL_SCP_STAGE_BASE);
		return;
	}

	/* BEFORE any 0xd3 -- a wrong-board payload must never be fetched (see
	 * scp_board_pairing_check above).  A payload with no readable trailer
	 * keeps the degraded path it already survives (no SC, no SCMI, board
	 * boots): it may be this board's SC on an older contract.
	 *
	 * A trailer naming a DIFFERENT board id means this FIT was built for
	 * another board, and its U-Boot, BL31 and DTB are that board's too.
	 * That stops the boot, with the EE watchdog armed so the board resets
	 * into the BootROM for a host to stage again; it used to boot the
	 * foreign U-Boot and DTB with no watchdog at all (GXL DDR3 boot-chain
	 * FMA FB14, operator decision 2026-10-19: claude amlogic/software/boot/
	 * gxl/audit/2026-10-19-gxl-ddr3-boot-chain-failure-modes.md).
	 */
	{
		int pair = scp_board_pairing_check(addr, size);

		if (pair == -EXDEV) {
			void meson_gx_wdt_arm_ms(u32 ms);

			puts("SCP: this FIT is for another board; stopping the boot\n");
			meson_gx_wdt_arm_ms(2000);
			for (;;)
				;
		}
		if (pair)
			return;
	}

	if (scp_fetch_window(addr, size, "scp"))
		return;
	scp_wait_ready(size, stat_found);
	/*
	 * Both remaining calls are READ-ONLY: they state what the SC left in
	 * the share SRAM and nothing more.  Nothing in this stage writes the
	 * OSPM window, so the channel reaches U-Boot proper's SCMI agent
	 * exactly as the SC left it.
	 */
	scp_report_sram_security();
	scp_probe_mailboxes();

	/*
	 * The CNTPCT delta is printed beside the watchdog's own milliseconds so
	 * the two clocks can be compared in every boot log: CNTFRQ_EL0 reads 0
	 * here, so this pair is the only statement of what CNTPCT actually
	 * ticks at -- and measured 2026-08-30 it does NOT agree with itself
	 * across two windows of one boot (~599 MHz vs ~617 MHz), which is the
	 * evidence that no bound may be built on it.
	 */
	scp_vprintf("SCP: leave at %u ms of %u (%u cnt for the stage)\n",
		    ao_wdt_elapsed_ms(), ao_wdt_period_ms(),
		    (u32)(arch_cnt() - c0));
}

void board_fit_image_post_process(const void *fit, int node, void **p_image,
				  size_t *p_size)
{
	const char *name = fit_get_name(fit, node, NULL);

	if (!name || strcmp(name, "scp"))
		return;
	if (!p_image || !p_size)
		return;

	meson_gxl_scp_upload(*p_image, *p_size);
}

#endif /* FIT_IMAGE_POST_PROCESS */
