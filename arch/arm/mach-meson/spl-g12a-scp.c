// SPDX-License-Identifier: GPL-2.0+
/*
 * G12A/SM1: start the SC (the system control processor -- the Cortex-M that
 * runs BL30) from the open U-Boot xPL chain.
 *
 * WHO OWNS THIS.  TF-A has aml_scpi_upload_scp_fw()
 * (plat/amlogic/common/aml_scpi.c) and g12a_scp_ensure_ready()
 * (plat/amlogic/g12a/g12a_bl31_setup.c) and it is tempting to read those as
 * "BL31 starts the SC".  They are a fallback: they only run when the SC is
 * NOT already ready, and on the vendor chain it always is, because the
 * VENDOR BL2 starts it before BL31 is entered.  Disassembled from
 * vendor/amlogic/blx/aml-s905d3-cc/bl2_new.bin (61440 B, runs at
 * 0xFFFA1000), the BL30 case of BL2's image dispatch is:
 *
 *   fffab014: ldr  x3, [x29, #192]
 *   fffab018: mov  x0, x20            ; BL30 image in memory
 *   fffab01c: mov  w2, #0x1000
 *   fffab020: bl   0xfffa149c         ; <-- the upload below
 *   fffab024: cbnz w0, error
 *   fffab028: mov  w20, #0x258        ; 600 tries
 *   fffab048: mov  w0, #0x3e8 ; bl 0xfffa5cd8      ; delay
 *   fffab050: mov  x0, #0x23c ; movk x0, #0xff80, lsl #16   ; 0xFF80023C
 *   fffab058: ldr  w0, [x0]
 *   fffab05c: ubfx x0, x0, #20, #2    ; bits [21:20]
 *   fffab060: cmp  w0, #0x3
 *   fffab064: b.ne 0xfffab02c         ; spin until the SC reports running
 *
 * and 0xfffa149c itself, which is the whole handshake:
 *
 *   fffa1518: mov  x0, #0xfffd0000 ; bl memcpy   ; BL30 -> AHB SRAM
 *   fffa1520: mov  x0, #0x1800 ; movk #0xfffe    ; first 0x1000 B -> 0xFFFE1800
 *   fffa1534: mov  x0, #0x200  ; movk #0xff80    ; 0xFF800200
 *   fffa1540: and  w0, w0, #0x3 ; cmp #0x3       ; wait for the SC ROM
 *   fffa158c: mov  x0, #0x7a00 ; movk #0xfffe    ; 0xFFFE7A00 payload
 *   fffa1594: str  w1, [x0]
 *   fffa15a8: bl   0xfffa1420                    ; write mailbox 3 SET
 *   fffa15b8: bl   0xfffa145c                    ; read mailbox 3 STAT
 *
 * with the two commands built on the stack at fffa14ac/fffa14bc as 0xd3
 * (SCPI SET_FW_ADDR, payload = 0xFFFD0000) and 0xd2 (SCPI FW_SIZE, payload =
 * the image size).  Note the vendor sends the BARE command word -- no
 * (size << 20) field, unlike TF-A's aml_scpi_cmd() -- and a 32-bit SRAM
 * address, not TF-A's 64-bit one.  The size bound it enforces (0x10C00) is
 * exactly 0xFFFE0C00 - 0xFFFD0000, which is how we know 0xFFFD0000..0xFFFE0C00
 * is the SC firmware window and 0xFFFE1800 a separate 4 KiB one.
 *
 * (That derivation used to read 0xFFFE1800 - 0xFFFD0000, which is 0x11800, not
 * 0x10C00.  The premise and the conclusion could not both hold: 0x10C00 is
 * exactly the window END the sentence CONCLUDES, 0xFFFE0C00.  The code always
 * used the right number -- G12A_SCP_FW_SRAM_MAX 0x00010C00, "to 0xFFFE0C00" --
 * so this was a broken derivation under a correct bound, which is precisely
 * what a future reader would use to widen it.)
 *
 * So the early loader owns starting the SC, which is also what
 * amlogic/software/boot/design/clean-slate-firmware-division.md assigns.
 * This file is the open chain's equivalent of that BL2 code, run from the
 * DRAM stage once the FIT's "scp" image has been loaded and before the jump
 * to BL31 -- the same ordering the vendor has.
 *
 * 0xFFFD0000 is also G12A_FW_HOLD_AML_DDR, the residual park for the DDR
 * training blobs (asm/arch/g12a-ddr-fw.h).  That is not a conflict: training
 * is complete before the DRAM stage runs, and the vendor BL2 reuses the same
 * SRAM for the same two purposes in the same order.
 */

#include <image.h>
#include <spl.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>

#if CONFIG_IS_ENABLED(FIT_IMAGE_POST_PROCESS)

/* AHB SRAM is 0xFFFA0000..0xFFFE7FFF (S905D3 datasheet ADDRSPACE "ahb sram"). */
#define G12A_SCP_FW_SRAM	0xFFFD0000UL	/* SC firmware window */
#define G12A_SCP_FW_SRAM_MAX	0x00010C00UL	/* to 0xFFFE0C00; vendor bound */
#define G12A_SCP_FW_HEAD	0xFFFE1800UL	/* first 4 KiB, also vendor */
#define G12A_SCP_FW_HEAD_SIZE	0x00001000UL

/*
 * The SC mask ROM's own ceiling on the @AML CONTAINER it is handed -- a
 * DIFFERENT and LOWER limit than G12A_SCP_FW_SRAM_MAX, which only bounds the
 * AHB staging window this file copies into.  Measured on silicon 2026-09-28 at
 * a 16 B container granule: 65 088 B (0xFE40) accepts with a live BL31 SCMI
 * answer, 65 104 B (0xFE50) refuses with "M3 CHK:1::1" (code 0x1, the ROM's
 * post-digest size check; a bad digest is 0x22) and the SC never starts.
 * /amlogic/software/boot/g12/audit/2026-09-21-g12a-sc-rom-tcm-copy-span-bracket.md
 * "Final 16 B (n16e1)".  The size passed to meson_g12a_scp_start() is the whole
 * container (header included), which is the quantity the ROM counts.
 *
 * The producer-side twin is scp-firmware's G12A_SC_ROM_CONTAINER_MAX
 * (product/amlogic/g12a, 1b270a76); this value must equal it.
 */
#define G12A_SC_ROM_CONTAINER_MAX	0x0000FE40UL	/* 65 088 B, measured */

/* HIU secure mailbox, channel 3 (AP -> SC).  Same regs as TF-A g12a_def.h. */
#define G12A_HIU_MAILBOX_SET_3		0xFF63C428UL
#define G12A_HIU_MAILBOX_STAT_3		0xFF63C42CUL
#define G12A_MHU_AP_TO_SCP_PAYLOAD	0xFFFE7A00UL

/*
 * AO handshake the vendor BL2 gates the upload on.  Not named in the S905D3
 * datasheet register list; purpose taken from the disassembly above: bits
 * [1:0] read 3 while the SC ROM is waiting for firmware and drop to 0 once it
 * has taken it.
 */
#define G12A_AO_SCP_HANDSHAKE		0xFF800200UL
#define G12A_AO_SCP_HANDSHAKE_MASK	0x3U

/* AML_AO_RTI_SCP_STAT in TF-A: bits [21:20] == 3 means the SC is running. */
#define G12A_AO_RTI_SCP_STAT		0xFF80023CUL

/* SC -> AP mailbox payload; the ROM's answer to a command lands here. */
#define G12A_MHU_SCP_TO_AP_PAYLOAD	0xFFFE7800UL
#define G12A_HIU_MAILBOX_STAT_0		0xFF63C408UL
#define G12A_HIU_MAILBOX_CLR_0		0xFF63C40CUL

#define SCPI_CMD_SET_FW_ADDR	0xd3
#define SCPI_CMD_FW_SIZE	0xd2

/*
 * udelay() does NOT delay in this phase.  Measured on AML-S905D3-CC: a
 * 20000-iteration loop of udelay(1000) -- nominally 20 s -- completed inside
 * the time the SC took to print ~40 characters of its own banner at 115200
 * baud, i.e. under 4 ms.  A wait built on it is not a wait, and it produced a
 * false "SCP never became ready" for an SC that BL31 found ready a few tens of
 * milliseconds later.  The architected counter is running regardless, so read
 * it directly.  CNTFRQ_EL0 of 0 means nobody programmed it; fall back to a
 * plain iteration bound rather than spinning forever.
 */
static u64 arch_cnt(void)
{
	u64 v;

	asm volatile("mrs %0, cntpct_el0" : "=r" (v));
	return v;
}

static u64 arch_frq(void)
{
	u64 v;

	asm volatile("mrs %0, cntfrq_el0" : "=r" (v));
	return v;
}

/*
 * A real-time deadline, or 0 when CNTFRQ_EL0 says nobody programmed the timer.
 *
 * 🔴 EVERY BOUNDED WAIT IN THIS FILE IS BUILT ON THIS NOW, NOT ON udelay().
 * The measurement at the top of this file is the reason, and it was applied to
 * exactly ONE of the four waits when it was made: the final SC-ready poll.  The
 * other three stayed on udelay(), so at the file's own measured ~0.2 us per
 * nominal 1000 us they were:
 *
 *   the SC-ROM-ready wait   1000 x udelay(1000)   nominal 1 s    real ~0.2 ms
 *   scp_send(), twice      100000 x udelay(10)    nominal 1 s    real ~20 ms
 *   the handshake-leaves-3   600 x udelay(1000)   nominal 0.6 s  real ~0.12 ms
 *
 * The first is the load-bearing one: on expiry meson_g12a_scp_start() returns
 * WITHOUT UPLOADING BL30, so a ~0.2 ms budget decided whether the SC ran at all
 * -- against a vendor BL2 which, disassembled at the top of this file, allows
 * itself 600 x 1000 us for the equivalent wait.  That is the same false-negative
 * class the final loop was converted to fix, one loop upstream of it.
 *
 * ⚠️ THIS CHANGES BOOT TIMING, IN ONE DIRECTION ONLY.  Each converted wait now
 * takes up to its NOMINAL duration in real time instead of a fraction of a
 * millisecond.  It cannot shorten a wait and it cannot skip an upload that
 * happens today; the only cost is that a boot which already fails takes up to
 * ~2.6 s longer to say so.  Unvalidated on hardware -- see the commit message.
 */
static u64 arch_deadline_us(u64 us)
{
	u64 frq = arch_frq();

	if (!frq)
		return 0;

	return arch_cnt() + ((frq * us) / 1000000ULL);
}

static bool arch_expired(u64 end)
{
	return end && arch_cnt() >= end;
}

static bool scp_running(void)
{
	return ((readl((void *)G12A_AO_RTI_SCP_STAT) >> 20) & 0x3U) == 0x3U;
}

/*
 * Send one bare SCPI command word on secure mailbox 3 and wait for the SC to
 * consume it (STAT_3 back to 0).  Bounded: an SC that never started leaves
 * STAT_3 set forever, and an unbounded spin here would hang the boot in xPL
 * exactly the way it used to hang in EL3.
 */
static int scp_send(u32 cmd, u32 payload)
{
	int t;
	int ret = -ETIMEDOUT;
	u64 end;

	writel(payload, (void *)G12A_MHU_AP_TO_SCP_PAYLOAD);
	writel(cmd, (void *)G12A_HIU_MAILBOX_SET_3);

	/*
	 * 1 s of REAL time, with the iteration count as the FALLBACK bound for
	 * a phase where CNTFRQ_EL0 reads 0.  Same shape as the SC-ready poll
	 * below, including its iteration count: the old 100000 was sized for a
	 * body containing udelay(10), and reusing it beside a real deadline
	 * would end the wait after 100000 bare MMIO reads -- far sooner than
	 * the second it is supposed to allow, i.e. the bug this is fixing, one
	 * layer down.  The deadline is what stops it when the timer works; the
	 * count is what stops it when the timer does not.
	 */
	end = arch_deadline_us(1000000ULL);
	for (t = 30000000; t > 0; t--) {
		if (!readl((void *)G12A_HIU_MAILBOX_STAT_3)) {
			ret = 0;
			break;
		}
		if (arch_expired(end))
			break;
	}

	if (ret)
		printf("SCP: cmd 0x%x pay 0x%x not taken (mb3 0x%x hs 0x%x)\n",
		       cmd, payload,
		       readl((void *)G12A_HIU_MAILBOX_STAT_3),
		       readl((void *)G12A_AO_SCP_HANDSHAKE));

	return ret;
}

static void meson_g12a_scp_start(const void *fw, size_t size)
{
	int t;
	u64 end;
	bool ready;
	u32 stat, hs, mb0, rsp;
	/*
	 * The AS-FOUND value of the ready register, sampled before this stage
	 * touches anything.  The final report below compares against it, which
	 * is the only way an AP-side reader can tell "the bits were never
	 * written by anybody" from "the SC started painting and stopped".
	 */
	u32 stat_found = readl((void *)G12A_AO_RTI_SCP_STAT);

	if (scp_running()) {
		printf("SCP: already running (stat 0x%x)\n", stat_found);
		return;
	}

	if (!size || size > G12A_SCP_FW_SRAM_MAX) {
		printf("SCP: BL30 size 0x%x out of range (max 0x%x)\n",
		       (u32)size, (u32)G12A_SCP_FW_SRAM_MAX);
		return;
	}

	/*
	 * Refuse here, before anything is staged and before either SCPI command
	 * is sent: the ROM would take the upload and then refuse it, leaving the
	 * SC mute with only an "M3 CHK:1::1" on the console to say why.  The
	 * staging check above stays -- it bounds a different window.
	 */
	if (size > G12A_SC_ROM_CONTAINER_MAX) {
		printf("SCP: BL30 container %u B (0x%x) exceeds the SC ROM ceiling %u B (0x%x) -- BL30 NOT uploaded, SC will not start\n",
		       (u32)size, (u32)size, (u32)G12A_SC_ROM_CONTAINER_MAX,
		       (u32)G12A_SC_ROM_CONTAINER_MAX);
		return;
	}

	memcpy((void *)G12A_SCP_FW_SRAM, fw, size);
	memcpy((void *)G12A_SCP_FW_HEAD, fw,
	       size < G12A_SCP_FW_HEAD_SIZE ? size : G12A_SCP_FW_HEAD_SIZE);


	/*
	 * The SC ROM must be waiting for firmware before the commands go out.
	 *
	 * 🔴 THE ONE WAIT WHOSE EXPIRY DECIDES WHETHER BL30 IS UPLOADED AT ALL.
	 * 1 s of REAL time; it used to be 1000 x udelay(1000), which is ~0.2 ms
	 * of real time in this phase.  The vendor BL2 allows 600 x 1000 us for
	 * the equivalent wait.
	 */
	ready = false;
	end = arch_deadline_us(1000000ULL);
	for (t = 30000000; t > 0; t--) {
		if ((readl((void *)G12A_AO_SCP_HANDSHAKE) &
		     G12A_AO_SCP_HANDSHAKE_MASK) == G12A_AO_SCP_HANDSHAKE_MASK) {
			ready = true;
			break;
		}
		if (arch_expired(end))
			break;
	}
	if (!ready) {
		/*
		 * 🔴 SAY WHAT WAS NOT DONE, NOT ONLY WHAT WAS OBSERVED.  This
		 * path returns without uploading BL30, so the SC never starts
		 * and every later symptom -- no SCMI, no DVFS, TF-A taking its
		 * g12a_scp_ensure_ready() fallback -- is downstream of a line
		 * that only said the handshake was wrong.  board_fit_image_
		 * post_process() is void in U-Boot's own API, so there is no
		 * return channel to the caller; the console line is the whole
		 * report and it now names the consequence.
		 */
		printf("SCP: ROM not ready to accept firmware (hs 0x%x stat 0x%x)"
		       " -- BL30 NOT uploaded, SC will not start\n",
		       readl((void *)G12A_AO_SCP_HANDSHAKE),
		       readl((void *)G12A_AO_RTI_SCP_STAT));
		return;
	}

	if (scp_send(SCPI_CMD_SET_FW_ADDR, G12A_SCP_FW_SRAM))
		return;
	if (scp_send(SCPI_CMD_FW_SIZE, size))
		return;

	/*
	 * The vendor waits for the handshake to leave 3 and then reach 0.
	 * 0.6 s of REAL time -- vendor's own 600 x 1000 us -- where this used to
	 * be 600 x udelay(1000), i.e. ~0.12 ms.  Expiry here is NOT fatal: the
	 * verdict below is the SC's own status bits, and this loop only avoids
	 * polling them before the ROM has had a chance to move.
	 */
	end = arch_deadline_us(600000ULL);
	for (t = 30000000; t > 0; t--) {
		if ((readl((void *)G12A_AO_SCP_HANDSHAKE) &
		     G12A_AO_SCP_HANDSHAKE_MASK) != G12A_AO_SCP_HANDSHAKE_MASK)
			break;
		if (arch_expired(end))
			break;
	}

	/*
	 * The verdict is the SC's own status bits, not the handshake.
	 *
	 * The bound is deliberately generous.  Measured on AML-S905D3-CC, the
	 * SC accepts the image ("M3 CHK:0"), then prints its own banner and
	 * brings up its tasks before it sets these bits; a 2000-iteration
	 * bound reported "never became ready" for an SC that BL31, a few tens
	 * of milliseconds later, found ready -- a false negative in the very
	 * report this code exists to produce.  It exits the instant the bits
	 * appear, so the bound only costs time when the SC genuinely fails.
	 */
	end = arch_deadline_us(3000000ULL);	/* 3 s of real time */
	for (t = 30000000; t > 0; t--) {
		if (scp_running()) {
			printf("SCP: started, 0x%x B at 0x%x (stat 0x%x)\n",
			       (u32)size, (u32)G12A_SCP_FW_SRAM,
			       readl((void *)G12A_AO_RTI_SCP_STAT));
			return;
		}
		if (arch_expired(end))
			break;
	}

	/*
	 * 🔴 REPORT THE READINGS AND WHAT THEY DISCRIMINATE -- NEVER A VERDICT
	 * THE FIELDS CANNOT CARRY.
	 *
	 * This used to print ONE sentence, "SCP: never became ready", while the
	 * same line's own four fields said the opposite.  Measured 2026-09-21 on
	 * AML-S905D3-CC, seven boots, verbatim:
	 *
	 *   SCP: never became ready (stat 0x3a00000 hs 0x0 mb0 0x1 rsp 0xcf57a456)
	 *
	 * on boots where BL31, tens of milliseconds later, got SCMI rc=0 from
	 * BOTH BASE and POWER_DOMAIN.  `hs 0x0` is a POSITIVE reading -- the
	 * header above states, from the vendor BL2 disassembly, that bits [1:0]
	 * read 3 while the SC ROM is WAITING for firmware and drop to 0 once it
	 * has TAKEN it -- and `mb0 0x1` is an SC->AP doorbell, i.e. traffic in
	 * the right direction.
	 *
	 * Three states reach this line and they are three different faults:
	 *
	 *   hs still 3                       the ROM never took the upload
	 *   hs left 3, stat == as-found      nobody ever PAINTED the bits; says
	 *                                    NOTHING about whether the SC runs
	 *   hs left 3, stat moved, not 3     a partial or retracted paint
	 *
	 * The middle one is what the open payload does today: its
	 * g12a_paint_scp_ready() is in no shipped image -- ZERO accesses to
	 * 0xFF80023C, no set and no clear (whole-image disassembly,
	 * amlogic/software/boot/g12/audit/
	 * 2026-09-21-g12a-sc-rom-tcm-copy-span-bracket.md).  The paint is the
	 * BL30 PAYLOAD's job, never the ROM's, so on such a payload this
	 * register can only ever read as-found no matter how healthy the SC is.
	 * Collapsing that into "never became ready" was read as a timeout and
	 * then as a wrong poll bit before the disassembly settled it -- which is
	 * the cost this rewrite exists to stop paying.  The bound is NOT the
	 * defect and is deliberately unchanged.
	 *
	 * Every register is sampled ONCE into a local first: the two lines below
	 * must describe one sample, not five re-reads of a moving target.
	 */
	stat = readl((void *)G12A_AO_RTI_SCP_STAT);
	hs = readl((void *)G12A_AO_SCP_HANDSHAKE);
	mb0 = readl((void *)G12A_HIU_MAILBOX_STAT_0);
	rsp = readl((void *)G12A_MHU_SCP_TO_AP_PAYLOAD);

	printf("SCP: ready bits %d of stat 0x%x (want 3, as-found 0x%x) hs 0x%x mb0 0x%x rsp 0x%x\n",
	       (stat >> 20) & 0x3U, stat, stat_found, hs, mb0, rsp);

	if ((hs & G12A_AO_SCP_HANDSHAKE_MASK) == G12A_AO_SCP_HANDSHAKE_MASK)
		printf("SCP: the SC ROM is STILL WAITING for firmware (hs 3) -- it never took the upload, so the SC is not running\n");
	else if (stat == stat_found)
		printf("SCP: the ROM TOOK the firmware (hs left 3)%s, and the ready bits are UNCHANGED from as-found -- nobody painted them. The paint is the BL30 payload's job, so this line says NOTHING about whether the SC is running; confirm with SCMI, not with this bit\n",
		       mb0 ? ", SC->AP doorbell raised"
		           : ", but no SC->AP doorbell was raised either, so nothing here witnesses the SC");
	else
		printf("SCP: the ready bits MOVED (as-found 0x%x) but never reached 3 -- a partial or retracted paint; confirm with SCMI, not with this bit\n",
		       stat_found);
}

/*
 * Called by spl_load_fit_image() for every image in the FIT, with the image
 * data still where it was read.  Only the SCP one is ours.  Doing the start
 * here rather than at spl_board_prepare_for_boot() keeps it at the same place
 * GXBB does it (arch/arm/mach-meson/spl-gxbb.c) and needs no state carried
 * across functions; the "scp" loadable is loaded before the jump to BL31
 * either way.
 */
void board_fit_image_post_process(const void *fit, int node, void **p_image,
				  size_t *p_size)
{
	const char *name = fit_get_name(fit, node, NULL);

	if (!name || strcmp(name, "scp"))
		return;

	meson_g12a_scp_start(*p_image, *p_size);
}

#endif /* FIT_IMAGE_POST_PROCESS */
