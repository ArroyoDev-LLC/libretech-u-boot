// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 * (C) Copyright 2018 Neil Armstrong <narmstrong@baylibre.com>
 */

#include <errno.h>
#include <init.h>
#include <log.h>
#include <net.h>
#include <time.h>
#include <vsprintf.h>
#include <asm/arch/boot.h>
#include <asm/arch/eth.h>
#include <asm/arch/g12a.h>
#include <asm/arch/mem.h>
#include <asm/arch/meson-vpu.h>
#include <asm/io.h>
#include <asm/armv8/mmu.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/libfdt.h>
#include <linux/printk.h>
#include <linux/sizes.h>
#include <linux/string.h>

int meson_get_boot_device(void)
{
	return readl(G12A_AO_SEC_GP_CFG0) & G12A_AO_BOOT_DEVICE;
}

#if defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT)
/*
 * The open TF-A puts its firmware in the first 16 MiB of DRAM (BL31 at
 * 0x00100000, mailboxes at 0x00200000, ramoops at 0x00300000, the optional
 * OP-TEE band at 0x00400000 -- arch/arm/include/asm/arch-meson/g12a.h and
 * plat/amlogic/{g12a,g12b}/*_def.h).  That is GXL's low map exactly; this
 * file is board-gx.c with the addresses named by g12a.h (operator decisions
 * 2026-09-26 and 2026-10-17).
 *
 * The -libre board DTS describes the block itself (meson-g12-libre-
 * firmware.dtsi), so on that DTB this is a no-op; it stays to correct a
 * vendor-layout DTB a libre board is handed.  The mechanics -- and the
 * reasons the nodes must be nodes rather than a bare /memreserve/ -- are in
 * board-common.c, shared with GXL.
 *
 * THREE nodes, ordered exactly as GXL orders them:
 *
 *   secure-firmware@0        0x00000000 +  2 MiB  no-map   BL31 -- and, unlike
 *                                                          GXL, the first
 *                                                          megabyte is
 *                                                          NOTHING: this
 *                                                          family has no ROM
 *                                                          mirror, the
 *                                                          megabyte is held
 *                                                          only to keep the
 *                                                          map GXL's
 *   secure-monitor-shmem@200000
 *                            0x00200000 +  1 MiB  no-map   SMC mailboxes, NS
 *   ramoops@300000           0x00300000 +  1 MiB  MAPPED   pstore
 *
 * The OP-TEE band above them is NOT described here: it is Linux's unless
 * OP-TEE runs, and meson_libre_optee_fixup() adds it when it does.
 *
 * The mailbox MiB is NON-secure -- it sits at TF-A's AML_NSDRAM0_BASE exactly
 * so meson_sm can reach it -- so it must not be described as secure
 * firmware.  It still needs a node, and that node must be no-map:
 * meson_sm_map_shmem() learns the base by SMC and maps it with
 * ioremap_cache(), and arm64's __ioremap_prot() refuses any
 * pfn_is_map_memory() page.  Unreserved, or reserved-but-mapped, the mapping
 * WARNs and returns NULL and the secure-monitor driver fails to probe.
 *
 * ramoops is the opposite: it needs a compatible so a driver DOES bind, and it
 * must stay MAPPED -- pstore cannot reach a no-map region.
 */
static void meson_add_libre_secure_reservation(void *fdt)
{
	int rmem, node;

	/*
	 * All three are wrong for this firmware, and all are deleted BEFORE
	 * the nodes are added: fdt_del_node() invalidates offsets, so the
	 * /reserved-memory lookup below has to happen afterwards.
	 *
	 *   secmon@5000000 -- BL31 is not there in this build.
	 *   secmon@5300000 -- the 32 MiB BL32 zone.  Commented out in our
	 *                     kernel DTS since d133f4a5ca74c, but a foreign or
	 *                     older blob may still carry it, and 0x05300000 is
	 *                     plain non-secure DRAM under this layout.
	 *   secure-monitor-shmem@e00000
	 *                  -- the mailbox node from before the 2026-10-17
	 *                     reorder, as an older -libre DTB still carries it.
	 *                     (secure-firmware@0 keeps its name and is
	 *                     replaced below with the new size.)
	 *   ramoops@*      -- see meson_del_ramoops_nodes(); pstore binds one.
	 *
	 * There is no hwrom@0 to delete: upstream declares that node only on
	 * GX (meson-gx.dtsi), because this family has no ROM mirror to name.
	 */
	meson_del_node(fdt, "/reserved-memory/secmon@5000000");
	meson_del_node(fdt, "/reserved-memory/secmon@5300000");
	meson_del_node(fdt, "/reserved-memory/secure-monitor-shmem@e00000");
	meson_del_ramoops_nodes(fdt);

	rmem = fdt_path_offset(fdt, "/reserved-memory");
	if (rmem < 0) {
		rmem = fdt_add_subnode(fdt, 0, "reserved-memory");
		if (rmem < 0)
			goto fail;
		if (fdt_setprop_u32(fdt, rmem, "#address-cells", 2) ||
		    fdt_setprop_u32(fdt, rmem, "#size-cells", 2) ||
		    fdt_setprop_empty(fdt, rmem, "ranges"))
			goto fail;
	}

	if (meson_add_rsv_node(fdt, "secure-firmware@0",
			       G12A_LIBRE_BLOCK_BASE, G12A_LIBRE_SECURE_SIZE,
			       true, NULL) < 0)
		goto fail;

	if (meson_add_rsv_node(fdt, "secure-monitor-shmem@200000",
			       G12A_LIBRE_SHARED_BASE, G12A_LIBRE_SHARED_SIZE,
			       true, NULL) < 0)
		goto fail;

	node = meson_add_rsv_node(fdt, "ramoops@300000",
				  G12A_LIBRE_RAMOOPS_BASE,
				  G12A_LIBRE_RAMOOPS_SIZE,
				  false, "ramoops");
	if (node < 0)
		goto fail;

	/*
	 * Documentation/devicetree/bindings/reserved-memory/ramoops.yaml:
	 * every zone size defaults to 0 and at least one must be non-zero, so
	 * a bare compatible+reg node does not bind.
	 *
	 * pmsg-size is separately load-bearing: fs/pstore/ram.c creates the
	 * pmsg zone only "if (cxt->pmsg_size)", so without it pstore/pmsg
	 * never registers and /dev/pmsg0 does not exist -- the console zone
	 * records fine, but userspace cannot write its own.  See the sizing
	 * and its provenance at GX_LIBRE_RAMOOPS_PMSG_SIZE in gx.h; the G12A
	 * constants are equal because the zone split is a property of the
	 * megabyte, not of where the megabyte sits.
	 */
	if (fdt_setprop_u32(fdt, node, "console-size",
			    G12A_LIBRE_RAMOOPS_CONSOLE_SIZE) ||
	    fdt_setprop_u32(fdt, node, "record-size",
			    G12A_LIBRE_RAMOOPS_RECORD_SIZE) ||
	    fdt_setprop_u32(fdt, node, "pmsg-size",
			    G12A_LIBRE_RAMOOPS_PMSG_SIZE) ||
	    fdt_setprop_u32(fdt, node, "ecc-size",
			    G12A_LIBRE_RAMOOPS_ECC_SIZE))
		goto fail;

	return;
fail:
	printf("meson: could not add the libre secure-layout reservations\n");
}
#endif /* CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT */

#if defined(CONFIG_MESON_G12A_LIBRE_SCMI_FIXUP)
/*
 * SM1 twin of MESON_GX_LIBRE_SCMI_FIXUP (board-gx.c): the -libre DTB carries
 * the open SC's SCMI package DISABLED (meson-sm1-libre-scmi.dtsi) and CPUs
 * with no OPP table.  Only the firmware knows whether an SC is answering, so
 * it says so here, on the blob handed to Linux, and only after ONE SCMI BASE
 * PROTOCOL_VERSION on the NS HP (OSPM) channel came back SUCCESS on this
 * boot.  The reasons each step is shaped as it is -- the ready bits as a
 * safety interlock and not a witness, the double bound on every wait, the
 * .data cache across relocation, asking silently and reporting later, and
 * deleting rather than disabling in U-Boot's own control FDT -- are measured
 * on GXL and argued at length in board-gx.c; they carry over unchanged.
 *
 * SC-DEPENDENT flips, made only when the SC answered:
 *
 *   /firmware/scmi           status -> okay
 *   /soc/mailbox@ff63c404    status -> okay
 *   /cpus/cpu@N  clocks        -> <protocol@14 0>
 *                power-domains -> <protocol@13 0>, power-domain-names "perf"
 *
 * There is nothing SC-INDEPENDENT on SM1: its Linux DT has no /scpi, and the
 * SC pets the AO watchdog itself.  A blob without the package (the eMMC's
 * vendor-leaf DTB) is left exactly as it arrived.  Nothing is created and no
 * phandle is minted: protocol@14/@13 carry labels, and the control DTB is
 * built with -@ (OF_LIBFDT_OVERLAY).
 *
 * Design: amlogic/software/boot/g12/sm1-linux-scmi-design.md section 3.
 */
#define G12A_SCMI_PATH		"/firmware/scmi"
#define G12A_SCMI_CLK_PATH	G12A_SCMI_PATH "/protocol@14"
#define G12A_SCMI_PERF_PATH	G12A_SCMI_PATH "/protocol@13"
#define G12A_SCMI_MHU_PATH	"/soc/mailbox@ff63c404"
#define G12A_SCMI_NR_CPUS	4

/*
 * AO RTI SCP status, BSP name AO_SEC_SD_CFG15.  The open SC paints bits
 * [21:20] = 3 LAST in hardware_init_hook(), after it seeds the heartbeat
 * block and ungates the MHU IRQs (scp-firmware master-lc 6698be5f,
 * product/amlogic/g12a/src/g12a_core.c g12a_paint_scp_ready();
 * include/g12a_regs.h:127-134).  Same register and test as
 * arch/arm/mach-meson/spl-g12a-scp.c.  It survives a warm reset, so it is
 * the interlock that keeps us out of a window nobody opened, never the
 * witness.
 */
#define G12A_AO_RTI_SCP_STAT	0xFF80023CUL
#define G12A_SCP_READY_SHIFT	20
#define G12A_SCP_READY_MASK	0x3U

/*
 * The OSPM SCMI channel: MHU_HIGH_AP_TO_SCP, the AHB SRAM window the SC
 * reads as its out-of-band mailbox (g12a_mmap.h:186-199,335-338; the DT's
 * cpu_scp_hpri, sram@fffe7000 + 0x600).
 */
#define G12A_SCMI_SHMEM_NS_HP	0xFFFE7600UL

/*
 * Meson MHU, channel 2 (HP): DT mailbox reg 0xFF63C404, ch2 RX at +0x18
 * and the AP's TX view at RX + 0x24 (drivers/mailbox/meson-mhu.c), i.e.
 * the SC's G12A_MHU_SCP_IN_HP doorbell 0xFF63C440 / STAT 0xFF63C444
 * (g12a_mmap.h:150-155).
 */
#define G12A_MHU_HP_RX		0xFF63C41CUL
#define G12A_MHU_HP_TX		(G12A_MHU_HP_RX + 0x24)
#define G12A_MHU_INTR_SET	0x0
#define G12A_MHU_INTR_STAT	0x4
#define G12A_MHU_INTR_CLR	0x8

/* SMT shared-memory header -- drivers/firmware/scmi/smt.h. */
#define G12A_SMT_CHAN_STATUS	0x04
#define G12A_SMT_FLAGS		0x10
#define G12A_SMT_LENGTH		0x14
#define G12A_SMT_MSG_HEADER	0x18
#define G12A_SMT_PAYLOAD	0x1c
#define G12A_SMT_CHAN_FREE	BIT(0)
#define G12A_SMT_CHAN_ERROR	BIT(1)
#define G12A_SMT_FLAG_INTR	BIT(0)

/* SCMI BASE, PROTOCOL_VERSION -- the message Linux sends first. */
#define G12A_SCMI_PROTO_BASE	0x10
#define G12A_SCMI_MSG_VERSION	0x00
#define G12A_SCMI_PROTO_SHIFT	10

#define G12A_SC_CHAN_IDLE_US	5000
#define G12A_SC_DOORBELL_US	30000	/* == meson-mhu's own TX bound */
#define G12A_SC_REPLY_US	50000	/* DT says arm,max-rx-timeout-ms = 30 */
#define G12A_SC_SPIN_MAX	1000000	/* reached only if the clock stands still */

static bool meson_g12a_sc_painted_ready(u32 *stat)
{
	*stat = readl((void *)G12A_AO_RTI_SCP_STAT);

	return ((*stat >> G12A_SCP_READY_SHIFT) & G12A_SCP_READY_MASK) ==
	       G12A_SCP_READY_MASK;
}

/* One SCMI BASE PROTOCOL_VERSION round trip, by hand (see board-gx.c). */
static int meson_g12a_sc_base_version(u32 *version)
{
	void *smt = (void *)G12A_SCMI_SHMEM_NS_HP;
	unsigned int spins;
	int ret = -ETIMEDOUT;
	ulong start;
	u32 st;

	start = timer_get_us();
	spins = 0;
	while (!(readl(smt + G12A_SMT_CHAN_STATUS) & G12A_SMT_CHAN_FREE)) {
		if (timer_get_us() - start >= G12A_SC_CHAN_IDLE_US ||
		    ++spins > G12A_SC_SPIN_MAX)
			return -EBUSY;
		udelay(10);
	}

	st = readl(smt + G12A_SMT_CHAN_STATUS);
	writel(st & ~G12A_SMT_CHAN_FREE, smt + G12A_SMT_CHAN_STATUS);
	writel(G12A_SMT_FLAG_INTR, smt + G12A_SMT_FLAGS);
	writel(sizeof(u32), smt + G12A_SMT_LENGTH);	/* msg_header only */
	writel((G12A_SCMI_PROTO_BASE << G12A_SCMI_PROTO_SHIFT) |
	       G12A_SCMI_MSG_VERSION, smt + G12A_SMT_MSG_HEADER);
	dsb();

	start = timer_get_us();
	spins = 0;
	while (readl((void *)G12A_MHU_HP_TX + G12A_MHU_INTR_STAT)) {
		if (timer_get_us() - start >= G12A_SC_DOORBELL_US ||
		    ++spins > G12A_SC_SPIN_MAX)
			goto out;
		udelay(1);
	}
	writel(1, (void *)G12A_MHU_HP_TX + G12A_MHU_INTR_SET);

	start = timer_get_us();
	spins = 0;
	for (;;) {
		st = readl(smt + G12A_SMT_CHAN_STATUS);
		if (st & G12A_SMT_CHAN_FREE)
			break;
		if (timer_get_us() - start >= G12A_SC_REPLY_US ||
		    ++spins > G12A_SC_SPIN_MAX)
			goto out;
		udelay(10);
	}

	if (st & G12A_SMT_CHAN_ERROR) {
		ret = -ECOMM;
		goto out;
	}
	/* msg_header + int32 status + uint32 version */
	if (readl(smt + G12A_SMT_LENGTH) < 3 * sizeof(u32)) {
		ret = -EPROTO;
		goto out;
	}
	if ((s32)readl(smt + G12A_SMT_PAYLOAD)) {		/* SCMI status */
		ret = -EPROTO;
		goto out;
	}

	*version = readl(smt + G12A_SMT_PAYLOAD + sizeof(u32));
	ret = 0;
out:
	/* Hand the channel on FREE, no error, and eat our own completion. */
	st = readl(smt + G12A_SMT_CHAN_STATUS);
	writel((st | G12A_SMT_CHAN_FREE) & ~G12A_SMT_CHAN_ERROR,
	       smt + G12A_SMT_CHAN_STATUS);
	st = readl((void *)G12A_MHU_HP_RX + G12A_MHU_INTR_STAT);
	if (st)
		writel(st, (void *)G12A_MHU_HP_RX + G12A_MHU_INTR_CLR);
	dsb();

	return ret;
}

/*
 * Asked ONCE per boot and remembered; all __section(".data") because the
 * first caller (board_fix_fdt) runs before relocation clears .bss.
 */
static int g12a_sc_verdict __section(".data") = -1;	/* -1 unasked, 0 mute, 1 answering */
static u32 g12a_sc_stat __section(".data");
static u32 g12a_sc_version __section(".data");
static bool g12a_sc_ready __section(".data");
static int g12a_sc_rc __section(".data");

static bool meson_g12a_sc_answers(void)
{
	if (g12a_sc_verdict >= 0)
		return g12a_sc_verdict == 1;

	g12a_sc_stat = 0;
	g12a_sc_version = 0;
	g12a_sc_rc = 0;
	g12a_sc_ready = meson_g12a_sc_painted_ready(&g12a_sc_stat);
	if (g12a_sc_ready)
		g12a_sc_rc = meson_g12a_sc_base_version(&g12a_sc_version);
	g12a_sc_verdict = g12a_sc_ready && !g12a_sc_rc;

	return g12a_sc_verdict == 1;
}

static void meson_g12a_sc_report(void)
{
	if (meson_g12a_sc_answers())
		printf("meson: scmi fixup: SC answered BASE PROTOCOL_VERSION v%u.%u (AO SCP_STAT 0x%08x)\n",
		       g12a_sc_version >> 16, g12a_sc_version & 0xffff,
		       g12a_sc_stat);
	else if (!g12a_sc_ready)
		printf("meson: scmi fixup: SC did not answer (AO SCP_STAT 0x%08x, not ready) -- SCMI package left off, CPUs left on the DTB's clocks\n",
		       g12a_sc_stat);
	else
		printf("meson: scmi fixup: SC did not answer (AO SCP_STAT 0x%08x ready, BASE PROTOCOL_VERSION rc %d) -- SCMI package left off, CPUs left on the DTB's clocks\n",
		       g12a_sc_stat, g12a_sc_rc);
}

enum {
	G12A_GUARD_NOT_RUN = 0,
	G12A_GUARD_NO_NODE,
	G12A_GUARD_ALREADY_OFF,
	G12A_GUARD_LEFT,
	G12A_GUARD_REMOVED,
	G12A_GUARD_FAILED,
};

static int g12a_sc_guard_action __section(".data") = G12A_GUARD_NOT_RUN;
static ulong g12a_sc_guard_us __section(".data");

static void meson_g12a_guard_report(void)
{
	static const char * const what[] = {
		[G12A_GUARD_NOT_RUN]	 = "did not run -- U-Boot's own control FDT was never checked",
		[G12A_GUARD_NO_NODE]	 = "absent from U-Boot's own control FDT -- nothing to guard",
		[G12A_GUARD_ALREADY_OFF] = "already disabled in U-Boot's own control FDT -- left as found",
		[G12A_GUARD_LEFT]	 = "SC live -- left enabled in U-Boot's own control FDT",
		[G12A_GUARD_REMOVED]	 = "no SC -- removed from U-Boot's own control FDT, its SCMI agent never bound",
		[G12A_GUARD_FAILED]	 = "could NOT be removed from U-Boot's own control FDT",
	};

	printf("meson: scmi guard: %s %s (query %lu us)\n", G12A_SCMI_PATH,
	       what[g12a_sc_guard_action], g12a_sc_guard_us);
}

static int meson_g12a_set_okay(void *fdt, const char *path)
{
	int off = fdt_path_offset(fdt, path);

	if (off < 0)
		return off;

	return fdt_setprop_string(fdt, off, "status", "okay");
}

static int meson_g12a_cpu_point_at_scmi(void *fdt, const char *cpu_path,
					u32 clk_ph, u32 perf_ph)
{
	fdt32_t spec[2];
	int off;

	off = fdt_path_offset(fdt, cpu_path);
	if (off < 0)
		return off;
	spec[0] = cpu_to_fdt32(clk_ph);
	spec[1] = cpu_to_fdt32(0);	/* CPU_CLK, the SC's clock 0 */
	if (fdt_setprop(fdt, off, "clocks", spec, sizeof(spec)))
		return -FDT_ERR_NOSPACE;

	off = fdt_path_offset(fdt, cpu_path);
	if (off < 0)
		return off;
	spec[0] = cpu_to_fdt32(perf_ph);
	spec[1] = cpu_to_fdt32(0);	/* the SC's single perf domain "CPU" */
	if (fdt_setprop(fdt, off, "power-domains", spec, sizeof(spec)))
		return -FDT_ERR_NOSPACE;

	off = fdt_path_offset(fdt, cpu_path);
	if (off < 0)
		return off;
	if (fdt_setprop_string(fdt, off, "power-domain-names", "perf"))
		return -FDT_ERR_NOSPACE;

	return 0;
}

int meson_g12a_libre_scmi_fixup(void *fdt)
{
	char cpu_path[sizeof("/cpus/cpu@0")];
	const char *missing = NULL;
	u32 clk_ph, perf_ph;
	int off, i;

	/* 1. Is the complete, labelled package here?  If not, touch nothing. */
	if (fdt_path_offset(fdt, G12A_SCMI_PATH) < 0)
		missing = G12A_SCMI_PATH;
	if (!missing && fdt_path_offset(fdt, G12A_SCMI_MHU_PATH) < 0)
		missing = G12A_SCMI_MHU_PATH;

	off = fdt_path_offset(fdt, G12A_SCMI_CLK_PATH);
	clk_ph = off < 0 ? 0 : fdt_get_phandle(fdt, off);
	off = fdt_path_offset(fdt, G12A_SCMI_PERF_PATH);
	perf_ph = off < 0 ? 0 : fdt_get_phandle(fdt, off);
	if (!missing && !clk_ph)
		missing = G12A_SCMI_CLK_PATH " (node or phandle)";
	if (!missing && !perf_ph)
		missing = G12A_SCMI_PERF_PATH " (node or phandle)";

	for (i = 0; !missing && i < G12A_SCMI_NR_CPUS; i++) {
		snprintf(cpu_path, sizeof(cpu_path), "/cpus/cpu@%d", i);
		if (fdt_path_offset(fdt, cpu_path) < 0)
			missing = "/cpus/cpu@N";
	}

	/* 2. Ask the SC -- only if the answer could change a write. */
	if (missing)
		printf("meson: scmi fixup: DTB lacks %s -- SCMI package left as is, SC not asked\n",
		       missing);
	else
		meson_g12a_sc_report();
	meson_g12a_guard_report();

	if (missing || !meson_g12a_sc_answers())
		return 0;

	/* 3. The SC-DEPENDENT flips. */
	if (meson_g12a_set_okay(fdt, G12A_SCMI_MHU_PATH))
		goto fail;
	printf("meson: scmi fixup: %s status -> okay\n", G12A_SCMI_MHU_PATH);

	if (meson_g12a_set_okay(fdt, G12A_SCMI_PATH))
		goto fail;
	printf("meson: scmi fixup: %s status -> okay\n", G12A_SCMI_PATH);

	for (i = 0; i < G12A_SCMI_NR_CPUS; i++) {
		snprintf(cpu_path, sizeof(cpu_path), "/cpus/cpu@%d", i);
		if (meson_g12a_cpu_point_at_scmi(fdt, cpu_path, clk_ph, perf_ph))
			goto fail;
		printf("meson: scmi fixup: %s clocks -> protocol@14 (phandle %u), power-domains -> protocol@13 (phandle %u) \"perf\"\n",
		       cpu_path, clk_ph, perf_ph);
	}

	return 0;
fail:
	printf("meson: scmi fixup: could not write the blob (out of space?)\n");
	return 0;
}

/*
 * U-Boot's OWN control FDT, from board_fix_fdt() in board_init_f: with no
 * SC, delete /firmware/scmi before DM binds the agent whose probe would
 * read the SC's shared-memory window.  With an SC, change nothing.
 */
int meson_g12a_libre_scmi_self_guard(void *fdt)
{
	const char *st;
	ulong t0;
	int off;

	off = fdt_path_offset(fdt, G12A_SCMI_PATH);
	if (off < 0) {
		g12a_sc_guard_action = G12A_GUARD_NO_NODE;
		return 0;
	}

	st = fdt_getprop(fdt, off, "status", NULL);
	if (st && strcmp(st, "okay") && strcmp(st, "ok")) {
		g12a_sc_guard_action = G12A_GUARD_ALREADY_OFF;
		return 0;
	}

	t0 = timer_get_us();
	if (meson_g12a_sc_answers()) {
		g12a_sc_guard_us = timer_get_us() - t0;
		g12a_sc_guard_action = G12A_GUARD_LEFT;
		return 0;
	}

	if (fdt_del_node(fdt, off)) {
		g12a_sc_guard_action = G12A_GUARD_FAILED;
		return 0;
	}

	g12a_sc_guard_us = timer_get_us() - t0;
	g12a_sc_guard_action = G12A_GUARD_REMOVED;

	return 0;
}
#endif /* CONFIG_MESON_G12A_LIBRE_SCMI_FIXUP */

/* Configure the reserved memory zones exported by the secure registers
 * into EFI and DTB reserved memory entries.
 */
void meson_init_reserved_memory(void *fdt)
{
	u64 bl31_size, bl31_start;
	u64 bl32_size, bl32_start;
	u32 reg;

	/*
	 * Get ARM Trusted Firmware reserved memory zones in :
	 * - AO_SEC_GP_CFG3: bl32 & bl31 size in KiB, can be 0
	 * - AO_SEC_GP_CFG5: bl31 physical start address, can be NULL
	 * - AO_SEC_GP_CFG4: bl32 physical start address, can be NULL
	 */
	reg = readl(G12A_AO_SEC_GP_CFG3);

	bl31_size = ((reg & G12A_AO_BL31_RSVMEM_SIZE_MASK)
			>> G12A_AO_BL31_RSVMEM_SIZE_SHIFT) * SZ_1K;
	bl32_size = (reg & G12A_AO_BL32_RSVMEM_SIZE_MASK) * SZ_1K;

	bl31_start = readl(G12A_AO_SEC_GP_CFG5);
	bl32_start = readl(G12A_AO_SEC_GP_CFG4);

#if defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT)
	/*
	 * The nodes are the MECHANISM; this /memreserve/ is defence in depth
	 * for a stale or foreign DTB reaching the kernel, and costs one entry.
	 * memblock takes the UNION of /memreserve/ and no-map nodes, so the two
	 * never conflict.  It also feeds the EFI memory map.
	 *
	 * It covers what FIRMWARE always owns -- the secure 2 MiB plus the
	 * mailboxes, 3 MiB from 0 -- and deliberately stops at ramoops; the
	 * OP-TEE band is Linux's unless OP-TEE runs.  A memreserve only keeps
	 * the OS from ALLOCATING there; it does not make a region usable as
	 * pstore.  So on a DTB that carries our ramoops node the extra MiB
	 * would be redundant, and on one that does not it would permanently
	 * sequester a MiB for a feature that blob cannot use.
	 */
	meson_add_libre_secure_reservation(fdt);
	meson_board_add_reserved_memory(fdt, G12A_LIBRE_FW_RSV_BASE,
					G12A_LIBRE_FW_RSV_SIZE);
	meson_libre_optee_fixup(fdt);
#endif

	/* Add BL31 reserved zone */
	if (bl31_start && bl31_size)
		meson_board_add_reserved_memory(fdt, bl31_start, bl31_size);

	/* Add BL32 reserved zone */
	if (bl32_start && bl32_size)
		meson_board_add_reserved_memory(fdt, bl32_start, bl32_size);

#if defined(CONFIG_VIDEO_MESON)
	meson_vpu_rsv_fb(fdt);
#endif
}

phys_size_t get_effective_memsize(void)
{
	phys_size_t val = readl(G12A_AO_SEC_GP_CFG0);
	/* Size is reported in MiB, convert it in bytes */
	return min(((val & G12A_AO_MEM_SIZE_MASK)
			>> G12A_AO_MEM_SIZE_SHIFT) * SZ_1M, (phys_size_t)0xf5000000);
}

static struct mm_region g12a_mem_map[] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0xf5000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0xf5000000UL,
		.phys = 0xf5000000UL,
		.size = 0x0b000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = g12a_mem_map;
