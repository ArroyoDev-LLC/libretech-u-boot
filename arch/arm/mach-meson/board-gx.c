// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 * (C) Copyright 2018 Neil Armstrong <narmstrong@baylibre.com>
 */

#include <errno.h>
#include <init.h>
#include <net.h>
#include <time.h>
#include <dm/of.h>
#include <fdt_support.h>
#include <fdtdec.h>
#include <asm/arch/boot.h>
#include <asm/arch/eth.h>
#include <asm/arch/gx.h>
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
#include <vsprintf.h>

int meson_get_boot_device(void)
{
	return readl(GX_AO_SEC_GP_CFG0) & GX_AO_BOOT_DEVICE;
}

#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT)
/*
 * The open TF-A puts its firmware in the first 16 MiB of DRAM (BL31 at
 * 0x00100000, mailboxes at 0x00200000, ramoops at 0x00300000, the optional
 * OP-TEE band at 0x00400000 -- arch/arm/include/asm/arch-meson/gx.h and
 * plat/amlogic/gxl/gxl_def.h).
 *
 * The -libre board DTS describes this block itself (meson-gx-libre-
 * firmware.dtsi); the vendor build of the same board boots the unsuffixed
 * DTS, which carries the vendor secmon@5000000 instead.  This runs from
 * ft_board_setup(), i.e. on whatever blob is actually being booted, so a
 * libre board handed a PXE-supplied or distro DTB of the vendor layout is
 * still corrected.  On the -libre DTB it is a no-op: every node it writes is
 * already there under the same name.
 *
 * It must create reserved-memory NODES, not merely an FDT /memreserve/.
 * A memreserve becomes a memblock_reserve, which leaves the region in the
 * kernel's LINEAR MAP; only a no-map node excludes it.  For TZASC-secured
 * memory that is the difference between safe and reachable by a speculative
 * access, which is an external abort.  (dram-reservations.md section 10, D1.)
 *
 * THREE nodes, because the block is not one thing, plus the SoC .dtsi's own
 * hwrom@0 for the ROM mirror, which is kept:
 *
 *   secure-firmware@100000   0x00100000 +  1 MiB  no-map   BL31 + its SCP
 *                                                          staging tail
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
 * meson_sm_map_shmem() (drivers/firmware/meson/meson_sm.c) learns the base by
 * SMC and maps it with ioremap_cache(), and arm64's __ioremap_prot() refuses
 * any pfn_is_map_memory() page (arch/arm64/mm/ioremap.c) -- i.e. anything left
 * in the linear map.  Unreserved, or reserved-but-mapped, the mapping WARNs
 * and returns NULL and the secure-monitor driver fails to probe.  Upstream
 * gets this for free: today's mailboxes at 0x050F8000 sit inside the no-map
 * secmon@5000000 (arch/arm64/boot/dts/amlogic/meson-gx.dtsi).
 *
 * ramoops is the opposite: it is the OS's own persistent storage, needs a
 * compatible so a driver DOES bind, and stays mapped.  Its default mem-type 0
 * makes fs/pstore/ram_core.c map it write-combined either way, so leaving it
 * in the linear map costs nothing in cache-flush terms.
 *
 * meson_del_node(), meson_del_ramoops_nodes() and meson_add_rsv_node() moved
 * to board-common.c when G12A/G12B/SM1 gained the same layout
 * (CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT).  The generic /reserved-memory
 * mechanics are identical between the two families; the ramoops zone
 * properties are set inline by each family, so the sizing constants stay
 * visible at the callsite rather than behind a row of positional u32 args.
 */
static void meson_add_libre_secure_reservation(void *fdt)
{
	int rmem, node;

	/*
	 * All of these are wrong for this firmware, and all are deleted
	 * BEFORE the nodes are added: fdt_del_node() invalidates offsets, so
	 * the /reserved-memory lookup below has to happen afterwards.
	 *
	 *   secmon@5000000           -- BL31 is not there in this build.
	 *   secure-firmware@0,
	 *   secure-monitor-shmem@e00000
	 *                            -- the block before the 2026-10-17
	 *                               reorder (14 MiB secure, mailboxes at
	 *                               0x00E00000), as an older libre DTB
	 *                               still carries them.
	 *   ramoops@*                -- see meson_del_ramoops_nodes().
	 *
	 * hwrom@0 stays: the ROM mirror is a hardware fact on every chain,
	 * and secure-firmware@100000 starts where it ends, so they do not
	 * overlap.
	 */
	meson_del_node(fdt, "/reserved-memory/secmon@5000000");
	meson_del_node(fdt, "/reserved-memory/secure-firmware@0");
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

	if (meson_add_rsv_node(fdt, "secure-firmware@100000",
			       GX_LIBRE_BL31_BASE, GX_LIBRE_BL31_SIZE,
			       true, NULL) < 0)
		goto fail;

	if (meson_add_rsv_node(fdt, "secure-monitor-shmem@200000",
			       GX_LIBRE_SHARED_BASE, GX_LIBRE_SHARED_SIZE,
			       true, NULL) < 0)
		goto fail;

	node = meson_add_rsv_node(fdt, "ramoops@300000",
				  GX_LIBRE_RAMOOPS_BASE, GX_LIBRE_RAMOOPS_SIZE,
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
	 * and its provenance at GX_LIBRE_RAMOOPS_PMSG_SIZE in gx.h.
	 */
	if (fdt_setprop_u32(fdt, node, "console-size",
			    GX_LIBRE_RAMOOPS_CONSOLE_SIZE) ||
	    fdt_setprop_u32(fdt, node, "record-size",
			    GX_LIBRE_RAMOOPS_RECORD_SIZE) ||
	    fdt_setprop_u32(fdt, node, "pmsg-size",
			    GX_LIBRE_RAMOOPS_PMSG_SIZE) ||
	    fdt_setprop_u32(fdt, node, "ecc-size",
			    GX_LIBRE_RAMOOPS_ECC_SIZE))
		goto fail;

	return;
fail:
	printf("meson: could not add the libre secure-layout reservations\n");
}
#endif /* CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT */

#if defined(CONFIG_MESON_GX_LIBRE_SCMI_FIXUP)
/*
 * The shipping kernel DTS carries the open-SC SCMI package DISABLED and the
 * vendor SCPI node enabled, because the vendor build of the same board boots
 * the same DTB against a BL30 that speaks SCPI on the same mailbox.  Only the
 * firmware knows which protocol the channel carries, so it says so here, on
 * the blob it hands Linux -- the same "firmware describes itself" reasoning as
 * meson_add_libre_secure_reservation() above.
 *
 * The flips fall into two sets, and the split is the whole point of the guard
 * below: SC-DEPENDENT flips are only made once an SC has ANSWERED on this
 * boot; SC-INDEPENDENT flips are made either way.
 *
 * SC-DEPENDENT -- load-bearing TOGETHER (measured on AML-S905X-CC-V2,
 * amlogic/software/boot/gxl/design/open-sc-architecture.md sections 3-4):
 *
 *   /firmware/scmi        status -> okay      the agent Linux binds
 *   /cpus/cpu@N clocks    -> protocol@14      the OPP core needs a usable clock
 *               power-domains -> protocol@13  scmi-cpufreq matches by this
 *
 * One SC-DEPENDENT flip is not answered-by the BASE reply alone, and is shut
 * before this guard runs at all -- every meson blob has it disabled at the top
 * of ft_board_setup(), so the incoming blob's own status cannot carry it into
 * Linux.  It returns to okay only on a SECOND question, meson_gx_pubmon():
 *
 *   amlogic,gxl-pubmon-   status -> disabled  every meson build, first write
 *   sc-shmem                                  to the blob, outside this Kconfig
 *                       status -> okay        only on a live PUBMON_GET_BLOCK
 *                                             (protocol 0x80): running, and
 *                                             publishing the very block this
 *                                             node's reg names
 *
 * The SC's BASE answer says the mailbox is live; it says nothing about the
 * sampler that reads the DDR PHY, nor about the KiB Linux would be told to
 * read.  So that node is earned twice, and neither witness is something the SC
 * left in SRAM -- share SRAM survives a warm reset, and a record in it says
 * nothing about the SC answering now.  Design: amlogic/software/boot/gxl/dram/
 * design/pub-mon-sc-architecture.md section 2a, "The production gate: a live
 * SCMI answer from this boot's SC".
 *
 * SC-INDEPENDENT, made either way:
 *
 *   /scpi                 status -> disabled  /scpi describes the VENDOR BL30's
 *                                             service on this mailbox, and a
 *                                             -libre image never loads a vendor
 *                                             BL30 -- so the node is false on
 *                                             this build whatever the SC did.
 *                                             Left enabled with no SC at all,
 *                                             Linux's SCPI driver reaches the
 *                                             same closed window the SCMI one
 *                                             does and takes the same abort
 *                                             (scpi_process_cmd, measured
 *                                             2026-08-30).
 *   AO watchdog           status -> okay      the ROM arms it before it loads
 *                                             anything; U-Boot refreshes it
 *                                             while it runs and, by default,
 *                                             stops it before the OS
 *                                             (MESON_GXL_AO_WDT_HANDOFF);
 *                                             wdt_handoff=1 hands it over
 *                                             running, for a kernel known to
 *                                             carry the driver.  Nothing about
 *                                             the node involves the SC
 *   amlogic,meson-sram-   status -> okay      the AO SRAM pool it writes is
 *   pstore                                    this image's to lay out; on the
 *                                             vendor chain the same SRAM is
 *                                             the vendor firmware's and the
 *                                             DTB ships the node disabled
 *                                             (AML-S805X-AC dies ~3 s into
 *                                             kernel boot with it enabled,
 *                                             measured 2026-10-11)
 *
 * And two more SC-INDEPENDENT flips, which describe what VENDOR's BL31 offers
 * and this one does not.  Which BL31 is running is settled by the image, not by
 * whether the SC came up, so these are made either way (both OPTIONAL -- a blob
 * without them is still complete):
 *
 *   amlogic,meson-gx-pm   status -> disabled  drivers/firmware/meson/
 *                                             meson_gx_pm.c registers
 *                                             suspend_ops AFTER psci's and
 *                                             wins: `echo mem` becomes a raw
 *                                             PSCI CPU_SUSPEND(0x0010000) whose
 *                                             return leaks unmapped to
 *                                             userspace (measured 2026-08-29:
 *                                             -2, INVALID_PARAMS, seen as
 *                                             ENOENT) and PSCI SYSTEM_SUSPEND
 *                                             is never sent.  Disabled, psci's
 *                                             own suspend_ops stand.
 *   arm,idle-state        status -> disabled  CPU_SLEEP_0 asks for a POWERDOWN
 *                                             idle (0x0010000) the open SC has
 *                                             no wake path for; refused, Linux
 *                                             retries it every ~18 us and the
 *                                             idle loop never reaches wfi
 *                                             (measured, kretprobe on
 *                                             cpu_suspend: -EINVAL back to back
 *                                             on the idle task).  Disabled,
 *                                             cpuidle keeps only arm64's
 *                                             built-in WFI state.
 *
 * The SCMI package must be COMPLETE before any SC-DEPENDENT flip is made: a
 * blob carrying only part of it (an old DTB, U-Boot's own control FDT) keeps
 * the SCMI package it arrived with, so the vendor-DTB path never sees a
 * half-flipped agent.  That completeness gate covers the SCMI package ONLY --
 * each SC-INDEPENDENT flip above is guarded by its own node's presence, so a
 * DTB with /scpi and no SCMI package (the AML-S905X-CC v1's, and every DTB
 * predating the package) still gets /scpi turned off.  Nothing is
 * created and no phandle is minted -- the DTS gives protocol@14 and
 * protocol@13 labels, and the kernel's dtc emits phandles for labelled nodes
 * (-@ under CONFIG_OF_OVERLAY).  A blob whose protocol nodes carry no phandle
 * is reported as missing, not repaired.
 *
 * libfdt offsets are invalidated by any setprop that changes the struct
 * block, so every node is re-looked-up by path right before it is written.
 */
#define GX_SCMI_PATH		"/firmware/scmi"
#define GX_SCMI_CLK_PATH	GX_SCMI_PATH "/protocol@14"
#define GX_SCMI_PERF_PATH	GX_SCMI_PATH "/protocol@13"
#define GX_SCPI_PATH		"/scpi"
#define GX_AO_WDT_COMPAT	"amlogic,meson-gxl-ao-wdt"
#define GX_SRAM_PSTORE_COMPAT	"amlogic,meson-sram-pstore"
#define GX_PM_COMPAT		"amlogic,meson-gx-pm"
#define GX_IDLE_STATE_COMPAT	"arm,idle-state"
#define GX_NR_CPUS		4

/*
 * The PUB-telemetry node, whose compatible is shared with the shut that
 * board-common.c applies to every meson blob (asm/arch/mem.h).  Nothing here
 * names the block: no SRAM address, no magic word, no offset in the SC's report
 * map.  The SC answers with the address and size it is actually publishing, and
 * they are checked against the node's own reg -- which is the whole point of
 * asking instead of remembering, since share SRAM survives a warm reset and a
 * record left in it says nothing about the SC answering now.  Design:
 * amlogic/software/boot/gxl/dram/design/pub-mon-sc-architecture.md section 2a,
 * "The production gate: a live SCMI answer from this boot's SC".
 */
#define GX_PUBMON_COMPAT	MESON_PUBMON_COMPAT

/*
 * ---------------------------------------------------------------------------
 * The guard: does an SC actually ANSWER?
 * ---------------------------------------------------------------------------
 *
 * Everything above is a statement about the SC, and until 2026-08-30 this code
 * made all of it without ever asking the SC anything.  That is not a cosmetic
 * gap.  Measured on AML-S905X-CC-V2 with a u-boot built from a branch that
 * carries this fixup but NOT the DRAM SPL's SC loader (audit
 * amlogic/software/boot/gxl/audit/2026-08-30-gxl-n42-boot-from-lc-master.md):
 * BL31 prints "SCP never became ready; not uploading; SCMI/SCPI unavailable",
 * this function flips /firmware/scmi to "okay" anyway, and Linux's very first
 * SCMI transaction dies --
 *
 *   Internal error: synchronous external abort: 0000000096000210 [#1] SMP
 *   Tainted: [M]=MACHINE_CHECK
 *   pc : shmem_tx_prepare+0x50/0x428
 *      version_get / scmi_base_protocol_init / scmi_probe
 *
 * -- because the SC never ran hardware_init_hook(), so the non-secure window
 * onto its mailbox was never opened.  Userspace is never reached.  Enabling a
 * node is an ASSERTION ABOUT HARDWARE, and this one has to be earned.
 *
 * WHAT COUNTS AS EARNING IT.  Only a reply the SC composed on THIS boot.  The
 * things that look like witnesses and are not:
 *
 *   - "the loader is compiled in" (CONFIG_MESON_GX_XPL_CHAIN): a build-time
 *     fact.  It is true on a board whose SC payload never started.
 *   - "U-Boot has an SCMI agent" (CONFIG_SCMI_FIRMWARE): likewise.  The agent
 *     existing is not the agent succeeding.
 *   - the SC ready bits, or any other word somebody painted in memory: those
 *     live in the always-on domain and survive a warm reset, so a stale one
 *     answers for the PREVIOUS boot.  This arc has been bitten by exactly this
 *     class twice already -- see instrument-liveness.md, and the SENSOR channel
 *     that "read" a pin that was not routed.
 *
 * So the witness is a live SCMI BASE PROTOCOL_VERSION round trip on the
 * non-secure OSPM channel: we compose the request, ring the SC's doorbell, and
 * require the SC to put the channel back with a SUCCESS status and a version
 * word.  That is the same protocol, the same channel and the same first
 * message that Linux's scmi_base_protocol_init() sends, so it is not a proxy
 * for the thing that faults -- it IS the thing that faults, executed early
 * enough that failing it costs a DT flip instead of the kernel.
 *
 * ORDER MATTERS, and the ready bits earn their place here as a SAFETY
 * interlock rather than as the witness.  A filtered non-secure WRITE to this
 * SRAM does not abort synchronously: it raises an SError that reaches
 * do_error() and resets the board (measured 2026-08-26, audit
 * 2026-08-26-gxl-share-sram-ns-aperture.md, boot W1).  So we must not compose a
 * message into a window that is still closed.  The SC opens that window
 * (AO_SEC_SHARED_AHB_SRAM_REG3_2 = 0x3000) and paints the ready bits from the
 * SAME function, hardware_init_hook(), and both live in the same always-on
 * block, so they are retained and cleared together: ready bits clear => the
 * window is closed => we touch nothing, which is precisely the shipping
 * failure mode (no loader, no payload, or a ROM that refused the image).
 *
 * BOUNDED, ALWAYS.  Worst case is 5 ms waiting for an idle channel + 30 ms for
 * the doorbell to be accepted + 50 ms for the reply = 85 ms, and it is 0 ms in
 * the case this guard exists for, because a board with no SC never gets past
 * the ready-bit read.  Nothing here can block.
 */

/*
 * AO RTI SCP status.  Secure alias AML_AO_RTI_SCP_STAT = 0xDA10023C in TF-A
 * plat/amlogic/gxl/gxl_def.h:186; this is its non-secure spelling, the same
 * one arch/arm/mach-meson/spl-gxl-scp.c:55 uses.  "Ready" is bits [21:20] both
 * set (AML_AO_RTI_SCP_READY_OFF 0x14 / AML_A0_RTI_SCP_READY_MASK 3,
 * gxl_def.h:187-189) -- the identical test BL31's gxl_scp_ready() makes.
 */
#define GX_AO_RTI_SCP_STAT	0xC810023CUL
#define GX_SCP_READY_SHIFT	20
#define GX_SCP_READY_MASK	0x3U

/*
 * The OSPM SCMI mailbox, non-secure spelling.  scp-firmware
 * product/amlogic/gxl/scp_ramfw/config_transport.c gives the OSPM channel
 * out_band_mailbox_address = GXL_SCMI_PAYLOAD_NS_HP (gxl_mmap.h) = the AHB
 * share SRAM at 0xC8000000 + 0x13400.  Same node the DT hands Linux
 * (cpu_scp_hpri, meson-gx.dtsi sram@c8000000) and the same one U-Boot's own
 * agent uses (arch/arm/dts/meson-gxl-scmi-u-boot.dtsi).
 */
#define GX_SCMI_SHMEM_NS_HP	0xC8013400UL

/*
 * Meson MHU doorbell, channel 2 (HP / OSPM).  Base 0xC883C404 is the DT
 * mailbox reg (hiubus + 0x404); drivers/mailbox/meson-mhu.c puts ch2 at
 * +0x18 and the AP's TX view at RX + 0x24, each channel being
 * SET/+0 STAT/+4 CLR/+8.
 */
#define GX_MHU_HP_RX		0xC883C41CUL
#define GX_MHU_HP_TX		(GX_MHU_HP_RX + 0x24)
#define GX_MHU_INTR_SET		0x0
#define GX_MHU_INTR_STAT	0x4
#define GX_MHU_INTR_CLR		0x8

/* SMT shared-memory header -- drivers/firmware/scmi/smt.h. */
#define GX_SMT_CHAN_STATUS	0x04
#define GX_SMT_FLAGS		0x10
#define GX_SMT_LENGTH		0x14
#define GX_SMT_MSG_HEADER	0x18
#define GX_SMT_PAYLOAD		0x1c
#define GX_SMT_CHAN_FREE	BIT(0)
#define GX_SMT_CHAN_ERROR	BIT(1)
#define GX_SMT_FLAG_INTR	BIT(0)

/* SCMI BASE, PROTOCOL_VERSION -- the message Linux sends first. */
#define GX_SCMI_PROTO_BASE	0x10
#define GX_SCMI_MSG_VERSION	0x00
#define GX_SCMI_PROTO_SHIFT	10

/*
 * The platform protocol the sampler is COMPILED IN WITH, and the one message
 * this guard needs from it.  An SC built without the sampler has no protocol
 * 0x80 at all and answers NOT_SUPPORTED, which is a different answer from
 * "protocol present, sampler not running" -- so they are asked apart and
 * printed apart.  Design: amlogic/software/boot/gxl/dram/design/
 * pub-mon-sc-architecture.md section 2a, "The production gate".
 */
#define GX_SCMI_PROTO_PUBMON	0x80	/* "GXL PUB telemetry" */
#define GX_PUBMON_MSG_GET_BLOCK	0x03	/* no parameters */
#define GX_SCMI_STATUS_OK	0
#define GX_SCMI_STATUS_NOT_SUPPORTED	(-1)
/*
 * Not an SCMI status -- the spec's error codes stop well short of this.  The
 * transaction starts from it, so a status that was never read prints as
 * something nobody can mistake for the SC saying "NOT_SUPPORTED" (-1).
 */
#define GX_SCMI_STATUS_NONE	(-999)
#define GX_PUBMON_F_RUNNING	BIT(0)	/* permit read back, samples >= 1 */
#define GX_PUBMON_BLOCK_VERSION	1

/* PUBMON_GET_BLOCK's return, as the SC lays it into the SMT payload. */
struct gx_pubmon_block {
	s32 status;
	u32 flags;
	u32 block_ns_addr;
	u32 block_size;
	u32 block_version;
};

/* The SCMI spec's status codes (DEN0056, "Status codes"), by name. */
static const char *meson_gx_scmi_status_name(s32 status)
{
	static const char *const names[] = {
		"SUCCESS", "NOT_SUPPORTED", "INVALID_PARAMETERS", "DENIED",
		"NOT_FOUND", "OUT_OF_RANGE", "BUSY", "COMMS_ERROR",
		"GENERIC_ERROR", "HARDWARE_ERROR", "PROTOCOL_ERROR",
	};

	if (status <= 0 && -status < (s32)ARRAY_SIZE(names))
		return names[-status];

	return "unknown status";
}

#define GX_SC_CHAN_IDLE_US	5000
#define GX_SC_DOORBELL_US	30000	/* == meson-mhu's own TX bound */
#define GX_SC_REPLY_US		50000	/* DT says arm,max-rx-timeout-ms = 30 */

static bool meson_gx_sc_painted_ready(u32 *stat)
{
	*stat = readl((void *)GX_AO_RTI_SCP_STAT);

	return ((*stat >> GX_SCP_READY_SHIFT) & GX_SCP_READY_MASK) ==
	       GX_SCP_READY_MASK;
}

/*
 * A wait bounded ONLY by a clock is bounded by nothing when the clock is not
 * running, and this arc has already paid for that once: every arch_cnt()
 * deadline in the DRAM SPL's SC loader was inert because CNTFRQ_EL0 reads 0 in
 * that stage, so the real bound of each poll was an iteration count nobody had
 * chosen (audit 2026-08-30-gxl-n55-sc-wait-bound.md).  This transaction is now
 * also run from board_init_f (meson_gx_libre_scmi_self_guard(), below), i.e.
 * before relocation and before initr_dm, so it must not assume that whatever
 * makes timer_get_us() advance has already happened there.
 *
 * Every wait below therefore carries BOTH bounds: the microsecond deadline,
 * which is the real one, and an iteration cap that can only be reached if the
 * clock is standing still.  The cap is far above any iteration count a live
 * clock produces -- at the 10 us and 1 us delays used here it is ~10 s and ~1 s
 * of real time -- so on a working timer it is unreachable and changes nothing.
 * The self-guard reports the microseconds the whole query took, which is the
 * standing witness that the clock is in fact live at that call site.
 */
#define GX_SC_SPIN_MAX		1000000

/*
 * One SCMI round trip, by hand: <proto, msg>, no input payload, and n_out
 * reply words read back after the SCMI status word.
 *
 * Deliberately not routed through UCLASS_SCMI_AGENT.  That path decides at DM
 * BIND time, long before ft_board_setup(), and it composes the message into
 * shared memory before anything has established that the window is open -- on
 * this SoC that is the SError above, i.e. a reset.  The guard has to own the
 * ordering, so it owns the transaction.  Each call is one message, it has no
 * side effects on the SC, and it leaves the channel FREE for the next owner --
 * except after a reply timeout, when the SC still owns the message and the
 * channel stays BUSY until the SC answers.
 *
 * @status is the SCMI return word, and is meaningful ONLY when this returns 0:
 * a non-zero return says the transaction itself failed, which the callers print
 * as a different answer than a "no" from the SC.  It starts at
 * GX_SCMI_STATUS_NONE -- a value no SCMI status can be -- so a failure can
 * never be read as an answer, not even as NOT_SUPPORTED.
 */
static int meson_gx_sc_xfer(u32 proto, u32 msg, s32 *status, u32 *out,
			    unsigned int n_out)
{
	void *smt = (void *)GX_SCMI_SHMEM_NS_HP;
	unsigned int spins, i, len;
	int ret = -ETIMEDOUT;
	ulong start;
	u32 st, hdr_req, hdr;

	*status = GX_SCMI_STATUS_NONE;
	hdr_req = (proto << GX_SCMI_PROTO_SHIFT) | msg;

	/* The channel is the SC's to initialise; wait for it to be idle. */
	start = timer_get_us();
	spins = 0;
	while (!(readl(smt + GX_SMT_CHAN_STATUS) & GX_SMT_CHAN_FREE)) {
		if (timer_get_us() - start >= GX_SC_CHAN_IDLE_US ||
		    ++spins > GX_SC_SPIN_MAX)
			return -EBUSY;
		udelay(10);
	}

	/*
	 * Take it, and compose <proto, msg> (no input payload).  FLAGS = 0 asks
	 * for POLLED completion: the SC rings the completion doorbell only when
	 * the agent set FLAG_INTR (mod_transport.c, transport_respond()), and
	 * this transaction polls CHAN_STATUS anyway -- so no doorbell of ours
	 * can ever be left pending for Linux, not even by a reply that arrives
	 * after we stopped waiting (landed review U1).
	 */
	st = readl(smt + GX_SMT_CHAN_STATUS);
	writel(st & ~GX_SMT_CHAN_FREE, smt + GX_SMT_CHAN_STATUS);
	writel(0, smt + GX_SMT_FLAGS);
	writel(sizeof(u32), smt + GX_SMT_LENGTH);	/* msg_header only */
	writel(hdr_req, smt + GX_SMT_MSG_HEADER);
	dsb();

	/* Ring the SC.  Any non-zero SET wakes the link (meson-mhu.c). */
	start = timer_get_us();
	spins = 0;
	while (readl((void *)GX_MHU_HP_TX + GX_MHU_INTR_STAT)) {
		if (timer_get_us() - start >= GX_SC_DOORBELL_US ||
		    ++spins > GX_SC_SPIN_MAX)
			goto out;	/* never rung: the channel is ours to free */
		udelay(1);
	}
	writel(1, (void *)GX_MHU_HP_TX + GX_MHU_INTR_SET);

	/* The SC answers by writing the response and putting the channel back. */
	start = timer_get_us();
	spins = 0;
	for (;;) {
		st = readl(smt + GX_SMT_CHAN_STATUS);
		if (st & GX_SMT_CHAN_FREE)
			break;
		if (timer_get_us() - start >= GX_SC_REPLY_US ||
		    ++spins > GX_SC_SPIN_MAX) {
			/*
			 * Rung and not answered.  The message is the SC's now,
			 * and it may still reply: marking the channel FREE here
			 * would let the next agent compose over a reply in
			 * flight.  Leave it BUSY -- the SC frees it when it
			 * answers, and an agent that finds it busy waits or
			 * gives up rather than racing (landed review U2).
			 */
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	if (st & GX_SMT_CHAN_ERROR) {
		ret = -ECOMM;
		goto out;
	}

	/*
	 * Is this answer OUR question?  The SCMI message_header is message id
	 * in [7:0], message type in [9:8], protocol id in [17:10] and the token
	 * in [31:18] -- drivers/firmware/scmi/smt.h says the same thing in
	 * code (SMT_HEADER_MESSAGE_ID / _MESSAGE_TYPE / _PROTOCOL_ID / _TOKEN)
	 * and GX_SCMI_PROTO_SHIFT is that [17:10] shift.  The type here is 0, a
	 * normal command, and the token we send is 0, so a reply word equal to
	 * the one we wrote is every field agreeing.  Anything else is somebody
	 * else's reply and counts as no answer at all: an answer to a different
	 * message is not evidence about this one.
	 */
	hdr = readl(smt + GX_SMT_MSG_HEADER);
	len = readl(smt + GX_SMT_LENGTH);
	if (hdr != hdr_req) {
		ret = -EREMOTEIO;
		goto out;
	}

	/*
	 * Read the STATUS before requiring a PAYLOAD.  An SC built without the
	 * sampler has no protocol 0x80 and answers with nothing but a header
	 * and a non-SUCCESS status; demanding the full payload first would turn
	 * a clean "no" into a protocol error, and the caller would print
	 * "did not answer" where it should print "no PUB sampler protocol".
	 */
	if (len < 2 * sizeof(u32)) {
		ret = -EPROTO;
		goto out;
	}

	*status = (s32)readl(smt + GX_SMT_PAYLOAD);

	if (*status == GX_SCMI_STATUS_OK) {
		/* msg_header + int32 status + n_out payload words */
		if (len < (2 + n_out) * sizeof(u32)) {
			ret = -EPROTO;
			goto out;
		}

		for (i = 0; i < n_out; i++)
			out[i] = readl(smt + GX_SMT_PAYLOAD +
				       (i + 1) * sizeof(u32));
	}

	ret = 0;
out:
	/*
	 * Reached only when nothing of ours is still with the SC: it answered,
	 * or it was never rung.  Hand the channel on as we found it -- FREE, no
	 * error -- so Linux's agent starts clean.  There is no completion
	 * doorbell to consume: FLAGS = 0 above means the SC never rings one.
	 */
	st = readl(smt + GX_SMT_CHAN_STATUS);
	writel((st | GX_SMT_CHAN_FREE) & ~GX_SMT_CHAN_ERROR,
	       smt + GX_SMT_CHAN_STATUS);
	dsb();

	return ret;
}

/* BASE PROTOCOL_VERSION: an answer with status SUCCESS is the whole question. */
static int meson_gx_sc_base_version(u32 *version)
{
	s32 status = GX_SCMI_STATUS_NONE;
	u32 out[1];
	int ret = meson_gx_sc_xfer(GX_SCMI_PROTO_BASE, GX_SCMI_MSG_VERSION,
				   &status, out, 1);

	if (ret)
		return ret;
	if (status)
		return -EPROTO;

	*version = out[0];

	return 0;
}

/*
 * PUBMON_GET_BLOCK: the SC's one-stop answer about its own sampler.  A
 * transport failure here is not a "no" from the SC -- the two have to be
 * printed apart, because one means the sampler is absent and the other means we
 * never got to ask.
 */
static int meson_gx_sc_pubmon_block(struct gx_pubmon_block *blk)
{
	/*
	 * Zeroed because the transaction fills it ONLY when the SC says
	 * SUCCESS: an SC that answers "no such protocol" sends header and
	 * status and no payload at all, and the fields then have to read as
	 * absent rather than as whatever the stack had lying on it.
	 */
	u32 out[4] = { 0 };
	int ret;

	ret = meson_gx_sc_xfer(GX_SCMI_PROTO_PUBMON, GX_PUBMON_MSG_GET_BLOCK,
			       &blk->status, out, 4);
	if (ret)
		return ret;

	blk->flags = out[0];
	blk->block_ns_addr = out[1];
	blk->block_size = out[2];
	blk->block_version = out[3];

	return 0;
}

/*
 * The verdict is asked ONCE per boot and remembered.
 *
 * Two blobs need it now -- U-Boot's own control FDT, guarded in board_init_f
 * (meson_gx_libre_scmi_self_guard(), below), and the blob handed to Linux,
 * flipped in ft_board_setup().  They are two stages apart, and the SC is a
 * second processor whose state is not ours: asking it twice would be two
 * different questions with two possible answers, and would put a second
 * message on a channel whose whole hazard is who touches it when.  ONE BASE
 * TRANSACTION PER BOOT, then -- the cached verdict is what keeps that true.
 * The PUB sampler question (2a) is a second message by design and is NOT
 * cached: a BASE reply is evidence about the mailbox, and the sampler's answer
 * has to be this boot's.
 *
 * ASKING AND REPORTING ARE SEPARATE, and that is not tidiness.  The two call
 * sites run at stages with different consoles: this defconfig carries
 * CONFIG_SILENT_CONSOLE_UNTIL_ENV=y and no CONFIG_PRE_CONSOLE_BUFFER, so
 * console_update_silent() holds GD_FLG_SILENT until the environment is read in
 * board_init_r.  EVERY printf() from board_init_f -- where the guard runs -- is
 * discarded, and nothing replays it.
 *
 * Measured 2026-08-30, on the first silicon boot of this guard: the console
 * showed neither the guard's own line NOR the "SC did not answer" line
 * ft_board_setup() had printed since the fixup shipped.  Caching the verdict at
 * a silent call site had consumed the only call that printed it -- so the guard
 * arrived with no witness of its own AND took an existing one with it.  The
 * guard was in fact working (U-Boot's control FDT answered FDT_ERR_NOTFOUND for
 * /firmware/scmi, and no SCMI agent bound), but the console could not say so,
 * and an instrument that cannot be read is not an instrument.
 *
 * So: ask silently, keep the facts, and let each call site report from them.
 * ft_board_setup() runs from `booti`, long after the environment is up, which
 * is why the whole story -- including what the guard did stages earlier -- is
 * legible there.
 *
 * EVERY ONE OF THESE IS __section(".data"), AND THAT IS LOAD-BEARING.  The
 * first caller runs BEFORE relocation and the reporting runs after, so the
 * cache has to cross it.  ARM copies the running image, .data included, and
 * fixes it up -- but it then CLEARS .bss, and a zero-initialised static lives
 * in .bss.  So a cache written pre-relocation survives only if the variable is
 * forced into .data; the ones that are not are silently zeroed between the
 * write and the read.
 *
 * Measured 2026-08-30, on the boot after the reporting split landed: with an SC
 * that demonstrably came up (`SCP: started ... stat 0x3b00000`, BL31 `SCMI GET
 * Base ver rc=0 ver=0x20000`) the fixup reported "SC answered BASE
 * PROTOCOL_VERSION v0.0 (AO SCP_STAT 0x00000000)" and "scmi guard: did not
 * run".  gx_sc_verdict carried correctly -- it is initialised to -1, so it was
 * already in .data -- while every zero-initialised companion beside it read
 * back as its own "never happened" sentinel.  The verdict was right and its
 * evidence was gone, which is the most misleading shape this could have taken:
 * the flips were correct and the console described a different board.
 *
 * -1 still means "not asked yet", so a cache genuinely lost costs one extra
 * bounded query and cannot produce a wrong answer.
 */
static int gx_sc_verdict __section(".data") = -1;	/* -1 unasked, 0 mute, 1 answering */
static u32 gx_sc_stat __section(".data");	/* AO SCP_STAT exactly as read */
static u32 gx_sc_version __section(".data");	/* BASE PROTOCOL_VERSION, if one came back */
static bool gx_sc_ready __section(".data");	/* did the ready bits paint at all? */
static int gx_sc_rc __section(".data");		/* why the transaction failed, if it did */

/* True only if an SC answered SCMI on this boot.  Records why, prints nothing. */
static bool meson_gx_sc_ask(void)
{
	gx_sc_stat = 0;
	gx_sc_version = 0;
	gx_sc_rc = 0;

	gx_sc_ready = meson_gx_sc_painted_ready(&gx_sc_stat);
	if (!gx_sc_ready)
		return false;

	gx_sc_rc = meson_gx_sc_base_version(&gx_sc_version);

	return gx_sc_rc == 0;
}

static bool meson_gx_sc_answers(void)
{
	if (gx_sc_verdict < 0)
		gx_sc_verdict = meson_gx_sc_ask() ? 1 : 0;

	return gx_sc_verdict == 1;
}

/*
 * Say what the SC answered, from the cache.  Callable at any stage: it makes
 * no transaction, so the line can be emitted wherever a console exists.
 */
static void meson_gx_sc_report(void)
{
	if (meson_gx_sc_answers()) {
		printf("meson: scmi fixup: SC answered BASE PROTOCOL_VERSION v%u.%u (AO SCP_STAT 0x%08x)\n",
		       gx_sc_version >> 16, gx_sc_version & 0xffff, gx_sc_stat);
		return;
	}

	if (!gx_sc_ready)
		printf("meson: scmi fixup: SC did not answer (AO SCP_STAT 0x%08x, not ready) -- SCMI package left off, CPUs left on the DTB's clocks\n",
		       gx_sc_stat);
	else
		printf("meson: scmi fixup: SC did not answer (AO SCP_STAT 0x%08x ready, BASE PROTOCOL_VERSION rc %d) -- SCMI package left off, CPUs left on the DTB's clocks\n",
		       gx_sc_stat, gx_sc_rc);
}

/*
 * What the self-guard did to U-Boot's OWN control FDT, recorded in
 * board_init_f and reported later from a stage that has a console.
 */
enum {
	GX_GUARD_NOT_RUN = 0,
	GX_GUARD_NO_NODE,
	GX_GUARD_ALREADY_OFF,
	GX_GUARD_LEFT,
	GX_GUARD_REMOVED,
	GX_GUARD_FAILED,
};

static int gx_sc_guard_action __section(".data") = GX_GUARD_NOT_RUN;
static ulong gx_sc_guard_us __section(".data");

static void meson_gx_guard_report(void)
{
	switch (gx_sc_guard_action) {
	case GX_GUARD_NO_NODE:
		printf("meson: scmi guard: %s absent from U-Boot's own control FDT -- nothing to guard\n",
		       GX_SCMI_PATH);
		break;
	case GX_GUARD_ALREADY_OFF:
		printf("meson: scmi guard: %s already disabled in U-Boot's own control FDT -- left as found\n",
		       GX_SCMI_PATH);
		break;
	case GX_GUARD_LEFT:
		printf("meson: scmi guard: SC live -- %s left enabled in U-Boot's own control FDT (query %lu us)\n",
		       GX_SCMI_PATH, gx_sc_guard_us);
		break;
	case GX_GUARD_REMOVED:
		printf("meson: scmi guard: no SC -- %s removed from U-Boot's own control FDT, its SCMI agent never bound (query %lu us)\n",
		       GX_SCMI_PATH, gx_sc_guard_us);
		break;
	case GX_GUARD_FAILED:
		printf("meson: scmi guard: could NOT remove %s from U-Boot's own control FDT\n",
		       GX_SCMI_PATH);
		break;
	default:
		printf("meson: scmi guard: did not run -- U-Boot's own control FDT was never checked\n");
		break;
	}
}

/*
 * Disable every node of one compatible.  Optional by design: 0 matches is
 * not an error.  Offsets are re-searched from the start after each write
 * because a setprop that grows the struct block invalidates them.
 */
static int meson_disable_by_compat(void *fdt, const char *compat)
{
	int off, n = 0;

	for (;;) {
		off = fdt_node_offset_by_compatible(fdt, -1, compat);
		while (off >= 0) {
			const char *st = fdt_getprop(fdt, off, "status", NULL);

			if (!st || strcmp(st, "disabled"))
				break;
			off = fdt_node_offset_by_compatible(fdt, off, compat);
		}
		if (off < 0)
			break;
		if (fdt_setprop_string(fdt, off, "status", "disabled"))
			return -FDT_ERR_NOSPACE;
		n++;
	}

	return n;
}

static int meson_set_status(void *fdt, const char *path, const char *status)
{
	int off = fdt_path_offset(fdt, path);

	if (off < 0)
		return off;

	return fdt_setprop_string(fdt, off, "status", status);
}

static int meson_cpu_point_at_scmi(void *fdt, const char *cpu_path,
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
	spec[1] = cpu_to_fdt32(0);	/* the SC's single perf domain */
	if (fdt_setprop(fdt, off, "power-domains", spec, sizeof(spec)))
		return -FDT_ERR_NOSPACE;

	off = fdt_path_offset(fdt, cpu_path);
	if (off < 0)
		return off;
	if (fdt_setprop_string(fdt, off, "power-domain-names", "perf"))
		return -FDT_ERR_NOSPACE;

	return 0;
}

/*
 * The PUB-telemetry gate: hand Linux the SC's sampler block on the SC's LIVE
 * answer, and on nothing else.
 *
 * WHY A QUESTION AND NOT A RECORD.  Share SRAM sits in the always-on domain and
 * survives a warm reset, so anything an SC left in it stays legible to a later
 * boot whose SC may have no sampler at all -- four blind reviews of the record
 * version kept producing exactly that boot (design doc section 2a, "The
 * production gate: a live SCMI answer from this boot's SC", and n848).  An SCMI
 * answer cannot be stale: either this SC is replying now, or there is no
 * answer.  So the gate is one PUBMON_GET_BLOCK round trip and no SRAM is read,
 * written, or remembered anywhere.
 *
 * This runs ONLY where an SC has already answered SCMI BASE, which is what
 * makes the round trip safe: that reply is the evidence the SC is up and its mailbox
 * window is open.  The second message is the same transaction as the first, on
 * a channel meson_gx_sc_xfer() proved idle and hands back FREE, and it has no
 * side effect on the SC.
 *
 * THE ANSWERS, one printed line each, all of them except the first meaning the
 * node stays exactly as the shut above left it -- disabled:
 *
 *   SUCCESS, running, block == reg   okay      the sampler is publishing the
 *                                              very KiB this node describes
 *   NOT_SUPPORTED                    disabled  no protocol 0x80 at all: an SC
 *                                              built without the sampler
 *   any other SCMI status            disabled  the protocol refused the
 *                                              message; printed by name
 *   SUCCESS, running clear           disabled  the protocol is there, the
 *                                              sampler is not up yet
 *   SUCCESS, block != reg            disabled  a real sampler, a block that is
 *                                              not the one this node names
 *   transport failure                disabled  we never got an answer, which is
 *                                              NOT a "no" from the SC and says
 *                                              so with its own rc
 *
 * The okay is the ONLY write this function makes, and only to the status
 * property: every node of this compatible is already disabled by the time it is
 * reached, on every meson build, at the top of ft_board_setup().
 */
static void meson_gx_pubmon(void *fdt)
{
	struct gx_pubmon_block blk = { GX_SCMI_STATUS_NONE, 0, 0, 0, 0 };
	u64 node_addr = OF_BAD_ADDR, node_size = 0;
	const fdt32_t *reg;
	int parent, len, na, ns, off;
	int ret;

	off = fdt_node_offset_by_compatible(fdt, -1, GX_PUBMON_COMPAT);
	if (off < 0) {
		printf("meson: scmi fixup: no %s node -- nothing to enable\n",
		       GX_PUBMON_COMPAT);
		return;
	}

	ret = meson_gx_sc_pubmon_block(&blk);
	if (ret) {
		printf("meson: scmi fixup: %s status -> disabled (PUBMON_GET_BLOCK did not answer, rc %d)\n",
		       GX_PUBMON_COMPAT, ret);
		return;
	}

	/*
	 * Only NOT_SUPPORTED means "no protocol 0x80": it is what the SC's SCMI
	 * core answers for a protocol it was built without.  Every other status
	 * is a protocol that IS there refusing this message, and is printed as
	 * that, by name (landed review U3).  The NOT_SUPPORTED text is matched
	 * by the dram-char host (plugins/gxl_s905x_cc FIXUP_LINES) -- keep it.
	 */
	if (blk.status == GX_SCMI_STATUS_NOT_SUPPORTED) {
		printf("meson: scmi fixup: %s status -> disabled (SC has no PUB sampler protocol, rc %d)\n",
		       GX_PUBMON_COMPAT, blk.status);
		return;
	}
	if (blk.status != GX_SCMI_STATUS_OK) {
		printf("meson: scmi fixup: %s status -> disabled (SC refused PUBMON_GET_BLOCK: %s, rc %d)\n",
		       GX_PUBMON_COMPAT, meson_gx_scmi_status_name(blk.status),
		       blk.status);
		return;
	}

	if (!(blk.flags & GX_PUBMON_F_RUNNING)) {
		printf("meson: scmi fixup: %s status -> disabled (SC PUB sampler not running)\n",
		       GX_PUBMON_COMPAT);
		return;
	}

	/*
	 * The block the SC is publishing has to BE the block this node
	 * describes -- otherwise "okay" points Linux at somebody else's memory.
	 *
	 * AND reg IS NOT A CPU ADDRESS.  &sram is `sram@c8000000` with
	 * ranges = <0 0x0 0xc8000000 0x14000>, so this node's reg is an OFFSET
	 * through that window -- <0x11c00 0x400>, i.e. 0xc8011c00 once
	 * translated.  Comparing the raw first cell against the SC's answer
	 * compares 0x11c00 with 0xc8011c00 and the gate never opens on any
	 * board (blind review 5).  fdt_translate_address() walks the same
	 * ranges the kernel's parser walks and returns OF_BAD_ADDR when the
	 * walk cannot be done, which matches nothing.  It is compiled whenever
	 * OF_LIBFDT is (boot/Makefile), so there is no extra config to depend
	 * on (blind review 6).
	 *
	 * HOW MANY CELLS each half of reg is is the PARENT's business:
	 * fdt_address_cells()/fdt_size_cells() on the parent say the split, and
	 * the size is the cell(s) that follow the address -- reg[na..na+ns-1].
	 * &sram today says 1+1, so that is reg[1]; hardcoding the index instead
	 * of asking would silently compare the wrong word on any parent that
	 * spells reg differently (blind review 6).  node_size is the whole
	 * field, so a size too wide to be the SC's 32-bit one cannot match.
	 */
	parent = fdt_parent_offset(fdt, off);
	na = parent >= 0 ? fdt_address_cells(fdt, parent) : 2;
	ns = parent >= 0 ? fdt_size_cells(fdt, parent) : 2;
	reg = fdt_getprop(fdt, off, "reg", &len);
	if (reg && parent >= 0 && na > 0 && ns > 0 &&
	    len >= (na + ns) * sizeof(fdt32_t)) {
		node_addr = fdt_translate_address(fdt, off, reg);
		node_size = fdtdec_get_number(reg + na, ns);
	}

	if (node_addr != blk.block_ns_addr || node_size != blk.block_size ||
	    blk.block_version != GX_PUBMON_BLOCK_VERSION) {
		printf("meson: scmi fixup: %s status -> disabled (SC block 0x%08x size 0x%x version %u does not match the node)\n",
		       GX_PUBMON_COMPAT, blk.block_ns_addr, blk.block_size,
		       blk.block_version);
		return;
	}

	if (fdt_setprop_string(fdt, off, "status", "okay")) {
		printf("meson: scmi fixup: could not set %s status (out of space?)\n",
		       GX_PUBMON_COMPAT);
		return;
	}

	printf("meson: scmi fixup: %s status -> okay (SC PUB sampler running, block 0x%08x size 0x%x)\n",
	       GX_PUBMON_COMPAT, blk.block_ns_addr, blk.block_size);
}

int meson_gx_libre_scmi_fixup(void *fdt)
{
	char cpu_path[sizeof("/cpus/cpu@0")];
	const char *missing = NULL;
	u32 clk_ph, perf_ph;
	bool sc_answers;
	int off, i;

	/*
	 * 1. Is the SC-DEPENDENT carrier here?  This gate now covers the SCMI
	 * package and NOTHING ELSE, and narrowing its scope is the fix: it used
	 * to cover the SC-INDEPENDENT flips too, so a DTB without the SCMI
	 * package got none of them.
	 *
	 * Measured 2026-09-03 on the AML-S905X-CC v1, whose kernel DTS carries
	 * /scpi but has never carried the SCMI package -- /firmware/scmi exists
	 * only in meson-gxl-s905x-libretech-cc-v2.dts, so the v1's DTB simply
	 * has no such node.  The all-or-nothing gate printed "DTB lacks
	 * /firmware/scmi -- left as is" and returned, /scpi stayed enabled, and
	 * Linux took exactly the abort the 2a comment below was written about,
	 * inside deferred_probe_work_func.  A missing SCMI package is a reason
	 * to skip the SCMI flips; it is never a reason to hand Linux a mailbox
	 * nobody is listening on.
	 *
	 * The SC-INDEPENDENT flips are each optional by their own presence
	 * check, below, so a blob carrying only part of the tree is still never
	 * half-flipped: every write is guarded by the node it writes.
	 */
	if (fdt_path_offset(fdt, GX_SCMI_PATH) < 0)
		missing = GX_SCMI_PATH;

	off = fdt_path_offset(fdt, GX_SCMI_CLK_PATH);
	clk_ph = off < 0 ? 0 : fdt_get_phandle(fdt, off);
	off = fdt_path_offset(fdt, GX_SCMI_PERF_PATH);
	perf_ph = off < 0 ? 0 : fdt_get_phandle(fdt, off);
	if (!missing && !clk_ph)
		missing = GX_SCMI_CLK_PATH " (node or phandle)";
	if (!missing && !perf_ph)
		missing = GX_SCMI_PERF_PATH " (node or phandle)";

	for (i = 0; !missing && i < GX_NR_CPUS; i++) {
		snprintf(cpu_path, sizeof(cpu_path), "/cpus/cpu@%d", i);
		if (fdt_path_offset(fdt, cpu_path) < 0)
			missing = "/cpus/cpu@N";
	}

	/*
	 * 2. Ask the SC before asserting anything that depends on it.  A mute
	 * SC costs us the SCMI package and nothing else: the blob keeps the
	 * /firmware/scmi it arrived with, so what Linux gets is the
	 * configuration this board booted before the package existed.
	 *
	 * With no carrier the answer cannot change a single write, so the SC is
	 * not asked at all -- the query is a live MMIO round trip into the AO
	 * shared window and this code asserts nothing it does not need.
	 */
	sc_answers = false;
	if (missing)
		printf("meson: scmi fixup: DTB lacks %s -- SCMI package left as is, SC not asked\n",
		       missing);
	else
		sc_answers = meson_gx_sc_answers();
	if (!missing)
		meson_gx_sc_report();

	/*
	 * And say what the self-guard did to U-Boot's OWN control FDT stages
	 * ago, because board_init_f's console is silent on this board (see the
	 * cache comment above).  This is the guard's only console witness.
	 */
	meson_gx_guard_report();

	/*
	 * 2a. /scpi -> disabled, either way, and this is NOT an SC-liveness
	 * question.  /scpi describes the VENDOR BL30's SCPI service on this
	 * mailbox, and a -libre image never loads a vendor BL30 -- BL31 says so
	 * itself ("not uploading").  So on this build the node is false whether
	 * or not our SC answered, and leaving it enabled is not the harmless
	 * "pre-package" configuration it looks like:
	 *
	 *   [    4.819905] Internal error: synchronous external abort: \
	 *                  0000000096000210 [#1]  SMP
	 *   [    4.859304] pc : scpi_process_cmd+0x1a4/0x2a0
	 *      scpi_send_message / scpi_probe / platform_probe
	 *
	 * -- measured 2026-08-30 on the very first boot of this guard, with
	 * /firmware/scmi correctly left disabled.  Linux's SCPI driver reaches
	 * the SAME shared-memory window the SCMI one does, so with no SC it
	 * takes the SAME abort, taints MACHINE_CHECK and never reaches
	 * userspace.  Guarding only the SCMI half moved the fault, it did not
	 * remove it.
	 *
	 * Optional by presence, not by carrier: a DTB with no /scpi has nothing
	 * to disable, and a DTB with no SCMI package still has this one to.
	 */
	if (fdt_path_offset(fdt, GX_SCPI_PATH) < 0) {
		printf("meson: scmi fixup: no %s node -- nothing to disable\n",
		       GX_SCPI_PATH);
	} else {
		if (meson_set_status(fdt, GX_SCPI_PATH, "disabled"))
			goto fail;
		printf("meson: scmi fixup: %s status -> disabled\n", GX_SCPI_PATH);
	}

	/*
	 * 2b. The PUB-telemetry node is shut on EVERY path, and it was shut
	 * already -- at the top of ft_board_setup(), outside this Kconfig,
	 * because the default cannot depend on whether the board that asks the
	 * question is even built in.  All that is left here is to say what that
	 * shut MEANS, and the reason differs on two paths: with no SCMI carrier
	 * in the blob the one question that could have opened the node was
	 * never asked at all, and asking it is not something this build does
	 * when it has nothing to put the answer in.  Printed only when a node is
	 * present to have been shut; a blob without one has nothing to report.
	 */
	if (!sc_answers &&
	    fdt_node_offset_by_compatible(fdt, -1, GX_PUBMON_COMPAT) >= 0)
		printf("meson: scmi fixup: %s status -> disabled (%s)\n",
		       GX_PUBMON_COMPAT,
		       missing ? "DTB has no SCMI package; SC not asked"
			       : "SC did not answer SCMI");

	if (sc_answers) {
		/* 2c. The SC-DEPENDENT flips. */
		if (meson_set_status(fdt, GX_SCMI_PATH, "okay"))
			goto fail;
		printf("meson: scmi fixup: %s status -> okay\n", GX_SCMI_PATH);

		for (i = 0; i < GX_NR_CPUS; i++) {
			snprintf(cpu_path, sizeof(cpu_path), "/cpus/cpu@%d", i);
			if (meson_cpu_point_at_scmi(fdt, cpu_path, clk_ph, perf_ph))
				goto fail;
			printf("meson: scmi fixup: %s clocks -> protocol@14 (phandle %u), power-domains -> protocol@13 (phandle %u) \"perf\"\n",
			       cpu_path, clk_ph, perf_ph);
		}

		/*
		 * And the PUB-telemetry node, already shut by 2b above.  An SCMI
		 * reply says the mailbox is live, not that the sampler is
		 * running, so this asks a second question of the SC's own -- and
		 * that question IS the gate: it is asked here, on a channel the
		 * BASE reply proved live, and nothing about the answer can be
		 * left over from an earlier boot.
		 */
		meson_gx_pubmon(fdt);
	}

	/*
	 * 3. The SC-INDEPENDENT flips, which happen either way.
	 *
	 * The AO watchdog is not the SC's: the BootROM arms it before it loads
	 * anything and CONFIG_MESON_GXL_AO_WDT_HANDOFF refreshes it from
	 * U-Boot's own board init while U-Boot runs, and by default stops it at
	 * wdt_handoff_to_os() (board_wdt_handoff()); env wdt_handoff=1 hands it to the OS
	 * running instead, for a kernel known to carry the driver.  The node is
	 * enabled either way, because the hand-off choice is the environment's,
	 * not the SC's: leaving it disabled because the SC was mute would take
	 * the watchdog away from a kernel that was handed it running, and the
	 * ROM's 5 s budget would reset the board in kernel init -- a SECOND
	 * defect, caused by the guard rather than found by it.
	 *
	 * Optional by presence: the AML-S905X-CC v1's DTB describes its
	 * watchdog as "amlogic,meson-gxbb-wdt" with no status property at all,
	 * i.e. already enabled and not this node.  Absent is not a failure.
	 */
	off = fdt_node_offset_by_compatible(fdt, -1, GX_AO_WDT_COMPAT);
	if (off < 0) {
		printf("meson: scmi fixup: no %s node -- left as is\n",
		       GX_AO_WDT_COMPAT);
	} else {
		if (fdt_setprop_string(fdt, off, "status", "okay"))
			goto fail;
		printf("meson: scmi fixup: %s status -> okay\n", GX_AO_WDT_COMPAT);
	}

	/*
	 * The AO SRAM pstore consumer is not the SC's either: its pool is AO
	 * SRAM this image lays out, whether or not the SC answered.  The DTB
	 * ships it disabled because on the vendor chain that SRAM belongs to
	 * the vendor firmware.  Optional by presence: only boards that carry
	 * the consumer (AML-S805X-AC today) have the node.
	 */
	off = fdt_node_offset_by_compatible(fdt, -1, GX_SRAM_PSTORE_COMPAT);
	if (off < 0) {
		printf("meson: scmi fixup: no %s node -- left as is\n",
		       GX_SRAM_PSTORE_COMPAT);
	} else {
		if (fdt_setprop_string(fdt, off, "status", "okay"))
			goto fail;
		printf("meson: scmi fixup: %s status -> okay\n",
		       GX_SRAM_PSTORE_COMPAT);
	}

	/*
	 * 4. The two vendor-BL31 descriptions this BL31 cannot honour.  Also
	 * SC-independent: both describe what the VENDOR's BL31 implements, and
	 * which BL31 is running is settled by the image, not by whether the SC
	 * came up.  meson-gx-pm's suspend_ops out-rank psci's on any build of
	 * this BL31, and CPU_SLEEP_0 has no wake path on it either way -- with
	 * no SC at all it is refused harder, not less.
	 */
	i = meson_disable_by_compat(fdt, GX_PM_COMPAT);
	if (i < 0)
		goto fail;
	printf("meson: scmi fixup: %s status -> disabled (%d node%s; PSCI SYSTEM_SUSPEND owns mem)\n",
	       GX_PM_COMPAT, i, i == 1 ? "" : "s");

	i = meson_disable_by_compat(fdt, GX_IDLE_STATE_COMPAT);
	if (i < 0)
		goto fail;
	printf("meson: scmi fixup: %s status -> disabled (%d node%s; idle is WFI)\n",
	       GX_IDLE_STATE_COMPAT, i, i == 1 ? "" : "s");

	return 0;
fail:
	printf("meson: scmi fixup: could not write the blob (out of space?)\n");
	return 0;
}

/*
 * ---------------------------------------------------------------------------
 * The same question, one blob earlier: U-Boot's OWN control FDT
 * ---------------------------------------------------------------------------
 *
 * Everything above is about the blob handed to Linux.  But this U-Boot has an
 * SCMI agent of its own -- CONFIG_SCMI_FIRMWARE + CONFIG_SCMI_AGENT_MAILBOX,
 * with /firmware/scmi in arch/arm/dts/meson-gxl-scmi-u-boot.dtsi merged into
 * the control DTB -- and that node makes the SAME assertion about hardware,
 * two stages before ft_board_setup() ever runs.
 *
 * With no SC the agent does not merely fail; it takes the board down, and it
 * does so before it has sent anything.  UCLASS_SCMI_AGENT's post_bind probes
 * the base protocol at DM BIND time, scmi_mailbox_probe() calls
 * scmi_dt_get_smt_buffer(), and that ends in scmi_smt_enable_intr() doing
 *
 *	ioread32(&hdr->flags)
 *
 * on the shared-memory window named by the "shmem" phandle -- cpu_scp_hpri at
 * 0xC8013400, so 0xC8013410.  If the SC never ran hardware_init_hook() that
 * window was never opened to the non-secure world, and the read is a
 * synchronous external abort:
 *
 *	"Synchronous Abort" handler, esr 0x96000210, far 0xc8013410
 *	Resetting CPU ...
 *
 * measured 2026-08-30 with a loader-carrying image and an inert SC payload
 * (audit 2026-08-30-gxl-n55-sc-wait-bound.md section 6, N57), and reproduced
 * with this branch's four bring-up symbols off, so it is not a diagnostic.
 * It is the same unguarded-window defect the fixup above was taught to avoid,
 * one client over, and it is invisible on v2026.07/lc-master only because that
 * branch carries no SC loader and so never has an SC to lose.
 *
 * WHY THE NODE IS REMOVED RATHER THAN THE PROBE MADE TO FAIL.  There is no
 * point in the agent's own path at which it can decline: the very first thing
 * it does with the window is read it, so "probe cleanly and return an error"
 * still costs the board.  The only clean answer is that the agent must not be
 * bound at all -- and the node is what binds it.  So the firmware describes
 * itself here too: with no SC, U-Boot's control FDT stops claiming one.
 *
 * WHY board_fix_fdt().  It is U-Boot's own hook for exactly this ("board-
 * specific manipulation of Device Tree ... e.g. to delete device from it",
 * dts/Kconfig), it runs in board_init_f after timer_init/serial_init/
 * console_init_f and before reloc_fdt() and initr_dm(), so the edit reaches
 * the relocated blob and lands before anything scans it, and it needs no new
 * call site of ours in generic code.
 *
 * WHY fdt_del_node() AND NOT status = "disabled".  Deleting always fits.
 * Setting a longer string does not: the control DTB's totalsize is exactly its
 * content, so growing "okay" (5 B) to "disabled" (9 B) is -FDT_ERR_NOSPACE and
 * the guard would silently do nothing on the one path that matters.  Nothing
 * outside the subtree refers into it, so there is no dangling phandle.
 *
 * WHEN THE SC IS THERE THIS FUNCTION CHANGES NOTHING -- it leaves the node
 * exactly as the DTS wrote it, and the one SC query it makes is the same query
 * ft_board_setup() used to make, moved earlier and cached, not added to.
 */
int meson_gx_libre_scmi_self_guard(void *fdt)
{
	const char *st;
	ulong t0;
	int off;

	off = fdt_path_offset(fdt, GX_SCMI_PATH);
	if (off < 0) {
		/* this build describes no agent: nothing to guard */
		gx_sc_guard_action = GX_GUARD_NO_NODE;
		return 0;
	}

	st = fdt_getprop(fdt, off, "status", NULL);
	if (st && strcmp(st, "okay") && strcmp(st, "ok")) {
		/* already off: leave the reason to whoever set it */
		gx_sc_guard_action = GX_GUARD_ALREADY_OFF;
		return 0;
	}

	t0 = timer_get_us();
	if (meson_gx_sc_answers()) {
		gx_sc_guard_us = timer_get_us() - t0;
		gx_sc_guard_action = GX_GUARD_LEFT;
		return 0;
	}

	if (fdt_del_node(fdt, off)) {
		gx_sc_guard_action = GX_GUARD_FAILED;
		return 0;
	}

	gx_sc_guard_us = timer_get_us() - t0;
	gx_sc_guard_action = GX_GUARD_REMOVED;

	return 0;
}
#endif /* CONFIG_MESON_GX_LIBRE_SCMI_FIXUP */

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
	reg = readl(GX_AO_SEC_GP_CFG3);

	bl31_size = ((reg & GX_AO_BL31_RSVMEM_SIZE_MASK)
			>> GX_AO_BL31_RSVMEM_SIZE_SHIFT) * SZ_1K;
	bl32_size = (reg & GX_AO_BL32_RSVMEM_SIZE_MASK) * SZ_1K;

	bl31_start = readl(GX_AO_SEC_GP_CFG5);
	bl32_start = readl(GX_AO_SEC_GP_CFG4);

	/*
	 * There used to be a fallback here installing 0x10000000 + 0x200000
	 * when all three registers read zero. That layout is not this family's:
	 * GXL/GXM put BL31 at 0x05100000, covered by the secmon@5000000 DT node.
	 * Measured on AML-S905X-CC-V2, whose GP_CFG3/4/5 do all read 0 so the
	 * fallback fired: the entire 0x10000000 + 0x200000 window was overwritten
	 * with BL31 and the SCP still answering PSCI. It reserved nothing.
	 */

#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT)
	/*
	 * The nodes are the MECHANISM; this /memreserve/ is defence in depth
	 * for a stale or foreign DTB reaching the kernel, and costs one entry.
	 * memblock takes the UNION of /memreserve/ and no-map nodes, so the two
	 * never conflict.  It also feeds the EFI memory map.
	 *
	 * It covers what FIRMWARE always owns -- ROM mirror, BL31 and the
	 * mailboxes, 3 MiB -- and deliberately stops at ramoops.  The OP-TEE
	 * band is not in it: that is Linux's unless OP-TEE runs.  A memreserve only keeps the OS
	 * from ALLOCATING there; it does not make a region usable as pstore.
	 * So on a DTB that carries our ramoops node the extra MiB would be
	 * redundant, and on one that does not it would permanently sequester a
	 * MiB for a feature that blob cannot use.
	 */
	meson_add_libre_secure_reservation(fdt);
	meson_board_add_reserved_memory(fdt, GX_LIBRE_SEC_BLOCK_BASE,
					GX_LIBRE_FW_RSV_SIZE);
	meson_libre_optee_fixup(fdt);
#else
	/* Add the firmware reserved zone below BL33 (see GX_FIRMWARE_MEM_SIZE) */
	meson_board_add_reserved_memory(fdt, 0, GX_FIRMWARE_MEM_SIZE);
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
	/* Size is reported in MiB, convert it in bytes */
	return ((readl(GX_AO_SEC_GP_CFG0) & GX_AO_MEM_SIZE_MASK)
			>> GX_AO_MEM_SIZE_SHIFT) * SZ_1M;
}

static struct mm_region gx_mem_map[] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0xc0000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		.virt = 0xc0000000UL,
		.phys = 0xc0000000UL,
		.size = 0x30000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = gx_mem_map;
