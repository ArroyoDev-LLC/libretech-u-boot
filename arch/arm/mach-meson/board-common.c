// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 */

#include <cpu_func.h>
#include <fastboot.h>
#include <init.h>
#include <net.h>
#include <asm/arch/boot.h>
#include <bootsource.h>
#include <env.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <asm/ptrace.h>
#include <asm/system.h>
#include <linux/libfdt.h>
#include <linux/err.h>
#include <string.h>
#include <asm/arch/mem.h>
#include <asm/arch/sm.h>
#include <asm/armv8/mmu.h>
#include <asm/unaligned.h>
#include <efi_loader.h>
#include <fdtdec.h>
#include <lmb.h>
#include <blk.h>
#include <fs.h>
#include <linux/sizes.h>
#include <u-boot/crc.h>
#include <usb_otg_role.h>

#include <asm/psci.h>
#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT)
#include <asm/arch/gx.h>
#elif defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT)
#include <asm/arch/g12a.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

__weak int board_init(void)
{
	return 0;
}

#if CONFIG_IS_ENABLED(BOOTSOURCE)
/*
 * The BootROM's boot device (AO_SEC_GP_CFG0 on every family) mapped onto the
 * platform vocabulary. Both phases: the SPL banner and U-Boot proper's
 * `bootsource` are spelled from this one switch.
 */
enum bootsource meson_bootsource(void)
{
	switch (meson_get_boot_device()) {
	case BOOT_DEVICE_EMMC:
		return BOOTSOURCE_EMMC;
	case BOOT_DEVICE_NAND:
		return BOOTSOURCE_NAND;
	case BOOT_DEVICE_SPI:
		return BOOTSOURCE_SPI;
	case BOOT_DEVICE_SD:
		return BOOTSOURCE_SD;
	case BOOT_DEVICE_USB:
		return BOOTSOURCE_USB;
	default:
		return BOOTSOURCE_UNKNOWN;
	}
}
#endif

/* XPL_BUILD covers SPL and TPL; SPL_BUILD alone leaves TPL linking this. */
#ifndef CONFIG_XPL_BUILD
int dram_init(void)
{
	const fdt64_t *val;
	int offset;
	int len;

	offset = fdt_path_offset(gd->fdt_blob, "/memory");
	if (offset < 0)
		return -EINVAL;

	val = fdt_getprop(gd->fdt_blob, offset, "reg", &len);
	if (len < sizeof(*val) * 2)
		return -EINVAL;

	/* Use unaligned access since cache is still disabled */
	gd->ram_size = get_unaligned_be64(&val[1]);

	return 0;
}
#endif

__weak int meson_ft_board_setup(void *blob, struct bd_info *bd)
{
	return 0;
}

/*
 * MESON_VDEC_DMA_LIMIT: CMA must be placed in the first 1 GB of physical
 * address space on SM1/G12 boards. Determined empirically: CMA at
 * 0xb3400000 (~2.87 GB) causes HEVC bus lockup (KI-036, 2026-04-04).
 */
#define MESON_VDEC_DMA_LIMIT	(0x40000000ULL)	/* 1 GB physical address cap */

static void meson_fixup_cma_size(void *blob)
{
	int cma_node;
	u64 cma_size, dt_cma_max;
	fdt32_t cma_val[2];
	fdt32_t alloc_ranges[4];
	const fdt32_t *sizep;
	phys_size_t ram;
	int len;

	cma_node = fdt_path_offset(blob, "/reserved-memory/linux,cma");
	if (cma_node < 0)
		return;

	/* Read board DTS CMA size as upper bound; default to unconstrained */
	dt_cma_max = (u64)-1;
	sizep = fdt_getprop(blob, cma_node, "size", &len);
	if (sizep && len >= 8)
		dt_cma_max = (u64)fdt32_to_cpu(sizep[0]) << 32 |
			     fdt32_to_cpu(sizep[1]);

	/*
	 * Use get_effective_memsize() instead of gd->ram_size because
	 * dram_init() reads the DTS /memory node which may understate
	 * the actual RAM. For example, S805X-AC has a 512MB DTS memory
	 * node but the 1GB variant has 1GB physical RAM.
	 *
	 * get_effective_memsize() reads the hardware register
	 * (GX_AO_SEC_GP_CFG0 on GX, G12A_AO_SEC_GP_CFG0 on G12A+)
	 * which reflects the actual detected RAM size.
	 */
	ram = get_effective_memsize();

	/*
	 * Apply the same 224 MiB/GiB formula as the Linux kernel's
	 * of_reserved_mem.c: (ram >> 5) * 7 = 7/32 of RAM.
	 * Align down to 4 MiB for CMA alignment requirements.
	 */
	cma_size = ((ram >> 5) * 7) & ~(4ULL * 1024 * 1024 - 1);

	/* Never exceed the board DTS-specified maximum */
	if (cma_size > dt_cma_max)
		cma_size = dt_cma_max;

	cma_val[0] = cpu_to_fdt32(0);
	cma_val[1] = cpu_to_fdt32((u32)cma_size);

	if (fdt_setprop(blob, cma_node, "size", cma_val, sizeof(cma_val)))
		printf("meson: failed to set CMA size\n");

	/*
	 * Set alloc-ranges to constrain CMA placement to the first 1 GB of
	 * physical address space (MESON_VDEC_DMA_LIMIT). Without this, the
	 * kernel may place CMA above 1 GB on 4GB boards, causing HEVC to
	 * DMA to unreachable addresses and hang the bus (KI-036).
	 */
	alloc_ranges[0] = cpu_to_fdt32(0);		/* start hi */
	alloc_ranges[1] = cpu_to_fdt32(0);		/* start lo = 0x0 */
	alloc_ranges[2] = cpu_to_fdt32(0);		/* size hi */
	alloc_ranges[3] = cpu_to_fdt32((u32)MESON_VDEC_DMA_LIMIT); /* size lo = 1 GB */
	if (fdt_setprop(blob, cma_node, "alloc-ranges",
			alloc_ranges, sizeof(alloc_ranges)))
		printf("meson: failed to set CMA alloc-ranges\n");
	else
		printf("meson: CMA %lluMB in first %lluMB (RAM %lluMB)\n",
		       cma_size >> 20, MESON_VDEC_DMA_LIMIT >> 20,
		       ram >> 20);
}

/*
 * Shut every PUB-telemetry shmem node, on EVERY meson build.
 *
 * This node hands Linux a KiB that only this boot's SC can make worth reading,
 * so the default has to be "off" wherever the blob could have come from -- a
 * board DTS, a vendor DTB, a `fdt set ... status okay` from the prompt, or a
 * blob cached by an earlier U-Boot.  Off is also the only state this function
 * can guarantee: everything that could put an okay here (the libre SCMI fixup
 * below) runs after it and only after asking the SC.
 *
 * The failure ladder has to END in something safe.  If the write fails -- a
 * blob out of space -- the node is removed instead, because a node Linux will
 * read is worse than a node Linux does not have.  If even the nop fails the
 * blob cannot be trusted at all and the boot stops here rather than hand it on.
 * A search that fails for anything other than "no such node" stops the boot on
 * the same argument: a blob this function could not walk is not a blob it has
 * proved shut.
 *
 * Nodes already disabled are skipped rather than rewritten, and the search
 * restarts after every write, because a setprop that grows a struct block
 * invalidates every offset after it.
 */
static int meson_shut_pubmon_nodes(void *blob)
{
	for (;;) {
		int off = fdt_node_offset_by_compatible(blob, -1,
						       MESON_PUBMON_COMPAT);
		const char *status;
		char name[64], was[16];
		int ret, set_ret;

		while (off >= 0) {
			status = fdt_getprop(blob, off, "status", NULL);

			if (!status || strcmp(status, "disabled"))
				break;
			off = fdt_node_offset_by_compatible(blob, off,
							    MESON_PUBMON_COMPAT);
		}
		/*
		 * -FDT_ERR_NOTFOUND is the only quiet stop, because that is what
		 * "no node of this compatible is left" looks like.  Any other
		 * negative means the blob could not be walked -- and a walk that
		 * failed is not evidence that every node was shut, so it fails
		 * ft_board_setup() instead of handing on a blob that may still
		 * carry an open one (blind review 6).
		 */
		if (off == -FDT_ERR_NOTFOUND)
			return 0;
		if (off < 0)
			return off;

		/*
		 * Say so whenever the shut CHANGES something, so a blob that
		 * arrived with the node open is visible on the console (landed
		 * review U6).  Name and old status are copied first: both point
		 * into the struct block that the write below may move or nop.
		 */
		status = fdt_getprop(blob, off, "status", NULL);
		strlcpy(was, status ? status : "(none)", sizeof(was));
		strlcpy(name, fdt_get_name(blob, off, NULL) ?: "?",
			sizeof(name));

		set_ret = fdt_setprop_string(blob, off, "status", "disabled");
		if (!set_ret) {
			printf("meson: %s %s status %s -> disabled (until the SC is asked)\n",
			       MESON_PUBMON_COMPAT, name, was);
			continue;
		}
		ret = fdt_nop_node(blob, off);
		if (ret)
			return ret;
		printf("meson: %s %s removed (status %s could not be set disabled, rc %d)\n",
		       MESON_PUBMON_COMPAT, name, was, set_ret);
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int ret;

	/*
	 * THE FIRST WRITE OF THE BLOB, and it has to be first: this is the
	 * default every later fixup is allowed only to qualify.
	 *
	 * Deliberately NOT inside meson_ft_board_setup(), which is where a
	 * board's own fixups go: that weak hook is the LAST thing this function
	 * does -- after the SCMI fixup below, which is the one thing allowed to
	 * put an okay back -- and any board may override it (vim3 does), so a
	 * shut living there would erase the SC's answer, and a board file could
	 * silently drop it.  Same reason it is outside the
	 * CONFIG_MESON_GX_LIBRE_SCMI_FIXUP gate: the default cannot depend on
	 * whether the board that asks the question is even built in.
	 */
	ret = meson_shut_pubmon_nodes(blob);
	if (ret)
		return ret;

	meson_init_reserved_memory(blob);
	meson_fixup_cma_size(blob);

	if (IS_ENABLED(CONFIG_MESON_GX_LIBRE_SCMI_FIXUP))
		meson_gx_libre_scmi_fixup(blob);

	if (IS_ENABLED(CONFIG_MESON_G12A_LIBRE_SCMI_FIXUP))
		meson_g12a_libre_scmi_fixup(blob);

	/*
	 * env usb_otg -> the OTG controller's dr_mode.  It reports its own
	 * failures; a DT it could not write is still the valid DT it was, so
	 * the boot goes on.
	 */
	usb_otg_fdt_fixup(blob);

	/*
	 * /chosen/libre-computer,boot-source: on this live tree EVT_FT_FIXUP
	 * is not sent, so bootsource's spy does not run (boot/bootsource.c).
	 */
	ret = bootsource_fdt_fixup(blob);
	if (ret)
		printf("meson: boot source not written to /chosen: %s\n",
		       fdt_strerror(ret));

	return meson_ft_board_setup(blob, bd);
}

#if !defined(CONFIG_XPL_BUILD) && CONFIG_IS_ENABLED(OF_BOARD_FIXUP)
/*
 * The blob above is the one Linux gets.  This one is U-Boot's OWN control FDT,
 * and it needs the same question asked of it two stages earlier, because this
 * U-Boot has an SCMI agent of its own and DM binds it long before
 * ft_board_setup() runs.  See meson_gx_libre_scmi_self_guard().
 *
 * Runs in board_init_f, after console_init_f and before reloc_fdt()/initr_dm().
 * MESON_GX_LIBRE_SCMI_FIXUP selects OF_BOARD_FIXUP, so this exists only on the
 * builds that describe an open SC; every other Meson target is unchanged and
 * has no board_fix_fdt() at all.
 *
 * "After console_init_f" is NOT "with a console": this defconfig sets
 * CONFIG_SILENT_CONSOLE_UNTIL_ENV=y with no CONFIG_PRE_CONSOLE_BUFFER, so
 * GD_FLG_SILENT is held over the whole of board_init_f and every printf() from
 * here is discarded unreplayed.  The guard therefore RECORDS what it did and
 * ft_board_setup() reports it later; see the cache comment in board-gx.c.
 */
int board_fix_fdt(void *rw_fdt_blob)
{
	if (IS_ENABLED(CONFIG_MESON_GX_LIBRE_SCMI_FIXUP))
		meson_gx_libre_scmi_self_guard(rw_fdt_blob);

	if (IS_ENABLED(CONFIG_MESON_G12A_LIBRE_SCMI_FIXUP))
		meson_g12a_libre_scmi_self_guard(rw_fdt_blob);

	return 0;
}
#endif

void meson_board_add_reserved_memory(void *fdt, u64 start, u64 size)
{
	int ret;

	ret = fdt_add_mem_rsv(fdt, start, size);
	if (ret)
		printf("Could not reserve zone @ 0x%llx\n", start);

	if (IS_ENABLED(CONFIG_EFI_LOADER))
		efi_add_memory_map(start, size, EFI_RESERVED_MEMORY_TYPE);
}

#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT) || \
    defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT)
/*
 * /reserved-memory editing for the open TF-A's DRAM-start secure block.
 *
 * These three helpers live here, not in board-gx.c or board-g12a.c, because
 * BOTH families need exactly the same FDT surgery.  GXL and G12A/G12B/SM1 now
 * share one low map -- 16 MiB reserved, ramoops at 0x00F00000 (operator
 * decision 2026-09-26) -- and only the node SET differs: GXL deletes the
 * hwrom@0 its DTS declares, G12A/G12B/SM1 delete a second secmon their
 * vendor layout used for BL32 and have no hwrom node to name.  The per-family
 * node lists stay in the per-family board file; the mechanics are here.
 *
 * The reservation must create reserved-memory NODES, not merely an FDT
 * /memreserve/.  A memreserve becomes a memblock_reserve, which leaves the
 * region in the kernel's LINEAR MAP; only a no-map node excludes it.  For
 * TZASC-secured memory that is the difference between safe and reachable by a
 * speculative access, which is an external abort.  It runs from
 * ft_board_setup(), i.e. on whatever blob is actually being booted, so it
 * corrects a PXE-supplied or distro DTB too -- which a compile-time overlay of
 * U-Boot's control FDT could not.
 *
 * See amlogic/software/boot/design/dram-reservations.md sections 9, 10 (D1),
 * 10e and 10i.
 */
void meson_del_node(void *fdt, const char *path)
{
	int off = fdt_path_offset(fdt, path);

	if (off == -FDT_ERR_NOTFOUND)
		return;

	if (off < 0 || fdt_del_node(fdt, off))
		printf("meson: could not delete %s\n", path);
}

/*
 * pstore binds exactly ONE ramoops region (fs/pstore/ram.c: "already
 * initialized" when cxt->max_dump_cnt is set), so a second node is not
 * additive -- it is a coin flip over which region records the next crash.
 * The board DTBs carry ramoops@7400000 (meson-libretech-ramoops.dtsi), which
 * under this layout is ALSO destroyed on every boot by the first stage's own
 * DRAM tests.  So the firmware-described region wins, and any other one is
 * removed rather than left to race.
 */
void meson_del_ramoops_nodes(void *fdt)
{
	int rmem, sub, guard;

	for (guard = 0; guard < 32; guard++) {
		rmem = fdt_path_offset(fdt, "/reserved-memory");
		if (rmem < 0)
			return;

		fdt_for_each_subnode(sub, fdt, rmem)
			if (!fdt_node_check_compatible(fdt, sub, "ramoops"))
				break;

		if (sub < 0)
			return;

		if (fdt_del_node(fdt, sub)) {
			printf("meson: could not delete a stale ramoops node\n");
			return;
		}
		/* fdt_del_node() invalidated every offset; re-walk. */
	}
}

/*
 * Add one /reserved-memory child, replacing a same-named one so a re-run does
 * not duplicate.  Returns its offset, or < 0.
 */
int meson_add_rsv_node(void *fdt, const char *name, u32 base, u32 size,
		       bool nomap, const char *compat)
{
	fdt32_t reg[4];
	int rmem, node, ac, sc, n = 0;

	rmem = fdt_path_offset(fdt, "/reserved-memory");
	if (rmem < 0)
		return rmem;

	/* Emit reg in the cell counts this blob's /reserved-memory declares. */
	ac = fdt_address_cells(fdt, rmem);
	sc = fdt_size_cells(fdt, rmem);
	if (ac < 1 || ac > 2 || sc < 1 || sc > 2) {
		printf("meson: /reserved-memory has #address-cells %d #size-cells %d\n",
		       ac, sc);
		return -FDT_ERR_BADNCELLS;
	}

	if (ac == 2)
		reg[n++] = cpu_to_fdt32(0);
	reg[n++] = cpu_to_fdt32(base);
	if (sc == 2)
		reg[n++] = cpu_to_fdt32(0);
	reg[n++] = cpu_to_fdt32(size);

	node = fdt_subnode_offset(fdt, rmem, name);
	if (node >= 0) {
		if (fdt_del_node(fdt, node))
			return -FDT_ERR_INTERNAL;
		rmem = fdt_path_offset(fdt, "/reserved-memory");
		if (rmem < 0)
			return rmem;
	}

	node = fdt_add_subnode(fdt, rmem, name);
	if (node < 0)
		return node;

	/*
	 * A reserved-memory child with reg + no-map and NO compatible is the
	 * upstream-recognised form for "firmware owns this, keep the OS out of
	 * it and out of the linear map".  Nothing binds a driver to it.
	 */
	if (compat && fdt_setprop_string(fdt, node, "compatible", compat))
		return -FDT_ERR_NOSPACE;
	if (fdt_setprop(fdt, node, "reg", reg, n * sizeof(reg[0])))
		return -FDT_ERR_NOSPACE;
	if (nomap && fdt_setprop_empty(fdt, node, "no-map"))
		return -FDT_ERR_NOSPACE;

	return node;
}

#if !defined(CONFIG_XPL_BUILD)
#if defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT)
#define MESON_LIBRE_OPTEE_BASE		GX_LIBRE_OPTEE_BASE
#define MESON_LIBRE_OPTEE_SIZE		GX_LIBRE_OPTEE_SIZE
#else
#define MESON_LIBRE_OPTEE_BASE		G12A_LIBRE_OPTEE_BASE
#define MESON_LIBRE_OPTEE_SIZE		G12A_LIBRE_OPTEE_SIZE
#endif

/*
 * OP-TEE is OPTIONAL in the libre block (operator, 2026-10-17), and it is a
 * FILE, not part of the boot image: with OP-TEE inside it, the image is over
 * the 1 MiB pre-partition budget.  BL31 is built with SPD=opteed and
 * OPTEE_ALLOW_SMC_LOAD, so it starts no BL32 and instead accepts ONE OP-TEE
 * image from the normal world by SMC.  U-Boot proper is that normal world:
 *
 *   board_late_init()  loads optee/tee.bin from an MMC partition when one is
 *                      there (meson_libre_optee_autoload()).
 *   sm optee           the same load from memory (cmd/meson/sm.c).
 *   ft_board_setup()   describes the band and /firmware/optee to the OS when
 *                      OP-TEE answers; otherwise CLOSES the loader, so no
 *                      later stage can start OP-TEE behind a DT that gave its
 *                      memory to Linux.
 *
 * amlogic/software/boot/design/libre-secure-block-and-optee.md section 2.1.
 *
 * Both questions are the trusted-OS Calls UID, a fast SMC32 in the
 * trusted-OS-end owner range, function 0xFF01, i.e. 0xBF00FF01:
 *
 *   OP-TEE running     its UID 384fb3e0-e7f8-11e3-af63-0002a5d5c51b in w0..w3
 *                      (drivers/tee/optee/optee_msg.h OPTEE_MSG_UID_0..3,
 *                      restated because that header is private to the TEE
 *                      driver these boards do not build);
 *   loader open        TF-A's optee_image_load_uuid
 *                      b1eafba3-5d31-4612-b906-c4c7a4be3cc0, packed by
 *                      smc_uuid_word() (services/spd/opteed/opteed_main.c);
 *   neither            a BL31 without SPD=opteed answers SMC_UNK.
 *
 * The load is NSSMC_OPTEED_CALL_LOAD_IMAGE, 0xBF000002, with the size in
 * w1:w2 and the physical address in w3:w4, high word first
 * (services/spd/opteed/teesmc_opteed.h).  A zero size disables loading for
 * good -- TF-A's own "block off loading" path.
 *
 * Nothing is cached: a load inside this boot changes the answer.
 */
#define MESON_OPTEE_SMC_CALLS_UID	0xBF00FF01UL
#define MESON_OPTEE_SMC_LOAD_IMAGE	0xBF000002UL
#define MESON_OPTEE_UID_0		0x384fb3e0UL
#define MESON_OPTEE_UID_1		0xe7f811e3UL
#define MESON_OPTEE_UID_2		0xaf630002UL
#define MESON_OPTEE_UID_3		0xa5d5c51bUL
#define MESON_OPTEE_LOADER_UID_0	0xa3fbeab1UL
#define MESON_OPTEE_LOADER_UID_1	0x1246315dUL
#define MESON_OPTEE_LOADER_UID_2	0xc7c406b9UL
#define MESON_OPTEE_LOADER_UID_3	0xc03cbea4UL

enum meson_optee_state {
	MESON_OPTEE_NONE,	/* no opteed, or loader closed with no OP-TEE */
	MESON_OPTEE_LOADER,	/* opteed waiting for an image */
	MESON_OPTEE_RUNNING,	/* OP-TEE answers */
};

static enum meson_optee_state meson_libre_optee_state(void)
{
	struct pt_regs regs = { 0 };
	u32 w[4];
	int i;

	regs.regs[0] = MESON_OPTEE_SMC_CALLS_UID;
	smc_call(&regs);
	for (i = 0; i < 4; i++)
		w[i] = regs.regs[i] & 0xffffffffUL;

	if (w[0] == MESON_OPTEE_UID_0 && w[1] == MESON_OPTEE_UID_1 &&
	    w[2] == MESON_OPTEE_UID_2 && w[3] == MESON_OPTEE_UID_3)
		return MESON_OPTEE_RUNNING;
	if (w[0] == MESON_OPTEE_LOADER_UID_0 && w[1] == MESON_OPTEE_LOADER_UID_1 &&
	    w[2] == MESON_OPTEE_LOADER_UID_2 && w[3] == MESON_OPTEE_LOADER_UID_3)
		return MESON_OPTEE_LOADER;
	return MESON_OPTEE_NONE;
}

static bool meson_libre_optee_present(void)
{
	return meson_libre_optee_state() == MESON_OPTEE_RUNNING;
}

/* One load SMC; size 0 closes the loader. Returns TF-A's result. */
static long meson_libre_optee_smc_load(ulong addr, ulong size)
{
	struct pt_regs regs = { 0 };

	regs.regs[0] = MESON_OPTEE_SMC_LOAD_IMAGE;
	regs.regs[1] = upper_32_bits(size);
	regs.regs[2] = lower_32_bits(size);
	regs.regs[3] = upper_32_bits(addr);
	regs.regs[4] = lower_32_bits(addr);
	smc_call(&regs);

	return (long)regs.regs[0];
}

/*
 * U-Boot's own map: with OP-TEE running, nothing U-Boot loads may land in
 * its band.  LMB passes the reservation on to the EFI memory map.
 */
static void meson_libre_optee_lmb(void)
{
	phys_addr_t addr = MESON_LIBRE_OPTEE_BASE;

	if (lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &addr, MESON_LIBRE_OPTEE_SIZE,
			  LMB_NOMAP))
		printf("meson: could not reserve the OP-TEE band in LMB\n");
}

/*
 * Where the image sits when U-Boot issues the load SMC.  opteed maps the
 * source with mmap_add_dynamic_region(), which refuses any overlap with a
 * STATIC BL31 region (lib/xlat_tables_v2/xlat_tables_core.c, -EPERM), and
 * BL31 maps non-secure DRAM statically up to 0x10000000 plus the MiB at
 * 0x10000000 (plat/amlogic/{gxl,g12a,g12b}/*_def.h).  So the source has to
 * sit above 0x10100000.  0x10200000 + the band's 12 MiB ends at 0x10E01000,
 * inside the free memory of the smallest (512 MiB) board.  Measured: a source
 * at ramdisk_addr_r 0x0D600000 was refused with -1 -- and since opteed clears
 * its one-shot flag before mapping, that also closed the loader.
 */
#define MESON_LIBRE_OPTEE_STAGE		0x10200000UL

/*
 * Hand the image at [addr, addr + size) -- the v2-header file LBS publishes
 * as out/<target>.tee.bin -- to opteed.  0 on success.
 */
int meson_libre_optee_load(ulong addr, ulong size)
{
	long ret;

	if (meson_libre_optee_state() != MESON_OPTEE_LOADER) {
		printf("meson: no OP-TEE loader to hand an image to\n");
		return -ENODEV;
	}
	if (!size || size > MESON_LIBRE_OPTEE_SIZE + SZ_4K) {
		printf("meson: OP-TEE image of %lu B does not fit its band\n", size);
		return -EINVAL;
	}
	if (MESON_LIBRE_OPTEE_STAGE + size > gd->ram_top) {
		printf("meson: no room to stage the OP-TEE image\n");
		return -ENOMEM;
	}

	if (addr != MESON_LIBRE_OPTEE_STAGE)
		memmove((void *)MESON_LIBRE_OPTEE_STAGE, (void *)addr, size);
	addr = MESON_LIBRE_OPTEE_STAGE;
	flush_dcache_range(addr, ALIGN(addr + size, ARCH_DMA_MINALIGN));
	ret = meson_libre_optee_smc_load(addr, size);
	if (ret || !meson_libre_optee_present()) {
		printf("meson: OP-TEE load refused (%ld)\n", ret);
		return -EIO;
	}

	meson_libre_optee_lmb();
	printf("meson: OP-TEE loaded from 0x%lx (%lu B)\n", addr, size);
	return 0;
}

/*
 * Look for the OP-TEE file on the MMC devices U-Boot already knows, first
 * partitions first, and load it if found.  The file is staged in the kernel
 * slot, which nothing occupies this early.  Silence when there is no loader
 * (a BL31 without opteed) or no file: OP-TEE is optional.
 */
static void meson_libre_optee_autoload(void)
{
	static const char * const paths[] = {
		"/optee/tee.bin",
		"/boot/optee/tee.bin",
	};
	struct blk_desc *desc;
	ulong addr = env_get_hex("kernel_addr_r", 0);
	loff_t size, got;
	int dev, part, i;

	if (!addr || meson_libre_optee_state() != MESON_OPTEE_LOADER)
		return;

	for (dev = 0; dev < 3; dev++) {
		desc = blk_get_devnum_by_uclass_id(UCLASS_MMC, dev);
		if (!desc)
			continue;
		for (part = 1; part <= 4; part++) {
			for (i = 0; i < ARRAY_SIZE(paths); i++) {
				if (fs_set_blk_dev_with_part(desc, part) ||
				    fs_size(paths[i], &size))
					continue;
				if (fs_set_blk_dev_with_part(desc, part) ||
				    fs_read(paths[i], addr, 0, 0, &got) ||
				    got != size)
					continue;
				printf("meson: OP-TEE image mmc %d:%d %s\n", dev,
				       part, paths[i]);
				meson_libre_optee_load(addr, got);
				return;
			}
		}
	}
}

/*
 * The OS's map: on an OP-TEE answer, add
 *
 *   /reserved-memory/optee-core@400000   reg = band, no-map
 *   /firmware/optee                      compatible = "linaro,optee-tz",
 *                                        method = "smc"
 *
 * Upstream bindings only (reserved-memory, linaro,optee-tz), so an unpatched
 * kernel binds its OP-TEE driver.  No shared-memory node: OP-TEE runs with
 * dynamic shared memory.  Called from meson_init_reserved_memory(), after the
 * family's block nodes exist, on whatever blob is being booted.
 *
 * On GX it says, once, that the band is not hardware-isolated: GXL's DMC
 * secure ranges do not deny the CPU port (design section 3.1), so OP-TEE's
 * memory is protected from the normal world only by this reservation.
 */
void meson_libre_optee_fixup(void *fdt)
{
	int fw, node;

	switch (meson_libre_optee_state()) {
	case MESON_OPTEE_RUNNING:
		break;
	case MESON_OPTEE_LOADER:
		/* Nothing was loaded: close the door before the OS. */
		if (meson_libre_optee_smc_load(0, 0) &&
		    meson_libre_optee_state() == MESON_OPTEE_LOADER)
			printf("meson: could not close the OP-TEE loader\n");
		return;
	default:
		return;
	}

	if (meson_add_rsv_node(fdt, "optee-core@400000", MESON_LIBRE_OPTEE_BASE,
			       MESON_LIBRE_OPTEE_SIZE, true, NULL) < 0)
		goto fail;

	fw = fdt_path_offset(fdt, "/firmware");
	if (fw < 0) {
		fw = fdt_add_subnode(fdt, 0, "firmware");
		if (fw < 0)
			goto fail;
	}

	node = fdt_subnode_offset(fdt, fw, "optee");
	if (node < 0) {
		node = fdt_add_subnode(fdt, fw, "optee");
		if (node < 0)
			goto fail;
	}

	if (fdt_setprop_string(fdt, node, "compatible", "linaro,optee-tz") ||
	    fdt_setprop_string(fdt, node, "method", "smc") ||
	    fdt_setprop_string(fdt, node, "status", "okay"))
		goto fail;

	printf("meson: OP-TEE present; reserved 0x%08x + 0x%08x for it\n",
	       MESON_LIBRE_OPTEE_BASE, MESON_LIBRE_OPTEE_SIZE);
	if (IS_ENABLED(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT))
		printf("meson: GXL cannot firewall DRAM; OP-TEE memory is protected by this reservation only\n");
	return;
fail:
	printf("meson: OP-TEE present but its DT nodes could not be added\n");
}
#endif /* !CONFIG_XPL_BUILD */

#endif /* CONFIG_MESON_{GX,G12A}_LIBRE_SECURE_LAYOUT */

int meson_generate_serial_ethaddr(void)
{
	u8 mac_addr[ARP_HLEN];
	char serial[SM_SERIAL_SIZE];
	u32 sid;
	u16 sid16;

	if (!meson_sm_get_serial(serial, SM_SERIAL_SIZE)) {
		sid = crc32(0, (unsigned char *)serial, SM_SERIAL_SIZE);
		sid16 = crc16_ccitt(0, (unsigned char *)serial,	SM_SERIAL_SIZE);

		/* Ensure the NIC specific bytes of the mac are not all 0 */
		if ((sid & 0xffffff) == 0)
			sid |= 0x800000;

		/* Non OUI / registered MAC address */
		mac_addr[0] = ((sid16 >> 8) & 0xfc) | 0x02;
		mac_addr[1] = (sid16 >>  0) & 0xff;
		mac_addr[2] = (sid >> 24) & 0xff;
		mac_addr[3] = (sid >> 16) & 0xff;
		mac_addr[4] = (sid >>  8) & 0xff;
		mac_addr[5] = (sid >>  0) & 0xff;

		eth_env_set_enetaddr("ethaddr", mac_addr);
	} else
		return -EINVAL;

	return 0;
}

static void meson_set_boot_source(void)
{
	const char *source;

	switch (meson_get_boot_device()) {
	case BOOT_DEVICE_EMMC:
		source = "emmc";
		break;

	case BOOT_DEVICE_NAND:
		source = "nand";
		break;

	case BOOT_DEVICE_SPI:
		source = "spi";
		break;

	case BOOT_DEVICE_SD:
		source = "sd";
		break;

	case BOOT_DEVICE_USB:
		source = "usb";
		break;

	default:
		source = "unknown";
	}

	env_set("boot_source", source);
}

__weak int meson_board_late_init(void)
{
	return 0;
}

int board_late_init(void)
{
#if !defined(CONFIG_XPL_BUILD) && \
    (defined(CONFIG_MESON_GX_LIBRE_SECURE_LAYOUT) || \
     defined(CONFIG_MESON_G12A_LIBRE_SECURE_LAYOUT))
	meson_libre_optee_autoload();
#endif
	meson_set_boot_source();

	if (CONFIG_IS_ENABLED(DFU) && CONFIG_IS_ENABLED(EFI_LOADER)) {
		/* Generate dfu_string for EFI capsule updates */
		meson_setup_capsule();
	}

	return meson_board_late_init();
}

#if defined(CONFIG_XPL) || !CONFIG_IS_ENABLED(SYSRESET)
void reset_cpu(void)
{
	/*
	 * We do not have BL31 running yet, so no PSCI.
	 * Instead, let the watchdog reset the board.
	 */
	for (;;)
		;
}
#endif
