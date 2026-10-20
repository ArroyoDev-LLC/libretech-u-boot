// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 MediaTek Inc.
 * Copyright (C) 2021 BayLibre, SAS
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <clk.h>
#include <dm.h>
#include <fdtdec.h>
#include <ram.h>
#include <asm/arch/misc.h>
#include <asm/armv8/mmu.h>
#include <asm/sections.h>
#include <asm/system.h>
#include <dm/uclass.h>
#include <dt-bindings/clock/mt8516-clk.h>
#include <linux/kernel.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * DRAM size: taken from the stage that measured it, never from a constant.
 *
 * MT8183 DRAM parts are selected at boot by BL2's libdram (MR5 + the MR8 rank
 * sizes pick the EMI record), so one image serves the 2 GiB and 4 GiB
 * assemblies and the DT /memory node cannot know which one it is on. BL2 hands
 * the size it brought up to BL33 in x3 (arch/arm/mach-mediatek/lowlevel_init.S):
 * 'LCDR' in the high 32 bits, the size in MiB in the low 32 -- the same
 * encoding mt8188/init.c reads. With it, U-Boot sizes gd->ram_size, maps that
 * much DDR in its MMU table, and (CONFIG_ARCH_FIXUP_FDT_MEMORY) rewrites the
 * OS DT's /memory at boot, so the DT values are placeholders.
 *
 * A BL2 that does not tag x3 (the MediaTek BSP chain, an older fork BL2)
 * keeps the DT size, and says so.
 */
#define MTK_BL33_ARG3_DRAM_TAG		0x4c434452ULL
#define MT8183_DRAM_SIZE_MIN		SZ_512M
#define MT8183_DRAM_SIZE_MAX		0x200000000ULL	/* 8 GiB: the EMI's reach */

extern unsigned long mtk_bl33_arg3;

/*
 * True when BL2 tagged x3, with the size it carries in *size -- which may be
 * 0: a tagged 0 MiB is an implausible size, not a missing one, and the
 * console has to say which of the two fallbacks was taken.
 */
static bool mt8183_bl2_dram_size(phys_size_t *size)
{
	u64 arg3 = mtk_bl33_arg3;

	if ((arg3 >> 32) != MTK_BL33_ARG3_DRAM_TAG)
		return false;

	*size = (phys_size_t)(arg3 & 0xffffffffULL) << 20;
	return true;
}

int dram_init(void)
{
	phys_size_t size;
	int ret;

	/* The DT supplies the base; its size is only the untagged fallback. */
	ret = fdtdec_setup_mem_size_base();
	if (ret)
		return ret;

	if (!mt8183_bl2_dram_size(&size)) {
		printf("DRAM: BL2 passed no size (x3 %#llx, no 'LCDR' tag); keeping DT %#llx\n",
		       (unsigned long long)mtk_bl33_arg3,
		       (unsigned long long)gd->ram_size);
		return 0;
	}
	if (size < MT8183_DRAM_SIZE_MIN || size > MT8183_DRAM_SIZE_MAX) {
		printf("DRAM: BL2 size %#llx implausible; keeping DT %#llx\n",
		       (unsigned long long)size,
		       (unsigned long long)gd->ram_size);
		return 0;
	}

	gd->ram_size = size;
	/*
	 * mem_map[0] is the DDR region (armv8-mem-map.c). Map exactly the
	 * DRAM BL2 brought up: no more (an unbacked normal-memory mapping can
	 * be speculatively fetched), no less (U-Boot relocates to the top).
	 * dram_init() runs before reserve_mmu() sizes the page tables.
	 */
	mem_map[0].size = size;

	return 0;
}

phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	/*
	 * Keep U-Boot and its buffers below 4 GiB: MMC and USB DMA take 32-bit
	 * addresses. Linux still gets all of it through the /memory fixup.
	 */
	return min_t(phys_addr_t, gd->ram_top, SZ_4G);
}

void reset_cpu(void)
{
	psci_system_reset();
}

int print_cpuinfo(void)
{
	printf("CPU:   MediaTek MT8183\n");
	return 0;
}
