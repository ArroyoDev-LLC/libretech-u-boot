// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * Tell the OS about BL32 (OP-TEE) from what OP-TEE itself wrote.
 *
 * When MediaTek BL2 hands a BL32, it gives BL32 a zeroed non-secure buffer
 * (BL32 x2, OP-TEE's external DT address) and gives BL33 the same address in
 * x1 as ('LCOT' << 32) | PA (TF-A plat/mediatek/mt8183/bl2_plat_setup.c,
 * MTK_BL33_ARG1_BL32_DT_TAG). OP-TEE built with CFG_GENERATE_DTB_OVERLAY
 * writes a DT overlay there before its first exit to the normal world:
 * /firmware/optee (compatible "linaro,optee-tz", method "smc") and its own
 * TZDRAM as a no-map /reserved-memory child. This file
 *
 *   1. copies that overlay at EVT_SETTINGS_R, before any boot command can
 *      load an image over the buffer, and reserves the overlay's
 *      reserved-memory ranges in LMB so U-Boot never loads into them;
 *   2. applies it to the OS DT in ft_system_setup(), which runs for bootm and
 *      for the EFI DT alike.
 *
 * So the OS DT gains OP-TEE's node and reservation exactly when OP-TEE ran
 * on this boot, at the address and size OP-TEE was built for; no DT carries
 * a static copy. With no tag (BL2 handed no BL32) this file does nothing. A
 * tag with no valid overlay means BL32 did not write one on this boot (BL2
 * zeroes the buffer first); U-Boot says so and adds nothing.
 *
 * If the overlay cannot be applied, ft_system_setup() fails and the boot
 * stops: an OS that does not know about a running OP-TEE would hand its
 * memory out.
 */

#include <event.h>
#include <fdt_support.h>
#include <lmb.h>
#include <malloc.h>
#include <mapmem.h>
#include <stdio.h>
#include <asm/global_data.h>
#include <linux/errno.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>
#include <linux/string.h>

DECLARE_GLOBAL_DATA_PTR;

#define MTK_BL33_ARG1_BL32_DT_TAG	0x4c434f54ULL	/* 'LCOT' */
/* OP-TEE maps CFG_DTB_MAX_SIZE (default 64 KiB) at the address it is given */
#define MTK_BL32_DT_MAX			SZ_64K

extern unsigned long mtk_bl33_arg1;

static void *bl32_dto;

static void bl32_dt_reserve(const void *dto)
{
	int frag, ov, rm, node, ac, sc, len;
	const fdt32_t *reg;
	phys_addr_t addr;
	phys_size_t size;
	int ret;

	fdt_for_each_subnode(frag, dto, 0) {
		ov = fdt_subnode_offset(dto, frag, "__overlay__");
		if (ov < 0)
			continue;
		rm = fdt_subnode_offset(dto, ov, "reserved-memory");
		if (rm < 0)
			continue;
		ac = fdt_address_cells(dto, rm);
		sc = fdt_size_cells(dto, rm);
		fdt_for_each_subnode(node, dto, rm) {
			reg = fdt_getprop(dto, node, "reg", &len);
			if (!reg || ac < 1 || sc < 1 ||
			    len < (ac + sc) * (int)sizeof(fdt32_t))
				continue;
			addr = fdt_read_number(reg, ac);
			size = fdt_read_number(reg + ac, sc);
			ret = lmb_alloc_mem(LMB_MEM_ALLOC_ADDR, 0, &addr, size,
					    LMB_NOMAP);
			printf("OP-TEE: %s %#llx..%#llx%s\n",
			       fdt_get_name(dto, node, NULL),
			       (unsigned long long)addr,
			       (unsigned long long)(addr + size - 1),
			       ret ? " (LMB reserve failed)" : "");
		}
	}
}

static int mtk_bl32_dt_settings_r(void)
{
	u64 arg1 = mtk_bl33_arg1;
	phys_addr_t pa;
	const void *src;
	u32 size;

	if ((arg1 >> 32) != MTK_BL33_ARG1_BL32_DT_TAG)
		return 0;

	pa = arg1 & 0xffffffffULL;
	if (pa < gd->ram_base ||
	    pa + MTK_BL32_DT_MAX > gd->ram_base + gd->ram_size) {
		printf("OP-TEE: BL2 DT buffer %#llx is outside DRAM; ignored\n",
		       (unsigned long long)pa);
		return 0;
	}

	src = map_sysmem(pa, MTK_BL32_DT_MAX);
	if (fdt_check_header(src) || fdt_totalsize(src) > MTK_BL32_DT_MAX) {
		printf("OP-TEE: no DT overlay at %#llx (BL32 wrote none on this boot); OS DT unchanged\n",
		       (unsigned long long)pa);
		unmap_sysmem(src);
		return 0;
	}

	size = fdt_totalsize(src);
	bl32_dto = malloc(size);
	if (!bl32_dto) {
		unmap_sysmem(src);
		return -ENOMEM;
	}
	memcpy(bl32_dto, src, size);
	unmap_sysmem(src);

	printf("OP-TEE: DT overlay from BL32 at %#llx, %u bytes\n",
	       (unsigned long long)pa, size);
	bl32_dt_reserve(bl32_dto);

	return 0;
}
EVENT_SPY_SIMPLE(EVT_SETTINGS_R, mtk_bl32_dt_settings_r);

int ft_system_setup(void *blob, struct bd_info *bd)
{
	void *dto;
	u32 size;
	int ret;

	if (!bl32_dto)
		return 0;

	/* fdt_overlay_apply() consumes the overlay: apply a copy */
	size = fdt_totalsize(bl32_dto);
	dto = malloc(size);
	if (!dto)
		return -FDT_ERR_NOSPACE;
	memcpy(dto, bl32_dto, size);

	ret = fdt_overlay_apply_verbose(blob, dto);
	free(dto);
	if (ret) {
		printf("OP-TEE: DT overlay NOT applied; refusing to boot an OS that would not know OP-TEE runs\n");
		return ret;
	}
	printf("OP-TEE: DT overlay applied to the OS DT\n");

	return 0;
}
