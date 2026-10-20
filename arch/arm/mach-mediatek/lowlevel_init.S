/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * Capture BL33 x2 and x3 before BSS clear.
 *
 * x2: MediaTek BL2 writes MTK_BL33_ARG2_USB ('LCUS', 0x4c435553) into
 * ep_info.args.arg2 when usb_dl_active — the FIP was received over
 * USB/UART DRAM download, not from the GPIO storage strap.
 *
 * x3: BL2 writes ('LCDR' << 32) | size_MiB, the DRAM size it brought up
 * (TF-A plat/mediatek/common/mtk_bl2_common.c, MTK_BL33_ARG3_DRAM_TAG).
 * Untagged when the BL2 does not report one.
 *
 * arg0/arg1 are claimed by TF-A common/desc_image_load.c
 * (fw_config / hw_config on non-head images).
 * Must live in .data: save_boot_params runs before BSS is zeroed.
 *
 * Pair: TF-A plat/mediatek/common/mtk_bl2_common.c, and
 * arch/arm/mach-mediatek/bootsource.c (x2), mt8183/init.c and
 * mt8188/init.c dram_init (x3).
 */

#include <linux/linkage.h>

	.data
	.align	3
	.global	mtk_bl33_arg2
mtk_bl33_arg2:
	.quad	0
	.global	mtk_bl33_arg3
mtk_bl33_arg3:
	.quad	0

	.text
ENTRY(save_boot_params)
	adrp	x8, mtk_bl33_arg2
	add	x8, x8, :lo12:mtk_bl33_arg2
	str	x2, [x8]
	adrp	x8, mtk_bl33_arg3
	add	x8, x8, :lo12:mtk_bl33_arg3
	str	x3, [x8]
	b	save_boot_params_ret
ENDPROC(save_boot_params)
