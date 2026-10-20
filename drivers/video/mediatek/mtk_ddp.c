// SPDX-License-Identifier: GPL-2.0
/*
 * Mediatek Display Data Path
 *
 * Copyright (c) 2022 BayLibre, SAS.
 * Author: Julien Masson <jmasson@baylibre.com>
 */

#include <asm/io.h>
#include <dm.h>
#include <dm/device_compat.h>

#define DISP_REG_CONFIG_DISP_RDMA1_SOUT_SEL   0xfd0
#define DISP_REG_CONFIG_DISP_DPI0_SEL_IN      0xfd8
#define DISP_REG_CONFIG_DISP_LVDS_SYS_CFG_00  0xfdc

/*
 * Configure the MMSYS display data path so RDMA1's output is routed to DPI0
 * (RDMA1_SOUT -> DPI0, DPI0_SEL_IN <- RDMA1, LVDS sys cfg -> DPI mode).
 *
 * The MMSYS node (mediatek,mt8365-mmsys) is owned by the UCLASS_CLK clock-gate
 * provider (clk-mt8365.c) so the <&mmsys CLK_MM_*> phandles resolve. u-boot's
 * 2026.04 DM core binds a single driver per node (no DM_ALLOW_MULTIPLE_DRIVERS
 * in this tree, unlike the BayLibre BSP), so the routing is done by mapping the
 * MMSYS register window from its ofnode directly here, instead of binding a
 * separate UCLASS_MISC device to the same node.
 */
void mtk_ddp_rdma_to_dpi(void __iomem *mmsys_base)
{
	writel(BIT(0), mmsys_base + DISP_REG_CONFIG_DISP_RDMA1_SOUT_SEL);

	writel(0, mmsys_base + DISP_REG_CONFIG_DISP_DPI0_SEL_IN);

	writel(BIT(0), mmsys_base + DISP_REG_CONFIG_DISP_LVDS_SYS_CFG_00);
}
