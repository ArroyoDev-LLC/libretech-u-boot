// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <dm.h>
#include <env.h>
#include <init.h>
#include <net.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/arch/gx.h>
#include <asm/arch/sm.h>
#include <asm/arch/eth.h>
#include <asm/arch/mem.h>
#include <asm/arch/dram-gx.h>
#include "../common/board-meson.h"

/*
 * DMC_REFR_CTRL2[15:8] (tREFI) in t100ns ticks: 38 = 3.8 us, legal for the
 * board's DDR4 (SK hynix H5AN8G6NAFR, 3.9 us required at Tc 85..95 C) across
 * its whole range.  The open chain -- the product default -- trains in our
 * TPL at 38 and the SC governor relaxes it while cool, so this is a no-op
 * there.  The opt-in vendor chain (libretech-cc_v2_defconfig, and the SPI
 * image libretech-cc_v2_spi_defconfig) trains with the vendor BL2, which
 * leaves 76 (7.6 us), legal only to 85 C, and nothing reads the DRAM's
 * temperature on that chain.  So U-Boot proper, the first stage of ours there,
 * lowers the field -- as aml-s905x-cc.c does for the DDR3 board.
 *
 * What it cannot do: the vendor BL2 also trains with read DBI ON, the set that
 * inverted one byte hot on this board (amlogic gxl dram n1231); turning it off
 * means re-training, which only our own TPL does.  The open chain is the fix.
 */
#define AML_S905X_CC_V2_REFI_T100NS	38
#define DMC_REFR_CTRL2_TREFI		GENMASK(15, 8)

static void aml_s905x_cc_v2_fix_refresh(void)
{
	u32 v = readl(DMC_REFR_CTRL2);
	u32 refi = FIELD_GET(DMC_REFR_CTRL2_TREFI, v);

	/*
	 * Lower only, tREFI only: tZQCI/tPVTI and the t100ns prescaler stay as
	 * the trainer set them.  A live tREFI change under traffic is how the
	 * SC governor runs on this board's open chain (amlogic n1233, 5/5).
	 */
	if (refi <= AML_S905X_CC_V2_REFI_T100NS)
		return;
	writel((v & ~DMC_REFR_CTRL2_TREFI) |
	       FIELD_PREP(DMC_REFR_CTRL2_TREFI, AML_S905X_CC_V2_REFI_T100NS),
	       DMC_REFR_CTRL2);
	printf("DRAM:  tREFI %u -> %u x 100 ns (DDR4 above 85 C)\n", refi,
	       (u32)FIELD_GET(DMC_REFR_CTRL2_TREFI, readl(DMC_REFR_CTRL2)));
}

#define EFUSE_SN_OFFSET		20
#define EFUSE_SN_SIZE		16
#define EFUSE_MAC_OFFSET	52
#define EFUSE_MAC_SIZE		6

#if IS_ENABLED(CONFIG_SET_DFU_ALT_INFO)
void set_dfu_alt_info(char *interface, char *devstr){
	meson_set_dfu_alt_info(interface, devstr);
}
#endif

int misc_init_r(void)
{
	aml_s905x_cc_v2_fix_refresh();
	meson_gxl_misc_init_r();

	return 0;
}
