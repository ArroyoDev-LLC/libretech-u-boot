// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2016 BayLibre, SAS
 * Author: Neil Armstrong <narmstrong@baylibre.com>
 */

#include <dm.h>
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
 * board's DDR3 (Nanya NT5CB256M16DP) across its whole Tc 0..95 C range.  The
 * open chain -- the product default -- writes this value in its SPL
 * (GXL_DDR3_REFI_T100NS in dram-timings-gx.h, operator decision D14), so this
 * is a no-op there.  The opt-in vendor chain (libretech-cc_defconfig) trains
 * with the vendor BL2, which leaves 76 (7.6 us), legal only to 85 C, and
 * nothing can read the DRAM's temperature on this board.  So U-Boot proper,
 * the first stage of ours in that chain, lowers the field.
 */
#define AML_S905X_CC_REFI_T100NS	38
#define DMC_REFR_CTRL2_TREFI		GENMASK(15, 8)

static void aml_s905x_cc_fix_refresh(void)
{
	u32 v = readl(DMC_REFR_CTRL2);
	u32 refi = FIELD_GET(DMC_REFR_CTRL2_TREFI, v);

	/*
	 * Lower only: the other fields (tZQCI, tPVTI and the t100ns prescaler
	 * the trainer derived from its clock) stay as the trainer set them.  A
	 * live tREFI change under traffic is safe on this DMC: 76 -> 38 -> 76
	 * from Linux under a verify load, 0 errors in 177 passes (amlogic n832).
	 * The line is printed only when the value is changed, so a boot log says
	 * whether the trainer left it short of the rule.
	 */
	if (refi <= AML_S905X_CC_REFI_T100NS)
		return;
	writel((v & ~DMC_REFR_CTRL2_TREFI) |
	       FIELD_PREP(DMC_REFR_CTRL2_TREFI, AML_S905X_CC_REFI_T100NS),
	       DMC_REFR_CTRL2);
	printf("DRAM:  tREFI %u -> %u x 100 ns (DDR3 above 85 C)\n", refi,
	       (u32)FIELD_GET(DMC_REFR_CTRL2_TREFI, readl(DMC_REFR_CTRL2)));
}

#if IS_ENABLED(CONFIG_SET_DFU_ALT_INFO)
void set_dfu_alt_info(char *interface, char *devstr){
	meson_set_dfu_alt_info(interface, devstr);
}
#endif

int misc_init_r(void)
{
	aml_s905x_cc_fix_refresh();
	meson_gxl_misc_init_r();

	return 0;
}
