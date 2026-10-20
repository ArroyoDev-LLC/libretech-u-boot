// SPDX-License-Identifier: GPL-2.0+
/*
 * Portions Copyright (C) 2015, Amlogic, Inc. All rights reserved.
 * Copyright (C) 2023, Ferass El Hafidi <funderscore@postmarketos.org>
 */
#include <spl.h>
#include <asm/io.h>
#include <asm/arch/gx.h>

/*
 * RUN R -- give GXL a real meson_power_init(), called from spl.c immediately
 * before dram_init().  Until now that call resolved to the __weak no-op in
 * spl.c, so the open SPL programmed NO supply rail at all: measured live
 * 2026-08-10 (RUN Q, 2 boots per firmware), our SPL left PWM_PWM_B
 * (0xc1108554) at its reset default 0x00500008 and PWM_MISC_REG_AB
 * (0xc1108558) at 0x00000000 -- the PWM block not even enabled -- while
 * product BL2 on the identical USB-BROM path had 0x000e000e / 0x00800002.
 *
 * On AML-S905X-CC-V2, PWM_B carries VDDEE (EE core logic -- the DDR PHY's
 * digital logic and its master delay lines), and 0x000e000e is the vendor
 * BL2 voltage table's 1000 mV entry.  Both words below are product's own
 * measured values; nothing is invented and nothing is swept.
 *
 * The duty word is PER BOARD (CONFIG_MESON_GXL_VDDEE_PWM_B, 0 = leave the
 * rail alone), because it comes from that board's own vendor BL2 voltage
 * table.  It is measured for AML-S905X-CC-V2 only; every other GXL board
 * needs its own measurement before it is set.
 *
 * VDDEE ONLY.  The VCCK half (PWM_D) is deliberately NOT enabled here: three
 * latent defects still stand in that path -- gx.h's GX_PWM_PWM_D 0x2195 /
 * GX_PWM_MISC_REG_CD 0x2196 are off by four words from the datasheet's
 * 0x2191 / 0x2192, meson_power_init_gxl() below writes GX_PWM_PWM_B where
 * PWM_D was meant, and its rail labels are inverted for this board.  Fixing
 * those is a separate change; enabling VCCK here would be a second variable.
 */
void meson_power_init(void)
{
#if CONFIG_MESON_GXL_VDDEE_PWM_B
	/*
	 * PWM_MISC_REG_AB (0xc1108558): clear the PWM_B pre-divider [22:16],
	 * set bit 23 (PWM_B output enable) and bit 1 (PWM_B clock enable).
	 * From reset 0 this lands product's measured live 0x00800002.
	 */
	clrsetbits_32(GX_PWM_MISC_REG_AB, 0x7f << 16, (1 << 23) | (1 << 1));

	/*
	 * PWM_PWM_B (0xc1108554) = {high, low} duty pair for VDDEE 1000 mV,
	 * the vendor BL2 table entry product programs on this path.
	 */
	writel(CONFIG_MESON_GXL_VDDEE_PWM_B, GX_PWM_PWM_B);

	/*
	 * Route PWM_B to its pad, or the duty word never reaches the
	 * regulator: PIN_MUX_REG1 (0xda8344b4) clear bit 10, PIN_MUX_REG2
	 * (0xda8344b8) clear bit 5 / set bit 11.  Product BL2 performs the
	 * same three RMWs on the same two registers.
	 */
	clrbits_32(GX_PIN_MUX_REG1, 1 << 10);
	clrsetbits_32(GX_PIN_MUX_REG2, 1 << 5, 1 << 11);
#endif
}

/* Unused (referenced nowhere) -- kept untouched; see the VCCK note above. */
void meson_power_init_gxl(void)
{
	/* TODO: Support more voltages */

	/* Init PWM B */
	clrsetbits_32(GX_PWM_MISC_REG_AB, 0x7f << 16, (1 << 23) | (1 << 1));

	/* Set voltage */
	if (CONFIG_IS_ENABLED(MESON_GX_VCCK_1120MV))
		writel(0x02001a, GX_PWM_PWM_B);
	else if (CONFIG_IS_ENABLED(MESON_GX_VCCK_1100MV))
		writel(0x040018, GX_PWM_PWM_B);
	else if (CONFIG_IS_ENABLED(MESON_GX_VCCK_1000MV))
		writel(0x0e000e, GX_PWM_PWM_B);

	clrbits_32(GX_PIN_MUX_REG1, 1 << 10);
	clrsetbits_32(GX_PIN_MUX_REG2, 1 << 5, 1 << 11);

	/* Init PWM D */
	clrsetbits_32(GX_PWM_MISC_REG_CD, 0x7f << 16, (1 << 23) | (1 << 1));

	/* Set voltage */
	if (CONFIG_IS_ENABLED(MESON_GX_VDDEE_1100MV))
		writel(0x040018, GX_PWM_PWM_B);
	else if (CONFIG_IS_ENABLED(MESON_GX_VDDEE_1000MV))
		writel(0x0e000e, GX_PWM_PWM_B);

	clrbits_32(GX_PIN_MUX_REG1, (1 << 9) | (1 << 11));
	setbits_32(GX_PIN_MUX_REG2, 1 << 12);
}
