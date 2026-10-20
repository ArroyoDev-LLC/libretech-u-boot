// SPDX-License-Identifier: GPL-2.0+
/*
 * G12B Alta (AML-A311D-CC) TPL power init — dual cluster VCCK_A/B.
 *
 * PWM_AO_D = VCCK duty (one PWM).  Cluster enables are GPIOs, not a
 * second PWM:
 *   VCCK_A / CPUA (A73): GPIOAO_4 high  — pwr_ctrl.c power_on_at_24M
 *   VCCK_B / CPUB (A53) + VCC3V3: TEST_N AO_GPIO_O[31] + AO_SEC_REG0[0]
 *     — boot-lite board/aml-a311d-cc/firmware/timing.c __bl2_reg
 * VDDEE = PWM_AO_B (same 840 mV tables as Solitude).
 *
 * Strong meson_power_init(); linked with CONFIG_MESON_G12B_POWER_INIT,
 * independently of CONFIG_MESON_G12_OPEN_PLL (which selects it), so the
 * SM1 Strategy A orchestrator in dram-g12a.c can run on Alta.
 */

#include <init.h>
#include <config.h>
#include <asm/io.h>
#include <asm/arch/g12a-ddr-fw.h>
#include <linux/types.h>

static void g12b_reg_clrset(ulong addr, u32 mask, u32 value)
{
	u32 v = readl((void *)addr);

	writel((v & ~mask) | value, (void *)addr);
}

void meson_power_init(void)
{
	u32 por, hw_id, vddee;

	/* VCCK PWM_AO_D 800 mV (Alta timing.c VCCK_VAL_REG1 / 36-step revB) */
	g12b_reg_clrset(G12A_AO_PWM_PWM_D, 0xffffffffU, G12B_ALTA_VCCK_PWM);
	g12b_reg_clrset(G12A_AO_PWM_MISC_CD, (0x7fU << 16),
			(1U << 23) | (1U << 1));
	/* AO_PIN_MUX_REG1[23:20] = 3 → PWM_D */
	g12b_reg_clrset(G12A_AO_RTI_PIN_MUX_REG1, (0xfU << 20), (3U << 20));

	/* VCCK_B / TEST_N: AO_SEC_REG0 bit0 + AO_GPIO_O bit31 */
	g12b_reg_clrset(G12A_AO_SEC_REG0, 0xffffffffU, (1U << 0));
	g12b_reg_clrset(G12A_AO_GPIO_O_EN_N, (1U << 31), 0);
	g12b_reg_clrset(G12A_AO_GPIO_O, 0, (1U << 31));

	/* VCCK_A / CPUA: GPIOAO_4 output high, mux GPIO */
	g12b_reg_clrset(G12A_AO_RTI_PIN_MUX_REG, (0xfU << 16), 0);
	g12b_reg_clrset(G12A_AO_GPIO_O_EN_N, (1U << 4), 0);
	g12b_reg_clrset(G12A_AO_GPIO_O, 0, (1U << 4));

	/* VDDEE PWM_AO_B: POR bit7 selects CFG0 vs CFG1 (timing.c PWM_CHK_HW) */
	por = readl((void *)G12A_ASSIST_POR_CONFIG);
	hw_id = (por >> 7) & 1U;
	vddee = hw_id ? G12B_ALTA_VDDEE_PWM_CFG0 : G12B_ALTA_VDDEE_PWM_CFG1;
	g12b_reg_clrset(G12A_AO_PWM_PWM_B, 0xffffffffU, vddee);
	g12b_reg_clrset(G12A_AO_PWM_MISC_AB, (0x7fU << 16),
			(1U << 23) | (1U << 1));
	/* AO_PIN_MUX_REG1[19:16] = 3 → PWM_B */
	g12b_reg_clrset(G12A_AO_RTI_PIN_MUX_REG1, (0xfU << 16), (3U << 16));
}
