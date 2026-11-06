// SPDX-License-Identifier: GPL-2.0+
/*
 * `bootrom` on G12A and SM1: the BootROM override word.
 *
 * AO_RTI_STATUS_REG2 = 0x4f5244b1 makes the SM1 BootROM enter USB before any
 * storage and wait for the host with no timeout (ROM 0xffff68c8/cc; measured:
 * it survives a watchdog reset, the ROM tries no storage, a full power cut
 * clears it). The word is secure-only for writes, so U-Boot proper goes through
 * TF-A's SiP SET_USB_BOOT_FUNC (0x82000043): FORCE (2) writes it, CLEAR (1)
 * zeroes it, and the call returns the register read back. A TF-A built without
 * AML_USB_FALLBACK forwards the call to an SCP over SCPI, which the open SC does
 * not answer, and returns 0: read back as not armed, so `unsupported`.
 *
 * G12B (A311D) is refused: its ROM is not disassembled and the word is
 * unmeasured there. The first stage of ours a host loads over USB clears the
 * word (spl.c, board_init_f), so a request is spent by one boot.
 *
 * Design: claude design/rules/bootloader/reset-to-usb-boot.md R1, R2.
 */

#include <bootrom.h>
#include <dm/device.h>
#include <event.h>
#include <stdio.h>
#include <asm/io.h>
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/types.h>

/* AO_RTI_STATUS_REG2, BSP v2019/arch/arm/mach-meson/g12a/firmware/scp_task/secure_apb.h:2798 */
#define G12_AO_RTI_STATUS_REG2		0xFF800008UL
/* The BootROM's "USB first, wait for the host forever" value. */
#define G12_OVD_USB_FOREVER		0x4f5244b1u

/* SET_USB_BOOT_FUNC, CLEAR / FORCE: BSP v2019/arch/arm/include/asm/arch-g12a/bl31_apis.h:57-60 */
#define G12_SIP_SET_USB_BOOT_FUNC	0x82000043
#define G12_USB_BOOT_CLEAR		1
#define G12_USB_BOOT_FORCE		2

/* socinfo major (board-info.c): G12A 0x28, G12B 0x29, SM1 0x2b */
#define G12_SOC_G12A			0x28
#define G12_SOC_SM1			0x2b

static unsigned long g12_usb_boot(unsigned long sub)
{
	struct arm_smccc_res res;

	arm_smccc_smc(G12_SIP_SET_USB_BOOT_FUNC, sub, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

/*
 * The SoC from the control DT's root compatible.  Not socinfo: in U-Boot
 * proper meson_get_socinfo() has no ao-secure syscon to read ("failed to get
 * regmap"), and AO_SEC_SD_CFG8 (0xff800220) reads 0 from the normal world
 * under our TF-A (both measured on the S905D3-CC).  G12B boards' DTs carry
 * "amlogic,g12b", so they are refused.
 */
/*
 * Does TF-A write the word itself (AML_USB_FALLBACK)?  Asked without arming
 * anything: an unknown subcommand (3) is answered ~0 by that TF-A, while one
 * without it forwards the call over SCPI, which the open SC does not answer,
 * and returns 0 after the SCPI timeout.  Asked once at boot (the timeout must
 * not land inside a host's DETACH), so the USB request's DETACH is stalled
 * where `bootrom` would answer unsupported.  The check is silent: it also
 * runs from the DFU setup handler, where a console write would re-enter the
 * gadget.
 */
#define G12_USB_BOOT_PROBE		3

static int g12_fallback_known;	/* 0 unknown, 1 yes, -1 no */

static int g12_bootrom_probe(void)
{
	if (!g12_fallback_known)
		g12_fallback_known =
			g12_usb_boot(G12_USB_BOOT_PROBE) == ~0UL ? 1 : -1;
	return 0;
}
EVENT_SPY_SIMPLE(EVT_LAST_STAGE_INIT, g12_bootrom_probe);

int bootrom_arch_check(void)
{
	if (!of_machine_is_compatible("amlogic,sm1") &&
	    !of_machine_is_compatible("amlogic,g12a"))
		return -ENOSYS;
	g12_bootrom_probe();
	return g12_fallback_known > 0 ? 0 : -ENOSYS;
}

int bootrom_arch_arm(void)
{
	unsigned long back = g12_usb_boot(G12_USB_BOOT_FORCE);
	u32 reg = readl((void __iomem *)G12_AO_RTI_STATUS_REG2);

	/* Both: the SMC's own read-back, and the register as we read it. */
	if (back != G12_OVD_USB_FOREVER || reg != G12_OVD_USB_FOREVER) {
		printf("bootrom: FORCE read back %08lx, AO_RTI_STATUS_REG2 %08x\n",
		       back, reg);
		return -EIO;
	}
	return 0;
}

void bootrom_arch_disarm(void)
{
	g12_usb_boot(G12_USB_BOOT_CLEAR);
}
