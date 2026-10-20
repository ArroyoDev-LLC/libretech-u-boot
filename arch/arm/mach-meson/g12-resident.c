// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic G12A/SM1 resident chain (CONFIG_MESON_G12A_RESIDENT).
 *
 * A TPL the BootROM loads from eMMC runs on every power-up, so a hang anywhere
 * from that TPL to U-Boot proper would recur on every power-up too.  The
 * recovery is the BootROM's own override word:
 *
 *   AO_RTI_STATUS_REG2 (0xFF800008) = 0x4f5244b1
 *
 * which the SM1 BootROM (bl1.bin sha256 94322c13...) reads before it scans any
 * medium: [31:8] = 0x4f5244 with [7:4] = 0xb makes it print OVD:1 and enter
 * USB before SPI and eMMC (0xffff08d8-0xffff092c, 0xffff0aac-0xffff0aec), and
 * the exact value 0x4f5244b1 makes that USB wait for the host with no timeout
 * (0xffff68c4).  The vendor writes it from its SCP for the USB burn tool (BL30
 * literals 0x4f5244b0/b1/c0 beside 0xff800008; SCPI 0xB0/0xB1/0xB2/0xB3 in BSP
 * v2019/drivers/usb/gadget/v2_burning/v2_common/optimus_progress.c:142-145).  It is
 * in the always-on domain: a watchdog reset keeps it, a power cycle clears it
 * (both measured: amlogic/software/boot/g12/audit/2026-10-07-sm1-usb-override-proof.md).
 *
 * TPL: on any boot device but USB, arm the word and the EE watchdog, as the
 * first act of board_init_f().  From then on, every TPL site that used to STOP
 * the EE watchdog (the USB window's hold-off and stop, the DRAM PLL init's
 * sticky-bit clear, the training pets) PETS it instead -- g12a_resident_wdt_keep()
 * -- so the one window runs on into the DRAM stage, BL31 and U-Boot proper.  A
 * hang anywhere up to U-Boot proper then ends in a watchdog reset, and the
 * BootROM waits on USB for the rig instead of loading the same TPL again.
 *
 * DRAM stage: the TPL delivered the boot FIT to DRAM; take it from there.
 *
 * ("Standalone" mode, MESON_G12A_RESIDENT_OVERRIDE=n: the TPL arms the EE
 * watchdog without the word, so a hang resets into the normal boot.)
 *
 * U-Boot proper: while the word is armed or the TPL armed the watchdog,
 * U-Boot owns the EE watchdog (driver
 * model, petted every second by its cyclic), so a hang in U-Boot proper is
 * also returned to the rig; the word is cleared at the last step before the
 * OS (board_quiesce_devices(), after wdt_handoff_to_os() has stopped the
 * watchdog), on both the booti/bootm and the UEFI paths.  A U-Boot `reset` or
 * panic before that goes to the rig too; `usbfb clear` clears the word by hand.
 * A non-secure write to AO_RTI_STATUS_REG2 raises an SError on this SoC
 * (measured: "Error" handler, esr 0xbe000011, on `mw.l 0xff800008` from U-Boot
 * proper), so the clear goes through the secure monitor: SiP SET_USB_BOOT_FUNC
 * (0x82000043, BSP v2019/arch/arm/include/asm/arch-g12a/bl31_apis.h:57) with
 * CLEAR_USB_BOOT (1, the same file :59), answered by a TF-A built with
 * AML_USB_FALLBACK=1.
 *
 * Plan: amlogic/software/boot/g12/plan/sm1-resident-chain.md "USB fallback"
 * and "Recovery gaps".
 */

#include <asm/io.h>
#include <asm/arch/boot.h>
#include <asm/arch/g12a-handoff.h>
#include <linux/bitops.h>
#include <linux/types.h>

/* AO_RTI_STATUS_REG2, BSP v2019/arch/arm/mach-meson/g12a/firmware/scp_task/secure_apb.h:2798 */
#define G12R_AO_RTI_STATUS_REG2		0xFF800008UL
/* The BootROM's "USB first, wait for the host forever" value (ROM 0xffff68c8/cc). */
#define G12R_OVD_USB_FOREVER		0x4f5244b1u

/*
 * EE watchdog, the block drivers/watchdog/meson_gxbb_wdt.c drives at
 * watchdog@f0d0 on cbus: P_WATCHDOG_CNTL / _TCNT / _RST, BSP
 * v2019/arch/arm/include/asm/arch-g12a/regs.h:13305, :13309, :13311.
 * TCNT [15:0] is the timeout and [31:16] the running count, both in ticks
 * (linux-6.18.y-lc drivers/watchdog/meson_gxbb_wdt.c:28-29, :105).
 */
#define G12R_WDT_CNTL			0xFFD0F0D0UL
#define G12R_WDT_TCNT			0xFFD0F0D8UL
#define G12R_WDT_RST			0xFFD0F0DCUL
#define   G12R_WDT_CLKDIV_EN		BIT(25)
#define   G12R_WDT_CLK_EN		BIT(24)
#define   G12R_WDT_EE_RESET		BIT(21)
#define   G12R_WDT_EN			BIT(18)
/* [17:0] divider of the 24 MHz xtal: 24000 = a 1 ms tick, so TCNT is in ms. */
#define   G12R_WDT_DIV_1MS		24000u

#if defined(CONFIG_TPL_BUILD)

/*
 * Set when this TPL armed the fallback.  In .data, not .bss: the G12 TPL never
 * returns from board_init_f(), so crt0 never clears its BSS, while .data is
 * part of the image the BootROM loads on every boot.
 */
static u32 g12r_wdt_owned __section(".data");

int g12a_resident_from_emmc(void)
{
	return meson_get_boot_device() == BOOT_DEVICE_EMMC;
}

/*
 * Arm on every boot device except USB.  The ROM leaves 1 in AO_SEC_GP_CFG0
 * [3:0] when it boots from eMMC (bl1.bin 0xffff0d10-0xffff0d14, and its
 * success path 0xffff0de0-0xffff0df0 does not touch the nibble again), but a
 * TPL that misreads its source must still be recoverable: armed, it falls
 * through the USB window's timeouts to NOSPL and the watchdog fires.  A
 * USB-delivered TPL is already where the fallback leads.
 */
static bool g12r_should_arm(void)
{
	/* MESON_G12A_RESIDENT_ARM_ON_USB: TEST ONLY, gate a USB-delivered image. */
	return IS_ENABLED(CONFIG_MESON_G12A_RESIDENT_ARM_ON_USB) ||
	       meson_get_boot_device() != BOOT_DEVICE_USB;
}

/**
 * g12a_resident_wdt_keep() - pet instead of stop, while this TPL owns the
 * EE watchdog.
 *
 * Called first by every TPL site that would otherwise stop or re-time it:
 * meson-usb-g12.c g12_wdt_hold_off() / g12_wdt_stop(), dram-g12a.c
 * g12a_ddr_pll_init() / g12a_wdt_pet_safe().
 *
 * Return: true when the watchdog was petted and the caller must leave it alone.
 */
bool g12a_resident_wdt_keep(void)
{
	if (!g12r_wdt_owned)
		return false;
	writel(0, (void __iomem *)G12R_WDT_RST);
	return true;
}

/**
 * g12a_resident_arm() - arm the EE watchdog (and, "rig" mode, the word).
 *
 * Only register writes: called before meson_timer_rate_init(), as the first
 * act of board_init_f().
 *
 * Return: the read-back of the EE watchdog CNTL when armed (EN set, never 0),
 * 0 when it did not arm.  The USB-override word is written only in the "rig"
 * mode (MESON_G12A_RESIDENT_OVERRIDE=y); "standalone" arms the watchdog alone.
 */
u32 g12a_resident_arm(void)
{
	if (!g12r_should_arm())
		return 0;

	/* meson_gxbb_wdt.c's order: divider + reset target, count, ping, enable. */
	writel(G12R_WDT_DIV_1MS | G12R_WDT_EE_RESET | G12R_WDT_CLK_EN |
	       G12R_WDT_CLKDIV_EN, (void __iomem *)G12R_WDT_CNTL);
	writel(CONFIG_MESON_G12A_RESIDENT_WDT_MS, (void __iomem *)G12R_WDT_TCNT);
	writel(0, (void __iomem *)G12R_WDT_RST);
	writel(readl((void __iomem *)G12R_WDT_CNTL) | G12R_WDT_EN,
	       (void __iomem *)G12R_WDT_CNTL);
	g12r_wdt_owned = 1;

	if (IS_ENABLED(CONFIG_MESON_G12A_RESIDENT_OVERRIDE))
		writel(G12R_OVD_USB_FOREVER, (void __iomem *)G12R_AO_RTI_STATUS_REG2);
	return readl((void __iomem *)G12R_WDT_CNTL);
}

/* g12a_resident_word() - read back the USB-override word, for the TPL print. */
u32 g12a_resident_word(void)
{
	return readl((void __iomem *)G12R_AO_RTI_STATUS_REG2);
}

#if defined(CONFIG_MESON_G12A_RESIDENT_M20_PROBE) || \
	defined(CONFIG_MESON_G12A_RESIDENT_M21_AO_SPIN)
/*
 * SC->AP heartbeat block and the SC's AO-watchdog pet publish, scp-firmware
 * master-lc product/amlogic/g12a/include/g12a_mmap.h:232 (0xFFFE7400; hb[0]
 * magic, hb[3] poll tick) and src/g12a_core.c WDT6_PUB_*_ADDR (0xFFFE7500
 * magic, +4 pet count, +8 AO CNTL as the SC read it).  AO watchdog CNTL,
 * scp-firmware g12a_regs.h:462 (G12A_AO_WATCHDOG_CNTL, bit 18 enable).
 */
#define G12R_SC_HB0		0xFFFE7400UL
#define G12R_SC_HB3		0xFFFE740CUL
#define G12R_SC_WDT6_MAGIC	0xFFFE7500UL
#define G12R_SC_WDT6_COUNT	0xFFFE7504UL
#define G12R_SC_WDT6_CNTL	0xFFFE7508UL
#define G12R_AO_WDT_CNTL	0xFF800120UL

#if defined(CONFIG_MESON_G12A_RESIDENT_M21_AO_SPIN)
/*
 * g12a_resident_m21_ee_off() - TEST ONLY: clear the EE watchdog's enable,
 * leaving only the AO watchdog able to reset the chip; v[0] AO CNTL, v[1] EE
 * CNTL before, v[2] EE CNTL after.
 */
void g12a_resident_m21_ee_off(u32 *v)
{
	v[0] = readl((void __iomem *)G12R_AO_WDT_CNTL);
	v[1] = readl((void __iomem *)G12R_WDT_CNTL);
	writel(v[1] & ~G12R_WDT_EN, (void __iomem *)G12R_WDT_CNTL);
	v[2] = readl((void __iomem *)G12R_WDT_CNTL);
}
#endif

#if defined(CONFIG_MESON_G12A_RESIDENT_M20_PROBE)
/* g12a_resident_m20_ee() - the EE watchdog as this TPL first finds it. */
void g12a_resident_m20_ee(u32 *v)
{
	v[0] = readl((void __iomem *)G12R_WDT_CNTL);
	v[1] = readl((void __iomem *)G12R_WDT_TCNT);
}

/* g12a_resident_m20_sample() - the SC's heartbeat + pet publish, AO CNTL. */
void g12a_resident_m20_sample(u32 *v)
{
	v[0] = readl((void __iomem *)G12R_SC_HB0);
	v[1] = readl((void __iomem *)G12R_SC_HB3);
	v[2] = readl((void __iomem *)G12R_SC_WDT6_MAGIC);
	v[3] = readl((void __iomem *)G12R_SC_WDT6_COUNT);
	v[4] = readl((void __iomem *)G12R_SC_WDT6_CNTL);
	v[5] = readl((void __iomem *)G12R_AO_WDT_CNTL);
	/*
	 * Must-fire clock for the two samples: the generic counter (low 32 bits
	 * of CNTPCT_EL0, 24 MHz, COUNTER_FREQUENCY) advances by the delay
	 * between them, so a frozen SC tick is not a delay that never happened.
	 */
	{
		u64 cnt;

		asm volatile("isb; mrs %0, cntpct_el0" : "=r" (cnt));
		v[6] = (u32)cnt;
	}
}
#endif /* M20_PROBE */
#endif /* M20_PROBE || M21_AO_SPIN */

/**
 * g12a_resident_rearm() - restore this TPL's watchdog after dram_init().
 *
 * The vendor DDR firmware in the residual (aml_ddr.fw, run from SRAM during
 * training) re-programs the EE watchdog with its own watchdog_init(5000):
 * CNTL = 0x03605dbf | EN, TCNT = 5000 (concat offset 0xa0, called at 0x6c00
 * and 0x96a8), so a healthy boot would leave the TPL with a 5 s window that
 * nothing pets on the eMMC path (measured: recgaps-1, "UF dram c 03645dbf
 * t 00001388", then a watchdog reset in window 2).  Put the armed
 * configuration back; a no-op when this TPL did not arm.
 */
void g12a_resident_rearm(void)
{
	if (!g12r_wdt_owned)
		return;
	writel(G12R_WDT_DIV_1MS | G12R_WDT_EE_RESET | G12R_WDT_CLK_EN |
	       G12R_WDT_CLKDIV_EN, (void __iomem *)G12R_WDT_CNTL);
	writel(CONFIG_MESON_G12A_RESIDENT_WDT_MS, (void __iomem *)G12R_WDT_TCNT);
	writel(0, (void __iomem *)G12R_WDT_RST);
	writel(readl((void __iomem *)G12R_WDT_CNTL) | G12R_WDT_EN,
	       (void __iomem *)G12R_WDT_CNTL);
}

#elif defined(MESON_G12A_DRAM_STAGE)

/**
 * g12a_resident_fit_in_ram() - did the TPL put the boot FIT in DRAM?
 *
 * The handoff block (SRAM 0xFFFB3C00) is the TPL's word for it: a valid block
 * with G12A_HANDOFF_F_FIT_RAM and a delivered length.  The DRAM stage's
 * board_init_f() has already refused a block with a bad magic or version.
 */
bool g12a_resident_fit_in_ram(void)
{
	struct g12a_xpl_handoff *h = (struct g12a_xpl_handoff *)G12A_HANDOFF_ADDR;

	return readl(&h->magic) == G12A_HANDOFF_MAGIC &&
	       (readl(&h->flags) & G12A_HANDOFF_F_FIT_RAM) &&
	       readl(&h->spl_len) != 0;
}

#elif !defined(CONFIG_XPL_BUILD)

#include <command.h>
#include <dm.h>
#include <env.h>
#include <event.h>
#include <stdio.h>
#include <wdt.h>
#include <linux/arm-smccc.h>
#include <linux/string.h>

/* SET_USB_BOOT_FUNC and CLEAR_USB_BOOT, BSP v2019/arch/arm/include/asm/arch-g12a/bl31_apis.h:57, :59 */
#define G12R_SIP_SET_USB_BOOT_FUNC	0x82000043
#define G12R_CLEAR_USB_BOOT		1

static u32 g12r_word(void)
{
	return readl((void __iomem *)G12R_AO_RTI_STATUS_REG2);
}

/*
 * Did the resident TPL arm the EE watchdog on this boot?  EN set AND the
 * timeout equal to the one the TPL programs.  The second term keeps U-Boot
 * from adopting the vendor DDR firmware's own 5 s arm (g12a_resident_rearm())
 * on a boot the TPL did not arm.  The take-over keys on this OR the override
 * word, so the "standalone" mode (no word) is taken over too.  The default
 * timeout (60000) is also what a 60 s systemd RuntimeWatchdogSec programs
 * through meson_gxbb_wdt (whose own default is 30 s), so a watchdog Linux
 * left running is adopted too if the EE watchdog survives a warm reset; the
 * TPL found the BootROM's own 5 s arm there on both resets measured (plan
 * M2.0), and adopting a running watchdog only keeps it petted.
 */
static bool g12r_tpl_armed(void)
{
	return (readl((void __iomem *)G12R_WDT_CNTL) & G12R_WDT_EN) &&
	       (readl((void __iomem *)G12R_WDT_TCNT) & 0xffff) ==
			CONFIG_MESON_G12A_RESIDENT_WDT_MS;
}

/* Set when U-Boot could not own the TPL's watchdog and left it counting. */
static bool g12r_left_counting;

/* Clear the word through BL31 (AML_USB_FALLBACK=1); return the read-back. */
static u32 g12r_clear_word(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(G12R_SIP_SET_USB_BOOT_FUNC, G12R_CLEAR_USB_BOOT,
		      0, 0, 0, 0, 0, 0, &res);
	return g12r_word();
}

/*
 * EVT_DM_POST_INIT_R: the TPL's watchdog has run from the TPL's first act to
 * here.  initr_watchdog() is a few initcalls later, and the meson_gxbb_wdt
 * probe rewrites CNTL without EN; so, while the word is armed or the TPL armed
 * the watchdog, probe the device now, start it at once (U-Boot's cyclic then
 * pets it every second) and force autostart so initr_watchdog() keeps it
 * running.  A set word stays armed until board_quiesce_devices().
 */
static int g12a_resident_take_over(void)
{
	u32 word = g12r_word();
	struct udevice *dev;
	int ret;

	printf("USB fallback: AO_RTI_STATUS_REG2 %08x, watchdog CNTL %08x TCNT %08x\n",
	       word, readl((void __iomem *)G12R_WDT_CNTL),
	       readl((void __iomem *)G12R_WDT_TCNT));
	/*
	 * The word alone still counts: a USB boot the rig serves after a hang
	 * carries the word but no TPL arm (ARM_ON_USB=n), and U-Boot proper
	 * keeps that boot returnable to the rig, as before standalone mode.
	 */
	if (word != G12R_OVD_USB_FOREVER && !g12r_tpl_armed())
		return 0;

	ret = uclass_first_device_err(UCLASS_WDT, &dev);
	if (!ret) {
		wdt_set_force_autostart(dev);
		ret = wdt_start(dev, CONFIG_MESON_G12A_RESIDENT_WDT_MS, 0);
	}
	if (ret) {
		/* Fail toward the rig: keep the TPL's watchdog counting. */
		writel(readl((void __iomem *)G12R_WDT_CNTL) | G12R_WDT_EN,
		       (void __iomem *)G12R_WDT_CNTL);
		g12r_left_counting = true;
		printf("USB fallback: armed; U-Boot could not own the watchdog (%d), it keeps counting\n",
		       ret);
		return 0;
	}
	printf("USB fallback: %s until the OS; watchdog owned by U-Boot, CNTL %08x TCNT %08x\n",
	       word == G12R_OVD_USB_FOREVER ? "armed (rig)" : "watchdog only (standalone)",
	       readl((void __iomem *)G12R_WDT_CNTL),
	       readl((void __iomem *)G12R_WDT_TCNT));
	return 0;
}
EVENT_SPY_SIMPLE(EVT_DM_POST_INIT_R, g12a_resident_take_over);

/*
 * The last step before the OS, on both boot paths: boot/bootm.c bootm_final()
 * and lib/efi_loader/efi_boottime.c efi_exit_boot_services() call it right
 * after wdt_handoff_to_os(), which has already stopped the watchdog unless
 * wdt_handoff=1.  Clear the word so an OS reboot boots storage again, and stop
 * a watchdog U-Boot did not own (the fail-toward-the-rig case above) unless
 * the OS was asked to take it.
 */
void board_quiesce_devices(void)
{
	u32 before = g12r_word();
	bool word = before == G12R_OVD_USB_FOREVER;

	/*
	 * Nothing to do unless the word is armed ("rig") or U-Boot left the
	 * TPL's watchdog counting outside the DM driver (either mode).  A
	 * watchdog the DM driver owns was already handled by
	 * wdt_handoff_to_os().
	 */
	if (!word && !g12r_left_counting)
		return;
	if (env_get_yesno("wdt_handoff") != 1)
		writel(readl((void __iomem *)G12R_WDT_CNTL) & ~G12R_WDT_EN,
		       (void __iomem *)G12R_WDT_CNTL);
	printf("USB fallback: disarmed before the OS, AO_RTI_STATUS_REG2 %08x -> %08x, watchdog CNTL %08x\n",
	       before, word ? g12r_clear_word() : before,
	       readl((void __iomem *)G12R_WDT_CNTL));
}

static int do_usbfb(struct cmd_tbl *cmdtp, int flag, int argc,
		    char *const argv[])
{
	u32 before = g12r_word();

	if (argc == 2 && !strcmp(argv[1], "clear")) {
		printf("USB fallback: cleared by command, AO_RTI_STATUS_REG2 %08x -> %08x\n",
		       before, g12r_clear_word());
		return CMD_RET_SUCCESS;
	}
	if (argc != 1)
		return CMD_RET_USAGE;
	printf("USB fallback: AO_RTI_STATUS_REG2 %08x (%s), watchdog CNTL %08x TCNT %08x\n",
	       before, before == G12R_OVD_USB_FOREVER ? "armed" : "clear",
	       readl((void __iomem *)G12R_WDT_CNTL),
	       readl((void __iomem *)G12R_WDT_TCNT));
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(usbfb, 2, 0, do_usbfb,
	   "resident chain: BootROM USB-fallback word",
	   "       - print the word and the EE watchdog\n"
	   "usbfb clear - clear the word through BL31 (a reset then boots storage)");

#endif
