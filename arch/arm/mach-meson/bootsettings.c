// SPDX-License-Identifier: GPL-2.0+
/*
 * The bootsettings command's Amlogic decode: GXL (S905X, S805X) and SM1 (S905D3).
 *
 * Moved from the Libre Computer app's core/bootsettings.go, which decoded the same registers
 * from `md.l` over the console; the addresses and bit meanings are the ones it uses (claude
 * tools/libre-computer-app/boot-settings-registers.md). Other Meson SoCs (G12A, G12B, AXG, A1)
 * print unknown: their strap and fuse bits are not established. Reads only.
 */

#include <bootsettings.h>
#include <dm/device.h>
#include <vsprintf.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/string.h>

/* GXL */
#define GXL_AO_SEC_GP_CFG0	0xc8100240	/* [3:0] boot device, [7:6] recovery dongle */
#define GXL_POR_CONFIG		0xc1107d54	/* P_ASSIST_POR_CONFIG: [7:6] the latched straps */
/*
 * AO_SEC_SD_CFG10: the BootROM's mirror of the boot-relevant OTP bits (the banner's FEAT:
 * word), readable from U-Boot proper: 0xadfc318c on AML-S805X-AC-V2 beside the banner's
 * FEAT:ADFC318C (2026-10-25). Bits (amlogic/software/boot/gxl/efuse-map.md §3, RE-GXL
 * gxl_bl1_regs.h AO_SEC_SD_CFG10_*): 1, 11, 17 the USB password's three slots, 4 verified
 * boot, 25 anti-rollback, 28 encrypted BL2; 29 gates the ROM's cache setup around verify and
 * 31 is set once the AO domain has sampled the OTP -- neither is a policy, and the other bits
 * are not named on GXL, so neither is decoded.
 */
#define GXL_AO_SEC_SD_CFG10	0xc8100228
#define GXL_CFG10_USB_PASSWORD	(BIT(1) | BIT(11) | BIT(17))
#define GXL_CFG10_VERIFY	BIT(4)
#define GXL_CFG10_ANTI_ROLLBACK	BIT(25)
#define GXL_CFG10_ENCRYPT	BIT(28)
/* SM1 */
#define SM1_AO_SEC_GP_CFG0	0xff800240	/* [3:0] boot device, [7:4] POC */
#define SM1_POR_CONFIG		0xffd08154	/* P_ASSIST_POR_CONFIG: [5:2] POC */
#define SM1_AO_SEC_SD_CFG10	0xff800228	/* the eFuse boot-config mirror */

/* A list of words joined by sep, printed as key=... */
struct joined {
	char buf[96];
	int len;
	char sep;
};

static void add(struct joined *j, const char *w)
{
	if (j->len && j->len < (int)sizeof(j->buf) - 1)
		j->buf[j->len++] = j->sep;
	j->len += snprintf(j->buf + j->len, sizeof(j->buf) - j->len, "%s", w);
	if (j->len >= (int)sizeof(j->buf))
		j->len = sizeof(j->buf) - 1;
}

static void gxl(void)
{
	u32 cfg0 = readl(GXL_AO_SEC_GP_CFG0), por = readl(GXL_POR_CONFIG);
	u32 cfg10 = readl(GXL_AO_SEC_SD_CFG10);
	unsigned int poc = (por >> 6) & 3, rcy = (cfg0 >> 6) & 3;
	bool usb_last = poc & 1, spi_skipped = poc & 2;
	struct joined order = { .sep = ',' }, efuse = { .sep = ';' };
	char straps[8];

	/* A recovery dongle sends the BootROM to it before anything else. */
	if (rcy == 1)
		add(&order, "usb");
	else if (rcy == 2)
		add(&order, "sd");
	if (!usb_last)
		add(&order, "usb");
	if (!spi_skipped)
		add(&order, "spi");
	add(&order, "emmc");
	add(&order, "sd");
	if (usb_last)
		add(&order, "usb");
	bootsettings_set("order", order.buf);
	snprintf(straps, sizeof(straps), "0x%x", poc);
	bootsettings_set("straps", straps);
	if (cfg10 & GXL_CFG10_VERIFY)
		add(&efuse, "verified-boot");
	if (cfg10 & GXL_CFG10_ENCRYPT)
		add(&efuse, "encrypted");
	if (!efuse.len)
		add(&efuse, "secure-boot-off");
	if (cfg10 & GXL_CFG10_ANTI_ROLLBACK)
		add(&efuse, "anti-rollback");
	if (cfg10 & GXL_CFG10_USB_PASSWORD)
		add(&efuse, "usb-password");
	bootsettings_set("efuse", efuse.buf);
	bootsettings_set_raw("AO_SEC_GP_CFG0", cfg0);
	bootsettings_set_raw("P_ASSIST_POR_CONFIG", por);
	bootsettings_set_raw("AO_SEC_SD_CFG10", cfg10);
}

static void sm1(void)
{
	u32 cfg0 = readl(SM1_AO_SEC_GP_CFG0), por = readl(SM1_POR_CONFIG);
	u32 cfg10 = readl(SM1_AO_SEC_SD_CFG10);
	unsigned int poc = (por >> 2) & 0xf;
	bool spi = !(poc & 1) || !(poc & 4);
	static const struct {
		const char *token;
		int enable;	/* the device's boot-enable bit in CFG10 */
	} devs[] = { { "spi", 20 }, { "emmc", 23 }, { "sd", 22 }, { "usb", 19 } };
	static const struct {
		const char *word;
		int bit;
	} enables[] = { { "usb-boot-off", 19 }, { "spi-boot-off", 20 }, { "recovery-boot-off", 21 },
			{ "sd-boot-off", 22 }, { "emmc-boot-off", 23 } };
	struct joined order = { .sep = ',' }, efuse = { .sep = ';' };
	char straps[8];
	int i;

	for (i = 0; i < ARRAY_SIZE(devs); i++) {
		if (!strcmp(devs[i].token, "spi") && !spi)
			continue;
		if (cfg10 & BIT(devs[i].enable))
			add(&order, devs[i].token);
	}
	bootsettings_set("order", order.len ? order.buf : "unknown");
	snprintf(straps, sizeof(straps), "0x%x", poc);
	bootsettings_set("straps", straps);
	if (cfg10 & BIT(4))
		add(&efuse, "verified-boot");
	if (cfg10 & BIT(28))
		add(&efuse, "encrypted");
	if (!efuse.len)
		add(&efuse, "secure-boot-off");
	add(&efuse, cfg10 & BIT(12) ? "jtag-on" : "jtag-off");
	for (i = 0; i < ARRAY_SIZE(enables); i++)
		if (!(cfg10 & BIT(enables[i].bit)))
			add(&efuse, enables[i].word);
	bootsettings_set("efuse", efuse.buf);
	bootsettings_set_raw("AO_SEC_GP_CFG0", cfg0);
	bootsettings_set_raw("P_ASSIST_POR_CONFIG", por);
	bootsettings_set_raw("AO_SEC_SD_CFG10", cfg10);
}

/*
 * The SoC, from the board's device tree (its root compatible names it): the SoC id word
 * AO_SEC_SD_CFG8 is not readable here -- meson_get_socinfo() needs the ao-secure syscon's
 * regmap, which U-Boot proper on the AML-S905X-CC-V2 does not have ("failed to get regmap"),
 * and a direct read of 0xc8100220 answered 0x00000002 (s7-nyc, 2026-10-24).
 */
void bootsettings_soc(void)
{
	if (IS_ENABLED(CONFIG_MESON_GXL) && of_machine_is_compatible("amlogic,meson-gxl"))
		gxl();
	else if (IS_ENABLED(CONFIG_MESON_G12A) && of_machine_is_compatible("amlogic,sm1"))
		sm1();
	else {
		bootsettings_set("order", "unknown");
		bootsettings_set("efuse", "unknown");
	}
}
