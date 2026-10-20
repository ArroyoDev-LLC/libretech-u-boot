/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * The boot source: the medium the BootROM loaded the first stage from.
 *
 * One vocabulary for every SoC family (bootloader policy §4 "Boot source
 * variables"): the SPL banner, the U-Boot proper banner, the `bootsource`
 * environment variable and the /chosen/libre-computer,boot-source property
 * in both the control FDT and the kernel FDT are all spelled from the one
 * table in boot/bootsource.c. A vendor mapping function turns the BootROM's
 * own identifier (Rockchip BROM_BOOTSOURCE_ID, Amlogic AO_SEC_GP_CFG0,
 * Allwinner eGON boot_media, Zynq SLCR BOOT_MODE) into this enum; nothing
 * else names a medium.
 *
 * The token names the MEDIUM, not the controller: "sd" and "emmc" rather
 * than "mmc0"/"mmc1", so a script can test `${bootsource}` without knowing
 * the board's MMC numbering. `bootdevice` carries that number.
 */

#ifndef __BOOTSOURCE_H
#define __BOOTSOURCE_H

enum bootsource {
	BOOTSOURCE_UNKNOWN = 0,	/* the chain could not tell (or did not say) */
	BOOTSOURCE_SD,		/* removable SD/microSD card */
	BOOTSOURCE_EMMC,	/* eMMC (user area or a hardware boot partition) */
	BOOTSOURCE_UFS,		/* UFS boot LUN */
	BOOTSOURCE_SPI,		/* SPI NOR */
	BOOTSOURCE_NAND,	/* raw or SPI NAND */
	BOOTSOURCE_USB,		/* the BootROM's USB download mode (MaskROM,
				 * Amlogic USB BootROM, FEL, BROM) */
	BOOTSOURCE_NET,		/* the BootROM fetched the stage over the network */
	BOOTSOURCE_RAM,		/* the stage was already in memory (JTAG, a
				 * debugger, a host that staged DRAM) */
	BOOTSOURCE_COUNT,
};

/* Environment variable and /chosen property names. One home each. */
#define BOOTSOURCE_ENV		"bootsource"
#define BOOTSOURCE_DEV_ENV	"bootdevice"
#define BOOTSOURCE_PROP		"libre-computer,boot-source"

/**
 * bootsource_token() - lower-case token for the env variable / DT property
 *
 * Return: "sd", "emmc", "ufs", "spi", "nand", "usb", "net", "ram" or
 *	   "unknown"; never NULL
 */
const char *bootsource_token(enum bootsource src);

/**
 * bootsource_name() - human form for a banner line
 *
 * Return: "SD", "eMMC", "UFS", "SPI NOR", "NAND", "USB", "network", "RAM"
 *	   or "unknown"; never NULL
 */
const char *bootsource_name(enum bootsource src);

/**
 * bootsource_from_token() - parse a token back (from a /chosen property)
 *
 * Return: the enum value, BOOTSOURCE_UNKNOWN for NULL or an unknown token
 */
enum bootsource bootsource_from_token(const char *token);

/**
 * bootsource_announce() - print "Boot source: <name>", once per phase
 *
 * Called by each SoC family's SPL as soon as it has a console, and by
 * bootsource_publish() in U-Boot proper.
 */
void bootsource_announce(enum bootsource src);

/**
 * bootsource_publish() - U-Boot proper: banner + env + remember for the FDT
 * @src: the boot source
 * @mmc_dev: the MMC device index the environment and boot.ini are read
 *	     from on this boot, or -1 to leave `bootdevice` alone
 *
 * Sets `bootsource` to the token, `bootdevice` to @mmc_dev when given,
 * prints the banner, and records @src for /chosen/libre-computer,boot-source
 * in the kernel FDT (the EVT_FT_FIXUP spy, or bootsource_fdt_fixup()).
 */
void bootsource_publish(enum bootsource src, int mmc_dev);

/**
 * bootsource_get() - the boot source bootsource_publish() recorded
 *
 * Return: the recorded source, BOOTSOURCE_UNKNOWN before publish
 */
enum bootsource bootsource_get(void);

#if IS_ENABLED(CONFIG_BOOTSOURCE) && !defined(CONFIG_XPL_BUILD)
/**
 * bootsource_fdt_fixup() - write /chosen/libre-computer,boot-source into @blob
 * @blob: the FDT the kernel boots
 *
 * For a board on a live tree, where EVT_FT_FIXUP is not sent: called from
 * its ft_board_setup().
 *
 * Return: 0, or a libfdt error
 */
int bootsource_fdt_fixup(void *blob);
#else
static inline int bootsource_fdt_fixup(void *blob)
{
	return 0;
}
#endif

#endif /* __BOOTSOURCE_H */
