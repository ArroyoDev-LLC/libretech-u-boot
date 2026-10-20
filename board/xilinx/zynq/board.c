// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2012 Michal Simek <monstr@monstr.eu>
 * (C) Copyright 2013 - 2018 Xilinx, Inc.
 */

#include <config.h>
#include <bootsource.h>
#include <debug_uart.h>
#include <dfu.h>
#include <efi_loader.h>
#include <init.h>
#include <log.h>
#include <dm/uclass.h>
#include <env.h>
#include <env_internal.h>
#include <fdtdec.h>
#include <fpga.h>
#include <malloc.h>
#include <memalign.h>
#include <mmc.h>
#include <watchdog.h>
#include <wdt.h>
#include <zynqpl.h>
#include <asm/global_data.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>
#include "../common/board.h"

DECLARE_GLOBAL_DATA_PTR;

#if !defined(CONFIG_XPL_BUILD) && defined(CONFIG_DEBUG_UART_BOARD_INIT)
void board_debug_uart_init(void)
{
	/* Add initialization sequence if UART is not configured */
}
#endif

int board_init(void)
{
	if (IS_ENABLED(CONFIG_XPL_BUILD))
		printf("Silicon version:\t%d\n", zynq_get_silicon_version());

	if (CONFIG_IS_ENABLED(DM_I2C) && CONFIG_IS_ENABLED(I2C_EEPROM))
		xilinx_read_eeprom();

	return 0;
}

/**
 * boot_targets_has() - whole-token search of a boot_targets list
 * @list: space-separated boot_targets value (may be NULL)
 * @tok:  token to look for (may be empty)
 *
 * Whole-token, so "mmc0" does not match "mmc1" and "qspi" does not match a
 * hypothetical "qspidev". Returns 0 for a NULL/empty list or an empty token.
 */
static int boot_targets_has(const char *list, const char *tok)
{
	size_t toklen = tok ? strlen(tok) : 0;
	const char *p = list;

	if (!list || !toklen)
		return 0;

	while (*p) {
		size_t len;

		while (*p == ' ')
			p++;
		len = strcspn(p, " ");
		if (!len)
			break;
		if (len == toklen && !memcmp(p, tok, toklen))
			return 1;
		p += len;
	}

	return 0;
}

int board_late_init(void)
{
	int env_targets_len = 0;
	const char *mode;
	char *new_targets;
	char *env_targets;

	if (IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT))
		configure_capsule_updates();

	/*
	 * The platform boot-source variables (include/bootsource.h), on every
	 * boot -- before the saved-environment early return below, because a
	 * runtime fact must not be frozen into whatever a saveenv captured.
	 * `bootdevice` is the SD (mmc0 on every Zynq board here: no eMMC) when
	 * the strap says SD; on a QSPI/NAND/JTAG boot the environment and
	 * boot.ini still come from that same first MMC, so the index holds.
	 */
	if (CONFIG_IS_ENABLED(BOOTSOURCE))
		bootsource_publish(zynq_bootsource(), 0);

	if (!(gd->flags & GD_FLG_ENV_DEFAULT)) {
		debug("Saved variables - Skipping\n");
		return 0;
	}

	if (!IS_ENABLED(CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG))
		return 0;

	switch ((zynq_slcr_get_boot_mode()) & ZYNQ_BM_MASK) {
	case ZYNQ_BM_QSPI:
		mode = "qspi";
		env_set("modeboot", "qspiboot");
		break;
	case ZYNQ_BM_NAND:
		mode = "nand";
		env_set("modeboot", "nandboot");
		break;
	case ZYNQ_BM_NOR:
		mode = "nor";
		env_set("modeboot", "norboot");
		break;
	case ZYNQ_BM_SD:
		mode = "mmc0";
		env_set("modeboot", "sdboot");
		break;
	case ZYNQ_BM_JTAG:
		/*
		 * Board-bootloader policy §2 "Default boot order": network/PXE
		 * is NOT a default boot target -- it is opted into by
		 * overriding boot_targets. This prepend is the runtime half of
		 * the same list e7e669bbcdc fixed at compile time in
		 * include/configs/zynq-common.h: it puts the DETECTED boot mode
		 * in front of the compiled-in boot_targets, and the JTAG arm
		 * used to smuggle "pxe dhcp" back in ahead of every storage
		 * target -- so a board strapped to JTAG boot mode would BOOTP
		 * and run whatever a LAN PXE server happened to offer, which is
		 * precisely the defect that commit removed.
		 *
		 * "jtag" alone is kept: it maps to bootcmd_jtag, which sources a
		 * script a debugger has already placed in DRAM at ${scriptaddr}
		 * -- a local, deliberate, physically-present-operator action,
		 * not an unattended network fetch. pxe/dhcp remain compiled in
		 * as commands, so `setenv boot_targets "... pxe dhcp"` (or a NOR
		 * config-sector override, §4) still works where it is wanted.
		 *
		 * Applies to EVERY Zynq board in this tree, zynq_e310 included.
		 */
		mode = "jtag";
		env_set("modeboot", "jtagboot");
		break;
	default:
		mode = "";
		env_set("modeboot", "");
		break;
	}

	/*
	 * One terminating char + one byte for space between mode
	 * and default boot_targets
	 */
	env_targets = env_get("boot_targets");
	if (env_targets)
		env_targets_len = strlen(env_targets);

	new_targets = calloc(1, strlen(mode) + env_targets_len + 2);
	if (!new_targets)
		return -ENOMEM;

	/*
	 * Prepend the detected boot mode, then copy the compiled-in list MINUS
	 * any entry equal to that mode.
	 *
	 * Without the filter, a board strapped to a mode that ALSO appears in
	 * BOOT_TARGET_DEVICES gets it listed twice -- and qspi, mmc0 and nor all
	 * appear there. A QSPI-strapped board came out as
	 *   "qspi mmc0 mmc1 qspi usb0 usb1 jtag"
	 * so bootstd probes qspi, fails, walks the whole list, and probes it a
	 * second time before reaching a medium that works. Observed baked into
	 * sdr-pro-alpha's saved environment, where it persists because this
	 * function only runs on a DEFAULT env (GD_FLG_ENV_DEFAULT above) -- so
	 * the duplicate is written once at saveenv and never re-derived.
	 *
	 * Only a whole-token match is removed: "mmc0" must not match "mmc1".
	 *
	 * The prepend REORDERS the compiled-in list; it must never INVENT a
	 * target. config_distro_bootcmd.h emits a bootcmd_<dev> only for the
	 * devices listed in BOOT_TARGET_DEVICES, so prepending a mode that is
	 * not in that list produces a boot_targets entry with no command behind
	 * it -- distro_bootcmd's `run bootcmd_qspi` then fails with
	 * '"bootcmd_qspi" not defined' on every boot, and bootstd walks a label
	 * it cannot resolve. That is live, not hypothetical: the Libre Computer
	 * NOR boards (CONFIG_BOARD_CONFIG, see include/configs/zynq-common.h)
	 * deliberately drop the qspi target because policy §4 puts no boot script
	 * in SPI NOR, and both of them are strapped to QSPI boot mode -- so this
	 * arm would otherwise hand them a dangling "qspi" every boot.
	 *
	 * So: prepend only a mode the compiled-in list actually contains. When it
	 * does not, the list is left in its compiled order (modeboot is still set
	 * either way -- that variable is independent of bootstd).
	 */
	if (!boot_targets_has(env_targets, mode))
		mode = "";

	if (*mode)
		strcpy(new_targets, mode);

	if (env_targets) {
		char *list = strdup(env_targets);
		char *rest, *tok;

		if (!list) {
			free(new_targets);
			return -ENOMEM;
		}

		rest = list;
		while ((tok = strsep(&rest, " ")) != NULL) {
			if (!*tok || !strcmp(tok, mode))
				continue;
			if (*new_targets)
				strcat(new_targets, " ");
			strcat(new_targets, tok);
		}

		free(list);
	}

	env_set("boot_targets", new_targets);
	free(new_targets);

	return board_late_init_xilinx();
}

#if !defined(CFG_SYS_SDRAM_BASE) && !defined(CFG_SYS_SDRAM_SIZE)
int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

int dram_init(void)
{
	if (fdtdec_setup_mem_size_base() != 0)
		return -EINVAL;

	zynq_ddrc_init();

	return 0;
}
#else
int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)CFG_SYS_SDRAM_BASE,
				    CFG_SYS_SDRAM_SIZE);

	zynq_ddrc_init();

	return 0;
}
#endif

enum env_location env_get_location(enum env_operation op, int prio)
{
	u32 bootmode = zynq_slcr_get_boot_mode() & ZYNQ_BM_MASK;

	if (prio)
		return ENVL_UNKNOWN;

	switch (bootmode) {
	case ZYNQ_BM_SD:
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_EXT4))
			return ENVL_EXT4;
		return ENVL_NOWHERE;
	case ZYNQ_BM_NAND:
		if (IS_ENABLED(CONFIG_ENV_IS_IN_NAND))
			return ENVL_NAND;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_UBI))
			return ENVL_UBI;
		return ENVL_NOWHERE;
	case ZYNQ_BM_NOR:
	case ZYNQ_BM_QSPI:
		/*
		 * SPI holds firmware only on Libre Computer-style boards
		 * (env proper is on the OS EFI/FAT media as uboot.env). Prefer
		 * SPI_FLASH only when that backend is explicitly enabled;
		 * otherwise fall through to FAT/EXT4 like the SD boot path.
		 */
		if (IS_ENABLED(CONFIG_ENV_IS_IN_SPI_FLASH))
			return ENVL_SPI_FLASH;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_EXT4))
			return ENVL_EXT4;
		return ENVL_NOWHERE;
	case ZYNQ_BM_JTAG:
	default:
		/* JTAG/unknown: still prefer FAT when configured (media present). */
		if (IS_ENABLED(CONFIG_ENV_IS_IN_FAT))
			return ENVL_FAT;
		if (IS_ENABLED(CONFIG_ENV_IS_IN_EXT4))
			return ENVL_EXT4;
		return ENVL_NOWHERE;
	}
}

#define DFU_ALT_BUF_LEN                SZ_1K

void configure_capsule_updates(void)
{
	ALLOC_CACHE_ALIGN_BUFFER(char, buf, DFU_ALT_BUF_LEN);

	/*
	 * zynq_adrv9009 supplies its own static update_info (a single binman
	 * blob, board/xilinx/zynq/zynq-adrv9009/board.c). The generic QSPI map
	 * below is wrong for it in a destructive way: "boot.bin raw 0 0x1500000"
	 * plus "u-boot.img raw <CONFIG_SYS_SPI_U_BOOT_OFFS> 0x500000" both run
	 * straight through the
	 * 17 416 352 B stock PL bitstream that occupies 0x300000-0x139C0A0 on
	 * that unit, and the second span is unreachable anyway -- U-Boot proper
	 * there has no 4-byte addressing, so anything at or above 16 MiB
	 * truncates. Leave its update_info alone.
	 */
	if (IS_ENABLED(CONFIG_ZYNQ_ADRV9009))
		return;

	memset(buf, 0, DFU_ALT_BUF_LEN);

	switch ((zynq_slcr_get_boot_mode()) & ZYNQ_BM_MASK) {
#if defined(CONFIG_SPL_FS_LOAD_PAYLOAD_NAME)
	case ZYNQ_BM_SD:
		snprintf(buf, DFU_ALT_BUF_LEN,
			 "mmc 0=boot.bin fat 0 1;"
			 "%s fat 0 1", CONFIG_SPL_FS_LOAD_PAYLOAD_NAME);
		break;
#if defined(CONFIG_SPL_SPI_LOAD)
	case ZYNQ_BM_QSPI:
		snprintf(buf, DFU_ALT_BUF_LEN,
			 "sf 0:0=boot.bin raw 0 0x1500000;"
			 "%s raw 0x%x 0x500000",
			 CONFIG_SPL_FS_LOAD_PAYLOAD_NAME,
			 CONFIG_SYS_SPI_U_BOOT_OFFS);
		break;
#endif
#endif
	default:
		return;
	}

	update_info.dfu_string = strdup(buf);
	debug("Capsule DFU: %s\n", update_info.dfu_string);
}
