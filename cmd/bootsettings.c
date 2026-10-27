// SPDX-License-Identifier: GPL-2.0+
/*
 * bootsettings - this board's boot settings, decoded on the board
 *
 * Libre Computer platform policy, "Boot settings: the bootsettings command". Each setting is an
 * environment variable prefixed boot_, set (not saved) and printed as printenv prints it, one
 * per line, in this order: boot_source (the variable the board already sets for its boot medium,
 * reused), boot_order (the BootROM's medium order, boot_source tokens joined by ','),
 * boot_straps (the word the order was decoded from, where the SoC has straps), boot_efuse (the
 * boot-relevant fuse state, ';'-separated words), and boot_raw_<register> for each register
 * read. A setting the SoC cannot read is boot_<name>=unknown. Read-only on the hardware.
 *
 * The decode is the SoC family's own (bootsettings_soc()); a host tool reads these lines and
 * never decodes the registers itself.
 */

#include <command.h>
#include <env.h>
#include <bootsettings.h>
#include <vsprintf.h>

void bootsettings_set(const char *name, const char *value)
{
	char var[48], val[128];

	/*
	 * A copy first: value may be env_get()'s pointer into the very entry env_set() replaces
	 * (boot_source is reused), and the old string is freed under it -- found on
	 * AML-S805X-AC-V2, where boot_source printed garbage.
	 */
	snprintf(val, sizeof(val), "%s", value);
	snprintf(var, sizeof(var), "boot_%s", name);
	env_set(var, val);
	printf("%s=%s\n", var, val);
}

void bootsettings_set_raw(const char *reg, u32 value)
{
	char name[40], hex[12];

	snprintf(name, sizeof(name), "raw_%s", reg);
	snprintf(hex, sizeof(hex), "0x%08x", value);
	bootsettings_set(name, hex);
}

/* A family that has no decode yet says so for every setting it cannot read. */
__weak void bootsettings_soc(void)
{
	bootsettings_set("order", "unknown");
	bootsettings_set("efuse", "unknown");
}

static int do_bootsettings(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	/*
	 * boot_source is the board's own: set at board_late_init (Amlogic: from AO_SEC_GP_CFG0).
	 * A board that sets only the cross-SoC bootsource gets it from there.
	 */
	const char *src = env_get("boot_source");

	if (!src || !*src)
		src = env_get("bootsource");
	bootsettings_set("source", src && *src ? src : "unknown");
	bootsettings_soc();
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(bootsettings, 1, 0, do_bootsettings,
	   "set and print this board's boot settings, decoded",
	   "\n    - boot_source, boot_order (the BootROM's boot order), boot_straps,\n"
	   "      boot_efuse (the boot-relevant fuses), then boot_raw_<register> for each\n"
	   "      register read: environment variables, one name=value per line");
