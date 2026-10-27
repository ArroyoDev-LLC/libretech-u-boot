/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The bootsettings command's per-SoC decode (cmd/bootsettings.c).
 */

#ifndef __BOOTSETTINGS_H
#define __BOOTSETTINGS_H

#include <linux/types.h>

/**
 * bootsettings_set() - set the environment variable boot_<name> to value, and print it
 * @name:	the setting without its boot_ prefix ("order", "straps", "efuse")
 * @value:	its value
 */
void bootsettings_set(const char *name, const char *value);

/**
 * bootsettings_set_raw() - set and print boot_raw_<reg> as 0x%08x
 * @reg:	the register's name ("AO_SEC_GP_CFG0")
 * @value:	what it read
 */
void bootsettings_set_raw(const char *reg, u32 value);

/**
 * bootsettings_soc() - set the family's boot_order, boot_straps, boot_efuse and
 * boot_raw_<register> through bootsettings_set()
 *
 * Called after boot_source. Each setting the family cannot read is set to "unknown", except
 * boot_straps, which a family with no strap pins leaves out. Reads only.
 */
void bootsettings_soc(void);

#endif
