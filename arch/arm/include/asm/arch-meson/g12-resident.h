/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Amlogic G12A/SM1 resident chain, the TPL's boot count
 * (CONFIG_MESON_G12A_RESIDENT_HANG_RETRIES, arch/arm/mach-meson/g12-resident.c).
 */

#ifndef __ASM_ARCH_MESON_G12_RESIDENT_H
#define __ASM_ARCH_MESON_G12_RESIDENT_H

#include <linux/bitops.h>
#include <linux/types.h>

/*
 * The count's register: AO_RTI_STATUS_REG1, BSP v2019 scp_task/secure_apb.h:2795.
 * [31:8] G12A_RESIDENT_HANG_MAGIC ("HNG"), so a value anything else left there
 * reads as no count; [7:0] the boots counted.
 */
#define G12A_RESIDENT_HANG_REG		0xFF800004UL
#define G12A_RESIDENT_HANG_MAGIC	0x484e4700u
#define G12A_RESIDENT_HANG_MAGIC_MASK	0xffffff00u

/* g12a_resident_hang(): [7:0] the count, and these flags */
#define G12A_RESIDENT_HANG_COUNT	GENMASK(7, 0)
/* this boot was counted; [7:0] includes it */
#define G12A_RESIDENT_HANG_COUNTED	BIT(8)
/* N boots were counted: the word is armed, the watchdog about to fire */
#define G12A_RESIDENT_HANG_AT_N		BIT(9)
/* TEST ONLY (MESON_G12A_RESIDENT_HANG_TEST): stop after counting */
#define G12A_RESIDENT_HANG_TEST		BIT(10)

u32 g12a_resident_hang(void);

#endif
