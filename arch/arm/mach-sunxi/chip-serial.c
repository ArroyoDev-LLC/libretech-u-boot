// SPDX-License-Identifier: GPL-2.0+
/*
 * Allwinner chip id (include/chip_serial.h): "%08x%08x" of SID word 0 and
 * word 3, every phase.  On SoCs newer than A23/A33 word 3 is first replaced by
 * crc32 of words 1-3, and a word 3 whose low 24 bits are zero gets bit 23 set
 * -- the same words board/sunxi/board.c get_unique_sid() hands to the MAC.
 */

#include <chip_serial.h>
#include <errno.h>
#include <asm/arch/cpu.h>
#include <asm/arch/sys_proto.h>

/* crc32(0, ...) of lib/crc32.c, which SPL may not link */
static u32 sid_crc32(const u8 *p, size_t len)
{
	u32 crc = ~0U;
	int k;

	while (len--) {
		crc ^= *p++;
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
	}

	return ~crc;
}

int chip_serial_get(char *buf, size_t size)
{
	unsigned int sid[4];
	u8 be[8];
	int i;

	if (sunxi_get_sid(sid))
		return -ENODEV;
	if (!sid[0])
		return -ENODATA;
#if !defined(CONFIG_MACH_SUN4I) && !defined(CONFIG_MACH_SUN5I) && \
    !defined(CONFIG_MACH_SUN6I) && !defined(CONFIG_MACH_SUN7I) && \
    !defined(CONFIG_MACH_SUN8I_A23) && !defined(CONFIG_MACH_SUN8I_A33)
	sid[3] = sid_crc32((const u8 *)&sid[1], 12);
#endif
	if ((sid[3] & 0xffffff) == 0)
		sid[3] |= 0x800000;

	for (i = 0; i < 4; i++) {
		be[i] = sid[0] >> (24 - 8 * i);
		be[4 + i] = sid[3] >> (24 - 8 * i);
	}

	return chip_serial_hex(buf, size, be, sizeof(be));
}
