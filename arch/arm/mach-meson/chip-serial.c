// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic chip id (include/chip_serial.h): the 12 serial bytes as 24 hex, in
 * the order the BootROM's eFuse mirror stores them.
 *
 * U-Boot proper runs non-secure and asks BL31 (GET_CHIP_ID, meson_sm_get_serial()).
 * TPL and SPL run at EL3 before BL31 and read the same 12 bytes from the
 * BootROM's eFuse mirror in SRAM directly: the base and offset BL31's own
 * reader uses (TF-A plat/amlogic/common/aml_sip_svc.c aml_local_get_chip_id(),
 * plat/amlogic/{g12a,gxl}/*_def.h).
 */

#include <chip_serial.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/sm.h>

#if IS_ENABLED(CONFIG_XPL_BUILD)
#if IS_ENABLED(CONFIG_MESON_G12A)
#define CHIP_ID_MIRROR		0xfffe7c04	/* AML_EFUSE_MIRROR_BASE + 0x04 */
#elif IS_ENABLED(CONFIG_MESON_GXL)
#define CHIP_ID_MIRROR		0xd9013c24	/* AML_CHIP_ID_MIRROR_BASE + 0x24 */
#endif
#endif

int chip_serial_get(char *buf, size_t size)
{
	u8 serial[SM_SERIAL_SIZE];

#if !IS_ENABLED(CONFIG_XPL_BUILD)
	int ret = meson_sm_get_serial(serial, sizeof(serial));

	if (ret)
		return ret;
#elif defined(CHIP_ID_MIRROR)
	u8 any = 0;
	int i;

	for (i = 0; i < sizeof(serial); i++) {
		serial[i] = readb(CHIP_ID_MIRROR + i);
		any |= serial[i];
	}
	if (!any)
		return -ENODATA;	/* unburned: BL31 refuses it too */
#else
	return -ENOSYS;
#endif

	return chip_serial_hex(buf, size, serial, sizeof(serial));
}
