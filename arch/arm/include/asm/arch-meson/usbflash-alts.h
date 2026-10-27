/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * The Libre Computer Meson DFU alternate names and storage extents, shared by
 * the two layers that serve a flash session: U-Boot proper
 * (board/libre-computer/common/board-meson.c, meson_set_dfu_alt_info()) and
 * the G12A/SM1 DRAM-stage SPL (arch/arm/mach-meson/spl-usbflash.c).  Both
 * build their lists from these definitions, so the host sees the same alt
 * names and extents whichever layer answered
 * (amlogic/software/boot/spl-usb-flasher-plan.md section 3).
 */

#ifndef __MESON_USBFLASH_ALTS_H__
#define __MESON_USBFLASH_ALTS_H__

/* mmc 0 (eMMC): raw alternates, `name raw <lba> <blocks> mmcpart <n>` */
#define MESON_DFU_MMC_EMMC_ALTS \
	"u-boot-bin-emmc raw 1 0x7ff mmcpart 0;" \
	"u-boot-bin-emmc-boot0 raw 0x1 0x7ff mmcpart 1;" \
	"u-boot-bin-emmc-boot1 raw 0x1 0x7ff mmcpart 2;" \
	"emmc raw 0 0 mmcpart 0;" \
	"emmc-boot0 raw 0 0 mmcpart 1;" \
	"emmc-boot1 raw 0 0 mmcpart 2"

/* mmc 1 (SD) */
#define MESON_DFU_MMC_SD_ALTS \
	"u-boot-bin-sd raw 1 0x7ff mmcpart 0;" \
	"sd raw 0 0 mmcpart 0"

/*
 * SPI NOR regions (byte offsets on the part).  The config alt's offset is not
 * a constant: it is board_config_nor_offset() (boot/board-config-nor.c), the
 * same function the config-block reader uses, and its size is
 * CONFIG_BOARD_CONFIG_NOR_SIZE.  The FIT region is clamped so it cannot run
 * into the config block.  Rationale at meson_dfu_sf_alts() in board-meson.c.
 */
#define MESON_DFU_SF_UBOOT_SIZE		0x1F0000
#define MESON_DFU_SF_FIT_OFFSET		0x200000
#define MESON_DFU_SF_FIT_SIZE		0x200000

#ifndef __ASSEMBLY__
#include <linux/types.h>

/*
 * The RDID bytes of the last board_nor_probe() of the bare SPIFC reader
 * (board/libre-computer/common/meson-spifc-nor.c), id[0] in bits 23:16.
 */
u32 board_nor_jedec_id(void);

/* The bare SPIFC reader's clock divider: clk81 / (@n + 1). */
void board_nor_set_div(u32 n);
#endif

#endif /* __MESON_USBFLASH_ALTS_H__ */
