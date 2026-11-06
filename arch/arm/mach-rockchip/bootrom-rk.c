// SPDX-License-Identifier: GPL-2.0+
/*
 * `bootrom` on Rockchip: the BOOT_BROM_DOWNLOAD flag in the boot-mode register.
 *
 * The MaskROM never reads the boot-mode register; our TPL does
 * (arch/arm/mach-rockchip/bootrom.c, check_back_to_brom_dnl_flag()): at its
 * entry it clears a set flag and returns BROM_BOOT_ENTER_DNL, and on an RK3399
 * SPI NOR boot it first puts the NOR in deep power-down so the MaskROM's next
 * read finds no bootable NOR (lc-master 07107e62993). So `forced` here means
 * armed and read back, and holds for a board whose first stage is that TPL.
 *
 * RK3399 limit: after the NOR is put to sleep the MaskROM rescans the later
 * media (eMMC, SD) before USB, so a later medium with a loader boots instead.
 * From a NOR boot, the check reads sector 64 of every MMC device present and
 * answers `unsupported` when one holds an idbloader (the RC4-coded header0 of
 * tools/rkcommon.c, magic RK_MAGIC).
 *
 * Design: claude design/rules/bootloader/reset-to-usb-boot.md R1 (Rockchip, and
 * "Two RK3399 limits").
 */

#include <blk.h>
#include <bootrom.h>
#include <dm.h>
#include <memalign.h>
#include <mmc.h>
#include <rc4.h>
#include <stdio.h>
#include <asm/io.h>
#include <asm/arch-rockchip/boot_mode.h>
#include <dm/ofnode.h>
#include <linux/errno.h>
#include <linux/string.h>

/* tools/rkcommon.c: RK_MAGIC, the header0 sector, its RC4 key */
#define RK_IDB_MAGIC		0x0ff0aa55
#define RK_IDB_SECTOR		64
static const unsigned char rk_idb_rc4_key[16] = {
	124, 78, 3, 4, 85, 5, 9, 7, 45, 44, 123, 56, 23, 13, 23, 17
};

static char bootrom_rk_why[48];

const char *bootrom_arch_reason(void)
{
	return bootrom_rk_why[0] ? bootrom_rk_why : NULL;
}

/* Did this boot come from SPI NOR?  The SPL names its boot device in /chosen. */
static bool bootrom_rk_nor_boot(void)
{
	const char *path = ofnode_read_chosen_string("u-boot,spl-boot-device");
	ofnode node;

	if (!path)
		return false;
	node = ofnode_path(path);
	return ofnode_valid(node) &&
	       ofnode_device_is_compatible(node, "jedec,spi-nor");
}

/* Does sector 64 of the MMC device's user area hold an idbloader header? */
static bool bootrom_rk_mmc_has_loader(struct mmc *mmc)
{
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, buf, 512);
	struct blk_desc *desc;
	u32 magic;

	if (mmc_init(mmc))
		return false;
	desc = mmc_get_blk_desc(mmc);
	if (!desc || blk_dselect_hwpart(desc, 0))
		return false;
	if (blk_dread(desc, RK_IDB_SECTOR, 1, buf) != 1)
		return false;
	/* a header0 written unencoded counts too */
	memcpy(&magic, buf, sizeof(magic));
	if (le32_to_cpu(magic) == RK_IDB_MAGIC)
		return true;
	rc4_encode(buf, 512, rk_idb_rc4_key);
	memcpy(&magic, buf, sizeof(magic));
	return le32_to_cpu(magic) == RK_IDB_MAGIC;
}

int bootrom_arch_check(void)
{
	int i;

	bootrom_rk_why[0] = '\0';
	if (!CONFIG_ROCKCHIP_BOOT_MODE_REG) {
		snprintf(bootrom_rk_why, sizeof(bootrom_rk_why), "no boot-mode register");
		return -ENOSYS;
	}
	if (!IS_ENABLED(CONFIG_ROCKCHIP_RK3399) || !bootrom_rk_nor_boot())
		return 0;

	/* RK3399 from NOR: a later medium with a loader would boot instead */
	for (i = 0; i < get_mmc_num(); i++) {
		struct mmc *mmc = find_mmc_device(i);

		if (mmc && bootrom_rk_mmc_has_loader(mmc)) {
			snprintf(bootrom_rk_why, sizeof(bootrom_rk_why),
				 "mmc %d holds a loader at sector %d", i, RK_IDB_SECTOR);
			return -EEXIST;
		}
	}
	return 0;
}

int bootrom_arch_arm(void)
{
	writel(BOOT_BROM_DOWNLOAD, CONFIG_ROCKCHIP_BOOT_MODE_REG);
	return readl(CONFIG_ROCKCHIP_BOOT_MODE_REG) == BOOT_BROM_DOWNLOAD ? 0 : -EIO;
}

void bootrom_arch_disarm(void)
{
	writel(BOOT_NORMAL, CONFIG_ROCKCHIP_BOOT_MODE_REG);
}
