// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019 Rockchip Electronics Co., Ltd.
 *
 * Copyright (C) 2019 Collabora Inc - https://www.collabora.com/
 *      Rohan Garg <rohan.garg@collabora.com>
 *
 * Based on puma-rk3399.c:
 *      (C) Copyright 2017 Theobroma Systems Design und Consulting GmbH
 */
#include <config.h>
#include <chip_serial.h>
#include <clk.h>
#include <cpu_func.h>
#include <env.h>
#include <env_internal.h>
#include <errno.h>
#include <search.h>
#include <dm.h>
#include <dm/uclass-internal.h>
#include <efi_loader.h>
#include <fastboot.h>
#include <hash.h>
#include <init.h>
#include <log.h>
#include <mmc.h>
#include <dm/uclass-internal.h>
#include <misc.h>
#include <part.h>
#include <ram.h>
#include <syscon.h>
#include <u-boot/uuid.h>
#include <u-boot/crc.h>
#include <u-boot/sha256.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/arch-rockchip/boot_mode.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/periph.h>
#include <power/regulator.h>

#if IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT) && IS_ENABLED(CONFIG_EFI_PARTITION)

#define DFU_ALT_BUF_LEN			SZ_1K

static struct efi_fw_image *fw_images;

static bool updatable_image(struct disk_partition *info)
{
	int i;
	bool ret = false;
	efi_guid_t image_type_guid;

	uuid_str_to_bin(info->type_guid, image_type_guid.b,
			UUID_STR_FORMAT_GUID);

	for (i = 0; i < update_info.num_images; i++) {
		if (!guidcmp(&fw_images[i].image_type_id, &image_type_guid)) {
			ret = true;
			break;
		}
	}

	return ret;
}

static void set_image_index(struct disk_partition *info, int index)
{
	int i;
	efi_guid_t image_type_guid;

	uuid_str_to_bin(info->type_guid, image_type_guid.b,
			UUID_STR_FORMAT_GUID);

	for (i = 0; i < update_info.num_images; i++) {
		if (!guidcmp(&fw_images[i].image_type_id, &image_type_guid)) {
			fw_images[i].image_index = index;
			break;
		}
	}
}

static int get_mmc_desc(struct blk_desc **desc)
{
	int ret;
	struct mmc *mmc;
	struct udevice *dev;

	/*
	 * For now the firmware images are assumed to
	 * be on the SD card
	 */
	ret = uclass_get_device(UCLASS_MMC, 1, &dev);
	if (ret)
		return -1;

	mmc = mmc_get_mmc_dev(dev);
	if (!mmc)
		return -ENODEV;

	if ((ret = mmc_init(mmc)))
		return ret;

	*desc = mmc_get_blk_desc(mmc);
	if (!*desc)
		return -1;

	return 0;
}

void set_dfu_alt_info(char *interface, char *devstr)
{
	const char *name;
	bool first = true;
	int p, len, devnum, ret;
	char buf[DFU_ALT_BUF_LEN];
	struct disk_partition info;
	struct blk_desc *desc = NULL;

	ret = get_mmc_desc(&desc);
	if (ret) {
		log_err("Unable to get mmc desc\n");
		return;
	}

	memset(buf, 0, sizeof(buf));
	name = blk_get_uclass_name(desc->uclass_id);
	devnum = desc->devnum;
	len = strlen(buf);

	len += snprintf(buf + len, DFU_ALT_BUF_LEN - len,
			 "%s %d=", name, devnum);

	for (p = 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		if (part_get_info(desc, p, &info))
			continue;

		/* Add entry to dfu_alt_info only for updatable images */
		if (updatable_image(&info)) {
			if (!first)
				len += snprintf(buf + len,
						DFU_ALT_BUF_LEN - len, ";");

			len += snprintf(buf + len, DFU_ALT_BUF_LEN - len,
					"%s%d_%s part %d %d",
					name, devnum, info.name, devnum, p);
			first = false;
		}
	}

	log_debug("dfu_alt_info => %s\n", buf);
	env_set("dfu_alt_info", buf);
}

__weak void rockchip_capsule_update_board_setup(void)
{
}

static void gpt_capsule_update_setup(void)
{
	int p, i, ret;
	struct disk_partition info;
	struct blk_desc *desc = NULL;

	fw_images = update_info.images;
	rockchip_capsule_update_board_setup();

	ret = get_mmc_desc(&desc);
	if (ret) {
		log_err("Unable to get mmc desc\n");
		return;
	}

	for (p = 1, i = 1; p <= MAX_SEARCH_PARTITIONS; p++) {
		if (part_get_info(desc, p, &info))
			continue;

		/*
		 * Since we have a GPT partitioned device, the updatable
		 * images could be stored in any order. Populate the
		 * image_index at runtime.
		 */
		if (updatable_image(&info)) {
			set_image_index(&info, i);
			i++;
		}
	}
}
#endif /* CONFIG_EFI_HAVE_CAPSULE_SUPPORT && CONFIG_EFI_PARTITION */

__weak int rk_board_late_init(void)
{
	return 0;
}

int board_late_init(void)
{
	setup_boot_mode();

#if IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT) && IS_ENABLED(CONFIG_EFI_PARTITION)
	gpt_capsule_update_setup();
#endif

	return rk_board_late_init();
}

int board_init(void)
{
	return 0;
}

#if !defined(CONFIG_SYS_DCACHE_OFF) && !defined(CONFIG_ARM64)
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

#if IS_ENABLED(CONFIG_USB_GADGET)
#include <usb.h>

#if IS_ENABLED(CONFIG_USB_GADGET_DOWNLOAD)
#define ROCKCHIP_G_DNL_UMS_PRODUCT_NUM	0x0010

int g_dnl_bind_fixup(struct usb_device_descriptor *dev, const char *name)
{
	if (!strcmp(name, "usb_dnl_ums"))
		put_unaligned(ROCKCHIP_G_DNL_UMS_PRODUCT_NUM, &dev->idProduct);
	else
		put_unaligned(CONFIG_USB_GADGET_PRODUCT_NUM, &dev->idProduct);

	return 0;
}
#endif /* CONFIG_USB_GADGET_DOWNLOAD */

#if IS_ENABLED(CONFIG_USB_GADGET_DWC2_OTG) && !IS_ENABLED(CONFIG_DM_USB_GADGET)
#include <linux/usb/otg.h>
#include <usb/dwc2_udc.h>

static struct dwc2_plat_otg_data otg_data = {
	.rx_fifo_sz	= 512,
	.np_tx_fifo_sz	= 16,
	.tx_fifo_sz	= 128,
};

int board_usb_init(int index, enum usb_init_type init)
{
	ofnode node;
	bool matched = false;

	/* find the usb_otg node */
	node = ofnode_by_compatible(ofnode_null(), "snps,dwc2");
	while (ofnode_valid(node)) {
		switch (usb_get_dr_mode(node)) {
		case USB_DR_MODE_OTG:
		case USB_DR_MODE_PERIPHERAL:
			matched = true;
			break;

		default:
			break;
		}

		if (matched)
			break;

		node = ofnode_by_compatible(node, "snps,dwc2");
	}
	if (!matched) {
		debug("Not found usb_otg device\n");
		return -ENODEV;
	}
	otg_data.regs_otg = ofnode_get_addr(node);

#ifdef CONFIG_ROCKCHIP_USB2_PHY
	int ret;
	u32 phandle, offset;
	ofnode phy_node;

	ret = ofnode_read_u32(node, "phys", &phandle);
	if (ret)
		return ret;

	node = ofnode_get_by_phandle(phandle);
	if (!ofnode_valid(node)) {
		debug("Not found usb phy device\n");
		return -ENODEV;
	}

	phy_node = ofnode_get_parent(node);
	if (!ofnode_valid(node)) {
		debug("Not found usb phy device\n");
		return -ENODEV;
	}

	otg_data.phy_of_node = phy_node;
	ret = ofnode_read_u32(node, "reg", &offset);
	if (ret)
		return ret;
	otg_data.regs_phy =  offset +
		(u32)syscon_get_first_range(ROCKCHIP_SYSCON_GRF);
#endif
	return dwc2_udc_probe(&otg_data);
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	return 0;
}
#endif /* CONFIG_USB_GADGET_DWC2_OTG */
#endif /* CONFIG_USB_GADGET */

#if IS_ENABLED(CONFIG_FASTBOOT)
int fastboot_set_reboot_flag(enum fastboot_reboot_reason reason)
{
	if (reason != FASTBOOT_REBOOT_REASON_BOOTLOADER)
		return -ENOTSUPP;

	printf("Setting reboot to fastboot flag ...\n");
	/* Set boot mode to fastboot */
	writel(BOOT_FASTBOOT, CONFIG_ROCKCHIP_BOOT_MODE_REG);

	return 0;
}
#endif

#ifdef CONFIG_MISC_INIT_R
/*
 * The MAC from the chip id (lib/chip_serial.c chip_serial_ethaddr()): SHA-256 of
 * serial#, the cpu id's 32 raw hex -- the same string this function hashed when
 * it was published as cpuid#, so a board keeps its MAC.
 */
int rockchip_setup_macaddr(void)
{
	int ret = chip_serial_ethaddr();

	return ret == -ENOSYS ? 0 : ret;
}

int rockchip_cpuid_from_efuse(const u32 cpuid_offset,
			      const u32 cpuid_length,
			      u8 *cpuid)
{
#if IS_ENABLED(CONFIG_ROCKCHIP_EFUSE) || IS_ENABLED(CONFIG_ROCKCHIP_OTP)
	struct udevice *dev;
	int ret;

	/* retrieve the device */
#if IS_ENABLED(CONFIG_ROCKCHIP_EFUSE)
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_DRIVER_GET(rockchip_efuse), &dev);
#elif IS_ENABLED(CONFIG_ROCKCHIP_OTP)
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_DRIVER_GET(rockchip_otp), &dev);
#endif
	if (ret) {
		debug("%s: could not find efuse device\n", __func__);
		return -1;
	}

	/* read the cpu_id range from the efuses */
	ret = misc_read(dev, cpuid_offset, cpuid, cpuid_length);
	if (ret < 0) {
		debug("%s: reading cpuid from the efuses failed\n",
		      __func__);
		return -1;
	}
#endif
	return 0;
}

int rockchip_cpuid_set(const u8 *cpuid, const u32 cpuid_length)
{
	/*
	 * serial# is this chip's id, its raw cpu-id bytes, replacing a saved one
	 * from another board (arch/arm/mach-rockchip/chip-serial.c). It is the
	 * only env variable that carries the id (claude docs repo
	 * design/rules/bootloader/policy.md "One variable, the raw bytes"): a
	 * cpuid# a saved environment still holds is deleted.
	 */
	if (env_get("cpuid#")) {
		char *const argv[] = { "setenv", "-f", "cpuid#", NULL };

		printf("cpuid#: dropped, the chip id is serial#\n");
		env_do_env_set(0, 3, argv, H_PROGRAMMATIC);
	}

	chip_serial_env();

	return 0;
}

__weak int rockchip_early_misc_init_r(void)
{
	return 0;
}

__weak int misc_init_r(void)
{
	const u32 cpuid_offset = CFG_CPUID_OFFSET;
	const u32 cpuid_length = 0x10;
	u8 cpuid[cpuid_length];
	int ret;

	ret = rockchip_early_misc_init_r();
	if (ret)
		return ret;

	ret = rockchip_cpuid_from_efuse(cpuid_offset, cpuid_length, cpuid);
	if (ret) {
		chip_serial_env();	/* says so, and drops a stored serial# */
		return ret;
	}

	ret = rockchip_cpuid_set(cpuid, cpuid_length);
	if (ret)
		return ret;

	ret = rockchip_setup_macaddr();

	return ret;
}
#endif

#if IS_ENABLED(CONFIG_BOARD_RNG_SEED) && IS_ENABLED(CONFIG_RNG_ROCKCHIP)
#include <rng.h>

/* Use hardware rng to seed Linux random. */
__weak int board_rng_seed(struct abuf *buf)
{
	struct udevice *dev;
	ulong len = env_get_ulong("rng_seed_size", 10, 64);
	u64 *data;

	if (len < 64) {
		/*
		 * rng_seed_size should be at least 32 bytes for Linux 5.19+,
		 * or 64 for older Linux kernel versions
		 */
		log_warning("Value for rng_seed_size (%lu) too low, Linux kernel RNG may fail to initialize early\n",
			    len);
	}

	data = malloc(len);
	if (!data) {
		printf("Out of memory\n");
		return -ENOMEM;
	}

	if (uclass_get_device(UCLASS_RNG, 0, &dev) || !dev) {
		printf("No RNG device\n");
		return -ENODEV;
	}

	if (dm_rng_read(dev, data, len)) {
		printf("Reading RNG failed\n");
		return -EIO;
	}

	abuf_init_set(buf, data, len);

	return 0;
}
#endif

int mmc_get_env_dev(void)
{
	int devnum;
	const char *boot_device;
	struct udevice *dev;

#ifdef CONFIG_ENV_MMC_DEVICE_INDEX
	devnum = CONFIG_ENV_MMC_DEVICE_INDEX;
#else
	devnum = 0;
#endif

	boot_device = ofnode_read_chosen_string("u-boot,spl-boot-device");
	if (!boot_device) {
		debug("%s: /chosen/u-boot,spl-boot-device not set\n", __func__);
		return devnum;
	}

	debug("%s: booted from %s\n", __func__, boot_device);

	if (uclass_find_device_by_ofnode(UCLASS_MMC, ofnode_path(boot_device), &dev)) {
		/*
		 * The FIT came from something that is not an MMC controller
		 * -- SPI NOR on a NOR-booted board. The environment is a
		 * file on a FAT partition (a NOR has none), so it lives on
		 * the first MMC device that actually has a medium: eMMC if
		 * one answers, else the SD. The default index alone is not
		 * that answer -- on a board whose eMMC slot is empty or dead
		 * it names a device that never initialises, and the env on
		 * the SD card is never found.
		 */
		struct mmc *mmc;
		int i;

		debug("%s: no U-Boot MMC device for %s, first present MMC\n",
		      __func__, boot_device);
		for (i = 0; i < 2; i++) {
			mmc = find_mmc_device(i);
			if (mmc && !mmc_init(mmc)) {
				debug("%s: mmc%d present\n", __func__, i);
				return i;
			}
		}
		return devnum;
	}

	devnum = dev->seq_;
	debug("%s: get MMC env from mmc%d\n", __func__, devnum);
	return devnum;
}
