// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2011 Samsung Electronics
 * Lukasz Majewski <l.majewski@samsung.com>
 *
 * Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
 */

#include <blk.h>
#include <command.h>
#include <console.h>
#include <dm/device.h>
#include <errno.h>
#include <g_dnl.h>
#include <malloc.h>
#include <mmc.h>
#include <part.h>
#include <usb.h>
#include <usb_mass_storage.h>
#include <watchdog.h>
#include <linux/delay.h>
#include <linux/printk.h>

static int ums_read_sector(struct ums *ums_dev,
			   ulong start, lbaint_t blkcnt, void *buf)
{
	struct blk_desc *block_dev = &ums_dev->block_dev;
	lbaint_t blkstart = start + ums_dev->start_sector;
	int ret;

	ret = blk_dselect_hwpart(block_dev, ums_dev->hwpart);
	if (ret && ret != -ENOSYS)
		return ret;

	return blk_dread(block_dev, blkstart, blkcnt, buf);
}

static int ums_write_sector(struct ums *ums_dev,
			    ulong start, lbaint_t blkcnt, const void *buf)
{
	struct blk_desc *block_dev = &ums_dev->block_dev;
	lbaint_t blkstart = start + ums_dev->start_sector;
	int ret;

	ret = blk_dselect_hwpart(block_dev, ums_dev->hwpart);
	if (ret && ret != -ENOSYS)
		return ret;

	return blk_dwrite(block_dev, blkstart, blkcnt, buf);
}

/*
 * SCSI UNMAP on an MMC LUN. mmc_berase() widens a range that is not
 * erase-group aligned to whole groups when the card cannot TRIM, which
 * would erase blocks the host never named; so without TRIM only the
 * group-aligned middle of the range is erased and its edges are left as
 * they are. With TRIM, or erase groups of one block (SD), the range is
 * erased exactly. Any other block device is erased through its driver's
 * erase op, which works on exact block ranges (blkmap RAM: zeroes).
 */
static int ums_erase_sector(struct ums *ums_dev, ulong start, lbaint_t blkcnt)
{
	struct blk_desc *block_dev = &ums_dev->block_dev;
	lbaint_t blkstart = start + ums_dev->start_sector;
	struct mmc *mmc;
	lbaint_t grp, first, end;
	int ret;

	if (!blkcnt)
		return 0;
	if (block_dev->uclass_id == UCLASS_MMC) {
		if (!CONFIG_IS_ENABLED(MMC_WRITE))
			return -EOPNOTSUPP;
		mmc = find_mmc_device(block_dev->devnum);
		if (!mmc)
			return -ENODEV;
		grp = mmc->erase_grp_size ? mmc->erase_grp_size : 1;
		if (!mmc->can_trim && grp > 1) {
			first = DIV_ROUND_UP(blkstart, grp) * grp;
			end = (blkstart + blkcnt) / grp * grp;
			if (end <= first)
				return 0;
			blkstart = first;
			blkcnt = end - first;
		}
	}

	ret = blk_dselect_hwpart(block_dev, ums_dev->hwpart);
	if (ret && ret != -ENOSYS)
		return ret;

	return blk_derase(block_dev, blkstart, blkcnt) == blkcnt ? 0 : -EIO;
}

/* Whether this LUN's device can take ums_erase_sector(). */
static bool ums_can_erase(struct blk_desc *block_dev)
{
	if (block_dev->uclass_id == UCLASS_MMC)
		return CONFIG_IS_ENABLED(MMC_WRITE);
#if CONFIG_IS_ENABLED(BLK)
	/* blk-uclass.c's blk_get_ops() is private to it */
	return block_dev->bdev &&
	       ((const struct blk_ops *)block_dev->bdev->driver->ops)->erase;
#else
	return false;
#endif
}

static struct ums *ums;
static int ums_count;

static void ums_fini(void)
{
	int i;

	for (i = 0; i < ums_count; i++)
		free((void *)ums[i].name);
	free(ums);
	ums = NULL;
	ums_count = 0;
}

#define UMS_NAME_LEN 16

static int ums_init(const char *devtype, const char *devnums_part_str, bool ro)
{
	char *s, *t, *devnum_part_str, *name;
	struct blk_desc *block_dev;
	struct disk_partition info;
	int partnum;
	int ret = -1;
	struct ums *ums_new;

	s = strdup(devnums_part_str);
	if (!s)
		return -1;

	t = s;
	ums_count = 0;

	for (;;) {
		devnum_part_str = strsep(&t, ",");
		if (!devnum_part_str)
			break;

		partnum = part_get_info_by_dev_and_name_or_num(devtype, devnum_part_str,
							       &block_dev, &info, 1);

		if (partnum < 0)
			goto cleanup;

		/* Check if the argument is in legacy format. If yes,
		 * expose all partitions by setting the partnum = 0
		 * e.g. ums 0 mmc 0
		 */
		if (!strchr(devnum_part_str, ':'))
			partnum = 0;

		ums_new = realloc(ums, (ums_count + 1) * sizeof(*ums));
		if (!ums_new)
			goto cleanup;
		ums = ums_new;

		/* if partnum = 0, expose all partitions */
		if (partnum == 0) {
			ums[ums_count].start_sector = 0;
			ums[ums_count].num_sectors = block_dev->lba;
		} else {
			ums[ums_count].start_sector = info.start;
			ums[ums_count].num_sectors = info.size;
		}

		ums[ums_count].read_sector = ums_read_sector;
		ums[ums_count].write_sector = ums_write_sector;
		ums[ums_count].erase_sector = (!ro && ums_can_erase(block_dev)) ?
					      ums_erase_sector : NULL;

		name = malloc(UMS_NAME_LEN);
		if (!name)
			goto cleanup;
		snprintf(name, UMS_NAME_LEN, "UMS disk %d", ums_count);
		ums[ums_count].name = name;
		ums[ums_count].block_dev = *block_dev;
		ums[ums_count].hwpart = block_dev->hwpart;
		/* realloc() does not clear the new entry */
		ums[ums_count].ro = ro;

		printf("UMS: LUN %d, dev %s %d, hwpart %d, sector %#x, count %#x\n",
		       ums_count, devtype, ums[ums_count].block_dev.devnum,
		       ums[ums_count].block_dev.hwpart,
		       ums[ums_count].start_sector,
		       ums[ums_count].num_sectors);

		ums_count++;
	}

	if (ums_count)
		ret = 0;

cleanup:
	free(s);

	if (ret < 0)
		ums_fini();

	return ret;
}

static int do_usb_mass_storage(struct cmd_tbl *cmdtp, int flag,
			       int argc, char *const argv[])
{
	const char *usb_controller;
	const char *devtype;
	const char *devnum;
	unsigned int controller_index;
	struct udevice *udc;
	int rc;
	int cable_ready_timeout __maybe_unused;
	bool ro = false;

	/* -r: every LUN read-only (MODE SENSE WP; WRITE refused, DATA PROTECT) */
	if (argc > 1 && !strcmp(argv[1], "-r")) {
		ro = true;
		argc--;
		argv++;
	}

	if (argc < 3)
		return CMD_RET_USAGE;

	usb_controller = argv[1];
	if (argc >= 4) {
		devtype = argv[2];
		devnum  = argv[3];
	} else {
		devtype = "mmc";
		devnum  = argv[2];
	}

	rc = ums_init(devtype, devnum, ro);
	if (rc < 0)
		return CMD_RET_FAILURE;

	controller_index = (unsigned int)(simple_strtoul(
				usb_controller,	NULL, 0));
	rc = udc_device_get_by_index(controller_index, &udc);
	if (rc) {
		pr_err("Couldn't init USB controller.\n");
		rc = CMD_RET_FAILURE;
		goto cleanup_ums_init;
	}

	rc = fsg_init(ums, ums_count, udc);
	if (rc) {
		pr_err("fsg_init failed\n");
		rc = CMD_RET_FAILURE;
		goto cleanup_board;
	}

	rc = g_dnl_register("usb_dnl_ums");
	if (rc) {
		pr_err("g_dnl_register failed\n");
		rc = CMD_RET_FAILURE;
		goto cleanup_board;
	}

	/* Timeout unit: seconds */
	cable_ready_timeout = UMS_CABLE_READY_TIMEOUT;

	if (!g_dnl_board_usb_cable_connected()) {
		/*
		 * Won't execute if we don't know whether the cable is
		 * connected.
		 */
		puts("Please connect USB cable.\n");

		while (!g_dnl_board_usb_cable_connected()) {
			if (ctrlc()) {
				puts("\rCTRL+C - Operation aborted.\n");
				rc = CMD_RET_SUCCESS;
				goto cleanup_register;
			}
			if (!cable_ready_timeout) {
				puts("\rUSB cable not detected.\n" \
				     "Command exit.\n");
				rc = CMD_RET_SUCCESS;
				goto cleanup_register;
			}

			printf("\rAuto exit in: %.2d s.", cable_ready_timeout);
			mdelay(1000);
			cable_ready_timeout--;
		}
		puts("\r\n");
	}

	while (1) {
		dm_usb_gadget_handle_interrupts(udc);

		rc = fsg_main_thread(NULL);
		if (rc) {
			/* Check I/O error */
			if (rc == -EIO)
				printf("\rCheck USB cable connection\n");

			/* Check CTRL+C */
			if (rc == -EPIPE)
				printf("\rCTRL+C - Operation aborted\n");

			rc = CMD_RET_SUCCESS;
			goto cleanup_register;
		}

		if (IS_ENABLED(CONFIG_CMD_UMS_ABORT_KEYED)) {
			/* Abort by pressing any key */
			if (tstc()) {
				getchar();
				printf("\rOperation aborted.\n");
				rc = CMD_RET_SUCCESS;
				goto cleanup_register;
			}
		}

		schedule();
	}

cleanup_register:
	g_dnl_release(udc);
	goto cleanup_ums_init;
cleanup_board:
	udc_device_put(udc);
cleanup_ums_init:
	ums_fini();

	return rc;
}

U_BOOT_CMD(ums, 5, 1, do_usb_mass_storage,
	"Use the UMS [USB Mass Storage]",
	"[-r] <USB_controller> [<devtype>] <dev[:part]>  e.g. ums 0 mmc 0\n"
	"    devtype defaults to mmc; -r exports every LUN read-only"
);
