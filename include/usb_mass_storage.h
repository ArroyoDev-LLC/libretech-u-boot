/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2011 Samsung Electrnoics
 * Lukasz Majewski <l.majewski@samsung.com>
 */

#ifndef __USB_MASS_STORAGE_H__
#define __USB_MASS_STORAGE_H__

#include <part.h>
#include <linux/usb/composite.h>

/* Wait at maximum 60 seconds for cable connection */
#define UMS_CABLE_READY_TIMEOUT	60

struct ums {
	int (*read_sector)(struct ums *ums_dev,
			   ulong start, lbaint_t blkcnt, void *buf);
	int (*write_sector)(struct ums *ums_dev,
			    ulong start, lbaint_t blkcnt, const void *buf);
	/*
	 * Discard [start, start + blkcnt) for a SCSI UNMAP, or NULL when the
	 * LUN cannot discard (then UNMAP is an invalid command and nothing
	 * advertises it). Never touches a block outside the range; may leave
	 * some of the range as it was (UNMAP is advisory). 0 or -errno.
	 */
	int (*erase_sector)(struct ums *ums_dev, ulong start, lbaint_t blkcnt);
	unsigned int start_sector;
	unsigned int num_sectors;
	const char *name;
	struct blk_desc block_dev;
	int hwpart;
	/*
	 * Export the LUN read-only: MODE SENSE reports it write-protected and
	 * f_mass_storage refuses every WRITE with DATA PROTECT before
	 * write_sector is reached.
	 */
	bool ro;
};

int fsg_init(struct ums *ums_devs, int count, struct udevice *udc);
void fsg_cleanup(void);
int fsg_main_thread(void *);
int fsg_add(struct usb_configuration *c);
#endif /* __USB_MASS_STORAGE_H__ */
