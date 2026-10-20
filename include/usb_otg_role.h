/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2026 Da Xue <da@libre.computer>
 *
 * The OTG port's role from the environment (boot/usb_otg_role.c).
 */

#ifndef __USB_OTG_ROLE_H
#define __USB_OTG_ROLE_H

/* device | host | otg; unset leaves the device tree's dr_mode */
#define USB_OTG_ENV	"usb_otg"

#if IS_ENABLED(CONFIG_USB_OTG_ROLE) && !defined(CONFIG_XPL_BUILD)
/**
 * usb_otg_fdt_fixup() - write env usb_otg as dr_mode on the OTG controller
 * @blob: the FDT the kernel boots
 *
 * Return: 0, also when nothing was written (reported on the console);
 * a libfdt error when the write into @blob failed.
 */
int usb_otg_fdt_fixup(void *blob);
#else
static inline int usb_otg_fdt_fixup(void *blob)
{
	return 0;
}
#endif

#endif /* __USB_OTG_ROLE_H */
