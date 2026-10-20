// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 BayLibre SAS
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <dm.h>
#include <net.h>

int board_init(void)
{
	struct udevice *dev;
	int ret;

	if (CONFIG_IS_ENABLED(USB_GADGET)) {
		ret = uclass_get_device(UCLASS_USB_GADGET_GENERIC, 0, &dev);
		if (ret)
			pr_warn("%s: USB gadget unavailable (%d) -- fastboot/DFU off, continuing boot\n",
				__func__, ret);
	}

	if (CONFIG_IS_ENABLED(USB_ETHER))
		usb_ether_init();

	return 0;
}
