// SPDX-License-Identifier: GPL-2.0+
/*
 * g_dnl.c -- USB Downloader Gadget
 *
 * Copyright (C) 2012 Samsung Electronics
 * Lukasz Majewski  <l.majewski@samsung.com>
 */

#include <chip_serial.h>
#include <log.h>
#include <malloc.h>

#include <mmc.h>
#include <part.h>
#include <usb.h>

#include <g_dnl.h>
#include <linux/string.h>
#include <usb_mass_storage.h>
#include <dfu.h>
#include <thor.h>
#include <version.h>

#include <env_callback.h>

#include "composite.c"

/*
 * One needs to define the following:
 * CONFIG_USB_GADGET_VENDOR_NUM
 * CONFIG_USB_GADGET_PRODUCT_NUM
 * CONFIG_USB_GADGET_MANUFACTURER
 * at e.g. ./configs/<board>_defconfig
 */

#define STRING_MANUFACTURER 25
#define STRING_PRODUCT 2
/* Index of String Descriptor describing this configuration */
#define STRING_USBDOWN 2
/* Index of String serial */
#define STRING_SERIAL  3
#define MAX_STRING_SERIAL	256
/* Number of supported configurations */
#define CONFIGURATION_NUMBER 1

#define DRIVER_VERSION		"usb_dnl 2.0"

static char g_dnl_serial[MAX_STRING_SERIAL];
static const char manufacturer[] = CONFIG_USB_GADGET_MANUFACTURER;
static const char product[] = CONFIG_USB_GADGET_PRODUCT;

void g_dnl_set_serialnumber(char *s)
{
	memset(g_dnl_serial, 0, MAX_STRING_SERIAL);
	strncpy(g_dnl_serial, s, MAX_STRING_SERIAL - 1);
}

static struct usb_device_descriptor device_desc = {
	.bLength = sizeof device_desc,
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB = __constant_cpu_to_le16(0x0200),
	.bDeviceClass = USB_CLASS_PER_INTERFACE,
	.bDeviceSubClass = 0, /*0x02:CDC-modem , 0x00:CDC-serial*/

	.idVendor = __constant_cpu_to_le16(CONFIG_USB_GADGET_VENDOR_NUM),
	.idProduct = __constant_cpu_to_le16(CONFIG_USB_GADGET_PRODUCT_NUM),
	/* .iProduct = DYNAMIC */
	/* .iSerialNumber = DYNAMIC */
	.bNumConfigurations = 1,
};

/*
 * static strings, in UTF-8
 * IDs for those strings are assigned dynamically at g_dnl_bind()
 */
static struct usb_string g_dnl_string_defs[] = {
	{.s = manufacturer},
	{.s = product},
	{.s = g_dnl_serial},
	{ }		/* end of list */
};

static struct usb_gadget_strings g_dnl_string_tab = {
	.language = 0x0409, /* en-us */
	.strings = g_dnl_string_defs,
};

static struct usb_gadget_strings *g_dnl_composite_strings[] = {
	&g_dnl_string_tab,
	NULL,
};

void g_dnl_set_product(const char *s)
{
	if (s)
		g_dnl_string_defs[1].s = s;
	else
		g_dnl_string_defs[1].s = product;
}

static int g_dnl_unbind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;

	debug("%s: calling usb_gadget_disconnect for "
			"controller '%s'\n", __func__, gadget->name);
	usb_gadget_disconnect(gadget);

	return 0;
}

static inline struct g_dnl_bind_callback *g_dnl_bind_callback_first(void)
{
	return ll_entry_start(struct g_dnl_bind_callback,
				g_dnl_bind_callbacks);
}

static inline struct g_dnl_bind_callback *g_dnl_bind_callback_end(void)
{
	return ll_entry_end(struct g_dnl_bind_callback,
				g_dnl_bind_callbacks);
}

/*
 * USB_GADGET_DNL_ACM: the DFU gadget also carries a CDC ACM function in the
 * same configuration, so the host flashing the board sees its console too.
 * DFU uses ep0 only, so the ACM endpoints come out of the UDC's data
 * endpoints without taking anything from DFU.
 */
static bool g_dnl_with_acm(const char *name)
{
	return CONFIG_IS_ENABLED(USB_GADGET_DNL_ACM) &&
	       !strcmp(name, "usb_dnl_dfu");
}

/*
 * USB_FUNCTION_ACM_DFU: the usbacm console's gadget with a resident DFU
 * function beside it, when the console built its entity list (f_acm.c).
 */
static bool g_dnl_console_with_dfu(const char *name)
{
	return IS_ENABLED(CONFIG_USB_FUNCTION_ACM_DFU) &&
	       !IS_ENABLED(CONFIG_XPL_BUILD) &&
	       !strcmp(name, "usb_serial_acm") && acm_console_has_dfu();
}

static int g_dnl_do_config(struct usb_configuration *c)
{
	const char *s = c->cdev->driver->name;
	struct g_dnl_bind_callback *callback = g_dnl_bind_callback_first();
	int ret;

	debug("%s: configuration: 0x%p composite dev: 0x%p\n",
	      __func__, c, c->cdev);

	for (; callback != g_dnl_bind_callback_end(); callback++) {
		if (strcmp(s, callback->usb_function_name))
			continue;

		ret = callback->fptr(c);
		if (!ret && g_dnl_console_with_dfu(s)) {
			/* The console stays up whether or not DFU could join it */
			ret = dfu_add_resident(c);
			if (ret)
				printf("%s: no DFU beside the console: %d\n",
				       __func__, ret);
			return 0;
		}
		if (ret || !g_dnl_with_acm(s))
			return ret;

		/* DFU stays up whether or not the console could join it */
		if (CONFIG_IS_ENABLED(USB_GADGET_DNL_ACM)) {
			ret = acm_dnl_attach(c);
			if (ret)
				printf("%s: no ACM console beside %s: %d\n",
				       __func__, s, ret);
		}
		return 0;
	}
	return -ENODEV;
}

static int g_dnl_config_register(struct usb_composite_dev *cdev)
{
	struct usb_configuration *config;
	const char *name = "usb_dnload";

	config = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(*config));
	if (!config)
		return -ENOMEM;

	memset(config, 0, sizeof(*config));

	config->label = name;
	config->bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER;
	config->bConfigurationValue = CONFIGURATION_NUMBER;
	config->iConfiguration = STRING_USBDOWN;
	config->bind = g_dnl_do_config;

	return usb_add_config(cdev, config);
}

__weak
int board_usb_init(int index, enum usb_init_type init)
{
	return 0;
}

__weak
int board_usb_cleanup(int index, enum usb_init_type init)
{
	return 0;
}

__weak
int g_dnl_bind_fixup(struct usb_device_descriptor *dev, const char *name)
{
	return 0;
}

__weak int g_dnl_get_board_bcd_device_number(int gcnum)
{
	return gcnum;
}

__weak int g_dnl_board_usb_cable_connected(void)
{
	return -EOPNOTSUPP;
}

static bool g_dnl_detach_request;

bool g_dnl_detach(void)
{
	return g_dnl_detach_request;
}

void g_dnl_trigger_detach(void)
{
	g_dnl_detach_request = true;
}

void g_dnl_clear_detach(void)
{
	g_dnl_detach_request = false;
}

/**
 * Update internal serial number variable when the "serial#" env var changes.
 *
 * Handle all cases, even when flags == H_PROGRAMMATIC or op == env_op_delete.
 */
static int on_serialno(const char *name, const char *value, enum env_op op,
		int flags)
{
	if (value)
		g_dnl_set_serialnumber((char *)value);
	return 0;
}
U_BOOT_ENV_CALLBACK(serialno, on_serialno);

static int g_dnl_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	int id, ret;
	int gcnum;

	debug("%s: gadget: 0x%p cdev: 0x%p\n", __func__, gadget, cdev);

	id = usb_string_id(cdev);

	if (id < 0)
		return id;
	g_dnl_string_defs[0].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;

	g_dnl_string_defs[1].id = id;
	device_desc.iProduct = id;

	/*
	 * A multi-function device whose ACM pair is grouped by an IAD declares
	 * the IAD class triple (USB IAD ECN); every other gadget keeps the
	 * per-interface class. device_desc is static, so set it each bind.
	 */
	if (g_dnl_with_acm(cdev->driver->name) ||
	    g_dnl_console_with_dfu(cdev->driver->name)) {
		device_desc.bDeviceClass = USB_CLASS_MISC;
		device_desc.bDeviceSubClass = 0x02;
		device_desc.bDeviceProtocol = 0x01;
	} else {
		device_desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
		device_desc.bDeviceSubClass = 0;
		device_desc.bDeviceProtocol = 0;
	}

	/*
	 * The ids too: the console's 0525:a4a7 below would otherwise outlive its
	 * gadget, and a board fixup that sets only the product id then gave the
	 * next one (DFU after the usbacm console) 0525:330c.
	 */
	device_desc.idVendor = cpu_to_le16(CONFIG_USB_GADGET_VENDOR_NUM);
	device_desc.idProduct = cpu_to_le16(CONFIG_USB_GADGET_PRODUCT_NUM);

	g_dnl_bind_fixup(&device_desc, cdev->driver->name);

	/*
	 * After the board's fixup, which may set the product id per gadget: a
	 * gadget carrying the console, the stand-alone usbacm one or DFU with
	 * ACM beside it, takes the Linux-USB serial gadget's id
	 * (USB_FUNCTION_ACM_LINUX_GADGET_ID), the one a CDC ACM device needs for
	 * ChromeOS to pass it on to Android or a VM.
	 */
	if (IS_ENABLED(CONFIG_USB_FUNCTION_ACM_LINUX_GADGET_ID) &&
	    (!strcmp(cdev->driver->name, "usb_serial_acm") ||
	     g_dnl_with_acm(cdev->driver->name))) {
		device_desc.idVendor = cpu_to_le16(0x0525);
		device_desc.idProduct = cpu_to_le16(0xa4a7);
	}

	/*
	 * iSerialNumber is the chip id in every phase that can read it
	 * (include/chip_serial.h), an environment-less xPL included.  In U-Boot
	 * proper serial# holds the same string, so the env callback and the
	 * functions' own serial# reads agree with it.
	 */
	{
		char serial[CHIP_SERIAL_MAX];

		if (!chip_serial_get(serial, sizeof(serial)))
			g_dnl_set_serialnumber(serial);
	}

	if (strlen(g_dnl_serial)) {
		id = usb_string_id(cdev);
		if (id < 0)
			return id;

		g_dnl_string_defs[2].id = id;
		device_desc.iSerialNumber = id;
	}

	ret = g_dnl_config_register(cdev);
	if (ret)
		goto error;

	gcnum = g_dnl_get_board_bcd_device_number((U_BOOT_VERSION_NUM << 4) |
						  U_BOOT_VERSION_NUM_PATCH);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(gcnum);
	else {
		debug("%s: controller '%s' not recognized\n",
			__func__, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	debug("%s: calling usb_gadget_connect for "
			"controller '%s'\n", __func__, gadget->name);
	usb_gadget_connect(gadget);

	return 0;

 error:
	g_dnl_unbind(cdev);
	return -ENOMEM;
}

static struct usb_composite_driver g_dnl_driver = {
	.name = NULL,
	.dev = &device_desc,
	.strings = g_dnl_composite_strings,
	.max_speed = USB_SPEED_SUPER,

	.bind = g_dnl_bind,
	.unbind = g_dnl_unbind,
};

/*
 * NOTICE:
 * Registering via USB function name won't be necessary after rewriting
 * g_dnl to support multiple USB functions.
 */
/* The usbacm console's own gadget, which a download gadget takes the UDC from */
#define G_DNL_ACM_CONSOLE	"usb_serial_acm"

static bool g_dnl_is_acm_console(const char *name)
{
	return name && !strcmp(name, G_DNL_ACM_CONSOLE);
}

/* A download gadget (not the usbacm console's) holds the UDC */
static bool g_dnl_download_up;

bool g_dnl_download_active(void)
{
	return g_dnl_download_up;
}

int g_dnl_register(const char *name)
{
	int ret;

	debug("%s: g_dnl_driver.name = %s\n", __func__, name);
	/*
	 * One UDC, one composite gadget: a running usbacm console steps aside
	 * for dfu/fastboot/ums and comes back at g_dnl_unregister().
	 */
	if (IS_ENABLED(CONFIG_USB_FUNCTION_ACM) && !IS_ENABLED(CONFIG_XPL_BUILD) &&
	    !g_dnl_is_acm_console(name))
		acm_console_suspend();
	g_dnl_driver.name = name;

	ret = usb_composite_register(&g_dnl_driver);
	if (ret) {
		printf("%s: failed!, error: %d\n", __func__, ret);
		return ret;
	}
	g_dnl_download_up = !g_dnl_is_acm_console(name);
	return 0;
}

void g_dnl_unregister(void)
{
	bool resume = !g_dnl_is_acm_console(g_dnl_driver.name);

	usb_composite_unregister(&g_dnl_driver);
	g_dnl_download_up = false;
	if (IS_ENABLED(CONFIG_USB_FUNCTION_ACM) && !IS_ENABLED(CONFIG_XPL_BUILD) &&
	    resume)
		acm_console_resume();
}

/*
 * End a download gadget that ran on @udc: unbind it while the controller is
 * still there, release the UDC (with device removal the controller goes),
 * then bring a suspended usbacm console back, which gets the UDC afresh.
 * Neither order of g_dnl_unregister() and udc_device_put() is right once the
 * put removes the controller: unregistering first resumed the console on a
 * UDC about to be removed (SError in acm_stdio_puts()); putting first
 * unbound the gadget -- and, in an SPL with the ACM console beside DFU,
 * queued its console output -- on a removed controller, and the next gadget
 * on it never came up.
 */
void g_dnl_release(struct udevice *udc)
{
	bool resume = !g_dnl_is_acm_console(g_dnl_driver.name);

	usb_composite_unregister(&g_dnl_driver);
	g_dnl_download_up = false;
	udc_device_put(udc);
	if (IS_ENABLED(CONFIG_USB_FUNCTION_ACM) && !IS_ENABLED(CONFIG_XPL_BUILD) &&
	    resume)
		acm_console_resume();
}
