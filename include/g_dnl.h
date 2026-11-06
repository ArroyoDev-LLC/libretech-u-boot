/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  Copyright (C) 2012 Samsung Electronics
 *  Lukasz Majewski <l.majewski@samsung.com>
 */

#ifndef __G_DOWNLOAD_H_
#define __G_DOWNLOAD_H_

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <linker_lists.h>

/*
 * @usb_fname: unescaped USB function name
 * @callback_ptr: bind callback, one per function name
 */
#define DECLARE_GADGET_BIND_CALLBACK(usb_fname, callback_ptr) \
	ll_entry_declare(struct g_dnl_bind_callback, \
			__usb_function_name_##usb_fname, \
			g_dnl_bind_callbacks) = { \
				.usb_function_name = #usb_fname, \
				.fptr = callback_ptr \
			}

typedef int (*g_dnl_bind_callback_f)(struct usb_configuration *);

/* used in Gadget downloader callback linker list */
struct g_dnl_bind_callback {
	const char *usb_function_name;
	g_dnl_bind_callback_f fptr;
};

int g_dnl_bind_fixup(struct usb_device_descriptor *, const char *);
int g_dnl_get_board_bcd_device_number(int gcnum);
int g_dnl_board_usb_cable_connected(void);
int g_dnl_register(const char *s);
void g_dnl_unregister(void);
struct udevice;
void g_dnl_release(struct udevice *udc);

/*
 * The usbacm console (f_acm) hands the UDC to a download gadget and takes it
 * back; g_dnl_register()/g_dnl_unregister() call these. No-ops when the
 * console is not started.
 */
void acm_console_suspend(void);
void acm_console_resume(void);
/* USB_FUNCTION_ACM_DFU: the console's gadget carries its resident DFU function */
bool acm_console_has_dfu(void);
/* CONSOLE_PREFER_USB: the usbacm console's host holds the port open and reads */
bool console_usb_host_ready(void);
/*
 * The next usbacm start keeps its gadget up when no host configures it in
 * time, and waits for one in the background (serviced from the console's
 * input poll and writes), instead of giving up.  A USB boot sets it: there
 * a host is on the port, only perhaps late.
 */
void acm_console_wait_for_host(bool on);
/* The usbacm console's gadget is configured by a host */
bool acm_console_connected(void);
/* A download gadget, not the usbacm console's, holds the UDC */
bool g_dnl_download_active(void);
void g_dnl_set_serialnumber(char *);
void g_dnl_set_product(const char *s);

bool g_dnl_detach(void);
void g_dnl_trigger_detach(void);
void g_dnl_clear_detach(void);
int run_usb_dnl_gadget(int usbctrl_index, char *usb_dnl_gadget);
/*
 * usb_dnl_last_detached() - did the last run_usb_dnl_gadget() end with the
 * host's DETACH (or its bus reset after one), rather than a timeout, Ctrl-C or
 * the host going away?  A flash session's `usbflash bootrom` acts only then.
 */
bool usb_dnl_last_detached(void);

/* f_acm.c, USB_GADGET_DNL_ACM: an ACM console beside DFU in one configuration */
int acm_dnl_attach(struct usb_configuration *c);
/* Console bytes for that ACM console (proper's `dnlacm` stdio, the xPL tee) */
void acm_dnl_console_putc(const char c);
void acm_dnl_console_puts(const char *str);
/* xPL: keep console bytes for it from now on (needs a heap) */
int acm_dnl_console_start(void);
/* xPL: true once acm_dnl_console_start() made the ring (safe before BSS is cleared) */
bool acm_dnl_console_on(void);
/* xPL: bytes already formatted for a terminal (CR/LF as is), e.g. a boot log */
void acm_dnl_console_write(const char *s, unsigned int len);

#endif /* __G_DOWNLOAD_H_ */
