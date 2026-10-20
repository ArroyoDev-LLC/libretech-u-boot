// SPDX-License-Identifier: GPL-2.0+
/*
 * dfu.c -- dfu command
 *
 * Copyright (C) 2015
 * Lukasz Majewski <l.majewski@majess.pl>
 *
 * Copyright (C) 2012 Samsung Electronics
 * authors: Andrzej Pietrasiewicz <andrzej.p@samsung.com>
 *	    Lukasz Majewski <l.majewski@samsung.com>
 */

#include <command.h>
#include <log.h>
#include <watchdog.h>
#include <dfu.h>
#include <console.h>
#include <g_dnl.h>
#include <usb.h>
#include <net.h>
#include <linux/printk.h>

__weak void dfu_background(void)
{
}

int run_usb_dnl_gadget(int usbctrl_index, char *usb_dnl_gadget)
{
	const char *exit_reason = "unknown";
	bool dfu_reset = false;
	struct udevice *udc;
	int ret, i = 0;

	ret = udc_device_get_by_index(usbctrl_index, &udc);
	if (ret) {
		pr_err("udc_device_get_by_index failed\n");
		return CMD_RET_FAILURE;
	}
	g_dnl_clear_detach();
	ret = g_dnl_register(usb_dnl_gadget);
	if (ret) {
		pr_err("g_dnl_register failed");
		ret = CMD_RET_FAILURE;
		goto err_detach;
	}

	/*
	 * Say so. Without this the gadget loop below is completely silent, and
	 * a running gadget, one that exited instantly, and leftover scrollback
	 * from the previous command are indistinguishable on the console.
	 */
	printf("Gadget \"%s\" up on UDC %d - waiting for host (Ctrl-C aborts)\n",
	       usb_dnl_gadget, usbctrl_index);

	while (1) {
		if (g_dnl_detach()) {
			/*
			 * Check if USB bus reset is performed after detach,
			 * which indicates that -R switch has been passed to
			 * dfu-util. In this case reboot the device
			 */
			if (dfu_usb_get_reset()) {
				dfu_reset = true;
				exit_reason = "host issued USB reset (dfu-util -R); rebooting";
				goto exit;
			}

			/*
			 * This extra number of dm_usb_gadget_handle_interrupts()
			 * calls is necessary to assure correct transmission
			 * completion with dfu-util
			 */
			if (++i == 10000) {
				exit_reason = "host detached";
				goto exit;
			}
		}

		/* A back end's work on what has arrived, between USB events */
		dfu_background();

		if (ctrlc()) {
			exit_reason = "aborted by Ctrl-C";
			goto exit;
		}

		if (dfu_get_defer_flush()) {
			/*
			 * Call to dm_usb_gadget_handle_interrupts() is necessary
			 * to act on ZLP OUT transaction from HOST PC after
			 * transmitting the whole file.
			 *
			 * If this ZLP OUT packet is NAK'ed, the HOST libusb
			 * function fails after timeout (by default it is set to
			 * 5 seconds). In such situation the dfu-util program
			 * exits with error message.
			 */
			dm_usb_gadget_handle_interrupts(udc);
			ret = dfu_flush(dfu_get_defer_flush(), NULL, 0, 0);
			dfu_set_defer_flush(NULL);
#ifdef CONFIG_DFU_TIMEOUT
			/*
			 * The flush (a storage write) is the host's work and can
			 * outlast an idle timeout: the host waits on it, it has
			 * not gone quiet. An xPL phase writes storage only with
			 * SPL_USBFLASH_WRITE; without it a flush is a RAM copy.
			 */
			if ((!IS_ENABLED(CONFIG_XPL_BUILD) ||
			     IS_ENABLED(CONFIG_SPL_USBFLASH_WRITE)) && dfu_get_timeout())
				dfu_host_active();
#endif
			if (ret) {
				pr_err("Deferred dfu_flush() failed!");
				exit_reason = "deferred flush failed";
				goto exit;
			}
		}

#ifdef CONFIG_DFU_TIMEOUT
		unsigned long wait_time = dfu_get_timeout();

		if (wait_time) {
			unsigned long current_time = dfu_timeout_elapsed();

			if (current_time > wait_time) {
				exit_reason = "inactivity timeout";
				printf("DFU: inactivity timeout after %lu ms\n",
				       wait_time);
				goto exit;
			}
		}
#endif

		/*
		 * f_dfu drops the timeout to 0 on the first request, so a host that
		 * leaves without DETACH used to strand this loop until Ctrl-C.  A
		 * board cable check (weak default: -EOPNOTSUPP, i.e. "connected")
		 * that answers 0 ends the session.
		 */
		if (!g_dnl_board_usb_cable_connected()) {
			exit_reason = "host gone";
			goto exit;
		}

		if (dfu_reinit_needed) {
			exit_reason = "re-init requested";
			goto exit;
		}

		schedule();
		dm_usb_gadget_handle_interrupts(udc);
	}
exit:
	printf("Gadget \"%s\" stopped: %s\n", usb_dnl_gadget, exit_reason);
	g_dnl_release(udc);
	goto out;
err_detach:
	udc_device_put(udc);
out:

	if (dfu_reset)
		do_reset(NULL, 0, 0, NULL);

	g_dnl_clear_detach();

	return ret;
}
