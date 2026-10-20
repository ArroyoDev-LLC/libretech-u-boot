// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2014, Xilinx, Inc
 *
 * USB Low level initialization(Specific to zynq)
 */

#include <dm.h>
#include <time.h>
#include <usb.h>
#include <linux/delay.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>
#include <asm/io.h>
#include <usb/ehci-ci.h>
#include <usb/ulpi.h>

#include "ehci.h"

struct zynq_ehci_priv {
	struct ehci_ctrl ehcictrl;
	struct usb_ehci *ehci;
};

/*
 * Remembered so the peripheral-mode transceiver setup can be re-applied from
 * ci_init_after_reset(), which has no udevice to work from. Only one USB
 * controller is usable on Zynq-7000 in practice, and ci_udc itself keeps a
 * single global controller, so a single slot matches the surrounding design.
 *
 * Captured on EVERY probe, host or peripheral -- see ci_init_after_reset().
 */
static struct ulpi_viewport zynq_ulpi_vp;

/*
 * Peripheral-mode transceiver setup.
 *
 * TERMSELECT enables the full-speed termination -- the 1.5k D+ pull-up -- which
 * is the only way a device signals attach. The pulldowns and both DRVVBUS bits
 * are host behaviour and must be off. EXTVBUSIND must be clear so the PHY uses
 * its INTERNAL VBUS comparators; with it set the PHY takes VBUS state from an
 * external indicator pin, and a board that does not wire one never asserts
 * session-valid.
 */
static void zynq_ulpi_init_device(void)
{
	struct ulpi_regs *ulpi = (struct ulpi_regs *)0;

	ulpi_write(&zynq_ulpi_vp, &ulpi->function_ctrl,
		   ULPI_FC_FULL_SPEED | ULPI_FC_TERMSELECT |
		   ULPI_FC_OPMODE_NORMAL | ULPI_FC_SUSPENDM);
	ulpi_write(&zynq_ulpi_vp, &ulpi->iface_ctrl, 0);
	ulpi_write(&zynq_ulpi_vp, &ulpi->otg_ctrl, 0);
	ulpi_write(&zynq_ulpi_vp, &ulpi->otg_ctrl_clear,
		   ULPI_OTG_DP_PULLDOWN | ULPI_OTG_DM_PULLDOWN |
		   ULPI_OTG_DRVVBUS | ULPI_OTG_DRVVBUS_EXT |
		   ULPI_OTG_EXTVBUSIND);
}

/*
 * ci_udc's ci_pullup() resets the controller (USBCMD_RST) before enabling the
 * pull-up, and that reset takes the ULPI transceiver's configuration with it --
 * measured on an ANTSDR E310: probe writes OTG_CONTROL = 0x00, yet with the
 * gadget running the PHY reads back 0x07, i.e. IdPullup plus BOTH D+/D-
 * pulldowns restored. Host-style pulldowns hold the data lines down, so the
 * pull-up can never signal attach and the gadget stays invisible.
 *
 * ci_pullup() calls this hook immediately after that reset for exactly this
 * purpose, so re-apply the peripheral setup here.
 *
 * Deliberately NOT conditional on the last probe having been USB_INIT_DEVICE:
 * ci_pullup() is reached only from the gadget path, so by construction a
 * peripheral is coming up whenever this runs, and the last probe's role says
 * nothing useful.
 *
 * KNOWN LIMITATION -- `usb start` followed by a gadget in the SAME boot does
 * not fully recover. Instrumented on an ANTSDR E310, the sequence is not what
 * you would guess: the device re-probe DOES happen and this hook DOES run --
 *
 *     ehci_zynq_probe init_type=0        <- usb start
 *     ehci_zynq_probe init_type=1        <- gadget: re-probed as device
 *     ulpi_init_device                   <- peripheral setup applied
 *     ci_init_after_reset                <- and applied again here
 *
 * That is necessary but not sufficient after a preceding `usb start` -- see
 * ci_init_after_run() below, which handles what comes after.
 */
void ci_init_after_reset(struct ehci_ctrl *ctrl)
{
	/*
	 * Drop port power before touching the transceiver. A preceding
	 * `usb start` leaves PORTSC.PP set, and the controller then keeps
	 * driving VBUS through the PHY, which re-asserts ULPI DrvVbus and
	 * overrides everything zynq_ulpi_init_device() writes below.
	 */
	if (ctrl && ctrl->hcor)
		clrbits_le32(&ctrl->hcor->or_portsc[0], EHCI_PS_PP);

	if (zynq_ulpi_vp.viewport_addr)
		zynq_ulpi_init_device();
}

/* ChipIdea OTGSC, relative to the EHCI operational (hcor) base. */
#define ZYNQ_OTGSC_OFF		0x64		/* hcor + 0x64 == base + 0x1a4 */
#define ZYNQ_OTGSC_ASV		(1 << 10)	/* A-session valid */
#define ZYNQ_OTGSC_BSV		(1 << 11)	/* B-session valid */
/*
 * "A session is still up." Both bits are checked because both are set by the
 * stale session host mode leaves behind and both clear together when it
 * collapses -- measured entering ci_pullup(): OTGSC 0x002d2d20 after
 * `usb start` (ASV and BSV set) versus 0x00211120 on a device-only start
 * (neither set, BSE set instead).
 */
#define ZYNQ_OTGSC_SESSION	(ZYNQ_OTGSC_ASV | ZYNQ_OTGSC_BSV)
#define ZYNQ_SESSION_END_MS	2500		/* cap on waiting for BSV=0 */
#define ZYNQ_SESSION_STABLE_MS	100		/* BSV must stay 0 this long */
#define ZYNQ_OTG_SETTLE_MS	50

/*
 * Second half of the peripheral transceiver setup, called from ci_pullup()
 * AFTER USBCMD.RUN is set.
 *
 * Once the core is running it owns ULPI OTG_CONTROL and drives it from its own
 * OTG state machine; viewport writes made before that point do not survive.
 * Measured per-step on an ANTSDR E310, everything up to and including the
 * epflush write leaves the PHY correct (OTG_CONTROL 0x00, FUNCTION_CTRL 0x45,
 * DEBUG 0x01/J) in both the device-only and the host-then-device case. The RUN
 * write is where they diverge:
 *
 *   device-only     RUN -> OTG_CONTROL 0x00, unchanged, stable
 *   after `usb start`  RUN -> OTG_CONTROL 0x05 at once, FUNCTION_CTRL 0x55
 *                          (OpMode non-driving) 50 ms later, then 0x07 / SE0
 *                          about 1 s later
 *
 * The trigger for the divergence is the VBUS session left behind by host mode:
 * entering ci_pullup() OTGSC reads BSV|BSE set after `usb start` and clear on a
 * device-only start. Re-applying the setup while that stale session is still up
 * holds for a few hundred ms and is then undone when the session collapses --
 * that collapse is what puts the PHY at 0x07. Re-applying once the session has
 * ended sticks indefinitely.
 *
 * So: wait for the stale session to end, then write the peripheral setup again.
 * On a clean device-only start BSV is already clear and this costs one stable
 * window plus one settle.
 */
void ci_init_after_run(struct ehci_ctrl *ctrl)
{
	struct ulpi_regs *ulpi = (struct ulpi_regs *)0;
	int i;

	if (!zynq_ulpi_vp.viewport_addr)
		return;

	if (ctrl && ctrl->hcor) {
		void *otgsc = (void *)((uintptr_t)ctrl->hcor + ZYNQ_OTGSC_OFF);
		ulong start = get_timer(0);
		ulong clear = 0;
		bool clearing = false;

		while (get_timer(start) < ZYNQ_SESSION_END_MS) {
			if (readl(otgsc) & ZYNQ_OTGSC_SESSION) {
				clearing = false;
			} else if (!clearing) {
				clearing = true;
				clear = get_timer(0);
			} else if (get_timer(clear) >= ZYNQ_SESSION_STABLE_MS) {
				break;
			}
			udelay(1000);
		}
	}

	for (i = 0; i < 3; i++) {
		zynq_ulpi_init_device();
		mdelay(ZYNQ_OTG_SETTLE_MS);
		if (!ulpi_read(&zynq_ulpi_vp, &ulpi->otg_ctrl))
			return;
	}

	printf("zynq usb: ULPI OTG_CONTROL stuck at 0x%02x\n",
	       ulpi_read(&zynq_ulpi_vp, &ulpi->otg_ctrl));
}

static int ehci_zynq_of_to_plat(struct udevice *dev)
{
	struct zynq_ehci_priv *priv = dev_get_priv(dev);

	priv->ehci = dev_read_addr_ptr(dev);
	if (!priv->ehci)
		return -EINVAL;

	return 0;
}

static int ehci_zynq_probe(struct udevice *dev)
{
	struct usb_plat *plat = dev_get_plat(dev);
	struct zynq_ehci_priv *priv = dev_get_priv(dev);
	struct ehci_hccr *hccr;
	struct ehci_hcor *hcor;
	struct ulpi_viewport ulpi_vp;
	/* Used for writing the ULPI data address */
	struct ulpi_regs *ulpi = (struct ulpi_regs *)0;
	int ret;

	hccr = (struct ehci_hccr *)((uint32_t)&priv->ehci->caplength);
	hcor = (struct ehci_hcor *)((uint32_t) hccr +
			HC_LENGTH(ehci_readl(&hccr->cr_capbase)));

	ulpi_vp.viewport_addr = (u32)&priv->ehci->ulpi_viewpoint;
	ulpi_vp.port_num = 0;

	ret = ulpi_init(&ulpi_vp);
	if (ret) {
		puts("zynq ULPI viewport init failed\n");
		return -1;
	}

	/*
	 * The ULPI transceiver has to be programmed for the role this
	 * controller is being registered in. Before this branch existed the
	 * host settings below were applied unconditionally, which silently
	 * broke peripheral mode: a gadget would come up (dfu/ums/fastboot all
	 * registered without error) but never enumerate, because the PHY was
	 * still wired as a host -- pulldowns on, VBUS driven, and no
	 * TERMSELECT to present the device pull-up.
	 *
	 * The peripheral half lives in zynq_ulpi_init_device() rather than
	 * inline, because ci_pullup()'s controller reset undoes it and it has
	 * to be re-applied from ci_init_after_reset().
	 */
	/*
	 * Captured for both roles: ci_init_after_reset() needs the viewport
	 * even when the last probe was a host probe (see there).
	 */
	zynq_ulpi_vp = ulpi_vp;

	if (plat->init_type == USB_INIT_DEVICE) {
		zynq_ulpi_init_device();
	} else {
		ulpi_write(&ulpi_vp, &ulpi->function_ctrl,
			   ULPI_FC_FULL_SPEED | ULPI_FC_OPMODE_NORMAL |
			   ULPI_FC_SUSPENDM);
		ulpi_write(&ulpi_vp, &ulpi->iface_ctrl, 0);

		ulpi_write(&ulpi_vp, &ulpi->otg_ctrl,
			   ULPI_OTG_DP_PULLDOWN | ULPI_OTG_DM_PULLDOWN |
			   ULPI_OTG_EXTVBUSIND);

		/* Set VBus */
		ulpi_write(&ulpi_vp, &ulpi->otg_ctrl_set,
			   ULPI_OTG_DRVVBUS | ULPI_OTG_DRVVBUS_EXT);
	}

	return ehci_register(dev, hccr, hcor, NULL, 0, plat->init_type);
}

static const struct udevice_id ehci_zynq_ids[] = {
	{ .compatible = "xlnx,zynq-usb-2.20a" },
	{ }
};

U_BOOT_DRIVER(ehci_zynq) = {
	.name	= "ehci_zynq",
	.id	= UCLASS_USB,
	.of_match = ehci_zynq_ids,
	.of_to_plat = ehci_zynq_of_to_plat,
	.probe = ehci_zynq_probe,
	.remove = ehci_deregister,
	.ops	= &ehci_usb_ops,
	.plat_auto	= sizeof(struct usb_plat),
	.priv_auto	= sizeof(struct zynq_ehci_priv),
	.flags	= DM_FLAG_ALLOC_PRIV_DMA,
};
