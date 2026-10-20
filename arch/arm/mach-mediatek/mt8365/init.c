// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 MediaTek Inc.
 * Copyright (C) 2023 BayLibre, SAS
 * Author: Julien Masson <jmasson@baylibre.com>
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <fdtdec.h>
#include <init.h>
#include <net.h>
#include <usb.h>
#include <asm/global_data.h>
#include <asm/system.h>
#include <dm/uclass.h>
#include <linux/sizes.h>
#include <wdt.h>

DECLARE_GLOBAL_DATA_PTR;

int dram_init(void)
{
	int ret;

	ret = fdtdec_setup_mem_size_base();
	if (ret)
		return ret;

	/*
	 * The control DT (mt8365-evk) hard-codes /memory = 3 GiB, but the
	 * actual DRAM varies per board (the g350 Pumpkin has 2 GiB). Trusting
	 * the DT made u-boot try to use absent memory ("DRAM: 3 GiB (total 0
	 * Bytes)" -> hang). Probe the real size so one DT works across SKUs --
	 * matches the mt7981/mt7986/mt7988 autodetect. get_ram_size() detects
	 * power-of-2 sizes, so the 2 GiB Pumpkin is exact; a 3 GiB EVK caps at
	 * 2 GiB (safe under-report).
	 */
	gd->ram_size = get_ram_size((void *)gd->ram_base, SZ_2G);

	return 0;
}

int board_init(void)
{
	/*
	 * Bring up the USB host bus so the on-board Microchip LAN9512
	 * (USB 2.0 hub + 10/100 Ethernet, driven by drivers/usb/eth/smsc95xx.c)
	 * on the RJ45 is enumerated and registered as a DM Ethernet device,
	 * giving u-boot DHCP/tftp over the wire. The LAN9512 sits behind the
	 * MT8365 SSUSB xhci host child (usb@11200000, "mediatek,mtk-xhci"),
	 * a separate role from the mtu3 gadget download port used for DFU.
	 *
	 * usb_init() walks the host controllers and probes each device; the
	 * smsc95xx U_BOOT_USB_DEVICE table (VID 0x0424 PID 0xec00) auto-binds
	 * the LAN9512 to UCLASS_ETH. A failure here is non-fatal: leave it to
	 * a later `usb start` rather than blocking boot.
	 */
	if (CONFIG_IS_ENABLED(USB_HOST_ETHER) && CONFIG_IS_ENABLED(DM_USB))
		usb_init();

	/*
	 * Gadget-side USB ethernet (RNDIS/CDC) is a distinct feature from the
	 * LAN9512 host adapter above; bind it only when explicitly enabled
	 * (matches the vendor board_init). With our host-ethernet defconfig
	 * CONFIG_USB_ETHER is unset, so this compiles out.
	 */
	if (CONFIG_IS_ENABLED(USB_ETHER))
		usb_ether_init();

	return 0;
}

void reset_cpu(void)
{
	struct udevice *wdt;

	if (IS_ENABLED(CONFIG_PSCI_RESET)) {
		psci_system_reset();
	} else {
		uclass_first_device(UCLASS_WDT, &wdt);
		if (wdt)
			wdt_expire_now(wdt, 0);
	}
}

int print_cpuinfo(void)
{
	printf("CPU:   MediaTek MT8365\n");
	return 0;
}
