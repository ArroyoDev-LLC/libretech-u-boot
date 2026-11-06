// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Jakub Kicinski <kubakici@wp.pl>
 *
 * 'wifi' command for the MT7601U background scan. Device bring-up follows
 * mt7601u_probe() of the Linux driver
 * (drivers/net/wireless/mediatek/mt7601u/usb.c).
 */

#include <command.h>
#include <cpu_func.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <vsprintf.h>
#include <linux/errno.h>
#include <linux/string.h>

#include "mt7601u.h"
#include "mcu.h"
#include "connect.h"

static struct mt7601u_dev *mt_dev;

static void mt7601u_free(struct mt7601u_dev *dev)
{
	if (!dev)
		return;
	free(dev->vend_buf);
	free(dev->cmd_buf);
	free(dev->resp_buf);
	free(dev->resp_buf2);
	free(dev->fw_buf);
	free(dev->rx_buf);
	free(dev->rx_buf2);
	free(dev->tx_buf);
	free(dev);
}

static struct mt7601u_dev *mt7601u_alloc(void)
{
	struct mt7601u_dev *dev = calloc(1, sizeof(*dev));

	if (!dev)
		return NULL;

	dev->vend_buf = memalign(ARCH_DMA_MINALIGN, ARCH_DMA_MINALIGN);
	dev->cmd_buf = memalign(ARCH_DMA_MINALIGN, MT_CMD_BUF_SIZE);
	dev->fw_buf = memalign(ARCH_DMA_MINALIGN,
			       ALIGN(MCU_FW_URB_SIZE, ARCH_DMA_MINALIGN));
	/* naturally aligned: each is a single xHCI TRB (async bulk IN) */
	dev->resp_buf = memalign(MCU_RESP_URB_SIZE, MCU_RESP_URB_SIZE);
	dev->resp_buf2 = memalign(MCU_RESP_URB_SIZE, MCU_RESP_URB_SIZE);
	dev->rx_buf = memalign(MT_RX_URB_SIZE, MT_RX_URB_SIZE);
	dev->rx_buf2 = memalign(MT_RX_URB_SIZE, MT_RX_URB_SIZE);
	dev->tx_buf = memalign(ARCH_DMA_MINALIGN,
			       ALIGN(MT_TX_BUF_SIZE, ARCH_DMA_MINALIGN));

	if (!dev->vend_buf || !dev->cmd_buf || !dev->resp_buf ||
	    !dev->resp_buf2 || !dev->fw_buf || !dev->rx_buf ||
	    !dev->rx_buf2 || !dev->tx_buf) {
		mt7601u_free(dev);
		return NULL;
	}

	/* calloc leaves handles at 0, a valid-looking handle: mark all free */
	dev->async_rx_h = -1;
	dev->async_rx_h2 = -1;
	dev->async_resp_h = -1;
	dev->async_resp_h2 = -1;

	return dev;
}

/* mt7601u_probe() */
static int mt7601u_probe(struct mt7601u_dev *dev)
{
	int ret;

	ret = mt7601u_usb_find(dev);
	if (ret)
		return ret;

	ret = mt7601u_wait_asic_ready(dev);
	if (ret) {
		printf("mt7601u: ASIC not ready\n");
		return ret;
	}

	dev->asic_rev = mt7601u_rr(dev, MT_ASIC_VERSION);
	dev->mac_rev = mt7601u_rr(dev, MT_MAC_CSR0);
	printf("mt7601u: ASIC revision: %08x MAC revision: %08x\n",
	       dev->asic_rev, dev->mac_rev);
	if ((dev->asic_rev >> 16) != 0x7601)
		return -ENODEV;

	/* Note: vendor driver skips this check for MT7601U */
	if (!(mt7601u_rr(dev, MT_EFUSE_CTRL) & MT_EFUSE_CTRL_SEL))
		printf("mt7601u: Warning: eFUSE not present\n");

	ret = mt7601u_init_hardware(dev);
	if (ret) {
		printf("mt7601u: hardware init failed:%d\n", ret);
		return ret;
	}

	return 0;
}

static int do_wifi_init(void)
{
	struct mt7601u_dev *dev;
	int ret;

	if (mt_dev) {
		/* re-init anyway: do not wait on a possibly wedged cmd path */
		mt_dev->busy++;
		mt7601u_connect_stop(mt_dev);
		/*
		 * finish=false aborts a posted cmd-resp TD under an in-flight
		 * reply. The dongle then NAKs every vendor OUT (WLAN_FUN_CTRL
		 * -110) while vendor IN still completes. Drain the reply first.
		 */
		mt7601u_scan_stop(mt_dev, true);
		mt_dev->busy--;
		mt7601u_free(mt_dev);
		mt_dev = NULL;
	}

	dev = mt7601u_alloc();
	if (!dev)
		return CMD_RET_FAILURE;

	dev->busy++;
	ret = mt7601u_probe(dev);
	if (!ret)
		ret = mt7601u_phy_set_channel(dev, 1);
	dev->busy--;

	if (ret) {
		mt7601u_free(dev);
		return CMD_RET_FAILURE;
	}

	mt_dev = dev;

	dev->busy++;
	mt7601u_restore_rf5_0(dev);
	mt7601u_print_cal_reg(dev);
	printf("mt7601u: ready  ASIC_VERSION %08x  MAC %pM  COM_REG0 %08x  fw %x.%x.%02x  chan %u\n",
	       dev->asic_rev, dev->macaddr, mt7601u_rr(dev, MT_MCU_COM_REG0),
	       (dev->fw_ver >> 12) & 0xf, (dev->fw_ver >> 8) & 0xf,
	       dev->fw_ver & 0xf, dev->chan);
	dev->busy--;

	return CMD_RET_SUCCESS;
}

static int do_wifi(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	struct mt7601u_dev *dev;
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (!strcmp(argv[1], "init"))
		return do_wifi_init();

	dev = mt_dev;
	if (!dev) {
		printf("mt7601u: not initialised, run 'wifi init'\n");
		return CMD_RET_FAILURE;
	}

	if (dev->gone && !(argc >= 3 && !strcmp(argv[1], "scan") &&
			   !strcmp(argv[2], "show"))) {
		printf("mt7601u: the USB controller went away (usb reset?); run 'wifi init'\n");
		return CMD_RET_FAILURE;
	}

	if (!strcmp(argv[1], "diag")) {
		dev->busy++;
		mt7601u_diag(dev, "on demand");
		dev->busy--;
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "join") || !strcmp(argv[1], "status"))
		return mt7601u_connect_cmd(dev, argc - 1, argv + 1);

	if (!strcmp(argv[1], "chan") && argc >= 3) {
		int chan = dectoul(argv[2], NULL);

		if (dev->scan.running) {
			printf("mt7601u: stop the scan first\n");
			return CMD_RET_FAILURE;
		}
		dev->busy++;
		/* A live join holds the packet-RX TDs. Scan start stops it
		 * before a channel set; do the same here.
		 */
		mt7601u_connect_stop(dev);
		ret = mt7601u_phy_set_channel(dev, chan);
		dev->busy--;
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (strcmp(argv[1], "scan") || argc < 3)
		return CMD_RET_USAGE;

	if (!strcmp(argv[2], "start")) {
		u32 dwell = argc >= 4 ? dectoul(argv[3], NULL) : 0;

		dev->busy++;
		ret = mt7601u_scan_start(dev, dwell);
		dev->busy--;
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}
	if (!strcmp(argv[2], "stop")) {
		dev->busy++;
		mt7601u_scan_stop(dev, true);
		dev->busy--;
		mt7601u_scan_show(dev, false);
		return CMD_RET_SUCCESS;
	}
	if (!strcmp(argv[2], "show")) {
		bool verbose = argc >= 4 && !strcmp(argv[3], "-v");

		mt7601u_scan_show(dev, verbose);
		return CMD_RET_SUCCESS;
	}

	return CMD_RET_USAGE;
}

U_BOOT_CMD(wifi, 5, 0, do_wifi,
	   "MT7601U USB Wi-Fi background scan + station join (experimental)",
	   "init                 - find the dongle, load firmware, init RX path\n"
	   "wifi chan <1-14>         - set a 20 MHz channel (scan stopped)\n"
	   "wifi scan start [dwell]  - start the cyclic scan, ch 1-11, dwell ms/channel (300)\n"
	   "wifi scan show [-v]      - BSS table and counters (-v: per-phase detail)\n"
	   "wifi scan stop           - unregister the cyclic, print the table\n"
	   "wifi join <ssid>         - join a scanned BSS as a station\n"
	   "wifi join <ssid> <bssid> <chan> - join directly, no scan (avoids the scan RX-DMA wedge)\n"
	   "wifi status              - connection state\n"
	   "wifi diag                - dongle registers + xHCI endpoint states");
