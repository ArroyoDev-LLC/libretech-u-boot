// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic Meson GXBB/GXL/G12 MHU doorbell mailbox
 *
 * Port of Linux drivers/mailbox/platform_mhu.c for U-Boot UCLASS_MAILBOX.
 * Compatible: "amlogic,meson-gxbb-mhu" (#mbox-cells = <1>, channel 0..2).
 *
 * Doorbell-only: payload lives in client-owned shared memory (e.g. SCMI SMT).
 * Channel layout (reg base = SET_0, e.g. 0xC883C404 / 0xFF63C404):
 *   ch0 SEC @ +0x00, ch1 LP @ +0x0c, ch2 HP @ +0x18
 *   each: SET/+0 STAT/+4 CLR/+8; TX view at RX + 0x24
 *
 * Copyright (C) 2016 BayLibre SAS (Linux platform_mhu)
 * Copyright (C) 2026 Libre Computer
 */

#define LOG_CATEGORY UCLASS_MAILBOX

#include <dm.h>
#include <errno.h>
#include <log.h>
#include <mailbox-uclass.h>
#include <time.h>
#include <asm/io.h>
#include <linux/delay.h>

#define INTR_SET_OFS	0x0
#define INTR_STAT_OFS	0x4
#define INTR_CLR_OFS	0x8

#define MHU_SEC_OFFSET	0x0
#define MHU_LP_OFFSET	0xc
#define MHU_HP_OFFSET	0x18
#define TX_REG_OFFSET	0x24

#define MHU_CHANS	3

/* Bound for TX-ack poll when remote never clears (no open SC yet). */
#define MESON_MHU_TX_TIMEOUT_US	30000

struct meson_mhu_link {
	void __iomem *tx_reg;
	void __iomem *rx_reg;
};

struct meson_mhu {
	void __iomem *base;
	struct meson_mhu_link link[MHU_CHANS];
};

static int meson_mhu_request(struct mbox_chan *chan)
{
	if (chan->id >= MHU_CHANS)
		return -EINVAL;

	return 0;
}

static int meson_mhu_free(struct mbox_chan *chan)
{
	return 0;
}

static int meson_mhu_send(struct mbox_chan *chan, const void *data)
{
	struct meson_mhu *mhu = dev_get_priv(chan->dev);
	void __iomem *tx = mhu->link[chan->id].tx_reg;
	ulong start;

	/* data is client context (e.g. SCMI SMT ptr); doorbell ignores it */
	(void)data;

	start = timer_get_us();
	while (readl(tx + INTR_STAT_OFS)) {
		if ((timer_get_us() - start) >= MESON_MHU_TX_TIMEOUT_US)
			return -ETIMEDOUT;
		udelay(1);
	}

	/* Ring remote; any non-zero SET value wakes the link */
	writel(1, tx + INTR_SET_OFS);

	return 0;
}

static int meson_mhu_recv(struct mbox_chan *chan, void *data)
{
	struct meson_mhu *mhu = dev_get_priv(chan->dev);
	void __iomem *rx = mhu->link[chan->id].rx_reg;
	u32 val;

	(void)data;

	val = readl(rx + INTR_STAT_OFS);
	if (!val)
		return -ENODATA;

	writel(val, rx + INTR_CLR_OFS);

	return 0;
}

static int meson_mhu_probe(struct udevice *dev)
{
	struct meson_mhu *mhu = dev_get_priv(dev);
	static const int chan_off[MHU_CHANS] = {
		MHU_SEC_OFFSET, MHU_LP_OFFSET, MHU_HP_OFFSET
	};
	int i;

	mhu->base = dev_read_addr_ptr(dev);
	if (!mhu->base)
		return -EINVAL;

	for (i = 0; i < MHU_CHANS; i++) {
		mhu->link[i].rx_reg = mhu->base + chan_off[i];
		mhu->link[i].tx_reg = mhu->link[i].rx_reg + TX_REG_OFFSET;
		/* Clear any stale TX/RX status left by prior firmware */
		writel(readl(mhu->link[i].tx_reg + INTR_STAT_OFS),
		       mhu->link[i].tx_reg + INTR_CLR_OFS);
		writel(readl(mhu->link[i].rx_reg + INTR_STAT_OFS),
		       mhu->link[i].rx_reg + INTR_CLR_OFS);
	}

	return 0;
}

static const struct udevice_id meson_mhu_ids[] = {
	{ .compatible = "amlogic,meson-gxbb-mhu" },
	{ }
};

static struct mbox_ops meson_mhu_ops = {
	.request = meson_mhu_request,
	.rfree = meson_mhu_free,
	.send = meson_mhu_send,
	.recv = meson_mhu_recv,
};

U_BOOT_DRIVER(meson_mhu) = {
	.name = "meson-mhu",
	.id = UCLASS_MAILBOX,
	.of_match = meson_mhu_ids,
	.probe = meson_mhu_probe,
	.priv_auto = sizeof(struct meson_mhu),
	.ops = &meson_mhu_ops,
};
