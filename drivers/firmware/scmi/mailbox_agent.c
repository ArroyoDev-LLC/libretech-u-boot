// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Linaro Limited.
 */

#define LOG_CATEGORY UCLASS_SCMI_AGENT

#include <dm.h>
#include <errno.h>
#include <mailbox.h>
#include <scmi_agent.h>
#include <scmi_agent-uclass.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <linux/compat.h>
#include <asm/io.h>
#include <stdio.h>

#include "smt.h"

#define TIMEOUT_30MS			30

/**
 * struct scmi_mbox_channel - Description of an SCMI mailbox transport
 * @smt:	Shared memory buffer
 * @mbox:	Mailbox channel description
 * @timeout_us:	Timeout in microseconds for the mailbox transfer
 */
struct scmi_mbox_channel {
	struct scmi_smt smt;
	struct mbox_chan mbox;
	ulong timeout_us;
};

/**
 * struct scmi_channel - Channel instance referenced in SCMI drivers
 * @ref: Reference to local channel instance
 **/
struct scmi_channel {
	struct scmi_mbox_channel ref;
};

static int scmi_mbox_process_msg(struct udevice *dev,
				 struct scmi_channel *channel,
				 struct scmi_msg *msg)
{
	struct scmi_mbox_channel *chan = &channel->ref;
	int ret;

	ret = scmi_write_msg_to_smt(dev, &chan->smt, msg);
	if (ret)
		return ret;

	/* EL3 is out of the path: the M0 reads the shmem uncached, so
	 * push the request out of D-cache before ringing the doorbell.
	 */
	scmi_smt_flush(&chan->smt);

	/* Give shm addr to mbox in case it is meaningful */
	ret = mbox_send(&chan->mbox, chan->smt.buf);
	if (ret) {
		dev_err(dev, "Message send failed: %d\n", ret);
		goto out;
	}

	/* Receive the response */
	ret = mbox_recv(&chan->mbox, chan->smt.buf, chan->timeout_us);
	if (ret) {
		/*
		 * printf first: gap §8.12 schb7 showed zero SCMI lines despite
		 * CONFIG+DT+strings — beat LOGLEVEL / UART races with chip_id.
		 */
		printf("SCMI: Response failed: %d, abort\n", ret);
		dev_err(dev, "Response failed: %d, abort\n", ret);
		/*
		 * Residual soft-fail path: dump HP MHU STAT + SMT header so a
		 * mute SC still yields doorbell/shmem evidence without =>.
		 * SM1 HP: AP TX 0xFF63C440 / RX 0xFF63C41C (TX = RX + 0x24).
		 */
		if (IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG)) {
			u32 *smt = (u32 *)chan->smt.buf;
			u32 *hb = (u32 *)0xFFFE7400; /* MHU_HIGH_SCP_TO_AP */

			/* dev_err: LOGLEVEL=4 suppresses LOGL_WARNING on this board */
			printf("SCMI soft-fail MHU/SMT dump: TXSET=%08x TXSTAT=%08x RXSET=%08x RXSTAT=%08x\n",
			       readl(0xFF63C440), readl(0xFF63C444),
			       readl(0xFF63C41C), readl(0xFF63C420));
			dev_err(dev,
				"SCMI soft-fail MHU/SMT dump: TXSET=%08x TXSTAT=%08x RXSET=%08x RXSTAT=%08x\n",
				readl(0xFF63C440), readl(0xFF63C444),
				readl(0xFF63C41C), readl(0xFF63C420));
			if (smt) {
				printf("SCMI SMT[0..6]=%08x %08x %08x %08x %08x %08x %08x\n",
				       smt[0], smt[1], smt[2], smt[3],
				       smt[4], smt[5], smt[6]);
				dev_err(dev,
					"SMT[0..6]=%08x %08x %08x %08x %08x %08x %08x\n",
					smt[0], smt[1], smt[2], smt[3],
					smt[4], smt[5], smt[6]);
			}
			/*
			 * SC heartbeat (gap §8.10): magic/STAT/IRQEN/tick at
			 * 0xFFFE7400. Also AP-view IRQEN @ SYS_CPU_CFG10.
			 */
			printf("SCMI SC-HB[0..3]=%08x %08x %08x %08x IRQEN=%08x\n",
			       hb[0], hb[1], hb[2], hb[3], readl(0xFF634670));
			dev_err(dev,
				"SC-HB[0..3]=%08x %08x %08x %08x IRQEN=%08x\n",
				hb[0], hb[1], hb[2], hb[3],
				readl(0xFF634670));
		}
		goto out;
	}

	/* Invalidate before reading the reply the M0 wrote uncached. */
	scmi_smt_inval(&chan->smt);
	ret = scmi_read_resp_from_smt(dev, &chan->smt, msg);

out:
	scmi_clear_smt_channel(&chan->smt);

	return ret;
}

static int setup_channel(struct udevice *dev, struct scmi_mbox_channel *chan)
{
	struct scmi_mbox_channel *base_chan = dev_get_plat(dev);
	int ret;

	ret = mbox_get_by_index(dev, 0, &chan->mbox);
	if (ret) {
		dev_err(dev, "Failed to find mailbox: %d\n", ret);
		return ret;
	}

	ret = scmi_dt_get_smt_buffer(dev, &chan->smt);
	if (ret) {
		dev_err(dev, "Failed to get shm resources: %d\n", ret);
		return ret;
	}

	chan->timeout_us = base_chan->timeout_us;

	return 0;
}

static int scmi_mbox_get_channel(struct udevice *dev,
				 struct udevice *protocol,
				 struct scmi_channel **channel)
{
	struct scmi_mbox_channel *base_chan = dev_get_plat(dev);
	struct scmi_mbox_channel *chan;
	int ret;

	if (!dev_has_ofnode(protocol) || !dev_read_prop(protocol, "shmem", NULL)) {
		/* Uses agent base channel */
		*channel = container_of(base_chan, struct scmi_channel, ref);

		return 0;
	}

	chan = calloc(1, sizeof(*chan));
	if (!chan)
		return -ENOMEM;

	/* Setup a dedicated channel for the protocol */
	ret = setup_channel(protocol, chan);
	if (ret) {
		free(chan);
		return ret;
	}

	*channel = (void *)chan;

	return 0;
}

int scmi_mbox_of_to_plat(struct udevice *dev)
{
	struct scmi_mbox_channel *chan = dev_get_plat(dev);

	chan->timeout_us = dev_read_u32_default(dev, "arm,max-rx-timeout-ms",
						TIMEOUT_30MS) * 1000;

	return setup_channel(dev, chan);
}

static const struct udevice_id scmi_mbox_ids[] = {
	{ .compatible = "arm,scmi" },
	{ }
};

static const struct scmi_agent_ops scmi_mbox_ops = {
	.of_get_channel = scmi_mbox_get_channel,
	.process_msg = scmi_mbox_process_msg,
};

U_BOOT_DRIVER(scmi_mbox) = {
	.name		= "scmi-over-mailbox",
	.id		= UCLASS_SCMI_AGENT,
	.of_match	= scmi_mbox_ids,
	.plat_auto	= sizeof(struct scmi_mbox_channel),
	.of_to_plat	= scmi_mbox_of_to_plat,
	.ops		= &scmi_mbox_ops,
};
