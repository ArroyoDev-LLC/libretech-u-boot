// SPDX-License-Identifier: GPL-2.0+
/*
 * Rockchip mailbox doorbell driver (U-Boot), for the RK3399 pmum0 SCMI server.
 *
 * The mailbox carries no data -- the SCMI message lives in the arm,scmi-shmem
 * DRAM page. send() just rings the A2B doorbell (a CMD-then-DAT write pair,
 * TRM 15.3.2, which raises the MCU interrupt on the M0), and recv() polls the
 * B2A status the resident server sets when its reply is in the shmem. So this
 * is the doorbell half of the open-SC SCMI path: SCMI_AGENT_MAILBOX rings this
 * instead of the RK_SIP_SCMI_AGENT0 SMC, and EL3 is out of the message path.
 *
 * The reply's DRAM ordering is the server's job (it barriers + reads back the
 * SMT page before ringing B2A), so a B2A ring means the reply is in place.
 */

#include <clk.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <mailbox-uclass.h>
#include <asm/io.h>

/* TRM 15.4: A2B is A-core -> MCU (send), B2A is MCU -> A-core (reply). */
#define MB_A2B_INTEN		0x00
#define MB_A2B_STATUS		0x04
#define MB_A2B_CMD(n)		(0x08 + (n) * 8)
#define MB_A2B_DAT(n)		(0x0c + (n) * 8)
#define MB_B2A_INTEN		0x28
#define MB_B2A_STATUS		0x2c

#define MB_NCHANS		4

struct rockchip_mbox {
	void __iomem *base;
	struct clk pclk;
};

static int rockchip_mbox_send(struct mbox_chan *chan, const void *data)
{
	struct rockchip_mbox *mb = dev_get_priv(chan->dev);

	if (chan->id >= MB_NCHANS)
		return -EINVAL;

	/*
	 * Ring the doorbell. The data pointer is the shmem address the SCMI
	 * mailbox agent already wrote and flushed; the mailbox itself carries
	 * nothing, so any value triggers the interrupt on the CMD-then-DAT
	 * write pair.
	 */
	writel(0, mb->base + MB_A2B_CMD(chan->id));
	writel(0, mb->base + MB_A2B_DAT(chan->id));

	return 0;
}

static int rockchip_mbox_recv(struct mbox_chan *chan, void *data)
{
	struct rockchip_mbox *mb = dev_get_priv(chan->dev);
	u32 status;

	if (chan->id >= MB_NCHANS)
		return -EINVAL;

	status = readl(mb->base + MB_B2A_STATUS);
	if (!(status & BIT(chan->id)))
		return -ENODATA;

	/* Consume the reply doorbell (write-1-to-clear). */
	writel(BIT(chan->id), mb->base + MB_B2A_STATUS);

	return 0;
}

static int rockchip_mbox_request(struct mbox_chan *chan)
{
	struct rockchip_mbox *mb = dev_get_priv(chan->dev);

	if (chan->id >= MB_NCHANS)
		return -EINVAL;

	/*
	 * Enable this channel both ways: A2B so the ring reaches the MCU, B2A
	 * so its reply sets the status this driver polls (the TRM gates the
	 * status on INTEN). Idempotent -- EL3 also arms them for the SMC path.
	 */
	setbits_le32(mb->base + MB_A2B_INTEN, BIT(chan->id));
	setbits_le32(mb->base + MB_B2A_INTEN, BIT(chan->id));

	return 0;
}

static int rockchip_mbox_rfree(struct mbox_chan *chan)
{
	return 0;
}

static int rockchip_mbox_probe(struct udevice *dev)
{
	struct rockchip_mbox *mb = dev_get_priv(dev);

	mb->base = dev_read_addr_ptr(dev);
	if (!mb->base)
		return -EINVAL;

	/*
	 * pclk is optional here: EL3 ungates pclk_mailbox_pmu at soc init and
	 * U-Boot does not gate unused clocks, so a tree without a usable clock
	 * phandle still works. Enable it when it is describable.
	 */
	if (!clk_get_by_index(dev, 0, &mb->pclk))
		clk_enable(&mb->pclk);

	return 0;
}

static const struct mbox_ops rockchip_mbox_ops = {
	.request	= rockchip_mbox_request,
	.rfree		= rockchip_mbox_rfree,
	.send		= rockchip_mbox_send,
	.recv		= rockchip_mbox_recv,
};

static const struct udevice_id rockchip_mbox_ids[] = {
	{ .compatible = "rockchip,rk3368-mailbox" },
	{ .compatible = "rockchip,rk3399-mailbox" },
	{ }
};

U_BOOT_DRIVER(rockchip_mbox) = {
	.name		= "rockchip_mbox",
	.id		= UCLASS_MAILBOX,
	.of_match	= rockchip_mbox_ids,
	.probe		= rockchip_mbox_probe,
	.priv_auto	= sizeof(struct rockchip_mbox),
	.ops		= &rockchip_mbox_ops,
};
