// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2025 Da Xue <da@libre.computer>
 *
 * Allwinner Crypto Engine PRNG driver for H3/H5 (sun8i-ce)
 *
 * The H3/H5 crypto engine uses DMA task descriptors for all operations
 * including PRNG. We build a task descriptor in memory, point the CE
 * at it, trigger it, and poll for completion.
 *
 * Based on Linux sun8i-ce-prng.c and sun8i-ce-core.c.
 */

#include <clk.h>
#include <dm.h>
#include <reset.h>
#include <rng.h>
#include <spl.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <cpu_func.h>
#include <memalign.h>

/* CE registers */
#define CE_TDQ		0x00	/* Task Descriptor Queue address */
#define CE_ICR		0x08	/* Interrupt Control Register */
#define CE_ISR		0x0C	/* Interrupt Status Register */
#define CE_TLR		0x10	/* Task Load Register */
#define CE_TSR		0x14	/* Task Status Register */
#define CE_ESR		0x18	/* Error Status Register */

/* Scatter-gather entry */
struct ce_sginfo {
	__le32 addr;
	__le32 len;
} __packed;

#define MAX_SG	8

/* CE task descriptor -- must match hardware layout exactly */
struct ce_task {
	__le32 t_id;
	__le32 t_common_ctl;
	__le32 t_sym_ctl;
	__le32 t_asym_ctl;
	__le32 t_key;
	__le32 t_iv;
	__le32 t_ctr;
	__le32 t_dlen;
	struct ce_sginfo t_src[MAX_SG];
	struct ce_sginfo t_dst[MAX_SG];
	__le32 t_next;
	__le32 t_reserved[3];
} __packed __aligned(8);

/* Algorithm IDs */
#define CE_ALG_PRNG	49

/* Common control bits */
#define CE_COMM_INT	BIT(31)

/* Symmetric control bits */
#define PRNG_LD		BIT(17)

/* PRNG sizes */
#define PRNG_SEED_SIZE	24	/* 192 bits = 6 words */
#define PRNG_DATA_SIZE	20	/* 160 bits = 5 words */

#define CE_FLOW_PRNG	3	/* use flow 3 for RNG per Linux driver */

struct sunxi_rng_plat {
	void __iomem *base;
	struct clk clk_bus;
	struct clk clk_mod;
	struct reset_ctl reset;
	u8 seed[PRNG_SEED_SIZE];
	bool seeded;
};

static int sunxi_rng_generate(struct sunxi_rng_plat *pdata, void *dst, size_t len)
{
	ALLOC_CACHE_ALIGN_BUFFER(struct ce_task, task, 1);
	ALLOC_CACHE_ALIGN_BUFFER(u8, seed_buf, PRNG_SEED_SIZE);
	ALLOC_CACHE_ALIGN_BUFFER(u8, out_buf, PRNG_DATA_SIZE);
	u8 *output = dst;
	int timeout;
	u32 v;

	while (len > 0) {
		size_t chunk = min(len, (size_t)PRNG_DATA_SIZE);

		memset(task, 0, sizeof(*task));
		memcpy(seed_buf, pdata->seed, PRNG_SEED_SIZE);
		memset(out_buf, 0, PRNG_DATA_SIZE);

		/* Build task descriptor */
		task->t_id = cpu_to_le32(CE_FLOW_PRNG);
		task->t_common_ctl = cpu_to_le32(CE_ALG_PRNG | CE_COMM_INT);
		task->t_sym_ctl = cpu_to_le32(PRNG_LD);
		task->t_dlen = cpu_to_le32(PRNG_DATA_SIZE / 4);
		task->t_key = cpu_to_le32(virt_to_phys(seed_buf));
		task->t_iv = cpu_to_le32(virt_to_phys(seed_buf));
		task->t_dst[0].addr = cpu_to_le32(virt_to_phys(out_buf));
		task->t_dst[0].len = cpu_to_le32(PRNG_DATA_SIZE / 4);
		task->t_next = 0;

		/* Flush caches */
		flush_dcache_range((ulong)task,
				   (ulong)task + roundup(sizeof(*task), ARCH_DMA_MINALIGN));
		flush_dcache_range((ulong)seed_buf,
				   (ulong)seed_buf + roundup(PRNG_SEED_SIZE, ARCH_DMA_MINALIGN));
		flush_dcache_range((ulong)out_buf,
				   (ulong)out_buf + roundup(PRNG_DATA_SIZE, ARCH_DMA_MINALIGN));

		/* Clear interrupt status */
		writel(0xf, pdata->base + CE_ISR);

		/* Enable interrupt for flow */
		writel(BIT(CE_FLOW_PRNG), pdata->base + CE_ICR);

		/* Write task descriptor address */
		writel(virt_to_phys(task), pdata->base + CE_TDQ);

		/* Trigger: 1 | (alg << 8), per sun8i_ce_run_task() */
		v = 1 | ((le32_to_cpu(task->t_common_ctl) & 0x7f) << 8);
		writel(v, pdata->base + CE_TLR);

		/* Poll for completion */
		timeout = 100000;
		while (!(readl(pdata->base + CE_ISR) & BIT(CE_FLOW_PRNG))) {
			if (--timeout <= 0) {
				printf("sunxi-rng: PRNG timeout (TLR=0x%x ISR=0x%x ESR=0x%x TSR=0x%x)\n",
				       v,
				       readl(pdata->base + CE_ISR),
				       readl(pdata->base + CE_ESR),
				       readl(pdata->base + CE_TSR));
				return -ETIMEDOUT;
			}
			udelay(1);
		}

		/* Clear interrupt */
		writel(BIT(CE_FLOW_PRNG), pdata->base + CE_ISR);

		/* Check for errors */
		if (readl(pdata->base + CE_ESR)) {
			printf("sunxi-rng: PRNG error 0x%x\n",
			       readl(pdata->base + CE_ESR));
			return -EIO;
		}

		/* Invalidate output cache */
		invalidate_dcache_range((ulong)out_buf,
					(ulong)out_buf + roundup(PRNG_DATA_SIZE, ARCH_DMA_MINALIGN));

		/* Copy output */
		memcpy(output, out_buf, chunk);
		output += chunk;
		len -= chunk;

		/* Update seed from output for next iteration */
		memcpy(pdata->seed, out_buf, min((size_t)PRNG_SEED_SIZE,
						 (size_t)PRNG_DATA_SIZE));
		pdata->seeded = true;
	}

	return 0;
}

static int sunxi_rng_read(struct udevice *dev, void *data, size_t len)
{
	struct sunxi_rng_plat *pdata = dev_get_plat(dev);

	if (!pdata->seeded) {
		u32 *seed = (u32 *)pdata->seed;
		int i;

		for (i = 0; i < PRNG_SEED_SIZE / 4; i++)
			seed[i] = get_timer(0) ^ (i * 0x5DEECE66D + 0xB);
		pdata->seeded = true;
	}

	return sunxi_rng_generate(pdata, data, len);
}

static int sunxi_rng_probe(struct udevice *dev)
{
	struct sunxi_rng_plat *pdata = dev_get_plat(dev);
	int ret;

	ret = clk_enable(&pdata->clk_bus);
	if (ret) {
		printf("sunxi-rng: failed to enable bus clock: %d\n", ret);
		return ret;
	}

	ret = clk_enable(&pdata->clk_mod);
	if (ret) {
		printf("sunxi-rng: failed to enable mod clock: %d\n", ret);
		return ret;
	}

	ret = reset_deassert(&pdata->reset);
	if (ret) {
		printf("sunxi-rng: failed to deassert reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int sunxi_rng_remove(struct udevice *dev)
{
	struct sunxi_rng_plat *pdata = dev_get_plat(dev);

	reset_assert(&pdata->reset);
	clk_disable(&pdata->clk_mod);
	clk_disable(&pdata->clk_bus);

	return 0;
}

static int sunxi_rng_of_to_plat(struct udevice *dev)
{
	struct sunxi_rng_plat *pdata = dev_get_plat(dev);
	int ret;

	pdata->base = dev_read_addr_ptr(dev);
	if (!pdata->base)
		return -ENODEV;

	ret = clk_get_by_name(dev, "bus", &pdata->clk_bus);
	if (ret)
		return ret;

	ret = clk_get_by_name(dev, "mod", &pdata->clk_mod);
	if (ret)
		return ret;

	ret = reset_get_by_index(dev, 0, &pdata->reset);
	if (ret)
		return ret;

	return 0;
}

static const struct dm_rng_ops sunxi_rng_ops = {
	.read = sunxi_rng_read,
};

static const struct udevice_id sunxi_rng_match[] = {
	{ .compatible = "allwinner,sun8i-h3-crypto" },
	{ .compatible = "allwinner,sun50i-h5-crypto" },
	{ },
};

U_BOOT_DRIVER(sunxi_rng) = {
	.name = "sunxi-rng",
	.id = UCLASS_RNG,
	.of_match = sunxi_rng_match,
	.ops = &sunxi_rng_ops,
	.probe = sunxi_rng_probe,
	.remove = sunxi_rng_remove,
	.plat_auto = sizeof(struct sunxi_rng_plat),
	.of_to_plat = sunxi_rng_of_to_plat,
};
