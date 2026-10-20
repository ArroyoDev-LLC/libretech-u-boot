// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 MediaTek Inc.
 * Copyright (C) 2025 BayLibre, SAS
 * Author: Julien Masson <jmasson@baylibre.com>
 *         Chris-QJ Chen <chris-qj.chen@mediatek.com>
 */

#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/system.h>
#include <dm/uclass.h>
#include <fdtdec.h>
#include <log.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <wdt.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * DRAM sizing: read the fitted density back out of EMI, do not trust the DT.
 *
 * One device tree and one defconfig serve the 2 GB / 4 GB / 8 GB SKUs of every
 * MT8188-die board (MED-MT88-MX-G5/G7, Genio 510/700 EVK), so the /memory node
 * cannot be a per-SKU constant: it is a claim that the RAM is PRESENT, and
 * over-declaring it hands Linux absent pages. BL2 (libdram) has already sized
 * the parts and programmed the answer into EMI_CONA/EMI_CONH by the time BL33
 * runs, so U-Boot can simply read it.
 *
 * This mirrors libdram's platform_memory_size() (mt8390/platform/emi.c:1190),
 * which sums get_rank_size_by_emi() (:758) over get_rank_nr_by_emi() (:747)
 * ranks. All three inputs are plain MMIO reads; the MMIO window 0x0 +
 * CONFIG_MTK_MEM_MAP_MMIO_SIZE (0x20000000) already covers them.
 *
 *   channels  what the EMI maps, not the board strap: EMI_CONA [9:8] is
 *             log2(channels per EMI) (libdram emi.c get_channel_nr_by_emi()'s
 *             decode), times 2 when the sub-EMI is in use, which libdram's
 *             emi_init() records as bit 30 of INFRACFG_AO_MEM + 0x038
 *             (0x50000533 for more than two channels, 0x10000533 otherwise).
 *             The strap (GPIO_TRAPPING_REG bit 8) says what the PCB is wired
 *             for; BL2 may bring up fewer channels than that (FORCE_2_CHANNEL
 *             on a one-package MED-MT88-MX; CFG_EMI_SINGLE_CHANNEL when CH1
 *             cannot carry data), and the RAM that exists is the RAM the EMI
 *             maps. A misread of the sub-EMI bit can only halve the result,
 *             never over-declare.
 *   ranks     EMI_CONA bits [17:16] set => 2 ranks, else 1
 *   per-rank  EMI_CONH nibbles: ch0r0 >>16, ch0r1 >>20, ch1r0 >>24, ch1r1 >>28
 *   scaling   quad = (channels == 4) ? 2 : 1; size = (nibble * quad) << 28
 *
 * A nibble is therefore 256 MiB of per-channel density before the quad-channel
 * doubling, and the total is rank0 = ch0r0 + ch1r0 (plus rank1 when dual rank).
 */
#define MT8188_IO_PHYS			0x10000000
#define MT8188_INFRACFG_AO_MEM_EMICLK	(MT8188_IO_PHYS + 0x00002038)
#define MT8188_INFRACFG_AO_MEM_SUB_EMI	BIT(30)
#define MT8188_EMI_BASE			(MT8188_IO_PHYS + 0x00219000)
#define MT8188_EMI_CONA			(MT8188_EMI_BASE + 0x000)
#define MT8188_EMI_CONH			(MT8188_EMI_BASE + 0x038)
#define MT8188_EMI_CONA_DUAL_RANK	(0x3 << 16)
#define MT8188_EMI_CONA_CHN_SHIFT	8
#define MT8188_EMI_CONA_CHN_MASK	0x3

/*
 * Sanity band for the computed size. The smallest MT8188-die SKU we ship is
 * 2 GB and the largest is 8 GB; the band is deliberately wider than that so a
 * future density is not rejected, but narrow enough that a garbage read (all
 * zeroes, all ones) falls outside it and we degrade to the DT constant rather
 * than over-declaring absent memory.
 */
#define MT8188_DRAM_SIZE_MIN		SZ_512M
#define MT8188_DRAM_SIZE_MAX		0x400000000ULL	/* 16 GiB */

/*
 * BL33 x3 from our BL2 (arch/arm/mach-mediatek/lowlevel_init.S): 'LCDR' in the
 * high 32 bits, the DRAM size in MiB that BL2 brought up in the low 32. The
 * bring-up stage owns the size; the EMI decode is the fallback for a BL2 that
 * does not tag x3 (the MediaTek BSP chain).
 */
#define MTK_BL33_ARG3_DRAM_TAG		0x4c434452ULL
extern unsigned long mtk_bl33_arg3;

static phys_size_t mt8188_bl2_dram_size(void)
{
	u64 arg3 = mtk_bl33_arg3;

	if ((arg3 >> 32) != MTK_BL33_ARG3_DRAM_TAG)
		return 0;

	return (phys_size_t)(arg3 & 0xffffffffULL) << 20;
}

static phys_size_t mt8188_emi_dram_size(void)
{
	u32 emiclk = readl(MT8188_INFRACFG_AO_MEM_EMICLK);
	u32 cona = readl(MT8188_EMI_CONA);
	u32 conh = readl(MT8188_EMI_CONH);
	unsigned int per_emi = 1U << ((cona >> MT8188_EMI_CONA_CHN_SHIFT) &
				      MT8188_EMI_CONA_CHN_MASK);
	unsigned int channels = per_emi *
		((emiclk & MT8188_INFRACFG_AO_MEM_SUB_EMI) ? 2 : 1);
	unsigned int ranks = (cona & MT8188_EMI_CONA_DUAL_RANK) ? 2 : 1;
	unsigned int quad = (channels == 4) ? 2 : 1;
	u64 rank_size[2];
	u64 total;

	rank_size[0] = (u64)(((conh >> 16) & 0xf) * quad) << 28;
	rank_size[1] = (u64)(((conh >> 20) & 0xf) * quad) << 28;

	if (channels > 1) {
		rank_size[0] += (u64)(((conh >> 24) & 0xf) * quad) << 28;
		rank_size[1] += (u64)(((conh >> 28) & 0xf) * quad) << 28;
	}

	total = rank_size[0];
	if (ranks > 1)
		total += rank_size[1];

	debug("DRAM: emiclk %#x cona %#x conh %#x -> %uch %urank size %#llx\n",
	      emiclk, cona, conh, channels, ranks, total);

	return (phys_size_t)total;
}

int dram_init(void)
{
	phys_size_t detected;
	int ret;

	/*
	 * The DT still supplies the base address, and its size stays as the
	 * fallback -- so a board whose EMI read is implausible boots exactly as
	 * it does today instead of failing or over-declaring.
	 */
	ret = fdtdec_setup_mem_size_base();
	if (ret)
		return ret;

	detected = mt8188_bl2_dram_size();
	if (detected) {
		phys_size_t emi = mt8188_emi_dram_size();

		if (emi != detected)
			printf("DRAM: BL2 size %#llx, EMI decode %#llx; using BL2\n",
			       (unsigned long long)detected,
			       (unsigned long long)emi);
	} else {
		detected = mt8188_emi_dram_size();
	}

	if (detected < MT8188_DRAM_SIZE_MIN || detected > MT8188_DRAM_SIZE_MAX) {
		printf("DRAM: implausible size %#llx, keeping DT %#llx\n",
		       (unsigned long long)detected,
		       (unsigned long long)gd->ram_size);
		return 0;
	}

	gd->ram_size = detected;

	return 0;
}

phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	/*
	 * Limit gd->ram_top not exceeding SZ_4G. Because some peripherals like
	 * MMC requires DMA buffer allocated below SZ_4G.
	 */
	return min(gd->ram_top, SZ_4G);
}

/*
 * Only provide reset_cpu() when the DM sysreset uclass does not. The genio-510
 * defconfig sets CONFIG_SYSRESET=y, so sysreset-uclass.c supplies reset_cpu and
 * a copy here would be a duplicate definition at LTO link. Upstream's
 * mt8370_genio_510_evk / mt8390_genio_700_evk defconfigs do NOT set SYSRESET,
 * so without this they fail to link ("undefined reference to reset_cpu" from
 * arch/arm/lib/reset.c and boot/bootm.c). Mirrors upstream's meson fix
 * 93c13379a44 ("arm: mach-meson: provide reset_cpu only when SYSRESET is
 * disabled") -- keeping the fallback rather than deleting it outright, which is
 * what the genio-510 port originally did.
 */
#if defined(CONFIG_XPL) || !CONFIG_IS_ENABLED(SYSRESET)
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
#endif

int print_cpuinfo(void)
{
	printf("CPU:   MediaTek MT8188\n");
	return 0;
}
