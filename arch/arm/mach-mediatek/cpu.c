// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 */

#include <asm/global_data.h>
#include <asm/io.h>
#include <bootsource.h>
#include <chip_serial.h>
#include <cpu_func.h>
#include <dm.h>
#include <dm/ofnode.h>
#include <env.h>
#include <event.h>
#include <fdtdec.h>
#include <init.h>
#include <linux/arm-smccc.h>
#include <linux/types.h>
#include <mapmem.h>
#include <membuf.h>
#include <stdio.h>
#include <stdio_dev.h>
#include <wdt.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <dm/uclass-internal.h>

DECLARE_GLOBAL_DATA_PTR;

#define MTK_SIP_PLAT_BINFO ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_64, \
					      ARM_SMCCC_OWNER_SIP, 0x529)

int arch_cpu_init(void)
{
	icache_enable();

	return 0;
}

void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}

/**
 * mediatek_sip_part_name - get the part name
 *
 * Retrieve the part name of platform description.
 *
 * This only applicable to SoCs that support SIP plat binfo SMC call.
 *
 * Returns: the part name or 0 if error or no part name
 */
u32 mediatek_sip_part_name(void)
{
	if (CONFIG_IS_ENABLED(TARGET_MT8188) || CONFIG_IS_ENABLED(TARGET_MT8189) ||
	    CONFIG_IS_ENABLED(TARGET_MT8195) || CONFIG_IS_ENABLED(TARGET_MT8365)) {
		struct arm_smccc_res res __maybe_unused;

		arm_smccc_smc(MTK_SIP_PLAT_BINFO, 0, 0, 0, 0, 0, 0, 0, &res);
		if (res.a0)
			return 0;

		return res.a1;
	}

	return 0;
}

/**
 * mediatek_sip_segment_name - get the segment name
 *
 * Retrieve the segment name of platform description.
 *
 * This only applicable to SoCs that support SIP plat binfo SMC call.
 *
 * Returns: the segment name or 0 if error or no segment name
 */
u32 mediatek_sip_segment_name(void)
{
	if (CONFIG_IS_ENABLED(TARGET_MT8188) || CONFIG_IS_ENABLED(TARGET_MT8189) ||
	    CONFIG_IS_ENABLED(TARGET_MT8195) || CONFIG_IS_ENABLED(TARGET_MT8365)) {
		struct arm_smccc_res res __maybe_unused;

		arm_smccc_smc(MTK_SIP_PLAT_BINFO, 1, 0, 0, 0, 0, 0, 0, &res);
		if (res.a0)
			return 0;

		return res.a1;
	}

	return 0;
}

/*
 * On-die unique ID (HRID). Linux nvmem (mtk-efuse) maps the whole eFuse
 * window as bytes; mtkclient and ATF's atf_arg_t.HRID[2] read 8 B at
 * +0x140 of that window (unique-id@140 in the control DT). socinfo-data*
 * is the part/segment name — not this. SIP MTK_SIP_PLAT_BINFO is also
 * part/segment, not the die serial. A failed read leaves serial# unset,
 * deleting a saved one.
 */
#define MTK_EFUSE_HRID_OFF	0x140
#define MTK_EFUSE_HRID_LEN	8

#if !CONFIG_IS_ENABLED(XPL_BUILD)
/* The chip id (include/chip_serial.h): the HRID as 16 hex, U-Boot proper only */
int chip_serial_get(char *buf, size_t size)
{
	static const char *const compat[] = {
		"mediatek,efuse",
		"mediatek,mt8186-efuse",
		"mediatek,mt8188-efuse",
		"mediatek,mt8173-efuse",
	};
	u8 id[MTK_EFUSE_HRID_LEN];
	ofnode node;
	fdt_addr_t addr;
	fdt_size_t size_dt;
	void *base;
	bool blank, burned;
	int i, c;

	addr = FDT_ADDR_T_NONE;
	size_dt = 0;
	for (c = 0; c < ARRAY_SIZE(compat) && addr == FDT_ADDR_T_NONE; c++) {
		ofnode_for_each_compatible_node(node, compat[c]) {
			if (!ofnode_is_enabled(node))
				continue;
			addr = ofnode_get_addr_size_index(node, 0, &size_dt);
			if (addr == FDT_ADDR_T_NONE ||
			    size_dt < MTK_EFUSE_HRID_OFF + MTK_EFUSE_HRID_LEN) {
				addr = FDT_ADDR_T_NONE;
				continue;
			}
			break;
		}
	}
	if (addr == FDT_ADDR_T_NONE)
		return -ENODEV;

	base = map_physmem(addr, size_dt, MAP_NOCACHE);
	if (!base)
		return -ENOMEM;

	for (i = 0; i < MTK_EFUSE_HRID_LEN; i++)
		id[i] = readb(base + MTK_EFUSE_HRID_OFF + i);
	unmap_physmem(base, MAP_NOCACHE);

	blank = true;
	burned = true;
	for (i = 0; i < MTK_EFUSE_HRID_LEN; i++) {
		if (id[i])
			blank = false;
		if (id[i] != 0xff)
			burned = false;
	}
	if (blank || burned)
		return -ENODATA;

	return chip_serial_hex(buf, size, id, sizeof(id));
}

/*
 * serial# = the chip id, replacing a saved one from another board; with no
 * stored ethaddr, the MAC from it (chip_serial_ethaddr()). U-Boot reads no
 * programmed MAC on MediaTek, so the chip id is the only source it has.
 */
void mtk_setup_serial(void)
{
	if (!chip_serial_env())
		chip_serial_ethaddr();
}

#if !IS_ENABLED(CONFIG_MTK_TZ_MOVABLE)
int arch_misc_init(void)
{
	mtk_setup_serial();

	return 0;
}
#endif

#if CONFIG_IS_ENABLED(BOOTSOURCE)
enum bootsource mtk_bootsource(void);

/*
 * BL33 x2 'LCUS' tag (USB DRAM-boot) then GPIO_TRAPPING_REG on
 * MT8188/MT8189; unknown on every other MediaTek TARGET with no
 * documented trap. See arch/arm/mach-mediatek/bootsource.c.
 * bootdevice follows the source: MMC index on eMMC/SD, mmc0 on SPI NOR
 * (env/boot.ini live on the first MMC, policy §4); -1 otherwise.
 */
static int mtk_bootsource_settings_r(void)
{
	enum bootsource src = mtk_bootsource();
	int mmc_dev = -1;

	switch (src) {
	case BOOTSOURCE_EMMC:
	case BOOTSOURCE_SD:
#ifdef CONFIG_ENV_MMC_DEVICE_INDEX
		mmc_dev = CONFIG_ENV_MMC_DEVICE_INDEX;
#else
		mmc_dev = 0;
#endif
		break;
	case BOOTSOURCE_SPI:
		mmc_dev = 0;
		break;
	default:
		break;
	}

	bootsource_publish(src, mmc_dev);

	return 0;
}
EVENT_SPY_SIMPLE(EVT_SETTINGS_R, mtk_bootsource_settings_r);
#endif

/* The USB console on a USB boot: common/usb_boot_acm_console.c (MTK_USB_BOOT_ACM_CONSOLE). */
#else
void mtk_setup_serial(void)
{
}
#endif
