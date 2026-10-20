// SPDX-License-Identifier: GPL-2.0+
/*
 * G12A/SM1 and GXL DRAM stages: the Meson family of the SPL USB flash
 * session (common/spl/spl_usbflash.c, <spl_usbflash.h>), milestones M1-M4
 * and M6 (SM1), M5 (GXL).
 *
 * On a USB boot the DRAM stage serves the session before it parses the FIT,
 * then continues the boot.  The session itself (alternate list, `script`,
 * UMS, the loop) is the common one; this file is what is Meson's:
 *   - the alternate names and extents, U-Boot proper's too
 *     (<asm/arch/usbflash-alts.h>);
 *   - the RAM alternates' addresses (the block of defines below);
 *   - the SPI NOR alternates;
 *   - `usbflash reset` through the EE watchdog;
 *   - the replay of the GX chain's earlier boot log onto the ACM console.
 * The device controller is dwc2_udc_otg.
 *
 * SPI NOR alternates (u-boot-bin, config, fit), in builds with SPL_DFU_VIRT
 * (the boards that carry NOR: G12A/SM1 CC, GXL CC-V2, AC, AC-V2), if the
 * controller answers a
 * JEDEC RDID: served by the bare SPIFC reader
 * (board/libre-computer/common/meson-spifc-nor.c: RDID and READ only) through
 * DFU virt entities, so the SPL has no SPI flash stack and no SPI write path
 * at all.  NOR stays read-only in every build.
 *
 * SPI NOR on a USB boot: the SPIFC clock gate (HHI_GCLK_MPEG0 bit 30) and the
 * NOR pin functions (G12A/SM1 BOOT_4/5/6/14, GXL BOOT_11/12/13/15) are
 * whatever the BootROM left.  The session
 * records them, enables the gate and selects the NOR function on those four
 * pins for its own reads, and puts back exactly the bits it changed when the
 * session ends, so U-Boot proper starts from the state the BootROM left
 * (board_config_nor_overlays() keys on that gate; lc-flash-android todo N6).
 *
 * The UMS buffers (2 x FSG_BUFLEN) come from the SPL heap in DRAM, where the
 * SD_EMMC DMA lands (it does not land in SRAM: plan M1 round 4 addendum).
 *
 * RAM spans: DRAM above the DRAM stage is unused during SPL.  The stage itself
 * is 0x08000000 + MESON_G12A_SPL_MAX_SIZE, the SPL malloc arena is
 * CONFIG_SPL_CUSTOM_SYS_MALLOC_ADDR (0x0a000000, 32 MiB), and the FIT the host
 * delivered sits at CONFIG_SPL_LOAD_FIT_ADDRESS (0x02000000) and must survive
 * the session, so both RAM alternates are placed above all three.  The kernel
 * alternate is not U-Boot proper's kernel slot (KERNEL_ADDR_R +
 * KERNEL_SLOT_SIZE, include/configs/meson64.h), which runs over the DRAM stage;
 * a full-length UPLOAD therefore returns a different length than proper's.
 *
 * Plan: amlogic/software/boot/spl-usb-flasher-plan.md
 */

#include <board-config.h>
#include <dfu.h>
#include <errno.h>
#include <spl.h>
#include <spl_usbflash.h>
#include <stdio.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <asm/arch/boot.h>
#if defined(CONFIG_MESON_G12A)
#include <asm/arch/g12a-handoff.h>
#else
#include <asm/arch/gx-handoff.h>
#endif
#include <asm/arch/gx-bootlog.h>
#include <asm/arch/usbflash-alts.h>

#if defined(MESON_G12A_DRAM_STAGE) || defined(MESON_GX_DRAM_STAGE)

#if CONFIG_IS_ENABLED(SPI_FLASH_SUPPORT) || CONFIG_IS_ENABLED(DM_SPI_FLASH)
#error "the SPL flash session reads NOR through the bare SPIFC reader: no SPL SPI flash stack"
#endif

#define USBFLASH_SCRIPT_ADDR	0x10000000
#define USBFLASH_SCRIPT_SIZE	0x8000
#define USBFLASH_KERNEL_SIZE	0x04000000
#define USBFLASH_TIMEOUT_MS	60000

#if defined(MESON_G12A_DRAM_STAGE)
/*
 * G12A/SM1: the FIT is staged at 0x02000000, the SPL heap is 0x0a000000 +
 * 32 MiB, so kernel sits at 0x0c000000.
 */
#define USBFLASH_KERNEL_ADDR	0x0c000000
#else
/*
 * GXL: the v2 and ac boards stage the FIT AT 0x0c000000
 * (CONFIG_SPL_LOAD_FIT_ADDRESS), so kernel goes above the script buffer,
 * clear of the FIT, of the SPL heap (0x0a000000 + 32 MiB) and of the DRAM
 * stage (0x08000000 + 0x40000); it ends at 0x14100000, inside the smallest
 * GXL board's DRAM.
 */
#define USBFLASH_KERNEL_ADDR	0x10100000
#endif

/* The NOR path (SPIFC gate, pin mux, bare reader), see top. */
#if CONFIG_IS_ENABLED(DFU_VIRT)
#define USBFLASH_NOR		1
#endif

#ifdef USBFLASH_NOR
/* One pin-mux register: the bits the NOR function owns, and their NOR value. */
struct usbflash_mux {
	ulong addr;
	u32 mask;
	u32 nor;
};

#if defined(MESON_G12A_DRAM_STAGE)
/*
 * SPIFC clock gate and the BOOT-bank pin functions, G12A/SM1.  Addresses and
 * names: BSP uboot/bl33/v2015/arch/arm/cpu/armv8/g12a/firmware/scp_task/
 * secure_apb.h (HHI_GCLK_MPEG0 line 2197, PERIPHS_PIN_MUX_0 line 580,
 * PERIPHS_PIN_MUX_1 line 583).  The SPIFC gate bit is the one
 * meson-spifc-nor.c board_nor_clk_enabled() tests.  Each PERIPHS_PIN_MUX
 * register holds eight pins, four bits per pin (BOOT_0..7 in _0, BOOT_8..15
 * in _1; drivers/pinctrl/meson/pinctrl-meson-g12a.c BANK_PMX("BOOT", ..., 0x0,
 * 0)), and function 3 is the NOR function of nor_d (BOOT_4), nor_q (BOOT_5),
 * nor_c (BOOT_6) and nor_cs (BOOT_14) (GROUP(nor_*, 3), same file; the four
 * groups the DT's nor_pins names).  The 4-bit eMMC uses BOOT_0..3 (data),
 * BOOT_8 (clk), BOOT_10 (cmd) and BOOT_13 (ds), none of which these masks
 * touch.
 */
#define USBFLASH_HHI_GCLK_MPEG0		0xff63c140
static const struct usbflash_mux usbflash_nor_mux[] = {
	{ 0xff6346c0, 0x0fff0000, 0x03330000 },	/* BOOT_4, BOOT_5, BOOT_6 */
	{ 0xff6346c4, 0x0f000000, 0x03000000 },	/* BOOT_14 */
};
/* SPIFC REG_CLOCK, P_SPI_FLASH_CLOCK (BSP arch-g12a/regs.h:12761) */
#define USBFLASH_SPIFC_CLOCK		0xffd14018
#else
/*
 * GXL: HHI_GCLK_MPEG0 bit 30 is the SPIFC gate as on G12 (meson-spifc-nor.c,
 * HIU 0xc883c000 + 0x140).  The pins are one bit per function, and every
 * function that can claim BOOT_11..15 is in PERIPHS_PIN_MUX_7 (0xc88344cc;
 * drivers/pinctrl/meson/pinctrl-meson-gxl.c): nor_d/q/c/cs are bits 13..10
 * on BOOT_11, 12, 13, 15; nand_ale/cle/wen_clk/dqs (bits 4, 3, 2, 0) and
 * emmc_ds (bit 28) would fight them and are cleared.  The GXL boards' eMMC
 * (emmc_pins: emmc_nand_d07, emmc_cmd, emmc_clk) does not use emmc_ds.
 * Measured with the NOR function selected by the shipping U-Boot on an
 * AML-S905X-CC-V2: 0xe0003c00 (arch/arm/mach-meson/tpl-nor-probe.c).
 */
#define USBFLASH_HHI_GCLK_MPEG0		0xc883c140
static const struct usbflash_mux usbflash_nor_mux[] = {
	{ 0xc88344cc, BIT(28) | GENMASK(13, 10) | GENMASK(4, 2) | BIT(0),
	  GENMASK(13, 10) },
};
/* SPIFC REG_CLOCK: spi@8c80 on cbus (meson-gx.dtsi) + 0x18 */
#define USBFLASH_SPIFC_CLOCK		0xc1108c98
#endif
#define USBFLASH_GCLK_SPIFC		BIT(30)
/*
 * SPIFC REG_CLOCK, which the bare reader never programs: on a NOR boot BL2
 * set it, on a USB boot it is whatever the BootROM left (M3 round 1 read all
 * 0xff and an unparsed RDID with it untouched).  Programmed with
 * drivers/spi/meson_spifc.c meson_spifc_set_speed()'s formula at n = 7,
 * clk81 / 8 (about 21 MHz at the 166.67 MHz clk81 measured on GX,
 * spifc-clk-gx.c): DIV n [17:12], CNT_HIGH (n + 1) / 2 - 1 [11:6],
 * CNT_LOW n [5:0].
 */
#define USBFLASH_SPIFC_CLOCK_N7		((7 << 12) | (3 << 6) | 7)
#endif /* USBFLASH_NOR */

/*
 * The EE watchdog, for `usbflash reset`: this stage runs before BL31, so
 * there is no PSCI, and the generic reset_cpu() of an xPL phase only spins
 * (arch/arm/mach-meson/board-common.c).  Addresses: G12A/SM1 watchdog@f0d0
 * on cbus (meson-g12-common.dtsi), names P_WATCHDOG_CNTL / _TCNT / _RST (BSP
 * arch-g12a/regs.h lines 13305, 13309, 13311); GXL watchdog@98d0 on cbus
 * (meson-gx.dtsi), P_WATCHDOG_CNTL / _TCNT / _RESET (BSP arch-gxl/regs.h
 * lines 167, 169, 170), the same block.  The sequence is
 * drivers/watchdog/meson_gxbb_wdt.c's: probe writes CNTL = (xtal / 1000)
 * [17:0] | EE_RESET (21) | CLK_EN (24) | CLKDIV_EN (25), so TCNT counts in
 * ms; set_timeout writes TCNT; start sets EN (18) after a RST ping.
 */
#if defined(MESON_G12A_DRAM_STAGE)
#define USBFLASH_WDT_CNTL		0xffd0f0d0
#define USBFLASH_WDT_TCNT		0xffd0f0d8
#define USBFLASH_WDT_RST		0xffd0f0dc
#else
#define USBFLASH_WDT_CNTL		0xc11098d0
#define USBFLASH_WDT_TCNT		0xc11098d8
#define USBFLASH_WDT_RST		0xc11098dc
#endif
#define USBFLASH_WDT_CNTL_CLKDIV_EN	BIT(25)
#define USBFLASH_WDT_CNTL_CLK_EN	BIT(24)
#define USBFLASH_WDT_CNTL_EE_RESET	BIT(21)
#define USBFLASH_WDT_CNTL_EN		BIT(18)
#define USBFLASH_WDT_XTAL_KHZ		24000	/* the watchdog's clock is xtal */

#ifdef USBFLASH_NOR
/* NOR regions served by the virt alternates; dfu->data.virt.dev_num indexes. */
struct usbflash_nor_region {
	const char *name;
	u32 offset;
	u32 size;
};

static struct usbflash_nor_region usbflash_nor[3];
static int usbflash_nnor;

/* What the session changed in the SPIFC gate and the pin mux, to put back. */
static struct {
	bool changed;
	u32 gclk;
	u32 mux[ARRAY_SIZE(usbflash_nor_mux)];
	u32 clock;
} usbflash_spifc;

/* Enable the SPIFC gate and the NOR pin function; record what was there. */
static void usbflash_spifc_on(void)
{
	const struct usbflash_mux *m;
	u32 gclk = readl(USBFLASH_HHI_GCLK_MPEG0);
	int i;

	printf("SPL usbflash: spifc gate %u (as found)\n",
	       !!(gclk & USBFLASH_GCLK_SPIFC));
	usbflash_spifc.gclk = gclk;
	for (i = 0; i < ARRAY_SIZE(usbflash_nor_mux); i++) {
		m = &usbflash_nor_mux[i];
		usbflash_spifc.mux[i] = readl(m->addr);
		printf("SPL usbflash: mux %lx = %x (as found)\n", m->addr,
		       usbflash_spifc.mux[i]);
	}
	usbflash_spifc.changed = true;
	setbits_le32(USBFLASH_HHI_GCLK_MPEG0, USBFLASH_GCLK_SPIFC);
	/* REG_CLOCK is readable only with the gate on */
	usbflash_spifc.clock = readl(USBFLASH_SPIFC_CLOCK);
	printf("SPL usbflash: spifc clock %x (as found)\n", usbflash_spifc.clock);
	writel(USBFLASH_SPIFC_CLOCK_N7, USBFLASH_SPIFC_CLOCK);
	for (i = 0; i < ARRAY_SIZE(usbflash_nor_mux); i++) {
		m = &usbflash_nor_mux[i];
		clrsetbits_le32(m->addr, m->mask, m->nor);
	}
}

/* Put back the bits usbflash_spifc_on() changed, and only those. */
static void usbflash_spifc_restore(void)
{
	const struct usbflash_mux *m;
	int i;

	if (!usbflash_spifc.changed)
		return;
	writel(usbflash_spifc.clock, USBFLASH_SPIFC_CLOCK);
	for (i = 0; i < ARRAY_SIZE(usbflash_nor_mux); i++) {
		m = &usbflash_nor_mux[i];
		clrsetbits_le32(m->addr, m->mask,
				usbflash_spifc.mux[i] & m->mask);
		printf("SPL usbflash: mux %lx restored %x\n", m->addr,
		       readl(m->addr));
	}
	clrsetbits_le32(USBFLASH_HHI_GCLK_MPEG0, USBFLASH_GCLK_SPIFC,
			usbflash_spifc.gclk & USBFLASH_GCLK_SPIFC);
	usbflash_spifc.changed = false;
	printf("SPL usbflash: spifc restored, gate %u clock %x\n",
	       !!(readl(USBFLASH_HHI_GCLK_MPEG0) & USBFLASH_GCLK_SPIFC),
	       usbflash_spifc.clock);
}

/*
 * Add one NOR region as a virt alternate.  A region that starts at or past
 * the end of a part of known size is left out, and one that runs past it is
 * cut there: only what the part holds is exposed.
 */
static int usbflash_add_nor(const char *name, u32 offset, u32 size, u32 part)
{
	struct usbflash_nor_region *r;
	char dev[8];

	if (part) {
		if (offset >= part) {
			printf("SPL usbflash: NOR %s at 0x%x is past the part, not listed\n",
			       name, offset);
			return 0;
		}
		if (size > part - offset)
			size = part - offset;
	}
	if (usbflash_nnor >= ARRAY_SIZE(usbflash_nor))
		return -ENOSPC;
	r = &usbflash_nor[usbflash_nnor];
	r->name = name;
	r->offset = offset;
	r->size = size;
	snprintf(dev, sizeof(dev), "%d", usbflash_nnor);
	usbflash_nnor++;
	printf("SPL usbflash: NOR %s 0x%x + 0x%x\n", name, offset, size);

	return spl_usbflash_add("virt", dev, name);
}

/* The NOR alternates, if a part answers RDID.  Same rules as proper's list. */
static void usbflash_probe_nor(void)
{
	u32 part, cfg = 0, fit_size = MESON_DFU_SF_FIT_SIZE;
	int ret;

	usbflash_spifc_on();
	if (!board_nor_init()) {
		puts("SPL usbflash: NOR controller did not come out of reset\n");
		return;
	}
	if (!board_nor_probe()) {
		printf("SPL usbflash: NOR absent (RDID %x)\n",
		       board_nor_jedec_id());
		return;
	}
	part = board_nor_size();
	printf("SPL usbflash: NOR answered, RDID %x, size 0x%x (0 = density unknown)\n",
	       board_nor_jedec_id(), part);

	ret = usbflash_add_nor("u-boot-bin", 0, MESON_DFU_SF_UBOOT_SIZE, part);
#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
	cfg = board_config_nor_offset();
	if (!ret)
		ret = usbflash_add_nor("config", cfg,
				       CONFIG_BOARD_CONFIG_NOR_SIZE, part);
	if (cfg > MESON_DFU_SF_FIT_OFFSET &&
	    cfg - MESON_DFU_SF_FIT_OFFSET < fit_size)
		fit_size = cfg - MESON_DFU_SF_FIT_OFFSET;
#endif
	if (!ret)
		ret = usbflash_add_nor("fit", MESON_DFU_SF_FIT_OFFSET, fit_size,
				       part);
	if (ret)
		printf("SPL usbflash: NOR alts not added (%d)\n", ret);
}

/* DFU virt back end: the NOR regions above, read-only. */
static struct usbflash_nor_region *usbflash_nor_of(struct dfu_entity *dfu)
{
	int i = dfu->data.virt.dev_num;

	return (i >= 0 && i < usbflash_nnor) ? &usbflash_nor[i] : NULL;
}

int dfu_get_medium_size_virt(struct dfu_entity *dfu, u64 *size)
{
	struct usbflash_nor_region *r = usbflash_nor_of(dfu);

	if (!r)
		return -ENODEV;
	*size = r->size;

	return 0;
}

int dfu_read_medium_virt(struct dfu_entity *dfu, u64 offset, void *buf,
			 long *len)
{
	struct usbflash_nor_region *r = usbflash_nor_of(dfu);

	if (!r)
		return -ENODEV;
	if (offset >= r->size) {
		*len = 0;
		return 0;
	}
	if (*len > r->size - offset)
		*len = r->size - offset;

	return board_nor_read(r->offset + offset, buf, *len) ? -EIO : 0;
}

int dfu_write_medium_virt(struct dfu_entity *dfu, u64 offset, void *buf,
			  long *len)
{
	puts("SPL usbflash: NOR write refused (read-only session)\n");

	return -EROFS;
}
#else /* !USBFLASH_NOR */
static void usbflash_spifc_restore(void)
{
}

static void usbflash_probe_nor(void)
{
	puts("SPL usbflash: no NOR path on this family\n");
}
#endif /* USBFLASH_NOR */

/* `usbflash reset`: the watchdog, 1 ms (see USBFLASH_WDT_CNTL). */
static void usbflash_reset(void)
{
	mdelay(20);		/* let the UART drain */
	writel(0, USBFLASH_WDT_CNTL);
	writel((USBFLASH_WDT_XTAL_KHZ & GENMASK(17, 0)) |
	       USBFLASH_WDT_CNTL_EE_RESET | USBFLASH_WDT_CNTL_CLK_EN |
	       USBFLASH_WDT_CNTL_CLKDIV_EN, USBFLASH_WDT_CNTL);
	writel(1, USBFLASH_WDT_TCNT);
	writel(0, USBFLASH_WDT_RST);
	setbits_le32(USBFLASH_WDT_CNTL, USBFLASH_WDT_CNTL_EN);
	for (;;)
		;
}

static const struct spl_usbflash_family usbflash_meson = {
	.emmc_alts	= MESON_DFU_MMC_EMMC_ALTS,
	.sd_alts	= MESON_DFU_MMC_SD_ALTS,
	.kernel_addr	= USBFLASH_KERNEL_ADDR,
	.kernel_size	= USBFLASH_KERNEL_SIZE,
	.script_addr	= USBFLASH_SCRIPT_ADDR,
	.script_size	= USBFLASH_SCRIPT_SIZE,
	.timeout_ms	= USBFLASH_TIMEOUT_MS,
	.probe_more	= usbflash_probe_nor,
	.restore	= usbflash_spifc_restore,
	/* GX chain: the TPL's and this stage's UART output (gx-bootlog.h) */
	.replay		= gx_bootlog_replay,
	.reset		= usbflash_reset,
};

/*
 * spl_board_init() - run the flash session on a USB boot, then return to the
 * normal SPL boot path (boot_from_devices(), the FIT from RAM).
 */
void spl_board_init(void)
{
	if (meson_get_boot_device() != BOOT_DEVICE_USB)
		return;

	spl_usbflash_session(&usbflash_meson);
}

#endif /* MESON_G12A_DRAM_STAGE || MESON_GX_DRAM_STAGE */
