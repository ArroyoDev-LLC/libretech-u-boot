// SPDX-License-Identifier: GPL-2.0+
/*
 * The Rockchip family of the SPL USB flash session (common/spl/spl_usbflash.c,
 * <spl_usbflash.h>): on a MaskROM (USB) boot the SPL serves the session before
 * it boots the FIT it was sent with, then continues the boot.
 *
 * What is Rockchip's:
 *   - the alternate names and extents (<asm/arch-rockchip/usbflash-alts.h>);
 *   - the RAM alternates' addresses (below);
 *   - `usbflash reset` through sysreset (the CRU's global soft reset);
 *   - the timeout.
 * With SPL_XPL_BOOTLOG the session replays the boot so far -- the TPL's
 * and this SPL's UART output (<xpl_bootlog.h>) -- into its ACM console.
 * With SPL_ROCKCHIP_USBFLASH_NOR, the SPI NOR regions as extra alternates
 * (read-write, see usbflash_probe_nor()); without it, none.
 * With SPL_DFU_VIRT, a read-only `info` alternate: what the board is, read
 * here because the MaskROM itself cannot be read back (the host's probe of
 * a board in USB boot mode, then `usbflash bootrom` to return it there).
 * `usbflash bootrom` arms the BOOT_BROM_DOWNLOAD flag our TPL acts on.
 *
 * RAM spans, RK3399 MaskROM boot: the SPL runs at 0 (CONFIG_SPL_TEXT_BASE),
 * the FIT it was sent with sits at CONFIG_SPL_LOAD_FIT_ADDRESS (0x00080000 on
 * roc-rk3399-pc-maskrom, the default 0x00200000 elsewhere),
 * U-Boot proper loads at CONFIG_TEXT_BASE (0x00800000), and the SPL heap and
 * stack end at CONFIG_SPL_STACK_R_ADDR (0x03e00000), with the BSS above.  Both
 * RAM alternates are placed above all of them, as on GXL.
 *
 * Plan: claude rockchip/rk3399/plan/spl-usb-flasher.md
 */

#include <chip_serial.h>
#include <dfu.h>
#include <fdtdec.h>
#include <malloc.h>
#include <mmc.h>
#include <spl.h>
#include <spl_usbflash.h>
#include <spi_flash.h>
#include <sysreset.h>
#include <vsprintf.h>
#include <xpl_bootlog.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/arch-rockchip/boot_mode.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/usbflash-alts.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/sizes.h>

DECLARE_GLOBAL_DATA_PTR;

#define USBFLASH_SCRIPT_ADDR	0x10000000
#define USBFLASH_SCRIPT_SIZE	0x8000
#define USBFLASH_KERNEL_ADDR	0x10100000
#define USBFLASH_KERNEL_SIZE	0x04000000
/* kernel-lzma's staging window: right above `kernel`, as large */
#define USBFLASH_LZMA_STAGING	(USBFLASH_KERNEL_ADDR + USBFLASH_KERNEL_SIZE)
/*
 * Shorter than Meson's 60 s: the MaskROM loader boots the FIT it carries, so
 * a boot with no host flashing tool waits this long once.
 */
#define USBFLASH_TIMEOUT_MS	10000

/* `usbflash reset`: sysreset, a global soft reset of the SoC. */
static void usbflash_reset(void)
{
	mdelay(20);		/* let the UART drain */
	sysreset_walk_halt(SYSRESET_COLD);
}

/*
 * `usbflash bootrom`: the BOOT_BROM_DOWNLOAD flag in the boot-mode register,
 * then the session's reset (cmd/bootrom.c's mechanism, bootrom-rk.c).  The
 * MaskROM never reads the register; our TPL does, first on any medium, and
 * returns to the MaskROM.  A medium whose first loader is another's boots
 * that instead, so the host confirms the return by the MaskROM's USB id.
 */
static int usbflash_bootrom_check(void)
{
	return CONFIG_ROCKCHIP_BOOT_MODE_REG ? 0 : -ENOSYS;
}

static int usbflash_bootrom_arm(void)
{
	writel(BOOT_BROM_DOWNLOAD, CONFIG_ROCKCHIP_BOOT_MODE_REG);

	return readl(CONFIG_ROCKCHIP_BOOT_MODE_REG) == BOOT_BROM_DOWNLOAD ? 0 : -EIO;
}

#if CONFIG_IS_ENABLED(DFU_VIRT)
/*
 * The `info` alternate (virt 0): one "key=value" per line after "lc-info 1",
 * the status readout's format (board/libre-computer/common/lc-status.c), built
 * once when the session starts.  A value the board cannot give is left out,
 * never written as 0 (`chip.error=` says why the chip id is missing).
 *   chip          the eFuse cpu id, as U-Boot proper's serial# (chip_serial_get)
 *   boot.source   the MaskROM's record of this boot (BROM_BOOTSOURCE_*)
 *   boot.flag     the boot-mode register as this SPL found it
 *   sd.present    the SD slot's card detect, mmc 1 (mmc_getcd)
 *   dram          the DRAM size the TPL set up, in bytes
 */
#define USBFLASH_INFO_MAX	512

static char usbflash_info[USBFLASH_INFO_MAX];
static int usbflash_info_len;

/*
 * One line of `info`.  SPL's tiny printf has snprintf but no vsnprintf, and
 * ignores the size; the lines below come to about 150 bytes in all.
 */
#define usbflash_info_put(...)						\
	(usbflash_info_len += snprintf(usbflash_info + usbflash_info_len,	\
				       sizeof(usbflash_info) - usbflash_info_len, \
				       __VA_ARGS__))

static void usbflash_info_build(void)
{
	char serial[CHIP_SERIAL_MAX];
	struct mmc *sd;
	int ret;

	usbflash_info_len = 0;
	usbflash_info_put("lc-info 1\n");
	ret = chip_serial_get(serial, sizeof(serial));
	if (ret)
		usbflash_info_put("chip.error=%d\n", ret);
	else
		usbflash_info_put("chip=%s\n", serial);
	usbflash_info_put("boot.source=%d\n", rockchip_brom_bootsource());
	if (CONFIG_ROCKCHIP_BOOT_MODE_REG)
		usbflash_info_put("boot.flag=0x%x\n",
				  readl(CONFIG_ROCKCHIP_BOOT_MODE_REG));
	sd = find_mmc_device(1);
	if (sd) {
		ret = mmc_getcd(sd);
		if (ret >= 0)
			usbflash_info_put("sd.present=%d\n", !!ret);
	}
	if (gd->ram_size)
		usbflash_info_put("dram=%lu\n", (unsigned long)gd->ram_size);
}

static void usbflash_add_info(void)
{
	int ret;

	usbflash_info_build();
	ret = spl_usbflash_add("virt", "0", "info");
	if (ret)
		printf("SPL usbflash: info alt not added (%d)\n", ret);
}

int dfu_get_medium_size_virt(struct dfu_entity *dfu, u64 *size)
{
	if (dfu->data.virt.dev_num)
		return -ENODEV;
	*size = usbflash_info_len;

	return 0;
}

int dfu_read_medium_virt(struct dfu_entity *dfu, u64 offset, void *buf,
			 long *len)
{
	if (dfu->data.virt.dev_num)
		return -ENODEV;
	if (offset >= usbflash_info_len) {
		*len = 0;
		return 0;
	}
	if (*len > usbflash_info_len - offset)
		*len = usbflash_info_len - offset;
	memcpy(buf, usbflash_info + offset, *len);

	return 0;
}

int dfu_write_medium_virt(struct dfu_entity *dfu, u64 offset, void *buf,
			  long *len)
{
	return -EROFS;
}
#else
static void usbflash_add_info(void)
{
}
#endif

#if CONFIG_IS_ENABLED(ROCKCHIP_USBFLASH_NOR)
/*
 * The SPI NOR alternates, read-write: the regions of the canonical NOR layout
 * (claude design/rules/bootloader/policy.md section 4) under the names the
 * Meson family's session uses.  The config block sits where boot/board-config.c
 * places it, the last 64 KiB of the flash or of the 16 MiB a 3-byte address
 * reaches; u-boot-bin and fit are clamped below it on a smaller part, so no
 * download can run from one region into the next.  dfu_sf erases and writes.
 */
static void usbflash_probe_nor(void)
{
	struct spi_flash *flash;
	char dev[16], spec[64];
	u32 cfg, boot, fit;
	int ret;

	flash = spi_flash_probe(CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS,
				CONFIG_SF_DEFAULT_SPEED, CONFIG_SF_DEFAULT_MODE);
	if (!flash || !flash->size) {
		puts("SPL usbflash: no SPI NOR, no NOR alts\n");
		return;
	}
	cfg = min_t(u32, flash->size, SZ_16M) - ROCKCHIP_DFU_SF_CONFIG_SIZE;
	boot = min_t(u32, ROCKCHIP_DFU_SF_UBOOT_SIZE, cfg);
	fit = cfg > ROCKCHIP_DFU_SF_FIT_OFFSET ?
	      min_t(u32, ROCKCHIP_DFU_SF_FIT_SIZE, cfg - ROCKCHIP_DFU_SF_FIT_OFFSET) : 0;
	printf("SPL usbflash: NOR %s %u KiB: u-boot-bin 0x0 + 0x%x, config 0x%x + 0x%x, fit 0x%x + 0x%x\n",
	       flash->name, flash->size >> 10, boot, cfg, ROCKCHIP_DFU_SF_CONFIG_SIZE,
	       ROCKCHIP_DFU_SF_FIT_OFFSET, fit);
	snprintf(dev, sizeof(dev), "%d:%d", CONFIG_SF_DEFAULT_BUS, CONFIG_SF_DEFAULT_CS);

	snprintf(spec, sizeof(spec), "u-boot-bin raw 0x0 0x%x", boot);
	ret = spl_usbflash_add("sf", dev, spec);
	if (!ret) {
		snprintf(spec, sizeof(spec), "config raw 0x%x 0x%x", cfg,
			 ROCKCHIP_DFU_SF_CONFIG_SIZE);
		ret = spl_usbflash_add("sf", dev, spec);
	}
	if (!ret && fit) {
		snprintf(spec, sizeof(spec), "fit raw 0x%x 0x%x",
			 ROCKCHIP_DFU_SF_FIT_OFFSET, fit);
		ret = spl_usbflash_add("sf", dev, spec);
	}
	if (ret)
		printf("SPL usbflash: NOR alts not all added (%d)\n", ret);
}
#else
static void usbflash_probe_nor(void)
{
}
#endif

/* The family's alternates after the MMC ones: `info`, then the SPI NOR's. */
static void usbflash_probe_more(void)
{
	usbflash_add_info();
	usbflash_probe_nor();
}

static const struct spl_usbflash_family usbflash_rockchip = {
	.emmc_alts	= ROCKCHIP_DFU_MMC_EMMC_ALTS,
	.sd_alts	= ROCKCHIP_DFU_MMC_SD_ALTS,
	.kernel_addr	= USBFLASH_KERNEL_ADDR,
	.kernel_size	= USBFLASH_KERNEL_SIZE,
	.script_addr	= USBFLASH_SCRIPT_ADDR,
	.script_size	= USBFLASH_SCRIPT_SIZE,
	.timeout_ms	= USBFLASH_TIMEOUT_MS,
	.lzma_staging_addr = USBFLASH_LZMA_STAGING,
	.reset		= usbflash_reset,
	.bootrom_check	= usbflash_bootrom_check,
	.bootrom_arm	= usbflash_bootrom_arm,
	.probe_more	= usbflash_probe_more,
#if CONFIG_IS_ENABLED(XPL_BOOTLOG)
	.replay		= xpl_bootlog_replay,
#endif
};

/*
 * The board's setup for a session, e.g. a faster CPU clock for the LZMA
 * alternate's unpacking; on a MaskROM boot only.
 */
__weak void rockchip_spl_usbflash_board(void)
{
}

/*
 * The session's gadget role in the SPL, on a MaskROM (USB) boot.
 *
 * A board DTS that wires the OTG controller to a host port sets it
 * `dr_mode = "host"` (ROC-RK3328-CC, rk3328-roc.dtsi), and
 * dwc2_udc_otg_bind() refuses that node, so no UDC would exist for the
 * session. U-Boot proper takes the role in its live tree
 * (board-rockchip.c); the SPL runs on the flat control blob, which binds
 * DM in spl_early_init() and has no free space. So, before that bind,
 * the blob is copied into a larger buffer, the usb-otg alias's node gets
 * `peripheral`, and gd->fdt_blob points at the copy. It lives only as
 * long as the SPL: U-Boot proper and the kernel get their own trees.
 * A controller that is not `host` (the ROC-RK3399-PC's Type-C DWC3 is
 * `otg`), and any boot from a medium, are left alone.
 */
int fdtdec_board_setup(const void *fdt_blob)
{
	const char *path, *mode;
	void *blob;
	int node, size;

	if (rockchip_brom_bootsource() != BROM_BOOTSOURCE_USB)
		return 0;
	path = fdt_get_alias(fdt_blob, "usb-otg");
	if (!path)
		return 0;
	node = fdt_path_offset(fdt_blob, path);
	if (node < 0)
		return 0;
	mode = fdt_getprop(fdt_blob, node, "dr_mode", NULL);
	if (!mode || strcmp(mode, "host"))
		return 0;

	size = fdt_totalsize(fdt_blob) + 64;
	blob = malloc(size);
	if (!blob || fdt_open_into(fdt_blob, blob, size)) {
		printf("usbflash: no room to set the OTG port's role\n");
		return 0;
	}
	node = fdt_path_offset(blob, path);
	if (node < 0 || fdt_setprop_string(blob, node, "dr_mode", "peripheral")) {
		printf("usbflash: the OTG port's role not set\n");
		return 0;
	}
	gd->fdt_blob = blob;

	return 0;
}

/* Run the flash session on a MaskROM (USB) boot; called from spl_board_init(). */
void rockchip_spl_usbflash(void)
{
	if (rockchip_brom_bootsource() != BROM_BOOTSOURCE_USB)
		return;

	rockchip_spl_usbflash_board();
	spl_usbflash_session(&usbflash_rockchip);
}
