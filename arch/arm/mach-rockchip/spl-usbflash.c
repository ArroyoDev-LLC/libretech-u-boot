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
 * No extra alternates.
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

#include <spl.h>
#include <spl_usbflash.h>
#include <sysreset.h>
#include <xpl_bootlog.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/usbflash-alts.h>
#include <linux/delay.h>

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

/* Run the flash session on a MaskROM (USB) boot; called from spl_board_init(). */
void rockchip_spl_usbflash(void)
{
	if (rockchip_brom_bootsource() != BROM_BOOTSOURCE_USB)
		return;

	rockchip_spl_usbflash_board();
	spl_usbflash_session(&usbflash_rockchip);
}
