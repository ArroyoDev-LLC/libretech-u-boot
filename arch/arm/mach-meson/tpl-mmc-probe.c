// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX TPL: SD/eMMC, READ ONLY.
 *
 * WHAT THIS FILE MAY DO: bind a controller, initialise a card, read sectors.
 * WHAT IT MAY NOT DO, AND STRUCTURALLY CANNOT: change one byte of the medium.
 * The only block-layer entry point compiled here is blk_dread(); there is no
 * blk_dwrite, no blk_derase, no mmc_switch of a write-protect or boot-config
 * field.  `grep` the map file if in doubt.
 *
 * WHY IT EXISTS, and why it is not the loader.  The chain's MMC arm
 * (tpl-load-mmc.c) is reached only from gx_tpl_fetch_spl()'s BOOT_DEVICE_SD /
 * BOOT_DEVICE_EMMC cases, and meson_get_boot_device() reports what the
 * BootROM actually booted from.  On the USB bring-up loop that is
 * BOOT_DEVICE_USB, so the loader is dead code on every run the rig can
 * perform: the only way to execute it would be to flash the image to a card
 * and boot from it, which is exactly the step nobody can take until the path
 * has been shown to work once.
 *
 * That is the same bind-order problem TPL_MESON_GX_NOR_PROBE was written for,
 * and it gets the same answer: a read-only diagnostic that runs on EVERY boot
 * device, so a board booting over the cable still exercises pinmux, the
 * controller, the clock, the block layer and the card.  It proves or refutes
 * the two things the loader cannot report from a USB boot -- that
 * apb@d0000000 and the mmc nodes bind at all, and that a real transaction
 * completes -- and it does so without a byte of write capability.
 *
 * NOT the loader, deliberately: it reads sector 0, not
 * MESON_GX_SPL_OFFSET, and it hands nothing to the next stage.  A diagnostic
 * that also booted would not be separable from the thing it is measuring.
 *
 * Never fatal.  A stage whose job is to boot must not hang because a
 * diagnostic read failed.
 *
 * Design: amlogic/software/boot/gxl/design/tpl-per-medium-shape.md.
 */

#include <blk.h>
#include <dm.h>
#include <errno.h>
#include <mmc.h>
#include <memalign.h>
#include <vsprintf.h>

#define MMC_PROBE_READ_LEN	16	/* bytes of sector 0 to report */
#define MMC_PROBE_OFFSET	0x1f0	/* ...starting here: ends in 55 aa */

/**
 * gx_tpl_mmc_devlist() - print what actually bound
 * @dev:	the MMC controller device just acquired
 *
 * The bootph-* marks on apb@d0000000 and the two mmc nodes are a claim about
 * BINDING, and "the read completed" only implies it -- so say it directly, the
 * way the NOR probe does.  The parent chain is the whole assertion: the
 * controller's simple-bus parent is a bound udevice with a name, not an
 * unvisited DT node.
 */
static void gx_tpl_mmc_devlist(struct udevice *dev)
{
	struct udevice *parent = dev_get_parent(dev);
	struct udevice *gparent = parent ? dev_get_parent(parent) : NULL;

	printf("MMC: DM %s < %s < %s\n", dev->name,
	       parent ? parent->name : "(none)",
	       gparent ? gparent->name : "(none)");
}

/**
 * gx_tpl_mmc_probe_one() - bind, initialise and read one controller
 * @seq:	the /aliases sequence number -- on OUR GXL boards 0 is eMMC and
 *		1 is the SD slot, which is the OPPOSITE of what meson-gx.dtsi's
 *		`mmc0 = &sd_emmc_c` comment says.  AML-S905X-CC and CC-V2 both
 *		put SD on sd_emmc_b and eMMC on sd_emmc_c; see
 *		MESON_GX_MMC_SD_DEVNUM (default 1) and MESON_GX_MMC_EMMC_DEVNUM
 *		(default 0), which spend fourteen lines on why that is not a
 *		typo.  This line used to state the dtsi's version, i.e. a
 *		surviving copy of the exact inversion spl.c:502-511 records
 *		as a live bug (`? 1 : 0`, "inverted on both GXL boards").  The
 *		CODE was already right -- gx_tpl_mmc_probe() passes the Kconfig
 *		symbols -- so only the docstring was lying.
 * @what:	a two-letter tag for the console line
 *
 * Every failure below is reported with the errno the call returned rather than
 * a summary, because the four things that can go wrong here -- the node never
 * bound, the controller would not probe, no card answered CMD0/CMD8, the read
 * itself failed -- are four different findings and a board with no card in the
 * slot must be distinguishable from a board whose controller never came up.
 */
static void gx_tpl_mmc_probe_one(int seq, const char *what)
{
	struct blk_desc *bd;
	struct udevice *dev;
	struct mmc *mmc;
	/* blk_dread() fills a whole 512 B sector: the buffer must be one. */
	ALLOC_CACHE_ALIGN_BUFFER(u8, buf, 512);
	int ret, i;

	/*
	 * uclass_get_device_by_seq() and not _by_index(): the seq comes from
	 * the /aliases node, which is what makes 0 mean the SD slot on this
	 * board rather than whichever controller fdtgrep emitted first.  If
	 * the aliases did not survive the grep this returns -ENODEV, which is
	 * itself the result -- an index lookup would instead succeed on the
	 * wrong card and report a confident measurement of nothing.
	 */
	ret = uclass_get_device_by_seq(UCLASS_MMC, seq, &dev);
	if (ret) {
		printf("MMC%d(%s): no dev (%d)\n", seq, what, ret);
		return;
	}

	gx_tpl_mmc_devlist(dev);

	ret = mmc_init_device(seq);
	if (ret) {
		printf("MMC%d: init_device %d\n", seq, ret);
		return;
	}

	mmc = find_mmc_device(seq);
	if (!mmc) {
		printf("MMC%d: no mmc\n", seq);
		return;
	}

	ret = mmc_init(mmc);
	if (ret) {
		/* No card in the slot lands here, and so does a dead one. */
		printf("MMC%d: mmc_init %d\n", seq, ret);
		return;
	}

	bd = mmc_get_blk_desc(mmc);
	if (!bd) {
		printf("MMC%d: no blk\n", seq);
		return;
	}

	printf("MMC%d: blksz %lu lba %lu pcfg %02x\n", seq,
	       (unsigned long)bd->blksz, (unsigned long)bd->lba,
	       mmc->part_config);

	if (blk_dread(bd, 0, 1, buf) != 1) {
		printf("MMC%d: read s0 failed\n", seq);
		return;
	}

	printf("MMC%d: @1f0", seq);
	for (i = 0; i < MMC_PROBE_READ_LEN; i++)
		printf(" %02x", buf[MMC_PROBE_OFFSET + i]);
	printf("\n");
}

/**
 * gx_tpl_mmc_probe() - read sector 0 off both boot-capable controllers
 *
 * Both, not the one the ROM booted from: on a USB boot there is no such one,
 * and on a card boot the OTHER controller is exactly the interesting control
 * (an eMMC read that works while the SD slot reports no card is a stronger
 * statement about the driver than either result alone).
 */
void gx_tpl_mmc_probe(void)
{
	gx_tpl_mmc_probe_one(CONFIG_MESON_GX_MMC_SD_DEVNUM, "sd");
	gx_tpl_mmc_probe_one(CONFIG_MESON_GX_MMC_EMMC_DEVNUM, "em");
}
