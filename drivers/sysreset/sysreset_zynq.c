// SPDX-License-Identifier: GPL-2.0+
/*
 * Zynq-7000 SLCR system reset, as a SYSRESET uclass provider.
 *
 * Board-bootloader policy §2 ("CPU power management & reset") requires that
 * reset go through the SYSRESET uclass on every architecture, and calls a
 * board- or arch-specific reset_cpu() a violation. Zynq-7000 had only the
 * latter: arch/arm/mach-zynq/cpu.c open-coded reset_cpu() around
 * zynq_slcr_cpu_reset(). This driver is that same SLCR sequence, reached
 * through the uclass instead.
 *
 * Why not the generic sysreset_syscon (a "syscon-reboot" DT node over the SLCR
 * regmap, writing PSS_RST_CTRL at offset 0x200), which would have needed no new
 * code at all: the SLCR is WRITE-PROTECTED. arch_cpu_init() ends with
 * zynq_slcr_lock() (arch/arm/mach-zynq/cpu.c), and the only other unlock in the
 * tree is the transient one clk_zynq.c takes around a set_rate, so by the time
 * U-Boot reaches its prompt the SLCR is locked and a bare regmap write to
 * PSS_RST_CTRL is silently discarded. syscon-reboot performs exactly one
 * regmap_update_bits() and has nowhere to put the 0xDF0D unlock, so it cannot
 * express this SoC's reset. sysreset_resetctl is no better: there is no Zynq
 * reset-controller driver in this tree (drivers/reset has no "xlnx,zynq-reset"
 * match; that compatible is Linux-side only), so it would have nothing to bind
 * to.
 *
 * zynq_slcr_cpu_reset() also carries the reboot_status workaround (clearing bits
 * 24-27 so a Xilinx FSBL re-loads the bitstream after a soft reboot). It is
 * preserved rather than dropped: this board no longer uses an FSBL, but the
 * sequence is shared by every Zynq board in the tree.
 *
 * There is no DT node for this. Like sysreset_tegra, the driver is instantiated
 * from a U_BOOT_DRVINFO so it needs no U-Boot-only addition to zynq-7000.dtsi --
 * that file is kept in sync with Linux, and a U-Boot-invented node in it would
 * be a divergence in a shared SoC dtsi for every Zynq board.
 */

#include <dm.h>
#include <errno.h>
#include <sysreset.h>
#include <asm/arch/sys_proto.h>

static int zynq_sysreset_request(struct udevice *dev, enum sysreset_t type)
{
	switch (type) {
	case SYSRESET_WARM:
	case SYSRESET_COLD:
		/*
		 * PSS_RST_CTRL.SOFT_RST resets the whole PS. Zynq-7000 has no
		 * separate warm/cold distinction reachable from the PS, so both
		 * requests map to the same sequence rather than one of them
		 * failing over to nothing.
		 */
		zynq_slcr_cpu_reset();
		break;
	default:
		return -EPROTONOSUPPORT;
	}

	return -EINPROGRESS;
}

static struct sysreset_ops zynq_sysreset_ops = {
	.request = zynq_sysreset_request,
};

U_BOOT_DRIVER(sysreset_zynq) = {
	.id	= UCLASS_SYSRESET,
	.name	= "sysreset_zynq",
	.ops	= &zynq_sysreset_ops,
};

U_BOOT_DRVINFO(sysreset_zynq) = {
	.name = "sysreset_zynq"
};
