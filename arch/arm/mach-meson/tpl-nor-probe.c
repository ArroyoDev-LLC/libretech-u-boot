// SPDX-License-Identifier: GPL-2.0+
/*
 * Amlogic GX TPL: SPI NOR, READ ONLY.
 *
 * WHAT THIS FILE MAY DO: assert CS, shift out an opcode, shift in bytes.
 * WHAT IT MAY NOT DO, AND STRUCTURALLY CANNOT: change one bit of the part.
 *
 * That is not a promise in a comment, it is a property of what is compiled.
 * The whole file issues exactly TWO opcodes -- 0x9F (READ IDENTIFICATION) and
 * 0x03 (READ DATA BYTES) -- and it talks to the SPI uclass directly rather
 * than through drivers/mtd/spi.  There is no spi_flash, no spi-nor core, no
 * erase or write callback and no WREN (0x06) anywhere in the TPL binary: a
 * NOR part ignores every write-class command until it has seen WREN, and this
 * stage does not contain the byte.  `grep` the map file if in doubt.
 *
 * WHY IT EXISTS.  The chain's SPI NOR arm was an honest refusal
 * (GX_TPL_LOAD_NO_NOR) because meson_spifc_probe()/set_speed() could not get a
 * clock in a phase with no CLK uclass.  That is fixed one layer down
 * (spifc-clk-gx.c).  A fix to a clock path is only worth what a transaction
 * proves, and the cheapest transaction that proves the whole path -- pinmux,
 * controller, divider, CS, both data directions -- is a JEDEC ID read.  So the
 * probe runs on EVERY boot device, not only when the ROM booted from NOR:
 * on the USB dev loop the board boots over the cable and the NOR arm would
 * otherwise never execute at all.
 *
 * Design: amlogic/software/boot/gxl/tpl-spl-chain-design.md.
 */

#include <dm.h>
#include <errno.h>
#include <spi.h>
#include <vsprintf.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <asm/arch/gx-handoff.h>

#define NOR_OP_RDID		0x9f	/* read identification -- 3 bytes out */
#define NOR_OP_READ		0x03	/* read data bytes, 3-byte address */

#define NOR_PROBE_READ_LEN	16	/* bytes of flash content to report */

/*
 * PERIPHS_PIN_MUX_7 -- 0xC88344CC, resolved in the vendor GXL BSP as
 * PERIPHS_PIN_MUX_7 (scp_task/secure_apb.h:143).  It is the periphs mux base
 * 0xc88344b0 (meson-gxl.dtsi, pinctrl@4b0 "mux" reg) plus register index 7,
 * which is where drivers/pinctrl/meson/pinctrl-meson-gxl.c puts every function
 * that can claim BOOT_11..BOOT_15:
 *
 *   nor_d  7,13   nor_q  7,12   nor_c  7,11   nor_cs 7,10
 *   nand_dqs 7,0  nand_ren_wr 7,1  nand_wen_clk 7,2
 *   nand_cle 7,3  nand_ale 7,4    emmc_ds 7,28
 *
 * All ten bits are in this one register, which is why muxing NOR is a single
 * read-modify-write and needs no bank walk: set the four, clear the five that
 * would fight them (nand_ren_wr is BOOT_14, which NOR does not use, so it is
 * left alone).
 *
 * Measured on AML-S905X-CC-V2 2026-08-12 from the shipping u-boot's prompt,
 * with `pinmux status` reporting BOOT_11..13/15 as nor_d/q/c/cs:
 *   0xc88344cc = 0xe0003c00   -- bits 13:10 set, bits 28/4/3/2/0 clear
 */
/*
 * SPIFC register block: cbus base 0xc1100000 + the spi@8c80 node's own reg
 * offset (meson-gx.dtsi), so 0xc1108c80; REG_CLOCK is offset 0x18 in
 * drivers/spi/meson_spifc.c.  Read only, and only to report the divider the
 * driver just programmed -- the driver reaches its registers through regmap.
 */
#define GXL_SPIFC_BASE		0xc1108c80UL
#define GXL_SPIFC_REG_CLOCK	0x18

#define GXL_PERIPHS_PIN_MUX_7	0xc88344ccUL
#define GXL_PIN_MUX_7_NOR_SET	(BIT(13) | BIT(12) | BIT(11) | BIT(10))
#define GXL_PIN_MUX_7_NOR_CLR	(BIT(28) | BIT(4) | BIT(3) | BIT(2) | BIT(0))

/**
 * gx_tpl_nor_mux() - mux BOOT_11/12/13/15 to nor_d/q/c/cs
 *
 * Prints the register before and after, because what the BootROM leaves here
 * on a USB boot has never been read, and a probe that reports only its own
 * result cannot be told apart from one that measured a pin that was never
 * connected.
 *
 * Return: the value found, for gx_tpl_nor_unmux() to put back.
 */
static u32 gx_tpl_nor_mux(void)
{
	u32 before = readl((void *)GXL_PERIPHS_PIN_MUX_7);
	u32 after = (before & ~GXL_PIN_MUX_7_NOR_CLR) | GXL_PIN_MUX_7_NOR_SET;

	writel(after, (void *)GXL_PERIPHS_PIN_MUX_7);
	after = readl((void *)GXL_PERIPHS_PIN_MUX_7);

	printf("NOR: MUX7 %08x -> %08x\n", before, after);

	return before;
}

/**
 * gx_tpl_nor_unmux() - put PERIPHS_PIN_MUX_7 back exactly as it was found
 * @before:	the value gx_tpl_nor_mux() read before it wrote
 *
 * 🔴 A PROBE RESTORES WHAT IT FOUND.  gx_tpl_nor_mux() asserted the NOR mux and
 * never put it back, and this probe runs on EVERY boot device (see the file
 * header), before gx_tpl_mmc_probe() and before gx_tpl_fetch_spl()'s eMMC read
 * -- spl.c places it at the first of the two gx_tpl_dm_init() blocks.  So a TPL
 * that booted from eMMC re-muxed BOOT_11..13/15 to NOR, cleared BIT(28)
 * emmc_ds, and then read the card with the pins left that way.
 *
 * ⚪ WHY THIS IS SAFE TO RESTORE RATHER THAN LEAVE.  Nothing downstream wants
 * the NOR mux held: gx_tpl_fetch_spl()'s BOOT_DEVICE_SPI arm is a named refusal
 * (GX_TPL_LOAD_NO_NOR), so no loader in this stage reads NOR.  The tree already
 * holds probes to this standard -- gxl_sramend_probe() in the SC firmware
 * "restores the word it found" for the same reason.
 *
 * ⚪ WHY IT WAS RANKED LAST AND STILL WORTH DOING.  This whole probe is a
 * bring-up build: CONFIG_TPL_MESON_GX_NOR_PROBE=y appears in exactly ONE of the
 * 1618 files under configs/, and that one is `tpl-nor.config`, a FRAGMENT, not
 * a product defconfig (must-FIND control on the same census: 350 of 1618 carry
 * CONFIG_SPL_TEXT_BASE).  It bites when tpl-nor.config and tpl-mmc.config are
 * combined on a board that needs emmc_ds -- i.e. HS400 -- and whether the TPL's
 * MMC init reaches HS400 is not established anywhere in this tree.  So the
 * hazard is conditional; leaving the hardware as found is not.
 */
static void gx_tpl_nor_unmux(u32 before)
{
	writel(before, (void *)GXL_PERIPHS_PIN_MUX_7);
	printf("NOR: MUX7 restored %08x\n",
	       readl((void *)GXL_PERIPHS_PIN_MUX_7));
}

/**
 * gx_tpl_nor_devlist() - print what actually bound, and what actually probed
 * @bus:	the SPI controller device the probe just acquired
 *
 * The TPL_SIMPLE_BUS repair is a claim about BINDING, and "the JEDEC read
 * completed" only implies it -- so say it directly.  Two lines:
 *
 *   NOR: DM spi@8c80 < bus@c1100000 < soc
 *   NOR: bus@c1100000: spi@8c80*
 *
 * The first is the parent chain, which is the whole assertion: spifc's parent
 * simple-bus is a bound udevice with a name, not an unvisited DT node.  The
 * second lists that bus's children with '*' for probed (DM_FLAG_ACTIVATED) and
 * '.' for bound-but-not-probed, because binding a bus and probing what hangs
 * off it are separate failures and the previous run could not tell them apart.
 *
 * Costs a printf and a loop.  Worth it: without this the only evidence for the
 * Kconfig fix is the absence of an error message.
 */
static void gx_tpl_nor_devlist(struct udevice *bus)
{
	struct udevice *parent = dev_get_parent(bus);
	struct udevice *gparent = parent ? dev_get_parent(parent) : NULL;
	struct udevice *child;

	printf("NOR: DM %s < %s < %s\n", bus->name,
	       parent ? parent->name : "(none)",
	       gparent ? gparent->name : "(none)");

	if (!parent)
		return;

	printf("NOR: %s:", parent->name);
	device_foreach_child(child, parent)
		printf(" %s%c", child->name, device_active(child) ? '*' : '.');
	printf("\n");
}

/**
 * gx_tpl_nor_xfer() - one command, then one read, with CS held across both
 * @slave:	the claimed bus
 * @cmd:	command bytes (opcode, then any address)
 * @cmdlen:	length of @cmd
 * @din:	where the reply goes
 * @dinlen:	how many bytes to shift in
 *
 * Return: 0, or negative on a controller error.
 *
 * Read-only by construction: @cmd is always one of the two opcodes above and
 * @din is the only buffer that moves.  Nothing in this function can be handed
 * a payload to write.
 */
static int gx_tpl_nor_xfer(struct spi_slave *slave, const u8 *cmd, int cmdlen,
			   u8 *din, int dinlen)
{
	int ret;

	ret = spi_xfer(slave, cmdlen * 8, cmd, NULL, SPI_XFER_BEGIN);
	if (ret)
		return ret;

	return spi_xfer(slave, dinlen * 8, NULL, din, SPI_XFER_END);
}

/**
 * gx_tpl_nor_probe() - read the JEDEC ID and a block, and say what came back
 *
 * Reports on the console and, because a console is not always the channel that
 * survives, into the SRAM breadcrumb window where a host can recover it over
 * BROM readMemory after the fact.
 *
 * Never fatal.  A stage whose job is to boot must not hang because a
 * diagnostic read failed, and this read is a diagnostic.
 */
void gx_tpl_nor_probe(void)
{
	struct udevice *bus;
	struct spi_slave *slave;
	u8 cmd[4], id[3], buf[NOR_PROBE_READ_LEN];
	u32 idword, mux7;
	int ret, i;

	writel(GX_NOR_PROBE_MAGIC | GX_NOR_PROBE_START,
	       (void *)GX_NOR_PROBE_ADDR);

	mux7 = gx_tpl_nor_mux();

	/*
	 * The board DT describes a child of the controller (flash@0,
	 * "jedec,spi-nor"), but this phase deliberately builds no driver for
	 * that compatible -- binding one would pull in the spi-nor core, which
	 * is where erase and write live.  So the child never binds, and
	 * _spi_get_bus_and_cs() creates a generic slave on the same chip
	 * select instead.  Speed and mode come from the same numbers the board
	 * DT gives u-boot proper: spi-max-frequency = <83333333>, mode 0.
	 */
	ret = _spi_get_bus_and_cs(0, 0, 83333333, 0, "spi_generic_drv",
				  "nor@0", &bus, &slave);
	if (ret) {
		printf("NOR: no bus (%d)\n", ret);
		writel(GX_NOR_PROBE_MAGIC | GX_NOR_PROBE_NOBUS,
		       (void *)GX_NOR_PROBE_ADDR);
		writel((u32)ret, (void *)GX_NOR_PROBE_ID_ADDR);
		goto out_unmux;
	}

	gx_tpl_nor_devlist(bus);

	ret = spi_claim_bus(slave);
	if (ret) {
		printf("NOR: claim failed (%d)\n", ret);
		writel(GX_NOR_PROBE_MAGIC | GX_NOR_PROBE_NOCLK,
		       (void *)GX_NOR_PROBE_ADDR);
		writel((u32)ret, (void *)GX_NOR_PROBE_ID_ADDR);
		goto out_unmux;
	}

	/*
	 * The number the whole clock fix is about.  spi_claim_bus() has just
	 * run meson_spifc_set_speed(), which in this phase took the HIU
	 * fallback rather than clk_get_rate(); REG_CLOCK is what it computed.
	 * Printing it makes the fix checkable against the value the FULL
	 * u-boot -- with the entire CLK uclass behind it -- leaves in the same
	 * register on the same board, instead of merely asserting they agree.
	 */
	printf("NOR: REG_CLOCK %08x\n",
	       readl((void *)(GXL_SPIFC_BASE + GXL_SPIFC_REG_CLOCK)));

	cmd[0] = NOR_OP_RDID;
	ret = gx_tpl_nor_xfer(slave, cmd, 1, id, sizeof(id));
	if (ret) {
		printf("NOR: RDID xfer failed (%d)\n", ret);
		writel(GX_NOR_PROBE_MAGIC | GX_NOR_PROBE_XFER,
		       (void *)GX_NOR_PROBE_ADDR);
		writel((u32)ret, (void *)GX_NOR_PROBE_ID_ADDR);
		goto out;
	}

	idword = ((u32)id[0] << 16) | ((u32)id[1] << 8) | id[2];
	printf("NOR: JEDEC %02x %02x %02x\n", id[0], id[1], id[2]);
	writel(idword, (void *)GX_NOR_PROBE_ID_ADDR);

	/*
	 * Read offset 0 whatever the ID said.  An unrecognised ID is a result,
	 * not a reason to stop measuring: 0xff/0xff/0xff and 0xff content
	 * together say "nothing is answering on MISO", while a real ID with
	 * 0xff content would say "the part is there and blank".  Those are
	 * different findings and only the second read separates them.
	 */
	cmd[0] = NOR_OP_READ;
	cmd[1] = 0;
	cmd[2] = 0;
	cmd[3] = 0;
	ret = gx_tpl_nor_xfer(slave, cmd, 4, buf, sizeof(buf));
	if (ret) {
		printf("NOR: READ xfer failed (%d)\n", ret);
		writel(GX_NOR_PROBE_MAGIC | GX_NOR_PROBE_XFER,
		       (void *)GX_NOR_PROBE_ADDR);
		goto out;
	}

	printf("NOR: @0");
	for (i = 0; i < (int)sizeof(buf); i++)
		printf(" %02x", buf[i]);
	printf("\n");

	writel(GX_NOR_PROBE_MAGIC | GX_NOR_PROBE_OK,
	       (void *)GX_NOR_PROBE_ADDR);
out:
	spi_release_bus(slave);
out_unmux:
	/* Leave the pins as this probe found them; see gx_tpl_nor_unmux(). */
	gx_tpl_nor_unmux(mux7);
}
