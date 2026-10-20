// SPDX-License-Identifier: GPL-2.0+
/*
 * HamGeek / RIGUKE XC7Z100 + ADRV9009 ("sdr-pro-alpha") — board-specific code.
 *
 * Deterministic network identity (board-bootloader policy §1 "Network MAC &
 * serial number"): a board's MAC must be derived from a unique hardware ID so
 * every unit gets a stable identity with no per-unit provisioning. A random
 * MAC is only ever a last-resort *failure signal*.
 *
 * Without this file U-Boot sets no ethaddr at all, so it never injects
 * local-mac-address into the FDT it hands Linux; the kernel's macb driver then
 * logs "invalid hw address, using random", addr_assign_type becomes
 * NET_ADDR_RANDOM, and anything that requires a permanent MAC (our fleet
 * client) loses the board's identity entirely. Measured on this unit before
 * the fix: end0 addr_assign_type = 3, fleet entity MAC 00:00:00:00:00:00.
 *
 * Fallback chain, highest priority first:
 *
 *   1. ADRV9009 / AD9528 unique ID — STUB, always "unsupported".
 *      See adrv9009_transceiver_unique_id() below; neither part has a
 *      per-device ID.
 *   2. QSPI NOR factory unique ID (Winbond W25Q256, opcode 0x4B) — the
 *      working source. Hashed to 6 bytes, marked locally-administered.
 *   3. U-Boot's own net_random_ethaddr() — left in place, untouched, as the
 *      final rung. A unit that comes up with a MAC that changes across boots
 *      means rungs 1 and 2 both failed.
 *
 * This is a straight port of board/xilinx/zynq/zynq-e310/board.c (same
 * derivation, same LAA/unicast forcing, same JEDEC gating). The two boards
 * carry the same W25Q256 part but different factory UIDs, so they derive
 * different MACs — which is the point.
 *
 * Note on the Zynq-7000 "Device DNA": the 57-bit Device DNA is a *PL fabric*
 * primitive (DNA_PORT), readable only through a custom AXI IP instantiated in
 * a bitstream. This board deliberately boots with a DARK PL (the bitstream
 * lifecycle moved to Linux userspace), so U-Boot has no DNA_PORT to read, and
 * per UG470 up to 32 devices may share a DNA value anyway. The PS-side eFUSE
 * controller at 0xF800D000 (DT node efuse@f800d000) exposes only the
 * secure-boot/AES/JTAG-disable control and status bits on Zynq-7000; unlike
 * ZynqMP it has no per-device unique ID field and no MAC eFUSE. So there is no
 * PS-readable SoC unique ID on this part, which is why rung 2 uses the QSPI
 * flash. Do not re-open this.
 */

#include <dm.h>
#include <dfu.h>
#include <efi_loader.h>
#include <env.h>
#include <log.h>
#include <net.h>
#include <spi.h>
#include <spi-mem.h>
#include <board-config.h>
#include <spi_flash.h>
#include <dm/uclass.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <u-boot/sha256.h>

/* Winbond "Read Unique ID Number": 0x4B, 4 dummy bytes, 8 data bytes. */
#define ADRV9009_SPINOR_OP_RDUID	0x4b
#define ADRV9009_SPINOR_RDUID_DUMMY	4
#define ADRV9009_SPINOR_RDUID_LEN	8

/* JEDEC "Read Identification": 0x9F, 3 data bytes (mfr, type, capacity). */
#define ADRV9009_SPINOR_OP_RDID		0x9f
#define ADRV9009_SPINOR_RDID_LEN	3
#define ADRV9009_SPINOR_MFR_WINBOND	0xef

/* Bytes of the identity hash rendered as hex into serial# (16 hex chars). */
#define ADRV9009_SERIAL_HEX_BYTES	8

/**
 * adrv9009_transceiver_unique_id() - rung 1: per-device ID from the RF chain
 *
 * STUB — always fails, by design.
 *
 * Neither part in this board's RF chain carries a per-device unique
 * identifier. The ADRV9009's PRODUCT_ID / SPI product-ID registers are
 * *common* to the die revision, and the AD9528 clock chip likewise exposes
 * only CHIPTYPE/PRODID/GRADE. There is no OTP or serial region on either.
 * Using any of those registers would give every board in the fleet the same
 * MAC — strictly worse than the random fallback.
 *
 * The rung is kept so the priority order of the chain is explicit in the code
 * and so a future part (or a vendor erratum saying otherwise) has an obvious
 * place to land. It is also unreachable in practice on this board: PS SPI0 is
 * left to Linux (CONFIG_ZYNQ_SPI is off) precisely so SPI bus 0 stays QSPI for
 * the SPL/u-boot.img load path.
 *
 * Return: always -ENOTSUPP (fall through to rung 2).
 */
static int adrv9009_transceiver_unique_id(u8 *uid, size_t len)
{
	return -ENOTSUPP;
}

/**
 * adrv9009_qspi_unique_id() - rung 2: QSPI NOR factory 64-bit unique ID
 * @uid: caller buffer, ADRV9009_SPINOR_RDUID_LEN bytes
 *
 * The QSPI flash is a Winbond W25Q256 (DT compatible "winbond,w25q256").
 * Winbond parts carry a factory-programmed, read-only 64-bit unique serial
 * number readable with opcode 0x4B. It is per-die, so it is a genuine
 * per-unit hardware ID.
 *
 * The JEDEC ID is read first and the manufacturer byte checked against
 * Winbond: 0x4B is a vendor extension, not a JEDEC-standard opcode, so on a
 * non-Winbond part it may mean something entirely different (or nothing).
 * Better to fall through to the random rung than to hash garbage.
 *
 * Both ops are spi_mem ops, so zynq_qspi_exec_op() emits the dummy phase.
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int adrv9009_qspi_unique_id(u8 *uid, size_t len)
{
	u8 id[ADRV9009_SPINOR_RDID_LEN];
	struct spi_slave *slave;
	struct udevice *dev;
	int ret, i;

	if (len < ADRV9009_SPINOR_RDUID_LEN)
		return -EINVAL;

	ret = uclass_first_device_err(UCLASS_SPI_FLASH, &dev);
	if (ret) {
		debug("adrv9009: no SPI flash device (%d)\n", ret);
		return ret;
	}

	slave = dev_get_parent_priv(dev);
	if (!slave)
		return -ENODEV;

	struct spi_mem_op rdid = SPI_MEM_OP(SPI_MEM_OP_CMD(ADRV9009_SPINOR_OP_RDID, 1),
					    SPI_MEM_OP_NO_ADDR,
					    SPI_MEM_OP_NO_DUMMY,
					    SPI_MEM_OP_DATA_IN(sizeof(id), id, 1));

	struct spi_mem_op rduid = SPI_MEM_OP(SPI_MEM_OP_CMD(ADRV9009_SPINOR_OP_RDUID, 1),
					     SPI_MEM_OP_NO_ADDR,
					     SPI_MEM_OP_DUMMY(ADRV9009_SPINOR_RDUID_DUMMY, 1),
					     SPI_MEM_OP_DATA_IN(ADRV9009_SPINOR_RDUID_LEN,
								uid, 1));

	ret = spi_mem_exec_op(slave, &rdid);
	if (ret) {
		debug("adrv9009: SPI NOR RDID failed (%d)\n", ret);
		return ret;
	}

	if (id[0] != ADRV9009_SPINOR_MFR_WINBOND) {
		debug("adrv9009: SPI NOR mfr 0x%02x is not Winbond, no unique ID\n",
		      id[0]);
		return -ENOTSUPP;
	}

	ret = spi_mem_exec_op(slave, &rduid);
	if (ret) {
		debug("adrv9009: SPI NOR RDUID failed (%d)\n", ret);
		return ret;
	}

	/* An all-0x00 or all-0xff answer means the part did not respond. */
	for (i = 0; i < ADRV9009_SPINOR_RDUID_LEN; i++)
		if (uid[i] != 0x00 && uid[i] != 0xff)
			return 0;

	debug("adrv9009: SPI NOR unique ID is degenerate, rejecting\n");
	return -ENODATA;
}

/**
 * adrv9009_setup_identity() - install a deterministic ethaddr + serial# if unset
 *
 * Never overrides an ethaddr already present in the environment: a value the
 * user (or a provisioning step) saved with `setenv ethaddr` wins over anything
 * derived here. That saved value persists in uboot.env on the boot media's
 * EFI/FAT partition (CONFIG_ENV_IS_IN_FAT + ENV_FAT_DEVICE_AND_PART=":auto",
 * u-boot 3874b868b08). It was formerly a raw QSPI NOR offset
 * (CONFIG_ENV_IS_IN_SPI_FLASH at ENV_OFFSET=0x200000), which violated
 * board-bootloader policy §4 -- do not describe it that way again.
 *
 * The 64-bit hardware ID is SHA-256'd and the first 6 bytes taken, so the MAC
 * is a stable function of the unit but does not leak the flash serial. We own
 * no OUI, so the address is marked locally-administered (bit 1 of byte 0 set)
 * and unicast (bit 0 cleared) — the same treatment our RK3328 boards give
 * their cpuid-derived MAC (arch/arm/mach-rockchip/board.c).
 */
static int adrv9009_setup_identity(void)
{
	u8 hash[SHA256_SUM_LEN];
	u8 uid[ADRV9009_SPINOR_RDUID_LEN];
	u8 mac[ARP_HLEN];
	char serial[ADRV9009_SERIAL_HEX_BYTES * 2 + 1];
	const char *src;
	int ret, i;

	/* Nothing to derive if both identities are already provisioned. */
	if (env_get("ethaddr") && env_get("serial#"))
		return 0;

	src = "adrv9009";
	ret = adrv9009_transceiver_unique_id(uid, sizeof(uid));
	if (ret) {
		src = "qspi-uid";
		ret = adrv9009_qspi_unique_id(uid, sizeof(uid));
	}
	if (ret) {
		/*
		 * Deliberately do nothing about the MAC: with
		 * CONFIG_NET_RANDOM_ETHADDR set, net/eth-uclass.c will call
		 * net_random_ethaddr(). A MAC that changes across boots is the
		 * documented failure signal for this chain. serial# is likewise
		 * left unset rather than filled with a fabricated value.
		 */
		printf("ADRV9009: no hardware unique ID (%d), MAC will be random\n",
		       ret);
		return 0;
	}

	sha256_csum_wd(uid, sizeof(uid), hash, CHUNKSZ_SHA256);

	/*
	 * serial# -- board-bootloader policy §1 "MAC & serial number", and
	 * /lbs §7. Derived from the SAME hash as the MAC, from the SAME W25Q256
	 * factory UID, so the two identities cannot disagree and neither needs
	 * per-unit provisioning.
	 *
	 * The hash, not the raw UID: the raw 64-bit value is the flash's own
	 * factory serial and printing it would leak a part serial into every
	 * SMBIOS table and USB descriptor. The first 8 hash bytes give a 16-hex
	 * -character serial, and because it is a stable function of the unit it
	 * is still a genuine per-unit identity.
	 *
	 * Bytes 0-5 of the same hash become the MAC below, so the serial and the
	 * MAC visibly share a prefix. That is intentional and harmless -- both
	 * are public identifiers of the same unit -- and it makes a mismatched
	 * pair (a sign someone provisioned one by hand) immediately obvious.
	 *
	 * lib/smbios.c picks this up as the SMBIOS type-1 serial number and UUID
	 * (it reads env "serial#" directly, smbios.c:528), and g_dnl exports it
	 * as the USB gadget iSerialNumber, so setting it here closes both.
	 */
	if (!env_get("serial#")) {
		for (i = 0; i < ADRV9009_SERIAL_HEX_BYTES; i++)
			sprintf(&serial[i * 2], "%02x", hash[i]);
		serial[ADRV9009_SERIAL_HEX_BYTES * 2] = '\0';
		env_set("serial#", serial);
	}

	if (env_get("ethaddr"))
		return 0;

	memcpy(mac, hash, ARP_HLEN);

	mac[0] &= 0xfe;	/* clear multicast bit */
	mac[0] |= 0x02;	/* set locally-administered bit (IEEE 802) */

	debug("ADRV9009: identity from %s\n", src);

	return eth_env_set_enetaddr("ethaddr", mac);
}

#if IS_ENABLED(CONFIG_BOARD_CONFIG)
/*
 * NOR config-sector backend (boot/board-config.c), ported from the sibling
 * zynq_e310.
 *
 * Unlike the Amlogic backend, which pokes the SPIFC registers bare-metal because
 * it also runs pre-relocation from fdtdec_board_setup(), this board only uses the
 * environment-override half, which runs from misc_init_r() with driver model
 * fully up. So it goes through the DM SPI-flash API and inherits ZYNQ_QSPI plus
 * the whole spi-nor probe (4-byte addressing, the W25Q256 entry in spi_nor_ids)
 * for free. CONFIG_BOARD_CONFIG_OVERLAYS is off, so fdtdec_board_setup() is never
 * installed and there is no pre-relocation path to serve.
 *
 * board_nor_clk_enabled() is deliberately not implemented: the weak default
 * returns true, and on Zynq the QSPI reference clock is already up (ps7_init ran
 * in the SPL, and the BootROM loaded that SPL from this very device).
 */
static struct udevice *adrv9009_nor_dev;

bool board_nor_init(void)
{
	int ret;

	if (adrv9009_nor_dev)
		return true;

	ret = spi_flash_probe_bus_cs(CONFIG_SF_DEFAULT_BUS,
				     CONFIG_SF_DEFAULT_CS, &adrv9009_nor_dev);
	if (ret) {
		debug("adrv9009: QSPI flash probe failed (%d)\n", ret);
		adrv9009_nor_dev = NULL;
		return false;
	}

	return true;
}

bool board_nor_probe(void)
{
	/*
	 * spi_flash_probe_bus_cs() already read the JEDEC ID and matched it
	 * against spi_nor_ids; a device handle is proof a flash answered.
	 */
	return adrv9009_nor_dev != NULL;
}

int board_nor_read(u32 offset, void *buf, u32 len)
{
	if (!adrv9009_nor_dev)
		return -ENODEV;

	return spi_flash_read_dm(adrv9009_nor_dev, offset, len, buf);
}
#endif /* CONFIG_BOARD_CONFIG */

int misc_init_r(void)
{
	int ret;

	ret = adrv9009_setup_identity();

	/*
	 * Apply the NOR config sector last, so its key=value overrides beat both
	 * the compiled-in defaults and the saved uboot.env -- including an
	 * ethaddr or serial#, if someone provisions one there.
	 *
	 * misc_init_r() rather than board_late_init(): board_r.c runs
	 * initr_env() (env loaded) -> misc_init_r() -> board_late_init(), so
	 * either would see a loaded environment, but board_late_init() on Zynq
	 * belongs to the shared board/xilinx/zynq/board.c and returns early on
	 * several paths.
	 */
	board_config_env_import();

	return ret;
}

/*
 * QSPI NOR (Winbond W25Q256, 32 MiB) DFU alternate-setting map.
 *
 * The boundaries are the MEASURED flash content, not the DT partition list --
 * which on this board describes something the flash contradicts. Ground truth:
 * rf/rfsim/reference/adrv9009-qspi-nor-contents.md in the claude repo, a
 * byte-for-byte read of the live device adjudicated against the vendor BOOT.BIN.
 *
 *   0x000000 +1984 KiB    bootloader SPL at 0 (the BootROM's boot-header search
 *                                    starts there) plus the compressed FIT at
 *                                    CONFIG_SYS_SPI_U_BOOT_OFFS (0x40000)
 *                                    *within* this same region, so a firmware
 *                                    update is one erase+write of one alt and
 *                                    can never leave a valid SPL paired with a
 *                                    stale payload.
 *   0x1F0000 + 64 KiB     config     config sector, per policy §4: one erase
 *                                    block of key=value env *overrides* (the
 *                                    env proper is uboot.env on the ESP,
 *                                    ENV_IS_IN_FAT). The POLICY DEFAULT offset.
 *   0x200000 +256 KiB     bootscript the QSPI boot-script slot the bootstd
 *                                    `qspi` target reads (script_offset_f /
 *                                    script_size_f, defaulted in
 *                                    board/xilinx/zynq/zynq-adrv9009.env). NOT
 *                                    offered as a DFU alt: it is boot DATA, not
 *                                    firmware, and it is written with `sf write`
 *                                    or from Linux mtd. Listed here so the map
 *                                    and the env can be checked against each
 *                                    other in one place.
 *
 * Both bootloader and config boundaries moved on 2026-07-31 (0x2f0000/0x2f0000
 * -> 0x1f0000/0x1f0000) when the FIT moved from the legacy 0x120000 to its
 * derived floor 0x40000 and BL33 gained LZ4 compression. See
 * arch/arm/dts/zynq-sdr-nor-u-boot.dtsi and configs/zynq_adrv9009_defconfig.
 *
 * THE MAP DELIBERATELY STOPS AT 0x300000 AND DOES NOT TILE THE DEVICE. Two
 * independent reasons, and each on its own is disqualifying:
 *
 *  1. 0x300000-0x139C0A0 is the stock 17 416 352 B PL bitstream. Exposing it as
 *     an alt invites a partial write, and a truncated bitstream is a broken
 *     bitstream -- there is no benefit to a recovery path that can only half
 *     work.
 *  2. U-Boot proper on this board has NO 4-byte addressing. The device measures
 *     SR3 = 0x60 (ADS = 0, ADP = 0) at the U-Boot prompt AFTER
 *     "SF: Detected w25q256", and CONFIG_SPI_FLASH_BAR is off in U-Boot proper.
 *     So ANY alt at or above 0x1000000 (16 MiB) would truncate: an earlier
 *     revision of this map offered "config raw 0x1400000" and "spare raw
 *     0x1410000", which would have resolved to 0x400000 and 0x410000 -- both
 *     inside the bitstream. A DFU write to the "spare" alt would have destroyed
 *     PL configuration data while reporting success.
 *
 * An alt that cannot be reached is worse than an absent one, so the high 16 MiB
 * is not offered at all. Recovery of the bitstream is a Linux-side mtd operation
 * against the vendor BOOT.BIN, which is where it already lives.
 *
 * This also deliberately does NOT reproduce the DT's five stock Xilinx
 * partitions (boot/bootenv/config/image/spare): a write aimed at the declared
 * "bootenv" (0x500000) or "config" (0x520000) lands inside the bitstream. That
 * partition list is removed from the DT for the same reason.
 */
#define ADRV9009_DFU_SF_ALTS \
	"bootloader raw 0x0 0x1f0000;" \
	"config raw 0x1f0000 0x10000"

#if IS_ENABLED(CONFIG_EFI_HAVE_CAPSULE_SUPPORT)
/*
 * EFI capsule firmware update (board-bootloader policy §6): the FMP target is
 * this board's SPI NOR, and it reuses the DFU alt map above verbatim -- the raw
 * FMP backend (lib/efi_loader/efi_firmware.c efi_firmware_raw_set_image()) does
 * exactly `env_set("dfu_alt_info", update_info.dfu_string)` then
 * `dfu_write_by_alt(image_index - 1, ...)`, so image_index 1 resolves to alt 0,
 * "bootloader raw 0x0 0x1f0000".
 *
 * Sharing the string with set_dfu_alt_info() is the point, not a shortcut: it
 * is what makes the capsule write span provably identical to the one the DFU
 * recovery path already uses, and therefore already reasoned about. It starts
 * at 0 because the flashed artifact is ONE binman blob (u-boot-zynq-nor.bin =
 * spl/boot.bin at 0 + the FIT at CONFIG_SYS_SPI_U_BOOT_OFFS), and it stops
 * at CONFIG_BOARD_CONFIG_NOR_OFFSET (0x1F0000) -- which is also the effective
 * LBS_TARGET_MAX_SIZE for configs/xil-z7100-sdr (inherited from
 * configs/xilinx-zynq7), so a blob that would overrun the alt fails the build
 * instead of overrunning the flash. 0x1F0000 is below the 17 416 352 B stock PL
 * bitstream at 0x300000-0x139C0A0, so no capsule this FMP can apply is able to
 * reach the bitstream.
 *
 * image_type_id is deliberately left ZERO. lib/efi_loader/efi_firmware.c
 * efi_gen_capsule_guids() then derives it as a UUIDv5 of
 * (CONFIG_EFI_CAPSULE_NAMESPACE_GUID, DT root compatible, fw_name) -- i.e. from
 * "hamgeek,xc7z100-adrv9009" -- which is what policy §6 asks for and what the
 * Amlogic/sunxi boards in this tree do. Naming it XILINX_BOOT_IMAGE_GUID
 * instead (the tree-wide constant in include/configs/zynq-common.h, which is
 * why board/xilinx/common/board.c's definition is #ifdef'd out for this board)
 * would give every Zynq board the same FMP identity, so a capsule built for one
 * would be accepted by another.
 *
 * fw_name follows the convention the capsule build script uses -- the board
 * slug uppercased with '-' -> '_', suffixed _BOOT.
 *
 * NOTE for whoever applies one: a capsule write starts at offset 0, so the SPL
 * is rewritten BEFORE the payload. An interrupted apply can therefore leave a
 * good SPL with a torn u-boot.img FIT, which halts at "Trying to boot from
 * SPI". That is inherent to a single-blob FMP and is recoverable only over
 * JTAG; there is no A/B bank on this flash and adding one would collide with
 * the bitstream.
 */
static struct efi_fw_image adrv9009_fw_images[] = {
	{
		.fw_name = u"XC7Z100_ADRV9009_BOOT",
		.image_index = 1,
	},
};

struct efi_capsule_update_info update_info = {
	.dfu_string = "sf 0:0=" ADRV9009_DFU_SF_ALTS,
	.num_images = ARRAY_SIZE(adrv9009_fw_images),
	.images = adrv9009_fw_images,
};
#endif /* CONFIG_EFI_HAVE_CAPSULE_SUPPORT */

#ifdef CONFIG_SET_DFU_ALT_INFO

/**
 * set_dfu_alt_info() - build dfu_alt_info for the QSPI NOR
 *
 * Why a compiled-in board hook rather than an environment default (identical
 * reasoning to zynq_e310, from which this is ported):
 *
 *  1. It survives a blank environment. The env lives on the ESP FAT as
 *     uboot.env (CONFIG_ENV_IS_IN_FAT), and a missing or corrupt env is exactly
 *     the situation in which someone reaches for DFU recovery. A map that only
 *     exists once the env has loaded is absent when it is needed most.
 *  2. It cannot be clobbered. zynq-common.h's boot targets bootcmd_usb_dfu0/1
 *     (and usb_thor0/1) do `setenv dfu_alt_info boot.scr ram $scriptaddr
 *     $script_size_f` during a bootstd scan -- that overwrite is permanent for
 *     the session and would destroy a CFG_EXTRA_ENV_SETTINGS default. This hook
 *     runs inside dfu_init_env_entities() on every `dfu` invocation, so the
 *     correct map is re-established each time.
 *
 * Keyed on @interface so the zynq-common boot.scr path is untouched: only "sf"
 * (and the bare no-interface form) is claimed here; `dfu <n> ram <n>` falls
 * through to whatever bootcmd_usb_dfu* already put in the environment.
 */
void set_dfu_alt_info(char *interface, char *devstr)
{
	debug("%s: interface %s devstr %s\n", __func__,
	      interface ? interface : "(null)", devstr ? devstr : "(null)");

	if (!interface) {
		/* Bare `dfu <n>`: multi-interface form, "<if> <dev>=<alts>". */
		env_set("dfu_alt_info", "sf 0:0=" ADRV9009_DFU_SF_ALTS);
		return;
	}

	if (!strcmp(interface, "sf"))
		env_set("dfu_alt_info", ADRV9009_DFU_SF_ALTS);

	/*
	 * Any other interface (notably "ram", used by bootcmd_usb_dfu* to
	 * receive a boot.scr) is deliberately left alone.
	 */
}
#endif /* CONFIG_SET_DFU_ALT_INFO */
