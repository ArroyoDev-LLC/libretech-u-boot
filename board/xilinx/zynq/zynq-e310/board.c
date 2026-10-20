// SPDX-License-Identifier: GPL-2.0+
/*
 * HamGeek / ANTSDR E310 (Xilinx Zynq-7020 + AD9361) — board-specific code.
 *
 * Deterministic network identity (board-bootloader policy §1 "Network MAC &
 * serial number"): a board's MAC must be derived from a unique hardware ID so
 * every unit gets a stable identity with no per-unit provisioning. A random
 * MAC is only ever a last-resort *failure signal*.
 *
 * Without this file U-Boot sets no ethaddr at all, so net/eth-uclass.c falls
 * through to net_random_ethaddr() and the board comes up with a different MAC
 * on every single boot (measured on the sibling unit: 9e:37:1e:f1:85:f6,
 * 16:9e:b1:3d:41:d8, da:5c:6c:2e:e8:f7 on three consecutive boots).
 *
 * Fallback chain, highest priority first:
 *
 *   1. AD9361/AD9363 transceiver unique ID — STUB, always "unsupported".
 *      See e310_ad9361_unique_id() below; the part has no per-device ID.
 *   2. QSPI NOR factory unique ID (Winbond W25Q256, opcode 0x4B) — the
 *      working source. Hashed to 6 bytes, marked locally-administered.
 *   3. U-Boot's own net_random_ethaddr() — left in place, untouched, as the
 *      final rung. A unit that comes up with a MAC that changes across boots
 *      means rungs 1 and 2 both failed.
 *
 * Note on the Zynq-7000 "Device DNA": the 57-bit Device DNA is a *PL fabric*
 * primitive (DNA_PORT), readable only through a custom AXI IP instantiated in
 * a bitstream — a PL rebuild, which is out of scope for this port. The PS-side
 * eFUSE controller at 0xF800D000 (DT node efuse@f800d000) exposes only the
 * secure-boot/AES/JTAG-disable control and status bits on Zynq-7000; unlike
 * ZynqMP it has no per-device unique ID field. So there is no PS-readable
 * SoC unique ID on this part, which is why rung 2 uses the QSPI flash.
 */

#include <dm.h>
#include <dfu.h>
#include <env.h>
#include <log.h>
#include <net.h>
#include <board-config.h>
#include <spi.h>
#include <spi-mem.h>
#include <spi_flash.h>
#include <dm/uclass.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <u-boot/sha256.h>

/* Winbond "Read Unique ID Number": 0x4B, 4 dummy bytes, 8 data bytes. */
#define E310_SPINOR_OP_RDUID		0x4b
#define E310_SPINOR_RDUID_DUMMY		4
#define E310_SPINOR_RDUID_LEN		8

/* JEDEC "Read Identification": 0x9F, 3 data bytes (mfr, type, capacity). */
#define E310_SPINOR_OP_RDID		0x9f
#define E310_SPINOR_RDID_LEN		3
#define E310_SPINOR_MFR_WINBOND		0xef

/**
 * e310_ad9361_unique_id() - rung 1: AD9361/AD9363 per-device unique ID
 *
 * STUB — always fails, by design.
 *
 * Analog Devices confirm the AD936x family carries NO per-device unique
 * identifier. The only ID-looking register, 0x037, is a *common* product ID
 * shared by every part of the same die revision, so it is worthless as a
 * source of per-unit identity. There is no OTP/serial region either.
 *
 * The rung is kept so the priority order of the chain is explicit in the code
 * and so a future part (or a future ADI erratum saying otherwise) has an
 * obvious place to land. Do NOT "implement" this by reading 0x037 — that
 * would give every E310 in the fleet the same MAC, which is strictly worse
 * than the random fallback.
 *
 * Return: always -ENOTSUPP (fall through to rung 2).
 */
static int e310_ad9361_unique_id(u8 *uid, size_t len)
{
	return -ENOTSUPP;
}

/**
 * e310_qspi_unique_id() - rung 2: QSPI NOR factory 64-bit unique ID
 * @uid: caller buffer, E310_SPINOR_RDUID_LEN bytes
 *
 * The E310 QSPI flash is a Winbond W25Q256 (JEDEC ef 40 19, confirmed on the
 * target). Winbond parts carry a factory-programmed, read-only 64-bit unique
 * serial number readable with opcode 0x4B. It is per-die, so it is a genuine
 * per-unit hardware ID.
 *
 * The JEDEC ID is read first and the manufacturer byte checked against
 * Winbond: 0x4B is a vendor extension, not a JEDEC-standard opcode, so on a
 * non-Winbond part it may mean something entirely different (or nothing).
 * Better to fall through to the random rung than to hash garbage.
 *
 * Return: 0 on success, negative errno otherwise.
 */
static int e310_qspi_unique_id(u8 *uid, size_t len)
{
	u8 id[E310_SPINOR_RDID_LEN];
	struct spi_slave *slave;
	struct udevice *dev;
	int ret, i;

	if (len < E310_SPINOR_RDUID_LEN)
		return -EINVAL;

	ret = uclass_first_device_err(UCLASS_SPI_FLASH, &dev);
	if (ret) {
		debug("e310: no SPI flash device (%d)\n", ret);
		return ret;
	}

	slave = dev_get_parent_priv(dev);
	if (!slave)
		return -ENODEV;

	struct spi_mem_op rdid = SPI_MEM_OP(SPI_MEM_OP_CMD(E310_SPINOR_OP_RDID, 1),
					    SPI_MEM_OP_NO_ADDR,
					    SPI_MEM_OP_NO_DUMMY,
					    SPI_MEM_OP_DATA_IN(sizeof(id), id, 1));

	struct spi_mem_op rduid = SPI_MEM_OP(SPI_MEM_OP_CMD(E310_SPINOR_OP_RDUID, 1),
					     SPI_MEM_OP_NO_ADDR,
					     SPI_MEM_OP_DUMMY(E310_SPINOR_RDUID_DUMMY, 1),
					     SPI_MEM_OP_DATA_IN(E310_SPINOR_RDUID_LEN,
								uid, 1));

	ret = spi_mem_exec_op(slave, &rdid);
	if (ret) {
		debug("e310: SPI NOR RDID failed (%d)\n", ret);
		return ret;
	}

	if (id[0] != E310_SPINOR_MFR_WINBOND) {
		debug("e310: SPI NOR mfr 0x%02x is not Winbond, no unique ID\n",
		      id[0]);
		return -ENOTSUPP;
	}

	ret = spi_mem_exec_op(slave, &rduid);
	if (ret) {
		debug("e310: SPI NOR RDUID failed (%d)\n", ret);
		return ret;
	}

	/* An all-0x00 or all-0xff answer means the part did not respond. */
	for (i = 0; i < E310_SPINOR_RDUID_LEN; i++)
		if (uid[i] != 0x00 && uid[i] != 0xff)
			return 0;

	debug("e310: SPI NOR unique ID is degenerate, rejecting\n");
	return -ENODATA;
}

/**
 * e310_setup_ethaddr() - install a deterministic ethaddr if none is set
 *
 * Never overrides an ethaddr already present in the environment: a value the
 * user (or a provisioning step) saved with `setenv ethaddr` wins over anything
 * derived here.
 *
 * The 64-bit hardware ID is SHA-256'd and the first 6 bytes taken, so the MAC
 * is a stable function of the unit but does not leak the flash serial. We own
 * no OUI, so the address is marked locally-administered (bit 1 of byte 0 set)
 * and unicast (bit 0 cleared) — the same treatment our RK3328 boards give
 * their cpuid-derived MAC (arch/arm/mach-rockchip/board.c).
 */
static int e310_setup_ethaddr(void)
{
	u8 hash[SHA256_SUM_LEN];
	u8 uid[E310_SPINOR_RDUID_LEN];
	u8 mac[ARP_HLEN];
	const char *src;
	int ret;

	if (env_get("ethaddr"))
		return 0;

	src = "ad9361";
	ret = e310_ad9361_unique_id(uid, sizeof(uid));
	if (ret) {
		src = "qspi-uid";
		ret = e310_qspi_unique_id(uid, sizeof(uid));
	}
	if (ret) {
		/*
		 * Deliberately do nothing: net/eth-uclass.c will call
		 * net_random_ethaddr(). A MAC that changes across boots is the
		 * documented failure signal for this chain.
		 */
		printf("E310: no hardware unique ID (%d), MAC will be random\n",
		       ret);
		return 0;
	}

	sha256_csum_wd(uid, sizeof(uid), hash, CHUNKSZ_SHA256);
	memcpy(mac, hash, ARP_HLEN);

	mac[0] &= 0xfe;	/* clear multicast bit */
	mac[0] |= 0x02;	/* set locally-administered bit (IEEE 802) */

	debug("E310: ethaddr from %s\n", src);

	return eth_env_set_enetaddr("ethaddr", mac);
}

#if IS_ENABLED(CONFIG_BOARD_CONFIG)
/*
 * NOR config-sector backend (boot/board-config.c).
 *
 * Unlike the Amlogic backend, which pokes the SPIFC registers bare-metal
 * because it also runs pre-relocation from fdtdec_board_setup(), the E310
 * only uses the environment-override half, which runs from misc_init_r() with
 * driver model fully up. So this goes through the DM SPI-flash API and
 * inherits ZYNQ_QSPI plus the whole spi-nor probe (4-byte addressing, the
 * W25Q256 entry in spi_nor_ids) for free.
 *
 * board_nor_clk_enabled() is deliberately not implemented: the weak default
 * returns true, and on Zynq the QSPI reference clock is already up (ps7_init
 * ran in the SPL, and the BootROM loaded that SPL from this very device).
 */
static struct udevice *e310_nor_dev;

bool board_nor_init(void)
{
	int ret;

	if (e310_nor_dev)
		return true;

	ret = spi_flash_probe_bus_cs(CONFIG_SF_DEFAULT_BUS,
				     CONFIG_SF_DEFAULT_CS, &e310_nor_dev);
	if (ret) {
		debug("e310: QSPI flash probe failed (%d)\n", ret);
		e310_nor_dev = NULL;
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
	return e310_nor_dev != NULL;
}

int board_nor_read(u32 offset, void *buf, u32 len)
{
	if (!e310_nor_dev)
		return -ENODEV;

	return spi_flash_read_dm(e310_nor_dev, offset, len, buf);
}
#endif /* CONFIG_BOARD_CONFIG */

int misc_init_r(void)
{
	int ret;

	ret = e310_setup_ethaddr();

	/*
	 * Apply the NOR config sector last, so its key=value overrides beat
	 * both the compiled-in defaults and the saved uboot.env -- including
	 * an ethaddr, if someone provisions one there.
	 *
	 * misc_init_r() rather than board_late_init(): board_r.c runs
	 * initr_env() (env loaded) -> misc_init_r() -> board_late_init(), so
	 * either would see a loaded environment, but board_late_init() on
	 * Zynq belongs to the shared board/xilinx/zynq/board.c and returns
	 * early on several paths.
	 */
	board_config_env_import();

	return ret;
}

#ifdef CONFIG_SET_DFU_ALT_INFO
/*
 * QSPI NOR (Winbond W25Q256, 32 MiB) DFU alternate-setting map.
 *
 * Libre Computer NOR-board layout: the bootloader is ONE artifact, so there
 * is exactly one alt to write for a firmware update and no way to end up
 * with a valid SPL paired with a stale or missing U-Boot payload.
 *
 *   0x000000 +1984 KiB  bootloader  combined SPL + u-boot.img blob, built by
 *                                   binman as u-boot-e310-nor.bin (see
 *                                   arch/arm/dts/zynq-e310-u-boot.dtsi). The
 *                                   BootROM loads the SPL from offset 0; the
 *                                   SPL loads the payload from
 *                                   CONFIG_SYS_SPI_U_BOOT_OFFS *within* this
 *                                   same region.
 *   0x1F0000 +  64 KiB  config      config sector, per the board-bootloader
 *                                   policy §4: one erase block of key=value
 *                                   env *overrides*. The env proper is never
 *                                   in NOR (ours is uboot.env on the ESP,
 *                                   ENV_IS_IN_FAT). Read at misc_init_r() by
 *                                   board_config_env_import() (CONFIG_BOARD_CONFIG).
 *   0x200000 +  30 MiB  spare       spare. Holds a golden copy of the
 *                                   bootloader blob. NOT a DT overlay FIT --
 *                                   which is why CONFIG_BOARD_CONFIG_OVERLAYS
 *                                   is off for this board; its default FIT
 *                                   offset is exactly 0x200000.
 *
 * The three alts tile the whole 32 MiB device with no gaps and no overlap.
 */
#define E310_DFU_SF_ALTS \
	"bootloader raw 0x0 0x1f0000;" \
	"config raw 0x1f0000 0x10000;" \
	"spare raw 0x200000 0x1e00000"

/**
 * set_dfu_alt_info() - build dfu_alt_info for the QSPI NOR
 *
 * Why a compiled-in board hook rather than an environment default:
 *
 *  1. It survives a blank environment. Our env lives on the ESP FAT as
 *     uboot.env (CONFIG_ENV_IS_IN_FAT), and a missing/corrupt env is exactly
 *     the situation in which someone reaches for DFU recovery. A map that
 *     only exists once the env has loaded is absent when it is needed most.
 *  2. It cannot be clobbered. zynq-common.h's distro boot targets
 *     bootcmd_usb_dfu0/1 (and usb_thor0/1) do `setenv dfu_alt_info boot.scr
 *     ram $scriptaddr $script_size_f` during a bootstd scan — that overwrite
 *     is permanent for the session and would destroy a CFG_EXTRA_ENV_SETTINGS
 *     default. This hook runs inside dfu_init_env_entities() on every `dfu`
 *     invocation, so the correct map is re-established each time.
 *
 * Keyed on @interface so the zynq-common boot.scr path is untouched: only
 * "sf" (and the bare no-interface form) is claimed here; `dfu <n> ram <n>`
 * falls through to whatever bootcmd_usb_dfu* already put in the environment.
 */
void set_dfu_alt_info(char *interface, char *devstr)
{
	debug("%s: interface %s devstr %s\n", __func__,
	      interface ? interface : "(null)", devstr ? devstr : "(null)");

	if (!interface) {
		/* Bare `dfu <n>`: multi-interface form, "<if> <dev>=<alts>". */
		env_set("dfu_alt_info", "sf 0:0=" E310_DFU_SF_ALTS);
		return;
	}

	if (!strcmp(interface, "sf"))
		env_set("dfu_alt_info", E310_DFU_SF_ALTS);

	/*
	 * Any other interface (notably "ram", used by bootcmd_usb_dfu* to
	 * receive a boot.scr) is deliberately left alone.
	 */
}
#endif /* CONFIG_SET_DFU_ALT_INFO */
