// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2025 Da Xue <da@libre.computer>
 */

#include <asm/arch/boot.h>
#include <asm/arch/gx.h>
#include <asm/arch/sm.h>
#include <asm/arch/eth.h>
#include <asm/arch/usbflash-alts.h>
#include <usbflash-proto.h>
#include <command.h>
#include <dfu.h>
#include <dm/ofnode.h>
#include <asm/io.h>
#include <board-config.h>
#include <bootm.h>
#include <bootsource.h>
#include <chip_serial.h>
#include <cyclic.h>
#include <dm.h>
#include <env.h>
#include <env_internal.h>
#include <ini.h>
#include <linux/stringify.h>
#include <malloc.h>
#if CONFIG_IS_ENABLED(MMC)
#include <mmc.h>
#endif
#include <net.h>
#include <spi_flash.h>
#include <vsprintf.h>
#include "board.h"
#include "board-meson.h"

#define EFUSE_MAC_OFFSET	52
#define EFUSE_MAC_SIZE		6

bool meson_usb_env_nowhere(void)
{
	return IS_ENABLED(CONFIG_ENV_IS_NOWHERE) &&
	       meson_get_boot_device() == BOOT_DEVICE_USB;
}

/*
 * Where the environment lives, by boot source (same rule as board-rockchip.c).
 *
 * USB: the stage was sent over the cable by a host (BootROM download); it is a
 * recovery or flash-session boot that must run on its compiled-in defaults
 * (plus the `usbflash` pair meson_usbflash_env() sets below), not on whatever
 * `uboot.env` sits on a medium that happens to be present. ENVL_NOWHERE also
 * makes `saveenv` refuse, so a session cannot leave its variables behind.
 *
 * Every other boot source keeps the default ladder (arch_env_get_location())
 * with ENVL_NOWHERE removed, so the list is exactly what it was before
 * CONFIG_ENV_IS_NOWHERE was enabled (FAT only on these boards). Left in,
 * NOWHERE would be the last entry of env_locations[], load "OK" when the FAT
 * load fails, and leave env_load_prio at 1, where saveenv is "not possible".
 */
#if IS_ENABLED(CONFIG_ENV_IS_NOWHERE)
enum env_location env_get_location(enum env_operation op, int prio)
{
	enum env_location loc;

	if (meson_get_boot_device() == BOOT_DEVICE_USB)
		return prio ? ENVL_UNKNOWN : ENVL_NOWHERE;

	loc = arch_env_get_location(op, prio);

	return loc == ENVL_NOWHERE ? ENVL_UNKNOWN : loc;
}

/*
 * The same USB recovery / flash-session boot must not act on the NOR config
 * block's `overlays=` either: board_config_nor_overlays() runs from the
 * generic fdtdec_board_setup(), before DM, with no board code in between, so
 * this hook is the only place the board can say so. The SPIFC clock gate is
 * no guard -- it is already ON at a USB boot of SM1. The env half is gated
 * at its board caller (misc_init_r()), under the same rule. Only compiled
 * where the environment can be NOWHERE; every other build links the __weak
 * "never skip" default.
 */
bool board_config_nor_skip(void)
{
	return meson_usb_env_nowhere();
}

/*
 * The host-driven flash session, set only on a USB boot: `usbflash`, its
 * timeout, and bootcmd_usbdfu (which PREBOOT runs after board_late_init) pointed at
 * it instead of the plain `dfu 0 60` of include/configs/meson64.h, which stays
 * untouched so every other build is unchanged. It is not in boot.env: that
 * file is compiled into every LC defconfig and its variables land in the
 * default environment of normal boots, where `saveenv` would persist them
 * into a uboot.env that a later USB boot of another build (no NOWHERE
 * override) would load. Here they exist only while the environment is
 * NOWHERE, so they can never be saved.
 *
 * Host protocol: DFU DNLOAD to alt `script` one `usbflash ...` line (plan
 * section 4, the `usbflash` command below; e.g. `usbflash ums mmc 0 ro`),
 * then DFU DETACH: dfu returns, the loop runs the step the command recorded
 * in usbflash_next, and enters dfu again when it returns. A `usbflash` line
 * the protocol does not accept is refused at the manifest step
 * (dfu_board_manifest_check()). The pre-protocol form
 * `setenv usbflash_next 'ums 0 mmc 0'` is still accepted, unchanged. A DETACH
 * with usbflash_next empty falls through to autoboot. When the DRAM-stage SPL
 * already served this boot's session, none runs here (meson_usbflash_env()).
 */

/*
 * USB_GADGET_DNL_ACM: each `dfu` of the session also carries a CDC ACM
 * function, whose console is the stdio device `dnlacm`. Add it beside the
 * existing devices so the host sees the session's console on /dev/ttyACM*;
 * between DFU sessions its output is kept in a bounded buffer and reaches
 * the next session's ttyACM. Only on this USB boot (the environment is
 * NOWHERE, so it cannot be saved); every other boot keeps its console list
 * unchanged.
 */
static void meson_usbflash_console(void)
{
	static const char * const names[] = { "stdout", "stderr", "stdin" };
	const char *cur;
	char *buf;
	size_t len;
	int i;

	if (!IS_ENABLED(CONFIG_USB_GADGET_DNL_ACM))
		return;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		cur = env_get(names[i]);
		if (!cur || strstr(cur, "dnlacm"))
			continue;
		/*
		 * Sized to the list: a fixed buffer cut a long one short and
		 * set a console list naming a device that does not exist.
		 */
		len = strlen(cur) + sizeof(",dnlacm");
		buf = malloc(len);
		if (!buf) {
			printf("usbflash: no memory to add dnlacm to %s, left as '%s'\n",
			       names[i], cur);
			continue;
		}
		snprintf(buf, len, "%s,dnlacm", cur);
		env_set(names[i], buf);
		free(buf);
	}
}

/*
 * The flash-session protocol line (<usbflash-proto.h>, plan section
 * 4), as U-Boot proper carries it out.  `usbflash <verb> ...` is a command,
 * so the host's DNLOAD of the line to the `script` alt runs it through the
 * script entity, and it records the step in usbflash_next for the session
 * loop below.  The value is built here from the parsed verb, never copied
 * from the host's bytes.  mmc <n> is 0 (eMMC) or 1 (SD), proper's numbering.
 */
static int meson_usbflash_cmd(const char *line, size_t len,
			      struct usbflash_cmd *cmd)
{
	int ret = usbflash_parse(line, len, cmd);

	if (!ret && cmd->verb == USBFLASH_UMS && cmd->mmc > 1)
		ret = -EINVAL;

	return ret;
}

static int do_usbflash(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	struct usbflash_cmd cmd;
	char line[USBFLASH_LINE_MAX], next[32];
	int i, n;

	/* Re-join the words the shell split, then parse as the SPL does */
	n = snprintf(line, sizeof(line), "%s", argv[0]);
	for (i = 1; i < argc && n < sizeof(line); i++)
		n += snprintf(line + n, sizeof(line) - n, " %s", argv[i]);
	if (n >= sizeof(line) || meson_usbflash_cmd(line, n, &cmd)) {
		printf("usbflash: refused\n");
		return CMD_RET_USAGE;
	}

	switch (cmd.verb) {
	case USBFLASH_UMS:
		snprintf(next, sizeof(next), "ums%s 0 mmc %d",
			 cmd.ro ? " -r" : "", cmd.mmc);
		env_set("usbflash_next", next);
		break;
	case USBFLASH_DFU:
		env_set("usbflash_next", "echo usbflash: DFU again");
		break;
	case USBFLASH_BOOT:
		env_set("usbflash_next", NULL);
		break;
	case USBFLASH_BOOT_KERNEL:
		/*
		 * proper's own `kernel` alt is the kernel slot at kernel_addr_r;
		 * the device tree is U-Boot's own (common/usbflash_bootm.c)
		 */
		env_set("usbflash_next",
			"bootm ${kernel_addr_r} ${kernel_addr_r} ${fdtcontroladdr}");
		break;
	case USBFLASH_RESET:
		env_set("usbflash_next", "reset");
		break;
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(usbflash, 6, 0, do_usbflash,
	   "flash-session step, for the next DFU DETACH",
	   "ums mmc <n> [ro] | dfu | boot [kernel] | reset");

/*
 * Refuse a `usbflash` line the protocol does not accept at the DFU manifest
 * step, so the host's GETSTATUS reads errTARGET and the session stays in DFU.
 * Other text on `script` is the pre-protocol form (`setenv usbflash_next
 * ...`, `echo`, `md`), which the hosts still send and which runs as before.
 */
int dfu_board_manifest_check(struct dfu_entity *dfu, const void *buf,
			     long len, u64 written)
{
	struct usbflash_cmd cmd;
	const char *s = buf;
	const size_t w = sizeof("usbflash") - 1;

	if (strcmp(dfu->name, "script") || len < (long)w || memcmp(s, "usbflash", w))
		return 0;
	if (len > (long)w && s[w] != ' ' && s[w] != '\n' && s[w] != '\r')
		return 0;	/* a longer word, e.g. usbflash_next */
	if (written)
		return -EFBIG;

	return meson_usbflash_cmd(s, len, &cmd);
}

/*
 * Did the DRAM-stage SPL already serve this USB boot's flash session?  It
 * marks the control DTB it hands on with an empty
 * /chosen/u-boot,spl-usbflash (arch/arm/mach-meson/spl-usbflash.c; plan
 * section 5, "one session per boot").  The DTB arrives with the FIT on
 * every boot, so the mark describes this boot only.
 */
static bool meson_usbflash_spl_ran(void)
{
	return ofnode_read_bool(ofnode_path("/chosen"), USBFLASH_FDT_PROP);
}

static void meson_usbflash_env(void)
{
	if (meson_get_boot_device() != BOOT_DEVICE_USB)
		return;

	if (meson_usbflash_spl_ran()) {
		/* bootcmd_usbdfu would otherwise stay meson64.h's `dfu 0 60` */
		puts("usbflash: the SPL served this boot's session; none here\n");
		env_set("bootcmd_usbdfu",
			"echo usbflash: SPL session ran, no session here");
		return;
	}

	env_set("usbflash_timeout", "60");
	env_set("usbflash",
		"if test \"$bootsource\" = usb; then "
			"setenv usbflash_next; "
			"dfu 0 $usbflash_timeout; "
			"while test -n \"$usbflash_next\"; do "
				"setenv usbflash_run \"$usbflash_next\"; "
				"setenv usbflash_next; "
				"run usbflash_run; "
				"dfu 0 $usbflash_timeout; "
			"done; "
		"fi");
	env_set("bootcmd_usbdfu", "run usbflash");

	meson_usbflash_console();
}
#else
static void meson_usbflash_env(void)
{
}
#endif

/*
 * Called from board_late_init() (arch/arm/mach-meson/board-common.c), the last
 * board hook before the main loop: in common/board_r.c EVT_SETTINGS_R (where
 * bootsource is published) < misc_init_r (the NOR board-config env import, which
 * could otherwise replace the flash-session variables from a stored config
 * block) < board_late_init < run_main_loop (PREBOOT).
 */
int meson_board_late_init(void)
{
	meson_usbflash_env();

	return 0;
}

#ifdef CONFIG_ENV_MMC_DEVICE_INDEX
int mmc_get_env_dev(void)
{
	debug("%s\n", __func__);
	int bootdevice_num = CONFIG_ENV_MMC_DEVICE_INDEX;
	switch (meson_get_boot_device()) {
		case BOOT_DEVICE_EMMC:
			break;

		case BOOT_DEVICE_SPI:
		case BOOT_DEVICE_USB:
			/* TODO: eMMC, NVME, SD, USB */
			if (!mmc_is_present(bootdevice_num))
				bootdevice_num = mmc_is_present(!bootdevice_num);
			break;
		case BOOT_DEVICE_SD:
			bootdevice_num = 1;
			break;
	}

	env_set_ulong("bootdevice", bootdevice_num);

	return bootdevice_num;
}
#endif

#if IS_ENABLED(CONFIG_SET_DFU_ALT_INFO)
#define DFU_ALT_INFO_ENV_LENGTH 1024
#define DFU_INTERFACE_SEPARATOR "&"
#ifdef CONFIG_DFU_RAM
#define MESON_DFU_RAM_ALTS \
	"load ram " __stringify(CONFIG_SYS_LOAD_ADDR) " " __stringify(CONFIG_SYS_BOOTM_LEN) ";" \
	"script script " SCRIPT_ADDR_R " 0x8000;" \
	"fdt ram " FDT_ADDR_R " " FDT_SLOT_SIZE ";" \
	"kernel ram " KERNEL_ADDR_R " " KERNEL_SLOT_SIZE ";" \
	"ramdisk ram " RAMDISK_ADDR_R " " RAMDISK_SLOT_SIZE
#define MESON_DFU_RAM "ram 0=" MESON_DFU_RAM_ALTS
#else
#define MESON_DFU_RAM
#endif
#ifdef CONFIG_DFU_SF
/*
 * SPI NOR alternate settings.
 *
 * The config alt is built at run time rather than being a literal, because the
 * offset it must name is not a constant. boot/board-config.c reads the block
 * from board_config_nor_offset() -- the last 64 KiB of the addressable device,
 * computed from the JEDEC density byte -- so a literal here would be correct
 * only for whatever part happened to be fitted when it was written. The
 * failure that produces is the worst one this facility has: `dfu` reports a
 * successful write, to bytes the reader never looks at. Both sides therefore
 * ask the same function, and cannot disagree.
 *
 * The other two boundaries are genuinely fixed and stay literal:
 *
 *   u-boot-bin  0 .. MESON_DFU_SF_UBOOT_SIZE -- the bootloader artifact. Its
 *       ceiling is the build's image-size limit (LBS_TARGET_MAX_SIZE), which
 *       is a property of the image, not of the config block; it happens to
 *       equal the config offset on a 2 MiB part and must not be tied to it.
 *   fit  MESON_DFU_SF_FIT_OFFSET .. +MESON_DFU_SF_FIT_SIZE -- the DT overlay
 *       FIT, at the fixed offset board-config.c reads it from.
 *
 * The FIT alt is clamped so that on a part large enough for the two regions to
 * meet, a FIT write cannot run into the config block.
 *
 * MESON_DFU_SF_UBOOT_SIZE, MESON_DFU_SF_FIT_OFFSET and MESON_DFU_SF_FIT_SIZE
 * are in <asm/arch/usbflash-alts.h>, shared with the SPL flash session.
 */

/* The sf device the alt list names ("sf 0:0="): SPI bus 0 (spi0 = &spifc), CS 0 */
#define MESON_DFU_SF_BUS		0
#define MESON_DFU_SF_CS			0

/*
 * The span the config alt names is the block size the reader reads --
 * CONFIG_BOARD_CONFIG_NOR_SIZE, the same symbol boot/board-config.c uses. The
 * 0 in the #else arm is never rendered: without the NOR transport cfg stays 0
 * and the config alt is omitted entirely.
 */
#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
#define MESON_DFU_SF_CFG_SIZE		CONFIG_BOARD_CONFIG_NOR_SIZE
#else
#define MESON_DFU_SF_CFG_SIZE		0
#endif

/*
 * Render the sf alt list into @buf. Returns @buf.
 *
 * The config alt is omitted entirely when NO FLASH ANSWERED -- not when the
 * size is unknown. Those are different: board_config_nor_offset() always
 * returns a usable offset (an unmeasurable part gets the last block of the
 * addressable window), so an unknown SIZE still gets an alt, and it names the
 * same bytes board-config.c reads. What is omitted is the case where the
 * controller is gated or the probe found nothing, since there is no device to
 * write to at all.
 */
static char *meson_dfu_sf_alts(char *buf, int len)
{
	u32 cfg = 0;
	u32 fit_size = MESON_DFU_SF_FIT_SIZE;
	int n = 0;

#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
	/*
	 * board_nor_size() is only valid after a probe, and this may be the
	 * first thing on this boot to touch the controller -- a USB boot never
	 * runs the config-block reader, because BL2 leaves the SPIFC clock
	 * gated.
	 *
	 * The bare reader (meson-spifc-nor.c) sets neither the NOR pin
	 * function nor the controller clock; on a NOR boot BL2 left both set.
	 * On a USB boot the BootROM leaves BOOT_4..6 / BOOT_14 on their eMMC
	 * function, so the RDID reads nothing and the config alt was missing
	 * from the first DFU session -- and present from the second on,
	 * because that session's sf entities (dfu_fill_entity_sf()) probed
	 * the DM SPI flash, whose bus applied its default pinctrl (nor_pins)
	 * and clock.  Probe that same device first, so the pins and clock are
	 * the ones the sf alts of this very list will use, and the list is
	 * the same in every session.  Without a NOR in the DT (spifc
	 * disabled, the 8-bit eMMC product) nothing is probed and the pins
	 * stay on the eMMC.
	 */
	if (CONFIG_IS_ENABLED(DM_SPI_FLASH)) {
		struct udevice *flash;

		spi_flash_probe_bus_cs(MESON_DFU_SF_BUS, MESON_DFU_SF_CS,
				       &flash);
	}
	if (board_nor_clk_enabled() && board_nor_init() && board_nor_probe())
		cfg = board_config_nor_offset();
#endif

	/*
	 * strlen() after each part rather than snprintf()'s return: this tree
	 * carries both the C99 "would have written" and the truncating
	 * conventions in different helpers, and the difference is only visible
	 * once a string is long enough to truncate -- i.e. never in testing.
	 */
	snprintf(buf, len, "u-boot-bin raw 0 0x%x;", MESON_DFU_SF_UBOOT_SIZE);
	n = strlen(buf);

	if (cfg) {
		snprintf(buf + n, len - n, "config raw 0x%x 0x%x;",
			 cfg, MESON_DFU_SF_CFG_SIZE);
		n = strlen(buf);

		if (cfg > MESON_DFU_SF_FIT_OFFSET &&
		    cfg - MESON_DFU_SF_FIT_OFFSET < fit_size)
			fit_size = cfg - MESON_DFU_SF_FIT_OFFSET;
	}

	snprintf(buf + n, len - n, "fit raw 0x%x 0x%x",
		 MESON_DFU_SF_FIT_OFFSET, fit_size);

	return buf;
}
#endif
#ifdef CONFIG_DFU_MMC
/* MESON_DFU_MMC_EMMC_ALTS / MESON_DFU_MMC_SD_ALTS: <asm/arch/usbflash-alts.h> */
#define MESON_DFU_MMC_EMMC "mmc 0=" MESON_DFU_MMC_EMMC_ALTS "&"

#ifdef MESON_DFU_MMC_SD_HIDE
#undef MESON_DFU_MMC_SD_ALTS
#define MESON_DFU_MMC_SD_ALTS
#define MESON_DFU_MMC_SD
#else
#define MESON_DFU_MMC_SD "mmc 1=" MESON_DFU_MMC_SD_ALTS "&"
#endif
#else
#define MESON_DFU_MMC_EMMC
#define MESON_DFU_MMC_SD
#endif

void meson_set_dfu_alt_info(char *interface, char *devstr)
{
	debug("%s: interface %s devstr %s\n", __func__, interface, devstr);
	char *dfu_alt_info;
	bool free_dfu_alt_info = false;
	if (interface){
#ifdef CONFIG_DFU_RAM
		if (strcmp(interface, "ram") == 0)
			dfu_alt_info = MESON_DFU_RAM_ALTS;
#endif
#ifdef CONFIG_DFU_SF
		if (strcmp(interface, "sf") == 0) {
			dfu_alt_info = calloc(1, DFU_ALT_INFO_ENV_LENGTH);
			free_dfu_alt_info = true;
			if (dfu_alt_info)
				meson_dfu_sf_alts(dfu_alt_info,
						  DFU_ALT_INFO_ENV_LENGTH);
		}
#endif
#ifdef CONFIG_DFU_MMC
		if (strcmp(interface, "mmc") == 0){
			if (!devstr){
				dfu_alt_info = calloc(1, DFU_ALT_INFO_ENV_LENGTH);
				free_dfu_alt_info = true;
				if (mmc_is_present(0))
					strcat(dfu_alt_info, MESON_DFU_MMC_EMMC);
#ifndef MESON_DFU_MMC_SD_HIDE
				if (mmc_is_present(1))
					strcat(dfu_alt_info, MESON_DFU_MMC_SD);
#endif
			}
			else if (strcmp(devstr, "0") == 0 && mmc_is_present(0))
				dfu_alt_info = MESON_DFU_MMC_EMMC_ALTS;
#ifndef MESON_DFU_MMC_SD_HIDE
			else if (strcmp(devstr, "1") == 0 && mmc_is_present(1))
				dfu_alt_info = MESON_DFU_MMC_SD_ALTS;
#endif
		}
#endif
	} else {
		dfu_alt_info = calloc(1, DFU_ALT_INFO_ENV_LENGTH);
		free_dfu_alt_info = true;
		if (!dfu_alt_info)
			return;
#ifdef CONFIG_DFU_SF
		strcpy(dfu_alt_info, "sf " __stringify(MESON_DFU_SF_BUS) ":"
		       __stringify(MESON_DFU_SF_CS) "=");
		meson_dfu_sf_alts(dfu_alt_info + strlen(dfu_alt_info),
				  DFU_ALT_INFO_ENV_LENGTH -
					  strlen(dfu_alt_info));
		strcat(dfu_alt_info, DFU_INTERFACE_SEPARATOR);
#endif
#ifdef CONFIG_DFU_MMC
		if (mmc_is_present(0))
			strcat(dfu_alt_info, MESON_DFU_MMC_EMMC);
#ifndef MESON_DFU_MMC_SD_HIDE
		if (mmc_is_present(1))
			strcat(dfu_alt_info, MESON_DFU_MMC_SD);
#endif
#endif
		strcat(dfu_alt_info, MESON_DFU_RAM);
	}
	
	if (dfu_alt_info)
		env_set("dfu_alt_info", dfu_alt_info);
	if (free_dfu_alt_info)
		free(dfu_alt_info);
}
#endif

/*
 * Where the environment and boot.ini live, by boot source (bootloader policy
 * §4 "Boot source variables", "`boot.ini` on SPI-NOR boot"):
 *
 *   eMMC / SD   the boot MMC device itself;
 *   SPI NOR     the first MMC device with a medium (the NOR carries no
 *               filesystem, and a NOR-booted board still reads boot.ini);
 *   USB         none -- a stage the BootROM took over the cable runs on its
 *               compiled-in defaults, not on a card it did not boot from;
 *   NAND/other  none.
 *
 * `bootdevice` is still set on every source (the splash and the block
 * commands' default device follow it); only the boot.ini read is gated.
 */
static int settings_r(void){
#ifdef CONFIG_ENV_MMC_DEVICE_INDEX
	int bootdevice_num = CONFIG_ENV_MMC_DEVICE_INDEX;
#else
	int bootdevice_num = 0;
#endif
	enum bootsource src = meson_bootsource();
	bool ini_from_mmc = false;
	char *bootdevice;

	switch (src) {
	case BOOTSOURCE_EMMC:
		ini_from_mmc = true;
		break;

	case BOOTSOURCE_SPI:
		if (!mmc_is_present(bootdevice_num))
			bootdevice_num = mmc_is_present(!bootdevice_num);
		ini_from_mmc = true;
		break;

	case BOOTSOURCE_SD:
		bootdevice_num = 1;
		ini_from_mmc = true;
		break;

	default:
		break;
	}

	bootdevice = simple_itoa(bootdevice_num);

	bootsource_publish(src, bootdevice_num);
#ifdef CONFIG_SPLASH_SOURCE
	env_set("splashdevpart", bootdevice);
#endif
#ifdef CONFIG_CMD_INI
	if (ini_from_mmc)
		env_ini_load("mmc", bootdevice);
#endif
	return 0;
}
EVENT_SPY_SIMPLE(EVT_SETTINGS_R, settings_r);

#if IS_ENABLED(CONFIG_MESON_GXL_AO_WDT_HANDOFF)
/*
 * The BootROM's own watchdog, in the always-on domain.  Write-only here on
 * purpose: 0xC810012C must never be READ (a read stalls both masters), and a
 * pet is a bare store of 0 -- no read-modify-write anywhere in this path.
 *
 * Petting does not change the ROM's 5000 ms period; it moves the deadline
 * forward.  U-Boot refreshes it while it runs and, BY DEFAULT, STOPS it
 * before the OS (a kernel without the watchdog driver would be reset 5 s in);
 * wdt_handoff=1 instead hands it over running, with the whole budget, for a
 * kernel known to carry the driver.  Rationale and the measurement:
 * CONFIG_MESON_GXL_AO_WDT_HANDOFF; policy: design/rules/bootloader/policy.md
 * "Watchdog hand-off".
 * This is the shared GXL Libre Computer implementation: v2 carried it since
 * a11a7a2ba95, and N29 moved it here so the v1 gets the same owner.
 */
#define GXL_AO_WDT_RESET	0xc810012cUL
/*
 * The control register (the one the kernel's meson_gxbb_wdt maps as
 * GXBB_WDT_CTRL_REG = 0x0 from the 0xC8100120 base) and its enable bit:
 * GXBB_WDT_CTRL_EN = BIT(18), drivers/watchdog/meson_gxbb_wdt.c:25 on
 * linux-6.18.y-lc.  Stopping is a read-modify-write of THIS register only.
 */
#define GXL_AO_WDT_CTRL		0xc8100120UL
#define GXL_AO_WDT_CTRL_EN	BIT(18)

static struct cyclic_info gxl_ao_wdt_cyclic;

static void gxl_ao_wdt_pet(struct cyclic_info *c)
{
	writel(0, GXL_AO_WDT_RESET);
}

static void gxl_ao_wdt_handoff_start(void)
{
	static bool started;

	if (started)
		return;

	started = true;
	writel(0, GXL_AO_WDT_RESET);
	cyclic_register(&gxl_ao_wdt_cyclic, gxl_ao_wdt_pet, 500000,
			"gxl_ao_wdt");
}

int board_early_init_r(void)
{
	gxl_ao_wdt_handoff_start();
	return 0;
}

/*
 * The raw AO watchdog's half of the platform hand-off, called ONCE by
 * wdt_handoff_to_os() (drivers/watchdog/wdt-uclass.c) at the last step before
 * the OS on both the booti/bootm and the UEFI paths.  The wdt_handoff decision
 * is made there, once, and arrives here as @run; this function reads no
 * environment and prints nothing (the helper prints the one line).
 *
 * @run: final refresh, the last instruction of ours the OS is guaranteed to
 * have run, so it bounds the OS's window and the watchdog is handed over
 * running.  Otherwise (the default) the watchdog is stopped by clearing EN in
 * the control register; no final refresh is needed then.
 */
bool board_wdt_handoff(bool run)
{
	if (run)
		writel(0, GXL_AO_WDT_RESET);
	else
		writel(readl(GXL_AO_WDT_CTRL) & ~GXL_AO_WDT_CTRL_EN,
		       GXL_AO_WDT_CTRL);

	return true;
}
#else
static void gxl_ao_wdt_handoff_start(void)
{
}
#endif

/*
 * serial# = the 12-byte chip id as 24 lowercase hex (design/rules/bootloader/
 * policy.md "Network MAC & serial number"), every Amlogic board, before
 * SMBIOS is built and before the NOR config import.  A saved serial# from
 * another board is replaced; a failed read deletes it and says so.
 */
void meson_setup_serial_env(void)
{
	chip_serial_env();
}

int meson_gxl_misc_init_r(void)
{
	u8 mac_addr[EFUSE_MAC_SIZE + 1];
	ssize_t len;

	if (!eth_env_get_enetaddr("ethaddr", mac_addr)) {
		len = meson_sm_read_efuse(EFUSE_MAC_OFFSET,
					  mac_addr, EFUSE_MAC_SIZE);
		mac_addr[len] = '\0';
		if (len == EFUSE_MAC_SIZE && is_valid_ethaddr(mac_addr))
			eth_env_set_enetaddr("ethaddr", mac_addr);
		else
			meson_generate_serial_ethaddr();
	}

	meson_setup_serial_env();

	/*
	 * A USB recovery boot runs on compiled-in env: no stored NOR env
	 * overrides (the saved env is already NOWHERE there). On a build with
	 * the NOR transport the whole board_config_env_import() is skipped on
	 * that boot -- its DRAM half too, if the build also has
	 * BOARD_CONFIG_DRAM (no LC defconfig has both). A build without NOR (a
	 * DRAM-only build, the host-staged block) keeps its import.
	 */
	if (!IS_ENABLED(CONFIG_BOARD_CONFIG_NOR) || !meson_usb_env_nowhere())
		board_config_env_import();

	gxl_ao_wdt_handoff_start();

	return 0;
}
