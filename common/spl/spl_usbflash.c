// SPDX-License-Identifier: GPL-2.0+
/*
 * The SPL USB flash session, shared by every SoC family whose SPL serves
 * one on a USB boot (<spl_usbflash.h>).  The family's SPL calls
 * spl_usbflash_session() and supplies a struct spl_usbflash_family: its
 * alternate names and extents, its RAM alternates' addresses, the session
 * timeout, its extra alternates (Meson: SPI NOR), its reset and the replay
 * of its earlier stages' console.  Meson: arch/arm/mach-meson/spl-usbflash.c;
 * Rockchip: arch/arm/mach-rockchip/spl-usbflash.c.
 *
 * The class drivers (f_dfu, f_mass_storage), the DFU core and its RAM/MMC/virt
 * back ends and the device controller are U-Boot's own, built for SPL; this
 * file is only the session loop and the alternate list.  No environment and
 * no shell: the alternate list is built here in C (dfu_alt_init() /
 * dfu_alt_add()), and the DFU core takes no environment read in this phase
 * (dfu_bare_phase() in include/dfu.h, dfu_env_get() in drivers/dfu/dfu.c).
 * That holds because this SPL is built without SPL_ENV_SUPPORT; the build
 * refuses below if one is ever enabled, so nothing stored on the board can
 * reach the session (operator decision 2026-10-02, claude
 * amlogic/software/boot/spl-usb-flasher-plan.md section 8).
 *
 * Alternates, built at run time from what answers (plan section 3):
 *   kernel, script   RAM, always (script: see below)
 *   eMMC (mmc 0)     the family's eMMC list if mmc 0 initialises; the boot0/
 *                    boot1 alternates only if the part reports boot partitions
 *                    (EXT_CSD BOOT_SIZE_MULT, mmc->capacity_boot)
 *   SD (mmc 1)       the family's SD list if mmc 1 initialises
 *   the family's     (probe_more), e.g. Meson's SPI NOR
 * mmc 0 is the eMMC and mmc 1 the SD slot (the DT aliases of every family
 * that runs this).  By default every storage alternate is read-only by
 * construction: SPL_MMC_WRITE is off (an MMC DNLOAD fails in blk_dwrite()).
 * A flashing target sets SPL_MMC_WRITE + SPL_USBFLASH_WRITE (Meson:
 * SPL_MESON_USBFLASH_WRITE selects it): its MMC alternates take DNLOAD and
 * its UMS may be read-write.
 *
 * The `script` alternate is a plain RAM buffer, not U-Boot proper's script
 * entity: SPL has no shell.  It holds one protocol line (plan section 4,
 * <usbflash-proto.h>, the parser U-Boot proper's `usbflash` command uses
 * too).  A download that is not a line this layer carries out is refused at
 * the DFU manifest step (dfu_board_manifest_check()): the host's GETSTATUS
 * reads errTARGET in dfuERROR, nothing is stored, and DFU goes on.  At the
 * end of a session (DETACH or the inactivity timeout) the stored line decides
 * what follows:
 *   empty                       the session ends and the boot continues
 *   usbflash ums mmc <n> ro     UMS of mmc <n>'s user area, read-only, for an
 *                               mmc that answered; when the host ejects the
 *                               medium, DFU again
 *   usbflash dfu                DFU again
 *   usbflash boot               the session ends and the boot continues
 *   usbflash reset              the family resets the board
 * `usbflash ums mmc <n>` (read-write) is the same UMS writable, and is refused
 * unless the build writes (SPL_USBFLASH_WRITE).  When the session ends and
 * the boot continues, spl_usbflash_fdt_fixup() can mark U-Boot proper's DTB
 * with /chosen/u-boot,spl-usbflash, so proper runs no second session (plan
 * section 5).
 *
 * A read-only UMS LUN is read-only three ways: struct ums .ro makes
 * f_mass_storage report it write-protected (MODE SENSE WP) and refuse every
 * WRITE with DATA PROTECT before .write_sector is reached; in a non-writing
 * build .write_sector writes nothing and the block device has no write op
 * (no SPL_MMC_WRITE).  Its buffers (2 x FSG_BUFLEN) come from the SPL heap
 * in DRAM.
 *
 * ACM console (SPL_USB_GADGET_DNL_ACM): every DFU gadget of the session also
 * carries U-Boot's CDC ACM function (g_dnl.c, f_acm.c, as in proper), and the
 * session's console lines reach its ttyACM through the console tee
 * (common/console.c), which the session starts first.  The UMS gadget carries
 * no ACM; what is printed while it is up reaches the ttyACM of the DFU session
 * after the eject.
 *
 * Timeout: the family's timeout_ms with no DFU request ends a DFU session;
 * f_dfu clears it on the first request.  If the storage alternates cannot be
 * configured the session still runs with the RAM alternates alone, and says
 * so.
 */

#include <blk.h>
#include <dfu.h>
#include <errno.h>
#include <fdt_support.h>
#include <g_dnl.h>
#include <malloc.h>
#include <mapmem.h>
#include <mmc.h>
#include <spl.h>
#include <spl_usbflash.h>
#include <stdio.h>
#include <usb.h>
#include <usb_mass_storage.h>
#include <usbflash-proto.h>
#include <watchdog.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>
#include <linux/string.h>
#include <linux/usb/gadget.h>

#if CONFIG_IS_ENABLED(ENV_SUPPORT)
#error "the SPL flash session runs without an environment: SPL_ENV_SUPPORT must be off"
#endif
/*
 * MMC writes only where the target asks for them: SPL_USBFLASH_WRITE turns
 * on read-write UMS and MMC DNLOAD.  Everywhere else the session stays
 * read-only, and an SPL_MMC_WRITE without that switch is a configuration
 * mistake.
 */
#define USBFLASH_WRITE	CONFIG_IS_ENABLED(USBFLASH_WRITE)
#if CONFIG_IS_ENABLED(MMC_WRITE) && !USBFLASH_WRITE
#error "SPL_MMC_WRITE without SPL_USBFLASH_WRITE: the flash session is read-only"
#endif

/*
 * UMS where the build has f_mass_storage (SPL_USB_FUNCTION_MASS_STORAGE);
 * without it the `usbflash ums` verb is refused at the manifest step.
 */
#define USBFLASH_UMS_OK	CONFIG_IS_ENABLED(USB_FUNCTION_MASS_STORAGE)

#define USBFLASH_MAX_ALTS	16

/* Between one gadget of the session and the next (see the session loop) */
#define USBFLASH_RESTART_GAP_MS	1000

/* The family of the session that is running; NULL outside a session. */
static const struct spl_usbflash_family *usbflash_fam;

/* mmc 0 / mmc 1 answered at session start (the UMS verb may name them) */
static bool usbflash_mmc_ok[2];

/* The session ran and the boot continues: mark proper's DTB (see below) */
static bool usbflash_session_ran;

/* `usbflash boot kernel`: the kernel alt U-Boot proper is to `bootm`, else 0 */
static ulong usbflash_bootm_addr;

/* One alternate: dfu_alt_add() arguments, kept intact (usbflash_config()). */
struct usbflash_alt {
	char *interface;
	char *devstr;
	char *spec;
};

static struct usbflash_alt *usbflash_alts;
static int usbflash_nalts;
/* The RAM alternates, first in the list: the RAM-only fallback registers these */
static int usbflash_nram;

static char *usbflash_strdup(const char *s)
{
	size_t n = strlen(s) + 1;
	char *p = malloc(n);

	if (p)
		memcpy(p, s, n);
	return p;
}

int spl_usbflash_add(const char *interface, const char *devstr,
		     const char *spec)
{
	struct usbflash_alt *a;

	if (usbflash_nalts >= USBFLASH_MAX_ALTS)
		return -ENOSPC;
	a = &usbflash_alts[usbflash_nalts];
	a->interface = usbflash_strdup(interface);
	a->devstr = usbflash_strdup(devstr);
	a->spec = usbflash_strdup(spec);
	if (!a->interface || !a->devstr || !a->spec)
		return -ENOMEM;
	usbflash_nalts++;

	return 0;
}

/*
 * Add every `;`-separated spec of @list on mmc @dev.  With @boot_parts false,
 * the specs naming `mmcpart 1` or `mmcpart 2` (the eMMC boot partitions) are
 * left out.
 */
static int usbflash_add_mmc_list(const char *dev, const char *list,
				 bool boot_parts)
{
	char spec[64];
	const char *p = list, *e;
	size_t n;
	int ret;

	while (*p) {
		e = strchr(p, ';');
		n = e ? (size_t)(e - p) : strlen(p);
		if (n >= sizeof(spec))
			return -E2BIG;
		memcpy(spec, p, n);
		spec[n] = '\0';
		if (boot_parts || (!strstr(spec, "mmcpart 1") &&
				   !strstr(spec, "mmcpart 2"))) {
			ret = spl_usbflash_add("mmc", dev, spec);
			if (ret)
				return ret;
		}
		p += n;
		if (*p == ';')
			p++;
	}

	return 0;
}

/* The eMMC alternates, if mmc 0 answers. */
static void usbflash_probe_emmc(void)
{
	struct mmc *mmc = find_mmc_device(0);
	int ret;

	if (!mmc) {
		puts("SPL usbflash: mmc 0 (eMMC) not bound\n");
		return;
	}
	ret = mmc_init(mmc);
	if (ret) {
		printf("SPL usbflash: mmc 0 (eMMC) absent (%d)\n", ret);
		return;
	}
	usbflash_mmc_ok[0] = true;
	printf("SPL usbflash: mmc 0 (eMMC) user 0x%x KiB, boot 0x%x KiB each, mode %u clock %u bus %u-bit\n",
	       (u32)(mmc->capacity_user >> 10), (u32)(mmc->capacity_boot >> 10),
	       (u32)mmc->selected_mode, mmc->clock, mmc->bus_width);
	ret = usbflash_add_mmc_list("0", usbflash_fam->emmc_alts,
				    mmc->capacity_boot != 0);
	if (ret)
		printf("SPL usbflash: eMMC alts not added (%d)\n", ret);
}

/*
 * Hand both cards to the next stage in a mode it can take them from.  An eMMC
 * the session left in HS200 is not re-initialised by a stage that does not
 * tune: U-Boot proper then found no bootflow on it.  mmc_deinit() steps each
 * card down to its best untuned mode, and returns at once when this stage
 * builds no high-speed mode.
 */
static void usbflash_mmc_handoff(void)
{
	int i;

	for (i = 0; i < 2; i++) {
		struct mmc *mmc;
		int ret;

		if (!usbflash_mmc_ok[i])
			continue;
		mmc = find_mmc_device(i);
		if (!mmc)
			continue;
		ret = mmc_deinit(mmc);
		printf("SPL usbflash: mmc %d handed off, mode %u clock %u (%d)\n",
		       i, (u32)mmc->selected_mode, mmc->clock, ret);
	}
}

/* The SD alternates, if mmc 1 answers.  No card fitted means none. */
static void usbflash_probe_sd(void)
{
	struct mmc *mmc = find_mmc_device(1);
	int ret;

	if (!mmc) {
		puts("SPL usbflash: mmc 1 (SD) not bound\n");
		return;
	}
	ret = mmc_init(mmc);
	if (ret) {
		printf("SPL usbflash: mmc 1 (SD) absent (%d)\n", ret);
		return;
	}
	usbflash_mmc_ok[1] = true;
	printf("SPL usbflash: mmc 1 (SD) user 0x%x KiB, mode %u clock %u bus %u-bit\n",
	       (u32)(mmc->capacity_user >> 10), (u32)mmc->selected_mode,
	       mmc->clock, mmc->bus_width);
	ret = usbflash_add_mmc_list("1", usbflash_fam->sd_alts, true);
	if (ret)
		printf("SPL usbflash: SD alts not added (%d)\n", ret);
}

/*
 * Register the list: everything that answered, or the RAM alternates alone
 * when @ram_only (the fallback).  0 or a negative errno.
 */
static int usbflash_config(bool ram_only)
{
	struct dfu_entity *dfu;
	int i, n = ram_only ? usbflash_nram : usbflash_nalts, ret;

	ret = dfu_alt_init(n, &dfu);
	if (ret)
		return ret;

	for (i = 0; i < n; i++) {
		/*
		 * dfu_alt_add() tokenises its strings in place, so it gets copies:
		 * the list stays intact for the RAM-only retry below a failed
		 * full registration.
		 */
		char interface[16], devstr[16], spec[80];

		strlcpy(interface, usbflash_alts[i].interface, sizeof(interface));
		strlcpy(devstr, usbflash_alts[i].devstr, sizeof(devstr));
		strlcpy(spec, usbflash_alts[i].spec, sizeof(spec));
		ret = dfu_alt_add(dfu, interface, devstr, spec);
		if (ret) {
			printf("SPL usbflash: alt %d refused (%d)\n", i, ret);
			return ret;
		}
	}

	return 0;
}

/* One RAM alternate: `<name> ram <addr> <size>` on ram 0. */
static int usbflash_add_ram(const char *name, ulong addr, ulong size)
{
	char spec[64];

	snprintf(spec, sizeof(spec), "%s ram 0x%lx 0x%lx", name, addr, size);

	return spl_usbflash_add("ram", "0", spec);
}

/* Build the alternate list from what answers on this board. */
static int usbflash_build_alts(void)
{
	int ret;

	usbflash_alts = calloc(USBFLASH_MAX_ALTS, sizeof(*usbflash_alts));
	if (!usbflash_alts)
		return -ENOMEM;

	/* The RAM alternates first: the fallback list is these. */
	ret = usbflash_add_ram("kernel", usbflash_fam->kernel_addr,
			       usbflash_fam->kernel_size);
	if (!ret)
		ret = usbflash_add_ram("script", usbflash_fam->script_addr,
				       usbflash_fam->script_size);
	/*
	 * kernel-lzma: an LZMA stream unpacked into `kernel` at the end of the
	 * download (dfu_ram.c DFU_RAM_LZMA); upload `kernel` to read it back.
	 */
	if (!ret && CONFIG_IS_ENABLED(DFU_RAM_LZMA) && usbflash_fam->lzma_staging_addr) {
		char spec[80];

		snprintf(spec, sizeof(spec), "kernel-lzma lzma 0x%lx 0x%lx 0x%lx",
			 usbflash_fam->kernel_addr, usbflash_fam->kernel_size,
			 usbflash_fam->lzma_staging_addr);
		ret = spl_usbflash_add("ram", "0", spec);
	}
	if (ret)
		return ret;
	usbflash_nram = usbflash_nalts;

	ret = mmc_initialize(NULL);
	if (ret)
		printf("SPL usbflash: mmc_initialize %d\n", ret);
	usbflash_probe_emmc();
	usbflash_probe_sd();
	if (usbflash_fam->probe_more)
		usbflash_fam->probe_more();
	printf("SPL usbflash: %d alts\n", usbflash_nalts);

	return 0;
}

static void usbflash_restore(void)
{
	if (usbflash_fam->restore)
		usbflash_fam->restore();
}

/*
 * Is @s a protocol line this layer carries out?  The parser is proper's too
 * (<usbflash-proto.h>); the SPL's own rules on top: a UMS must be read-only
 * unless this build writes (USBFLASH_WRITE), and name an MMC device that
 * answered.
 */
static int usbflash_allowed(const char *s, size_t len, struct usbflash_cmd *cmd)
{
	int ret = usbflash_parse(s, len, cmd);

	if (ret)
		return ret;
	if (cmd->verb == USBFLASH_UMS) {
		if (!USBFLASH_UMS_OK)
			return -ENOSYS;	/* a build without f_mass_storage */
		if (!cmd->ro && !USBFLASH_WRITE)
			return -EROFS;	/* read-write UMS: writing builds only */
		if (cmd->mmc > 1 || !usbflash_mmc_ok[cmd->mmc])
			return -ENODEV;
	}

	return 0;
}

/*
 * DFU manifest hook (drivers/dfu/dfu.c): a `script` download that is not an
 * allowed line is refused at the manifest step, so the host's GETSTATUS
 * reads errTARGET, nothing reaches the buffer, and DFU goes on.  Outside a
 * session, nothing to check.
 */
int dfu_board_manifest_check(struct dfu_entity *dfu, const void *buf,
			     long len, u64 written)
{
	struct usbflash_cmd cmd;
	int ret;

	if (!usbflash_fam || strcmp(dfu->name, "script"))
		return 0;
	if (written || len < 0)
		return -EFBIG;
	ret = usbflash_allowed(buf, len, &cmd);
	if (ret)
		printf("SPL usbflash: script refused at manifest (%d)\n", ret);

	return ret;
}

/*
 * A new transaction on `script` starts from an empty buffer, so a shorter
 * line never ends in the tail of an earlier, longer one.
 */
void dfu_initiated_callback(struct dfu_entity *dfu)
{
	if (usbflash_fam && !strcmp(dfu->name, "script"))
		memset(map_sysmem(usbflash_fam->script_addr,
				  usbflash_fam->script_size), 0,
		       usbflash_fam->script_size);
}

/*
 * What the host left in the `script` alternate: 0 and @cmd, 1 for nothing
 * (continue the boot), or a negative errno (refused).
 */
static int usbflash_next(struct usbflash_cmd *cmd)
{
	ulong size = usbflash_fam->script_size;
	const char *s = map_sysmem(usbflash_fam->script_addr, size);
	size_t len = strnlen(s, size);

	if (!len)
		return 1;
	if (len == size)
		return -E2BIG;		/* no terminator in the buffer */

	return usbflash_allowed(s, len, cmd);
}

#if USBFLASH_UMS_OK
static int usbflash_ums_read(struct ums *ums_dev, ulong start,
			     lbaint_t blkcnt, void *buf)
{
	struct blk_desc *block_dev = &ums_dev->block_dev;
	int ret;

	/* As cmd/usb_mass_storage.c; a no-op when the part is selected */
	ret = blk_dselect_hwpart(block_dev, ums_dev->hwpart);
	if (ret && ret != -ENOSYS)
		return ret;

	return blk_dread(block_dev, start + ums_dev->start_sector, blkcnt, buf);
}

/*
 * A writing build: as cmd/usb_mass_storage.c.  Otherwise not reached while
 * .ro is set (f_mass_storage do_write()), and writes nothing.
 */
static int usbflash_ums_write(struct ums *ums_dev, ulong start,
			      lbaint_t blkcnt, const void *buf)
{
#if USBFLASH_WRITE
	struct blk_desc *block_dev = &ums_dev->block_dev;
	int ret;

	ret = blk_dselect_hwpart(block_dev, ums_dev->hwpart);
	if (ret && ret != -ENOSYS)
		return ret;

	return blk_dwrite(block_dev, start + ums_dev->start_sector, blkcnt,
			  buf);
#else
	puts("SPL usbflash: UMS write refused\n");

	return 0;
#endif
}

/*
 * UMS of mmc @n's user area until the host ejects the medium; read-only
 * unless @ro is false, which usbflash_allowed() admits only in a writing
 * build.
 */
static int usbflash_ums(int n, bool ro)
{
	struct mmc *mmc = find_mmc_device(n);
	struct blk_desc *desc;
	struct udevice *udc;
	struct ums lun = { };
	int ret;

	if (!mmc || mmc_init(mmc)) {
		printf("SPL usbflash: UMS: mmc %d not available\n", n);
		return -ENODEV;
	}
	desc = mmc_get_blk_desc(mmc);
	if (!desc || !desc->lba) {
		printf("SPL usbflash: UMS: mmc %d has no block device\n", n);
		return -ENODEV;
	}
	/* The UMS LUN is the user area: select it (a boot alt may not have) */
	ret = blk_dselect_hwpart(desc, 0);
	if (ret && ret != -ENOSYS) {
		printf("SPL usbflash: UMS: user area not selected (%d)\n", ret);
		return ret;
	}

	lun.read_sector = usbflash_ums_read;
	lun.write_sector = usbflash_ums_write;
	lun.start_sector = 0;
	lun.num_sectors = desc->lba;
	lun.name = "UMS disk 0";
	lun.block_dev = *desc;
	lun.hwpart = 0;		/* user area */
#if USBFLASH_WRITE
	lun.ro = ro;
	printf("SPL usbflash: UMS mmc %d %s, %u sectors of %lu B\n",
	       n, lun.ro ? "read-only" : "read-write", lun.num_sectors,
	       desc->blksz);
#else
	lun.ro = true;
	printf("SPL usbflash: UMS mmc %d read-only, %u sectors of %lu B\n",
	       n, lun.num_sectors, desc->blksz);
#endif

	ret = udc_device_get_by_index(0, &udc);
	if (ret) {
		printf("SPL usbflash: UMS: no UDC (%d)\n", ret);
		return ret;
	}
	ret = fsg_init(&lun, 1, udc);
	if (!ret)
		ret = g_dnl_register("usb_dnl_ums");
	if (ret) {
		printf("SPL usbflash: UMS: gadget not registered (%d)\n", ret);
		udc_device_put(udc);
		return ret;
	}

	puts("SPL usbflash: UMS up\n");
	do {
		dm_usb_gadget_handle_interrupts(udc);
		ret = fsg_main_thread(NULL);
		schedule();
	} while (!ret);

	g_dnl_release(udc);
	/*
	 * f_mass_storage's busy_indicator() draws its spinner as "\r<c>" with
	 * no newline (shared with proper's `ums`, so left alone there). Blank
	 * the spinner and end its line, so the next line starts at column 0
	 * on the UART and on the ttyACM ring alike.
	 */
	puts("\r \r\n");
	printf("SPL usbflash: UMS over (%d)\n", ret);

	return 0;
}
#else
/* Not reached: usbflash_allowed() refuses the verb in a build without UMS. */
static int usbflash_ums(int n, bool ro)
{
	return -ENOSYS;
}
#endif /* USBFLASH_UMS_OK */

/*
 * One console line per `script` line the session carries out, numbered from 1
 * in the session: a marker the host can look for on the UART and, with
 * SPL_USB_GADGET_DNL_ACM, on the ttyACM of the next DFU session.
 */
static void usbflash_mark(int n, const struct usbflash_cmd *cmd)
{
	switch (cmd->verb) {
	case USBFLASH_UMS:
		printf("SPL usbflash: script %d: ums mmc %d%s\n", n, cmd->mmc,
		       cmd->ro ? " ro" : "");
		break;
	case USBFLASH_DFU:
		printf("SPL usbflash: script %d: dfu\n", n);
		break;
	case USBFLASH_BOOT:
		printf("SPL usbflash: script %d: boot\n", n);
		break;
	case USBFLASH_BOOT_KERNEL:
		printf("SPL usbflash: script %d: boot kernel\n", n);
		break;
	case USBFLASH_RESET:
		printf("SPL usbflash: script %d: reset\n", n);
		break;
	}
}

void spl_usbflash_session(const struct spl_usbflash_family *fam)
{
	char *script;
	int acm_ret = 0, nscript = 0, ret;

	usbflash_fam = fam;
	/*
	 * SPL_USB_GADGET_DNL_ACM: from here on every console line also goes to
	 * the ACM console the DFU gadget carries (the full heap is up).
	 */
	if (CONFIG_IS_ENABLED(USB_GADGET_DNL_ACM))
		acm_ret = acm_dnl_console_start();
	/*
	 * The console starts with the boot so far -- the earlier stages'
	 * output, which the tee above never saw.  Before any new line, so the
	 * ACM console reads in the UART's order.
	 */
	if (CONFIG_IS_ENABLED(USB_GADGET_DNL_ACM) && !acm_ret && fam->replay)
		fam->replay(acm_dnl_console_write);
	/* The host driver keys the session's start on this line. */
	puts("SPL usbflash: session\n");
	if (CONFIG_IS_ENABLED(USB_GADGET_DNL_ACM))
		printf("SPL usbflash: ACM console beside DFU (%d)\n", acm_ret);

	ret = usbflash_build_alts();
	if (ret || usbflash_nalts < usbflash_nram || usbflash_nram < 2) {
		usbflash_restore();
		printf("SPL usbflash: no alt list (%d), boot continues\n", ret);
		usbflash_fam = NULL;
		return;
	}
	ret = usbflash_config(false);
	if (ret) {
		dfu_free_entities();
		puts("SPL usbflash: storage alts unavailable, RAM alts only\n");
		ret = usbflash_config(true);
		if (ret) {
			dfu_free_entities();
			usbflash_restore();
			printf("SPL usbflash: no alts (%d), boot continues\n",
			       ret);
			usbflash_fam = NULL;
			return;
		}
	}

	script = map_sysmem(fam->script_addr, fam->script_size);
	for (;;) {
		struct usbflash_cmd cmd;
		int next;

		memset(script, 0, fam->script_size);
		dfu_set_timeout(fam->timeout_ms);
		/* a host that goes quiet ends the session too, as one that never came */
		dfu_set_idle_timeout(CONFIG_SPL_USBFLASH_IDLE_TIMEOUT_MS);
		ret = run_usb_dnl_gadget(0, "usb_dnl_dfu");

		next = usbflash_next(&cmd);
		if (next == 1)
			break;			/* nothing pending: boot */
		if (next < 0) {
			/* not reached: the manifest step refuses these */
			printf("SPL usbflash: script refused (%d), DFU again\n",
			       next);
			continue;
		}
		usbflash_mark(++nscript, &cmd);
		if (cmd.verb == USBFLASH_BOOT) {
			puts("SPL usbflash: usbflash boot\n");
			break;
		}
		if (cmd.verb == USBFLASH_BOOT_KERNEL) {
			/* the FIT stays in the kernel alt for proper's bootm */
			usbflash_bootm_addr = fam->kernel_addr;
			printf("SPL usbflash: usbflash boot kernel (0x%lx)\n",
			       usbflash_bootm_addr);
			break;
		}
		if (cmd.verb == USBFLASH_RESET) {
			dfu_free_entities();
			usbflash_restore();
			puts("SPL usbflash: reset\n");
			fam->reset();
		}
		if (cmd.verb == USBFLASH_UMS)
			usbflash_ums(cmd.mmc, cmd.ro);
		/*
		 * Let the host see the last gadget go before the next one comes:
		 * on RK3399 a gadget restarted at once, after a SuperSpeed
		 * session the host ended with DETACH, trained while the host was
		 * still processing the old link's disconnect and was never seen
		 * (0 of 3 boots); with this pause 3 of 3.
		 */
		mdelay(USBFLASH_RESTART_GAP_MS);
		puts("SPL usbflash: DFU again\n");
	}

	dfu_free_entities();
	usbflash_restore();
	usbflash_mmc_handoff();
	/* One session per boot: see spl_usbflash_fdt_fixup() */
	usbflash_session_ran = true;
	usbflash_fam = NULL;
	printf("SPL usbflash: session over (%d), boot continues\n", ret);
}

/*
 * spl_usbflash_fdt_fixup() - tell U-Boot proper the session ran.
 *
 * Called by the family's SPL with the control DTB the FIT loader placed after
 * U-Boot proper, with 8 KiB of room (spl_fit_append_fdt()): the copy proper
 * runs on.  It gets an empty /chosen/u-boot,spl-usbflash, and proper then runs
 * no second session (plan section 5).  The DTB comes with the FIT on every
 * boot, so the mark cannot outlive the boot it describes.
 */
void spl_usbflash_fdt_fixup(void *blob)
{
	int chosen, ret;

	if (!usbflash_session_ran || !blob)
		return;
	chosen = fdt_find_or_add_subnode(blob, 0, "chosen");
	ret = chosen < 0 ? chosen :
	      fdt_setprop_empty(blob, chosen, USBFLASH_FDT_PROP);
	printf("SPL usbflash: /chosen/" USBFLASH_FDT_PROP " %s (%d)\n",
	       ret ? "NOT set" : "set", ret);
	if (ret || !usbflash_bootm_addr)
		return;
	/* `usbflash boot kernel`: proper boots the FIT in the kernel alt */
	ret = fdt_setprop_u64(blob, chosen, USBFLASH_FDT_BOOTM,
			      usbflash_bootm_addr);
	printf("SPL usbflash: /chosen/" USBFLASH_FDT_BOOTM " 0x%lx %s (%d)\n",
	       usbflash_bootm_addr, ret ? "NOT set" : "set", ret);
}
