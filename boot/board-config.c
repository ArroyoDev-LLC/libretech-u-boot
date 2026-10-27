// SPDX-License-Identifier: GPL-2.0+
/*
 * Board configuration block -- platform-neutral implementation
 *
 * Implements the Libre Computer board-bootloader policy "board config block":
 * plain-text key=value environment *overrides* (optionally behind a CRC32
 * header) plus an "overlays=" key naming DT overlays to apply. The environment
 * proper is never stored here.
 *
 * The facility is generic; what differs between boards is the *transport* that
 * carries the block, and each transport is separately Kconfig-gated:
 *
 *   BOARD_CONFIG_NOR  -- one erase block of SPI NOR holds the config block, and
 *                        a second NOR region holds the overlay FIT. The read
 *                        path comes from four board hooks (see
 *                        include/board-config.h).
 *   BOARD_CONFIG_DRAM -- the boot host (pyamlboot) stages the config block and
 *                        the overlay FIT directly into DRAM, for boards that
 *                        have no usable NOR. Default n, so a flash-booting
 *                        board compiles none of it.
 *
 * Every console message carries its transport -- "board-config-nor:",
 * "board-config-dram:", "board-env-nor:", "board-env-dram:" -- because the
 * transports funnel through the same helpers and an operator diagnosing a
 * missing overlay needs to know which one actually ran. The shared helpers
 * take the transport as a @tag rather than hard-coding one: a hard-coded
 * prefix is exactly the bug this replaced, where a NOR-less USB boot reported
 * "nor-config: applied <overlay>".
 *
 * Two entry points, both optional and independently Kconfig-gated:
 *
 *   board_config_env_import() -- import the key=value overrides. Called by the
 *                             board from misc_init_r(), which board_r.c runs
 *                             after initr_env(), i.e. with the environment
 *                             proper already loaded.
 *   fdtdec_board_setup()   -- load a FIT of pre-compiled DTBOs and apply the
 *                             ones named by "overlays=" to the control FDT,
 *                             before DM init. Compiled only when
 *                             CONFIG_BOARD_CONFIG_OVERLAYS is set, so a
 *                             board that provides its own fdtdec_board_setup()
 *                             can still use the env-override half.
 *
 * (C) Copyright 2025 Da Xue <da@libre.computer>
 */

#include <linux/libfdt.h>
#include <linux/compiler.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <u-boot/crc.h>
#include <env.h>
#include <fdt_support.h>
#include <malloc.h>
#include <board-config.h>
#include <string.h>

#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
/* NOR transport layout -- see the CONFIG_BOARD_CONFIG_NOR_* help in boot/Kconfig */
#define NOR_SECTOR_SIZE		CONFIG_BOARD_CONFIG_NOR_SIZE

#endif

/*
 * Parser limits. Every one of these is a *stack* buffer in a function that
 * may run before relocation, so they cannot simply be made generous -- but
 * nothing may be dropped on the floor for exceeding one either. Each limit
 * below has an explicit diagnostic at the point it bites.
 */
#define BOARD_CONFIG_KEY_LEN	128	/* incl. NUL */
#define BOARD_CONFIG_VAL_LEN	512	/* incl. NUL */

#if IS_ENABLED(CONFIG_BOARD_CONFIG_OVERLAYS)
#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
/* NOR transport: where the overlay FIT lives in flash */
#define NOR_FIT_OFFSET		CONFIG_BOARD_CONFIG_NOR_FIT_OFFSET
#endif

/* Transport-independent ceiling on the overlay FIT */
#define BOARD_CONFIG_FIT_MAX_SIZE	CONFIG_BOARD_CONFIG_OVERLAY_FIT_MAX_SIZE

/*
 * Scratch buffer in DRAM. Board-declared: the loader may write
 * [SCRATCH_ADDR, SCRATCH_ADDR + SCRATCH_SIZE) and nothing outside it.
 */
#define SCRATCH_ADDR		CONFIG_BOARD_CONFIG_OVERLAY_SCRATCH_ADDR
#define SCRATCH_SIZE		CONFIG_BOARD_CONFIG_OVERLAY_SCRATCH_SIZE

/* Board-declared ceiling on the in-place growth of the control FDT */
#define BOARD_CONFIG_FDT_MAX_SIZE	CONFIG_BOARD_CONFIG_OVERLAY_FDT_MAX_SIZE

/* Slack fdt_open_into() is asked for on top of the overlay's own size */
#define BOARD_CONFIG_FDT_PAD	4096

/*
 * Both scratch users are bounded by compile-time constants, so an
 * under-sized scratch window is a build failure, not a run-time overrun.
 * The .dtbo copy is placed at ALIGN(fit_size, 64) and is bounded at run
 * time -- fit_size is only known then.
 */
#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
#if CONFIG_BOARD_CONFIG_NOR_SIZE > CONFIG_BOARD_CONFIG_OVERLAY_SCRATCH_SIZE
#error "BOARD_CONFIG_OVERLAY_SCRATCH_SIZE is smaller than BOARD_CONFIG_NOR_SIZE"
#endif
#endif
#if CONFIG_BOARD_CONFIG_OVERLAY_FIT_MAX_SIZE > CONFIG_BOARD_CONFIG_OVERLAY_SCRATCH_SIZE
#error "BOARD_CONFIG_OVERLAY_SCRATCH_SIZE is smaller than BOARD_CONFIG_OVERLAY_FIT_MAX_SIZE"
#endif

/* FIT header read size */
#define FIT_HDR_SIZE		64

/* Longest "overlays=" value accepted from the config block, incl. NUL */
#define BOARD_CONFIG_OVERLAYS_LEN	256

/* Longest "need = " dep list accepted from a FIT /overlay-deps node */
#define BOARD_CONFIG_OV_DEPS_LEN	128
#endif

#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
/*
 * __weak platform hooks. A board overrides only the ones it needs; the
 * default board_nor_read() returns -ENOSYS, which makes the NOR transport
 * bail out silently, so a board that provides none simply no-ops.
 */
__weak bool board_nor_clk_enabled(void)
{
	return true;
}

__weak bool board_nor_init(void)
{
	return true;
}

__weak bool board_nor_probe(void)
{
	return true;
}

__weak int board_nor_read(u32 offset, void *buf, u32 len)
{
	return -ENOSYS;
}

/*
 * board_nor_size()'s __weak default and board_config_nor_offset() are in
 * boot/board-config-nor.c, which the SPL flash session also builds, so the
 * config block has one offset in every phase.
 */

/* Policy, not hardware -- see the kerneldoc in include/board-config.h. */
__weak bool board_config_nor_skip(void)
{
	return false;
}
#endif /* CONFIG_BOARD_CONFIG_NOR */

/*
 * Is @klen bytes at @k a plausible environment variable name?
 * [A-Za-z_][A-Za-z0-9_.]*
 */
static bool board_config_key_ok(const char *k, int klen)
{
	int i;

	if (klen <= 0)
		return false;

	if (!isalpha(k[0]) && k[0] != '_')
		return false;

	for (i = 1; i < klen; i++)
		if (!isalnum(k[i]) && k[i] != '_' && k[i] != '.')
			return false;

	return true;
}

/**
 * board_config_is_valid() - does this block hold a board config at all?
 * @start: first byte of the block (after any CRC header)
 * @size: bytes available
 *
 * A config block is written to a dedicated erase block (NOR) or staged by the
 * boot host (DRAM), but nothing guarantees that region has ever been written --
 * it may still hold the tail of whatever image previously occupied it, i.e.
 * arbitrary binary. An *erased* block is safe (the parsers stop at the first
 * 0xff), but leftover binary is not: scanning it for "<something>=<something>"
 * will eventually find a byte run that looks like an assignment, and a firmware
 * image is exactly the kind of data that contains strings like "bootcmd=..." or
 * "bootargs=..." in its built-in default environment. Importing those would
 * silently override the real environment with fossils of an old build.
 *
 * So require the whole block to be a plausible plain-text config before any
 * of it is used: printable ASCII only, and every entry either a comment or a
 * "key=value" with a syntactically valid key. One bad byte or one bad key
 * rejects the entire block -- a partially-garbage block is not a config
 * block, and taking the good-looking prefix of one is exactly the failure
 * mode this guards against.
 *
 * Return: true if the block may be parsed, false to reject it outright.
 */
static bool board_config_is_valid(const char *start, u32 size)
{
	const char *p, *q, *end, *eq;
	int entries = 0;

	end = start + size;

	for (p = start; p < end; ) {
		/* 0xff is erased flash: end of content */
		if ((unsigned char)*p == 0xff)
			break;

		/* NUL separates entries; two in a row end the block */
		if (*p == '\0') {
			if (p + 1 >= end || *(p + 1) == '\0')
				break;
			p++;
			continue;
		}

		if (*p == '\n' || *p == '\r') {
			p++;
			continue;
		}

		/* one entry, up to the next separator */
		eq = NULL;
		for (q = p; q < end && *q != '\0' && *q != '\n' &&
		     (unsigned char)*q != 0xff; q++) {
			unsigned char c = *q;

			if (c != '\t' && (c < 0x20 || c > 0x7e))
				return false;	/* not plain text */

			if (!eq && c == '=')
				eq = q;
		}

		if (*p == '#') {		/* comment line */
			p = q;
			continue;
		}

		if (!eq || !board_config_key_ok(p, eq - p))
			return false;

		entries++;
		p = q;
	}

	return entries > 0;
}

#if IS_ENABLED(CONFIG_BOARD_CONFIG_DRAM)
/*
 * DRAM transport: a host-staged block, for boards with no usable NOR.
 * See the BOARD_CONFIG_DRAM help text for the wire format. The USB boot host
 * writes it after BL2 has brought DRAM up and before BL33 runs, so it is
 * already in place by fdtdec_board_setup().
 *
 * Nothing distinguishes an absent block from whatever the previous boot
 * left at that address, so the magic and the CRC are the whole gate: a
 * block that fails either is treated as absent and the NOR transport runs
 * unchanged.
 *
 * The magic string is on the wire between the host tool and this code -- it
 * is deliberately NOT renamed with the Kconfig symbols.
 */
#define BOARD_CONFIG_DRAM_MAGIC	"LCRAMCFG"

struct board_config_dram_hdr {
	char magic[8];
	__le32 version;
	__le32 env_size;
	__le32 fit_size;
	__le32 crc;
};

/*
 * Validate the block and return pointers into it, or NULL if absent.
 * Either out-param pair may be NULL.
 */
static struct board_config_dram_hdr *board_config_dram_find(const char **env,
							    u32 *env_size,
							    void **fit,
							    u32 *fit_size)
{
	struct board_config_dram_hdr *hdr =
		(void *)(uintptr_t)CONFIG_BOARD_CONFIG_DRAM_ADDR;
	u32 esize, fsize, payload;

	if (memcmp(hdr->magic, BOARD_CONFIG_DRAM_MAGIC, 8))
		return NULL;

	if (le32_to_cpu(hdr->version) != 1)
		return NULL;

	esize = le32_to_cpu(hdr->env_size);
	fsize = le32_to_cpu(hdr->fit_size);
	payload = esize + fsize;
	/* payload < esize catches the u32 wrap of a hostile/garbage header */
	if (!payload || payload < esize ||
	    payload > CONFIG_BOARD_CONFIG_DRAM_MAX_SIZE)
		return NULL;

	if (crc32(0, (const unsigned char *)(hdr + 1), payload) !=
	    le32_to_cpu(hdr->crc))
		return NULL;

	if (env) {
		*env = (const char *)(hdr + 1);
		*env_size = esize;
	}
	if (fit) {
		*fit = fsize ? (u8 *)(hdr + 1) + esize : NULL;
		*fit_size = fsize;
	}

	return hdr;
}
#endif /* CONFIG_BOARD_CONFIG_DRAM */

#if IS_ENABLED(CONFIG_BOARD_CONFIG_OVERLAYS)
/*
 * Scan env data for "overlays=" and copy the value to out_buf.
 * start is the first byte of env entries (after any header).
 * Entries are separated by \0 (binary/fw_setenv) or \n (text).
 * Returns length of value, or 0 if not found. Sets *truncated when the value
 * did not fit @out_size, so the caller can say so.
 */
static int board_config_scan_overlays(const char *start, u32 size,
				      char *out_buf, int out_size,
				      bool *truncated)
{
	const char *prefix = "overlays=";
	int pfxlen = 9;
	const char *p, *end, *val;
	int i;

	end = start + size;

	for (p = start; p + pfxlen < end; p++) {
		if (p != start && *(p - 1) != '\0' && *(p - 1) != '\n')
			continue;
		if (strncmp(p, prefix, pfxlen) != 0)
			continue;

		val = p + pfxlen;
		for (i = 0; i < out_size - 1 && val + i < end; i++) {
			if (val[i] == '\0' || val[i] == '\n' ||
			    (unsigned char)val[i] == 0xff)
				break;
			out_buf[i] = val[i];
		}
		out_buf[i] = '\0';

		/*
		 * The copy stopped on the buffer, not on a separator: the
		 * config asked for more overlays than were taken. Silently
		 * booting a prefix of the requested set is the failure this
		 * facility is most prone to -- the board comes up missing
		 * hardware, with the cause several layers away.
		 */
		if (i == out_size - 1 && val + i < end &&
		    val[i] != '\0' && val[i] != '\n' &&
		    (unsigned char)val[i] != 0xff)
			*truncated = true;

		return i;
	}

	return 0;
}

/*
 * Find "overlays=" in a config block.
 * Tries binary env format first (4-byte CRC32 + null-separated entries),
 * then falls back to raw text format (no header, newline-separated).
 */
static int board_config_get_overlays(const char *data, u32 size,
				     char *out_buf, int out_size,
				     bool *truncated)
{
	u32 env_crc, calc_crc;
	int ret;

	/*
	 * Try binary env: first 4 bytes are CRC32 over the rest. The NOR
	 * sector is always a whole erase block, but a host-staged block
	 * carries only the bytes the host wrote, so a payload of 4 bytes or
	 * fewer is possible -- and "size - 4" is unsigned, so computing the
	 * CRC over it unguarded would walk 4 GiB of DRAM before relocation.
	 */
	if (size > 4) {
		env_crc = le32_to_cpu(*(const u32 *)data);
		calc_crc = crc32(0, (const unsigned char *)data + 4, size - 4);
		if (env_crc == calc_crc &&
		    board_config_is_valid(data + 4, size - 4)) {
			ret = board_config_scan_overlays(data + 4, size - 4,
							 out_buf, out_size,
							 truncated);
			if (ret)
				return ret;
		}
	}

	/* Fall back to raw text format (no CRC header) */
	if (!board_config_is_valid(data, size))
		return 0;

	return board_config_scan_overlays(data, size, out_buf, out_size,
					  truncated);
}

#define BOARD_CONFIG_OV_MAX		32
#define BOARD_CONFIG_OV_NAME_LEN	48

static int board_config_fit_resolve_name(const void *fit, const char *name,
					 int namelen, char *out)
{
	char path[64];
	char aname[BOARD_CONFIG_OV_NAME_LEN];
	const char *canon;
	int alias_node, aliases_node, canon_len;

	if (namelen <= 0 || namelen >= BOARD_CONFIG_OV_NAME_LEN)
		return -1;

	memcpy(aname, name, namelen);
	aname[namelen] = '\0';

	if (namelen + 9 < (int)sizeof(path)) {
		memcpy(path, "/images/", 8);
		memcpy(path + 8, aname, namelen + 1);
		if (fdt_path_offset(fit, path) >= 0) {
			memcpy(out, aname, namelen + 1);
			return 0;
		}
	}

	aliases_node = fdt_path_offset(fit, "/overlay-aliases");
	if (aliases_node < 0)
		return -1;
	alias_node = fdt_subnode_offset(fit, aliases_node, aname);
	if (alias_node < 0)
		return -1;
	canon = fdt_getprop(fit, alias_node, "target", &canon_len);
	if (!canon || canon_len <= 0)
		return -1;
	if (canon[canon_len - 1] == '\0')
		canon_len--;
	if (canon_len <= 0 || canon_len >= BOARD_CONFIG_OV_NAME_LEN)
		return -1;
	memcpy(out, canon, canon_len);
	out[canon_len] = '\0';
	return 0;
}

static int board_config_ov_already(char names[][BOARD_CONFIG_OV_NAME_LEN],
				   int count, const char *name)
{
	int i;

	for (i = 0; i < count; i++)
		if (!strcmp(names[i], name))
			return 1;
	return 0;
}

static int board_config_ov_add_with_deps(const void *fit,
					 char apply[][BOARD_CONFIG_OV_NAME_LEN],
					 int *count, const char *name,
					 int depth, const char *tag)
{
	char path[80];
	char needbuf[BOARD_CONFIG_OV_DEPS_LEN];
	const char *need;
	int node, need_len, nlen;
	const char *p, *q;

	if (depth > 6 || *count >= BOARD_CONFIG_OV_MAX)
		return -1;
	if (board_config_ov_already(apply, *count, name))
		return 0;

	nlen = strlen(name);
	if (nlen + 20 >= (int)sizeof(path))
		return -1;
	memcpy(path, "/overlay-deps/", 14);
	memcpy(path + 14, name, nlen + 1);
	node = fdt_path_offset(fit, path);
	if (node >= 0) {
		need = fdt_getprop(fit, node, "need", &need_len);
		if (need && need_len > 0) {
			if (need[need_len - 1] == '\0')
				need_len--;
			if (need_len >= (int)sizeof(needbuf)) {
				printf("board-config-%s: %s: dep list of %d bytes exceeds the %d-byte limit -- overlay SKIPPED\n",
				       tag, name, need_len,
				       (int)sizeof(needbuf) - 1);
				return -1;
			}
			if (need_len > 0) {
				memcpy(needbuf, need, need_len);
				needbuf[need_len] = '\0';
				p = needbuf;
				while (*p) {
					char dep[BOARD_CONFIG_OV_NAME_LEN];
					int dlen;

					while (*p == ' ')
						p++;
					if (!*p)
						break;
					q = p;
					while (*q && *q != ' ')
						q++;
					dlen = q - p;
					if (dlen >= BOARD_CONFIG_OV_NAME_LEN) {
						printf("board-config-%s: %s: dep name of %d bytes exceeds the %d-byte limit -- overlay SKIPPED\n",
						       tag, name, dlen,
						       BOARD_CONFIG_OV_NAME_LEN - 1);
						return -1;
					}
					if (dlen > 0) {
						memcpy(dep, p, dlen);
						dep[dlen] = '\0';
						if (board_config_ov_add_with_deps(fit, apply,
										  count, dep,
										  depth + 1,
										  tag))
							return -1;
					}
					p = q;
				}
			}
		}
	}

	if (board_config_ov_already(apply, *count, name))
		return 0;
	if (*count >= BOARD_CONFIG_OV_MAX)
		return -1;
	memcpy(apply[*count], name, nlen + 1);
	(*count)++;
	return 0;
}

/*
 * Expand @overlays (a space-separated name list) against @fit and apply each
 * resulting .dtbo to @fdt. @tag names the transport that supplied the block,
 * and prefixes every message: both transports funnel through here, so a
 * hard-coded prefix would misreport which one the operator is looking at.
 *
 * @fit must already be validated (FDT_MAGIC, totalsize <=
 * BOARD_CONFIG_FIT_MAX_SIZE) and must live at SCRATCH_ADDR: the writable
 * per-overlay copy is staged at ALIGN(fit_size, 64) bytes past it, and the
 * bound below is against the board-declared scratch window. Every transport
 * therefore stages its FIT in scratch before calling here -- the NOR path reads
 * it there directly, and the host-staged DRAM path relocates it there.
 */
static int board_config_apply_overlays(void *fdt, void *fit, u32 fit_size,
				       const char *overlays, const char *tag)
{
	const char *name, *next;
	int namelen;
	char path[64];
	int node;
	const void *dtbo_data;
	void *dtbo_copy;
	int dtbo_size;
	int fdt_size;
	int ret;

	{
		char apply[BOARD_CONFIG_OV_MAX][BOARD_CONFIG_OV_NAME_LEN];
		int apply_count = 0;
		int new_fdt_size;
		u32 dtbo_off;
		int i;

		name = overlays;
		while (*name) {
			char resolved[BOARD_CONFIG_OV_NAME_LEN];

			while (*name == ' ')
				name++;
			if (*name == '\0')
				break;
			next = name;
			while (*next && *next != ' ')
				next++;
			namelen = next - name;
			if (namelen == 0)
				break;

			if (board_config_fit_resolve_name(fit, name, namelen,
							  resolved)) {
				/* keep original name; apply path will error if missing */
				if (namelen >= BOARD_CONFIG_OV_NAME_LEN) {
					printf("board-config-%s: %.*s: overlay name of %d bytes exceeds the %d-byte limit -- SKIPPED\n",
					       tag, namelen, name, namelen,
					       BOARD_CONFIG_OV_NAME_LEN - 1);
					name = next;
					continue;
				}
				memcpy(resolved, name, namelen);
				resolved[namelen] = '\0';
			}
			if (board_config_ov_add_with_deps(fit, apply,
							  &apply_count,
							  resolved, 0, tag))
				printf("board-config-%s: dep expand failed for %s\n",
				       tag, resolved);
			name = next;
		}

		for (i = 0; i < apply_count; i++) {
			const char *oname = apply[i];
			int olen = strlen(oname);

			if (olen + 9 > (int)sizeof(path)) {
				printf("board-config-%s: %s: name does not fit the %d-byte FIT path buffer -- SKIPPED\n",
				       tag, oname, (int)sizeof(path));
				continue;
			}
			memcpy(path, "/images/", 8);
			memcpy(path + 8, oname, olen + 1);

			node = fdt_path_offset(fit, path);
			if (node < 0) {
				printf("board-config-%s: %s not found in FIT\n",
				       tag, path);
				continue;
			}

			dtbo_data = fdt_getprop(fit, node, "data", &dtbo_size);
			if (!dtbo_data || dtbo_size <= 0) {
				printf("board-config-%s: %s has no data\n",
				       tag, path);
				continue;
			}

			/*
			 * Bound both writes before making either. Neither
			 * region is discoverable at run time this early --
			 * the scratch window is whatever the board declared
			 * in BOARD_CONFIG_OVERLAY_SCRATCH_SIZE, and the
			 * headroom after the control FDT is whatever it
			 * declared in BOARD_CONFIG_OVERLAY_FDT_MAX_SIZE -- so
			 * the declaration is the bound. Dropping an overlay
			 * always beats writing past either: this runs at
			 * initcall #2, where a trampled neighbour or a
			 * corrupted control FDT surfaces much later as an
			 * unattributable hang, and where the console may not
			 * yet exist to say what happened.
			 */
			dtbo_off = ALIGN(fit_size, 64);
			if (dtbo_off > (u32)SCRATCH_SIZE ||
			    (u32)dtbo_size > (u32)SCRATCH_SIZE - dtbo_off) {
				printf("board-config-%s: %s: %d-byte overlay staged at scratch+0x%x leaves the 0x%x-byte scratch window -- SKIPPED\n",
				       tag, oname, dtbo_size, dtbo_off,
				       (u32)SCRATCH_SIZE);
				continue;
			}

			fdt_size = fdt_totalsize(fdt);
			new_fdt_size = fdt_size + dtbo_size +
				       BOARD_CONFIG_FDT_PAD;
			if (new_fdt_size > (int)BOARD_CONFIG_FDT_MAX_SIZE) {
				printf("board-config-%s: %s: control FDT would have to grow to %d bytes, past the 0x%x limit -- SKIPPED\n",
				       tag, oname, new_fdt_size,
				       (u32)BOARD_CONFIG_FDT_MAX_SIZE);
				continue;
			}

			ret = fdt_open_into(fdt, fdt, new_fdt_size);
			if (ret) {
				printf("board-config-%s: fdt_open_into failed: %s\n",
				       tag, fdt_strerror(ret));
				continue;
			}

			/*
			 * fdt_overlay_apply() modifies the overlay in-place
			 * (resolves phandles), so we need a writable copy.
			 * Place it after the FIT in scratch memory -- bounded
			 * just above.
			 */
			dtbo_copy = (u8 *)fit + dtbo_off;
			memcpy(dtbo_copy, dtbo_data, dtbo_size);

			ret = fdt_overlay_apply_verbose(fdt, dtbo_copy);
			if (ret)
				printf("board-config-%s: failed to apply %s\n",
				       tag, oname);
			else
				printf("board-config-%s: applied %s\n",
				       tag, oname);
		}
	}

	return 0;
}

#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
/*
 * NOR transport: read the config block and the overlay FIT from NOR into
 * scratch, then hand the name list to board_config_apply_overlays(). Called
 * from fdtdec_board_setup(). fdt is the control FDT (gd->fdt_blob) -- writable
 * at this point since reserve_fdt() hasn't run yet.
 */
static int board_config_nor_overlays(void *fdt)
{
	void *scratch = (void *)(uintptr_t)SCRATCH_ADDR;
	char overlay_buf[BOARD_CONFIG_OVERLAYS_LEN];
	bool truncated = false;
	const char *overlays;
	void *fit;
	u32 fit_size;
	u32 offset;
	int ret;

	/*
	 * Boot policy first, before the controller is touched: a board can
	 * rule that this boot must not act on what NOR stores (a USB recovery
	 * or flash-session boot). Said on the console, because "nothing was
	 * applied" must be told apart from "nothing was stored".
	 */
	if (board_config_nor_skip()) {
		printf("board-config-nor: skipped by boot policy -- stored overlays not applied\n");
		return 0;
	}

	/* Step 0: check NOR controller is clocked, init, and probe flash */
	if (!board_nor_clk_enabled())
		return 0;

	if (!board_nor_init())
		return 0;

	if (!board_nor_probe())
		return 0;

	/* After the probe: that is what makes the capacity, and so this, valid */
	offset = board_config_nor_offset();

	/* Step 1: read config block from NOR */
	ret = board_nor_read(offset, scratch, NOR_SECTOR_SIZE);
	if (ret == -ENOSYS)
		return 0;	/* no platform backend -- silent no-op */
	if (ret) {
		printf("board-config-nor: NOR read env failed\n");
		return 0;
	}

	/* Step 2: find overlays= key (handles both binary and text env) */
	if (!board_config_get_overlays(scratch, NOR_SECTOR_SIZE, overlay_buf,
				       sizeof(overlay_buf), &truncated)) {
		printf("board-config-nor: no overlays set\n");
		return 0;
	}

	overlays = overlay_buf;
	printf("board-config-nor: overlays=%s\n", overlays);
	if (truncated)
		printf("board-config-nor: overlays= exceeds the %d-byte limit and was TRUNCATED -- the overlays past the end will NOT be applied\n",
		       BOARD_CONFIG_OVERLAYS_LEN - 1);

	/* Step 3: read FIT header to get total size */
	fit = scratch;
	ret = board_nor_read(NOR_FIT_OFFSET, fit, FIT_HDR_SIZE);
	if (ret) {
		printf("board-config-nor: NOR read FIT header failed\n");
		return 0;
	}

	if (fdt_magic(fit) != FDT_MAGIC) {
		printf("board-config-nor: invalid FIT magic\n");
		return 0;
	}

	fit_size = fdt_totalsize(fit);
	if (fit_size > BOARD_CONFIG_FIT_MAX_SIZE) {
		printf("board-config-nor: FIT too large (%u)\n", fit_size);
		return 0;
	}

	/* Step 4: read full FIT into scratch */
	ret = board_nor_read(NOR_FIT_OFFSET, fit, fit_size);
	if (ret) {
		printf("board-config-nor: NOR read FIT failed\n");
		return 0;
	}

	/* Step 5: expand overlays= with auto-deps, then apply in order */
	return board_config_apply_overlays(fdt, fit, fit_size, overlays, "nor");
}
#endif /* CONFIG_BOARD_CONFIG_NOR */

#if IS_ENABLED(CONFIG_BOARD_CONFIG_DRAM)
/*
 * DRAM transport: apply overlays named by a host-staged block.
 *
 * Returns true if a valid block carrying a FIT was present -- whether or not
 * any overlay actually applied -- so the caller skips the NOR path. A USB
 * boot that staged a block is deliberately overriding whatever the NOR
 * sector says, and silently falling back to NOR after, say, a bad FIT magic
 * would boot a tree the operator did not ask for.
 */
static bool board_config_dram_overlays(void *fdt)
{
	void *scratch = (void *)(uintptr_t)SCRATCH_ADDR;
	char overlay_buf[BOARD_CONFIG_OVERLAYS_LEN];
	bool truncated = false;
	const char *env;
	void *fit;
	u32 env_size, fit_size;

	if (!board_config_dram_find(&env, &env_size, &fit, &fit_size) || !fit)
		return false;

	/*
	 * Bound the relocation below. BOARD_CONFIG_FIT_MAX_SIZE is asserted at
	 * compile time to fit the scratch window, so re-using it here means the
	 * memmove() cannot leave the window the board declared.
	 */
	if (fit_size > BOARD_CONFIG_FIT_MAX_SIZE) {
		printf("board-config-dram: FIT too large (%u)\n", fit_size);
		return true;
	}

	if (!board_config_get_overlays(env, env_size, overlay_buf,
				       sizeof(overlay_buf), &truncated)) {
		printf("board-config-dram: no overlays set\n");
		return true;
	}

	printf("board-config-dram: overlays=%s\n", overlay_buf);
	if (truncated)
		printf("board-config-dram: overlays= exceeds the %d-byte limit and was TRUNCATED -- the overlays past the end will NOT be applied\n",
		       BOARD_CONFIG_OVERLAYS_LEN - 1);

	/*
	 * The staged FIT follows the env text at whatever offset the host's
	 * payload happened to end on; libfdt walks require alignment, and
	 * applying in place faulted before relocation (a string pointer
	 * dereferenced as an address). Relocate it into the scratch window --
	 * unused on this path, since nothing was read from NOR -- which also
	 * puts it exactly where board_config_apply_overlays() expects to stage
	 * the per-overlay copies after it.
	 */
	memmove(scratch, fit, fit_size);

	if (fdt_magic(scratch) != FDT_MAGIC) {
		printf("board-config-dram: invalid FIT magic\n");
		return true;
	}

	if (fdt_totalsize(scratch) > fit_size) {
		printf("board-config-dram: FIT truncated\n");
		return true;
	}

	board_config_apply_overlays(fdt, scratch, fdt_totalsize(scratch),
				    overlay_buf, "dram");

	return true;
}
#endif /* CONFIG_BOARD_CONFIG_DRAM */
#endif /* CONFIG_BOARD_CONFIG_OVERLAYS */

/*
 * Import all key=value pairs from a config block into the u-boot env.
 * Called from misc_init_r (after env is loaded from FAT).
 * Config-block values override FAT env values.
 *
 * @tag names the transport that supplied @start, and prefixes every message:
 * both transports funnel through here.
 *
 * @rejected is incremented once per entry that was parsed but NOT applied.
 * env_set() enforces ENV_WRITEABLE_LIST, ENV_OVERWRITE and the ethaddr
 * write-once rule, so a config block may legitimately ask for a write the
 * environment refuses; such an entry must be reported and must not be counted
 * as imported. Returns the number of variables actually set.
 */
static int board_config_import_entries(const char *start, u32 size,
				       int *rejected, const char *tag)
{
	const char *p, *end, *eq;
	char key[BOARD_CONFIG_KEY_LEN], val[BOARD_CONFIG_VAL_LEN];
	int klen, vlen;
	int count = 0;
	int ret;

	end = start + size;

	for (p = start; p < end; ) {
		if (*p == '\0' || *p == '\n' || (unsigned char)*p == 0xff) {
			if (*p == '\0' && (p + 1 >= end || *(p + 1) == '\0'))
				break;
			if ((unsigned char)*p == 0xff)
				break;
			p++;
			continue;
		}

		eq = NULL;
		for (eq = p; eq < end && *eq != '=' && *eq != '\0' &&
		     *eq != '\n' && (unsigned char)*eq != 0xff; eq++)
			;

		if (eq >= end || *eq != '=') {
			while (p < end && *p != '\0' && *p != '\n' &&
			       (unsigned char)*p != 0xff)
				p++;
			continue;
		}

		klen = eq - p;
		if (klen == 0 || klen >= (int)sizeof(key)) {
			if (klen) {
				printf("board-env-%s: %.*s...: key of %d bytes exceeds the %d-byte limit -- entry SKIPPED\n",
				       tag, 16, p, klen, (int)sizeof(key) - 1);
				(*rejected)++;
			}
			p = eq + 1;
			while (p < end && *p != '\0' && *p != '\n' &&
			       (unsigned char)*p != 0xff)
				p++;
			continue;
		}

		memcpy(key, p, klen);
		key[klen] = '\0';

		p = eq + 1;
		vlen = 0;
		while (p + vlen < end &&
		       *(p + vlen) != '\0' && *(p + vlen) != '\n' &&
		       (unsigned char)*(p + vlen) != 0xff)
			vlen++;

		/*
		 * vlen is the length of the WHOLE value, not a capped copy.
		 * The old code capped it at sizeof(val) - 1 and then advanced
		 * p by that, which left p in the middle of the value: the
		 * remainder was re-parsed as a fresh entry, and any '=' in it
		 * became a bogus key. Measure first, then decide.
		 *
		 * A value that does not fit is skipped outright rather than
		 * truncated. Half a bootargs is not a smaller bootargs, it is
		 * a wrong one, and it would be applied under the right name.
		 */
		if (vlen >= (int)sizeof(val)) {
			printf("board-env-%s: %s: value of %d bytes exceeds the %d-byte limit -- entry SKIPPED\n",
			       tag, key, vlen, (int)sizeof(val) - 1);
			(*rejected)++;
			p += vlen;
			continue;
		}

		memcpy(val, p, vlen);
		val[vlen] = '\0';

		ret = env_set(key, val);
		if (ret) {
			/*
			 * Refused by env_set() -- not writeable, not
			 * overwriteable, or an ethaddr that is already set.
			 * The environment does NOT contain what the config
			 * block asked for; say so instead of printing the
			 * requested value as if it had been applied.
			 */
			printf("board-env-%s: %s: REJECTED by env_set (%d), not applied\n",
			       tag, key, ret);
			(*rejected)++;
			p += vlen;
			continue;
		}

		/*
		 * The value is in the environment now (printenv says it); the
		 * console gets the count below, not a second copy of the block --
		 * whose overlays= the overlay pass has already printed.
		 */
		debug("board-env-%s: %s=%s\n", tag, key, val);
		count++;

		p += vlen;
	}

	return count;
}

#if IS_ENABLED(CONFIG_BOARD_CONFIG_DRAM)
/* Import key=value entries from a host-staged DRAM block, if present. */
static int board_config_dram_env_import(void)
{
	const char *env;
	u32 env_size;
	int count, rejected = 0;

	if (!board_config_dram_find(&env, &env_size, NULL, NULL) || !env_size)
		return 0;

	if (!board_config_is_valid(env, env_size))
		return 0;

	count = board_config_import_entries(env, env_size, &rejected, "dram");

	if (count)
		printf("board-env-dram: imported %d variable%s\n", count, count == 1 ? "" : "s");
	if (rejected)
		printf("board-env-dram: %d entries NOT applied\n", rejected);

	return count;
}
#endif /* CONFIG_BOARD_CONFIG_DRAM */

#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
static int board_config_nor_env_import(void)
{
	char *buf;
	u32 env_crc, calc_crc, offset;
	int count, rejected = 0;
	int ret;

	if (!board_nor_clk_enabled())
		return 0;

	if (!board_nor_init())
		return 0;

	if (!board_nor_probe())
		return 0;

	/* After the probe: that is what makes the capacity, and so this, valid */
	offset = board_config_nor_offset();

	buf = malloc(NOR_SECTOR_SIZE);
	if (!buf)
		return 0;

	ret = board_nor_read(offset, buf, NOR_SECTOR_SIZE);
	if (ret == -ENOSYS) {
		free(buf);
		return 0;	/* no platform backend -- silent no-op */
	}
	if (ret) {
		printf("board-env-nor: NOR read failed\n");
		free(buf);
		return 0;
	}

	env_crc = le32_to_cpu(*(u32 *)buf);
	calc_crc = crc32(0, (unsigned char *)buf + 4, NOR_SECTOR_SIZE - 4);

	if (env_crc == calc_crc && board_config_is_valid(buf + 4,
							NOR_SECTOR_SIZE - 4))
		count = board_config_import_entries(buf + 4,
						    NOR_SECTOR_SIZE - 4,
						    &rejected, "nor");
	else if (board_config_is_valid(buf, NOR_SECTOR_SIZE))
		count = board_config_import_entries(buf, NOR_SECTOR_SIZE,
						    &rejected, "nor");
	else
		count = 0;	/* erased, or not a config block at all */

	/*
	 * "imported" counts only what the environment actually took. An absent
	 * or erased sector stays silent (count and rejected both 0); a sector
	 * whose entries were refused does not.
	 */
	if (count)
		printf("board-env-nor: imported %d variable%s\n", count, count == 1 ? "" : "s");
	if (rejected)
		printf("board-env-nor: %d entries NOT applied\n", rejected);

	free(buf);
	return count;
}
#endif /* CONFIG_BOARD_CONFIG_NOR */

int board_config_env_import(void)
{
	int count = 0;

	/*
	 * NOR first, then the host-staged DRAM block on top: env_set() makes
	 * the later import win, so a USB boot's overrides beat the NOR
	 * sector. On a NOR-less board the NOR half no-ops (board_nor_read()
	 * returns -ENOSYS) and only the DRAM half contributes.
	 */
#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
	count += board_config_nor_env_import();
#endif
#if IS_ENABLED(CONFIG_BOARD_CONFIG_DRAM)
	count += board_config_dram_env_import();
#endif

	return count;
}

#if IS_ENABLED(CONFIG_BOARD_CONFIG_OVERLAYS)
__weak void board_config_begin(void)
{
}

int fdtdec_board_setup(const void *fdt_blob)
{
	board_config_begin();
#if IS_ENABLED(CONFIG_BOARD_CONFIG_DRAM)
	if (board_config_dram_overlays((void *)fdt_blob))
		return 0;
#endif
#if IS_ENABLED(CONFIG_BOARD_CONFIG_NOR)
	return board_config_nor_overlays((void *)fdt_blob);
#else
	return 0;
#endif
}
#endif
