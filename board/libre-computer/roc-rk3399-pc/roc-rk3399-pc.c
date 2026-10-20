// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 */

#include <bootsource.h>
#include <command.h>
#include <dm.h>
#include <env.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <log.h>
#include <malloc.h>
#include <spl_gpio.h>
#include <string.h>
#include <usb_otg_role.h>
#include <vsprintf.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/libfdt.h>

#include <asm/arch-rockchip/clock.h>
#include <asm/arch-rockchip/cru.h>
#include <linux/delay.h>
#include <power/regulator.h>
#include <asm/arch-rockchip/gpio.h>
#include <asm/arch-rockchip/grf_rk3399.h>
#include <asm/arch-rockchip/sdram.h>
#include <asm/arch-rockchip/usbflash-alts.h>

#define PMUGRF_BASE	0xff320000
#define GPIO0_BASE	0xff720000

#ifdef CONFIG_XPL_BUILD

/**
 * LED setup for roc-rk3399-pc
 *
 * 1. Set the low power leds (only during POR, pwr_key env is 'y')
 *    glow yellow LED, termed as low power
 *    poll for on board power key press
 *    once powe key pressed, turn off yellow
 * 2. Turn on red LED, indicating full power mode
 */
void led_setup(void)
{
	struct rockchip_gpio_regs * const gpio0 = (void *)GPIO0_BASE;
	struct rk3399_pmugrf_regs * const pmugrf = (void *)PMUGRF_BASE;
	bool press_pwr_key = false;

	if (IS_ENABLED(CONFIG_SPL_ENV_SUPPORT)) {
		env_init();
		env_load();
		if (env_get_yesno("pwr_key") == 1)
			press_pwr_key = true;
	}

	if (press_pwr_key && !strcmp(get_reset_cause(), "POR")) {
		spl_gpio_output(gpio0, GPIO(BANK_A, 2), 1);

		spl_gpio_set_pull(&pmugrf->gpio0_p, GPIO(BANK_A, 5),
				  GPIO_PULL_NORMAL);
		while (readl(&gpio0->ext_port) & 0x20)
			;

		spl_gpio_output(gpio0, GPIO(BANK_A, 2), 0);
	}

	spl_gpio_output(gpio0, GPIO(BANK_B, 5), 1);
}

#endif

/*
 * CONFIG_ROC_RK3399_PC_CPU_L_MHZ: the little cluster U-Boot runs on, from the
 * 600 MHz rkclk_init() leaves to an RK3399 cluster-0 OPP: vdd_cpu_l to the
 * OPP's voltage first (Linux rk3399.dtsi), then the PLL. A rail that cannot be
 * set leaves the clock alone. U-Boot proper calls it from rk_board_late_init();
 * the SPL, when it has the RK808 (CONFIG_SPL_REGULATOR_RK8XX), before its USB
 * flash session, whose LZMA unpacking is bound by this core.
 */
static void roc_pc_cpu_l_clock(void)
{
	int mhz = CONFIG_ROC_RK3399_PC_CPU_L_MHZ, uv, ret;
	enum apll_l_frequencies f;
	struct udevice *reg;

	if (!CONFIG_IS_ENABLED(DM_REGULATOR))
		return;

	switch (mhz) {
	case 0:
		return;
	case 1008:
		f = APLL_L_1008_MHZ;
		uv = 925000;
		break;
	case 1200:
		f = APLL_L_1200_MHZ;
		uv = 1000000;
		break;
	case 1416:
		f = APLL_L_1416_MHZ;
		uv = 1125000;
		break;
	default:
		printf("CPU L: %d MHz is not a cluster-0 OPP, staying at 600 MHz\n",
		       mhz);
		return;
	}

	ret = regulator_get_by_platname("vdd_cpu_l", &reg);
	if (!ret)
		ret = regulator_set_value(reg, uv);
	if (ret) {
		printf("CPU L: vdd_cpu_l not set (%d), staying at 600 MHz\n", ret);
		return;
	}
	/* The ramp, 6001 uV/us in the DT: 300 mV in 50 us; give it 200 */
	udelay(200);
	rk3399_configure_cpu_l(rockchip_get_cru(), f);
	printf("CPU L: %d MHz at %d mV\n", mhz, regulator_get_value(reg) / 1000);
}

#ifdef CONFIG_XPL_BUILD
void rockchip_spl_usbflash_board(void)
{
	if (CONFIG_IS_ENABLED(REGULATOR_RK8XX))
		roc_pc_cpu_l_clock();
}
#endif

#if IS_ENABLED(CONFIG_ROC_RK3399_PC_REVFIX) && !defined(CONFIG_XPL_BUILD)

/*
 * Board revision, read from the LPDDR4 population.
 *
 * ROC-RK3399-PC is one board in three revisions (V1.0, V1.1-A, V1.2A). The
 * one hardware difference this SoC can see is the DRAM package: V1.0/V1.1-A
 * carry a single 366-pin LPDDR4 that presents TWO chip-selects per channel,
 * V1.2A carries two 200-ball packages that present ONE. Same 4 GiB either
 * way, so the capacity never told anyone -- the rank count does.
 *
 * TPL publishes what it detected in PMUGRF_OS_REG2
 * (drivers/ram/rockchip/sdram_rk3399.c, `writel(sys_reg2,
 * &dram->pmugrf->os_reg2)`), encoded by sdram_common.c's
 * SYS_REG_ENC_RANK(cap_info->rank, channel) with the shifts in
 * asm/arch-rockchip/sdram.h: SYS_REG_RANK_SHIFT(ch) = 11 + ch * 16, mask 1,
 * value = rank - 1. Measured on a V1.2A: OS_REG2 = 0x32c1f2c1 (bits 11 and
 * 27 clear, rank 1/1); on a V1.0 TF-A decodes the same register as
 * `rank 2/2, 2 ch`.
 *
 * V1.0 and V1.1-A share the 366-pin package. Production of that
 * population was V1.1-A, so a two-rank board is reported as v1.1. A V1.0
 * reads the same register and is treated as v1.1. An empty register, a
 * type other than LPDDR4, a channel count other than two, or mixed ranks
 * stays unknown.
 *
 * The rule presumes the 4 GiB population, which is the only one any drawing
 * describes (no LPDDR4 part number is on any sheet; both measured units read
 * "DRAM: 4 GiB"). A hypothetical 2 GiB single-rank 366-pin board would read
 * as V1.2A here; extending the signature to cs0_row/cs1 needs a V1.0 OS_REG2
 * value that has not been read.
 */
enum roc_pc_rev {
	ROC_PC_REV_UNKNOWN,
	ROC_PC_REV_V12,
	ROC_PC_REV_V11,
};

struct roc_pc_rev_info {
	u32 os_reg2;
	u32 rank[2];
	u32 num_ch;
	u32 dramtype;
	enum roc_pc_rev rev;
};

static const char *roc_pc_rev_name(enum roc_pc_rev rev)
{
	switch (rev) {
	case ROC_PC_REV_V12:
		return "v1.2";
	case ROC_PC_REV_V11:
		return "v1.1";
	default:
		return "unknown";
	}
}

static void roc_pc_revision(struct roc_pc_rev_info *ri)
{
	struct rk3399_pmugrf_regs * const pmugrf = (void *)PMUGRF_BASE;
	u32 v = readl(&pmugrf->os_reg2);
	int ch;

	ri->os_reg2 = v;
	ri->dramtype = (v >> SYS_REG_DDRTYPE_SHIFT) & SYS_REG_DDRTYPE_MASK;
	ri->num_ch = 1 + ((v >> SYS_REG_NUM_CH_SHIFT) & SYS_REG_NUM_CH_MASK);
	for (ch = 0; ch < 2; ch++)
		ri->rank[ch] = 1 + ((v >> SYS_REG_RANK_SHIFT(ch)) &
				    SYS_REG_RANK_MASK);

	ri->rev = ROC_PC_REV_UNKNOWN;

	/* DRAM absent / TPL never ran: nothing was published. */
	if (!v)
		return;
	/* Both revisions are dual-channel LPDDR4; anything else is not ours. */
	if (ri->dramtype != LPDDR4 || ri->num_ch != 2)
		return;
	/* Mixed rank across channels is not a population we ship. */
	if (ri->rank[0] != ri->rank[1])
		return;

	if (ri->rank[0] == 1)
		ri->rev = ROC_PC_REV_V12;
	else if (ri->rank[0] == 2)
		ri->rev = ROC_PC_REV_V11;
}

int rk_board_late_init(void);

int rk_board_late_init(void)
{
	struct roc_pc_rev_info ri;

	roc_pc_cpu_l_clock();
	roc_pc_revision(&ri);
	env_set("board_rev", roc_pc_rev_name(ri.rev));
	printf("roc-rk3399-pc: revision %s (OS_REG2 %08x, ranks ch0 %u ch1 %u)\n",
	       roc_pc_rev_name(ri.rev), ri.os_reg2, ri.rank[0], ri.rank[1]);

	return 0;
}

/*
 * Prepend one string to the root compatible list, keeping every existing
 * entry after it, so the kernel can match the revision first and fall back
 * to the board / SoC strings it matches today.
 *
 * Idempotent: a tree that already carries the string -- the kernel's
 * rk3399-roc-pc-v1.2.dts leads with it -- is left alone, so booting the
 * revision-specific DTB does not produce a doubled compatible.
 *
 * Bare board only: the string names a revision of ROC-RK3399-PC itself, so
 * it is prepended only to a tree whose root compatible LEADS with
 * "libre-computer,roc-rk3399-pc". A mezzanine tree
 * ("firefly,roc-rk3399-pc-mezzanine", "rockchip,rk3399") sits on the same
 * base board and gets the regulator fixup below, but no "-v1.2" variant of
 * its own compatible exists, and prepending the bare-board string would make
 * a mezzanine tree claim to be the bare board -- so its compatible is left
 * untouched. Any other tree (a foreign DTB booted on this hardware) is left
 * alone for the same reason.
 */
#define ROC_PC_BASE_COMPAT	"libre-computer,roc-rk3399-pc"

static int roc_pc_fdt_prepend_compatible(void *blob, const char *compat)
{
	const void *old;
	const char *first;
	int oldlen, newlen, ret, len;
	char *buf;

	if (!fdt_node_check_compatible(blob, 0, compat)) {
		printf("roc-rk3399-pc: fdt: root compatible already carries \"%s\"\n",
		       compat);
		return 0;
	}

	first = fdt_stringlist_get(blob, 0, "compatible", 0, &len);
	if (!first || len < 0 || strcmp(first, ROC_PC_BASE_COMPAT)) {
		printf("roc-rk3399-pc: fdt: root compatible leads with \"%s\", not the bare board; left as is\n",
		       first ? first : "(none)");
		return 0;
	}

	old = fdt_getprop(blob, 0, "compatible", &oldlen);
	if (!old || oldlen < 0)
		oldlen = 0;

	newlen = strlen(compat) + 1 + oldlen;
	buf = malloc(newlen);
	if (!buf)
		return -ENOMEM;

	strcpy(buf, compat);
	if (oldlen)
		memcpy(buf + strlen(compat) + 1, old, oldlen);

	ret = fdt_setprop(blob, 0, "compatible", buf, newlen);
	free(buf);
	if (ret)
		return ret;

	printf("roc-rk3399-pc: fdt: root compatible now leads with \"%s\"\n",
	       compat);
	return 0;
}

/*
 * V1.2A has no MP8859. The DC_12V buck-boost is an SC8903 (V1.2A schematic
 * p15 POWER_IN, U9), and that part has NO i2c: its 40 pins are power, sense,
 * and resistor-strapped MODE/ILIM/FREQ/FB, with /CE driven by an on-sheet
 * network (Q5/Q1, VBUS_TYPEC0 and DC_12V sense) and by no SoC GPIO. Output
 * is set by the FB divider R218 124K / R219 13.7K at 1.2 V, i.e. 12 V, from
 * VBUS_TYPEC0 through L7. Measured: bus 7 answers only 0x22 (fusb0) and
 * 0x66 is -EREMOTEIO.
 *
 * So on this revision dc_12v is a FIXED regulator with no control at all,
 * and the DT must say so rather than lose the rail. The kernel's dc_12v is a
 * CHILD of the mps,mp8859 node and sys_12v (the root of the whole rail tree)
 * names it as vin-supply; a supply whose node exists but never registers is
 * an -EPROBE_DEFER forever in regulator_dev_lookup(), so disabling mp8859
 * with that reference left in place would stop every regulator on the board
 * from probing. Simply dropping the reference (what this fixup did first)
 * keeps the kernel booting but truncates the tree: sys_12v becomes a root
 * with no parent and the VBUS_TYPEC0 -> 12 V hop disappears.
 *
 * Instead: for every child of the mp8859 node that carries a phandle, add a
 * root-level regulator-fixed node with the same regulator-name, voltage
 * constraints, always-on/boot-on and vin-supply (VBUS_TYPEC0, as wired),
 * give it a fresh phandle, re-point every vin-supply in the tree from the
 * child to the new node, and only then disable the mp8859 node. The kernel
 * sees a complete chain: vcc_vbus_typec0 -> dc_12v (fixed) -> sys_12v.
 */
#define ROC_PC_MAX_SUPPLIES	4
#define ROC_PC_SUPPLY_NAME_LEN	32

struct roc_pc_supply {
	char name[ROC_PC_SUPPLY_NAME_LEN];	/* regulator-name */
	u32 phandle;				/* child's phandle */
	u32 vin;				/* child's vin-supply, 0 if none */
	u32 min_uv, max_uv;			/* 0 if absent */
	bool always_on, boot_on;
};

static int roc_pc_fdt_u32(const void *blob, int node, const char *name,
			  u32 *out)
{
	const fdt32_t *prop;
	int len;

	prop = fdt_getprop(blob, node, name, &len);
	if (!prop || len != sizeof(*prop))
		return -FDT_ERR_NOTFOUND;
	*out = fdt32_to_cpu(*prop);
	return 0;
}

/* Read everything about one mp8859 child before the tree is mutated. */
static int roc_pc_read_supply(const void *blob, int child,
			      struct roc_pc_supply *s)
{
	const char *name;
	int len;

	memset(s, 0, sizeof(*s));
	s->phandle = fdt_get_phandle(blob, child);
	if (!s->phandle)
		return -FDT_ERR_NOTFOUND;

	name = fdt_getprop(blob, child, "regulator-name", &len);
	if (!name || len <= 0 || len > ROC_PC_SUPPLY_NAME_LEN)
		return -FDT_ERR_BADVALUE;
	memcpy(s->name, name, len);
	s->name[ROC_PC_SUPPLY_NAME_LEN - 1] = '\0';

	roc_pc_fdt_u32(blob, child, "vin-supply", &s->vin);
	roc_pc_fdt_u32(blob, child, "regulator-min-microvolt", &s->min_uv);
	roc_pc_fdt_u32(blob, child, "regulator-max-microvolt", &s->max_uv);
	s->always_on = fdt_getprop(blob, child, "regulator-always-on",
				   NULL) != NULL;
	s->boot_on = fdt_getprop(blob, child, "regulator-boot-on",
				 NULL) != NULL;
	return 0;
}

/*
 * Add /regulator-<name> as a regulator-fixed with the child's constraints
 * and a fresh phandle. Returns the new phandle through *newph.
 */
static int roc_pc_add_fixed_supply(void *blob, const struct roc_pc_supply *s,
				   u32 *newph)
{
	char nodename[ROC_PC_SUPPLY_NAME_LEN + 16];
	int node, ret, i;

	strcpy(nodename, "regulator-");
	strncat(nodename, s->name, ROC_PC_SUPPLY_NAME_LEN);
	for (i = 0; nodename[i]; i++)
		if (nodename[i] == '_')
			nodename[i] = '-';

	ret = fdt_generate_phandle(blob, newph);
	if (ret)
		return ret;

	node = fdt_add_subnode(blob, 0, nodename);
	if (node < 0)
		return node;

	ret = fdt_setprop_string(blob, node, "compatible", "regulator-fixed");
	if (!ret)
		ret = fdt_setprop_string(blob, node, "regulator-name", s->name);
	if (!ret && s->min_uv)
		ret = fdt_setprop_u32(blob, node, "regulator-min-microvolt",
				      s->min_uv);
	if (!ret && s->max_uv)
		ret = fdt_setprop_u32(blob, node, "regulator-max-microvolt",
				      s->max_uv);
	if (!ret && s->always_on)
		ret = fdt_setprop_empty(blob, node, "regulator-always-on");
	if (!ret && s->boot_on)
		ret = fdt_setprop_empty(blob, node, "regulator-boot-on");
	if (!ret && s->vin)
		ret = fdt_setprop_u32(blob, node, "vin-supply", s->vin);
	if (!ret)
		ret = fdt_setprop_u32(blob, node, "phandle", *newph);
	if (ret)
		return ret;

	printf("roc-rk3399-pc: fdt: added /%s (regulator-fixed \"%s\", %u uV, vin-supply phandle %u, phandle %u)\n",
	       nodename, s->name, s->min_uv, s->vin, *newph);
	return 0;
}

/* Re-point every vin-supply = <oldph> in the tree to newph. */
static int roc_pc_retarget_vin_supply(void *blob, u32 oldph, u32 newph)
{
	int off, ret, n = 0;
	u32 v;

	for (off = fdt_next_node(blob, -1, NULL); off >= 0;
	     off = fdt_next_node(blob, off, NULL)) {
		if (roc_pc_fdt_u32(blob, off, "vin-supply", &v) || v != oldph)
			continue;
		/* Same size in place: no offset shift, the walk continues. */
		ret = fdt_setprop_u32(blob, off, "vin-supply", newph);
		if (ret)
			return ret;
		printf("roc-rk3399-pc: fdt: %s vin-supply phandle %u -> %u\n",
		       fdt_get_name(blob, off, NULL), oldph, newph);
		n++;
	}
	return n;
}

static int roc_pc_fdt_disable_mp8859(void *blob)
{
	struct roc_pc_supply sup[ROC_PC_MAX_SUPPLIES];
	int node, child, ret, i, nsup = 0, n = 0;

	/*
	 * No mp8859 node at all is the rk3399-roc-pc-v1.2.dts case: that DTS
	 * deletes the node and carries the root dc_12v itself, so there is
	 * nothing left for this fixup to do. Not an error.
	 */
	node = fdt_node_offset_by_compatible(blob, -1, "mps,mp8859");
	if (node < 0) {
		printf("roc-rk3399-pc: fdt: no mps,mp8859 node (%s), dc_12v already fixed\n",
		       fdt_strerror(node));
		return 0;
	}

	/*
	 * A node that is already disabled is the other already-fixed shape: a
	 * blob this fixup ran on once before (the bench command run twice, or
	 * a tree saved after fixup). Its fixed supply is already in place, and
	 * adding it again only fails with FDT_ERR_EXISTS -- which the second
	 * run used to print as "fdt fixup failed" over a tree that was right.
	 */
	if (!fdtdec_get_is_enabled(blob, node)) {
		printf("roc-rk3399-pc: fdt: %s already disabled, dc_12v already fixed\n",
		       fdt_get_name(blob, node, NULL));
		return 0;
	}

	/*
	 * Read the children BEFORE mutating anything: adding a root node and
	 * disabling this one both shift every offset after them.
	 */
	fdt_for_each_subnode(child, blob, node) {
		if (nsup < ROC_PC_MAX_SUPPLIES &&
		    !roc_pc_read_supply(blob, child, &sup[nsup]))
			nsup++;
	}

	for (i = 0; i < nsup; i++) {
		u32 newph;

		ret = roc_pc_add_fixed_supply(blob, &sup[i], &newph);
		if (ret)
			return ret;
		ret = roc_pc_retarget_vin_supply(blob, sup[i].phandle, newph);
		if (ret < 0)
			return ret;
		n += ret;
	}

	/* Offsets moved with the additions above; look the node up again. */
	node = fdt_node_offset_by_compatible(blob, -1, "mps,mp8859");
	if (node < 0)
		return node;

	ret = fdt_set_node_status(blob, node, FDT_STATUS_DISABLED);
	if (ret < 0)
		return ret;

	printf("roc-rk3399-pc: fdt: %s status = \"disabled\" (%d fixed supplies added, %d vin-supply refs re-pointed)\n",
	       fdt_get_name(blob, node, NULL), nsup, n);
	return 0;
}

/*
 * V1.2A takes GPIO0_A1 and GPIO0_B4 back from the expansion header (J20 pins
 * 25/26 are unconnected on this revision) for the eDP touch-panel level
 * shifters: GPIO0_A1 -> TP_EN_EDP (Q6), GPIO0_B4 -> TP_INT_EDP (Q2),
 * schematic V1.2A p28. The kernel's rk3399-roc-pc-v1.2.dts names the two
 * lines so; the shared rk3399-roc-pc.dtsi names them by pin ("GPIO0_A1",
 * "GPIO0_B4") because on V1.0/V1.1-A they are header pins. Booting a plain
 * tree on a V1.2A, rename exactly those two entries of gpio0's
 * gpio-line-names so the fixed-up tree describes the same hardware the
 * revision DTS does -- a header pin that is not a header pin is a wiring
 * fact, not a label.
 *
 * Only an entry that still carries the base-board pin name is touched: a
 * tree that already names them (the -v1.2 DTS, or this fixup run twice) is
 * left alone, and a tree that names them anything else, or has no
 * gpio-line-names at all (the Plus DTS deletes the property), is not ours
 * to rewrite.
 */
#define ROC_PC_GPIO0_REG	0xff720000

static const struct {
	int line;
	const char *base;
	const char *v12;
} roc_pc_v12_lines[] = {
	{ 1, "GPIO0_A1", "TP_EN_EDP" },
	{ 12, "GPIO0_B4", "TP_INT_EDP" },
};

static int roc_pc_fdt_v12_line_names(void *blob)
{
	const char *prop = "gpio-line-names";
	const char *s;
	char *buf, *p;
	int node, count, i, j, len, newlen, ret, renamed = 0;

	node = fdt_node_offset_by_compat_reg(blob, "rockchip,gpio-bank",
					     ROC_PC_GPIO0_REG);
	if (node < 0) {
		printf("roc-rk3399-pc: fdt: no gpio bank at %08x (%s), line names left alone\n",
		       ROC_PC_GPIO0_REG, fdt_strerror(node));
		return 0;
	}

	count = fdt_stringlist_count(blob, node, prop);
	if (count < 0) {
		printf("roc-rk3399-pc: fdt: %s has no %s, left alone\n",
		       fdt_get_name(blob, node, NULL), prop);
		return 0;
	}

	/* Every string may grow by the longest rename; bound the buffer by that. */
	fdt_getprop(blob, node, prop, &len);
	newlen = len;
	for (j = 0; j < ARRAY_SIZE(roc_pc_v12_lines); j++)
		newlen += strlen(roc_pc_v12_lines[j].v12) + 1;
	buf = malloc(newlen);
	if (!buf)
		return -ENOMEM;

	p = buf;
	for (i = 0; i < count; i++) {
		s = fdt_stringlist_get(blob, node, prop, i, &len);
		if (!s || len < 0) {
			free(buf);
			return len < 0 ? len : -FDT_ERR_BADVALUE;
		}
		for (j = 0; j < ARRAY_SIZE(roc_pc_v12_lines); j++) {
			if (i == roc_pc_v12_lines[j].line &&
			    !strcmp(s, roc_pc_v12_lines[j].base)) {
				printf("roc-rk3399-pc: fdt: %s line %d \"%s\" -> \"%s\"\n",
				       fdt_get_name(blob, node, NULL), i, s,
				       roc_pc_v12_lines[j].v12);
				s = roc_pc_v12_lines[j].v12;
				renamed++;
				break;
			}
		}
		len = strlen(s) + 1;
		memcpy(p, s, len);
		p += len;
	}

	if (!renamed) {
		free(buf);
		printf("roc-rk3399-pc: fdt: %s line names already V1.2A\n",
		       fdt_get_name(blob, node, NULL));
		return 0;
	}

	ret = fdt_setprop(blob, node, prop, buf, p - buf);
	free(buf);
	return ret;
}

#if IS_ENABLED(CONFIG_ROC_RK3399_PC_BENCH_CLKCAP)
/*
 * Bench-only clock cap, gated on the environment variable bench_clkcap.
 *
 * On the NYC bench the board is powered from ONE USB hub port's 5 V VBUS,
 * and at full DVFS (A53 1416 / A72 1800 / GPU 800 MHz) a sustained load
 * browns the rail out and resets the SoC to MaskROM (measured 2026-09-16).
 * Our own RAM-boot harness caps the clocks from its initramfs, but a
 * foreign kernel booted through the product order (EFI -> GRUB -> Debian)
 * never runs that initramfs, so the only place the cap can come from is
 * the device tree U-Boot hands over.
 *
 * With bench_clkcap=1 every opp-* node whose opp-hz is above the ceiling
 * is deleted from the OPP table each consumer points at: 816 MHz for the
 * arm,cortex-a53 cluster, 1008 MHz for arm,cortex-a72, 500 MHz for the
 * arm,mali-t860 GPU -- the highest OPP per domain at <= 0.9 V, roughly
 * half the ceilings. Tables are found through the consumers'
 * operating-points-v2 phandles and nodes are judged by opp-hz, never by
 * name, so the cap survives a renumbered or foreign table. The DDR table
 * is untouched (its governor idles it on its own). The kernel then reports
 * the ceiling as cpuinfo_max_freq / devfreq max_freq, which is how a run
 * proves the cap was applied.
 *
 * Unset (the product default) this is a no-op with no output: a product
 * boot never sees it. Never set it in a defconfig or a saved product env.
 */
#define ROC_PC_CAP_A53_HZ	816000000ULL
#define ROC_PC_CAP_A72_HZ	1008000000ULL
#define ROC_PC_CAP_GPU_HZ	500000000ULL

static const struct {
	const char *compat;	/* the OPP consumer */
	u64 max_hz;
} roc_pc_clkcaps[] = {
	{ "arm,cortex-a53", ROC_PC_CAP_A53_HZ },
	{ "arm,cortex-a72", ROC_PC_CAP_A72_HZ },
	{ "arm,mali-t860", ROC_PC_CAP_GPU_HZ },
};

/*
 * Delete every child of the OPP table with phandle tbl_ph whose opp-hz is
 * above max_hz. Each deletion shifts the offsets after it, so the walk
 * restarts from the table after every one. Returns the number removed.
 */
static int roc_pc_fdt_cap_opp_table(void *blob, u32 tbl_ph, u64 max_hz)
{
	int tbl, child, ret, len, removed = 0;
	const fdt64_t *hz;

again:
	tbl = fdt_node_offset_by_phandle(blob, tbl_ph);
	if (tbl < 0)
		return tbl;

	fdt_for_each_subnode(child, blob, tbl) {
		hz = fdt_getprop(blob, child, "opp-hz", &len);
		if (!hz || len != sizeof(*hz))
			continue;
		if (fdt64_ld(hz) <= max_hz)
			continue;
		ret = fdt_del_node(blob, child);
		if (ret)
			return ret;
		removed++;
		goto again;
	}
	return removed;
}

static int roc_pc_fdt_bench_clkcap(void *blob)
{
	int i, off, ret, len, n, total = 0;
	const fdt32_t *ph;

	if (env_get_yesno("bench_clkcap") != 1)
		return 0;

	for (i = 0; i < ARRAY_SIZE(roc_pc_clkcaps); i++) {
		for (off = fdt_node_offset_by_compatible(blob, -1,
						roc_pc_clkcaps[i].compat);
		     off >= 0;
		     off = fdt_node_offset_by_compatible(blob, off,
						roc_pc_clkcaps[i].compat)) {
			ph = fdt_getprop(blob, off, "operating-points-v2", &len);
			if (!ph || len < sizeof(*ph))
				continue;
			/* Only the first table: the RK3399 consumers name one. */
			ret = roc_pc_fdt_cap_opp_table(blob, fdt32_to_cpu(*ph),
						       roc_pc_clkcaps[i].max_hz);
			if (ret < 0)
				return ret;
			total += ret;
		}
	}

	n = total;
	printf("roc-rk3399-pc: bench_clkcap=1: OPPs capped at A53 %llu / A72 %llu / GPU %llu MHz (%d opp nodes removed)\n",
	       ROC_PC_CAP_A53_HZ / 1000000, ROC_PC_CAP_A72_HZ / 1000000,
	       ROC_PC_CAP_GPU_HZ / 1000000, n);
	return 0;
}
#else
static inline int roc_pc_fdt_bench_clkcap(void *blob) { return 0; }
#endif /* CONFIG_ROC_RK3399_PC_BENCH_CLKCAP */

int ft_board_setup(void *blob, struct bd_info *bd)
{
	struct roc_pc_rev_info ri;
	int ret;

	roc_pc_revision(&ri);

	switch (ri.rev) {
	case ROC_PC_REV_V12:
		ret = roc_pc_fdt_prepend_compatible(blob,
				"libre-computer,roc-rk3399-pc-v1.2");
		if (ret)
			break;
		ret = roc_pc_fdt_disable_mp8859(blob);
		if (ret)
			break;
		ret = roc_pc_fdt_v12_line_names(blob);
		break;
	case ROC_PC_REV_V11:
		/*
		 * V1.0 and V1.1-A read identically (same DRAM package).
		 * Production of that population was V1.1-A, so the string
		 * is v1.1. The MP8859 and the header GPIO names stay: both
		 * are still true on V1.1-A.
		 */
		ret = roc_pc_fdt_prepend_compatible(blob,
				"libre-computer,roc-rk3399-pc-v1.1");
		break;
	default:
		printf("roc-rk3399-pc: fdt: revision unknown (OS_REG2 %08x), no fixup\n",
		       ri.os_reg2);
		ret = 0;
		break;
	}

	if (ret)
		printf("roc-rk3399-pc: fdt fixup failed: %s\n", fdt_strerror(ret));

	/* Independent of the revision: the bench powers every unit the same way. */
	ret = roc_pc_fdt_bench_clkcap(blob);
	if (ret)
		printf("roc-rk3399-pc: bench_clkcap fixup failed: %s\n",
		       fdt_strerror(ret));

	/* env usb_otg -> the Type-C OTG controller's dr_mode; reports its own failures */
	usb_otg_fdt_fixup(blob);

	/* /chosen boot source: on this live tree EVT_FT_FIXUP is not sent */
	ret = bootsource_fdt_fixup(blob);
	if (ret)
		printf("roc-rk3399-pc: boot source not written to /chosen: %s\n",
		       fdt_strerror(ret));

	/* A failed fixup must not stop the boot; the DT is still valid. */
	return 0;
}

#if IS_ENABLED(CONFIG_ROC_RK3399_PC_REVFIX_CMD)
/*
 * Bench-only: run ft_board_setup() on a blob the user copied somewhere with
 * room (`fdt move $fdtcontroladdr <addr> <size>`), so the fixup can be read
 * back with `fdt print` on a board that has nothing to bootm.
 */
static int do_roc_revfix_test(struct cmd_tbl *cmdtp, int flag, int argc,
			      char *const argv[])
{
	void *blob;
	int ret;

	if (argc != 2)
		return CMD_RET_USAGE;

	blob = (void *)hextoul(argv[1], NULL);
	ret = fdt_check_header(blob);
	if (ret) {
		printf("bad fdt at %p: %s\n", blob, fdt_strerror(ret));
		return CMD_RET_FAILURE;
	}

	ret = ft_board_setup(blob, NULL);
	printf("ft_board_setup(%p) = %d\n", blob, ret);
	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(roc_revfix_test, 2, 0, do_roc_revfix_test,
	   "run the ROC-RK3399-PC revision fixup on a device tree blob",
	   "<fdt_addr>");
#endif /* CONFIG_ROC_RK3399_PC_REVFIX_CMD */

#endif /* CONFIG_ROC_RK3399_PC_REVFIX && !CONFIG_XPL_BUILD */
