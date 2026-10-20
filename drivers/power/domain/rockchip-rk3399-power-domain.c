// SPDX-License-Identifier: GPL-2.0+
/*
 * Rockchip RK3399 power-domain controller ("rockchip,rk3399-power-controller")
 *
 * A minimal provider for the 33 `power-domains = <&power RK3399_PD_*>`
 * consumers in rk3399-base.dtsi, which is shared byte-for-byte with Linux
 * and therefore cannot be re-pointed at anything else.
 *
 * The transition is Linux's rockchip_pd_power() (drivers/pmdomain/rockchip/
 * pm-domains.c), body only -- the domain table, the six steps, the two-stage
 * idle handshake, the inverted PWRDN_ST and the QoS save/restore. Nothing of
 * genpd, pm_clk or the idle-time power-off machinery is here: a bootloader
 * powers domains ON and hands off.
 *
 * Register facts that decide the shape of the code:
 *
 *  - PMU_PWRDN_ST is INVERTED: 1'b0 = powered on, 1'b1 = powered off.
 *  - The idle handshake is two polls. PMU_BUS_IDLE_ACK asserts before the
 *    NIU has drained; PMU_BUS_IDLE_ST is the one that means it.
 *  - The handshake is CLOCKED. An idle request to a domain whose bus clock
 *    is gated never acknowledges, so the per-domain `clocks` in the DT are
 *    ungated before any request is written.
 *  - These PMU registers have no hiword write-mask: read-modify-write.
 *  - Three domain shapes: power + idle (most), idle-only (vopb, vopl, cci0,
 *    cci1 -- no rail; "on" is answered from the idle state) and power-only
 *    (tcpd0, tcpd1, cci, vo).
 *
 * PMU_BUS_IDLE_REQ is shared with BL31. Its DFS DMA hold (dfs.c
 * dfs_dma_hold()) reads the register, requests idle only on NIUs whose bit
 * is NOT already set, and releases only what it set -- on the stated
 * assumption that "an OS keeps the idle request asserted for a domain it
 * has powered off". Its suspend path (pmu.c pmu_power_domains_suspend())
 * leaves a powered-off domain the same way. So a domain this driver powers
 * OFF is left with its request ASSERTED, exactly as Linux and BL31 leave it;
 * releasing it would invite BL31 to idle a dead NIU on the next DDR switch
 * and refuse the switch. What this driver guarantees instead is that it
 * never RETURNS mid-transition: every poll is bounded, and a failed step is
 * unwound before the error is reported.
 *
 * Clocks are ungated for the transition and left ungated afterwards, on both
 * paths. Linux drops its reference at the end because its clock framework
 * refcounts; U-Boot's clk_disable() writes the gate bit, and the consumers
 * on this SoC rely on the reset (ungated) state rather than enabling their
 * own clocks. Re-gating here would take a clock away from the device the
 * transition was made for.
 */

#define LOG_CATEGORY UCLASS_POWER_DOMAIN

#include <clk.h>
#include <command.h>
#include <dm.h>
#include <log.h>
#include <power-domain-uclass.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <dm/ofnode.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/iopoll.h>
#include <dt-bindings/power/rk3399-power.h>

#define PMU_PWRDN_CON		0x14
#define PMU_PWRDN_ST		0x18
#define PMU_BUS_IDLE_REQ	0x60
#define PMU_BUS_IDLE_ST		0x64
#define PMU_BUS_IDLE_ACK	0x68

/* NoC QoS generator, one 0x20 window per master (rockchip,rk3399-qos) */
#define QOS_PRIORITY		0x08
#define QOS_MODE		0x0c
#define QOS_BANDWIDTH		0x10
#define QOS_SATURATION		0x14
#define QOS_EXTCONTROL		0x18
#define QOS_NREGS		5

#define RK3399_PD_NUM		(RK3399_PD_PMU + 1)
#define RK3399_PD_MAX_QOS	2
#define RK3399_PD_MAX_CLKS	4

/* Linux bounds both the rail poll and each idle poll at 10 ms */
#define RK3399_PD_TIMEOUT_US	10000

/**
 * struct rk3399_pd_info - one row of Linux's rk3399_pm_domains[]
 * @name: domain name, as Linux names it
 * @pwr_mask: bit in PMU_PWRDN_CON / PMU_PWRDN_ST; 0 for an idle-only domain
 * @req_mask: bit in PMU_BUS_IDLE_REQ / _ST / _ACK; 0 for a power-only domain
 *
 * DOMAIN_RK3399() sets status == pwr and idle == ack == req, so two masks
 * carry the whole row.
 */
struct rk3399_pd_info {
	const char *name;
	u32 pwr_mask;
	u32 req_mask;
};

static const struct rk3399_pd_info rk3399_pd_info[RK3399_PD_NUM] = {
	[RK3399_PD_TCPD0]	= { "tcpd0",     BIT(8),  0 },
	[RK3399_PD_TCPD1]	= { "tcpd1",     BIT(9),  0 },
	[RK3399_PD_CCI]		= { "cci",       BIT(10), 0 },
	[RK3399_PD_CCI0]	= { "cci0",      0,       BIT(15) },
	[RK3399_PD_CCI1]	= { "cci1",      0,       BIT(16) },
	[RK3399_PD_PERILP]	= { "perilp",    BIT(11), BIT(1) },
	[RK3399_PD_PERIHP]	= { "perihp",    BIT(12), BIT(2) },
	[RK3399_PD_CENTER]	= { "center",    BIT(13), BIT(14) },
	[RK3399_PD_VIO]		= { "vio",       BIT(14), BIT(17) },
	[RK3399_PD_GPU]		= { "gpu",       BIT(15), BIT(0) },
	[RK3399_PD_VCODEC]	= { "vcodec",    BIT(16), BIT(3) },
	[RK3399_PD_VDU]		= { "vdu",       BIT(17), BIT(4) },
	[RK3399_PD_RGA]		= { "rga",       BIT(18), BIT(5) },
	[RK3399_PD_IEP]		= { "iep",       BIT(19), BIT(6) },
	[RK3399_PD_VO]		= { "vo",        BIT(20), 0 },
	[RK3399_PD_VOPB]	= { "vopb",      0,       BIT(7) },
	[RK3399_PD_VOPL]	= { "vopl",      0,       BIT(8) },
	[RK3399_PD_ISP0]	= { "isp0",      BIT(22), BIT(9) },
	[RK3399_PD_ISP1]	= { "isp1",      BIT(23), BIT(10) },
	[RK3399_PD_HDCP]	= { "hdcp",      BIT(24), BIT(11) },
	[RK3399_PD_GMAC]	= { "gmac",      BIT(25), BIT(23) },
	[RK3399_PD_EMMC]	= { "emmc",      BIT(26), BIT(24) },
	[RK3399_PD_USB3]	= { "usb3",      BIT(27), BIT(12) },
	[RK3399_PD_EDP]		= { "edp",       BIT(28), BIT(22) },
	[RK3399_PD_GIC]		= { "gic",       BIT(29), BIT(27) },
	[RK3399_PD_SD]		= { "sd",        BIT(30), BIT(28) },
	[RK3399_PD_SDIOAUDIO]	= { "sdioaudio", BIT(31), BIT(29) },
};

/**
 * struct rk3399_pd_state - per-domain runtime state
 * @present: the DT has a power-domain@ node for this id
 * @parent: id of the enclosing power-domain@ node, or -1 at the top level
 * @nqos: number of pm_qos windows (0..RK3399_PD_MAX_QOS)
 * @qos: their base addresses
 * @qos_save: their contents, saved on power-off and restored on power-on
 * @node: the power-domain@ node, for its `clocks`
 * @refcount: consumers holding the domain on through power_domain_on()
 *
 * Static rather than in the device's priv on purpose. The uclass removes a
 * provider after every consumer power-off (power-domain-uclass.c
 * dev_power_domain_ctrl(): "power_domain_get() bound the device, thus we
 * must remove it again"), which frees priv -- and a refcount or a saved QoS
 * set that disappears between one consumer's off and the next's is worse
 * than none. probe() refills the DT-derived fields idempotently.
 */
struct rk3399_pd_state {
	bool present;
	s8 parent;
	u8 nqos;
	phys_addr_t qos[RK3399_PD_MAX_QOS];
	u32 qos_save[RK3399_PD_MAX_QOS][QOS_NREGS];
	ofnode node;
	int refcount;
};

static struct rk3399_pd_state rk3399_pd_state[RK3399_PD_NUM];
static void __iomem *rk3399_pmu_base;

static bool rk3399_pd_valid(unsigned long id)
{
	return id < RK3399_PD_NUM && rk3399_pd_info[id].name &&
	       rk3399_pd_state[id].present;
}

static u32 rk3399_pmu_read(u32 off)
{
	return readl(rk3399_pmu_base + off);
}

static bool rk3399_pd_is_idle(unsigned long id)
{
	u32 mask = rk3399_pd_info[id].req_mask;

	return (rk3399_pmu_read(PMU_BUS_IDLE_ST) & mask) == mask;
}

static bool rk3399_pd_is_on(unsigned long id)
{
	const struct rk3399_pd_info *info = &rk3399_pd_info[id];

	/* idle-only domain: no rail to read, "on" means "not idled" */
	if (!info->pwr_mask)
		return !rk3399_pd_is_idle(id);

	/* 1'b0: power on, 1'b1: power off */
	return !(rk3399_pmu_read(PMU_PWRDN_ST) & info->pwr_mask);
}

static void rk3399_pd_log_regs(const char *what, unsigned long id)
{
	log_debug("pd %s: %s: PWRDN_CON %08x ST %08x IDLE_REQ %08x ST %08x ACK %08x\n",
		  rk3399_pd_info[id].name, what,
		  rk3399_pmu_read(PMU_PWRDN_CON), rk3399_pmu_read(PMU_PWRDN_ST),
		  rk3399_pmu_read(PMU_BUS_IDLE_REQ), rk3399_pmu_read(PMU_BUS_IDLE_ST),
		  rk3399_pmu_read(PMU_BUS_IDLE_ACK));
}

/*
 * The NIU idle handshake: write the request, wait for the ACK, then wait
 * for the idle STATUS -- the ACK alone only says the request was seen.
 */
static int rk3399_pd_set_idle(unsigned long id, bool idle)
{
	const struct rk3399_pd_info *info = &rk3399_pd_info[id];
	u32 mask = info->req_mask;
	u32 target = idle ? mask : 0;
	u32 val;
	int ret;

	if (!mask)
		return 0;

	clrsetbits_le32(rk3399_pmu_base + PMU_BUS_IDLE_REQ, mask, target);

	ret = readl_poll_timeout(rk3399_pmu_base + PMU_BUS_IDLE_ACK, val,
				 (val & mask) == target, RK3399_PD_TIMEOUT_US);
	if (ret) {
		log_err("pd %s: no idle ack (want %s), ACK %08x\n", info->name,
			idle ? "idle" : "active", val);
		return ret;
	}

	ret = readl_poll_timeout(rk3399_pmu_base + PMU_BUS_IDLE_ST, val,
				 (val & mask) == target, RK3399_PD_TIMEOUT_US);
	if (ret) {
		log_err("pd %s: idle status did not follow (want %s), ST %08x\n",
			info->name, idle ? "idle" : "active", val);
		return ret;
	}

	return 0;
}

static int rk3399_pd_set_power(unsigned long id, bool on)
{
	const struct rk3399_pd_info *info = &rk3399_pd_info[id];
	u32 mask = info->pwr_mask;
	u32 val;
	int ret;

	if (!mask)
		return 0;

	/* PWRDN_CON: 1 = request power-down */
	clrsetbits_le32(rk3399_pmu_base + PMU_PWRDN_CON, mask, on ? 0 : mask);

	/* PWRDN_ST: 1'b0 = on, 1'b1 = off */
	ret = readl_poll_timeout(rk3399_pmu_base + PMU_PWRDN_ST, val,
				 !!(val & mask) == !on, RK3399_PD_TIMEOUT_US);
	if (ret)
		log_err("pd %s: rail did not go %s, PWRDN_ST %08x\n", info->name,
			on ? "on" : "off", val);

	return ret;
}

static void rk3399_pd_save_qos(unsigned long id)
{
	struct rk3399_pd_state *st = &rk3399_pd_state[id];
	static const u16 off[QOS_NREGS] = { QOS_PRIORITY, QOS_MODE,
		QOS_BANDWIDTH, QOS_SATURATION, QOS_EXTCONTROL };
	int i, r;

	for (i = 0; i < st->nqos; i++)
		for (r = 0; r < QOS_NREGS; r++)
			st->qos_save[i][r] = readl((void __iomem *)st->qos[i] + off[r]);
}

static void rk3399_pd_restore_qos(unsigned long id)
{
	struct rk3399_pd_state *st = &rk3399_pd_state[id];
	static const u16 off[QOS_NREGS] = { QOS_PRIORITY, QOS_MODE,
		QOS_BANDWIDTH, QOS_SATURATION, QOS_EXTCONTROL };
	int i, r;

	for (i = 0; i < st->nqos; i++)
		for (r = 0; r < QOS_NREGS; r++)
			writel(st->qos_save[i][r], (void __iomem *)st->qos[i] + off[r]);
}

/*
 * Ungate every clock the domain node names. clk_enable_bulk() would do
 * this, but it wants a udevice and a domain is a subnode; the semantics
 * are the same -- -ENOSYS (provider has no enable) is forgiven, anything
 * else is the missing gate the handshake would hang on, and is fatal.
 */
static int rk3399_pd_enable_clocks(unsigned long id)
{
	struct rk3399_pd_state *st = &rk3399_pd_state[id];
	struct clk clk;
	int i, n, ret;

	n = ofnode_count_phandle_with_args(st->node, "clocks", "#clock-cells", 0);
	if (n == -ENOENT)
		return 0;
	if (n < 0)
		return n;

	for (i = 0; i < n; i++) {
		ret = clk_get_by_index_nodev(st->node, i, &clk);
		if (ret) {
			log_err("pd %s: clock %d: get failed (%d)\n",
				rk3399_pd_info[id].name, i, ret);
			return ret;
		}
		ret = clk_enable(&clk);
		if (ret && ret != -ENOSYS) {
			log_err("pd %s: clock %d (id %lu): enable failed (%d)\n",
				rk3399_pd_info[id].name, i, clk.id, ret);
			return ret;
		}
	}

	return 0;
}

static bool rk3399_pd_child_on(unsigned long id)
{
	unsigned long c;

	for (c = 0; c < RK3399_PD_NUM; c++)
		if (rk3399_pd_state[c].present &&
		    rk3399_pd_state[c].parent == (s8)id && rk3399_pd_is_on(c))
			return true;

	return false;
}

/**
 * rk3399_pd_transition() - power a domain on or off, hardware truth only
 * @id: domain id
 * @on: target state
 *
 * ON: parents first (DT nesting: PD_VIO -> PD_VO -> PD_VOPB, outside-in),
 * ungate clocks, clear PWRDN_CON, poll PWRDN_ST, release BUS_IDLE_REQ, poll
 * ACK then ST, restore QoS.
 *
 * OFF: refuse while any nested child is on, ungate clocks, save QoS, assert
 * BUS_IDLE_REQ, poll ACK then ST, set PWRDN_CON, poll PWRDN_ST. The idle
 * request stays asserted on a powered-off domain -- see the header comment
 * for why that is the contract BL31 relies on.
 *
 * Return: 0 on success or when already in the target state; -EBUSY when a
 * child holds a parent on; the failing step's error otherwise, with the
 * idle request unwound if the rail did not move.
 */
static int rk3399_pd_transition(unsigned long id, bool on)
{
	const struct rk3399_pd_info *info = &rk3399_pd_info[id];
	struct rk3399_pd_state *st = &rk3399_pd_state[id];
	int ret;

	if (rk3399_pd_is_on(id) == on)
		return 0;

	if (on && st->parent >= 0) {
		ret = rk3399_pd_transition(st->parent, true);
		if (ret)
			return ret;
	}

	if (!on && rk3399_pd_child_on(id)) {
		log_err("pd %s: a nested domain is still on, refusing power-off\n",
			info->name);
		return -EBUSY;
	}

	ret = rk3399_pd_enable_clocks(id);
	if (ret)
		return ret;

	rk3399_pd_log_regs(on ? "before on" : "before off", id);

	if (!on) {
		rk3399_pd_save_qos(id);
		ret = rk3399_pd_set_idle(id, true);
		if (ret)
			return ret;
	}

	ret = rk3399_pd_set_power(id, on);
	if (ret) {
		/* the rail did not move: do not leave a request behind it */
		if (!on)
			rk3399_pd_set_idle(id, false);
		return ret;
	}

	if (on) {
		ret = rk3399_pd_set_idle(id, false);
		if (ret)
			return ret;
		rk3399_pd_restore_qos(id);
	}

	rk3399_pd_log_regs(on ? "after on" : "after off", id);

	return 0;
}

static int rk3399_pd_request(struct power_domain *power_domain)
{
	if (!rk3399_pd_valid(power_domain->id)) {
		log_err("pd: no power-domain@%lu node\n", power_domain->id);
		return -EINVAL;
	}

	return 0;
}

static int rk3399_pd_on(struct power_domain *power_domain)
{
	struct rk3399_pd_state *st = &rk3399_pd_state[power_domain->id];
	int ret;

	ret = rk3399_pd_transition(power_domain->id, true);
	if (ret)
		return ret;

	st->refcount++;

	return 0;
}

/*
 * A consumer's off drops its reference; the domain is powered off only when
 * the last one goes. Two dwc3 controllers share PD_USB3 and the uclass has
 * no counting of its own that survives a provider remove.
 */
static int rk3399_pd_off(struct power_domain *power_domain)
{
	struct rk3399_pd_state *st = &rk3399_pd_state[power_domain->id];

	if (st->refcount <= 0)
		return 0;
	if (--st->refcount > 0)
		return 0;

	return rk3399_pd_transition(power_domain->id, false);
}

static void rk3399_pd_scan(ofnode parent, s8 parent_id)
{
	ofnode node;

	ofnode_for_each_subnode(node, parent) {
		struct rk3399_pd_state *st;
		u32 id;
		int i, n;

		if (ofnode_read_u32(node, "reg", &id) || id >= RK3399_PD_NUM ||
		    !rk3399_pd_info[id].name) {
			log_warning("pd: %s: bad or unknown reg, ignored\n",
				    ofnode_get_name(node));
			continue;
		}

		st = &rk3399_pd_state[id];
		st->present = true;
		st->parent = parent_id;
		st->node = node;
		st->nqos = 0;

		n = ofnode_count_phandle_with_args(node, "pm_qos", NULL, 0);
		for (i = 0; i < n && i < RK3399_PD_MAX_QOS; i++) {
			ofnode q = ofnode_parse_phandle(node, "pm_qos", i);
			fdt_addr_t addr = ofnode_valid(q) ? ofnode_get_addr(q) :
							    FDT_ADDR_T_NONE;

			if (addr == FDT_ADDR_T_NONE) {
				log_warning("pd %s: pm_qos %d unreadable, ignored\n",
					    rk3399_pd_info[id].name, i);
				continue;
			}
			st->qos[st->nqos++] = addr;
		}

		rk3399_pd_scan(node, id);
	}
}

static int rk3399_pd_probe(struct udevice *dev)
{
	fdt_addr_t base = dev_read_addr(dev_get_parent(dev));

	if (base == FDT_ADDR_T_NONE)
		return -EINVAL;

	rk3399_pmu_base = (void __iomem *)base;
	rk3399_pd_scan(dev_ofnode(dev), -1);

	return 0;
}

static const struct power_domain_ops rk3399_pd_ops = {
	.request = rk3399_pd_request,
	.on = rk3399_pd_on,
	.off = rk3399_pd_off,
};

static const struct udevice_id rk3399_pd_ids[] = {
	{ .compatible = "rockchip,rk3399-power-controller" },
	{ }
};

U_BOOT_DRIVER(rk3399_power_domain) = {
	.name = "rk3399_power_domain",
	.id = UCLASS_POWER_DOMAIN,
	.of_match = rk3399_pd_ids,
	.probe = rk3399_pd_probe,
	.ops = &rk3399_pd_ops,
};

#if IS_ENABLED(CONFIG_CMD_RK3399_PD)

static int rk3399_pd_cmd_get(void)
{
	struct udevice *dev;
	int ret;

	ret = uclass_get_device_by_driver(UCLASS_POWER_DOMAIN,
					  DM_DRIVER_GET(rk3399_power_domain),
					  &dev);
	if (ret)
		printf("rkpd: no power-controller device (%d)\n", ret);

	return ret;
}

static void rk3399_pd_cmd_regs(void)
{
	printf("PWRDN_CON %08x PWRDN_ST %08x BUS_IDLE_REQ %08x BUS_IDLE_ST %08x BUS_IDLE_ACK %08x\n",
	       rk3399_pmu_read(PMU_PWRDN_CON), rk3399_pmu_read(PMU_PWRDN_ST),
	       rk3399_pmu_read(PMU_BUS_IDLE_REQ), rk3399_pmu_read(PMU_BUS_IDLE_ST),
	       rk3399_pmu_read(PMU_BUS_IDLE_ACK));
}

static long rk3399_pd_cmd_lookup(const char *arg)
{
	unsigned long id;
	char *end;

	id = simple_strtoul(arg, &end, 10);
	if (*end == '\0')
		return rk3399_pd_valid(id) ? (long)id : -1;

	for (id = 0; id < RK3399_PD_NUM; id++)
		if (rk3399_pd_valid(id) && !strcmp(rk3399_pd_info[id].name, arg))
			return id;

	return -1;
}

static int do_rkpd(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	unsigned long id;
	long lid;
	int ret;

	if (rk3399_pd_cmd_get())
		return CMD_RET_FAILURE;

	if (argc == 1 || (argc == 2 && !strcmp(argv[1], "list"))) {
		rk3399_pd_cmd_regs();
		printf("%3s %-10s %-4s %-8s %-8s %s\n", "id", "name", "prnt",
		       "pwr", "idle", "state");
		for (id = 0; id < RK3399_PD_NUM; id++) {
			const struct rk3399_pd_info *info = &rk3399_pd_info[id];
			struct rk3399_pd_state *st = &rk3399_pd_state[id];

			if (!rk3399_pd_valid(id))
				continue;
			printf("%3lu %-10s %4d %08x %08x %s%s ref=%d\n", id,
			       info->name, st->parent, info->pwr_mask,
			       info->req_mask, rk3399_pd_is_on(id) ? "on" : "OFF",
			       info->req_mask && rk3399_pd_is_idle(id) ? "+idle" : "",
			       st->refcount);
		}
		return CMD_RET_SUCCESS;
	}

	if (argc == 2 && !strcmp(argv[1], "regs")) {
		rk3399_pd_cmd_regs();
		return CMD_RET_SUCCESS;
	}

	if (argc != 3)
		return CMD_RET_USAGE;

	lid = rk3399_pd_cmd_lookup(argv[2]);
	if (lid < 0) {
		printf("rkpd: unknown domain '%s'\n", argv[2]);
		return CMD_RET_FAILURE;
	}
	id = lid;

	if (!strcmp(argv[1], "on"))
		ret = rk3399_pd_transition(id, true);
	else if (!strcmp(argv[1], "off"))
		ret = rk3399_pd_transition(id, false);
	else
		return CMD_RET_USAGE;

	printf("rkpd: %s %s: ret=%d, now %s\n", argv[1], rk3399_pd_info[id].name,
	       ret, rk3399_pd_is_on(id) ? "on" : "OFF");
	rk3399_pd_cmd_regs();

	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(rkpd, 3, 0, do_rkpd,
	   "RK3399 power domains (bring-up)",
	   "[list]        - every domain: masks, parent, state, refcount\n"
	   "rkpd regs          - the five PMU registers, raw\n"
	   "rkpd on|off <id|name> - drive one domain directly (bypasses the consumer refcount)");

#endif /* CONFIG_CMD_RK3399_PD */
