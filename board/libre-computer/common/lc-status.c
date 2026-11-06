// SPDX-License-Identifier: GPL-2.0+
/*
 * The board's live status, read beside the console (CONFIG_MESON_LC_STATUS,
 * CONFIG_ROCKCHIP_LC_STATUS).
 *
 * A cyclic callback samples the board about once a second into a text
 * buffer, one "key=value" per line after a "lc-status <version>" line. The
 * DFU virt alternate "status" (listed by meson_set_dfu_alt_info() for the
 * interface "virt") returns that text on UPLOAD. With the usbacm console's
 * resident DFU the host reads it at the prompt over EP0, so nothing is typed
 * on the console and its endpoints carry nothing.
 *
 * Both halves run only while U-Boot polls: the console serves DFU from its
 * input poll and a cyclic callback runs from schedule(). So the text carries
 * its own sample number and time, and a reader judges its age; a clock or a
 * line the board cannot read is left out, never written as 0.
 *
 * The UPLOAD copies the text once, when it starts (the size request), so a
 * sample taken between two blocks never mixes into one reading.
 */

#include <clk.h>
#include <cyclic.h>
#include <dfu.h>
#include <dm.h>
#include <errno.h>
#include <event.h>
#include <log.h>
#include <mmc.h>
#include <time.h>
#include <vsprintf.h>
#include <asm/gpio.h>
#include <dm/ofnode.h>
#include <dm/uclass.h>
#include <bootrom.h>
#if IS_ENABLED(CONFIG_ROCKCHIP_RK3399)
#include <dt-bindings/clock/rk3399-cru.h>
#define LC_STATUS_CLKC		clk_rk3399
#elif IS_ENABLED(CONFIG_MESON_G12A)
#include <dt-bindings/clock/g12a-clkc.h>
#define LC_STATUS_CLKC		meson_clk_g12a
#else
#include <dt-bindings/clock/gxbb-clkc.h>
#define LC_STATUS_CLKC		meson_clk_gxbb
#endif
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/string.h>

#define LC_STATUS_VERSION	1
#define LC_STATUS_PERIOD_US	1000000
#define LC_STATUS_MAX		1024

static const struct {
	const char *name;
	unsigned long id;
} lc_status_clks[] = {
#if IS_ENABLED(CONFIG_ROCKCHIP_RK3399)
	/* the CRU's PLLs (drivers/clk/rockchip/clk_rk3399.c answers each) */
	{ "apll_l", PLL_APLLL },	/* the little cluster's clock */
	{ "apll_b", PLL_APLLB },	/* the big cluster's clock */
	{ "dpll", PLL_DPLL },		/* DRAM */
	{ "gpll", PLL_GPLL },		/* the general peripheral PLL */
	{ "cpll", PLL_CPLL },
	{ "npll", PLL_NPLL },
	{ "vpll", PLL_VPLL },		/* video */
#else
	{ "sys_pll", CLKID_SYS_PLL },	/* the CPU clock's PLL */
	{ "fixed_pll", CLKID_FIXED_PLL },
	{ "clk81", CLKID_CLK81 },	/* the peripheral bus clock */
#endif
};

static struct cyclic_info lc_status_cyclic;
static char lc_status_text[LC_STATUS_MAX];
static int lc_status_len;
static char lc_status_copy[LC_STATUS_MAX];
static int lc_status_copy_len;
static u32 lc_status_seq;

static struct udevice *lc_status_clkc;

/*
 * The SD slot's card detect: the first MMC node with cd-gpios. Its pin is
 * read here, or through the MMC driver when that driver holds the pin
 * itself (Rockchip's dw_mmc requests cd-gpios at probe, so a second request
 * is refused with -EBUSY).
 */
static struct gpio_desc lc_status_cd;
static struct mmc *lc_status_cd_mmc;
static bool lc_status_has_cd;
static int lc_status_cd_last = -1;
static u32 lc_status_cd_changes;

static int lc_status_put(int n, const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	n += vscnprintf(lc_status_text + n, sizeof(lc_status_text) - n, fmt,
			args);
	va_end(args);

	return n;
}

static void lc_status_sample(struct cyclic_info *c)
{
	ulong t0 = timer_get_us();
	int n = 0, i, v;

	n = lc_status_put(n, "lc-status %d\nseq=%u\nms=%lu\n",
			  LC_STATUS_VERSION, ++lc_status_seq, get_timer(0));

	for (i = 0; lc_status_clkc && i < ARRAY_SIZE(lc_status_clks); i++) {
		struct clk clk = {
			.dev = lc_status_clkc,
			.id = lc_status_clks[i].id,
		};
		ulong rate = clk_get_rate(&clk);

		if (rate && !IS_ERR_VALUE(rate))
			n = lc_status_put(n, "clk.%s=%lu\n",
					  lc_status_clks[i].name, rate);
	}

	if (lc_status_has_cd) {
		v = lc_status_cd_mmc ? mmc_getcd(lc_status_cd_mmc) :
				       dm_gpio_get_value(&lc_status_cd);
		if (v >= 0) {
			if (lc_status_cd_last >= 0 && v != lc_status_cd_last)
				lc_status_cd_changes++;
			lc_status_cd_last = v;
			n = lc_status_put(n, "sd.present=%d\nsd.changes=%u\n",
					  v, lc_status_cd_changes);
		}
	}

	/* The sample's own time, up to this line */
	n = lc_status_put(n, "cost_us=%lu\n", timer_get_us() - t0);
	lc_status_len = n;
}

static void lc_status_find_cd(void)
{
	struct udevice *dev;
	struct uclass *uc;

	uclass_id_foreach_dev(UCLASS_MMC, dev, uc) {
		int ret;

		if (!ofnode_read_prop(dev_ofnode(dev), "cd-gpios", NULL))
			continue;
		ret = gpio_request_by_name_nodev(dev_ofnode(dev), "cd-gpios", 0,
						 &lc_status_cd, GPIOD_IS_IN);
		if (ret == -EBUSY && device_active(dev)) {
			lc_status_cd_mmc = mmc_get_mmc_dev(dev);
			if (!lc_status_cd_mmc || mmc_getcd(lc_status_cd_mmc) < 0) {
				lc_status_cd_mmc = NULL;
				continue;
			}
		} else if (ret) {
			continue;
		}
		lc_status_has_cd = true;
		return;
	}
}

static int lc_status_init(void)
{
	if (uclass_get_device_by_driver(UCLASS_CLK,
					DM_DRIVER_GET(LC_STATUS_CLKC),
					&lc_status_clkc))
		lc_status_clkc = NULL;
	lc_status_find_cd();
	lc_status_sample(&lc_status_cyclic);
	cyclic_register(&lc_status_cyclic, lc_status_sample,
			LC_STATUS_PERIOD_US, "lc_status");

	return 0;
}
EVENT_SPY_SIMPLE(EVT_LAST_STAGE_INIT, lc_status_init);

/*
 * Keys read when the host asks, not in the once-a-second sample: what U-Boot
 * is waiting for and whether a `bootrom` request is pending change between
 * samples, and the app decides its next step on them (design R4, R5).
 *
 * wait=prompt|menu|countdown|pause|busy (busy: a command, a script or
 * `bootcmd`, where a USB request pends until the next wait or the OS
 * hand-off). bootrom=forced|unsupported is what `bootrom` would answer.
 */
static int lc_status_live(char *buf, int size)
{
	static const char *const waits[] = {
		[BOOTROM_WAIT_NONE] = "busy",
		[BOOTROM_WAIT_PROMPT] = "prompt",
		[BOOTROM_WAIT_MENU] = "menu",
		[BOOTROM_WAIT_COUNTDOWN] = "countdown",
		[BOOTROM_WAIT_PAUSE] = "pause",
	};
	enum bootrom_wait w;

	if (!IS_ENABLED(CONFIG_CMD_BOOTROM))
		return 0;
	w = bootrom_wait_get();

	return scnprintf(buf, size, "wait=%s\nbootrom=%s\nbootrom.pending=%d\n",
			 w < ARRAY_SIZE(waits) && waits[w] ? waits[w] : "?",
			 bootrom_check() ? "unsupported" : "forced",
			 bootrom_requested());
}

/* DFU virt back end: "status" (virt 0), read-only */
int dfu_get_medium_size_virt(struct dfu_entity *dfu, u64 *size)
{
	if (dfu->data.virt.dev_num != 0)
		return -ENODEV;
	memcpy(lc_status_copy, lc_status_text, lc_status_len);
	lc_status_copy_len = lc_status_len;
	lc_status_copy_len += lc_status_live(lc_status_copy + lc_status_copy_len,
					     sizeof(lc_status_copy) -
					     lc_status_copy_len);
	*size = lc_status_copy_len;

	return 0;
}

int dfu_read_medium_virt(struct dfu_entity *dfu, u64 offset, void *buf,
			 long *len)
{
	if (dfu->data.virt.dev_num != 0)
		return -ENODEV;
	if (offset >= lc_status_copy_len) {
		*len = 0;
		return 0;
	}
	if (*len > lc_status_copy_len - offset)
		*len = lc_status_copy_len - offset;
	memcpy(buf, lc_status_copy + offset, *len);

	return 0;
}

int dfu_write_medium_virt(struct dfu_entity *dfu, u64 offset, void *buf,
			  long *len)
{
	return -EROFS;
}
