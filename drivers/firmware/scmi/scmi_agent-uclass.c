// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 Linaro Limited.
 */

#define LOG_CATEGORY UCLASS_SCMI_AGENT

#include <dm.h>
#include <errno.h>
#include <scmi_agent.h>
#include <scmi_agent-uclass.h>
#include <scmi_protocols.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <stdio.h>

/**
 * struct error_code - Helper structure for SCMI error code conversion
 * @scmi:	SCMI error code
 * @errno:	Related standard error number
 */
struct error_code {
	int scmi;
	int errno;
};

static const struct error_code scmi_linux_errmap[] = {
	{ .scmi = SCMI_NOT_SUPPORTED, .errno = -EOPNOTSUPP, },
	{ .scmi = SCMI_INVALID_PARAMETERS, .errno = -EINVAL, },
	{ .scmi = SCMI_DENIED, .errno = -EACCES, },
	{ .scmi = SCMI_NOT_FOUND, .errno = -ENOENT, },
	{ .scmi = SCMI_OUT_OF_RANGE, .errno = -ERANGE, },
	{ .scmi = SCMI_BUSY, .errno = -EBUSY, },
	{ .scmi = SCMI_COMMS_ERROR, .errno = -ECOMM, },
	{ .scmi = SCMI_GENERIC_ERROR, .errno = -EIO, },
	{ .scmi = SCMI_HARDWARE_ERROR, .errno = -EREMOTEIO, },
	{ .scmi = SCMI_PROTOCOL_ERROR, .errno = -EPROTO, },
	{ .scmi = SCMI_IN_USE, .errno = -EADDRINUSE, },
};

/**
 * scmi_protocol_is_supported - check availability of protocol
 * @dev:	SCMI agent device
 * @proto_id:	Identifier of protocol
 *
 * check if the protocol, @proto_id, is provided by the SCMI agent,
 * @dev.
 *
 * Return:	0 on success, error code otherwise
 */
static bool scmi_protocol_is_supported(struct udevice *dev,
				       enum scmi_std_protocol proto_id)
{
	struct scmi_agent_priv *priv;
	int i;

	if (proto_id == SCMI_PROTOCOL_ID_BASE)
		return true;

	priv = dev_get_uclass_plat(dev);
	if (!priv) {
		dev_err(dev, "No priv data found\n");
		return false;
	}

	for (i = 0; i < priv->num_protocols; i++)
		if (priv->protocols[i] == proto_id)
			return true;

	return false;
}

struct udevice *scmi_get_protocol(struct udevice *dev,
				  enum scmi_std_protocol id)
{
	struct scmi_agent_priv *priv;
	struct udevice *proto;

	priv = dev_get_uclass_plat(dev);
	if (!priv) {
		dev_err(dev, "No priv data found\n");
		return NULL;
	}

	switch (id) {
	case SCMI_PROTOCOL_ID_BASE:
		proto = priv->base_dev;
		break;
#if IS_ENABLED(CONFIG_SCMI_POWER_DOMAIN)
	case SCMI_PROTOCOL_ID_POWER_DOMAIN:
		proto = priv->pwdom_dev;
		break;
#endif
#if IS_ENABLED(CONFIG_CLK_SCMI)
	case SCMI_PROTOCOL_ID_CLOCK:
		proto = priv->clock_dev;
		break;
#endif
#if IS_ENABLED(CONFIG_RESET_SCMI)
	case SCMI_PROTOCOL_ID_RESET_DOMAIN:
		proto = priv->resetdom_dev;
		break;
#endif
#if IS_ENABLED(CONFIG_DM_REGULATOR_SCMI)
	case SCMI_PROTOCOL_ID_VOLTAGE_DOMAIN:
		proto = priv->voltagedom_dev;
		break;
#endif
#if IS_ENABLED(CONFIG_PINCTRL_SCMI) || IS_ENABLED(CONFIG_PINCTRL_IMX_SCMI)
	case SCMI_PROTOCOL_ID_PINCTRL:
		proto = priv->pinctrl_dev;
		break;
#endif
#if IS_ENABLED(CONFIG_SCMI_ID_VENDOR_80)
	case SCMI_PROTOCOL_ID_VENDOR_80:
		proto = priv->vendor_dev_80;
		break;
#endif
#if IS_ENABLED(CONFIG_SCMI_ID_VENDOR_82)
	case SCMI_PROTOCOL_ID_VENDOR_82:
		proto = priv->vendor_dev_82;
		break;
#endif
	default:
		dev_err(dev, "Protocol not supported\n");
		proto = NULL;
		break;
	}
	if (proto && device_probe(proto))
		dev_err(dev, "Probe failed\n");

	return proto;
}

/**
 * scmi_add_protocol - add protocol to agent
 * @dev:	SCMI agent device
 * @proto_id:	SCMI protocol ID
 * @proto:	SCMI protocol device
 *
 * Associate the protocol instance, @proto, to the agent, @dev,
 * for later use.
 *
 * Return:	0 on success, error code on failure
 */
static int scmi_add_protocol(struct udevice *dev,
			     enum scmi_std_protocol proto_id,
			     struct udevice *proto)
{
	struct scmi_agent_priv *priv;

	priv = dev_get_uclass_plat(dev);
	if (!priv) {
		dev_err(dev, "No priv data found\n");
		return -ENODEV;
	}

	switch (proto_id) {
	case SCMI_PROTOCOL_ID_BASE:
		priv->base_dev = proto;
		break;
#if IS_ENABLED(CONFIG_SCMI_POWER_DOMAIN)
	case SCMI_PROTOCOL_ID_POWER_DOMAIN:
		priv->pwdom_dev = proto;
		break;
#endif
#if IS_ENABLED(CONFIG_CLK_SCMI)
	case SCMI_PROTOCOL_ID_CLOCK:
		priv->clock_dev = proto;
		break;
#endif
#if IS_ENABLED(CONFIG_RESET_SCMI)
	case SCMI_PROTOCOL_ID_RESET_DOMAIN:
		priv->resetdom_dev = proto;
		break;
#endif
#if IS_ENABLED(CONFIG_DM_REGULATOR_SCMI)
	case SCMI_PROTOCOL_ID_VOLTAGE_DOMAIN:
		priv->voltagedom_dev = proto;
		break;
#endif
#if IS_ENABLED(CONFIG_PINCTRL_SCMI) || IS_ENABLED(CONFIG_PINCTRL_IMX_SCMI)
	case SCMI_PROTOCOL_ID_PINCTRL:
		priv->pinctrl_dev = proto;
		break;
#endif
#if IS_ENABLED(CONFIG_SCMI_ID_VENDOR_80)
	case SCMI_PROTOCOL_ID_VENDOR_80:
		priv->vendor_dev_80 = proto;
		break;
#endif
#if IS_ENABLED(CONFIG_SCMI_ID_VENDOR_82)
	case SCMI_PROTOCOL_ID_VENDOR_82:
		priv->vendor_dev_82 = proto;
		break;
#endif
	default:
		dev_err(dev, "Protocol not supported\n");
		return -EPROTO;
	}

	return 0;
}

int scmi_to_linux_errno(s32 scmi_code)
{
	int n;

	if (!scmi_code)
		return 0;

	for (n = 0; n < ARRAY_SIZE(scmi_linux_errmap); n++)
		if (scmi_code == scmi_linux_errmap[n].scmi)
			return scmi_linux_errmap[n].errno;

	return -EPROTO;
}

static struct udevice *find_scmi_protocol_device(struct udevice *dev)
{
	struct udevice *parent = NULL, *protocol;

	for (protocol = dev; protocol; protocol = parent) {
		parent = dev_get_parent(protocol);
		if (!parent ||
		    device_get_uclass_id(parent) == UCLASS_SCMI_AGENT)
			break;
	}

	if (!parent) {
		dev_err(dev, "Invalid SCMI device, agent not found\n");
		return NULL;
	}

	return protocol;
}

static const struct scmi_agent_ops *transport_dev_ops(struct udevice *dev)
{
	return (const struct scmi_agent_ops *)dev->driver->ops;
}

/**
 * scmi_of_get_channel() - Get SCMI channel handle
 *
 * @dev:	SCMI agent device
 * @channel:	Output reference to the SCMI channel upon success
 *
 * On return, @channel will be set.
 * Return	0 on success and a negative errno on failure
 */
static int scmi_of_get_channel(struct udevice *dev, struct udevice *protocol,
			       struct scmi_channel **channel)
{
	const struct scmi_agent_ops *ops;

	ops = transport_dev_ops(dev);
	if (ops->of_get_channel)
		return ops->of_get_channel(dev, protocol, channel);
	else
		return -EPROTONOSUPPORT;
}

int devm_scmi_of_get_channel(struct udevice *dev)
{
	struct udevice *protocol;
	struct scmi_agent_proto_priv *priv;
	int ret;

	protocol = find_scmi_protocol_device(dev);
	if (!protocol)
		return -ENODEV;

	priv = dev_get_parent_priv(protocol);
	ret = scmi_of_get_channel(protocol->parent, protocol, &priv->channel);
	if (ret == -EPROTONOSUPPORT) {
		/* Drivers without a get_channel operator don't need a channel ref */
		priv->channel = NULL;

		return 0;
	}

	return ret;
}

/**
 * scmi_process_msg() - Send and process an SCMI message
 *
 * Send a message to an SCMI server.
 * Caller sets scmi_msg::out_msg_sz to the output message buffer size.
 *
 * @dev:	SCMI agent device
 * @channel:	Communication channel for the device
 * @msg:	Message structure reference
 *
 * On return, scmi_msg::out_msg_sz stores the response payload size.
 * Return:	0 on success and a negative errno on failure
 */
static int scmi_process_msg(struct udevice *dev, struct scmi_channel *channel,
			    struct scmi_msg *msg)
{
	const struct scmi_agent_ops *ops;

	ops = transport_dev_ops(dev);
	if (ops->process_msg)
		return ops->process_msg(dev, channel, msg);
	else
		return -EPROTONOSUPPORT;
}

int devm_scmi_process_msg(struct udevice *dev, struct scmi_msg *msg)
{
	struct udevice *protocol;
	struct scmi_agent_proto_priv *priv;

	protocol = find_scmi_protocol_device(dev);
	if (!protocol)
		return -ENODEV;

	priv = dev_get_parent_priv(protocol);

	return scmi_process_msg(protocol->parent, priv->channel, msg);
}

/**
 * scmi_fill_base_info - get base information about SCMI server
 * @agent:	SCMI agent device
 * @dev:	SCMI protocol device
 *
 * By using Base protocol commands, collect the base information
 * about SCMI server.
 *
 * Return: 0 on success, error code on failure
 */
static int scmi_fill_base_info(struct udevice *agent, struct udevice *dev)
{
	struct scmi_agent_priv *priv = dev_get_uclass_plat(agent);
	int ret;

	ret = scmi_base_protocol_version(dev, &priv->version);
	if (ret) {
		dev_err(dev, "protocol_version() failed (%d)\n", ret);
		return ret;
	}
	/* check for required version */
	if (priv->version < SCMI_BASE_PROTOCOL_VERSION) {
		dev_err(dev, "base protocol version (%d) lower than expected\n",
			priv->version);
		return -EPROTO;
	}

	ret = scmi_base_protocol_attrs(dev, &priv->num_agents,
				       &priv->num_protocols);
	if (ret) {
		dev_err(dev, "protocol_attrs() failed (%d)\n", ret);
		return ret;
	}
	ret = scmi_base_discover_vendor(dev, &priv->vendor);
	if (ret) {
		dev_err(dev, "base_discover_vendor() failed (%d)\n", ret);
		return ret;
	}
	ret = scmi_base_discover_sub_vendor(dev, &priv->sub_vendor);
	if (ret) {
		if (ret != -EOPNOTSUPP) {
			dev_err(dev, "base_discover_sub_vendor() failed (%d)\n",
				ret);
			return ret;
		}
		priv->sub_vendor = "NA";
	}
	ret = scmi_base_discover_impl_version(dev, &priv->impl_version);
	if (ret) {
		dev_err(dev, "base_discover_impl_version() failed (%d)\n",
			ret);
		return ret;
	}

	ret = scmi_base_discover_agent(dev, 0xffffffff,
				       &priv->agent_id, &priv->agent_name);
	if (ret) {
		if (ret != -EOPNOTSUPP) {
			dev_err(dev,
				"base_discover_agent() failed for myself (%d)\n",
				ret);
			return ret;
		}
		priv->agent_id = 0xffffffff;
		priv->agent_name = "NA";
	}

	ret = scmi_base_discover_list_protocols(dev, &priv->protocols);
	if (ret != priv->num_protocols) {
		dev_err(dev, "base_discover_list_protocols() failed (%d)\n",
			ret);
		return -EPROTO;
	}

	return 0;
}

static struct driver *scmi_proto_driver_get(unsigned int proto_id)
{
	struct scmi_proto_driver *start, *entry;
	int n_ents;

	start = ll_entry_start(struct scmi_proto_driver, scmi_proto_driver);
	n_ents = ll_entry_count(struct scmi_proto_driver, scmi_proto_driver);

	for (entry = start; entry != start + n_ents; entry++) {
		if (entry->match->proto_id == proto_id)
			return entry->driver;
	}

	return NULL;
}

/*
 * Residual bring-up (SM1 / gap §8.12–§8.13): always-on printf so a mute
 * soft-fail path or a silent BASE *success* still leaves a UART fingerprint.
 * SC-HB @ 0xFFFE7400 is painted by open SC (scp-firmware 271ea712).
 *
 * hb[4] / hb[5] are the SC's two cumulative MHU fault counters, published by
 * scp-firmware's product/amlogic/g12a/include/g12a_core.h as
 * G12A_SC_HB_OFF_MHU_SIGFAIL (signal_message() failures, ACKed anyway) and
 * G12A_SC_HB_OFF_MHU_REQFAIL ("RX is dead if != 0").  Unlike hb[0..3] they are
 * NOT repainted by the poll tick: they are seeded once by
 * g12a_sc_heartbeat_init() and only ever incremented, so the != 0 contract is
 * only meaningful on a build whose startup hook reaches that seed.
 *
 * They are printed here because this target cannot be asked for them any other
 * way: aml-s905d3-cc_libre_residual_opensc_defconfig sets CONFIG_BOOTDELAY=-1
 * and CONFIG_BOOTCOMMAND="", so the "=> " it prints is never read and there is
 * no Linux to read 0xFFFE7410 from either.
 *
 * SC-HB-END6 terminates the banner so a SHORT banner is visibly short: an
 * absent hb[4]/hb[5] is then an absent line and not a zero, which is the whole
 * failure mode this print exists to remove.
 *
 * SC-STKP @ 0xFFFE7420 is the SC's stack high-water block, published by
 * scp-firmware 0414021e (product/amlogic/g12a/include/g12a_mmap.h,
 * G12A_STACK_PROBE_*) only in an SCP_G12A_STACK_PAINT=ON build.  Word [0] is
 * the 'STKP' magic 0x53544B50 and is always printed; words [1..6] are printed
 * ONLY when it matches, because on an OFF build the block is whatever SRAM
 * held and printing it would be garbage formatted as measurements (the
 * open-sc-size.md E4 reader defect).  [6] is the SC's publish sequence, written
 * last: it is read before AND after [3..5] and both reads are printed, so a
 * torn read shows as two different values, and 0 means [3..5] are not
 * readings yet.  An OFF build does not clear the block, so a magic can survive
 * from a prior ON boot across a warm reset -- read SC-STKP beside this boot's
 * SC-HB, not alone.  readl(), not the plain loads above: two plain loads of the
 * seq with no store between them may be folded into one.
 *
 * SC-CPWR @ 0xFFFE7440 / 0xFFFE7460 / 0xFFFE7480 are the SC's per-core
 * power-on verdicts for cpu1 / cpu2 / cpu3, published by scp-firmware
 * 2241b91a (product/amlogic/g12a/include/g12a_mmap.h, G12A_CPU_PWR_DIAG_*) only
 * in an SCP_G12A_CPU_POWER_DIAG=ON build.  A core POWER_STATE_SET is async by
 * the SCMI spec, so BL31's CPU_ON rc cannot carry the SC's verdict; this block
 * is where it goes.  Read HERE because this banner runs in U-Boot proper, i.e.
 * after BL31's N3 crumb has issued CPU_ON cpu2 from bl31_plat_runtime_setup().
 * Word [0] is always printed: 'CPWR' 0x43505752 = finished, 'CPWB' 0x43505742
 * = the SC is still inside its ACK poll, anything else = no attempt this boot
 * (or an OFF build, which does not clear the block).  [1..7] are printed only
 * under 'CPWR': core, last step (1..4), CNTL0 0xFF8000E0 / CNTL1 0xFF8000E4 /
 * MEM_PD0 0xFF8000F4 read after the poll, poll time (nominal us), rc.  CNTL1
 * bit n is the A55 power ACK and bit n+16 the deny (smpstate D2; the bit
 * positions are inference).
 *
 * SC-CPWS @ 0xFFFE74A0 / 0xFFFE74C0 / 0xFFFE74E0 is the SM1 extension written
 * by scp-firmware c2881c51 (SCP_G12A_CPU_POWER_SM1, the vendor-faithful
 * sequence; claude:amlogic/software/boot/g12/audit/
 * 2026-09-28-g12a-bl30-cpu-power-on-re.md) before that core's 'CPWR' magic:
 * MEM_PD1 0xFF8000F8, the core's SYS_CPU_STATUS3+4n word, the accept /
 * settle / on poll iterations, and flags (bit0 = no verdict in time).  It is
 * printed only under 'CPWR' AND a step value the DIAG-only build never
 * produces (0 or >= 12; DIAG always ends at 4), because a DIAG-only build
 * does not write it.  Under this option the main block's step is the audit's
 * row number, its word [6] (printed as poll_us= on the SC-CPWR line, and again
 * as gate= here) is the raw word the SC's chip gate read: ASSIST_HW_REV
 * 0xFFD0814C, SM1 when [7:0] == 0x2b (scp-firmware 22719306; before that,
 * ce80c19e read AO_SEC_SD_CFG8 0xFF800220, which is 0x2 at SC time on the
 * -libre chain), and its rc is: 0 on, -11 denied, -12 no verdict and never on,
 * -15 verdict never cleared, -7 accepted but never on, -10 not SM1.
 */
#define SCMI_SC_STKP_BASE	0xFFFE7420UL
#define SCMI_SC_STKP_MAGIC	0x53544B50U /* 'STKP' */
#define SCMI_SC_CPWR_BASE	0xFFFE7440UL
#define SCMI_SC_CPWR_STRIDE	0x20UL
#define SCMI_SC_CPWR_MAGIC	0x43505752U /* 'CPWR' */
#define SCMI_SC_CPWS_BASE	0xFFFE74A0UL
#define SCMI_SC_CPWS_STRIDE	0x20UL
#define SCMI_SC_CPWS_WORDS	6
static void scmi_residual_probe_banner(struct udevice *dev, const char *what,
				       int code)
{
	u32 *hb = (u32 *)0xFFFE7400;
	u32 txstat, irqen, h0, h1, h2, h3, h4, h5;
	u32 s0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, seq_a = 0, seq_b = 0;
	u32 cp[3][8] = { { 0 } };
	u32 cs[3][SCMI_SC_CPWS_WORDS] = { { 0 } };
	bool has_cs[3] = { false, false, false };
	unsigned int c, w;

	if (!IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG))
		return;

	/* Read first, then one-field-per-line — chip_id races a long printf. */
	txstat = readl(0xFF63C444);
	irqen = readl(0xFF634670);
	h0 = hb[0];
	h1 = hb[1];
	h2 = hb[2];
	h3 = hb[3];
	h4 = hb[4];
	h5 = hb[5];
	s0 = readl(SCMI_SC_STKP_BASE + 0x00);
	if (s0 == SCMI_SC_STKP_MAGIC) {
		s1 = readl(SCMI_SC_STKP_BASE + 0x04);
		s2 = readl(SCMI_SC_STKP_BASE + 0x08);
		seq_a = readl(SCMI_SC_STKP_BASE + 0x18);
		s3 = readl(SCMI_SC_STKP_BASE + 0x0C);
		s4 = readl(SCMI_SC_STKP_BASE + 0x10);
		s5 = readl(SCMI_SC_STKP_BASE + 0x14);
		seq_b = readl(SCMI_SC_STKP_BASE + 0x18);
	}
	for (c = 0; c < 3; c++) {
		ulong blk = SCMI_SC_CPWR_BASE + c * SCMI_SC_CPWR_STRIDE;

		cp[c][0] = readl(blk);
		if (cp[c][0] == SCMI_SC_CPWR_MAGIC)
			for (w = 1; w < 8; w++)
				cp[c][w] = readl(blk + w * 4);
		has_cs[c] = cp[c][0] == SCMI_SC_CPWR_MAGIC &&
			    (cp[c][2] == 0 || cp[c][2] >= 12);
		if (has_cs[c]) {
			ulong ext = SCMI_SC_CPWS_BASE + c * SCMI_SC_CPWS_STRIDE;

			for (w = 0; w < SCMI_SC_CPWS_WORDS; w++)
				cs[c][w] = readl(ext + w * 4);
		}
	}
	printf("SCMI: %s (%d)\n", what, code);
	printf("SCMI: TXSTAT=%08x IRQEN=%08x\n", txstat, irqen);
	printf("SCMI: SC-HB0=%08x\n", h0);
	printf("SCMI: SC-HB1=%08x\n", h1);
	printf("SCMI: SC-HB2=%08x\n", h2);
	printf("SCMI: SC-HB3=%08x\n", h3);
	printf("SCMI: SC-HB4=%08x SIGFAIL\n", h4);
	printf("SCMI: SC-HB5=%08x REQFAIL\n", h5);
	printf("SCMI: SC-STKP0=%08x\n", s0);
	if (s0 == SCMI_SC_STKP_MAGIC) {
		printf("SCMI: SC-STKP1=%08x SIZE\n", s1);
		printf("SCMI: SC-STKP2=%08x PAINTED\n", s2);
		printf("SCMI: SC-STKP6=%08x SEQ-BEFORE\n", seq_a);
		printf("SCMI: SC-STKP3=%08x HEADROOM\n", s3);
		printf("SCMI: SC-STKP4=%08x HEAP-USED\n", s4);
		printf("SCMI: SC-STKP5=%08x STACK-HW\n", s5);
		printf("SCMI: SC-STKP6=%08x SEQ-AFTER\n", seq_b);
	}
	for (c = 0; c < 3; c++) {
		printf("SCMI: SC-CPWR cpu%u MAGIC=%08x\n", c + 1, cp[c][0]);
		if (cp[c][0] == SCMI_SC_CPWR_MAGIC)
			printf("SCMI: SC-CPWR cpu%u core=%u step=%u CNTL0=%08x CNTL1=%08x MEM_PD0=%08x poll_us=%u rc=%d\n",
			       c + 1, cp[c][1], cp[c][2], cp[c][3], cp[c][4],
			       cp[c][5], cp[c][6], (int)cp[c][7]);
		if (has_cs[c])
			printf("SCMI: SC-CPWS cpu%u gate=%08x MEM_PD1=%08x STATUS=%08x accept_it=%u settle_it=%u on_it=%u flags=%x\n",
			       c + 1, cp[c][6], cs[c][0], cs[c][1], cs[c][2],
			       cs[c][3], cs[c][4], cs[c][5]);
	}
	printf("SCMI: SC-HB-END6\n");
	(void)dev;
}

/*
 * Residual banner OPP discovery. The perf / clock round-trips below flip
 * the CPU between the lowest and highest level the SC ITSELF advertises,
 * never a rate typed here: the open SC's table changed once already
 * (scp-firmware 6698be5f -- 996 MHz / 1.2 GHz, because 1000 MHz puts the
 * sys_pll DCO below its floor), and a hard-coded 1.0 GHz made the perf flip
 * skip and the CLOCK SET back get INVALID_PARAMETERS, stranding the CPU at
 * 1.2 GHz. Pages are followed until num_remaining == 0 (bounded); a page
 * larger than SCMI_RESID_DESC_MAX fails with -ETOOSMALL in the transport,
 * which the caller reports rather than guessing a level.
 */
#define SCMI_RESID_DESC_MAX	8
#define SCMI_RESID_DESC_PAGES	8

static int scmi_residual_perf_opps(struct udevice *proto, u32 domain,
				   u32 *lo, u32 *hi)
{
	u32 idx = 0, lo_v = U32_MAX, hi_v = 0;
	int page;

	for (page = 0; page < SCMI_RESID_DESC_PAGES; page++) {
		struct {
			u32 domain_id;
			u32 level_index;
		} in = { .domain_id = domain, .level_index = idx };
		struct {
			s32 status;
			u32 num_levels;	/* [11:0] returned, [31:16] remaining */
			struct {
				u32 performance_level;
				u32 power_cost;
				u32 attributes;
			} lv[SCMI_RESID_DESC_MAX];
		} out = { 0 };
		struct scmi_msg msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF,
						  SCMI_PERF_DESCRIBE_LEVELS,
						  in, out);
		int er = devm_scmi_process_msg(proto, &msg);
		u32 nret, nrem, i;

		if (er >= 0)
			er = scmi_to_linux_errno(out.status);
		if (er)
			return er;
		nret = out.num_levels & GENMASK(11, 0);
		nrem = out.num_levels >> 16;
		if (nret > SCMI_RESID_DESC_MAX)
			return -EPROTO;
		for (i = 0; i < nret; i++) {
			u32 v = out.lv[i].performance_level;

			if (v < lo_v)
				lo_v = v;
			if (v > hi_v)
				hi_v = v;
		}
		idx += nret;
		if (!nret || !nrem)
			break;
	}
	if (!hi_v)
		return -ENODATA;
	*lo = lo_v;
	*hi = hi_v;
	return 0;
}

static int scmi_residual_clk_opps(struct udevice *proto, u32 clock_id,
				  u64 *lo, u64 *hi)
{
	u32 idx = 0;
	u64 lo_v = U64_MAX, hi_v = 0;
	int page;

	for (page = 0; page < SCMI_RESID_DESC_PAGES; page++) {
		struct scmi_clk_describe_rates_in in = {
			.clock_id = clock_id,
			.rate_index = idx,
		};
		struct {
			s32 status;
			u32 num_rates_flags;
			struct {
				u32 rate_lsb;
				u32 rate_msb;
			} rate[SCMI_RESID_DESC_MAX];
		} out = { 0 };
		struct scmi_msg msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
						  SCMI_CLOCK_DESCRIBE_RATES,
						  in, out);
		int er = devm_scmi_process_msg(proto, &msg);
		u32 nret, nrem, i;

		if (er >= 0)
			er = scmi_to_linux_errno(out.status);
		if (er)
			return er;
		nret = out.num_rates_flags & SCMI_CLK_DESC_RATES_NRET_MASK;
		nrem = out.num_rates_flags >> SCMI_CLK_DESC_RATES_NREM_SHIFT;
		if (nret > SCMI_RESID_DESC_MAX)
			return -EPROTO;
		/* Range form: rate[0] = min, rate[1] = max, rate[2] = step. */
		if (out.num_rates_flags & SCMI_CLK_DESC_RATES_RANGE) {
			if (nret < 2)
				return -EPROTO;
			nret = 2;
			nrem = 0;
		}
		for (i = 0; i < nret; i++) {
			u64 v = ((u64)out.rate[i].rate_msb << 32) |
				out.rate[i].rate_lsb;

			if (v < lo_v)
				lo_v = v;
			if (v > hi_v)
				hi_v = v;
		}
		idx += nret;
		if (!nret || !nrem)
			break;
	}
	if (!hi_v)
		return -ENODATA;
	*lo = lo_v;
	*hi = hi_v;
	return 0;
}

/*
 * SCMI agent devices binds devices of various uclasses depending on
 * the FDT description. scmi_bind_protocol() is a generic bind sequence
 * called by the uclass at bind stage, that is uclass post_bind.
 */
static int scmi_bind_protocols(struct udevice *dev)
{
	int ret = 0;
	ofnode node;
	const char *name;
	struct driver *drv;
	struct udevice *agent, *proto;

	if (IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG)) {
		printf("SCMI: agent post_bind %s\n", dev->name);
		/* Pre-probe SC-HB (open SC paints in hardware_init_hook). */
		scmi_residual_probe_banner(dev, "pre-BASE SC-HB", 0);
	}

	if (!uclass_get_device(UCLASS_SCMI_AGENT, 1, &agent)) {
		/* This is a second SCMI agent */
		dev_err(dev, "Cannot have more than one SCMI agent\n");
		return -EEXIST;
	}

	/* initialize the device from device tree */
	drv = DM_DRIVER_GET(scmi_base_drv);
	name = "scmi-base.0";
	ret = device_bind(dev, drv, name, NULL, ofnode_null(), &proto);
	if (ret) {
		dev_err(dev, "failed to bind base protocol\n");
		return ret;
	}
	ret = scmi_add_protocol(dev, SCMI_PROTOCOL_ID_BASE, proto);
	if (ret) {
		dev_err(dev, "failed to add protocol: %s, ret: %d\n",
			proto->name, ret);
		return ret;
	}

	ret = device_probe(proto);
	if (ret) {
		/*
		 * A refused / failed BASE probe must not fail post_bind:
		 * lists_bind_fdt returns that error, dm_scan_fdt_node
		 * surfaces it, and initr_dm hangs ("Please RESET").
		 * Measured on ROC-RK3399-PC with RK3399_SCMI_M0_FAULT=1
		 * (GENERIC_ERROR after the 10 ms BL31 bound). Residual
		 * Amlogic banners stay behind SCMI_RESIDUAL_DIAG.
		 */
		dev_err(dev,
			"base protocol probe failed (%d); continuing without SCMI\n",
			ret);
		if (IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG))
			scmi_residual_probe_banner(dev, "BASE probe FAIL", ret);
		return 0;
	}

	scmi_residual_probe_banner(dev, "BASE probe OK", 0);

	ret = scmi_fill_base_info(dev, proto);
	if (ret) {
		dev_err(dev,
			"base info failed (%d); continuing without SCMI\n",
			ret);
		if (IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG))
			scmi_residual_probe_banner(dev, "BASE info FAIL", ret);
		return 0;
	}

	if (IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG)) {
		struct scmi_agent_priv *apriv = dev_get_uclass_plat(dev);
		u32 i, n = apriv->num_protocols;

		/* One field per line — chip_id races long printfs (gap §8.13). */
		printf("SCMI: protocols n=%u\n", n);
		for (i = 0; i < n; i++)
			printf("SCMI: proto-id %02x\n", apriv->protocols[i]);

		/*
		 * N4: SENSOR_READING_GET on sensor 0 (TS_DDR). U-Boot has
		 * no scmi sensor cmd / protocol driver — residual print
		 * only, via the already-probed BASE channel. One field
		 * per line (sns0-rc / sns0-val).
		 */
		if (scmi_protocol_is_supported(dev, SCMI_PROTOCOL_ID_SENSOR)) {
			struct scmi_sensor_reading_get_in sin = {
				.sensor_id = 0,
				.flags = 0,
			};
			struct scmi_sensor_reading_get_out sout = { 0 };
			struct scmi_msg smsg =
				SCMI_MSG_IN(SCMI_PROTOCOL_ID_SENSOR,
					    SCMI_SENSOR_READING_GET,
					    sin, sout);
			int ser = devm_scmi_process_msg(proto, &smsg);
			s64 sval = 0;

			if (ser >= 0)
				ser = scmi_to_linux_errno(sout.status);
			if (ser == 0)
				sval = ((s64)sout.value_high << 32) |
				       sout.value_low;
			printf("SCMI: sns0-rc %d\n", ser);
			printf("SCMI: sns0-val %lld\n", (long long)sval);
		} else {
			printf("SCMI: sns0-rc %d\n", -ENOENT);
			printf("SCMI: sns0-val 0\n");
		}

		/*
		 * N5: PERF LEVEL GET/SET on domain 0. The OPPs are the
		 * lowest / highest level the SC reports in
		 * PERF_DESCRIBE_LEVELS (open SC 6698be5f: 996 MHz /
		 * 1.2 GHz), never typed here. No scmi perf driver —
		 * residual print via BASE channel. DESCRIBE → GET →
		 * same-rate SET → other OPP SET → CLOCK_RATE_GET →
		 * SET back to the GET level → GET. Ends at the level
		 * it found. One field per line (perf0-desc-rc /
		 * perf0-opp-lo / perf0-opp-hi / perf0-get /
		 * perf0-set-rc / clk0-hz-after / perf0-restore-rc /
		 * perf0-get-end). Not board DVFS / PWM.
		 */
		if (scmi_protocol_is_supported(dev, SCMI_PROTOCOL_ID_PERF)) {
			struct {
				u32 domain_id;
			} gin = { .domain_id = 0 };
			struct {
				s32 status;
				u32 performance_level;
			} gout = { 0 };
			struct scmi_msg gmsg =
				SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF,
					    SCMI_PERF_LEVEL_GET,
					    gin, gout);
			int ger;
			u32 level = 0;
			u32 level_end = 0;
			int set_er = -ENOENT;
			int rst_er = -ENOENT;
			bool flipped = false;
			unsigned long long hz_after = 0;
			u32 opp_lo = 0, opp_hi = 0;
			int desc_er = scmi_residual_perf_opps(proto, 0,
							      &opp_lo,
							      &opp_hi);

			printf("SCMI: perf0-desc-rc %d\n", desc_er);
			printf("SCMI: perf0-opp-lo %u\n", opp_lo);
			printf("SCMI: perf0-opp-hi %u\n", opp_hi);

			ger = devm_scmi_process_msg(proto, &gmsg);
			if (ger >= 0)
				ger = scmi_to_linux_errno(gout.status);
			if (ger == 0)
				level = gout.performance_level;
			printf("SCMI: perf0-get %u\n", level);

			if (ger == 0) {
				struct {
					u32 domain_id;
					u32 performance_level;
				} sin = {
					.domain_id = 0,
					.performance_level = level,
				};
				struct {
					s32 status;
				} sout = { 0 };
				struct scmi_msg smsg =
					SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF,
						    SCMI_PERF_LEVEL_SET,
						    sin, sout);
				u32 other = (level == opp_hi) ? opp_lo :
					    opp_hi;

				set_er = devm_scmi_process_msg(proto, &smsg);
				if (set_er >= 0)
					set_er = scmi_to_linux_errno(
						sout.status);
				/*
				 * Same-rate first. Only flip OPP when
				 * DESCRIBE gave two distinct levels and
				 * GET was one of them (else SET other
				 * would be INVALID_PARAMETERS on a
				 * non-OPP GET).
				 */
				if (set_er == 0 && desc_er == 0 &&
				    opp_lo != opp_hi &&
				    (level == opp_lo || level == opp_hi)) {
					sin.performance_level = other;
					sout.status = 0;
					smsg = SCMI_MSG_IN(
						SCMI_PROTOCOL_ID_PERF,
						SCMI_PERF_LEVEL_SET,
						sin, sout);
					set_er = devm_scmi_process_msg(
						proto, &smsg);
					if (set_er >= 0)
						set_er = scmi_to_linux_errno(
							sout.status);
					/*
					 * Restore even on a failed SET: the
					 * SC may have moved before refusing.
					 */
					flipped = true;
				}
				if (set_er == 0)
					mdelay(50);
			}
			printf("SCMI: perf0-set-rc %d\n", set_er);

			{
				struct scmi_clk_rate_get_in cin = {
					.clock_id = 0,
				};
				struct scmi_clk_rate_get_out cout = { 0 };
				struct scmi_msg cmsg =
					SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
						    SCMI_CLOCK_RATE_GET,
						    cin, cout);

				if (devm_scmi_process_msg(proto, &cmsg) >= 0 &&
				    scmi_to_linux_errno(cout.status) == 0)
					hz_after =
					    ((unsigned long long)cout.rate_msb
					     << 32) | cout.rate_lsb;
			}
			printf("SCMI: clk0-hz-after %llu\n", hz_after);

			/* Leave the CPU at the level the GET found. */
			if (flipped) {
				struct {
					u32 domain_id;
					u32 performance_level;
				} rin = {
					.domain_id = 0,
					.performance_level = level,
				};
				struct {
					s32 status;
				} rout = { 0 };
				struct scmi_msg rmsg =
					SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF,
						    SCMI_PERF_LEVEL_SET,
						    rin, rout);

				rst_er = devm_scmi_process_msg(proto, &rmsg);
				if (rst_er >= 0)
					rst_er = scmi_to_linux_errno(
						rout.status);
				if (rst_er == 0)
					mdelay(50);
			}
			printf("SCMI: perf0-restore-rc %d\n", rst_er);

			gout.status = 0;
			gout.performance_level = 0;
			gmsg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF,
					   SCMI_PERF_LEVEL_GET, gin, gout);
			ger = devm_scmi_process_msg(proto, &gmsg);
			if (ger >= 0)
				ger = scmi_to_linux_errno(gout.status);
			if (ger == 0)
				level_end = gout.performance_level;
			printf("SCMI: perf0-get-end %u\n", level_end);
		} else {
			printf("SCMI: perf0-get 0\n");
			printf("SCMI: perf0-set-rc %d\n", -ENOENT);
			printf("SCMI: clk0-hz-after 0\n");
		}
	}

	dev_for_each_subnode(node, dev) {
		u32 protocol_id;

		if (!ofnode_is_enabled(node))
			continue;

		if (ofnode_read_u32(node, "reg", &protocol_id))
			continue;

		drv = NULL;
		name = ofnode_get_name(node);

		if (!scmi_protocol_is_supported(dev, protocol_id))
			continue;

		drv = scmi_proto_driver_get(protocol_id);
		if (!drv) {
			dev_dbg(dev, "Ignore unsupported SCMI protocol %#x\n",
				protocol_id);
			continue;
		}

		ret = device_bind(dev, drv, name, NULL, node, &proto);
		if (ret) {
			dev_err(dev, "failed to bind %s protocol\n", drv->name);
			break;
		}
		ret = scmi_add_protocol(dev, protocol_id, proto);
		if (ret) {
			dev_err(dev, "failed to add protocol: %s, ret: %d\n",
				proto->name, ret);
			break;
		}

		/*
		 * Residual only: force-probe so sticky DFU still leaves a
		 * UART OK/FAIL for Clock / Power Domain (no `=>` needed).
		 * proto-rc FIRST (id+rc one line) — UART drops mid-burst
		 * before misc_init_r chip_id (gap §8.13 clkpd2).
		 *
		 * CLK is the last advertised protocol: chip_id truncates the
		 * 3rd post-probe line (measured p02-clkhz). Do CLOCK_RATE_GET
		 * (+ P0-3 CLOCK_RATE_SET found→other OPP→found + GET after
		 * each SET) before prints, put the round-trip on the
		 * surviving proto-rc line, then emit clk0-hz /
		 * clk0-set-rc / clk0-hz-after / clk0-set2-rc /
		 * clk0-hz-after2 / clk0-opp-lo / clk0-opp-hi on own
		 * lines.
		 *
		 * Real rate change between the lowest and highest rate
		 * the SC reports in CLOCK_DESCRIBE_RATES (open SC
		 * 6698be5f config_g12a_clock.c cpu_rates[]: 996 +
		 * 1200 MHz; 1000 MHz is below the sys_pll DCO floor).
		 * The second SET is the rate the first GET found, so
		 * the CPU ends where it started; the SC matches rates
		 * exactly, so a found rate outside the table skips the
		 * round-trip (set-rc -ENOENT). Same-rate no-op already
		 * measured (p03-clkset2). Do not power-off CPU0.
		 */
		if (IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG)) {
			int pr = device_probe(proto);

			if (!pr && IS_ENABLED(CONFIG_CLK_SCMI) &&
			    protocol_id == SCMI_PROTOCOL_ID_CLOCK) {
				struct scmi_clk_rate_get_in in = {
					.clock_id = 0,
				};
				struct scmi_clk_rate_get_out out = { 0 };
				struct scmi_msg msg =
					SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
						    SCMI_CLOCK_RATE_GET, in, out);
				u64 opp_lo = 0, opp_hi = 0;
				int desc_er = scmi_residual_clk_opps(proto, 0,
								     &opp_lo,
								     &opp_hi);
				int er = devm_scmi_process_msg(proto, &msg);
				unsigned long long hz = 0;
				int set_er = -ENOENT;
				int set2_er = -ENOENT;
				unsigned long long hz_after = 0;
				unsigned long long hz_after2 = 0;
				/* SC-reported OPPs (not typed here). */
				unsigned long long set_hz = 0;
				unsigned long long back_hz = 0;

				if (er >= 0)
					hz = ((unsigned long long)out.rate_msb
					      << 32) | out.rate_lsb;
				if (desc_er == 0 && opp_lo != opp_hi &&
				    (hz == opp_lo || hz == opp_hi)) {
					set_hz = (hz == opp_hi) ? opp_lo :
								  opp_hi;
					back_hz = hz;
				}

				/*
				 * P0-3 round-trip: SET other OPP, GET, SET
				 * the found rate, GET. Both SETs always
				 * issued once the found rate is a listed
				 * OPP, so UART proves restore.
				 */
				if (er >= 0 && set_hz != 0) {
					struct scmi_clk_rate_set_in sin = {
						.flags = SCMI_CLK_RATE_ROUND_CLOSEST,
						.clock_id = 0,
						.rate_lsb = (u32)set_hz,
						.rate_msb = (u32)(set_hz >> 32),
					};
					struct scmi_clk_rate_set_out sout = { 0 };
					struct scmi_msg smsg =
						SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
							    SCMI_CLOCK_RATE_SET,
							    sin, sout);
					struct scmi_clk_rate_get_in ain = {
						.clock_id = 0,
					};
					struct scmi_clk_rate_get_out aout = { 0 };
					struct scmi_msg amsg =
						SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
							    SCMI_CLOCK_RATE_GET,
							    ain, aout);
					struct scmi_clk_rate_set_in bin = {
						.flags = SCMI_CLK_RATE_ROUND_CLOSEST,
						.clock_id = 0,
						.rate_lsb = (u32)back_hz,
						.rate_msb = (u32)(back_hz >> 32),
					};
					struct scmi_clk_rate_set_out bout = { 0 };
					struct scmi_msg bmsg =
						SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
							    SCMI_CLOCK_RATE_SET,
							    bin, bout);
					struct scmi_clk_rate_get_in cin = {
						.clock_id = 0,
					};
					struct scmi_clk_rate_get_out cout = { 0 };
					struct scmi_msg cmsg =
						SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
							    SCMI_CLOCK_RATE_GET,
							    cin, cout);

					set_er = devm_scmi_process_msg(proto,
								      &smsg);
					if (set_er >= 0)
						set_er = scmi_to_linux_errno(
							sout.status);
					/*
					 * SC defers XTAL park / SYS_PLL relock
					 * until after the SET MHU ACK (vendor
					 * USERLOWTASK pattern). Settle so GET
					 * is not in-flight while apps CPU is
					 * on XTAL.
					 */
					if (set_er == 0)
						mdelay(50);
					if (devm_scmi_process_msg(proto,
								  &amsg) >= 0)
						hz_after =
						    ((unsigned long long)
							 aout.rate_msb
						     << 32) |
						    aout.rate_lsb;
					set2_er = devm_scmi_process_msg(proto,
									&bmsg);
					if (set2_er >= 0)
						set2_er = scmi_to_linux_errno(
							bout.status);
					if (set2_er == 0)
						mdelay(50);
					if (devm_scmi_process_msg(proto,
								  &cmsg) >= 0)
						hz_after2 =
						    ((unsigned long long)
							 cout.rate_msb
						     << 32) |
						    cout.rate_lsb;
				}

				/* Round-trip rides the surviving proto-rc line */
				printf("SCMI: proto-rc %02x %d hz %llu set-rc %d hz-after %llu set2-rc %d hz-after2 %llu lo %llu hi %llu\n",
				       protocol_id, pr, hz, set_er, hz_after,
				       set2_er, hz_after2,
				       (unsigned long long)opp_lo,
				       (unsigned long long)opp_hi);
				printf("SCMI: clk0-hz %llu\n", hz);
				printf("SCMI: clk0-rc %d\n", er);
				printf("SCMI: clk0-set-rc %d\n", set_er);
				printf("SCMI: clk0-hz-after %llu\n", hz_after);
				printf("SCMI: clk0-set2-rc %d\n", set2_er);
				printf("SCMI: clk0-hz-after2 %llu\n", hz_after2);
				printf("SCMI: clk0-desc-rc %d\n", desc_er);
				printf("SCMI: clk0-opp-lo %llu\n",
				       (unsigned long long)opp_lo);
				printf("SCMI: clk0-opp-hi %llu\n",
				       (unsigned long long)opp_hi);
			} else {
				printf("SCMI: proto-rc %02x %d\n",
				       protocol_id, pr);
				printf("SCMI: proto-ok %02x %d\n", protocol_id,
				       pr ? 0 : 1);

				/*
				 * P0-3: PD GET + POWER_STATE_SET same-ON
				 * (clock same-rate pattern). Do **not**
				 * power-off CPU0.
				 *
				 * DEN 0056 Type bit 30 = lost
				 * (U-Boot SCMI_PWD_PSTATE_TYPE_LOST;
				 * Linux GENERIC_OFF = BIT(30)).
				 * SCP CORE GET is a composite: L0 =
				 * bits[3:0]. Measured pd0-st 00020110
				 * is Type=not-lost but L0=OFF — SET
				 * of the raw GET would drop CPU0.
				 */
				if (!pr &&
				    IS_ENABLED(CONFIG_SCMI_POWER_DOMAIN) &&
				    protocol_id ==
					    SCMI_PROTOCOL_ID_POWER_DOMAIN) {
					u32 pstate = 0xffffffff;
					u32 pstate_after = 0xffffffff;
					u32 attrs = 0;
					u32 set_ps;
					u8 *pdname = NULL;
					int er = scmi_pwd_state_get(proto, 0,
								    &pstate);
					int set_er = -1;

					printf("SCMI: pd0-rc %d\n", er);
					printf("SCMI: pd0-st %08x\n", pstate);
					if (!er &&
					    !scmi_pwd_attrs(proto, 0, &attrs,
							    &pdname) &&
					    pdname) {
						printf("SCMI: pd0-nm %s\n",
						       pdname);
						free(pdname);
					}
					if (!er) {
						set_ps = pstate;
						if (pstate &
						    SCMI_PWD_PSTATE_TYPE_LOST) {
							printf("SCMI: pd0-set-abort lost\n");
							set_er = -EPERM;
						} else if ((pstate & 0xf) ==
							   0) {
							/*
							 * Composite L0=OFF.
							 * SET raw GET would
							 * drop CPU0 — force
							 * L0=ON (same Type).
							 */
							set_ps = (pstate &
								  ~0xfU) |
								 0x1U;
							printf("SCMI: pd0-set-on %08x\n",
							       set_ps);
							set_er = scmi_pwd_state_set(
								proto, 0, 0,
								set_ps);
						} else {
							set_er = scmi_pwd_state_set(
								proto, 0, 0,
								set_ps);
						}
						if (set_er != -EPERM)
							scmi_pwd_state_get(
								proto, 0,
								&pstate_after);
					}
					printf("SCMI: pd0-set-rc %d\n",
					       set_er);
					printf("SCMI: pd0-st-after %08x\n",
					       pstate_after);
				}
			}
			/* Soft-continue — do not abort initr_dm on protocol fail. */
			ret = 0;
		}
	}

	if (IS_ENABLED(CONFIG_SCMI_RESIDUAL_DIAG)) {
		printf("SCMI: bind-done %d\n", ret);
		/* Let UART FIFO drain before misc_init_r chip_id (PD SET adds 2 lines). */
		mdelay(120);
	}

	return ret;
}

UCLASS_DRIVER(scmi_agent) = {
	.id		= UCLASS_SCMI_AGENT,
	.name		= "scmi_agent",
	.post_bind	= scmi_bind_protocols,
	.per_device_plat_auto = sizeof(struct scmi_agent_priv),
	.per_child_auto	= sizeof(struct scmi_agent_proto_priv),
};
