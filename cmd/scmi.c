// SPDX-License-Identifier: GPL-2.0+
/*
 *  SCMI (System Control and Management Interface) utility command
 *
 *  Copyright (c) 2023 Linaro Limited
 *		Author: AKASHI Takahiro
 */

#include <command.h>
#include <clk.h>
#include <exports.h>
#include <reset.h>
#include <scmi_agent.h>
#include <scmi_agent-uclass.h>
#include <stdlib.h>
#include <asm/io.h>
#include <asm/types.h>
#include <dm/device.h>
#include <dm/uclass.h> /* uclass_get_device */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/string.h>

struct {
	enum scmi_std_protocol id;
	const char *name;
} protocol_name[] = {
	{SCMI_PROTOCOL_ID_BASE, "Base"},
	{SCMI_PROTOCOL_ID_POWER_DOMAIN, "Power domain management"},
	{SCMI_PROTOCOL_ID_SYSTEM, "System power management"},
	{SCMI_PROTOCOL_ID_PERF, "Performance domain management"},
	{SCMI_PROTOCOL_ID_CLOCK, "Clock management"},
	{SCMI_PROTOCOL_ID_SENSOR, "Sensor management"},
	{SCMI_PROTOCOL_ID_RESET_DOMAIN, "Reset domain management"},
	{SCMI_PROTOCOL_ID_VOLTAGE_DOMAIN, "Voltage domain management"},
	{SCMI_PROTOCOL_ID_PINCTRL, "Pin control management"},
};

/**
 * get_agent() - get SCMI agent device
 *
 * Return:	Pointer to SCMI agent device on success, NULL on failure
 */
static struct udevice *get_agent(void)
{
	struct udevice *agent;

	if (uclass_get_device(UCLASS_SCMI_AGENT, 0, &agent)) {
		printf("Cannot find any SCMI agent\n");
		return NULL;
	}

	return agent;
}

/**
 * get_base_proto() - get SCMI base protocol device
 * @agent:	SCMI agent device
 *
 * Return:	Pointer to SCMI base protocol device on success,
 *		NULL on failure
 */
static struct udevice *get_base_proto(struct udevice *agent)
{
	struct udevice *base_proto;

	if (!agent) {
		agent = get_agent();
		if (!agent)
			return NULL;
	}

	base_proto = scmi_get_protocol(agent, SCMI_PROTOCOL_ID_BASE);
	if (!base_proto) {
		printf("SCMI base protocol not found\n");
		return NULL;
	}

	return base_proto;
}

/**
 * get_proto_name() - get the name of SCMI protocol
 *
 * @id:		SCMI Protocol ID
 *
 * Get the printable name of the protocol, @id
 *
 * Return:	Name string on success, NULL on failure
 */
static const char *get_proto_name(enum scmi_std_protocol id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(protocol_name); i++)
		if (id == protocol_name[i].id)
			return protocol_name[i].name;

	return NULL;
}

/**
 * do_scmi_info() - get the information of SCMI services
 *
 * @cmdtp:	Command table
 * @flag:	Command flag
 * @argc:	Number of arguments
 * @argv:	Argument array
 *
 * Get the information of SCMI services using various interfaces
 * provided by the Base protocol.
 *
 * Return:	CMD_RET_SUCCESS on success, CMD_RET_RET_FAILURE on failure
 */
static int do_scmi_info(struct cmd_tbl *cmdtp, int flag, int argc,
			char * const argv[])
{
	struct udevice *agent, *base_proto;
	u32 agent_id, num_protocols;
	u8 *agent_name, *protocols;
	int i, ret;

	if (argc != 1)
		return CMD_RET_USAGE;

	agent = get_agent();
	if (!agent)
		return CMD_RET_FAILURE;
	base_proto = get_base_proto(agent);
	if (!base_proto)
		return CMD_RET_FAILURE;

	printf("SCMI device: %s\n", agent->name);
	printf("  protocol version: 0x%x\n", scmi_version(agent));
	printf("  # of agents: %d\n", scmi_num_agents(agent));
	for (i = 0; i < scmi_num_agents(agent); i++) {
		ret = scmi_base_discover_agent(base_proto, i, &agent_id,
					       &agent_name);
		if (ret) {
			if (ret != -EOPNOTSUPP)
				printf("base_discover_agent() failed for id: %d (%d)\n",
				       i, ret);
			break;
		}
		printf("    %c%2d: %s\n", i == scmi_agent_id(agent) ? '>' : ' ',
		       i, agent_name);
		free(agent_name);
	}
	printf("  # of protocols: %d\n", scmi_num_protocols(agent));
	num_protocols = scmi_num_protocols(agent);
	protocols = scmi_protocols(agent);
	if (protocols)
		for (i = 0; i < num_protocols; i++)
			printf("      %s\n", get_proto_name(protocols[i]));
	printf("  vendor: %s\n", scmi_vendor(agent));
	printf("  sub vendor: %s\n", scmi_sub_vendor(agent));
	printf("  impl version: 0x%x\n", scmi_impl_version(agent));

	return CMD_RET_SUCCESS;
}

/**
 * do_scmi_set_dev() - set access permission to device
 *
 * @cmdtp:	Command table
 * @flag:	Command flag
 * @argc:	Number of arguments
 * @argv:	Argument array
 *
 * Set access permission to device with SCMI_BASE_SET_DEVICE_PERMISSIONS
 *
 * Return:	CMD_RET_SUCCESS on success, CMD_RET_RET_FAILURE on failure
 */
static int do_scmi_set_dev(struct cmd_tbl *cmdtp, int flag, int argc,
			   char * const argv[])
{
	u32 agent_id, device_id, flags, attributes;
	char *end;
	struct udevice *base_proto;
	int ret;

	if (argc != 4)
		return CMD_RET_USAGE;

	agent_id = simple_strtoul(argv[1], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	device_id = simple_strtoul(argv[2], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	flags = simple_strtoul(argv[3], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	base_proto = get_base_proto(NULL);
	if (!base_proto)
		return CMD_RET_FAILURE;

	ret = scmi_base_protocol_message_attrs(base_proto,
					       SCMI_BASE_SET_DEVICE_PERMISSIONS,
					       &attributes);
	if (ret) {
		printf("This operation is not supported\n");
		return CMD_RET_FAILURE;
	}

	ret = scmi_base_set_device_permissions(base_proto, agent_id,
					       device_id, flags);
	if (ret) {
		printf("%s access to device:%u failed (%d)\n",
		       flags ? "Allowing" : "Denying", device_id, ret);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

/**
 * do_scmi_set_proto() - set protocol permission to device
 *
 * @cmdtp:	Command table
 * @flag:	Command flag
 * @argc:	Number of arguments
 * @argv:	Argument array
 *
 * Set protocol permission to device with SCMI_BASE_SET_PROTOCOL_PERMISSIONS
 *
 * Return:	CMD_RET_SUCCESS on success, CMD_RET_RET_FAILURE on failure
 */
static int do_scmi_set_proto(struct cmd_tbl *cmdtp, int flag, int argc,
			     char * const argv[])
{
	u32 agent_id, device_id, protocol_id, flags, attributes;
	char *end;
	struct udevice *base_proto;
	int ret;

	if (argc != 5)
		return CMD_RET_USAGE;

	agent_id = simple_strtoul(argv[1], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	device_id = simple_strtoul(argv[2], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	protocol_id = simple_strtoul(argv[3], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	flags = simple_strtoul(argv[4], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	base_proto = get_base_proto(NULL);
	if (!base_proto)
		return CMD_RET_FAILURE;

	ret = scmi_base_protocol_message_attrs(base_proto,
					       SCMI_BASE_SET_PROTOCOL_PERMISSIONS,
					       &attributes);
	if (ret) {
		printf("This operation is not supported\n");
		return CMD_RET_FAILURE;
	}

	ret = scmi_base_set_protocol_permissions(base_proto, agent_id,
						 device_id, protocol_id,
						 flags);
	if (ret) {
		printf("%s access to protocol:0x%x on device:%u failed (%d)\n",
		       flags ? "Allowing" : "Denying", protocol_id, device_id,
		       ret);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

/**
 * do_scmi_reset() - reset platform resource settings
 *
 * @cmdtp:	Command table
 * @flag:	Command flag
 * @argc:	Number of arguments
 * @argv:	Argument array
 *
 * Reset platform resource settings with BASE_RESET_AGENT_CONFIGURATION
 *
 * Return:	CMD_RET_SUCCESS on success, CMD_RET_RET_FAILURE on failure
 */
static int do_scmi_reset(struct cmd_tbl *cmdtp, int flag, int argc,
			 char * const argv[])
{
	u32 agent_id, flags, attributes;
	char *end;
	struct udevice *base_proto;
	int ret;

	if (argc != 3)
		return CMD_RET_USAGE;

	agent_id = simple_strtoul(argv[1], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	flags = simple_strtoul(argv[2], &end, 16);
	if (*end != '\0')
		return CMD_RET_USAGE;

	base_proto = get_base_proto(NULL);
	if (!base_proto)
		return CMD_RET_FAILURE;

	ret = scmi_base_protocol_message_attrs(base_proto,
					       SCMI_BASE_RESET_AGENT_CONFIGURATION,
					       &attributes);
	if (ret) {
		printf("Reset is not supported\n");
		return CMD_RET_FAILURE;
	}

	ret = scmi_base_reset_agent_configuration(base_proto, agent_id, flags);
	if (ret) {
		printf("Reset failed (%d)\n", ret);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

static struct udevice *get_proto(enum scmi_std_protocol id, const char *what)
{
	struct udevice *agent, *proto;

	agent = get_agent();
	if (!agent)
		return NULL;
	proto = scmi_get_protocol(agent, id);
	if (!proto)
		printf("SCMI %s protocol not found\n", what);
	return proto;
}

static int parse_u32(const char *s, u32 *out)
{
	char *end;

	*out = simple_strtoul(s, &end, 0);
	return (*end != '\0') ? -EINVAL : 0;
}

/**
 * do_scmi_clk() - CLOCK_RATE_GET / SET / CONFIG_SET / DESCRIBE_RATES by id
 *
 * clk get <id>
 * clk set <id> <hz>
 * clk en|dis <id>
 * clk rates <id>
 */
static int do_scmi_clk(struct cmd_tbl *cmdtp, int flag, int argc,
		       char * const argv[])
{
	struct udevice *dev;
	struct clk clk = { 0 };
	u32 id, hz;
	ulong rate;

	if (argc < 3)
		return CMD_RET_USAGE;
	if (parse_u32(argv[2], &id))
		return CMD_RET_USAGE;

	dev = get_proto(SCMI_PROTOCOL_ID_CLOCK, "clock");
	if (!dev)
		return CMD_RET_FAILURE;
	clk.dev = dev;
	clk.id = id;

	if (!strcmp(argv[1], "get")) {
		if (argc != 3)
			return CMD_RET_USAGE;
		rate = clk_get_rate(&clk);
		if (IS_ERR_VALUE(rate)) {
			printf("clk %u get err %ld\n", id, (long)rate);
			return CMD_RET_FAILURE;
		}
		printf("clk %u %lu\n", id, rate);
		return CMD_RET_SUCCESS;
	}
	if (!strcmp(argv[1], "set")) {
		if (argc != 4 || parse_u32(argv[3], &hz))
			return CMD_RET_USAGE;
		rate = clk_set_rate(&clk, hz);
		if (IS_ERR_VALUE(rate)) {
			printf("clk %u set %u err %ld\n", id, hz, (long)rate);
			return CMD_RET_FAILURE;
		}
		printf("clk %u set %u -> %lu\n", id, hz, rate);
		return CMD_RET_SUCCESS;
	}
	if (!strcmp(argv[1], "en") || !strcmp(argv[1], "dis")) {
		int ret;

		if (argc != 3)
			return CMD_RET_USAGE;
		ret = argv[1][1] == 'n' ? clk_enable(&clk) : clk_disable(&clk);
		if (ret) {
			printf("clk %u %s err %d\n", id, argv[1], ret);
			return CMD_RET_FAILURE;
		}
		printf("clk %u %s ok\n", id, argv[1]);
		return CMD_RET_SUCCESS;
	}
	if (!strcmp(argv[1], "rates")) {
		struct scmi_clk_describe_rates_in in;
		struct scmi_clk_describe_rates_out out = { 0 };
		struct scmi_msg msg;
		int ret, i;
		u32 flags, nret;
		u64 hz;

		if (argc != 3)
			return CMD_RET_USAGE;
		in.clock_id = id;
		in.rate_index = 0;
		msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_CLOCK,
				  SCMI_CLOCK_DESCRIBE_RATES, in, out);
		ret = devm_scmi_process_msg(dev, &msg);
		if (ret) {
			printf("clk %u rates err %d\n", id, ret);
			return CMD_RET_FAILURE;
		}
		ret = scmi_to_linux_errno(out.status);
		if (ret) {
			printf("clk %u rates err %d\n", id, ret);
			return CMD_RET_FAILURE;
		}
		flags = out.num_rates_flags;
		nret = flags & SCMI_CLK_DESC_RATES_NRET_MASK;
		if (flags & SCMI_CLK_DESC_RATES_RANGE) {
			u64 min = ((u64)out.rate[0].rate_msb << 32) |
				  out.rate[0].rate_lsb;
			u64 max = ((u64)out.rate[1].rate_msb << 32) |
				  out.rate[1].rate_lsb;
			u64 step = ((u64)out.rate[2].rate_msb << 32) |
				   out.rate[2].rate_lsb;

			printf("clk %u range %u %llu %llu %llu\n",
			       id, nret, min, max, step);
		} else {
			printf("clk %u discrete %u", id, nret);
			for (i = 0; i < nret && i < SCMI_CLK_DESC_RATES_MAX;
			     i++) {
				hz = ((u64)out.rate[i].rate_msb << 32) |
				     out.rate[i].rate_lsb;
				printf(" %llu", hz);
			}
			printf("\n");
		}
		return CMD_RET_SUCCESS;
	}
	return CMD_RET_USAGE;
}

#if CONFIG_IS_ENABLED(SCMI_RESIDUAL_DIAG)
/**
 * do_scmi_rd_reset() - raw RESET_DOMAIN_RESET with a caller-chosen flags word
 *
 * rd reset <id> <flags>
 *
 * Sent on the BASE protocol channel, like do_scmi_perf(): the header carries
 * protocol 0x16 and message 0x4, so the SC routes it to its Reset Domain
 * handler whichever protocol device opened the channel.  This needs no
 * CONFIG_RESET_SCMI and no protocol@16 DT node, and -- unlike the reset
 * uclass, which has only assert (flags 0x2) and deassert (0x0) -- it can send
 * the autonomous-reset flag (SCMI_RD_RESET_FLAG_CYCLE, 0x1) and must-reject
 * words.  reset_state is always 0 (architectural cold reset).
 *
 * Prints the transport return and the raw SCMI status (s32, e.g. -1 =
 * NOT_SUPPORTED), not the errno mapping, so a UART log reads the SC's answer
 * directly.  A served autonomous reset of a whole-SoC domain replies and then
 * takes the board down.
 *
 * Residual / open-SC bring-up only: compiled only where the residual SCMI
 * banner is (CONFIG_SCMI_RESIDUAL_DIAG), so shipping builds -- which set
 * only the CONFIG_SCMI_BASE_PROBE_OPTIONAL soft-fail -- do not carry it.
 */
static int do_scmi_rd_reset(int argc, char * const argv[])
{
	struct udevice *base_proto;
	struct scmi_rd_reset_in in;
	struct scmi_rd_reset_out out = { .status = 0 };
	struct scmi_msg msg;
	u32 id, flags;
	int ret;

	if (argc != 4 || parse_u32(argv[2], &id) || parse_u32(argv[3], &flags))
		return CMD_RET_USAGE;

	base_proto = get_base_proto(NULL);
	if (!base_proto)
		return CMD_RET_FAILURE;

	in.domain_id = id;
	in.flags = flags;
	in.reset_state = 0;
	msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_RESET_DOMAIN, SCMI_RESET_DOMAIN_RESET,
			  in, out);
	ret = devm_scmi_process_msg(base_proto, &msg);
	/* out.status stays 0 if the transport failed before a reply. */
	printf("rd %u reset flags 0x%x ret %d status %d\n", id, flags, ret,
	       out.status);

	return (ret || out.status) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}
#endif

/**
 * do_scmi_rd() - reset-domain ATTRIBUTES / ASSERT / DEASSERT by id
 *
 * rd get <id>
 * rd assert <id>
 * rd deassert <id>
 * rd reset <id> <flags>   (residual builds only, see do_scmi_rd_reset())
 */
static int do_scmi_rd(struct cmd_tbl *cmdtp, int flag, int argc,
		      char * const argv[])
{
	struct udevice *dev;
	struct reset_ctl rst = { 0 };
	struct scmi_rd_attr_in in;
	struct scmi_rd_attr_out out;
	struct scmi_msg msg;
	u32 id;
	int ret;

#if CONFIG_IS_ENABLED(SCMI_RESIDUAL_DIAG)
	if (argc >= 2 && !strcmp(argv[1], "reset"))
		return do_scmi_rd_reset(argc, argv);
#endif
	if (argc != 3)
		return CMD_RET_USAGE;
	if (parse_u32(argv[2], &id))
		return CMD_RET_USAGE;

	dev = get_proto(SCMI_PROTOCOL_ID_RESET_DOMAIN, "reset");
	if (!dev)
		return CMD_RET_FAILURE;
	rst.dev = dev;
	rst.id = id;

	if (!strcmp(argv[1], "get")) {
		in.domain_id = id;
		msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_RESET_DOMAIN,
				  SCMI_RESET_DOMAIN_ATTRIBUTES, in, out);
		ret = devm_scmi_process_msg(dev, &msg);
		if (ret || scmi_to_linux_errno(out.status)) {
			printf("rd %u attr err %d/%d\n", id, ret, out.status);
			return CMD_RET_FAILURE;
		}
		{
			char name[SCMI_RD_NAME_LEN + 1];

			memcpy(name, out.name, SCMI_RD_NAME_LEN);
			name[SCMI_RD_NAME_LEN] = '\0';
			printf("rd %u %s\n", id, name);
		}
		return CMD_RET_SUCCESS;
	}
	if (!strcmp(argv[1], "assert") || !strcmp(argv[1], "deassert")) {
		ret = reset_request(&rst);
		if (ret) {
			printf("rd %u request err %d\n", id, ret);
			return CMD_RET_FAILURE;
		}
		ret = strcmp(argv[1], "assert") ?
			reset_deassert(&rst) : reset_assert(&rst);
		if (ret) {
			printf("rd %u %s err %d\n", id, argv[1], ret);
			return CMD_RET_FAILURE;
		}
		printf("rd %u %s ok\n", id, argv[1]);
		return CMD_RET_SUCCESS;
	}
	return CMD_RET_USAGE;
}

/**
 * do_scmi_perf() - PERF LEVEL_GET / LEVEL_SET / LEVELS-NOTIFY by domain
 *
 * perf get    <domain>          - PERF_LEVEL_GET
 * perf set    <domain> <level>  - PERF_LEVEL_SET (drives the SC's mod_dvfs,
 *                                 which then emits LEVEL_CHANGED to subscribers)
 * perf notify <domain> <on|off> - PERF_NOTIFY_LEVEL (subscribe LEVEL_CHANGED)
 *
 * Sent on the BASE protocol channel: the open GXL SC serves PERFORMANCE
 * (0x13) but this tree has no perf protocol driver to bind, exactly like the
 * CLOCK/SENSOR messages the residual bring-up banner sends.  The message
 * header carries protocol 0x13 and the message id, so the SC routes it to
 * its PERF handler regardless of which protocol device opened the channel.
 * This is the boot-shell proof for the SCP->AP (P2A) notification wire: the
 * subscribe arms the subscriber list and the level set drives the emitter;
 * the delivered packet is then read back from the shared hpri window.
 */
static int do_scmi_perf(struct cmd_tbl *cmdtp, int flag, int argc,
			char * const argv[])
{
	struct udevice *agent, *base_proto;
	struct scmi_msg msg;
	int ret;

	if (argc < 3)
		return CMD_RET_USAGE;

	agent = get_agent();
	if (!agent)
		return CMD_RET_FAILURE;
	base_proto = get_base_proto(agent);
	if (!base_proto)
		return CMD_RET_FAILURE;

	if (!strcmp(argv[1], "notify")) {
		struct scmi_perf_notify_in in;
		struct scmi_perf_out out = { 0 };
		u32 domain;
		int en;

		if (argc != 4)
			return CMD_RET_USAGE;
		if (parse_u32(argv[2], &domain))
			return CMD_RET_USAGE;
		en = !strcmp(argv[3], "on") || !strcmp(argv[3], "1");

		in.domain_id = domain;
		in.notify_enable = en ? 1 : 0;
		msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF, SCMI_PERF_NOTIFY_LEVEL,
				  in, out);
		ret = devm_scmi_process_msg(base_proto, &msg);
		if (ret < 0) {
			printf("perf %u notify err %d\n", domain, ret);
			return CMD_RET_FAILURE;
		}
		ret = scmi_to_linux_errno(out.status);
		printf("perf %u notify %s rc %d\n", domain, en ? "on" : "off",
		       ret);
		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "get")) {
		struct {
			u32 domain_id;
		} in;
		struct {
			s32 status;
			u32 performance_level;
		} out = { 0 };
		u32 domain;

		if (argc != 3 || parse_u32(argv[2], &domain))
			return CMD_RET_USAGE;
		in.domain_id = domain;
		msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF, SCMI_PERF_LEVEL_GET,
				  in, out);
		ret = devm_scmi_process_msg(base_proto, &msg);
		if (ret < 0) {
			printf("perf %u get err %d\n", domain, ret);
			return CMD_RET_FAILURE;
		}
		ret = scmi_to_linux_errno(out.status);
		if (ret) {
			printf("perf %u get rc %d\n", domain, ret);
			return CMD_RET_FAILURE;
		}
		printf("perf %u level %u\n", domain, out.performance_level);
		return CMD_RET_SUCCESS;
	}

	if (!strcmp(argv[1], "set")) {
		struct scmi_perf_in in;
		struct scmi_perf_out out = { 0 };
		u32 domain, level;

		if (argc != 4 || parse_u32(argv[2], &domain) ||
		    parse_u32(argv[3], &level))
			return CMD_RET_USAGE;
		in.domain_id = domain;
		in.perf_level = level;
		msg = SCMI_MSG_IN(SCMI_PROTOCOL_ID_PERF, SCMI_PERF_LEVEL_SET,
				  in, out);
		ret = devm_scmi_process_msg(base_proto, &msg);
		if (ret < 0) {
			printf("perf %u set %u err %d\n", domain, level, ret);
			return CMD_RET_FAILURE;
		}
		ret = scmi_to_linux_errno(out.status);
		if (ret) {
			printf("perf %u set %u rc %d\n", domain, level, ret);
			return CMD_RET_FAILURE;
		}
		printf("perf %u set %u ok\n", domain, level);
		return CMD_RET_SUCCESS;
	}

	return CMD_RET_USAGE;
}

/*
 * scmi rx poll - minimal AP-side notification RX consumer (diagnostic)
 *
 * The shipping U-Boot SCMI client is a request/response driver: after a
 * PERF LEVEL_SET it reads the response and clears the doorbell as its
 * completion-ack (board-gx.c), so a SCP->AP (P2A) notification that shares
 * that channel is consumed as the response and never surfaces. This is the
 * missing rx half the P2A delivery work names: it reads the notification back
 * out-of-band, WITHOUT going through the request/response path, and decodes
 * whether the packet present in each NS SMT window is a NOTIFICATION (type=1)
 * rather than a command/response (type=0).
 *
 * Provenance of the layout/addresses is scp-firmware
 * product/amlogic/gxl/include/gxl_mmap.h (same map the SC composes from):
 *   cpu_scp_hpri 0xC8013400 / cpu_scp_lpri 0xC8013000, message header at
 *   window+0x18 (status/FREE at +0x04), and the Meson MHU doorbell SET/STAT/CLR
 *   at HP 0xC883C41C / LP 0xC883C410 (STAT=SET+4, CLR=SET+8). When the SC is
 *   built with SCP_GXL_P2A_NOTIFICATION+SCP_GXL_P2A_TX_WITNESS it also echoes
 *   [attempted][transmitted][busy-dropped] into the hpri window tail at
 *   0xC80137F0, which tells "dropped at the FREE-bit check" apart from
 *   "transmitted."
 *
 * A notification found here is consumed the way a real rx consumer would: the
 * channel's doorbell is cleared (not ack-cleared as a response) and the window
 * is handed back FREE. Purely a shell diagnostic -- it runs nothing at boot and
 * changes no DT or default behaviour.
 */
#define RX_SMT_HP	0xc8013400	/* cpu_scp_hpri (also the shipping P2A window) */
#define RX_SMT_LP	0xc8013000	/* cpu_scp_lpri (the caveat-2 rx window) */
#define RX_SMT_STATUS	0x04		/* bit0 = FREE */
#define RX_SMT_HEADER	0x18
#define RX_SMT_FREE	BIT(0)
#define RX_MHU_HP_SET	0xc883c41c	/* GXL_MHU_SCP_OUT_HP */
#define RX_MHU_LP_SET	0xc883c410	/* GXL_MHU_SCP_OUT_LP */
#define RX_MHU_STAT	0x04
#define RX_MHU_CLR	0x08
#define RX_TX_WITNESS	0xc80137f0	/* GXL_SCMI_TX_WITNESS_BASE (only if lever on) */

#define RX_HDR_TYPE(h)	(((h) >> 8) & 0x3)   /* GXL custom hdr: type at [9:8] */
#define RX_HDR_PROTO(h)	(((h) >> 10) & 0xff)  /* GXL custom hdr: proto at [17:10] */
#define RX_HDR_MSG(h)	((h) & 0xff)          /* GXL custom hdr: msg_id at [7:0] */
#define RX_TYPE_NOTIF	3   /* GXL wire type: P->A notification = 0b11 */

static int rx_poll_channel(const char *name, ulong smt, ulong db_set, int *found)
{
	u32 hdr = readl((void *)smt + RX_SMT_HEADER);
	u32 status = readl((void *)smt + RX_SMT_STATUS);
	u32 db = readl((void *)db_set + RX_MHU_STAT);
	u32 type = RX_HDR_TYPE(hdr);

	printf("  %s: header=0x%08x type=%u proto=0x%02x msgid=0x%02x db_stat=0x%x free=%d\n",
	       name, hdr, type, RX_HDR_PROTO(hdr), RX_HDR_MSG(hdr), db,
	       !!(status & RX_SMT_FREE));

	if (type == RX_TYPE_NOTIF) {
		printf("  %s: NOTIFICATION protocol=0x%02x message=0x%02x -> consumed\n",
		       name, RX_HDR_PROTO(hdr), RX_HDR_MSG(hdr));
		writel(db, (void *)db_set + RX_MHU_CLR);
		writel(status | RX_SMT_FREE, (void *)smt + RX_SMT_STATUS);
		*found = 1;
	}
	return 0;
}

static int do_scmi_rx(struct cmd_tbl *cmdtp, int flag, int argc,
		      char * const argv[])
{
	int found = 0;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (strcmp(argv[1], "poll"))
		return CMD_RET_USAGE;

	printf("scmi rx poll: P2A notification RX witness\n");
	rx_poll_channel("hp", RX_SMT_HP, RX_MHU_HP_SET, &found);
	rx_poll_channel("lp", RX_SMT_LP, RX_MHU_LP_SET, &found);

	if (readl((void *)RX_TX_WITNESS + 0x0) ||
	    readl((void *)RX_TX_WITNESS + 0x4) ||
	    readl((void *)RX_TX_WITNESS + 0x8))
		printf("  sc-side tx witness: attempted=%u transmitted=%u busy-dropped=%u\n",
		       readl((void *)RX_TX_WITNESS + 0x0),
		       readl((void *)RX_TX_WITNESS + 0x4),
		       readl((void *)RX_TX_WITNESS + 0x8));
	else
		printf("  sc-side tx witness: absent (lever off, or no notify fired)\n");

	return found ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

static struct cmd_tbl cmd_scmi_sub[] = {
	U_BOOT_CMD_MKENT(info, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_info, "", ""),
	U_BOOT_CMD_MKENT(clk, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_clk, "", ""),
	U_BOOT_CMD_MKENT(rd, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_rd, "", ""),
	U_BOOT_CMD_MKENT(perf, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_perf, "", ""),
	U_BOOT_CMD_MKENT(rx, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_rx, "", ""),
	U_BOOT_CMD_MKENT(perm_dev, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_set_dev, "", ""),
	U_BOOT_CMD_MKENT(perm_proto, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_set_proto, "", ""),
	U_BOOT_CMD_MKENT(reset, CONFIG_SYS_MAXARGS, 1,
			 do_scmi_reset, "", ""),
};

/**
 * do_scmi() - SCMI utility
 *
 * @cmdtp:	Command table
 * @flag:	Command flag
 * @argc:	Number of arguments
 * @argv:	Argument array
 *
 * Provide user interfaces to SCMI protocols.
 *
 * Return:	CMD_RET_SUCCESS on success,
 *		CMD_RET_USAGE or CMD_RET_RET_FAILURE on failure
 */
static int do_scmi(struct cmd_tbl *cmdtp, int flag,
		   int argc, char *const argv[])
{
	struct cmd_tbl *cp;

	if (argc < 2)
		return CMD_RET_USAGE;

	argc--; argv++;

	cp = find_cmd_tbl(argv[0], cmd_scmi_sub, ARRAY_SIZE(cmd_scmi_sub));
	if (!cp)
		return CMD_RET_USAGE;

	return cp->cmd(cmdtp, flag, argc, argv);
}

#if CONFIG_IS_ENABLED(SCMI_RESIDUAL_DIAG)
#define SCMI_RD_RESET_HELP \
	" rd reset <id> <flags> - raw RESET_DOMAIN_RESET on the BASE channel\n" \
	"   (flags 0x1 = autonomous reset; prints the raw SCMI status)\n"
#else
#define SCMI_RD_RESET_HELP ""
#endif

U_BOOT_LONGHELP(scmi,
	" - SCMI utility\n"
	" info - get the info of SCMI services\n"
	" clk get <id> - CLOCK_RATE_GET\n"
	" clk set <id> <hz> - CLOCK_RATE_SET then GET\n"
	" clk en|dis <id> - CLOCK_CONFIG_SET\n"
	" clk rates <id> - CLOCK_DESCRIBE_RATES\n"
	" rd get <id> - RESET_DOMAIN_ATTRIBUTES\n"
	" rd assert|deassert <id> - RESET_DOMAIN_RESET\n"
	SCMI_RD_RESET_HELP
	" perf get <domain> - PERF_LEVEL_GET\n"
	" perf set <domain> <level> - PERF_LEVEL_SET (drives the SC emitter)\n"
	" perf notify <domain> <on|off> - PERF_NOTIFY_LEVEL (subscribe LEVEL_CHANGED)\n"
	" rx poll - drain any SCP->AP (P2A) notification from the hpri/lpri windows\n"
	" perm_dev <agent-id in hex> <device-id in hex> <flags in hex>\n"
	"   - set access permission to device\n"
	" perm_proto <agent-id in hex> <device-id in hex> <protocol-id in hex> <flags in hex>\n"
	"   - set protocol permission to device\n"
	" reset <agent-id in hex> <flags in hex>\n"
	"   - reset platform resource settings\n");

U_BOOT_CMD(scmi, CONFIG_SYS_MAXARGS, 0, do_scmi, "SCMI utility",
	   scmi_help_text);
