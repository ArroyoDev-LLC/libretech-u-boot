// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2024 Collabora
 */

#include <command.h>
#include <errno.h>
#include <dm.h>
#include <dm/uclass-internal.h>
#include <usb/tcpm.h>
#include <usb/typec_dp.h>

#define LIMIT_DEV	32
#define LIMIT_PARENT	20

static struct udevice *currdev;

static int do_dev(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int devnum, ret;

	switch (argc) {
	case 2:
		devnum = (int)dectoul(argv[1], NULL);
		ret = tcpm_get(devnum, &currdev);
		if (ret) {
			log_err("Can't get TCPM %d: %d (%s)!\n", devnum, ret, errno_str(ret));
			return CMD_RET_FAILURE;
		}
	case 1:
		if (!currdev) {
			log_err("TCPM device is not set!\n\n");
			return CMD_RET_USAGE;
		}

		printf("dev: %d @ %s\n", dev_seq(currdev), currdev->name);
	}

	return CMD_RET_SUCCESS;
}

static int do_list(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	struct udevice *dev;
	int ret, err = 0;

	printf("| ID | %-*.*s| %-*.*s| %s @ %s\n",
	       LIMIT_DEV, LIMIT_DEV, "Name",
	       LIMIT_PARENT, LIMIT_PARENT, "Parent name",
	       "Parent uclass", "seq");

	for (ret = uclass_first_device_check(UCLASS_TCPM, &dev); dev;
	     ret = uclass_next_device_check(&dev)) {
		if (ret)
			err = ret;

		printf("| %2d | %-*.*s| %-*.*s| %s @ %d | status: %i\n",
		       dev_seq(dev),
		       LIMIT_DEV, LIMIT_DEV, dev->name,
		       LIMIT_PARENT, LIMIT_PARENT, dev->parent->name,
		       dev_get_uclass_name(dev->parent), dev_seq(dev->parent),
		       ret);
	}

	if (err)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

int do_print_info(struct udevice *dev)
{
	enum typec_orientation orientation = tcpm_get_orientation(dev);
	const char *state = tcpm_get_state(dev);
	int pd_rev = tcpm_get_pd_rev(dev);
	int mv = tcpm_get_voltage(dev);
	int ma = tcpm_get_current(dev);
	enum typec_role pwr_role = tcpm_get_pwr_role(dev);
	enum typec_data_role data_role = tcpm_get_data_role(dev);
	bool connected = tcpm_is_connected(dev);

	/*
	 * Print the state whether or not the port is connected. It was shown
	 * only for a disconnected port, which hides it in exactly the case
	 * where it matters most: an attached port that is not doing what you
	 * expect is diagnosed by which state it settled in, and nothing else
	 * printed here distinguishes "no PD contract" from "contract, wrong
	 * roles".
	 */
	printf("TCPM State:  %s\n", state);

	if (!connected)
		return 0;

	printf("Orientation: %s\n", typec_orientation_name[orientation]);
	printf("PD Revision: %s\n", typec_pd_rev_name[pd_rev]);
	printf("Power Role:  %s\n", typec_role_name[pwr_role]);
	printf("Data Role:   %s\n", typec_data_role_name[data_role]);
	printf("Voltage:     %2d.%03d V\n", mv / 1000, mv % 1000);
	printf("Current:     %2d.%03d A\n", ma / 1000, ma % 1000);

	/*
	 * What the partner offered decides what this port can ask of it: a
	 * data-role swap needs DATA_SWAP in its first PDO, and none of it
	 * applies without a contract (Type-C current only).
	 */
	if (pwr_role == TYPEC_SINK) {
		const u32 *caps;
		bool contract;
		unsigned int i, n = tcpm_get_source_caps(dev, &caps, &contract);

		printf("Contract:    %s\n", contract ? "explicit" : "none");
		printf("Source PDOs:");
		for (i = 0; i < n; i++)
			printf(" %08x", caps[i]);
		printf("%s\n", n ? "" : " none");
		if (n)
			printf("PDO1 flags: %s%s%s\n",
			       caps[0] & PDO_FIXED_DATA_SWAP ? " data-swap" : "",
			       caps[0] & PDO_FIXED_USB_COMM ? " usb-comm" : "",
			       caps[0] & PDO_FIXED_DUAL_ROLE ? " dual-role" : "");
	}

	return 0;
}

static int do_info(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	if (!currdev) {
		printf("First, set the TCPM device!\n");
		return CMD_RET_USAGE;
	}

	return do_print_info(currdev);
}

static const char *dp_pin_name(u32 conf)
{
	static const char * const name[] = {
		"A", "B", "C", "D", "E", "F"
	};
	u32 pin = (conf >> 8) & 0xff;
	int i;

	for (i = 0; i < ARRAY_SIZE(name); i++)
		if (pin == BIT(i))
			return name[i];

	return pin ? "multiple" : "none";
}

static int do_dp(struct cmd_tbl *cmdtp, int flag, int argc,
		 char *const argv[])
{
	struct typec_dp_state *dp;
	u16 svids[SVID_DISCOVERY_MAX];
	unsigned int n, i;
	bool enter;
	int ret;

	if (!currdev) {
		printf("First, set the TCPM device!\n");
		return CMD_RET_USAGE;
	}

	/*
	 * The port's own state, not a stack local. `exit` used to assert
	 * active = true and mode_pos = 1 on a local to force an exit against
	 * a port whose real state it could not see -- a guess, and a second
	 * answer to a question the port can answer itself.
	 */
	dp = tcpm_dp_state(currdev);

	enter = argc > 1 && !strcmp(argv[1], "enter");

	/*
	 * `exit` exists so a partner left in DisplayPort alt mode has a
	 * software way out. Without it the only recovery is unplugging the
	 * cable, which is not available on a board at the end of a lease.
	 */
	if (argc > 1 && !strcmp(argv[1], "exit")) {
		bool force = argc > 2 && !strcmp(argv[2], "-f");

		/*
		 * The case that matters is the one this port CANNOT see: a
		 * partner left in alt mode by a previous boot. port->dp is
		 * zeroed at reset, so ->active is false and the honest answer
		 * is "not in alt mode" -- while the monitor is still in it and
		 * the only other way out is unplugging the cable.
		 *
		 * An earlier revision handled that by asserting active = true
		 * and mode_pos = 1 on a stack local, which was a fabrication
		 * and rightly removed. `-f` is the same capability without the
		 * lie: the operator, who can see the monitor, states that the
		 * partner is in the mode, and object position 1 is used
		 * because that is where DisplayPort is found on every partner
		 * this board has met. Without -f the port's own state decides.
		 */
		if (!dp->active && !force) {
			printf("DisplayPort exit: not in alt mode (use 'exit -f' to send one anyway)\n");
			return CMD_RET_SUCCESS;
		}

		if (force && !dp->active) {
			printf("DisplayPort exit: forcing, port has no record of an active mode\n");
			dp->active = true;
			if (!dp->mode_pos)
				dp->mode_pos = 1;
		}

		ret = typec_dp_exit(currdev, dp);
		printf("DisplayPort exit: %s\n",
		       ret ? errno_str(ret) : "left alt mode");

		return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
	}

	ret = enter ? typec_dp_enter(currdev, dp)
		    : typec_dp_discover(currdev, dp);

	/*
	 * Print what was learned even on failure. A refused Enter Mode with
	 * the partner's capability VDO in hand is a diagnosis; the same
	 * failure with nothing printed is a mystery.
	 */
	n = tcpm_get_partner_svids(currdev, svids, ARRAY_SIZE(svids));
	printf("Partner SVIDs:");
	for (i = 0; i < n; i++)
		printf(" %04x%s", svids[i],
		       svids[i] == USB_TYPEC_DP_SID ? " (DisplayPort)" : "");
	if (!n)
		printf(" none");
	printf("\n");

	printf("Port DP VDO:    %08x\n", dp->port_vdo);
	printf("Partner DP VDO: %08x\n", dp->partner_vdo);

	if (dp->partner_vdo)
		printf("  capability %u, signalling %u, %s, pin DFP_D %02x UFP_D %02x\n",
		       DP_CAP_CAPABILITY(dp->partner_vdo),
		       DP_CAP_DP_SIGNALLING(dp->partner_vdo),
		       dp->partner_vdo & DP_CAP_RECEPTACLE ? "receptacle"
							   : "plug",
		       DP_CAP_DFP_D_PIN_ASSIGN(dp->partner_vdo),
		       DP_CAP_UFP_D_PIN_ASSIGN(dp->partner_vdo));

	if (enter) {
		printf("DP Status:      %08x  connection %u, hpd %u, irq_hpd %u%s\n",
		       dp->status, DP_STATUS_CONNECTION(dp->status),
		       !!(dp->status & DP_STATUS_HPD_STATE),
		       !!(dp->status & DP_STATUS_IRQ_HPD),
		       dp->status & DP_STATUS_PREFER_MULTI_FUNC ?
		       ", prefers multi-function" : "");
		printf("DP Configure:   %08x  pin assignment %s\n",
		       dp->conf, dp_pin_name(dp->conf));
	}

	if (ret) {
		printf("DisplayPort %s failed: %d (%s)\n",
		       enter ? "entry" : "discovery", ret, errno_str(ret));
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

static int do_attach(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	enum tcpm_attach_role role = TCPM_ATTACH_DRP;
	unsigned int timeout_ms = 5000;
	int ret;

	if (!currdev) {
		printf("First, set the TCPM device!\n");
		return CMD_RET_USAGE;
	}

	if (argc > 1) {
		if (!strcmp(argv[1], "sink"))
			role = TCPM_ATTACH_SINK;
		else if (!strcmp(argv[1], "source"))
			role = TCPM_ATTACH_SOURCE;
		else if (strcmp(argv[1], "drp"))
			return CMD_RET_USAGE;
	}
	if (argc > 2)
		timeout_ms = dectoul(argv[2], NULL);

	ret = tcpm_force_attach(currdev, role, timeout_ms);
	if (ret)
		printf("TCPM attach: %d (%s)\n", ret, errno_str(ret));
	do_print_info(currdev);

	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

static struct cmd_tbl subcmd[] = {
	U_BOOT_CMD_MKENT(dev, 2, 1, do_dev, "", ""),
	U_BOOT_CMD_MKENT(list, 1, 1, do_list, "", ""),
	U_BOOT_CMD_MKENT(info, 1, 1, do_info, "", ""),
	U_BOOT_CMD_MKENT(dp, 2, 1, do_dp, "", ""),
	U_BOOT_CMD_MKENT(attach, 3, 1, do_attach, "", ""),
};

static int do_tcpm(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	struct cmd_tbl *cmd;

	argc--;
	argv++;

	cmd = find_cmd_tbl(argv[0], subcmd, ARRAY_SIZE(subcmd));
	if (!cmd || argc > cmd->maxargs)
		return CMD_RET_USAGE;

	return cmd->cmd(cmdtp, flag, argc, argv);
}

 /**************************************************/

U_BOOT_CMD(tcpm, CONFIG_SYS_MAXARGS, 1, do_tcpm,
	"TCPM sub-system",
	"list          - list TCPM devices\n"
	"tcpm dev [ID]      - show or [set] operating TCPM device\n"
	"tcpm info          - dump information\n"
	"tcpm dp            - discover the partner's DisplayPort alt mode\n"
	"tcpm dp enter      - and enter it as a source\n"
	"tcpm attach [drp|sink|source] [ms] - restart the port's attach and run it\n"
	"                     to ready, tracing each state (default drp, 5000 ms)\n"
	"tcpm dp exit [-f]  - leave it, for a partner left stuck in alt mode\n"
	"                     (-f sends one even when this port has no record,\n"
	"                      e.g. a partner left in the mode by a prior boot)\n"
);
