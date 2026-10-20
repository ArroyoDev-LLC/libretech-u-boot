// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2021
 * Samuel Dionne-Riel <samuel@dionne-riel.com>
 */

#include <cli.h>
#include <command.h>
#include <stdio.h>

static int do_pause(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char *message = "Press any key to continue...";
	int c;

	if (argc == 2)
		message = argv[1];

	/* No newline, so it sticks to the bottom of the screen */
	printf("%s", message);

	/*
	 * Wait on "any" key.  ctrlc()/had_ctrlc() cannot be used here: ctrlc()
	 * is tstc()-gated and returns immediately when no key is pending, so it
	 * cannot implement a blocking wait, and had_ctrlc() only reports what a
	 * previous ctrlc() latched.  Test the key we blocked for ourselves, the
	 * same way ctrlc() tests the key it polls.
	 */
	c = getchar();

	/* Since there was no newline, we need it now */
	printf("\n");

	/*
	 * Ctrl-C means "do not continue".  Report it, so that a caller written
	 * as "if pause '...'; then <action>; fi" can actually skip <action>.
	 * A ';' chain ignores this, exactly as it does for every other command.
	 */
	if (c == CTL_CH('c'))
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(pause, 2, 1, do_pause,
	"delay until user input",
	"[prompt] - Wait until users presses any key. [prompt] can be used to customize the message.\n"
	"           Returns failure if Ctrl-C is pressed, success otherwise.\n"
);
