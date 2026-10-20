// SPDX-License-Identifier: GPL-2.0+
/*
 * chip_serial_env(): serial# follows the chip, never a stored value
 * (include/chip_serial.h).  Sandbox has no chip id, so this file supplies one.
 */

#include <chip_serial.h>
#include <env.h>
#include <env_internal.h>
#include <errno.h>
#include <search.h>
#include <test/lib.h>
#include <test/test.h>
#include <test/ut.h>

static int chip_test_fail;

int chip_serial_get(char *buf, size_t size)
{
	static const u8 id[] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef };

	if (chip_test_fail)
		return -EIO;

	return chip_serial_hex(buf, size, id, sizeof(id));
}

static int set_forced(const char *val)
{
	char *const argv[] = { "setenv", "-f", "serial#", (char *)val, NULL };

	return env_do_env_set(0, val ? 4 : 3, argv, H_PROGRAMMATIC);
}

static int lib_test_chip_serial_hex(struct unit_test_state *uts)
{
	static const u8 b[] = { 0x00, 0x9f, 0xa0, 0xff };
	char buf[9];

	ut_assertok(chip_serial_hex(buf, sizeof(buf), b, sizeof(b)));
	ut_asserteq_str("009fa0ff", buf);
	ut_asserteq(-ENOSPC, chip_serial_hex(buf, 8, b, sizeof(b)));

	return 0;
}
LIB_TEST(lib_test_chip_serial_hex, 0);

static int lib_test_chip_serial_env(struct unit_test_state *uts)
{
	chip_test_fail = 0;

	/* no serial#: set from the chip */
	set_forced(NULL);
	ut_assertnull(env_get("serial#"));
	ut_assertok(chip_serial_env());
	ut_asserteq_str("0123456789abcdef", env_get("serial#"));

	/* a stored serial# from another board: replaced */
	ut_assertok(set_forced("00stale0"));
	ut_asserteq_str("00stale0", env_get("serial#"));
	ut_assertok(chip_serial_env());
	ut_asserteq_str("0123456789abcdef", env_get("serial#"));

	/* already right: left alone */
	ut_assertok(chip_serial_env());
	ut_asserteq_str("0123456789abcdef", env_get("serial#"));

	/* chip unreadable: a stored serial# is deleted, not kept */
	ut_assertok(set_forced("00stale0"));
	chip_test_fail = 1;
	ut_asserteq(-EIO, chip_serial_env());
	ut_assertnull(env_get("serial#"));

	/* chip unreadable, nothing stored: still nothing */
	ut_asserteq(-EIO, chip_serial_env());
	ut_assertnull(env_get("serial#"));

	chip_test_fail = 0;

	return 0;
}
LIB_TEST(lib_test_chip_serial_env, 0);

/*
 * chip_serial_ethaddr(): SHA-256 of serial#, first six bytes, unicast and
 * locally administered; eth1addr flips the last bit; a stored ethaddr is kept.
 * sha256("0123456789abcdef") begins 9f9f5111f7b2 (Python hashlib).
 */
static int lib_test_chip_serial_ethaddr(struct unit_test_state *uts)
{
	char *const del_eth[] = { "setenv", "-f", "ethaddr", NULL };
	char *const del_eth1[] = { "setenv", "-f", "eth1addr", NULL };

	chip_test_fail = 0;
	set_forced(NULL);
	env_do_env_set(0, 3, del_eth, H_PROGRAMMATIC);
	env_do_env_set(0, 3, del_eth1, H_PROGRAMMATIC);

	ut_assertok(chip_serial_env());
	ut_assertok(chip_serial_ethaddr());
	ut_asserteq_str("9e:9f:51:11:f7:b2", env_get("ethaddr"));
	ut_asserteq_str("9e:9f:51:11:f7:b3", env_get("eth1addr"));

	/* a stored ethaddr is the override: kept */
	ut_assertok(chip_serial_ethaddr());
	ut_asserteq_str("9e:9f:51:11:f7:b2", env_get("ethaddr"));

	return 0;
}
LIB_TEST(lib_test_chip_serial_ethaddr, 0);
