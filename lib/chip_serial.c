// SPDX-License-Identifier: GPL-2.0+
/*
 * The SoC's unique id as the bootloader's serial string (include/chip_serial.h).
 */

#include <chip_serial.h>
#include <errno.h>
#include <stdio.h>
#include <linux/string.h>
#if !IS_ENABLED(CONFIG_XPL_BUILD)
#include <env.h>
#include <env_internal.h>
#include <hash.h>
#include <net.h>
#include <search.h>
#include <u-boot/sha256.h>
#endif

__weak int chip_serial_get(char *buf, size_t size)
{
	return -ENOSYS;
}

int chip_serial_hex(char *buf, size_t size, const u8 *bytes, size_t len)
{
	static const char hex[] = "0123456789abcdef";
	size_t i;

	if (size < 2 * len + 1)
		return -ENOSPC;
	for (i = 0; i < len; i++) {
		buf[2 * i] = hex[bytes[i] >> 4];
		buf[2 * i + 1] = hex[bytes[i] & 0xf];
	}
	buf[2 * len] = '\0';

	return 0;
}

void chip_serial_announce(void)
{
	char serial[CHIP_SERIAL_MAX];
	int ret;

	ret = chip_serial_get(serial, sizeof(serial));
	if (!ret)
		printf("Chip id: %s\n", serial);
	else if (ret != -ENOSYS)
		printf("Chip id: unreadable (%d)\n", ret);
}

#if !IS_ENABLED(CONFIG_XPL_BUILD)
int chip_serial_env(void)
{
	char serial[CHIP_SERIAL_MAX];
	const char *old;
	int ret;

	old = env_get("serial#");
	ret = chip_serial_get(serial, sizeof(serial));
	if (ret) {
		printf("serial#: chip id unreadable (%d)\n", ret);
		if (old) {
			char *const argv[] = { "setenv", "-f", "serial#", NULL };

			printf("serial#: dropping stored %s\n", old);
			env_do_env_set(0, 3, argv, H_PROGRAMMATIC);
		}
		return ret;
	}
	if (old && !strcmp(old, serial))
		return 0;
	if (old)
		printf("serial#: stored %s is not this chip's, now %s\n", old,
		       serial);

	{
		char *const argv[] = { "setenv", "-f", "serial#", serial, NULL };

		return env_do_env_set(0, 4, argv, H_PROGRAMMATIC) ? -EPERM : 0;
	}
}

int chip_serial_ethaddr(void)
{
#if CONFIG_IS_ENABLED(HASH) && CONFIG_IS_ENABLED(SHA256) && \
    (IS_ENABLED(CONFIG_NET) || IS_ENABLED(CONFIG_NET_LWIP))
	const char *serial = env_get("serial#");
	u8 hash[SHA256_SUM_LEN];
	int size = sizeof(hash);
	u8 mac_addr[6];

	/* A stored ethaddr is the operator's override: keep it */
	if (env_get("ethaddr"))
		return 0;
	if (!serial)
		return -ENODATA;
	if (hash_block("sha256", (void *)serial, strlen(serial), hash, &size))
		return -EIO;

	memcpy(mac_addr, hash, 6);
	mac_addr[0] &= 0xfe;	/* unicast */
	mac_addr[0] |= 0x02;	/* locally administered (IEEE 802) */
	eth_env_set_enetaddr("ethaddr", mac_addr);

	/* the second interface: the last byte's low bit flipped */
	mac_addr[5] ^= 0x01;
	eth_env_set_enetaddr("eth1addr", mac_addr);

	return 0;
#else
	return -ENOSYS;
#endif
}
#endif
