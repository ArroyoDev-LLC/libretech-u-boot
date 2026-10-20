// SPDX-License-Identifier: GPL-2.0+
/*
 * `devid` -- drive the EL3 device credential service from the u-boot prompt.
 * A TEST instrument: the product reaches the service through a kernel driver
 * (claude design/software/device-identity.md DI3).  Protocol and SiP ids:
 * TF-A plat/amlogic/common/include/aml_devid.h.
 *
 *   devid ident                 Identify: board id, UID, E
 *   devid burn <addr> <len>     Burn the blob at <addr>
 *   devid answer <nonce-hex>    Answer: serial and the 32-byte MAC
 *
 * Every line it prints starts `DEVID ` so a harness can score it.  Arguments
 * are parsed base 16.
 */

#include <command.h>
#include <hexdump.h>
#include <linux/arm-smccc.h>
#include <linux/string.h>
#include <mapmem.h>
#include <vsprintf.h>

#define SM_GET_SHARE_MEM_INPUT_BASE	0x82000020
#define SM_GET_SHARE_MEM_OUTPUT_BASE	0x82000021
#define SM_DEVID_IDENTIFY		0xC20000D0
#define SM_DEVID_BURN			0xC20000D1
#define SM_DEVID_ANSWER			0xC20000D2
#define DEVID_SHMEM_MAX			0x1000
#define DEVID_NONCE_MAX			64

/* struct aml_devid_ident in TF-A, byte for byte */
struct devid_ident {
	u8 magic[4];
	u32 board_id;
	u8 uid_len;
	u8 uid[16];
	u8 zero[3];
	u8 E[32];
} __packed;

static ulong devid_shmem(ulong fid)
{
	struct arm_smccc_res res;

	arm_smccc_smc(fid, 0, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static void devid_hex(const char *label, const u8 *p, size_t n)
{
	size_t i;

	printf(" %s=", label);
	for (i = 0; i < n; i++)
		printf("%02x", p[i]);
}

static int do_devid_ident(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct arm_smccc_1_2_regs a = { .a0 = SM_DEVID_IDENTIFY }, r;
	struct devid_ident id;
	ulong out = devid_shmem(SM_GET_SHARE_MEM_OUTPUT_BASE);

	arm_smccc_1_2_smc(&a, &r);
	printf("DEVID ident rc=%ld", (long)r.a0);
	if ((long)r.a0 == (long)sizeof(id)) {
		memcpy(&id, map_sysmem(out, sizeof(id)), sizeof(id));
		printf(" magic=%.4s board=0x%x", id.magic, id.board_id);
		devid_hex("uid", id.uid, id.uid_len);
		devid_hex("E", id.E, sizeof(id.E));
	}
	printf("\n");
	return (long)r.a0 == (long)sizeof(id) ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

static int do_devid_burn(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	struct arm_smccc_1_2_regs a = { .a0 = SM_DEVID_BURN }, r;
	ulong in = devid_shmem(SM_GET_SHARE_MEM_INPUT_BASE);
	ulong addr, len;

	if (argc != 3)
		return CMD_RET_USAGE;
	addr = hextoul(argv[1], NULL);
	len = hextoul(argv[2], NULL);
	if (len > DEVID_SHMEM_MAX)
		return CMD_RET_USAGE;
	memcpy(map_sysmem(in, len), map_sysmem(addr, len), len);
	a.a1 = len;
	arm_smccc_1_2_smc(&a, &r);
	printf("DEVID burn rc=%ld\n", (long)r.a0);
	return r.a0 == 0 ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

static int do_devid_answer(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	struct arm_smccc_1_2_regs a = { .a0 = SM_DEVID_ANSWER }, r;
	ulong in = devid_shmem(SM_GET_SHARE_MEM_INPUT_BASE);
	u8 nonce[DEVID_NONCE_MAX], mac[32];
	size_t n;

	if (argc != 2)
		return CMD_RET_USAGE;
	n = strlen(argv[1]) / 2;
	if (n == 0 || n > sizeof(nonce) || hex2bin(nonce, argv[1], n))
		return CMD_RET_USAGE;
	memcpy(map_sysmem(in, n), nonce, n);
	a.a1 = n;
	arm_smccc_1_2_smc(&a, &r);
	printf("DEVID answer rc=%ld", (long)r.a0);
	if (r.a0 == 0) {
		memcpy(mac, &r.a2, 8);
		memcpy(mac + 8, &r.a3, 8);
		memcpy(mac + 16, &r.a4, 8);
		memcpy(mac + 24, &r.a5, 8);
		printf(" serial=%lu", r.a1);
		devid_hex("mac", mac, sizeof(mac));
	}
	printf("\n");
	return r.a0 == 0 ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

static struct cmd_tbl cmd_devid_sub[] = {
	U_BOOT_CMD_MKENT(ident, 1, 0, do_devid_ident, "", ""),
	U_BOOT_CMD_MKENT(burn, 3, 0, do_devid_burn, "", ""),
	U_BOOT_CMD_MKENT(answer, 2, 0, do_devid_answer, "", ""),
};

static int do_devid(struct cmd_tbl *cmdtp, int flag, int argc,
		    char *const argv[])
{
	struct cmd_tbl *c;

	if (argc < 2)
		return CMD_RET_USAGE;
	c = find_cmd_tbl(argv[1], cmd_devid_sub, ARRAY_SIZE(cmd_devid_sub));
	if (!c)
		return CMD_RET_USAGE;
	return c->cmd(cmdtp, flag, argc - 1, argv + 1);
}

U_BOOT_CMD(devid, 4, 0, do_devid,
	   "EL3 device credential service (TEST instrument)",
	   "ident               - board id, UID and the device key E\n"
	   "devid burn <addr> <len>   - burn the blob at <addr> (hex)\n"
	   "devid answer <nonce-hex>  - serial and MAC for <nonce>");
