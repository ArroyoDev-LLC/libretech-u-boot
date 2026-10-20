// SPDX-License-Identifier: GPL-2.0+
/*
 * Rockchip chip id (include/chip_serial.h): the 16 cpu-id bytes at eFuse
 * CFG_CPUID_OFFSET, raw, as 32 hex -- the bytes themselves, as on Amlogic
 * (claude docs repo design/rules/bootloader/policy.md "One variable, the raw
 * bytes"). Linux's drivers/soc/rockchip/rockchip-cpuinfo.c prints its own mix
 * of them; that string is Linux's, not the bootloader's.
 *
 * U-Boot proper reads the eFuse through the rockchip_efuse / rockchip_otp
 * misc driver.  TPL and SPL have no devicetree driver model for it (TPL is
 * OF_PLATDATA), so they read the same words with the driver's own register
 * sequence (drivers/misc/rockchip-efuse.c rockchip_rk3399_efuse_read() /
 * rockchip_rk3328_efuse_read()) at the SoC's fixed eFuse base.
 */

#include <config.h>
#include <chip_serial.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/string.h>
#if !IS_ENABLED(CONFIG_XPL_BUILD)
#include <dm.h>
#include <misc.h>
#endif

#define CPUID_LEN		16

#if !IS_ENABLED(CONFIG_XPL_BUILD)
static int cpuid_read(u8 *cpuid)
{
#if IS_ENABLED(CONFIG_ROCKCHIP_EFUSE) || IS_ENABLED(CONFIG_ROCKCHIP_OTP)
	struct udevice *dev;
	int ret;

#if IS_ENABLED(CONFIG_ROCKCHIP_EFUSE)
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_DRIVER_GET(rockchip_efuse), &dev);
#else
	ret = uclass_get_device_by_driver(UCLASS_MISC,
					  DM_DRIVER_GET(rockchip_otp), &dev);
#endif
	if (ret)
		return ret;
	ret = misc_read(dev, CFG_CPUID_OFFSET, cpuid, CPUID_LEN);

	return ret < 0 ? ret : 0;
#else
	return -ENOSYS;
#endif
}
#elif IS_ENABLED(CONFIG_ROCKCHIP_RK3399) || IS_ENABLED(CONFIG_ROCKCHIP_RK3328)
#define EFUSE_CTRL		0x0000
#define EFUSE_DOUT		0x0004
#define RK3399_ADDR(n)		((n) << 16)
#define RK3399_STROBSFTSEL	BIT(9)
#define RK3399_RSB		BIT(7)
#define RK3399_PD		BIT(5)
#define EFUSE_PGENB		BIT(3)
#define EFUSE_LOAD		BIT(2)
#define EFUSE_STROBE		BIT(1)
#define EFUSE_CSB		BIT(0)
#define RK3328_INT_STATUS	0x0018
#define RK3328_INT_FINISH	BIT(0)
#define RK3328_DOUT		0x0020
#define RK3328_AUTO_CTRL	0x0024
#define RK3328_AUTO_RD		BIT(1)
#define RK3328_AUTO_ENB		BIT(0)

#if IS_ENABLED(CONFIG_ROCKCHIP_RK3399)
#define EFUSE_BASE		0xff690000	/* efuse0, rk3399-base.dtsi */
#define EFUSE_BYTE_OFF		0
#else
#define EFUSE_BASE		0xff260000	/* rk3328.dtsi */
#define EFUSE_BYTE_OFF		0x60		/* rk3328_data.offset */
#endif

/* one 32-bit eFuse word, @word counted from the start of the array */
static int efuse_word(unsigned int word, u32 *val)
{
	void __iomem *base = (void __iomem *)EFUSE_BASE;
#if IS_ENABLED(CONFIG_ROCKCHIP_RK3399)
	writel(EFUSE_LOAD | EFUSE_PGENB | RK3399_STROBSFTSEL | RK3399_RSB,
	       base + EFUSE_CTRL);
	udelay(1);
	setbits_le32(base + EFUSE_CTRL, EFUSE_STROBE | RK3399_ADDR(word));
	udelay(1);
	*val = readl(base + EFUSE_DOUT);
	clrbits_le32(base + EFUSE_CTRL, EFUSE_STROBE);
	udelay(1);
	writel(RK3399_PD | EFUSE_CSB, base + EFUSE_CTRL);
#else
	int i;

	writel(RK3328_AUTO_RD | RK3328_AUTO_ENB | RK3399_ADDR(word),
	       base + RK3328_AUTO_CTRL);
	udelay(1);
	for (i = 0; !(readl(base + RK3328_INT_STATUS) & RK3328_INT_FINISH);
	     i++) {
		if (i == 50)
			return -ETIMEDOUT;
		udelay(1);
	}
	*val = readl(base + RK3328_DOUT);
	writel(RK3328_INT_FINISH, base + RK3328_INT_STATUS);
#endif

	return 0;
}

static int cpuid_read(u8 *cpuid)
{
	unsigned int off = EFUSE_BYTE_OFF + CFG_CPUID_OFFSET;
	unsigned int i;
	u32 w = 0;
	int ret;

	for (i = 0; i < CPUID_LEN; i++, off++) {
		if (i == 0 || !(off & 3)) {
			ret = efuse_word(off / 4, &w);
			if (ret)
				return ret;
		}
		cpuid[i] = w >> (8 * (off & 3));
	}

	return 0;
}
#else
static int cpuid_read(u8 *cpuid)
{
	return -ENOSYS;
}
#endif

int chip_serial_get(char *buf, size_t size)
{
	u8 cpuid[CPUID_LEN], any = 0;
	int i, ret;

	ret = cpuid_read(cpuid);
	if (ret)
		return ret;
	for (i = 0; i < CPUID_LEN; i++)
		any |= cpuid[i];
	if (!any)
		return -ENODATA;

	return chip_serial_hex(buf, size, cpuid, CPUID_LEN);
}
