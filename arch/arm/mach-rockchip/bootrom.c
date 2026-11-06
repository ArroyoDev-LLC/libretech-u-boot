// SPDX-License-Identifier: GPL-2.0+
/**
 * Copyright (c) 2017 Google, Inc
 */

#include <config.h>
#include <hang.h>
#include <setjmp.h>
#include <asm/arch-rockchip/bootrom.h>
#include <asm/arch-rockchip/boot_mode.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/system.h>

/*
 * Force the jmp_buf to the data-section, as .bss will not be valid
 * when save_boot_params is invoked.
 */
static jmp_buf brom_ctx __section(".data");

static void _back_to_bootrom(enum rockchip_bootrom_cmd brom_cmd)
{
	longjmp(brom_ctx, brom_cmd);
}

void back_to_bootrom(enum rockchip_bootrom_cmd brom_cmd)
{
#if CONFIG_IS_ENABLED(LIBCOMMON_SUPPORT)
	puts("Returning to boot ROM...\n");
#endif
	_back_to_bootrom(brom_cmd);
}

#if defined(CONFIG_ROCKCHIP_RK3399) && defined(CONFIG_TPL_BUILD)
/*
 * One opcode to the SPI NOR on SPI1 CS0, through the controller as the
 * BootROM left it after reading the TPL (or trying to): frames forced to
 * 8 bits, nothing else touched. Runs before the console and the timer, so the
 * wait for the shift register to empty is a bounded spin, not a udelay().
 */
#define RK3399_SPI1_BASE	0xff1d0000
#define RK_SPI_CTRLR0		0x00
#define RK_SPI_ENR		0x08
#define RK_SPI_SER		0x0c
#define RK_SPI_SR		0x24
#define RK_SPI_TXDR		0x400
#define RK_SPI_SR_BUSY		BIT(0)
#define RK_SPI_SR_TF_EMPT	BIT(2)

static void rk3399_spi1_nor_opcode(u8 op)
{
	void __iomem *b = (void __iomem *)RK3399_SPI1_BASE;
	int i;

	writel(0, b + RK_SPI_ENR);
	clrsetbits_le32(b + RK_SPI_CTRLR0, 3, 1);	/* DFS 8 bit */
	writel(1, b + RK_SPI_SER);
	writel(1, b + RK_SPI_ENR);
	writel(op, b + RK_SPI_TXDR);
	for (i = 0; i < 1000000; i++)
		if ((readl(b + RK_SPI_SR) & (RK_SPI_SR_BUSY | RK_SPI_SR_TF_EMPT)) ==
		    RK_SPI_SR_TF_EMPT)
			break;
	writel(0, b + RK_SPI_SER);
	writel(0, b + RK_SPI_ENR);
}
#endif

/*
 * we back to bootrom download mode if get a
 * BOOT_BROM_DOWNLOAD flag in boot mode register
 *
 * note: the boot mode register is configured by
 * application(next stage bootloader, kernel, etc),
 * and the bootrom never check this register, so we need
 * to check it and back to bootrom at very early bootstage(before
 * some basic configurations(such as interrupts) been
 * changed by TPL/SPL, as the bootrom download operation
 * relies on many default settings(such as interrupts) by
 * itself.
 */
static bool check_back_to_brom_dnl_flag(void)
{
	u32 boot_mode;

	if (CONFIG_ROCKCHIP_BOOT_MODE_REG) {
		boot_mode = readl(CONFIG_ROCKCHIP_BOOT_MODE_REG);
		if (boot_mode == BOOT_BROM_DOWNLOAD) {
			u32 src;

			writel(0, CONFIG_ROCKCHIP_BOOT_MODE_REG);
			/*
			 * The flag asked for the BootROM's download mode, and
			 * a stage the BootROM loaded OVER USB is already in
			 * it: the flag is satisfied, not pending. Returning
			 * BROM_BOOT_ENTER_DNL here would be a bounce, and the
			 * BootROM does not survive one from a USB-loaded stage
			 * -- measured on RK3399: the device stays enumerated
			 * and answers no transfer until a power cycle. This
			 * happens whenever the flag was set (an SPL that
			 * failed every boot device, a `reboot loader`) and the
			 * BootROM then found no bootable medium and fell into
			 * download mode by itself, leaving the flag armed for
			 * the first TPL sent over the cable.
			 */
#ifdef CFG_IRAM_BASE
			src = readl(BROM_BOOTSOURCE_ID_ADDR);
#else
			src = BROM_BOOTSOURCE_UNKNOWN;
#endif
			if (src == BROM_BOOTSOURCE_USB || src == 0x81)
				return false;
#if defined(CONFIG_ROCKCHIP_RK3399) && defined(CONFIG_TPL_BUILD)
			/*
			 * From a SPI NOR boot the RK3399 BootROM does not take
			 * BROM_BOOT_ENTER_DNL: it reads this TPL from the NOR
			 * again (measured on ROC-RK3399-PC: the second pass
			 * found the flag already cleared). Put the NOR in Deep
			 * Power-Down first, so the BootROM's next read finds no
			 * bootable NOR and it falls into download mode the way a
			 * blank medium does. A TPL loaded over USB wakes it.
			 */
			if (src == BROM_BOOTSOURCE_SPINOR)
				rk3399_spi1_nor_opcode(0xb9);
#endif
			return true;
		}
	}

	return false;
}

/*
 * All Rockchip BROM implementations enter with a valid stack-pointer,
 * so this can safely be implemented in C (providing a single
 * implementation both for ARMv7 and AArch64).
 */
int save_boot_params(void)
{
	int  ret = setjmp(brom_ctx);

	switch (ret) {
	case 0:
#if defined(CONFIG_ROCKCHIP_RK3399) && defined(CONFIG_TPL_BUILD) && defined(CFG_IRAM_BASE)
		/*
		 * Loaded over USB: release the NOR from Deep Power-Down (0xAB),
		 * which the flag path above, or a bench recipe, may have left
		 * it in; the session and the next boot read it. A NOR that is
		 * not asleep ignores the opcode.
		 */
		{
			u32 src = readl(BROM_BOOTSOURCE_ID_ADDR);

			if (src == BROM_BOOTSOURCE_USB || src == 0x81)
				rk3399_spi1_nor_opcode(0xab);
		}
#endif
		if (check_back_to_brom_dnl_flag()) {
#if defined(CONFIG_ROCKCHIP_RK3399) && defined(CONFIG_TPL_BUILD)
			/* the download mode waits armed (B186 F25) */
			rk3399_tsadc_arm_early();
#endif
			_back_to_bootrom(BROM_BOOT_ENTER_DNL);
		}
		/*
		 * This is the initial pass through this function
		 * (i.e. saving the context), setjmp just setup up the
		 * brom_ctx: transfer back into the startup-code at
		 * 'save_boot_params_ret' and let the compiler know
		 * that this will not return.
		 */
		save_boot_params_ret();
		while (true)
			/* does not return */;
		break;

	case BROM_BOOT_NEXTSTAGE:
		/*
		 * To instruct the BROM to boot the next stage, we
		 * need to return 0 to it: i.e. we need to rewrite
		 * the return code once more.
		 */
		ret = 0;
		break;
	case BROM_BOOT_ENTER_DNL:
		/*
		 * A non-zero return value will instruct the BROM enter
		 * download mode.
		 */
		ret = 1;
		break;
	default:
#if CONFIG_IS_ENABLED(LIBCOMMON_SUPPORT)
		puts("FATAL: unexpected command to back_to_bootrom()\n");
#endif
		hang();
	};

	return ret;
}
