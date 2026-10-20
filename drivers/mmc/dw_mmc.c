// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2012 SAMSUNG Electronics
 * Jaehoon Chung <jh80.chung@samsung.com>
 * Rajeshawari Shinde <rajeshwari.s@samsung.com>
 */

#include <bouncebuf.h>
#include <cpu_func.h>
#include <div64.h>
#include <dm.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <mmc.h>
#include <dwmmc.h>
#include <time.h>
#include <wait_bit.h>
#include <asm/cache.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <power/regulator.h>

#define PAGE_SIZE 4096

/*
 * Every wait in this driver is bounded by TIME, never by a poll count: a
 * count is a function of how fast the CPU reads a register, which changes
 * with the core clock, the cache state and whatever a cyclic tick does in
 * between, so the same count meant 111 ms on one boot and 404 ms on the next
 * (measured on RK3399, the CMD19 CDONE wait at 100,000 polls).
 *
 * The budgets come from the bus, not from taste:
 *  - command phase: the CIU's own response timeout, TMOUT[7:0] = 255 card
 *    clocks (SD Physical Layer NCR max 64, eMMC NCR max 64), plus the 10 ms
 *    margin Linux's dw_mmc adds to its cto timer; while the card holds DAT0
 *    low the CIU stalls a wait_prvdata_complete command, so that case gets
 *    the busy budget instead;
 *  - data phase, in the CIU's data timeout counter (TMOUT[31:8], card
 *    clocks): a block read / write gets the SD spec's 100 ms read / 250 ms
 *    write ceilings (Part 1 §4.6.2.1-2) with margin, the tuning block gets
 *    the 150 ms / 40 executions the spec allows a whole tuning process
 *    (Part 1 §4.2.4.5) -- a CMD19 whose data never comes then costs 4 ms,
 *    not the 111 ms of the register's reset value (0xffffff clocks at 150
 *    MHz), and never the 42 s it would be at 400 kHz;
 *  - resets: the reset bits self-clear within a few clocks; 100 ms is the
 *    bound that says "the block is dead" rather than "slow".
 */
#define DWMCI_CTO_CLKS		255
#define DWMCI_CTO_MARGIN_MS	10
#define DWMCI_BUSY_TIMEOUT_MS	500
#define DWMCI_DATA_TIMEOUT_MS	500
#define DWMCI_TUNING_TIMEOUT_MS	4
#define DWMCI_RESET_TIMEOUT_MS	100
#define DWMCI_UPD_CLK_TIMEOUT_MS 100

/* IDMAC status bits that mean the HOST side failed, not the bus */
#define DWMCI_IDSTS_FBE		BIT(2)	/* fatal bus error */
#define DWMCI_IDSTS_DU		BIT(4)	/* descriptor unavailable */
#define DWMCI_IDSTS_AIS		BIT(9)	/* abnormal interrupt summary */
#define DWMCI_IDSTS_HOST_ERR	(DWMCI_IDSTS_FBE | DWMCI_IDSTS_DU)

/* Internal DMA Controller (IDMAC) descriptor for 32-bit addressing mode */
struct dwmci_idmac32 {
	u32 des0;	/* Control descriptor */
	u32 des1;	/* Buffer size */
	u32 des2;	/* Buffer physical address */
	u32 des3;	/* Next descriptor physical address */
} __aligned(ARCH_DMA_MINALIGN);

/* Internal DMA Controller (IDMAC) descriptor for 64-bit addressing mode */
struct dwmci_idmac64 {
	u32 des0;	/* Control descriptor */
	u32 des1;	/* Reserved */
	u32 des2;	/* Buffer sizes */
	u32 des3;	/* Reserved */
	u32 des4;	/* Lower 32-bits of Buffer Address Pointer 1 */
	u32 des5;	/* Upper 32-bits of Buffer Address Pointer 1 */
	u32 des6;	/* Lower 32-bits of Next Descriptor Address */
	u32 des7;	/* Upper 32-bits of Next Descriptor Address */
} __aligned(ARCH_DMA_MINALIGN);

/* Register offsets for DW MMC blocks with 32-bit IDMAC */
static const struct dwmci_idmac_regs dwmci_idmac_regs32 = {
	.dbaddrl	= DWMCI_DBADDR,
	.idsts		= DWMCI_IDSTS,
	.idinten	= DWMCI_IDINTEN,
	.dscaddrl	= DWMCI_DSCADDR,
	.bufaddrl	= DWMCI_BUFADDR,
};

/* Register offsets for DW MMC blocks with 64-bit IDMAC */
static const struct dwmci_idmac_regs dwmci_idmac_regs64 = {
	.dbaddrl	= DWMCI_DBADDRL,
	.dbaddru	= DWMCI_DBADDRU,
	.idsts		= DWMCI_IDSTS64,
	.idinten	= DWMCI_IDINTEN64,
	.dscaddrl	= DWMCI_DSCADDRL,
	.dscaddru	= DWMCI_DSCADDRU,
	.bufaddrl	= DWMCI_BUFADDRL,
	.bufaddru	= DWMCI_BUFADDRU,
};

/* Set the reset bit(s) in @value and wait for them to self-clear; 1 = done */
static int dwmci_wait_reset(struct dwmci_host *host, u32 value)
{
	ulong start = get_timer(0);
	u32 ctrl;

	dwmci_writel(host, DWMCI_CTRL, value);

	for (;;) {
		ctrl = dwmci_readl(host, DWMCI_CTRL);
		if (!(ctrl & DWMCI_RESET_ALL))
			return 1;
		if (get_timer(start) > DWMCI_RESET_TIMEOUT_MS) {
			printf("%s: reset 0x%x did not clear (CTRL 0x%08x)\n",
			       host->name, value, ctrl);
			return 0;
		}
	}
}

/*
 * The command-phase budget for the card clock the host is running: the CIU
 * raises RTO after DWMCI_CTO_CLKS card clocks without a response start bit,
 * so CDONE (with or without an error) is due within that plus the margin.
 */
static ulong dwmci_cto_us(struct dwmci_host *host)
{
	uint clock = host->mmc && host->mmc->clock ? host->mmc->clock : 400000;

	return DIV_ROUND_UP(DWMCI_CTO_CLKS * 1000000ULL, clock) +
	       DWMCI_CTO_MARGIN_MS * 1000;
}

/*
 * Program the CIU's data timeout for the transfer about to start: @ms of
 * card clocks, capped at the counter's width (0xffffff) -- which at 150 MHz
 * is 111 ms, so the cap only shortens a budget at the high rates -- and the
 * response timeout at its maximum, which is what the command budget above
 * assumes.
 */
static void dwmci_set_data_timeout(struct dwmci_host *host, uint ms)
{
	uint clock = host->mmc && host->mmc->clock ? host->mmc->clock : 400000;
	u64 clks = (u64)ms * clock;

	do_div(clks, 1000);
	if (!clks || clks > 0xffffff)
		clks = 0xffffff;

	dwmci_writel(host, DWMCI_TMOUT, ((u32)clks << 8) | DWMCI_CTO_CLKS);
}

static void dwmci_set_idma_desc32(struct dwmci_idmac32 *desc, u32 control,
				  u32 buf_size, u32 buf_addr)
{
	phys_addr_t desc_phys = virt_to_phys(desc);
	u32 next_desc_phys = desc_phys + sizeof(struct dwmci_idmac32);

	desc->des0 = control;
	desc->des1 = buf_size;
	desc->des2 = buf_addr;
	desc->des3 = next_desc_phys;
}

static void dwmci_set_idma_desc64(struct dwmci_idmac64 *desc, u32 control,
				  u32 buf_size, u64 buf_addr)
{
	phys_addr_t desc_phys = virt_to_phys(desc);
	u64 next_desc_phys = desc_phys + sizeof(struct dwmci_idmac64);

	desc->des0 = control;
	desc->des1 = 0;
	desc->des2 = buf_size;
	desc->des3 = 0;
	desc->des4 = buf_addr & 0xffffffff;
	desc->des5 = buf_addr >> 32;
	desc->des6 = next_desc_phys & 0xffffffff;
	desc->des7 = next_desc_phys >> 32;
}

static void dwmci_prepare_desc(struct dwmci_host *host, struct mmc_data *data,
			       void *cur_idmac, void *bounce_buffer)
{
	struct dwmci_idmac32 *desc32 = cur_idmac;
	struct dwmci_idmac64 *desc64 = cur_idmac;
	ulong data_start, data_end;
	unsigned int blk_cnt, i;

	data_start = (ulong)cur_idmac;
	blk_cnt = data->blocks;

	for (i = 0;; i++) {
		phys_addr_t buf_phys = virt_to_phys(bounce_buffer);
		unsigned int flags, cnt;

		flags = DWMCI_IDMAC_OWN | DWMCI_IDMAC_CH;
		if (i == 0)
			flags |= DWMCI_IDMAC_FS;
		if (blk_cnt <= 8) {
			flags |= DWMCI_IDMAC_LD;
			cnt = data->blocksize * blk_cnt;
		} else {
			cnt = data->blocksize * 8;
		}

		if (host->dma_64bit_address) {
			dwmci_set_idma_desc64(desc64, flags, cnt,
					      buf_phys + i * PAGE_SIZE);
			desc64++;
		} else {
			dwmci_set_idma_desc32(desc32, flags, cnt,
					      buf_phys + i * PAGE_SIZE);
			desc32++;
		}

		if (blk_cnt <= 8)
			break;
		blk_cnt -= 8;
	}

	if (host->dma_64bit_address)
		data_end = (ulong)desc64;
	else
		data_end = (ulong)desc32;
	flush_dcache_range(data_start, roundup(data_end, ARCH_DMA_MINALIGN));
}

static void dwmci_prepare_data(struct dwmci_host *host, struct mmc_data *data,
			       void *cur_idmac, void *bounce_buffer)
{
	const u32 idmacl = virt_to_phys(cur_idmac) & 0xffffffff;
	const u32 idmacu = (u64)virt_to_phys(cur_idmac) >> 32;
	unsigned long ctrl;

	dwmci_wait_reset(host, DWMCI_CTRL_FIFO_RESET);

	/* Clear IDMAC interrupt */
	dwmci_writel(host, host->regs->idsts, 0xffffffff);

	dwmci_writel(host, host->regs->dbaddrl, idmacl);
	if (host->dma_64bit_address)
		dwmci_writel(host, host->regs->dbaddru, idmacu);

	dwmci_prepare_desc(host, data, cur_idmac, bounce_buffer);

	ctrl = dwmci_readl(host, DWMCI_CTRL);
	ctrl |= DWMCI_IDMAC_EN | DWMCI_DMA_EN;
	dwmci_writel(host, DWMCI_CTRL, ctrl);

	ctrl = dwmci_readl(host, DWMCI_BMOD);
	ctrl |= DWMCI_BMOD_IDMAC_FB | DWMCI_BMOD_IDMAC_EN;
	dwmci_writel(host, DWMCI_BMOD, ctrl);

	dwmci_writel(host, DWMCI_BLKSIZ, data->blocksize);
	dwmci_writel(host, DWMCI_BYTCNT, data->blocksize * data->blocks);
}

static int dwmci_fifo_ready(struct dwmci_host *host, u32 bit, u32 *len)
{
	u32 timeout = 20000;

	*len = dwmci_readl(host, DWMCI_STATUS);
	while (--timeout && (*len & bit)) {
		udelay(200);
		*len = dwmci_readl(host, DWMCI_STATUS);
	}

	if (!timeout) {
		debug("%s: FIFO underflow timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static unsigned int dwmci_get_timeout(struct mmc *mmc, const unsigned int size)
{
	unsigned int timeout;

	timeout = size * 8;	/* counting in bits */
	timeout *= 10;		/* wait 10 times as long */
	timeout /= mmc->clock;
	timeout /= mmc->bus_width;
	timeout /= mmc->ddr_mode ? 2 : 1;
	timeout *= 1000;	/* counting in msec */
	timeout = (timeout < 1000) ? 1000 : timeout;

	return timeout;
}

/*
 * Put the CIU back into a known state after a transfer went wrong: a full
 * controller/FIFO/DMA reset (which also clears a start_cmd the CIU never
 * finished), a clock-update command to re-sync the CIU with the card clock,
 * and an IDMAC reset. Without this, a data command that failed at the
 * response stage -- or one whose completion the CIU never signalled --
 * leaves the data state machine "busy", and every later command, issued
 * with wait_prvdata_complete, sits in the CMD register with start_cmd set
 * until a HLE refuses the next write. A tuning sweep hits exactly that at
 * the edge of its passing window.
 */
static void dwmci_recover(struct dwmci_host *host)
{
	ulong start;
	u32 ctrl;

	/*
	 * Take the IDMAC off the bus before the CIU is reset, as Linux's
	 * dw_mci_idmac_stop_dma() does: an IDMAC still armed for the failed
	 * transfer would otherwise start moving whatever the reset FIFO holds.
	 */
	if (!host->fifo_mode) {
		ctrl = dwmci_readl(host, DWMCI_CTRL);
		ctrl &= ~(DWMCI_IDMAC_EN | DWMCI_DMA_EN);
		dwmci_writel(host, DWMCI_CTRL, ctrl);
		ctrl = dwmci_readl(host, DWMCI_BMOD);
		ctrl &= ~(DWMCI_BMOD_IDMAC_EN | DWMCI_BMOD_IDMAC_FB);
		ctrl |= DWMCI_BMOD_IDMAC_RESET;
		dwmci_writel(host, DWMCI_BMOD, ctrl);
	}

	dwmci_wait_reset(host, DWMCI_RESET_ALL);
	dwmci_writel(host, DWMCI_RINTSTS, DWMCI_INTMSK_ALL);
	dwmci_writel(host, DWMCI_CMD, DWMCI_CMD_PRV_DAT_WAIT |
		     DWMCI_CMD_UPD_CLK | DWMCI_CMD_START);

	start = get_timer(0);
	while (dwmci_readl(host, DWMCI_CMD) & DWMCI_CMD_START) {
		if (get_timer(start) > DWMCI_UPD_CLK_TIMEOUT_MS) {
			/*
			 * The CIU did not take a clock update after a full
			 * reset: reset it once more without the data-busy
			 * wait, which is the only thing that can stall it.
			 */
			printf("%s: clock update after reset did not complete\n",
			       host->name);
			dwmci_wait_reset(host, DWMCI_RESET_ALL);
			dwmci_writel(host, DWMCI_CMD, DWMCI_CMD_UPD_CLK |
				     DWMCI_CMD_START);
			break;
		}
		udelay(10);
	}
	dwmci_writel(host, DWMCI_RINTSTS, DWMCI_INTMSK_ALL);
}

/*
 * Tell the card and the CIU's data state machine that the data transfer
 * they were in is over: a STOP_TRANSMISSION issued with stop_abort_cmd,
 * which the Synopsys databook names as the way the command / data state
 * machines "return correctly to idle" when a transfer in progress is
 * stopped, and what Linux's dw_mmc sends (send_stop_abort()) before it
 * resets on any failed data command -- the tuning block included. The card
 * may already be back in tran (a 64-byte tuning block is long gone by the
 * time software reacts), in which case the STOP is merely illegal there and
 * costs nothing; a card still in data or rcv is put back where the next
 * command expects it. No wait_prvdata_complete: a stuck data phase is
 * exactly what this is for.
 */
static void dwmci_abort_data(struct dwmci_host *host)
{
	ulong start = timer_get_us(), cto = dwmci_cto_us(host);
	u32 mask;

	/*
	 * The STOP's own response must be waited for in full: with a shorter
	 * response timeout left over from the failed command the CIU gives up
	 * on the R1b before it arrives, the reset that follows cuts the CIU
	 * off from a card that is still streaming the aborted read, and every
	 * later command is then answered with silence (measured, RK3399).
	 */
	dwmci_writel(host, DWMCI_TMOUT,
		     (dwmci_readl(host, DWMCI_TMOUT) & ~0xff) | DWMCI_CTO_CLKS);
	dwmci_writel(host, DWMCI_RINTSTS, DWMCI_INTMSK_ALL);
	dwmci_writel(host, DWMCI_CMDARG, 0);
	dwmci_writel(host, DWMCI_CMD, MMC_CMD_STOP_TRANSMISSION |
		     DWMCI_CMD_ABORT_STOP | DWMCI_CMD_RESP_EXP |
		     DWMCI_CMD_CHECK_CRC | DWMCI_CMD_USE_HOLD_REG |
		     DWMCI_CMD_START);

	for (;;) {
		mask = dwmci_readl(host, DWMCI_RINTSTS);
		if (mask & (DWMCI_INTMSK_CDONE | DWMCI_INTMSK_HLE))
			break;
		if (timer_get_us() - start > cto)
			break;
	}
	host->abort_rintsts = mask;

	/* R1b: the card holds DAT0 low until it has actually stopped */
	start = get_timer(0);
	while (dwmci_readl(host, DWMCI_STATUS) & DWMCI_BUSY) {
		if (get_timer(start) > DWMCI_BUSY_TIMEOUT_MS) {
			debug("%s: card still busy %u ms after the abort\n",
			      host->name, DWMCI_BUSY_TIMEOUT_MS);
			break;
		}
	}
	debug("%s: abort: RINTSTS 0x%08x\n", host->name, mask);
}

/*
 * The software backstop over the data phase: for a block transfer the
 * size-derived figure as before (>= 1 s, always above the CIU's own timeout);
 * for the tuning block the CIU's budget plus the command margin -- a CMD19
 * the CIU stops signalling on (neither DTO nor DRTO, data state machine
 * busy; measured on RK3399 at the edge of the eye) then costs 14 ms, not
 * the 1 s the size formula floors at.
 */
static int dwmci_data_transfer(struct dwmci_host *host, struct mmc_data *data,
			       bool tuning)
{
	struct mmc *mmc = host->mmc;
	int ret = 0;
	u32 timeout, mask, size, i, len = 0;
	u32 *buf = NULL;
	ulong start = get_timer(0);

	size = data->blocksize * data->blocks;
	if (data->flags == MMC_DATA_READ)
		buf = (unsigned int *)data->dest;
	else
		buf = (unsigned int *)data->src;

	if (tuning)
		timeout = DWMCI_TUNING_TIMEOUT_MS + DWMCI_CTO_MARGIN_MS;
	else
		timeout = dwmci_get_timeout(mmc, size);

	size /= 4;

	for (;;) {
		mask = dwmci_readl(host, DWMCI_RINTSTS);
		/* Error during data transfer */
		if (mask & (DWMCI_DATA_ERR | DWMCI_DATA_TOUT)) {
			debug("%s: DATA ERROR! rintsts=0x%08x\n", __func__,
			      mask);

			host->err_rintsts = mask;
			dwmci_abort_data(host);
			dwmci_recover(host);

			ret = -EINVAL;
			break;
		}

		/*
		 * The IDMAC's own faults -- a descriptor it could not take,
		 * a fatal AXI/AHB error -- stop the transfer without the CIU
		 * ever raising a data error; DTO would then never come and
		 * this loop would run to its software timeout.
		 */
		if (!host->fifo_mode) {
			u32 idsts = dwmci_readl(host, host->regs->idsts);

			if (idsts & DWMCI_IDSTS_HOST_ERR) {
				printf("%s: IDMAC fault (IDSTS 0x%08x, RINTSTS 0x%08x)\n",
				       host->name, idsts, mask);
				host->err_rintsts = mask;
				dwmci_abort_data(host);
				dwmci_recover(host);
				ret = -EIO;
				break;
			}
		}

		if (host->fifo_mode && size) {
			len = 0;
			if (data->flags == MMC_DATA_READ &&
			    (mask & (DWMCI_INTMSK_RXDR | DWMCI_INTMSK_DTO))) {
				dwmci_writel(host, DWMCI_RINTSTS,
					     mask & (DWMCI_INTMSK_RXDR |
						     DWMCI_INTMSK_DTO));
				while (size) {
					ret = dwmci_fifo_ready(host,
							DWMCI_FIFO_EMPTY,
							&len);
					if (ret < 0)
						break;

					len = (len >> DWMCI_FIFO_SHIFT) &
						    DWMCI_FIFO_MASK;
					len = min(size, len);
					for (i = 0; i < len; i++)
						*buf++ =
						dwmci_readl(host, DWMCI_DATA);
					size = size > len ? (size - len) : 0;
				}
			} else if (data->flags == MMC_DATA_WRITE &&
				   (mask & DWMCI_INTMSK_TXDR)) {
				while (size) {
					ret = dwmci_fifo_ready(host,
							DWMCI_FIFO_FULL,
							&len);
					if (ret < 0)
						break;

					len = host->fifo_depth - ((len >>
						   DWMCI_FIFO_SHIFT) &
						   DWMCI_FIFO_MASK);
					len = min(size, len);
					for (i = 0; i < len; i++)
						dwmci_writel(host, DWMCI_DATA,
							     *buf++);
					size = size > len ? (size - len) : 0;
				}
				dwmci_writel(host, DWMCI_RINTSTS,
					     DWMCI_INTMSK_TXDR);
			}
		}

		/* Data arrived correctly */
		if (mask & DWMCI_INTMSK_DTO) {
			ret = 0;
			break;
		}

		/*
		 * The software backstop above the CIU's own data timeout:
		 * with TMOUT programmed per transfer the CIU reports DRTO
		 * first, so reaching this means the CIU itself stopped
		 * signalling -- which is the one case that must not be left
		 * as it is.
		 */
		if (get_timer(start) > timeout) {
			printf("%s: no data completion in %u ms (RINTSTS 0x%08x)\n",
			       host->name, timeout, mask);
			host->err_rintsts = mask;
			dwmci_abort_data(host);
			dwmci_recover(host);
			ret = -ETIMEDOUT;
			break;
		}
	}

	dwmci_writel(host, DWMCI_RINTSTS, mask);

	return ret;
}

/*
 * Finish the IDMAC side of a transfer. After a data phase that completed,
 * wait (bounded) for the IDMAC's own completion so the last descriptor's
 * bytes have landed before the bounce buffer is copied out; after one that
 * FAILED the IDMAC has already been reset by the recovery, its completion
 * can never come, and waiting for it would add a full second to every
 * data error.
 */
static int dwmci_dma_transfer(struct dwmci_host *host, uint flags,
			      struct bounce_buffer *bbstate, bool failed)
{
	int ret = 0;
	u32 mask, ctrl;

	if (!failed) {
		if (flags == MMC_DATA_READ)
			mask = DWMCI_IDINTEN_RI;
		else
			mask = DWMCI_IDINTEN_TI;

		ret = wait_for_bit_le32(host->ioaddr + host->regs->idsts, mask,
					true, 1000, false);
		if (ret) {
			printf("%s: IDMAC did not complete a finished transfer (IDSTS 0x%08x)\n",
			       host->name,
			       dwmci_readl(host, host->regs->idsts));
			dwmci_recover(host);
		}
	}

	/* Clear interrupts */
	dwmci_writel(host, host->regs->idsts, DWMCI_IDINTEN_MASK);

	ctrl = dwmci_readl(host, DWMCI_CTRL);
	ctrl &= ~DWMCI_DMA_EN;
	dwmci_writel(host, DWMCI_CTRL, ctrl);

	bounce_buffer_stop(bbstate);
	return ret;
}

static int dwmci_set_transfer_mode(struct dwmci_host *host,
				   struct mmc_data *data)
{
	unsigned long mode;

	mode = DWMCI_CMD_DATA_EXP;
	if (data->flags & MMC_DATA_WRITE)
		mode |= DWMCI_CMD_RW;

	return mode;
}

/*
 * Wait for the card to release DAT0 before a command is issued with
 * wait_prvdata_complete. Returns the busy state at the end: a card still busy
 * past the budget is reported by the caller's command timeout, not here --
 * the CIU will hold the command until DAT0 rises, and the command-phase wait
 * below is what bounds that.
 */
static bool dwmci_wait_while_busy(struct dwmci_host *host, struct mmc_cmd *cmd)
{
	ulong start = get_timer(0);

	while (dwmci_readl(host, DWMCI_STATUS) & DWMCI_BUSY) {
		if (get_timer(start) > DWMCI_BUSY_TIMEOUT_MS) {
			debug("%s: card busy for %u ms before CMD%d\n",
			      host->name, DWMCI_BUSY_TIMEOUT_MS, cmd->cmdidx);
			return true;
		}
	}
	return false;
}

/* The tuning block is expected to fail at bad sample phases: report quietly */
static bool dwmci_is_tuning_cmd(const struct mmc_cmd *cmd)
{
	return cmd->cmdidx == MMC_CMD_SEND_TUNING_BLOCK ||
	       cmd->cmdidx == MMC_CMD_SEND_TUNING_BLOCK_HS200;
}

/*
 * Wait for the CIU to finish the command phase: CDONE, or VOLTSW for the
 * voltage-switch command. Bounded by the command budget; a card holding
 * DAT0 low stalls a wait_prvdata_complete command in the CIU, and that case
 * gets the busy budget instead. Returns the RINTSTS that ended the wait, or
 * 0 when nothing did.
 */
static u32 dwmci_wait_cmd_done(struct dwmci_host *host, ulong *elapsed_us)
{
	ulong start = timer_get_us(), budget = dwmci_cto_us(host);
	u32 mask;

	for (;;) {
		mask = dwmci_readl(host, DWMCI_RINTSTS);
		if (host->volt_switching && (mask & DWMCI_INTMSK_VOLTSW))
			break;
		if (mask & (DWMCI_INTMSK_CDONE | DWMCI_INTMSK_HLE))
			break;
		*elapsed_us = timer_get_us() - start;
		if (*elapsed_us > budget) {
			if (budget < DWMCI_BUSY_TIMEOUT_MS * 1000UL &&
			    (dwmci_readl(host, DWMCI_STATUS) & DWMCI_BUSY)) {
				budget = DWMCI_BUSY_TIMEOUT_MS * 1000UL;
				continue;
			}
			return 0;
		}
	}
	*elapsed_us = timer_get_us() - start;
	return mask;
}

static int dwmci_send_cmd_common(struct dwmci_host *host, struct mmc_cmd *cmd,
				 struct mmc_data *data, void *cur_idmac)
{
	int ret, flags = 0;
	ulong elapsed = 0;
	u32 mask;
	bool done, busy;
	struct bounce_buffer bbstate;

	busy = dwmci_wait_while_busy(host, cmd);
	dwmci_writel(host, DWMCI_RINTSTS, DWMCI_INTMSK_ALL);
	if (data)
		dwmci_set_data_timeout(host, dwmci_is_tuning_cmd(cmd) ?
				       DWMCI_TUNING_TIMEOUT_MS :
				       DWMCI_DATA_TIMEOUT_MS);

	if (data) {
		if (host->fifo_mode) {
			dwmci_writel(host, DWMCI_BLKSIZ, data->blocksize);
			dwmci_writel(host, DWMCI_BYTCNT,
				     data->blocksize * data->blocks);
			dwmci_wait_reset(host, DWMCI_CTRL_FIFO_RESET);
		} else {
			if (data->flags == MMC_DATA_READ) {
				ret = bounce_buffer_start(&bbstate,
						(void *)data->dest,
						data->blocksize *
						data->blocks, GEN_BB_WRITE);
			} else {
				ret = bounce_buffer_start(&bbstate,
						(void *)data->src,
						data->blocksize *
						data->blocks, GEN_BB_READ);
			}

			if (ret)
				return ret;

			dwmci_prepare_data(host, data, cur_idmac,
					   bbstate.bounce_buffer);
		}
	}

	dwmci_writel(host, DWMCI_CMDARG, cmd->cmdarg);
	host->err_rintsts = 0;

	if (data)
		flags = dwmci_set_transfer_mode(host, data);

	if ((cmd->resp_type & MMC_RSP_136) && (cmd->resp_type & MMC_RSP_BUSY))
		return -EBUSY;

	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		flags |= DWMCI_CMD_ABORT_STOP;
	else
		flags |= DWMCI_CMD_PRV_DAT_WAIT;

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		flags |= DWMCI_CMD_RESP_EXP;
		if (cmd->resp_type & MMC_RSP_136)
			flags |= DWMCI_CMD_RESP_LENGTH;
	}

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= DWMCI_CMD_CHECK_CRC;

	host->volt_switching = (cmd->cmdidx == SD_CMD_SWITCH_UHS18V);
	if (host->volt_switching)
		flags |= DWMCI_CMD_VOLT_SWITCH;

	flags |= cmd->cmdidx | DWMCI_CMD_START | DWMCI_CMD_USE_HOLD_REG;

	debug("Sending CMD%d\n", cmd->cmdidx);

	dwmci_writel(host, DWMCI_CMD, flags);

	mask = dwmci_wait_cmd_done(host, &elapsed);
	done = mask != 0;

	if (done && host->volt_switching && (mask & DWMCI_INTMSK_VOLTSW))
		dwmci_writel(host, DWMCI_RINTSTS, DWMCI_INTMSK_VOLTSW);
	else if (done && !data && (mask & DWMCI_INTMSK_CDONE))
		dwmci_writel(host, DWMCI_RINTSTS, mask);

	if (!done) {
		/*
		 * The CIU never finished it: start_cmd is still set, no
		 * response bit came. Not a bus event (those all end in CDONE)
		 * but a host-side stall, so say so -- unless it is the
		 * tuning block, where the sweep reports the outcome.
		 */
		if (dwmci_is_tuning_cmd(cmd))
			debug("%s: CMD%d not completed by the CIU in %lu us%s\n",
			      host->name, cmd->cmdidx, elapsed,
			      busy ? " (card busy)" : "");
		else
			printf("%s: CMD%d not completed by the CIU in %lu us%s\n",
			       host->name, cmd->cmdidx, elapsed,
			       busy ? " (card busy)" : "");
		mask = dwmci_readl(host, DWMCI_RINTSTS);
		ret = -ETIMEDOUT;
		goto cmd_failed;
	}

	if (mask & DWMCI_INTMSK_HLE) {
		/*
		 * The CIU refused the CMD write: it was still busy with the
		 * previous command. A driver fault, never a card's.
		 */
		printf("%s: CMD%d refused (hardware locked, RINTSTS 0x%08x)\n",
		       host->name, cmd->cmdidx, mask);
		ret = -EIO;
		goto cmd_failed;
	}

	if (mask & DWMCI_INTMSK_RTO) {
		/*
		 * Timeout here is not necessarily fatal. (e)MMC cards
		 * will splat here when they receive CMD55 as they do
		 * not support this command and that is exactly the way
		 * to tell them apart from SD cards. Thus, this output
		 * below shall be debug(). eMMC cards also do not favor
		 * CMD8, please keep that in mind.
		 */
		debug("%s: Response Timeout\n", __func__);
		ret = -ETIMEDOUT;
		goto cmd_failed;
	} else if (mask & DWMCI_INTMSK_RE) {
		debug("%s: Response Error\n", __func__);
		ret = -EIO;
		goto cmd_failed;
	} else if ((cmd->resp_type & MMC_RSP_CRC) &&
		   (mask & DWMCI_INTMSK_RCRC)) {
		debug("%s: Response CRC Error\n", __func__);
		ret = -EIO;
		goto cmd_failed;
	}

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136) {
			cmd->response[0] = dwmci_readl(host, DWMCI_RESP3);
			cmd->response[1] = dwmci_readl(host, DWMCI_RESP2);
			cmd->response[2] = dwmci_readl(host, DWMCI_RESP1);
			cmd->response[3] = dwmci_readl(host, DWMCI_RESP0);
		} else {
			cmd->response[0] = dwmci_readl(host, DWMCI_RESP0);
		}
	}

	if (data) {
		int dma_ret;

		ret = dwmci_data_transfer(host, data, dwmci_is_tuning_cmd(cmd));
		if (!host->fifo_mode) {
			/*
			 * The DMA completion must not overwrite a data
			 * error: the IDMAC can finish its descriptor for a
			 * block that failed CRC, and the caller would then
			 * be handed corrupt data as a success.
			 */
			dma_ret = dwmci_dma_transfer(host, data->flags,
						     &bbstate, ret != 0);
			if (!ret)
				ret = dma_ret;
		}
	}

	udelay(100);

	return ret;

cmd_failed:
	host->err_rintsts = mask;
	/*
	 * A command that had data armed and did not reach its data phase
	 * (no response, a bad one, or no completion at all) leaves the CIU
	 * -- and possibly the card -- with that transfer pending: abort it
	 * and reset, so the next command can be issued, and release the
	 * bounce buffer the data phase would have. A command with no data
	 * that the CIU never completed (or refused) is reset the same way;
	 * one that merely got no or a bad response is left alone, since the
	 * CIU has already returned to idle and the core expects exactly
	 * that (CMD55 on an eMMC, CMD8 on an old card).
	 */
	if (data || !done || (mask & DWMCI_INTMSK_HLE)) {
		if (data)
			dwmci_abort_data(host);
		dwmci_recover(host);
		if (data && !host->fifo_mode)
			bounce_buffer_stop(&bbstate);
	}

	return ret;
}

/*
 * The RINTSTS bits that mean the BUS failed, not the host's programming.
 * RTO is one of them for a BLOCK command (the only kind this set is applied
 * to): a card that answered CMD17/18/24/25 a moment ago and now does not is
 * a bus event at a marginal phase, not the CMD55-on-eMMC probe the core
 * expects to time out.
 */
#define DWMCI_BUS_ERR	(DWMCI_INTMSK_RTO | DWMCI_INTMSK_RE | DWMCI_INTMSK_RCRC | \
			 DWMCI_INTMSK_DCRC | DWMCI_INTMSK_DRTO | \
			 DWMCI_INTMSK_SBE | DWMCI_INTMSK_EBE)

static bool dwmci_is_block_cmd(const struct mmc_cmd *cmd)
{
	switch (cmd->cmdidx) {
	case MMC_CMD_READ_SINGLE_BLOCK:
	case MMC_CMD_READ_MULTIPLE_BLOCK:
	case MMC_CMD_WRITE_SINGLE_BLOCK:
	case MMC_CMD_WRITE_MULTIPLE_BLOCK:
		return true;
	default:
		return false;
	}
}

/*
 * Issue the command; if it is a block read / write that failed on the bus
 * and the platform has a bus_error handler, let the handler recover (re-tune,
 * lower the clock, ...) and re-issue the same command for as long as it says
 * to. The mmc core has no retry of its own for a block transfer -- one bad
 * CRC fails the whole `mmc read` -- and the handler is where a platform that
 * can re-tune or step its card clock turns that into a retry that passes.
 * The handler's own commands (a STOP, the tuning block) come back through
 * here with @in_bus_error set and are never themselves retried.
 */
static int dwmci_send_cmd_retry(struct dwmci_host *host, struct mmc_cmd *cmd,
				struct mmc_data *data, void *cur_idmac)
{
	int ret, recovered;

	for (;;) {
		ret = dwmci_send_cmd_common(host, cmd, data, cur_idmac);
		if (!ret || !data || !host->bus_error || host->in_bus_error ||
		    !dwmci_is_block_cmd(cmd) ||
		    !(host->err_rintsts & DWMCI_BUS_ERR))
			return ret;

		host->in_bus_error = true;
		recovered = host->bus_error(host, cmd, data, host->err_rintsts);
		host->in_bus_error = false;
		if (recovered)
			return ret;
	}
}

#ifdef CONFIG_DM_MMC
static int dwmci_send_cmd(struct udevice *dev, struct mmc_cmd *cmd,
			  struct mmc_data *data)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else
static int dwmci_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			  struct mmc_data *data)
{
#endif
	struct dwmci_host *host = mmc->priv;
	const size_t buf_size = data ? DIV_ROUND_UP(data->blocks, 8) : 0;

	if (host->dma_64bit_address) {
		ALLOC_CACHE_ALIGN_BUFFER(struct dwmci_idmac64, idmac, buf_size);
		return dwmci_send_cmd_retry(host, cmd, data, idmac);
	} else {
		ALLOC_CACHE_ALIGN_BUFFER(struct dwmci_idmac32, idmac, buf_size);
		return dwmci_send_cmd_retry(host, cmd, data, idmac);
	}
}

static int dwmci_control_clken(struct dwmci_host *host, bool on)
{
	const u32 val = on ? DWMCI_CLKEN_ENABLE | DWMCI_CLKEN_LOW_PWR : 0;
	const u32 cmd_only_clk = DWMCI_CMD_PRV_DAT_WAIT | DWMCI_CMD_UPD_CLK;
	ulong start = get_timer(0);
	u32 flags, mask;

	dwmci_writel(host, DWMCI_CLKENA, val);

	/* Inform CIU */
	flags = DWMCI_CMD_START | cmd_only_clk;
	if (host->volt_switching)
		flags |= DWMCI_CMD_VOLT_SWITCH;
	dwmci_writel(host, DWMCI_CMD, flags);

	/*
	 * A clock-update command is done when the CIU clears start_cmd -- that
	 * is what Linux's mci_send_cmd() waits for. It does NOT raise CDONE
	 * (nothing was sent to the card), which is what this loop used to poll
	 * for: it never saw one and simply fell out of its 10,000 iterations
	 * every time. With the data-busy wait the update can also sit behind
	 * a busy card, hence the busy budget. A CIU that has not taken it by
	 * then is left as it is and reported: a reset here would drop a clock
	 * the update was about to enable.
	 */
	for (;;) {
		mask = dwmci_readl(host, DWMCI_CMD);
		if (!(mask & DWMCI_CMD_START))
			break;
		if (get_timer(start) > DWMCI_BUSY_TIMEOUT_MS) {
			printf("%s: clock update (CLKENA %s) did not complete\n",
			       host->name, on ? "on" : "off");
			return -ETIMEDOUT;
		}
	}
	dwmci_writel(host, DWMCI_RINTSTS, DWMCI_INTMSK_CDONE);

	return 0;
}

/*
 * Update the clock divider.
 *
 * To prevent a clock glitch keep the clock stopped during the update of
 * clock divider and clock source.
 */
static int dwmci_update_div(struct dwmci_host *host, u32 div)
{
	int ret;

	/* Disable clock */
	ret = dwmci_control_clken(host, false);
	if (ret)
		return ret;

	/* Set clock to desired speed */
	dwmci_writel(host, DWMCI_CLKDIV, div);
	dwmci_writel(host, DWMCI_CLKSRC, 0);

	/* Enable clock */
	return dwmci_control_clken(host, true);
}

static int dwmci_setup_bus(struct dwmci_host *host, u32 freq)
{
	u32 div;
	unsigned long sclk;
	int ret;

	if (!freq)
		return 0;

	/*
	 * If host->get_mmc_clk isn't defined, then assume that host->bus_hz is
	 * source clock value. host->bus_hz should be set by user.
	 */
	if (host->get_mmc_clk) {
		sclk = host->get_mmc_clk(host, freq);
	} else if (host->bus_hz) {
		sclk = host->bus_hz;
	} else {
		debug("%s: Didn't get source clock value\n", __func__);
		return -EINVAL;
	}

	if (sclk == freq)
		div = 0; /* bypass mode */
	else
		div = DIV_ROUND_UP(sclk, 2 * freq);

	ret = dwmci_update_div(host, div);
	if (ret)
		return ret;

	host->clock = freq;

	return 0;
}

#if CONFIG_IS_ENABLED(DM_REGULATOR)
/*
 * Set @reg as close to @target as its constraints allow, refusing anything
 * outside [@min_uV, @max_uV]. The SD signalling levels are ranges, not
 * points (Linux: 2.7-3.6 V for "3.3 V", 1.7-1.95 V for "1.8 V"), and a
 * board's vqmmc rail is often a 3.0 V LDO -- an exact 3300000 request is
 * refused by the regulator core and would leave the card at 1.8 V.
 */
static int dwmci_regulator_set_range(struct udevice *reg, int target,
				     int min_uV, int max_uV)
{
	struct dm_regulator_uclass_plat *uc_pdata = dev_get_uclass_plat(reg);
	int uV = target;

	if (uc_pdata->max_uV != -ENODATA && uV > uc_pdata->max_uV)
		uV = uc_pdata->max_uV;
	if (uc_pdata->min_uV != -ENODATA && uV < uc_pdata->min_uV)
		uV = uc_pdata->min_uV;
	if (uV < min_uV || uV > max_uV)
		return -EINVAL;

	return regulator_set_value(reg, uV);
}

static int dwmci_set_vqmmc(struct dwmci_host *host, struct mmc *mmc)
{
	int target, min_uV, max_uV, cur, ret;

	if (!mmc->vqmmc_supply)
		return 0;

	if (mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		target = 1800000;
		min_uV = 1700000;
		max_uV = 1950000;
	} else {
		target = 3300000;
		min_uV = 2700000;
		max_uV = 3600000;
	}

	cur = regulator_get_value(mmc->vqmmc_supply);
	if (cur < 0)
		cur = target;

	/*
	 * Let the platform re-program whatever follows this rail (an IO
	 * domain) with the HIGHER of the two levels before the rail moves,
	 * and the final level after -- so it is never set for 1.8 V while
	 * the pads see more than that.
	 */
	if (host->io_voltage) {
		ret = host->io_voltage(host, max(cur, target), true);
		if (ret && ret != -ENOSYS && ret != -ENOENT)
			return ret;
	}

	ret = dwmci_regulator_set_range(mmc->vqmmc_supply, target, min_uV,
					max_uV);
	debug("%s: vqmmc %d -> %d uV: %d\n", __func__, cur, target, ret);
	if (ret && ret != -ENOSYS)
		return ret;

	if (host->io_voltage) {
		ret = host->io_voltage(host, target, false);
		if (ret && ret != -ENOSYS && ret != -ENOENT)
			return ret;
	}

	return 0;
}
#else
static inline int dwmci_set_vqmmc(struct dwmci_host *host, struct mmc *mmc)
{
	return 0;
}
#endif

#ifdef CONFIG_DM_MMC
/*
 * The core cuts vmmc, calls this, then restores vmmc. A card that switched
 * to 1.8 V only returns to 3.3 V signalling on a power cycle (SD spec), and
 * on a re-init (`mmc dev`, `mmc rescan`) that is the ONLY way to recover it
 * -- but the core's own off/on delay is 2 ms, too short for a board whose
 * card-VDD rail has bulk capacitance to collapse below the reset threshold.
 * Hold it off long enough here, and put vqmmc back to 3.3 V so the card
 * powers up in the initial state.
 */
static int dwmci_host_power_cycle(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct dwmci_host *host = mmc->priv;

	mdelay(host->power_off_ms ? host->power_off_ms : 20);

#if CONFIG_IS_ENABLED(DM_REGULATOR)
	if (mmc->vqmmc_supply)
		dwmci_regulator_set_range(mmc->vqmmc_supply, 3300000,
					  2700000, 3600000);
	if (host->io_voltage)
		host->io_voltage(host, 3300000, false);
#endif
	mmc->signal_voltage = MMC_SIGNAL_VOLTAGE_330;

	return 0;
}

/*
 * DAT0 as the controller sees it: STATUS.data_busy is 1 while the card holds
 * DAT0 low. The core uses this around CMD11 to see the card take the bus
 * low for the switch and release it afterwards.
 */
static int dwmci_wait_dat0(struct udevice *dev, int state, int timeout_us)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
	struct dwmci_host *host = mmc->priv;
	ulong start = timer_get_us();
	u32 status;

	for (;;) {
		status = dwmci_readl(host, DWMCI_STATUS);
		if (!!(status & DWMCI_BUSY) == !state)
			return 0;
		if (timer_get_us() - start > timeout_us)
			return -ETIMEDOUT;
		udelay(10);
	}
}

static int dwmci_set_ios(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else
static int dwmci_set_ios(struct mmc *mmc)
{
#endif
	struct dwmci_host *host = (struct dwmci_host *)mmc->priv;
	u32 ctype, regs;

	debug("Bus width = %d, clock: %d%s\n", mmc->bus_width, mmc->clock,
	      mmc->clk_disable ? " (gated)" : "");

	/*
	 * The core gates the card clock around a signal-voltage switch (SD
	 * spec: >= 5 ms with the clock stopped while VDDIO moves) and while
	 * the card is powered off; honour that instead of re-enabling it.
	 */
	if (mmc->clk_disable)
		dwmci_control_clken(host, false);
	else
		dwmci_setup_bus(host, mmc->clock);
	switch (mmc->bus_width) {
	case 8:
		ctype = DWMCI_CTYPE_8BIT;
		break;
	case 4:
		ctype = DWMCI_CTYPE_4BIT;
		break;
	default:
		ctype = DWMCI_CTYPE_1BIT;
		break;
	}

	dwmci_writel(host, DWMCI_CTYPE, ctype);

	regs = dwmci_readl(host, DWMCI_UHS_REG);
	if (mmc->ddr_mode)
		regs |= DWMCI_DDR_MODE;
	else
		regs &= ~DWMCI_DDR_MODE;
	if (mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_180)
		regs |= DWMCI_UHS_VOLT_18V;
	else
		regs &= ~DWMCI_UHS_VOLT_18V;

	dwmci_writel(host, DWMCI_UHS_REG, regs);

	if (host->clksel) {
		int ret;

		ret = host->clksel(host);
		if (ret)
			return ret;
	}

	return dwmci_set_vqmmc(host, mmc);
}

static void dwmci_init_fifo(struct dwmci_host *host)
{
	u32 fifo_thr, fifoth_val;

	if (!host->fifo_depth) {
		u32 fifo_size;

		/*
		 * Automatically detect FIFO depth from FIFOTH register.
		 * Power-on value of RX_WMark is FIFO_DEPTH-1.
		 */
		fifo_size = dwmci_readl(host, DWMCI_FIFOTH);
		fifo_size = ((fifo_size & RX_WMARK_MASK) >> RX_WMARK_SHIFT) + 1;
		host->fifo_depth = fifo_size;
	}

	fifo_thr = host->fifo_depth / 2;
	fifoth_val = MSIZE(0x2) | RX_WMARK(fifo_thr - 1) | TX_WMARK(fifo_thr);
	dwmci_writel(host, DWMCI_FIFOTH, fifoth_val);
}

static void dwmci_init_dma(struct dwmci_host *host)
{
	int addr_config;

	if (host->fifo_mode)
		return;

	addr_config = (dwmci_readl(host, DWMCI_HCON) >> 27) & 0x1;
	if (addr_config == 1) {
		host->dma_64bit_address = true;
		host->regs = &dwmci_idmac_regs64;
		debug("%s: IDMAC supports 64-bit address mode\n", __func__);
	} else {
		host->dma_64bit_address = false;
		host->regs = &dwmci_idmac_regs32;
		debug("%s: IDMAC supports 32-bit address mode\n", __func__);
	}

	dwmci_writel(host, host->regs->idinten, DWMCI_IDINTEN_MASK);
}

static int dwmci_init(struct mmc *mmc)
{
	struct dwmci_host *host = mmc->priv;

	if (host->board_init)
		host->board_init(host);

	dwmci_writel(host, DWMCI_PWREN, 1);

	if (!dwmci_wait_reset(host, DWMCI_RESET_ALL)) {
		debug("%s[%d] Fail-reset!!\n", __func__, __LINE__);
		return -EIO;
	}

	/* Enumerate at 400KHz */
	dwmci_setup_bus(host, mmc->cfg->f_min);

	dwmci_writel(host, DWMCI_RINTSTS, 0xffffffff);
	dwmci_writel(host, DWMCI_INTMASK, 0);

	dwmci_writel(host, DWMCI_TMOUT, 0xffffffff);

	dwmci_writel(host, DWMCI_BMOD, 1);
	dwmci_init_fifo(host);
	dwmci_init_dma(host);

	dwmci_writel(host, DWMCI_CLKENA, 0);
	dwmci_writel(host, DWMCI_CLKSRC, 0);

	return 0;
}

#if CONFIG_IS_ENABLED(DM_MMC)
int dwmci_probe(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	return dwmci_init(mmc);
}

/*
 * The core calls this at every (re-)initialisation, after the card power
 * cycle and before the first command, "to clear old configuration for mmc
 * rescan". Run the probe-time init again: a full controller reset, the
 * 400 kHz enumeration clock, FIFO / DMA / timeout registers -- so nothing
 * the previous session left in the CIU (a tuned mode's state, a transfer
 * it never completed) carries into the new one. Measured on RK3399: a
 * re-init that carried the state across wedged at the first command after
 * the tuning sweep; one that reset here negotiated SDR104 every time.
 */
static int dwmci_reinit(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);

	return dwmci_init(mmc);
}

const struct dm_mmc_ops dm_dwmci_ops = {
	.send_cmd	= dwmci_send_cmd,
	.set_ios	= dwmci_set_ios,
	.wait_dat0	= dwmci_wait_dat0,
	.host_power_cycle = dwmci_host_power_cycle,
	.reinit		= dwmci_reinit,
};

#else
static const struct mmc_ops dwmci_ops = {
	.send_cmd	= dwmci_send_cmd,
	.set_ios	= dwmci_set_ios,
	.init		= dwmci_init,
};
#endif

void dwmci_setup_cfg(struct mmc_config *cfg, struct dwmci_host *host,
		     u32 max_clk, u32 min_clk)
{
	cfg->name = host->name;
#if !CONFIG_IS_ENABLED(DM_MMC)
	cfg->ops = &dwmci_ops;
#endif
	cfg->f_min = min_clk;
	cfg->f_max = max_clk;

	cfg->voltages = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;

	cfg->host_caps = host->caps;

	if (host->buswidth == 8) {
		cfg->host_caps |= MMC_MODE_8BIT;
		cfg->host_caps &= ~MMC_MODE_4BIT;
	} else {
		cfg->host_caps |= MMC_MODE_4BIT;
		cfg->host_caps &= ~MMC_MODE_8BIT;
	}
	cfg->host_caps |= MMC_MODE_HS | MMC_MODE_HS_52MHz;

	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;
}

#ifdef CONFIG_BLK
int dwmci_bind(struct udevice *dev, struct mmc *mmc, struct mmc_config *cfg)
{
	return mmc_bind(dev, mmc, cfg);
}
#else
int add_dwmci(struct dwmci_host *host, u32 max_clk, u32 min_clk)
{
	dwmci_setup_cfg(&host->cfg, host, max_clk, min_clk);

	host->mmc = mmc_create(&host->cfg, host);
	if (!host->mmc)
		return -1;

	return 0;
}
#endif
