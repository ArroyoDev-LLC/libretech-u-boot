// SPDX-License-Identifier: GPL-2.0+
/*
 * Cadence MHDP DisplayPort controller (cdn-dp) on RK3399.
 *
 * Clocks, resets, an APB liveness test, the ucpu firmware load and the full
 * DPTX video path, exposed both as UCLASS_DISPLAY ops and as a `cdn_dp`
 * command for bring-up.
 *
 * The ops report success only when every step of the sequence succeeded, and
 * that is load-bearing rather than tidy: rk_display_init() treats a successful
 * .read_edid as "this output is usable" and stops looking, and display_enable()
 * returning 0 marks the encoder in use and stops rk_vop_probe() walking its
 * remaining output ports. An op that returned 0 for a link it cannot light
 * would produce a board with no picture AND no error, having skipped the HDMI
 * that would have worked.
 */

#include <clk.h>
#include <command.h>
#include <console.h>
#include <display.h>
#include <div64.h>
#include <dm.h>
#include <edid.h>
#include <log.h>
#include <malloc.h>
#include <reset.h>
#include <display_options.h>
#include <generic-phy.h>
#include <syscon.h>
#include <vsprintf.h>
#include <phy/rockchip-typec.h>
#include <asm/io.h>
#include <asm/arch-rockchip/clock.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/string.h>
#if IS_ENABLED(CONFIG_CDN_DP_FW_SCRIPT)
#include <fw_loader.h>
#endif

/*
 * APB register map. These offsets describe the controller's own address
 * decode, as documented in Linux's cdn-dp-reg.h -- they say nothing about the
 * firmware image, which stays opaque.
 */
#define CDN_DP_APB_CTRL		0x00
#define  APB_IRAM_PATH		BIT(2)	/* APB may reach IRAM */
#define  APB_DRAM_PATH		BIT(1)	/* APB may reach DRAM */
#define  APB_XT_RESET		BIT(0)	/* hold the ucpu in reset */
#define CDN_DP_KEEP_ALIVE	0x18
#define CDN_DP_VER_L		0x1c
#define CDN_DP_VER_H		0x20

#define CDN_DP_ADDR_IMEM	0x10000	/* 64 KiB */
#define CDN_DP_ADDR_DMEM	0x20000	/* 32 KiB */
#define CDN_DP_DMEM_SIZE	0x8000

/* Mailbox, and the clock/reset ("CAR") block. Both from Linux's cdn-dp-reg.c. */
#define CDN_DP_MAILBOX_FULL	0x08
#define CDN_DP_MAILBOX_EMPTY	0x0c
#define CDN_DP_MAILBOX_WR_DATA	0x10
#define CDN_DP_MAILBOX_RD_DATA	0x14
#define CDN_DP_SW_CLK_H		0x40
#define CDN_DP_SW_EVENTS0	0x44
#define CDN_DP_APB_INT_MASK	0x6c

#define CDN_DP_SOURCE_DPTX_CAR		0x0904
#define CDN_DP_SOURCE_PHY_CAR		0x0908
#define CDN_DP_SOURCE_PKT_CAR		0x0918
#define CDN_DP_SOURCE_AIF_CAR		0x091c
#define CDN_DP_SOURCE_CIPHER_CAR	0x0920
#define CDN_DP_SOURCE_CRYPTO_CAR	0x0924

#define DPTX_FRMR_DATA_CLK_RSTN_EN	BIT(11)
#define DPTX_FRMR_DATA_CLK_EN		BIT(10)
#define DPTX_PHY_DATA_RSTN_EN		BIT(9)
#define DPTX_PHY_DATA_CLK_EN		BIT(8)
#define DPTX_PHY_CHAR_RSTN_EN		BIT(7)
#define DPTX_PHY_CHAR_CLK_EN		BIT(6)
#define SOURCE_AUX_SYS_CLK_RSTN_EN	BIT(5)
#define SOURCE_AUX_SYS_CLK_EN		BIT(4)
#define DPTX_SYS_CLK_RSTN_EN		BIT(3)
#define DPTX_SYS_CLK_EN			BIT(2)
#define CFG_DPTX_VIF_CLK_RSTN_EN	BIT(1)
#define CFG_DPTX_VIF_CLK_EN		BIT(0)

#define SOURCE_PHY_RSTN_EN		BIT(1)
#define SOURCE_PHY_CLK_EN		BIT(0)

#define SOURCE_PKT_SYS_RSTN_EN		BIT(3)
#define SOURCE_PKT_SYS_CLK_EN		BIT(2)
#define SOURCE_PKT_DATA_RSTN_EN		BIT(1)
#define SOURCE_PKT_DATA_CLK_EN		BIT(0)

#define SPDIF_CDR_CLK_RSTN_EN		BIT(5)
#define SPDIF_CDR_CLK_EN		BIT(4)
#define SOURCE_AIF_SYS_RSTN_EN		BIT(3)
#define SOURCE_AIF_SYS_CLK_EN		BIT(2)
#define SOURCE_AIF_CLK_RSTN_EN		BIT(1)
#define SOURCE_AIF_CLK_EN		BIT(0)

#define SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN	BIT(3)
#define SOURCE_CIPHER_SYS_CLK_EN		BIT(2)
#define SOURCE_CIPHER_CHAR_CLK_RSTN_EN		BIT(1)
#define SOURCE_CIPHER_CHAR_CLK_EN		BIT(0)

#define SOURCE_CRYPTO_SYS_CLK_RSTN_EN	BIT(1)
#define SOURCE_CRYPTO_SYS_CLK_EN	BIT(0)

#define MB_MODULE_ID_GENERAL	0x0a
#define GENERAL_MAIN_CONTROL	0x01
#define FW_STANDBY		0
#define FW_ACTIVE		1

/*
 * The DPTX module's opcodes, from Linux's cdn-dp-reg.h:323-340. Only the ones
 * this driver issues are named; the rest of the module's surface (audio setup,
 * power management, forced lanes) is not reachable from here and an unused
 * #define is an invitation to guess at a protocol we have not exercised.
 *
 * DPTX_READ_REGISTER (0x07) is deliberately absent. Linux #defines it and never
 * sends it, so the GPL reference does not show its request framing or its reply
 * layout -- and a mailbox request framed wrongly is not a failed read, it
 * desyncs the byte stream for every transaction after it (see
 * cdn_dp_mailbox_validate_receive()). Nothing below needs a DPTX register read:
 * link training reports through DPTX_READ_LINK_STAT, and config_video is
 * write-only.
 */
#define MB_MODULE_ID_DP_TX	0x01
#define DPTX_GET_EDID		0x02
#define DPTX_READ_DPCD		0x03
#define DPTX_WRITE_DPCD		0x04
#define DPTX_ENABLE_EVENT	0x05
#define DPTX_WRITE_FIELD	0x08
#define DPTX_TRAINING_CONTROL	0x09
#define DPTX_READ_EVENT		0x0a
#define DPTX_READ_LINK_STAT	0x0b
#define DPTX_SET_VIDEO		0x0c
#define DPTX_HPD_STATE		0x11

#define DPTX_EVENT_ENABLE_HPD		BIT(0)
#define DPTX_EVENT_ENABLE_TRAINING	BIT(1)

/* DPTX_TRAINING_CONTROL payload, cdn-dp-reg.h:348-350. */
#define LINK_TRAINING_NOT_ACTIVE	0
#define LINK_TRAINING_RUN		1
#define LINK_TRAINING_RESTART		2

/* DPTX_SET_VIDEO payload, cdn-dp-reg.h:352-353. */
#define CONTROL_VIDEO_IDLE		0
#define CONTROL_VIDEO_VALID		1

/*
 * The training-status bits the firmware reports in the SECOND byte of a
 * DPTX_READ_EVENT reply (cdn-dp-reg.h:373-380). Linux tests exactly one of
 * them, EQ_PHASE_FINISHED, and ignores the rest; the failure bits are named
 * here because a training timeout with CLK_RECOVERY_FAILED set is a different
 * fault from one with nothing set at all, and printing "timed out" for both
 * throws that away.
 */
#define FULL_LT_STARTED			BIT(0)
#define FAST_LT_STARTED			BIT(1)
#define CLK_RECOVERY_FINISHED		BIT(2)
#define EQ_PHASE_FINISHED		BIT(3)
#define FAST_LT_START_FINISHED		BIT(4)
#define CLK_RECOVERY_FAILED		BIT(5)
#define EQ_PHASE_FAILED			BIT(6)
#define FAST_LT_FAILED			BIT(7)

/*
 * Linux's LINK_TRAINING_RETRY_MS / LINK_TRAINING_TIMEOUT_MS, cdn-dp-reg.c:21-22.
 * The budget is the firmware's, not ours: the host issues one
 * DPTX_TRAINING_CONTROL and then only watches, so shortening this does not
 * make training faster, it just makes a slow sink look like a dead one.
 */
#define LINK_TRAINING_RETRY_MS		20
#define LINK_TRAINING_TIMEOUT_MS	500

/*
 * One EDID block, and the response framing around it. The firmware answers a
 * DPTX_GET_EDID with two bytes of its own (length, and the 128-byte-pair index
 * it read) ahead of the block, so the size handed to the header validator is
 * EDID_BLOCK_SIZE + 2 -- a validator fed the bare 128 rejects every reply.
 */
/*
 * Gap between EDID retries, and it is measured rather than guessed.
 *
 * 20 ms was tried first and is demonstrably too short in one specific case: an
 * EDID read issued immediately after a PREVIOUS SUCCESSFUL one failed all four
 * attempts even with three 20 ms gaps between them, i.e. the DDC engine was
 * still unavailable 60 ms after a completed 130-byte transfer. In the easier
 * case -- an EDID read following ordinary DPCD traffic -- a single 20 ms gap
 * was enough, and the second attempt succeeded where the first had not.
 *
 * So the recovery interval after a full block read is somewhere above 60 ms.
 * 200 ms is chosen with margin rather than tuned to the edge: this is a
 * once-per-boot read on a path that already spends seconds on PD negotiation
 * and firmware load, and the worst case only materialises when the read is
 * failing anyway.
 */
#define EDID_RETRY_DELAY_MS	200

#define EDID_BLOCK_SIZE		128
#define EDID_EXT_COUNT_OFF	126
#define EDID_DTD_OFF		54	/* first detailed timing descriptor */
#define EDID_VERSION_OFF	18
#define EDID_REVISION_OFF	19
#define EDID_INPUT_DEF_OFF	20	/* video input definition, carries bpc */

/*
 * Sleeping polls, not readx_poll_timeout(). U-Boot's readx_poll_timeout()
 * busy-spins with no delay between reads, and a multi-second mailbox timeout
 * spun that way hammers the APB for the entire wait.
 */
#define MAILBOX_POLL_US		10
#define MAILBOX_TIMEOUT_US	5000000
#define KEEP_ALIVE_POLL_US	2000
#define KEEP_ALIVE_TIMEOUT_US	1000000

/*
 * The firmware container's header, and the ONLY structural fact this driver
 * uses about dptx.bin. It is Linux's struct cdn_firmware_header, not something
 * learned from the blob: LICENCE.rockchip forbids reverse engineering,
 * decompilation and disassembly, so the two sections are copied verbatim to
 * the addresses the header names and are never interpreted.
 */
struct cdn_dp_fw_header {
	__le32 size_bytes;	/* whole file, header included */
	__le32 header_size;	/* 16 */
	__le32 iram_size;
	__le32 dram_size;
};

#define CDN_DP_FW_HDR_SIZE	16
#define CDN_DP_IRAM_SIZE	0x10000
#define CDN_DP_FW_MAX		(CDN_DP_FW_HDR_SIZE + CDN_DP_IRAM_SIZE + \
				 CDN_DP_DMEM_SIZE)

/*
 * CONFIG_CDN_DP_FW_FIT_ADDR exists only when the FIT arm is selected, but the
 * IS_ENABLED(CONFIG_CDN_DP_FW_FIT) branches below must still COMPILE when it is
 * not: IS_ENABLED() discards the branch during optimisation, which is long
 * after the preprocessor and the parser have had to resolve every identifier
 * inside it. Aliasing it once, with a zero fallback, keeps those branches
 * compilable without spreading #ifdef through the function bodies -- and
 * without inventing a second, silently-wrong spelling of the load address,
 * since the alias is unreachable whenever the fallback is what it holds.
 */
#if IS_ENABLED(CONFIG_CDN_DP_FW_FIT)
#define CDN_DP_FW_FIT_ADDR	CONFIG_CDN_DP_FW_FIT_ADDR
#else
#define CDN_DP_FW_FIT_ADDR	0
#endif

/*
 * RK3399 general-GRF fields cdn-dp owns, from Linux's cdn-dp-core.c:43-48.
 *
 * SOC_CON26 is shared with the Type-C PHY driver, whose uphy_dp_sel is bit 3 of
 * this same register. That is safe only because RK3399 GRF registers are
 * write-masked -- the upper half-word selects which bits the lower half-word
 * applies to -- so neither driver has to read-modify-write and neither can
 * clobber the other's field.
 */
#define GRF_SOC_CON9		0x6224
#define  DP_SEL_VOP_LIT		BIT(12)
#define GRF_SOC_CON26		0x6268
#define  DPTX_HPD_SEL		(3 << 12)
#define  DPTX_HPD_DEL		(2 << 12)
#define  DPTX_HPD_SEL_MASK	(3 << 28)

/* DPTX opcodes for the host-capability handshake. */
#define DPTX_SET_HOST_CAPABILITIES	0x01
#define DPTX_WRITE_REGISTER		0x06

#define CDN_DP_AUX_SWAP_INVERSION_CONTROL	0x280c
#define  AUX_HOST_INVERT			3

#define CDN_DP_MAX_LINK_RATE	0x14	/* DP_LINK_BW_5_4 */
#define SCRAMBLER_EN		BIT(4)
#define VOLTAGE_LEVEL_2		2
#define PRE_EMPHASIS_LEVEL_3	3
#define PTS1			BIT(0)
#define PTS2			BIT(1)
#define PTS3			BIT(2)
#define PTS4			BIT(3)
#define FAST_LT_NOT_SUPPORT	0
#define LANE_MAPPING_NORMAL	0x1b
#define LANE_MAPPING_FLIPPED	0xe4
#define ENHANCED		1

/*
 * The DPTX video path's registers, from Linux's cdn-dp-reg.h. Every one of
 * these is written THROUGH the mailbox by cdn_dp_reg_write() -- they are inside
 * the window the running ucpu owns, so a writel() to any of them is a bus
 * error and not a shortcut. See cdn_dp_reg_write()'s comment.
 */
#define CDN_DP_BND_HSYNC2VSYNC		0x0b00	/* cdn-dp-reg.h:108 */
#define  VIF_BYPASS_INTERLACE		BIT(13)	/* cdn-dp-reg.h:356 */
#define CDN_DP_HSYNC2VSYNC_POL_CTRL	0x0b10	/* cdn-dp-reg.h:112 */
#define CDN_DP_FRAMER_TU		0x2208	/* cdn-dp-reg.h:135 */
#define  TU_CNT_RST_EN			BIT(15)	/* cdn-dp-reg.h:355 */
#define CDN_DP_FRAMER_PXL_REPR		0x220c	/* cdn-dp-reg.h:136 */
#define CDN_DP_FRAMER_SP		0x2210	/* cdn-dp-reg.h:137 */
#define  DP_FRAMER_SP_HSP		BIT(1)	/* cdn-dp-reg.h:361 */
#define  DP_FRAMER_SP_VSP		BIT(0)	/* cdn-dp-reg.h:362 */
#define CDN_DP_VC_TABLE(x)		(0x2218 + ((x) << 2))	/* :139 */
#define CDN_DP_VB_ID			0x2258	/* cdn-dp-reg.h:140 */
#define CDN_DP_FRONT_BACK_PORCH		0x2278	/* cdn-dp-reg.h:148 */
#define CDN_DP_BYTE_COUNT		0x227c	/* cdn-dp-reg.h:149 */
#define CDN_DP_MSA_HORIZONTAL_0		0x2280	/* cdn-dp-reg.h:152 */
#define CDN_DP_MSA_HORIZONTAL_1		0x2284
#define CDN_DP_MSA_VERTICAL_0		0x2288
#define CDN_DP_MSA_VERTICAL_1		0x228c
#define CDN_DP_MSA_MISC			0x2290
#define CDN_DP_STREAM_CONFIG		0x2294	/* cdn-dp-reg.h:157 */
#define CDN_DP_HORIZONTAL		0x22b0	/* cdn-dp-reg.h:164 */
#define CDN_DP_VERTICAL_0		0x22b4
#define CDN_DP_VERTICAL_1		0x22b8

/* enum vic_color_depth, cdn-dp-reg.h:441-447. */
#define BCS_6			0x01
#define BCS_8			0x02
#define BCS_10			0x04
#define BCS_12			0x08
#define BCS_16			0x10

/* enum vic_pxl_encoding_format, cdn-dp-core.h:33-39. Only RGB is produced. */
#define PXL_RGB			0x01
#define YCBCR_4_4_4		0x02
#define YCBCR_4_2_2		0x04
#define YCBCR_4_2_0		0x08
#define Y_ONLY			0x10

/* enum vic_bt_type, cdn-dp-core.h:449-452; only reached on the YUV paths. */
#define BT_601			0

/* The transfer-unit search's starting point, cdn-dp-reg.h:389. */
#define TU_SIZE			30

/*
 * Link rates, in kHz, for the bandwidth codes DPCD and the firmware both use.
 * Linux gets these from drm_dp_bw_code_to_link_rate(); U-Boot has no DRM DP
 * helper, so they are spelled out. Unlike the DRM helper this does NOT fall
 * back to 162000 for an unrecognised code -- see cdn_dp_bw_code_to_link_rate().
 */
#define DP_LINK_BW_1_62		0x06
#define DP_LINK_BW_2_7		0x0a
#define DP_LINK_BW_5_4		0x14
#define DP_LINK_BW_8_1		0x1e

/* DPCD addresses this driver reads, from DisplayPort 1.2. */
#define DP_DPCD_REV		0x000
#define DP_MAX_LINK_RATE	0x001
#define DP_MAX_LANE_COUNT	0x002
#define  DP_MAX_LANE_COUNT_MASK	0x1f
#define DP_SINK_COUNT		0x200

/*
 * The link configuration the FIRMWARE writes into the sink, and the status the
 * sink writes back. Read only to describe a training failure: the ucpu owns
 * this exchange, so a host write here would fight it. 0x100/0x101 are the
 * discriminator -- they are the first thing a full link-training sequence
 * writes. Finding them still zero after a timeout is therefore SUGGESTIVE of
 * training never having run, not proof of it: the firmware is not told to stop
 * before these are read, so "not written yet" reads identically. The claim was
 * stated as proof when this was written; that is WITHDRAWN, and the caveat is
 * spelled out at the branch that prints it.
 */
#define DP_LINK_BW_SET		0x100
#define DP_LANE_COUNT_SET	0x101
#define  DP_LANE_COUNT_MASK	0x0f
#define DP_TRAINING_PATTERN_SET	0x102
#define DP_TRAINING_LANE0_SET	0x103
#define DP_LANE0_1_STATUS	0x202
#define  DP_LANE_CR_DONE	0x01
#define DP_SINK_STATUS		0x205
#define DP_ADJUST_REQUEST_LANE0_1	0x206
#define DP_SET_POWER		0x600

/*
 * How long to wait for the sink to report itself present after the PHY comes
 * up. Linux's cdn_dp_check_sink_connection() budgets 5 s; a monitor waking from
 * standby is the case that needs it.
 */
#define SINK_CONNECT_TIMEOUT_MS	5000

/* Nominal SCLK_DP_CORE, per the node's own assigned-clock-rates. */
#define CDN_DP_CORE_HZ		100000000

struct rk_cdn_dp_priv {
	void __iomem *regs;

	struct clk_bulk clks;
	struct clk core_clk;		/* SCLK_DP_CORE, read back and checked */

	struct reset_ctl core_rst;
	struct reset_ctl dptx_rst;
	struct reset_ctl apb_rst;

	ulong core_rate;
	bool apb_live;			/* set by the liveness test below */

	u16 fw_version;			/* VER_H:VER_L, valid once fw_loaded */
	bool fw_loaded;

	/*
	 * rk_cdn_dp_fw_init() ran to COMPLETION -- not merely "the ucpu is
	 * executing", which is what fw_loaded means and where it is set.
	 *
	 * The two are different states and conflating them was a bug. fw_loaded
	 * is announced the moment the firmware answers the mailbox, because
	 * every step AFTER that point (ensure_phy, the HPD query, set_host_cap)
	 * needs a live mailbox to be attempted at all. So if any of those
	 * failed, rk_cdn_dp_fw_init() returned an error with fw_loaded still
	 * true, and the flag then said "initialised" to three callers that
	 * meant it differently:
	 *   - .read_edid skipped the re-init and used a controller whose host
	 *     capabilities had never been programmed;
	 *   - .enable passed its !fw_loaded guard and trained a link on it;
	 *   - .detect queried HPD with no PHY power and returned 0 -- "no
	 *     sink" -- where -ENOSYS was the honest answer. rk_display_init()
	 *     reads that 0 as evidence the connector is empty and suppresses
	 *     the fallback timing, so a monitor that IS plugged in reads as
	 *     absent.
	 */
	bool init_done;

	/*
	 * The DisplayPort half of the Type-C PHY the monitor is actually on.
	 * Index 1 of the node's `phys`, i.e. tcphy1 -- see cdn_dp_get_dp_phy().
	 */
	struct phy dp_phy;
	bool phy_on;

	void __iomem *grf_base;

	bool flip;			/* connector orientation, from the PHY */
	u8 lanes;			/* 4 for pin assignment C/E, 2 for D/F */

	/*
	 * What link training actually NEGOTIATED, read back from the firmware
	 * -- not what was requested. max_rate is in kHz. Both are zero until
	 * cdn_dp_train_link() has succeeded, and every consumer treats zero as
	 * "not trained" rather than as a default, because a TU calculation
	 * against an assumed rate produces a plausible number for a link that
	 * was never established.
	 */
	unsigned int max_rate;
	u8 max_lanes;

	/*
	 * The sink's EDID, cached by cdn_dp_read_edid_cached().
	 *
	 * edid_len is 0 until a CHECKSUM-VALID block 0 has been read, then 128,
	 * or 256 once a CTA-861 extension block has been read cleanly as well.
	 * It is the cache's only validity flag: there is no separate "valid"
	 * bool that could disagree with the byte count.
	 */
	u8 edid[EDID_EXT_SIZE];
	int edid_len;

	/*
	 * DPCD 0x000-0x002 (revision, max link rate, max lane count), cached on
	 * the same terms as the EDID and invalidated with it. .mode_valid is
	 * called once per candidate timing and needs these every time; re-reading
	 * them over AUX per mode would put four more transactions on the channel
	 * whose flakiness the EDID cache exists to route around.
	 */
	u8 sink_caps[3];
	bool sink_caps_valid;

	/*
	 * Video attributes for cdn_dp_config_video(). color_depth is bits per
	 * COMPONENT and comes from the sink's EDID; color_fmt is PXL_RGB
	 * because nothing in this driver produces YUV -- the other formats are
	 * carried in the arithmetic only so it stays diffable against Linux's.
	 */
	u8 color_depth;
	u8 color_fmt;

	/*
	 * What the transfer-unit search in cdn_dp_config_video() settled on,
	 * kept only so the `video` command can print the numbers it actually
	 * programmed rather than recomputing them and printing an agreement it
	 * did not observe. Zeroed at the top of every config_video() run: a TU
	 * left over from an earlier, differently-trained link printed beside a
	 * fresh rate reads as two measurements that agree.
	 */
	u8 tu_size;
	u32 valid_symbols;		/* whole symbols per TU */
	u32 valid_symbols_milli;	/* the fractional part, in thousandths */
};

/*
 * The three resets Linux's cdn_dp_clk_enable() cycles, as one block-level
 * reset. Factored out of probe because rk_cdn_dp_fw_init() needs exactly the
 * same thing: once firmware has run, the DPTX clock/reset ("CAR") registers at
 * 0x9xx stop accepting writes, and holding the ucpu in APB_XT_RESET does NOT
 * restore them -- measured, twice. Only re-asserting these three does.
 */
static void cdn_dp_block_reset(struct rk_cdn_dp_priv *priv)
{
	reset_assert(&priv->core_rst);
	reset_assert(&priv->dptx_rst);
	reset_assert(&priv->apb_rst);
	udelay(10);
	reset_deassert(&priv->core_rst);
	reset_deassert(&priv->dptx_rst);
	reset_deassert(&priv->apb_rst);
}

static int rk_cdn_dp_of_to_plat(struct udevice *dev)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	int ret;

	priv->regs = dev_read_addr_ptr(dev);
	if (!priv->regs)
		return -EINVAL;

	ret = clk_get_bulk(dev, &priv->clks);
	if (ret)
		return log_msg_ret("clks", ret);

	ret = clk_get_by_name(dev, "core-clk", &priv->core_clk);
	if (ret)
		return log_msg_ret("core-clk", ret);

	/*
	 * The three resets Linux's cdn_dp_clk_enable() cycles. "spdif" is
	 * resolvable too, and is inside this driver's softrst window, but is
	 * deliberately left alone: it gates the audio path only, and a
	 * bring-up should move exactly what the reference moves and no more.
	 */
	ret = reset_get_by_name(dev, "core", &priv->core_rst);
	if (ret)
		return log_msg_ret("core rst", ret);
	ret = reset_get_by_name(dev, "dptx", &priv->dptx_rst);
	if (ret)
		return log_msg_ret("dptx rst", ret);
	ret = reset_get_by_name(dev, "apb", &priv->apb_rst);
	if (ret)
		return log_msg_ret("apb rst", ret);

	/*
	 * The general GRF, which carries DPTX_HPD_SEL.
	 *
	 * syscon_get_first_range(), deliberately the same call the Type-C PHY
	 * driver uses, rather than a regmap via the node's rockchip,grf
	 * phandle. The two drivers write adjacent fields of the SAME register
	 * -- uphy_dp_sel is bit 3 of SOC_CON26, DPTX_HPD_SEL is bits 13:12 --
	 * so they must agree on the base and on the units of the offset, and
	 * the cheapest way to guarantee that is to resolve it identically.
	 * RK3399's write-masked registers are what make the sharing safe
	 * without either driver read-modify-writing.
	 *
	 * The clock, PCLK_VIO_GRF, is named "grf" in the node's clock-names
	 * and so is already covered by the clk_get_bulk above; unlike Linux,
	 * this driver does not enable it around each write.
	 */
	priv->grf_base = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);
	if (IS_ERR(priv->grf_base))
		return log_msg_ret("grf", PTR_ERR(priv->grf_base));

	return 0;
}

static int rk_cdn_dp_probe(struct udevice *dev)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	int ret;

	/*
	 * RK3399_PD_HDCP is named in the node. With POWER_DOMAIN=y and the
	 * rockchip-rk3399-power-domain provider, device_probe() has already
	 * powered it on before this runs; with POWER_DOMAIN=n device_probe()
	 * never reaches dev_power_domain_on() and this driver relies on the
	 * domain being on out of reset. Say which, because the symptom of an
	 * unpowered domain is a bus abort several steps later, in the middle
	 * of a firmware load.
	 *
	 * Turning POWER_DOMAIN on WITHOUT a provider is worse than leaving it
	 * off: the core would then try to resolve <&power RK3399_PD_HDCP>,
	 * find nothing, and fail this probe.
	 */
	if (!CONFIG_IS_ENABLED(POWER_DOMAIN))
		log_debug("cdn-dp: PD_HDCP unmanaged; relying on what TPL/SPL/BL31 left\n");

	/*
	 * SCLK_DP_CORE and SCLK_SPDIF_REC_DPTX are applied by the DM core from
	 * assigned-clock-rates before this runs. That path only survives
	 * because the RK3399 clock driver was taught these four ids; do not
	 * repeat that work here.
	 */
	ret = clk_enable_bulk(&priv->clks);
	if (ret)
		return log_msg_ret("clk enable", ret);

	cdn_dp_block_reset(priv);

	priv->core_rate = clk_get_rate(&priv->core_clk);
	if (!priv->core_rate) {
		log_err("cdn-dp: core clock rate is 0\n");
		clk_disable_bulk(&priv->clks);
		return -EINVAL;
	}

	/*
	 * Loud, because the failure it guards is silent. The firmware scales
	 * its AUX bit timing by the core clock and rejects only zero, so a
	 * wrong-by-20x rate never fails outright -- it just makes AUX flaky,
	 * which is indistinguishable from a bad cable. Check this number
	 * before believing any later result.
	 */
	if (priv->core_rate != CDN_DP_CORE_HZ)
		log_warning("cdn-dp: core clock is %lu Hz, expected %u Hz\n",
			    priv->core_rate, CDN_DP_CORE_HZ);

	log_debug("cdn-dp: regs %p, core clk %lu Hz\n", priv->regs,
		  priv->core_rate);

	return 0;
}

/*
 * APB liveness test -- and the RK3399_PD_HDCP power-domain test.
 *
 * This must run BEFORE any firmware work. Loading the firmware is tens of
 * thousands of consecutive writes into IRAM and DRAM; if the domain is down,
 * the abort lands somewhere inside that loop and reads like a firmware
 * problem. One word, at a known address, is the whole diagnosis.
 */
#define CDN_DP_TEST_OFFS	(CDN_DP_ADDR_DMEM + CDN_DP_DMEM_SIZE - 4)
#define CDN_DP_TEST_PAT		0xa5a5a5a5

static int rk_cdn_dp_apb_test(struct udevice *dev)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	void __iomem *addr = priv->regs + CDN_DP_TEST_OFFS;
	u32 got_a, got_b;

	/*
	 * This test is DESTRUCTIVE to a running firmware and always was: the
	 * APB_CTRL write below asserts APB_XT_RESET, which halts the ucpu, and
	 * the two pattern writes land on the last word of the DMEM image it was
	 * executing against. So it REFUSES while firmware is up, rather than
	 * running and then unwinding the state (the previous shape, which was
	 * honest but left an operator who typed `apbtest` after `fw` with a
	 * halted core and a DMEM word of test pattern to recover from).
	 *
	 * Refusing loses nothing: the question this test answers -- does the
	 * APB aperture decode, does DMEM store, is PD_HDCP powered -- has
	 * already been answered YES by the load itself, since KEEP_ALIVE going
	 * non-zero required tens of thousands of writes through that same
	 * aperture to land. The one caller that needs the test before a load,
	 * rk_cdn_dp_fw_init(), clears fw_loaded and block-resets before
	 * calling here, so it is never refused.
	 */
	if (priv->fw_loaded) {
		printf("cdn-dp: firmware is running -- refusing: the APB test halts the\n"
		       "        ucpu and overwrites a DMEM word. The load already proved\n"
		       "        the aperture (KEEP_ALIVE 0x%08x); run 'cdn_dp apbtest'\n"
		       "        before 'cdn_dp fw', or after 'reset'.\n",
		       readl(priv->regs + CDN_DP_KEEP_ALIVE));
		return -EBUSY;
	}

	/*
	 * DMEM is reachable from the APB only while APB_DRAM_PATH is set, and
	 * the ucpu is held in reset so nothing else drives that RAM -- exactly
	 * the state the reference establishes before its first DMEM write.
	 */
	writel(APB_IRAM_PATH | APB_DRAM_PATH | APB_XT_RESET,
	       priv->regs + CDN_DP_APB_CTRL);

	writel(CDN_DP_TEST_PAT, addr);
	got_a = readl(addr);

	writel(~CDN_DP_TEST_PAT, addr);
	got_b = readl(addr);

	/*
	 * The ucpu stays in reset. There is no firmware in it, so releasing it
	 * would run whatever the IRAM happens to contain.
	 *
	 * The address is the LAST word of DMEM, and each of those three
	 * properties earns its place: it is RAM, so a readback proves storage
	 * rather than a register that might legitimately read zero; it is at
	 * the far end of the window, so a pass also proves the aperture is
	 * fully decoded and not aliased down to a few bytes; and the firmware
	 * image's DRAM section covers it exactly, so the first real load wipes
	 * any residue and no test pattern can later be mistaken for firmware
	 * state. Neither pattern is 0 or 0xffffffff, because those two values
	 * are the diagnoses we need to be able to read.
	 */
	printf("cdn-dp: APB test at DMEM+0x%x\n", CDN_DP_DMEM_SIZE - 4);
	printf("  wrote %08x read %08x\n", CDN_DP_TEST_PAT, got_a);
	printf("  wrote %08x read %08x\n", (u32)~CDN_DP_TEST_PAT, got_b);

	if (got_a == CDN_DP_TEST_PAT && got_b == (u32)~CDN_DP_TEST_PAT) {
		priv->apb_live = true;
		printf("  PASS: APB alive, DMEM stores, PD_HDCP is powered\n");
		return 0;
	}

	if (got_a == 0xffffffff && got_b == 0xffffffff)
		printf("  FAIL: all-ones -- the bus returned a default, nothing stored.\n"
		       "        Rule out, in this order: the four clocks enabled and the\n"
		       "        core clock reading %u; the three resets deasserted;\n"
		       "        THEN blame PD_HDCP.\n", CDN_DP_CORE_HZ);
	else if (!got_a && !got_b)
		printf("  FAIL: all-zeros -- same class as all-ones, a constant from the\n"
		       "        bus rather than storage.\n");
	else
		printf("  FAIL: readback neither matches nor is constant -- partial\n"
		       "        decode, or the wrong base address.\n");

	return -EIO;
}

/*
 * Firmware.
 *
 * The blob is opaque. Everything below moves bytes to addresses the header
 * names and then speaks the host-side mailbox protocol; nothing inspects,
 * decodes or transforms the image, which is what LICENCE.rockchip requires.
 */

/**
 * cdn_dp_fw_check() - validate the container header, return the total size
 *
 * The size oracle as well as the validator: every caller needs to know how
 * many bytes the image occupies, and the header is the only thing that says
 * so. Returns the total byte count, or a negative errno.
 *
 * Each test below has a distinct failure it catches, and the order matters --
 * an LZMA node that silently decompressed to nothing, a truncated TFTP
 * transfer, and a load address pointing at unrelated DRAM all arrive here
 * looking similar.
 */
static int cdn_dp_fw_check(const void *fw, size_t avail)
{
	const struct cdn_dp_fw_header *hdr = fw;
	u32 size, hdr_size, iram, dram;

	if (avail < CDN_DP_FW_HDR_SIZE) {
		log_err("cdn-dp: firmware shorter than its header (%zu B)\n",
			avail);
		return -EINVAL;
	}

	size = le32_to_cpu(hdr->size_bytes);
	hdr_size = le32_to_cpu(hdr->header_size);
	iram = le32_to_cpu(hdr->iram_size);
	dram = le32_to_cpu(hdr->dram_size);

	if (hdr_size != CDN_DP_FW_HDR_SIZE) {
		log_err("cdn-dp: header_size %u, expected %u -- this does not look like dptx.bin\n",
			hdr_size, CDN_DP_FW_HDR_SIZE);
		return -EINVAL;
	}

	/*
	 * Bound BEFORE adding, so a hostile or garbage header cannot overflow
	 * the sum and pass a check it should fail.
	 */
	if (iram > CDN_DP_IRAM_SIZE || dram > CDN_DP_DMEM_SIZE) {
		log_err("cdn-dp: sections do not fit the controller (iram %u/%u, dram %u/%u)\n",
			iram, CDN_DP_IRAM_SIZE, dram, CDN_DP_DMEM_SIZE);
		return -EINVAL;
	}

	if (iram % 4 || dram % 4) {
		log_err("cdn-dp: section sizes not word multiples (iram %u, dram %u)\n",
			iram, dram);
		return -EINVAL;
	}

	if (size != hdr_size + iram + dram) {
		log_err("cdn-dp: size_bytes %u != %u + %u + %u\n",
			size, hdr_size, iram, dram);
		return -EINVAL;
	}

	if (size > avail) {
		log_err("cdn-dp: firmware claims %u B, only %zu B available\n",
			size, avail);
		return -EINVAL;
	}

	return size;
}

/**
 * cdn_dp_get_firmware() - point @fwp at the image, however this build ships it
 * @fwp:   set to the image on success
 * @freep: set to a pointer the CALLER must free(), or NULL if it must not
 *
 * There is deliberately no fallback between the two arms: a silent fall back
 * to a network fetch would hide a corrupt NOR image behind a working display.
 *
 * The two arms differ in who owns the memory, which is why @freep exists rather
 * than the caller guessing from the Kconfig: the FIT arm hands back a pointer
 * INTO the buffer SPL decompressed to, which must not be freed, while the
 * script arm allocates. Getting that backwards is either a leak of ~96 KB per
 * `cdn_dp fw` or a free() of an address the allocator never issued.
 */
static int cdn_dp_get_firmware(const void **fwp, void **freep)
{
	*freep = NULL;

	if (IS_ENABLED(CONFIG_CDN_DP_FW_FIT)) {
		const void *fw = (const void *)(ulong)CDN_DP_FW_FIT_ADDR;
		int ret;

		/*
		 * SPL already read this out of the FIT, verified its sha256
		 * over the COMPRESSED bytes, and LZMA-decompressed it to here,
		 * so there is nothing to load and nothing to check twice.
		 *
		 * The bound passed as `avail` is the largest image the
		 * controller could accept, not a measured buffer length --
		 * SPL leaves no size behind. cdn_dp_fw_check() then reads the
		 * real length out of the header. That is sound only because
		 * the header itself was covered by SPL's hash.
		 */
		ret = cdn_dp_fw_check(fw, CDN_DP_FW_MAX);
		if (ret < 0) {
			log_err("cdn-dp: no valid firmware at 0x%08x -- is 'dptx' named in fit,firmware?\n",
				CDN_DP_FW_FIT_ADDR);
			return ret;
		}

		*fwp = fw;
		return ret;
	}

#if IS_ENABLED(CONFIG_CDN_DP_FW_SCRIPT)
	{
		void *buf;
		size_t len = 0;
		int ret;

		/*
		 * request_firmware_into_buf_via_script() COPIES into a caller
		 * buffer -- it does not hand back the script's load address --
		 * so the buffer has to exist before the fetch and be sized for
		 * the largest image the controller could accept. malloc, not a
		 * static array: ~96 KB of permanently-resident BSS to serve a
		 * command that runs at most once per boot is the wrong trade,
		 * and this path is only reachable from U-Boot proper, where
		 * the heap is up.
		 */
		buf = malloc(CDN_DP_FW_MAX);
		if (!buf) {
			log_err("cdn-dp: no memory for a %u B firmware buffer\n",
				CDN_DP_FW_MAX);
			return -ENOMEM;
		}

		/*
		 * The script name is also the environment PREFIX: fw_loader.c
		 * builds "cdn_dp_fw_addr"/"cdn_dp_fw_size" from it, NOT the
		 * "fw_addr"/"fw_size" that include/fw_loader.h's example uses.
		 * A script written to the header comment returns -EINVAL here
		 * with the file transferred and nothing to show for it; the
		 * Kconfig help for CDN_DP_FW_SCRIPT carries the working form.
		 */
		ret = request_firmware_into_buf_via_script(buf, CDN_DP_FW_MAX,
							   "cdn_dp_fw", &len);
		if (ret) {
			log_err("cdn-dp: 'cdn_dp_fw' script did not deliver a firmware (%d)\n",
				ret);
			free(buf);
			return ret;
		}

		/*
		 * Checked against the length the SCRIPT actually delivered, not
		 * against the buffer size. This arm is the one that can be
		 * handed a truncated TFTP transfer, and a header validated
		 * against CDN_DP_FW_MAX would accept an image whose tail never
		 * arrived.
		 */
		ret = cdn_dp_fw_check(buf, len);
		if (ret < 0) {
			free(buf);
			return ret;
		}

		*fwp = buf;
		*freep = buf;
		return ret;
	}
#else
	return -ENOSYS;
#endif
}

static int cdn_dp_mailbox_write(struct rk_cdn_dp_priv *priv, u8 val)
{
	u32 full;
	int ret;

	ret = readl_poll_sleep_timeout(priv->regs + CDN_DP_MAILBOX_FULL, full,
				       !full, MAILBOX_POLL_US,
				       MAILBOX_TIMEOUT_US);
	if (ret)
		return ret;

	writel(val, priv->regs + CDN_DP_MAILBOX_WR_DATA);

	return 0;
}

static int cdn_dp_mailbox_read(struct rk_cdn_dp_priv *priv)
{
	u32 empty;
	int ret;

	ret = readl_poll_sleep_timeout(priv->regs + CDN_DP_MAILBOX_EMPTY, empty,
				       !empty, MAILBOX_POLL_US,
				       MAILBOX_TIMEOUT_US);
	if (ret)
		return ret;

	return readl(priv->regs + CDN_DP_MAILBOX_RD_DATA) & 0xff;
}

/**
 * cdn_dp_mailbox_send() - write one request: 4-byte header, then the payload
 *
 * The header is opcode, module id, then the payload size BIG-endian across two
 * bytes. That byte order is the one thing here worth stating explicitly: every
 * other multi-byte field this driver hands the firmware (DPCD addresses, EDID
 * block indices) is also big-endian, while the CPU and every register access in
 * this file are little-endian, so writing the size with a plain u16 store would
 * assemble a request the firmware silently mis-frames -- it would then read
 * `size` bytes of the NEXT request as this one's payload and the mailbox would
 * stay desynchronised for the rest of the session.
 *
 * @message may be NULL when @size is 0 (DPTX_HPD_STATE has no payload).
 */
static int cdn_dp_mailbox_send(struct rk_cdn_dp_priv *priv, u8 module_id,
			       u8 opcode, u16 size, const u8 *message)
{
	u8 header[4];
	int ret, i;

	header[0] = opcode;
	header[1] = module_id;
	header[2] = (size >> 8) & 0xff;
	header[3] = size & 0xff;

	for (i = 0; i < ARRAY_SIZE(header); i++) {
		ret = cdn_dp_mailbox_write(priv, header[i]);
		if (ret)
			return log_msg_ret("mbox hdr", ret);
	}

	for (i = 0; i < size; i++) {
		ret = cdn_dp_mailbox_write(priv, message[i]);
		if (ret)
			return log_msg_ret("mbox payload", ret);
	}

	return 0;
}

/**
 * cdn_dp_mailbox_validate_receive() - consume a response header, check it
 *
 * The drain loop on the mismatch path is not defensive tidiness, it is what
 * makes a failed transaction recoverable: the mailbox is a byte stream with no
 * framing of its own, so leaving an unread payload behind means the NEXT
 * request's caller reads those stale bytes as its own header and every
 * subsequent transaction is off by however many bytes were left. Breaking out
 * of the drain on error is deliberate too -- if reads are already timing out
 * the firmware has stopped answering and continuing to poll just burns the
 * full timeout once per remaining byte.
 */
static int cdn_dp_mailbox_validate_receive(struct rk_cdn_dp_priv *priv,
					   u8 module_id, u8 opcode,
					   u16 req_size)
{
	u8 header[4];
	u32 mbox_size;
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(header); i++) {
		ret = cdn_dp_mailbox_read(priv);
		if (ret < 0)
			return log_msg_ret("mbox rsp hdr", ret);

		header[i] = ret;
	}

	mbox_size = (header[2] << 8) | header[3];

	if (opcode != header[0] || module_id != header[1] ||
	    req_size != mbox_size) {
		/*
		 * log_err, not log_debug. This is the ONLY place -EINVAL comes
		 * from in the whole mailbox layer, so every opaque -22 a caller
		 * ever reports was decided here -- and with the detail compiled
		 * out, the three failures below are indistinguishable from each
		 * other at the console:
		 *
		 *   opcode/module mismatch  -- the byte stream is desynced, and
		 *                              we are reading someone else's
		 *                              reply as our header;
		 *   size mismatch, header   -- the firmware answered the right
		 *   otherwise correct          command with a shorter payload
		 *                              than the request implies, which
		 *                              is how a failed transaction
		 *                              downstream of the firmware (an
		 *                              EDID read that never reached the
		 *                              sink's EEPROM over DDC) appears
		 *                              from up here;
		 *   garbage in every field  -- desync, but from further back.
		 *
		 * Those want three different responses and the first two look
		 * identical without these numbers. Cost is a printf on a path
		 * that is already an error return.
		 */
		log_err("cdn-dp: mailbox reply mismatch: op %02x/%02x mod %02x/%02x size %u/%u\n",
			header[0], opcode, header[1], module_id, mbox_size,
			req_size);

		/*
		 * Print the payload when it is short enough to be a status
		 * rather than data. A reply whose opcode and module are RIGHT
		 * and whose size is a handful of bytes is not a framing error
		 * -- it is the firmware answering the command we asked for and
		 * telling us it failed. Measured: a DDC/EDID read to a sink
		 * that will not answer comes back as op 02, mod 01, size 1.
		 * Those bytes are the only thing the firmware ever says about
		 * why, and draining them silently discards the diagnosis.
		 */
		for (i = 0; i < mbox_size; i++) {
			ret = cdn_dp_mailbox_read(priv);
			if (ret < 0)
				break;
			if (mbox_size <= 8)
				log_err("cdn-dp:   payload[%d] = %02x\n", i,
					ret);
		}

		return -EINVAL;
	}

	return 0;
}

static int cdn_dp_mailbox_read_receive(struct rk_cdn_dp_priv *priv,
				       u8 *buff, u16 buff_size)
{
	int ret, i;

	for (i = 0; i < buff_size; i++) {
		ret = cdn_dp_mailbox_read(priv);
		if (ret < 0)
			return log_msg_ret("mbox body", ret);

		buff[i] = ret;
	}

	return 0;
}

/*
 * Take the firmware out of standby. Kept as a hand-rolled byte loop rather
 * than routed through cdn_dp_mailbox_send()/validate_receive() because that is
 * what the reference does: this is a GENERAL-module message and its reply is
 * five bytes, i.e. a 4-byte header plus one, which the validator would have to
 * be told is a payload of size 1. Matching the reference exactly matters more
 * here than sharing code -- this is the transaction that proves the mailbox
 * works at all, and it must not be the one carrying a novel framing assumption.
 */
static int cdn_dp_set_firmware_active(struct rk_cdn_dp_priv *priv, bool enable)
{
	u8 msg[5];
	int ret, i;

	msg[0] = GENERAL_MAIN_CONTROL;
	msg[1] = MB_MODULE_ID_GENERAL;
	msg[2] = 0;			/* size, high byte */
	msg[3] = 1;			/* size, low byte */
	msg[4] = enable ? FW_ACTIVE : FW_STANDBY;

	for (i = 0; i < ARRAY_SIZE(msg); i++) {
		ret = cdn_dp_mailbox_write(priv, msg[i]);
		if (ret) {
			log_err("cdn-dp: mailbox write %d stalled (%d)\n", i,
				ret);
			return ret;
		}
	}

	/*
	 * FIVE bytes, not four: a 4-byte response header plus the one payload
	 * byte this request's size field asked for. The mailbox is a byte
	 * stream with no framing of its own, so a byte left unread is not
	 * discarded -- the NEXT transaction reads it as the first byte of its
	 * own header and every subsequent one is off by one. Linux reads
	 * sizeof(msg) here for exactly this reason.
	 *
	 * This read 4 until it was caught in review. It was latent only while
	 * this was the last transaction the driver ever issued; the moment
	 * anything followed it -- event config, an HPD query, DPCD -- the
	 * orphan byte broke it.
	 */
	for (i = 0; i < ARRAY_SIZE(msg); i++) {
		ret = cdn_dp_mailbox_read(priv);
		if (ret < 0) {
			log_err("cdn-dp: mailbox read %d stalled (%d)\n", i,
				ret);
			return ret;
		}
	}

	return 0;
}

/**
 * cdn_dp_event_config() - arm the firmware's HPD and training event reporting
 *
 * Linux issues this immediately after set_firmware_active(), and so does
 * rk_cdn_dp_fw_init() below, because the firmware comes out of standby with
 * event reporting OFF: until this lands, SW_EVENTS0 never changes and a plug
 * or unplug is invisible. It is fire-and-forget -- DPTX_ENABLE_EVENT has no
 * reply to validate, so a failure here can only be a stalled write.
 *
 * The payload is five bytes of which only the first is set; the reference
 * memsets the rest and does not say what they are for. Neither do we -- they
 * are zeroed because that is the byte pattern known to work, not because their
 * meaning has been established.
 */
static int cdn_dp_event_config(struct rk_cdn_dp_priv *priv)
{
	u8 msg[5];

	memset(msg, 0, sizeof(msg));
	msg[0] = DPTX_EVENT_ENABLE_HPD | DPTX_EVENT_ENABLE_TRAINING;

	return cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX, DPTX_ENABLE_EVENT,
				   sizeof(msg), msg);
}

/**
 * cdn_dp_get_hpd_status() - ask the firmware whether a sink is plugged in
 *
 * Returns the status byte (non-zero = connected), or a negative errno.
 *
 * A zero-length request: the opcode alone is the whole message, which is why
 * cdn_dp_mailbox_send() has to tolerate a NULL payload pointer.
 */
static int cdn_dp_get_hpd_status(struct rk_cdn_dp_priv *priv)
{
	u8 status;
	int ret;

	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX, DPTX_HPD_STATE,
				  0, NULL);
	if (ret)
		return log_msg_ret("hpd send", ret);

	ret = cdn_dp_mailbox_validate_receive(priv, MB_MODULE_ID_DP_TX,
					      DPTX_HPD_STATE, sizeof(status));
	if (ret)
		return log_msg_ret("hpd hdr", ret);

	ret = cdn_dp_mailbox_read_receive(priv, &status, sizeof(status));
	if (ret)
		return log_msg_ret("hpd body", ret);

	return status;
}

/**
 * cdn_dp_dpcd_read() - read @len bytes of DPCD from @addr over AUX
 *
 * The reply echoes the request's 5-byte header (length then 24-bit address)
 * ahead of the data, so the expected size is sizeof(reg) + len and the echo has
 * to be consumed before the payload even though nothing here inspects it. It is
 * read into a scratch buffer rather than skipped with a loop of discards so the
 * shape stays identical to the reference.
 */
static int cdn_dp_dpcd_read(struct rk_cdn_dp_priv *priv, u32 addr, u8 *data,
			    u16 len)
{
	u8 msg[5], reg[5];
	int ret;

	msg[0] = (len >> 8) & 0xff;
	msg[1] = len & 0xff;
	msg[2] = (addr >> 16) & 0xff;
	msg[3] = (addr >> 8) & 0xff;
	msg[4] = addr & 0xff;

	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX, DPTX_READ_DPCD,
				  sizeof(msg), msg);
	if (ret)
		return log_msg_ret("dpcd rd send", ret);

	ret = cdn_dp_mailbox_validate_receive(priv, MB_MODULE_ID_DP_TX,
					      DPTX_READ_DPCD,
					      sizeof(reg) + len);
	if (ret)
		return log_msg_ret("dpcd rd hdr", ret);

	ret = cdn_dp_mailbox_read_receive(priv, reg, sizeof(reg));
	if (ret)
		return log_msg_ret("dpcd rd echo", ret);

	ret = cdn_dp_mailbox_read_receive(priv, data, len);
	if (ret)
		return log_msg_ret("dpcd rd data", ret);

	return 0;
}

/**
 * cdn_dp_dpcd_write() - write one DPCD byte at @addr over AUX
 *
 * The address check at the end is the only acknowledgement this protocol
 * offers: the firmware reports a mailbox-level success even when the AUX
 * transaction itself was NAKed or timed out, so a reply echoing a DIFFERENT
 * address is the one detectable sign that what came back is not an answer to
 * this request. It does not prove the sink accepted the byte; DPTX_GET_LAST_AUX_STAUS
 * would, and is not implemented here.
 */
static int cdn_dp_dpcd_write(struct rk_cdn_dp_priv *priv, u32 addr, u8 value)
{
	u8 msg[6], reg[5];
	int ret;

	msg[0] = 0;			/* length, high byte */
	msg[1] = 1;			/* length: always a single byte */
	msg[2] = (addr >> 16) & 0xff;
	msg[3] = (addr >> 8) & 0xff;
	msg[4] = addr & 0xff;
	msg[5] = value;

	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX, DPTX_WRITE_DPCD,
				  sizeof(msg), msg);
	if (ret)
		return log_msg_ret("dpcd wr send", ret);

	ret = cdn_dp_mailbox_validate_receive(priv, MB_MODULE_ID_DP_TX,
					      DPTX_WRITE_DPCD, sizeof(reg));
	if (ret)
		return log_msg_ret("dpcd wr hdr", ret);

	ret = cdn_dp_mailbox_read_receive(priv, reg, sizeof(reg));
	if (ret)
		return log_msg_ret("dpcd wr echo", ret);

	if (addr != ((u32)reg[2] << 16 | (u32)reg[3] << 8 | reg[4])) {
		log_err("cdn-dp: dpcd write echoed addr %06x, sent %06x\n",
			(u32)reg[2] << 16 | (u32)reg[3] << 8 | reg[4], addr);
		return -EINVAL;
	}

	return 0;
}

/**
 * cdn_dp_get_edid_block() - read one 128-byte EDID block over AUX
 *
 * @block is the EDID block number; the firmware is addressed in 128-byte PAIRS,
 * hence the block/2 (which pair) and block%2 (which half) split -- one AUX
 * segment read returns 256 bytes and the firmware hands back the requested half.
 *
 * Retried four times, as in the reference. A first read on a freshly plugged
 * sink genuinely does fail: the DDC/AUX channel comes up before the sink's
 * EDID EEPROM is answering, and a single attempt turns that race into "no
 * monitor". Every retry re-sends from scratch, which is what makes it safe --
 * validate_receive() has drained whatever the failed attempt left behind.
 *
 * One deliberate divergence from Linux: the reference falls out of the loop
 * with ret == 0 when the last attempt READ cleanly but the echoed length/pair
 * did not match, and so reports success for a block it cannot vouch for. Here
 * that is -EIO. A wrong block silently accepted becomes a wrong mode, which is
 * exactly the failure this bring-up is trying not to ship.
 */
static int cdn_dp_get_edid_block(struct rk_cdn_dp_priv *priv, u8 *edid,
				 uint block, size_t length)
{
	u8 msg[2], reg[2];
	int ret = -EIO;
	int i;

	for (i = 0; i < 4; i++) {
		/*
		 * Back off between attempts, which the reference does not do.
		 *
		 * Measured on this board: a DDC read that lands too soon after
		 * a previous one fails with the firmware's one-byte 0xff
		 * status, and the four retries -- issued back-to-back with no
		 * gap -- all fall inside the same window, so the whole call
		 * fails. A single unrelated DPCD read between two EDID reads is
		 * enough to make the second succeed, which is what pointed at a
		 * recovery interval rather than at framing.
		 *
		 * Skipped on the first attempt so the common case pays nothing.
		 */
		if (i)
			mdelay(EDID_RETRY_DELAY_MS);

		msg[0] = block / 2;
		msg[1] = block % 2;

		ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX,
					  DPTX_GET_EDID, sizeof(msg), msg);
		if (ret)
			continue;

		ret = cdn_dp_mailbox_validate_receive(priv, MB_MODULE_ID_DP_TX,
						      DPTX_GET_EDID,
						      sizeof(reg) + length);
		if (ret)
			continue;

		ret = cdn_dp_mailbox_read_receive(priv, reg, sizeof(reg));
		if (ret)
			continue;

		ret = cdn_dp_mailbox_read_receive(priv, edid, length);
		if (ret)
			continue;

		if (reg[0] == length && reg[1] == block / 2)
			return 0;

		log_debug("cdn-dp: edid block %u echo len %u pair %u, wanted %zu/%u\n",
			  block, reg[0], reg[1], length, block / 2);
		ret = -EIO;
	}

	log_err("cdn-dp: EDID block %u failed after 4 tries (%d)\n", block, ret);

	return ret;
}

/**
 * cdn_dp_edid_block_ok() - is this 128-byte block a whole, intact EDID block?
 *
 * The mod-256 checksum is the only end-to-end integrity check EDID offers, and
 * it is the one that matters here: a half-completed AUX transfer hands back
 * bytes that still carry the header magic, so a block that is merely plausible
 * decodes into a plausible -- and wrong -- mode. Nothing is cached, and nothing
 * is turned into a timing, that does not pass this.
 */
static bool cdn_dp_edid_block_ok(const u8 *block)
{
	u8 sum = 0;
	int i;

	for (i = 0; i < EDID_BLOCK_SIZE; i++)
		sum += block[i];

	return sum == 0;
}

/**
 * cdn_dp_read_edid_cached() - read the EDID once, serve every later request
 *                             from the cache
 * @force: read the sink again even when a cached copy exists
 *
 * Measured on this board: the first EDID read after the link has settled is
 * reliable and repeats of it are not -- cdn_dp_get_edid_block()'s own comment
 * records a completed 130-byte transfer leaving the DDC engine unavailable for
 * longer than all four of its retries. Three callers want the same bytes (the
 * display uclass via .read_edid, `cdn_dp edid`, and cdn_dp_bringup()), so the
 * sink is read once and the bytes are kept. That makes the repeat-read
 * instability irrelevant to bring-up instead of something bring-up has to
 * survive.
 *
 * Only a checksum-valid block is ever stored, and the read lands in a local
 * buffer first: caching a failed read would convert a transient AUX failure
 * into a permanently wrong mode, which is worse than the flakiness it papers
 * over, and a partial second read must not be able to corrupt a good first one.
 *
 * @force keeps the cache from hiding a regression in the read path itself --
 * `cdn_dp edid -f` still exercises the hardware every time.
 *
 * Returns the number of cached bytes (128, or 256 when the sink has an
 * extension block that also read cleanly), or a negative errno.
 */
static int cdn_dp_read_edid_cached(struct rk_cdn_dp_priv *priv, bool force)
{
	static const u8 magic[8] = { 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0 };
	u8 buf[EDID_EXT_SIZE];
	int ret;

	if (priv->edid_len && !force)
		return priv->edid_len;

	/*
	 * Invalidated BEFORE the read, not after a failure. The reason to force
	 * a re-read is that the sink may have changed, so a forced read that
	 * fails must not leave the previous monitor's bytes behind to be served
	 * as if they described the current one. The DPCD capabilities go with
	 * them -- they describe the same sink.
	 */
	priv->edid_len = 0;
	priv->sink_caps_valid = false;

	ret = cdn_dp_get_edid_block(priv, buf, 0, EDID_BLOCK_SIZE);
	if (ret)
		return ret;

	if (memcmp(buf, magic, sizeof(magic))) {
		log_err("cdn-dp: EDID block 0 has no header magic -- not cached\n");
		return -EIO;
	}

	if (!cdn_dp_edid_block_ok(buf)) {
		log_err("cdn-dp: EDID block 0 checksum BAD -- not cached\n");
		return -EIO;
	}

	priv->edid_len = EDID_BLOCK_SIZE;

	/*
	 * The extension block is not decoration.
	 * edid_get_timing_validate() only walks the CTA-861 detailed timings
	 * when it is handed >= EDID_EXT_SIZE bytes, and those timings are every
	 * fallback mode the sink has. Stopping at 128 bytes leaves the base
	 * block's preferred timing as the ONLY candidate, so a preferred mode
	 * that .mode_valid or the VOP's max_output check rejects has nothing to
	 * fall back to and the output is lost rather than downgraded.
	 *
	 * A bad extension is not fatal: 128 good bytes are still a usable EDID.
	 * It is reported and the base block is kept.
	 */
	if (buf[EDID_EXT_COUNT_OFF]) {
		ret = cdn_dp_get_edid_block(priv, buf + EDID_BLOCK_SIZE, 1,
					    EDID_BLOCK_SIZE);
		if (ret)
			log_err("cdn-dp: EDID extension read failed (%d) -- base block only\n",
				ret);
		else if (!cdn_dp_edid_block_ok(buf + EDID_BLOCK_SIZE))
			log_err("cdn-dp: EDID extension checksum BAD -- base block only\n");
		else
			priv->edid_len = EDID_EXT_SIZE;
	}

	memcpy(priv->edid, buf, priv->edid_len);

	return priv->edid_len;
}

/**
 * cdn_dp_edid_bpc() - bits per COMPONENT the cached EDID claims
 *
 * Linux's mode_valid reads this from drm_display_info.bpc and maps everything
 * that is not 6 or 10 onto 8. The same mapping is done here, from the cached
 * bytes, because .mode_valid runs INSIDE edid_get_timing_validate() and so
 * cannot be handed the panel_bits_per_colour that same call is still computing.
 *
 * That mapping also absorbs the "colour bit depth is undefined" encoding, which
 * the EDID 1.4 formula turns into 4 -- not a DP pixel depth at all, and the
 * value that fell straight through cdn_dp_config_video()'s switch to -EINVAL
 * before this became the single derivation. Here it lands in the default arm
 * and becomes 8, the DP default.
 *
 * This is the ONLY place a colour depth is derived from the EDID in this
 * driver. `cdn_dp video` calls it too rather than using edid_get_timing()'s
 * out-parameter, because two derivations would let the bring-up command print
 * a depth -- and a bandwidth figure computed from it -- that the .enable path
 * will not program: edid_get_timing() passes 12 and 16 straight through, this
 * maps them to 8, and the resulting bandwidth numbers differ by 1.5x.
 */
static u8 cdn_dp_edid_bpc(struct rk_cdn_dp_priv *priv)
{
	int depth;

	/* EDID 1.3 and earlier reserve the field, so it says nothing. */
	if (priv->edid_len < EDID_BLOCK_SIZE ||
	    priv->edid[EDID_VERSION_OFF] != 1 ||
	    priv->edid[EDID_REVISION_OFF] < 4)
		return 8;

	/*
	 * Bit 7 first, as drm_edid.c's drm_parse_display_props() does. Bits 6:4
	 * are the bit-depth field ONLY for a digital input; for an analog one
	 * the same bits are the signal level standard, sync configuration and
	 * serration bits, and decoding those as a depth yields a number with no
	 * relation to anything. A DP sink is always digital, so this never
	 * fires here -- it is checked anyway so the default arm below stays the
	 * answer for "undefined", not the catch-all hiding a misread field.
	 */
	if (!(priv->edid[EDID_INPUT_DEF_OFF] & 0x80))
		return 8;

	depth = ((priv->edid[EDID_INPUT_DEF_OFF] & 0x70) >> 3) + 4;

	switch (depth) {
	case 6:
	case 10:
		return depth;
	default:
		return 8;
	}
}

/**
 * cdn_dp_get_sink_caps() - DPCD 0x000-0x002, cached with the EDID
 * @caps: three bytes: revision, max link rate code, max lane count
 */
static int cdn_dp_get_sink_caps(struct rk_cdn_dp_priv *priv, u8 *caps)
{
	int ret;

	if (!priv->sink_caps_valid) {
		ret = cdn_dp_dpcd_read(priv, DP_DPCD_REV, priv->sink_caps,
				       sizeof(priv->sink_caps));
		if (ret)
			return ret;
		priv->sink_caps_valid = true;
	}

	memcpy(caps, priv->sink_caps, sizeof(priv->sink_caps));

	return 0;
}

/*
 * Tell the firmware what the core clock actually is, in MHz.
 *
 * Not cosmetic: the firmware scales its AUX bit timing off this, and it is the
 * software half of the same hazard the probe already warns about. A wrong
 * value here does not fail -- it makes AUX flaky in a way indistinguishable
 * from a bad cable.
 */
static void cdn_dp_set_fw_clk(struct rk_cdn_dp_priv *priv)
{
	writel(priv->core_rate / 1000000, priv->regs + CDN_DP_SW_CLK_H);
}

/*
 * Written over the APB before any firmware runs, which is the only order that
 * can work: there is no mailbox until the ucpu is executing.
 *
 * The caller MUST have the ucpu held in reset. These are the DPTX block's own
 * clock/reset registers, and rewriting them underneath a running ucpu earns an
 * SError on the first posted write -- measured, from running `cdn_dp fw` twice.
 *
 * SError, so ASYNCHRONOUS (ESR EC 0x2f): the reported PC is wherever execution
 * had got to when the write was NACKed, not the store itself. Do not read that
 * PC as the faulting instruction -- it cost a cycle here to learn that.
 */
static void cdn_dp_clock_reset(struct rk_cdn_dp_priv *priv)
{
	u32 val;

	val = DPTX_FRMR_DATA_CLK_RSTN_EN | DPTX_FRMR_DATA_CLK_EN |
	      DPTX_PHY_DATA_RSTN_EN | DPTX_PHY_DATA_CLK_EN |
	      DPTX_PHY_CHAR_RSTN_EN | DPTX_PHY_CHAR_CLK_EN |
	      SOURCE_AUX_SYS_CLK_RSTN_EN | SOURCE_AUX_SYS_CLK_EN |
	      DPTX_SYS_CLK_RSTN_EN | DPTX_SYS_CLK_EN |
	      CFG_DPTX_VIF_CLK_RSTN_EN | CFG_DPTX_VIF_CLK_EN;
	writel(val, priv->regs + CDN_DP_SOURCE_DPTX_CAR);

	val = SOURCE_PHY_RSTN_EN | SOURCE_PHY_CLK_EN;
	writel(val, priv->regs + CDN_DP_SOURCE_PHY_CAR);

	val = SOURCE_PKT_SYS_RSTN_EN | SOURCE_PKT_SYS_CLK_EN |
	      SOURCE_PKT_DATA_RSTN_EN | SOURCE_PKT_DATA_CLK_EN;
	writel(val, priv->regs + CDN_DP_SOURCE_PKT_CAR);

	val = SPDIF_CDR_CLK_RSTN_EN | SPDIF_CDR_CLK_EN |
	      SOURCE_AIF_SYS_RSTN_EN | SOURCE_AIF_SYS_CLK_EN |
	      SOURCE_AIF_CLK_RSTN_EN | SOURCE_AIF_CLK_EN;
	writel(val, priv->regs + CDN_DP_SOURCE_AIF_CAR);

	val = SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN | SOURCE_CIPHER_SYS_CLK_EN |
	      SOURCE_CIPHER_CHAR_CLK_RSTN_EN | SOURCE_CIPHER_CHAR_CLK_EN;
	writel(val, priv->regs + CDN_DP_SOURCE_CIPHER_CAR);

	val = SOURCE_CRYPTO_SYS_CLK_RSTN_EN | SOURCE_CRYPTO_SYS_CLK_EN;
	writel(val, priv->regs + CDN_DP_SOURCE_CRYPTO_CAR);

	/* Unmask the mailbox and PM interrupts. */
	writel(0, priv->regs + CDN_DP_APB_INT_MASK);
}

static int cdn_dp_load_firmware(struct rk_cdn_dp_priv *priv, const void *fw)
{
	const struct cdn_dp_fw_header *hdr = fw;
	u32 iram = le32_to_cpu(hdr->iram_size);
	u32 dram = le32_to_cpu(hdr->dram_size);
	const u32 *i_mem = fw + CDN_DP_FW_HDR_SIZE;
	const u32 *d_mem = fw + CDN_DP_FW_HDR_SIZE + iram;
	u32 reg;
	int i, ret;

	/* Hold the ucpu in reset and open the APB's path to both RAMs. */
	writel(APB_IRAM_PATH | APB_DRAM_PATH | APB_XT_RESET,
	       priv->regs + CDN_DP_APB_CTRL);

	/*
	 * get_unaligned_le32(), not *i_mem++. The source is wherever SPL
	 * decompressed to or wherever a TFTP transfer landed, and neither
	 * guarantees 4-byte alignment; an unaligned u32 load traps on arm64.
	 */
	for (i = 0; i < iram; i += 4)
		writel(get_unaligned_le32((const u8 *)i_mem + i),
		       priv->regs + CDN_DP_ADDR_IMEM + i);

	for (i = 0; i < dram; i += 4)
		writel(get_unaligned_le32((const u8 *)d_mem + i),
		       priv->regs + CDN_DP_ADDR_DMEM + i);

	/* Release the ucpu: it starts executing what we just wrote. */
	writel(0, priv->regs + CDN_DP_APB_CTRL);

	/*
	 * KEEP_ALIVE going non-zero is the firmware's first sign of life, and
	 * the only one available before the mailbox works.
	 */
	ret = readl_poll_sleep_timeout(priv->regs + CDN_DP_KEEP_ALIVE, reg, reg,
				       KEEP_ALIVE_POLL_US,
				       KEEP_ALIVE_TIMEOUT_US);
	if (ret) {
		log_err("cdn-dp: firmware loaded but KEEP_ALIVE stayed 0 -- ucpu is not running\n");
		return ret;
	}

	priv->fw_version = (readl(priv->regs + CDN_DP_VER_H) & 0xff) << 8;
	priv->fw_version |= readl(priv->regs + CDN_DP_VER_L) & 0xff;

	return 0;
}

/**
 * cdn_dp_reg_write() - write a DPTX register THROUGH the firmware
 *
 * Once the ucpu is running it owns the DPTX register file, and the host reaches
 * it only by asking. This is not an optimisation or a convenience wrapper: the
 * APB path to those addresses is closed, so the obvious writel() is not a
 * faster equivalent, it is a bus error.
 */
static int cdn_dp_reg_write(struct rk_cdn_dp_priv *priv, u16 addr, u32 val)
{
	u8 msg[6];

	msg[0] = (addr >> 8) & 0xff;
	msg[1] = addr & 0xff;
	msg[2] = (val >> 24) & 0xff;
	msg[3] = (val >> 16) & 0xff;
	msg[4] = (val >> 8) & 0xff;
	msg[5] = val & 0xff;

	return cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX,
				   DPTX_WRITE_REGISTER, sizeof(msg), msg);
}

/**
 * cdn_dp_set_host_cap() - tell the firmware what this source can do
 *
 * The lane mapping byte is the one that matters and the one that is easy to get
 * wrong: it encodes the connector orientation, so a flipped plug described as
 * normal inverts the lane and AUX polarity at the SBU pins and presents exactly
 * as a bad cable. It comes from the PHY rather than from a guess for that
 * reason -- the PHY is what actually muxed the lanes.
 */
static int cdn_dp_set_host_cap(struct rk_cdn_dp_priv *priv)
{
	u8 msg[8];
	int ret;

	msg[0] = CDN_DP_MAX_LINK_RATE;
	msg[1] = priv->lanes | SCRAMBLER_EN;
	msg[2] = VOLTAGE_LEVEL_2;
	msg[3] = PRE_EMPHASIS_LEVEL_3;
	msg[4] = PTS1 | PTS2 | PTS3 | PTS4;
	msg[5] = FAST_LT_NOT_SUPPORT;
	msg[6] = priv->flip ? LANE_MAPPING_FLIPPED : LANE_MAPPING_NORMAL;
	msg[7] = ENHANCED;

	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX,
				  DPTX_SET_HOST_CAPABILITIES, sizeof(msg), msg);
	if (ret)
		return ret;

	/*
	 * Through the MAILBOX, not writel(). The DPTX register file at 0x2800+
	 * belongs to the running firmware and is not reachable over the APB: a
	 * direct store there is NACKed and comes back as an SError. Measured --
	 * this function's first version used writel() and aborted every time,
	 * on a run where the preceding HPD query over this same mailbox had
	 * just succeeded, which is what localised it to the store rather than
	 * to the protocol.
	 */
	return cdn_dp_reg_write(priv, CDN_DP_AUX_SWAP_INVERSION_CONTROL,
				AUX_HOST_INVERT);
}

/**
 * cdn_dp_reg_write_bit() - write a BIT FIELD of a DPTX register via the mailbox
 *
 * DPTX_WRITE_FIELD (0x08) exists because the host cannot read-modify-write a
 * register it is not allowed to read: the ucpu owns the register file, so the
 * only way to change some bits and leave the rest is to make the firmware do
 * it. The payload is address, start bit, bit count, then the value -- Linux's
 * cdn_dp_reg_write_bit(), cdn-dp-reg.c:198-214.
 */
static int cdn_dp_reg_write_bit(struct rk_cdn_dp_priv *priv, u16 addr,
				u8 start_bit, u8 bits_no, u32 val)
{
	u8 field[8];

	field[0] = (addr >> 8) & 0xff;
	field[1] = addr & 0xff;
	field[2] = start_bit;
	field[3] = bits_no;
	field[4] = (val >> 24) & 0xff;
	field[5] = (val >> 16) & 0xff;
	field[6] = (val >> 8) & 0xff;
	field[7] = val & 0xff;

	return cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX, DPTX_WRITE_FIELD,
				   sizeof(field), field);
}

/**
 * cdn_dp_bw_code_to_link_rate() - DPCD bandwidth code to a rate in kHz
 *
 * Linux calls drm_dp_bw_code_to_link_rate(), which returns 162000 for anything
 * it does not recognise. This returns 0 instead, and the difference matters
 * here: the caller is decoding a byte the FIRMWARE produced to describe a link
 * it just trained, so an unrecognised code means the reply was not what we
 * think it was. Substituting the slowest legal rate would turn that into a
 * plausible number, a TU calculation built on it, and a report of a trained
 * 1.62 Gbps link that nothing observed.
 */
static unsigned int cdn_dp_bw_code_to_link_rate(u8 bw_code)
{
	switch (bw_code) {
	case DP_LINK_BW_1_62:
		return 162000;
	case DP_LINK_BW_2_7:
		return 270000;
	case DP_LINK_BW_5_4:
		return 540000;
	case DP_LINK_BW_8_1:
		return 810000;
	default:
		return 0;
	}
}

/**
 * cdn_dp_training_start() - run link training and wait for the EQ phase
 * @event_bits: OR of every status byte seen, so a failure can be described
 * @polls: how many DPTX_READ_EVENT replies were actually READ
 * @run_delivered: whether the DPTX_TRAINING_CONTROL / LINK_TRAINING_RUN send
 *                 itself succeeded
 * @poll_aborted: whether the poll loop left through one of its three mailbox
 *                failures rather than by finishing or timing out
 *
 * @polls is what makes @event_bits interpretable, and without it the two were
 * conflated. @event_bits == 0 means "no start bit was reported" only if the
 * firmware was ever asked and kept being asked for the whole budget.
 *
 * WITHDRAWN: the previous wording of this paragraph said the three
 * mailbox-failure exits below ("event send", "event hdr", "event body") leave
 * @polls == 0, and cdn_dp_training_diag()'s comment said they "all leave
 * @event_bits at the zero it was initialised to". Both are true only on the
 * FIRST iteration. Those exits are inside the poll loop, so on iteration 2 and
 * later they leave @polls >= 1, @run_delivered true, and @event_bits at
 * whatever the earlier polls happened to OR in -- which can be 0. That shape
 * is indistinguishable from a full 500 ms of polling that saw no start bit,
 * and the diagnostic printed the exoneration reserved for that case: "training
 * never began, so lane drive levels and cable quality are not the fault",
 * after a single 20 ms sample. @poll_aborted is the discriminator; it says the
 * window was cut short, whatever @polls reached.
 *
 * @run_delivered exists because there is a FOURTH exit with @polls == 0, and
 * it is the one BEFORE LINK_TRAINING_RUN reaches the ucpu: the "train run" send
 * below. Describing it as "LINK_TRAINING_RUN was delivered but the first
 * DPTX_READ_EVENT round trip failed" -- which is what the @polls == 0 arm of
 * cdn_dp_training_diag() said until this flag existed -- asserts a delivery
 * that provably did not happen, and it is the same defect the diagnostic was
 * rewritten to remove, moved one state over. "Never sent" and "sent, no reply"
 * point at different halves of the mailbox, so the log has to say which.
 *
 * The host does NOT drive link training. It issues one DPTX_TRAINING_CONTROL
 * with LINK_TRAINING_RUN and then only watches: the clock-recovery loop, the
 * equalisation loop, the voltage-swing and pre-emphasis adjustments and every
 * DPCD write they need (DP_LINK_BW_SET, DP_LANE_COUNT_SET, DP_TRAINING_PATTERN_SET,
 * DP_TRAINING_LANE0_SET...) all happen inside the firmware, bounded by the
 * limits cdn_dp_set_host_cap() already handed it. That is why there is no DPCD
 * link configuration anywhere in this driver, and why writing any by hand here
 * would fight the ucpu for the same registers.
 *
 * The progress report is byte 1 of a DPTX_READ_EVENT reply. Byte 0's meaning is
 * NOT established -- Linux reads both and tests only byte 1, and the reference
 * does not say what byte 0 carries, so this does not either.
 */
static int cdn_dp_training_start(struct rk_cdn_dp_priv *priv, u8 *event_bits,
				 unsigned int *polls, bool *run_delivered,
				 bool *poll_aborted)
{
	u8 msg = LINK_TRAINING_RUN;
	u8 event[2];
	ulong start;
	int ret;

	*event_bits = 0;
	*polls = 0;
	*run_delivered = false;
	*poll_aborted = false;

	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX,
				  DPTX_TRAINING_CONTROL, sizeof(msg), &msg);
	if (ret)
		return log_msg_ret("train run", ret);

	/*
	 * Set AFTER the send returns 0, so it records a request the mailbox
	 * accepted -- not one this function intended to make.
	 */
	*run_delivered = true;

	start = get_timer(0);
	while (get_timer(start) < LINK_TRAINING_TIMEOUT_MS) {
		/*
		 * Sleep BEFORE the first poll, as the reference does. Training
		 * takes milliseconds at best, and an immediate DPTX_READ_EVENT
		 * costs a full mailbox round trip to be told nothing has
		 * happened yet -- on a mailbox the firmware is also using.
		 */
		mdelay(LINK_TRAINING_RETRY_MS);

		/*
		 * @poll_aborted, not @polls == 0, is what marks these three
		 * exits. They are inside the loop, so on the second and later
		 * iterations @polls is already >= 1 and @run_delivered is
		 * true: the "nobody asked" state they represent is NOT
		 * distinguishable from a completed 500 ms budget by the two
		 * counters alone. cdn_dp_training_diag()'s zero-@event_bits
		 * arm read exactly that shape as "training never began, so
		 * lane drive levels and cable quality are not the fault" --
		 * an exoneration issued after one 20 ms sample of a 500 ms
		 * window.
		 */
		ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX,
					  DPTX_READ_EVENT, 0, NULL);
		if (ret) {
			*poll_aborted = true;
			return log_msg_ret("event send", ret);
		}

		ret = cdn_dp_mailbox_validate_receive(priv, MB_MODULE_ID_DP_TX,
						      DPTX_READ_EVENT,
						      sizeof(event));
		if (ret) {
			*poll_aborted = true;
			return log_msg_ret("event hdr", ret);
		}

		ret = cdn_dp_mailbox_read_receive(priv, event, sizeof(event));
		if (ret) {
			*poll_aborted = true;
			return log_msg_ret("event body", ret);
		}

		/*
		 * A complete round trip. Counted BEFORE the status byte is
		 * examined, because what it records is that the firmware was
		 * asked and answered -- not what it answered.
		 */
		(*polls)++;

		/*
		 * Accumulated, not latched to the last read. The status byte
		 * describes the phase the firmware is IN, so a CLK_RECOVERY_FAILED
		 * raised on one poll can be gone by the next -- and the whole
		 * value of the failure path is being able to say WHICH phase
		 * did not complete.
		 */
		*event_bits |= event[1];

		if (event[1] & EQ_PHASE_FINISHED)
			return 0;
	}

	/*
	 * Deliberately NOT an early exit on CLK_RECOVERY_FAILED / EQ_PHASE_FAILED.
	 * The firmware retries internally -- that is what the 500 ms budget is
	 * for -- so a failure bit seen mid-run is a phase that has to be redone,
	 * not a verdict. Bailing on it would report a failure the hardware was
	 * still in the middle of recovering from.
	 */
	return -ETIMEDOUT;
}

/**
 * cdn_dp_get_training_status() - read back the rate and lane count negotiated
 *
 * DPTX_READ_LINK_STAT returns TEN bytes and Linux consumes exactly two of them:
 * status[0] is the DPCD bandwidth code and status[1] the lane count. What the
 * other eight carry is not established by the reference and is not guessed at
 * here -- they are read because the reply's length is part of its framing, and
 * a validator told to expect fewer would reject every reply and leave the rest
 * in the mailbox.
 */
static int cdn_dp_get_training_status(struct rk_cdn_dp_priv *priv)
{
	u8 status[10];
	unsigned int rate;
	int ret;

	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX,
				  DPTX_READ_LINK_STAT, 0, NULL);
	if (ret)
		return log_msg_ret("link stat send", ret);

	ret = cdn_dp_mailbox_validate_receive(priv, MB_MODULE_ID_DP_TX,
					      DPTX_READ_LINK_STAT,
					      sizeof(status));
	if (ret)
		return log_msg_ret("link stat hdr", ret);

	ret = cdn_dp_mailbox_read_receive(priv, status, sizeof(status));
	if (ret)
		return log_msg_ret("link stat body", ret);

	rate = cdn_dp_bw_code_to_link_rate(status[0]);
	if (!rate) {
		log_err("cdn-dp: link status reports bandwidth code 0x%02x, which is not a DP rate\n",
			status[0]);
		return -EPROTO;
	}

	/*
	 * A lane count outside 1..4 is not a slower link, it is a reply that
	 * does not mean what we think it means -- and it would go on to divide
	 * the TU calculation below. Zero in particular reads as "trained with
	 * no lanes", which is not a state.
	 */
	if (!status[1] || status[1] > 4) {
		log_err("cdn-dp: link status reports %u lanes\n", status[1]);
		return -EPROTO;
	}

	priv->max_rate = rate;
	priv->max_lanes = status[1];

	return 0;
}

/**
 * cdn_dp_training_diag() - report SINK-side state after training did not finish
 *
 * A training timeout at the top of this driver is a single errno, and an errno
 * cannot distinguish the two failures that produce it. Either the firmware ran
 * training and the sink would not lock, or the firmware never ran training at
 * all -- and those have opposite fixes, so the log has to say which.
 *
 * The DPCD alone does NOT tell them apart, and used to be asserted here as
 * though it did. The firmware, not the host, writes the link configuration:
 * DP_LINK_BW_SET and DP_LANE_COUNT_SET are written by the ucpu as the FIRST
 * act of a full link-training sequence. But cdn_dp_training_start() gives up
 * on a timeout without ever telling the ucpu to stop, so an all-zero link
 * configuration matches TWO states -- training never started, and training
 * started and has not written the DPCD yet -- and the reader cannot tell which
 * from the DPCD bytes. A discriminator that matches both of the things it is
 * meant to separate is not one.
 *
 * @event_bits separates them, and it does not depend on the ucpu having
 * stopped: it is the OR of every DPTX_READ_EVENT status byte cdn_dp_training_start()
 * saw, i.e. the FIRMWARE's own report of what it did, delivered over the
 * mailbox the host owns rather than over the AUX channel the ucpu may still be
 * holding. FULL_LT_STARTED / FAST_LT_STARTED say a training sequence began.
 * Zero link configuration WITH a start bit is a ucpu that has not written the
 * DPCD yet.
 *
 * Zero link configuration WITHOUT one is FOUR states, not one, which is why
 * @polls, @run_delivered and @poll_aborted are all read here. Only the last of
 * the four -- a poll loop that ran to its full budget and never saw a start
 * bit -- is a training that never began, and only it may exonerate lane drive
 * levels and cable quality.
 *
 * WITHDRAWN: this paragraph used to say the "event send" / "event hdr" /
 * "event body" exits "all leave @event_bits at the zero it was initialised to"
 * and identified them with @polls == 0. That holds only on the first
 * iteration. They are inside the poll loop, so a failure on iteration 2 or
 * later leaves @polls >= 1 and @run_delivered true, with @event_bits still 0
 * if the earlier replies carried nothing -- and the final arm below then
 * printed the exoneration on the strength of a single 20 ms sample out of a
 * 500 ms window. @poll_aborted is what actually separates "the window was cut
 * short" from "the window completed", at any @polls.
 *
 * The third state is @polls == 0 with @run_delivered FALSE: the "train run"
 * send failed, so LINK_TRAINING_RUN never reached the ucpu at all. That is a
 * fourth zero-poll exit that 4006834b68b did not enumerate, and its arm printed
 * "LINK_TRAINING_RUN was delivered but the first DPTX_READ_EVENT round trip
 * failed" -- an assertion about a delivery that did not occur, on the very
 * commit whose subject was a diagnostic exonerating the cable on no evidence.
 * The two have different fixes: nothing was asked of the ucpu here, so this
 * one does not even reach the question of whether the poll path works.
 *
 * 08891231902 named a narrower residual than this and missed this one: a start
 * bit raised and cleared BETWEEN two samples is a poll that happened and saw
 * nothing, whereas the case above is zero polls completed. Both survive; they
 * are different shapes.
 *
 * The residual uncertainty on the @polls > 0 arm is that sampling question:
 * the status byte is read every LINK_TRAINING_RETRY_MS, so a start that both
 * raised and cleared its bit between two samples would be missed. That is a
 * sampling question about a mailbox reply, not an unknown about who owns AUX.
 *
 * If the link configuration carries a rate and a lane count but the lane status
 * bytes have no CR_DONE, training ran and the sink did not lock, which is the
 * ordinary marginal-link failure -- but ONLY if the firmware reported a start
 * bit for THIS attempt. DPCD 0x100/0x101 are not cleared between attempts, and
 * cdn_dp_train_link() has two callers in one boot (rk_cdn_dp_enable() and
 * cdn_dp_bringup()), so a second call can read the first one's bytes and the
 * marginal-link verdict would blame the physical layer on an attempt the ucpu
 * never ran. This function is NOT told which of the two calls it is on -- @when
 * is only "fail" / "pass" / "readback-fail" -- so no message here may assert
 * that a byte was written by "the earlier call": on the FIRST call of a boot
 * there is no earlier call, and 863806dde8a's new arm said it anyway. The
 * messages say the provenance is UNKNOWN, which is what is actually known.
 *
 * The converse of the start-bit gate is that its ABSENCE is only evidence on a
 * window that ran to its full budget, for the sampling reason stated above. A
 * cut-short or zero-poll window therefore gets its own arm in the CR_DONE
 * branch and does NOT exonerate -- the same rule the zero-configuration branch
 * applies. 863806dde8a's note that "@poll_aborted needs no separate arm here"
 * because the message it lands on is "conservative" was wrong on both halves:
 * that message exonerates, and it did so on exactly the boot where a genuine
 * marginal link is indistinguishable from an unwatched window.
 *
 * WITHDRAWN: @run_delivered was the gate on that arm and is not enough.
 * 6994b797764 added the flag and consulted it in only one branch; 32259e84540
 * added it to the other and called the gap closed. It is not: @run_delivered
 * says the DPTX_TRAINING_CONTROL send returned 0, i.e. the mailbox ACCEPTED the
 * request -- which is exactly what its own comment in cdn_dp_training_start()
 * claims and no more. A send that succeeds while the ucpu never starts leaves
 * @run_delivered true, cfg[] holding the previous call's bytes, and the arm
 * asserting "the firmware DID configure the sink". The gate is now
 * @event_bits & (FULL_LT_STARTED | FAST_LT_STARTED), the firmware's own report,
 * which was already being computed for the zero-configuration branch and read
 * nowhere else.
 *
 * Neither arm's advice may say "read the log_msg_ret tag": log_msg_ret() is
 * ((void)(_msg), _ret) unless CONFIG_IS_ENABLED(LOG) && CONFIG_LOG_ERROR_RETURN
 * (include/log.h), and configs/roc-rk3399-pc_defconfig sets no CONFIG_LOG* at
 * all -- so the tag cannot print on the one board this diagnostic exists for.
 * The messages name the failing call site instead and state that precondition.
 *
 * Read over AUX, which is known to work at this point -- but NOT for the reason
 * first written here. "This only runs from the .enable path" was false and is
 * WITHDRAWN: cdn_dp_train_link(), this function's sole caller, is called from
 * rk_cdn_dp_enable() AND from cdn_dp_bringup(), the `cdn_dp` command path. The
 * precondition holds on both, and that is what it rests on now: each reaches
 * training only after reading an EDID over this same AUX channel this boot
 * (.read_edid before .enable; cdn_dp_get_edid() before the training call in
 * cdn_dp_bringup()). A third caller that trains without reading an EDID would
 * break it, which is exactly why the reason has to name the real condition
 * rather than a call site.
 *
 * ⚠️ It is still NOT known that the ucpu has stopped using AUX.
 * cdn_dp_training_start() returns -ETIMEDOUT without ever sending
 * DPTX_TRAINING_CONTROL with LINK_TRAINING_NOT_ACTIVE, so on the timeout path
 * the firmware may still own the AUX channel while these reads go out. That
 * remains true and is why every DPCD value below is evidence rather than
 * proof. Sending LINK_TRAINING_NOT_ACTIVE first was considered and NOT done:
 * no source we hold ever sends it. Linux's cdn-dp-reg.c, which this mailbox is
 * mirrored from, defines LINK_TRAINING_NOT_ACTIVE in cdn-dp-reg.h and never
 * uses it; the Cadence MHDP8546 bridge driver does not use this opcode at all.
 * Its meaning therefore rests entirely on the name of a macro, the opcode is
 * fire-and-forget so a success return would say only that the request was
 * delivered, and there is no ack that would tell the host when the ucpu had
 * actually stopped and released AUX. Guessing at firmware state with an
 * unexercised opcode, inside the diagnostic whose job is to be trustworthy
 * about firmware state, buys nothing @event_bits does not already give.
 *
 * Diagnostic only -- it changes nothing and its own failures are printed rather
 * than propagated, because a failed diagnostic must not become the reported
 * cause of the fault it was called to describe.
 */
static void cdn_dp_training_diag(struct rk_cdn_dp_priv *priv, const char *when,
				 u8 event_bits, unsigned int polls,
				 bool run_delivered, bool poll_aborted)
{
	u8 caps[4], cfg[7], lane[6], power;
	char hpd_str[16];
	int hpd, ret;

	/*
	 * cdn_dp_get_hpd_status() returns the status byte OR a negative errno
	 * from the mailbox, and a mailbox that is not answering is one of the
	 * two states this diagnostic exists to tell apart. Printed with a bare
	 * %d it came out as `hpd=-22`, with nothing to separate it from a sink
	 * that has deasserted HPD -- so a failed query is rendered as ERR<n>
	 * and the success case prints exactly what it always did.
	 */
	hpd = cdn_dp_get_hpd_status(priv);
	if (hpd < 0)
		snprintf(hpd_str, sizeof(hpd_str), "ERR%d", hpd);
	else
		snprintf(hpd_str, sizeof(hpd_str), "%d", hpd);

	/*
	 * The sink's own capability bytes, re-read HERE rather than reused from
	 * the copy .read_edid took. They are what bounds the link the firmware
	 * may negotiate, and a sink that reports a different maximum on the
	 * failing boot than on the passing one is a completely different fault
	 * from a sink that reports the same maximum and will not lock to it.
	 * Reusing the cached copy could not tell those apart.
	 */
	ret = cdn_dp_dpcd_read(priv, DP_DPCD_REV, caps, sizeof(caps));
	if (ret) {
		printf("cdn-dp: training diag(%s): DPCD sink-caps read failed (%d)\n",
		       when, ret);
		return;
	}

	/*
	 * 0x100..0x106 in one read: the link configuration AND the four
	 * per-lane drive settings the firmware last wrote. The drive settings
	 * are the half that was missing. bw_set/lane_count_set only say that
	 * training was attempted; DP_TRAINING_LANE0_SET says at WHICH voltage
	 * swing and pre-emphasis it was attempted, and a clock-recovery loop
	 * that ended at level 0 has a different cause from one that walked to
	 * the maximum and still got nothing.
	 */
	ret = cdn_dp_dpcd_read(priv, DP_LINK_BW_SET, cfg, sizeof(cfg));
	if (ret) {
		printf("cdn-dp: training diag(%s): DPCD link-config read failed (%d)\n",
		       when, ret);
		return;
	}

	/*
	 * 0x202..0x207: lane status, align, sink status, and the two ADJUST
	 * REQUEST bytes. The adjust requests are the SINK's answer -- the swing
	 * and pre-emphasis it is asking the source for -- so together with
	 * 0x103..0x106 above they say whether the source gave the sink what it
	 * asked for. A sink asking for a level the source already drove is a
	 * link that cannot be closed by adjusting drive at all.
	 */
	ret = cdn_dp_dpcd_read(priv, DP_LANE0_1_STATUS, lane, sizeof(lane));
	if (ret) {
		printf("cdn-dp: training diag(%s): DPCD lane-status read failed (%d)\n",
		       when, ret);
		return;
	}

	ret = cdn_dp_dpcd_read(priv, DP_SET_POWER, &power, 1);
	if (ret)
		power = 0xff;

	printf("cdn-dp: training diag(%s): hpd=%s bw_set=0x%02x lane_count_set=0x%02x lane01=0x%02x lane23=0x%02x align=0x%02x sink=0x%02x power=0x%02x\n",
	       when, hpd_str, cfg[0], cfg[1], lane[0], lane[1], lane[2],
	       lane[3], power);
	printf("cdn-dp: training diag(%s): sink_caps rev=0x%02x max_rate=0x%02x max_lanes=0x%02x downspread=0x%02x\n",
	       when, caps[0], caps[1], caps[2], caps[3]);
	printf("cdn-dp: training diag(%s): tps=0x%02x lane_set=%02x %02x %02x %02x adjust=%02x %02x\n",
	       when, cfg[2], cfg[3], cfg[4], cfg[5], cfg[6], lane[4], lane[5]);

	/*
	 * State the conclusion the log used to leave to a human reading hex.
	 *
	 * Both branches, because one of them was missing: the headline finding
	 * of the commit that added this diagnostic ("lane01/lane23 = 0x00 is no
	 * CR_DONE on any lane") was a person decoding the bytes, while the only
	 * classifier emitted here covered the OTHER case. DP_LANE_CR_DONE was
	 * defined and never read. A diagnostic that does not state its own
	 * verdict makes its reader re-derive it, and a re-derivation is where a
	 * wrong claim gets into a commit message.
	 *
	 * CR_DONE is bit 0 of each lane's nibble: 0x202 holds lanes 0 and 1,
	 * 0x203 lanes 2 and 3.
	 */
	if (!cfg[0] && !(cfg[1] & DP_LANE_COUNT_MASK)) {
		/*
		 * An all-zero link configuration is split by the firmware's own
		 * start bits rather than left as the two-state verdict it used
		 * to print. See the @event_bits paragraph in the function
		 * comment for why the mailbox report is usable here when the
		 * DPCD bytes on their own are not.
		 */
		if (event_bits & (FULL_LT_STARTED | FAST_LT_STARTED))
			printf("cdn-dp: training diag(%s): the sink's link configuration is UNWRITTEN but the firmware reported a training START (status 0x%02x, %u poll(s)) -- the ucpu began training and had not written the DPCD when this was read\n",
			       when, event_bits, polls);
		else if (!run_delivered)
			printf("cdn-dp: training diag(%s): the sink's link configuration is UNWRITTEN and LINK_TRAINING_RUN was NEVER SENT (status 0x%02x, 0 polls) -- the DPTX_TRAINING_CONTROL mailbox send itself failed, so the ucpu was never asked to train and nothing here says anything about the link; this does NOT exonerate lane drive levels or cable quality. Suspect the mailbox on the SEND side: the failing call is the DPTX_TRAINING_CONTROL send in cdn_dp_training_start(). Its errno is NOT printed on this build -- log_msg_ret() is a no-op unless CONFIG_LOG_ERROR_RETURN is set, and roc-rk3399-pc_defconfig does not set it\n",
			       when, event_bits);
		/*
		 * @polls == 0 with @run_delivered true means the FIRST
		 * DPTX_READ_EVENT round trip failed, and nothing else. This is
		 * the one place that finding is stated; the CR_DONE branch
		 * below points here rather than restating it.
		 *
		 * 72dd5b1fbfb named a second way in -- the
		 * `while (get_timer(start) < LINK_TRAINING_TIMEOUT_MS)` header
		 * evaluating false on its FIRST test, returning -ETIMEDOUT with
		 * the body never entered and no round trip attempted -- and
		 * said "its reachability is NOT established here". 788d65d00a2
		 * then stated the same thing as fact in the CR_DONE branch and
		 * split an arm around it, leaving two comments in one function
		 * disagreeing about one state. Neither is right: the state is
		 * UNREACHABLE. get_timer(base) is
		 * `tick_to_time(get_ticks()) - base` (lib/time.c:145-148), and
		 * @start is get_timer(0) sampled in the statement immediately
		 * before the loop, so the header's first test measures the
		 * elapsed time of two adjacent statements against a 500 ms
		 * budget. It cannot be false there.
		 *
		 * So the disjunction is dropped and this says the one thing the
		 * code permits. The three mailbox aborts (@poll_aborted true)
		 * are what actually produce @polls == 0 here, and they ARE that
		 * failing first round trip -- which is why naming its three
		 * steps is a statement about a loop that DID run one iteration,
		 * not, as 788d65d00a2's arm had it, about one that may not have
		 * run at all.
		 */
		else if (!polls)
			printf("cdn-dp: training diag(%s): the sink's link configuration is UNWRITTEN and NO status byte was ever read (status 0x%02x, 0 polls) -- LINK_TRAINING_RUN was delivered and the FIRST DPTX_READ_EVENT round trip failed, at one of its three steps (send, header, body) in cdn_dp_training_start()'s poll loop, so whether training began is UNKNOWN; this does NOT exonerate lane drive levels or cable quality. That step's errno is NOT printed on this build -- log_msg_ret() is a no-op unless CONFIG_LOG_ERROR_RETURN is set, and roc-rk3399-pc_defconfig does not set it\n",
			       when, event_bits);
		else if (poll_aborted)
			printf("cdn-dp: training diag(%s): the sink's link configuration is UNWRITTEN and the poll loop was CUT SHORT by a mailbox failure after %u poll(s) (status 0x%02x) -- the 500 ms training window was not watched to its end, so whether training began is UNKNOWN; this does NOT exonerate lane drive levels or cable quality. Suspect the mailbox: the failing call is one of the three DPTX_READ_EVENT steps (send, header, body) in cdn_dp_training_start()'s poll loop. Its errno is NOT printed on this build -- log_msg_ret() is a no-op unless CONFIG_LOG_ERROR_RETURN is set, and roc-rk3399-pc_defconfig does not set it\n",
			       when, polls, event_bits);
		/*
		 * "across the FULL poll(s) of the training window" was one
		 * word too strong: EQ_PHASE_FINISHED returns 0 from inside the
		 * loop, so a run that ended early rather than exhausting the
		 * 500 ms budget also arrives here. What the gate above
		 * actually establishes is that the loop was not CUT SHORT by a
		 * mailbox failure, which is what may exonerate; that is what
		 * this now says. (Whether a run can both reach
		 * EQ_PHASE_FINISHED and leave @event_bits without a start bit
		 * is a sampling question, not one whose reachability is
		 * established here.)
		 */
		else
			printf("cdn-dp: training diag(%s): the sink's link configuration is UNWRITTEN and the firmware never reported a training START across all %u poll(s) the loop completed, and the loop was not cut short (status 0x%02x) -- training never began, so lane drive levels and cable quality are not the fault\n",
			       when, polls, event_bits);
	} else if (!(lane[0] & DP_LANE_CR_DONE) &&
		   !(lane[0] & (DP_LANE_CR_DONE << 4)) &&
		   !(lane[1] & DP_LANE_CR_DONE) &&
		   !(lane[1] & (DP_LANE_CR_DONE << 4))) {
		/*
		 * "The firmware DID configure the sink" is an inference from
		 * DPCD 0x100/0x101 being non-zero, and those bytes are not
		 * cleared between attempts. cdn_dp_train_link() has two callers
		 * in one boot, rk_cdn_dp_enable() and cdn_dp_bringup(), so a
		 * second call can always read back the first call's bytes.
		 *
		 * WITHDRAWN: @run_delivered is NOT a sufficient gate for that
		 * inference, and 32259e84540 shipped it as one. It records only
		 * that the DPTX_TRAINING_CONTROL mailbox send RETURNED 0 -- a
		 * request the mailbox accepted, which is what its own comment
		 * in cdn_dp_training_start() says. It does not record that the
		 * ucpu ever started, and on the two-call boot above call #2's
		 * send can succeed while the ucpu does nothing; cfg[] then
		 * still holds call #1's bytes and this arm asserted firmware
		 * configuration and blamed the physical layer anyway. That is
		 * the same over-claim 6994b797764 exists to remove, one gate
		 * further in.
		 *
		 * The discriminator that DOES say the ucpu started is the
		 * firmware's own report, and it is already computed thirty
		 * lines above for the other branch: FULL_LT_STARTED /
		 * FAST_LT_STARTED in @event_bits. It gates this arm now.
		 *
		 * WITHDRAWN: "@poll_aborted needs no separate arm here: a
		 * cut-short window simply fails to raise a start bit, so it
		 * lands on the conservative message, which is correct."
		 * 863806dde8a wrote that and it is wrong twice over. The
		 * message it lands on is NOT conservative -- it ends "that
		 * does NOT implicate clock recovery, lane drive levels or
		 * cable quality", which EXONERATES the physical layer. And the
		 * absence of a start bit is not evidence on a window that was
		 * not watched to its end: @event_bits is sampled every
		 * LINK_TRAINING_RETRY_MS, so a start that both raised and
		 * cleared its bit between two samples is missed (the residual
		 * this file already states for the @polls > 0 arm above). A
		 * boot where the poll loop is cut short therefore took a
		 * GENUINE marginal link -- non-zero cfg[], no CR_DONE -- and
		 * exonerated it, where before 863806dde8a it printed the
		 * correct physical-layer verdict. That commit's purpose was to
		 * stop exonerating on insufficient evidence; this arm did it
		 * with the polarity flipped.
		 *
		 * The rule is the one the zero-configuration branch above
		 * already applies and enforces: only a poll loop that ran to
		 * its FULL BUDGET may exonerate. It is applied here now, so a
		 * cut-short or zero-poll window gets its own UNKNOWN arm and
		 * the exoneration is reachable only from a completed window.
		 */
		if (!run_delivered)
			printf("cdn-dp: training diag(%s): the sink's link configuration is NON-ZERO but LINK_TRAINING_RUN was NEVER SENT on this attempt -- the DPTX_TRAINING_CONTROL mailbox send itself failed, so bw_set/lane_count_set were NOT written by this attempt. DPCD 0x100/0x101 are not cleared between attempts and cdn_dp_train_link() is called up to twice per boot (rk_cdn_dp_enable() and cdn_dp_bringup()), but this diagnostic is not told which call it is on -- on the first call of a boot there is no earlier call at all -- so where these bytes came from is UNKNOWN and they are NOT evidence that the firmware configured the sink; no lane reached CR_DONE, but with nothing asked of the ucpu that does NOT implicate clock recovery, lane drive levels or cable quality. Suspect the mailbox on the SEND side: the failing call is the DPTX_TRAINING_CONTROL send in cdn_dp_training_start(). Its errno is NOT printed on this build -- log_msg_ret() is a no-op unless CONFIG_LOG_ERROR_RETURN is set, and roc-rk3399-pc_defconfig does not set it\n",
			       when);
		/*
		 * @polls == 0 is a DIFFERENT state from a loop cut short, and
		 * folding the two into one arm made this branch assert a call
		 * that may never have been made. Split, and named the way the
		 * zero-configuration branch above names it -- including its
		 * reachability finding, which is stated there and NOT restated
		 * here: the poll-loop header cannot be false on its first test,
		 * so @polls == 0 with @run_delivered true IS the first round
		 * trip failing.
		 *
		 * 788d65d00a2 wrote that state's unreachable half as fact while
		 * the zero-configuration branch still called it unestablished,
		 * and split this arm around a case the code does not permit.
		 * The split stays -- @polls == 0 and a cut-short loop really
		 * are different -- but the arm now claims only what is
		 * reachable.
		 *
		 * Its `!(event_bits & (FULL_LT_STARTED | FAST_LT_STARTED))`
		 * conjunct is dropped as well: it read as a discriminating gate
		 * and could not discriminate. *event_bits |= event[1] runs only
		 * after (*polls)++ in cdn_dp_training_start(), so @polls == 0
		 * implies @event_bits is still the 0 it was initialised to, and
		 * the conjunct is a tautology on this arm.
		 */
		else if (!polls)
			printf("cdn-dp: training diag(%s): the sink's link configuration is NON-ZERO, no lane reached CR_DONE, and NO status byte was ever read (status 0x%02x, 0 polls) -- LINK_TRAINING_RUN was delivered and the FIRST DPTX_READ_EVENT round trip failed, at one of its three steps (send, header, body) in cdn_dp_training_start()'s poll loop, so whether the ucpu ran this attempt is UNKNOWN and the absence of a start bit says nothing. This does NOT exonerate clock recovery, lane drive levels or cable quality -- a genuine marginal link produces exactly these bytes. That step's errno is NOT printed on this build -- log_msg_ret() is a no-op unless CONFIG_LOG_ERROR_RETURN is set, and roc-rk3399-pc_defconfig does not set it\n",
			       when, event_bits);
		else if (!(event_bits & (FULL_LT_STARTED | FAST_LT_STARTED)) &&
			 poll_aborted)
			printf("cdn-dp: training diag(%s): the sink's link configuration is NON-ZERO, no lane reached CR_DONE, and the firmware reported no training START (status 0x%02x) -- but the poll loop was CUT SHORT by a mailbox failure after %u poll(s), so the 500 ms training window was not watched to its end and the absence of a start bit says nothing: a start raised and cleared between two samples would have been missed. Whether the ucpu ran this attempt is UNKNOWN, and this does NOT exonerate clock recovery, lane drive levels or cable quality -- a genuine marginal link produces exactly these bytes. Suspect the mailbox: the failing call is one of the three DPTX_READ_EVENT steps (send, header, body) in cdn_dp_training_start()'s poll loop. Its errno is NOT printed on this build -- log_msg_ret() is a no-op unless CONFIG_LOG_ERROR_RETURN is set, and roc-rk3399-pc_defconfig does not set it\n",
			       when, event_bits, polls);
		/*
		 * "across the FULL poll(s) of the training window" is the same
		 * word 72dd5b1fbfb corrected in the zero-configuration arm and
		 * left standing here: EQ_PHASE_FINISHED returns 0 from inside
		 * cdn_dp_training_start()'s loop, so a run that ended early
		 * rather than exhausting the 500 ms budget arrives here too,
		 * with @poll_aborted false and @polls >= 1. What the gates
		 * above establish is that the loop was not CUT SHORT by a
		 * mailbox failure, not that it ran to its budget.
		 */
		else if (!(event_bits & (FULL_LT_STARTED | FAST_LT_STARTED)))
			printf("cdn-dp: training diag(%s): the sink's link configuration is NON-ZERO but the firmware reported NO training START across all %u poll(s) the loop completed, and the loop was not cut short (status 0x%02x) -- LINK_TRAINING_RUN was accepted by the mailbox, which says the request was queued and NOT that the ucpu ran it. DPCD 0x100/0x101 are not cleared between attempts and cdn_dp_train_link() is called up to twice per boot (rk_cdn_dp_enable() and cdn_dp_bringup()), but this diagnostic is not told which call it is on -- on the first call of a boot there is no earlier call at all -- so where bw_set/lane_count_set came from is UNKNOWN and they are NOT evidence that the firmware configured the sink on this attempt; no lane reached CR_DONE, but that does NOT implicate clock recovery, lane drive levels or cable quality\n",
			       when, polls, event_bits);
		else
			printf("cdn-dp: training diag(%s): the firmware DID configure the sink and reported a training START (status 0x%02x, %u poll(s)), and NO lane reached CR_DONE -- clock recovery never succeeded (physical layer)\n",
			       when, event_bits, polls);
	}
}

/**
 * cdn_dp_train_link() - train the link, then read back what was negotiated
 *
 * On any failure priv->max_rate and priv->max_lanes are left at zero rather
 * than at whatever a previous successful run put there. A stale rate surviving
 * a failed retrain is exactly the input that makes cdn_dp_config_video()
 * produce a valid-looking TU for a link that is no longer up.
 */
static int cdn_dp_train_link(struct rk_cdn_dp_priv *priv, u8 *event_bits)
{
	unsigned int polls;
	bool run_delivered;
	bool poll_aborted;
	int ret;

	priv->max_rate = 0;
	priv->max_lanes = 0;

	/*
	 * polls, run_delivered and poll_aborted stay local: they exist to keep
	 * cdn_dp_training_diag() from reading a zero @event_bits as a verdict
	 * when it is only an absence of evidence, and no caller of this function
	 * has any other use for them.
	 */
	ret = cdn_dp_training_start(priv, event_bits, &polls, &run_delivered,
				    &poll_aborted);
	if (ret) {
		cdn_dp_training_diag(priv, "fail", *event_bits, polls,
				     run_delivered, poll_aborted);
		return ret;
	}

	/*
	 * The SAME dump on the way out of a SUCCESSFUL training run, and that
	 * symmetry is the whole point.
	 *
	 * A failing boot described on its own says only that clock recovery did
	 * not happen; it cannot say which of the twenty-odd values it prints is
	 * abnormal, because there is nothing to call normal. Two boots of the
	 * same firmware on the same board and cable, one trained and one not,
	 * reduce that to a diff -- and a diff is a finding where a single dump
	 * is a list. Printed unconditionally rather than behind a debug flag for
	 * the same reason: the fault is intermittent, so the passing sample has
	 * to be captured by the run that happens to pass, not requested
	 * afterwards.
	 *
	 * ⚠️ Both arms above USED to be followed by rockchip_typec_phy_dp_diag(),
	 * a Type-C PHY register dump. Those two calls reached this branch as a
	 * shared-worktree leak (see the commit removing them): their declaration
	 * and their implementation live in the B113 investigation branch and were
	 * never on production, so the calls did not compile here. If that PHY dump
	 * lands, it must re-add its own call sites -- the commit that carries the
	 * implementation no longer carries them.
	 */
	ret = cdn_dp_get_training_status(priv);
	cdn_dp_training_diag(priv, ret ? "readback-fail" : "pass", *event_bits,
			     polls, run_delivered, poll_aborted);

	return ret;
}

/**
 * cdn_dp_set_video_status() - tell the framer whether the video stream is valid
 *
 * CONTROL_VIDEO_IDLE before reconfiguring, CONTROL_VIDEO_VALID once pixels are
 * actually arriving. There is no reply to validate, so a failure here can only
 * be a stalled mailbox write -- and, as with every other fire-and-forget
 * opcode, a success return says the request was DELIVERED, not that the framer
 * agreed with it.
 */
static int cdn_dp_set_video_status(struct rk_cdn_dp_priv *priv, int active)
{
	u8 msg = !!active;

	return cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX, DPTX_SET_VIDEO,
				   sizeof(msg), &msg);
}

/**
 * cdn_dp_get_msa_misc() - build the MSA MISC0/MISC1 word for the current format
 *
 * Straight from Linux's cdn_dp_get_msa_misc(), cdn-dp-reg.c:583-628, including
 * the shape of the arithmetic: MISC0's colorimetry field sits at bit 1 and the
 * bit-depth field at bit 5, which is what the 2* and 32* multipliers are. The
 * YUV arms are unreachable here (color_fmt is always PXL_RGB) and are kept
 * only so this stays diffable against the reference.
 */
static u32 cdn_dp_get_msa_misc(struct rk_cdn_dp_priv *priv)
{
	u8 val[2] = { 0, 0 };

	switch (priv->color_fmt) {
	case PXL_RGB:
	case Y_ONLY:
		val[0] = 0;
		break;
	/* YUV defaults to BT.601, as in the reference. */
	case YCBCR_4_4_4:
		val[0] = 6 + BT_601 * 8;
		break;
	case YCBCR_4_2_2:
		val[0] = 5 + BT_601 * 8;
		break;
	case YCBCR_4_2_0:
		val[0] = 5;
		break;
	}

	switch (priv->color_depth) {
	case 6:
		val[1] = 0;
		break;
	case 8:
		val[1] = 1;
		break;
	case 10:
		val[1] = 2;
		break;
	case 12:
		val[1] = 3;
		break;
	case 16:
		val[1] = 4;
		break;
	}

	return 2 * val[0] + 32 * val[1] +
	       ((priv->color_fmt == Y_ONLY) ? BIT(14) : 0);
}

/**
 * cdn_dp_sync_negative() - is this sync pulse active-low, as DP_FRAMER_SP wants
 * @flags: display_timing flags
 * @high:  the DISPLAY_FLAGS_*SYNC_HIGH bit for the axis
 * @low:   the DISPLAY_FLAGS_*SYNC_LOW bit for the same axis
 *
 * The two sides state polarity with OPPOSITE senses, and the flags are named
 * similarly enough that copying the reference verbatim gives a driver that
 * trains, configures, reports success, and paints a wrong or unstable picture
 * with nothing anywhere reading as an error.
 *
 * Linux (cdn-dp-core.c:551-552) sets
 *   video->h_sync_polarity = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);
 * i.e. the field it feeds to DP_FRAMER_SP (cdn-dp-reg.c:710-711) and to
 * MSA_HORIZONTAL_1 bit 15 (cdn-dp-reg.c:733) means sync is NEGATIVE.
 * struct display_timing names the POSITIVE case instead.
 *
 * THREE branches, not two, because a display_timing can state neither.
 * common/edid.c:307-315 always sets exactly one of the pair, so on the EDID
 * path any spelling agrees -- but lib/fdtdec.c:1045-1055 sets NEITHER when the
 * DT omits hsync-active, and there both two-branch spellings invent an answer:
 * `!!(flags & _LOW)` claims positive, `!(flags & _HIGH)` claims negative. They
 * differ only in WHICH fiction they tell.
 *
 * The unstated case follows the reference's own default. A DRM mode with
 * neither NHSYNC nor PHSYNC yields h_sync_polarity == 0, i.e. POSITIVE, so
 * that is what a timing which never said is given here. Refusing the mode was
 * the alternative and is worse: it would fail every DT-supplied timing that
 * omits hsync-active, which is a legal thing for a DT to omit, and DP sinks
 * take their polarity from the MSA rather than needing a particular one.
 * log_debug so a picture that is wrong for this reason names its own cause.
 */
static bool cdn_dp_sync_negative(u32 flags, u32 high, u32 low)
{
	if (flags & high)
		return false;
	if (flags & low)
		return true;

	log_debug("cdn-dp: timing states no sync polarity -- assuming positive, as DRM does\n");

	return false;
}

/**
 * cdn_dp_config_video() - program the framer and MSA for @timing
 *
 * Linux's cdn_dp_config_video(), cdn-dp-reg.c:630-782, with the DRM mode
 * replaced by U-Boot's display_timing. The two describe the same thing
 * differently and the translation is where this is easy to get wrong: DRM
 * counts from the active edge (hsync_start = hdisplay + front porch), so
 * `mode->htotal - mode->hsync_start` is sync width PLUS back porch, not the
 * back porch. Each derived expression is named below rather than inlined.
 *
 * The whole conversion, so it can be checked without holding both files open.
 * DRM's mode is hdisplay <= hsync_start <= hsync_end <= htotal, each an
 * absolute position measured from the start of the active area; display_timing
 * gives the four intervals directly.
 *
 *   DRM (cdn-dp-reg.c)               here                       register
 *   hsync_start - hdisplay           hfront                     :716
 *   htotal - hsync_end               hback                      :717
 *   htotal                           htotal                     :727
 *   htotal - hsync_start             hsync + hback              :727
 *   hsync_end - hsync_start          hsync                      :732, :759
 *   hdisplay                         hactive                    :722, :733, :760
 *   vtotal                           vtotal                     :738, :771
 *   vtotal - vsync_start             vsync + vback              :739, :766
 *   vsync_end - vsync_start          vsync                      :744
 *   vdisplay                         vactive                    :745, :765
 *
 * The two that are easy to get backwards are `htotal - hsync_start` and
 * `vtotal - vsync_start`: they span the sync pulse AND the back porch, and
 * writing the back porch alone there shifts the image sideways or up by the
 * sync width with no error reported anywhere.
 *
 * REQUIRES a trained link. Every number here is scaled by the negotiated rate
 * and lane count, so running it against max_rate == 0 does not produce a
 * conservative configuration, it divides by zero.
 *
 * What this canNOT do is verify itself. The registers it writes are inside the
 * ucpu's window and this driver has no DPTX register read (see the opcode block
 * at the top), so a successful return means every mailbox write was accepted --
 * not that the framer is producing a stream. Only pixels from a VOP can show
 * that.
 */
static int cdn_dp_config_video(struct rk_cdn_dp_priv *priv,
			       const struct display_timing *timing)
{
	u32 hactive = timing->hactive.typ;
	u32 hfront = timing->hfront_porch.typ;
	u32 hsync = timing->hsync_len.typ;
	u32 hback = timing->hback_porch.typ;
	u32 vactive = timing->vactive.typ;
	u32 vfront = timing->vfront_porch.typ;
	u32 vsync = timing->vsync_len.typ;
	u32 vback = timing->vback_porch.typ;
	u32 htotal = hactive + hfront + hsync + hback;
	u32 vtotal = vactive + vfront + vsync + vback;
	u32 clock = timing->pixelclock.typ / 1000;	/* kHz, Linux's mode->clock */
	bool hsync_neg, vsync_neg;
	u8 tu_size_reg = TU_SIZE;
	u32 val, link_rate, rem;
	u64 symbol, fifo, drain;
	u8 bit_per_pix;
	int ret;

	/*
	 * Nothing below may be reported as this run's result unless this run
	 * produced it. A TU and symbol count from a previous, differently
	 * trained link surviving a failure here is a number the `video` command
	 * would print beside a fresh rate as though the two agreed.
	 */
	priv->tu_size = 0;
	priv->valid_symbols = 0;
	priv->valid_symbols_milli = 0;

	if (!priv->max_rate || !priv->max_lanes) {
		log_err("cdn-dp: no trained link -- train before configuring video\n");
		return -EINVAL;
	}

	/*
	 * Interlaced modes are refused rather than bypassed. The reference sets
	 * VIF_BYPASS_INTERLACE unconditionally and carries an interlaced flag
	 * it never applies in this function, so what an interlaced mode would
	 * actually do here is undetermined -- and a mode that is silently
	 * mis-timed is worse than one that is refused.
	 */
	if (timing->flags & DISPLAY_FLAGS_INTERLACED) {
		log_err("cdn-dp: interlaced modes are not supported\n");
		return -EINVAL;
	}

	/*
	 * Polarity, three-way -- see cdn_dp_sync_negative() for why the unstated
	 * case is its own branch and why it resolves to positive.
	 *
	 * This must follow the interlaced refusal above, not precede it:
	 * common/edid.c:317-318 ASSIGNS DISPLAY_FLAGS_INTERLACED over flags
	 * rather than or-ing it in, wiping both polarity bits, so an interlaced
	 * timing carries no usable polarity at all -- it would land in the
	 * "stated nothing" branch and be given a polarity it never had.
	 */
	hsync_neg = cdn_dp_sync_negative(timing->flags, DISPLAY_FLAGS_HSYNC_HIGH,
					 DISPLAY_FLAGS_HSYNC_LOW);
	vsync_neg = cdn_dp_sync_negative(timing->flags, DISPLAY_FLAGS_VSYNC_HIGH,
					 DISPLAY_FLAGS_VSYNC_LOW);

	bit_per_pix = (priv->color_fmt == YCBCR_4_2_2) ?
		      (priv->color_depth * 2) : (priv->color_depth * 3);

	link_rate = priv->max_rate / 1000;		/* MHz */

	/*
	 * The guard at the top of this function tests max_rate, but the two
	 * divisions below are by link_rate, and the truncating /1000 between
	 * them is not injective at zero: any max_rate under 1000 kHz is
	 * non-zero and yields link_rate == 0, i.e. a division by zero inside
	 * do_div(). Unreachable today -- cdn_dp_train_link() reads back one of
	 * 1620000 / 2700000 / 5400000 -- so this is guarding the derived value
	 * rather than fixing an observed fault, which is the point: the check
	 * belongs on the number that is actually used as a divisor.
	 */
	if (!link_rate) {
		log_err("cdn-dp: trained rate %u kHz is below 1 MHz -- refusing to compute a TU\n",
			priv->max_rate);
		return -EINVAL;
	}

	ret = cdn_dp_reg_write(priv, CDN_DP_BND_HSYNC2VSYNC,
			       VIF_BYPASS_INTERLACE);
	if (ret)
		return log_msg_ret("bnd h2v", ret);

	ret = cdn_dp_reg_write(priv, CDN_DP_HSYNC2VSYNC_POL_CTRL, 0);
	if (ret)
		return log_msg_ret("h2v pol", ret);

	/*
	 * Find a transfer-unit size and valid-symbol count, exactly as the
	 * reference does:
	 * 1. with the link clock fixed, start at TU 32;
	 * 2. VS = TU * Pclk * Bpp / (Lclk * lanes);
	 * 3. if the fractional part lands outside 0.1..0.85, or VS < 2, or
	 *    TU - VS < 4, add 2 to TU and try again.
	 *
	 * The fraction window is the part worth stating: a VS whose remainder
	 * sits near a whole symbol makes the rate governor alternate between
	 * two stuffing patterns, and the resulting jitter shows up as an
	 * unstable image rather than as an error anyone can read.
	 *
	 * Step 3's TU-VS test is spelled `symbol + 4 > tu_size_reg` here, not
	 * the reference's `tu_size_reg - symbol < 4`. Same inequality in exact
	 * arithmetic, but symbol is u64 and tu_size_reg is u8, so that
	 * subtraction is evaluated unsigned: an over-bandwidth mode, where
	 * symbol exceeds the TU, wraps it to ~2^64 rather than going negative.
	 * ~2^64 is not < 4, so the reference's loop ACCEPTS the very case the
	 * test exists to reject, and programs a TU smaller than its own symbol
	 * count.
	 */
	do {
		tu_size_reg += 2;
		symbol = (u64)tu_size_reg * clock * bit_per_pix;
		do_div(symbol, priv->max_lanes * link_rate * 8);
		rem = do_div(symbol, 1000);
		if (tu_size_reg > 64) {
			log_err("cdn-dp: no valid TU for %u kHz, %u lanes at %u MHz\n",
				clock, priv->max_lanes, link_rate);
			return -EINVAL;
		}
	} while (symbol <= 1 || symbol + 4 > tu_size_reg || rem > 850 ||
		 rem < 100);

	/* Recorded only so `cdn_dp video` prints what was programmed. */
	priv->tu_size = tu_size_reg;
	priv->valid_symbols = (u32)symbol;
	priv->valid_symbols_milli = rem;

	log_debug("cdn-dp: TU %u, %llu.%03u valid symbols per TU\n",
		  tu_size_reg, symbol, rem);

	val = (u32)symbol + (tu_size_reg << 8);
	val |= TU_CNT_RST_EN;
	ret = cdn_dp_reg_write(priv, CDN_DP_FRAMER_TU, val);
	if (ret)
		return log_msg_ret("framer tu", ret);

	/*
	 * The rate-governor FIFO depth, in Linux's exact form. Its derivation
	 * is not given in the reference and is not reconstructed here; the one
	 * thing that IS clear is that it is a function of the symbol count just
	 * chosen, so it has to be recomputed whenever training changes the rate
	 * or the lane count.
	 */
	fifo = lldiv((u64)clock * (symbol + 1), 1000) + link_rate;
	do_div(fifo, priv->max_lanes * link_rate);
	drain = lldiv(8 * (symbol + 1), bit_per_pix);

	/*
	 * The reference computes this as one expression in a u32 and cannot
	 * express a negative depth, so a subtraction that goes below zero
	 * becomes a huge positive FIFO depth that is written out as if it were
	 * a real answer. Refuse instead. The TU loop above should already make
	 * this unreachable -- the ratio of the two terms is TU/VS, which it
	 * holds above 1 -- so reaching it means the search accepted something
	 * it should not have, which is worth failing on rather than papering
	 * over with a clamp.
	 */
	if (drain < fifo) {
		log_err("cdn-dp: FIFO depth underflows (%llu < %llu) at TU %u\n",
			drain, fifo, tu_size_reg);
		return -ERANGE;
	}

	fifo = drain - fifo + 2;
	ret = cdn_dp_reg_write(priv, CDN_DP_VC_TABLE(15), (u32)fifo);
	if (ret)
		return log_msg_ret("vc table", ret);

	switch (priv->color_depth) {
	case 6:
		val = BCS_6;
		break;
	case 8:
		val = BCS_8;
		break;
	case 10:
		val = BCS_10;
		break;
	case 12:
		val = BCS_12;
		break;
	case 16:
		val = BCS_16;
		break;
	default:
		log_err("cdn-dp: %u bits per component is not a DP pixel depth\n",
			priv->color_depth);
		return -EINVAL;
	}

	val += priv->color_fmt << 8;
	ret = cdn_dp_reg_write(priv, CDN_DP_FRAMER_PXL_REPR, val);
	if (ret)
		return log_msg_ret("pxl repr", ret);

	/*
	 * cdn-dp-reg.c:710-711. Both bits assert for a NEGATIVE sync -- see the
	 * polarity note above; hsync_neg is already in the framer's sense, so it
	 * must not be inverted again here.
	 */
	val = hsync_neg ? DP_FRAMER_SP_HSP : 0;
	val |= vsync_neg ? DP_FRAMER_SP_VSP : 0;
	ret = cdn_dp_reg_write(priv, CDN_DP_FRAMER_SP, val);
	if (ret)
		return log_msg_ret("framer sp", ret);

	/* Front porch in the high half, back porch in the low half. */
	val = hfront << 16 | hback;
	ret = cdn_dp_reg_write(priv, CDN_DP_FRONT_BACK_PORCH, val);
	if (ret)
		return log_msg_ret("porch", ret);

	val = hactive * bit_per_pix / 8;
	ret = cdn_dp_reg_write(priv, CDN_DP_BYTE_COUNT, val);
	if (ret)
		return log_msg_ret("byte count", ret);

	/* MSA horizontal: total, then (sync width + back porch). */
	val = htotal | ((hsync + hback) << 16);
	ret = cdn_dp_reg_write(priv, CDN_DP_MSA_HORIZONTAL_0, val);
	if (ret)
		return log_msg_ret("msa h0", ret);

	/* Bit 15 is MSA MISC's sync-polarity bit, negative-asserted as above. */
	val = hsync | (hactive << 16) | (hsync_neg << 15);
	ret = cdn_dp_reg_write(priv, CDN_DP_MSA_HORIZONTAL_1, val);
	if (ret)
		return log_msg_ret("msa h1", ret);

	val = vtotal | ((vsync + vback) << 16);
	ret = cdn_dp_reg_write(priv, CDN_DP_MSA_VERTICAL_0, val);
	if (ret)
		return log_msg_ret("msa v0", ret);

	val = vsync | (vactive << 16) | (vsync_neg << 15);
	ret = cdn_dp_reg_write(priv, CDN_DP_MSA_VERTICAL_1, val);
	if (ret)
		return log_msg_ret("msa v1", ret);

	ret = cdn_dp_reg_write(priv, CDN_DP_MSA_MISC,
			       cdn_dp_get_msa_misc(priv));
	if (ret)
		return log_msg_ret("msa misc", ret);

	ret = cdn_dp_reg_write(priv, CDN_DP_STREAM_CONFIG, 1);
	if (ret)
		return log_msg_ret("stream cfg", ret);

	val = hsync | (hactive << 16);
	ret = cdn_dp_reg_write(priv, CDN_DP_HORIZONTAL, val);
	if (ret)
		return log_msg_ret("dp h", ret);

	val = vactive | ((vsync + vback) << 16);
	ret = cdn_dp_reg_write(priv, CDN_DP_VERTICAL_0, val);
	if (ret)
		return log_msg_ret("dp v0", ret);

	ret = cdn_dp_reg_write(priv, CDN_DP_VERTICAL_1, vtotal);
	if (ret)
		return log_msg_ret("dp v1", ret);

	/*
	 * DP_VB_ID bit 2 clear. The reference does this last and does not say
	 * what the bit is; VB-ID bit 2 is the DisplayPort spec's NO_VIDEO_STREAM
	 * flag, so clearing it is consistent with declaring a stream present --
	 * but that reading is inference from the standard, not from the
	 * reference, and it is the only line here whose purpose is not directly
	 * attributable.
	 */
	return cdn_dp_reg_write_bit(priv, CDN_DP_VB_ID, 2, 1, 0);
}

/**
 * cdn_dp_get_dp_phy() - resolve and power on the DisplayPort half of the PHY
 *
 * This is the step whose absence made every DPCD read dead rather than flaky.
 * The AUX channel is not a cdn-dp block: it is lane 8 of the Type-C PHY, so its
 * analog driver, bias, LDO, resistance calibration and the AUXDA_POLARITY bit
 * that decides which SBU pin is AUX+ are all tcphy state, programmed only by
 * rockchip_tcphy_dp_aux_calibration() inside the PHY's power_on. cdn-dp's own
 * DP_AUX_* block is framing and FIFOs with no analog front end.
 *
 * Index 1, not 0. The node lists phys = <&tcphy0_dp>, <&tcphy1_dp>, and the
 * monitor is on Type-C1 (the receptacle beside the USB-A pair, i2c4/fusb1,
 * TCPM dev 1). Index 0 is the rear power+OTG port. Getting this wrong points
 * the DPTX at the other receptacle, where it finds nothing.
 */
static int cdn_dp_get_dp_phy(struct udevice *dev)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	struct rockchip_typec_phy_dp_state dp_state;
	int ret;

	if (!priv->dp_phy.dev) {
		ret = generic_phy_get_by_index(dev, 1, &priv->dp_phy);
		if (ret)
			return log_msg_ret("dp phy", ret);
	}

	/*
	 * Ask the PHY what the connector is doing BEFORE powering it on, and
	 * fail loudly if the port is not in DisplayPort alt mode. Powering the
	 * PHY into DP on a connector carrying USB would re-mux four live lanes.
	 */
	ret = rockchip_typec_phy_get_dp_state(&priv->dp_phy, &dp_state);
	if (ret) {
		/*
		 * No longer "run `tcpm dp enter` first". Asking the PHY for the
		 * DP state now REQUESTS the mode from the port manager, so
		 * reaching here means the entry was ATTEMPTED and did not
		 * produce a DisplayPort link this PHY can carry -- nothing
		 * attached, a USB-only cable or partner, or a pin assignment
		 * with no four- or two-lane DP in it. Telling the operator to
		 * type a command that has already run would send them after the
		 * wrong thing.
		 *
		 * An earlier version of this said the port manager "enters the
		 * mode by itself for any connector the board declared". It did,
		 * briefly, and that was the bug: entry on any probe meant
		 * `usb start` reconfigured USB-C partners for DisplayPort.
		 * Entry follows this request and nothing else.
		 */
		log_err("cdn-dp: no DisplayPort link on this Type-C port (%d) -- alt mode not entered; `tcpm dev 1; tcpm dp` to see what the partner offered\n",
			ret);
		return ret;
	}

	priv->flip = dp_state.flip;
	priv->lanes = dp_state.lanes;

	ret = generic_phy_power_on(&priv->dp_phy);
	if (ret)
		return log_msg_ret("phy power on", ret);
	priv->phy_on = true;

	/*
	 * Route the Type-C PHY's HPD into the DPTX's HPD input. Without this
	 * the firmware's very next HPD query answers 0 -- not because the
	 * monitor is absent, but because nothing is connected to the pin the
	 * firmware is looking at. Linux writes it here too, after phy_power_on
	 * and before its first get_hpd_status.
	 */
	writel(DPTX_HPD_SEL_MASK | DPTX_HPD_SEL,
	       priv->grf_base + GRF_SOC_CON26);

	return 0;
}

/**
 * cdn_dp_ensure_phy() - power the DP PHY on if it is not already
 *
 * Every AUX transaction needs this, not just the ones on the .enable path, and
 * that distinction was the bug: cdn_dp_get_dp_phy() was called ONLY from
 * .enable, while .read_edid ran before it. display_read_timing() calls
 * .read_edid, and rk_display_init() only reaches display_enable() once a timing
 * comes back -- so the PHY was powered on after the first successful EDID read
 * and never before one. EDID needs AUX, AUX needs the PHY's analog front end,
 * and the PHY was waiting for EDID to succeed.
 *
 * The same hole made the diagnostic commands lie. `cdn_dp edid` and
 * `cdn_dp dpcd` reach the AUX channel without going through .enable at all, so
 * run standalone at the prompt they addressed an unpowered PHY and failed every
 * time -- which reads exactly like an intermittent AUX channel and is not one.
 * Measurements taken that way describe this bug, not the hardware.
 *
 * Idempotent on priv->phy_on, which cdn_dp_get_dp_phy() already set and nothing
 * consulted. Being callable from anywhere that is about to touch AUX is the
 * point; a second generic_phy_power_on() would re-run the tcphy DP calibration
 * underneath a live link.
 */
static int cdn_dp_ensure_phy(struct udevice *dev)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);

	if (!priv->phy_on)
		return cdn_dp_get_dp_phy(dev);

	/*
	 * Powered already, so do NOT re-read the connector state, and the
	 * previous version of this function did.
	 *
	 * The intent was sound -- cdn_dp_get_dp_phy() is the only writer of
	 * priv->flip and priv->lanes, rk_cdn_dp_fw_init() used to call it
	 * unconditionally, and short-circuiting on phy_on froze both. But the
	 * repair did not work and made things worse, so it is gone rather than
	 * adjusted:
	 *
	 *   - flip could not be refreshed at all. Once the PHY is muxed,
	 *     rockchip_typec_phy_get_dp_state() deliberately returns priv->flip
	 *     -- the orientation the lanes and AUX are PHYSICALLY muxed for --
	 *     and only reports the live port answer while MODE_DISCONNECT. This
	 *     branch runs exactly when the PHY is muxed, so the re-read handed
	 *     back the value cdn-dp already held. The comment claiming a
	 *     re-plug would now be caught was simply wrong.
	 *   - lanes COULD be refreshed, because dp->conf is re-read. That is
	 *     the harm: refreshing one half of a pair that describes a single
	 *     mux state replaced a consistent stale pair with a possibly
	 *     inconsistent one -- lanes = 2 from a renegotiated pin assignment
	 *     D, against a PHY still muxed 4-lane for C.
	 *
	 * The correct fix is to re-mux, not to re-read, and re-muxing under a
	 * live link is exactly what phy_on exists to prevent. So a cable event
	 * after power-on is out of scope here and belongs to whatever tears the
	 * link down; consistency with the hardware is the property worth
	 * keeping.
	 */
	return 0;
}

/**
 * rk_cdn_dp_fw_init() - load the ucpu firmware and bring it out of standby
 */
static int rk_cdn_dp_fw_init(struct udevice *dev)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	const void *fw;
	void *fw_alloc;
	int ret;

	/*
	 * Put the controller back to its cold state FIRST, so this sequence is
	 * idempotent. It was not: on a second `cdn_dp fw` the APB test was
	 * skipped (apb_live already set) and cdn_dp_clock_reset() rewrote the
	 * DPTX CAR registers while the firmware from the first run was
	 * executing, and the first posted write aborted.
	 *
	 * A block reset, not just APB_XT_RESET. Halting the ucpu was tried
	 * first and is NOT sufficient -- once firmware has run, the 0x9xx
	 * window stays closed until these three resets are cycled. Costs
	 * nothing on the first run, where it repeats what probe just did.
	 */
	cdn_dp_block_reset(priv);
	writel(APB_IRAM_PATH | APB_DRAM_PATH | APB_XT_RESET,
	       priv->regs + CDN_DP_APB_CTRL);
	priv->fw_loaded = false;
	priv->init_done = false;

	/*
	 * The APB test before the load. A firmware load is tens of thousands
	 * of consecutive writes, and if the aperture is dead the abort lands
	 * somewhere inside that loop and reads like a firmware problem.
	 */
	if (!priv->apb_live) {
		ret = rk_cdn_dp_apb_test(dev);
		if (ret)
			return ret;
	}

	ret = cdn_dp_get_firmware(&fw, &fw_alloc);
	if (ret < 0)
		return ret;
	printf("cdn-dp: firmware %d B\n", ret);

	cdn_dp_set_fw_clk(priv);
	cdn_dp_clock_reset(priv);

	/*
	 * The image is dead to us the moment the last word is in IRAM/DRAM, so
	 * the buffer is released here rather than at every later return. On the
	 * FIT arm fw_alloc is NULL and free(NULL) is a no-op -- the ownership
	 * question was answered by cdn_dp_get_firmware(), not by this caller.
	 */
	ret = cdn_dp_load_firmware(priv, fw);
	free(fw_alloc);
	if (ret)
		return ret;
	printf("cdn-dp: ucpu running, firmware version %u.%u\n",
	       priv->fw_version >> 8, priv->fw_version & 0xff);

	ret = cdn_dp_set_firmware_active(priv, true);
	if (ret)
		return ret;

	/*
	 * Straight after activation, as in the reference. It must come before
	 * fw_loaded is announced rather than lazily on the first `cdn_dp hpd`,
	 * because the firmware latches the plug state it sees while events are
	 * enabled; arming it later means the first query answers about a
	 * transition that has already happened.
	 */
	ret = cdn_dp_event_config(priv);
	if (ret) {
		log_err("cdn-dp: event config failed (%d)\n", ret);
		return ret;
	}

	priv->fw_loaded = true;
	printf("cdn-dp: firmware ACTIVE, mailbox answering\n");

	/*
	 * The PHY last, matching Linux, and the ordering is measured rather
	 * than assumed -- I got it wrong in both directions before settling
	 * here.
	 *
	 * It must come AFTER cdn_dp_clock_reset(). Powering the PHY first was
	 * tried, on the theory that the tcphy reset would pull the DPTX's
	 * PHY-facing clocks out from under a running controller; that produced
	 * "failed to wait TCPHY enter A0" and -110 on every attempt. The
	 * dependency runs the other way: DPTX_PHY_DATA_CLK and
	 * DPTX_PHY_CHAR_CLK are what cdn_dp_clock_reset() enables, and the A0
	 * handshake is between the PHY and a DPTX whose clocks are already
	 * running. With clock_reset first, A0 completes.
	 *
	 * Deasserting pipe_rst on the DP path (matching Linux) was also tried
	 * against the A0 timeout and did NOT fix it; it is kept because Linux
	 * does it unconditionally, not because it helped.
	 */
	ret = cdn_dp_ensure_phy(dev);
	if (ret)
		return ret;
	printf("cdn-dp: PHY up on tcphy1, %u lanes, %s orientation\n",
	       priv->lanes, priv->flip ? "flipped" : "normal");

	/*
	 * HPD before host capabilities, as in the reference. It is also the
	 * cheapest probe available: it is the first transaction that needs
	 * BOTH a synced mailbox and the GRF routing done in cdn_dp_get_dp_phy(),
	 * so a failure here separates "the firmware stopped listening" from
	 * "the sink is not reachable" before anything is configured.
	 */
	ret = cdn_dp_get_hpd_status(priv);
	if (ret < 0) {
		log_err("cdn-dp: HPD query failed (%d)\n", ret);
		return ret;
	}
	printf("cdn-dp: HPD %s\n", ret ? "asserted" : "not asserted");

	ret = cdn_dp_set_host_cap(priv);
	if (ret) {
		log_err("cdn-dp: set_host_cap failed (%d)\n", ret);
		return ret;
	}

	/*
	 * Last, and only here: this is the flag that means "everything above
	 * succeeded". Any early return between the fw_loaded announcement and
	 * this line leaves it false, which is exactly the point -- see its
	 * declaration.
	 */
	priv->init_done = true;

	return 0;
}

/**
 * cdn_dp_wait_sink() - wait for the sink to report itself over AUX
 *
 * The first transaction that actually crosses the cable, and therefore the real
 * test of everything above it. A monitor waking from standby is why this has a
 * budget rather than being a single read.
 */
static int cdn_dp_wait_sink(struct rk_cdn_dp_priv *priv)
{
	ulong start = get_timer(0);
	u8 sink_count;
	int ret;

	do {
		ret = cdn_dp_dpcd_read(priv, DP_SINK_COUNT, &sink_count, 1);
		if (!ret && sink_count)
			return 0;
		mdelay(50);
	} while (get_timer(start) < SINK_CONNECT_TIMEOUT_MS);

	log_err("cdn-dp: no sink on AUX after %u ms (last read %d)\n",
		SINK_CONNECT_TIMEOUT_MS, ret);

	return ret ? ret : -ENODEV;
}

/*
 * UCLASS_DISPLAY ops.
 *
 * Every one of these returns success ONLY when the step it names actually
 * completed. If .enable returns 0 while the controller cannot light a link,
 * display_enable() marks the encoder in use, rk_display_init() reports success
 * and rk_vop_probe() stops walking its output ports -- the board comes up with
 * no picture AND no error, which is worse than failing, because failing lets
 * the walk reach an output that works.
 */
static int rk_cdn_dp_read_edid(struct udevice *dev, u8 *buf, int buf_size)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	int ret, len;

	/*
	 * The one place a lazy init is correct: this is the first op the uclass
	 * calls, nothing is configured yet, and the block reset it starts with
	 * has nothing to tear down. .enable deliberately does NOT do this --
	 * see the comment there.
	 *
	 * init_done, not fw_loaded: a previous attempt that got the ucpu
	 * running and then failed at the PHY, the HPD query or set_host_cap
	 * must be RETRIED here, not skipped. fw_loaded would have skipped it.
	 */
	if (!priv->init_done) {
		ret = rk_cdn_dp_fw_init(dev);
		if (ret)
			return ret;
	}

	/*
	 * And the PHY, for the same reason and in the same place. The EDID read
	 * below is an AUX transaction, AUX is lane 8 of the Type-C PHY, and
	 * until this runs there is no analog driver, bias, LDO, resistance
	 * calibration or AUXDA_POLARITY behind it -- so the read cannot
	 * succeed, and the HPD query above answers 0 because GRF_SOC_CON26 has
	 * not routed the connector's HPD into the DPTX either.
	 *
	 * This has to be here rather than left to .enable: .enable runs only
	 * after a timing has already been read.
	 */
	ret = cdn_dp_ensure_phy(dev);
	if (ret)
		return ret;

	/*
	 * A failed HPD QUERY is fatal -- it means the mailbox stopped
	 * answering, and nothing below it can be trusted. A deasserted HPD is
	 * only a warning: the EDID read is the transaction that actually
	 * crosses the cable, so it is the better evidence, and it cannot
	 * produce a false success -- with no sink it fails and this op returns
	 * the error.
	 */
	ret = cdn_dp_get_hpd_status(priv);
	if (ret < 0) {
		log_err("cdn-dp: HPD query failed (%d)\n", ret);
		return ret;
	}
	if (!ret)
		log_debug("cdn-dp: HPD deasserted -- reading EDID anyway\n");

	ret = cdn_dp_read_edid_cached(priv, false);
	if (ret < 0)
		return ret;

	/*
	 * Whole blocks only, and as many as the caller can hold.
	 *
	 * Handing back 128 when 256 are cached is not a harmless truncation:
	 * edid_get_timing_validate() reaches the CTA-861 extension timings only
	 * when buf_size >= EDID_EXT_SIZE, so a short return throws away every
	 * fallback mode the sink has. The uclass passes EDID_EXT_SIZE, so the
	 * full cache normally survives; a smaller caller gets the base block
	 * rather than a partial extension, which would fail the extension's own
	 * checksum in any consumer that checked it.
	 */
	if (buf_size < EDID_BLOCK_SIZE)
		return -ENOSPC;

	len = priv->edid_len;
	if (len > buf_size)
		len = EDID_BLOCK_SIZE;

	memcpy(buf, priv->edid, len);

	/* The byte COUNT, not 0: display_read_timing() passes it on as buf_size. */
	return len;
}

/**
 * rk_cdn_dp_mode_valid() - Linux's cdn_dp_bridge_mode_valid(), cdn-dp-core.c:275
 *
 * Called once per candidate detailed timing from inside
 * edid_get_timing_validate(), so returning false here is how a mode gets
 * skipped in favour of the next one in the EDID rather than programmed and
 * failed later.
 */
static bool rk_cdn_dp_mode_valid(struct udevice *dev,
				 const struct display_timing *timing)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	u32 requested, actual, rate, lanes, source_max, sink_max;
	u8 caps[3];
	u8 bpc;

	/*
	 * Deliberately NO check on uc_plat->source_id here.
	 *
	 * Which VOP may drive a given mode is a scanout question, and it is
	 * already answered generically one layer up: rk_display_init() compares
	 * the timing against rkvop_driverdata.max_output_{width,height} (VOPL
	 * 2560x1600, VOPB 4096x2160) before display_enable() sets in_use, so an
	 * over-large mode leaves the encoder unclaimed for VOPB exactly as a
	 * refusal here would. Refusing every VOPL mode instead would also deny
	 * the ones VOPL CAN scan out -- a 1920x1080 DP sink on a board where
	 * VOPB has already taken HDMI would get no picture at all rather than
	 * the picture VOPL is perfectly able to produce.
	 *
	 * It was also the more fragile of the two spellings: source_id is
	 * ofnode_read_u32_default(remote, "reg", -1), so the literal 1 means
	 * VOPL only because of how rk3399.dtsi happens to number the endpoints.
	 * A DT that deletes dp_in_vopb or leaves an endpoint unnumbered changes
	 * what the constant means without changing this file. max_output_* is
	 * keyed on the VOP's own driver data and cannot drift that way.
	 */
	if (cdn_dp_get_sink_caps(priv, caps)) {
		log_err("cdn-dp: sink capability read failed -- no mode can be validated\n");
		return false;
	}

	bpc = cdn_dp_edid_bpc(priv);

	/*
	 * pixelclock is in Hz here and mode->clock is in kHz in the reference,
	 * hence the extra /1000; the second /1000 turns bits per second into
	 * Mbit/s, matching the budget below.
	 */
	requested = timing->pixelclock.typ / 1000 * bpc * 3 / 1000;

	/*
	 * priv->lanes rather than a literal 4: it is what the Type-C PHY
	 * reported for this pin assignment (4 for C/E, 2 for D/F), and it is
	 * valid because .read_edid has already run fw_init, which is the only
	 * way this op is reached.
	 */
	source_max = priv->lanes;
	sink_max = caps[2] & DP_MAX_LANE_COUNT_MASK;
	lanes = min(source_max, sink_max);

	source_max = cdn_dp_bw_code_to_link_rate(CDN_DP_MAX_LINK_RATE);
	sink_max = cdn_dp_bw_code_to_link_rate(caps[1]);
	rate = min(source_max, sink_max);

	/*
	 * The 0.8 is the reference's derating for 8b/10b coding plus blanking
	 * overhead, and the /100 converts a kHz link rate into Mbit/s per lane
	 * (10 bits per symbol, 1000 kHz per MHz).
	 *
	 * An unrecognised bandwidth code makes cdn_dp_bw_code_to_link_rate()
	 * return 0 rather than the reference's 162000 fallback, so rate -- and
	 * with it the whole budget -- collapses to 0 and every mode is
	 * rejected. That is deliberate and it is fail-safe: rejecting hands the
	 * encoder back to the port walk, whereas inventing a rate would train
	 * against a link nobody described.
	 */
	actual = rate * lanes / 100 * 8 / 10;

	if (!actual || requested > actual) {
		log_debug("cdn-dp: %ux%u needs %u of %u Mbps at %u bpc -- rejected\n",
			  timing->hactive.typ, timing->vactive.typ, requested,
			  actual, bpc);
		return false;
	}

	return true;
}

static int rk_cdn_dp_enable(struct udevice *dev, int panel_bpp,
			    const struct display_timing *timing)
{
	struct display_plat *uc_plat = dev_get_uclass_plat(dev);
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	u8 bits = 0;
	u32 val;
	int ret;

	/*
	 * A hard failure, NOT a lazy init.
	 *
	 * rk_cdn_dp_fw_init() opens with cdn_dp_block_reset() and reloads the
	 * ucpu, which would tear the PHY down in the middle of an enable. It is
	 * also unnecessary: .read_edid is the only route to this op and it has
	 * already run the init, so !init_done here means something reset the
	 * controller since -- a state to report, not to paper over.
	 *
	 * init_done rather than fw_loaded, so that a controller whose ucpu is
	 * running but whose host capabilities were never programmed is refused
	 * instead of having a link trained on it.
	 */
	if (!priv->init_done) {
		log_err("cdn-dp: enable with no completed init -- refusing (re-init here would reset the PHY mid-enable)\n");
		return -ENODEV;
	}

	/*
	 * Route a VOP into the DPTX video interface, cdn-dp-core.c:632. Until
	 * this write nothing feeds the framer, which is precisely why
	 * `cdn_dp video` cannot produce a picture and this op can.
	 *
	 * GRF_SOC_CON9 is write-masked -- the upper half-word selects which
	 * bits the lower half-word applies to -- so this sets or clears
	 * DP_SEL_VOP_LIT without touching the register's other fields and
	 * without a read-modify-write.
	 *
	 * Both arms are live. VOPL driving DP is a legal configuration for any
	 * mode inside its 2560x1600 scanout limit, and rk_display_init()'s
	 * max_output check is what rejects the modes it cannot reach -- before
	 * display_enable() runs, so a mode this encoder must not take never
	 * arrives here at all. Hard-coding the VOPB arm would mux the wrong VOP
	 * into the framer on a board where VOPL legitimately owns the DP port.
	 */
	val = uc_plat->source_id ? (DP_SEL_VOP_LIT | (DP_SEL_VOP_LIT << 16)) :
				   (DP_SEL_VOP_LIT << 16);
	writel(val, priv->grf_base + GRF_SOC_CON9);

	/*
	 * Colour depth from the sink's EDID, not from panel_bpp: panel_bpp is
	 * the VOP framebuffer's bits per PIXEL (32 for the DP path) and
	 * cdn_dp_config_video() wants bits per COMPONENT, which it looks up in
	 * a BCS_* switch that 32 falls straight through to -EINVAL.
	 */
	priv->color_fmt = PXL_RGB;
	priv->color_depth = cdn_dp_edid_bpc(priv);

	ret = cdn_dp_train_link(priv, &bits);
	if (ret) {
		log_err("cdn-dp: link training FAILED (%d), status 0x%02x\n",
			ret, bits);
		return ret;
	}
	/*
	 * Say what was actually observed, not "trained" flat.
	 *
	 * cdn_dp_training_start() returns 0 the moment a polled status byte
	 * carries EQ_PHASE_FINISHED, and @bits is the OR of EVERY status byte
	 * it saw -- so a clock-recovery or EQ failure the firmware raised and
	 * then retried out of is still set here on the success path. The
	 * reference never looks (cdn-dp-core.c), and neither did this, which
	 * made the line claim a clean run the code had not checked for.
	 *
	 * Not a failure: the firmware retries internally and a link that
	 * reached EQ is a trained link. It is worth printing because it is the
	 * first thing to suspect when a link that came up goes on to produce an
	 * unstable picture -- exactly the marginal-cable case that otherwise
	 * gets blamed on the framer.
	 */
	if (bits & (CLK_RECOVERY_FAILED | EQ_PHASE_FAILED | FAST_LT_FAILED))
		printf("cdn-dp: link trained after a retried phase (status 0x%02x), %u kHz, %u lanes, %u bpc\n",
		       bits, priv->max_rate, priv->max_lanes,
		       priv->color_depth);
	else
		printf("cdn-dp: link trained, %u kHz, %u lanes, %u bpc\n",
		       priv->max_rate, priv->max_lanes, priv->color_depth);

	/*
	 * IDLE -> config -> VALID, cdn-dp-core.c:639, :645, :651. The idle
	 * comes first so the framer is not packing a stream out of half-written
	 * timing registers while they are being rewritten.
	 */
	ret = cdn_dp_set_video_status(priv, CONTROL_VIDEO_IDLE);
	if (ret) {
		log_err("cdn-dp: could not idle the video stream (%d)\n", ret);
		return ret;
	}

	ret = cdn_dp_config_video(priv, timing);
	if (ret) {
		log_err("cdn-dp: video configuration FAILED (%d) -- stream left idle\n",
			ret);
		return ret;
	}

	ret = cdn_dp_set_video_status(priv, CONTROL_VIDEO_VALID);
	if (ret) {
		log_err("cdn-dp: SET_VIDEO(VALID) FAILED (%d) -- framer configured but the stream was never declared valid\n",
			ret);
		return ret;
	}

	/*
	 * Reached only when every mailbox request above was accepted. Nothing
	 * in the DPTX register file can be read back to confirm the framer
	 * agreed with any of it (see the opcode block at the top of the file),
	 * so this is the last thing this driver can honestly assert -- the
	 * monitor is the only remaining test.
	 */
	return 0;
}

/**
 * rk_cdn_dp_detect() - is a sink attached to the Type-C connector?
 *
 * Unlike HDMI, where HPD is a pin the host can read at any time, DP's HPD
 * arrives through the ucpu firmware -- so this is only answerable once the
 * firmware is running and the PHY is up. Both are true by the time the uclass
 * asks: display_detect() is called only after a read_timing attempt, and
 * rk_cdn_dp_read_edid() does the lazy fw_init + cdn_dp_ensure_phy() first.
 *
 * The order below is therefore "answer if we can, refuse if we cannot" and
 * never "assume connected": if the init has not completed we return -ENOSYS,
 * which the caller must not read as a sink, rather than 0, which would be an
 * assertion the driver has no evidence for.
 *
 * The test is init_done, not fw_loaded, and the difference is load-bearing.
 * With a running ucpu but no PHY power and no GRF_SOC_CON26 routing, the HPD
 * query below answers 0 for a connector that has a monitor on it -- and
 * rk_display_init() takes a 0 from here as proof the port is empty and
 * suppresses the fallback timing. A half-initialised controller must say "I
 * cannot tell", never "nothing is there".
 */
static int rk_cdn_dp_detect(struct udevice *dev)
{
	struct rk_cdn_dp_priv *priv = dev_get_priv(dev);
	int ret;

	if (!priv->init_done)
		return -ENOSYS;

	ret = cdn_dp_get_hpd_status(priv);
	if (ret < 0)
		return ret;

	return !!ret;
}

static const struct dm_display_ops rk_cdn_dp_ops = {
	.read_edid	= rk_cdn_dp_read_edid,
	.mode_valid	= rk_cdn_dp_mode_valid,
	.detect		= rk_cdn_dp_detect,
	.enable		= rk_cdn_dp_enable,
};

static const struct udevice_id rk_cdn_dp_ids[] = {
	{ .compatible = "rockchip,rk3399-cdn-dp" },
	{ }
};

U_BOOT_DRIVER(cdn_dp_rockchip) = {
	.name		= "cdn_dp_rockchip",
	.id		= UCLASS_DISPLAY,
	.of_match	= rk_cdn_dp_ids,
	.ops		= &rk_cdn_dp_ops,
	.of_to_plat	= rk_cdn_dp_of_to_plat,
	.probe		= rk_cdn_dp_probe,
	.priv_auto	= sizeof(struct rk_cdn_dp_priv),
};

#if IS_ENABLED(CONFIG_CMD_CDN_DP)

static int cdn_dp_get(struct udevice **devp)
{
	int ret;

	/*
	 * by_driver, not by_uclass: this deliberately does NOT go through the
	 * VOP. It probes the DP node and nothing else, so the command can be
	 * run on a board whose HDMI is happily lit and change nothing.
	 */
	ret = uclass_get_device_by_driver(UCLASS_DISPLAY,
					  DM_DRIVER_GET(cdn_dp_rockchip), devp);
	if (ret)
		printf("cdn_dp: no device (%d) -- is the cdn_dp node okay?\n",
		       ret);

	return ret;
}

static int do_cdn_dp_info(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	int ret;

	ret = cdn_dp_get(&dev);
	if (ret)
		return CMD_RET_FAILURE;
	priv = dev_get_priv(dev);

	printf("dev:        %s\n", dev->name);
	printf("regs:       0x%08lx\n", (ulong)dev_read_addr(dev));
	printf("core clk:   %lu Hz%s\n", priv->core_rate,
	       priv->core_rate == CDN_DP_CORE_HZ ? "" : "  <-- NOT 100 MHz");
	printf("PD_HDCP:    %s\n", CONFIG_IS_ENABLED(POWER_DOMAIN) ?
	       "managed" : "unmanaged (no POWER_DOMAIN)");
	printf("APB test:   %s\n", priv->apb_live ? "passed" : "not run / failed");
	printf("FW source:  %s\n",
	       IS_ENABLED(CONFIG_CDN_DP_FW_FIT) ? "FIT node (SPL-decompressed)" :
	       IS_ENABLED(CONFIG_CDN_DP_FW_SCRIPT) ? "env script" : "none");
	if (IS_ENABLED(CONFIG_CDN_DP_FW_FIT))
		printf("FW addr:    0x%08x\n", CDN_DP_FW_FIT_ADDR);
	printf("FW loaded:  %s\n", priv->fw_loaded ? "yes" : "no");
	/*
	 * Separate line because they are separate states: a "yes/no" pair here
	 * is precisely the half-initialised controller the ops now refuse.
	 */
	printf("Init done:  %s\n", priv->init_done ? "yes" : "no");
	if (priv->edid_len)
		printf("EDID:       %d bytes cached\n", priv->edid_len);
	else
		printf("EDID:       not cached\n");
	if (priv->max_rate)
		printf("link:       trained, %u kHz, %u lanes\n", priv->max_rate,
		       priv->max_lanes);
	else
		printf("link:       not trained\n");

	/*
	 * Only meaningful once firmware runs; printed anyway because before
	 * that they are part of the liveness picture -- with the ucpu held in
	 * reset these should read 0, and 0xffffffff here is the same finding
	 * as an all-ones DMEM readback.
	 */
	printf("KEEP_ALIVE: 0x%08x\n", readl(priv->regs + CDN_DP_KEEP_ALIVE));
	printf("VER:        %u.%u\n",
	       readl(priv->regs + CDN_DP_VER_H) & 0xff,
	       readl(priv->regs + CDN_DP_VER_L) & 0xff);

	return CMD_RET_SUCCESS;
}

static int do_cdn_dp_apbtest(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{
	struct udevice *dev;
	int ret;

	ret = cdn_dp_get(&dev);
	if (ret)
		return CMD_RET_FAILURE;

	return rk_cdn_dp_apb_test(dev) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

static int do_cdn_dp_fw(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	struct udevice *dev;
	int ret;

	ret = cdn_dp_get(&dev);
	if (ret)
		return CMD_RET_FAILURE;

	return rk_cdn_dp_fw_init(dev) ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

/*
 * Both of the commands below need a running, ACTIVE ucpu. Refuse early and
 * say why: without firmware the mailbox never empties, so the alternative is
 * the caller sitting through a 5 s MAILBOX_TIMEOUT_US per byte and reading the
 * result as a dead monitor rather than a driver used out of order.
 */
static int cdn_dp_get_live(struct udevice **devp, struct rk_cdn_dp_priv **privp)
{
	int ret;

	ret = cdn_dp_get(devp);
	if (ret)
		return ret;

	*privp = dev_get_priv(*devp);
	if (!(*privp)->fw_loaded) {
		printf("cdn_dp: firmware is not active -- run 'cdn_dp fw' first\n");
		return -ENODEV;
	}

	/*
	 * Every command behind this helper -- hpd, edid, dpcd, train, video --
	 * is about to put a transaction on AUX, so the PHY has to be on. It was
	 * not: cdn_dp_get_dp_phy() was reachable only through .enable, which no
	 * command here goes through. Run standalone at the prompt they all
	 * addressed an unpowered PHY and failed every time, which is
	 * indistinguishable from a flaky AUX channel by looking at the output.
	 * (`cdn_dp sink` is NOT in that list -- it hand-rolls these checks
	 * rather than calling this helper, and carries its own ensure_phy.)
	 *
	 * Failing here when no DisplayPort link can be had is deliberate and is
	 * an improvement: cdn_dp_get_dp_phy() says so and names the command
	 * that shows what the partner actually offered, where the old
	 * behaviour was four retries and -22.
	 */
	ret = cdn_dp_ensure_phy(*devp);
	if (ret)
		return ret;

	return 0;
}

static int do_cdn_dp_hpd(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	int ret;

	if (cdn_dp_get_live(&dev, &priv))
		return CMD_RET_FAILURE;

	ret = cdn_dp_get_hpd_status(priv);
	if (ret < 0) {
		printf("cdn_dp: HPD query failed (%d)\n", ret);
		return CMD_RET_FAILURE;
	}

	printf("HPD:        %s (0x%02x)\n", ret ? "asserted" : "deasserted", ret);
	printf("SW_EVENTS0: 0x%08x\n", readl(priv->regs + CDN_DP_SW_EVENTS0));

	/*
	 * No "this 0 may mean unrouted" advisory here any more, and its removal
	 * is the fix, not a loss of caution. It said the driver did neither
	 * phy_power_on() nor GRF_SOC_CON26 DPTX_HPD_SEL routing, so a 0 could
	 * not be trusted -- and both are now done before this command reads
	 * HPD: cdn_dp_get_live() above calls cdn_dp_ensure_phy(), and
	 * cdn_dp_get_dp_phy() writes DPTX_HPD_SEL right after powering the PHY.
	 * A deasserted HPD at this point means nothing is plugged in, and the
	 * note sent the operator looking for a routing bug that no longer
	 * exists.
	 */
	return CMD_RET_SUCCESS;
}

/*
 * Just enough EDID decode to tell a real block from a plausible-looking one.
 * The checksum is the load-bearing part: a half-transferred AUX read returns
 * bytes that pass the magic-header test, and the manufacturer id it then
 * prints looks like a genuine (wrong) monitor.
 */
static void cdn_dp_print_edid(const u8 *e)
{
	static const u8 magic[8] = { 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0 };
	const u8 *dtd = e + EDID_DTD_OFF;
	u32 hactive, vactive, pixclk;
	u16 mfg;

	if (memcmp(e, magic, sizeof(magic))) {
		printf("EDID: header magic missing -- not an EDID block\n");
		return;
	}

	/*
	 * The same test the cache gates on, not a second copy of it: a block
	 * this prints as "ok" is by construction one cdn_dp_read_edid_cached()
	 * would have stored.
	 */
	printf("EDID: checksum %s\n", cdn_dp_edid_block_ok(e) ? "ok" : "BAD");

	/*
	 * The manufacturer id is three 5-bit letters packed BIG-endian, which
	 * is why this is get_unaligned_be16() while the product code two bytes
	 * later is little-endian. Both come out of a byte buffer with no
	 * alignment guarantee, hence the unaligned accessors either way.
	 */
	mfg = get_unaligned_be16(e + 8);
	printf("  vendor:  %c%c%c\n",
	       '@' + ((mfg >> 10) & 0x1f),
	       '@' + ((mfg >> 5) & 0x1f),
	       '@' + (mfg & 0x1f));
	printf("  product: 0x%04x  serial: 0x%08x\n",
	       get_unaligned_le16(e + 10), get_unaligned_le32(e + 12));
	printf("  version: %u.%u  extensions: %u\n", e[18], e[19],
	       e[EDID_EXT_COUNT_OFF]);

	/*
	 * First detailed timing descriptor = the preferred mode. Pixel clock is
	 * in 10 kHz units, and the active/blank counts are 12-bit values split
	 * across a shared nibble byte. A zero pixel clock means this descriptor
	 * is a monitor-name/range block instead, not a timing.
	 */
	pixclk = get_unaligned_le16(dtd) * 10;
	if (!pixclk) {
		printf("  preferred timing: none (first descriptor is not a DTD)\n");
		return;
	}

	hactive = dtd[2] | ((dtd[4] & 0xf0) << 4);
	vactive = dtd[5] | ((dtd[7] & 0xf0) << 4);
	printf("  preferred timing: %ux%u @ %u kHz pixel clock\n",
	       hactive, vactive, pixclk);
}

static int do_cdn_dp_edid(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	bool force = false;
	bool cached;
	int ret;

	if (argc > 1) {
		if (strcmp(argv[1], "-f"))
			return CMD_RET_USAGE;
		force = true;
	}

	if (cdn_dp_get_live(&dev, &priv))
		return CMD_RET_FAILURE;

	/*
	 * HPD first, and only as a warning. The EDID read is attempted either
	 * way because the HPD path is the less-trusted of the two here (see the
	 * note in 'cdn_dp hpd'), and a successful EDID over a supposedly
	 * deasserted HPD is a far more useful finding than a refusal.
	 */
	ret = cdn_dp_get_hpd_status(priv);
	if (ret <= 0)
		printf("cdn_dp: HPD reads %d -- trying EDID anyway\n", ret);

	/*
	 * Whether this went to the sink is stated rather than left to be
	 * inferred: a cached answer proves nothing about the AUX channel right
	 * now, and reading it as if it did is exactly how a cache hides the
	 * repeat-read instability it exists to work around. `-f` is the way to
	 * ask the hardware again.
	 */
	cached = priv->edid_len && !force;

	ret = cdn_dp_read_edid_cached(priv, force);
	if (ret < 0) {
		printf("cdn_dp: EDID read failed (%d)\n", ret);
		return CMD_RET_FAILURE;
	}

	printf("EDID: %d bytes, %s\n", ret,
	       cached ? "SERVED FROM CACHE (no AUX traffic; use -f to re-read)" :
			"read from the sink");

	print_buffer(0, priv->edid, 1, ret, 16);
	cdn_dp_print_edid(priv->edid);

	return CMD_RET_SUCCESS;
}

/*
 * The DPCD accessors' only caller. They exist because AUX is the layer under
 * EDID and every later step (link status, training, sink power) is a DPCD
 * transaction: when an EDID read fails, the question is immediately "does AUX
 * work at all?", and DPCD 0x00000 (the receiver capability revision) answers it
 * with one byte from a register every DP sink implements.
 */
/*
 * GENERAL_REGISTER_READ (module 0x0a, opcode 0x07): one 32-bit load done BY
 * THE FIRMWARE at a ucpu address. No rk3399 host sends it, which is why the
 * opcode is not among the driver's named ones; the framing is upstream
 * cdns_mhdp_reg_read()'s: a BE32 address in, an 8-byte reply of the BE32
 * address echo then the BE32 value. It is a probe instrument (B42): it shows
 * what the ucpu, not the host's APB, sees in the DP-core register space. The
 * vendor image ignores address 0 (no reply), so 0 is refused here rather than
 * left to time out.
 */
#define GENERAL_REGISTER_READ	0x07

static int cdn_dp_fw_reg_read(struct rk_cdn_dp_priv *priv, u32 addr, u32 *val)
{
	u8 msg[4], reply[8];
	int ret;

	put_unaligned_be32(addr, msg);
	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_GENERAL,
				  GENERAL_REGISTER_READ, sizeof(msg), msg);
	if (ret)
		return log_msg_ret("reg rd send", ret);

	ret = cdn_dp_mailbox_validate_receive(priv, MB_MODULE_ID_GENERAL,
					      GENERAL_REGISTER_READ,
					      sizeof(reply));
	if (ret)
		return log_msg_ret("reg rd hdr", ret);

	ret = cdn_dp_mailbox_read_receive(priv, reply, sizeof(reply));
	if (ret)
		return log_msg_ret("reg rd body", ret);

	if (get_unaligned_be32(reply) != addr) {
		printf("cdn_dp: register read echoed %08x for %08x\n",
		       get_unaligned_be32(reply), addr);
		return -EINVAL;
	}
	*val = get_unaligned_be32(reply + 4);

	return 0;
}

/* Each word is a mailbox round trip (up to the 5 s timeout when unanswered). */
#define CDN_DP_REGREAD_MAX	0x100

static int do_cdn_dp_regread(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	u32 addr, val, n, i;
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	/* Firmware only: no PHY or alt mode is needed for a ucpu-side load. */
	if (cdn_dp_get(&dev))
		return CMD_RET_FAILURE;
	priv = dev_get_priv(dev);
	if (!priv->fw_loaded) {
		printf("cdn_dp: firmware is not active -- run 'cdn_dp fw' first\n");
		return CMD_RET_FAILURE;
	}

	addr = hextoul(argv[1], NULL);
	n = argc > 2 ? hextoul(argv[2], NULL) : 1;
	/* the firmware does one 32-bit load: an unaligned address is refused */
	if (addr & 3) {
		printf("cdn_dp: address %08x is not 4-byte aligned\n", addr);
		return CMD_RET_FAILURE;
	}
	if (n > CDN_DP_REGREAD_MAX) {
		printf("cdn_dp: at most %#x words per call\n", CDN_DP_REGREAD_MAX);
		return CMD_RET_FAILURE;
	}
	for (i = 0; i < n; i++, addr += 4) {
		if (ctrlc())
			return CMD_RET_FAILURE;
		if (!addr) {
			printf("ucpu 00000000: skipped (the firmware does not answer 0)\n");
			continue;
		}
		ret = cdn_dp_fw_reg_read(priv, addr, &val);
		if (ret) {
			/* the stream may be out of step: force 'cdn_dp fw' first */
			priv->fw_loaded = false;
			printf("ucpu %08x: read failed (%d) -- run 'cdn_dp fw' before any other cdn_dp command\n",
			       addr, ret);
			return CMD_RET_FAILURE;
		}
		printf("ucpu %08x: %08x\n", addr, val);
	}

	return CMD_RET_SUCCESS;
}

/*
 * One raw mailbox transaction for the B42 M2 no-sink check: send, then read
 * whatever header comes back and as much payload as it announces (up to
 * @max), printing both. Nothing is validated against an expected size --
 * comparing what the vendor image and the free image actually answer is the
 * point, and a validator would drain and hide exactly that.
 */
static int cdn_dp_raw_xfer(struct rk_cdn_dp_priv *priv, const char *what,
			   u8 module, u8 op, const u8 *req, u16 len, u8 *rsp,
			   u16 max)
{
	u8 hdr[4];
	int ret, i, size;

	ret = cdn_dp_mailbox_send(priv, module, op, len, req);
	if (ret) {
		printf("m2: %-12s send failed (%d)\n", what, ret);
		return ret;
	}
	for (i = 0; i < 4; i++) {
		ret = cdn_dp_mailbox_read(priv);
		if (ret < 0) {
			printf("m2: %-12s no reply (%d)\n", what, ret);
			return ret;
		}
		hdr[i] = ret;
	}
	size = (hdr[2] << 8) | hdr[3];
	printf("m2: %-12s reply op %02x mod %02x size %d:", what, hdr[0], hdr[1],
	       size);
	/*
	 * A reply to some other request means the stream is already out of
	 * step: say so rather than reading its payload as ours.
	 */
	if (hdr[0] != op || hdr[1] != module) {
		printf(" <not the reply to op %02x mod %02x>\n", op, module);
		return -EBADMSG;
	}
	for (i = 0; i < size; i++) {
		ret = cdn_dp_mailbox_read(priv);
		if (ret < 0) {
			printf(" <read %d failed (%d)>\n", i, ret);
			return ret;
		}
		if (i < max)
			rsp[i] = ret;
		if (i < 16)
			printf(" %02x", ret);
	}
	printf("%s\n", size > 16 ? " ..." : "");

	return size;
}

/* WRITE_REGISTER / WRITE_FIELD have no reply: send only. */
static int cdn_dp_raw_send(struct rk_cdn_dp_priv *priv, const char *what,
			   u8 op, const u8 *req, u16 len)
{
	int ret;

	ret = cdn_dp_mailbox_send(priv, MB_MODULE_ID_DP_TX, op, len, req);
	printf("m2: %-12s sent (%d)\n", what, ret);

	return ret;
}

#define M2_SCRATCH	0x2280	/* MSA register: harmless with no stream */

/* m2nosink's WRITE_FIELD: addr BE16, start, nbits, value BE32 -- then read back */
static int m2_field(struct rk_cdn_dp_priv *priv, const char *what, u8 start,
		    u8 nbits, u32 value)
{
	u8 req[8];
	u32 val;

	put_unaligned_be16(M2_SCRATCH, req);
	req[2] = start;
	req[3] = nbits;
	put_unaligned_be32(value, req + 4);
	if (cdn_dp_raw_send(priv, "WRITE_FIELD", DPTX_WRITE_FIELD, req, 8) ||
	    cdn_dp_fw_reg_read(priv, M2_SCRATCH, &val))
		return -EIO;
	printf("m2: scratch %04x = %08x after WRITE_FIELD %s\n", M2_SCRATCH,
	       val, what);
	return 0;
}

/*
 * WRITE_REGISTER, read back, and the value must be the one written. -EIO is a
 * failed transaction (the stream may be out of step); -EINVAL is a completed
 * one that read back the wrong value (the stream is fine).
 */
static int m2_reg(struct rk_cdn_dp_priv *priv, const char *what, u32 value)
{
	u8 req[6];
	u32 val;

	put_unaligned_be16(M2_SCRATCH, req);
	put_unaligned_be32(value, req + 2);
	if (cdn_dp_raw_send(priv, "WRITE_REG", DPTX_WRITE_REGISTER, req, 6) ||
	    cdn_dp_fw_reg_read(priv, M2_SCRATCH, &val))
		return -EIO;
	printf("m2: scratch %04x = %08x %s\n", M2_SCRATCH, val, what);
	if (val != value) {
		printf("m2: WRITE_REGISTER did not take (wanted %08x)\n", value);
		return -EINVAL;
	}
	return 0;
}

/*
 * The plan's M2 acceptance without a sink, as raw transactions: HPD_STATE,
 * READ_DPCD, WRITE_DPCD and GET_EDID with nothing plugged in, then a
 * WRITE_REGISTER and WRITE_FIELDs to a scratch DP-core register, each read
 * back through GENERAL_REGISTER_READ, and the register restored.
 *
 * Refused with a sink attached (it would write DPCD 0x600 and the live MSA).
 * Stops at the first failed transaction: a reply that never came, or came
 * part-way, can leave bytes in the mailbox stream that the next transaction
 * would read as its header, so the session is reported unusable instead.
 */
static int do_cdn_dp_m2nosink(struct cmd_tbl *cmdtp, int flag, int argc,
			      char *const argv[])
{
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	u8 req[8], rsp[140];
	u32 orig;
	int ret;

	if (cdn_dp_get(&dev))
		return CMD_RET_FAILURE;
	priv = dev_get_priv(dev);
	if (!priv->fw_loaded) {
		printf("cdn_dp: firmware is not active -- run 'cdn_dp fw' first\n");
		return CMD_RET_FAILURE;
	}

	ret = cdn_dp_raw_xfer(priv, "HPD_STATE", MB_MODULE_ID_DP_TX,
			      DPTX_HPD_STATE, NULL, 0, rsp, sizeof(rsp));
	if (ret < 0)
		goto desync;
	/* fail closed: only an explicit "no sink" lets the writes below run */
	if (ret < 1 || rsp[0]) {
		printf("m2: no explicit HPD 0 (size %d, HPD %02x) -- this check is for no sink\n",
		       ret, ret >= 1 ? rsp[0] : 0xff);
		return CMD_RET_FAILURE;
	}

	req[0] = 0; req[1] = 1;			/* len 1 */
	req[2] = 0; req[3] = 0; req[4] = 0;	/* DPCD 0x00000 */
	if (cdn_dp_raw_xfer(priv, "READ_DPCD", MB_MODULE_ID_DP_TX,
			    DPTX_READ_DPCD, req, 5, rsp, sizeof(rsp)) < 0)
		goto desync;

	req[0] = 0; req[1] = 1;			/* len 1 */
	req[2] = 0; req[3] = 0x06; req[4] = 0;	/* DPCD 0x00600 (SET_POWER) */
	req[5] = 1;				/* D0 */
	if (cdn_dp_raw_xfer(priv, "WRITE_DPCD", MB_MODULE_ID_DP_TX,
			    DPTX_WRITE_DPCD, req, 6, rsp, sizeof(rsp)) < 0)
		goto desync;

	req[0] = 0; req[1] = 0;			/* block 0 */
	if (cdn_dp_raw_xfer(priv, "GET_EDID", MB_MODULE_ID_DP_TX,
			    DPTX_GET_EDID, req, 2, rsp, sizeof(rsp)) < 0)
		goto desync;

	if (cdn_dp_fw_reg_read(priv, M2_SCRATCH, &orig))
		goto desync;
	printf("m2: scratch %04x = %08x before\n", M2_SCRATCH, orig);

	ret = m2_reg(priv, "after WRITE_REGISTER 12345678", 0x12345678);
	if (ret)
		goto mismatch;
	/*
	 * Linux's own call form, cdn_dp_reg_write_bit(DP_VB_ID, 4, 1, enable):
	 * start 4, 1 bit, value 1 unshifted -- run on 0x12345678, whose bit 4
	 * is 1, so "clears bit 4" (12345668) and "no effect" (12345678) differ.
	 */
	if (m2_field(priv, "(start 4, 1 bit, value 1: Linux's form)", 4, 1, 1))
		goto desync;
	ret = m2_reg(priv, "after WRITE_REGISTER 12345678", 0x12345678);
	if (ret)
		goto mismatch;
	/*
	 * Field bits 7..4 <- 0xa as the vendor image reads the request:
	 * start = the field's MSB, value in position (measured: 12345678 ->
	 * 123456a8 on vendor 4.117 and free 0.11 alike).
	 */
	if (m2_field(priv, "[7:4] <- a0 (start 7, 4 bits)", 7, 4, 0xa0))
		goto desync;
	ret = m2_reg(priv, "restored", orig);
	if (ret == -EIO)
		goto desync;

	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;

mismatch:
	if (ret == -EIO)
		goto desync;
	/* the stream is in step: put the register back, then report */
	ret = m2_reg(priv, "restored", orig);
	if (ret == -EIO)
		goto desync;
	printf("m2: WRITE_REGISTER read back a different value; %s\n",
	       ret ? "the restore did not take either" : "register restored");
	return CMD_RET_FAILURE;

desync:
	/* force a reload: cdn_dp_get_live() and the probes refuse until 'cdn_dp fw' */
	priv->fw_loaded = false;
	printf("m2: a transaction failed; the mailbox stream may be out of step -- run 'cdn_dp fw' before any other cdn_dp command\n");
	return CMD_RET_FAILURE;
}

static int do_cdn_dp_dpcd(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	u32 addr;
	u8 val;
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (cdn_dp_get_live(&dev, &priv))
		return CMD_RET_FAILURE;

	addr = hextoul(argv[1], NULL);
	if (addr > 0xfffff) {
		printf("cdn_dp: DPCD addresses are 20-bit\n");
		return CMD_RET_FAILURE;
	}

	if (argc > 2) {
		val = hextoul(argv[2], NULL);
		ret = cdn_dp_dpcd_write(priv, addr, val);
		if (ret) {
			printf("cdn_dp: DPCD write failed (%d)\n", ret);
			return CMD_RET_FAILURE;
		}
		printf("DPCD %05x <- %02x\n", addr, val);

		return CMD_RET_SUCCESS;
	}

	ret = cdn_dp_dpcd_read(priv, addr, &val, sizeof(val));
	if (ret) {
		printf("cdn_dp: DPCD read failed (%d)\n", ret);
		return CMD_RET_FAILURE;
	}
	printf("DPCD %05x -> %02x\n", addr, val);

	return CMD_RET_SUCCESS;
}

/*
 * The end-to-end test: wait for the sink over AUX, then read DPCD 0x000-0x00f.
 * If this prints a DPCD revision, every layer below it worked -- firmware,
 * mailbox, PHY, HPD routing, lane mapping and the cable.
 */
static int do_cdn_dp_sink(struct cmd_tbl *cmdtp, int flag, int argc,
			  char *const argv[])
{
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	u8 dpcd[16];
	int ret, i;

	ret = cdn_dp_get(&dev);
	if (ret)
		return CMD_RET_FAILURE;
	priv = dev_get_priv(dev);

	if (!priv->fw_loaded) {
		printf("cdn_dp: firmware not loaded -- run `cdn_dp fw` first\n");
		return CMD_RET_FAILURE;
	}

	/*
	 * This command hand-rolls the checks cdn_dp_get_live() does instead of
	 * calling it, so it did not inherit the PHY power-on and was still
	 * putting DPCD reads on an unpowered AUX channel. That matters more
	 * here than anywhere else: `cdn_dp sink` is the command you reach for
	 * to decide whether AUX is marginal, so it was the one most likely to
	 * manufacture the "flaky AUX" reading that turned out to be this bug.
	 */
	ret = cdn_dp_ensure_phy(dev);
	if (ret)
		return CMD_RET_FAILURE;

	ret = cdn_dp_wait_sink(priv);
	if (ret)
		return CMD_RET_FAILURE;
	printf("sink present\n");

	ret = cdn_dp_dpcd_read(priv, DP_DPCD_REV, dpcd, sizeof(dpcd));
	if (ret) {
		printf("DPCD read failed: %d\n", ret);
		return CMD_RET_FAILURE;
	}

	printf("DPCD 0x000:");
	for (i = 0; i < sizeof(dpcd); i++)
		printf(" %02x", dpcd[i]);
	printf("\n");
	printf("DPCD rev:   %u.%u\n", dpcd[0] >> 4, dpcd[0] & 0xf);
	printf("max rate:   0x%02x (%u lanes advertised)\n", dpcd[1],
	       dpcd[2] & 0x1f);

	return CMD_RET_SUCCESS;
}

/*
 * Decode the training status byte. Printed on success as well as on failure:
 * a link that came up with CLK_RECOVERY_FAILED seen along the way is a link
 * that retried, which is worth knowing about a cable before it is blamed for
 * something else.
 */
static void cdn_dp_print_train_status(u8 bits)
{
	printf("status:     0x%02x%s%s%s%s%s%s%s%s\n", bits,
	       bits & FULL_LT_STARTED ? " full-started" : "",
	       bits & FAST_LT_STARTED ? " fast-started" : "",
	       bits & CLK_RECOVERY_FINISHED ? " cr-done" : "",
	       bits & EQ_PHASE_FINISHED ? " eq-done" : "",
	       bits & FAST_LT_START_FINISHED ? " fast-done" : "",
	       bits & CLK_RECOVERY_FAILED ? " CR-FAILED" : "",
	       bits & EQ_PHASE_FAILED ? " EQ-FAILED" : "",
	       bits & FAST_LT_FAILED ? " FAST-FAILED" : "");
}

/**
 * cdn_dp_bringup() - train to the sink's preferred mode and load that timing
 *
 * Everything `train` and `video` have in common, up to and including
 * cdn_dp_config_video(), and stopping with the stream IDLE. Whether
 * CONTROL_VIDEO_VALID follows is the caller's decision and the only difference
 * between the two commands.
 *
 * Prints each step's result as it goes rather than only at the end, because
 * every step here is somewhere bring-up stops and WHICH one it stopped at is
 * the entire diagnostic value of the command.
 *
 * Returns a negative errno, which the caller turns into CMD_RET_FAILURE. On
 * success @timing holds the mode that was programmed.
 */
static int cdn_dp_bringup(struct rk_cdn_dp_priv *priv,
			  struct display_timing *timing)
{
	u32 requested, budget;
	u8 sink_caps[3];
	int edid_bpc, ret;
	u8 bits = 0;
	u8 bpc;

	ret = cdn_dp_wait_sink(priv);
	if (ret)
		return ret;

	/*
	 * The sink's own limits, printed before training rather than after, so
	 * a link that trains DOWN can be told apart from one that trained to
	 * everything the sink offered.
	 */
	ret = cdn_dp_get_sink_caps(priv, sink_caps);
	if (ret) {
		printf("cdn_dp: DPCD capability read failed (%d)\n", ret);
		return ret;
	}
	printf("sink max:   0x%02x (%u kHz), %u lanes\n", sink_caps[1],
	       cdn_dp_bw_code_to_link_rate(sink_caps[1]),
	       sink_caps[2] & DP_MAX_LANE_COUNT_MASK);
	printf("source max: 0x%02x (%u kHz), %u lanes\n", CDN_DP_MAX_LINK_RATE,
	       cdn_dp_bw_code_to_link_rate(CDN_DP_MAX_LINK_RATE), priv->lanes);

	/*
	 * Through the cache, so a `cdn_dp video` run straight after a
	 * `cdn_dp edid` does not re-read the sink -- the repeat read is the one
	 * that fails. The bytes are handed to edid_get_timing() with the LENGTH
	 * the cache holds so that, when the extension block read cleanly, the
	 * CTA-861 detailed timings are candidates too.
	 */
	ret = cdn_dp_read_edid_cached(priv, false);
	if (ret < 0) {
		printf("cdn_dp: EDID read failed (%d)\n", ret);
		return ret;
	}

	ret = edid_get_timing(priv->edid, ret, timing, &edid_bpc);
	if (ret) {
		printf("cdn_dp: no usable detailed timing in EDID (%d)\n", ret);
		return ret;
	}

	/*
	 * The depth the PRODUCTION path will program, from the one derivation
	 * this driver has -- not edid_get_timing()'s out-parameter.
	 *
	 * This command exists to be quoted as evidence, so it must report what
	 * .enable would do, and .enable takes cdn_dp_edid_bpc(). The two
	 * disagree in both directions: edid_get_timing() returns -1 for "not
	 * stated" and 4 for the EDID 1.4 "undefined" encoding (its formula is
	 * ((video_input_definition & 0x70) >> 3) + 4, so encoding 0 comes out
	 * as 4, not as the -1 the caller is told to expect -- and 4 is not a DP
	 * depth, BCS_6 being the smallest), while it passes 12 and 16 straight
	 * through where cdn_dp_edid_bpc() maps them to 8 as Linux's mode_valid
	 * does. On a 12-bpc EDID the old code here programmed BCS_12 and
	 * printed a bandwidth figure 1.5x larger than the one .enable computes
	 * from BCS_8 -- a number no run of the real path would ever produce.
	 *
	 * The raw value is still read and printed when it differs, because the
	 * difference is diagnostic: it is the sink asking for a depth this
	 * driver has decided not to give it.
	 */
	bpc = cdn_dp_edid_bpc(priv);
	if (edid_bpc != bpc)
		printf("cdn_dp: EDID states %d bpc, driving %u (the depth .enable programs)\n",
		       edid_bpc, bpc);

	priv->color_fmt = PXL_RGB;
	priv->color_depth = bpc;

	/*
	 * Polarity printed through cdn_dp_sync_negative(), the same call
	 * cdn_dp_config_video() makes, so the line a human reads cannot
	 * disagree with the bit the framer was given -- including for a timing
	 * that states no polarity at all, where a bare HIGH test would print
	 * the opposite of what is programmed.
	 */
	printf("mode:       %ux%u @ %u Hz pixel clock, %u bpc, %csync %csync\n",
	       timing->hactive.typ, timing->vactive.typ,
	       timing->pixelclock.typ, bpc,
	       cdn_dp_sync_negative(timing->flags, DISPLAY_FLAGS_HSYNC_HIGH,
				    DISPLAY_FLAGS_HSYNC_LOW) ? '-' : '+',
	       cdn_dp_sync_negative(timing->flags, DISPLAY_FLAGS_VSYNC_HIGH,
				    DISPLAY_FLAGS_VSYNC_LOW) ? '-' : '+');

	ret = cdn_dp_train_link(priv, &bits);
	if (ret) {
		printf("cdn_dp: LINK TRAINING FAILED (%d)\n", ret);
		cdn_dp_print_train_status(bits);
		if (ret == -ETIMEDOUT && !bits)
			printf("note: the firmware reported no training activity at all --\n"
			       "      suspect the host-capability handshake or the PHY, not the cable.\n");
		return ret;
	}

	printf("trained:    %u kHz, %u lanes\n", priv->max_rate,
	       priv->max_lanes);
	cdn_dp_print_train_status(bits);

	/*
	 * Linux's mode_valid arithmetic, run against what was ACTUALLY trained
	 * rather than against the maxima: bandwidth needed vs bandwidth
	 * available, derated by the 0.8 the reference uses for 8b/10b coding
	 * plus blanking overhead. A mode over budget is reported and the
	 * configuration is still attempted -- the TU search below is the real
	 * arbiter, and a refusal here would hide its (more precise) verdict.
	 */
	requested = timing->pixelclock.typ / 1000 * bpc * 3 / 1000;
	budget = priv->max_rate / 100 * priv->max_lanes * 8 / 10;
	printf("bandwidth:  %u of %u Mbps available (%s)\n", requested, budget,
	       requested > budget ? "OVER BUDGET" : "fits");

	/*
	 * IDLE -> config -> VALID is the reference's order, cdn-dp-core.c:639,
	 * :645, :651. The idle comes first because the framer must not be
	 * packing a stream out of half-written timing registers while they are
	 * being rewritten.
	 */
	ret = cdn_dp_set_video_status(priv, CONTROL_VIDEO_IDLE);
	if (ret) {
		printf("cdn_dp: could not idle the video stream (%d)\n", ret);
		return ret;
	}

	ret = cdn_dp_config_video(priv, timing);
	if (ret) {
		printf("cdn_dp: video configuration FAILED (%d)\n", ret);
		return ret;
	}

	printf("tu size:    %u, %u.%03u valid symbols per TU\n", priv->tu_size,
	       priv->valid_symbols, priv->valid_symbols_milli);
	printf("video:      framer and MSA programmed, stream still IDLE\n");

	return 0;
}

/*
 * Train the link against the sink's preferred mode, then program the framer for
 * that mode -- and stop there.
 *
 * What this command deliberately does NOT do is issue CONTROL_VIDEO_VALID or
 * touch GRF_SOC_CON9's DP_SEL_VOP_LIT. Nothing in U-Boot is driving pixels into
 * the DPTX's video interface yet, so declaring the stream valid would tell the
 * framer to start packing a stream that does not exist, and routing a VOP here
 * would take that VOP away from the HDMI output that currently works. Both are
 * one line each, and both belong in the change that wires up the VOP -- not in
 * the one that can only print an unverifiable success.
 */
static int do_cdn_dp_train(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	struct display_timing timing;
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;

	if (cdn_dp_get_live(&dev, &priv))
		return CMD_RET_FAILURE;

	if (cdn_dp_bringup(priv, &timing))
		return CMD_RET_FAILURE;

	printf("note: the link is trained and the timing is loaded, but nothing is\n"
	       "      feeding pixels -- no VOP is routed here and CONTROL_VIDEO_VALID\n"
	       "      was not issued. This is NOT a picture, and the register writes\n"
	       "      cannot be read back to confirm they took effect.\n");

	return CMD_RET_SUCCESS;
}

/*
 * The whole DPTX-side video sequence: train, IDLE, config_video, VALID. `train`
 * stops one step short of this on purpose; this is the step that tells the
 * framer to start packing.
 *
 * It still does not produce a picture, and the closing note says so in those
 * words, because a clean run of a command called "video" is exactly the output
 * somebody will quote as evidence that video works. What is missing is the
 * pixel source: no VOP is routed to the DPTX (GRF_SOC_CON9's DP_SEL_VOP_LIT is
 * untouched here -- that routing belongs to rk_cdn_dp_enable(), which is
 * reached through the VOP's own port walk with a framebuffer behind it, and
 * doing it from here would take a VOP away from the HDMI output that currently
 * works), so the framer is packing whatever the idle video interface presents.
 *
 * Nothing in this command can be verified from this side either. Every register
 * it programmed lives behind the ucpu's mailbox and there is no DPTX register
 * read to confirm any of them (see the opcode block at the top of the file), so
 * success here means every request was ACCEPTED, not that any of it took
 * effect. Only a monitor lighting up can say that.
 */
static int do_cdn_dp_video(struct cmd_tbl *cmdtp, int flag, int argc,
			   char *const argv[])
{
	struct display_timing timing;
	struct rk_cdn_dp_priv *priv;
	struct udevice *dev;
	int ret;

	if (cdn_dp_get_live(&dev, &priv))
		return CMD_RET_FAILURE;

	ret = cdn_dp_bringup(priv, &timing);
	if (ret) {
		printf("cdn_dp: VIDEO BRING-UP FAILED (%d) -- stream left idle\n",
		       ret);
		return CMD_RET_FAILURE;
	}

	ret = cdn_dp_set_video_status(priv, CONTROL_VIDEO_VALID);
	if (ret) {
		printf("cdn_dp: SET_VIDEO(VALID) FAILED (%d) -- the framer was\n"
		       "        configured but never told the stream is valid\n",
		       ret);
		return CMD_RET_FAILURE;
	}

	printf("link:       %u kHz, %u lanes\n", priv->max_rate,
	       priv->max_lanes);
	printf("video:      stream declared VALID\n");
	printf("NOT A PICTURE: no VOP is routed into the DPTX video interface, so\n"
	       "      the framer is packing an idle input. None of the registers\n"
	       "      above can be read back, so this says the firmware ACCEPTED\n"
	       "      the configuration -- not that it is producing a stream.\n");

	return CMD_RET_SUCCESS;
}

static struct cmd_tbl cdn_dp_subcmd[] = {
	U_BOOT_CMD_MKENT(info, 1, 1, do_cdn_dp_info, "", ""),
	U_BOOT_CMD_MKENT(apbtest, 1, 1, do_cdn_dp_apbtest, "", ""),
	U_BOOT_CMD_MKENT(fw, 1, 1, do_cdn_dp_fw, "", ""),
	U_BOOT_CMD_MKENT(hpd, 1, 1, do_cdn_dp_hpd, "", ""),
	U_BOOT_CMD_MKENT(edid, 2, 1, do_cdn_dp_edid, "", ""),
	U_BOOT_CMD_MKENT(dpcd, 3, 1, do_cdn_dp_dpcd, "", ""),
	U_BOOT_CMD_MKENT(regread, 3, 1, do_cdn_dp_regread, "", ""),
	U_BOOT_CMD_MKENT(m2nosink, 1, 1, do_cdn_dp_m2nosink, "", ""),
	U_BOOT_CMD_MKENT(sink, 1, 1, do_cdn_dp_sink, "", ""),
	U_BOOT_CMD_MKENT(train, 1, 1, do_cdn_dp_train, "", ""),
	U_BOOT_CMD_MKENT(video, 1, 1, do_cdn_dp_video, "", ""),
};

static int do_cdn_dp(struct cmd_tbl *cmdtp, int flag, int argc,
		     char *const argv[])
{
	struct cmd_tbl *cmd;

	argc--;
	argv++;

	if (argc < 1)
		return CMD_RET_USAGE;

	cmd = find_cmd_tbl(argv[0], cdn_dp_subcmd, ARRAY_SIZE(cdn_dp_subcmd));
	if (!cmd || argc > cmd->maxargs)
		return CMD_RET_USAGE;

	return cmd->cmd(cmdtp, flag, argc, argv);
}

U_BOOT_CMD(cdn_dp, CONFIG_SYS_MAXARGS, 1, do_cdn_dp,
	   "Cadence MHDP DisplayPort bring-up",
	   "info     - probe the controller and show clock/reset/APB state\n"
	   "cdn_dp apbtest  - DMEM write/readback; also the PD_HDCP test (refused while firmware runs)\n"
	   "cdn_dp fw       - load the ucpu firmware and take it out of standby\n"
	   "cdn_dp hpd      - ask the firmware whether a sink is plugged in\n"
	   "cdn_dp edid [-f] - show the cached EDID (-f: re-read it over AUX)\n"
	   "cdn_dp dpcd <addr> [val] - read (or write) one DPCD byte over AUX\n"
	   "cdn_dp regread <addr> [n] - n (hex, max 0x100) 32-bit loads done by\n"
	   "                  the firmware at ucpu addresses (GENERAL_REGISTER_READ)\n"
	   "cdn_dp m2nosink - raw M2 mailbox checks that need no sink\n"
	   "cdn_dp sink     - wait for the sink on AUX and dump its DPCD header\n"
	   "cdn_dp train    - train the link to the EDID's preferred mode and\n"
	   "                  load that timing; leaves the stream IDLE\n"
	   "cdn_dp video    - train, configure and declare the stream VALID;\n"
	   "                  still no picture -- no VOP feeds the DPTX"
);

#endif /* CONFIG_CMD_CDN_DP */
