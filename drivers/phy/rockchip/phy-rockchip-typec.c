// SPDX-License-Identifier: GPL-2.0-only
/*
 * ROCKCHIP Type-C PHY driver.
 *
 * Copyright (C) 2020 Amarula Solutions(India)
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Chris Zhong <zyw@rock-chips.com>
 *         Kever Yang <kever.yang@rock-chips.com>
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <dm/ofnode_graph.h>
#include <dm/uclass-internal.h>
#include <generic-phy.h>
#include <i2c.h>
#include <reset.h>
#include <syscon.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include <linux/iopoll.h>
#include <asm/arch-rockchip/clock.h>
#include <phy/rockchip-typec.h>
#if CONFIG_IS_ENABLED(TYPEC_TCPM)
#include <usb/tcpm.h>
#include <usb/typec_dp.h>
#elif CONFIG_IS_ENABLED(TYPEC_FUSB302_CC)
#include <usb/tcpm.h>
#endif

#define usleep_range(a, b) udelay((b))

#define CMN_SSM_BANDGAP			(0x21 << 2)
#define CMN_SSM_BIAS			(0x22 << 2)
#define CMN_PLLSM0_PLLEN		(0x29 << 2)
#define CMN_PLLSM0_PLLPRE		(0x2a << 2)
#define CMN_PLLSM0_PLLVREF		(0x2b << 2)
#define CMN_PLLSM0_PLLLOCK		(0x2c << 2)
#define CMN_PLLSM1_PLLEN		(0x31 << 2)
#define CMN_PLLSM1_PLLPRE		(0x32 << 2)
#define CMN_PLLSM1_PLLVREF		(0x33 << 2)
#define CMN_PLLSM1_PLLLOCK		(0x34 << 2)
#define CMN_PLLSM1_USER_DEF_CTRL	(0x37 << 2)
#define CMN_ICAL_OVRD			(0xc1 << 2)
#define CMN_PLL0_VCOCAL_OVRD		(0x83 << 2)
#define CMN_PLL0_VCOCAL_INIT		(0x84 << 2)
#define CMN_PLL0_VCOCAL_ITER		(0x85 << 2)
#define CMN_PLL0_LOCK_REFCNT_START	(0x90 << 2)
#define CMN_PLL0_LOCK_PLLCNT_START	(0x92 << 2)
#define CMN_PLL0_LOCK_PLLCNT_THR	(0x93 << 2)
#define CMN_PLL0_INTDIV			(0x94 << 2)
#define CMN_PLL0_FRACDIV		(0x95 << 2)
#define CMN_PLL0_HIGH_THR		(0x96 << 2)
#define CMN_PLL0_DSM_DIAG		(0x97 << 2)
#define CMN_PLL0_SS_CTRL1		(0x98 << 2)
#define CMN_PLL0_SS_CTRL2		(0x99 << 2)
#define CMN_PLL1_VCOCAL_START		(0xa1 << 2)
#define CMN_PLL1_VCOCAL_OVRD		(0xa3 << 2)
#define CMN_PLL1_VCOCAL_INIT		(0xa4 << 2)
#define CMN_PLL1_VCOCAL_ITER		(0xa5 << 2)
#define CMN_PLL1_LOCK_REFCNT_START	(0xb0 << 2)
#define CMN_PLL1_LOCK_PLLCNT_START	(0xb2 << 2)
#define CMN_PLL1_LOCK_PLLCNT_THR	(0xb3 << 2)
#define CMN_PLL1_INTDIV			(0xb4 << 2)
#define CMN_PLL1_FRACDIV		(0xb5 << 2)
#define CMN_PLL1_HIGH_THR		(0xb6 << 2)
#define CMN_PLL1_DSM_DIAG		(0xb7 << 2)
#define CMN_PLL1_SS_CTRL1		(0xb8 << 2)
#define CMN_PLL1_SS_CTRL2		(0xb9 << 2)
#define CMN_RXCAL_OVRD			(0xd1 << 2)

#define CMN_TXPUCAL_CTRL		(0xe0 << 2)
#define CMN_TXPUCAL_OVRD		(0xe1 << 2)
#define CMN_TXPDCAL_CTRL		(0xf0 << 2)
#define CMN_TXPDCAL_OVRD		(0xf1 << 2)

/* For CMN_TXPUCAL_CTRL, CMN_TXPDCAL_CTRL */
#define CMN_TXPXCAL_START		BIT(15)
#define CMN_TXPXCAL_DONE		BIT(14)
#define CMN_TXPXCAL_NO_RESPONSE		BIT(13)
#define CMN_TXPXCAL_CURRENT_RESPONSE	BIT(12)

#define CMN_TXPU_ADJ_CTRL		(0x108 << 2)
#define CMN_TXPD_ADJ_CTRL		(0x10c << 2)

/*
 * For CMN_TXPUCAL_CTRL, CMN_TXPDCAL_CTRL,
 *     CMN_TXPU_ADJ_CTRL, CMN_TXPDCAL_CTRL
 *
 * NOTE: some of these registers are documented to be 2's complement
 * signed numbers, but then documented to be always positive.  Weird.
 * In such a case, using CMN_CALIB_CODE_POS() avoids the unnecessary
 * sign extension.
 */
#define CMN_CALIB_CODE_WIDTH	7
#define CMN_CALIB_CODE_OFFSET	0
#define CMN_CALIB_CODE_MASK	GENMASK(CMN_CALIB_CODE_WIDTH, 0)
#define CMN_CALIB_CODE(x)	\
	sign_extend32((x) >> CMN_CALIB_CODE_OFFSET, CMN_CALIB_CODE_WIDTH)

#define CMN_CALIB_CODE_POS_MASK	GENMASK(CMN_CALIB_CODE_WIDTH - 1, 0)
#define CMN_CALIB_CODE_POS(x)	\
	(((x) >> CMN_CALIB_CODE_OFFSET) & CMN_CALIB_CODE_POS_MASK)

#define CMN_DIAG_PLL0_FBH_OVRD		(0x1c0 << 2)
#define CMN_DIAG_PLL0_FBL_OVRD		(0x1c1 << 2)
#define CMN_DIAG_PLL0_OVRD		(0x1c2 << 2)
#define CMN_DIAG_PLL0_V2I_TUNE		(0x1c5 << 2)
#define CMN_DIAG_PLL0_CP_TUNE		(0x1c6 << 2)
#define CMN_DIAG_PLL0_LF_PROG		(0x1c7 << 2)
#define CMN_DIAG_PLL1_FBH_OVRD		(0x1d0 << 2)
#define CMN_DIAG_PLL1_FBL_OVRD		(0x1d1 << 2)
#define CMN_DIAG_PLL1_OVRD		(0x1d2 << 2)
#define CMN_DIAG_PLL1_V2I_TUNE		(0x1d5 << 2)
#define CMN_DIAG_PLL1_CP_TUNE		(0x1d6 << 2)
#define CMN_DIAG_PLL1_LF_PROG		(0x1d7 << 2)
#define CMN_DIAG_PLL1_PTATIS_TUNE1	(0x1d8 << 2)
#define CMN_DIAG_PLL1_PTATIS_TUNE2	(0x1d9 << 2)
#define CMN_DIAG_PLL1_INCLK_CTRL	(0x1da << 2)
#define CMN_DIAG_HSCLK_SEL		(0x1e0 << 2)

#define XCVR_PSM_RCTRL(n)		((0x4001 | ((n) << 9)) << 2)
#define XCVR_PSM_CAL_TMR(n)		((0x4002 | ((n) << 9)) << 2)
#define XCVR_PSM_A0IN_TMR(n)		((0x4003 | ((n) << 9)) << 2)
#define TX_TXCC_CAL_SCLR_MULT(n)	((0x4047 | ((n) << 9)) << 2)
#define TX_TXCC_CPOST_MULT_00(n)	((0x404c | ((n) << 9)) << 2)
#define TX_TXCC_CPOST_MULT_01(n)	((0x404d | ((n) << 9)) << 2)
#define TX_TXCC_CPOST_MULT_10(n)	((0x404e | ((n) << 9)) << 2)
#define TX_TXCC_CPOST_MULT_11(n)	((0x404f | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_000(n)	((0x4050 | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_001(n)	((0x4051 | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_010(n)	((0x4052 | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_011(n)	((0x4053 | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_100(n)	((0x4054 | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_101(n)	((0x4055 | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_110(n)	((0x4056 | ((n) << 9)) << 2)
#define TX_TXCC_MGNFS_MULT_111(n)	((0x4057 | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_000(n)	((0x4058 | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_001(n)	((0x4059 | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_010(n)	((0x405a | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_011(n)	((0x405b | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_100(n)	((0x405c | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_101(n)	((0x405d | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_110(n)	((0x405e | ((n) << 9)) << 2)
#define TX_TXCC_MGNLS_MULT_111(n)	((0x405f | ((n) << 9)) << 2)

#define XCVR_DIAG_PLLDRC_CTRL(n)	((0x40e0 | ((n) << 9)) << 2)
#define XCVR_DIAG_BIDI_CTRL(n)		((0x40e8 | ((n) << 9)) << 2)
#define XCVR_DIAG_LANE_FCM_EN_MGN(n)	((0x40f2 | ((n) << 9)) << 2)
#define TX_PSC_A0(n)			((0x4100 | ((n) << 9)) << 2)
#define TX_PSC_A1(n)			((0x4101 | ((n) << 9)) << 2)
#define TX_PSC_A2(n)			((0x4102 | ((n) << 9)) << 2)
#define TX_PSC_A3(n)			((0x4103 | ((n) << 9)) << 2)
#define TX_RCVDET_CTRL(n)		((0x4120 | ((n) << 9)) << 2)
#define TX_RCVDET_EN_TMR(n)		((0x4122 | ((n) << 9)) << 2)
#define TX_RCVDET_ST_TMR(n)		((0x4123 | ((n) << 9)) << 2)
#define TX_DIAG_TX_DRV(n)		((0x41e1 | ((n) << 9)) << 2)
#define TX_DIAG_BGREF_PREDRV_DELAY	(0x41e7 << 2)

/* Use this for "n" in macros like "_MULT_XXX" to target the aux channel */
#define AUX_CH_LANE			8

#define TX_ANA_CTRL_REG_1		(0x5020 << 2)

#define TXDA_DP_AUX_EN			BIT(15)
#define AUXDA_SE_EN			BIT(14)
#define TXDA_CAL_LATCH_EN		BIT(13)
#define AUXDA_POLARITY			BIT(12)
#define TXDA_DRV_POWER_ISOLATION_EN	BIT(11)
#define TXDA_DRV_POWER_EN_PH_2_N	BIT(10)
#define TXDA_DRV_POWER_EN_PH_1_N	BIT(9)
#define TXDA_BGREF_EN			BIT(8)
#define TXDA_DRV_LDO_EN			BIT(7)
#define TXDA_DECAP_EN_DEL		BIT(6)
#define TXDA_DECAP_EN			BIT(5)
#define TXDA_UPHY_SUPPLY_EN_DEL		BIT(4)
#define TXDA_UPHY_SUPPLY_EN		BIT(3)
#define TXDA_LOW_LEAKAGE_EN		BIT(2)
#define TXDA_DRV_IDLE_LOWI_EN		BIT(1)
#define TXDA_DRV_CMN_MODE_EN		BIT(0)

#define TX_ANA_CTRL_REG_2		(0x5021 << 2)

#define AUXDA_DEBOUNCING_CLK		BIT(15)
#define TXDA_LPBK_RECOVERED_CLK_EN	BIT(14)
#define TXDA_LPBK_ISI_GEN_EN		BIT(13)
#define TXDA_LPBK_SERIAL_EN		BIT(12)
#define TXDA_LPBK_LINE_EN		BIT(11)
#define TXDA_DRV_LDO_REDC_SINKIQ	BIT(10)
#define XCVR_DECAP_EN_DEL		BIT(9)
#define XCVR_DECAP_EN			BIT(8)
#define TXDA_MPHY_ENABLE_HS_NT		BIT(7)
#define TXDA_MPHY_SA_MODE		BIT(6)
#define TXDA_DRV_LDO_RBYR_FB_EN		BIT(5)
#define TXDA_DRV_RST_PULL_DOWN		BIT(4)
#define TXDA_DRV_LDO_BG_FB_EN		BIT(3)
#define TXDA_DRV_LDO_BG_REF_EN		BIT(2)
#define TXDA_DRV_PREDRV_EN_DEL		BIT(1)
#define TXDA_DRV_PREDRV_EN		BIT(0)

#define TXDA_COEFF_CALC_CTRL		(0x5022 << 2)

#define TX_HIGH_Z			BIT(6)
#define TX_VMARGIN_OFFSET		3
#define TX_VMARGIN_MASK			0x7
#define LOW_POWER_SWING_EN		BIT(2)
#define TX_FCM_DRV_MAIN_EN		BIT(1)
#define TX_FCM_FULL_MARGIN		BIT(0)

#define TX_DIG_CTRL_REG_2		(0x5024 << 2)

#define TX_HIGH_Z_TM_EN			BIT(15)
#define TX_RESCAL_CODE_OFFSET		0
#define TX_RESCAL_CODE_MASK		0x3f

#define TXDA_CYA_AUXDA_CYA		(0x5025 << 2)
#define TX_ANA_CTRL_REG_3		(0x5026 << 2)
#define TX_ANA_CTRL_REG_4		(0x5027 << 2)
#define TX_ANA_CTRL_REG_5		(0x5029 << 2)

#define RX_PSC_A0(n)			((0x8000 | ((n) << 9)) << 2)
#define RX_PSC_A1(n)			((0x8001 | ((n) << 9)) << 2)
#define RX_PSC_A2(n)			((0x8002 | ((n) << 9)) << 2)
#define RX_PSC_A3(n)			((0x8003 | ((n) << 9)) << 2)
#define RX_PSC_CAL(n)			((0x8006 | ((n) << 9)) << 2)
#define RX_PSC_RDY(n)			((0x8007 | ((n) << 9)) << 2)
#define RX_IQPI_ILL_CAL_OVRD		(0x8023 << 2)
#define RX_EPI_ILL_CAL_OVRD		(0x8033 << 2)
#define RX_SDCAL0_OVRD			(0x8041 << 2)
#define RX_SDCAL1_OVRD			(0x8049 << 2)
#define RX_SLC_INIT			(0x806d << 2)
#define RX_SLC_RUN			(0x806e << 2)
#define RX_CDRLF_CNFG2			(0x8081 << 2)
#define RX_SIGDET_HL_FILT_TMR(n)	((0x8090 | ((n) << 9)) << 2)
#define RX_SLC_IOP0_OVRD		(0x8101 << 2)
#define RX_SLC_IOP1_OVRD		(0x8105 << 2)
#define RX_SLC_QOP0_OVRD		(0x8109 << 2)
#define RX_SLC_QOP1_OVRD		(0x810d << 2)
#define RX_SLC_EOP0_OVRD		(0x8111 << 2)
#define RX_SLC_EOP1_OVRD		(0x8115 << 2)
#define RX_SLC_ION0_OVRD		(0x8119 << 2)
#define RX_SLC_ION1_OVRD		(0x811d << 2)
#define RX_SLC_QON0_OVRD		(0x8121 << 2)
#define RX_SLC_QON1_OVRD		(0x8125 << 2)
#define RX_SLC_EON0_OVRD		(0x8129 << 2)
#define RX_SLC_EON1_OVRD		(0x812d << 2)
#define RX_SLC_IEP0_OVRD		(0x8131 << 2)
#define RX_SLC_IEP1_OVRD		(0x8135 << 2)
#define RX_SLC_QEP0_OVRD		(0x8139 << 2)
#define RX_SLC_QEP1_OVRD		(0x813d << 2)
#define RX_SLC_EEP0_OVRD		(0x8141 << 2)
#define RX_SLC_EEP1_OVRD		(0x8145 << 2)
#define RX_SLC_IEN0_OVRD		(0x8149 << 2)
#define RX_SLC_IEN1_OVRD		(0x814d << 2)
#define RX_SLC_QEN0_OVRD		(0x8151 << 2)
#define RX_SLC_QEN1_OVRD		(0x8155 << 2)
#define RX_SLC_EEN0_OVRD		(0x8159 << 2)
#define RX_SLC_EEN1_OVRD		(0x815d << 2)
#define RX_REE_CTRL_DATA_MASK(n)	((0x81bb | ((n) << 9)) << 2)
#define RX_DIAG_SIGDET_TUNE(n)		((0x81dc | ((n) << 9)) << 2)
#define RX_DIAG_SC2C_DELAY		(0x81e1 << 2)

#define PMA_LANE_CFG			(0xc000 << 2)
#define PIPE_CMN_CTRL1			(0xc001 << 2)
#define PIPE_CMN_CTRL2			(0xc002 << 2)
#define PIPE_COM_LOCK_CFG1		(0xc003 << 2)
#define PIPE_COM_LOCK_CFG2		(0xc004 << 2)
#define PIPE_RCV_DET_INH		(0xc005 << 2)
#define DP_MODE_CTL			(0xc008 << 2)
#define DP_CLK_CTL			(0xc009 << 2)
#define STS				(0xc00F << 2)
#define PHY_ISO_CMN_CTRL		(0xc010 << 2)
#define PHY_DP_TX_CTL			(0xc408 << 2)
#define PMA_CMN_CTRL1			(0xc800 << 2)
#define PHY_PMA_ISO_CMN_CTRL		(0xc810 << 2)
#define PHY_ISOLATION_CTRL		(0xc81f << 2)
#define PHY_PMA_ISO_XCVR_CTRL(n)	((0xcc11 | ((n) << 6)) << 2)
#define PHY_PMA_ISO_LINK_MODE(n)	((0xcc12 | ((n) << 6)) << 2)
#define PHY_PMA_ISO_PWRST_CTRL(n)	((0xcc13 | ((n) << 6)) << 2)
#define PHY_PMA_ISO_TX_DATA_LO(n)	((0xcc14 | ((n) << 6)) << 2)
#define PHY_PMA_ISO_TX_DATA_HI(n)	((0xcc15 | ((n) << 6)) << 2)
#define PHY_PMA_ISO_RX_DATA_LO(n)	((0xcc16 | ((n) << 6)) << 2)
#define PHY_PMA_ISO_RX_DATA_HI(n)	((0xcc17 | ((n) << 6)) << 2)
#define TX_BIST_CTRL(n)			((0x4140 | ((n) << 9)) << 2)
#define TX_BIST_UDDWR(n)		((0x4141 | ((n) << 9)) << 2)

/*
 * Selects which PLL clock will be driven on the analog high speed
 * clock 0: PLL 0 div 1
 * clock 1: PLL 1 div 2
 */
#define CLK_PLL_CONFIG			0x30
#define CLK_PLL_MASK			0x33

#define CMN_READY			BIT(0)

#define DP_PLL_CLOCK_ENABLE		BIT(2)
#define DP_PLL_ENABLE			BIT(0)
#define DP_PLL_DATA_RATE_RBR		((2 << 12) | (4 << 8))
#define DP_PLL_DATA_RATE_HBR		((2 << 12) | (4 << 8))
#define DP_PLL_DATA_RATE_HBR2		((1 << 12) | (2 << 8))

#define DP_MODE_A0			BIT(4)
#define DP_MODE_A2			BIT(6)
#define DP_MODE_ENTER_A0		0xc101
#define DP_MODE_ENTER_A2		0xc104

#define PHY_MODE_SET_TIMEOUT		100000

#define PIN_ASSIGN_C_E			0x51d9
#define PIN_ASSIGN_D_F			0x5100

#define MODE_DISCONNECT			0
#define MODE_UFP_USB			BIT(0)
#define MODE_DFP_USB			BIT(1)
#define MODE_DFP_DP			BIT(2)

struct usb3phy_reg {
	u32 offset;
	u32 enable_bit;
	u32 write_enable;
};

/**
 * struct rockchip_usb3phy_port_cfg: usb3-phy port configuration.
 * @reg: the base address for usb3-phy config.
 * @typec_conn_dir: the register of type-c connector direction.
 * @usb3tousb2_en: the register of type-c force usb2 to usb2 enable.
 * @external_psm: the register of type-c phy external psm clock.
 * @pipe_status: the register of type-c phy pipe status.
 * @usb3_host_disable: the register of type-c usb3 host disable.
 * @usb3_host_port: the register of type-c usb3 host port.
 * @uphy_dp_sel: the register of type-c phy DP select control.
 */
struct rockchip_usb3phy_port_cfg {
	unsigned int reg;
	struct usb3phy_reg typec_conn_dir;
	struct usb3phy_reg usb3tousb2_en;
	struct usb3phy_reg external_psm;
	struct usb3phy_reg pipe_status;
	struct usb3phy_reg usb3_host_disable;
	struct usb3phy_reg usb3_host_port;
	struct usb3phy_reg uphy_dp_sel;
};

struct rockchip_tcphy {
	void __iomem *reg_base;
	void __iomem *grf_base;
	struct clk clk_core;
	struct clk clk_ref;
	struct reset_ctl uphy_rst;
	struct reset_ctl pipe_rst;
	struct reset_ctl tcphy_rst;
	const struct rockchip_usb3phy_port_cfg *port_cfgs;
	u8 mode;
	bool flip;
	/*
	 * The USB role the controller on this port runs as, from its set_mode
	 * (dwc3 sets it before power-on): MODE_UFP_USB for a gadget,
	 * MODE_DFP_USB for a host. What rockchip_tcphy_get_mode() answers when
	 * no port manager has negotiated anything yet; 0 before any set_mode.
	 */
	u8 usb_role;
};

struct phy_reg {
	u16 value;
	u32 addr;
};

static struct phy_reg usb3_pll_cfg[] = {
	{ 0xf0,		CMN_PLL0_VCOCAL_INIT },
	{ 0x18,		CMN_PLL0_VCOCAL_ITER },
	{ 0xd0,		CMN_PLL0_INTDIV },
	{ 0x4a4a,	CMN_PLL0_FRACDIV },
	{ 0x34,		CMN_PLL0_HIGH_THR },
	{ 0x1ee,	CMN_PLL0_SS_CTRL1 },
	{ 0x7f03,	CMN_PLL0_SS_CTRL2 },
	{ 0x20,		CMN_PLL0_DSM_DIAG },
	{ 0,		CMN_DIAG_PLL0_OVRD },
	{ 0,		CMN_DIAG_PLL0_FBH_OVRD },
	{ 0,		CMN_DIAG_PLL0_FBL_OVRD },
	{ 0x7,		CMN_DIAG_PLL0_V2I_TUNE },
	{ 0x45,		CMN_DIAG_PLL0_CP_TUNE },
	{ 0x8,		CMN_DIAG_PLL0_LF_PROG },
};

static struct phy_reg dp_pll_cfg[] = {
	{ 0xf0,		CMN_PLL1_VCOCAL_INIT },
	{ 0x18,		CMN_PLL1_VCOCAL_ITER },
	{ 0x30b9,	CMN_PLL1_VCOCAL_START },
	{ 0x21c,	CMN_PLL1_INTDIV },
	{ 0,		CMN_PLL1_FRACDIV },
	{ 0x5,		CMN_PLL1_HIGH_THR },
	{ 0x35,		CMN_PLL1_SS_CTRL1 },
	{ 0x7f1e,	CMN_PLL1_SS_CTRL2 },
	{ 0x20,		CMN_PLL1_DSM_DIAG },
	{ 0,		CMN_PLLSM1_USER_DEF_CTRL },
	{ 0,		CMN_DIAG_PLL1_OVRD },
	{ 0,		CMN_DIAG_PLL1_FBH_OVRD },
	{ 0,		CMN_DIAG_PLL1_FBL_OVRD },
	{ 0x6,		CMN_DIAG_PLL1_V2I_TUNE },
	{ 0x45,		CMN_DIAG_PLL1_CP_TUNE },
	{ 0x8,		CMN_DIAG_PLL1_LF_PROG },
	{ 0x100,	CMN_DIAG_PLL1_PTATIS_TUNE1 },
	{ 0x7,		CMN_DIAG_PLL1_PTATIS_TUNE2 },
	{ 0x4,		CMN_DIAG_PLL1_INCLK_CTRL },
};

static inline int property_enable(struct rockchip_tcphy *priv,
				  const struct usb3phy_reg *reg, bool en)
{
	u32 mask = 1 << reg->write_enable;
	u32 val = en << reg->enable_bit;

	return writel(val | mask, priv->grf_base + reg->offset);
}

static void rockchip_tcphy_cfg_24m(struct rockchip_tcphy *priv)
{
	u32 i, rdata;

	/*
	 * cmn_ref_clk_sel = 3, select the 24Mhz for clk parent
	 * cmn_psm_clk_dig_div = 2, set the clk division to 2
	 */
	writel(0x830, priv->reg_base + PMA_CMN_CTRL1);
	for (i = 0; i < 4; i++) {
		/*
		 * The following PHY configuration assumes a 24 MHz reference
		 * clock.
		 */
		writel(0x90, priv->reg_base + XCVR_DIAG_LANE_FCM_EN_MGN(i));
		writel(0x960, priv->reg_base + TX_RCVDET_EN_TMR(i));
		writel(0x30, priv->reg_base + TX_RCVDET_ST_TMR(i));
	}

	rdata = readl(priv->reg_base + CMN_DIAG_HSCLK_SEL);
	rdata &= ~CLK_PLL_MASK;
	rdata |= CLK_PLL_CONFIG;
	writel(rdata, priv->reg_base + CMN_DIAG_HSCLK_SEL);
}

static void rockchip_tcphy_cfg_usb3_pll(struct rockchip_tcphy *priv)
{
	u32 i;

	/* load the configuration of PLL0 */
	for (i = 0; i < ARRAY_SIZE(usb3_pll_cfg); i++)
		writel(usb3_pll_cfg[i].value,
		       priv->reg_base + usb3_pll_cfg[i].addr);
}

/*
 * PLL1 is the DisplayPort link clock, and it is configured on BOTH paths --
 * Linux calls this from its DP branch and again from its USB3 branch. That is
 * not belt-and-braces: without it PLL1 is never enabled and there is no DP
 * link clock at all, so a later switch into DP mode has nothing to run on.
 * RBR is the default rate; the link rate is renegotiated once DPCD is
 * readable.
 */
static void rockchip_tcphy_cfg_dp_pll(struct rockchip_tcphy *priv)
{
	u32 i;

	/* set the default mode to RBR */
	writel(DP_PLL_CLOCK_ENABLE | DP_PLL_ENABLE | DP_PLL_DATA_RATE_RBR,
	       priv->reg_base + DP_CLK_CTL);

	/* load the configuration of PLL1 */
	for (i = 0; i < ARRAY_SIZE(dp_pll_cfg); i++)
		writel(dp_pll_cfg[i].value,
		       priv->reg_base + dp_pll_cfg[i].addr);
}

static void rockchip_tcphy_tx_usb3_cfg_lane(struct rockchip_tcphy *priv,
					    u32 lane)
{
	writel(0x7799, priv->reg_base + TX_PSC_A0(lane));
	writel(0x7798, priv->reg_base + TX_PSC_A1(lane));
	writel(0x5098, priv->reg_base + TX_PSC_A2(lane));
	writel(0x5098, priv->reg_base + TX_PSC_A3(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_000(lane));
	writel(0xbf, priv->reg_base + XCVR_DIAG_BIDI_CTRL(lane));
}

static void rockchip_tcphy_rx_usb3_cfg_lane(struct rockchip_tcphy *priv,
					    u32 lane)
{
	writel(0xa6fd, priv->reg_base + RX_PSC_A0(lane));
	writel(0xa6fd, priv->reg_base + RX_PSC_A1(lane));
	writel(0xa410, priv->reg_base + RX_PSC_A2(lane));
	writel(0x2410, priv->reg_base + RX_PSC_A3(lane));
	writel(0x23ff, priv->reg_base + RX_PSC_CAL(lane));
	writel(0x13, priv->reg_base + RX_SIGDET_HL_FILT_TMR(lane));
	writel(0x03e7, priv->reg_base + RX_REE_CTRL_DATA_MASK(lane));
	writel(0x1004, priv->reg_base + RX_DIAG_SIGDET_TUNE(lane));
	writel(0x2010, priv->reg_base + RX_PSC_RDY(lane));
	writel(0xfb, priv->reg_base + XCVR_DIAG_BIDI_CTRL(lane));
}

/*
 * Where the orientation comes from.
 *
 * Plug orientation is measured on the CC lines by the Type-C port controller,
 * so the port manager is the only thing that knows it. Linux carries it from
 * TCPM to this PHY over extcon (EXTCON_PROP_USB_TYPEC_POLARITY); U-Boot had no
 * such link and this driver simply assumed a normally-inserted plug, which
 * costs SuperSpeed outright on a reversed one -- USB 2.0 is duplicated across
 * the receptacle and comes up either way, SuperSpeed is not.
 *
 * Ask the TCPM uclass. Assumes a single Type-C port, which holds for the boards
 * that describe one connector today, and degrades to the old fixed-normal
 * behaviour when TCPM is absent, unbuilt, or nothing is attached.
 */
#if CONFIG_IS_ENABLED(TYPEC_TCPM)
/**
 * rockchip_tcphy_find_tcpm() - the port manager owning THIS phy's connector
 *
 * Resolved through the device tree's connector graph, which is the only thing
 * that associates a Type-C PHY with a port manager: this phy's port/endpoint
 * points at one of the connector's endpoints, and the connector belongs to a
 * TCPC.
 *
 * This used to be uclass_first_device_err(UCLASS_TCPM, ...) -- take whichever
 * one bound first. That is indistinguishable from correct on a board with a
 * single connector, and silently wrong on a board with two, where a phy can
 * be handed the orientation of the OTHER port's cable and map its SuperSpeed
 * lanes to the wrong pins.
 */
static struct udevice *rockchip_tcphy_find_tcpm(struct udevice *dev, bool probe)
{
	const struct dm_tcpm_ops *ops;
	struct udevice *tcpm;
	ofnode ep, connector, node;

	ep = ofnode_graph_get_endpoint_by_regs(dev_ofnode(dev), -1, -1);
	if (!ofnode_valid(ep))
		return NULL;

	connector = ofnode_graph_get_remote_port_parent(ep);
	if (!ofnode_valid(connector))
		return NULL;

	/*
	 * Match on BOUND devices and probe only the winner.
	 *
	 * uclass_foreach_dev_probe() would probe every TCPM in the system as a
	 * side effect of one PHY asking which connector it feeds. On this
	 * board that starts the front port's state machine unbidden, in the
	 * middle of `usb start`, which is exactly what the dtsi's "U-Boot
	 * will not auto-probe this port" arrangement exists to prevent -- and
	 * it can push the bus-powered rear inlet into a PD renegotiation at
	 * the same time.
	 *
	 * Matching unprobed is safe because get_connector_node() is a pure
	 * ofnode read; fusb302's is a dev_read_subnode() and nothing more,
	 * and it is already called at bind time.
	 */
	uclass_find_first_device(UCLASS_TCPM, &tcpm);
	for (; tcpm; uclass_find_next_device(&tcpm)) {
		ops = dev_get_driver_ops(tcpm);
		if (!ops || !ops->get_connector_node)
			continue;

		if (ops->get_connector_node(tcpm, &node))
			continue;

		if (!ofnode_equal(node, connector))
			continue;

		/*
		 * @probe distinguishes callers by what they will DO with the
		 * answer, not by what they are asking about.
		 *
		 * rockchip_tcphy_get_orientation() and
		 * rockchip_typec_phy_get_dp_state() both pay for the probe:
		 * the first needs live CC state, and the second is about to
		 * REQUEST DisplayPort alt mode via tcpm_enter_dp_altmode(),
		 * which needs a probed port manager to ask.
		 *
		 * Note the probe itself does not enter anything, and an earlier
		 * version of this comment said it did. Entry follows a
		 * consumer's request; a probe on its own is what `usb start`
		 * does, and entering on that would hand a USB-C dock four
		 * DisplayPort lanes it never asked for.
		 *
		 * rockchip_tcphy_get_mode() does not, and must not:
		 * rockchip_usb3_phy_power_on() asks on EVERY call, so it is
		 * asking "has anything already happened?" and a port
		 * bring-up per call would be a cost with no question behind it.
		 */
		if (!probe)
			return device_active(tcpm) ? tcpm : NULL;

		if (device_probe(tcpm))
			return NULL;

		return tcpm;
	}

	return NULL;
}
#endif

#if !CONFIG_IS_ENABLED(TYPEC_TCPM) && CONFIG_IS_ENABLED(TYPEC_FUSB302_CC)
/**
 * rockchip_tcphy_cc_orientation() - orientation without the port manager
 *
 * The SPL has no TCPM, but it can follow the same connector graph to the
 * port controller and measure the CC lines once (fusb302_cc_orientation()).
 *
 * Return: an enum typec_orientation, or -errno
 */
static int rockchip_tcphy_cc_orientation(struct udevice *dev)
{
	struct udevice *bus, *chip;
	ofnode ep, connector, tcpc;
	u32 reg;
	int ret;

	ep = ofnode_graph_get_endpoint_by_regs(dev_ofnode(dev), -1, -1);
	if (!ofnode_valid(ep))
		return -ENOENT;
	connector = ofnode_graph_get_remote_port_parent(ep);
	if (!ofnode_valid(connector))
		return -ENOENT;
	tcpc = ofnode_get_parent(connector);
	if (!ofnode_device_is_compatible(tcpc, "fcs,fusb302"))
		return -ENODEV;

	ret = ofnode_read_u32(tcpc, "reg", &reg);
	if (!ret)
		ret = uclass_get_device_by_ofnode(UCLASS_I2C,
						  ofnode_get_parent(tcpc), &bus);
	if (!ret)
		ret = i2c_get_chip(bus, reg, 1, &chip);
	if (ret)
		return ret;

	return fusb302_cc_orientation(chip);
}
#endif

static void rockchip_tcphy_get_orientation(struct udevice *dev,
					   struct rockchip_tcphy *priv)
{
	priv->flip = false;

#if !CONFIG_IS_ENABLED(TYPEC_TCPM) && CONFIG_IS_ENABLED(TYPEC_FUSB302_CC)
	int orientation = rockchip_tcphy_cc_orientation(dev);

	printf("Type-C: %s plug (%d)\n",
	       orientation == TYPEC_ORIENTATION_REVERSE ? "reversed" :
	       orientation == TYPEC_ORIENTATION_NORMAL ? "normal" : "unknown",
	       orientation);
	if (orientation == TYPEC_ORIENTATION_REVERSE)
		priv->flip = true;
#endif

#if CONFIG_IS_ENABLED(TYPEC_TCPM)
	struct udevice *tcpm = rockchip_tcphy_find_tcpm(dev, true);

	if (!tcpm) {
		/*
		 * Say so rather than assuming. A board whose connector graph
		 * is incomplete gets the normal orientation, which is a guess
		 * that is right half the time -- worth a line in the log
		 * rather than a silent coin flip.
		 */
		dev_warn(dev, "no Type-C port manager for this phy; assuming normal orientation\n");
		return;
	}

	if (tcpm_get_orientation(tcpm) == TYPEC_ORIENTATION_REVERSE)
		priv->flip = true;
#endif
}

/**
 * rockchip_tcphy_get_mode() - what this connector has actually negotiated
 *
 * Pulled from the port manager, in the same direction as the orientation
 * lookup above and through the same connector-graph walk. The alternative --
 * pushing the negotiated link from TCPM into the PHY -- has no facility to
 * push it through: generic_phy_configure() and generic_phy_set_mode() both
 * return 0 when unimplemented AND when the phy handle is invalid, so neither
 * can tell "the PHY took it" from "there is no PHY", and PHY_MODE_DP has zero
 * users tree-wide.
 *
 * Returns a BARE MODE_DFP_DP for a 4-lane DisplayPort link, because
 * rockchip_tcphy_init() tests it by exact equality. It also returns ONE of
 * MODE_DFP_USB / MODE_UFP_USB rather than both: those are mutually exclusive
 * roles -- DFP means we are the host, UFP the device -- and the old
 * unconditional OR was harmless only while nothing read the value.
 */
static int rockchip_tcphy_get_mode(struct udevice *dev,
				   struct rockchip_tcphy *priv)
{
	/*
	 * With nothing negotiated, the role is the controller's own: a dwc3 bound
	 * as a gadget is the USB device on this port, and calling it the host
	 * (the old unconditional MODE_DFP_USB) described the opposite of what
	 * the port is doing. Host stays the answer when no controller said.
	 */
	u8 fallback = priv->usb_role ?: MODE_DFP_USB;
#if CONFIG_IS_ENABLED(TYPEC_TCPM)
	struct udevice *tcpm = rockchip_tcphy_find_tcpm(dev, false);
	int mode;

	if (!tcpm)
		return fallback;

	mode = tcpm_get_data_role(tcpm) == TYPEC_HOST ? MODE_DFP_USB
						      : MODE_UFP_USB;

	if (IS_ENABLED(CONFIG_TYPEC_DISPLAYPORT)) {
		struct typec_dp_state *dp = tcpm_dp_state(tcpm);

		/*
		 * Pin assignment C or E is four DP lanes and no USB
		 * SuperSpeed, which is the only configuration this PHY has a
		 * 4-lane path for; D is the 2-lane multiplexed one and stays
		 * on the USB3 branch, where dp_cfg_lane() already programs
		 * the DP pair.
		 */
		if (dp->active &&
		    (DP_CONF_GET_PIN_ASSIGN(dp->conf) &
		     (BIT(DP_PIN_ASSIGN_C) | BIT(DP_PIN_ASSIGN_E))))
			return MODE_DFP_DP;
	}

	return mode;
#else
	return fallback;
#endif
}

/**
 * rockchip_typec_phy_get_dp_state() - the DP link this PHY carries
 * @phy: the "dp-port" phy of a rockchip,rk3399-typec-phy
 * @state: filled in on success only
 *
 * Exported from the PHY rather than left for the display controller to work
 * out by reading a Type-C port manager itself, and that is a correctness
 * choice rather than a layering preference.
 *
 * Answering "how many DP lanes, and which way round?" first requires
 * answering "WHICH of the two Type-C connectors is this?", and the only thing
 * in the system that says so is the device tree's connector graph -- the walk
 * rockchip_tcphy_find_tcpm() performs. A consumer asking on its own behalf
 * would either duplicate that walk or take the shortcut this driver already
 * had to abandon: uclass_first_device_err(UCLASS_TCPM, ...), "whichever one
 * bound first", which is indistinguishable from correct on a board with one
 * connector and silently wrong on this board, which has two. cdn-dp feeds the
 * orientation it is given straight into the firmware's lane mapping
 * (LANE_MAPPING_FLIPPED vs LANE_MAPPING_NORMAL), so that mistake would surface
 * as a mirrored four-lane link with nothing logged anywhere -- the same class
 * of symptom as a bad cable.
 *
 * The PHY is also the only place the negotiated link can be reconciled with
 * what was actually programmed into the lane mux; see the priv->mode branch.
 *
 * Return: 0, or a negative errno. -ENODEV means the connector is not in a
 * DisplayPort alt mode this PHY can carry, which is a real answer -- there is
 * deliberately no "success with lanes == 0" spelling for it, because a zero
 * lane count reaching cdn_dp_set_host_cap() would be a link failure whose
 * cause nothing reports.
 */
int rockchip_typec_phy_get_dp_state(struct phy *phy,
				    struct rockchip_typec_phy_dp_state *state)
{
#if CONFIG_IS_ENABLED(TYPEC_TCPM)
	struct rockchip_tcphy *priv;
	struct typec_dp_state *dp;
	struct udevice *tcpm;
	u8 pin_assign;
	int ret;

	if (!state)
		return -EINVAL;

	if (!IS_ENABLED(CONFIG_TYPEC_DISPLAYPORT))
		return -ENOTSUPP;

	/*
	 * Identity, not shape. dev_get_priv() on some other driver's device
	 * does not return NULL for a caller to test -- it returns that
	 * driver's private data reinterpreted as a struct rockchip_tcphy,
	 * which is a plausible-looking answer built out of unrelated memory.
	 */
	if (!generic_phy_valid(phy) ||
	    phy->dev->driver != DM_DRIVER_GET(rockchip_tcphy_dp_port))
		return -EINVAL;

	priv = dev_get_priv(dev_get_parent(phy->dev));

	/*
	 * probe = true, and this is the one caller for which that is right.
	 *
	 * It used to be false, matching rockchip_tcphy_get_mode(), on the
	 * reasoning that "an unprobed port manager has negotiated nothing, so
	 * probing one just to be told so starts a state machine for no answer".
	 * That was sound only while entering DisplayPort alt mode required a
	 * human to type `tcpm dp enter`: probing could genuinely never produce
	 * a DisplayPort state, so the probe bought nothing.
	 *
	 * What changed is that this function now REQUESTS the mode -- see the
	 * tcpm_enter_dp_altmode() call below -- and a request needs a probed
	 * device to make. So the answer this function exists to give does not
	 * exist until the probe has run, and returning -ENODEV without probing
	 * would report "not in DisplayPort alt mode" about a port that was
	 * never asked: an untested assumption wearing a measurement's clothes,
	 * and the exact reason boot-time DP failed while the same board did 4K
	 * DisplayPort from the prompt.
	 *
	 * ⚠️ The probe does NOT itself enter the mode, and an earlier version
	 * of this comment claimed it did. That was briefly true and was a bug:
	 * entering on any probe meant `usb start` reconfigured USB-C partners
	 * for DisplayPort. Do not restore probe = false on the strength of the
	 * old wording -- the flag is still needed, for the request.
	 *
	 * The blast radius stays narrow because rockchip_tcphy_find_tcpm()
	 * matches on BOUND devices and probes only the winner: the connector
	 * this phy's own graph points at, never every TCPM in the system. The
	 * other Type-C port is not touched.
	 *
	 * rockchip_tcphy_get_mode() deliberately keeps probe = false. It is
	 * called from rockchip_usb3_phy_power_on() on every power-on of a USB3
	 * phy, where the question really is "has anything already happened?"
	 * and paying for a port bring-up per call would be wrong.
	 */
	tcpm = rockchip_tcphy_find_tcpm(phy->dev, true);
	if (!tcpm)
		return -ENODEV;

	dp = tcpm_dp_state(tcpm);
	if (!dp->active) {
		/*
		 * Ask for the mode. This call IS the intent -- the only caller
		 * that reaches here is cdn-dp on its way to lighting a display,
		 * so this is the point at which "somebody wants DisplayPort"
		 * becomes true, and it is the reason entry is requested rather
		 * than done to every partner the port manager meets.
		 *
		 * TCPM still owns the exchange; this asks, it does not send.
		 * Doing it here rather than at TCPM probe is what keeps
		 * `usb start` from configuring a dock for four DisplayPort
		 * lanes and taking its SuperSpeed away.
		 */
		ret = tcpm_enter_dp_altmode(tcpm);
		if (ret)
			return ret == -ENOENT ? -ENODEV : ret;

		/* Re-read: typec_dp_enter() writes through this pointer. */
		dp = tcpm_dp_state(tcpm);
		if (!dp->active)
			return -ENODEV;
	}

	/*
	 * The same pin assignments rockchip_tcphy_init() muxes for, read as
	 * lane counts rather than as a mode: C and E are DisplayPort on all
	 * four lanes (PIN_ASSIGN_C_E), D and F are two DP lanes beside USB
	 * SuperSpeed (PIN_ASSIGN_D_F). A and B carry no DisplayPort on this
	 * SoC -- rk_typec_mux_set() drops them -- so they are -ENODEV.
	 *
	 * Tested in that order, not as an exact match: a partner may accept
	 * more than one bit, and four lanes is the better link whenever it is
	 * on offer. It also keeps this in agreement with
	 * rockchip_tcphy_get_mode(), which returns MODE_DFP_DP -- and so muxes
	 * PIN_ASSIGN_C_E -- on the same C|E test.
	 */
	pin_assign = DP_CONF_GET_PIN_ASSIGN(dp->conf);
	if (pin_assign & (BIT(DP_PIN_ASSIGN_C) | BIT(DP_PIN_ASSIGN_E)))
		state->lanes = 4;
	else if (pin_assign & (BIT(DP_PIN_ASSIGN_D) | BIT(DP_PIN_ASSIGN_F)))
		state->lanes = 2;
	else
		return -ENODEV;

	/*
	 * Where the orientation comes from depends on whether this PHY has
	 * been muxed yet, and that distinction is the reason this lives here.
	 *
	 * Once rockchip_tcphy_set_mode() has run, priv->flip is not a cached
	 * copy of the port manager's answer -- it is the value that WAS
	 * written to typec_conn_dir and AUXDA_POLARITY, i.e. the orientation
	 * the lanes and the AUX channel are physically muxed for. A cable
	 * re-plugged between power_on and this call changes what the port
	 * manager reports and changes nothing in the PHY, so re-reading TCPM
	 * would hand the caller a lane mapping that disagrees with the
	 * hardware. Before the PHY is muxed there is no such value, and the
	 * live answer is the only one there is.
	 */
	if (priv->mode == MODE_DISCONNECT)
		state->flip = tcpm_get_orientation(tcpm) ==
			      TYPEC_ORIENTATION_REVERSE;
	else
		state->flip = priv->flip;

	return 0;
#else
	return -ENOTSUPP;
#endif
}

/*
 * Configure one lane of the pair NOT carrying SuperSpeed.
 *
 * Both pairs have to be programmed even in a USB3-only configuration: the PHY
 * comes out of reset with a lane mux that only works for a normally-inserted
 * plug, and switching PMA_LANE_CFG without having configured the other pair
 * leaves it unprogrammed. Ported from Linux's tcphy_dp_cfg_lane().
 */
static void rockchip_tcphy_dp_cfg_lane(struct rockchip_tcphy *priv, u32 lane)
{
	u16 rdata;

	writel(0xbefc, priv->reg_base + XCVR_PSM_RCTRL(lane));
	writel(0x6799, priv->reg_base + TX_PSC_A0(lane));
	writel(0x6798, priv->reg_base + TX_PSC_A1(lane));
	writel(0x98, priv->reg_base + TX_PSC_A2(lane));
	writel(0x98, priv->reg_base + TX_PSC_A3(lane));

	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_000(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_001(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_010(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_011(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_100(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_101(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_110(lane));
	writel(0, priv->reg_base + TX_TXCC_MGNFS_MULT_111(lane));
	writel(0, priv->reg_base + TX_TXCC_CPOST_MULT_10(lane));
	writel(0, priv->reg_base + TX_TXCC_CPOST_MULT_01(lane));
	writel(0, priv->reg_base + TX_TXCC_CPOST_MULT_00(lane));
	writel(0, priv->reg_base + TX_TXCC_CPOST_MULT_11(lane));

	writel(0x128, priv->reg_base + TX_TXCC_CAL_SCLR_MULT(lane));
	writel(0x400, priv->reg_base + TX_DIAG_TX_DRV(lane));

	rdata = readl(priv->reg_base + XCVR_DIAG_PLLDRC_CTRL(lane));
	rdata = (rdata & 0x8fff) | 0x6000;
	writel(rdata, priv->reg_base + XCVR_DIAG_PLLDRC_CTRL(lane));
}

/* AUX polarity follows the plug, same as Linux's tcphy_dp_aux_set_flip(). */
static void rockchip_tcphy_dp_aux_set_flip(struct rockchip_tcphy *priv)
{
	u16 tx_ana_ctrl_reg_1;

	tx_ana_ctrl_reg_1 = readl(priv->reg_base + TX_ANA_CTRL_REG_1);
	if (!priv->flip)
		tx_ana_ctrl_reg_1 |= AUXDA_POLARITY;
	else
		tx_ana_ctrl_reg_1 &= ~AUXDA_POLARITY;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);
}

/*
 * Ported byte for byte from Linux's tcphy_dp_aux_calibration(). Without it
 * the AUX channel never drives, so there is no DPCD and no EDID.
 *
 * The tail is explicitly undocumented magic, and includes a block that turns
 * off the LDO and BGREF this same function just spent time turning on, and
 * another that undoes the LDO voltage it just programmed. Upstream's entire
 * justification is "Without this magic, things seem worse." Do not clean it
 * up, drop it, or reorder it.
 *
 * One upstream defect is carried forward deliberately: @calib is u16 while
 * its operands are signed, and the OR into TX_DIG_CTRL_REG_2 does not re-mask
 * with TX_RESCAL_CODE_MASK, so a negative adjustment can spill past bit 5.
 * Fixing it here would mean shipping calibration values this hardware has
 * never been run with; it belongs upstream, against a board that can measure
 * the difference.
 */
static void rockchip_tcphy_dp_aux_calibration(struct rockchip_tcphy *priv)
{
	u16 val;
	u16 tx_ana_ctrl_reg_1;
	u16 tx_ana_ctrl_reg_2;
	s32 pu_calib_code, pd_calib_code;
	s32 pu_adj, pd_adj;
	s32 calib_raw;
	u16 calib;

	/*
	 * Calculate calibration code as per docs: use an average of the
	 * pull down and pull up.  Then add in adjustments.
	 */
	val = readl(priv->reg_base + CMN_TXPUCAL_CTRL);
	pu_calib_code = CMN_CALIB_CODE_POS(val);
	val = readl(priv->reg_base + CMN_TXPDCAL_CTRL);
	pd_calib_code = CMN_CALIB_CODE_POS(val);
	val = readl(priv->reg_base + CMN_TXPU_ADJ_CTRL);
	pu_adj = CMN_CALIB_CODE(val);
	val = readl(priv->reg_base + CMN_TXPD_ADJ_CTRL);
	pd_adj = CMN_CALIB_CODE(val);
	calib_raw = (pu_calib_code + pd_calib_code) / 2 + pu_adj + pd_adj;
	calib = calib_raw;

	/*
	 * DIAGNOSTIC ONLY -- this does not change what is written, on purpose.
	 *
	 * The four operands are read from per-part calibration registers, so
	 * calib_raw varies from boot to boot on the same board.  It is not
	 * bounded to TX_RESCAL_CODE_MASK anywhere: the average alone spans
	 * 0..127 (CMN_CALIB_CODE_POS is 7 bits) and both adjustments are
	 * sign-extended, so the sum can land outside 0..0x3f in either
	 * direction.  The OR below does not re-mask, so when it does:
	 *
	 *   calib_raw > 0x3f  -- spills into bits 6..14, which are reserved
	 *   calib_raw < 0     -- wraps as u16 and sets BIT(15), which is
	 *                        TX_HIGH_Z_TM_EN.  That parks the AUX
	 *                        transmitter in high-Z, i.e. AUX stops driving
	 *                        entirely, for the whole boot.
	 *
	 * The second case is a candidate mechanism for the intermittent AUX
	 * failure tracked as B77 -- it is per-boot, all-or-nothing, kills DPCD
	 * and EDID together, and is invisible to every protocol-level probe.
	 * Print the operands so a corpus of boots can be correlated against
	 * EDID success before anything here is changed; a fix that is applied
	 * in the same step as the measurement cannot be attributed.
	 */
	if (calib_raw < 0 || calib_raw > TX_RESCAL_CODE_MASK)
		printf("tcphy: AUX rescal %d outside 0..%d (pu %d, pd %d, pu_adj %d, pd_adj %d)%s\n",
		       calib_raw, TX_RESCAL_CODE_MASK, pu_calib_code,
		       pd_calib_code, pu_adj, pd_adj,
		       calib_raw < 0 ? " -- WRAPS, sets TX_HIGH_Z_TM_EN" : "");
	else
		debug("tcphy: AUX rescal %d (pu %d, pd %d, pu_adj %d, pd_adj %d)\n",
		      calib_raw, pu_calib_code, pd_calib_code, pu_adj, pd_adj);

	/* disable txda_cal_latch_en for rewrite the calibration values */
	tx_ana_ctrl_reg_1 = readl(priv->reg_base + TX_ANA_CTRL_REG_1);
	tx_ana_ctrl_reg_1 &= ~TXDA_CAL_LATCH_EN;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);

	/* write the calibration, then delay 10 ms as sample in docs */
	val = readl(priv->reg_base + TX_DIG_CTRL_REG_2);
	val &= ~(TX_RESCAL_CODE_MASK << TX_RESCAL_CODE_OFFSET);
	val |= calib << TX_RESCAL_CODE_OFFSET;
	writel(val, priv->reg_base + TX_DIG_CTRL_REG_2);
	debug("tcphy: TX_DIG_CTRL_REG_2 <- 0x%04x (high-z %s)\n", val,
	      (val & TX_HIGH_Z_TM_EN) ? "SET" : "clear");
	if (val & TX_HIGH_Z_TM_EN)
		printf("tcphy: TX_DIG_CTRL_REG_2 = 0x%04x has TX_HIGH_Z_TM_EN set -- AUX will not drive\n",
		       val);
	usleep_range(10000, 10050);

	/*
	 * Enable signal for latch that sample and holds calibration values.
	 * Activate this signal for 1 clock cycle to sample new calibration
	 * values.
	 */
	tx_ana_ctrl_reg_1 |= TXDA_CAL_LATCH_EN;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);
	usleep_range(150, 200);

	/* set TX Voltage Level and TX Deemphasis to 0 */
	writel(0, priv->reg_base + PHY_DP_TX_CTL);

	/* re-enable decap */
	tx_ana_ctrl_reg_2 = XCVR_DECAP_EN;
	writel(tx_ana_ctrl_reg_2, priv->reg_base + TX_ANA_CTRL_REG_2);
	udelay(1);
	tx_ana_ctrl_reg_2 |= XCVR_DECAP_EN_DEL;
	writel(tx_ana_ctrl_reg_2, priv->reg_base + TX_ANA_CTRL_REG_2);

	writel(0, priv->reg_base + TX_ANA_CTRL_REG_3);

	tx_ana_ctrl_reg_1 |= TXDA_UPHY_SUPPLY_EN;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);
	udelay(1);
	tx_ana_ctrl_reg_1 |= TXDA_UPHY_SUPPLY_EN_DEL;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);

	writel(0, priv->reg_base + TX_ANA_CTRL_REG_5);

	/*
	 * Programs txda_drv_ldo_prog[15:0], Sets driver LDO
	 * voltage 16'h1001 for DP-AUX-TX and RX
	 */
	writel(0x1001, priv->reg_base + TX_ANA_CTRL_REG_4);

	/* re-enables Bandgap reference for LDO */
	tx_ana_ctrl_reg_1 |= TXDA_DRV_LDO_EN;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);
	udelay(5);
	tx_ana_ctrl_reg_1 |= TXDA_BGREF_EN;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);

	/*
	 * re-enables the transmitter pre-driver, driver data selection MUX,
	 * and receiver detect circuits.
	 */
	tx_ana_ctrl_reg_2 |= TXDA_DRV_PREDRV_EN;
	writel(tx_ana_ctrl_reg_2, priv->reg_base + TX_ANA_CTRL_REG_2);
	udelay(1);
	tx_ana_ctrl_reg_2 |= TXDA_DRV_PREDRV_EN_DEL;
	writel(tx_ana_ctrl_reg_2, priv->reg_base + TX_ANA_CTRL_REG_2);

	/*
	 * Do all the undocumented magic:
	 * - Turn on TXDA_DP_AUX_EN, whatever that is, even though sample
	 *   never shows this going on.
	 * - Turn on TXDA_DECAP_EN (and TXDA_DECAP_EN_DEL) even though
	 *   docs say for aux it's always 0.
	 * - Turn off the LDO and BGREF, which we just spent time turning
	 *   on above (???).
	 *
	 * Without this magic, things seem worse.
	 */
	tx_ana_ctrl_reg_1 |= TXDA_DP_AUX_EN;
	tx_ana_ctrl_reg_1 |= TXDA_DECAP_EN;
	tx_ana_ctrl_reg_1 &= ~TXDA_DRV_LDO_EN;
	tx_ana_ctrl_reg_1 &= ~TXDA_BGREF_EN;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);
	udelay(1);
	tx_ana_ctrl_reg_1 |= TXDA_DECAP_EN_DEL;
	writel(tx_ana_ctrl_reg_1, priv->reg_base + TX_ANA_CTRL_REG_1);

	/*
	 * Undo the work we did to set the LDO voltage.
	 * This doesn't seem to help nor hurt, but it kinda goes with the
	 * undocumented magic above.
	 */
	writel(0, priv->reg_base + TX_ANA_CTRL_REG_4);

	/* Don't set voltage swing to 400 mV peak to peak (differential) */
	writel(0, priv->reg_base + TXDA_COEFF_CALC_CTRL);

	/* Init TXDA_CYA_AUXDA_CYA for unknown magic reasons */
	writel(0, priv->reg_base + TXDA_CYA_AUXDA_CYA);

	/*
	 * More undocumented magic, presumably the goal of which is to
	 * make the "auxda_source_aux_oen" be ignored and instead to decide
	 * about "high impedance state" based on what software puts in the
	 * register TXDA_COEFF_CALC_CTRL (see TX_HIGH_Z).  Since we only
	 * program that register once and we don't set the bit TX_HIGH_Z,
	 * presumably the goal here is that we should never put the analog
	 * driver in high impedance state.
	 */
	val = readl(priv->reg_base + TX_DIG_CTRL_REG_2);
	val |= TX_HIGH_Z_TM_EN;
	writel(val, priv->reg_base + TX_DIG_CTRL_REG_2);
}

/*
 * @mode is a PARAMETER, not priv->mode, and that is the whole point: it is the
 * mode being REQUESTED, while priv->mode is the mode the PHY is already in.
 * Reading the destination out of the field that records the current state is
 * what let rockchip_dp_phy_power_on() gate its entire setup behind
 * "priv->mode == MODE_DISCONNECT" and still believe it had configured
 * DisplayPort, and what let rockchip_usb3_phy_power_on() write PIN_ASSIGN_D_F
 * over a connector that had already negotiated four DP lanes. Linux's
 * tcphy_phy_init() takes the same argument for the same reason.
 */
static int rockchip_tcphy_init(struct phy *phy, struct rockchip_tcphy *priv,
			       u8 mode)
{
	const struct rockchip_usb3phy_port_cfg *cfg = priv->port_cfgs;
	u32 val;
	int ret;

	ret = clk_enable(&priv->clk_core);
	if (ret) {
		dev_err(phy->dev, "failed to enable core clk (ret=%d)\n", ret);
		return ret;
	}

	ret = clk_enable(&priv->clk_ref);
	if (ret) {
		dev_err(phy->dev, "failed to enable ref clk (ret=%d)\n", ret);
		goto err_clk_core;
	}

	ret = reset_deassert(&priv->tcphy_rst);
	if (ret) {
		dev_err(phy->dev, "failed to deassert uphy-tcphy reset (ret=%d)\n",
			ret);
		goto err_clk_ref;
	}

	property_enable(priv, &cfg->typec_conn_dir, priv->flip);
	rockchip_tcphy_dp_aux_set_flip(priv);

	rockchip_tcphy_cfg_24m(priv);

	rockchip_tcphy_cfg_usb3_pll(priv);
	rockchip_tcphy_cfg_dp_pll(priv);

	/*
	 * SuperSpeed rides lanes 0/1 for a normally-inserted plug and 3/2 for a
	 * reversed one -- note the pair is not symmetric, it is TX 3 / RX 2, not
	 * TX 2 / RX 3. The other pair still has to be programmed, and the lane
	 * mux written, or the PHY keeps its reset default and only the normal
	 * orientation ever links. Mirrors Linux's non-DP branch.
	 */
	/*
	 * EXACT equality, not a mask test, and Linux is the same. MODE_DFP_DP
	 * on its own means "four lanes, DisplayPort only"; ORed with a USB
	 * mode it would mean the 2-lane multiplexed pin assignment, which is
	 * the branch below. Testing with & here would silently take the
	 * 2-lane path for a 4-lane link -- and it would complete perfectly,
	 * leaving a link nobody asked for.
	 */
	if (mode == MODE_DFP_DP) {
		/*
		 * No flip conditional: all four lanes get identical config
		 * and PIN_ASSIGN_C_E is unconditional. Orientation is handled
		 * entirely by typec_conn_dir and the AUX polarity bit, both
		 * already written above.
		 */
		rockchip_tcphy_dp_cfg_lane(priv, 0);
		rockchip_tcphy_dp_cfg_lane(priv, 1);
		rockchip_tcphy_dp_cfg_lane(priv, 2);
		rockchip_tcphy_dp_cfg_lane(priv, 3);

		writel(PIN_ASSIGN_C_E, priv->reg_base + PMA_LANE_CFG);
	} else {
		if (priv->flip) {
			rockchip_tcphy_tx_usb3_cfg_lane(priv, 3);
			rockchip_tcphy_rx_usb3_cfg_lane(priv, 2);
			rockchip_tcphy_dp_cfg_lane(priv, 0);
			rockchip_tcphy_dp_cfg_lane(priv, 1);
		} else {
			rockchip_tcphy_tx_usb3_cfg_lane(priv, 0);
			rockchip_tcphy_rx_usb3_cfg_lane(priv, 1);
			rockchip_tcphy_dp_cfg_lane(priv, 2);
			rockchip_tcphy_dp_cfg_lane(priv, 3);
		}

		writel(PIN_ASSIGN_D_F, priv->reg_base + PMA_LANE_CFG);
	}

	writel(DP_MODE_ENTER_A2, priv->reg_base + DP_MODE_CTL);

	ret = reset_deassert(&priv->uphy_rst);
	if (ret) {
		dev_err(phy->dev, "failed to deassert uphy rst (ret=%d)\n",
			ret);
		goto err_tcphy_rst;
	}

	ret = readl_poll_sleep_timeout(priv->reg_base + PMA_CMN_CTRL1,
				       val, val & CMN_READY, 10,
				       PHY_MODE_SET_TIMEOUT);
	if (ret < 0) {
		dev_err(phy->dev, "PMA Timeout!\n");
		ret = -ETIMEDOUT;
		goto err_uphy_rst;
	}

	if (mode == MODE_DFP_DP) {
		/*
		 * DP_MODE_CTL is a mixed request/status register: requests in
		 * bits 3:0, ACHIEVED state in 7:4, four positions above. Never
		 * test it against the value that was written -- A2 was
		 * REQUESTED as DP_MODE_ENTER_A2 and is REPORTED as DP_MODE_A2.
		 */
		ret = readl_poll_sleep_timeout(priv->reg_base + DP_MODE_CTL,
					       val, val & DP_MODE_A2, 1000,
					       PHY_MODE_SET_TIMEOUT);
		if (ret < 0) {
			dev_err(phy->dev, "failed to wait TCPHY enter A2\n");
			goto err_uphy_rst;
		}

		rockchip_tcphy_dp_aux_calibration(priv);

		/*
		 * Deassert the pipe reset on the DisplayPort path too, exactly
		 * as Linux's tcphy_phy_init() does unconditionally.
		 *
		 * This used to return here on the reasoning that "DP takes no
		 * pipe". That is true of DATA -- DP carries no PIPE traffic --
		 * but it does not follow that the reset may stay asserted.
		 *
		 * What IS measured: with the early return in place, the first
		 * caller to reach this path (cdn-dp's firmware init) got
		 * "failed to wait TCPHY enter A0" and -110 from
		 * rockchip_dp_phy_power_on(), on a connector that had just
		 * negotiated DP alt mode cleanly (SVIDs ff01 04e8, pin
		 * assignment C, HPD asserted). Deasserting here is Linux's
		 * behaviour and the first thing to try against that symptom;
		 * whether it is sufficient is not yet established.
		 */
		ret = reset_deassert(&priv->pipe_rst);
		if (ret) {
			dev_err(phy->dev,
				"failed to deassert pipe rst on DP path (ret=%d)\n",
				ret);
			goto err_uphy_rst;
		}

		return 0;
	}

	ret = reset_deassert(&priv->pipe_rst);
	if (ret) {
		dev_err(phy->dev, "failed to deassert pipe rst (ret=%d)\n",
			ret);
		goto err_uphy_rst;
	}

	return 0;

err_uphy_rst:
	reset_assert(&priv->uphy_rst);
err_tcphy_rst:
	reset_assert(&priv->tcphy_rst);
err_clk_ref:
	clk_disable(&priv->clk_ref);
err_clk_core:
	clk_disable(&priv->clk_core);
	return ret;
}

static void rockchip_tcphy_exit(struct rockchip_tcphy *priv)
{
	reset_assert(&priv->tcphy_rst);
	reset_assert(&priv->uphy_rst);
	reset_assert(&priv->pipe_rst);
	clk_disable(&priv->clk_core);
	clk_disable(&priv->clk_ref);
}

/**
 * rockchip_tcphy_set_mode() - put the PHY into @new_mode from ANY current mode
 *
 * The only place priv->mode is assigned and the only place the PHY is
 * initialised. Three states matter -- MODE_DISCONNECT, a USB3 lane
 * configuration (PIN_ASSIGN_D_F) and 4-lane DisplayPort (PIN_ASSIGN_C_E) --
 * and every transition between them is the same two steps: tear down whatever
 * is running, then initialise for the destination.
 *
 * Saying that once replaces the per-caller "if (priv->mode == MODE_DISCONNECT)
 * init()" tests, which were only ever correct for a PHY coming up from cold,
 * and which each caller got wrong in its own way for the transition this board
 * actually performs: `usb start` leaves MODE_DFP_USB, and `tcpm dp enter` then
 * asks for MODE_DFP_DP.
 *
 * Idempotent: a request for the mode the PHY is already in is a no-op, so a
 * repeated power_on neither re-muxes lanes nor bounces the clocks.
 *
 * On failure the PHY is left in MODE_DISCONNECT with clocks and resets as they
 * were before init -- never half-way between two lane configurations. The next
 * caller decides what to do by reading priv->mode, so a mode that lies is how
 * one failure becomes permanent.
 */
static int rockchip_tcphy_set_mode(struct phy *phy,
				   struct rockchip_tcphy *priv, u8 new_mode)
{
	int ret;

	if (priv->mode == new_mode)
		return 0;

	/*
	 * A mode change from anything but DISCONNECT is a full teardown.
	 * 4-lane DP and USB3 want different lane configs, a different
	 * PMA_LANE_CFG and a different set of PLLs, and PMA_LANE_CFG is
	 * latched out of uphy reset -- it cannot be rewritten under a running
	 * PHY.
	 */
	if (priv->mode != MODE_DISCONNECT) {
		rockchip_tcphy_exit(priv);
		priv->mode = MODE_DISCONNECT;
	}

	/*
	 * Re-read the orientation on every transition rather than trusting the
	 * one cached at the first power_on: the cable can be re-plugged
	 * between `usb start` and `tcpm dp enter`, and a stale flip maps the
	 * lanes to the wrong pins with no other symptom.
	 */
	rockchip_tcphy_get_orientation(phy->dev, priv);
	dev_dbg(phy->dev, "Type-C orientation: %s\n",
		priv->flip ? "reverse" : "normal");

	ret = rockchip_tcphy_init(phy, priv, new_mode);
	if (ret) {
		dev_err(phy->dev, "failed to init tcphy (ret=%d)\n", ret);
		return ret;
	}

	priv->mode = new_mode;

	return 0;
}

static int tcphy_cfg_usb3_to_usb2_only(struct rockchip_tcphy *priv,
				       bool value)
{
	const struct rockchip_usb3phy_port_cfg *cfg = priv->port_cfgs;

	property_enable(priv, &cfg->usb3tousb2_en, value);
	property_enable(priv, &cfg->usb3_host_disable, value);
	property_enable(priv, &cfg->usb3_host_port, !value);

	return 0;
}

static int rockchip_usb3_phy_power_on(struct phy *phy)
{
	struct udevice *parent = dev_get_parent(phy->dev);
	struct rockchip_tcphy *priv = dev_get_priv(parent);
	const struct rockchip_usb3phy_port_cfg *cfg = priv->port_cfgs;
	const struct usb3phy_reg *reg = &cfg->pipe_status;
	int timeout, new_mode;
	u32 val;
	int ret;

	new_mode = rockchip_tcphy_get_mode(phy->dev, priv);
	if (new_mode < 0) {
		dev_err(phy->dev, "invalid mode %d\n", new_mode);
		return new_mode;
	}

	/*
	 * The connector negotiated 4-lane DisplayPort, so no lanes are left
	 * for SuperSpeed: point this port at USB 2 and leave the PHY alone.
	 *
	 * This used to fall through, and the fall-through was the bug.
	 * rockchip_tcphy_init() would be entered with priv->mode still
	 * MODE_DISCONNECT, take the USB3 branch, and write PIN_ASSIGN_D_F over
	 * a DisplayPort link -- and then the bookkeeping below would OR in
	 * "new_mode & (MODE_DFP_USB | MODE_UFP_USB)", which for MODE_DFP_DP is
	 * zero. The PHY was left fully clocked and out of reset while
	 * priv->mode still read MODE_DISCONNECT, so power_off took its
	 * "nothing to do" exit and the clocks and resets were never released.
	 */
	if (!(new_mode & (MODE_DFP_USB | MODE_UFP_USB))) {
		tcphy_cfg_usb3_to_usb2_only(priv, true);
		return 0;
	}

	ret = rockchip_tcphy_set_mode(phy, priv, new_mode);
	if (ret)
		return ret;

	/* wait TCPHY for pipe ready */
	for (timeout = 0; timeout < 100; timeout++) {
		val = readl(priv->grf_base + reg->offset);
		if (!(val & BIT(reg->enable_bit))) {
			/* enable usb3 host */
			tcphy_cfg_usb3_to_usb2_only(priv, false);
			return 0;
		}
		usleep_range(10, 20);
	}

	/*
	 * The pipe never came ready, so the PHY is not usable in this mode.
	 * Unwind all the way to MODE_DISCONNECT: the old code tore down only
	 * when priv->mode still happened to read MODE_DISCONNECT, which was
	 * true for a cold PHY and false for a re-configuration -- exactly the
	 * case where a leaked configuration does damage.
	 */
	rockchip_tcphy_exit(priv);
	priv->mode = MODE_DISCONNECT;

	return -ETIMEDOUT;
}

static int rockchip_usb3_phy_power_off(struct phy *phy)
{
	struct udevice *parent = dev_get_parent(phy->dev);
	struct rockchip_tcphy *priv = dev_get_priv(parent);

	/*
	 * Tearing down the USB3 half must not touch a PHY that is currently
	 * DisplayPort. rockchip_dp_phy_power_on() deliberately parks the USB3
	 * controller on USB 2 (tcphy_cfg_usb3_to_usb2_only(priv, true))
	 * because in a 4-lane pin assignment there are no lanes left for
	 * SuperSpeed; calling this with false would clear usb3_host_disable
	 * and set usb3_host_port, pointing the USB3 controller back at lanes
	 * carrying a live DP link. Anything that tears down the dwc3/xhci
	 * glue -- `usb stop`, a re-probe, `usb reset` -- reaches here, so the
	 * guard has to come first.
	 */
	if (priv->mode == MODE_DFP_DP)
		return 0;

	tcphy_cfg_usb3_to_usb2_only(priv, false);

	if (priv->mode == MODE_DISCONNECT)
		return 0;

	/*
	 * A plain compare, not the old `mode &= ~(MODE_UFP_USB |
	 * MODE_DFP_USB)`. priv->mode is a single mode, never an OR of modes
	 * -- rockchip_tcphy_set_mode() is its only assigner outside the
	 * failure unwinds -- so the mask arithmetic was the last survivor of
	 * the scheme this driver retired, working only because clearing both
	 * USB bits from one USB mode happens to yield MODE_DISCONNECT.
	 */
	priv->mode = MODE_DISCONNECT;
	rockchip_tcphy_exit(priv);

	return 0;
}

/*
 * The controller's USB role, before power-on (dwc3_setup_phy_mode()). Only a
 * record: the lane setup is the same either way; it is what
 * rockchip_tcphy_get_mode() reports until a port manager says otherwise.
 */
static int rockchip_usb3_phy_set_mode(struct phy *phy, enum phy_mode mode,
				      int submode)
{
	struct rockchip_tcphy *priv = dev_get_priv(dev_get_parent(phy->dev));

	if (mode >= PHY_MODE_USB_DEVICE && mode <= PHY_MODE_USB_DEVICE_SS)
		priv->usb_role = MODE_UFP_USB;
	else if (mode >= PHY_MODE_USB_HOST && mode <= PHY_MODE_USB_HOST_SS)
		priv->usb_role = MODE_DFP_USB;
	else
		return -EINVAL;

	return 0;
}

static struct phy_ops rockchip_tcphy_usb3_ops = {
	.set_mode = rockchip_usb3_phy_set_mode,
	.power_on = rockchip_usb3_phy_power_on,
	.power_off = rockchip_usb3_phy_power_off,
};

/*
 * The DP half of the PHY is a SEPARATE phy device, bound from the dp-port
 * subnode, and that is the whole point of it rather than a convenience.
 *
 * rockchip_usb3_phy_power_on() runs at `usb start`, which on this board can
 * happen BEFORE `tcpm dp enter` ever has, and does not run again afterwards.
 * A design where only the USB3 phy pulls the negotiated link would capture
 * whatever the connector was at that one moment, and entering DisplayPort
 * later could never re-mux the lanes. Giving DP its own phy gives it its own
 * power_on, called by its own consumer -- the display controller -- at the
 * time the display is actually wanted.
 */
static int __maybe_unused rockchip_dp_phy_power_on(struct phy *phy)
{
	struct udevice *parent = dev_get_parent(phy->dev);
	struct rockchip_tcphy *priv = dev_get_priv(parent);
	const struct rockchip_usb3phy_port_cfg *cfg = priv->port_cfgs;
	int new_mode, ret;
	u32 val;

	new_mode = rockchip_tcphy_get_mode(phy->dev, priv);
	if (new_mode < 0) {
		dev_err(phy->dev, "invalid mode %d\n", new_mode);
		return new_mode;
	}

	if (new_mode != MODE_DFP_DP) {
		dev_warn(phy->dev, "connector is not in 4-lane DisplayPort\n");
		return -ENODEV;
	}

	/*
	 * Route the DP controller's output to this Type-C phy.
	 *
	 * Asymmetric on rk3399, and deliberately left that way. SOC_CON26
	 * bit 3 selects WHICH of the two Type-C phys the DP controller feeds,
	 * and the port-0 row encodes "bit 3 = 0" by putting the enable bit on
	 * top of the write-enable bit ({ 0x6268, 19, 19 }, so
	 * property_enable()'s "val | mask" is 1 << 19 for either value of
	 * @en). Clearing it is therefore a no-op for port 0 -- correctly so,
	 * port 0 has nothing to hand back -- and a real hand-back for port 1
	 * ({ 0x6268, 3, 19 }). Linux carries the identical table. Do not
	 * "fix" the port-0 row.
	 */
	property_enable(priv, &cfg->uphy_dp_sel, 1);

	/*
	 * The transition this phy exists for. `usb start` runs before
	 * `tcpm dp enter` and does not run again, so this arrives with
	 * priv->mode already MODE_DFP_USB. The setup used to be gated on
	 * "priv->mode == MODE_DISCONNECT", which in exactly that order skipped
	 * rockchip_tcphy_init() outright: PMA_LANE_CFG left at PIN_ASSIGN_D_F,
	 * lanes 0 and 1 still carrying USB3 TX/RX config, the AUX channel
	 * never calibrated -- and then it entered A0 on that and returned
	 * success. Which is to say the dp-port phy did not do the one thing it
	 * was split out to do.
	 */
	ret = rockchip_tcphy_set_mode(phy, priv, MODE_DFP_DP);
	if (ret)
		goto err_dp_sel;

	/*
	 * All four lanes now carry DisplayPort, so the USB3 controller's port
	 * has none: point it at USB 2 rather than leaving it looking at lanes
	 * that were re-muxed out from under it. USB 2.0 is duplicated across
	 * the receptacle and is unaffected.
	 */
	tcphy_cfg_usb3_to_usb2_only(priv, true);

	writel(DP_MODE_ENTER_A0, priv->reg_base + DP_MODE_CTL);

	/* Achieved state is reported four bits above the request. */
	ret = readl_poll_sleep_timeout(priv->reg_base + DP_MODE_CTL, val,
				       val & DP_MODE_A0, 1000,
				       PHY_MODE_SET_TIMEOUT);
	if (ret < 0) {
		dev_err(phy->dev, "failed to wait TCPHY enter A0\n");
		goto err_a0;
	}

	return 0;

err_a0:
	/*
	 * Park the PHY in A2 and then tear it down completely.
	 *
	 * Parking alone was the whole of the old error path and was not
	 * enough. rockchip_tcphy_init() returns from its DisplayPort branch
	 * with pipe_rst still asserted -- DP takes no pipe -- so a PHY left
	 * here sits in the 4-lane DP lane configuration with no pipe while
	 * priv->mode claims MODE_DFP_DP. The next `usb start` then found a
	 * mode that was neither DISCONNECT nor what it wanted, skipped init,
	 * and polled a pipe-status bit that cannot clear while pipe_rst is
	 * held: 100 iterations, -ETIMEDOUT, and no SuperSpeed on that port
	 * until a board reset. Going all the way back to MODE_DISCONNECT
	 * means the next caller re-initialises from cold, which always works.
	 */
	writel(DP_MODE_ENTER_A2, priv->reg_base + DP_MODE_CTL);
	rockchip_tcphy_exit(priv);
	priv->mode = MODE_DISCONNECT;
	/*
	 * Deliberately NOT restoring the usb3-to-usb2 GRF bits here. Doing so
	 * sets usb3_host_port on a PHY whose pipe reset was just asserted and
	 * whose clocks are off -- a state rockchip_tcphy_pre_init() never
	 * produces -- so it advertises a dead PHY as a live SuperSpeed port.
	 * It also buys nothing: the next successful
	 * rockchip_usb3_phy_power_on() calls tcphy_cfg_usb3_to_usb2_only(priv,
	 * false) itself before it touches anything, so SuperSpeed comes back
	 * on the next attach either way.
	 */
err_dp_sel:
	property_enable(priv, &cfg->uphy_dp_sel, 0);

	return ret;
}

static int __maybe_unused rockchip_dp_phy_power_off(struct phy *phy)
{
	struct udevice *parent = dev_get_parent(phy->dev);
	struct rockchip_tcphy *priv = dev_get_priv(parent);
	const struct rockchip_usb3phy_port_cfg *cfg = priv->port_cfgs;

	/*
	 * Exact equality, not a mask test, and it matches the invariant
	 * rockchip_tcphy_get_mode() maintains: MODE_DFP_DP is never ORed with
	 * a USB mode here, because this port never negotiates the 2-lane pin
	 * assignment D. A power_off arriving while the PHY is in a USB3
	 * configuration is not ours to tear down.
	 */
	if (priv->mode != MODE_DFP_DP)
		return 0;

	writel(DP_MODE_ENTER_A2, priv->reg_base + DP_MODE_CTL);

	/*
	 * Give the DP controller's mux back. It was claimed in power_on and
	 * nothing ever released it -- not here, not on either failure path --
	 * so the controller stayed pointed at a Type-C phy that had been
	 * powered down. (A no-op for port 0; see the comment in power_on.)
	 */
	property_enable(priv, &cfg->uphy_dp_sel, 0);

	rockchip_tcphy_exit(priv);
	priv->mode = MODE_DISCONNECT;

	return 0;
}

/*
 * DisplayPort is U-Boot proper's: an xPL phase brings the PHY up for USB 3 at
 * most (an SPL USB flash session), so it neither binds the DP port nor links
 * its code.
 */
#if !IS_ENABLED(CONFIG_XPL_BUILD)
static struct phy_ops rockchip_tcphy_dp_ops = {
	.power_on = rockchip_dp_phy_power_on,
	.power_off = rockchip_dp_phy_power_off,
};
#endif

static void rockchip_tcphy_pre_init(struct udevice *dev)
{
	struct rockchip_tcphy *priv = dev_get_priv(dev);
	const struct rockchip_usb3phy_port_cfg *cfg = priv->port_cfgs;

	reset_assert(&priv->tcphy_rst);
	reset_assert(&priv->uphy_rst);
	reset_assert(&priv->pipe_rst);

	/* select external psm clock */
	property_enable(priv, &cfg->external_psm, 1);
	property_enable(priv, &cfg->usb3tousb2_en, 0);

	priv->mode = MODE_DISCONNECT;
}

static int rockchip_tcphy_parse_dt(struct udevice *dev)
{
	struct rockchip_tcphy *priv = dev_get_priv(dev);
	int ret;

	priv->grf_base = syscon_get_first_range(ROCKCHIP_SYSCON_GRF);
	if (IS_ERR(priv->grf_base))
		return PTR_ERR(priv->grf_base);

	ret = clk_get_by_name(dev, "tcpdcore", &priv->clk_core);
	if (ret) {
		dev_err(dev, "failed to get tcpdcore clk (ret=%d)\n", ret);
		return ret;
	}

	ret = clk_get_by_name(dev, "tcpdphy-ref", &priv->clk_ref);
	if (ret) {
		dev_err(dev, "failed to get tcpdphy-ref clk (ret=%d)\n", ret);
		return ret;
	}

	ret = reset_get_by_name(dev, "uphy", &priv->uphy_rst);
	if (ret) {
		dev_err(dev, "failed to get uphy reset (ret=%d)\n", ret);
		return ret;
	}

	ret = reset_get_by_name(dev, "uphy-pipe", &priv->pipe_rst);
	if (ret) {
		dev_err(dev, "failed to get uphy-pipe reset (ret=%d)\n", ret);
		return ret;
	}

	ret = reset_get_by_name(dev, "uphy-tcphy", &priv->tcphy_rst);
	if (ret) {
		dev_err(dev, "failed to get uphy-tcphy reset (ret=%d)\n", ret);
		return ret;
	}

	return 0;
}

static int rockchip_tcphy_probe(struct udevice *dev)
{
	struct rockchip_tcphy *priv = dev_get_priv(dev);
	const struct rockchip_usb3phy_port_cfg *phy_cfgs;
	unsigned int reg;
	int index, ret;

	priv->reg_base = dev_read_addr_ptr(dev);
	if (!priv->reg_base)
		return -EINVAL;

	ret = dev_read_u32_index(dev, "reg", 1, &reg);
	if (ret) {
		dev_err(dev, "failed to read reg property (ret = %d)\n", ret);
		return ret;
	}

	phy_cfgs = (const struct rockchip_usb3phy_port_cfg *)
					dev_get_driver_data(dev);
	if (!phy_cfgs)
		return -EINVAL;

	/* find out a proper config which can be matched with dt. */
	index = 0;
	while (phy_cfgs[index].reg) {
		if (phy_cfgs[index].reg == reg) {
			priv->port_cfgs = &phy_cfgs[index];
			break;
		}

		++index;
	}

	if (!priv->port_cfgs) {
		dev_err(dev, "failed find proper phy-cfg\n");
		return -EINVAL;
	}

	ret = rockchip_tcphy_parse_dt(dev);
	if (ret)
		return ret;

	rockchip_tcphy_pre_init(dev);

	return 0;
}

static int rockchip_tcphy_bind(struct udevice *dev)
{
	struct udevice *tcphy_dev;
	ofnode node;
	const char *name;
	int ret = 0;

	dev_for_each_subnode(node, dev) {
		if (!ofnode_valid(node)) {
			dev_info(dev, "subnode %s not found\n", dev->name);
			return -ENXIO;
		}

		name = ofnode_get_name(node);
		dev_dbg(dev, "subnode %s\n", name);

		if (!strcasecmp(name, "dp-port")) {
			if (IS_ENABLED(CONFIG_XPL_BUILD))
				continue;
			ret = device_bind_driver_to_node(dev,
							 "rockchip_tcphy_dp_port",
							 name, node, &tcphy_dev);
			if (ret) {
				dev_err(dev,
					"'%s' cannot bind 'rockchip_tcphy_dp_port'\n",
					name);
				return ret;
			}
		} else if (!strcasecmp(name, "usb3-port")) {
			ret = device_bind_driver_to_node(dev,
							 "rockchip_tcphy_usb3_port",
							 name, node, &tcphy_dev);
			if (ret) {
				dev_err(dev,
					"'%s' cannot bind 'rockchip_tcphy_usb3_port'\n",
					name);
				return ret;
			}
		}
	}

	return ret;
}

static const struct rockchip_usb3phy_port_cfg rk3399_typec_phy_cfgs[] = {
	{
		.reg			= 0xff7c0000,
		.typec_conn_dir		= { 0xe580, 0, 16 },
		.usb3tousb2_en		= { 0xe580, 3, 19 },
		.external_psm		= { 0xe588, 14, 30 },
		.pipe_status		= { 0xe5c0, 0, 0 },
		.usb3_host_disable	= { 0x2434, 0, 16 },
		.usb3_host_port		= { 0x2434, 12, 28 },
		.uphy_dp_sel		= { 0x6268, 19, 19 },
	},
	{
		.reg			= 0xff800000,
		.typec_conn_dir		= { 0xe58c, 0, 16 },
		.usb3tousb2_en		= { 0xe58c, 3, 19 },
		.external_psm		= { 0xe594, 14, 30 },
		.pipe_status		= { 0xe5c0, 16, 16 },
		.usb3_host_disable	= { 0x2444, 0, 16 },
		.usb3_host_port		= { 0x2444, 12, 28 },
		.uphy_dp_sel		= { 0x6268, 3, 19 },
	},
	{ /* sentinel */ }
};

static const struct udevice_id rockchip_typec_phy_ids[] = {
	{
		.compatible = "rockchip,rk3399-typec-phy",
		.data = (ulong)&rk3399_typec_phy_cfgs,
	},
	{ /* sentinel */ }
};

U_BOOT_DRIVER(rockchip_tcphy_usb3_port) = {
	.name		= "rockchip_tcphy_usb3_port",
	.id		= UCLASS_PHY,
	.ops		= &rockchip_tcphy_usb3_ops,
};

#if !IS_ENABLED(CONFIG_XPL_BUILD)
U_BOOT_DRIVER(rockchip_tcphy_dp_port) = {
	.name		= "rockchip_tcphy_dp_port",
	.id		= UCLASS_PHY,
	.ops		= &rockchip_tcphy_dp_ops,
};
#endif

U_BOOT_DRIVER(rockchip_typec_phy) = {
	.name	= "rockchip_typec_phy",
	.id	= UCLASS_NOP,
	.of_match = rockchip_typec_phy_ids,
	.probe = rockchip_tcphy_probe,
	.bind = rockchip_tcphy_bind,
	.priv_auto	= sizeof(struct rockchip_tcphy),
};
