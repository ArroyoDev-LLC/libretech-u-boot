/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2015-2017 Google, Inc
 *
 * Structured Vendor Defined Message definitions, trimmed to what U-Boot
 * needs to initiate a mode discovery and entry. The unstructured VDM,
 * cable/product-type and firmware-update definitions from the Linux
 * header are deliberately absent -- nothing here consumes them, and a
 * field nobody reads is a field nobody keeps correct.
 */

#ifndef __LINUX_USB_PD_VDO_H
#define __LINUX_USB_PD_VDO_H

#include "pd.h"

/*
 * VDM object is minimum of VDM header + 6 additional data objects.
 */
#define VDO_MAX_OBJECTS		6
#define VDO_MAX_SIZE		(VDO_MAX_OBJECTS + 1)

/*
 * VDM header
 * ----------
 * <31:16>  :: SVID
 * <15>     :: VDM type ( 1b == structured, 0b == unstructured )
 * <14:13>  :: Structured VDM version
 * <12:11>  :: reserved
 * <10:8>   :: object position (1-7 valid ... used for enter/exit mode only)
 * <7:6>    :: command type (SVDM only?)
 * <5>      :: reserved (SVDM), command type (UVDM)
 * <4:0>    :: command
 */
#define VDO(vid, type, ver, custom)			\
	(((vid) << 16) |				\
	 ((type) << 15) |				\
	 ((ver) << 13) |				\
	 ((custom) & 0x7FFF))

#define VDO_SVDM_TYPE		BIT(15)
#define VDO_SVDM_VERS(x)	((x) << 13)
#define VDO_OPOS(x)		((x) << 8)
#define VDO_CMDT(x)		((x) << 6)
#define VDO_SVDM_VERS_MASK	VDO_SVDM_VERS(0x3)
#define VDO_OPOS_MASK		VDO_OPOS(0x7)
#define VDO_CMDT_MASK		VDO_CMDT(0x3)

/* Discovery is addressed to the USB-IF PD SID, not to the alt mode's own */
#define USB_SID_PD		0xff00

#define CMDT_INIT		0
#define CMDT_RSP_ACK		1
#define CMDT_RSP_NAK		2
#define CMDT_RSP_BUSY		3

/* Structured VDM command */
#define CMD_DISCOVER_IDENT	1
#define CMD_DISCOVER_SVID	2
#define CMD_DISCOVER_MODES	3
#define CMD_ENTER_MODE		4
#define CMD_EXIT_MODE		5
#define CMD_ATTENTION		6

#define VDO_CMD_VENDOR(x)	(((0x10 + (x)) & 0x1f))

#define PD_VDO_VID(vdo)		((vdo) >> 16)
#define PD_VDO_SVDM(vdo)	(((vdo) >> 15) & 1)
#define PD_VDO_SVDM_VER(vdo)	(((vdo) >> 13) & 0x3)
#define PD_VDO_OPOS(vdo)	(((vdo) >> 8) & 0x7)
#define PD_VDO_CMD(vdo)		((vdo) & 0x1f)
#define PD_VDO_CMDT(vdo)	(((vdo) >> 6) & 0x3)

/* Structured VDM version */
#define SVDM_VER_1_0		0
#define SVDM_VER_2_0		1

/* ID header, VDO 1 of a Discover Identity response */
#define PD_IDH_MODAL_SUPP(vdo)	((vdo) & BIT(26))

/*
 * Discover SVIDs response packs two SVIDs per VDO, high half first, and
 * terminates on a zero SVID in either half. A response that is completely
 * full (6 VDOs, i.e. cnt == 7 including the header) with no zero in it means
 * "ask again" -- and the full-response test is load bearing on its own,
 * because some dockers stop short of 12 SVIDs without writing the 0x0000
 * terminator.
 */
#define PD_VDO_SVID_SVID0(vdo)	((vdo) >> 16)
#define PD_VDO_SVID_SVID1(vdo)	((vdo) & 0xffff)

/* Timers, in ms, from the USB PD specification */
#define PD_T_VDM_BUSY		100
#define PD_T_VDM_WAIT_MODE_E	100
#define PD_T_VDM_SNDR_RSP	30

#endif /* __LINUX_USB_PD_VDO_H */
