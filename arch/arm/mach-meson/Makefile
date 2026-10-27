# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (c) 2016 Beniamino Galvani <b.galvani@gmail.com>

obj-y += board-common.o sm.o board-info.o capsule.o chip-serial.o
obj-y += wdt-reset.o usb-host-present.o
# The bootsettings command's decode: U-Boot proper only (TPL/SPL have no command line).
ifeq ($(CONFIG_XPL_BUILD),)
obj-$(CONFIG_CMD_BOOTSETTINGS) += bootsettings.o
endif
obj-$(CONFIG_MESON_GX) += board-gx.o
ifneq ($(filter y,$(CONFIG_SPL_BUILD) $(CONFIG_TPL_BUILD)),)
obj-$(CONFIG_MESON_GXBB) += dram-gxbb.o
obj-$(CONFIG_MESON_GXL) += dram-gxl.o
# Strategy A (SM1 residual-opensc / vendor .fw orchestrator).
# Strategy B Phase 1 (Alta open PLL) replaces its dram_init() — do not
# link both (duplicate dram_init).
ifneq ($(CONFIG_MESON_G12_OPEN_PLL),y)
obj-$(CONFIG_MESON_G12A) += dram-g12a.o
endif
obj-$(CONFIG_MESON_G12_OPEN_PLL) += dram-g12.o
# G12B board power (strong meson_power_init).  Own symbol, so Alta can run
# the Strategy A orchestrator above with the G12B power sequence.
obj-$(CONFIG_MESON_G12B_POWER_INIT) += spl-g12b.o
# Phase-scoped USB (GXL).  $(PHASE_) matches CONFIG_IS_ENABLED(MESON_USB_BOOT).
obj-$(CONFIG_$(PHASE_)MESON_USB_BOOT) += meson-usb.o
obj-$(CONFIG_$(PHASE_)MESON_USB_BOOT) += meson-u1-asm.o
# G12A/SM1 TPL residual only (Option 2).  Not GXL meson-usb.o.
obj-$(CONFIG_$(PHASE_)MESON_G12A_USB_RESIDUAL) += meson-usb-g12.o
# G12A/SM1 TPL register-level eMMC reader (no DM, no DTB).  Read-only.
obj-$(CONFIG_$(PHASE_)MESON_G12A_EMMC_REG) += tpl-emmc-g12.o
obj-$(CONFIG_MESON_GX) += dram-gx.o
obj-$(CONFIG_MESON_GXBB) += spl-gxbb.o
obj-$(CONFIG_MESON_GXL) += spl-gxl.o
obj-$(CONFIG_MESON_GX) += spl-gx.o
obj-$(CONFIG_MESON_GX) += spl-boot-debug.o
# G12A/SM1 + GXL open first stage: board_init_f in spl.c (also TPL phase)
obj-$(CONFIG_ARCH_MESON) += spl.o
# DRAM stage of TPL->SPL->FIT (G12A/SM1).  Built both phases; C-gated in TPL.
obj-$(CONFIG_MESON_G12A_XPL_CHAIN) += spl-dram-stage.o
# G12A/SM1 SC (BL30) start.  Body is #if'd on FIT_IMAGE_POST_PROCESS, which is
# an SPL-phase-only option, so this compiles to nothing in TPL.
obj-$(CONFIG_MESON_G12A_XPL_CHAIN) += spl-g12a-scp.o
# G12A/SM1 + GXL DRAM stage: the Meson family of the USB flash session
# (common/spl/spl_usbflash.c), spl_board_init().  C-gated on the DRAM stages,
# so TPL compiles it to nothing.
obj-$(CONFIG_SPL_USBFLASH) += spl-usbflash.o
# G12A early banner (same ASM for TPL and single-stage SPL)
obj-$(CONFIG_MESON_G12A) += spl-boot-debug.o
# DRAM stage of TPL->SPL->FIT (GXL).  Built both phases; C-gated in TPL on
# MESON_GX_DRAM_STAGE.  O1: own file alongside SM1's spl-dram-stage.c — no
# shared collapse, no G12A_* rename.
obj-$(CONFIG_MESON_GX_XPL_CHAIN) += spl-dram-stage-gx.o
# GXL SC start: the DRAM SPL asks the on-chip SC-ROM to fetch the FIT "scp"
# node.  Body is #if'd on FIT_IMAGE_POST_PROCESS, an SPL-phase-only option, so
# this compiles to nothing in TPL.  Twin of spl-g12a-scp.o above; not TCM
# memcpy, not TPL.
obj-$(CONFIG_MESON_GX_XPL_CHAIN) += spl-gxl-scp.o
# GX chain boot log (UART transcript for the USB console).  Per phase.
obj-$(CONFIG_$(PHASE_)MESON_GX_BOOTLOG) += gx-bootlog.o
# TPL fetches the DRAM stage off SD/eMMC.  Build-only: never run.
obj-$(CONFIG_TPL_MESON_GX_LOAD_MMC) += tpl-load-mmc.o
# Boot from SPI NOR: TPL reads the DRAM stage, the DRAM stage reads the FIT,
# both through the bare SPIFC reader.  Each file is C-gated on its phase.
obj-$(CONFIG_MESON_GX_LOAD_NOR) += tpl-load-nor.o spl-load-nor.o
# TPL SPI NOR read probe (JEDEC ID + one block).  Read-only.
obj-$(CONFIG_TPL_MESON_GX_NOR_PROBE) += tpl-nor-probe.o
# TPL SD/eMMC read probe (bind + sector 0).  Read-only.
obj-$(CONFIG_TPL_MESON_GX_MMC_PROBE) += tpl-mmc-probe.o
# SPIFC parent-rate override: clk81 from the HIU, no CLK uclass.
obj-$(CONFIG_MESON_SPIFC) += spifc-clk-gx.o
endif

obj-$(CONFIG_MESON_AXG) += board-axg.o
obj-$(CONFIG_MESON_G12A) += board-g12a.o
# G12A/SM1 resident chain: TPL arms the USB fallback, the DRAM stage takes the
# FIT the TPL delivered, U-Boot proper disarms.  All three phases, C-gated.
obj-$(CONFIG_MESON_G12A_RESIDENT) += g12-resident.o
obj-$(CONFIG_MESON_A1) += board-a1.o
