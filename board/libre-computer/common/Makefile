# SPDX-License-Identifier: GPL-2.0+
ifeq ($(CONFIG_XPL_BUILD),y)
# xPL: no board code, except the bare SPIFC NOR reader (RDID + READ only),
# which the SPL flash session's read-only NOR alternates use
# (arch/arm/mach-meson/spl-usbflash.c, CONFIG_SPL_DFU_VIRT), and a GX chain
# that boots from NOR reads its stages with (CONFIG_MESON_GX_LOAD_NOR, both
# phases).
obj- := __dummy__.o
ifeq ($(CONFIG_ARCH_MESON),y)
ifneq ($(CONFIG_$(PHASE_)DFU_VIRT)$(CONFIG_MESON_GX_LOAD_NOR),)
obj-y	+= meson-spifc-nor.o
endif
endif
else
obj-y	:= board.o
obj-$(CONFIG_ARCH_MESON)	+= board-meson.o
obj-$(CONFIG_ARCH_MESON)	+= meson-spifc-nor.o
obj-$(CONFIG_ARCH_SUNXI)	+= board-sunxi.o
obj-$(CONFIG_ARCH_ROCKCHIP)	+= board-rockchip.o
endif
