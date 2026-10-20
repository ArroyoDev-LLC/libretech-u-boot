# SPDX-License-Identifier: GPL-2.0+
#
# SoC-scoped link flags for arm64 Amlogic (Meson) targets.
#
# config.mk:44-47 includes, in order, arch/$(ARCH)/config.mk, then
# $(CPUDIR)/config.mk, then -- when CONFIG_SYS_SOC is set --
# $(CPUDIR)/$(SOC)/config.mk.  With CONFIG_SYS_CPU="armv8" (arch/arm/Kconfig,
# `default "armv8" if ARM64`) and CONFIG_SYS_SOC="meson"
# (arch/arm/mach-meson/Kconfig, `config SYS_SOC / default "meson"`), that
# resolves to this file.  `sinclude` means it is simply absent until it
# exists, which is why adding it states a policy for arm64 Amlogic ONLY and
# cannot reach Rockchip, Allwinner, MediaTek or any other arm64 target.
#
# Cortex-A53 erratum 843419: select the ADR remedy rather than the veneer.
#
# The workaround is NOT disabled here.  ld's --fix-cortex-a53-843419 takes an
# argument (GNU ld --help): `full` (the GCC driver's LINK_SPEC default) emits a
# 4096 B veneer section, `adr` rewrites the vulnerable adrp in place and costs
# zero bytes.  Detection is identical under both; the only failure mode `adr`
# adds is "referenced address out of range of an ADR", which ld reports as a
# HARD LINK ERROR.
#
# This is deliberately NOT -mno-fix-cortex-a53-843419, which DISABLES the
# workaround on a presumptively-affected part.  That stays banned.  See
# claude:amlogic/software/boot/gxl/dram/design/spl-size-budget.md 9.4 and 10.3.
#
# Scoped to the xPL stages, and that scope is the whole argument.
#
# ADR reaches +/-1 MB.  Every Amlogic SPL/TPL is an SRAM stage bounded by
# CONFIG_{SPL,TPL}_MAX_SIZE = 0xb000 (the BootROM amlimage payload cap,
# tools/amlimage.c VARIANT("gxl", 1, 1, 0xb000)) or, for the GXL DRAM stage,
# MESON_GX_SPL_MAX_SIZE = 0x13e00 -- so every intra-image reference is inside
# 128 KB and the out-of-range failure cannot arise.  BL33 (u-boot proper) is
# ~1.1 MB on these targets, i.e. within a factor of the ADR range rather than
# three orders below it, and it has no size cap that the 4096 B veneer quantum
# can decide.  So the stage boundary is exactly where the fact lives: under it,
# `adr` is free and the quantum decides whether a build links; over it, `full`
# costs nothing that matters.
#
# What this buys, measured at v2026.07/lc-master e8fba1cfb00: the two shipping
# open-chain SPLs libretech-ac_libre_defconfig (41035 B) and
# libretech-cc_v2_libre_defconfig (40987 B) are stub-free with 4021 B and
# 4069 B of nominal spare, i.e. guaranteed-usable -75 B and -27 B, and the size
# gate calls both EXPOSED: an edit of ANY size, including one that removes
# code, can instantiate the veneer and put them over the cap.  With the ADR
# remedy no veneer can be instantiated, guaranteed-usable == nominal spare, and
# both profiles are OK.
#
# Supersedes the narrower CONFIG_MESON_DRAM_FAULT_CLASS guard this file carries
# on topic/gxl-chain-instr (ff54dc52a12): that scope was chosen to guarantee no
# shipping artifact changed, which structurally excluded the shipping profiles
# that turned out to need it.
#
# Except a stage whose BSS is placed 1 MiB or more from its text.  The DRAM
# stages (G12A/SM1 and GXL: text at 0x08000000, BSS in SRAM at 0xfffb0000 /
# 0xd900d000) are not the SRAM stages the reach argument above is about: a
# vulnerable adrp that targets BSS has no ADR remedy, and ld then fails the
# link (`erratum 843419 immediate ... out of range for ADR`; seen on the SM1
# DRAM stage of v2026.07/lc-splflash, 2026-10-03).  Whether such an adrp
# exists is decided by the layout, so any edit can break the link.  Such a
# stage takes `full`, the toolchain default, which can cost one 4096 B veneer
# section.  The test is the phase's own CONFIG_<PHASE>_TEXT_BASE and
# CONFIG_<PHASE>_BSS_START_ADDR; a phase without a separate BSS address
# keeps `adr`.
ifdef CONFIG_XPL_BUILD
xpl_843419_text := $(CONFIG_$(PHASE_)TEXT_BASE)
xpl_843419_bss := $(CONFIG_$(PHASE_)BSS_START_ADDR)
xpl_843419_far := $(if $(and $(xpl_843419_text),$(xpl_843419_bss)),$(shell \
	d=$$(( $(xpl_843419_bss) - $(xpl_843419_text) )); \
	[ $$d -lt 0 ] && d=$$(( -d )); [ $$d -ge 1048576 ] && echo y))
ifeq ($(xpl_843419_far),y)
LDFLAGS_FINAL += --fix-cortex-a53-843419=full
else
LDFLAGS_FINAL += --fix-cortex-a53-843419=adr
endif
endif
