# SPDX-License-Identifier: GPL-2.0+
#
# Copyright (c) 2013 Samsung Electronics Co., Ltd.
#	http://www.samsung.com

obj-$(CONFIG_EXYNOS_ACE_SHA)	+= ace_sha.o
obj-y += aes/
#
# The RSA modular-exponentiation uclass is only ever a consumer of lib/rsa/,
# and lib/Makefile descends into that directory per PHASE
# (obj-$(CONFIG_$(PHASE_)RSA) += rsa/).  This directory was descended
# unconditionally, and rsa_mod_exp/Makefile keys its objects off the
# non-phase CONFIG_RSA / CONFIG_RSA_SOFTWARE_EXP, so an xPL phase with
# CONFIG_SPL_CRYPTO=y but CONFIG_SPL_RSA=n compiled mod_exp_sw.o into SPL
# while lib/rsa/rsa-mod-exp.c was left out -- an undefined reference to
# rsa_mod_exp_sw() at link time (under LTO, a linker SIGSEGV rather than a
# readable diagnostic).  That combination is exactly a hash-only
# SPL_FIT_SIGNATURE, which is what the GXL -libre chain uses.  Gate on the
# same per-phase symbol lib/Makefile uses so the two agree.
#
obj-$(CONFIG_$(PHASE_)RSA) += rsa_mod_exp/
obj-y += fsl/
obj-y += hash/
obj-y += aspeed/
obj-y += nuvoton/
obj-y += tegra/
