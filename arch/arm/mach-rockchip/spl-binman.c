// SPDX-License-Identifier: GPL-2.0+
/*
 * The optional binman symbols of the Rockchip SPL (spl.c's RAM boot: where
 * binman put a payload in the image, if it did). Their own object, built
 * without LTO (Makefile): binman reads "optional" from the weak binding,
 * and LTO turns a weak definition into a strong one.
 */

#include <binman_sym.h>
#include <linux/types.h>

#if CONFIG_IS_ENABLED(RAM_DEVICE) && IS_ENABLED(CONFIG_SPL_LOAD_FIT)
binman_sym_declare_optional(ulong, payload, image_pos);
binman_sym_declare_optional(ulong, payload, size);
#endif
