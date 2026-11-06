/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MT7601U MCU firmware, linked in at build time from the file named by
 * CONFIG_WIFI_MT7601U_FIRMWARE_PATH (the stock linux-firmware mt7601u.bin).
 * Nothing binary is kept in the tree.
 */

.section .rodata.mt7601u_fw,"a"
.balign 16
.global mt7601u_fw_start
mt7601u_fw_start:
.incbin MT7601U_FW_PATH
.global mt7601u_fw_end
mt7601u_fw_end:
.balign 16
