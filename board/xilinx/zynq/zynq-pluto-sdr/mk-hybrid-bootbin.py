#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""Assemble a hybrid Zynq-7000 BOOT.BIN = stock FSBL + our u-boot, without bootgen.

The HamGeek/ANTSDR E310 (Zynq-7020) stock BOOT.BIN (QSPI mtd0) is a standard bootgen
image with two partitions: [0] fsbl.elf (load 0x0) and [1] u-boot.elf (load 0x04000000).
The BootROM loads only the FSBL; the FSBL then reads the partition-header table (PHT) and
loads partition 1 (u-boot) to its load address. Partition DATA checksums are disabled
(checksum_word_off = 0), so we can swap the u-boot partition without recomputing a data
checksum — only the PHT entry's length words and its header checksum need updating.

Our u-boot is built with TEXT_BASE=0x04000000, the exact load address the stock PHT already
specifies for u-boot, and u-boot.bin carries its DTB appended (OF_SEPARATE) — so the stock
FSBL's proven-good DDR init runs, then hands off to OUR broken-cd u-boot. This removes the
RE'd-ps7_init/DDR brick risk entirely: the DDR bring-up is the stock FSBL's.

Layout constants read from the stock image this session:
  boot header:        FSBL src 0x1700 len 0x18008 (load 0x0); IHT @0x8C0; PHT @0xC80
  PHT[0] FSBL @0xC80: data_word_off 0x5C0 (byte 0x1700), len 0x6002 w
  PHT[1] uboot@0xCC0: data_word_off 0x65D0 (byte 0x19740), load/exec 0x04000000, len patched
  u-boot data starts at byte 0x19740 (last partition -> just replace + extend)
Checksums are Xilinx ~sum: stored = ~(sum of preceding header words) & 0xFFFFFFFF.
"""
import struct, sys, hashlib

STOCK = sys.argv[1]          # stock mtd0 BOOT.BIN backup
UBOOT = sys.argv[2]          # our u-boot-dtb.bin (u-boot proper + appended DTB, TEXT_BASE 0x4000000)
OUT   = sys.argv[3]          # output hybrid BOOT.BIN

UBOOT_DATA_OFF = 0x19740     # byte offset where partition-1 (u-boot) data begins
PHT1           = 0xCC0       # partition-1 header offset
MTD0_SIZE      = 0x100000    # 1 MB partition ceiling

stock = bytearray(open(STOCK, "rb").read())
ub    = open(UBOOT, "rb").read()

# sanity: our u-boot must be word-aligned and carry a DTB (d00dfeed) so it is self-contained
if len(ub) % 4:
    ub = ub + b"\x00" * (4 - len(ub) % 4)
assert b"\xd0\x0d\xfe\xed" in ub, "u-boot binary has no appended DTB (need OF_SEPARATE u-boot-dtb.bin)"

# assemble: keep stock header+IHT+IH+PHT+FSBL verbatim up to the u-boot data, then our u-boot
hybrid = bytearray(stock[:UBOOT_DATA_OFF]) + bytearray(ub)
assert len(hybrid) <= MTD0_SIZE, f"hybrid {len(hybrid)} exceeds mtd0 {MTD0_SIZE}"

# patch PHT[1] length words (enc/unenc/total, all in 32-bit words) to our u-boot size
ub_words = len(ub) // 4
for off in (0x00, 0x04, 0x08):
    struct.pack_into("<I", hybrid, PHT1 + off, ub_words)

# recompute PHT[1] header checksum = ~sum(words 0..14) & 0xFFFFFFFF, stored at +0x3C
s = sum(struct.unpack_from("<I", hybrid, PHT1 + i*4)[0] for i in range(15)) & 0xFFFFFFFF
struct.pack_into("<I", hybrid, PHT1 + 0x3C, (~s) & 0xFFFFFFFF)

open(OUT, "wb").write(hybrid)

# report
def u(b, o): return struct.unpack_from("<I", b, o)[0]
print(f"stock FSBL: byte 0x1700 len 0x18008  sha256={hashlib.sha256(bytes(stock[0x1700:0x1700+0x18008])).hexdigest()[:24]}")
print(f"our u-boot: {UBOOT}  {len(ub)} bytes ({ub_words} words = 0x{ub_words:X})")
print(f"hybrid    : {OUT}  {len(hybrid)} bytes (0x{len(hybrid):X})  fits mtd0 1M: {len(hybrid)<=MTD0_SIZE}")
print(f"PHT[1] patched: len words=0x{u(hybrid,PHT1):X} load=0x{u(hybrid,PHT1+0x0C):08X} "
      f"exec=0x{u(hybrid,PHT1+0x10):08X} data_word_off=0x{u(hybrid,PHT1+0x14):X} "
      f"chksum=0x{u(hybrid,PHT1+0x3C):08X}")
print(f"boot-header checksum @0x48 unchanged=0x{u(hybrid,0x48):08X} (FSBL/BootROM path untouched)")
print(f"hybrid sha256={hashlib.sha256(bytes(hybrid)).hexdigest()}")
