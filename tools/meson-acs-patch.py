#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""
Patch Amlogic ACS (Amlogic Config Structure) into SPL binary.

The GXL ROM reads byte 4-5 of the BL2/SPL binary as an ACS pointer.
ACS contains pll_set_t with CPU clock and voltage settings that the
ROM uses to configure the SoC before jumping to the binary.

Usage:
    meson-acs-patch.py spl/u-boot-spl.bin [--cpu-clk 1200] [--vddee 1000] [--vcck 1120]
"""

import argparse
import struct
import sys


def build_acs(cpu_clk=1200, vddee=1000, vcck=1120):
    """Build ACS binary data matching vendor format exactly."""

    # pll_set_t: 64 bytes (vendor uses 0x40)
    pll = bytearray(64)
    struct.pack_into('<I', pll, 0, cpu_clk)         # cpu_clk
    struct.pack_into('<I', pll, 4, 0)               # spi_ctrl
    struct.pack_into('<I', pll, 8, (vcck << 16) | vddee)  # packed
    # rest is zeros (matches vendor)

    # ACS header: 80 bytes structure, but acs_len field = total ACS size
    # Vendor uses acs_len=0x100 (256) which includes padding
    hdr = bytearray(80)
    hdr[0:5] = b'acs__'
    hdr[5] = 0x24       # chip_type
    struct.pack_into('<H', hdr, 6, 0x0100)  # acs_len = 256 (vendor value)
    struct.pack_into('<I', hdr, 8, 0x40)    # pad1 = 0x40 (vendor value)

    # ddrs_ entry (no DDR config -- SPL handles DDR init internally)
    hdr[16:21] = b'ddrs_'
    hdr[21] = 2          # version
    struct.pack_into('<H', hdr, 22, 0)  # length = 0
    struct.pack_into('<Q', hdr, 24, 0)  # address = 0

    # ddrt_ entry (no DDR timings)
    hdr[32:37] = b'ddrt_'
    hdr[37] = 1          # version
    struct.pack_into('<H', hdr, 38, 0)  # length = 0
    struct.pack_into('<Q', hdr, 40, 0)  # address = 0

    # pll__ entry
    hdr[48:53] = b'pll__'
    hdr[53] = 1          # version
    struct.pack_into('<H', hdr, 54, 0x40)  # length = 64 (vendor value)
    # pll_addr at offset 56 -- patched later

    return hdr + pll


def main():
    parser = argparse.ArgumentParser(description="Patch ACS into Meson SPL")
    parser.add_argument("spl", help="SPL binary to patch (modified in-place)")
    parser.add_argument("--cpu-clk", type=int, default=1200, help="CPU clock MHz")
    parser.add_argument("--vddee", type=int, default=1000, help="VDDEE voltage mV")
    parser.add_argument("--vcck", type=int, default=1120, help="VCCK voltage mV")
    args = parser.parse_args()

    with open(args.spl, "rb") as f:
        spl = bytearray(f.read())

    acs_data = build_acs(args.cpu_clk, args.vddee, args.vcck)
    acs_offset = len(spl)

    # Patch pll_addr in ACS header to point to pll_set_t in SRAM
    # SPL loads at SRAM (0xd9000000 for GXL after amlimage extraction)
    # The amlimage payload starts at file offset 0xff0 in the image
    # So SRAM address = 0xd9000000 + 0xff0 + acs_offset + pll_offset
    # But the ROM reads pll_addr as absolute SRAM address
    pll_data_offset = 80  # pll_set_t follows ACS header (80 bytes)
    pll_sram_addr = 0xd9000000 + 0xff0 + acs_offset + pll_data_offset
    struct.pack_into('<Q', acs_data, 56, pll_sram_addr)

    # Set byte 4-5: ACS entry point
    # ROM convention: value at byte 4-5 = SRAM offset + 0x1000
    acs_entry = acs_offset + 0x1000
    struct.pack_into('<H', spl, 4, acs_entry)
    # Set byte 6-7: SRAM high bytes
    struct.pack_into('<H', spl, 6, 0xd900)

    # Append ACS data
    spl.extend(acs_data)

    with open(args.spl, "wb") as f:
        f.write(spl)

    total = len(spl)
    print(f"ACS patched: cpu_clk={args.cpu_clk} vddee={args.vddee} vcck={args.vcck}")
    print(f"  ACS at offset 0x{acs_offset:x}, entry=0x{acs_entry:x}")
    print(f"  pll_addr=0x{pll_sram_addr:x}")
    print(f"  Total: {total} bytes (max 45056)")
    if total > 45056:
        print(f"  WARNING: exceeds 0xb000 payload limit!", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
