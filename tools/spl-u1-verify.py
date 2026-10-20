#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""Verify GXL SPL U1 interim: RUN open SPL -> BROM USB alive + OMOR marker.

Usage (on libre-computer, port 1 of PPPS hub):
  sudo PYTHONPATH=~/git/pyamlboot python3 tools/spl-u1-verify.py \\
      --image <build-dir>/spl/u-boot-spl-signed.bin.new [--ppps] [--port 1]

--image is REQUIRED: name the image from the tree you actually built.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time

for p in (
    os.environ.get("PYAMLBOOT_PATH"),
    os.path.expanduser("~/git/pyamlboot"),
    "/home/dxue/git/pyamlboot",
):
    if p and os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
        break

import usb.core
import usb.util
from pyamlboot import pyamlboot

VID, PID = 0x1B8E, 0xC003
SPL_LOAD = 0xD9000000
MARKER_ADDR = 0xD900C100
PROG_ADDR = 0xD900C104
# DWC2 core capability breadcrumbs, written by meson_usb_device_start()
# before any PHY/FIFO work. ARCHITECTURE is GHWCFG2[4:3].
GSNPSID_ADDR = 0xD900C11C
GHWCFG2_ADDR = 0xD900C124
MARKER_VAL = 0x524F4D4F  # OMOR le
TRAMP = struct.pack("<I", 0x14000400)


def fmt_hwcfg(snpsid: int, ghwcfg2: int) -> str:
    """Decode the two capability words into a one-line verdict.

    The question this answers: can this core do buffer-DMA at all? Three DMA
    attempts were made on it without ever reading GHWCFG2.
    """
    if not ghwcfg2 and not snpsid:
        return "hwcfg=absent (old SPL, or device_start not reached)"
    arch = (ghwcfg2 >> 3) & 0x3
    archname = {
        0: "SLAVE-ONLY(PIO; DMA_EN is a no-op)",
        1: "EXTERNAL-DMA",
        2: "INTERNAL-DMA(buffer-DMA possible)",
        3: "reserved",
    }[arch]
    opmode = ghwcfg2 & 0x7
    # GSNPSID is "OT" + BCD version, e.g. 0x4F54330A = OT3 v3.10a
    ver = f"{(snpsid >> 16) & 0xFFFF:04x}.{snpsid & 0xFFFF:04x}"
    return (
        f"snpsid=0x{snpsid:08x}(v{ver}) ghwcfg2=0x{ghwcfg2:08x} "
        f"arch={arch}:{archname} opmode={opmode}"
    )


def fmt_prog(v: int) -> str:
    bits = []
    names = (
        (0x0001, "START"),
        (0x0010, "WORLDCUP"),
        (0x0020, "PHYPOR"),
        (0x0100, "USBRST"),
        (0x0200, "ENUMDONE"),
        (0x0400, "RXFLVL"),
        (0x0800, "SETUPRX"),
        (0x1000, "SETUPDONE"),
        (0x2000, "ID_TX"),
        (0x4000, "ID_OK"),
        (0x8000, "IN_XFER"),
    )
    # High-half flags.  Only bits 26/27 are CLEAN there: ep0_tx() writes
    # NPTxFSpcAvail[11:0] into word bits 27:16 and this part's whole DFIFO is
    # 712 words, so the count cannot reach 1024 -- and ep0_tx() masks 26/27 off
    # explicitly besides.  U1P_DMA_ON (bit 16) and U1P_DMA_STALL (bit 18) sit
    # INSIDE the count and are deliberately not decoded here: a set bit there
    # cannot be told from a stale snapshot bit, so printing them as booleans
    # would be the same two-states-one-string defect this decoder exists to
    # avoid.  Only a clear bit would carry information, which is not a flag.
    hi_names = ((0x04000000, "STOP"),)
    lo = v & 0xFFFF
    # 31:28 is the NPTX request-queue nibble OR'd with NPTxFSpcAvail[15:12];
    # 27:16 is NPTxFSpcAvail[11:0].  fmt_prog() used to label the whole high
    # half "nptx_free", which folded the queue nibble into the free-dword count
    # and made every printed value wrong by up to 15 * 4096.
    #
    # The free count is read as 25:16, NOT 27:16: bits 26/27 are the flag
    # window above, and no count is lost taking only ten bits because the whole
    # DFIFO is 712 words.  15:12 of the field alias the queue nibble at 31:28
    # for the same reason -- a count that cannot reach 4096 never sets them --
    # so the nibble is clean.
    free = (v >> 16) & 0x03FF
    q = (v >> 28) & 0xF
    for bit, name in names:
        if lo & bit:
            bits.append(name)
    for bit, name in hi_names:
        if v & bit:
            bits.append(name)
    return f"0x{v:08x}[{','.join(bits) or '-'}] nptx_free={free} nptx_q={q}"


def log(m: str) -> None:
    print(m, flush=True)


def ppps_cycle(port: int, delay: float = 2.5) -> None:
    script = "/tmp/scripts/ppps-cycle2.py"
    if os.path.isfile(script):
        import subprocess

        subprocess.run(
            [
                "sudo",
                "-n",
                "env",
                "PYTHONUNBUFFERED=1",
                "python3",
                script,
                str(port),
                str(delay),
            ],
            check=False,
        )
        return
    rt, clear, set_, pp = 0x23, 1, 3, 8

    all_hubs = []
    for vid, pid in ((0x2109, 0x2817), (0x2109, 0x0817)):
        for d in usb.core.find(find_all=True, idVendor=vid, idProduct=pid):
            all_hubs.append(d)

    """Pick the parent hub for the board, not merely a top-level hub.

    The board can sit behind a CASCADED hub: on libre-computer its path is
    (2,3,2), i.e. port 2 of the hub at (2,3). Cycling port 2 of the hub at (2,)
    leaves the board powered, so the stale BROM survives and the first
    writeMemory fails with a pipe error. Match the hub whose port_numbers are
    exactly the board's path minus its last element.
    """
    target = None
    for d in usb.core.find(find_all=True, idVendor=VID, idProduct=PID):
        pn = tuple(getattr(d, "port_numbers", None) or ())
        if pn and pn[-1] == port:
            target = pn
            break

    hubs = []
    if target:
        want = target[:-1]
        for h in all_hubs:
            if tuple(getattr(h, "port_numbers", None) or ()) == want:
                hubs.append(h)
        if hubs:
            log(f"ppps: parent hub path={want} port={port} (board at {target})")
    if not hubs:
        # Cold start: no board enumerated, so its path is unknowable. Spray
        # every hub on the requested port, as before.
        hubs = [h for h in all_hubs
                if len(getattr(h, "port_numbers", None) or ()) <= 1]
        log(f"ppps: no board at port {port}; spraying {len(hubs)} top-level hub(s)")
    for h in hubs:
        try:
            h.ctrl_transfer(rt, clear, pp, port, None, timeout=2000)
        except Exception:
            pass
    time.sleep(delay)
    for h in hubs:
        try:
            h.ctrl_transfer(rt, set_, pp, port, None, timeout=2000)
        except Exception:
            pass
    log("PPPS done")


def wait_dev(port: int, t: float = 20.0):
    t0 = time.time()
    while time.time() - t0 < t:
        for d in usb.core.find(find_all=True, idVendor=VID, idProduct=PID):
            pn = getattr(d, "port_numbers", None) or ()
            if pn and pn[-1] == port:
                try:
                    try:
                        d.set_configuration()
                    except Exception:
                        pass
                    return d
                except Exception:
                    pass
        time.sleep(0.15)
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    # REQUIRED, deliberately: there is no correct default. The build tree is
    # per-target and per-worktree, so any baked-in path names some OTHER tree.
    # This one used to default to ~/git/u-boot/spl/u-boot-spl-signed.bin.new --
    # a private upstream-master checkout, and a file absent even there.
    ap.add_argument("--image", required=True)
    ap.add_argument("--port", type=int, default=1)
    ap.add_argument("--ppps", action="store_true", default=True)
    ap.add_argument("--no-ppps", action="store_true")
    args = ap.parse_args()
    do_ppps = not args.no_ppps

    img = open(args.image, "rb").read()
    log(f"image {args.image} {len(img)} B @1000={img[0x1000:0x1008].hex()}")

    if do_ppps:
        ppps_cycle(args.port)
        time.sleep(2.5)

    dev = wait_dev(args.port, 20)
    if not dev:
        log("no BROM")
        return 1
    time.sleep(0.4)
    dev = wait_dev(args.port, 5) or dev
    soc = pyamlboot.AmlogicSoC.__new__(pyamlboot.AmlogicSoC)
    soc.dev = dev
    try:
        dev.set_configuration()
    except Exception:
        pass

    pre = bytes(dev.ctrl_transfer(0xC0, 0x20, 0, 0, 8, timeout=1500))
    log("pre " + pre.hex())
    soc.writeMemory(MARKER_ADDR, b"\x00" * 8)
    log("marker cleared " + bytes(soc.readSimpleMemory(MARKER_ADDR, 4)).hex())

    ok = False
    for bl in (512, 64):
        try:
            soc.writeLargeMemory(SPL_LOAD, img, bl, True)
        except Exception as e:
            log(f"bulk{bl} err {e}")
            continue
        en = bytes(soc.readSimpleMemory(SPL_LOAD + 0x1000, 8))
        am = bytes(soc.readSimpleMemory(SPL_LOAD + 0x10, 4))
        log(f"bulk{bl} entry={en.hex()} aml={am}")
        if en == img[0x1000:0x1008] and am == b"@AML":
            ok = True
            log(f"ok bulk{bl}")
            break
    if not ok:
        log("VERIFY FAIL")
        return 2

    soc.writeMemory(SPL_LOAD, TRAMP)
    assert bytes(soc.readSimpleMemory(SPL_LOAD + 0x10, 4)) == b"@AML"
    soc.writeMemory(0xD900C000, b"\x00" * 64)
    log("RUN base+tramp")
    soc.run(SPL_LOAD)

    # Rapid keep-handle: product asm serves ~8s then returns to C/power.
    # Isolation (park forever) greened at +0.01..0.43s; product needs the
    # full serve window so we do not miss 02 22 after CRT0 latency.
    # After RUN, BROM stop serving — dispose stale handle and re-find fast.
    # Worldcup/C path may TX 02 22 while host still holds dead keep-handle.
    try:
        usb.util.dispose_resources(dev)
    except Exception:
        pass
    log("post-RUN rapid refind 30s for 02 22")
    t0 = time.time()
    while time.time() - t0 < 30:
        d = wait_dev(args.port, 0.3)
        if not d:
            if int((time.time() - t0) * 2) % 4 == 0:
                log(f"+{time.time()-t0:.1f} gone")
            time.sleep(0.05)
            continue
        try:
            try:
                d.set_configuration()
            except Exception:
                pass
            raw = d.ctrl_transfer(0xC0, 0x20, 0, 0, 8, timeout=300)
            ret = bytes(raw) if raw is not None else b""
            if len(ret) < 2:
                raise RuntimeError(f"short id len={len(ret)}")
            soc2 = pyamlboot.AmlogicSoC.__new__(pyamlboot.AmlogicSoC)
            soc2.dev = d
            try:
                mk = bytes(soc2.readSimpleMemory(MARKER_ADDR, 4))
                prog = bytes(soc2.readSimpleMemory(PROG_ADDR, 4))
                mk_u32 = struct.unpack("<I", mk)[0]
                prog_u32 = struct.unpack("<I", prog)[0]
                snpsid = struct.unpack(
                    "<I", bytes(soc2.readSimpleMemory(GSNPSID_ADDR, 4)))[0]
                ghwcfg2 = struct.unpack(
                    "<I", bytes(soc2.readSimpleMemory(GHWCFG2_ADDR, 4)))[0]
            except Exception:
                mk = b"\x00\x00\x00\x00"
                prog_u32 = 0
                mk_u32 = 0
                snpsid = 0
                ghwcfg2 = 0
            tag = (
                "SPL"
                if ret[1] == 0x22
                else ("BROM" if ret[:4] == bytes([2, 4, 0, 0]) else "OTHER")
            )
            log(
                f"+{time.time()-t0:.2f} {tag} {ret.hex()} mk={mk.hex()} "
                f"prog={fmt_prog(prog_u32)} bus={d.bus} addr={d.address}"
            )
            if ret[1] == 0x22:
                log(fmt_hwcfg(snpsid, ghwcfg2))
                log("SUCCESS U2 identify 02 22")
                return 0
            if mk_u32 == MARKER_VAL and ret[1] != 0x22:
                # Prefer 02 22; keep looking if only OMOR
                pass
        except Exception as e:
            now = time.time() - t0
            if now < 2.0 or int(now) % 2 == 0:
                log(f"+{now:.2f} refind fail {e}")
        time.sleep(0.05)

    log("FAIL")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
