#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""Product-path smoke: load open SPL → 02 22 → wait post-DDR → bulk FIT stub.

Usage (libre-computer PPPS port 1):
  sudo PYTHONPATH=~/git/pyamlboot python3 tools/spl-fit-smoke.py \\
      --image <build-dir>/spl/u-boot-spl-signed.bin.acm [--port 1]

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
FIT_LOAD = 0x05000000
STAGE_ADDR = 0xD900C120
PROG_ADDR = 0xD900C104
TRAMP = struct.pack("<I", 0x14000400)
FDT_MAGIC = struct.pack(">I", 0xD00DFEED)

STAGE_NAMES = {
    0: "entry",
    1: "after_early_id",
    2: "after_sft_disc",
    3: "after_smc",
    4: "after_power",
    5: "enter_ddr",
    6: "ddr_ok",
    7: "fit_done",
    0x50: "before_wdt",
    0x51: "after_wdt",
    0x53: "before_power",
    0x5C: "smc_skipped",
    0xDF: "ddr_fail",
    0xBD: "bootdev_not_usb",
    0xAA00: "post_id_serve",
    0xBB01: "post_id_done",
}


def log(m: str) -> None:
    print(m, flush=True)


def ppps(port: int = 1, delay: float = 2.5) -> None:
    import subprocess

    script = "/tmp/scripts/ppps-cycle2.py"
    if os.path.isfile(script):
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


def wait_dev(port: int, t: float = 30.0):
    t0 = time.time()
    while time.time() - t0 < t:
        for d in usb.core.find(find_all=True, idVendor=VID, idProduct=PID):
            pn = getattr(d, "port_numbers", None) or ()
            if pn and pn[-1] == port:
                try:
                    d.set_configuration()
                except Exception:
                    pass
                return d
        time.sleep(0.15)
    return None


def identify(d, timeout_ms: int = 800) -> bytes | None:
    try:
        raw = d.ctrl_transfer(0xC0, 0x20, 0, 0, 8, timeout=timeout_ms)
        ret = bytes(raw) if raw is not None else b""
        if len(ret) < 2:
            return None
        return ret
    except Exception:
        return None


def wrap(d):
    soc = pyamlboot.AmlogicSoC.__new__(pyamlboot.AmlogicSoC)
    soc.dev = d
    return soc


def main() -> int:
    ap = argparse.ArgumentParser()
    # REQUIRED, deliberately: there is no correct default. The build tree is
    # per-target and per-worktree, so any baked-in path names some OTHER tree.
    # This one used to default to ~/git/u-boot/spl/u-boot-spl-signed.bin.acm --
    # a private upstream-master checkout, and a file absent even there.
    ap.add_argument("--image", required=True)
    ap.add_argument("--port", type=int, default=1)
    ap.add_argument(
        "--ddr-wait",
        type=float,
        default=25.0,
        help="seconds to wait for post-DDR SPL USB re-enum",
    )
    ap.add_argument(
        "--fit-size",
        type=int,
        default=4096,
        help="dummy FIT stub size (min 4; FDT magic + pattern)",
    )
    args = ap.parse_args()

    img = open(args.image, "rb").read()
    log(f"image {args.image} {len(img)} B")
    ppps(args.port, 2.5)
    time.sleep(2.5)
    dev = wait_dev(args.port, 20)
    if not dev:
        log("no BROM")
        return 1
    soc = wrap(dev)
    pre = identify(dev, 1500)
    log("pre " + (pre.hex() if pre else "fail"))
    ok = False
    for bl in (512, 64):
        try:
            soc.writeLargeMemory(SPL_LOAD, img, bl, True)
        except Exception as e:
            log(f"bulk{bl} {e}")
            continue
        en = bytes(soc.readSimpleMemory(SPL_LOAD + 0x1000, 8))
        am = bytes(soc.readSimpleMemory(SPL_LOAD + 0x10, 4))
        if en == img[0x1000:0x1008] and am == b"@AML":
            ok = True
            log(f"ok bulk{bl}")
            break
    if not ok:
        log("VERIFY FAIL")
        return 2
    soc.writeMemory(SPL_LOAD, TRAMP)
    log("RUN")
    soc.run(SPL_LOAD)
    try:
        usb.util.dispose_resources(dev)
    except Exception:
        pass

    # Vendor order: no early USB serve — dispose BROM handle, wait SMC/power/DDR.
    try:
        usb.util.dispose_resources(dev)
    except Exception:
        pass
    # Short WDT (~8 s/step) reboots to BROM on hang — poll USB soon + often
    min_post = 3.0
    log(f"settle {min_post:.0f}s then poll USB/stage up to {args.ddr_wait}s...")
    time.sleep(min_post)
    t0 = time.time()
    post = None
    last_err = None
    while time.time() - t0 < args.ddr_wait:
        d = wait_dev(args.port, 0.5)
        if not d:
            if int(time.time() - t0) % 3 == 0:
                log(f"+{time.time()-t0:.1f} no 1b8e")
            continue
        ret = identify(d, 600)
        if not ret or len(ret) < 2 or ret[1] != 0x22:
            if ret:
                log(f"+{time.time()-t0:.2f} post id={ret.hex()} (not 22)")
            try:
                usb.util.dispose_resources(d)
            except Exception:
                pass
            time.sleep(0.2)
            continue
        log(f"+{time.time()-t0:.2f} post-DDR id={ret.hex()}")
        # Probe allow_bulk: tiny WR to FIT addr — STALL if still pre-DDR
        n = max(4, min(args.fit_size, 64))
        stub = FDT_MAGIC + bytes((i & 0xFF) for i in range(n - 4))
        soc2 = wrap(d)
        try:
            soc2.writeLargeMemory(FIT_LOAD, stub, 64, True)
            log(f"FIT bulk64 ok ({n} B) — allow_bulk live")
            post = d
            break
        except Exception as e:
            last_err = e
            log(f"+{time.time()-t0:.2f} FIT probe {e} (retry)")
            try:
                usb.util.dispose_resources(d)
            except Exception:
                pass
            time.sleep(0.5)
    if not post:
        log(f"FAIL no post-DDR bulk ({last_err})")
        d = wait_dev(args.port, 15)
        if d:
            ret = identify(d, 800)
            log(f"final id={ret.hex() if ret else 'none'}")
            try:
                soc_b = wrap(d)
                st = bytes(soc_b.readSimpleMemory(STAGE_ADDR, 4))
                pr = bytes(soc_b.readSimpleMemory(PROG_ADDR, 4))
                stage = int.from_bytes(st, "little")
                prog = int.from_bytes(pr, "little")
                lo = stage & 0xFFFF
                name = STAGE_NAMES.get(lo, STAGE_NAMES.get(stage & 0xFFFFFF, "?"))
                log(
                    f"SRAM stage=0x{stage:08x} ({name}) "
                    f"magic_ok={((stage >> 16) == 0x5354)} prog=0x{prog:x}"
                )
                for a, n in (
                    (0xD900C108, "DSTS"),
                    (0xD900C10C, "DCTL"),
                    (0xD900C110, "GINTSTS"),
                    (0xD900C114, "GUSBCFG"),
                    (0xD900C118, "GOTGCTL"),
                ):
                    try:
                        v = int.from_bytes(
                            bytes(soc_b.readSimpleMemory(a, 4)), "little"
                        )
                        log(f"  dump {n}=0x{v:08x}")
                    except Exception as e:
                        log(f"  dump {n} {e}")
            except Exception as e:
                log(f"stage read {e}")
        else:
            log("final: no 1b8e at all (WDT/power dead?)")
        return 4

    # Also try SRAM bulk if DRAM FIT_LOAD failed above (diagnostic)
    # (probe already targeted FIT_LOAD; optional second path unused)

    # Larger stub if requested
    if args.fit_size > 64:
        n = args.fit_size
        stub = FDT_MAGIC + bytes((i & 0xFF) for i in range(n - 4))
        soc2 = wrap(post)
        log(f"write FIT stub {n} B @ 0x{FIT_LOAD:08x}")
        t1 = time.time()
        ok_w = False
        for bl in (512, 64):
            try:
                soc2.writeLargeMemory(FIT_LOAD, stub, bl, True)
                log(f"FIT bulk{bl} ok in {time.time()-t1:.2f}s")
                ok_w = True
                break
            except Exception as e:
                log(f"FIT bulk{bl} {e}")
        if not ok_w:
            log("FAIL FIT large write")
            return 5

    log(f"RUN @ 0x{FIT_LOAD:08x}")
    try:
        wrap(post).run(FIT_LOAD)
        log("SUCCESS product FIT smoke (post-DDR 02 22 + bulk + RUN)")
        return 0
    except Exception as e:
        log(f"RUN {e}")
        log("SUCCESS FIT write (RUN soft)")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
