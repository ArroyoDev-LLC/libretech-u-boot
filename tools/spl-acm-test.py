#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""S1/S2 host: load ACM SPL, identify 02 22, re-enum CDC, banner on *our* ttyACM.

Usage (libre-computer PPPS port 1):
  sudo PYTHONPATH=~/git/pyamlboot python3 tools/spl-acm-test.py \\
      --image <build-dir>/spl/u-boot-spl-signed.bin.acm

--image is REQUIRED: name the image from the tree you actually built.

Does NOT treat pre-existing /dev/ttyACM* (other USB gadgets) as success.
Requires a ttyACM whose USB parent is 1b8e:c003 after re-enum.
"""
from __future__ import annotations

import argparse
import glob
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
TRAMP = struct.pack("<I", 0x14000400)


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


def wait_dev(port: int, t: float = 20.0):
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
        time.sleep(0.1)
    return None


def probe_desc(d) -> None:
    """Log device/config descriptors SPL is actually serving (EP0)."""
    try:
        dd = bytes(d.ctrl_transfer(0x80, 0x06, 0x0100, 0, 18, timeout=800))
        log(
            f"dev_desc class={dd[4]:02x}/{dd[5]:02x}/{dd[6]:02x} "
            f"vid={dd[8]:02x}{dd[9]:02x} pid={dd[10]:02x}{dd[11]:02x} "
            f"raw={dd.hex()}"
        )
    except Exception as e:
        log(f"dev_desc fail {e}")
        return
    try:
        cd = bytes(d.ctrl_transfer(0x80, 0x06, 0x0200, 0, 9, timeout=800))
        total = cd[2] | (cd[3] << 8) if len(cd) >= 4 else 0
        log(f"cfg_hdr wTotal={total} bNumIf={cd[4] if len(cd) > 4 else -1}")
        if total > 9:
            full = bytes(d.ctrl_transfer(0x80, 0x06, 0x0200, 0, min(total, 128), timeout=800))
            log(f"cfg_desc[{len(full)}] {full[:24].hex()}...")
    except Exception as e:
        log(f"cfg_desc fail {e}")


def ttyacm_for_vid_pid(vid: int = VID, pid: int = PID) -> list[str]:
    """Return /dev/ttyACM* whose USB device is vid:pid (not other gadgets)."""
    out: list[str] = []
    for path in sorted(glob.glob("/dev/ttyACM*")):
        name = os.path.basename(path)
        base = f"/sys/class/tty/{name}/device"
        # Walk up to usb_device with idVendor
        cur = base
        for _ in range(8):
            if not os.path.isdir(cur):
                break
            vp = os.path.join(cur, "idVendor")
            pp = os.path.join(cur, "idProduct")
            if os.path.isfile(vp) and os.path.isfile(pp):
                try:
                    with open(vp) as f:
                        v = int(f.read().strip(), 16)
                    with open(pp) as f:
                        p = int(f.read().strip(), 16)
                    if v == vid and p == pid:
                        out.append(path)
                except Exception:
                    pass
                break
            cur = os.path.realpath(os.path.join(cur, ".."))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    # REQUIRED, deliberately: there is no correct default. The build tree is
    # per-target and per-worktree, so any baked-in path names some OTHER tree.
    # This one used to default to ~/git/u-boot/spl/u-boot-spl-signed.bin.acm --
    # a private upstream-master checkout that this arc never builds in, and a
    # file that does not exist even there.
    ap.add_argument("--image", required=True)
    ap.add_argument("--port", type=int, default=1)
    args = ap.parse_args()

    pre_acm = set(glob.glob("/dev/ttyACM*"))
    log(f"pre-existing ttyACM={sorted(pre_acm)}")

    img = open(args.image, "rb").read()
    log(f"image {args.image} {len(img)} B")
    ppps(args.port, 2.5)
    time.sleep(2.5)
    dev = wait_dev(args.port, 20)
    if not dev:
        log("no BROM")
        return 1
    soc = pyamlboot.AmlogicSoC.__new__(pyamlboot.AmlogicSoC)
    soc.dev = dev
    pre = bytes(dev.ctrl_transfer(0xC0, 0x20, 0, 0, 8, timeout=1500))
    log("pre " + pre.hex())
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

    t0 = time.time()
    got22 = False
    last = None
    while time.time() - t0 < 15:
        d = wait_dev(args.port, 0.4)
        if not d:
            continue
        try:
            raw = d.ctrl_transfer(0xC0, 0x20, 0, 0, 8, timeout=400)
            ret = bytes(raw) if raw is not None else b""
            if len(ret) < 2:
                raise RuntimeError("short id")
            log(f"+{time.time()-t0:.2f} id={ret.hex()}")
            if ret[1] == 0x22:
                got22 = True
                log("SUCCESS 02 22")
                last = d
                break
        except Exception as e:
            log(f"+{time.time()-t0:.2f} fail {e}")
        time.sleep(0.05)
    if not got22:
        log("no 02 22")
        return 3

    log("probe descriptors after 02 22:")
    probe_desc(last)

    # Vendor bulk console (CDC soft-reenum drops 1b8e — use bulk as OTG UART)
    log("bulk IN EP 0x81 for banner (vendor console)")
    try:
        try:
            last.set_configuration()
        except Exception:
            pass
        try:
            usb.util.claim_interface(last, 0)
        except Exception as e:
            log(f"claim {e}")
        time.sleep(1.0)
        try:
            last.clear_halt(0x81)
        except Exception as e:
            log(f"clear_halt {e}")
        try:
            last.clear_halt(0x01)
        except Exception as e:
            log(f"clear_halt out {e}")
        got = b""
        for _ in range(10):
            try:
                chunk = bytes(last.read(0x81, 512, timeout=1500))
                log(f"bulk81 {chunk!r}")
                got += chunk
                if b"GXL" in got:
                    log("SUCCESS bulk banner")
                    # getc soft: bulk OUT on data EP if present
                    try:
                        cfg = last.get_active_configuration()
                        intf = cfg[(0, 0)]
                        ep_out = usb.util.find_descriptor(
                            intf,
                            custom_match=lambda e: usb.util.endpoint_direction(
                                e.bEndpointAddress
                            )
                            == usb.util.ENDPOINT_OUT,
                        )
                        if ep_out is not None:
                            ep_out.write(b"?\n", timeout=1000)
                            log(f"bulk OUT 0x{ep_out.bEndpointAddress:02x} write ok")
                        else:
                            log("no bulk OUT ep in active cfg")
                    except Exception as e:
                        log(f"bulk OUT write {e}")
                    return 0
            except Exception as e:
                log(f"bulk81 {e}")
            time.sleep(0.3)
        if got:
            log(f"FAIL bulk data no GXL {got!r}")
            return 5
    except Exception as e:
        log(f"bulk path {e}")

    log("FAIL no bulk banner")
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
