#!/usr/bin/env python3
"""PPPS the parent hub port that owns an Amlogic device path.

Usage:
  ppps-path.py                 # power-cycle every 1b8e:c003 parent port
  ppps-path.py 2               # only devices whose last hop is 2
  ppps-path.py --path 2,3,2    # exact path
  ppps-path.py 2 --delay 2.5   # VBUS-off dwell

Resolution is cached.  The hub that owns a board is discovered from the LIVE
device's port path, so the moment the board is legitimately absent -- which is
now the normal state after SPL teardown (meson_usb_device_stop) and is also
true of a wedged board -- there is nothing to resolve from.  The old fallback
guessed "top USB2 VIA, ports 3,2,1", cycled the wrong hub, and the board never
came back: measured 3 of 5 lifecycle cycles lost to "no BROM after PPPS"
(~/local/build/gxl-otg-stop2.log).

So every successful resolution is persisted and reused when the device is gone.
The cache stores the hub's TOPOLOGY (bus + port_numbers + vid:pid), never its
USB address -- an address is reassigned on every re-enumeration, so caching it
would aim at whatever device inherited the number.
"""
from __future__ import annotations

import json
import os
import sys
import time

import usb.core

PORT_POWER = 8
RT = 0x23
CLEAR, SET = 1, 3
VID, PID = 0x1B8E, 0xC003
VIA_USB2, VIA_USB3 = 0x2817, 0x0817

def _cache_path() -> str:
    """Cache lives in the INVOKING user's tree, not root's.

    PPPS needs raw hub control transfers, so this runs under sudo -- where a
    bare ~ expands to /root and the cache silently lands somewhere the next
    (non-sudo) reader cannot see.  Honour SUDO_USER.
    """
    env = os.environ.get("PPPS_PATH_CACHE")
    if env:
        return os.path.expanduser(env)
    sudo_user = os.environ.get("SUDO_USER")
    home = os.path.expanduser("~" + sudo_user) if sudo_user else os.path.expanduser("~")
    return os.path.join(home, "local/build/ppps-path-cache.json")


CACHE = _cache_path()


def log(m: str) -> None:
    print(m, flush=True)


def cache_load() -> dict:
    try:
        with open(CACHE) as f:
            return json.load(f)
    except Exception:
        return {}


def cache_store(key: str, hub, port: int, dev_path: tuple) -> None:
    """Persist hub topology (not address) so an absent device is recoverable."""
    data = cache_load()
    data[key] = {
        "hub_bus": hub.bus,
        "hub_ports": list(getattr(hub, "port_numbers", None) or ()),
        "hub_vid": hub.idVendor,
        "hub_pid": hub.idProduct,
        "port": port,
        "dev_path": list(dev_path),
    }
    try:
        os.makedirs(os.path.dirname(CACHE), exist_ok=True)
        tmp = CACHE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=1, sort_keys=True)
        os.replace(tmp, CACHE)
    except Exception as e:
        log(f"  cache write failed (non-fatal): {e}")


def hub_from_cache(entry: dict):
    """Re-resolve the live hub device from cached topology."""
    want_ports = tuple(entry.get("hub_ports") or ())
    for d in usb.core.find(find_all=True, idVendor=entry["hub_vid"],
                           idProduct=entry["hub_pid"]):
        if d.bus != entry["hub_bus"]:
            continue
        if tuple(getattr(d, "port_numbers", None) or ()) == want_ports:
            return d
    return None


def find_hub_for_path(path: tuple[int, ...], prefer_bus: int | None = None):
    """Return (hub_dev, port_on_hub) such that hub.port leads to path.

    Prefer USB2 VIA (0x2817) on the same bus as the device — USB3 companion
    (0x0817) PORT_POWER often does not cut USB2 VBUS, so product 02 22 sticks.
    """
    if not path:
        return None, None
    parent_path = path[:-1]
    port = path[-1]
    candidates = []
    for d in usb.core.find(find_all=True):
        try:
            if d.bDeviceClass == 9 or (
                d.idVendor == 0x2109 and d.idProduct in (VIA_USB2, VIA_USB3)
            ):
                pn = tuple(getattr(d, "port_numbers", None) or ())
                candidates.append((pn, d))
        except Exception:
            continue

    def score(d) -> int:
        s = 0
        if prefer_bus is not None and d.bus == prefer_bus:
            s += 100
        # USB2 VIA preferred over USB3 companion
        if d.idVendor == 0x2109 and d.idProduct == VIA_USB2:
            s += 50
        elif d.idVendor == 0x2109 and d.idProduct == VIA_USB3:
            s += 10
        return s

    # exact parent path match, highest score
    matches = [(score(d), d) for pn, d in candidates if pn == parent_path]
    if matches:
        matches.sort(key=lambda x: -x[0])
        return matches[0][1], port
    # fallback: top USB2 VIA, first hop of path
    for pn, d in sorted(candidates, key=lambda x: -score(x[1])):
        if d.idVendor == 0x2109 and d.idProduct == VIA_USB2 and len(pn) <= 1:
            return d, path[0] if path else port
    return None, None


def cycle(hub, port: int, delay: float = 2.5) -> None:
    log(
        f"cycle hub bus={hub.bus} addr={hub.address} "
        f"ports={getattr(hub, 'port_numbers', None)} port={port}"
    )
    try:
        hub.ctrl_transfer(RT, CLEAR, PORT_POWER, port, None, timeout=1000)
        log("  PORT_POWER cleared")
    except Exception as e:
        log(f"  CLEAR fail: {e}")
        return
    time.sleep(delay)
    # companion USB3 same vendor
    for d in usb.core.find(find_all=True, idVendor=0x2109, idProduct=VIA_USB3):
        pn = tuple(getattr(d, "port_numbers", None) or ())
        hpn = tuple(getattr(hub, "port_numbers", None) or ())
        if pn == hpn or (not hpn and len(pn) <= 1):
            try:
                d.ctrl_transfer(RT, CLEAR, PORT_POWER, port, None, timeout=1000)
                log(f"  companion off bus={d.bus}")
            except Exception as e:
                log(f"  companion off fail: {e}")
    time.sleep(0.4)
    try:
        hub.ctrl_transfer(RT, SET, PORT_POWER, port, None, timeout=1000)
        log("  PORT_POWER set")
    except Exception as e:
        log(f"  SET fail: {e}")
        return
    for d in usb.core.find(find_all=True, idVendor=0x2109, idProduct=VIA_USB3):
        pn = tuple(getattr(d, "port_numbers", None) or ())
        hpn = tuple(getattr(hub, "port_numbers", None) or ())
        if pn == hpn or (not hpn and len(pn) <= 1):
            try:
                d.ctrl_transfer(RT, SET, PORT_POWER, port, None, timeout=1000)
                log(f"  companion on bus={d.bus}")
            except Exception as e:
                log(f"  companion on fail: {e}")
    time.sleep(2.5)


def report(expect_path=None) -> int:
    aml = list(usb.core.find(find_all=True, idVendor=VID, idProduct=PID))
    log(f"after: amlogic count={len(aml)}")
    hit = False
    for d in aml:
        pn = tuple(getattr(d, "port_numbers", None) or ())
        log(f"  path={pn}")
        if expect_path and pn == tuple(expect_path):
            hit = True
        try:
            d.set_configuration()
            r = bytes(d.ctrl_transfer(0xC0, 0x20, 0, 0, 8, timeout=600))
            log(f"  id={r.hex()}")
        except Exception as e:
            log(f"  id fail {e}")
    if expect_path:
        return 0 if hit else 2
    return 0 if aml else 2


def main() -> int:
    last = None
    exact = None
    delay = 2.5
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--path" and i + 1 < len(args):
            exact = tuple(int(x) for x in args[i + 1].split(","))
            i += 2
        elif args[i] == "--delay" and i + 1 < len(args):
            delay = float(args[i + 1])
            i += 2
        else:
            last = int(args[i])
            i += 1

    key = f"path={','.join(str(x) for x in exact)}" if exact else f"last={last}"

    targets = []  # (path, bus)
    for d in usb.core.find(find_all=True, idVendor=VID, idProduct=PID):
        pn = tuple(getattr(d, "port_numbers", None) or ())
        if exact and pn != exact:
            continue
        if last is not None and (not pn or pn[-1] != last):
            continue
        targets.append((pn, d.bus))
        log(f"target amlogic path={pn} bus={d.bus}")

    if not targets:
        # Device absent — the normal state after a clean SPL teardown, and the
        # case the old blind "ports 3,2,1" sweep got wrong.  Use what worked.
        entry = cache_load().get(key)
        if entry:
            hub = hub_from_cache(entry)
            if hub is not None:
                log(f"no live device; cached hub bus={entry['hub_bus']} "
                    f"ports={tuple(entry['hub_ports'])} port={entry['port']}")
                cycle(hub, entry["port"], delay)
                return report(entry.get("dev_path"))
            log(f"no live device; cached hub {entry['hub_vid']:04x}:"
                f"{entry['hub_pid']:04x} bus={entry['hub_bus']} not present")
        else:
            log(f"no live device and no cache entry for {key}")
        log("falling back to top USB2 VIA ports 3,2,1 (may cycle the wrong hub)")
        for d in usb.core.find(find_all=True, idVendor=0x2109, idProduct=VIA_USB2):
            pn = tuple(getattr(d, "port_numbers", None) or ())
            if len(pn) <= 1:
                for port in (3, 2, 1):
                    cycle(d, port, delay)
                break
        return report(entry.get("dev_path") if entry else None)

    done = set()
    for path, bus in targets:
        hub, port = find_hub_for_path(path, prefer_bus=bus)
        if hub is None:
            log(f"no hub for path {path}")
            continue
        k = (hub.bus, hub.address, port)
        if k in done:
            continue
        done.add(k)
        log(f"using hub vid={hub.idVendor:04x}:{hub.idProduct:04x} bus={hub.bus}")
        cache_store(key, hub, port, path)
        cycle(hub, port, delay)

    return report(targets[0][0] if targets else None)


if __name__ == "__main__":
    raise SystemExit(main())
