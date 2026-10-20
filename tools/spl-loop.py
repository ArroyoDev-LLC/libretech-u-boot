#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""
Host-side closed-loop helper for Amlogic GXL SPL USB bring-up.

Designed for rack use on s7-apple with a VIA hub + uhubctl VBUS cycle
(default location 3-2 port 3 → BROM 1b8e:c003).  Does not burn eFuses.

Typical loop (Phase 1 / design U3):
  1. uhubctl VBUS cycle → BROM re-entry
  2. wait for 1b8e:c003
  3. identify (ROM vs SPL fingerprint)
  4. optional: load SPL via ROM protocol + run
  5. optional: wait for SPL re-enum + re-identify
  6. optional: writeLargeMemory throughput (blockLength 64 vs 512)

Requires: pyusb (python3-usb), pyamlboot, uhubctl (for --cycle).

Examples:
  # Cycle hub, wait for BROM, print identify
  sudo ./tools/spl-loop.py --cycle --identify

  # Full pass-1: cycle, load SPL, wait re-enum, identify
  sudo ./tools/spl-loop.py --cycle --spl spl/u-boot-spl.bin --wait-spl

  # BROM writeLargeMemory bench (64 B vs 512 B blockLength)
  sudo ./tools/spl-loop.py --cycle --bench --bench-size 65536

  # After SPL is already up (no cycle): DRAM bench at FIT load address
  ./tools/spl-loop.py --no-cycle --bench --bench-addr 0x05000000 \\
      --bench-size 1048576 --blocks 64,512

  # Combined image extract + pass1-only (like meson-usb-boot --pass1-only)
  sudo ./tools/spl-loop.py --cycle --image u-boot-meson-with-spl.bin --wait-spl

See also:
  tools/meson-usb-boot.py          — two-pass SPL + FIT boot
  amlogic/software/boot/gxl/spl-closed-loop-design.md
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# pyamlboot discovery (repo clone preferred over site-packages)
# ---------------------------------------------------------------------------
_PYAML_CANDIDATES = (
    os.environ.get("PYAMLBOOT_PATH"),
    os.path.expanduser("~/git/pyamlboot"),
    "/home/dxue/git/pyamlboot",
)
for _p in _PYAML_CANDIDATES:
    if _p and os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
        break

try:
    from pyamlboot import pyamlboot
except ImportError as e:
    sys.stderr.write(
        "ERROR: pyamlboot not importable. Install pyusb + clone pyamlboot:\n"
        "  apt install python3-usb\n"
        "  export PYTHONPATH=~/git/pyamlboot  (or pip install pyamlboot)\n"
        f"  ({e})\n"
    )
    sys.exit(2)

try:
    import usb.core
    import usb.util
except ImportError as e:
    sys.stderr.write(f"ERROR: pyusb required (python3-usb): {e}\n")
    sys.exit(2)

# ---------------------------------------------------------------------------
# Constants (match tools/meson-usb-boot.py + meson-usb.c)
# ---------------------------------------------------------------------------
USB_VID = 0x1B8E
USB_PID = 0xC003

SPL_PAD_TO = 0x1FE00
AMLIMAGE_MAGIC = b"\x40\x41\x4d\x4c"  # "@AML"

SPL_LOAD_ADDRESS = 0xD9000000
SPL_ENTRY_ADDRESS = 0xD9001000  # CONFIG_SPL_TEXT_BASE
BL2_PARAMS_ADDRESS = 0xD900C000
FIT_LOAD_ADDRESS = 0x05000000

# High SRAM scratch for BROM writeLargeMemory benches (below top of BL2 SRAM)
ROM_BENCH_ADDRESS = 0xD9008000

# Identify fingerprints (first 4 of 8-byte REQ_IDENTIFY_HOST response)
# ROM (GXL, measured):  02 04 00 00 …  → printed historically as "ROM: 2.4 Stage: 0.0"
# SPL (meson-usb.c):     02 22 00 20 …  → "ROM: 2.34 Stage: 0.32" via meson-usb-boot
IDENTIFY_ROM = bytes((0x02, 0x04, 0x00, 0x00))
IDENTIFY_SPL = bytes((0x02, 0x22, 0x00, 0x20))

DEFAULT_UHUBCTL = os.environ.get("UHUBCTL", "/usr/sbin/uhubctl")
DEFAULT_HUB = "3-2"
DEFAULT_PORT = 3
DEFAULT_CYCLE_DELAY = 2.0  # uhubctl -d seconds off


# ---------------------------------------------------------------------------
# Hub / wait / identify
# ---------------------------------------------------------------------------
def hub_cycle(
    location: str,
    port: int,
    delay: float,
    uhubctl: str,
    use_sudo: bool,
    dry_run: bool = False,
) -> None:
    """Cycle VBUS on hub port via uhubctl (re-enter BROM)."""
    cmd = [uhubctl, "-l", location, "-p", str(port), "-a", "cycle", "-d", str(delay)]
    if use_sudo:
        cmd = ["sudo"] + cmd
    print(f"Hub cycle: {' '.join(cmd)}")
    if dry_run:
        return
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout.strip():
        print(r.stdout.rstrip())
    if r.returncode != 0:
        msg = r.stderr.strip() or r.stdout.strip() or f"exit {r.returncode}"
        raise RuntimeError(f"uhubctl failed: {msg}")



def wait_until_configured(timeout: float = 5.0, poll: float = 0.1) -> usb.core.Device:
    """
    After VID:PID appears, wait until the device has an active configuration
    with a bulk OUT endpoint (HS enum complete). BROM bulk EIO is common if we
    write while the hub still shows only 'power connect' without highspeed.
    """
    deadline = time.time() + timeout
    last_err = None
    while time.time() < deadline:
        dev = usb.core.find(idVendor=USB_VID, idProduct=USB_PID)
        if dev is None:
            time.sleep(poll)
            continue
        try:
            # Touching get_active_configuration forces kernel config if needed
            try:
                cfg = dev.get_active_configuration()
            except usb.core.USBError:
                dev.set_configuration()
                cfg = dev.get_active_configuration()
            intf = cfg[(0, 0)]
            ep = usb.util.find_descriptor(
                intf,
                custom_match=lambda e: usb.util.endpoint_direction(
                    e.bEndpointAddress
                ) == usb.util.ENDPOINT_OUT,
            )
            if ep is not None:
                mps = ep.wMaxPacketSize
                print(f"  configured: bus {dev.bus} dev {dev.address} "
                      f"bulk OUT MPS={mps}")
                return dev
        except (usb.core.USBError, ValueError) as e:
            last_err = e
        time.sleep(poll)
    raise TimeoutError(
        f"device not configured for bulk within {timeout:.0f}s"
        + (f" ({last_err})" if last_err else "")
    )


def wait_for_device(timeout: float, poll: float = 0.25) -> usb.core.Device:
    """Wait until 1b8e:c003 appears; return the pyusb device."""
    print(f"Waiting for Amlogic USB device ({USB_VID:04x}:{USB_PID:04x}) "
          f"timeout={timeout:.0f}s...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        dev = usb.core.find(idVendor=USB_VID, idProduct=USB_PID)
        if dev is not None:
            print(f"  Found: bus {dev.bus} device {dev.address}")
            return dev
        time.sleep(poll)
    raise TimeoutError(
        f"No {USB_VID:04x}:{USB_PID:04x} within {timeout:.0f}s "
        "(check hub cycle / USB boot button / cable)"
    )


def classify_identify(raw: bytes) -> str:
    """Map identify payload to ROM / SPL / unknown label."""
    head = bytes(raw[:4]) if len(raw) >= 4 else bytes(raw)
    if head == IDENTIFY_ROM or (len(head) >= 2 and head[0] == 0x02 and head[1] == 0x04):
        return "ROM"
    if head == IDENTIFY_SPL or (len(head) >= 2 and head[0] == 0x02 and head[1] == 0x22):
        return "SPL"
    return "UNKNOWN"


def format_identify(raw: bytes) -> str:
    """Human + hex dump of identify response."""
    hexstr = " ".join(f"{b:02x}" for b in raw)
    # Match meson-usb-boot.py style for first 4 bytes (as ordinals)
    if len(raw) >= 4:
        legacy = (f"ROM: {raw[0]}.{raw[1]} Stage: {raw[2]}.{raw[3]}")
    else:
        legacy = "(short)"
    kind = classify_identify(raw)
    return f"{kind:7s}  [{hexstr}]  ({legacy})"


def identify_raw(soc: pyamlboot.AmlogicSoC) -> bytes:
    """REQ_IDENTIFY_HOST → raw 8 bytes (pyamlboot.identify returns chr-joined)."""
    s = soc.identify()
    # pyamlboot joins with chr(); recover bytes
    return bytes(ord(c) & 0xFF for c in s)


def connect_soc(timeout: float = 5.0) -> pyamlboot.AmlogicSoC:
    """Open AmlogicSoC (re-finds 1b8e:c003)."""
    return pyamlboot.AmlogicSoC(
        idVendor=USB_VID, idProduct=USB_PID, timeout=timeout
    )


def dispose_soc(soc: pyamlboot.AmlogicSoC | None) -> None:
    if soc is None:
        return
    try:
        if hasattr(soc, "disposeDevice"):
            soc.disposeDevice()
        elif getattr(soc, "dev", None) is not None:
            usb.util.dispose_resources(soc.dev)
    except Exception as e:
        print(f"  warning: dispose: {e}")


# ---------------------------------------------------------------------------
# SPL load (ROM protocol)
# ---------------------------------------------------------------------------
def extract_spl_from_image(image_data: bytes) -> bytes:
    """Extract amlimage SPL from padded binman image (same as meson-usb-boot)."""
    if len(image_data) < 0x18:
        return image_data
    if image_data[0x10:0x14] != AMLIMAGE_MAGIC:
        return image_data
    total_size = struct.unpack_from("<I", image_data, 0x14)[0]
    actual_size = total_size + 0x10
    print(f"  amlimage: {total_size} bytes payload, {actual_size} bytes total")
    return image_data[:actual_size]


def load_spl(
    soc: pyamlboot.AmlogicSoC,
    spl_data: bytes,
    block_length: int = 64,
    entry: int = SPL_ENTRY_ADDRESS,
    load_addr: int = SPL_LOAD_ADDRESS,
    do_run: bool = True,
) -> float:
    """
    writeLargeMemory SPL → dummy BL2 params → optional run.
    Returns wall seconds for the large write only.
    """
    print(
        f"Sending SPL ({len(spl_data)} bytes) → 0x{load_addr:08x} "
        f"blockLength={block_length}..."
    )
    t0 = time.perf_counter()
    soc.writeLargeMemory(load_addr, spl_data, block_length, True)
    dt = time.perf_counter() - t0
    rate = (len(spl_data) / dt / 1024.0) if dt > 0 else 0.0
    print(f"  write: {dt*1000:.1f} ms  ({rate:.0f} KiB/s)")

    # ROM ritual: 64 B at BL2 params (zeros) — no eFuse touch
    soc.writeMemory(BL2_PARAMS_ADDRESS, b"\x00" * 64)
    print(f"  wrote 64 B zeros @ 0x{BL2_PARAMS_ADDRESS:08x}")

    if do_run:
        print(f"  run @ 0x{entry:08x}")
        # GXL BROM ignores RUN at entry; RUN at load base with branch trampoline
        soc.writeMemory(SPL_LOAD_ADDRESS, struct.pack("<I", 0x14000400))
        print(f"  trampoline @0x{SPL_LOAD_ADDRESS:08x} -> 0x{entry:08x}")
        soc.run(SPL_LOAD_ADDRESS)
    return dt


# ---------------------------------------------------------------------------
# Throughput bench
# ---------------------------------------------------------------------------
def measure_write_throughput(
    soc: pyamlboot.AmlogicSoC,
    address: int,
    size: int,
    block_length: int,
    pattern: int = 0xA5,
) -> tuple[float, float]:
    """
    writeLargeMemory of `size` bytes with given blockLength.
    Returns (seconds, bytes_per_sec).
    """
    # Pad to block multiple so appendZeros is not required for timing purity
    pad = (block_length - (size % block_length)) % block_length
    total = size + pad
    data = bytes([pattern & 0xFF]) * total

    t0 = time.perf_counter()
    soc.writeLargeMemory(address, data, block_length, False)
    dt = time.perf_counter() - t0
    bps = (total / dt) if dt > 0 else 0.0
    return dt, bps


def _reopen_primed(
    hub_between: dict | None,
    settle: float,
    timeout: float,
) -> pyamlboot.AmlogicSoC:
    """Optional VBUS cycle, wait, configure, open, identify (prime EP0)."""
    if hub_between:
        hub_cycle(**hub_between)
    wait_for_device(timeout)
    time.sleep(settle)
    wait_until_configured(timeout=max(5.0, settle + 3.0))
    soc = connect_soc(timeout=min(10.0, timeout))
    # BROM bulk OUT often EIO until an EP0 transfer primes the device.
    identify_raw(soc)
    return soc


def run_bench(
    soc: pyamlboot.AmlogicSoC,
    address: int,
    size: int,
    blocks: list[int],
    kind: str,
    *,
    hub_between: dict | None = None,
    settle: float = 2.0,
    timeout: float = 30.0,
    retries: int = 3,
) -> pyamlboot.AmlogicSoC:
    """
    Measure each blockLength with retries.

    On FAIL, dispose + optional VBUS cycle (hub_between) + re-prime and retry.
    Between successful lengths, keep the same handle (no forced cycle) unless
    the next attempt needs recovery.
    Returns the live AmlogicSoC handle (may be a new object).
    """
    print(
        f"\n=== writeLargeMemory bench ({kind}) "
        f"addr=0x{address:08x} size={size} ==="
    )
    print(f"{'block':>8}  {'ms':>10}  {'KiB/s':>10}  {'MiB/s':>10}")
    for bl in blocks:
        last_err = None
        for attempt in range(1, retries + 1):
            try:
                dt, bps = measure_write_throughput(soc, address, size, bl)
                print(
                    f"{bl:8d}  {dt*1000:10.1f}  {bps/1024:10.1f}  "
                    f"{bps/1024/1024:10.3f}"
                    + (f"  (try {attempt})" if attempt > 1 else "")
                )
                last_err = None
                break
            except Exception as e:
                last_err = e
                print(
                    f"{bl:8d}  FAIL try {attempt}/{retries}: "
                    f"{type(e).__name__}: {e}"
                )
                dispose_soc(soc)
                soc = None
                if attempt >= retries:
                    break
                # Recover: cycle if configured, else just re-open/prime
                try:
                    # Prefer cycle on retry when hub_between known; always re-prime
                    soc = _reopen_primed(hub_between, settle, timeout)
                except Exception as re:
                    print(f"  recover failed: {re}")
                    break
        if last_err is not None:
            print(f"{bl:8d}  GIVE UP after {retries} tries")
    return soc


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_blocks(s: str) -> list[int]:
    out = []
    for part in s.split(","):
        part = part.strip()
        if not part:
            continue
        v = int(part, 0)
        if v <= 0:
            raise argparse.ArgumentTypeError(f"bad block length: {part}")
        out.append(v)
    if not out:
        raise argparse.ArgumentTypeError("empty --blocks")
    return out


def parse_size(s: str) -> int:
    """Accept 65536, 64k, 1M, 0x10000."""
    s = s.strip().lower()
    mult = 1
    if s.endswith("kib"):
        mult, s = 1024, s[:-3]
    elif s.endswith("kb") or s.endswith("k"):
        mult, s = 1024, s[:-2] if s.endswith("kb") else s[:-1]
    elif s.endswith("mib"):
        mult, s = 1024 * 1024, s[:-3]
    elif s.endswith("mb") or s.endswith("m"):
        mult, s = 1024 * 1024, (s[:-2] if s.endswith("mb") else s[:-1])
    return int(s, 0) * mult


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="spl-loop.py",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "GXL SPL closed-loop host tool: uhubctl cycle → wait BROM → "
            "identify (ROM vs SPL) → optional SPL load → writeLargeMemory bench."
        ),
        epilog="""
defaults (s7-apple AML-S905X-CC on VIA hub):
  hub location 3-2, port 3, uhubctl /usr/sbin/uhubctl

identify fingerprints (first 4 bytes of REQ_IDENTIFY_HOST):
  ROM  02 04 00 00
  SPL  02 22 00 20   (u-boot arch/arm/mach-meson/meson-usb.c)

bench notes:
  On BROM, large writes should target SRAM (default 0xd9008000).
  After SPL + DDR, use DRAM (e.g. --bench-addr 0x05000000).
  blockLength=64 is the historical pyamlboot default; 512 matches HS bulk MPS.
  This script never touches eFuses.

environment:
  PYAMLBOOT_PATH   directory containing the pyamlboot package parent
  UHUBCTL          path to uhubctl binary
""",
    )

    g_hub = p.add_argument_group("hub / power")
    g_hub.add_argument(
        "-l", "--hub", default=DEFAULT_HUB, metavar="LOC",
        help=f"uhubctl -l location (default: {DEFAULT_HUB})",
    )
    g_hub.add_argument(
        "-p", "--port", type=int, default=DEFAULT_PORT, metavar="N",
        help=f"uhubctl -p port (default: {DEFAULT_PORT})",
    )
    g_hub.add_argument(
        "--cycle", action=argparse.BooleanOptionalAction, default=True,
        help="VBUS cycle before wait (default: on; --no-cycle to skip)",
    )
    g_hub.add_argument(
        "--cycle-delay", type=float, default=DEFAULT_CYCLE_DELAY, metavar="SEC",
        help=f"uhubctl -d off-time seconds (default: {DEFAULT_CYCLE_DELAY})",
    )
    g_hub.add_argument(
        "--uhubctl", default=DEFAULT_UHUBCTL, metavar="PATH",
        help=f"uhubctl binary (default: {DEFAULT_UHUBCTL})",
    )
    g_hub.add_argument(
        "--sudo", action=argparse.BooleanOptionalAction, default=True,
        help="prefix uhubctl with sudo (default: on)",
    )

    g_usb = p.add_argument_group("USB device")
    g_usb.add_argument(
        "--timeout", type=float, default=30.0, metavar="SEC",
        help="device appear timeout (default: 30)",
    )
    g_usb.add_argument(
        "--settle", type=float, default=2.0, metavar="SEC",
        help="sleep after device appears before open (default: 2.0)",
    )
    g_usb.add_argument(
        "--identify", action="store_true", default=True,
        help=argparse.SUPPRESS,  # always identify; kept for clarity in docs
    )
    g_usb.add_argument(
        "--identify-only", action="store_true",
        help="cycle/wait/identify then exit (no load, no bench)",
    )

    g_spl = p.add_argument_group("SPL load (ROM protocol)")
    g_spl.add_argument(
        "--spl", metavar="PATH",
        help="SPL binary (signed amlimage or raw) to load via writeLargeMemory",
    )
    g_spl.add_argument(
        "--image", metavar="PATH",
        help="u-boot-meson-with-spl.bin; extract SPL (pad 0x1fe00) then load",
    )
    g_spl.add_argument(
        "--spl-block", type=int, default=64, metavar="N",
        help="blockLength for SPL inject (default: 64, ROM-friendly)",
    )
    g_spl.add_argument(
        "--no-run", action="store_true",
        help="load SPL but do not REQ_RUN_IN_ADDR",
    )
    g_spl.add_argument(
        "--wait-spl", action="store_true",
        help="after run, dispose + wait for re-enum + re-identify (SPL vs ROM)",
    )
    g_spl.add_argument(
        "--reboot-delay", type=float, default=5.0, metavar="SEC",
        help="sleep after SPL run before re-wait (default: 5; DDR init time)",
    )

    g_bench = p.add_argument_group("writeLargeMemory throughput")
    g_bench.add_argument(
        "--bench", action="store_true",
        help="measure writeLargeMemory throughput for each --blocks value",
    )
    g_bench.add_argument(
        "--bench-size", type=parse_size, default=65536, metavar="N",
        help="transfer size (default: 65536; accepts 64k, 1M, 0x10000)",
    )
    g_bench.add_argument(
        "--bench-addr", type=lambda s: int(s, 0), default=None, metavar="ADDR",
        help="destination address (default: SRAM 0xd9008000 on ROM, "
             f"0x{FIT_LOAD_ADDRESS:08x} on SPL)",
    )
    g_bench.add_argument(
        "--blocks", type=parse_blocks, default=[64, 512], metavar="LIST",
        help="comma-separated blockLength values (default: 64,512)",
    )
    g_bench.add_argument(
        "--bench-after-spl", action="store_true",
        help="run bench after --wait-spl on the re-enumerated device",
    )
    g_bench.add_argument(
        "--bench-cycle", action=argparse.BooleanOptionalAction, default=True,
        help="on bulk FAIL, VBUS-cycle before retry (default: on; "
             "--no-bench-cycle re-opens without power cycle)",
    )
    g_bench.add_argument(
        "--bench-retries", type=int, default=3, metavar="N",
        help="attempts per blockLength on FAIL (default: 3)",
    )

    p.add_argument(
        "--dry-run", action="store_true",
        help="print hub command only; do not touch USB",
    )
    return p


def read_spl_bytes(args: argparse.Namespace) -> bytes | None:
    if args.image:
        with open(args.image, "rb") as f:
            full = f.read()
        chunk = full[:SPL_PAD_TO] if len(full) >= SPL_PAD_TO else full
        data = extract_spl_from_image(chunk)
        print(f"SPL from image {args.image}: {len(data)} bytes")
        return data
    if args.spl:
        with open(args.spl, "rb") as f:
            data = f.read()
        if data[0x10:0x14] == AMLIMAGE_MAGIC:
            data = extract_spl_from_image(data)
        print(f"SPL from {args.spl}: {len(data)} bytes")
        return data
    return None


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.identify_only and (args.spl or args.image or args.bench):
        # allow identify-only to short-circuit load/bench even if also passed
        pass

    spl_data = None
    if not args.identify_only:
        try:
            spl_data = read_spl_bytes(args)
        except OSError as e:
            print(f"ERROR: reading SPL/image: {e}", file=sys.stderr)
            return 1

    # --- 1. Hub cycle ---
    if args.cycle:
        try:
            hub_cycle(
                args.hub, args.port, args.cycle_delay,
                args.uhubctl, args.sudo, dry_run=args.dry_run,
            )
        except RuntimeError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 1
        if args.dry_run:
            print("dry-run: stopping before USB wait")
            return 0
    else:
        print("Skipping hub cycle (--no-cycle)")

    # --- 2. Wait for device ---
    try:
        wait_for_device(args.timeout)
    except TimeoutError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    time.sleep(args.settle)
    try:
        wait_until_configured(timeout=max(5.0, args.settle + 3.0))
    except TimeoutError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    # --- 3. Identify ---
    soc = None
    try:
        soc = connect_soc(timeout=min(10.0, args.timeout))
    except ValueError as e:
        print(f"ERROR: open device: {e}", file=sys.stderr)
        return 1

    try:
        raw = identify_raw(soc)
        print(f"Identify: {format_identify(raw)}")
        kind = classify_identify(raw)

        if args.identify_only:
            return 0

        # --- 4. Optional SPL load ---
        if spl_data is not None:
            if kind == "SPL":
                print(
                    "WARNING: device already fingerprints as SPL; "
                    "loading SPL via ROM protocol may fail"
                )
            load_spl(
                soc, spl_data,
                block_length=args.spl_block,
                do_run=not args.no_run,
            )

            if args.wait_spl and not args.no_run:
                dispose_soc(soc)
                soc = None
                print(
                    f"\nWaiting {args.reboot_delay:.1f}s for SPL USB re-enumeration..."
                )
                time.sleep(args.reboot_delay)
                try:
                    wait_for_device(args.timeout)
                except TimeoutError as e:
                    print(f"ERROR: SPL re-enum: {e}", file=sys.stderr)
                    print(
                        "Possible causes: DDR fail (UART 'DF'), PHY/disconnect, crash"
                    )
                    return 1
                time.sleep(args.settle)
                try:
                    wait_until_configured(timeout=max(5.0, args.settle + 3.0))
                except TimeoutError as e:
                    print(f"ERROR: SPL re-enum config: {e}", file=sys.stderr)
                    return 1
                soc = connect_soc(timeout=min(10.0, args.timeout))
                raw2 = identify_raw(soc)
                print(f"Identify after SPL: {format_identify(raw2)}")
                kind = classify_identify(raw2)
                if kind != "SPL":
                    print(
                        "WARNING: expected SPL fingerprint 02 22 00 20; "
                        f"got {kind} — SPL USB path may not be running"
                    )

        # --- 5. Throughput bench ---
        do_bench = args.bench or args.bench_after_spl
        if args.bench_after_spl and not args.wait_spl:
            print(
                "WARNING: --bench-after-spl without --wait-spl; "
                "benching current device"
            )
        if do_bench:
            if args.bench_addr is not None:
                addr = args.bench_addr
            elif kind == "SPL":
                addr = FIT_LOAD_ADDRESS
            else:
                addr = ROM_BENCH_ADDRESS
            hub_between = None
            if args.bench_cycle:
                hub_between = dict(
                    location=args.hub,
                    port=args.port,
                    delay=args.cycle_delay,
                    uhubctl=args.uhubctl,
                    use_sudo=args.sudo,
                )
            soc = run_bench(
                soc, addr, args.bench_size, args.blocks, kind,
                hub_between=hub_between,
                settle=args.settle,
                timeout=args.timeout,
                retries=max(1, args.bench_retries),
            )

    except usb.core.USBError as e:
        print(f"ERROR: USB: {e}", file=sys.stderr)
        if getattr(e, "errno", None) == 13 or "Access denied" in str(e):
            print(
                "hint: run as root (sudo python3 …) or install a udev rule for "
                "1b8e:c003 (MODE=0666), e.g. from aml-usb-load-uboot README",
                file=sys.stderr,
            )
        return 1
    except Exception as e:
        print(f"ERROR: {type(e).__name__}: {e}", file=sys.stderr)
        return 1
    finally:
        dispose_soc(soc)

    return 0


if __name__ == "__main__":
    sys.exit(main())
