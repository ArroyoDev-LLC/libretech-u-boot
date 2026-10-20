#!/usr/bin/env python3
"""Post-DDR SPL bulk bench: load product SPL → 02 22 → timed WR_LARGE + optional CRC.

Usage (libre-computer PPPS):
  sudo PYTHONPATH=~/git/pyamlboot python3 tools/spl-bulk-bench.py \\
      [--size 1M] [--block 512] [--port 1] [--verify]
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import time
import zlib

import usb.core
import usb.util

for p in (
    os.environ.get("PYAMLBOOT_PATH"),
    os.path.expanduser("~/git/pyamlboot"),
    "/home/dxue/git/pyamlboot",
):
    if p and os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
        break

from pyamlboot import pyamlboot

VID, PID = 0x1B8E, 0xC003
SPL_LOAD = 0xD9000000
FIT_LOAD = 0x05000000
STAGE_ADDR = 0xD900C120
PROG_ADDR = 0xD900C104
BULK_DMA_REQ_ADDR = 0xD900C194
CLK81_REQ_ADDR = 0xD900C19C
FIFO_REQ_ADDR = 0xD900C1A4
CLK81_BEFORE_ADDR = 0xD900C168
CLK81_AFTER_ADDR = 0xD900C16C
# HHI_MPEG_CLK_CNTL mux table (Linux mux_table_clk81) on GXL fixed_pll 2550 MHz
CLK81_SRC = {6: ("fclk_div3", 850.0), 5: ("fclk_div4", 637.5), 7: ("fclk_div5", 510.0)}
TRAMP = struct.pack("<I", 0x14000400)


def log(m: str) -> None:
    print(m, flush=True)


def ppps(port: int = 1, delay: float = 2.5) -> None:
    import subprocess

    # Path-aware PPPS first: on this rig the USB3 companion's PORT_POWER does
    # not cut USB2 VBUS, so the USB2 VIA 2109:2817 parent hub must be the one
    # cycled (design doc, 2026-08-06).  ppps-cycle2.py is the older fallback.
    # Repo copy first — /tmp is tmpfs and the hub-path cache the script keeps
    # is what makes a cycle work once the board has correctly disconnected.
    path_script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "ppps-path.py")
    if not os.path.isfile(path_script):
        path_script = "/tmp/scripts/ppps-path.py"
    if os.path.isfile(path_script):
        subprocess.run(
            ["sudo", "-n", "env", "PYTHONUNBUFFERED=1", "python3",
             path_script, str(port), "--delay", str(delay)],
            check=False,
        )
        return
    script = "/tmp/scripts/ppps-cycle2.py"
    if os.path.isfile(script):
        subprocess.run(
            ["sudo", "-n", "env", "PYTHONUNBUFFERED=1", "python3", script, str(port), str(delay)],
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
        time.sleep(0.1)
    return None


def identify(d, timeout_ms: int = 800):
    try:
        raw = d.ctrl_transfer(0xC0, 0x20, 0, 0, 8, timeout=timeout_ms)
        return bytes(raw) if raw is not None else None
    except Exception:
        return None


def wrap(d):
    soc = pyamlboot.AmlogicSoC.__new__(pyamlboot.AmlogicSoC)
    soc.dev = d
    return soc


def _find_bulk_out(dev):
    """Prefer BROM bulk OUT EP 0x02; fall back to any bulk OUT."""
    cfg = dev.get_active_configuration()
    intf = cfg[(0, 0)]

    def is_out(e):
        return (
            usb.util.endpoint_direction(e.bEndpointAddress)
            == usb.util.ENDPOINT_OUT
        )

    ep2 = usb.util.find_descriptor(
        intf,
        custom_match=lambda e: is_out(e) and (e.bEndpointAddress & 0x7F) == 2,
    )
    if ep2 is not None:
        return ep2
    ep = usb.util.find_descriptor(intf, custom_match=is_out)
    if ep is None:
        raise RuntimeError("no bulk OUT ep")
    return ep


def write_large_fast(
    dev, address: int, data: bytes, block: int, urb: int, chunk: int = 4096
) -> None:
    """WR_LARGE with large multi-packet bulk URBs (fewer pyusb round-trips).

    Protocol: wValue=block (MPS-ish), wIndex=blockCount, header has byte len.
    Host then streams data in ``urb``-sized bulk OUT writes (USB stack splits
    to 512 B HS packets). Classic pyamlboot does one ep.write(block) per
    packet — that caps ~6 MiB/s from userspace.

    Device BROM RXFIFO is ~1 KiB; a single WR_LARGE >4 KiB desyncs PIO and
    fails full-pattern CRC. Split into ``chunk``-byte WR_LARGEs (default 4 KiB).
    """
    if len(data) % block:
        raise ValueError("data length must be multiple of block")
    if urb < block:
        urb = block
    if urb % block:
        urb = ((urb + block - 1) // block) * block
    if chunk < block:
        chunk = block
    if chunk % block:
        chunk = ((chunk + block - 1) // block) * block

    ep = _find_bulk_out(dev)
    prefer = os.environ.get("BULK_OUT_EP")
    if prefer is not None:
        want = int(prefer, 0)
        cfg = dev.get_active_configuration()
        intf = cfg[(0, 0)]
        for e in intf:
            if (e.bEndpointAddress & 0x7F) == want and usb.util.endpoint_direction(
                e.bEndpointAddress
            ) == usb.util.ENDPOINT_OUT:
                ep = e
                break
    log(
        f"bulk OUT ep=0x{ep.bEndpointAddress:02x} mps={ep.wMaxPacketSize} "
        f"chunk={chunk}"
    )
    offset = 0
    for offset in range(0, len(data), chunk):
        n = min(chunk, len(data) - offset)
        if n % block:
            raise ValueError("chunk path requires block-aligned lengths")
        block_count = n // block
        control = struct.pack("<IIII", address + offset, n, 0, 0)
        dev.ctrl_transfer(0x40, 0x11, block, block_count, control, timeout=5000)
        end = offset + n
        pos = offset
        for pos in range(offset, end, urb):
            m = min(urb, end - pos)
            ep.write(data[pos : pos + m], timeout=30000)


def write_large_pipeline(
    dev, address: int, data: bytes, block: int, urb: int, depth: int
) -> None:
    """WR_LARGE + libusb-1.0 async multi-URB bulk OUT (overlap in-flight URBs).

    Uses the open pyusb libusb1 handle/ctx. Falls back to write_large_fast
    if depth < 2 or handle/ctx cannot be resolved.
    """
    import ctypes

    if depth < 2:
        write_large_fast(dev, address, data, block, urb)
        return
    if len(data) % block:
        raise ValueError("data length must be multiple of block")
    if urb < block:
        urb = block
    if urb % block:
        urb = ((urb + block - 1) // block) * block

    try:
        handle = dev._ctx.handle.handle  # c_void_p libusb_device_handle*
        libusb_ctx = dev.backend.ctx  # c_void_p libusb_context*
        lib = dev.backend.lib  # CDLL
    except Exception as e:
        log(f"pipeline: no libusb handle ({e}); sequential URB")
        write_large_fast(dev, address, data, block, urb)
        return

    # x86_64 libusb_transfer (non-ISO)
    class Xfer(ctypes.Structure):
        _fields_ = [
            ("dev_handle", ctypes.c_void_p),
            ("flags", ctypes.c_uint8),
            ("endpoint", ctypes.c_uint8),
            ("type", ctypes.c_uint8),
            ("_pad0", ctypes.c_uint8),
            ("timeout", ctypes.c_uint),
            ("status", ctypes.c_int),
            ("length", ctypes.c_int),
            ("actual_length", ctypes.c_int),
            ("_pad1", ctypes.c_int),
            ("callback", ctypes.c_void_p),
            ("user_data", ctypes.c_void_p),
            ("buffer", ctypes.c_void_p),
            ("num_iso_packets", ctypes.c_int),
        ]

    class Timeval(ctypes.Structure):
        _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

    LIBUSB_TRANSFER_COMPLETED = 0
    LIBUSB_TRANSFER_TYPE_BULK = 2
    LIBUSB_SUCCESS = 0

    lib.libusb_alloc_transfer.restype = ctypes.c_void_p
    lib.libusb_alloc_transfer.argtypes = [ctypes.c_int]
    lib.libusb_free_transfer.argtypes = [ctypes.c_void_p]
    lib.libusb_submit_transfer.argtypes = [ctypes.c_void_p]
    lib.libusb_submit_transfer.restype = ctypes.c_int
    lib.libusb_cancel_transfer.argtypes = [ctypes.c_void_p]
    lib.libusb_handle_events_timeout_completed.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(Timeval),
        ctypes.POINTER(ctypes.c_int),
    ]
    lib.libusb_handle_events_timeout_completed.restype = ctypes.c_int
    # libusb_fill_bulk_transfer is a static inline — fill Xfer by hand

    ep = _find_bulk_out(dev)
    ep_addr = int(ep.bEndpointAddress)
    block_count = len(data) // block
    control = struct.pack("<IIII", address, len(data), 0, 0)
    dev.ctrl_transfer(0x40, 0x11, block, block_count, control, timeout=5000)

    state = {
        "off": 0,
        "inflight": 0,
        "err": None,
        "done": 0,
        "bufs": [],
        "xfers": [],
    }

    CB = ctypes.CFUNCTYPE(None, ctypes.POINTER(Xfer))

    def on_complete(xfer_p):
        x = xfer_p.contents
        if x.status != LIBUSB_TRANSFER_COMPLETED:
            if state["err"] is None:
                state["err"] = f"xfer status={x.status}"
            state["inflight"] -= 1
            return
        state["done"] += x.actual_length
        state["inflight"] -= 1
        if state["err"] is not None or state["off"] >= len(data):
            return
        n = min(urb, len(data) - state["off"])
        buf = (ctypes.c_ubyte * n).from_buffer_copy(
            data[state["off"] : state["off"] + n]
        )
        state["bufs"].append(buf)
        state["off"] += n
        x.buffer = ctypes.cast(buf, ctypes.c_void_p)
        x.length = n
        x.actual_length = 0
        rc = lib.libusb_submit_transfer(ctypes.cast(xfer_p, ctypes.c_void_p))
        if rc != LIBUSB_SUCCESS:
            state["err"] = f"resubmit rc={rc}"
        else:
            state["inflight"] += 1

    c_cb = CB(on_complete)
    state["cb"] = c_cb  # keep alive

    for _ in range(depth):
        if state["off"] >= len(data):
            break
        n = min(urb, len(data) - state["off"])
        buf = (ctypes.c_ubyte * n).from_buffer_copy(
            data[state["off"] : state["off"] + n]
        )
        state["bufs"].append(buf)
        state["off"] += n
        xfer = lib.libusb_alloc_transfer(0)
        if not xfer:
            raise RuntimeError("libusb_alloc_transfer failed")
        state["xfers"].append(xfer)
        # Fill struct (libusb_fill_bulk_transfer is header-only inline)
        xp = ctypes.cast(xfer, ctypes.POINTER(Xfer)).contents
        xp.dev_handle = handle
        xp.flags = 0
        xp.endpoint = ep_addr & 0xFF
        xp.type = LIBUSB_TRANSFER_TYPE_BULK
        xp.timeout = 30000
        xp.status = -1
        xp.length = n
        xp.actual_length = 0
        xp.callback = ctypes.cast(c_cb, ctypes.c_void_p)
        xp.user_data = None
        xp.buffer = ctypes.cast(buf, ctypes.c_void_p)
        xp.num_iso_packets = 0
        rc = lib.libusb_submit_transfer(xfer)
        if rc != LIBUSB_SUCCESS:
            raise RuntimeError(f"libusb_submit_transfer rc={rc}")
        state["inflight"] += 1

    tv = Timeval(0, 200000)  # 200 ms
    deadline = time.time() + 120.0
    while state["inflight"] > 0 and state["err"] is None:
        if time.time() > deadline:
            state["err"] = "pipeline timeout"
            break
        lib.libusb_handle_events_timeout_completed(
            libusb_ctx, ctypes.byref(tv), None
        )

    if state["err"]:
        for x in state["xfers"]:
            lib.libusb_cancel_transfer(x)
        # drain cancellations
        t_end = time.time() + 2.0
        while state["inflight"] > 0 and time.time() < t_end:
            lib.libusb_handle_events_timeout_completed(
                libusb_ctx, ctypes.byref(tv), None
            )

    for x in state["xfers"]:
        lib.libusb_free_transfer(x)

    if state["err"]:
        raise RuntimeError(state["err"])
    if state["done"] != len(data):
        raise RuntimeError(f"short write {state['done']}/{len(data)}")


def parse_size(s: str) -> int:
    s = s.strip().upper()
    if s.endswith("M"):
        return int(float(s[:-1]) * 1024 * 1024)
    if s.endswith("K"):
        return int(float(s[:-1]) * 1024)
    return int(s)


def main() -> int:
    ap = argparse.ArgumentParser()
    # REQUIRED, deliberately: there is no correct default. The build tree is
    # per-target and per-worktree, so any baked-in path names some OTHER tree.
    # This one used to default to ~/git/u-boot/spl/u-boot-spl-signed.bin -- a
    # private upstream-master checkout this arc never builds in. That file DOES
    # exist there, which made this the worst of the four: it would have booted a
    # weeks-old image from an unrelated branch and reported the numbers as ours.
    ap.add_argument("--image", required=True)
    ap.add_argument("--port", type=int, default=1)
    ap.add_argument("--size", default="1M", help="payload size e.g. 256K, 1M, 4M")
    ap.add_argument(
        "--block",
        type=int,
        default=512,
        help="protocol blockLength (wValue); also default URB size for classic path",
    )
    ap.add_argument(
        "--urb",
        type=int,
        default=1048576,
        help="host URB chunk bytes (0=classic per-block; default 1 MiB multi-pkt)",
    )
    ap.add_argument(
        "--pipeline",
        type=int,
        default=1,
        help="async multi-URB depth (1=sequential; 2+ = libusb pipeline)",
    )
    ap.add_argument(
        "--chunk",
        type=int,
        default=4096,
        help="device WR_LARGE size bytes (0=one shot; default 4096 for CRC)",
    )
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=FIT_LOAD)
    ap.add_argument("--verify", action="store_true", help="readback CRC via RD_LARGE")
    ap.add_argument(
        "--bulk-dma",
        type=int,
        default=0,
        metavar="N",
        help="buffer-DMA bulk OUT variant: 1=INCR4, 2=INCR16, 3=INCR16+RX512 "
        "(0 = asm PIO spin)",
    )
    ap.add_argument(
        "--rxfifo",
        type=int,
        default=None,
        metavar="N",
        help="RX FIFO depth rung: 1=384, 2=448, 3=512 words (BROM leaves 256; "
        "the part's whole DFIFO is 712)",
    )
    ap.add_argument(
        "--clk81",
        type=int,
        default=None,
        metavar="N",
        help="clk81 (DWC2 AHB clock) rung: 0=stash only, 1=212.5, 2=283.3, "
        "3=318.75, 4=425 MHz",
    )
    ap.add_argument("--ddr-wait", type=float, default=20.0)
    args = ap.parse_args()

    nbytes = parse_size(args.size)
    if nbytes < 64 or nbytes % args.block:
        # pad to block
        nbytes = ((nbytes + args.block - 1) // args.block) * args.block

    img = open(args.image, "rb").read()
    log(
        f"image {args.image} {len(img)} B; bench {nbytes} B block={args.block} "
        f"urb={args.urb or args.block} @ 0x{args.addr:08x}"
    )
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
    if args.bulk_dma:
        # SPL reads this in meson_usb_receive_fit() to pick buffer-DMA bulk
        # OUT over the asm PIO spin.  One image, host-selected path.
        req = b"DMA" + bytes(str(args.bulk_dma), "ascii")
        soc.writeMemory(BULK_DMA_REQ_ADDR, req)
        log(f"bulk-dma variant {args.bulk_dma} @ 0x{BULK_DMA_REQ_ADDR:08x}")
    if args.rxfifo is not None:
        # RX FIFO depth in words: 1=384, 2=448, 3=512 (DFIFO budget is 712)
        soc.writeMemory(FIFO_REQ_ADDR, b"FIF" + bytes(str(args.rxfifo), "ascii"))
        log(f"rxfifo rung {args.rxfifo} @ 0x{FIFO_REQ_ADDR:08x}")
    if args.clk81 is not None:
        # 0 = stash HHI_MPEG_CLK_CNTL without touching it; 1-4 = rate rungs
        soc.writeMemory(CLK81_REQ_ADDR, b"CLK" + bytes(str(args.clk81), "ascii"))
        log(f"clk81 rung {args.clk81} @ 0x{CLK81_REQ_ADDR:08x}")
    soc.writeMemory(SPL_LOAD, TRAMP)
    log("RUN")
    soc.run(SPL_LOAD)
    try:
        usb.util.dispose_resources(dev)
    except Exception:
        pass

    # Product power+DDR ~2s then worldcup 02 22 (see fit-smoke settle).
    # Ignore early 02 22 ghosts in the first min_post seconds after RUN.
    time.sleep(0.5)
    t0 = time.time()
    post = None
    min_post = 1.8
    while time.time() - t0 < args.ddr_wait:
        elapsed = time.time() - t0
        d = wait_dev(args.port, 0.4)
        if not d:
            continue
        ret = identify(d, 600)
        if not ret or len(ret) < 2 or ret[1] != 0x22:
            try:
                usb.util.dispose_resources(d)
            except Exception:
                pass
            continue
        if elapsed < min_post:
            log(f"+{elapsed:.2f} early 02 22 (wait >={min_post:.1f}s for DDR)")
            try:
                usb.util.dispose_resources(d)
            except Exception:
                pass
            time.sleep(0.2)
            continue
        log(f"+{elapsed:.2f} post-DDR id={ret.hex()}")
        try:
            st = bytes(wrap(d).readSimpleMemory(STAGE_ADDR, 4))
            stage = int.from_bytes(st, "little")
            log(
                f"  stage=0x{stage:08x} magic_ok={((stage >> 16) == 0x5354)}"
            )
        except Exception as e:
            log(f"  stage soft-fail {e}")
        try:
            wrap(d).writeLargeMemory(
                args.addr, b"\xd0\x0d\xfe\xed" + bytes(60), 64, True
            )
            post = d
            break
        except Exception as e:
            log(f"probe {e}")
            try:
                usb.util.dispose_resources(d)
            except Exception:
                pass
    if not post:
        log("FAIL no post-DDR bulk")
        return 4

    # Build patterned payload
    pattern = bytearray(nbytes)
    for i in range(nbytes):
        pattern[i] = (i * 131 + 17) & 0xFF
    # FDT-looking header so readers don't treat as garbage
    pattern[0:4] = b"\xd0\x0d\xfe\xed"
    crc_exp = zlib.crc32(bytes(pattern)) & 0xFFFFFFFF

    soc2 = wrap(post)
    payload = bytes(pattern)
    if args.pipeline > 1 and args.urb:
        mode = f"pipeline={args.pipeline} urb={args.urb}"
    elif args.urb:
        mode = f"urb={args.urb} chunk={args.chunk}"
    else:
        mode = f"classic block={args.block}"
    log(f"bench write {nbytes} B {mode}...")
    t1 = time.time()
    try:
        if args.pipeline > 1 and args.urb:
            write_large_pipeline(
                post, args.addr, payload, args.block, args.urb, args.pipeline
            )
        elif args.urb:
            write_large_fast(post, args.addr, payload, args.block, args.urb, args.chunk or len(payload))
        else:
            soc2.writeLargeMemory(args.addr, payload, args.block, False)
    except Exception as e:
        log(f"FAIL write {e}")
        return 5
    dt = time.time() - t1
    mib_s = (nbytes / (1024 * 1024)) / dt if dt > 0 else 0
    # Report vs 20 MiB/s product target (design P4)
    pct_tgt = 100.0 * mib_s / 20.0
    log(
        f"WRITE ok {nbytes} B in {dt:.3f}s = {mib_s:.2f} MiB/s "
        f"({pct_tgt:.0f}% of 20 MiB/s target)"
    )
    log(f"CRC32 expect 0x{crc_exp:08x}")

    if args.verify:
        log("verify readback via REQ_READ_MEM (64 B chunks)...")
        t2 = time.time()
        try:
            chunks = []
            left = nbytes
            off = 0
            while left > 0:
                n = 64 if left >= 64 else left
                got = bytes(soc2.readSimpleMemory(args.addr + off, 64))
                chunks.append(got[:n])
                off += n
                left -= n
            rb = b"".join(chunks)
        except Exception as e:
            log(f"FAIL read {e}")
            return 6
        dt2 = time.time() - t2
        crc_got = zlib.crc32(rb) & 0xFFFFFFFF
        mib_r = (nbytes / (1024 * 1024)) / dt2 if dt2 > 0 else 0
        match = crc_got == crc_exp and rb == bytes(pattern)
        log(
            f"READ  {nbytes} B in {dt2:.3f}s = {mib_r:.2f} MiB/s "
            f"CRC 0x{crc_got:08x} match={match}"
        )
        if not match:
            # first mismatch
            for i, (a, b) in enumerate(zip(bytes(pattern), rb)):
                if a != b:
                    log(f"first mismatch @ +{i} exp={a:02x} got={b:02x}")
                    break
            return 7
        log("SUCCESS bulk bench write+CRC verify")
    else:
        log("SUCCESS bulk bench write (no verify)")

    # stage if readable
    try:
        st = int.from_bytes(bytes(soc2.readSimpleMemory(STAGE_ADDR, 4)), "little")
        log(f"stage=0x{st:08x}")
    except Exception as e:
        log(f"stage {e}")
    # clk81 before/after, stashed by the SPL (BROM READ_MEM cannot reach HHI)
    for label, addr in (("clk81_before", CLK81_BEFORE_ADDR),
                        ("clk81_after", CLK81_AFTER_ADDR)):
        try:
            v = int.from_bytes(bytes(soc2.readSimpleMemory(addr, 4)), "little")
            sel, div = (v >> 12) & 0x7, (v & 0x7F) + 1
            name, rate = CLK81_SRC.get(sel, ("sel%d?" % sel, None))
            mhz = f"{rate / div:.2f} MHz" if rate else "?"
            log(f"{label}=0x{v:08x} sel={sel}({name}) div={div} -> {mhz}")
        except Exception as e:
            log(f"{label} {e}")
    # U1P progress word: bit16 = DMA armed, bit17 = returned to PIO
    try:
        pg = int.from_bytes(bytes(soc2.readSimpleMemory(PROG_ADDR, 4)), "little")
        log(f"prog=0x{pg:08x} dma_on={bool(pg & 0x10000)} dma_off={bool(pg & 0x20000)}")
    except Exception as e:
        log(f"prog {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
