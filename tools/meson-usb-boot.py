#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""
USB boot tool for Amlogic Meson GX boards with open-source SPL.

Requires pyamlboot: pip install pyamlboot

Two-pass boot flow:
  Pass 1: Send SPL to SRAM via ROM protocol.  SPL does DDR init, then
           re-enumerates as a USB device (VID=1b8e PID=c003) using the
           Amlogic USB protocol.
  Pass 2: SPL (acting as USB device) receives the FIT image via
           writeLargeMemory to CONFIG_SPL_LOAD_FIT_ADDRESS, then boots
           when REQ_RUN_IN_ADDR is sent.  SPL does NOT need to be
           re-sent on pass 2.

TPL->SPL->FIT chain (CONFIG_MESON_GX_XPL_CHAIN): the SRAM stage is TPL and the
DRAM stage is a separate raw binary the host writes into DRAM in pass 2, before
the FIT, until TPL learns to fetch it from the boot medium itself:

    meson-usb-boot.py --spl tpl/u-boot-tpl-signed.bin \
                      --dram-spl spl/u-boot-spl.bin --fit u-boot.itb

Usage:
    meson-usb-boot.py u-boot-meson-with-spl.bin
    meson-usb-boot.py --spl spl/u-boot-spl-signed.bin --fit u-boot.itb
"""

import argparse
import os
import struct
import sys
import time

from pyamlboot import pyamlboot

# Where the FIT starts inside a combined binman image (--image).  Like the FIT
# staging address below, it is read from the build (CONFIG_MESON_GX_FIT_OFFSET)
# and never from a literal: it was a constant 0x1fe00 here until the GXL
# DRAM-SPL window was widened (FIT at 0x4c000), and a stale copy would cut the
# image at the wrong byte.
FIT_OFFSET_SYMBOL = "CONFIG_MESON_GX_FIT_OFFSET"

# amlimage magic at offset 0x10
AMLIMAGE_MAGIC = b'\x40\x41\x4d\x4c'  # "@AML"

# DRAM address the FIT is staged at.  This MUST equal the SPL's
# CONFIG_SPL_LOAD_FIT_ADDRESS: the SPL does not read the address off the wire,
# it reads its own compiled-in constant.  So there is no default here and no
# literal to drift -- resolve_fit_address() reads it out of the u-boot config
# that built the image being sent.
#
# It used to be a hardcoded 0x05000000 with the comment "must match
# CONFIG_SPL_LOAD_FIT_ADDRESS", which is a second copy of a value nothing kept
# in step.  When 1663e862b56 moved the V2 open chain to 0x0c000000 the tool
# kept staging at 0x05000000 and the board sat at "Trying to boot from RAM"
# looking at an empty address -- a silent failure that reads as a boot hang.
FIT_LOAD_ADDRESS = None

# Where resolve_fit_address() looks, in order.  Both are real workflows:
# LBS writes out/<target> beside out/<target>.config, and an in-tree u-boot
# build has .config at the tree root.
FIT_ADDR_SYMBOL = "CONFIG_SPL_LOAD_FIT_ADDRESS"

# DRAM address of the SPL stage in the TPL->SPL->FIT chain.  Must match
# CONFIG_MESON_GX_SPL_ENTRY / CONFIG_SPL_TEXT_BASE.  Used only with --dram-spl:
# in the chain the SRAM stage is TPL, and until TPL learns to fetch SPL itself
# (T3) the host writes it into DRAM in the same session, before the FIT.  The
# SRAM stage's receive loop runs until REQ_RUN_IN_ADDR, so any number of
# writeLargeMemory transfers may precede it.
DRAM_SPL_ADDRESS = 0x08000000

# SRAM address where ROM loads BL2/SPL (amlimage header goes here)
SPL_LOAD_ADDRESS = 0xd9000000

# SPL entry point: code starts at offset 0x1000 within the amlimage
# (amlimage header = 0x1000 bytes, SPL code at load_addr + 0x1000).
# The ROM 'run' command branches directly -- it does NOT parse the amlimage
# header to find the entry.  Running at SPL_LOAD_ADDRESS (0xd9000000) would
# execute the header zeros causing an undefined-instruction fault.
# SPL_ENTRY_ADDRESS must match CONFIG_SPL_TEXT_BASE in the U-Boot config.
SPL_ENTRY_ADDRESS = 0xd9001000

# BL2 run parameters address (ROM protocol requirement)
BL2_PARAMS_ADDRESS = 0xd900c000

# Amlogic ROM USB IDs
USB_VID = 0x1b8e
USB_PID = 0xc003

# FDT magic for FIT detection
FDT_MAGIC = 0xd00dfeed

# Full USB port path (tuple of hub port numbers) identifying WHICH board to
# boot, set by --port.  None keeps the historical behaviour: take whatever
# usb.core.find returns first.
#
# That default is only safe with exactly one board attached.  A rig carrying
# two GX-CHIPs -- both 1b8e:c003, indistinguishable by VID/PID -- resolves to
# whichever the backend happens to enumerate first, so "which board did I just
# boot" becomes a function of enumeration order.  The port path is the only
# stable identifier here: the USB address changes on every re-enumeration, and
# the BootROM exposes no serial.
PORT_PATH = None


def _config_int(path, symbol):
    """Return the integer value of symbol in a u-boot config, or None."""
    try:
        with open(path) as f:
            for line in f:
                if line.startswith(symbol + "="):
                    return int(line.split("=", 1)[1].strip(), 0)
    except OSError:
        return None
    return None


def resolve_fit_address(args):
    """Derive the FIT staging address from the build, never from a literal.

    Order: an explicit --fit-addr, then the u-boot config that produced the
    image being sent.  There is deliberately NO fallback constant: the SPL
    reads its own compiled-in CONFIG_SPL_LOAD_FIT_ADDRESS, so a guess here
    stages the payload where nothing looks for it and the board simply stops
    at "Trying to boot from RAM".
    """
    return _resolve_from_build(args, args.fit_addr, "--fit-addr",
                               FIT_ADDR_SYMBOL)


def resolve_fit_offset(args):
    """Derive where the FIT starts in a combined --image, from the build."""
    return _resolve_from_build(args, args.fit_offset, "--fit-offset",
                               FIT_OFFSET_SYMBOL)


def _resolve_from_build(args, override, flag, symbol):
    """Return (value, source) for symbol: the flag if given, else the config.

    Exits when neither is available: every caller describes a value the
    built image hard-codes, so a guess here is a silently wrong boot.
    """
    import os

    if override is not None:
        return int(override, 0), flag

    candidates = []
    ref = args.image or args.fit or args.spl
    if ref:
        ref = os.path.abspath(ref)
        # LBS layout: out/<target> alongside out/<target>.config
        candidates.append(ref + ".config")
        # u-boot build dir: <builddir>/.config beside spl/, u-boot.itb, ...
        d = os.path.dirname(ref)
        candidates.append(os.path.join(d, ".config"))
        candidates.append(os.path.join(d, os.pardir, ".config"))
    # In-tree build: this script lives in <tree>/tools/
    candidates.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   os.pardir, ".config"))

    tried = []
    for c in candidates:
        c = os.path.normpath(c)
        if c in tried:
            continue
        tried.append(c)
        value = _config_int(c, symbol)
        if value is not None:
            return value, c

    print(f"ERROR: could not determine {symbol}.")
    print("  The built image hard-codes it; this tool must not guess it.")
    print("  Looked in:")
    for c in tried:
        print(f"    {c}")
    print(f"  Pass {flag} 0x... , or run beside the build that made the image.")
    sys.exit(1)


def _port_path_of(dev):
    return tuple(getattr(dev, "port_numbers", None) or ())


def _find_device(quiet=False):
    """Return the ROM USB device, honouring PORT_PATH when set.

    With PORT_PATH unset this REFUSES to choose when more than one
    1b8e:c003 is attached, rather than taking the first match.  Two GX-CHIP
    boards are indistinguishable by VID/PID, so first-match makes "which
    board am I about to write SPL into and RUN code on" a function of
    enumeration order -- and the wrong answer is a GXL image executed on an
    SM1 board.  A single attached board keeps working with no --port.
    """
    import usb.core
    found = list(usb.core.find(find_all=True, idVendor=USB_VID,
                               idProduct=USB_PID))
    if PORT_PATH is not None:
        for dev in found:
            if _port_path_of(dev) == PORT_PATH:
                return dev
        return None
    if len(found) > 1:
        if not quiet:
            print(f"ERROR: {len(found)} devices match "
                  f"{USB_VID:04x}:{USB_PID:04x} and they are "
                  f"indistinguishable by VID/PID. Pass --port to say which "
                  f"board to boot:")
            for dev in found:
                path = ",".join(str(p) for p in _port_path_of(dev))
                print(f"  --port {path}    (bus {dev.bus} device {dev.address})")
        sys.exit(1)
    return found[0] if found else None


# The four identify bytes OUR SRAM stage returns, from
# arch/arm/mach-meson/meson-usb.c:
#     static const u8 identify_rsp[8] = { 0x02, 0x22, 0x00, 0x20, 0, 0, 0, 0 };
# printed by connect() below as "ROM: 2.34 Stage: 0.32".  The BootROM's own
# answer is 0x02,0x04,0x00,0x00 -- "ROM: 2.4 Stage: 0.0".
#
# WHY THIS MATTERS ENOUGH TO GATE ON.  The ROM and the stage enumerate with the
# SAME 1b8e:c003 VID/PID at the same port path, so nothing else in the flow can
# tell them apart.  Measured 2026-08-12: in a failing chain boot pass 2 read
# "ROM: 2.4 Stage: 0.0", wrote the DRAM stage to 0x08000000 through a responder
# that has not initialised DRAM (at 1835 KB/s, against 272-283 KB/s on the
# eleven good runs of the same session), and only then timed out on the 1.2 MB
# FIT.  This tool ALREADY PRINTED the discriminator and proceeded regardless;
# every failure previously attributed to a bulk-transfer race would have been
# caught by comparing it.
STAGE_IDENTIFY = (0x02, 0x22, 0x00, 0x20)
ROM_IDENTIFY = (0x02, 0x04, 0x00, 0x00)

# The (bus, address) pass 1 ran the SRAM stage on, set by send_spl(), and
# whether pass 2 has since seen the board RESET back into the BootROM.
#
# Our stage either keeps pass 1's device or re-enumerates as itself
# (STAGE_IDENTIFY).  A device at the port with a NEW address that answers
# ROM_IDENTIFY is therefore a SoC that reset -- the TPL's DRAM-init failure
# stop (DF) or NOSPL ends in its watchdog -- and the stage will never come.
# Probing it further is not harmless: wait_for_stage()'s 0.1 s re-probe and
# connect_stage()'s retries sent the fresh BootROM a stream of identify
# requests, after which the NEXT session stalled on its first write in 5
# of 6 re-stages; with nothing polling across the reset it re-staged 6 of
# 6 (GXL DDR3 FMA round F3, claude amlogic/software/boot/gxl/audit/
# 2026-10-19-gxl-ddr3-boot-chain-failure-modes.md).  So pass 2 stops at
# the first such answer and says why.
PASS1_DEV = None
BOARD_RESET_SEEN = False


def _is_reset_rom(raw, ident):
    """True when `raw` answered as the BootROM from a new device address."""
    global BOARD_RESET_SEEN
    if (ident == ROM_IDENTIFY and PASS1_DEV is not None
            and (raw.bus, raw.address) != PASS1_DEV):
        if not BOARD_RESET_SEEN:
            print(f"  BOARD RESET: the BootROM is back at bus {raw.bus} "
                  f"device {raw.address} (pass 1 was device {PASS1_DEV[1]}); "
                  "our stage will not answer -- not probing it further. "
                  "Check the UART for 'DF' (DRAM init failed) or 'NOSPL'.")
        BOARD_RESET_SEEN = True
        return True
    return False


def _ident_str(ident):
    return " ".join(f"{b:02x}" for b in ident)


def connect():
    """Connect to Amlogic ROM USB device via pyamlboot."""
    raw = _find_device()
    if raw is None:
        where = "" if PORT_PATH is None else f" at USB port path {PORT_PATH}"
        print(f"ERROR: no {USB_VID:04x}:{USB_PID:04x} device{where}")
        sys.exit(1)
    # Same idiom tools/spl-bulk-bench.py uses to bind pyamlboot to an
    # already-selected device: AmlogicSoC.__init__ would re-find it, which
    # would re-introduce the first-match choice _find_device() just refused.
    dev = pyamlboot.AmlogicSoC.__new__(pyamlboot.AmlogicSoC)
    dev.dev = raw
    print(f"  Port path {_port_path_of(raw)}: "
          f"bus {raw.bus} device {raw.address}")
    s = dev.identify()
    # Keep the historical human line byte-for-byte -- every log and every doc
    # in this project greps for it -- and additionally keep the raw tuple so a
    # caller can GATE on it instead of reading it and shrugging.
    dev.identify_bytes = tuple(ord(c) for c in s[:4])
    print(f"  ROM: {ord(s[0])}.{ord(s[1])} Stage: {ord(s[2])}.{ord(s[3])}")
    return dev


def _configured(dev):
    """True once the host kernel has set a configuration on `dev` (sysfs).

    Where there is nothing to read -- no Linux sysfs (macOS, Windows), or a
    libusb that reports no port numbers -- answer True: the check is an
    ordering refinement, and without it the stager behaves as it did before
    the check existed, instead of waiting out its whole timeout.
    """
    root = "/sys/bus/usb/devices"
    ports = getattr(dev, "port_numbers", None)
    if not ports or not os.path.isdir(root):
        return True
    name = f"{dev.bus}-" + ".".join(str(p) for p in ports)
    try:
        with open(f"{root}/{name}/bConfigurationValue") as f:
            return f.read().strip() not in ("", "0")
    except OSError:
        return False


def wait_for_stage(timeout, probe_ms=5000, gap=0.1):
    """Wait until OUR stage answers identify on the pinned port, not a fixed time.

    Returns the seconds it took, or None when `timeout` expired first.

    Each probe is one identify on whatever 1b8e:c003 sits at PORT_PATH:

    - the BootROM, before it has jumped to the run address, answers
      ROM_IDENTIFY at once -- probe again after `gap`;
    - while the TPL trains DRAM nobody serves EP0, but the SETUP is not lost:
      the core ACKs it into the RX FIFO, and the TPL's session take-over
      deliberately leaves it there and answers it (meson-usb.c, "Worldcup").
      So the probe simply blocks until the TPL is ready.  A probe that times
      out first is harmless: the TPL arms its answer to the stale SETUP, drops
      the next SETUP while EP0 is not idle, and the next probe's IN stage
      collects that (identical) answer -- measured with --stage-probe-ms 100,
      4 of 4 boots, read-back bit-identical.  `probe_ms` only sets how often
      a slow DRAM training is re-probed;
    - when the TPL re-enumerates, the device goes away mid-probe (USBError)
      and comes back as a new device -- probe that one.

    Only identify is sent, which the BootROM supports; an unsupported request
    is what deranges its EP0 (see connect_stage()).

    A re-enumerated device is probed only once the host kernel has CONFIGURED
    it (sysfs bConfigurationValue set), so the kernel's SET_CONFIGURATION --
    which re-arms the stage's bulk endpoints -- cannot land inside pass 2.
    The check is passive: it reads sysfs and sends nothing on the bus.
    """
    import usb.core
    import usb.util

    t0 = time.time()
    deadline = t0 + timeout
    probes = 0
    while time.time() < deadline:
        raw = _find_device(quiet=True)
        if raw is None or not _configured(raw):
            time.sleep(gap)
            continue
        probes += 1
        try:
            ident = tuple(raw.ctrl_transfer(0xc0, pyamlboot.REQ_IDENTIFY_HOST,
                                            0, 0, 8, timeout=probe_ms)[:4])
        except usb.core.USBError:
            ident = None
        usb.util.dispose_resources(raw)
        if _is_reset_rom(raw, ident):
            return None
        if ident == STAGE_IDENTIFY:
            dt = time.time() - t0
            print(f"  stage answered after {dt:.2f}s ({probes} probe(s))")
            return dt
        time.sleep(gap)
    print(f"  no stage answer within {timeout}s ({probes} probe(s))")
    return None


def connect_stage(timeout, retries, delay, settle=0.5):
    """Pass-2 connect that REFUSES to write to anything but our own stage.

    Returns a connected device whose identify is STAGE_IDENTIFY, or None.
    `settle` is the pause after the device is found; wait_for_stage() having
    already had our stage answer, the caller passes 0.

    The retry is not cosmetic: the failure mode is a take-over that has not
    happened *yet* -- TPL printed "USB" on its console and there was no second
    "GXL:BL1:" banner, so the SoC never reset -- so re-scanning and
    re-identifying is exactly the recovery this needs.  What it must never do
    is what the old code did: push 1.2 MB into an address the responder cannot
    have backed with DRAM, and then blame the bulk path for the timeout.
    """
    import usb.util

    seen = None
    last_err = None
    for attempt in range(1, retries + 1):
        if BOARD_RESET_SEEN:
            return None
        if not wait_for_device(timeout):
            return None
        if settle:
            time.sleep(settle)
        try:
            dev = connect()
        except Exception as e:
            # A responder that cannot answer identify() AT ALL is the same
            # decision as one that answers with the wrong bytes: do NOT write
            # to it, and let the take-over have the next retry.  Measured on
            # GXL 2026-09-17 (amlogic N43): a single unsupported control
            # request deranges the BootROM's EP0 -- the trigger returns Errno
            # 75 Overflow, then identify()/GET_DESCRIPTOR fail Errno 75 or
            # time out (Errno 110) for >=30 s.  Before this guard connect()
            # raised straight out of the retry loop, so the gate never ran.
            last_err = f"{type(e).__name__}: {e}"
            print(f"  pass-2 GATE: identify FAILED ({last_err}) -- "
                  f"attempt {attempt}/{retries}, not writing")
            if attempt < retries:
                time.sleep(delay)
            continue
        if dev.identify_bytes == STAGE_IDENTIFY:
            if attempt > 1:
                print(f"  pass-2 gate: our stage answered on attempt {attempt}")
            return dev

        seen = dev.identify_bytes
        if _is_reset_rom(dev.dev, seen):
            usb.util.dispose_resources(dev.dev)
            return None
        who = "the BootROM" if seen == ROM_IDENTIFY else "an unknown responder"
        print(f"  pass-2 GATE: {who} is answering, not our stage "
              f"(identify {_ident_str(seen)}, expected "
              f"{_ident_str(STAGE_IDENTIFY)}) -- "
              f"attempt {attempt}/{retries}, not writing")
        usb.util.dispose_resources(dev.dev)
        del dev
        if attempt < retries:
            time.sleep(delay)

    print(f"ERROR: pass 2 never reached our stage.  Last identify: "
          f"{_ident_str(seen) if seen else '(none)'}")
    if seen is None and last_err:
        print(f"  The last identify attempt FAILED at the USB layer "
              f"({last_err}) -- the responder's EP0 did not answer.")
    if seen == ROM_IDENTIFY:
        print("  That is the BootROM.  The SRAM stage started its USB session "
              "(check the console for 'USB') but the host re-attached to the "
              "ROM instead -- writing here would target un-initialised DRAM.")
    return None


def extract_spl_from_image(image_data):
    """Extract the actual SPL (amlimage) from a padded binman image.

    The binman output pads SPL to SPL_PAD_TO with 0xff.  The amlimage
    header at offset 0x10 contains the actual payload size.
    """
    if len(image_data) < 0x18:
        return image_data

    # Check for amlimage magic at offset 0x10
    if image_data[0x10:0x14] != AMLIMAGE_MAGIC:
        return image_data

    # Total image size at offset 0x14 (little-endian u32)
    total_size = struct.unpack_from('<I', image_data, 0x14)[0]

    # Add the 16-byte preamble before @AML magic
    actual_size = total_size + 0x10
    print(f"  amlimage: {total_size} bytes payload, {actual_size} bytes total")
    return image_data[:actual_size]


def wait_for_device(timeout=30):
    """Wait for Amlogic ROM USB device to appear."""
    where = "" if PORT_PATH is None else f" at port path {PORT_PATH}"
    print(f"Waiting for Amlogic USB device (1b8e:c003){where}...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        dev = _find_device()
        if dev is not None:
            print(f"  Found: bus {dev.bus} device {dev.address}")
            return True
        time.sleep(0.05)
    return False


def _witness_offsets(spl_data):
    """Pick file offsets whose bytes DIFFER between builds of the same tree.

    The entry bytes at 0x1000 are the arm64 image header -- `B #0x28`, `NOP`,
    text_offset -- which is byte-identical in every SPL this tree produces.
    Verifying only those cannot distinguish "my image landed" from "the image
    already in SRAM is a different build of the same tree", because both
    answer with the same 16 bytes.  So verify a second, build-varying window
    too.

    The image is padded, so the final window is useless: measured 2026-08-24
    on out/aml-s905x-cc-libre.usb.bl2, len 49152 with the last non-zero byte
    at 0x7399, the last aligned window is 64 zero bytes -- constant across
    every build, and therefore exactly as useless as the header it was meant
    to supplement.

    Return TWO windows, because they catch different failures:

    - the last window of REAL CONTENT -- late enough that a short or partial
      write cannot reach it (0x7398 on the image above);
    - the most byte-diverse window in the image -- the tail window there holds
      only 3 distinct byte values, so on its own it would match between two
      builds differing only in the middle at equal length.
    """
    floor = 0x1000 + 0x10
    off = (len(spl_data) - 64) & ~0x3
    tail = floor
    while off > floor:
        if len(set(spl_data[off:off + 64])) > 1:
            tail = off
            break
        off -= 4

    best, best_n = floor, -1
    for cand in range(floor, max(floor, tail) + 1, 64):
        n = len(set(spl_data[cand:cand + 64]))
        if n > best_n:
            best, best_n = cand, n

    return sorted({tail, best})


def brom_warmup(dev):
    """Spend a throwaway memory write so the REAL first transfer is the second.

    ERRATUM (GXL BootROM, measured 2026-08-31 on s7-nyc and ai-swift-16): the
    ROM discards the FIRST memory-write command after every USB enumeration,
    at ANY blockLength.  A standalone probe wrote a self-describing 48 KiB
    pattern to SRAM and read it back through the 64-byte CONTROL path (so the
    path under test is not also the measuring instrument), one fresh
    enumeration per trial:

        no warm-up, blockLength=512 .......... 0/3
        no warm-up, blockLength=64  .......... 0/3   <-- 64 is NOT "reliable"
        SET_CONFIGURATION first, 512 ......... 0/3
        throwaway 64B writeLargeMemory, 512 .. 3/3
        throwaway 64B writeLargeMemory, 64 ... 3/3
        control-path REQ_WRITE_MEM first, 512  3/3

    Reading SRAM *before* the write settles what "corruption" was: the failing
    write is a complete no-op, byte-identical to the pre-write contents at
    every probed offset.  Never truncation, never scrambling, never a
    block-size boundary.  The three per-port "corruption modes" (all-zero,
    all-ff, "7f1..." garbage) are those ports' residual SRAM read through a
    write that never happened.

    So the 512-then-64 retry below never demonstrated anything about block
    size: 512 was always the sacrificial write and 64 always the second one.

    Any vendor WRITE clears the state; a control IN (identify) and
    SET_CONFIGURATION do not (0/3).  The state is on the DEVICE, not the host
    -- a different process writing to a warm device passes on its first write.

    This is a mask-ROM erratum with a cheap host-side workaround, so nothing
    is being fixed at the wrong layer: the BootROM is the layer and it is not
    ours to change.  Our own SRAM stage does NOT have this defect (its first
    write lands 2/2 with no warm-up), which is why this is scoped to the
    pass-1 BootROM and not applied in pass 2.

    Full measurement: amlogic/software/boot/gxl/audit/
    2026-08-31-gxl-brom-swallows-the-first-write-after-enumeration.md
    """
    print("  BROM warm-up: throwaway 64-byte write (the ROM discards the "
          "first write after every enumeration)")
    try:
        dev.writeLargeMemory(SPL_LOAD_ADDRESS, b"\x00" * 64, 64, True)
    except Exception as e:
        # The swallowed first write sometimes manifests as a control-transfer
        # timeout rather than a silent no-op (measured on s7-nyc physical 6
        # and on ai-swift-16 3-5.2).  That is a second spelling of the same
        # first-write failure, not a different defect -- the state is cleared
        # either way, which is the entire purpose of this call.  Swallowing
        # the error here is therefore correct, not a papered-over failure.
        print(f"  BROM warm-up write raised {type(e).__name__}: {e}")
        print("  (expected on some ports -- the ROM state is cleared anyway)")


# The ROM's own USB (dwc2) registers, read through REQ_READ_MEM: core
# 0x00..0x18, 0x24..0x5c (skipping GRXSTSP at 0x20, which pops the RX FIFO),
# device 0x800..0x81c, IN endpoints 0x900..0x97c, OUT endpoints 0xb00..0xb7c.
BROM_PRE_READS = ((0xc9100000, 0x1c), (0xc9100024, 0x3c), (0xc9100800, 0x20),
                  (0xc9100900, 0x80), (0xc9100b00, 0x80))


def brom_pre_reads(dev):
    """Read-only ROM requests before the warm-up write (--brom-pre-reads).

    DIAGNOSTIC, not a workaround (GXL, measured 2026-10-18 on s7-nyc slot1):
    after a Linux `reboot -f` the BootROM sometimes stalls on the real SPL
    write.  These reads before the warm-up make the base DTB stage 10 of 10
    (1 of 8 without) but the padded DTB 0 of 10 (9 of 9 without): they swap
    which layout stalls rather than clearing the state.  Measurement:
    amlogic/software/boot/audit/2026-10-17-gxl-optee-xtest-and-warm-reset.md
    """
    print("  BROM pre-reads: dwc2 registers through the ROM before the "
          "first write (--brom-pre-reads)")
    for addr, length in BROM_PRE_READS:
        off = 0
        while off < length:
            n = min(64, length - off)
            dev.readSimpleMemory(addr + off, n)
            off += n


def _write_spl_verified(dev, spl_data):
    """Bulk-write SPL; verify entry AND a build-varying witness.

    The 512-then-64 loop is retained as a belt-and-braces path.  It is NOT a
    block-size fix: see brom_warmup() for why 64 only ever "worked" because
    it was the second attempt.  With the warm-up in place, block=512 lands on
    the first try and this fallback should never fire -- if it does, the
    cause is something other than the first-write erratum, and that is worth
    knowing rather than papering over.

    The all-zero test alone does NOT cover that case.  Measured 2026-08-24 on
    s7-nyc: seven consecutive bulk-512 writes "verified OK" and the board ran
    an image ~7 commits older every time, because SRAM still held a previous
    build of THIS tree whose header matched byte-for-byte.  The failure only
    became visible when a vendor BL2 -- a different header -- was sent and the
    same bulk-512 path reported "verify FAIL" and fell back to 64.  A verify
    that can only catch zeros is a verify that reports success for the exact
    stale-image case the fallback exists to handle.
    """
    expected = spl_data[0x1000:0x1010]
    witnesses = _witness_offsets(spl_data)
    for block in (512, 64):
        print(f"  writeLargeMemory block={block}...")
        t0 = time.time()
        dev.writeLargeMemory(SPL_LOAD_ADDRESS, spl_data, block, True)
        dt = time.time() - t0
        # readSimpleMemory max 64B
        got = bytes(dev.readSimpleMemory(SPL_ENTRY_ADDRESS, 16))
        print(f"  {len(spl_data)/dt/1024:.0f} KB/s  entry={got.hex()}")
        if got != expected or got == b"\x00" * 16:
            print(f"  verify FAIL (expected {expected.hex()})")
            continue
        bad = None
        for woff in witnesses:
            waddr = SPL_LOAD_ADDRESS + woff
            if bytes(dev.readSimpleMemory(waddr, 64)) != spl_data[woff:woff + 64]:
                bad = waddr
                break
        if bad is None:
            where = ", ".join(f"0x{SPL_LOAD_ADDRESS + w:08x}" for w in witnesses)
            print(f"  verified OK (block={block}, witnesses {where})")
            return block
        print(f"  verify FAIL: entry matched but witness @0x{bad:08x} did "
              f"not -- SRAM holds a DIFFERENT build, this write did not land")
    raise RuntimeError("SPL bulk write did not land in SRAM (512 and 64)")


def send_spl(dev, spl_data):
    """Send SPL to SRAM and execute."""
    global PASS1_DEV
    PASS1_DEV = (dev.dev.bus, dev.dev.address)
    print(f"Sending SPL ({len(spl_data)} bytes) to 0x{SPL_LOAD_ADDRESS:08x}...")
    _write_spl_verified(dev, spl_data)

    # Write dummy BL2 params -- ROM may require a write at this address.
    dev.writeMemory(BL2_PARAMS_ADDRESS, b'\x00' * 64)

    # GXL BROM (measured s7-apple + libre-computer 2026-08-04): REQ_RUN_IN_ADDR
    # at SPL_ENTRY (0xd9001000) is ignored. RUN at load base executes. Plant
    # AArch64 B trampoline over amlimage preamble zeros, then run at base.
    #   B +0x1000: imm26 = 0x400 -> 0x14000400
    tramp = struct.pack('<I', 0x14000400)
    dev.writeMemory(SPL_LOAD_ADDRESS, tramp)
    print(f"Running at 0x{SPL_LOAD_ADDRESS:08x} (trampoline -> 0x{SPL_ENTRY_ADDRESS:08x})...")
    dev.run(SPL_LOAD_ADDRESS)


def send_fit(dev, fit_data):
    """Write FIT image to DRAM."""
    print(f"Sending FIT ({len(fit_data)} bytes) to 0x{FIT_LOAD_ADDRESS:08x}...")
    t0 = time.time()
    dev.writeLargeMemory(FIT_LOAD_ADDRESS, fit_data, 512, True)
    dt = time.time() - t0
    print(f"  {len(fit_data)/dt/1024:.0f} KB/s")


def _dram_witness_offsets(spl_data):
    """64-byte-aligned offsets spanning the DRAM SPL, for the readback.

    Deliberately spread over the whole image so a SHORT write (some prefix
    landed) is distinguishable from a write that landed NOTHING: offset 0 and
    the last window of real content bracket the transfer, the interior ones
    say where a partial stopped.
    """
    n = len(spl_data)
    last = max(0, (n - 64) & ~0x3f)
    cands = [0, last // 4, last // 2, (3 * last) // 4, last]
    return sorted({c & ~0x3f for c in cands if c + 64 <= n})


def _write_large_counted(dev, address, data, block, append_zeros):
    """pyamlboot's _writeLargeMemory, but SUMMING what ep.write() accepted.

    pyamlboot throws the per-packet return value away, so a bulk that
    "completes" reports only wall time -- which is how a 5334 KB/s DRAM-SPL
    completion (an order of magnitude above its 384/388 KB/s siblings, N145
    signature (b)) could be recorded without anyone able to say whether the
    bytes reached the device.

    Returns (claimed, accepted, err) -- err is None on success, otherwise the
    string naming which packet failed and how.  It does NOT raise: a caller
    that wants to read the destination back must still be able to, and the
    exception escaping is what made every failing arm unmeasurable.
    """
    import usb.util
    from struct import pack

    if append_zeros:
        data = data + b"\x00" * (len(data) % block)
    total = len(data)
    blocks = total // block + (1 if total % block else 0)

    cfg = dev.dev.get_active_configuration()
    intf = cfg[(0, 0)]
    ep = usb.util.find_descriptor(
        intf,
        custom_match=lambda e: usb.util.endpoint_direction(
            e.bEndpointAddress) == usb.util.ENDPOINT_OUT)

    try:
        # 0x11 = REQ_WR_LARGE_MEM (arch/arm/mach-meson/meson-usb.c).
        dev.dev.ctrl_transfer(bmRequestType=0x40, bRequest=0x11,
                              wValue=block, wIndex=blocks,
                              data_or_wLength=pack('<IIII', address,
                                                   total, 0, 0))
    except Exception as e:                        # noqa: BLE001
        return total, 0, f"header {type(e).__name__}: {e}"

    accepted, off = 0, 0
    while off < total:
        try:
            accepted += ep.write(data[off:off + block], 5000)
        except Exception as e:                    # noqa: BLE001
            return total, accepted, (f"packet at off=0x{off:x} "
                                     f"{type(e).__name__}: {e}")
        off += block
    return total, accepted, None


def _ep0_read(dev, addr, n=64):
    try:
        return bytes(dev.readSimpleMemory(addr, n)), None
    except Exception as e:                        # noqa: BLE001
        return None, f"{type(e).__name__}: {e}"


def _ep0_read_reconnect(addr, n=64):
    """Retry the readback on a FRESH libusb handle after EP0 timed out.

    "EP0 did not answer this handle" and "the device is gone" are different
    facts, and only the second one makes the destination unreadable.  A new
    handle also re-runs identify, so the answer says WHO is on the bus now --
    our stage, or a BootROM that took over.
    """
    import usb.util

    try:
        raw = _find_device(quiet=True)
        if raw is None:
            return None, "no 1b8e:c003 at the port path any more"
        d = pyamlboot.AmlogicSoC.__new__(pyamlboot.AmlogicSoC)
        d.dev = raw
        s = d.identify()
        who = tuple(ord(c) for c in s[:4])
        tag = ("our stage" if who == STAGE_IDENTIFY else
               "the BootROM" if who == ROM_IDENTIFY else "unknown")
        got = bytes(d.readSimpleMemory(addr, n))
        usb.util.dispose_resources(raw)
        return got, f"fresh handle, responder={tag} ({_ident_str(who)})"
    except Exception as e:                        # noqa: BLE001
        return None, f"fresh handle also failed: {type(e).__name__}: {e}"


def send_dram_spl(dev, spl_data):
    """Write the chain's DRAM SPL stage before the FIT -- and READ IT BACK.

    N145 has two signatures and neither said what is actually AT 0x08000000
    when the chain fails to jump.  A readback separates "the stage has the
    data and does not act on it" from "the stage never got it", which every
    remaining candidate mechanism is conditioned on.  Measured 2026-08-31:
    ai-swift-16 BIT-IDENTICAL 6/6, s7-nyc 0/12 -- with the destination proven
    UNTOUCHED on the two arms where EP0 outlived the bulk.

    READBACK-UNREACHABLE is a distinct outcome, not a failure of the
    experiment: on most failing arms EP0 is dead after the bulk, so "the
    readback did not answer" and "the payload is wrong" must not collapse
    into one string.  The pre-read is what makes UNTOUCHED provable -- the
    same technique that settled the BootROM first-write erratum.
    """
    print(f"Sending DRAM SPL ({len(spl_data)} bytes) to "
          f"0x{DRAM_SPL_ADDRESS:08x}...")
    offs = _dram_witness_offsets(spl_data)

    pre = {}
    for o in offs:
        got, err = _ep0_read(dev, DRAM_SPL_ADDRESS + o)
        pre[o] = got
        if got is None:
            print(f"  RB pre  @+0x{o:06x}: EP0 UNREACHABLE ({err})")
        else:
            print(f"  RB pre  @+0x{o:06x}: {got[:16].hex()}")

    t0 = time.time()
    claimed, accepted, err = _write_large_counted(dev, DRAM_SPL_ADDRESS,
                                                 spl_data, 512, True)
    dt = time.time() - t0
    print(f"  {len(spl_data)/dt/1024:.0f} KB/s")
    print(f"  RB bulk: claimed={claimed} accepted={accepted} "
          f"{'OK' if accepted == claimed else 'SHORT'} in {dt:.3f}s"
          + (f"  FAILED: {err}" if err else ""))

    n_match = n_pre = n_other = n_dead = 0
    for o in offs:
        want = spl_data[o:o + 64]
        got, rerr = _ep0_read(dev, DRAM_SPL_ADDRESS + o)
        if got is None:
            got, rerr = _ep0_read_reconnect(DRAM_SPL_ADDRESS + o)
        if got is None:
            n_dead += 1
            print(f"  RB post @+0x{o:06x}: READBACK-UNREACHABLE ({rerr})")
            continue
        if got == want:
            n_match += 1
            verdict = "MATCH"
        elif pre[o] is not None and got == pre[o]:
            n_pre += 1
            verdict = "UNTOUCHED (identical to pre-write)"
        else:
            n_other += 1
            verdict = "MISMATCH"
        print(f"  RB post @+0x{o:06x}: {verdict}  got={got[:16].hex()} "
              f"want={want[:16].hex()}  [{rerr or 'same handle'}]")

    if n_dead == len(offs):
        v = "READBACK-UNREACHABLE"
    elif n_match == len(offs):
        v = "BIT-IDENTICAL"
    elif n_match == 0 and n_pre:
        v = "UNTOUCHED"
    else:
        v = "PARTIAL"
    print(f"  RB VERDICT: {v} (match={n_match} pre={n_pre} "
          f"other={n_other} dead={n_dead} of {len(offs)})")
    if err:
        # The readback is the measurement; the bulk failure is still fatal to
        # the chain, so do not go on to push the FIT at a dead endpoint.
        raise RuntimeError(f"DRAM-SPL bulk failed after the readback: {err}")
    if v in ("UNTOUCHED", "PARTIAL"):
        # The readback PROVES the DRAM SPL is not at its entry as sent.
        # Going on would end in REQ_RUN into it: the TPL branches into
        # whatever is there with its watchdog stopped, and the board goes
        # dark until a power cycle (FMA round F1 FC2, claude amlogic/
        # software/boot/gxl/audit/2026-10-19-gxl-ddr3-boot-chain-failure-
        # modes.md).  READBACK-UNREACHABLE proves nothing either way and
        # still proceeds; the TPL's own entry check is the backstop there.
        raise RuntimeError(f"DRAM SPL did not land ({v}); not sending RUN. "
                           "Power-cycle the board back into the BootROM "
                           "and retry.")


# The SPL's FIT working buffer on meson is a FIXED address, not a size-aware
# allocation: board_spl_fit_buffer_addr() in arch/arm/mach-meson/spl.c returns
# spl_get_load_buffer(), which is CONFIG_TEXT_BASE + 0x4000000 = 0x05000000 on
# GXL.  BL31's load address is 0x05100000, exactly 1 MiB above it.  With an
# external-data FIT only the ~2 KiB header is copied there and BL31 is clear.
# With an EMBEDDED-data FIT the whole thing is copied, so any FIT larger than
# 1 MiB contains its own BL31 destination and is overwritten mid-parse.
GX_FIT_BUFFER = 0x05000000
GX_ATF_LOAD = 0x05100000


def check_fit_is_external(fit_data, path):
    """Refuse a FIT whose image data is embedded and would overrun BL31.

    This is not a workaround for an unexplained failure: the mechanism is
    measured (see the module comment above).  binman's u-boot-amlogic.fit.itb
    is the mkimage INTERMEDIATE and always carries embedded data; the bootable
    FIT is the external-data repack binman packs into the image.
    """
    if len(fit_data) < 8:
        return
    magic, totalsize = struct.unpack(">II", fit_data[:8])
    if magic != FDT_MAGIC:
        return
    if totalsize < len(fit_data):
        return                      # external data: header only, fine
    span = GX_ATF_LOAD - GX_FIT_BUFFER
    if totalsize <= span:
        return                      # embedded, but small enough to clear BL31
    print(f"ERROR: {path} is an EMBEDDED-data FIT of {totalsize} bytes.")
    print(f"  The SPL copies the whole FIT to 0x{GX_FIT_BUFFER:08x}, so it "
          f"spans 0x{GX_FIT_BUFFER + totalsize:08x} and contains BL31's own")
    print(f"  load address 0x{GX_ATF_LOAD:08x}.  Loading the atf image "
          f"overwrites the FIT while it is still being parsed; the next")
    print("  lookups fail with \"cannot find image node\" and BL31 is never "
          "entered.  Measured on aml-s905x-cc-v2 2026-08-12.")
    print("  Stage the FIT that is IN the image instead: pass the whole "
          "u-boot-meson-with-spl.bin / out/<target>, or extract the binman")
    print("  'fit' entry, never u-boot-amlogic.fit.itb.")
    sys.exit(1)


def run_pass1(args, spl_data):
    """Pass 1: the SRAM stage to the BootROM, and RUN.  Pins PORT_PATH."""
    global PORT_PATH
    import usb.util

    print("\n=== Pass 1: DDR initialization ===")
    if not wait_for_device(args.timeout):
        print("ERROR: No Amlogic USB device found")
        sys.exit(1)
    # Talk to the BootROM once the host kernel has finished enumerating it
    # (configured in sysfs), rather than after a fixed 0.5 s.
    t_cfg = time.time()
    while time.time() < t_cfg + 5:
        raw = _find_device(quiet=True)
        if raw is not None and _configured(raw):
            break
        time.sleep(0.02)

    try:
        dev = connect()
    except Exception as e:
        # Same decision as pass 2's guard: a device whose identify() does not
        # answer must not be written to.  Without this, a bare traceback is
        # the whole diagnosis.
        print(f"ERROR: pass-1 identify failed: {type(e).__name__}: {e}")
        print("  The device did not answer identify().  Reset it back into "
              "the BootROM (power-cycle the port) and retry.")
        sys.exit(1)

    # Pass 1 must talk to the BootROM, for the same reason pass 2 must talk to
    # our stage: ROM and stage share 1b8e:c003 at the same port path, so the
    # identify line is the ONLY discriminator -- and printing it while
    # proceeding regardless is what this tool used to do at both ends.
    # Measured 2026-08-24 on s7-nyc: after a vendor BL2 booted and entered its
    # own USB stage, the next pass 1 read "ROM: 2.2 Stage: 0.8", wrote into
    # that stage instead of the ROM, and died in a libusb timeout.  A board in
    # a running stage needs a reset back into the ROM, which is a physical
    # act -- so say that, rather than writing into whatever answered.
    # (A board already in our stage resumes with --pass2-only instead.)
    if dev.identify_bytes != ROM_IDENTIFY:
        who = ("our own SRAM stage" if dev.identify_bytes == STAGE_IDENTIFY
               else "an unknown responder")
        print(f"ERROR: pass-1 GATE: {who} is answering, not the BootROM "
              f"(identify {_ident_str(dev.identify_bytes)}, expected "
              f"{_ident_str(ROM_IDENTIFY)}) -- not writing.")
        print("  The board is already running something.  Reset it back into "
              "the BootROM and retry" + (", or resume with --pass2-only."
                                         if dev.identify_bytes == STAGE_IDENTIFY
                                         else "."))
        sys.exit(1)

    # Pin pass 2 to the board pass 1 actually talked to, even when --port was
    # omitted.  Without this, pass 2 re-runs the search and can land on a
    # different board than the one now running our SPL -- and BOTH passes
    # would report success, which is the hardest version of this to notice.
    if PORT_PATH is None:
        PORT_PATH = _port_path_of(dev.dev)
        print(f"  pinning pass 2 to port path {PORT_PATH}")

    # Erratum workaround -- must come AFTER the ROM gate above (we only warm
    # up something we have just confirmed is the BootROM) and BEFORE the first
    # real transfer.  See brom_warmup().
    if args.brom_pre_reads:
        brom_pre_reads(dev)
    if args.brom_pre_read_count:
        # Diagnostic: N identical 4-byte reads of GOTGCTL, to tell the number
        # of requests before the first write from which registers are read.
        print(f"  BROM pre-reads: {args.brom_pre_read_count} x 4-byte read "
              "of 0xc9100000")
        for _ in range(args.brom_pre_read_count):
            dev.readSimpleMemory(0xc9100000, 4)
    brom_warmup(dev)

    send_spl(dev, spl_data)

    # Release the pass-1 libusb handle BEFORE waiting for SPL re-enumeration.
    # SPL soft-disconnects DWC2, reinitializes as USB device, then reconnects.
    # If we keep the old handle open, the pass-2 connect() will get EBUSY.
    usb.util.dispose_resources(dev.dev)
    del dev


def main():
    parser = argparse.ArgumentParser(
        description="USB boot for Amlogic Meson GX boards with open-source SPL")
    parser.add_argument("image", nargs="?",
                        help="u-boot-meson-with-spl.bin (contains SPL + FIT)")
    parser.add_argument("--spl",
                        help="SPL binary (signed amlimage)")
    parser.add_argument("--fit",
                        help="FIT image (u-boot.itb)")
    parser.add_argument("--dram-spl",
                        help="TPL->SPL chain: raw spl/u-boot-spl.bin to write "
                             f"to 0x{DRAM_SPL_ADDRESS:08x} before the FIT.  "
                             "The SRAM stage in this mode is TPL "
                             "(tpl/u-boot-tpl-signed.bin), passed as --spl.")
    parser.add_argument("--timeout", type=int, default=30,
                        help="USB detection timeout in seconds (default: 30)")
    parser.add_argument("--stage-probe-ms", type=int, default=5000,
                        help="timeout of one pass-2 stage probe (identify) in "
                             "ms (default: 5000); a probe blocks while the "
                             "TPL trains DRAM, and a timed-out one is simply "
                             "repeated")
    parser.add_argument("--reboot-delay", type=float, default=0,
                        help="Extra fixed wait before pass 2 starts probing "
                             "for the stage (default: 0 -- pass 2 starts as "
                             "soon as the stage answers identify)")
    parser.add_argument("--pass2-retries", type=int, default=4,
                        help="How many times pass 2 may re-scan and "
                             "re-identify when the responder is not our stage "
                             "(default: 4).  A responder that identifies as "
                             "the BootROM is NEVER written to.")
    parser.add_argument("--pass2-retry-delay", type=float, default=1.5,
                        help="Seconds between pass-2 identify retries "
                             "(default: 1.5)")
    parser.add_argument("--pass1-only", action="store_true",
                        help="Only do pass 1 (DDR init), don't boot")
    parser.add_argument("--pass2-only", action="store_true",
                        help="The board already runs our stage (a resumed "
                             "session, or a stage back after a host bus "
                             "reset): skip pass 1, wait for the stage, then "
                             "send --dram-spl and --fit.  Needs --port")
    parser.add_argument("--identify-only", action="store_true",
                        help="Wait for our stage to answer identify at --port "
                             "and exit 0 (1 at --timeout); sends nothing else")
    parser.add_argument("--port",
                        help="USB port path of the board to boot, e.g. 2,3,2 "
                             "(from /sys/bus/usb/devices/3-2.3.2).  REQUIRED "
                             "when more than one 1b8e:c003 is attached -- they "
                             "are indistinguishable by VID/PID, so without it "
                             "the target is whichever enumerates first.")
    parser.add_argument("--probe-ladder", action="store_true",
                        help="Pass 2 diagnostic instead of the chain: write "
                             "512, 1K, 2K ... of --dram-spl to --probe-addr "
                             "as separate WR_LARGE transfers, reading back "
                             "over EP0 after each, and stop at the first rung "
                             "that does not land.  This is the N145 "
                             "reproducer.")
    parser.add_argument("--probe-block", default="512",
                        help="blockLength for --probe-ladder (default 512). "
                             "64 exercises the stage's SHORT-packet path "
                             "instead of its 512 B full-MPS path.")
    parser.add_argument("--probe-addr", default=hex(DRAM_SPL_ADDRESS),
                        help="Destination for --probe-ladder "
                             f"(default 0x{DRAM_SPL_ADDRESS:08x}).  Varying "
                             "it is how the wedge was shown to be "
                             "address-independent.")
    parser.add_argument("--brom-pre-reads", action="store_true",
                        help="Pass 1: read the ROM's USB controller registers "
                             "before the warm-up write.  A diagnostic for the "
                             "GXL BootROM stall after a Linux warm restart "
                             "(see brom_pre_reads()); off by default")
    parser.add_argument("--brom-pre-read-count", type=int, default=0,
                        help="Diagnostic: N identical 4-byte ROM reads "
                             "before the warm-up write (default 0)")
    parser.add_argument("--fit-addr",
                        help="Override the FIT staging address. Normally "
                             "derived from the build's "
                             "CONFIG_SPL_LOAD_FIT_ADDRESS -- only pass this "
                             "when no u-boot config is reachable.")
    parser.add_argument("--fit-offset",
                        help="Override where the FIT starts in --image. "
                             "Normally derived from the build's "
                             "CONFIG_MESON_GX_FIT_OFFSET.")
    args = parser.parse_args()

    global PORT_PATH
    if args.port:
        PORT_PATH = tuple(int(x) for x in args.port.split(","))
    if (args.pass2_only or args.identify_only) and PORT_PATH is None:
        parser.error("--pass2-only and --identify-only need --port: with no "
                     "pass 1 there is no board to pin")

    if args.identify_only:
        dt = wait_for_stage(args.timeout, probe_ms=args.stage_probe_ms)
        sys.exit(0 if dt is not None else 1)

    # Parse input files
    if args.image:
        with open(args.image, "rb") as f:
            full_image = f.read()
        fit_offset, src = resolve_fit_offset(args)
        print(f"FIT at image offset 0x{fit_offset:x} ({FIT_OFFSET_SYMBOL} from {src})")
        spl_data = extract_spl_from_image(full_image[:fit_offset])
        fit_data = full_image[fit_offset:]
        if len(fit_data) == 0:
            print("ERROR: Image contains no FIT data after SPL pad boundary")
            sys.exit(1)
        # Verify FIT magic
        if len(fit_data) >= 4:
            magic = struct.unpack('>I', fit_data[:4])[0]
            if magic != FDT_MAGIC:
                print(f"WARNING: FIT does not start with FDT magic "
                      f"(got 0x{magic:08x}, expected 0x{FDT_MAGIC:08x})")
    elif args.spl and args.fit:
        with open(args.spl, "rb") as f:
            spl_data = f.read()
        with open(args.fit, "rb") as f:
            fit_data = f.read()
    elif args.spl and args.pass1_only:
        with open(args.spl, "rb") as f:
            spl_data = f.read()
        fit_data = None
    elif args.pass2_only and args.fit:
        spl_data = b""			# pass 1 is skipped: no SRAM stage to send
        with open(args.fit, "rb") as f:
            fit_data = f.read()
    else:
        parser.error("Provide either a combined image, or --spl and --fit, "
                     "or --spl with --pass1-only")

    if fit_data is not None:
        check_fit_is_external(fit_data, args.fit or args.image)

    print(f"SPL: {len(spl_data)} bytes"
          + (f", FIT: {len(fit_data)} bytes" if fit_data else ""))

    if fit_data is not None:
        global FIT_LOAD_ADDRESS
        FIT_LOAD_ADDRESS, src = resolve_fit_address(args)
        print(f"FIT staging address 0x{FIT_LOAD_ADDRESS:08x} "
              f"({FIT_ADDR_SYMBOL} from {src})")

    if args.pass2_only:
        # The board is already running our stage (a resumed session, or a
        # stage that came back after a host bus reset): no BootROM pass.
        print("\n=== Pass 1 skipped (--pass2-only): the stage is already running ===")
    else:
        run_pass1(args, spl_data)
        if args.pass1_only:
            print("\nPass 1 done (--pass1-only).  Check UART for DDR init output.")
            return

    # The stage takes over the session after DRAM training (or re-enumerates
    # as its own device); wait for it to answer rather than for a fixed time.
    if args.reboot_delay:
        print(f"\nWaiting {args.reboot_delay}s before probing the stage...")
        time.sleep(args.reboot_delay)
    print("\nWaiting for the stage to answer identify...")
    ready = wait_for_stage(args.timeout, probe_ms=args.stage_probe_ms)
    if BOARD_RESET_SEEN:
        print("ERROR: the board reset back into the BootROM before our stage "
              "answered; not starting pass 2.  The BootROM was left alone, so "
              "it can be staged again now, with no power cycle.")
        sys.exit(1)

    # --- Pass 2: Deliver FIT to SPL USB device ---
    print("\n=== Pass 2: Load FIT via SPL USB device ===")
    dev = connect_stage(args.timeout, args.pass2_retries, args.pass2_retry_delay,
                        settle=0 if ready is not None else 0.5)
    if dev is None:
        print("Possible causes:")
        print("  - DDR init failed (check UART for 'DF' message)")
        print("  - SPL DWC2 init failed")
        print("  - the stage's USB take-over lost the race and the BootROM "
              "re-enumerated in its place (see the GATE lines above)")
        sys.exit(1)

    if args.probe_ladder:
        if not args.dram_spl:
            print("ERROR: --probe-ladder needs --dram-spl for its payload.")
            sys.exit(1)
        with open(args.dram_spl, "rb") as f:
            blob = f.read()
        addr = int(args.probe_addr, 0)
        blk = int(args.probe_block, 0)
        for size in (blk, 512, 1024, 2048, 4096, 8192, 16384, 32768,
                     len(blob)):
            if size > len(blob):
                continue
            print(f"\n--- PROBE {size} B -> 0x{addr:08x} block={blk} ---")
            pre, _ = _ep0_read(dev, addr)
            t0 = time.time()
            claimed, accepted, err = _write_large_counted(dev, addr,
                                                          blob[:size], blk,
                                                          True)
            dt = time.time() - t0
            print(f"  claimed={claimed} accepted={accepted} "
                  f"{dt * 1000:.1f} ms {accepted / dt / 1024:.0f} KB/s"
                  + (f"  FAILED: {err}" if err else "  OK"))
            got, rerr = _ep0_read(dev, addr)
            if got is None:
                print(f"  readback: EP0 dead on this handle ({rerr})")
                got, rerr = _ep0_read_reconnect(addr)
            src = rerr or "same handle"
            if got is None:
                print(f"  readback: UNREACHABLE ({rerr})")
            elif got == blob[:64]:
                print(f"  readback: MATCH {got[:16].hex()}  [{src}]")
            elif pre is not None and got == pre:
                print(f"  readback: UNTOUCHED {got[:16].hex()}  [{src}]")
            else:
                print(f"  readback: MISMATCH got={got[:16].hex()} "
                      f"want={blob[:16].hex()}  [{src}]")
            if err or got is None:
                print(f"  PROBE LADDER STOPS AT {size} B")
                sys.exit(2)
        print("\nPROBE LADDER: every rung landed")
        sys.exit(0)

    if args.dram_spl:
        with open(args.dram_spl, "rb") as f:
            send_dram_spl(dev, f.read())
    send_fit(dev, fit_data)
    # Signal FIT delivery complete.  SPL will exit USB loop and boot from RAM.
    print(f"Signalling run at 0x{FIT_LOAD_ADDRESS:08x}...")
    import usb.core
    try:
        dev.run(FIT_LOAD_ADDRESS)
    except usb.core.USBError as e:
        # The stage drops off USB as soon as it takes the jump, so the run
        # request's status stage usually errors (EIO) on a GOOD boot: every
        # staged boot on AML-S905X-CC and CC-V2 did, 2026-10-19, and the
        # stager exited 1 on boots that reached Linux.  Gone from the bus a
        # moment later means the jump was taken; still there means the
        # request did not land, and that stays a failure.
        time.sleep(1.0)
        if _find_device(quiet=True) is not None:
            print(f"ERROR: run request failed ({e}) and the device is "
                  f"still on the bus")
            sys.exit(1)
        print(f"  run: the device left the bus as it jumped ({e}); taken")

    print("\nDone. Check UART for boot output.")
    print("Expected: SPL prints 'FIT', then loads ATF + u-boot from RAM.")


if __name__ == "__main__":
    main()
