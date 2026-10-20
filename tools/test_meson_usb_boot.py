#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""Host-side test for meson-usb-boot.py's pass-2 identify gate.

WHY THIS EXISTS RATHER THAN A HARDWARE RUN.  The gate answers a failure that
is INTERMITTENT -- measured 1 in 12 chain boots on 2026-08-12, and 0 in 16 the
day it was fixed, including 4 boots deliberately run with --reboot-delay 0 to
shorten the take-over race.  A path that cannot be made to fire on demand
cannot be proven by booting more boards; what CAN be proven, exactly, is the
decision the host makes when it is handed each of the two possible responders.
That is what this checks:

  - the BootROM's identify (02 04 00 00) must NEVER be written to, and must be
    retried the requested number of times before the tool gives up
  - our stage's identify (02 22 00 20) must be accepted
  - a stage that answers only on a later attempt must be accepted there

Run: python3 tools/test_meson_usb_boot.py   (no hardware, no pyamlboot)
"""

import importlib.util
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))

# meson-usb-boot.py imports pyamlboot and pyusb at module scope, and neither is
# needed to exercise the gate.  Stub them before the import rather than
# refactoring the tool: the tool's import list is part of what ships.
for name in ("usb", "usb.core", "usb.util", "usb.backend",
             "usb.backend.libusb1", "pyamlboot", "pyamlboot.pyamlboot"):
    sys.modules.setdefault(name, types.ModuleType(name))
sys.modules["usb"].core = sys.modules["usb.core"]
sys.modules["usb"].util = sys.modules["usb.util"]
sys.modules["usb.util"].dispose_resources = lambda _dev: None
sys.modules["pyamlboot"].pyamlboot = sys.modules["pyamlboot.pyamlboot"]
sys.modules["pyamlboot.pyamlboot"].AmlogicSoC = type("AmlogicSoC", (), {})

_spec = importlib.util.spec_from_file_location(
    "meson_usb_boot", os.path.join(HERE, "meson-usb-boot.py"))
mub = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mub)


class FakeDev:
    """Minimal stand-in for the connected device.

    `writes` is the assertion that matters: the gate's contract is that a
    responder it did not recognise is never written to.
    """

    def __init__(self, ident):
        self._ident = ident
        # connect() publishes this; connect_stage() gates on it.
        self.identify_bytes = tuple(ident)
        self.dev = object()
        self.writes = []

    def identify(self):
        return "".join(chr(b) for b in self._ident) + "\0" * 4

    def writeLargeMemory(self, *a, **kw):
        self.writes.append(("writeLargeMemory", a))

    def run(self, *a):
        self.writes.append(("run", a))


def install(responders):
    """Make connect() return each of `responders` in turn."""
    seq = list(responders)
    made = []

    def fake_connect():
        dev = FakeDev(seq.pop(0))
        made.append(dev)
        return dev

    mub.connect = fake_connect
    mub.wait_for_device = lambda timeout: True
    return made


FAILURES = []


def check(name, cond):
    print(("PASS  " if cond else "FAIL  ") + name)
    if not cond:
        FAILURES.append(name)


def main():
    # The two identify strings this whole gate is about, taken from the tool
    # itself so a change there fails the test rather than silently passing it.
    rom = mub.ROM_IDENTIFY
    stage = mub.STAGE_IDENTIFY
    check("ROM and stage identifies differ", rom != stage)
    check("stage identify is 02 22 00 20", stage == (0x02, 0x22, 0x00, 0x20))

    # 1. Only the BootROM ever answers -> refuse, and write nothing.
    made = install([rom] * 3)
    got = mub.connect_stage(timeout=1, retries=3, delay=0)
    check("BootROM-only: refuses (returns None)", got is None)
    check("BootROM-only: retried exactly 3 times", len(made) == 3)
    check("BootROM-only: NOTHING was written",
          all(not d.writes for d in made))

    # 2. Our stage answers first time -> accept on attempt 1.
    made = install([stage, stage])
    got = mub.connect_stage(timeout=1, retries=3, delay=0)
    check("stage-first: accepted", got is not None)
    check("stage-first: took one attempt", len(made) == 1)

    # 3. The take-over completes late -> the retry is what recovers it.
    made = install([rom, rom, stage])
    got = mub.connect_stage(timeout=1, retries=4, delay=0)
    check("late take-over: accepted on attempt 3", got is not None)
    check("late take-over: three attempts made", len(made) == 3)
    check("late take-over: the two ROM handles were never written",
          not made[0].writes and not made[1].writes)

    # 4. An unknown responder is treated like the ROM: refuse, never write.
    made = install([(0x02, 0x99, 0x00, 0x99)] * 2)
    got = mub.connect_stage(timeout=1, retries=2, delay=0)
    check("unknown responder: refuses", got is None)
    check("unknown responder: NOTHING was written",
          all(not d.writes for d in made))

    # 5. The device never enumerates -> None, not a crash.
    mub.wait_for_device = lambda timeout: False
    check("no device: returns None",
          mub.connect_stage(timeout=1, retries=2, delay=0) is None)

    # 6. The embedded-data FIT gate (N35).  An external-data FIT passes; an
    #    embedded one big enough to reach BL31's 0x05100000 is refused.
    def fit(totalsize, filesize):
        import struct as _s
        head = _s.pack(">II", 0xd00dfeed, totalsize)
        return head + b"\0" * (filesize - len(head))

    ok = True
    try:
        mub.check_fit_is_external(fit(2048, 1207296), "external.bin")
    except SystemExit:
        ok = False
    check("external-data FIT: accepted", ok)

    refused = False
    try:
        mub.check_fit_is_external(fit(1206006, 1206006), "embedded.itb")
    except SystemExit:
        refused = True
    check("embedded-data FIT over 1 MiB: refused", refused)

    small = True
    try:
        mub.check_fit_is_external(fit(4096, 4096), "small-embedded.itb")
    except SystemExit:
        small = False
    check("embedded-data FIT under 1 MiB: accepted", small)

    # 7. A responder whose identify() RAISES is retried and refused, not
    #    raised out of the loop.  Measured on GXL 2026-09-17 (amlogic N43):
    #    one unsupported control request deranges the BootROM's EP0, and
    #    connect() raised Errno 75 / Errno 110 straight out of this gate.
    calls = []

    def raising_connect():
        calls.append(1)
        raise OSError("[Errno 75] Overflow")

    mub.connect = raising_connect
    mub.wait_for_device = lambda timeout: True
    got = mub.connect_stage(timeout=1, retries=3, delay=0)
    check("identify-raises: refuses (returns None)", got is None)
    check("identify-raises: retried exactly 3 times", len(calls) == 3)

    # 8. identify() raises on the first attempts, then our stage answers ->
    #    the retry recovers it (the deranged state can be transient).
    fails = [OSError("x"), OSError("x")]

    def flaky_connect():
        if fails:
            raise fails.pop(0)
        return FakeDev(stage)

    mub.connect = flaky_connect
    mub.wait_for_device = lambda timeout: True
    got = mub.connect_stage(timeout=1, retries=4, delay=0)
    check("identify-flaky: accepted after failures", got is not None)

    # 9. The FIT offset in a combined --image comes from the build
    #    (CONFIG_MESON_GX_FIT_OFFSET), the flag overrides it, and with neither
    #    the tool refuses rather than cutting the image at a guessed byte.
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        img = os.path.join(d, "aml-s905x-cc")
        with open(img + ".config", "w") as f:
            f.write("CONFIG_MESON_GX_FIT_OFFSET=0x4c000\n")
        a = types.SimpleNamespace(image=img, fit=None, spl=None,
                                  fit_offset=None, fit_addr=None)
        off, src = mub.resolve_fit_offset(a)
        check("fit offset: read from <image>.config", off == 0x4c000 and src.endswith(".config"))
        a.fit_offset = "0x1fe00"
        off, src = mub.resolve_fit_offset(a)
        check("fit offset: --fit-offset overrides the config", off == 0x1fe00 and src == "--fit-offset")
    with tempfile.TemporaryDirectory() as d:
        a = types.SimpleNamespace(image=os.path.join(d, "x"), fit=None, spl=None,
                                  fit_offset=None, fit_addr=None)
        # The tool also looks for <tree>/.config beside tools/; point the
        # module at an empty tree so an in-tree build cannot satisfy it.
        os.makedirs(os.path.join(d, "tree", "tools"))
        real_file = mub.__file__
        mub.__file__ = os.path.join(d, "tree", "tools", "meson-usb-boot.py")
        refused = False
        try:
            mub.resolve_fit_offset(a)
        except SystemExit:
            refused = True
        finally:
            mub.__file__ = real_file
        check("fit offset: no config and no flag -> refused", refused)

    print()
    if FAILURES:
        print("%d FAILURE(S): %s" % (len(FAILURES), ", ".join(FAILURES)))
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
