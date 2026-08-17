#!/usr/bin/env python3
"""Watch the wheel and log every change, to work out what its controls do.

Tries the XInput path first (raw USB, analog triggers), then falls back to pygame for
HID devices. Which one you get matters: this wheel's Switch-emulation personality
reports its pedals as plain buttons, while its XInput personality carries them as
analog trigger bytes. Same wheel, different information.

Records *transitions* rather than sampling fixed windows. An earlier version prompted
"hold full left for 4 seconds" and averaged each window, which conflated the value with
how fast you moved into it -- two runs disagreed purely on timing, and a pedal that was
really a switch averaged out looking analog. Logging changes removes the clock from the
measurement: go at any pace, in any order, and the trace still says what moved and when.

    .venv/bin/python mac/calibrate_input.py            # 45s
    .venv/bin/python mac/calibrate_input.py 90

Streams to logs/input-calibration.jsonl as it goes, so killing it keeps the data.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "mac"))
JSONL = REPO / "logs" / "input-calibration.jsonl"

#: A control must move at least this much to count as a change -- enough to ignore
#: potentiometer noise on a cheap wheel, small enough to keep real pedal travel.
EPS = 0.02

PROMPT = """
Do these at your own pace, pausing a moment between each:
  1. hands off everything
  2. wheel fully LEFT, then fully RIGHT, then centre
  3. throttle pedal down SLOWLY to the floor, then release
  4. brake pedal down SLOWLY to the floor, then release
  5. LEFT paddle, then RIGHT paddle
"""

#: Standard wired-XInput button bit assignments, for readable output.
XINPUT_BITS = {
    0: "dpad up", 1: "dpad down", 2: "dpad left", 3: "dpad right",
    4: "start", 5: "back", 6: "L3", 7: "R3",
    8: "LB / left paddle", 9: "RB / right paddle", 10: "guide",
    12: "A", 13: "B", 14: "X", 15: "Y",
}


def main() -> int:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0
    JSONL.parent.mkdir(exist_ok=True)

    from xinput import XInputDevice

    dev = XInputDevice.find()
    if dev is None:
        print("No XInput wheel found. Is it plugged in, and in its XInput mode?")
        return 1

    print(f"\nDevice: {dev.name}  (XInput over raw USB -- analog triggers available)")
    print(f"Recording {duration:.0f}s.")
    print(PROMPT)

    stream = JSONL.open("w", buffering=1)
    stream.write(json.dumps({"device": dev.name, "mode": "xinput"}) + "\n")

    fields = ("lx", "ly", "rx", "ry", "lt", "rt")
    last = dict.fromkeys(fields)
    lo = dict.fromkeys(fields, 9.9)
    hi = dict.fromkeys(fields, -9.9)
    counts = {f: 0 for f in fields}
    seen = {f: set() for f in fields}
    last_btn = 0

    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            s = dev.poll()
            if s is None:
                continue
            t = round(time.time() - t0, 3)
            for f in fields:
                v = round(getattr(s, f), 4)
                lo[f], hi[f] = min(lo[f], v), max(hi[f], v)
                seen[f].add(v)
                if last[f] is None or abs(v - last[f]) >= EPS:
                    counts[f] += 1
                    stream.write(json.dumps({"t": t, "c": f, "v": v}) + "\n")
                    print(f"  {t:6.2f}s  {f} -> {v:+.3f}", flush=True)
                    last[f] = v
            if s.buttons != last_btn:
                bits = [b for b in range(16) if s.buttons & (1 << b)]
                names = ", ".join(f"bit {b} ({XINPUT_BITS.get(b, '?')})" for b in bits) or "none"
                stream.write(
                    json.dumps({"t": t, "c": "buttons", "v": s.buttons, "bits": bits}) + "\n"
                )
                print(f"  {t:6.2f}s  buttons -> {names}", flush=True)
                last_btn = s.buttons
    except KeyboardInterrupt:
        print("\nstopped early")
    finally:
        stream.close()
        dev.close()

    print("\n  control   min      max     span   distinct   verdict")
    for f in fields:
        span = hi[f] - lo[f]
        d = len(seen[f])
        verdict = (
            "unused" if span < 0.05
            else f"ANALOG ({d} levels)" if d > 8
            else f"DIGITAL ({d} levels)"
        )
        print(f"  {f:<8}  {lo[f]:+.3f}  {hi[f]:+.3f}  {span:6.3f}  {d:8d}   {verdict}")
    print(f"\nSaved to {JSONL}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
