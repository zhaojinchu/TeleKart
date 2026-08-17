#!/usr/bin/env python3
"""Measure real wheel speed from the encoders.

Answers a question we have only ever guessed at: how fast do these wheels actually turn?
Every speed figure in this project so far has been arithmetic from a nameplate, and the
document those numbers came from has already been wrong once about this power chain.

**Read-only.** This touches the four encoder input pins and nothing else, so it can run
while the ``telekart2`` service is driving the car. Drive with the wheel as normal; this
just watches.

    # 1. is anything wired up? spin a wheel by hand and watch the counts move
    .venv/bin/python encoder_probe.py --raw

    # 2a. edges per revolution, by turning a wheel a counted number of times
    .venv/bin/python encoder_probe.py --calibrate --turns 10

    # 2b. better: push the car a measured distance -- a tape measure cannot miscount
    .venv/bin/python encoder_probe.py --metres 5

    # 3. how fast does it actually go? drive flat out while this runs
    .venv/bin/python encoder_probe.py --seconds 60

Counts rising edges on the A channel of each wheel only. Direction is not decoded --
we are measuring speed, and the gear you selected already says which way you are going.
That halves the interrupt load for free.
"""

from __future__ import annotations

import math
import sys
import time

import config

try:
    import pigpio
except ImportError:
    sys.exit("pigpio not importable -- run with ~/telekart2/.venv/bin/python")

CIRCUMFERENCE_M = math.pi * config.WHEEL_DIAMETER_M


def main() -> int:
    args = sys.argv[1:]
    raw = "--raw" in args
    calibrate = "--calibrate" in args
    seconds = 3600.0
    if "--seconds" in args:
        seconds = float(args[args.index("--seconds") + 1])
    turns = 10
    if "--turns" in args:
        turns = int(args[args.index("--turns") + 1])
    metres = 0.0
    if "--metres" in args:
        metres = float(args[args.index("--metres") + 1])

    pi = pigpio.pi()
    if not pi.connected:
        return _fail("cannot reach pigpiod -- sudo systemctl start pigpiod")

    for pin in (config.ENC_LEFT_A, config.ENC_RIGHT_A):
        pi.set_mode(pin, pigpio.INPUT)
        # Hall encoders are often open-collector; a pull-up is required for those and
        # harmless for push-pull outputs.
        pi.set_pull_up_down(pin, pigpio.PUD_UP)
        # At a plausible top speed edges are ~300 us apart, so 20 us of debounce removes
        # electrical noise without ever eating a real edge.
        pi.set_glitch_filter(pin, 20)

    # tally() counts in pigpio's C thread -- no Python call per edge, which matters at a
    # few thousand edges per second on a Zero 2 W.
    cb_l = pi.callback(config.ENC_LEFT_A, pigpio.RISING_EDGE)
    cb_r = pi.callback(config.ENC_RIGHT_A, pigpio.RISING_EDGE)

    try:
        if raw:
            return _raw(pi, cb_l, cb_r, min(seconds, 30.0))
        if metres > 0:
            return _calibrate_distance(cb_l, cb_r, metres)
        if calibrate:
            return _calibrate(cb_l, cb_r, turns)
        return _measure(cb_l, cb_r, seconds)
    finally:
        cb_l.cancel()
        cb_r.cancel()
        pi.stop()


def _raw(pi, cb_l, cb_r, seconds: float) -> int:
    """Is anything actually connected? Spin each wheel by hand and watch."""
    print("Spin each wheel by hand. Counts should climb, and the level should flicker.")
    print("If a wheel's count never moves, that encoder is not wired or not powered.\n")
    print("   left_count  right_count   levelA(L,R)")
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end:
            print(
                f"   {cb_l.tally():10d}  {cb_r.tally():11d}   "
                f"{pi.read(config.ENC_LEFT_A)},{pi.read(config.ENC_RIGHT_A)}",
                end="\r",
                flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def _calibrate(cb_l, cb_r, turns: int) -> int:
    """Counts per revolution, measured rather than assumed.

    Triggered by movement rather than keypresses: starts on the first edge and finishes
    after the wheel has been still for a moment. Nothing here can read a keyboard --
    this often runs through a non-interactive ssh with no terminal attached.
    """
    idle_stop = 2.5
    print(f"Mark a tyre, then turn ONE wheel by hand exactly {turns} full revolutions.")
    print(f"Timing starts on its own when you begin, and finishes {idle_stop:.0f}s after")
    print("you stop. Take as long as you like.\n")
    print("  waiting for movement...", flush=True)

    cb_l.reset_tally()
    cb_r.reset_tally()

    deadline = time.monotonic() + 120.0
    while cb_l.tally() == 0 and cb_r.tally() == 0:
        if time.monotonic() > deadline:
            print("  nothing counted in 2 minutes. Check wiring/power, or try --raw.")
            return 1
        time.sleep(0.02)

    print("  counting...", flush=True)
    last_total = -1
    last_change = time.monotonic()
    while time.monotonic() - last_change < idle_stop:
        total = cb_l.tally() + cb_r.tally()
        if total != last_total:
            last_total = total
            last_change = time.monotonic()
            print(f"    {total} edges so far   ", end="\r", flush=True)
        time.sleep(0.05)

    left, right = cb_l.tally(), cb_r.tally()
    print(" " * 40, end="\r")
    print(f"\n  left  {left:6d} edges  ->  {left / turns:7.1f} per revolution")
    print(f"  right {right:6d} edges  ->  {right / turns:7.1f} per revolution")
    print(f"\n  config.ENC_COUNTS_PER_REV is currently {config.ENC_COUNTS_PER_REV}")
    moved = max(left, right)
    if moved:
        print(f"  measured: about {round(moved / turns)} -- put that in config.py")
    else:
        print("  nothing counted. Check wiring/power, or run with --raw first.")
    return 0


def _calibrate_distance(cb_l, cb_r, metres: float) -> int:
    """Counts per revolution, derived from rolling a measured distance.

    Better than counting revolutions by hand: a tape measure cannot lose count at turn
    eight, and this measures the thing we actually want -- edges per metre of ground --
    without going through an assumed wheel diameter at all.

    Push the car; do not drive it. Wheelspin under power would inflate the count.
    """
    idle_stop = 2.5
    print(f"Push the car (do NOT drive it) in a straight line exactly {metres:.1f} m.")
    print(f"Counting starts when it moves and finishes {idle_stop:.0f}s after it stops.\n")
    print("  waiting for movement...", flush=True)

    cb_l.reset_tally()
    cb_r.reset_tally()
    deadline = time.monotonic() + 180.0
    while cb_l.tally() == 0 and cb_r.tally() == 0:
        if time.monotonic() > deadline:
            print("  nothing counted. Check wiring/power, or try --raw.")
            return 1
        time.sleep(0.02)

    print("  counting...", flush=True)
    last_total, last_change = -1, time.monotonic()
    while time.monotonic() - last_change < idle_stop:
        total = cb_l.tally() + cb_r.tally()
        if total != last_total:
            last_total, last_change = total, time.monotonic()
            print(f"    {total} edges so far   ", end="\r", flush=True)
        time.sleep(0.05)

    left, right = cb_l.tally(), cb_r.tally()
    print(" " * 40, end="\r")
    print(f"\n  over {metres:.1f} m:   left {left} edges,  right {right} edges\n")
    for name, n in (("left", left), ("right", right)):
        if n < 10:
            print(f"  {name:5s}: too few edges to trust -- encoder likely not connected")
            continue
        per_m = n / metres
        per_rev = per_m * CIRCUMFERENCE_M
        print(f"  {name:5s}: {per_m:7.1f} edges/m  ->  {per_rev:6.1f} edges/rev "
              f"(wheel {config.WHEEL_DIAMETER_M * 1000:.0f} mm)")
    print(f"\n  config.ENC_COUNTS_PER_REV is currently {config.ENC_COUNTS_PER_REV}")
    return 0


def _measure(cb_l, cb_r, seconds: float) -> int:
    """Live speed, and the peak seen so far."""
    cpr = config.ENC_COUNTS_PER_REV
    print(f"Measuring for {seconds:.0f}s. Drive it. (counts/rev = {cpr}, "
          f"wheel {config.WHEEL_DIAMETER_M * 1000:.0f} mm)\n")
    print("     left            right           peak")
    print("     rpm    m/s      rpm    m/s      rpm    m/s")

    # Wait for the wheels to actually turn before starting the clock. A fixed window
    # started when the command was typed measures whatever happened to be going on then,
    # which in practice is a person still reading the instructions.
    cb_l.reset_tally()
    cb_r.reset_tally()
    print("  waiting for the wheels to move...", flush=True)
    deadline = time.monotonic() + 600.0
    while cb_l.tally() < 60 and cb_r.tally() < 60:  # 60 edges = real rotation, not noise
        if time.monotonic() > deadline:
            print("  nothing moved in 10 minutes -- giving up.")
            return 1
        time.sleep(0.02)
    print(f"  moving. measuring for {seconds:.0f}s\n", flush=True)

    cb_l.reset_tally()
    cb_r.reset_tally()
    last_l, last_r = 0, 0
    peak_rpm = 0.0
    t_last = time.monotonic()
    t_end = t_last + seconds

    try:
        while time.monotonic() < t_end:
            time.sleep(0.25)
            now = time.monotonic()
            dt = now - t_last
            t_last = now

            l, r = cb_l.tally(), cb_r.tally()
            dl, dr = l - last_l, r - last_r
            last_l, last_r = l, r

            rpm_l = (dl / dt) / cpr * 60.0
            rpm_r = (dr / dt) / cpr * 60.0
            peak_rpm = max(peak_rpm, rpm_l, rpm_r)

            print(
                f"  {rpm_l:7.1f} {_mps(rpm_l):6.2f}   {rpm_r:7.1f} {_mps(rpm_r):6.2f}   "
                f"{peak_rpm:7.1f} {_mps(peak_rpm):6.2f}",
                end="\r",
                flush=True,
            )
    except KeyboardInterrupt:
        pass

    print(f"\n\n  PEAK: {peak_rpm:.1f} rpm = {_mps(peak_rpm):.2f} m/s")
    print(f"  (10.6 V at the motors implies ~318 rpm / ~1.08 m/s unloaded)")
    return 0


def _mps(rpm: float) -> float:
    return rpm / 60.0 * CIRCUMFERENCE_M


def _fail(msg: str) -> int:
    print(msg)
    return 1


if __name__ == "__main__":
    sys.exit(main())
