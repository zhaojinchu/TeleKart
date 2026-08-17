#!/usr/bin/env python3
"""Bring-up utility: hold the steering servo at given pulse widths so a human can look.

Finding true centre needs eyes on the linkage, so this exists to make the round trip
cheap: name some pulse widths, watch the wheels, read the numbers back into config.py.

    .venv/bin/python servo_probe.py 1500 1600 1700       # 2 s each, then release
    .venv/bin/python servo_probe.py --hold 4 1500

Nothing here touches the motor pins. Run it only with ``main.py`` stopped -- two
processes driving GPIO18 produce a pulse train that is neither one's idea of a position.

Safety: holds are short and the pulse train is always released at the end. A servo parked
against a mechanical stop stalls, and this one shares the Pi's 5 V rail, so a sustained
stall browns out the SoC rather than politely reporting an error.
"""

from __future__ import annotations

import sys
import time

import config

try:
    import pigpio
except ImportError:
    sys.exit("pigpio not importable -- run this with ~/telekart2/.venv/bin/python")

#: Wider than config's driving limits on purpose: calibration has to be able to look
#: outside the current window to discover that the window is in the wrong place. Still
#: inside what an HS-311 will accept, so a typo cannot drive the horn into its end stop.
ABS_MIN_US = 900
#: Past ~2100 an HS-311 is out of its rated span and is being pushed toward its internal
#: end stop. Raised to 2400 during bring-up on 2026-08-16 only to *measure* where travel
#: actually ends -- the tell is that each further step produces less motion, not more.
#: Do not treat anything above 2100 as a usable driving limit.
ABS_MAX_US = 2400


def main() -> int:
    args = sys.argv[1:]
    hold = 2.0
    if "--hold" in args:
        i = args.index("--hold")
        hold = float(args[i + 1])
        del args[i : i + 2]

    if not args:
        return _usage()

    try:
        values = [int(a) for a in args]
    except ValueError:
        return _usage()

    clamped = [v for v in values if not ABS_MIN_US <= v <= ABS_MAX_US]
    if clamped:
        print(f"refusing {clamped}: outside the safe band {ABS_MIN_US}-{ABS_MAX_US} us")
        return 1

    pi = pigpio.pi()
    if not pi.connected:
        return _fail("cannot reach pigpiod -- sudo systemctl start pigpiod")

    print(f"centre in config.py is {config.vehicle_config().servo_centre_us} us")
    print(f"driving limits are {config.vehicle_config().servo_min_us}"
          f"-{config.vehicle_config().servo_max_us} us\n")
    try:
        for us in values:
            print(f"  {us:4d} us  holding {hold:.1f}s ...", flush=True)
            pi.set_servo_pulsewidth(config.SERVO, us)
            time.sleep(hold)
    finally:
        # Always stop pulsing, including on Ctrl-C. Leaving a servo energised against a
        # stop is the failure this whole file is trying to avoid.
        pi.set_servo_pulsewidth(config.SERVO, 0)
        pi.stop()
        print("\nreleased (pulses stopped)")
    return 0


def _usage() -> int:
    print(__doc__)
    return 2


def _fail(msg: str) -> int:
    print(msg)
    return 1


if __name__ == "__main__":
    sys.exit(main())
