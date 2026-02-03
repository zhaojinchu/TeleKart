#!/usr/bin/env python3
"""
testA_servo_constant.py

TEST A: "Set once and do nothing"
- Sets a constant servo pulse once (default 1500us center)
- Then sleeps without touching PWM again

Goal:
- If servo STILL jitters, it's likely power/ground/EMI/servo hardware (or PWM backend behavior),
  not your update loop / print timing.
- If jitter STOPS, your main code's repeated updates / logging / timing is contributing.

Run examples:
  python3 testA_servo_constant.py
  python3 testA_servo_constant.py --pin 18 --us 1500 --seconds 10
  python3 testA_servo_constant.py --pin 18 --us 1700 --seconds 15
"""

import argparse
import time

from gpiozero import Device, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi el
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pin", type=int, default=18, help="Servo signal GPIO (default: 18)")
    ap.add_argument("--us", type=float, default=1500.0, help="Pulse width in microseconds (default: 1500)")
    ap.add_argument("--freq", type=float, default=50.0, help="PWM frequency in Hz (default: 50)")
    ap.add_argument("--seconds", type=float, default=10.0, help="How long to hold the pulse (default: 10)")
    args = ap.parse_args()

    # Use lgpio backend (Pi 5-friendly, no pigpio daemon)
    Device.pin_factory = LGPIOFactory()

    period_us = 1_000_000.0 / float(args.freq)  # e.g., 50Hz => 20000us

    # Create PWM output
    pwm = PWMOutputDevice(args.pin, frequency=float(args.freq), initial_value=0.0)

    try:
        pw_us = float(args.us)
        duty = clamp(pw_us / period_us, 0.0, 1.0)

        print(f"[Test A] Setting once: pin={args.pin}, freq={args.freq}Hz, period={period_us:.1f}us")
        print(f"[Test A] Pulse width={pw_us:.1f}us => duty={duty:.6f}")
        print(f"[Test A] Holding for {args.seconds:.1f}s without updating PWM...")

        # SET ONCE
        pwm.value = duty

        # DO NOTHING ELSE
        time.sleep(float(args.seconds))

        print("[Test A] Done. Turning PWM off.")
    finally:
        try:
            pwm.off()
        finally:
            pwm.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
