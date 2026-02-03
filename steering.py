#!/usr/bin/env python3
"""
steering.py — RC steering servo controller for Raspberry Pi 5
Uses gpiozero with the lgpio pin factory (NO pigpio daemon needed).

Input is normalized steering in [-1.0, +1.0]:
  -1 = full left, 0 = center, +1 = full right

Modes:
  --mode keyboard : arrow keys / A-D for testing
  --mode stdin    : read floats from stdin (one per line)
  --mode udp      : read ascii float packets on UDP
"""

import argparse
import time
import sys
import socket
import select
import termios
import tty

from gpiozero import Device, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory


class SteeringPWM:
    def __init__(self, pin: int, left_us: int, center_us: int, right_us: int,
                 hz: float = 100.0, timeout_s: float = 0.75, max_rate_us_per_s: float = 2500.0):
        if not (left_us < center_us < right_us):
            raise ValueError("Calibration must satisfy left_us < center_us < right_us")

        self.pin = pin
        self.left_us = int(left_us)
        self.center_us = int(center_us)
        self.right_us = int(right_us)

        self.loop_hz = float(hz)
        self.timeout_s = float(timeout_s)
        self.max_rate = float(max_rate_us_per_s)

        # Servo PWM is 50 Hz (20 ms period)
        self.servo_freq = 50.0
        self.period_us = 20000.0

        self.pwm = PWMOutputDevice(self.pin, frequency=self.servo_freq, initial_value=0.0)

        self._target_us = float(self.center_us)
        self._current_us = float(self.center_us)
        self._last_update = time.time()

        self.last_cmd = time.time()
        self._apply_us(self.center_us)

    def _apply_us(self, pw_us: float):
        # deadband: don't touch PWM unless meaningful change
        if self._last_applied_us is not None and abs(pw_us - self._last_applied_us) < 1.0:
            return
        self._last_applied_us = float(pw_us)

        duty = max(0.0, min(1.0, float(pw_us) / self.period_us))
        self.pwm.value = duty

    def set_norm(self, x: float):
        if x != x:  # NaN
            return
        x = max(-1.0, min(1.0, x))
        if x >= 0:
            self._target_us = self.center_us + x * (self.right_us - self.center_us)
        else:
            self._target_us = self.center_us + x * (self.center_us - self.left_us)
        self.last_cmd = time.time()

    def center(self):
        self._target_us = float(self.center_us)

    def update(self):
        now = time.time()
        dt = max(1e-4, now - self._last_update)
        self._last_update = now

        # Failsafe
        if (now - self.last_cmd) > self.timeout_s:
            self.center()

        # Rate limit
        max_step = self.max_rate * dt
        delta = self._target_us - self._current_us
        if abs(delta) > max_step:
            self._current_us += max_step if delta > 0 else -max_step
        else:
            self._current_us = self._target_us

        self._apply_us(self._current_us)
        print("steering angle", self._current_us, "target angle", self._target_us)

    def stop(self):
        self.pwm.off()
        self.pwm.close()


def run_stdin(steer: SteeringPWM):
    period = 1.0 / steer.loop_hz
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if r:
            line = sys.stdin.readline()
            if not line:
                break
            try:
                steer.set_norm(float(line.strip()))
            except ValueError:
                pass

        steer.update()
        time.sleep(period)


def run_udp(steer: SteeringPWM, host: str, port: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.setblocking(False)

    period = 1.0 / steer.loop_hz
    while True:
        try:
            data, _ = sock.recvfrom(256)
            if data:
                try:
                    steer.set_norm(float(data.decode("utf-8", errors="ignore").strip()))
                except ValueError:
                    pass
        except BlockingIOError:
            pass

        steer.update()
        time.sleep(period)


def run_keyboard(steer: SteeringPWM, step: float):
    period = 1.0 / steer.loop_hz
    steer_val = 0.0

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def read_key():
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":  # ESC seq
            if select.select([sys.stdin], [], [], 0.0)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == "[" and select.select([sys.stdin], [], [], 0.0)[0]:
                    ch3 = sys.stdin.read(1)
                    return f"ESC[{ch3}"
        return ch

    try:
        print("Keyboard steering:")
        print("  ←/A = left   →/D = right   SPACE = center   Q = quit")
        while True:
            k = read_key()
            if k:
                if k in ("q", "Q"):
                    break
                elif k == " ":
                    steer_val = 0.0
                elif k in ("a", "A", "ESC[D"):
                    steer_val -= step
                elif k in ("d", "D", "ESC[C"):
                    steer_val += step

                steer_val = max(-1.0, min(1.0, steer_val))
                steer.set_norm(steer_val)

            steer.update()
            time.sleep(period)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pin", type=int, default=18, help="Servo signal GPIO (default: 18).")
    ap.add_argument("--left-us", type=int, default=1100)
    ap.add_argument("--center-us", type=int, default=1500)
    ap.add_argument("--right-us", type=int, default=1900)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--timeout", type=float, default=0.75)
    ap.add_argument("--max-rate", type=float, default=2500.0)
    ap.add_argument("--mode", choices=["keyboard", "stdin", "udp"], default="keyboard")
    ap.add_argument("--key-step", type=float, default=0.08)
    ap.add_argument("--udp-host", type=str, default="0.0.0.0")
    ap.add_argument("--udp-port", type=int, default=9999)
    args = ap.parse_args()

    # Force lgpio backend
    Device.pin_factory = LGPIOFactory()

    steer = SteeringPWM(
        pin=args.pin,
        left_us=args.left_us,
        center_us=args.center_us,
        right_us=args.right_us,
        hz=args.hz,
        timeout_s=args.timeout,
        max_rate_us_per_s=args.max_rate,
    )

    try:
        if args.mode == "keyboard":
            run_keyboard(steer, step=args.key_step)
        elif args.mode == "stdin":
            run_stdin(steer)
        else:
            run_udp(steer, host=args.udp_host, port=args.udp_port)
    except KeyboardInterrupt:
        pass
    finally:
        steer.stop()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
