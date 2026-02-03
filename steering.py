#!/usr/bin/env python3
"""
steering.py — Low-jitter RC steering for Raspberry Pi 5 using lgpio hardware PWM.

Input is normalized steering in [-1.0, +1.0]:
  -1 = full left, 0 = center, +1 = full right

Modes:
  --mode keyboard : arrow keys / A-D for testing
  --mode stdin    : read floats from stdin (one per line)
  --mode udp      : read ascii float packets on UDP
"""

import argparse
import socket
import select
import sys
import termios
import time
import tty
from contextlib import suppress

try:
    import lgpio
except ImportError as exc:
    lgpio = None
    _LGPIO_IMPORT_ERROR = exc
else:
    _LGPIO_IMPORT_ERROR = None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def release_gpio_if_busy(pin: int, chip: int, verbose: bool) -> None:
    if lgpio is None:
        if verbose:
            print(f"lgpio module not available: {_LGPIO_IMPORT_ERROR}", file=sys.stderr)
        return
    try:
        handle = lgpio.gpiochip_open(chip)
    except lgpio.error as exc:
        if verbose:
            print(f"Failed to open gpiochip {chip}: {exc}", file=sys.stderr)
        return
    try:
        with suppress(lgpio.error):
            lgpio.gpio_free(handle, pin)
            if verbose:
                print(f"Released GPIO {pin} on chip {chip}.", file=sys.stderr)
    finally:
        lgpio.gpiochip_close(handle)


class SteeringPWM:
    def __init__(
        self,
        pin: int,
        left_us: int,
        center_us: int,
        right_us: int,
        hz: float = 200.0,
        timeout_s: float = 0.5,
        max_rate_us_per_s: float = 4000.0,
        deadband_us: float = 2.0,
        verbose: bool = False,
        chip: int = 0,
        force_release: bool = False,
    ):
        if lgpio is None:
            raise RuntimeError(f"lgpio module not available: {_LGPIO_IMPORT_ERROR}")
        if not (left_us < center_us < right_us):
            raise ValueError("Calibration must satisfy left_us < center_us < right_us")

        self.pin = int(pin)
        self.left_us = int(left_us)
        self.center_us = int(center_us)
        self.right_us = int(right_us)
        self.loop_hz = float(hz)
        self.timeout_s = float(timeout_s)
        self.max_rate = float(max_rate_us_per_s)
        self.deadband_us = float(deadband_us)
        self.verbose = bool(verbose)
        self.chip = int(chip)

        self.servo_freq = 50.0
        self.period_us = 20000.0

        if force_release:
            release_gpio_if_busy(self.pin, self.chip, self.verbose)

        self.handle = lgpio.gpiochip_open(self.chip)
        try:
            lgpio.gpio_claim_output(self.handle, self.pin, 0)
        except lgpio.error as exc:
            lgpio.gpiochip_close(self.handle)
            raise RuntimeError(
                f"GPIO {self.pin} is busy or unavailable. Try --force-release or choose another pin."
            ) from exc

        self._target_us = float(self.center_us)
        self._current_us = float(self.center_us)
        self._last_update = time.monotonic()
        self._last_applied_us = None
        self.last_cmd = time.monotonic()
        self._apply_us(self.center_us)

    def _apply_us(self, pw_us: float) -> None:
        if self._last_applied_us is not None and abs(pw_us - self._last_applied_us) < self.deadband_us:
            return
        self._last_applied_us = float(pw_us)
        duty = clamp(float(pw_us) / self.period_us, 0.0, 1.0)
        lgpio.tx_pwm(self.handle, self.pin, self.servo_freq, duty * 100.0)

    def set_norm(self, value: float) -> None:
        if value != value:
            return
        value = clamp(value, -1.0, 1.0)
        if value >= 0:
            self._target_us = self.center_us + value * (self.right_us - self.center_us)
        else:
            self._target_us = self.center_us + value * (self.center_us - self.left_us)
        self.last_cmd = time.monotonic()

    def center(self) -> None:
        self._target_us = float(self.center_us)

    def update(self) -> None:
        now = time.monotonic()
        dt = max(1e-4, now - self._last_update)
        self._last_update = now

        if (now - self.last_cmd) > self.timeout_s:
            self.center()

        max_step = self.max_rate * dt
        delta = self._target_us - self._current_us
        if abs(delta) > max_step:
            self._current_us += max_step if delta > 0 else -max_step
        else:
            self._current_us = self._target_us

        self._apply_us(self._current_us)
        if self.verbose:
            print(f"steer_us={self._current_us:.1f} target_us={self._target_us:.1f}")

    def stop(self) -> None:
        with suppress(lgpio.error):
            lgpio.tx_pwm(self.handle, self.pin, 0, 0)
            lgpio.gpio_free(self.handle, self.pin)
        lgpio.gpiochip_close(self.handle)


def run_stdin(steer: SteeringPWM) -> None:
    period = 1.0 / steer.loop_hz
    next_tick = time.monotonic()
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
        next_tick += period
        time.sleep(max(0.0, next_tick - time.monotonic()))


def run_udp(steer: SteeringPWM, host: str, port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.setblocking(False)

    period = 1.0 / steer.loop_hz
    next_tick = time.monotonic()
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
        next_tick += period
        time.sleep(max(0.0, next_tick - time.monotonic()))


def run_keyboard(steer: SteeringPWM, step: float) -> None:
    period = 1.0 / steer.loop_hz
    steer_val = 0.0
    next_tick = time.monotonic()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def read_key():
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return None
        ch = sys.stdin.read(1)
        if ch == "\x1b":
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
                if k == " ":
                    steer_val = 0.0
                elif k in ("a", "A", "ESC[D"):
                    steer_val -= step
                elif k in ("d", "D", "ESC[C"):
                    steer_val += step

                steer_val = clamp(steer_val, -1.0, 1.0)
                steer.set_norm(steer_val)

            steer.update()
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pin", type=int, default=18, help="Servo signal GPIO (PWM-capable: 12/13/18/19).")
    ap.add_argument("--left-us", type=int, default=1100)
    ap.add_argument("--center-us", type=int, default=1500)
    ap.add_argument("--right-us", type=int, default=1900)
    ap.add_argument("--hz", type=float, default=200.0, help="Control loop rate (Hz).")
    ap.add_argument("--timeout", type=float, default=0.5)
    ap.add_argument("--max-rate", type=float, default=4000.0)
    ap.add_argument("--deadband-us", type=float, default=2.0)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--mode", choices=["keyboard", "stdin", "udp"], default="keyboard")
    ap.add_argument("--key-step", type=float, default=0.08)
    ap.add_argument("--udp-host", type=str, default="0.0.0.0")
    ap.add_argument("--udp-port", type=int, default=9999)
    ap.add_argument("--lgpio-chip", type=int, default=0)
    ap.add_argument("--force-release", action="store_true",
                    help="Force-release GPIO before claiming it (use if you see 'GPIO busy').")
    args = ap.parse_args()

    steer = SteeringPWM(
        pin=args.pin,
        left_us=args.left_us,
        center_us=args.center_us,
        right_us=args.right_us,
        hz=args.hz,
        timeout_s=args.timeout,
        max_rate_us_per_s=args.max_rate,
        deadband_us=args.deadband_us,
        verbose=args.verbose,
        chip=args.lgpio_chip,
        force_release=args.force_release,
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
