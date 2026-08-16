"""Headless client: connect, arm, drive, dump telemetry -- no Qt, no wheel.

This is the tool you reach for when the GUI is misbehaving and you need to know
whether the fault is in the car or in the app. It drives the *same*
``LinkManager`` the desktop application uses, so a problem that reproduces here
is a networking or firmware problem, and one that does not is a UI problem.
That split is most of the diagnostic value.

It is also the bench toolkit. The scripted patterns are the ones you actually
want with the car on blocks: a throttle step for PID work, a slow ramp to find
the stiction threshold, a steering sweep to watch the electronic differential
engage, and a square for odometry closure.

Run against the simulator with no hardware at all::

    telekart-sim &
    python -m telekart_app.cli --host 127.0.0.1 --arm --drive step --seconds 6
"""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import signal
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Callable, Iterator

from telekart_protocol import ControlFlags, Fault, TelemetryFlags, VehicleState

from .config.settings import Settings, load_shared_key
from .model.snapshots import LinkState
from .net.link_manager import LinkManager

#: How long to keep waiting for the video stream once control and telemetry are
#: already up, before accepting a DEGRADED link and driving without a picture.
VIDEO_SETTLE_S = 2.0

# --------------------------------------------------------------------------
# Command
# --------------------------------------------------------------------------


@dataclass(slots=True)
class Command:
    """Satisfies ControlCommandLike. Mutable on purpose.

    The TX thread reads whatever is in the box at its own 100 Hz cadence, so
    this is written in place and re-published rather than reallocated per
    update -- the same discipline the input thread uses.
    """

    steering: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    flags: ControlFlags = ControlFlags.NONE


# --------------------------------------------------------------------------
# Drive patterns
# --------------------------------------------------------------------------

#: A pattern is a function of elapsed seconds returning (steer, throttle, brake).
Pattern = Callable[[float], "tuple[float, float, float]"]


def _pattern_idle(_t: float) -> tuple[float, float, float]:
    return 0.0, 0.0, 0.0


def _make_step(level: float, delay: float = 1.0) -> Pattern:
    """Zero, then a hard step to `level`. The classic closed-loop test.

    The leading delay is not cosmetic: it gives the arming sequence time to
    settle and leaves a clean pre-step baseline in the telemetry so rise time
    and overshoot can actually be read off the log.
    """

    def pattern(t: float) -> tuple[float, float, float]:
        return 0.0, (level if t >= delay else 0.0), 0.0

    return pattern


def _make_ramp(level: float, period: float) -> Pattern:
    """Slow linear throttle ramp -- how you find the stiction threshold.

    Watch for the duty at which the wheels first turn. On this drivetrain
    expect 0.20-0.28 rather than the ~0.12 a 12 V build would show, because the
    boost regulator leaves roughly 5 V at the motor terminals.
    """

    def pattern(t: float) -> tuple[float, float, float]:
        return 0.0, min(level, level * (t / period if period > 0 else 1.0)), 0.0

    return pattern


def _make_sweep(level: float, period: float) -> Pattern:
    """Steering sweep at constant throttle.

    Run this with the wheels off the ground and watch rpm_l and rpm_r diverge:
    that is the electronic differential doing its job. If they stay equal
    through a sweep, the mixer is not wired in and the tyres will scrub.
    """

    def pattern(t: float) -> tuple[float, float, float]:
        return math.sin(2.0 * math.pi * t / period), level, 0.0

    return pattern


def _make_square(level: float, leg: float) -> Pattern:
    """Straight, turn, repeat. Feeds the odometry closure check.

    Expect the loop not to close perfectly -- 5-15 % is normal without an IMU,
    and seeing the size of the error is the point of running it.
    """

    def pattern(t: float) -> tuple[float, float, float]:
        phase = t % (leg * 2.0)
        return (0.0, level, 0.0) if phase < leg else (1.0, level * 0.6, 0.0)

    return pattern


def build_pattern(name: str, level: float, period: float) -> Pattern:
    if name == "idle":
        return _pattern_idle
    if name == "step":
        return _make_step(level)
    if name == "ramp":
        return _make_ramp(level, period)
    if name == "sweep":
        return _make_sweep(level, period)
    if name == "square":
        return _make_square(level, period / 2.0)
    raise ValueError(f"unknown drive pattern {name!r}")


# --------------------------------------------------------------------------
# Keyboard
# --------------------------------------------------------------------------


@contextlib.contextmanager
def raw_terminal() -> Iterator[bool]:
    """Put stdin in cbreak mode, and restore it on every exit path.

    Restoration is wrapped in try/finally rather than left to atexit because a
    terminal left in raw mode after a crash is genuinely unpleasant to recover
    from, and this tool is one Ctrl-C away from that at all times. Yields False
    when stdin is not a tty, so piped invocations degrade instead of failing.
    """
    if not sys.stdin.isatty():
        yield False
        return
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield True
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


class KeyboardDriver:
    """WASD in a terminal, with decay so a missed key-up cannot stick the throttle.

    Terminals give key-*press* events only -- there is no key-up. So rather than
    latching a value until some imagined release, every axis decays toward zero
    and each keypress tops it back up. Stop typing and the car coasts to a stop
    within a few hundred milliseconds, which is the correct failure mode for a
    control input that can silently stop arriving.
    """

    RISE = 0.12
    DECAY = 0.90

    def __init__(self) -> None:
        self.steer = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self.estop = False
        self.quit = False
        self._lock = threading.Lock()

    def feed(self, key: str) -> None:
        with self._lock:
            k = key.lower()
            if k == "w":
                self.throttle = min(1.0, self.throttle + self.RISE * 2.0)
            elif k == "s":
                self.brake = min(1.0, self.brake + self.RISE * 3.0)
            elif k == "a":
                self.steer = max(-1.0, self.steer - self.RISE * 2.0)
            elif k == "d":
                self.steer = min(1.0, self.steer + self.RISE * 2.0)
            elif k == " ":
                self.estop = True
            elif k in ("q", "\x03", "\x04"):  # q, Ctrl-C, Ctrl-D
                self.quit = True

    def sample(self) -> tuple[float, float, float]:
        with self._lock:
            self.throttle *= self.DECAY
            self.brake *= self.DECAY
            self.steer *= self.DECAY
            for name in ("throttle", "brake"):
                if getattr(self, name) < 0.01:
                    setattr(self, name, 0.0)
            if abs(self.steer) < 0.01:
                self.steer = 0.0
            return self.steer, self.throttle, self.brake

    def reader(self, stop: threading.Event) -> None:
        while not stop.is_set():
            try:
                ch = sys.stdin.read(1)
            except (OSError, ValueError):
                return
            if not ch:
                return
            self.feed(ch)


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def _fault_names(faults: Fault) -> str:
    if not faults:
        return "none"
    return "|".join(f.name for f in Fault if f and (faults & f))


def _flag_names(flags: TelemetryFlags) -> str:
    if not flags:
        return "-"
    return "|".join(f.name for f in TelemetryFlags if f and (flags & f))


def print_status(link: LinkManager, quiet: bool, as_json: bool) -> None:
    sample = link.telemetry_box.peek()
    snap = link.link_box.peek()
    if sample is None:
        return
    pkt = sample.packet

    if as_json:
        sys.stdout.write(
            json.dumps(
                {
                    "t": round(pkt.car_time_us / 1e6, 3),
                    "state": pkt.state.name,
                    "speed": round(pkt.speed_mps, 3),
                    "v_max": round(pkt.v_max_mps, 3),
                    "rpm": [pkt.rpm_l, pkt.rpm_r],
                    "rpm_target": [pkt.rpm_target_l, pkt.rpm_target_r],
                    "duty": [round(pkt.duty_l_f, 3), round(pkt.duty_r_f, 3)],
                    "servo_us": pkt.servo_us,
                    "steer_deg": round(pkt.steer_angle_deg, 2),
                    "pose": [round(v, 3) for v in pkt.pose_m],
                    "distance": round(pkt.distance_m, 3),
                    "slip": round(pkt.slip, 4),
                    "pack_v": round(pkt.pack_volts, 2),
                    "rtt_ms": round(sample.rtt * 1000.0, 1),
                    "faults": _fault_names(pkt.faults),
                    "flags": _flag_names(pkt.flags),
                },
                separators=(",", ":"),
            )
            + "\n"
        )
        sys.stdout.flush()
        return

    if quiet:
        return

    rtt_ms = sample.rtt * 1000.0
    age = snap.telemetry_age if snap is not None else 0.0
    line = (
        f"{pkt.state.name:<8} "
        f"{pkt.speed_mps:5.2f}/{pkt.v_max_mps:4.2f} m/s  "
        f"rpm {pkt.rpm_l:6.1f}/{pkt.rpm_r:6.1f} "
        f"(tgt {pkt.rpm_target_l:6.1f}/{pkt.rpm_target_r:6.1f})  "
        f"duty {pkt.duty_l_f:+.2f}/{pkt.duty_r_f:+.2f}  "
        f"servo {pkt.servo_us:4d}us  "
        f"d {pkt.distance_m:6.2f}m  "
        f"rtt {rtt_ms:5.1f}ms age {age * 1000:4.0f}ms  "
        f"{_fault_names(pkt.faults)}"
    )
    # Overwrite one line on a terminal, but emit discrete lines when piped --
    # a redirected carriage-return status becomes one unreadable mega-line, and
    # this output wants to be greppable and diffable in a log.
    if sys.stdout.isatty():
        sys.stdout.write("\r" + line.ljust(170)[:170])
    else:
        sys.stdout.write(line + "\n")
    sys.stdout.flush()


def print_summary(link: LinkManager) -> None:
    sample = link.telemetry_box.peek()
    snap = link.link_box.peek()
    print()
    if sample is None:
        print("no telemetry was received")
        return
    pkt = sample.packet
    x, y, heading = pkt.pose_m
    print("--- final ---")
    print(f"  state        {pkt.state.name}")
    print(f"  faults       {_fault_names(pkt.faults)}")
    print(f"  flags        {_flag_names(pkt.flags)}")
    print(f"  speed        {pkt.speed_mps:.3f} m/s   (measured ceiling {pkt.v_max_mps:.3f})")
    print(f"  wheel rpm    L {pkt.rpm_l:.1f}   R {pkt.rpm_r:.1f}")
    print(f"  pose         x {x:.3f} m  y {y:.3f} m  heading {math.degrees(heading):.1f} deg")
    print(f"  distance     {pkt.distance_m:.3f} m")
    print(f"  slip index   {pkt.slip:.4f}")
    if pkt.pack_mv:
        print(f"  pack         {pkt.pack_volts:.2f} V")
    if snap is not None:
        print(f"  rtt          {snap.rtt * 1000:.1f} ms  (p95 {snap.rtt_p95 * 1000:.1f} ms)")
    if pkt.throttled:
        print(f"  throttled    0x{pkt.throttled:X}  <-- Pi undervoltage/throttling, check the 5 V rail")


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="telekart-cli",
        description=(
            "Headless TeleKart client. Drives the same LinkManager as the desktop "
            "app, so a fault that reproduces here is in the car or the link, and "
            "one that does not is in the UI."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    conn = p.add_argument_group("connection")
    conn.add_argument("--host", default="", help="car address; empty discovers via mDNS")
    conn.add_argument("--key", default="", help="shared key (else the saved one for --car-id)")
    conn.add_argument("--car-id", default="", help="car id, for looking up a saved key")
    conn.add_argument("--driver", default=os.environ.get("USER", "cli"))
    conn.add_argument("--connect-timeout", type=float, default=10.0, metavar="S")

    drive = p.add_argument_group("driving")
    drive.add_argument("--arm", action="store_true", help="arm before driving")
    drive.add_argument(
        "--drive",
        default="idle",
        choices=("idle", "step", "ramp", "sweep", "square"),
        help="scripted pattern",
    )
    drive.add_argument("--keys", action="store_true", help="interactive WASD (overrides --drive)")
    drive.add_argument("--level", type=float, default=0.5, metavar="0..1",
                       help="throttle level used by the pattern")
    drive.add_argument("--period", type=float, default=4.0, metavar="S",
                       help="pattern period (ramp/sweep/square)")
    drive.add_argument("--seconds", type=float, default=10.0, metavar="S",
                       help="run duration; 0 runs until interrupted")

    out = p.add_argument_group("output")
    out.add_argument("--json", action="store_true", help="line-delimited JSON telemetry")
    out.add_argument("--quiet", "-q", action="store_true", help="suppress the live line")
    out.add_argument("--rate", type=float, default=10.0, metavar="HZ",
                     help="status print rate")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if not 0.0 <= args.level <= 1.0:
        print("--level must be between 0 and 1", file=sys.stderr)
        return 2

    settings = Settings.load()
    if args.host:
        settings.link.host = args.host
    if args.car_id:
        settings.car_id = args.car_id

    key = args.key or load_shared_key(settings.car_id or "telekart")

    link = LinkManager(settings)
    command = Command()
    stop = threading.Event()

    # SIGINT must reach the cleanup path rather than unwinding through the
    # driving loop, or the car keeps the last command it was given.
    def handle_signal(_sig: int, _frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    keyboard = KeyboardDriver() if args.keys else None
    pattern = build_pattern(args.drive, args.level, args.period)

    try:
        target = args.host or "(discovering)"
        print(f"connecting to {target} ...", file=sys.stderr)
        link.connect(args.host, shared_key=key, driver=args.driver)

        # Hold out for LIVE for a moment before settling for DEGRADED. The video
        # stream is a second TCP connection that lands a few hundred
        # milliseconds after the session handshake, so a client that accepts the
        # first usable state it sees will almost always report "video is down"
        # about a link that is about to be perfectly healthy.
        deadline = time.monotonic() + args.connect_timeout
        degraded_deadline = 0.0
        while time.monotonic() < deadline:
            state = link.state
            if state is LinkState.LIVE:
                break
            if state is LinkState.DEGRADED:
                if degraded_deadline == 0.0:
                    degraded_deadline = time.monotonic() + VIDEO_SETTLE_S
                elif time.monotonic() >= degraded_deadline:
                    break
            else:
                degraded_deadline = 0.0
            if state is LinkState.FAILED:
                print(f"connection failed: {link.last_error}", file=sys.stderr)
                return 1
            if stop.is_set():
                return 130
            time.sleep(0.05)
        else:
            print(f"timed out connecting (state {link.state.value})", file=sys.stderr)
            return 1

        info = link.session_info
        if info is not None:
            print(f"connected: car={info.car_id} fw={info.fw_version}", file=sys.stderr)
        if link.state is LinkState.DEGRADED:
            print("note: video is down; driving is still permitted", file=sys.stderr)

        if args.arm:
            link.arm()
            armed_by = time.monotonic() + 3.0
            while time.monotonic() < armed_by:
                sample = link.telemetry_box.peek()
                if sample is not None and sample.packet.state is VehicleState.ARMED:
                    break
                time.sleep(0.05)
            sample = link.telemetry_box.peek()
            state = sample.packet.state.name if sample else "unknown"
            if state != "ARMED":
                print(f"warning: car did not arm (state {state})", file=sys.stderr)
            else:
                print("armed", file=sys.stderr)

        with raw_terminal() as interactive:
            reader: threading.Thread | None = None
            if keyboard is not None:
                if not interactive:
                    print("--keys needs a tty; falling back to idle", file=sys.stderr)
                    keyboard = None
                else:
                    print("WASD to drive, space = E-STOP, q to quit", file=sys.stderr)
                    reader = threading.Thread(
                        target=keyboard.reader, args=(stop,), daemon=True, name="cli-keys"
                    )
                    reader.start()

            started = time.monotonic()
            next_print = started
            print_interval = 1.0 / args.rate if args.rate > 0 else 0.0

            while not stop.is_set():
                now = time.monotonic()
                elapsed = now - started
                if args.seconds > 0 and elapsed >= args.seconds:
                    break

                if keyboard is not None:
                    steer, throttle, brake = keyboard.sample()
                    if keyboard.quit:
                        break
                    if keyboard.estop:
                        command.flags |= ControlFlags.ESTOP
                        link.estop()
                        keyboard.estop = False
                else:
                    steer, throttle, brake = pattern(elapsed)

                command.steering = steer
                command.throttle = throttle
                command.brake = brake
                if args.arm:
                    command.flags |= ControlFlags.ARM_INTENT
                link.command_box.put(command)

                if print_interval and now >= next_print:
                    next_print = now + print_interval
                    print_status(link, args.quiet, args.json)

                time.sleep(0.005)

        return 0

    finally:
        # Order matters. Zero the command first so the last thing the car
        # received is neutral, give the TX thread a couple of ticks to actually
        # put it on the wire, and only then tear the link down. Closing first
        # would leave the car holding whatever it had until its own failsafe
        # expires -- which works, but deliberately relying on the failsafe for
        # an ordinary exit is how you stop noticing when it breaks.
        command.steering = 0.0
        command.throttle = 0.0
        command.brake = 0.0
        command.flags = ControlFlags.NONE
        with contextlib.suppress(Exception):
            link.command_box.put(command)
            time.sleep(0.05)
        with contextlib.suppress(Exception):
            if args.arm:
                link.disarm()
                time.sleep(0.1)
        if not args.json:
            with contextlib.suppress(Exception):
                print_summary(link)
        with contextlib.suppress(Exception):
            link.close()


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
