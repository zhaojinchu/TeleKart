#!/usr/bin/env python3
"""Find this car's real steering endpoints by walking outward from centre.

**Nothing here is seeded from a previous firmware.** The old ESP32 build carried
1100 / 2000 / 2500 us in its source; those numbers describe a different linkage
on a different chassis and using them as a starting point is how a servo ends up
stalled against a stop it was never told about. The sweep starts at 1500 us --
the mechanical centre of a standard hobby servo and nothing more -- and stops
where *this* car says stop.

The procedure, from calibration.md section 2:

* Motor leads unscrewed, wheels off the ground, **steering linkage disconnected
  from the horn**. That last one is the procedure, not caution: you are going to
  command the servo into its limit, and a servo commanded into a bound linkage
  stalls at 700-800 mA on the same 5 V rail as the SoC.
* 25 us at a time, outward, half a second at each step.
* A continuous hum or buzz means you have reached the mechanical limit. Stop
  immediately -- ``s`` at the prompt, or Ctrl-C, both of which relax the servo
  before anything else happens.
* The endpoint is recorded ``--backoff`` microseconds back from the first sign
  of strain.

Run it again with the linkage connected: the rack's own travel limits are
usually tighter than the servo's, and the tighter pair is the one to keep.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

_PI_ROOT = Path(__file__).resolve().parents[1]
_PROTOCOL_ROOT = _PI_ROOT.parent / "packages" / "telekart_protocol"
for _candidate in (_PI_ROOT, _PROTOCOL_ROOT):
    if _candidate.is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from telekart.app import HardwarePanic, PanicChain  # noqa: E402
from telekart.config import ConfigError, VehicleConfig  # noqa: E402
from telekart.constants import (  # noqa: E402
    LOCAL_CONFIG_PATH,
    SERVO_ABS_MAX_US,
    SERVO_ABS_MIN_US,
    SERVO_NOMINAL_CENTER_US,
)
from telekart.hal.base import GpioBackend, GpioError, select_backend  # noqa: E402
from telekart.util.clock import RealClock  # noqa: E402

#: Where every sweep begins. Not a measurement and not inherited from anywhere:
#: it is the nominal centre of a hobby servo, and the sweep exists precisely to
#: replace it with something measured.
START_US = SERVO_NOMINAL_CENTER_US

#: Outward step. Small enough that the servo reaches each position and settles
#: before the next one, so a strain you hear is caused by the step you are on.
STEP_US = 25

#: How far the sweep is allowed to travel from centre before it stops on its
#: own. Half the HS-311's nominal 1000-2000 us range again, which is wider than
#: any linkage on this chassis and still 200 us inside the absolute limits.
DEFAULT_SPAN_US = 400

#: The midpoint of a good pair of endpoints lands within this of 1500. Further
#: out means the horn is a spline tooth off (15 degrees each) or the tie rod is
#: the wrong length -- both mechanical problems that a software offset only
#: hides, at the cost of losing lock in one direction.
CENTRE_TOLERANCE_US = 40


@dataclass(slots=True)
class Endpoint:
    """One direction's result."""

    label: str
    limit_us: int
    strained_at: int | None
    stopped_early: bool

    def describe(self) -> str:
        if self.strained_at is not None:
            return f"{self.label}: {self.limit_us} us (strain reported at {self.strained_at} us)"
        if self.stopped_early:
            return f"{self.label}: {self.limit_us} us (operator stopped, no strain)"
        return f"{self.label}: {self.limit_us} us (reached the sweep bound cleanly)"


class Aborted(Exception):
    """The operator asked to stop. The servo is relaxed before this propagates."""


class Servo:
    """Raw pulse access with a hard clamp and a guaranteed relax.

    Deliberately not :class:`~telekart.drivers.servo.SteeringServo`: that class
    clamps to the configured endpoints, and the configured endpoints are the
    thing this script exists to discover.
    """

    def __init__(self, gpio: GpioBackend, pin: int, dwell: float) -> None:
        self._gpio = gpio
        self._pin = pin
        self._dwell = dwell
        self._clock = RealClock()

    def write(self, pulse_us: int) -> int:
        if pulse_us < SERVO_ABS_MIN_US:
            pulse_us = SERVO_ABS_MIN_US
        elif pulse_us > SERVO_ABS_MAX_US:
            pulse_us = SERVO_ABS_MAX_US
        self._gpio.set_servo_pulse(self._pin, pulse_us)
        self._clock.sleep(self._dwell)
        return pulse_us

    def relax(self) -> None:
        """Stop the pulse train. The servo goes limp and stops drawing current,
        which is the only correct response to 'it is buzzing'."""
        self._gpio.set_servo_pulse(self._pin, 0)


# --------------------------------------------------------------------------
# Operator interaction
# --------------------------------------------------------------------------

_PROMPT = "  {pulse:>4} us   [Enter] next   s = stop here   a = abort  > "


def prompt_step(pulse_us: int) -> str:
    """Returns 'next', 'stop' or 'abort'."""
    try:
        answer = input(_PROMPT.format(pulse=pulse_us)).strip().lower()
    except EOFError:
        return "abort"
    except KeyboardInterrupt:
        print()
        return "abort"
    if answer in ("a", "abort", "q"):
        return "abort"
    if answer in ("s", "stop", "b", "back"):
        return "stop"
    return "next"


def ask(question: str) -> bool:
    try:
        return input(f"  {question} [y/N] ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def require_operator() -> None:
    if not sys.stdin.isatty():
        raise SystemExit(
            "servo_calibrate needs an operator watching and listening to the "
            "servo at every step; it refuses to run non-interactively"
        )


# --------------------------------------------------------------------------
# The sweep
# --------------------------------------------------------------------------


def sweep(servo: Servo, label: str, direction: int, span_us: int, backoff_us: int) -> Endpoint:
    """Walk outward from centre one step at a time until told to stop."""
    print()
    print(f"  --- {label} ---")
    print("      Listen. A continuous hum that does not stop means the servo is")
    print("      straining against its limit: press 's' immediately.")
    print()

    bound = START_US + direction * span_us
    last_clean = START_US
    strained_at: int | None = None
    stopped_early = False

    servo.write(START_US)
    pulse = START_US
    while True:
        pulse += direction * STEP_US
        if (direction > 0 and pulse > bound) or (direction < 0 and pulse < bound):
            print(f"      reached the {bound} us sweep bound with no strain reported")
            break
        written = servo.write(pulse)
        action = prompt_step(written)
        if action == "abort":
            servo.relax()
            raise Aborted(f"{label} aborted at {written} us")
        if action == "stop":
            strained_at = written
            stopped_early = True
            # Relax first, ask questions second. Whatever made the operator stop
            # is still happening while the pulse train continues.
            servo.relax()
            break
        last_clean = written

    if strained_at is None:
        limit = last_clean if last_clean != START_US else bound
    else:
        # Back off toward centre from the point where strain was reported.
        limit = strained_at - direction * backoff_us
        if (direction > 0 and limit < START_US) or (direction < 0 and limit > START_US):
            limit = START_US

    servo.write(START_US)
    return Endpoint(label, int(limit), strained_at, stopped_early)


def verify(servo: Servo, minimum: int, maximum: int, passes: int) -> bool:
    """Sweep between the recorded endpoints a few times and ask.

    The point is duration: a strain that is inaudible on one pass is obvious
    after five, and finding it here is much cheaper than finding it on the track.
    """
    print()
    print(f"  Verifying {minimum} us .. {maximum} us over {passes} passes.")
    print("  Watch for buzz at either extreme.")
    for index in range(passes):
        servo.write(minimum)
        servo.write(maximum)
        print(f"      pass {index + 1}/{passes}")
    servo.write((minimum + maximum) // 2)
    return ask("Did it move cleanly and stay quiet at both ends?")


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def report(left: Endpoint, right: Endpoint, centre: int) -> None:
    minimum, maximum = sorted((left.limit_us, right.limit_us))
    midpoint = (minimum + maximum) // 2
    offset = midpoint - START_US

    print()
    print("=" * 68)
    print("  Measured endpoints")
    print(f"    {left.describe()}")
    print(f"    {right.describe()}")
    print(f"    usable travel: {maximum - minimum} us, midpoint {midpoint} us")
    print()

    if abs(offset) > CENTRE_TOLERANCE_US:
        print(f"  WARNING: the midpoint is {offset:+d} us from 1500.")
        print("    The horn is probably a spline tooth out -- an HS-311 has 24 teeth,")
        print("    so 15 degrees per tooth, and you cannot get closer than +/-7.5 degrees")
        print("    by choosing a tooth. Fix it mechanically and re-run: a large software")
        print("    offset makes the travel asymmetric and costs you lock in one direction.")
        print()

    print("  Add to pi/config/config.local.yaml (git-ignored, per-vehicle):")
    print()
    print("    params:")
    print(f"      steer_min_us: {minimum}")
    print(f"      steer_max_us: {maximum}")
    print(f"      steer_center_us: {centre}")
    print()
    print("  steer_min_us stays numerically below steer_max_us. If that makes the")
    print("  car steer the wrong way once the linkage is on, set steer_invert: true")
    print("  -- do not swap the two numbers.")
    print()
    print("  Still to measure by hand (calibration.md section 2.5):")
    print("    steer_max_deg -- protractor against each front wheel face at full")
    print("    lock, then record the AVERAGE of the inner and outer angles. The")
    print("    two differ because Ackermann geometry is doing its job.")
    print("=" * 68)


def write_local(config: VehicleConfig, minimum: int, maximum: int, centre: int, path: Path) -> None:
    changed = config.apply_params(
        {"steer_min_us": minimum, "steer_max_us": maximum, "steer_center_us": centre}
    )
    config.save_local(path)
    print(f"  wrote {', '.join(changed) or 'no changes'} to {path}")


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Walk outward from 1500 us to find this car's real steering limits.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--sweep-safe",
        action="store_true",
        help="the stepped outward sweep with an operator prompt at every step",
    )
    parser.add_argument(
        "--center-only",
        action="store_true",
        help="hold 1500 us so the horn can be fitted, then exit (calibration.md 2.1)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="sweep between the endpoints already in the config and listen",
    )
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    parser.add_argument(
        "--span", type=int, default=DEFAULT_SPAN_US, help="how far from 1500 us to sweep"
    )
    parser.add_argument(
        "--backoff", type=int, default=50, help="microseconds to back off from the strain point"
    )
    parser.add_argument("--dwell", type=float, default=0.5, help="seconds to hold each step")
    parser.add_argument("--passes", type=int, default=5, help="verification sweeps")
    parser.add_argument(
        "--write",
        action="store_true",
        help="write the result into config.local.yaml as well as printing it",
    )
    return parser


def load_config(path: Path | None) -> VehicleConfig:
    if path is not None:
        return VehicleConfig.load(path, LOCAL_CONFIG_PATH)
    return VehicleConfig.load_default()


def preflight() -> bool:
    print()
    print("  " + "=" * 66)
    print("   Before you start:")
    print("     - motor leads UNSCREWED from OUT1-OUT4")
    print("     - wheels off the ground")
    print("     - STEERING LINKAGE DISCONNECTED from the servo horn")
    print()
    print("   You are about to command the servo into its mechanical limit. With")
    print("   a linkage attached that is a stall, at 700-800 mA, off the same 5 V")
    print("   rail as the SoC.")
    print("  " + "=" * 66)
    print()
    try:
        return input("  Type LINKAGE OFF to continue: ").strip().upper() == "LINKAGE OFF"
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def run_center_only(servo: Servo) -> int:
    print()
    print(f"  Holding {START_US} us. Fit the horn as close to straight ahead as the")
    print("  spline allows -- 24 teeth is 15 degrees each, so within a tooth is as")
    print("  good as it gets. Take up the rest with the tie rod and steer_trim_us.")
    servo.write(START_US)
    try:
        input("  Press Enter when the horn is on. ")
    except (EOFError, KeyboardInterrupt):
        print()
    servo.relax()
    print("  Servo relaxed. Next: --sweep-safe")
    return 0


def run_check(servo: Servo, config: VehicleConfig, passes: int) -> int:
    print()
    print(f"  Config says {config.steer_min_us} .. {config.steer_max_us} us, "
          f"centre {config.steer_center_us}.")
    quiet = verify(servo, config.steer_min_us, config.steer_max_us, passes)
    servo.relax()
    if quiet:
        print("  Endpoints confirmed.")
        return 0
    print("  Re-run --sweep-safe: an endpoint that buzzes is past the mechanical limit.")
    return 1


def run_sweep(servo: Servo, config: VehicleConfig, args: argparse.Namespace) -> int:
    span = max(STEP_US, min(int(args.span), SERVO_ABS_MAX_US - START_US))
    left = sweep(servo, "LEFT (decreasing)", -1, span, args.backoff)
    right = sweep(servo, "RIGHT (increasing)", +1, span, args.backoff)

    minimum, maximum = sorted((left.limit_us, right.limit_us))
    if maximum - minimum < 4 * STEP_US:
        servo.relax()
        print()
        print(f"  Only {maximum - minimum} us of travel was found. That is not a")
        print("  calibration, it is a servo that is not moving. Check the signal wire,")
        print("  the 5 V feed, and that the horn is not already against a stop.")
        return 1

    midpoint = (minimum + maximum) // 2
    centre = midpoint if abs(midpoint - START_US) > CENTRE_TOLERANCE_US else START_US

    quiet = verify(servo, minimum, maximum, args.passes)
    servo.relax()
    report(left, right, centre)
    if not quiet:
        print()
        print("  The verification pass was not clean. Re-run with a larger --backoff.")
        return 1

    if args.write:
        write_local(config, minimum, maximum, centre, LOCAL_CONFIG_PATH)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    modes = sum(bool(flag) for flag in (args.sweep_safe, args.center_only, args.check))
    if modes != 1:
        print(
            "choose exactly one of --sweep-safe, --center-only or --check",
            file=sys.stderr,
        )
        return 2

    require_operator()

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    if args.sweep_safe and not preflight():
        print("  Aborted. The servo was never driven.")
        return 2

    try:
        gpio = select_backend(args.backend)
    except GpioError as exc:
        print(f"cannot open the GPIO backend: {exc}", file=sys.stderr)
        return 2

    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    servo = Servo(gpio, config.pins.servo, args.dwell)
    status = 1
    with PanicChain(panic):
        try:
            if args.center_only:
                status = run_center_only(servo)
            elif args.check:
                status = run_check(servo, config, args.passes)
            else:
                status = run_sweep(servo, config, args)
        except Aborted as exc:
            print(f"\n  {exc} -- servo relaxed, nothing recorded")
            status = 2
        except KeyboardInterrupt:
            servo.relax()
            print("\n  interrupted -- servo relaxed")
            status = 2
        finally:
            servo.relax()
            gpio.cleanup()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
