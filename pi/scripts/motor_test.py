#!/usr/bin/env python3
"""Bring-up phase 4: the first phase that draws real current.

Safety glasses. Wheels off the ground. Hand on the master switch.

Four things get established here, and each one is cheaper to find out now than
in any later phase:

**Which way is forward.** There is no sensor for this. The operator watches and
answers, and the answer becomes ``invert_left`` / ``invert_right``.

**Where the wheel breaks away.** The L298N is a Darlington bridge with ~1.4 V of
fixed drop, and 1.4 V out of a 9 V rail is 0.16 of duty before any current flows
at all. Stiction accounts for the rest, and the total lands near 0.22. Anything
below 0.05 means the RPM threshold is firing on noise; anything above 0.30 means
something is binding.

**That brake and coast are the right way round.** ``IN1 == IN2`` with the enable
up is a BRAKE. Coast requires the enable LOW. The ``--truth-table`` mode spins a
wheel up and releases it both ways with a stopwatch on the encoder, so the
answer is a number rather than an impression.

**Whether the regulator holds.** The S18V20ALV sustains about 1.5 A for both
motors *combined*, and it is simultaneous demand that trips it. ``--both`` ramps
the pair together and watches for the signature: both wheels losing speed at the
same instant, which is a supply collapse and not a mechanical stall.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

_PI_ROOT = Path(__file__).resolve().parents[1]
_PROTOCOL_ROOT = _PI_ROOT.parent / "packages" / "telekart_protocol"
for _candidate in (_PI_ROOT, _PROTOCOL_ROOT):
    if _candidate.is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from telekart.app import HardwarePanic, PanicChain  # noqa: E402
from telekart.config import ConfigError, VehicleConfig  # noqa: E402
from telekart.constants import CONTROL_PERIOD_S, LOCAL_CONFIG_PATH  # noqa: E402
from telekart.drivers.encoder import QuadratureEncoder  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.hal.base import GpioError, select_backend  # noqa: E402
from telekart.util.clock import RealClock  # noqa: E402

#: Acceptance band for the break-away duty, from calibration.md section 4.3.
DEADBAND_MIN = 0.05
DEADBAND_MAX = 0.30

#: The four max_rpm figures must land within this of each other or one corner
#: has a mechanical problem, or one bridge channel is weaker than the other.
WHEEL_SPREAD_LIMIT = 0.20

#: A wheel that loses this much speed while the duty is unchanged has either
#: stalled or lost its supply. Which one it is depends on whether the other
#: wheel did it at the same moment.
COLLAPSE_FRACTION = 0.50


@dataclass(slots=True)
class Step:
    duty: float
    rpm: float


@dataclass(slots=True)
class RampResult:
    wheel: str
    direction: str
    steps: list[Step] = field(default_factory=list)
    deadband: float = 0.0
    max_rpm: float = 0.0
    collapsed_at: float | None = None

    @property
    def key(self) -> str:
        return f"{self.wheel}_{'fwd' if self.direction == 'forward' else 'rev'}"


class Bench:
    """Both motors and both encoders, paced at control rate.

    Uses the real :class:`MotorPair` rather than raw pin writes so that what is
    measured here is what the control loop will actually apply -- inversion, the
    duty ceiling, the combined budget and the dead-time sequencing included.
    """

    def __init__(self, gpio: object, config: VehicleConfig, clock: RealClock) -> None:
        self.gpio = gpio
        self.config = config
        self.clock = clock
        self.motors = MotorPair(gpio, config.pins.motors, config, clock)
        self.encoder_l = QuadratureEncoder(
            gpio,
            config.pins.encoders.left_a,
            config.pins.encoders.left_b,
            cpr=config.encoder_cpr,
            invert=config.encoder_invert_left,
        )
        self.encoder_r = QuadratureEncoder(
            gpio,
            config.pins.encoders.right_a,
            config.pins.encoders.right_b,
            cpr=config.encoder_cpr,
            invert=config.encoder_invert_right,
        )
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        # Present only on MockBackend, so `--backend mock` is a genuine dry run
        # of this script rather than a run with nothing turning.
        self.pump = getattr(gpio, "step", None)

    def sample(self, dt: float) -> tuple[float, float]:
        """One tick of measurement. Returns the magnitude of each wheel's RPM.

        Exactly one ``sample()`` per encoder per tick: the call consumes the
        edges accumulated since the last one, so calling it twice hands the
        second caller an empty window and a velocity of zero.
        """
        if self.pump is not None:
            self.pump(dt)
        self.rpm_l = self.encoder_l.sample(dt).rpm
        self.rpm_r = self.encoder_r.sample(dt).rpm
        return (abs(self.rpm_l), abs(self.rpm_r))

    def hold(self, duty_l: float, duty_r: float, seconds: float) -> tuple[float, float]:
        """Command a duty for ``seconds``, returning the settled RPM of each wheel."""
        deadline = self.clock.monotonic() + seconds
        dt = CONTROL_PERIOD_S
        while self.clock.monotonic() < deadline:
            self.sample(dt)
            self.motors.note_speed(self.rpm_l, self.rpm_r)
            self.encoder_l.set_direction_hint(_sign(duty_l))
            self.encoder_r.set_direction_hint(_sign(duty_r))
            self.motors.drive(duty_l, duty_r)
            self.clock.sleep(dt)
        return (abs(self.rpm_l), abs(self.rpm_r))

    def coast(self, seconds: float = 0.0) -> None:
        self.motors.coast()
        if seconds > 0.0:
            self.settle(seconds)

    def brake(self, strength: float, seconds: float = 0.0) -> None:
        self.motors.brake(strength)
        if seconds > 0.0:
            self.settle(seconds)

    def settle(self, seconds: float) -> None:
        """Keep sampling while nothing is commanded, so the RPM estimate stays
        current during a spin-down."""
        deadline = self.clock.monotonic() + seconds
        dt = CONTROL_PERIOD_S
        while self.clock.monotonic() < deadline:
            self.sample(dt)
            self.clock.sleep(dt)

    def close(self) -> None:
        self.motors.coast()
        self.encoder_l.close()
        self.encoder_r.close()


def _sign(value: float) -> int:
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


# --------------------------------------------------------------------------
# Ramp
# --------------------------------------------------------------------------


def ramp_one(
    bench: Bench, wheel: str, direction: str, args: argparse.Namespace, threshold: float
) -> RampResult:
    """Walk the duty up on one wheel and record where it starts and how fast it ends."""
    result = RampResult(wheel, direction)
    sign = 1.0 if direction == "forward" else -1.0
    left = wheel == "left"
    peak = 0.0

    print(f"  --- {wheel} {direction} ---")
    duty = args.start
    while duty <= args.max_duty + 1e-9:
        command = duty * sign
        rpm_l, rpm_r = bench.hold(
            command if left else 0.0, 0.0 if left else command, args.dwell
        )
        rpm = rpm_l if left else rpm_r
        result.steps.append(Step(duty, rpm))
        print(f"      duty {duty:5.2f}   {rpm:7.1f} RPM")

        if result.deadband == 0.0 and rpm > threshold:
            result.deadband = duty
            print(f"      break-away at duty {duty:.2f}")

        if rpm > peak:
            peak = rpm
        elif peak > threshold and rpm < peak * COLLAPSE_FRACTION and result.collapsed_at is None:
            result.collapsed_at = duty
            print(f"      SPEED COLLAPSE at duty {duty:.2f}: {peak:.1f} -> {rpm:.1f} RPM")

        duty += args.step

    result.max_rpm = peak
    bench.coast(args.settle)
    print()
    return result


def run_ramp(bench: Bench, args: argparse.Namespace, config: VehicleConfig) -> tuple[list[RampResult], list[str]]:
    results: list[RampResult] = []
    failures: list[str] = []
    threshold = config.stall_rpm_threshold

    wheels = ("left", "right") if args.wheel == "both" else (args.wheel,)
    for wheel in wheels:
        for direction in ("forward", "reverse"):
            results.append(ramp_one(bench, wheel, direction, args, threshold))

    print("=" * 72)
    print("  Break-away duty and top speed, one wheel at a time")
    for result in results:
        note = ""
        if result.deadband == 0.0:
            note = "  NEVER TURNED"
        elif not DEADBAND_MIN <= result.deadband <= DEADBAND_MAX:
            note = "  OUT OF BAND"
        print(
            f"    {result.key:>10}: deadband {result.deadband:5.2f}   "
            f"max {result.max_rpm:6.1f} RPM{note}"
        )

    for result in results:
        if result.deadband == 0.0:
            failures.append(
                f"{result.key}: never exceeded {threshold:.1f} RPM at any duty up to "
                f"{args.max_duty:.2f}. Check the motor leads, the pack, and that the "
                "wheel is free to turn"
            )
        elif result.deadband < DEADBAND_MIN:
            failures.append(
                f"{result.key}: break-away at {result.deadband:.2f} is below "
                f"{DEADBAND_MIN:.2f} -- stall_rpm_threshold is firing on encoder noise"
            )
        elif result.deadband > DEADBAND_MAX:
            failures.append(
                f"{result.key}: break-away at {result.deadband:.2f} is above "
                f"{DEADBAND_MAX:.2f} -- something is binding on that corner"
            )
        if result.collapsed_at is not None:
            failures.append(
                f"{result.key}: speed collapsed at duty {result.collapsed_at:.2f} with "
                "only one motor running. A single motor cannot exceed the 1.5 A budget, "
                "so this is a mechanical stall or a tired pack, not the regulator"
            )

    speeds = [result.max_rpm for result in results if result.max_rpm > 0.0]
    if len(speeds) >= 2:
        spread = (max(speeds) - min(speeds)) / max(speeds)
        print(f"    spread across directions: {spread * 100:.1f} %")
        if spread > WHEEL_SPREAD_LIMIT:
            failures.append(
                f"top speeds differ by {spread * 100:.1f} %, over the "
                f"{WHEEL_SPREAD_LIMIT * 100:.0f} % limit -- one corner has more drag, "
                "or one bridge channel is weaker than the other"
            )
    print()
    return results, failures


# --------------------------------------------------------------------------
# Direction
# --------------------------------------------------------------------------


def run_direction(bench: Bench, args: argparse.Namespace) -> list[str]:
    """The one measurement with no sensor: which way did it actually turn?"""
    failures: list[str] = []
    inversions: dict[str, bool] = {}

    print("=" * 72)
    print("  Direction check. Watch the wheel, then answer.")
    print()
    for wheel in ("left", "right"):
        left = wheel == "left"
        duty = args.direction_duty
        rpm_l, rpm_r = bench.hold(duty if left else 0.0, 0.0 if left else duty, args.dwell * 3)
        turned = rpm_l if left else rpm_r
        bench.coast(args.settle)
        if turned < 1.0:
            failures.append(
                f"{wheel}: the encoder saw {turned:.1f} RPM at duty {duty:.2f}. Either "
                "the wheel did not turn or that encoder is not counting -- fix that "
                "before recording a direction"
            )
        answer = ask(f"Did the {wheel} wheel turn FORWARD (the way the car drives)?")
        inversions[wheel] = not answer

    print()
    print("  Add to pi/config/config.local.yaml:")
    print()
    print("    params:")
    print(f"      invert_left: {str(inversions['left']).lower()}")
    print(f"      invert_right: {str(inversions['right']).lower()}")
    print()
    if any(inversions.values()):
        print("  These are requires_disarm parameters, so they cannot be changed")
        print("  while the car is armed. Set them, restart, and re-run this check.")
    print()
    return failures


# --------------------------------------------------------------------------
# Truth table
# --------------------------------------------------------------------------


def spin_down(bench: Bench, wheel: str, seconds: float, floor: float) -> float:
    """Seconds until the wheel drops below ``floor`` RPM, or ``seconds`` if not."""
    start = bench.clock.monotonic()
    deadline = start + seconds
    dt = CONTROL_PERIOD_S
    while bench.clock.monotonic() < deadline:
        rpm_l, rpm_r = bench.sample(dt)
        if (rpm_l if wheel == "left" else rpm_r) <= floor:
            return bench.clock.monotonic() - start
        bench.clock.sleep(dt)
    return seconds


def run_truth_table(bench: Bench, args: argparse.Namespace, config: VehicleConfig) -> list[str]:
    """Spin up, release two ways, and compare.

    The pass criterion is the speed *remaining* after a fixed window rather than
    the time taken to reach a floor. Near the floor both curves are dominated by
    bearing and gearbox friction and they converge, so a time-to-floor test has
    almost no discrimination left exactly where it needs some.
    """
    failures: list[str] = []
    wheel = "left" if args.wheel in ("left", "both") else "right"
    left = wheel == "left"
    duty = min(args.truth_duty, config.max_duty)
    floor = max(config.stall_rpm_threshold, 5.0)
    # Roughly one mechanical time constant. Much longer and both curves have
    # sunk into the friction floor, where they converge and tell you nothing.
    window = max(0.1, args.release)

    print("=" * 72)
    print(f"  Truth table on the {wheel} wheel at duty {duty:.2f}.")
    print("  Coast should free-wheel down slowly. Brake should stop it hard.")
    print()

    def release(braking: bool) -> tuple[float, float, float]:
        bench.hold(duty if left else 0.0, 0.0 if left else duty, args.spinup)
        peak = abs(bench.rpm_l if left else bench.rpm_r)
        if braking:
            bench.brake(config.brake_strength)
        else:
            bench.coast()
        bench.settle(window)
        remaining = abs(bench.rpm_l if left else bench.rpm_r)
        # Then let it stop completely before the next run, and time that too.
        to_floor = window + spin_down(bench, wheel, args.spindown, floor)
        bench.coast(args.settle)
        return peak, remaining, to_floor

    peak_coast, coasted, coast_time = release(False)
    print(
        f"    COAST (enable LOW):            {peak_coast:6.1f} -> {coasted:6.1f} RPM "
        f"after {window:.2f} s   (floor at {coast_time:5.2f} s)"
    )
    peak_brake, braked, brake_time = release(True)
    print(
        f"    BRAKE (IN pair EQUAL, EN up):  {peak_brake:6.1f} -> {braked:6.1f} RPM "
        f"after {window:.2f} s   (floor at {brake_time:5.2f} s)"
    )
    print()

    if peak_coast < floor * 3 or peak_brake < floor * 3:
        failures.append(
            f"the wheel only reached {min(peak_coast, peak_brake):.1f} RPM before being "
            "released; the comparison proves nothing. Raise --truth-duty or --spinup"
        )
    elif braked >= coasted:
        # A swapped pair does not produce a near miss: brake and coast trade
        # places outright and this ratio goes above 1.
        failures.append(
            f"braking left {braked:.1f} RPM and coasting left {coasted:.1f} RPM. Braking "
            "is not stopping the wheel at all. Either IN1/IN2 are crossed, or the enable "
            "is being dropped for the brake -- and on an L298 that is a coast"
        )
    else:
        print(
            f"    brake removed {peak_brake - braked:.1f} RPM against coast's "
            f"{peak_coast - coasted:.1f}: the truth table is the right way round"
        )
        if braked > coasted * 0.85:
            print(
                f"    NOTE: only {(1.0 - braked / coasted) * 100:.0f} % better than coasting. "
                f"brake_strength is {config.brake_strength:.2f}; raise it if the car "
                "does not stop convincingly on the track."
            )
    print()
    return failures


# --------------------------------------------------------------------------
# Both motors -- the regulator watch
# --------------------------------------------------------------------------


def run_both(bench: Bench, args: argparse.Namespace, config: VehicleConfig) -> list[str]:
    """Ramp the pair together and watch for a simultaneous collapse.

    Both wheels losing speed in the same instant is the boost converter folding
    back, not two mechanical stalls that happened to coincide. A single wheel
    dropping while the other holds is the opposite diagnosis and a different fix.
    """
    failures: list[str] = []
    print("=" * 72)
    print("  Both motors together. This is the combined 1.5 A budget under test.")
    print()

    peak_l = peak_r = 0.0
    trip_duty: float | None = None
    duty = args.start
    while duty <= args.max_duty + 1e-9:
        rpm_l, rpm_r = bench.hold(duty, duty, args.dwell)
        print(f"      duty {duty:5.2f}   left {rpm_l:7.1f}   right {rpm_r:7.1f} RPM")
        collapsed_l = peak_l > config.stall_rpm_threshold and rpm_l < peak_l * COLLAPSE_FRACTION
        collapsed_r = peak_r > config.stall_rpm_threshold and rpm_r < peak_r * COLLAPSE_FRACTION
        if collapsed_l and collapsed_r and trip_duty is None:
            trip_duty = duty
            print(f"      BOTH wheels collapsed at duty {duty:.2f}: that is the regulator")
        elif collapsed_l != collapsed_r:
            failures.append(
                f"at duty {duty:.2f} only the {'left' if collapsed_l else 'right'} wheel "
                "collapsed. One wheel is a stall or a weak bridge channel, not a supply "
                "problem"
            )
        peak_l = max(peak_l, rpm_l)
        peak_r = max(peak_r, rpm_r)
        duty += args.step
    bench.coast(args.settle)

    print()
    if trip_duty is None:
        print(f"    No regulator trip up to duty {args.max_duty:.2f} on both motors.")
    else:
        print(f"    Regulator trips at about duty {trip_duty:.2f} on both motors.")
        print("    That is the number duty_sum_max exists to stay under: keep")
        print(f"    duty_sum_max below {trip_duty * 2:.2f} (currently {config.duty_sum_max:.2f}).")
        if config.duty_sum_max >= trip_duty * 2:
            failures.append(
                f"duty_sum_max is {config.duty_sum_max:.2f} but the pair browns out at a "
                f"combined {trip_duty * 2:.2f}. The budget is not protecting anything"
            )
    print()
    return failures


# --------------------------------------------------------------------------
# Shared
# --------------------------------------------------------------------------


def ask(question: str) -> bool:
    try:
        return input(f"  {question} [y/N] ").strip().lower() in ("y", "yes")
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def confirm(config: VehicleConfig, args: argparse.Namespace) -> bool:
    print()
    print("  " + "=" * 66)
    print("   MOTOR TEST -- this drives the motors.")
    print("     - wheels off the ground, 20 mm of air under every tyre")
    print("     - safety glasses on")
    print("     - master switch reachable without moving your feet")
    print("     - nothing loose within 300 mm of a wheel")
    print(f"     - duty is capped at {args.max_duty:.2f}, each step held {args.dwell:g} s")
    print("     - abort at any time with Ctrl-C; the bridge is disabled first")
    print("  " + "=" * 66)
    if not sys.stdin.isatty():
        print("  no tty: refusing to drive the motors unattended")
        return False
    try:
        return input("  Type GO to continue: ").strip().upper() == "GO"
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Duty ramp, direction check, break-away duty and regulator watch.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--ramp", action="store_true", help="per-wheel duty ramp (default)")
    parser.add_argument("--direction", action="store_true", help="ask which way each wheel turns")
    parser.add_argument(
        "--truth-table", action="store_true", help="prove brake and coast are not swapped"
    )
    parser.add_argument(
        "--both", action="store_true", help="ramp both motors together and watch the regulator"
    )
    parser.add_argument(
        "--wheel", choices=("left", "right", "both"), default="both", help="which wheel"
    )
    parser.add_argument("--start", type=float, default=0.10, help="first duty in a ramp")
    parser.add_argument("--step", type=float, default=0.02, help="duty increment")
    parser.add_argument("--max-duty", type=float, default=0.45, help="highest duty commanded")
    parser.add_argument("--dwell", type=float, default=0.35, help="seconds held at each step")
    parser.add_argument("--settle", type=float, default=1.0, help="coast between runs")
    parser.add_argument(
        "--direction-duty", type=float, default=0.30, help="duty for the direction check"
    )
    parser.add_argument("--truth-duty", type=float, default=0.50, help="duty for the truth table")
    parser.add_argument("--spinup", type=float, default=2.0, help="truth-table spin-up seconds")
    parser.add_argument(
        "--release",
        type=float,
        default=0.25,
        help="window over which brake and coast are compared; about one mechanical "
        "time constant, because much longer and both have reached the friction floor",
    )
    parser.add_argument(
        "--spindown", type=float, default=4.0, help="how long to wait for a full stop"
    )
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    return parser


def load_config(path: Path | None) -> VehicleConfig:
    if path is not None:
        return VehicleConfig.load(path, LOCAL_CONFIG_PATH)
    return VehicleConfig.load_default()


def open_backend(name: str, clock: RealClock):  # noqa: ANN201 - GpioBackend
    """Hand the mock the loop's clock so `--backend mock` is a real dry run."""
    gpio = select_backend(name)
    if getattr(gpio, "step", None) is not None:
        gpio.clock = clock
    return gpio


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not any((args.ramp, args.direction, args.truth_table, args.both)):
        args.ramp = True

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    args.max_duty = min(abs(args.max_duty), config.max_duty)
    if not confirm(config, args):
        print("  Aborted. Nothing was driven.")
        return 2

    clock = RealClock()
    try:
        gpio = open_backend(args.backend, clock)
    except GpioError as exc:
        print(f"cannot open the GPIO backend: {exc}", file=sys.stderr)
        return 2

    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    failures: list[str] = []
    status = 1
    with PanicChain(panic):
        bench = None
        try:
            bench = Bench(gpio, config, clock)
            panic.attach(motors=bench.motors)
            print()
            if args.ramp:
                _results, problems = run_ramp(bench, args, config)
                failures.extend(problems)
            if args.direction:
                failures.extend(run_direction(bench, args))
            if args.truth_table:
                failures.extend(run_truth_table(bench, args, config))
            if args.both:
                failures.extend(run_both(bench, args, config))

            print("=" * 72)
            if failures:
                print("  FAIL")
                for problem in failures:
                    print(f"    - {problem}")
                status = 1
            else:
                print("  PASS -- phase 4 gates met. Next: scripts/encoder_bench.py")
                status = 0
            print("=" * 72)
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            status = 2
        finally:
            if bench is not None:
                bench.close()
            gpio.cleanup()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
