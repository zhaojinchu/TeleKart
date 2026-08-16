#!/usr/bin/env python3
"""Measure this drivetrain and write calibration.yaml.

Nothing in the firmware hardcodes a top speed. The nameplate says a GA37-520
does 360 RPM at 12 V; this car runs 9 V through a Darlington bridge that eats
1.4 V of it, with 1.5 A shared between two motors, and the truth is somewhere
near 150-200 RPM. Every speed-scaled thing in the system -- the throttle map,
the telemetry ``v_max``, the desktop speedometer -- reads ``max_rpm_measured``,
which is what lets the same build work unchanged when the L298N is eventually
replaced with a MOSFET bridge.

The sweep runs **one wheel at a time**, in both directions, because two motors
at once measures the *regulator* rather than the motors: the 1.5 A budget is a
shared resource, so each wheel's max_rpm would really be "the speed this wheel
reaches while the other one is also drawing", and targets set from that can
never both be met.

``--off-ground`` and ``--on-ground`` are not a default and an override. One of
them is required, every run, because the flag travels with the data and a bench
number recorded as a ground number is worse than no number at all -- it is
optimistic by 20-40 % and nothing downstream can tell.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_PI_ROOT = Path(__file__).resolve().parents[1]
_PROTOCOL_ROOT = _PI_ROOT.parent / "packages" / "telekart_protocol"
for _candidate in (_PI_ROOT, _PROTOCOL_ROOT):
    if _candidate.is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from telekart.app import HardwarePanic, PanicChain  # noqa: E402
from telekart.calibration import (  # noqa: E402
    WHEEL_KEYS,
    AutoCalibrator,
    CalibrationAborted,
    CalibrationError,
    DriveCalibration,
)
from telekart.config import ConfigError, VehicleConfig  # noqa: E402
from telekart.constants import CALIBRATION_PATH, LOCAL_CONFIG_PATH, rpm_to_mps  # noqa: E402
from telekart.drivers.encoder import QuadratureEncoder  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.drivers.servo import SteeringServo  # noqa: E402
from telekart.hal.base import GpioError, select_backend  # noqa: E402
from telekart.util.clock import RealClock  # noqa: E402

#: Acceptance thresholds, from calibration.md section 4.3.
MAX_RPM_SPREAD = 0.20
BENCH_RPM_MIN = 120.0
BENCH_RPM_MAX = 260.0
DEADBAND_MIN = 0.05
DEADBAND_MAX = 0.30

#: What the load is expected to cost when the same sweep is re-run with the
#: wheels on the floor. A ground figure that does not drop at all means the
#: wheels were not actually touching, or the run went open loop.
GROUND_DROP_MIN = 0.20
GROUND_DROP_MAX = 0.40


class Progress:
    """One line per stage. The sweep takes a couple of minutes and silence is
    indistinguishable from a hang."""

    def __init__(self) -> None:
        self._stage = ""

    def __call__(self, stage: str, fraction: float) -> None:
        if stage != self._stage:
            if self._stage:
                print()
            self._stage = stage
        print(f"\r  {stage:<12} {fraction * 100:5.1f} %", end="", flush=True)
        if stage == "done":
            print()


def check(calibration: DriveCalibration, config: VehicleConfig, on_ground: bool) -> list[str]:
    """The acceptance gates. A calibration that fails these is not a result."""
    failures: list[str] = []

    speeds = [calibration.max_rpm.get(key, 0.0) for key in WHEEL_KEYS]
    if any(value <= 0.0 for value in speeds):
        failures.append("at least one direction never turned")
        return failures

    spread = (max(speeds) - min(speeds)) / max(speeds)
    if spread > MAX_RPM_SPREAD:
        failures.append(
            f"the four max_rpm values spread {spread * 100:.1f} %, over the "
            f"{MAX_RPM_SPREAD * 100:.0f} % limit -- one corner has mechanical drag, "
            "or one bridge channel is weaker than the other"
        )

    measured = calibration.max_rpm_measured
    if not on_ground:
        if measured > BENCH_RPM_MAX:
            failures.append(
                f"max_rpm_measured is {measured:.0f} RPM, above {BENCH_RPM_MAX:.0f}. "
                "You are not current-limited, which means encoder_cpr is probably "
                "wrong -- most likely by an integer factor. Re-measure it with "
                "scripts/encoder_test.py --hand"
            )
        elif measured < BENCH_RPM_MIN * 0.67:
            failures.append(
                f"max_rpm_measured is {measured:.0f} RPM, well under {BENCH_RPM_MIN:.0f}. "
                "Mechanical drag, a flat pack, or the regulator is not actually at 9 V"
            )
        elif measured < BENCH_RPM_MIN:
            failures.append(
                f"max_rpm_measured is {measured:.0f} RPM, under the {BENCH_RPM_MIN:.0f}-"
                f"{BENCH_RPM_MAX:.0f} band this drivetrain should reach on the bench"
            )

    for key in WHEEL_KEYS:
        deadband = calibration.deadband.get(key, 0.0)
        if deadband < DEADBAND_MIN:
            failures.append(
                f"{key}: deadband {deadband:.2f} is below {DEADBAND_MIN:.2f} -- the RPM "
                "threshold is firing on encoder noise"
            )
        elif deadband > DEADBAND_MAX:
            failures.append(
                f"{key}: deadband {deadband:.2f} is above {DEADBAND_MAX:.2f} -- something "
                "is binding on that corner"
            )

    for key in WHEEL_KEYS:
        points = calibration.ff_lut.get(key, [])
        if len(points) < 2:
            failures.append(f"{key}: the feedforward table has fewer than two points")
            continue
        duties = [duty for _rpm, duty in points]
        if any(later < earlier for earlier, later in zip(duties, duties[1:])):
            failures.append(
                f"{key}: the feedforward table is not monotonic even after "
                "monotonization, which means the sweep itself was unstable -- go back "
                "to scripts/encoder_bench.py"
            )

    return failures


def summarise(calibration: DriveCalibration, config: VehicleConfig) -> None:
    print()
    print("=" * 72)
    print("  Measured")
    for key in WHEEL_KEYS:
        points = calibration.ff_lut.get(key, [])
        print(
            f"    {key:>10}: max {calibration.max_rpm.get(key, 0.0):6.1f} RPM   "
            f"deadband {calibration.deadband.get(key, 0.0):5.2f}   "
            f"{len(points)} ff points"
        )
    measured = calibration.max_rpm_measured
    v_max = rpm_to_mps(measured, config.wheel_diameter_m)
    print()
    print(f"    max_rpm_measured  {measured:6.1f} RPM   (the minimum of the four)")
    print(f"    v_max             {v_max:6.3f} m/s  = {v_max * 3.6:5.2f} km/h")
    print(f"    on_ground         {calibration.on_ground}")
    print(f"    measured_at       {calibration.measured_at}")


def compare(previous: DriveCalibration | None, fresh: DriveCalibration) -> None:
    """A ground run is expected to be slower than a bench run, by a known amount."""
    if previous is None or not previous.is_measured:
        return
    if previous.on_ground == fresh.on_ground:
        return
    bench = previous if not previous.on_ground else fresh
    ground = fresh if not previous.on_ground else previous
    if bench.max_rpm_measured <= 0.0:
        return
    drop = 1.0 - ground.max_rpm_measured / bench.max_rpm_measured
    print()
    print(
        f"    versus the {'bench' if not previous.on_ground else 'ground'} run on file: "
        f"{drop * 100:+.0f} % change in max_rpm_measured"
    )
    if drop < GROUND_DROP_MIN:
        print(
            "    That is less than the 20-40 % the load should cost. Either the wheels"
        )
        print("    were not actually touching, or the run went open loop.")
    elif drop > GROUND_DROP_MAX:
        print("    That is more than the 20-40 % expected. Check tyre drag and the pack.")
    else:
        print("    Within the 20-40 % the load should cost. Both numbers are real.")


def confirm(config: VehicleConfig, on_ground: bool) -> bool:
    print()
    print("  " + "=" * 66)
    if on_ground:
        print("   ON-GROUND calibration. The car WILL move across the floor.")
        print("     - clear 3 m in every direction")
        print("     - master switch reachable without chasing the car")
    else:
        print("   BENCH calibration. Wheels off the ground.")
        print("     - 20 mm of air under every tyre")
        print("     - the car cannot walk off the blocks")
    print("     - safety glasses on")
    print("     - fully charged pack: a calibration on a half-flat pack measures the pack")
    print(f"     - regulator at 9.0 V, and >= 8.5 V under a 40 % load")
    print(f"     - geometry and encoder_cpr already measured: cpr={config.encoder_cpr}, "
          f"wheel={config.wheel_diameter_m:.3f} m")
    print("     - bring-up phases 1-5 passed, phase 5 especially: a sweep on a loop")
    print("       that is dropping deadlines measures the deadlines")
    print("  " + "=" * 66)
    if not sys.stdin.isatty():
        print("  no tty: refusing to run a calibration unattended")
        return False
    try:
        return input("  Type GO to continue: ").strip().upper() == "GO"
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the drive auto-calibration and write calibration.yaml.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ground = parser.add_mutually_exclusive_group(required=True)
    ground.add_argument(
        "--off-ground", action="store_true", help="wheels in the air (the bench run)"
    )
    ground.add_argument(
        "--on-ground", action="store_true", help="wheels on the floor (the real run)"
    )
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    parser.add_argument(
        "--output", type=Path, default=CALIBRATION_PATH, help="where to write the result"
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="measure and report, but write nothing"
    )
    parser.add_argument("--dwell", type=float, default=0.90, help="seconds held at each duty")
    parser.add_argument("--settle", type=float, default=0.60, help="seconds between runs")
    parser.add_argument(
        "--deadband-step", type=float, default=0.01, help="duty increment in the break-away search"
    )
    parser.add_argument("--lut-points", type=int, default=6, help="feedforward breakpoints")
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
    on_ground = bool(args.on_ground)

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    if not confirm(config, on_ground):
        print("  Aborted. Nothing was driven.")
        return 2

    previous = DriveCalibration.load(args.output)

    clock = RealClock()
    try:
        gpio = open_backend(args.backend, clock)
    except GpioError as exc:
        print(f"cannot open the GPIO backend: {exc}", file=sys.stderr)
        return 2

    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    status = 1
    with PanicChain(panic):
        try:
            motors = MotorPair(gpio, config.pins.motors, config, clock)
            servo = SteeringServo(gpio, config.pins.servo, config, clock)
            panic.attach(motors=motors, servo=servo)
            encoder_l = QuadratureEncoder(
                gpio,
                config.pins.encoders.left_a,
                config.pins.encoders.left_b,
                cpr=config.encoder_cpr,
                invert=config.encoder_invert_left,
            )
            encoder_r = QuadratureEncoder(
                gpio,
                config.pins.encoders.right_a,
                config.pins.encoders.right_b,
                cpr=config.encoder_cpr,
                invert=config.encoder_invert_right,
            )

            calibrator = AutoCalibrator(
                motors=motors,
                encoder_l=encoder_l,
                encoder_r=encoder_r,
                config=config,
                clock=clock,
                servo=servo,
                # MockBackend needs its plant advanced in step with the clock;
                # the real car's plant is the car.
                pump=getattr(gpio, "step", None),
                on_progress=Progress(),
                settle_s=args.settle,
                dwell_s=args.dwell,
                deadband_step=args.deadband_step,
                lut_points=args.lut_points,
            )

            print()
            print("  Sweeping left_fwd, left_rev, right_fwd, right_rev -- one wheel at a")
            print("  time, both directions. Ctrl-C stops and disables the bridge.")
            calibration = calibrator.run(on_ground=on_ground)

            summarise(calibration, config)
            compare(previous, calibration)
            failures = check(calibration, config, on_ground)

            print()
            print("  GATES")
            if failures:
                for problem in failures:
                    print(f"    [FAIL] {problem}")
            else:
                print("    [PASS] four max_rpm within 20 %")
                print("    [PASS] deadbands within 0.05 .. 0.30")
                print("    [PASS] feedforward tables monotonic")
                if not on_ground:
                    print(f"    [PASS] max_rpm_measured inside {BENCH_RPM_MIN:.0f}"
                          f"-{BENCH_RPM_MAX:.0f} RPM")

            if args.dry_run:
                print()
                print("  --dry-run: nothing written.")
                status = 1 if failures else 0
            elif failures:
                print()
                print("  Not written. A calibration that fails its gates is not a result:")
                print("  everything downstream would scale off a number you already know")
                print("  is wrong. Fix the cause and run it again.")
                status = 1
            else:
                calibration.save(args.output)
                print()
                print(f"  Written to {args.output}")
                print("  Restart telekart-control to pick it up. CALIBRATION_MISSING")
                print("  should clear and TelemetryFlags.CALIBRATED should set.")
                status = 0
            print("=" * 72)
        except CalibrationAborted as exc:
            print(f"\n  {exc}")
            status = 2
        except CalibrationError as exc:
            print(f"\n  calibration failed: {exc}")
            status = 1
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            status = 2
        finally:
            gpio.cleanup()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
