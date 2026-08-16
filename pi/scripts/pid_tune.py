#!/usr/bin/env python3
"""Step-response measurement for the per-wheel speed loop.

Feedforward carries the load; the PID only trims. That is what makes this loop
tunable at all when the plant gain is roughly 2x uncertain, and it is why the
first thing this script offers is ``--open-loop``: if the feedforward table
alone cannot track within +-15 % across 20-100 % of ``max_rpm_measured``, no
PID gain will rescue it and the problem is in calibration.yaml.

There is a ceiling on what can be tuned here and it is worth knowing before you
start. The encoder velocity estimate carries a 25 ms output filter, which puts
its corner at about 6.4 Hz, so the closed loop cannot usefully exceed ~7 Hz of
bandwidth. Chasing past that produces oscillation caused by filter phase rather
than by the plant, and ``pid_kd`` makes it worse rather than better.

What comes out of each step: rise time, overshoot, settling time, steady-state
error and peak-to-peak ripple, checked against the phase 7 gates.
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
from telekart.calibration import DriveCalibration  # noqa: E402
from telekart.config import ConfigError, VehicleConfig  # noqa: E402
from telekart.constants import CALIBRATION_PATH, CONTROL_PERIOD_S, LOCAL_CONFIG_PATH  # noqa: E402
from telekart.control.pid import PID  # noqa: E402
from telekart.control.shaping import clamp, rate_limit  # noqa: E402
from telekart.drivers.encoder import QuadratureEncoder  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.drivers.servo import SteeringServo  # noqa: E402
from telekart.hal.base import GpioError, select_backend  # noqa: E402
from telekart.util.clock import RealClock  # noqa: E402

#: Phase 7 gates, from bringup.md.
SETTLE_LIMIT_S = 0.40
SETTLE_BAND = 0.05
STEADY_ERROR_LIMIT = 0.03
RIPPLE_LIMIT = 0.08

#: tuning.md section 2.3: pid_kp is in duty per RPM, so this product is the
#: fraction of full authority a full-scale error commands. Outside this band you
#: are in the wrong decade and no amount of careful bisection will help.
KP_PRODUCT_MIN = 0.5
KP_PRODUCT_MAX = 1.5

#: Integrator saturation time, in seconds, that behaves. Faster and the
#: integrator dominates transients; slower and droop never gets corrected.
I_SATURATE_MIN_S = 0.5
I_SATURATE_MAX_S = 2.0


@dataclass(slots=True)
class Trace:
    dt: float
    target: list[float] = field(default_factory=list)
    rpm: list[float] = field(default_factory=list)
    duty: list[float] = field(default_factory=list)

    def add(self, target: float, rpm: float, duty: float) -> None:
        self.target.append(target)
        self.rpm.append(rpm)
        self.duty.append(duty)


@dataclass(slots=True)
class StepMetrics:
    setpoint: float
    rise_s: float | None
    overshoot: float
    settle_s: float | None
    steady_error: float
    ripple: float
    final: float


def analyse(trace: Trace, setpoint: float, tail_s: float) -> StepMetrics:
    """Reduce a step response to the five numbers that decide a gain change."""
    values = trace.rpm
    dt = trace.dt
    if not values or setpoint <= 0.0:
        return StepMetrics(setpoint, None, 0.0, None, 1.0, 0.0, 0.0)

    tail_count = max(1, int(tail_s / dt))
    tail = values[-tail_count:]
    final = sum(tail) / len(tail)
    peak = max(values)

    rise: float | None = None
    low = 0.1 * setpoint
    high = 0.9 * setpoint
    crossed_low: float | None = None
    for index, value in enumerate(values):
        if crossed_low is None and value >= low:
            crossed_low = index * dt
        if crossed_low is not None and value >= high:
            rise = index * dt - crossed_low
            break

    # Settling is the LAST time it left the band, not the first time it entered:
    # a response that touches the band and then overshoots out of it has not
    # settled, and reporting the first entry would say it had.
    band = SETTLE_BAND * setpoint
    settle: float | None = None
    for index in range(len(values) - 1, -1, -1):
        if abs(values[index] - setpoint) > band:
            settle = (index + 1) * dt
            break
    else:
        settle = 0.0
    if settle is not None and settle >= len(values) * dt:
        settle = None

    return StepMetrics(
        setpoint=setpoint,
        rise_s=rise,
        overshoot=max(0.0, (peak - setpoint) / setpoint),
        settle_s=settle,
        steady_error=abs(final - setpoint) / setpoint,
        ripple=(max(tail) - min(tail)) / setpoint,
        final=final,
    )


class SpeedLoop:
    """One wheel's speed loop, built the same way the control loop builds it.

    Feedforward from the calibration table, a PID with one-sided output limits
    so the trim can only ever push in the commanded direction, and the deadband
    floor applied *after* both -- because the table is a curve relative to the
    deadband and adding it twice puts a step at the bottom of the throttle that
    no gain removes.
    """

    def __init__(
        self,
        wheel: str,
        config: VehicleConfig,
        calibration: DriveCalibration,
        motors: MotorPair,
        encoder: QuadratureEncoder,
        other: QuadratureEncoder,
        gpio: object,
        clock: RealClock,
        gains: tuple[float, float, float],
        closed_loop: bool,
    ) -> None:
        self.wheel = wheel
        self.left = wheel == "left"
        self.config = config
        self.calibration = calibration
        self.motors = motors
        self.encoder = encoder
        self.other = other
        self.clock = clock
        self.closed_loop = closed_loop
        self.pid = PID(*gains, i_clamp=config.pid_i_clamp)
        self.target = 0.0
        self.rpm = 0.0
        self.duty = 0.0
        #: With the acceleration limiter engaged the target itself takes 280 ms
        #: to reach half of a 140 RPM ceiling, and what gets measured is the
        #: limiter rather than the loop. Tuning wants a true step; the limiter
        #: is switched back on for the reversal test, where it is the point.
        self.ramp = False
        self.pump = getattr(gpio, "step", None)

    def reset(self) -> None:
        self.pid.reset()
        self.target = 0.0
        self.motors.coast()

    def tick(self, dt: float, demand: float) -> None:
        if self.pump is not None:
            self.pump(dt)

        sample = self.encoder.sample(dt)
        other = self.other.sample(dt)
        self.rpm = sample.rpm
        self.motors.note_speed(
            sample.rpm if self.left else other.rpm,
            other.rpm if self.left else sample.rpm,
        )

        if self.ramp:
            # The RPM target is rate-limited, not the duty. Inrush during a
            # throttle step is what trips the regulator, and rate-limiting the
            # duty instead would fight the PID and make the loop untunable.
            rate = (
                self.config.accel_rpm_per_s
                if abs(demand) > abs(self.target)
                else self.config.decel_rpm_per_s
            )
            self.target = rate_limit(demand, self.target, rate, dt)
        else:
            self.target = demand

        ceiling = self.config.max_duty
        feedforward = self.calibration.feedforward(self.wheel, self.target)
        if abs(self.target) < 1.0:
            self.pid.reset()
            duty = 0.0
        elif self.closed_loop:
            if self.target > 0.0:
                self.pid.set_output_limits(0.0, ceiling)
            else:
                self.pid.set_output_limits(-ceiling, 0.0)
            duty = self.pid.update(self.target, sample.rpm, dt, feedforward=feedforward)
        else:
            duty = clamp(feedforward, -ceiling, ceiling)

        magnitude = abs(duty)
        deadband = self.calibration.deadband_for(self.wheel, self.target)
        if 0.0 < magnitude < deadband <= ceiling:
            duty = deadband if duty >= 0.0 else -deadband

        self.duty = duty
        self.encoder.set_direction_hint(1 if duty > 0.0 else (-1 if duty < 0.0 else 0))
        self.motors.drive(duty if self.left else 0.0, 0.0 if self.left else duty)

    def run(self, demand: float, seconds: float, trace: Trace | None = None) -> None:
        dt = CONTROL_PERIOD_S
        deadline = self.clock.monotonic() + seconds
        while self.clock.monotonic() < deadline:
            self.tick(dt, demand)
            if trace is not None:
                trace.add(self.target, self.rpm, self.duty)
            self.clock.sleep(dt)


# --------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------


def run_step(loop: SpeedLoop, setpoint: float, args: argparse.Namespace) -> StepMetrics:
    loop.reset()
    loop.run(0.0, args.settle)
    trace = Trace(CONTROL_PERIOD_S)
    loop.run(setpoint, args.hold, trace)
    loop.reset()
    loop.run(0.0, args.settle)
    return analyse(trace, setpoint, args.tail)


def print_step(metrics: StepMetrics, fraction: float) -> list[str]:
    failures: list[str] = []
    rise = f"{metrics.rise_s * 1000:6.0f} ms" if metrics.rise_s is not None else "     --"
    settle = (
        f"{metrics.settle_s * 1000:6.0f} ms" if metrics.settle_s is not None else "  never"
    )
    print(
        f"    {fraction * 100:3.0f} % -> {metrics.setpoint:6.1f} RPM   "
        f"rise {rise}   overshoot {metrics.overshoot * 100:5.1f} %   "
        f"settle {settle}   error {metrics.steady_error * 100:5.1f} %   "
        f"ripple {metrics.ripple * 100:5.1f} %"
    )
    if metrics.steady_error > STEADY_ERROR_LIMIT:
        failures.append(
            f"{fraction * 100:.0f} % setpoint: steady-state error "
            f"{metrics.steady_error * 100:.1f} % exceeds {STEADY_ERROR_LIMIT * 100:.0f} % "
            "-- raise pid_ki"
        )
    if metrics.ripple > RIPPLE_LIMIT:
        failures.append(
            f"{fraction * 100:.0f} % setpoint: ripple {metrics.ripple * 100:.1f} % exceeds "
            f"{RIPPLE_LIMIT * 100:.0f} % -- pid_kp is too high, drop it to 60 %"
        )
    if abs(fraction - 0.5) < 0.01:
        if metrics.settle_s is None or metrics.settle_s > SETTLE_LIMIT_S:
            failures.append(
                f"the 50 % step did not settle to +-5 % within {SETTLE_LIMIT_S * 1000:.0f} ms"
            )
    return failures


def run_open_loop(loop: SpeedLoop, args: argparse.Namespace, max_rpm: float) -> list[str]:
    """tuning.md section 2.2. If this gate fails, stop: a bad ff_lut cannot be
    rescued by any PID."""
    failures: list[str] = []
    print("=" * 78)
    print("  Open loop -- feedforward alone, closed_loop off")
    loop.closed_loop = False
    for fraction in (0.2, 0.4, 0.6, 0.8, 1.0):
        setpoint = fraction * max_rpm
        loop.reset()
        loop.run(0.0, args.settle)
        trace = Trace(CONTROL_PERIOD_S)
        loop.run(setpoint, args.hold, trace)
        metrics = analyse(trace, setpoint, args.tail)
        error = (metrics.final - setpoint) / setpoint
        flag = "" if abs(error) <= 0.15 else "   OUT OF BAND"
        print(
            f"    {fraction * 100:3.0f} % -> asked {setpoint:6.1f}   got {metrics.final:6.1f} RPM   "
            f"{error * 100:+6.1f} %{flag}"
        )
        if abs(error) > 0.15:
            failures.append(
                f"open loop at {fraction * 100:.0f} % is {error * 100:+.1f} % off, outside "
                "+-15 %. The feedforward table is wrong -- re-run calibrate_drive"
            )
    loop.reset()
    loop.closed_loop = True
    print()
    return failures


def run_reversal(loop: SpeedLoop, args: argparse.Namespace, max_rpm: float) -> list[str]:
    """Forward, then reverse. The sequencer must take the duty to zero, wait out
    the dead time, flip the pins and only then ramp."""
    failures: list[str] = []
    print("=" * 78)
    print("  Direction reversal")
    setpoint = 0.35 * max_rpm
    # The acceleration limiter belongs in this test: a reversal is exactly the
    # transient it exists to soften.
    was_ramped = loop.ramp
    loop.ramp = True
    loop.reset()
    loop.run(setpoint, args.hold)
    forward_rpm = loop.rpm

    trace = Trace(CONTROL_PERIOD_S)
    loop.run(-setpoint, args.hold + 1.0, trace)
    reverse_rpm = loop.rpm
    loop.reset()
    loop.ramp = was_ramped

    print(f"    forward {forward_rpm:6.1f} RPM  ->  reverse {reverse_rpm:6.1f} RPM")
    zero_ticks = sum(1 for duty in trace.duty if abs(duty) < 1e-6)
    print(
        f"    {zero_ticks} ticks at zero duty during the changeover "
        f"({zero_ticks * CONTROL_PERIOD_S * 1000:.0f} ms; dead time is "
        f"{loop.config.direction_deadtime_ms} ms)"
    )
    if reverse_rpm >= -1.0:
        failures.append(
            "the wheel never reversed. Either reverse_enabled is off, or the wheel "
            f"never fell below reverse_allowed_rpm ({loop.config.reverse_allowed_rpm:.0f})"
        )
    if zero_ticks * CONTROL_PERIOD_S * 1000 < loop.config.direction_deadtime_ms:
        failures.append(
            "the changeover spent less than direction_deadtime_ms at zero duty -- the "
            "IN pins may be flipping under a live enable, which is a shoot-through"
        )
    print()
    return failures


def run_stall_test(loop: SpeedLoop, args: argparse.Namespace, max_rpm: float) -> list[str]:
    """Anti-windup, by hand. A burst on release is a bug, not a tuning problem."""
    failures: list[str] = []
    print("=" * 78)
    print("  Anti-windup: hold the wheel, then let go")
    setpoint = 0.5 * max_rpm
    loop.reset()
    loop.run(setpoint, 1.0)
    print(f"    Holding {setpoint:.0f} RPM. Block the wheel with a piece of wood now.")
    if not wait("    Press Enter once it is held. "):
        loop.reset()
        return failures
    loop.run(setpoint, args.stall_hold)
    print("    Let go.")
    if not wait("    Press Enter once it is free. "):
        loop.reset()
        return failures

    trace = Trace(CONTROL_PERIOD_S)
    loop.run(setpoint, args.hold, trace)
    loop.reset()

    peak = max(trace.rpm) if trace.rpm else 0.0
    overshoot = (peak - setpoint) / setpoint if setpoint > 0.0 else 0.0
    print(f"    peak after release {peak:6.1f} RPM ({overshoot * 100:+.1f} %)")
    if overshoot > 0.15:
        failures.append(
            f"{overshoot * 100:.0f} % overshoot burst on release. The integrator wound "
            "up while the output was saturated: conditional integration is not freezing "
            "it. That is a bug in PID.update, not something to fix by lowering pid_ki"
        )
    print()
    return failures


# --------------------------------------------------------------------------
# Shared
# --------------------------------------------------------------------------


def wait(message: str) -> bool:
    try:
        input(message)
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def print_decade_checks(config: VehicleConfig, max_rpm: float) -> None:
    """Two pieces of arithmetic that catch a gain in the wrong decade before any
    time is spent bisecting one."""
    product = config.pid_kp * max_rpm
    print()
    print("  Sanity arithmetic")
    print(
        f"    pid_kp * max_rpm_measured = {config.pid_kp:.4f} * {max_rpm:.0f} = "
        f"{product:.2f}   (want {KP_PRODUCT_MIN}-{KP_PRODUCT_MAX})"
    )
    if not KP_PRODUCT_MIN <= product <= KP_PRODUCT_MAX:
        print("      -> wrong decade. Fix this before tuning anything else.")

    typical_error = 0.1 * max_rpm
    if config.pid_ki > 0.0 and typical_error > 0.0:
        saturate = config.pid_i_clamp / (config.pid_ki * typical_error)
        print(
            f"    integrator saturates in pid_i_clamp / (pid_ki * error) = "
            f"{config.pid_i_clamp:.2f} / ({config.pid_ki:.3f} * {typical_error:.0f}) = "
            f"{saturate:.2f} s   (want {I_SATURATE_MIN_S}-{I_SATURATE_MAX_S})"
        )
        if saturate < I_SATURATE_MIN_S:
            print("      -> pid_ki too high: the integrator will dominate transients.")
        elif saturate > I_SATURATE_MAX_S:
            print("      -> pid_ki too low: steady-state droop will never be corrected.")
    print()


def confirm(args: argparse.Namespace, max_rpm: float) -> bool:
    print()
    print("  " + "=" * 74)
    print("   PID STEP TEST -- this spins the wheels.")
    print("     - wheels off the ground unless you are deliberately tuning on the floor")
    print("     - safety glasses on")
    print("     - master switch reachable without moving your feet")
    print(f"     - steps up to {max_rpm:.0f} RPM, held {args.hold:g} s each")
    print("  " + "=" * 74)
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
        description="Step response with overshoot, settling and ripple readout.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--levels",
        default="0.3,0.5,0.8",
        help="setpoints as fractions of max_rpm_measured",
    )
    parser.add_argument("--wheel", choices=("left", "right"), default="left")
    parser.add_argument("--kp", type=float, default=None, help="override pid_kp for this run")
    parser.add_argument("--ki", type=float, default=None, help="override pid_ki for this run")
    parser.add_argument("--kd", type=float, default=None, help="override pid_kd for this run")
    parser.add_argument("--hold", type=float, default=3.0, help="seconds held at each setpoint")
    parser.add_argument("--settle", type=float, default=1.5, help="coast between steps")
    parser.add_argument(
        "--tail", type=float, default=1.0, help="tail window used for steady state and ripple"
    )
    parser.add_argument(
        "--open-loop", action="store_true", help="feedforward-only tracking gate first"
    )
    parser.add_argument(
        "--ramped",
        action="store_true",
        help="apply accel_rpm_per_s to the step, which measures the limiter as well "
        "as the loop; off by default because tuning wants a true step",
    )
    parser.add_argument("--reversal", action="store_true", help="also test a direction reversal")
    parser.add_argument(
        "--stall-test", action="store_true", help="anti-windup check; needs a piece of wood"
    )
    parser.add_argument("--stall-hold", type=float, default=2.0, help="seconds held blocked")
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    parser.add_argument(
        "--calibration", type=Path, default=CALIBRATION_PATH, help="calibration.yaml"
    )
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

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    calibration = DriveCalibration.load(args.calibration)
    if calibration is None or not calibration.is_measured:
        print(
            f"no usable calibration at {args.calibration}. Run "
            "scripts/calibrate_drive.py --off-ground first: without a feedforward "
            "table there is nothing here worth tuning.",
            file=sys.stderr,
        )
        return 2

    gains = (
        config.pid_kp if args.kp is None else args.kp,
        config.pid_ki if args.ki is None else args.ki,
        config.pid_kd if args.kd is None else args.kd,
    )
    max_rpm = calibration.max_rpm_measured
    try:
        levels = [float(part) for part in args.levels.split(",") if part.strip()]
    except ValueError:
        print("--levels must be a comma-separated list of fractions", file=sys.stderr)
        return 2

    if not confirm(args, max_rpm):
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
        loop = None
        try:
            motors = MotorPair(gpio, config.pins.motors, config, clock)
            servo = SteeringServo(gpio, config.pins.servo, config, clock)
            panic.attach(motors=motors, servo=servo)
            # Centre and hold the steering: a step test at an angle measures tyre
            # scrub on one wheel and not on the other.
            servo.center()
            servo.update(CONTROL_PERIOD_S)

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
            left = args.wheel == "left"
            loop = SpeedLoop(
                args.wheel,
                config,
                calibration,
                motors,
                encoder_l if left else encoder_r,
                encoder_r if left else encoder_l,
                gpio,
                clock,
                gains,
                closed_loop=bool(config.closed_loop),
            )
            loop.ramp = bool(args.ramped)

            print()
            print(f"  {calibration.summary()}")
            print(f"  gains kp={gains[0]:.4f} ki={gains[1]:.4f} kd={gains[2]:.4f} "
                  f"i_clamp={config.pid_i_clamp:.2f}   wheel={args.wheel}")
            print_decade_checks(config, max_rpm)

            if args.open_loop:
                failures.extend(run_open_loop(loop, args, max_rpm))

            print("=" * 78)
            print("  Closed-loop steps")
            for fraction in levels:
                metrics = run_step(loop, fraction * max_rpm, args)
                failures.extend(print_step(metrics, fraction))
            print()

            if args.reversal:
                failures.extend(run_reversal(loop, args, max_rpm))
            if args.stall_test:
                failures.extend(run_stall_test(loop, args, max_rpm))

            print("=" * 78)
            if failures:
                print("  FAIL")
                for problem in failures:
                    print(f"    - {problem}")
                print()
                print("  Symptom table: tuning.md section 2.6.")
                status = 1
            else:
                print("  PASS -- phase 7 gates met.")
                status = 0
            print("=" * 78)
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            status = 2
        finally:
            if loop is not None:
                loop.motors.coast()
            gpio.cleanup()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
