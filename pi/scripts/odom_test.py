#!/usr/bin/env python3
"""Check the dead reckoning against a tape measure.

With no IMU, heading comes entirely from the bicycle model and two wheel
encoders. That is enough to be useful and it is not enough to be trusted, so
every number here gets compared against something physical:

``--straight``  Drive a known distance by odometry, then measure what the car
                actually covered and how far sideways it went. The distance
                error is a wheel-diameter error -- the *rolling* diameter under
                load, which is smaller than the moulded one -- and the lateral
                deviation is a steering trim error. Gate: under 0.3 m over 5 m.

``--circle``    Full lock for one turn. ``R = wheelbase / tan(steer_max_deg)``,
                so the default geometry should trace a circle about 0.90 m
                across. If the real circle is much larger, ``steer_max_deg`` is
                overstated and both the odometry and the electronic differential
                are wrong in the same direction.

``--square``    Four legs and four quarter turns, back to the start. Gate:
                closure error under 15 % of the perimeter. Expect 5-15 %: that
                is inherent to dead reckoning off two wheels, not a bug.

``--geometry``  No motion at all. Prints what the configured geometry predicts,
                so the numbers to check with a tape are on screen before the car
                moves.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

_PI_ROOT = Path(__file__).resolve().parents[1]
_PROTOCOL_ROOT = _PI_ROOT.parent / "packages" / "telekart_protocol"
for _candidate in (_PI_ROOT, _PROTOCOL_ROOT):
    if _candidate.is_dir() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from telekart.app import HardwarePanic, PanicChain  # noqa: E402
from telekart.calibration import DriveCalibration  # noqa: E402
from telekart.config import ConfigError, VehicleConfig  # noqa: E402
from telekart.constants import (  # noqa: E402
    CALIBRATION_PATH,
    CONTROL_PERIOD_S,
    LOCAL_CONFIG_PATH,
    counts_to_metres,
)
from telekart.control.mixer import DifferentialMixer  # noqa: E402
from telekart.control.pid import PID  # noqa: E402
from telekart.control.shaping import clamp  # noqa: E402
from telekart.drivers.encoder import QuadratureEncoder  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.drivers.servo import SteeringServo  # noqa: E402
from telekart.hal.base import GpioError, select_backend  # noqa: E402
from telekart.odometry import BicycleOdometry  # noqa: E402
from telekart.util.clock import RealClock  # noqa: E402

#: bringup.md gate 10.1 -- lateral deviation over a 5 m straight.
STRAIGHT_DEVIATION_LIMIT = 0.3
STRAIGHT_REFERENCE_M = 5.0

#: bringup.md gate 10.2 -- square closure, as a fraction of the perimeter.
CLOSURE_LIMIT = 0.15

#: A wheel diameter this far out is a measurement error, not tyre wear.
DIAMETER_WARN = 0.05


@dataclass(slots=True)
class Run:
    """What one leg produced, all from the odometry."""

    distance: float
    x: float
    y: float
    heading: float
    slip: float


class Driver:
    """Both wheels, the servo, the mixer and the odometry, at control rate.

    The same pipeline the control loop runs, minus the network and the safety
    state machine: feedforward from the calibration, a PID trim per wheel, the
    deadband floor after both, the combined-duty budget, and odometry fed from
    the encoder counts rather than from the commanded speed.
    """

    def __init__(
        self,
        gpio: object,
        config: VehicleConfig,
        calibration: DriveCalibration,
        clock: RealClock,
    ) -> None:
        self.gpio = gpio
        self.config = config
        self.calibration = calibration
        self.clock = clock
        self.motors = MotorPair(gpio, config.pins.motors, config, clock)
        self.servo = SteeringServo(gpio, config.pins.servo, config, clock)
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
        self.mixer = DifferentialMixer(config)
        self.odometry = BicycleOdometry(config)
        self.pid_l = PID(config.pid_kp, config.pid_ki, config.pid_kd, i_clamp=config.pid_i_clamp)
        self.pid_r = PID(config.pid_kp, config.pid_ki, config.pid_kd, i_clamp=config.pid_i_clamp)
        self.pump = getattr(gpio, "step", None)

    # -- one tick -----------------------------------------------------------

    def tick(self, dt: float, target_rpm: float, steering: float) -> None:
        if self.pump is not None:
            self.pump(dt)

        self.servo.set_normalized(steering)
        self.servo.update(dt)
        angle = self.servo.applied_angle

        sample_l = self.encoder_l.sample(dt)
        sample_r = self.encoder_r.sample(dt)
        self.motors.note_speed(sample_l.rpm, sample_r.rpm)

        diameter = self.config.wheel_diameter_m
        cpr = self.config.encoder_cpr
        self.odometry.update(
            counts_to_metres(sample_l.counts, cpr, diameter),
            counts_to_metres(sample_r.counts, cpr, diameter),
            angle,
            dt,
        )

        targets = self.mixer.mix(target_rpm, angle)
        duty_l = self._wheel_duty("left", self.pid_l, targets.rpm_l, sample_l.rpm, dt)
        duty_r = self._wheel_duty("right", self.pid_r, targets.rpm_r, sample_r.rpm, dt)

        # Combined budget before the per-motor ceiling, scaling both together so
        # the differential's ratio survives the clamp.
        budget = self.config.duty_sum_max
        total = abs(duty_l) + abs(duty_r)
        if total > budget and total > 0.0:
            scale = budget / total
            duty_l *= scale
            duty_r *= scale

        self.encoder_l.set_direction_hint(_sign(duty_l))
        self.encoder_r.set_direction_hint(_sign(duty_r))
        self.motors.drive(duty_l, duty_r)

    def _wheel_duty(
        self, wheel: str, pid: PID, target: float, measured: float, dt: float
    ) -> float:
        ceiling = self.config.max_duty
        if abs(target) < 1.0:
            pid.reset()
            return 0.0
        feedforward = self.calibration.feedforward(wheel, target)
        if target > 0.0:
            pid.set_output_limits(0.0, ceiling)
        else:
            pid.set_output_limits(-ceiling, 0.0)
        duty = pid.update(target, measured, dt, feedforward=feedforward)
        deadband = self.calibration.deadband_for(wheel, target)
        magnitude = abs(duty)
        if 0.0 < magnitude < deadband <= ceiling:
            duty = deadband if duty >= 0.0 else -deadband
        return clamp(duty, -ceiling, ceiling)

    # -- manoeuvres ---------------------------------------------------------

    def drive_until(
        self,
        target_rpm: float,
        steering: float,
        *,
        metres: float | None = None,
        radians: float | None = None,
        timeout: float,
    ) -> Run:
        """Drive until a distance or a heading change is reached, or time out."""
        start_distance = self.odometry.distance
        start_heading = self.odometry.heading
        turned = 0.0
        previous = start_heading
        deadline = self.clock.monotonic() + timeout
        dt = CONTROL_PERIOD_S

        while self.clock.monotonic() < deadline:
            self.tick(dt, target_rpm, steering)
            self.clock.sleep(dt)

            heading = self.odometry.heading
            step = heading - previous
            # Unwrap: the heading is wrapped to +-pi and a turn that crosses the
            # boundary would otherwise read as a jump of a full revolution.
            if step > math.pi:
                step -= math.tau
            elif step < -math.pi:
                step += math.tau
            turned += step
            previous = heading

            if metres is not None and self.odometry.distance - start_distance >= metres:
                break
            if radians is not None and abs(turned) >= abs(radians):
                break

        self.stop()
        x, y, heading = self.odometry.pose
        return Run(
            distance=self.odometry.distance - start_distance,
            x=x,
            y=y,
            heading=heading,
            slip=self.odometry.slip_index,
        )

    def stop(self, seconds: float = 1.0) -> None:
        self.motors.brake(self.config.brake_strength)
        deadline = self.clock.monotonic() + seconds
        dt = CONTROL_PERIOD_S
        while self.clock.monotonic() < deadline:
            if self.pump is not None:
                self.pump(dt)
            sample_l = self.encoder_l.sample(dt)
            sample_r = self.encoder_r.sample(dt)
            diameter = self.config.wheel_diameter_m
            cpr = self.config.encoder_cpr
            self.odometry.update(
                counts_to_metres(sample_l.counts, cpr, diameter),
                counts_to_metres(sample_r.counts, cpr, diameter),
                self.servo.applied_angle,
                dt,
            )
            self.clock.sleep(dt)
        self.motors.coast()

    def close(self) -> None:
        self.motors.coast()
        self.servo.relax()
        self.encoder_l.close()
        self.encoder_r.close()


def _sign(value: float) -> int:
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


# --------------------------------------------------------------------------
# Geometry report
# --------------------------------------------------------------------------


def report_geometry(config: VehicleConfig, calibration: DriveCalibration | None) -> None:
    lock = config.steer_max_rad
    radius = config.wheelbase_m / math.tan(lock) if lock > 0.0 else math.inf
    split = config.track_width_m * math.tan(lock) / (2.0 * config.wheelbase_m)
    circumference = config.wheel_circumference_m

    print()
    print("=" * 72)
    print("  What the configured geometry predicts. Check each one with a tape.")
    print()
    print(f"    wheel diameter      {config.wheel_diameter_m * 1000:7.1f} mm")
    print(f"    wheel circumference {circumference * 1000:7.1f} mm per revolution")
    print(f"    wheelbase           {config.wheelbase_m * 1000:7.1f} mm")
    print(f"    track width         {config.track_width_m * 1000:7.1f} mm")
    print(f"    steering lock       {config.steer_max_deg:7.1f} deg")
    print()
    print(f"    turn radius at full lock   {radius:6.3f} m  "
          f"-> a circle {radius * 2:5.2f} m across")
    print(f"    differential split         {split * 100:6.1f} %  "
          f"-> {split * 200:5.1f} % between the wheels")
    print(f"    encoder resolution         {circumference / config.encoder_cpr * 1000:6.3f} mm "
          f"per count at {config.encoder_cpr} cpr")
    if calibration is not None and calibration.is_measured:
        v_max = calibration.max_speed_mps(config.wheel_diameter_m)
        print(f"    top speed                  {v_max:6.3f} m/s = {v_max * 3.6:5.2f} km/h "
              f"at {calibration.max_rpm_measured:.0f} RPM")
    print()
    print("  Ten wheel revolutions is "
          f"{circumference * 10:.3f} m -- roll the car that far and measure it.")
    print("=" * 72)


# --------------------------------------------------------------------------
# Manoeuvres
# --------------------------------------------------------------------------


def run_straight(driver: Driver, args: argparse.Namespace, target_rpm: float) -> list[str]:
    failures: list[str] = []
    print("=" * 72)
    print(f"  Straight line: {args.straight:g} m by odometry at {target_rpm:.0f} RPM.")
    print("  Mark the start point on the floor and line the car up against a straight edge.")
    if not wait("  Press Enter to drive. "):
        return ["operator aborted"]

    driver.odometry.reset()
    run = driver.drive_until(
        target_rpm, 0.0, metres=args.straight, timeout=args.timeout
    )

    print()
    print(f"    odometry says   {run.distance:6.3f} m travelled")
    print(f"    pose            x {run.x:+6.3f}  y {run.y:+6.3f}  "
          f"heading {math.degrees(run.heading):+6.1f} deg")
    print(f"    slip index      {run.slip:6.3f} rad/s")
    print()

    actual = ask_float("  Measured distance along the floor, in metres (blank to skip): ")
    lateral = ask_float("  Measured sideways deviation at the end, in metres (blank to skip): ")

    if actual is not None and run.distance > 0.0:
        ratio = actual / run.distance
        suggested = driver.config.wheel_diameter_m * ratio
        print()
        print(f"    odometry / tape = {ratio:.4f}")
        print(f"    wheel_diameter_m {driver.config.wheel_diameter_m:.4f} "
              f"-> {suggested:.4f} m")
        print("    That is the ROLLING diameter under load, which is smaller than the")
        print("    moulded one. It is the right number to put in the config.")
        if abs(ratio - 1.0) > DIAMETER_WARN:
            print(f"    A {abs(ratio - 1.0) * 100:.1f} % error is more than tyre squash;")
            print("    check encoder_cpr as well (scripts/encoder_test.py --hand).")

    if lateral is not None:
        scaled = lateral * (STRAIGHT_REFERENCE_M / max(args.straight, 1e-6))
        print()
        print(f"    deviation {lateral:+.3f} m over {args.straight:g} m "
              f"= {scaled:+.3f} m scaled to {STRAIGHT_REFERENCE_M:g} m")
        if abs(scaled) > STRAIGHT_DEVIATION_LIMIT:
            failures.append(
                f"lateral deviation {scaled:+.2f} m over {STRAIGHT_REFERENCE_M:g} m exceeds "
                f"{STRAIGHT_DEVIATION_LIMIT:g} m. Rule out a drivetrain imbalance first "
                "(swap the motor leads and see whether the pull follows), then correct "
                "steer_trim_us"
            )
        # Small-angle: the heading error that produced this deviation, converted
        # into the trim that would remove it.
        angle = math.atan2(lateral, max(args.straight, 1e-6))
        span = (driver.config.steer_max_us - driver.config.steer_min_us) / 2.0
        lock = driver.config.steer_max_rad
        if lock > 0.0 and span > 0.0:
            trim = -angle / lock * span
            print(f"    suggested steer_trim_us adjustment: {trim:+.0f} us "
                  f"(currently {driver.config.steer_trim_us})")
            print("    Apply half of it, re-run, and converge. Full corrections overshoot.")
    print()
    return failures


def run_circle(driver: Driver, args: argparse.Namespace, target_rpm: float) -> list[str]:
    failures: list[str] = []
    lock = driver.config.steer_max_rad
    predicted = driver.config.wheelbase_m / math.tan(lock) if lock > 0.0 else math.inf
    print("=" * 72)
    print(f"  Full lock circle. Predicted radius {predicted:.3f} m, "
          f"diameter {predicted * 2:.3f} m.")
    print("  Chalk the floor or lay a marker where the rear axle centre starts.")
    if not wait("  Press Enter to drive one full turn. "):
        return ["operator aborted"]

    driver.odometry.reset()
    run = driver.drive_until(
        target_rpm, args.steering, radians=math.tau, timeout=args.timeout
    )

    measured_radius = run.distance / math.tau if run.distance > 0.0 else 0.0
    print()
    print(f"    odometry says   {run.distance:6.3f} m of arc, "
          f"radius {measured_radius:6.3f} m")
    print(f"    ending heading  {math.degrees(run.heading):+6.1f} deg "
          f"(a full turn should wrap back near 0)")
    print(f"    slip index      {run.slip:6.3f} rad/s")

    actual = ask_float("  Measured circle DIAMETER on the floor, in metres (blank to skip): ")
    if actual is not None and actual > 0.0:
        radius = actual / 2.0
        implied = math.degrees(math.atan(driver.config.wheelbase_m / radius))
        print()
        print(f"    measured radius {radius:.3f} m -> steer_max_deg {implied:.1f} "
              f"(config says {driver.config.steer_max_deg:.1f})")
        if abs(implied - driver.config.steer_max_deg) > 3.0:
            failures.append(
                f"steer_max_deg is {driver.config.steer_max_deg:.1f} but the circle implies "
                f"{implied:.1f}. The usual cause is measuring the servo horn's angle rather "
                "than the wheel's, which overstates lock badly -- and both the odometry and "
                "the differential are wrong in the same direction until it is fixed"
            )
    print()
    return failures


def run_square(driver: Driver, args: argparse.Namespace, target_rpm: float) -> list[str]:
    failures: list[str] = []
    side = args.square
    perimeter = 4.0 * side
    print("=" * 72)
    print(f"  Square: four {side:g} m legs with a quarter turn between each.")
    print("  Mark the start. You need about {:.0f} m of clear floor.".format(side + 2))
    if not wait("  Press Enter to drive. "):
        return ["operator aborted"]

    driver.odometry.reset()
    for leg in range(4):
        print(f"    leg {leg + 1}/4 ...", flush=True)
        driver.drive_until(target_rpm, 0.0, metres=side, timeout=args.timeout)
        print(f"    turn {leg + 1}/4 ...", flush=True)
        driver.drive_until(
            target_rpm * 0.6, args.steering, radians=math.pi / 2.0, timeout=args.timeout
        )

    x, y, heading = driver.odometry.pose
    closure = math.hypot(x, y)
    fraction = closure / perimeter
    print()
    print(f"    odometry closure  {closure:6.3f} m "
          f"({fraction * 100:5.1f} % of the {perimeter:.1f} m perimeter)")
    print(f"    ending heading    {math.degrees(heading):+6.1f} deg")
    print(f"    distance          {driver.odometry.distance:6.3f} m")
    print(f"    slip index        {driver.odometry.slip_index:6.3f} rad/s")

    measured = ask_float("  Measured distance from the start mark, in metres (blank to skip): ")
    if measured is not None:
        print(f"    tape closure      {measured:6.3f} m "
              f"({measured / perimeter * 100:5.1f} % of the perimeter)")
        if measured / perimeter > CLOSURE_LIMIT:
            failures.append(
                f"closure error {measured / perimeter * 100:.1f} % exceeds "
                f"{CLOSURE_LIMIT * 100:.0f} %. Track width and steer_max_deg are the two "
                "parameters that move this; 5-15 % is inherent to dead reckoning off two "
                "wheels and cannot be tuned out"
            )
    if fraction > CLOSURE_LIMIT:
        failures.append(
            f"the odometry does not even close on itself ({fraction * 100:.1f} %), which is "
            "a geometry error rather than a slip problem"
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


def ask_float(message: str) -> float | None:
    try:
        answer = input(message).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return None
    if not answer:
        return None
    try:
        return float(answer)
    except ValueError:
        print("    not a number; skipping")
        return None


def confirm(target_rpm: float, speed: float) -> bool:
    print()
    print("  " + "=" * 68)
    print("   ON-GROUND TEST -- the car drives across the floor.")
    print(f"     - {target_rpm:.0f} RPM is about {speed:.2f} m/s")
    print("     - clear 3 m in every direction, and a hard flat surface")
    print("     - master switch reachable without chasing the car")
    print("     - Ctrl-C stops it and disables the bridge")
    print("  " + "=" * 68)
    if not sys.stdin.isatty():
        print("  no tty: refusing to drive the car unattended")
        return False
    try:
        return input("  Type GO to continue: ").strip().upper() == "GO"
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare dead reckoning against a tape measure.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--geometry", action="store_true", help="predictions only, no motion")
    parser.add_argument("--straight", type=float, default=None, metavar="M", help="straight leg")
    parser.add_argument("--circle", action="store_true", help="one full-lock circle")
    parser.add_argument("--square", type=float, default=None, metavar="M", help="square side")
    parser.add_argument(
        "--speed", type=float, default=0.25, help="fraction of max_rpm_measured to drive at"
    )
    parser.add_argument(
        "--steering", type=float, default=1.0, help="normalized steering for turns, -1..+1"
    )
    parser.add_argument("--timeout", type=float, default=30.0, help="per-leg time limit")
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
    moving = args.straight is not None or args.circle or args.square is not None
    if not moving and not args.geometry:
        args.geometry = True

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    calibration = DriveCalibration.load(args.calibration)
    if args.geometry and not moving:
        report_geometry(config, calibration)
        return 0

    if calibration is None or not calibration.is_measured:
        print(
            f"no usable calibration at {args.calibration}. Run "
            "scripts/calibrate_drive.py first -- without a measured top speed there "
            "is no way to ask for a sensible ground speed.",
            file=sys.stderr,
        )
        return 2

    target_rpm = max(1.0, args.speed * calibration.max_rpm_measured)
    speed = config.speed_for_rpm(target_rpm)
    report_geometry(config, calibration)
    if not confirm(target_rpm, speed):
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
    driver = None
    with PanicChain(panic):
        try:
            driver = Driver(gpio, config, calibration, clock)
            panic.attach(motors=driver.motors, servo=driver.servo)

            if args.straight is not None:
                failures.extend(run_straight(driver, args, target_rpm))
            if args.circle:
                failures.extend(run_circle(driver, args, target_rpm))
            if args.square is not None:
                failures.extend(run_square(driver, args, target_rpm))

            print("=" * 72)
            if failures:
                print("  FAIL")
                for problem in failures:
                    print(f"    - {problem}")
                status = 1
            else:
                print("  PASS")
                status = 0
            print("=" * 72)
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            status = 2
        finally:
            if driver is not None:
                driver.close()
            gpio.cleanup()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
