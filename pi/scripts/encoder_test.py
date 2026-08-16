#!/usr/bin/env python3
"""Verify the encoders and measure the real counts per revolution.

``encoder_cpr`` is A-channel edges per revolution of the *output shaft, as this
decoder counts them* -- x2, either edge on channel A only. It is not "quadrature
counts", not the hall PPR, and not the gear ratio times anything. Measure it;
do not compute it.

The reason is that the ratio printed on a GA37-520 gearbox is a rounded
marketing figure. A "1:30" is frequently 29.86:1. Half a percent sounds like
nothing -- it is 25 mm of odometry error over 5 m -- but the same half percent
biases **every RPM reading and therefore every PID setpoint**, permanently, in a
way that looks exactly like a tuning problem and is not one.

Two modes:

``--hand``      Motor leads unscrewed, battery off, powerbank only. Turn each
                wheel exactly ``--revs`` revolutions by hand and the count is
                the answer. Ten revolutions rather than one because the mark
                alignment error is a fixed few counts and ten revolutions
                amortise it tenfold.

``--powered``   Wheels off the ground, battery on. Drives one wheel at a time at
                low duty and checks that the right encoder counted, that the
                other one did not, and that the implied speed is plausible.
"""

from __future__ import annotations

import argparse
import statistics
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
from telekart.constants import ENCODER_GLITCH_US, LOCAL_CONFIG_PATH  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.hal.base import Edge, GpioBackend, GpioError, Pull, select_backend  # noqa: E402
from telekart.util.clock import RealClock  # noqa: E402

#: Per calibration.md section 3: three trials on one wheel must agree to this,
#: and the two wheels to half of it. A wider left/right spread means the two
#: gearboxes are not the same ratio, which a single global encoder_cpr cannot
#: represent -- worth discovering here rather than in the closed-loop phase.
TRIAL_TOLERANCE = 0.02
WHEEL_TOLERANCE = 0.01

#: Duty for the powered checks. Above the ~0.22 Darlington deadband and nothing
#: like enough to matter if something is wired wrong.
DEFAULT_POWERED_DUTY = 0.30


@dataclass(slots=True)
class Channel:
    label: str
    pin: int
    count: int = 0


@dataclass(slots=True)
class WheelResult:
    wheel: str
    trials: list[float] = field(default_factory=list)

    @property
    def mean(self) -> float:
        return statistics.fmean(self.trials) if self.trials else 0.0

    @property
    def spread(self) -> float:
        """Peak-to-peak as a fraction of the mean."""
        if len(self.trials) < 2 or self.mean <= 0.0:
            return 0.0
        return (max(self.trials) - min(self.trials)) / self.mean


class EdgeCounter:
    """Raw edge counting on all four channels.

    Deliberately not :class:`~telekart.drivers.encoder.QuadratureEncoder`: this
    script exists to check the *wiring*, and it has to work on a car whose
    encoder driver has never been run.
    """

    def __init__(self, gpio: GpioBackend, config: VehicleConfig, glitch_us: int) -> None:
        pins = config.pins.encoders
        self.channels = {
            "left_a": Channel("L_A", pins.left_a),
            "left_b": Channel("L_B", pins.left_b),
            "right_a": Channel("R_A", pins.right_a),
            "right_b": Channel("R_B", pins.right_b),
        }
        self._by_pin = {channel.pin: channel for channel in self.channels.values()}
        self._handles = []
        for channel in self.channels.values():
            gpio.setup_input(channel.pin, Pull.UP, glitch_us)
            self._handles.append(gpio.add_edge_callback(channel.pin, Edge.BOTH, self._on_edge))

    def _on_edge(self, pin: int, _level: int, _tick_us: int) -> None:
        channel = self._by_pin.get(pin)
        if channel is not None:
            channel.count += 1

    def snapshot(self) -> dict[str, int]:
        return {name: channel.count for name, channel in self.channels.items()}

    def close(self) -> None:
        for handle in self._handles:
            handle.cancel()
        self._handles.clear()

    def line(self) -> str:
        return "  ".join(
            f"{channel.label}={channel.count:6d}" for channel in self.channels.values()
        )


# --------------------------------------------------------------------------
# Hand mode
# --------------------------------------------------------------------------


def wait(message: str) -> bool:
    try:
        input(f"  {message}")
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    return True


def hand_trial(counter: EdgeCounter, wheel: str, revs: float, direction: str) -> float | None:
    """One hand-turned trial. Returns counts per revolution, or None if aborted."""
    key = f"{wheel}_a"
    if not wait(f"Mark the {wheel} wheel and the chassis, then press Enter. "):
        return None
    before = counter.snapshot()[key]
    print(
        f"      Now turn the {wheel} wheel exactly {revs:g} revolutions {direction}, "
        "slowly -- under one turn a second."
    )
    if not wait("Press Enter when the marks line up again. "):
        return None
    after = counter.snapshot()[key]

    delta = after - before
    if delta <= 0:
        print("      No counts at all. That channel is not wired, or the encoder")
        print("      is 5 V-only and needs its own supply (wiring.md section 9).")
        return None
    cpr = delta / revs
    print(f"      {delta} edges over {revs:g} revolutions  ->  {cpr:.1f} counts/rev")
    return cpr


def run_hand(counter: EdgeCounter, args: argparse.Namespace, config: VehicleConfig) -> int:
    wheels = ("left", "right") if args.wheel == "both" else (args.wheel,)
    results: dict[str, WheelResult] = {wheel: WheelResult(wheel) for wheel in wheels}
    failures: list[str] = []

    print()
    print("  Motor leads unscrewed. Motor battery off. Powerbank only.")
    print()

    for wheel in wheels:
        print(f"  --- {wheel} wheel, {args.trials} trials ---")
        for trial in range(args.trials):
            print(f"    trial {trial + 1}/{args.trials}")
            cpr = hand_trial(counter, wheel, args.revs, "in one direction")
            if cpr is None:
                failures.append(f"{wheel}: trial {trial + 1} produced no usable count")
                break
            results[wheel].trials.append(cpr)

        if args.reverse and results[wheel].trials:
            print("    reverse check -- the decoder is direction-blind by design,")
            print("    so turning the other way must give the SAME count")
            reverse_cpr = hand_trial(counter, wheel, args.revs, "the OTHER way")
            if reverse_cpr is None:
                failures.append(f"{wheel}: the reverse trial produced no usable count")
            else:
                error = abs(reverse_cpr - results[wheel].mean) / max(results[wheel].mean, 1.0)
                if error > TRIAL_TOLERANCE:
                    failures.append(
                        f"{wheel}: reverse count differs by {error * 100:.1f} % "
                        "-- x2 decoding on channel A alone cannot be directional, "
                        "so this means the turns were not equal"
                    )
                else:
                    print(f"      matches to {error * 100:.1f} %")
        print()

    print("=" * 68)
    for wheel, result in results.items():
        if not result.trials:
            continue
        print(
            f"  {wheel:>5}: mean {result.mean:.1f} counts/rev over "
            f"{len(result.trials)} trials, spread {result.spread * 100:.1f} %"
        )
        if result.spread > TRIAL_TOLERANCE:
            failures.append(
                f"{wheel}: trials spread {result.spread * 100:.1f} %, "
                f"over the {TRIAL_TOLERANCE * 100:.0f} % limit"
            )

    if len(results) == 2 and all(result.trials for result in results.values()):
        left = results["left"].mean
        right = results["right"].mean
        disagreement = abs(left - right) / max((left + right) / 2.0, 1.0)
        print(f"  left vs right: {disagreement * 100:.1f} %")
        if disagreement > WHEEL_TOLERANCE:
            failures.append(
                f"left and right disagree by {disagreement * 100:.1f} %, over the "
                f"{WHEEL_TOLERANCE * 100:.0f} % limit -- the two gearboxes are not "
                "the same ratio, and one global encoder_cpr cannot describe both"
            )

    measured = [result.mean for result in results.values() if result.trials]
    if measured:
        suggestion = int(round(statistics.fmean(measured)))
        print()
        print(f"  Measured encoder_cpr: {suggestion}   (config says {config.encoder_cpr})")
        print("  Put it in pi/config/config.local.yaml:")
        print()
        print("    params:")
        print(f"      encoder_cpr: {suggestion}")
        print()
        print(f"  Resolution: pi * {config.wheel_diameter_m:.3f} / {suggestion} = "
              f"{3.14159265 * config.wheel_diameter_m / suggestion * 1000:.3f} mm per count")

    return report_failures(failures)


def run_noise(counter: EdgeCounter, seconds: float) -> int:
    """Any count at rest is noise pickup, and noise corrupts odometry silently."""
    print()
    print(f"  Watching all four channels for {seconds:g} s. Do not touch the car.")
    before = counter.snapshot()
    RealClock().sleep(seconds)
    after = counter.snapshot()

    noisy = {name: after[name] - before[name] for name in after if after[name] != before[name]}
    if not noisy:
        print(f"  Zero counts in {seconds:g} s on all four channels.")
        return report_failures([])
    detail = ", ".join(f"{name}+{delta}" for name, delta in noisy.items())
    return report_failures(
        [
            f"counts arrived while stationary: {detail}. That is noise pickup -- "
            "check the encoder ground return (wiring.md section 5) and the 100 nF "
            "decoupling cap at the encoder end (section 6)"
        ]
    )


# --------------------------------------------------------------------------
# Powered mode
# --------------------------------------------------------------------------


def run_powered(
    counter: EdgeCounter,
    motors: MotorPair,
    config: VehicleConfig,
    args: argparse.Namespace,
) -> int:
    clock = RealClock()
    failures: list[str] = []
    duty = min(abs(args.duty), 0.60)

    print()
    print("  Wheels off the ground. Safety glasses. Hand on the master switch.")
    print(f"  Each burst is {args.seconds:g} s at {duty:.2f} duty, one wheel at a time.")
    print()

    for wheel in ("left", "right"):
        for direction, sign in (("forward", 1.0), ("reverse", -1.0)):
            label = f"{wheel} {direction}"
            before = counter.snapshot()
            start = clock.monotonic()

            deadline = start + args.seconds
            while clock.monotonic() < deadline:
                command = duty * sign
                motors.drive(command if wheel == "left" else 0.0,
                             0.0 if wheel == "left" else command)
                clock.sleep(0.010)
            motors.coast()
            elapsed = clock.monotonic() - start
            clock.sleep(0.5)  # let it spin down before the next burst

            after = counter.snapshot()
            own = after[f"{wheel}_a"] - before[f"{wheel}_a"]
            other = "right" if wheel == "left" else "left"
            cross = after[f"{other}_a"] - before[f"{other}_a"]
            own_b = after[f"{wheel}_b"] - before[f"{wheel}_b"]

            rate = own / elapsed if elapsed > 0.0 else 0.0
            rpm = rate * 60.0 / config.encoder_cpr
            print(
                f"  {label:>14}: {own:6d} edges  {rate:7.0f} edges/s  "
                f"~{rpm:6.1f} RPM   (B channel {own_b:6d}, other wheel {cross:4d})"
            )

            if own <= 0:
                failures.append(
                    f"{label}: the commanded wheel produced no counts. Either the "
                    "motor did not turn or that encoder is not wired"
                )
            if own_b <= 0:
                failures.append(
                    f"{label}: channel B never toggled. The firmware polls B as a "
                    "direction-disagreement check and will report it dead"
                )
            if cross > own * 0.05:
                failures.append(
                    f"{label}: the OTHER wheel counted {cross} edges. Encoder "
                    "channels are crossed, or both encoders share a signal wire"
                )

    print()
    print("  Bursts complete. Compare the implied RPM against the 120-260 band")
    print("  that calibrate_drive expects at full duty: well above it here at")
    print(f"  {duty:.2f} duty means encoder_cpr is probably wrong by an integer factor.")
    return report_failures(failures)


# --------------------------------------------------------------------------
# Shared
# --------------------------------------------------------------------------


def report_failures(failures: list[str]) -> int:
    print("=" * 68)
    if not failures:
        print("  PASS")
        print("=" * 68)
        return 0
    print("  FAIL")
    for problem in failures:
        print(f"    - {problem}")
    print("=" * 68)
    return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Verify the encoders and measure the real counts per revolution.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--hand", action="store_true", help="hand-turned cpr measurement")
    parser.add_argument("--powered", action="store_true", help="short powered bursts")
    parser.add_argument("--noise", action="store_true", help="count edges at rest only")
    parser.add_argument("--revs", type=float, default=10.0, help="revolutions per trial")
    parser.add_argument("--trials", type=int, default=3, help="trials per wheel")
    parser.add_argument(
        "--wheel", choices=("left", "right", "both"), default="both", help="which wheel"
    )
    parser.add_argument(
        "--reverse", action="store_true", help="also turn the other way and compare"
    )
    parser.add_argument(
        "--noise-seconds", type=float, default=30.0, help="stationary watch duration"
    )
    parser.add_argument(
        "--duty", type=float, default=DEFAULT_POWERED_DUTY, help="powered burst duty"
    )
    parser.add_argument("--seconds", type=float, default=1.5, help="powered burst duration")
    parser.add_argument(
        "--glitch-us", type=int, default=ENCODER_GLITCH_US, help="daemon-side glitch filter"
    )
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    return parser


def load_config(path: Path | None) -> VehicleConfig:
    if path is not None:
        return VehicleConfig.load(path, LOCAL_CONFIG_PATH)
    return VehicleConfig.load_default()


def confirm_powered() -> bool:
    print()
    print("  " + "=" * 66)
    print("   Powered mode drives the motors. Before you continue:")
    print("     - wheels off the ground, 20 mm of air under every tyre")
    print("     - safety glasses on")
    print("     - master switch reachable without moving your feet")
    print("  " + "=" * 66)
    if not sys.stdin.isatty():
        print("  no tty: refusing to run powered mode unattended")
        return False
    try:
        return input("  Type GO to continue: ").strip().upper() == "GO"
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    modes = sum(bool(flag) for flag in (args.hand, args.powered, args.noise))
    if modes != 1:
        print("choose exactly one of --hand, --powered or --noise", file=sys.stderr)
        return 2
    if args.hand and not sys.stdin.isatty():
        print("--hand needs an operator to turn the wheel", file=sys.stderr)
        return 2
    if args.powered and not confirm_powered():
        print("  Aborted. Nothing was driven.")
        return 2

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    try:
        gpio = select_backend(args.backend)
    except GpioError as exc:
        print(f"cannot open the GPIO backend: {exc}", file=sys.stderr)
        return 2

    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    counter = EdgeCounter(gpio, config, args.glitch_us)
    status = 1
    with PanicChain(panic):
        try:
            if args.hand:
                status = run_hand(counter, args, config)
            elif args.noise:
                status = run_noise(counter, args.noise_seconds)
            else:
                motors = MotorPair(gpio, config.pins.motors, config, RealClock())
                panic.attach(motors=motors)
                try:
                    status = run_powered(counter, motors, config, args)
                finally:
                    motors.coast()
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            status = 2
        finally:
            counter.close()
            gpio.cleanup()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
