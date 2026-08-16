#!/usr/bin/env python3
"""Measure the firmware's own latency chain, one link at a time.

Three numbers decide whether a 10 ms control loop fits on this board, and none
of them can be reasoned about from first principles because every one is a round
trip to a daemon over a socket:

**What a GPIO call costs.** pigpio is a client library talking to ``pigpiod``
over a local socket. Every ``write``, every paired PWM update, every servo pulse
is a syscall pair. The control loop makes a fixed number of them per tick, and
that fixed number times the measured cost is the floor under the loop budget.

**How late an encoder edge arrives.** The callback carries pigpio's own
microsecond timestamp for when the edge *happened*; this measures how long after
that the Python callback actually ran. The M/T velocity estimator closes its
window on an edge, so delivery lag that approaches a control period makes the
estimate describe a moment that has already passed.

**What the loop body costs.** Everything above plus the arithmetic, per tick,
against the 6 ms budget -- which is deliberately well under the 10 ms period,
because the Pi also has a WiFi stack to run and that shows up as control packet
loss long before it shows up as a missed deadline.

Deliberately *not* here: glass-to-glass video latency and network round trip.
The first needs a phone stopwatch pointed at a screen (tuning.md section 5.1),
and the second belongs to the desktop app, which measures it from the
``echo_client_time_us`` the car returns in every telemetry packet.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
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
    CONTROL_PERIOD_S,
    ENCODER_GLITCH_US,
    LOCAL_CONFIG_PATH,
    LOOP_BUDGET_S,
    LOOP_P99_BUDGET_S,
    SERVO_NOMINAL_CENTER_US,
)
from telekart.control.mixer import DifferentialMixer  # noqa: E402
from telekart.control.pid import PID  # noqa: E402
from telekart.drivers.encoder import QuadratureEncoder  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.hal.base import Edge, GpioBackend, GpioError, Pull, select_backend  # noqa: E402
from telekart.util.clock import DeadlineScheduler, RealClock  # noqa: E402

#: An edge delivered later than this makes the M/T window describe a moment that
#: has already passed. A fifth of a control period is the point at which it
#: starts to matter.
EDGE_LAG_BUDGET_S = 0.002

#: How many of each GPIO call the 100 Hz loop makes per tick. Counted from the
#: control loop rather than guessed: two encoder samples (one backend tick read
#: each, plus one channel-B read), one paired PWM write, and up to four
#: direction writes on a tick that changes direction.
CALLS_PER_TICK = {
    "ticks_us": 2,
    "read": 2,
    "set_pwm_pair": 1,
    "write": 4,
    "set_servo_pulse": 1,
}


@dataclass(slots=True)
class Timing:
    name: str
    samples: list[float]

    def percentile(self, q: float) -> float:
        if not self.samples:
            return 0.0
        ordered = sorted(self.samples)
        index = min(len(ordered) - 1, max(0, int(q * len(ordered)) - 1))
        return ordered[index]

    @property
    def p50(self) -> float:
        return self.percentile(0.50)

    @property
    def p99(self) -> float:
        return self.percentile(0.99)

    @property
    def maximum(self) -> float:
        return max(self.samples) if self.samples else 0.0

    @property
    def mean(self) -> float:
        return statistics.fmean(self.samples) if self.samples else 0.0

    def line(self, scale: float = 1e6, unit: str = "us") -> str:
        return (
            f"    {self.name:<20} n={len(self.samples):<7} "
            f"mean {self.mean * scale:8.1f} {unit}   "
            f"p50 {self.p50 * scale:8.1f}   p99 {self.p99 * scale:8.1f}   "
            f"max {self.maximum * scale:8.1f}"
        )


def measure(name: str, call, iterations: int) -> Timing:  # noqa: ANN001 - any callable
    """Time one operation repeatedly.

    ``perf_counter`` rather than the injected clock: this is measuring the clock
    path itself, and the injected ``RealClock`` adds a Python frame to every
    reading that would be indistinguishable from the cost under test.
    """
    samples: list[float] = []
    counter = time.perf_counter
    for index in range(iterations):
        start = counter()
        call(index)
        samples.append(counter() - start)
    return Timing(name, samples)


# --------------------------------------------------------------------------
# GPIO call costs
# --------------------------------------------------------------------------


def measure_gpio(gpio: GpioBackend, config: VehicleConfig, iterations: int) -> list[Timing]:
    pins = config.pins.motors
    servo_pin = config.pins.servo
    encoder_pin = config.pins.encoders.left_a

    gpio.setup_output(pins.in1, False)
    gpio.setup_input(encoder_pin, Pull.UP, ENCODER_GLITCH_US)

    timings = [
        measure("ticks_us", lambda _i: gpio.ticks_us(), iterations),
        measure("read", lambda _i: gpio.read(encoder_pin), iterations),
        # Alternating levels: a backend that deduplicates identical writes would
        # otherwise be measured doing nothing at all.
        measure("write", lambda i: gpio.write(pins.in1, bool(i & 1)), iterations),
        measure(
            "set_pwm_pair",
            lambda i: gpio.set_pwm_pair(
                pins.ena, pins.enb, config.pwm_hz, 0.0, 0.001 * (i & 1)
            ),
            iterations,
        ),
        measure(
            "set_servo_pulse",
            lambda i: gpio.set_servo_pulse(
                servo_pin, SERVO_NOMINAL_CENTER_US + (i & 1) * 10
            ),
            iterations,
        ),
    ]
    gpio.set_pwm_pair(pins.ena, pins.enb, config.pwm_hz, 0.0, 0.0)
    gpio.set_servo_pulse(servo_pin, 0)
    gpio.write(pins.in1, False)
    return timings


def report_gpio(timings: list[Timing]) -> tuple[float, list[str]]:
    failures: list[str] = []
    print()
    print("  --- GPIO call cost ---")
    for timing in timings:
        print(timing.line())

    budget = 0.0
    print()
    print("  Per control tick, at the call counts the loop actually makes:")
    for timing in timings:
        count = CALLS_PER_TICK.get(timing.name, 0)
        if count == 0:
            continue
        cost = timing.p99 * count
        budget += cost
        print(f"    {count} x {timing.name:<18} {cost * 1e6:8.1f} us at p99")
    print(f"    {'total':<22} {budget * 1e6:8.1f} us of a "
          f"{LOOP_BUDGET_S * 1e6:.0f} us budget "
          f"({budget / LOOP_BUDGET_S * 100:.1f} %)")
    if budget > LOOP_BUDGET_S:
        failures.append(
            f"GPIO calls alone need {budget * 1e6:.0f} us per tick, over the "
            f"{LOOP_BUDGET_S * 1e6:.0f} us loop budget. Nothing else fits around them"
        )
    return budget, failures


# --------------------------------------------------------------------------
# Edge delivery lag
# --------------------------------------------------------------------------


class EdgeLag:
    """Time between an edge's own timestamp and the callback running.

    The two clocks have different epochs, so the offset between them is taken
    once, back to back, and applied to every sample. It drifts -- they are
    different clock sources -- but over a run of a few seconds the drift is
    orders of magnitude below the thing being measured.
    """

    def __init__(self, gpio: GpioBackend) -> None:
        self.samples: list[float] = []
        tick = gpio.ticks_us()
        self._offset_us = (time.monotonic_ns() // 1000) - tick

    def on_edge(self, _pin: int, _level: int, tick_us: int) -> None:
        now_us = time.monotonic_ns() // 1000
        lag_us = now_us - (tick_us + self._offset_us)
        # tick_us wraps at 2**32 while the monotonic clock does not, so fold the
        # difference back through the same wrap before believing it.
        if lag_us < -2_000_000_000:
            lag_us += 1 << 32
        elif lag_us > 2_000_000_000:
            lag_us -= 1 << 32
        if -1000 < lag_us < 1_000_000:
            self.samples.append(lag_us / 1e6)


def measure_edges(
    gpio: GpioBackend, config: VehicleConfig, motors: MotorPair | None, seconds: float, duty: float
) -> Timing:
    pin = config.pins.encoders.left_a
    gpio.setup_input(pin, Pull.UP, ENCODER_GLITCH_US)
    lag = EdgeLag(gpio)
    handle = gpio.add_edge_callback(pin, Edge.BOTH, lag.on_edge)

    clock = RealClock()
    pump = getattr(gpio, "step", None)
    deadline = clock.monotonic() + seconds
    dt = CONTROL_PERIOD_S
    try:
        while clock.monotonic() < deadline:
            if motors is not None:
                motors.drive(duty, 0.0)
            if pump is not None:
                pump(dt)
            clock.sleep(dt)
    finally:
        if motors is not None:
            motors.coast()
        handle.cancel()
    return Timing("edge delivery", lag.samples)


def report_edges(timing: Timing) -> list[str]:
    failures: list[str] = []
    print()
    print("  --- Encoder edge delivery lag ---")
    if not timing.samples:
        print("    no edges arrived: nothing was turning")
        return failures
    print(timing.line())
    if timing.p99 > EDGE_LAG_BUDGET_S:
        failures.append(
            f"edges arrive {timing.p99 * 1e3:.2f} ms late at p99, over the "
            f"{EDGE_LAG_BUDGET_S * 1e3:.0f} ms budget. The M/T window closes on an edge, "
            "so a late edge is a velocity estimate describing a moment that has passed"
        )
    return failures


# --------------------------------------------------------------------------
# Loop body cost
# --------------------------------------------------------------------------


def measure_loop(
    gpio: GpioBackend,
    config: VehicleConfig,
    motors: MotorPair,
    encoder_l: QuadratureEncoder,
    encoder_r: QuadratureEncoder,
    seconds: float,
) -> tuple[Timing, DeadlineScheduler]:
    """Time the per-tick work with the bridge disabled.

    Duty stays at zero throughout: this measures computation, and there is no
    reason to have the wheels turning to do it.
    """
    clock = RealClock()
    mixer = DifferentialMixer(config)
    pid_l = PID(config.pid_kp, config.pid_ki, config.pid_kd, i_clamp=config.pid_i_clamp)
    pid_r = PID(config.pid_kp, config.pid_ki, config.pid_kd, i_clamp=config.pid_i_clamp)
    scheduler = DeadlineScheduler(clock, CONTROL_PERIOD_S)
    scheduler.start()
    pump = getattr(gpio, "step", None)

    samples: list[float] = []
    counter = time.perf_counter
    deadline = clock.monotonic() + seconds
    while clock.monotonic() < deadline:
        dt = scheduler.wait_next()
        if pump is not None:
            pump(dt)
        start = counter()
        now_us = gpio.ticks_us()
        sample_l = encoder_l.sample(dt, now_us)
        sample_r = encoder_r.sample(dt, now_us)
        targets = mixer.mix(60.0, 0.0)
        pid_l.update(targets.rpm_l, sample_l.rpm, dt)
        pid_r.update(targets.rpm_r, sample_r.rpm, dt)
        motors.note_speed(sample_l.rpm, sample_r.rpm)
        motors.drive(0.0, 0.0)
        samples.append(counter() - start)
    return Timing("loop body", samples), scheduler


def report_loop(timing: Timing, scheduler: DeadlineScheduler) -> list[str]:
    failures: list[str] = []
    stats = scheduler.stats.snapshot()
    print()
    print("  --- Loop body ---")
    print(timing.line())
    print(
        f"    {'period':<20} n={stats.count:<7} "
        f"mean {stats.mean * 1e3:8.3f} ms   p50 {stats.p50 * 1e3:8.3f}   "
        f"p99 {stats.p99 * 1e3:8.3f}   max {stats.max * 1e3:8.3f}"
    )
    print(f"    overruns {scheduler.overruns}, skipped deadlines {scheduler.skipped}")

    if timing.p99 > LOOP_BUDGET_S:
        failures.append(
            f"the loop body needs {timing.p99 * 1e3:.2f} ms at p99, over the "
            f"{LOOP_BUDGET_S * 1e3:.0f} ms budget. What is left is the headroom the WiFi "
            "stack needs, and losing it shows up as control packet loss first"
        )
    if stats.p99 > LOOP_P99_BUDGET_S:
        failures.append(
            f"loop period p99 is {stats.p99 * 1e3:.2f} ms, over the "
            f"{LOOP_P99_BUDGET_S * 1e3:.0f} ms limit. Check SCHED_FIFO is actually applied "
            "and that nothing else is pinned to the same core"
        )
    return failures


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Measure GPIO call cost, encoder delivery lag and loop body cost.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--gpio", action="store_true", help="GPIO call costs only")
    parser.add_argument("--edges", action="store_true", help="edge delivery lag only")
    parser.add_argument("--loop", action="store_true", help="loop body cost only")
    parser.add_argument(
        "--iterations", type=int, default=2000, help="samples per GPIO operation"
    )
    parser.add_argument("--seconds", type=float, default=5.0, help="duration of the timed runs")
    parser.add_argument(
        "--duty",
        type=float,
        default=0.0,
        help="duty on the left motor during the edge test; 0 means spin it by hand",
    )
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    return parser


def load_config(path: Path | None) -> VehicleConfig:
    if path is not None:
        return VehicleConfig.load(path, LOCAL_CONFIG_PATH)
    return VehicleConfig.load_default()


def open_backend(name: str, clock: RealClock) -> GpioBackend:
    """Hand the mock the loop's clock so `--backend mock` is a real dry run."""
    gpio = select_backend(name)
    if getattr(gpio, "step", None) is not None:
        gpio.clock = clock
    return gpio


def confirm(duty: float) -> bool:
    print()
    print("  " + "=" * 66)
    print(f"   The edge test drives the LEFT motor at {duty:.2f} duty.")
    print("     - wheels off the ground")
    print("     - safety glasses on")
    print("  " + "=" * 66)
    if not sys.stdin.isatty():
        print("  no tty: refusing to drive the motor unattended")
        return False
    try:
        return input("  Type GO to continue: ").strip().upper() == "GO"
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    everything = not (args.gpio or args.edges or args.loop)
    do_gpio = everything or args.gpio
    do_edges = everything or args.edges
    do_loop = everything or args.loop

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    duty = min(abs(args.duty), config.max_duty)
    if do_edges and duty > 0.0 and not confirm(duty):
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
        try:
            print()
            print(f"  backend {type(gpio).__name__}   period "
                  f"{CONTROL_PERIOD_S * 1e3:.1f} ms   budget {LOOP_BUDGET_S * 1e3:.1f} ms")
            if do_gpio:
                # Worth stating: this section writes real pins. The enables never
                # exceed 0.1 % duty, which is two orders of magnitude below the
                # ~22 % a Darlington bridge needs before it conducts at all, so
                # nothing moves even with the battery connected.
                print("  the GPIO section toggles IN1 and writes the enables at <=0.1 % duty")

            if do_gpio:
                _budget, problems = report_gpio(
                    measure_gpio(gpio, config, args.iterations)
                )
                failures.extend(problems)

            motors = MotorPair(gpio, config.pins.motors, config, clock)
            panic.attach(motors=motors)

            if do_edges:
                if duty <= 0.0:
                    print()
                    print(f"  Spin the LEFT wheel by hand for the next {args.seconds:g} s.")
                    if sys.stdin.isatty():
                        try:
                            input("  Press Enter to start. ")
                        except (EOFError, KeyboardInterrupt):
                            print()
                            return 2
                failures.extend(
                    report_edges(
                        measure_edges(
                            gpio,
                            config,
                            motors if duty > 0.0 else None,
                            args.seconds,
                            duty,
                        )
                    )
                )

            if do_loop:
                encoder_l = QuadratureEncoder(
                    gpio,
                    config.pins.encoders.left_a,
                    config.pins.encoders.left_b,
                    cpr=config.encoder_cpr,
                )
                encoder_r = QuadratureEncoder(
                    gpio,
                    config.pins.encoders.right_a,
                    config.pins.encoders.right_b,
                    cpr=config.encoder_cpr,
                )
                timing, scheduler = measure_loop(
                    gpio, config, motors, encoder_l, encoder_r, args.seconds
                )
                failures.extend(report_loop(timing, scheduler))

            print()
            print("=" * 78)
            if failures:
                print("  FAIL")
                for problem in failures:
                    print(f"    - {problem}")
                status = 1
            else:
                print("  PASS -- every link in the local latency chain is inside budget.")
                print("  Video latency is measured separately, with a phone stopwatch:")
                print("  tuning.md section 5.1. Network round trip comes from the app,")
                print("  out of echo_client_time_us in every telemetry packet.")
                status = 0
            print("=" * 78)
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            status = 2
        finally:
            gpio.cleanup()
    return status


if __name__ == "__main__":
    raise SystemExit(main())
