#!/usr/bin/env python3
"""Measure what the encoders actually cost, and decide whether x2 decoding holds.

This is the phase that decides whether the whole decode design works on this
hardware. If it fails here, no amount of PID tuning helps.

The arithmetic under test::

    edges per second, per wheel  =  encoder_cpr * RPM / 60
    at 660 cpr and 200 RPM       =  2200 /s per wheel
    both wheels                  =  4400 /s

    each edge is one pigpio notification, one socket wakeup, and one Python
    callback whose body is `count += 1; last_tick = tick`

Every one of those crosses into the interpreter and takes the GIL. The question
is not whether that costs something -- it is whether it costs less than the
budget, and the only way to know is to measure it on the board.

What comes out:

* edges per second, per wheel, measured
* this process's CPU, from ``resource.getrusage``
* **per-thread** CPU from ``/proc/self/task/*/stat`` -- which is how you tell
  the notification thread's share from the control loop's, and they are very
  different numbers
* pigpiod's own CPU, from ``/proc/<pid>/stat``
* control-loop period p50/p95/p99/max
* microseconds of CPU per edge, both gross and marginal against an idle
  baseline, and a projection to the 4400 edges/s design point

Hard gates, from bringup.md phase 5:

* loop **p99 <= 12 ms**. A 10 ms loop whose p99 is above that is dropping a
  command every hundred ticks, and the mean will never tell you.
* **pigpiod + this process < 60 %** of one core.
* pigpiod alone **< 25 %** of one core.

If it fails, there are two levers and they go in this order: raise the encoder
glitch filter (every edge it suppresses is a callback you do not pay for --
watch that the total edge count does not fall, or you are eating real edges),
then switch the CPU governor to performance. Dropping to x1 decoding is not a
lever: it costs a pigpiod socket round trip of 50-100 us *per edge* to read
channel B, which is 100-200 % of a core. It is strictly worse.
"""

from __future__ import annotations

import argparse
import json
import os
import resource
import sys
import threading
from dataclasses import dataclass, field
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
    LOOP_OVERRUN_FAULT_COUNT,
    LOOP_P99_BUDGET_S,
)
from telekart.control.mixer import DifferentialMixer  # noqa: E402
from telekart.control.pid import PID  # noqa: E402
from telekart.drivers.encoder import QuadratureEncoder  # noqa: E402
from telekart.drivers.motor import MotorPair  # noqa: E402
from telekart.hal.base import GpioError, select_backend  # noqa: E402
from telekart.util.clock import DeadlineScheduler, RealClock  # noqa: E402

#: The design point this measurement has to be judged against, not the speed the
#: bench run happened to reach. 200 RPM on both wheels at 660 cpr.
DESIGN_EDGES_PER_S = 4400.0

#: Gate values, restated here as numbers so the report is self-contained.
CPU_TOTAL_GATE = 0.60
CPU_PIGPIOD_GATE = 0.25

_CLK_TCK = float(os.sysconf("SC_CLK_TCK")) if hasattr(os, "sysconf") else 100.0


def open_backend(name: str, clock: RealClock):  # noqa: ANN201 - GpioBackend
    """Open the backend, and hand the mock the loop's clock.

    Only the mock has a clock to hand it. Doing so makes ``--backend mock`` a
    real dry run of this harness -- synthesised edges arrive with timestamps in
    the same time base the encoder driver reads -- rather than a run that
    exercises the reporting and none of the measurement.
    """
    gpio = select_backend(name)
    if getattr(gpio, "step", None) is not None:
        gpio.clock = clock
    return gpio


# --------------------------------------------------------------------------
# CPU accounting
# --------------------------------------------------------------------------


def read_stat_cpu(path: str) -> float | None:
    """utime + stime from a /proc stat file, in seconds.

    The comm field is parenthesised and may contain spaces, so everything is
    parsed relative to the *last* closing parenthesis rather than by splitting
    the whole line -- a process called "my prog" would otherwise shift every
    field after it.
    """
    try:
        raw = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    close = raw.rfind(")")
    if close < 0:
        return None
    fields = raw[close + 2 :].split()
    if len(fields) < 13:
        return None
    try:
        return (int(fields[11]) + int(fields[12])) / _CLK_TCK
    except (ValueError, IndexError):
        return None


def find_pigpiod() -> int | None:
    proc = Path("/proc")
    if not proc.is_dir():
        return None
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8", errors="replace").strip()
        except OSError:
            continue
        if comm == "pigpiod":
            return int(entry.name)
    return None


def thread_names() -> dict[int, str]:
    """Native thread id to Python thread name.

    CPython does not push thread names down to the kernel on this version, so
    ``comm`` in /proc reads the same for every thread. Going the other way --
    asking Python which of its threads owns each native id -- is what makes the
    per-thread table readable, and it is exactly the notification thread's row
    that this whole measurement is about.
    """
    names: dict[int, str] = {}
    for thread in threading.enumerate():
        native = getattr(thread, "native_id", None)
        if native is not None:
            names[int(native)] = thread.name
    return names


@dataclass(slots=True)
class CpuSample:
    wall: float
    process: float
    pigpiod: float | None
    threads: dict[int, float] = field(default_factory=dict)


class CpuSampler:
    """Differences of counters, which is the only honest way to read them."""

    def __init__(self, clock: RealClock) -> None:
        self._clock = clock
        self._pigpiod_pid = find_pigpiod()

    @property
    def pigpiod_pid(self) -> int | None:
        return self._pigpiod_pid

    def sample(self) -> CpuSample:
        usage = resource.getrusage(resource.RUSAGE_SELF)
        pigpiod = None
        if self._pigpiod_pid is not None:
            pigpiod = read_stat_cpu(f"/proc/{self._pigpiod_pid}/stat")

        threads: dict[int, float] = {}
        task_dir = Path("/proc/self/task")
        if task_dir.is_dir():
            for entry in task_dir.iterdir():
                if not entry.name.isdigit():
                    continue
                cpu = read_stat_cpu(str(entry / "stat"))
                if cpu is not None:
                    threads[int(entry.name)] = cpu

        return CpuSample(
            wall=self._clock.monotonic(),
            process=usage.ru_utime + usage.ru_stime,
            pigpiod=pigpiod,
            threads=threads,
        )


@dataclass(slots=True)
class PhaseResult:
    name: str
    seconds: float
    ticks: int
    overruns: int
    edges_l: int
    edges_r: int
    process_cpu: float
    pigpiod_cpu: float | None
    thread_cpu: dict[int, float]
    p50: float
    p95: float
    p99: float
    maximum: float
    rpm_l: float
    rpm_r: float

    @property
    def edges(self) -> int:
        return self.edges_l + self.edges_r

    @property
    def edge_rate(self) -> float:
        return self.edges / self.seconds if self.seconds > 0.0 else 0.0

    @property
    def process_fraction(self) -> float:
        return self.process_cpu / self.seconds if self.seconds > 0.0 else 0.0

    @property
    def pigpiod_fraction(self) -> float:
        if self.pigpiod_cpu is None or self.seconds <= 0.0:
            return 0.0
        return self.pigpiod_cpu / self.seconds

    @property
    def total_fraction(self) -> float:
        return self.process_fraction + self.pigpiod_fraction

    @property
    def total_cpu(self) -> float:
        return self.process_cpu + (self.pigpiod_cpu or 0.0)


# --------------------------------------------------------------------------
# The loop under measurement
# --------------------------------------------------------------------------


class BenchLoop:
    """A control loop body with the same per-tick encoder work as the real one.

    Not the real ``DriveController``: this has to run before the safety state
    machine will arm anything, and mixing an arming failure into a CPU
    measurement helps nobody. What it does reproduce exactly is the part being
    measured -- two ``QuadratureEncoder.sample`` calls, the channel-B poll they
    each do, a mixer, two PIDs and one paired PWM write per tick.
    """

    def __init__(
        self,
        gpio: object,
        config: VehicleConfig,
        motors: MotorPair,
        encoder_l: QuadratureEncoder,
        encoder_r: QuadratureEncoder,
        clock: RealClock,
    ) -> None:
        self.gpio = gpio
        self._config = config
        self._motors = motors
        self._encoder_l = encoder_l
        self._encoder_r = encoder_r
        self._clock = clock
        self._mixer = DifferentialMixer(config)
        self._pid_l = PID(config.pid_kp, config.pid_ki, config.pid_kd, i_clamp=config.pid_i_clamp)
        self._pid_r = PID(config.pid_kp, config.pid_ki, config.pid_kd, i_clamp=config.pid_i_clamp)
        self.rpm_l = 0.0
        self.rpm_r = 0.0

    def tick(self, dt: float, duty: float) -> None:
        now_us = self._clock.monotonic_us() & 0xFFFFFFFF
        sample_l = self._encoder_l.sample(dt, now_us)
        sample_r = self._encoder_r.sample(dt, now_us)
        self.rpm_l = sample_l.rpm
        self.rpm_r = sample_r.rpm

        # The arithmetic the real loop does with those samples, so the tick cost
        # is not flattered by leaving it out.
        targets = self._mixer.mix(120.0, 0.0)
        self._pid_l.update(targets.rpm_l, sample_l.rpm, dt)
        self._pid_r.update(targets.rpm_r, sample_r.rpm, dt)

        self._motors.note_speed(sample_l.rpm, sample_r.rpm)
        sign = 1 if duty >= 0.0 else -1
        self._encoder_l.set_direction_hint(sign if duty != 0.0 else 0)
        self._encoder_r.set_direction_hint(sign if duty != 0.0 else 0)
        self._motors.drive(duty, duty)


def run_phase(
    name: str,
    loop: BenchLoop,
    sampler: CpuSampler,
    clock: RealClock,
    *,
    seconds: float,
    duty: float,
    period: float,
    encoder_l: QuadratureEncoder,
    encoder_r: QuadratureEncoder,
) -> PhaseResult:
    print(f"  {name}: {seconds:g} s at duty {duty:.2f} ...", flush=True)

    scheduler = DeadlineScheduler(clock, period)
    scheduler.start()
    edges_l_before = encoder_l.total_edges
    edges_r_before = encoder_r.total_edges
    before = sampler.sample()

    # Present only on MockBackend. On the real car the plant is the car and the
    # edges arrive on pigpio's notification thread.
    pump = getattr(loop.gpio, "step", None)

    ticks = 0
    deadline = clock.monotonic() + seconds
    while clock.monotonic() < deadline:
        dt = scheduler.wait_next()
        if pump is not None:
            pump(dt)
        loop.tick(dt, duty)
        ticks += 1

    after = sampler.sample()
    stats = scheduler.stats.snapshot()
    elapsed = after.wall - before.wall

    thread_cpu = {
        tid: after.threads[tid] - before.threads.get(tid, 0.0) for tid in after.threads
    }
    pigpiod_cpu = None
    if before.pigpiod is not None and after.pigpiod is not None:
        pigpiod_cpu = after.pigpiod - before.pigpiod

    return PhaseResult(
        name=name,
        seconds=elapsed,
        ticks=ticks,
        overruns=scheduler.overruns,
        edges_l=encoder_l.total_edges - edges_l_before,
        edges_r=encoder_r.total_edges - edges_r_before,
        process_cpu=after.process - before.process,
        pigpiod_cpu=pigpiod_cpu,
        thread_cpu=thread_cpu,
        p50=stats.p50,
        p95=stats.p95,
        p99=stats.p99,
        maximum=stats.max,
        rpm_l=loop.rpm_l,
        rpm_r=loop.rpm_r,
    )


# --------------------------------------------------------------------------
# Reporting
# --------------------------------------------------------------------------


def print_phase(phase: PhaseResult, names: dict[int, str]) -> None:
    print()
    print(f"  --- {phase.name} ---")
    print(f"    wall           {phase.seconds:8.2f} s over {phase.ticks} ticks")
    print(
        f"    edges          left {phase.edges_l:8d}  right {phase.edges_r:8d}  "
        f"total {phase.edges:8d}"
    )
    print(
        f"    edge rate      left {phase.edges_l / max(phase.seconds, 1e-9):8.0f}/s  "
        f"right {phase.edges_r / max(phase.seconds, 1e-9):8.0f}/s  "
        f"total {phase.edge_rate:8.0f}/s"
    )
    print(f"    measured rpm   left {phase.rpm_l:8.1f}    right {phase.rpm_r:8.1f}")
    print(
        f"    loop period    p50 {phase.p50 * 1e3:6.3f} ms  p95 {phase.p95 * 1e3:6.3f} ms  "
        f"p99 {phase.p99 * 1e3:6.3f} ms  max {phase.maximum * 1e3:6.3f} ms"
    )
    print(f"    overruns       {phase.overruns}")
    print(
        f"    process cpu    {phase.process_cpu:8.3f} s  "
        f"({phase.process_fraction * 100:5.1f} % of one core)"
    )
    if phase.pigpiod_cpu is None:
        print("    pigpiod cpu    unavailable (no /proc, or pigpiod is not running)")
    else:
        print(
            f"    pigpiod cpu    {phase.pigpiod_cpu:8.3f} s  "
            f"({phase.pigpiod_fraction * 100:5.1f} % of one core)"
        )

    if phase.thread_cpu:
        print("    per-thread cpu:")
        for tid, cpu in sorted(phase.thread_cpu.items(), key=lambda item: -item[1]):
            if cpu <= 0.0005:
                continue
            label = names.get(tid, "unnamed")
            share = cpu / max(phase.seconds, 1e-9) * 100.0
            print(f"      tid {tid:<8} {label:<22} {cpu:7.3f} s  {share:5.1f} %")


def print_gates(loaded: PhaseResult, baseline: PhaseResult | None) -> int:
    print()
    print("=" * 72)
    print("  GATES")

    failures = 0

    def gate(name: str, ok: bool, detail: str) -> None:
        nonlocal failures
        if not ok:
            failures += 1
        print(f"    [{'PASS' if ok else 'FAIL'}]  {name:<34} {detail}")

    gate(
        "loop p99 <= 12 ms",
        loaded.p99 <= LOOP_P99_BUDGET_S,
        f"p99 = {loaded.p99 * 1e3:.3f} ms",
    )
    gate(
        f"total CPU < {CPU_TOTAL_GATE * 100:.0f} % of one core",
        loaded.total_fraction < CPU_TOTAL_GATE,
        f"total = {loaded.total_fraction * 100:.1f} %",
    )
    if loaded.pigpiod_cpu is None:
        print("    [skip]  pigpiod < 25 % of one core       not measurable here")
    else:
        gate(
            f"pigpiod < {CPU_PIGPIOD_GATE * 100:.0f} % of one core",
            loaded.pigpiod_fraction < CPU_PIGPIOD_GATE,
            f"pigpiod = {loaded.pigpiod_fraction * 100:.1f} %",
        )
    gate(
        f"overruns < {LOOP_OVERRUN_FAULT_COUNT}",
        loaded.overruns < LOOP_OVERRUN_FAULT_COUNT,
        f"overruns = {loaded.overruns}",
    )

    print()
    print("  DERIVED")
    if loaded.edges > 0:
        gross = loaded.total_cpu / loaded.edges * 1e6
        print(f"    gross CPU per edge            {gross:8.2f} us")
    else:
        print("    gross CPU per edge            no edges: nothing turned")

    if baseline is not None and loaded.edges > baseline.edges:
        extra_edges = loaded.edges - baseline.edges
        extra_cpu = loaded.total_cpu - baseline.total_cpu * (loaded.seconds / max(baseline.seconds, 1e-9))
        marginal = extra_cpu / extra_edges * 1e6
        print(f"    marginal CPU per edge         {marginal:8.2f} us")
        projected = (
            baseline.total_fraction + marginal * 1e-6 * DESIGN_EDGES_PER_S
        )
        print(
            f"    projected at {DESIGN_EDGES_PER_S:.0f} edges/s   "
            f"{projected * 100:8.1f} % of one core"
        )
        if projected >= CPU_TOTAL_GATE:
            failures += 1
            print(
                "    [FAIL]  the projection to the 200 RPM design point is over "
                "budget even though this run was not"
            )
    else:
        print("    marginal CPU per edge         needs a baseline phase with fewer edges")

    print()
    if failures:
        print(f"  {failures} gate(s) FAILED.")
        print("  Levers, in this order:")
        print("    1. raise --glitch-us above 30. Every edge the daemon filter")
        print("       suppresses is a callback you do not pay for -- but watch that")
        print("       the total edge count does not fall, or you are eating real edges.")
        print("    2. CPU governor to performance, then re-check vcgencmd get_throttled,")
        print("       because that costs supply headroom.")
        print("  Do NOT 'fix' this by dropping to x1 decoding: reading channel B per")
        print("  edge is a pigpiod socket round trip of 50-100 us, which is 100-200 %")
        print("  of a core. It is strictly worse.")
    else:
        print("  All gates passed. The x2 decode design holds on this hardware.")
    print("=" * 72)
    return 1 if failures else 0


def as_json(phases: list[PhaseResult], names: dict[int, str]) -> dict[str, object]:
    return {
        "design_edges_per_s": DESIGN_EDGES_PER_S,
        "phases": [
            {
                "name": phase.name,
                "seconds": round(phase.seconds, 4),
                "ticks": phase.ticks,
                "overruns": phase.overruns,
                "edges_l": phase.edges_l,
                "edges_r": phase.edges_r,
                "edge_rate": round(phase.edge_rate, 2),
                "rpm_l": round(phase.rpm_l, 2),
                "rpm_r": round(phase.rpm_r, 2),
                "p50_us": int(phase.p50 * 1e6),
                "p95_us": int(phase.p95 * 1e6),
                "p99_us": int(phase.p99 * 1e6),
                "max_us": int(phase.maximum * 1e6),
                "process_cpu_s": round(phase.process_cpu, 4),
                "pigpiod_cpu_s": None
                if phase.pigpiod_cpu is None
                else round(phase.pigpiod_cpu, 4),
                "process_fraction": round(phase.process_fraction, 4),
                "pigpiod_fraction": round(phase.pigpiod_fraction, 4),
                "threads": {
                    names.get(tid, str(tid)): round(cpu, 4)
                    for tid, cpu in phase.thread_cpu.items()
                    if cpu > 0.0005
                },
            }
            for phase in phases
        ],
    }


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Measure encoder CPU cost and loop jitter, and check the phase 5 gates.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seconds", type=float, default=30.0, help="loaded measurement duration")
    parser.add_argument(
        "--baseline-seconds",
        type=float,
        default=5.0,
        help="idle measurement, subtracted to get the marginal cost per edge",
    )
    parser.add_argument(
        "--duty",
        type=float,
        default=0.60,
        help="open-loop duty on both motors during the loaded phase",
    )
    parser.add_argument(
        "--hand",
        action="store_true",
        help="do not drive the motors; the operator spins the wheels by hand",
    )
    parser.add_argument(
        "--period", type=float, default=CONTROL_PERIOD_S, help="control loop period"
    )
    parser.add_argument(
        "--glitch-us",
        type=int,
        default=ENCODER_GLITCH_US,
        help="daemon-side glitch filter; the first lever if the gates fail",
    )
    parser.add_argument("--backend", default="auto", help="auto, pigpio or mock")
    parser.add_argument("--config", type=Path, default=None, help="vehicle config YAML")
    parser.add_argument("--json", type=Path, default=None, help="write the numbers here too")
    return parser


def load_config(path: Path | None) -> VehicleConfig:
    if path is not None:
        return VehicleConfig.load(path, LOCAL_CONFIG_PATH)
    return VehicleConfig.load_default()


def confirm(duty: float, seconds: float) -> bool:
    print()
    print("  " + "=" * 66)
    print(f"   This drives BOTH motors at {duty:.2f} duty for {seconds:g} seconds.")
    print("     - wheels off the ground")
    print("     - safety glasses on")
    print("     - master switch reachable without moving your feet")
    print("     - watch the L298N heatsink: abort at 80 C")
    print("  " + "=" * 66)
    if not sys.stdin.isatty():
        print("  no tty: refusing to drive the motors unattended")
        return False
    try:
        return input("  Type GO to continue: ").strip().upper() == "GO"
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2

    duty = 0.0 if args.hand else max(0.0, min(abs(args.duty), config.max_duty))
    if duty > 0.0 and not confirm(duty, args.seconds):
        print("  Aborted. Nothing was driven.")
        return 2

    clock = RealClock()
    try:
        gpio = open_backend(args.backend, clock)
    except GpioError as exc:
        print(f"cannot open the GPIO backend: {exc}", file=sys.stderr)
        return 2

    sampler = CpuSampler(clock)
    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    phases: list[PhaseResult] = []
    status = 1

    with PanicChain(panic):
        try:
            motors = MotorPair(gpio, config.pins.motors, config, clock)
            panic.attach(motors=motors)
            encoder_l = QuadratureEncoder(
                gpio,
                config.pins.encoders.left_a,
                config.pins.encoders.left_b,
                cpr=config.encoder_cpr,
                invert=config.encoder_invert_left,
                glitch_us=args.glitch_us,
            )
            encoder_r = QuadratureEncoder(
                gpio,
                config.pins.encoders.right_a,
                config.pins.encoders.right_b,
                cpr=config.encoder_cpr,
                invert=config.encoder_invert_right,
                glitch_us=args.glitch_us,
            )
            loop = BenchLoop(gpio, config, motors, encoder_l, encoder_r, clock)

            print()
            print(f"  glitch filter {args.glitch_us} us   cpr {config.encoder_cpr}   "
                  f"period {args.period * 1e3:.1f} ms")
            if sampler.pigpiod_pid is None:
                print("  pigpiod not found: its CPU share will be reported as unavailable")
            else:
                print(f"  pigpiod pid {sampler.pigpiod_pid}")
            print()

            baseline = None
            if args.baseline_seconds > 0.0:
                baseline = run_phase(
                    "baseline (idle)",
                    loop,
                    sampler,
                    clock,
                    seconds=args.baseline_seconds,
                    duty=0.0,
                    period=args.period,
                    encoder_l=encoder_l,
                    encoder_r=encoder_r,
                )
                phases.append(baseline)

            if args.hand:
                print("  Spin BOTH wheels by hand, steadily, for the whole loaded phase.")
                if sys.stdin.isatty():
                    try:
                        input("  Press Enter to start. ")
                    except (EOFError, KeyboardInterrupt):
                        print()
                        return 2

            loaded = run_phase(
                "loaded" if duty > 0.0 else "loaded (hand-spun)",
                loop,
                sampler,
                clock,
                seconds=args.seconds,
                duty=duty,
                period=args.period,
                encoder_l=encoder_l,
                encoder_r=encoder_r,
            )
            phases.append(loaded)
            motors.coast()

            names = thread_names()
            for phase in phases:
                print_phase(phase, names)
            status = print_gates(loaded, baseline)

            if args.json is not None:
                args.json.parent.mkdir(parents=True, exist_ok=True)
                args.json.write_text(
                    json.dumps(as_json(phases, names), indent=2), encoding="utf-8"
                )
                print(f"  wrote {args.json}")
        except KeyboardInterrupt:
            print("\n  interrupted -- outputs are being safed")
            status = 2
        finally:
            gpio.cleanup()

    return status


if __name__ == "__main__":
    raise SystemExit(main())
