"""Measured drivetrain characteristics, and the sweep that measures them.

Nothing in this firmware hardcodes a top speed. The nameplate says a GA37-520
does 360 RPM at 12 V; this car runs 9 V through a Darlington bridge that eats
1.4 V of it, sharing 1.5 A between two motors, so the truth is somewhere near
150-200 RPM and it moves with pack state, tyre surface and temperature. Every
speed-scaled thing in the system -- the throttle mapping, the telemetry
``v_max``, the desktop speedometer -- reads :attr:`DriveCalibration.max_rpm_measured`
instead of a constant, which is what lets the same build work unchanged when the
L298N is eventually replaced with a MOSFET bridge.

Three numbers come out of the sweep, per wheel and per direction, because they
genuinely differ per wheel and per direction:

* **deadband** -- the duty at which a stopped wheel breaks away. Bridge drop plus
  stiction; around 0.22 on this hardware.
* **max_rpm** -- the speed reached at the duty each wheel can hold while the
  other one is also driving, i.e. at ``duty_sum_max / 2``. Not at ``max_duty``:
  the sweep runs one wheel at a time because the converter cannot feed both,
  and quoting that figure as the car's top speed would put every gauge
  permanently out of reach.
* **ff_lut** -- (rpm, duty) breakpoints between the two. This is the feedforward
  table, and it is what carries the load in the control loop. The PID only trims
  it. A loop where the PID has to find the operating point from scratch on a
  plant whose gain is uncertain by 2x is a loop that cannot be tuned.

The result is written to ``pi/config/calibration.yaml``, deliberately a
different file from ``config.yaml`` and ``config.local.yaml``, so that a machine
that writes numbers can never clobber a file a human edits.
"""

from __future__ import annotations

import os
import tempfile
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence

from .config import VehicleConfig
from .constants import CALIBRATION_PATH, rpm_to_mps
from .drivers.encoder import QuadratureEncoder
from .drivers.motor import MotorPair
from .drivers.servo import SteeringServo
from .log import get_logger
from .util.clock import Clock

_log = get_logger(__name__)

LEFT_FWD = "left_fwd"
LEFT_REV = "left_rev"
RIGHT_FWD = "right_fwd"
RIGHT_REV = "right_rev"

#: Every key a complete calibration carries, in sweep order.
WHEEL_KEYS: tuple[str, ...] = (LEFT_FWD, LEFT_REV, RIGHT_FWD, RIGHT_REV)

#: Used only until a real measurement exists, and paired with
#: ``Fault.CALIBRATION_MISSING`` so the app can say so rather than quietly
#: scaling its gauge to a guess. Deliberately at the pessimistic end of the
#: plausible range: an uncalibrated car should feel slow, not surprising.
PROVISIONAL_MAX_RPM = 120.0

#: Duty used to bring a wheel to rest between sweep legs. Braking current is
#: recovered kinetic energy, not converter current, so this is free.
_STOP_BRAKE_DUTY = 0.5

#: Format version of calibration.yaml. Bumped if the meaning of a field changes;
#: a file from a newer build is ignored rather than misread.
CALIBRATION_FORMAT = 2


class CalibrationError(RuntimeError):
    """The calibration procedure could not produce a usable result."""


class CalibrationAborted(CalibrationError):
    """The operator, or a safety layer, stopped the sweep part way through."""


@dataclass
class DriveCalibration:
    """What the sweep measured. Plain data, safe to construct empty.

    An empty instance is the correct state for a car that has never been
    calibrated: every accessor degrades to something conservative and
    :attr:`is_measured` is False, which is what raises
    ``Fault.CALIBRATION_MISSING`` at startup.
    """

    max_rpm: dict[str, float] = field(default_factory=dict)
    deadband: dict[str, float] = field(default_factory=dict)
    #: Per direction key, ascending ``(rpm, duty)`` breakpoints. RPM is an
    #: unsigned magnitude -- the direction already lives in the key.
    ff_lut: dict[str, list[tuple[float, float]]] = field(default_factory=dict)
    measured_at: str = ""
    #: Whether the wheels were on the ground. A bench calibration with the
    #: wheels in the air measures a drivetrain with no load on it and will
    #: overstate max_rpm by a wide margin, so the flag travels with the data.
    on_ground: bool = False

    # -- queries ------------------------------------------------------------

    @property
    def is_measured(self) -> bool:
        """True when all four directions have a max_rpm."""
        return all(self.max_rpm.get(key, 0.0) > 0.0 for key in WHEEL_KEYS)

    @property
    def max_rpm_measured(self) -> float:
        """The speed the *car* can hold: the slowest of the four directions.

        The minimum rather than the mean, because the axle can only go as fast
        as its slower wheel before the differential starts fighting. Everything
        downstream scales off this one number.
        """
        values = [self.max_rpm.get(key, 0.0) for key in WHEEL_KEYS]
        usable = [value for value in values if value > 0.0]
        if len(usable) < len(WHEEL_KEYS):
            return PROVISIONAL_MAX_RPM
        return min(usable)

    def max_speed_mps(self, wheel_diameter_m: float) -> float:
        """Measured top speed in m/s. Published in every telemetry packet, and
        what the desktop speedometer scales itself to."""
        return rpm_to_mps(self.max_rpm_measured, wheel_diameter_m)

    def key_for(self, wheel: str, rpm: float) -> str:
        """Resolve ``"left"`` plus a signed RPM to a direction key.

        A fully-qualified key passes straight through, so callers that already
        know the direction do not have to fabricate a signed RPM to ask about it.
        """
        if wheel in WHEEL_KEYS:
            return wheel
        return wheel + ("_rev" if rpm < 0.0 else "_fwd")

    def feedforward(self, wheel: str, rpm: float) -> float:
        """Duty expected to hold ``rpm`` on ``wheel``. Signed; 0.0 if unknown.

        Below the lowest measured breakpoint the table is interpolated from the
        origin rather than clamped to the breakpoint, so that a target of zero
        asks for zero duty. Above the highest it is held flat: extrapolating a
        curve that flattens out by construction would only ever ask for duty the
        bridge does not have.
        """
        if rpm != rpm:
            return 0.0
        points = self.ff_lut.get(self.key_for(wheel, rpm))
        if not points:
            return 0.0
        magnitude = rpm if rpm >= 0.0 else -rpm

        first_rpm, first_duty = points[0]
        if magnitude <= first_rpm:
            duty = 0.0 if first_rpm <= 0.0 else first_duty * (magnitude / first_rpm)
            return duty if rpm >= 0.0 else -duty

        previous_rpm = first_rpm
        previous_duty = first_duty
        for point_rpm, point_duty in points:
            if magnitude <= point_rpm:
                span = point_rpm - previous_rpm
                if span <= 0.0:
                    duty = point_duty
                else:
                    ratio = (magnitude - previous_rpm) / span
                    duty = previous_duty + (point_duty - previous_duty) * ratio
                return duty if rpm >= 0.0 else -duty
            previous_rpm = point_rpm
            previous_duty = point_duty

        return previous_duty if rpm >= 0.0 else -previous_duty

    def deadband_for(self, wheel: str, rpm: float = 1.0) -> float:
        """Break-away duty for a wheel and direction, or 0.0 if unmeasured.

        Zero when unmeasured on purpose: guessing high makes an uncalibrated car
        lurch off the line, and the integrator climbs through the real deadband
        in about a tenth of a second anyway.
        """
        return self.deadband.get(self.key_for(wheel, rpm), 0.0)

    def max_rpm_for(self, wheel: str, rpm: float = 1.0) -> float:
        return self.max_rpm.get(self.key_for(wheel, rpm), 0.0)

    def rpm_at_duty(self, wheel: str, duty: float) -> float:
        """Inverse of :meth:`feedforward`: the speed a duty is expected to hold."""
        magnitude = duty if duty >= 0.0 else -duty
        rpm = rpm_at_duty(self.ff_lut.get(self.key_for(wheel, duty), ()), magnitude)
        return rpm if duty >= 0.0 else -rpm

    # -- construction -------------------------------------------------------

    def set_wheel(
        self,
        key: str,
        *,
        deadband: float,
        max_rpm: float,
        points: Sequence[tuple[float, float]],
    ) -> None:
        """Record one direction's results, monotonizing the table as it goes."""
        if key not in WHEEL_KEYS:
            raise ValueError(f"unknown wheel key {key!r}; expected one of {WHEEL_KEYS}")
        self.deadband[key] = deadband
        self.max_rpm[key] = max_rpm
        self.ff_lut[key] = monotonize(points)

    def stamp(self, *, on_ground: bool) -> None:
        """Record when and how this was measured.

        Wall-clock time, from ``datetime`` rather than from the injected Clock:
        the Clock is monotonic by design and has no epoch, and this field exists
        to be read by a human wondering whether the numbers predate the new
        tyres. It is never used for control.
        """
        self.measured_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
        self.on_ground = on_ground

    def summary(self) -> str:
        if not self.is_measured:
            return "calibration: none (using provisional limits)"
        parts = [
            f"{key}={self.max_rpm.get(key, 0.0):.0f}rpm/db{self.deadband.get(key, 0.0):.2f}"
            for key in WHEEL_KEYS
        ]
        ground = "on ground" if self.on_ground else "wheels up"
        return f"calibration: {' '.join(parts)} ({ground}, {self.measured_at})"

    # -- persistence --------------------------------------------------------

    def as_dict(self) -> dict[str, Any]:
        return {
            "format": CALIBRATION_FORMAT,
            "measured_at": self.measured_at,
            "on_ground": self.on_ground,
            "max_rpm": {key: round(value, 2) for key, value in self.max_rpm.items()},
            "deadband": {key: round(value, 4) for key, value in self.deadband.items()},
            "ff_lut": {
                key: [[round(rpm, 2), round(duty, 4)] for rpm, duty in points]
                for key, points in self.ff_lut.items()
            },
        }

    def save(self, path: Path = CALIBRATION_PATH) -> None:
        """Write atomically. The car is powered down with a battery switch, and
        a half-written calibration file is a car that boots uncalibrated."""
        payload = self.as_dict()
        header = (
            "# TeleKart drive calibration -- machine written, do not hand edit.\n"
            "# Produced by tools/calibrate. Kept separate from config.yaml so an\n"
            "# automated sweep can never clobber a hand-tuned setting.\n"
            "# max_rpm and ff_lut are measured on THIS car with THIS pack; they\n"
            "# are not transferable between vehicles.\n"
        )
        _write_yaml_atomic(Path(path), payload, header)
        _log.info("calibration written", path=str(path), summary=self.summary())

    @classmethod
    def load(cls, path: Path = CALIBRATION_PATH) -> "DriveCalibration | None":
        """Read a calibration, or ``None`` if there is not a usable one.

        A missing file is normal -- a fresh car has never been calibrated. A
        corrupt or future-format file logs loudly and still returns ``None``:
        refusing to boot would leave no way to drive the car to a bench and
        diagnose it, and the provisional limits are safe by construction.
        """
        path = Path(path)
        if not path.is_file():
            _log.info("no calibration file", path=str(path))
            return None
        try:
            data = _read_yaml(path)
        except Exception as exc:  # noqa: BLE001 - any read problem is the same problem
            _log.error("calibration file unreadable", path=str(path), error=str(exc))
            return None
        if not isinstance(data, dict):
            _log.error("calibration file is not a mapping", path=str(path))
            return None

        version = data.get("format", 1)
        if not isinstance(version, int) or version > CALIBRATION_FORMAT:
            _log.error(
                "calibration file is from a newer build",
                path=str(path),
                format=version,
                understood=CALIBRATION_FORMAT,
            )
            return None

        calibration = cls(
            measured_at=str(data.get("measured_at", "")),
            on_ground=bool(data.get("on_ground", False)),
        )
        calibration.max_rpm = _float_map(data.get("max_rpm"))
        calibration.deadband = _float_map(data.get("deadband"))
        raw_lut = data.get("ff_lut")
        if isinstance(raw_lut, dict):
            for key, points in raw_lut.items():
                if key not in WHEEL_KEYS or not isinstance(points, list):
                    continue
                pairs: list[tuple[float, float]] = []
                for entry in points:
                    if isinstance(entry, (list, tuple)) and len(entry) == 2:
                        try:
                            pairs.append((float(entry[0]), float(entry[1])))
                        except (TypeError, ValueError):
                            continue
                if pairs:
                    calibration.ff_lut[key] = monotonize(pairs)

        if not calibration.is_measured:
            _log.warning(
                "calibration file is incomplete; provisional limits stay in force",
                path=str(path),
                have=sorted(calibration.max_rpm),
            )
        else:
            _log.info("calibration loaded", path=str(path), summary=calibration.summary())
        return calibration

    def __repr__(self) -> str:
        return f"DriveCalibration({self.summary()})"


def rpm_at_duty(points: Sequence[tuple[float, float]], duty: float) -> float:
    """Invert a feedforward table: what speed does this duty hold?

    The forward direction of the same piecewise-linear curve. Below the first
    breakpoint the wheel is not conducting yet, so the answer is zero; above the
    last one the table is held flat rather than extrapolated, for the same
    reason it is in :meth:`DriveCalibration.feedforward`.
    """
    if not points:
        return 0.0
    previous_rpm, previous_duty = 0.0, 0.0
    for point_rpm, point_duty in points:
        if duty <= point_duty:
            span = point_duty - previous_duty
            if span <= 0.0:
                return point_rpm
            ratio = (duty - previous_duty) / span
            if ratio < 0.0:
                return 0.0
            return previous_rpm + (point_rpm - previous_rpm) * ratio
        previous_rpm = point_rpm
        previous_duty = point_duty
    return previous_rpm


def monotonize(points: Sequence[tuple[float, float]]) -> list[tuple[float, float]]:
    """Sort by RPM and force the duty to be non-decreasing.

    A sweep on real hardware produces the odd inversion -- a pack that sagged on
    one step, a wheel that found a smoother patch of floor -- and an inverted
    pair in a lookup table becomes a feedforward that *reduces* duty as the
    target rises. Taking a running maximum is the least destructive fix: it
    keeps every point that was consistent and flattens the ones that were not.
    """
    cleaned = [
        (float(rpm), float(duty))
        for rpm, duty in points
        if rpm == rpm and duty == duty and rpm >= 0.0
    ]
    if not cleaned:
        return []
    cleaned.sort(key=lambda item: item[0])

    result: list[tuple[float, float]] = []
    highest = 0.0
    for rpm, duty in cleaned:
        if duty < highest:
            duty = highest
        else:
            highest = duty
        if result and rpm - result[-1][0] < 1e-6:
            # Same speed measured twice: keep the higher duty, which is the
            # conservative one -- underestimating the duty needed leaves the PID
            # to make up the difference, overestimating makes the car surge.
            if duty > result[-1][1]:
                result[-1] = (result[-1][0], duty)
            continue
        result.append((rpm, duty))
    return result


# --------------------------------------------------------------------------
# Auto-calibration
# --------------------------------------------------------------------------


class AutoCalibrator:
    """Drives the sweep that produces a :class:`DriveCalibration`.

    One wheel at a time, always. Both together would measure a drivetrain
    sharing a converter that cannot feed both, so the numbers would describe a
    condition the control loop never operates in -- and the sweep itself would
    spend half its time in regulator hiccup.

    The procedure is cooperative rather than blocking-with-sleeps so it works
    identically on hardware and against ``MockBackend`` + ``FakeClock``: pass a
    ``pump`` callable and it is invoked with each settling slice, which is where
    the mock advances its plant. On the real car ``pump`` is ``None`` and the
    encoder callbacks arrive on pigpio's own thread while this one sleeps.
    """

    __slots__ = (
        "_motors", "_encoder_l", "_encoder_r", "_servo", "_config", "_clock",
        "_pump", "_on_progress", "_period", "_settle_s", "_dwell_s",
        "_deadband_step", "_deadband_dwell_s", "_lut_points", "_aborted",
        "_running", "_cmd_l", "_cmd_r",
    )

    def __init__(
        self,
        *,
        motors: MotorPair,
        encoder_l: QuadratureEncoder,
        encoder_r: QuadratureEncoder,
        config: VehicleConfig,
        clock: Clock,
        servo: SteeringServo | None = None,
        pump: Callable[[float], None] | None = None,
        on_progress: Callable[[str, float], None] | None = None,
        period: float = 0.010,
        settle_s: float = 0.60,
        dwell_s: float = 0.90,
        deadband_step: float = 0.01,
        deadband_dwell_s: float = 0.30,
        lut_points: int = 6,
    ) -> None:
        if period <= 0.0:
            raise ValueError("period must be positive")
        if lut_points < 2:
            raise ValueError("lut_points must be at least 2")
        if not 0.0 < deadband_step < 0.5:
            raise ValueError("deadband_step must be a small positive duty")
        self._motors = motors
        self._encoder_l = encoder_l
        self._encoder_r = encoder_r
        self._servo = servo
        self._config = config
        self._clock = clock
        self._pump = pump
        self._on_progress = on_progress
        self._period = period
        self._settle_s = settle_s
        self._dwell_s = dwell_s
        self._deadband_step = deadband_step
        self._deadband_dwell_s = deadband_dwell_s
        self._lut_points = lut_points
        self._aborted = False
        self._running = False
        self._cmd_l = 0.0
        self._cmd_r = 0.0

    # -- control ------------------------------------------------------------

    def abort(self) -> None:
        """Stop at the next settling slice. Safe to call from another thread:
        one boolean store, read by the sweep loop."""
        self._aborted = True

    @property
    def running(self) -> bool:
        return self._running

    def run(self, *, on_ground: bool = False) -> DriveCalibration:
        """Measure all four directions. Always leaves the bridge coasting.

        Raises :class:`CalibrationAborted` if stopped part way, and
        :class:`CalibrationError` if a wheel never moved at all -- a result that
        is worse than no result, because the resulting table would tell the
        control loop that no duty ever produces motion.
        """
        if self._running:
            raise CalibrationError("a calibration sweep is already running")
        if self._motors.panicked:
            raise CalibrationError("motors are panic-latched; clear it first")
        self._aborted = False
        self._running = True
        calibration = DriveCalibration()
        try:
            if self._servo is not None:
                # Centre the steering first: a sweep at full lock measures tyre
                # scrub as if it were motor loss, and the numbers come out low
                # on one wheel and high on the other.
                self._servo.center()
            self._cmd_l = 0.0
            self._cmd_r = 0.0
            self._motors.coast()
            self._settle(self._settle_s)

            for index, key in enumerate(WHEEL_KEYS):
                left = key.startswith("left")
                forward = key.endswith("_fwd")
                self._progress(key, index / len(WHEEL_KEYS))
                deadband, max_rpm, points = self._sweep_one(key, left, forward)
                if max_rpm <= 0.0:
                    raise CalibrationError(
                        f"{key} never turned; check wiring, the pack, and that "
                        "the wheels are free to move"
                    )
                calibration.set_wheel(
                    key, deadband=deadband, max_rpm=max_rpm, points=points
                )
                self._cmd_l = 0.0
                self._cmd_r = 0.0
                self._motors.coast()
                self._settle(self._settle_s)

            calibration.stamp(on_ground=on_ground)
            self._progress("done", 1.0)
            _log.info("calibration complete", summary=calibration.summary())
            return calibration
        finally:
            # Whatever happened -- success, abort, an exception from the
            # backend -- the bridge is left disabled. This is the only place in
            # the file that is allowed to matter more than the measurement.
            self._running = False
            try:
                self._motors.coast()
            except Exception:  # noqa: BLE001
                pass

    # -- one direction ------------------------------------------------------

    def _sweep_one(
        self, key: str, left: bool, forward: bool
    ) -> tuple[float, float, list[tuple[float, float]]]:
        sign = 1.0 if forward else -1.0
        ceiling = self._config.max_duty

        # The dead-time sequencer refuses to flip the bridge while the wheel is
        # still turning against the new direction, and coasting down from full
        # speed on friction alone takes about three seconds. Braking to a stop
        # first is both faster and the thing that makes a reverse sweep measure
        # a break-away duty rather than measuring the gate.
        self._stop()
        deadband = self._find_deadband(key, left, sign, ceiling)
        self._command(left, 0.0)
        self._settle(self._settle_s)

        points: list[tuple[float, float]] = []
        # From just above the break-away point to the ceiling. Starting at the
        # deadband rather than at zero keeps every point on the part of the
        # curve the control loop actually uses.
        span = ceiling - deadband
        steps = self._lut_points
        for index in range(steps):
            fraction = (index + 1) / steps
            duty = deadband + span * fraction
            rpm = self._measure(left, sign * duty, self._dwell_s)
            # Magnitudes: the table is keyed by direction, so storing a signed
            # RPM would put the whole reverse curve below zero and the lookup
            # would have to know to flip it back.
            points.append((rpm if rpm >= 0.0 else -rpm, duty))
            self._progress(key, 0.5 + 0.5 * fraction)

        # Top speed is quoted at the duty each wheel can actually have while
        # the *other* one is also driving. The sweep runs one wheel at a time
        # out of necessity -- the converter cannot feed both -- but a top speed
        # measured that way is a number the car never reaches on a straight,
        # and every gauge downstream scales off it.
        shared = self._config.duty_sum_max * 0.5
        if shared > ceiling:
            shared = ceiling
        max_rpm = rpm_at_duty(points, shared)
        self._command(left, 0.0)
        return (deadband, max_rpm, points)

    def _find_deadband(self, key: str, left: bool, sign: float, ceiling: float) -> float:
        """Rising duty search for the break-away point.

        Rising, and from a standstill each time, because that is the condition
        the deadband describes: stiction is only in the way while the wheel is
        stopped. A falling search would find the much lower duty at which a
        turning wheel keeps turning, which is a different number and the wrong
        one to compensate with.
        """
        # Any sustained motion counts. A higher bar would report the duty at
        # which the wheel reaches some arbitrary speed, which is a different
        # number and would make the deadband floor overshoot every small command.
        threshold = self._config.stall_rpm_threshold * 0.4
        if threshold < 2.0:
            threshold = 2.0
        duty = self._deadband_step
        while duty <= ceiling:
            rpm = self._measure(left, sign * duty, self._deadband_dwell_s)
            magnitude = rpm if rpm >= 0.0 else -rpm
            if magnitude >= threshold:
                _log.info("deadband found", wheel=key, duty=duty, rpm=magnitude)
                return duty
            duty += self._deadband_step
        _log.warning(
            "no break-away duty found below the ceiling", wheel=key, ceiling=ceiling
        )
        return ceiling

    def _measure(self, left: bool, duty: float, dwell: float) -> float:
        """Hold a duty, let it settle, and return the mean RPM over the tail.

        The mean over the last third rather than the final instantaneous value:
        M/T output is filtered but still steps as edges land, and a single
        sample lands wherever it lands.
        """
        self._command(left, duty)
        samples = self._settle(dwell, sample_tail=True, left=left)
        if not samples:
            return 0.0
        return sum(samples) / len(samples)

    def _stop(self, timeout: float = 4.0) -> None:
        """Brake both wheels to a standstill, then release.

        Braking rather than coasting because the energy comes out of the wheel's
        own inertia and not out of the converter, so it costs nothing and takes
        a fraction of the time.
        """
        threshold = self._config.reverse_allowed_rpm * 0.5
        if threshold < 1.0:
            threshold = 1.0
        elapsed = 0.0
        while elapsed < timeout:
            self._cmd_l = 0.0
            self._cmd_r = 0.0
            self._motors.brake(_STOP_BRAKE_DUTY)
            self._settle(self._period * 4.0, brake=True)
            elapsed += self._period * 4.0
            if (
                abs(self._encoder_l.rpm) <= threshold
                and abs(self._encoder_r.rpm) <= threshold
            ):
                break
        self._motors.coast()
        self._settle(self._settle_s)

    def _command(self, left: bool, duty: float) -> None:
        if left:
            self._cmd_l = duty
            self._cmd_r = 0.0
        else:
            self._cmd_l = 0.0
            self._cmd_r = duty
        self._motors.drive(self._cmd_l, self._cmd_r)

    # -- pacing -------------------------------------------------------------

    def _settle(
        self,
        seconds: float,
        *,
        sample_tail: bool = False,
        left: bool = True,
        brake: bool = False,
    ) -> list[float]:
        """Run the loop's per-tick work for ``seconds`` of clock time.

        Everything the control loop would do to keep the estimators honest --
        re-issue the command so the dead-time sequencer advances, feed measured
        speed back to the driver, sample both encoders at a steady cadence -- is
        done here too. An encoder sampled at an irregular rate produces an M/T
        estimate with a filter that means nothing.
        """
        tail_from = seconds * (2.0 / 3.0)
        elapsed = 0.0
        samples: list[float] = []
        period = self._period
        motors = self._motors
        while elapsed < seconds:
            if self._aborted:
                raise CalibrationAborted("calibration stopped by request")
            self._clock.sleep(period)
            if self._pump is not None:
                self._pump(period)
            elapsed += period

            now_us = -1
            sample_l = self._encoder_l.sample(period, now_us)
            sample_r = self._encoder_r.sample(period, now_us)
            motors.note_speed(sample_l.rpm, sample_r.rpm)
            self._encoder_l.set_direction_hint(_sign(motors.applied_duty_l))
            self._encoder_r.set_direction_hint(_sign(motors.applied_duty_r))
            if self._servo is not None:
                self._servo.update(period)
            # Re-issue the *requested* command, not the applied one. The
            # dead-time sequencer and the rise clamp only advance on a call, so
            # a wheel asked to reverse would otherwise sit at zero duty for the
            # whole dwell -- and echoing back the clamped value would freeze the
            # ramp at whatever the first tick allowed.
            if brake:
                motors.brake(_STOP_BRAKE_DUTY)
            else:
                motors.drive(self._cmd_l, self._cmd_r)

            if sample_tail and elapsed >= tail_from:
                samples.append(sample_l.rpm if left else sample_r.rpm)
        return samples

    def _progress(self, stage: str, fraction: float) -> None:
        if self._on_progress is None:
            return
        if fraction < 0.0:
            fraction = 0.0
        elif fraction > 1.0:
            fraction = 1.0
        self._on_progress(stage, fraction)


def _sign(value: float) -> int:
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


# --------------------------------------------------------------------------
# YAML, kept local
# --------------------------------------------------------------------------


def _require_yaml() -> Any:
    """Import PyYAML with an actionable message.

    Deferred so that importing this module on a machine without PyYAML still
    works -- the calibration *maths* is pure Python and the whole test suite
    exercises it without ever touching a file.
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise CalibrationError(
            "PyYAML is required to read or write calibration.yaml. "
            "Install it with 'sudo apt install python3-yaml' on the Pi."
        ) from exc
    return yaml


def _read_yaml(path: Path) -> Any:
    yaml = _require_yaml()
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _write_yaml_atomic(path: Path, payload: dict[str, Any], header: str) -> None:
    yaml = _require_yaml()
    path.parent.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump(payload, default_flow_style=False, sort_keys=True)
    handle = tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=str(path.parent), prefix=f".{path.name}.", delete=False
    )
    tmp = Path(handle.name)
    try:
        with handle:
            handle.write(header)
            handle.write(body)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise


def _float_map(raw: Any) -> dict[str, float]:
    if not isinstance(raw, dict):
        return {}
    result: dict[str, float] = {}
    for key, value in raw.items():
        if key not in WHEEL_KEYS:
            continue
        try:
            result[key] = float(value)
        except (TypeError, ValueError):
            continue
    return result
