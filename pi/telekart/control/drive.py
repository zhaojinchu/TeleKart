"""The 100 Hz loop body: one driver command in, one bridge command out.

Owns nothing it did not construct. The drivers, the safety machine, the
odometry and the calibration are all injected, which is what lets this run in a
test against ``MockBackend`` and ``FakeClock`` with no hardware, no sockets and
no wall-clock time at all.

The pipeline, in order, because the order is the design::

    shaping  ->  mixer  ->  feedforward + PID  ->  deadband floor
             ->  combined-duty budget  ->  clamp  ->  dead time  ->  H-bridge

**Feedforward carries the load; the PID only trims.** The calibration table
already knows roughly what duty holds a given RPM on this wheel in this
direction, so the regulator starts from the right neighbourhood and spends its
authority on the difference. That is the only reason the loop is tunable at all
on a plant whose gain moves by a factor of two between a fresh pack and a flat
one.

**The speed loop never commands duty against the requested direction.** A PID
allowed to go negative while the wheel is turning forwards is a plugging brake:
locked-rotor current out of a converter that sustains 1.5 A, and a bridge that
gets hot doing something the driver did not ask for. Braking is a deliberate
command on its own channel, never a side effect of overshoot.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol

from telekart_protocol import ControlFlags, Fault, TelemetryFlags, VehicleState

from ..config import VehicleConfig
from ..constants import counts_to_metres
from ..drivers.encoder import QuadratureEncoder
from ..drivers.motor import MotorPair
from ..drivers.servo import SteeringServo
from ..hal.base import GpioBackend
from ..log import get_logger
from ..util.clock import Clock
from .mixer import DifferentialMixer
from .pid import PID
from .safety import FeedbackSnapshot, SafetyStateMachine
from .shaping import ExpoCurve, clamp, deadzone, rate_limit, speed_sensitive_scale

if TYPE_CHECKING:  # `telekart.odometry` imports control.shaping, so importing it
    # here at runtime would close a cycle. The annotation is all that is needed.
    from ..odometry import BicycleOdometry

_log = get_logger(__name__)

#: Targets below this count as "stop", not as "creep very slowly". Without it
#: the deadband floor would kick a stationary wheel to the conduction threshold
#: every time the rate limiter left a fractional RPM on the target.
MIN_TARGET_RPM = 1.0

#: Duties below this are zero. Matches the motor driver's own epsilon.
_DUTY_EPS = 1e-4

#: Brake input below this is treated as released. The desktop app already has a
#: pedal deadzone; this is the firmware refusing to act on the residue.
BRAKE_THRESHOLD = 0.02

#: Steering must be inside this band before the straight-line sync correction
#: is allowed to act. Outside it the electronic differential is deliberately
#: making the wheels turn at different speeds, and a term that fights that is a
#: term that scrubs tyres.
SYNC_STEER_BAND = 0.10

#: Ceiling on the sync correction as a fraction of measured top speed. It is a
#: trim, not an authority: 5 % is enough to null a slow drift and far too
#: little to steer with.
SYNC_LIMIT_FRACTION = 0.05

#: The sync error decays at this rate per second once steering leaves the
#: band, so a correction earned on a straight does not survive into a corner.
SYNC_DECAY_PER_S = 3.0


@dataclass(frozen=True, slots=True)
class DriveCommand:
    """One command from the driver, already normalized and authenticated."""

    steering: float  # -1 (full left) .. +1 (full right)
    throttle: float  # 0 .. 1
    brake: float  # 0 .. 1
    flags: ControlFlags
    #: Car-clock time the packet was accepted. Carried so the loop can tell a
    #: fresh command from the one it already used without consulting the clock.
    received_at: float


#: What the loop uses before the first packet arrives, and whenever it must
#: treat the driver as absent. A module-level constant, so no allocation.
NEUTRAL_COMMAND = DriveCommand(0.0, 0.0, 0.0, ControlFlags.NONE, 0.0)


@dataclass(frozen=True, slots=True)
class DriveState:
    """Everything the telemetry packet needs, in SI units.

    Built fresh each tick and handed across to the network thread by reference.
    Immutable so the reader can never see half of one tick and half of the next.
    """

    rpm_l: float
    rpm_r: float
    rpm_target_l: float
    rpm_target_r: float
    duty_l: float
    duty_r: float
    servo_us: int
    steer_angle: float
    speed: float
    v_max: float
    pose: tuple[float, float, float]
    distance: float
    slip: float
    state: VehicleState
    faults: Fault
    flags: TelemetryFlags


class DriveController:
    """The loop body. One :meth:`tick` per control period, forever."""

    __slots__ = (
        "_gpio", "_config", "_clock", "_motors", "_servo",
        "_encoder_l", "_encoder_r", "_safety", "_odometry", "_calibration",
        "_mixer", "_pid_l", "_pid_r", "_feedback",
        "_inbox", "_command", "_prev_flags",
        "_throttle_curve", "_steer_curve",
        "_target_rpm", "_dir_l", "_dir_r", "_sync_error", "_clipped",
        "_was_allowed", "_state", "ticks",
    )

    def __init__(
        self,
        *,
        gpio: GpioBackend,
        config: VehicleConfig,
        clock: Clock,
        motors: MotorPair,
        servo: SteeringServo,
        encoder_l: QuadratureEncoder,
        encoder_r: QuadratureEncoder,
        safety: SafetyStateMachine,
        odometry: BicycleOdometry,
        calibration: "DriveCalibrationLike",
    ) -> None:
        self._gpio = gpio
        self._config = config
        self._clock = clock
        self._motors = motors
        self._servo = servo
        self._encoder_l = encoder_l
        self._encoder_r = encoder_r
        self._safety = safety
        self._odometry = odometry
        self._calibration = calibration

        self._mixer = DifferentialMixer(config)
        self._pid_l = _make_pid(config)
        self._pid_r = _make_pid(config)
        self._feedback = FeedbackSnapshot()

        #: Single-slot mailbox. One writer (the network thread), one reader
        #: (this one), and the value is an immutable dataclass reference, so
        #: assignment is atomic under the GIL and no lock is needed. Do not
        #: "fix" this into a queue: a queue would deliver stale commands in
        #: order, and a control stream wants the newest one and nothing else.
        self._inbox: DriveCommand | None = None
        self._command = NEUTRAL_COMMAND
        self._prev_flags = ControlFlags.NONE

        self._throttle_curve = ExpoCurve(config.throttle_expo)
        self._steer_curve = ExpoCurve(config.steer_expo)

        self._target_rpm = 0.0
        self._dir_l = 0
        self._dir_r = 0
        self._sync_error = 0.0
        self._clipped = False
        self._was_allowed = False
        self._state = _initial_state(config, calibration)
        self.ticks = 0

        if not calibration.is_measured:
            safety.raise_fault(
                Fault.CALIBRATION_MISSING,
                "run tools/calibrate; provisional speed limits are in force",
            )

        _log.info(
            "drive controller ready",
            closed_loop=bool(config.closed_loop),
            max_rpm=calibration.max_rpm_measured,
            calibrated=calibration.is_measured,
        )

    # -- inputs -------------------------------------------------------------

    def submit_command(self, cmd: DriveCommand) -> None:
        """Hand a fresh command to the loop. Thread-safe, single-slot.

        Also stamps the safety machine's staleness clock. The network layer
        normally does that itself, and doing it here as well is deliberate
        belt-and-braces: a command that reached the loop is proof the link is
        alive, and no future refactor of the network layer can accidentally
        disconnect the failsafe from the thing it is watching.
        """
        self._inbox = cmd
        self._safety.note_control_packet()

    # -- the loop -----------------------------------------------------------

    def tick(self, dt: float) -> DriveState:
        """One control iteration. Never raises.

        Anything unexpected resolves to less duty, never to an exception: an
        exception here unwinds the control thread and leaves pigpiod holding
        whatever duty was last written, which it will hold forever.
        """
        self.ticks += 1
        config = self._config

        pending = self._inbox
        if pending is not None:
            self._command = pending
        command = self._command

        # --- measure -------------------------------------------------------
        # One backend tick read shared by both encoders: on pigpio that is a
        # socket round trip, and two of them per iteration is 2 % of the budget
        # spent asking the same question twice.
        now_us = self._gpio.ticks_us()
        sample_l = self._encoder_l.sample(dt, now_us)
        sample_r = self._encoder_r.sample(dt, now_us)
        rpm_l = sample_l.rpm
        rpm_r = sample_r.rpm
        self._motors.note_speed(rpm_l, rpm_r)

        feedback = self._feedback
        feedback.set(
            rpm_l=rpm_l,
            rpm_r=rpm_r,
            duty_l=self._motors.applied_duty_l,
            duty_r=self._motors.applied_duty_r,
            total_edges_l=self._encoder_l.total_edges,
            total_edges_r=self._encoder_r.total_edges,
            stale_l=sample_l.stale,
            stale_r=sample_r.stale,
            braking=self._motors.braking,
        )
        feedback.gpio_error = self._gpio.pop_gpio_error()

        # In-band E-stop, before the safety update so the latch takes effect on
        # this tick rather than the next one. The TCP session carries an ESTOP
        # message too, but that path is ordered, retransmitted and therefore
        # slow; this bit rides the 100 Hz UDP stream and is the one that arrives
        # while something is actually happening. Level-triggered, not edge:
        # request_estop() is idempotent, and re-latching every tick the flag is
        # held is exactly what should happen if a CLEAR_ESTOP races a stream
        # that is still asserting it.
        if command.flags & ControlFlags.ESTOP:
            self._safety.request_estop()

        verdict = self._safety.update(dt, throttle=command.throttle, measured=feedback)
        allowed = verdict.allow_drive

        if command.flags & ControlFlags.RESET_ODOM and not (
            self._prev_flags & ControlFlags.RESET_ODOM
        ):
            self._odometry.reset()
        self._prev_flags = command.flags

        # --- steering ------------------------------------------------------
        max_rpm = self._calibration.max_rpm_measured
        v_max = self._calibration.max_speed_mps(config.wheel_diameter_m)
        mean_rpm = (rpm_l + rpm_r) * 0.5
        speed = config.speed_for_rpm(mean_rpm)
        speed_fraction = 0.0
        if v_max > 0.0:
            speed_fraction = (speed if speed >= 0.0 else -speed) / v_max

        steer_angle = self._apply_steering(command, dt, speed_fraction, allowed)

        # --- throttle ------------------------------------------------------
        ceiling = verdict.duty_ceiling
        if ceiling > config.max_duty:
            ceiling = config.max_duty
        pit = bool(command.flags & ControlFlags.PIT_LIMITER)
        if pit and config.pit_duty < ceiling:
            ceiling = config.pit_duty

        # A brake axis resting at 1 % is a miscalibrated pedal, not a request
        # to short the windings on every tick.
        braking = allowed and command.brake > BRAKE_THRESHOLD
        target = self._axle_target(command, max_rpm, pit, braking, allowed)
        self._target_rpm = self._limit_target(target, dt)

        # Asking for the opposite direction while still rolling: brake to the
        # reversal threshold rather than coast to it. The motor driver refuses
        # to flip the bridge under speed -- correctly, because plugging a
        # spinning motor draws locked-rotor current -- and on friction alone
        # this drivetrain takes about three seconds to coast down from speed,
        # which is three seconds of a reverse request doing nothing at all.
        if (
            allowed
            and not braking
            and self._target_rpm * mean_rpm < 0.0
            and (mean_rpm if mean_rpm >= 0.0 else -mean_rpm) > config.reverse_allowed_rpm
        ):
            braking = True

        targets = self._mixer.mix(self._target_rpm, steer_angle)
        target_l, target_r = self._apply_sync(
            targets.rpm_l, targets.rpm_r, command.steering, max_rpm, dt
        )

        # --- regulate ------------------------------------------------------
        if allowed and not braking and ceiling > _DUTY_EPS:
            duty_l = self._wheel_duty(
                self._pid_l, "left", target_l, rpm_l, dt, ceiling,
                self._safety.encoder_failed_l, True,
            )
            duty_r = self._wheel_duty(
                self._pid_r, "right", target_r, rpm_r, dt, ceiling,
                self._safety.encoder_failed_r, False,
            )
            duty_l, duty_r, self._clipped = _apply_budget(
                duty_l, duty_r, config.duty_sum_max, ceiling
            )
        else:
            duty_l = 0.0
            duty_r = 0.0
            self._clipped = False
            self._pid_l.reset()
            self._pid_r.reset()

        # --- actuate -------------------------------------------------------
        if allowed and not self._was_allowed and self._motors.panicked:
            # The only place the panic latch is released, and only on the
            # transition *into* a drivable state -- never while already there.
            # A panic stop raised mid-drive must survive until the operator has
            # disarmed and armed again, or the very next tick would undo it.
            self._motors.clear_panic()

        if not allowed:
            if verdict.force_brake > 0.0 and not verdict.force_coast:
                self._motors.brake(verdict.force_brake)
            else:
                self._motors.coast()
        elif braking:
            demand = command.brake if command.brake > BRAKE_THRESHOLD else 1.0
            self._motors.brake(config.brake_strength * demand)
        else:
            self._motors.drive(duty_l, duty_r)

        applied_l = self._motors.applied_duty_l
        applied_r = self._motors.applied_duty_r
        self._encoder_l.set_direction_hint(_sign(applied_l))
        self._encoder_r.set_direction_hint(_sign(applied_r))

        # --- odometry ------------------------------------------------------
        self._integrate_odometry(sample_l.counts, sample_r.counts, steer_angle, dt)

        self._was_allowed = allowed
        self._state = self._build_state(
            rpm_l, rpm_r, target_l, target_r, applied_l, applied_r,
            steer_angle, speed, v_max, braking, pit,
        )
        return self._state

    def panic_stop(self) -> None:
        """Kill the bridge and let the steering go limp. Signal-handler safe.

        No allocation, no formatting, no logging: this runs from ``atexit``,
        from ``SIGTERM`` and from ``sys.excepthook``. The regulators are wound
        back too, so that if the process does survive, re-arming starts from
        rest instead of from whatever the integrators held when it happened.
        """
        self._motors.panic_stop()
        self._target_rpm = 0.0
        self._dir_l = 0
        self._dir_r = 0
        self._pid_l.reset()
        self._pid_r.reset()
        try:
            self._servo.relax()
        except Exception:  # noqa: BLE001 - a panic stop may not fail
            pass

    # -- pipeline stages ----------------------------------------------------

    def _apply_steering(
        self, command: DriveCommand, dt: float, speed_fraction: float, allowed: bool
    ) -> float:
        """Shape the steering input and hand it to the servo.

        Returns the *modelled* road-wheel angle after slew and clamping, not the
        commanded one. Odometry and the electronic differential both want to
        know where the wheels are, and on this servo those differ by enough to
        matter during a quick transition.
        """
        config = self._config
        servo = self._servo
        state = self._safety.state

        if not allowed and config.servo_relax_when_disarmed and state in (
            VehicleState.SAFE,
            VehicleState.FAULT,
            VehicleState.ESTOP,
            VehicleState.BOOT,
        ):
            # Limp while parked. The servo is on the Pi's own 5 V rail, so a
            # stalled servo browns out the computer, and there is nothing to
            # steer anyway. In FAILSAFE it stays powered: the car is still
            # moving and the wheels should hold their last position.
            if not servo.relaxed:
                servo.relax()
            return servo.applied_angle

        self._steer_curve.ensure(config.steer_expo)
        value = deadzone(command.steering, config.steer_deadzone)
        value = self._steer_curve.apply(value)
        value *= speed_sensitive_scale(speed_fraction, config.steer_speed_reduction)
        servo.set_normalized(value)
        servo.update(dt)
        return servo.applied_angle

    def _axle_target(
        self,
        command: DriveCommand,
        max_rpm: float,
        pit: bool,
        braking: bool,
        allowed: bool,
    ) -> float:
        """Driver throttle to an axle speed target in RPM."""
        if not allowed or braking:
            return 0.0
        config = self._config
        self._throttle_curve.ensure(config.throttle_expo)
        value = deadzone(command.throttle, config.throttle_deadband)
        value = self._throttle_curve.apply(value)
        if value <= 0.0:
            return 0.0

        target = value * max_rpm
        if pit and config.max_duty > 0.0:
            # The pit limiter caps speed as well as duty. Duty alone would still
            # let the car reach most of its top speed on a downhill, which is
            # not what a pit limiter is for.
            target *= config.pit_duty / config.max_duty
        if command.flags & ControlFlags.REVERSE_REQ and config.reverse_enabled:
            target = -target
        return target

    def _limit_target(self, target: float, dt: float) -> float:
        """Rate-limit the RPM target, not the duty.

        Inrush during a throttle step is what trips the boost converter, and
        inrush is a function of how fast the *speed* command moves. Limiting
        duty instead would still allow a step from 0 to full speed on a wheel
        already at the deadband.
        """
        config = self._config
        current = self._target_rpm
        magnitude_target = target if target >= 0.0 else -target
        magnitude_current = current if current >= 0.0 else -current
        if target * current < 0.0 or magnitude_target < magnitude_current:
            rate = config.decel_rpm_per_s
        else:
            rate = config.accel_rpm_per_s
        return rate_limit(target, current, rate, dt)

    def _apply_sync(
        self,
        target_l: float,
        target_r: float,
        steering: float,
        max_rpm: float,
        dt: float,
    ) -> tuple[float, float]:
        """Null slow left/right distance divergence on a straight.

        Two nominally identical motors are not identical, and a car that pulls
        half a degree per metre is a car that cannot be driven in a straight
        line hands-off. The correction acts only near centre; through a corner
        the electronic differential is *supposed* to be running the wheels at
        different speeds and this term would fight it.
        """
        gain = self._config.straight_sync_gain
        magnitude_steer = steering if steering >= 0.0 else -steering
        if gain <= 0.0 or magnitude_steer > SYNC_STEER_BAND or self._target_rpm == 0.0:
            # Decay rather than snap to zero: a step change in the correction
            # would show up as a twitch exactly at turn-in.
            decay = SYNC_DECAY_PER_S * dt
            if decay >= 1.0:
                self._sync_error = 0.0
            else:
                self._sync_error -= self._sync_error * decay
            return (target_l, target_r)

        divergence = self._odometry.turn_rate_encoders * self._config.track_width_m
        self._sync_error += divergence
        limit = max_rpm * SYNC_LIMIT_FRACTION
        correction = clamp(gain * self._sync_error, -limit, limit)
        # Positive divergence means the right wheel ran further, so slow it.
        return (target_l + correction * 0.5, target_r - correction * 0.5)

    def _wheel_duty(
        self,
        pid: PID,
        wheel: str,
        target: float,
        measured: float,
        dt: float,
        ceiling: float,
        encoder_failed: bool,
        left: bool,
    ) -> float:
        """Feedforward plus trim for one wheel, with the deadband floor applied."""
        magnitude_target = target if target >= 0.0 else -target
        if magnitude_target < MIN_TARGET_RPM:
            # Throttle released means coast, not "regulate down to zero". A
            # speed loop asked to hold 0 RPM against a rolling car commands duty
            # against the direction of travel, which is a plugging brake nobody
            # asked for.
            pid.reset()
            self._set_direction(left, 0)
            return 0.0

        direction = 1 if target > 0.0 else -1
        if direction != self._get_direction(left):
            # The feedforward table for the other direction is a different
            # curve, so everything the integrator learned is about a plant that
            # is no longer connected.
            pid.reset()
            self._set_direction(left, direction)

        feedforward = self._calibration.feedforward(wheel, target)
        if feedforward == 0.0:
            feedforward = self._open_loop_duty(target, ceiling)

        closed_loop = bool(self._config.closed_loop) and not encoder_failed
        if closed_loop:
            # One-sided limits: the speed loop may only ever push in the
            # direction the driver asked for. Setting them on the PID rather
            # than clamping afterwards keeps the anti-windup exact.
            if direction > 0:
                pid.set_output_limits(0.0, ceiling)
            else:
                pid.set_output_limits(-ceiling, 0.0)
            duty = pid.update(target, measured, dt, feedforward=feedforward)
        else:
            duty = clamp(feedforward, -ceiling, ceiling)

        # Deadband floor. Below the bridge's conduction threshold nothing moves
        # at all, so a command in that band is heat and nothing else -- and the
        # integrator would wind up trying to escape it. Applied only when a real
        # target is being asked for, which is what stops it creeping at rest.
        magnitude = duty if duty >= 0.0 else -duty
        deadband = self._calibration.deadband_for(wheel, target)
        if 0.0 < magnitude < deadband and deadband <= ceiling:
            duty = deadband if duty >= 0.0 else -deadband
        return duty

    def _open_loop_duty(self, target: float, ceiling: float) -> float:
        """Fallback when no feedforward table exists.

        Linear in the target against whatever top speed is believed, which on an
        uncalibrated car is the deliberately pessimistic provisional figure.
        Monotone and predictable; it is not meant to be accurate, it is meant to
        be somewhere sane for the PID to start from.
        """
        max_rpm = self._calibration.max_rpm_measured
        if max_rpm <= 0.0:
            return 0.0
        return clamp(target / max_rpm, -1.0, 1.0) * ceiling

    def _integrate_odometry(
        self, counts_l: int, counts_r: int, steer_angle: float, dt: float
    ) -> None:
        config = self._config
        cpr = config.encoder_cpr
        diameter = config.wheel_diameter_m
        d_left = counts_to_metres(counts_l, cpr, diameter)
        d_right = counts_to_metres(counts_r, cpr, diameter)

        # A failed encoder reports zero travel, which would swing the pose hard
        # toward the dead side. Substituting the partner wheel through the same
        # differential ratio the mixer used is exactly the right estimate, and
        # it degrades the pose gracefully instead of destroying it.
        failed_l = self._safety.encoder_failed_l
        failed_r = self._safety.encoder_failed_r
        if failed_l != failed_r:
            split = self._mixer.split_for(steer_angle)
            if failed_l and (1.0 - split) != 0.0:
                d_left = d_right * (1.0 + split) / (1.0 - split)
            elif failed_r and (1.0 + split) != 0.0:
                d_right = d_left * (1.0 - split) / (1.0 + split)

        self._odometry.update(d_left, d_right, steer_angle, dt)

    def _build_state(
        self,
        rpm_l: float,
        rpm_r: float,
        target_l: float,
        target_r: float,
        duty_l: float,
        duty_r: float,
        steer_angle: float,
        speed: float,
        v_max: float,
        braking: bool,
        pit: bool,
    ) -> DriveState:
        config = self._config
        safety = self._safety
        flags = TelemetryFlags.NONE

        if (
            self._clipped
            or self._motors.limiter_active
            or self._servo.slewing
            or safety.faults & (Fault.BROWNOUT | Fault.LOW_BATTERY)
        ):
            flags |= TelemetryFlags.LIMITER_ACTIVE
        if self._encoder_l.direction_uncertain or self._encoder_r.direction_uncertain:
            flags |= TelemetryFlags.DIRECTION_UNCERTAIN
        if self._calibration.is_measured:
            flags |= TelemetryFlags.CALIBRATED
        if config.closed_loop and not (safety.encoder_failed_l or safety.encoder_failed_r):
            flags |= TelemetryFlags.CLOSED_LOOP
        if self._target_rpm < 0.0:
            flags |= TelemetryFlags.REVERSE_ENGAGED
        if pit:
            flags |= TelemetryFlags.PIT_LIMITER
        if braking or self._motors.braking:
            flags |= TelemetryFlags.BRAKING
        if not (safety.encoder_failed_l or safety.encoder_failed_r):
            flags |= TelemetryFlags.ODOM_VALID

        return DriveState(
            rpm_l=rpm_l,
            rpm_r=rpm_r,
            rpm_target_l=target_l,
            rpm_target_r=target_r,
            duty_l=duty_l,
            duty_r=duty_r,
            servo_us=self._servo.applied_us,
            steer_angle=steer_angle,
            speed=speed,
            v_max=v_max,
            pose=self._odometry.pose,
            distance=self._odometry.distance,
            slip=self._odometry.slip_index,
            state=safety.state,
            faults=safety.faults,
            flags=flags,
        )

    # -- direction bookkeeping ----------------------------------------------

    def _get_direction(self, left: bool) -> int:
        return self._dir_l if left else self._dir_r

    def _set_direction(self, left: bool, value: int) -> None:
        if left:
            self._dir_l = value
        else:
            self._dir_r = value

    # -- accessors ----------------------------------------------------------

    @property
    def state(self) -> DriveState:
        """The most recent tick's state, for a reader that missed it."""
        return self._state

    @property
    def calibration(self) -> "DriveCalibrationLike":
        return self._calibration

    @calibration.setter
    def calibration(self, value: "DriveCalibrationLike") -> None:
        """Adopt a fresh calibration, e.g. straight after a sweep.

        Resets both regulators: every integral they hold describes a plant whose
        feedforward has just changed underneath them.
        """
        self._calibration = value
        self._pid_l.reset()
        self._pid_r.reset()
        self._dir_l = 0
        self._dir_r = 0
        _log.info("calibration adopted", max_rpm=value.max_rpm_measured)

    def refresh_params(self) -> None:
        """Re-read everything the loop caches after a parameter push.

        The curve tables notice a changed exponent by themselves on the next
        tick; gains and motor inversion do not, because nothing compares them.
        """
        config = self._config
        self._pid_l.set_gains(config.pid_kp, config.pid_ki, config.pid_kd)
        self._pid_r.set_gains(config.pid_kp, config.pid_ki, config.pid_kd)
        self._motors.refresh_config()

    def __repr__(self) -> str:
        return (
            f"DriveController(ticks={self.ticks}, state={self._safety.state.name}, "
            f"target={self._target_rpm:.1f}rpm)"
        )


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


class DriveCalibrationLike(Protocol):
    """Structural stand-in for :class:`telekart.calibration.DriveCalibration`.

    A protocol rather than the real import so that ``control`` does not depend
    on ``calibration``, which depends on ``drivers`` and would close a cycle
    back through this module. It also documents exactly how much of the
    calibration the loop actually consumes, which is less than it looks.
    """

    @property
    def is_measured(self) -> bool:
        ...

    @property
    def max_rpm_measured(self) -> float:
        ...

    def max_speed_mps(self, wheel_diameter_m: float) -> float:
        ...

    def feedforward(self, wheel: str, rpm: float) -> float:
        ...

    def deadband_for(self, wheel: str, rpm: float = 1.0) -> float:
        ...


def _make_pid(config: VehicleConfig) -> PID:
    return PID(
        config.pid_kp,
        config.pid_ki,
        config.pid_kd,
        i_clamp=config.pid_i_clamp,
        output_limits=(-config.max_duty, config.max_duty),
    )


def _initial_state(config: VehicleConfig, calibration: DriveCalibrationLike) -> DriveState:
    """A zeroed state so telemetry has something coherent to send before the
    first tick, rather than None or a half-built packet."""
    return DriveState(
        rpm_l=0.0,
        rpm_r=0.0,
        rpm_target_l=0.0,
        rpm_target_r=0.0,
        duty_l=0.0,
        duty_r=0.0,
        servo_us=0,
        steer_angle=0.0,
        speed=0.0,
        v_max=calibration.max_speed_mps(config.wheel_diameter_m),
        pose=(0.0, 0.0, 0.0),
        distance=0.0,
        slip=0.0,
        state=VehicleState.BOOT,
        faults=Fault.NONE,
        flags=TelemetryFlags.NONE,
    )


def _apply_budget(
    duty_l: float, duty_r: float, budget: float, ceiling: float
) -> tuple[float, float, bool]:
    """Combined-duty budget, then the per-motor ceiling.

    The converter trips on simultaneous demand, not on either motor alone, so
    the sum is what has to be bounded. Scaling both by one factor is what keeps
    the electronic differential's ratio intact -- clipping the larger one alone
    would square the wheels up mid-corner, which is exactly the scrub the
    differential exists to avoid.

    Repeated inside ``MotorPair`` as the authoritative clamp. Done here too
    because the controller has to *know* the duty it is going to get -- for
    telemetry, and because a clip the controller applied itself would otherwise
    never reach the driver and never light ``LIMITER_ACTIVE``. The third element
    of the result is exactly that: "something here modified the command".
    """
    magnitude_l = duty_l if duty_l >= 0.0 else -duty_l
    magnitude_r = duty_r if duty_r >= 0.0 else -duty_r
    total = magnitude_l + magnitude_r
    clipped = False
    if total > budget and total > 0.0:
        scale = budget / total
        duty_l *= scale
        duty_r *= scale
        clipped = True
    limited_l = clamp(duty_l, -ceiling, ceiling)
    limited_r = clamp(duty_r, -ceiling, ceiling)
    if limited_l != duty_l or limited_r != duty_r:
        clipped = True
    return (limited_l, limited_r, clipped)


def _sign(value: float) -> int:
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0
