"""Vehicle plant, encoders, odometry and safety behaviour for the simulator.

The simulator has to be believable in the specific ways that break real code:
motors that lag, a bridge that will not start below ~12 % duty, a boost
regulator that folds back when both motors pull at once, encoders that report
integers with noise, and wheels that let go. Anything smoother than that lets a
HUD or a controller be tuned against a fiction.

Nothing here imports the firmware. The simulator must run in an environment
where ``pi/`` is not installed at all, so the few algorithms that MUST agree
with the firmware bit-for-bit (the M/T velocity estimator, the electronic
differential, the odometry integration) are reimplemented against
``docs/INTERFACES.md`` and carry a comment saying which section they track.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any

from telekart_protocol import (
    CRITICAL_FAULTS,
    ControlFlags,
    Fault,
    TelemetryFlags,
    VehicleState,
)
from telekart_protocol.constants import (
    FAILSAFE_BRAKE_AT_MS,
    FAILSAFE_COAST_AT_MS,
    FAILSAFE_DISARM_AT_MS,
)
from telekart_protocol.params import PARAMS, ParamError, coerce_all, defaults

TWO_PI = 2.0 * math.pi
GRAVITY = 9.80665

#: Tick base for synthesized encoder edge timestamps. pigpio's tick wraps at
#: 2**32 microseconds (~71.6 minutes) and the firmware's tick_diff is written to
#: survive that, so the simulator wraps too -- a wrap bug that only shows up 72
#: minutes into a session is exactly the kind of thing this is here to catch.
TICK_WRAP = 1 << 32


def clamp(value: float, lo: float, hi: float) -> float:
    # Explicit branches rather than min(max(...)): this is called several times
    # per wheel per 100 Hz tick, and two builtin calls per invocation is real
    # overhead for something the interpreter can do with two comparisons.
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % TWO_PI - math.pi


def tick_diff(earlier: int, later: int) -> int:
    """Wrap-safe difference of two 32-bit microsecond ticks."""
    return (later - earlier) & (TICK_WRAP - 1)


def deadzone(value: float, dz: float) -> float:
    """Rescales so the output stays continuous at the edge of the zone."""
    if dz <= 0.0:
        return value
    magnitude = abs(value)
    if magnitude <= dz:
        return 0.0
    return math.copysign((magnitude - dz) / (1.0 - dz), value)


def expo(value: float, gamma: float) -> float:
    if gamma == 1.0:
        return value
    return math.copysign(abs(value) ** gamma, value)


def rate_limit(target: float, current: float, max_rate: float, dt: float) -> float:
    step = max_rate * dt
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


# --------------------------------------------------------------------------
# Plant constants
# --------------------------------------------------------------------------


@dataclass(slots=True)
class PlantParams:
    """Physical constants of the modelled car.

    These are not user parameters -- they are the simulator's stand-in for
    hardware, and the numbers are chosen so the emergent behaviour matches the
    measured car rather than the nameplate. In particular NOTHING here sets a
    top speed: the top speed falls out of the bridge drop, the friction and the
    regulator current limit, which is the same way the real car gets one.
    """

    mass_kg: float = 1.60
    #: Fraction of the mass carried by the driven (rear) axle.
    rear_weight_fraction: float = 0.58

    #: Motor constant at the OUTPUT shaft (gearbox already folded in), so it is
    #: both V*s/rad and N*m/A. 0.318 gives ~270 RPM no-load at 9 V.
    motor_k: float = 0.318
    motor_r_ohm: float = 3.0
    #: Rotor inertia reflected through the 19:1 gearbox plus the wheel. Sized so
    #: tau = J*R/k^2 lands on the 0.15 s the firmware and docs assume.
    inertia_kg_m2: float = 0.00506
    coulomb_nm: float = 0.0060
    viscous_nm_per_rad_s: float = 0.00090

    #: Darlington saturation loss across the L298N, both halves of the bridge,
    #: at the ~0.5 A this drivetrain actually pulls. This is most of the reason
    #: a stock L298N car is slow, and it is why the boost is set to 9 V rather
    #: than to the pack voltage.
    bridge_drop_v: float = 1.40
    #: Below this duty a stationary motor does not break away. Real L298N
    #: behaviour and the single most surprising thing about driving one.
    stall_deadband: float = 0.12

    #: Pololu S18V20ALV set to 9 V. It sustains roughly 1.5 A for BOTH motors
    #: combined; beyond that it drops out of regulation rather than current
    #: limiting cleanly, which is what the BROWNOUT fault exists to name.
    boost_v: float = 9.0
    boost_i_limit_a: float = 1.50
    boost_efficiency: float = 0.85

    pack_v_full: float = 8.4  # 7.2 V NiMH off the charger
    pack_v_nominal: float = 7.2
    pack_r_int_ohm: float = 0.090

    #: Peak tyre friction coefficients the --wheelspin knob interpolates between.
    mu_dry: float = 1.00
    #: At 0.05 the two driven wheels can muster ~0.46 N between them against
    #: 0.47 N of rolling resistance, so --wheelspin 1.0 really does mean the car
    #: sits there spinning. Anything gentler and the default acceleration ramp
    #: never breaks traction, which would make the knob decorative.
    mu_ice: float = 0.05
    #: Slip ratio at which grip peaks. Past it the curve DECLINES, which is the
    #: whole reason wheelspin runs away instead of settling: a saturating curve
    #: would find an equilibrium on its own and traction control would have
    #: nothing to do.
    peak_slip: float = 0.15
    #: Deterministic left/right grip asymmetry. Without it, launch wheelspin is
    #: perfectly symmetric, the encoder differential stays zero, and the slip
    #: index -- which measures exactly that differential (INTERFACES.md 5) --
    #: never moves. Real axles are never symmetric.
    mu_asymmetry: float = 0.08

    crr: float = 0.030  # rolling resistance coefficient
    drag_area: float = 0.010  # 0.5 * rho * Cd * A, N/(m/s)^2

    cpu_idle_c: float = 42.0
    cpu_load_c: float = 18.0
    cpu_tau_s: float = 25.0


@dataclass(slots=True)
class SimFaultOptions:
    """Injected misbehaviour. Everything here defaults to a healthy car."""

    encoder_fault: str = ""  # "", "left", "right"
    battery_drain_v_per_min: float = 0.0
    track_width_error: float = 0.0  # fraction, e.g. 0.03 for +3 %
    wheelspin: float = 0.15  # 0 = full grip, 1 = ice
    reject_arm: bool = False


# --------------------------------------------------------------------------
# Encoders
# --------------------------------------------------------------------------


class EncoderModel:
    """Synthesizes what the firmware's x2 decoder would actually see.

    The hardware count is monotonic: the callback fires on either edge of
    channel A and does nothing but increment. Direction is inferred downstream
    from the commanded H-bridge state (INTERFACES.md 3), so this model must not
    helpfully sign anything.
    """

    __slots__ = (
        "_cpr",
        "_dead",
        "_edges",
        "_last_edge_tick_us",
        "_prev_count_f",
        "_reported",
        "_rng",
        "_travel_rad",
    )

    def __init__(self, cpr: int, rng: random.Random) -> None:
        if cpr <= 0:
            raise ValueError(f"encoder cpr must be positive, got {cpr}")
        self._cpr = cpr
        self._rng = rng
        self._travel_rad = 0.0
        self._edges = 0
        self._reported = 0
        self._last_edge_tick_us = 0
        self._dead = False
        self._prev_count_f = 0.0

    def set_cpr(self, cpr: int) -> None:
        if cpr <= 0:
            raise ValueError(f"encoder cpr must be positive, got {cpr}")
        self._cpr = cpr

    def kill(self) -> None:
        """Simulate a severed channel-A wire: edges stop, counts freeze."""
        self._dead = True

    @property
    def dead(self) -> bool:
        return self._dead

    @property
    def total(self) -> int:
        return self._reported

    @property
    def last_edge_tick_us(self) -> int:
        return self._last_edge_tick_us

    def step(self, omega: float, dt: float, tick_us: int) -> None:
        """Advance by `dt` at wheel speed `omega` (rad/s), ending at `tick_us`."""
        if self._dead:
            return

        self._travel_rad += abs(omega) * dt
        count_f = self._travel_rad * self._cpr / TWO_PI
        prev = self._prev_count_f
        self._prev_count_f = count_f

        edges = int(count_f)
        if edges > self._edges:
            # Interpolate WHEN inside the tick the last edge fell. This is the
            # whole point of the M/T estimator downstream: a velocity window
            # that closes on a loop tick quantizes to +/-45 % at 20 RPM, one
            # that closes on an edge does not.
            span = count_f - prev
            frac = 1.0 if span <= 0.0 else clamp((edges - prev) / span, 0.0, 1.0)
            edge_dt_us = int(dt * frac * 1_000_000.0)
            # A few microseconds of jitter: pigpiod timestamps an edge when its
            # notification thread gets to it, not when the transistor switched.
            edge_dt_us += self._rng.randint(-20, 20)
            self._last_edge_tick_us = (
                tick_us - int(dt * 1_000_000.0) + edge_dt_us
            ) % TICK_WRAP
            self._edges = edges

        # +/-1 count of reporting noise. Applied to the TOTAL, not to the delta,
        # because that is where it comes from on real hardware -- a marginal
        # edge that the glitch filter accepts one tick and rejects the next.
        self._reported = self._edges + self._rng.randint(-1, 1)

    def reset(self) -> None:
        self._travel_rad = 0.0
        self._edges = 0
        self._reported = 0
        self._prev_count_f = 0.0


class MTVelocity:
    """M/T velocity estimation, tracking `pi/telekart/drivers/encoder.py`.

    The measurement window closes on an EDGE rather than on the loop tick, and
    the output carries a 25 ms first-order filter. That filter is not optional
    and it is not a detail: it is the binding constraint on closed-loop
    bandwidth (6.37 Hz, docs/tuning.md 2.1). Reproducing it here is the
    difference between a HUD that behaves the same on the simulator and on the
    car, and one that only looks good on the simulator.
    """

    __slots__ = (
        "_cpr",
        "_have_window",
        "_idle_s",
        "_raw_rpm",
        "_rpm",
        "_stall_s",
        "_tau",
        "_win_count",
        "_win_tick_us",
    )

    def __init__(
        self, cpr: int, tau: float = 0.025, stall_window_s: float = 0.12
    ) -> None:
        self._cpr = cpr
        self._tau = tau
        self._stall_s = stall_window_s
        self._have_window = False
        self._win_count = 0
        self._win_tick_us = 0
        self._raw_rpm = 0.0
        self._rpm = 0.0
        self._idle_s = 0.0

    def set_cpr(self, cpr: int) -> None:
        self._cpr = cpr

    @property
    def rpm(self) -> float:
        return self._rpm

    @property
    def raw_rpm(self) -> float:
        return self._raw_rpm

    @property
    def stale(self) -> bool:
        return self._idle_s >= self._stall_s

    def reset(self) -> None:
        self._have_window = False
        self._raw_rpm = 0.0
        self._rpm = 0.0
        self._idle_s = 0.0

    def update(
        self, total_counts: int, last_edge_tick_us: int, dt: float, sign: int
    ) -> float:
        """One control tick. Returns the filtered, signed RPM.

        `sign` is the commanded direction hint (-1, 0, +1); 0 keeps the previous
        sign, which is what raises DIRECTION_UNCERTAIN upstream.
        """
        if not self._have_window:
            self._have_window = True
            self._win_count = total_counts
            self._win_tick_us = last_edge_tick_us
            self._idle_s = 0.0
        else:
            d_counts = total_counts - self._win_count
            d_t_us = tick_diff(self._win_tick_us, last_edge_tick_us)
            if d_counts > 0 and 0 < d_t_us < 2_000_000:
                self._raw_rpm = d_counts * 60_000_000.0 / (self._cpr * d_t_us)
                self._win_count = total_counts
                self._win_tick_us = last_edge_tick_us
                self._idle_s = 0.0
            else:
                # The window deliberately stays OPEN across ticks with no edges.
                # Closing it every tick is the naive estimator this exists to
                # avoid. It only collapses to zero once the stall window expires.
                self._idle_s += dt
                if self._idle_s >= self._stall_s:
                    self._raw_rpm = 0.0
                    self._win_count = total_counts
                    self._win_tick_us = last_edge_tick_us

        target = self._raw_rpm * (
            sign if sign != 0 else (1 if self._rpm >= 0.0 else -1)
        )
        alpha = dt / (self._tau + dt)
        self._rpm += (target - self._rpm) * alpha
        return self._rpm


# --------------------------------------------------------------------------
# Odometry
# --------------------------------------------------------------------------


class SimOdometry:
    """Midpoint bicycle dead reckoning, tracking INTERFACES.md 5.

    Fed from the NOISY encoder counts on purpose. Publishing the true pose would
    make every drift-correction experiment downstream meaningless, and the
    injectable track-width error is there so a systematic bias -- the kind you
    actually get from a mistyped measurement -- can be reproduced on demand.
    """

    __slots__ = (
        "_distance",
        "_slip",
        "_steer_lagged",
        "_steer_tau",
        "_theta",
        "_track_width",
        "_wheelbase",
        "_x",
        "_y",
    )

    def __init__(
        self, wheelbase_m: float, track_width_m: float, steer_tau: float = 0.10
    ) -> None:
        if wheelbase_m <= 0.0 or track_width_m <= 0.0:
            raise ValueError("wheelbase and track width must be positive")
        self._wheelbase = wheelbase_m
        self._track_width = track_width_m
        self._steer_tau = steer_tau
        self._steer_lagged = 0.0
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._distance = 0.0
        self._slip = 0.0

    def set_geometry(self, wheelbase_m: float, track_width_m: float) -> None:
        if wheelbase_m <= 0.0 or track_width_m <= 0.0:
            raise ValueError("wheelbase and track width must be positive")
        self._wheelbase = wheelbase_m
        self._track_width = track_width_m

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self._x, self._y, self._theta)

    @property
    def distance(self) -> float:
        return self._distance

    @property
    def slip_index(self) -> float:
        return self._slip

    def reset(self) -> None:
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._distance = 0.0
        self._slip = 0.0
        self._steer_lagged = 0.0

    def place(self, x: float, y: float, heading: float) -> None:
        self._x = x
        self._y = y
        self._theta = wrap_pi(heading)
        self._steer_lagged = 0.0

    def update(
        self, d_left: float, d_right: float, steer_angle: float, dt: float
    ) -> None:
        # The HS-311 takes ~0.19 s per 60 degrees. Treating the commanded angle
        # as instantaneous is one of the larger sources of heading error, so the
        # firmware lags it and so does this.
        alpha = dt / (self._steer_tau + dt)
        self._steer_lagged += (steer_angle - self._steer_lagged) * alpha

        # INTERFACES.md 5 writes this as `d * tan(delta) / wheelbase`. The minus
        # is because `delta` here is positive to the RIGHT (the ControlPacket
        # convention) while heading is counter-clockwise-positive; the two
        # formulas describe the same motion in opposite sign conventions.
        d = (d_left + d_right) * 0.5
        d_theta = -d * math.tan(self._steer_lagged) / self._wheelbase
        half = d_theta * 0.5
        self._x += d * math.cos(self._theta + half)
        self._y += d * math.sin(self._theta + half)
        self._theta = wrap_pi(self._theta + d_theta)
        self._distance += abs(d)

        d_theta_enc = (d_right - d_left) / self._track_width
        instant = abs(d_theta_enc - d_theta)
        # Lightly smoothed: the raw per-tick difference is a few hundred
        # microradians and unreadable on a gauge. tau is short enough that a
        # genuine break-away still shows up inside one wheel revolution.
        beta = dt / (0.20 + dt)
        self._slip += (instant - self._slip) * beta


# --------------------------------------------------------------------------
# Controller
# --------------------------------------------------------------------------


class _PI:
    """Conditional-integration PI trim, mirroring pi/telekart/control/pid.py.

    Deliberately a copy rather than an import: the simulator has to run in an
    environment where the firmware package is not installed.
    """

    __slots__ = ("_clamp", "_i", "ki", "kp", "saturated")

    def __init__(self, kp: float, ki: float, i_clamp: float) -> None:
        self.kp = kp
        self.ki = ki
        self._i = 0.0
        self._clamp = i_clamp
        self.saturated = False

    def set_gains(self, kp: float, ki: float, i_clamp: float) -> None:
        self.kp = kp
        self.ki = ki
        self._clamp = i_clamp

    def reset(self) -> None:
        self._i = 0.0
        self.saturated = False

    def update(
        self, error: float, dt: float, feedforward: float, lo: float, hi: float
    ) -> float:
        candidate = feedforward + self.kp * error + self._i
        # Freeze the integrator while saturated instead of winding it up into a
        # command nobody asked for and nobody can cancel quickly.
        if lo < candidate < hi:
            self._i += self.ki * error * dt
            if self._i > self._clamp:
                self._i = self._clamp
            elif self._i < -self._clamp:
                self._i = -self._clamp
            self.saturated = False
        else:
            self.saturated = True
        out = feedforward + self.kp * error + self._i
        return clamp(out, lo, hi)


@dataclass(frozen=True, slots=True)
class SimCommand:
    """What the driver asked for, whatever the source."""

    steering: float = 0.0  # -1..+1
    throttle: float = 0.0  # 0..1
    brake: float = 0.0  # 0..1
    flags: ControlFlags = ControlFlags.NONE


NEUTRAL_COMMAND = SimCommand()


@dataclass(frozen=True, slots=True)
class SimSnapshot:
    """One consistent view of the car, replaced wholesale every tick."""

    t: float
    state: VehicleState
    faults: Fault
    flags: TelemetryFlags
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
    # Dead-reckoned pose, as published to the app.
    x: float
    y: float
    heading: float
    distance: float
    slip: float
    # Ground truth. Never leaves the simulator over the wire -- the camera uses
    # it, and integration tests can assert against it.
    true_x: float
    true_y: float
    true_heading: float
    true_speed: float
    true_slip_l: float
    true_slip_r: float
    pack_v: float
    supply_v: float
    current_a: float
    cpu_temp_c: float
    throttled: int
    loop_p99_us: int


# --------------------------------------------------------------------------
# The vehicle
# --------------------------------------------------------------------------


class VehicleSim:
    """Plant, drivetrain controller, encoders, odometry and safety in one object.

    One object because they share a tick and every one of them reads state the
    others just wrote; splitting them would mean publishing five mutable
    surfaces to keep in step. The whole thing is stepped by a single caller.
    """

    def __init__(
        self,
        *,
        params: dict[str, Any] | None = None,
        plant: PlantParams | None = None,
        faults: SimFaultOptions | None = None,
        seed: int = 1,
    ) -> None:
        self.plant = plant or PlantParams()
        self.inject = faults or SimFaultOptions()
        self.params: dict[str, Any] = defaults()
        if params:
            self.params.update(coerce_all(params))

        if not 0.0 <= self.inject.wheelspin <= 1.0:
            raise ValueError(f"wheelspin must be 0..1, got {self.inject.wheelspin}")
        if self.inject.encoder_fault not in ("", "left", "right"):
            raise ValueError(f"unknown encoder fault {self.inject.encoder_fault!r}")
        if abs(self.inject.track_width_error) > 0.5:
            raise ValueError(
                "track-width-error beyond +/-50 % is not a calibration error"
            )

        self._rng_l = random.Random(seed ^ 0x51ED0001)
        self._rng_r = random.Random(seed ^ 0x51ED0002)
        self._rng_plant = random.Random(seed ^ 0x51ED0003)

        # --- plant state -------------------------------------------------
        self.omega_l = 0.0  # wheel angular velocity, rad/s
        self.omega_r = 0.0
        self.v = 0.0  # body speed, m/s
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.duty_l = 0.0
        self.duty_r = 0.0
        self.brake_cmd = 0.0
        self.steer_angle = 0.0  # radians, after servo slew
        self.servo_us = int(self.params["steer_center_us"])
        self._servo_target_us = float(self.servo_us)
        self._servo_applied_us = float(self.servo_us)
        self.supply_v = self.plant.boost_v
        self.pack_v = self.plant.pack_v_full
        self._pack_v_oc = self.plant.pack_v_full
        self.current_a = 0.0
        self.cpu_temp_c = self.plant.cpu_idle_c
        self.true_slip_l = 0.0
        self.true_slip_r = 0.0

        # --- encoders / estimation ---------------------------------------
        cpr = int(self.params["encoder_cpr"])
        self._enc_l = EncoderModel(cpr, self._rng_l)
        self._enc_r = EncoderModel(cpr, self._rng_r)
        self._mt_l = MTVelocity(cpr)
        self._mt_r = MTVelocity(cpr)
        if self.inject.encoder_fault == "left":
            self._enc_l.kill()
        elif self.inject.encoder_fault == "right":
            self._enc_r.kill()
        self._prev_total_l = 0
        self._prev_total_r = 0
        self.rpm_l = 0.0
        self.rpm_r = 0.0
        self.rpm_target_l = 0.0
        self.rpm_target_r = 0.0
        self._target_rpm = 0.0

        self.odometry = SimOdometry(*self._believed_geometry())

        self._pid_l = _PI(
            float(self.params["pid_kp"]),
            float(self.params["pid_ki"]),
            float(self.params["pid_i_clamp"]),
        )
        self._pid_r = _PI(
            float(self.params["pid_kp"]),
            float(self.params["pid_ki"]),
            float(self.params["pid_i_clamp"]),
        )

        # --- safety ------------------------------------------------------
        self.state = VehicleState.BOOT
        self.faults = Fault.NONE
        self._neutral_s = 0.0
        self._stale_s = 0.0
        self._stall_s_l = 0.0
        self._stall_s_r = 0.0
        self._estop_latched = False
        self._session_present = False
        self._arm_intent = False
        self._limiter_active = False
        self._direction_uncertain = False
        self._reverse_engaged = False
        self._boot_s = 0.0
        self._t = 0.0
        self._tick_us = 0
        self.loop_p99_us = 0
        self.video_active = False

        # Auto-calibration, exactly as the real car does it: measure, never
        # assume. Everything downstream scales off this one number.
        self.max_rpm_measured = self._measure_max_rpm()
        self.v_max = (
            self.max_rpm_measured
            / 60.0
            * math.pi
            * float(self.params["wheel_diameter_m"])
        )

    # ------------------------------------------------------------------ setup

    def _believed_geometry(self) -> tuple[float, float]:
        """The chassis dimensions the ODOMETRY thinks it has: (wheelbase, track).

        `--track-width-error` scales both, not just the track width. Per
        INTERFACES.md 5 the dead-reckoned heading comes from the steer angle and
        the wheelbase, so an error confined to the track width would move only
        the slip index and leave the published pose perfect -- which would make
        the flag useless for the drift-correction work it exists to support. It
        also happens to be what really goes wrong: one mis-read tape measure
        produces both numbers.
        """
        error = 1.0 + self.inject.track_width_error
        return (
            float(self.params["wheelbase_m"]) * error,
            float(self.params["track_width_m"]) * error,
        )

    def _mu(self, side: int) -> float:
        p = self.plant
        base = p.mu_dry + self.inject.wheelspin * (p.mu_ice - p.mu_dry)
        return base * (1.0 + p.mu_asymmetry * side)

    def _measure_max_rpm(self) -> float:
        """Run the plant flat out until it settles, and read the answer off it.

        One wheel at a time at `max_duty`, which is what the real bring-up
        routine does -- `DriveCalibration.max_rpm` has four entries for exactly
        this reason. Driving both at once would measure the combined-duty budget
        instead of the motor, and the two are different numbers: the budget is
        what limits you on the track, the motor is what the gauge scales to.

        Measuring rather than assuming means --battery-drain, a changed
        max_duty or a different wheel diameter all move v_max the way they would
        on the car, and the desktop speedometer rescales itself accordingly.
        """
        saved = (
            self.omega_l,
            self.omega_r,
            self.v,
            self.supply_v,
            self.pack_v,
            self.current_a,
        )
        self.omega_l = self.omega_r = 0.0
        self.v = 0.0
        duty = float(self.params["max_duty"])
        dt = 0.005
        for _ in range(1200):  # 6 s of virtual time; tau is 0.15 s
            self._step_plant(duty, 0.0, 0.0, dt)
        rpm = abs(self.omega_l) * 60.0 / TWO_PI
        (
            self.omega_l,
            self.omega_r,
            self.v,
            self.supply_v,
            self.pack_v,
            self.current_a,
        ) = saved
        self.x = self.y = self.theta = 0.0
        # A car that cannot move is a configuration error, not a runtime one.
        if rpm < 1.0:
            raise ValueError(
                "plant model produced no motion at full duty; check PlantParams "
                "(bridge_drop_v vs boost_v) and max_duty"
            )
        return rpm

    def apply_params(self, values: dict[str, Any]) -> list[str]:
        """Validate and apply a SET_PARAMS payload. Raises ParamError.

        Refuses `requires_disarm` parameters while armed, exactly as the
        firmware does, so the app's error path gets exercised for free.
        """
        coerced = coerce_all(values)
        if self.state == VehicleState.ARMED:
            blocked = [n for n in coerced if PARAMS[n].requires_disarm]
            if blocked:
                raise ParamError(
                    "cannot change while armed: " + ", ".join(sorted(blocked))
                )
        changed: list[str] = []
        for name, value in coerced.items():
            if self.params.get(name) != value:
                changed.append(name)
            self.params[name] = value
        if changed:
            self._rebuild_derived()
        return changed

    def _rebuild_derived(self) -> None:
        cpr = int(self.params["encoder_cpr"])
        self._enc_l.set_cpr(cpr)
        self._enc_r.set_cpr(cpr)
        self._mt_l.set_cpr(cpr)
        self._mt_r.set_cpr(cpr)
        self.odometry.set_geometry(*self._believed_geometry())
        self._pid_l.set_gains(
            float(self.params["pid_kp"]),
            float(self.params["pid_ki"]),
            float(self.params["pid_i_clamp"]),
        )
        self._pid_r.set_gains(
            float(self.params["pid_kp"]),
            float(self.params["pid_ki"]),
            float(self.params["pid_i_clamp"]),
        )
        self.max_rpm_measured = self._measure_max_rpm()
        self.v_max = (
            self.max_rpm_measured
            / 60.0
            * math.pi
            * float(self.params["wheel_diameter_m"])
        )

    # ------------------------------------------------------------- safety API

    def note_session(self, present: bool) -> None:
        """TCP presence. Losing it while armed is an E-stop condition."""
        was = self._session_present
        self._session_present = present
        if (
            was
            and not present
            and self.state in (VehicleState.ARMED, VehicleState.FAILSAFE)
        ):
            # docs/protocol.md 5.8: the session doubles as a presence signal,
            # and its loss stops the car even if UDP is still arriving from
            # somewhere. Latching means a reconnecting app must CLEAR_ESTOP,
            # which is the behaviour we want it built against.
            self.request_estop()

    def request_arm(self) -> tuple[bool, str]:
        if self.inject.reject_arm:
            return (False, "arming disabled by --reject-arm")
        if self._estop_latched:
            return (False, "E-stop is latched")
        if self.faults & CRITICAL_FAULTS:
            return (False, "critical fault active")
        if self.state == VehicleState.ARMED:
            return (True, "")
        if not self._session_present:
            return (False, "no session")
        if self._neutral_s * 1000.0 < float(self.params["arm_neutral_ms"]):
            return (False, "throttle not neutral for long enough")
        self.state = VehicleState.ARMED
        self._arm_intent = True
        self._stale_s = 0.0
        self._pid_l.reset()
        self._pid_r.reset()
        return (True, "")

    def request_disarm(self) -> None:
        self._arm_intent = False
        if self.state in (VehicleState.ARMED, VehicleState.FAILSAFE):
            self.state = VehicleState.SAFE
        self._pid_l.reset()
        self._pid_r.reset()

    def request_estop(self) -> None:
        self._estop_latched = True
        self._arm_intent = False
        self.faults |= Fault.ESTOP_LATCHED
        self.state = VehicleState.ESTOP
        self._pid_l.reset()
        self._pid_r.reset()

    def clear_estop(self) -> tuple[bool, str]:
        if not self._estop_latched:
            return (True, "")
        self._estop_latched = False
        self.faults &= ~Fault.ESTOP_LATCHED
        self.state = (
            VehicleState.SAFE
            if not (self.faults & CRITICAL_FAULTS)
            else VehicleState.FAULT
        )
        return (True, "")

    def clear_faults(self) -> None:
        keep = Fault.ESTOP_LATCHED if self._estop_latched else Fault.NONE
        self.faults = keep
        self._stall_s_l = 0.0
        self._stall_s_r = 0.0
        if self.state == VehicleState.FAULT:
            self.state = VehicleState.SAFE

    def reset_odometry(self) -> None:
        """Re-origin the estimated pose at the current position, like the car.

        Note that this does NOT move the true pose. After a RESET_ODOM the two
        frames no longer coincide, which is exactly what happens on the real
        vehicle and exactly what a map overlay has to cope with.
        """
        self.odometry.reset()

    def place(self, x: float, y: float, heading: float) -> None:
        """Put the car somewhere, with both frames agreeing.

        Called once at startup so the odometry frame and the track frame start
        coincident. Everything that separates them afterwards is real drift.
        """
        self.x = x
        self.y = y
        self.theta = wrap_pi(heading)
        self.odometry.place(x, y, self.theta)

    def note_control_packet(self) -> None:
        self._stale_s = 0.0

    def note_link_age(self, age_s: float) -> None:
        """Age of the newest accepted control packet, in WALL seconds.

        Wall, not simulated: link staleness is a property of the real network
        between here and a real driver, and under `--realtime 0` simulated time
        runs far ahead of it. Getting this wrong makes the failsafe fire on
        every fast-forward run.
        """
        self._stale_s = age_s

    # ---------------------------------------------------------------- stepping

    def step(self, dt: float, command: SimCommand) -> SimSnapshot:
        """One control tick. Never raises: clamp, drop, set a fault flag."""
        if dt <= 0.0:
            dt = 1e-4
        elif dt > 0.25:
            # A monstrous dt means the host stalled. Integrating it would fling
            # the car across the map; clip and let LOOP_OVERRUN say why.
            dt = 0.25
            self.faults |= Fault.LOOP_OVERRUN
        self._t += dt
        self._tick_us = (self._tick_us + int(dt * 1_000_000.0)) % TICK_WRAP

        if self.state == VehicleState.BOOT:
            self._boot_s += dt
            if self._boot_s >= 0.5:
                self.state = VehicleState.SAFE

        throttle = clamp(command.throttle, 0.0, 1.0)
        brake = clamp(command.brake, 0.0, 1.0)
        steering = clamp(command.steering, -1.0, 1.0)
        flags = command.flags

        if flags & ControlFlags.RESET_ODOM:
            self.odometry.reset()
        if flags & ControlFlags.ESTOP and not self._estop_latched:
            self.request_estop()
        self._arm_intent = bool(flags & ControlFlags.ARM_INTENT) or self._arm_intent

        if throttle <= 0.02 and brake <= 0.02:
            self._neutral_s += dt
        else:
            self._neutral_s = 0.0

        self._update_failsafe(dt)
        allow_drive, force_brake, force_coast = self._safety_outputs()

        # `brake` from the driver is a normalized pedal position; the safety
        # machine's forced brake is already a DUTY. Scaling the latter by
        # brake_strength again would silently make every failsafe stop 60 %
        # weaker than the parameter says.
        brake_duty = brake * float(self.params["brake_strength"])
        if force_coast:
            throttle = 0.0
            brake_duty = 0.0
        if force_brake > 0.0:
            throttle = 0.0
            brake_duty = max(brake_duty, force_brake)
        if not allow_drive:
            throttle = 0.0

        self._update_steering(steering, dt)
        duty_l, duty_r = self._update_drivetrain(
            throttle, brake_duty, flags, dt, allow_drive
        )
        if force_coast:
            duty_l = duty_r = 0.0
            brake_duty = 0.0

        self._step_plant(duty_l, duty_r, brake_duty, dt)
        self._step_encoders(dt)
        self._step_odometry(dt)
        self._step_health(dt, duty_l, duty_r)

        return self._snapshot()

    # -- safety ---------------------------------------------------------------

    def _update_failsafe(self, dt: float) -> None:
        if self.state == VehicleState.ARMED:
            timeout = float(self.params["control_timeout_ms"]) / 1000.0
            if self._stale_s > timeout:
                self.state = VehicleState.FAILSAFE
                self.faults |= Fault.CONTROL_TIMEOUT
        elif self.state == VehicleState.FAILSAFE:
            timeout = float(self.params["control_timeout_ms"]) / 1000.0
            if self._stale_s <= timeout:
                self.state = VehicleState.ARMED
            elif (self._stale_s - timeout) * 1000.0 >= FAILSAFE_DISARM_AT_MS:
                self.state = VehicleState.SAFE
                self._arm_intent = False

        if self.faults & CRITICAL_FAULTS and self.state in (
            VehicleState.ARMED,
            VehicleState.FAILSAFE,
        ):
            self.state = (
                VehicleState.ESTOP if self._estop_latched else VehicleState.FAULT
            )
            self._arm_intent = False

    def _safety_outputs(self) -> tuple[bool, float, bool]:
        """(allow_drive, force_brake, force_coast) for this tick."""
        if self.state == VehicleState.ARMED:
            return (True, 0.0, False)
        if self.state == VehicleState.FAILSAFE:
            # Coast, then brake, then coast, then disarm. Coast first because
            # braking a car that is already sliding is worse than letting it
            # settle; coast again afterwards so nothing cooks the bridge.
            since_stale_ms = (
                self._stale_s - float(self.params["control_timeout_ms"]) / 1000.0
            ) * 1000.0
            if since_stale_ms < FAILSAFE_BRAKE_AT_MS:
                return (False, 0.0, True)
            if since_stale_ms < FAILSAFE_COAST_AT_MS:
                return (False, float(self.params["failsafe_brake_duty"]), False)
            return (False, 0.0, True)
        if self.state == VehicleState.ESTOP:
            return (False, float(self.params["brake_strength"]), False)
        return (False, 0.0, True)

    # -- steering -------------------------------------------------------------

    def _update_steering(self, steering: float, dt: float) -> None:
        p = self.params
        value = deadzone(steering, float(p["steer_deadzone"]))
        value = expo(value, float(p["steer_expo"]))
        if bool(p["steer_invert"]):
            value = -value
        # Speed-sensitive scaling: less lock available the faster you go.
        frac = clamp(abs(self.v) / self.v_max, 0.0, 1.0) if self.v_max > 0.0 else 0.0
        value *= 1.0 - float(p["steer_speed_reduction"]) * frac

        center = float(p["steer_center_us"]) + float(p["steer_trim_us"])
        span = (float(p["steer_max_us"]) - float(p["steer_min_us"])) * 0.5
        target = clamp(
            center + value * span,
            float(p["steer_min_us"]),
            float(p["steer_max_us"]),
        )
        if self.state not in (VehicleState.ARMED, VehicleState.FAILSAFE) and bool(
            p["servo_relax_when_disarmed"]
        ):
            target = center

        self._servo_target_us = target
        self._servo_applied_us = rate_limit(
            target, self._servo_applied_us, float(p["steer_rate_us_per_s"]), dt
        )
        # Never rewrite a pulse that differs from the last by less than the hold
        # deadband: that is what stops the HS-311 buzzing on the shared 5 V rail.
        if abs(self._servo_applied_us - self.servo_us) >= float(p["steer_hold_us"]):
            self.servo_us = round(self._servo_applied_us)

        normalized = 0.0
        if span > 0.0:
            normalized = clamp((self._servo_applied_us - center) / span, -1.0, 1.0)
        self.steer_angle = math.radians(float(p["steer_max_deg"])) * normalized

    # -- drivetrain -----------------------------------------------------------

    def _update_drivetrain(
        self,
        throttle: float,
        brake_duty: float,
        flags: ControlFlags,
        dt: float,
        allow_drive: bool,
    ) -> tuple[float, float]:
        p = self.params
        self._limiter_active = False

        value = deadzone(throttle, float(p["throttle_deadband"]))
        value = expo(value, float(p["throttle_expo"]))

        reverse = bool(flags & ControlFlags.REVERSE_REQ) and bool(p["reverse_enabled"])
        ceiling = float(p["max_duty"])
        if flags & ControlFlags.PIT_LIMITER:
            ceiling = min(ceiling, float(p["pit_duty"]))
            self._limiter_active = True

        target = (
            value * self.max_rpm_measured * (ceiling / max(float(p["max_duty"]), 1e-6))
        )
        if reverse:
            target = -target
        if not allow_drive:
            target = 0.0

        measured = (self.rpm_l + self.rpm_r) * 0.5
        # Direction changes are gated on actually being slow, and the ramp is on
        # the RPM TARGET rather than on duty: it is inrush during a throttle
        # step that trips the regulator, not the steady draw.
        if target * measured < 0.0 and abs(measured) > float(p["reverse_allowed_rpm"]):
            target = 0.0
        rate = (
            float(p["accel_rpm_per_s"])
            if abs(target) > abs(self._target_rpm)
            else float(p["decel_rpm_per_s"])
        )
        self._target_rpm = rate_limit(target, self._target_rpm, rate, dt)
        self._reverse_engaged = self._target_rpm < -1.0

        # Electronic differential (INTERFACES.md 4). Two motors on one rear axle
        # with Ackermann steering and no mechanical diff MUST run at different
        # speeds in a turn, or the per-wheel loops fight the geometry.
        # steer_angle is positive-right, so a positive split makes the LEFT
        # wheel the outer, faster one.
        split = (
            float(p["track_width_m"])
            * math.tan(self.steer_angle)
            / (2.0 * float(p["wheelbase_m"]))
        )
        self.rpm_target_l = self._target_rpm * (1.0 + split)
        self.rpm_target_r = self._target_rpm * (1.0 - split)

        if brake_duty > 0.005 or not allow_drive:
            self._pid_l.reset()
            self._pid_r.reset()
            self.brake_cmd = brake_duty
            self.duty_l = 0.0
            self.duty_r = 0.0
            return (0.0, 0.0)
        self.brake_cmd = 0.0

        duty_l = self._wheel_duty(
            self._pid_l, self.rpm_target_l, self.rpm_l, dt, ceiling
        )
        duty_r = self._wheel_duty(
            self._pid_r, self.rpm_target_r, self.rpm_r, dt, ceiling
        )

        # Combined-duty budget. The regulator trips on SIMULTANEOUS demand, not
        # on either motor alone, so the ceiling has to be on the sum.
        budget = float(p["duty_sum_max"])
        total = abs(duty_l) + abs(duty_r)
        if total > budget and total > 0.0:
            scale = budget / total
            duty_l *= scale
            duty_r *= scale
            self._limiter_active = True

        if bool(p["invert_left"]):
            duty_l = -duty_l
        if bool(p["invert_right"]):
            duty_r = -duty_r

        self.duty_l = duty_l
        self.duty_r = duty_r
        return (duty_l, duty_r)

    def _wheel_duty(
        self,
        pid: _PI,
        target_rpm: float,
        measured_rpm: float,
        dt: float,
        ceiling: float,
    ) -> float:
        if self.max_rpm_measured <= 0.0:
            return 0.0
        # Feedforward carries the load; the PID only trims. That is what makes
        # this tunable at all when the plant gain is ~2x uncertain.
        ff = clamp(target_rpm / self.max_rpm_measured, -ceiling, ceiling)
        if bool(self.params["closed_loop"]):
            duty = pid.update(target_rpm - measured_rpm, dt, ff, -ceiling, ceiling)
            if pid.saturated:
                self._limiter_active = True
        else:
            duty = ff
        if abs(duty) < 1e-3:
            return 0.0
        # Deadband compensation: below the break-away duty the bridge produces
        # nothing at all, so a small command must be pushed up to the edge of
        # motion rather than quietly doing nothing.
        db = self.plant.stall_deadband
        if abs(duty) < db:
            duty = math.copysign(db, duty)
        return clamp(duty, -ceiling, ceiling)

    # -- plant ----------------------------------------------------------------

    def _step_plant(
        self, duty_l: float, duty_r: float, brake: float, dt: float
    ) -> None:
        p = self.plant

        # Supply: two passes. The first sizes the demand at the nominal rail,
        # the second re-solves at the folded-back rail. A single pass oscillates
        # at the current limit; a full solve is not worth the arithmetic.
        supply = self._supply_voltage(0.0)
        i_l, _ = self._motor_currents(duty_l, self.omega_l, supply)
        i_r, _ = self._motor_currents(duty_r, self.omega_r, supply)
        demand = abs(i_l) + abs(i_r)
        supply = self._supply_voltage(demand)
        i_l, applied_l = self._motor_currents(duty_l, self.omega_l, supply)
        i_r, applied_r = self._motor_currents(duty_r, self.omega_r, supply)
        self.supply_v = supply
        self.current_a = abs(i_l) + abs(i_r)

        radius = float(self.params["wheel_diameter_m"]) * 0.5
        normal = p.mass_kg * GRAVITY * p.rear_weight_fraction * 0.5

        self.omega_l, f_l, self.true_slip_l = self._wheel_step(
            self.omega_l, i_l, applied_l, brake, normal, radius, self._mu(-1), dt
        )
        self.omega_r, f_r, self.true_slip_r = self._wheel_step(
            self.omega_r, i_r, applied_r, brake, normal, radius, self._mu(+1), dt
        )

        drag = p.drag_area * self.v * abs(self.v)
        rolling = p.crr * p.mass_kg * GRAVITY
        rolling_force = 0.0
        if abs(self.v) > 0.01:
            rolling_force = math.copysign(rolling, self.v)
        accel = (f_l + f_r - drag - rolling_force) / p.mass_kg
        self.v += accel * dt
        if abs(self.v) < 0.004 and abs(f_l + f_r) < rolling:
            self.v = 0.0

        # Kinematic bicycle for the body, with the plant supplying the speed.
        #
        # SIGN CONVENTION, and it is worth stating once loudly: `steer_angle` is
        # POSITIVE TO THE RIGHT, matching ControlPacket.steering and therefore
        # what the servo driver and the HUD both use. The world frame is the
        # usual right-handed one (x forward, y left, heading counter-clockwise),
        # so a right turn must DECREASE heading -- hence the minus. Drop it and
        # every plotted path comes out mirrored.
        d = self.v * dt
        d_theta = -d * math.tan(self.steer_angle) / float(self.params["wheelbase_m"])
        self.x += d * math.cos(self.theta + d_theta * 0.5)
        self.y += d * math.sin(self.theta + d_theta * 0.5)
        self.theta = wrap_pi(self.theta + d_theta)

    def _supply_voltage(self, demand_a: float) -> float:
        p = self.plant
        pack = self._pack_v_oc
        if demand_a > 0.0:
            # Boost input current, reflected through efficiency, sags the pack.
            input_a = demand_a * p.boost_v / max(p.boost_efficiency * pack, 0.5)
            pack = max(0.5, self._pack_v_oc - input_a * p.pack_r_int_ohm)
        self.pack_v = pack
        out = min(p.boost_v, max(pack, 0.5) * 1.9)  # a boost cannot exceed its ratio
        if demand_a > p.boost_i_limit_a:
            # Past the limit the S18V20ALV does not current-limit gracefully; it
            # drops out of regulation, both motors lose torque together, and the
            # firmware sees two encoders hit zero at once. That is BROWNOUT, and
            # it is deliberately a different fault code from a mechanical stall.
            out *= p.boost_i_limit_a / demand_a
        return out

    def _motor_currents(
        self, duty: float, omega: float, supply: float
    ) -> tuple[float, float]:
        """Returns (armature current, effective applied volts).

        Zero duty means zero current, not a shorted armature. On the L298, EN
        low is high-Z -- a genuine coast. Braking is IN1 == IN2 with EN HIGH and
        is handled separately, because getting these two backwards shorts a
        spinning motor every time you meant to let it roll.
        """
        p = self.plant
        magnitude = abs(duty)
        if magnitude < 1e-4:
            return (0.0, 0.0)
        rail = max(0.0, supply - p.bridge_drop_v)
        applied = math.copysign(magnitude * rail, duty)
        current = (applied - p.motor_k * omega) / p.motor_r_ohm
        return (current, applied)

    def _wheel_step(
        self,
        omega: float,
        current: float,
        applied_v: float,
        brake: float,
        normal_n: float,
        radius: float,
        mu: float,
        dt: float,
    ) -> tuple[float, float, float]:
        """Advance one wheel. Returns (omega, tractive force, slip ratio)."""
        p = self.plant
        torque = p.motor_k * current

        # Stiction: below the break-away voltage a stopped motor stays stopped.
        # This is the ~12 % deadband. It is gated on the wheel being stopped
        # because that is the real behaviour -- a motor already turning keeps
        # turning well below the duty it needed to start.
        if abs(omega) < 0.15 and abs(applied_v) < p.stall_deadband * max(
            0.0, p.boost_v - p.bridge_drop_v
        ):
            torque = 0.0

        if brake > 0.0:
            # A shorted bridge is a generator into a short: braking torque is
            # k^2*omega/R, scaled by how hard the brake is applied. Note this is
            # IN1 == IN2 with EN high -- coast is EN low, which is the opposite
            # of what everyone expects.
            torque -= brake * p.motor_k * p.motor_k * omega / p.motor_r_ohm

        if abs(omega) > 1e-3:
            torque -= math.copysign(p.coulomb_nm, omega)
            torque -= p.viscous_nm_per_rad_s * omega
        elif abs(torque) < p.coulomb_nm:
            torque = 0.0

        wheel_v = omega * radius
        # The 0.15 m/s floor is not cosmetic. Slip ratio is undefined at rest --
        # the textbook denominator goes to zero -- and without a floor every
        # standing start reports a slip spike of 1.5 that means nothing. The
        # floor is well below the speeds where slip actually matters.
        reference = max(abs(wheel_v), abs(self.v), 0.15)
        slip = clamp((wheel_v - self.v) / reference, -2.0, 2.0)
        # Rational tyre curve: rises to mu*N at `peak_slip`, then falls away as
        # 1/s. Past the peak, spinning harder buys LESS grip, which is exactly
        # why a wheel that lets go keeps going.
        peak = p.peak_slip
        force = mu * normal_n * (2.0 * peak * slip) / (peak * peak + slip * slip)

        omega += (torque - force * radius) / p.inertia_kg_m2 * dt
        if abs(omega) < 1e-4:
            omega = 0.0
        return (omega, force, slip)

    # -- sensors --------------------------------------------------------------

    def _step_encoders(self, dt: float) -> None:
        self._enc_l.step(self.omega_l, dt, self._tick_us)
        self._enc_r.step(self.omega_r, dt, self._tick_us)

        sign_l = 0
        if self.duty_l > 1e-3:
            sign_l = 1
        elif self.duty_l < -1e-3:
            sign_l = -1
        sign_r = 0
        if self.duty_r > 1e-3:
            sign_r = 1
        elif self.duty_r < -1e-3:
            sign_r = -1

        self.rpm_l = self._mt_l.update(
            self._enc_l.total, self._enc_l.last_edge_tick_us, dt, sign_l
        )
        self.rpm_r = self._mt_r.update(
            self._enc_r.total, self._enc_r.last_edge_tick_us, dt, sign_r
        )

        moving = not (self._mt_l.stale and self._mt_r.stale)
        self._direction_uncertain = moving and sign_l == 0 and sign_r == 0

    def _step_odometry(self, dt: float) -> None:
        circumference = math.pi * float(self.params["wheel_diameter_m"])
        cpr = float(self.params["encoder_cpr"])

        total_l = self._enc_l.total
        total_r = self._enc_r.total
        d_counts_l = total_l - self._prev_total_l
        d_counts_r = total_r - self._prev_total_r
        self._prev_total_l = total_l
        self._prev_total_r = total_r

        sign_l = 1.0 if self.rpm_l >= 0.0 else -1.0
        sign_r = 1.0 if self.rpm_r >= 0.0 else -1.0
        if bool(self.params["encoder_invert_left"]):
            sign_l = -sign_l
        if bool(self.params["encoder_invert_right"]):
            sign_r = -sign_r

        d_left = d_counts_l / cpr * circumference * sign_l
        d_right = d_counts_r / cpr * circumference * sign_r
        self.odometry.update(d_left, d_right, self.steer_angle, dt)

    def _step_health(self, dt: float, duty_l: float, duty_r: float) -> None:
        p = self.plant
        self._pack_v_oc = max(
            0.5,
            p.pack_v_full - self.inject.battery_drain_v_per_min * (self._t / 60.0),
        )

        load = (abs(duty_l) + abs(duty_r)) * 0.5
        target_temp = p.cpu_idle_c + p.cpu_load_c * load
        self.cpu_temp_c += (target_temp - self.cpu_temp_c) * (dt / (p.cpu_tau_s + dt))

        commanded_l = abs(duty_l) > p.stall_deadband
        commanded_r = abs(duty_r) > p.stall_deadband
        threshold = float(self.params["stall_rpm_threshold"])
        window = float(self.params["stall_detect_ms"]) / 1000.0

        self._stall_s_l = (
            self._stall_s_l + dt
            if (commanded_l and abs(self.rpm_l) < threshold)
            else 0.0
        )
        self._stall_s_r = (
            self._stall_s_r + dt
            if (commanded_r and abs(self.rpm_r) < threshold)
            else 0.0
        )

        if self._stall_s_l > window and self._stall_s_r > window:
            # Both at once while commanded is the regulator letting go, not two
            # simultaneous mechanical stalls. Different cause, different fix,
            # different fault bit.
            self.faults |= Fault.BROWNOUT
        else:
            if self._stall_s_l > window:
                self.faults |= Fault.STALL_L
            if self._stall_s_r > window:
                self.faults |= Fault.STALL_R

        if self._enc_l.dead and commanded_l:
            self.faults |= Fault.ENCODER_FAIL_L
        if self._enc_r.dead and commanded_r:
            self.faults |= Fault.ENCODER_FAIL_R

        low = float(self.params["low_battery_v"])
        critical = float(self.params["critical_battery_v"])
        if low > 0.0 and self.pack_v < low:
            self.faults |= Fault.LOW_BATTERY
        if critical > 0.0 and self.pack_v < critical:
            self.faults |= Fault.CRITICAL_BATTERY

        if self.state == VehicleState.SAFE and self.faults & CRITICAL_FAULTS:
            self.state = VehicleState.FAULT

    # -- output ---------------------------------------------------------------

    def _flags(self) -> TelemetryFlags:
        flags = TelemetryFlags.ODOM_VALID | TelemetryFlags.CALIBRATED
        if self._limiter_active:
            flags |= TelemetryFlags.LIMITER_ACTIVE
        if self._direction_uncertain:
            flags |= TelemetryFlags.DIRECTION_UNCERTAIN
        if bool(self.params["closed_loop"]):
            flags |= TelemetryFlags.CLOSED_LOOP
        if self._reverse_engaged:
            flags |= TelemetryFlags.REVERSE_ENGAGED
        if self.brake_cmd > 0.0:
            flags |= TelemetryFlags.BRAKING
        if self.video_active:
            flags |= TelemetryFlags.VIDEO_ACTIVE
        return flags

    def _snapshot(self) -> SimSnapshot:
        x, y, heading = self.odometry.pose
        circumference = math.pi * float(self.params["wheel_diameter_m"])
        # Reported speed comes from the encoders, like the car's does, so the
        # HUD sees the same chunkiness at low speed that it will see for real.
        speed = (self.rpm_l + self.rpm_r) * 0.5 / 60.0 * circumference
        throttled = 0
        if self.faults & Fault.LOW_BATTERY:
            throttled |= 0x50000  # under-voltage has occurred, per vcgencmd
        return SimSnapshot(
            t=self._t,
            state=self.state,
            faults=self.faults,
            flags=self._flags(),
            rpm_l=self.rpm_l,
            rpm_r=self.rpm_r,
            rpm_target_l=self.rpm_target_l,
            rpm_target_r=self.rpm_target_r,
            duty_l=self.duty_l,
            duty_r=self.duty_r,
            servo_us=self.servo_us,
            steer_angle=self.steer_angle,
            speed=speed,
            v_max=self.v_max,
            x=x,
            y=y,
            heading=heading,
            distance=self.odometry.distance,
            slip=self.odometry.slip_index,
            true_x=self.x,
            true_y=self.y,
            true_heading=self.theta,
            true_speed=self.v,
            true_slip_l=self.true_slip_l,
            true_slip_r=self.true_slip_r,
            pack_v=self.pack_v,
            supply_v=self.supply_v,
            current_a=self.current_a,
            cpu_temp_c=self.cpu_temp_c,
            throttled=throttled,
            loop_p99_us=self.loop_p99_us,
        )


# --------------------------------------------------------------------------
# Loop statistics
# --------------------------------------------------------------------------


@dataclass(slots=True)
class JitterStats:
    """Streaming period statistics. p99 is the number that matters."""

    _samples: list[float] = field(default_factory=list)
    _limit: int = 4096
    count: int = 0
    worst: float = 0.0

    def add(self, dt: float) -> None:
        self.count += 1
        self.worst = max(self.worst, dt)
        if len(self._samples) < self._limit:
            self._samples.append(dt)
        else:
            self._samples[self.count % self._limit] = dt

    def p99_us(self) -> int:
        if not self._samples:
            return 0
        ordered = sorted(self._samples)
        idx = min(len(ordered) - 1, int(len(ordered) * 0.99))
        return int(ordered[idx] * 1_000_000.0)

    def reset(self) -> None:
        self._samples.clear()
        self.count = 0
        self.worst = 0.0
