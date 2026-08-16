"""A GPIO backend with a drivetrain underneath it.

This is what lets most of the firmware be written and tested on a laptop. A mock
that merely records pin writes proves that the code ran; a mock that turns those
pin writes into wheel speeds, encoder edges and supply current proves that the
control loop *closes*. The difference shows up the first time a PID gain is
wrong: against a recording mock the test passes, against this one the wheels
never reach the setpoint.

The model is small but it is physics, not a curve fit, and the numbers below
come out of it rather than being pasted in:

* **~0.22 duty of deadband.** Not a fudge factor -- the L298N is a Darlington
  bridge with ~1.4 V of fixed drop, and 1.4 V of a 9 V rail is 0.16 duty before
  any current flows at all. Stiction accounts for the rest.
* **~0.14 s mechanical time constant.** J / (kt*ke/R + b), which is what a first
  order lag actually is. Modelling inertia instead of hard-coding tau means the
  lag correctly changes with rail voltage and load.
* **150-200 RPM top speed, not the nameplate 360.** Falls out of the back-EMF
  balance at 9 V through a bridge that eats 1.4 V of it.
* **Regulator hiccup under simultaneous load.** A full-throttle step from rest
  demands ~4 A from a converter that sustains 1.5 A. It trips, both wheels coast
  to zero *together*, and that simultaneity is precisely the signature the
  firmware's brownout inference looks for. A model that never browns out would
  let that code be written and never exercised.

Advance it with :meth:`MockBackend.step`, in lockstep with a ``FakeClock``::

    clock.advance(dt)
    backend.step(dt)

Edges are timestamped inside the interval that *ends* at the clock's current
time, so M/T velocity estimation sees realistic inter-edge microseconds.
"""

from __future__ import annotations

import enum
import math
import random
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ..constants import (
    BRIDGE_DROP_V,
    BRIDGE_OHMS,
    DEFAULT_ENCODER_CPR,
    HW_PWM_CHANNEL_0_PINS,
    HW_PWM_CHANNEL_1_PINS,
    MOTOR_KE_V_S_PER_RAD,
    MOTOR_KT_NM_PER_A,
    MOTOR_WINDING_OHMS,
    PACK_NOMINAL_V,
    PIN_ENA,
    PIN_ENB,
    PIN_ENC_L_A,
    PIN_ENC_L_B,
    PIN_ENC_R_A,
    PIN_ENC_R_B,
    PIN_ESTOP_BUTTON,
    PIN_IN1,
    PIN_IN2,
    PIN_IN3,
    PIN_IN4,
    PIN_SERVO,
    PIN_STATUS_LED,
    RAIL_NOMINAL_V,
    REGULATOR_RECOVERY_S,
    REGULATOR_TRIP_A,
    REGULATOR_TRIP_DELAY_S,
    SERVO_NOMINAL_CENTER_US,
    SERVO_SLEW_US_PER_S,
    rad_s_to_rpm,
)
from ..util.clock import Clock
from ..util.ringbuf import RingBuffer
from .base import CallbackHandle, Edge, EdgeCallback, GpioBackend, GpioError, Pull

if TYPE_CHECKING:  # avoids a runtime dependency of the HAL on the config layer
    from ..config import HardwarePins

LEFT = "left"
RIGHT = "right"

#: Angular speeds below this count as stopped, so stiction can hold the wheel.
_OMEGA_EPS = 1e-4

#: Integration substep. dt is 10 ms in the firmware and the fastest mode in the
#: model has a ~140 ms time constant, so this is about accuracy at large steps
#: rather than stability -- a test that calls step(0.5) still gets sane numbers.
_MAX_SUBSTEP_S = 0.005

#: Refuses to synthesise more than this many edges in one call. Without it,
#: step(3600.0) would try to deliver seven million callbacks.
_MAX_EDGES_PER_STEP = 20_000


class DriveMode(enum.IntEnum):
    """What the H-bridge is actually doing, derived from EN and the IN pair.

    BRAKE is IN1 == IN2 with the enable high. That is genuinely how an L298
    behaves and it is the opposite of most people's intuition: coasting requires
    dropping the enable, and 'both low' shorts the motor through the low-side
    transistors just as surely as 'both high' shorts it through the high side.
    """

    COAST = 0
    FORWARD = 1
    REVERSE = 2
    BRAKE = 3


# --------------------------------------------------------------------------
# Plant
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class PlantParams:
    """Physical constants of one TeleKart drivetrain, at the output shaft.

    Referring everything to the output shaft rather than the motor shaft keeps
    the gearbox ratio out of every equation, and the encoder is specified in
    counts per output revolution anyway.
    """

    # --- electrical ---
    rail_nominal_v: float = RAIL_NOMINAL_V
    #: Output impedance of the boost converter plus wiring. Produces the sag
    #: under load that the firmware sees as reduced top speed when both motors
    #: pull together.
    rail_source_ohms: float = 0.35
    #: Output capacitance seen as a first-order lag on the rail.
    rail_tau_s: float = 0.020
    regulator_trip_a: float = REGULATOR_TRIP_A
    regulator_trip_delay_s: float = REGULATOR_TRIP_DELAY_S
    regulator_recovery_s: float = REGULATOR_RECOVERY_S
    #: Below this pack voltage the converter can no longer hold 9 V out.
    pack_knee_v: float = 6.0
    quiescent_a: float = 0.05

    bridge_drop_v: float = BRIDGE_DROP_V
    bridge_ohms: float = BRIDGE_OHMS
    winding_ohms: float = MOTOR_WINDING_OHMS

    # --- mechanical ---
    ke_v_s_per_rad: float = MOTOR_KE_V_S_PER_RAD
    kt_nm_per_a: float = MOTOR_KT_NM_PER_A
    #: Lumped: rotor inertia reflected through the 30:1 gearbox, the wheels, and
    #: the share of vehicle mass each wheel accelerates. Fitted so the
    #: small-signal time constant lands at the ~0.15 s the real car shows.
    inertia_kg_m2: float = 5.05e-3
    viscous_nm_s_per_rad: float = 0.00716
    #: Rolling resistance and gearbox drag once moving.
    coulomb_nm: float = 0.030
    #: Extra torque needed to break away from rest. This plus the bridge drop is
    #: where the ~0.22 duty deadband comes from.
    stiction_nm: float = 0.020
    #: Tyre scrub at full lock. Ackermann front steering with a solid rear axle
    #: drags the inside rear wheel sideways; it costs real current.
    steer_scrub_nm: float = 0.020

    # --- encoder ---
    cpr: int = DEFAULT_ENCODER_CPR

    # --- battery ---
    pack_nominal_v: float = PACK_NOMINAL_V
    pack_drain_v_per_s: float = 0.0

    # --- steering ---
    servo_center_us: float = SERVO_NOMINAL_CENTER_US
    servo_half_range_us: float = 400.0
    servo_slew_us_per_s: float = SERVO_SLEW_US_PER_S

    @property
    def electrical_ohms(self) -> float:
        return self.winding_ohms + self.bridge_ohms

    @property
    def time_constant_s(self) -> float:
        """Small-signal mechanical time constant, for sanity checks in tests."""
        damping = (
            self.kt_nm_per_a * self.ke_v_s_per_rad / self.electrical_ohms
            + self.viscous_nm_s_per_rad
        )
        return self.inertia_kg_m2 / damping

    def deadband_duty(self, rail_v: float | None = None) -> float:
        """Duty at which a stopped wheel just breaks away."""
        rail = self.rail_nominal_v if rail_v is None else rail_v
        if rail <= 0.0:
            return 1.0
        static = self.stiction_nm + self.coulomb_nm
        current = static / self.kt_nm_per_a
        volts = self.bridge_drop_v + current * self.electrical_ohms
        return min(1.0, volts / rail)


class WheelPlant:
    """One motor, one gearbox, one wheel, one encoder disc."""

    __slots__ = (
        "name", "omega", "phase", "current_a", "rail_current_a",
        "mode", "duty", "blocked", "encoder_dead", "total_counts", "_params",
    )

    def __init__(self, name: str, params: PlantParams) -> None:
        self.name = name
        self._params = params
        self.omega = 0.0  # rad/s at the output shaft, + is the FORWARD pin sense
        self.phase = 0.0  # encoder counts, fractional and signed
        self.current_a = 0.0
        self.rail_current_a = 0.0
        self.mode = DriveMode.COAST
        self.duty = 0.0
        #: Wheel physically jammed -- a kerb, a stone, a hand. Draws stall
        #: current and turns not at all, which is exactly what the firmware's
        #: stall detector exists to catch before the bridge cooks.
        self.blocked = False
        #: Encoder wiring failure: the wheel turns, nothing counts.
        self.encoder_dead = False
        self.total_counts = 0

    def set_command(self, mode: DriveMode, duty: float) -> None:
        self.mode = mode
        self.duty = 0.0 if duty <= 0.0 else (1.0 if duty >= 1.0 else duty)

    def substep(self, dt: float, rail_v: float, scrub_nm: float, cpr: int) -> None:
        params = self._params
        if self.blocked:
            self.omega = 0.0

        omega = self.omega
        mode = self.mode

        if mode is DriveMode.COAST:
            # Enable low means both halves of the bridge are off: the winding is
            # open, so no current flows regardless of back-EMF.
            current = 0.0
            rail_current = 0.0
        else:
            back_emf = params.ke_v_s_per_rad * omega
            if mode is DriveMode.BRAKE:
                applied = 0.0
            elif mode is DriveMode.FORWARD:
                applied = self.duty * rail_v
            else:
                applied = -self.duty * rail_v

            net_v = applied - back_emf
            magnitude = abs(net_v) - params.bridge_drop_v
            if magnitude <= 0.0:
                current = 0.0
            else:
                current = math.copysign(magnitude / params.electrical_ohms, net_v)

            if mode is DriveMode.BRAKE:
                # The bridge is only shorted for `duty` of each PWM period, so
                # the average braking current scales with it.
                current *= self.duty
                # Braking current comes out of the motor's own kinetic energy,
                # not out of the converter.
                rail_current = 0.0
            else:
                # Averaged over a PWM period the rail sees duty * motor current.
                # The sign matters and is not academic: when the rail collapses
                # during a regulator hiccup, back-EMF exceeds the applied
                # voltage, current reverses through the freewheel diodes, and
                # the motor brakes itself instead of coasting. That is why both
                # wheels decay together during a brownout, which is the signal
                # the firmware uses to tell a brownout from a stall.
                rail_current = self.duty * current
                if applied < 0.0:
                    rail_current = -rail_current
                if rail_current < 0.0:
                    # The converter cannot sink; recovered energy just charges
                    # the output capacitor.
                    rail_current = 0.0

        torque = params.kt_nm_per_a * current

        if self.blocked:
            self.current_a = current
            self.rail_current_a = rail_current
            return

        if -_OMEGA_EPS < omega < _OMEGA_EPS:
            static = params.stiction_nm + params.coulomb_nm + scrub_nm
            if abs(torque) <= static:
                omega = 0.0
            else:
                net = torque - math.copysign(static, torque)
                omega = net / params.inertia_kg_m2 * dt
        else:
            kinetic = (
                params.coulomb_nm + scrub_nm + params.viscous_nm_s_per_rad * abs(omega)
            )
            net = torque - math.copysign(kinetic, omega)
            updated = omega + net / params.inertia_kg_m2 * dt
            if updated * omega < 0.0 and abs(torque) < params.coulomb_nm + scrub_nm:
                # Friction can bring a wheel to rest; it cannot drive it backwards.
                updated = 0.0
            omega = updated

        # Trapezoidal phase update: using the mean speed over the substep keeps
        # the synthesised edge count right during hard acceleration, where the
        # end-of-step speed would overcount by several percent.
        self.phase += (self.omega + omega) * 0.5 / math.tau * cpr * dt
        self.omega = omega
        self.current_a = current
        self.rail_current_a = rail_current

    # -- views --------------------------------------------------------------

    @property
    def rpm(self) -> float:
        return rad_s_to_rpm(self.omega)

    def reset(self) -> None:
        self.omega = 0.0
        self.phase = 0.0
        self.current_a = 0.0
        self.rail_current_a = 0.0
        self.mode = DriveMode.COAST
        self.duty = 0.0
        self.total_counts = 0

    def __repr__(self) -> str:
        return (
            f"WheelPlant({self.name}, rpm={self.rpm:.1f}, mode={self.mode.name}, "
            f"duty={self.duty:.2f}, i={self.current_a:.2f}A)"
        )


class BoostRegulator:
    """Pololu S18V20ALV in the role it actually plays here: the speed limit.

    A 7.2 V NiMH pack boosted to 9 V will not sustain the ~4 A that two motors
    demand off the line. The converter droops, then folds back into hiccup mode:
    output off for a few hundred milliseconds, retry, repeat. Modelled because
    the firmware has to tell that apart from a mechanical stall -- both look like
    'commanded but not turning', and the fix for one is nothing like the fix for
    the other.
    """

    __slots__ = ("_params", "voltage", "load_a", "tripped", "trip_count",
                 "_over_s", "_recover_s", "pack_v", "_elapsed")

    def __init__(self, params: PlantParams) -> None:
        self._params = params
        self.pack_v = params.pack_nominal_v
        self.voltage = params.rail_nominal_v
        self.load_a = 0.0
        self.tripped = False
        self.trip_count = 0
        self._over_s = 0.0
        self._recover_s = 0.0
        self._elapsed = 0.0

    @property
    def ceiling_v(self) -> float:
        """What the converter can hold given the pack's present state."""
        params = self._params
        if self.pack_v >= params.pack_knee_v:
            return params.rail_nominal_v
        if self.pack_v <= 0.0:
            return 0.0
        return params.rail_nominal_v * (self.pack_v / params.pack_knee_v)

    def step(self, dt: float, load_a: float) -> float:
        params = self._params
        self._elapsed += dt
        if params.pack_drain_v_per_s:
            self.pack_v = max(0.0, self.pack_v - params.pack_drain_v_per_s * dt)

        total = load_a + params.quiescent_a
        self.load_a = total

        if self.tripped:
            self._recover_s -= dt
            if self._recover_s <= 0.0:
                self.tripped = False
                self._over_s = 0.0
            target = 0.0
        else:
            if total > params.regulator_trip_a:
                self._over_s += dt
            else:
                self._over_s = max(0.0, self._over_s - dt)
            if self._over_s >= params.regulator_trip_delay_s:
                self.trip()
                target = 0.0
            else:
                target = max(0.0, self.ceiling_v - params.rail_source_ohms * total)

        # First-order because the output capacitor cannot change instantly --
        # this is what makes the collapse take a few milliseconds rather than
        # appearing between two loop ticks.
        alpha = 1.0 - math.exp(-dt / params.rail_tau_s) if params.rail_tau_s > 0 else 1.0
        self.voltage += (target - self.voltage) * alpha
        if self.voltage < 0.0:
            self.voltage = 0.0
        return self.voltage

    def trip(self, duration: float | None = None) -> None:
        """Force hiccup mode. Fault injection uses this to reproduce a brownout
        without having to arrange the current draw that causes one."""
        if not self.tripped:
            self.trip_count += 1
        self.tripped = True
        self._recover_s = (
            self._params.regulator_recovery_s if duration is None else duration
        )
        self._over_s = 0.0

    def reset(self) -> None:
        self.pack_v = self._params.pack_nominal_v
        self.voltage = self._params.rail_nominal_v
        self.load_a = 0.0
        self.tripped = False
        self.trip_count = 0
        self._over_s = 0.0
        self._recover_s = 0.0
        self._elapsed = 0.0

    def __repr__(self) -> str:
        return (
            f"BoostRegulator(v={self.voltage:.2f}, load={self.load_a:.2f}A, "
            f"tripped={self.tripped}, trips={self.trip_count})"
        )


class DrivetrainPlant:
    """Both wheels, the converter that feeds them, and the steering servo.

    Deliberately free of any GPIO knowledge so the simulator in ``tools/`` can
    drive the same physics through a different front end.
    """

    __slots__ = ("params", "left", "right", "regulator", "cpr",
                 "servo_target_us", "servo_us", "elapsed")

    def __init__(self, params: PlantParams | None = None) -> None:
        self.params = params or PlantParams()
        self.left = WheelPlant(LEFT, self.params)
        self.right = WheelPlant(RIGHT, self.params)
        self.regulator = BoostRegulator(self.params)
        #: Mutable so a test can match the plant to a non-default encoder_cpr
        #: without rebuilding the frozen parameter set.
        self.cpr = self.params.cpr
        self.servo_target_us = 0.0
        self.servo_us = float(self.params.servo_center_us)
        self.elapsed = 0.0

    # -- inputs -------------------------------------------------------------

    def set_command(self, side: str, mode: DriveMode, duty: float) -> None:
        self.wheel(side).set_command(mode, duty)

    def set_servo_pulse(self, pulse_us: float) -> None:
        self.servo_target_us = float(pulse_us)

    def wheel(self, side: str) -> WheelPlant:
        if side == LEFT:
            return self.left
        if side == RIGHT:
            return self.right
        raise ValueError(f"unknown wheel {side!r}; expected {LEFT!r} or {RIGHT!r}")

    # -- integration --------------------------------------------------------

    def step(self, dt: float) -> None:
        if dt <= 0.0:
            return
        substeps = int(dt / _MAX_SUBSTEP_S) + 1
        h = dt / substeps
        params = self.params
        for _ in range(substeps):
            self._advance_servo(h)
            scrub = params.steer_scrub_nm * abs(self.steer_fraction)
            rail = self.regulator.voltage
            self.left.substep(h, rail, scrub, self.cpr)
            self.right.substep(h, rail, scrub, self.cpr)
            self.regulator.step(
                h, self.left.rail_current_a + self.right.rail_current_a
            )
        self.elapsed += dt

    def _advance_servo(self, dt: float) -> None:
        # A pulse width of zero means the pulse train stopped. A real servo goes
        # limp; modelling it as holding position is close enough for odometry
        # work and avoids inventing a restoring force the car does not have.
        if self.servo_target_us <= 0.0:
            return
        limit = self.params.servo_slew_us_per_s * dt
        error = self.servo_target_us - self.servo_us
        if error > limit:
            error = limit
        elif error < -limit:
            error = -limit
        self.servo_us += error

    # -- views --------------------------------------------------------------

    @property
    def steer_fraction(self) -> float:
        """-1..+1 about the nominal centre. Only used for scrub friction, so it
        does not need the calibrated centre from the config."""
        params = self.params
        if params.servo_half_range_us <= 0.0:
            return 0.0
        value = (self.servo_us - params.servo_center_us) / params.servo_half_range_us
        return -1.0 if value < -1.0 else (1.0 if value > 1.0 else value)

    @property
    def rail_voltage(self) -> float:
        return self.regulator.voltage

    @property
    def current_total_a(self) -> float:
        return self.left.rail_current_a + self.right.rail_current_a

    @property
    def brownout(self) -> bool:
        return self.regulator.tripped

    def reset(self) -> None:
        self.left.reset()
        self.right.reset()
        self.regulator.reset()
        self.servo_target_us = 0.0
        self.servo_us = float(self.params.servo_center_us)
        self.elapsed = 0.0

    def __repr__(self) -> str:
        return (
            f"DrivetrainPlant(l={self.left.rpm:.1f}rpm, r={self.right.rpm:.1f}rpm, "
            f"rail={self.rail_voltage:.2f}V, servo={self.servo_us:.0f}us)"
        )


# --------------------------------------------------------------------------
# Pin map
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class MockPins:
    """Which simulated pin is which. Defaults match how the car is wired.

    Held as plain integers rather than as a :class:`~telekart.config.HardwarePins`
    so the HAL layer keeps no dependency on the config layer.
    """

    ena: int = PIN_ENA
    in1: int = PIN_IN1
    in2: int = PIN_IN2
    in3: int = PIN_IN3
    in4: int = PIN_IN4
    enb: int = PIN_ENB
    servo: int = PIN_SERVO
    enc_l_a: int = PIN_ENC_L_A
    enc_l_b: int = PIN_ENC_L_B
    enc_r_a: int = PIN_ENC_R_A
    enc_r_b: int = PIN_ENC_R_B
    status_led: int | None = PIN_STATUS_LED
    estop_button: int | None = PIN_ESTOP_BUTTON

    @classmethod
    def from_hardware_pins(cls, pins: "HardwarePins") -> "MockPins":
        return cls(
            ena=pins.motors.ena,
            in1=pins.motors.in1,
            in2=pins.motors.in2,
            in3=pins.motors.in3,
            in4=pins.motors.in4,
            enb=pins.motors.enb,
            servo=pins.servo,
            enc_l_a=pins.encoders.left_a,
            enc_l_b=pins.encoders.left_b,
            enc_r_a=pins.encoders.right_a,
            enc_r_b=pins.encoders.right_b,
            status_led=pins.status_led,
            estop_button=pins.estop_button,
        )


class _MockCallback:
    __slots__ = ("pin", "edge", "func", "cancelled")

    def __init__(self, pin: int, edge: Edge, func: EdgeCallback) -> None:
        self.pin = pin
        self.edge = edge
        self.func = func
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True

    def wants(self, level: int) -> bool:
        if self.cancelled:
            return False
        if self.edge is Edge.BOTH:
            return True
        if self.edge is Edge.RISING:
            return level == 1
        return level == 0


@dataclass(slots=True)
class MockOp:
    """One recorded backend operation, for assertions about ordering."""

    t_us: int
    kind: str
    pin: int
    value: float


# --------------------------------------------------------------------------
# Backend
# --------------------------------------------------------------------------


class MockBackend(GpioBackend):
    """:class:`~telekart.hal.base.GpioBackend` on top of :class:`DrivetrainPlant`."""

    def __init__(
        self,
        *,
        pins: MockPins | None = None,
        plant: DrivetrainPlant | None = None,
        params: PlantParams | None = None,
        clock: Clock | None = None,
        seed: int = 0,
        edge_jitter_us: float = 0.0,
        history: int = 256,
        strict: bool = True,
    ) -> None:
        self.pins = pins or MockPins()
        self.plant = plant or DrivetrainPlant(params)
        self.clock = clock
        # Raising on a misused interface is the mock's job. It never runs on the
        # car, so a loud failure here is free, and the pair-PWM rule is exactly
        # the kind of mistake that stays invisible until the hardware behaves
        # strangely on a bench with the wheels off the ground.
        self.strict = strict

        self._rng = random.Random(seed)
        self._edge_jitter_us = edge_jitter_us

        self._levels: dict[int, bool] = {}
        self._modes: dict[int, str] = {}
        self._pulls: dict[int, Pull] = {}
        self._glitch_us: dict[int, int] = {}
        self._last_edge_us: dict[int, int] = {}
        self._callbacks: dict[int, list[_MockCallback]] = {}

        self._pwm_pins: tuple[int, int] | None = None
        self._pwm_freq = 0
        self._pwm_duty: dict[int, float] = {self.pins.ena: 0.0, self.pins.enb: 0.0}
        self._servo_us: dict[int, int] = {}

        self._tick_us = 0
        self._cleaned = False

        self.history: RingBuffer[MockOp] = RingBuffer(history)
        self.edges_delivered = 0
        self.callback_errors = 0
        self.dropped_by_glitch = 0

        # The E-stop button is wired to ground with a pull-up, so idle is HIGH.
        if self.pins.estop_button is not None:
            self._levels[self.pins.estop_button] = True

    # -- digital ------------------------------------------------------------

    def setup_output(self, pin: int, initial: bool = False) -> None:
        self._modes[pin] = "out"
        self._levels[pin] = bool(initial)
        self._record("setup_output", pin, 1.0 if initial else 0.0)

    def write(self, pin: int, value: bool) -> None:
        if self.strict and self._modes.get(pin) not in ("out", None):
            raise GpioError(f"write to GPIO{pin}, which was configured as an input")
        self._levels[pin] = bool(value)
        self._record("write", pin, 1.0 if value else 0.0)

    def setup_input(self, pin: int, pull: Pull = Pull.NONE, glitch_us: int = 0) -> None:
        self._modes[pin] = "in"
        self._pulls[pin] = pull
        self._glitch_us[pin] = max(0, int(glitch_us))
        if pin not in self._levels:
            self._levels[pin] = pull is Pull.UP
        self._record("setup_input", pin, float(glitch_us))

    def read(self, pin: int) -> bool:
        encoder = self._encoder_channel(pin)
        if encoder is not None:
            wheel, offset = encoder
            return _channel_level(wheel.phase + offset)
        return self._levels.get(pin, False)

    # -- PWM ----------------------------------------------------------------

    def set_pwm_pair(
        self, pin_a: int, pin_b: int, freq_hz: int, duty_a: float, duty_b: float
    ) -> None:
        if self.strict:
            if pin_a not in HW_PWM_CHANNEL_0_PINS:
                raise GpioError(f"GPIO{pin_a} is not on hardware PWM channel 0")
            if pin_b not in HW_PWM_CHANNEL_1_PINS:
                raise GpioError(f"GPIO{pin_b} is not on hardware PWM channel 1")
            if self._pwm_pins is not None and self._pwm_pins != (pin_a, pin_b):
                raise GpioError(
                    f"PWM pair changed from {self._pwm_pins} to {(pin_a, pin_b)}"
                )
        self._pwm_pins = (pin_a, pin_b)
        self._pwm_freq = int(freq_hz)
        self._pwm_duty[pin_a] = _clamp01(duty_a)
        self._pwm_duty[pin_b] = _clamp01(duty_b)
        self._record("pwm", pin_a, self._pwm_duty[pin_a])
        self._record("pwm", pin_b, self._pwm_duty[pin_b])

    # -- servo --------------------------------------------------------------

    def set_servo_pulse(self, pin: int, pulse_us: int) -> None:
        pulse = int(pulse_us)
        self._servo_us[pin] = pulse
        if pin == self.pins.servo:
            self.plant.set_servo_pulse(pulse)
        self._record("servo", pin, float(pulse))

    # -- edges --------------------------------------------------------------

    def add_edge_callback(
        self, pin: int, edge: Edge, callback: EdgeCallback
    ) -> CallbackHandle:
        handle = _MockCallback(pin, edge, callback)
        self._callbacks.setdefault(pin, []).append(handle)
        return handle

    # -- misc ---------------------------------------------------------------

    def ticks_us(self) -> int:
        if self.clock is not None:
            return self.clock.monotonic_us() & 0xFFFFFFFF
        return self._tick_us & 0xFFFFFFFF

    def cleanup(self) -> None:
        if self._cleaned:
            return
        self._cleaned = True
        for pin in self._pwm_duty:
            self._pwm_duty[pin] = 0.0
        for pin in list(self._servo_us):
            self._servo_us[pin] = 0
        self.plant.set_servo_pulse(0)
        for handles in self._callbacks.values():
            for handle in handles:
                handle.cancel()
        self._callbacks.clear()
        self._apply_commands()
        self._record("cleanup", -1, 0.0)

    # -- simulation ---------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance the plant by ``dt`` and deliver any encoder edges it produced.

        Call *after* advancing the clock: the interval simulated is the one that
        ends now, so an edge timestamp is never in the future relative to a
        control tick that is about to read it.
        """
        if dt <= 0.0:
            return
        dt_us = dt * 1_000_000.0
        if self.clock is not None:
            end_us = self.clock.monotonic_us()
        else:
            self._tick_us += int(round(dt_us))
            end_us = self._tick_us
        start_us = end_us - dt_us

        self._apply_commands()

        left_before = self.plant.left.phase
        right_before = self.plant.right.phase
        self.plant.step(dt)

        self._emit_wheel_edges(
            self.plant.left, self.pins.enc_l_a, self.pins.enc_l_b,
            left_before, start_us, dt_us,
        )
        self._emit_wheel_edges(
            self.plant.right, self.pins.enc_r_a, self.pins.enc_r_b,
            right_before, start_us, dt_us,
        )

    def run_for(self, seconds: float, dt: float = 0.01) -> None:
        """Step repeatedly, advancing an attached FakeClock alongside.

        The convenience that makes a sixty-second drive test three lines long.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        advance = getattr(self.clock, "advance", None)
        remaining = seconds
        while remaining > 1e-12:
            slice_s = dt if remaining >= dt else remaining
            if advance is not None:
                advance(slice_s)
            self.step(slice_s)
            remaining -= slice_s

    # -- command decoding ---------------------------------------------------

    def _apply_commands(self) -> None:
        """Turn the current pin state into a drive mode for each wheel."""
        pins = self.pins
        self.plant.left.set_command(
            *_decode_bridge(
                self._pwm_duty.get(pins.ena, 0.0),
                self._levels.get(pins.in1, False),
                self._levels.get(pins.in2, False),
            )
        )
        self.plant.right.set_command(
            *_decode_bridge(
                self._pwm_duty.get(pins.enb, 0.0),
                self._levels.get(pins.in3, False),
                self._levels.get(pins.in4, False),
            )
        )

    # -- edge synthesis -----------------------------------------------------

    def _emit_wheel_edges(
        self,
        wheel: WheelPlant,
        pin_a: int,
        pin_b: int,
        phase_before: float,
        start_us: float,
        dt_us: float,
    ) -> None:
        phase_after = wheel.phase
        # True wheel motion, counted whether or not the encoder is reporting it.
        # A test can then assert "the wheel turned 200 counts and the firmware
        # saw none", which is the whole point of the encoder-failure injection.
        wheel.total_counts += abs(
            int(math.floor(phase_after)) - int(math.floor(phase_before))
        )
        if wheel.encoder_dead:
            return
        self._emit_channel(pin_a, phase_before, phase_after, start_us, dt_us, 0.0)
        # Channel B is a fixed quarter-cycle away in *space*; the lead/lag
        # relationship therefore flips with direction on its own, which is what
        # makes it a usable disagreement check. Only synthesised when something
        # is listening, because the firmware polls B rather than watching it.
        if self._callbacks.get(pin_b):
            self._emit_channel(pin_b, phase_before, phase_after, start_us, dt_us, -0.5)

    def _emit_channel(
        self,
        pin: int,
        phase_before: float,
        phase_after: float,
        start_us: float,
        dt_us: float,
        offset: float,
    ) -> None:
        listeners = self._callbacks.get(pin)
        if not listeners:
            return
        p0 = phase_before + offset
        p1 = phase_after + offset
        if p0 == p1:
            return
        span = p1 - p0
        if span > 0.0:
            k = math.floor(p0) + 1.0
            direction = 1.0
        else:
            k = math.ceil(p0) - 1.0
            direction = -1.0

        emitted = 0
        while (direction > 0.0 and k <= p1) or (direction < 0.0 and k >= p1):
            fraction = (k - p0) / span
            tick = start_us + fraction * dt_us
            if self._edge_jitter_us:
                tick += self._rng.uniform(-self._edge_jitter_us, self._edge_jitter_us)
            index = int(k) if direction > 0.0 else int(k) - 1
            level = 1 if index % 2 == 0 else 0
            self._deliver(pin, level, int(tick) & 0xFFFFFFFF, listeners)
            k += direction
            emitted += 1
            if emitted >= _MAX_EDGES_PER_STEP:
                break

    def _deliver(
        self, pin: int, level: int, tick_us: int, listeners: list[_MockCallback]
    ) -> None:
        glitch = self._glitch_us.get(pin, 0)
        if glitch:
            last = self._last_edge_us.get(pin)
            if last is not None and 0 <= (tick_us - last) & 0xFFFFFFFF < glitch:
                self.dropped_by_glitch += 1
                return
        self._last_edge_us[pin] = tick_us
        self.edges_delivered += 1
        for handle in listeners:
            if not handle.wants(level):
                continue
            try:
                handle.func(pin, level, tick_us)
            except Exception:  # noqa: BLE001
                # pigpio swallows exceptions in its notification thread too. A
                # test that relies on one propagating would be testing a
                # behaviour the real backend does not have.
                self.callback_errors += 1

    def inject_edge(self, pin: int, level: int, tick_us: int | None = None) -> None:
        """Deliver an edge by hand -- for testing noise rejection and the
        encoder driver's handling of edges that the plant would never produce."""
        listeners = self._callbacks.get(pin)
        if not listeners:
            return
        self._deliver(pin, 1 if level else 0, tick_us if tick_us is not None else self.ticks_us(), listeners)

    # -- test controls ------------------------------------------------------

    def set_input(self, pin: int, level: bool) -> None:
        """Drive a simulated input, such as the E-stop button."""
        self._levels[pin] = bool(level)

    def press_estop(self) -> None:
        """The button pulls the pin to ground, so pressed reads LOW."""
        if self.pins.estop_button is not None:
            self._levels[self.pins.estop_button] = False

    def release_estop(self) -> None:
        if self.pins.estop_button is not None:
            self._levels[self.pins.estop_button] = True

    def block_wheel(self, side: str, blocked: bool = True) -> None:
        """Jam a wheel: it stops turning and keeps drawing stall current."""
        self.plant.wheel(side).blocked = blocked

    def fail_encoder(self, side: str, dead: bool = True) -> None:
        """Silence one encoder while the wheel keeps turning."""
        self.plant.wheel(side).encoder_dead = dead

    def trip_regulator(self, duration: float | None = None) -> None:
        self.plant.regulator.trip(duration)

    def set_pack_voltage(self, volts: float) -> None:
        self.plant.regulator.pack_v = max(0.0, volts)

    # -- introspection ------------------------------------------------------

    def pwm_duty(self, pin: int) -> float:
        return self._pwm_duty.get(pin, 0.0)

    @property
    def pwm_freq(self) -> int:
        return self._pwm_freq

    def pin_level(self, pin: int) -> bool:
        return self._levels.get(pin, False)

    def pin_mode(self, pin: int) -> str:
        """``"in"``, ``"out"``, or ``""`` when the pin was never configured.
        Lets a test assert that a driver claimed the pins it says it owns."""
        return self._modes.get(pin, "")

    def pin_pull(self, pin: int) -> Pull:
        """Which pull the driver asked for. The E-stop button is only safe if
        something pulled it up, so that is worth being able to check."""
        return self._pulls.get(pin, Pull.NONE)

    def glitch_filter_us(self, pin: int) -> int:
        return self._glitch_us.get(pin, 0)

    def servo_pulse_us(self, pin: int | None = None) -> int:
        return self._servo_us.get(self.pins.servo if pin is None else pin, 0)

    @property
    def led_on(self) -> bool:
        led = self.pins.status_led
        return bool(led is not None and self._levels.get(led, False))

    def wheel_rpm(self, side: str) -> float:
        return self.plant.wheel(side).rpm

    def total_counts(self, side: str) -> int:
        return self.plant.wheel(side).total_counts

    @property
    def rail_voltage(self) -> float:
        return self.plant.rail_voltage

    @property
    def outputs_safe(self) -> bool:
        """True when neither bridge can drive. The panic-stop tests assert this."""
        pins = self.pins
        return (
            self._pwm_duty.get(pins.ena, 0.0) <= 0.0
            and self._pwm_duty.get(pins.enb, 0.0) <= 0.0
        )

    @property
    def cleaned_up(self) -> bool:
        return self._cleaned

    def ops(self, kind: str | None = None) -> list[MockOp]:
        if kind is None:
            return self.history.snapshot()
        return [op for op in self.history if op.kind == kind]

    def reset(self) -> None:
        """Back to a freshly-constructed plant, keeping registered callbacks."""
        self.plant.reset()
        self.edges_delivered = 0
        self.callback_errors = 0
        self.dropped_by_glitch = 0
        self._last_edge_us.clear()
        self.history.clear()

    def _record(self, kind: str, pin: int, value: float) -> None:
        self.history.append(MockOp(self.ticks_us(), kind, pin, value))

    def _encoder_channel(self, pin: int) -> tuple[WheelPlant, float] | None:
        pins = self.pins
        if pin == pins.enc_l_a:
            return (self.plant.left, 0.0)
        if pin == pins.enc_l_b:
            return (self.plant.left, -0.5)
        if pin == pins.enc_r_a:
            return (self.plant.right, 0.0)
        if pin == pins.enc_r_b:
            return (self.plant.right, -0.5)
        return None

    def __repr__(self) -> str:
        return (
            f"MockBackend({self.plant!r}, edges={self.edges_delivered}, "
            f"safe={self.outputs_safe})"
        )


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def _clamp01(value: float) -> float:
    if value <= 0.0:
        return 0.0
    return 1.0 if value >= 1.0 else value


def _decode_bridge(enable_duty: float, in_a: bool, in_b: bool) -> tuple[DriveMode, float]:
    """One half of an L298 as the firmware drives it.

    The truth table, which is worth stating in full because getting it backwards
    shorts a spinning motor every time you meant to let it roll::

        EN=0  any    -> COAST   (outputs high-Z)
        EN>0  A!=B   -> DRIVE   (direction from which of A/B is high)
        EN>0  A==B   -> BRAKE   (both windings tied to the same rail)
    """
    if enable_duty <= 0.0:
        return (DriveMode.COAST, 0.0)
    if in_a == in_b:
        return (DriveMode.BRAKE, enable_duty)
    return (DriveMode.FORWARD if in_a else DriveMode.REVERSE, enable_duty)


def _channel_level(phase: float) -> bool:
    """Square wave that toggles once per encoder count."""
    return int(math.floor(phase)) % 2 == 0


def make_mock(clock: Clock | None = None, **kwargs: Any) -> MockBackend:
    """Convenience constructor for fixtures: ``make_mock(FakeClock())``."""
    return MockBackend(clock=clock, **kwargs)


__all__ = [
    "DriveMode",
    "PlantParams",
    "WheelPlant",
    "BoostRegulator",
    "DrivetrainPlant",
    "MockPins",
    "MockOp",
    "MockBackend",
    "make_mock",
    "LEFT",
    "RIGHT",
]
