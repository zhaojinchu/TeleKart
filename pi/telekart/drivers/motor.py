"""The L298N dual H-bridge, driven as one device because it physically is one.

Both enables sit on the SoC's two hardware PWM channels, which share a clock
divider, so the duties can only be written as a pair. That constraint is what
makes ``MotorPair`` a single object rather than two ``Motor`` instances: two
independent motor objects would each want to write their own channel, and the
first one to pick a different frequency would silently corrupt the other.

The truth table this driver is built around, stated once and in full because it
is the opposite of what most people assume::

    EN = 0,  IN1/IN2 anything   -> COAST   (both halves off, outputs high-Z)
    EN > 0,  IN1 != IN2         -> DRIVE   (direction from which input is high)
    EN > 0,  IN1 == IN2         -> BRAKE   (winding shorted to one rail)

"Both inputs low" is a brake, not a coast. Getting that backwards shorts a
spinning motor every single time you meant to let it roll, and the symptom --
a car that stops far more abruptly than it should and a bridge that runs hot --
looks nothing like a software bug.
"""

from __future__ import annotations

from ..config import MotorPins, VehicleConfig
from ..hal.base import GpioBackend
from ..log import get_logger
from ..util.clock import Clock

_log = get_logger(__name__)

#: Fastest the *applied* duty may rise, in duty per second. This is protection,
#: not feel: it exists so that a single corrupt command, a decode glitch, or a
#: freshly-armed integrator cannot step the bridge from rest to full rail in one
#: tick and trip the boost converter. Full scale in 50 ms is far quicker than
#: the RPM rate limiter above it ever asks for, so in normal driving this clamp
#: never engages -- and when it does, ``limiter_active`` says so.
MAX_DUTY_SLEW_PER_S = 20.0

#: Duty magnitudes below this are treated as zero. Well under the ~0.22 the
#: Darlington bridge needs before it conducts at all, so nothing is lost.
_DUTY_EPS = 1e-4


class _Bridge:
    """One half of the L298: two direction pins, a latched sign, an applied duty.

    Per-side state lives here rather than in parallel arrays on ``MotorPair`` so
    that the left motor's dead-time timer cannot be applied to the right motor's
    direction change. The enable duty is *computed* here and *written* by the
    owner, because the two enables must reach the SoC as one paired operation.
    """

    __slots__ = (
        "_gpio", "_pin_a", "_pin_b", "_invert",
        "sign", "duty", "limited", "_level_a", "_level_b",
        "_ready_at", "changing",
    )

    def __init__(self, gpio: GpioBackend, pin_a: int, pin_b: int, invert: bool) -> None:
        self._gpio = gpio
        self._pin_a = pin_a
        self._pin_b = pin_b
        self._invert = invert
        #: -1, 0 or +1: the direction the IN pins are currently latched into.
        #: Zero means "both low", which is the brake pattern -- harmless while
        #: the enable is zero, and the state the external pull-downs give us.
        self.sign = 0
        #: Signed duty as last written to the enable channel.
        self.duty = 0.0
        self.limited = False
        self._level_a = False
        self._level_b = False
        self._ready_at = 0.0
        self.changing = False

    # -- setup --------------------------------------------------------------

    def configure(self) -> None:
        """Claim the pins and drive them to the safe pattern.

        The initial level is part of ``setup_output`` deliberately: IN1 (GPIO5)
        and IN2 (GPIO6) are pulled *up* by the SoC at boot, and a separate
        "become an output, then write low" would leave a window where the pins
        are driven high. External 10k pull-downs are the real fix; this is the
        software half of it.
        """
        self._gpio.setup_output(self._pin_a, False)
        self._gpio.setup_output(self._pin_b, False)
        self._level_a = False
        self._level_b = False
        self.sign = 0
        self.duty = 0.0
        self.changing = False

    def set_invert(self, invert: bool) -> None:
        """Re-read the inversion flag. Only meaningful while disarmed -- the
        parameter is marked ``requires_disarm`` for exactly this reason."""
        if invert == self._invert:
            return
        self._invert = invert
        # The latch now means the opposite of what it did, so drop it and let
        # the next command re-sequence through the dead time.
        self._apply_sign(0)

    # -- sequencing ---------------------------------------------------------

    def plan(
        self,
        requested: float,
        rpm: float,
        now: float,
        deadtime_s: float,
        reverse_rpm: float,
        max_step: float,
    ) -> float:
        """Decide this tick's signed duty and flip the IN pins if it is safe to.

        Returns the enable magnitude to write. Never raises: every input has
        already been sanitised by the caller, and anything unexpected resolves
        to a smaller duty rather than to an exception on a moving vehicle.
        """
        self.limited = False

        if requested > _DUTY_EPS:
            want = 1
        elif requested < -_DUTY_EPS:
            want = -1
        else:
            want = 0
            requested = 0.0

        if want != 0 and want != self.sign:
            if not self.changing:
                self.changing = True
                self._ready_at = now + deadtime_s
            # Three conditions, all of which must hold before the inputs move:
            #
            #   the dead time has elapsed        -- both halves fully off, so a
            #                                       shoot-through path cannot exist
            #   the enable is already at zero    -- covers direction_deadtime_ms=0,
            #                                       where the timer alone would let
            #                                       the pins flip under live PWM
            #   the wheel is not spinning against the new direction -- otherwise
            #                                       this is a plugging brake, which
            #                                       draws locked-rotor current from
            #                                       a converter that sustains 1.5 A
            if (
                now >= self._ready_at
                and self.duty == 0.0
                and rpm * want >= -reverse_rpm
            ):
                self.changing = False
                self._apply_sign(want)
            else:
                self.limited = True
                requested = 0.0
        elif self.changing:
            # The command came back to the direction already latched; abandon
            # the changeover instead of making the driver wait out a dead time
            # for a reversal that is no longer being asked for.
            self.changing = False

        # Rises are rate limited, falls are not: cutting the enable is always
        # safe and there is never a reason to make a stop take longer.
        target = requested
        if (target >= 0.0) == (self.duty >= 0.0):
            current = self.duty if self.duty >= 0.0 else -self.duty
        else:
            current = 0.0
        magnitude = target if target >= 0.0 else -target
        if magnitude > current + max_step:
            magnitude = current + max_step
            self.limited = True
        self.duty = magnitude if target >= 0.0 else -magnitude
        return magnitude

    def set_brake_pattern(self) -> None:
        """Both inputs low. With the enable up this is a brake; with the enable
        down it is the safe idle pattern. Same pins, opposite meanings."""
        self._apply_sign(0)
        self.duty = 0.0
        self.changing = False

    # -- pin writes ---------------------------------------------------------

    def _apply_sign(self, sign: int) -> None:
        if sign == 0:
            self._write(False, False)
        else:
            forward = sign > 0
            if self._invert:
                forward = not forward
            if forward:
                self._write(True, False)
            else:
                self._write(False, True)
        self.sign = sign

    def _write(self, level_a: bool, level_b: bool) -> None:
        # Deduplicated because a pigpio write is a socket round trip; four of
        # them per tick for pins that have not moved is 400 pointless round
        # trips a second on a core that has other work.
        if level_a != self._level_a:
            self._level_a = level_a
            self._gpio.write(self._pin_a, level_a)
        if level_b != self._level_b:
            self._level_b = level_b
            self._gpio.write(self._pin_b, level_b)

    def panic(self) -> None:
        """Unconditional, undeduplicated write of the safe pattern.

        Bypasses the level cache on purpose: panic paths run when the process
        state is already suspect, and a cache that says "already low" is exactly
        the thing that would be wrong.
        """
        try:
            self._gpio.write(self._pin_a, False)
            self._gpio.write(self._pin_b, False)
        except Exception:  # noqa: BLE001 - a panic stop may not fail
            pass
        self._level_a = False
        self._level_b = False
        self.sign = 0
        self.duty = 0.0
        self.changing = False


class MotorPair:
    """Both drive motors, because the PWM channels are physically coupled.

    Everything that protects the hardware lives here rather than in the control
    loop above it: the per-motor duty ceiling, the combined-duty budget the
    boost converter needs, the rise-rate clamp, and the direction dead-time
    sequencing. The controller is free to ask for anything; this class is what
    guarantees the bridge only ever sees something it can survive.
    """

    __slots__ = (
        "_gpio", "_clock", "_config", "_ena", "_enb", "_left", "_right",
        "_pwm_hz", "_rpm_l", "_rpm_r", "_panicked", "_braking",
        "_brake_strength", "_last_drive_at", "limiter_active",
    )

    def __init__(
        self,
        gpio: GpioBackend,
        pins: MotorPins,
        config: VehicleConfig,
        clock: Clock,
    ) -> None:
        if not 0.0 < config.max_duty <= 1.0:
            raise ValueError(f"max_duty must be in (0, 1], got {config.max_duty}")
        if config.duty_sum_max < config.max_duty:
            raise ValueError(
                f"duty_sum_max ({config.duty_sum_max}) is below max_duty "
                f"({config.max_duty}); one motor alone would exceed the budget"
            )
        if config.pwm_hz <= 0:
            raise ValueError(f"pwm_hz must be positive, got {config.pwm_hz}")

        self._gpio = gpio
        self._clock = clock
        self._config = config
        self._ena = pins.ena
        self._enb = pins.enb
        self._left = _Bridge(gpio, pins.in1, pins.in2, bool(config.invert_left))
        self._right = _Bridge(gpio, pins.in3, pins.in4, bool(config.invert_right))
        self._pwm_hz = int(config.pwm_hz)
        self._rpm_l = 0.0
        self._rpm_r = 0.0
        self._panicked = False
        self._braking = False
        self._brake_strength = 0.0
        #: Negative until the first command, so the first tick uses the nominal
        #: period instead of a meaningless interval measured from construction.
        self._last_drive_at = -1.0
        #: True when a protection clamp modified the command this tick. The
        #: telemetry LIMITER_ACTIVE bit is fed from here; if it lights up during
        #: ordinary driving, the desktop app's rate limits are looser than the
        #: firmware's and the HUD is disagreeing with the car.
        self.limiter_active = False

        # Direction pins before enables, always. The enables come up at zero
        # duty, which is a coast, so there is no instant at which the bridge is
        # enabled into an unknown direction pattern.
        self._left.configure()
        self._right.configure()
        self._gpio.set_pwm_pair(self._ena, self._enb, self._pwm_hz, 0.0, 0.0)

        _log.info(
            "motor pair ready",
            ena=self._ena,
            enb=self._enb,
            pwm_hz=self._pwm_hz,
            max_duty=config.max_duty,
            duty_sum_max=config.duty_sum_max,
            invert_left=bool(config.invert_left),
            invert_right=bool(config.invert_right),
        )

    # -- feedback -----------------------------------------------------------

    def note_speed(self, rpm_l: float, rpm_r: float) -> None:
        """Tell the driver how fast the wheels are actually turning.

        The dead-time sequencer needs measured speed to enforce
        ``reverse_allowed_rpm``, and the driver has no encoders of its own --
        it is deliberately the dumbest thing that can still protect the bridge.
        The control loop calls this once per tick, before :meth:`drive`.
        """
        self._rpm_l = rpm_l if rpm_l == rpm_l else 0.0
        self._rpm_r = rpm_r if rpm_r == rpm_r else 0.0

    # -- commands -----------------------------------------------------------

    def drive(self, duty_l: float, duty_r: float) -> None:
        """Command both motors. Duties are -1..+1; sign is direction.

        Applies, in order: NaN rejection, the combined-duty budget, the
        per-motor ceiling, direction dead-time sequencing, and the rise-rate
        clamp. The result is written as one paired PWM operation.

        Duties stay in the *vehicle* frame for the whole of this method, and for
        everything ``MotorPair`` exposes. Motor inversion is a fact about how the
        leads were soldered, so it is applied at the last possible moment -- in
        ``_Bridge._apply_sign``, as the direction pins are written. Inverting any
        earlier would put the sign of ``duty`` and the sign of the measured
        ``rpm`` into different frames, and the dead-time sequencer compares
        exactly those two to decide whether a reversal would be a plugging brake.
        """
        if self._panicked:
            # Latched off until clear_panic(). Keep writing zeros rather than
            # returning early: pigpiod retains whatever it was last told, so
            # "do nothing" is not the same as "stay stopped".
            self._write_enables(0.0, 0.0)
            return

        self.limiter_active = False
        self._braking = False
        self._brake_strength = 0.0
        config = self._config
        self._pwm_hz = int(config.pwm_hz)

        # NaN propagates through every comparison as False, so a NaN duty would
        # sail past a naive clamp and reach the bridge. Catch it explicitly.
        if duty_l != duty_l:
            duty_l = 0.0
            self.limiter_active = True
        if duty_r != duty_r:
            duty_r = 0.0
            self.limiter_active = True

        # Combined budget before the per-motor ceiling. The converter trips on
        # simultaneous demand, not on either motor alone, and scaling both by
        # the same factor is what preserves the electronic differential's ratio
        # instead of squaring up the wheels mid-corner.
        magnitude_l = duty_l if duty_l >= 0.0 else -duty_l
        magnitude_r = duty_r if duty_r >= 0.0 else -duty_r
        total = magnitude_l + magnitude_r
        budget = config.duty_sum_max
        if total > budget and total > 0.0:
            scale = budget / total
            duty_l *= scale
            duty_r *= scale
            self.limiter_active = True

        ceiling = config.max_duty
        if duty_l > ceiling:
            duty_l = ceiling
            self.limiter_active = True
        elif duty_l < -ceiling:
            duty_l = -ceiling
            self.limiter_active = True
        if duty_r > ceiling:
            duty_r = ceiling
            self.limiter_active = True
        elif duty_r < -ceiling:
            duty_r = -ceiling
            self.limiter_active = True

        now = self._clock.monotonic()
        deadtime = config.direction_deadtime_ms / 1000.0
        reverse_rpm = config.reverse_allowed_rpm
        # One tick's worth of allowed rise. Derived from the loop period rather
        # than assumed: a slow tick may raise duty further, which is correct,
        # because the limit is duty per second and not duty per iteration.
        max_step = MAX_DUTY_SLEW_PER_S * self._period_since_last(now)

        enable_l = self._left.plan(duty_l, self._rpm_l, now, deadtime, reverse_rpm, max_step)
        enable_r = self._right.plan(duty_r, self._rpm_r, now, deadtime, reverse_rpm, max_step)
        if self._left.limited or self._right.limited:
            self.limiter_active = True

        self._write_enables(enable_l, enable_r)

    def brake(self, strength: float) -> None:
        """Short both windings through the bridge at ``strength`` duty.

        IN pair EQUAL and the enable up. Both inputs are driven LOW rather than
        HIGH: low-side braking dissipates in the lower transistors, matches the
        pattern the external pull-downs hold at power-up, and needs no pin
        change at all when coming from an idle state.

        The braking current comes out of the motor's own kinetic energy and not
        out of the boost converter, so the combined-duty budget deliberately
        does not apply here.
        """
        if self._panicked:
            self._write_enables(0.0, 0.0)
            return
        self.limiter_active = False
        if strength != strength or strength <= 0.0:
            self.coast()
            return
        if strength > 1.0:
            strength = 1.0
            self.limiter_active = True

        self._left.set_brake_pattern()
        self._right.set_brake_pattern()
        self._braking = True
        self._brake_strength = strength
        self._write_enables(strength, strength)

    def coast(self) -> None:
        """Enables to zero, inputs to the safe pattern. Outputs go high-Z.

        This is the *only* way to coast on an L298. Leaving the enable up and
        squaring the inputs is a brake, however much it looks like "no command".
        """
        self._left.set_brake_pattern()
        self._right.set_brake_pattern()
        self._braking = False
        self._brake_strength = 0.0
        self.limiter_active = False
        self._write_enables(0.0, 0.0)

    def panic_stop(self) -> None:
        """Idempotent, allocation-free, safe from a signal handler.

        No formatting, no logging, no container construction, and every write
        wrapped: this runs from ``atexit``, from ``SIGTERM``, and from
        ``sys.excepthook``, which means it runs at moments when something has
        already gone wrong. Enables first so the bridge is disabled before the
        direction pins move, and the level cache is bypassed entirely because
        after a crash the cache is exactly the thing not to trust.

        Latches: every later :meth:`drive` writes zeros until
        :meth:`clear_panic`. pigpiod outlives this process and holds whatever it
        was last told, so a panic stop that could be undone by one stale command
        would not be a panic stop.
        """
        self._panicked = True
        self._braking = False
        self._brake_strength = 0.0
        try:
            self._gpio.set_pwm_pair(self._ena, self._enb, self._pwm_hz, 0.0, 0.0)
        except Exception:  # noqa: BLE001 - a panic stop may not fail
            pass
        self._left.panic()
        self._right.panic()

    def clear_panic(self) -> None:
        """Release the panic latch. The caller must have re-established that
        driving is permitted; the driver itself has no way to know."""
        self._panicked = False

    # -- state --------------------------------------------------------------

    @property
    def panicked(self) -> bool:
        return self._panicked

    @property
    def braking(self) -> bool:
        return self._braking

    @property
    def brake_strength(self) -> float:
        return self._brake_strength

    @property
    def applied_duty_l(self) -> float:
        """Signed duty actually applied to the left motor, in the vehicle frame.

        It is not the *requested* duty -- dead-time sequencing and the rise clamp
        can both make it differ, and telemetry that echoed the request would hide
        precisely the cases worth seeing.

        Positive always means forward for the vehicle regardless of how the leads
        are soldered, because inversion is applied below this, at the pin write.
        Downstream this feeds the encoder direction hint, the stall/brownout
        comparison against measured RPM, and the telemetry packet -- all of which
        are vehicle-frame, so no un-inversion belongs here.
        """
        return self._left.duty

    @property
    def applied_duty_r(self) -> float:
        return self._right.duty

    @property
    def direction_l(self) -> int:
        """-1, 0 or +1: the direction the left IN pins are latched into."""
        return self._left.sign

    @property
    def direction_r(self) -> int:
        return self._right.sign

    @property
    def changeover_active(self) -> bool:
        """True while either side is waiting out a direction change."""
        return self._left.changing or self._right.changing

    def refresh_config(self) -> None:
        """Pick up ``invert_left`` / ``invert_right`` after a parameter push.

        Both are ``requires_disarm``, so this only ever runs on a stationary
        car; it drops the direction latch so the next command re-sequences.
        """
        self._left.set_invert(bool(self._config.invert_left))
        self._right.set_invert(bool(self._config.invert_right))

    # -- internals ----------------------------------------------------------

    def _write_enables(self, duty_a: float, duty_b: float) -> None:
        # One paired write, always. The backend deduplicates identical values,
        # so an unchanged command costs nothing on the wire.
        self._gpio.set_pwm_pair(self._ena, self._enb, self._pwm_hz, duty_a, duty_b)

    def _period_since_last(self, now: float) -> float:
        """Seconds since the previous command, for the duty-per-second clamp.

        Measured here rather than taken as an argument so that ``drive()`` keeps
        the signature the interface contract fixes, and so it stays correct when
        the caller is a calibration sweep pacing itself at some other rate. The
        first call, and any gap longer than a quarter second, counts as one
        nominal control period: a long pause must not authorise an instant step
        to full duty.
        """
        previous = self._last_drive_at
        self._last_drive_at = now
        if previous < 0.0:
            return 0.010
        delta = now - previous
        if delta <= 0.0 or delta > 0.25:
            return 0.010
        return delta

    def __repr__(self) -> str:
        return (
            f"MotorPair(l={self._left.duty:+.3f}, r={self._right.duty:+.3f}, "
            f"braking={self._braking}, limited={self.limiter_active}, "
            f"panicked={self._panicked})"
        )
