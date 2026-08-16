"""Steering: one HS-311 on DMA-timed pulses, with the clamps it needs to survive.

Three things shape this driver, and all three are consequences of the wiring
rather than preferences.

**No hardware PWM is available.** GPIO18 is on PWM channel 0, which is the same
channel as GPIO12 (ENA). The moment the motors run, the servo cannot have it.
pigpio's DMA-timed pulse train is the alternative and it is a good one -- a
couple of microseconds of jitter -- but it means pulses are written as integers
and every write costs a socket round trip.

**The servo shares the Pi's 5 V rail.** A servo held against its end stop stalls
and pulls well over an amp out of the same rail feeding the SoC, which browns
out the *computer*. Hence the absolute pulse clamp that nothing can bypass, the
slew limit that caps peak current, and ``relax()`` as the correct disarmed
state: stopping the pulse train drops holding current to near zero.

**Small pulse changes make it buzz.** A servo commanded 1500, 1501, 1500, 1499
hunts audibly and eats current doing it. The ``steer_hold_us`` deadband --
carried over from the previous firmware deliberately -- suppresses rewrites
below a threshold. At the default 8 us out of a ~600 us span that is 1.3 % of
travel, which is finer than the HS-311's own deadband, so nothing is given up.
"""

from __future__ import annotations

from ..config import VehicleConfig
from ..constants import SERVO_ABS_MAX_US, SERVO_ABS_MIN_US
from ..hal.base import GpioBackend
from ..log import get_logger
from ..util.clock import Clock

_log = get_logger(__name__)


class SteeringServo:
    """Angle in, clamped and slew-limited pulses out.

    Commands set a *target*; :meth:`update` moves the modelled shaft position
    toward it at ``steer_rate_us_per_s`` and writes a pulse only when the change
    clears the jitter deadband. That split is what lets the control loop call
    ``set_angle`` at 100 Hz without the servo ever seeing a step it cannot
    physically follow.
    """

    __slots__ = (
        "_gpio", "_pin", "_config", "_clock",
        "_target_us", "_position_us", "_written_us",
        "_relaxed", "_force_write", "_clamped", "writes",
    )

    def __init__(
        self,
        gpio: GpioBackend,
        pin: int,
        config: VehicleConfig,
        clock: Clock,
    ) -> None:
        if config.steer_min_us >= config.steer_max_us:
            raise ValueError(
                f"steer_min_us ({config.steer_min_us}) must be below "
                f"steer_max_us ({config.steer_max_us})"
            )
        if not config.steer_min_us <= config.steer_center_us <= config.steer_max_us:
            raise ValueError(
                f"steer_center_us ({config.steer_center_us}) is outside "
                f"[{config.steer_min_us}, {config.steer_max_us}]"
            )
        if config.steer_max_deg <= 0.0:
            raise ValueError("steer_max_deg must be positive")
        if config.steer_rate_us_per_s <= 0.0:
            raise ValueError("steer_rate_us_per_s must be positive")

        self._gpio = gpio
        self._pin = pin
        self._config = config
        self._clock = clock

        centre = float(self._centre_us())
        self._target_us = centre
        #: Where we believe the shaft is. Not a measurement -- the HS-311 has no
        #: feedback out -- but it is exactly what the pulse train has asked for,
        #: which is the best estimate available and the one odometry should use.
        self._position_us = centre
        self._written_us = 0
        #: Boots limp. No pulses until something commands steering, so a car
        #: sitting at the start line is not holding the wheels against a kerb
        #: and drawing current from the SoC's rail to do it.
        self._relaxed = True
        self._force_write = False
        self._clamped = False
        self.writes = 0

        _log.info(
            "steering servo ready",
            pin=pin,
            centre_us=int(centre),
            min_us=config.steer_min_us,
            max_us=config.steer_max_us,
            lock_deg=config.steer_max_deg,
            rate_us_per_s=config.steer_rate_us_per_s,
        )

    # -- commands -----------------------------------------------------------

    def set_angle(self, radians: float) -> None:
        """Command a road-wheel angle. Positive is to the RIGHT.

        Right-positive matches the wire protocol's steering field and the
        ``steer_min_us``/``steer_max_us`` naming, so the sign never has to be
        flipped between the app, the telemetry HUD and the servo.
        """
        if radians != radians:  # NaN: a corrupt angle must not become a pulse
            return
        limit = self._config.steer_max_rad
        if limit <= 0.0:
            return
        self.set_normalized(radians / limit)

    def set_normalized(self, value: float) -> None:
        """Command -1 (full left) .. +1 (full right)."""
        if value != value:
            return
        if value > 1.0:
            value = 1.0
        elif value < -1.0:
            value = -1.0
        if self._config.steer_invert:
            value = -value
        self._set_target(self._pulse_for_fraction(value))

    def set_pulse_us(self, us: int) -> None:
        """Write a raw pulse width. Calibration only.

        Still hard-clamped to the absolute limits -- those exist to stop a servo
        being driven into its end stop, and no caller gets to opt out of that --
        but deliberately *not* clamped to ``steer_min_us``/``steer_max_us``,
        because finding those two numbers is what this method is for.
        """
        self._set_target(float(_clamp_absolute(us)))

    def center(self) -> None:
        """Target the calibrated centre, trim included. Slew still applies."""
        self._set_target(float(self._centre_us()))

    def relax(self) -> None:
        """Stop the pulse train. The servo goes limp and stops drawing current.

        The correct disarmed state on this car, not merely a power saving: the
        servo is on the Pi's own 5 V rail, so a stalled servo takes the computer
        down with it. The modelled position is left alone so that the next
        command resumes from where the shaft was last told to be.
        """
        self._relaxed = True
        self._force_write = False
        self._written_us = 0
        self._gpio.set_servo_pulse(self._pin, 0)

    # -- per-tick -----------------------------------------------------------

    def update(self, dt: float) -> None:
        """Advance the slew limit and write a pulse if it cleared the deadband.

        Called every control tick. Does nothing while relaxed: a limp servo has
        no position to slew, and restarting the pulse train is an explicit act.
        """
        if self._relaxed:
            return
        if dt > 0.0:
            step = self._config.steer_rate_us_per_s * dt
            error = self._target_us - self._position_us
            if error > step:
                error = step
                self._clamped = True
            elif error < -step:
                error = -step
                self._clamped = True
            else:
                self._clamped = False
            self._position_us += error

        pulse = int(self._position_us + 0.5) if self._position_us >= 0.0 else 0
        delta = pulse - self._written_us
        if delta < 0:
            delta = -delta
        # A hold of zero still means "do not rewrite an identical value": the
        # pulse is an integer, so a sub-microsecond change is not representable
        # and rewriting it is pure socket traffic.
        hold = self._config.steer_hold_us
        if hold < 1:
            hold = 1
        if self._force_write or delta >= hold:
            self._force_write = False
            self._written_us = pulse
            self.writes += 1
            self._gpio.set_servo_pulse(self._pin, pulse)

    # -- state --------------------------------------------------------------

    @property
    def applied_us(self) -> int:
        """The pulse width actually on the wire. Zero means the train is
        stopped, which telemetry renders as a relaxed servo rather than as an
        impossible 0 us pulse."""
        return self._written_us

    @property
    def applied_angle(self) -> float:
        """Modelled road-wheel angle in radians, after slew and clamping.

        Derived from the modelled position rather than from the last written
        pulse so the value stays continuous through the jitter deadband, and
        stays meaningful while relaxed. Odometry consumes this.
        """
        return self._angle_for_pulse(self._position_us)

    @property
    def target_us(self) -> int:
        return int(self._target_us + 0.5)

    @property
    def relaxed(self) -> bool:
        return self._relaxed

    @property
    def slewing(self) -> bool:
        """True when the slew limit is holding the servo back -- i.e. the
        command is moving faster than the servo is allowed to."""
        return self._clamped

    @property
    def at_target(self) -> bool:
        difference = self._target_us - self._position_us
        return -0.5 < difference < 0.5

    def sync_position(self) -> None:
        """Declare the modelled position to be the target, skipping the slew.

        For calibration and bench work only, where a settling delay is handled
        by the procedure itself. Never call this from the control loop: the
        slew limit is what keeps the servo's peak current off the SoC's rail.
        """
        self._position_us = self._target_us
        self._force_write = True

    # -- internals ----------------------------------------------------------

    def _set_target(self, pulse_us: float) -> None:
        self._target_us = pulse_us
        if self._relaxed:
            # Coming out of limp: the shaft may have been pushed anywhere, so
            # force the first write even if the modelled position has not moved.
            self._relaxed = False
            self._force_write = True

    def _centre_us(self) -> int:
        """Calibrated centre plus trim, clamped into the mechanical range.

        Trim is the single biggest lever on odometry heading drift, which is why
        it is a separate parameter from the centre: the centre is measured once
        with the linkage off, the trim is adjusted from the driver's seat.
        """
        centre = int(self._config.steer_center_us) + int(self._config.steer_trim_us)
        low = _clamp_absolute(self._config.steer_min_us)
        high = _clamp_absolute(self._config.steer_max_us)
        if centre < low:
            return low
        if centre > high:
            return high
        return centre

    def _pulse_for_fraction(self, fraction: float) -> float:
        """-1..+1 to microseconds, honouring an asymmetric linkage.

        The two half-ranges are almost never equal on a real car -- the servo
        horn and the track rod see to that -- so each side is scaled against its
        own limit instead of against a shared half-span.
        """
        centre = self._centre_us()
        low = _clamp_absolute(self._config.steer_min_us)
        high = _clamp_absolute(self._config.steer_max_us)
        if fraction >= 0.0:
            pulse = centre + fraction * (high - centre)
        else:
            pulse = centre + fraction * (centre - low)
        if pulse < low:
            pulse = float(low)
        elif pulse > high:
            pulse = float(high)
        return pulse

    def _angle_for_pulse(self, pulse_us: float) -> float:
        centre = self._centre_us()
        high = _clamp_absolute(self._config.steer_max_us)
        low = _clamp_absolute(self._config.steer_min_us)
        if pulse_us >= centre:
            span = high - centre
            fraction = (pulse_us - centre) / span if span > 0 else 0.0
        else:
            span = centre - low
            fraction = (pulse_us - centre) / span if span > 0 else 0.0
        if self._config.steer_invert:
            fraction = -fraction
        if fraction > 1.0:
            fraction = 1.0
        elif fraction < -1.0:
            fraction = -1.0
        return fraction * self._config.steer_max_rad

    def __repr__(self) -> str:
        return (
            f"SteeringServo(pin={self._pin}, applied={self._written_us}us, "
            f"target={self.target_us}us, relaxed={self._relaxed})"
        )


def _clamp_absolute(us: int | float) -> int:
    """The clamp nothing bypasses.

    Wider than any sane calibration, because its job is not to shape the
    steering but to stop a corrupt parameter or a stray calibration command
    driving the HS-311 into a mechanical stop, where it stalls against the Pi's
    own 5 V rail.
    """
    value = int(us)
    if value < SERVO_ABS_MIN_US:
        return SERVO_ABS_MIN_US
    if value > SERVO_ABS_MAX_US:
        return SERVO_ABS_MAX_US
    return value
