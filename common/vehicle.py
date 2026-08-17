"""Vehicle behaviour: raw driver intent in, actuator targets out.

Pure Python, no hardware, no I/O, no clock of its own -- ``update()`` is handed the time.
That is deliberate: this is the safety-critical core, and it is the one part of v1 that
absolutely has to be testable on a laptop.

Both the Pi and the simulator run *this same model*. The Pi feeds its output to pigpio;
the simulator draws it on screen. If they ran separate logic the simulator would be
decoration -- agreeing with the real thing only until it mattered.

``pi/vehicle.py`` is a byte-identical copy (the Pi is a separate install).
"""

from __future__ import annotations

from dataclasses import dataclass

from protocol import (
    DIR_BRAKE,
    DIR_COAST,
    DIR_FWD,
    DIR_REV,
    STATE_DRIVE,
    STATE_FAILSAFE,
    STATE_INIT,
    STATE_SAFE,
    WATCHDOG_TIMEOUT_S,
    clamp,
)


@dataclass
class VehicleConfig:
    """Tunables. The Pi overrides these from ``pi/config.py``; the simulator uses defaults."""

    #: Nothing moves below roughly this duty -- the L298N's Darlington stage drops ~1.4 V,
    #: so throttle maps into MIN..MAX rather than up from zero.
    min_duty: float = 0.30
    #: Starting cap. Raise it once the car drives sanely. The Pololu regulator sustains
    #: only ~1.5 A for both motors combined.
    max_duty: float = 0.60

    #: Throttle units per second. 0 -> full in ~0.4 s. A step input is how you brown out.
    slew_per_s: float = 2.5

    #: Below this, coast rather than buzz the motors at minimum duty.
    throttle_deadband: float = 0.02

    #: Pedal travel below this is treated as "foot resting on the pedal", not braking.
    brake_deadband: float = 0.05
    #: Braking works by shorting the motor through the bridge and PWMing that short.
    #: Below roughly this duty the short is too brief to resist anything, so pedal travel
    #: maps into min_brake_duty..1.0 rather than up from zero -- the same reason throttle
    #: maps into min_duty..max_duty.
    min_brake_duty: float = 0.25

    #: Forced coast when reversing direction, instead of slamming a spinning motor
    #: through the bridge the other way.
    dir_change_coast_s: float = 0.10

    servo_centre_us: int = 1500
    #: Pulse travel from centre to full lock, measured **separately per side**. A steering
    #: bellcrank is rarely symmetric: the same wheel angle costs a different number of
    #: microseconds left versus right. One shared span would trade a correct lock on one
    #: side for a short one on the other, and equal *angle* is what a driver feels.
    servo_range_left_us: int = 300
    servo_range_right_us: int = 300
    servo_min_us: int = 1200
    servo_max_us: int = 1800
    servo_trim_us: int = 0  # bench calibration offset

    watchdog_timeout_s: float = WATCHDOG_TIMEOUT_S

    #: How long FAILSAFE must persist before servo pulses stop entirely. The HS-311 hangs
    #: off the Pi's own 5 V rail and a stalled one can brown out the SoC.
    servo_idle_off_s: float = 2.0


@dataclass
class Outputs:
    """What the actuators should be doing right now."""

    state: str = STATE_INIT
    armed: bool = False
    throttle_cmd: float = 0.0
    throttle_out: float = 0.0
    steer_cmd: float = 0.0
    direction: str = DIR_COAST
    #: 0.0-1.0. Meaningful for FWD/REV; BRAKE holds EN high; COAST drops EN.
    duty: float = 0.0
    #: Microseconds, or 0 meaning "stop sending pulses entirely".
    servo_us: int = 0

    @property
    def duty_pct(self) -> float:
        return self.duty * 100.0


class Vehicle:
    """Driver intent -> actuator targets, with the failsafes that keep the hardware alive."""

    def __init__(self, cfg: VehicleConfig | None = None) -> None:
        self.cfg = cfg or VehicleConfig()
        self.out = Outputs()
        self._last_cmd_t: float | None = None
        self._last_update_t: float | None = None
        self._coast_until: float = 0.0
        self._failsafe_since: float | None = None
        #: Direction the bridge is currently energised in, for the reversal guard.
        self._energised_dir: str = DIR_COAST

    # -- main entry point ------------------------------------------------------

    def update(self, cmd: dict | None, now: float) -> Outputs:
        """Advance the model. ``cmd`` is a decoded control message, or ``None`` if
        nothing arrived this tick. Call this every tick regardless -- the watchdog only
        trips because time passes, not because a packet arrives."""
        dt = 0.0 if self._last_update_t is None else max(0.0, now - self._last_update_t)
        self._last_update_t = now

        if cmd is not None:
            self._last_cmd_t = now
            self.out.throttle_cmd = float(cmd.get("throttle", 0.0))
            self.out.steer_cmd = float(cmd.get("steer", 0.0))
            self._arm_req = bool(cmd.get("arm", False))
            self._brake_req = float(cmd.get("brake", 0.0))

        stale = self._last_cmd_t is None or (now - self._last_cmd_t) > self.cfg.watchdog_timeout_s

        if stale:
            self._enter_failsafe(now)
        else:
            self._failsafe_since = None
            if self._arm_req:
                self.out.state = STATE_DRIVE
                self.out.armed = True
            else:
                self.out.state = STATE_SAFE
                self.out.armed = False

        self._update_throttle(now, dt)
        self._update_servo(now)
        return self.out

    # -- internals -------------------------------------------------------------

    def _enter_failsafe(self, now: float) -> None:
        if self._failsafe_since is None:
            self._failsafe_since = now
        self.out.state = STATE_FAILSAFE
        self.out.armed = False
        # Intent is stale, so stop reporting it as if it were live.
        self.out.throttle_cmd = 0.0
        self.out.steer_cmd = 0.0
        self._arm_req = False
        self._brake_req = 0.0

    def _update_throttle(self, now: float, dt: float) -> None:
        cfg = self.cfg
        out = self.out

        # Disarmed or failsafed: cut drive immediately rather than ramping down. Coast is
        # the safe resting state -- the car rolls, nothing is energised.
        if not out.armed:
            out.throttle_out = 0.0
            out.direction = DIR_COAST
            out.duty = 0.0
            self._energised_dir = DIR_COAST
            return

        if self._brake_req > cfg.brake_deadband:
            # On the L298, EN high with IN1 == IN2 shorts the motor through the bridge:
            # that is the brake. Coast is EN *low*. This inverts most people's intuition.
            # PWMing that short is what makes braking proportional to pedal travel.
            out.throttle_out = 0.0
            out.direction = DIR_BRAKE
            out.duty = cfg.min_brake_duty + clamp(self._brake_req, 0.0, 1.0) * (
                1.0 - cfg.min_brake_duty
            )
            self._energised_dir = DIR_COAST
            return

        target = clamp(out.throttle_cmd, -1.0, 1.0)
        step = cfg.slew_per_s * dt
        if target > out.throttle_out:
            out.throttle_out = min(target, out.throttle_out + step)
        else:
            out.throttle_out = max(target, out.throttle_out - step)

        want = DIR_FWD if out.throttle_out > 0 else DIR_REV if out.throttle_out < 0 else DIR_COAST

        # Reversal guard: never flip an energised bridge straight over.
        if want != DIR_COAST and self._energised_dir not in (DIR_COAST, want):
            self._coast_until = now + cfg.dir_change_coast_s
            self._energised_dir = DIR_COAST

        if now < self._coast_until or abs(out.throttle_out) < cfg.throttle_deadband:
            out.direction = DIR_COAST
            out.duty = 0.0
            self._energised_dir = DIR_COAST
            return

        out.direction = want
        out.duty = cfg.min_duty + abs(out.throttle_out) * (cfg.max_duty - cfg.min_duty)
        self._energised_dir = want

    def _update_servo(self, now: float) -> None:
        cfg = self.cfg
        # Steering stays live while merely disarmed -- it is useful for bench calibration
        # and a servo is not what hurts you. Pulses stop only once the link is properly
        # gone, so a stalled servo cannot sit on the 5 V rail unattended.
        if self._failsafe_since is not None and (now - self._failsafe_since) > cfg.servo_idle_off_s:
            self.out.servo_us = 0
            return
        steer = self.out.steer_cmd
        # steer is negative for left, so the left span still subtracts from centre.
        span = cfg.servo_range_right_us if steer >= 0 else cfg.servo_range_left_us
        us = cfg.servo_centre_us + cfg.servo_trim_us + steer * span
        self.out.servo_us = int(clamp(us, cfg.servo_min_us, cfg.servo_max_us))

    # Set by ``update``; declared here so a fresh instance is well-defined.
    _arm_req: bool = False
    _brake_req: float = 0.0
