"""The input chain: raw device numbers in, wire-ready commands out.

Zero I/O. No clock, no sockets, no Qt, no SDL. Everything it needs arrives as
arguments, which is what lets the whole thing be tested exhaustively and driven
from a script at ten thousand times real time.

The order of operations is fixed by docs/INTERFACES.md §9 and is not a matter of
taste:

    calibrate -> deadzone -> saturation -> curve -> speed-sensitive steering
              -> rate limit -> smoothing -> quantize

Two of those placements are worth defending, because getting them backwards
produces a control that feels wrong in a way that is very hard to diagnose:

* **Saturation before the curve.** Saturation is what makes 100 % reachable on a
  pedal whose travel stops a little short. Applying it after the curve would
  clip the curved output instead of the input, which changes the shape of the
  response as well as its endpoint.
* **Rate limit before smoothing.** The rate limit is a hard bound on how fast the
  command may move, and it must remain a hard bound. A filter after it can only
  round the corners; a filter before it would be re-shaped by the limiter into
  something neither element specified.

Division of responsibility with the firmware: this file owns *feel* -- curves,
deadzone, smoothing. The firmware owns *protection* -- duty slew clamps, servo
rate limits, stall detection, the failsafe ramp. Never filter in both places;
an EMA here plus an EMA there is second-order lag nobody can reason about. And
the rate limits configured here must stay tighter than the firmware's, so that
`TelemetryFlags.LIMITER_ACTIVE` stays dark during ordinary driving.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Protocol

from telekart_protocol.constants import BRAKE_SCALE, STEERING_SCALE, THROTTLE_SCALE

from .curves import (
    DEFAULT_BRAKE_CURVE,
    DEFAULT_STEER_CURVE,
    DEFAULT_THROTTLE_CURVE,
    LINEAR,
    Curve,
    CurveSpec,
)
from .mapping import Control

#: Below this the two ends of a calibrated range are indistinguishable from
#: noise, and the normalization would amplify jitter into full-scale garbage.
MIN_RANGE_SPAN = 0.05

#: Deadzone plus saturation cannot eat the whole travel; leave a usable band.
MAX_DEADZONE = 0.60
MAX_SATURATION = 0.60
_MIN_USABLE_SPAN = 0.10

#: dt larger than this means the input thread was descheduled (a laptop lid, a
#: garbage-collection pause, a debugger breakpoint). Rate limits and filters get
#: this ceiling instead of the true value, so that resuming after a two-second
#: stall does not let the command jump the entire travel in one tick.
MAX_DT = 0.10


class ChainConfigError(ValueError):
    """A chain configuration is unusable. Raised at construction, never per tick."""


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class AxisCalibration:
    """Raw device units to normalized units.

    Bipolar (steering) uses `lo`, `rest`, `hi` and produces -1..+1, scaling each
    side of centre independently -- wheels are not symmetric about their centre
    and forcing one scale factor onto both halves puts the electrical centre off
    the mechanical one.

    Unipolar (pedals) uses `rest` and `hi` and produces 0..1. `hi` may be below
    `rest`; that is the normal case for a Logitech pedal, which idles at +1.0 and
    reads -1.0 fully depressed. `invert` is deliberately *not* how that gets
    handled -- rest/hi are the measurement, `invert` is the driver's explicit
    "this is backwards" switch, and keeping the two separate means the wizard can
    never end up double-inverting.
    """

    bipolar: bool
    rest: float = 0.0
    lo: float = -1.0
    hi: float = 1.0
    invert: bool = False
    #: Applied immediately after normalization, before clamping. Rotation lock
    #: rides here: using 240 degrees of a 900-degree wheel is a pre-gain of 3.75.
    pre_gain: float = 1.0

    def __post_init__(self) -> None:
        for name in ("rest", "lo", "hi", "pre_gain"):
            value = getattr(self, name)
            if not math.isfinite(value):
                raise ChainConfigError(f"calibration {name} must be finite")
        if not (0.05 <= self.pre_gain <= 40.0):
            raise ChainConfigError(f"pre_gain {self.pre_gain} outside [0.05, 40]")
        if self.bipolar:
            if not (self.lo < self.rest < self.hi):
                raise ChainConfigError(
                    "bipolar calibration needs lo < rest < hi, got "
                    f"{self.lo} / {self.rest} / {self.hi}"
                )
            if (self.rest - self.lo) < MIN_RANGE_SPAN or (
                self.hi - self.rest
            ) < MIN_RANGE_SPAN:
                raise ChainConfigError(
                    f"bipolar calibration half-span below {MIN_RANGE_SPAN}: "
                    f"{self.lo} / {self.rest} / {self.hi}"
                )
        elif abs(self.hi - self.rest) < MIN_RANGE_SPAN:
            raise ChainConfigError(
                f"unipolar calibration span below {MIN_RANGE_SPAN}: "
                f"rest={self.rest} full={self.hi}"
            )

    @classmethod
    def steering(
        cls, lo: float, rest: float, hi: float, *, invert: bool = False,
        pre_gain: float = 1.0,
    ) -> "AxisCalibration":
        return cls(True, rest=rest, lo=lo, hi=hi, invert=invert, pre_gain=pre_gain)

    @classmethod
    def pedal(
        cls, rest: float, full: float, *, invert: bool = False, pre_gain: float = 1.0
    ) -> "AxisCalibration":
        return cls(False, rest=rest, lo=min(rest, full), hi=full, invert=invert,
                   pre_gain=pre_gain)

    @classmethod
    def identity(cls, control: Control) -> "AxisCalibration":
        """Pass-through. Used by scripted and replayed sources, which already
        speak normalized units."""
        if control is Control.STEER:
            return cls.steering(-1.0, 0.0, 1.0)
        return cls.pedal(0.0, 1.0)

    def to_dict(self) -> dict[str, object]:
        return {
            "bipolar": self.bipolar,
            "rest": self.rest,
            "lo": self.lo,
            "hi": self.hi,
            "invert": self.invert,
            "pre_gain": self.pre_gain,
        }

    @classmethod
    def from_dict(cls, data: object) -> "AxisCalibration":
        if not isinstance(data, dict):
            raise ChainConfigError(
                f"calibration must be an object, got {type(data).__name__}"
            )
        return cls(
            bipolar=bool(data.get("bipolar", False)),
            rest=_as_float(data.get("rest", 0.0), "rest"),
            lo=_as_float(data.get("lo", -1.0), "lo"),
            hi=_as_float(data.get("hi", 1.0), "hi"),
            invert=bool(data.get("invert", False)),
            pre_gain=_as_float(data.get("pre_gain", 1.0), "pre_gain"),
        )


@dataclass(frozen=True, slots=True)
class OneEuroConfig:
    """One-euro filter settings.

    Chosen over a fixed low-pass because the requirement is contradictory: hold
    a steady steering angle without visible jitter, *and* do not lag a flick of
    the wheel. One-euro resolves it by raising its cutoff with the observed
    speed of the signal -- slow movement gets smoothed hard, fast movement passes
    through nearly untouched.
    """

    enabled: bool = True
    #: Cutoff at zero velocity, Hz. Lower is smoother and laggier.
    min_cutoff: float = 4.0
    #: How aggressively the cutoff rises with signal speed, Hz per unit/s.
    beta: float = 0.35
    #: Cutoff of the derivative estimate, Hz. Rarely worth changing.
    d_cutoff: float = 1.0

    def __post_init__(self) -> None:
        if not (math.isfinite(self.min_cutoff) and self.min_cutoff > 0.0):
            raise ChainConfigError("one-euro min_cutoff must be positive and finite")
        if not (math.isfinite(self.beta) and self.beta >= 0.0):
            raise ChainConfigError("one-euro beta must be non-negative and finite")
        if not (math.isfinite(self.d_cutoff) and self.d_cutoff > 0.0):
            raise ChainConfigError("one-euro d_cutoff must be positive and finite")

    def to_dict(self) -> dict[str, object]:
        return {
            "enabled": self.enabled,
            "min_cutoff": self.min_cutoff,
            "beta": self.beta,
            "d_cutoff": self.d_cutoff,
        }

    @classmethod
    def from_dict(cls, data: object) -> "OneEuroConfig":
        if not isinstance(data, dict):
            raise ChainConfigError(
                f"smoothing must be an object, got {type(data).__name__}"
            )
        return cls(
            enabled=bool(data.get("enabled", True)),
            min_cutoff=_as_float(data.get("min_cutoff", 4.0), "min_cutoff"),
            beta=_as_float(data.get("beta", 0.35), "beta"),
            d_cutoff=_as_float(data.get("d_cutoff", 1.0), "d_cutoff"),
        )


NO_SMOOTHING = OneEuroConfig(enabled=False)


@dataclass(frozen=True, slots=True)
class AxisChainConfig:
    """Everything the chain does to one control."""

    calibration: AxisCalibration
    deadzone: float = 0.0
    saturation: float = 0.0
    curve: Curve = LINEAR
    #: Units per second moving away from rest, and back toward it. They differ
    #: on purpose: a throttle that ramps in over 300 ms but releases in 60 ms is
    #: both smooth and honest about lifting off.
    rate_rise: float = 8.0
    rate_fall: float = 16.0
    smoothing: OneEuroConfig = NO_SMOOTHING

    def __post_init__(self) -> None:
        if not isinstance(self.calibration, AxisCalibration):
            raise ChainConfigError("calibration must be an AxisCalibration")
        if not isinstance(self.curve, Curve):
            raise ChainConfigError("curve must be a Curve")
        if not (math.isfinite(self.deadzone) and 0.0 <= self.deadzone <= MAX_DEADZONE):
            raise ChainConfigError(f"deadzone {self.deadzone} outside [0, {MAX_DEADZONE}]")
        if not (
            math.isfinite(self.saturation) and 0.0 <= self.saturation <= MAX_SATURATION
        ):
            raise ChainConfigError(
                f"saturation {self.saturation} outside [0, {MAX_SATURATION}]"
            )
        if 1.0 - self.deadzone - self.saturation < _MIN_USABLE_SPAN:
            raise ChainConfigError(
                f"deadzone {self.deadzone} + saturation {self.saturation} leaves "
                f"less than {_MIN_USABLE_SPAN} of usable travel"
            )
        for name in ("rate_rise", "rate_fall"):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ChainConfigError(f"{name} must be positive and finite")

    def to_dict(self) -> dict[str, object]:
        return {
            "calibration": self.calibration.to_dict(),
            "deadzone": self.deadzone,
            "saturation": self.saturation,
            "curve": self.curve.spec.to_dict(),
            "rate_rise": self.rate_rise,
            "rate_fall": self.rate_fall,
            "smoothing": self.smoothing.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: object) -> "AxisChainConfig":
        if not isinstance(data, dict):
            raise ChainConfigError(
                f"axis config must be an object, got {type(data).__name__}"
            )
        return cls(
            calibration=AxisCalibration.from_dict(data.get("calibration", {})),
            deadzone=_as_float(data.get("deadzone", 0.0), "deadzone"),
            saturation=_as_float(data.get("saturation", 0.0), "saturation"),
            curve=Curve(CurveSpec.from_dict(data.get("curve", {"kind": "linear"}))),
            rate_rise=_as_float(data.get("rate_rise", 8.0), "rate_rise"),
            rate_fall=_as_float(data.get("rate_fall", 16.0), "rate_fall"),
            smoothing=OneEuroConfig.from_dict(
                data.get("smoothing", {"enabled": False})
            ),
        )


@dataclass(frozen=True, slots=True)
class ChainConfig:
    steer: AxisChainConfig
    throttle: AxisChainConfig
    brake: AxisChainConfig
    #: Fraction of steering range removed at measured top speed. Mirrors the
    #: firmware's `steer_speed_reduction`; set one of the two to zero, never both
    #: to a non-zero value, or the reduction applies twice.
    speed_sensitive_steering: float = 0.0
    #: With digital pedals the driver can hold both buttons at once. Deciding
    #: here that brake wins means the HUD shows what the car will actually do.
    brake_cuts_throttle: bool = True
    #: Brake command asserted when the input device disappears mid-drive. Zero by
    #: default, and that default is the considered one: the tested response to a
    #: dead input is for the TX thread to *stop sending*, which hands the car to
    #: the firmware's coast-brake-disarm schedule. Inventing a second, untested
    #: stopping path in the app is worse than reusing the one on the car.
    disconnect_brake: float = 0.0

    def __post_init__(self) -> None:
        for name in ("steer", "throttle", "brake"):
            if not isinstance(getattr(self, name), AxisChainConfig):
                raise ChainConfigError(f"{name} must be an AxisChainConfig")
        if not self.steer.calibration.bipolar:
            raise ChainConfigError("steering calibration must be bipolar")
        if self.throttle.calibration.bipolar or self.brake.calibration.bipolar:
            raise ChainConfigError("pedal calibrations must be unipolar")
        if not (
            math.isfinite(self.speed_sensitive_steering)
            and 0.0 <= self.speed_sensitive_steering <= 0.80
        ):
            raise ChainConfigError(
                f"speed_sensitive_steering {self.speed_sensitive_steering} "
                "outside [0, 0.80]"
            )
        if not (
            math.isfinite(self.disconnect_brake) and 0.0 <= self.disconnect_brake <= 1.0
        ):
            raise ChainConfigError("disconnect_brake must be within [0, 1]")

    def axis(self, control: Control) -> AxisChainConfig:
        if control is Control.STEER:
            return self.steer
        if control is Control.THROTTLE:
            return self.throttle
        return self.brake

    def with_axis(self, control: Control, cfg: AxisChainConfig) -> "ChainConfig":
        if control is Control.STEER:
            return replace(self, steer=cfg)
        if control is Control.THROTTLE:
            return replace(self, throttle=cfg)
        return replace(self, brake=cfg)

    def to_dict(self) -> dict[str, object]:
        return {
            "steer": self.steer.to_dict(),
            "throttle": self.throttle.to_dict(),
            "brake": self.brake.to_dict(),
            "speed_sensitive_steering": self.speed_sensitive_steering,
            "brake_cuts_throttle": self.brake_cuts_throttle,
            "disconnect_brake": self.disconnect_brake,
        }

    @classmethod
    def from_dict(cls, data: object) -> "ChainConfig":
        if not isinstance(data, dict):
            raise ChainConfigError(
                f"chain config must be an object, got {type(data).__name__}"
            )
        return cls(
            steer=AxisChainConfig.from_dict(data.get("steer", {})),
            throttle=AxisChainConfig.from_dict(data.get("throttle", {})),
            brake=AxisChainConfig.from_dict(data.get("brake", {})),
            speed_sensitive_steering=_as_float(
                data.get("speed_sensitive_steering", 0.0), "speed_sensitive_steering"
            ),
            brake_cuts_throttle=bool(data.get("brake_cuts_throttle", True)),
            disconnect_brake=_as_float(
                data.get("disconnect_brake", 0.0), "disconnect_brake"
            ),
        )


def default_chain_config(*, digital_pedals: bool = False) -> ChainConfig:
    """Sane starting point.

    With digital pedals every shaping stage except the rate limit is inert -- a
    curve applied to a signal that is only ever 0 or 1 returns 0 or 1 -- so the
    rate limit takes over the job of pedal travel and gets much gentler values.
    Those numbers are the difference between a drivable car and an on/off switch.
    """
    if digital_pedals:
        pedal_rise, pedal_fall = 1.6, 4.0
        pedal_curve = LINEAR
        pedal_smoothing = OneEuroConfig(min_cutoff=3.0, beta=0.05)
    else:
        pedal_rise, pedal_fall = 8.0, 16.0
        pedal_curve = DEFAULT_THROTTLE_CURVE
        pedal_smoothing = OneEuroConfig(min_cutoff=8.0, beta=0.20)

    return ChainConfig(
        steer=AxisChainConfig(
            calibration=AxisCalibration.identity(Control.STEER),
            deadzone=0.04,
            saturation=0.02,
            curve=DEFAULT_STEER_CURVE,
            rate_rise=6.0,
            rate_fall=6.0,
            smoothing=OneEuroConfig(min_cutoff=4.0, beta=0.35),
        ),
        throttle=AxisChainConfig(
            calibration=AxisCalibration.identity(Control.THROTTLE),
            deadzone=0.03,
            saturation=0.03,
            curve=pedal_curve,
            rate_rise=pedal_rise,
            rate_fall=pedal_fall,
            smoothing=pedal_smoothing,
        ),
        brake=AxisChainConfig(
            calibration=AxisCalibration.identity(Control.BRAKE),
            deadzone=0.03,
            saturation=0.03,
            curve=LINEAR if digital_pedals else DEFAULT_BRAKE_CURVE,
            rate_rise=pedal_fall,
            rate_fall=pedal_fall,
            smoothing=pedal_smoothing,
        ),
        speed_sensitive_steering=0.0,
    )


# --------------------------------------------------------------------------
# Stages -- pure functions, each independently testable
# --------------------------------------------------------------------------


def calibrate_bipolar(raw: float, cal: AxisCalibration) -> float:
    """Raw steering units to -1..+1. Each side of centre scales independently."""
    if raw >= cal.rest:
        value = (raw - cal.rest) / (cal.hi - cal.rest)
    else:
        value = (raw - cal.rest) / (cal.rest - cal.lo)
    value *= cal.pre_gain
    if value > 1.0:
        value = 1.0
    elif value < -1.0:
        value = -1.0
    return -value if cal.invert else value


def calibrate_unipolar(raw: float, cal: AxisCalibration) -> float:
    """Raw pedal units to 0..1."""
    value = (raw - cal.rest) / (cal.hi - cal.rest)
    value *= cal.pre_gain
    if value > 1.0:
        value = 1.0
    elif value < 0.0:
        value = 0.0
    return 1.0 - value if cal.invert else value


def center_deadzone(value: float, dz: float) -> float:
    """Kill jitter around centre, rescaling so the response stays continuous.

    The rescale is the whole point. Subtracting the deadzone without it leaves a
    control that can never reach full scale; not subtracting at all leaves a step
    at the deadzone edge that feels like the wheel snapping out of a detent.
    """
    if dz <= 0.0:
        return value
    if value > dz:
        return (value - dz) / (1.0 - dz)
    if value < -dz:
        return (value + dz) / (1.0 - dz)
    return 0.0


def floor_deadzone(value: float, dz: float) -> float:
    """The one-sided version, for pedals. Same continuity argument."""
    if dz <= 0.0:
        return value
    if value <= dz:
        return 0.0
    return (value - dz) / (1.0 - dz)


def saturate(value: float, sat: float) -> float:
    """Treat the top `sat` of travel as full scale, rescaling below it.

    Without this, a pedal that physically stops at 96 % of its calibrated span --
    which is every pedal, once the calibration sweep was done with more
    enthusiasm than the driving -- can never command 100 %.
    """
    if sat <= 0.0:
        return -1.0 if value < -1.0 else 1.0 if value > 1.0 else value
    scaled = value / (1.0 - sat)
    return -1.0 if scaled < -1.0 else 1.0 if scaled > 1.0 else scaled


def rate_limit(target: float, current: float, rise: float, fall: float, dt: float) -> float:
    """Move `current` toward `target`, capped at rise/fall units per second.

    "Rise" is movement away from rest and "fall" is movement toward it, judged by
    magnitude, so a steering input crossing centre is limited by the rate that
    applies to the direction it is actually moving.
    """
    if dt <= 0.0:
        return current
    step = (rise if abs(target) >= abs(current) else fall) * dt
    delta = target - current
    if delta > step:
        return current + step
    if delta < -step:
        return current - step
    return target


def speed_scale(speed_frac: float, max_reduction: float) -> float:
    """Steering authority as a function of speed. 1.0 means untouched."""
    if max_reduction <= 0.0:
        return 1.0
    if speed_frac <= 0.0:
        return 1.0
    if speed_frac > 1.0:
        speed_frac = 1.0
    return 1.0 - max_reduction * speed_frac


def quantize(value: float, scale: int, *, signed: bool) -> int:
    """To the integer units the wire carries. Clamps; never raises."""
    ticks = round(value * scale)
    lo = -scale if signed else 0
    if ticks < lo:
        return lo
    if ticks > scale:
        return scale
    return ticks


def static_map(cfg: AxisChainConfig, raw: float) -> float:
    """calibrate -> deadzone -> saturation -> curve, with no state at all.

    The stateless half of the chain. Every monotonicity, range, and continuity
    property is a property of *this* function; the stateful stages that follow
    only ever move an output toward a value this produced.
    """
    cal = cfg.calibration
    if cal.bipolar:
        value = calibrate_bipolar(raw, cal)
        value = center_deadzone(value, cfg.deadzone)
        value = saturate(value, cfg.saturation)
        return cfg.curve.apply(value)
    value = calibrate_unipolar(raw, cal)
    value = floor_deadzone(value, cfg.deadzone)
    value = saturate(value, cfg.saturation)
    return cfg.curve.apply_unipolar(value)


# --------------------------------------------------------------------------
# One-euro filter
# --------------------------------------------------------------------------


class OneEuroFilter:
    """Adaptive-cutoff low-pass. Stateful, single-threaded, one per axis."""

    __slots__ = ("_cfg", "_x_prev", "_dx_prev", "_primed")

    def __init__(self, cfg: OneEuroConfig) -> None:
        self._cfg = cfg
        self._x_prev = 0.0
        self._dx_prev = 0.0
        self._primed = False

    @property
    def config(self) -> OneEuroConfig:
        return self._cfg

    def set_config(self, cfg: OneEuroConfig) -> None:
        self._cfg = cfg

    def reset(self, value: float = 0.0) -> None:
        self._x_prev = value
        self._dx_prev = 0.0
        self._primed = False

    def filter(self, x: float, dt: float) -> float:
        cfg = self._cfg
        if not cfg.enabled or dt <= 0.0:
            self._x_prev = x
            self._primed = True
            return x
        if not self._primed:
            # First sample defines the state. Starting from zero instead would
            # ramp the very first command up from centre, which on a wheel held
            # at lock is a visible, wrong flick.
            self._x_prev = x
            self._dx_prev = 0.0
            self._primed = True
            return x

        dx = (x - self._x_prev) / dt
        dx_hat = self._dx_prev + _alpha(cfg.d_cutoff, dt) * (dx - self._dx_prev)
        cutoff = cfg.min_cutoff + cfg.beta * (dx_hat if dx_hat >= 0.0 else -dx_hat)
        x_hat = self._x_prev + _alpha(cutoff, dt) * (x - self._x_prev)
        self._dx_prev = dx_hat
        self._x_prev = x_hat
        return x_hat


def _alpha(cutoff: float, dt: float) -> float:
    """Exponential-smoothing coefficient for a given cutoff, in 0..1.

    Derived from the time constant rather than assumed, so the filter behaves
    identically whether the thread ran at 250 Hz or limped through at 90 Hz.
    """
    tau = 1.0 / (2.0 * math.pi * cutoff)
    return dt / (tau + dt)


# --------------------------------------------------------------------------
# Output
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class ChainOutput:
    """One tick's result. Immutable so it can cross a LatestBox unescorted."""

    #: Post-quantization floats -- exactly what the wire carries, so the HUD and
    #: the car cannot disagree by a rounding step.
    steer: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    steer_q: int = 0
    throttle_q: int = 0
    brake_q: int = 0
    #: Steering authority actually applied by the speed-sensitive assist.
    steer_assist: float = 1.0
    #: The source reported no device. See `ChainConfig.disconnect_brake`.
    input_lost: bool = False
    #: A raw value was NaN, infinite, or wildly out of range and was replaced.
    #: A control path must not raise, so this flag is how it gets reported.
    sanitized: bool = False


NEUTRAL_OUTPUT = ChainOutput()


class RawInput(Protocol):
    """The shape `InputChain.update_from` needs.

    Structural rather than a concrete import: `RawSample` lives in sources.py
    alongside the things that do I/O, and this module stays clean of them.
    """

    @property
    def steer(self) -> float: ...

    @property
    def throttle(self) -> float: ...

    @property
    def brake(self) -> float: ...

    @property
    def connected(self) -> bool: ...


# --------------------------------------------------------------------------
# The chain
# --------------------------------------------------------------------------


class InputChain:
    """Runs the chain and owns the small amount of state it needs.

    Not thread-safe and not meant to be: exactly one instance lives on the
    250 Hz input thread. Reconfiguration from the GUI thread goes through
    `pending_config`, which is a single reference assignment -- atomic under the
    GIL -- and gets picked up at the top of the next tick.
    """

    __slots__ = (
        "_cfg",
        "pending_config",
        "_steer",
        "_throttle",
        "_brake",
        "_f_steer",
        "_f_throttle",
        "_f_brake",
    )

    def __init__(self, cfg: ChainConfig | None = None) -> None:
        self._cfg = cfg if cfg is not None else default_chain_config()
        self.pending_config: ChainConfig | None = None
        self._steer = 0.0
        self._throttle = 0.0
        self._brake = 0.0
        self._f_steer = OneEuroFilter(self._cfg.steer.smoothing)
        self._f_throttle = OneEuroFilter(self._cfg.throttle.smoothing)
        self._f_brake = OneEuroFilter(self._cfg.brake.smoothing)

    @property
    def config(self) -> ChainConfig:
        return self._cfg

    def set_config(self, cfg: ChainConfig) -> None:
        """Immediate reconfiguration. Call from the thread that owns the chain;
        from any other thread assign `pending_config` instead."""
        self._cfg = cfg
        self._f_steer.set_config(cfg.steer.smoothing)
        self._f_throttle.set_config(cfg.throttle.smoothing)
        self._f_brake.set_config(cfg.brake.smoothing)

    def reset(self) -> None:
        """Drop all state back to neutral. Call on disarm and on device change --
        carrying a half-applied throttle across an arm boundary would be a
        genuinely dangerous surprise."""
        self._steer = 0.0
        self._throttle = 0.0
        self._brake = 0.0
        self._f_steer.reset()
        self._f_throttle.reset()
        self._f_brake.reset()

    @property
    def state(self) -> tuple[float, float, float]:
        """Post-rate-limit, pre-filter values. For tests and the wizard preview."""
        return (self._steer, self._throttle, self._brake)

    def update_from(
        self, sample: RawInput, dt: float, *, speed_frac: float = 0.0
    ) -> ChainOutput:
        """`update` fed straight from a source's sample. What the input thread calls."""
        return self.update(
            sample.steer,
            sample.throttle,
            sample.brake,
            dt,
            speed_frac=speed_frac,
            connected=sample.connected,
        )

    def update(
        self,
        steer_raw: float,
        throttle_raw: float,
        brake_raw: float,
        dt: float,
        *,
        speed_frac: float = 0.0,
        connected: bool = True,
    ) -> ChainOutput:
        """One tick. Never raises: bad numbers are replaced and flagged."""
        pending = self.pending_config
        if pending is not None:
            self.pending_config = None
            self.set_config(pending)

        cfg = self._cfg
        sanitized = False

        if dt < 0.0 or not math.isfinite(dt):
            dt = 0.0
            sanitized = True
        elif dt > MAX_DT:
            dt = MAX_DT

        if not math.isfinite(steer_raw):
            steer_raw = cfg.steer.calibration.rest
            sanitized = True
        if not math.isfinite(throttle_raw):
            throttle_raw = cfg.throttle.calibration.rest
            sanitized = True
        if not math.isfinite(brake_raw):
            brake_raw = cfg.brake.calibration.rest
            sanitized = True

        if connected:
            steer_target = static_map(cfg.steer, steer_raw)
            throttle_target = static_map(cfg.throttle, throttle_raw)
            brake_target = static_map(cfg.brake, brake_raw)
        else:
            # The wheel was unplugged. Steering ramps to centre at its normal
            # rate; throttle is cut without waiting for the ramp, because there
            # is no reading left to justify holding it open.
            steer_target = 0.0
            throttle_target = 0.0
            brake_target = cfg.disconnect_brake
            self._throttle = 0.0
            self._f_throttle.reset(0.0)

        assist = speed_scale(speed_frac, cfg.speed_sensitive_steering)
        if assist != 1.0:
            steer_target *= assist

        if cfg.brake_cuts_throttle and brake_target > 0.0:
            throttle_target = 0.0

        steer = rate_limit(
            steer_target, self._steer, cfg.steer.rate_rise, cfg.steer.rate_fall, dt
        )
        throttle = rate_limit(
            throttle_target,
            self._throttle,
            cfg.throttle.rate_rise,
            cfg.throttle.rate_fall,
            dt,
        )
        brake = rate_limit(
            brake_target, self._brake, cfg.brake.rate_rise, cfg.brake.rate_fall, dt
        )
        self._steer = steer
        self._throttle = throttle
        self._brake = brake

        steer = self._f_steer.filter(steer, dt)
        throttle = self._f_throttle.filter(throttle, dt)
        brake = self._f_brake.filter(brake, dt)

        steer_q = quantize(steer, STEERING_SCALE, signed=True)
        throttle_q = quantize(throttle, THROTTLE_SCALE, signed=False)
        brake_q = quantize(brake, BRAKE_SCALE, signed=False)

        return ChainOutput(
            steer=steer_q / STEERING_SCALE,
            throttle=throttle_q / THROTTLE_SCALE,
            brake=brake_q / BRAKE_SCALE,
            steer_q=steer_q,
            throttle_q=throttle_q,
            brake_q=brake_q,
            steer_assist=assist,
            input_lost=not connected,
            sanitized=sanitized,
        )


def _as_float(value: object, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ChainConfigError(f"{what} must be a number, got {type(value).__name__}")
    return float(value)
