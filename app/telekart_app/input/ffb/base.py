"""The force-feedback seam.

The user's wheel has no force feedback, so nothing here drives a motor today.
The interface exists anyway, and it is worth being explicit about why: force
feedback is the one input feature that cannot be bolted on later without
touching the input thread, the telemetry consumer, and the profile format all at
once. Defining the seam now costs one small file and makes a future backend a
drop-in.

The split between this base class and a real backend is deliberate:

* The **backend** owns transport -- opening the device, uploading effects,
  keeping them alive. That is the part that differs between SDL haptics, the
  Logitech HID protocol, and a Linux evdev FF device.
* The **base class** owns policy -- how much centring spring at what speed, what
  slip feels like. `update()` here is a complete, working feel model written
  against the effect primitives, so a new backend inherits a sensible car
  without reimplementing any of it.

The car's telemetry is the only input to that model. There is no force sensor on
an RC car, so everything is synthesised from speed, commanded steering angle and
the slip index -- which is exactly what a racing sim does, and it is convincing
for the same reasons.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Any, ClassVar


@dataclass(frozen=True, slots=True)
class FfbCapabilities:
    """What a backend can actually do. All False on the null backend.

    The UI reads this to decide which sliders to show. Showing a spring-strength
    slider that does nothing is worse than showing none at all.
    """

    constant_force: bool = False
    spring: bool = False
    damper: bool = False
    friction: bool = False
    rumble: bool = False
    autocenter: bool = False
    #: Number of simultaneous effects the device will hold, when known.
    max_effects: int = 0

    @property
    def any_force(self) -> bool:
        return self.constant_force or self.spring or self.damper or self.friction

    @property
    def any(self) -> bool:
        return self.any_force or self.rumble or self.autocenter


NO_CAPABILITIES = FfbCapabilities()


@dataclass(frozen=True, slots=True)
class FfbState:
    """Everything the feel model consumes, in normalized or SI units.

    Assembled from the telemetry packet and the chain output by whoever owns the
    input thread. Immutable, so it can be handed across threads without a copy.
    """

    #: Speed as a fraction of the car's *measured* maximum. Nothing here uses an
    #: absolute speed: the drivetrain's top speed is a measurement that changes
    #: with the battery and the surface, and a force model tied to an absolute
    #: number would feel different every session.
    speed_frac: float = 0.0
    #: What the app commanded, -1..+1.
    steer_command: float = 0.0
    #: What the car reports, radians at the road wheel.
    steer_angle: float = 0.0
    #: Encoder-versus-model disagreement. Rises with wheelspin and with a bad
    #: geometry calibration.
    slip: float = 0.0
    limiter_active: bool = False
    braking: bool = False
    armed: bool = False
    faulted: bool = False


NEUTRAL_STATE = FfbState()


@dataclass(frozen=True, slots=True)
class FfbCommand:
    """What the feel model decided, before any capability filtering.

    Separating the decision from the dispatch is what makes the model testable
    without a device, and what lets the null backend show the settings screen
    exactly what a real wheel would be doing.
    """

    spring: float = 0.0
    damper: float = 0.0
    friction: float = 0.0
    constant: float = 0.0
    rumble: float = 0.0
    #: Rumble burst length, seconds. Zero when there is nothing to feel.
    rumble_duration: float = 0.0
    silent: bool = True


SILENT_COMMAND = FfbCommand()


@dataclass(frozen=True, slots=True)
class FfbTuning:
    """Feel parameters. Normalized 0..1 forces throughout.

    Defaults are conservative because an over-strong centring spring on a wheel
    with no torque headroom is indistinguishable from a broken one.
    """

    #: Centring spring at a standstill, and how much it grows with speed. Real
    #: steering gets heavier with speed; this is the single cue that makes a
    #: synthetic model feel like a car rather than a rubber band.
    center_strength: float = 0.25
    center_speed_gain: float = 0.45
    #: Damping, which is what stops the wheel oscillating when released.
    damper: float = 0.15
    #: Friction, mostly to mask a cheap wheel's cogging.
    friction: float = 0.05
    #: Slip above this triggers a rumble; the amplitude scales past it.
    slip_threshold: float = 0.25
    slip_rumble: float = 0.40
    #: Rumble when the firmware's protection limiter engages, so the driver
    #: feels what `TelemetryFlags.LIMITER_ACTIVE` is telling the HUD.
    limiter_rumble: float = 0.25
    #: Global scale, exposed as one slider.
    master: float = 1.0

    def __post_init__(self) -> None:
        for name in (
            "center_strength",
            "center_speed_gain",
            "damper",
            "friction",
            "slip_threshold",
            "slip_rumble",
            "limiter_rumble",
            "master",
        ):
            value = getattr(self, name)
            if not (0.0 <= float(value) <= 2.0):
                raise ValueError(f"ffb tuning {name}={value} outside [0, 2]")


class FfbBackend(abc.ABC):
    """One force-feedback transport.

    Effect setters default to no-ops so a partial backend -- rumble only, say --
    implements what it has and inherits silence for the rest. Every one of them
    takes normalized units and must clamp rather than raise: they are called from
    the input thread at 250 Hz.
    """

    #: Registry key. Set on every subclass.
    backend_id: ClassVar[str] = "base"
    #: Human-readable, for the settings screen.
    label: ClassVar[str] = "Force feedback"

    __slots__ = ("_tuning", "_rumble_until")

    def __init__(self, tuning: FfbTuning | None = None) -> None:
        self._tuning = tuning if tuning is not None else FfbTuning()
        self._rumble_until = 0.0

    # -- discovery ----------------------------------------------------------

    @property
    def available(self) -> bool:
        """Cheap probe: is this backend usable on this machine at all?

        Checked before `open`, so the registry can skip a backend whose driver
        library is not installed without paying for a device enumeration.
        """
        return False

    @property
    @abc.abstractmethod
    def capabilities(self) -> FfbCapabilities: ...

    @property
    @abc.abstractmethod
    def is_open(self) -> bool: ...

    # -- lifecycle ----------------------------------------------------------

    @abc.abstractmethod
    def open(self, *, guid: str = "", device: Any = None) -> bool:
        """Claim the device. Returns success; must not raise."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release the device and stop every effect. Must be idempotent, and
        must be safe to call from an atexit handler -- a wheel left holding full
        force after the app exits keeps holding it until it is unplugged."""

    def __enter__(self) -> "FfbBackend":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- tuning -------------------------------------------------------------

    @property
    def tuning(self) -> FfbTuning:
        return self._tuning

    def set_tuning(self, tuning: FfbTuning) -> None:
        self._tuning = tuning

    # -- effect primitives --------------------------------------------------

    def set_autocenter(self, strength: float) -> None:
        """Device-side centring spring, 0..1. Cheaper than a real spring effect
        on wheels that support it, and completely ignored by those that do not."""

    def set_constant_force(self, force: float) -> None:
        """Steady torque, -1..+1. Positive pulls the wheel clockwise."""

    def set_spring(self, center: float, strength: float) -> None:
        """Force proportional to displacement from `center` (-1..+1)."""

    def set_damper(self, strength: float) -> None:
        """Force proportional to wheel velocity, opposing it."""

    def set_friction(self, strength: float) -> None:
        """Constant opposing force independent of speed."""

    def rumble(self, low: float, high: float, duration: float) -> None:
        """Dual-motor rumble burst, 0..1 each, seconds."""

    def stop_all(self) -> None:
        """Silence everything immediately."""

    # -- the feel model -----------------------------------------------------

    def compute(self, state: FfbState, dt: float) -> FfbCommand:
        """The feel model. Pure: same state in, same command out, no device.

        Everything is synthesised, because an RC car has no torque sensor. The
        cues, in order of how much they matter:

        1. A centring spring that stiffens with speed. This is the one cue that
           makes a synthetic model read as a car instead of a rubber band.
        2. Damping, which stops the wheel oscillating when the driver lets go.
        3. A rumble on slip or on the firmware's protection limiter, so the two
           conditions the HUD flags are also felt.
        """
        if not state.armed or state.faulted:
            # Disarmed means the servo is relaxed on the car. Holding a centring
            # force against road wheels that are limp is a lie the driver will
            # believe, and it is the sort of lie that gets a car driven into a
            # wall during the first corner after arming.
            return SILENT_COMMAND

        tuning = self._tuning
        master = tuning.master

        speed = state.speed_frac
        if speed < 0.0:
            speed = 0.0
        elif speed > 1.0:
            speed = 1.0

        spring = (tuning.center_strength + tuning.center_speed_gain * speed) * master
        if spring > 1.0:
            spring = 1.0

        amplitude = 0.0
        if state.slip > tuning.slip_threshold:
            span = 1.0 - tuning.slip_threshold
            scaled = (state.slip - tuning.slip_threshold) / span if span > 0.0 else 1.0
            amplitude = tuning.slip_rumble * (scaled if scaled < 1.0 else 1.0)
        if state.limiter_active and tuning.limiter_rumble > amplitude:
            amplitude = tuning.limiter_rumble
        amplitude *= master
        if amplitude > 1.0:
            amplitude = 1.0

        return FfbCommand(
            spring=spring,
            damper=tuning.damper * master,
            friction=tuning.friction * master,
            constant=0.0,
            rumble=amplitude,
            # Two ticks' worth, re-issued every tick. A burst that outlives the
            # condition that caused it teaches the driver nothing.
            rumble_duration=(dt * 2.0 if dt * 2.0 > 0.02 else 0.02),
            silent=False,
        )

    def update(self, state: FfbState, dt: float) -> None:
        """One tick: compute, then dispatch through what the device can do.

        Called from the input thread. Never raises, and allocates only the
        command object.
        """
        if not self.is_open:
            return
        command = self.compute(state, dt)
        if command.silent:
            self.stop_all()
            return

        caps = self.capabilities
        if caps.spring:
            self.set_spring(0.0, command.spring)
        elif caps.autocenter:
            self.set_autocenter(command.spring)
        if caps.constant_force and command.constant != 0.0:
            self.set_constant_force(command.constant)
        if caps.damper:
            self.set_damper(command.damper)
        if caps.friction:
            self.set_friction(command.friction)
        if caps.rumble and command.rumble > 0.0:
            self.rumble(command.rumble, command.rumble * 0.5, command.rumble_duration)

    def __repr__(self) -> str:
        return f"{type(self).__name__}(open={self.is_open})"
