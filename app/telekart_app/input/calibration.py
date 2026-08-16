"""The calibration state machine.

No Qt, no SDL, no timers of its own. The wizard UI is somebody else's file; it
feeds this snapshots and calls `advance()`, and this decides what to say next and
what the resulting profile is. Keeping the two apart is what makes calibration
testable: a whole calibration run -- wiggle detection, range capture, deadzone
measurement, verification -- executes in a few milliseconds against synthetic
snapshots, with no window on screen and no wheel on the desk.

The design principle throughout is that **the driver never sees an axis index**.
They are asked to turn the wheel and press the pedals, and the machine works out
which numbers moved. That is not a cosmetic choice: the previous generation of
this project shipped a config file with `throttle_button = 7` in it, and every
new wheel meant editing JSON by hand and guessing.

One hardware quirk gets explicit handling because it is common and utterly
baffling when you meet it: many wheels default to a **combined pedal mode**, in
which throttle and brake are wired to opposite halves of a single axis instead
of one axis each. Auto-assignment then picks the same axis for both pedals, and
without a specific diagnosis the driver is left with a wheel that brakes when
they lift off. The machine detects that case, says so in words, and offers the
split as a workaround while they find the button combination that fixes it.
"""

from __future__ import annotations

import enum
import math
import time
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Protocol

from .chain import (
    NO_SMOOTHING,
    AxisCalibration,
    AxisChainConfig,
    ChainConfig,
    ChainConfigError,
    InputChain,
    calibrate_bipolar,
    calibrate_unipolar,
    default_chain_config,
    static_map,
)
from .curves import Curve, CurveSpec
from .mapping import (
    ACTION_LABELS,
    Action,
    ActionBinding,
    Control,
    ControlBinding,
    InputMap,
    InputRef,
    RefKind,
    axis as axis_ref,
    button as button_ref,
    resolve_binding,
)
from .profile import DeviceKind, DeviceProfile, preset_for_device
from .sources import EMPTY_SNAPSHOT, DeviceSnapshot

Clock = Callable[[], float]

#: Idle capture length. Two seconds is enough for several hundred samples at any
#: sane pump rate, and short enough that nobody stops paying attention.
IDLE_CAPTURE_S = 2.0
#: Wiggle capture length, per control.
WIGGLE_CAPTURE_S = 4.0
#: Range sweep length.
RANGE_CAPTURE_S = 6.0
#: Deadzone re-measurement at rest, after ranges are known.
DEADZONE_CAPTURE_S = 1.5
#: Saturation capture: "press it as hard as you would when driving".
SATURATION_CAPTURE_S = 3.0

#: An axis must move at least this much of full scale during a wiggle to count.
#: Below it, the movement is indistinguishable from a neighbouring axis's
#: crosstalk, which is real on cheap wheels sharing an ADC.
MIN_WIGGLE_TRAVEL = 0.25

#: An axis resting within this of +/-1.0 is almost certainly a pedal that idles
#: at one end of its travel, which is how every Logitech pedal set behaves.
REST_EXTREME_TOLERANCE = 0.15

#: Deadzone floor and ceiling derived from measured noise.
MIN_MEASURED_DEADZONE = 0.01
MAX_MEASURED_DEADZONE = 0.25
#: Noise is converted to a deadzone at this many sigma. Three sigma leaves about
#: one sample in 370 outside the band, and the rate limiter absorbs those.
DEADZONE_SIGMA = 3.0

#: Verification thresholds.
VERIFY_FULL = 0.98
VERIFY_NEUTRAL = 0.02


class CalStep(enum.Enum):
    DETECT = "detect"
    IDLE = "idle"
    ASSIGN_STEER = "assign_steer"
    ASSIGN_THROTTLE = "assign_throttle"
    ASSIGN_BRAKE = "assign_brake"
    RANGES = "ranges"
    DEADZONE = "deadzone"
    SATURATION = "saturation"
    INVERSION = "inversion"
    CURVE = "curve"
    ROTATION_LOCK = "rotation_lock"
    BINDINGS = "bindings"
    VERIFY = "verify"
    DONE = "done"


STEP_ORDER: tuple[CalStep, ...] = (
    CalStep.DETECT,
    CalStep.IDLE,
    CalStep.ASSIGN_STEER,
    CalStep.ASSIGN_THROTTLE,
    CalStep.ASSIGN_BRAKE,
    CalStep.RANGES,
    CalStep.DEADZONE,
    CalStep.SATURATION,
    CalStep.INVERSION,
    CalStep.CURVE,
    CalStep.ROTATION_LOCK,
    CalStep.BINDINGS,
    CalStep.VERIFY,
    CalStep.DONE,
)

#: Steps that only make sense with a device attached and that the driver may
#: legitimately skip -- someone recalibrating one pedal should not have to
#: re-teach the machine every button.
SKIPPABLE: frozenset[CalStep] = frozenset(
    {
        CalStep.SATURATION,
        CalStep.INVERSION,
        CalStep.CURVE,
        CalStep.ROTATION_LOCK,
        CalStep.BINDINGS,
        CalStep.VERIFY,
    }
)

STEP_TITLES: dict[CalStep, str] = {
    CalStep.DETECT: "Find your controller",
    CalStep.IDLE: "Hands off",
    CalStep.ASSIGN_STEER: "Steering",
    CalStep.ASSIGN_THROTTLE: "Throttle",
    CalStep.ASSIGN_BRAKE: "Brake",
    CalStep.RANGES: "Full travel",
    CalStep.DEADZONE: "Centre and rest",
    CalStep.SATURATION: "Comfortable full",
    CalStep.INVERSION: "Directions",
    CalStep.CURVE: "Response",
    CalStep.ROTATION_LOCK: "Steering lock",
    CalStep.BINDINGS: "Buttons",
    CalStep.VERIFY: "Check",
    CalStep.DONE: "Done",
}

STEP_PROMPTS: dict[CalStep, str] = {
    CalStep.DETECT: "Plug in your wheel or controller.",
    CalStep.IDLE: "Take your hands and feet off everything for a moment.",
    CalStep.ASSIGN_STEER: "Turn fully LEFT, then fully RIGHT, then let go.",
    CalStep.ASSIGN_THROTTLE: "Press the throttle all the way down, then release it.",
    CalStep.ASSIGN_BRAKE: "Press the brake all the way down, then release it.",
    CalStep.RANGES: "Sweep everything: lock to lock, both pedals to the floor.",
    CalStep.DEADZONE: "Hands and feet off again.",
    CalStep.SATURATION: "Press each pedal as far as you actually would when driving.",
    CalStep.INVERSION: "Check that the bars move the way you expect. Flip any that do not.",
    CalStep.CURVE: "Choose how the controls respond. Linear is a fine place to start.",
    CalStep.ROTATION_LOCK: "Turn the wheel to where you want full lock to be, then continue.",
    CalStep.BINDINGS: "Press the control you want for each action.",
    CalStep.VERIFY: "Move everything through its full range once more.",
    CalStep.DONE: "Calibration complete.",
}


class DeviceInfo(Protocol):
    """What the machine needs to know about the attached device."""

    @property
    def guid(self) -> str: ...

    @property
    def name(self) -> str: ...

    @property
    def num_axes(self) -> int: ...

    @property
    def num_buttons(self) -> int: ...

    @property
    def num_hats(self) -> int: ...


class _AxisStats:
    """Streaming statistics for one axis, with the *time* of each extreme.

    The timestamps are the interesting part. Knowing that an axis reached its
    minimum before its maximum is what turns "turn left, then right" into a
    signed assignment without asking the driver which way is positive.
    """

    __slots__ = ("count", "_sum", "_sumsq", "minimum", "maximum", "t_min", "t_max", "first", "last")

    def __init__(self) -> None:
        self.count = 0
        self._sum = 0.0
        self._sumsq = 0.0
        self.minimum = math.inf
        self.maximum = -math.inf
        self.t_min = 0.0
        self.t_max = 0.0
        self.first = 0.0
        self.last = 0.0

    def add(self, value: float, t: float) -> None:
        if not math.isfinite(value):
            return
        if self.count == 0:
            self.first = value
        self.count += 1
        self._sum += value
        self._sumsq += value * value
        self.last = value
        if value < self.minimum:
            self.minimum = value
            self.t_min = t
        if value > self.maximum:
            self.maximum = value
            self.t_max = t

    @property
    def mean(self) -> float:
        return self._sum / self.count if self.count else 0.0

    @property
    def sigma(self) -> float:
        if self.count < 2:
            return 0.0
        variance = self._sumsq / self.count - self.mean * self.mean
        return math.sqrt(variance) if variance > 0.0 else 0.0

    @property
    def span(self) -> float:
        if self.count == 0:
            return 0.0
        return self.maximum - self.minimum


@dataclass(slots=True)
class IdleReport:
    """What the idle capture found."""

    means: tuple[float, ...] = ()
    sigmas: tuple[float, ...] = ()
    #: Axis indices resting near +/-1.0. Almost always pedals.
    resting_at_extreme: tuple[int, ...] = ()
    #: Axis indices whose noise is large enough to need a real deadzone.
    noisy: tuple[int, ...] = ()
    samples: int = 0


@dataclass(slots=True)
class AssignResult:
    """One control's auto-assignment."""

    control: Control
    ref: InputRef = field(default_factory=InputRef)
    negative_ref: InputRef = field(default_factory=InputRef)
    #: Raw value at rest, and at the far end of travel.
    rest: float = 0.0
    extreme: float = 0.0
    minimum: float = 0.0
    maximum: float = 0.0
    invert: bool = False
    digital: bool = False
    confident: bool = False
    detail: str = ""


@dataclass(slots=True)
class VerifyReport:
    steer_left: float = 0.0
    steer_right: float = 0.0
    throttle_max: float = 0.0
    brake_max: float = 0.0
    returns_to_neutral: bool = False
    #: Populated as checks fail, so the UI can list what still needs doing.
    problems: tuple[str, ...] = ()

    @property
    def passed(self) -> bool:
        return not self.problems


@dataclass(slots=True)
class _Working:
    """Mutable calibration in progress. Converted to a profile on demand."""

    rest: dict[Control, float] = field(default_factory=dict)
    lo: dict[Control, float] = field(default_factory=dict)
    hi: dict[Control, float] = field(default_factory=dict)
    invert: dict[Control, bool] = field(default_factory=dict)
    pre_gain: dict[Control, float] = field(default_factory=dict)
    deadzone: dict[Control, float] = field(default_factory=dict)
    saturation: dict[Control, float] = field(default_factory=dict)
    curve: dict[Control, Curve] = field(default_factory=dict)
    refs: dict[Control, ControlBinding] = field(default_factory=dict)
    actions: dict[Action, InputRef] = field(default_factory=dict)


class CalibrationMachine:
    """Drives a calibration run. Feed it snapshots; ask it what to say.

    Nothing here blocks, sleeps, or talks to hardware. `feed` is cheap enough to
    call from a 60 Hz UI timer, and every timed step measures elapsed time from
    the injected clock, so a test can run the whole sequence instantly.
    """

    __slots__ = (
        "_clock",
        "_step",
        "_step_started",
        "_snapshot",
        "_device",
        "_profile",
        "_working",
        "_stats",
        "_stats_started",
        "_idle",
        "_assign",
        "_warnings",
        "_binding_index",
        "_binding_order",
        "_binding_armed",
        "_prev_buttons",
        "_verify",
        "_verify_chain",
        "_verify_last_t",
        "_activity_seen",
        "_cancelled",
        "on_change",
        "auto_advance",
    )

    def __init__(
        self,
        *,
        clock: Clock = time.perf_counter,
        on_change: Callable[[], None] | None = None,
        auto_advance: bool = True,
    ) -> None:
        self._clock = clock
        self._step = CalStep.DETECT
        self._step_started = clock()
        self._snapshot: DeviceSnapshot = EMPTY_SNAPSHOT
        self._device: DeviceInfo | None = None
        self._profile: DeviceProfile | None = None
        self._working = _Working()
        self._stats: list[_AxisStats] = []
        self._stats_started = 0.0
        self._idle = IdleReport()
        self._assign: dict[Control, AssignResult] = {}
        self._warnings: list[str] = []
        self._binding_order: tuple[Action, ...] = _default_binding_order()
        self._binding_index = 0
        self._binding_armed = False
        self._prev_buttons = 0
        self._verify = VerifyReport()
        self._verify_chain: InputChain | None = None
        self._verify_last_t = 0.0
        self._activity_seen = False
        self._cancelled = False
        self.on_change = on_change
        self.auto_advance = auto_advance

    # ------------------------------------------------------------------
    # Attachment
    # ------------------------------------------------------------------

    def attach(
        self,
        *,
        guid: str,
        name: str,
        num_axes: int,
        num_buttons: int,
        num_hats: int = 0,
    ) -> None:
        """Tell the machine which device it is calibrating.

        Seeds a preset immediately, so every later step edits a working profile
        rather than building one from nothing -- which means an abandoned
        calibration still leaves something drivable behind.
        """
        self._device = _StaticDeviceInfo(guid, name, num_axes, num_buttons, num_hats)
        self._profile = preset_for_device(
            guid=guid,
            name=name,
            num_axes=num_axes,
            num_buttons=num_buttons,
            num_hats=num_hats,
        )
        self._seed_working_from(self._profile)
        self._stats = [_AxisStats() for _ in range(max(num_axes, 0))]
        self._notify()

    def attach_device(self, device: DeviceInfo) -> None:
        self.attach(
            guid=device.guid,
            name=device.name,
            num_axes=device.num_axes,
            num_buttons=device.num_buttons,
            num_hats=device.num_hats,
        )

    def _seed_working_from(self, profile: DeviceProfile) -> None:
        working = _Working()
        for control in Control:
            cfg = profile.chain.axis(control)
            cal = cfg.calibration
            working.rest[control] = cal.rest
            working.lo[control] = cal.lo
            working.hi[control] = cal.hi
            working.invert[control] = cal.invert
            working.pre_gain[control] = cal.pre_gain
            working.deadzone[control] = cfg.deadzone
            working.saturation[control] = cfg.saturation
            working.curve[control] = cfg.curve
            working.refs[control] = profile.mapping.binding(control)
        for entry in profile.mapping.actions:
            working.actions[entry.action] = entry.ref
        self._working = working

    # ------------------------------------------------------------------
    # Public state
    # ------------------------------------------------------------------

    @property
    def step(self) -> CalStep:
        return self._step

    @property
    def title(self) -> str:
        return STEP_TITLES[self._step]

    @property
    def prompt(self) -> str:
        if self._step is CalStep.BINDINGS:
            action = self.binding_target
            if action is not None:
                return f"Press the control for: {ACTION_LABELS[action]}"
            return "All actions bound."
        return STEP_PROMPTS[self._step]

    @property
    def detail(self) -> str:
        """Secondary line: what the machine currently believes."""
        step = self._step
        if step is CalStep.DETECT:
            if self._device is None:
                return "No controller found yet."
            device = self._device
            return (
                f"{device.name} - {device.num_axes} axes, "
                f"{device.num_buttons} buttons"
            )
        if step is CalStep.IDLE:
            return f"{self._idle.samples} samples"
        if step in (CalStep.ASSIGN_STEER, CalStep.ASSIGN_THROTTLE, CalStep.ASSIGN_BRAKE):
            result = self._assign.get(_ASSIGN_CONTROL[step])
            if result is None or not result.ref.is_bound:
                return "Nothing detected yet."
            return f"{result.ref.describe()}{' - ' + result.detail if result.detail else ''}"
        if step is CalStep.RANGES:
            return self._range_detail()
        if step is CalStep.DEADZONE:
            return "; ".join(
                f"{c.value} {self._working.deadzone.get(c, 0.0) * 100.0:.1f}%"
                for c in Control
            )
        if step is CalStep.SATURATION:
            return "; ".join(
                f"{c.value} {self._working.saturation.get(c, 0.0) * 100.0:.1f}%"
                for c in Control
            )
        if step is CalStep.ROTATION_LOCK:
            gain = self._working.pre_gain.get(Control.STEER, 1.0)
            return f"using 1/{gain:.2f} of the wheel's travel for full lock"
        if step is CalStep.VERIFY:
            return "; ".join(self._verify.problems) if self._verify.problems else "All good."
        return ""

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple(self._warnings)

    @property
    def idle_report(self) -> IdleReport:
        return self._idle

    @property
    def verify_report(self) -> VerifyReport:
        return self._verify

    @property
    def assignments(self) -> dict[Control, AssignResult]:
        return dict(self._assign)

    @property
    def device_name(self) -> str:
        return self._device.name if self._device is not None else ""

    @property
    def cancelled(self) -> bool:
        return self._cancelled

    @property
    def step_index(self) -> int:
        return STEP_ORDER.index(self._step)

    @property
    def overall_progress(self) -> float:
        return self.step_index / (len(STEP_ORDER) - 1)

    @property
    def step_progress(self) -> float:
        """0..1 within a timed step; 0.0 for steps that wait on the driver."""
        duration = _STEP_DURATION.get(self._step, 0.0)
        if duration <= 0.0:
            return 0.0
        elapsed = self._clock() - self._step_started
        value = elapsed / duration
        return 1.0 if value > 1.0 else 0.0 if value < 0.0 else value

    @property
    def can_advance(self) -> bool:
        step = self._step
        if step is CalStep.DETECT:
            return self._device is not None
        if step is CalStep.DONE:
            return False
        if step is CalStep.BINDINGS:
            return self.binding_target is None
        duration = _STEP_DURATION.get(step, 0.0)
        if duration > 0.0:
            return self._clock() - self._step_started >= duration
        return True

    @property
    def can_skip(self) -> bool:
        return self._step in SKIPPABLE

    @property
    def binding_target(self) -> Action | None:
        if self._binding_index >= len(self._binding_order):
            return None
        return self._binding_order[self._binding_index]

    @property
    def profile(self) -> DeviceProfile:
        """The profile as it stands. Valid at every step, never half-built."""
        return self._build_profile()

    # ------------------------------------------------------------------
    # Feeding
    # ------------------------------------------------------------------

    def feed(self, snapshot: DeviceSnapshot) -> None:
        """One device sample. Safe to call at any rate, including not at all."""
        if self._cancelled:
            return
        self._snapshot = snapshot
        now = self._clock()

        if snapshot.connected and (snapshot.buttons or _any_moved(snapshot, self._stats)):
            self._activity_seen = True

        handler = _FEED.get(self._step)
        if handler is not None:
            handler(self, snapshot, now)

        if self.auto_advance and self._should_auto_advance(now):
            self.advance()

    def _should_auto_advance(self, now: float) -> bool:
        duration = _STEP_DURATION.get(self._step, 0.0)
        if duration <= 0.0:
            return False
        return (now - self._step_started) >= duration

    # -- per-step feed handlers ----------------------------------------

    def _feed_detect(self, snapshot: DeviceSnapshot, now: float) -> None:
        return None

    def _feed_idle(self, snapshot: DeviceSnapshot, now: float) -> None:
        self._accumulate(snapshot, now)

    def _feed_wiggle(self, snapshot: DeviceSnapshot, now: float) -> None:
        self._accumulate(snapshot, now)
        self._track_buttons(snapshot, now)

    def _feed_ranges(self, snapshot: DeviceSnapshot, now: float) -> None:
        self._accumulate(snapshot, now)

    def _feed_deadzone(self, snapshot: DeviceSnapshot, now: float) -> None:
        self._accumulate(snapshot, now)

    def _feed_saturation(self, snapshot: DeviceSnapshot, now: float) -> None:
        self._accumulate(snapshot, now)

    def _feed_bindings(self, snapshot: DeviceSnapshot, now: float) -> None:
        """Bind the next action to whatever the driver presses.

        Waits for every button to be released before arming. Without that, the
        press that confirmed the previous action gets consumed as the next one
        the moment the step advances, and the driver binds four actions to one
        button in half a second.
        """
        target = self.binding_target
        if target is None:
            return
        pressed = snapshot.buttons & ~self._prev_buttons
        edges = snapshot.pressed_edges
        self._prev_buttons = snapshot.buttons

        if not self._binding_armed:
            if snapshot.buttons == 0:
                self._binding_armed = True
            return

        candidate = pressed | edges
        if candidate:
            index = (candidate & -candidate).bit_length() - 1
            self._working.actions[target] = button_ref(index)
            self._binding_index += 1
            self._binding_armed = False
            self._notify()
            return

        for hat_index, (hx, hy) in enumerate(snapshot.hats):
            if hx:
                self._working.actions[target] = InputRef(
                    RefKind.HAT, hat_index, 0, 1 if hx > 0 else -1
                )
            elif hy:
                self._working.actions[target] = InputRef(
                    RefKind.HAT, hat_index, 1, 1 if hy > 0 else -1
                )
            else:
                continue
            self._binding_index += 1
            self._binding_armed = False
            self._notify()
            return

    def _feed_verify(self, snapshot: DeviceSnapshot, now: float) -> None:
        chain = self._verify_chain
        if chain is None:
            return
        dt = now - self._verify_last_t
        self._verify_last_t = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02

        mapping = self._build_mapping()
        steer_raw = resolve_binding(
            mapping.steer, snapshot.axes, snapshot.buttons, snapshot.hats, 0
        )
        throttle_raw = resolve_binding(
            mapping.throttle, snapshot.axes, snapshot.buttons, snapshot.hats, 0
        )
        brake_raw = resolve_binding(
            mapping.brake, snapshot.axes, snapshot.buttons, snapshot.hats, 0
        )
        out = chain.update(
            steer_raw, throttle_raw, brake_raw, dt, connected=snapshot.connected
        )

        report = self._verify
        if out.steer < report.steer_left:
            report.steer_left = out.steer
        if out.steer > report.steer_right:
            report.steer_right = out.steer
        if out.throttle > report.throttle_max:
            report.throttle_max = out.throttle
        if out.brake > report.brake_max:
            report.brake_max = out.brake
        if (
            abs(out.steer) <= VERIFY_NEUTRAL
            and out.throttle <= VERIFY_NEUTRAL
            and out.brake <= VERIFY_NEUTRAL
        ):
            report.returns_to_neutral = True
        self._recheck_verify()

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def advance(self) -> bool:
        """Finish the current step and move on. Returns False if not ready."""
        if self._cancelled or self._step is CalStep.DONE:
            return False
        if not self.can_advance:
            return False
        exit_handler = _EXIT.get(self._step)
        if exit_handler is not None:
            exit_handler(self)
        index = STEP_ORDER.index(self._step)
        self._enter(STEP_ORDER[min(index + 1, len(STEP_ORDER) - 1)])
        return True

    def back(self) -> bool:
        """Return to the previous step, discarding its captured data."""
        if self._cancelled:
            return False
        index = STEP_ORDER.index(self._step)
        if index == 0:
            return False
        self._enter(STEP_ORDER[index - 1])
        return True

    def skip(self) -> bool:
        """Leave this step's values as they are and move on."""
        if not self.can_skip:
            return False
        index = STEP_ORDER.index(self._step)
        self._enter(STEP_ORDER[min(index + 1, len(STEP_ORDER) - 1)])
        return True

    def restart_step(self) -> None:
        self._enter(self._step)

    def cancel(self) -> None:
        self._cancelled = True
        self._notify()

    def _enter(self, step: CalStep) -> None:
        self._step = step
        self._step_started = self._clock()
        self._reset_stats()
        if step is CalStep.BINDINGS:
            self._binding_index = 0
            self._binding_armed = False
            self._prev_buttons = self._snapshot.buttons
        elif step is CalStep.VERIFY:
            self._verify = VerifyReport()
            self._verify_last_t = self._clock()
            try:
                self._verify_chain = InputChain(self._build_chain())
            except ChainConfigError as exc:
                # A configuration this machine built should always be valid; if
                # it is not, that is a bug worth surfacing here rather than at
                # the first corner.
                self._verify_chain = None
                self._warn(f"cannot verify: {exc}")
            self._recheck_verify()
        self._notify()

    # ------------------------------------------------------------------
    # Step exits -- where the measurements turn into settings
    # ------------------------------------------------------------------

    def _exit_idle(self) -> None:
        means: list[float] = []
        sigmas: list[float] = []
        extreme: list[int] = []
        noisy: list[int] = []
        samples = 0
        for index, stats in enumerate(self._stats):
            means.append(stats.mean)
            sigmas.append(stats.sigma)
            samples = max(samples, stats.count)
            if abs(abs(stats.mean) - 1.0) <= REST_EXTREME_TOLERANCE:
                extreme.append(index)
            if stats.sigma * DEADZONE_SIGMA > MIN_MEASURED_DEADZONE:
                noisy.append(index)
        self._idle = IdleReport(
            means=tuple(means),
            sigmas=tuple(sigmas),
            resting_at_extreme=tuple(extreme),
            noisy=tuple(noisy),
            samples=samples,
        )
        self._check_combined_pedal_hint()

    def _exit_assign_steer(self) -> None:
        self._finish_assign(Control.STEER)

    def _exit_assign_throttle(self) -> None:
        self._finish_assign(Control.THROTTLE)

    def _exit_assign_brake(self) -> None:
        self._finish_assign(Control.BRAKE)
        self._check_combined_pedal_conflict()

    def _exit_ranges(self) -> None:
        """Turn the sweep into calibration endpoints.

        The 2 % inward margin matters more than it looks. A range captured from
        an enthusiastic sweep is wider than the range the driver reaches while
        actually driving, and without the margin full lock becomes something you
        have to fight for.
        """
        margin = 0.02
        for control in Control:
            binding = self._working.refs.get(control, ControlBinding())
            index = _axis_index(binding)
            if index is None or index >= len(self._stats):
                continue
            stats = self._stats[index]
            if stats.count == 0 or stats.span < MIN_WIGGLE_TRAVEL:
                continue
            low = stats.minimum
            high = stats.maximum
            inset = (high - low) * margin
            low += inset
            high -= inset
            if control is Control.STEER:
                rest = self._idle.means[index] if index < len(self._idle.means) else 0.0
                if not (low < rest < high):
                    rest = (low + high) * 0.5
                self._working.lo[control] = low
                self._working.rest[control] = rest
                self._working.hi[control] = high
            else:
                rest = self._working.rest.get(control, stats.first)
                # Whichever end is farther from rest is "fully pressed".
                full = low if abs(low - rest) > abs(high - rest) else high
                self._working.rest[control] = rest
                self._working.lo[control] = min(rest, full)
                self._working.hi[control] = full

    def _exit_deadzone(self) -> None:
        """Convert measured rest noise into a deadzone in normalized units."""
        for control in Control:
            binding = self._working.refs.get(control, ControlBinding())
            index = _axis_index(binding)
            if index is None or index >= len(self._stats):
                # A digital control has no noise to measure and needs no
                # deadzone; a button is either pressed or it is not.
                self._working.deadzone[control] = 0.0
                continue
            stats = self._stats[index]
            if stats.count < 2:
                continue
            raw_band = stats.sigma * DEADZONE_SIGMA
            drift = abs(stats.mean - self._working.rest.get(control, stats.mean))
            span = self._normalizing_span(control)
            if span <= 0.0:
                continue
            deadzone = (raw_band + drift) / span
            if deadzone < MIN_MEASURED_DEADZONE:
                deadzone = MIN_MEASURED_DEADZONE
            elif deadzone > MAX_MEASURED_DEADZONE:
                deadzone = MAX_MEASURED_DEADZONE
                self._warn(
                    f"{control.value} is very noisy at rest; a {deadzone * 100:.0f}% "
                    "deadzone is the most this will apply. Check the wiring or the "
                    "potentiometer."
                )
            self._working.deadzone[control] = deadzone

    def _exit_saturation(self) -> None:
        """Set saturation from how far the driver actually pushes.

        Measured against the *calibrated* value, not the raw one, so it composes
        with whatever the ranges step decided.
        """
        for control in Control:
            binding = self._working.refs.get(control, ControlBinding())
            index = _axis_index(binding)
            if index is None or index >= len(self._stats):
                continue
            stats = self._stats[index]
            if stats.count == 0:
                continue
            cal = self._calibration_for(control)
            if cal is None:
                continue
            if control is Control.STEER:
                reached = max(
                    abs(_apply_cal(cal, stats.minimum)),
                    abs(_apply_cal(cal, stats.maximum)),
                )
            else:
                reached = max(
                    _apply_cal(cal, stats.minimum), _apply_cal(cal, stats.maximum)
                )
            if reached <= 0.0:
                continue
            saturation = 1.0 - reached
            if saturation < 0.0:
                saturation = 0.0
            elif saturation > 0.30:
                # Anything past 30 % means the range capture was wrong, not that
                # the driver is gentle. Refusing to encode it here keeps a bad
                # measurement from silently becoming a hair-trigger pedal.
                self._warn(
                    f"{control.value} only reached {reached * 100:.0f}% of its "
                    "calibrated travel; re-run the full-travel step."
                )
                saturation = 0.30
            self._working.saturation[control] = saturation

    def _exit_verify(self) -> None:
        self._recheck_verify()

    # ------------------------------------------------------------------
    # Wizard-driven setters
    # ------------------------------------------------------------------

    def set_inverted(self, control: Control, inverted: bool) -> None:
        self._working.invert[control] = inverted
        self._notify()

    def toggle_inverted(self, control: Control) -> None:
        self.set_inverted(control, not self._working.invert.get(control, False))

    def set_deadzone(self, control: Control, value: float) -> None:
        self._working.deadzone[control] = _clamp(value, 0.0, MAX_MEASURED_DEADZONE)
        self._notify()

    def set_saturation(self, control: Control, value: float) -> None:
        self._working.saturation[control] = _clamp(value, 0.0, 0.30)
        self._notify()

    def set_curve(self, control: Control, spec: CurveSpec) -> None:
        self._working.curve[control] = Curve(spec)
        self._notify()

    def set_binding(self, control: Control, binding: ControlBinding) -> None:
        """Manual override for the rare device auto-assignment cannot read."""
        self._working.refs[control] = binding
        self._notify()

    def set_action(self, action: Action, ref: InputRef) -> None:
        self._working.actions[action] = ref
        self._notify()

    def clear_action(self, action: Action) -> None:
        self._working.actions.pop(action, None)
        self._notify()

    def skip_binding(self) -> None:
        """Leave the current action unbound and move to the next."""
        target = self.binding_target
        if target is not None:
            self._working.actions.pop(target, None)
            self._binding_index += 1
            self._binding_armed = False
            self._notify()

    def set_rotation_lock(self, degrees: float, device_rotation_deg: float = 0.0) -> None:
        """Use `degrees` of wheel travel for full lock.

        Expressed as a pre-gain rather than as its own stage: using 240 degrees
        of a 900-degree wheel is arithmetically identical to multiplying the
        normalized steering by 3.75 and clamping, and one multiply in the
        calibration step is cheaper and easier to reason about than another
        stage in the chain.
        """
        profile = self._profile
        full = device_rotation_deg
        if full <= 0.0 and profile is not None:
            full = profile.device_rotation_deg
        if full <= 0.0 or degrees <= 0.0:
            self._working.pre_gain[Control.STEER] = 1.0
        else:
            self._working.pre_gain[Control.STEER] = _clamp(full / degrees, 1.0, 10.0)
        if profile is not None:
            self._profile = replace(
                profile,
                rotation_lock_deg=max(degrees, 0.0),
                device_rotation_deg=full,
            )
        self._notify()

    def capture_rotation_lock(self) -> float:
        """Take the wheel's current position as full lock.

        Returns the resulting pre-gain. Works without knowing the wheel's
        mechanical range, which most generic wheels do not report.
        """
        cal = self._calibration_for(Control.STEER)
        index = _axis_index(self._working.refs.get(Control.STEER, ControlBinding()))
        if cal is None or index is None or index >= len(self._snapshot.axes):
            return self._working.pre_gain.get(Control.STEER, 1.0)
        # Undo the existing pre-gain so repeated captures do not compound.
        base = replace(cal, pre_gain=1.0)
        value = abs(_apply_cal(base, self._snapshot.axes[index]))
        if value < 0.10:
            self._warn("turn the wheel further before capturing the lock position")
            return self._working.pre_gain.get(Control.STEER, 1.0)
        gain = _clamp(1.0 / value, 1.0, 10.0)
        self._working.pre_gain[Control.STEER] = gain
        profile = self._profile
        if profile is not None and profile.device_rotation_deg > 0.0:
            self._profile = replace(
                profile, rotation_lock_deg=profile.device_rotation_deg / gain
            )
        self._notify()
        return gain

    def use_combined_pedal_split(self) -> None:
        """Apply the workaround for a wheel stuck in combined-pedal mode.

        Throttle takes one half of the shared axis and brake the other. It is a
        genuine loss -- the two can never be applied together, which rules out
        left-foot braking and makes trail braking impossible -- so the warning
        stays visible afterwards.
        """
        throttle = self._assign.get(Control.THROTTLE)
        brake = self._assign.get(Control.BRAKE)
        if throttle is None or brake is None:
            return
        index = _axis_index_of_ref(throttle.ref)
        if index is None or index != _axis_index_of_ref(brake.ref):
            return
        throttle_positive = throttle.extreme >= throttle.rest
        self._working.refs[Control.THROTTLE] = ControlBinding(
            InputRef(
                RefKind.AXIS_POSITIVE if throttle_positive else RefKind.AXIS_NEGATIVE,
                index,
            )
        )
        self._working.refs[Control.BRAKE] = ControlBinding(
            InputRef(
                RefKind.AXIS_NEGATIVE if throttle_positive else RefKind.AXIS_POSITIVE,
                index,
            )
        )
        for control in (Control.THROTTLE, Control.BRAKE):
            self._working.rest[control] = 0.0
            self._working.lo[control] = 0.0
            self._working.hi[control] = 1.0
            self._working.invert[control] = False
        self._warn(
            "Using one shared pedal axis, split down the middle. Throttle and "
            "brake cannot be pressed together in this mode."
        )
        self._notify()

    # ------------------------------------------------------------------
    # Live preview
    # ------------------------------------------------------------------

    def preview(self) -> tuple[float, float, float]:
        """Current steer/throttle/brake through the static part of the chain.

        Static only -- no rate limit, no filter. The wizard's bars should track
        the driver's hands exactly, and a smoothed bar during calibration would
        hide precisely the jitter the driver is being asked to look for.
        """
        snapshot = self._snapshot
        mapping = self._build_mapping()
        values: list[float] = []
        for control in Control:
            binding = mapping.binding(control)
            raw = resolve_binding(
                binding, snapshot.axes, snapshot.buttons, snapshot.hats, 0
            )
            cfg = self._axis_config(control)
            values.append(static_map(cfg, raw) if cfg is not None else 0.0)
        return (values[0], values[1], values[2])

    def raw_axes(self) -> tuple[float, ...]:
        """Unprocessed axis values, for the wizard's diagnostic panel."""
        return self._snapshot.axes

    # ------------------------------------------------------------------
    # Building the result
    # ------------------------------------------------------------------

    def _build_mapping(self) -> InputMap:
        working = self._working
        actions = tuple(
            ActionBinding(action, ref)
            for action, ref in working.actions.items()
            if ref.is_bound
        )
        return InputMap(
            steer=working.refs.get(Control.STEER, ControlBinding()),
            throttle=working.refs.get(Control.THROTTLE, ControlBinding()),
            brake=working.refs.get(Control.BRAKE, ControlBinding()),
            actions=actions,
        )

    def _calibration_for(self, control: Control) -> AxisCalibration | None:
        working = self._working
        rest = working.rest.get(control)
        hi = working.hi.get(control)
        if rest is None or hi is None:
            return None
        try:
            if control is Control.STEER:
                lo = working.lo.get(control, -1.0)
                return AxisCalibration.steering(
                    lo,
                    rest,
                    hi,
                    invert=working.invert.get(control, False),
                    pre_gain=working.pre_gain.get(control, 1.0),
                )
            return AxisCalibration.pedal(
                rest,
                hi,
                invert=working.invert.get(control, False),
                pre_gain=working.pre_gain.get(control, 1.0),
            )
        except ChainConfigError:
            return None

    def _axis_config(self, control: Control) -> AxisChainConfig | None:
        cal = self._calibration_for(control)
        if cal is None:
            return None
        base = None
        if self._profile is not None:
            base = self._profile.chain.axis(control)
        try:
            return AxisChainConfig(
                calibration=cal,
                deadzone=self._working.deadzone.get(control, 0.0),
                saturation=self._working.saturation.get(control, 0.0),
                curve=self._working.curve.get(
                    control, base.curve if base is not None else Curve()
                ),
                rate_rise=base.rate_rise if base is not None else 8.0,
                rate_fall=base.rate_fall if base is not None else 16.0,
                smoothing=base.smoothing if base is not None else NO_SMOOTHING,
            )
        except ChainConfigError:
            # A half-finished measurement can produce an invalid combination --
            # a deadzone wider than the surviving travel, say. Falling back to
            # the preset for that one axis keeps the wizard usable instead of
            # dead-ending on a validation error.
            return base

    def _build_chain(self) -> ChainConfig:
        """Assemble a valid ChainConfig, falling back per-axis if a measurement
        did not work out. Never returns something the chain would reject."""
        base = self._profile.chain if self._profile is not None else None
        if base is None:
            base = default_chain_config()
        parts: dict[Control, AxisChainConfig] = {}
        for control in Control:
            built = self._axis_config(control)
            if built is None:
                built = base.axis(control)
                self._warn(
                    f"{control.value} calibration is incomplete; keeping the "
                    "preset values for it."
                )
            parts[control] = built
        return ChainConfig(
            steer=parts[Control.STEER],
            throttle=parts[Control.THROTTLE],
            brake=parts[Control.BRAKE],
            speed_sensitive_steering=base.speed_sensitive_steering,
            brake_cuts_throttle=base.brake_cuts_throttle,
            disconnect_brake=base.disconnect_brake,
        )

    def _build_profile(self) -> DeviceProfile:
        device = self._device
        base = self._profile
        if base is None:
            base = preset_for_device(
                guid=device.guid if device else "",
                name=device.name if device else "",
                num_axes=device.num_axes if device else 0,
                num_buttons=device.num_buttons if device else 0,
                num_hats=device.num_hats if device else 0,
            )
        mapping = self._build_mapping()
        kind = base.kind
        if mapping.throttle.is_digital or mapping.brake.is_digital:
            kind = DeviceKind.WHEEL_DIGITAL
        return replace(
            base,
            mapping=mapping,
            chain=self._build_chain(),
            kind=kind,
            updated=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _reset_stats(self) -> None:
        count = len(self._stats)
        if count == 0 and self._device is not None:
            count = self._device.num_axes
        self._stats = [_AxisStats() for _ in range(count)]
        self._stats_started = self._clock()
        self._prev_buttons = self._snapshot.buttons

    def _accumulate(self, snapshot: DeviceSnapshot, now: float) -> None:
        axes = snapshot.axes
        stats = self._stats
        if len(stats) < len(axes):
            stats.extend(_AxisStats() for _ in range(len(axes) - len(stats)))
        for index, value in enumerate(axes):
            stats[index].add(value, now)

    def _track_buttons(self, snapshot: DeviceSnapshot, now: float) -> None:
        """Remember which buttons were pressed during a wiggle, and when.

        This is what makes digital pedals work: if no axis moved but a button
        went down, that button *is* the pedal.
        """
        pressed = (snapshot.buttons & ~self._prev_buttons) | snapshot.pressed_edges
        self._prev_buttons = snapshot.buttons
        if not pressed:
            return
        control = _ASSIGN_CONTROL.get(self._step)
        if control is None:
            return
        result = self._assign.get(control)
        if result is None:
            result = AssignResult(control=control)
            self._assign[control] = result
        index = (pressed & -pressed).bit_length() - 1
        if not result.ref.is_bound or result.ref.kind is not RefKind.BUTTON:
            result.ref = button_ref(index)
            result.digital = True
            result.confident = True
            result.rest = 0.0
            result.extreme = 1.0
            result.detail = "digital"
        elif control is Control.STEER and not result.negative_ref.is_bound:
            # Second distinct button during the steering wiggle: the driver was
            # asked to go left first, so the first button is left.
            if index != result.ref.index:
                result.negative_ref = result.ref
                result.ref = button_ref(index)
                result.detail = "two buttons"

    def _finish_assign(self, control: Control) -> None:
        result = self._assign.get(control)
        if result is None:
            result = AssignResult(control=control)
            self._assign[control] = result

        best_index = -1
        best_span = MIN_WIGGLE_TRAVEL
        for index, stats in enumerate(self._stats):
            if stats.count == 0:
                continue
            if stats.span > best_span:
                best_span = stats.span
                best_index = index

        if best_index < 0:
            if result.ref.is_bound:
                self._apply_assignment(control, result)
                return
            self._warn(
                f"nothing moved for {control.value}. Bind it by hand, or run the "
                "step again and move it further."
            )
            return

        stats = self._stats[best_index]
        idle_mean = (
            self._idle.means[best_index]
            if best_index < len(self._idle.means)
            else stats.first
        )
        result.ref = axis_ref(best_index)
        result.negative_ref = InputRef()
        result.digital = False
        result.minimum = stats.minimum
        result.maximum = stats.maximum
        result.confident = best_span >= MIN_WIGGLE_TRAVEL * 1.5

        if control is Control.STEER:
            # The driver was asked for left first. Whichever extreme arrived
            # first is therefore the left one, and if that was the maximum the
            # axis reads backwards.
            result.rest = idle_mean
            result.invert = stats.t_max < stats.t_min
            result.extreme = stats.minimum if result.invert else stats.maximum
            result.detail = "inverted" if result.invert else ""
        else:
            result.rest = idle_mean
            far_low = abs(stats.minimum - idle_mean)
            far_high = abs(stats.maximum - idle_mean)
            result.extreme = stats.minimum if far_low > far_high else stats.maximum
            result.invert = False
            result.detail = f"rest {result.rest:+.2f} full {result.extreme:+.2f}"

        self._apply_assignment(control, result)

    def _apply_assignment(self, control: Control, result: AssignResult) -> None:
        working = self._working
        working.refs[control] = ControlBinding(result.ref, result.negative_ref)
        working.invert[control] = result.invert
        if result.digital:
            working.rest[control] = 0.0
            working.lo[control] = -1.0 if control is Control.STEER else 0.0
            working.hi[control] = 1.0
            working.deadzone[control] = 0.0
            working.saturation[control] = 0.0
            return
        if control is Control.STEER:
            working.rest[control] = result.rest
            working.lo[control] = min(result.minimum, result.rest - MIN_WIGGLE_TRAVEL)
            working.hi[control] = max(result.maximum, result.rest + MIN_WIGGLE_TRAVEL)
        else:
            working.rest[control] = result.rest
            working.hi[control] = result.extreme
            working.lo[control] = min(result.rest, result.extreme)

    def _check_combined_pedal_hint(self) -> None:
        """Warn early if the idle picture looks like combined pedals."""
        device = self._device
        if device is None or device.num_axes < 2:
            return
        if self._idle.resting_at_extreme:
            return
        lowered = device.name.lower()
        if not any(h in lowered for h in ("wheel", "racing", "logitech", "g2", "force")):
            return
        self._warn(
            "No pedal axis is resting at one end of its travel. Many wheels ship "
            "in 'combined pedals' mode, where throttle and brake share a single "
            "axis. If the next steps pick the same control twice, that is why -- "
            "check your wheel's manual for the button combination that separates "
            "them."
        )

    def _check_combined_pedal_conflict(self) -> None:
        throttle = self._assign.get(Control.THROTTLE)
        brake = self._assign.get(Control.BRAKE)
        if throttle is None or brake is None:
            return
        t_index = _axis_index_of_ref(throttle.ref)
        b_index = _axis_index_of_ref(brake.ref)
        if t_index is None or t_index != b_index:
            return
        opposite = (throttle.extreme - throttle.rest) * (brake.extreme - brake.rest) < 0
        if opposite:
            self._warn(
                "Throttle and brake are on the same axis, moving in opposite "
                "directions: your wheel is in combined-pedal mode. Switch it out "
                "of that mode for independent pedals, or continue and the axis "
                "will be split down the middle."
            )
        else:
            self._warn(
                "Throttle and brake both moved the same control the same way. "
                "Re-run the brake step and press only the brake, or bind it by "
                "hand."
            )

    def _range_detail(self) -> str:
        parts: list[str] = []
        for control in Control:
            index = _axis_index(self._working.refs.get(control, ControlBinding()))
            if index is None or index >= len(self._stats):
                parts.append(f"{control.value} digital")
                continue
            stats = self._stats[index]
            parts.append(f"{control.value} {stats.minimum:+.2f}..{stats.maximum:+.2f}")
        return "; ".join(parts)

    def _normalizing_span(self, control: Control) -> float:
        working = self._working
        rest = working.rest.get(control, 0.0)
        hi = working.hi.get(control, 1.0)
        lo = working.lo.get(control, -1.0)
        if control is Control.STEER:
            return min(abs(hi - rest), abs(rest - lo))
        return abs(hi - rest)

    def _recheck_verify(self) -> None:
        report = self._verify
        problems: list[str] = []
        if report.steer_left > -VERIFY_FULL:
            problems.append("full left lock not reached")
        if report.steer_right < VERIFY_FULL:
            problems.append("full right lock not reached")
        if report.throttle_max < VERIFY_FULL:
            problems.append("full throttle not reached")
        if report.brake_max < VERIFY_FULL:
            problems.append("full brake not reached")
        if not report.returns_to_neutral:
            problems.append("controls do not return to neutral")
        report.problems = tuple(problems)

    def _warn(self, text: str) -> None:
        if text not in self._warnings:
            self._warnings.append(text)
            self._notify()

    def clear_warnings(self) -> None:
        self._warnings.clear()
        self._notify()

    def _notify(self) -> None:
        callback = self.on_change
        if callback is not None:
            try:
                callback()
            except Exception:
                # The UI's repaint is not this object's problem, and a wizard
                # that dies mid-calibration loses the whole run.
                pass


# --------------------------------------------------------------------------
# Module-level tables and helpers
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class _StaticDeviceInfo:
    guid: str
    name: str
    num_axes: int
    num_buttons: int
    num_hats: int


_ASSIGN_CONTROL: dict[CalStep, Control] = {
    CalStep.ASSIGN_STEER: Control.STEER,
    CalStep.ASSIGN_THROTTLE: Control.THROTTLE,
    CalStep.ASSIGN_BRAKE: Control.BRAKE,
}

_STEP_DURATION: dict[CalStep, float] = {
    CalStep.IDLE: IDLE_CAPTURE_S,
    CalStep.ASSIGN_STEER: WIGGLE_CAPTURE_S,
    CalStep.ASSIGN_THROTTLE: WIGGLE_CAPTURE_S,
    CalStep.ASSIGN_BRAKE: WIGGLE_CAPTURE_S,
    CalStep.RANGES: RANGE_CAPTURE_S,
    CalStep.DEADZONE: DEADZONE_CAPTURE_S,
    CalStep.SATURATION: SATURATION_CAPTURE_S,
}

_FEED: dict[CalStep, Callable[[CalibrationMachine, DeviceSnapshot, float], None]] = {
    CalStep.DETECT: CalibrationMachine._feed_detect,
    CalStep.IDLE: CalibrationMachine._feed_idle,
    CalStep.ASSIGN_STEER: CalibrationMachine._feed_wiggle,
    CalStep.ASSIGN_THROTTLE: CalibrationMachine._feed_wiggle,
    CalStep.ASSIGN_BRAKE: CalibrationMachine._feed_wiggle,
    CalStep.RANGES: CalibrationMachine._feed_ranges,
    CalStep.DEADZONE: CalibrationMachine._feed_deadzone,
    CalStep.SATURATION: CalibrationMachine._feed_saturation,
    CalStep.BINDINGS: CalibrationMachine._feed_bindings,
    CalStep.VERIFY: CalibrationMachine._feed_verify,
}

_EXIT: dict[CalStep, Callable[[CalibrationMachine], None]] = {
    CalStep.IDLE: CalibrationMachine._exit_idle,
    CalStep.ASSIGN_STEER: CalibrationMachine._exit_assign_steer,
    CalStep.ASSIGN_THROTTLE: CalibrationMachine._exit_assign_throttle,
    CalStep.ASSIGN_BRAKE: CalibrationMachine._exit_assign_brake,
    CalStep.RANGES: CalibrationMachine._exit_ranges,
    CalStep.DEADZONE: CalibrationMachine._exit_deadzone,
    CalStep.SATURATION: CalibrationMachine._exit_saturation,
    CalStep.VERIFY: CalibrationMachine._exit_verify,
}


def _default_binding_order() -> tuple[Action, ...]:
    """Most important first, so an impatient driver who stops halfway still has
    an E-stop bound."""
    return (
        Action.ESTOP,
        Action.ARM,
        Action.DISARM,
        Action.REVERSE,
        Action.HORN,
        Action.PIT_LIMITER,
        Action.HEADLIGHTS,
        Action.RESET_ODOM,
        Action.CLEAR_FAULTS,
        Action.MARK_LAP,
        Action.CYCLE_CAMERA,
        Action.TOGGLE_HUD,
        Action.TOGGLE_RECORD,
    )


def _axis_index(binding: ControlBinding) -> int | None:
    return _axis_index_of_ref(binding.positive)


def _axis_index_of_ref(ref: InputRef) -> int | None:
    if ref.kind in (RefKind.AXIS, RefKind.AXIS_POSITIVE, RefKind.AXIS_NEGATIVE):
        return ref.index
    return None


def _apply_cal(cal: AxisCalibration, raw: float) -> float:
    if cal.bipolar:
        return calibrate_bipolar(raw, cal)
    return calibrate_unipolar(raw, cal)


def _any_moved(snapshot: DeviceSnapshot, stats: list[_AxisStats]) -> bool:
    for index, value in enumerate(snapshot.axes):
        if index >= len(stats):
            return True
        entry = stats[index]
        if entry.count and abs(value - entry.first) > 0.10:
            return True
    return False


def _clamp(value: float, lo: float, hi: float) -> float:
    if not math.isfinite(value):
        return lo
    return lo if value < lo else hi if value > hi else value


def summarize(machine: CalibrationMachine) -> dict[str, Any]:
    """Flat summary for the wizard's final page and for the log."""
    profile = machine.profile
    out: dict[str, Any] = {
        "device": profile.name,
        "profile_id": profile.profile_id,
        "kind": profile.kind.value,
        "steering": profile.mapping.steer.describe(),
        "throttle": profile.mapping.throttle.describe(),
        "brake": profile.mapping.brake.describe(),
    }
    for control in Control:
        cfg = profile.chain.axis(control)
        out[f"{control.value}_deadzone"] = round(cfg.deadzone, 4)
        out[f"{control.value}_saturation"] = round(cfg.saturation, 4)
        out[f"{control.value}_curve"] = cfg.curve.spec.kind.value
    out["warnings"] = list(machine.warnings)
    out["conflicts"] = profile.mapping.conflicts()
    return out
