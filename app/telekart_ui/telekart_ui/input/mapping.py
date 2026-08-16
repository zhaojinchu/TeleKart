"""Which physical control does what.

Everything in here answers one question: given the raw arrays SDL hands us --
axes, a button bitmask, hats -- which number is steering, which is throttle,
and which button means E-stop. It is the only layer that knows an axis index
exists. Nothing above it, and no part of the UI outside the calibration wizard,
should ever mention one.

The awkward case this file is shaped around is a wheel with **digital pedals**:
two ordinary buttons where an analog pedal would be. Once a control can be
sourced from a button, a hat direction, a keyboard key, or half of a shared
axis, the mapping has to be a small tagged union rather than an int, and the
rest of the chain has to be indifferent to which one it got.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

from telekart_protocol.constants import ControlFlags


class MappingError(ValueError):
    """A binding is unusable. Raised at load/construction time only."""


class Control(enum.Enum):
    """The three things the car actually takes from the driver."""

    STEER = "steer"
    THROTTLE = "throttle"
    BRAKE = "brake"


class RefKind(enum.Enum):
    """Where one number comes from."""

    NONE = "none"
    #: A full analog axis, used across its whole travel.
    AXIS = "axis"
    #: The positive half of an axis, zero elsewhere. Exists for wheels stuck in
    #: "combined pedals" mode, where throttle and brake share one axis: throttle
    #: pushes it one way, brake the other. Splitting it is a usable fallback
    #: while the driver works out how to switch their wheel out of that mode.
    AXIS_POSITIVE = "axis+"
    AXIS_NEGATIVE = "axis-"
    #: A button. Pressed reads 1.0. Digital pedals live here.
    BUTTON = "button"
    #: One direction of a hat/D-pad.
    HAT = "hat"
    #: A keyboard key, identified by its Qt key code (see the KEY_* constants).
    KEY = "key"


#: Qt.Key values for the keyboard preset. Hard-coded rather than imported so
#: this module -- and every test that touches it -- stays free of Qt. These are
#: public, documented, stable Qt constants; they have not changed since Qt 4.
KEY_LEFT = 0x01000012
KEY_UP = 0x01000013
KEY_RIGHT = 0x01000014
KEY_DOWN = 0x01000015
KEY_RETURN = 0x01000004
KEY_ENTER = 0x01000005
KEY_ESCAPE = 0x01000000
KEY_BACKSPACE = 0x01000003
KEY_SPACE = 0x20
KEY_A = 0x41
KEY_D = 0x44
KEY_F = 0x46
KEY_O = 0x4F
KEY_R = 0x52
KEY_S = 0x53
KEY_W = 0x57

KEY_NAMES: dict[int, str] = {
    KEY_LEFT: "Left",
    KEY_UP: "Up",
    KEY_RIGHT: "Right",
    KEY_DOWN: "Down",
    KEY_RETURN: "Return",
    KEY_ENTER: "Enter",
    KEY_ESCAPE: "Esc",
    KEY_BACKSPACE: "Backspace",
    KEY_SPACE: "Space",
    KEY_A: "A",
    KEY_D: "D",
    KEY_F: "F",
    KEY_O: "O",
    KEY_R: "R",
    KEY_S: "S",
    KEY_W: "W",
}


class Action(enum.IntEnum):
    """Discrete things a driver can ask for.

    Values are bit positions, not opaque ids: the input thread hands the TX
    thread a plain `int` bitmask, which costs nothing to build and nothing to
    copy across the LatestBox. An IntFlag would read better and allocate an enum
    instance on every `|`.
    """

    ESTOP = 0
    ARM = 1
    DISARM = 2
    CLEAR_FAULTS = 3
    RESET_ODOM = 4
    REVERSE = 5

    # Gone, and why -- so nobody adds them back looking for a feature:
    #   PIT_LIMITER   a track-day duty cap; this station has no track mode.
    #   HEADLIGHTS    no hardware on the car, and no firmware handler either.
    #   HORN          likewise.
    #   MARK_LAP      lap timing is gone.
    #   CYCLE_CAMERA  there is one camera.
    #   TOGGLE_RECORD session recording is gone.
    #   TOGGLE_HUD    a window shortcut, not a car control; it never belonged
    #                 on a device that also steers.


class ActionMode(enum.Enum):
    #: Active only while the control is held.
    HOLD = "hold"
    #: Each press flips a latched state.
    TOGGLE = "toggle"
    #: Each press fires once; there is no "on" state.
    EDGE = "edge"


#: How each action behaves. REVERSE is a toggle because holding a paddle through
#: a three-point turn is miserable. ESTOP is held *and* latched by the state
#: machine on the car -- the app just keeps asserting the bit for as long as the
#: button is down, and the car decides when it is allowed to come back out.
ACTION_MODES: dict[Action, ActionMode] = {
    Action.ESTOP: ActionMode.HOLD,
    Action.ARM: ActionMode.EDGE,
    Action.DISARM: ActionMode.EDGE,
    Action.CLEAR_FAULTS: ActionMode.EDGE,
    Action.RESET_ODOM: ActionMode.EDGE,
    Action.REVERSE: ActionMode.TOGGLE,
}

#: Actions that map straight onto a control-packet flag. The rest are
#: session-channel messages (ARM/DISARM/CLEAR_FAULTS).
ACTION_FLAGS: dict[Action, ControlFlags] = {
    Action.ESTOP: ControlFlags.ESTOP,
    Action.REVERSE: ControlFlags.REVERSE_REQ,
    Action.RESET_ODOM: ControlFlags.RESET_ODOM,
}

#: Actions the session client owns.
SESSION_ACTIONS: tuple[Action, ...] = (Action.ARM, Action.DISARM, Action.CLEAR_FAULTS)

ACTION_LABELS: dict[Action, str] = {
    Action.ESTOP: "Emergency stop",
    Action.ARM: "Arm",
    Action.DISARM: "Disarm",
    Action.CLEAR_FAULTS: "Clear faults",
    Action.RESET_ODOM: "Reset odometry",
    Action.REVERSE: "Reverse",
}


def action_bit(action: Action) -> int:
    return 1 << int(action)


@dataclass(frozen=True, slots=True)
class InputRef:
    """One physical source of one number."""

    kind: RefKind = RefKind.NONE
    #: Axis index, button index, hat index, or Qt key code.
    index: int = 0
    #: HAT only: 0 selects the hat's x component, 1 its y.
    hat_axis: int = 0
    #: HAT only: which direction of that component counts as pressed.
    sign: int = 1

    def __post_init__(self) -> None:
        if not isinstance(self.kind, RefKind):
            raise MappingError(f"ref kind must be a RefKind, got {self.kind!r}")
        if self.kind is not RefKind.NONE and self.index < 0:
            raise MappingError(f"{self.kind.value} index must not be negative")
        if self.kind is RefKind.HAT:
            if self.hat_axis not in (0, 1):
                raise MappingError("hat_axis must be 0 (x) or 1 (y)")
            if self.sign not in (-1, 1):
                raise MappingError("hat sign must be -1 or +1")

    @property
    def is_bound(self) -> bool:
        return self.kind is not RefKind.NONE

    @property
    def is_analog(self) -> bool:
        return self.kind in (RefKind.AXIS, RefKind.AXIS_POSITIVE, RefKind.AXIS_NEGATIVE)

    @property
    def is_digital(self) -> bool:
        return self.kind in (RefKind.BUTTON, RefKind.HAT, RefKind.KEY)

    def describe(self) -> str:
        """Driver-facing text. Never shows a bare axis number without context."""
        if self.kind is RefKind.NONE:
            return "unbound"
        if self.kind is RefKind.AXIS:
            return f"axis {self.index}"
        if self.kind is RefKind.AXIS_POSITIVE:
            return f"axis {self.index} (push)"
        if self.kind is RefKind.AXIS_NEGATIVE:
            return f"axis {self.index} (pull)"
        if self.kind is RefKind.BUTTON:
            return f"button {self.index}"
        if self.kind is RefKind.HAT:
            axis = "x" if self.hat_axis == 0 else "y"
            return f"hat {self.index} {axis}{'+' if self.sign > 0 else '-'}"
        return KEY_NAMES.get(self.index, f"key 0x{self.index:X}")

NO_REF = InputRef()


def axis(index: int) -> InputRef:
    return InputRef(RefKind.AXIS, index)


def button(index: int) -> InputRef:
    return InputRef(RefKind.BUTTON, index)


def key(code: int) -> InputRef:
    return InputRef(RefKind.KEY, code)


def hat(index: int, hat_axis: int, sign: int) -> InputRef:
    return InputRef(RefKind.HAT, index, hat_axis, sign)


@dataclass(frozen=True, slots=True)
class ControlBinding:
    """Where one of the three driver controls comes from.

    `negative` is used only for a bipolar control built from two digital inputs
    -- steering from a D-pad or from the arrow keys. For everything else it stays
    unbound and the resolved value is just `positive`.
    """

    positive: InputRef = NO_REF
    negative: InputRef = NO_REF

    @property
    def is_bound(self) -> bool:
        return self.positive.is_bound or self.negative.is_bound

    @property
    def is_digital(self) -> bool:
        """True when the driver gets no proportional control from this binding.

        The chain needs to know: with a digital pedal, the rate limiter *is* the
        pedal travel, and a response curve applied to a signal that is only ever
        0 or 1 does precisely nothing.
        """
        if self.positive.is_analog or self.negative.is_analog:
            return False
        return self.positive.is_digital or self.negative.is_digital

    def describe(self) -> str:
        if self.negative.is_bound:
            return f"{self.negative.describe()} / {self.positive.describe()}"
        return self.positive.describe()

@dataclass(frozen=True, slots=True)
class ActionBinding:
    action: Action
    ref: InputRef

@dataclass(frozen=True, slots=True)
class InputMap:
    """The complete device-to-meaning translation for one device."""

    steer: ControlBinding = ControlBinding()
    throttle: ControlBinding = ControlBinding()
    brake: ControlBinding = ControlBinding()
    actions: tuple[ActionBinding, ...] = ()

    def binding(self, control: Control) -> ControlBinding:
        if control is Control.STEER:
            return self.steer
        if control is Control.THROTTLE:
            return self.throttle
        return self.brake

    def with_binding(self, control: Control, binding: ControlBinding) -> "InputMap":
        if control is Control.STEER:
            return InputMap(binding, self.throttle, self.brake, self.actions)
        if control is Control.THROTTLE:
            return InputMap(self.steer, binding, self.brake, self.actions)
        return InputMap(self.steer, self.throttle, binding, self.actions)

    def action_ref(self, action: Action) -> InputRef:
        for entry in self.actions:
            if entry.action is action:
                return entry.ref
        return NO_REF

    def with_action(self, action: Action, ref: InputRef) -> "InputMap":
        kept = tuple(e for e in self.actions if e.action is not action)
        if ref.is_bound:
            kept = kept + (ActionBinding(action, ref),)
        return InputMap(self.steer, self.throttle, self.brake, kept)

    def conflicts(self) -> list[str]:
        """Human-readable list of physical inputs bound to more than one thing.

        Not an error: binding the same button to HORN and MARK_LAP is a
        legitimate, if odd, choice. It is worth showing in the wizard because it
        is almost never intentional.
        """
        seen: dict[tuple[Any, ...], list[str]] = {}
        for control in Control:
            binding = self.binding(control)
            for ref in (binding.positive, binding.negative):
                if ref.is_bound:
                    seen.setdefault(_ref_key(ref), []).append(control.value)
        for entry in self.actions:
            if entry.ref.is_bound:
                seen.setdefault(_ref_key(entry.ref), []).append(
                    ACTION_LABELS[entry.action]
                )
        out: list[str] = []
        for ref_key, users in seen.items():
            if len(users) > 1:
                kind = RefKind(ref_key[0])
                probe = InputRef(kind, int(ref_key[1]), int(ref_key[2]), int(ref_key[3]))
                out.append(f"{probe.describe()} is bound to {', '.join(users)}")
        out.sort()
        return out

# --------------------------------------------------------------------------
# Resolution -- called from the 250 Hz input thread
# --------------------------------------------------------------------------


def resolve_ref(
    ref: InputRef,
    axes: Sequence[float],
    buttons: int,
    hats: Sequence[tuple[int, int]],
    keys: int,
) -> float:
    """One physical input to a number in 0..1 (digital) or -1..+1 (full axis).

    Out-of-range indices return 0.0. They happen for real -- a profile saved
    against a wheel with three axes gets loaded while a two-axis gamepad is
    plugged in -- and this is a per-tick path, so it must not raise.
    """
    kind = ref.kind
    if kind is RefKind.NONE:
        return 0.0
    index = ref.index
    if kind is RefKind.AXIS:
        return axes[index] if index < len(axes) else 0.0
    if kind is RefKind.AXIS_POSITIVE:
        if index >= len(axes):
            return 0.0
        value = axes[index]
        return value if value > 0.0 else 0.0
    if kind is RefKind.AXIS_NEGATIVE:
        if index >= len(axes):
            return 0.0
        value = axes[index]
        return -value if value < 0.0 else 0.0
    if kind is RefKind.BUTTON:
        return 1.0 if (buttons >> index) & 1 else 0.0
    if kind is RefKind.HAT:
        if index >= len(hats):
            return 0.0
        component = hats[index][ref.hat_axis]
        return 1.0 if component * ref.sign > 0 else 0.0
    # KEY refs are stored as Qt key codes, which are far too large and sparse to
    # index a bitmask. `KeyboardSource` rewrites them to compact bit positions
    # before resolving, and `keys` is that compact mask -- so this branch only
    # ever sees a small index. The mask below is belt and braces.
    return 1.0 if (keys >> (index & 63)) & 1 else 0.0


def resolve_binding(
    binding: ControlBinding,
    axes: Sequence[float],
    buttons: int,
    hats: Sequence[tuple[int, int]],
    keys: int,
) -> float:
    positive = resolve_ref(binding.positive, axes, buttons, hats, keys)
    if binding.negative.is_bound:
        return positive - resolve_ref(binding.negative, axes, buttons, hats, keys)
    return positive


def resolve_actions(
    actions: Iterable[ActionBinding],
    axes: Sequence[float],
    buttons: int,
    hats: Sequence[tuple[int, int]],
    keys: int,
) -> int:
    """Bitmask of actions whose physical input is currently held."""
    held = 0
    for entry in actions:
        if resolve_ref(entry.ref, axes, buttons, hats, keys) >= 0.5:
            held |= 1 << int(entry.action)
    return held


class ActionState:
    """Turns per-poll held/pressed bitmasks into flags and one-shot events.

    Lives on the TX side of the LatestBox, not the input side: toggles must flip
    exactly once per press, and the input thread may produce several samples the
    TX thread never sees. Feeding this from the samples that were actually sent
    is what keeps the latched state and the wire in agreement.
    """

    __slots__ = ("_toggles", "_pending", "_held", "_prev_pressed")

    def __init__(self) -> None:
        self._toggles = 0
        self._pending = 0
        self._held = 0
        self._prev_pressed = 0

    def update(self, held: int, pressed: int) -> None:
        """`held` is the current mask; `pressed` is the rising edges since the
        last poll, accumulated by the source."""
        self._held = held
        for action, mode in ACTION_MODES.items():
            bit = 1 << int(action)
            if not (pressed & bit):
                continue
            if mode is ActionMode.TOGGLE:
                self._toggles ^= bit
            elif mode is ActionMode.EDGE:
                self._pending |= bit
        self._prev_pressed = pressed

    @property
    def toggles(self) -> int:
        return self._toggles

    def is_latched(self, action: Action) -> bool:
        return bool(self._toggles & (1 << int(action)))

    def set_latched(self, action: Action, value: bool) -> None:
        """Force a toggle, e.g. when telemetry reports the car disagrees."""
        bit = 1 << int(action)
        if value:
            self._toggles |= bit
        else:
            self._toggles &= ~bit

    def take_events(self) -> int:
        """Consume the pending edge-mode actions. Empties the pending set."""
        events = self._pending
        self._pending = 0
        return events

    def control_flags(self) -> ControlFlags:
        """The ControlFlags bits this state implies. Held actions come from the
        live mask, toggled ones from the latch."""
        flags = ControlFlags.NONE
        for action, flag in ACTION_FLAGS.items():
            bit = 1 << int(action)
            mode = ACTION_MODES[action]
            if mode is ActionMode.HOLD:
                if self._held & bit:
                    flags |= flag
            elif mode is ActionMode.TOGGLE:
                if self._toggles & bit:
                    flags |= flag
            elif self._pending & bit:
                # An edge action that maps to a flag (RESET_ODOM) rides one
                # packet. The TX thread calls take_events() after building it.
                flags |= flag
        return flags

    def reset(self) -> None:
        self._toggles = 0
        self._pending = 0
        self._held = 0
        self._prev_pressed = 0


@dataclass(slots=True)
class EdgeTracker:
    """Rising/falling edges of a bitmask across polls.

    Sources own one of these. Note what it cannot see: the cached SDL state is a
    *level*, so a button pressed and released between two cache updates leaves no
    trace here at all. That is why `SdlHub` also latches JOYBUTTONDOWN events
    into `DeviceSnapshot.pressed_edges` while it drains the event queue, and why
    `JoystickSource` merges the two. Levels alone would drop a quick tap on a
    bouncy microswitch, and a dropped tap on the E-stop button is not acceptable.
    """

    previous: int = 0
    pressed: int = field(default=0)
    released: int = field(default=0)

    def update(self, current: int) -> None:
        changed = current ^ self.previous
        self.pressed = changed & current
        self.released = changed & self.previous
        self.previous = current

    def reset(self) -> None:
        self.previous = 0
        self.pressed = 0
        self.released = 0


def _ref_key(ref: InputRef) -> tuple[Any, ...]:
    return (ref.kind.value, ref.index, ref.hat_axis, ref.sign)


def _as_int(value: object, what: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise MappingError(f"{what} must be an integer, got {type(value).__name__}")
    return value
