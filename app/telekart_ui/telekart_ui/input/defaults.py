"""The two fixed input profiles, and how a device gets one.

The previous station had a profile store, a six-page calibration wizard, a
drag-to-edit curve editor and per-device persistence -- about 3,600 lines whose
whole output was the two configurations below. They are constants now.

That trade is deliberate and it does cost something: a wheel whose axes are not
in the Logitech order, or whose pedals do not idle at +1.0, will feel wrong and
there is no wizard to fix it. The escape hatch is ``axes`` in settings.json,
which names the axis index for each control. That covers the real case (a device
that reports its pedals in a different order) without a UI for the imaginary one
(a driver who wants a bespoke response curve).
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from ..core.log import get_logger
from .chain import AxisCalibration, ChainConfig, OneEuroConfig, default_chain_config
from .curves import LINEAR
from .mapping import (
    ACTION_LABELS,
    KEY_BACKSPACE,
    KEY_DOWN,
    KEY_F,
    KEY_LEFT,
    KEY_O,
    KEY_R,
    KEY_RETURN,
    KEY_RIGHT,
    KEY_SPACE,
    KEY_UP,
    Action,
    ActionBinding,
    ControlBinding,
    InputMap,
    axis,
    button,
    key,
)

_log = get_logger(__name__)

#: Logitech's axis order, which nearly every wheel vendor copied: steering on 0,
#: throttle on 2, brake on 5. Overridable per-control from settings.json.
WHEEL_AXES = {"steer": 0, "throttle": 2, "brake": 5}

#: Where the pedals live on a wheel that has none -- buttons, not axes.
DIGITAL_THROTTLE_BUTTON = 7
DIGITAL_BRAKE_BUTTON = 6


@dataclass(frozen=True, slots=True)
class Profile:
    """Everything the input thread needs to interpret one device."""

    name: str
    mapping: InputMap
    chain: ChainConfig
    #: True when throttle and brake are buttons rather than analog pedals. The
    #: HUD does not care, but it explains the much gentler rate limits.
    digital_pedals: bool = False


#: Wheel actions in descending order of how much it matters that they are
#: reachable. A device with too few buttons loses them from the bottom.
_WHEEL_ACTION_ORDER = (
    Action.ESTOP,
    Action.ARM,
    Action.DISARM,
    Action.CLEAR_FAULTS,
    Action.REVERSE,
    Action.RESET_ODOM,
)


def _wheel_actions(
    *, num_buttons: int, avoid: frozenset[int] = frozenset()
) -> tuple[ActionBinding, ...]:
    """Wheel button bindings, stepping around buttons that are pedals.

    ``avoid`` exists for the digital-pedal case: on that wheel two of the buttons
    *are* the pedals, and a binding that also put "reverse" on one of them would
    engage reverse every time the driver braked. Actions shift up to the next
    free button instead, deterministically.

    Actions that run past the end of the device are **dropped, not wrapped**. A
    binding to button 7 on a six-button wheel is not a binding; it is a control
    the driver can look up in the docs, press for, and never trigger. Losing
    reset-odometry on a cheap wheel is a fact worth logging; a phantom binding is
    a bug worth an afternoon.
    """
    used = set(avoid)
    out: list[ActionBinding] = []
    dropped: list[Action] = []
    index = 0
    for action in _WHEEL_ACTION_ORDER:
        while index in used:
            index += 1
        if num_buttons > 0 and index >= num_buttons:
            dropped.append(action)
            continue
        used.add(index)
        out.append(ActionBinding(action, button(index)))
        index += 1
    if dropped:
        _log.warning(
            "%d-button device: %s unbound (use the keyboard for those)",
            num_buttons,
            ", ".join(ACTION_LABELS[a].lower() for a in dropped),
        )
    return tuple(out)


def wheel_profile(
    *,
    name: str = "Wheel",
    num_axes: int = 6,
    num_buttons: int = 12,
    axes: dict[str, int] | None = None,
) -> Profile:
    """An analog wheel with pedals, or a wheel whose pedals are buttons.

    The axis count is the tiebreaker rather than the device name, because it is
    the one that catches the case this matters for: a wheel reporting one or two
    axes has no analog pedals, whatever its name says.
    """
    order = dict(WHEEL_AXES)
    if axes:
        order.update({k: int(v) for k, v in axes.items() if k in order})

    analog_pedals = num_axes >= 3 or bool(axes)
    chain = default_chain_config(digital_pedals=not analog_pedals)

    if analog_pedals:
        chain = ChainConfig(
            steer=replace(
                chain.steer,
                # pre_gain 2.5 because a 900-degree wheel reports its full
                # travel across the axis, and the car's steering is done well
                # before the driver has turned that far.
                calibration=AxisCalibration.steering(-1.0, 0.0, 1.0, pre_gain=2.5),
                deadzone=0.02,
                saturation=0.01,
            ),
            # Logitech pedals idle at +1.0 and read -1.0 fully depressed. That is
            # the measurement, encoded directly -- not an "invert" flag, which is
            # a different thing and would double up with this one.
            throttle=replace(chain.throttle, calibration=AxisCalibration.pedal(1.0, -1.0)),
            brake=replace(chain.brake, calibration=AxisCalibration.pedal(1.0, -1.0)),
            speed_sensitive_steering=chain.speed_sensitive_steering,
            # A wheel has two feet available, so left-foot braking works and the
            # app must not silently cut throttle when the brake moves.
            brake_cuts_throttle=False,
        )
        mapping = InputMap(
            steer=ControlBinding(axis(order["steer"])),
            throttle=ControlBinding(axis(order["throttle"])),
            brake=ControlBinding(axis(order["brake"])),
            actions=_wheel_actions(num_buttons=num_buttons),
        )
        return Profile(name=name, mapping=mapping, chain=chain, digital_pedals=False)

    throttle_button = DIGITAL_THROTTLE_BUTTON
    brake_button = DIGITAL_BRAKE_BUTTON
    if 0 < num_buttons <= DIGITAL_THROTTLE_BUTTON:
        # Not enough buttons for the 6/7 pair. Cheap wheels put the paddles
        # last, so use the top two and shift the actions clear of them.
        throttle_button = max(num_buttons - 1, 1)
        brake_button = max(throttle_button - 1, 0)

    chain = replace(
        chain,
        steer=replace(
            chain.steer,
            calibration=AxisCalibration.steering(-1.0, 0.0, 1.0, pre_gain=2.5),
        ),
    )
    mapping = InputMap(
        steer=ControlBinding(axis(order["steer"])),
        throttle=ControlBinding(button(throttle_button)),
        brake=ControlBinding(button(brake_button)),
        actions=_wheel_actions(
            num_buttons=num_buttons,
            avoid=frozenset({throttle_button, brake_button}),
        ),
    )
    return Profile(name=name, mapping=mapping, chain=chain, digital_pedals=True)


def keyboard_profile() -> Profile:
    """WASD and the arrow keys. Every control is digital, including steering.

    Steering gets the gentlest rate limit in the app: a key press is an
    instantaneous demand for full lock, and without a ramp the car would slam the
    servo into its stop every time. 0.45 s lock to lock is close to what the
    HS-311 manages anyway, so the limit costs nothing real.

    Only the arrows are bound. WASD is aliased onto them at the window's event
    boundary, because ``ControlBinding`` holds one ref per direction and both
    cannot be bound at once.
    """
    chain = default_chain_config(digital_pedals=True)
    chain = replace(
        chain,
        steer=replace(
            chain.steer,
            calibration=AxisCalibration.steering(-1.0, 0.0, 1.0),
            deadzone=0.0,
            saturation=0.0,
            curve=LINEAR,
            rate_rise=2.2,
            rate_fall=4.4,
            smoothing=OneEuroConfig(min_cutoff=6.0, beta=0.10),
        ),
    )
    mapping = InputMap(
        steer=ControlBinding(key(KEY_RIGHT), key(KEY_LEFT)),
        throttle=ControlBinding(key(KEY_UP)),
        brake=ControlBinding(key(KEY_DOWN)),
        actions=(
            ActionBinding(Action.ESTOP, key(KEY_SPACE)),
            ActionBinding(Action.ARM, key(KEY_RETURN)),
            ActionBinding(Action.DISARM, key(KEY_BACKSPACE)),
            ActionBinding(Action.CLEAR_FAULTS, key(KEY_F)),
            ActionBinding(Action.RESET_ODOM, key(KEY_O)),
            ActionBinding(Action.REVERSE, key(KEY_R)),
        ),
    )
    return Profile(
        name="Keyboard", mapping=mapping, chain=chain, digital_pedals=True
    )


def profile_for_device(
    *,
    name: str,
    num_axes: int,
    num_buttons: int,
    axes: dict[str, int] | None = None,
) -> Profile:
    """The profile for a device that was just plugged in.

    There is only one shape of answer: anything with an axis is treated as a
    wheel. A gamepad works -- stick on axis 0, triggers wherever the config says
    -- it just is not what this is tuned for.
    """
    return wheel_profile(
        name=name or "Wheel", num_axes=num_axes, num_buttons=num_buttons, axes=axes
    )


__all__ = [
    "WHEEL_AXES",
    "Profile",
    "keyboard_profile",
    "profile_for_device",
    "wheel_profile",
]
