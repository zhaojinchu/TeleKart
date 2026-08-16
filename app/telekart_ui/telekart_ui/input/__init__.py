"""Driver input: SDL device, keyboard, shaping chain, and the 250 Hz thread.

Import layering, which the modules are written against and which keeps the
whole package testable without a device or a display:

    curves <- chain <- defaults
    mapping <- sources <- sdl_backend
    thread depends on chain, mapping and sources

Only ``sdl_backend`` imports pygame, and it is resolved lazily below so that a
test -- or a machine with no SDL -- can still import everything else.
"""

from __future__ import annotations

from typing import Any

from .chain import (
    MAX_DEADZONE,
    MAX_SATURATION,
    NEUTRAL_OUTPUT,
    AxisCalibration,
    AxisChainConfig,
    ChainConfig,
    ChainConfigError,
    ChainOutput,
    InputChain,
    OneEuroConfig,
    default_chain_config,
)
from .curves import Curve, CurveKind, CurveSpec
from .defaults import Profile, keyboard_profile, profile_for_device, wheel_profile
from .mapping import (
    ACTION_LABELS,
    SESSION_ACTIONS,
    Action,
    ActionBinding,
    ActionMode,
    ActionState,
    Control,
    ControlBinding,
    InputMap,
    InputRef,
    RefKind,
)
from .sources import (
    NEUTRAL_SAMPLE,
    DeviceSnapshot,
    JoystickSource,
    KeyboardSource,
    NullSource,
    RawSample,
)
from .thread import DEFAULT_INPUT_HZ, InputThread, telemetry_speed_source

_LAZY = {"SdlHub", "SdlDevice", "install_qt_pump", "describe_devices"}


def __getattr__(name: str) -> Any:
    """Resolve the SDL names on first use.

    Importing ``telekart_ui.input`` must not import pygame: the chain and the
    mapping are pure, and their tests run in environments that have no SDL at
    all. Anything that actually touches a device asks for it by name and pays
    the import then.
    """
    if name in _LAZY:
        from . import sdl_backend

        return getattr(sdl_backend, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "AxisCalibration",
    "AxisChainConfig",
    "ChainConfig",
    "ChainConfigError",
    "ChainOutput",
    "InputChain",
    "OneEuroConfig",
    "default_chain_config",
    "MAX_DEADZONE",
    "MAX_SATURATION",
    "NEUTRAL_OUTPUT",
    "Curve",
    "CurveKind",
    "CurveSpec",
    "Profile",
    "keyboard_profile",
    "profile_for_device",
    "wheel_profile",
    "Action",
    "ActionBinding",
    "ActionMode",
    "ActionState",
    "ACTION_LABELS",
    "Control",
    "ControlBinding",
    "InputMap",
    "InputRef",
    "RefKind",
    "SESSION_ACTIONS",
    "DeviceSnapshot",
    "JoystickSource",
    "KeyboardSource",
    "NullSource",
    "RawSample",
    "NEUTRAL_SAMPLE",
    "InputThread",
    "DEFAULT_INPUT_HZ",
    "telemetry_speed_source",
    "SdlHub",
    "SdlDevice",
    "install_qt_pump",
    "describe_devices",
]
