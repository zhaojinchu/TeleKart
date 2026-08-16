"""Driver input: devices, mapping, calibration, shaping, and force feedback.

Import layering, which is deliberate and worth preserving:

    curves  <-  chain  <-  profile  <-  calibration
    mapping <-  sources <- sdl_backend
    ffb (independent)

Only `sdl_backend` imports pygame, and it is not imported eagerly here. That is
what lets the chain, the mapping, the profiles and the calibration state machine
be tested -- and reasoned about -- on a machine with no SDL, no Qt, and no wheel
plugged in.
"""

from __future__ import annotations

from typing import Any

from .chain import (
    AxisCalibration,
    AxisChainConfig,
    ChainConfig,
    ChainConfigError,
    ChainOutput,
    InputChain,
    OneEuroConfig,
    OneEuroFilter,
    default_chain_config,
    static_map,
)
from .curves import Curve, CurveKind, CurveSpec
from .ffb import FfbBackend, FfbCapabilities, FfbState, NullFfb, create_ffb
from .mapping import (
    Action,
    ActionBinding,
    ActionState,
    Control,
    ControlBinding,
    InputMap,
    InputRef,
    RefKind,
)
from .profile import DeviceKind, DeviceProfile, ProfileStore, preset_for_device
from .sources import (
    DeviceSnapshot,
    InputSource,
    JoystickSource,
    KeyboardSource,
    NullSource,
    RawSample,
    ReplaySource,
    ScriptSource,
    ScriptStep,
)

__all__ = [
    "Action",
    "ActionBinding",
    "ActionState",
    "AxisCalibration",
    "AxisChainConfig",
    "ChainConfig",
    "ChainConfigError",
    "ChainOutput",
    "Control",
    "ControlBinding",
    "Curve",
    "CurveKind",
    "CurveSpec",
    "DeviceKind",
    "DeviceProfile",
    "DeviceSnapshot",
    "FfbBackend",
    "FfbCapabilities",
    "FfbState",
    "InputChain",
    "InputMap",
    "InputRef",
    "InputSource",
    "JoystickSource",
    "KeyboardSource",
    "NullFfb",
    "NullSource",
    "OneEuroConfig",
    "OneEuroFilter",
    "ProfileStore",
    "RawSample",
    "RefKind",
    "ReplaySource",
    "ScriptSource",
    "ScriptStep",
    "create_ffb",
    "default_chain_config",
    "preset_for_device",
    "static_map",
    # Lazy, see __getattr__:
    "SdlHub",
    "SdlDevice",
    "install_qt_pump",
]

#: Names that live in `sdl_backend` and therefore need pygame. Resolved on first
#: access so that `import telekart_app.input` stays cheap and dependency-free --
#: the calibration wizard's tests, the property tests, and the simulator all
#: import this package without ever touching SDL.
_LAZY = frozenset({"SdlHub", "SdlDevice", "install_qt_pump", "describe_devices"})


def __getattr__(name: str) -> Any:
    if name in _LAZY:
        from . import sdl_backend

        return getattr(sdl_backend, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__() -> list[str]:
    return sorted(set(__all__) | _LAZY)
