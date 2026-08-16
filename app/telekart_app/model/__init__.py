"""Immutable snapshots, unit conversion, and the model the UI binds to.

``AppModel`` is *not* imported eagerly. It needs PySide6, and this package is
imported by the network layer for its snapshot types -- pulling Qt in there
would defeat the headless integration tests. The PEP 562 hook below keeps
``from telekart_app.model import AppModel`` working for the UI while everyone
else pays nothing.
"""

from __future__ import annotations

from typing import Any

from . import units
from .snapshots import (
    EMPTY_INPUT,
    EMPTY_LINK,
    EMPTY_SESSION,
    EMPTY_VEHICLE,
    TELEMETRY_STALE_S,
    VIDEO_STALE_S,
    InputSnapshot,
    LinkSnapshot,
    LinkState,
    SessionSnapshot,
    VehicleSnapshot,
    differs,
)
from .units import SpeedMode, UnitFormatter

_LAZY = {"AppModel", "SnapshotSources", "VehicleController", "DEFAULT_TICK_HZ"}

__all__ = [
    "VehicleSnapshot",
    "LinkSnapshot",
    "InputSnapshot",
    "SessionSnapshot",
    "LinkState",
    "differs",
    "EMPTY_VEHICLE",
    "EMPTY_LINK",
    "EMPTY_INPUT",
    "EMPTY_SESSION",
    "TELEMETRY_STALE_S",
    "VIDEO_STALE_S",
    "units",
    "UnitFormatter",
    "SpeedMode",
    "AppModel",
    "SnapshotSources",
    "VehicleController",
    "DEFAULT_TICK_HZ",
]


def __getattr__(name: str) -> Any:
    if name in _LAZY:
        from . import app_model

        return getattr(app_model, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
