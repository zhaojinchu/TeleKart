"""The immutable snapshots, the 60 Hz model, and the composition root.

``snapshots`` and ``units`` are Qt-free and importable anywhere. ``app_model``
and ``runtime`` are not, so they are resolved on first use -- which is what lets
the net stack import ``model.snapshots`` without pulling in PySide6.
"""

from __future__ import annotations

from typing import Any

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

_LAZY = {
    "AppModel": "app_model",
    "SnapshotSources": "app_model",
    "VehicleController": "app_model",
    "DEFAULT_TICK_HZ": "app_model",
    "state_is_drivable": "app_model",
    "AppRuntime": "runtime",
}


def __getattr__(name: str) -> Any:
    module = _LAZY.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    return getattr(importlib.import_module(f".{module}", __name__), name)


__all__ = [
    "InputSnapshot",
    "LinkSnapshot",
    "LinkState",
    "SessionSnapshot",
    "VehicleSnapshot",
    "EMPTY_INPUT",
    "EMPTY_LINK",
    "EMPTY_SESSION",
    "EMPTY_VEHICLE",
    "TELEMETRY_STALE_S",
    "VIDEO_STALE_S",
    "differs",
    "AppModel",
    "AppRuntime",
    "SnapshotSources",
    "VehicleController",
    "DEFAULT_TICK_HZ",
    "state_is_drivable",
]
