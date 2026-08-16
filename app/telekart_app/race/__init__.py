"""Lap timing and the race session state machine.

The racing-sim layer proper is deferred. What exists here is the seam it will
plug into: a ``LapSource`` interface with a manual implementation, and a
director with no Qt dependency at all.
"""

from __future__ import annotations

from .director import (
    EMPTY_RACE,
    Lap,
    RaceConfig,
    RaceDirector,
    RaceMode,
    RaceSnapshot,
    RaceState,
)
from .lap_source import LapEvent, LapSource, ManualLapSource, NullLapSource, make_lap_source

__all__ = [
    "LapEvent",
    "LapSource",
    "ManualLapSource",
    "NullLapSource",
    "make_lap_source",
    "RaceDirector",
    "RaceMode",
    "RaceState",
    "RaceSnapshot",
    "RaceConfig",
    "Lap",
    "EMPTY_RACE",
]
