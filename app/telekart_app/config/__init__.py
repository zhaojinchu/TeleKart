"""Filesystem locations and persisted settings."""

from __future__ import annotations

from . import paths
from .settings import (
    DEFAULT_SCALE_FACTOR,
    SETTINGS_VERSION,
    DisplaySettings,
    LinkSettings,
    RaceSettings,
    RecordingSettings,
    Settings,
    VideoSettings,
    forget_shared_key,
    known_cars,
    load_shared_key,
    save_shared_key,
)

__all__ = [
    "paths",
    "Settings",
    "SETTINGS_VERSION",
    "DEFAULT_SCALE_FACTOR",
    "LinkSettings",
    "VideoSettings",
    "RecordingSettings",
    "DisplaySettings",
    "RaceSettings",
    "load_shared_key",
    "save_shared_key",
    "forget_shared_key",
    "known_cars",
]
