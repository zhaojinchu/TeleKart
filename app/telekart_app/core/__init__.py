"""Threading primitives and logging shared by every other subpackage.

Nothing here imports Qt, PyAV or SDL. The net stack and the headless
integration tests depend on that.
"""

from __future__ import annotations

from .latest_box import LatestBox
from .log import Throttle, configure, get_logger, set_level
from .paced_loop import (
    DEFAULT_SWITCH_INTERVAL,
    EMPTY_JITTER,
    JitterSnapshot,
    JitterStats,
    PacedLoop,
    tune_thread_switching,
)

__all__ = [
    "LatestBox",
    "PacedLoop",
    "JitterStats",
    "JitterSnapshot",
    "EMPTY_JITTER",
    "tune_thread_switching",
    "DEFAULT_SWITCH_INTERVAL",
    "get_logger",
    "configure",
    "set_level",
    "Throttle",
]
