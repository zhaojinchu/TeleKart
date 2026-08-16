"""Infrastructure with no opinion about vehicles: time, buffers, scheduling.

``rt`` is deliberately not re-exported. It touches ``ctypes`` and ``/proc``, and
nothing should pay for that just by importing ``telekart.util``; import it
explicitly with ``from telekart.util import rt``.
"""

from __future__ import annotations

from .clock import (
    Clock,
    DeadlineScheduler,
    FakeClock,
    JitterSnapshot,
    JitterStats,
    RealClock,
)
from .ringbuf import FloatRingBuffer, RingBuffer

__all__ = [
    "Clock",
    "RealClock",
    "FakeClock",
    "DeadlineScheduler",
    "JitterStats",
    "JitterSnapshot",
    "RingBuffer",
    "FloatRingBuffer",
]
