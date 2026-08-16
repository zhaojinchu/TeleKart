"""Where lap boundaries come from.

The racing-sim layer is deferred, so this file builds the *seam* and one
implementation, not the features. Everything downstream -- the director, the
recorder, the HUD -- consumes ``LapEvent`` and knows nothing about how the
crossing was detected. Adding a transponder, an ArUco gate on the video feed,
or a GPS geofence later is then a new class here and no change anywhere else.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol


@dataclass(frozen=True, slots=True)
class LapEvent:
    """One start/finish crossing.

    ``t`` is captured at the moment of detection, not when the event is polled.
    That is what keeps lap timing accurate while the director is ticked at a
    lazy 60 Hz: a 16 ms polling interval would otherwise be 16 ms of error on
    every lap, and laps are compared to the millisecond.
    """

    t: float  # perf_counter at detection
    wall_t: float  # time.time, for the recording
    source: str
    #: 0..1. A button press is 1.0. A vision detector that half-saw a marker
    #: reports what it actually believes, and the director can be told to
    #: ignore anything below a threshold rather than guess for it.
    confidence: float = 1.0
    detail: str = ""
    meta: Mapping[str, Any] = field(default_factory=dict)


class LapSource(Protocol):
    """The interface every lap detector implements.

    Polling rather than callbacks, deliberately. The director is a plain
    object ticked from wherever the app likes -- a Qt timer, a test loop, a
    headless integration run -- and a callback-based source would drag its
    detection thread into the director's state machine, which is precisely the
    coupling that makes a race engine untestable.
    """

    @property
    def name(self) -> str: ...

    @property
    def active(self) -> bool: ...

    def start(self) -> None: ...

    def stop(self) -> None: ...

    def poll(self) -> list[LapEvent]:
        """Drain and return every event detected since the last call."""
        ...


class _QueuedLapSource:
    """Shared machinery: a bounded, thread-safe event queue."""

    __slots__ = ("_events", "_lock", "_active", "_name")

    def __init__(self, name: str, maxlen: int = 64) -> None:
        self._name = name
        self._events: deque[LapEvent] = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._active = False

    @property
    def name(self) -> str:
        return self._name

    @property
    def active(self) -> bool:
        return self._active

    def start(self) -> None:
        self._active = True

    def stop(self) -> None:
        self._active = False
        with self._lock:
            self._events.clear()

    def poll(self) -> list[LapEvent]:
        with self._lock:
            if not self._events:
                return []
            events = list(self._events)
            self._events.clear()
        return events

    def _push(self, event: LapEvent) -> None:
        if not self._active:
            return
        with self._lock:
            self._events.append(event)


class ManualLapSource(_QueuedLapSource):
    """A button, a key, or a test calling ``trigger()``.

    The reference implementation, and genuinely useful: on a hand-drawn track
    with no timing gear, a human with a thumb is the timing system. It is also
    what the whole race layer is developed against, which keeps the seam
    honest -- if the director ever needs something a button cannot provide,
    the abstraction is wrong.
    """

    def __init__(self, name: str = "manual") -> None:
        super().__init__(name)

    def trigger(self, detail: str = "") -> LapEvent | None:
        """Record a crossing *now*. Returns the event, or None when inactive."""
        if not self._active:
            return None
        event = LapEvent(
            t=time.perf_counter(),
            wall_t=time.time(),
            source=self._name,
            confidence=1.0,
            detail=detail,
        )
        self._push(event)
        return event


class NullLapSource(_QueuedLapSource):
    """Never fires. Free drive with no timing, without a None check everywhere."""

    def __init__(self) -> None:
        super().__init__("none")


def make_lap_source(kind: str) -> LapSource:
    """Build a source by name, for the settings file.

    Unknown names fall back to manual rather than raising: a settings file
    naming a detector this build does not have should cost timing, not startup.
    """
    if kind == "none":
        return NullLapSource()
    return ManualLapSource()
