"""The session state machine for timed driving.

Zero Qt, by design. The director is a plain object driven by ``tick()``, so a
headless integration test can run a whole ten-lap race against the simulator in
milliseconds with no event loop, no widgets and no display. Anything that
needed a QObject here would put the race rules behind a GUI, and race rules are
exactly the part worth testing.
"""

from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field
from typing import Callable

from telekart_protocol import VehicleState

from ..core.log import get_logger
from .lap_source import LapEvent, LapSource, NullLapSource

_log = get_logger(__name__)


class RaceMode(enum.Enum):
    FREE_DRIVE = "free_drive"  # no timing, laps still counted
    PRACTICE = "practice"  # open-ended, best lap tracked
    TIMED_LAPS = "timed_laps"  # finishes after target_laps

    @classmethod
    def parse(cls, value: str) -> "RaceMode":
        try:
            return cls(value)
        except ValueError:
            return cls.FREE_DRIVE


class RaceState(enum.Enum):
    IDLE = "idle"
    #: Started, waiting for the first crossing. The first lap of a session is
    #: an out-lap: timing it from the button press would time the walk to the
    #: start line as well.
    WAITING = "waiting"
    RUNNING = "running"
    FINISHED = "finished"
    ABORTED = "aborted"


@dataclass(frozen=True, slots=True)
class Lap:
    index: int
    t_start: float
    t_end: float
    duration: float
    valid: bool = True
    source: str = ""
    detail: str = ""


@dataclass(frozen=True, slots=True)
class RaceSnapshot:
    state: RaceState = RaceState.IDLE
    mode: RaceMode = RaceMode.FREE_DRIVE
    laps: tuple[Lap, ...] = ()
    lap_count: int = 0
    target_laps: int = 0
    current_lap: int = 0
    current_lap_time: float = 0.0
    last_lap: Lap | None = None
    best_lap: Lap | None = None
    #: Current lap against the best so far, live. None until there is a best.
    delta: float | None = None
    total_time: float = 0.0
    invalid_reason: str = ""

    @property
    def remaining(self) -> int:
        return max(0, self.target_laps - self.lap_count) if self.target_laps else 0


EMPTY_RACE = RaceSnapshot()

Listener = Callable[[RaceSnapshot], None]


class RaceDirector:
    """Turns lap crossings into a session.

    Single-threaded by contract: ``tick`` and the mutating methods are called
    from one thread (the GUI thread in the app, the test thread in a test).
    The lap *source* may be threaded -- that is its problem, and why it hands
    over timestamped events rather than calling in.
    """

    def __init__(
        self,
        source: LapSource | None = None,
        *,
        mode: RaceMode = RaceMode.FREE_DRIVE,
        target_laps: int = 0,
        min_lap_time: float = 5.0,
        min_confidence: float = 0.5,
        clock: Callable[[], float] = time.perf_counter,
    ) -> None:
        if target_laps < 0:
            raise ValueError("target_laps must not be negative")
        if min_lap_time < 0.0:
            raise ValueError("min_lap_time must not be negative")
        self._source: LapSource = source if source is not None else NullLapSource()
        self._mode = mode
        self._target_laps = target_laps
        self._min_lap_time = min_lap_time
        self._min_confidence = min_confidence
        self._clock = clock

        self._state = RaceState.IDLE
        self._laps: list[Lap] = []
        self._lap_start = 0.0
        self._session_start = 0.0
        self._session_end = 0.0
        self._best: Lap | None = None
        self._invalid_reason = ""
        self._listeners: list[Listener] = []
        self._snapshot = EMPTY_RACE
        self._pending_laps: list[Lap] = []

    # -- configuration ------------------------------------------------------

    @property
    def mode(self) -> RaceMode:
        return self._mode

    @property
    def state(self) -> RaceState:
        return self._state

    @property
    def source(self) -> LapSource:
        return self._source

    def configure(
        self,
        *,
        mode: RaceMode | None = None,
        target_laps: int | None = None,
        min_lap_time: float | None = None,
    ) -> None:
        """Change the rules. Refused while running -- changing the lap target
        mid-race would silently rewrite what the driver is driving towards."""
        if self._state is RaceState.RUNNING or self._state is RaceState.WAITING:
            raise RuntimeError("cannot reconfigure a race that is under way")
        if mode is not None:
            self._mode = mode
        if target_laps is not None:
            if target_laps < 0:
                raise ValueError("target_laps must not be negative")
            self._target_laps = target_laps
        if min_lap_time is not None:
            if min_lap_time < 0.0:
                raise ValueError("min_lap_time must not be negative")
            self._min_lap_time = min_lap_time
        # Publish immediately: the lap-target readout must change the moment
        # the operator changes it, not at whatever later tick happens to run.
        self._publish(self._clock())

    def set_source(self, source: LapSource) -> None:
        if self._state in (RaceState.RUNNING, RaceState.WAITING):
            raise RuntimeError("cannot swap the lap source mid-race")
        self._source.stop()
        self._source = source

    def add_listener(self, listener: Listener) -> None:
        self._listeners.append(listener)

    def remove_listener(self, listener: Listener) -> None:
        if listener in self._listeners:
            self._listeners.remove(listener)

    # -- control ------------------------------------------------------------

    def start(self) -> None:
        now = self._clock()
        self._laps.clear()
        self._pending_laps.clear()
        self._best = None
        self._invalid_reason = ""
        self._session_start = now
        self._session_end = 0.0
        self._lap_start = 0.0
        self._state = RaceState.WAITING
        self._source.start()
        _log.info("race started: mode=%s target=%d", self._mode.value, self._target_laps)
        self._publish(now)

    def stop(self) -> None:
        """End the session normally. An unfinished lap is discarded, not timed."""
        if self._state in (RaceState.IDLE, RaceState.FINISHED, RaceState.ABORTED):
            return
        self._session_end = self._clock()
        self._state = RaceState.FINISHED
        self._source.stop()
        self._publish(self._session_end)

    def abort(self, reason: str = "") -> None:
        if self._state in (RaceState.IDLE, RaceState.ABORTED):
            return
        self._session_end = self._clock()
        self._state = RaceState.ABORTED
        self._invalid_reason = reason
        self._source.stop()
        _log.info("race aborted: %s", reason or "no reason given")
        self._publish(self._session_end)

    def reset(self) -> None:
        self._source.stop()
        self._state = RaceState.IDLE
        self._laps.clear()
        self._pending_laps.clear()
        self._best = None
        self._lap_start = 0.0
        self._session_start = 0.0
        self._session_end = 0.0
        self._invalid_reason = ""
        self._publish(self._clock())

    def invalidate_current(self, reason: str) -> None:
        """Mark the lap in progress as not counting.

        Called when the car E-stops, spins, or leaves the track. Kept as an
        explicit method rather than inferred from telemetry so the rule stays
        visible: the director does not decide what a valid lap is, the caller
        does.
        """
        if self._state is RaceState.RUNNING:
            self._invalid_reason = reason

    def note_vehicle_state(self, state: VehicleState) -> None:
        """Feed the car's reported state in. E-stop invalidates the lap.

        Uses the state the *car* reported, never a local assumption -- the app
        never decides what the vehicle is doing.
        """
        if state in (VehicleState.ESTOP, VehicleState.FAULT, VehicleState.FAILSAFE):
            self.invalidate_current(state.name.lower())

    # -- the tick -----------------------------------------------------------

    def tick(self, now: float | None = None) -> RaceSnapshot:
        """Drain the lap source and advance. Safe to call at any rate."""
        moment = now if now is not None else self._clock()
        if self._state in (RaceState.WAITING, RaceState.RUNNING):
            for event in self._source.poll():
                self._handle(event)
        self._publish(moment)
        return self._snapshot

    def _handle(self, event: LapEvent) -> None:
        if event.confidence < self._min_confidence:
            _log.debug(
                "ignoring low-confidence lap event (%.2f from %s)",
                event.confidence,
                event.source,
            )
            return

        if self._state is RaceState.WAITING:
            # First crossing arms the clock; the out-lap is not timed.
            self._lap_start = event.t
            self._state = RaceState.RUNNING
            self._invalid_reason = ""
            return

        duration = event.t - self._lap_start
        if duration < self._min_lap_time:
            # A double-tapped button, or a marker seen twice in one pass. No
            # real lap on this car is this short, so rejecting is safe and
            # accepting would corrupt the best-lap column permanently.
            _log.debug("ignoring %.3fs lap: below the %.1fs floor", duration, self._min_lap_time)
            return

        lap = Lap(
            index=len(self._laps) + 1,
            t_start=self._lap_start,
            t_end=event.t,
            duration=duration,
            valid=not self._invalid_reason,
            source=event.source,
            detail=self._invalid_reason,
        )
        self._laps.append(lap)
        self._pending_laps.append(lap)
        if lap.valid and (self._best is None or lap.duration < self._best.duration):
            self._best = lap
        self._lap_start = event.t
        self._invalid_reason = ""

        if self._mode is RaceMode.TIMED_LAPS and self._target_laps:
            if len(self._laps) >= self._target_laps:
                self._session_end = event.t
                self._state = RaceState.FINISHED
                self._source.stop()
                _log.info("race finished: %d laps", len(self._laps))

    # -- output -------------------------------------------------------------

    def take_new_laps(self) -> list[Lap]:
        """Laps completed since the last call, for the recorder.

        Draining rather than exposing the list is what stops the recorder from
        writing the same lap twice after a UI refresh.
        """
        laps = list(self._pending_laps)
        self._pending_laps.clear()
        return laps

    @property
    def snapshot(self) -> RaceSnapshot:
        return self._snapshot

    def _publish(self, now: float) -> None:
        current_time = 0.0
        delta: float | None = None
        if self._state is RaceState.RUNNING and self._lap_start:
            current_time = now - self._lap_start
            if self._best is not None:
                delta = current_time - self._best.duration

        if self._session_start == 0.0:
            total = 0.0
        elif self._session_end:
            total = self._session_end - self._session_start
        else:
            total = now - self._session_start

        snapshot = RaceSnapshot(
            state=self._state,
            mode=self._mode,
            laps=tuple(self._laps),
            lap_count=len(self._laps),
            target_laps=self._target_laps,
            current_lap=len(self._laps) + 1 if self._state is RaceState.RUNNING else 0,
            current_lap_time=current_time,
            last_lap=self._laps[-1] if self._laps else None,
            best_lap=self._best,
            delta=delta,
            total_time=total,
            invalid_reason=self._invalid_reason,
        )
        self._snapshot = snapshot
        for listener in self._listeners:
            try:
                listener(snapshot)
            except Exception as exc:
                _log.error("race listener raised: %s", exc)


@dataclass(slots=True)
class RaceConfig:
    """Serialisable form of the rules, for the settings file."""

    mode: str = RaceMode.FREE_DRIVE.value
    target_laps: int = 5
    min_lap_time: float = 5.0
    lap_source: str = "manual"
    extras: dict[str, object] = field(default_factory=dict)
