"""Depth-1 cross-thread handoff. The backbone of the app's threading model."""

from __future__ import annotations

import threading
from typing import Generic, TypeVar

T = TypeVar("T")


class LatestBox(Generic[T]):
    """A one-slot mailbox with a generation counter.

    Why this instead of a Qt signal per packet -- the reasoning is the whole
    justification for the module, so it is written down rather than rediscovered:

    * **Signal storm.** Telemetry arrives at 50 Hz and video at 30 fps. A
      queued cross-thread ``Signal`` emit costs an event-loop post, an
      allocation, and a dispatch *per subscriber*. Six HUD widgets subscribing
      to telemetry is 300 dispatches a second whose only effect is to schedule
      repaints that the compositor will coalesce anyway.
    * **Torn reads.** With one signal per value, a widget can be handed field A
      from packet N and field B from packet N+1 -- the speedometer and the RPM
      bar then disagree on screen. Here the consumer takes one immutable object
      per frame, so every widget in a repaint sees the same instant.
    * **Backpressure that is actually correct for this data.** A queue would
      make the consumer replay a backlog of stale commands after a stall. For
      control, telemetry and video the newest value is the *only* one worth
      having; overwriting is the right policy, not a compromise.

    Ownership rules, which the rest of the app is written against:

    * Exactly one consumer calls :meth:`take`. ``take`` is stateful -- it
      remembers the generation it last handed out -- so a second consumer
      would silently steal updates from the first.
    * Any number of readers may call :meth:`peek`, which never consumes.
      That is how the input thread reads telemetry speed for the
      speed-sensitive steering assist while ``AppModel`` still gets every
      update.
    * A ``LatestBox`` holds a reference, nothing more. Put immutable values in
      it. If the value owns a resource (a ``FrameBundle`` owning a decoded
      frame), the box overwriting it is what releases the old one, so the
      consumer must keep its own reference for as long as it is painting.

    The lock is not decoration. Value and generation must move together or a
    consumer can take a new generation with the previous value; under
    free-threaded builds the GIL is not there to make the pair atomic either.
    An uncontended ``threading.Lock`` acquire/release is tens of nanoseconds,
    which is nothing next to the packet encode it accompanies.
    """

    __slots__ = ("_cond", "_value", "_generation", "_taken", "_puts", "_dropped")

    def __init__(self, initial: T | None = None) -> None:
        self._cond = threading.Condition(threading.Lock())
        self._value: T | None = initial
        # Generation 0 means "never written". `take` compares against `_taken`,
        # which starts equal, so an initial value is visible to `peek` but is
        # not delivered as an update -- a seeded default is not news.
        self._generation = 0
        self._taken = 0
        self._puts = 0
        self._dropped = 0

    # -- producer side ------------------------------------------------------

    def put(self, value: T) -> None:
        """Overwrite the slot. Never blocks, never raises, never grows."""
        with self._cond:
            if self._generation != self._taken:
                # The consumer never saw the previous value. Counting this is
                # how a too-slow consumer becomes visible instead of merely
                # feeling laggy.
                self._dropped += 1
            self._value = value
            self._generation += 1
            self._puts += 1
            self._cond.notify_all()

    # -- consumer side ------------------------------------------------------

    def take(self) -> tuple[T, int] | None:
        """Return ``(value, generation)`` if it changed since the last take.

        ``None`` means "nothing new", which is the common case at 60 Hz for a
        50 Hz producer and must therefore be cheap.
        """
        with self._cond:
            if self._generation == self._taken:
                return None
            self._taken = self._generation
            # _value cannot be None here: _generation only advances in put().
            return self._value, self._generation  # type: ignore[return-value]

    def peek(self) -> T | None:
        """Read without consuming. Safe from any number of threads."""
        with self._cond:
            return self._value

    def peek_versioned(self) -> tuple[T | None, int]:
        """Value and generation, read atomically, without consuming.

        This is how the TX thread measures whether its command source is still
        alive without needing the producer to timestamp anything: the
        generation stops advancing the moment the input thread stalls, and the
        gap between that and now is the staleness the HUD must show.
        """
        with self._cond:
            return self._value, self._generation

    def wait(self, timeout: float | None = None) -> tuple[T, int] | None:
        """Block until a new value is available, then take it.

        Only for tests and for the headless CLI. No thread in the GUI process
        uses it: the paced threads must keep their own cadence rather than
        inherit a producer's, and the GUI thread must never block at all.
        """
        with self._cond:
            if self._generation == self._taken:
                self._cond.wait_for(lambda: self._generation != self._taken, timeout)
                if self._generation == self._taken:
                    return None
            self._taken = self._generation
            return self._value, self._generation  # type: ignore[return-value]

    # -- introspection ------------------------------------------------------

    @property
    def generation(self) -> int:
        """Number of puts since construction. Compare to detect a stalled producer."""
        with self._cond:
            return self._generation

    @property
    def puts(self) -> int:
        with self._cond:
            return self._puts

    @property
    def dropped(self) -> int:
        """Values overwritten before the consumer took them."""
        with self._cond:
            return self._dropped

    @property
    def pending(self) -> bool:
        with self._cond:
            return self._generation != self._taken

    def clear(self) -> None:
        """Drop the held value without resetting the counters.

        Used when a link goes down: a stale ``FrameBundle`` sitting in the box
        would otherwise keep a decoded frame -- and the video widget's idea of
        "current" -- alive across a reconnect.
        """
        with self._cond:
            self._value = None
            self._generation += 1
            self._puts += 1
            self._cond.notify_all()

    def reset(self) -> None:
        """Back to the as-constructed state, statistics included."""
        with self._cond:
            self._value = None
            self._generation = 0
            self._taken = 0
            self._puts = 0
            self._dropped = 0

    def __repr__(self) -> str:
        with self._cond:
            return (
                f"<LatestBox gen={self._generation} taken={self._taken} "
                f"puts={self._puts} dropped={self._dropped}>"
            )
