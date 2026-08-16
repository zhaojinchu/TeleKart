"""Drift-free loop pacing with jitter statistics, for the app's worker threads."""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass

#: Sorting a window is O(n log n) but happens once per statistics read (1 Hz on
#: the HUD), while `add` happens at up to 250 Hz. Trading the read for the write
#: is the right way round, and it gives exact percentiles over the window rather
#: than the drifting estimate a P-square streaming algorithm would.
DEFAULT_WINDOW = 1024


@dataclass(frozen=True, slots=True)
class JitterSnapshot:
    """Period statistics, all in seconds.

    p99 is the number that matters. A 10 ms loop with a 10.1 ms mean and a
    28 ms p99 is failing -- the mean says nothing about the frame where the
    command went out 18 ms late.
    """

    count: int
    mean: float
    p50: float
    p95: float
    p99: float
    max: float
    peak: float
    overruns: int

    @property
    def rate_hz(self) -> float:
        return 1.0 / self.mean if self.mean > 0.0 else 0.0


EMPTY_JITTER = JitterSnapshot(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)


class JitterStats:
    """Windowed period statistics with an allocation-free ``add``."""

    __slots__ = ("_buf", "_cap", "_i", "_n", "_sum", "_peak", "_lock", "_scratch")

    def __init__(self, window: int = DEFAULT_WINDOW) -> None:
        if window < 8:
            raise ValueError(f"jitter window must be at least 8 samples, got {window}")
        self._cap = window
        # Preallocated so the 100 Hz path never touches the allocator. `_scratch`
        # is the sort buffer; reusing it keeps snapshot() from allocating either.
        self._buf = [0.0] * window
        self._scratch = [0.0] * window
        self._i = 0
        self._n = 0
        self._sum = 0.0
        self._peak = 0.0
        self._lock = threading.Lock()

    def add(self, dt: float) -> None:
        with self._lock:
            i = self._i
            if self._n == self._cap:
                self._sum -= self._buf[i]
            else:
                self._n += 1
            self._buf[i] = dt
            self._sum += dt
            self._i = i + 1 if i + 1 < self._cap else 0
            if dt > self._peak:
                self._peak = dt

    def snapshot(self, overruns: int = 0) -> JitterSnapshot:
        with self._lock:
            n = self._n
            if n == 0:
                return EMPTY_JITTER
            scratch = self._scratch
            scratch[:n] = self._buf[:n]
            mean = self._sum / n
            peak = self._peak
        window = scratch[:n]
        window.sort()
        return JitterSnapshot(
            count=n,
            mean=mean,
            p50=window[_index(n, 0.50)],
            p95=window[_index(n, 0.95)],
            p99=window[_index(n, 0.99)],
            max=window[-1],
            peak=peak,
            overruns=overruns,
        )

    def reset(self) -> None:
        with self._lock:
            self._i = 0
            self._n = 0
            self._sum = 0.0
            self._peak = 0.0


def _index(n: int, quantile: float) -> int:
    """Nearest-rank index. No interpolation: with a 1024-sample window the
    difference is below the timer resolution we are measuring."""
    idx = math.ceil(quantile * n) - 1
    return 0 if idx < 0 else n - 1 if idx >= n else idx


class PacedLoop:
    """Fixed-period pacing against absolute deadlines.

    ``time.sleep(period)`` accumulates every scheduling delay: at 100 Hz a
    consistent 0.4 ms of wake-up latency is 4 % slow, which after a minute is
    240 packets never sent. Deadlines are therefore computed from a fixed
    origin, never from "now", so a late wake-up is absorbed by the next
    interval instead of shifting every interval that follows.

    On an overrun the missed deadlines are *skipped*, not queued. A control
    stream wants the current command, and firing four back-to-back packets to
    catch up after a stall is exactly the wrong response -- it bursts the link
    at the moment it is least healthy.
    """

    __slots__ = ("period", "name", "_spin", "_next", "_last", "_started", "overruns", "stats")

    def __init__(
        self,
        period: float,
        *,
        name: str = "loop",
        window: int = DEFAULT_WINDOW,
        spin_margin: float = 0.0,
    ) -> None:
        if period <= 0.0:
            raise ValueError(f"{name}: period must be positive, got {period}")
        if spin_margin < 0.0 or spin_margin >= period:
            raise ValueError(
                f"{name}: spin_margin must be in [0, {period}), got {spin_margin}"
            )
        self.period = period
        self.name = name
        # Busy-wait the final `spin_margin` seconds. Off by default: macOS
        # nanosleep lands within ~1 ms, which is inside the budget for every
        # loop in this app, and a spinning thread costs a core for nothing.
        # Raise it only if a jitter snapshot says you need it.
        self._spin = spin_margin
        self._next = 0.0
        self._last = 0.0
        self._started = False
        self.overruns = 0
        self.stats = JitterStats(window)

    def start(self) -> None:
        now = time.perf_counter()
        self._next = now
        self._last = now
        self._started = True
        self.overruns = 0
        self.stats.reset()

    def wait_next(self) -> float:
        """Sleep until the next deadline. Returns the actual elapsed dt.

        Returns immediately (dt still measured) when already past the deadline.
        """
        if not self._started:
            self.start()
        self._next += self.period
        now = time.perf_counter()
        remaining = self._next - now
        if remaining > 0.0:
            sleep_for = remaining - self._spin
            if sleep_for > 0.0:
                time.sleep(sleep_for)
            if self._spin > 0.0:
                deadline = self._next
                while time.perf_counter() < deadline:
                    pass
            now = time.perf_counter()
        else:
            missed = int(-remaining // self.period) + 1
            self.overruns += missed
            self._next += missed * self.period
        dt = now - self._last
        self._last = now
        self.stats.add(dt)
        return dt

    def resync(self) -> None:
        """Re-anchor the deadline to now without counting an overrun.

        For the deliberate gap: a thread that was parked while the link was down
        has not "missed" anything, and letting it report a 30-second overrun
        would poison the statistics the HUD shows.
        """
        now = time.perf_counter()
        self._next = now
        self._last = now

    def sleep_until_next(self, event: threading.Event) -> bool:
        """Pace, but wake early if ``event`` is set. Returns ``event.is_set()``.

        Worker threads use this so a shutdown does not have to wait out a full
        period -- and so a 250 ms telemetry-idle period is still interruptible.
        """
        if not self._started:
            self.start()
        self._next += self.period
        now = time.perf_counter()
        remaining = self._next - now
        if remaining > 0.0:
            event.wait(remaining)
            now = time.perf_counter()
        else:
            missed = int(-remaining // self.period) + 1
            self.overruns += missed
            self._next += missed * self.period
        dt = now - self._last
        self._last = now
        self.stats.add(dt)
        return event.is_set()

    def snapshot(self) -> JitterSnapshot:
        return self.stats.snapshot(self.overruns)


#: A paced thread wakes from `sleep`/`Event.wait` with the GIL released and
#: must reacquire it before it can do anything. CPython only offers the GIL to
#: a waiter every `sys.getswitchinterval()` seconds -- 5 ms by default -- so a
#: CPU-bound thread (a repaint, a curve rebuild, a numpy pass) can hold a 100 Hz
#: loop off for longer than its whole period.
#:
#: Measured on this project's target machine, 100 Hz TX and 250 Hz input running
#: against two CPU-bound Python threads:
#:
#:     switch interval   control TX rate   TX p99
#:     5.0 ms (default)      55.7 Hz       67.7 ms
#:     1.0 ms                99.1 Hz       19.7 ms
#:     0.5 ms               100.0 Hz       15.0 ms
#:
#: 56 Hz with a 68 ms p99 is not a cosmetic problem: it is a third of the car's
#: 200 ms failsafe budget spent inside the app, before the network is involved.
DEFAULT_SWITCH_INTERVAL = 0.0005


def tune_thread_switching(interval: float = DEFAULT_SWITCH_INTERVAL) -> float:
    """Shorten CPython's GIL handoff interval. Returns the previous value.

    Called once, early, by whatever starts the paced threads. The cost is more
    frequent GIL handoffs -- a few percent of throughput on CPU-bound Python,
    which this app does not do in bulk -- and the benefit is the table above.
    """
    if interval <= 0.0:
        raise ValueError("switch interval must be positive")
    previous = sys.getswitchinterval()
    sys.setswitchinterval(interval)
    return previous
