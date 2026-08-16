"""Time, injected rather than imported.

No module in the firmware calls ``time.monotonic()`` directly. A :class:`Clock`
is passed to every constructor that needs one, which is what lets the whole test
suite run on :class:`FakeClock`: a simulated sixty-second drive completes in
milliseconds and produces byte-identical results on every machine. Reaching for
``time.monotonic()`` inside the control path would trade that away for nothing.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Protocol, runtime_checkable


@runtime_checkable
class Clock(Protocol):
    """The only source of time in the firmware."""

    def monotonic(self) -> float:
        """Seconds since an arbitrary epoch. Never goes backwards."""
        ...

    def monotonic_us(self) -> int:
        """The same instant in integer microseconds.

        Separate from :meth:`monotonic` because the encoder path compares
        against pigpio's 32-bit microsecond ticks, and doing that in floats
        loses resolution once the process has been up for a few hours.
        """
        ...

    def sleep(self, seconds: float) -> None:
        """Block for ``seconds``. A non-positive value returns immediately."""
        ...


class RealClock:
    """``time.monotonic`` and ``time.sleep``, with no adjustments of any kind."""

    __slots__ = ()

    def monotonic(self) -> float:
        return time.monotonic()

    def monotonic_us(self) -> int:
        # time.monotonic_ns avoids the float rounding that a *1e6 conversion
        # introduces after the machine has been up for a while.
        return time.monotonic_ns() // 1000

    def sleep(self, seconds: float) -> None:
        if seconds > 0.0:
            time.sleep(seconds)

    def __repr__(self) -> str:
        return "RealClock()"


class FakeClock:
    """Virtual time under the test's control.

    ``sleep`` advances the clock instead of blocking, so a scheduler paced
    against this clock runs at full speed and still sees exactly the periods it
    asked for. ``auto_advance`` exists for the awkward case of code that polls
    ``monotonic()`` in a loop with no sleep -- leave it at zero unless a test
    actually needs it, because non-zero makes reads have side effects.
    """

    __slots__ = ("_now", "_lock", "_auto_advance", "sleep_calls", "slept")

    def __init__(self, start: float = 0.0, auto_advance: float = 0.0) -> None:
        if auto_advance < 0.0:
            raise ValueError("auto_advance must not be negative")
        self._now = float(start)
        self._lock = threading.Lock()
        self._auto_advance = float(auto_advance)
        self.sleep_calls = 0
        self.slept = 0.0

    def monotonic(self) -> float:
        if self._auto_advance:
            with self._lock:
                self._now += self._auto_advance
                return self._now
        return self._now

    def monotonic_us(self) -> int:
        return int(round(self.monotonic() * 1_000_000.0))

    def sleep(self, seconds: float) -> None:
        if seconds <= 0.0:
            return
        with self._lock:
            self._now += seconds
            self.sleep_calls += 1
            self.slept += seconds

    def advance(self, seconds: float) -> None:
        """Move virtual time forward. Negative values are rejected: a clock that
        can run backwards makes every elapsed-time assertion meaningless."""
        if seconds < 0.0:
            raise ValueError(f"cannot advance a monotonic clock by {seconds}")
        with self._lock:
            self._now += seconds

    def set(self, value: float) -> None:
        with self._lock:
            if value < self._now:
                raise ValueError("cannot move a monotonic clock backwards")
            self._now = value

    @property
    def now(self) -> float:
        return self._now

    def __repr__(self) -> str:
        return f"FakeClock(now={self._now:.6f})"


# --------------------------------------------------------------------------
# Period statistics
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class JitterSnapshot:
    """Loop period statistics, all in seconds."""

    p50: float
    p95: float
    p99: float
    max: float
    count: int
    mean: float = 0.0
    min: float = 0.0

    @property
    def p99_us(self) -> int:
        return int(self.p99 * 1_000_000.0)

    @property
    def max_us(self) -> int:
        return int(self.max * 1_000_000.0)

    def exceeds(self, budget: float) -> bool:
        """True when p99 is over budget -- the only test that matters."""
        return self.count > 0 and self.p99 > budget


class JitterStats:
    """Streaming period statistics over a fixed histogram.

    A histogram rather than a sorted sample list because ``add`` runs on every
    control tick: one multiply, one index, one increment, no allocation and no
    unbounded memory. The cost is quantisation, and 25 us buckets on a 10 ms
    period is 0.25 % -- far finer than the thing being measured.

    p99 is the number to look at. A loop whose mean is 10.0 ms and whose p99 is
    18 ms is dropping a command every hundred ticks, and the mean will never
    tell you that.
    """

    __slots__ = ("_buckets", "_bucket_s", "_n_buckets", "_count", "_sum", "_max", "_min")

    def __init__(self, bucket_s: float = 25e-6, n_buckets: int = 2048) -> None:
        if bucket_s <= 0.0:
            raise ValueError("bucket_s must be positive")
        if n_buckets < 2:
            raise ValueError("need at least two buckets")
        self._buckets = [0] * n_buckets
        self._bucket_s = bucket_s
        self._n_buckets = n_buckets
        self._count = 0
        self._sum = 0.0
        self._max = 0.0
        self._min = 0.0

    def add(self, dt: float) -> None:
        """Record one period. Values beyond the histogram's range land in the
        top bucket, but ``max`` is tracked exactly so an outlier is never lost."""
        if dt < 0.0:
            dt = 0.0
        index = int(dt / self._bucket_s)
        if index >= self._n_buckets:
            index = self._n_buckets - 1
        self._buckets[index] += 1
        if self._count == 0 or dt < self._min:
            self._min = dt
        if dt > self._max:
            self._max = dt
        self._count += 1
        self._sum += dt

    def snapshot(self) -> JitterSnapshot:
        count = self._count
        if count == 0:
            return JitterSnapshot(0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0)
        return JitterSnapshot(
            p50=self._quantile(0.50),
            p95=self._quantile(0.95),
            p99=self._quantile(0.99),
            max=self._max,
            count=count,
            mean=self._sum / count,
            min=self._min,
        )

    def reset(self) -> None:
        buckets = self._buckets
        for i in range(self._n_buckets):
            buckets[i] = 0
        self._count = 0
        self._sum = 0.0
        self._max = 0.0
        self._min = 0.0

    # -- internals ----------------------------------------------------------

    def _quantile(self, q: float) -> float:
        """Upper edge of the bucket containing the q-th percentile.

        Reporting the upper edge means the answer is never optimistic, which is
        the right bias for a latency budget.
        """
        target = q * self._count
        cumulative = 0
        buckets = self._buckets
        for index in range(self._n_buckets):
            cumulative += buckets[index]
            if cumulative >= target:
                edge = (index + 1) * self._bucket_s
                # The top bucket is unbounded; report the true maximum instead
                # of a bucket edge that would understate it.
                if index == self._n_buckets - 1:
                    return self._max
                return edge if edge < self._max else self._max
        return self._max

    @property
    def count(self) -> int:
        return self._count

    def __repr__(self) -> str:
        snap = self.snapshot()
        return (
            f"JitterStats(count={snap.count}, p50={snap.p50 * 1e3:.3f}ms, "
            f"p99={snap.p99 * 1e3:.3f}ms, max={snap.max * 1e3:.3f}ms)"
        )


# --------------------------------------------------------------------------
# Loop pacing
# --------------------------------------------------------------------------


class DeadlineScheduler:
    """Fixed-period pacing against absolute deadlines.

    The naive form -- ``sleep(period - elapsed)`` -- accumulates every scrap of
    error it makes, so a loop that is 0.3 ms late once is 0.3 ms late forever.
    Scheduling against ``start + n * period`` instead means a late tick is
    followed by a short sleep and the phase is recovered immediately.

    When a tick overruns so badly that whole deadlines have passed, those
    deadlines are *skipped* rather than run back-to-back. Catching up by running
    four iterations with no sleep would turn one hiccup into a burst that starves
    everything else on the core.
    """

    __slots__ = ("_clock", "_period", "_deadline", "_last_wake", "_started",
                 "overruns", "skipped", "stats")

    def __init__(self, clock: Clock, period: float) -> None:
        if period <= 0.0:
            raise ValueError(f"period must be positive, got {period}")
        self._clock = clock
        self._period = period
        self._deadline = 0.0
        self._last_wake = 0.0
        self._started = False
        self.overruns = 0
        self.skipped = 0
        self.stats = JitterStats()

    def start(self) -> None:
        """Anchor the schedule at 'now'. Safe to call again to re-anchor after a
        deliberate pause (arming, a calibration run) so the pause does not read
        as one enormous overrun."""
        now = self._clock.monotonic()
        self._last_wake = now
        self._deadline = now + self._period
        self._started = True

    def wait_next(self) -> float:
        """Sleep until the next deadline; return the measured period in seconds.

        The first call after :meth:`start` returns approximately ``period``.
        """
        if not self._started:
            self.start()

        now = self._clock.monotonic()
        remaining = self._deadline - now
        if remaining > 0.0:
            self._clock.sleep(remaining)
            now = self._clock.monotonic()

        dt = now - self._last_wake
        self._last_wake = now
        self.stats.add(dt)

        deadline = self._deadline + self._period
        if deadline <= now:
            # We are already past the next deadline: the body took longer than a
            # full period. Skip whole periods until the deadline is in the future.
            behind = now - deadline
            missed = int(behind / self._period) + 1
            deadline += missed * self._period
            self.overruns += 1
            self.skipped += missed
        self._deadline = deadline
        return dt

    def reset(self) -> None:
        self.overruns = 0
        self.skipped = 0
        self.stats.reset()
        self._started = False

    @property
    def period(self) -> float:
        return self._period

    @property
    def deadline(self) -> float:
        """Absolute time the current iteration must finish by."""
        return self._deadline

    def time_remaining(self) -> float:
        """Seconds left in this period. Negative means the budget is blown --
        useful for shedding optional work before the deadline rather than after."""
        return self._deadline - self._clock.monotonic()

    def __repr__(self) -> str:
        return (
            f"DeadlineScheduler(period={self._period * 1e3:.1f}ms, "
            f"overruns={self.overruns}, skipped={self.skipped})"
        )
