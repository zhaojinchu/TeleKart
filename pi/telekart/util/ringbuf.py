"""Fixed-capacity ring buffers that do not allocate while running.

Both types preallocate their storage and overwrite in place. That property is
the point: these sit in the 100 Hz loop and in the encoder callback path, where
a list that grows by ``append`` eventually triggers a resize, and a resize
eventually lands on the tick where the deadline was already tight.
"""

from __future__ import annotations

import math
from array import array
from typing import Generic, Iterator, Sequence, TypeVar

T = TypeVar("T")


class RingBuffer(Generic[T]):
    """Bounded FIFO of arbitrary objects; oldest entries are overwritten.

    ``append`` performs one index assignment into a preallocated list and two
    integer updates. Iteration and :meth:`snapshot` do allocate, but they are
    diagnostic operations and never run on a control tick.
    """

    __slots__ = ("_slots", "_capacity", "_write", "_count", "_total")

    def __init__(self, capacity: int) -> None:
        if capacity < 1:
            raise ValueError(f"ring capacity must be >= 1, got {capacity}")
        # A list of None of fixed length: `append` only ever assigns into it.
        self._slots: list[T | None] = [None] * capacity
        self._capacity = capacity
        self._write = 0
        self._count = 0
        self._total = 0

    # -- writing ------------------------------------------------------------

    def append(self, item: T) -> None:
        slots = self._slots
        write = self._write
        slots[write] = item
        write += 1
        self._write = 0 if write == self._capacity else write
        if self._count < self._capacity:
            self._count += 1
        self._total += 1

    def extend(self, items: Sequence[T]) -> None:
        for item in items:
            self.append(item)

    def clear(self) -> None:
        """Drop the contents. Releases references so nothing is kept alive."""
        for i in range(self._capacity):
            self._slots[i] = None
        self._write = 0
        self._count = 0

    # -- reading ------------------------------------------------------------

    def __len__(self) -> int:
        return self._count

    def __bool__(self) -> bool:
        return self._count > 0

    def __iter__(self) -> Iterator[T]:
        """Oldest first."""
        capacity = self._capacity
        start = (self._write - self._count) % capacity
        slots = self._slots
        for offset in range(self._count):
            item = slots[(start + offset) % capacity]
            # `item` cannot be None for offset < count unless a caller stored
            # None deliberately, which the type says they will not.
            yield item  # type: ignore[misc]

    def __getitem__(self, index: int) -> T:
        """Index 0 is the oldest retained entry; -1 is the newest."""
        count = self._count
        if index < 0:
            index += count
        if not 0 <= index < count:
            raise IndexError(f"ring index {index} out of range ({count} entries)")
        start = (self._write - count) % self._capacity
        return self._slots[(start + index) % self._capacity]  # type: ignore[return-value]

    def snapshot(self) -> list[T]:
        """A plain list, oldest first. Allocates; for diagnostics only."""
        return list(self)

    def latest(self) -> T | None:
        if self._count == 0:
            return None
        return self._slots[(self._write - 1) % self._capacity]

    def oldest(self) -> T | None:
        if self._count == 0:
            return None
        return self._slots[(self._write - self._count) % self._capacity]

    # -- properties ---------------------------------------------------------

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def full(self) -> bool:
        return self._count == self._capacity

    @property
    def total_appended(self) -> int:
        """Lifetime count, including entries that have been overwritten."""
        return self._total

    @property
    def dropped(self) -> int:
        """How many entries have been overwritten and lost."""
        return self._total - self._count

    def __repr__(self) -> str:
        return f"RingBuffer(capacity={self._capacity}, len={self._count}, total={self._total})"


class FloatRingBuffer:
    """Ring of doubles backed by ``array('d')`` -- no boxing, no allocation.

    Used for the numeric histories the loop keeps (RPM, RTT, duty). The
    statistics helpers scan the buffer, so call them at telemetry rate, not at
    control rate.
    """

    __slots__ = ("_data", "_capacity", "_write", "_count", "_total")

    def __init__(self, capacity: int, fill: float = 0.0) -> None:
        if capacity < 1:
            raise ValueError(f"ring capacity must be >= 1, got {capacity}")
        self._data = array("d", [fill]) * capacity
        self._capacity = capacity
        self._write = 0
        self._count = 0
        self._total = 0

    def append(self, value: float) -> None:
        data = self._data
        write = self._write
        data[write] = value
        write += 1
        self._write = 0 if write == self._capacity else write
        if self._count < self._capacity:
            self._count += 1
        self._total += 1

    def clear(self) -> None:
        self._write = 0
        self._count = 0

    def __len__(self) -> int:
        return self._count

    def __iter__(self) -> Iterator[float]:
        capacity = self._capacity
        start = (self._write - self._count) % capacity
        data = self._data
        for offset in range(self._count):
            yield data[(start + offset) % capacity]

    def __getitem__(self, index: int) -> float:
        count = self._count
        if index < 0:
            index += count
        if not 0 <= index < count:
            raise IndexError(f"ring index {index} out of range ({count} entries)")
        start = (self._write - count) % self._capacity
        return self._data[(start + index) % self._capacity]

    def snapshot(self) -> list[float]:
        return list(self)

    def latest(self) -> float:
        """Most recent value, or 0.0 when empty."""
        if self._count == 0:
            return 0.0
        return self._data[(self._write - 1) % self._capacity]

    # -- statistics ---------------------------------------------------------

    def mean(self) -> float:
        if self._count == 0:
            return 0.0
        total = 0.0
        for value in self:
            total += value
        return total / self._count

    def minimum(self) -> float:
        if self._count == 0:
            return 0.0
        return min(self)

    def maximum(self) -> float:
        if self._count == 0:
            return 0.0
        return max(self)

    def stdev(self) -> float:
        """Population standard deviation; 0.0 with fewer than two samples."""
        if self._count < 2:
            return 0.0
        mean = self.mean()
        acc = 0.0
        for value in self:
            delta = value - mean
            acc += delta * delta
        return math.sqrt(acc / self._count)

    @property
    def capacity(self) -> int:
        return self._capacity

    @property
    def full(self) -> bool:
        return self._count == self._capacity

    @property
    def total_appended(self) -> int:
        return self._total

    def __repr__(self) -> str:
        return f"FloatRingBuffer(capacity={self._capacity}, len={self._count})"
