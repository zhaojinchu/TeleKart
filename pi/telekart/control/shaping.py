"""Stateless shaping maths, plus one memoised curve.

Every function here is pure: same inputs, same output, no clock, no config, no
hidden state. That is what makes them exhaustively testable, and it is why the
tuning-sensitive parts of the control loop live in this file instead of being
scattered through it.

One rule that is easy to get wrong and expensive to debug: **a deadzone must
rescale**. Subtracting the threshold and stopping there leaves a step at the
edge of the zone -- the output jumps from 0 to whatever the raw value is minus
the threshold, and full deflection no longer reaches 1.0. Rescaling the
remaining range keeps the output continuous at the edge and still reaches full
scale at the stops. Both properties matter: the first is what stops the car
twitching as the stick leaves centre, the second is what stops the driver
finding that full lock is only 96 % of full lock.
"""

from __future__ import annotations

import math

#: Resolution of the precomputed curve tables. 256 entries with linear
#: interpolation between them is accurate to ~1e-5 for the exponents in range,
#: which is four orders of magnitude finer than the 1/1000 the wire carries.
CURVE_TABLE_SIZE = 256


def clamp(value: float, lo: float, hi: float) -> float:
    """Bound ``value`` to [lo, hi]. NaN resolves to ``lo``.

    NaN is handled explicitly because every comparison against NaN is False, so
    the obvious two-branch form passes it straight through to whatever is
    downstream -- which, in this codebase, is an H-bridge.
    """
    if value != value:
        return lo
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def lerp(a: float, b: float, t: float) -> float:
    """Linear interpolation, unclamped in ``t``."""
    return a + (b - a) * t


def deadzone(value: float, dz: float) -> float:
    """Suppress a band around zero and rescale what is left.

    ``deadzone(dz, dz)`` is 0.0 and ``deadzone(1.0, dz)`` is 1.0 for any valid
    ``dz``: continuous at the edge, full scale still reachable at the stop.
    """
    if value != value:
        return 0.0
    if dz <= 0.0:
        return value
    if dz >= 1.0:
        # A deadzone covering the whole range would divide by zero below. Refuse
        # to produce an infinity in a control path; the axis is simply dead.
        return 0.0
    magnitude = value if value >= 0.0 else -value
    if magnitude <= dz:
        return 0.0
    scaled = (magnitude - dz) / (1.0 - dz)
    if scaled > 1.0:
        scaled = 1.0
    return scaled if value >= 0.0 else -scaled


def expo(value: float, gamma: float) -> float:
    """Sign-preserving power curve. ``gamma`` of 1.0 is linear.

    Above 1.0 the response near neutral is softened, which is what makes a
    twitchy steering axis drivable without giving up full lock. Applied to the
    magnitude and the sign put back, so the curve is symmetric through zero and
    has no kink there.
    """
    if value != value:
        return 0.0
    if gamma <= 0.0 or gamma == 1.0:
        return value
    magnitude = value if value >= 0.0 else -value
    if magnitude > 1.0:
        magnitude = 1.0
    shaped = magnitude**gamma
    return shaped if value >= 0.0 else -shaped


def rate_limit(target: float, current: float, max_rate: float, dt: float) -> float:
    """Move ``current`` toward ``target`` by at most ``max_rate * dt``.

    A hard slew clamp, not a filter: below the limit the output equals the
    target exactly, with no lag at all. That distinction is the whole reason the
    firmware smooths this way and never with an EMA -- an EMA here plus an EMA
    in the desktop app is second-order lag that nobody can reason about.
    """
    if target != target:
        return current
    if max_rate <= 0.0 or dt <= 0.0:
        return current
    step = max_rate * dt
    delta = target - current
    if delta > step:
        return current + step
    if delta < -step:
        return current - step
    return target


def wrap_pi(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    if angle != angle:
        return 0.0
    wrapped = math.remainder(angle, math.tau)
    # math.remainder returns a value in [-pi, pi]; normalise the -pi edge so the
    # interval is half-open and a heading of exactly 180 degrees has one
    # representation rather than two.
    if wrapped <= -math.pi:
        wrapped += math.tau
    return wrapped


def speed_sensitive_scale(speed_frac: float, max_reduction: float) -> float:
    """Steering authority as a function of speed fraction.

    Linear from full authority at rest to ``1 - max_reduction`` at the measured
    top speed. Linear rather than something cleverer because the driver has to
    build a model of it, and a curve whose gain changes shape mid-corner is a
    curve nobody learns.
    """
    if speed_frac != speed_frac or max_reduction <= 0.0:
        return 1.0
    if speed_frac < 0.0:
        speed_frac = 0.0
    elif speed_frac > 1.0:
        speed_frac = 1.0
    if max_reduction > 1.0:
        max_reduction = 1.0
    return 1.0 - max_reduction * speed_frac


class ExpoCurve:
    """A precomputed ``expo`` table with linear interpolation.

    Memoised, not stateful: the table is a pure function of ``gamma`` and is
    rebuilt only when ``gamma`` changes, which happens when an operator moves a
    slider and never on a control tick. ``pow()`` costs about 80 ns on an A53;
    at 100 Hz across steering and throttle that is not the problem -- the
    problem is that the desktop app's input chain runs the same curve at 250 Hz
    over several axes, and this class is shared with it in spirit if not in
    code. Keeping both ends table-driven keeps them feeling identical.
    """

    __slots__ = ("_gamma", "_table", "_size")

    def __init__(self, gamma: float = 1.0, size: int = CURVE_TABLE_SIZE) -> None:
        if size < 2:
            raise ValueError(f"curve table needs at least two entries, got {size}")
        self._size = size
        self._gamma = 0.0
        self._table = [0.0] * size
        self.rebuild(gamma)

    def rebuild(self, gamma: float) -> None:
        """Recompute the table. Allocation happens here and nowhere else."""
        if gamma <= 0.0:
            gamma = 1.0
        if gamma == self._gamma:
            return
        self._gamma = gamma
        last = self._size - 1
        table = self._table
        for i in range(self._size):
            table[i] = (i / last) ** gamma

    def ensure(self, gamma: float) -> None:
        """Rebuild only if ``gamma`` moved. Cheap enough to call every tick."""
        if gamma != self._gamma:
            self.rebuild(gamma)

    def apply(self, value: float) -> float:
        """Sign-preserving lookup with linear interpolation between entries."""
        if value != value:
            return 0.0
        magnitude = value if value >= 0.0 else -value
        if magnitude >= 1.0:
            shaped = 1.0
        else:
            position = magnitude * (self._size - 1)
            index = int(position)
            fraction = position - index
            table = self._table
            low = table[index]
            shaped = low + (table[index + 1] - low) * fraction
        return shaped if value >= 0.0 else -shaped

    @property
    def gamma(self) -> float:
        return self._gamma

    def __repr__(self) -> str:
        return f"ExpoCurve(gamma={self._gamma:.3f}, size={self._size})"
