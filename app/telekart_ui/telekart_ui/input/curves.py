"""Response curves and their lookup tables.

A curve is described analytically (`CurveSpec`) and evaluated from a 256-entry
table (`Curve`). The split exists because the two uses have opposite
requirements: a curve is *written* once, as a shape you can reason about, and
*read* 750 times a second by the input thread, which wants an indexed lookup
with no transcendental call and no allocation.

256 entries is not arbitrary. Every index the hot path produces is in 0..255,
which is inside CPython's small-integer cache, so `int(x * 255)` hands back a
pre-existing object rather than allocating one. A larger table would defeat that
for no measurable accuracy gain; the piecewise-linear error is already an order
of magnitude below the 1/1000 quantization the wire imposes.

**Two kinds, linear and expo.** The previous station also had an s-curve and a
five-point custom curve, because it had a drag-to-edit curve editor to feed. With
no editor there is nothing to author a custom shape with, and the defaults below
are the shapes that were actually driven.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

#: Table length. See the module note about the small-int cache before changing it.
LUT_SIZE = 256
_LUT_LAST = LUT_SIZE - 1
_LUT_LAST_F = float(_LUT_LAST)

#: Bounds are wider than anything the defaults use, so a hand-edited config with
#: an adventurous gamma is not rejected for being one step past today's taste.
GAMMA_MIN = 0.1
GAMMA_MAX = 8.0


class CurveKind(enum.Enum):
    LINEAR = "linear"
    EXPO = "expo"


class CurveError(ValueError):
    """A curve description is unusable. Raised at construction, never in a loop."""


@dataclass(frozen=True, slots=True)
class CurveSpec:
    """What the curve *is*, independent of how it gets evaluated.

    Both kinds satisfy f(0) == 0 and f(1) == 1 exactly. That is a hard
    requirement, not a nicety: the driver must always be able to command zero and
    to command full scale, and a curve that quietly capped throttle at 0.98 would
    be invisible on the HUD and infuriating on track.
    """

    kind: CurveKind = CurveKind.LINEAR
    #: EXPO only. 1.0 is linear; above 1.0 gives finer control near zero, which
    #: is the direction you want on a throttle. Below 1.0 is legal but coarse
    #: near zero, and the LUT approximates it poorly near the origin (see
    #: `lut_error_bound`).
    gamma: float = 1.0

    def __post_init__(self) -> None:
        if not isinstance(self.kind, CurveKind):
            raise CurveError(f"curve kind must be a CurveKind, got {self.kind!r}")
        if self.kind is CurveKind.EXPO:
            if not math.isfinite(self.gamma):
                raise CurveError("expo gamma must be finite")
            if not (GAMMA_MIN <= self.gamma <= GAMMA_MAX):
                raise CurveError(
                    f"expo gamma {self.gamma} outside [{GAMMA_MIN}, {GAMMA_MAX}]"
                )

    @classmethod
    def linear(cls) -> "CurveSpec":
        return cls(CurveKind.LINEAR)

    @classmethod
    def expo(cls, gamma: float) -> "CurveSpec":
        return cls(CurveKind.EXPO, gamma=gamma)


def evaluate(spec: CurveSpec, x: float) -> float:
    """Analytic curve value for x in 0..1. Cold path only -- the hot path uses a LUT."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    if spec.kind is CurveKind.LINEAR:
        return x
    return x**spec.gamma


def build_lut(spec: CurveSpec) -> tuple[float, ...]:
    """Sample `spec` into the table the hot path reads.

    A tuple, deliberately, and not `array('d')`: indexing a tuple returns an
    existing float object, while indexing an array constructs a new one on every
    read. At 250 Hz across three axes that is 750 pointless allocations a second.
    """
    return tuple(evaluate(spec, i / _LUT_LAST_F) for i in range(LUT_SIZE))


def lut_error_bound(spec: CurveSpec) -> float:
    """Worst-case |LUT - analytic| over 0..1, measured rather than estimated.

    Kept because it is the evidence for the module's central claim. **Expo with
    gamma below 1** has infinite slope at the origin, so no evenly-spaced table
    follows it there: error reaches about 1e-2 in the first few thousandths of
    travel and drops to 1e-5 beyond it. Everything else lands around 1e-5, well
    under the 1/1000 step the control packet quantizes to -- which is the
    resolution that actually reaches the car.
    """
    lut = build_lut(spec)
    worst = 0.0
    # Probe between the nodes, where piecewise-linear interpolation is worst.
    for i in range(_LUT_LAST):
        for frac in (0.25, 0.5, 0.75):
            x = (i + frac) / _LUT_LAST_F
            approx = lut[i] + (lut[i + 1] - lut[i]) * frac
            err = abs(approx - evaluate(spec, x))
            if err > worst:
                worst = err
    return worst


class Curve:
    """A `CurveSpec` plus its table. Immutable; share one across threads freely."""

    __slots__ = ("_spec", "_lut")

    def __init__(self, spec: CurveSpec | None = None) -> None:
        self._spec = spec if spec is not None else CurveSpec()
        self._lut = build_lut(self._spec)

    @property
    def spec(self) -> CurveSpec:
        return self._spec

    @property
    def lut(self) -> tuple[float, ...]:
        return self._lut

    def apply_unipolar(self, x: float) -> float:
        """Curve a 0..1 value. Out-of-range inputs clamp; this never raises."""
        if x <= 0.0:
            return 0.0
        if x >= 1.0:
            return 1.0
        pos = x * _LUT_LAST_F
        i = int(pos)
        lut = self._lut
        low = lut[i]
        return low + (lut[i + 1] - low) * (pos - i)

    def apply(self, x: float) -> float:
        """Curve a -1..+1 value, preserving sign.

        Sign-preserving rather than curving the shifted range, because a steering
        curve must stay symmetric about centre -- an asymmetric one pulls the car
        toward one side at every stick deflection.
        """
        if x < 0.0:
            return -self.apply_unipolar(-x)
        return self.apply_unipolar(x)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Curve) and other._spec == self._spec

    def __hash__(self) -> int:
        return hash(self._spec)

    def __repr__(self) -> str:
        return f"Curve({self._spec!r})"


#: Shared instances. Building a LUT costs 256 evaluations, so the fixed profiles
#: reference these rather than each building their own copy.
LINEAR = Curve(CurveSpec.linear())
DEFAULT_STEER_CURVE = Curve(CurveSpec.expo(1.3))
DEFAULT_THROTTLE_CURVE = Curve(CurveSpec.expo(1.8))
DEFAULT_BRAKE_CURVE = Curve(CurveSpec.expo(1.4))

__all__ = [
    "LUT_SIZE",
    "GAMMA_MIN",
    "GAMMA_MAX",
    "CurveKind",
    "CurveError",
    "CurveSpec",
    "Curve",
    "evaluate",
    "build_lut",
    "lut_error_bound",
    "LINEAR",
    "DEFAULT_STEER_CURVE",
    "DEFAULT_THROTTLE_CURVE",
    "DEFAULT_BRAKE_CURVE",
]
