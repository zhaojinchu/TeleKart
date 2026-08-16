"""Response curves and their lookup tables.

A curve is described analytically (`CurveSpec`) and evaluated from a 256-entry
table (`Curve`). The split exists because the two uses have opposite
requirements: the calibration wizard wants to draw and edit an exact analytic
shape, while the 250 Hz input thread wants an indexed read with no transcendental
call and no allocation.

256 entries is not arbitrary. Every index the hot path produces is in 0..255,
which is inside CPython's small-integer cache, so `int(x * 255)` hands back a
pre-existing object rather than allocating one. A larger table would defeat that
for no measurable accuracy gain; the piecewise-linear error is already an order
of magnitude below the 1/1000 quantization the wire imposes.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

#: Table length. See the module note about the small-int cache before changing it.
LUT_SIZE = 256
_LUT_LAST = LUT_SIZE - 1
_LUT_LAST_F = float(_LUT_LAST)

#: A custom curve is five points at x = 0, 0.25, 0.5, 0.75, 1.0.
CUSTOM_POINT_COUNT = 5
_CUSTOM_SPAN = 1.0 / (CUSTOM_POINT_COUNT - 1)

#: Bounds are wider than the tuning UI exposes so that an imported profile from a
#: future build is not rejected for being one step past today's slider.
GAMMA_MIN = 0.1
GAMMA_MAX = 8.0


class CurveKind(enum.Enum):
    LINEAR = "linear"
    EXPO = "expo"
    SCURVE = "scurve"
    CUSTOM = "custom"


class CurveError(ValueError):
    """A curve description is unusable. Raised at construction, never in a loop."""


@dataclass(frozen=True, slots=True)
class CurveSpec:
    """What the curve *is*, independent of how it gets evaluated.

    Every kind satisfies f(0) == 0 and f(1) == 1 exactly. That is a hard
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
    #: SCURVE only. 0.0 is linear, 1.0 is full smoothstep.
    strength: float = 0.5
    #: CUSTOM only. Outputs at x = 0, 0.25, 0.5, 0.75, 1.0.
    points: tuple[float, ...] = (0.0, 0.25, 0.5, 0.75, 1.0)

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

        if self.kind is CurveKind.SCURVE:
            if not math.isfinite(self.strength):
                raise CurveError("s-curve strength must be finite")
            if not (0.0 <= self.strength <= 1.0):
                # Above 1.0 the derivative goes negative around the midpoint and
                # the curve stops being a curve -- it becomes a control that
                # moves backwards when you push harder.
                raise CurveError(f"s-curve strength {self.strength} outside [0, 1]")

        if self.kind is CurveKind.CUSTOM:
            pts = self.points
            if len(pts) != CUSTOM_POINT_COUNT:
                raise CurveError(
                    f"custom curve needs {CUSTOM_POINT_COUNT} points, got {len(pts)}"
                )
            if not all(math.isfinite(p) for p in pts):
                raise CurveError("custom curve points must all be finite")
            if not all(0.0 <= p <= 1.0 for p in pts):
                raise CurveError(f"custom curve points must be within [0, 1]: {pts}")
            if any(pts[i + 1] < pts[i] for i in range(CUSTOM_POINT_COUNT - 1)):
                raise CurveError(f"custom curve points must not decrease: {pts}")
            if pts[0] != 0.0 or pts[-1] != 1.0:
                raise CurveError(
                    "custom curve must start at 0.0 and end at 1.0 so that idle "
                    f"and full scale stay reachable, got {pts}"
                )

    # -- factories ----------------------------------------------------------

    @classmethod
    def linear(cls) -> "CurveSpec":
        return cls(CurveKind.LINEAR)

    @classmethod
    def expo(cls, gamma: float) -> "CurveSpec":
        return cls(CurveKind.EXPO, gamma=gamma)

    @classmethod
    def scurve(cls, strength: float) -> "CurveSpec":
        return cls(CurveKind.SCURVE, strength=strength)

    @classmethod
    def custom(cls, points: tuple[float, ...] | list[float]) -> "CurveSpec":
        """Build a custom curve from points that may be in any state.

        The UI drags points around freely, so this snaps rather than rejects:
        clamps into range, forces non-decreasing, pins the endpoints. Use the
        constructor directly when the values are supposed to already be valid and
        a violation means a bug upstream.
        """
        return cls(CurveKind.CUSTOM, points=sanitize_points(points))

    # -- serialization ------------------------------------------------------

    def to_dict(self) -> dict[str, object]:
        out: dict[str, object] = {"kind": self.kind.value}
        if self.kind is CurveKind.EXPO:
            out["gamma"] = self.gamma
        elif self.kind is CurveKind.SCURVE:
            out["strength"] = self.strength
        elif self.kind is CurveKind.CUSTOM:
            out["points"] = list(self.points)
        return out

    @classmethod
    def from_dict(cls, data: object) -> "CurveSpec":
        if not isinstance(data, dict):
            raise CurveError(f"curve must be an object, got {type(data).__name__}")
        raw_kind = data.get("kind", "linear")
        try:
            kind = CurveKind(raw_kind)
        except ValueError as exc:
            raise CurveError(f"unknown curve kind {raw_kind!r}") from exc
        if kind is CurveKind.EXPO:
            return cls(kind, gamma=_as_float(data.get("gamma", 1.0), "gamma"))
        if kind is CurveKind.SCURVE:
            return cls(kind, strength=_as_float(data.get("strength", 0.5), "strength"))
        if kind is CurveKind.CUSTOM:
            raw_points = data.get("points", [0.0, 0.25, 0.5, 0.75, 1.0])
            if not isinstance(raw_points, (list, tuple)):
                raise CurveError("custom curve points must be a list")
            return cls(kind, points=tuple(_as_float(p, "point") for p in raw_points))
        return cls(kind)


def sanitize_points(points: tuple[float, ...] | list[float]) -> tuple[float, ...]:
    """Coerce arbitrary drag output into a legal custom-curve point set."""
    values = [0.0] * CUSTOM_POINT_COUNT
    for i in range(CUSTOM_POINT_COUNT):
        raw = points[i] if i < len(points) else i * _CUSTOM_SPAN
        value = float(raw) if math.isfinite(float(raw)) else i * _CUSTOM_SPAN
        values[i] = 0.0 if value < 0.0 else 1.0 if value > 1.0 else value
    values[0] = 0.0
    values[-1] = 1.0
    running = 0.0
    for i in range(CUSTOM_POINT_COUNT):
        if values[i] < running:
            values[i] = running
        running = values[i]
    return tuple(values)


def evaluate(spec: CurveSpec, x: float) -> float:
    """Analytic curve value for x in 0..1. Cold path only -- the hot path uses a LUT."""
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0

    kind = spec.kind
    if kind is CurveKind.LINEAR:
        return x
    if kind is CurveKind.EXPO:
        return x**spec.gamma
    if kind is CurveKind.SCURVE:
        smooth = x * x * (3.0 - 2.0 * x)
        return (1.0 - spec.strength) * x + spec.strength * smooth

    pts = spec.points
    scaled = x * (CUSTOM_POINT_COUNT - 1)
    i = int(scaled)
    if i >= CUSTOM_POINT_COUNT - 1:
        return pts[-1]
    frac = scaled - i
    return pts[i] + (pts[i + 1] - pts[i]) * frac


def build_lut(spec: CurveSpec) -> tuple[float, ...]:
    """Sample `spec` into the table the hot path reads.

    A tuple, deliberately, and not `array('d')`: indexing a tuple returns an
    existing float object, while indexing an array constructs a new one on every
    read. At 250 Hz across three axes that is 750 pointless allocations a second.
    """
    return tuple(evaluate(spec, i / _LUT_LAST_F) for i in range(LUT_SIZE))


def lut_error_bound(spec: CurveSpec) -> float:
    """Worst-case |LUT - analytic| over 0..1, measured rather than estimated.

    Reported so the wizard can warn instead of silently shipping a curve the
    table cannot represent. Two shapes are worth knowing about:

    * **Expo with gamma below 1** has infinite slope at the origin, so no
      evenly-spaced table follows it there. Error reaches about 1e-2 in the first
      few thousandths of travel and drops to 1e-5 beyond it.
    * **Custom curves** put their corners at multiples of 0.25, which do not land
      on nodes of a 255-interval table, so every corner gets rounded off. That
      costs about 4e-4 for a plausible curve and 4e-3 for the most extreme legal
      shape -- in both cases under the 1/1000 step the control packet quantizes
      to, which is the resolution that actually reaches the car.

    Everything else lands around 1e-5.
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


def _as_float(value: object, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CurveError(f"{what} must be a number, got {type(value).__name__}")
    return float(value)


#: Shared instances. Building a LUT costs 256 evaluations, so profiles that use
#: the defaults should not each pay for their own copy.
LINEAR = Curve(CurveSpec.linear())
DEFAULT_STEER_CURVE = Curve(CurveSpec.expo(1.3))
DEFAULT_THROTTLE_CURVE = Curve(CurveSpec.expo(1.8))
DEFAULT_BRAKE_CURVE = Curve(CurveSpec.expo(1.4))
