"""Track geometry and a pure-pursuit driver.

Its job is to remove the human from the loop. Reference laps, telemetry for HUD
work, video with genuine optical flow, a soak test that runs overnight -- none
of those should need somebody holding a wheel, and none of them are reproducible
if somebody is.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from telekart_protocol import ControlFlags

from .physics import SimCommand, clamp

#: Where `--track oval` looks. tools/telekart_sim/tracks/, next to the package.
TRACKS_DIR = Path(__file__).resolve().parent.parent / "tracks"

#: The virtual operator holds ARM_INTENT continuously, exactly as the desktop
#: app's heartbeat does, so the car's arming logic sees the same traffic either
#: way and no code path exists that only autodrive exercises.
ARM_REQUEST_FLAGS = ControlFlags.ARM_INTENT


class TrackError(ValueError):
    """A track file is missing, malformed, or geometrically unusable."""


@dataclass(slots=True)
class Track:
    """A closed centreline with a width, in metres, in the world frame."""

    name: str
    width_m: float
    closed: bool
    points: list[tuple[float, float]]
    #: Cumulative arclength at each point; `s[-1]` is the total length.
    s: list[float]
    start_x: float
    start_y: float
    start_heading: float

    @property
    def length(self) -> float:
        return self.s[-1]

    def bounds(self) -> tuple[float, float, float, float]:
        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        margin = self.width_m * 2.0
        return (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)

    def nearest_index(self, x: float, y: float, hint: int = 0) -> int:
        """Index of the closest centreline point.

        Searches a window around `hint` rather than the whole polyline: it is
        O(1) per tick, and on a figure-8 it also stops the driver latching onto
        the other branch at the crossing, which a global search does every time.
        """
        count = len(self.points)
        if count == 0:
            raise TrackError(f"track {self.name!r} has no points")
        span = min(count, 96)
        best = hint % count
        best_d2 = float("inf")
        for offset in range(-span // 4, span):
            idx = (hint + offset) % count
            px, py = self.points[idx]
            dx = px - x
            dy = py - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best = idx
        return best

    def advance(self, index: int, distance: float) -> tuple[float, float]:
        """The point `distance` metres further along from `index`."""
        target = self.s[index] + distance
        if self.closed:
            target %= self.length
        elif target >= self.length:
            return self.points[-1]

        lo, hi = 0, len(self.s) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if self.s[mid] < target:
                lo = mid + 1
            else:
                hi = mid
        idx = max(1, lo)
        s0 = self.s[idx - 1]
        s1 = self.s[idx]
        t = 0.0 if s1 <= s0 else (target - s0) / (s1 - s0)
        x0, y0 = self.points[idx - 1]
        x1, y1 = self.points[idx]
        return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)

    def curvature(self, index: int) -> float:
        """Menger curvature through the neighbouring samples, 1/m."""
        count = len(self.points)
        if count < 3:
            return 0.0
        step = max(1, count // 180)
        ax, ay = self.points[(index - step) % count]
        bx, by = self.points[index % count]
        cx, cy = self.points[(index + step) % count]
        area2 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        d_ab = math.hypot(bx - ax, by - ay)
        d_bc = math.hypot(cx - bx, cy - by)
        d_ca = math.hypot(ax - cx, ay - cy)
        denominator = d_ab * d_bc * d_ca
        if denominator < 1e-9:
            return 0.0
        return abs(2.0 * area2 / denominator)

    @classmethod
    def load(cls, spec: str) -> Track:
        """Load by bare name (`oval`) or by path."""
        path = Path(spec)
        if not path.suffix:
            path = TRACKS_DIR / f"{spec}.json"
        if not path.is_file():
            available = sorted(p.stem for p in TRACKS_DIR.glob("*.json"))
            raise TrackError(
                f"no track {spec!r} at {path}; available: {', '.join(available) or 'none'}"
            )
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise TrackError(f"cannot read track {path}: {exc}") from exc
        return cls.from_dict(raw, fallback_name=path.stem)

    @classmethod
    def from_dict(cls, raw: object, fallback_name: str = "track") -> Track:
        if not isinstance(raw, dict):
            raise TrackError("track file must contain a JSON object")

        points_raw = raw.get("centerline_m")
        if not isinstance(points_raw, list) or len(points_raw) < 3:
            raise TrackError(
                "track needs a 'centerline_m' list of at least 3 [x, y] pairs"
            )
        points: list[tuple[float, float]] = []
        for item in points_raw:
            if (
                not isinstance(item, (list, tuple))
                or len(item) != 2
                or not all(
                    isinstance(v, (int, float)) and not isinstance(v, bool)
                    for v in item
                )
            ):
                raise TrackError(
                    f"centerline entry {item!r} is not an [x, y] pair of numbers"
                )
            points.append((float(item[0]), float(item[1])))

        width = raw.get("width_m", 0.9)
        if (
            not isinstance(width, (int, float))
            or isinstance(width, bool)
            or width <= 0.0
        ):
            raise TrackError(f"width_m must be a positive number, got {width!r}")

        closed = bool(raw.get("closed", True))
        if closed and points[0] != points[-1]:
            points.append(points[0])

        s = [0.0]
        for i in range(1, len(points)):
            step = math.hypot(
                points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]
            )
            if step <= 0.0:
                step = 1e-6  # keep s strictly increasing so the bisect stays valid
            s.append(s[-1] + step)
        if s[-1] < 1.0:
            raise TrackError(
                "track is shorter than a metre; check the units (metres, not cm)"
            )

        start = raw.get("start")
        if isinstance(start, dict):
            start_x = float(start.get("x", points[0][0]))
            start_y = float(start.get("y", points[0][1]))
            start_heading = math.radians(float(start.get("heading_deg", 0.0)))
        else:
            start_x, start_y = points[0]
            start_heading = math.atan2(points[1][1] - start_y, points[1][0] - start_x)

        name = raw.get("name")
        return cls(
            name=name if isinstance(name, str) else fallback_name,
            width_m=float(width),
            closed=closed,
            points=points,
            s=s,
            start_x=start_x,
            start_y=start_y,
            start_heading=start_heading,
        )


class PurePursuitDriver:
    """Geometric path tracker: aim at a point on the centreline ahead of you.

    Pure pursuit rather than a heading PID because the lookahead distance is a
    single physically meaningful knob -- short and it weaves, long and it cuts
    corners -- and because it degrades gracefully when the pose it is given is
    wrong, which matters here since it is driven from ODOMETRY, drift and all.
    """

    __slots__ = (
        "_arm_hold_s",
        "_index",
        "_laps",
        "_lateral_accel",
        "_lookahead_base",
        "_lookahead_gain",
        "_prev_s",
        "_steer_max",
        "_track",
        "_v_max",
        "_wheelbase",
        "cross_track_error",
        "target_speed",
    )

    def __init__(
        self,
        track: Track,
        *,
        wheelbase_m: float,
        steer_max_rad: float,
        v_max: float,
        lookahead_base_m: float = 0.40,
        lookahead_gain_s: float = 0.90,
        lateral_accel_max: float = 0.80,
    ) -> None:
        if wheelbase_m <= 0.0 or steer_max_rad <= 0.0:
            raise ValueError("wheelbase and steering lock must be positive")
        if v_max <= 0.0:
            raise ValueError(
                "autodrive needs a positive v_max; calibration produced none"
            )
        self._track = track
        self._wheelbase = wheelbase_m
        self._steer_max = steer_max_rad
        self._v_max = v_max
        self._lookahead_base = lookahead_base_m
        self._lookahead_gain = lookahead_gain_s
        self._lateral_accel = lateral_accel_max
        self._index = 0
        self._laps = 0
        self._prev_s = 0.0
        self._arm_hold_s = 0.0
        self.cross_track_error = 0.0
        self.target_speed = 0.0

    @property
    def laps(self) -> int:
        return self._laps

    def reset(self) -> None:
        self._index = 0
        self._laps = 0
        self._prev_s = 0.0
        self._arm_hold_s = 0.0

    def update(
        self, x: float, y: float, heading: float, speed: float, dt: float, armed: bool
    ) -> SimCommand:
        """One tick of virtual operator.

        `armed` is passed in rather than assumed because the car will not arm
        until the throttle has sat at neutral for `arm_neutral_ms` -- a virtual
        operator that stamps on the throttle immediately never gets to drive.
        """
        if not armed:
            self._arm_hold_s += dt
            return SimCommand(0.0, 0.0, 0.0, ARM_REQUEST_FLAGS)
        self._arm_hold_s = 0.0

        track = self._track
        self._index = track.nearest_index(x, y, self._index)
        near_x, near_y = track.points[self._index]
        self.cross_track_error = math.hypot(near_x - x, near_y - y)

        s_now = track.s[self._index]
        if track.closed and self._prev_s - s_now > track.length * 0.5:
            self._laps += 1
        self._prev_s = s_now

        lookahead = clamp(
            self._lookahead_base + self._lookahead_gain * abs(speed),
            self._lookahead_base,
            max(self._lookahead_base, track.length * 0.25),
        )
        goal_x, goal_y = track.advance(self._index, lookahead)

        dx = goal_x - x
        dy = goal_y - y
        # Into the body frame: x forward, y left.
        forward = dx * math.cos(heading) + dy * math.sin(heading)
        left = -dx * math.sin(heading) + dy * math.cos(heading)
        distance = math.hypot(forward, left)
        if distance < 1e-4:
            delta = 0.0
        else:
            # delta = atan(2 L sin(alpha) / Ld). sin(alpha) is `left / distance`,
            # and the sign convention here is positive steering = right, so the
            # geometric result is negated on the way out.
            delta = math.atan2(2.0 * self._wheelbase * left, distance * distance)
        steering = clamp(-delta / self._steer_max, -1.0, 1.0)

        # Slow for the corner you are about to be in, not the one you are in.
        curvature = max(
            track.curvature(self._index),
            track.curvature(
                (self._index + max(1, len(track.points) // 60)) % len(track.points)
            ),
        )
        limit = self._v_max
        if curvature > 1e-3:
            limit = min(limit, math.sqrt(self._lateral_accel / curvature))
        # And back off further when the wheel is already turned: the cornering
        # limit above assumes the line is being followed, and it is not always.
        limit *= 1.0 - 0.35 * abs(steering)
        self.target_speed = max(0.06, limit)

        error = self.target_speed - speed
        throttle = clamp(self.target_speed / self._v_max + 2.0 * error, 0.0, 1.0)
        brake = 0.0
        if error < -0.12 * self._v_max:
            throttle = 0.0
            brake = clamp(-error / self._v_max, 0.0, 1.0)

        return SimCommand(
            steering=steering, throttle=throttle, brake=brake, flags=ARM_REQUEST_FLAGS
        )
