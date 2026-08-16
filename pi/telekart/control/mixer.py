"""The electronic differential. Not optional, and not a refinement.

Two motors bolted to a rear axle, Ackermann steering at the front, and no
mechanical differential between them. In a turn the rear wheels are on circles
of different radius, so they *must* run at different speeds. A per-wheel PID
that is told to make them equal will succeed, and the way it succeeds is by
dragging the inside tyre sideways -- scrubbing rubber, wasting current from a
converter that has none to spare, and pushing the car wide of the line the
driver asked for.

The split falls straight out of the geometry::

    split = track_width * tan(delta) / (2 * wheelbase)

With the shipped 150 mm track and 200 mm wheelbase at the 24-degree lock, that
is +/-16.7 %: the outside wheel runs 33 % faster than the inside one. That is
not a rounding error to be absorbed by a PID, it is the answer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import VehicleConfig

#: The inside wheel never runs below this fraction of the axle target. A
#: geometry typo -- 1.5 m of track on a 200 mm wheelbase, say -- would otherwise
#: ask the inside wheel to reverse while the outside one drives forward, which
#: this chassis physically cannot do and which would sequence a direction change
#: mid-corner.
_MAX_SPLIT = 0.75


@dataclass(frozen=True, slots=True)
class WheelTargets:
    """Per-wheel speed targets in RPM. Signed: negative is reverse."""

    rpm_l: float
    rpm_r: float

    @property
    def mean(self) -> float:
        return (self.rpm_l + self.rpm_r) * 0.5

    @property
    def spread(self) -> float:
        """Difference between the wheels, positive when the left runs faster."""
        return self.rpm_l - self.rpm_r


class DifferentialMixer:
    """Axle speed and steer angle in, per-wheel targets out.

    Reads geometry from the config on every call rather than caching it, so a
    corrected wheelbase pushed from the tuning UI takes effect on the next tick.
    The cost is three attribute loads; the alternative is a mixer that quietly
    keeps using the number that was wrong.
    """

    __slots__ = ("_config",)

    def __init__(self, config: VehicleConfig) -> None:
        if config.wheelbase_m <= 0.0:
            raise ValueError(f"wheelbase_m must be positive, got {config.wheelbase_m}")
        if config.track_width_m <= 0.0:
            raise ValueError(f"track_width_m must be positive, got {config.track_width_m}")
        self._config = config

    def mix(self, target_rpm: float, steer_angle: float) -> WheelTargets:
        """Split an axle target across the two wheels.

        ``steer_angle`` is in radians and positive to the RIGHT, matching the
        wire protocol's steering sign and the servo driver. Steering right puts
        the LEFT wheel on the outside of the turn, so the left target is the
        larger one.

        The mean of the two targets always equals ``target_rpm``: the
        differential redistributes speed, it never adds or removes any.
        """
        if target_rpm != target_rpm:
            return WheelTargets(0.0, 0.0)
        if steer_angle != steer_angle:
            steer_angle = 0.0

        split = self.split_for(steer_angle)
        return WheelTargets(
            rpm_l=target_rpm * (1.0 + split),
            rpm_r=target_rpm * (1.0 - split),
        )

    def split_for(self, steer_angle: float) -> float:
        """Fractional speed split for a steer angle. Positive means left-faster."""
        config = self._config
        wheelbase = config.wheelbase_m
        if wheelbase <= 0.0:
            return 0.0
        # Clamped short of a right angle: tan() diverges there, and a steering
        # rack that reached 90 degrees would be a broken linkage, not a corner.
        if steer_angle > 1.4:
            steer_angle = 1.4
        elif steer_angle < -1.4:
            steer_angle = -1.4
        split = config.track_width_m * math.tan(steer_angle) / (2.0 * wheelbase)
        if split > _MAX_SPLIT:
            return _MAX_SPLIT
        if split < -_MAX_SPLIT:
            return -_MAX_SPLIT
        return split

    def turn_radius(self, steer_angle: float) -> float:
        """Radius of the path the rear axle centre follows, in metres.

        Positive for a left turn, negative for a right one, and ``inf`` when
        pointed straight. Not used by the control loop -- odometry integrates
        the heading rate directly -- but it is the number to print when the
        geometry is being checked against a tape measure on the floor.
        """
        if steer_angle == 0.0:
            return math.inf
        tangent = math.tan(steer_angle)
        if tangent == 0.0:
            return math.inf
        return -self._config.wheelbase_m / tangent

    def __repr__(self) -> str:
        config = self._config
        return (
            f"DifferentialMixer(track={config.track_width_m:.3f}m, "
            f"wheelbase={config.wheelbase_m:.3f}m, "
            f"max_split={self.split_for(config.steer_max_rad):+.3f})"
        )
