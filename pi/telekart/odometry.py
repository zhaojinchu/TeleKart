"""Dead reckoning from wheel encoders and the steering angle.

The car has two rear encoders and a steering servo with no position feedback,
so there are two independent estimates of how fast the heading is changing: the
difference between the wheels, and the bicycle model applied to the commanded
steer angle. Neither is right. Their *disagreement*, however, is free to compute
and is a genuinely good wheelspin and bad-calibration signal, which is what
``slip_index`` is.

Two details that matter more than they look:

**Midpoint integration, not Euler.** Advancing along the heading at the start of
the step and then rotating accumulates error proportional to the turn rate;
advancing along the *mid-step* heading does not, to second order. It costs one
extra addition. There is no reason to use Euler here.

**The steering lags.** An HS-311 needs about 0.19 s per 60 degrees, so at the
rates a driver actually moves the stick the wheels are meaningfully behind the
command. Feeding the commanded angle straight into the bicycle model assumes
instant steering, which reads as a car that turns in slightly early -- and the
error accumulates as heading, which is the one state with nothing to correct it.
A first-order lag with a 0.10 s time constant is a cheap and large improvement.

**Stated limitation:** with no IMU, heading drifts. Expect 5-15 % closure error
on a 5 m square. That is inherent to dead reckoning off two wheels, not a bug to
be tuned out. :meth:`BicycleOdometry.set_heading_source` exists so an MPU-6050
can be complementary-filtered in later without restructuring anything.
"""

from __future__ import annotations

import math
from typing import Protocol, runtime_checkable

from .config import VehicleConfig
from .control.shaping import wrap_pi

#: Time constant of the servo lag model, seconds. Derived from the HS-311's
#: 0.19 s per 60 degrees, which over the +/-24 degree working range is roughly
#: 0.08 s end to end; 0.10 s allows for the linkage and for the 5 V rail this
#: servo actually runs on rather than the 6 V it is quoted at.
STEER_LAG_TAU_S = 0.10

#: Slip is evaluated over a window, not per tick, and this is the window.
#:
#: The arithmetic that forces it: at 660 counts per revolution on a 65 mm wheel,
#: one count is 0.31 mm, and the encoder difference between the wheels is a
#: small integer number of counts per tick. Across a 150 mm track that single
#: count is 2.1 mrad, and at 100 Hz it lands as 0.21 rad/s of quantisation on a
#: heading rate that is itself only about 0.2 rad/s in a full-lock corner. A
#: per-tick slip index is therefore reading its own rounding error. Over a
#: quarter second the same +/-1 count sits on top of ninety, and the noise floor
#: drops by a factor of twenty-five.
SLIP_WINDOW_S = 0.25

#: Time constant of the filter applied to the windowed result, so the published
#: value moves smoothly rather than stepping once per window.
SLIP_FILTER_TAU_S = 0.20

#: Default weight given to an external heading source per update. At 100 Hz
#: this is a ~0.5 s time constant, which trusts the gyro for heading rate and
#: the magnetometer or fused estimate for the slow correction without letting
#: it yank the pose around.
DEFAULT_HEADING_GAIN = 0.02


@runtime_checkable
class HeadingSource(Protocol):
    """Anything that can offer an absolute heading in radians.

    Returns ``None`` when it has nothing to say this tick -- an IMU that has not
    finished converging, or a fused estimate whose covariance is still large.
    The odometry then simply does not correct, which is the correct behaviour
    and is why the hook is not just a float.
    """

    def heading(self) -> float | None:
        ...


class BicycleOdometry:
    """Pose, distance and slip from wheel travel and steer angle.

    Sign conventions, fixed here and consistent with the rest of the firmware:

    * ``steer_angle`` is positive to the **right**, matching the wire protocol's
      steering field and the servo driver's pulse mapping.
    * heading ``theta`` is positive **counter-clockwise**, the standard maths
      convention, so ``x += d*cos(theta)`` works without a sign flip.

    Those two together mean the heading *decreases* in a right turn, which is
    why the bicycle term carries a minus sign. Getting this wrong produces a
    pose that mirrors every corner, and it is invisible until the first time
    someone drives a square.
    """

    __slots__ = (
        "_config", "_x", "_y", "_theta", "_distance", "_slip", "_velocity",
        "_steer_lagged", "_heading_source", "_heading_gain",
        "_last_delta_theta", "_last_delta_theta_enc",
        "_slip_bike", "_slip_enc", "_slip_dt", "updates",
    )

    def __init__(self, config: VehicleConfig) -> None:
        if config.wheelbase_m <= 0.0:
            raise ValueError(f"wheelbase_m must be positive, got {config.wheelbase_m}")
        if config.track_width_m <= 0.0:
            raise ValueError(f"track_width_m must be positive, got {config.track_width_m}")
        self._config = config
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._distance = 0.0
        self._slip = 0.0
        self._velocity = 0.0
        self._steer_lagged = 0.0
        self._heading_source: HeadingSource | None = None
        self._heading_gain = DEFAULT_HEADING_GAIN
        self._last_delta_theta = 0.0
        self._last_delta_theta_enc = 0.0
        self._slip_bike = 0.0
        self._slip_enc = 0.0
        self._slip_dt = 0.0
        self.updates = 0

    # -- integration --------------------------------------------------------

    def update(
        self, d_left: float, d_right: float, steer_angle: float, dt: float
    ) -> None:
        """Integrate one tick. Distances are signed metres of wheel travel.

        Never raises: a NaN from a divide-by-zero upstream is dropped rather
        than allowed to poison the pose, because a pose that has gone to NaN
        cannot be recovered without a reset and takes the HUD with it.
        """
        if dt <= 0.0:
            return
        if d_left != d_left:
            d_left = 0.0
        if d_right != d_right:
            d_right = 0.0
        if steer_angle != steer_angle:
            steer_angle = 0.0

        # First-order servo lag. The commanded angle is what the loop asked for;
        # this is an estimate of where the road wheels have actually got to.
        alpha = dt / (STEER_LAG_TAU_S + dt)
        self._steer_lagged += (steer_angle - self._steer_lagged) * alpha
        delta = self._steer_lagged

        distance = (d_left + d_right) * 0.5
        wheelbase = self._config.wheelbase_m

        # Minus because the steer angle is right-positive and the heading is
        # counter-clockwise-positive. See the class docstring.
        delta_theta = -distance * math.tan(delta) / wheelbase

        half = self._theta + delta_theta * 0.5
        self._x += distance * math.cos(half)
        self._y += distance * math.sin(half)
        self._theta = wrap_pi(self._theta + delta_theta)

        # Path length, not displacement: an odometer that ran backwards when the
        # car reversed would be a trip meter nobody could read.
        self._distance += distance if distance >= 0.0 else -distance
        self._velocity = distance / dt

        # The other heading estimate, from the wheels alone. A right turn has
        # the left wheel travelling farther, so this is (right - left) to match
        # the counter-clockwise-positive convention.
        delta_theta_enc = (d_right - d_left) / self._config.track_width_m
        self._last_delta_theta = delta_theta
        self._last_delta_theta_enc = delta_theta_enc

        # Accumulate both estimates and compare them once per window. Dividing
        # by the window length makes the index a rate in rad/s, so it means the
        # same thing at any loop period -- a per-tick difference would read four
        # times larger at 400 Hz for identical physical slip.
        self._slip_bike += delta_theta
        self._slip_enc += delta_theta_enc
        self._slip_dt += dt
        if self._slip_dt >= SLIP_WINDOW_S:
            disagreement = (self._slip_enc - self._slip_bike) / self._slip_dt
            if disagreement < 0.0:
                disagreement = -disagreement
            alpha = self._slip_dt / (SLIP_FILTER_TAU_S + self._slip_dt)
            self._slip += (disagreement - self._slip) * alpha
            self._slip_bike = 0.0
            self._slip_enc = 0.0
            self._slip_dt = 0.0

        if self._heading_source is not None:
            self._apply_heading_source()

        self.updates += 1

    def reset(self) -> None:
        """Back to the origin, pointing along +x. Distance and slip too.

        Does not clear the steering lag state: the road wheels are wherever they
        are, and pretending they snapped to centre because the driver zeroed the
        trip meter would inject a heading error at the next tick.
        """
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._distance = 0.0
        self._slip = 0.0
        self._velocity = 0.0
        self._last_delta_theta = 0.0
        self._last_delta_theta_enc = 0.0
        self._slip_bike = 0.0
        self._slip_enc = 0.0
        self._slip_dt = 0.0
        self.updates = 0

    # -- heading source hook ------------------------------------------------

    def set_heading_source(
        self, source: HeadingSource | None, gain: float = DEFAULT_HEADING_GAIN
    ) -> None:
        """Attach an absolute heading reference, or detach with ``None``.

        Complementary rather than replacing: the encoders are trustworthy over
        one tick and drift over a minute, an IMU heading is noisy over one tick
        and stable over a minute, and blending them with a fixed gain is the
        cheapest thing that gets both properties. A gain of 1.0 would hand the
        heading to the sensor outright, which is a legitimate thing to want when
        the sensor is a fused estimate rather than a raw gyro.
        """
        if not 0.0 <= gain <= 1.0:
            raise ValueError(f"heading gain must be in [0, 1], got {gain}")
        self._heading_source = source
        self._heading_gain = gain

    def _apply_heading_source(self) -> None:
        source = self._heading_source
        if source is None:
            return
        external = source.heading()
        if external is None or external != external:
            return
        error = wrap_pi(external - self._theta)
        self._theta = wrap_pi(self._theta + self._heading_gain * error)

    # -- state --------------------------------------------------------------

    @property
    def pose(self) -> tuple[float, float, float]:
        """``(x, y, heading)`` in metres and radians."""
        return (self._x, self._y, self._theta)

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def heading(self) -> float:
        return self._theta

    @property
    def distance(self) -> float:
        """Path length travelled, in metres. Always increases."""
        return self._distance

    @property
    def velocity(self) -> float:
        """Signed ground speed from the last tick, m/s."""
        return self._velocity

    @property
    def slip_index(self) -> float:
        """Filtered disagreement between the two heading estimates, rad/s.

        Near zero on a car whose geometry is right and whose wheels are gripping.
        Rises with wheelspin, with a mis-measured track width, and with a
        steering trim that is off -- which is the point: it is one number that
        says "the odometry is currently lying to you".
        """
        return self._slip

    @property
    def steer_angle_lagged(self) -> float:
        """Estimated actual road-wheel angle after the servo lag model."""
        return self._steer_lagged

    @property
    def turn_rate(self) -> float:
        """Last tick's bicycle-model heading change, radians."""
        return self._last_delta_theta

    @property
    def turn_rate_encoders(self) -> float:
        """Last tick's encoder-derived heading change, radians."""
        return self._last_delta_theta_enc

    def __repr__(self) -> str:
        return (
            f"BicycleOdometry(x={self._x:.3f}m, y={self._y:.3f}m, "
            f"heading={math.degrees(self._theta):.1f}deg, "
            f"distance={self._distance:.2f}m, slip={self._slip:.3f})"
        )
