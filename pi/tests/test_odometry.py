"""Dead reckoning: a straight line stays straight and a circle closes.

There is no IMU on this car, so heading comes entirely from the bicycle model
and the encoders. That makes two properties worth pinning down hard. A straight
line must not curve -- any bias here becomes metres of error over a run. And a
constant-curvature arc must close, because if the integrator loses ground on a
circle it is losing ground on every corner too.

The model's known limitation is drift, not bias: 5-15 % closure error on a 5 m
square is expected on the real car and comes from tyre slip and a rolling radius
that changes with load. None of that exists in these tests, so here the numbers
have to be nearly exact.
"""

from __future__ import annotations

import math

import pytest

from telekart.config import VehicleConfig
from telekart.odometry import BicycleOdometry

DT = 0.01
#: Ten time constants of the 0.10 s steering lag. The HS-311 does not snap to
#: an angle, and the odometry models that -- so a test that wants a known
#: curvature has to let the lag settle before it starts measuring.
SETTLE_S = 1.0


def settle(odometry: BicycleOdometry, steer_angle: float) -> None:
    steps = int(round(SETTLE_S / DT))
    for _ in range(steps):
        odometry.update(0.0, 0.0, steer_angle, DT)


def turn_radius(config: VehicleConfig, steer_angle: float) -> float:
    """Radius of the arc, as a magnitude."""
    return abs(config.wheelbase_m / math.tan(steer_angle))


def wheel_split(config: VehicleConfig, distance: float, steer_angle: float) -> tuple[float, float]:
    """The pair of wheel distances the geometry actually produces for an arc.

    ``steer_angle`` is right-positive, so a positive angle puts the LEFT wheel
    on the outside and it covers more ground. Same expression the differential
    mixer uses, which is what makes ``slip_index`` fall to zero here.
    """
    split = config.track_width_m * math.tan(steer_angle) / (2.0 * config.wheelbase_m)
    return distance * (1.0 + split), distance * (1.0 - split)


# --------------------------------------------------------------------------
# Rest
# --------------------------------------------------------------------------


def test_a_fresh_odometry_sits_at_the_origin(odometry: BicycleOdometry) -> None:
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))
    assert odometry.distance == pytest.approx(0.0)
    assert odometry.slip_index == pytest.approx(0.0)


def test_standing_still_does_not_move_the_pose(odometry: BicycleOdometry) -> None:
    for _ in range(100):
        odometry.update(0.0, 0.0, 0.3, DT)
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))
    assert odometry.distance == pytest.approx(0.0)


def test_reset_returns_to_the_origin(odometry: BicycleOdometry) -> None:
    for _ in range(200):
        odometry.update(0.01, 0.01, 0.0, DT)
    odometry.reset()
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))
    assert odometry.distance == pytest.approx(0.0)


# --------------------------------------------------------------------------
# Straight
# --------------------------------------------------------------------------


def test_a_straight_line_is_straight(odometry: BicycleOdometry) -> None:
    for _ in range(500):
        odometry.update(0.01, 0.01, 0.0, DT)

    x, y, heading = odometry.pose
    assert x == pytest.approx(5.0, rel=1e-9)
    assert y == pytest.approx(0.0, abs=1e-12)
    assert heading == pytest.approx(0.0, abs=1e-12)
    assert odometry.distance == pytest.approx(5.0, rel=1e-9)


def test_driving_backwards_retraces_the_line(odometry: BicycleOdometry) -> None:
    for _ in range(200):
        odometry.update(0.01, 0.01, 0.0, DT)
    for _ in range(200):
        odometry.update(-0.01, -0.01, 0.0, DT)

    x, y, heading = odometry.pose
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-12)
    assert heading == pytest.approx(0.0, abs=1e-12)
    # The odometer records ground covered, whichever way the car was pointing:
    # a trip meter that ran backwards on reverse would be unreadable.
    assert odometry.distance == pytest.approx(4.0, rel=1e-6)


def test_a_heading_offset_is_carried_into_the_line(
    odometry: BicycleOdometry, config: VehicleConfig
) -> None:
    """Drive a turn, straighten up, and the straight leg must run along the new
    heading rather than reverting to the x axis."""
    delta = math.radians(24.0)
    settle(odometry, delta)
    for _ in range(200):
        left, right = wheel_split(config, 0.01, delta)
        odometry.update(left, right, delta, DT)

    settle(odometry, 0.0)
    before = odometry.pose
    for _ in range(100):
        odometry.update(0.01, 0.01, 0.0, DT)
    x, y, heading = odometry.pose

    # Not exact: ten time constants of settling leaves 4.5e-5 of the original
    # steer angle in the lag model, so the "straight" leg curves by a hundredth
    # of a degree. That residual is the model being honest, not an error.
    assert heading == pytest.approx(before[2], abs=1e-4)
    assert x - before[0] == pytest.approx(math.cos(heading), abs=1e-4)
    assert y - before[1] == pytest.approx(math.sin(heading), abs=1e-4)


# --------------------------------------------------------------------------
# Arcs
# --------------------------------------------------------------------------


def test_a_full_circle_closes(odometry: BicycleOdometry, config: VehicleConfig) -> None:
    """Full lock, exactly one circumference, back where it started.

    With the shipped geometry that is a 0.449 m radius -- a circle about 0.90 m
    across, which is the number the bench cross-check in calibration.md asks you
    to measure with a tape.
    """
    delta = config.steer_max_rad
    radius = turn_radius(config, delta)
    assert radius == pytest.approx(0.449, abs=0.005)

    settle(odometry, delta)
    steps = 2000
    arc = 2.0 * math.pi * radius / steps
    for _ in range(steps):
        left, right = wheel_split(config, arc, delta)
        odometry.update(left, right, delta, DT)

    x, y, heading = odometry.pose
    assert math.hypot(x, y) < 0.02 * radius
    assert abs(heading) < math.radians(1.0)
    assert odometry.distance == pytest.approx(2.0 * math.pi * radius, rel=1e-6)


def test_a_half_circle_lands_across_the_diameter(
    odometry: BicycleOdometry, config: VehicleConfig
) -> None:
    delta = config.steer_max_rad
    radius = turn_radius(config, delta)

    settle(odometry, delta)
    steps = 1000
    arc = math.pi * radius / steps
    for _ in range(steps):
        left, right = wheel_split(config, arc, delta)
        odometry.update(left, right, delta, DT)

    x, y, heading = odometry.pose
    assert abs(heading) == pytest.approx(math.pi, abs=math.radians(1.0))
    assert x == pytest.approx(0.0, abs=0.02 * radius)
    # Positive steer angle is a RIGHT turn and heading is counter-clockwise
    # positive, so the centre of the circle is to the right of the car and the
    # half circle finishes one diameter to the right: negative y.
    assert y == pytest.approx(-2.0 * radius, abs=0.02 * radius)


def test_left_and_right_arcs_are_mirror_images(config: VehicleConfig) -> None:
    def arc(sign: float) -> tuple[float, float, float]:
        odometry = BicycleOdometry(config)
        delta = sign * math.radians(15.0)
        settle(odometry, delta)
        for _ in range(300):
            left, right = wheel_split(config, 0.005, delta)
            odometry.update(left, right, delta, DT)
        return odometry.pose

    left_x, left_y, left_heading = arc(1.0)
    right_x, right_y, right_heading = arc(-1.0)
    assert right_x == pytest.approx(left_x, rel=1e-9)
    assert right_y == pytest.approx(-left_y, rel=1e-9)
    assert right_heading == pytest.approx(-left_heading, rel=1e-9)


def test_midpoint_integration_survives_a_coarse_step(config: VehicleConfig) -> None:
    """Second order, not Euler -- and the difference is not academic.

    A quarter circle in eight steps is 11 degrees of heading change per step.
    Integrating with the heading from the *start* of each step puts the endpoint
    about 10 % out; taking the midpoint heading keeps it under 1 %. Both cost
    the same, which is why there is no reason to use the worse one.
    """
    delta = math.radians(24.0)
    radius = turn_radius(config, delta)

    coarse = BicycleOdometry(config)
    settle(coarse, delta)
    steps = 8
    arc = (math.pi / 2.0) * radius / steps
    for _ in range(steps):
        left, right = wheel_split(config, arc, delta)
        coarse.update(left, right, delta, 0.1)

    x, y, _heading = coarse.pose
    assert x == pytest.approx(radius, rel=0.02)
    assert y == pytest.approx(-radius, rel=0.02)


def test_heading_stays_wrapped(odometry: BicycleOdometry, config: VehicleConfig) -> None:
    delta = config.steer_max_rad
    settle(odometry, delta)
    for _ in range(3000):
        left, right = wheel_split(config, 0.005, delta)
        odometry.update(left, right, delta, DT)
        assert -math.pi - 1e-9 <= odometry.pose[2] <= math.pi + 1e-9


# --------------------------------------------------------------------------
# Steering lag
# --------------------------------------------------------------------------


def test_the_steering_lag_delays_the_turn(config: VehicleConfig) -> None:
    """Assuming instant steering is wrong: the HS-311 is quoted at 0.19 s per
    60 degrees, and pretending otherwise puts the first fifth of a second of
    every corner in the wrong place."""
    lagged = BicycleOdometry(config)
    delta = config.steer_max_rad

    early = 0.0
    for _ in range(5):  # 50 ms, half a time constant
        left, right = wheel_split(config, 0.01, delta)
        lagged.update(left, right, delta, DT)
        early = abs(lagged.pose[2])

    settled = BicycleOdometry(config)
    settle(settled, delta)
    for _ in range(5):
        left, right = wheel_split(config, 0.01, delta)
        settled.update(left, right, delta, DT)

    assert early < abs(settled.pose[2])


# --------------------------------------------------------------------------
# Slip
# --------------------------------------------------------------------------


def test_slip_is_near_zero_when_the_wheels_match_the_geometry(
    odometry: BicycleOdometry, config: VehicleConfig
) -> None:
    delta = math.radians(18.0)
    settle(odometry, delta)
    for _ in range(200):
        left, right = wheel_split(config, 0.005, delta)
        odometry.update(left, right, delta, DT)
    assert odometry.slip_index == pytest.approx(0.0, abs=1e-6)


def test_slip_rises_when_the_wheels_disagree_with_the_steering(
    odometry: BicycleOdometry, config: VehicleConfig
) -> None:
    """Equal wheel distances at full lock is physically impossible without the
    tyres scrubbing. It is exactly the signal a wheelspin indicator wants, and
    it is free."""
    delta = config.steer_max_rad
    settle(odometry, delta)
    for _ in range(200):
        odometry.update(0.005, 0.005, delta, DT)
    assert odometry.slip_index > 1e-3


def test_slip_is_zero_on_a_straight_line(odometry: BicycleOdometry) -> None:
    for _ in range(200):
        odometry.update(0.01, 0.01, 0.0, DT)
    assert odometry.slip_index == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------------------
# Robustness
# --------------------------------------------------------------------------


def test_a_nan_measurement_is_dropped_not_integrated(odometry: BicycleOdometry) -> None:
    """A pose that has gone to NaN cannot be recovered without a reset, and it
    takes the whole HUD with it. Dropping the sample is the only useful
    response inside a control tick."""
    for bad in (float("nan"),):
        odometry.update(bad, 0.01, 0.0, DT)
        odometry.update(0.01, bad, 0.0, DT)
        odometry.update(0.01, 0.01, bad, DT)
    assert all(math.isfinite(value) for value in odometry.pose)
    assert math.isfinite(odometry.distance)
    assert math.isfinite(odometry.slip_index)


def test_non_positive_dt_is_ignored(odometry: BicycleOdometry) -> None:
    for _ in range(50):
        odometry.update(0.01, 0.01, 0.0, DT)
    before = odometry.pose
    odometry.update(0.01, 0.01, 0.2, 0.0)
    odometry.update(0.01, 0.01, 0.2, -0.05)
    assert odometry.pose == pytest.approx(before)
