"""The electronic differential, checked against the closed-form geometry.

Two motors on a solid rear axle with Ackermann front steering and no mechanical
differential: in a turn the rear wheels *must* run at different speeds. Getting
the split wrong does not look like a bug -- the car still drives -- it looks like
tyre scrub, a heavier steering feel, and two per-wheel PIDs quietly fighting
each other for the whole corner.
"""

from __future__ import annotations

import math

import pytest

from telekart.config import VehicleConfig
from telekart.control.mixer import DifferentialMixer, WheelTargets


def expected_split(config: VehicleConfig, steer_angle: float) -> float:
    return config.track_width_m * math.tan(steer_angle) / (2.0 * config.wheelbase_m)


# --------------------------------------------------------------------------
# Degenerate cases
# --------------------------------------------------------------------------


def test_straight_ahead_leaves_both_wheels_equal(mixer: DifferentialMixer) -> None:
    targets = mixer.mix(120.0, 0.0)
    assert isinstance(targets, WheelTargets)
    assert targets.rpm_l == pytest.approx(120.0)
    assert targets.rpm_r == pytest.approx(120.0)


def test_zero_target_stays_zero_at_full_lock(
    mixer: DifferentialMixer, config: VehicleConfig
) -> None:
    targets = mixer.mix(0.0, config.steer_max_rad)
    assert targets.rpm_l == pytest.approx(0.0)
    assert targets.rpm_r == pytest.approx(0.0)


# --------------------------------------------------------------------------
# The closed form
# --------------------------------------------------------------------------


def test_split_matches_the_closed_form_at_full_lock(
    mixer: DifferentialMixer, config: VehicleConfig
) -> None:
    """split = track_width * tan(delta) / (2 * wheelbase).

    With the shipped geometry that is 0.150 * tan(24 deg) / 0.400 = 0.167, so
    the two wheels differ by a third of the target. Not a rounding error.
    """
    delta = config.steer_max_rad
    split = expected_split(config, delta)
    assert split == pytest.approx(0.167, abs=0.002)

    targets = mixer.mix(150.0, delta)
    assert abs(targets.rpm_r - targets.rpm_l) == pytest.approx(2.0 * split * 150.0)
    assert (targets.rpm_l + targets.rpm_r) / 2.0 == pytest.approx(150.0)


@pytest.mark.parametrize("degrees", (-24.0, -12.0, -3.0, 3.0, 12.0, 24.0))
def test_split_matches_the_closed_form_across_the_range(
    mixer: DifferentialMixer, config: VehicleConfig, degrees: float
) -> None:
    delta = math.radians(degrees)
    split = expected_split(config, delta)
    targets = mixer.mix(100.0, delta)
    assert targets.rpm_l == pytest.approx(100.0 * (1.0 + split), abs=1e-6)
    assert targets.rpm_r == pytest.approx(100.0 * (1.0 - split), abs=1e-6)


def test_the_outside_wheel_is_the_faster_one(
    mixer: DifferentialMixer, config: VehicleConfig
) -> None:
    """Fixes the sign convention, which is the one thing here that cannot be
    derived from magnitudes alone.

    ``steer_angle`` is positive to the RIGHT, matching the wire protocol's
    steering field and the servo driver's pulse mapping. Steering right puts the
    LEFT wheel on the outside of the arc, so the left target is the larger one.
    """
    right_turn = mixer.mix(100.0, math.radians(20.0))
    assert right_turn.rpm_l > right_turn.rpm_r

    left_turn = mixer.mix(100.0, math.radians(-20.0))
    assert left_turn.rpm_r > left_turn.rpm_l


def test_the_split_agrees_with_the_odometry_model(
    mixer: DifferentialMixer, config: VehicleConfig
) -> None:
    """The consistency check that actually protects the car.

    Feed the mixer's own wheel speeds back through the encoder-derived heading
    rate, ``(d_right - d_left) / track_width``, and it must reproduce the
    bicycle model's ``-d * tan(delta) / wheelbase`` exactly -- the same
    expression ``BicycleOdometry.update`` integrates, minus sign included. If
    the mixer and the odometry disagree about which way positive steering
    turns, the slip index reads full scale on every corner and any traction
    work built on it is garbage.
    """
    dt = 0.01
    for degrees in (-24.0, -10.0, 10.0, 24.0):
        delta = math.radians(degrees)
        targets = mixer.mix(120.0, delta)

        d_left = config.speed_for_rpm(targets.rpm_l) * dt
        d_right = config.speed_for_rpm(targets.rpm_r) * dt
        distance = (d_left + d_right) / 2.0

        from_encoders = (d_right - d_left) / config.track_width_m
        from_bicycle = -distance * math.tan(delta) / config.wheelbase_m
        assert from_encoders == pytest.approx(from_bicycle, rel=1e-9, abs=1e-15)


# --------------------------------------------------------------------------
# Symmetries
# --------------------------------------------------------------------------


def test_mirroring_the_steer_angle_swaps_the_wheels(mixer: DifferentialMixer) -> None:
    delta = math.radians(18.0)
    left = mixer.mix(90.0, delta)
    right = mixer.mix(90.0, -delta)
    assert left.rpm_l == pytest.approx(right.rpm_r)
    assert left.rpm_r == pytest.approx(right.rpm_l)


def test_reverse_negates_both_wheels(mixer: DifferentialMixer) -> None:
    """Backing round a corner is the same geometry with the sign flipped; the
    outside wheel still covers more ground."""
    delta = math.radians(15.0)
    forward = mixer.mix(80.0, delta)
    backward = mixer.mix(-80.0, delta)
    assert backward.rpm_l == pytest.approx(-forward.rpm_l)
    assert backward.rpm_r == pytest.approx(-forward.rpm_r)


def test_split_grows_with_steering_angle(mixer: DifferentialMixer) -> None:
    previous = 0.0
    for degrees in (0.0, 5.0, 10.0, 15.0, 20.0, 24.0):
        targets = mixer.mix(100.0, math.radians(degrees))
        spread = abs(targets.rpm_r - targets.rpm_l)
        assert spread >= previous - 1e-12
        previous = spread


def test_mean_speed_is_preserved_at_every_angle(mixer: DifferentialMixer) -> None:
    """The differential redistributes speed between the wheels; it must not add
    or remove any. If the mean drifted, throttle response would change with
    steering angle."""
    for degrees in (-24.0, -8.0, 0.0, 8.0, 24.0):
        targets = mixer.mix(140.0, math.radians(degrees))
        assert (targets.rpm_l + targets.rpm_r) / 2.0 == pytest.approx(140.0)


# --------------------------------------------------------------------------
# Robustness
# --------------------------------------------------------------------------


def test_a_nonsense_steer_angle_does_not_produce_infinities(
    mixer: DifferentialMixer,
) -> None:
    """tan() goes to 1e16 next to a right angle. A corrupt parameter must not
    turn that into a NaN wheel target inside the control loop."""
    for delta in (math.pi / 2 - 1e-9, -math.pi / 2 + 1e-9, 10.0, -10.0):
        targets = mixer.mix(100.0, delta)
        assert math.isfinite(targets.rpm_l)
        assert math.isfinite(targets.rpm_r)


def test_wheel_targets_are_immutable(mixer: DifferentialMixer) -> None:
    """A frozen result is what stops one stage of the pipeline editing another's
    output in place."""
    targets = mixer.mix(50.0, 0.1)
    with pytest.raises(Exception):
        targets.rpm_l = 0.0  # type: ignore[misc]
