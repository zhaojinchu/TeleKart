"""Property tests for the shaping primitives.

These are pure functions with no state, which makes exhaustive-ish testing cheap
and worth doing: they sit between the operator's thumb and the H-bridge, and a
discontinuity in any of them is felt directly through the car. Sampling is a
fixed grid plus a seeded RNG, so a failure is always reproducible.
"""

from __future__ import annotations

import math
import random

import pytest

from telekart.control.shaping import (
    clamp,
    deadzone,
    expo,
    lerp,
    rate_limit,
    speed_sensitive_scale,
    wrap_pi,
)

#: Fine enough that a step of 1 % would not slip between two samples.
GRID = [i / 200.0 for i in range(-200, 201)]
DEADZONES = (0.0, 0.01, 0.03, 0.10, 0.20)
GAMMAS = (0.5, 1.0, 1.8, 3.0)


def _rng() -> random.Random:
    return random.Random(20260816)


# --------------------------------------------------------------------------
# clamp / lerp
# --------------------------------------------------------------------------


def test_clamp_bounds_and_passthrough() -> None:
    assert clamp(0.5, 0.0, 1.0) == 0.5
    assert clamp(-3.0, 0.0, 1.0) == 0.0
    assert clamp(9.0, 0.0, 1.0) == 1.0
    assert clamp(-1.0, -1.0, 1.0) == -1.0


def test_clamp_result_is_always_inside_the_interval() -> None:
    rng = _rng()
    for _ in range(2000):
        lo = rng.uniform(-10.0, 0.0)
        hi = lo + rng.uniform(0.0, 20.0)
        value = rng.uniform(-50.0, 50.0)
        assert lo <= clamp(value, lo, hi) <= hi


def test_clamp_passes_nan_free_infinities_to_the_bound() -> None:
    """A corrupt parameter must land on a limit, not propagate infinity into duty."""
    assert clamp(float("inf"), -1.0, 1.0) == 1.0
    assert clamp(float("-inf"), -1.0, 1.0) == -1.0


def test_lerp_endpoints_and_midpoint() -> None:
    assert lerp(2.0, 4.0, 0.0) == pytest.approx(2.0)
    assert lerp(2.0, 4.0, 1.0) == pytest.approx(4.0)
    assert lerp(2.0, 4.0, 0.5) == pytest.approx(3.0)
    assert lerp(-1.0, 1.0, 0.25) == pytest.approx(-0.5)


def test_lerp_is_monotonic_in_t() -> None:
    previous = lerp(-3.0, 7.0, 0.0)
    for step in range(1, 101):
        current = lerp(-3.0, 7.0, step / 100.0)
        assert current >= previous
        previous = current


# --------------------------------------------------------------------------
# deadzone
# --------------------------------------------------------------------------


@pytest.mark.parametrize("dz", DEADZONES)
def test_deadzone_output_stays_in_range(dz: float) -> None:
    for value in GRID:
        assert -1.0 <= deadzone(value, dz) <= 1.0


@pytest.mark.parametrize("dz", DEADZONES)
def test_deadzone_is_monotonic(dz: float) -> None:
    previous = deadzone(-1.0, dz)
    for value in GRID:
        current = deadzone(value, dz)
        assert current >= previous - 1e-12
        previous = current


@pytest.mark.parametrize("dz", DEADZONES)
def test_deadzone_zeroes_inside_and_reaches_full_scale_outside(dz: float) -> None:
    assert deadzone(0.0, dz) == 0.0
    if dz > 0.0:
        assert deadzone(dz * 0.99, dz) == 0.0
        assert deadzone(-dz * 0.99, dz) == 0.0
    # Full deflection must still mean full deflection. A deadzone that costs
    # you the top of the range means 100 % throttle is unreachable, which is
    # the failure the rescaling exists to prevent.
    assert deadzone(1.0, dz) == pytest.approx(1.0)
    assert deadzone(-1.0, dz) == pytest.approx(-1.0)


@pytest.mark.parametrize("dz", (0.01, 0.03, 0.10, 0.20))
def test_deadzone_is_continuous_at_its_edge(dz: float) -> None:
    """No step at the boundary.

    An unrescaled deadzone jumps straight from 0 to ``dz`` the instant the stick
    clears it, and on a throttle that is a lurch you can feel.
    """
    for epsilon in (1e-9, 1e-7, 1e-5):
        assert deadzone(dz + epsilon, dz) == pytest.approx(0.0, abs=1e-4)
        assert deadzone(-dz - epsilon, dz) == pytest.approx(0.0, abs=1e-4)


@pytest.mark.parametrize("dz", DEADZONES)
def test_deadzone_is_odd_about_zero(dz: float) -> None:
    for value in GRID:
        assert deadzone(-value, dz) == pytest.approx(-deadzone(value, dz))


def test_deadzone_of_zero_is_the_identity() -> None:
    for value in GRID:
        assert deadzone(value, 0.0) == pytest.approx(value)


# --------------------------------------------------------------------------
# expo
# --------------------------------------------------------------------------


@pytest.mark.parametrize("gamma", GAMMAS)
def test_expo_output_stays_in_range(gamma: float) -> None:
    for value in GRID:
        assert -1.0 <= expo(value, gamma) <= 1.0


@pytest.mark.parametrize("gamma", GAMMAS)
def test_expo_preserves_sign_and_endpoints(gamma: float) -> None:
    assert expo(0.0, gamma) == 0.0
    assert expo(1.0, gamma) == pytest.approx(1.0)
    assert expo(-1.0, gamma) == pytest.approx(-1.0)
    for value in GRID:
        shaped = expo(value, gamma)
        assert shaped == 0.0 or (shaped > 0.0) == (value > 0.0)


@pytest.mark.parametrize("gamma", GAMMAS)
def test_expo_is_monotonic(gamma: float) -> None:
    previous = expo(-1.0, gamma)
    for value in GRID:
        current = expo(value, gamma)
        assert current >= previous - 1e-12
        previous = current


def test_expo_gamma_one_is_the_identity() -> None:
    for value in GRID:
        assert expo(value, 1.0) == pytest.approx(value)


def test_expo_above_one_gives_finer_control_near_neutral() -> None:
    """Which is the whole point of the parameter: 1.0 is linear, higher is finer."""
    for value in GRID:
        assert abs(expo(value, 1.8)) <= abs(value) + 1e-12
        assert abs(expo(value, 0.5)) >= abs(value) - 1e-12
    assert expo(0.5, 2.0) == pytest.approx(0.25)
    assert expo(-0.5, 2.0) == pytest.approx(-0.25)


# --------------------------------------------------------------------------
# rate_limit
# --------------------------------------------------------------------------


def test_rate_limit_never_moves_further_than_the_rate_allows() -> None:
    rng = _rng()
    for _ in range(2000):
        current = rng.uniform(-1.0, 1.0)
        target = rng.uniform(-1.0, 1.0)
        max_rate = rng.uniform(0.1, 50.0)
        dt = rng.uniform(1e-4, 0.05)
        result = rate_limit(target, current, max_rate, dt)
        assert abs(result - current) <= max_rate * dt + 1e-12
        # Never overshoots the target either.
        assert min(current, target) - 1e-12 <= result <= max(current, target) + 1e-12


def test_rate_limit_reaches_the_target_when_it_can() -> None:
    assert rate_limit(1.0, 0.0, 100.0, 0.01) == pytest.approx(1.0)
    assert rate_limit(-1.0, 0.0, 100.0, 0.01) == pytest.approx(-1.0)
    assert rate_limit(0.5, 0.5, 0.0, 0.01) == pytest.approx(0.5)


def test_rate_limit_converges_monotonically() -> None:
    current = 0.0
    previous_error = 1.0
    for _ in range(200):
        current = rate_limit(1.0, current, 20.0, 0.01)
        error = abs(1.0 - current)
        assert error <= previous_error + 1e-12
        previous_error = error
    assert current == pytest.approx(1.0)


def test_rate_limit_of_zero_dt_does_not_move() -> None:
    assert rate_limit(1.0, 0.25, 1000.0, 0.0) == pytest.approx(0.25)


# --------------------------------------------------------------------------
# wrap_pi
# --------------------------------------------------------------------------


def test_wrap_pi_lands_in_a_single_turn() -> None:
    rng = _rng()
    for _ in range(5000):
        angle = rng.uniform(-100.0, 100.0)
        wrapped = wrap_pi(angle)
        assert -math.pi - 1e-12 <= wrapped <= math.pi + 1e-12


def test_wrap_pi_is_periodic() -> None:
    rng = _rng()
    for _ in range(1000):
        angle = rng.uniform(-math.pi, math.pi)
        for turns in (-3, -1, 1, 2, 7):
            assert wrap_pi(angle + turns * math.tau) == pytest.approx(
                wrap_pi(angle), abs=1e-9
            )


def test_wrap_pi_known_values() -> None:
    assert wrap_pi(0.0) == pytest.approx(0.0)
    assert wrap_pi(math.pi / 2) == pytest.approx(math.pi / 2)
    assert wrap_pi(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert abs(wrap_pi(math.pi)) == pytest.approx(math.pi)
    assert wrap_pi(math.tau) == pytest.approx(0.0, abs=1e-9)


# --------------------------------------------------------------------------
# speed_sensitive_scale
# --------------------------------------------------------------------------


def test_speed_sensitive_scale_endpoints() -> None:
    assert speed_sensitive_scale(0.0, 0.45) == pytest.approx(1.0)
    assert speed_sensitive_scale(1.0, 0.45) == pytest.approx(0.55)
    assert speed_sensitive_scale(0.5, 0.0) == pytest.approx(1.0)


def test_speed_sensitive_scale_is_bounded_and_decreasing() -> None:
    for reduction in (0.0, 0.2, 0.45, 0.8):
        previous = speed_sensitive_scale(0.0, reduction)
        for step in range(0, 101):
            value = speed_sensitive_scale(step / 100.0, reduction)
            assert 1.0 - reduction - 1e-12 <= value <= 1.0 + 1e-12
            assert value <= previous + 1e-12
            previous = value


def test_speed_sensitive_scale_clamps_a_nonsense_speed_fraction() -> None:
    """Speed fraction comes from measured RPM over a measured maximum, and on a
    freshly calibrated car it can briefly exceed 1.0. Steering authority must
    not go negative when it does."""
    assert speed_sensitive_scale(-0.5, 0.45) == pytest.approx(1.0)
    assert speed_sensitive_scale(3.0, 0.45) == pytest.approx(0.55)
