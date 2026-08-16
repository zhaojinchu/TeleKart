"""PID behaviour, with most of the attention on anti-windup.

Windup is the failure that hurts on this car specifically: the wheel spends
real time saturated -- a blocked wheel, a regulator hiccup, the bottom of the
deadband -- and an integrator that keeps charging through it produces a burst of
duty on release. On a bench that is startling. With the wheels on the ground it
is the car leaving without you.
"""

from __future__ import annotations

import math
import random

import pytest

from telekart.control.pid import PID

DT = 0.01


def _run(pid: PID, setpoint: float, measured: float, seconds: float, **kwargs: float) -> float:
    output = 0.0
    for _ in range(int(round(seconds / DT))):
        output = pid.update(setpoint, measured, DT, **kwargs)
    return output


# --------------------------------------------------------------------------
# Terms in isolation
# --------------------------------------------------------------------------


def test_proportional_term_alone() -> None:
    pid = PID(0.05, 0.0, 0.0)
    assert pid.update(10.0, 0.0, DT) == pytest.approx(0.5)
    assert pid.update(-10.0, 0.0, DT) == pytest.approx(-0.5)
    assert not pid.saturated


def test_output_is_clamped_and_flagged() -> None:
    pid = PID(0.05, 0.0, 0.0)
    assert pid.update(100.0, 0.0, DT) == pytest.approx(1.0)
    assert pid.saturated
    assert pid.update(-100.0, 0.0, DT) == pytest.approx(-1.0)
    assert pid.saturated
    pid.update(1.0, 0.0, DT)
    assert not pid.saturated


def test_custom_output_limits_are_respected() -> None:
    pid = PID(1.0, 0.0, 0.0, output_limits=(0.0, 0.5))
    assert pid.update(10.0, 0.0, DT) == pytest.approx(0.5)
    assert pid.update(-10.0, 0.0, DT) == pytest.approx(0.0)


def test_integral_term_accumulates_at_the_expected_rate() -> None:
    pid = PID(0.0, 0.1, 0.0, i_clamp=1.0)
    output = _run(pid, 1.0, 0.0, 0.5)
    # ki * error * elapsed, give or take whether the first tick is counted.
    assert output == pytest.approx(0.1 * 1.0 * 0.5, rel=0.05, abs=0.1 * DT)


def test_integral_is_clamped() -> None:
    pid = PID(0.0, 5.0, 0.0, i_clamp=0.4)
    output = _run(pid, 1.0, 0.0, 5.0)
    assert output == pytest.approx(0.4, abs=1e-6)
    output = _run(pid, -1.0, 0.0, 10.0)
    assert output == pytest.approx(-0.4, abs=1e-6)


def test_derivative_damps_an_approaching_measurement() -> None:
    """Whatever form the derivative takes, it must oppose the motion."""
    without = PID(0.02, 0.0, 0.0)
    with_kd = PID(0.02, 0.0, 0.01)
    plain = damped = 0.0
    measured = 0.0
    for _ in range(20):
        measured += 2.0  # closing on the setpoint at 200 RPM/s
        plain = without.update(100.0, measured, DT)
        damped = with_kd.update(100.0, measured, DT)
    assert damped < plain


def test_derivative_kick_is_bounded_by_the_filter() -> None:
    """Encoder velocity is chunky. An unfiltered derivative on a one-sample jump
    would command full duty from a quantisation artefact."""
    pid = PID(0.0, 0.0, 0.01, derivative_lpf_hz=10.0)
    pid.update(100.0, 100.0, DT)
    output = pid.update(100.0, 90.0, DT)  # a 10 RPM step in one 10 ms tick
    assert math.isfinite(output)
    assert abs(output) <= 1.0


# --------------------------------------------------------------------------
# Anti-windup
# --------------------------------------------------------------------------


def test_integrator_does_not_wind_up_while_saturated() -> None:
    """The test that matters.

    Five seconds of an error the actuator cannot answer. With conditional
    integration the integrator is frozen, so the moment the error clears the
    output falls back to the clamp. Without it, the integrator holds tens of
    units of charge and the output stays pinned for seconds afterwards.
    """
    pid = PID(0.05, 0.5, 0.0, i_clamp=0.4)
    saturated_output = _run(pid, 200.0, 0.0, 5.0)
    assert saturated_output == pytest.approx(1.0)
    assert pid.saturated

    released = pid.update(200.0, 200.0, DT)  # error goes to zero
    assert released <= 0.4 + 1e-6, "integrator kept charging while the output was pinned"
    assert not pid.saturated


def test_no_overshoot_burst_after_a_stall_release() -> None:
    """The bench procedure: hold the wheel for two seconds, then let go.

    A burst on release is the documented signature of broken anti-windup, and
    it is a bug rather than something to be tuned around by lowering ki.
    """
    pid = PID(0.005, 0.020, 0.0, i_clamp=0.4)
    target = 100.0

    # Wheel blocked: the measurement stays at zero however hard we push.
    for _ in range(200):
        pid.update(target, 0.0, DT)

    # Released, and now tracking perfectly. Nothing left over should show up.
    peak = 0.0
    for _ in range(100):
        peak = max(peak, pid.update(target, target, DT))
    assert peak <= 0.4 + 1e-6


def test_feedforward_plus_integral_stays_inside_the_limits() -> None:
    """Feedforward carries the load, so the integrator has to be clamped against
    what is left of the range rather than against the full range."""
    pid = PID(0.0, 5.0, 0.0, i_clamp=0.4)
    for _ in range(500):
        output = pid.update(100.0, 0.0, DT, feedforward=0.9)
        assert output <= 1.0 + 1e-9

    settled = pid.update(100.0, 100.0, DT, feedforward=0.9)
    assert settled <= 1.0 + 1e-9
    assert settled >= 0.9 - 1e-9


def test_integrator_unwinds_immediately_when_the_error_reverses() -> None:
    pid = PID(0.005, 0.05, 0.0, i_clamp=0.4)
    _run(pid, 150.0, 0.0, 3.0)
    assert pid.saturated

    # Overspeed now: the controller must be able to command down at once.
    reversed_output = pid.update(150.0, 400.0, DT)
    assert reversed_output < 0.0


# --------------------------------------------------------------------------
# Lifecycle
# --------------------------------------------------------------------------


def test_reset_clears_the_integrator_and_the_derivative_history() -> None:
    pid = PID(0.0, 5.0, 0.01, i_clamp=0.4)
    _run(pid, 100.0, 0.0, 2.0)
    pid.reset()
    assert pid.update(100.0, 100.0, DT) == pytest.approx(0.0)
    assert not pid.saturated


def test_set_gains_takes_effect_without_disturbing_the_state() -> None:
    pid = PID(0.01, 0.0, 0.0)
    assert pid.update(10.0, 0.0, DT) == pytest.approx(0.1)
    pid.set_gains(0.05, 0.0, 0.0)
    assert pid.update(10.0, 0.0, DT) == pytest.approx(0.5)


# --------------------------------------------------------------------------
# Robustness -- this runs at 100 Hz and may never raise
# --------------------------------------------------------------------------


def test_zero_and_negative_dt_are_survivable() -> None:
    pid = PID(0.01, 0.1, 0.01)
    for dt in (0.0, -0.01):
        output = pid.update(50.0, 10.0, dt)
        assert math.isfinite(output)
        assert -1.0 <= output <= 1.0


def test_output_stays_in_range_for_arbitrary_inputs() -> None:
    rng = random.Random(20260816)
    pid = PID(0.01, 0.05, 0.005)
    for _ in range(5000):
        output = pid.update(
            rng.uniform(-500.0, 500.0),
            rng.uniform(-500.0, 500.0),
            rng.uniform(0.001, 0.05),
            feedforward=rng.uniform(-1.0, 1.0),
        )
        assert -1.0 <= output <= 1.0
        assert math.isfinite(output)


# --------------------------------------------------------------------------
# Closed loop against a first-order plant
# --------------------------------------------------------------------------


def test_step_response_converges_with_near_zero_steady_state_error() -> None:
    """A 0.15 s first-order lag with a gain of 165 RPM per unit duty, which is
    roughly what this drivetrain is. PI action must remove the droop.

    The integrator clamp is opened to 1.0 here because this test runs with no
    feedforward at all. On the car the clamp is 0.4 and the feedforward carries
    the operating point; a 0.4 integrator on its own cannot reach the 0.61 duty
    that 100 RPM needs, and would settle 19 % low by design rather than by bug.
    """
    pid = PID(0.005, 0.020, 0.0, i_clamp=1.0)
    tau, gain = 0.15, 165.0
    target = 100.0

    rpm = 0.0
    history = []
    for _ in range(500):  # five seconds
        duty = pid.update(target, rpm, DT)
        rpm += (gain * duty - rpm) * (DT / tau)
        history.append(rpm)

    assert history[-1] == pytest.approx(target, rel=0.02)
    assert max(history) <= target * 1.35, "overshoot beyond a third of the step"
    # And it got there in a sensible time rather than creeping for the whole run.
    settled_at = next(
        index for index, value in enumerate(history) if abs(value - target) < 0.05 * target
    )
    assert settled_at * DT < 1.5


def test_feedforward_carries_the_load_and_the_pid_only_trims() -> None:
    """With a correct feedforward the integrator should barely move -- that is
    what makes the loop tunable when the plant gain is ~2x uncertain."""
    pid = PID(0.005, 0.020, 0.0, i_clamp=0.4)
    tau, gain = 0.15, 165.0
    target = 100.0
    feedforward = target / gain

    rpm = 0.0
    trim = 0.0
    for _ in range(300):
        duty = pid.update(target, rpm, DT, feedforward=feedforward)
        trim = duty - feedforward
        rpm += (gain * duty - rpm) * (DT / tau)

    assert rpm == pytest.approx(target, rel=0.02)
    assert abs(trim) < 0.05
