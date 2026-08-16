"""The whole loop, closed, in virtual time.

Everything below runs the assembled controller against the mock drivetrain: a
first-order motor lag, a stiction deadband, rail sag under combined load, and
synthesised encoder edges. That is the difference between proving the code ran
and proving the loop *closes* -- against a mock that only records pin writes,
a wrong PID gain passes every test.

Sixty seconds of driving here costs milliseconds of wall clock and produces the
same numbers on every machine.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import pytest

from telekart.config import VehicleConfig
from telekart.constants import CONTROL_PERIOD_S
from telekart.hal.mock_backend import MockBackend

from telekart_protocol import ControlFlags, TelemetryFlags, VehicleState

if TYPE_CHECKING:
    # Import-time only. Two packages in this repository are called ``tests``, so
    # a runtime `from .conftest import ...` resolves to whichever was imported
    # first when both suites are collected in one session.
    from .conftest import Vehicle

#: Long enough for a 0.15 s plant behind a 250 RPM/s target ramp to settle.
SETTLE_S = 3.0


def steady_rpm(vehicle: Vehicle, throttle: float, *, steering: float = 0.0) -> tuple[float, float]:
    vehicle.run(SETTLE_S, command={"throttle": throttle, "steering": steering})
    return vehicle.rpm


# --------------------------------------------------------------------------
# Nothing moves until it is armed
# --------------------------------------------------------------------------


def test_a_disarmed_car_ignores_full_throttle(vehicle: Vehicle) -> None:
    vehicle.run(2.0, command={"throttle": 1.0})
    left, right = vehicle.rpm
    assert left == pytest.approx(0.0, abs=1e-6)
    assert right == pytest.approx(0.0, abs=1e-6)
    assert vehicle.duty == (0.0, 0.0)


def test_arming_takes_and_reports_itself(vehicle: Vehicle) -> None:
    accepted, reason = vehicle.arm()
    assert accepted, reason
    assert vehicle.state is VehicleState.ARMED
    state = vehicle.step()
    assert state.state is VehicleState.ARMED


# --------------------------------------------------------------------------
# Closed loop
# --------------------------------------------------------------------------


def test_rpm_converges_on_its_target(vehicle: Vehicle) -> None:
    """The property that matters: whatever the shaping chain decided the target
    should be, the wheels get there."""
    accepted, reason = vehicle.arm()
    assert accepted, reason

    state = vehicle.run(SETTLE_S, command={"throttle": 0.6})
    assert state.rpm_target_l > 5.0, "the shaping chain produced no target at all"
    assert state.rpm_l == pytest.approx(state.rpm_target_l, rel=0.10)
    assert state.rpm_r == pytest.approx(state.rpm_target_r, rel=0.10)

    # And the estimate agrees with what the plant is really doing.
    truth_l, truth_r = vehicle.rpm
    assert state.rpm_l == pytest.approx(truth_l, rel=0.15)
    assert state.rpm_r == pytest.approx(truth_r, rel=0.15)


def test_more_throttle_gives_more_speed(vehicle: Vehicle) -> None:
    vehicle.arm()
    slow = steady_rpm(vehicle, 0.35)[0]
    medium = steady_rpm(vehicle, 0.6)[0]
    fast = steady_rpm(vehicle, 0.95)[0]
    assert slow < medium < fast


def test_the_loop_holds_a_steady_speed(vehicle: Vehicle) -> None:
    """Peak-to-peak ripple under 8 % of the setpoint. Sustained oscillation here
    is the symptom of pid_kp being too high."""
    vehicle.arm()
    vehicle.run(SETTLE_S, command={"throttle": 0.6})

    samples = []
    for _ in range(200):  # two seconds
        vehicle.submit(throttle=0.6)
        state = vehicle.step()
        samples.append(state.rpm_l)

    mean = sum(samples) / len(samples)
    assert mean > 5.0
    assert (max(samples) - min(samples)) < 0.08 * mean


def test_releasing_the_throttle_slows_the_car(vehicle: Vehicle) -> None:
    vehicle.arm()
    moving = steady_rpm(vehicle, 0.8)[0]
    stopped = steady_rpm(vehicle, 0.0)[0]
    assert moving > 20.0
    assert stopped < moving * 0.2


# --------------------------------------------------------------------------
# The electronic differential
# --------------------------------------------------------------------------


def test_the_differential_engages_in_a_turn(vehicle: Vehicle, config: VehicleConfig) -> None:
    """Two motors on a solid rear axle with Ackermann steering: at full lock the
    wheels must differ by about a third, or the tyres scrub and the two PIDs
    spend the corner fighting the geometry."""
    vehicle.arm()
    state = vehicle.run(SETTLE_S, command={"throttle": 0.7, "steering": 1.0})

    mean_target = (state.rpm_target_l + state.rpm_target_r) / 2.0
    assert mean_target > 5.0
    spread = abs(state.rpm_target_r - state.rpm_target_l) / mean_target

    split = config.track_width_m * math.tan(config.steer_max_rad) / (2.0 * config.wheelbase_m)
    assert spread == pytest.approx(2.0 * split, rel=0.25)

    # And it is not merely a target: the wheels actually run at different speeds.
    left, right = vehicle.rpm
    assert abs(left - right) > 0.10 * (abs(left) + abs(right)) / 2.0


def test_straight_ahead_keeps_the_wheels_together(vehicle: Vehicle) -> None:
    vehicle.arm()
    state = vehicle.run(SETTLE_S, command={"throttle": 0.7, "steering": 0.0})
    assert state.rpm_target_l == pytest.approx(state.rpm_target_r, rel=1e-6)
    left, right = vehicle.rpm
    assert left == pytest.approx(right, rel=0.05, abs=1.0)


def test_opposite_lock_mirrors_the_split(vehicle: Vehicle) -> None:
    vehicle.arm()
    left_lock = vehicle.run(SETTLE_S, command={"throttle": 0.7, "steering": -1.0})
    left_spread = left_lock.rpm_target_r - left_lock.rpm_target_l

    right_lock = vehicle.run(SETTLE_S, command={"throttle": 0.7, "steering": 1.0})
    right_spread = right_lock.rpm_target_r - right_lock.rpm_target_l

    assert left_spread * right_spread < 0.0, "both locks split the same way"


# --------------------------------------------------------------------------
# Steering
# --------------------------------------------------------------------------


def test_the_servo_stays_inside_its_calibrated_limits(
    vehicle: Vehicle, config: VehicleConfig
) -> None:
    """The endpoints were found by walking outward until the servo strained.
    Driving past them stalls it at 700-800 mA on the Pi's own 5 V rail."""
    vehicle.arm()
    for steering in (-1.0, -0.5, 0.0, 0.5, 1.0, -1.0):
        state = vehicle.run(0.5, command={"throttle": 0.2, "steering": steering})
        assert config.steer_min_us <= state.servo_us <= config.steer_max_us


def test_steering_moves_the_servo_in_opposite_directions(vehicle: Vehicle) -> None:
    vehicle.arm()
    left = vehicle.run(1.0, command={"steering": -1.0}).servo_us
    centre = vehicle.run(1.0, command={"steering": 0.0}).servo_us
    right = vehicle.run(1.0, command={"steering": 1.0}).servo_us
    assert (left - centre) * (right - centre) < 0
    assert abs(left - centre) > 10
    assert abs(right - centre) > 10


# --------------------------------------------------------------------------
# Braking
# --------------------------------------------------------------------------


def test_the_brake_command_stops_the_car_faster_than_lifting_off(
    vehicle: Vehicle,
) -> None:
    vehicle.arm()
    vehicle.run(SETTLE_S, command={"throttle": 0.8})
    from_speed = vehicle.rpm[0]
    assert from_speed > 20.0

    braked = vehicle.run(0.5, command={"throttle": 0.0, "brake": 1.0})
    assert vehicle.rpm[0] < from_speed * 0.5
    assert braked.flags & TelemetryFlags.BRAKING


# --------------------------------------------------------------------------
# Failsafe
# --------------------------------------------------------------------------


def test_failsafe_fires_when_commands_stop(vehicle: Vehicle, config: VehicleConfig) -> None:
    """No packets, no motion. The staged schedule is coast, brake, coast,
    disarm -- and by the end of it the bridge must be off."""
    vehicle.arm()
    vehicle.run(SETTLE_S, command={"throttle": 0.7})
    assert vehicle.state is VehicleState.ARMED

    # Nothing submitted from here on: the link is dead.
    vehicle.run(config.control_timeout_s + 0.1)
    assert vehicle.state is VehicleState.FAILSAFE

    vehicle.run(1.5)
    assert vehicle.state is not VehicleState.ARMED
    assert vehicle.state is not VehicleState.FAILSAFE
    assert vehicle.duty == (0.0, 0.0)
    assert vehicle.rpm[0] == pytest.approx(0.0, abs=2.0)


def test_a_brief_dropout_does_not_cost_the_run(
    vehicle: Vehicle, config: VehicleConfig
) -> None:
    vehicle.arm()
    vehicle.run(SETTLE_S, command={"throttle": 0.6})

    vehicle.run(config.control_timeout_s + 0.05)  # stale, but only just
    state = vehicle.run(0.5, command={"throttle": 0.6})

    assert vehicle.state is VehicleState.ARMED
    assert state.rpm_l > 5.0


def test_the_estop_flag_in_a_packet_stops_the_car(vehicle: Vehicle) -> None:
    vehicle.arm()
    vehicle.run(SETTLE_S, command={"throttle": 0.7})
    vehicle.run(0.5, command={"throttle": 0.7, "flags": ControlFlags.ESTOP})

    assert vehicle.state is VehicleState.ESTOP
    vehicle.run(1.0, command={"throttle": 1.0, "flags": ControlFlags.ESTOP})
    assert vehicle.duty == (0.0, 0.0)


# --------------------------------------------------------------------------
# Protection
# --------------------------------------------------------------------------


def test_the_combined_duty_budget_is_respected_under_full_load(
    vehicle: Vehicle, config: VehicleConfig
) -> None:
    """It is simultaneous demand that trips the boost regulator, so the sum of
    the two duties is the quantity that has to be capped."""
    vehicle.arm()
    for _ in range(400):
        vehicle.submit(throttle=1.0, steering=0.0)
        vehicle.step()
        left, right = vehicle.duty
        assert left + right <= config.duty_sum_max + 1e-6
        assert left <= config.max_duty + 1e-6
        assert right <= config.max_duty + 1e-6


def test_panic_stop_disables_the_bridge_from_any_state(vehicle: Vehicle) -> None:
    vehicle.arm()
    vehicle.run(SETTLE_S, command={"throttle": 0.9})
    vehicle.controller.panic_stop()
    assert vehicle.gpio.outputs_safe
    vehicle.controller.panic_stop()
    assert vehicle.gpio.outputs_safe


def test_a_blocked_wheel_raises_a_stall_and_not_a_brownout(
    vehicle: Vehicle, gpio: MockBackend
) -> None:
    from telekart_protocol import Fault

    vehicle.arm()
    vehicle.run(1.0, command={"throttle": 0.7})
    gpio.block_wheel("left", True)
    vehicle.run(vehicle.config.stall_detect_s + 0.5, command={"throttle": 0.7})

    assert vehicle.safety.faults & Fault.STALL_L
    assert not vehicle.safety.faults & Fault.BROWNOUT


def test_a_dead_encoder_is_noticed(vehicle: Vehicle, gpio: MockBackend) -> None:
    """The wheel keeps turning and nothing counts it. Left undetected, odometry
    quietly halves its distance and the speedometer lies."""
    from telekart_protocol import Fault

    vehicle.arm()
    vehicle.run(1.0, command={"throttle": 0.7})
    gpio.fail_encoder("left", True)
    vehicle.run(vehicle.config.stall_detect_s + 0.5, command={"throttle": 0.7})

    assert vehicle.safety.faults & (Fault.ENCODER_FAIL_L | Fault.STALL_L)


# --------------------------------------------------------------------------
# Telemetry surface
# --------------------------------------------------------------------------


def test_drive_state_carries_everything_telemetry_needs(vehicle: Vehicle) -> None:
    vehicle.arm()
    state = vehicle.run(SETTLE_S, command={"throttle": 0.6, "steering": 0.3})

    assert state.v_max > 0.0, "v_max must come from the measured calibration"
    assert state.speed > 0.0
    assert len(state.pose) == 3
    assert all(math.isfinite(value) for value in state.pose)
    assert state.distance > 0.0
    assert math.isfinite(state.slip)
    assert state.flags & TelemetryFlags.CLOSED_LOOP
    assert state.flags & TelemetryFlags.CALIBRATED


def test_odometry_advances_with_the_wheels(vehicle: Vehicle) -> None:
    vehicle.arm()
    first = vehicle.run(1.0, command={"throttle": 0.7}).distance
    second = vehicle.run(1.0, command={"throttle": 0.7}).distance
    assert second > first > 0.0


# --------------------------------------------------------------------------
# The loop may never raise
# --------------------------------------------------------------------------


def test_garbage_commands_are_clamped_not_rejected(vehicle: Vehicle) -> None:
    """A corrupt axis value must cost one clamped command, not the control
    thread. Nothing in here is allowed to raise."""
    vehicle.arm()
    nonsense = (
        {"throttle": float("nan"), "steering": 0.0},
        {"throttle": 5.0, "steering": -12.0},
        {"throttle": -1.0, "brake": float("inf")},
        {"steering": float("nan"), "brake": -3.0},
    )
    for index in range(200):
        vehicle.submit(**nonsense[index % len(nonsense)])
        vehicle.step()
        left, right = vehicle.duty
        assert 0.0 <= left <= 1.0
        assert 0.0 <= right <= 1.0


def test_a_very_long_tick_does_not_break_the_loop(vehicle: Vehicle) -> None:
    """A missed deadline is survivable; a controller that integrates a 500 ms dt
    into a duty step is not."""
    vehicle.arm()
    vehicle.run(1.0, command={"throttle": 0.5})
    vehicle.submit(throttle=0.5)
    state = vehicle.step(0.5)
    assert math.isfinite(state.duty_l)
    assert abs(state.duty_l) <= 1.0


def test_ticking_without_ever_receiving_a_command_is_harmless(vehicle: Vehicle) -> None:
    for _ in range(500):
        vehicle.clock.advance(CONTROL_PERIOD_S)
        vehicle.gpio.step(CONTROL_PERIOD_S)
        vehicle.controller.tick(CONTROL_PERIOD_S)
    assert vehicle.duty == (0.0, 0.0)
