"""The safety state machine: transitions, the staged failsafe, and fault triage.

Three things here are not conveniences.

**Arming takes four conditions**, and one of them is time -- neutral throttle
held for ``arm_neutral_ms``. A car that arms the instant you ask lets a
half-pressed trigger become motion.

**The failsafe is staged, not a single action.** Coast, then brake, then coast,
then disarm. Braking immediately on a 200 ms WiFi hiccup would be its own
hazard mid-corner; never braking at all lets an unpowered car roll for metres.

**A brownout is not a stall.** Both look like "commanded but not turning" and
the fixes are nothing alike, so they get separate fault codes and the machine
has to tell them apart from the timing alone.
"""

from __future__ import annotations

import math
from typing import Any, Callable

import pytest

from telekart.config import VehicleConfig
from telekart.constants import CONTROL_PERIOD_S
from telekart.control.safety import SafetyOutput, SafetyStateMachine
from telekart.util.clock import FakeClock

from telekart_protocol import (
    FAILSAFE_BRAKE_AT_MS,
    FAILSAFE_COAST_AT_MS,
    FAILSAFE_DISARM_AT_MS,
    Fault,
    VehicleState,
)

MOVING = {"rpm_l": 80.0, "rpm_r": 80.0, "duty_l": 0.5, "duty_r": 0.5}
STOPPED = {"rpm_l": 0.0, "rpm_r": 0.0, "duty_l": 0.0, "duty_r": 0.0}


class Driver:
    """Runs the machine at loop rate so the timing assertions mean something.

    Edge counts are synthesised from the RPM being reported, because the stall
    diagnosis genuinely depends on them: a wheel that has produced no counts at
    all since arming is a wiring fault, and one that was counting a moment ago
    and has stopped is a jam. A driver that always reported zero edges would
    turn every stall test into an encoder-failure test.
    """

    def __init__(
        self,
        safety: SafetyStateMachine,
        clock: FakeClock,
        feedback: Callable[..., Any],
    ) -> None:
        self.safety = safety
        self.clock = clock
        self.feedback = feedback
        self.last: SafetyOutput | None = None
        self.edges_l = 0
        self.edges_r = 0

    def run(
        self,
        seconds: float,
        *,
        throttle: float = 0.0,
        link: bool = True,
        dt: float = CONTROL_PERIOD_S,
        count_edges: bool = True,
        **feedback: float,
    ) -> SafetyOutput:
        steps = max(1, int(round(seconds / dt)))
        cpr = 660
        for _ in range(steps):
            if link:
                self.safety.note_control_packet()
            self.clock.advance(dt)
            if count_edges:
                self.edges_l += int(abs(feedback.get("rpm_l", 0.0)) * cpr * dt / 60.0)
                self.edges_r += int(abs(feedback.get("rpm_r", 0.0)) * cpr * dt / 60.0)
            self.last = self.safety.update(
                dt,
                throttle=throttle,
                measured=self.feedback(
                    edges_l=self.edges_l, edges_r=self.edges_r, **feedback
                ),
            )
        assert self.last is not None
        return self.last

    def arm(self, config: VehicleConfig) -> tuple[bool, str]:
        """All four preconditions, then the request.

        ``set_session_valid`` is the one the control loop cannot satisfy for
        itself: it belongs to the TCP session layer, and without it a car would
        arm for anyone who could reach the UDP port.
        """
        self.safety.set_session_valid(True)
        self.run(config.arm_neutral_s + 0.2, throttle=0.0, **STOPPED)
        accepted, reason = self.safety.request_arm()
        if accepted:
            self.run(2 * CONTROL_PERIOD_S, throttle=0.0, **STOPPED)
        return accepted, reason


@pytest.fixture
def driver(
    safety: SafetyStateMachine, clock: FakeClock, feedback: Callable[..., Any]
) -> Driver:
    return Driver(safety, clock, feedback)


# --------------------------------------------------------------------------
# Resting states
# --------------------------------------------------------------------------


def test_a_fresh_machine_does_not_allow_drive(
    safety: SafetyStateMachine, driver: Driver
) -> None:
    assert safety.state in (VehicleState.BOOT, VehicleState.SAFE)
    output = driver.run(0.05, **STOPPED)
    assert safety.state is VehicleState.SAFE
    assert not output.allow_drive
    assert output.force_coast
    assert output.force_brake == pytest.approx(0.0)


def test_disarmed_is_not_a_fault(safety: SafetyStateMachine, driver: Driver) -> None:
    driver.run(0.5, **STOPPED)
    assert safety.faults == Fault.NONE


# --------------------------------------------------------------------------
# Arming
# --------------------------------------------------------------------------


def test_arming_requires_a_session(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """Reaching the UDP port is not authorisation. The session handshake is."""
    driver.run(config.arm_neutral_s + 0.2, throttle=0.0, **STOPPED)
    accepted, reason = safety.request_arm()
    assert not accepted
    assert "session" in reason.lower()


def test_losing_the_session_while_armed_disarms(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """The operator's laptop closed its lid. Waiting for the failsafe to notice
    leaves the car armed for a second longer than anyone intended."""
    driver.arm(config)
    safety.set_session_valid(False)
    driver.run(0.05, **STOPPED)
    assert safety.state is not VehicleState.ARMED


def test_arming_requires_neutral_throttle_to_have_been_held(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    safety.set_session_valid(True)
    driver.run(config.arm_neutral_s + 0.2, throttle=0.9, **STOPPED)
    accepted, reason = safety.request_arm()
    assert not accepted
    assert reason, "a refusal has to say why; the app shows this string"
    assert safety.state is not VehicleState.ARMED


def test_arming_succeeds_once_neutral_has_been_held_long_enough(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    accepted, reason = driver.arm(config)
    assert accepted, reason
    assert safety.state is VehicleState.ARMED

    output = driver.run(0.05, throttle=0.3, **MOVING)
    assert output.allow_drive
    assert output.duty_ceiling > 0.0
    assert output.duty_ceiling <= config.max_duty + 1e-9


def test_arming_is_refused_while_a_critical_fault_is_active(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    safety.set_session_valid(True)
    safety.raise_fault(Fault.OVERTEMP, "bridge at 85C")
    driver.run(config.arm_neutral_s + 0.2, throttle=0.0, **STOPPED)
    accepted, reason = safety.request_arm()
    assert not accepted
    assert reason


def test_a_non_critical_fault_does_not_block_arming(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """LOW_BATTERY is a warning. Refusing to arm on it would strand the car."""
    safety.raise_fault(Fault.LOW_BATTERY, "5.9 V")
    accepted, reason = driver.arm(config)
    assert accepted, reason


def test_disarm_returns_to_safe(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    safety.request_disarm()
    output = driver.run(0.05, **STOPPED)
    assert safety.state is VehicleState.SAFE
    assert not output.allow_drive


def test_a_critical_fault_while_armed_forces_a_disarm(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    safety.raise_fault(Fault.GPIO_ERROR, "pigpio write failed")
    output = driver.run(0.05, throttle=0.5, **MOVING)
    assert safety.state is not VehicleState.ARMED
    assert not output.allow_drive


# --------------------------------------------------------------------------
# E-stop
# --------------------------------------------------------------------------


def test_estop_latches_and_stops_the_car(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    safety.request_estop()
    output = driver.run(0.05, throttle=0.8, **MOVING)

    assert safety.state is VehicleState.ESTOP
    assert safety.faults & Fault.ESTOP_LATCHED
    assert not output.allow_drive
    assert output.force_brake > 0.0 or output.force_coast


def test_estop_survives_further_commands(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """Latched means latched: a full-throttle packet arriving afterwards changes
    nothing until a human clears it."""
    driver.arm(config)
    safety.request_estop()
    output = driver.run(1.0, throttle=1.0, **MOVING)
    assert safety.state is VehicleState.ESTOP
    assert not output.allow_drive


def test_clearing_estop_never_lands_straight_back_in_armed(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    safety.request_estop()
    driver.run(0.1, throttle=0.0, **STOPPED)

    accepted, reason = safety.clear_estop()
    driver.run(0.05, throttle=0.0, **STOPPED)
    if accepted:
        assert safety.state is not VehicleState.ARMED
        assert not safety.faults & Fault.ESTOP_LATCHED
    else:
        assert reason
        assert safety.state is VehicleState.ESTOP


# --------------------------------------------------------------------------
# Staged failsafe
# --------------------------------------------------------------------------


def stale_for(driver: Driver, config: VehicleConfig, milliseconds: float) -> SafetyOutput:
    """Advance to ``milliseconds`` past the moment the link went stale.

    Stale begins when ``control_timeout_ms`` expires, not when the last packet
    arrived -- otherwise FAILSAFE_BRAKE_AT_MS (50) would fall inside the 200 ms
    timeout and the car would brake before it had decided anything was wrong.
    """
    driver.run(config.control_timeout_s + milliseconds / 1000.0, link=False, **MOVING)
    assert driver.last is not None
    return driver.last


def test_a_live_link_is_not_a_failsafe(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    output = driver.run(2.0, throttle=0.4, link=True, **MOVING)
    assert safety.state is VehicleState.ARMED
    assert output.allow_drive
    assert not output.force_coast
    assert output.force_brake == pytest.approx(0.0)


def test_failsafe_starts_by_coasting(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """A 200 ms WiFi hiccup on a Pi Zero 2 W is ordinary. Slamming the brakes on
    it, mid-corner, is a hazard of its own."""
    driver.arm(config)
    output = stale_for(driver, config, FAILSAFE_BRAKE_AT_MS * 0.4)

    assert safety.state is VehicleState.FAILSAFE
    assert not output.allow_drive
    assert output.force_coast
    assert output.force_brake == pytest.approx(0.0)
    # Not a fault yet: a hiccup that recovers inside a second never happened as
    # far as the fault register is concerned.
    assert not safety.faults & Fault.CONTROL_TIMEOUT


def test_failsafe_then_brakes(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """Coasting alone is not enough -- an unbraked car keeps rolling for metres."""
    driver.arm(config)
    midpoint = (FAILSAFE_BRAKE_AT_MS + FAILSAFE_COAST_AT_MS) / 2.0
    output = stale_for(driver, config, midpoint)

    assert safety.state is VehicleState.FAILSAFE
    assert output.force_brake == pytest.approx(config.failsafe_brake_duty, rel=0.01)
    assert not output.allow_drive


def test_failsafe_releases_the_brake_before_disarming(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """A shorted bridge held for a second is heat the L298N's stock heatsink
    cannot shed."""
    driver.arm(config)
    midpoint = (FAILSAFE_COAST_AT_MS + FAILSAFE_DISARM_AT_MS) / 2.0
    output = stale_for(driver, config, midpoint)

    assert output.force_coast
    assert output.force_brake == pytest.approx(0.0)


def test_failsafe_finally_disarms(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    output = stale_for(driver, config, FAILSAFE_DISARM_AT_MS + 100)

    assert safety.state is not VehicleState.ARMED
    assert safety.state is not VehicleState.FAILSAFE
    assert not output.allow_drive
    assert safety.faults & Fault.CONTROL_TIMEOUT


def test_a_packet_arriving_mid_failsafe_recovers(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """The link coming back before the disarm deadline must not require a
    re-arm. Losing 400 ms of WiFi should not cost you the run."""
    driver.arm(config)
    stale_for(driver, config, FAILSAFE_BRAKE_AT_MS + 20)
    assert safety.state is VehicleState.FAILSAFE

    output = driver.run(0.2, throttle=0.2, link=True, **MOVING)
    assert safety.state is VehicleState.ARMED
    assert output.allow_drive


def test_failsafe_does_not_fire_when_disarmed(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """A car sitting in SAFE with nobody connected is not in a failsafe. It is
    just parked, and the HUD must not scream about it."""
    driver.run(3.0, link=False, **STOPPED)
    assert safety.state is not VehicleState.FAILSAFE


# --------------------------------------------------------------------------
# Stall and brownout
# --------------------------------------------------------------------------


def test_one_wheel_stalling_raises_only_that_wheels_fault(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """Hardware protection, not a nicety: a stalled motor puts several watts
    into a bridge whose heatsink handles two or three."""
    driver.arm(config)
    driver.run(0.3, throttle=0.6, **MOVING)
    driver.run(
        config.stall_detect_s + 0.3,
        throttle=0.6,
        rpm_l=0.0,
        rpm_r=80.0,
        duty_l=0.6,
        duty_r=0.6,
    )

    assert safety.faults & Fault.STALL_L
    assert not safety.faults & Fault.STALL_R
    assert not safety.faults & Fault.BROWNOUT


def test_a_wheel_that_never_counted_is_a_wiring_fault_not_a_jam(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """No current sensor on this car, so the discriminator is history.

    An encoder that has produced no edges at all since arming while its partner
    spins is a disconnected wire. Calling that a stall would stop the car dead
    in the middle of a track for a fault that does not need it to.
    """
    driver.arm(config)
    driver.run(
        config.stall_detect_s + 0.3,
        throttle=0.6,
        rpm_l=0.0,
        rpm_r=80.0,
        duty_l=0.6,
        duty_r=0.6,
        count_edges=False,
    )

    assert safety.faults & Fault.ENCODER_FAIL_L
    assert not safety.faults & Fault.STALL_L


def test_a_commanded_wheel_that_turns_is_not_a_stall(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    driver.run(config.stall_detect_s + 0.5, throttle=0.6, **MOVING)
    assert not safety.faults & Fault.STALL_L
    assert not safety.faults & Fault.STALL_R


def test_a_wheel_at_rest_with_no_command_is_not_a_stall(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    driver.arm(config)
    driver.run(config.stall_detect_s + 0.5, throttle=0.0, **STOPPED)
    assert not safety.faults & Fault.STALL_L


def test_both_wheels_dying_together_is_a_brownout_not_a_stall(
    safety: SafetyStateMachine, driver: Driver, config: VehicleConfig
) -> None:
    """The regulator sustains ~1.5 A for both motors and folds back into hiccup
    mode above that. Both wheels go to zero in the same instant, which is a
    signature no mechanical stall produces."""
    driver.arm(config)
    driver.run(0.5, throttle=0.8, **MOVING)
    driver.run(
        config.stall_detect_s + 0.3,
        throttle=0.8,
        rpm_l=0.0,
        rpm_r=0.0,
        duty_l=0.7,
        duty_r=0.7,
    )

    assert safety.faults & Fault.BROWNOUT
    assert not safety.faults & (Fault.STALL_L | Fault.STALL_R)


# --------------------------------------------------------------------------
# Fault bookkeeping
# --------------------------------------------------------------------------


def test_faults_are_sticky_until_cleared(
    safety: SafetyStateMachine, driver: Driver
) -> None:
    safety.raise_fault(Fault.ENCODER_FAIL_L, "no edges for 2 s")
    driver.run(1.0, **MOVING)
    assert safety.faults & Fault.ENCODER_FAIL_L

    safety.clear_faults()
    driver.run(0.05, **MOVING)
    assert not safety.faults & Fault.ENCODER_FAIL_L


def test_raising_the_same_fault_twice_is_harmless(safety: SafetyStateMachine) -> None:
    safety.raise_fault(Fault.STALL_L, "first")
    safety.raise_fault(Fault.STALL_L, "again")
    assert safety.faults & Fault.STALL_L


# --------------------------------------------------------------------------
# Robustness -- this runs inside the control loop
# --------------------------------------------------------------------------


def test_update_never_raises_on_nonsense_input(
    safety: SafetyStateMachine, clock: FakeClock, feedback: Callable[..., Any]
) -> None:
    for throttle in (float("nan"), float("inf"), -5.0, 12.0):
        for dt in (0.0, -0.01, 10.0):
            clock.advance(0.001)
            output = safety.update(
                dt, throttle=throttle, measured=feedback(rpm_l=math.nan, rpm_r=1e9)
            )
            assert 0.0 <= output.duty_ceiling <= 1.0
            assert math.isfinite(output.force_brake)


def test_output_is_immutable(safety: SafetyStateMachine, driver: Driver) -> None:
    output = driver.run(0.05, **STOPPED)
    with pytest.raises(Exception):
        output.allow_drive = True  # type: ignore[misc]
