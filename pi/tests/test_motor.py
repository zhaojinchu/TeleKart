"""H-bridge driving, and above all the truth table.

On an L298 with the enable high, ``IN1 == IN2`` -- both high **or** both low --
is a BRAKE. Coasting requires dropping the enable. That is the opposite of most
people's intuition, and getting it backwards means every time you meant to let
the car roll you instead short a spinning motor into the bridge. The pin-state
tests below are the cheap check; ``test_braking_stops_the_wheel_faster_than
_coasting`` is the one that cannot be satisfied by a wrong implementation whose
assertions were "fixed" to match it.
"""

from __future__ import annotations

import math

import pytest

from telekart.config import VehicleConfig
from telekart.drivers.motor import MotorPair
from telekart.hal.mock_backend import DriveMode, MockBackend, MockPins, PlantParams
from telekart.util.clock import FakeClock


def build(**params: object) -> tuple[MockBackend, VehicleConfig, MotorPair, FakeClock]:
    """A motor pair on its own mock, for the tests that need a modified config."""
    clock = FakeClock(start=1000.0)
    config = VehicleConfig()
    for name, value in params.items():
        setattr(config, name, value)
    gpio = MockBackend(
        pins=MockPins.from_hardware_pins(config.pins),
        params=PlantParams(cpr=config.encoder_cpr),
        clock=clock,
        strict=True,
    )
    return gpio, config, MotorPair(gpio, config.pins.motors, config, clock), clock


def ramp(
    motors: MotorPair,
    clock: FakeClock,
    duty_l: float,
    duty_r: float,
    *,
    ticks: int = 20,
    dt: float = 0.010,
) -> None:
    """Hold a command for ``ticks`` control periods.

    Nothing reaches its commanded duty in one call, by design: the applied duty
    rises at 20 duty/s so a corrupt command cannot step the bridge from rest to
    full rail in a single tick, and a fresh direction has to wait out the
    dead time first. Tests that want the steady-state duty have to run the loop.
    """
    for _ in range(ticks):
        motors.drive(duty_l, duty_r)
        clock.advance(dt)
    motors.drive(duty_l, duty_r)


def left_mode(gpio: MockBackend) -> DriveMode:
    """What the plant makes of the current pin state.

    Read through the plant rather than by inspecting pins directly because the
    plant implements the L298 truth table independently -- if the driver and the
    model disagree, one of them is wrong and the test says so.
    """
    gpio.step(1e-6)
    return gpio.plant.left.mode


def right_mode(gpio: MockBackend) -> DriveMode:
    gpio.step(1e-6)
    return gpio.plant.right.mode


# --------------------------------------------------------------------------
# Construction
# --------------------------------------------------------------------------


def test_construction_leaves_the_bridge_disabled(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig
) -> None:
    """GPIO5 and GPIO6 idle HIGH on this SoC, so 'nothing has been written yet'
    is not a safe state -- with the enable up it is a brake."""
    pins = config.pins.motors
    assert gpio.pwm_duty(pins.ena) == 0.0
    assert gpio.pwm_duty(pins.enb) == 0.0
    for pin in (pins.in1, pins.in2, pins.in3, pins.in4):
        assert gpio.pin_mode(pin) == "out"
        assert gpio.pin_level(pin) is False
    assert left_mode(gpio) is DriveMode.COAST
    assert right_mode(gpio) is DriveMode.COAST


# --------------------------------------------------------------------------
# The truth table
# --------------------------------------------------------------------------


def test_coast_drops_the_enable(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    ramp(motors, clock, 0.5, 0.5)
    motors.coast()
    pins = config.pins.motors
    assert gpio.pwm_duty(pins.ena) == 0.0
    assert gpio.pwm_duty(pins.enb) == 0.0
    assert left_mode(gpio) is DriveMode.COAST
    assert right_mode(gpio) is DriveMode.COAST


def test_brake_holds_the_enable_and_ties_the_in_pins_together(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig
) -> None:
    """The inversion catcher. EN stays up at ``strength``; the IN pair is EQUAL."""
    motors.brake(0.4)
    pins = config.pins.motors

    assert gpio.pwm_duty(pins.ena) == pytest.approx(0.4)
    assert gpio.pwm_duty(pins.enb) == pytest.approx(0.4)
    assert gpio.pin_level(pins.in1) == gpio.pin_level(pins.in2)
    assert gpio.pin_level(pins.in3) == gpio.pin_level(pins.in4)
    assert left_mode(gpio) is DriveMode.BRAKE
    assert right_mode(gpio) is DriveMode.BRAKE


def test_drive_forward_splits_the_in_pins(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    ramp(motors, clock, 0.5, 0.5)
    pins = config.pins.motors
    assert gpio.pin_level(pins.in1) != gpio.pin_level(pins.in2)
    assert gpio.pin_level(pins.in3) != gpio.pin_level(pins.in4)
    assert gpio.pwm_duty(pins.ena) == pytest.approx(0.5)
    assert left_mode(gpio) is DriveMode.FORWARD
    assert right_mode(gpio) is DriveMode.FORWARD


def test_drive_reverse_flips_the_in_pair(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    ramp(motors, clock, 0.5, 0.5)
    forward = (gpio.pin_level(config.pins.motors.in1), gpio.pin_level(config.pins.motors.in2))
    motors.coast()
    ramp(motors, clock, -0.5, -0.5)
    reverse = (gpio.pin_level(config.pins.motors.in1), gpio.pin_level(config.pins.motors.in2))
    assert reverse == (not forward[0], not forward[1])
    assert left_mode(gpio) is DriveMode.REVERSE


def test_braking_stops_the_wheel_faster_than_coasting(config: VehicleConfig) -> None:
    """The physical proof, independent of any assertion about pin levels.

    Spin a wheel up, then release it two ways. Coasting is an open winding and
    the wheel free-wheels down against friction alone; braking shorts it and the
    back-EMF does the work. If these come out the same -- or the wrong way
    round -- the IN pins are crossed and no pin-level assertion will tell you.
    """
    def spin_up() -> tuple[MockBackend, MotorPair, FakeClock]:
        gpio, _config, motors, clock = build()
        for _ in range(200):
            motors.drive(0.8, 0.8)
            clock.advance(0.005)
            gpio.step(0.005)
        return gpio, motors, clock

    def release(gpio: MockBackend, clock: FakeClock) -> float:
        for _ in range(60):  # 300 ms
            clock.advance(0.005)
            gpio.step(0.005)
        return gpio.wheel_rpm("left")

    gpio_coast, motors_coast, clock_coast = spin_up()
    peak = gpio_coast.wheel_rpm("left")
    assert peak > 50.0, "the plant never span up; the rest of this test proves nothing"
    motors_coast.coast()
    coasted = release(gpio_coast, clock_coast)

    gpio_brake, motors_brake, clock_brake = spin_up()
    motors_brake.brake(1.0)
    braked = release(gpio_brake, clock_brake)

    # Crossed IN pins swap the two behaviours outright, so the ratio would come
    # out well above 1 rather than a little high. There is no near miss here.
    assert braked < coasted * 0.75, (
        f"brake left {braked:.1f} RPM and coast left {coasted:.1f} from a "
        f"{peak:.1f} RPM start; IN1/IN2 are almost certainly crossed"
    )
    assert coasted - braked > 0.15 * peak


# --------------------------------------------------------------------------
# Limits
# --------------------------------------------------------------------------


def test_per_motor_duty_is_clamped_to_max_duty(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    ramp(motors, clock, 1.0, 0.0)
    assert gpio.pwm_duty(config.pins.motors.ena) == pytest.approx(config.max_duty)
    assert motors.limiter_active


def test_combined_duty_budget_scales_both_motors(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    """The regulator sustains ~1.5 A for BOTH motors, and it is simultaneous
    demand that trips it. Budgeting the sum is the only lever the firmware has."""
    ramp(motors, clock, 0.9, 0.9)
    pins = config.pins.motors
    left = gpio.pwm_duty(pins.ena)
    right = gpio.pwm_duty(pins.enb)

    assert left + right <= config.duty_sum_max + 1e-9
    # Scaled, not truncated: the ratio between the wheels must survive, or the
    # electronic differential is undone by the protection limiter.
    assert left == pytest.approx(right)
    assert motors.limiter_active


def test_the_budget_preserves_the_differential_ratio(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    ramp(motors, clock, 0.85, 0.60)
    pins = config.pins.motors
    left = gpio.pwm_duty(pins.ena)
    right = gpio.pwm_duty(pins.enb)
    assert left + right <= config.duty_sum_max + 1e-9
    assert left / right == pytest.approx(0.85 / 0.60, rel=0.02)


def test_an_ordinary_command_is_not_limited(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    """LIMITER_ACTIVE during normal driving means the HUD and the car disagree,
    which feels awful. It must only light when something really was clipped."""
    ramp(motors, clock, 0.5, 0.5)
    assert gpio.pwm_duty(config.pins.motors.ena) == pytest.approx(0.5)
    assert not motors.limiter_active


def test_inversion_flips_one_side_only() -> None:
    gpio, config, motors, clock = build(invert_left=True)
    ramp(motors, clock, 0.5, 0.5)
    assert left_mode(gpio) is DriveMode.REVERSE
    assert right_mode(gpio) is DriveMode.FORWARD


def test_inversion_is_applied_once_and_only_at_the_pins() -> None:
    """Inversion is a fact about how the leads were soldered, so it belongs at
    the pin write and nowhere else.

    Applying it a second time anywhere above the bridge cancels it out and the
    car simply drives the wrong way -- but it also, more quietly, puts the sign
    of the commanded duty and the sign of the measured RPM into different
    frames, and the dead-time sequencer compares exactly those two to decide
    whether a reversal would be a plugging brake.

    Everything MotorPair reports must therefore stay in the vehicle frame:
    positive is forward for the *car*, on both sides, however each motor is
    wired. This is what the encoder direction hint, the stall detector and the
    telemetry packet all consume.
    """
    gpio, _config, motors, clock = build(invert_left=True)
    ramp(motors, clock, 0.5, 0.5)

    # Same sign on both sides, because the vehicle is going forward on both.
    assert motors.applied_duty_l > 0.0
    assert motors.applied_duty_r > 0.0
    assert motors.direction_l == 1
    assert motors.direction_r == 1
    # ...while the pins disagree, which is the whole point of the flag.
    assert left_mode(gpio) is DriveMode.REVERSE
    assert right_mode(gpio) is DriveMode.FORWARD

    # And an uninverted pair reports identically, so nothing downstream can
    # tell the two builds apart.
    gpio2, _c2, motors2, clock2 = build()
    ramp(motors2, clock2, 0.5, 0.5)
    assert motors2.applied_duty_l == pytest.approx(motors.applied_duty_l)
    assert motors2.applied_duty_r == pytest.approx(motors.applied_duty_r)


def test_an_inverted_motor_still_refuses_to_plug_a_spinning_wheel() -> None:
    """The reverse-plugging guard compares commanded sign against measured RPM.
    If inversion had been applied above the bridge those two would be in
    different frames, and this guard would wave through exactly the case it
    exists to catch."""
    gpio, config, motors, clock = build(invert_left=True)

    # Wheels genuinely rolling forward, well above the reversal threshold.
    for _ in range(200):
        motors.drive(0.8, 0.8)
        clock.advance(0.005)
        gpio.step(0.005)

    # Mind the frame. The plant turns its shaft in the *pin* sense, so an
    # inverted motor driving the car forward reads negative here. On the real
    # vehicle the encoder is what converts that back: a car wired with
    # invert_left also sets encoder_invert_left, and note_speed() is fed from
    # the encoder sample, never from the raw shaft. Mirror that conversion.
    plant_l = gpio.wheel_rpm("left")
    plant_r = gpio.wheel_rpm("right")
    vehicle_l = -plant_l if config.invert_left else plant_l
    vehicle_r = -plant_r if config.invert_right else plant_r
    assert vehicle_l > config.reverse_allowed_rpm
    assert vehicle_r > config.reverse_allowed_rpm

    # Ask for full reverse. The bridge must not flip while it is still rolling.
    motors.note_speed(vehicle_l, vehicle_r)
    motors.drive(-0.8, -0.8)
    assert motors.direction_l == 1, (
        "the inverted side flipped direction under speed -- that is a plugging "
        "brake drawing locked-rotor current from a 1.5 A converter"
    )
    assert gpio.pwm_duty(config.pins.motors.ena) == 0.0


def test_pwm_is_written_as_a_pair_at_the_configured_frequency(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    """Both channels share one clock divider. The mock is strict about this, so
    a driver that wrote them separately would already have raised."""
    ramp(motors, clock, 0.3, 0.7)
    assert gpio.pwm_freq == config.pwm_hz

    pwm_ops = gpio.ops("pwm")
    assert pwm_ops, "no PWM was written at all"
    pins = config.pins.motors
    for index in range(0, len(pwm_ops) - 1, 2):
        assert pwm_ops[index].pin == pins.ena
        assert pwm_ops[index + 1].pin == pins.enb
        assert pwm_ops[index].t_us == pwm_ops[index + 1].t_us


# --------------------------------------------------------------------------
# Direction dead time
# --------------------------------------------------------------------------


def test_direction_change_is_sequenced_with_dead_time() -> None:
    """Never flip the IN pins under a live enable.

    The plant is deliberately not stepped here: the wheels stay at rest, so the
    reverse-threshold precondition is satisfied and what is left under test is
    the sequence itself -- duty to zero, wait, flip, ramp.
    """
    gpio, config, motors, clock = build()
    deadtime_us = int(config.direction_deadtime_ms * 1000)
    pins = config.pins.motors

    ramp(motors, clock, 0.5, 0.5)
    forward_in1 = gpio.pin_level(pins.in1)

    flip_us: int | None = None
    for _ in range(40):  # 400 ms, comfortably past a 30 ms dead time
        clock.advance(0.010)
        motors.drive(-0.5, -0.5)
        if flip_us is None and gpio.pin_level(pins.in1) != forward_in1:
            flip_us = clock.monotonic_us()

    assert flip_us is not None, "the direction pins never flipped"

    # Strictly before the flip: the flip tick's own enable write happens after
    # the pins have already moved, which is the correct order and not a
    # violation. What must never appear is live duty in the run-up to it.
    window = [
        op
        for op in gpio.ops("pwm")
        if op.pin == pins.ena and flip_us - deadtime_us <= op.t_us < flip_us
    ]
    assert window, "the flip happened with no dead-time window at all"
    assert all(op.value == 0.0 for op in window), (
        "the IN pins moved while the bridge was still enabled -- that is a "
        "shoot-through across the H-bridge"
    )

    # And afterwards it drives again rather than sulking at zero forever.
    ramp(motors, clock, -0.5, -0.5)
    assert gpio.pwm_duty(pins.ena) > 0.0
    assert left_mode(gpio) is DriveMode.REVERSE


def test_no_dead_time_when_the_direction_does_not_change() -> None:
    gpio, config, motors, clock = build()
    ramp(motors, clock, 0.4, 0.4)
    ramp(motors, clock, 0.6, 0.6, ticks=3)
    assert gpio.pwm_duty(config.pins.motors.ena) == pytest.approx(0.6)


# --------------------------------------------------------------------------
# Panic stop
# --------------------------------------------------------------------------


def test_panic_stop_disables_both_bridges_and_is_idempotent(
    gpio: MockBackend, motors: MotorPair, clock: FakeClock
) -> None:
    ramp(motors, clock, 0.8, 0.8)
    motors.panic_stop()
    assert gpio.outputs_safe
    motors.panic_stop()
    motors.panic_stop()
    assert gpio.outputs_safe


def test_panic_stop_latches_against_a_later_command(
    gpio: MockBackend, motors: MotorPair, clock: FakeClock
) -> None:
    """pigpiod outlives this process and holds whatever it was last told, so a
    panic stop one stale command could undo would not be a panic stop."""
    ramp(motors, clock, 0.8, 0.8)
    motors.panic_stop()
    ramp(motors, clock, 0.8, 0.8)
    assert gpio.outputs_safe
    assert motors.panicked

    motors.clear_panic()
    ramp(motors, clock, 0.5, 0.5)
    assert gpio.pwm_duty(gpio.pins.ena) > 0.0


def test_panic_stop_survives_a_backend_that_has_already_been_cleaned_up(
    gpio: MockBackend, motors: MotorPair, clock: FakeClock
) -> None:
    """It is wired to atexit and to signal handlers, so it will be called after
    cleanup at least once in the life of every process. It may not raise."""
    ramp(motors, clock, 0.6, 0.6)
    gpio.cleanup()
    motors.panic_stop()
    assert gpio.outputs_safe


# --------------------------------------------------------------------------
# Hot-path robustness
# --------------------------------------------------------------------------


def test_absurd_duties_are_clamped_rather_than_rejected(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    ramp(motors, clock, 12.0, -9.0)
    pins = config.pins.motors
    assert 0.0 <= gpio.pwm_duty(pins.ena) <= config.max_duty
    assert 0.0 <= gpio.pwm_duty(pins.enb) <= config.max_duty


def test_a_nan_duty_never_reaches_the_hardware(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig
) -> None:
    """A NaN that gets as far as the PWM register is a duty nobody chose. The
    control path clamps and flags; it does not raise."""
    motors.drive(float("nan"), 0.5)
    pins = config.pins.motors
    assert not math.isnan(gpio.pwm_duty(pins.ena))
    assert not math.isnan(gpio.pwm_duty(pins.enb))


def test_repeated_identical_commands_do_not_rewrite_the_direction_pins(
    gpio: MockBackend, motors: MotorPair, config: VehicleConfig, clock: FakeClock
) -> None:
    """A pigpio write is a socket round trip. Four of them a tick for pins that
    have not moved is 400 pointless round trips a second on a busy core."""
    ramp(motors, clock, 0.4, 0.4)
    before = len(gpio.ops("write"))
    for _ in range(20):
        motors.drive(0.4, 0.4)
        clock.advance(0.010)
    assert len(gpio.ops("write")) == before
