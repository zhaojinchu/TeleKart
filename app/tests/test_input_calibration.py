"""Calibration, mapping, sources and profile storage.

The calibration machine is the piece with the most branches and the least
tolerance for being wrong: everything downstream trusts the numbers it produces,
and a driver only finds out it guessed badly by driving into something. It is
also, deliberately, free of Qt and SDL -- so a complete calibration run,
including the wiggle detection and the combined-pedal diagnosis, is simulated
here against synthetic device snapshots in a few milliseconds.

Runs under pytest, and standalone (`python3 test_input_calibration.py`).
"""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

_HERE = Path(__file__).resolve()
for _candidate in (
    _HERE.parents[1],  # app/
    _HERE.parents[2] / "packages" / "telekart_protocol",
):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from telekart_app.input.calibration import (  # noqa: E402
    DEADZONE_CAPTURE_S,
    IDLE_CAPTURE_S,
    RANGE_CAPTURE_S,
    SATURATION_CAPTURE_S,
    WIGGLE_CAPTURE_S,
    CalibrationMachine,
    CalStep,
    summarize,
)
from telekart_app.input.chain import InputChain  # noqa: E402
from telekart_app.input.curves import CurveSpec  # noqa: E402
from telekart_app.input.mapping import (  # noqa: E402
    ACTION_FLAGS,
    Action,
    ActionState,
    Control,
    ControlBinding,
    EdgeTracker,
    InputMap,
    InputRef,
    RefKind,
    action_bit,
    button,
    hat,
    resolve_binding,
)
from telekart_app.input.profile import (  # noqa: E402
    DeviceProfile,
    ProfileStore,
    identity_profile,
    preset_keyboard,
)
from telekart_app.input.sources import (  # noqa: E402
    DeviceSnapshot,
    JoystickSource,
    KeyboardSource,
    NullSource,
    RawSample,
    ReplaySource,
    ScriptSource,
    write_jsonl,
)
from telekart_protocol.constants import ControlFlags  # noqa: E402

TICK = 0.01


class FakeClock:
    __slots__ = ("t",)

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class FakeDevice:
    """A joystick made of numbers. Satisfies DeviceInfo and DeviceReader."""

    def __init__(
        self,
        *,
        guid: str = "fake-guid",
        name: str = "Fake Wheel",
        num_axes: int = 3,
        num_buttons: int = 12,
        num_hats: int = 1,
        rest: tuple[float, ...] | None = None,
    ) -> None:
        self.guid = guid
        self.name = name
        self.num_axes = num_axes
        self.num_buttons = num_buttons
        self.num_hats = num_hats
        self.axes = list(rest) if rest is not None else [0.0] * num_axes
        self.buttons = 0
        self.hats = [(0, 0)] * num_hats
        self.pressed_edges = 0
        self.connected = True
        self._generation = 0

    @property
    def snapshot(self) -> DeviceSnapshot:
        self._generation += 1
        edges = self.pressed_edges
        self.pressed_edges = 0
        return DeviceSnapshot(
            t=0.0,
            generation=self._generation,
            axes=tuple(self.axes),
            buttons=self.buttons,
            button_count=self.num_buttons,
            hats=tuple(self.hats),
            pressed_edges=edges,
            connected=self.connected,
        )

    def press(self, index: int) -> None:
        self.buttons |= 1 << index
        self.pressed_edges |= 1 << index

    def release(self, index: int) -> None:
        self.buttons &= ~(1 << index)


def _run_step(
    machine: CalibrationMachine,
    device: FakeDevice,
    clock: FakeClock,
    shape,
    duration: float,
    *,
    limit: float | None = None,
) -> None:
    """Feed the machine until it leaves the current step.

    `shape(u, elapsed)` positions the device, with u running 0..1 across the
    step's own capture window -- normalised against the step duration and not
    against the loop's bail-out, so a sweep genuinely reaches its endpoints
    before the machine stops looking.
    """
    stop_after = duration + 1.0 if limit is None else limit
    start_step = machine.step
    start_t = clock.t
    while machine.step is start_step and clock.t - start_t < stop_after:
        elapsed = clock.t - start_t
        shape(min(elapsed / duration, 1.0), elapsed)
        machine.feed(device.snapshot)
        clock.advance(TICK)


def _analog_wheel() -> FakeDevice:
    """Three axes: steering centred, two pedals idling at +1 like a Logitech."""
    return FakeDevice(
        guid="analog-wheel",
        name="Fake Racing Wheel",
        num_axes=6,
        num_buttons=12,
        rest=(0.0, 0.0, 1.0, 0.0, 0.0, 1.0),
    )


# --------------------------------------------------------------------------
# A whole calibration, end to end
# --------------------------------------------------------------------------


def _calibrate_analog(clock: FakeClock, device: FakeDevice) -> CalibrationMachine:
    machine = CalibrationMachine(clock=clock)
    assert machine.step is CalStep.DETECT
    assert not machine.can_advance

    machine.attach_device(device)
    assert machine.can_advance
    assert machine.advance()
    assert machine.step is CalStep.IDLE

    def idle(_u: float, _t: float) -> None:
        return None

    _run_step(machine, device, clock, idle, IDLE_CAPTURE_S)
    assert machine.step is CalStep.ASSIGN_STEER
    report = machine.idle_report
    assert report.samples > 100
    # Both pedals idle at one end of travel, which is how they get found.
    assert set(report.resting_at_extreme) == {2, 5}

    def steer_wiggle(u: float, _t: float) -> None:
        # Left first, then right: the order is what gives the sign.
        if u < 0.4:
            device.axes[0] = -u / 0.4
        elif u < 0.8:
            device.axes[0] = (u - 0.4) / 0.4 * 2.0 - 1.0
        else:
            device.axes[0] = 1.0 - (u - 0.8) / 0.2

    _run_step(machine, device, clock, steer_wiggle, WIGGLE_CAPTURE_S)
    assert machine.step is CalStep.ASSIGN_THROTTLE
    steer = machine.assignments[Control.STEER]
    assert steer.ref.kind is RefKind.AXIS and steer.ref.index == 0
    assert not steer.invert
    assert steer.confident

    device.axes[0] = 0.0

    def pedal(index: int):
        def shape(u: float, _t: float) -> None:
            device.axes[index] = 1.0 - 2.0 * (u if u < 0.5 else 1.0 - u) * 2.0

        return shape

    _run_step(machine, device, clock, pedal(2), WIGGLE_CAPTURE_S)
    assert machine.step is CalStep.ASSIGN_BRAKE
    throttle = machine.assignments[Control.THROTTLE]
    assert throttle.ref.index == 2
    assert abs(throttle.rest - 1.0) < 0.05
    assert throttle.extreme < -0.9

    device.axes[2] = 1.0
    _run_step(machine, device, clock, pedal(5), WIGGLE_CAPTURE_S)
    assert machine.step is CalStep.RANGES
    brake = machine.assignments[Control.BRAKE]
    assert brake.ref.index == 5
    device.axes[5] = 1.0

    def sweep(u: float, _t: float) -> None:
        # Square rather than a ramp: the endpoints are held, so the capture sees
        # the true extremes regardless of where the sample times land.
        device.axes[0] = -1.0 if (u < 0.25 or 0.5 <= u < 0.75) else 1.0
        pedal_value = 1.0 if (u < 0.25 or 0.5 <= u < 0.75) else -1.0
        device.axes[2] = pedal_value
        device.axes[5] = pedal_value

    _run_step(machine, device, clock, sweep, RANGE_CAPTURE_S)
    assert machine.step is CalStep.DEADZONE

    noise = [0.0]

    def jitter(_u: float, _t: float) -> None:
        device.axes[0] = 0.004 if noise[0] % 2 else -0.004
        device.axes[2] = 1.0
        device.axes[5] = 1.0
        noise[0] += 1

    _run_step(machine, device, clock, jitter, DEADZONE_CAPTURE_S)
    assert machine.step is CalStep.SATURATION
    device.axes[0] = 0.0

    def comfortable(u: float, _t: float) -> None:
        device.axes[0] = 0.0
        device.axes[2] = 1.0 - 1.9 * (u if u < 0.5 else 1.0 - u) * 2.0
        device.axes[5] = 1.0 - 1.9 * (u if u < 0.5 else 1.0 - u) * 2.0

    _run_step(machine, device, clock, comfortable, SATURATION_CAPTURE_S)
    assert machine.step is CalStep.INVERSION

    device.axes[2] = 1.0
    device.axes[5] = 1.0
    for _ in range(3):
        machine.feed(device.snapshot)
        clock.advance(TICK)
    assert machine.advance()  # inversion accepted as detected
    assert machine.step is CalStep.CURVE

    machine.set_curve(Control.THROTTLE, CurveSpec.expo(1.8))
    assert machine.advance()
    assert machine.step is CalStep.ROTATION_LOCK
    return machine


def test_analog_wheel_calibration_produces_a_drivable_profile() -> None:
    clock = FakeClock()
    device = _analog_wheel()
    machine = _calibrate_analog(clock, device)

    # Half a turn should be full lock.
    device.axes[0] = 0.5
    machine.feed(device.snapshot)
    gain = machine.capture_rotation_lock()
    assert 1.8 < gain < 2.2, gain
    device.axes[0] = 0.0
    assert machine.advance()
    assert machine.step is CalStep.BINDINGS

    order = [
        Action.ESTOP,
        Action.ARM,
        Action.DISARM,
        Action.REVERSE,
        Action.HORN,
    ]
    for index, action in enumerate(order):
        assert machine.binding_target is action
        machine.feed(device.snapshot)  # released: arms the capture
        clock.advance(TICK)
        device.press(index)
        machine.feed(device.snapshot)
        clock.advance(TICK)
        device.release(index)
    while machine.binding_target is not None:
        machine.skip_binding()
    assert machine.advance()
    assert machine.step is CalStep.VERIFY

    def verify(u: float, _t: float) -> None:
        if u < 0.25:
            device.axes[0] = -1.0
        elif u < 0.5:
            device.axes[0] = 1.0
        else:
            device.axes[0] = 0.0
        device.axes[2] = -1.0 if 0.5 <= u < 0.7 else 1.0
        device.axes[5] = -1.0 if 0.7 <= u < 0.9 else 1.0

    _run_step(machine, device, clock, verify, 4.0, limit=4.0)
    machine.advance()
    report = machine.verify_report
    assert report.passed, report.problems

    profile = machine.profile
    assert profile.mapping.steer.positive.index == 0
    assert profile.mapping.throttle.positive.index == 2
    assert profile.mapping.brake.positive.index == 5
    assert profile.mapping.action_ref(Action.ESTOP) == button(0)
    assert profile.mapping.action_ref(Action.HORN) == button(4)
    assert profile.mapping.conflicts() == []

    # The measured deadzone reflects the injected jitter and nothing more.
    steer_cfg = profile.chain.axis(Control.STEER)
    assert 0.005 < steer_cfg.deadzone < 0.05, steer_cfg.deadzone
    assert profile.chain.throttle.curve.spec.kind.value == "expo"

    # And the resulting chain reaches full scale in both directions.
    chain = InputChain(profile.chain)
    for _ in range(3000):
        out = chain.update(1.0, -1.0, -1.0, 0.004)
    assert out.steer_q == 1000
    assert out.throttle_q == 1000
    assert out.brake_q == 1000
    for _ in range(3000):
        out = chain.update(-1.0, 1.0, 1.0, 0.004)
    assert out.steer_q == -1000
    assert out.throttle_q == 0
    assert out.brake_q == 0

    summary = summarize(machine)
    assert summary["steering"] == "axis 0"
    assert summary["conflicts"] == []


def test_inverted_steering_axis_is_detected_from_the_wiggle_order() -> None:
    """Right-first movement on a backwards axis must come out inverted."""
    clock = FakeClock()
    device = FakeDevice(num_axes=3, rest=(0.0, 1.0, 1.0))
    machine = CalibrationMachine(clock=clock)
    machine.attach_device(device)
    machine.advance()
    _run_step(machine, device, clock, lambda u, t: None, IDLE_CAPTURE_S)

    def backwards(u: float, _t: float) -> None:
        # The driver turns left first, but this axis reads positive when they do.
        if u < 0.4:
            device.axes[0] = u / 0.4
        elif u < 0.8:
            device.axes[0] = 1.0 - (u - 0.4) / 0.4 * 2.0
        else:
            device.axes[0] = -1.0 + (u - 0.8) / 0.2

    _run_step(machine, device, clock, backwards, WIGGLE_CAPTURE_S)
    assert machine.assignments[Control.STEER].invert


def test_digital_pedals_are_detected_as_buttons() -> None:
    clock = FakeClock()
    device = FakeDevice(
        guid="digital", name="Cheap Wheel", num_axes=1, num_buttons=10, rest=(0.0,)
    )
    machine = CalibrationMachine(clock=clock)
    machine.attach_device(device)
    machine.advance()
    _run_step(machine, device, clock, lambda u, t: None, IDLE_CAPTURE_S)

    def steer(u: float, _t: float) -> None:
        device.axes[0] = -1.0 if u < 0.4 else (1.0 if u < 0.8 else 0.0)

    _run_step(machine, device, clock, steer, WIGGLE_CAPTURE_S)
    assert machine.step is CalStep.ASSIGN_THROTTLE

    def press_throttle(u: float, _t: float) -> None:
        if 0.2 < u < 0.6:
            device.press(7)
        else:
            device.release(7)

    _run_step(machine, device, clock, press_throttle, WIGGLE_CAPTURE_S)
    throttle = machine.assignments[Control.THROTTLE]
    assert throttle.digital
    assert throttle.ref == button(7)

    def press_brake(u: float, _t: float) -> None:
        if 0.2 < u < 0.6:
            device.press(6)
        else:
            device.release(6)

    _run_step(machine, device, clock, press_brake, WIGGLE_CAPTURE_S)
    brake = machine.assignments[Control.BRAKE]
    assert brake.digital
    assert brake.ref == button(6)

    profile = machine.profile
    assert profile.digital_pedals
    assert profile.kind.value == "wheel_digital"
    # A digital pedal gets no deadzone: a switch has no noise to reject.
    assert profile.chain.throttle.deadzone == 0.0

    chain = InputChain(profile.chain)
    for _ in range(4000):
        out = chain.update(0.0, 1.0, 0.0, 0.004)
    assert out.throttle_q == 1000


def test_combined_pedal_mode_is_diagnosed_and_workable() -> None:
    """The quirk that makes a wheel brake when you lift off."""
    clock = FakeClock()
    device = FakeDevice(
        guid="combined", name="Racing Wheel", num_axes=2, num_buttons=10, rest=(0.0, 0.0)
    )
    machine = CalibrationMachine(clock=clock)
    machine.attach_device(device)
    machine.advance()
    _run_step(machine, device, clock, lambda u, t: None, IDLE_CAPTURE_S)

    # No axis rests at an extreme, so the early hint fires.
    assert any("combined" in w for w in machine.warnings)

    def steer(u: float, _t: float) -> None:
        device.axes[0] = -1.0 if u < 0.4 else (1.0 if u < 0.8 else 0.0)

    _run_step(machine, device, clock, steer, WIGGLE_CAPTURE_S)

    def throttle(u: float, _t: float) -> None:
        device.axes[1] = 1.0 if 0.2 < u < 0.6 else 0.0

    _run_step(machine, device, clock, throttle, WIGGLE_CAPTURE_S)

    def brake(u: float, _t: float) -> None:
        device.axes[1] = -1.0 if 0.2 < u < 0.6 else 0.0

    _run_step(machine, device, clock, brake, WIGGLE_CAPTURE_S)

    assert machine.assignments[Control.THROTTLE].ref.index == 1
    assert machine.assignments[Control.BRAKE].ref.index == 1
    assert any("combined-pedal mode" in w for w in machine.warnings)

    machine.use_combined_pedal_split()
    mapping = machine.profile.mapping
    assert mapping.throttle.positive.kind is RefKind.AXIS_POSITIVE
    assert mapping.brake.positive.kind is RefKind.AXIS_NEGATIVE
    assert mapping.throttle.positive.index == mapping.brake.positive.index

    # Resolving the split axis gives one pedal or the other, never both.
    cases = ((1.0, 1.0, 0.0), (-1.0, 0.0, 1.0), (0.0, 0.0, 0.0))
    for value, expect_throttle, expect_brake in cases:
        axes = (0.0, value)
        assert resolve_binding(mapping.throttle, axes, 0, (), 0) == expect_throttle
        assert resolve_binding(mapping.brake, axes, 0, (), 0) == expect_brake


def test_machine_is_usable_after_cancel_and_never_returns_a_broken_profile() -> None:
    clock = FakeClock()
    device = _analog_wheel()
    machine = CalibrationMachine(clock=clock)
    # A profile is available before anything has been measured.
    assert isinstance(machine.profile, DeviceProfile)
    machine.attach_device(device)
    for step in (CalStep.DETECT, CalStep.IDLE):
        assert isinstance(machine.profile.chain.axis(Control.STEER).deadzone, float)
        machine.advance()
    machine.cancel()
    assert machine.cancelled
    assert not machine.advance()
    InputChain(machine.profile.chain)  # must still construct


def test_back_and_restart_do_not_corrupt_state() -> None:
    clock = FakeClock()
    device = _analog_wheel()
    machine = CalibrationMachine(clock=clock)
    machine.attach_device(device)
    machine.advance()
    _run_step(machine, device, clock, lambda u, t: None, IDLE_CAPTURE_S)
    assert machine.step is CalStep.ASSIGN_STEER
    assert machine.back()
    assert machine.step is CalStep.IDLE
    machine.restart_step()
    assert machine.step is CalStep.IDLE
    assert machine.step_progress == 0.0


# --------------------------------------------------------------------------
# Mapping and actions
# --------------------------------------------------------------------------


def test_resolve_ref_never_raises_on_a_stale_profile() -> None:
    """A profile saved against a six-axis wheel, loaded with a two-axis pad."""
    mapping = InputMap(
        steer=ControlBinding(InputRef(RefKind.AXIS, 0)),
        throttle=ControlBinding(InputRef(RefKind.AXIS, 5)),
        brake=ControlBinding(InputRef(RefKind.BUTTON, 40)),
    )
    axes = (0.4, 0.0)
    assert resolve_binding(mapping.steer, axes, 0, (), 0) == 0.4
    assert resolve_binding(mapping.throttle, axes, 0, (), 0) == 0.0
    assert resolve_binding(mapping.brake, axes, 0, (), 0) == 0.0
    assert resolve_binding(ControlBinding(hat(3, 0, 1)), axes, 0, ((0, 0),), 0) == 0.0


def test_bipolar_digital_steering() -> None:
    mapping = InputMap(steer=ControlBinding(button(1), button(0)))
    assert resolve_binding(mapping.steer, (), 0b01, (), 0) == -1.0
    assert resolve_binding(mapping.steer, (), 0b10, (), 0) == 1.0
    assert resolve_binding(mapping.steer, (), 0b11, (), 0) == 0.0
    assert resolve_binding(mapping.steer, (), 0, (), 0) == 0.0


def test_action_state_hold_toggle_and_edge() -> None:
    state = ActionState()
    estop = action_bit(Action.ESTOP)
    reverse = action_bit(Action.REVERSE)
    odom = action_bit(Action.RESET_ODOM)

    state.update(estop, estop)
    assert state.control_flags() & ControlFlags.ESTOP
    state.update(0, 0)
    assert not (state.control_flags() & ControlFlags.ESTOP)

    state.update(reverse, reverse)
    assert state.is_latched(Action.REVERSE)
    assert state.control_flags() & ControlFlags.REVERSE_REQ
    state.update(0, 0)
    assert state.control_flags() & ControlFlags.REVERSE_REQ
    state.update(reverse, reverse)
    assert not state.is_latched(Action.REVERSE)

    state.update(odom, odom)
    assert state.control_flags() & ControlFlags.RESET_ODOM
    events = state.take_events()
    assert events & odom
    assert not (state.control_flags() & ControlFlags.RESET_ODOM)
    assert state.take_events() == 0

    state.update(action_bit(Action.ARM), action_bit(Action.ARM))
    assert state.take_events() & action_bit(Action.ARM)
    # ARM is a session message, not a control flag.
    assert Action.ARM not in ACTION_FLAGS


def test_edge_tracker() -> None:
    tracker = EdgeTracker()
    tracker.update(0b0011)
    assert tracker.pressed == 0b0011
    assert tracker.released == 0
    tracker.update(0b0010)
    assert tracker.pressed == 0
    assert tracker.released == 0b0001
    tracker.reset()
    assert tracker.previous == 0


def test_conflicts_are_reported() -> None:
    mapping = InputMap(
        steer=ControlBinding(InputRef(RefKind.AXIS, 0)),
        throttle=ControlBinding(button(3)),
        brake=ControlBinding(button(4)),
    ).with_action(Action.HORN, button(3))
    conflicts = mapping.conflicts()
    assert len(conflicts) == 1
    assert "button 3" in conflicts[0]


# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------


def test_joystick_source_reads_the_cache_and_tracks_edges() -> None:
    device = FakeDevice(num_axes=3, num_buttons=8, rest=(0.0, 1.0, 1.0))
    mapping = InputMap(
        steer=ControlBinding(InputRef(RefKind.AXIS, 0)),
        throttle=ControlBinding(InputRef(RefKind.AXIS, 1)),
        brake=ControlBinding(button(6)),
    ).with_action(Action.ESTOP, button(0))
    source = JoystickSource(device, mapping)

    device.axes[0] = -0.5
    sample = source.poll()
    assert sample.steer == -0.5
    assert sample.throttle == 1.0
    assert sample.connected
    assert sample.pressed == 0

    device.press(0)
    sample = source.poll()
    assert sample.is_held(Action.ESTOP)
    assert sample.was_pressed(Action.ESTOP)
    sample = source.poll()
    assert sample.is_held(Action.ESTOP)
    # The press fires once, not on every poll of the same state.
    assert not sample.was_pressed(Action.ESTOP)


def test_joystick_tap_between_snapshots_is_not_lost() -> None:
    """A press and release inside one pump interval still reaches the TX side."""
    device = FakeDevice(num_axes=1, num_buttons=8, rest=(0.0,))
    mapping = InputMap().with_action(Action.MARK_LAP, button(3))
    source = JoystickSource(device, mapping)
    source.poll()
    device.press(3)
    device.release(3)  # level is back to zero before the snapshot is taken
    sample = source.poll()
    assert sample.was_pressed(Action.MARK_LAP)
    assert not sample.is_held(Action.MARK_LAP)


def test_joystick_disconnect_reports_neutral_not_stale() -> None:
    device = FakeDevice(num_axes=2, rest=(0.0, 0.0))
    mapping = InputMap(
        steer=ControlBinding(InputRef(RefKind.AXIS, 0)),
        throttle=ControlBinding(InputRef(RefKind.AXIS, 1)),
    )
    source = JoystickSource(device, mapping)
    device.axes[0] = 0.8
    device.axes[1] = 1.0
    assert source.poll().throttle == 1.0
    device.connected = False
    sample = source.poll()
    assert not sample.connected
    assert sample.steer == 0.0 and sample.throttle == 0.0


def test_keyboard_source_is_a_digital_wheel() -> None:
    profile = preset_keyboard()
    source = KeyboardSource(profile.mapping)
    from telekart_app.input.mapping import KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_UP

    assert source.connected
    assert source.poll().steer == 0.0

    source.press(KEY_LEFT)
    assert source.poll().steer == -1.0
    source.press(KEY_RIGHT)
    assert source.poll().steer == 0.0
    source.release(KEY_LEFT)
    assert source.poll().steer == 1.0

    source.press(KEY_UP)
    source.press(KEY_DOWN)
    sample = source.poll()
    assert sample.throttle == 1.0 and sample.brake == 1.0

    source.press(KEY_SPACE)
    assert source.poll().is_held(Action.ESTOP)
    source.release_all()
    sample = source.poll()
    assert sample.steer == 0.0 and sample.throttle == 0.0
    assert not sample.is_held(Action.ESTOP)

    # An unbound key is ignored rather than mapped to whatever bit is free.
    source.press(0x7F7F)
    assert source.poll().steer == 0.0
    assert set(source.bound_keys()) >= {KEY_LEFT, KEY_RIGHT, KEY_UP, KEY_DOWN, KEY_SPACE}


def test_keyboard_ramp_through_the_chain_is_drivable() -> None:
    profile = preset_keyboard()
    source = KeyboardSource(profile.mapping)
    chain = InputChain(profile.chain)
    from telekart_app.input.mapping import KEY_RIGHT

    source.press(KEY_RIGHT)
    ticks = 0
    out = chain.update_from(source.poll(), 0.004)
    while out.steer < 1.0 and ticks < 5000:
        out = chain.update_from(source.poll(), 0.004)
        ticks += 1
    seconds = ticks * 0.004
    # Slow enough not to slam the servo into its stop, fast enough to drive.
    assert 0.3 < seconds < 1.2, seconds


def test_script_source_evaluates_deterministically() -> None:
    clock = FakeClock()
    source = ScriptSource.full_lap(clock=clock)
    assert source.duration > 15.0
    assert source.poll().throttle == 0.0
    clock.t = source.duration * 0.5
    mid = source.poll()
    assert -1.0 <= mid.steer <= 1.0
    clock.t = source.duration + 1.0
    assert source.poll().throttle == 0.0
    assert source.finished
    assert source.sample_at(3.0) == source.sample_at(3.0)


def test_script_source_arm_action_fires_once() -> None:
    clock = FakeClock()
    source = ScriptSource.full_lap(clock=clock)
    fired = 0
    while clock.t < source.duration:
        if source.poll().was_pressed(Action.ARM):
            fired += 1
        clock.t += 0.01
    assert fired == 1


def test_replay_round_trips_through_jsonl() -> None:
    clock = FakeClock()
    script = ScriptSource.full_lap(clock=clock)
    samples: list[RawSample] = []
    while clock.t < script.duration:
        samples.append(script.poll())
        clock.t += 0.02
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "drive.jsonl"
        written = write_jsonl(path, samples)
        assert written == len(samples)

        replay_clock = FakeClock()
        replay = ReplaySource.from_jsonl(path, clock=replay_clock)
        assert abs(replay.duration - samples[-1].t) < 1e-6
        replay_clock.t = 0.0
        first = replay.poll()
        assert abs(first.steer - samples[0].steer) < 1e-6
        replay_clock.t = replay.duration
        last = replay.poll()
        assert abs(last.throttle - samples[-1].throttle) < 1e-6
        assert replay.finished


def test_null_source_is_permanently_disconnected() -> None:
    source = NullSource()
    sample = source.poll()
    assert not sample.connected
    assert sample.steer == 0.0
    chain = InputChain(identity_profile().chain)
    out = chain.update_from(sample, 0.004)
    assert out.input_lost
    assert out.throttle == 0.0


# --------------------------------------------------------------------------
# Profile storage
# --------------------------------------------------------------------------


def test_profile_store_round_trip() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "profiles.json"
        store = ProfileStore(path)
        assert len(store) == 0

        profile = store.resolve(
            guid="abc123", name="Fake Wheel", num_axes=2, num_buttons=12
        )
        store.save_profile(profile)
        assert path.exists()

        reloaded = ProfileStore(path)
        assert len(reloaded) == 1
        again = reloaded.get("abc123")
        assert again is not None
        assert again.to_dict() == profile.to_dict()
        assert reloaded.last_error == ""

        assert reloaded.remove("abc123")
        reloaded.save()
        assert len(ProfileStore(path)) == 0


def test_profile_store_survives_a_corrupt_file() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "profiles.json"
        path.write_text("{not json at all", encoding="utf-8")
        store = ProfileStore(path)
        assert len(store) == 0
        assert store.last_error
        # The unreadable file is kept, not overwritten.
        assert list(Path(tmp).glob("profiles.json.corrupt-*"))
        store.save_profile(identity_profile("x", "X"))
        assert len(ProfileStore(path)) == 1


def test_profile_store_keeps_good_profiles_when_one_is_bad() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "profiles.json"
        good = identity_profile("good", "Good")
        payload = {
            "schema": 1,
            "profiles": {
                "good": good.to_dict(),
                "bad": {"kind": "wheel_analog", "chain": {"steer": {"deadzone": 5.0}}},
            },
        }
        path.write_text(json.dumps(payload), encoding="utf-8")
        store = ProfileStore(path)
        assert store.get("good") is not None
        assert store.get("bad") is None
        assert "bad" in store.last_error


def test_atomic_save_leaves_no_temp_file() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "nested" / "profiles.json"
        store = ProfileStore(path)
        store.save_profile(identity_profile("a", "A"))
        assert path.exists()
        assert not list(path.parent.glob("*.tmp"))


# --------------------------------------------------------------------------
# Runner
# --------------------------------------------------------------------------


def _main() -> int:
    tests = [
        (name, obj)
        for name, obj in sorted(globals().items())
        if name.startswith("test_") and callable(obj)
    ]
    failures = 0
    for name, test in tests:
        try:
            test()
        except AssertionError as exc:
            failures += 1
            print(f"FAIL {name}: {exc}")
        except Exception as exc:  # noqa: BLE001 - a runner wants everything
            failures += 1
            print(f"ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"ok   {name}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_main())
