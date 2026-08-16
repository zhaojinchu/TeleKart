"""The 250 Hz input thread.

Section 9's threading table: reads cached SDL axes, runs the chain, writes a
LatestBox. It never touches Qt and never touches a socket, so all of this runs
without either -- which is the point of the design and the reason it is
testable at all.
"""

from __future__ import annotations

import time

from telekart_protocol import ControlFlags

from telekart_app.core.latest_box import LatestBox
from telekart_app.input.mapping import Action, action_bit
from telekart_app.input.sources import RawSample
from telekart_app.input.thread import InputThread, telemetry_speed_source


class _StubSource:
    """A source under the test's control. Structural, like the real ones."""

    def __init__(self) -> None:
        self.sample = RawSample(connected=True)
        self.polls = 0

    @property
    def name(self) -> str:
        return "Stub"

    @property
    def connected(self) -> bool:
        return self.sample.connected

    def poll(self) -> RawSample:
        self.polls += 1
        return self.sample


def _run(thread: InputThread, ticks: int = 40) -> None:
    """Step the loop body directly rather than starting the thread.

    Deterministic, instant, and it exercises the same code -- ``run`` is a
    pacing wrapper around ``_tick``.
    """
    now = time.perf_counter()
    for i in range(ticks):
        thread._tick(now + i * 0.004, 0.004)


def test_publishes_a_command_every_tick() -> None:
    box: LatestBox = LatestBox()
    source = _StubSource()
    thread = InputThread(box, source=source)
    _run(thread, 5)
    assert source.polls == 5
    assert box.puts == 5
    command = box.peek()
    assert command is not None
    assert command.steering == 0.0 and command.throttle == 0.0


def test_a_held_pedal_ramps_and_reaches_full_scale() -> None:
    box: LatestBox = LatestBox()
    source = _StubSource()
    source.sample = RawSample(steer=0.0, throttle=1.0, brake=0.0, connected=True)
    thread = InputThread(box, source=source)

    _run(thread, 2)
    early = box.peek()
    assert early is not None
    assert 0.0 < early.throttle < 1.0, "the rate limiter is not doing anything"

    _run(thread, 400)
    late = box.peek()
    assert late is not None
    assert late.throttle == 1.0, "full travel never reached full scale"


def test_output_is_always_in_range_even_for_hostile_input() -> None:
    box: LatestBox = LatestBox()
    source = _StubSource()
    thread = InputThread(box, source=source)
    for raw in (float("nan"), float("inf"), -1e9, 1e9, -3.0, 3.0):
        source.sample = RawSample(steer=raw, throttle=raw, brake=raw, connected=True)
        _run(thread, 20)
        command = box.peek()
        assert command is not None
        assert -1.0 <= command.steering <= 1.0
        assert 0.0 <= command.throttle <= 1.0
        assert 0.0 <= command.brake <= 1.0


def test_losing_the_device_cuts_throttle() -> None:
    box: LatestBox = LatestBox()
    source = _StubSource()
    source.sample = RawSample(throttle=1.0, connected=True)
    thread = InputThread(box, source=source)
    _run(thread, 400)
    assert box.peek().throttle == 1.0

    source.sample = RawSample(throttle=1.0, connected=False)
    _run(thread, 1)
    assert box.peek().throttle == 0.0, "throttle survived the wheel being unplugged"


def test_held_actions_become_control_flags() -> None:
    box: LatestBox = LatestBox()
    source = _StubSource()
    source.sample = RawSample(held=action_bit(Action.HORN), connected=True)
    thread = InputThread(box, source=source)
    _run(thread, 3)
    assert box.peek().flags & ControlFlags.HORN


def test_toggle_actions_latch_on_one_press() -> None:
    box: LatestBox = LatestBox()
    source = _StubSource()
    thread = InputThread(box, source=source)

    bit = action_bit(Action.HEADLIGHTS)
    source.sample = RawSample(held=bit, pressed=bit, connected=True)
    _run(thread, 1)
    assert box.peek().flags & ControlFlags.HEADLIGHTS

    # Released, but the latch holds.
    source.sample = RawSample(connected=True)
    _run(thread, 5)
    assert box.peek().flags & ControlFlags.HEADLIGHTS

    source.sample = RawSample(held=bit, pressed=bit, connected=True)
    _run(thread, 1)
    source.sample = RawSample(connected=True)
    _run(thread, 5)
    assert not (box.peek().flags & ControlFlags.HEADLIGHTS)


def test_an_edge_flag_rides_more_than_one_transmit_period() -> None:
    """RESET_ODOM must not be lost to the 250 Hz / 100 Hz rate mismatch.

    The box is depth 1, so three of every five commands are overwritten before
    the TX thread ever sees them. A flag carried on exactly one command would
    be dropped three times in five.
    """
    box: LatestBox = LatestBox()
    source = _StubSource()
    thread = InputThread(box, source=source)

    bit = action_bit(Action.RESET_ODOM)
    source.sample = RawSample(held=bit, pressed=bit, connected=True)
    thread._tick(0.0, 0.004)
    source.sample = RawSample(connected=True)

    carried = 0
    for i in range(1, 40):  # 40 ticks at 250 Hz is 160 ms
        thread._tick(i * 0.004, 0.004)
        if box.peek().flags & ControlFlags.RESET_ODOM:
            carried += 1
    # 60 ms of hold is at least five 100 Hz transmit ticks.
    assert 10 <= carried <= 20, f"held for {carried} ticks"

    thread._tick(1.0, 0.004)
    assert not (box.peek().flags & ControlFlags.RESET_ODOM), "the flag never cleared"


def test_session_actions_are_queued_for_the_gui_thread() -> None:
    box: LatestBox = LatestBox()
    source = _StubSource()
    thread = InputThread(box, source=source)

    bit = action_bit(Action.ARM)
    source.sample = RawSample(held=bit, pressed=bit, connected=True)
    thread._tick(0.0, 0.004)
    assert thread.take_events() == (Action.ARM,)
    assert thread.take_events() == (), "an event was delivered twice"


def test_wire_flagged_actions_are_not_also_queued() -> None:
    """RESET_ODOM rides the control packet; sending it twice resets twice."""
    box: LatestBox = LatestBox()
    source = _StubSource()
    thread = InputThread(box, source=source)

    bit = action_bit(Action.RESET_ODOM)
    source.sample = RawSample(held=bit, pressed=bit, connected=True)
    thread._tick(0.0, 0.004)
    assert Action.RESET_ODOM not in thread.take_events()


def test_speed_source_peeks_and_never_takes() -> None:
    """AppModel is the telemetry box's single consumer."""

    class _Packet:
        speed_fraction = 0.5

    class _Sample:
        packet = _Packet()

    box: LatestBox = LatestBox()
    read = telemetry_speed_source(box)
    assert read() == 0.0  # empty box

    box.put(_Sample())
    assert read() == 0.5
    assert read() == 0.5, "the reader consumed the value"
    assert box.take() is not None, "the model was starved of its update"


def test_speed_source_survives_junk() -> None:
    box: LatestBox = LatestBox()
    read = telemetry_speed_source(box)
    box.put(object())
    assert read() == 0.0


def test_switching_source_resets_the_chain() -> None:
    box: LatestBox = LatestBox()
    first = _StubSource()
    first.sample = RawSample(throttle=1.0, connected=True)
    thread = InputThread(box, source=first)
    _run(thread, 400)
    assert box.peek().throttle == 1.0

    second = _StubSource()
    thread.set_source(second)
    thread._tick(0.0, 0.004)
    assert box.peek().throttle == 0.0, (
        "a half-applied throttle survived a device change"
    )


def test_a_raising_source_does_not_kill_the_loop() -> None:
    class _Exploding(_StubSource):
        def poll(self) -> RawSample:
            raise RuntimeError("device fell over")

    box: LatestBox = LatestBox()
    thread = InputThread(box, source=_Exploding())
    thread._shutdown.set()  # one pass through run(), then out
    thread.run()  # must not raise
