"""The rules every widget is written against.

These are not tests of a feature. They are tests of the four properties that
make the UI layer tractable at all, and each one has a concrete failure mode
written next to it.
"""

from __future__ import annotations

import pytest

from telekart_protocol import Fault, TelemetryPacket, VehicleState

from telekart_ui.model.app_model import AppModel
from telekart_ui.model.snapshots import (
    InputSnapshot,
    LinkSnapshot,
    LinkState,
    SessionSnapshot,
    differs,
)
from telekart_ui.net.telemetry_rx import TelemetrySample


def _packet(**kw) -> TelemetryPacket:
    base = dict(session_id=1, sequence=1, car_time_us=1000)
    base.update(kw)
    return TelemetryPacket(**base)


def _sample(packet: TelemetryPacket, recv_t: float = 1.0) -> TelemetrySample:
    return TelemetrySample(packet=packet, recv_t=recv_t, rtt=0.01)


def test_model_builds_with_no_sockets(model):
    """The whole point of the SnapshotSources protocol."""
    model.tick_once()
    assert model.vehicle.valid is False
    assert model.link.state is LinkState.OFFLINE


def test_one_tick_drains_every_box(model, boxes):
    """All five boxes in one pass, so every widget sees the same instant.

    Draining per-box on separate timers would let the speed readout come from
    packet N and the steering arc from N+1, which paints as a fault that is not
    there.
    """
    boxes.telemetry_box.put(_sample(_packet(state=VehicleState.ARMED, speed_mm_s=1200)))
    boxes.link_box.put(LinkSnapshot(state=LinkState.LIVE))
    boxes.input_box.put(InputSnapshot(throttle=0.5, transmitting=True))
    boxes.session_box.put(SessionSnapshot(active=True, car_id="kart-1"))
    boxes.video_box.put(object())

    model.tick_once()

    assert model.vehicle.state is VehicleState.ARMED
    assert model.vehicle.speed == pytest.approx(1.2)
    assert model.link.state is LinkState.LIVE
    assert model.input.throttle == pytest.approx(0.5)
    assert model.session.car_id == "kart-1"
    assert model.frame is not None


def test_frame_is_offered_exactly_once(model, boxes):
    """``take_frame`` has take semantics, not peek.

    Peek semantics would make the video widget repaint the same picture sixty
    times a second; and a second consumer calling ``take()`` on the box would
    steal frames from anything subscribed to frameReady.
    """
    bundle = object()
    boxes.video_box.put(bundle)
    model.tick_once()

    assert model.take_frame() is bundle
    assert model.take_frame() is None
    # The model still holds it: that reference is what keeps the decoded pixels
    # alive while the widget paints.
    assert model.frame is bundle


def test_faults_are_edge_triggered(model, boxes):
    """One signal per newly-set bit, not one per packet.

    Faults are sticky on the car, so a level-triggered alert would fire fifty
    times a second for as long as the fault is latched.
    """
    raised: list[tuple[int, str]] = []
    cleared: list[tuple[int, str]] = []
    model.faultRaised.connect(lambda bit, text: raised.append((bit, text)))
    model.faultCleared.connect(lambda bit, text: cleared.append((bit, text)))

    faulted = _packet(faults=Fault.STALL_L | Fault.LOW_BATTERY, sequence=1)
    boxes.telemetry_box.put(_sample(faulted))
    model.tick_once()
    assert len(raised) == 2

    # Same faults again: no new signals.
    boxes.telemetry_box.put(_sample(_packet(faults=faulted.faults, sequence=2), recv_t=1.02))
    model.tick_once()
    assert len(raised) == 2

    boxes.telemetry_box.put(_sample(_packet(faults=Fault.STALL_L, sequence=3), recv_t=1.04))
    model.tick_once()
    assert [text for _bit, text in cleared] == ["Battery low"]


def test_state_change_is_reported_from_the_car_only(model, boxes):
    """The app never asserts it is armed; it renders what the car said."""
    states: list[VehicleState] = []
    model.stateChanged.connect(states.append)

    boxes.telemetry_box.put(_sample(_packet(state=VehicleState.SAFE)))
    model.tick_once()
    boxes.telemetry_box.put(_sample(_packet(state=VehicleState.ARMED, sequence=2), 1.02))
    model.tick_once()

    assert states == [VehicleState.SAFE, VehicleState.ARMED]
    # arm() is a request. It must not move the model's idea of the state.
    model.arm()
    assert model.vehicle.state is VehicleState.ARMED


def test_unchanged_values_emit_nothing(model, boxes):
    """Sub-epsilon float noise must not wake every subscriber.

    A parked car still reports encoder noise. Without the epsilon this is a
    signal storm at the telemetry rate for a picture that never changes.
    """
    emitted: list[object] = []
    model.vehicleChanged.connect(emitted.append)

    boxes.telemetry_box.put(_sample(_packet(speed_mm_s=1000)))
    model.tick_once()
    assert len(emitted) == 1

    # 1 mm/s apart: below the 2 mm/s epsilon on `speed`.
    boxes.telemetry_box.put(_sample(_packet(speed_mm_s=1001, sequence=2), 1.02))
    model.tick_once()
    assert len(emitted) == 1

    boxes.telemetry_box.put(_sample(_packet(speed_mm_s=1200, sequence=3), 1.04))
    model.tick_once()
    assert len(emitted) == 2


def test_telemetry_goes_stale_without_new_packets(model, boxes, monkeypatch):
    """Stale means "not news", not "zero". The last values stay on screen."""
    boxes.telemetry_box.put(_sample(_packet(speed_mm_s=1500), recv_t=100.0))
    monkeypatch.setattr("time.perf_counter", lambda: 100.0)
    model.tick_once()
    assert model.vehicle.stale is False

    monkeypatch.setattr("time.perf_counter", lambda: 101.0)
    model.tick_once()
    assert model.vehicle.stale is True
    assert model.vehicle.speed == pytest.approx(1.5)


def test_commands_are_safe_without_a_controller(model):
    """No controller must be a log line, never an exception.

    The window wires these to menu items that exist before a car does.
    """
    for name in ("arm", "disarm", "estop", "clear_estop", "clear_faults", "reset_odom"):
        getattr(model, name)()


def test_differs_ignores_sequence_and_timestamps():
    a = LinkSnapshot(state=LinkState.LIVE, rtt=0.010)
    b = LinkSnapshot(state=LinkState.LIVE, rtt=0.0101)
    assert differs(a, b) is False
    assert differs(a, LinkSnapshot(state=LinkState.LIVE, rtt=0.030)) is True


def test_model_stop_releases_the_frame(qapp, boxes):
    """Several megabytes of decoded picture must not survive shutdown."""
    model = AppModel(boxes)
    boxes.video_box.put(object())
    model.tick_once()
    assert model.frame is not None
    model.stop()
    assert model.frame is None
