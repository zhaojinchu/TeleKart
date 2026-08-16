"""The app's network stack against a real simulator, over real sockets.

``docs/INTERFACES.md`` 11 calls for exactly this: drive the net stack with no Qt
widgets. It is the most valuable test in the repository, because it is the only
one where the two independently-written halves of the system have to agree --
the app's ``LinkManager`` on one side, the simulator's ``SimServer`` on the
other, with nothing shared but ``telekart_protocol``. Every unit test on either
side can pass while the handshake, the session token derivation, the sequence
guard or the telemetry layout quietly disagree.

The simulator runs in-process on a thread and on non-default ports, so the test
neither shells out nor collides with a simulator somebody left running.
"""

from __future__ import annotations

import socket
import threading
import time

import pytest

from telekart_protocol import ControlFlags, VehicleState

from telekart_app.config.settings import Settings
from telekart_app.model.snapshots import LinkState
from telekart_app.net.control_tx import ControlCommand
from telekart_app.net.link_manager import LinkManager

pytest.importorskip("telekart_sim")

from telekart_sim.autodrive import Track  # noqa: E402
from telekart_sim.physics import PlantParams, SimFaultOptions, VehicleSim  # noqa: E402
from telekart_sim.transport import (  # noqa: E402
    NetworkOptions,
    ServerOptions,
    SimServer,
)

KEY = "integration-key"
CAR_ID = "telekart-itest"


def _free_ports(count: int) -> list[int]:
    """Ports the OS just handed out. Racy in principle, fine in practice, and
    far better than hardcoding numbers a developer's own simulator may hold."""
    socks = []
    ports = []
    for _ in range(count):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        ports.append(s.getsockname()[1])
        socks.append(s)
    for s in socks:
        s.close()
    return ports


@pytest.fixture(scope="module")
def sim():
    control_port, session_port, video_port = _free_ports(3)
    track = Track.load("oval")
    vehicle = VehicleSim(
        params={},
        plant=PlantParams(),
        faults=SimFaultOptions(),
        seed=1,
    )
    vehicle.place(track.start_x, track.start_y, track.start_heading)
    server = SimServer(
        vehicle=vehicle,
        track=track,
        options=ServerOptions(
            car_id=CAR_ID,
            shared_key=KEY,
            bind_host="127.0.0.1",
            control_port=control_port,
            session_port=session_port,
            video_port=video_port,
            # No mDNS: this test connects by address, and a multicast responder
            # in a unit test is a good way to confuse everything else on the LAN.
            mdns=False,
            status_interval_s=0.0,
        ),
        network=NetworkOptions(),
        video=None,
    )
    thread = threading.Thread(target=server.run, name="itest-sim", daemon=True)
    thread.start()
    _wait(lambda: server.stats.ticks > 5, 5.0, "the simulator never ticked")
    try:
        yield server, session_port, control_port
    finally:
        server.stop()
        thread.join(timeout=5.0)


@pytest.fixture()
def link(sim):
    _server, session_port, _control = sim
    settings = Settings()
    settings.link.telemetry_port = 0  # ephemeral; the app tells the car which
    settings.video.enabled = False    # covered by the sim's own video tests
    settings.recording.enabled = False
    manager = LinkManager(settings)
    try:
        manager.connect(
            "127.0.0.1:%d" % session_port, shared_key=KEY, driver="itest"
        )
        _wait(
            lambda: manager.state is LinkState.LIVE,
            10.0,
            "the link never went live: %s" % (manager.state,),
        )
        yield manager
    finally:
        manager.close()


def _wait(predicate, timeout: float, message: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(message + " (after %.1fs)" % timeout)


def _telemetry(manager: LinkManager):
    sample = manager.telemetry_box.peek()
    assert sample is not None, "no telemetry has arrived"
    return sample.packet


#: ``arm_neutral_ms`` default, plus slack for the 100 Hz control stream.
_NEUTRAL_HOLD_S = 0.5 + 0.25


def _arm(manager: LinkManager) -> None:
    """Satisfy the car's arming preconditions, then ask.

    INTERFACES.md 4: arming needs an explicit ARM, throttle at neutral for
    ``arm_neutral_ms``, a valid session, and no critical fault. Holding neutral
    first is not test scaffolding -- it is the sequence the safety state machine
    requires, and skipping it is how the request gets refused.
    """
    manager.command_box.put(ControlCommand())
    _wait(lambda: manager.telemetry_box.peek() is not None, 5.0, "no telemetry")
    time.sleep(_NEUTRAL_HOLD_S)
    manager.arm()
    _wait(
        lambda: _telemetry(manager).state is VehicleState.ARMED,
        5.0,
        "the car did not arm",
    )


# --------------------------------------------------------------------------
# The handshake
# --------------------------------------------------------------------------


def test_handshake_establishes_a_session(link: LinkManager) -> None:
    info = link.session_info
    assert info is not None
    assert info.car_id == CAR_ID
    assert info.session_id != 0
    assert info.udp_key, "no UDP key was derived from the session token"
    assert "video" in info.caps


def test_telemetry_flows_and_is_authentic(link: LinkManager) -> None:
    _wait(
        lambda: link.telemetry_box.peek() is not None, 5.0, "no telemetry arrived"
    )
    packet = _telemetry(link)
    assert packet.session_id == link.session_info.session_id
    # Authentication happened in TelemetryRxThread: an unpacked packet is a
    # packet whose truncated HMAC verified against the derived key.
    _wait(
        lambda: link._telemetry.stats().packets > 20,
        5.0,
        "telemetry is not arriving at rate",
    )
    stats = link._telemetry.stats()
    assert stats.bad == 0, "%d telemetry packets failed to decode" % stats.bad
    assert stats.foreign == 0, "packets arrived for another session"


def test_the_car_publishes_a_measured_top_speed(link: LinkManager) -> None:
    """Nothing hardcodes a top speed; the gauge scales off this."""
    _wait(
        lambda: link.telemetry_box.peek() is not None, 5.0, "no telemetry arrived"
    )
    packet = _telemetry(link)
    assert packet.v_max_mm_s > 0
    assert 0.05 < packet.v_max_mps < 10.0


def test_control_stream_reaches_the_car(link: LinkManager, sim) -> None:
    server, _session, _control = sim
    before = server.stats.control_rx
    link.command_box.put(ControlCommand())
    _wait(
        lambda: server.stats.control_rx > before + 20,
        5.0,
        "control packets are not being accepted",
    )
    assert server.stats.control_bad == 0, (
        "%d control packets failed authentication" % server.stats.control_bad
    )
    _wait(
        lambda: _telemetry(link).echo_sequence > 0,
        5.0,
        "the car never echoed a control sequence",
    )


# --------------------------------------------------------------------------
# Arming and driving
# --------------------------------------------------------------------------


def test_arming_is_refused_until_the_throttle_has_been_neutral(
    link: LinkManager,
) -> None:
    """The neutral hold is a safety precondition, not a formality."""
    link.command_box.put(ControlCommand(throttle=0.8))
    _wait(lambda: link.telemetry_box.peek() is not None, 5.0, "no telemetry")
    time.sleep(0.3)
    link.arm()
    time.sleep(0.4)
    assert _telemetry(link).state is not VehicleState.ARMED, (
        "the car armed with the throttle open"
    )
    _arm(link)  # and it arms once the precondition is met
    link.command_box.put(ControlCommand())
    link.disarm()


def test_arm_then_drive_then_disarm(link: LinkManager) -> None:
    _arm(link)

    # The app never asserts its own armed state -- it reads the car's.
    assert _telemetry(link).armed

    link.command_box.put(
        ControlCommand(steering=0.2, throttle=0.9, flags=ControlFlags.ARM_INTENT)
    )
    _wait(
        lambda: _telemetry(link).speed_mm_s > 50,
        8.0,
        "the car did not move under throttle",
    )
    packet = _telemetry(link)
    assert abs(packet.rpm_l) > 5 and abs(packet.rpm_r) > 5
    assert packet.servo_us > 0, "steering never reached the servo"
    assert packet.distance_mm > 0, "odometry did not advance"

    link.command_box.put(ControlCommand())
    link.disarm()
    _wait(
        lambda: _telemetry(link).state is not VehicleState.ARMED,
        5.0,
        "the car did not disarm",
    )


def test_estop_latches_on_the_car(link: LinkManager) -> None:
    _arm(link)

    link.estop()
    _wait(
        lambda: _telemetry(link).state is VehicleState.ESTOP,
        5.0,
        "the E-stop did not reach the car",
    )
    # It is a latch: arming again must be refused until it is explicitly cleared.
    link.arm()
    time.sleep(0.4)
    assert _telemetry(link).state is VehicleState.ESTOP

    link.clear_estop()
    _wait(
        lambda: _telemetry(link).state is not VehicleState.ESTOP,
        5.0,
        "clearing the E-stop had no effect",
    )


# --------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------


def test_parameters_round_trip_and_the_car_is_authoritative(
    link: LinkManager,
) -> None:
    _wait(lambda: bool(link.params()), 5.0, "the car never sent its parameters")
    params = link.params()
    assert len(params) > 20
    assert "max_duty" in params

    original = float(params["max_duty"])
    target = round(min(0.95, max(0.2, original - 0.1)), 3)
    link.set_params({"max_duty": target})
    _wait(
        lambda: float(link.params().get("max_duty", original)) == target,
        5.0,
        "the car never echoed the new parameter",
    )
    # Restore, so the ordering of tests in this module cannot matter.
    link.set_params({"max_duty": original})
    _wait(
        lambda: float(link.params().get("max_duty", target)) == original,
        5.0,
        "could not restore max_duty",
    )
