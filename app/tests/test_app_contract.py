"""The invariants ``docs/INTERFACES.md`` section 9 states about the app.

These are the ones that are cheap to violate by accident and expensive to
diagnose afterwards: a second consumer on a LatestBox, a second 60 Hz clock, a
decoder that quietly buffers five frames, or a HUD that decides for itself that
the car is armed.
"""

from __future__ import annotations

import time

import pytest

from telekart_protocol import Fault, TelemetryFlags, TelemetryPacket, VehicleState

from telekart_app.core.latest_box import LatestBox
from telekart_app.model.app_model import DEFAULT_TICK_HZ, AppModel


class _Sample:
    """What TelemetryRxThread puts in the box: a packet plus arrival time."""

    __slots__ = ("packet", "recv_t")

    def __init__(self, packet: TelemetryPacket) -> None:
        self.packet = packet
        self.recv_t = time.perf_counter()


class _Fixture:
    """Five bare boxes. No sockets, no car, no simulator -- section 9's claim
    that the whole UI is drivable from this is what these tests rely on."""

    def __init__(self) -> None:
        self.telemetry_box: LatestBox = LatestBox()
        self.link_box: LatestBox = LatestBox()
        self.input_box: LatestBox = LatestBox()
        self.session_box: LatestBox = LatestBox()
        self.video_box: LatestBox = LatestBox()


class _RecordingController:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def __getattr__(self, name: str):
        def record(*_args, **_kwargs):
            self.calls.append(name)
            return 0

        return record


def _packet(state: VehicleState, sequence: int = 1) -> TelemetryPacket:
    return TelemetryPacket.from_si(
        session_id=7,
        sequence=sequence,
        car_time_us=sequence * 20_000,
        state=state,
        faults=Fault.NONE,
        flags=TelemetryFlags.ODOM_VALID,
        speed_mps=0.4,
        v_max_mps=0.6,
    )


@pytest.fixture()
def model(qapp):
    sources = _Fixture()
    controller = _RecordingController()
    m = AppModel(sources, controller=controller)
    return m, sources, controller


# --------------------------------------------------------------------------
# One tick, one drain
# --------------------------------------------------------------------------


def test_the_model_owns_exactly_one_tick(model) -> None:
    m, _sources, _c = model
    assert m._timer.interval() == round(1000.0 / DEFAULT_TICK_HZ)
    children = [c for c in m.children() if c.__class__.__name__ == "QTimer"]
    assert len(children) == 1, "AppModel must own exactly one timer"


def test_drive_screen_rides_the_model_tick_rather_than_its_own(model) -> None:
    """A second 60 Hz clock beats against the first and drops video frames."""
    from telekart_app.ui.screens.drive import DriveScreen

    m, _sources, _c = model
    screen = DriveScreen(m)
    timers = [c for c in screen.children() if c.__class__.__name__ == "QTimer"]
    assert timers == [], "the drive screen must not run its own frame clock"


def test_one_drain_per_tick_gives_a_consistent_snapshot(model) -> None:
    m, sources, _c = model
    seen: list[tuple[float, float]] = []
    m.vehicleChanged.connect(lambda v: seen.append((v.speed, v.v_max)))

    sources.telemetry_box.put(_Sample(_packet(VehicleState.ARMED, 1)))
    m.tick_once()
    assert seen == [(pytest.approx(0.4), pytest.approx(0.6))]

    # Nothing new: no signal, because an unchanged packet is not news.
    m.tick_once()
    assert len(seen) == 1


# --------------------------------------------------------------------------
# LatestBox discipline
# --------------------------------------------------------------------------


def test_take_frame_hands_each_bundle_over_exactly_once(model) -> None:
    """The model is the video box's single consumer; the view pulls from it."""
    m, sources, _c = model
    sentinel = object()
    sources.video_box.put(sentinel)

    m.tick_once()
    assert m.take_frame() is sentinel
    assert m.take_frame() is None, "a bundle was handed out twice"

    # And the model keeps its own reference, which is what keeps the decoded
    # pixels alive while a widget paints them.
    assert m.frame is sentinel


def test_take_frame_is_none_before_any_video(model) -> None:
    m, _sources, _c = model
    m.tick_once()
    assert m.take_frame() is None


def test_peeking_a_box_does_not_starve_the_model(model) -> None:
    """The input thread peeks telemetry for the speed-sensitive assist."""
    m, sources, _c = model
    sources.telemetry_box.put(_Sample(_packet(VehicleState.ARMED, 1)))
    assert sources.telemetry_box.peek() is not None
    m.tick_once()
    assert m.vehicle.valid, "peek consumed the update"


# --------------------------------------------------------------------------
# The app never asserts its own armed state
# --------------------------------------------------------------------------


def test_arm_requests_but_never_claims(model) -> None:
    m, sources, controller = model
    sources.telemetry_box.put(_Sample(_packet(VehicleState.SAFE, 1)))
    m.tick_once()
    assert m.vehicle.state is VehicleState.SAFE

    m.arm()
    assert controller.calls == ["arm"]
    m.tick_once()
    # The car has not said anything new. The app must still show SAFE.
    assert m.vehicle.state is VehicleState.SAFE
    assert not m.vehicle.armed

    sources.telemetry_box.put(_Sample(_packet(VehicleState.ARMED, 2)))
    m.tick_once()
    assert m.vehicle.armed, "the car said ARMED and the app did not follow"


def test_state_signal_fires_only_on_the_car_s_transitions(model) -> None:
    m, sources, _c = model
    states: list[VehicleState] = []
    m.stateChanged.connect(states.append)

    for i, state in enumerate(
        (
            VehicleState.SAFE,
            VehicleState.SAFE,
            VehicleState.ARMED,
            VehicleState.ARMED,
            VehicleState.ESTOP,
        ),
        start=1,
    ):
        sources.telemetry_box.put(_Sample(_packet(state, i)))
        m.tick_once()

    assert states == [VehicleState.SAFE, VehicleState.ARMED, VehicleState.ESTOP]


def test_faults_are_edge_triggered(model) -> None:
    m, sources, _c = model
    raised: list[int] = []
    cleared: list[int] = []
    m.faultRaised.connect(lambda bit, _text: raised.append(bit))
    m.faultCleared.connect(lambda bit, _text: cleared.append(bit))

    def push(seq: int, faults: Fault) -> None:
        sources.telemetry_box.put(
            _Sample(
                TelemetryPacket.from_si(
                    session_id=7,
                    sequence=seq,
                    car_time_us=seq * 20_000,
                    state=VehicleState.ARMED,
                    faults=faults,
                    speed_mps=0.4,
                    v_max_mps=0.6,
                )
            )
        )
        m.tick_once()

    push(1, Fault.NONE)
    push(2, Fault.STALL_L)
    push(3, Fault.STALL_L)  # still latched: must not re-fire
    push(4, Fault.NONE)

    assert raised == [int(Fault.STALL_L)]
    assert cleared == [int(Fault.STALL_L)]


# --------------------------------------------------------------------------
# Video decode settings
# --------------------------------------------------------------------------


def test_decoder_never_uses_frame_threading() -> None:
    """Frame threading buffers 3-5 frames: 100-170 ms straight onto the driver."""
    pytest.importorskip("av")
    from telekart_protocol import VideoCodec

    from telekart_app.video.decoder import VideoDecoder

    decoder = VideoDecoder(VideoCodec.H264)
    try:
        ctx = decoder._ctx
        # PyAV returns an enum whose value is FFmpeg's FF_THREAD_* bitmask;
        # zero is "no threading". FRAME is 1 and SLICE is 2, so a non-zero
        # value here is the latency bug this assertion exists to catch.
        assert ctx.thread_type.value == 0, "frame or slice threading is enabled"
        assert ctx.thread_type.name == "NONE"
        assert ctx.thread_count == 1
        assert int(ctx.flags) & 0x00080000, "AV_CODEC_FLAG_LOW_DELAY is not set"
    finally:
        decoder.close()


def test_frame_bundle_holds_the_frame_and_the_image() -> None:
    """Nothing else may build a QImage over a decoded plane."""
    pytest.importorskip("av")
    import av
    import numpy as np

    from telekart_app.video.frame import PIXEL_FORMAT, FrameBundle

    rgb = np.zeros((64, 96, 3), dtype=np.uint8)
    rgb[..., 0] = 200
    frame = av.VideoFrame.from_ndarray(rgb, format="rgb24").reformat(
        format=PIXEL_FORMAT
    )
    bundle = FrameBundle.from_video_frame(frame)
    assert bundle.frame is frame
    assert bundle.image.width() == 96 and bundle.image.height() == 64
    assert bundle.buffer.nbytes > 0
    # The image must be a view, not a copy: that is the whole reason the
    # bundle has to keep the frame alive.
    assert bundle.image.constBits() is not None


def test_frame_bundle_rejects_an_unconverted_frame() -> None:
    pytest.importorskip("av")
    import av
    import numpy as np

    from telekart_app.video.frame import FrameBundle

    frame = av.VideoFrame.from_ndarray(
        np.zeros((16, 16, 3), dtype=np.uint8), format="rgb24"
    )
    with pytest.raises(ValueError):
        FrameBundle.from_video_frame(frame)
