"""The synthetic camera has to survive its own encoder.

The simulator exists so the desktop app's decode path is exercised for real.
That only works if what comes out the far end resembles what went in -- and for
a long while it did not: the encoder set a microsecond time base for pts
precision but never set a frame rate, so FFmpeg's libx264 wrapper inferred
1,000,000 fps. ``tune=zerolatency`` sizes the VBV buffer as one frame's worth of
bitrate at the stream's frame rate, which at that rate is a fraction of a bit,
and rate control pinned QP at its maximum. Every frame arrived greyscale, both
chroma planes flattened to 128, and nothing anywhere raised.

A colour assertion across a real encode/decode round trip is the only thing
that catches it.
"""

from __future__ import annotations

import numpy as np
import pytest

from telekart_sim.autodrive import Track
from telekart_sim.video_gen import FrameEncoder, SceneRenderer, VideoConfig

av = pytest.importorskip("av")


def _decoder():
    ctx = av.CodecContext.create("h264", "r")
    # The same settings INTERFACES.md 9 pins for the app. Frame threading here
    # would make the test pass or fail depending on buffering, not on colour.
    ctx.thread_type = "NONE"
    ctx.thread_count = 1
    return ctx


def _roundtrip(config: VideoConfig, frames: int = 16) -> np.ndarray:
    track = Track.load("oval")
    renderer = SceneRenderer(config, track, seed=1)
    encoder = FrameEncoder(config)
    decoder = _decoder()
    decoded = []
    for i in range(frames):
        image = renderer.render(
            x=track.start_x + i * 0.05,
            y=track.start_y,
            heading=track.start_heading,
            speed=0.4,
            yaw_rate=0.0,
            timestamp_ms=i * 33,
            frame_index=i,
        )
        for packet in encoder.encode(image.copy(), i * 33_333, force_keyframe=(i == 0)):
            for frame in decoder.decode(av.packet.Packet(packet.payload)):
                decoded.append(frame)
    encoder.close()
    assert decoded, "the encoder produced no decodable frames"
    return decoded[-1].reformat(format="rgb24").to_ndarray()


def _is_monochrome(rgb: np.ndarray) -> bool:
    """True when every pixel has R == G == B, i.e. the chroma is gone."""
    return bool(
        np.array_equal(rgb[..., 0], rgb[..., 1])
        and np.array_equal(rgb[..., 1], rgb[..., 2])
    )


def test_encoder_declares_a_frame_rate() -> None:
    """Without this, x264's rate control has no idea how fast the stream runs."""
    config = VideoConfig()
    encoder = FrameEncoder(config)
    try:
        assert encoder._context.framerate.numerator == config.fps
        assert encoder._context.framerate.denominator == 1
        # The time base stays in microseconds: a pts is a capture timestamp and
        # the HUD's latency readout is computed from it.
        assert encoder._context.time_base.denominator == 1_000_000
    finally:
        encoder.close()


def test_the_rendered_scene_is_in_colour() -> None:
    config = VideoConfig(width=320, height=240)
    track = Track.load("oval")
    renderer = SceneRenderer(config, track, seed=1)
    image = renderer.render(
        x=track.start_x,
        y=track.start_y,
        heading=track.start_heading,
        speed=0.0,
        yaw_rate=0.0,
        timestamp_ms=0,
        frame_index=0,
    )
    assert not _is_monochrome(image), "the renderer itself lost colour"


def test_encode_decode_round_trip_keeps_colour() -> None:
    rgb = _roundtrip(VideoConfig(width=320, height=240))
    assert not _is_monochrome(rgb), (
        "the decoded stream is greyscale; check the encoder's frame rate "
        "against tune=zerolatency's VBV sizing"
    )


def test_round_trip_resembles_the_source_scene() -> None:
    """What comes out matches what went in, channel by channel.

    Compared against the renderer's own last frame rather than against
    hardcoded colours, so this stays a test of the codec path and not of the
    track layout or the palette. The tolerance is loose because the codec is
    lossy at a real bitrate; the *channel spread* is the part that matters,
    because that is precisely what a collapsed rate-control state destroys.
    """
    config = VideoConfig(width=320, height=240)
    track = Track.load("oval")
    renderer = SceneRenderer(config, track, seed=1)
    source = renderer.render(
        x=track.start_x + 15 * 0.05,
        y=track.start_y,
        heading=track.start_heading,
        speed=0.4,
        yaw_rate=0.0,
        timestamp_ms=15 * 33,
        frame_index=15,
    ).astype(np.float32)
    decoded = _roundtrip(config).astype(np.float32)

    src_means = [source[..., c].mean() for c in range(3)]
    out_means = [decoded[..., c].mean() for c in range(3)]
    for channel, (want, got) in enumerate(zip(src_means, out_means)):
        assert abs(want - got) < 12.0, (
            f"channel {channel} drifted: rendered {want:.1f}, decoded {got:.1f}"
        )

    src_spread = max(src_means) - min(src_means)
    out_spread = max(out_means) - min(out_means)
    assert src_spread > 10.0, "the source scene has no colour to preserve"
    assert out_spread > src_spread * 0.6, (
        f"channel spread collapsed from {src_spread:.1f} to {out_spread:.1f}"
    )

    sky = decoded[:40]
    assert sky[..., 2].mean() > sky[..., 0].mean() + 20.0, "the sky is not blue"
