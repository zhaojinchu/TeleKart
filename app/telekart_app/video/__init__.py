"""Video receive, decode and frame lifetime.

Importing this package pulls in PyAV and QtGui. That is why nothing outside it
imports it at module scope -- ``LinkManager`` loads it lazily, so a headless
integration test can drive control and telemetry with neither installed.
"""

from __future__ import annotations

from .decoder import CODEC_OPTIONS, CONTAINER_OPTIONS, DecoderStats, VideoDecoder
from .frame import PIXEL_FORMAT, FrameBundle, live_count, peak_count, reset_counters
from .receiver import VideoReceiver, VideoRxThread, VideoStats

__all__ = [
    "FrameBundle",
    "PIXEL_FORMAT",
    "live_count",
    "peak_count",
    "reset_counters",
    "VideoDecoder",
    "DecoderStats",
    "CODEC_OPTIONS",
    "CONTAINER_OPTIONS",
    "VideoReceiver",
    "VideoRxThread",
    "VideoStats",
]
