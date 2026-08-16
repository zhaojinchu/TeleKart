"""The camera half of the vehicle firmware, deliberately its own OS process.

It shares nothing with ``telekart`` but the wire protocol package. That is the
point: the GIL is real, so encoding must not be able to inject jitter into the
100 Hz control loop, and on a 512 MB Pi Zero 2 W the OOM policy has to be able
to guarantee the kernel takes the camera and never the car. Neither guarantee
survives putting both jobs in one interpreter.

Nothing here imports ``picamera2`` at module scope, so the whole package
imports and its logic is testable on a development machine with
`camera.SyntheticSource` standing in for the sensor.
"""

from __future__ import annotations

from .camera import (
    CameraError,
    CameraSource,
    EncodedFrame,
    Picamera2Source,
    SyntheticSource,
    create_source,
)
from .config import ConfigError, VideoConfig, load_config
from .framing import FrameSequencer, frame_bytes, header_bytes
from .server import ClientStats, ServerStats, VideoServer

__version__ = "2.0.0"

__all__ = [
    "__version__",
    "CameraError",
    "CameraSource",
    "ClientStats",
    "ConfigError",
    "EncodedFrame",
    "FrameSequencer",
    "Picamera2Source",
    "ServerStats",
    "SyntheticSource",
    "VideoConfig",
    "VideoServer",
    "create_source",
    "frame_bytes",
    "header_bytes",
    "load_config",
]
