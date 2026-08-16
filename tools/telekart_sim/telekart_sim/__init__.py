"""A TeleKart car that is not there.

The simulator exists because the car is separate hardware that is not always
reachable, and because most of the interesting failures -- a 3 % packet loss, a
dead encoder, a flat pack, a severed session, a protocol version mismatch --
are difficult to produce on demand with a real vehicle and trivial to produce
here. It speaks the wire protocol exactly: same ports, same packets, same
handshake, same mDNS service, same framed H.264. Anything less and the desktop
app would be developed against a fiction.

Programmatic use, for integration tests that want no sockets of their own::

    from telekart_sim import SimCommand, VehicleSim

    car = VehicleSim(seed=1)
    for _ in range(500):
        snapshot = car.step(0.01, SimCommand(steering=0.0, throttle=0.6))
    assert snapshot.rpm_l > 0

Everything is deterministic under a fixed ``seed``, which is what lets those
tests assert exact numbers rather than ranges.
"""

from __future__ import annotations

from .autodrive import PurePursuitDriver, Track, TrackError
from .physics import (
    NEUTRAL_COMMAND,
    JitterStats,
    MTVelocity,
    PlantParams,
    SimCommand,
    SimFaultOptions,
    SimOdometry,
    SimSnapshot,
    VehicleSim,
)
from .transport import (
    NetworkOptions,
    ServerOptions,
    SimServer,
    SimStats,
    detect_local_ip,
)
from .video_gen import EncoderUnavailable, FrameEncoder, SceneRenderer, VideoConfig

__version__ = "2.0.0"

__all__ = [
    "NEUTRAL_COMMAND",
    "EncoderUnavailable",
    "FrameEncoder",
    "JitterStats",
    "MTVelocity",
    "NetworkOptions",
    "PlantParams",
    "PurePursuitDriver",
    "SceneRenderer",
    "ServerOptions",
    "SimCommand",
    "SimFaultOptions",
    "SimOdometry",
    "SimServer",
    "SimSnapshot",
    "SimStats",
    "Track",
    "TrackError",
    "VehicleSim",
    "VideoConfig",
    "__version__",
    "detect_local_ip",
]
