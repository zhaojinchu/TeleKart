"""TeleKart shared wire protocol.

Imported unmodified by the Pi firmware, the desktop application, and the
simulator. That single-source property is the whole point of the package: the
previous generation of this project kept a hand-written C struct and a
hand-written Python ``struct.Struct`` in sync by hand, which is a bug waiting
for the least convenient moment.

Constraints, deliberately tight:

* **Standard library only.** It has to install on a Pi Zero 2 W without a
  compiler and on macOS without Homebrew.
* **Python 3.11+ syntax.** Raspberry Pi OS Bookworm ships 3.11; the desktop app
  runs 3.12. No PEP 695 generics, no ``type`` statements.
"""

from __future__ import annotations

from .constants import (
    CONTROL_LOOP_HZ,
    CONTROL_RATE_HZ,
    CONTROL_TIMEOUT_MS,
    CRITICAL_FAULTS,
    FAILSAFE_BRAKE_AT_MS,
    FAILSAFE_COAST_AT_MS,
    FAILSAFE_DISARM_AT_MS,
    MAGIC_CONTROL,
    MAGIC_TELEMETRY,
    MAGIC_VIDEO,
    MDNS_SERVICE_TYPE,
    PROTO_VERSION,
    SESSION_TTL_S,
    TCP_SESSION_PORT,
    TCP_VIDEO_PORT,
    TELEMETRY_RATE_HZ,
    UDP_CONTROL_PORT,
    UDP_TELEMETRY_PORT,
    ControlFlags,
    Fault,
    TelemetryFlags,
    VehicleState,
    VideoCodec,
    VideoFrameFlags,
)
from .control import CONTROL_PACKET_LEN, ControlPacket, ProtocolError, peek_session_id
from .params import PARAMS, ParamDef, ParamError, coerce, coerce_all, defaults
from .session import ErrorCode, LineReader, Message, MsgType, SessionError
from .telemetry import TELEMETRY_PACKET_LEN, TelemetryPacket
from .video import (
    VIDEO_HEADER_LEN,
    FrameHeader,
    FrameReader,
    VideoProtocolError,
    pack_frame,
)

__version__ = "2.0.0"

__all__ = [
    "__version__",
    # constants
    "PROTO_VERSION",
    "MAGIC_CONTROL",
    "MAGIC_TELEMETRY",
    "MAGIC_VIDEO",
    "UDP_CONTROL_PORT",
    "UDP_TELEMETRY_PORT",
    "TCP_SESSION_PORT",
    "TCP_VIDEO_PORT",
    "MDNS_SERVICE_TYPE",
    "CONTROL_RATE_HZ",
    "TELEMETRY_RATE_HZ",
    "CONTROL_LOOP_HZ",
    "CONTROL_TIMEOUT_MS",
    "FAILSAFE_BRAKE_AT_MS",
    "FAILSAFE_COAST_AT_MS",
    "FAILSAFE_DISARM_AT_MS",
    "SESSION_TTL_S",
    "CRITICAL_FAULTS",
    # enums
    "VehicleState",
    "ControlFlags",
    "TelemetryFlags",
    "Fault",
    "VideoCodec",
    "VideoFrameFlags",
    # packets
    "ControlPacket",
    "CONTROL_PACKET_LEN",
    "TelemetryPacket",
    "TELEMETRY_PACKET_LEN",
    "ProtocolError",
    "peek_session_id",
    # session
    "Message",
    "MsgType",
    "ErrorCode",
    "SessionError",
    "LineReader",
    # params
    "PARAMS",
    "ParamDef",
    "ParamError",
    "coerce",
    "coerce_all",
    "defaults",
    # video
    "FrameHeader",
    "FrameReader",
    "VIDEO_HEADER_LEN",
    "VideoProtocolError",
    "pack_frame",
]
