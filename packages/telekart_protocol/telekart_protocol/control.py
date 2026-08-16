"""ControlPacket -- app to car, ~100 Hz over UDP.

Fixed 32 bytes. UDP and latest-wins on purpose: a control stream wants the
newest command, never a retransmitted stale one, and TCP head-of-line blocking
during a WiFi hiccup would deliver exactly the wrong thing.

Layout (little-endian, no padding)::

    offset  size  field
    0       4     magic            MAGIC_CONTROL
    4       2     version          PROTO_VERSION
    6       4     session_id       from the TCP handshake
    10      4     sequence         strictly increasing; replay guard
    14      8     client_time_us   app monotonic clock; echoed back for RTT
    22      2     steering         -1000 (full left) .. +1000 (full right)
    24      2     throttle         0 .. 1000
    26      2     brake            0 .. 1000
    28      2     flags            ControlFlags
    30      2     reserved         must be 0

**There is no authentication tag.** Earlier revisions carried a truncated
HMAC-SHA256 here, keyed per session. It was removed deliberately, and what it
was actually defending against is worth stating so nobody re-adds it by
accident and nobody assumes it is still there:

The threat was never a determined attacker with a packet capture -- it was a
*second or stale controller*. A reconnected laptop, an app instance that did
not quite die, another machine on the LAN running a copy. The tag plus the
strictly-increasing sequence meant the car accepted commands from exactly one
session and ignored everything else.

Without it, the remaining filters are the session id, the source-address pin
the car takes from the TCP session, and the sequence check. Those stop
*accidents* -- a stray packet, a reordered datagram, a process talking to the
wrong car -- but any process that learns the session id and sends from the
right host can drive.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .constants import (
    BRAKE_SCALE,
    MAGIC_CONTROL,
    PROTO_VERSION,
    STEERING_SCALE,
    THROTTLE_SCALE,
    ControlFlags,
)

_STRUCT = struct.Struct("<IHIIQhhhHH")
CONTROL_PACKET_LEN = _STRUCT.size
assert CONTROL_PACKET_LEN == 32, f"control packet layout drifted: {CONTROL_PACKET_LEN}"


class ProtocolError(ValueError):
    """Raised when a packet cannot be trusted. Callers drop the datagram."""


def _clamp(value: int, lo: int, hi: int) -> int:
    return lo if value < lo else hi if value > hi else value


@dataclass(frozen=True, slots=True)
class ControlPacket:
    session_id: int
    sequence: int
    client_time_us: int
    steering: int = 0  # -1000 .. +1000
    throttle: int = 0  # 0 .. 1000
    brake: int = 0  # 0 .. 1000
    flags: ControlFlags = ControlFlags.NONE
    version: int = PROTO_VERSION

    # -- construction -------------------------------------------------------

    @classmethod
    def from_normalized(
        cls,
        session_id: int,
        sequence: int,
        client_time_us: int,
        steering: float,
        throttle: float,
        brake: float,
        flags: ControlFlags = ControlFlags.NONE,
    ) -> "ControlPacket":
        """Build from floats: steering -1..+1, throttle/brake 0..1.

        Values are clamped, not rejected. A control path is the wrong place to
        raise on an out-of-range float from a miscalibrated axis.
        """
        return cls(
            session_id=session_id,
            sequence=sequence & 0xFFFFFFFF,
            client_time_us=client_time_us & 0xFFFFFFFFFFFFFFFF,
            steering=_clamp(round(steering * STEERING_SCALE), -STEERING_SCALE, STEERING_SCALE),
            throttle=_clamp(round(throttle * THROTTLE_SCALE), 0, THROTTLE_SCALE),
            brake=_clamp(round(brake * BRAKE_SCALE), 0, BRAKE_SCALE),
            flags=flags,
        )

    # -- normalized views ---------------------------------------------------

    @property
    def steering_f(self) -> float:
        return self.steering / STEERING_SCALE

    @property
    def throttle_f(self) -> float:
        return self.throttle / THROTTLE_SCALE

    @property
    def brake_f(self) -> float:
        return self.brake / BRAKE_SCALE

    @property
    def estop(self) -> bool:
        return bool(self.flags & ControlFlags.ESTOP)

    # -- wire format --------------------------------------------------------

    def pack(self) -> bytes:
        return _STRUCT.pack(
            MAGIC_CONTROL,
            self.version,
            self.session_id,
            self.sequence,
            self.client_time_us,
            self.steering,
            self.throttle,
            self.brake,
            int(self.flags),
            0,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "ControlPacket":
        """Parse. Raises ProtocolError on any failure.

        Checks run cheapest-first so that garbage traffic costs almost nothing.
        """
        if len(data) != CONTROL_PACKET_LEN:
            raise ProtocolError(
                f"control packet is {len(data)} bytes, expected {CONTROL_PACKET_LEN}"
            )

        (
            magic,
            version,
            session_id,
            sequence,
            client_time_us,
            steering,
            throttle,
            brake,
            flags,
            _reserved,
        ) = _STRUCT.unpack(data)

        if magic != MAGIC_CONTROL:
            raise ProtocolError(f"bad control magic 0x{magic:08X}")
        if version != PROTO_VERSION:
            raise ProtocolError(
                f"protocol version {version}, this build speaks {PROTO_VERSION}"
            )

        return cls(
            session_id=session_id,
            sequence=sequence,
            client_time_us=client_time_us,
            steering=_clamp(steering, -STEERING_SCALE, STEERING_SCALE),
            throttle=_clamp(throttle, 0, THROTTLE_SCALE),
            brake=_clamp(brake, 0, BRAKE_SCALE),
            flags=ControlFlags(flags),
            version=version,
        )


def peek_session_id(data: bytes) -> int | None:
    """Read the session id without fully parsing.

    Only for diagnostics -- "a packet arrived for a session I don't know about"
    is worth logging. Never use this to make a control decision.
    """
    if len(data) != CONTROL_PACKET_LEN:
        return None
    magic, _version, session_id = struct.unpack_from("<IHI", data, 0)
    return session_id if magic == MAGIC_CONTROL else None
