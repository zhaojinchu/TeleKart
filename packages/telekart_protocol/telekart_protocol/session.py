"""Session channel -- newline-delimited JSON over TCP.

Everything that must not be lost lives here: the handshake, parameter push,
arm/disarm, calibration control. Low rate, so JSON's cost is irrelevant and its
debuggability (``nc telekart.local 4212`` shows you a readable conversation) is
worth a lot during bring-up.

The TCP connection doubles as a *presence* signal. If it drops, the car has lost
its operator, and the firmware treats that as an E-stop condition regardless of
whether UDP control packets are still arriving from somewhere.
"""

from __future__ import annotations

import enum
import json
from dataclasses import dataclass, field
from typing import Any

from .constants import MAX_SESSION_LINE_LEN, PROTO_VERSION


class MsgType(str, enum.Enum):
    # handshake
    HELLO = "hello"  # app -> car
    HELLO_ACK = "hello_ack"  # car -> app
    ERROR = "error"  # car -> app

    # arming / safety
    ARM = "arm"
    DISARM = "disarm"
    ESTOP = "estop"
    CLEAR_ESTOP = "clear_estop"
    CLEAR_FAULTS = "clear_faults"

    # parameters
    GET_PARAMS = "get_params"
    SET_PARAMS = "set_params"
    PARAMS = "params"  # car -> app; the authoritative echo

    # calibration
    CALIBRATE = "calibrate"
    CALIBRATION_STATUS = "calibration_status"

    # misc
    RESET_ODOM = "reset_odom"
    STATE = "state"  # car -> app; unsolicited state change
    PING = "ping"
    PONG = "pong"
    ACK = "ack"


class ErrorCode(str, enum.Enum):
    PROTOCOL_VERSION = "protocol_version"
    BAD_REQUEST = "bad_request"
    UNKNOWN_PARAM = "unknown_param"
    PARAM_OUT_OF_RANGE = "param_out_of_range"
    NOT_ALLOWED_IN_STATE = "not_allowed_in_state"
    BUSY = "busy"
    INTERNAL = "internal"


class SessionError(Exception):
    """Malformed or untrustworthy session traffic."""


@dataclass(slots=True)
class Message:
    type: MsgType
    #: Correlates a reply with its request. Zero means unsolicited.
    id: int = 0
    data: dict[str, Any] = field(default_factory=dict)

    def encode(self) -> bytes:
        payload = {"type": self.type.value, "id": self.id, **self.data}
        line = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        if len(line) + 1 > MAX_SESSION_LINE_LEN:
            raise SessionError(f"session message too large: {len(line)} bytes")
        return line.encode("utf-8") + b"\n"

    @classmethod
    def decode(cls, line: bytes | str) -> "Message":
        if isinstance(line, bytes):
            if len(line) > MAX_SESSION_LINE_LEN:
                raise SessionError(f"session line too long: {len(line)} bytes")
            try:
                line = line.decode("utf-8")
            except UnicodeDecodeError as exc:
                raise SessionError("session line is not valid UTF-8") from exc

        try:
            obj = json.loads(line)
        except json.JSONDecodeError as exc:
            raise SessionError(f"malformed session JSON: {exc}") from exc

        if not isinstance(obj, dict):
            raise SessionError("session message must be a JSON object")

        raw_type = obj.pop("type", None)
        try:
            msg_type = MsgType(raw_type)
        except ValueError as exc:
            raise SessionError(f"unknown session message type {raw_type!r}") from exc

        raw_id = obj.pop("id", 0)
        if not isinstance(raw_id, int):
            raise SessionError("session message id must be an integer")

        return cls(type=msg_type, id=raw_id, data=obj)

    # -- typed accessors ----------------------------------------------------

    def require(self, key: str, kind: type) -> Any:
        value = self.data.get(key)
        if not isinstance(value, kind):
            raise SessionError(
                f"{self.type.value}: field {key!r} must be {kind.__name__}, "
                f"got {type(value).__name__}"
            )
        return value


# --------------------------------------------------------------------------
# Constructors for the messages with real structure
# --------------------------------------------------------------------------


def hello(
    *,
    msg_id: int,
    app_version: str,
    driver: str,
    telemetry_port: int,
) -> Message:
    """App -> car. The first message the app sends, and the whole handshake.

    There is no ``auth`` field: this build has no shared key. The car answers
    with ``hello_ack`` and the session is open.

    The app tells the car which UDP port to send telemetry to rather than
    assuming the default, so two apps on one machine can coexist.
    """
    return Message(
        MsgType.HELLO,
        msg_id,
        {
            "proto": PROTO_VERSION,
            "app_version": app_version,
            "driver": driver,
            "telemetry_port": telemetry_port,
        },
    )


def hello_ack(
    *,
    msg_id: int,
    car_id: str,
    fw_version: str,
    session_id: int,
    caps: list[str],
    video_port: int,
    control_port: int,
) -> Message:
    return Message(
        MsgType.HELLO_ACK,
        msg_id,
        {
            "proto": PROTO_VERSION,
            "car_id": car_id,
            "fw_version": fw_version,
            "session_id": session_id,
            "caps": caps,
            "video_port": video_port,
            "control_port": control_port,
        },
    )


def error(msg_id: int, code: ErrorCode, detail: str = "") -> Message:
    return Message(MsgType.ERROR, msg_id, {"code": code.value, "detail": detail})


def ack(msg_id: int, **extra: Any) -> Message:
    return Message(MsgType.ACK, msg_id, dict(extra))


def set_params(msg_id: int, values: dict[str, Any]) -> Message:
    return Message(MsgType.SET_PARAMS, msg_id, {"values": values})


def params(msg_id: int, values: dict[str, Any], *, applied: bool = True) -> Message:
    """Car -> app. The authoritative parameter set.

    The app must render *this*, never the value it optimistically typed. A UI
    showing a setting the car never accepted is how you end up debugging the
    wrong vehicle.
    """
    return Message(MsgType.PARAMS, msg_id, {"values": values, "applied": applied})


def state(vehicle_state: int, faults: int, detail: str = "") -> Message:
    return Message(
        MsgType.STATE, 0, {"state": vehicle_state, "faults": faults, "detail": detail}
    )


# --------------------------------------------------------------------------
# Framing helper
# --------------------------------------------------------------------------


class LineReader:
    """Accumulates TCP bytes and yields complete newline-delimited messages.

    TCP gives you a byte stream, not messages -- a single ``recv`` can deliver
    half a line or three of them. This exists so no caller has to reinvent that
    and get it subtly wrong.
    """

    __slots__ = ("_buf", "_limit")

    def __init__(self, limit: int = MAX_SESSION_LINE_LEN) -> None:
        self._buf = bytearray()
        self._limit = limit

    def feed(self, chunk: bytes) -> list[Message]:
        self._buf.extend(chunk)
        messages: list[Message] = []
        while True:
            idx = self._buf.find(b"\n")
            if idx < 0:
                if len(self._buf) > self._limit:
                    self._buf.clear()
                    raise SessionError("session line exceeded limit with no newline")
                return messages
            line = bytes(self._buf[:idx])
            del self._buf[: idx + 1]
            if line.strip():
                messages.append(Message.decode(line))

    def reset(self) -> None:
        self._buf.clear()
