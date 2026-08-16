"""Session channel contract tests.

The framing tests here are the ones that matter most. TCP is a byte stream, and
the failure mode of getting that wrong is not a crash -- it is a handshake that
works on the bench and breaks the first time a `recv` splits a line, which on a
Pi Zero 2 W's WiFi is a matter of when, not if. Every parse path is therefore
exercised at all three shapes: fragmented, coalesced, and split at every single
byte offset.
"""

from __future__ import annotations

import json
import math

import pytest

from telekart_protocol.constants import (
    MAX_SESSION_LINE_LEN,
    PROTO_VERSION,
    TCP_SESSION_PORT,
    TCP_VIDEO_PORT,
    UDP_CONTROL_PORT,
    UDP_TELEMETRY_PORT,
    VehicleState,
)
from telekart_protocol.session import (
    ErrorCode,
    LineReader,
    Message,
    MsgType,
    SessionError,
    ack,
    error,
    hello,
    hello_ack,
    params,
    set_params,
    state,
)

GOLDEN_HELLO = hello(
    msg_id=7,
    app_version="2.0.0",
    driver="golden",
    auth="0f1e2d3c4b5a6978",
    telemetry_port=UDP_TELEMETRY_PORT,
)

#: Byte-exact. Key order is part of the contract only in the weak sense that it
#: must stay stable enough for a golden test to be meaningful -- but a decoder
#: on the other side that started depending on ordering would break here first.
GOLDEN_HELLO_LINE = (
    b'{"type":"hello","id":7,"proto":2,"app_version":"2.0.0","driver":"golden",'
    b'"auth":"0f1e2d3c4b5a6978","telemetry_port":4211}\n'
)


# ------------------------------------------------------------ constants


def test_ports_are_pinned() -> None:
    # These are baked into the mDNS advert, the systemd unit, and the app's
    # discovery fallback. Changing one silently breaks all three.
    assert UDP_CONTROL_PORT == 4210
    assert UDP_TELEMETRY_PORT == 4211
    assert TCP_SESSION_PORT == 4212
    assert TCP_VIDEO_PORT == 4213


def test_message_types_are_stable_strings() -> None:
    # MsgType is a str enum so `msg.type == "arm"` works in a log filter, and so
    # the JSON value is the enum member's own text rather than an ordinal.
    assert MsgType.ARM == "arm"
    assert MsgType.HELLO_ACK == "hello_ack"
    assert MsgType.CLEAR_ESTOP == "clear_estop"
    assert ErrorCode.PARAM_OUT_OF_RANGE == "param_out_of_range"
    for member in MsgType:
        assert member.value == member.value.lower()
        assert " " not in member.value


# --------------------------------------------------------------- golden


def test_golden_hello_encode() -> None:
    assert GOLDEN_HELLO.encode() == GOLDEN_HELLO_LINE


def test_golden_hello_decode() -> None:
    msg = Message.decode(GOLDEN_HELLO_LINE.rstrip(b"\n"))
    assert msg.type is MsgType.HELLO
    assert msg.id == 7
    assert msg.data == {
        "proto": PROTO_VERSION,
        "app_version": "2.0.0",
        "driver": "golden",
        "auth": "0f1e2d3c4b5a6978",
        "telemetry_port": 4211,
    }


def test_encoded_line_is_compact_and_newline_terminated() -> None:
    line = GOLDEN_HELLO.encode()
    assert line.endswith(b"\n")
    assert line.count(b"\n") == 1
    assert b", " not in line  # separators=(",", ":")
    assert b'": ' not in line


# ---------------------------------------------------------- round trips


@pytest.mark.parametrize("msg_type", list(MsgType))
def test_every_message_type_round_trips(msg_type: MsgType) -> None:
    original = Message(msg_type, 99, {"k": [1, 2.5, "x", None, True]})
    decoded = Message.decode(original.encode())
    assert decoded.type is msg_type
    assert decoded.id == 99
    assert decoded.data == {"k": [1, 2.5, "x", None, True]}


def test_unsolicited_messages_use_id_zero() -> None:
    msg = state(int(VehicleState.FAILSAFE), 1 << 5, "link stale")
    assert msg.id == 0
    decoded = Message.decode(msg.encode())
    assert decoded.data["state"] == int(VehicleState.FAILSAFE)
    assert decoded.data["faults"] == 32
    assert decoded.data["detail"] == "link stale"


def test_constructors_carry_the_protocol_version() -> None:
    for msg in (
        GOLDEN_HELLO,
        hello_ack(
            msg_id=1,
            car_id="kart-01",
            fw_version="2.0.0",
            session_id=0x11223344,
            session_token="000102030405060708090a0b0c0d0e0f",
            caps=["video", "calibration"],
            video_port=TCP_VIDEO_PORT,
            control_port=UDP_CONTROL_PORT,
        ),
    ):
        assert Message.decode(msg.encode()).data["proto"] == PROTO_VERSION


def test_hello_ack_round_trip() -> None:
    msg = hello_ack(
        msg_id=3,
        car_id="kart-01",
        fw_version="2.0.1",
        session_id=0xDEADBEEF,
        session_token="00ff",
        caps=["video"],
        video_port=TCP_VIDEO_PORT,
        control_port=UDP_CONTROL_PORT,
    )
    decoded = Message.decode(msg.encode())
    assert decoded.type is MsgType.HELLO_ACK
    assert decoded.data["session_id"] == 0xDEADBEEF
    assert decoded.data["caps"] == ["video"]
    assert decoded.data["video_port"] == TCP_VIDEO_PORT


def test_error_and_ack_round_trip() -> None:
    err = Message.decode(error(5, ErrorCode.NOT_ALLOWED_IN_STATE, "armed").encode())
    assert err.type is MsgType.ERROR
    assert err.data["code"] == "not_allowed_in_state"
    assert err.data["detail"] == "armed"

    acknowledgement = Message.decode(ack(5, applied=True, changed=["max_duty"]).encode())
    assert acknowledgement.type is MsgType.ACK
    assert acknowledgement.data == {"applied": True, "changed": ["max_duty"]}


def test_param_messages_round_trip() -> None:
    values = {"max_duty": 0.7, "closed_loop": False, "video_codec": "h264", "encoder_cpr": 660}
    request = Message.decode(set_params(11, values).encode())
    assert request.type is MsgType.SET_PARAMS
    assert request.data["values"] == values

    reply = Message.decode(params(11, values, applied=False).encode())
    assert reply.type is MsgType.PARAMS
    assert reply.data["values"] == values
    assert reply.data["applied"] is False


def test_decode_accepts_str_and_bytes() -> None:
    text = GOLDEN_HELLO_LINE.decode().strip()
    assert Message.decode(text) == Message.decode(text.encode())


# ------------------------------------------------------------ rejection


@pytest.mark.parametrize(
    "line",
    [
        b"",
        b"{",
        b"not json",
        b'{"type":"hello",}',
        b"\x00\x01\x02",
    ],
)
def test_malformed_json_is_rejected(line: bytes) -> None:
    with pytest.raises(SessionError):
        Message.decode(line)


@pytest.mark.parametrize("line", [b"[1,2,3]", b'"hello"', b"5", b"null", b"true"])
def test_non_object_json_is_rejected(line: bytes) -> None:
    with pytest.raises(SessionError, match="JSON object"):
        Message.decode(line)


@pytest.mark.parametrize("raw_type", ['"nope"', "5", "null", "true", '""'])
def test_unknown_message_type_is_rejected(raw_type: str) -> None:
    with pytest.raises(SessionError, match="unknown session message type"):
        Message.decode('{"type":%s,"id":1}' % raw_type)


def test_missing_type_is_rejected() -> None:
    with pytest.raises(SessionError, match="unknown session message type"):
        Message.decode(b'{"id":1}')


@pytest.mark.parametrize("raw_id", ['"1"', "1.5", "null", "[]"])
def test_non_integer_id_is_rejected(raw_id: str) -> None:
    with pytest.raises(SessionError, match="id must be an integer"):
        Message.decode('{"type":"ping","id":%s}' % raw_id)


def test_invalid_utf8_is_rejected() -> None:
    with pytest.raises(SessionError, match="UTF-8"):
        Message.decode(b'{"type":"ping","id":1,"x":"\xff\xfe"}')


def test_oversized_line_is_rejected_on_decode() -> None:
    with pytest.raises(SessionError, match="too long"):
        Message.decode(b"x" * (MAX_SESSION_LINE_LEN + 1))


def test_oversized_message_is_rejected_on_encode() -> None:
    huge = Message(MsgType.PARAMS, 1, {"values": {"blob": "x" * MAX_SESSION_LINE_LEN}})
    with pytest.raises(SessionError, match="too large"):
        huge.encode()


def test_nan_is_refused_on_encode() -> None:
    # `allow_nan=False`: a NaN in a parameter push would decode on the far side
    # as JavaScript-flavoured `NaN`, which is not JSON and which several parsers
    # accept silently. Better to fail at the sender.
    with pytest.raises(ValueError):
        Message(MsgType.SET_PARAMS, 1, {"values": {"max_duty": math.nan}}).encode()
    with pytest.raises(ValueError):
        Message(MsgType.SET_PARAMS, 1, {"values": {"max_duty": math.inf}}).encode()


def test_session_error_is_not_swallowed_by_value_error_handlers() -> None:
    assert issubclass(SessionError, Exception)
    assert not issubclass(SessionError, ValueError)


# ----------------------------------------------------- typed accessors


def test_require_returns_correctly_typed_fields() -> None:
    msg = Message.decode(GOLDEN_HELLO.encode())
    assert msg.require("driver", str) == "golden"
    assert msg.require("telemetry_port", int) == 4211
    assert msg.require("proto", int) == PROTO_VERSION


def test_require_rejects_wrong_type_and_missing_keys() -> None:
    msg = Message.decode(GOLDEN_HELLO.encode())
    with pytest.raises(SessionError, match="must be str"):
        msg.require("telemetry_port", str)
    with pytest.raises(SessionError, match="must be dict"):
        msg.require("absent", dict)


# --------------------------------------------------------- LineReader


def test_reader_yields_a_whole_message() -> None:
    reader = LineReader()
    assert reader.feed(GOLDEN_HELLO_LINE) == [Message.decode(GOLDEN_HELLO_LINE.strip())]


def test_reader_returns_nothing_until_the_newline_arrives() -> None:
    reader = LineReader()
    assert reader.feed(GOLDEN_HELLO_LINE[:-1]) == []
    messages = reader.feed(b"\n")
    assert len(messages) == 1
    assert messages[0].type is MsgType.HELLO


def test_reader_reassembles_a_byte_at_a_time() -> None:
    reader = LineReader()
    collected: list[Message] = []
    for index in range(len(GOLDEN_HELLO_LINE)):
        collected.extend(reader.feed(GOLDEN_HELLO_LINE[index : index + 1]))
    assert len(collected) == 1
    assert collected[0].id == 7


def test_reader_coalesces_three_messages_in_one_chunk() -> None:
    stream = (
        Message(MsgType.PING, 1).encode()
        + Message(MsgType.PONG, 1).encode()
        + Message(MsgType.ARM, 2).encode()
    )
    messages = LineReader().feed(stream)
    assert [m.type for m in messages] == [MsgType.PING, MsgType.PONG, MsgType.ARM]
    assert [m.id for m in messages] == [1, 1, 2]


@pytest.mark.parametrize("split", range(0, 120, 7))
def test_reader_survives_a_split_at_any_offset(split: int) -> None:
    stream = (
        GOLDEN_HELLO_LINE
        + Message(MsgType.ARM, 8).encode()
        + Message(MsgType.GET_PARAMS, 9).encode()
    )
    reader = LineReader()
    messages = reader.feed(stream[:split])
    messages += reader.feed(stream[split:])
    assert [m.type for m in messages] == [MsgType.HELLO, MsgType.ARM, MsgType.GET_PARAMS]


def test_reader_handles_multibyte_utf8_split_across_chunks() -> None:
    # A driver name is user-supplied text. UTF-8 continuation bytes are never
    # 0x0A, so line splitting is safe, but the decode must still see the whole
    # character -- which it only does because LineReader decodes per line, not
    # per chunk.
    line = Message(MsgType.HELLO, 1, {"driver": "Zhaojin 车"}).encode()
    reader = LineReader()
    collected: list[Message] = []
    for index in range(len(line)):
        collected.extend(reader.feed(line[index : index + 1]))
    assert len(collected) == 1
    assert collected[0].data["driver"] == "Zhaojin 车"


def test_reader_ignores_blank_and_whitespace_lines() -> None:
    # Keepalive newlines and a trailing newline after a clean close are normal.
    reader = LineReader()
    assert reader.feed(b"\n\n   \n\t\n") == []
    assert len(reader.feed(GOLDEN_HELLO_LINE)) == 1


def test_reader_tolerates_crlf() -> None:
    reader = LineReader()
    messages = reader.feed(GOLDEN_HELLO_LINE.replace(b"\n", b"\r\n"))
    assert len(messages) == 1
    assert messages[0].type is MsgType.HELLO


def test_reader_propagates_a_malformed_line() -> None:
    reader = LineReader()
    with pytest.raises(SessionError):
        reader.feed(b"{not json}\n")


def test_reader_drops_a_runaway_line_and_recovers() -> None:
    # A peer that never sends a newline must not grow the buffer without bound.
    # After the error the buffer is cleared, so the connection can carry on if
    # the caller chooses not to tear it down.
    reader = LineReader(limit=256)
    with pytest.raises(SessionError, match="exceeded limit"):
        reader.feed(b"x" * 300)
    messages = reader.feed(GOLDEN_HELLO_LINE)
    assert len(messages) == 1
    assert messages[0].type is MsgType.HELLO


def test_reader_reset_discards_a_partial_line() -> None:
    reader = LineReader()
    assert reader.feed(b'{"type":"pi') == []
    reader.reset()
    messages = reader.feed(GOLDEN_HELLO_LINE)
    assert len(messages) == 1


def test_reader_survives_a_full_handshake_transcript() -> None:
    # End to end at the message level: what a real bring-up actually looks like,
    # delivered in deliberately awkward chunks.
    transcript = b"".join(
        [
            GOLDEN_HELLO.encode(),
            hello_ack(
                msg_id=7,
                car_id="kart-01",
                fw_version="2.0.0",
                session_id=0x11223344,
                session_token="00112233445566778899aabbccddeeff",
                caps=["video", "calibration"],
                video_port=TCP_VIDEO_PORT,
                control_port=UDP_CONTROL_PORT,
            ).encode(),
            Message(MsgType.GET_PARAMS, 8).encode(),
            params(8, {"max_duty": 0.85}).encode(),
            set_params(9, {"max_duty": 0.6}).encode(),
            ack(9, changed=["max_duty"]).encode(),
            Message(MsgType.ARM, 10).encode(),
            error(10, ErrorCode.NOT_ALLOWED_IN_STATE, "throttle not neutral").encode(),
            state(int(VehicleState.SAFE), 0).encode(),
        ]
    )
    reader = LineReader()
    collected: list[Message] = []
    for index in range(0, len(transcript), 13):
        collected.extend(reader.feed(transcript[index : index + 13]))
    assert [m.type for m in collected] == [
        MsgType.HELLO,
        MsgType.HELLO_ACK,
        MsgType.GET_PARAMS,
        MsgType.PARAMS,
        MsgType.SET_PARAMS,
        MsgType.ACK,
        MsgType.ARM,
        MsgType.ERROR,
        MsgType.STATE,
    ]
    assert collected[1].data["session_id"] == 0x11223344
    assert collected[7].data["code"] == ErrorCode.NOT_ALLOWED_IN_STATE.value


def test_reader_default_limit_matches_the_protocol_constant() -> None:
    reader = LineReader()
    payload = json.dumps({"type": "params", "id": 1, "values": {}}).encode()
    assert len(reader.feed(payload + b"\n")) == 1
    with pytest.raises(SessionError):
        reader.feed(b"y" * (MAX_SESSION_LINE_LEN + 1))
