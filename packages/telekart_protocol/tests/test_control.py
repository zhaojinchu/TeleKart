"""Control packet contract tests.

The golden vector in `GOLDEN_CONTROL_HEX` is the single most load-bearing
assertion in this repository. The app encodes control packets and the car
decodes them, and the two are built by different people at different times; a
round-trip test alone would happily pass while both ends drifted together.
Asserting a hard-coded byte string in *both* directions is what makes that
impossible.
"""

from __future__ import annotations

import dataclasses
import random
import struct

import pytest

from telekart_protocol.constants import (
    BRAKE_SCALE,
    CONTROL_TIMEOUT_MS,
    FAILSAFE_BRAKE_AT_MS,
    FAILSAFE_COAST_AT_MS,
    FAILSAFE_DISARM_AT_MS,
    MAC_TAG_LEN,
    MAGIC_CONTROL,
    PROTO_VERSION,
    SEQUENCE_REPLAY_WINDOW,
    STEERING_SCALE,
    THROTTLE_SCALE,
    ControlFlags,
)
from telekart_protocol.control import (
    CONTROL_PACKET_LEN,
    ControlPacket,
    ProtocolError,
    peek_session_id,
)
from telekart_protocol.crypto import (
    compute_tag,
    derive_udp_key,
    make_session_token,
    normalize_shared_key,
    verify_tag,
)

from . import (
    GOLDEN_PASSPHRASE,
    GOLDEN_SESSION_TOKEN,
    GOLDEN_SHARED_KEY,
    GOLDEN_UDP_KEY,
    OTHER_UDP_KEY,
    flip_bit,
    splice,
)

# Byte offsets into the wire layout, from the module docstring of control.py.
OFF_MAGIC = 0
OFF_VERSION = 4
OFF_SESSION = 6
OFF_SEQUENCE = 10
OFF_CLIENT_TIME = 14
OFF_STEERING = 22
OFF_THROTTLE = 24
OFF_BRAKE = 26
OFF_FLAGS = 28
OFF_RESERVED = 30
OFF_TAG = 32

SIGNED_LEN = CONTROL_PACKET_LEN - MAC_TAG_LEN

GOLDEN_CONTROL = ControlPacket(
    session_id=0x11223344,
    sequence=0x0000002A,
    client_time_us=0x0000000123456789,
    steering=-250,
    throttle=750,
    brake=0,
    flags=ControlFlags.ARM_INTENT | ControlFlags.HEADLIGHTS,
)

GOLDEN_CONTROL_HEX = (
    "43544b31"  # magic 'CTK1'
    "0200"  # version 2
    "44332211"  # session_id 0x11223344
    "2a000000"  # sequence 42
    "8967452301000000"  # client_time_us 0x0000000123456789
    "06ff"  # steering -250
    "ee02"  # throttle 750
    "0000"  # brake 0
    "3000"  # flags ARM_INTENT | HEADLIGHTS
    "0000"  # reserved
    "fc3361ca92c91da4"  # truncated HMAC-SHA256 over bytes [0, 32)
)
GOLDEN_CONTROL_BYTES = bytes.fromhex(GOLDEN_CONTROL_HEX)


def _retag(data: bytes, key: bytes = GOLDEN_UDP_KEY) -> bytes:
    """Re-sign a hand-edited packet so a decode test reaches the field checks.

    Without this, every "what happens when field X is bogus" test would trip the
    MAC first and prove only that the MAC works.
    """
    body = data[:SIGNED_LEN]
    return body + compute_tag(key, body)


# ---------------------------------------------------------------- layout


def test_struct_size_is_pinned() -> None:
    assert CONTROL_PACKET_LEN == 40
    assert struct.calcsize("<IHIIQhhhHH8s") == 40
    assert SIGNED_LEN == 32


def test_magic_is_the_documented_ascii() -> None:
    # The magic exists so a stray datagram is rejected before we spend an HMAC
    # on it; its little-endian byte order is part of the wire contract.
    assert MAGIC_CONTROL.to_bytes(4, "little") == b"CTK1"


def test_protocol_constants_are_pinned() -> None:
    assert PROTO_VERSION == 2
    assert MAC_TAG_LEN == 8
    assert STEERING_SCALE == THROTTLE_SCALE == BRAKE_SCALE == 1000
    assert SEQUENCE_REPLAY_WINDOW == 0  # strictly increasing; no window


def test_failsafe_schedule_is_ordered() -> None:
    # The car coasts, then brakes, then coasts, then disarms. Getting these out
    # of order would mean braking before the link is actually considered stale.
    assert FAILSAFE_BRAKE_AT_MS < FAILSAFE_COAST_AT_MS < FAILSAFE_DISARM_AT_MS
    assert CONTROL_TIMEOUT_MS < FAILSAFE_DISARM_AT_MS


# ------------------------------------------------------------ golden key


def test_shared_key_derivation_matches_golden() -> None:
    assert normalize_shared_key(GOLDEN_PASSPHRASE) == GOLDEN_SHARED_KEY
    assert normalize_shared_key(GOLDEN_PASSPHRASE.encode()) == GOLDEN_SHARED_KEY


def test_udp_key_derivation_matches_golden() -> None:
    # Pinned separately from the packet vectors: if the KDF changed, every
    # golden packet below would regenerate consistently and hide the break.
    assert derive_udp_key(GOLDEN_SHARED_KEY, GOLDEN_SESSION_TOKEN) == GOLDEN_UDP_KEY


def test_kdf_rejects_degenerate_inputs() -> None:
    with pytest.raises(ValueError):
        derive_udp_key(b"", GOLDEN_SESSION_TOKEN)
    with pytest.raises(ValueError):
        derive_udp_key(GOLDEN_SHARED_KEY, b"short")
    with pytest.raises(ValueError):
        normalize_shared_key("")


def test_kdf_is_session_separated() -> None:
    a = derive_udp_key(GOLDEN_SHARED_KEY, bytes(16))
    b = derive_udp_key(GOLDEN_SHARED_KEY, bytes([1]) + bytes(15))
    assert a != b


def test_session_tokens_are_fresh() -> None:
    assert len({make_session_token() for _ in range(32)}) == 32
    assert len(make_session_token()) == 16


def test_tag_is_truncated_and_constant_time_checked() -> None:
    tag = compute_tag(GOLDEN_UDP_KEY, b"abc")
    assert len(tag) == MAC_TAG_LEN
    assert tag.hex() == "48a18cd0f7d70409"
    assert verify_tag(GOLDEN_UDP_KEY, b"abc", tag)
    assert not verify_tag(GOLDEN_UDP_KEY, b"abd", tag)
    assert not verify_tag(OTHER_UDP_KEY, b"abc", tag)


# --------------------------------------------------------------- golden


def test_golden_pack() -> None:
    assert GOLDEN_CONTROL.pack(GOLDEN_UDP_KEY).hex() == GOLDEN_CONTROL_HEX


def test_golden_unpack() -> None:
    pkt = ControlPacket.unpack(GOLDEN_CONTROL_BYTES, GOLDEN_UDP_KEY)
    assert pkt == GOLDEN_CONTROL
    assert pkt.session_id == 0x11223344
    assert pkt.sequence == 42
    assert pkt.client_time_us == 0x0000000123456789
    assert pkt.steering == -250
    assert pkt.throttle == 750
    assert pkt.brake == 0
    assert pkt.flags is ControlFlags.ARM_INTENT | ControlFlags.HEADLIGHTS
    assert pkt.version == PROTO_VERSION


def test_golden_normalized_views() -> None:
    pkt = ControlPacket.unpack(GOLDEN_CONTROL_BYTES, GOLDEN_UDP_KEY)
    assert pkt.steering_f == pytest.approx(-0.25)
    assert pkt.throttle_f == pytest.approx(0.75)
    assert pkt.brake_f == pytest.approx(0.0)
    assert pkt.estop is False


def test_from_normalized_reproduces_the_golden_packet() -> None:
    # Pins the float -> scaled-int conversion, which is the half of the contract
    # the byte vector alone does not cover.
    assert (
        ControlPacket.from_normalized(
            session_id=0x11223344,
            sequence=0x0000002A,
            client_time_us=0x0000000123456789,
            steering=-0.25,
            throttle=0.75,
            brake=0.0,
            flags=ControlFlags.ARM_INTENT | ControlFlags.HEADLIGHTS,
        )
        == GOLDEN_CONTROL
    )


def test_pack_is_deterministic_and_non_mutating() -> None:
    first = GOLDEN_CONTROL.pack(GOLDEN_UDP_KEY)
    second = GOLDEN_CONTROL.pack(GOLDEN_UDP_KEY)
    assert first == second == GOLDEN_CONTROL_BYTES


def test_key_changes_only_the_tag() -> None:
    other = GOLDEN_CONTROL.pack(OTHER_UDP_KEY)
    assert other[:SIGNED_LEN] == GOLDEN_CONTROL_BYTES[:SIGNED_LEN]
    assert other[SIGNED_LEN:] != GOLDEN_CONTROL_BYTES[SIGNED_LEN:]


# ---------------------------------------------------------- round trips


@pytest.mark.parametrize(
    "steering,throttle,brake",
    [
        (0, 0, 0),
        (STEERING_SCALE, THROTTLE_SCALE, BRAKE_SCALE),
        (-STEERING_SCALE, 0, BRAKE_SCALE),
        (1, 1, 1),
        (-1, 999, 500),
        (-999, 1000, 0),
    ],
)
def test_round_trip_extremes(steering: int, throttle: int, brake: int) -> None:
    pkt = ControlPacket(
        session_id=0xFFFFFFFF,
        sequence=0xFFFFFFFF,
        client_time_us=0xFFFFFFFFFFFFFFFF,
        steering=steering,
        throttle=throttle,
        brake=brake,
        flags=ControlFlags.ESTOP,
    )
    assert ControlPacket.unpack(pkt.pack(GOLDEN_UDP_KEY), GOLDEN_UDP_KEY) == pkt


def test_round_trip_every_defined_flag() -> None:
    for flag in ControlFlags:
        pkt = ControlPacket(session_id=1, sequence=1, client_time_us=1, flags=flag)
        decoded = ControlPacket.unpack(pkt.pack(GOLDEN_UDP_KEY), GOLDEN_UDP_KEY)
        assert decoded.flags == flag
    combined = ControlFlags(0)
    for flag in ControlFlags:
        combined |= flag
    pkt = ControlPacket(session_id=1, sequence=1, client_time_us=1, flags=combined)
    assert ControlPacket.unpack(pkt.pack(GOLDEN_UDP_KEY), GOLDEN_UDP_KEY).flags == combined


def test_defined_flags_fit_the_wire_field() -> None:
    # flags is a uint16 on the wire. Adding a 17th bit upstream must fail here
    # rather than as a struct.error inside the car's 100 Hz TX path.
    assert max(int(f) for f in ControlFlags) < 1 << 16


def test_randomized_round_trip() -> None:
    rng = random.Random(0xC0FFEE)
    for _ in range(500):
        pkt = ControlPacket(
            session_id=rng.getrandbits(32),
            sequence=rng.getrandbits(32),
            client_time_us=rng.getrandbits(64),
            steering=rng.randint(-STEERING_SCALE, STEERING_SCALE),
            throttle=rng.randint(0, THROTTLE_SCALE),
            brake=rng.randint(0, BRAKE_SCALE),
            flags=ControlFlags(rng.getrandbits(7)),
        )
        assert ControlPacket.unpack(pkt.pack(GOLDEN_UDP_KEY), GOLDEN_UDP_KEY) == pkt


# -------------------------------------------------------- normalization


@pytest.mark.parametrize(
    "steering,expected",
    [(-2.0, -1000), (-1.0, -1000), (0.0, 0), (0.5, 500), (1.0, 1000), (7.5, 1000)],
)
def test_from_normalized_clamps_steering(steering: float, expected: int) -> None:
    pkt = ControlPacket.from_normalized(1, 1, 1, steering, 0.0, 0.0)
    assert pkt.steering == expected


@pytest.mark.parametrize(
    "throttle,expected", [(-3.0, 0), (-0.001, 0), (0.0, 0), (0.333, 333), (1.0, 1000), (9.0, 1000)]
)
def test_from_normalized_clamps_throttle(throttle: float, expected: int) -> None:
    assert ControlPacket.from_normalized(1, 1, 1, 0.0, throttle, 0.0).throttle == expected


@pytest.mark.parametrize("brake,expected", [(-1.0, 0), (0.25, 250), (1.0, 1000), (4.0, 1000)])
def test_from_normalized_clamps_brake(brake: float, expected: int) -> None:
    assert ControlPacket.from_normalized(1, 1, 1, 0.0, 0.0, brake).brake == expected


@pytest.mark.parametrize(
    "bad,exc", [(float("nan"), ValueError), (float("inf"), OverflowError), (float("-inf"), OverflowError)]
)
def test_from_normalized_rejects_nonfinite_axes(bad: float, exc: type[BaseException]) -> None:
    """Pinning a sharp edge, not endorsing it.

    `from_normalized` clamps out-of-range floats but is *not* NaN-safe: `round()`
    raises on non-finite input. This is an encode path on the app side, not a
    decode path on the car, so raising is acceptable -- but it means the input
    chain owns finiteness. A one-euro filter fed a divide-by-zero dt will emit
    NaN, and it must be caught there, not here.
    """
    with pytest.raises(exc):
        ControlPacket.from_normalized(1, 1, 1, bad, 0.0, 0.0)
    with pytest.raises(exc):
        ControlPacket.from_normalized(1, 1, 1, 0.0, bad, 0.0)
    with pytest.raises(exc):
        ControlPacket.from_normalized(1, 1, 1, 0.0, 0.0, bad)


def test_from_normalized_masks_counters() -> None:
    pkt = ControlPacket.from_normalized(1, 1 << 32, (1 << 64) + 5, 0.0, 0.0, 0.0)
    assert pkt.sequence == 0
    assert pkt.client_time_us == 5


def test_estop_property() -> None:
    assert ControlPacket(1, 1, 1, flags=ControlFlags.ESTOP).estop is True
    assert ControlPacket(1, 1, 1, flags=ControlFlags.HORN).estop is False


def test_packet_is_immutable() -> None:
    # Immutability is what lets the single-slot mailbox between the network
    # thread and the control thread be lock-free.
    with pytest.raises(dataclasses.FrozenInstanceError):
        GOLDEN_CONTROL.throttle = 1  # type: ignore[misc]


# ----------------------------------------------------------- rejection


def test_wrong_key_is_rejected() -> None:
    with pytest.raises(ProtocolError, match="authentication"):
        ControlPacket.unpack(GOLDEN_CONTROL_BYTES, OTHER_UDP_KEY)


@pytest.mark.parametrize("index", range(CONTROL_PACKET_LEN))
def test_single_bit_tamper_anywhere_is_rejected(index: int) -> None:
    # Covers the body, the reserved field, and the tag itself -- every byte.
    with pytest.raises(ProtocolError):
        ControlPacket.unpack(flip_bit(GOLDEN_CONTROL_BYTES, index), GOLDEN_UDP_KEY)


def test_bad_magic_is_rejected_before_the_hmac() -> None:
    bad = _retag(splice(GOLDEN_CONTROL_BYTES, OFF_MAGIC, b"\x00\x00\x00\x00"))
    with pytest.raises(ProtocolError, match="magic"):
        ControlPacket.unpack(bad, GOLDEN_UDP_KEY)


def test_telemetry_magic_is_not_accepted_as_control() -> None:
    from telekart_protocol.constants import MAGIC_TELEMETRY

    bad = _retag(splice(GOLDEN_CONTROL_BYTES, OFF_MAGIC, MAGIC_TELEMETRY.to_bytes(4, "little")))
    with pytest.raises(ProtocolError, match="magic"):
        ControlPacket.unpack(bad, GOLDEN_UDP_KEY)


@pytest.mark.parametrize("version", [0, 1, 3, 255, 65535])
def test_version_mismatch_is_rejected(version: int) -> None:
    if version == PROTO_VERSION:
        pytest.skip("that is the current version")
    bad = _retag(splice(GOLDEN_CONTROL_BYTES, OFF_VERSION, version.to_bytes(2, "little")))
    with pytest.raises(ProtocolError, match="version"):
        ControlPacket.unpack(bad, GOLDEN_UDP_KEY)


@pytest.mark.parametrize("length", [0, 1, 39, 41, 64, 128])
def test_wrong_length_is_rejected(length: int) -> None:
    data = (GOLDEN_CONTROL_BYTES + bytes(128))[:length]
    with pytest.raises(ProtocolError, match="bytes"):
        ControlPacket.unpack(data, GOLDEN_UDP_KEY)


def test_protocol_error_is_a_value_error() -> None:
    # Callers in the datagram path catch ValueError as a backstop.
    assert issubclass(ProtocolError, ValueError)


def test_garbage_only_ever_raises_protocol_error() -> None:
    # The decode path must never surprise the receive loop with struct.error,
    # UnicodeDecodeError, or anything else it does not catch.
    rng = random.Random(20260816)
    for _ in range(2000):
        size = rng.choice([0, 1, 20, 39, 40, 41, 200])
        data = bytes(rng.getrandbits(8) for _ in range(size))
        try:
            ControlPacket.unpack(data, GOLDEN_UDP_KEY)
        except ProtocolError:
            pass


# ------------------------------------------------- decode-side clamping


def test_out_of_range_wire_values_are_clamped_not_rejected() -> None:
    # A correctly-signed packet with an implausible axis value is more likely a
    # sender bug than an attack. Dropping it would stall the control stream and
    # trip the failsafe, which is worse than clamping.
    tampered = splice(GOLDEN_CONTROL_BYTES, OFF_STEERING, struct.pack("<hhh", 5000, 30000, -5))
    pkt = ControlPacket.unpack(_retag(tampered), GOLDEN_UDP_KEY)
    assert pkt.steering == STEERING_SCALE
    assert pkt.throttle == THROTTLE_SCALE
    assert pkt.brake == 0


def test_negative_steering_beyond_scale_is_clamped() -> None:
    tampered = splice(GOLDEN_CONTROL_BYTES, OFF_STEERING, struct.pack("<h", -30000))
    assert ControlPacket.unpack(_retag(tampered), GOLDEN_UDP_KEY).steering == -STEERING_SCALE


def test_nonzero_reserved_field_is_accepted() -> None:
    # Reserved is documented as "must be 0" for senders but is deliberately not
    # enforced on receive, so a future build can use it without a version bump.
    tampered = splice(GOLDEN_CONTROL_BYTES, OFF_RESERVED, b"\xef\xbe")
    assert ControlPacket.unpack(_retag(tampered), GOLDEN_UDP_KEY).steering == -250


def test_unknown_flag_bit_survives_decode() -> None:
    # A newer app may set a flag this firmware does not know. Preserving it is
    # what lets the value be forwarded to a log without loss.
    tampered = splice(GOLDEN_CONTROL_BYTES, OFF_FLAGS, struct.pack("<H", 1 << 15))
    pkt = ControlPacket.unpack(_retag(tampered), GOLDEN_UDP_KEY)
    assert int(pkt.flags) & (1 << 15)


# ------------------------------------------------------ peek_session_id


def test_peek_session_id_reads_without_authenticating() -> None:
    # Diagnostics only: it must work on a packet whose tag is garbage, because
    # "a packet arrived for a session I do not know" is worth logging cheaply.
    forged = GOLDEN_CONTROL_BYTES[:SIGNED_LEN] + bytes(MAC_TAG_LEN)
    assert peek_session_id(forged) == 0x11223344
    assert peek_session_id(GOLDEN_CONTROL_BYTES) == 0x11223344


def test_peek_session_id_rejects_wrong_shape() -> None:
    assert peek_session_id(b"") is None
    assert peek_session_id(GOLDEN_CONTROL_BYTES[:-1]) is None
    assert peek_session_id(GOLDEN_CONTROL_BYTES + b"\x00") is None
    assert peek_session_id(splice(GOLDEN_CONTROL_BYTES, OFF_MAGIC, b"\xde\xad\xbe\xef")) is None


def test_peek_session_id_matches_the_documented_offset() -> None:
    for session_id in (0, 1, 0x7FFFFFFF, 0xFFFFFFFF):
        pkt = ControlPacket(session_id=session_id, sequence=0, client_time_us=0)
        data = pkt.pack(GOLDEN_UDP_KEY)
        assert struct.unpack_from("<I", data, OFF_SESSION)[0] == session_id
        assert peek_session_id(data) == session_id
