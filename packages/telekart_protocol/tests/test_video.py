"""Video framing contract tests.

Resynchronisation is the reason this module exists. A video link that wedges on
one corrupt byte and stays black until someone restarts the app is a worse
failure than a second of macroblocks, and it is exactly what a naive
length-prefixed reader does. Every corruption shape that can plausibly reach the
reader is exercised below, including the nasty one -- a magic word straddling a
`recv` boundary.
"""

from __future__ import annotations

import dataclasses
import random
import struct

import pytest

from telekart_protocol.constants import (
    MAGIC_VIDEO,
    MAX_VIDEO_FRAME_LEN,
    VideoCodec,
    VideoFrameFlags,
)
from telekart_protocol.video import (
    VIDEO_HEADER_LEN,
    FrameHeader,
    FrameReader,
    VideoProtocolError,
    pack_frame,
)

_RAW = struct.Struct("<IIQHHI")

#: Deliberately contains no occurrence of the magic word.
NOISE = b"\xaa" * 30

GOLDEN_HEADER = FrameHeader(
    sequence=0x00000101,
    pts_us=0x0000000123456789,
    flags=VideoFrameFlags.KEYFRAME | VideoFrameFlags.DROPPED_BEFORE,
    codec=VideoCodec.H264,
    length=5,
)

GOLDEN_HEADER_HEX = (
    "54585644"  # magic 'TXVD'
    "01010000"  # sequence 257
    "8967452301000000"  # pts_us
    "0300"  # flags KEYFRAME | DROPPED_BEFORE
    "0000"  # codec H264
    "05000000"  # length 5
)
#: A five-byte Annex-B IDR stub, so the golden covers header+payload framing.
GOLDEN_PAYLOAD = bytes.fromhex("0000000165")
GOLDEN_FRAME_HEX = GOLDEN_HEADER_HEX + GOLDEN_PAYLOAD.hex()
GOLDEN_FRAME_BYTES = bytes.fromhex(GOLDEN_FRAME_HEX)


def _raw_header(
    magic: int = MAGIC_VIDEO,
    sequence: int = 1,
    pts_us: int = 0,
    flags: int = 0,
    codec: int = 0,
    length: int = 0,
) -> bytes:
    """Build a header byte-for-byte, bypassing FrameHeader's own validation."""
    return _RAW.pack(magic, sequence, pts_us, flags, codec, length)


# ---------------------------------------------------------------- layout


def test_struct_size_is_pinned() -> None:
    assert VIDEO_HEADER_LEN == 24
    assert struct.calcsize("<IIQHHI") == 24


def test_magic_is_the_documented_ascii() -> None:
    assert MAGIC_VIDEO.to_bytes(4, "little") == b"TXVD"


def test_frame_limit_is_pinned() -> None:
    assert MAX_VIDEO_FRAME_LEN == 4 * 1024 * 1024


def test_defined_bits_fit_their_wire_fields() -> None:
    assert max(int(f) for f in VideoFrameFlags) < 1 << 16
    assert max(int(c) for c in VideoCodec) < 1 << 16


# --------------------------------------------------------------- golden


def test_golden_header_pack() -> None:
    assert GOLDEN_HEADER.pack().hex() == GOLDEN_HEADER_HEX


def test_golden_header_unpack() -> None:
    header = FrameHeader.unpack(bytes.fromhex(GOLDEN_HEADER_HEX))
    assert header == GOLDEN_HEADER
    assert header.sequence == 257
    assert header.pts_us == 0x0000000123456789
    assert header.codec is VideoCodec.H264
    assert header.length == 5
    assert header.keyframe is True
    assert header.dropped_before is True


def test_golden_frame_matches_pack_frame() -> None:
    assert (
        pack_frame(
            0x00000101,
            0x0000000123456789,
            GOLDEN_PAYLOAD,
            keyframe=True,
            dropped_before=True,
        ).hex()
        == GOLDEN_FRAME_HEX
    )


def test_golden_frame_reads_back() -> None:
    frames = FrameReader().feed(GOLDEN_FRAME_BYTES)
    assert len(frames) == 1
    header, payload = frames[0]
    assert header == GOLDEN_HEADER
    assert payload == GOLDEN_PAYLOAD


def test_header_ignores_trailing_bytes() -> None:
    # `unpack_from` reads the first 24 bytes; the payload follows in the same
    # buffer, so the header parse must not object to it.
    assert FrameHeader.unpack(GOLDEN_FRAME_BYTES) == GOLDEN_HEADER


def test_header_is_immutable() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        GOLDEN_HEADER.length = 0  # type: ignore[misc]


# ---------------------------------------------------------- round trips


@pytest.mark.parametrize("codec", list(VideoCodec))
@pytest.mark.parametrize("keyframe", [False, True])
@pytest.mark.parametrize("dropped", [False, True])
def test_flag_and_codec_combinations_round_trip(
    codec: VideoCodec, keyframe: bool, dropped: bool
) -> None:
    payload = b"payload"
    frames = FrameReader().feed(
        pack_frame(1, 2, payload, codec=codec, keyframe=keyframe, dropped_before=dropped)
    )
    (header, decoded), = frames
    assert header.codec is codec
    assert header.keyframe is keyframe
    assert header.dropped_before is dropped
    assert decoded == payload


def test_config_flag_round_trips() -> None:
    # SPS/PPS travels as its own frame so the decoder can be primed before the
    # first IDR arrives; `pack_frame` has no keyword for it, so it is set here.
    header = FrameHeader(
        sequence=0, pts_us=0, flags=VideoFrameFlags.CONFIG, codec=VideoCodec.H264, length=4
    )
    parsed = FrameHeader.unpack(header.pack())
    assert parsed.flags is VideoFrameFlags.CONFIG
    assert parsed.keyframe is False


def test_zero_length_payload_round_trips() -> None:
    frames = FrameReader().feed(pack_frame(9, 9, b""))
    assert frames == [(FrameHeader(9, 9, VideoFrameFlags.NONE, VideoCodec.H264, 0), b"")]


def test_large_payload_round_trips() -> None:
    payload = bytes(random.Random(1).getrandbits(8) for _ in range(65536))
    frames = FrameReader().feed(pack_frame(1, 1, payload, keyframe=True))
    assert frames[0][1] == payload


def test_counter_fields_are_masked() -> None:
    header = FrameHeader.unpack(
        FrameHeader(
            sequence=(1 << 32) + 3,
            pts_us=(1 << 64) + 5,
            flags=VideoFrameFlags.NONE,
            codec=VideoCodec.H264,
            length=0,
        ).pack()
    )
    assert header.sequence == 3
    assert header.pts_us == 5


def test_sequence_gaps_are_visible_to_the_decoder() -> None:
    # The reader does not renumber or interpolate: a gap in `sequence` is how
    # the app knows frames were lost even when DROPPED_BEFORE was not set.
    stream = pack_frame(10, 0, b"a") + pack_frame(14, 0, b"b", dropped_before=True)
    frames = FrameReader().feed(stream)
    assert [h.sequence for h, _ in frames] == [10, 14]
    assert frames[1][0].dropped_before is True


# ------------------------------------------------------------- framing


def test_reader_returns_nothing_on_a_partial_header() -> None:
    reader = FrameReader()
    assert reader.feed(GOLDEN_FRAME_BYTES[:10]) == []
    assert reader.feed(GOLDEN_FRAME_BYTES[10:VIDEO_HEADER_LEN]) == []
    assert len(reader.feed(GOLDEN_FRAME_BYTES[VIDEO_HEADER_LEN:])) == 1


def test_reader_reassembles_a_byte_at_a_time() -> None:
    reader = FrameReader()
    collected: list[tuple[FrameHeader, bytes]] = []
    for index in range(len(GOLDEN_FRAME_BYTES)):
        collected.extend(reader.feed(GOLDEN_FRAME_BYTES[index : index + 1]))
    assert len(collected) == 1
    assert collected[0][1] == GOLDEN_PAYLOAD


def test_reader_coalesces_a_whole_gop_in_one_chunk() -> None:
    stream = b"".join(
        pack_frame(n, n * 33333, bytes([n]) * (n + 1), keyframe=(n == 0)) for n in range(15)
    )
    frames = FrameReader().feed(stream)
    assert [h.sequence for h, _ in frames] == list(range(15))
    assert frames[0][0].keyframe is True
    assert frames[3][1] == b"\x03" * 4


@pytest.mark.parametrize("split", range(1, 60, 3))
def test_reader_survives_a_split_at_any_offset(split: int) -> None:
    stream = pack_frame(1, 1, b"first-frame") + pack_frame(2, 2, b"second-frame")
    reader = FrameReader()
    frames = reader.feed(stream[:split])
    frames += reader.feed(stream[split:])
    assert [payload for _, payload in frames] == [b"first-frame", b"second-frame"]


# -------------------------------------------------------------- resync


def test_reader_skips_leading_garbage() -> None:
    frames = FrameReader().feed(NOISE + GOLDEN_FRAME_BYTES)
    assert len(frames) == 1
    assert frames[0][1] == GOLDEN_PAYLOAD


def test_reader_recovers_from_garbage_between_frames() -> None:
    stream = pack_frame(1, 1, b"before") + NOISE + pack_frame(2, 2, b"after")
    frames = FrameReader().feed(stream)
    assert [payload for _, payload in frames] == [b"before", b"after"]


def test_reader_buffers_only_a_partial_magic_when_nothing_matches() -> None:
    # No magic in sight: the reader must not keep growing its buffer, but it
    # also must not discard a magic word that has only partly arrived.
    reader = FrameReader()
    assert reader.feed(NOISE) == []
    frames = reader.feed(GOLDEN_FRAME_BYTES)
    assert len(frames) == 1


def test_reader_recovers_when_the_magic_straddles_a_chunk_boundary() -> None:
    # The genuinely nasty case: `recv` returns garbage whose last two bytes are
    # the first half of the next frame's magic. Anything that discards the whole
    # buffer here loses a frame every time the link glitches.
    magic = MAGIC_VIDEO.to_bytes(4, "little")
    reader = FrameReader()
    assert reader.feed(b"\xaa" * 28 + magic[:2]) == []
    frames = reader.feed(magic[2:] + GOLDEN_FRAME_BYTES[4:])
    assert len(frames) == 1
    assert frames[0][0] == GOLDEN_HEADER
    assert frames[0][1] == GOLDEN_PAYLOAD


def test_reader_recovers_from_an_implausible_length() -> None:
    # A corrupted length field is the one that would otherwise make the reader
    # wait forever for bytes that are never coming.
    poisoned = _raw_header(length=MAX_VIDEO_FRAME_LEN + 1)
    frames = FrameReader().feed(poisoned + GOLDEN_FRAME_BYTES)
    assert len(frames) == 1
    assert frames[0][1] == GOLDEN_PAYLOAD


def test_reader_recovers_from_an_unknown_codec() -> None:
    poisoned = _raw_header(codec=0x7777)
    frames = FrameReader().feed(poisoned + GOLDEN_FRAME_BYTES)
    assert len(frames) == 1
    assert frames[0][0].codec is VideoCodec.H264


def test_reader_advances_past_a_false_magic_inside_garbage() -> None:
    # A magic word turns up inside compressed payload bytes by chance about once
    # every 4 GB. The reader will lock onto it, fail the codec check, and must
    # then step forward -- resyncing from index 1, never from index 0, which is
    # what stops it looping on the same false start forever.
    false_start = b"\xaa" * 8 + _raw_header(codec=0x4242, length=7) + b"\xaa" * 3
    frames = FrameReader().feed(false_start + GOLDEN_FRAME_BYTES)
    assert [payload for _, payload in frames] == [GOLDEN_PAYLOAD]


def test_reset_discards_a_partial_frame() -> None:
    reader = FrameReader()
    assert reader.feed(GOLDEN_FRAME_BYTES[:12]) == []
    reader.reset()
    frames = reader.feed(GOLDEN_FRAME_BYTES)
    assert len(frames) == 1


def test_reader_never_raises_on_random_bytes() -> None:
    # The video thread catches VideoProtocolError to tear the connection down;
    # anything else would escape and kill the thread silently.
    rng = random.Random(90210)
    reader = FrameReader()
    for _ in range(300):
        chunk = bytes(rng.getrandbits(8) for _ in range(rng.randint(1, 200)))
        for _header, payload in reader.feed(chunk):
            assert isinstance(payload, bytes)


def test_random_garbage_never_starves_a_following_frame() -> None:
    rng = random.Random(1337)
    for _ in range(200):
        noise_len = rng.randint(1, 200)
        noise = bytes(rng.getrandbits(8) for _ in range(noise_len))
        if MAGIC_VIDEO.to_bytes(4, "little") in noise:
            continue  # a chance magic makes the expected outcome ambiguous
        frames = FrameReader().feed(noise + GOLDEN_FRAME_BYTES)
        assert frames[-1][1] == GOLDEN_PAYLOAD


# ----------------------------------------------------------- rejection


@pytest.mark.parametrize("length", [0, 1, 23])
def test_short_header_is_rejected(length: int) -> None:
    with pytest.raises(VideoProtocolError, match="need 24"):
        FrameHeader.unpack(GOLDEN_FRAME_BYTES[:length])


def test_bad_magic_is_rejected() -> None:
    with pytest.raises(VideoProtocolError, match="magic"):
        FrameHeader.unpack(_raw_header(magic=0xDEADBEEF))


def test_implausible_length_is_rejected() -> None:
    with pytest.raises(VideoProtocolError, match="implausible"):
        FrameHeader.unpack(_raw_header(length=MAX_VIDEO_FRAME_LEN + 1))
    # The limit itself is inclusive.
    assert FrameHeader.unpack(_raw_header(length=MAX_VIDEO_FRAME_LEN)).length == MAX_VIDEO_FRAME_LEN


def test_unknown_codec_is_rejected() -> None:
    with pytest.raises(VideoProtocolError, match="codec"):
        FrameHeader.unpack(_raw_header(codec=99))


def test_unknown_frame_flag_survives() -> None:
    # Flags are forward-compatible; codecs are not, because a codec we cannot
    # name is a stream we cannot decode.
    header = FrameHeader.unpack(_raw_header(flags=1 << 15))
    assert int(header.flags) & (1 << 15)


def test_video_protocol_error_is_a_value_error() -> None:
    assert issubclass(VideoProtocolError, ValueError)
