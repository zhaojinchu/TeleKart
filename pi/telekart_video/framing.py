"""Sender-side helpers around the shared video framing.

The wire format itself is `telekart_protocol.video` and lives there because the
desktop decoder has to agree with it byte for byte. What this module adds is
only what a *sender* needs and a receiver does not: sequence allocation, and a
way to emit the header separately from the payload.

That separation matters. One capture is handed to every connected client, but
the DROPPED_BEFORE bit is per-client -- one viewer's link can be behind while
another's is not. Building the 24-byte header per client and writing it ahead
of the shared payload buffer keeps the encoded frame itself a single object no
matter how many clients exist.
"""

from __future__ import annotations

from telekart_protocol.constants import MAX_VIDEO_FRAME_LEN, VideoCodec, VideoFrameFlags
from telekart_protocol.video import VIDEO_HEADER_LEN, FrameHeader, pack_frame

__all__ = [
    "MAX_PAYLOAD_LEN",
    "VIDEO_HEADER_LEN",
    "FrameSequencer",
    "frame_bytes",
    "frame_flags",
    "header_bytes",
    "is_sendable",
]

#: Payload ceiling enforced by the decoder's header validation. A frame past it
#: is unsendable, so it is dropped at the source rather than tearing down a
#: connection the far end would have to re-establish.
MAX_PAYLOAD_LEN = MAX_VIDEO_FRAME_LEN

_SEQ_MASK = 0xFFFFFFFF


class FrameSequencer:
    """Allocates the capture-order sequence stamped into every frame header.

    Assigned once per capture, not once per client, so a gap in what a client
    receives says exactly how many frames its own link lost. Wraps at 2**32 to
    match the header field; at 30 fps that is a little over four and a half
    years of continuous streaming, and the receiver only ever compares
    adjacent values.
    """

    __slots__ = ("_next", "_issued")

    def __init__(self, start: int = 0) -> None:
        if not 0 <= start <= _SEQ_MASK:
            raise ValueError(f"start sequence {start} does not fit in 32 bits")
        self._next = start
        self._issued = 0

    def assign(self) -> int:
        sequence = self._next
        self._next = (sequence + 1) & _SEQ_MASK
        self._issued += 1
        return sequence

    @property
    def issued(self) -> int:
        """Total frames sequenced since construction or the last reset."""
        return self._issued

    @property
    def peek(self) -> int:
        """The sequence the next `assign` will hand out."""
        return self._next

    def reset(self, start: int = 0) -> None:
        if not 0 <= start <= _SEQ_MASK:
            raise ValueError(f"start sequence {start} does not fit in 32 bits")
        self._next = start
        self._issued = 0


def frame_flags(*, keyframe: bool = False, dropped_before: bool = False) -> VideoFrameFlags:
    flags = VideoFrameFlags.NONE
    if keyframe:
        flags |= VideoFrameFlags.KEYFRAME
    if dropped_before:
        flags |= VideoFrameFlags.DROPPED_BEFORE
    return flags


def header_bytes(
    sequence: int,
    pts_us: int,
    length: int,
    *,
    codec: VideoCodec = VideoCodec.H264,
    keyframe: bool = False,
    dropped_before: bool = False,
) -> bytes:
    """Just the 24-byte header, for callers writing header and payload apart."""
    return FrameHeader(
        sequence=sequence,
        pts_us=pts_us,
        flags=frame_flags(keyframe=keyframe, dropped_before=dropped_before),
        codec=codec,
        length=length,
    ).pack()


def frame_bytes(
    sequence: int,
    pts_us: int,
    payload: bytes,
    *,
    codec: VideoCodec = VideoCodec.H264,
    keyframe: bool = False,
    dropped_before: bool = False,
) -> bytes:
    """Header and payload in one buffer. Copies; prefer `header_bytes` on the
    serving path, where the payload is shared between clients."""
    return pack_frame(
        sequence,
        pts_us,
        payload,
        codec=codec,
        keyframe=keyframe,
        dropped_before=dropped_before,
    )


def is_sendable(length: int) -> bool:
    """False for a payload the receiver's header validation would reject.

    Checked before every send: an oversized frame is a bug or a corrupt encoder
    buffer, and dropping one frame is strictly better than raising on the
    capture path or making the client resynchronise.
    """
    return 0 < length <= MAX_PAYLOAD_LEN
