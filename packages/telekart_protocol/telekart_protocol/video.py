"""Video stream framing.

A 24-byte header ahead of each Annex-B access unit (or JPEG). Raw Annex-B would
also work -- PyAV's parser can find frame boundaries unaided -- but the header
buys a per-frame latency readout on the HUD, which is worth having every single
day of tuning, and an explicit "frames were dropped" bit so the decoder knows to
expect corruption rather than silently rendering garbage.

Layout (little-endian)::

    offset  size  field
    0       4     magic       MAGIC_VIDEO
    4       4     sequence    monotonically increasing, gaps mean drops
    8       8     pts_us      capture timestamp, CLOCK_MONOTONIC on the car
    16      2     flags       VideoFrameFlags
    18      2     codec       VideoCodec
    20      4     length      payload bytes that follow
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .constants import MAGIC_VIDEO, MAX_VIDEO_FRAME_LEN, VideoCodec, VideoFrameFlags

_HEADER = struct.Struct("<IIQHHI")
VIDEO_HEADER_LEN = _HEADER.size
assert VIDEO_HEADER_LEN == 24, f"video header layout drifted: {VIDEO_HEADER_LEN}"


class VideoProtocolError(ValueError):
    """The stream is not parseable and the connection should be torn down."""


@dataclass(frozen=True, slots=True)
class FrameHeader:
    sequence: int
    pts_us: int
    flags: VideoFrameFlags
    codec: VideoCodec
    length: int

    @property
    def keyframe(self) -> bool:
        return bool(self.flags & VideoFrameFlags.KEYFRAME)

    @property
    def dropped_before(self) -> bool:
        return bool(self.flags & VideoFrameFlags.DROPPED_BEFORE)

    def pack(self) -> bytes:
        return _HEADER.pack(
            MAGIC_VIDEO,
            self.sequence & 0xFFFFFFFF,
            self.pts_us & 0xFFFFFFFFFFFFFFFF,
            int(self.flags) & 0xFFFF,
            int(self.codec) & 0xFFFF,
            self.length,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "FrameHeader":
        if len(data) < VIDEO_HEADER_LEN:
            raise VideoProtocolError(
                f"video header is {len(data)} bytes, need {VIDEO_HEADER_LEN}"
            )
        magic, sequence, pts_us, flags, codec, length = _HEADER.unpack_from(data, 0)
        if magic != MAGIC_VIDEO:
            raise VideoProtocolError(f"bad video magic 0x{magic:08X}")
        if length > MAX_VIDEO_FRAME_LEN:
            raise VideoProtocolError(f"implausible video frame length {length}")
        try:
            codec_enum = VideoCodec(codec)
        except ValueError as exc:
            raise VideoProtocolError(f"unknown video codec {codec}") from exc
        return cls(
            sequence=sequence,
            pts_us=pts_us,
            flags=VideoFrameFlags(flags),
            codec=codec_enum,
            length=length,
        )


def pack_frame(
    sequence: int,
    pts_us: int,
    payload: bytes,
    *,
    codec: VideoCodec = VideoCodec.H264,
    keyframe: bool = False,
    dropped_before: bool = False,
) -> bytes:
    flags = VideoFrameFlags.NONE
    if keyframe:
        flags |= VideoFrameFlags.KEYFRAME
    if dropped_before:
        flags |= VideoFrameFlags.DROPPED_BEFORE
    header = FrameHeader(
        sequence=sequence,
        pts_us=pts_us,
        flags=flags,
        codec=codec,
        length=len(payload),
    )
    return header.pack() + payload


class FrameReader:
    """Reassembles length-prefixed frames from a TCP byte stream.

    Resynchronises on the magic word if the stream is ever corrupted, rather
    than wedging permanently -- a video link that recovers on its own is worth
    the twenty lines.
    """

    __slots__ = ("_buf",)

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list[tuple[FrameHeader, bytes]]:
        self._buf.extend(chunk)
        frames: list[tuple[FrameHeader, bytes]] = []
        while True:
            if len(self._buf) < VIDEO_HEADER_LEN:
                return frames
            try:
                header = FrameHeader.unpack(bytes(self._buf[:VIDEO_HEADER_LEN]))
            except VideoProtocolError:
                if not self._resync():
                    return frames
                continue
            total = VIDEO_HEADER_LEN + header.length
            if len(self._buf) < total:
                return frames
            payload = bytes(self._buf[VIDEO_HEADER_LEN:total])
            del self._buf[:total]
            frames.append((header, payload))

    def _resync(self) -> bool:
        """Drop one byte and hunt for the next magic word. False if none left."""
        needle = MAGIC_VIDEO.to_bytes(4, "little")
        idx = self._buf.find(needle, 1)
        if idx < 0:
            # Keep the last 3 bytes: a magic word may be split across chunks.
            keep = min(len(self._buf), 3)
            del self._buf[: len(self._buf) - keep]
            return False
        del self._buf[:idx]
        return True

    def reset(self) -> None:
        self._buf.clear()
