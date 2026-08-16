"""PyAV decode, configured for latency rather than throughput."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import av
import av.codec.context
from av.video.reformatter import VideoReformatter

from telekart_protocol import FrameHeader, VideoCodec

from ..core.log import Throttle, get_logger
from .frame import PIXEL_FORMAT, FrameBundle

_log = get_logger(__name__)
_throttle = Throttle(5.0)

#: AV_CODEC_FLAG_LOW_DELAY. PyAV spells the enum member differently across the
#: versions this app supports (``LOW_DELAY`` then ``low_delay``), and the flag
#: itself is a stable FFmpeg ABI constant, so the literal is the reliable floor.
_AV_CODEC_FLAG_LOW_DELAY = 0x00080000

#: Codec-level options. ``flags=low_delay`` duplicates the attribute below on
#: purpose -- one is the PyAV path, the other survives a PyAV enum rename.
#:
#: The container-level knobs people cite for this problem (``fflags=nobuffer``,
#: ``probesize=32``, ``analyzeduration=0``) have no place here and are not set:
#: the frame protocol delivers whole access units with an explicit length, so we
#: feed a bare CodecContext and there is no demuxer to probe, buffer or guess
#: with in the first place.
CODEC_OPTIONS: dict[str, str] = {"flags": "low_delay"}

_CODEC_NAMES = {VideoCodec.H264: "h264", VideoCodec.MJPEG: "mjpeg"}

#: Rebuild the decoder after this many consecutive failures. A context that has
#: lost its reference frames does recover at the next keyframe, but one that has
#: been fed genuine garbage sometimes does not, and a rebuild costs ~1 ms.
_MAX_CONSECUTIVE_ERRORS = 30


@dataclass(frozen=True, slots=True)
class DecoderStats:
    frames: int = 0
    packets: int = 0
    errors: int = 0
    resets: int = 0
    corrupt: int = 0
    decode_ms_avg: float = 0.0
    clock_offset: float = 0.0


class VideoDecoder:
    """One codec context, one reformatter, one thread.

    ``CodecContext.decode`` is explicitly not thread-safe; this object must be
    touched only by the video thread that owns it.
    """

    __slots__ = (
        "_codec",
        "_ctx",
        "_reformatter",
        "_target",
        "_frames",
        "_packets",
        "_errors",
        "_resets",
        "_corrupt_frames",
        "_consecutive",
        "_await_keyframe",
        "_decode_sum",
        "_decode_n",
        "_clock_offset",
        "_last_sequence",
    )

    def __init__(
        self,
        codec: VideoCodec = VideoCodec.H264,
        *,
        target_size: tuple[int, int] | None = None,
    ) -> None:
        if codec not in _CODEC_NAMES:
            raise ValueError(f"unsupported video codec {codec!r}")
        self._codec = codec
        self._target = _validate_target(target_size)
        # One reformatter for the object's life: `VideoFrame.reformat` builds a
        # fresh sws context per call, which at 30 fps is 30 setups a second for
        # a conversion whose parameters never change.
        self._reformatter = VideoReformatter()
        self._ctx = self._open()
        self._frames = 0
        self._packets = 0
        self._errors = 0
        self._resets = 0
        self._corrupt_frames = 0
        self._consecutive = 0
        self._await_keyframe = True
        self._decode_sum = 0.0
        self._decode_n = 0
        self._clock_offset = 0.0
        self._last_sequence = -1

    # -- configuration ------------------------------------------------------

    def _open(self) -> av.codec.context.CodecContext:
        ctx = av.codec.context.CodecContext.create(_CODEC_NAMES[self._codec], "r")
        # NEVER 'FRAME' or 'AUTO'. Frame threading holds back `thread_count`
        # frames before emitting the first one -- 3 to 5 frames, 100-170 ms at
        # 30 fps. It is the single most common cause of "H.264 teleop feels
        # laggy", it has nothing to do with the Pi, and it is invisible in any
        # measurement that does not compare capture pts to display time.
        ctx.thread_type = "NONE"
        ctx.thread_count = 1
        ctx.flags |= _low_delay_flag()
        ctx.options = dict(CODEC_OPTIONS)
        ctx.open()
        return ctx

    def set_target_size(self, width: int, height: int) -> None:
        """Scale decoded frames to the widget on the decode thread.

        The GUI thread then does a 1:1 blit. Scaling in ``paintEvent`` instead
        would put a full-frame sws pass on the thread that has 16 ms to do
        everything else in the application.
        """
        target = _validate_target((width, height))
        if target != self._target:
            self._target = target

    @property
    def target_size(self) -> tuple[int, int] | None:
        return self._target

    @property
    def codec(self) -> VideoCodec:
        return self._codec

    def set_codec(self, codec: VideoCodec) -> None:
        if codec == self._codec:
            return
        if codec not in _CODEC_NAMES:
            raise ValueError(f"unsupported video codec {codec!r}")
        self._codec = codec
        self.reset()

    # -- decoding -----------------------------------------------------------

    def decode(
        self, header: FrameHeader, payload: bytes, recv_t: float | None = None
    ) -> list[FrameBundle]:
        """Decode one access unit. Returns the frames it produced, usually one.

        Never raises. A corrupt stream must cost frames, not the video thread:
        the picture is a convenience and the car keeps driving without it.
        """
        now = recv_t if recv_t is not None else time.perf_counter()
        self._packets += 1

        if header.dropped_before or (
            self._last_sequence >= 0 and header.sequence > self._last_sequence + 1
        ):
            # Reference frames are missing. Everything until the next keyframe
            # is decodable but may be visibly smeared, so it is flagged rather
            # than suppressed.
            self._await_keyframe = True
        if header.keyframe:
            self._await_keyframe = False
        self._last_sequence = header.sequence

        started = time.perf_counter()
        try:
            packet = av.Packet(payload)
            frames = self._ctx.decode(packet)
        except Exception as exc:  # av.FFmpegError and anything a codec surprises us with
            self._errors += 1
            self._consecutive += 1
            _throttle.log(_log, logging.WARNING, "decode", "video decode failed: %s", exc)
            if self._consecutive >= _MAX_CONSECUTIVE_ERRORS:
                self.reset()
            return []

        self._consecutive = 0
        if not frames:
            return []

        self._observe_clock(header.pts_us, now)
        bundles: list[FrameBundle] = []
        for frame in frames:
            try:
                size = _fit_inside(frame.width, frame.height, self._target)
                converted = self._reformatter.reformat(
                    frame,
                    width=size[0] if size else None,
                    height=size[1] if size else None,
                    format=PIXEL_FORMAT,
                )
                bundle = FrameBundle.from_video_frame(
                    converted,
                    sequence=header.sequence,
                    pts_us=header.pts_us,
                    recv_t=now,
                    decode_t=time.perf_counter(),
                    latency=self._latency(header.pts_us, now),
                    keyframe=header.keyframe,
                    corrupt=self._await_keyframe,
                )
            except Exception as exc:
                self._errors += 1
                _throttle.log(
                    _log, logging.ERROR, "reformat", "frame conversion failed: %s", exc
                )
                continue
            if bundle.corrupt:
                self._corrupt_frames += 1
            self._frames += 1
            bundles.append(bundle)

        elapsed = time.perf_counter() - started
        self._decode_sum += elapsed
        self._decode_n += 1
        return bundles

    def reset(self) -> None:
        """Rebuild the codec context. Safe to call at any time."""
        try:
            self._ctx.close()
        except Exception:
            pass
        self._ctx = self._open()
        self._resets += 1
        self._consecutive = 0
        self._await_keyframe = True
        self._last_sequence = -1

    def close(self) -> None:
        try:
            self._ctx.close()
        except Exception:
            pass

    # -- latency ------------------------------------------------------------

    def _observe_clock(self, pts_us: int, now: float) -> None:
        """Track the offset between the car's CLOCK_MONOTONIC and perf_counter.

        The two clocks share no epoch, so a raw subtraction is meaningless. The
        *minimum* observed difference is the best estimate of the offset,
        because the fastest frame is the one that queued the least; everything
        above it is transport latency, which is exactly what we want to
        measure. The estimate creeps upward slowly so that clock drift and a
        genuinely improving link both get followed.
        """
        if pts_us <= 0:
            return
        observed = now - pts_us / 1_000_000.0
        if self._clock_offset == 0.0 or observed < self._clock_offset:
            self._clock_offset = observed
        else:
            self._clock_offset += 1e-4  # ~0.1 ms per frame of upward creep

    def _latency(self, pts_us: int, now: float) -> float:
        if pts_us <= 0 or self._clock_offset == 0.0:
            return 0.0
        latency = now - (pts_us / 1_000_000.0 + self._clock_offset)
        return latency if latency > 0.0 else 0.0

    # -- statistics ---------------------------------------------------------

    def stats(self) -> DecoderStats:
        avg = (self._decode_sum / self._decode_n * 1000.0) if self._decode_n else 0.0
        return DecoderStats(
            frames=self._frames,
            packets=self._packets,
            errors=self._errors,
            resets=self._resets,
            corrupt=self._corrupt_frames,
            decode_ms_avg=avg,
            clock_offset=self._clock_offset,
        )


def _low_delay_flag() -> int:
    flags = av.codec.context.Flags
    for name in ("LOW_DELAY", "low_delay"):
        value = getattr(flags, name, None)
        if value is not None:
            return int(value)
    return _AV_CODEC_FLAG_LOW_DELAY


def _validate_target(target: tuple[int, int] | None) -> tuple[int, int] | None:
    if target is None:
        return None
    width, height = target
    if width <= 0 or height <= 0:
        return None
    # sws is happiest on even dimensions and a widget size is arbitrary;
    # rounding here is cheaper than a chroma-alignment surprise later.
    return (width - (width % 2), height - (height % 2))


def _fit_inside(
    src_w: int, src_h: int, box: tuple[int, int] | None
) -> tuple[int, int] | None:
    """Largest size with the SOURCE aspect ratio that fits inside ``box``.

    The target size is a *widget*, and a widget is whatever shape the driver
    dragged the window into. Scaling straight to it stretches a 4:3 camera into
    16:10 -- which is not a cosmetic problem on a driving screen, because the
    apparent radius of a corner and the apparent offset of the car in its lane
    are exactly the two things the driver is reading off the picture.

    Letterboxing still happens in the view, and the blit is still 1:1: the
    frame now matches one of the widget's dimensions exactly and is smaller on
    the other, so ``VideoView`` computes a scale of 1.0 and draws without a
    resampling pass.
    """
    if box is None or src_w <= 0 or src_h <= 0:
        return box
    box_w, box_h = box
    scale = min(box_w / src_w, box_h / src_h)
    width = max(2, int(src_w * scale + 0.5))
    height = max(2, int(src_h * scale + 0.5))
    return (width - (width % 2), height - (height % 2))
