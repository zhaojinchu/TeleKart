"""Frame ownership. The one place a QImage may be built over a foreign buffer."""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any

import av
from PySide6.QtGui import QImage

#: bgra in memory, read as 32-bit little-endian, is exactly Qt's RGB32. Choosing
#: it means the GUI thread blits without converting: RGB888 would be one byte
#: narrower on the wire between threads but would cost a full-frame format
#: conversion inside every paintEvent, which is the opposite of the trade we
#: want -- the decode thread has time, the paint thread does not.
PIXEL_FORMAT = "bgra"
_QT_FORMAT = QImage.Format.Format_RGB32

_live_lock = threading.Lock()
_live = 0
_peak = 0


@dataclass(slots=True)
class FrameBundle:
    """Owns the decoded ``av.VideoFrame`` AND the ``QImage`` that views it.

    **Nothing else in this codebase may construct a QImage from a raw buffer.**

    ``QImage(buffer, w, h, stride, fmt)`` does not copy. It holds a bare
    pointer into memory that belongs to the AVFrame, and PyAV frees that memory
    when the last Python reference to the frame goes away. Build the QImage,
    drop the frame, keep painting: the result is a use-after-free that shows up
    as a torn frame today, a wrong colour tomorrow and a segfault in front of an
    audience -- and never in a debugger, because the freed pages usually still
    hold the old picture.

    Binding the two together in one object makes the lifetime a single fact:
    while a widget holds the bundle, the pixels behind the QImage are alive. The
    LatestBox overwriting the bundle is what releases them, which is also why
    the video widget must keep its own reference for the duration of a paint
    rather than calling ``peek()`` twice.

    ``memoryview`` of the plane is held as well as the frame. Belt and braces:
    the exported buffer keeps the plane pinned even if a future PyAV changes how
    frames release their data.
    """

    frame: av.VideoFrame
    image: QImage
    buffer: memoryview
    width: int
    height: int
    sequence: int = 0
    pts_us: int = 0
    recv_t: float = 0.0  # perf_counter when the last byte arrived
    decode_t: float = 0.0  # perf_counter when the frame was ready
    #: Capture-to-decoded, using the car's pts corrected by the receiver's clock
    #: offset estimate. Zero until enough frames have arrived to estimate it.
    latency: float = 0.0
    keyframe: bool = False
    #: Frames were lost before this one and no keyframe has arrived since, so
    #: the picture may be smeared. Deliberately not a reason to hide it: a
    #: corrupted live picture is safer for a driver than a frozen clean one.
    corrupt: bool = False
    meta: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_video_frame(
        cls,
        frame: av.VideoFrame,
        *,
        sequence: int = 0,
        pts_us: int = 0,
        recv_t: float = 0.0,
        decode_t: float = 0.0,
        latency: float = 0.0,
        keyframe: bool = False,
        corrupt: bool = False,
    ) -> "FrameBundle":
        """Wrap a decoded frame. The frame must already be in ``PIXEL_FORMAT``.

        Reformatting is the decoder's job and happens on the decode thread; a
        conversion here would be a conversion on whichever thread happened to
        call, which for a GUI is the wrong one.
        """
        if frame.format.name != PIXEL_FORMAT:
            raise ValueError(
                f"FrameBundle needs {PIXEL_FORMAT}, got {frame.format.name}; "
                "reformat on the decode thread"
            )
        plane = frame.planes[0]
        view = memoryview(plane)
        image = QImage(view, frame.width, frame.height, plane.line_size, _QT_FORMAT)
        bundle = cls(
            frame=frame,
            image=image,
            buffer=view,
            width=frame.width,
            height=frame.height,
            sequence=sequence,
            pts_us=pts_us,
            recv_t=recv_t,
            decode_t=decode_t,
            latency=latency,
            keyframe=keyframe,
            corrupt=corrupt,
        )
        _track(+1)
        return bundle

    @property
    def nbytes(self) -> int:
        return self.buffer.nbytes

    def copy_image(self) -> QImage:
        """A QImage that owns its pixels, detached from this bundle.

        For the screenshot and clip-export paths only. Every per-frame display
        path must use ``image`` and keep the bundle alive instead -- copying a
        frame costs a full-resolution memcpy at 30 fps.
        """
        return self.image.copy()

    def __del__(self) -> None:
        # Only to keep the live counter honest for the tracemalloc soak test;
        # the actual lifetime is plain refcounting.
        _track(-1)


def _track(delta: int) -> None:
    global _live, _peak
    with _live_lock:
        _live += delta
        if _live > _peak:
            _peak = _live


def live_count() -> int:
    """Bundles currently alive.

    The soak test asserts this stays bounded. A steady climb is the signature
    of a widget that stored a bundle and never let it go -- which is a leak of
    whole decoded frames, several megabytes a second at 30 fps.
    """
    with _live_lock:
        return _live


def peak_count() -> int:
    with _live_lock:
        return _peak


def reset_counters() -> None:
    global _live, _peak
    with _live_lock:
        _live = 0
        _peak = 0
