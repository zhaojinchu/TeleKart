"""Video receive thread: TCP framing, decode, publish."""

from __future__ import annotations

import logging
import socket
import threading
import time
from dataclasses import dataclass

from telekart_protocol import FrameHeader, FrameReader, TCP_VIDEO_PORT, VideoCodec

from ..core.latest_box import LatestBox
from ..core.log import Throttle, get_logger
from .decoder import VideoDecoder
from .frame import FrameBundle

_log = get_logger(__name__)
_throttle = Throttle(5.0)

#: One read should usually swallow a whole frame at 2 Mbit/s, so the framer
#: assembles rather than concatenates in most iterations.
_RECV_SIZE = 1 << 16

_STATS_WINDOW = 1.0

#: A connection shorter than this never really worked, so it is treated as a
#: failed connect for backoff purposes.
_MIN_HEALTHY_CONNECTION = 2.0


@dataclass(frozen=True, slots=True)
class VideoStats:
    connected: bool = False
    frames: int = 0
    dropped: int = 0
    errors: int = 0
    bytes_total: int = 0
    fps: float = 0.0
    bitrate: float = 0.0  # bits per second
    latency: float = 0.0
    decode_ms: float = 0.0
    last_frame_t: float = 0.0
    detail: str = ""


class VideoReceiver:
    """TCP socket plus the protocol's frame reader. No decoding here."""

    __slots__ = ("_address", "_port", "_connect_timeout", "_sock", "_reader", "_buf", "_view")

    def __init__(
        self, address: str, port: int = TCP_VIDEO_PORT, *, connect_timeout: float = 3.0
    ) -> None:
        self._address = address
        self._port = port
        self._connect_timeout = connect_timeout
        self._sock: socket.socket | None = None
        self._reader = FrameReader()
        # Reused across every read so a 30 fps stream does not allocate a
        # 64 KiB buffer per iteration.
        self._buf = bytearray(_RECV_SIZE)
        self._view = memoryview(self._buf)

    def connect(self) -> None:
        sock = socket.create_connection(
            (self._address, self._port), timeout=self._connect_timeout
        )
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            # A big receive buffer absorbs a keyframe arriving while the decode
            # thread is busy, instead of pushing back on the car's encoder.
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError:
            pass
        sock.settimeout(0.5)
        self._reader.reset()
        self._sock = sock

    def read(self) -> list[tuple[FrameHeader, bytes]]:
        """Read once. Empty list on timeout; raises OSError when the peer goes."""
        sock = self._sock
        if sock is None:
            raise OSError("video receiver is not connected")
        try:
            nbytes = sock.recv_into(self._view)
        except TimeoutError:
            return []
        if nbytes == 0:
            raise OSError("car closed the video stream")
        return self._reader.feed(self._view[:nbytes])

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        self._reader.reset()


class VideoRxThread(threading.Thread):
    """Owns the socket, the framer and the decoder, and publishes bundles.

    Everything from bytes to a blittable QImage happens here so the GUI thread
    only ever paints. Publication is depth-1 and drop-oldest by construction:
    if the UI is repainting slowly, the right thing is to skip frames, not to
    build a backlog that would show the driver a progressively older world.
    """

    def __init__(
        self,
        box: LatestBox[FrameBundle],
        *,
        codec: VideoCodec = VideoCodec.H264,
        target_size: tuple[int, int] | None = None,
        reconnect_delay: float = 1.0,
        max_reconnect_delay: float = 5.0,
        name: str = "VideoRx",
    ) -> None:
        super().__init__(name=name, daemon=True)
        self._box = box
        self._codec = codec
        self._target_size = target_size
        self._reconnect_delay = reconnect_delay
        self._max_reconnect_delay = max_reconnect_delay

        self._shutdown = threading.Event()
        self._retarget = threading.Event()
        self._lock = threading.Lock()
        self._address = ""
        self._port = TCP_VIDEO_PORT
        self._enabled = False

        self._connected = False
        self._frames = 0
        self._dropped = 0
        self._errors = 0
        self._bytes = 0
        self._fps = 0.0
        self._bitrate = 0.0
        self._latency = 0.0
        self._decode_ms = 0.0
        self._last_frame_t = 0.0
        self._detail = ""
        self._win_start = 0.0
        self._win_frames = 0
        self._win_bytes = 0
        self._last_sequence = -1

    # -- control ------------------------------------------------------------

    def set_target(self, address: str, port: int = TCP_VIDEO_PORT) -> None:
        with self._lock:
            self._address = address
            self._port = port
            self._enabled = True
        self._retarget.set()

    def clear_target(self) -> None:
        with self._lock:
            self._enabled = False
            self._address = ""
        self._retarget.set()

    def set_display_size(self, width: int, height: int) -> None:
        """Ask the decode thread to scale to the widget.

        Applied by the video thread at the top of its next iteration rather
        than immediately: the decoder belongs to that thread, and reaching into
        a live CodecContext from the GUI thread is how PyAV segfaults.
        """
        with self._lock:
            self._target_size = (width, height) if width > 0 and height > 0 else None

    def set_codec(self, codec: VideoCodec) -> None:
        with self._lock:
            self._codec = codec
        self._retarget.set()

    def stop(self) -> None:
        self._shutdown.set()
        self._retarget.set()

    # -- loop ---------------------------------------------------------------

    def run(self) -> None:
        decoder: VideoDecoder | None = None
        receiver: VideoReceiver | None = None
        delay = self._reconnect_delay
        connected_at = 0.0
        applied_target: tuple[int, int] | None = None
        applied_codec = self._codec

        while not self._shutdown.is_set():
            with self._lock:
                enabled = self._enabled
                address = self._address
                port = self._port
                target = self._target_size
                codec = self._codec

            if not enabled or not address:
                if receiver is not None:
                    receiver.close()
                    receiver = None
                    self._set_connected(False, "idle")
                    self._box.clear()
                self._retarget.wait(0.25)
                self._retarget.clear()
                continue

            if decoder is None or codec != applied_codec:
                if decoder is not None:
                    decoder.close()
                decoder = VideoDecoder(codec, target_size=target)
                applied_codec = codec
                applied_target = target
            elif target != applied_target:
                decoder.set_target_size(*(target or (0, 0)))
                applied_target = target

            if receiver is None:
                receiver = VideoReceiver(address, port)
                try:
                    receiver.connect()
                except OSError as exc:
                    receiver = None
                    self._set_connected(False, str(exc))
                    _throttle.log(
                        _log, logging.INFO, "connect", "video connect failed: %s", exc
                    )
                    if self._shutdown.wait(delay):
                        break
                    delay = min(self._max_reconnect_delay, delay * 2.0)
                    continue
                connected_at = time.perf_counter()
                self._set_connected(True, "")
                decoder.reset()
                _log.info("video stream connected to %s:%d", address, port)

            try:
                frames = receiver.read()
            except OSError as exc:
                receiver.close()
                receiver = None
                self._set_connected(False, str(exc))
                lifetime = time.perf_counter() - connected_at
                if lifetime < _MIN_HEALTHY_CONNECTION:
                    # A listener that accepts and immediately closes -- the
                    # camera process crashed, or is restarting -- would
                    # otherwise be reconnected to twenty times a second
                    # forever. Backing off on a short-lived connection, not
                    # just on a refused one, is what stops the hammering.
                    _throttle.log(
                        _log,
                        logging.INFO,
                        "flap",
                        "video connection lasted %.2fs (%s); backing off",
                        lifetime,
                        exc,
                    )
                    if self._shutdown.wait(delay):
                        break
                    delay = min(self._max_reconnect_delay, delay * 2.0)
                else:
                    delay = self._reconnect_delay
                    _log.info("video stream dropped: %s", exc)
                continue

            for header, payload in frames:
                self._account_frame(header, len(payload))
                for bundle in decoder.decode(header, payload):
                    self._publish(bundle)
                stats = decoder.stats()
                self._decode_ms = stats.decode_ms_avg
                self._errors = stats.errors

        if receiver is not None:
            receiver.close()
        if decoder is not None:
            decoder.close()
        self._set_connected(False, "stopped")
        _log.info("video receive thread exiting")

    # -- internals ----------------------------------------------------------

    def _publish(self, bundle: FrameBundle) -> None:
        now = bundle.decode_t or time.perf_counter()
        with self._lock:
            self._frames += 1
            self._last_frame_t = now
            self._latency = bundle.latency
            self._win_frames += 1
            if self._win_start == 0.0:
                self._win_start = now
            else:
                elapsed = now - self._win_start
                if elapsed >= _STATS_WINDOW:
                    self._fps = self._win_frames / elapsed
                    self._bitrate = self._win_bytes * 8.0 / elapsed
                    self._win_start = now
                    self._win_frames = 0
                    self._win_bytes = 0
        # Outside the lock: putting into the box takes the box's own lock, and
        # nesting two locks in one order here and the other order elsewhere is
        # how a deadlock gets built.
        self._box.put(bundle)

    def _account_frame(self, header: FrameHeader, size: int) -> None:
        with self._lock:
            self._bytes += size
            self._win_bytes += size
            if self._last_sequence >= 0:
                gap = header.sequence - self._last_sequence - 1
                if 0 < gap < 1000:
                    self._dropped += gap
            self._last_sequence = header.sequence
            if header.dropped_before:
                # The sender's bounded queue already told us it discarded
                # frames, which is more reliable than inferring it from gaps.
                self._dropped += 1

    def _set_connected(self, connected: bool, detail: str) -> None:
        with self._lock:
            self._connected = connected
            self._detail = detail
            if not connected:
                self._fps = 0.0
                self._bitrate = 0.0
                self._win_start = 0.0
                self._win_frames = 0
                self._win_bytes = 0
                self._last_sequence = -1

    def stats(self) -> VideoStats:
        now = time.perf_counter()
        with self._lock:
            fps = self._fps
            bitrate = self._bitrate
            if self._last_frame_t and now - self._last_frame_t > _STATS_WINDOW:
                fps = 0.0
                bitrate = 0.0
            return VideoStats(
                connected=self._connected,
                frames=self._frames,
                dropped=self._dropped,
                errors=self._errors,
                bytes_total=self._bytes,
                fps=fps,
                bitrate=bitrate,
                latency=self._latency,
                decode_ms=self._decode_ms,
                last_frame_t=self._last_frame_t,
                detail=self._detail,
            )
