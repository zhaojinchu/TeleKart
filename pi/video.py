"""picamera2 -> MJPEG over HTTP, standard library only.

``http.server.ThreadingHTTPServer`` is enough for this: one camera producing frames, a
handful of clients consuming them. Adding aiohttp or flask would be a dependency and a
deployment step in exchange for nothing.

The shape is the standard picamera2 MJPEG example: the encoder writes JPEG frames into a
buffer, and every client thread waits on a ``threading.Condition`` for the *newest*
frame. Nobody queues frames -- a slow client gets fewer frames, never stale ones, and
can never make the camera or another client wait.
"""

from __future__ import annotations

import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import config
import protocol

try:  # pragma: no cover - absent on any machine that is not the Pi
    from picamera2 import Picamera2
    from picamera2.encoders import MJPEGEncoder
    from picamera2.outputs import FileOutput
except ImportError:  # pragma: no cover
    Picamera2 = None
    MJPEGEncoder = None
    FileOutput = None


class FrameBuffer(io.BufferedIOBase):
    """Newest-frame-wins buffer, written by the encoder and read by client threads.

    ``FileOutput`` treats this as a file: it calls ``write`` and then ``flush``, so this
    subclasses ``io.BufferedIOBase`` to inherit the rest of the file protocol rather than
    stubbing methods one AttributeError at a time. MJPEGEncoder hands over one complete
    JPEG per call, so a write *is* a frame -- no reassembly needed.
    """

    def __init__(self) -> None:
        super().__init__()
        self.frame: bytes | None = None
        self.condition = threading.Condition()
        self.count = 0
        self._recent: list[float] = []
        self._started = time.monotonic()

    def write(self, buf: bytes) -> int:
        now = time.monotonic()
        with self.condition:
            self.frame = bytes(buf)
            self.count += 1
            self._recent.append(now)
            self.condition.notify_all()
        return len(buf)

    def writable(self) -> bool:
        return True

    def wait_for_frame(self, timeout: float = 5.0) -> bytes | None:
        with self.condition:
            if not self.condition.wait(timeout):
                return None
            return self.frame

    @property
    def fps(self) -> float:
        """Frames encoded over the trailing second.

        The window is pruned on read, not only on write, so a camera that stops
        producing frames shows fps falling to zero instead of freezing at the last
        healthy number -- which is exactly the case you need telemetry to tell you about.
        """
        now = time.monotonic()
        with self.condition:
            cutoff = now - 1.0
            while self._recent and self._recent[0] < cutoff:
                self._recent.pop(0)
            if not self._recent:
                return 0.0
            span = min(1.0, now - self._started)
            return 0.0 if span <= 0 else len(self._recent) / span


class _Handler(BaseHTTPRequestHandler):
    """Serves exactly two paths. Everything else is a 404."""

    protocol_version = "HTTP/1.0"  # no keep-alive bookkeeping to get wrong
    server: "_Server"

    def do_GET(self) -> None:  # noqa: N802 - name fixed by BaseHTTPRequestHandler
        if self.path == protocol.STREAM_PATH:
            self._serve_stream()
        elif self.path == protocol.HEALTH_PATH:
            self._serve_health()
        else:
            self.send_error(404)

    def _serve_stream(self) -> None:
        buffer = self.server.buffer
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={protocol.MJPEG_BOUNDARY}",
        )
        self.end_headers()
        try:
            while True:
                frame = buffer.wait_for_frame()
                if frame is None:
                    return  # camera stopped producing; let the client reconnect
                self.wfile.write(f"--{protocol.MJPEG_BOUNDARY}\r\n".encode())
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.end_headers()
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            # A driving app that closes its window, or a Wi-Fi hiccup, is the normal
            # way a stream ends. It is not an error and must not print a traceback --
            # at 20 fps a noisy handler would bury the control-loop log.
            pass

    def _serve_health(self) -> None:
        buffer = self.server.buffer
        body = json.dumps(
            {
                "ok": True,
                "uptime": round(time.monotonic() - self.server.started_at, 1),
                "frames": buffer.count,
                "fps": round(buffer.fps, 1),
            }
        ).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except OSError:
            pass

    def log_message(self, fmt: str, *args) -> None:
        """Silence the per-request access log; the 1 Hz summary is the log that matters."""

    def handle_one_request(self) -> None:
        try:
            super().handle_one_request()
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.close_connection = True


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, handler, buffer: FrameBuffer) -> None:
        self.buffer = buffer
        self.started_at = time.monotonic()
        super().__init__(addr, handler)

    def handle_error(self, request, client_address) -> None:
        """Client disconnects are routine. Do not dump a traceback per dropped frame."""


class VideoServer:
    """Camera + HTTP server. ``start()`` / ``stop()``, both safe to call once each."""

    def __init__(self, port: int = config.VIDEO_PORT) -> None:
        self.port = port
        self.buffer = FrameBuffer()
        self.picam: object | None = None
        self._server: _Server | None = None
        self._thread: threading.Thread | None = None

    @property
    def fps(self) -> float:
        """Measured encoder output rate, reported as ``cam_fps`` in telemetry."""
        return self.buffer.fps

    @property
    def frames(self) -> int:
        return self.buffer.count

    def start(self) -> None:
        if Picamera2 is None:
            raise RuntimeError(
                "picamera2 is not importable. It comes from apt, not pip: "
                "`sudo apt install python3-picamera2`, and the venv must be created "
                "with --system-site-packages."
            )

        self.picam = Picamera2()
        cfg = self.picam.create_video_configuration(
            main={"size": (config.CAM_WIDTH, config.CAM_HEIGHT), "format": "RGB888"},
            # The important line. Asking only for 640x480 lets picamera2 pick the
            # sensor mode that matches it most cheaply, which on the IMX219 is a
            # 1280x960 *crop* -- a ~2.6x telephoto with no view of the ground ahead of
            # the wheels, effectively undrivable. Naming the full-FOV binned raw mode
            # forces the whole sensor to be read and the main stream to be a downscale
            # of it, so the driver sees what the car sees.
            raw={"size": config.CAM_RAW_SIZE},
            controls={"FrameDurationLimits": _frame_duration_limits(config.CAM_FPS)},
        )
        self.picam.configure(cfg)
        self.picam.start_recording(_encoder(config.CAM_JPEG_QUALITY), FileOutput(self.buffer))

        self._serve()

    def _serve(self) -> None:
        self._server = _Server((config.BIND_HOST, self.port), _Handler, self.buffer)
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="video-http", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
            self._server = None
        if self.picam is not None:
            for action in (self.picam.stop_recording, self.picam.close):
                try:
                    action()
                except Exception:
                    pass
            self.picam = None
        # Wake any client thread still blocked waiting for a frame it will never get.
        with self.buffer.condition:
            self.buffer.condition.notify_all()


def _frame_duration_limits(fps: int) -> tuple[int, int]:
    """picamera2 wants microseconds per frame, min and max, to pin the rate."""
    us = int(1_000_000 / fps)
    return (us, us)


def _encoder(quality: int):
    """Build the MJPEG encoder, tolerating both picamera2 signatures.

    Older picamera2 builds take only a bitrate here; newer ones accept a JPEG quality.
    Guessing wrong is a TypeError at bring-up on the Pi, which is exactly where it is
    most annoying to debug, so try the good one and fall back to the default.
    """
    try:
        return MJPEGEncoder(q=quality)
    except TypeError:
        return MJPEGEncoder()
