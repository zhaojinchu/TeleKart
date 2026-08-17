"""MJPEG reader thread.

Two rules shape this file. **Newest frame wins** -- when decoding falls behind, a stale
but smooth feed is worse than a dropped one, because you steer by what you see. And
**it never raises into the driving loop**: a video dropout must degrade the HUD, never
take down the thing holding the watchdog alive.
"""

from __future__ import annotations

import io
import threading
import time
from collections import deque

import pygame
import requests

BOUNDARY_MARK = b"--"


class VideoClient:
    def __init__(self, url: str, boundary: str) -> None:
        self.url = url
        self._boundary = BOUNDARY_MARK + boundary.encode()
        self.connected = False
        self.fps = 0.0
        self.frames = 0
        self.error = ""
        self._lock = threading.Lock()
        self._surface: pygame.Surface | None = None
        self._stamps: deque[float] = deque(maxlen=40)
        self._running = True
        self._thread = threading.Thread(target=self._run, name="video", daemon=True)

    def start(self) -> "VideoClient":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False

    def latest(self) -> pygame.Surface | None:
        """The most recent decoded frame, or None if nothing has arrived yet."""
        with self._lock:
            return self._surface

    # -- internals -------------------------------------------------------------

    def _run(self) -> None:
        backoff = 0.5
        while self._running:
            try:
                self._read_stream()
                backoff = 0.5
            except Exception as exc:  # noqa: BLE001 -- the whole point is to never escape
                self.error = f"{type(exc).__name__}: {exc}"
            self.connected = False
            self.fps = 0.0
            for _ in range(int(backoff * 20)):
                if not self._running:
                    return
                time.sleep(0.05)
            backoff = min(backoff * 2, 2.0)

    def _read_stream(self) -> None:
        with requests.get(self.url, stream=True, timeout=(3.0, 5.0)) as r:
            r.raise_for_status()
            self.connected = True
            self.error = ""
            buf = bytearray()
            for chunk in r.iter_content(chunk_size=16384):
                if not self._running:
                    return
                if not chunk:
                    continue
                buf += chunk
                newest = None
                while True:
                    jpg, consumed = self._next_part(buf)
                    if consumed == 0:
                        break
                    del buf[:consumed]
                    if jpg:
                        newest = jpg  # keep only the last complete frame in this pass
                if len(buf) > 4_000_000:  # not a stream we understand; resync
                    buf.clear()
                if newest is not None:
                    self._store(newest)

    def _next_part(self, buf: bytearray) -> tuple[bytes | None, int]:
        """Pull one part off the front of ``buf``. Returns (jpeg, bytes consumed)."""
        i = buf.find(self._boundary)
        if i < 0:
            return None, 0
        j = buf.find(b"\r\n\r\n", i)
        if j < 0:
            return None, 0
        head = bytes(buf[i : j + 4]).decode("latin-1", "replace").lower()
        start = j + 4
        length = -1
        for line in head.split("\r\n"):
            if line.startswith("content-length:"):
                try:
                    length = int(line.split(":", 1)[1].strip())
                except ValueError:
                    length = -1
        if length >= 0:
            if len(buf) < start + length:
                return None, 0
            return bytes(buf[start : start + length]), start + length
        # No Content-Length: the frame runs to the next boundary.
        k = buf.find(self._boundary, start)
        if k < 0:
            return None, 0
        return bytes(buf[start:k]).rstrip(b"\r\n"), k

    def _store(self, jpg: bytes) -> None:
        if not jpg.startswith(b"\xff\xd8"):
            return
        try:
            surf = pygame.image.load(io.BytesIO(jpg), "frame.jpg")
        except Exception:  # noqa: BLE001 -- a torn frame is not a reason to stop
            return
        now = time.monotonic()
        self._stamps.append(now)
        if len(self._stamps) > 1 and self._stamps[-1] > self._stamps[0]:
            self.fps = (len(self._stamps) - 1) / (self._stamps[-1] - self._stamps[0])
        self.frames += 1
        with self._lock:
            self._surface = surf  # overwrite: the old one is already too old
