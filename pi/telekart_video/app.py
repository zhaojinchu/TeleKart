"""Process entry point: supervise the camera, serve the frames, stay alive.

This process exists separately from ``telekart-control`` for two reasons that
are both about isolation rather than tidiness. The GIL is real, and encoding
work sharing an interpreter with the 100 Hz control loop would show up as loop
jitter. And on a 512 MB board the OOM killer will eventually have to choose:
the policy here and in the systemd unit makes sure it chooses this process.

Nothing in here is allowed to be fatal except a bad configuration. A camera
that will not open, a camera that stops producing frames, a client that wedges
-- all of them are recovered from in place, because a car that loses its
picture must keep driving.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any

from .camera import CameraError, CameraSource, EncodedFrame, create_source
from .config import ConfigError, VideoConfig, load_config
from .server import ServerStats, VideoServer

_log = logging.getLogger("telekart_video")

#: Housekeeping cadence: status file freshness and the stall check.
_TICK_S = 0.5


def _raise_oom_score(minimum: int) -> None:
    """Make sure the kernel reaches for this process before the control loop.

    The systemd unit already sets OOMScoreAdjust=500; this covers the
    hand-started case during bring-up. Raising the value never needs privilege
    and lowering it does, so this can only ever make the camera a *better*
    OOM candidate -- it cannot accidentally protect it at the car's expense.
    """
    path = Path("/proc/self/oom_score_adj")
    if not path.exists():
        return  # not Linux; nothing to do and nothing to warn about
    try:
        current = int(path.read_text().strip())
    except (OSError, ValueError) as exc:
        _log.warning("cannot read oom_score_adj: %s", exc)
        return
    if current >= minimum:
        return
    try:
        path.write_text(str(minimum))
    except OSError as exc:
        _log.warning("cannot raise oom_score_adj to %d: %s", minimum, exc)
    else:
        _log.info("oom_score_adj raised %d -> %d", current, minimum)


class _StatusFile:
    """A tiny JSON heartbeat the control process can read for camera health.

    Cross-process, one-directional, and best effort by design: the control
    loop needs to set CAMERA_DOWN and VIDEO_ACTIVE in telemetry, and giving it
    a file to stat is far less coupling than a socket between two processes
    that must otherwise be able to fail independently.
    """

    __slots__ = ("_path", "_enabled")

    def __init__(self, path: Path | None) -> None:
        self._path = path
        self._enabled = path is not None
        if path is None:
            return
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            _log.info("status file disabled (%s): %s", path, exc)
            self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    def write(self, payload: dict[str, Any]) -> None:
        if not self._enabled or self._path is None:
            return
        tmp = self._path.with_suffix(".tmp")
        try:
            tmp.write_text(json.dumps(payload, separators=(",", ":")), encoding="utf-8")
            # Atomic replace: a reader never sees a half-written file, which
            # would otherwise look exactly like a crashed camera.
            os.replace(tmp, self._path)
        except OSError as exc:
            _log.info("status file write failed, disabling: %s", exc)
            self._enabled = False

    def remove(self) -> None:
        if self._path is None:
            return
        try:
            self._path.unlink(missing_ok=True)
        except OSError:
            pass


class Supervisor:
    """Owns the server, the camera, and the policy for restarting the camera."""

    def __init__(self, config: VideoConfig, *, force_synthetic: bool = False) -> None:
        self._config = config
        self._force_synthetic = force_synthetic
        self._stop = asyncio.Event()
        self._source: CameraSource | None = None
        self._status = _StatusFile(config.status_path)
        self._started_at = time.monotonic()
        # Written by the camera thread, read by the event loop. No lock: there
        # is exactly one writer and these are single attribute assignments,
        # which are atomic under the GIL. Do not "fix" this into a lock.
        self.frames = 0
        self.last_frame_at = 0.0
        self.camera_ok = False
        self.source_name = "none"
        self._server = VideoServer(config)

    # -- camera thread -----------------------------------------------------

    def on_frame(self, frame: EncodedFrame) -> None:
        self.frames += 1
        self.last_frame_at = time.monotonic()
        self._server.submit_frame(frame)

    # -- lifecycle ---------------------------------------------------------

    def request_stop(self) -> None:
        self._stop.set()

    async def run(self) -> int:
        await self._server.start()
        stopper = asyncio.create_task(self._stop.wait(), name="stop")
        camera = asyncio.create_task(self._camera_loop(), name="camera")
        house = asyncio.create_task(self._housekeeping(), name="housekeeping")
        tasks = (stopper, camera, house)
        status = 0
        try:
            # Waiting on all three, not just the stop event: a supervisor task
            # that dies unexpectedly must end the process so systemd restarts
            # it, rather than leaving a listening socket serving nothing.
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            self._stop.set()
            for task in tasks:
                task.cancel()
            for task in tasks:
                try:
                    await task
                except asyncio.CancelledError:
                    pass
                except Exception:
                    _log.exception("supervisor task %s failed", task.get_name())
                    status = 1
            await self._shutdown_camera()
            await self._server.stop()
            self._status.remove()
        return status

    async def _sleep(self, seconds: float) -> bool:
        """Sleep, returning True if a stop was requested instead."""
        try:
            await asyncio.wait_for(self._stop.wait(), timeout=seconds)
        except asyncio.TimeoutError:
            return False
        return True

    async def _camera_loop(self) -> None:
        backoff = self._config.restart_backoff_s
        while not self._stop.is_set():
            source = create_source(self._config, force_synthetic=self._force_synthetic)
            try:
                # Opening the camera blocks for the better part of a second on
                # this board, so it runs off the event loop or the server stops
                # accepting connections while it happens.
                await asyncio.to_thread(source.start, self.on_frame)
            except Exception as exc:
                self.camera_ok = False
                if isinstance(exc, CameraError):
                    # Expected and self-describing: no camera, no libcamera,
                    # device busy. A traceback would only bury the message.
                    _log.error("camera unavailable: %s", exc)
                else:
                    _log.exception("unexpected failure starting the camera")
                if await self._sleep(backoff):
                    return
                # Back off so a permanently absent camera does not spin the CPU
                # the control loop is trying to use.
                backoff = min(backoff * 2.0, self._config.restart_backoff_max_s)
                continue

            self._source = source
            self.camera_ok = True
            self.source_name = source.description
            self.last_frame_at = time.monotonic()
            backoff = self._config.restart_backoff_s
            if not source.decodable:
                _log.warning("stream payload is not decodable; framing only")

            reason = await self._watch_frames()
            self.camera_ok = False
            await self._shutdown_camera()
            if self._stop.is_set():
                return
            _log.warning("restarting the camera: %s", reason)
            if await self._sleep(self._config.restart_backoff_s):
                return

    async def _watch_frames(self) -> str:
        """Block until the camera stops producing, or a stop is requested.

        A camera that has silently wedged is the failure mode that matters:
        libcamera raises nothing, the process looks healthy, and the picture
        is simply frozen. Only the absence of frames reveals it.
        """
        while not self._stop.is_set():
            if await self._sleep(_TICK_S):
                return "stopping"
            source = self._source
            if source is not None and not source.running:
                return "capture thread exited"
            idle = time.monotonic() - self.last_frame_at
            if idle > self._config.frame_timeout_s:
                return f"no frames for {idle:.1f}s"
        return "stopping"

    async def _shutdown_camera(self) -> None:
        source, self._source = self._source, None
        if source is None:
            return
        try:
            await asyncio.to_thread(source.stop)
        except Exception:
            _log.exception("camera stop failed")

    # -- reporting ---------------------------------------------------------

    async def _housekeeping(self) -> None:
        interval = self._config.stats_interval_s
        next_report = time.monotonic() + interval
        last_frames = 0
        last_out = 0
        last_at = time.monotonic()
        while not self._stop.is_set():
            if await self._sleep(_TICK_S):
                return
            now = time.monotonic()
            stats = self._server.stats()
            self._status.write(self._status_payload(now, stats))
            if now < next_report:
                continue
            elapsed = max(1e-6, now - last_at)
            fps_in = (self.frames - last_frames) / elapsed
            fps_out = (stats.frames_out - last_out) / elapsed
            _log.info(
                "camera=%s in=%.1ffps out=%.1ffps clients=%d dropped=%d %s",
                "up" if self.camera_ok else "DOWN",
                fps_in,
                fps_out,
                len(stats.clients),
                stats.frames_dropped,
                self._server.describe_clients(),
            )
            last_frames = self.frames
            last_out = stats.frames_out
            last_at = now
            next_report = now + interval

    def _status_payload(self, now: float, stats: ServerStats) -> dict[str, Any]:
        cfg = self._config
        return {
            "pid": os.getpid(),
            "uptime_s": round(now - self._started_at, 1),
            "camera_ok": self.camera_ok,
            "source": self.source_name,
            "codec": cfg.codec.name.lower(),
            "width": cfg.width,
            "height": cfg.height,
            "fps": cfg.fps,
            "port": cfg.port,
            "frames": self.frames,
            "since_frame_s": round(now - self.last_frame_at, 2)
            if self.last_frame_at
            else None,
            "clients": len(stats.clients),
            "frames_dropped": stats.frames_dropped,
            "listening": stats.listening,
        }


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="telekart-video",
        description="TeleKart camera process: capture, hardware-encode, serve over TCP.",
    )
    parser.add_argument("--config", type=Path, default=None, help="path to telekart.yaml")
    parser.add_argument(
        "--no-config", action="store_true", help="ignore config files entirely"
    )
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="generate a test pattern instead of opening the camera",
    )
    parser.add_argument("--bind", default=None, help="listen address")
    parser.add_argument("--port", type=int, default=None, help="listen port")
    parser.add_argument("--codec", choices=("h264", "mjpeg"), default=None)
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--fps", type=int, default=None)
    parser.add_argument("--bitrate", type=int, default=None, help="bits per second")
    parser.add_argument(
        "--iperiod", type=int, default=None, help="frames between IDRs (GOP length)"
    )
    parser.add_argument("--max-clients", type=int, default=None)
    parser.add_argument("--queue-depth", type=int, default=None)
    parser.add_argument(
        "--exposure-mode", choices=("fixed", "auto"), default=None,
        help="'fixed' caps exposure and raises gain to fight motion blur",
    )
    parser.add_argument("--exposure-us", type=int, default=None)
    parser.add_argument("--stats-interval", type=float, default=None, dest="stats_interval_s")
    parser.add_argument(
        "--no-status-file", action="store_true", help="do not write the health heartbeat"
    )
    parser.add_argument(
        "--log-level",
        default=os.environ.get("TELEKART_LOG_LEVEL", "INFO"),
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
    )
    return parser


def _overrides(args: argparse.Namespace) -> dict[str, Any]:
    values: dict[str, Any] = {
        "video_codec": args.codec,
        "video_width": args.width,
        "video_height": args.height,
        "video_fps": args.fps,
        "video_bitrate": args.bitrate,
        "video_iperiod": args.iperiod,
        "bind": args.bind,
        "port": args.port,
        "max_clients": args.max_clients,
        "queue_depth": args.queue_depth,
        "exposure_mode": args.exposure_mode,
        "exposure_us": args.exposure_us,
        "stats_interval_s": args.stats_interval_s,
    }
    if args.synthetic:
        values["synthetic"] = True
    if args.no_status_file:
        values["status_path"] = ""
    return {k: v for k, v in values.items() if v is not None}


def _install_signal_handlers(loop: asyncio.AbstractEventLoop, supervisor: Supervisor) -> None:
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            loop.add_signal_handler(sig, supervisor.request_stop)
        except (NotImplementedError, RuntimeError, ValueError):
            # No signal support (Windows, or a non-main thread). The process
            # still exits on KeyboardInterrupt, just less gracefully.
            _log.debug("cannot install handler for %s", sig)


async def _amain(config: VideoConfig, *, force_synthetic: bool) -> int:
    supervisor = Supervisor(config, force_synthetic=force_synthetic)
    _install_signal_handlers(asyncio.get_running_loop(), supervisor)
    _log.info("telekart-video starting: %s", config.describe())
    return await supervisor.run()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        stream=sys.stderr,
    )
    # --no-config wins over --config; a run that names both is asking for
    # defaults and should get them rather than an argument-order surprise.
    config_path = None if args.no_config else args.config
    try:
        config = load_config(
            config_path,
            explicit=config_path is not None,
            use_file=not args.no_config,
            overrides=_overrides(args),
        )
    except ConfigError as exc:
        # The only fatal class of error in this process, and it happens before
        # a single frame moves.
        _log.error("configuration error: %s", exc)
        return 2

    _raise_oom_score(config.oom_score_adj_min)

    try:
        return asyncio.run(_amain(config, force_synthetic=args.synthetic))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
