"""Capture and hardware encoding.

Every libcamera setting in `Picamera2Source` is load-bearing and carries the
reason it is there. Most of them look like tuning and are not: they are the
difference between a stream that is 60 ms behind and one that is 200 ms behind
while still reporting 30 fps.

`picamera2` is imported lazily so this module -- and the framing and server
modules that use it -- import on a laptop, where `SyntheticSource` stands in
for the sensor.
"""

from __future__ import annotations

import abc
import logging
import threading
import time
from dataclasses import dataclass
from fractions import Fraction
from typing import Any, Callable

from telekart_protocol.constants import VideoCodec

from .config import VideoConfig

_log = logging.getLogger(__name__)


class CameraError(RuntimeError):
    """The camera could not be opened or configured. The supervisor retries."""


@dataclass(frozen=True, slots=True)
class EncodedFrame:
    """One complete access unit, already compressed, ready to frame and send."""

    data: bytes
    pts_us: int
    keyframe: bool
    codec: VideoCodec


#: Called from the encoder's own thread, once per frame. Must not block and
#: must not raise: an exception here propagates into libcamera's callback and
#: takes the capture pipeline down with it.
FrameSink = Callable[[EncodedFrame], None]


def monotonic_us() -> int:
    """CLOCK_MONOTONIC microseconds -- the time base the frame header declares."""
    return time.monotonic_ns() // 1000


class CameraSource(abc.ABC):
    """A source of encoded frames. Started once, stopped once, restartable."""

    @abc.abstractmethod
    def start(self, sink: FrameSink) -> None:
        """Begin capture. Blocks for as long as the hardware needs (~1 s on the
        Pi Zero 2 W), so the supervisor runs it off the event loop."""

    @abc.abstractmethod
    def stop(self) -> None:
        """Idempotent. Never raises; a failure to stop is logged, not thrown."""

    @property
    @abc.abstractmethod
    def running(self) -> bool: ...

    @property
    @abc.abstractmethod
    def description(self) -> str:
        """One line for the startup log, naming what is actually producing frames."""

    @property
    def decodable(self) -> bool:
        """False when the payload is filler that no decoder will render."""
        return True


# --------------------------------------------------------------------------
# picamera2
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class _Picamera2Api:
    """The handful of picamera2 names used here, resolved once."""

    Picamera2: Any
    H264Encoder: Any
    MJPEGEncoder: Any
    Output: Any
    Transform: Any | None


_api_cache: _Picamera2Api | None = None
_output_cls_cache: Any = None


def _import_picamera2() -> _Picamera2Api:
    global _api_cache
    if _api_cache is not None:
        return _api_cache
    try:
        from picamera2 import Picamera2
        from picamera2.encoders import H264Encoder, MJPEGEncoder
        from picamera2.outputs import Output
    except Exception as exc:  # ImportError, but libcamera bindings fail in creative ways
        raise CameraError(
            f"picamera2 is not importable ({exc}); install python3-picamera2 from apt "
            "and create the venv with --system-site-packages, or run with --synthetic"
        ) from exc

    transform: Any | None
    try:
        from libcamera import Transform
        transform = Transform
    except Exception:
        # Only needed for hflip/vflip. Everything else works without it.
        transform = None

    _api_cache = _Picamera2Api(
        Picamera2=Picamera2,
        H264Encoder=H264Encoder,
        MJPEGEncoder=MJPEGEncoder,
        Output=Output,
        Transform=transform,
    )
    return _api_cache


def _output_class(api: _Picamera2Api) -> Any:
    """Build the `Output` subclass picamera2 pushes encoded frames into.

    Defined here rather than at module scope because the base class only exists
    once picamera2 has imported, and this module must import without it.
    """
    global _output_cls_cache
    if _output_cls_cache is not None:
        return _output_cls_cache

    class _SinkOutput(api.Output):  # type: ignore[misc, name-defined]
        """Adapts picamera2's output protocol to a plain callable."""

        def __init__(self, sink: FrameSink, codec: VideoCodec) -> None:
            super().__init__()
            self._sink = sink
            self._codec = codec
            self.dropped = 0

        def outputframe(
            self,
            frame: Any,
            keyframe: bool = True,
            timestamp: int | None = None,
            *args: Any,
            **kwargs: Any,
        ) -> None:
            # The signature has grown across picamera2 releases (packet, audio,
            # ...). Swallowing the extras keeps this working across the apt
            # package on Bookworm and anything newer.
            try:
                if type(frame) is not bytes:
                    # The encoder recycles its buffer the moment this returns,
                    # so a memoryview handed downstream is a use-after-free
                    # that shows up as intermittently corrupt video.
                    frame = bytes(frame)
                self._sink(
                    EncodedFrame(
                        data=frame,
                        pts_us=_pts_from(timestamp),
                        keyframe=bool(keyframe),
                        codec=self._codec,
                    )
                )
            except Exception:
                # This runs on libcamera's encoder thread. Letting anything
                # escape kills the pipeline and the process never notices.
                self.dropped += 1
                _log.exception("video sink raised; frame dropped")

    _output_cls_cache = _SinkOutput
    return _SinkOutput


#: Widest gap tolerated between an encoder-supplied timestamp and our own clock
#: before the timestamp is treated as unusable (wrong epoch or wrong unit).
_PTS_SANITY_US = 5_000_000


def _pts_from(timestamp: int | None) -> int:
    """Prefer the capture timestamp, fall back to now.

    picamera2 hands down the sensor timestamp in microseconds, which is what
    the header wants: it dates the frame at capture, so the HUD's latency
    readout includes encode time instead of hiding it. Older builds pass None,
    and some pass nanoseconds. Anything implausibly far from our own monotonic
    clock is discarded rather than shipped as a wild pts that would make the
    latency display meaningless.
    """
    now = monotonic_us()
    if isinstance(timestamp, int) and abs(now - timestamp) < _PTS_SANITY_US:
        return timestamp
    return now


class Picamera2Source(CameraSource):
    """Pi Camera Module 2 through libcamera, encoded by the VideoCore block."""

    def __init__(self, config: VideoConfig) -> None:
        self._config = config
        self._picam: Any = None
        self._output: Any = None
        self._lock = threading.Lock()

    @property
    def running(self) -> bool:
        return self._picam is not None

    @property
    def description(self) -> str:
        return f"picamera2 hardware encoder, {self._config.describe()}"

    def _controls(self) -> dict[str, Any]:
        cfg = self._config
        frame_us = cfg.frame_duration_us
        controls: dict[str, Any] = {
            # THE sleeper bug of this whole process. Left unpinned, libcamera's
            # AE is free to extend frame duration in a dim room: the sensor
            # quietly drops to 15 or 10 fps, every frame arrives ~100 ms late,
            # and nothing anywhere reports an error. Pinning min == max == the
            # requested period makes the frame rate a constraint rather than a
            # suggestion, and forces AE to trade gain instead of time.
            "FrameDurationLimits": (frame_us, frame_us),
        }
        if cfg.exposure_mode == "fixed":
            # Motion blur is an exposure problem, not a codec problem. A 33 ms
            # exposure at 1.5 m/s smears 50 mm of ground across the frame, which
            # is fatal for the deferred marker-detection work and merely ugly
            # for driving. Capping exposure and buying the light back as gain
            # trades blur for noise, and sharp edges matter more than clean
            # flats here.
            controls["AeEnable"] = False
            controls["ExposureTime"] = min(cfg.exposure_us, frame_us)
            controls["AnalogueGain"] = cfg.analogue_gain
        else:
            controls["AeEnable"] = True
        controls["AwbEnable"] = cfg.awb_enable
        return controls

    def _full_fov_mode(self, picam: Any, output: tuple[int, int]) -> dict[str, Any] | None:
        """Pick a sensor mode that reads the WHOLE sensor, not a centre crop.

        Left to choose for itself, libcamera matches the requested output size
        to the nearest sensor mode -- and on an IMX219 a 640x480 request lands
        on a mode that crops 1280x960 out of a 3280x2464 array. That is a 2.6x
        crop, so the picture arrives looking zoomed in with a telephoto field of
        view, which is exactly wrong for driving: you want to see the kerb you
        are about to hit, not a close-up of the floor.

        So ask for a raw stream whose crop covers the full array, and let the
        ISP downscale. Chosen by measuring the crop rectangle rather than
        hardcoding 1640x1232, because the same code has to be right on an
        IMX708 or an OV5647 with entirely different mode tables.

        Returns None when the modes cannot be read, in which case libcamera's
        default choice stands -- a cropped picture is much better than no start.
        """
        if not self._config.full_fov:
            return None
        try:
            modes = list(picam.sensor_modes)
        except Exception as exc:  # noqa: BLE001 - vendor code, unknown failures
            _log.warning("cannot read sensor modes (%s); using libcamera's choice", exc)
            return None
        if not modes:
            return None

        def crop_area(mode: dict[str, Any]) -> int:
            crop = mode.get("crop_limits")
            if not crop or len(crop) < 4:
                return 0
            return int(crop[2]) * int(crop[3])

        widest = max((crop_area(m) for m in modes), default=0)
        if widest <= 0:
            return None

        # Among the full-FOV modes take the SMALLEST: every extra sensor pixel
        # is ISP work and memory bandwidth on a 512 MB board, and it is all
        # thrown away in the downscale to 640x480 anyway.
        full = [m for m in modes if crop_area(m) == widest and m.get("size")]
        if not full:
            return None
        chosen = min(full, key=lambda m: m["size"][0] * m["size"][1])

        size = tuple(int(v) for v in chosen["size"])
        if size[0] < output[0] or size[1] < output[1]:
            # Upscaling from a smaller full-FOV mode would trade real detail for
            # field of view. Let libcamera decide instead.
            return None

        _log.info(
            "sensor mode %dx%d selected for full field of view (crop %s)",
            size[0],
            size[1],
            chosen.get("crop_limits"),
        )
        raw: dict[str, Any] = {"size": size}
        fmt = chosen.get("unpacked") or chosen.get("format")
        if fmt is not None:
            raw["format"] = str(fmt)
        return raw

    def _make_encoder(self, api: _Picamera2Api) -> Any:
        cfg = self._config
        if cfg.codec is VideoCodec.MJPEG:
            # MJPEGEncoder is the V4L2 M2M path into the same bcm2835-codec
            # block. Do NOT swap it for JpegEncoder, which is software
            # (simplejpeg) and eats an entire A53 core at 640x480.
            if cfg.mjpeg_qp is not None:
                return api.MJPEGEncoder(bitrate=cfg.bitrate, qp=cfg.mjpeg_qp)
            return api.MJPEGEncoder(bitrate=cfg.bitrate)
        # There is no B-frame setting to hunt for below: the VideoCore H.264
        # encoder cannot produce them at all. Zero reordering delay is free
        # here, and no amount of searching will turn up a flag to disable it.
        return api.H264Encoder(
            bitrate=cfg.bitrate,
            # SPS/PPS in front of every IDR. A client that joins mid-stream, or
            # rejoins after a drop, then resynchronises from the stream itself
            # with no out-of-band parameter-set exchange to get wrong.
            repeat=True,
            # V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME is not honoured reliably by
            # bcm2835-codec, so on-demand keyframes do not exist on this
            # hardware. A short GOP is the ONLY recovery mechanism after loss,
            # which is why 15 frames (0.5 s at 30 fps) is the default rather
            # than the usual 1-2 s.
            iperiod=cfg.iperiod,
        )

    def start(self, sink: FrameSink) -> None:
        with self._lock:
            if self._picam is not None:
                return
            api = _import_picamera2()
            cfg = self._config
            try:
                picam = api.Picamera2()
            except Exception as exc:
                raise CameraError(f"cannot open the camera: {exc}") from exc

            try:
                kwargs: dict[str, Any] = {
                    # YUV420 is what both hardware encoders consume; anything
                    # else inserts an ISP conversion for nothing.
                    "main": {"size": cfg.resolution, "format": "YUV420"},
                    "controls": self._controls(),
                    # Four buffers is the low-latency, low-memory choice on a
                    # 512 MB board: enough that the ISP and the encoder never
                    # wait on each other, few enough that a stalled consumer
                    # cannot hide a third of a second of stale frames.
                    "buffer_count": cfg.buffer_count,
                    # No completed-request queue. With queue=True picamera2
                    # holds the most recent request so a later capture can be
                    # served instantly -- excellent for stills, and for a live
                    # stream it is one guaranteed frame of extra age.
                    "queue": False,
                }
                raw = self._full_fov_mode(picam, cfg.resolution)
                if raw is not None:
                    kwargs["raw"] = raw

                if (cfg.hflip or cfg.vflip) and api.Transform is not None:
                    kwargs["transform"] = api.Transform(
                        hflip=1 if cfg.hflip else 0,
                        vflip=1 if cfg.vflip else 0,
                    )
                elif cfg.hflip or cfg.vflip:
                    _log.warning("libcamera.Transform unavailable; hflip/vflip ignored")

                picam.configure(picam.create_video_configuration(**kwargs))
                encoder = self._make_encoder(api)
                output = _output_class(api)(sink, cfg.codec)
                # An explicit bitrate on the encoder makes start_recording's
                # `quality` argument a no-op, which is what we want: the link
                # budget decides the bitrate, not a preset.
                picam.start_recording(encoder, output)
            except CameraError:
                self._close(picam)
                raise
            except Exception as exc:
                self._close(picam)
                raise CameraError(f"cannot configure the camera: {exc}") from exc

            self._picam = picam
            self._output = output
            _log.info("camera started: %s", self.description)

    def stop(self) -> None:
        with self._lock:
            picam, self._picam = self._picam, None
            self._output = None
        if picam is None:
            return
        try:
            picam.stop_recording()
        except Exception:
            _log.exception("stop_recording failed; closing anyway")
        self._close(picam)
        _log.info("camera stopped")

    @staticmethod
    def _close(picam: Any) -> None:
        try:
            picam.close()
        except Exception:
            # Leaking the handle is survivable; systemd restarts this process.
            _log.exception("Picamera2.close failed")


# --------------------------------------------------------------------------
# Synthetic source
# --------------------------------------------------------------------------


class _SyntheticEncoder(abc.ABC):
    @abc.abstractmethod
    def encode(self, index: int) -> list[tuple[bytes, bool]]:
        """Return zero or more (payload, keyframe) pairs for frame `index`."""

    @property
    @abc.abstractmethod
    def decodable(self) -> bool: ...

    def close(self) -> None:
        return None


class _FillerEncoder(_SyntheticEncoder):
    """Correctly sized, correctly paced, deliberately meaningless payloads.

    Exists so the framing, queueing, drop and eviction logic can be exercised
    end to end on a machine with no camera and no codec at all. The byte
    pattern is deterministic so a test can assert on it.
    """

    _MARKER = b"TKSYNTH\x00"

    def __init__(self, config: VideoConfig) -> None:
        average = max(64, config.bitrate // (8 * config.fps))
        if config.codec is VideoCodec.MJPEG:
            # Every JPEG stands alone: an MJPEG stream is all keyframes and all
            # frames are about the same size. Claiming otherwise would make the
            # server's keyframe gate hold a client back from frames that were
            # all valid entry points.
            self._iperiod = 1
            self._p_bytes = max(256, average)
            self._k_bytes = self._p_bytes
        else:
            # Roughly the shape of a real stream: P-frames well under the
            # average, an IDR several times larger, so the bounded queue sees
            # the bursty arrival pattern the hardware encoder produces.
            self._iperiod = config.iperiod
            self._p_bytes = max(256, int(average * 0.7))
            self._k_bytes = max(2048, self._p_bytes * 5)
        self._buf = bytearray(self._k_bytes * 2)
        for i in range(len(self._buf)):
            self._buf[i] = (i * 37 + 11) & 0xFF
        self._rng = 0x2545F491

    @property
    def decodable(self) -> bool:
        return False

    def _jitter(self, size: int) -> int:
        # xorshift32: deterministic, no allocation, no random module state.
        x = self._rng
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        self._rng = x
        return size + (x % max(1, size // 4)) - size // 8

    def encode(self, index: int) -> list[tuple[bytes, bool]]:
        keyframe = index % self._iperiod == 0
        size = self._jitter(self._k_bytes if keyframe else self._p_bytes)
        size = max(64, min(size, len(self._buf)))
        view = memoryview(self._buf)[:size]
        payload = bytearray(view)
        payload[: len(self._MARKER)] = self._MARKER
        payload[8:12] = (index & 0xFFFFFFFF).to_bytes(4, "little")
        payload[12] = 1 if keyframe else 0
        return [(bytes(payload), keyframe)]


class _PyAvEncoder(_SyntheticEncoder):
    """Real, decodable frames when PyAV happens to be installed.

    Worth the code: it lets the desktop video pipeline -- decode, FrameBundle
    lifetime, latency readout -- be developed against this process instead of
    against a Pi on a bench.
    """

    def __init__(self, config: VideoConfig) -> None:
        import av  # optional; resolved at construction and never on the Pi

        self._av = av
        self._width = config.width
        self._height = config.height
        name = "mjpeg" if config.codec is VideoCodec.MJPEG else "libx264"
        ctx = av.CodecContext.create(name, "w")
        ctx.width = config.width
        ctx.height = config.height
        ctx.pix_fmt = "yuvj420p" if name == "mjpeg" else "yuv420p"
        ctx.bit_rate = config.bitrate
        ctx.time_base = Fraction(1, config.fps)
        if name == "libx264":
            ctx.gop_size = config.iperiod
            # zerolatency is the whole point: without it x264 buffers frames
            # and the synthetic stream would be less responsive than the Pi's.
            ctx.options = {"tune": "zerolatency", "preset": "ultrafast", "g": str(config.iperiod)}
        ctx.open()
        self._ctx = ctx
        self._pattern = _Pattern(config.width, config.height)

    @property
    def decodable(self) -> bool:
        return True

    def encode(self, index: int) -> list[tuple[bytes, bool]]:
        frame = self._av.VideoFrame(self._width, self._height, self._ctx.pix_fmt)
        self._pattern.render(frame, index)
        frame.pts = index
        frame.time_base = self._ctx.time_base
        out: list[tuple[bytes, bool]] = []
        for packet in self._ctx.encode(frame):
            out.append((bytes(packet), bool(packet.is_keyframe)))
        return out

    def close(self) -> None:
        try:
            self._ctx.close()
        except Exception:
            _log.debug("PyAV context close failed", exc_info=True)


class _Pattern:
    """A moving bar over a gradient: enough motion to make an encoder work.

    A static image compresses to nothing and would make every P-frame a few
    bytes, which hides exactly the queueing behaviour the synthetic mode is
    there to exercise.
    """

    __slots__ = ("_w", "_h", "_row", "_bar")

    def __init__(self, width: int, height: int) -> None:
        self._w = width
        self._h = height
        self._row = bytes((16 + (x * 219) // max(1, width - 1)) & 0xFF for x in range(width))
        self._bar = bytes([235]) * max(8, width // 20)

    def render(self, frame: Any, index: int) -> None:
        bar_w = len(self._bar)
        offset = (index * 11) % max(1, self._w - bar_w)
        luma = bytearray(self._row)
        luma[offset : offset + bar_w] = self._bar
        self._fill(frame.planes[0], self._h, bytes(luma))
        flat = bytes([128 + ((index * 3) % 32) - 16]) * (self._w // 2)
        self._fill(frame.planes[1], self._h // 2, flat)
        self._fill(frame.planes[2], self._h // 2, flat)

    @staticmethod
    def _fill(plane: Any, rows: int, row: bytes) -> None:
        # Planes are stride-padded; a plain repeat of the row would be the
        # wrong length and shear the image.
        stride = plane.line_size
        buf = bytearray(plane.buffer_size)
        n = min(stride, len(row))
        for r in range(min(rows, plane.buffer_size // max(1, stride))):
            start = r * stride
            buf[start : start + n] = row[:n]
        plane.update(bytes(buf))


class SyntheticSource(CameraSource):
    """A camera-shaped frame generator for machines with no camera."""

    def __init__(self, config: VideoConfig) -> None:
        self._config = config
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._encoder: _SyntheticEncoder = _make_synthetic_encoder(config)
        self._sink: FrameSink | None = None

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    @property
    def decodable(self) -> bool:
        return self._encoder.decodable

    @property
    def description(self) -> str:
        kind = "decodable" if self._encoder.decodable else "filler"
        return f"synthetic {kind} source, {self._config.describe()}"

    def start(self, sink: FrameSink) -> None:
        if self.running:
            return
        self._sink = sink
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="telekart-synthetic-camera", daemon=True
        )
        self._thread.start()
        if not self._encoder.decodable:
            _log.warning(
                "synthetic frames carry filler payload; the stream is framed correctly "
                "but no decoder will render it (install PyAV for decodable frames)"
            )
        _log.info("camera started: %s", self.description)

    def stop(self) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
            if thread.is_alive():
                _log.warning("synthetic camera thread did not exit")
        self._encoder.close()
        self._sink = None

    def _run(self) -> None:
        period = 1.0 / self._config.fps
        sink = self._sink
        if sink is None:
            return
        index = 0
        # Absolute deadlines, not sleep(period): the latter accumulates the
        # scheduler's error and the stream drifts slowly off the nominal rate.
        deadline = time.monotonic() + period
        while not self._stop.is_set():
            try:
                packets = self._encoder.encode(index)
            except Exception:
                _log.exception("synthetic encoder failed; falling back to filler payloads")
                self._encoder = _FillerEncoder(self._config)
                packets = []
            for payload, keyframe in packets:
                sink(
                    EncodedFrame(
                        data=payload,
                        pts_us=monotonic_us(),
                        keyframe=keyframe,
                        codec=self._config.codec,
                    )
                )
            index += 1
            now = time.monotonic()
            if deadline <= now:
                # Skip missed deadlines outright rather than sprinting to catch
                # up; a burst of frames is worse than a gap for a live view.
                deadline = now + period
            else:
                self._stop.wait(deadline - now)
                deadline += period


def _make_synthetic_encoder(config: VideoConfig) -> _SyntheticEncoder:
    try:
        return _PyAvEncoder(config)
    except ImportError:
        return _FillerEncoder(config)
    except Exception:
        _log.warning("PyAV present but unusable; using filler payloads", exc_info=True)
        return _FillerEncoder(config)


# --------------------------------------------------------------------------
# Selection
# --------------------------------------------------------------------------


def create_source(config: VideoConfig, *, force_synthetic: bool = False) -> CameraSource:
    """Pick a source. Never falls back to synthetic silently.

    On the car, a missing camera is a fault the operator has to see -- quietly
    streaming a test pattern would look like a working video link and is
    exactly the sort of thing that gets discovered at speed.
    """
    if force_synthetic or config.synthetic:
        return SyntheticSource(config)
    return Picamera2Source(config)
