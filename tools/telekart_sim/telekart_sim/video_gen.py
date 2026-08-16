"""Synthetic camera: a rendered scene, encoded exactly the way the Pi encodes.

A simulator that handed the app raw RGB would leave the entire decode path
untested, and the decode path is where the latency bugs live. So this renders a
real scene with real optical flow, hands it to libx264 with the same
low-latency settings the Pi uses, and lets the app do the full job.

Three things in the picture are load-bearing rather than decorative:

* **A ground plane with a scrolling checkerboard.** Optical flow is what makes a
  video feed feel connected to the controls. A static test card cannot tell you
  whether your pipeline is 80 ms or 300 ms behind.
* **Speed-proportional motion blur**, strongest at the bottom of the frame where
  the ground moves fastest, because that is what a rolling-shutter camera on a
  moving car actually produces.
* **A large millisecond timestamp.** Point a phone at the screen next to the
  app window, film both, and step through the video: the difference is your
  true glass-to-glass latency. Nothing else measures that honestly.
"""

from __future__ import annotations

import contextlib
import math
from dataclasses import dataclass
from fractions import Fraction
from typing import Any

import numpy as np
from telekart_protocol import VideoCodec

from .autodrive import Track

#: 7-segment masks, bit order a b c d e f g.
_SEGMENTS: tuple[int, ...] = (
    0b1111110,  # 0
    0b0110000,  # 1
    0b1101101,  # 2
    0b1111001,  # 3
    0b0110011,  # 4
    0b1011011,  # 5
    0b1011111,  # 6
    0b1110000,  # 7
    0b1111111,  # 8
    0b1111011,  # 9
)

_SKY_TOP = np.array((86, 126, 184), dtype=np.float32)
_SKY_HORIZON = np.array((196, 214, 232), dtype=np.float32)
_HILL = np.array((94, 116, 96), dtype=np.float32)

#: Surface ids rasterized into the track texture.
_GRASS = 0
_ASPHALT = 1
_EDGE_LINE = 2
_START_LINE = 3

_PALETTE = np.array(
    (
        (58, 104, 46),  # grass
        (64, 66, 70),  # asphalt
        (232, 232, 226),  # painted edge line
        (245, 245, 245),  # start/finish
    ),
    dtype=np.float32,
)


@dataclass(slots=True)
class VideoConfig:
    width: int = 640
    height: int = 480
    fps: int = 30
    bitrate: int = 2_000_000
    gop: int = 15
    codec: str = "h264"
    fov_deg: float = 72.0
    camera_height_m: float = 0.12
    #: Positive pitches the camera DOWN. 8 degrees puts the horizon at about a
    #: third of the frame height, which is where a car camera actually sits.
    camera_pitch_down_deg: float = 8.0
    far_clip_m: float = 24.0
    max_blur_px: float = 9.0

    def validate(self) -> None:
        if self.width % 2 or self.height % 2:
            raise ValueError(
                f"H.264 needs even dimensions, got {self.width}x{self.height}"
            )
        if not 64 <= self.width <= 1920 or not 64 <= self.height <= 1080:
            raise ValueError(f"implausible video size {self.width}x{self.height}")
        if not 1 <= self.fps <= 120:
            raise ValueError(f"video fps out of range: {self.fps}")
        if self.gop < 1:
            raise ValueError(f"keyframe interval must be >= 1, got {self.gop}")
        if self.codec not in ("h264", "mjpeg"):
            raise ValueError(f"unsupported codec {self.codec!r}")
        if not 20.0 <= self.fov_deg <= 150.0:
            raise ValueError(f"field of view out of range: {self.fov_deg}")


class SceneRenderer:
    """Renders the world from the car's pose with an inverse perspective map.

    The expensive part -- turning every pixel below the horizon into a world
    coordinate -- is separable: the row decides how far ahead the ray lands and
    the column decides how far to the side. So it is two outer products and four
    multiply-adds per frame rather than a per-pixel ray cast.
    """

    def __init__(self, config: VideoConfig, track: Track, *, seed: int = 1) -> None:
        config.validate()
        self.config = config
        self.track = track
        self._rng = np.random.default_rng(seed ^ 0x51ED0005)

        width = config.width
        height = config.height
        focal = (width * 0.5) / math.tan(math.radians(config.fov_deg) * 0.5)
        cx = (width - 1) * 0.5
        cy = (height - 1) * 0.5

        pitch = math.radians(config.camera_pitch_down_deg)
        sin_p = math.sin(pitch)
        cos_p = math.cos(pitch)

        rows = np.arange(height, dtype=np.float32)
        cols = np.arange(width, dtype=np.float32)
        # Vehicle frame: X forward, Y left, Z up. Image right is -Y, down is -Z.
        z_img = -(rows - cy) / focal
        y_img = -(cols - cx) / focal
        x_rot = cos_p + z_img * sin_p
        z_rot = -sin_p + z_img * cos_p

        ground = z_rot < -1e-4
        first = int(np.argmax(ground)) if ground.any() else height
        self._horizon_row = first
        self._ground_rows = height - first
        if self._ground_rows <= 8:
            raise ValueError(
                "camera geometry puts the horizon at the bottom of the frame; "
                "raise camera_pitch_down_deg or widen the field of view"
            )

        depth = np.zeros(height, dtype=np.float32)
        depth[first:] = config.camera_height_m / -z_rot[first:]
        np.clip(depth, 0.0, config.far_clip_m * 4.0, out=depth)

        # Row-only forward reach and column-only lateral scale.
        self._forward = (depth * x_rot).astype(np.float32)[first:]
        self._depth = depth[first:]
        self._lateral = (depth[first:, None] * y_img[None, :]).astype(np.float32)

        self._sky = self._build_sky(width, height, first)
        self._grid, self._origin, self._res = self._rasterize_track(track)
        self._speckle = self._rng.random(self._grid.shape, dtype=np.float32)

        # Blur strength per ground row. Image-plane flow from forward motion
        # scales as 1/depth^2, which is why the bottom of the frame smears and
        # the horizon does not.
        flow = 1.0 / np.maximum(self._depth, 0.05) ** 2
        self._flow_profile = (flow / float(flow.max())).astype(np.float32)

        haze = np.clip(self._depth / config.far_clip_m, 0.0, 1.0) ** 1.3
        self._haze_keep = (1.0 - haze)[:, None, None].astype(np.float32)
        self._haze_add = (haze[:, None] * _SKY_HORIZON[None, :])[:, None, :].astype(
            np.float32
        )

        self._frame = np.zeros((height, width, 3), dtype=np.uint8)
        self._focal = focal

    # -- setup ---------------------------------------------------------------

    def _build_sky(self, width: int, height: int, horizon: int) -> np.ndarray:
        sky = np.zeros((max(horizon, 1), width, 3), dtype=np.float32)
        if horizon <= 0:
            return sky.astype(np.uint8)
        t = np.linspace(0.0, 1.0, horizon, dtype=np.float32)[:, None]
        sky[:] = (
            _SKY_TOP[None, None, :] * (1.0 - t[:, :, None])
            + _SKY_HORIZON[None, None, :] * t[:, :, None]
        )
        band = max(2, horizon // 14)
        # A band of distant hills. Purely so the horizon is not a hard seam --
        # a hard seam reads as a rendering bug and distracts from real ones.
        ridge = np.linspace(0.0, 1.0, width, dtype=np.float32)
        wobble = (np.sin(ridge * 11.0) * 0.5 + np.sin(ridge * 4.0) * 0.5) * 0.5 + 0.5
        for col in range(width):
            top = horizon - int(band * (0.45 + 0.55 * wobble[col]))
            sky[max(top, 0) : horizon, col] = _HILL
        return np.clip(sky, 0, 255).astype(np.uint8)

    def _rasterize_track(
        self, track: Track
    ) -> tuple[np.ndarray, tuple[float, float], float]:
        """Bake the track into a world-space surface texture.

        Per-pixel distance to a 300-segment polyline every frame would be
        hopeless; a 2 cm grid computed once is a gather. Each segment only
        touches the cells inside its own bounding box -- everything further away
        than the track half-width is grass regardless, so evaluating it would be
        arithmetic spent to confirm a foregone conclusion.
        """
        res = 0.02
        half = track.width_m * 0.5
        min_x, min_y, max_x, max_y = track.bounds()
        width_cells = int((max_x - min_x) / res) + 2
        height_cells = int((max_y - min_y) / res) + 2
        if width_cells * height_cells > 24_000_000:
            raise ValueError(
                f"track bounding box {max_x - min_x:.1f} x {max_y - min_y:.1f} m is too "
                "large to rasterize at 2 cm"
            )

        gx = (min_x + (np.arange(width_cells, dtype=np.float32) + 0.5) * res)[None, :]
        gy = (min_y + (np.arange(height_cells, dtype=np.float32) + 0.5) * res)[:, None]

        best = np.full((height_cells, width_cells), 1e9, dtype=np.float32)
        points = track.points
        reach = half + res
        for i in range(len(points) - 1):
            ax, ay = points[i]
            bx, by = points[i + 1]
            dx = bx - ax
            dy = by - ay
            length2 = dx * dx + dy * dy
            if length2 < 1e-12:
                continue
            col0 = max(0, int((min(ax, bx) - reach - min_x) / res))
            col1 = min(width_cells, int((max(ax, bx) + reach - min_x) / res) + 2)
            row0 = max(0, int((min(ay, by) - reach - min_y) / res))
            row1 = min(height_cells, int((max(ay, by) + reach - min_y) / res) + 2)
            if col0 >= col1 or row0 >= row1:
                continue
            local_x = gx[:, col0:col1]
            local_y = gy[row0:row1, :]
            t = ((local_x - ax) * dx + (local_y - ay) * dy) / length2
            np.clip(t, 0.0, 1.0, out=t)
            px = ax + t * dx
            py = ay + t * dy
            np.minimum(
                best[row0:row1, col0:col1],
                np.hypot(local_x - px, local_y - py),
                out=best[row0:row1, col0:col1],
            )

        world_x = gx
        world_y = gy
        line = 0.05
        grid = np.full(best.shape, _GRASS, dtype=np.uint8)
        grid[best <= half] = _EDGE_LINE
        grid[best <= half - line] = _ASPHALT

        # Start/finish: a band across the track, perpendicular to the start
        # heading, so laps are visible in the video without any overlay.
        sx, sy = track.start_x, track.start_y
        nx = math.cos(track.start_heading)
        ny = math.sin(track.start_heading)
        along = (world_x - sx) * nx + (world_y - sy) * ny
        band = (np.abs(along) <= 0.09) & (best <= half - line)
        grid[band] = _START_LINE

        return grid, (min_x, min_y), res

    # -- rendering -----------------------------------------------------------

    def render(
        self,
        *,
        x: float,
        y: float,
        heading: float,
        speed: float,
        yaw_rate: float,
        timestamp_ms: int,
        frame_index: int,
    ) -> np.ndarray:
        frame = self._frame
        horizon = self._horizon_row
        if horizon > 0:
            frame[:horizon] = self._sky

        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        forward = self._forward[:, None]
        lateral = self._lateral
        world_x = x + forward * cos_h - lateral * sin_h
        world_y = y + forward * sin_h + lateral * cos_h

        origin_x, origin_y = self._origin
        res = self._res
        rows, cols = self._grid.shape
        cell_x = ((world_x - origin_x) / res).astype(np.int32)
        cell_y = ((world_y - origin_y) / res).astype(np.int32)
        inside = (cell_x >= 0) & (cell_x < cols) & (cell_y >= 0) & (cell_y < rows)
        np.clip(cell_x, 0, cols - 1, out=cell_x)
        np.clip(cell_y, 0, rows - 1, out=cell_y)
        surface = self._grid[cell_y, cell_x]
        surface[~inside] = _GRASS

        # 0.5 m checkerboard on the grass: the flow reference. The +4096 offset
        # is there so a plain int32 truncation behaves like floor for negative
        # coordinates -- it is even, so the parity that makes the checker is
        # untouched, and it is a great deal cheaper than np.floor.
        checker = (
            (world_x * 2.0 + 4096.0).astype(np.int32)
            + (world_y * 2.0 + 4096.0).astype(np.int32)
        ) & 1

        shade = 0.80 + 0.20 * checker.astype(np.float32)
        speckle = self._speckle[cell_y, cell_x]
        np.copyto(shade, 0.86 + 0.14 * speckle, where=surface == _ASPHALT)
        np.copyto(shade, np.float32(1.0), where=surface == _EDGE_LINE)
        start_mask = surface == _START_LINE
        if start_mask.any():
            fine = (
                (world_x * 8.0 + 4096.0).astype(np.int32)
                + (world_y * 8.0 + 4096.0).astype(np.int32)
            ) & 1
            np.copyto(shade, 0.12 + 0.88 * fine.astype(np.float32), where=start_mask)

        ground = np.take(_PALETTE, surface, axis=0)
        ground *= shade[:, :, None]

        # Distance haze. Also hides the checkerboard aliasing that any flat
        # ground plane develops near the horizon.
        ground *= self._haze_keep
        ground += self._haze_add

        blurred = self._motion_blur(ground, speed, yaw_rate)
        np.clip(blurred, 0.0, 255.0, out=blurred)
        frame[horizon:] = blurred

        self._draw_timestamp(frame, timestamp_ms, frame_index)
        return frame

    def _motion_blur(
        self, ground: np.ndarray, speed: float, yaw_rate: float
    ) -> np.ndarray:
        """Box blur via cumulative sums, in place.

        Both passes gather with `np.take` rather than fancy indexing: on a
        (rows, cols, 3) float32 array `take` along the column axis is roughly
        three times faster, which is the difference between hitting 30 fps on a
        laptop and not.
        """
        config = self.config
        rows = ground.shape[0]
        cols = ground.shape[1]

        # Forward smear: a vertical box whose width follows the optical-flow
        # profile, so the bottom of the frame smears and the horizon stays sharp.
        span = abs(speed) / max(config.fps, 1) * self._focal
        strength = min(config.max_blur_px, span * 6.0)
        if strength >= 1.0:
            widths = (self._flow_profile * strength).astype(np.int32)
            # Flow falls off as 1/depth^2, so only the bottom of the frame ever
            # earns a blur. Cumsumming the rest is pure waste at 30 fps.
            if widths[-1] >= 1:
                active = int(np.argmax(widths >= 1))
                widths = widths[active:]
                band = ground[active:]
                height = band.shape[0]
                cumulative = np.empty((height + 1, cols, 3), dtype=np.float32)
                cumulative[0] = 0.0
                np.cumsum(band, axis=0, out=cumulative[1:])
                index = np.arange(height, dtype=np.int32)
                lo = np.maximum(index - widths, 0)
                hi = np.minimum(index + widths + 1, height)
                np.subtract(
                    np.take(cumulative, hi, axis=0),
                    np.take(cumulative, lo, axis=0),
                    out=band,
                )
                band /= (hi - lo)[:, None, None].astype(np.float32)

        # Yaw smear: horizontal and uniform, because a rotation moves the whole
        # image at nearly the same rate.
        yaw_px = abs(yaw_rate) / max(config.fps, 1) * self._focal
        k = int(min(config.max_blur_px, yaw_px))
        if k >= 1:
            cumulative = np.empty((rows, cols + 1, 3), dtype=np.float32)
            cumulative[:, 0] = 0.0
            np.cumsum(ground, axis=1, out=cumulative[:, 1:])
            index = np.arange(cols, dtype=np.int32)
            lo = np.maximum(index - k, 0)
            hi = np.minimum(index + k + 1, cols)
            np.subtract(
                np.take(cumulative, hi, axis=1),
                np.take(cumulative, lo, axis=1),
                out=ground,
            )
            ground /= (hi - lo)[None, :, None].astype(np.float32)
        return ground

    # -- overlay -------------------------------------------------------------

    def _draw_timestamp(
        self, frame: np.ndarray, timestamp_ms: int, frame_index: int
    ) -> None:
        height, width = frame.shape[0], frame.shape[1]
        digit_h = max(16, height // 8)
        digit_w = max(10, int(digit_h * 0.58))
        thickness = max(2, digit_h // 8)
        gap = max(3, digit_w // 5)
        digits = 5  # wraps every 100 s, which is longer than any latency
        pad = max(4, digit_h // 6)

        box_w = digits * digit_w + (digits - 1) * gap + 2 * pad + digit_h
        box_h = digit_h + 2 * pad
        frame[0:box_h, 0 : min(box_w, width)] = 0

        value = timestamp_ms % (10**digits)
        for position in range(digits):
            place = 10 ** (digits - 1 - position)
            digit = (value // place) % 10
            left = pad + position * (digit_w + gap)
            if left + digit_w > width:
                break
            self._draw_digit(frame, left, pad, digit_w, digit_h, thickness, digit)

        # Alternating strobe. On a filmed screen this is unambiguous even when
        # the digits are motion-blurred by the recording camera itself.
        strobe_left = pad + digits * (digit_w + gap)
        if strobe_left + digit_h <= width:
            level = 255 if (frame_index & 1) else 40
            frame[pad : pad + digit_h, strobe_left : strobe_left + digit_h] = level

        # A one-second sweep bar for sub-frame timing.
        sweep = int((timestamp_ms % 1000) / 1000.0 * (width - 4))
        frame[height - 8 : height - 2, sweep : sweep + 4] = 255

    @staticmethod
    def _draw_digit(
        frame: np.ndarray,
        left: int,
        top: int,
        width: int,
        height: int,
        thickness: int,
        digit: int,
    ) -> None:
        mask = _SEGMENTS[digit % 10]
        mid = top + height // 2
        right = left + width
        bottom = top + height
        half = thickness // 2
        if mask & 0b1000000:
            frame[top : top + thickness, left + thickness : right - thickness] = 255
        if mask & 0b0100000:
            frame[top + thickness : mid, right - thickness : right] = 255
        if mask & 0b0010000:
            frame[mid : bottom - thickness, right - thickness : right] = 255
        if mask & 0b0001000:
            frame[bottom - thickness : bottom, left + thickness : right - thickness] = (
                255
            )
        if mask & 0b0000100:
            frame[mid : bottom - thickness, left : left + thickness] = 255
        if mask & 0b0000010:
            frame[top + thickness : mid, left : left + thickness] = 255
        if mask & 0b0000001:
            frame[
                mid - half : mid - half + thickness,
                left + thickness : right - thickness,
            ] = 255


# --------------------------------------------------------------------------
# Encoding
# --------------------------------------------------------------------------


class EncoderUnavailable(RuntimeError):
    """PyAV is missing or has no usable encoder. Raised at construction only."""


@dataclass(slots=True)
class EncodedFrame:
    payload: bytes
    keyframe: bool
    pts_us: int


class FrameEncoder:
    """libx264 (or MJPEG) with the settings the Pi uses, and no container.

    No container and no muxer is the point: the app feeds these bytes straight
    into a CodecContext, which removes one to two frames of buffering compared
    with going through a demuxer.
    """

    def __init__(self, config: VideoConfig) -> None:
        config.validate()
        try:
            import av
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise EncoderUnavailable(
                "PyAV is not installed, so the simulator cannot produce video. "
                "Install it (pip install 'av>=14,<19') or run with --no-video."
            ) from exc

        self._av = av
        self.config = config
        self.codec = VideoCodec.H264 if config.codec == "h264" else VideoCodec.MJPEG

        name = "libx264" if config.codec == "h264" else "mjpeg"
        try:
            context = av.CodecContext.create(name, "w")
        except Exception as exc:
            raise EncoderUnavailable(
                f"cannot create the {name} encoder: {exc}"
            ) from exc

        context.width = config.width
        context.height = config.height
        context.pix_fmt = "yuv420p" if config.codec == "h264" else "yuvj420p"
        context.time_base = Fraction(1, 1_000_000)
        # The frame rate must be stated, not inferred. The time base is
        # microseconds so that a pts is a real capture timestamp -- but FFmpeg's
        # libx264 wrapper derives x264's fps from the time base when no frame
        # rate is set, which makes it believe the stream runs at 1,000,000 fps.
        # `tune=zerolatency` then sizes the VBV buffer as one frame's worth of
        # bitrate at that rate, which is a fraction of a bit, and rate control
        # pins QP at its maximum: the picture arrives greyscale, with both
        # chroma planes flattened to 128. Two characters of cause, an entire
        # colourless video pipeline of effect.
        context.framerate = Fraction(config.fps, 1)
        context.bit_rate = config.bitrate
        if config.codec == "h264":
            context.gop_size = config.gop
            # zerolatency turns off lookahead and B-frames; without it x264
            # holds frames back and every millisecond of that lands on the
            # driver. ultrafast is what a Pi Zero 2 W can actually sustain.
            context.options = {
                "preset": "ultrafast",
                "tune": "zerolatency",
                "profile": "baseline",
                "sliced_threads": "0",
                "x264-params": "repeat-headers=1:sync-lookahead=0:rc-lookahead=0",
            }
            context.thread_count = 1
        self._context = context
        self._pict_type_i = av.video.frame.PictureType.I
        self._pict_type_none = av.video.frame.PictureType.NONE
        self._closed = False

    @property
    def extradata(self) -> bytes:
        raw = getattr(self._context, "extradata", None)
        return bytes(raw) if raw else b""

    def encode(
        self, rgb: np.ndarray, pts_us: int, *, force_keyframe: bool = False
    ) -> list[EncodedFrame]:
        """Encode one frame. Returns whatever packets came out, possibly none."""
        if self._closed:
            return []
        frame = self._av.VideoFrame.from_ndarray(rgb, format="rgb24")
        frame = frame.reformat(format=self._context.pix_fmt)
        frame.pts = pts_us
        frame.time_base = Fraction(1, 1_000_000)
        frame.pict_type = self._pict_type_i if force_keyframe else self._pict_type_none
        return self._collect(self._context.encode(frame), pts_us)

    def flush(self) -> list[EncodedFrame]:
        if self._closed:
            return []
        try:
            return self._collect(self._context.encode(None), 0)
        except Exception:  # noqa: BLE001 - flushing a dead encoder must not throw
            return []

    def _collect(self, packets: Any, fallback_pts: int) -> list[EncodedFrame]:
        out: list[EncodedFrame] = []
        for packet in packets:
            payload = bytes(packet)
            if not payload:
                continue
            pts = packet.pts if packet.pts is not None else fallback_pts
            out.append(
                EncodedFrame(
                    payload=payload,
                    keyframe=bool(packet.is_keyframe) or self.codec is VideoCodec.MJPEG,
                    pts_us=int(pts),
                )
            )
        return out

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        # Closing a codec context that already errored raises from inside
        # libav; teardown is not a place to propagate that.
        with contextlib.suppress(Exception):
            self._context.close()
