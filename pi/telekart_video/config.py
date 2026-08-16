"""Configuration for the camera process, loaded without touching ``telekart``.

The camera runs in its own OS process and deliberately shares no Python state
with the control loop, so it cannot import ``telekart.config``. What it *does*
share is the parameter registry in ``telekart_protocol.params`` -- the video
group there (codec, size, fps, bitrate, iperiod) is the same set the desktop
tuning UI edits, and reading it from one place is what stops the two ends
disagreeing about what "30 fps" means.

Everything the desktop never needs to touch (exposure policy, socket tuning,
supervisor timings) lives in a ``camera:`` section of the same YAML file and is
defaulted here, because pushing it through the wire protocol would mean a
protocol change every time a socket knob moves.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from telekart_protocol.constants import TCP_VIDEO_PORT, VideoCodec
from telekart_protocol.params import PARAMS, ParamError, coerce

_log = logging.getLogger(__name__)


class ConfigError(Exception):
    """The configuration is unusable. Always fatal, always at startup."""


# Names of the video parameters that come from the shared registry. Anything
# here is settable from the desktop app, so its range lives in params.py and is
# not duplicated below.
_SHARED_PARAMS: tuple[str, ...] = (
    "video_codec",
    "video_width",
    "video_height",
    "video_fps",
    "video_bitrate",
    "video_iperiod",
)

_CODECS: dict[str, VideoCodec] = {"h264": VideoCodec.H264, "mjpeg": VideoCodec.MJPEG}

_CAMERA_DEFAULTS: dict[str, Any] = {
    # "fixed" forces a short exposure and buys the gain back; "auto" hands
    # brightness to the AE algorithm and accepts the motion blur.
    "exposure_mode": "fixed",
    "exposure_us": 6000,
    "analogue_gain": 6.0,
    "awb_enable": True,
    "hflip": False,
    "vflip": False,
    "buffer_count": 4,
    # MJPEG quality is expressed as a quantiser on the hardware JPEG block.
    # None lets the encoder pick from the bitrate alone.
    "mjpeg_qp": None,
    "synthetic": False,
    "frame_timeout_s": 3.0,
    "restart_backoff_s": 1.0,
    "restart_backoff_max_s": 30.0,
    "stats_interval_s": 10.0,
    "status_path": "/run/telekart/video-status.json",
    # The systemd unit already sets OOMScoreAdjust=500. This floor exists so a
    # hand-started camera (bring-up, debugging) inherits the same policy: on a
    # 512 MB board the kernel must reach for this process and never the car.
    "oom_score_adj_min": 500,
}

_SERVER_DEFAULTS: dict[str, Any] = {
    "bind": "0.0.0.0",
    "port": TCP_VIDEO_PORT,
    "max_clients": 2,
    # Two frames: one being written, one waiting. Three would only add a frame
    # of latency to a link that is already behind.
    "queue_depth": 2,
    "send_buffer_bytes": 64 * 1024,
    "write_timeout_s": 2.0,
    "start_on_keyframe": True,
}


@dataclass(frozen=True, slots=True)
class VideoConfig:
    """Fully resolved, validated settings. Constructed once, never mutated."""

    codec: VideoCodec
    width: int
    height: int
    fps: int
    bitrate: int
    iperiod: int

    exposure_mode: str
    exposure_us: int
    analogue_gain: float
    awb_enable: bool
    hflip: bool
    vflip: bool
    buffer_count: int
    mjpeg_qp: int | None
    synthetic: bool

    bind: str
    port: int
    max_clients: int
    queue_depth: int
    send_buffer_bytes: int
    write_timeout_s: float
    start_on_keyframe: bool

    frame_timeout_s: float
    restart_backoff_s: float
    restart_backoff_max_s: float
    stats_interval_s: float
    status_path: Path | None
    oom_score_adj_min: int

    def __post_init__(self) -> None:
        # Fail here, at construction, or not at all: nothing downstream may
        # raise once frames are moving.
        if self.exposure_mode not in ("fixed", "auto"):
            raise ConfigError(
                f"camera.exposure_mode must be 'fixed' or 'auto', got {self.exposure_mode!r}"
            )
        if not 100 <= self.exposure_us <= 100_000:
            raise ConfigError(f"camera.exposure_us out of range: {self.exposure_us}")
        if not 1.0 <= self.analogue_gain <= 32.0:
            raise ConfigError(f"camera.analogue_gain out of range: {self.analogue_gain}")
        # Below four the encoder and the ISP fight over buffers and the pipeline
        # stalls; above six the win is nil and the memory is not there.
        if not 3 <= self.buffer_count <= 8:
            raise ConfigError(f"camera.buffer_count out of range: {self.buffer_count}")
        if self.mjpeg_qp is not None and not 1 <= self.mjpeg_qp <= 100:
            raise ConfigError(f"camera.mjpeg_qp out of range: {self.mjpeg_qp}")
        if not 1 <= self.port <= 65535:
            raise ConfigError(f"camera.server.port out of range: {self.port}")
        if not self.bind:
            raise ConfigError("camera.server.bind must not be empty")
        if not 1 <= self.max_clients <= 8:
            raise ConfigError(f"camera.server.max_clients out of range: {self.max_clients}")
        if not 1 <= self.queue_depth <= 8:
            raise ConfigError(f"camera.server.queue_depth out of range: {self.queue_depth}")
        # A send buffer smaller than a keyframe turns every IDR into a stall.
        if not 8 * 1024 <= self.send_buffer_bytes <= 4 * 1024 * 1024:
            raise ConfigError(
                f"camera.server.send_buffer_bytes out of range: {self.send_buffer_bytes}"
            )
        for name in ("write_timeout_s", "frame_timeout_s", "restart_backoff_s",
                     "restart_backoff_max_s", "stats_interval_s"):
            value = getattr(self, name)
            if not 0.0 < value <= 600.0:
                raise ConfigError(f"camera.{name} out of range: {value}")
        if self.restart_backoff_max_s < self.restart_backoff_s:
            raise ConfigError("camera.restart_backoff_max_s is below restart_backoff_s")
        if not -1000 <= self.oom_score_adj_min <= 1000:
            raise ConfigError(
                f"camera.oom_score_adj_min out of range: {self.oom_score_adj_min}"
            )

    @property
    def frame_duration_us(self) -> int:
        """Exact frame period handed to FrameDurationLimits, in microseconds."""
        return round(1_000_000 / self.fps)

    @property
    def gop_seconds(self) -> float:
        """Worst-case time to recover a decodable picture after loss."""
        return self.iperiod / self.fps

    @property
    def resolution(self) -> tuple[int, int]:
        return (self.width, self.height)

    def describe(self) -> str:
        return (
            f"{self.width}x{self.height}@{self.fps} {self.codec.name.lower()} "
            f"{self.bitrate // 1000}kbps iperiod={self.iperiod} "
            f"({self.gop_seconds:.2f}s GOP) exposure={self.exposure_mode}"
        )


# --------------------------------------------------------------------------
# YAML loading
# --------------------------------------------------------------------------


def default_config_path() -> Path:
    """``pi/config/telekart.yaml``, located relative to this file.

    Resolved from ``__file__`` rather than the working directory: systemd sets
    WorkingDirectory, a developer shell does not, and a camera that silently
    runs defaults because it was started from the wrong cwd is a bad afternoon.
    """
    return Path(__file__).resolve().parent.parent / "config" / "telekart.yaml"


def _read_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ModuleNotFoundError as exc:  # pragma: no cover - environment dependent
        raise _MissingYaml(str(exc)) from exc

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read {path}: {exc}") from exc

    try:
        loaded = yaml.safe_load(text)
    except Exception as exc:  # yaml.YAMLError, but the module may be absent above
        raise ConfigError(f"{path} is not valid YAML: {exc}") from exc

    if loaded is None:
        return {}
    if not isinstance(loaded, dict):
        raise ConfigError(f"{path} must contain a mapping at the top level")
    return loaded


class _MissingYaml(Exception):
    """PyYAML is not installed. Recoverable only when no file was demanded."""


def _deep_merge(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in overlay.items():
        existing = merged.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merged[key] = _deep_merge(existing, value)
        else:
            merged[key] = value
    return merged


def _flatten_params(raw: Any) -> dict[str, Any]:
    """Accept both ``params: {video: {...}}`` and a flat ``params: {...}``.

    The checked-in config groups its parameters for readability; the grouping
    carries no meaning, so both spellings must load identically.
    """
    flat: dict[str, Any] = {}
    if not isinstance(raw, dict):
        return flat
    for key, value in raw.items():
        if isinstance(value, dict) and key not in PARAMS:
            for inner_key, inner_value in value.items():
                flat[inner_key] = inner_value
        else:
            flat[key] = value
    return flat


def _resolve_shared(params: dict[str, Any]) -> dict[str, Any]:
    """Validate the shared video params, falling back to registry defaults.

    A single bad number must not stop the camera: the default is a working
    value, and a car with a picture at the wrong bitrate beats a car with none.
    Every rejection is logged, because silently running something other than
    what the file says is the actual hazard.
    """
    resolved: dict[str, Any] = {}
    for name in _SHARED_PARAMS:
        definition = PARAMS[name]
        if name not in params:
            resolved[name] = definition.default
            continue
        try:
            resolved[name] = coerce(name, params[name])
        except ParamError as exc:
            _log.warning("ignoring %s from config: %s", name, exc)
            resolved[name] = definition.default
    return resolved


def _pick(section: dict[str, Any], defaults: dict[str, Any], prefix: str) -> dict[str, Any]:
    values = dict(defaults)
    for key, value in section.items():
        if key in defaults:
            values[key] = value
        elif key != "server":
            _log.warning("ignoring unknown key %s%s in config", prefix, key)
    return values


def _as_bool(value: Any, name: str) -> bool:
    if isinstance(value, bool):
        return value
    raise ConfigError(f"{name} must be true or false, got {value!r}")


def _as_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a number, got {value!r}")
    if int(value) != value:
        raise ConfigError(f"{name} must be a whole number, got {value!r}")
    return int(value)


def _as_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a number, got {value!r}")
    return float(value)


# --------------------------------------------------------------------------
# Environment overrides
# --------------------------------------------------------------------------

_ENV_TRUE = frozenset({"1", "true", "yes", "on"})
_ENV_FALSE = frozenset({"0", "false", "no", "off"})


def _env_bool(name: str) -> bool | None:
    raw = os.environ.get(name)
    if raw is None:
        return None
    lowered = raw.strip().lower()
    if lowered in _ENV_TRUE:
        return True
    if lowered in _ENV_FALSE:
        return False
    raise ConfigError(f"{name}={raw!r} is not a boolean")


def _env_int(name: str) -> int | None:
    raw = os.environ.get(name)
    if raw is None:
        return None
    try:
        return int(raw.strip())
    except ValueError as exc:
        raise ConfigError(f"{name}={raw!r} is not an integer") from exc


def load_config(
    path: Path | None = None,
    *,
    explicit: bool = False,
    use_file: bool = True,
    overrides: dict[str, Any] | None = None,
) -> VideoConfig:
    """Build a `VideoConfig` from YAML, environment, and caller overrides.

    Precedence, lowest to highest: registry defaults, ``telekart.yaml``,
    ``config.local.yaml`` beside it, ``TELEKART_VIDEO_*`` environment
    variables, then `overrides` (the command line).

    `explicit` means the operator named the file. A named file that cannot be
    read is fatal; an auto-discovered one that is merely absent is not, so the
    module still starts on a laptop with no ``pi/config`` tree at all.
    `use_file=False` skips file loading altogether, which is what ``--no-config``
    is for.
    """
    if path is None and use_file:
        env_path = os.environ.get("TELEKART_CONFIG")
        if env_path:
            path = Path(env_path)
            explicit = True
        else:
            path = default_config_path()

    document: dict[str, Any] = {}
    if path is None:
        _log.info("config file loading disabled; using defaults")
    elif path.exists():
        try:
            document = _read_yaml(path)
        except _MissingYaml:
            # PyYAML is a hard dependency on the car and is absent only on a
            # development machine. Refusing to start there would make the
            # synthetic mode -- the entire point of which is running without
            # the Pi -- unreachable.
            if explicit:
                raise ConfigError(
                    f"PyYAML is required to read {path}; install it or pass --no-config"
                ) from None
            _log.warning("PyYAML unavailable; ignoring %s and using defaults", path)
        else:
            local = path.with_name("config.local.yaml")
            if local.exists():
                document = _deep_merge(document, _read_yaml(local))
    elif explicit:
        raise ConfigError(f"config file not found: {path}")
    else:
        _log.info("no config at %s; using defaults", path)

    shared = _resolve_shared(_flatten_params(document.get("params", {})))

    camera_section = document.get("camera", {})
    if not isinstance(camera_section, dict):
        raise ConfigError("'camera' must be a mapping")
    camera = _pick(camera_section, _CAMERA_DEFAULTS, "camera.")

    server_section = camera_section.get("server", {})
    if not isinstance(server_section, dict):
        raise ConfigError("'camera.server' must be a mapping")
    server = _pick(server_section, _SERVER_DEFAULTS, "camera.server.")

    port = _env_int("TELEKART_VIDEO_PORT")
    if port is not None:
        server["port"] = port
    bind = os.environ.get("TELEKART_VIDEO_BIND")
    if bind:
        server["bind"] = bind
    synthetic = _env_bool("TELEKART_VIDEO_SYNTHETIC")
    if synthetic is not None:
        camera["synthetic"] = synthetic

    for key, value in (overrides or {}).items():
        if value is None:
            continue
        if key in shared:
            shared[key] = value
        elif key in server:
            server[key] = value
        elif key in camera:
            camera[key] = value
        else:
            raise ConfigError(f"unknown override {key!r}")

    # Re-validate after the overrides. A bad value in the file falls back to the
    # default because the car has to boot; a bad value on the command line is
    # deliberate, and running something other than what was typed is worse than
    # refusing to start.
    for name in _SHARED_PARAMS:
        try:
            shared[name] = coerce(name, shared[name])
        except ParamError as exc:
            raise ConfigError(str(exc)) from exc

    codec_name = shared["video_codec"]
    codec = _CODECS.get(str(codec_name))
    if codec is None:
        raise ConfigError(f"unsupported video_codec {codec_name!r}")

    status_raw = camera["status_path"]
    status_path = Path(str(status_raw)) if status_raw else None

    qp_raw = camera["mjpeg_qp"]

    return VideoConfig(
        codec=codec,
        width=_as_int(shared["video_width"], "video_width"),
        height=_as_int(shared["video_height"], "video_height"),
        fps=_as_int(shared["video_fps"], "video_fps"),
        bitrate=_as_int(shared["video_bitrate"], "video_bitrate"),
        iperiod=_as_int(shared["video_iperiod"], "video_iperiod"),
        exposure_mode=str(camera["exposure_mode"]),
        exposure_us=_as_int(camera["exposure_us"], "camera.exposure_us"),
        analogue_gain=_as_float(camera["analogue_gain"], "camera.analogue_gain"),
        awb_enable=_as_bool(camera["awb_enable"], "camera.awb_enable"),
        hflip=_as_bool(camera["hflip"], "camera.hflip"),
        vflip=_as_bool(camera["vflip"], "camera.vflip"),
        buffer_count=_as_int(camera["buffer_count"], "camera.buffer_count"),
        mjpeg_qp=None if qp_raw is None else _as_int(qp_raw, "camera.mjpeg_qp"),
        synthetic=_as_bool(camera["synthetic"], "camera.synthetic"),
        bind=str(server["bind"]),
        port=_as_int(server["port"], "camera.server.port"),
        max_clients=_as_int(server["max_clients"], "camera.server.max_clients"),
        queue_depth=_as_int(server["queue_depth"], "camera.server.queue_depth"),
        send_buffer_bytes=_as_int(
            server["send_buffer_bytes"], "camera.server.send_buffer_bytes"
        ),
        write_timeout_s=_as_float(server["write_timeout_s"], "camera.server.write_timeout_s"),
        start_on_keyframe=_as_bool(
            server["start_on_keyframe"], "camera.server.start_on_keyframe"
        ),
        frame_timeout_s=_as_float(camera["frame_timeout_s"], "camera.frame_timeout_s"),
        restart_backoff_s=_as_float(camera["restart_backoff_s"], "camera.restart_backoff_s"),
        restart_backoff_max_s=_as_float(
            camera["restart_backoff_max_s"], "camera.restart_backoff_max_s"
        ),
        stats_interval_s=_as_float(camera["stats_interval_s"], "camera.stats_interval_s"),
        status_path=status_path,
        oom_score_adj_min=_as_int(camera["oom_score_adj_min"], "camera.oom_score_adj_min"),
    )
