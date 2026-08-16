"""Where files live and what is configurable.

One module, because there is very little left to configure. This station has no
settings screen: everything below is either a path, a network timing constant
that only changes when the network does, or a display preference with one
sensible default. Editing the JSON by hand is the supported way to change any of
it, which is why the file is written with indentation and read back leniently.

Car *parameters* are not here and are not the app's business -- they live in
``pi/config/config.local.yaml`` on the vehicle.
"""

from __future__ import annotations

import json
import os
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

from platformdirs import PlatformDirs

from . import APP_NAME
from .core.log import get_logger

_log = get_logger(__name__)

#: Redirect every path under one root. Tests set it to a tmp_path so a test run
#: can never touch a real configuration. It is also the honest way to run two
#: configurations side by side.
HOME_ENV_VAR = "TELEKART_HOME"

#: appauthor=False keeps macOS at ``~/Library/Application Support/TeleKart``
#: rather than nesting under a vendor directory nobody types.
_dirs = PlatformDirs(appname=APP_NAME, appauthor=False, roaming=False)


# --------------------------------------------------------------------------
# Paths
# --------------------------------------------------------------------------


def _override_root() -> Path | None:
    raw = os.environ.get(HOME_ENV_VAR)
    if not raw:
        return None
    return Path(raw).expanduser()


def config_dir() -> Path:
    root = _override_root()
    return root / "config" if root else Path(_dirs.user_config_dir)


def log_dir() -> Path:
    """macOS: ``~/Library/Logs/TeleKart`` -- where Console.app expects it."""
    root = _override_root()
    return root / "logs" if root else Path(_dirs.user_log_dir)


def settings_file() -> Path:
    return config_dir() / "settings.json"


def ensure_dirs() -> None:
    for path in (config_dir(), log_dir()):
        path.mkdir(parents=True, exist_ok=True)


# --------------------------------------------------------------------------
# Settings
# --------------------------------------------------------------------------


@dataclass
class LinkSettings:
    """Network identity and timing. Change these when the network changes."""

    #: Blank means ``telekart.local``. Accepts ``host``, ``host:port`` or an
    #: IP literal -- an IP literal skips name resolution entirely, which is the
    #: path a locally-run firmware process takes.
    host: str = ""

    #: 0 asks the OS for the port the car should send telemetry to. Pinning it
    #: is only useful behind a firewall rule that names a port.
    telemetry_port: int = 0

    connect_timeout: float = 4.0
    discovery_timeout: float = 3.0
    ping_interval: float = 1.0
    ping_timeout: float = 5.0

    #: How many duplicate E-stop datagrams to fire immediately, and the gap
    #: between them. One packet is enough on a healthy link; the burst exists
    #: for the unhealthy one, which is the only time an E-stop is pressed.
    estop_burst: int = 10
    estop_burst_gap: float = 0.002

    #: Connect on launch without showing the panel.
    auto_connect: bool = False


@dataclass
class VideoSettings:
    enabled: bool = True
    #: Physical pixels the decoder reformats to. Set by the video widget at
    #: runtime and persisted only so the first frame after a relaunch is
    #: already the right size.
    target_width: int = 0
    target_height: int = 0


@dataclass
class Settings:
    driver: str = "driver"
    #: The id the *car* reported at the last successful handshake. Recorded for
    #: the log and for the window title; never typed, never used to authenticate.
    car_id: str = ""
    #: "kmh", "mph" or "ms". One line, no UI.
    speed_unit: str = "kmh"
    log_level: str = "INFO"
    #: Force a wheel's axis roles when auto-detection picks wrong:
    #: ``{"steer": 0, "throttle": 2, "brake": 3}``. Empty means auto-detect.
    axes: dict[str, int] = field(default_factory=dict)

    link: LinkSettings = field(default_factory=LinkSettings)
    video: VideoSettings = field(default_factory=VideoSettings)

    _path: Path | None = field(default=None, repr=False, compare=False)

    # -- persistence --------------------------------------------------------

    @classmethod
    def load(cls, path: Path | None = None) -> "Settings":
        """Read, or return defaults. A broken file is never fatal.

        A settings file that fails to parse must not stop the app launching --
        the driver's next move would be to delete it anyway, and an app that
        will not start is a worse way to say "your JSON has a trailing comma".
        """
        target = path or settings_file()
        settings = cls()
        settings._path = target
        try:
            raw = json.loads(target.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return settings
        except (OSError, ValueError) as exc:
            _log.warning("settings unreadable (%s); using defaults", exc)
            return settings
        if not isinstance(raw, dict):
            return settings
        _apply(settings, raw)
        return settings

    def save(self, path: Path | None = None) -> None:
        target = path or self._path or settings_file()
        payload = {k: v for k, v in asdict(self).items() if not k.startswith("_")}
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            tmp = target.with_name(target.name + ".tmp")
            tmp.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
            # Atomic replace: a crash mid-write must not leave a truncated file
            # that the next launch reports as corrupt.
            os.replace(tmp, target)
        except OSError as exc:
            _log.warning("could not save settings: %s", exc)


def _apply(target: Any, raw: dict[str, Any]) -> None:
    """Copy known keys, recursing into nested dataclasses. Unknown keys are
    ignored so a file written by a newer build still loads."""
    for f in fields(target):
        if f.name.startswith("_") or f.name not in raw:
            continue
        value = raw[f.name]
        current = getattr(target, f.name)
        if hasattr(current, "__dataclass_fields__") and isinstance(value, dict):
            _apply(current, value)
        elif isinstance(current, dict) and isinstance(value, dict):
            setattr(target, f.name, dict(value))
        elif isinstance(value, type(current)) or (
            isinstance(current, float) and isinstance(value, int)
        ):
            setattr(target, f.name, type(current)(value))


__all__ = [
    "HOME_ENV_VAR",
    "LinkSettings",
    "Settings",
    "VideoSettings",
    "config_dir",
    "ensure_dirs",
    "log_dir",
    "settings_file",
]
