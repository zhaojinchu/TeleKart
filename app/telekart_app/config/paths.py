"""Where the app keeps its files.

Every path in the application resolves through here. Scattering
``Path.home() / ".telekart"`` around the codebase is how you end up with a
settings file the uninstaller misses and a database nobody can find.
"""

from __future__ import annotations

import os
from pathlib import Path

from platformdirs import PlatformDirs

from .. import APP_NAME

#: Redirect every path under one root. Tests set it to a tmp_path so a test run
#: can never touch -- or worse, corrupt -- a real driving session's database.
#: It is also the honest way to run two configurations side by side.
HOME_ENV_VAR = "TELEKART_HOME"

#: appauthor=False keeps macOS at ``~/Library/Application Support/TeleKart``
#: rather than nesting under a vendor directory nobody types.
_dirs = PlatformDirs(appname=APP_NAME, appauthor=False, roaming=False)


def _override_root() -> Path | None:
    raw = os.environ.get(HOME_ENV_VAR)
    if not raw:
        return None
    return Path(raw).expanduser()


def config_dir() -> Path:
    """Settings and credentials. macOS: ``~/Library/Application Support/TeleKart``."""
    root = _override_root()
    return root / "config" if root else Path(_dirs.user_config_dir)


def data_dir() -> Path:
    """Recordings and the session database."""
    root = _override_root()
    return root / "data" if root else Path(_dirs.user_data_dir)


def cache_dir() -> Path:
    """Regenerable only. Deleting this must never lose a driving session."""
    root = _override_root()
    return root / "cache" if root else Path(_dirs.user_cache_dir)


def log_dir() -> Path:
    """macOS: ``~/Library/Logs/TeleKart`` -- where Console.app expects to find it."""
    root = _override_root()
    return root / "logs" if root else Path(_dirs.user_log_dir)


def settings_file() -> Path:
    return config_dir() / "settings.json"


def credentials_file() -> Path:
    """Shared keys, kept out of ``settings.json`` deliberately.

    settings.json is the file people paste into a bug report or copy to a
    second laptop. Splitting the secrets out means doing that leaks nothing,
    and it lets this file carry 0600 permissions without arguing about whether
    a window geometry needs them.
    """
    return config_dir() / "credentials.json"


def database_file() -> Path:
    return data_dir() / "sessions.sqlite3"


def recordings_dir() -> Path:
    """Video captures and exports. The telemetry itself lives in the database."""
    return data_dir() / "recordings"


def ensure_dirs() -> None:
    """Create every directory the app writes to. Call once at startup.

    Deliberately eager and at startup: a permissions problem must surface
    before the driver is mid-session and the recorder starts silently dropping
    rows.
    """
    for path in (config_dir(), data_dir(), cache_dir(), log_dir(), recordings_dir()):
        path.mkdir(parents=True, exist_ok=True)


def describe() -> dict[str, str]:
    """For the About dialog and for ``--paths`` on the CLI."""
    return {
        "config": str(config_dir()),
        "data": str(data_dir()),
        "cache": str(cache_dir()),
        "logs": str(log_dir()),
        "database": str(database_file()),
        "recordings": str(recordings_dir()),
    }
