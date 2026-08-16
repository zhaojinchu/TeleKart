"""Persisted application settings.

Plain JSON rather than ``QSettings``: the file is meant to be readable, diffable
and hand-editable during bring-up, and nothing here is Qt's business. It also
keeps ``config`` importable with no GUI toolkit present.
"""

from __future__ import annotations

import json
import os
import stat
import threading
import time
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Any

from telekart_protocol import UDP_TELEMETRY_PORT

from ..core.log import get_logger
from . import paths

_log = get_logger(__name__)

#: Bumped whenever a field changes meaning. Adding a field does NOT need a bump:
#: unknown keys are dropped and missing keys take their default, so old and new
#: files both load. Only a *reinterpretation* needs a migration.
SETTINGS_VERSION = 1

#: A 1:10 scale car at 3 m/s is doing a scale 108 km/h. The real number reads as
#: "slow"; the scale number is the one that matches what the driver sees. Both
#: stay available -- the diagnostic overlay shows true m/s -- but the speedometer
#: defaults to scale, because that is the honest translation of the experience.
DEFAULT_SCALE_FACTOR = 10.0


@dataclass(slots=True)
class VideoSettings:
    enabled: bool = True
    #: Decode-side reformat target. The decode thread scales to this so the GUI
    #: thread does a 1:1 blit; setting it to the widget size is the whole point.
    #: 0 means "native", i.e. no scaling.
    target_width: int = 0
    target_height: int = 0
    #: Stop decoding rather than show smeared macroblocks after a loss. Off by
    #: default: for teleoperation a corrupted-but-live picture beats a frozen
    #: one, because a frozen picture looks like a road that is still there.
    freeze_on_corruption: bool = False
    show_stats_overlay: bool = False


@dataclass(slots=True)
class RecordingSettings:
    #: On from day one. A session you did not record is a session you cannot
    #: explain afterwards, and the disk cost is about 8 MB an hour.
    enabled: bool = True
    #: Telemetry arrives at 50 Hz; 25 Hz is plenty for after-the-fact analysis
    #: and halves the row count. Set equal to the telemetry rate for tuning work.
    telemetry_hz: float = 25.0
    inputs: bool = True
    keep_days: int = 90


@dataclass(slots=True)
class LinkSettings:
    #: Empty means "discover". A pinned host skips mDNS entirely, which is what
    #: you want on a congested LAN or when Bonjour is being unhelpful.
    host: str = ""
    #: 0 asks the OS for an ephemeral port, which is what lets two apps run on
    #: one machine -- the port is sent to the car in the handshake.
    telemetry_port: int = UDP_TELEMETRY_PORT
    auto_connect: bool = False
    discovery_timeout: float = 3.0
    connect_timeout: float = 4.0
    #: Bounds on the reconnect backoff. The floor is not zero because a car that
    #: is rebooting will refuse connections for several seconds and hammering it
    #: only delays the moment it is ready.
    reconnect_min_delay: float = 0.5
    reconnect_max_delay: float = 8.0
    #: TCP is silent about a peer that vanished; only traffic finds a half-open
    #: connection. At 1 Hz a dead link is detected inside the ping timeout.
    ping_interval: float = 1.0
    ping_timeout: float = 5.0
    #: Number of datagrams in the E-stop burst. UDP loses packets exactly when
    #: you least want it to, so the one message that must arrive is never
    #: entrusted to a single datagram.
    estop_burst: int = 10
    estop_burst_gap: float = 0.002


@dataclass(slots=True)
class DisplaySettings:
    #: "scale_kmh" | "scale_mph" | "real_kmh" | "real_mps"
    speed_mode: str = "scale_kmh"
    scale_factor: float = DEFAULT_SCALE_FACTOR
    #: Always shows true SI alongside the scaled reading.
    diagnostic_overlay: bool = False
    theme: str = "dark"


@dataclass(slots=True)
class RaceSettings:
    mode: str = "free_drive"
    target_laps: int = 5
    #: Debounce for the lap trigger. A double-tapped button must not register
    #: two laps, and no real lap on this car is under five seconds.
    min_lap_time: float = 5.0
    lap_source: str = "manual"


@dataclass(slots=True)
class Settings:
    """The whole persisted state of the app.

    ``input`` and ``ui`` are deliberately untyped blobs. They belong to the
    input-chain and UI workstreams, which own their own schemas; forcing their
    fields through this dataclass would make every one of their changes a change
    here too.
    """

    version: int = SETTINGS_VERSION
    driver: str = ""
    car_id: str = ""
    log_level: str = "INFO"

    link: LinkSettings = field(default_factory=LinkSettings)
    video: VideoSettings = field(default_factory=VideoSettings)
    recording: RecordingSettings = field(default_factory=RecordingSettings)
    display: DisplaySettings = field(default_factory=DisplaySettings)
    race: RaceSettings = field(default_factory=RaceSettings)

    input: dict[str, Any] = field(default_factory=dict)
    ui: dict[str, Any] = field(default_factory=dict)

    # -- persistence --------------------------------------------------------

    @classmethod
    def load(cls, path: Path | None = None) -> "Settings":
        """Read from disk. A broken file is quarantined, never fatal.

        Refusing to start because a preferences file has a stray comma would be
        the wrong trade: the app's job is to drive a car, and every setting here
        has a working default.
        """
        target = path or paths.settings_file()
        try:
            raw = target.read_text(encoding="utf-8")
        except FileNotFoundError:
            return cls()
        except OSError as exc:
            _log.warning("cannot read %s (%s); using defaults", target, exc)
            return cls()

        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                raise ValueError("settings file must contain a JSON object")
        except (ValueError, TypeError) as exc:
            backup = target.with_suffix(f".corrupt-{int(time.time())}.json")
            try:
                target.replace(backup)
                _log.error("settings file was unreadable (%s); moved to %s", exc, backup)
            except OSError:
                _log.error("settings file was unreadable (%s) and could not be moved", exc)
            return cls()

        return cls.from_dict(data)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Settings":
        data = _migrate(dict(data))
        settings = cls()
        for f in fields(cls):
            if f.name not in data:
                continue
            value = data[f.name]
            current = getattr(settings, f.name)
            if _is_section(current):
                _apply_section(current, value, f.name)
            elif isinstance(current, dict):
                if isinstance(value, dict):
                    setattr(settings, f.name, dict(value))
                else:
                    _log.warning("settings: ignoring %s, expected an object", f.name)
            else:
                ok, coerced = _coerce_scalar(current, value)
                if ok:
                    setattr(settings, f.name, coerced)
                else:
                    _log.warning(
                        "settings: ignoring %s=%r (expected %s)",
                        f.name,
                        value,
                        type(current).__name__,
                    )
        settings.version = SETTINGS_VERSION
        return settings

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def save(self, path: Path | None = None) -> None:
        """Write atomically.

        A half-written settings file is indistinguishable from a corrupt one,
        and the app is most likely to be killed while quitting -- which is
        exactly when this is being written.
        """
        target = path or paths.settings_file()
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(self.to_dict(), indent=2, sort_keys=False)
        tmp = target.with_name(target.name + ".tmp")
        try:
            tmp.write_text(payload + "\n", encoding="utf-8")
            os.replace(tmp, target)
        except OSError as exc:
            _log.error("could not save settings to %s: %s", target, exc)
            tmp.unlink(missing_ok=True)


def _is_section(value: object) -> bool:
    return isinstance(
        value, (LinkSettings, VideoSettings, RecordingSettings, DisplaySettings, RaceSettings)
    )


def _apply_section(section: Any, value: Any, name: str) -> None:
    if not isinstance(value, dict):
        _log.warning("settings: ignoring %s, expected an object", name)
        return
    for f in fields(section):
        if f.name not in value:
            continue
        ok, coerced = _coerce_scalar(getattr(section, f.name), value[f.name])
        if ok:
            setattr(section, f.name, coerced)
        else:
            _log.warning("settings: ignoring %s.%s=%r", name, f.name, value[f.name])


def _coerce_scalar(current: Any, incoming: Any) -> tuple[bool, Any]:
    """Type-check one setting against its default.

    ``bool`` is checked before ``int`` throughout: in Python ``True`` passes an
    ``isinstance(int)`` test, so a naive check turns ``"target_laps": true``
    into one lap instead of rejecting it.
    """
    if isinstance(current, bool):
        return (True, incoming) if isinstance(incoming, bool) else (False, current)
    if isinstance(incoming, bool):
        return (False, current)
    if isinstance(current, float):
        if isinstance(incoming, (int, float)):
            return (True, float(incoming))
        return (False, current)
    if isinstance(current, int):
        return (True, incoming) if isinstance(incoming, int) else (False, current)
    if isinstance(current, str):
        return (True, incoming) if isinstance(incoming, str) else (False, current)
    return (False, current)


def _migrate(data: dict[str, Any]) -> dict[str, Any]:
    """Bring an older file forward. Each step is one version.

    Version 1 is the first shipped schema, so there is nothing to migrate yet;
    the dispatch exists so the first real migration is an edit and not a
    redesign. A file from the future is loaded on a best-effort basis -- unknown
    keys are dropped by `from_dict` anyway.
    """
    version = data.get("version")
    if not isinstance(version, int):
        version = SETTINGS_VERSION
    if version > SETTINGS_VERSION:
        _log.warning(
            "settings file is version %d, this build understands %d; "
            "unknown fields will be dropped on the next save",
            version,
            SETTINGS_VERSION,
        )
    return data


# --------------------------------------------------------------------------
# Shared keys
# --------------------------------------------------------------------------

_cred_lock = threading.Lock()


def load_shared_key(car_id: str, path: Path | None = None) -> str:
    """Remembered passphrase for a car, or "" if none.

    Stored in a separate 0600 file rather than in the keychain: the threat model
    in ``telekart_protocol.crypto`` is a second laptop on the LAN, not somebody
    with your unlocked user account. Keeping it out of settings.json is what
    stops it appearing in a screenshot or a pasted bug report.
    """
    target = path or paths.credentials_file()
    with _cred_lock:
        store = _read_credentials(target)
    value = store.get(car_id, "")
    return value if isinstance(value, str) else ""


def save_shared_key(car_id: str, key: str, path: Path | None = None) -> None:
    if not car_id:
        raise ValueError("car_id is required to store a shared key")
    target = path or paths.credentials_file()
    with _cred_lock:
        store = _read_credentials(target)
        store[car_id] = key
        _write_credentials(target, store)


def forget_shared_key(car_id: str, path: Path | None = None) -> None:
    target = path or paths.credentials_file()
    with _cred_lock:
        store = _read_credentials(target)
        if store.pop(car_id, None) is not None:
            _write_credentials(target, store)


def known_cars(path: Path | None = None) -> list[str]:
    target = path or paths.credentials_file()
    with _cred_lock:
        return sorted(_read_credentials(target))


def _read_credentials(target: Path) -> dict[str, Any]:
    try:
        data = json.loads(target.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}
    except (OSError, ValueError) as exc:
        _log.warning("credentials file unreadable (%s); treating as empty", exc)
        return {}
    return data if isinstance(data, dict) else {}


def _write_credentials(target: Path, store: dict[str, Any]) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_name(target.name + ".tmp")
    try:
        # Create with 0600 *before* writing. Writing first and chmod-ing after
        # leaves a window where the secret is world-readable.
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, stat.S_IRUSR | stat.S_IWUSR)
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(store, handle, indent=2)
            handle.write("\n")
        os.replace(tmp, target)
    except OSError as exc:
        _log.error("could not write credentials to %s: %s", target, exc)
        tmp.unlink(missing_ok=True)
