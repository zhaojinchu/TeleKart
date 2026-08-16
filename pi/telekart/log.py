"""Structured logging, plus the in-memory ring the diagnostics endpoint reads.

Two rules that shape everything here:

* **Fields, not sentences.** ``log.warning("stall", wheel="left", rpm=0.4)``
  survives grep, journalctl filtering, and being pasted into a bug report;
  ``f"stall on left at {rpm} rpm"`` does not.
* **Nothing logs from the 100 Hz loop.** Formatting a record costs tens of
  microseconds and the handler may block on the SD card. The loop records
  numbers into ring buffers and something slower turns them into log lines.
  :class:`RateLimiter` is here for the cases where a loop-adjacent path really
  must say something.
"""

from __future__ import annotations

import json
import logging
import logging.handlers
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import IO, Any, Mapping

from .util.ringbuf import RingBuffer

#: Key under which structured fields ride on a LogRecord. One namespaced key
#: rather than loose attributes, so a field called `msg` or `name` cannot
#: collide with LogRecord's own attributes and blow up the formatter.
FIELDS_KEY = "tk_fields"

DEFAULT_RING_CAPACITY = 512

#: journald maps a leading "<N>" to a syslog priority. Emitting it means
#: `journalctl -p warning` works and the console colours errors, for the cost of
#: three characters per line.
_SYSLOG_PRIORITY = {
    logging.CRITICAL: 2,
    logging.ERROR: 3,
    logging.WARNING: 4,
    logging.INFO: 6,
    logging.DEBUG: 7,
}

_ring: "LogRing | None" = None
_configured = False


# --------------------------------------------------------------------------
# Field rendering
# --------------------------------------------------------------------------


def _render_value(value: Any) -> str:
    if isinstance(value, float):
        # Six digits is more than any physical quantity here deserves, and it
        # keeps repr(0.1 + 0.2) out of the logs.
        return f"{value:.6g}"
    if isinstance(value, (str, bytes)):
        text = value.decode("utf-8", "replace") if isinstance(value, bytes) else value
        if any(ch in text for ch in " =\"\n"):
            return json.dumps(text)
        return text
    return str(value)


def _render_fields(fields: Mapping[str, Any]) -> str:
    return " ".join(f"{key}={_render_value(value)}" for key, value in fields.items())


def _record_fields(record: logging.LogRecord) -> dict[str, Any]:
    fields = getattr(record, FIELDS_KEY, None)
    return dict(fields) if isinstance(fields, dict) else {}


class TextFormatter(logging.Formatter):
    """``12:04:07.913 WARN  telekart.drive  stall  wheel=left rpm=0.4``"""

    def __init__(self, *, syslog_prefix: bool = False, use_color: bool = False) -> None:
        super().__init__()
        self._syslog_prefix = syslog_prefix
        self._use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        stamp = time.strftime("%H:%M:%S", time.localtime(record.created))
        head = f"{stamp}.{int(record.msecs):03d} {record.levelname:<5} {record.name}"
        message = record.getMessage()
        fields = _record_fields(record)
        line = f"{head}  {message}"
        if fields:
            line = f"{line}  {_render_fields(fields)}"
        if record.exc_info:
            line = f"{line}\n{self.formatException(record.exc_info)}"
        if self._syslog_prefix:
            line = f"<{_SYSLOG_PRIORITY.get(record.levelno, 6)}>{line}"
        return line


class JsonFormatter(logging.Formatter):
    """One JSON object per line. For the file handler and for anything that
    ships logs off the car, where a parser beats a regex."""

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "t": round(record.created, 6),
            "level": record.levelname,
            "logger": record.name,
            "msg": record.getMessage(),
        }
        fields = _record_fields(record)
        if fields:
            payload["fields"] = fields
        if record.exc_info:
            payload["exc"] = self.formatException(record.exc_info)
        # default=str so a stray Path or enum in a field never takes the log
        # handler down with a TypeError.
        return json.dumps(payload, separators=(",", ":"), default=str)


# --------------------------------------------------------------------------
# Ring buffer
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class LogEntry:
    """One retained record, already flattened so the diagnostics path never has
    to touch a live LogRecord (which holds references to argument objects)."""

    t: float
    level: int
    level_name: str
    logger: str
    message: str
    fields: dict[str, Any]
    exc: str | None = None

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "t": round(self.t, 6),
            "level": self.level_name,
            "logger": self.logger,
            "msg": self.message,
        }
        if self.fields:
            payload["fields"] = self.fields
        if self.exc:
            payload["exc"] = self.exc
        return payload

    def as_text(self) -> str:
        stamp = time.strftime("%H:%M:%S", time.localtime(self.t))
        line = f"{stamp} {self.level_name:<5} {self.logger}  {self.message}"
        if self.fields:
            line = f"{line}  {_render_fields(self.fields)}"
        return line


class LogRing(logging.Handler):
    """Keeps the last N records in memory so a fault report can carry the run-up
    to the fault. The alternative -- asking the operator for the journal after
    the car has already been power-cycled -- does not work in practice."""

    def __init__(self, capacity: int = DEFAULT_RING_CAPACITY, level: int = logging.DEBUG) -> None:
        super().__init__(level)
        self._entries: RingBuffer[LogEntry] = RingBuffer(capacity)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            exc = self.format_exception(record)
            entry = LogEntry(
                t=record.created,
                level=record.levelno,
                level_name=record.levelname,
                logger=record.name,
                message=record.getMessage(),
                fields=_record_fields(record),
                exc=exc,
            )
            self._entries.append(entry)
        except Exception:  # noqa: BLE001 - a logging handler must never propagate
            self.handleError(record)

    @staticmethod
    def format_exception(record: logging.LogRecord) -> str | None:
        if not record.exc_info:
            return None
        return logging.Formatter().formatException(record.exc_info)

    def entries(self, *, min_level: int = logging.DEBUG, limit: int = 0) -> list[LogEntry]:
        """Oldest first. ``limit`` keeps only the newest N of the matches."""
        selected = [e for e in self._entries if e.level >= min_level]
        if limit > 0 and len(selected) > limit:
            selected = selected[-limit:]
        return selected

    def as_dicts(self, *, min_level: int = logging.DEBUG, limit: int = 0) -> list[dict[str, Any]]:
        return [e.as_dict() for e in self.entries(min_level=min_level, limit=limit)]

    def as_text(self, *, min_level: int = logging.DEBUG, limit: int = 0) -> str:
        return "\n".join(e.as_text() for e in self.entries(min_level=min_level, limit=limit))

    def clear(self) -> None:
        self._entries.clear()

    @property
    def capacity(self) -> int:
        return self._entries.capacity

    @property
    def dropped(self) -> int:
        """Records that aged out of the ring. A large number next to a fault
        report means the run-up was noisy."""
        return self._entries.dropped

    def __len__(self) -> int:
        return len(self._entries)


# --------------------------------------------------------------------------
# Structured logger facade
# --------------------------------------------------------------------------


class StructuredLogger:
    """Thin wrapper over ``logging.Logger`` that takes fields as keywords.

    Deliberately not a LoggerAdapter: the adapter API mangles ``extra`` in ways
    that make bound context and per-call fields fight each other.
    """

    __slots__ = ("_logger", "_context")

    def __init__(self, logger: logging.Logger, context: Mapping[str, Any] | None = None) -> None:
        self._logger = logger
        self._context = dict(context) if context else {}

    # -- construction -------------------------------------------------------

    def bind(self, **fields: Any) -> "StructuredLogger":
        """A child logger that adds ``fields`` to everything it emits. Used to
        stamp a wheel name or a session id once instead of at every call site."""
        merged = dict(self._context)
        merged.update(fields)
        return StructuredLogger(self._logger, merged)

    def child(self, suffix: str) -> "StructuredLogger":
        return StructuredLogger(self._logger.getChild(suffix), self._context)

    # -- emission -----------------------------------------------------------

    def _emit(self, level: int, msg: str, fields: dict[str, Any], exc_info: bool = False) -> None:
        if not self._logger.isEnabledFor(level):
            return
        if self._context:
            merged = dict(self._context)
            merged.update(fields)
            fields = merged
        self._logger.log(level, msg, extra={FIELDS_KEY: fields}, exc_info=exc_info)

    def debug(self, msg: str, **fields: Any) -> None:
        self._emit(logging.DEBUG, msg, fields)

    def info(self, msg: str, **fields: Any) -> None:
        self._emit(logging.INFO, msg, fields)

    def warning(self, msg: str, **fields: Any) -> None:
        self._emit(logging.WARNING, msg, fields)

    def error(self, msg: str, **fields: Any) -> None:
        self._emit(logging.ERROR, msg, fields)

    def critical(self, msg: str, **fields: Any) -> None:
        self._emit(logging.CRITICAL, msg, fields)

    def exception(self, msg: str, **fields: Any) -> None:
        """Log at ERROR with the active traceback attached."""
        self._emit(logging.ERROR, msg, fields, exc_info=True)

    def log(self, level: int, msg: str, **fields: Any) -> None:
        self._emit(level, msg, fields)

    # -- introspection ------------------------------------------------------

    def is_enabled_for(self, level: int) -> bool:
        return self._logger.isEnabledFor(level)

    @property
    def name(self) -> str:
        return self._logger.name

    @property
    def raw(self) -> logging.Logger:
        """The underlying stdlib logger, for the rare caller that needs it."""
        return self._logger

    def __repr__(self) -> str:
        return f"StructuredLogger({self._logger.name!r})"


def get_logger(name: str, **context: Any) -> StructuredLogger:
    """The one way to obtain a logger. ``name`` is conventionally ``__name__``."""
    return StructuredLogger(logging.getLogger(name), context or None)


# --------------------------------------------------------------------------
# Rate limiting
# --------------------------------------------------------------------------


class RateLimiter:
    """Allows one event per interval and counts what it suppressed.

    Fault paths fire at loop rate: a wheel that has stalled has stalled on every
    one of the next thousand ticks. Without this, one mechanical problem
    produces a thousand log lines a second, the SD card becomes the bottleneck,
    and the loop misses its deadline *because of the logging*. The suppression
    count goes into the next line that does get through, so nothing is hidden.
    """

    __slots__ = ("_interval", "_next_at", "_suppressed")

    def __init__(self, interval: float) -> None:
        if interval < 0.0:
            raise ValueError("interval must not be negative")
        self._interval = interval
        self._next_at = 0.0
        self._suppressed = 0

    def allow(self, now: float) -> bool:
        """Callers pass the time from their injected Clock; this module refuses
        to have an opinion about where 'now' comes from."""
        if now >= self._next_at:
            self._next_at = now + self._interval
            return True
        self._suppressed += 1
        return False

    def take_suppressed(self) -> int:
        """Number suppressed since the last call. Attach it to the line you are
        about to emit."""
        count = self._suppressed
        self._suppressed = 0
        return count

    @property
    def suppressed(self) -> int:
        return self._suppressed

    def reset(self) -> None:
        self._next_at = 0.0
        self._suppressed = 0


# --------------------------------------------------------------------------
# Setup
# --------------------------------------------------------------------------


def _running_under_journald() -> bool:
    """systemd sets JOURNAL_STREAM to the device/inode of the unit's stderr."""
    return bool(os.environ.get("JOURNAL_STREAM"))


def _resolve_level(level: int | str) -> int:
    if isinstance(level, int):
        return level
    resolved = logging.getLevelName(level.upper())
    if not isinstance(resolved, int):
        raise ValueError(f"unknown log level {level!r}")
    return resolved


def setup_logging(
    *,
    level: int | str = "INFO",
    stream: IO[str] | None = None,
    json_output: bool = False,
    path: Path | str | None = None,
    ring_capacity: int = DEFAULT_RING_CAPACITY,
    syslog_prefix: bool | None = None,
    force: bool = False,
) -> LogRing:
    """Configure the root logger once and return the diagnostics ring.

    Idempotent: calling it a second time is a no-op unless ``force`` is set, so
    a library-style import cannot quietly reconfigure a running car's logging.
    """
    global _ring, _configured
    if _configured and not force and _ring is not None:
        return _ring

    root = logging.getLogger()
    for handler in list(root.handlers):
        root.removeHandler(handler)
        handler.close()

    resolved = _resolve_level(level)
    root.setLevel(resolved)

    if syslog_prefix is None:
        syslog_prefix = _running_under_journald()

    console = logging.StreamHandler(stream if stream is not None else sys.stderr)
    console.setLevel(resolved)
    console.setFormatter(
        JsonFormatter() if json_output else TextFormatter(syslog_prefix=syslog_prefix)
    )
    root.addHandler(console)

    if path is not None:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        # Bounded and rotated: an unbounded log file on an SD card eventually
        # fills the root filesystem, and a full root filesystem on a Pi is a
        # much worse day than a truncated log.
        file_handler = logging.handlers.RotatingFileHandler(
            target, maxBytes=2 * 1024 * 1024, backupCount=3, encoding="utf-8"
        )
        file_handler.setLevel(resolved)
        file_handler.setFormatter(JsonFormatter())
        root.addHandler(file_handler)

    ring = LogRing(ring_capacity)
    root.addHandler(ring)

    _ring = ring
    _configured = True
    return ring


def log_ring() -> LogRing | None:
    """The ring installed by :func:`setup_logging`, if any."""
    return _ring


def set_level(level: int | str, logger_name: str = "") -> None:
    """Change a level at runtime -- the session channel exposes this so a car
    can be put into debug logging without a restart."""
    resolved = _resolve_level(level)
    target = logging.getLogger(logger_name)
    target.setLevel(resolved)
    if not logger_name:
        # Handlers must not be the thing that filters a level the operator just
        # asked for; the logger stays authoritative.
        for handler in target.handlers:
            if handler.level > resolved:
                handler.setLevel(resolved)


def reset_logging() -> None:
    """Tear the configuration down. Tests use this; the firmware does not."""
    global _ring, _configured
    root = logging.getLogger()
    for handler in list(root.handlers):
        root.removeHandler(handler)
        handler.close()
    _ring = None
    _configured = False
