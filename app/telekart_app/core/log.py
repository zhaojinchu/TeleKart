"""Logging setup for the desktop app.

Stdlib only, on purpose: this is imported by every other module, including ones
that must stay importable in a bare interpreter with no Qt and no platformdirs.
"""

from __future__ import annotations

import logging
import logging.handlers
import sys
import threading
import time
from pathlib import Path
from typing import Any

#: Every logger in the app hangs off this root so one call configures the lot
#: and a stray third-party logger cannot inherit our handlers by accident.
ROOT_LOGGER_NAME = "telekart"

_FILE_FORMAT = "%(asctime)s %(levelname)-7s %(threadName)-14s %(name)-28s %(message)s"
_CONSOLE_FORMAT = "%(levelname)-7s %(name)-24s %(message)s"

_LOG_BYTES = 4 * 1024 * 1024
_LOG_BACKUPS = 5

_configured = False
_configure_lock = threading.Lock()


def get_logger(name: str) -> logging.Logger:
    """Logger for a submodule. Pass ``__name__``; the prefix is normalised."""
    if name.startswith(ROOT_LOGGER_NAME + "."):
        return logging.getLogger(name)
    if name in ("__main__", ROOT_LOGGER_NAME):
        return logging.getLogger(ROOT_LOGGER_NAME)
    short = name.removeprefix("telekart_app.").removeprefix("telekart_app")
    return logging.getLogger(f"{ROOT_LOGGER_NAME}.{short}" if short else ROOT_LOGGER_NAME)


def configure(
    *,
    level: int | str = logging.INFO,
    log_dir: Path | None = None,
    console: bool = True,
    filename: str = "telekart.log",
    force: bool = False,
) -> Path | None:
    """Install handlers. Idempotent unless ``force``. Returns the log file path.

    File logging is the important half. A teleoperation bug shows up as "the car
    twitched once, three minutes ago" -- unreproducible, and gone from the
    terminal scrollback by the time anyone notices. The rotating file is the
    only record that survives.
    """
    global _configured
    with _configure_lock:
        root = logging.getLogger(ROOT_LOGGER_NAME)
        if _configured and not force:
            root.setLevel(_coerce_level(level))
            return _existing_path(root)

        for handler in list(root.handlers):
            root.removeHandler(handler)
            handler.close()

        root.setLevel(_coerce_level(level))
        # Our records must not also reach the interpreter's root logger, which
        # a library may have pointed at stderr with its own format.
        root.propagate = False

        if console:
            stream = logging.StreamHandler(sys.stderr)
            stream.setFormatter(logging.Formatter(_CONSOLE_FORMAT))
            root.addHandler(stream)

        path: Path | None = None
        if log_dir is None:
            # Imported here rather than at module scope so `core.log` keeps its
            # stdlib-only property: a caller that passes an explicit directory
            # never touches platformdirs.
            from ..config import paths

            log_dir = paths.log_dir()
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            path = log_dir / filename
            rotating = logging.handlers.RotatingFileHandler(
                path, maxBytes=_LOG_BYTES, backupCount=_LOG_BACKUPS, encoding="utf-8"
            )
            rotating.setFormatter(logging.Formatter(_FILE_FORMAT))
            root.addHandler(rotating)
        except OSError as exc:
            path = None
            root.warning("file logging disabled: %s", exc)

        _configured = True
        return path


def _existing_path(root: logging.Logger) -> Path | None:
    for handler in root.handlers:
        if isinstance(handler, logging.handlers.RotatingFileHandler):
            return Path(handler.baseFilename)
    return None


def _coerce_level(level: int | str) -> int:
    if isinstance(level, int):
        return level
    resolved = logging.getLevelName(level.upper())
    if not isinstance(resolved, int):
        raise ValueError(f"unknown log level {level!r}")
    return resolved


def set_level(level: int | str) -> None:
    logging.getLogger(ROOT_LOGGER_NAME).setLevel(_coerce_level(level))


class Throttle:
    """Rate limiter for log sites that can fire at packet rate.

    A decode failure on a corrupt stream repeats 30 times a second. Unthrottled
    that is a megabyte a minute of identical lines, and it hides whatever
    happened next -- so the failure that matters gets buried by the failure that
    is merely loud.

    Not a logging.Filter: a Filter still pays for record construction and
    argument formatting before it can drop anything.
    """

    __slots__ = ("_interval", "_lock", "_next", "_suppressed")

    def __init__(self, interval: float = 5.0) -> None:
        if interval <= 0.0:
            raise ValueError("throttle interval must be positive")
        self._interval = interval
        self._lock = threading.Lock()
        self._next: dict[str, float] = {}
        self._suppressed: dict[str, int] = {}

    def allow(self, key: str) -> int:
        """Return the suppressed count (>=0) if this call may log, else -1.

        The suppressed count belongs in the message: "decode failed (x412)" is
        a diagnosis, "decode failed" repeated once every five seconds is not.
        """
        now = time.monotonic()
        with self._lock:
            due = self._next.get(key, 0.0)
            if now < due:
                self._suppressed[key] = self._suppressed.get(key, 0) + 1
                return -1
            self._next[key] = now + self._interval
            return self._suppressed.pop(key, 0)

    def log(
        self,
        logger: logging.Logger,
        level: int,
        key: str,
        msg: str,
        *args: Any,
    ) -> None:
        suppressed = self.allow(key)
        if suppressed < 0:
            return
        if suppressed:
            logger.log(level, msg + " (+%d suppressed)", *args, suppressed)
        else:
            logger.log(level, msg, *args)

    def reset(self, key: str | None = None) -> None:
        with self._lock:
            if key is None:
                self._next.clear()
                self._suppressed.clear()
            else:
                self._next.pop(key, None)
                self._suppressed.pop(key, None)


def install_thread_excepthook() -> None:
    """Route worker-thread crashes into the log.

    Without this a ``threading.Thread`` that dies prints to stderr and vanishes:
    the TX thread stops sending, the car failsafes, and the app shows a link
    that simply went quiet with no explanation anywhere.
    """
    logger = get_logger("thread")

    def hook(args: threading.ExceptHookArgs) -> None:
        if args.exc_type is SystemExit:
            return
        name = args.thread.name if args.thread is not None else "<unknown>"
        logger.critical(
            "unhandled exception in thread %s",
            name,
            exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
        )

    threading.excepthook = hook


def install_qt_message_handler() -> None:
    """Send Qt's own diagnostics through this logger.

    Qt writes to stderr by default, so its warnings never reach the log file --
    and "QPainter::begin: Paint device returned engine == 0" repeated a thousand
    times is precisely the sort of thing you want in the file, next to the
    telemetry that provoked it.

    Imported lazily: `core.log` must stay usable with no Qt installed.
    """
    from PySide6.QtCore import QtMsgType, qInstallMessageHandler

    logger = get_logger("qt")
    levels = {
        QtMsgType.QtDebugMsg: logging.DEBUG,
        QtMsgType.QtInfoMsg: logging.INFO,
        QtMsgType.QtWarningMsg: logging.WARNING,
        QtMsgType.QtCriticalMsg: logging.ERROR,
        QtMsgType.QtFatalMsg: logging.CRITICAL,
    }

    def handler(mode: QtMsgType, context: object, message: str) -> None:
        logger.log(levels.get(mode, logging.INFO), "%s", message)

    qInstallMessageHandler(handler)
