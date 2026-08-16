"""Entry point: check the environment, build a QApplication, hand off.

The preflight is not defensive programming for its own sake. Each check below
corresponds to a failure that is otherwise reported as a stack trace from deep
inside a library, or -- worse -- as an app that launches and then behaves subtly
wrong.
"""

from __future__ import annotations

import argparse
import sys

from . import APP_NAME, __version__
from .app import LaunchOptions, run

#: PySide6-Essentials 6.11 and pygame-ce wheels. Both bounds are facts about
#: wheel availability, not guesses about the future.
_MIN_PYTHON = (3, 12)
_MAX_PYTHON = (3, 14)


def check_python() -> str:
    if sys.version_info < _MIN_PYTHON:
        raise SystemExit(
            f"{APP_NAME} needs Python {_MIN_PYTHON[0]}.{_MIN_PYTHON[1]} or newer; "
            f"this is {sys.version.split()[0]}."
        )
    if sys.version_info >= _MAX_PYTHON:
        # A warning, not an error: if the wheels have landed by then, the app
        # will work fine and refusing to start would be wrong.
        return (
            f"warning: running on Python {sys.version.split()[0]}, which is past the "
            f"tested ceiling of {_MAX_PYTHON[0]}.{_MAX_PYTHON[1] - 1}."
        )
    return ""


def check_pygame() -> str:
    """pygame-ce, never pygame. They claim the same module name.

    Installing both leaves whichever landed last on sys.path, and the symptom is
    a wheel that reports zero axes rather than an import error -- so this is
    checked at startup where it can be said plainly.
    """
    try:
        import pygame
    except ImportError:
        return "no pygame-ce: the wheel will not be readable (keyboard driving still works)."
    if not getattr(pygame, "IS_CE", False):
        raise SystemExit(
            "Upstream 'pygame' is installed, but this app requires 'pygame-ce'.\n"
            "They claim the same module name; having both breaks imports.\n"
            "  pip uninstall -y pygame pygame-ce && pip install pygame-ce"
        )
    return ""


def check_qt() -> str:
    try:
        import PySide6  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            f"PySide6 is not importable ({exc}). Run 'make setup-ui'."
        ) from exc
    return ""


def preflight() -> list[str]:
    """Fatal problems raise; survivable ones come back as warnings."""
    return [note for note in (check_python(), check_qt(), check_pygame()) if note]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="telekart-ui", description=f"{APP_NAME} driving station."
    )
    parser.add_argument(
        "--host", default="", help="car hostname or address; blank uses telekart.local"
    )
    parser.add_argument("--driver", default="", help="name recorded in the session")
    parser.add_argument(
        "--codec",
        default="h264",
        choices=("h264", "mjpeg"),
        help="video codec the car is configured for (default: h264)",
    )
    parser.add_argument(
        "--log-level", default="", choices=("DEBUG", "INFO", "WARNING", "ERROR")
    )
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument(
        "--connect",
        action="store_true",
        help="connect immediately using the remembered passphrase",
    )
    parser.add_argument("--version", action="version", version=f"{APP_NAME} {__version__}")
    return parser


def main(argv: list[str] | None = None) -> int:
    for note in preflight():
        print(note, file=sys.stderr)

    args = build_parser().parse_args(argv)

    from PySide6.QtWidgets import QApplication

    # Only argv[0]: argparse owns the command line, and Qt would otherwise try
    # to interpret our flags as its own.
    app = QApplication(sys.argv[:1])
    return run(
        app,
        LaunchOptions(
            host=args.host,
            driver=args.driver,
            log_level=args.log_level,
            codec=args.codec,
            fullscreen=args.fullscreen,
            connect=args.connect,
        ),
    )


if __name__ == "__main__":
    raise SystemExit(main())
