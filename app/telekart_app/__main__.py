"""Entry point: check the environment, then hand over to the bootstrap.

Everything before ``QApplication`` exists is here, and it is all about failing
in a way somebody can act on. A driving station that dies with
``ModuleNotFoundError: pygame`` on a laptop at a track has told its user
nothing; the same failure with "run ``make setup-app``" is a fifteen-second fix.

Two checks in particular are worth the lines they take:

**Python version.** The floor is hard because the code genuinely needs 3.12.
The ceiling is a *warning*, not a refusal: it exists in ``pyproject.toml``
because the binary wheels were not all published for newer interpreters yet,
which is a packaging fact with an expiry date. If the wheels imported anyway,
the application works, and refusing to start would be this file inventing a
problem the machine has already solved.

**pygame-ce versus pygame.** They claim the same import name, so installing
both leaves a directory that satisfies neither -- and the symptom is an import
error or a missing attribute somewhere deep in the input stack, hours after the
mistake. Detecting it here costs one metadata scan and names the exact command
that fixes it.

``Fusion`` is set on the very first line after the application object exists.
The macOS native style ignores large parts of QSS -- scrollbars, item views,
spin boxes -- and the widgets polish themselves as they are constructed, so the
style has to be in place before the first one is built rather than after.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import Sequence

from . import APP_NAME, __version__

#: Hard floor. Below this the source does not run: the codebase uses 3.12
#: syntax and typing throughout.
MIN_PYTHON = (3, 12)

#: Soft ceiling, mirroring ``pyproject.toml``. Warned about, never enforced.
TESTED_BELOW_PYTHON = (3, 14)


@dataclass(frozen=True, slots=True)
class Check:
    """One startup assertion and the exact action that resolves it."""

    ok: bool
    fatal: bool
    message: str


def check_python() -> Check:
    version = sys.version_info
    if version < MIN_PYTHON:
        return Check(
            False,
            True,
            "%s needs Python %d.%d or newer; this is %d.%d.%d at %s.\n"
            "  Fix: run 'make setup-app' in the repository, which builds "
            "app/.venv with a supported interpreter, then launch with "
            "app/.venv/bin/python -m telekart_app."
            % (
                APP_NAME,
                MIN_PYTHON[0],
                MIN_PYTHON[1],
                version.major,
                version.minor,
                version.micro,
                sys.executable,
            ),
        )
    if version[:2] >= TESTED_BELOW_PYTHON:
        return Check(
            True,
            False,
            "Running on Python %d.%d, which is newer than this build was "
            "tested against. Everything imported, so it should be fine; if "
            "video or input misbehaves, try app/.venv from 'make setup-app'."
            % (version.major, version.minor),
        )
    return Check(True, False, "")


def check_qt() -> Check:
    try:
        import PySide6  # noqa: F401  (probe only)
        from PySide6.QtWidgets import QApplication  # noqa: F401
    except ImportError as exc:
        return Check(
            False,
            True,
            "PySide6 is not importable (%s).\n"
            "  Fix: run 'make setup-app', or 'pip install PySide6-Essentials' "
            "into the environment you are launching from (%s)."
            % (exc, sys.executable),
        )
    return Check(True, False, "")


def check_pygame() -> Check:
    """pygame-ce, never upstream pygame, and never both.

    The two projects install into the same ``pygame`` package directory. With
    both present, whichever was installed last wins for some files and loses for
    others, and the result is an import that succeeds and then fails on an
    attribute nobody expects to be missing.
    """
    installed = _installed_distributions()
    has_ce = "pygame-ce" in installed
    has_upstream = "pygame" in installed

    if has_ce and has_upstream:
        return Check(
            False,
            True,
            "Both pygame and pygame-ce are installed. They share the 'pygame' "
            "import name and the combination is broken.\n"
            "  Fix: pip uninstall -y pygame pygame-ce && pip install pygame-ce",
        )

    if has_upstream and not has_ce:
        return Check(
            False,
            True,
            "Upstream pygame is installed; this application needs pygame-ce.\n"
            "  Fix: pip uninstall -y pygame && pip install pygame-ce\n"
            "  Why: pygame-ce ships wheels for every interpreter this app "
            "supports, including macOS arm64, which upstream has repeatedly "
            "lagged on.",
        )

    if not has_ce:
        return Check(
            True,
            False,
            "pygame-ce is not installed, so no wheel, pad or joystick will be "
            "found. Telemetry, video and tuning still work; driving does not.\n"
            "  Fix: pip install pygame-ce",
        )
    return Check(True, False, "")


def preflight() -> list[Check]:
    return [check_python(), check_qt(), check_pygame()]


def report(checks: Sequence[Check], stream: object = None) -> bool:
    """Print every message; return True when nothing fatal was found.

    Every check is reported rather than stopping at the first failure: someone
    setting up a fresh machine would otherwise fix the interpreter, rerun, and
    discover the input library problem only then.
    """
    out = stream if stream is not None else sys.stderr
    fatal = False
    for check in checks:
        if not check.message:
            continue
        prefix = "ERROR: " if not check.ok else "note:  "
        print(prefix + check.message, file=out)  # type: ignore[arg-type]
        fatal = fatal or (check.fatal and not check.ok)
    return not fatal


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="telekart_app",
        description="%s desktop driving station." % (APP_NAME,),
    )
    parser.add_argument(
        "--host",
        default="",
        help="car hostname or address; blank discovers telekart.local",
    )
    parser.add_argument("--driver", default="", help="name recorded in the session")
    parser.add_argument(
        "--screen",
        default="",
        choices=("", "connect", "drive", "garage", "diagnostics", "settings"),
        help="screen to open on startup",
    )
    parser.add_argument(
        "--log-level",
        default="",
        choices=("", "DEBUG", "INFO", "WARNING", "ERROR"),
        help="overrides the saved setting for this run",
    )
    parser.add_argument(
        "--fullscreen", action="store_true", help="start the window fullscreen"
    )
    parser.add_argument(
        "--connect",
        action="store_true",
        help="connect immediately using the remembered passphrase",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="start telekart-sim locally and connect to it",
    )
    parser.add_argument("--version", action="version", version=__version__)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parsed = build_parser().parse_args(list(argv) if argv is not None else None)

    if not report(preflight()):
        return 2

    from PySide6.QtWidgets import QApplication

    # Only argv[0]: argparse owns this command line, and handing Qt flags it
    # does not recognise makes it print its own usage and exit.
    app = QApplication(sys.argv[:1])
    # Before any widget exists. See the module docstring.
    app.setStyle("Fusion")

    from .main import LaunchOptions, run

    return run(
        app,
        LaunchOptions(
            host=parsed.host,
            driver=parsed.driver,
            screen=parsed.screen,
            log_level=parsed.log_level,
            fullscreen=parsed.fullscreen,
            connect=parsed.connect,
            simulator=parsed.sim,
        ),
    )


def _installed_distributions() -> set[str]:
    """Normalised names of everything installed in this environment."""
    from importlib import metadata

    names: set[str] = set()
    for distribution in metadata.distributions():
        name = distribution.metadata["Name"]
        if name:
            names.add(name.strip().lower().replace("_", "-"))
    return names


if __name__ == "__main__":
    sys.exit(main())
