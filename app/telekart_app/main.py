"""Application bootstrap: build everything once, in a defensible order, and own
the shutdown.

The order below is not arbitrary and getting it wrong is quiet rather than loud:

1. **Paths and logging**, before anything that might want to complain. A failure
   during startup that is only visible in a terminal nobody kept is a failure
   that gets reported as "it didn't open".
2. **Theme**, which sets the Fusion style, the fonts and the palette, before the
   first widget exists. Qt polishes widgets as they are constructed, so a
   stylesheet applied afterwards costs a second full polish of the entire tree.
3. **Model**, which owns the threads and the sockets. Constructed before the
   window so the window renders the real current state on its first paint
   instead of an empty one that fills in a frame later.
4. **Window**, then ``model.start()``. Starting the threads first would deliver
   change signals to a model whose subscribers do not exist yet -- harmless, but
   it means the first telemetry packet of the session is the one that is thrown
   away.

Shutdown runs in reverse and is wired to ``aboutToQuit`` rather than to the
window's close event, because the app can also be terminated by the dock, by
⌘Q, or by the session manager, and every one of those has to stop the TX thread.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication

from . import APP_NAME, ORG_DOMAIN, ORG_NAME, __version__
from .config import paths
from .config.settings import Settings, load_shared_key
from .core import log
from .input.profile import ProfileStore
from .model.app_model import AppModel
from .ui.main_window import MainWindow
from .ui.theme.qss import apply_theme
from .ui.theme.tokens import THEME

_log = log.get_logger(__name__)

#: Defaults baked into ``telekart-sim``'s own argument parser. Repeated here
#: rather than imported: the app must not depend on the simulator package being
#: installed, and these two strings are part of its documented CLI.
SIM_CAR_ID = "telekart-sim"
SIM_KEY = "telekart"
SIM_HOST = "127.0.0.1"


@dataclass(frozen=True, slots=True)
class LaunchOptions:
    """Everything the command line can say about this run."""

    host: str = ""
    driver: str = ""
    screen: str = ""
    log_level: str = ""
    fullscreen: bool = False
    connect: bool = False
    simulator: bool = False


# --------------------------------------------------------------------------
# Simulator
# --------------------------------------------------------------------------


class SimulatorController:
    """Starts and stops a local ``telekart-sim`` process.

    Worth having in the app rather than leaving to a terminal: the single most
    common first-run outcome is "I have no car yet", and a button that produces
    a protocol-identical one turns that from a dead end into a working session.

    Nothing here waits for the simulator to be ready. The session client already
    retries with backoff, so pointing it at a port that is not listening yet
    costs one failed connect and then succeeds -- which is both simpler and more
    honest than a blocking readiness probe that would freeze the UI if the
    simulator failed to start at all.
    """

    def __init__(
        self,
        *,
        repo_root: Path | None = None,
        host: str = SIM_HOST,
        key: str = SIM_KEY,
        car_id: str = SIM_CAR_ID,
    ) -> None:
        self._repo_root = repo_root or _repo_root()
        self._host = host
        self._key = key
        self._car_id = car_id
        self._process: subprocess.Popen[bytes] | None = None

    @property
    def host(self) -> str:
        return self._host

    @property
    def key(self) -> str:
        return self._key

    @property
    def car_id(self) -> str:
        return self._car_id

    def running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def command(self) -> list[str] | None:
        """How to start the simulator on this machine, or ``None`` if we cannot.

        Ordered by how specific each answer is: an explicit override, then the
        dedicated environment ``make setup-sim`` builds, then whatever is on
        PATH, then the package inside this interpreter.
        """
        override = os.environ.get("TELEKART_SIM")
        if override:
            return [override]

        venv = self._repo_root / "tools" / "telekart_sim" / ".venv" / "bin" / "telekart-sim"
        if venv.is_file() and os.access(venv, os.X_OK):
            return [str(venv)]

        found = shutil.which("telekart-sim")
        if found:
            return [found]

        try:
            import importlib.util

            if importlib.util.find_spec("telekart_sim") is not None:
                return [sys.executable, "-m", "telekart_sim"]
        except (ImportError, ValueError):
            pass
        return None

    def launch(self) -> tuple[bool, str]:
        if self.running():
            return True, "The simulator is already running."

        command = self.command()
        if command is None:
            return False, (
                "No simulator found. Run 'make setup-sim' in the repository, "
                "or set TELEKART_SIM to the telekart-sim executable."
            )
        argv = command + ["--car-id", self._car_id, "--key", self._key]
        try:
            self._process = subprocess.Popen(argv)
        except OSError as exc:
            _log.error("could not start the simulator: %s", exc)
            return False, "Could not start the simulator: %s" % (exc,)
        _log.info("started the simulator: %s", " ".join(argv))
        return True, "Simulator starting; connecting to %s…" % (self._host,)

    def stop(self) -> None:
        process = self._process
        self._process = None
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            # A simulator that ignores SIGTERM would otherwise keep the car's
            # ports bound and make the next launch fail for no visible reason.
            _log.warning("the simulator ignored SIGTERM; killing it")
            process.kill()


# --------------------------------------------------------------------------
# Application
# --------------------------------------------------------------------------


class TeleKartApplication:
    """Owns the object graph for one run of the desktop app."""

    def __init__(self, app: QApplication, options: LaunchOptions) -> None:
        self.app = app
        self.options = options

        paths.ensure_dirs()
        self.settings = Settings.load()
        level = options.log_level or self.settings.log_level
        log_path = log.configure(level=level)
        log.install_thread_excepthook()
        log.install_qt_message_handler()
        _log.info("%s %s starting (log: %s)", APP_NAME, __version__, log_path)

        if options.driver:
            self.settings.driver = options.driver
        if options.host:
            self.settings.link.host = options.host

        self.applied_theme = apply_theme(app, THEME)
        _log.info("theme applied: %s", self.applied_theme.font_roles.describe())

        self.profiles = ProfileStore()
        self.profiles.load()

        self.simulator = SimulatorController()
        # Given Settings, the model builds the real object graph (link manager,
        # recorder, race director, input thread). Given boxes instead, it builds
        # nothing -- which is how the UI tests drive it with no sockets at all.
        self.model = AppModel(settings=self.settings, profiles=self.profiles)
        self.window = MainWindow(
            self.model,
            self.settings,
            self.profiles,
            theme=THEME,
            simulator=self.simulator,
        )

        app.aboutToQuit.connect(self.shutdown)

    def run(self) -> int:
        self.model.start()
        self.window.show()

        if self.options.screen:
            self.window.show_screen(self.options.screen)
        if self.options.fullscreen:
            self.window.showFullScreen()
        if self.options.simulator:
            self.window.launch_simulator()
        elif self.options.connect or self.settings.link.auto_connect:
            self._auto_connect()

        return self.app.exec()

    def _auto_connect(self) -> None:
        """Reconnect to the last car without asking for the passphrase again.

        The key is looked up by car id, which is what the credentials file is
        keyed by. An empty result is not an error: the session client will fail
        the handshake, the connect screen will say why, and the driver types it
        once more.
        """
        host = self.settings.link.host
        car_id = self.settings.car_id
        key = load_shared_key(car_id) if car_id else ""
        self.model.connect_to(host, shared_key=key)

    def shutdown(self) -> None:
        _log.info("shutting down")
        self.model.stop()
        self.simulator.stop()
        self.settings.save()


def run(app: QApplication, options: LaunchOptions) -> int:
    """Build and run. Separated from ``__main__`` so tests can drive it."""
    QCoreApplication.setApplicationName(APP_NAME)
    QCoreApplication.setOrganizationName(ORG_NAME)
    QCoreApplication.setOrganizationDomain(ORG_DOMAIN)
    QCoreApplication.setApplicationVersion(__version__)
    return TeleKartApplication(app, options).run()


def _repo_root() -> Path:
    """The checkout this module was imported from.

    ``app/telekart_app/main.py`` -> ``app/telekart_app`` -> ``app`` -> repo. Used
    only to find the simulator's virtualenv, and every caller of it tolerates
    the answer being wrong -- an installed wheel has no repository above it, and
    then PATH is the right place to look anyway.
    """
    return Path(__file__).resolve().parents[2]


__all__ = [
    "SIM_CAR_ID",
    "SIM_HOST",
    "SIM_KEY",
    "LaunchOptions",
    "SimulatorController",
    "TeleKartApplication",
    "run",
]
