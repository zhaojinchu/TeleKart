"""Application bootstrap: build everything once, in a defensible order.

The order below is not arbitrary, and getting it wrong is quiet rather than loud:

1. **Paths and logging**, before anything that might want to complain. A failure
   during startup that is only visible in a terminal nobody kept is a failure
   that gets reported as "it didn't open".
2. **Theme**, which sets the Fusion style and the stylesheet, before the first
   widget exists. Qt polishes widgets as they are constructed, so a stylesheet
   applied afterwards costs a second full polish of the entire tree.
3. **Model**, which owns the threads and the sockets. Constructed before the
   window so the window renders the real current state on its first paint
   instead of an empty one that fills in a frame later.
4. **Window**, then ``model.start()``. Starting the threads first would deliver
   change signals to a model whose subscribers do not exist yet -- harmless, but
   it means the first telemetry packet of the session is the one thrown away.

Shutdown runs in reverse and is wired to ``aboutToQuit`` rather than to the
window's close event, because the app can also be terminated by the dock, by ⌘Q,
or by the session manager, and every one of those has to stop the TX thread.
"""

from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtCore import QCoreApplication
from PySide6.QtWidgets import QApplication

from telekart_protocol import VideoCodec

from . import APP_NAME, ORG_DOMAIN, ORG_NAME, __version__
from . import config
from .config import Settings
from .core import log
from .model.app_model import AppModel
from .ui.theme import apply_theme
from .ui.window import MainWindow

_log = log.get_logger(__name__)


@dataclass(frozen=True, slots=True)
class LaunchOptions:
    """Everything the command line can say about this run."""

    host: str = ""
    driver: str = ""
    log_level: str = ""
    codec: str = "h264"
    fullscreen: bool = False
    connect: bool = False


class TeleKartApplication:
    """Owns the object graph for one run."""

    def __init__(self, app: QApplication, options: LaunchOptions) -> None:
        self.app = app
        self.options = options

        config.ensure_dirs()
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

        apply_theme(app)

        codec = VideoCodec.MJPEG if options.codec == "mjpeg" else VideoCodec.H264
        # Given Settings, the model builds the real object graph. Given boxes
        # instead, it builds nothing -- which is how the tests drive it with no
        # sockets at all.
        self.model = AppModel(settings=self.settings, codec=codec)
        self.window = MainWindow(self.model, self.settings)

        app.aboutToQuit.connect(self.shutdown)

    def run(self) -> int:
        self.model.start()
        self.window.show()

        if self.options.fullscreen:
            self.window.showFullScreen()

        if self.options.connect or self.settings.link.auto_connect:
            self.model.connect_to(self.settings.link.host, driver=self.settings.driver)
        else:
            self.window.show_panel()

        return self.app.exec()

    def shutdown(self) -> None:
        _log.info("shutting down")
        self.model.stop()
        self.settings.save()


def run(app: QApplication, options: LaunchOptions) -> int:
    """Build and run. Separated from ``__main__`` so tests can drive it."""
    QCoreApplication.setApplicationName(APP_NAME)
    QCoreApplication.setOrganizationName(ORG_NAME)
    QCoreApplication.setOrganizationDomain(ORG_DOMAIN)
    QCoreApplication.setApplicationVersion(__version__)
    return TeleKartApplication(app, options).run()


__all__ = ["LaunchOptions", "TeleKartApplication", "run"]
