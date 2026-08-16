"""About, and the environment report that makes a bug report useful.

Half of this dialog is a credit and half is a diagnostic. The diagnostic half
earns its place: almost every "it looks wrong on my machine" report comes down
to which font Qt actually resolved, which Qt build is loaded, or where the
config directory ended up -- and asking someone to run three commands in a
terminal to find out loses most of them. One button that copies the lot as text
turns that exchange into a paste.
"""

from __future__ import annotations

import platform
import sys

from PySide6 import __version__ as pyside_version
from PySide6.QtCore import QSize, Qt, __version__ as qt_version
from PySide6.QtWidgets import (
    QApplication,
    QDialog,
    QDialogButtonBox,
    QGridLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from telekart_protocol import PROTO_VERSION

from ... import APP_NAME, __version__
from ...config import paths
from ..theme import fonts
from ..theme.tokens import THEME, Theme


class AboutDialog(QDialog):
    """Version, environment, and a one-click copy of both."""

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent)
        self._theme = theme
        self.setWindowTitle("About " + APP_NAME)
        self.setMinimumSize(QSize(520, 440))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(
            theme.space.xl, theme.space.xl, theme.space.xl, theme.space.lg
        )
        layout.setSpacing(theme.space.md)

        title = QLabel(APP_NAME, self)
        title.setProperty("variant", "title")
        layout.addWidget(title)

        strap = QLabel(
            "Teleoperated RC car: low-latency video, wheel input, telemetry "
            "HUD and tuning.",
            self,
        )
        strap.setProperty("variant", "caption")
        strap.setWordWrap(True)
        layout.addWidget(strap)

        grid = QGridLayout()
        grid.setHorizontalSpacing(theme.space.lg)
        grid.setVerticalSpacing(theme.space.xs)
        grid.setColumnStretch(1, 1)
        for row, (key, value) in enumerate(environment_report().items()):
            caption = QLabel(key, self)
            caption.setProperty("variant", "eyebrow")
            grid.addWidget(caption, row, 0, Qt.AlignmentFlag.AlignTop)

            body = QLabel(value, self)
            body.setProperty("variant", "mono")
            body.setWordWrap(True)
            body.setTextInteractionFlags(
                Qt.TextInteractionFlag.TextSelectableByMouse
            )
            grid.addWidget(body, row, 1)
        layout.addLayout(grid)
        layout.addStretch(1)

        note = QLabel(
            "Built against the shared wire protocol in packages/telekart_protocol. "
            "The car refuses a protocol version it does not recognise rather "
            "than negotiating, because a misparsed control packet is a runaway "
            "car.",
            self,
        )
        note.setProperty("variant", "caption")
        note.setWordWrap(True)
        layout.addWidget(note)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close, self)
        copy = QPushButton("Copy diagnostics", self)
        copy.clicked.connect(self._copy)
        buttons.addButton(copy, QDialogButtonBox.ButtonRole.ActionRole)
        buttons.rejected.connect(self.reject)
        buttons.accepted.connect(self.accept)
        layout.addWidget(buttons)

    def _copy(self) -> None:
        clipboard = QApplication.clipboard()
        if clipboard is not None:
            clipboard.setText(diagnostics_text())


def environment_report() -> dict[str, str]:
    """Everything worth knowing about this installation, as label -> value."""
    roles = fonts.roles()
    return {
        "Version": "%s (protocol %d)" % (__version__, PROTO_VERSION),
        "Python": "%s (%s)" % (platform.python_version(), sys.executable),
        "Qt": "PySide6 %s / Qt %s" % (pyside_version, qt_version),
        "Platform": "%s %s (%s)"
        % (platform.system(), platform.release(), platform.machine()),
        "Fonts": roles.describe(),
        "Config": str(paths.config_dir()),
        "Data": str(paths.data_dir()),
        "Logs": str(paths.log_dir()),
    }


def diagnostics_text() -> str:
    """The report as plain text, for pasting into an issue."""
    report = environment_report()
    width = max(len(key) for key in report)
    return "\n".join("%-*s  %s" % (width, key, value) for key, value in report.items())


__all__ = ["AboutDialog", "diagnostics_text", "environment_report"]
