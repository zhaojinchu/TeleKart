"""Shared fixtures.

The GUI tests here need a ``QApplication`` but not a display: everything is
either painted into a QPixmap or measured from a layout, so the offscreen
platform plugin is enough and the suite stays runnable in CI and over ssh.
"""

from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")


@pytest.fixture(scope="session")
def qapp():
    """One QApplication for the whole session.

    Qt permits exactly one per process and destroying it mid-run leaves
    dangling C++ objects behind, so this is deliberately session-scoped and
    never torn down.
    """
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance()
    if app is None:
        app = QApplication([])
        app.setStyle("Fusion")
    return app
