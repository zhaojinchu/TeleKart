"""Shared fixtures.

Every test runs against a throwaway ``TELEKART_HOME`` so a test run can never
read -- or worse, write -- a real configuration or a real stored passphrase.
"""

from __future__ import annotations

import os
import sys

import pytest

# Offscreen before Qt is imported anywhere, or the first import wins and a CI
# box with no display fails with a platform-plugin error instead of running.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.fixture(autouse=True)
def isolated_home(tmp_path, monkeypatch):
    monkeypatch.setenv("TELEKART_HOME", str(tmp_path / "telekart"))
    yield tmp_path


@pytest.fixture
def settings():
    from telekart_ui.config import Settings

    return Settings()


@pytest.fixture
def boxes():
    """The five ``LatestBox`` objects ``AppModel`` drains, and nothing else.

    This is the fixture that makes the UI testable: an ``AppModel`` built from
    these has no sockets, no SDL, no PyAV and no car, but presents the exact
    surface every widget uses.
    """
    from telekart_ui.core.latest_box import LatestBox

    class Sources:
        def __init__(self) -> None:
            self.telemetry_box: LatestBox = LatestBox()
            self.link_box: LatestBox = LatestBox()
            self.input_box: LatestBox = LatestBox()
            self.session_box: LatestBox = LatestBox()
            self.video_box: LatestBox = LatestBox()

    return Sources()


@pytest.fixture
def model(qapp, boxes):
    from telekart_ui.model.app_model import AppModel

    return AppModel(boxes)


@pytest.fixture
def runtime(qapp, settings):
    """A real runtime with the input thread and SDL not started.

    Not a mock: the keyboard tests drive the genuine ``KeyboardSource`` and the
    genuine ``InputChain``, and step the input thread by hand. A fake here would
    prove only that the fake works.
    """
    from telekart_ui.model.runtime import AppRuntime

    rt = AppRuntime(settings, enable_input=False)
    yield rt
    rt.link.close()


def pytest_report_header(config):
    return f"python {sys.version.split()[0]}  qt platform {os.environ.get('QT_QPA_PLATFORM')}"
