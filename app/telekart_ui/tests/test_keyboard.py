"""Keyboard driving, and the four ways it has broken before.

Three of these are the content of commit 835ab3d ("Wire keyboard driving, which
was never connected at all"). The fourth is a gap that commit left behind: the
methods for it existed and nothing called them.

The tests drive the real ``KeyboardSource`` and the real ``InputChain`` through
the real window event handlers. The only thing faked is the passage of time.
"""

from __future__ import annotations

import pytest
from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QKeyEvent
from PySide6.QtTest import QTest

from telekart_protocol import ControlFlags

from telekart_ui.config import Settings
from telekart_ui.input.mapping import KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE, KEY_UP
from telekart_ui.model.app_model import AppModel
from telekart_ui.ui.window import MainWindow

_DT = 0.004  # one 250 Hz input tick
_SETTLE_TICKS = 400  # long enough for the digital-steering rate limit to arrive


def _press(key: int, *, autorepeat: bool = False) -> QKeyEvent:
    return QKeyEvent(QEvent.Type.KeyPress, key, Qt.KeyboardModifier.NoModifier, "", autorepeat)


def _release(key: int, *, autorepeat: bool = False) -> QKeyEvent:
    return QKeyEvent(
        QEvent.Type.KeyRelease, key, Qt.KeyboardModifier.NoModifier, "", autorepeat
    )


@pytest.fixture
def window(qapp, runtime):
    settings = Settings()
    model = AppModel(runtime=runtime)
    win = MainWindow(model, settings)
    win.resize(1024, 640)
    # The window opens the panel by itself while the link is OFFLINE, which is
    # correct and is asserted separately. These tests are about the driving
    # state, so close it -- through the real path, not by hiding the widget.
    win._hide_panel()
    yield win
    win.close()


def _settle(runtime, ticks: int = _SETTLE_TICKS):
    """Step the input thread by hand. No threads, no wall clock."""
    for _ in range(ticks):
        runtime.input._tick(0.0, _DT)
    return runtime.input.output


def test_wasd_aliases_onto_the_arrow_keys(window, runtime):
    """WASD must work even though only the arrows are bound.

    ControlBinding holds one ref per direction, so both cannot be bound in the
    profile; the alias happens at the event boundary instead. A driver who
    tries WASD, gets nothing, and concludes the app is broken is not wrong to.
    """
    for key, field, expected in (
        (Qt.Key.Key_W, "throttle", 1.0),
        (Qt.Key.Key_S, "brake", 1.0),
        (Qt.Key.Key_A, "steer", -1.0),
        (Qt.Key.Key_D, "steer", 1.0),
    ):
        runtime.keyboard_release_all()
        runtime.input.reset()
        window.keyPressEvent(_press(key))
        out = _settle(runtime)
        assert getattr(out, field) == pytest.approx(expected, abs=0.01), key


def test_arrow_keys_still_work(window, runtime):
    window.keyPressEvent(_press(Qt.Key.Key_Up))
    assert _settle(runtime).throttle == pytest.approx(1.0, abs=0.01)


def test_a_real_keystroke_reaches_the_chain(qapp, window, runtime):
    """Delivered to the focused widget, not by calling the handler by hand.

    Every other test here invokes ``window.keyPressEvent`` directly, which
    proves the handler is correct and proves nothing about whether a keystroke
    ever gets to it. The focused widget is the VideoView, so this asserts the
    event actually propagates up to the window -- the exact gap that let
    keyboard driving ship unwired once already.
    """
    window.show()
    window.video.setFocus(Qt.FocusReason.OtherFocusReason)
    qapp.processEvents()
    assert qapp.focusWidget() is window.video

    QTest.keyPress(window.video, Qt.Key.Key_W)
    qapp.processEvents()
    assert _settle(runtime).throttle == pytest.approx(1.0, abs=0.01)

    QTest.keyRelease(window.video, Qt.Key.Key_W)
    qapp.processEvents()
    assert _settle(runtime).throttle == pytest.approx(0.0, abs=0.01)


def test_release_returns_to_neutral(window, runtime):
    window.keyPressEvent(_press(Qt.Key.Key_W))
    assert _settle(runtime).throttle == pytest.approx(1.0, abs=0.01)
    window.keyReleaseEvent(_release(Qt.Key.Key_W))
    assert _settle(runtime).throttle == pytest.approx(0.0, abs=0.01)


def test_autorepeat_is_filtered_on_both_edges(window, runtime):
    """A held key must look held, not tapped.

    Qt synthesises a full release/press pair for every repeat of a held key.
    Forwarding those makes a held throttle look like frantic tapping to the rate
    limiter, which resolves to *no throttle at all* -- the exact symptom that
    made keyboard driving look broken rather than absent.
    """
    window.keyPressEvent(_press(Qt.Key.Key_W))
    _settle(runtime)

    for _ in range(20):
        window.keyReleaseEvent(_release(Qt.Key.Key_W, autorepeat=True))
        window.keyPressEvent(_press(Qt.Key.Key_W, autorepeat=True))
        runtime.input._tick(0.0, _DT)

    assert runtime.input.output.throttle == pytest.approx(1.0, abs=0.01)


def test_unbound_keys_are_not_forwarded(window, runtime):
    """Anything the profile does not bind has no business in the input chain."""
    before = runtime.input.output
    window.keyPressEvent(_press(Qt.Key.Key_Z))
    window.keyPressEvent(_press(Qt.Key.Key_7))
    after = _settle(runtime, 10)
    assert after.throttle == before.throttle
    assert after.steer == before.steer


def test_the_connect_panel_blocks_driving(window, runtime):
    """Typing a passphrase must not drive the car.

    The panel has text fields. Without the gate, a passphrase containing 'w'
    opens the throttle and the space bar E-stops the car mid-word.
    """
    window.show_panel()
    window.keyPressEvent(_press(Qt.Key.Key_W))
    window.keyPressEvent(_press(Qt.Key.Key_Space))
    out = _settle(runtime, 50)
    assert out.throttle == pytest.approx(0.0)
    command = runtime.link.command_box.peek()
    if command is not None:
        assert not (command.flags & ControlFlags.ESTOP)


def test_the_panel_opens_itself_when_there_is_no_car(qapp, runtime):
    """Offline is not a state the driver should have to discover a menu for."""
    win = MainWindow(AppModel(runtime=runtime), Settings())
    try:
        assert win._panel_open is True
    finally:
        win.close()


def test_a_dead_link_marks_the_picture_as_history(window):
    """Found by killing the firmware mid-session.

    The last frame stays on screen when the car goes away -- which is right, the
    driver wants to see where it was -- but it must be dimmed and badged, or it
    looks exactly like a road that is still there. The previous station gated
    this on the link still being usable, so the badge never appeared in the one
    case that matters.
    """
    from telekart_ui.model.snapshots import LinkSnapshot, LinkState

    window._on_link(LinkSnapshot(state=LinkState.LIVE, video_ok=True, video_age=0.0))
    assert window.video._stale is False

    window._on_link(LinkSnapshot(state=LinkState.FAILED, detail="car closed the session"))
    assert window.video._stale is True

    window._on_link(LinkSnapshot(state=LinkState.LIVE, video_ok=True, video_age=5.0))
    assert window.video._stale is True


def test_space_reaches_the_estop(window, runtime):
    """Space is the E-stop and nothing may swallow it.

    In the previous station the navigation rail took StrongFocus and consumed
    Space to activate an item, so whether the E-stop worked depended on where
    the driver had last clicked. There is no rail here, and this asserts the
    key actually arrives.
    """
    window.keyPressEvent(_press(Qt.Key.Key_Space))
    runtime.input._tick(0.0, _DT)
    command = runtime.link.command_box.peek()
    assert command is not None
    assert command.flags & ControlFlags.ESTOP


def test_deactivating_the_window_releases_every_key(window, runtime):
    """Alt-tab with the throttle held must not leave the car driving.

    The window never sees the key release, so the input thread keeps publishing
    full throttle -- and because the app is still transmitting happily, no
    failsafe fires. Both methods this relies on existed in the previous station
    and had no callers at all.
    """
    window.keyPressEvent(_press(Qt.Key.Key_W))
    assert _settle(runtime).throttle == pytest.approx(1.0, abs=0.01)

    window.changeEvent(QEvent(QEvent.Type.WindowDeactivate))
    assert _settle(runtime).throttle == pytest.approx(0.0, abs=0.01)


def test_escape_leaves_fullscreen_and_never_disarms(window, runtime):
    """An accidental disarm at speed leaves a car travelling with no steering."""
    commands: list[str] = []
    window._model.disarm = lambda: commands.append("disarm")  # type: ignore[method-assign]
    window.showFullScreen()
    window.keyPressEvent(_press(Qt.Key.Key_Escape))
    assert commands == []
    assert not window.isFullScreen()


def test_bound_keys_covers_every_control(runtime):
    """The window's filter must not exclude a control the profile binds."""
    bound = runtime.bound_keys()
    for key in (KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_SPACE):
        assert key in bound
