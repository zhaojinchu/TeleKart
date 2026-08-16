"""The window. There is only one, and it shows one thing.

Video fills it edge to edge with the HUD floating on top. The connect panel is
an overlay that covers the picture when there is no car. No navigation rail, no
screen stack, no status bar, no modal dialogs -- a modal over a live video feed
is a modal over a moving car.

**Escape leaves fullscreen and does nothing else.** In particular it never
disarms. The reflex to hit Escape when something looks wrong is universal, and
if that reflex cut the motors at speed the car would be a projectile with no
steering. The correct panic control is the E-stop, on Space and on ⌘.

**Keyboard driving is wired here, and this is where it goes wrong.** Four
behaviours below are regressions this codebase has already shipped once; each
one is commented with what it costs to get wrong, and each has a test.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable, ClassVar

from PySide6.QtCore import QEvent, Qt, Slot
from PySide6.QtGui import QAction, QCloseEvent, QKeyEvent, QKeySequence, QResizeEvent
from PySide6.QtWidgets import QMainWindow, QWidget

from .. import APP_NAME
from ..config import Settings
from ..core.log import get_logger
from ..model.snapshots import LinkSnapshot, LinkState, SessionSnapshot, VehicleSnapshot
from ..model.units import SpeedMode, UnitFormatter
from .connect_panel import ConnectPanel
from .video_view import VideoView

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..model.app_model import AppModel

_log = get_logger(__name__)

#: Video age past which the last frame stops being evidence about the world.
#: Three frame intervals at 30 fps plus a keyframe interval of slack: below this
#: a dropped frame is normal, above it the picture is history.
_VIDEO_STALE_S = 0.60


class MainWindow(QMainWindow):
    """Video, HUD, and a connect overlay."""

    #: WASD aliased onto the arrow keys the keyboard profile actually binds.
    #: Arrows are canonical, but three decades of driving games have trained
    #: WASD into people's hands, and a driver who tries it, gets nothing, and
    #: concludes the app is broken is not wrong to. Aliased at the event
    #: boundary rather than bound in the profile because ControlBinding holds
    #: one ref per direction, so both cannot be bound at once.
    _KEY_ALIASES: ClassVar[dict[Qt.Key, Qt.Key]] = {
        Qt.Key.Key_W: Qt.Key.Key_Up,
        Qt.Key.Key_S: Qt.Key.Key_Down,
        Qt.Key.Key_A: Qt.Key.Key_Left,
        Qt.Key.Key_D: Qt.Key.Key_Right,
    }

    def __init__(
        self,
        model: "AppModel",
        settings: Settings,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._model = model
        self._settings = settings
        self._session_active = False
        self._armed = False
        # Tracked explicitly rather than read back from ``panel.isVisible()``.
        # Qt reports a child as not visible whenever any ancestor is hidden, so
        # widget visibility answers "is it on screen right now", while the
        # question here is "has the driver got the panel open" -- and the key
        # filter must not depend on whether the window happens to be showing.
        self._panel_open = False

        self.setWindowTitle(APP_NAME)
        self.setMinimumSize(720, 460)

        self.video = VideoView(self)
        self.hud = self.video.hud
        self.setCentralWidget(self.video)

        self.panel = ConnectPanel(self)
        self.panel.connectRequested.connect(self._on_connect_requested)
        self.panel.disconnectRequested.connect(model.disconnect)
        self.panel.dismissed.connect(self._hide_panel)

        self._build_menus()
        self._apply_display_settings()

        self.video.set_frame_provider(model.take_frame)
        self.video.displaySizeChanged.connect(model.set_display_size)

        model.ticked.connect(self._on_tick)
        model.vehicleChanged.connect(self._on_vehicle)
        model.linkChanged.connect(self._on_link)
        model.inputChanged.connect(self.hud.set_input)
        model.sessionChanged.connect(self._on_session)

        # Paint the current state immediately rather than waiting for the first
        # change signal, so a window shown mid-session is never briefly empty.
        self._on_vehicle(model.vehicle)
        self._on_link(model.link)
        self.hud.set_input(model.input)
        self._on_session(model.session)

    # -- construction -------------------------------------------------------

    def _build_menus(self) -> None:
        bar = self.menuBar()
        # Explicit, though it is the default on macOS: the alternative is a menu
        # bar drawn inside the window, and on a fullscreen driving view that is
        # a strip of chrome across the top of the picture.
        bar.setNativeMenuBar(True)

        file_menu = bar.addMenu("&File")
        self._add(file_menu, "Connect…", "Ctrl+K", self.show_panel)
        self._add(file_menu, "Disconnect", None, self._model.disconnect)
        file_menu.addSeparator()
        quit_action = self._add(
            file_menu, "Quit " + APP_NAME, QKeySequence.StandardKey.Quit, self.close
        )
        quit_action.setMenuRole(QAction.MenuRole.QuitRole)

        car = bar.addMenu("&Car")
        self._act_arm = self._add(car, "Arm", "Ctrl+Return", self._model.arm)
        self._act_disarm = self._add(car, "Disarm", "Ctrl+Shift+Return", self._model.disarm)
        car.addSeparator()
        # ⌘. is the Mac convention for "stop what you are doing", and it is
        # unreachable by accident with one hand on a wheel.
        self._add(car, "Emergency stop", "Ctrl+.", self._model.estop)
        self._add(car, "Clear E-stop", "Ctrl+Shift+E", self._model.clear_estop)
        self._add(car, "Clear faults", None, self._model.clear_faults)
        self._add(car, "Reset odometry", None, self._model.reset_odom)

        view = bar.addMenu("&View")
        self._act_hud = self._add_checkable(view, "Head-up display", "Ctrl+H", self._toggle_hud)
        self._act_hud.setChecked(True)
        self._act_detail = self._add_checkable(
            view, "Detail row", "Ctrl+D", self._toggle_detail
        )
        view.addSeparator()
        self._act_full = self._add(
            view, "Full screen", QKeySequence.StandardKey.FullScreen, self.toggle_fullscreen
        )
        self._act_full.setCheckable(True)
        # F11 as well as the platform chord: it is what every driver expects,
        # and on macOS the standard key is ⌃⌘F, which nobody guesses.
        self._act_full.setShortcuts(
            [QKeySequence(QKeySequence.StandardKey.FullScreen), QKeySequence("F11")]
        )

    def _add(
        self,
        menu: Any,
        text: str,
        shortcut: QKeySequence.StandardKey | str | None,
        slot: Callable[[], None],
    ) -> QAction:
        """A plain action whose slot takes no arguments.

        ``QAction.triggered`` carries a ``checked`` flag, so the slot is wrapped
        rather than connected directly -- binding a zero-argument method straight
        to it relies on the binding layer discarding the extra argument, which
        works right up until a model method grows an optional first parameter
        and starts receiving ``False`` for it.
        """
        return self._make_action(menu, text, shortcut, lambda _checked=False: slot())

    def _add_checkable(
        self,
        menu: Any,
        text: str,
        shortcut: QKeySequence.StandardKey | str | None,
        slot: Callable[[bool], None],
    ) -> QAction:
        action = self._make_action(menu, text, shortcut, lambda checked=False: slot(checked))
        action.setCheckable(True)
        return action

    def _make_action(
        self,
        menu: Any,
        text: str,
        shortcut: QKeySequence.StandardKey | str | None,
        handler: Callable[..., None],
    ) -> QAction:
        action = QAction(text, self)
        if shortcut is not None:
            action.setShortcut(QKeySequence(shortcut))
        action.triggered.connect(handler)
        # Window-scoped so the chord still works when the video view has taken
        # focus for its own key handling.
        action.setShortcutContext(Qt.ShortcutContext.WindowShortcut)
        menu.addAction(action)
        self.addAction(action)
        return action

    # -- the connect panel --------------------------------------------------

    @Slot()
    def show_panel(self) -> None:
        self._panel_open = True
        self.panel.setGeometry(self.centralWidget().geometry())
        self.panel.prefill(self._settings.link.host)
        self.panel.raise_()
        self.panel.show()

    def _hide_panel(self) -> None:
        self._panel_open = False
        self.panel.hide()
        # Focus back to the video, or the next keystroke goes nowhere and
        # keyboard driving silently stops working.
        self.video.setFocus(Qt.FocusReason.OtherFocusReason)

    def _on_connect_requested(self, host: str) -> None:
        self._settings.link.host = host
        self._model.connect_to(host, driver=self._settings.driver)

    # -- window behaviours --------------------------------------------------

    def toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
        self._act_full.setChecked(self.isFullScreen())

    def _toggle_hud(self, checked: bool) -> None:
        self.hud.set_instruments_visible(checked)

    def _toggle_detail(self, checked: bool) -> None:
        self.hud.set_detail_visible(checked)

    def _apply_display_settings(self) -> None:
        self.hud.set_formatter(
            UnitFormatter(speed_mode=SpeedMode.parse(self._settings.speed_unit))
        )

    def resizeEvent(self, event: QResizeEvent) -> None:
        if self._panel_open:
            self.panel.setGeometry(self.centralWidget().geometry())
        super().resizeEvent(event)

    # -- keyboard -----------------------------------------------------------

    def _driving_key(self, event: QKeyEvent) -> int:
        return int(self._KEY_ALIASES.get(Qt.Key(event.key()), Qt.Key(event.key())))

    def _forwards_keys(self) -> bool:
        """Only drive when the driving surface is what the driver is looking at.

        The connect panel has text fields in it. Without this gate, typing a
        text field containing a 'w' would open the throttle -- and the space
        bar while typing a hostname would E-stop the car.
        """
        return not self._panel_open

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key.Key_Escape:
            # Leaves fullscreen and stops there. It is NOT wired to disarm and
            # must never be: an accidental disarm at speed leaves a car with no
            # drive and no steering authority, travelling.
            if self.isFullScreen():
                self.showNormal()
                self._act_full.setChecked(False)
            event.accept()
            return
        # Auto-repeat is filtered on BOTH edges. Qt synthesises a full
        # release/press pair for every repeat of a held key, and forwarding
        # those makes a held throttle look like frantic tapping to the rate
        # limiter -- which resolves to no throttle at all.
        if not event.isAutoRepeat() and self._forwards_keys():
            code = self._driving_key(event)
            # Only keys the profile actually binds. Anything else is a menu
            # chord or a stray keystroke and has no business in the input chain.
            if code in self._model.bound_keys():
                self._model.key_pressed(code)
                event.accept()
                return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event: QKeyEvent) -> None:
        if not event.isAutoRepeat() and self._forwards_keys():
            code = self._driving_key(event)
            if code in self._model.bound_keys():
                self._model.key_released(code)
                event.accept()
                return
        super().keyReleaseEvent(event)

    def changeEvent(self, event: QEvent) -> None:
        """Release every held key when the window stops being active.

        Alt-tab away with the throttle held and the window never sees the key
        release: the input thread keeps publishing full throttle and the car
        keeps driving, with the app transmitting happily so no failsafe fires.
        Both methods this relies on existed in the previous station and had no
        callers at all.
        """
        if event.type() == QEvent.Type.WindowDeactivate:
            self._model.release_all_keys()
        super().changeEvent(event)

    # -- model --------------------------------------------------------------

    def _on_tick(self, _now: float) -> None:
        self.video.tick()

    def _on_vehicle(self, v: VehicleSnapshot) -> None:
        self.hud.set_vehicle(v)
        self._act_arm.setEnabled(not v.armed)
        self._act_disarm.setEnabled(v.armed)
        if v.armed != self._armed:
            self._armed = v.armed
            if v.armed:
                self._on_armed()

    def _on_armed(self) -> None:
        """The car says it is armed. Show the driver the car.

        Deliberately not symmetric: arming hides the panel and goes fullscreen,
        disarming does neither. A driver who disarms to adjust something usually
        re-arms within seconds, and yanking the window out of fullscreen each
        time would be worse than leaving it.
        """
        if self._panel_open:
            self._hide_panel()
        if not self.isFullScreen():
            self.showFullScreen()
            self._act_full.setChecked(True)

    def _on_link(self, link: LinkSnapshot) -> None:
        self.hud.set_link(link)
        self.panel.set_link(link)
        # A frame stops being evidence about the world the moment it stops being
        # replaced -- and a dead link is the strongest case of that, not an
        # exemption from it. Gating this on `state.usable` (as the previous
        # station did) meant the picture went un-dimmed and un-labelled at
        # exactly the moment it was most dangerous: the car gone, the last frame
        # still on screen, looking like a road that is still there.
        self.video.set_stale(
            not link.state.usable or not link.video_ok or link.video_age > _VIDEO_STALE_S
        )
        self.video.set_placeholder(*_placeholder_for(link))
        # Come back automatically when there is nothing to drive. The car
        # E-stopped when the session dropped, so the driver has to act anyway;
        # putting the panel in front of them is the whole of the recovery UI.
        if link.state in (LinkState.OFFLINE, LinkState.FAILED) and not self._panel_open:
            self.show_panel()

    def _on_session(self, session: SessionSnapshot) -> None:
        if session.active and not self._session_active:
            self._session_active = True
            if session.car_id:
                self._settings.car_id = session.car_id
            self._hide_panel()
            _log.info(
                "connected to %s (firmware %s)",
                session.car_id or "car",
                session.fw_version or "unknown",
            )
        elif not session.active:
            self._session_active = False

    # -- shutdown -----------------------------------------------------------

    def closeEvent(self, event: QCloseEvent) -> None:
        self._settings.save()
        super().closeEvent(event)


def _placeholder_for(link: LinkSnapshot) -> tuple[str, str]:
    """What to say when there is no picture, given why there is no picture.

    "No video" is useless on its own -- it is true whether the car is switched
    off, the camera process died, or nobody has pressed connect yet, and those
    have completely different next actions.
    """
    state = link.state
    if state is LinkState.OFFLINE:
        return ("not connected", "press ⌘K to connect to a car")
    if state is LinkState.RESOLVING:
        return ("looking for the car", link.detail or "")
    if state is LinkState.CONNECTING:
        return ("connecting", link.detail or link.address)
    if state is LinkState.FAILED:
        return ("connection failed", link.detail)
    if not link.video_ok:
        return ("no video", "the car is connected but not sending a picture")
    return ("waiting for the first frame", link.detail)


__all__ = ["MainWindow"]
