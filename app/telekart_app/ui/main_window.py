"""The window: a 64 px navigation rail, a stack of screens, and a native menu.

**A rail, not tabs and not a hamburger.** Five destinations, all of them
top-level, all of them reachable in one click from anywhere -- that is what a
rail is for, and it is the idiom every professional tool with this shape uses.
Tabs would imply the screens are views of one document; a menu would hide the
destinations behind a click and cost the driver the muscle memory of "the drive
screen is the second icon down".

**A real menu bar.** On macOS it is the system menu bar, and every action that
has a standard chord uses ``QKeySequence.StandardKey`` so it comes out as ⌘Q,
⌘, and ⌃⌘F rather than a Windows shortcut transliterated onto a Mac. Actions
that macOS relocates -- About, Preferences, Quit -- carry the right ``MenuRole``
so they land in the application menu where a Mac user looks for them.

**No modal dialogs except the calibration wizard.** A modal over a live video
feed is a modal over a moving car. Errors go to the status bar and to inline
labels on the screen that caused them, where they can be read without being
dismissed first.

**Escape leaves fullscreen and does nothing else.** In particular it never
disarms. The reflex to hit Escape when something looks wrong is universal, and
if that reflex cut the motors at speed the car would be a projectile with no
steering -- the correct panic control is the E-stop, which is on a chord you
cannot hit by accident and on a physical button on the wheel.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable

from PySide6.QtCore import QSize, Qt, Slot
from PySide6.QtGui import QAction, QCloseEvent, QKeyEvent, QKeySequence
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QStackedWidget,
    QStatusBar,
    QWidget,
)

from telekart_protocol import Fault

from .. import APP_NAME
from ..config.settings import Settings, save_shared_key
from ..input.profile import ProfileStore
from ..model.snapshots import LinkSnapshot, SessionSnapshot, VehicleSnapshot
from ..model.units import SpeedMode, UnitFormatter, state_text
from .dialogs.about import AboutDialog
from .dialogs.calibration_wizard import CalibrationWizard
from .screens.connect import ConnectScreen
from .screens.diagnostics import DiagnosticsScreen
from .screens.drive import DriveScreen
from .screens.garage import GarageScreen
from .screens.settings import SettingsScreen
from .theme.tokens import THEME, Theme
from .widgets.rail import RAIL_WIDTH, NavigationRail

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..main import SimulatorController
    from ..model.app_model import AppModel

#: Rail order. Connect first because it is where a cold start begins, drive
#: second because it is where every session ends up, settings pinned to the
#: bottom because it is chrome rather than a destination.
_RAIL: tuple[tuple[str, str, str, bool], ...] = (
    ("connect", "link", "Connect", False),
    ("drive", "drive", "Drive", False),
    ("garage", "tune", "Garage", False),
    ("diagnostics", "telemetry", "Diagnostics", False),
    ("settings", "settings", "Settings", True),
)

_STATUS_MS = 6000


class MainWindow(QMainWindow):
    """Owns the screens, the menu, and the two window-level behaviours
    (fullscreen and screen switching) that no screen may reach up and perform."""

    def __init__(
        self,
        model: "AppModel",
        settings: Settings,
        profiles: ProfileStore,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        simulator: "SimulatorController | None" = None,
    ) -> None:
        super().__init__(parent)
        self._model = model
        self._settings = settings
        self._profiles = profiles
        self._theme = theme
        self._simulator = simulator
        self._pending_key = ""
        self._remember_key = False
        self._session_active = False

        self.setWindowTitle(APP_NAME)
        self.setMinimumSize(QSize(1024, 680))

        self._build_central(theme)
        self._build_status_bar(theme)
        self._build_menus()

        model.linkChanged.connect(self._on_link)
        model.vehicleChanged.connect(self._on_vehicle)
        model.sessionChanged.connect(self._on_session)
        model.faultRaised.connect(self._on_fault)

        # After the menus exist: the screen switcher enables menu actions.
        self.show_screen("connect")
        self.apply_display_settings()
        self._on_link(model.link)
        self._on_vehicle(model.vehicle)
        self._on_session(model.session)

    # -- construction -------------------------------------------------------

    def _build_central(self, theme: Theme) -> None:
        central = QWidget(self)
        central.setObjectName("Root")
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.rail = NavigationRail(central, theme=theme)
        for key, icon, tooltip, bottom in _RAIL:
            self.rail.add_item(key, icon, tooltip, bottom=bottom)
        self.rail.setFixedWidth(RAIL_WIDTH)
        self.rail.currentChanged.connect(self.show_screen)
        layout.addWidget(self.rail)

        self.stack = QStackedWidget(central)
        layout.addWidget(self.stack, 1)

        self.connect_screen = ConnectScreen(self._model, self.stack, theme=theme)
        self.drive_screen = DriveScreen(self._model, self.stack, theme=theme)
        self.garage_screen = GarageScreen(self._model, self.stack, theme=theme)
        self.diagnostics_screen = DiagnosticsScreen(self._model, self.stack, theme=theme)
        self.settings_screen = SettingsScreen(
            self._model, self._settings, self._profiles, self.stack, theme=theme
        )

        self._screens: dict[str, QWidget] = {
            "connect": self.connect_screen,
            "drive": self.drive_screen,
            "garage": self.garage_screen,
            "diagnostics": self.diagnostics_screen,
            "settings": self.settings_screen,
        }
        for widget in self._screens.values():
            self.stack.addWidget(widget)

        self.connect_screen.connectRequested.connect(self._on_connect_requested)
        self.connect_screen.disconnectRequested.connect(self._model.disconnect)
        self.connect_screen.simulatorRequested.connect(self.launch_simulator)

        self.garage_screen.paramsSubmitted.connect(self._model.set_params)
        self.garage_screen.reloadRequested.connect(self._model.request_params)
        self.garage_screen.calibrationRequested.connect(self._model.calibrate)

        self.settings_screen.settingsChanged.connect(self._on_settings_changed)
        self.settings_screen.profileChanged.connect(self._model.set_input_profile)
        self.settings_screen.calibrationRequested.connect(self.open_calibration_wizard)

        self.drive_screen.armedChanged.connect(self._on_armed)

        self.setCentralWidget(central)

    def _build_status_bar(self, theme: Theme) -> None:
        bar = QStatusBar(self)
        bar.setSizeGripEnabled(False)
        # Permanent widgets are packed hard against the right edge by
        # QStatusBar's own layout, which does not read stylesheet padding --
        # so without this the vehicle state's last letter touches the window
        # frame. Symmetric so the transient message on the left matches.
        bar.setContentsMargins(theme.space.md, 0, theme.space.md, 0)

        self._status_state = QLabel("—", bar)
        self._status_state.setProperty("variant", "eyebrow")
        self._status_link = QLabel("offline", bar)
        self._status_link.setProperty("variant", "caption")
        self._status_car = QLabel("", bar)
        self._status_car.setProperty("variant", "caption")

        for widget in (self._status_car, self._status_link, self._status_state):
            bar.addPermanentWidget(widget)
        self.setStatusBar(bar)

    def _build_menus(self) -> None:
        bar = self.menuBar()
        # Explicit, though it is the default on macOS: the alternative is a
        # menu bar drawn inside the window, and on a fullscreen driving view
        # that is a strip of chrome across the top of the picture.
        bar.setNativeMenuBar(True)

        file_menu = bar.addMenu("&File")
        self._add(file_menu, "Connect…", QKeySequence.StandardKey.Open,
                  lambda: self.show_screen("connect"))
        self._add(file_menu, "Disconnect", None, self._model.disconnect)
        self._add(file_menu, "Launch simulator", "Ctrl+Shift+S", self.launch_simulator)
        file_menu.addSeparator()
        prefs = self._add(
            file_menu, "Preferences…", QKeySequence.StandardKey.Preferences,
            lambda: self.show_screen("settings"),
        )
        prefs.setMenuRole(QAction.MenuRole.PreferencesRole)
        quit_action = self._add(
            file_menu, "Quit " + APP_NAME, QKeySequence.StandardKey.Quit, self.close
        )
        quit_action.setMenuRole(QAction.MenuRole.QuitRole)

        car = bar.addMenu("&Car")
        self._act_arm = self._add(car, "Arm", "Ctrl+Return", self._model.arm)
        self._act_disarm = self._add(car, "Disarm", "Ctrl+Shift+Return", self._model.disarm)
        car.addSeparator()
        # Cmd+. is the Mac convention for "stop what you are doing", and it is
        # unreachable by accident with one hand on a wheel.
        self._add(car, "Emergency stop", "Ctrl+.", self._model.estop)
        self._add(car, "Clear E-stop", None, self._model.clear_estop)
        self._add(car, "Clear faults", None, self._model.clear_faults)
        self._add(car, "Reset odometry", None, self._model.reset_odom)
        car.addSeparator()
        self._add(car, "Push parameters", "Ctrl+S", self.garage_screen.push)
        self._add(car, "Reload parameters", None, self._model.request_params)
        self._add(car, "Calibrate controls…", None, self.open_calibration_wizard)

        view = bar.addMenu("&View")
        for index, (key, _icon, label, _bottom) in enumerate(_RAIL, start=1):
            self._add(view, label, "Ctrl+%d" % (index,), _screen_switcher(self, key))
        view.addSeparator()

        self._act_hud = self._add_checkable(
            view, "Head-up display", "Ctrl+H", self._toggle_hud
        )
        self._act_hud.setChecked(True)
        self._act_diag = self._add_checkable(
            view, "Diagnostic overlay", "Ctrl+D", self._toggle_diag_overlay
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

        window = bar.addMenu("&Window")
        self._add(window, "Minimise", "Ctrl+M", self.showMinimized)
        self._add(window, "Close", QKeySequence.StandardKey.Close, self.close)

        help_menu = bar.addMenu("&Help")
        about = self._add(help_menu, "About " + APP_NAME, None, self.open_about)
        about.setMenuRole(QAction.MenuRole.AboutRole)

    def _add(
        self,
        menu: Any,
        text: str,
        shortcut: QKeySequence.StandardKey | str | None,
        slot: Callable[[], None],
    ) -> QAction:
        """A plain action whose slot takes no arguments.

        ``QAction.triggered`` carries a ``checked`` flag, so the slot is wrapped
        rather than connected directly. Binding a zero-argument method straight
        to it relies on the binding layer quietly discarding the extra
        argument, which is exactly the sort of thing that works until the day a
        model method grows an optional first parameter and starts receiving
        ``False`` for it.
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
        # Window-scoped so the chord still works when the drive screen has
        # taken focus for its own key handling.
        action.setShortcutContext(Qt.ShortcutContext.WindowShortcut)
        menu.addAction(action)
        self.addAction(action)
        return action

    # -- navigation ---------------------------------------------------------

    @Slot(str)
    def show_screen(self, key: str) -> None:
        widget = self._screens.get(key)
        if widget is None:
            return
        self.stack.setCurrentWidget(widget)
        self.rail.set_current(key)
        self._act_diag.setEnabled(key == "drive")
        self._act_hud.setEnabled(key == "drive")

    @property
    def current_screen(self) -> str:
        for key, widget in self._screens.items():
            if widget is self.stack.currentWidget():
                return key
        return ""

    # -- window behaviours --------------------------------------------------

    def toggle_fullscreen(self) -> None:
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()
        self._act_full.setChecked(self.isFullScreen())

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
        super().keyPressEvent(event)

    def _on_armed(self, armed: bool) -> None:
        """The car says it is armed. Show the driver the car.

        Deliberately not symmetric: arming switches to the drive screen and
        goes fullscreen, disarming does neither. A driver who disarms to adjust
        something usually re-arms within seconds, and yanking the window out of
        fullscreen each time would be worse than leaving it.
        """
        if not armed:
            return
        self.show_screen("drive")
        if not self.isFullScreen():
            self.showFullScreen()
            self._act_full.setChecked(True)

    def _toggle_hud(self, checked: bool) -> None:
        self.drive_screen.set_hud_visible(checked)

    def _toggle_diag_overlay(self, checked: bool) -> None:
        self.drive_screen.set_diagnostics_visible(checked)
        self._settings.display.diagnostic_overlay = checked
        self._settings.save()

    # -- actions ------------------------------------------------------------

    def _on_connect_requested(self, host: str, key: str, remember: bool) -> None:
        self._pending_key = key
        self._remember_key = remember
        self._model.connect_to(host, shared_key=key)
        self.statusBar().showMessage("Connecting to %s…" % (host,), _STATUS_MS)

    def launch_simulator(self) -> None:
        if self._simulator is None:
            self.statusBar().showMessage(
                "No simulator is configured for this build.", _STATUS_MS
            )
            return
        ok, detail = self._simulator.launch()
        self.statusBar().showMessage(detail, _STATUS_MS)
        if ok:
            self._model.connect_to(self._simulator.host, shared_key=self._simulator.key)

    def open_calibration_wizard(self) -> None:
        profile = self._model.input_profile()
        if profile is None:
            self.statusBar().showMessage(
                "Plug a controller in first — the wizard calibrates a device, "
                "not a preference.",
                _STATUS_MS,
            )
            return
        wizard = CalibrationWizard(
            profile, self._model.device_snapshot, self, theme=self._theme
        )
        wizard.exec()
        calibrated = wizard.result_profile()
        if calibrated is None:
            return
        self._profiles.save_profile(calibrated)
        self._model.set_input_profile(calibrated)
        self.settings_screen.reload_profiles()
        self.statusBar().showMessage(
            "Saved the calibration for %s." % (calibrated.name or "this device",),
            _STATUS_MS,
        )

    def open_about(self) -> None:
        AboutDialog(self, theme=self._theme).exec()

    def apply_display_settings(self) -> None:
        display = self._settings.display
        formatter = UnitFormatter(
            speed_mode=SpeedMode.parse(display.speed_mode),
            scale_factor=display.scale_factor,
        )
        self.drive_screen.set_formatter(formatter)
        self.drive_screen.set_diagnostics_visible(display.diagnostic_overlay)
        self._act_diag.setChecked(display.diagnostic_overlay)

    def _on_settings_changed(self) -> None:
        self._settings.save()
        self.apply_display_settings()
        self.statusBar().showMessage("Settings saved.", 2000)

    # -- model --------------------------------------------------------------

    def _on_link(self, link: LinkSnapshot) -> None:
        self._status_link.setText(_link_text(link))
        self._status_car.setText(link.car_id or link.host or "")

    def _on_vehicle(self, vehicle: VehicleSnapshot) -> None:
        self._status_state.setText(
            state_text(vehicle.state) if vehicle.valid else "—"
        )
        self.rail.set_badge("diagnostics", _fault_count(vehicle.faults))
        self._act_arm.setEnabled(not vehicle.armed)
        self._act_disarm.setEnabled(vehicle.armed)

    def _on_session(self, session: SessionSnapshot) -> None:
        if session.active and not self._session_active:
            self._session_active = True
            if self._remember_key and self._pending_key and session.car_id:
                # Stored against the car id the *car* reported, not the host
                # that was typed: the same car answers to several names, and a
                # key filed under "192.168.1.42" is a key nobody finds again.
                save_shared_key(session.car_id, self._pending_key)
            self._pending_key = ""
            self.statusBar().showMessage(
                "Connected to %s (firmware %s)."
                % (session.car_id or "car", session.fw_version or "unknown"),
                _STATUS_MS,
            )
        elif not session.active:
            self._session_active = False
        if session.last_error:
            self.statusBar().showMessage(session.last_error, _STATUS_MS)

    def _on_fault(self, bit: int, text: str) -> None:
        self.statusBar().showMessage("Fault: " + text, _STATUS_MS)

    # -- shutdown -----------------------------------------------------------

    def closeEvent(self, event: QCloseEvent) -> None:
        self.connect_screen.shutdown()
        self.diagnostics_screen.shutdown()
        self._settings.save()
        super().closeEvent(event)


def _screen_switcher(window: MainWindow, key: str) -> Any:
    """A late-binding-free switcher for the View menu.

    A lambda in the loop would capture the loop variable, so every entry would
    open the last screen -- the classic version of this bug, and one that looks
    fine until the fifth menu item.
    """

    def switch() -> None:
        window.show_screen(key)

    return switch


def _link_text(link: LinkSnapshot) -> str:
    if not link.state.usable:
        return link.state.value
    return "%s · %.0f ms · %.1f%% loss" % (
        link.state.value,
        link.rtt * 1000.0,
        link.loss * 100.0,
    )


def _fault_count(faults: Fault) -> int:
    return int(faults).bit_count()


__all__ = ["MainWindow"]
