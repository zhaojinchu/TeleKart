"""Application preferences, and the one place the driver shapes the feel.

Two kinds of setting live here and it is worth being explicit about which is
which, because they are persisted differently and they mean different things.

*Application settings* -- units, video, link timeouts, recording -- belong to
this machine and are written to ``settings.json``. They never leave the app.

*Input feel* -- deadzone, saturation, response curve, rate limits -- belongs to
the **device**, is keyed by its SDL GUID, and is written to the profile store.
Unplug the wheel, plug it into a different port or a different laptop with the
same profile file, and the calibration follows it.

The division of responsibility between this screen and the firmware is settled
and shows up here as a warning rather than a slider: the app owns *feel* and the
firmware owns *protection*. The rate limits below must stay tighter than the
firmware's duty slew clamp, so that the firmware's limiter never engages during
normal driving -- and when it does, ``TelemetryFlags.LIMITER_ACTIVE`` lights up
the HUD so you can see that the two have drifted apart.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from PySide6.QtCore import QSize, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ...config.settings import Settings
from ...input.chain import (
    MAX_DEADZONE,
    MAX_SATURATION,
    AxisChainConfig,
    ChainConfigError,
)
from ...input.curves import CUSTOM_POINT_COUNT, Curve, CurveKind, CurveSpec, evaluate
from ...input.mapping import Control
from ...input.profile import DeviceProfile, ProfileStore
from ...model.units import SpeedMode
from ..theme.tokens import THEME, Theme
from ..widgets.card import Card
from ..widgets.curve_editor import CurveEditor

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...model.app_model import AppModel

_SPEED_MODES: tuple[tuple[str, SpeedMode], ...] = (
    ("Scale km/h", SpeedMode.SCALE_KMH),
    ("Scale mph", SpeedMode.SCALE_MPH),
    ("Real km/h", SpeedMode.REAL_KMH),
    ("Real m/s", SpeedMode.REAL_MPS),
)

_CONTROLS: tuple[tuple[str, Control], ...] = (
    ("Steering", Control.STEER),
    ("Throttle", Control.THROTTLE),
    ("Brake", Control.BRAKE),
)

_CURVE_KINDS: tuple[tuple[str, CurveKind], ...] = (
    ("Linear", CurveKind.LINEAR),
    ("Expo", CurveKind.EXPO),
    ("S-curve", CurveKind.SCURVE),
    ("Custom", CurveKind.CUSTOM),
)

#: Writes are coalesced: dragging a spin box emits a change per step, and
#: rewriting a JSON file forty times during one drag is pointless I/O.
_SAVE_DELAY_MS = 500


class SettingsScreen(QWidget):
    """Preferences, unit choices, and per-device input feel."""

    settingsChanged = Signal()
    profileChanged = Signal(object)
    calibrationRequested = Signal()

    def __init__(
        self,
        model: "AppModel",
        settings: Settings,
        profiles: ProfileStore,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
    ) -> None:
        super().__init__(parent)
        self._model = model
        self._settings = settings
        self._profiles = profiles
        self._theme = theme
        self._profile: DeviceProfile | None = None
        self._loading = False

        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.setInterval(_SAVE_DELAY_MS)
        self._save_timer.timeout.connect(self.settingsChanged.emit)

        self.setObjectName("Root")
        self._build(theme)
        self._load_from_settings()
        self._reload_profiles()

    # -- construction -------------------------------------------------------

    def _build(self, theme: Theme) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        outer.addWidget(scroll)

        page = QWidget(scroll)
        scroll.setWidget(page)

        columns = QHBoxLayout(page)
        columns.setContentsMargins(
            theme.space.xl, theme.space.xl, theme.space.xl, theme.space.xl
        )
        columns.setSpacing(theme.space.lg)

        left = QVBoxLayout()
        left.setSpacing(theme.space.lg)
        right = QVBoxLayout()
        right.setSpacing(theme.space.lg)
        columns.addLayout(left, 1)
        columns.addLayout(right, 1)

        left.addWidget(self._build_driver(theme))
        left.addWidget(self._build_display(theme))
        left.addWidget(self._build_recording(theme))
        left.addStretch(1)

        right.addWidget(self._build_link(theme))
        right.addWidget(self._build_video(theme))
        right.addWidget(self._build_input(theme))
        right.addStretch(1)

    def _build_driver(self, theme: Theme) -> Card:
        card = Card("Driver", self, theme=theme)
        grid = _grid(theme)

        self.driver = QLineEdit(card)
        self.driver.setPlaceholderText("shown in the car's session log")
        self.driver.textEdited.connect(self._on_changed)
        _row(grid, 0, "Name", self.driver, card)

        self.car_id = QLineEdit(card)
        self.car_id.setPlaceholderText("blank connects to whichever car answers")
        self.car_id.textEdited.connect(self._on_changed)
        _row(grid, 1, "Preferred car", self.car_id, card)

        card.add_layout(grid)
        return card

    def _build_display(self, theme: Theme) -> Card:
        card = Card("Display", self, theme=theme)
        grid = _grid(theme)

        self.speed_mode = QComboBox(card)
        for label, _ in _SPEED_MODES:
            self.speed_mode.addItem(label)
        self.speed_mode.setToolTip(
            "A 1:10 car at 3 m/s is doing a scale 108 km/h. The real number "
            "reads as slow; the scale number is the speed the scene is moving "
            "at in the size the driver perceives. Engineering readouts always "
            "show true SI regardless of this setting."
        )
        self.speed_mode.currentIndexChanged.connect(self._on_changed)
        _row(grid, 0, "Speedometer", self.speed_mode, card)

        self.scale_factor = QDoubleSpinBox(card)
        self.scale_factor.setRange(1.0, 32.0)
        self.scale_factor.setSingleStep(0.5)
        self.scale_factor.setDecimals(1)
        self.scale_factor.setSuffix(" : 1")
        self.scale_factor.valueChanged.connect(self._on_changed)
        _row(grid, 1, "Chassis scale", self.scale_factor, card)

        self.diag_overlay = QCheckBox("Show the diagnostic overlay on the HUD", card)
        self.diag_overlay.toggled.connect(self._on_changed)
        grid.addWidget(self.diag_overlay, 2, 1)

        card.add_layout(grid)
        return card

    def _build_recording(self, theme: Theme) -> Card:
        card = Card("Recording", self, theme=theme,
                    subtitle="About 8 MB an hour")
        grid = _grid(theme)

        self.record_enabled = QCheckBox("Record every session", card)
        self.record_enabled.setToolTip(
            "A session you did not record is a session you cannot explain "
            "afterwards."
        )
        self.record_enabled.toggled.connect(self._on_changed)
        grid.addWidget(self.record_enabled, 0, 1)

        self.record_hz = QDoubleSpinBox(card)
        self.record_hz.setRange(1.0, 50.0)
        self.record_hz.setSingleStep(5.0)
        self.record_hz.setDecimals(0)
        self.record_hz.setSuffix(" Hz")
        self.record_hz.setToolTip(
            "Telemetry arrives at 50 Hz. 25 is plenty for after-the-fact "
            "analysis; match the telemetry rate when you are tuning."
        )
        self.record_hz.valueChanged.connect(self._on_changed)
        _row(grid, 1, "Telemetry rate", self.record_hz, card)

        self.record_inputs = QCheckBox("Record driver inputs", card)
        self.record_inputs.toggled.connect(self._on_changed)
        grid.addWidget(self.record_inputs, 2, 1)

        self.record_keep = QSpinBox(card)
        self.record_keep.setRange(1, 3650)
        self.record_keep.setSuffix(" days")
        self.record_keep.valueChanged.connect(self._on_changed)
        _row(grid, 3, "Keep for", self.record_keep, card)

        card.add_layout(grid)
        return card

    def _build_link(self, theme: Theme) -> Card:
        card = Card("Link", self, theme=theme)
        grid = _grid(theme)

        self.link_host = QLineEdit(card)
        self.link_host.setPlaceholderText("blank discovers telekart.local")
        self.link_host.textEdited.connect(self._on_changed)
        _row(grid, 0, "Pinned host", self.link_host, card)

        self.telemetry_port = QSpinBox(card)
        self.telemetry_port.setRange(0, 65535)
        self.telemetry_port.setToolTip(
            "0 asks the OS for a free port, which is what lets two apps run on "
            "one machine. The port is sent to the car in the handshake."
        )
        self.telemetry_port.valueChanged.connect(self._on_changed)
        _row(grid, 1, "Telemetry port", self.telemetry_port, card)

        self.auto_connect = QCheckBox("Connect automatically at startup", card)
        self.auto_connect.toggled.connect(self._on_changed)
        grid.addWidget(self.auto_connect, 2, 1)

        self.estop_burst = QSpinBox(card)
        self.estop_burst.setRange(1, 40)
        self.estop_burst.setToolTip(
            "UDP loses packets exactly when you least want it to, so the one "
            "message that must arrive is never entrusted to a single datagram."
        )
        self.estop_burst.valueChanged.connect(self._on_changed)
        _row(grid, 3, "E-stop burst", self.estop_burst, card)

        card.add_layout(grid)
        return card

    def _build_video(self, theme: Theme) -> Card:
        card = Card("Video", self, theme=theme)
        grid = _grid(theme)

        self.video_enabled = QCheckBox("Receive video", card)
        self.video_enabled.toggled.connect(self._on_changed)
        grid.addWidget(self.video_enabled, 0, 1)

        self.freeze_on_corruption = QCheckBox("Hide corrupted frames", card)
        self.freeze_on_corruption.setToolTip(
            "Off by default. For teleoperation a corrupted-but-live picture "
            "beats a frozen clean one, because a frozen picture looks like a "
            "road that is still there."
        )
        self.freeze_on_corruption.toggled.connect(self._on_changed)
        grid.addWidget(self.freeze_on_corruption, 1, 1)

        self.video_stats = QCheckBox("Show decoder statistics", card)
        self.video_stats.toggled.connect(self._on_changed)
        grid.addWidget(self.video_stats, 2, 1)

        note = QLabel(
            "Resolution, frame rate and bitrate belong to the camera and live "
            "in Garage → Camera, because the car has to agree to them.",
            card,
        )
        note.setProperty("variant", "caption")
        note.setWordWrap(True)
        grid.addWidget(note, 3, 0, 1, 2)

        card.add_layout(grid)
        return card

    def _build_input(self, theme: Theme) -> Card:
        card = Card("Input feel", self, theme=theme,
                    subtitle="Stored per device, keyed by its GUID")

        top = QHBoxLayout()
        top.setSpacing(theme.space.sm)
        self.profile_box = QComboBox(card)
        self.profile_box.currentIndexChanged.connect(self._on_profile_selected)
        top.addWidget(self.profile_box, 1)

        calibrate = QPushButton("Calibrate…", card)
        calibrate.setProperty("variant", "primary")
        calibrate.clicked.connect(self.calibrationRequested.emit)
        top.addWidget(calibrate)
        card.add_layout(top)

        self.control_box = QComboBox(card)
        for label, _ in _CONTROLS:
            self.control_box.addItem(label)
        self.control_box.currentIndexChanged.connect(self._load_axis)
        card.add_widget(self.control_box)

        self.curve = CurveEditor(card, theme=theme)
        self.curve.setMinimumHeight(200)
        self.curve.curveChanged.connect(self._on_curve_edited)
        card.add_widget(self.curve)

        grid = _grid(theme)
        self.curve_kind = QComboBox(card)
        for label, _ in _CURVE_KINDS:
            self.curve_kind.addItem(label)
        self.curve_kind.currentIndexChanged.connect(self._on_curve_kind)
        _row(grid, 0, "Curve", self.curve_kind, card)

        self.curve_amount = QDoubleSpinBox(card)
        self.curve_amount.setRange(0.1, 8.0)
        self.curve_amount.setSingleStep(0.1)
        self.curve_amount.setDecimals(2)
        self.curve_amount.valueChanged.connect(self._on_curve_kind)
        _row(grid, 1, "Amount", self.curve_amount, card)

        # Ranges come from the chain module rather than from taste. Two limits
        # govern these: each stage has its own ceiling, and together they must
        # leave usable travel -- and only the chain knows both numbers.
        self.deadzone = QDoubleSpinBox(card)
        self.deadzone.setRange(0.0, MAX_DEADZONE)
        self.deadzone.setSingleStep(0.01)
        self.deadzone.setDecimals(3)
        self.deadzone.valueChanged.connect(self._on_axis_edited)
        _row(grid, 2, "Deadzone", self.deadzone, card)

        self.saturation = QDoubleSpinBox(card)
        self.saturation.setRange(0.0, MAX_SATURATION)
        self.saturation.setSingleStep(0.01)
        self.saturation.setDecimals(3)
        self.saturation.setToolTip(
            "Clips the top of the travel and rescales, so full scale is "
            "reachable without bottoming the pedal out."
        )
        self.saturation.valueChanged.connect(self._on_axis_edited)
        _row(grid, 3, "Saturation", self.saturation, card)

        self.rate_rise = QDoubleSpinBox(card)
        self.rate_rise.setRange(0.5, 60.0)
        self.rate_rise.setSingleStep(0.5)
        self.rate_rise.setDecimals(1)
        self.rate_rise.setSuffix(" /s")
        self.rate_rise.valueChanged.connect(self._on_axis_edited)
        _row(grid, 4, "Rate, applying", self.rate_rise, card)

        self.rate_fall = QDoubleSpinBox(card)
        self.rate_fall.setRange(0.5, 120.0)
        self.rate_fall.setSingleStep(0.5)
        self.rate_fall.setDecimals(1)
        self.rate_fall.setSuffix(" /s")
        self.rate_fall.valueChanged.connect(self._on_axis_edited)
        _row(grid, 5, "Rate, releasing", self.rate_fall, card)
        card.add_layout(grid)

        self.input_error = QLabel("", card)
        self.input_error.setProperty("variant", "fault")
        self.input_error.setWordWrap(True)
        self.input_error.setVisible(False)
        card.add_widget(self.input_error)

        note = QLabel(
            "Keep these rate limits tighter than the firmware's slew clamp. If "
            "the car's limiter engages during normal driving the HUD and the "
            "car disagree, which feels like the wheel has gone soft.",
            card,
        )
        note.setProperty("variant", "caption")
        note.setWordWrap(True)
        card.add_widget(note)
        return card

    # -- application settings ----------------------------------------------

    def _load_from_settings(self) -> None:
        self._loading = True
        s = self._settings
        self.driver.setText(s.driver)
        self.car_id.setText(s.car_id)

        mode = SpeedMode.parse(s.display.speed_mode)
        for index, (_, candidate) in enumerate(_SPEED_MODES):
            if candidate is mode:
                self.speed_mode.setCurrentIndex(index)
                break
        self.scale_factor.setValue(s.display.scale_factor)
        self.diag_overlay.setChecked(s.display.diagnostic_overlay)

        self.record_enabled.setChecked(s.recording.enabled)
        self.record_hz.setValue(s.recording.telemetry_hz)
        self.record_inputs.setChecked(s.recording.inputs)
        self.record_keep.setValue(s.recording.keep_days)

        self.link_host.setText(s.link.host)
        self.telemetry_port.setValue(s.link.telemetry_port)
        self.auto_connect.setChecked(s.link.auto_connect)
        self.estop_burst.setValue(s.link.estop_burst)

        self.video_enabled.setChecked(s.video.enabled)
        self.freeze_on_corruption.setChecked(s.video.freeze_on_corruption)
        self.video_stats.setChecked(s.video.show_stats_overlay)
        self._loading = False

    def _on_changed(self, *_args: Any) -> None:
        if self._loading:
            return
        s = self._settings
        s.driver = self.driver.text().strip()
        s.car_id = self.car_id.text().strip()

        s.display.speed_mode = _SPEED_MODES[self.speed_mode.currentIndex()][1].value
        s.display.scale_factor = self.scale_factor.value()
        s.display.diagnostic_overlay = self.diag_overlay.isChecked()

        s.recording.enabled = self.record_enabled.isChecked()
        s.recording.telemetry_hz = self.record_hz.value()
        s.recording.inputs = self.record_inputs.isChecked()
        s.recording.keep_days = self.record_keep.value()

        s.link.host = self.link_host.text().strip()
        s.link.telemetry_port = self.telemetry_port.value()
        s.link.auto_connect = self.auto_connect.isChecked()
        s.link.estop_burst = self.estop_burst.value()

        s.video.enabled = self.video_enabled.isChecked()
        s.video.freeze_on_corruption = self.freeze_on_corruption.isChecked()
        s.video.show_stats_overlay = self.video_stats.isChecked()

        self._save_timer.start()

    # -- device profiles ----------------------------------------------------

    def reload_profiles(self) -> None:
        """Re-read the store. Called after the calibration wizard writes one."""
        self._reload_profiles()

    def _reload_profiles(self) -> None:
        self._loading = True
        current = self._profile.profile_id if self._profile is not None else ""
        self.profile_box.clear()
        profiles = list(self._profiles.all())
        for profile in profiles:
            self.profile_box.addItem(profile.name or profile.profile_id, profile.profile_id)
        self._loading = False

        if not profiles:
            self._profile = None
            self.curve.setEnabled(False)
            self.input_error.setText(
                "No device profile yet. Plug a wheel or a pad in and run "
                "Calibrate — the profile is created from what it actually does."
            )
            self.input_error.setVisible(True)
            return

        self.curve.setEnabled(True)
        self.input_error.setVisible(False)
        index = max(0, self.profile_box.findData(current))
        self.profile_box.setCurrentIndex(index)
        self._on_profile_selected()

    def _on_profile_selected(self) -> None:
        if self._loading:
            return
        profile_id = self.profile_box.currentData()
        self._profile = (
            self._profiles.get(str(profile_id)) if profile_id is not None else None
        )
        self._load_axis()

    def _current_control(self) -> Control:
        return _CONTROLS[max(0, self.control_box.currentIndex())][1]

    def _load_axis(self) -> None:
        profile = self._profile
        if profile is None:
            return
        self._loading = True
        control = self._current_control()
        axis = profile.chain.axis(control)
        spec = axis.curve.spec

        for index, (_, kind) in enumerate(_CURVE_KINDS):
            if kind is spec.kind:
                self.curve_kind.setCurrentIndex(index)
                break
        self.curve_amount.setValue(
            spec.gamma if spec.kind is CurveKind.EXPO else spec.strength
        )
        self.curve_amount.setEnabled(
            spec.kind in (CurveKind.EXPO, CurveKind.SCURVE)
        )
        self.curve.set_editable(spec.kind is CurveKind.CUSTOM)
        self.curve.set_points(_points_from_spec(spec))
        self.curve.set_deadzone(axis.deadzone)
        self.curve.set_saturation(axis.saturation)

        self.deadzone.setValue(axis.deadzone)
        self.saturation.setValue(axis.saturation)
        self.rate_rise.setValue(axis.rate_rise)
        self.rate_fall.setValue(axis.rate_fall)
        self._loading = False

    def _on_curve_kind(self, *_args: Any) -> None:
        if self._loading or self._profile is None:
            return
        kind = _CURVE_KINDS[self.curve_kind.currentIndex()][1]
        self.curve_amount.setEnabled(kind in (CurveKind.EXPO, CurveKind.SCURVE))
        self.curve.set_editable(kind is CurveKind.CUSTOM)
        if kind is CurveKind.EXPO:
            spec = CurveSpec.expo(self.curve_amount.value())
        elif kind is CurveKind.SCURVE:
            spec = CurveSpec.scurve(min(1.0, self.curve_amount.value()))
        elif kind is CurveKind.CUSTOM:
            spec = CurveSpec.custom(_sample_editor(self.curve))
        else:
            spec = CurveSpec.linear()
        self._loading = True
        self.curve.set_points(_points_from_spec(spec))
        self._loading = False
        self._store_axis(spec)

    def _on_curve_edited(self) -> None:
        if self._loading or self._profile is None:
            return
        if _CURVE_KINDS[self.curve_kind.currentIndex()][1] is not CurveKind.CUSTOM:
            return
        self._store_axis(CurveSpec.custom(_sample_editor(self.curve)))

    def _on_axis_edited(self, *_args: Any) -> None:
        if self._loading or self._profile is None:
            return
        self.curve.set_deadzone(self.deadzone.value())
        self.curve.set_saturation(self.saturation.value())
        self._store_axis(None)

    def _store_axis(self, spec: CurveSpec | None) -> None:
        """Rebuild the axis config and persist it.

        ``AxisChainConfig`` validates in its constructor -- deadzone plus
        saturation must leave usable travel -- and that validation is the right
        place for it. Here it is a user typing, not a program starting, so the
        failure is shown inline and the previous configuration is kept.
        """
        profile = self._profile
        if profile is None:
            return
        control = self._current_control()
        axis = profile.chain.axis(control)
        try:
            updated = AxisChainConfig(
                calibration=axis.calibration,
                deadzone=self.deadzone.value(),
                saturation=self.saturation.value(),
                curve=Curve(spec) if spec is not None else axis.curve,
                rate_rise=self.rate_rise.value(),
                rate_fall=self.rate_fall.value(),
                smoothing=axis.smoothing,
            )
        except ChainConfigError as exc:
            self.input_error.setText(str(exc))
            self.input_error.setVisible(True)
            return

        self.input_error.setVisible(False)
        profile = profile.with_chain(profile.chain.with_axis(control, updated))
        self._profile = profile
        self._profiles.save_profile(profile)
        self.profileChanged.emit(profile)

    def sizeHint(self) -> QSize:
        return QSize(1040, 760)


# --------------------------------------------------------------------------
# Small layout helpers
# --------------------------------------------------------------------------


def _grid(theme: Theme) -> QGridLayout:
    grid = QGridLayout()
    grid.setContentsMargins(0, 0, 0, 0)
    grid.setHorizontalSpacing(theme.space.md)
    grid.setVerticalSpacing(theme.space.sm)
    grid.setColumnStretch(1, 1)
    return grid


def _row(grid: QGridLayout, row: int, label: str, widget: QWidget, parent: QWidget) -> None:
    caption = QLabel(label, parent)
    caption.setProperty("variant", "eyebrow")
    caption.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
    grid.addWidget(caption, row, 0)
    grid.addWidget(widget, row, 1)


def _points_from_spec(spec: CurveSpec) -> list[tuple[float, float]]:
    """The five control points a :class:`CurveEditor` shows for a spec."""
    step = 1.0 / (CUSTOM_POINT_COUNT - 1)
    return [(i * step, evaluate(spec, i * step)) for i in range(CUSTOM_POINT_COUNT)]


def _sample_editor(editor: CurveEditor) -> list[float]:
    """The editor's curve, resampled onto the five points a spec stores.

    The editor allows points anywhere; ``CurveSpec.custom`` stores outputs at
    fixed x. Sampling rather than copying means an editor with points dragged
    off the grid still round-trips to something the firmware-side registry
    would accept.
    """
    step = 1.0 / (CUSTOM_POINT_COUNT - 1)
    return [editor.evaluate(i * step) for i in range(CUSTOM_POINT_COUNT)]


__all__ = ["SettingsScreen"]
