"""Vehicle setup, generated from the shared parameter registry.

Not one line of this screen knows what a parameter is called. It walks
:data:`telekart_protocol.params.PARAMS` and builds a row per entry, choosing the
editor from ``kind``, the range from ``minimum``/``maximum``, the increment from
``step`` and the suffix from ``unit``. Adding a parameter to the registry makes
it appear here, correctly bounded and labelled, with no edit to this file -- and
more importantly, it cannot appear here *wrongly*, because the firmware
validates against the same registry that drew the widget.

The other rule this screen exists to enforce: **never show a value the car has
not echoed back.** Every row has two values. The editor holds what the operator
wants; the small readout beside it holds what the car last said it applied. They
are visually distinct and they never merge. A UI that optimistically shows the
typed value as though it were live is a UI that will, sooner or later, tell
somebody their steering trim is 12 µs when the car refused the push and is still
running 0 -- and they will spend an afternoon tuning against a number that was
never real.

Rows whose ``requires_disarm`` flag is set are locked while the car reports
ARMED. That is a courtesy, not a safety mechanism: the firmware refuses those
pushes itself. Doing it here as well means the refusal never happens, which is
better than showing the operator an error they could not have predicted.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Mapping

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QPainter, QPaintEvent
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from telekart_protocol import ErrorCode, VehicleState
from telekart_protocol.params import GROUPS, PARAMS, ParamDef
from telekart_protocol.params import group as params_in_group

from ...model.snapshots import SessionSnapshot, VehicleSnapshot
from ..theme.tokens import THEME, Theme, with_alpha
from ..widgets.card import Card

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...model.app_model import AppModel

_GROUP_TITLES: dict[str, str] = {
    "drive": "Drive",
    "pid": "Speed loop",
    "steering": "Steering",
    "geometry": "Geometry",
    "safety": "Safety",
    "pwm": "Bridge",
    "video": "Camera",
}

#: Row states, in the order they escalate. The dot colour comes from the theme
#: rather than from here so a fault dot and a fault badge are the same red.
_STATE_COLORS: dict[str, str] = {
    "unknown": "text_tertiary",
    "synced": "cyan",
    "dirty": "accent",
    "pending": "warn",
    "rejected": "bad",
}

#: Error codes that mean "your push was refused", as opposed to a session-level
#: problem. Taken from the protocol enum rather than written out, so a renamed
#: code is a build error here instead of a row whose dot never turns red.
_REJECTION_CODES: frozenset[str] = frozenset(
    (
        ErrorCode.PARAM_OUT_OF_RANGE.value,
        ErrorCode.UNKNOWN_PARAM.value,
        ErrorCode.NOT_ALLOWED_IN_STATE.value,
    )
)

_CALIBRATE_TIP = (
    "Measures the four max-RPM figures, the deadbands and the feedforward "
    "table on this car. Everything downstream scales off the result, which is "
    "why nothing in this application hardcodes a top speed."
)
_CALIBRATE_ARMED_TIP = (
    "Disarm first. Calibration drives both motors to their limits under its "
    "own control, and it will not start while a driver is holding the car."
)

_STATE_TOOLTIPS: dict[str, str] = {
    "unknown": "The car has not reported this value yet.",
    "synced": "The car has confirmed this value.",
    "dirty": "Edited but not pushed. The car is still running the value shown "
             "on the right.",
    "pending": "Pushed; waiting for the car to echo it back.",
    "rejected": "The car refused this value. It is still running the value "
                "shown on the right.",
}


#: Form column bounds. A parameter row is a label, its editor, and the value
#: the car reports; the first two have to stay side by side at any window width
#: or the association between them is lost, so the row's slack goes into a
#: spacer rather than into either column.
_LABEL_MAX_W = 340
_EDITOR_MIN_W = 180
_EDITOR_MAX_W = 320


class _StatusDot(QWidget):
    """Six pixels of state. Small on purpose -- forty-odd rows of these must
    read as a column of texture, with the odd bright one drawing the eye."""

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent)
        self._theme = theme
        self._state = "unknown"
        self.setFixedSize(QSize(14, 14))
        self.setToolTip(_STATE_TOOLTIPS["unknown"])

    def set_state(self, state: str) -> None:
        if state != self._state:
            self._state = state
            self.setToolTip(_STATE_TOOLTIPS.get(state, ""))
            self.update()

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        color = self._theme.rgb(_STATE_COLORS.get(self._state, "text_tertiary"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(with_alpha(color, 0.25))
        painter.drawEllipse(self.rect().center(), 6, 6)
        painter.setBrush(color)
        painter.drawEllipse(self.rect().center(), 3, 3)
        painter.end()


class ParamRow(QWidget):
    """One registry entry: label, editor, the car's applied value, and a dot."""

    edited = Signal(str)

    def __init__(
        self,
        definition: ParamDef,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
    ) -> None:
        super().__init__(parent)
        self.definition = definition
        self._theme = theme
        self._applied: Any = None
        self._state = "unknown"
        self._locked = False

        self.dot = _StatusDot(self, theme=theme)
        self.editor = _build_editor(definition, self)
        self.applied_label = QLabel("--", self)
        self.applied_label.setProperty("variant", "mono")
        self.applied_label.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        self.applied_label.setMinimumWidth(96)
        self.applied_label.setToolTip("The value the car reports it is running.")

        title = definition.label
        if definition.measured:
            title += " *"
        self.name_label = QLabel(title, self)
        self.name_label.setMinimumWidth(190)
        self.name_label.setWordWrap(True)

        tooltip = definition.description or definition.name
        if definition.measured:
            tooltip += (
                "\n\n* Measured during bring-up, not chosen. Overwriting it "
                "with a guess is how an afternoon disappears into tuning "
                "against the wrong plant."
            )
        if definition.requires_disarm:
            tooltip += "\n\nThe car refuses this change while armed."
        self.name_label.setToolTip(tooltip)
        self.editor.setToolTip(tooltip)

        # Bounded label and editor with the slack between them, not inside
        # them. Letting the label column stretch put half a metre of empty
        # screen between "Deceleration limit" and the box that sets it on a
        # wide window, which is the one thing a settings form must never do:
        # the reader has to be able to see which control belongs to which name
        # without tracking across the page.
        self.name_label.setMaximumWidth(_LABEL_MAX_W)
        self.editor.setMinimumWidth(_EDITOR_MIN_W)
        self.editor.setMaximumWidth(_EDITOR_MAX_W)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(theme.space.sm)
        layout.addWidget(self.dot)
        layout.addWidget(self.name_label, 0)
        layout.addWidget(self.editor, 1)
        layout.addStretch(1)
        # Pinned to the right edge, under its "On the car" header.
        layout.addWidget(self.applied_label, 0)

        _connect_editor(self.editor, self._on_edit)

    # -- value plumbing -----------------------------------------------------

    @property
    def name(self) -> str:
        return self.definition.name

    def value(self) -> Any:
        return _editor_value(self.editor)

    def set_editor_value(self, value: Any) -> None:
        _set_editor_value(self.editor, value)

    def dirty(self) -> bool:
        if self._applied is None:
            return True
        return not _same(self.value(), self._applied)

    def set_applied(self, value: Any) -> None:
        """Record what the car says it is running.

        The editor is only re-seeded when the operator has not touched it. An
        unconditional overwrite would delete an edit in progress the instant a
        params echo arrived for an unrelated parameter.
        """
        first = self._applied is None
        self._applied = value
        self.applied_label.setText(_format_value(self.definition, value))
        if first or not self.dirty():
            blocked = self.editor.blockSignals(True)
            self.set_editor_value(value)
            self.editor.blockSignals(blocked)
        self.refresh_state()

    def mark_pending(self) -> None:
        self._set_state("pending")

    def mark_rejected(self) -> None:
        self._set_state("rejected")

    def clear_applied(self) -> None:
        self._applied = None
        self.applied_label.setText("--")
        self._set_state("unknown")

    def revert(self) -> None:
        """Throw the edit away and show the car's value again."""
        if self._applied is None:
            return
        blocked = self.editor.blockSignals(True)
        self.set_editor_value(self._applied)
        self.editor.blockSignals(blocked)
        self.refresh_state()

    def refresh_state(self) -> None:
        if self._applied is None:
            self._set_state("unknown")
        else:
            self._set_state("dirty" if self.dirty() else "synced")

    def _set_state(self, state: str) -> None:
        self._state = state
        self.dot.set_state(state)

    @property
    def state(self) -> str:
        return self._state

    def set_locked(self, locked: bool) -> None:
        """Disable the editor because the car would refuse the push anyway."""
        if locked == self._locked:
            return
        self._locked = locked
        self.editor.setEnabled(not locked)
        self.name_label.setEnabled(not locked)

    def _on_edit(self) -> None:
        self.refresh_state()
        self.edited.emit(self.name)


class GarageScreen(QWidget):
    """Every tunable the car exposes, grouped, with a live sync indicator."""

    paramsSubmitted = Signal(dict)
    reloadRequested = Signal()
    calibrationRequested = Signal(str, bool)

    def __init__(
        self,
        model: "AppModel",
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
    ) -> None:
        super().__init__(parent)
        self._model = model
        self._theme = theme
        self._rows: dict[str, ParamRow] = {}
        self._pending: set[str] = set()
        self._armed = False
        self._show_advanced = False

        self.setObjectName("Root")
        self._build(theme)

        model.sessionChanged.connect(self._on_session)
        model.vehicleChanged.connect(self._on_vehicle)
        self._on_session(model.session)
        self._on_vehicle(model.vehicle)

    # -- construction -------------------------------------------------------

    def _build(self, theme: Theme) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(
            theme.space.xl, theme.space.xl, theme.space.xl, theme.space.lg
        )
        outer.setSpacing(theme.space.md)

        outer.addWidget(self._build_header(theme))

        self.tabs = QTabWidget(self)
        self.tabs.setDocumentMode(True)
        for name in GROUPS:
            self.tabs.addTab(
                self._build_group(name, theme), _GROUP_TITLES.get(name, name.title())
            )
        outer.addWidget(self.tabs, 1)

        outer.addLayout(self._build_actions(theme))

    def _build_header(self, theme: Theme) -> QWidget:
        card = Card("Vehicle", self, theme=theme, dense=True)
        row = QHBoxLayout()
        row.setSpacing(theme.space.lg)

        self._summary = QLabel("Not connected.", card)
        self._summary.setProperty("variant", "caption")
        self._summary.setWordWrap(True)
        row.addWidget(self._summary, 1)

        self.advanced_toggle = QCheckBox("Show advanced", card)
        self.advanced_toggle.setToolTip(
            "Gains, dead times and jitter deadbands. Hidden by default because "
            "they are only meaningful once the measured values below are right."
        )
        self.advanced_toggle.toggled.connect(self._set_advanced)
        row.addWidget(self.advanced_toggle)
        card.add_layout(row)
        return card

    def _build_group(self, name: str, theme: Theme) -> QWidget:
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)

        page = QWidget(scroll)
        page.setObjectName("Root")
        layout = QVBoxLayout(page)
        layout.setContentsMargins(
            theme.space.lg, theme.space.lg, theme.space.lg, theme.space.lg
        )
        layout.setSpacing(theme.space.sm)

        header = QGridLayout()
        header.setContentsMargins(0, 0, 0, 0)
        legend = QLabel("Parameter", page)
        legend.setProperty("variant", "eyebrow")
        header.addWidget(legend, 0, 0)
        applied = QLabel("On the car", page)
        applied.setProperty("variant", "eyebrow")
        applied.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )
        header.addWidget(applied, 0, 1)
        header.setColumnStretch(0, 1)
        layout.addLayout(header)

        for definition in params_in_group(name):
            row = ParamRow(definition, page, theme=theme)
            row.edited.connect(self._on_row_edited)
            self._rows[definition.name] = row
            layout.addWidget(row)
        layout.addStretch(1)

        scroll.setWidget(page)
        return scroll

    def _build_actions(self, theme: Theme) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(theme.space.sm)

        self._status = QLabel("", self)
        self._status.setProperty("variant", "caption")
        row.addWidget(self._status, 1)

        self.calibrate_button = QPushButton("Run drive calibration…", self)
        self.calibrate_button.setToolTip(_CALIBRATE_TIP)
        self.calibrate_button.clicked.connect(
            lambda: self.calibrationRequested.emit("drive", False)
        )
        row.addWidget(self.calibrate_button)

        self.reload_button = QPushButton("Reload from car", self)
        self.reload_button.clicked.connect(self.reloadRequested.emit)
        row.addWidget(self.reload_button)

        self.revert_button = QPushButton("Revert edits", self)
        self.revert_button.clicked.connect(self.revert)
        row.addWidget(self.revert_button)

        self.push_button = QPushButton("Push changes", self)
        self.push_button.setProperty("variant", "primary")
        self.push_button.clicked.connect(self.push)
        row.addWidget(self.push_button)
        return row

    # -- actions ------------------------------------------------------------

    def changed_values(self) -> dict[str, Any]:
        """Only rows the operator actually moved.

        Pushing the whole set to change one value would also re-push every
        parameter the car had legitimately adjusted for itself, and would make
        a single typo look like forty rejected values.
        """
        return {
            name: row.value()
            for name, row in self._rows.items()
            if row.state == "dirty"
        }

    def push(self) -> None:
        values = self.changed_values()
        if not values:
            self._status.setText("Nothing to push.")
            return
        for name in values:
            self._pending.add(name)
            self._rows[name].mark_pending()
        self._status.setText("Pushed %d value(s); waiting for the car." % (len(values),))
        self.paramsSubmitted.emit(values)

    def revert(self) -> None:
        for row in self._rows.values():
            row.revert()
        self._pending.clear()
        self._status.setText("Reverted to the values the car reports.")

    def _set_advanced(self, show: bool) -> None:
        self._show_advanced = show
        self._apply_visibility()

    def _apply_visibility(self) -> None:
        for row in self._rows.values():
            row.setVisible(self._show_advanced or not row.definition.advanced)

    def _on_row_edited(self, name: str) -> None:
        self._pending.discard(name)
        dirty = sum(1 for row in self._rows.values() if row.state == "dirty")
        self._status.setText(
            "" if not dirty else "%d unsaved change(s)." % (dirty,)
        )

    # -- model --------------------------------------------------------------

    def _on_session(self, session: SessionSnapshot) -> None:
        params: Mapping[str, Any] = session.params
        for name, row in self._rows.items():
            if name in params:
                row.set_applied(params[name])
            elif not session.active:
                row.clear_applied()

        if session.last_error_code in _REJECTION_CODES and self._pending:
            for name in self._pending:
                self._rows[name].mark_rejected()
            self._status.setText(session.last_error or "The car refused a value.")
            self._pending.clear()
        elif session.params_pending == 0 and self._pending:
            self._pending.clear()
            self._status.setText("The car echoed every pushed value.")

        self._summary.setText(_summary_text(session))
        self._apply_visibility()

    def _on_vehicle(self, vehicle: VehicleSnapshot) -> None:
        armed = vehicle.state is VehicleState.ARMED
        if armed == self._armed:
            return
        self._armed = armed
        for row in self._rows.values():
            row.set_locked(armed and row.definition.requires_disarm)
        self.calibrate_button.setEnabled(not armed)
        self.calibrate_button.setToolTip(
            _CALIBRATE_ARMED_TIP if armed else _CALIBRATE_TIP
        )

    def sizeHint(self) -> QSize:
        return QSize(1040, 760)


# --------------------------------------------------------------------------
# Editor construction -- the whole "generic" part
# --------------------------------------------------------------------------


def _decimals_for(definition: ParamDef) -> int:
    step = definition.step
    if step is None or step <= 0.0:
        return 3
    text = ("%.10f" % (step,)).rstrip("0")
    if "." not in text:
        return 0
    fraction = text.split(".", 1)[1]
    return min(6, len(fraction))


def _build_editor(definition: ParamDef, parent: QWidget) -> QWidget:
    kind = definition.kind
    if kind == "bool":
        box = QCheckBox(parent)
        return box
    if kind == "enum":
        combo = QComboBox(parent)
        combo.addItems(list(definition.choices))
        return combo
    if kind == "int":
        spin = QSpinBox(parent)
        spin.setRange(
            int(definition.minimum) if definition.minimum is not None else -1_000_000,
            int(definition.maximum) if definition.maximum is not None else 1_000_000,
        )
        spin.setSingleStep(int(definition.step) if definition.step else 1)
        if definition.unit:
            spin.setSuffix(" " + definition.unit)
        # Wheel scrolling over a spin box inside a scroll area is how somebody
        # changes the encoder count by accident while looking for the video tab.
        spin.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        spin.setKeyboardTracking(False)
        spin.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.UpDownArrows)
        return spin

    spin = QDoubleSpinBox(parent)
    spin.setDecimals(_decimals_for(definition))
    spin.setRange(
        definition.minimum if definition.minimum is not None else -1e9,
        definition.maximum if definition.maximum is not None else 1e9,
    )
    spin.setSingleStep(definition.step if definition.step else 0.01)
    if definition.unit:
        spin.setSuffix(" " + definition.unit)
    spin.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    spin.setKeyboardTracking(False)
    spin.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    spin.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
    return spin


def _connect_editor(editor: QWidget, slot: Any) -> None:
    if isinstance(editor, QCheckBox):
        editor.toggled.connect(slot)
    elif isinstance(editor, QComboBox):
        editor.currentIndexChanged.connect(slot)
    elif isinstance(editor, (QSpinBox, QDoubleSpinBox)):
        editor.valueChanged.connect(slot)


def _editor_value(editor: QWidget) -> Any:
    if isinstance(editor, QCheckBox):
        return editor.isChecked()
    if isinstance(editor, QComboBox):
        return editor.currentText()
    if isinstance(editor, QSpinBox):
        return int(editor.value())
    if isinstance(editor, QDoubleSpinBox):
        return float(editor.value())
    return None


def _set_editor_value(editor: QWidget, value: Any) -> None:
    if isinstance(editor, QCheckBox):
        editor.setChecked(bool(value))
    elif isinstance(editor, QComboBox):
        index = editor.findText(str(value))
        if index >= 0:
            editor.setCurrentIndex(index)
    elif isinstance(editor, QSpinBox):
        if isinstance(value, (int, float)):
            editor.setValue(int(value))
    elif isinstance(editor, QDoubleSpinBox):
        if isinstance(value, (int, float)):
            editor.setValue(float(value))


def _same(a: Any, b: Any) -> bool:
    """Editor value versus car value, compared the way the widget rounds.

    A ``QDoubleSpinBox`` showing three decimals cannot represent 0.0501, so a
    car reporting that would leave the row permanently dirty and the operator
    permanently unable to make the dot go quiet.
    """
    if isinstance(a, bool) or isinstance(b, bool):
        return bool(a) == bool(b)
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= 5e-7 + 5e-4 * abs(float(b))
    return a == b


def _format_value(definition: ParamDef, value: Any) -> str:
    if value is None:
        return "--"
    if definition.kind == "bool":
        return "on" if value else "off"
    if definition.kind == "enum":
        return str(value)
    if definition.kind == "int":
        text = "%d" % (int(value),)
    else:
        text = "%.*f" % (_decimals_for(definition), float(value))
    return text + (" " + definition.unit if definition.unit else "")


def _summary_text(session: SessionSnapshot) -> str:
    if not session.active:
        return "Not connected. These are the registry defaults, not the car's values."
    known = sum(1 for name in PARAMS if name in session.params)
    return "%s · firmware %s · %d of %d parameters reported" % (
        session.car_id or "car",
        session.fw_version or "unknown",
        known,
        len(PARAMS),
    )


__all__ = ["GarageScreen", "ParamRow"]
