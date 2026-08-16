"""Raw telemetry, rolling plots, and the log -- the screen you open when
something is wrong.

Everything here is deliberately unglamorous. The driving HUD's job is to be
readable at a glance while moving; this screen's job is to be *complete*, so
that a question like "did the limiter engage before or after the RPM collapsed"
has an answer without re-running the drive.

The table and the plots read the same three snapshots the HUD does, and the
table's rows are literally the HUD diagnostic overlay's rows -- imported, not
re-derived. Two lists of diagnostic fields would drift within a week, and the
first symptom would be a value visible in one place and missing from the other
precisely when someone needed it.

Plot buffers are fixed-size numpy rings. Telemetry arrives at 50 Hz and a
session lasts tens of minutes; an unbounded list would be a slow, invisible leak
that only shows up on the long runs where the plots matter most.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QObject, QSize, Qt, QTimer, Signal
from PySide6.QtGui import QHideEvent, QShowEvent
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ...core.log import ROOT_LOGGER_NAME
from ...model.snapshots import InputSnapshot, LinkSnapshot, VehicleSnapshot
from ..hud.diag_overlay import drive_rows, input_rows, link_rows
from ..theme import fonts
from ..theme.tokens import THEME, Theme
from ..widgets.card import Card

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...model.app_model import AppModel

#: Samples kept per trace. 1500 at 50 Hz is thirty seconds, which is about as
#: far back as anyone reads a live plot; the session recorder owns the archive.
_HISTORY = 1500

#: Redraw rate. Twenty is smooth enough for a scrolling trace and a fifth of the
#: pyqtgraph work of sixty -- and this screen is often open *because* the
#: machine is already struggling.
_PLOT_MS = 50

#: Table refresh. Sixty rows of text five times a second is legible; faster is
#: unreadable and costs sixty QTableWidgetItem updates for nothing.
_TABLE_MS = 200

_LOG_LEVELS = ("DEBUG", "INFO", "WARNING", "ERROR")

_configured = False


def _configure_pyqtgraph(theme: Theme) -> None:
    """Match pyqtgraph to the application palette. Idempotent.

    pyqtgraph reads these as globals at widget-construction time, so they have
    to be set before the first PlotWidget exists and there is no per-widget
    override for the background.
    """
    global _configured
    if _configured:
        return
    pg.setConfigOption("background", theme.color.bg_raised)
    pg.setConfigOption("foreground", theme.color.text_secondary)
    pg.setConfigOptions(antialias=True, useOpenGL=False)
    _configured = True


class _Ring:
    """Fixed-size float ring. Allocation happens once, at construction."""

    __slots__ = ("_count", "_data", "_index", "_size")

    def __init__(self, size: int) -> None:
        self._data = np.zeros(size, dtype=np.float64)
        self._size = size
        self._index = 0
        self._count = 0

    def add(self, value: float) -> None:
        self._data[self._index] = value
        self._index = (self._index + 1) % self._size
        if self._count < self._size:
            self._count += 1

    def clear(self) -> None:
        self._count = 0
        self._index = 0

    def view(self) -> np.ndarray:
        """Oldest-first. Copies only when the ring has wrapped."""
        if self._count < self._size:
            return self._data[: self._count]
        return np.concatenate((self._data[self._index :], self._data[: self._index]))

    def __len__(self) -> int:
        return self._count


class _LogBridge(QObject, logging.Handler):
    """Moves log records onto the GUI thread.

    A logging handler runs on whichever thread called ``log()`` -- the video
    decode thread, the session thread, the TX thread -- and none of them may
    touch a widget. Emitting a Qt signal to a GUI-thread receiver is
    auto-queued, which is the one sanctioned crossing.
    """

    recordLogged = Signal(int, str)

    def __init__(self, parent: QObject | None = None) -> None:
        QObject.__init__(self, parent)
        logging.Handler.__init__(self)
        self.setFormatter(
            logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s",
                              datefmt="%H:%M:%S")
        )

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self.recordLogged.emit(record.levelno, self.format(record))
        except (RuntimeError, ValueError):
            # The widget went away between the log call and delivery. Losing a
            # line during teardown is not worth taking the process with it.
            pass


class DiagnosticsScreen(QWidget):
    """Telemetry table, three rolling plots, and the live log."""

    def __init__(
        self,
        model: "AppModel",
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
    ) -> None:
        super().__init__(parent)
        _configure_pyqtgraph(theme)
        self._model = model
        self._theme = theme

        self._vehicle = model.vehicle
        self._link = model.link
        self._input = model.input
        self._t0 = time.perf_counter()

        self._t_drive = _Ring(_HISTORY)
        self._t_link = _Ring(_HISTORY)
        self._traces: dict[str, _Ring] = {
            name: _Ring(_HISTORY)
            for name in (
                "rpm_l",
                "rpm_r",
                "rpm_tl",
                "rpm_tr",
                "duty_l",
                "duty_r",
                "rtt",
                "loss",
            )
        }

        # Before the widgets: the log pane's level selector configures the
        # bridge as it is built.
        self._bridge = _LogBridge(self)
        self._bridge.setLevel(logging.INFO)
        self._bridge.recordLogged.connect(self._on_record)

        self.setObjectName("Root")
        self._build(theme)

        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(_PLOT_MS)
        self._plot_timer.timeout.connect(self._redraw)
        self._table_timer = QTimer(self)
        self._table_timer.setInterval(_TABLE_MS)
        self._table_timer.timeout.connect(self._refresh_table)

        # Both roots. ``core.log`` sets ``propagate = False`` on the telekart
        # logger so its records never reach the interpreter root, and a console
        # attached only to the interpreter root would therefore show zeroconf
        # and PyAV chatter and none of this application's own output.
        for logger in _log_roots():
            logger.addHandler(self._bridge)

        model.vehicleChanged.connect(self._on_vehicle)
        model.linkChanged.connect(self._on_link)
        model.inputChanged.connect(self._on_input)

    # -- construction -------------------------------------------------------

    def _build(self, theme: Theme) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(
            theme.space.lg, theme.space.lg, theme.space.lg, theme.space.lg
        )
        outer.setSpacing(theme.space.md)

        vertical = QSplitter(Qt.Orientation.Vertical, self)
        top = QSplitter(Qt.Orientation.Horizontal, vertical)

        top.addWidget(self._build_table(theme))
        top.addWidget(self._build_plots(theme))
        top.setStretchFactor(0, 2)
        top.setStretchFactor(1, 5)

        vertical.addWidget(top)
        vertical.addWidget(self._build_log(theme))
        vertical.setStretchFactor(0, 3)
        vertical.setStretchFactor(1, 1)
        outer.addWidget(vertical)

    def _build_table(self, theme: Theme) -> Card:
        card = Card("Telemetry", self, theme=theme, dense=True,
                    subtitle="Every field, as received")
        self.table = QTableWidget(0, 2, card)
        self.table.setHorizontalHeaderLabels(["Field", "Value"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setShowGrid(False)
        self.table.setAlternatingRowColors(True)
        self.table.setFont(fonts.mono_font(theme.type.micro, theme=theme))
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        card.add_widget(self.table, 1)
        return card

    def _build_plots(self, theme: Theme) -> Card:
        card = Card("Live traces", self, theme=theme, dense=True,
                    subtitle="Last %d s · cyan is measured, purple is target"
                             % (_HISTORY // 50,))

        self._plots = pg.GraphicsLayoutWidget(card)
        self._plots.setMinimumHeight(320)
        card.add_widget(self._plots, 1)

        self._curves: dict[str, pg.PlotDataItem] = {}
        speed = self._add_plot("Wheel speed", "RPM", row=0)
        self._curves["rpm_l"] = _curve(speed, theme.color.cyan, 2.0)
        self._curves["rpm_r"] = _curve(speed, theme.color.purple, 2.0)
        self._curves["rpm_tl"] = _curve(speed, theme.color.cyan, 1.0, dashed=True)
        self._curves["rpm_tr"] = _curve(speed, theme.color.purple, 1.0, dashed=True)

        duty = self._add_plot("Bridge duty", "%", row=1, link_to=speed)
        self._curves["duty_l"] = _curve(duty, theme.color.cyan, 2.0)
        self._curves["duty_r"] = _curve(duty, theme.color.purple, 2.0)

        link = self._add_plot("Link", "ms / %", row=2, link_to=speed)
        self._curves["rtt"] = _curve(link, theme.color.accent, 2.0)
        self._curves["loss"] = _curve(link, theme.color.warn, 1.5)
        return card

    def _add_plot(
        self, title: str, units: str, *, row: int, link_to: object | None = None
    ) -> pg.PlotItem:
        plot = self._plots.addPlot(row=row, col=0, title=title)
        plot.showGrid(x=False, y=True, alpha=0.16)
        plot.setLabel("left", units)
        plot.setMouseEnabled(x=False, y=False)
        plot.hideButtons()
        plot.setMenuEnabled(False)
        # Peak-mode downsampling plus view clipping. Fifteen hundred samples
        # across roughly six hundred pixels means pyqtgraph would otherwise
        # stroke two and a half segments per pixel per curve, six curves deep --
        # and peak mode keeps the spikes, which are the only part of a link
        # trace anybody is looking for.
        plot.setDownsampling(auto=True, mode="peak")
        plot.setClipToView(True)
        if link_to is not None:
            # One shared x range so a spike lines up vertically across all
            # three. Reading cause and effect off plots with independent axes
            # is guesswork.
            plot.setXLink(link_to)
        return plot

    def _build_log(self, theme: Theme) -> Card:
        card = Card("Log", self, theme=theme, dense=True)

        controls = QHBoxLayout()
        controls.setSpacing(theme.space.sm)

        controls.addWidget(QLabel("Level", card))
        self.level_box = QComboBox(card)
        self.level_box.addItems(list(_LOG_LEVELS))
        self.level_box.setCurrentText("INFO")
        self.level_box.currentTextChanged.connect(self._set_level)
        controls.addWidget(self.level_box)

        self.follow = QCheckBox("Follow", card)
        self.follow.setChecked(True)
        controls.addWidget(self.follow)
        controls.addStretch(1)

        clear = QPushButton("Clear", card)
        clear.setProperty("variant", "quiet")
        clear.clicked.connect(lambda: self.console.clear())
        controls.addWidget(clear)
        card.add_layout(controls)

        self.console = QPlainTextEdit(card)
        self.console.setReadOnly(True)
        self.console.setUndoRedoEnabled(False)
        # Bounded on purpose: an unbounded console is an unbounded allocation
        # in the one place people leave open for an hour.
        self.console.setMaximumBlockCount(2000)
        self.console.setFont(fonts.mono_font(theme.type.micro, theme=theme))
        self.console.setMinimumHeight(120)
        card.add_widget(self.console, 1)

        self._set_level("INFO")
        return card

    # -- lifecycle ----------------------------------------------------------

    def showEvent(self, event: QShowEvent) -> None:
        self._plot_timer.start()
        self._table_timer.start()
        self._refresh_table()
        super().showEvent(event)

    def hideEvent(self, event: QHideEvent) -> None:
        self._plot_timer.stop()
        self._table_timer.stop()
        super().hideEvent(event)

    def shutdown(self) -> None:
        """Detach the log handler before the widget dies."""
        for logger in _log_roots():
            logger.removeHandler(self._bridge)

    # -- model --------------------------------------------------------------

    def _on_vehicle(self, v: VehicleSnapshot) -> None:
        self._vehicle = v
        now = time.perf_counter() - self._t0
        self._t_drive.add(now)
        self._traces["rpm_l"].add(v.rpm_l)
        self._traces["rpm_r"].add(v.rpm_r)
        self._traces["rpm_tl"].add(v.rpm_target_l)
        self._traces["rpm_tr"].add(v.rpm_target_r)
        self._traces["duty_l"].add(v.duty_l * 100.0)
        self._traces["duty_r"].add(v.duty_r * 100.0)

    def _on_link(self, link: LinkSnapshot) -> None:
        self._link = link
        self._t_link.add(time.perf_counter() - self._t0)
        self._traces["rtt"].add(link.rtt * 1000.0)
        self._traces["loss"].add(link.loss * 100.0)

    def _on_input(self, sample: InputSnapshot) -> None:
        self._input = sample

    # -- rendering ----------------------------------------------------------

    def _redraw(self) -> None:
        drive_t = self._t_drive.view()
        link_t = self._t_link.view()
        for name in ("rpm_l", "rpm_r", "rpm_tl", "rpm_tr", "duty_l", "duty_r"):
            self._curves[name].setData(drive_t, self._traces[name].view())
        for name in ("rtt", "loss"):
            self._curves[name].setData(link_t, self._traces[name].view())

    def _refresh_table(self) -> None:
        rows: list[tuple[str, str]] = []
        rows.append(("— vehicle —", ""))
        rows.extend(drive_rows(self._vehicle))
        rows.append(("— link —", ""))
        rows.extend(link_rows(self._link))
        rows.append(("— input —", ""))
        rows.extend(input_rows(self._input))

        if self.table.rowCount() != len(rows):
            self.table.setRowCount(len(rows))
        for index, (key, value) in enumerate(rows):
            for column, text in ((0, key), (1, value)):
                item = self.table.item(index, column)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(index, column, item)
                if item.text() != text:
                    item.setText(text)

    def _set_level(self, name: str) -> None:
        self._bridge.setLevel(getattr(logging, name, logging.INFO))

    def _on_record(self, level: int, text: str) -> None:
        self.console.appendPlainText(text)
        if self.follow.isChecked():
            bar = self.console.verticalScrollBar()
            bar.setValue(bar.maximum())

    def sizeHint(self) -> QSize:
        return QSize(1180, 800)


def _log_roots() -> tuple[logging.Logger, logging.Logger]:
    return logging.getLogger(ROOT_LOGGER_NAME), logging.getLogger()


def _curve(
    plot: pg.PlotItem, color: str, width: float, *, dashed: bool = False
) -> pg.PlotDataItem:
    pen = pg.mkPen(color=color, width=width)
    if dashed:
        pen.setStyle(Qt.PenStyle.DashLine)
    item = plot.plot(pen=pen)
    # Per-curve rather than only on the PlotItem: the plot-level setting is
    # applied when a curve is added, and relying on that ordering is the kind
    # of thing that silently stops working across a pyqtgraph release.
    item.setDownsampling(auto=True, method="peak")
    item.setClipToView(True)
    return item


__all__ = ["DiagnosticsScreen"]
