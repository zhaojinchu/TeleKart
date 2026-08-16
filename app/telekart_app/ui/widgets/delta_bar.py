"""Centre-zero delta bar: time gained or lost against a reference.

This widget and :class:`~telekart_app.ui.widgets.stat_tile.StatTile`'s delta
field are the only places in the application that use green and red. Everything
else -- link health, arming state, limiter activity, shift lights -- is drawn
from accent, amber, cyan and purple specifically so that green here can mean
exactly one thing: **faster**. A driver reading this bar mid-corner gets a
colour and a direction and needs neither a label nor a legend.

Negative is ahead and extends to the left, which is the convention every timing
screen in motorsport already uses.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import (
    PaintedWidget,
    ValueFollower,
    clamp,
    draw_text,
    finite,
    font_pt,
    format_signed,
    stroke_pen,
)

#: Auto-range ladder, in seconds. Discrete steps rather than a continuous fit,
#: because a scale that slides while you are watching it makes the bar
#: unreadable -- the length stops meaning anything.
_RANGE_STEPS = (0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 30.0)

#: Grow when the value exceeds this fraction of range; shrink only below this
#: fraction of the next step down. The gap is the hysteresis that stops the
#: scale flapping between two steps.
_GROW_AT = 0.92
_SHRINK_AT = 0.55


class DeltaBar(PaintedWidget):
    """Signed bar around a centre zero, with a tabular readout."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        label: str = "DELTA",
        unit: str = "s",
        decimals: int = 3,
        range_s: float = 1.0,
        auto_range: bool = True,
        lower_is_better: bool = True,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._label = label
        self._unit = unit
        self._decimals = decimals
        self._range = _snap_range(range_s)
        self._auto_range = auto_range
        self._lower_is_better = lower_is_better

        self._delta: float | None = None
        self._follow = ValueFollower(0.0, theme.duration.bar_tau_s)
        self._readout = "--"

        self._track = QRectF()
        self._label_rect = QRectF()
        self._readout_rect = QRectF()
        self._tick_rect = QRectF()
        self._radius = 3.0

        self._font_label = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_readout = fonts.numeric_font(
            theme.type.readout, theme.weight.medium, theme=theme
        )
        self._font_tick = fonts.numeric_font(theme.type.micro, theme=theme)

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumSize(QSize(200, 56))

    # -- inputs -------------------------------------------------------------

    def set_delta(self, delta: float | None) -> None:
        """Seconds against the reference. ``None`` means no reference yet."""
        if delta is None:
            self._delta = None
            self._readout = "--"
            self._follow.set_target(0.0)
            self.request_animation()
            self.update()
            return

        value = finite(delta)
        self._delta = value
        self._readout = format_signed(value, self._decimals)
        if self._auto_range:
            self._retune_range(abs(value))
        self._follow.set_target(value)
        self.request_animation()

    def set_range(self, range_s: float) -> None:
        """Fix the full-scale value and stop auto-ranging."""
        self._auto_range = False
        new = _snap_range(abs(finite(range_s, 1.0)))
        if new != self._range:
            self._range = new
            self.invalidate_static()

    def set_auto_range(self, enabled: bool) -> None:
        self._auto_range = enabled
        if enabled and self._delta is not None:
            self._retune_range(abs(self._delta))

    def set_label(self, label: str) -> None:
        self._label = label
        self.invalidate_static()

    def _retune_range(self, magnitude: float) -> None:
        target = self._range
        while magnitude > target * _GROW_AT and target < _RANGE_STEPS[-1]:
            target = _next_range(target, 1)
        while target > _RANGE_STEPS[0]:
            smaller = _next_range(target, -1)
            if magnitude < smaller * _SHRINK_AT:
                target = smaller
            else:
                break
        if target != self._range:
            self._range = target
            self.invalidate_static()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(340, 62)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.md)
        label_h = 13.0
        track_h = 9.0
        tick_h = 12.0
        # The readout takes whatever is left after the fixed rows. Deriving it
        # the other way round pushed the scale labels off the bottom edge as
        # soon as this widget was stacked with anything else in a card.
        readout_h = max(16.0, height - label_h - track_h - tick_h - 7.0)

        self._label_rect = QRectF(pad, 2.0, width * 0.5, label_h)
        self._readout_rect = QRectF(pad, label_h + 1.0, width - 2.0 * pad, readout_h)
        track_top = self._readout_rect.bottom() + 3.0
        self._track = QRectF(pad, track_top, width - 2.0 * pad, track_h)
        self._tick_rect = QRectF(pad, self._track.bottom() + 1.0, width - 2.0 * pad, tick_h)
        self._radius = track_h * 0.35

        self._font_readout = fonts.numeric_font(
            font_pt(readout_h * 0.78, 14, 40), theme.weight.medium, theme=theme
        )

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(mix(q.bg_base, q.line, 0.55))
        painter.drawRoundedRect(self._track, self._radius, self._radius)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        cx = self._track.center().x()
        painter.setPen(stroke_pen(q.text_secondary, 1.4, round_cap=False))
        painter.drawLine(
            QPointF(cx, self._track.top() - 2.0), QPointF(cx, self._track.bottom() + 2.0)
        )

        painter.setPen(stroke_pen(with_alpha(q.text_tertiary, 0.55), 1.0, round_cap=False))
        for frac in (-1.0, -0.5, 0.5, 1.0):
            x = cx + self._track.width() * 0.5 * frac
            painter.drawLine(
                QPointF(x, self._track.top()), QPointF(x, self._track.bottom())
            )

        draw_text(
            painter,
            self._label_rect,
            self._label,
            self._font_label,
            q.text_tertiary,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )
        for frac in (-1.0, 0.0, 1.0):
            x = cx + self._track.width() * 0.5 * frac
            draw_text(
                painter,
                QRectF(x - 30.0, self._tick_rect.top(), 60.0, self._tick_rect.height()),
                format_signed(frac * self._range, 1) if frac else "0",
                self._font_tick,
                q.text_tertiary,
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        cx = self._track.center().x()
        half = self._track.width() * 0.5

        if self._delta is None:
            draw_text(
                painter,
                self._readout_rect,
                "--",
                self._font_readout,
                q.text_tertiary,
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )
            return

        value = self._follow.value
        frac = clamp(value / self._range, -1.0, 1.0)
        color = self.theme.delta_color(value, lower_is_better=self._lower_is_better)

        extent = abs(frac) * half
        if extent > 0.5:
            rect = (
                QRectF(cx, self._track.top(), extent, self._track.height())
                if frac > 0.0
                else QRectF(cx - extent, self._track.top(), extent, self._track.height())
            )
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(color)
            painter.drawRoundedRect(rect, self._radius, self._radius)
            painter.setBrush(Qt.BrushStyle.NoBrush)

        if abs(frac) >= 1.0:
            # Pinned: say so with an arrowhead rather than letting the bar look
            # like a value that happens to equal full scale.
            tip_x = cx + half + 5.0 if frac > 0.0 else cx - half - 5.0
            painter.setPen(stroke_pen(color, 1.6))
            painter.drawLine(
                QPointF(tip_x, self._track.top()),
                QPointF(tip_x, self._track.bottom()),
            )

        draw_text(
            painter,
            self._readout_rect,
            self._readout + " " + self._unit,
            self._font_readout,
            color,
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
        )

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        return self._follow.advance(dt)


def _snap_range(value: float) -> float:
    for step in _RANGE_STEPS:
        if value <= step:
            return step
    return _RANGE_STEPS[-1]


def _next_range(current: float, direction: int) -> float:
    try:
        index = _RANGE_STEPS.index(current)
    except ValueError:
        return _snap_range(current)
    index = max(0, min(len(_RANGE_STEPS) - 1, index + direction))
    return _RANGE_STEPS[index]


__all__ = ["DeltaBar"]
