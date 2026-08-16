"""A labelled number, optionally with a unit, a delta and a trend trace.

The delta is the only thing in this widget -- and, by convention, one of only
two places in the whole application -- allowed to use green and red. It routes
through ``Theme.delta_color`` rather than picking a colour itself, so "green
means faster" stays mechanically true instead of merely documented. Everything
else on the tile is neutral, accent, or the caller's explicit state colour.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, with_alpha
from .base import (
    PaintedWidget,
    clamp,
    draw_text,
    finite,
    font_pt,
    format_fixed,
    format_signed,
    stroke_pen,
    text_width,
)

_TREND_SAMPLES = 60
_TREND_MAX_HEIGHT = 34.0


class StatTile(PaintedWidget):
    """One measurement, sized to be scanned rather than read."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        label: str = "",
        unit: str = "",
        decimals: int = 1,
        flat: bool = False,
        show_trend: bool = False,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._label = label
        self._unit = unit
        self._decimals = decimals
        #: Flat tiles skip their own surface, for use inside an existing Card.
        self._flat = flat
        self._show_trend = show_trend

        self._value_text = "--"
        self._value_w = 0.0
        self._value_color: QColor | None = None
        self._delta_text = ""
        self._delta_color: QColor | None = None
        self._history: list[float] = []
        self._trend = QPolygonF()

        self._surface = QRectF()
        self._label_rect = QRectF()
        self._value_rect = QRectF()
        self._unit_rect = QRectF()
        self._delta_rect = QRectF()
        self._trend_rect = QRectF()

        self._font_label = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_value = fonts.numeric_font(
            theme.type.readout, theme.weight.medium, theme=theme
        )
        self._font_unit = fonts.ui_font(theme.type.label, theme=theme)
        self._font_delta = fonts.numeric_font(
            theme.type.label, theme.weight.medium, theme=theme
        )

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.setMinimumSize(QSize(96, 62))

    # -- inputs -------------------------------------------------------------

    def set_label(self, label: str) -> None:
        self._label = label
        self.invalidate_static()

    def set_unit(self, unit: str) -> None:
        self._unit = unit
        self.invalidate_static()
        self.on_layout(self.width(), self.height())

    def set_value(self, value: float, decimals: int | None = None) -> None:
        """Set from a number. Non-finite input shows ``--`` rather than ``nan``."""
        if decimals is not None:
            self._decimals = decimals
        self._set_value_text(format_fixed(value, self._decimals))
        if self._show_trend:
            self._history.append(finite(value))
            if len(self._history) > _TREND_SAMPLES:
                del self._history[: len(self._history) - _TREND_SAMPLES]
            self._rebuild_trend()
        self.update()

    def set_text(self, text: str) -> None:
        """Set a pre-formatted value, e.g. a lap time or a state name."""
        self._set_value_text(text)
        self.update()

    def _set_value_text(self, text: str) -> None:
        """Cache the rendered width so the unit can sit against the number.

        Measured on assignment rather than in the paint path: the value changes
        at telemetry rate, the widget repaints at frame rate, and a font metric
        lookup is not free.
        """
        self._value_text = text
        self._value_w = text_width(self._font_value, text)

    def set_value_color(self, color: QColor | None) -> None:
        """Tint the number. Used for state tiles; never for a delta."""
        self._value_color = color
        self.update()

    def set_delta(
        self,
        delta: float | None,
        *,
        lower_is_better: bool = True,
        decimals: int = 2,
        unit: str = "",
    ) -> None:
        """Set the comparison figure. ``None`` clears it."""
        if delta is None:
            self._delta_text = ""
            self._delta_color = None
        else:
            value = finite(delta)
            self._delta_text = format_signed(value, decimals) + unit
            self._delta_color = self.theme.delta_color(
                value, lower_is_better=lower_is_better
            )
        self.update()

    def set_history(self, samples: list[float]) -> None:
        self._history = [finite(v) for v in samples[-_TREND_SAMPLES:]]
        self._rebuild_trend()
        self.update()

    def clear(self) -> None:
        self._value_text = "--"
        self._delta_text = ""
        self._delta_color = None
        self._history.clear()
        self._trend = QPolygonF()
        self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(150, 78)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.md)
        self._surface = QRectF(0.5, 0.5, width - 1.0, height - 1.0)

        inner_w = width - 2.0 * pad
        label_h = 13.0
        self._label_rect = QRectF(pad, float(theme.space.sm), inner_w, label_h)

        value_top = self._label_rect.bottom() + 2.0
        value_h = max(18.0, height - value_top - theme.space.sm)
        value_pt = font_pt(min(value_h * 0.62, width * 0.22), 13, 48)
        self._font_value = fonts.numeric_font(
            value_pt, theme.weight.medium, theme=theme
        )

        unit_w = text_width(self._font_unit, self._unit) + 6.0 if self._unit else 0.0
        # Reserved unconditionally: a delta appearing later must not reflow the
        # value, because a number that shifts sideways when it updates is
        # exactly what the tabular font is there to prevent.
        delta_w = 62.0
        trend_w = min(inner_w * 0.34, 74.0) if self._show_trend else 0.0

        self._value_rect = QRectF(
            pad, value_top, max(24.0, inner_w - unit_w - trend_w), value_h
        )
        # The unit is positioned against the rendered number in paint_dynamic;
        # only its size is fixed here.
        self._unit_rect = QRectF(0.0, 0.0, unit_w, value_h * 0.5)
        self._delta_rect = QRectF(
            width - pad - delta_w, float(theme.space.sm), delta_w, label_h
        )
        # Height-capped for the same reason as the link panel's sparkline: a
        # trend line is a small multiple, and a full-height one on a stretched
        # tile is both wrong-looking and expensive to stroke.
        trend_h = min(_TREND_MAX_HEIGHT, value_h * 0.60)
        self._trend_rect = QRectF(
            width - pad - trend_w,
            value_top + (value_h - trend_h) * 0.5,
            trend_w,
            trend_h,
        )
        self._value_w = text_width(self._font_value, self._value_text)
        self._rebuild_trend()

    def _rebuild_trend(self) -> None:
        if not self._show_trend or self._trend_rect.isEmpty():
            return
        count = len(self._history)
        if count < 2:
            self._trend = QPolygonF()
            return
        lo = min(self._history)
        hi = max(self._history)
        span = hi - lo
        if span < 1e-9:
            span = 1.0
            lo -= 0.5
        rect = self._trend_rect
        step = rect.width() / (count - 1)
        points = QPolygonF()
        for i, value in enumerate(self._history):
            frac = clamp((value - lo) / span, 0.0, 1.0)
            points.append(
                QPointF(rect.left() + i * step, rect.bottom() - rect.height() * frac)
            )
        self._trend = points

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q
        if not self._flat:
            painter.setPen(stroke_pen(q.line, 1.0, round_cap=False))
            painter.setBrush(q.bg_raised)
            painter.drawRoundedRect(
                self._surface, theme.radius.card, theme.radius.card
            )
            painter.setBrush(Qt.BrushStyle.NoBrush)

        if self._label:
            draw_text(
                painter,
                self._label_rect,
                self._label,
                self._font_label,
                q.text_tertiary,
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        draw_text(
            painter,
            self._value_rect,
            self._value_text,
            self._font_value,
            self._value_color if self._value_color is not None else q.text_primary,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )

        if self._unit:
            # Sits on the number's baseline, hard against it. A unit parked at
            # the far edge of the tile reads as a separate field.
            painter.setFont(self._font_unit)
            painter.setPen(q.text_tertiary)
            painter.drawText(
                QRectF(
                    self._value_rect.left() + self._value_w + 4.0,
                    self._value_rect.center().y() - 1.0,
                    self._unit_rect.width(),
                    self._unit_rect.height(),
                ),
                int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                self._unit,
            )

        if self._delta_text:
            draw_text(
                painter,
                self._delta_rect,
                self._delta_text,
                self._font_delta,
                self._delta_color if self._delta_color is not None else q.text_secondary,
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )

        if self._show_trend and self._trend.count() >= 2:
            painter.setPen(stroke_pen(with_alpha(q.cyan, 0.85), 1.3))
            painter.drawPolyline(self._trend)


__all__ = ["StatTile"]
