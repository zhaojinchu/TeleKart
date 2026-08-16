"""Raw axis against processed output, for the input calibration screen.

Calibration goes wrong quietly. A pedal whose rest position drifts, a wheel
whose captured range is 15 % short because the operator did not sweep it fully,
an inverted axis that only shows up as "steering feels backwards at speed" --
all of these are obvious the moment the raw hardware value and the chain's
output are drawn on the same widget, and nearly invisible when only one of them
is on screen.

The top track is what the driver's hardware reports, in SDL's native -1..+1,
with the captured range and the deadzone drawn on it. The bottom track is what
the chain produces after calibration, deadzone, saturation, curve, assist,
rate limit and smoothing. Watching the gap between them is the whole point.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import (
    PaintedWidget,
    clamp,
    draw_text,
    finite,
    format_fixed,
    format_signed,
    stroke_pen,
)


class AxisBar(PaintedWidget):
    """Two stacked tracks: hardware value on top, chain output below."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        label: str = "AXIS",
        bipolar: bool = True,
        show_output: bool = True,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._label = label
        #: Steering is bipolar and fills from the centre; pedals are unipolar
        #: and fill from the left. The mapping of the *raw* track is always
        #: -1..+1 because that is what SDL reports for both.
        self._bipolar = bipolar
        self._show_output = show_output

        self._raw = 0.0
        self._processed = 0.0
        self._captured: tuple[float, float] | None = None
        self._deadzone = 0.0
        self._saturation = 1.0
        self._inverted = False

        self._label_rect = QRectF()
        self._raw_track = QRectF()
        self._out_track = QRectF()
        self._raw_label_rect = QRectF()
        self._out_label_rect = QRectF()
        self._raw_value_rect = QRectF()
        self._out_value_rect = QRectF()
        self._radius = 3.0

        self._font_label = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_tag = fonts.ui_font(theme.type.micro, theme=theme)
        self._font_value = fonts.numeric_font(
            theme.type.label, theme.weight.medium, theme=theme
        )

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumSize(QSize(220, 54))

    # -- inputs -------------------------------------------------------------

    def set_raw(self, value: float) -> None:
        """Hardware axis value, SDL's native -1..+1."""
        self._raw = clamp(finite(value), -1.0, 1.0)
        self.update()

    def set_processed(self, value: float) -> None:
        """Chain output: -1..+1 when bipolar, 0..1 otherwise."""
        lo = -1.0 if self._bipolar else 0.0
        self._processed = clamp(finite(value), lo, 1.0)
        self.update()

    def set_captured_range(self, lo: float | None, hi: float | None) -> None:
        """Min/max seen during a calibration sweep. ``None`` clears the markers."""
        if lo is None or hi is None:
            self._captured = None
        else:
            self._captured = (
                clamp(finite(lo), -1.0, 1.0),
                clamp(finite(hi), -1.0, 1.0),
            )
        self.update()

    def set_deadzone(self, value: float) -> None:
        self._deadzone = clamp(finite(value), 0.0, 0.9)
        self.invalidate_static()

    def set_saturation(self, value: float) -> None:
        self._saturation = clamp(finite(value), 0.1, 1.0)
        self.invalidate_static()

    def set_inverted(self, inverted: bool) -> None:
        """Only affects the label; the caller has already inverted the value."""
        self._inverted = inverted
        self.invalidate_static()

    def set_label(self, label: str) -> None:
        self._label = label
        self.invalidate_static()

    def set_bipolar(self, bipolar: bool) -> None:
        if bipolar == self._bipolar:
            return
        self._bipolar = bipolar
        self.invalidate_static()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(320, 58)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.sm)
        tag_w = 34.0
        value_w = 54.0
        label_h = 13.0

        self._label_rect = QRectF(pad, 1.0, width - 2.0 * pad, label_h)

        rows = 2 if self._show_output else 1
        avail = height - label_h - 2.0 * pad
        row_h = avail / rows
        track_h = min(11.0, max(6.0, row_h * 0.62))

        left = pad + tag_w
        track_w = max(20.0, width - left - value_w - pad)

        top = label_h + pad
        self._raw_track = QRectF(
            left, top + (row_h - track_h) * 0.5, track_w, track_h
        )
        self._raw_label_rect = QRectF(pad, top, tag_w, row_h)
        self._raw_value_rect = QRectF(left + track_w + 4.0, top, value_w - 4.0, row_h)

        if self._show_output:
            top2 = top + row_h
            self._out_track = QRectF(
                left, top2 + (row_h - track_h) * 0.5, track_w, track_h
            )
            self._out_label_rect = QRectF(pad, top2, tag_w, row_h)
            self._out_value_rect = QRectF(
                left + track_w + 4.0, top2, value_w - 4.0, row_h
            )

        self._radius = track_h * 0.35

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q

        text = self._label + ("  (INVERTED)" if self._inverted else "")
        draw_text(
            painter,
            self._label_rect,
            text,
            self._font_label,
            q.warn if self._inverted else q.text_tertiary,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(mix(q.bg_base, q.line, 0.55))
        painter.drawRoundedRect(self._raw_track, self._radius, self._radius)
        if self._show_output:
            painter.drawRoundedRect(self._out_track, self._radius, self._radius)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        self._paint_bands(painter)

        painter.setPen(stroke_pen(with_alpha(q.text_tertiary, 0.7), 1.0, round_cap=False))
        if self._bipolar:
            cx = self._raw_track.center().x()
            painter.drawLine(
                QPointF(cx, self._raw_track.top() - 1.0),
                QPointF(cx, self._raw_track.bottom() + 1.0),
            )
            if self._show_output:
                painter.drawLine(
                    QPointF(cx, self._out_track.top() - 1.0),
                    QPointF(cx, self._out_track.bottom() + 1.0),
                )

        draw_text(
            painter,
            self._raw_label_rect,
            "RAW",
            self._font_tag,
            q.text_tertiary,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )
        if self._show_output:
            draw_text(
                painter,
                self._out_label_rect,
                "OUT",
                self._font_tag,
                q.text_tertiary,
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            )

    def _paint_bands(self, painter: QPainter) -> None:
        """Deadzone and saturation, shaded on the raw track."""
        q = self.theme.q
        track = self._raw_track
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(with_alpha(q.warn, 0.13))

        if self._deadzone > 0.0:
            if self._bipolar:
                half = track.width() * 0.5
                w = half * self._deadzone
                painter.drawRect(
                    QRectF(track.center().x() - w, track.top(), 2.0 * w, track.height())
                )
            else:
                painter.drawRect(
                    QRectF(
                        track.left(),
                        track.top(),
                        track.width() * self._deadzone,
                        track.height(),
                    )
                )

        if self._saturation < 1.0:
            if self._bipolar:
                half = track.width() * 0.5
                edge = half * self._saturation
                painter.drawRect(
                    QRectF(track.left(), track.top(), half - edge, track.height())
                )
                painter.drawRect(
                    QRectF(
                        track.center().x() + edge,
                        track.top(),
                        half - edge,
                        track.height(),
                    )
                )
            else:
                x = track.left() + track.width() * self._saturation
                painter.drawRect(
                    QRectF(x, track.top(), track.right() - x, track.height())
                )
        painter.setBrush(Qt.BrushStyle.NoBrush)

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q

        self._fill(painter, self._raw_track, self._raw, True, q.text_secondary)
        draw_text(
            painter,
            self._raw_value_rect,
            format_signed(self._raw, 2),
            self._font_value,
            q.text_secondary,
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
        )

        if self._captured is not None:
            lo, hi = self._captured
            painter.setPen(stroke_pen(q.purple, 1.6, round_cap=False))
            for value in (lo, hi):
                x = self._raw_track.left() + self._raw_track.width() * (
                    0.5 + 0.5 * clamp(value, -1.0, 1.0)
                )
                painter.drawLine(
                    QPointF(x, self._raw_track.top() - 2.5),
                    QPointF(x, self._raw_track.bottom() + 2.5),
                )

        if self._show_output:
            self._fill(painter, self._out_track, self._processed, self._bipolar, q.accent)
            draw_text(
                painter,
                self._out_value_rect,
                format_signed(self._processed, 2)
                if self._bipolar
                else format_fixed(self._processed, 2),
                self._font_value,
                q.text_primary,
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )

    def _fill(
        self,
        painter: QPainter,
        track: QRectF,
        value: float,
        bipolar: bool,
        color: QColor,
    ) -> None:
        if track.isEmpty():
            return
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(color)
        if bipolar:
            cx = track.center().x()
            extent = track.width() * 0.5 * abs(value)
            if extent > 0.4:
                rect = (
                    QRectF(cx, track.top(), extent, track.height())
                    if value > 0.0
                    else QRectF(cx - extent, track.top(), extent, track.height())
                )
                painter.drawRoundedRect(rect, self._radius, self._radius)
        else:
            extent = track.width() * clamp(value, 0.0, 1.0)
            if extent > 0.4:
                painter.drawRoundedRect(
                    QRectF(track.left(), track.top(), extent, track.height()),
                    self._radius,
                    self._radius,
                )
        painter.setBrush(Qt.BrushStyle.NoBrush)


__all__ = ["AxisBar"]
