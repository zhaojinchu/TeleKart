"""Throttle / brake / duty bar with a commanded-versus-applied ghost.

The ghost is the point of this widget. The desktop owns *feel* (curves, expo,
deadzone, smoothing) and the firmware owns *protection* (duty slew, combined
current budget, stall detection), so what the driver asked for and what the car
did are genuinely different numbers. Drawing only one of them hides exactly the
disagreement worth seeing -- a throttle bar pinned at 100 % while the car does
60 % is the boost regulator's current budget clamping, and it is invisible if
the HUD only plots one trace.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import (
    PaintedWidget,
    ValueFollower,
    clamp,
    draw_text,
    finite,
    stroke_pen,
)


class BarMeter(PaintedWidget):
    """A single normalized 0..1 channel, drawn as a filled track.

    Vertical by default because that is how a pedal trace reads on a data
    logger, and because two of these side by side is the standard
    throttle/brake pair every driver already knows how to scan.
    """

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        label: str = "",
        accent: QColor | None = None,
        vertical: bool = True,
        bipolar: bool = False,
        show_peak: bool = True,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._label = label
        self._accent = accent if accent is not None else theme.q.cyan
        self._vertical = vertical
        #: Bipolar tracks fill from the middle -- used for signed duty, where
        #: the sign is the direction the H-bridge is driving.
        self._bipolar = bipolar
        self._show_peak = show_peak

        self._value = ValueFollower(0.0, theme.duration.bar_tau_s)
        self._request: float | None = None
        self._peak = 0.0
        self._peak_hold_s = 0.0
        self._readout = "0"

        self._track = QRectF()
        self._caption_rect = QRectF()
        self._readout_rect = QRectF()
        self._radius = 3.0
        self._grid: tuple[float, ...] = ()

        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_readout = fonts.numeric_font(
            theme.type.label, theme.weight.medium, theme=theme
        )

        if vertical:
            self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
            self.setMinimumSize(QSize(44, 90))
        else:
            self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            self.setMinimumSize(QSize(120, 34))

    # -- inputs -------------------------------------------------------------

    def set_value(self, value: float) -> None:
        """Applied value. 0..1, or -1..1 when bipolar."""
        lo = -1.0 if self._bipolar else 0.0
        v = clamp(finite(value), lo, 1.0)
        self._value.set_target(v)
        magnitude = abs(v)
        if magnitude > self._peak:
            self._peak = magnitude
            self._peak_hold_s = self.theme.duration.peak_hold_ms / 1000.0
        text = "%d" % (round(v * 100.0),)
        if text != self._readout:
            self._readout = text
        self.request_animation()

    def set_request(self, value: float | None) -> None:
        """What the driver asked for, drawn as a ghost. ``None`` hides it."""
        if value is None:
            if self._request is not None:
                self._request = None
                self.update()
            return
        lo = -1.0 if self._bipolar else 0.0
        self._request = clamp(finite(value), lo, 1.0)
        self.update()

    def set_label(self, label: str) -> None:
        self._label = label
        self.invalidate_static()

    def set_accent(self, color: QColor) -> None:
        self._accent = color
        self.invalidate_static()

    def reset_peak(self) -> None:
        self._peak = 0.0
        self._peak_hold_s = 0.0
        self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(48, 160) if self._vertical else QSize(200, 38)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.xs)
        caption_h = 14.0 if self._label else 0.0
        readout_h = 16.0

        if self._vertical:
            self._caption_rect = QRectF(0.0, 0.0, width, caption_h)
            self._readout_rect = QRectF(0.0, height - readout_h, width, readout_h)
            top = caption_h + (pad if caption_h else 0.0)
            bottom = height - readout_h - pad
            track_w = min(float(width) - 2.0 * pad, 18.0)
            self._track = QRectF(
                (width - track_w) * 0.5, top, track_w, max(8.0, bottom - top)
            )
            self._radius = track_w * 0.28
        else:
            self._caption_rect = QRectF(0.0, 0.0, 46.0, height)
            self._readout_rect = QRectF(width - 40.0, 0.0, 40.0, height)
            left = (self._caption_rect.width() + pad) if self._label else pad
            right = width - self._readout_rect.width() - pad
            track_h = min(float(height) - 2.0 * pad, 14.0)
            self._track = QRectF(
                left, (height - track_h) * 0.5, max(8.0, right - left), track_h
            )
            self._radius = track_h * 0.28

        self._grid = (0.25, 0.5, 0.75) if not self._bipolar else (0.5,)

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(mix(q.bg_base, q.line, 0.55))
        painter.drawRoundedRect(self._track, self._radius, self._radius)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.setPen(stroke_pen(with_alpha(q.bg_base, 0.75), 1.0, round_cap=False))
        for frac in self._grid:
            if self._vertical:
                y = self._track.bottom() - self._track.height() * frac
                painter.drawLine(
                    QPointF(self._track.left(), y), QPointF(self._track.right(), y)
                )
            else:
                x = self._track.left() + self._track.width() * frac
                painter.drawLine(
                    QPointF(x, self._track.top()), QPointF(x, self._track.bottom())
                )

        if self._label:
            draw_text(
                painter,
                self._caption_rect,
                self._label,
                self._font_caption,
                q.text_tertiary,
                Qt.AlignmentFlag.AlignCenter
                if self._vertical
                else Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        value = self._value.value

        fill = self._fill_rect(value)
        if fill is not None:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(self._accent)
            painter.drawRoundedRect(fill, self._radius, self._radius)
            painter.setBrush(Qt.BrushStyle.NoBrush)

        if self._request is not None and abs(self._request - value) > 0.01:
            self._draw_marker(
                painter, self._request, with_alpha(q.text_primary, 0.85), 1.6
            )

        if self._show_peak and self._peak > 0.02:
            self._draw_marker(painter, self._peak, with_alpha(self._accent, 0.75), 1.4)

        draw_text(
            painter,
            self._readout_rect,
            self._readout,
            self._font_readout,
            q.text_primary if abs(value) > 0.01 else q.text_tertiary,
            Qt.AlignmentFlag.AlignCenter
            if self._vertical
            else Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
        )

    def _fill_rect(self, value: float) -> QRectF | None:
        track = self._track
        if self._bipolar:
            if abs(value) < 0.005:
                return None
            if self._vertical:
                mid = track.center().y()
                extent = track.height() * 0.5 * abs(value)
                return (
                    QRectF(track.left(), mid - extent, track.width(), extent)
                    if value > 0.0
                    else QRectF(track.left(), mid, track.width(), extent)
                )
            mid = track.center().x()
            extent = track.width() * 0.5 * abs(value)
            return (
                QRectF(mid, track.top(), extent, track.height())
                if value > 0.0
                else QRectF(mid - extent, track.top(), extent, track.height())
            )

        if value < 0.005:
            return None
        if self._vertical:
            extent = track.height() * value
            return QRectF(track.left(), track.bottom() - extent, track.width(), extent)
        return QRectF(track.left(), track.top(), track.width() * value, track.height())

    def _draw_marker(
        self, painter: QPainter, value: float, color: QColor, width: float
    ) -> None:
        track = self._track
        overhang = 2.0
        painter.setPen(stroke_pen(color, width, round_cap=False))
        pos = clamp(0.5 + 0.5 * value if self._bipolar else value, 0.0, 1.0)
        if self._vertical:
            y = track.bottom() - track.height() * pos
            painter.drawLine(
                QPointF(track.left() - overhang, y), QPointF(track.right() + overhang, y)
            )
        else:
            x = track.left() + track.width() * pos
            painter.drawLine(
                QPointF(x, track.top() - overhang), QPointF(x, track.bottom() + overhang)
            )

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        moving = self._value.advance(dt)
        if self._peak > 0.0:
            if self._peak_hold_s > 0.0:
                self._peak_hold_s -= dt
                moving = True
            else:
                self._peak -= self.theme.duration.peak_fall_per_s * dt
                if self._peak <= abs(self._value.value):
                    self._peak = abs(self._value.value)
                if self._peak > 0.0:
                    moving = True
        return moving


__all__ = ["BarMeter"]
