"""Steering-angle arc: commanded input against the angle the servo reached.

Two traces, deliberately. The servo is slew-limited in firmware (its rate limit
also caps the peak current it can pull off the Pi's 5 V rail, which the SoC
shares), so during a fast input the commanded angle and the applied angle
genuinely differ. Showing only one of them makes a correctly-working rate limit
look like input lag.

The scale comes from ``steer_max_deg``, which is a *measured* parameter -- read
off a protractor at full lock during bring-up, not derived from the servo's
data sheet. Feeding a guess into this widget also feeds it into the odometry
model and the electronic differential, so the number matters.
"""

from __future__ import annotations

import math

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
    font_pt,
    format_fixed,
    format_signed,
    polar,
    stroke_pen,
)

#: Mechanical full lock maps onto this many degrees of drawn arc either side of
#: centre. Full lock is typically 24 degrees; drawing it at 1:1 would be a
#: barely-visible wiggle.
_HALF_SPAN_DEG = 56.0

#: Sagitta factor: how much of the radius the drawn arc actually occupies
#: vertically. Used to fit the arc into the widget's height without clipping.
_COS_HALF = math.cos(math.radians(_HALF_SPAN_DEG))


class SteeringIndicator(PaintedWidget):
    """Arc, pointer, centre detent and a signed degree readout."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        lock_deg: float = 24.0,
        caption: str = "STEER",
    ) -> None:
        super().__init__(parent, theme=theme)
        self._lock = max(1.0, abs(finite(lock_deg, 24.0)))
        self._caption = caption

        self._applied = ValueFollower(0.0, theme.duration.needle_tau_s)
        self._command: float | None = None
        self._servo_us: int | None = None
        self._readout = format_signed(0.0, 1) + "°"
        self._sub = ""

        self._cx = 0.0
        self._cy = 0.0
        self._radius = 0.0
        self._arc_rect = QRectF()
        self._arc_width = 5.0
        self._readout_rect = QRectF()
        self._caption_rect = QRectF()
        self._sub_rect = QRectF()

        self._font_readout = fonts.numeric_font(
            theme.type.heading, theme.weight.medium, theme=theme
        )
        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_tick = fonts.numeric_font(theme.type.micro, theme=theme)
        self._font_sub = fonts.numeric_font(theme.type.micro, theme=theme)

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        self.setMinimumSize(QSize(140, 84))

    # -- inputs -------------------------------------------------------------

    def set_angle_deg(self, degrees: float) -> None:
        """Applied steering angle, signed, right positive."""
        value = clamp(finite(degrees), -self._lock * 1.2, self._lock * 1.2)
        self._applied.set_target(value)
        self._readout = format_signed(value, 1) + "°"
        self.request_animation()

    def set_command_deg(self, degrees: float | None) -> None:
        """Commanded angle, drawn as a ghost tick. ``None`` hides it."""
        if degrees is None:
            if self._command is not None:
                self._command = None
                self.update()
            return
        self._command = clamp(finite(degrees), -self._lock * 1.2, self._lock * 1.2)
        self.update()

    def set_normalized(self, value: float) -> None:
        """Convenience for input-side values in -1..+1."""
        self.set_angle_deg(clamp(finite(value), -1.0, 1.0) * self._lock)

    def set_lock_deg(self, degrees: float) -> None:
        value = max(1.0, abs(finite(degrees, self._lock)))
        if abs(value - self._lock) < 1e-6:
            return
        self._lock = value
        self.invalidate_static()

    def set_servo_us(self, pulse_us: int | None) -> None:
        """Raw pulse width, shown small. Invaluable during servo calibration."""
        self._servo_us = pulse_us
        self._sub = "" if pulse_us is None else "%d us" % (pulse_us,)
        self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(220, 120)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.sm)

        self._cx = width * 0.5
        self._cy = height - pad - 12.0
        # 0.36 of the width, not 0.46: the full-lock labels sit at roughly one
        # radius out horizontally, and anything wider pushes them off the edge.
        self._radius = max(
            14.0,
            min(width * 0.36, (height - pad - 22.0) / max(0.15, 1.0 - _COS_HALF)),
        )
        self._arc_width = max(3.0, self._radius * 0.10)

        r = self._radius - self._arc_width * 0.5
        self._arc_rect = QRectF(self._cx - r, self._cy - r, 2.0 * r, 2.0 * r)

        # Text scales off the smaller of radius and height, so a short, wide
        # instrument does not end up with a readout taller than its own arc.
        text_base = min(self._radius, height * 0.9)
        self._font_readout = fonts.numeric_font(
            font_pt(text_base * 0.24, 12, 64), theme.weight.medium, theme=theme
        )
        self._font_caption = fonts.ui_font(
            font_pt(text_base * 0.10, 8, 24), theme.weight.semibold, theme=theme
        )
        self._font_tick = fonts.numeric_font(font_pt(text_base * 0.10, 8, 22), theme=theme)
        self._font_sub = fonts.numeric_font(font_pt(text_base * 0.10, 8, 22), theme=theme)

        rh = text_base * 0.34
        self._readout_rect = QRectF(
            self._cx - self._radius, self._cy - rh - 12.0, 2.0 * self._radius, rh
        )
        self._caption_rect = QRectF(
            self._cx - self._radius,
            self._readout_rect.top() - text_base * 0.20,
            2.0 * self._radius,
            text_base * 0.18,
        )
        self._sub_rect = QRectF(
            self._cx - self._radius, self._cy - 12.0, 2.0 * self._radius, 14.0
        )

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q

        painter.setPen(
            stroke_pen(mix(q.bg_base, q.line, 0.9), self._arc_width, round_cap=False)
        )
        painter.drawArc(
            self._arc_rect,
            int((90.0 - _HALF_SPAN_DEG) * 16),
            int(2.0 * _HALF_SPAN_DEG * 16),
        )

        r_out = self._radius + 1.0
        for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):
            deg = 90.0 - _HALF_SPAN_DEG * frac
            major = frac in (-1.0, 0.0, 1.0)
            length = self._radius * (0.14 if major else 0.09)
            painter.setPen(
                stroke_pen(
                    q.text_secondary if major else with_alpha(q.text_tertiary, 0.6),
                    1.6 if major else 1.0,
                    round_cap=False,
                )
            )
            x0, y0 = polar(self._cx, self._cy, r_out, deg)
            x1, y1 = polar(self._cx, self._cy, r_out + length, deg)
            painter.drawLine(QPointF(x0, y0), QPointF(x1, y1))

            if not major:
                continue
            lx, ly = polar(self._cx, self._cy, r_out + length + self._radius * 0.10, deg)
            box = QRectF(lx - 18.0, ly - 8.0, 36.0, 16.0)
            draw_text(
                painter,
                box,
                format_fixed(abs(frac) * self._lock, 0),
                self._font_tick,
                q.text_tertiary,
            )

        draw_text(
            painter, self._caption_rect, self._caption, self._font_caption, q.text_tertiary
        )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        frac = clamp(self._applied.value / self._lock, -1.2, 1.2)
        deg = 90.0 - _HALF_SPAN_DEG * frac

        if abs(frac) > 0.004:
            painter.setPen(stroke_pen(q.accent, self._arc_width, round_cap=False))
            painter.drawArc(
                self._arc_rect, int(90.0 * 16), int(-(_HALF_SPAN_DEG * frac) * 16)
            )

        if self._command is not None:
            c_frac = clamp(self._command / self._lock, -1.2, 1.2)
            if abs(c_frac - frac) > 0.01:
                self._tick(painter, 90.0 - _HALF_SPAN_DEG * c_frac, q.text_primary, 1.6)

        x, y = polar(self._cx, self._cy, self._radius - self._arc_width * 0.5, deg)
        knob = max(2.5, self._arc_width * 0.62)
        painter.setPen(stroke_pen(q.bg_base, 1.5))
        painter.setBrush(q.text_primary)
        painter.drawEllipse(QPointF(x, y), knob, knob)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        draw_text(
            painter, self._readout_rect, self._readout, self._font_readout, q.text_primary
        )
        if self._sub:
            draw_text(painter, self._sub_rect, self._sub, self._font_sub, q.text_tertiary)

    def _tick(self, painter: QPainter, deg: float, color: QColor, width: float) -> None:
        r_in = self._radius - self._arc_width
        r_out = self._radius + self._arc_width * 0.35
        x0, y0 = polar(self._cx, self._cy, r_in, deg)
        x1, y1 = polar(self._cx, self._cy, r_out, deg)
        painter.setPen(stroke_pen(color, width))
        painter.drawLine(QPointF(x0, y0), QPointF(x1, y1))

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        return self._applied.advance(dt)


__all__ = ["SteeringIndicator"]
