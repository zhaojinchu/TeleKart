"""Shift lights, driven by measured encoder RPM against a configured redline.

Nothing here invents a scale. The RPM is what the Hall encoders actually
counted -- x2 decoded at 660 counts per output revolution, filtered by the M/T
estimator -- and the redline is a parameter the operator set. On this car the
regulator-limited ceiling is somewhere around 150-200 RPM rather than the
motor's 360 RPM nameplate, so a strip calibrated against the data sheet would
never light up at all.

Colour progression is cyan -> amber -> accent, then a purple flash over the
limit. Green is conspicuously absent: it means *faster* on the delta bar and
must not pick up a second meaning here.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import PaintedWidget, clamp, draw_text, finite, font_pt, stroke_pen


class ShiftLights(PaintedWidget):
    """A progressive LED bar between ``start_fraction`` of redline and redline."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        segments: int = 12,
        redline_rpm: float = 0.0,
        start_fraction: float = 0.55,
        show_readout: bool = True,
        caption: str = "RPM",
    ) -> None:
        super().__init__(parent, theme=theme)
        self._segments = max(3, segments)
        self._redline = max(0.0, finite(redline_rpm))
        self._start = clamp(finite(start_fraction, 0.55), 0.0, 0.95)
        self._show_readout = show_readout
        self._caption = caption

        self._rpm = 0.0
        self._lit = 0
        self._over = False
        self._blink_on = True
        self._blink_accum = 0.0
        self._readout = "0"

        self._rects: list[QRectF] = []
        self._colors: list[QColor] = []
        self._readout_rect = QRectF()
        self._caption_rect = QRectF()
        self._radius = 2.0

        self._font_readout = fonts.numeric_font(
            theme.type.heading, theme.weight.medium, theme=theme
        )
        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumSize(QSize(160, 34))

    # -- inputs -------------------------------------------------------------

    def set_rpm(self, rpm: float) -> None:
        """Measured wheel/motor RPM. Absolute value; reverse lights the same."""
        value = abs(finite(rpm))
        self._rpm = value
        self._readout = "%d" % (int(value),)

        lit = 0
        over = False
        if self._redline > 0.0:
            frac = value / self._redline
            over = frac >= 1.0
            if frac > self._start:
                span = 1.0 - self._start
                lit = int(((frac - self._start) / span) * self._segments + 1e-9) + 1
                lit = min(lit, self._segments)

        changed = lit != self._lit or over != self._over
        self._lit = lit
        self._over = over
        if over:
            self.request_animation()
        elif changed:
            self._blink_on = True
        self.update()

    def set_redline(self, rpm: float) -> None:
        """Configured limit. Zero disables the strip and dims it."""
        value = max(0.0, finite(rpm))
        if abs(value - self._redline) < 1e-6:
            return
        self._redline = value
        self.set_rpm(self._rpm)
        self.invalidate_static()

    def set_start_fraction(self, fraction: float) -> None:
        self._start = clamp(finite(fraction, 0.55), 0.0, 0.95)
        self.set_rpm(self._rpm)

    def set_segments(self, count: int) -> None:
        count = max(3, count)
        if count == self._segments:
            return
        self._segments = count
        self.invalidate_static()
        self.on_layout(self.width(), self.height())
        self.set_rpm(self._rpm)

    @property
    def over_limit(self) -> bool:
        return self._over

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(320, 40)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.xs)
        readout_w = 54.0 if self._show_readout else 0.0
        caption_h = 12.0 if self._caption else 0.0

        strip_w = max(20.0, width - readout_w - 2.0 * pad)
        gap = max(2.0, strip_w * 0.012)
        seg_w = (strip_w - gap * (self._segments - 1)) / self._segments
        seg_h = max(6.0, height - caption_h - 2.0 * pad)
        top = caption_h + pad

        # A shallow rise toward the middle, the way a real wheel's LED bar is
        # curved. Precomputed here so the paint path stays a rect blit.
        rise = min(seg_h * 0.28, 6.0)
        self._rects = []
        for i in range(self._segments):
            t = (2.0 * i / (self._segments - 1)) - 1.0
            dy = rise * (t * t)
            self._rects.append(
                QRectF(pad + i * (seg_w + gap), top + dy, seg_w, seg_h - dy)
            )
        self._radius = min(3.0, seg_w * 0.30)

        self._colors = [self._zone_color(i) for i in range(self._segments)]

        self._readout_rect = QRectF(
            width - readout_w - pad, top, readout_w, seg_h
        )
        self._caption_rect = QRectF(pad, 0.0, strip_w, caption_h)

        self._font_readout = fonts.numeric_font(
            font_pt(seg_h * 0.62, 11, 40), theme.weight.medium, theme=theme
        )
        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )

    def _zone_color(self, index: int) -> QColor:
        q = self.theme.q
        third = self._segments / 3.0
        if index < third:
            return q.cyan
        if index < 2.0 * third:
            return q.warn
        return q.accent

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        armed = self._redline > 0.0
        painter.setPen(Qt.PenStyle.NoPen)
        for i, rect in enumerate(self._rects):
            base = self._colors[i] if armed else q.text_tertiary
            painter.setBrush(mix(q.bg_base, base, 0.16 if armed else 0.08))
            painter.drawRoundedRect(rect, self._radius, self._radius)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        if self._caption:
            draw_text(
                painter,
                self._caption_rect,
                self._caption if armed else self._caption + "  (NO REDLINE SET)",
                self._font_caption,
                q.text_tertiary if armed else q.warn,
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q

        if self._over and self._blink_on:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(q.purple)
            for rect in self._rects:
                painter.drawRoundedRect(rect, self._radius, self._radius)
            painter.setBrush(Qt.BrushStyle.NoBrush)
        elif not self._over and self._lit:
            painter.setPen(Qt.PenStyle.NoPen)
            for i in range(self._lit):
                painter.setBrush(self._colors[i])
                painter.drawRoundedRect(self._rects[i], self._radius, self._radius)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            # A single hairline outline on the leading segment reads as "this
            # is the edge of the band" without adding another colour.
            painter.setPen(
                stroke_pen(with_alpha(q.text_primary, 0.55), 1.0, round_cap=False)
            )
            painter.drawRoundedRect(
                self._rects[self._lit - 1], self._radius, self._radius
            )

        if self._show_readout:
            color = (
                q.purple
                if self._over
                else (q.text_primary if self._lit else q.text_secondary)
            )
            draw_text(
                painter,
                self._readout_rect,
                self._readout,
                self._font_readout,
                color,
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        if not self._over:
            self._blink_on = True
            return False
        self._blink_accum += dt
        period = self.theme.duration.blink_ms / 1000.0
        while self._blink_accum >= period:
            self._blink_accum -= period
            self._blink_on = not self._blink_on
        return True


__all__ = ["ShiftLights"]
