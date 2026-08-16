"""Circular speed dial.

The full-scale value is **never** hardcoded. It comes from
``TelemetryPacket.v_max_mm_s``, which is the top speed auto-calibration
actually measured on this drivetrain, and the whole dial rescales when it
changes. That is what lets the same build read correctly on today's
L298N-limited car (roughly 0.6-0.7 m/s, because the boost regulator sustains
about 1.5 A for both motors combined) and on a MOSFET bridge later, with no
code change on either side.

If no calibration has been done the gauge says so, in words, rather than
inventing a plausible scale. A speedometer that is confidently wrong is worse
than one that admits it does not know.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, QSize
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QWidget

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
    polar,
    stroke_pen,
    text_height,
    text_width,
)

#: 240 degrees of sweep, opening downward. Wide enough that the cursor moves a
#: useful distance for a small speed change, closed enough that the readout has
#: somewhere to live.
_START_DEG = 210.0
_SPAN_DEG = 240.0

#: Minor ticks between two labelled majors.
_SUBDIVISIONS = 4


class SpeedGauge(PaintedWidget):
    """Progress arc, a cursor on the band, and a large tabular readout."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        unit: str = "m/s",
        unit_scale: float = 1.0,
        decimals: int = 2,
        caption: str = "SPEED",
    ) -> None:
        super().__init__(parent, theme=theme)
        self._unit = unit
        self._unit_scale = unit_scale
        self._decimals = decimals
        self._caption = caption

        self._v_max = 0.0
        #: Dial full scale: ``v_max`` rounded up to a round number, so the tick
        #: labels are 0.2/0.4/0.6 rather than 0.227/0.453/0.680.
        self._scale = 0.0
        self._step = 0.0
        self._divisions = 6
        self._limit = 0.0

        self._speed = ValueFollower(0.0, theme.duration.needle_tau_s)
        self._target = ValueFollower(0.0, theme.duration.needle_tau_s)
        self._target_valid = False
        self._readout = "--"
        self._sub_readout = ""

        self._cx = 0.0
        self._cy = 0.0
        self._radius = 0.0
        self._arc_rect = QRectF()
        self._arc_width = 4.0
        self._readout_rect = QRectF()
        self._unit_rect = QRectF()
        self._caption_rect = QRectF()
        self._sub_rect = QRectF()

        self._font_readout = fonts.numeric_font(theme.type.readout, theme=theme)
        self._font_unit = fonts.ui_font(theme.type.label, theme=theme)
        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_tick = fonts.numeric_font(theme.type.micro, theme=theme)
        self._font_sub = fonts.numeric_font(theme.type.label, theme=theme)

        self.setMinimumSize(QSize(150, 140))

    # -- inputs -------------------------------------------------------------

    def set_speed(self, mps: float) -> None:
        """Current speed, SI.

        Absolute value: the dial is unsigned. Direction is the drivetrain's
        business and is reported by the reverse indicator; a scale that ran
        backwards past zero would halve the resolution of the half that
        matters.
        """
        value = abs(finite(mps))
        self._speed.set_target(value)
        self._refresh_readout(value)
        self.request_animation()

    def set_target_speed(self, mps: float | None) -> None:
        """Commanded speed, drawn as a ghost marker. ``None`` hides it."""
        if mps is None:
            if self._target_valid:
                self._target_valid = False
                self.update()
            return
        self._target_valid = True
        self._target.set_target(abs(finite(mps)))
        self.request_animation()

    def set_v_max(self, mps: float) -> None:
        """Measured full scale, from telemetry. Rebuilds the dial face."""
        value = max(0.0, finite(mps))
        if abs(value - self._v_max) < 1e-6:
            return
        self._v_max = value
        self._scale, self._step, self._divisions = _dial_scale(value)
        self._refresh_readout(self._speed.target)
        self.invalidate_static()
        self.on_layout(self.width(), self.height())

    def set_limit(self, mps: float) -> None:
        """Optional caution band, e.g. a pit limiter or a self-imposed cap."""
        self._limit = max(0.0, finite(mps))
        self.invalidate_static()

    def set_caption(self, text: str) -> None:
        self._caption = text
        self.invalidate_static()

    def set_sub_readout(self, text: str) -> None:
        """Small secondary line under the main number (distance, RPM, ...)."""
        if text != self._sub_readout:
            self._sub_readout = text
            self.update()

    @property
    def calibrated(self) -> bool:
        return self._scale > 0.0

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(240, 220)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.md)
        side = min(float(width), float(height) * 1.12)
        self._radius = max(10.0, side * 0.5 - pad)
        self._cx = width * 0.5
        # The sweep opens downward, so the visual centre of mass sits above the
        # geometric centre; nudging the dial down re-balances it in its box.
        self._cy = height * 0.5 + self._radius * 0.10

        self._arc_width = max(3.0, self._radius * 0.085)
        inset = self._arc_width * 0.5 + 1.0
        r = self._radius - inset
        self._arc_rect = QRectF(self._cx - r, self._cy - r, 2.0 * r, 2.0 * r)

        r = self._radius
        self._font_readout = fonts.numeric_font(
            font_pt(r * 0.23, 12, 96), theme.weight.medium, theme=theme
        )
        self._font_unit = fonts.ui_font(
            font_pt(r * 0.10, 8, 30), theme.weight.medium, theme=theme
        )
        self._font_caption = fonts.ui_font(
            font_pt(r * 0.095, 8, 26), theme.weight.semibold, theme=theme
        )
        self._font_tick = fonts.numeric_font(font_pt(r * 0.095, 8, 24), theme=theme)
        self._font_sub = fonts.numeric_font(font_pt(r * 0.105, 8, 28), theme=theme)

        # The hub stack is deliberately NARROWER than the dial. Full-width text
        # boxes reach x = +-r, which is past the tick label ring, and the
        # caption ends up printed through the scale numbers.
        hub_w = r * 1.45
        cap_w = r * 0.8
        self._readout_rect = QRectF(
            self._cx - hub_w * 0.5, self._cy - r * 0.30, hub_w, r * 0.40
        )
        # Wider than the hub stack: this rect also carries the NOT CALIBRATED
        # notice, and the dial is open at the bottom so nothing collides here.
        self._unit_rect = QRectF(
            self._cx - r * 0.95, self._cy + r * 0.11, r * 1.9, r * 0.16
        )
        self._sub_rect = QRectF(
            self._cx - hub_w * 0.5, self._cy + r * 0.29, hub_w, r * 0.16
        )
        # The caption goes *below* the stack, not above the number. The dial's
        # interior is a ring of scale labels, so any horizontal band across the
        # middle lands on two of them; the open bottom of the sweep is the only
        # clear space there is.
        self._caption_rect = QRectF(
            self._cx - cap_w * 0.5, self._cy + r * 0.46, cap_w, r * 0.15
        )

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q
        calibrated = self.calibrated

        track = mix(q.bg_base, q.line, 0.9) if calibrated else with_alpha(q.line, 0.6)
        painter.setPen(stroke_pen(track, self._arc_width, round_cap=False))
        painter.drawArc(self._arc_rect, int(_START_DEG * 16), int(-_SPAN_DEG * 16))

        if calibrated and 0.0 < self._limit < self._scale:
            frac = clamp(self._limit / self._scale, 0.0, 1.0)
            start = _START_DEG - _SPAN_DEG * frac
            painter.setPen(
                stroke_pen(with_alpha(q.warn, 0.45), self._arc_width, round_cap=False)
            )
            painter.drawArc(
                self._arc_rect, int(start * 16), int(-(_SPAN_DEG * (1.0 - frac)) * 16)
            )

        self._paint_ticks(painter, calibrated)

        draw_text(
            painter, self._caption_rect, self._caption, self._font_caption, q.text_tertiary
        )
        draw_text(
            painter,
            self._unit_rect,
            self._unit if calibrated else "NOT CALIBRATED",
            self._font_unit,
            q.text_secondary if calibrated else q.warn,
        )

    def _paint_ticks(self, painter: QPainter, calibrated: bool) -> None:
        q = self.theme.q
        divisions = self._divisions if calibrated else 6
        r_outer = self._radius - self._arc_width - 2.0
        major_len = self._radius * 0.10
        minor_len = self._radius * 0.055
        label_r = r_outer - major_len - self._radius * 0.02
        label_decimals = 0 if self._step * self._unit_scale >= 1.0 else 1
        th = text_height(self._font_tick)

        minor_pen = stroke_pen(with_alpha(q.text_tertiary, 0.55), 1.0, round_cap=False)
        major_pen = stroke_pen(q.text_secondary, 1.6, round_cap=False)

        total = divisions * _SUBDIVISIONS
        for i in range(total + 1):
            frac = i / total
            deg = _START_DEG - _SPAN_DEG * frac
            is_major = (i % _SUBDIVISIONS) == 0
            painter.setPen(major_pen if is_major else minor_pen)
            x0, y0 = polar(self._cx, self._cy, r_outer, deg)
            x1, y1 = polar(
                self._cx, self._cy, r_outer - (major_len if is_major else minor_len), deg
            )
            painter.drawLine(QPointF(x0, y0), QPointF(x1, y1))

            if not is_major or not calibrated:
                continue
            value = (i // _SUBDIVISIONS) * self._step * self._unit_scale
            label = format_fixed(value, label_decimals)
            # Pull the label in by its own half-extent projected onto the
            # radius. Without this, a label centred on the ring at a shallow
            # angle sticks out through the arc band and gets painted over.
            tw = text_width(self._font_tick, label)
            radians = math.radians(deg)
            inset = 0.5 * tw * abs(math.cos(radians)) + 0.5 * th * abs(
                math.sin(radians)
            )
            lx, ly = polar(self._cx, self._cy, label_r - inset, deg)
            draw_text(
                painter,
                QRectF(lx - 24.0, ly - th * 0.5, 48.0, th),
                label,
                self._font_tick,
                q.text_tertiary,
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        if not self.calibrated:
            draw_text(
                painter, self._readout_rect, "--", self._font_readout, q.text_tertiary
            )
            return

        frac = clamp(self._speed.value / self._scale, 0.0, 1.0)

        if frac > 0.001:
            painter.setPen(stroke_pen(q.accent, self._arc_width, round_cap=False))
            painter.drawArc(
                self._arc_rect, int(_START_DEG * 16), int(-(_SPAN_DEG * frac) * 16)
            )

        if self._target_valid:
            t_frac = clamp(self._target.value / self._scale, 0.0, 1.0)
            self._band_marker(painter, t_frac, q.purple, max(2.0, self._arc_width * 0.4))

        # A cursor across the band rather than a needle through the middle.
        # A full-length needle sweeps straight over the scale numbers and the
        # readout, and at the sizes this widget actually gets laid out at there
        # is no arrangement where it does not collide with one or the other.
        self._band_marker(painter, frac, q.text_primary, max(2.0, self._arc_width * 0.5))

        draw_text(
            painter, self._readout_rect, self._readout, self._font_readout, q.text_primary
        )
        if self._sub_readout:
            draw_text(
                painter, self._sub_rect, self._sub_readout, self._font_sub, q.text_secondary
            )

    def _band_marker(
        self, painter: QPainter, fraction: float, color: QColor, width: float
    ) -> None:
        """A radial line across the arc band at ``fraction`` of full scale."""
        deg = _START_DEG - _SPAN_DEG * clamp(fraction, 0.0, 1.0)
        r_mid = self._arc_rect.width() * 0.5
        r_in = r_mid - self._arc_width * 0.5 - 1.0
        r_out = r_mid + self._arc_width * 0.5 + 1.0
        x0, y0 = polar(self._cx, self._cy, r_in, deg)
        x1, y1 = polar(self._cx, self._cy, r_out, deg)
        painter.setPen(stroke_pen(color, width, round_cap=False))
        painter.drawLine(QPointF(x0, y0), QPointF(x1, y1))

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        moving = self._speed.advance(dt)
        if self._target_valid:
            moving = self._target.advance(dt) or moving
        return moving

    # -- internals ----------------------------------------------------------

    def _refresh_readout(self, value: float) -> None:
        self._readout = (
            format_fixed(value * self._unit_scale, self._decimals)
            if self.calibrated
            else "--"
        )


def _dial_scale(v_max: float) -> tuple[float, float, int]:
    """Round ``v_max`` up to a dial that can be labelled in round numbers.

    Returns ``(full_scale, major_step, divisions)``. Labelling a 0.68 m/s car
    in steps of 0.113333 is technically exact and completely unreadable; the
    needle simply never reaches the last tick, which is what every real gauge
    does anyway.
    """
    if v_max <= 0.0:
        return 0.0, 0.0, 6
    raw = v_max / 6.0
    magnitude = 10.0 ** math.floor(math.log10(raw))
    for multiple in (1.0, 2.0, 2.5, 5.0, 10.0):
        step = multiple * magnitude
        divisions = int(math.ceil(v_max / step - 1e-9))
        if 2 <= divisions <= 10:
            return step * divisions, step, divisions
    step = 10.0 * magnitude
    divisions = max(2, min(10, int(math.ceil(v_max / step - 1e-9))))
    return step * divisions, step, divisions


__all__ = ["SpeedGauge"]
