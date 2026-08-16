"""Bottom-centre HUD zone: shift lights, speed, pedals, steering.

The five things a driver reads without moving their eyes off the road, in one
block at the bottom of the picture where a real car's instrument binnacle sits.
Everything else on the HUD is a glance; this is peripheral vision.

Two of the values here come from :class:`InputSnapshot` -- the pedal bars and
the steering command -- and that is deliberate. ``InputSnapshot`` carries what
the TX thread actually put on the wire, not what the wheel is doing, so a break
anywhere in the input chain shows up as a pedal bar that stops moving while the
driver's foot is still down. Driving off the raw device would hide exactly the
failure the driver most needs to see.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter, QPainterPath
from PySide6.QtWidgets import QWidget

from ...model.units import UnitFormatter
from ..theme.tokens import THEME, Theme, mix, with_alpha
from ..widgets.base import PaintedWidget, ValueFollower, clamp, finite, stroke_pen
from ..widgets.led_strip import ShiftLights
from . import (
    SMOOTH_TAU_S,
    draw_scrim,
    hero_font,
    label_color,
    label_font,
    micro_label_font,
    readout_font,
)

#: Zone geometry. Fixed rather than elastic: the driver's eye learns where the
#: speed digit sits, and a block that reflows when the window is resized costs
#: them that muscle memory every time.
_PAD = 14.0
_LIGHTS_H = 26.0
_MAIN_H = 78.0
_STEER_H = 40.0
_GAP = 10.0
_BAR_W = 22.0
_WIDTH = 600
_HEIGHT = int(_PAD * 2 + _LIGHTS_H + _GAP + _MAIN_H + _GAP * 0.8 + _STEER_H)

#: Half-angle of the steering arc, degrees. Wider than the car's real lock on
#: purpose: the arc is a *display* range, and a needle that reaches the end stop
#: at full lock leaves no room to show the servo lagging behind the command.
_ARC_HALF_DEG = 62.0


class WheelBar(PaintedWidget):
    """Shift lights, the speed number, throttle/brake and the steering arc."""

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent, theme=theme)

        self._formatter = UnitFormatter()
        self._speed = ValueFollower(0.0, SMOOTH_TAU_S)
        self._throttle = ValueFollower(0.0, SMOOTH_TAU_S)
        self._brake = ValueFollower(0.0, SMOOTH_TAU_S)
        self._steer_cmd = ValueFollower(0.0, SMOOTH_TAU_S)
        self._steer_actual = ValueFollower(0.0, SMOOTH_TAU_S)

        self._lock_deg = 0.0
        self._has_actual = False
        self._transmitting = False
        self._gear = "N"
        self._speed_text = "0"
        self._unit_text = "km/h"

        self._font_hero = hero_font(theme)
        self._font_unit = label_font(theme)
        self._font_label = micro_label_font(theme)
        self._font_gear = readout_font(theme, size=theme.type.heading)

        self._scrim = QRectF()
        self._lights_rect = QRectF()
        self._speed_rect = QRectF()
        self._unit_rect = QRectF()
        self._gear_rect = QRectF()
        self._thr_rect = QRectF()
        self._brk_rect = QRectF()
        self._thr_label = QRectF()
        self._brk_label = QRectF()
        self._arc_centre = QPointF()
        self._arc_radius = 1.0
        self._arc_rect = QRectF()

        self._lights = ShiftLights(
            self, theme=theme, segments=14, show_readout=False, caption=""
        )
        self._lights.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self._lights.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setFixedSize(QSize(_WIDTH, _HEIGHT))

    # -- inputs -------------------------------------------------------------

    def set_formatter(self, formatter: UnitFormatter) -> None:
        self._formatter = formatter
        self._refresh_speed_text()
        self.update()

    def set_speed(self, mps: float) -> None:
        self._speed.set_target(abs(finite(mps)))
        self.request_animation()

    def set_pedals(self, throttle: float, brake: float) -> None:
        self._throttle.set_target(clamp(finite(throttle), 0.0, 1.0))
        self._brake.set_target(clamp(finite(brake), 0.0, 1.0))
        self.request_animation()

    def set_transmitting(self, transmitting: bool) -> None:
        """False greys the pedal bars: the command is not reaching the car."""
        if transmitting != self._transmitting:
            self._transmitting = transmitting
            self.update()

    def set_steering(self, command: float, actual_rad: float, *, has_actual: bool) -> None:
        """``command`` is -1..+1 as sent; ``actual_rad`` is the car's servo angle.

        ``has_actual`` is separate from "actual is zero" because an uncalibrated
        car reports a steering lock of zero, and a needle pinned at centre would
        claim the wheels are straight when the truth is that nobody has measured
        the linkage yet.
        """
        self._steer_cmd.set_target(clamp(finite(command), -1.0, 1.0))
        if has_actual and self._lock_deg > 0.0:
            normalized = math.degrees(finite(actual_rad)) / self._lock_deg
            self._steer_actual.set_target(clamp(normalized, -1.0, 1.0))
        self._has_actual = has_actual and self._lock_deg > 0.0
        self.request_animation()

    def set_lock_deg(self, degrees: float) -> None:
        self._lock_deg = max(0.0, finite(degrees))

    def set_redline_rpm(self, rpm: float) -> None:
        self._lights.set_redline(max(0.0, finite(rpm)))

    def set_rpm(self, rpm: float) -> None:
        self._lights.set_rpm(finite(rpm))

    def set_gear(self, gear: str) -> None:
        """``D`` / ``R`` / ``P`` / ``N``. One character, always the same width."""
        if gear != self._gear:
            self._gear = gear
            self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(_WIDTH, _HEIGHT)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        w = float(width)
        self._scrim = QRectF(0.0, 0.0, w, float(height))

        inner_l = _PAD
        inner_r = w - _PAD
        y = _PAD

        self._lights_rect = QRectF(inner_l, y, inner_r - inner_l, _LIGHTS_H)
        self._lights.setGeometry(self._lights_rect.toRect())
        y += _LIGHTS_H + _GAP

        self._thr_rect = QRectF(inner_l, y, _BAR_W, _MAIN_H - 14.0)
        self._brk_rect = QRectF(inner_r - _BAR_W, y, _BAR_W, _MAIN_H - 14.0)
        self._thr_label = QRectF(inner_l - 6.0, y + _MAIN_H - 14.0, _BAR_W + 12.0, 13.0)
        self._brk_label = QRectF(
            inner_r - _BAR_W - 6.0, y + _MAIN_H - 14.0, _BAR_W + 12.0, 13.0
        )

        centre_l = inner_l + _BAR_W + 18.0
        centre_r = inner_r - _BAR_W - 18.0
        self._speed_rect = QRectF(centre_l, y - 4.0, centre_r - centre_l, _MAIN_H - 20.0)
        self._unit_rect = QRectF(centre_l, y + _MAIN_H - 24.0, centre_r - centre_l, 16.0)
        # The gear chip rides on the speed block's baseline rather than in the
        # status cluster: direction is a driving input, not a status.
        self._gear_rect = QRectF(centre_l, y + _MAIN_H - 26.0, 34.0, 20.0)
        y += _MAIN_H + _GAP * 0.8

        self._arc_rect = QRectF(inner_l, y, inner_r - inner_l, _STEER_H)
        self._arc_radius = _STEER_H * 1.9
        self._arc_centre = QPointF(w * 0.5, y + _STEER_H + self._arc_radius * 0.42)

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        draw_scrim(painter, self._scrim, theme.radius.overlay)

        q = theme.q
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(with_alpha(q.text_primary, 0.10))
        for rect in (self._thr_rect, self._brk_rect):
            painter.drawRoundedRect(rect, 3.0, 3.0)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.setFont(self._font_label)
        painter.setPen(label_color(theme))
        painter.drawText(
            self._thr_label,
            int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter),
            "thr",
        )
        painter.drawText(
            self._brk_label,
            int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter),
            "brk",
        )

        self._paint_arc_track(painter)

    def _paint_arc_track(self, painter: QPainter) -> None:
        q = self.theme.q
        radius = self._arc_radius
        box = QRectF(
            self._arc_centre.x() - radius,
            self._arc_centre.y() - radius,
            radius * 2.0,
            radius * 2.0,
        )
        painter.setPen(stroke_pen(with_alpha(q.text_primary, 0.14), 3.0))
        painter.drawArc(
            box,
            int((90.0 - _ARC_HALF_DEG) * 16.0),
            int(_ARC_HALF_DEG * 2.0 * 16.0),
        )
        # Centre notch. Without it the arc has no zero and the driver cannot
        # tell a small steady offset from straight-ahead.
        painter.setPen(stroke_pen(with_alpha(q.text_primary, 0.45), 2.0))
        top = QPointF(self._arc_centre.x(), self._arc_centre.y() - radius)
        painter.drawLine(
            QPointF(top.x(), top.y() - 5.0), QPointF(top.x(), top.y() + 4.0)
        )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q

        self._paint_bar(painter, self._thr_rect, self._throttle.value, q.accent)
        self._paint_bar(painter, self._brk_rect, self._brake.value, q.bad)

        painter.setFont(self._font_hero)
        painter.setPen(q.text_primary)
        painter.drawText(
            self._speed_rect,
            int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom),
            self._speed_text,
        )

        painter.setFont(self._font_unit)
        painter.setPen(label_color(theme))
        painter.drawText(
            self._unit_rect,
            int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop),
            self._unit_text,
        )

        self._paint_gear(painter)
        self._paint_steering(painter)

    def _paint_bar(
        self, painter: QPainter, rect: QRectF, value: float, color: QColor
    ) -> None:
        if value <= 0.001:
            return
        filled = rect.height() * clamp(value, 0.0, 1.0)
        fill = QRectF(rect.left(), rect.bottom() - filled, rect.width(), filled)
        painter.setPen(Qt.PenStyle.NoPen)
        # A command that is not being transmitted is drawn as a ghost. Showing
        # it at full strength would be the HUD mirroring the driver's foot
        # rather than reporting the car, which is the one thing it must not do.
        painter.setBrush(
            color if self._transmitting else mix(self.theme.q.bg_base, color, 0.45)
        )
        painter.drawRoundedRect(fill, 3.0, 3.0)
        painter.setBrush(Qt.BrushStyle.NoBrush)

    def _paint_gear(self, painter: QPainter) -> None:
        q = self.theme.q
        color = q.text_secondary
        if self._gear == "R":
            color = q.warn
        elif self._gear == "P":
            color = q.cyan
        elif self._gear == "D":
            color = q.text_primary
        painter.setFont(self._font_gear)
        painter.setPen(color)
        painter.drawText(
            self._gear_rect,
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            self._gear,
        )

    def _paint_steering(self, painter: QPainter) -> None:
        q = self.theme.q
        radius = self._arc_radius
        box = QRectF(
            self._arc_centre.x() - radius,
            self._arc_centre.y() - radius,
            radius * 2.0,
            radius * 2.0,
        )

        command = self._steer_cmd.value
        span = -command * _ARC_HALF_DEG
        if abs(span) > 0.2:
            painter.setPen(stroke_pen(q.accent, 4.0))
            painter.drawArc(box, int(90.0 * 16.0), int(span * 16.0))

        if self._has_actual:
            self._paint_arc_marker(
                painter, self._steer_actual.value, q.cyan, 9.0, 2.0
            )
        self._paint_arc_marker(painter, command, q.accent, 13.0, 2.5)

    def _paint_arc_marker(
        self,
        painter: QPainter,
        value: float,
        color: QColor,
        length: float,
        width: float,
    ) -> None:
        angle = math.radians(90.0 - value * _ARC_HALF_DEG)
        cx = self._arc_centre.x()
        cy = self._arc_centre.y()
        outer = self._arc_radius + 3.0
        inner = self._arc_radius - length
        painter.setPen(stroke_pen(color, width))
        painter.drawLine(
            QPointF(cx + math.cos(angle) * inner, cy - math.sin(angle) * inner),
            QPointF(cx + math.cos(angle) * outer, cy - math.sin(angle) * outer),
        )

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        moving = self._speed.advance(dt)
        moving = self._throttle.advance(dt) or moving
        moving = self._brake.advance(dt) or moving
        moving = self._steer_cmd.advance(dt) or moving
        moving = self._steer_actual.advance(dt) or moving
        self._refresh_speed_text()
        return moving

    def _refresh_speed_text(self) -> None:
        value, unit = self._formatter.speed(self._speed.value)
        decimals = 0 if self._formatter.scaled else 1
        text = "%.*f" % (decimals, value if math.isfinite(value) else 0.0)
        if text != self._speed_text:
            self._speed_text = text
        if unit != self._unit_text:
            self._unit_text = unit


def steering_arc_path(centre: QPointF, radius: float) -> QPainterPath:
    """The bare arc, exported so the diagnostic overlay can reuse the geometry."""
    path = QPainterPath()
    box = QRectF(centre.x() - radius, centre.y() - radius, radius * 2.0, radius * 2.0)
    path.arcMoveTo(box, 90.0 - _ARC_HALF_DEG)
    path.arcTo(box, 90.0 - _ARC_HALF_DEG, _ARC_HALF_DEG * 2.0)
    return path


__all__ = ["WheelBar", "steering_arc_path"]
