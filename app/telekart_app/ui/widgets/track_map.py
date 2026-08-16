"""Dead-reckoned minimap: trail, pose, grid and scale bar.

What this draws is odometry, not ground truth. With no IMU the heading drifts,
and a 5 m square closes with something like 5-15 % error, so the map is a
navigation aid and a wheelspin/calibration signal -- not a position reference.
The grid and the scale bar are there precisely so that drift is legible rather
than hidden by an auto-fit that silently rescales.

The trail is stored in world coordinates and drawn through a ``QTransform``
with a cosmetic pen. Appending a pose is then O(1) and zooming costs nothing,
where re-projecting every point into screen space on each new sample would be
O(n) at telemetry rate for the whole session.

The deferred racing layer extends this through ``set_reference_line``,
``set_sector_marks`` and ``set_markers`` rather than by subclassing, so a lap
overlay does not have to know anything about how the base map fits its view.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QPainter, QPen, QPolygonF, QTransform
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import PaintedWidget, draw_text, finite, format_fixed, stroke_pen

#: Poses retained. At 50 Hz telemetry decimated to ~10 Hz of stored points that
#: is roughly ten minutes of driving, which is longer than the battery lasts.
_TRAIL_CAP = 6000
_TRIM_CHUNK = 1200

#: Minimum half-span of the view, so a stationary car does not get an
#: infinitely zoomed map of its own noise.
_MIN_HALF_SPAN_M = 0.75

#: Above this the grid stops being a grid and starts being a fill.
_MAX_GRID_LINES = 120


@dataclass(frozen=True, slots=True)
class MapMarker:
    x: float
    y: float
    label: str = ""
    color_key: str = "purple"


class TrackMap(PaintedWidget):
    """Top-down minimap of the odometry pose."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        caption: str = "TRACK",
        follow: bool = False,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._caption = caption
        self._follow = follow

        self._pts: list[tuple[float, float]] = []
        self._trail = QPolygonF()
        self._reference = QPolygonF()
        self._reference_color_key = "purple"
        self._sector_marks: list[tuple[float, float, float]] = []
        self._markers: list[MapMarker] = []

        self._pose = (0.0, 0.0, 0.0)
        self._has_pose = False

        self._min_x = -1.0
        self._max_x = 1.0
        self._min_y = -1.0
        self._max_y = 1.0

        self._plot = QRectF()
        self._xform = QTransform()
        self._grid_step = 1.0
        self._m_per_px = 0.01
        self._caption_rect = QRectF()
        self._scale_rect = QRectF()

        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_scale = fonts.numeric_font(theme.type.micro, theme=theme)

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(QSize(140, 120))

    # -- data ---------------------------------------------------------------

    def append_pose(self, x: float, y: float, heading: float) -> None:
        """Add a trail point and move the car. Metres and radians."""
        px = finite(x)
        py = finite(y)
        self._pose = (px, py, finite(heading))
        self._has_pose = True

        self._pts.append((px, py))
        self._trail.append(QPointF(px, py))
        if len(self._pts) > _TRAIL_CAP + _TRIM_CHUNK:
            del self._pts[:_TRIM_CHUNK]
            self._rebuild_trail()

        grew = self._grow_bounds(px, py)
        if grew:
            self._refit()
        self.update()

    def set_pose(self, x: float, y: float, heading: float) -> None:
        """Move the car without extending the trail."""
        self._pose = (finite(x), finite(y), finite(heading))
        self._has_pose = True
        if self._follow:
            self._refit()
        self.update()

    def clear_trail(self) -> None:
        self._pts.clear()
        self._trail = QPolygonF()
        self._min_x = self._min_y = -1.0
        self._max_x = self._max_y = 1.0
        self._refit()
        self.update()

    def set_trail(self, points: list[tuple[float, float]]) -> None:
        self._pts = [(finite(x), finite(y)) for x, y in points[-_TRAIL_CAP:]]
        self._rebuild_trail()
        self._recompute_bounds()
        self._refit()
        self.update()

    def set_reference_line(
        self, points: list[tuple[float, float]], color_key: str = "purple"
    ) -> None:
        """A racing line, a previous lap, or a planned path."""
        polygon = QPolygonF()
        for x, y in points:
            polygon.append(QPointF(finite(x), finite(y)))
        self._reference = polygon
        self._reference_color_key = color_key
        self.update()

    def set_sector_marks(self, marks: list[tuple[float, float, float]]) -> None:
        """``(x, y, heading)`` split points, drawn as gates across the track."""
        self._sector_marks = [(finite(x), finite(y), finite(h)) for x, y, h in marks]
        self.update()

    def set_markers(self, markers: list[MapMarker]) -> None:
        self._markers = list(markers)
        self.update()

    def set_follow(self, follow: bool) -> None:
        """Follow mode keeps the car centred instead of fitting the whole trail."""
        self._follow = follow
        self._refit()
        self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(240, 220)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.sm)
        top = float(theme.space.lg)
        self._plot = QRectF(
            pad, top, max(20.0, width - 2.0 * pad), max(20.0, height - top - pad)
        )
        self._caption_rect = QRectF(pad, 0.0, self._plot.width(), top)
        self._scale_rect = QRectF(
            self._plot.left() + 4.0, self._plot.bottom() - 18.0, 110.0, 14.0
        )
        self._refit()

    def _grow_bounds(self, x: float, y: float) -> bool:
        grew = False
        if x < self._min_x:
            self._min_x, grew = x, True
        if x > self._max_x:
            self._max_x, grew = x, True
        if y < self._min_y:
            self._min_y, grew = y, True
        if y > self._max_y:
            self._max_y, grew = y, True
        return grew or self._follow

    def _recompute_bounds(self) -> None:
        if not self._pts:
            self._min_x = self._min_y = -1.0
            self._max_x = self._max_y = 1.0
            return
        xs = [p[0] for p in self._pts]
        ys = [p[1] for p in self._pts]
        self._min_x, self._max_x = min(xs), max(xs)
        self._min_y, self._max_y = min(ys), max(ys)

    def _rebuild_trail(self) -> None:
        polygon = QPolygonF()
        for x, y in self._pts:
            polygon.append(QPointF(x, y))
        self._trail = polygon

    def _refit(self) -> None:
        """Rebuild the world->screen transform, invalidating the grid if it moved.

        The grid is part of the cached background, so it is only re-rendered
        when the view actually changes -- which, once a lap is established, is
        almost never.
        """
        if self._plot.isEmpty():
            return

        if self._follow and self._has_pose:
            cx, cy = self._pose[0], self._pose[1]
            half = max(_MIN_HALF_SPAN_M, self._m_per_px * self._plot.width() * 0.5)
        else:
            cx = (self._min_x + self._max_x) * 0.5
            cy = (self._min_y + self._max_y) * 0.5
            half = max(
                _MIN_HALF_SPAN_M,
                (self._max_x - self._min_x) * 0.5,
                (self._max_y - self._min_y) * 0.5,
            ) * 1.12

        scale = min(self._plot.width(), self._plot.height()) / (2.0 * half)

        xform = QTransform()
        xform.translate(self._plot.center().x(), self._plot.center().y())
        # World y points north; screen y points down. The flip lives here and
        # nowhere else, so no drawing code has to remember it.
        xform.scale(scale, -scale)
        xform.translate(-cx, -cy)

        step = _nice_step(2.0 * half / 5.0)
        m_per_px = 1.0 / scale if scale > 0.0 else 0.01

        if (
            abs(m_per_px - self._m_per_px) > self._m_per_px * 0.02
            or step != self._grid_step
            or self._xform.dx() != xform.dx()
            or self._xform.dy() != xform.dy()
        ):
            self.invalidate_static()

        self._xform = xform
        self._grid_step = step
        self._m_per_px = m_per_px

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q
        plot = self._plot

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(mix(q.bg_raised, q.bg_base, 0.55))
        painter.drawRoundedRect(plot, theme.radius.control, theme.radius.control)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.save()
        painter.setClipRect(plot)
        self._paint_grid(painter)
        painter.restore()

        painter.setPen(stroke_pen(q.line, 1.0, round_cap=False))
        painter.drawRoundedRect(plot, theme.radius.control, theme.radius.control)

        draw_text(
            painter,
            self._caption_rect,
            self._caption,
            self._font_caption,
            q.text_tertiary,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )
        self._paint_scale_bar(painter)

    def _paint_grid(self, painter: QPainter) -> None:
        q = self.theme.q
        step = self._grid_step
        if step <= 0.0:
            return
        inverse, ok = self._xform.inverted()
        if not ok:
            return

        tl = inverse.map(self._plot.topLeft())
        br = inverse.map(self._plot.bottomRight())
        x0 = math.floor(min(tl.x(), br.x()) / step) * step
        x1 = math.ceil(max(tl.x(), br.x()) / step) * step
        y0 = math.floor(min(tl.y(), br.y()) / step) * step
        y1 = math.ceil(max(tl.y(), br.y()) / step) * step

        minor = stroke_pen(with_alpha(q.line, 0.85), 1.0, round_cap=False)
        minor.setCosmetic(True)
        axis = stroke_pen(with_alpha(q.text_tertiary, 0.55), 1.0, round_cap=False)
        axis.setCosmetic(True)

        painter.save()
        painter.setTransform(self._xform, True)
        steps_x = int(round((x1 - x0) / step))
        steps_y = int(round((y1 - y0) / step))
        # A degenerate transform -- an odometry glitch putting the pose a
        # thousand kilometres away -- could ask for millions of lines. Bail out
        # rather than freeze the UI thread; 120 gridlines across is already far
        # denser than anything readable.
        if steps_x > _MAX_GRID_LINES or steps_y > _MAX_GRID_LINES:
            painter.restore()
            return
        for i in range(steps_x + 1):
            x = x0 + i * step
            painter.setPen(axis if abs(x) < step * 0.01 else minor)
            painter.drawLine(QPointF(x, y0), QPointF(x, y1))
        for i in range(steps_y + 1):
            y = y0 + i * step
            painter.setPen(axis if abs(y) < step * 0.01 else minor)
            painter.drawLine(QPointF(x0, y), QPointF(x1, y))
        painter.restore()

    def _paint_scale_bar(self, painter: QPainter) -> None:
        q = self.theme.q
        if self._m_per_px <= 0.0:
            return
        length_px = self._grid_step / self._m_per_px
        if length_px < 8.0 or length_px > self._plot.width():
            return
        y = self._scale_rect.center().y()
        x0 = self._scale_rect.left()
        painter.setPen(stroke_pen(q.text_tertiary, 1.2, round_cap=False))
        painter.drawLine(QPointF(x0, y), QPointF(x0 + length_px, y))
        painter.drawLine(QPointF(x0, y - 3.0), QPointF(x0, y + 3.0))
        painter.drawLine(
            QPointF(x0 + length_px, y - 3.0), QPointF(x0 + length_px, y + 3.0)
        )
        draw_text(
            painter,
            QRectF(x0 + length_px + 5.0, y - 8.0, 60.0, 16.0),
            format_fixed(self._grid_step, 1 if self._grid_step < 1.0 else 0) + " m",
            self._font_scale,
            q.text_tertiary,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        painter.save()
        painter.setClipRect(self._plot)

        painter.save()
        painter.setTransform(self._xform, True)

        if self._reference.count() >= 2:
            pen = QPen(with_alpha(self.theme.rgb(self._reference_color_key), 0.85))
            pen.setWidthF(1.6)
            pen.setCosmetic(True)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            painter.setPen(pen)
            painter.drawPolyline(self._reference)

        if self._trail.count() >= 2:
            pen = QPen(q.cyan)
            pen.setWidthF(1.8)
            # Cosmetic, or the scale in the transform would make the trail
            # hairline-thin when zoomed out and slab-thick when zoomed in.
            pen.setCosmetic(True)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            painter.setPen(pen)
            painter.drawPolyline(self._trail)

        for x, y, heading in self._sector_marks:
            pen = QPen(with_alpha(q.warn, 0.9))
            pen.setWidthF(1.6)
            pen.setCosmetic(True)
            painter.setPen(pen)
            half = self._m_per_px * 9.0
            dx = -math.sin(heading) * half
            dy = math.cos(heading) * half
            painter.drawLine(QPointF(x - dx, y - dy), QPointF(x + dx, y + dy))

        painter.restore()

        for marker in self._markers:
            pos = self._xform.map(QPointF(marker.x, marker.y))
            color = self.theme.rgb(marker.color_key)
            painter.setPen(stroke_pen(q.bg_base, 1.4))
            painter.setBrush(color)
            painter.drawEllipse(pos, 3.5, 3.5)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            if marker.label:
                draw_text(
                    painter,
                    QRectF(pos.x() + 6.0, pos.y() - 8.0, 70.0, 16.0),
                    marker.label,
                    self._font_scale,
                    color,
                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
                )

        if self._has_pose:
            self._paint_car(painter)

        painter.restore()

    def _paint_car(self, painter: QPainter) -> None:
        q = self.theme.q
        x, y, heading = self._pose
        pos = self._xform.map(QPointF(x, y))
        painter.save()
        painter.translate(pos)
        # Screen y is down, so a positive (counter-clockwise) world heading is
        # a negative rotation here.
        painter.rotate(-math.degrees(heading))
        arrow = QPolygonF(
            [QPointF(8.0, 0.0), QPointF(-5.0, 5.0), QPointF(-2.5, 0.0), QPointF(-5.0, -5.0)]
        )
        painter.setPen(stroke_pen(q.bg_base, 1.2))
        painter.setBrush(q.accent)
        painter.drawPolygon(arrow)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.restore()


def _nice_step(raw: float) -> float:
    """1/2/5 x 10^n grid spacing, so the scale bar reads as a round number."""
    if raw <= 0.0:
        return 1.0
    magnitude = 10.0 ** math.floor(math.log10(raw))
    for multiple in (1.0, 2.0, 5.0):
        if raw <= multiple * magnitude:
            return multiple * magnitude
    return 10.0 * magnitude


__all__ = ["MapMarker", "TrackMap"]
