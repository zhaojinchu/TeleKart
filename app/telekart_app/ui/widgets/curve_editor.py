"""Editable input-response curve with a live input dot.

Interpolation is monotone cubic (Fritsch-Carlson), not a natural spline, and
that is a safety choice rather than an aesthetic one. A natural spline through
hand-placed control points overshoots between them, so a curve the operator
drew as "gentle to half throttle" can contain a section where a small pedal
movement produces a large output step -- or worse, a non-monotone region where
pressing further gives *less* throttle. Fritsch-Carlson cannot overshoot.

The drawn curve is generated from the same 256-entry lookup table the input
chain uses at 250 Hz, evaluated the same way, so what is on screen is exactly
what the car will be sent. Drawing the ideal spline while the chain interpolates
a table would be a small lie that only shows up as a feel mismatch nobody can
place.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QKeyEvent, QMouseEvent, QPainter, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import (
    PaintedWidget,
    clamp,
    draw_text,
    finite,
    format_fixed,
    stroke_pen,
)

#: Must match the input chain's table size. A 256-entry table quantizes to
#: about 0.4 % of range, which is below the resolution of the wire format's
#: +-1000 quantization anyway.
LUT_SIZE = 256

_HIT_RADIUS = 9.0
_MIN_X_GAP = 0.03


class CurveEditor(PaintedWidget):
    """Drag control points to shape a 0..1 -> 0..1 response."""

    curveChanged = Signal()

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        points: list[tuple[float, float]] | None = None,
        caption: str = "RESPONSE",
        editable: bool = True,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._points: list[tuple[float, float]] = list(
            points if points else [(0.0, 0.0), (0.5, 0.32), (1.0, 1.0)]
        )
        self._caption = caption
        self._editable = editable
        self._deadzone = 0.0
        self._saturation = 1.0

        self._lut: list[float] = [0.0] * LUT_SIZE
        self._curve = QPolygonF()
        self._drag_index = -1
        self._selected = -1
        self._hover = -1
        self._input: float | None = None

        self._plot = QRectF()
        self._caption_rect = QRectF()
        self._readout_rect = QRectF()

        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_readout = fonts.numeric_font(theme.type.micro, theme=theme)
        self._font_axis = fonts.numeric_font(theme.type.micro, theme=theme)

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(QSize(170, 140))
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._rebuild_lut()

    # -- model --------------------------------------------------------------

    def points(self) -> list[tuple[float, float]]:
        return list(self._points)

    def set_points(self, points: list[tuple[float, float]]) -> None:
        """Replace the control points. Endpoints are forced to (0,0) and (1,1)."""
        cleaned = [
            (clamp(finite(x), 0.0, 1.0), clamp(finite(y), 0.0, 1.0)) for x, y in points
        ]
        cleaned.sort(key=lambda p: p[0])
        if len(cleaned) < 2:
            cleaned = [(0.0, 0.0), (1.0, 1.0)]
        cleaned[0] = (0.0, 0.0)
        cleaned[-1] = (1.0, 1.0)
        self._points = _monotonize(cleaned)
        self._rebuild_lut()
        self.update()

    def lut(self) -> list[float]:
        """The evaluated table, for handing straight to the input chain."""
        return list(self._lut)

    def evaluate(self, x: float) -> float:
        """Table lookup with linear interpolation, exactly as the chain does."""
        t = clamp(finite(x), 0.0, 1.0) * (LUT_SIZE - 1)
        i = int(t)
        if i >= LUT_SIZE - 1:
            return self._lut[LUT_SIZE - 1]
        frac = t - i
        return self._lut[i] + (self._lut[i + 1] - self._lut[i]) * frac

    def set_deadzone(self, value: float) -> None:
        """Shade the region the deadzone removes, upstream of this curve."""
        self._deadzone = clamp(finite(value), 0.0, 0.9)
        self.invalidate_static()

    def set_saturation(self, value: float) -> None:
        """Shade the top-end clip, i.e. where full output is reached early."""
        self._saturation = clamp(finite(value), 0.1, 1.0)
        self.invalidate_static()

    def set_input(self, value: float | None) -> None:
        """Live axis position. ``None`` hides the dot."""
        self._input = None if value is None else clamp(finite(value), 0.0, 1.0)
        self.update()

    def set_editable(self, editable: bool) -> None:
        self._editable = editable
        if not editable:
            self._drag_index = -1
            self._selected = -1
        self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(240, 200)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        # Wide enough for a right-aligned "1.0" in the tabular font at every
        # size this widget is used at; at 24 px it clipped to "l.0".
        left = float(theme.space.xxl)
        top = float(theme.space.lg)
        right = float(theme.space.sm)
        bottom = float(theme.space.lg)
        self._plot = QRectF(
            left, top, max(20.0, width - left - right), max(20.0, height - top - bottom)
        )
        self._caption_rect = QRectF(left, 0.0, self._plot.width(), top)
        self._readout_rect = QRectF(
            left, self._plot.bottom(), self._plot.width(), bottom
        )
        self._rebuild_curve()

    def _to_screen(self, x: float, y: float) -> QPointF:
        return QPointF(
            self._plot.left() + self._plot.width() * x,
            self._plot.bottom() - self._plot.height() * y,
        )

    def _to_data(self, pos: QPointF) -> tuple[float, float]:
        if self._plot.width() <= 0.0 or self._plot.height() <= 0.0:
            return 0.0, 0.0
        return (
            clamp((pos.x() - self._plot.left()) / self._plot.width(), 0.0, 1.0),
            clamp((self._plot.bottom() - pos.y()) / self._plot.height(), 0.0, 1.0),
        )

    # -- curve --------------------------------------------------------------

    def _rebuild_lut(self) -> None:
        xs = [p[0] for p in self._points]
        ys = [p[1] for p in self._points]
        slopes = _fritsch_carlson(xs, ys)
        for i in range(LUT_SIZE):
            x = i / (LUT_SIZE - 1)
            self._lut[i] = clamp(_hermite(xs, ys, slopes, x), 0.0, 1.0)
        self._rebuild_curve()

    def _rebuild_curve(self) -> None:
        if self._plot.isEmpty():
            return
        polygon = QPolygonF()
        # One screen pixel per sample is plenty; sampling all 256 entries on a
        # 200 px wide plot is three times the points for no visible gain.
        steps = max(24, int(self._plot.width()))
        for i in range(steps + 1):
            x = i / steps
            polygon.append(self._to_screen(x, self.evaluate(x)))
        self._curve = polygon

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q
        plot = self._plot

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(mix(q.bg_raised, q.bg_base, 0.55))
        painter.drawRoundedRect(plot, theme.radius.control, theme.radius.control)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        if self._deadzone > 0.0:
            painter.setBrush(with_alpha(q.warn, 0.10))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRect(
                QRectF(
                    plot.left(), plot.top(), plot.width() * self._deadzone, plot.height()
                )
            )
        if self._saturation < 1.0:
            painter.setBrush(with_alpha(q.warn, 0.10))
            painter.setPen(Qt.PenStyle.NoPen)
            x = plot.left() + plot.width() * self._saturation
            painter.drawRect(QRectF(x, plot.top(), plot.right() - x, plot.height()))
        painter.setBrush(Qt.BrushStyle.NoBrush)

        grid = stroke_pen(with_alpha(q.line, 0.9), 1.0, round_cap=False)
        for i in range(1, 4):
            frac = i / 4.0
            painter.setPen(grid)
            x = plot.left() + plot.width() * frac
            y = plot.bottom() - plot.height() * frac
            painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()))
            painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y))

        painter.setPen(stroke_pen(with_alpha(q.text_tertiary, 0.4), 1.0, round_cap=False))
        painter.drawLine(
            self._to_screen(0.0, 0.0), self._to_screen(1.0, 1.0)
        )

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
        for frac in (0.0, 0.5, 1.0):
            y = plot.bottom() - plot.height() * frac
            draw_text(
                painter,
                QRectF(0.0, y - 7.0, plot.left() - 4.0, 14.0),
                format_fixed(frac, 1),
                self._font_axis,
                q.text_tertiary,
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q

        if self._curve.count() >= 2:
            painter.setPen(stroke_pen(q.accent, 2.0))
            painter.drawPolyline(self._curve)

        if self._editable:
            for i, (x, y) in enumerate(self._points):
                pos = self._to_screen(x, y)
                endpoint = i == 0 or i == len(self._points) - 1
                radius = 4.0 if endpoint else 5.0
                if i == self._selected:
                    painter.setPen(stroke_pen(q.accent, 1.5))
                    painter.setBrush(q.text_primary)
                elif i == self._hover:
                    painter.setPen(stroke_pen(q.text_primary, 1.5))
                    painter.setBrush(q.bg_base)
                else:
                    painter.setPen(stroke_pen(q.text_secondary, 1.4))
                    painter.setBrush(q.bg_base)
                painter.drawEllipse(pos, radius, radius)
            painter.setBrush(Qt.BrushStyle.NoBrush)

        if self._input is not None:
            out = self.evaluate(self._input)
            pos = self._to_screen(self._input, out)
            painter.setPen(
                stroke_pen(with_alpha(q.cyan, 0.45), 1.0, round_cap=False)
            )
            painter.drawLine(QPointF(self._plot.left(), pos.y()), pos)
            painter.drawLine(QPointF(pos.x(), self._plot.bottom()), pos)
            painter.setPen(stroke_pen(q.bg_base, 1.5))
            painter.setBrush(q.cyan)
            painter.drawEllipse(pos, 4.0, 4.0)
            painter.setBrush(Qt.BrushStyle.NoBrush)

            draw_text(
                painter,
                self._readout_rect,
                format_fixed(self._input, 2) + "  ->  " + format_fixed(out, 2),
                self._font_readout,
                q.cyan,
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )

    # -- interaction --------------------------------------------------------

    def _hit(self, pos: QPointF) -> int:
        for i, (x, y) in enumerate(self._points):
            screen = self._to_screen(x, y)
            dx = screen.x() - pos.x()
            dy = screen.y() - pos.y()
            if dx * dx + dy * dy <= _HIT_RADIUS * _HIT_RADIUS:
                return i
        return -1

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        if not self._editable:
            return
        pos = event.position()
        if self._drag_index >= 0:
            self._move_point(self._drag_index, pos)
            return
        hover = self._hit(pos)
        if hover != self._hover:
            self._hover = hover
            self.setCursor(
                Qt.CursorShape.SizeAllCursor
                if hover > 0 and hover < len(self._points) - 1
                else Qt.CursorShape.ArrowCursor
            )
            self.update()

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if not self._editable:
            return
        if event.button() == Qt.MouseButton.LeftButton:
            index = self._hit(event.position())
            self._selected = index
            self._drag_index = index
            self.update()
        elif event.button() == Qt.MouseButton.RightButton:
            index = self._hit(event.position())
            if 0 < index < len(self._points) - 1:
                del self._points[index]
                self._selected = -1
                self._rebuild_lut()
                self.update()
                self.curveChanged.emit()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        if self._drag_index >= 0:
            self._drag_index = -1
            self.curveChanged.emit()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        if not self._editable or event.button() != Qt.MouseButton.LeftButton:
            return
        if self._hit(event.position()) >= 0:
            return
        x, y = self._to_data(event.position())
        insert_at = len(self._points) - 1
        for i, (px, _) in enumerate(self._points):
            if px > x:
                insert_at = i
                break
        self._points.insert(insert_at, (x, y))
        self._points = _monotonize(self._points)
        self._selected = insert_at
        self._rebuild_lut()
        self.update()
        self.curveChanged.emit()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if not self._editable or self._selected <= 0:
            super().keyPressEvent(event)
            return
        if self._selected >= len(self._points) - 1:
            super().keyPressEvent(event)
            return
        step = 0.01 if event.modifiers() & Qt.KeyboardModifier.ShiftModifier else 0.05
        key = event.key()
        x, y = self._points[self._selected]
        if key == Qt.Key.Key_Left:
            x -= step
        elif key == Qt.Key.Key_Right:
            x += step
        elif key == Qt.Key.Key_Up:
            y += step
        elif key == Qt.Key.Key_Down:
            y -= step
        elif key in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            del self._points[self._selected]
            self._selected = -1
            self._rebuild_lut()
            self.update()
            self.curveChanged.emit()
            return
        else:
            super().keyPressEvent(event)
            return
        self._set_point(self._selected, x, y)
        self.curveChanged.emit()

    def _move_point(self, index: int, pos: QPointF) -> None:
        x, y = self._to_data(pos)
        self._set_point(index, x, y)

    def _set_point(self, index: int, x: float, y: float) -> None:
        """Constrain a point so the control polygon stays strictly increasing.

        Clamping here rather than sorting afterwards keeps the point the user
        is holding under the cursor instead of letting it swap places with its
        neighbour mid-drag.
        """
        last = len(self._points) - 1
        if index <= 0 or index >= last:
            # Endpoints are pinned: a throttle curve that does not pass through
            # (0,0) has a dead or a live pedal at rest, and neither is a curve
            # setting.
            return
        lo_x = self._points[index - 1][0] + _MIN_X_GAP
        hi_x = self._points[index + 1][0] - _MIN_X_GAP
        lo_y = self._points[index - 1][1]
        hi_y = self._points[index + 1][1]
        self._points[index] = (
            clamp(x, lo_x, max(lo_x, hi_x)),
            clamp(y, lo_y, max(lo_y, hi_y)),
        )
        self._rebuild_lut()
        self.update()


# --------------------------------------------------------------------------
# Interpolation
# --------------------------------------------------------------------------


def _monotonize(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Enforce strictly increasing x and non-decreasing y."""
    result: list[tuple[float, float]] = []
    for x, y in sorted(points, key=lambda p: p[0]):
        if result:
            px, py = result[-1]
            if x - px < _MIN_X_GAP * 0.5:
                continue
            if y < py:
                y = py
        result.append((x, y))
    if len(result) < 2:
        return [(0.0, 0.0), (1.0, 1.0)]
    result[0] = (0.0, 0.0)
    result[-1] = (1.0, 1.0)
    return result


def _fritsch_carlson(xs: list[float], ys: list[float]) -> list[float]:
    """Tangents for shape-preserving monotone cubic interpolation."""
    n = len(xs)
    if n < 2:
        return [0.0] * n
    deltas = [
        (ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]) if xs[i + 1] > xs[i] else 0.0
        for i in range(n - 1)
    ]
    slopes = [0.0] * n
    slopes[0] = deltas[0]
    slopes[n - 1] = deltas[n - 2]
    for i in range(1, n - 1):
        if deltas[i - 1] * deltas[i] <= 0.0:
            slopes[i] = 0.0
        else:
            slopes[i] = (deltas[i - 1] + deltas[i]) * 0.5

    for i in range(n - 1):
        if deltas[i] == 0.0:
            slopes[i] = 0.0
            slopes[i + 1] = 0.0
            continue
        a = slopes[i] / deltas[i]
        b = slopes[i + 1] / deltas[i]
        s = a * a + b * b
        if s > 9.0:
            t = 3.0 / (s**0.5)
            slopes[i] = t * a * deltas[i]
            slopes[i + 1] = t * b * deltas[i]
    return slopes


def _hermite(xs: list[float], ys: list[float], slopes: list[float], x: float) -> float:
    n = len(xs)
    if x <= xs[0]:
        return ys[0]
    if x >= xs[n - 1]:
        return ys[n - 1]
    lo = 0
    hi = n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] > x:
            hi = mid
        else:
            lo = mid
    h = xs[lo + 1] - xs[lo]
    if h <= 0.0:
        return ys[lo]
    t = (x - xs[lo]) / h
    t2 = t * t
    t3 = t2 * t
    return (
        (2.0 * t3 - 3.0 * t2 + 1.0) * ys[lo]
        + (t3 - 2.0 * t2 + t) * h * slopes[lo]
        + (-2.0 * t3 + 3.0 * t2) * ys[lo + 1]
        + (t3 - t2) * h * slopes[lo + 1]
    )


__all__ = ["LUT_SIZE", "CurveEditor"]
