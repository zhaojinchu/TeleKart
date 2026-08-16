"""Shared machinery for the custom-painted instruments.

Every instrument in this package splits its drawing in two:

*static*
    Dial faces, tick marks, scale labels, grid lines, bezels. These change only
    when the widget is resized or the theme changes, so they are rendered once
    into a ``QPixmap`` and blitted. Re-stroking sixty tick marks and their text
    on every frame, across a dozen instruments, at 60 Hz, is how a HUD turns
    into a space heater -- and on a laptop that is also decoding H.264 in
    another thread, that heat comes straight out of the video frame rate.

*dynamic*
    Needles, bars, dots, readouts. Painted on top every frame.

The other thing this module owns is smoothing. Telemetry arrives at 50 Hz and
the UI paints at 60 Hz, so a needle driven straight from the last packet
visibly stutters. :class:`ValueFollower` is a first-order lag with a
theme-defined time constant, advanced by real elapsed time so it behaves the
same whether the app is painting at 60 Hz or struggling at 20. Its timer stops
itself once the value settles, so a parked car costs nothing.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QElapsedTimer, QEvent, QRectF, QSize, Qt, QTimer
from PySide6.QtGui import (
    QColor,
    QFont,
    QFontMetricsF,
    QPainter,
    QPaintEvent,
    QPen,
    QPixmap,
    QResizeEvent,
)
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme.tokens import THEME, Theme

#: Animation tick. Faster than this buys nothing on a 60 Hz panel and costs a
#: wakeup; slower makes needle motion visibly steppy.
_ANIM_INTERVAL_MS = 16


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def finite(value: float, fallback: float = 0.0) -> float:
    """NaN and inf are display bugs, not exceptions.

    A divide-by-zero upstream must pin a needle, not raise inside a paint
    event: Qt swallows the traceback, the widget stops repainting, and the
    driver is left looking at a frozen instrument that still looks plausible.
    """
    return value if math.isfinite(value) else fallback


class ValueFollower:
    """First-order lag toward a target, advanced by wall-clock dt.

    ``tau`` is the time to reach ~63 % of a step. Instruments use the theme's
    ``needle_tau_s`` or ``bar_tau_s`` so that everything on screen settles with
    the same character; two gauges lagging differently reads as one of them
    being broken.
    """

    __slots__ = ("target", "value", "tau", "epsilon")

    def __init__(self, initial: float = 0.0, tau: float = 0.07, epsilon: float = 1e-4) -> None:
        self.target = initial
        self.value = initial
        self.tau = tau
        self.epsilon = epsilon

    def set_target(self, value: float) -> None:
        self.target = finite(value)

    def snap(self, value: float) -> None:
        """Jump without animating. Used on first data and on range changes."""
        self.target = finite(value)
        self.value = self.target

    def advance(self, dt: float) -> bool:
        """Step the value. Returns True while still moving."""
        delta = self.target - self.value
        if abs(delta) <= self.epsilon:
            self.value = self.target
            return False
        if self.tau <= 0.0 or dt <= 0.0:
            self.value = self.target
            return False
        # 1 - exp(-dt/tau) rather than a fixed per-frame fraction: a dropped
        # frame then produces the same trajectory instead of a slower one.
        self.value += delta * (1.0 - math.exp(-dt / self.tau))
        return True


class PaintedWidget(QWidget):
    """Base class for every hand-drawn instrument.

    Subclasses override :meth:`on_layout` (precompute geometry),
    :meth:`paint_static` (cached background) and :meth:`paint_dynamic` (live
    content), and call :meth:`request_animation` when a follower needs driving.
    """

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent)
        self._theme = theme
        self._static: QPixmap | None = None
        self._static_dpr = 0.0
        self._anim_timer: QTimer | None = None
        self._anim_clock = QElapsedTimer()
        self.setObjectName("Instrument")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, False)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    # -- theme --------------------------------------------------------------

    @property
    def theme(self) -> Theme:
        return self._theme

    def set_theme(self, theme: Theme) -> None:
        self._theme = theme
        self.invalidate_static()
        self.on_layout(self.width(), self.height())

    # -- static cache -------------------------------------------------------

    def invalidate_static(self) -> None:
        """Drop the cached background. Cheap; call it whenever unsure."""
        self._static = None
        self.update()

    def resizeEvent(self, event: QResizeEvent) -> None:
        self._static = None
        self.on_layout(self.width(), self.height())
        super().resizeEvent(event)

    def changeEvent(self, event: QEvent) -> None:
        kind = event.type()
        if kind in (
            QEvent.Type.FontChange,
            QEvent.Type.PaletteChange,
            QEvent.Type.StyleChange,
            QEvent.Type.ThemeChange,
        ):
            self._static = None
            self.on_layout(self.width(), self.height())
        super().changeEvent(event)

    def paintEvent(self, event: QPaintEvent) -> None:
        width = self.width()
        height = self.height()
        if width <= 0 or height <= 0:
            return

        dpr = self.devicePixelRatioF()
        need = QSize(int(width * dpr + 0.5), int(height * dpr + 0.5))
        if self._static is None or self._static_dpr != dpr or self._static.size() != need:
            self._rebuild_static(width, height, dpr, need)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)
        if self._static is not None:
            painter.drawPixmap(0, 0, self._static)
        self.paint_dynamic(painter, width, height)
        painter.end()

    def _rebuild_static(self, width: int, height: int, dpr: float, need: QSize) -> None:
        pixmap = QPixmap(need)
        pixmap.setDevicePixelRatio(dpr)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)
        self.paint_static(painter, width, height)
        painter.end()
        self._static = pixmap
        self._static_dpr = dpr

    # -- animation ----------------------------------------------------------

    def request_animation(self) -> None:
        """Start the follower timer if it is not already running."""
        if self._anim_timer is None:
            timer = QTimer(self)
            timer.setInterval(_ANIM_INTERVAL_MS)
            timer.setTimerType(Qt.TimerType.PreciseTimer)
            timer.timeout.connect(self._on_anim_tick)
            self._anim_timer = timer
        if not self._anim_timer.isActive():
            self._anim_clock.restart()
            self._anim_timer.start()

    def _on_anim_tick(self) -> None:
        dt = self._anim_clock.restart() / 1000.0
        # A long stall (the app was backgrounded, or a modal blocked the loop)
        # must not produce one enormous integration step that overshoots.
        if dt > 0.25:
            dt = 0.25
        if not self.advance_animation(dt) and self._anim_timer is not None:
            self._anim_timer.stop()
        self.update()

    def advance_animation(self, dt: float) -> bool:
        """Advance followers. Return True while anything is still moving."""
        return False

    # -- subclass hooks -----------------------------------------------------

    def on_layout(self, width: int, height: int) -> None:
        """Precompute geometry for this size. Called on resize and theme change."""

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        """Draw everything that does not change between frames."""

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        """Draw the live content, over the cached background."""


# --------------------------------------------------------------------------
# Painting helpers
# --------------------------------------------------------------------------


def stroke_pen(color: QColor, width: float, *, round_cap: bool = True) -> QPen:
    pen = QPen(color)
    pen.setWidthF(width)
    pen.setCapStyle(
        Qt.PenCapStyle.RoundCap if round_cap else Qt.PenCapStyle.FlatCap
    )
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    return pen


def draw_text(
    painter: QPainter,
    rect: QRectF,
    text: str,
    font: QFont,
    color: QColor,
    align: Qt.AlignmentFlag = Qt.AlignmentFlag.AlignCenter,
) -> None:
    painter.setFont(font)
    painter.setPen(color)
    painter.drawText(rect, int(align), text)


def text_width(font: QFont, text: str) -> float:
    return QFontMetricsF(font).horizontalAdvance(text)


def text_height(font: QFont) -> float:
    return QFontMetricsF(font).height()


def polar(cx: float, cy: float, radius: float, degrees: float) -> tuple[float, float]:
    """Screen-space polar. Angles increase counter-clockwise, 0 at 3 o'clock,
    matching Qt's ``drawArc`` convention so the two never disagree."""
    a = math.radians(degrees)
    return cx + radius * math.cos(a), cy - radius * math.sin(a)


def font_pt(desired: float, minimum: int, maximum: int) -> int:
    """Clamp a size-derived point size.

    Instruments scale their type with their box, and that scaling has to be
    bounded at both ends. Below about 8 pt nothing is legible; above about
    100 pt a single glyph costs milliseconds to rasterise. A link panel
    stretched to 1200x800 was asking for 274 pt type and spending 18 ms a frame
    drawing two digits -- twice the entire per-frame budget, for a number that
    was already unreadably large.
    """
    value = int(desired) if math.isfinite(desired) else minimum
    return minimum if value < minimum else maximum if value > maximum else value


def format_fixed(value: float, decimals: int) -> str:
    """Fixed-point formatting that never emits ``nan`` or ``-0.0``.

    ``nan`` on a HUD is worse than a wrong number: it is unreadable at a glance
    and it changes the string width, which shifts everything next to it.
    """
    if not math.isfinite(value):
        return "--"
    if value == 0.0:
        value = 0.0  # collapses -0.0, which prints as "-0.0" and looks broken
    return "%.*f" % (decimals, value)


def format_signed(value: float, decimals: int) -> str:
    if not math.isfinite(value):
        return "--"
    if abs(value) < 0.5 * (10.0**-decimals):
        return "%+.*f" % (decimals, 0.0)
    return "%+.*f" % (decimals, value)


def format_clock(seconds: float) -> str:
    """``M:SS.mmm``. Lap times, session times, anything a stopwatch produces."""
    if not math.isfinite(seconds) or seconds < 0.0:
        return "--:--.---"
    minutes = int(seconds // 60.0)
    rest = seconds - minutes * 60.0
    return "%d:%06.3f" % (minutes, rest)


__all__ = [
    "PaintedWidget",
    "ValueFollower",
    "clamp",
    "draw_text",
    "finite",
    "font_pt",
    "format_clock",
    "format_fixed",
    "format_signed",
    "lerp",
    "polar",
    "stroke_pen",
    "text_height",
    "text_width",
]
