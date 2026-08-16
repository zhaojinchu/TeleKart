"""Link health for the three independent streams, plus RTT and loss.

The three streams fail independently and for different reasons, so they get
three independent indicators. Control is UDP at 100 Hz and its loss is what
trips the failsafe; telemetry is UDP at 50 Hz and its loss only costs you
instrument freshness; video is a TCP frame stream whose stall says nothing at
all about whether the car is still driveable. Collapsing them into one
"connected" light is how an operator ends up pulling the battery because the
picture froze.

RTT is measured properly: the car echoes ``client_time_us`` from the last
control packet it accepted, and the app subtracts it from its own clock. No
clock synchronisation is involved, so the number is a true round trip rather
than a one-way estimate with an unknown offset baked into it.
"""

from __future__ import annotations

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import (
    PaintedWidget,
    clamp,
    draw_text,
    finite,
    font_pt,
    format_fixed,
    stroke_pen,
    text_width,
)

#: Channel keys, in display order. Fixed rather than configurable: these are
#: the three streams the protocol defines, and a fourth would be a protocol
#: change, not a UI change.
CHANNELS: tuple[tuple[str, str], ...] = (
    ("ctrl", "CTRL"),
    ("tlm", "TLM"),
    ("vid", "VID"),
)

_PIPS = 4
_SPARK_SAMPLES = 72
_SPARK_MAX_HEIGHT = 50.0

#: Round-trip thresholds. 60 ms is where a wheel starts to feel disconnected
#: from the car; 120 ms is where a driver begins over-correcting, which is its
#: own hazard well before the 200 ms failsafe timeout.
_RTT_GOOD_MS = 60.0
_RTT_POOR_MS = 120.0

_LOSS_WARN = 0.01
_LOSS_BAD = 0.05


class _Channel:
    __slots__ = ("key", "label", "alive", "quality", "rate_hz")

    def __init__(self, key: str, label: str) -> None:
        self.key = key
        self.label = label
        self.alive = False
        self.quality = 0.0
        self.rate_hz: float | None = None


class LinkQuality(PaintedWidget):
    """Three pip rows, a round-trip readout, a loss readout and an RTT trace."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        show_sparkline: bool = True,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._channels = [_Channel(key, label) for key, label in CHANNELS]
        self._by_key = {c.key: c for c in self._channels}
        self._rtt_ms: float | None = None
        self._rtt_text = "--"
        self._rtt_w = 0.0
        self._loss = 0.0
        self._show_sparkline = show_sparkline

        self._history: list[float] = []
        self._spark = QPolygonF()
        self._spark_max = 1.0

        self._rows: list[QRectF] = []
        self._label_w = 34.0
        self._pip = QRectF()
        self._pip_gap = 3.0
        self._rate_w = 40.0
        self._rtt_rect = QRectF()
        self._rtt_unit_rect = QRectF()
        self._loss_rect = QRectF()
        self._spark_rect = QRectF()
        self._divider_x = 0.0

        self._font_label = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_rate = fonts.numeric_font(theme.type.micro, theme=theme)
        self._font_rtt = fonts.numeric_font(
            theme.type.readout, theme.weight.medium, theme=theme
        )
        self._font_small = fonts.ui_font(theme.type.micro, theme=theme)
        self._font_loss = fonts.numeric_font(theme.type.label, theme=theme)

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumSize(QSize(250, 68))

    # -- inputs -------------------------------------------------------------

    def set_channel(
        self,
        key: str,
        alive: bool,
        quality: float = 1.0,
        rate_hz: float | None = None,
    ) -> None:
        """Update one stream. Unknown keys are ignored rather than raising.

        Ignored because this is fed from a decode path, and a firmware from the
        future reporting a fourth stream must not take the HUD down.
        """
        channel = self._by_key.get(key)
        if channel is None:
            return
        channel.alive = alive
        channel.quality = clamp(finite(quality), 0.0, 1.0)
        channel.rate_hz = rate_hz
        self.update()

    def set_rtt_ms(self, value: float | None) -> None:
        """Round-trip time in milliseconds. ``None`` means no echo yet."""
        if value is None:
            self._rtt_ms = None
        else:
            self._rtt_ms = max(0.0, finite(value))
            self._history.append(self._rtt_ms)
            if len(self._history) > _SPARK_SAMPLES:
                del self._history[: len(self._history) - _SPARK_SAMPLES]
            self._rebuild_spark()
        self._rtt_text = "--" if self._rtt_ms is None else format_fixed(self._rtt_ms, 0)
        self._rtt_w = text_width(self._font_rtt, self._rtt_text)
        self.update()

    def set_loss(self, fraction: float) -> None:
        """Control-stream loss as a fraction, 0..1."""
        self._loss = clamp(finite(fraction), 0.0, 1.0)
        self.update()

    def clear_history(self) -> None:
        self._history.clear()
        self._spark = QPolygonF()
        self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(340, 72)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.sm)
        rows = len(self._channels)
        usable_h = height - 2.0 * pad
        row_h = usable_h / rows

        pip_w = 7.0
        pip_h = max(5.0, min(9.0, row_h * 0.46))
        self._pip = QRectF(0.0, 0.0, pip_w, pip_h)
        self._pip_gap = 3.0

        self._rows = [
            QRectF(pad, pad + i * row_h, 132.0, row_h) for i in range(rows)
        ]
        self._divider_x = pad + 132.0 + float(theme.space.sm)

        right = self._divider_x + float(theme.space.md)
        right_w = max(60.0, width - right - pad)

        self._rtt_rect = QRectF(right, pad - 2.0, right_w * 0.56, row_h * 1.5)
        # Width only; the unit is placed against the rendered number in
        # paint_dynamic so "26 ms" reads as one figure rather than two fields.
        self._rtt_unit_rect = QRectF(0.0, 0.0, 26.0, row_h * 0.7)
        self._loss_rect = QRectF(
            right, self._rtt_rect.bottom() - 2.0, right_w * 0.56, row_h
        )
        # Height-capped and centred. A sparkline is a small-multiple: past
        # about 50 px of amplitude it conveys nothing extra, and a full-height
        # trace across a stretched panel is both wrong-looking and genuinely
        # expensive -- 60 near-vertical antialiased segments over 650 px cost
        # more to rasterise than everything else in this widget put together.
        spark_h = min(_SPARK_MAX_HEIGHT, usable_h - row_h * 0.5)
        self._spark_rect = QRectF(
            right + right_w * 0.60,
            pad + (usable_h - spark_h) * 0.5,
            right_w * 0.40,
            max(8.0, spark_h),
        )

        self._font_rtt = fonts.numeric_font(
            font_pt(row_h * 1.05, 14, 34), theme.weight.medium, theme=theme
        )
        self._rtt_w = text_width(self._font_rtt, self._rtt_text)
        self._rebuild_spark()

    def _rebuild_spark(self) -> None:
        """Recompute the trace on new data, not on every frame.

        The polygon only changes when a sample arrives (50 Hz at most), so
        rebuilding it in the paint path would be sixty rebuilds a second for
        fifty new points.
        """
        if not self._show_sparkline or self._spark_rect.isEmpty():
            return
        count = len(self._history)
        if count < 2:
            self._spark = QPolygonF()
            return
        peak = max(self._history)
        self._spark_max = max(20.0, peak * 1.15)
        rect = self._spark_rect
        step = rect.width() / (count - 1)
        points = QPolygonF()
        for i, value in enumerate(self._history):
            frac = clamp(value / self._spark_max, 0.0, 1.0)
            points.append(
                QPointF(rect.left() + i * step, rect.bottom() - rect.height() * frac)
            )
        self._spark = points

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        painter.setPen(stroke_pen(q.line, 1.0, round_cap=False))
        painter.drawLine(
            QPointF(self._divider_x, float(self.theme.space.sm)),
            QPointF(self._divider_x, height - self.theme.space.sm),
        )
        if self._show_sparkline:
            painter.setPen(
                stroke_pen(with_alpha(q.line, 0.8), 1.0, round_cap=False)
            )
            painter.drawLine(
                QPointF(self._spark_rect.left(), self._spark_rect.bottom()),
                QPointF(self._spark_rect.right(), self._spark_rect.bottom()),
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q

        # Non-strict: _rows is empty until the first layout pass, and raising
        # inside a paint event kills the widget's repaints for good.
        for row, channel in zip(self._rows, self._channels, strict=False):
            color = self._channel_color(channel)
            draw_text(
                painter,
                QRectF(row.left(), row.top(), self._label_w, row.height()),
                channel.label,
                self._font_label,
                q.text_secondary if channel.alive else q.text_tertiary,
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            )

            lit = 0 if not channel.alive else _pips_for(channel.quality)
            x = row.left() + self._label_w + 4.0
            y = row.center().y() - self._pip.height() * 0.5
            painter.setPen(Qt.PenStyle.NoPen)
            for i in range(_PIPS):
                painter.setBrush(
                    color if i < lit else mix(q.bg_base, q.line, 0.85)
                )
                painter.drawRoundedRect(
                    QRectF(
                        x + i * (self._pip.width() + self._pip_gap),
                        y,
                        self._pip.width(),
                        self._pip.height(),
                    ),
                    1.5,
                    1.5,
                )
            painter.setBrush(Qt.BrushStyle.NoBrush)

            rate_x = x + _PIPS * (self._pip.width() + self._pip_gap) + 6.0
            rate_text = (
                "--"
                if (not channel.alive or channel.rate_hz is None)
                else format_fixed(channel.rate_hz, 0)
            )
            draw_text(
                painter,
                QRectF(rate_x, row.top(), self._rate_w, row.height()),
                rate_text,
                self._font_rate,
                q.text_tertiary,
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )

        rtt_color = self._rtt_color()
        draw_text(
            painter,
            self._rtt_rect,
            self._rtt_text,
            self._font_rtt,
            rtt_color,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )
        draw_text(
            painter,
            QRectF(
                self._rtt_rect.left() + self._rtt_w + 4.0,
                self._rtt_rect.center().y() - 1.0,
                self._rtt_unit_rect.width(),
                self._rtt_unit_rect.height(),
            ),
            "ms",
            self._font_small,
            q.text_tertiary,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )
        draw_text(
            painter,
            self._loss_rect,
            "LOSS " + format_fixed(self._loss * 100.0, 1) + "%",
            self._font_loss,
            self._loss_color(),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
        )

        if self._show_sparkline and self._spark.count() >= 2:
            painter.setPen(stroke_pen(with_alpha(rtt_color, 0.9), 1.4))
            painter.drawPolyline(self._spark)

    # -- colour rules -------------------------------------------------------

    def _channel_color(self, channel: _Channel) -> QColor:
        theme = self.theme
        if not channel.alive:
            return theme.link_color("down")
        if channel.quality >= 0.75:
            return theme.link_color("nominal")
        if channel.quality >= 0.35:
            return theme.link_color("degraded")
        return theme.link_color("down")

    def _rtt_color(self) -> QColor:
        theme = self.theme
        if self._rtt_ms is None:
            return theme.link_color("idle")
        if self._rtt_ms <= _RTT_GOOD_MS:
            return theme.q.text_primary
        if self._rtt_ms <= _RTT_POOR_MS:
            return theme.link_color("degraded")
        return theme.link_color("down")

    def _loss_color(self) -> QColor:
        theme = self.theme
        if self._loss < _LOSS_WARN:
            return theme.q.text_secondary
        if self._loss < _LOSS_BAD:
            return theme.link_color("degraded")
        return theme.link_color("down")


def _pips_for(quality: float) -> int:
    """At least one pip for any live channel: zero pips means "dead"."""
    return max(1, min(_PIPS, int(quality * _PIPS + 0.999)))


__all__ = ["CHANNELS", "LinkQuality"]
