"""Top-right HUD zone: what state the car is in and whether the link is real.

The state badge is the largest non-numeric element on the screen because it
answers the only question whose wrong answer hurts: *are the motors live?* It
shows the state the **car** reported. The app never decides it is armed on the
strength of having sent an ARM -- if the session dropped between the request and
the acknowledgement, a locally-derived badge would say ARMED about a car that is
already coasting to a stop, and the driver would keep steering something that is
no longer listening.

The three pips underneath are the reason the badge can be trusted. CTRL is the
outbound command stream, TLM the inbound telemetry, VID the picture. A badge
reading ARMED next to a dead TLM pip means "the last thing the car said, some
time ago", and the pip is what makes that visible without a paragraph of text.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QWidget

from ..theme.tokens import THEME, Theme, mix, with_alpha
from ..widgets.base import PaintedWidget, ValueFollower, clamp, finite
from . import (
    SMOOTH_TAU_S,
    draw_scrim,
    label_color,
    micro_label_font,
    readout_font,
)

_PAD = 14.0
_BADGE_H = 42.0
#: The active-flag strip gets its own row under the badge. It used to be laid
#: out *inside* the badge rectangle, which put "LIMITER CAL CL ODOM VIDEO"
#: straight through the middle of the word ARMED -- and the state badge is the
#: one element on this screen whose legibility is not negotiable.
_FLAGS_H = 13.0
_PIPS_H = 22.0
_CELL_H = 46.0
_WIDTH = 280
_HEIGHT = int(
    _PAD * 2 + _BADGE_H + 6.0 + _FLAGS_H + 10.0 + _PIPS_H + 12.0 + _CELL_H * 2
)

_PIP_RADIUS = 4.0

#: Round-trip thresholds, in seconds. 40 ms is roughly one control period plus
#: one telemetry period on a healthy 2.4 GHz link; past 90 ms the driver is
#: steering where the car was, not where it is.
_RTT_GOOD = 0.040
_RTT_WARN = 0.090

#: Telemetry loss. One packet in a hundred is invisible at 50 Hz; five in a
#: hundred is a picture of the car that skips.
_LOSS_GOOD = 0.01
_LOSS_WARN = 0.05


class StatusCluster(PaintedWidget):
    """State badge, channel pips, and the four numbers that qualify them."""

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent, theme=theme)

        self._state_text = "OFFLINE"
        self._state_key = "boot"
        self._flag_text = ""

        self._pips: list[tuple[str, str]] = [
            ("ctrl", "idle"),
            ("tlm", "idle"),
            ("vid", "idle"),
        ]

        self._rtt = ValueFollower(0.0, SMOOTH_TAU_S)
        self._loss = ValueFollower(0.0, SMOOTH_TAU_S)
        self._volts = 0.0
        self._temp = 0.0
        self._throttled = 0

        self._font_badge = readout_font(theme)
        self._font_value = readout_font(theme, size=theme.type.title)
        self._font_label = micro_label_font(theme)

        self._scrim = QRectF()
        self._badge_rect = QRectF()
        self._flag_rect = QRectF()
        self._pip_rects: list[QRectF] = []
        self._cells: list[tuple[QRectF, QRectF]] = []

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setFixedSize(QSize(_WIDTH, _HEIGHT))

    # -- inputs -------------------------------------------------------------

    def set_state(self, text: str, key: str) -> None:
        """``text`` is the driver-facing label; ``key`` selects the state colour."""
        changed = text != self._state_text or key != self._state_key
        self._state_text = text
        self._state_key = key
        if changed:
            self.invalidate_static()

    def set_flags(self, text: str) -> None:
        """A short strip of active-flag abbreviations, or "" for none."""
        if text != self._flag_text:
            self._flag_text = text
            self.update()

    def set_channel(self, name: str, health: str) -> None:
        """``health`` is a :class:`LinkColors` role: nominal/degraded/down/idle."""
        for index, (key, current) in enumerate(self._pips):
            if key == name:
                if current != health:
                    self._pips[index] = (key, health)
                    self.update()
                return

    def set_rtt(self, seconds: float) -> None:
        self._rtt.set_target(max(0.0, finite(seconds)))
        self.request_animation()

    def set_loss(self, fraction: float) -> None:
        self._loss.set_target(clamp(finite(fraction), 0.0, 1.0))
        self.request_animation()

    def set_power(self, volts: float, celsius: float, throttled: int) -> None:
        if (volts, celsius, throttled) == (self._volts, self._temp, self._throttled):
            return
        self._volts = finite(volts)
        self._temp = finite(celsius)
        self._throttled = throttled
        self.update()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(_WIDTH, _HEIGHT)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        w = float(width)
        self._scrim = QRectF(0.0, 0.0, w, float(height))

        left = _PAD
        right = w - _PAD
        y = _PAD

        self._badge_rect = QRectF(left, y, right - left, _BADGE_H)
        y += _BADGE_H + 6.0
        self._flag_rect = QRectF(left, y, right - left, _FLAGS_H)
        y += _FLAGS_H + 10.0

        pip_w = (right - left) / 3.0
        self._pip_rects = [
            QRectF(left + i * pip_w, y, pip_w, _PIPS_H) for i in range(3)
        ]
        y += _PIPS_H + 12.0

        cell_w = (right - left) / 2.0
        self._cells = []
        for index in range(4):
            col = index % 2
            row = index // 2
            box = QRectF(left + col * cell_w, y + row * _CELL_H, cell_w, _CELL_H)
            self._cells.append(
                (
                    QRectF(box.left(), box.top(), box.width(), 13.0),
                    QRectF(box.left(), box.top() + 12.0, box.width(), _CELL_H - 14.0),
                )
            )

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        draw_scrim(painter, self._scrim, theme.radius.overlay)

        color = theme.state_color(self._state_key)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(with_alpha(color, 0.18))
        painter.drawRoundedRect(
            self._badge_rect, theme.radius.control, theme.radius.control
        )
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.setFont(self._font_badge)
        painter.setPen(color)
        painter.drawText(
            self._badge_rect,
            int(Qt.AlignmentFlag.AlignCenter),
            self._state_text,
        )

        painter.setFont(self._font_label)
        painter.setPen(label_color(theme))
        for rect, label in zip(
            (c[0] for c in self._cells), ("rtt ms", "loss %", "pack v", "soc °c")
        ):
            painter.drawText(
                rect,
                int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                label,
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme

        if self._flag_text:
            painter.setFont(self._font_label)
            painter.setPen(with_alpha(theme.q.text_primary, 0.65))
            painter.drawText(
                self._flag_rect,
                int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter),
                self._flag_text,
            )

        painter.setFont(self._font_label)
        for rect, (name, health) in zip(self._pip_rects, self._pips):
            color = theme.link_color(health)
            dot = QRectF(
                rect.left(),
                rect.center().y() - _PIP_RADIUS,
                _PIP_RADIUS * 2.0,
                _PIP_RADIUS * 2.0,
            )
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(dot)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(
                with_alpha(theme.q.text_primary, 0.75 if health != "idle" else 0.35)
            )
            painter.drawText(
                QRectF(
                    rect.left() + _PIP_RADIUS * 2.0 + 6.0,
                    rect.top(),
                    rect.width() - _PIP_RADIUS * 2.0 - 6.0,
                    rect.height(),
                ),
                int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                name,
            )

        values = (
            (self._rtt_text(), self._rtt_color()),
            (self._loss_text(), self._loss_color()),
            (self._volts_text(), theme.q.text_primary),
            (self._temp_text(), self._temp_color()),
        )
        painter.setFont(self._font_value)
        for (_, value_rect), (text, color) in zip(self._cells, values):
            painter.setPen(color)
            painter.drawText(
                value_rect,
                int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop),
                text,
            )

    # -- value formatting ---------------------------------------------------

    def _rtt_text(self) -> str:
        value = self._rtt.value
        # Zero means "never measured", which is a different statement from a
        # zero-millisecond round trip and must not read as a perfect link.
        return "--" if value <= 0.0 else "%.0f" % (value * 1000.0,)

    def _rtt_color(self) -> QColor:
        value = self._rtt.value
        q = self.theme.q
        if value <= 0.0:
            return q.text_tertiary
        if value <= _RTT_GOOD:
            return q.text_primary
        return q.warn if value <= _RTT_WARN else q.bad

    def _loss_text(self) -> str:
        return "%.1f" % (self._loss.value * 100.0,)

    def _loss_color(self) -> QColor:
        q = self.theme.q
        value = self._loss.value
        if value <= _LOSS_GOOD:
            return q.text_primary
        return q.warn if value <= _LOSS_WARN else q.bad

    def _volts_text(self) -> str:
        return "--" if self._volts <= 0.0 else "%.2f" % (self._volts,)

    def _temp_text(self) -> str:
        return "--" if self._temp <= 0.0 else "%.0f" % (self._temp,)

    def _temp_color(self) -> QColor:
        q = self.theme.q
        if self._throttled:
            # vcgencmd's bitmask is the ground truth here: the Pi tells us it
            # is capping itself long before the die temperature looks alarming.
            return q.bad if self._throttled & 0x7 else q.warn
        if self._temp <= 0.0:
            return q.text_tertiary
        if self._temp >= 80.0:
            return q.bad
        return q.warn if self._temp >= 70.0 else q.text_primary

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        moving = self._rtt.advance(dt)
        return self._loss.advance(dt) or moving


def health_for_age(age: float, *, warn: float, down: float, active: bool) -> str:
    """Channel age -> a :class:`LinkColors` role.

    Shared by the cluster's three pips so a stale video channel and a stale
    telemetry channel are coloured by the same rule and differ only in their
    thresholds, which the caller supplies.
    """
    if not active:
        return "idle"
    if age >= down:
        return "down"
    return "degraded" if age >= warn else "nominal"


def blend_state(theme: Theme, key: str, t: float) -> QColor:
    """State colour blended toward the base ground. For soft chips and fills."""
    return mix(theme.q.bg_base, theme.state_color(key), t)


__all__ = ["StatusCluster", "blend_state", "health_for_age"]
