"""Centre event layer: the things that must interrupt driving.

Normally invisible, and that is the whole design. The centre of the picture is
where the driver is looking, so nothing lives there permanently -- but a latched
E-stop, a lost signal or a brownout is precisely the moment when covering the
road is the correct trade, because there is no useful driving left to do.

Two rules that are easy to get wrong:

**Pulse at 1.5 Hz, never faster.** A faster blink stops reading as emphasis and
starts reading as a broken display; it also makes the text unreadable, so the
driver has to wait for a bright phase to find out what is wrong. The pulse is
sinusoidal between 55 % and 100 % opacity rather than an on/off flash, for the
same reason.

**Rank, then truncate.** Three messages at once is the limit. A fault cascade --
a brownout takes both encoders down, which trips both stall detectors -- can
easily set six bits at the same instant, and a wall of red text conveys less
than the single most severe line does.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QColor, QFont, QFontMetricsF, QPainter
from PySide6.QtWidgets import QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, with_alpha
from ..widgets.base import PaintedWidget
from . import draw_scrim, micro_label_font

#: Warning pulse rate, hertz. The upper bound, not a suggestion.
PULSE_HZ = 1.5

_PULSE_FLOOR = 0.55
_MAX_EVENTS = 3
_PAD = 16.0
_ROW_H = 30.0
_TITLE_H = 30.0
_WIDTH = 560


class Severity(enum.IntEnum):
    """Ordered so ``sorted(reverse=True)`` is a ranking, not a lookup table."""

    INFO = 0
    WARNING = 1
    CRITICAL = 2


@dataclass(frozen=True, slots=True)
class BannerEvent:
    severity: Severity
    title: str
    detail: str = ""

    def __lt__(self, other: "BannerEvent") -> bool:
        return self.severity < other.severity


class EventBanner(PaintedWidget):
    """A ranked, pulsing stack of at most three interrupting messages."""

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent, theme=theme)

        self._events: tuple[BannerEvent, ...] = ()
        self._phase = 0.0
        self._pulsing = False

        self._font_title = fonts.ui_font(
            theme.type.title, theme.weight.bold, theme=theme
        )
        self._font_title.setCapitalization(QFont.Capitalization.AllUppercase)
        self._font_title.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.4)
        self._font_detail = fonts.ui_font(theme.type.body, theme.weight.medium, theme=theme)
        self._font_label = micro_label_font(theme)

        self._scrim = QRectF()
        self._rows: list[tuple[QRectF, QRectF]] = []

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setVisible(False)

    # -- inputs -------------------------------------------------------------

    def set_events(self, events: list[BannerEvent]) -> bool:
        """Replace the displayed set. Returns True when the geometry changed.

        The caller uses the return value to decide whether to re-place the
        banner; recomputing the overlay layout on every tick would be waste,
        and doing it never would leave a two-line banner sized for one.
        """
        ranked = tuple(sorted(events, reverse=True)[:_MAX_EVENTS])
        if ranked == self._events:
            return False

        was = self.size()
        self._events = ranked
        self._pulsing = any(e.severity >= Severity.WARNING for e in ranked)
        if not ranked:
            self._phase = 0.0
            self.setVisible(False)
            self.invalidate_static()
            return was != QSize(0, 0)

        self.resize(self.sizeHint())
        # Explicitly, not only via the resize: replacing two warnings with two
        # different warnings leaves the height unchanged, so no resizeEvent
        # arrives and the row rectangles would still describe the old set.
        self.on_layout(self.width(), self.height())
        self.setVisible(True)
        self.invalidate_static()
        if self._pulsing:
            self.request_animation()
        return self.size() != was

    def clear(self) -> None:
        self.set_events([])

    @property
    def active(self) -> bool:
        return bool(self._events)

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        if not self._events:
            return QSize(_WIDTH, 0)
        metrics = QFontMetricsF(self._font_detail)
        height = _PAD * 2 + _TITLE_H + _ROW_H * (len(self._events) - 1)
        if self._events[0].detail:
            height += metrics.height() + 4.0
        return QSize(_WIDTH, int(height + 0.5))

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        self._scrim = QRectF(0.0, 0.0, float(width), float(height))
        inner_l = _PAD
        inner_w = float(width) - _PAD * 2.0

        self._rows = []
        y = _PAD
        if self._events:
            metrics = QFontMetricsF(self._font_detail)
            title = QRectF(inner_l, y, inner_w, _TITLE_H)
            y += _TITLE_H
            detail_h = metrics.height() + 4.0 if self._events[0].detail else 0.0
            detail = QRectF(inner_l, y, inner_w, detail_h)
            y += detail_h
            self._rows.append((title, detail))
            for _ in self._events[1:]:
                self._rows.append(
                    (QRectF(inner_l, y, inner_w, _ROW_H), QRectF(0.0, 0.0, 0.0, 0.0))
                )
                y += _ROW_H

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        if not self._events:
            return
        # A slightly denser scrim than the rest of the HUD: this panel has to
        # win against whatever the camera happens to be pointing at, and it is
        # on screen for seconds, not for the whole drive.
        draw_scrim(painter, self._scrim, self.theme.radius.overlay, alpha=0.72)

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        if not self._events or not self._rows:
            return
        theme = self.theme
        alpha = self._pulse_alpha()

        head = self._events[0]
        title_rect, detail_rect = self._rows[0]
        painter.setFont(self._font_title)
        painter.setPen(with_alpha(self._color(head.severity), alpha))
        painter.drawText(
            title_rect,
            int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter),
            head.title,
        )

        if head.detail and detail_rect.height() > 0.0:
            painter.setFont(self._font_detail)
            painter.setPen(with_alpha(theme.q.text_primary, 0.80))
            painter.drawText(
                detail_rect,
                int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter),
                head.detail,
            )

        painter.setFont(self._font_label)
        for event, (rect, _) in zip(self._events[1:], self._rows[1:]):
            painter.setPen(with_alpha(self._color(event.severity), 0.85))
            painter.drawText(
                rect,
                int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter),
                event.title if not event.detail else event.title + " — " + event.detail,
            )

    def _color(self, severity: Severity) -> QColor:
        q = self.theme.q
        if severity is Severity.CRITICAL:
            return q.bad
        return q.warn if severity is Severity.WARNING else q.cyan

    def _pulse_alpha(self) -> float:
        if not self._pulsing:
            return 1.0
        # cos gives a smooth ramp in both directions; a square wave at this
        # rate is a strobe and makes the text hard to read at all.
        return _PULSE_FLOOR + (1.0 - _PULSE_FLOOR) * (
            0.5 + 0.5 * math.cos(self._phase * math.tau)
        )

    # -- animation ----------------------------------------------------------

    def advance_animation(self, dt: float) -> bool:
        if not self._pulsing:
            return False
        self._phase += dt * PULSE_HZ
        if self._phase >= 1.0:
            self._phase -= math.floor(self._phase)
        return True


__all__ = ["PULSE_HZ", "BannerEvent", "EventBanner", "Severity"]
