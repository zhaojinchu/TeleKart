"""Per-sector status blocks, in the colour language every timing screen uses.

Purple beats green beats yellow. That ordering is not arbitrary decoration --
it is the convention on every professional timing screen, so a driver already
knows it, and reusing it costs nothing. It also happens to be consistent with
this application's rule that green means faster: a green sector is a personal
best, a purple one is the best anyone has done this session, and neither ever
means "fine" or "connected".

A sector that has not been driven yet is drawn as an empty outline rather than
a dim colour, because "no data" and "slow" must not look similar at a glance.
"""

from __future__ import annotations

import enum

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QSizePolicy, QWidget

from ..theme import fonts
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import PaintedWidget, draw_text, format_clock, stroke_pen


class SectorStatus(enum.IntEnum):
    """Outcome of one sector, worst to best."""

    PENDING = 0
    SLOWER = 1
    PERSONAL_BEST = 2
    SESSION_BEST = 3


_STATUS_KEY: dict[SectorStatus, str] = {
    SectorStatus.PENDING: "text_tertiary",
    SectorStatus.SLOWER: "warn",
    SectorStatus.PERSONAL_BEST: "good",
    SectorStatus.SESSION_BEST: "purple",
}


class _Sector:
    __slots__ = ("status", "time_s", "rect", "text_rect")

    def __init__(self) -> None:
        self.status = SectorStatus.PENDING
        self.time_s: float | None = None
        self.rect = QRectF()
        self.text_rect = QRectF()


class SectorLights(PaintedWidget):
    """A row of sector blocks with optional split times underneath."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        sectors: int = 3,
        show_times: bool = True,
        caption: str = "SECTORS",
    ) -> None:
        super().__init__(parent, theme=theme)
        self._sectors = [_Sector() for _ in range(max(1, sectors))]
        self._show_times = show_times
        self._caption = caption
        self._active = -1

        self._caption_rect = QRectF()
        self._radius = 2.0

        self._font_caption = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )
        self._font_time = fonts.numeric_font(theme.type.micro, theme=theme)
        self._font_index = fonts.ui_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setMinimumSize(QSize(160, 40))

    # -- model --------------------------------------------------------------

    def set_sector_count(self, count: int) -> None:
        count = max(1, count)
        if count == len(self._sectors):
            return
        self._sectors = [_Sector() for _ in range(count)]
        self._active = -1
        self.invalidate_static()
        self.on_layout(self.width(), self.height())

    def set_sector(
        self, index: int, status: SectorStatus, time_s: float | None = None
    ) -> None:
        """Record one sector's outcome. Out-of-range indices are ignored."""
        if not 0 <= index < len(self._sectors):
            return
        sector = self._sectors[index]
        sector.status = status
        sector.time_s = time_s
        self.update()

    def set_active(self, index: int) -> None:
        """The sector currently being driven, outlined in accent."""
        if index != self._active:
            self._active = index
            self.update()

    def reset(self) -> None:
        for sector in self._sectors:
            sector.status = SectorStatus.PENDING
            sector.time_s = None
        self._active = -1
        self.update()

    @property
    def count(self) -> int:
        return len(self._sectors)

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(300, 46)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        theme = self.theme
        pad = float(theme.space.xs)
        caption_h = 12.0 if self._caption else 0.0
        time_h = 13.0 if self._show_times else 0.0

        count = len(self._sectors)
        gap = float(theme.space.xs)
        block_w = (width - 2.0 * pad - gap * (count - 1)) / count
        block_h = max(6.0, height - caption_h - time_h - 2.0 * pad)
        top = caption_h + pad

        for i, sector in enumerate(self._sectors):
            x = pad + i * (block_w + gap)
            sector.rect = QRectF(x, top, block_w, block_h)
            sector.text_rect = QRectF(x, top + block_h + 1.0, block_w, time_h)

        self._caption_rect = QRectF(pad, 0.0, width - 2.0 * pad, caption_h)
        self._radius = min(3.0, block_h * 0.3)

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        if self._caption:
            draw_text(
                painter,
                self._caption_rect,
                self._caption,
                self._font_caption,
                q.text_tertiary,
                Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            )
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(mix(q.bg_base, q.line, 0.5))
        for sector in self._sectors:
            painter.drawRoundedRect(sector.rect, self._radius, self._radius)
        painter.setBrush(Qt.BrushStyle.NoBrush)

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        for i, sector in enumerate(self._sectors):
            color = self._color(sector.status)
            if sector.status is SectorStatus.PENDING:
                painter.setPen(stroke_pen(with_alpha(color, 0.7), 1.0, round_cap=False))
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawRoundedRect(
                    sector.rect.adjusted(0.5, 0.5, -0.5, -0.5),
                    self._radius,
                    self._radius,
                )
            else:
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(color)
                painter.drawRoundedRect(sector.rect, self._radius, self._radius)
                painter.setBrush(Qt.BrushStyle.NoBrush)

            if i == self._active:
                painter.setPen(stroke_pen(q.accent, 1.6, round_cap=False))
                painter.drawRoundedRect(
                    sector.rect.adjusted(-1.0, -1.0, 1.0, 1.0),
                    self._radius + 1.0,
                    self._radius + 1.0,
                )

            if self._show_times:
                text = "--:--.---" if sector.time_s is None else format_clock(sector.time_s)
                draw_text(
                    painter,
                    sector.text_rect,
                    text,
                    self._font_time,
                    q.text_secondary
                    if sector.status is not SectorStatus.PENDING
                    else q.text_tertiary,
                )

    def _color(self, status: SectorStatus) -> QColor:
        return self.theme.rgb(_STATUS_KEY[status])


__all__ = ["SectorLights", "SectorStatus"]
