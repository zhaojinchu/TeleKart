"""64 px icon navigation rail.

One painted widget rather than a column of QToolButtons. Two reasons: the
active-item marker slides between positions, which needs a single coordinate
space to animate in, and a rail built from buttons inherits each button's own
focus rectangle, hover rectangle and padding, none of which line up with a
3 px bar pinned to the widget edge.

Fault badges are part of the rail on purpose. A driver looking at the video
feed should learn that something is wrong without changing screens, and the
rail is the one piece of chrome visible from every screen.
"""

from __future__ import annotations

from PySide6.QtCore import QEvent, QPointF, QRectF, QSize, Qt, Signal
from PySide6.QtGui import QColor, QHelpEvent, QKeyEvent, QMouseEvent, QPainter
from PySide6.QtWidgets import QSizePolicy, QToolTip, QWidget

from ..theme import fonts
from ..theme.icons import icon_pixmap
from ..theme.tokens import THEME, Theme, mix, with_alpha
from .base import PaintedWidget, ValueFollower, draw_text, stroke_pen

RAIL_WIDTH = 64
_ITEM_HEIGHT = 52
_ICON_SIZE = 22
_BRAND_HEIGHT = 60
_MARKER_WIDTH = 3.0


class _RailItem:
    __slots__ = ("key", "icon", "tooltip", "bottom", "rect", "badge", "enabled")

    def __init__(self, key: str, icon: str, tooltip: str, bottom: bool) -> None:
        self.key = key
        self.icon = icon
        self.tooltip = tooltip
        self.bottom = bottom
        self.rect = QRectF()
        self.badge = 0
        self.enabled = True


class NavigationRail(PaintedWidget):
    """Vertical icon rail with a sliding accent marker on the active item."""

    currentChanged = Signal(str)

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
        show_brand: bool = True,
    ) -> None:
        super().__init__(parent, theme=theme)
        self._items: list[_RailItem] = []
        self._current = ""
        self._hover = -1
        self._focus_index = -1
        self._show_brand = show_brand
        self._marker = ValueFollower(0.0, theme.duration.fast_ms / 1000.0 * 0.6)
        self._brand_rect = QRectF()

        self._font_badge = fonts.numeric_font(
            theme.type.micro, theme.weight.semibold, theme=theme
        )

        self.setObjectName("NavRail")
        self.setFixedWidth(RAIL_WIDTH)
        self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setCursor(Qt.CursorShape.ArrowCursor)

    # -- model --------------------------------------------------------------

    def add_item(
        self, key: str, icon: str, tooltip: str = "", *, bottom: bool = False
    ) -> None:
        """Append an item. ``bottom`` pins it to the bottom of the rail.

        Raises on a duplicate key: the key is what ``currentChanged`` carries
        and what the screen stack is keyed by, so a silent duplicate would show
        the wrong screen forever.
        """
        if any(item.key == key for item in self._items):
            raise ValueError("duplicate rail item key %r" % (key,))
        self._items.append(_RailItem(key, icon, tooltip or key.title(), bottom))
        if not self._current:
            self._current = key
            self._focus_index = 0
        self.invalidate_static()
        self.on_layout(self.width(), self.height())

    def set_current(self, key: str) -> None:
        if key == self._current or not any(i.key == key for i in self._items):
            return
        self._current = key
        self._focus_index = self._index_of(key)
        self._marker.set_target(self._marker_target())
        self.request_animation()
        self.update()
        self.currentChanged.emit(key)

    def current(self) -> str:
        return self._current

    def set_badge(self, key: str, count: int) -> None:
        """Fault or notification count. Zero hides the badge."""
        for item in self._items:
            if item.key == key:
                count = max(0, count)
                if item.badge != count:
                    item.badge = count
                    self.update()
                return

    def set_item_enabled(self, key: str, enabled: bool) -> None:
        for item in self._items:
            if item.key == key:
                item.enabled = enabled
                self.update()
                return

    def keys(self) -> list[str]:
        return [item.key for item in self._items]

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        return QSize(RAIL_WIDTH, 520)

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        self._brand_rect = QRectF(0.0, 0.0, width, _BRAND_HEIGHT)
        top = float(_BRAND_HEIGHT if self._show_brand else self.theme.space.sm)
        bottom = float(height) - self.theme.space.sm

        for item in self._items:
            if item.bottom:
                continue
            item.rect = QRectF(0.0, top, width, _ITEM_HEIGHT)
            top += _ITEM_HEIGHT

        for item in reversed(self._items):
            if not item.bottom:
                continue
            bottom -= _ITEM_HEIGHT
            item.rect = QRectF(0.0, bottom, width, _ITEM_HEIGHT)

        self._marker.snap(self._marker_target())

    def _marker_target(self) -> float:
        index = self._index_of(self._current)
        if index < 0:
            return 0.0
        return self._items[index].rect.center().y()

    def _index_of(self, key: str) -> int:
        for i, item in enumerate(self._items):
            if item.key == key:
                return i
        return -1

    def _item_at(self, y: float) -> int:
        for i, item in enumerate(self._items):
            if item.rect.top() <= y < item.rect.bottom():
                return i
        return -1

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        q = self.theme.q
        painter.fillRect(QRectF(0.0, 0.0, width, height), q.bg_raised)
        painter.setPen(stroke_pen(q.line, 1.0, round_cap=False))
        painter.drawLine(QPointF(width - 0.5, 0.0), QPointF(width - 0.5, height))

        if self._show_brand:
            pixmap = icon_pixmap("mark", q.accent, 26, self.devicePixelRatioF())
            painter.drawPixmap(
                QPointF(
                    self._brand_rect.center().x() - 13.0,
                    self._brand_rect.center().y() - 13.0,
                ),
                pixmap,
            )
            painter.setPen(stroke_pen(with_alpha(q.line, 0.9), 1.0, round_cap=False))
            painter.drawLine(
                QPointF(self.theme.space.md, _BRAND_HEIGHT - 0.5),
                QPointF(width - self.theme.space.md, _BRAND_HEIGHT - 0.5),
            )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        q = theme.q
        dpr = self.devicePixelRatioF()
        current_index = self._index_of(self._current)

        for i, item in enumerate(self._items):
            rect = item.rect
            if rect.isEmpty():
                continue
            active = i == current_index
            hovered = i == self._hover and item.enabled and not active

            if active:
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(mix(q.bg_raised, q.accent, 0.10))
                painter.drawRect(rect)
                painter.setBrush(Qt.BrushStyle.NoBrush)
            elif hovered:
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(mix(q.bg_raised, q.text_primary, 0.06))
                painter.drawRect(rect)
                painter.setBrush(Qt.BrushStyle.NoBrush)

            if not item.enabled:
                tint = with_alpha(q.text_tertiary, 0.45)
            elif active:
                tint = q.accent
            elif hovered:
                tint = q.text_primary
            else:
                tint = q.text_secondary

            pixmap = icon_pixmap(item.icon, tint, _ICON_SIZE, dpr)
            half = _ICON_SIZE * 0.5
            painter.drawPixmap(
                QPointF(rect.center().x() - half, rect.center().y() - half), pixmap
            )

            if item.badge > 0:
                self._paint_badge(painter, rect, item.badge)

            if self.hasFocus() and i == self._focus_index:
                painter.setPen(
                    stroke_pen(with_alpha(q.accent, 0.85), 1.0, round_cap=False)
                )
                painter.drawRoundedRect(
                    rect.adjusted(4.0, 3.0, -4.0, -3.0),
                    theme.radius.control,
                    theme.radius.control,
                )

        if current_index >= 0:
            y = self._marker.value
            half = _ITEM_HEIGHT * 0.32
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(q.accent)
            painter.drawRoundedRect(
                QRectF(0.0, y - half, _MARKER_WIDTH, half * 2.0), 1.5, 1.5
            )
            painter.setBrush(Qt.BrushStyle.NoBrush)

    def _paint_badge(self, painter: QPainter, rect: QRectF, count: int) -> None:
        q = self.theme.q
        text = "9+" if count > 9 else str(count)
        r = 7.5
        cx = rect.center().x() + _ICON_SIZE * 0.5 + 1.0
        cy = rect.center().y() - _ICON_SIZE * 0.5 + 1.0
        painter.setPen(stroke_pen(q.bg_raised, 1.5))
        painter.setBrush(q.bad)
        painter.drawEllipse(QPointF(cx, cy), r, r)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        draw_text(
            painter,
            QRectF(cx - r, cy - r, 2.0 * r, 2.0 * r),
            text,
            self._font_badge,
            QColor(q.bg_base),
        )

    # -- interaction --------------------------------------------------------

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        index = self._item_at(event.position().y())
        if index != self._hover:
            self._hover = index
            self.setCursor(
                Qt.CursorShape.PointingHandCursor
                if index >= 0 and self._items[index].enabled
                else Qt.CursorShape.ArrowCursor
            )
            self.update()

    def leaveEvent(self, event: QEvent) -> None:
        if self._hover != -1:
            self._hover = -1
            self.setCursor(Qt.CursorShape.ArrowCursor)
            self.update()
        super().leaveEvent(event)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        if event.button() != Qt.MouseButton.LeftButton:
            return
        index = self._item_at(event.position().y())
        if index >= 0 and self._items[index].enabled:
            self.set_current(self._items[index].key)

    def keyPressEvent(self, event: QKeyEvent) -> None:
        key = event.key()
        if key in (Qt.Key.Key_Down, Qt.Key.Key_Up):
            step = 1 if key == Qt.Key.Key_Down else -1
            index = self._focus_index
            for _ in range(len(self._items)):
                index = (index + step) % len(self._items)
                if self._items[index].enabled:
                    break
            self._focus_index = index
            self.update()
            return
        if key in (Qt.Key.Key_Return, Qt.Key.Key_Enter, Qt.Key.Key_Space):
            if 0 <= self._focus_index < len(self._items):
                self.set_current(self._items[self._focus_index].key)
            return
        super().keyPressEvent(event)

    def event(self, event: QEvent) -> bool:
        # Tooltips are the rail's only labels, so they are not decoration --
        # an icon-only navigation with no tooltips is a guessing game.
        if event.type() == QEvent.Type.ToolTip and isinstance(event, QHelpEvent):
            index = self._item_at(event.pos().y())
            if index >= 0:
                QToolTip.showText(event.globalPos(), self._items[index].tooltip, self)
            else:
                QToolTip.hideText()
            return True
        return super().event(event)

    def advance_animation(self, dt: float) -> bool:
        return self._marker.advance(dt)


__all__ = ["RAIL_WIDTH", "NavigationRail"]
