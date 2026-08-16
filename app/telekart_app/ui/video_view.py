"""The video surface: one blit, one overlay, one pull per UI frame.

Three decisions are load-bearing here.

**Nothing repaints on frame arrival.** The decode thread never touches a widget;
it drops a :class:`~telekart_app.video.frame.FrameBundle` into a depth-1 box and
carries on. The UI's own 60 Hz tick calls :meth:`VideoView.tick`, which takes the
bundle *if* there is a new one and repaints. Video therefore runs at 30 and the
HUD at 60, which is the correct pairing: the picture only changes 30 times a
second, but a needle driven by 50 Hz telemetry and smoothed to 70 ms needs 60 Hz
to look like motion rather than stepping. Letting the decoder drive repaints
would invert that -- the HUD would stutter at camera rate and the GUI thread
would be woken by a thread it has no business being coupled to.

**The bundle is held for the whole paint.** ``FrameBundle`` owns an ``av.VideoFrame``
and a ``QImage`` that views its buffer without copying. Storing the image and
dropping the bundle is a use-after-free that renders correctly most of the time.
So the view keeps its own reference until the next one replaces it, and takes
from the provider exactly once per tick rather than peeking twice.

**The pixel path is isolated.** :class:`FramePresenter` is the only thing that
knows the frame is a ``QImage`` drawn with ``QPainter``. That is the pragmatic
choice today -- a 720p ``drawImage`` of a pre-converted RGB32 buffer is one to two
milliseconds of GUI time, and the conversion already happened on the decode
thread where there is slack. When it stops being enough, a ``QOpenGLWidget`` with
a YUV fragment shader replaces the presenter and this widget's paint path, and
every caller keeps working, because callers only ever see ``tick()``,
``set_frame_provider()``, ``hud`` and ``displaySizeChanged``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

from PySide6.QtCore import QRect, QRectF, QSize, Qt, QTimer, Signal
from PySide6.QtGui import (
    QColor,
    QFont,
    QFontMetricsF,
    QPainter,
    QPaintEvent,
    QResizeEvent,
    QShowEvent,
)
from PySide6.QtWidgets import QSizePolicy, QWidget

from .hud.hud_overlay import HudOverlay
from .theme import fonts
from .theme.tokens import THEME, Theme, with_alpha

if TYPE_CHECKING:  # pragma: no cover - typing only
    # Import-time-free on purpose: ``video.frame`` pulls in PyAV, and the whole
    # UI layer stays constructible in a test environment that has no codecs.
    from ..video.frame import FrameBundle

FrameProvider = Callable[[], "FrameBundle | None"]

#: How long the widget waits after the last resize before telling the decoder to
#: reformat. Dragging a window edge produces hundreds of resize events, and
#: reconfiguring an H.264 scaler on each one stalls the decode thread for longer
#: than the drag lasts.
_RESIZE_SETTLE_MS = 180

#: Below this relative scale error the blit is treated as 1:1 and the smooth
#: transform is skipped. The decode thread reformats to the widget size, so the
#: normal case is exact; a few pixels of mismatch during a resize is not worth
#: a bilinear pass over a full frame.
_SCALE_EPSILON = 0.02


class FramePresenter:
    """Holds the current frame and puts it on a painter. The swappable part.

    Deliberately not a QWidget. It owns no geometry and no repaint policy, only
    the answer to "what pixels, drawn how" -- which is exactly the part a GPU
    path replaces.
    """

    __slots__ = ("_bundle",)

    def __init__(self) -> None:
        self._bundle: "FrameBundle | None" = None

    def present(self, bundle: "FrameBundle | None") -> None:
        self._bundle = bundle

    def release(self) -> None:
        """Drop the frame. Releases several megabytes of decoded picture."""
        self._bundle = None

    @property
    def bundle(self) -> "FrameBundle | None":
        return self._bundle

    @property
    def has_content(self) -> bool:
        return self._bundle is not None

    def content_size(self) -> QSize:
        bundle = self._bundle
        return QSize(0, 0) if bundle is None else QSize(bundle.width, bundle.height)

    def paint(self, painter: QPainter, target: QRectF) -> None:
        bundle = self._bundle
        if bundle is None or target.width() <= 0.0 or target.height() <= 0.0:
            return
        source = QRectF(0.0, 0.0, float(bundle.width), float(bundle.height))
        smooth = abs(target.width() / source.width() - 1.0) > _SCALE_EPSILON
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, smooth)
        painter.drawImage(target, bundle.image, source)


class VideoView(QWidget):
    """Full-bleed video with the HUD composited over it.

    The HUD is a *child* of this widget rather than a sibling. That is what
    makes Qt repaint the picture beneath a HUD panel when the panel updates;
    two siblings in the same backing store do not blend, and the overlay would
    either erase the video or leave trails.
    """

    #: Physical pixels the decoder should reformat to. Debounced.
    displaySizeChanged = Signal(int, int)

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent)
        self._theme = theme
        self._presenter = FramePresenter()
        self._provider: FrameProvider | None = None

        self._frames = 0
        self._stale = False
        self._placeholder = "no video"
        self._placeholder_detail = "connect to a car, or launch the simulator"

        self._font_placeholder = fonts.ui_font(
            theme.type.title, theme.weight.medium, theme=theme
        )
        self._font_detail = fonts.ui_font(theme.type.body, theme.weight.regular, theme=theme)
        self._font_badge = fonts.ui_font(theme.type.micro, theme.weight.semibold, theme=theme)
        self._font_badge.setCapitalization(QFont.Capitalization.AllUppercase)
        self._font_badge.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.2)

        self._target = QRectF()

        self._resize_timer = QTimer(self)
        self._resize_timer.setSingleShot(True)
        self._resize_timer.setInterval(_RESIZE_SETTLE_MS)
        self._resize_timer.timeout.connect(self._announce_display_size)

        self.hud = HudOverlay(self, theme=theme)

        self.setObjectName("VideoView")
        self.setAutoFillBackground(False)
        # The widget paints every pixel it owns -- picture or placeholder -- so
        # Qt can skip clearing it first. That is a full-screen fill saved on
        # every frame.
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent, True)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(QSize(320, 180))
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # -- wiring -------------------------------------------------------------

    def set_frame_provider(self, provider: FrameProvider | None) -> None:
        """Install the source of frames.

        The provider must have *take* semantics: return a bundle only when
        there is a new one, and ``None`` otherwise. Peek semantics would make
        this widget repaint the same picture sixty times a second.
        """
        self._provider = provider

    def tick(self) -> bool:
        """One UI frame. Returns True when a new picture was taken.

        The HUD does not need calling: its instruments animate themselves and
        Qt repaints them over whatever this widget just drew.
        """
        provider = self._provider
        if provider is None:
            return False
        bundle = provider()
        if bundle is None:
            return False
        previous = self._presenter.content_size()
        self._presenter.present(bundle)
        self._frames += 1
        if self._presenter.content_size() != previous:
            self._recompute_target()
            # The picture just changed shape, so the letterbox did too and the
            # HUD has to follow it. This is the path a stream resolution change
            # takes; the resize path is handled in resizeEvent.
            self._place_hud()
            self.update()
        else:
            # Only the picture changed, so only the picture area is dirty. On a
            # letterboxed 16:9 stream in a 16:10 window that is a real saving,
            # and it costs one rectangle.
            self.update(self._target.toRect())
        return True

    def clear(self) -> None:
        """Drop the current frame and go back to the placeholder."""
        self._presenter.release()
        self._frames = 0
        self._stale = False
        self._recompute_target()
        self._place_hud()
        self.update()

    def set_stale(self, stale: bool) -> None:
        """Mark the picture as no longer live.

        A frozen frame is the most dangerous thing this widget can show: it
        looks exactly like a road that is still there. Dimming it and saying so
        costs nothing and removes the ambiguity.
        """
        if stale != self._stale:
            self._stale = stale
            self.update()

    def set_placeholder(self, text: str, detail: str = "") -> None:
        self._placeholder = text
        self._placeholder_detail = detail
        if not self._presenter.has_content:
            self.update()

    @property
    def has_frame(self) -> bool:
        return self._presenter.has_content

    @property
    def frames_shown(self) -> int:
        return self._frames

    def frame_rect(self) -> QRect:
        """Where the picture is, in widget coordinates. Letterboxed."""
        return self._target.toRect()

    def display_size(self) -> QSize:
        """The size the decoder should reformat to, in physical pixels."""
        dpr = self.devicePixelRatioF()
        return QSize(int(self.width() * dpr + 0.5), int(self.height() * dpr + 0.5))

    # -- geometry -----------------------------------------------------------

    def resizeEvent(self, event: QResizeEvent) -> None:
        self._recompute_target()
        self._place_hud()
        self._resize_timer.start()
        super().resizeEvent(event)

    def showEvent(self, event: QShowEvent) -> None:
        # Qt does not deliver a resize event to a widget that has never been
        # shown -- it queues one for the show. A view constructed at its final
        # size and then shown would therefore never have placed its overlay, so
        # the geometry is established here as well; both paths are idempotent.
        self._recompute_target()
        self._place_hud()
        super().showEvent(event)

    def _place_hud(self) -> None:
        """Put the HUD over the *picture*, not over the widget.

        The overlay's own margin rule is written against the picture edge --
        cameras vignette, and an instrument hard against the frame edge sits in
        the darkest, least stable part of the image. On a letterboxed stream the
        widget edge and the picture edge are different places, and anchoring to
        the widget leaves the status cluster floating half over a black bar.
        With no picture yet the target is the whole widget, so the placeholder
        and its HUD still fill the space.
        """
        rect = self._target.toRect()
        if rect.width() <= 0 or rect.height() <= 0:
            rect = self.rect()
        if self.hud.geometry() != rect:
            self.hud.setGeometry(rect)

    def _recompute_target(self) -> None:
        content = self._presenter.content_size()
        w = float(self.width())
        h = float(self.height())
        if content.width() <= 0 or content.height() <= 0 or w <= 0.0 or h <= 0.0:
            self._target = QRectF(0.0, 0.0, w, h)
            return
        scale = min(w / content.width(), h / content.height())
        tw = content.width() * scale
        th = content.height() * scale
        self._target = QRectF((w - tw) * 0.5, (h - th) * 0.5, tw, th)

    def _announce_display_size(self) -> None:
        size = self.display_size()
        if size.width() > 0 and size.height() > 0:
            self.displaySizeChanged.emit(size.width(), size.height())

    # -- painting -----------------------------------------------------------

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        q = self._theme.q

        if not self._presenter.has_content:
            painter.fillRect(self.rect(), q.bg_base)
            self._paint_placeholder(painter)
            painter.end()
            return

        # Letterbox bars, painted only where they exist. On an exact-fit stream
        # this is two empty rectangles and costs nothing.
        target = self._target
        if target.top() > 0.0 or target.left() > 0.0:
            painter.fillRect(self.rect(), QColor(0, 0, 0))

        self._presenter.paint(painter, target)

        if self._stale:
            self._paint_stale(painter, target)
        elif self._corrupt():
            self._paint_badge(painter, target, "recovering", q.warn)

        painter.end()

    def _corrupt(self) -> bool:
        bundle = self._presenter.bundle
        return bundle is not None and bundle.corrupt

    def _paint_placeholder(self, painter: QPainter) -> None:
        q = self._theme.q
        rect = QRectF(self.rect())
        painter.setFont(self._font_placeholder)
        painter.setPen(q.text_secondary)
        painter.drawText(
            QRectF(rect.left(), rect.center().y() - 26.0, rect.width(), 30.0),
            int(Qt.AlignmentFlag.AlignCenter),
            self._placeholder,
        )
        if self._placeholder_detail:
            painter.setFont(self._font_detail)
            painter.setPen(q.text_tertiary)
            painter.drawText(
                QRectF(rect.left(), rect.center().y() + 6.0, rect.width(), 22.0),
                int(Qt.AlignmentFlag.AlignCenter),
                self._placeholder_detail,
            )

    def _paint_stale(self, painter: QPainter, target: QRectF) -> None:
        painter.fillRect(target, with_alpha(self._theme.q.bg_base, 0.55))
        self._paint_badge(painter, target, "last frame", self._theme.q.bad)

    def _paint_badge(
        self, painter: QPainter, target: QRectF, text: str, color: QColor
    ) -> None:
        painter.setFont(self._font_badge)
        width = QFontMetricsF(self._font_badge).horizontalAdvance(text) + 20.0
        chip = QRectF(target.left() + 20.0, target.top() + 20.0, width, 20.0)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(with_alpha(color, 0.22))
        painter.drawRoundedRect(chip, 4.0, 4.0)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(color)
        painter.drawText(chip, int(Qt.AlignmentFlag.AlignCenter), text)


__all__ = ["FramePresenter", "FrameProvider", "VideoView"]
