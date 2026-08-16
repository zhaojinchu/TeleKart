"""The head-up display: three edge-anchored zones over the live picture.

This module holds the drawing vocabulary the four zone widgets share, rather
than re-exporting them. Two reasons, and the first is mechanical: the overlay
composes the zones, so if the zones imported their helpers *from* the overlay
the package would have an import cycle that only breaks when someone imports a
zone first. Putting the shared primitives in the package root gives every zone
one dependency that depends on nothing back.

The second is that these primitives are the HUD's design contract, and there is
value in having them in one short file that a reviewer can read end to end:

**Scrim, not a box.** Every element sits on a rounded rectangle of 55 % black
with a one-pixel 8 %-white top edge. Pure black panels turn a driving view into
a dashboard with a letterbox; a scrim keeps the picture visible underneath while
still guaranteeing contrast when the car drives out of shade into direct sun.
The bright top edge is what stops the panel dissolving into a dark scene.

**Two numeric sizes and one label size.** ``hero`` for the speed, ``readout``
for everything else numeric, ``label`` for the small uppercase captions. A third
numeric size makes the reader work out the hierarchy instead of seeing it.

**Labels at 55 % opacity with positive tracking.** A caption at full strength
competes with its own value. Letter-spacing is what keeps eleven-pixel uppercase
legible when it is composited over moving video rather than a flat ground.

**Tabular figures everywhere**, via :func:`~telekart_app.ui.theme.fonts.numeric_font`.
A speed readout whose digits are proportionally spaced shifts sideways every
time a 1 becomes a 7, and at 60 Hz that reads as vibration.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter

from ..theme import fonts
from ..theme.tokens import THEME, Theme, with_alpha

#: Scrim opacity. Measured against the worst case this app has: a white sky at
#: midday behind a black-on-scrim readout. Below ~0.5 the digits lose their
#: edges; above ~0.65 the panel starts to read as chrome bolted over the video.
SCRIM_ALPHA = 0.55

#: The 1 px highlight along the scrim's top edge. Small enough to be felt rather
#: than seen, which is the point -- it defines the panel without drawing a box.
SCRIM_EDGE_ALPHA = 0.08

#: Captions. Any brighter and the label competes with the number it describes.
LABEL_ALPHA = 0.55

#: Absolute letter-spacing, in pixels, for the uppercase caption role.
LABEL_TRACKING = 1.2

#: Follower time constant for every fast-changing HUD number. Inside the
#: 60-80 ms band: fast enough that the readout is not lying about the car,
#: slow enough that 50 Hz telemetry noise does not strobe the digits.
SMOOTH_TAU_S = 0.070

_BLACK = QColor(0, 0, 0)
_WHITE = QColor(255, 255, 255)


def scrim_brush(alpha: float = SCRIM_ALPHA) -> QColor:
    color = QColor(_BLACK)
    color.setAlphaF(alpha)
    return color


def draw_scrim(
    painter: QPainter,
    rect: QRectF,
    radius: float,
    *,
    alpha: float = SCRIM_ALPHA,
    edge: bool = True,
) -> None:
    """Fill the standard HUD panel ground into ``rect``.

    The top edge is stroked as a straight hairline inset by half a pixel rather
    than as a rounded outline: an outline all the way round reads as a border,
    and a border is a widget. A highlight only along the top reads as light
    falling on a raised surface, which is what the panel is meant to be.
    """
    if rect.width() <= 0.0 or rect.height() <= 0.0:
        return
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(scrim_brush(alpha))
    painter.drawRoundedRect(rect, radius, radius)
    painter.setBrush(Qt.BrushStyle.NoBrush)
    if not edge:
        return
    highlight = QColor(_WHITE)
    highlight.setAlphaF(SCRIM_EDGE_ALPHA)
    painter.setPen(highlight)
    inset = min(radius, rect.width() * 0.5)
    y = rect.top() + 0.5
    painter.drawLine(
        int(rect.left() + inset), int(y), int(rect.right() - inset), int(y)
    )


def label_font(theme: Theme = THEME) -> QFont:
    """The one caption role: uppercase, tracked, eleven pixels."""
    font = fonts.ui_font(theme.type.label, theme.weight.semibold, theme=theme)
    font.setCapitalization(QFont.Capitalization.AllUppercase)
    font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, LABEL_TRACKING)
    return font


def micro_label_font(theme: Theme = THEME) -> QFont:
    """Ten pixels, for captions inside an already-small panel."""
    font = fonts.ui_font(theme.type.micro, theme.weight.semibold, theme=theme)
    font.setCapitalization(QFont.Capitalization.AllUppercase)
    font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, LABEL_TRACKING)
    return font


def hero_font(theme: Theme = THEME) -> QFont:
    """The speed number, and nothing else on the HUD."""
    return fonts.numeric_font(theme.type.hero, theme.weight.bold, theme=theme)


def readout_font(theme: Theme = THEME, *, size: int | None = None) -> QFont:
    """Every HUD number that is not the speed."""
    return fonts.numeric_font(
        theme.type.readout if size is None else size, theme.weight.medium, theme=theme
    )


def label_color(theme: Theme = THEME) -> QColor:
    return with_alpha(theme.q.text_primary, LABEL_ALPHA)


__all__ = [
    "LABEL_ALPHA",
    "LABEL_TRACKING",
    "SCRIM_ALPHA",
    "SCRIM_EDGE_ALPHA",
    "SMOOTH_TAU_S",
    "draw_scrim",
    "hero_font",
    "label_color",
    "label_font",
    "micro_label_font",
    "readout_font",
    "scrim_brush",
]
