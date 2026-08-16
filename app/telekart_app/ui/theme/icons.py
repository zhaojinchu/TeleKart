"""Vector icons, generated in Python and rasterised through QSvgRenderer.

The geometry lives in this file as SVG path data rather than in a directory of
``.svg`` assets. Three reasons, in order of how much they cost when ignored:

* An icon has to be drawn in four different colours (normal, hover, active,
  disabled) and the sane way to do that is to substitute the stroke colour
  before rasterising. A file on disk means either shipping four copies of every
  glyph or post-processing pixels, and post-processing pixels destroys the
  antialiasing at the edges.
* Icon files are the classic thing that goes missing from a wheel because
  someone forgot a ``package-data`` line, and it fails at runtime, on the user's
  machine, as a blank toolbar.
* Some of these shapes are genuinely parametric -- the gear, the signal bars --
  and computing them is shorter and more correct than hand-fitting bezier
  handles.

Everything is a 24x24 viewBox, stroked, round caps and joins, so glyphs sit
together on a toolbar without one looking heavier than its neighbours.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap

from .tokens import THEME, Theme, with_alpha

try:
    from PySide6.QtSvg import QSvgRenderer

    _HAVE_SVG = True
except ImportError:  # pragma: no cover - QtSvg ships with PySide6-Essentials
    _HAVE_SVG = False


# --------------------------------------------------------------------------
# Parametric path generators
# --------------------------------------------------------------------------


def _polar(cx: float, cy: float, r: float, deg: float) -> tuple[float, float]:
    a = math.radians(deg)
    return cx + r * math.cos(a), cy + r * math.sin(a)


def _path(data: str) -> str:
    """Wrap generated path data in a real ``<path>`` element.

    Every generator below returns *markup*, not a bare ``d`` attribute. Bare
    path data spliced into the document is not an element, so the renderer drops
    it silently and the glyph comes out as whatever literal elements happened to
    sit next to it -- a lone dot for ``link``, a lone circle for ``settings``,
    nothing at all for ``signal``. Returning a finished element makes that
    mistake unrepresentable, which is the only reason this one-line function
    exists.
    """
    return '<path d="%s"/>' % (data,)


def _radial_teeth(count: int, r_inner: float, r_outer: float, offset: float = 0.0) -> str:
    """``count`` evenly spaced radial spokes, for gear and sun glyphs."""
    parts: list[str] = []
    for i in range(count):
        deg = offset + (360.0 / count) * i
        x0, y0 = _polar(12.0, 12.0, r_inner, deg)
        x1, y1 = _polar(12.0, 12.0, r_outer, deg)
        parts.append("M%.2f %.2fL%.2f %.2f" % (x0, y0, x1, y1))
    return _path("".join(parts))


def _signal_bars(count: int = 4) -> str:
    """Ascending bars, bottom-aligned, evenly pitched."""
    parts: list[str] = []
    base = 20.0
    pitch = 5.0
    for i in range(count):
        x = 4.0 + i * pitch
        top = base - 4.0 - i * 4.0
        parts.append("M%.2f %.2fV%.2f" % (x, base, top))
    return _path("".join(parts))


def _arc(cx: float, cy: float, r: float, start_deg: float, end_deg: float) -> str:
    """A single SVG arc segment. Used for gauge and steering glyphs."""
    x0, y0 = _polar(cx, cy, r, start_deg)
    x1, y1 = _polar(cx, cy, r, end_deg)
    large = 1 if abs(end_deg - start_deg) > 180.0 else 0
    sweep = 1 if end_deg > start_deg else 0
    return _path(
        "M%.2f %.2fA%.2f %.2f 0 %d %d %.2f %.2f"
        % (x0, y0, r, r, large, sweep, x1, y1)
    )


# --------------------------------------------------------------------------
# The set
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class IconSpec:
    body: str
    stroke_width: float = 1.8
    #: Solid glyphs (record dot, play triangle) fill instead of stroking.
    filled: bool = False


def _spec(body: str, width: float = 1.8, filled: bool = False) -> IconSpec:
    return IconSpec(body, width, filled)


_ICONS: dict[str, IconSpec] = {
    # -- navigation rail -------------------------------------------------
    "drive": _spec(
        '<circle cx="12" cy="12" r="9"/><circle cx="12" cy="12" r="3.1"/>'
        '<path d="M12 15.1V21M9.35 10.45 4.2 7.5M14.65 10.45 19.8 7.5"/>'
    ),
    "telemetry": _spec('<path d="M2 12h3.4l2.6-7.4 4 14.8 2.6-7.4H22"/>'),
    "tune": _spec(
        '<path d="M5 21v-6M5 11V3M12 21v-9M12 8V3M19 21v-4M19 13V3'
        'M2.5 13h5M9.5 6h5M16.5 15h5"/>'
    ),
    "calibrate": _spec(
        '<circle cx="12" cy="12" r="8.5"/><circle cx="12" cy="12" r="3.6"/>'
        '<path d="M12 1.5v3.4M12 19.1v3.4M1.5 12h3.4M19.1 12h3.4"/>'
    ),
    "video": _spec(
        '<rect x="2" y="6" width="14" height="12" rx="2.2"/>'
        '<path d="m16 10.5 6-3.6v10.2l-6-3.6z"/>'
    ),
    "track": _spec(
        '<path d="m2 6.6 6.6-3 6.8 3 6.6-3v13.8l-6.6 3-6.8-3-6.6 3z'
        'M8.6 3.6v13.8M15.4 6.6v13.8"/>'
    ),
    "session": _spec(
        '<path d="M8.5 6H21M8.5 12H21M8.5 18H21"/>'
        '<circle cx="3.6" cy="6" r="1.1"/><circle cx="3.6" cy="12" r="1.1"/>'
        '<circle cx="3.6" cy="18" r="1.1"/>'
    ),
    "settings": _spec(
        '<circle cx="12" cy="12" r="3.4"/>' + _radial_teeth(8, 6.4, 9.0) + ""
    ),
    # -- brand mark ------------------------------------------------------
    # Double chevron over a baseline: motion, and legible at 20 px where
    # anything resembling an actual vehicle turns to mush.
    "mark": _spec(
        '<path d="M4.6 4.8 11.4 12l-6.8 7.2M12.6 4.8 19.4 12l-6.8 7.2"/>',
        2.4,
    ),
    # -- status ----------------------------------------------------------
    "link": _spec(
        _arc(12, 17, 9.4, 200, 340)
        + _arc(12, 17, 5.6, 205, 335)
        + '<circle cx="12" cy="17.4" r="1.3" fill="currentColor" stroke="none"/>'
    ),
    "signal": _spec(_signal_bars(4), 2.0),
    "battery": _spec(
        '<rect x="1.8" y="7" width="17" height="10" rx="2"/>'
        '<path d="M21.6 10.6v2.8"/>'
    ),
    "cpu": _spec(
        '<rect x="6.5" y="6.5" width="11" height="11" rx="1.8"/>'
        '<rect x="10" y="10" width="4" height="4" rx="0.8"/>'
        '<path d="M9.4 2.4v4.1M14.6 2.4v4.1M9.4 17.5v4.1M14.6 17.5v4.1'
        'M2.4 9.4h4.1M2.4 14.6h4.1M17.5 9.4h4.1M17.5 14.6h4.1"/>'
    ),
    "thermometer": _spec(
        '<path d="M13.8 14.2V4.6a2.4 2.4 0 0 0-4.8 0v9.6a4.4 4.4 0 1 0 4.8 0z"/>'
    ),
    "warning": _spec(
        '<path d="M12 3.4 22 20.6H2z"/><path d="M12 9.6v4.6M12 17.6h.01"/>'
    ),
    "fault": _spec(
        '<circle cx="12" cy="12" r="9"/><path d="M12 7.2v5.4M12 16.2h.01"/>'
    ),
    "check": _spec('<path d="m4.4 12.6 5 5L19.6 7"/>', 2.1),
    "info": _spec(
        '<circle cx="12" cy="12" r="9"/><path d="M12 16.4v-5M12 7.8h.01"/>'
    ),
    "gauge": _spec(
        _arc(12, 14.5, 8.6, 180, 360)
        + '<path d="M12 14.5 16.6 9.6"/>'
        + '<circle cx="12" cy="14.5" r="1.5" fill="currentColor" stroke="none"/>'
    ),
    "stopwatch": _spec(
        '<circle cx="12" cy="13.6" r="7.8"/>'
        '<path d="M12 9.6v4h2.8M9.4 1.9h5.2M19 6.6l1.6-1.6"/>'
    ),
    "gamepad": _spec(
        '<rect x="1.8" y="7.2" width="20.4" height="10.6" rx="4.4"/>'
        '<path d="M6.4 10.8v3.4M4.7 12.5h3.4"/>'
        '<circle cx="16.2" cy="11.5" r="1.05" fill="currentColor" stroke="none"/>'
        '<circle cx="18.6" cy="14" r="1.05" fill="currentColor" stroke="none"/>'
    ),
    "flag": _spec(
        '<path d="M4.6 15.2s1.1-1 4-1 5 2 8 2 3.4-1 3.4-1V3.4s-1 1-3.4 1'
        '-5-2-8-2-4 1-4 1z"/><path d="M4.6 21.6V2.4"/>'
    ),
    "eye": _spec(
        '<path d="M1.6 12S5.4 5.2 12 5.2 22.4 12 22.4 12 18.6 18.8 12 18.8 1.6 12 1.6 12z"/>'
        '<circle cx="12" cy="12" r="3.1"/>'
    ),
    "lock": _spec(
        '<rect x="4.4" y="10.6" width="15.2" height="10.4" rx="2.2"/>'
        '<path d="M8 10.6V7.4a4 4 0 0 1 8 0v3.2"/>'
    ),
    "unlock": _spec(
        '<rect x="4.4" y="10.6" width="15.2" height="10.4" rx="2.2"/>'
        '<path d="M8 10.6V7.4a4 4 0 0 1 7.6-1.8"/>'
    ),
    # -- actions ---------------------------------------------------------
    "arm": _spec('<path d="M12 2.6v9.2"/>' + _arc(12, 13.4, 8.2, 235, 485)),
    "estop": _spec(
        '<path d="M8.3 2.6h7.4l5.7 5.7v7.4l-5.7 5.7H8.3l-5.7-5.7V8.3z"/>'
        '<path d="M8.4 12h7.2"/>',
        2.0,
    ),
    "record": _spec('<circle cx="12" cy="12" r="6.4" fill="currentColor"/>', 0.0, True),
    "play": _spec('<path d="M7.6 4.8 19 12 7.6 19.2z" fill="currentColor"/>', 0.0, True),
    "pause": _spec(
        '<path d="M7.4 4.8h3.2v14.4H7.4zM13.4 4.8h3.2v14.4h-3.2z" fill="currentColor"/>',
        0.0,
        True,
    ),
    "stop": _spec(
        '<rect x="5.6" y="5.6" width="12.8" height="12.8" rx="1.6" fill="currentColor"/>',
        0.0,
        True,
    ),
    "refresh": _spec(
        _arc(12, 12, 8.2, 300, 620) + '<path d="M20.2 5.4v4.4h-4.4"/>'
    ),
    "save": _spec(
        '<path d="M4.6 3.4h11.2L20.4 8v12.6H4.6z"/>'
        '<path d="M8.2 3.4v5.4h6.6V3.4M8.2 20.6v-6.2h7.6v6.2"/>'
    ),
    "folder": _spec(
        '<path d="M2.6 6.4a1.8 1.8 0 0 1 1.8-1.8h4.3l2.2 2.6h8.7a1.8 1.8 0 0 1 1.8 1.8'
        'v9.4a1.8 1.8 0 0 1-1.8 1.8H4.4a1.8 1.8 0 0 1-1.8-1.8z"/>'
    ),
    "trash": _spec(
        '<path d="M3.4 6.2h17.2M9 6.2V4.4a1.4 1.4 0 0 1 1.4-1.4h3.2A1.4 1.4 0 0 1 15 4.4v1.8'
        'M5.6 6.2l1 13.2a1.6 1.6 0 0 0 1.6 1.4h7.6a1.6 1.6 0 0 0 1.6-1.4l1-13.2"/>'
    ),
    "plus": _spec('<path d="M12 4.6v14.8M4.6 12h14.8"/>', 2.0),
    "minus": _spec('<path d="M4.6 12h14.8"/>', 2.0),
    "close": _spec('<path d="M5.6 5.6 18.4 18.4M18.4 5.6 5.6 18.4"/>', 2.0),
    "menu": _spec('<path d="M3.4 6.6h17.2M3.4 12h17.2M3.4 17.4h17.2"/>', 2.0),
    "search": _spec('<circle cx="10.8" cy="10.8" r="7"/><path d="m15.9 15.9 5 5"/>'),
    "reset": _spec(
        _arc(12, 12, 8.2, 240, -80) + '<path d="M3.8 5.4v4.4h4.4"/>'
    ),
    # -- chevrons --------------------------------------------------------
    "chevron-left": _spec('<path d="M15 4.8 7.6 12 15 19.2"/>', 2.0),
    "chevron-right": _spec('<path d="M9 4.8 16.4 12 9 19.2"/>', 2.0),
    "chevron-up": _spec('<path d="M4.8 15 12 7.6 19.2 15"/>', 2.0),
    "chevron-down": _spec('<path d="M4.8 9 12 16.4 19.2 9"/>', 2.0),
}

ICON_NAMES: tuple[str, ...] = tuple(sorted(_ICONS))

_SVG_TEMPLATE = (
    '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" '
    'viewBox="0 0 24 24" fill="none" stroke="%s" stroke-width="%.2f" '
    'stroke-linecap="round" stroke-linejoin="round" color="%s">%s</svg>'
)

#: (name, argb, pixel size) -> rendered pixmap. Icons are drawn constantly by
#: the navigation rail and every toolbar; re-rasterising SVG on each repaint is
#: the sort of thing that shows up as a warm laptop and nothing else.
_CACHE: dict[tuple[str, int, int], QPixmap] = {}


# --------------------------------------------------------------------------
# Rasterisation
# --------------------------------------------------------------------------


def svg_source(name: str, color: QColor) -> str:
    """The full SVG document for one icon at one colour.

    Public because the ``QSvgWidget``-based splash and the About page want the
    mark at an arbitrary size without going through a pixmap.
    """
    spec = _ICONS.get(name)
    if spec is None:
        raise KeyError("unknown icon %r" % (name,))
    hexcolor = color.name(QColor.NameFormat.HexRgb)
    return _SVG_TEMPLATE % (
        "none" if spec.filled else hexcolor,
        spec.stroke_width,
        hexcolor,
        spec.body,
    )


def icon_pixmap(
    name: str,
    color: QColor,
    size: int = 20,
    dpr: float = 1.0,
) -> QPixmap:
    """A device-pixel-ratio-correct pixmap of one icon.

    Painted widgets that draw their own icons (the navigation rail) call this
    directly rather than going through ``QIcon``, because they need to blend
    between two colours during the hover transition and ``QIcon`` only knows
    about discrete modes.
    """
    px = max(1, int(round(size * dpr)))
    key = (name, color.rgba(), px)
    cached = _CACHE.get(key)
    if cached is not None:
        return cached

    pixmap = QPixmap(px, px)
    pixmap.setDevicePixelRatio(dpr)
    pixmap.fill(Qt.GlobalColor.transparent)

    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
    if _HAVE_SVG:
        renderer = QSvgRenderer(svg_source(name, color).encode("utf-8"))
        renderer.render(painter, QRectF(0.0, 0.0, float(px), float(px)))
    else:
        _draw_placeholder(painter, px, color)
    painter.end()

    _CACHE[key] = pixmap
    return pixmap


def icon(
    name: str,
    *,
    color: QColor | None = None,
    active: QColor | None = None,
    selected: QColor | None = None,
    disabled: QColor | None = None,
    size: int = 20,
    theme: Theme = THEME,
) -> QIcon:
    """A four-state ``QIcon``.

    Both 1x and 2x pixmaps are attached so the same icon stays crisp when the
    window is dragged from a Retina display to an external monitor -- Qt picks
    by pixel size at paint time, and an icon that only exists at 1x turns to
    mush the moment it is upscaled.
    """
    normal = color if color is not None else theme.q.text_secondary
    hover = active if active is not None else theme.q.text_primary
    on = selected if selected is not None else theme.q.accent
    off = disabled if disabled is not None else with_alpha(theme.q.text_tertiary, 0.55)

    result = QIcon()
    for mode, tint in (
        (QIcon.Mode.Normal, normal),
        (QIcon.Mode.Active, hover),
        (QIcon.Mode.Selected, on),
        (QIcon.Mode.Disabled, off),
    ):
        for dpr in (1.0, 2.0):
            result.addPixmap(icon_pixmap(name, tint, size, dpr), mode, QIcon.State.Off)
    return result


def state_icon(name: str, color: QColor, size: int = 20) -> QIcon:
    """A single-colour icon, for cases where the colour *is* the information."""
    result = QIcon()
    for dpr in (1.0, 2.0):
        pixmap = icon_pixmap(name, color, size, dpr)
        for mode in (
            QIcon.Mode.Normal,
            QIcon.Mode.Active,
            QIcon.Mode.Selected,
            QIcon.Mode.Disabled,
        ):
            result.addPixmap(pixmap, mode, QIcon.State.Off)
    return result


def icon_size(size: int = 20) -> QSize:
    return QSize(size, size)


def clear_cache() -> None:
    """Drop rasterised icons. Called if the theme is ever swapped at runtime."""
    _CACHE.clear()


#: Arrow glyphs the stylesheet needs as files. Qt stylesheets can only place a
#: sub-control arrow through ``image: url(...)``, and Qt's url() resolves
#: through QFile -- no data: URIs -- so combo boxes, spin boxes, tree branches
#: and submenu indicators need actual images on disk. Generating them from the
#: same SVG source as every other icon is what keeps them the same weight and
#: the same colour as the rest of the set.
ARROW_EXPORTS: tuple[tuple[str, str], ...] = (
    ("down", "chevron-down"),
    ("up", "chevron-up"),
    ("right", "chevron-right"),
)

#: Rendered at 12 logical px: large enough that a 2 px stroke in a 24 unit
#: viewBox does not disappear, small enough to sit inside a 24 px control.
ARROW_SIZE = 12


def export_stylesheet_arrows(
    directory: str,
    variants: dict[str, QColor],
    size: int = ARROW_SIZE,
) -> bool:
    """Write ``<name><variant>.png`` and its ``@2x`` twin for each arrow.

    Returns False rather than raising if the directory is unwritable: an
    application that will not start because it could not cache an 800-byte
    arrow is worse than one with plain arrows.
    """
    try:
        os.makedirs(directory, exist_ok=True)
    except OSError:
        return False

    for name, icon_name in ARROW_EXPORTS:
        for suffix, color in variants.items():
            for dpr, tag in ((1.0, ""), (2.0, "@2x")):
                pixmap = icon_pixmap(icon_name, color, size, dpr)
                path = os.path.join(directory, name + suffix + tag + ".png")
                if not pixmap.save(path, "PNG"):
                    return False
    return True


#: Check marks sit inside a 16 px indicator, so they render a little smaller
#: than the arrows to leave a margin against the indicator's border.
INDICATOR_SIZE = 10

#: (filename, icon) for the marks drawn inside a QCheckBox indicator.
INDICATOR_EXPORTS = (
    ("check", "check"),
    ("indeterminate", "minus"),
)


def export_stylesheet_indicators(
    directory: str,
    color: QColor,
    size: int = INDICATOR_SIZE,
) -> bool:
    """Write the check and indeterminate marks for QCheckBox indicators.

    Qt draws no native mark once ``::indicator`` carries any QSS of its own, so
    a styled checkbox without one of these renders as a plain filled square --
    checked and unchecked distinguishable only by colour. For a boolean like
    "closed-loop speed control" that ambiguity is worth avoiding.

    Same failure policy as the arrows: return False rather than raise.
    """
    try:
        os.makedirs(directory, exist_ok=True)
    except OSError:
        return False

    for name, icon_name in INDICATOR_EXPORTS:
        for dpr, tag in ((1.0, ""), (2.0, "@2x")):
            pixmap = icon_pixmap(icon_name, color, size, dpr)
            if not pixmap.save(os.path.join(directory, name + tag + ".png"), "PNG"):
                return False
    return True


def _draw_placeholder(painter: QPainter, px: int, color: QColor) -> None:
    """Visible-but-obviously-wrong fallback if QtSvg is unavailable.

    A blank square would be indistinguishable from "this button has no icon",
    which is the failure mode that wastes an afternoon.
    """
    pen = QPen(color)
    pen.setWidthF(max(1.0, px / 14.0))
    painter.setPen(pen)
    inset = px * 0.18
    painter.drawRoundedRect(
        QRectF(inset, inset, px - 2 * inset, px - 2 * inset),
        px * 0.12,
        px * 0.12,
    )
    painter.drawLine(int(inset), int(inset), int(px - inset), int(px - inset))


__all__ = [
    "ARROW_EXPORTS",
    "ARROW_SIZE",
    "ICON_NAMES",
    "IconSpec",
    "clear_cache",
    "export_stylesheet_arrows",
    "icon",
    "icon_pixmap",
    "icon_size",
    "state_icon",
    "svg_source",
]
