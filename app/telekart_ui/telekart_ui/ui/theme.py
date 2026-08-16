"""Colours, fonts, and the small stylesheet the connect panel needs.

The previous station carried 2,813 lines here: 496 of design tokens, an 880-line
QSS template covering ~180 selectors, 515 lines generating 41 SVG icons, and 323
of font-role resolution. That existed because it had five screens of ordinary Qt
widgets to style and a rail full of icons to draw.

This station has one screen. Almost everything on it is painted directly onto
the video, and the only stock Qt widgets in the app are three fields and two
buttons on the connect panel. So: one palette, three fonts, ~40 lines of QSS.

**No icons at all.** That is deliberate beyond mere economy. The icon renderer
scaled its painter by ``devicePixelRatio`` while drawing onto a pixmap that
already carried that ratio, so every icon was wrong on every Retina display --
correct at dpr 1, a magnified corner at dpr 2, blank at dpr 3. Text labels and
painted shapes cannot have that bug.
"""

from __future__ import annotations

from dataclasses import dataclass

from PySide6.QtGui import QColor, QFont

# --------------------------------------------------------------------------
# Palette
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Palette:
    """The complete colour vocabulary.

    Anything that looks like a fourteenth colour -- a hover tint, a scrim, a
    disabled label -- is *derived* from these by :func:`mix` or
    :func:`with_alpha`. Deriving rather than adding is what stops a new panel
    inventing its own slightly-different grey.

    ``accent`` is the driver's own commanded input, focus, and "the active
    thing"; never a status. ``good``/``warn``/``bad`` are health, in that order
    of severity. ``cyan`` is a *measured* value as opposed to a commanded one --
    the steering arc uses accent for what the app sent and cyan for what the
    servo actually reports, and the gap between them is the story.
    """

    bg_base: str = "#0B0D10"
    bg_raised: str = "#14171C"
    bg_overlay: str = "#1B1F26"
    line: str = "#262B33"
    text_primary: str = "#E8EBF0"
    text_secondary: str = "#98A1AE"
    text_tertiary: str = "#5F6875"
    accent: str = "#FF4D2E"
    good: str = "#2FD07A"
    warn: str = "#FFB020"
    bad: str = "#FF4757"
    cyan: str = "#33C9FF"


PALETTE = Palette()


class Colors:
    """``QColor`` twins, built once at import.

    Instruments read these in ``paintEvent``. Constructing a QColor from a hex
    string re-parses it every time, and a HUD at 60 Hz does that thousands of
    times a second for values that never change.
    """

    bg_base = QColor(PALETTE.bg_base)
    bg_raised = QColor(PALETTE.bg_raised)
    bg_overlay = QColor(PALETTE.bg_overlay)
    line = QColor(PALETTE.line)
    text_primary = QColor(PALETTE.text_primary)
    text_secondary = QColor(PALETTE.text_secondary)
    text_tertiary = QColor(PALETTE.text_tertiary)
    accent = QColor(PALETTE.accent)
    good = QColor(PALETTE.good)
    warn = QColor(PALETTE.warn)
    bad = QColor(PALETTE.bad)
    cyan = QColor(PALETTE.cyan)


C = Colors


def with_alpha(color: QColor, alpha: float) -> QColor:
    out = QColor(color)
    out.setAlphaF(max(0.0, min(1.0, alpha)))
    return out


def mix(a: QColor, b: QColor, t: float) -> QColor:
    """Blend, for derived shades. ``t=0`` is ``a``, ``t=1`` is ``b``."""
    t = max(0.0, min(1.0, t))
    return QColor(
        round(a.red() + (b.red() - a.red()) * t),
        round(a.green() + (b.green() - a.green()) * t),
        round(a.blue() + (b.blue() - a.blue()) * t),
    )


# --------------------------------------------------------------------------
# State colours
# --------------------------------------------------------------------------

#: One place that decides what colour a vehicle state is. The HUD badge, the
#: fault banner and the connect panel all read it, so they cannot disagree about
#: whether E-STOP is red.
STATE_COLORS: dict[str, QColor] = {
    "Booting": C.text_secondary,
    "Safe": C.text_secondary,
    "Armed": C.good,
    "Failsafe": C.warn,
    "E-STOP": C.bad,
    "Fault": C.bad,
    "Unknown": C.text_tertiary,
}


def state_color(label: str) -> QColor:
    return STATE_COLORS.get(label, C.text_tertiary)


def health_color(age: float, *, warn: float, down: float) -> QColor:
    """Green / amber / red from the age of the last packet on a channel.

    Age rather than a socket flag, for the reason written up in
    ``net/link_manager.py``: a socket that is up and delivering nothing is not a
    healthy channel, and a reconnecting one flaps several times a second. Age is
    monotone, so an indicator driven by it cannot chatter.
    """
    if age >= down:
        return C.bad
    if age >= warn:
        return C.warn
    return C.good


# --------------------------------------------------------------------------
# Fonts
# --------------------------------------------------------------------------

#: Sizes, in points. Five, and no others.
SIZE_HERO = 64  # the speed number
SIZE_TITLE = 18
SIZE_BODY = 13
SIZE_LABEL = 11
SIZE_MICRO = 9


#: Named explicitly for the same reason as the mono list below: a bare QFont()
#: resolves to "Sans Serif", which does not exist on macOS, so Qt spends ~90 ms
#: populating alias tables on first use before substituting anyway.
_UI_FAMILIES = ("SF Pro Text", "Helvetica Neue", "Inter", "Segoe UI", "DejaVu Sans")


def ui_font(size: int, weight: QFont.Weight = QFont.Weight.Normal) -> QFont:
    font = QFont()
    font.setFamilies(list(_UI_FAMILIES))
    font.setPointSize(size)
    font.setWeight(weight)
    return font


def numeric_font(size: int, weight: QFont.Weight = QFont.Weight.DemiBold) -> QFont:
    """For anything whose digits change while the driver is looking at it.

    Tabular figures are the point: with proportional digits a speed readout
    going 99 -> 100 -> 99 jitters horizontally, and the eye reads the movement
    as the value changing more than it did.
    """
    font = ui_font(size, weight)
    font.setStyleHint(QFont.StyleHint.SansSerif)
    font.setStyleStrategy(QFont.StyleStrategy.PreferAntialias)
    # Tabular figures where the toolkit offers them. setFeature arrived in
    # Qt 6.7; guarded rather than assumed, because a missing font feature must
    # degrade to slightly-jittery digits and never to a crash on launch.
    setter = getattr(font, "setFeature", None)
    if setter is not None:
        try:
            setter("tnum", 1)
        except (TypeError, ValueError):
            pass
    return font


#: Named explicitly rather than taken from ``QFontDatabase.systemFont``, which
#: answers "Monospace" -- a family that does not exist on macOS, so Qt spends
#: ~90 ms populating alias tables on first use and then substitutes anyway.
#: Menlo before SF Mono: SF Mono ships with macOS but is not registered as a
#: user-selectable family, so asking for it costs an alias-table population
#: (~75 ms on first use) and then substitutes anyway.
_MONO_FAMILIES = ("Menlo", "SF Mono", "Monaco", "DejaVu Sans Mono", "Courier New")


def mono_font(size: int = SIZE_MICRO) -> QFont:
    """Fixed pitch, for the detail row.

    Fixed pitch there and nowhere else: it is the one readout whose columns must
    not shuffle sideways as the digits change, because it is read by comparing
    successive glances rather than by looking once.
    """
    font = QFont()
    font.setFamilies(list(_MONO_FAMILIES))
    font.setStyleHint(QFont.StyleHint.Monospace)
    font.setFixedPitch(True)
    font.setPointSize(size)
    return font


# --------------------------------------------------------------------------
# Stylesheet
# --------------------------------------------------------------------------

#: Only the connect panel uses stock Qt widgets, so this covers exactly those.
#: If it grows past a screenful, something has gone wrong with the premise.
QSS = f"""
QWidget#ConnectPanel {{
    background: {PALETTE.bg_raised};
    border: 1px solid {PALETTE.line};
    border-radius: 10px;
}}
QLabel {{
    color: {PALETTE.text_primary};
    background: transparent;
}}
QLabel[variant="title"] {{
    font-size: {SIZE_TITLE}pt;
    font-weight: 600;
}}
QLabel[variant="caption"] {{
    color: {PALETTE.text_secondary};
    font-size: {SIZE_LABEL}pt;
}}
QLabel[variant="error"] {{
    color: {PALETTE.bad};
    font-size: {SIZE_LABEL}pt;
}}
QLineEdit {{
    background: {PALETTE.bg_base};
    border: 1px solid {PALETTE.line};
    border-radius: 6px;
    padding: 7px 9px;
    color: {PALETTE.text_primary};
    selection-background-color: {PALETTE.accent};
}}
QLineEdit:focus {{
    border-color: {PALETTE.accent};
}}
QPushButton {{
    background: {PALETTE.bg_overlay};
    border: 1px solid {PALETTE.line};
    border-radius: 6px;
    padding: 8px 16px;
    color: {PALETTE.text_primary};
}}
QPushButton:hover {{ border-color: {PALETTE.text_tertiary}; }}
QPushButton:default, QPushButton[variant="primary"] {{
    background: {PALETTE.accent};
    border-color: {PALETTE.accent};
    color: #FFFFFF;
    font-weight: 600;
}}
QPushButton:disabled {{
    color: {PALETTE.text_tertiary};
    border-color: {PALETTE.line};
    background: {PALETTE.bg_raised};
}}
QCheckBox {{ color: {PALETTE.text_secondary}; }}
"""


def apply_theme(app: object) -> None:
    """Fusion plus the stylesheet, before the first widget is constructed.

    Fusion explicitly: macOS's native style ignores large parts of QSS, so the
    panel would come out half-styled. And before any widget exists, because Qt
    polishes widgets as they are built -- applying it later costs a second full
    polish of the whole tree.
    """
    app.setStyle("Fusion")  # type: ignore[attr-defined]
    app.setStyleSheet(QSS)  # type: ignore[attr-defined]


__all__ = [
    "PALETTE",
    "Palette",
    "Colors",
    "C",
    "QSS",
    "SIZE_HERO",
    "SIZE_TITLE",
    "SIZE_BODY",
    "SIZE_LABEL",
    "SIZE_MICRO",
    "apply_theme",
    "health_color",
    "mix",
    "mono_font",
    "numeric_font",
    "state_color",
    "ui_font",
    "with_alpha",
]
