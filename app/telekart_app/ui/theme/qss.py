"""Stylesheet assembly and application-wide palette.

``telekart.qss`` is a template full of ``{{token}}`` placeholders; this module
resolves them against the one :class:`~telekart_app.ui.theme.tokens.Theme` that
the painted instruments also read, so a hover tint on a QPushButton and a hover
tint drawn by QPainter are literally the same number.

Two things here are load-bearing and easy to get wrong:

* ``setStyle("Fusion")`` must happen **before** anything else. The macOS style
  ignores large parts of QSS -- scrollbars, item views, spin boxes -- and the
  result is an application that looks styled on Linux and half-styled on the
  machine it is actually being developed on.
* A full ``QPalette`` is set as well as the stylesheet, because QSS does not
  reach scrollbar arrows drawn by the base style, item-view selection in
  non-styled views, disabled text, or the colours Qt hands to a native file
  dialog.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass

from PySide6.QtCore import QDir, QStandardPaths
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

from . import fonts, icons
from .tokens import THEME, Theme, mix, mix_hex, rgba_css

_BLACK = "#000000"
_WHITE = "#FFFFFF"

_PLACEHOLDER = re.compile(r"\{\{\s*([A-Za-z0-9_.\-]+)\s*\}\}")

_QSS_FILENAME = "telekart.qss"

#: Qt search-path prefix for the generated arrow images; the stylesheet refers
#: to them as ``url(tkarrow:down.png)``.
_ARROW_PREFIX = "tkarrow"

#: None until the first attempt, then True/False. Cached because the export
#: writes six files and the token table is rebuilt for the palette as well as
#: for the stylesheet.
_arrows_ready: bool | None = None


@dataclass(frozen=True, slots=True)
class AppliedTheme:
    """What ``apply_theme`` actually did, for the diagnostics panel."""

    style: str
    font_roles: fonts.FontRoles
    stylesheet_length: int


# --------------------------------------------------------------------------
# Token table
# --------------------------------------------------------------------------


def qss_tokens(theme: Theme = THEME) -> dict[str, str]:
    """Every name the stylesheet may reference, resolved to a literal.

    Derived values are computed here rather than written into the ``.qss`` so
    that a painted widget can ask for the same derived value -- ``btn.bg``, say
    -- and get the identical colour without duplicating the arithmetic.

    Hover and pressed states are *blends* rather than translucent overlays.
    Every control in this application sits on an opaque ground, so a blend is
    pixel-identical, and it sidesteps Qt's stylesheet colour parser, which
    reads the fourth argument of ``rgba()`` as an integer and silently renders
    ``rgba(r, g, b, 0.6)`` as fully transparent.
    """
    c = theme.color
    tokens: dict[str, str] = {}

    for name in (
        "bg_base",
        "bg_raised",
        "bg_overlay",
        "line",
        "text_primary",
        "text_secondary",
        "text_tertiary",
        "accent",
        "good",
        "warn",
        "bad",
        "purple",
        "cyan",
    ):
        tokens["color." + name] = getattr(c, name)

    for name in ("xs", "sm", "md", "lg", "xl", "xxl"):
        tokens["space." + name] = str(getattr(theme.space, name))

    for name in ("control", "card", "overlay"):
        tokens["radius." + name] = str(getattr(theme.radius, name))

    for name in (
        "hero",
        "readout",
        "display",
        "title",
        "heading",
        "body",
        "label",
        "micro",
    ):
        tokens["font." + name] = str(getattr(theme.type, name))

    resolved = fonts.roles()
    tokens["family.ui"] = resolved.ui
    tokens["family.numeric"] = resolved.numeric
    tokens["family.mono"] = resolved.mono

    # -- text ---------------------------------------------------------------
    tokens["text.disabled"] = qss_disabled_text(theme)

    # -- neutral controls ---------------------------------------------------
    tokens["btn.bg"] = mix_hex(c.bg_raised, c.text_primary, 0.07)
    tokens["btn.bg_hover"] = mix_hex(c.bg_raised, c.text_primary, 0.14)
    tokens["btn.bg_pressed"] = mix_hex(c.bg_raised, _BLACK, 0.30)
    tokens["btn.bg_disabled"] = mix_hex(c.bg_raised, c.bg_base, 0.55)
    tokens["btn.border"] = c.line
    tokens["btn.border_hover"] = mix_hex(c.line, c.text_secondary, 0.40)

    # -- accent -------------------------------------------------------------
    tokens["accent.bg"] = c.accent
    tokens["accent.bg_hover"] = mix_hex(c.accent, _WHITE, 0.14)
    tokens["accent.bg_pressed"] = mix_hex(c.accent, _BLACK, 0.20)
    tokens["accent.soft"] = mix_hex(c.bg_raised, c.accent, 0.18)
    tokens["accent.soft_hover"] = mix_hex(c.bg_raised, c.accent, 0.28)
    # Dark ink on accent: #FF4D2E against #0B0D10 is ~5.9:1, against white only
    # ~3.2:1, so the darker option is the accessible one here.
    tokens["accent.on"] = c.bg_base

    # -- destructive --------------------------------------------------------
    tokens["danger.bg"] = mix_hex(c.bad, _BLACK, 0.18)
    tokens["danger.bg_hover"] = c.bad
    tokens["danger.bg_pressed"] = mix_hex(c.bad, _BLACK, 0.34)
    tokens["danger.soft"] = mix_hex(c.bg_raised, c.bad, 0.20)
    tokens["warn.soft"] = mix_hex(c.bg_raised, c.warn, 0.20)

    # -- fields -------------------------------------------------------------
    tokens["field.bg"] = c.bg_base
    tokens["field.bg_hover"] = mix_hex(c.bg_base, c.bg_raised, 0.45)
    tokens["field.bg_disabled"] = mix_hex(c.bg_base, c.bg_raised, 0.20)

    # -- selection ----------------------------------------------------------
    tokens["sel.bg"] = mix_hex(c.bg_raised, c.accent, 0.30)
    tokens["sel.bg_inactive"] = mix_hex(c.bg_raised, c.text_secondary, 0.14)

    # -- scrollbars ---------------------------------------------------------
    tokens["scroll.handle"] = mix_hex(c.bg_raised, c.text_primary, 0.18)
    tokens["scroll.handle_hover"] = mix_hex(c.bg_raised, c.text_primary, 0.32)
    tokens["scroll.handle_pressed"] = mix_hex(c.bg_raised, c.text_primary, 0.46)

    # -- tables -------------------------------------------------------------
    tokens["header.bg"] = mix_hex(c.bg_raised, c.bg_base, 0.50)
    tokens["header.bg_hover"] = mix_hex(c.bg_raised, c.text_primary, 0.08)
    tokens["row.alt"] = mix_hex(c.bg_raised, c.bg_base, 0.35)
    tokens["row.hover"] = mix_hex(c.bg_raised, c.text_primary, 0.06)

    # -- sliders ------------------------------------------------------------
    tokens["slider.groove"] = mix_hex(c.bg_base, c.text_primary, 0.12)
    tokens["slider.fill"] = c.accent
    tokens["slider.handle"] = c.text_primary
    tokens["slider.handle_border"] = mix_hex(c.bg_base, c.text_primary, 0.45)

    # -- focus / chrome -----------------------------------------------------
    tokens["focus.ring"] = c.accent
    tokens["focus.ring_soft"] = mix_hex(c.bg_base, c.accent, 0.50)
    tokens["tooltip.border"] = mix_hex(c.line, c.text_tertiary, 0.45)

    # The HUD bar floats over live video, so it is the one surface that has to
    # be genuinely translucent rather than blended against a known ground.
    tokens["hud.bg"] = rgba_css(c.bg_base, 0.82)

    tokens.update(_arrow_tokens(theme))
    tokens.update(_indicator_tokens(theme))
    return tokens


def _indicator_tokens(theme: Theme) -> dict[str, str]:
    """Check marks for QCheckBox, as whole declarations like the arrows."""
    names = ("check", "indeterminate")
    if not _ensure_arrow_assets(theme):
        return {"indicator." + n: "" for n in names}
    size = icons.INDICATOR_SIZE
    return {
        "indicator."
        + name: "image: url(%s:%s.png); width: %dpx; height: %dpx;"
        % (_ARROW_PREFIX, name, size, size)
        for name in names
    }


def _arrow_tokens(theme: Theme) -> dict[str, str]:
    """Whole declarations, so a failed export degrades to an empty rule.

    Substituting just the URL would leave ``width``/``height`` behind and size
    a sub-control around an image that does not exist, which hides the arrow
    more thoroughly than having no rule at all.
    """
    names = ("down", "up", "right")
    states = ("", "_off", "_on")
    if not _ensure_arrow_assets(theme):
        return {"arrow." + n + s: "" for n in names for s in states}

    suffixes = {"": "", "_off": "-off", "_on": "-on"}
    size = icons.ARROW_SIZE
    return {
        "arrow." + name + state: (
            "image: url(%s:%s%s.png); width: %dpx; height: %dpx;"
            % (_ARROW_PREFIX, name, suffixes[state], size, size)
        )
        for name in names
        for state in states
    }


def _ensure_arrow_assets(theme: Theme) -> bool:
    """Rasterise the stylesheet's arrows once per process and register them."""
    global _arrows_ready
    if _arrows_ready is not None:
        return _arrows_ready

    base = QStandardPaths.writableLocation(
        QStandardPaths.StandardLocation.CacheLocation
    )
    if not base:
        _arrows_ready = False
        return False

    directory = os.path.join(base, "qss-arrows")
    ok = icons.export_stylesheet_arrows(
        directory,
        {
            "": theme.q.text_secondary,
            "-off": QColor(qss_disabled_text(theme)),
            "-on": theme.q.accent,
        },
    )
    # The check mark is drawn on top of the accent-filled indicator, so it takes
    # the on-accent colour rather than any of the arrows' greys.
    ok = icons.export_stylesheet_indicators(directory, theme.q.bg_base) and ok
    if ok:
        QDir.addSearchPath(_ARROW_PREFIX, directory)
    _arrows_ready = ok
    return ok


def qss_disabled_text(theme: Theme) -> str:
    """The disabled-text colour, shared by the stylesheet and the arrow export."""
    return mix_hex(theme.color.text_tertiary, theme.color.bg_raised, 0.40)


# --------------------------------------------------------------------------
# Template
# --------------------------------------------------------------------------


def qss_template() -> str:
    """The raw ``telekart.qss`` text, from the installed package."""
    try:
        from importlib.resources import files

        return (
            files(__package__ or "telekart_app.ui.theme")
            .joinpath(_QSS_FILENAME)
            .read_text(encoding="utf-8")
        )
    except (ImportError, FileNotFoundError, ModuleNotFoundError, TypeError):
        # Loose-file fallback: running the gallery or a widget straight out of
        # a checkout, where the package may not be installed.
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, _QSS_FILENAME), encoding="utf-8") as handle:
            return handle.read()


def build_qss(theme: Theme = THEME, *, template: str | None = None) -> str:
    """Substitute the token table into the template.

    An unresolved token raises. Leaving ``{{scroll.handle}}`` in the sheet
    would make Qt drop that one declaration and keep going, so the failure
    would surface as a single wrongly-coloured scrollbar somebody notices
    weeks later.
    """
    source = qss_template() if template is None else template
    table = qss_tokens(theme)
    missing: list[str] = []

    def replace(match: re.Match[str]) -> str:
        key = match.group(1)
        value = table.get(key)
        if value is None:
            missing.append(key)
            return ""
        return value

    result = _PLACEHOLDER.sub(replace, source)
    if missing:
        raise KeyError(
            "unknown stylesheet token(s): %s" % (", ".join(sorted(set(missing))),)
        )
    return result


#: Namespaces the staleness check ignores. These are the raw design scales and
#: the generated arrow family: the painted instruments read them straight off
#: the Theme object, so "the stylesheet does not mention color.cyan" is a
#: statement about the stylesheet, not a stale token.
_SHARED_NAMESPACES = ("color.", "space.", "radius.", "font.", "family.", "arrow.")


def unused_tokens(theme: Theme = THEME) -> list[str]:
    """Derived tokens defined here but referenced nowhere. Guards against rot.

    A derived token with no consumer is dead arithmetic that still has to be
    kept correct every time the palette moves, which is exactly the kind of
    thing that quietly stops being correct.
    """
    source = qss_template()
    referenced = {m.group(1) for m in _PLACEHOLDER.finditer(source)}
    return sorted(
        name
        for name in set(qss_tokens(theme)) - referenced
        if not name.startswith(_SHARED_NAMESPACES)
    )


# --------------------------------------------------------------------------
# Palette
# --------------------------------------------------------------------------


def build_palette(theme: Theme = THEME) -> QPalette:
    """A complete QPalette covering all three colour groups.

    Fusion draws a surprising amount from the palette rather than from the
    stylesheet: scrollbar arrows in views the stylesheet does not match, the
    frame shading around spin boxes, disabled text everywhere, and the colours
    handed to Qt's non-native dialogs. Setting only the stylesheet leaves those
    at Fusion's light-grey defaults, which on a #0B0D10 ground is unmissable.
    """
    t = qss_tokens(theme)
    q = theme.q
    palette = QPalette()

    window = q.bg_base
    base = QColor(t["field.bg"])
    alt_base = QColor(t["row.alt"])
    button = QColor(t["btn.bg"])
    disabled_text = QColor(t["text.disabled"])
    highlight = QColor(t["sel.bg"])

    active = (
        (QPalette.ColorRole.Window, window),
        (QPalette.ColorRole.WindowText, q.text_primary),
        (QPalette.ColorRole.Base, base),
        (QPalette.ColorRole.AlternateBase, alt_base),
        (QPalette.ColorRole.ToolTipBase, q.bg_overlay),
        (QPalette.ColorRole.ToolTipText, q.text_primary),
        (QPalette.ColorRole.PlaceholderText, q.text_tertiary),
        (QPalette.ColorRole.Text, q.text_primary),
        (QPalette.ColorRole.Button, button),
        (QPalette.ColorRole.ButtonText, q.text_primary),
        (QPalette.ColorRole.BrightText, q.bad),
        (QPalette.ColorRole.Highlight, highlight),
        (QPalette.ColorRole.HighlightedText, q.text_primary),
        (QPalette.ColorRole.Link, q.cyan),
        (QPalette.ColorRole.LinkVisited, q.purple),
        # The 3D shading roles. Fusion uses these for every bevel and for the
        # little arrows it draws itself; leaving them light is what produces
        # the classic "dark theme with silver scrollbar buttons".
        (QPalette.ColorRole.Light, mix(q.bg_raised, q.text_primary, 0.16)),
        (QPalette.ColorRole.Midlight, mix(q.bg_raised, q.text_primary, 0.10)),
        (QPalette.ColorRole.Mid, q.line),
        (QPalette.ColorRole.Dark, mix(q.bg_base, QColor(_BLACK), 0.30)),
        (QPalette.ColorRole.Shadow, QColor(_BLACK)),
    )

    for group in (
        QPalette.ColorGroup.Active,
        QPalette.ColorGroup.Inactive,
        QPalette.ColorGroup.Disabled,
    ):
        for role, color in active:
            palette.setColor(group, role, color)

    # An inactive window must still be readable -- a driver alt-tabbing to a
    # notes window and back should not see the HUD grey out.
    palette.setColor(
        QPalette.ColorGroup.Inactive,
        QPalette.ColorRole.Highlight,
        QColor(t["sel.bg_inactive"]),
    )
    palette.setColor(
        QPalette.ColorGroup.Inactive,
        QPalette.ColorRole.HighlightedText,
        q.text_secondary,
    )

    for role in (
        QPalette.ColorRole.Text,
        QPalette.ColorRole.WindowText,
        QPalette.ColorRole.ButtonText,
        QPalette.ColorRole.ToolTipText,
        QPalette.ColorRole.HighlightedText,
    ):
        palette.setColor(QPalette.ColorGroup.Disabled, role, disabled_text)
    palette.setColor(
        QPalette.ColorGroup.Disabled,
        QPalette.ColorRole.Base,
        QColor(t["field.bg_disabled"]),
    )
    palette.setColor(
        QPalette.ColorGroup.Disabled,
        QPalette.ColorRole.Button,
        QColor(t["btn.bg_disabled"]),
    )
    palette.setColor(
        QPalette.ColorGroup.Disabled,
        QPalette.ColorRole.Highlight,
        QColor(t["sel.bg_inactive"]),
    )

    # Qt 6.6 added an explicit Accent role that some platform styles consult.
    accent_role = getattr(QPalette.ColorRole, "Accent", None)
    if accent_role is not None:
        for group in (
            QPalette.ColorGroup.Active,
            QPalette.ColorGroup.Inactive,
            QPalette.ColorGroup.Disabled,
        ):
            palette.setColor(group, accent_role, q.accent)

    return palette


# --------------------------------------------------------------------------
# Application
# --------------------------------------------------------------------------


def apply_theme(app: QApplication, theme: Theme = THEME) -> AppliedTheme:
    """Style the whole application. Call once, immediately after construction.

    Order matters and is not negotiable: style, then fonts, then palette, then
    stylesheet. The stylesheet is last because Qt re-polishes every existing
    widget when it changes, and doing that before the palette is set means
    every widget gets polished twice for no reason.
    """
    app.setStyle("Fusion")
    # Read the style's identity now: once a stylesheet is installed, app.style()
    # returns Qt's QStyleSheetStyle proxy and the real name is no longer visible.
    style = app.style()
    name_fn = getattr(style, "name", None)
    style_name = (name_fn() if callable(name_fn) else "") or style.objectName()

    roles = fonts.resolve_roles(theme)
    app.setFont(fonts.base_application_font(theme))

    app.setPalette(build_palette(theme))

    sheet = build_qss(theme)
    app.setStyleSheet(sheet)

    return AppliedTheme(
        style=style_name or "unknown",
        font_roles=roles,
        stylesheet_length=len(sheet),
    )


def restyle(widget: object) -> None:
    """Re-evaluate stylesheet rules for a widget whose dynamic property changed.

    Qt does not re-run selector matching when a property used in a selector --
    ``variant``, ``state`` -- is written. Without this, setting
    ``variant="danger"`` after the widget is shown does nothing at all, which
    is a genuinely confusing hour the first time it happens to someone.
    """
    style = getattr(widget, "style", None)
    if style is None:
        return
    current = style()
    current.unpolish(widget)
    current.polish(widget)
    update = getattr(widget, "update", None)
    if update is not None:
        update()


__all__ = [
    "AppliedTheme",
    "apply_theme",
    "build_palette",
    "build_qss",
    "qss_template",
    "qss_tokens",
    "restyle",
    "unused_tokens",
]
