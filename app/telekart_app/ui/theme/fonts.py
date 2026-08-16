"""Font registration and role resolution.

Two jobs. First, register whatever font files are shipped in
``telekart_app/assets/fonts`` so the application looks identical on a machine
that has never heard of Inter. Second, resolve the preference chains declared in
``tokens.FontStacks`` against what is actually installed, and hand out
ready-made ``QFont`` objects for the three roles the interface uses.

The numeric role is the one that matters. Every number this application draws --
speed, RPM, lap time, RTT, duty, delta -- goes through :func:`numeric_font`,
which is monospaced and additionally requests the ``tnum``/``lnum`` OpenType
features. Proportional digits make a running timer shuffle sideways on every
frame where a 1 becomes a 7, and there is no amount of layout work that hides
it; it simply reads as amateur.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

from PySide6.QtGui import QFont, QFontDatabase, QGuiApplication

from .tokens import THEME, Theme

#: Extensions QFontDatabase can register. ``.ttc`` collections register every
#: face they contain and return them all from ``applicationFontFamilies``.
_FONT_SUFFIXES = (".ttf", ".otf", ".ttc")


@dataclass(frozen=True, slots=True)
class LoadedFonts:
    """What ``addApplicationFont`` actually accepted, for diagnostics."""

    families: tuple[str, ...]
    files: tuple[str, ...]
    rejected: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class FontRoles:
    """The families that won resolution, one per role.

    Kept so the About/diagnostics panel can say *which* font it is drawing
    with. When someone reports that the numbers look wrong on their machine,
    the first useful question is which family Qt picked, and guessing is slow.
    """

    ui: str
    numeric: str
    mono: str
    bundled: LoadedFonts

    def describe(self) -> str:
        return "ui=%s numeric=%s mono=%s bundled=%d" % (
            self.ui,
            self.numeric,
            self.mono,
            len(self.bundled.families),
        )


_ROLES: FontRoles | None = None
_CACHE: dict[tuple[str, int, int, bool], QFont] = {}


# --------------------------------------------------------------------------
# Registration
# --------------------------------------------------------------------------


def load_application_fonts(directory: str | os.PathLike[str]) -> LoadedFonts:
    """Register every font file in ``directory``. Missing directory is fine.

    Deliberately tolerant: a bundled font failing to load is a cosmetic
    downgrade to the system stack, not a reason to refuse to start a driving
    station. Anything rejected is reported rather than swallowed.
    """
    _require_gui_application("load_application_fonts")

    families: list[str] = []
    files: list[str] = []
    rejected: list[str] = []

    path = os.fspath(directory)
    if not os.path.isdir(path):
        return LoadedFonts((), (), ())

    for name in sorted(os.listdir(path)):
        if not name.lower().endswith(_FONT_SUFFIXES):
            continue
        full = os.path.join(path, name)
        font_id = QFontDatabase.addApplicationFont(full)
        if font_id < 0:
            rejected.append(full)
            continue
        files.append(full)
        for family in QFontDatabase.applicationFontFamilies(font_id):
            if family not in families:
                families.append(family)

    return LoadedFonts(tuple(families), tuple(files), tuple(rejected))


def resolve_roles(
    theme: Theme = THEME,
    *,
    assets_dir: str | os.PathLike[str] | None = None,
) -> FontRoles:
    """Register bundled fonts and pick a concrete family for each role.

    Called once from ``qss.apply_theme``. Repeat calls return the cached result
    so a dialog constructed later cannot accidentally re-register every file.
    """
    global _ROLES
    if _ROLES is not None:
        return _ROLES

    if assets_dir is None:
        assets_dir = default_assets_dir()

    bundled = load_application_fonts(assets_dir)
    installed = set(QFontDatabase.families())

    _ROLES = FontRoles(
        ui=_first_available(theme.fonts.ui, installed, fallback=_system_ui_family()),
        numeric=_first_available(
            theme.fonts.numeric, installed, fallback=_system_mono_family()
        ),
        mono=_first_available(
            theme.fonts.mono, installed, fallback=_system_mono_family()
        ),
        bundled=bundled,
    )
    _CACHE.clear()
    return _ROLES


def roles() -> FontRoles:
    """The resolved roles, resolving them now if nobody has yet."""
    return _ROLES if _ROLES is not None else resolve_roles()


def default_assets_dir() -> str:
    """``telekart_app/assets/fonts``, located relative to this file.

    Relative to ``__file__`` rather than the working directory because the app
    is launched from a desktop shortcut as often as from a terminal.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "assets", "fonts"))


# --------------------------------------------------------------------------
# Font construction
# --------------------------------------------------------------------------


def ui_font(
    size: int | None = None,
    weight: int | None = None,
    *,
    italic: bool = False,
    theme: Theme = THEME,
) -> QFont:
    """Proportional interface font: labels, buttons, prose."""
    return _build(
        "ui",
        theme.type.body if size is None else size,
        theme.weight.regular if weight is None else weight,
        italic,
        theme,
    )


def numeric_font(
    size: int | None = None,
    weight: int | None = None,
    *,
    theme: Theme = THEME,
) -> QFont:
    """Tabular figures. Use this for **every** number, without exception."""
    return _build(
        "numeric",
        theme.type.readout if size is None else size,
        theme.weight.medium if weight is None else weight,
        False,
        theme,
    )


def mono_font(
    size: int | None = None,
    weight: int | None = None,
    *,
    theme: Theme = THEME,
) -> QFont:
    """Code, packet dumps, log lines."""
    return _build(
        "mono",
        theme.type.label if size is None else size,
        theme.weight.regular if weight is None else weight,
        False,
        theme,
    )


def apply_tabular_figures(font: QFont) -> QFont:
    """Request ``tnum`` and ``lnum`` on an existing font, in place.

    Available from Qt 6.7. Probed rather than assumed because the app supports
    a range of PySide6 builds, and because the numeric stack is monospaced
    anyway -- this is the belt to that stack's braces, not the only guarantee.
    """
    setter = getattr(font, "setFeature", None)
    tag_type = getattr(QFont, "Tag", None)
    if setter is None or tag_type is None:
        return font
    from_string = getattr(tag_type, "fromString", None)
    if from_string is None:
        return font
    for name in ("tnum", "lnum"):
        try:
            tag = from_string(name)
            if tag is None:
                return font
            setter(tag, 1)
        except (TypeError, ValueError, RuntimeError):
            # Capability probe: an older or differently-built PySide6 raises
            # here, and the monospaced family already gives tabular metrics.
            return font
    return font


def base_application_font(theme: Theme = THEME) -> QFont:
    """The font handed to ``QApplication.setFont`` before the stylesheet."""
    return ui_font(theme.type.body, theme.weight.regular, theme=theme)


# --------------------------------------------------------------------------
# Internals
# --------------------------------------------------------------------------


def _build(role: str, size: int, weight: int, italic: bool, theme: Theme) -> QFont:
    key = (role, size, weight, italic)
    cached = _CACHE.get(key)
    if cached is None:
        resolved = roles()
        stack: tuple[str, ...]
        if role == "numeric":
            family, stack = resolved.numeric, theme.fonts.numeric
        elif role == "mono":
            family, stack = resolved.mono, theme.fonts.mono
        else:
            family, stack = resolved.ui, theme.fonts.ui

        font = QFont(family)
        # Give Qt the whole chain as well: if the resolved family is missing a
        # glyph, Qt walks the rest of the list itself instead of dropping to
        # its own default, which on Linux is frequently something unpleasant.
        font.setFamilies([family, *stack])
        font.setPointSize(size)
        font.setWeight(QFont.Weight(weight))
        font.setItalic(italic)
        if role == "ui":
            font.setStyleHint(QFont.StyleHint.SansSerif)
        else:
            font.setStyleHint(QFont.StyleHint.Monospace)
            font.setFixedPitch(True)
            apply_tabular_figures(font)
        _CACHE[key] = font
        cached = font
    # Copy out: QFont is implicitly shared, so this is a refcount bump, and it
    # stops one widget's setBold() from silently restyling every other caller.
    return QFont(cached)


def _first_available(
    chain: tuple[str, ...], installed: set[str], *, fallback: str
) -> str:
    for family in chain:
        if family in installed:
            return family
    return fallback


def _system_ui_family() -> str:
    font = QFontDatabase.systemFont(QFontDatabase.SystemFont.GeneralFont)
    return font.family()


def _system_mono_family() -> str:
    font = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
    return font.family()


def _require_gui_application(what: str) -> None:
    """QFontDatabase is meaningless before the platform plugin is up.

    Fails loudly at setup time, which is where this class of mistake belongs;
    the alternative is a window that renders in Qt's fallback font for reasons
    nobody can reconstruct later.
    """
    if QGuiApplication.instance() is None:
        raise RuntimeError(
            "%s requires a QGuiApplication; construct QApplication first" % (what,)
        )


__all__ = [
    "FontRoles",
    "LoadedFonts",
    "apply_tabular_figures",
    "base_application_font",
    "default_assets_dir",
    "load_application_fonts",
    "mono_font",
    "numeric_font",
    "resolve_roles",
    "roles",
    "ui_font",
]
