"""Visual system: one token object, one stylesheet, one icon set.

Import order inside the package is ``tokens`` -> ``fonts``/``icons`` -> ``qss``
and there are no cycles, so a test that only wants the palette can import
``tokens`` alone without dragging in QtSvg or a font database.
"""

from __future__ import annotations

from .fonts import (
    FontRoles,
    LoadedFonts,
    apply_tabular_figures,
    base_application_font,
    load_application_fonts,
    mono_font,
    numeric_font,
    resolve_roles,
    roles,
    ui_font,
)
from .icons import ICON_NAMES, icon, icon_pixmap, icon_size, state_icon, svg_source
from .qss import (
    AppliedTheme,
    apply_theme,
    build_palette,
    build_qss,
    qss_tokens,
    restyle,
)
from .tokens import (
    THEME,
    LinkColors,
    Palette,
    Radii,
    SeriesColors,
    Spacing,
    StateColors,
    Strokes,
    Theme,
    TypeScale,
    Weights,
    hex_of,
    mix,
    mix_hex,
    rgba_css,
    theme,
    with_alpha,
)

__all__ = [
    "AppliedTheme",
    "FontRoles",
    "ICON_NAMES",
    "LinkColors",
    "LoadedFonts",
    "Palette",
    "Radii",
    "SeriesColors",
    "Spacing",
    "StateColors",
    "Strokes",
    "THEME",
    "Theme",
    "TypeScale",
    "Weights",
    "apply_tabular_figures",
    "apply_theme",
    "base_application_font",
    "build_palette",
    "build_qss",
    "hex_of",
    "icon",
    "icon_pixmap",
    "icon_size",
    "load_application_fonts",
    "mix",
    "mix_hex",
    "mono_font",
    "numeric_font",
    "qss_tokens",
    "resolve_roles",
    "restyle",
    "rgba_css",
    "roles",
    "state_icon",
    "svg_source",
    "theme",
    "ui_font",
    "with_alpha",
]
