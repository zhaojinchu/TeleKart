"""Every icon in the set must actually draw something.

These exist because six of them did not. ``_arc``, ``_radial_teeth`` and
``_signal_bars`` returned bare SVG path *data* rather than a ``<path>``
element, and the renderer silently dropped it -- so ``signal`` was invisible,
``link`` was a lone dot and ``settings`` was a lone circle. Nothing failed, no
exception was raised, and the navigation rail simply had holes in it.

A pixel-coverage floor is the only assertion that catches that class of bug:
the SVG parses, the QPixmap is created, and the glyph is missing anyway.
"""

from __future__ import annotations

import pytest
from PySide6.QtGui import QColor

from telekart_app.ui.theme import icons

pytest.importorskip("PySide6.QtSvg")


def _coverage(name: str, size: int = 32) -> int:
    """Number of non-transparent pixels in a rendered icon."""
    image = icons.icon_pixmap(name, QColor("#ffffff"), size).toImage()
    return sum(
        1
        for y in range(image.height())
        for x in range(image.width())
        if image.pixelColor(x, y).alpha() > 10
    )


#: A 32 px glyph at a 1.8 unit stroke in a 24 unit viewBox covers well over a
#: hundred pixels. Sixty is far below anything legitimate and far above the
#: fourteen that the broken ``link`` glyph managed.
_MIN_COVERAGE = 60


@pytest.mark.parametrize("name", sorted(icons.ICON_NAMES))
def test_every_icon_draws_something(qapp, name: str) -> None:
    coverage = _coverage(name)
    assert coverage >= _MIN_COVERAGE, (
        f"icon {name!r} rendered {coverage} pixels; it is blank or nearly so"
    )


@pytest.mark.parametrize(
    "generated",
    [
        icons._arc(12, 17, 9.4, 200, 340),
        icons._radial_teeth(8, 6.4, 9.0),
        icons._signal_bars(4),
    ],
)
def test_path_generators_emit_elements_not_bare_data(generated: str) -> None:
    """The generators must return markup, not a naked ``d`` attribute."""
    assert generated.startswith("<path d=\"")
    assert generated.endswith('"/>')


def test_generated_paths_survive_into_the_document(qapp) -> None:
    """The composed icons keep every element their body was built from."""
    source = icons.svg_source("settings", QColor("#ffffff"))
    assert source.count("<path") >= 1
    assert source.count("<circle") >= 1
    # A stray "M12.00" outside an attribute would mean data leaked into the
    # document as text again.
    assert ">M" not in source
