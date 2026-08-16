"""The decoder scales to the widget; it must not reshape the picture doing it.

``set_display_size`` exists so the sws pass happens on the decode thread and
the GUI thread does a 1:1 blit. The size it is handed is a *widget*, though,
and a widget is whatever shape the window happens to be -- so scaling straight
to it stretched a 4:3 camera into 16:10. On a driving screen that is not
cosmetic: the apparent radius of a corner and the apparent offset of the car in
its lane are exactly the two things the driver reads off the picture.
"""

from __future__ import annotations

import pytest

from telekart_app.video.decoder import _fit_inside


def _aspect(size: tuple[int, int]) -> float:
    return size[0] / size[1]


@pytest.mark.parametrize(
    ("src", "box"),
    [
        ((640, 480), (1536, 950)),   # 4:3 into a wide window
        ((1280, 720), (800, 800)),   # 16:9 into a square window
        ((640, 480), (320, 480)),    # 4:3 into a tall window
        ((1920, 1080), (1920, 1080)),
        ((800, 600), (1000, 400)),
    ],
)
def test_fit_preserves_source_aspect(src, box) -> None:
    fitted = _fit_inside(src[0], src[1], box)
    assert fitted is not None
    # Even-rounding for chroma alignment costs at most one pixel per axis.
    assert _aspect(fitted) == pytest.approx(_aspect(src), rel=0.01)


@pytest.mark.parametrize(
    ("src", "box"),
    [
        ((640, 480), (1536, 950)),
        ((1280, 720), (800, 800)),
        ((640, 480), (320, 480)),
        ((800, 600), (1000, 400)),
    ],
)
def test_fit_stays_inside_the_box(src, box) -> None:
    fitted = _fit_inside(src[0], src[1], box)
    assert fitted is not None
    assert fitted[0] <= box[0]
    assert fitted[1] <= box[1]


def test_fit_touches_one_edge_so_the_blit_is_one_to_one(qapp=None) -> None:
    """The fitted frame must fill the box on one axis.

    ``VideoView`` computes ``scale = min(w/cw, h/ch)`` and skips the smooth
    transform when that is within 2 % of 1.0. If the decoder undershot both
    axes the GUI thread would resample every frame.
    """
    fitted = _fit_inside(640, 480, (1536, 950))
    assert fitted is not None
    scale = min(1536 / fitted[0], 950 / fitted[1])
    assert scale == pytest.approx(1.0, abs=0.02)


def test_fit_returns_even_dimensions() -> None:
    for src in ((641, 481), (1279, 721), (640, 480)):
        fitted = _fit_inside(src[0], src[1], (999, 777))
        assert fitted is not None
        assert fitted[0] % 2 == 0 and fitted[1] % 2 == 0


def test_no_box_means_no_scaling() -> None:
    assert _fit_inside(640, 480, None) is None
