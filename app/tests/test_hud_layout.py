"""HUD geometry invariants.

Two of these are regressions. The status cluster laid its active-flag strip out
*inside* the state badge, so "LIMITER CAL CL ODOM VIDEO" was drawn straight
through the middle of the word ARMED -- and the state badge is the one element
on the driving screen whose legibility is not negotiable, because it answers
"are the motors live?". And the HUD anchored itself to the video *widget*
rather than to the letterboxed picture inside it, which left the cluster
floating half over a black bar.
"""

from __future__ import annotations

from PySide6.QtCore import QSize

from telekart_app.ui.hud.status_cluster import StatusCluster
from telekart_app.ui.video_view import VideoView


class _StubPresenter:
    """A presenter with a fixed content size and no pixels.

    Stands in for a decoded frame so the letterbox geometry can be exercised
    without PyAV, a codec or a socket -- the geometry is what is under test.
    """

    def __init__(self, width: int, height: int) -> None:
        self._size = QSize(width, height)

    def content_size(self) -> QSize:
        return self._size

    @property
    def has_content(self) -> bool:
        return True

    @property
    def bundle(self):  # pragma: no cover - never painted in these tests
        return None

    def present(self, bundle) -> None:  # pragma: no cover
        pass

    def release(self) -> None:  # pragma: no cover
        self._size = QSize(0, 0)

    def paint(self, painter, target) -> None:  # pragma: no cover
        pass


def test_flag_strip_does_not_overlap_the_state_badge(qapp) -> None:
    cluster = StatusCluster()
    cluster.resize(cluster.sizeHint())
    cluster.on_layout(cluster.width(), cluster.height())

    badge = cluster._badge_rect
    flags = cluster._flag_rect
    assert badge.height() > 0 and flags.height() > 0
    assert not badge.intersects(flags), (
        "the flag strip is inside the state badge; the two texts will collide"
    )
    assert flags.top() >= badge.bottom()


def test_cluster_contents_fit_inside_the_widget(qapp) -> None:
    cluster = StatusCluster()
    cluster.resize(cluster.sizeHint())
    cluster.on_layout(cluster.width(), cluster.height())
    bottom = max(rect.bottom() for _, rect in cluster._cells)
    assert bottom <= cluster.height(), "the value cells run off the bottom"
    assert cluster._flag_rect.bottom() <= cluster._pip_rects[0].top()


def test_hud_follows_the_letterboxed_picture(qapp) -> None:
    """A 4:3 frame in a 16:10 widget must pull the HUD in with it."""
    view = VideoView()
    view.resize(QSize(1600, 900))
    view.show()

    # No picture yet: the HUD owns the whole widget so the placeholder is centred.
    assert view.hud.geometry() == view.rect()

    view._presenter = _StubPresenter(640, 480)  # type: ignore[assignment]
    view._recompute_target()
    view._place_hud()

    picture = view.frame_rect()
    assert view.hud.geometry() == picture
    assert picture.width() < view.width(), "expected letterbox bars"
    assert picture.height() == view.height()


def test_hud_geometry_is_never_empty(qapp) -> None:
    view = VideoView()
    view.resize(QSize(400, 300))
    view.show()
    view.clear()
    assert view.hud.width() > 0 and view.hud.height() > 0
