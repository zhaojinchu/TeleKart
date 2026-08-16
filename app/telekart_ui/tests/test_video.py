"""Video geometry and frame lifetime.

Two separate concerns, both of which have bitten this codebase:

* **Geometry** -- the decoder reformats to the widget size, so the blit is 1:1
  and the letterbox is computed from the *stream's* aspect, not the widget's.
* **Lifetime** -- a ``FrameBundle`` owns an ``av.VideoFrame`` and a ``QImage``
  that views its buffer with no copy. Keeping the image and dropping the bundle
  is a use-after-free that renders correctly most of the time, which is the
  worst way for a bug like that to behave.
"""

from __future__ import annotations

import gc

import pytest

av = pytest.importorskip("av", reason="PyAV is needed for the decode path")

from telekart_ui.video import frame as frame_mod  # noqa: E402
from telekart_ui.video.decoder import _fit_inside  # noqa: E402
from telekart_ui.video.frame import PIXEL_FORMAT, FrameBundle  # noqa: E402
from telekart_ui.ui.video_view import FramePresenter, VideoView  # noqa: E402


def _bundle(width: int = 640, height: int = 480) -> FrameBundle:
    frame = av.VideoFrame(width, height, PIXEL_FORMAT)
    return FrameBundle.from_video_frame(frame)


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------


def test_fit_inside_preserves_aspect_and_rounds_even():
    """Odd dimensions break chroma subsampling in several encoders."""
    for src_w, src_h, box in [
        (640, 480, (1280, 720)),
        (1280, 720, (640, 480)),
        (1920, 1080, (1000, 1000)),
        (641, 481, (800, 600)),
    ]:
        w, h = _fit_inside(src_w, src_h, box)
        assert w % 2 == 0 and h % 2 == 0
        assert w <= box[0] and h <= box[1]
        assert abs((w / h) - (src_w / src_h)) < 0.02


def test_letterbox_centres_a_4_by_3_stream_in_a_16_by_9_window(qapp):
    view = VideoView()
    view.resize(1600, 900)
    view.show()
    view.set_frame_provider(lambda: _bundle(640, 480))
    assert view.tick() is True

    rect = view.frame_rect()
    # 4:3 in 16:9 is pillarboxed: full height, bars left and right, centred.
    assert rect.height() == pytest.approx(900, abs=2)
    assert rect.left() == pytest.approx(1600 - rect.right() - 1, abs=2)
    assert rect.width() < 1600
    view.close()


def test_the_hud_follows_the_picture_not_the_widget(qapp):
    """Anchoring to the widget leaves the status cluster over a black bar."""
    view = VideoView()
    view.resize(1600, 900)
    view.show()
    view.set_frame_provider(lambda: _bundle(640, 480))
    view.tick()
    assert view.hud.geometry() == view.frame_rect()
    view.close()


def test_display_size_is_reported_in_physical_pixels(qapp):
    view = VideoView()
    view.resize(800, 600)
    size = view.display_size()
    dpr = view.devicePixelRatioF()
    assert size.width() == int(800 * dpr + 0.5)
    view.close()


def test_a_view_with_no_frame_paints_the_placeholder(qapp):
    view = VideoView()
    view.resize(640, 480)
    view.show()
    view.set_placeholder("not connected", "press ⌘K")
    assert view.has_frame is False
    # The HUD still gets the whole widget, so the placeholder is centred in it.
    assert view.hud.geometry() == view.rect()
    view.close()


# --------------------------------------------------------------------------
# Lifetime
# --------------------------------------------------------------------------


def test_bundle_image_views_the_frame_without_copying():
    bundle = _bundle()
    assert bundle.image.width() == bundle.width
    assert bundle.image.height() == bundle.height
    assert bundle.nbytes > 0


def test_bundle_rejects_an_unconverted_frame():
    """Reformatting must happen on the decode thread, never in paintEvent."""
    frame = av.VideoFrame(640, 480, "yuv420p")
    with pytest.raises(ValueError):
        FrameBundle.from_video_frame(frame)


def test_live_count_returns_to_zero():
    """The soak test in miniature: nothing may pin decoded pixels."""
    frame_mod.reset_counters()
    bundles = [_bundle() for _ in range(8)]
    assert frame_mod.live_count() == 8
    bundles.clear()
    gc.collect()
    assert frame_mod.live_count() == 0


def test_presenter_release_frees_the_frame():
    frame_mod.reset_counters()
    presenter = FramePresenter()
    presenter.present(_bundle())
    assert frame_mod.live_count() == 1
    presenter.release()
    gc.collect()
    assert frame_mod.live_count() == 0
    assert presenter.has_content is False


def test_view_clear_releases_the_frame(qapp):
    frame_mod.reset_counters()
    view = VideoView()
    view.resize(640, 480)
    view.set_frame_provider(lambda: _bundle())
    view.tick()
    assert frame_mod.live_count() == 1
    view.clear()
    gc.collect()
    assert frame_mod.live_count() == 0
    view.close()


def test_provider_returning_none_is_not_a_repaint(qapp):
    """Take semantics: no new frame means no work at all."""
    view = VideoView()
    view.resize(640, 480)
    view.set_frame_provider(lambda: None)
    assert view.tick() is False
    assert view.frames_shown == 0
    view.close()
