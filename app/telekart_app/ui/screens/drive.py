"""The driving screen: full-bleed video with the HUD composited over it.

No chrome. No panels, no toolbar, no status strip -- the picture reaches all
four edges and the instruments float on it. Everything a driver needs mid-corner
is on the HUD; everything else is a menu item or another screen, because a
control the driver has to look at is a control they cannot use while driving.

The frame clock is the model's, not this screen's. ``AppModel`` runs the app's
single 60 Hz tick (INTERFACES.md 9): it drains every LatestBox, then emits
``ticked``, and this screen pulls the frame the model just took. A second timer
here would be a second 60 Hz clock beating against the first -- the model would
sometimes drain two frames between two repaints, and the one in the middle would
be dropped for no reason other than phase.

The subscription is made on show and dropped on hide, so sitting in the garage
tuning parameters does not spin a repaint loop over a video widget nobody is
looking at.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from PySide6.QtCore import Signal
from PySide6.QtGui import QHideEvent, QShowEvent
from PySide6.QtWidgets import QHBoxLayout, QWidget

from ...model.snapshots import (
    InputSnapshot,
    LinkSnapshot,
    LinkState,
    SessionSnapshot,
    VehicleSnapshot,
)
from ...model.units import UnitFormatter
from ..theme.tokens import THEME, Theme
from ..video_view import VideoView

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ...model.app_model import AppModel

#: Video age past which the last frame stops being evidence about the world.
#: Three frame intervals at 30 fps plus a keyframe interval of slack: below
#: this a dropped frame is normal, above it the picture is history.
_VIDEO_STALE_S = 0.60


class DriveScreen(QWidget):
    """Video plus HUD, and nothing else."""

    #: Raised when the car reports that it has become armed, so the window can
    #: go fullscreen. The screen does not do it itself: fullscreen is a window
    #: property and a screen that reaches up to its window is a screen that
    #: cannot be embedded anywhere else.
    armedChanged = Signal(bool)

    def __init__(
        self,
        model: "AppModel",
        parent: QWidget | None = None,
        *,
        theme: Theme = THEME,
    ) -> None:
        super().__init__(parent)
        self._model = model
        self._theme = theme
        self._armed = False

        self.video = VideoView(self, theme=theme)
        self.hud = self.video.hud

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.video)

        self._ticking = False

        self.setObjectName("DriveScreen")
        self.setFocusProxy(self.video)
        self.setAutoFillBackground(False)

        self.video.set_frame_provider(model.take_frame)
        self.video.displaySizeChanged.connect(model.set_display_size)

        model.vehicleChanged.connect(self._on_vehicle)
        model.linkChanged.connect(self._on_link)
        model.inputChanged.connect(self._on_input)
        model.sessionChanged.connect(self._on_session)

        # Paint the current state immediately rather than waiting for the next
        # change signal. Switching to this screen mid-drive must not show an
        # empty HUD until the car happens to move.
        self._on_link(model.link)
        self._on_vehicle(model.vehicle)
        self._on_input(model.input)
        self._on_session(model.session)

    # -- configuration ------------------------------------------------------

    def set_formatter(self, formatter: UnitFormatter) -> None:
        self.hud.set_formatter(formatter)

    def set_diagnostics_visible(self, visible: bool) -> None:
        self.hud.set_diagnostics_visible(visible)

    def toggle_diagnostics(self) -> bool:
        return self.hud.toggle_diagnostics()

    def set_hud_visible(self, visible: bool) -> None:
        self.hud.set_instruments_visible(visible)

    @property
    def armed(self) -> bool:
        return self._armed

    # -- frame clock --------------------------------------------------------

    def showEvent(self, event: QShowEvent) -> None:
        if not self._ticking:
            self._model.ticked.connect(self._on_tick)
            self._ticking = True
        super().showEvent(event)

    def hideEvent(self, event: QHideEvent) -> None:
        if self._ticking:
            self._model.ticked.disconnect(self._on_tick)
            self._ticking = False
        # Releasing the frame here is what stops a backgrounded drive screen
        # pinning several megabytes of decoded picture for the whole session.
        self.video.clear()
        super().hideEvent(event)

    def _on_tick(self, _now: float) -> None:
        self.video.tick()

    # -- model --------------------------------------------------------------

    def _on_vehicle(self, v: VehicleSnapshot) -> None:
        self.hud.set_vehicle(v)
        if v.armed != self._armed:
            self._armed = v.armed
            self.armedChanged.emit(self._armed)

    def _on_link(self, link: LinkSnapshot) -> None:
        self.hud.set_link(link)
        self.video.set_stale(
            link.state.usable and link.video_ok and link.video_age > _VIDEO_STALE_S
        )
        self.video.set_placeholder(*_placeholder_for(link))

    def _on_input(self, sample: InputSnapshot) -> None:
        self.hud.set_input(sample)

    def _on_session(self, session: SessionSnapshot) -> None:
        self.hud.set_params(session.params)


def _placeholder_for(link: LinkSnapshot) -> tuple[str, str]:
    """What to say when there is no picture, given why there is no picture.

    "No video" is useless on its own -- it is true whether the car is switched
    off, the camera process died, or nobody has pressed connect yet, and those
    have completely different next actions.
    """
    state = link.state
    if state is LinkState.OFFLINE:
        return ("not connected", "connect to a car, or launch the simulator")
    if state is LinkState.DISCOVERING:
        return ("looking for a car", "browsing for telekart.local")
    if state is LinkState.CONNECTING:
        return ("connecting", link.detail or link.address)
    if state is LinkState.RECONNECTING:
        return ("reconnecting", link.detail)
    if state is LinkState.FAILED:
        return ("connection failed", link.detail)
    if not link.video_ok:
        return ("no video", "the car is connected but not sending a picture")
    return ("waiting for the first frame", link.detail)


__all__ = ["DriveScreen"]
