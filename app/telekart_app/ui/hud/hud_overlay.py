"""The HUD as a whole: three edge-anchored zones over an untouched centre.

The layout rule is one line long and everything else follows from it: **the
centre of the video stays clear.** A driver reading a corner apex is looking at
the middle of the frame, and anything parked there -- however small, however
useful -- is read as an obstacle for the first hundred milliseconds and as
clutter for ever after. So the instruments live against the edges: state and
link at the top right, the driving block at the bottom centre, the diagnostic
dump at the top left when it is asked for.

The one thing allowed in the centre is the event banner, and only because by
the time it appears there is no useful driving left to do.

There is a second, quieter dividend. This overlay is a child of the video
widget, so Qt repaints the picture beneath whichever zone updates. Small zones
against the edges mean small dirty rectangles; a HUD spread across the middle
would force a full-frame re-blit sixty times a second for a number that changed
in one corner.
"""

from __future__ import annotations

from typing import Any, Mapping

from PySide6.QtCore import QSize, Qt
from PySide6.QtGui import QResizeEvent
from PySide6.QtWidgets import QWidget
from telekart_protocol import CRITICAL_FAULTS, Fault, TelemetryFlags, VehicleState

from ...model.snapshots import (
    EMPTY_INPUT,
    EMPTY_LINK,
    EMPTY_VEHICLE,
    InputSnapshot,
    LinkSnapshot,
    LinkState,
    VehicleSnapshot,
)
from ...model.units import UnitFormatter, fault_text, hud_flag_texts, mps_to_rpm, state_text
from ..theme.tokens import THEME, Theme
from .diag_overlay import DiagOverlay
from .event_banner import BannerEvent, EventBanner, Severity
from .status_cluster import StatusCluster, health_for_age
from .wheel_bar import WheelBar

#: Distance from the picture edge to the nearest instrument. Generous on
#: purpose: cameras vignette, and a panel hard against the frame edge sits in
#: the darkest, least stable part of the image.
MARGIN = 20

#: Where the event banner's centre sits, as a fraction of the picture height.
#: Above the true centre so it lands in the sky rather than on the road.
_BANNER_Y = 0.32

#: Channel age thresholds, seconds. Telemetry runs at 50 Hz and control at 100,
#: so a tenth of a second is already several missed packets; the car's own
#: failsafe starts at 200 ms and the pip must go red before that, not after.
_TLM_WARN, _TLM_DOWN = 0.10, 0.30
_CTRL_WARN, _CTRL_DOWN = 0.05, 0.20
#: Video is a 30 fps stream and one dropped keyframe interval is normal.
_VID_WARN, _VID_DOWN = 0.35, 1.20


class HudOverlay(QWidget):
    """Composites the four HUD zones over whatever widget it is a child of.

    Purely snapshot-driven: there is no tick. Each zone runs the shared
    :class:`~telekart_app.ui.widgets.base.ValueFollower` animation while its
    values are still moving and stops its timer when they settle, so a car
    sitting still on the bench costs nothing while a car at speed gets the full
    60 Hz. Driving the whole overlay from one unconditional timer instead would
    repaint four panels sixty times a second to show four numbers that had not
    changed.
    """

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent)
        self._theme = theme

        self._vehicle = EMPTY_VEHICLE
        self._link = EMPTY_LINK
        self._input = EMPTY_INPUT
        self._params: Mapping[str, Any] = {}
        self._formatter = UnitFormatter()
        self._instruments_visible = True
        self._diagnostics_wanted = False

        self.cluster = StatusCluster(self, theme=theme)
        self.wheel = WheelBar(self, theme=theme)
        self.banner = EventBanner(self, theme=theme)
        self.diagnostics = DiagOverlay(self, theme=theme)

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAutoFillBackground(False)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    # -- configuration ------------------------------------------------------

    @property
    def theme(self) -> Theme:
        return self._theme

    def set_formatter(self, formatter: UnitFormatter) -> None:
        self._formatter = formatter
        self.wheel.set_formatter(formatter)

    def set_params(self, params: Mapping[str, Any]) -> None:
        """The car's echoed parameter set.

        Two values matter to the HUD and both scale an instrument rather than
        decorate it: ``steer_max_deg`` turns the servo angle into a fraction of
        lock, and ``wheel_diameter_m`` turns the measured ``v_max`` into a
        redline for the shift lights. Neither is guessed if the car has not
        sent it -- the affected instrument simply says it does not know, which
        is the honest answer for an uncalibrated car.
        """
        self._params = params
        lock = params.get("steer_max_deg")
        self.wheel.set_lock_deg(float(lock) if isinstance(lock, (int, float)) else 0.0)
        self._refresh_redline()

    def set_instruments_visible(self, visible: bool) -> None:
        """Hide the driving instruments for a clean picture.

        The event banner is deliberately *not* hidden. Turning the HUD off is a
        request for a clean view, not a request to stop being told the car has
        latched an E-stop.
        """
        self._instruments_visible = visible
        self._apply_visibility()

    def set_diagnostics_visible(self, visible: bool) -> None:
        """Ask for the diagnostic dump.

        The *request* is remembered separately from whether it is currently on
        screen, because the panel is wide and a narrow window cannot show it
        without covering the state cluster. Forgetting the request would mean
        that enabling the overlay before the window is sized -- which is what
        happens when the setting is restored at startup -- silently loses it.
        """
        self._diagnostics_wanted = visible
        self.diagnostics.setVisible(visible and self._fits_diagnostics())
        self._place()

    def toggle_diagnostics(self) -> bool:
        self.set_diagnostics_visible(not self._diagnostics_wanted)
        return self._diagnostics_wanted

    @property
    def diagnostics_visible(self) -> bool:
        return self._diagnostics_wanted

    # -- data ---------------------------------------------------------------

    def set_vehicle(self, v: VehicleSnapshot) -> None:
        self._vehicle = v
        self.cluster.set_state(_state_label(v, self._link), _state_key(v, self._link))
        self.cluster.set_flags(" ".join(hud_flag_texts(v.flags)))
        self.cluster.set_power(v.pack_volts, v.cpu_temp_c, v.throttled)

        self.wheel.set_speed(v.speed)
        self.wheel.set_rpm(max(abs(v.rpm_l), abs(v.rpm_r)))
        self.wheel.set_steering(
            self._input.steering, v.steer_angle, has_actual=v.valid and not v.stale
        )
        self.wheel.set_gear(_gear_of(v))
        self._refresh_redline()

        self.diagnostics.set_vehicle(v)
        self._refresh_events()

    def set_link(self, link: LinkSnapshot) -> None:
        self._link = link
        active = link.state.usable
        self.cluster.set_channel(
            "ctrl",
            health_for_age(
                self._input.command_age,
                warn=_CTRL_WARN,
                down=_CTRL_DOWN,
                active=active and self._input.transmitting,
            ),
        )
        self.cluster.set_channel(
            "tlm",
            health_for_age(
                link.telemetry_age, warn=_TLM_WARN, down=_TLM_DOWN, active=active
            ),
        )
        self.cluster.set_channel(
            "vid",
            health_for_age(
                link.video_age, warn=_VID_WARN, down=_VID_DOWN, active=link.video_ok
            ),
        )
        self.cluster.set_rtt(link.rtt)
        self.cluster.set_loss(link.loss)
        self.cluster.set_state(
            _state_label(self._vehicle, link), _state_key(self._vehicle, link)
        )

        self.diagnostics.set_link(link)
        self._refresh_events()

    def set_input(self, sample: InputSnapshot) -> None:
        self._input = sample
        self.wheel.set_pedals(sample.throttle, sample.brake)
        self.wheel.set_transmitting(sample.transmitting)
        self.wheel.set_steering(
            sample.steering,
            self._vehicle.steer_angle,
            has_actual=self._vehicle.valid and not self._vehicle.stale,
        )
        self.diagnostics.set_input(sample)
        self._refresh_events()

    # -- derived ------------------------------------------------------------

    def _refresh_redline(self) -> None:
        """Shift-light redline from the car's *measured* ceiling.

        Nothing here hardcodes a top speed: ``v_max`` is whatever calibration
        measured on this drivetrain, and the same build therefore lights up
        correctly on today's L298N-limited car and on a MOSFET bridge later.
        """
        diameter = self._params.get("wheel_diameter_m")
        if not isinstance(diameter, (int, float)) or diameter <= 0.0:
            self.wheel.set_redline_rpm(0.0)
            return
        self.wheel.set_redline_rpm(mps_to_rpm(self._vehicle.v_max, float(diameter)))

    def _refresh_events(self) -> None:
        events = build_events(self._vehicle, self._link, self._input)
        if self.banner.set_events(events):
            self._place()

    # -- layout -------------------------------------------------------------

    def resizeEvent(self, event: QResizeEvent) -> None:
        self._apply_visibility()
        self._place()
        super().resizeEvent(event)

    def _fits_diagnostics(self) -> bool:
        need = (
            self.diagnostics.sizeHint().width()
            + self.cluster.width()
            + MARGIN * 3
        )
        return self.width() >= need

    def _apply_visibility(self) -> None:
        show = self._instruments_visible
        w = self.width()
        h = self.height()
        self.cluster.setVisible(
            show and w >= self.cluster.width() + MARGIN * 2 and h >= 260
        )
        self.wheel.setVisible(
            show
            and w >= self.wheel.width() + MARGIN * 2
            and h >= self.wheel.height() + self.cluster.height() + MARGIN * 3
        )
        self.diagnostics.setVisible(
            self._diagnostics_wanted and self._fits_diagnostics()
        )

    def _place(self) -> None:
        w = self.width()
        h = self.height()
        if w <= 0 or h <= 0:
            return

        self.cluster.move(w - MARGIN - self.cluster.width(), MARGIN)
        self.diagnostics.move(MARGIN, MARGIN)

        wheel = self.wheel
        wheel.move((w - wheel.width()) // 2, h - MARGIN - wheel.height())

        banner = self.banner
        size: QSize = banner.size()
        if size.width() > 0:
            top = int(h * _BANNER_Y - size.height() * 0.5)
            # Never let the banner reach the driving block: the driver has to
            # be able to read the speed while deciding what to do about the
            # fault the banner is reporting.
            ceiling = h - MARGIN * 2 - wheel.height() - size.height()
            if ceiling < MARGIN:
                ceiling = MARGIN
            if top > ceiling:
                top = ceiling
            if top < MARGIN:
                top = MARGIN
            banner.move((w - size.width()) // 2, top)
            banner.raise_()


# --------------------------------------------------------------------------
# Event ranking
# --------------------------------------------------------------------------


def build_events(
    vehicle: VehicleSnapshot, link: LinkSnapshot, sample: InputSnapshot
) -> list[BannerEvent]:
    """Everything that deserves the centre of the picture, most severe first.

    A pure function of the three snapshots so the ranking can be unit-tested
    without a widget, which matters: this is the code that decides what a
    driver is told during a fault cascade, and "looks right on screen" is not
    a test.
    """
    events: list[BannerEvent] = []

    if link.state is LinkState.FAILED:
        events.append(
            BannerEvent(Severity.CRITICAL, "connection failed", link.detail)
        )
    elif link.state is LinkState.STALE:
        events.append(
            BannerEvent(
                Severity.CRITICAL,
                "signal lost",
                "no telemetry for %.1f s" % (link.telemetry_age,),
            )
        )
    elif link.state is LinkState.RECONNECTING:
        events.append(BannerEvent(Severity.WARNING, "reconnecting", link.detail))
    elif link.state is LinkState.DEGRADED:
        events.append(BannerEvent(Severity.WARNING, "video down", link.detail))

    if vehicle.state is VehicleState.ESTOP:
        events.append(
            BannerEvent(
                Severity.CRITICAL,
                "e-stop latched",
                "Car \u25b8 Clear E-stop  (\u2303\u21e7E), then Arm (\u2303\u23ce)",
            )
        )
    elif vehicle.state is VehicleState.FAILSAFE:
        events.append(
            BannerEvent(Severity.CRITICAL, "failsafe", "the car stopped hearing us")
        )
    elif vehicle.state is VehicleState.FAULT:
        events.append(BannerEvent(Severity.CRITICAL, "fault", ""))

    faults = int(vehicle.faults)
    if faults:
        critical = faults & int(CRITICAL_FAULTS)
        bit = 1
        while bit <= faults:
            if faults & bit:
                events.append(
                    BannerEvent(
                        Severity.CRITICAL if bit & critical else Severity.WARNING,
                        fault_text(bit),
                        "",
                    )
                )
            bit <<= 1
        # Undervoltage on the Pi's own rail is called out by name rather than
        # left as one fault among many: it is the failure that looks like a
        # dozen unrelated ones -- camera stalls, GPIO errors, loop overruns --
        # and knowing to check the power bank first saves an hour.
        if faults & int(Fault.PI_UNDERVOLTAGE):
            events.append(
                BannerEvent(
                    Severity.CRITICAL, "pi undervoltage", "check the power bank"
                )
            )

    if vehicle.armed and not sample.transmitting:
        events.append(
            BannerEvent(
                Severity.CRITICAL, "no command link", "the car is armed and unheld"
            )
        )
    elif vehicle.armed and not sample.device_connected:
        events.append(
            BannerEvent(Severity.WARNING, "controller disconnected", sample.device)
        )

    if vehicle.valid and vehicle.stale and link.state is not LinkState.STALE:
        events.append(BannerEvent(Severity.WARNING, "telemetry stale", ""))

    return events


def _state_label(vehicle: VehicleSnapshot, link: LinkSnapshot) -> str:
    if not link.state.usable or not vehicle.valid:
        return "offline"
    return state_text(vehicle.state)


def _state_key(vehicle: VehicleSnapshot, link: LinkSnapshot) -> str:
    if not link.state.usable or not vehicle.valid:
        return "boot"
    return vehicle.state.name.lower()


def _gear_of(vehicle: VehicleSnapshot) -> str:
    if vehicle.flags & TelemetryFlags.PIT_LIMITER:
        return "P"
    if vehicle.flags & TelemetryFlags.REVERSE_ENGAGED:
        return "R"
    return "D" if vehicle.armed else "N"


__all__ = ["MARGIN", "HudOverlay", "build_events"]
