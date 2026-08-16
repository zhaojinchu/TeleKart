"""The head-up display: four zones painted over the video.

Everything here is drawn with ``QPainter`` onto transparent children of the
video widget. No stock Qt widget appears on the driving surface, because a
styled widget over a moving picture reads as a dialog that failed to close.

**What is on screen, and why only this.** A driver mid-corner can read about
four things. The zones are ordered by how badly you need them:

* **Speed, throttle, brake, steering** -- bottom centre, nearest the picture's
  centre of attention.
* **State and link health** -- top right. The state badge is what the *car*
  reported, never an app-side assumption, and the three channel pips age out to
  amber and red so a HUD that has stopped being true says so.
* **Faults** -- centre, interrupting, and every message names the key that acts
  on it. A warning with no affordance is a bug: the previous station told the
  driver "e-stop latched, clear it to re-arm" and then offered no way to do it.
* **Diagnostics** -- top left, hidden behind a shortcut. One row, not a panel.

Everything in the zones is *smoothed* on its way to the screen. Telemetry
arrives at 50 Hz and the display runs at 60, so an unsmoothed needle steps
visibly at the beat frequency between the two. The smoothing is display-only and
never touches what goes on the wire.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer
from PySide6.QtGui import (
    QColor,
    QFont,
    QFontMetricsF,
    QPainter,
    QPaintEvent,
    QPen,
    QResizeEvent,
)
from PySide6.QtWidgets import QWidget

from telekart_protocol import CRITICAL_FAULTS, Fault, VehicleState

from ..model.snapshots import InputSnapshot, LinkSnapshot, LinkState, VehicleSnapshot
from ..model.units import (
    UnitFormatter,
    fault_texts,
    format_latency,
    format_temperature,
    format_voltage,
    hud_flag_texts,
    rad_to_deg,
    state_text,
)
from .theme import (
    C,
    SIZE_HERO,
    SIZE_LABEL,
    SIZE_MICRO,
    SIZE_TITLE,
    health_color,
    mono_font,
    numeric_font,
    state_color,
    ui_font,
    with_alpha,
)

#: Display smoothing time constant. 70 ms is long enough to kill the 50-to-60 Hz
#: stepping and short enough that the readout is not lying about the present.
SMOOTH_TAU_S = 0.070

#: Distance from the *picture* edge, not the widget edge. Cameras vignette, and
#: an instrument in the corner sits in the darkest part of the image.
MARGIN = 20

#: Below these widths a zone is hidden rather than overlapped. A HUD that
#: collides with itself is worse than one that is missing a panel.
MIN_WIDTH_DETAIL = 900
MIN_WIDTH_STATUS = 560

_ANIM_MS = 16


class ValueFollower:
    """A first-order smoother that knows when it has arrived.

    ``settled`` is the point: the animation timer stops when every follower has
    settled, so a parked car costs no repaints at all.
    """

    __slots__ = ("_value", "_target", "_tau")

    def __init__(self, initial: float = 0.0, tau: float = SMOOTH_TAU_S) -> None:
        self._value = initial
        self._target = initial
        self._tau = max(1e-4, tau)

    @property
    def value(self) -> float:
        return self._value

    @property
    def settled(self) -> bool:
        return abs(self._target - self._value) < 1e-3

    def set_target(self, target: float) -> None:
        self._target = target

    def snap(self, value: float) -> None:
        self._value = self._target = value

    def advance(self, dt: float) -> bool:
        """Step toward the target. Returns True while still moving."""
        if self.settled:
            self._value = self._target
            return False
        alpha = 1.0 - math.exp(-dt / self._tau)
        self._value += (self._target - self._value) * alpha
        return True


class _Zone(QWidget):
    """A transparent, click-through child of the video widget."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    def advance(self, dt: float) -> bool:  # pragma: no cover - overridden
        return False


def _scrim(painter: QPainter, rect: QRectF, radius: float = 8.0) -> None:
    """A dark rounded panel behind text.

    Not decoration: white text on a sunlit road is unreadable, and the camera
    has no idea what the HUD is about to draw on top of it.
    """
    painter.setPen(Qt.PenStyle.NoPen)
    painter.setBrush(with_alpha(C.bg_base, 0.62))
    painter.drawRoundedRect(rect, radius, radius)


def _draw_text(
    painter: QPainter,
    rect: QRectF,
    text: str,
    font: QFont,
    color: QColor,
    align: Qt.AlignmentFlag = Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
) -> None:
    painter.setFont(font)
    painter.setPen(color)
    painter.drawText(rect, int(align), text)


# --------------------------------------------------------------------------
# Bottom centre: speed, pedals, steering
# --------------------------------------------------------------------------


class WheelBar(_Zone):
    """Speed, throttle, brake and steering. The driving instruments.

    The steering arc carries two markers: what the app *commanded* (accent) and
    what the servo *reports* (cyan). They normally sit on top of each other, and
    the moment they separate is the moment the car has stopped doing what it was
    told -- which is worth seeing before you work it out from the picture.
    """

    WIDTH = 620
    HEIGHT = 150

    _ARC_SPAN_DEG = 124.0

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(self.WIDTH, self.HEIGHT)
        self._formatter = UnitFormatter()
        self._speed = ValueFollower()
        self._throttle = ValueFollower()
        self._brake = ValueFollower()
        self._steer_cmd = ValueFollower()
        self._steer_actual = ValueFollower()
        self._transmitting = False
        self._reverse = False

        self._font_hero = numeric_font(SIZE_HERO, QFont.Weight.Bold)
        self._font_unit = ui_font(SIZE_LABEL)
        self._font_label = ui_font(SIZE_MICRO, QFont.Weight.DemiBold)

    def set_formatter(self, formatter: UnitFormatter) -> None:
        self._formatter = formatter
        self.update()

    def set_vehicle(self, v: VehicleSnapshot) -> None:
        self._speed.set_target(abs(v.speed))
        self._steer_actual.set_target(max(-1.0, min(1.0, rad_to_deg(v.steer_angle) / 30.0)))
        self._reverse = v.speed < -0.02
        self.update()

    def set_input(self, i: InputSnapshot) -> None:
        self._throttle.set_target(i.throttle)
        self._brake.set_target(i.brake)
        self._steer_cmd.set_target(i.steering)
        self._transmitting = i.transmitting
        self.update()

    def advance(self, dt: float) -> bool:
        moving = False
        for follower in (
            self._speed,
            self._throttle,
            self._brake,
            self._steer_cmd,
            self._steer_actual,
        ):
            moving |= follower.advance(dt)
        return moving

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect())
        _scrim(painter, rect, 10.0)

        self._paint_speed(painter, QRectF(rect.left() + 18, rect.top() + 8, 210, 92))
        self._paint_pedals(painter, QRectF(rect.left() + 240, rect.top() + 20, 120, 82))
        self._paint_arc(painter, QRectF(rect.right() - 260, rect.top() + 14, 240, 120))
        painter.end()

    def _paint_speed(self, painter: QPainter, rect: QRectF) -> None:
        text = self._formatter.speed_text(self._speed.value)
        _draw_text(
            painter,
            rect,
            text,
            self._font_hero,
            C.text_primary,
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
        )
        width = QFontMetricsF(self._font_hero).horizontalAdvance(text)
        unit = QRectF(rect.right() + 6, rect.center().y() + 4, 60, 20)
        _draw_text(painter, unit, self._formatter.speed_suffix, self._font_unit, C.text_secondary)
        if self._reverse:
            badge = QRectF(rect.right() - width - 44, rect.center().y() - 10, 34, 20)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(with_alpha(C.warn, 0.22))
            painter.drawRoundedRect(badge, 4, 4)
            _draw_text(
                painter, badge, "R", self._font_label, C.warn, Qt.AlignmentFlag.AlignCenter
            )

    def _paint_pedals(self, painter: QPainter, rect: QRectF) -> None:
        # Greyed when nothing is going out. Without this the bars mirror the
        # wheel the driver is holding even when the packets are not reaching the
        # car, which is the single most misleading thing a HUD can do.
        throttle_color = C.accent if self._transmitting else C.text_tertiary
        brake_color = C.bad if self._transmitting else C.text_tertiary
        bar_w = 26.0
        gap = 16.0
        self._paint_bar(
            painter,
            QRectF(rect.left(), rect.top(), bar_w, rect.height() - 16),
            self._throttle.value,
            throttle_color,
            "THR",
        )
        self._paint_bar(
            painter,
            QRectF(rect.left() + bar_w + gap, rect.top(), bar_w, rect.height() - 16),
            self._brake.value,
            brake_color,
            "BRK",
        )

    def _paint_bar(
        self, painter: QPainter, rect: QRectF, value: float, color: QColor, label: str
    ) -> None:
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(with_alpha(C.text_primary, 0.10))
        painter.drawRoundedRect(rect, 3, 3)
        filled = max(0.0, min(1.0, value)) * rect.height()
        if filled > 0.5:
            painter.setBrush(color)
            painter.drawRoundedRect(
                QRectF(rect.left(), rect.bottom() - filled, rect.width(), filled), 3, 3
            )
        _draw_text(
            painter,
            QRectF(rect.left() - 4, rect.bottom() + 2, rect.width() + 8, 14),
            label,
            self._font_label,
            C.text_tertiary,
            Qt.AlignmentFlag.AlignCenter,
        )

    def _paint_arc(self, painter: QPainter, rect: QRectF) -> None:
        centre = QPointF(rect.center().x(), rect.bottom())
        radius = min(rect.width() * 0.5, rect.height()) - 6.0
        span = self._ARC_SPAN_DEG
        start = 90.0 - span * 0.5
        box = QRectF(centre.x() - radius, centre.y() - radius, radius * 2, radius * 2)

        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(with_alpha(C.text_primary, 0.16), 4.0, c=Qt.PenCapStyle.FlatCap))
        painter.drawArc(box, int(start * 16), int(span * 16))

        # Centre notch: the reference the driver checks against.
        painter.setPen(QPen(with_alpha(C.text_primary, 0.35), 2.0))
        painter.drawLine(
            QPointF(centre.x(), centre.y() - radius - 4),
            QPointF(centre.x(), centre.y() - radius + 6),
        )

        cmd = max(-1.0, min(1.0, self._steer_cmd.value))
        painter.setPen(
            QPen(
                C.accent if self._transmitting else C.text_tertiary,
                4.0,
                c=Qt.PenCapStyle.RoundCap,
            )
        )
        sweep = -cmd * span * 0.5
        painter.drawArc(box, int(90 * 16), int(sweep * 16))

        self._paint_marker(painter, centre, radius, cmd, span, C.accent, 7.0)
        self._paint_marker(
            painter, centre, radius, self._steer_actual.value, span, C.cyan, 4.0
        )

    def _paint_marker(
        self,
        painter: QPainter,
        centre: QPointF,
        radius: float,
        value: float,
        span: float,
        color: QColor,
        size: float,
    ) -> None:
        angle = math.radians(90.0 - value * span * 0.5)
        point = QPointF(
            centre.x() + math.cos(angle) * radius,
            centre.y() - math.sin(angle) * radius,
        )
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(color)
        painter.drawEllipse(point, size * 0.5, size * 0.5)


# --------------------------------------------------------------------------
# Top right: state and link health
# --------------------------------------------------------------------------


class StatusCluster(_Zone):
    """State badge, flag strip, channel pips and four numbers."""

    WIDTH = 290
    HEIGHT = 116

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFixedSize(self.WIDTH, self.HEIGHT)
        self._vehicle = VehicleSnapshot()
        self._link = LinkSnapshot()
        self._font_state = ui_font(SIZE_TITLE, QFont.Weight.DemiBold)
        self._font_label = ui_font(SIZE_MICRO, QFont.Weight.DemiBold)
        self._font_value = numeric_font(SIZE_LABEL)

    def set_vehicle(self, v: VehicleSnapshot) -> None:
        self._vehicle = v
        self.update()

    def set_link(self, link: LinkSnapshot) -> None:
        self._link = link
        self.update()

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect())
        _scrim(painter, rect, 10.0)

        v = self._vehicle
        label = state_text(v.state) if v.valid else "—"
        color = state_color(label) if v.valid else C.text_tertiary
        # A stale HUD reading ARMED means "unknown", not "safe". Dimming the
        # badge is how that gets said without a second indicator to read.
        if v.stale and v.valid:
            color = with_alpha(color, 0.45)
        _draw_text(
            painter,
            QRectF(rect.left() + 16, rect.top() + 10, 160, 28),
            label.upper(),
            self._font_state,
            color,
        )

        flags = hud_flag_texts(v.flags) if v.valid else []
        if flags:
            _draw_text(
                painter,
                QRectF(rect.left() + 16, rect.top() + 38, rect.width() - 32, 14),
                " · ".join(flags[:4]),
                self._font_label,
                C.warn,
            )

        self._paint_pips(painter, QRectF(rect.right() - 84, rect.top() + 16, 68, 14))
        self._paint_cells(
            painter, QRectF(rect.left() + 14, rect.bottom() - 44, rect.width() - 28, 34)
        )
        painter.end()

    def _paint_pips(self, painter: QPainter, rect: QRectF) -> None:
        link = self._link
        pips = (
            ("C", link.telemetry_age if link.state.usable else 99.0, 0.3, 0.6),
            ("T", link.telemetry_age if link.state.usable else 99.0, 0.3, 0.6),
            ("V", link.video_age if link.video_ok else 99.0, 0.5, 1.0),
        )
        x = rect.left()
        for label, age, warn, down in pips:
            color = health_color(age, warn=warn, down=down)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(QPointF(x + 4, rect.center().y()), 4, 4)
            _draw_text(
                painter,
                QRectF(x + 10, rect.top(), 14, rect.height()),
                label,
                self._font_label,
                C.text_tertiary,
            )
            x += 23

    def _paint_cells(self, painter: QPainter, rect: QRectF) -> None:
        link = self._link
        v = self._vehicle
        usable = link.state.usable
        cells = (
            ("ms", format_latency(link.rtt) if usable else "—",
             health_color(link.rtt, warn=0.08, down=0.20) if usable else C.text_tertiary),
            ("loss", f"{link.loss * 100.0:.1f}" if usable else "—",
             health_color(link.loss, warn=0.02, down=0.10) if usable else C.text_tertiary),
            ("fps", f"{link.video_fps:.0f}" if link.video_ok else "—",
             C.good if link.video_fps >= 20.0 else C.warn if link.video_ok else C.text_tertiary),
            ("V", format_voltage(v.pack_volts) if v.valid else "—",
             _volts_color(v.pack_volts) if v.valid else C.text_tertiary),
        )
        width = rect.width() / len(cells)
        for i, (label, value, color) in enumerate(cells):
            cell = QRectF(rect.left() + i * width, rect.top(), width, rect.height())
            _draw_text(
                painter,
                QRectF(cell.left(), cell.top(), cell.width(), 12),
                label,
                self._font_label,
                C.text_tertiary,
            )
            _draw_text(
                painter,
                QRectF(cell.left(), cell.top() + 12, cell.width(), 20),
                value,
                self._font_value,
                color,
            )


def _volts_color(volts: float) -> QColor:
    """Thresholds for a 6-cell NiMH pack boosted to 9 V.

    Below 6.6 V the boost regulator starts dropping out under load, which shows
    up as a brownout fault rather than as a slow fade, so amber has to come well
    before that.
    """
    if volts <= 0.0:
        return C.text_tertiary
    if volts < 6.6:
        return C.bad
    if volts < 7.0:
        return C.warn
    return C.good


# --------------------------------------------------------------------------
# Centre: faults
# --------------------------------------------------------------------------


class Severity(IntEnum):
    INFO = 0
    WARNING = 1
    CRITICAL = 2


#: Faults that ``build_events`` reports through a dedicated, more actionable
#: entry rather than by name. ESTOP_LATCHED is the vehicle state;
#: CALIBRATION_MISSING is a whole workflow, not a fault to be cleared.
_SELF_REPORTING_FAULTS = Fault.ESTOP_LATCHED | Fault.CALIBRATION_MISSING


@dataclass(frozen=True, slots=True)
class BannerEvent:
    severity: Severity
    title: str
    detail: str = ""

    @property
    def color(self) -> QColor:
        if self.severity is Severity.CRITICAL:
            return C.bad
        if self.severity is Severity.WARNING:
            return C.warn
        return C.text_secondary


def build_events(
    vehicle: VehicleSnapshot, link: LinkSnapshot, *, estop_key: str = "Ctrl+Shift+E"
) -> list[BannerEvent]:
    """Rank what the driver needs to know. Pure, so it can be asserted directly.

    Every message that describes a latched condition names the key that clears
    it. That is the lesson of the previous station's E-stop banner, which told
    the driver the car was latched and then left them to discover an unshortcut
    menu item.
    """
    events: list[BannerEvent] = []

    if vehicle.valid and vehicle.state is VehicleState.ESTOP:
        events.append(
            BannerEvent(
                Severity.CRITICAL,
                "E-STOP LATCHED",
                f"press {estop_key} to clear, then hold throttle at neutral to arm",
            )
        )
    if link.state is LinkState.FAILED:
        events.append(
            BannerEvent(
                Severity.CRITICAL,
                "LINK LOST",
                # The car E-stops the instant the session drops, so reconnecting
                # is only half the recovery. Saying so here saves the driver
                # working out why arming is refused.
                (link.detail or "session ended") + " — press ⌘K to reconnect",
            )
        )
    elif link.state is LinkState.STALE:
        events.append(
            BannerEvent(
                Severity.CRITICAL, "TELEMETRY STALE", link.detail or "the HUD is not current"
            )
        )
    elif link.state is LinkState.DEGRADED:
        events.append(BannerEvent(Severity.WARNING, "NO PICTURE", link.detail or "video down"))

    if vehicle.valid and vehicle.faults:
        # Two fault bits already have dedicated entries above and below, and
        # those entries say something useful about what to do next. Listing the
        # raw bit as well produces the same warning twice -- once actionable,
        # once not, and the useless copy is the one that says "press F to
        # clear", which does not clear either of them.
        reported = Fault(int(vehicle.faults) & ~int(_SELF_REPORTING_FAULTS))
        critical = Fault(int(reported) & int(CRITICAL_FAULTS))
        other = Fault(int(reported) & ~int(CRITICAL_FAULTS))
        for text in fault_texts(critical):
            events.append(BannerEvent(Severity.CRITICAL, text.upper()))
        for text in fault_texts(other):
            events.append(BannerEvent(Severity.WARNING, text.upper(), "press F to clear"))

    if vehicle.valid and not vehicle.calibrated:
        events.append(
            BannerEvent(
                Severity.WARNING,
                "NOT CALIBRATED",
                "speed and steering are guesses until pi/scripts/calibrate_drive.py runs",
            )
        )

    # Belt and braces: a fault the firmware grows later must not be able to
    # collide with one of the hand-written entries and print twice.
    seen: set[str] = set()
    unique: list[BannerEvent] = []
    for event in events:
        if event.title in seen:
            continue
        seen.add(event.title)
        unique.append(event)
    return unique


class EventBanner(_Zone):
    """Up to three ranked events, pulsing so they are not mistaken for chrome."""

    WIDTH = 580
    MAX_EVENTS = 3
    _PULSE_HZ = 1.5

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._events: list[BannerEvent] = []
        self._phase = 0.0
        self._font_title = ui_font(SIZE_LABEL, QFont.Weight.Bold)
        self._font_detail = ui_font(SIZE_MICRO)
        self.setFixedWidth(self.WIDTH)
        self.setFixedHeight(1)

    def set_events(self, events: list[BannerEvent]) -> None:
        if events == self._events:
            return
        self._events = events[: self.MAX_EVENTS]
        self.setFixedHeight(max(1, len(self._events) * 44))
        self.setVisible(bool(self._events))
        self.update()

    @property
    def events(self) -> list[BannerEvent]:
        return list(self._events)

    def advance(self, dt: float) -> bool:
        if not self._events:
            return False
        self._phase = (self._phase + dt * self._PULSE_HZ) % 1.0
        self.update()
        return True

    def paintEvent(self, event: QPaintEvent) -> None:
        if not self._events:
            return
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        # 55..100 % rather than 0..100 %: a banner that fades to nothing is one
        # the driver can glance at and miss entirely.
        alpha = 0.55 + 0.45 * (0.5 + 0.5 * math.sin(self._phase * math.tau))
        y = 0.0
        for item in self._events:
            rect = QRectF(0, y, float(self.width()), 40)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(with_alpha(C.bg_base, 0.72))
            painter.drawRoundedRect(rect, 6, 6)
            painter.setBrush(with_alpha(item.color, 0.9 * alpha))
            painter.drawRoundedRect(QRectF(rect.left(), rect.top(), 3, rect.height()), 1.5, 1.5)
            _draw_text(
                painter,
                QRectF(rect.left() + 14, rect.top() + 5, rect.width() - 24, 16),
                item.title,
                self._font_title,
                with_alpha(item.color, alpha),
            )
            if item.detail:
                _draw_text(
                    painter,
                    QRectF(rect.left() + 14, rect.top() + 21, rect.width() - 24, 14),
                    item.detail,
                    self._font_detail,
                    C.text_secondary,
                )
            y += 44
        painter.end()


# --------------------------------------------------------------------------
# Top left: the detail row
# --------------------------------------------------------------------------


class DetailRow(_Zone):
    """One row of numbers, hidden by default.

    The only debugging affordance left, and it is deliberately about the *app
    and the link* rather than the drivetrain: it is how you tell "the link is
    slow" from "the decoder is slow", which is the question a laggy picture
    actually poses.
    """

    HEIGHT = 26

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._font = mono_font(SIZE_MICRO)
        self._text = ""
        self.setFixedHeight(self.HEIGHT)
        self.setVisible(False)

    def set_state(self, link: LinkSnapshot, vehicle: VehicleSnapshot) -> None:
        text = (
            f"vid {link.video_latency * 1000:5.0f}ms"
            f"  dec {link.video_decode_ms:4.1f}ms"
            f"  fps {link.video_fps:4.1f}"
            f"  ctrl {link.control_hz:5.1f}Hz"
            f"  tlm {link.telemetry_hz:4.1f}Hz"
            f"  loop p99 {vehicle.loop_p99 * 1000:4.1f}ms"
            f"  soc {format_temperature(vehicle.cpu_temp_c)}C"
        )
        if text != self._text:
            self._text = text
            metrics = QFontMetricsF(self._font)
            self.setFixedWidth(int(metrics.horizontalAdvance(text)) + 24)
            self.update()

    def paintEvent(self, event: QPaintEvent) -> None:
        if not self._text:
            return
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(self.rect())
        _scrim(painter, rect, 5.0)
        _draw_text(
            painter,
            QRectF(rect.left() + 12, rect.top(), rect.width() - 24, rect.height()),
            self._text,
            self._font,
            C.text_secondary,
        )
        painter.end()


# --------------------------------------------------------------------------
# The overlay
# --------------------------------------------------------------------------


class HudOverlay(QWidget):
    """Places the four zones and owns the one animation timer.

    One timer, not four: it runs only while some zone is still moving, so a
    parked, disconnected app repaints nothing at all.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.wheel = WheelBar(self)
        self.status = StatusCluster(self)
        self.banner = EventBanner(self)
        self.detail = DetailRow(self)

        self._instruments_visible = True
        self._detail_visible = False
        self._vehicle = VehicleSnapshot()
        self._link = LinkSnapshot()

        self._timer = QTimer(self)
        self._timer.setInterval(_ANIM_MS)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._animate)

    # -- configuration ------------------------------------------------------

    def set_formatter(self, formatter: UnitFormatter) -> None:
        self.wheel.set_formatter(formatter)

    def set_instruments_visible(self, visible: bool) -> None:
        self._instruments_visible = visible
        self._relayout()

    def set_detail_visible(self, visible: bool) -> None:
        self._detail_visible = visible
        self._relayout()

    def toggle_detail(self) -> bool:
        self.set_detail_visible(not self._detail_visible)
        return self._detail_visible

    @property
    def detail_visible(self) -> bool:
        return self._detail_visible

    # -- data ---------------------------------------------------------------

    def set_vehicle(self, v: VehicleSnapshot) -> None:
        self._vehicle = v
        self.wheel.set_vehicle(v)
        self.status.set_vehicle(v)
        self._refresh_derived()

    def set_link(self, link: LinkSnapshot) -> None:
        self._link = link
        self.status.set_link(link)
        self._refresh_derived()

    def set_input(self, sample: InputSnapshot) -> None:
        self.wheel.set_input(sample)
        self._kick()

    def _refresh_derived(self) -> None:
        self.banner.set_events(build_events(self._vehicle, self._link))
        self.detail.set_state(self._link, self._vehicle)
        self._relayout()
        self._kick()

    # -- animation ----------------------------------------------------------

    def _kick(self) -> None:
        if not self._timer.isActive():
            self._timer.start()

    def _animate(self) -> None:
        dt = _ANIM_MS / 1000.0
        moving = False
        for zone in (self.wheel, self.status, self.banner, self.detail):
            if zone.isVisible():
                moving |= zone.advance(dt)
        if not moving:
            self._timer.stop()

    # -- geometry -----------------------------------------------------------

    def resizeEvent(self, event: QResizeEvent) -> None:
        self._relayout()
        super().resizeEvent(event)

    def _relayout(self) -> None:
        w = self.width()
        h = self.height()
        if w <= 0 or h <= 0:
            return

        show_instruments = self._instruments_visible
        self.wheel.setVisible(show_instruments and w >= 480)
        self.status.setVisible(show_instruments and w >= MIN_WIDTH_STATUS)
        self.detail.setVisible(self._detail_visible and w >= MIN_WIDTH_DETAIL)

        if self.wheel.isVisible():
            self.wheel.move((w - self.wheel.width()) // 2, h - MARGIN - self.wheel.height())
        if self.status.isVisible():
            self.status.move(w - MARGIN - self.status.width(), MARGIN)
        if self.detail.isVisible():
            self.detail.move(MARGIN, MARGIN)

        if self.banner.events:
            self.banner.setVisible(True)
            x = (w - self.banner.width()) // 2
            y = int(h * 0.32)
            # Never let the banner sit on the instruments: it is the one thing
            # that must stay readable when everything else has gone wrong.
            if self.wheel.isVisible():
                y = min(y, self.wheel.y() - self.banner.height() - 12)
            self.banner.move(x, max(MARGIN, y))
        else:
            self.banner.setVisible(False)


__all__ = [
    "SMOOTH_TAU_S",
    "BannerEvent",
    "DetailRow",
    "EventBanner",
    "HudOverlay",
    "Severity",
    "StatusCluster",
    "ValueFollower",
    "WheelBar",
    "build_events",
]
