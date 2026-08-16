"""The toggleable diagnostic dump, and the reason the main HUD stays readable.

Every raw number the app knows lives here and **nowhere else**: duty cycles,
encoder-derived RPM, the true SI speed behind the scaled speedometer, packet
counters, the axis floats that were actually transmitted. Off by default, bound
to a key, and gone the moment the driver stops debugging.

Splitting it out is what keeps the driving HUD a driving HUD. Every teleop
overlay drifts toward a debug dump because every individual number looks
justified in isolation -- and the result is a screen where the one value that
matters is surrounded by nineteen that do not, and the driver reads none of
them. The rule that makes the split hold is mechanical rather than tasteful:
if a number is diagnostic, it goes here, and no exceptions are granted for
numbers that are *nearly* driving-relevant.

Strings are rebuilt when a snapshot arrives, not when the widget paints. The
snapshots already change at telemetry rate and only when a value moved past its
epsilon, so a parked car costs one formatting pass rather than sixty a second.
"""

from __future__ import annotations

import math

from PySide6.QtCore import QRectF, QSize, Qt
from PySide6.QtGui import QPainter
from PySide6.QtWidgets import QWidget

from ...model.snapshots import InputSnapshot, LinkSnapshot, VehicleSnapshot
from ...model.units import flag_texts, state_text
from ..theme import fonts
from ..theme.tokens import THEME, Theme
from ..widgets.base import PaintedWidget
from . import draw_scrim, label_color, micro_label_font

_PAD = 12.0
_COL_W = 232.0
_ROW_H = 14.0
_HEADER_H = 20.0
_GUTTER = 10.0

Row = tuple[str, str]
Section = tuple[str, list[Row]]


class DiagOverlay(PaintedWidget):
    """Three columns of monospaced key/value pairs: drive, link, input."""

    def __init__(self, parent: QWidget | None = None, *, theme: Theme = THEME) -> None:
        super().__init__(parent, theme=theme)

        self._sections: list[Section] = [
            ("drive", []),
            ("link", []),
            ("input", []),
        ]

        self._font_row = fonts.mono_font(theme.type.micro, theme.weight.regular, theme=theme)
        self._font_header = micro_label_font(theme)

        self._scrim = QRectF()
        self._columns: list[tuple[QRectF, list[tuple[QRectF, QRectF]]]] = []

        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground, True)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.setVisible(False)
        self.resize(self.sizeHint())

    # -- inputs -------------------------------------------------------------

    def set_vehicle(self, v: VehicleSnapshot) -> None:
        self._sections[0] = ("drive", drive_rows(v))
        self._relayout()

    def set_link(self, link: LinkSnapshot) -> None:
        self._sections[1] = ("link", link_rows(link))
        self._relayout()

    def set_input(self, sample: InputSnapshot) -> None:
        self._sections[2] = ("input", input_rows(sample))
        self._relayout()

    def _relayout(self) -> None:
        wanted = self.sizeHint()
        if wanted != self.size():
            self.resize(wanted)
        # Unconditionally, not only on a resize. The cell rectangles are built
        # per row, and a section that gains rows without changing the widget's
        # overall height -- which is what happens whenever a *different*
        # section is already the tallest -- would otherwise keep the old, short
        # cell list and silently stop drawing its extra rows.
        self.on_layout(self.width(), self.height())
        self.invalidate_static()

    # -- geometry -----------------------------------------------------------

    def sizeHint(self) -> QSize:
        rows = max((len(rows) for _, rows in self._sections), default=0)
        width = _PAD * 2.0 + _COL_W * 3.0 + _GUTTER * 2.0
        height = _PAD * 2.0 + _HEADER_H + max(rows, 1) * _ROW_H
        return QSize(int(width + 0.5), int(height + 0.5))

    def on_layout(self, width: int, height: int) -> None:
        if width <= 0 or height <= 0:
            return
        self._scrim = QRectF(0.0, 0.0, float(width), float(height))
        self._columns = []
        for index, (_, rows) in enumerate(self._sections):
            left = _PAD + index * (_COL_W + _GUTTER)
            header = QRectF(left, _PAD, _COL_W, _HEADER_H)
            cells: list[tuple[QRectF, QRectF]] = []
            y = _PAD + _HEADER_H
            for _ in rows:
                cells.append(
                    (
                        QRectF(left, y, _COL_W * 0.52, _ROW_H),
                        QRectF(left + _COL_W * 0.52, y, _COL_W * 0.48, _ROW_H),
                    )
                )
                y += _ROW_H
            self._columns.append((header, cells))

    # -- painting -----------------------------------------------------------

    def paint_static(self, painter: QPainter, width: int, height: int) -> None:
        theme = self.theme
        draw_scrim(painter, self._scrim, theme.radius.card, alpha=0.68)

        key_color = label_color(theme)
        value_color = theme.q.text_primary
        # Truncating zip, never strict. These two lists are rebuilt together,
        # but a paintEvent that raises is a widget that stops updating with no
        # traceback anywhere -- so a transient mismatch draws fewer rows for one
        # frame instead of taking the HUD down.
        for (title, rows), (header, cells) in zip(self._sections, self._columns):
            painter.setFont(self._font_header)
            painter.setPen(theme.q.cyan)
            painter.drawText(
                header,
                int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                title,
            )
            painter.setFont(self._font_row)
            for (key, value), (key_rect, value_rect) in zip(rows, cells):
                painter.setPen(key_color)
                painter.drawText(
                    key_rect,
                    int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
                    key,
                )
                painter.setPen(value_color)
                painter.drawText(
                    value_rect,
                    int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter),
                    value,
                )

    def paint_dynamic(self, painter: QPainter, width: int, height: int) -> None:
        """Deliberately empty: the panel is static between snapshots.

        Everything is drawn into the cached pixmap instead, so the intermediate
        repaints the rest of the HUD forces on this widget cost one blit. This
        overlay is the largest thing on the screen when it is up, and that
        saving is the difference between a debug view you can drive with and
        one that halves the frame rate.
        """


# --------------------------------------------------------------------------
# Row builders
# --------------------------------------------------------------------------


def _pct(value: float) -> str:
    return "%+.1f%%" % (value * 100.0,)


def _ms(seconds: float) -> str:
    return "%.1f" % (seconds * 1000.0,)


def drive_rows(v: VehicleSnapshot) -> list[Row]:
    x, y, heading = v.pose
    return [
        ("state", state_text(v.state)),
        ("valid/stale", ("y" if v.valid else "n") + "/" + ("y" if v.stale else "n")),
        ("faults", "0x%05X" % (int(v.faults),)),
        ("flags", " ".join(flag_texts(v.flags)) or "-"),
        ("duty l", _pct(v.duty_l)),
        ("duty r", _pct(v.duty_r)),
        ("rpm l", "%.1f" % (v.rpm_l,)),
        ("rpm r", "%.1f" % (v.rpm_r,)),
        ("rpm tgt l", "%.1f" % (v.rpm_target_l,)),
        ("rpm tgt r", "%.1f" % (v.rpm_target_r,)),
        ("servo", "%d us" % (v.servo_us,)),
        ("steer", "%+.2f deg" % (math.degrees(v.steer_angle),)),
        ("speed", "%.3f m/s" % (v.speed,)),
        ("v_max", "%.3f m/s" % (v.v_max,)),
        ("slip", "%.3f" % (v.slip,)),
        ("distance", "%.2f m" % (v.distance,)),
        ("pose x/y", "%+.2f %+.2f" % (x, y)),
        ("heading", "%+.1f deg" % (math.degrees(heading),)),
        ("pack", "%.2f V" % (v.pack_volts,)),
        ("soc temp", "%.1f C" % (v.cpu_temp_c,)),
        ("throttled", "0x%05X" % (v.throttled,)),
        ("loop p99", _ms(v.loop_p99) + " ms"),
        ("tlm seq", "%d" % (v.sequence,)),
    ]


def link_rows(link: LinkSnapshot) -> list[Row]:
    return [
        ("state", link.state.value),
        ("detail", link.detail or "-"),
        ("car", link.car_id or link.host or "-"),
        ("address", link.address or "-"),
        ("session", "%d" % (link.session_id,)),
        ("tlm age", _ms(link.telemetry_age) + " ms"),
        ("vid age", _ms(link.video_age) + " ms"),
        ("ses age", "%.1f s" % (link.session_age,)),
        ("rtt", _ms(link.rtt) + " ms"),
        ("rtt p95", _ms(link.rtt_p95) + " ms"),
        ("rtt min", _ms(link.rtt_min) + " ms"),
        ("tlm rate", "%.1f Hz" % (link.telemetry_hz,)),
        ("ctrl rate", "%.1f Hz" % (link.control_hz,)),
        ("video", "%.1f fps" % (link.video_fps,)),
        ("bitrate", "%.0f kbps" % (link.video_bitrate / 1000.0,)),
        ("vid latency", _ms(link.video_latency) + " ms"),
        ("loss", "%.2f%%" % (link.loss * 100.0,)),
        ("tlm pkts", "%d" % (link.telemetry_packets,)),
        ("tlm lost", "%d" % (link.telemetry_lost,)),
        ("tlm bad", "%d" % (link.telemetry_bad,)),
        ("ctrl sent", "%d" % (link.control_sent,)),
        ("ctrl errs", "%d" % (link.control_errors,)),
        ("frames", "%d / %d" % (link.video_frames, link.video_dropped)),
    ]


def input_rows(sample: InputSnapshot) -> list[Row]:
    return [
        ("device", sample.device or "-"),
        ("connected", "y" if sample.device_connected else "n"),
        ("transmitting", "y" if sample.transmitting else "n"),
        ("steer", "%+.4f" % (sample.steering,)),
        ("throttle", "%.4f" % (sample.throttle,)),
        ("brake", "%.4f" % (sample.brake,)),
        ("flags", "0x%02X" % (int(sample.flags),)),
        ("arm intent", "y" if sample.arm_intent else "n"),
        ("estop", "y" if sample.estop_latched else "n"),
        ("ctrl seq", "%d" % (sample.sequence,)),
        ("cmd age", _ms(sample.command_age) + " ms"),
        ("tx rate", "%.1f Hz" % (sample.rate_hz,)),
    ]


__all__ = ["DiagOverlay", "Row", "drive_rows", "input_rows", "link_rows"]
