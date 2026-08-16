"""Custom instruments and containers.

Every widget here is standalone: construct it, feed it numbers, put it in a
layout. None of them import the model, the network stack, or the protocol
package, which is what makes the whole set exercisable from a gallery script
with no car, no simulator and no sockets -- and what means a rendering bug can
be reproduced without any of those either.

They all read the same :mod:`telekart_app.ui.theme.tokens` object the
stylesheet reads. That shared token object is the single decision that keeps
the painted instruments and the styled controls looking like one application.
"""

from __future__ import annotations

from .axis_bar import AxisBar
from .bar_meter import BarMeter
from .base import (
    PaintedWidget,
    ValueFollower,
    format_clock,
    format_fixed,
    format_signed,
)
from .card import Card
from .curve_editor import LUT_SIZE, CurveEditor
from .delta_bar import DeltaBar
from .gauge import SpeedGauge
from .led_strip import ShiftLights
from .link_quality import CHANNELS, LinkQuality
from .rail import RAIL_WIDTH, NavigationRail
from .sector_lights import SectorLights, SectorStatus
from .stat_tile import StatTile
from .steering import SteeringIndicator
from .track_map import MapMarker, TrackMap

__all__ = [
    "CHANNELS",
    "LUT_SIZE",
    "RAIL_WIDTH",
    "AxisBar",
    "BarMeter",
    "Card",
    "CurveEditor",
    "DeltaBar",
    "LinkQuality",
    "MapMarker",
    "NavigationRail",
    "PaintedWidget",
    "SectorLights",
    "SectorStatus",
    "ShiftLights",
    "SpeedGauge",
    "StatTile",
    "SteeringIndicator",
    "TrackMap",
    "ValueFollower",
    "format_clock",
    "format_fixed",
    "format_signed",
]
