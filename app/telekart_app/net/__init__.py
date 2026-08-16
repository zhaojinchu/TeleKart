"""The network stack: discovery, session, control TX, telemetry RX, supervision.

No Qt anywhere in this package. That is what lets the integration suite drive
the whole stack against the simulator with no widgets and no display.
"""

from __future__ import annotations

from . import discovery
from .control_tx import ControlCommand, ControlCommandLike, ControlTxStats, ControlTxThread
from .discovery import DiscoveredCar, ZeroconfBrowser, find_car, resolve, split_host_port
from .link_manager import (
    SUPERVISOR_HZ,
    TELEMETRY_STALE_S,
    VIDEO_STALE_S,
    LinkManager,
)
from .session_client import SessionClient, SessionEvent, SessionEventKind, SessionInfo
from .telemetry_rx import TelemetryRxThread, TelemetrySample, TelemetryStats

__all__ = [
    "discovery",
    "DiscoveredCar",
    "ZeroconfBrowser",
    "resolve",
    "find_car",
    "split_host_port",
    "SessionClient",
    "SessionInfo",
    "SessionEvent",
    "SessionEventKind",
    "ControlTxThread",
    "ControlCommand",
    "ControlCommandLike",
    "ControlTxStats",
    "TelemetryRxThread",
    "TelemetrySample",
    "TelemetryStats",
    "LinkManager",
    "TELEMETRY_STALE_S",
    "VIDEO_STALE_S",
    "SUPERVISOR_HZ",
]
