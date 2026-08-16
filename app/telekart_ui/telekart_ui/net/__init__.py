"""The network stack: resolution, session, control TX, telemetry RX, supervision.

No Qt anywhere in this package. That is what lets a test drive the whole stack
with no widgets and no display.
"""

from __future__ import annotations

from . import discovery
from .control_tx import ControlCommand, ControlCommandLike, ControlTxStats, ControlTxThread
from .discovery import DEFAULT_HOSTNAME, Target, resolve, split_host_port
from .link_manager import SUPERVISOR_HZ, LinkManager
from .session_client import SessionClient, SessionEvent, SessionEventKind, SessionInfo
from .telemetry_rx import TelemetryRxThread, TelemetrySample, TelemetryStats

__all__ = [
    "discovery",
    "DEFAULT_HOSTNAME",
    "Target",
    "resolve",
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
    "SUPERVISOR_HZ",
]
