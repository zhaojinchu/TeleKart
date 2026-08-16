"""The control layer: shaping, mixing, regulation, safety, and the loop body.

Import order inside this package runs one way only -- ``shaping`` and ``pid``
depend on nothing, ``mixer`` and ``safety`` depend on the config, and ``drive``
composes all of them with the drivers. Nothing here imports the network or
telemetry layers: the control loop must be constructible, runnable and testable
with no socket in the process.
"""

from __future__ import annotations

from .drive import DriveCommand, DriveController, DriveState
from .mixer import DifferentialMixer, WheelTargets
from .pid import PID
from .safety import FeedbackSnapshot, SafetyOutput, SafetyStateMachine
from .shaping import (
    ExpoCurve,
    clamp,
    deadzone,
    expo,
    lerp,
    rate_limit,
    speed_sensitive_scale,
    wrap_pi,
)

__all__ = [
    "DriveController",
    "DriveCommand",
    "DriveState",
    "DifferentialMixer",
    "WheelTargets",
    "PID",
    "SafetyStateMachine",
    "SafetyOutput",
    "FeedbackSnapshot",
    "ExpoCurve",
    "clamp",
    "deadzone",
    "expo",
    "lerp",
    "rate_limit",
    "speed_sensitive_scale",
    "wrap_pi",
]
