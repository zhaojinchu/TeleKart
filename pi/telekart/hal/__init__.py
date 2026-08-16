"""Hardware abstraction: one interface, one real backend, one simulated one.

Only :mod:`telekart.hal.base` is imported eagerly. Pulling in the pigpio backend
would import ``pigpio`` on a machine that may not have it, and pulling in the
mock would drag the plant model into the firmware process for no reason. Use
:func:`select_backend`, or import the concrete class directly when a test wants
a specific one.
"""

from __future__ import annotations

from .base import (
    BACKEND_AUTO,
    BACKEND_MOCK,
    BACKEND_PIGPIO,
    CallbackHandle,
    Edge,
    EdgeCallback,
    GpioBackend,
    GpioError,
    Pull,
    select_backend,
    tick_diff,
)

__all__ = [
    "GpioBackend",
    "GpioError",
    "Pull",
    "Edge",
    "EdgeCallback",
    "CallbackHandle",
    "tick_diff",
    "select_backend",
    "BACKEND_AUTO",
    "BACKEND_PIGPIO",
    "BACKEND_MOCK",
]
