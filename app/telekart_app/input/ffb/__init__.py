"""Pluggable force feedback.

The wheel this app was built for has none, so the null backend is what actually
runs. The seam exists because force feedback cannot be added later without
touching the input thread, the telemetry consumer and the profile format at
once, and one small package now is cheaper than that later.

Adding a real backend means: subclass `FfbBackend`, implement `open`, `close`,
`is_open`, `capabilities` and whichever effect setters the hardware has, then
call `registry.register()` from the new module. Nothing else in the app changes,
and the feel model in `base.compute` comes along for free.
"""

from __future__ import annotations

from .base import (
    NEUTRAL_STATE,
    NO_CAPABILITIES,
    SILENT_COMMAND,
    FfbBackend,
    FfbCapabilities,
    FfbCommand,
    FfbState,
    FfbTuning,
)
from .null import NullFfb
from .registry import (
    ENV_OVERRIDE,
    RegistryError,
    available,
    create,
    labels,
    register,
    registered,
    unregister,
)

#: Friendlier alias for callers outside this package, where a bare `create()`
#: would read as "create what?".
create_ffb = create

__all__ = [
    "ENV_OVERRIDE",
    "FfbBackend",
    "FfbCapabilities",
    "FfbCommand",
    "FfbState",
    "FfbTuning",
    "NEUTRAL_STATE",
    "NO_CAPABILITIES",
    "NullFfb",
    "RegistryError",
    "SILENT_COMMAND",
    "available",
    "create",
    "create_ffb",
    "labels",
    "register",
    "registered",
    "unregister",
]
