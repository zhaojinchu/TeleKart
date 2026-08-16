"""Backend selection, with the null backend as an unremovable floor.

`create()` cannot fail. Whatever goes wrong -- a missing driver library, a
device that refuses to open, a backend module that raises on import -- the
caller gets a working `FfbBackend` object and a reason string it can show. The
app is fully functional with force feedback disabled, so there is no situation in
which failing to obtain a backend should stop anything.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Callable

from .base import FfbBackend, FfbTuning
from .null import NullFfb

#: Environment override, mostly for bisecting a backend problem.
ENV_OVERRIDE = "TELEKART_FFB"

BackendFactory = Callable[[], FfbBackend]


@dataclass(frozen=True, slots=True)
class _Entry:
    backend_id: str
    label: str
    factory: BackendFactory
    #: Higher wins under "auto". Null is pinned below everything.
    priority: int


_REGISTRY: dict[str, _Entry] = {}


class RegistryError(ValueError):
    """A backend registration is malformed. Raised at import time, not at use."""


def register(
    backend_id: str,
    factory: BackendFactory,
    *,
    label: str = "",
    priority: int = 10,
) -> None:
    """Add a backend. Call at import time from the backend's own module."""
    if not backend_id:
        raise RegistryError("a backend needs a non-empty id")
    if backend_id == "null" and priority > -1:
        raise RegistryError("the null backend is registered by this module only")
    _REGISTRY[backend_id] = _Entry(
        backend_id=backend_id, label=label or backend_id, factory=factory, priority=priority
    )


def unregister(backend_id: str) -> bool:
    """Remove a backend. Refuses to remove null, which must always exist."""
    if backend_id == "null":
        raise RegistryError("the null backend cannot be unregistered")
    return _REGISTRY.pop(backend_id, None) is not None


def registered() -> tuple[str, ...]:
    """Every registered id, best-first. Includes ids whose driver is missing."""
    return tuple(
        entry.backend_id
        for entry in sorted(_REGISTRY.values(), key=lambda e: -e.priority)
    )


def labels() -> dict[str, str]:
    return {entry.backend_id: entry.label for entry in _REGISTRY.values()}


def available() -> tuple[str, ...]:
    """Ids that report themselves usable on this machine, best-first.

    Probing constructs each backend, so this is a cold path -- call it when the
    settings screen opens, not on every tick.
    """
    usable: list[str] = []
    for entry in sorted(_REGISTRY.values(), key=lambda e: -e.priority):
        backend = _try_build(entry)
        if backend is not None and backend.available:
            usable.append(entry.backend_id)
    return tuple(usable)


def create(name: str = "auto", *, tuning: FfbTuning | None = None) -> FfbBackend:
    """Build a backend. Always returns one.

    `auto` picks the highest-priority available backend, falling back to null.
    The environment variable wins over `auto` but not over an explicit name, so a
    developer can force a backend globally without breaking a caller that has
    already made a deliberate choice.
    """
    if name == "auto":
        override = os.environ.get(ENV_OVERRIDE, "").strip()
        if override:
            name = override

    if name in ("", "auto"):
        for entry in sorted(_REGISTRY.values(), key=lambda e: -e.priority):
            if entry.backend_id == "null":
                continue
            backend = _try_build(entry, tuning)
            if backend is not None and backend.available:
                return backend
        return NullFfb(tuning, reason="no force-feedback backend is available")

    entry = _REGISTRY.get(name)
    if entry is None:
        return NullFfb(tuning, reason=f"unknown force-feedback backend {name!r}")
    backend = _try_build(entry, tuning)
    if backend is None:
        return NullFfb(tuning, reason=f"{name} failed to initialise")
    if not backend.available:
        return NullFfb(tuning, reason=f"{name} is not available on this machine")
    return backend


def _try_build(entry: _Entry, tuning: FfbTuning | None = None) -> FfbBackend | None:
    try:
        backend = entry.factory()
    except Exception:
        # A backend whose driver library is half-installed raises from its
        # constructor. That is a reason to skip it, never a reason to stop.
        return None
    if tuning is not None:
        backend.set_tuning(tuning)
    return backend


# The floor. Registered here rather than in null.py so that importing the
# registry is sufficient to guarantee it exists, whatever import order the app
# happens to use.
_REGISTRY["null"] = _Entry(
    backend_id="null",
    label=NullFfb.label,
    factory=NullFfb,
    priority=-100,
)
