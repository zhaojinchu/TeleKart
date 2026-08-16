"""The backend that is always there.

Every code path that touches force feedback goes through a backend object, and
this is the one that is guaranteed to exist. There is no `if ffb is not None`
anywhere in the app, and no configuration in which force feedback is "off" in a
way that needs its own branch -- it is always on, and sometimes it is this.

It is not quite a no-op. It records the last value each effect was asked for,
which costs one attribute store and buys two real things: the settings screen
can show what the feel model *would* be doing, and the tests can assert the feel
model's behaviour without any hardware at all.
"""

from __future__ import annotations

from typing import Any

from .base import (
    NO_CAPABILITIES,
    SILENT_COMMAND,
    FfbBackend,
    FfbCapabilities,
    FfbCommand,
    FfbState,
    FfbTuning,
)


class NullFfb(FfbBackend):
    """No device, no forces, always available."""

    backend_id = "null"
    label = "None (no force feedback)"

    __slots__ = (
        "_open",
        "_reason",
        "last_spring",
        "last_autocenter",
        "last_constant",
        "last_damper",
        "last_friction",
        "last_rumble",
        "last_command",
        "updates",
    )

    def __init__(
        self, tuning: FfbTuning | None = None, *, reason: str = ""
    ) -> None:
        super().__init__(tuning)
        self._open = False
        #: Why the app ended up here. Empty when null was chosen deliberately;
        #: otherwise it carries the failure that made a real backend unusable,
        #: which is the difference between "this wheel has no FFB" and "the
        #: driver library is missing" on the settings screen.
        self._reason = reason
        self.last_spring: tuple[float, float] = (0.0, 0.0)
        self.last_autocenter: float = 0.0
        self.last_constant: float = 0.0
        self.last_damper: float = 0.0
        self.last_friction: float = 0.0
        self.last_rumble: tuple[float, float, float] = (0.0, 0.0, 0.0)
        #: What the feel model asked for on the last tick, unfiltered by
        #: capabilities. This is what the settings screen previews.
        self.last_command: FfbCommand = SILENT_COMMAND
        self.updates: int = 0

    @property
    def available(self) -> bool:
        return True

    @property
    def capabilities(self) -> FfbCapabilities:
        return NO_CAPABILITIES

    @property
    def is_open(self) -> bool:
        return self._open

    @property
    def reason(self) -> str:
        return self._reason

    def open(self, *, guid: str = "", device: Any = None) -> bool:
        self._open = True
        return True

    def close(self) -> None:
        self._open = False
        self.stop_all()

    # -- recording -----------------------------------------------------------

    def set_autocenter(self, strength: float) -> None:
        self.last_autocenter = strength

    def set_constant_force(self, force: float) -> None:
        self.last_constant = force

    def set_spring(self, center: float, strength: float) -> None:
        self.last_spring = (center, strength)

    def set_damper(self, strength: float) -> None:
        self.last_damper = strength

    def set_friction(self, strength: float) -> None:
        self.last_friction = strength

    def rumble(self, low: float, high: float, duration: float) -> None:
        self.last_rumble = (low, high, duration)

    def stop_all(self) -> None:
        self.last_spring = (0.0, 0.0)
        self.last_autocenter = 0.0
        self.last_constant = 0.0
        self.last_damper = 0.0
        self.last_friction = 0.0
        self.last_rumble = (0.0, 0.0, 0.0)

    def update(self, state: FfbState, dt: float) -> None:
        # The model runs even though nothing reaches a motor. Running the real
        # model rather than a preview means the preview cannot drift away from
        # the behaviour a real backend would produce.
        self.updates += 1
        if not self._open:
            return
        self.last_command = self.compute(state, dt)
        super().update(state, dt)
