"""The single seam between the firmware and the hardware.

Every GPIO access in the codebase goes through :class:`GpioBackend`. That is
what makes :class:`~telekart.hal.mock_backend.MockBackend` -- and therefore
Mac-side development of most of the firmware -- possible, and what would make
swapping pigpio for sysfs PWM or lgpio a one-file change.

The surface is deliberately small and slightly awkward in one place: PWM is a
*pair* operation. See :meth:`GpioBackend.set_pwm_pair`.
"""

from __future__ import annotations

import enum
import os
import platform
from abc import ABC, abstractmethod
from typing import Any, Callable, Protocol, runtime_checkable

#: pigpio's tick is a 32-bit microsecond counter, so it wraps every ~71.6 minutes.
TICK_WRAP = 1 << 32
_TICK_HALF = 1 << 31

#: pigpio delivers 2 as the level when a watchdog fires rather than a real edge.
LEVEL_TIMEOUT = 2


class GpioError(RuntimeError):
    """A GPIO operation failed in a way the caller cannot paper over.

    Raised at construction and configuration time only. Once the control loop is
    running, backends record the failure and expose it through
    :meth:`GpioBackend.pop_gpio_error`, because raising out of a control tick
    leaves the H-bridge in whatever state it was last commanded into.
    """


class Pull(enum.Enum):
    NONE = enum.auto()
    UP = enum.auto()
    DOWN = enum.auto()


class Edge(enum.Enum):
    RISING = enum.auto()
    FALLING = enum.auto()
    BOTH = enum.auto()


#: ``(pin, level, tick_us)`` -- pigpio's callback shape, kept verbatim so the
#: real backend does not have to repack an argument tuple thousands of times a
#: second in its notification thread.
EdgeCallback = Callable[[int, int, int], None]


@runtime_checkable
class CallbackHandle(Protocol):
    def cancel(self) -> None:
        """Stop delivering. Must be idempotent -- cleanup paths run twice."""
        ...


def tick_diff(earlier: int, later: int) -> int:
    """Wrap-safe difference of two 32-bit microsecond ticks, in microseconds.

    Returns a **signed** result in roughly +/-35 minutes. Signed rather than
    pigpio's always-positive convention on purpose: with an unsigned difference,
    two callbacks delivered out of order produce 4294 seconds instead of a small
    negative number, and the M/T velocity estimator would turn that into a
    plausible-looking near-zero RPM instead of an obvious anomaly.
    """
    delta = (later - earlier) & 0xFFFFFFFF
    return delta - TICK_WRAP if delta >= _TICK_HALF else delta


class GpioBackend(ABC):
    """Abstract pin access. Implementations: PigpioBackend, MockBackend."""

    #: Class-level defaults so a subclass is not obliged to call ``super().__init__``.
    #: Writing to them creates an instance attribute, which is what we want.
    gpio_errors: int = 0
    _last_gpio_error: str | None = None

    # --- digital -----------------------------------------------------------

    @abstractmethod
    def setup_output(self, pin: int, initial: bool = False) -> None:
        """Configure ``pin`` as an output and drive it to ``initial``.

        The initial level is part of the call rather than a separate ``write``
        because on BCM283x GPIO0-8 come up with pull-ups: IN1 (GPIO5) and IN2
        (GPIO6) idle HIGH, and with both IN pins high an enabled bridge is a
        brake. The window between "become an output" and "get driven low" must
        not exist in the code even though hardware pull-downs are the real fix.
        """

    @abstractmethod
    def write(self, pin: int, value: bool) -> None: ...

    @abstractmethod
    def setup_input(self, pin: int, pull: Pull = Pull.NONE, glitch_us: int = 0) -> None:
        """Configure ``pin`` as an input.

        ``glitch_us`` applies a hardware/daemon-side glitch filter: an edge is
        reported only after the level has been stable for that long. Applied in
        the daemon rather than in Python because the encoder produces ~2000
        edges a second and each one crossing into the interpreter costs far more
        than the filtering does.
        """

    @abstractmethod
    def read(self, pin: int) -> bool: ...

    # --- PWM ---------------------------------------------------------------

    @abstractmethod
    def set_pwm_pair(
        self, pin_a: int, pin_b: int, freq_hz: int, duty_a: float, duty_b: float
    ) -> None:
        """Set BOTH hardware PWM channels at once. Duties are 0.0..1.0.

        This is a pair operation and not two calls BY DESIGN. On BCM283x the two
        hardware PWM channels share a single clock divider, so writing GPIO12 at
        1 kHz and then GPIO13 at 4 kHz corrupts channel 0. Making the pair the
        only available operation renders that mistake unrepresentable.

        Duties are unsigned; direction lives on the IN pins.
        """

    # --- servo -------------------------------------------------------------

    @abstractmethod
    def set_servo_pulse(self, pin: int, pulse_us: int) -> None:
        """DMA-timed servo pulse. ``pulse_us == 0`` stops the pulse train, which
        lets the servo go limp -- that is the correct disarmed state here,
        because the servo shares the Pi's 5 V rail with the SoC."""

    # --- edges -------------------------------------------------------------

    @abstractmethod
    def add_edge_callback(
        self, pin: int, edge: Edge, callback: EdgeCallback
    ) -> CallbackHandle:
        """Register ``callback`` for edges on ``pin``.

        The callback runs on the backend's own notification thread, not the
        control thread. Its body must do nothing but record numbers.
        """

    # --- misc --------------------------------------------------------------

    @abstractmethod
    def ticks_us(self) -> int:
        """Backend microsecond tick, same time base as the callback's tick_us.
        Wraps at 2**32 like pigpio's; use :func:`tick_diff` to subtract."""

    @abstractmethod
    def cleanup(self) -> None:
        """Return every pin to a safe state and release the backend.

        Must be idempotent and must not raise: it runs from ``atexit`` and from
        signal handlers, sometimes after something has already gone wrong.
        """

    # --- error reporting (concrete) ----------------------------------------

    def note_gpio_error(self, detail: str) -> None:
        """Record a failure that happened somewhere it could not be raised.

        The control loop polls :meth:`pop_gpio_error` once a tick and turns a
        non-None result into ``Fault.GPIO_ERROR``.
        """
        self.gpio_errors += 1
        self._last_gpio_error = detail

    def pop_gpio_error(self) -> str | None:
        """Most recent unreported error, cleared by reading. Allocation-free on
        the common path, which is why it is a poll and not a callback."""
        detail = self._last_gpio_error
        if detail is not None:
            self._last_gpio_error = None
        return detail

    # --- lifecycle sugar ---------------------------------------------------

    def __enter__(self) -> "GpioBackend":
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.cleanup()


# --------------------------------------------------------------------------
# Backend selection
# --------------------------------------------------------------------------

BACKEND_AUTO = "auto"
BACKEND_PIGPIO = "pigpio"
BACKEND_MOCK = "mock"

_ARM_MACHINES = frozenset({"aarch64", "armv7l", "armv6l", "arm64"})


def select_backend(name: str = BACKEND_AUTO, **kwargs: Any) -> GpioBackend:
    """Build a backend.

    ``'auto'`` picks pigpio on an ARM machine where the module imports, and the
    mock everywhere else. The ``TELEKART_BACKEND`` environment variable
    overrides ``'auto'`` -- that is how you force the mock on the Pi itself to
    bench-test the control loop with the motor leads still disconnected. An
    explicit argument always wins over the environment, because a caller that
    named a backend meant it.

    Backend-specific keyword arguments are passed straight through.
    """
    requested = (name or BACKEND_AUTO).strip().lower()
    if requested == BACKEND_AUTO:
        from ..constants import ENV_BACKEND

        env = os.environ.get(ENV_BACKEND, "").strip().lower()
        if env:
            requested = env

    if requested == BACKEND_AUTO:
        requested = BACKEND_PIGPIO if _looks_like_a_pi() else BACKEND_MOCK

    if requested == BACKEND_PIGPIO:
        from .pigpio_backend import PigpioBackend

        return PigpioBackend(**kwargs)
    if requested == BACKEND_MOCK:
        from .mock_backend import MockBackend

        return MockBackend(**kwargs)
    raise GpioError(
        f"unknown GPIO backend {requested!r}; expected "
        f"{BACKEND_AUTO!r}, {BACKEND_PIGPIO!r} or {BACKEND_MOCK!r}"
    )


def _looks_like_a_pi() -> bool:
    """ARM plus an importable pigpio. Importability rather than a connection
    test, because 'pigpiod is not running yet' is a startup ordering problem
    that deserves its own clear error, not a silent downgrade to the mock --
    silently mocking on the real car means the wheels never turn and nothing
    says why."""
    if platform.machine() not in _ARM_MACHINES:
        return False
    from importlib.util import find_spec

    try:
        return find_spec("pigpio") is not None
    except (ImportError, ValueError):
        return False
