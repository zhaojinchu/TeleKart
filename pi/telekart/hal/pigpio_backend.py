"""The real backend: pigpio over its local socket.

pigpio is used rather than RPi.GPIO or gpiozero for two reasons that matter
here: DMA-timed servo pulses with single-digit microsecond jitter, and edge
callbacks with a hardware glitch filter that are delivered on a daemon thread
instead of being polled from Python.

It also has one property that shapes the entire safety design: **the daemon
outlives its client**. Kill this process at 80 % duty and pigpiod happily keeps
the motors running forever. Nothing in this file can fix that -- see the
four-layer panic stop in ``telekart/app.py`` -- but :meth:`PigpioBackend.cleanup`
is layer one and must never fail to run.

The module imports cleanly on a machine with no pigpio installed; the import is
deferred to construction so that ``import telekart.hal.pigpio_backend`` on a Mac
is not an error.
"""

from __future__ import annotations

from typing import Any

from ..constants import (
    HW_PWM_CHANNEL_0_PINS,
    HW_PWM_CHANNEL_1_PINS,
    SERVO_ABS_MAX_US,
    SERVO_ABS_MIN_US,
)
from ..log import get_logger
from .base import CallbackHandle, Edge, EdgeCallback, GpioBackend, GpioError, Pull

_log = get_logger(__name__)

#: pigpio expresses duty in parts per million of the period.
_DUTY_SCALE = 1_000_000


def _import_pigpio() -> Any:
    try:
        import pigpio
    except ImportError as exc:  # pragma: no cover - depends on the machine
        raise GpioError(
            "the pigpio module is not installed. On Raspberry Pi OS: "
            "'sudo apt install python3-pigpio pigpio' then "
            "'sudo systemctl enable --now pigpiod'."
        ) from exc
    return pigpio


class _PigpioCallback:
    """Wraps pigpio's callback object so cancelling twice is harmless."""

    __slots__ = ("_callback",)

    def __init__(self, callback: Any) -> None:
        self._callback = callback

    def cancel(self) -> None:
        callback = self._callback
        if callback is None:
            return
        self._callback = None
        try:
            callback.cancel()
        except Exception as exc:  # noqa: BLE001 - cleanup must never propagate
            _log.debug("edge callback cancel failed", error=str(exc))


class PigpioBackend(GpioBackend):
    """pigpio-backed :class:`~telekart.hal.base.GpioBackend`."""

    def __init__(
        self,
        host: str = "localhost",
        port: int = 8888,
        *,
        dedup_writes: bool = True,
    ) -> None:
        pigpio = _import_pigpio()
        self._pigpio = pigpio
        self._pi = pigpio.pi(host, port)
        if not self._pi.connected:
            raise GpioError(
                f"cannot reach pigpiod at {host}:{port}. Start it with "
                "'sudo systemctl enable --now pigpiod'. It must be running "
                "before the firmware, and it must stay running."
            )

        self._edge_map = {
            Edge.RISING: pigpio.RISING_EDGE,
            Edge.FALLING: pigpio.FALLING_EDGE,
            Edge.BOTH: pigpio.EITHER_EDGE,
        }
        self._pull_map = {
            Pull.NONE: pigpio.PUD_OFF,
            Pull.UP: pigpio.PUD_UP,
            Pull.DOWN: pigpio.PUD_DOWN,
        }

        self._dedup = dedup_writes
        self._outputs: set[int] = set()
        self._callbacks: list[_PigpioCallback] = []
        self._closed = False

        # PWM pair state. `_pwm_pins` is None until the first call validates the
        # pin assignment; after that a different pair is a programming error.
        self._pwm_pins: tuple[int, int] | None = None
        self._pwm_freq = 0
        self._pwm_duty: tuple[int, int] = (-1, -1)
        self._servo_us: dict[int, int] = {}

        _log.info(
            "pigpio connected",
            host=host,
            port=port,
            version=self._safe_version(),
        )

    # -- digital ------------------------------------------------------------

    def setup_output(self, pin: int, initial: bool = False) -> None:
        pi = self._pi
        # Level first, then mode: driving the pin before enabling the output
        # means it never passes through the SoC's boot-time pull state. On
        # GPIO5/6 (IN1/IN2) that state is HIGH, and IN1==IN2 with the bridge
        # enabled is a brake.
        pi.write(pin, 1 if initial else 0)
        pi.set_mode(pin, self._pigpio.OUTPUT)
        pi.write(pin, 1 if initial else 0)
        self._outputs.add(pin)

    def write(self, pin: int, value: bool) -> None:
        try:
            self._pi.write(pin, 1 if value else 0)
        except Exception as exc:  # noqa: BLE001 - never raise into the loop
            self.note_gpio_error(f"write(pin={pin}) failed: {exc}")

    def setup_input(self, pin: int, pull: Pull = Pull.NONE, glitch_us: int = 0) -> None:
        pi = self._pi
        pi.set_mode(pin, self._pigpio.INPUT)
        pi.set_pull_up_down(pin, self._pull_map[pull])
        if glitch_us > 0:
            # Clamped to pigpio's documented maximum rather than trusted: a
            # config typo here would otherwise take the daemon down.
            pi.set_glitch_filter(pin, min(int(glitch_us), 300_000))

    def read(self, pin: int) -> bool:
        try:
            return bool(self._pi.read(pin))
        except Exception as exc:  # noqa: BLE001
            self.note_gpio_error(f"read(pin={pin}) failed: {exc}")
            # A failed read of the E-stop button must not read as "pressed" or
            # as "released" by accident; False matches the pulled-up idle level
            # of every input this firmware uses.
            return False

    # -- PWM ----------------------------------------------------------------

    def set_pwm_pair(
        self, pin_a: int, pin_b: int, freq_hz: int, duty_a: float, duty_b: float
    ) -> None:
        if self._pwm_pins is None:
            self._configure_pwm_pair(pin_a, pin_b, freq_hz)
        elif self._pwm_pins != (pin_a, pin_b):
            raise GpioError(
                f"PWM pair changed from {self._pwm_pins} to {(pin_a, pin_b)}; "
                "the two hardware channels are fixed at construction"
            )

        freq = int(freq_hz)
        raw_a = _duty_to_ppm(duty_a)
        raw_b = _duty_to_ppm(duty_b)
        if self._dedup and freq == self._pwm_freq and (raw_a, raw_b) == self._pwm_duty:
            return

        # Both channels every time, in the same order, even when only one moved.
        # They share a clock divider; writing them as a unit is the invariant
        # this method exists to enforce.
        ok_a = self._hardware_pwm(pin_a, freq, raw_a)
        ok_b = self._hardware_pwm(pin_b, freq, raw_b)
        if ok_a and ok_b:
            self._pwm_freq = freq
            self._pwm_duty = (raw_a, raw_b)
        else:
            # Force a full rewrite next tick: the cached state is now a lie.
            self._pwm_duty = (-1, -1)

    def _configure_pwm_pair(self, pin_a: int, pin_b: int, freq_hz: int) -> None:
        """First-call validation. This is the one place PWM failure raises.

        It runs during construction of the motor driver, which is configuration
        time, so a hard failure here is correct and a fault flag would not be:
        a car whose PWM never worked must refuse to arm, not drive at whatever
        duty the pins happen to sit at.
        """
        if pin_a not in HW_PWM_CHANNEL_0_PINS:
            raise GpioError(
                f"GPIO{pin_a} is not on hardware PWM channel 0 "
                f"(expected one of {sorted(HW_PWM_CHANNEL_0_PINS)})"
            )
        if pin_b not in HW_PWM_CHANNEL_1_PINS:
            raise GpioError(
                f"GPIO{pin_b} is not on hardware PWM channel 1 "
                f"(expected one of {sorted(HW_PWM_CHANNEL_1_PINS)})"
            )
        if not 100 <= freq_hz <= 25_000:
            raise GpioError(f"PWM frequency {freq_hz} Hz is outside the usable range")

        # Probe at zero duty: safe on a wired-up car, and it proves the daemon
        # will actually accept a hardware PWM write before anything spins.
        for pin in (pin_a, pin_b):
            result = self._raw_hardware_pwm(pin, int(freq_hz), 0)
            if result != 0:
                raise GpioError(
                    f"hardware PWM on GPIO{pin} was refused by pigpiod ({result}). "
                    "The usual cause is the onboard audio driver: snd_bcm2835 "
                    "claims BOTH PWM channels at boot. Add 'dtparam=audio=off' to "
                    "/boot/firmware/config.txt and reboot. Also check that pigpiod "
                    "is running as root, which hardware PWM requires."
                )
        self._pwm_pins = (pin_a, pin_b)
        self._pwm_freq = int(freq_hz)
        self._pwm_duty = (0, 0)
        _log.info("hardware PWM pair configured", pin_a=pin_a, pin_b=pin_b, freq_hz=freq_hz)

    def _raw_hardware_pwm(self, pin: int, freq_hz: int, duty_ppm: int) -> int:
        """Returns 0 on success or a negative pigpio error code.

        pigpio's Python client normally converts error codes into exceptions,
        but ``pigpio.exceptions`` is a module-level global that any other import
        could have flipped. Handle both shapes rather than trusting it.
        """
        try:
            result = self._pi.hardware_PWM(pin, freq_hz, duty_ppm)
        except Exception as exc:  # noqa: BLE001 - pigpio.error is not importable here
            _log.debug("hardware_PWM raised", pin=pin, error=str(exc))
            return -1
        return int(result) if result is not None else 0

    def _hardware_pwm(self, pin: int, freq_hz: int, duty_ppm: int) -> bool:
        result = self._raw_hardware_pwm(pin, freq_hz, duty_ppm)
        if result != 0:
            self.note_gpio_error(
                f"hardware PWM write failed on GPIO{pin} (code {result})"
            )
            return False
        return True

    # -- servo --------------------------------------------------------------

    def set_servo_pulse(self, pin: int, pulse_us: int) -> None:
        pulse = int(pulse_us)
        if pulse != 0:
            # Clamped, not rejected: this runs on a control tick, and a servo
            # driven into its end stop stalls against the Pi's own 5 V rail.
            if pulse < SERVO_ABS_MIN_US:
                pulse = SERVO_ABS_MIN_US
            elif pulse > SERVO_ABS_MAX_US:
                pulse = SERVO_ABS_MAX_US
        if self._dedup and self._servo_us.get(pin) == pulse:
            return
        try:
            self._pi.set_servo_pulsewidth(pin, pulse)
            self._servo_us[pin] = pulse
        except Exception as exc:  # noqa: BLE001
            self._servo_us.pop(pin, None)
            self.note_gpio_error(f"servo pulse write failed on GPIO{pin}: {exc}")

    # -- edges --------------------------------------------------------------

    def add_edge_callback(
        self, pin: int, edge: Edge, callback: EdgeCallback
    ) -> CallbackHandle:
        try:
            raw = self._pi.callback(pin, self._edge_map[edge], callback)
        except Exception as exc:  # noqa: BLE001
            # Registration happens at construction time, so raising is right.
            raise GpioError(f"could not register an edge callback on GPIO{pin}: {exc}") from exc
        handle = _PigpioCallback(raw)
        self._callbacks.append(handle)
        return handle

    # -- misc ---------------------------------------------------------------

    def ticks_us(self) -> int:
        try:
            return int(self._pi.get_current_tick())
        except Exception as exc:  # noqa: BLE001
            self.note_gpio_error(f"get_current_tick failed: {exc}")
            return 0

    def cleanup(self) -> None:
        """Layer one of the panic-stop chain. Idempotent, and swallows everything.

        Order matters: PWM to zero first so the bridge is disabled, then the
        direction pins low, then the servo goes limp, and only then do we drop
        the socket. Releasing the socket first would leave pigpiod holding the
        last duty cycle indefinitely.
        """
        if self._closed:
            return
        self._closed = True

        if self._pwm_pins is not None:
            for pin in self._pwm_pins:
                try:
                    self._pi.hardware_PWM(pin, self._pwm_freq or 1000, 0)
                except Exception:  # noqa: BLE001
                    pass
        for pin in sorted(self._outputs):
            try:
                self._pi.write(pin, 0)
            except Exception:  # noqa: BLE001
                pass
        for pin in list(self._servo_us):
            try:
                self._pi.set_servo_pulsewidth(pin, 0)
            except Exception:  # noqa: BLE001
                pass
        for handle in self._callbacks:
            handle.cancel()
        self._callbacks.clear()
        try:
            self._pi.stop()
        except Exception:  # noqa: BLE001
            pass
        _log.info("pigpio backend cleaned up", gpio_errors=self.gpio_errors)

    # -- diagnostics --------------------------------------------------------

    def _safe_version(self) -> str:
        try:
            return f"pigpiod {self._pi.get_pigpio_version()} hw {self._pi.get_hardware_revision():#x}"
        except Exception:  # noqa: BLE001
            return "unknown"

    @property
    def connected(self) -> bool:
        return bool(getattr(self._pi, "connected", False)) and not self._closed

    @property
    def pi(self) -> Any:
        """The raw pigpio handle. For calibration tools and the panic-stop shim
        only -- normal code goes through the backend interface."""
        return self._pi

    def __repr__(self) -> str:
        return f"PigpioBackend(connected={self.connected}, errors={self.gpio_errors})"


def _duty_to_ppm(duty: float) -> int:
    """0.0..1.0 to pigpio's parts-per-million, clamped.

    Clamped rather than validated because this is on the control path. A duty
    outside the range means a bug upstream; the right response is to run at the
    limit and let the limiter flag say so, not to raise while the car moves.
    """
    if duty <= 0.0:
        return 0
    if duty >= 1.0:
        return _DUTY_SCALE
    return int(duty * _DUTY_SCALE)
