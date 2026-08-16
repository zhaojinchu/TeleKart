"""The control process: three threads, four panic-stop layers, one lifecycle.

**The threads.** asyncio on the main thread owns every socket. One
``ControlThread`` -- a plain ``threading.Thread`` raised to ``SCHED_FIFO`` and
paced against absolute deadlines -- runs the whole 100 Hz loop. pigpio's own
notification thread delivers encoder edges into callbacks that do nothing but
add to a counter. Nothing else exists, and nothing crosses between them except
through the handoffs described at :class:`Mailbox` and :class:`_Request`.

**The panic-stop chain.** ``pigpiod`` retains GPIO state after its client dies.
A segfault or ``kill -9`` at 80 % duty leaves the motors running *indefinitely*.
No single mechanism covers every way a process can end, so there are four:

1. In-process: :class:`HardwarePanic`, wired to ``atexit``, ``SIGTERM`` /
   ``SIGINT`` / ``SIGHUP``, ``sys.excepthook``, ``threading.excepthook``, and a
   context manager around :func:`main`. Covers clean exits, unhandled
   exceptions in either thread, and operator Ctrl-C.
2. ``ExecStopPost=`` in the systemd unit, running ``pigs`` directly. **The only
   layer that covers SIGKILL**, and systemd runs it on every stop path.
3. External 10 kOhm pull-downs on ENA/ENB/IN1-IN4. Covers reboot, power loss,
   and a ribbon cable that worked loose.
4. The battery-side switch: the human's E-stop, and the only one that still
   works when everything else is on fire.

**The watchdog is pinged from the control thread, never from asyncio.** That is
the entire point of having one. A watchdog fed by the event loop stays happy
while the control loop is wedged with the motors at 60 % duty, which is the
exact failure it exists to catch.
"""

from __future__ import annotations

import argparse
import asyncio
import atexit
import math
import os
import queue
import signal
import socket
import sys
import threading
import types
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Generic, Iterable, TypeVar

from telekart_protocol import ControlPacket, Fault, VehicleState
from telekart_protocol.constants import (
    TCP_SESSION_PORT,
    TCP_VIDEO_PORT,
    TELEMETRY_RATE_HZ,
    UDP_CONTROL_PORT,
)

from .calibration import (
    AutoCalibrator,
    CalibrationAborted,
    CalibrationError,
    DriveCalibration,
)
from .config import ConfigError, HardwarePins, VehicleConfig
from .constants import (
    CALIBRATION_PATH,
    CONTROL_CPU_AFFINITY,
    CONTROL_PERIOD_S,
    DEFAULT_CONFIG_PATH,
    ENCODER_GLITCH_US,
    ENV_BACKEND,
    ENV_CAR_ID,
    ENV_LOG_LEVEL,
    FIRMWARE_VERSION,
    GC_COLLECT_PERIOD_S,
    HEALTH_POLL_PERIOD_S,
    LOCAL_CONFIG_PATH,
    LOOP_OVERRUN_FAULT_COUNT,
    LOOP_P99_BUDGET_S,
    OOM_SCORE_CONTROL,
    RT_PRIORITY_CONTROL,
    STATUS_LED_PERIOD_S,
)
from .control.drive import DriveCommand, DriveController, DriveState
from .control.safety import SafetyStateMachine
from .drivers.encoder import QuadratureEncoder
from .drivers.motor import MotorPair
from .drivers.servo import SteeringServo
from .hal.base import BACKEND_AUTO, Edge, GpioBackend, Pull, select_backend
from .log import RateLimiter, get_logger, setup_logging
from .net.control_link import ControlLink
from .net.discovery import Discovery
from .net.session_server import (
    CalibrationBusy,
    CalibrationProgress,
    CalibrationUnavailable,
    NotAllowedInState,
    ProgressSink,
    SessionInfo,
    SessionServer,
)
from .net.telemetry_tx import TelemetrySender, VehicleSample
from .odometry import BicycleOdometry
from .util import rt
from .util.clock import Clock, DeadlineScheduler, RealClock

_log = get_logger(__name__)

T = TypeVar("T")

#: How long a session-channel request may wait for the control thread. Longer
#: than any legitimate tick and far shorter than the systemd watchdog, so a
#: wedged loop produces a clear error on the operator's screen before the
#: process gets restarted underneath them.
REQUEST_TIMEOUT_S = 2.0

#: Control-plane requests served per tick. Bounded so a burst of session traffic
#: cannot eat the loop's budget; anything beyond this waits one period.
MAX_REQUESTS_PER_TICK = 4

#: Debounce for the local E-stop button. Generous because it is a mechanical
#: contact and because a missed press is far worse than a late one.
ESTOP_GLITCH_US = 5000

#: Cadence of the asyncio-side housekeeping task.
SUPERVISOR_PERIOD_S = 1.0 / TELEMETRY_RATE_HZ

_EMPTY_SAMPLE = VehicleSample()


# --------------------------------------------------------------------------
# Cross-thread handoffs
# --------------------------------------------------------------------------


class Mailbox(Generic[T]):
    """Depth-one handoff between exactly one writer and exactly one reader.

    **Why there is no lock here, and why adding one would be a regression.**

    ``put`` performs two operations: a ``STORE_ATTR`` of an object reference and
    an increment of an integer attribute. Under CPython's GIL each of those is
    indivisible with respect to other threads -- a reader can observe the old
    pair or the new pair, never a half-written reference. The generation is
    stored *after* the value, so a reader that sees a new generation is
    guaranteed to see at least the value that went with it.

    ``self._generation += 1`` is a read-modify-write and is **not** atomic in
    general. It is safe here only because there is exactly one writer. The same
    argument covers ``_seen``, which only the reader ever touches.

    The worst case is therefore benign: if the writer overwrites between the
    reader's generation read and its value read, the reader gets a *newer* value
    stamped with an older generation and re-reads it next time. For a
    latest-wins command stream that is not merely acceptable, it is exactly what
    you want.

    A ``queue.Queue`` here would add a lock acquisition and a condition-variable
    wait to a 100 Hz path, and would buffer stale commands that must never be
    executed. Do not "fix" this into one.
    """

    __slots__ = ("_value", "_generation", "_seen")

    def __init__(self, initial: T | None = None) -> None:
        self._value: T | None = initial
        self._generation = 0
        self._seen = 0

    def put(self, value: T) -> None:
        """Writer side. Overwrites unconditionally: newest wins."""
        self._value = value
        self._generation += 1

    def take(self) -> T | None:
        """Reader side. None when nothing new has arrived since the last take."""
        generation = self._generation
        if generation == self._seen:
            return None
        self._seen = generation
        return self._value

    def peek(self) -> T | None:
        """Latest value, regardless of whether it has been taken."""
        return self._value

    @property
    def generation(self) -> int:
        return self._generation


@dataclass(slots=True)
class _Request:
    """One control-plane call from the session channel to the control thread.

    A queue rather than a mailbox, deliberately. The streams are latest-wins and
    lossy by design; a control-plane request must be executed exactly once and
    answered exactly once, and dropping "disarm" because "arm" arrived a
    millisecond later would be indefensible. The rate is a handful per minute,
    so the lock inside the queue costs nothing measurable.
    """

    kind: str
    payload: Any = None
    future: asyncio.Future[Any] | None = None
    loop: asyncio.AbstractEventLoop | None = None

    def resolve(self, value: Any) -> None:
        """Hand the answer back to the event loop. Runs on the control thread,
        so ``call_soon_threadsafe`` is the only legal way to touch the future."""
        loop, future = self.loop, self.future
        if loop is None or future is None or loop.is_closed():
            return
        try:
            loop.call_soon_threadsafe(_settle_future, future, value)
        except RuntimeError:
            pass  # the loop shut down between the check and the call


def _settle_future(future: asyncio.Future[Any], value: Any) -> None:
    if future.done():
        return
    if isinstance(value, BaseException):
        future.set_exception(value)
    else:
        future.set_result(value)


class ControlLoopStalled(RuntimeError):
    """The control thread did not answer a request in time."""


# --------------------------------------------------------------------------
# Layer 1 of the panic chain
# --------------------------------------------------------------------------

#: Preformatted. A panic stop may run from a signal handler, and building a
#: string there can deadlock against a logging handler the interrupted thread
#: was already inside. os.write to fd 2 is the only output this path performs.
_PANIC_NOTE = b"telekart: panic stop\n"


class HardwarePanic:
    """Drives every motor output to a known-safe state, from anywhere.

    Idempotent, allocation-free, and it never raises. Two levels of belt and
    braces: the :class:`MotorPair` if construction got that far, then raw pin
    writes regardless -- because a panic stop has to work when the very object
    that was supposed to manage the pins is the thing that broke.

    The order is not arbitrary. Enables go to zero **first**, which coasts the
    bridge. Clearing the direction pins while an enable is still high walks the
    outputs through IN1 == IN2, and on an L298 that is a brake -- a hard short
    across a spinning motor at the exact moment you were trying to calm things
    down.
    """

    __slots__ = (
        "_gpio", "_pins", "_pwm_hz", "_motors", "_servo", "fired", "failures",
        "_bridge_pins",
    )

    def __init__(self, gpio: GpioBackend, pins: HardwarePins, pwm_hz: int) -> None:
        self._gpio = gpio
        self._pins = pins
        self._pwm_hz = int(pwm_hz)
        self._motors: MotorPair | None = None
        self._servo: SteeringServo | None = None
        self.fired = 0
        self.failures = 0
        #: Built once, here, and only iterated in :meth:`stop`. Spelling the
        #: same six pins as a tuple literal inside ``stop`` would build that
        #: tuple on the heap every time it ran -- the pins are attribute loads,
        #: not constants, so the peephole optimiser cannot fold them away. An
        #: allocation is the one thing this path must not do: it runs from a
        #: signal handler and from ``sys.excepthook``, which is exactly where a
        #: MemoryError would land.
        m = pins.motors
        self._bridge_pins: tuple[int, ...] = (
            m.ena, m.enb, m.in1, m.in2, m.in3, m.in4,
        )

    def attach(
        self, motors: MotorPair | None = None, servo: SteeringServo | None = None
    ) -> None:
        """Wire in the drivers once they exist. Before this, the raw path alone
        is what stands between a construction failure and a live bridge."""
        if motors is not None:
            self._motors = motors
        if servo is not None:
            self._servo = servo

    def stop(self, announce: bool = True) -> None:
        self.fired += 1
        if announce:
            try:
                os.write(2, _PANIC_NOTE)
            except OSError:
                pass

        motors = self._motors
        if motors is not None:
            try:
                motors.panic_stop()
            except BaseException:  # noqa: BLE001 - a panic stop may not fail
                self.failures += 1

        gpio = self._gpio
        pins = self._pins.motors
        try:
            gpio.set_pwm_pair(pins.ena, pins.enb, self._pwm_hz, 0.0, 0.0)
        except BaseException:  # noqa: BLE001
            self.failures += 1
        # Then drive the enables low outright, so the pins leave PWM mode and
        # sit at a level that survives this process exiting.
        for pin in self._bridge_pins:
            try:
                gpio.write(pin, False)
            except BaseException:  # noqa: BLE001
                self.failures += 1
        try:
            # Pulse width zero stops the train and lets the servo go limp, which
            # is the correct disarmed state: it shares the Pi's own 5 V rail.
            gpio.set_servo_pulse(self._pins.servo, 0)
        except BaseException:  # noqa: BLE001
            self.failures += 1

        led = self._pins.status_led
        if led is not None:
            try:
                gpio.write(led, False)
            except BaseException:  # noqa: BLE001
                pass


class PanicChain:
    """Installs, and later removes, every in-process panic-stop hook.

    Used as a context manager around :func:`main` so that even a ``return`` out
    of an unexpected branch safes the hardware on the way past.
    """

    __slots__ = (
        "_panic", "_on_signal", "_previous", "_sys_hook", "_thread_hook",
        "_installed", "_signals_seen",
    )

    def __init__(
        self, panic: HardwarePanic, on_signal: Callable[[int], None] | None = None
    ) -> None:
        self._panic = panic
        self._on_signal = on_signal
        self._previous: dict[int, Any] = {}
        self._sys_hook: Any = None
        self._thread_hook: Any = None
        self._installed = False
        self._signals_seen = 0

    def __enter__(self) -> "PanicChain":
        self.install()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: types.TracebackType | None,
    ) -> bool:
        # Fires on every path out of main(), including a plain return. The
        # hardware is safe before this frame is gone.
        self._panic.stop(announce=exc is not None)
        self.remove()
        return False

    def install(self) -> None:
        if self._installed:
            return
        self._installed = True

        atexit.register(self._panic.stop, False)

        # signal.signal rather than loop.add_signal_handler: the latter only
        # runs when the event loop gets scheduled, and a process worth killing
        # is very often a process whose event loop is what stopped running.
        for name in ("SIGTERM", "SIGINT", "SIGHUP"):
            signum = getattr(signal, name, None)
            if signum is None:  # pragma: no cover - platform dependent
                continue
            try:
                self._previous[int(signum)] = signal.signal(signum, self._handle_signal)
            except (OSError, ValueError, RuntimeError):
                # Not the main thread, or the platform disallows it. The other
                # three layers still apply.
                pass

        self._sys_hook = sys.excepthook
        sys.excepthook = self._handle_exception
        self._thread_hook = threading.excepthook
        threading.excepthook = self._handle_thread_exception

    def remove(self) -> None:
        if not self._installed:
            return
        self._installed = False
        atexit.unregister(self._panic.stop)
        for signum, previous in self._previous.items():
            try:
                signal.signal(signum, previous)
            except (OSError, ValueError, RuntimeError):
                pass
        self._previous.clear()
        if self._sys_hook is not None:
            sys.excepthook = self._sys_hook
        if self._thread_hook is not None:
            threading.excepthook = self._thread_hook

    # -- handlers -----------------------------------------------------------

    def _handle_signal(self, signum: int, _frame: types.FrameType | None) -> None:
        self._signals_seen += 1
        self._panic.stop()
        if self._signals_seen > 1:
            # Second Ctrl-C, or systemd escalating. The outputs are already
            # safe; leave immediately rather than waiting on a shutdown that is
            # evidently not happening.
            os._exit(1)
        callback = self._on_signal
        if callback is not None:
            try:
                callback(signum)
            except BaseException:  # noqa: BLE001 - nothing may escape a handler
                os._exit(1)
            return

        # No callback: restore what installing this handler took away.
        #
        # Replacing the default SIGINT handler also removes the KeyboardInterrupt
        # it would have raised, so without this a caller's `except
        # KeyboardInterrupt` is dead code and its `finally:` never runs. The
        # bring-up scripts in pi/scripts/ are all shaped that way -- Ctrl-C would
        # safe the outputs via panic.stop() above and then the script would carry
        # on to the next iteration and command duty again, which is precisely the
        # opposite of what the operator just asked for. Raising here unwinds the
        # main thread through those handlers instead.
        #
        # Only the panic stop has to be signal-safe, and it has already run.
        if signum == getattr(signal, "SIGINT", None):
            raise KeyboardInterrupt
        raise SystemExit(128 + int(signum))

    def _handle_exception(
        self,
        exc_type: type[BaseException],
        exc: BaseException,
        tb: types.TracebackType | None,
    ) -> None:
        self._panic.stop()
        hook = self._sys_hook
        if hook is not None:
            hook(exc_type, exc, tb)

    def _handle_thread_exception(self, args: threading.ExceptHookArgs) -> None:
        # An exception escaping any thread means an invariant this process
        # depends on is gone. It is never safe to keep driving afterwards.
        self._panic.stop()
        hook = self._thread_hook
        if hook is not None:
            hook(args)


# --------------------------------------------------------------------------
# systemd integration
# --------------------------------------------------------------------------

_NOTIFY_READY = b"READY=1"
_NOTIFY_WATCHDOG = b"WATCHDOG=1"
_NOTIFY_STOPPING = b"STOPPING=1"


class SystemdNotifier:
    """``sd_notify`` without the ``systemd-python`` dependency.

    The protocol is one AF_UNIX datagram of newline-separated ``KEY=value``
    pairs, so a hand-rolled client is a dozen lines and removes a C extension
    from a 512 MB board's dependency list.

    :meth:`ping` is the only method on a hot path and it allocates nothing: the
    payload is a module-level constant and ``sendto`` is one syscall.
    """

    __slots__ = ("_sock", "_addr", "watchdog_interval", "sent", "errors")

    def __init__(self) -> None:
        self._sock: socket.socket | None = None
        self._addr = ""
        self.watchdog_interval = 0.0
        self.sent = 0
        self.errors = 0

        address = os.environ.get("NOTIFY_SOCKET", "")
        if not address:
            return
        # A leading '@' means the abstract namespace, which Python spells with a
        # leading NUL.
        if address.startswith("@"):
            address = "\0" + address[1:]
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        except (AttributeError, OSError) as exc:  # pragma: no cover - Linux only
            _log.debug("systemd notification socket unavailable", error=str(exc))
            return
        sock.setblocking(False)
        self._sock = sock
        self._addr = address

        watchdog_pid = os.environ.get("WATCHDOG_PID", "").strip()
        if watchdog_pid and watchdog_pid != str(os.getpid()):
            # The variables were inherited by a child process; they are not ours
            # to act on. Pinging anyway would keep a unit alive on behalf of a
            # process systemd is not actually watching.
            _log.debug("ignoring inherited watchdog environment", pid=watchdog_pid)
            return
        try:
            usec = int(os.environ.get("WATCHDOG_USEC", "0"))
        except ValueError:
            usec = 0
        if usec > 0:
            # A third of the interval, not a half: the loop that sends these is
            # the loop being watched, and the margin has to survive one bad tick
            # without the unit being restarted underneath a moving car.
            self.watchdog_interval = usec / 3_000_000.0
            _log.info(
                "systemd watchdog active",
                interval_s=round(self.watchdog_interval, 3),
                timeout_s=round(usec / 1_000_000.0, 3),
            )

    @property
    def available(self) -> bool:
        return self._sock is not None

    def _send(self, payload: bytes) -> bool:
        sock = self._sock
        if sock is None:
            return False
        try:
            sock.sendto(payload, self._addr)
        except OSError:
            self.errors += 1
            return False
        self.sent += 1
        return True

    def ready(self) -> bool:
        return self._send(_NOTIFY_READY)

    def ping(self) -> bool:
        """Feed the watchdog. Called from the control thread only."""
        return self._send(_NOTIFY_WATCHDOG)

    def stopping(self) -> bool:
        return self._send(_NOTIFY_STOPPING)

    def status(self, text: str) -> bool:
        return self._send(b"STATUS=" + text.encode("utf-8", "replace"))

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()


# --------------------------------------------------------------------------
# Health
# --------------------------------------------------------------------------

_THERMAL_PATH = Path("/sys/class/thermal/thermal_zone0/temp")

#: Bits of vcgencmd's get_throttled word. The low nibble is "right now"; bits
#: 16-19 are "has happened since boot" and stay set until reboot.
_THROTTLED_UNDERVOLT_NOW = 1 << 0
_THROTTLED_UNDERVOLT_EVER = 1 << 16
_THROTTLED_ANY_NOW = (1 << 1) | (1 << 2) | (1 << 3)
_THROTTLED_ANY_EVER = (1 << 17) | (1 << 18) | (1 << 19)


class HealthMonitor:
    """SoC temperature and ``vcgencmd get_throttled``, polled at 1 Hz.

    Never from the control thread: ``vcgencmd`` is a subprocess, and forking on
    a Pi Zero 2 W costs milliseconds against a 10 ms budget. The results are
    read back by the control thread as plain attribute loads, which are atomic
    for ints and floats and need no synchronisation.

    Note what is deliberately *not* here. Undervoltage raises
    ``PI_UNDERVOLTAGE``, which is a warning; it never raises anything in
    ``CRITICAL_FAULTS``. Killing the drive because the Pi's 5 V rail dipped once
    mid-corner would be its own hazard, and SoC temperature is not the L298N's
    temperature -- ``OVERTEMP`` belongs to the bridge, not to the CPU.
    """

    __slots__ = ("_faults", "_vcgencmd_ok", "throttled", "cpu_temp_c",
                 "polls", "failures")

    def __init__(self, faults: Mailbox[int]) -> None:
        self._faults = faults
        self._vcgencmd_ok = True
        self.throttled = 0
        self.cpu_temp_c = 0.0
        self.polls = 0
        self.failures = 0

    async def run(self, period: float = HEALTH_POLL_PERIOD_S) -> None:
        while True:
            await self.poll()
            await asyncio.sleep(period)

    async def poll(self) -> None:
        self.polls += 1
        self.cpu_temp_c = _read_cpu_temp()
        if not self._vcgencmd_ok:
            return
        word = await self._read_throttled()
        if word is None:
            return
        self.throttled = word
        self._publish_faults(word)

    def _publish_faults(self, word: int) -> None:
        faults = 0
        if word & (_THROTTLED_UNDERVOLT_NOW | _THROTTLED_UNDERVOLT_EVER):
            faults |= int(Fault.PI_UNDERVOLTAGE)
        if word & (_THROTTLED_ANY_NOW | _THROTTLED_ANY_EVER):
            faults |= int(Fault.PI_THROTTLED)
        if faults:
            self._faults.put(faults)

    async def _read_throttled(self) -> int | None:
        try:
            process = await asyncio.create_subprocess_exec(
                "vcgencmd",
                "get_throttled",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.DEVNULL,
            )
        except (OSError, ValueError) as exc:
            # Not a Pi, or vcgencmd is not on PATH. Say so once and stop asking:
            # a failing subprocess spawn every second is not free.
            self._vcgencmd_ok = False
            _log.info("vcgencmd unavailable; throttle reporting disabled", error=str(exc))
            return None
        try:
            stdout, _ = await asyncio.wait_for(process.communicate(), 2.0)
        except asyncio.TimeoutError:
            self.failures += 1
            process.kill()
            # Reaped explicitly. A killed child that is never waited on is a
            # zombie, and this poll runs once a second forever.
            try:
                await process.wait()
            except (ProcessLookupError, OSError):
                pass
            return None
        _, _, value = stdout.decode("ascii", "replace").strip().partition("=")
        try:
            return int(value, 0)
        except ValueError:
            self.failures += 1
            return None


def _read_cpu_temp() -> float:
    """Degrees Celsius from sysfs. A file read, not a subprocess."""
    try:
        raw = _THERMAL_PATH.read_text(encoding="ascii")
    except OSError:
        return 0.0
    try:
        return int(raw.strip()) / 1000.0
    except ValueError:
        return 0.0


# --------------------------------------------------------------------------
# Local controls
# --------------------------------------------------------------------------

#: Twenty slots of 50 ms each: one second of pattern per state, bit 0 first.
#: Indexed by VehicleState, so the LED alone tells you what the car thinks it is
#: doing when there is no laptop in sight.
_LED_PATTERNS: tuple[int, ...] = (
    0b10101010101010101010,  # BOOT     - fast blink
    0b00000000000000000011,  # SAFE     - one short pulse a second
    0b11111111111111111111,  # ARMED    - solid
    0b00110011001100110011,  # FAILSAFE - urgent double
    0b11001100110011001100,  # ESTOP    - inverted double
    0b11101110111011101110,  # FAULT    - mostly on, stuttering
)
_LED_SLOTS = 20


class LocalControls:
    """The local E-stop button and the status LED.

    The button is read through an edge callback rather than polled, because a
    poll is a socket round trip to pigpiod inside the 10 ms budget while a
    callback costs the control thread one attribute load. The glitch filter
    lives in the daemon, so contact bounce never reaches Python at all.
    """

    __slots__ = ("_gpio", "_button", "_led", "_handle", "_led_state",
                 "pressed", "presses")

    def __init__(self, gpio: GpioBackend, pins: HardwarePins) -> None:
        self._gpio = gpio
        self._button = pins.estop_button
        self._led = pins.status_led
        self._handle: Any = None
        self._led_state: bool | None = None
        #: Written by the notification thread, read by the control thread. A
        #: bool store is one bytecode; see Mailbox for the full argument.
        self.pressed = False
        self.presses = 0

        if self._led is not None:
            gpio.setup_output(self._led, False)
        if self._button is not None:
            # Wired to ground, so the pull-up makes idle read HIGH -- and a
            # broken wire reads as "not pressed" rather than as an E-stop that
            # cannot be cleared.
            gpio.setup_input(self._button, Pull.UP, ESTOP_GLITCH_US)
            self.pressed = not gpio.read(self._button)
            self._handle = gpio.add_edge_callback(self._button, Edge.BOTH, self._on_edge)

    def _on_edge(self, _pin: int, level: int, _tick_us: int) -> None:
        # Runs on the backend's notification thread. Two stores and nothing
        # else: no logging, no allocation, no lock.
        if level == 0:
            self.pressed = True
            self.presses += 1
        elif level == 1:
            self.pressed = False

    def update_led(self, state: VehicleState, slot: int) -> None:
        """Advance the blink pattern. Writes only on a change, because every
        write is a socket round trip to pigpiod."""
        led = self._led
        if led is None:
            return
        index = int(state)
        pattern = _LED_PATTERNS[index] if 0 <= index < len(_LED_PATTERNS) else 0
        wanted = bool(pattern >> (slot % _LED_SLOTS) & 1)
        if wanted is self._led_state:
            return
        self._led_state = wanted
        self._gpio.write(led, wanted)

    def close(self) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            handle.cancel()
        if self._led is not None:
            try:
                self._gpio.write(self._led, False)
            except Exception:  # noqa: BLE001 - shutdown path
                pass


# --------------------------------------------------------------------------
# Calibration seam
# --------------------------------------------------------------------------


#: Routines the session channel will start. One today; the servo and encoder
#: procedures in docs/calibration.md are done by hand with the tools in tools/.
CALIBRATION_ROUTINES = ("drive",)


class CalibrationJob:
    """Runs an :class:`AutoCalibrator` sweep on a thread of its own.

    The sweep is self-paced -- it sleeps, samples both encoders and re-issues
    its own commands at a steady 100 Hz for tens of seconds -- so it cannot be
    driven a tick at a time from the control loop, and it must not run *on* the
    control loop either: a forty-second blocking call there would miss four
    thousand deadlines and the systemd watchdog would restart the car mid-sweep.

    So it gets its own thread, and the control thread **parks** for the
    duration: no ``drive.tick``, no motor writes, nothing that could fight the
    sweep for the same H-bridge. It keeps doing everything else -- feeding the
    watchdog, publishing telemetry, and draining the command mailbox so the
    safety machine's link timer stays honest while the operator sits watching
    the progress bar.
    """

    __slots__ = (
        "_config", "_clock", "_motors", "_servo", "_encoder_l", "_encoder_r",
        "_path", "_pump", "_thread", "_calibrator", "_sink", "_result",
        "_active", "runs", "failures",
    )

    def __init__(
        self,
        *,
        config: VehicleConfig,
        clock: Clock,
        motors: MotorPair,
        servo: SteeringServo,
        encoder_l: QuadratureEncoder,
        encoder_r: QuadratureEncoder,
        path: Path,
        pump: Callable[[float], None] | None = None,
    ) -> None:
        self._config = config
        self._clock = clock
        self._motors = motors
        self._servo = servo
        self._encoder_l = encoder_l
        self._encoder_r = encoder_r
        self._path = path
        self._pump = pump
        self._thread: threading.Thread | None = None
        self._calibrator: AutoCalibrator | None = None
        self._sink: ProgressSink | None = None
        self._result: DriveCalibration | None = None
        self._active = False
        self.runs = 0
        self.failures = 0

    @property
    def active(self) -> bool:
        """True while the sweep owns the drivetrain. One boolean load."""
        return self._active

    def start(self, routine: str, on_ground: bool, sink: ProgressSink) -> None:
        """Begin a sweep. Called on the control thread, which parks afterwards."""
        if routine not in CALIBRATION_ROUTINES:
            raise ValueError(
                f"unknown calibration routine {routine!r}; "
                f"expected one of {list(CALIBRATION_ROUTINES)}"
            )
        if self._active:
            raise CalibrationBusy("a calibration run is already in progress")

        self._sink = sink
        self._result = None
        self._calibrator = AutoCalibrator(
            motors=self._motors,
            encoder_l=self._encoder_l,
            encoder_r=self._encoder_r,
            servo=self._servo,
            config=self._config,
            clock=self._clock,
            # On the mock backend the plant only advances when something steps
            # it, and the control thread stops doing that while parked -- so the
            # sweep pumps it at its own cadence instead.
            pump=self._pump,
            on_progress=self._report_stage,
        )
        self._active = True
        self.runs += 1
        self._thread = threading.Thread(
            target=self._run, args=(on_ground,), name="telekart-calibrate", daemon=True
        )
        self._thread.start()

    def cancel(self) -> None:
        """Stop at the next settling slice. ``abort`` is one boolean store, and
        the sweep checks it every 10 ms, so this returns immediately."""
        calibrator = self._calibrator
        if calibrator is not None:
            calibrator.abort()

    def take_result(self) -> DriveCalibration | None:
        """Collect a finished sweep's output exactly once, from the control
        thread, which is the only thread allowed to hand it to the loop."""
        result, self._result = self._result, None
        return result

    def join(self, timeout: float) -> None:
        thread = self._thread
        if thread is not None:
            thread.join(timeout)

    # -- the worker ---------------------------------------------------------

    def _run(self, on_ground: bool) -> None:
        calibrator = self._calibrator
        assert calibrator is not None
        try:
            result = calibrator.run(on_ground=on_ground)
        except CalibrationAborted as exc:
            self._finish(CalibrationProgress("cancelled", 1.0, done=True, error=str(exc)))
            return
        except CalibrationError as exc:
            self.failures += 1
            self._finish(CalibrationProgress("failed", 1.0, done=True, error=str(exc)))
            return
        except Exception as exc:  # noqa: BLE001 - a bad sweep must not kill the car
            self.failures += 1
            _log.exception("calibration sweep failed")
            self._finish(CalibrationProgress("failed", 1.0, done=True, error=repr(exc)))
            return

        try:
            # Written here rather than on the control thread: an SD-card write
            # can block for hundreds of milliseconds, and this thread has
            # nothing left to be late for.
            result.save(self._path)
            saved = True
        except Exception as exc:  # noqa: BLE001
            saved = False
            _log.error("could not save the calibration", path=str(self._path),
                       error=str(exc))

        self._result = result
        self._finish(
            CalibrationProgress(
                "done",
                1.0,
                done=True,
                result={
                    "max_rpm": dict(result.max_rpm),
                    "deadband": dict(result.deadband),
                    "max_rpm_measured": result.max_rpm_measured,
                    "on_ground": result.on_ground,
                    "measured_at": result.measured_at,
                    "saved": saved,
                    "path": str(self._path),
                },
            )
        )

    def _report_stage(self, stage: str, fraction: float) -> None:
        sink = self._sink
        if sink is not None:
            sink(CalibrationProgress(stage, fraction))

    def _finish(self, report: CalibrationProgress) -> None:
        sink = self._sink
        # Cleared before the last report goes out, so a client that reacts to
        # `done` by starting another sweep is not told the car is busy.
        self._sink = None
        self._active = False
        self._motors.coast()
        if sink is not None:
            sink(report)


# --------------------------------------------------------------------------
# The control thread
# --------------------------------------------------------------------------


class ControlThread(threading.Thread):
    """The 100 Hz loop, and the only thread allowed to touch the drivetrain."""

    def __init__(
        self,
        *,
        clock: Clock,
        drive: DriveController,
        safety: SafetyStateMachine,
        odometry: BicycleOdometry,
        motors: MotorPair,
        config: VehicleConfig,
        controls: LocalControls,
        notifier: SystemdNotifier,
        panic: HardwarePanic,
        health: HealthMonitor,
        command_box: Mailbox[DriveCommand],
        sample_box: Mailbox[VehicleSample],
        fault_box: Mailbox[int],
        requests: queue.SimpleQueue[_Request],
        calibration: CalibrationJob,
        on_fatal: Callable[[BaseException], None],
        plant_step: Callable[[float], None] | None = None,
        period: float = CONTROL_PERIOD_S,
    ) -> None:
        super().__init__(name="telekart-control", daemon=False)
        self._clock = clock
        self._drive = drive
        self._safety = safety
        self._odometry = odometry
        self._motors = motors
        self._config = config
        self._controls = controls
        self._notifier = notifier
        self._panic = panic
        self._health = health
        self._command_box = command_box
        self._sample_box = sample_box
        self._fault_box = fault_box
        self._requests = requests
        self._calibration = calibration
        self._on_fatal = on_fatal
        self._plant_step = plant_step
        self._period = period

        self._stop_event = threading.Event()
        self._calibrating = False
        self._estop_latched = False
        self._seen_overruns = 0
        self._consecutive_overruns = 0
        self._overrun_fault_raised = False
        self._led_slot = 0
        self._led_accumulator = 0.0
        self._watchdog_due = 0.0
        self._publish_toggle = False
        self._last_state: DriveState | None = None

        self.gc_controller = rt.GcController()
        self.scheduler: DeadlineScheduler | None = None
        self.rt_status: rt.RtStatus | None = None
        self.ticks = 0
        self.started_at = 0.0
        #: Written by the supervisor on the asyncio thread and read here when a
        #: sample is built. Computing a percentile means scanning two thousand
        #: histogram buckets, which is far too much work to do inside a 10 ms
        #: budget -- and the answer is a diagnostic, so a second of staleness is
        #: irrelevant. One writer, one reader, one integer: no lock needed.
        self.loop_p99_us = 0

    # -- control-plane entry points (called from the asyncio thread) ---------

    def submit(self, request: _Request) -> None:
        self._requests.put(request)

    def stop(self) -> None:
        self._stop_event.set()

    # -- the loop -----------------------------------------------------------

    def run(self) -> None:
        # Per-thread on Linux, which is why this is here and not in main(): the
        # asyncio thread must stay on CFS, where a blocking call cannot lock up
        # the machine.
        self.rt_status = rt.apply_realtime(
            priority=RT_PRIORITY_CONTROL,
            cpus=CONTROL_CPU_AFFINITY,
            gc_controller=self.gc_controller,
        )

        scheduler = DeadlineScheduler(self._clock, self._period)
        self.scheduler = scheduler
        scheduler.start()
        self.started_at = self._clock.monotonic()
        self._watchdog_due = self.started_at

        try:
            while not self._stop_event.is_set():
                dt = scheduler.wait_next()
                self._tick(dt, scheduler)
        except BaseException as exc:  # noqa: BLE001 - see below
            # An exception here is a bug, and a bug in the loop that owns the
            # H-bridge is not survivable. Safe the hardware, then hand the
            # failure to the lifecycle so the process exits non-zero and systemd
            # restarts it -- rather than leaving a dead thread and live motors.
            self._panic.stop()
            _log.exception("control loop failed", tick=self.ticks)
            try:
                self._on_fatal(exc)
            except BaseException:  # noqa: BLE001
                pass
        finally:
            self._shutdown()

    def _tick(self, dt: float, scheduler: DeadlineScheduler) -> None:
        self.ticks += 1
        calibrating = self._calibrating

        step = self._plant_step
        if step is not None and not calibrating:
            # Development mode only: advance the simulated drivetrain in
            # lockstep so `--backend mock` behaves like a car rather than a dead
            # bench. None on real hardware, so the branch costs one null test.
            # Skipped during a sweep, which pumps the plant itself.
            step(dt)

        self._serve_requests()

        pending = self._fault_box.take()
        if pending:
            self._safety.raise_fault(Fault(pending), "host health")

        if self._controls.pressed:
            if not self._estop_latched:
                self._estop_latched = True
                self._safety.request_estop()
                if calibrating:
                    self._calibration.cancel()
        else:
            # Releasing the button does not clear the E-stop -- that needs an
            # explicit CLEAR_ESTOP -- it only re-arms the edge detector.
            self._estop_latched = False

        command = self._command_box.take()
        if command is not None:
            # Noted even while parked. The operator's app keeps streaming at
            # 100 Hz throughout a sweep, and letting the safety machine believe
            # the link went stale for forty seconds would produce a spurious
            # CONTROL_TIMEOUT the moment the loop resumed.
            self._safety.note_control_packet()

        if calibrating:
            self._poll_calibration()
            state = self._last_state
        else:
            if command is not None:
                self._drive.submit_command(command)
            state = self._drive.tick(dt)
            self._last_state = state

        self._track_overruns(scheduler)
        self._publish(state)
        self._update_led(dt, state)
        self._feed_watchdog(self._clock.monotonic())

    def _track_overruns(self, scheduler: DeadlineScheduler) -> None:
        """One overrun is a scheduling hiccup; a run of them means the loop does
        not fit in its period, and the driver deserves to be told."""
        overruns = scheduler.overruns
        if overruns > self._seen_overruns:
            self._seen_overruns = overruns
            self._consecutive_overruns += 1
            if (
                self._consecutive_overruns >= LOOP_OVERRUN_FAULT_COUNT
                and not self._overrun_fault_raised
            ):
                self._overrun_fault_raised = True
                self._safety.raise_fault(
                    Fault.LOOP_OVERRUN, "control loop missed consecutive deadlines"
                )
        elif self._consecutive_overruns:
            self._consecutive_overruns = 0

    def _publish(self, state: DriveState | None) -> None:
        # Telemetry goes out at half the control rate, so building the snapshot
        # on alternate ticks costs one small immutable object per 20 ms and
        # nothing at all on the ticks in between.
        self._publish_toggle = not self._publish_toggle
        if not self._publish_toggle:
            return
        self._sample_box.put(
            _sample_from_state(
                state,
                safety=self._safety,
                throttled=self._health.throttled,
                cpu_temp_c=self._health.cpu_temp_c,
                loop_p99_us=self.loop_p99_us,
            )
        )

    def _update_led(self, dt: float, state: DriveState | None) -> None:
        self._led_accumulator += dt
        if self._led_accumulator < STATUS_LED_PERIOD_S:
            return
        self._led_accumulator = 0.0
        self._led_slot += 1
        vehicle_state = state.state if state is not None else self._safety.state
        self._controls.update_led(vehicle_state, self._led_slot)

    def _feed_watchdog(self, now: float) -> None:
        interval = self._notifier.watchdog_interval
        if interval <= 0.0 or now < self._watchdog_due:
            return
        self._watchdog_due = now + interval
        self._notifier.ping()

    # -- calibration --------------------------------------------------------

    def _poll_calibration(self) -> None:
        """Notice that the sweep has finished and take the drivetrain back."""
        job = self._calibration
        if job.active:
            return
        self._calibrating = False
        result = job.take_result()
        if result is not None:
            # Adopting on this thread is the point: the loop is the only reader
            # of the calibration, so handing it over here means no tick ever
            # sees half of an old table and half of a new one.
            self._drive.calibration = result
            # A sweep that measured all four directions is a strong statement
            # that the drivetrain works, which is exactly what the sticky faults
            # from before it were about. Anything still genuinely wrong -- an
            # undervolting supply, a hot SoC -- is re-raised within the second
            # by the health poller.
            self._safety.clear_faults()
            self._overrun_fault_raised = False
        self._motors.clear_panic()
        # The sweep held the drivetrain for tens of seconds of wall time. Not
        # re-anchoring here would charge the resumed loop with one enormous
        # overrun and a p99 that means nothing for the rest of the session.
        scheduler = self.scheduler
        if scheduler is not None:
            scheduler.start()

    # -- control-plane execution --------------------------------------------

    def _serve_requests(self) -> None:
        requests = self._requests
        for _ in range(MAX_REQUESTS_PER_TICK):
            try:
                request = requests.get_nowait()
            except queue.Empty:
                return
            try:
                result: Any = self._execute(request)
            except BaseException as exc:  # noqa: BLE001 - answered, never raised
                result = exc
            request.resolve(result)

    def _execute(self, request: _Request) -> Any:
        kind = request.kind
        if kind == "arm":
            return self._safety.request_arm()
        if kind == "disarm":
            self._safety.request_disarm()
            return None
        if kind == "estop":
            self._safety.request_estop()
            return None
        if kind == "clear_estop":
            return self._safety.clear_estop()
        if kind == "clear_faults":
            self._safety.clear_faults()
            self._overrun_fault_raised = False
            return None
        if kind == "reset_odom":
            self._odometry.reset()
            return None
        if kind == "session_valid":
            # One of the four arming conditions, and a disarm trigger on the way
            # down. Routed through the queue rather than called from the socket
            # handler so it is ordered against every other safety transition.
            self._safety.set_session_valid(bool(request.payload))
            return None
        if kind == "params":
            return self._apply_params(request.payload)
        if kind == "calibrate":
            return self._start_calibration(request.payload)
        if kind == "cancel_calibration":
            return self._cancel_calibration()
        raise ValueError(f"unknown control request {kind!r}")

    def _apply_params(self, payload: Any) -> dict[str, Any]:
        """Applied here rather than on the event loop so the tick that reads a
        parameter can never see half of a change. Costs one log line and a small
        dict on an operator action, which is a rate nobody can measure."""
        values = dict(payload)
        # Raises ParamError listing every problem, having applied nothing. The
        # session handler turns that into PARAM_OUT_OF_RANGE.
        self._config.apply_params(values)
        # Gains and motor inversion are cached inside the loop; nothing notices
        # a change by itself.
        self._drive.refresh_params()
        return {name: getattr(self._config, name) for name in values}

    def _start_calibration(self, payload: Any) -> None:
        routine, on_ground, sink = payload
        job = self._calibration
        if self._calibrating or job.active:
            raise CalibrationBusy("a calibration run is already in progress")
        if self._safety.state is VehicleState.ARMED:
            raise NotAllowedInState("disarm before starting a calibration run")
        # The sweep takes the drivetrain from here. Draining the mailbox stops a
        # command that arrived a moment ago from being replayed at it, and
        # clearing the panic latch is what lets the sweep drive at all.
        self._command_box.take()
        self._drive.panic_stop()
        self._motors.clear_panic()
        # Raises ValueError for an unknown routine, which the session handler
        # turns into BAD_REQUEST; nothing has been parked at that point.
        job.start(routine, on_ground, sink)
        self._calibrating = True
        return None

    def _cancel_calibration(self) -> None:
        job = self._calibration
        if job.active:
            job.cancel()
        elif self._calibrating:
            # The sweep finished between the request and this tick.
            self._poll_calibration()
        return None

    # -- teardown -----------------------------------------------------------

    def _shutdown(self) -> None:
        if self._calibrating:
            # The sweep thread is holding the H-bridge. Ask it to stop and give
            # it a couple of settling slices before the panic stop goes in
            # underneath it, so it is not still writing duty afterwards.
            self._calibration.cancel()
            self._calibration.join(1.0)
            self._calibrating = False
        try:
            self._drive.panic_stop()
        except BaseException:  # noqa: BLE001
            self._panic.stop()
        else:
            self._panic.stop(announce=False)
        self.gc_controller.restore()
        _log.info(
            "control loop stopped",
            ticks=self.ticks,
            seconds=round(self._clock.monotonic() - self.started_at, 1),
        )


def _sample_from_state(
    state: DriveState | None,
    *,
    safety: SafetyStateMachine,
    throttled: int,
    cpu_temp_c: float,
    loop_p99_us: int,
) -> VehicleSample:
    """Flatten a control-loop result into the immutable telemetry snapshot.

    Built on the control thread so the asyncio side never reads live loop state
    and can never observe half of one iteration and half of the next.
    """
    if state is None:
        # No tick has completed yet, or a calibration owns the drivetrain. The
        # safety state is still authoritative and still worth publishing.
        return VehicleSample(
            state=safety.state,
            faults=safety.faults,
            throttled=throttled,
            cpu_temp_c=cpu_temp_c,
            loop_p99_us=loop_p99_us,
        )
    x, y, heading = state.pose
    return VehicleSample(
        state=state.state,
        faults=state.faults,
        flags=state.flags,
        rpm_l=state.rpm_l,
        rpm_r=state.rpm_r,
        rpm_target_l=state.rpm_target_l,
        rpm_target_r=state.rpm_target_r,
        duty_l=state.duty_l,
        duty_r=state.duty_r,
        servo_us=state.servo_us,
        steer_angle_deg=math.degrees(state.steer_angle),
        speed_mps=state.speed,
        v_max_mps=state.v_max,
        x_m=x,
        y_m=y,
        heading_rad=heading,
        distance_m=state.distance,
        slip=state.slip,
        cpu_temp_c=cpu_temp_c,
        throttled=throttled,
        loop_p99_us=loop_p99_us,
    )


# --------------------------------------------------------------------------
# The application
# --------------------------------------------------------------------------


@dataclass(slots=True)
class AppOptions:
    config_path: Path = DEFAULT_CONFIG_PATH
    local_config_path: Path | None = LOCAL_CONFIG_PATH
    calibration_path: Path = CALIBRATION_PATH
    backend: str = BACKEND_AUTO
    log_level: str = "INFO"
    json_logs: bool = False
    log_file: Path | None = None
    bind_host: str = "0.0.0.0"
    control_port: int = UDP_CONTROL_PORT
    session_port: int = TCP_SESSION_PORT
    video_port: int = TCP_VIDEO_PORT
    mdns: bool = True
    use_defaults: bool = False
    duration: float = 0.0


class Application:
    """Owns every object in the process, and the order they live and die in."""

    def __init__(self, options: AppOptions, config: VehicleConfig, gpio: GpioBackend) -> None:
        self.options = options
        self.config = config
        self.gpio = gpio
        self.clock: Clock = RealClock()

        self._command_box: Mailbox[DriveCommand] = Mailbox()
        self._sample_box: Mailbox[VehicleSample] = Mailbox(_EMPTY_SAMPLE)
        self._fault_box: Mailbox[int] = Mailbox()
        self._requests: queue.SimpleQueue[_Request] = queue.SimpleQueue()

        self.notifier = SystemdNotifier()
        self.health = HealthMonitor(self._fault_box)
        self.panic = HardwarePanic(gpio, config.pins, config.pwm_hz)

        # --- drivers -------------------------------------------------------
        self.motors = MotorPair(gpio, config.pins.motors, config, self.clock)
        self.servo = SteeringServo(gpio, config.pins.servo, config, self.clock)
        # Attached the moment they exist: between here and the end of
        # construction there is real hardware and no supervisor.
        self.panic.attach(self.motors, self.servo)

        encoders = config.pins.encoders
        self.encoder_l = QuadratureEncoder(
            gpio,
            encoders.left_a,
            encoders.left_b,
            cpr=config.encoder_cpr,
            invert=bool(config.encoder_invert_left),
            glitch_us=ENCODER_GLITCH_US,
        )
        self.encoder_r = QuadratureEncoder(
            gpio,
            encoders.right_a,
            encoders.right_b,
            cpr=config.encoder_cpr,
            invert=bool(config.encoder_invert_right),
            glitch_us=ENCODER_GLITCH_US,
        )

        # --- control -------------------------------------------------------
        self.safety = SafetyStateMachine(config, self.clock)
        self.odometry = BicycleOdometry(config)
        # An empty DriveCalibration is the correct state for a car that has
        # never been calibrated: every accessor degrades to a conservative
        # provisional value and `is_measured` is False, which is what makes the
        # drive controller raise Fault.CALIBRATION_MISSING at construction.
        self.calibration = DriveCalibration.load(options.calibration_path)
        if self.calibration is None:
            _log.warning(
                "no calibration on file; speed targets and feedforward are guesses",
                path=str(options.calibration_path),
            )
            self.calibration = DriveCalibration()

        self.drive = DriveController(
            gpio=gpio,
            config=config,
            clock=self.clock,
            motors=self.motors,
            servo=self.servo,
            encoder_l=self.encoder_l,
            encoder_r=self.encoder_r,
            safety=self.safety,
            odometry=self.odometry,
            calibration=self.calibration,
        )

        self.controls = LocalControls(gpio, config.pins)
        self.calibration_job = CalibrationJob(
            config=config,
            clock=self.clock,
            motors=self.motors,
            servo=self.servo,
            encoder_l=self.encoder_l,
            encoder_r=self.encoder_r,
            path=options.calibration_path,
            pump=_plant_step_for(gpio),
        )

        # --- network -------------------------------------------------------
        self.link = ControlLink(
            clock=self.clock,
            on_command=self._on_control_packet,
            host=options.bind_host,
            port=options.control_port,
        )
        self.telemetry = TelemetrySender(clock=self.clock, source=self, echo=self.link)
        self.service = VehicleServiceImpl(self)
        self.sessions = SessionServer(
            config=config,
            clock=self.clock,
            service=self.service,
            on_open=self._on_session_open,
            on_close=self._on_session_close,
            host=options.bind_host,
            port=options.session_port,
            control_port=options.control_port,
            video_port=options.video_port,
            activity=lambda: self.link.last_accept_time,
        )
        self.discovery = Discovery(
            car_id=config.car_id,
            port=options.session_port,
            control_port=options.control_port,
            video_port=options.video_port,
        )

        self.control_thread = ControlThread(
            clock=self.clock,
            drive=self.drive,
            safety=self.safety,
            odometry=self.odometry,
            motors=self.motors,
            config=config,
            controls=self.controls,
            notifier=self.notifier,
            panic=self.panic,
            health=self.health,
            command_box=self._command_box,
            sample_box=self._sample_box,
            fault_box=self._fault_box,
            requests=self._requests,
            calibration=self.calibration_job,
            on_fatal=self._on_control_fatal,
            plant_step=_plant_step_for(gpio),
        )

        self._loop: asyncio.AbstractEventLoop | None = None
        self._shutdown = asyncio.Event()
        self._fatal: BaseException | None = None
        self._gc_last = 0.0
        self._jitter_log = RateLimiter(30.0)
        self._background: set[asyncio.Task[Any]] = set()

    # -- TelemetrySource ----------------------------------------------------

    def sample(self) -> VehicleSample:
        """Latest published snapshot. One attribute load; never blocks."""
        return self._sample_box.peek() or _EMPTY_SAMPLE

    # -- network callbacks --------------------------------------------------

    def _on_control_packet(self, packet: ControlPacket, received_at: float) -> None:
        """Runs on the asyncio thread, once per accepted control packet.

        The immutable command is built *here* rather than on the control thread
        so the allocation lands where a few microseconds do not matter. The
        control thread only ever loads a reference out of the mailbox.
        """
        self._command_box.put(
            DriveCommand(
                steering=packet.steering_f,
                throttle=packet.throttle_f,
                brake=packet.brake_f,
                flags=packet.flags,
                received_at=received_at,
            )
        )

    def _on_session_open(self, info: SessionInfo) -> None:
        self.link.open_session(info.session_id, info.peer_host)
        self.telemetry.open_session(info.session_id, info.telemetry_addr)
        # A valid session is one of the four things arming requires. Without
        # this the car would refuse every ARM and give a reason nobody could act
        # on from the app.
        self._spawn(self.call("session_valid", True))

    def _on_session_close(self, info: SessionInfo, reason: str) -> None:
        """Presence lost. The TCP connection *is* the operator, and losing it is
        an E-stop condition even if valid UDP is still arriving from somewhere
        -- which is precisely the case this rule exists to cover."""
        self.link.close_session()
        self.telemetry.close_session()
        self._spawn(self.call("session_valid", False))
        self._spawn(self.service.estop(f"operator link lost: {reason}"))
        _log.warning("presence lost, stopping the vehicle",
                     session=info.session_id, reason=reason)

    def _spawn(self, coro: Any) -> None:
        """Fire and forget on the event loop, keeping a strong reference.

        asyncio only holds a weak one, so a task that nothing references can be
        collected mid-flight -- which here would mean an E-stop that silently
        never happened.
        """
        loop = self._loop
        if loop is None or loop.is_closed():
            coro.close()
            return
        task = loop.create_task(coro)
        self._background.add(task)
        task.add_done_callback(self._background.discard)

    def _on_control_fatal(self, exc: BaseException) -> None:
        """Called from the control thread when its loop died."""
        self._fatal = exc
        loop = self._loop
        if loop is not None and not loop.is_closed():
            try:
                loop.call_soon_threadsafe(self._shutdown.set)
            except RuntimeError:
                pass

    def request_shutdown(self, signum: int | None = None) -> None:
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        try:
            loop.call_soon_threadsafe(self._shutdown.set)
        except RuntimeError:
            return
        if signum is not None:
            _log.info("shutdown requested", signal=signal.Signals(signum).name)

    # -- control-plane RPC --------------------------------------------------

    async def call(self, kind: str, payload: Any = None) -> Any:
        """Run ``kind`` on the control thread and wait for its answer."""
        loop = asyncio.get_running_loop()
        future: asyncio.Future[Any] = loop.create_future()
        self.control_thread.submit(_Request(kind, payload, future, loop))
        try:
            return await asyncio.wait_for(future, REQUEST_TIMEOUT_S)
        except asyncio.TimeoutError as exc:
            _log.error("control thread did not answer", request=kind)
            raise ControlLoopStalled(f"control loop did not answer {kind!r}") from exc

    # -- lifecycle ----------------------------------------------------------

    async def run(self) -> int:
        self._loop = asyncio.get_running_loop()
        self.control_thread.start()

        await self.link.start()
        await self.sessions.start()
        self.telemetry.start()
        if self.options.mdns:
            await self.discovery.start()

        health_task = self._loop.create_task(self.health.run(), name="health")
        supervisor = self._loop.create_task(self._supervise(), name="supervisor")

        self.notifier.ready()
        self.notifier.status(f"telekart {FIRMWARE_VERSION} ready")
        _log.info(
            "telekart control process ready",
            car_id=self.config.car_id,
            firmware=FIRMWARE_VERSION,
            backend=type(self.gpio).__name__,
            config=self.config.summary(),
        )

        if self.options.duration > 0.0:
            self._loop.call_later(self.options.duration, self._shutdown.set)

        try:
            await self._shutdown.wait()
        finally:
            self.notifier.stopping()
            for task in (health_task, supervisor):
                task.cancel()
            await self._teardown()

        if self._fatal is not None:
            _log.critical("exiting because the control loop failed",
                          error=repr(self._fatal))
            return 1
        return 0

    async def _supervise(self) -> None:
        """Housekeeping that must not run on the control thread.

        Pushes state changes to the operator, computes the loop's p99 (a scan of
        two thousand histogram buckets, which is far too much work for a 10 ms
        budget), triggers the deliberate garbage collections, and notices if the
        control thread has gone away.
        """
        counter = 0
        while True:
            await asyncio.sleep(SUPERVISOR_PERIOD_S)
            counter += 1
            sample = self.sample()
            self.sessions.publish_state(int(sample.state), int(sample.faults))

            if counter % TELEMETRY_RATE_HZ == 0:
                self._refresh_loop_stats()
                now = self.clock.monotonic()
                if now - self._gc_last >= GC_COLLECT_PERIOD_S:
                    self._gc_last = now
                    # Deliberate, on this thread, where a 20 ms pause is invisible.
                    self.control_thread.gc_controller.maybe_collect(
                        now, GC_COLLECT_PERIOD_S
                    )

            if not self.control_thread.is_alive() and self._fatal is None:
                self._fatal = RuntimeError("control thread exited unexpectedly")
                self._shutdown.set()
                return

    def _refresh_loop_stats(self) -> None:
        scheduler = self.control_thread.scheduler
        if scheduler is None:
            return
        snapshot = scheduler.stats.snapshot()
        self.control_thread.loop_p99_us = snapshot.p99_us
        if snapshot.exceeds(LOOP_P99_BUDGET_S) and self._jitter_log.allow(
            self.clock.monotonic()
        ):
            _log.warning(
                "control loop p99 over budget",
                p99_ms=round(snapshot.p99 * 1e3, 2),
                max_ms=round(snapshot.max * 1e3, 2),
                overruns=scheduler.overruns,
                suppressed=self._jitter_log.take_suppressed(),
            )

    async def _teardown(self) -> None:
        await self.discovery.stop()
        await self.sessions.stop()
        await self.telemetry.stop()
        await self.link.stop()

        self.control_thread.stop()
        # Generous relative to a 10 ms period: the thread only has to finish the
        # tick it is in. If it cannot, the panic stop in main()'s context manager
        # and systemd's ExecStopPost both still run.
        self.control_thread.join(timeout=2.0)
        if self.control_thread.is_alive():
            _log.error("control thread did not stop; forcing outputs safe")
            self.panic.stop()

        self.controls.close()
        self.notifier.close()
        try:
            self.gpio.cleanup()
        except Exception:  # noqa: BLE001 - cleanup path
            _log.exception("gpio cleanup failed")


# --------------------------------------------------------------------------
# The service the session channel talks to
# --------------------------------------------------------------------------


class VehicleServiceImpl:
    """Adapts the session channel's requests onto the control thread.

    Every mutating method is a round trip. When the control loop is not
    answering, the tuple-returning calls report that as a refusal rather than
    raising: "arm was refused because the loop is wedged" is a useful thing to
    see on a HUD, and a traceback is not.
    """

    __slots__ = ("_app",)

    def __init__(self, app: Application) -> None:
        self._app = app

    # -- reads --------------------------------------------------------------

    def snapshot(self) -> tuple[int, int]:
        sample = self._app.sample()
        return (int(sample.state), int(sample.faults))

    def is_armed(self) -> bool:
        return self._app.sample().state == VehicleState.ARMED

    def parameters(self) -> dict[str, Any]:
        return self._app.config.as_params()

    # -- safety -------------------------------------------------------------

    async def arm(self) -> tuple[bool, str]:
        try:
            accepted, reason = await self._app.call("arm")
        except ControlLoopStalled as exc:
            return (False, str(exc))
        return (bool(accepted), str(reason))

    async def disarm(self) -> None:
        await self._quiet("disarm")

    async def estop(self, reason: str) -> None:
        _log.warning("emergency stop", reason=reason)
        await self._quiet("estop")

    async def clear_estop(self) -> tuple[bool, str]:
        try:
            cleared, reason = await self._app.call("clear_estop")
        except ControlLoopStalled as exc:
            return (False, str(exc))
        return (bool(cleared), str(reason))

    async def clear_faults(self) -> None:
        await self._quiet("clear_faults")

    async def reset_odometry(self) -> None:
        await self._quiet("reset_odom")

    async def _quiet(self, kind: str) -> None:
        """Fire a request whose only failure mode is a stalled loop -- which the
        watchdog is already dealing with."""
        try:
            await self._app.call(kind)
        except ControlLoopStalled:
            _log.error("request dropped; control loop is not responding", request=kind)

    # -- parameters ---------------------------------------------------------

    async def apply_parameters(
        self, values: dict[str, Any], *, persist: bool = False
    ) -> dict[str, Any]:
        try:
            applied = await self._app.call("params", values)
        except ControlLoopStalled as exc:
            raise NotAllowedInState(str(exc)) from exc
        if persist:
            # File I/O on the event loop's worker pool, never on the control
            # thread: an SD card write can block for hundreds of milliseconds.
            await asyncio.to_thread(self._save_local)
        return dict(applied)

    def _save_local(self) -> None:
        target = self._app.options.local_config_path or LOCAL_CONFIG_PATH
        try:
            self._app.config.save_local(target)
        except (ConfigError, OSError) as exc:
            _log.error("could not persist parameters", path=str(target), error=str(exc))

    # -- calibration --------------------------------------------------------

    async def start_calibration(
        self, routine: str, on_ground: bool, progress: ProgressSink
    ) -> None:
        try:
            await self._app.call("calibrate", (routine, on_ground, progress))
        except ControlLoopStalled as exc:
            raise CalibrationUnavailable(str(exc)) from exc

    async def cancel_calibration(self) -> None:
        await self._quiet("cancel_calibration")


# --------------------------------------------------------------------------
# Startup
# --------------------------------------------------------------------------


def _plant_step_for(gpio: GpioBackend) -> Callable[[float], None] | None:
    """``MockBackend.step`` when the mock is in use, otherwise nothing.

    Bound once so the control loop's check is a null test on a local, not a
    ``getattr`` on every tick.
    """
    step = getattr(gpio, "step", None)
    return step if callable(step) else None


def _attach_mock_context(gpio: GpioBackend, config: VehicleConfig, clock: Clock) -> None:
    """Give a mock backend the clock and pin map it cannot infer for itself.

    ``select_backend`` has no generic way to pass these through, and a mock
    without a clock timestamps its synthesised encoder edges from a counter that
    never advances -- which looks exactly like a dead encoder.
    """
    try:
        from .hal.mock_backend import MockBackend, MockPins
    except ImportError:  # pragma: no cover - the mock always ships
        return
    if not isinstance(gpio, MockBackend):
        return
    if gpio.clock is None:
        gpio.clock = clock
    gpio.pins = MockPins.from_hardware_pins(config.pins)


def load_config(options: AppOptions) -> VehicleConfig:
    if options.use_defaults:
        config = VehicleConfig()
        # VehicleConfig.load() applies the environment itself; the bare
        # constructor does not.
        car_id = os.environ.get(ENV_CAR_ID)
        if car_id:
            config.car_id = car_id
        config.check()
        _log.warning("running on registry defaults; no config file was read")
        return config
    return VehicleConfig.load(options.config_path, options.local_config_path)


def build_backend(options: AppOptions, config: VehicleConfig, clock: Clock) -> GpioBackend:
    gpio = select_backend((options.backend or BACKEND_AUTO).strip().lower())
    _attach_mock_context(gpio, config, clock)
    return gpio


def parse_args(argv: Iterable[str] | None = None) -> AppOptions:
    parser = argparse.ArgumentParser(
        prog="telekart-control",
        description="TeleKart vehicle control process: 100 Hz loop, UDP control "
        "and telemetry, TCP session channel.",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH,
                        help="base configuration file")
    parser.add_argument("--local-config", type=Path, default=LOCAL_CONFIG_PATH,
                        help="per-vehicle overlay (missing is normal)")
    parser.add_argument("--calibration", type=Path, default=CALIBRATION_PATH,
                        help="machine-written calibration file")
    parser.add_argument("--defaults", action="store_true",
                        help="ignore the config files and use registry defaults")
    parser.add_argument("--backend", default=BACKEND_AUTO,
                        help=f"gpio backend: auto, pigpio, or mock "
                             f"(overrides ${ENV_BACKEND})")
    parser.add_argument("--log-level", default=os.environ.get(ENV_LOG_LEVEL, "INFO"))
    parser.add_argument("--json-logs", action="store_true",
                        help="one JSON object per line instead of text")
    parser.add_argument("--log-file", type=Path, default=None,
                        help="additionally write rotated JSON logs here")
    parser.add_argument("--bind", default="0.0.0.0", help="address to listen on")
    parser.add_argument("--control-port", type=int, default=UDP_CONTROL_PORT)
    parser.add_argument("--session-port", type=int, default=TCP_SESSION_PORT)
    parser.add_argument("--video-port", type=int, default=TCP_VIDEO_PORT,
                        help="advertised only; the video process owns that socket")
    parser.add_argument("--no-mdns", action="store_true",
                        help="skip the mDNS advertisement; connect by IP")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="exit cleanly after N seconds (soak tests and CI)")
    args = parser.parse_args(list(argv) if argv is not None else None)

    return AppOptions(
        config_path=args.config,
        local_config_path=args.local_config,
        calibration_path=args.calibration,
        backend=args.backend,
        log_level=args.log_level,
        json_logs=args.json_logs,
        log_file=args.log_file,
        bind_host=args.bind,
        control_port=args.control_port,
        session_port=args.session_port,
        video_port=args.video_port,
        mdns=not args.no_mdns,
        use_defaults=args.defaults,
        duration=max(0.0, args.duration),
    )


def main(argv: Iterable[str] | None = None) -> int:
    options = parse_args(argv)
    setup_logging(
        level=options.log_level, json_output=options.json_logs, path=options.log_file
    )

    # The kernel must take the camera and never the car. Done before anything
    # large is allocated, so the bias is in place for the whole of startup.
    rt.set_oom_score_adj(OOM_SCORE_CONTROL)

    clock = RealClock()
    try:
        config = load_config(options)
    except ConfigError as exc:
        _log.critical("configuration is unusable", error=str(exc))
        return 2

    try:
        gpio = build_backend(options, config, clock)
    except Exception as exc:  # noqa: BLE001 - startup failures are reported, not raised
        _log.critical("gpio backend unavailable", error=str(exc))
        return 3

    # The panic object exists before any driver does, so a failure part way
    # through construction still leaves the bridge coasting.
    panic = HardwarePanic(gpio, config.pins, config.pwm_hz)
    application: Application | None = None

    def on_signal(signum: int) -> None:
        if application is not None:
            application.request_shutdown(signum)

    with PanicChain(panic, on_signal):
        try:
            application = Application(options, config, gpio)
        except Exception as exc:  # noqa: BLE001
            _log.critical("startup failed", error=repr(exc))
            _log.exception("startup traceback")
            return 4
        # Hand the fully built drivers to the panic object that is already
        # armed, rather than swapping in a second one.
        panic.attach(application.motors, application.servo)

        try:
            return asyncio.run(application.run())
        except KeyboardInterrupt:
            return 0
        except Exception as exc:  # noqa: BLE001
            _log.critical("control process failed", error=repr(exc))
            _log.exception("failure traceback")
            return 5


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())


__all__ = [
    "CALIBRATION_ROUTINES",
    "AppOptions",
    "Application",
    "CalibrationJob",
    "ControlLoopStalled",
    "ControlThread",
    "HardwarePanic",
    "HealthMonitor",
    "LocalControls",
    "Mailbox",
    "PanicChain",
    "SystemdNotifier",
    "VehicleServiceImpl",
    "build_backend",
    "load_config",
    "main",
    "parse_args",
]
