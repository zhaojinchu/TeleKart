"""The 50 Hz telemetry stream, car to app.

Two things here are not obvious and both matter.

**The echo.** Every packet carries the ``client_time_us`` and ``sequence`` of
the most recently accepted control packet, copied verbatim. The app subtracts
that from its own monotonic clock and gets true round-trip latency without the
two clocks ever having to agree, without NTP, and without a separate ping. It
costs twelve bytes and it is the single most useful number on the HUD.

**The socket is deliberately raw.** A 98-byte fire-and-forget datagram does not
need an asyncio transport, a protocol object, or a flow-control callback. A
non-blocking ``sendto`` is one syscall; the transport machinery around it is
several allocations and a couple of microseconds of Python per packet, for no
behaviour we want.
"""

from __future__ import annotations

import asyncio
import socket
from dataclasses import dataclass
from typing import Protocol

from telekart_protocol import Fault, TelemetryFlags, TelemetryPacket, VehicleState
from telekart_protocol.constants import TELEMETRY_RATE_HZ

from ..log import RateLimiter, get_logger
from ..util.clock import Clock, JitterStats

_log = get_logger(__name__)

_SEND_LOG_INTERVAL_S = 5.0

#: DSCP EF. Access points map this to the WMM "voice" access category, which
#: gets a shorter contention window than best effort. On a congested 2.4 GHz
#: channel that is worth several milliseconds of the latency budget, and the car
#: sends 5 kB/s, so it is not a class of traffic anyone need feel bad about
#: prioritising.
_IPTOS_DSCP_EF = 0xB8


@dataclass(frozen=True, slots=True)
class VehicleSample:
    """One consistent snapshot of the vehicle, in SI units.

    Immutable and built in one place per tick, so a packet can never contain the
    speed from one control iteration and the pose from the next. Mirrors the
    argument list of :meth:`TelemetryPacket.from_si` exactly; the scaling to
    wire integers stays inside ``telekart_protocol`` where it belongs.
    """

    state: VehicleState = VehicleState.BOOT
    faults: Fault = Fault.NONE
    flags: TelemetryFlags = TelemetryFlags.NONE

    rpm_l: float = 0.0
    rpm_r: float = 0.0
    rpm_target_l: float = 0.0
    rpm_target_r: float = 0.0
    duty_l: float = 0.0
    duty_r: float = 0.0

    servo_us: int = 0
    steer_angle_deg: float = 0.0

    speed_mps: float = 0.0
    #: Measured, never a nameplate figure. The app scales its speedometer off
    #: this, which is what lets the same build drive an L298N-limited car today
    #: and a MOSFET-bridge car later with no change on either side.
    v_max_mps: float = 0.0

    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    distance_m: float = 0.0
    slip: float = 0.0

    pack_volts: float = 0.0
    cpu_temp_c: float = 0.0
    #: Raw vcgencmd get_throttled bitmask, passed through unmodified so the app
    #: can decode bits this firmware does not know about.
    throttled: int = 0
    loop_p99_us: float = 0.0


class TelemetrySource(Protocol):
    """Whatever can produce a :class:`VehicleSample`. Implemented by the app."""

    def sample(self) -> VehicleSample:
        """Called once per telemetry period on the asyncio thread. Must not
        block and must not touch the control thread's mutable state."""
        ...


class ControlEcho(Protocol):
    """The two fields copied out of the last accepted control packet.

    Satisfied by :class:`~telekart.net.control_link.ControlLink`, which runs on
    the same thread as the sender, so these are plain attribute reads.
    """

    echo_client_time_us: int
    echo_sequence: int


@dataclass(slots=True)
class TelemetryStats:
    sent: int = 0
    errors: int = 0
    #: Periods where no session was open, so there was nowhere to send.
    idle: int = 0
    last_error: str = ""


class TelemetrySender:
    """Paces, builds, signs and sends the telemetry stream."""

    def __init__(
        self,
        *,
        clock: Clock,
        source: TelemetrySource,
        echo: ControlEcho,
        rate_hz: int = TELEMETRY_RATE_HZ,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError(f"telemetry rate must be positive, got {rate_hz}")
        self._clock = clock
        self._source = source
        self._echo = echo
        self._period = 1.0 / rate_hz

        self._sock: socket.socket | None = None
        self._session_id = 0
        self._key = b""
        self._addr: tuple[str, int] | None = None
        self._sequence = 0
        self._running = False
        self._task: asyncio.Task[None] | None = None

        self.stats = TelemetryStats()
        #: Period jitter of the sender itself. If this is bad while the control
        #: loop's p99 is fine, the problem is the event loop, not the car.
        self.jitter = JitterStats()
        self._error_log = RateLimiter(_SEND_LOG_INTERVAL_S)

    # -- lifecycle ----------------------------------------------------------

    def open(self) -> None:
        """Create the socket. Separate from :meth:`run` so a failure to build a
        socket is a startup error rather than a task that dies later."""
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, _IPTOS_DSCP_EF)
        except OSError as exc:
            _log.debug("could not set telemetry DSCP", error=str(exc))
        self._sock = sock

    def close(self) -> None:
        sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()

    def open_session(self, session_id: int, key: bytes, addr: tuple[str, int]) -> None:
        self._session_id = session_id
        self._key = key
        self._addr = addr
        self._sequence = 0
        self.stats.sent = 0
        self.stats.errors = 0
        self.stats.idle = 0
        _log.info("telemetry session opened", session_id=session_id, dest=f"{addr[0]}:{addr[1]}")

    def close_session(self) -> None:
        if self._session_id:
            _log.info("telemetry session closed", session_id=self._session_id,
                      sent=self.stats.sent, errors=self.stats.errors)
        self._session_id = 0
        self._key = b""
        self._addr = None

    # -- the loop -----------------------------------------------------------

    def start(self) -> asyncio.Task[None]:
        """Schedule :meth:`run` on the running loop and return its task."""
        self.open()
        self._task = asyncio.get_running_loop().create_task(self.run(), name="telemetry-tx")
        return self._task

    async def stop(self) -> None:
        self._running = False
        task, self._task = self._task, None
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        self.close_session()
        self.close()

    async def run(self) -> None:
        """Send one packet per period, paced against absolute deadlines.

        Absolute rather than ``sleep(period)`` for the same reason the control
        loop is: relative sleeps accumulate every scrap of scheduling error, so
        a 50 Hz loop quietly becomes a 47 Hz one. Not reusing
        :class:`DeadlineScheduler` because that one sleeps the *thread*, which
        on the event loop would stop every other coroutine dead.
        """
        self._running = True
        self.open()
        period = self._period
        clock = self._clock
        last_wake = clock.monotonic()
        deadline = last_wake + period
        try:
            while self._running:
                now = clock.monotonic()
                delay = deadline - now
                if delay > 0.0:
                    await asyncio.sleep(delay)
                    now = clock.monotonic()

                self.jitter.add(now - last_wake)
                last_wake = now

                self.send_once(now)

                deadline += period
                if deadline <= now:
                    # The loop was descheduled for longer than a period. Skip
                    # the missed slots rather than sending a burst to catch up;
                    # telemetry is a sampled stream and stale samples are worse
                    # than absent ones.
                    deadline = now + period
        except asyncio.CancelledError:
            raise
        finally:
            self._running = False

    def send_once(self, now: float) -> bool:
        """Build and send one packet. Returns False when there was nothing to do."""
        sock = self._sock
        addr = self._addr
        if sock is None or addr is None or not self._key:
            self.stats.idle += 1
            return False

        sample = self._source.sample()
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        packet = TelemetryPacket.from_si(
            session_id=self._session_id,
            sequence=self._sequence,
            car_time_us=self._clock.monotonic_us(),
            state=sample.state,
            faults=sample.faults,
            flags=sample.flags,
            echo_client_time_us=self._echo.echo_client_time_us,
            echo_sequence=self._echo.echo_sequence,
            rpm_l=sample.rpm_l,
            rpm_r=sample.rpm_r,
            rpm_target_l=sample.rpm_target_l,
            rpm_target_r=sample.rpm_target_r,
            duty_l=sample.duty_l,
            duty_r=sample.duty_r,
            servo_us=sample.servo_us,
            steer_angle_deg=sample.steer_angle_deg,
            speed_mps=sample.speed_mps,
            v_max_mps=sample.v_max_mps,
            x_m=sample.x_m,
            y_m=sample.y_m,
            heading_rad=sample.heading_rad,
            distance_m=sample.distance_m,
            slip=sample.slip,
            pack_volts=sample.pack_volts,
            cpu_temp_c=sample.cpu_temp_c,
            throttled=sample.throttled,
            loop_p99_us=sample.loop_p99_us,
        )

        try:
            sock.sendto(packet.pack(self._key), addr)
        except (BlockingIOError, InterruptedError):
            # The socket buffer is momentarily full. Dropping this sample is
            # correct: the next one is 20 ms away and carries newer numbers.
            self.stats.errors += 1
            return False
        except OSError as exc:
            # ENETUNREACH / EHOSTUNREACH while the WiFi reassociates, or an ICMP
            # port-unreachable from an app that just quit. Neither is fatal to
            # the car, so neither may raise out of here.
            self.stats.errors += 1
            self.stats.last_error = str(exc)
            if self._error_log.allow(now):
                _log.warning(
                    "telemetry send failed",
                    error=str(exc),
                    dest=f"{addr[0]}:{addr[1]}",
                    suppressed=self._error_log.take_suppressed(),
                )
            return False

        self.stats.sent += 1
        return True

    # -- introspection ------------------------------------------------------

    @property
    def active(self) -> bool:
        return bool(self._key) and self._addr is not None

    @property
    def sequence(self) -> int:
        return self._sequence

    def __repr__(self) -> str:
        return (
            f"TelemetrySender(session={self._session_id}, sent={self.stats.sent}, "
            f"errors={self.stats.errors})"
        )


__all__ = [
    "TelemetrySender",
    "TelemetrySource",
    "TelemetryStats",
    "VehicleSample",
    "ControlEcho",
]
