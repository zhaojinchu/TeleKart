"""Control transmit thread: 100 Hz UDP, the sequence counter, the RTT ring,
and the E-stop path."""

from __future__ import annotations

import logging
import socket
import threading
import time
from dataclasses import dataclass
from operator import attrgetter
from typing import Any, Callable, Protocol

from telekart_protocol import CONTROL_RATE_HZ, ControlFlags, ControlPacket, UDP_CONTROL_PORT

from ..core.latest_box import LatestBox
from ..core.log import Throttle, get_logger
from ..core.paced_loop import PacedLoop
from ..model.snapshots import InputSnapshot

_log = get_logger(__name__)
_throttle = Throttle(5.0)

#: 1024 slots at 100 Hz is 10.2 s of send history -- comfortably longer than any
#: RTT worth measuring, and a power of two so the index is a mask, not a modulo.
_RTT_RING = 1024
_RTT_MASK = _RTT_RING - 1


@dataclass(frozen=True, slots=True)
class ControlCommand:
    """One shaped command from the input chain.

    The input thread constructs these and puts them in a ``LatestBox``; this
    thread is the only reader. Immutable because the two threads share it with
    no lock: publishing a reference to a frozen object is the entire handoff.
    """

    steering: float = 0.0  # -1..+1
    throttle: float = 0.0  # 0..1
    brake: float = 0.0  # 0..1
    flags: ControlFlags = ControlFlags.NONE
    #: perf_counter at which the axes were sampled. Optional -- leave it at 0.0
    #: and staleness is measured from the LatestBox generation instead, which is
    #: nearly as good and needs no cooperation.
    t_read: float = 0.0


class ControlCommandLike(Protocol):
    """What this thread actually requires of a command.

    Stated as a Protocol so the input workstream is free to carry extra fields
    of its own without either side importing the other's dataclass.
    """

    steering: float
    throttle: float
    brake: float
    flags: ControlFlags


#: Accessors are resolved once per command class and cached. The input chain
#: names its steering axis `steer` and may not carry `flags` at all; forcing a
#: rename across a workstream boundary would be a worse trade than one dict
#: lookup and an attrgetter per tick. `attrgetter` pulls all three axes in a
#: single C call, so the tolerance costs about 100 ns at 100 Hz.
_AXES: dict[type, "tuple[Any, Any]"] = {}


def _accessors(command: object) -> tuple[Any, Any]:
    cls = type(command)
    entry = _AXES.get(cls)
    if entry is None:
        steer = "steering" if hasattr(command, "steering") else "steer"
        axes = attrgetter(steer, "throttle", "brake")
        flags = attrgetter("flags") if hasattr(command, "flags") else None
        entry = (axes, flags)
        _AXES[cls] = entry
    return entry


@dataclass(frozen=True, slots=True)
class ControlTxStats:
    sent: int = 0
    errors: int = 0
    rate_hz: float = 0.0
    sequence: int = 0
    last_error: str = ""
    transmitting: bool = False
    estop_latched: bool = False
    overruns: int = 0


class ControlTxThread(threading.Thread):
    """Fixed-rate control transmitter.

    Paced independently of everything else in the app on purpose. The car
    failsafes on a 200 ms gap in this stream, so it must not be able to stall
    because a widget is repainting, a decoder is busy, or the session socket is
    blocked on a reconnect. A stalled UI costs a few milliseconds of stale axis
    values and nothing more -- which is the correct failure mode.
    """

    def __init__(
        self,
        command_box: LatestBox[ControlCommandLike],
        echo_box: LatestBox[InputSnapshot],
        *,
        rate_hz: int = CONTROL_RATE_HZ,
        burst: int = 10,
        burst_gap: float = 0.0,
        tcp_estop: Callable[[], None] | None = None,
        name: str = "ControlTx",
    ) -> None:
        super().__init__(name=name, daemon=True)
        if rate_hz <= 0:
            raise ValueError(f"control rate must be positive, got {rate_hz}")
        if burst < 1:
            raise ValueError(f"estop burst must be at least 1 packet, got {burst}")
        if burst_gap < 0.0:
            raise ValueError("estop burst gap must not be negative")

        self._commands = command_box
        self._echo = echo_box
        self._loop = PacedLoop(1.0 / rate_hz, name=name)
        self._burst = burst
        self._burst_gap = burst_gap
        self._tcp_estop = tcp_estop
        # Named `_shutdown`, not `_stop`: `threading.Thread._stop` is a real
        # private method that `join()` calls internally, and shadowing it with
        # an Event makes every join on this thread raise TypeError. The same
        # applies to the other Thread subclasses in this package.
        self._shutdown = threading.Event()

        # One lock covers the socket, the sequence counter and the RTT ring.
        # They must move together: two threads (this one and whoever calls
        # estop()) both allocate sequence numbers, and a duplicate sequence is
        # silently dropped by the car's replay guard.
        self._tx = threading.Lock()
        self._sock: socket.socket | None = None
        self._addr: tuple[str, int] | None = None
        self._session_id = 0
        self._key = b""
        self._sequence = 0

        self._seq_ring = [0] * _RTT_RING
        self._time_ring = [0.0] * _RTT_RING

        self._estop_latched = False
        self._arm_intent = False
        self._device = ""
        self._device_connected = False

        self._sent = 0
        self._errors = 0
        self._last_error = ""
        self._rate_hz = 0.0
        self._win_start = 0.0
        self._win_count = 0

        # Publishing an immutable snapshot per packet would be 100 short-lived
        # objects a second whose only consumer repaints at 60 Hz. Every other
        # packet is more resolution than the HUD can show.
        self._echo_divisor = max(1, rate_hz // 50)
        self._echo_phase = 0
        self._last_gen = -1
        self._cmd_seen_at = 0.0

    # -- configuration ------------------------------------------------------

    def set_target(
        self,
        address: str,
        session_id: int,
        key: bytes,
        *,
        port: int = UDP_CONTROL_PORT,
    ) -> None:
        """Point the stream at a car. Until this is called nothing is sent."""
        if not key:
            raise ValueError("a session key is required before transmitting")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connectionless, but a connected UDP socket lets the kernel cache the
        # route and surfaces ICMP errors as ECONNREFUSED, which is how we learn
        # the car's listener died without waiting for a telemetry timeout.
        try:
            sock.connect((address, port))
        except OSError as exc:
            sock.close()
            raise OSError(f"cannot reach control port {address}:{port}: {exc}") from exc
        with self._tx:
            old, self._sock = self._sock, sock
            self._addr = (address, port)
            self._session_id = session_id
            self._key = key
        if old is not None:
            old.close()
        _log.info("control stream -> %s:%d session %d", address, port, session_id)

    def clear_target(self) -> None:
        """Stop transmitting. The key is dropped, so nothing can go out with a
        session the car has forgotten."""
        with self._tx:
            sock, self._sock = self._sock, None
            self._addr = None
            self._session_id = 0
            self._key = b""
            self._rate_hz = 0.0
        if sock is not None:
            sock.close()

    def set_arm_intent(self, intent: bool) -> None:
        """Heartbeat-level "I still want to be armed".

        Deliberately not a claim that the car *is* armed -- the app never
        asserts that. It is the operator's continuing consent, and dropping it
        is one of the ways the car returns to SAFE.
        """
        self._arm_intent = intent

    def set_input_device(self, name: str, connected: bool) -> None:
        self._device = name
        self._device_connected = connected

    # -- e-stop -------------------------------------------------------------

    def estop(self) -> None:
        """Latch E-stop and burst it out now.

        Three independent mechanisms, because this is the one message that must
        not be lost:

        1. A burst of datagrams sent from the calling thread immediately, not
           at the next tick -- 10 ms of latency is 30 cm at this car's speed.
        2. A latch: every subsequent packet carries ESTOP until it is cleared,
           so a WiFi fade that swallows the whole burst is covered by the next
           hundred packets rather than by hope.
        3. The out-of-band TCP path, which is reliable and ordered and whose
           mere disconnection the car already treats as an E-stop condition.
        """
        self._estop_latched = True
        if self._tcp_estop is not None:
            try:
                self._tcp_estop()
            except Exception as exc:  # never let the TCP side block the UDP burst
                _log.error("out-of-band TCP estop failed: %s", exc)
        if self._burst_gap > 0.0:
            # Spacing the burst spreads it across WiFi retry windows, but the
            # caller is usually the GUI thread and must not sleep. Hand it off.
            threading.Thread(
                target=self._send_burst, name="EstopBurst", daemon=True
            ).start()
        else:
            self._send_burst()

    def clear_estop(self) -> None:
        """Stop asserting E-stop. Does not clear the car's latch -- that needs
        an explicit CLEAR_ESTOP over the session channel, by design."""
        self._estop_latched = False

    @property
    def estop_latched(self) -> bool:
        return self._estop_latched

    def _send_burst(self) -> None:
        now = time.perf_counter()
        for i in range(self._burst):
            # Full stop, not the driver's current pedal position: an E-stop is
            # not a request to keep whatever throttle was applied.
            self._transmit(0.0, 0.0, 1.0, ControlFlags.ESTOP, now)
            if self._burst_gap > 0.0 and i + 1 < self._burst:
                time.sleep(self._burst_gap)
                now = time.perf_counter()

    # -- RTT ----------------------------------------------------------------

    def rtt_for(self, sequence: int) -> float | None:
        """Send time for an echoed sequence, or None if it has aged out."""
        idx = sequence & _RTT_MASK
        with self._tx:
            if self._seq_ring[idx] != sequence:
                return None
            return self._time_ring[idx]

    # -- loop ---------------------------------------------------------------

    def stop(self) -> None:
        self._shutdown.set()

    def run(self) -> None:
        self._loop.start()
        while not self._shutdown.is_set():
            now = time.perf_counter()
            command, generation = self._commands.peek_versioned()
            if generation != self._last_gen:
                self._last_gen = generation
                self._cmd_seen_at = now

            if command is None:
                steering = throttle = 0.0
                brake = 0.0
                flags = ControlFlags.NONE
                age = 0.0
            else:
                axes, get_flags = _accessors(command)
                steering, throttle, brake = axes(command)
                flags = get_flags(command) if get_flags is not None else ControlFlags.NONE
                t_read = getattr(command, "t_read", 0.0)
                age = now - t_read if t_read > 0.0 else now - self._cmd_seen_at

            if flags & ControlFlags.ESTOP and not self._estop_latched:
                # An E-stop asserted by the input layer -- a button on the
                # wheel -- gets the identical treatment to the one in the UI:
                # latch, burst, and out-of-band TCP. Handling it here as well
                # means a wheel button works even if whoever wired it only set
                # the flag and never called LinkManager.estop().
                self.estop()

            if self._arm_intent:
                flags |= ControlFlags.ARM_INTENT
            if self._estop_latched:
                # The latch overrides the pedals for as long as it is set.
                steering = 0.0
                throttle = 0.0
                brake = 1.0
                flags |= ControlFlags.ESTOP

            sent, sequence = self._transmit(steering, throttle, brake, flags, now)
            self._account(now, sent)

            self._echo_phase += 1
            if self._echo_phase >= self._echo_divisor:
                self._echo_phase = 0
                self._publish_echo(steering, throttle, brake, flags, sequence, now, age, sent)

            if self._loop.sleep_until_next(self._shutdown):
                break

        _log.info("control transmit thread exiting")

    # -- internals ----------------------------------------------------------

    def _transmit(
        self,
        steering: float,
        throttle: float,
        brake: float,
        flags: ControlFlags,
        now: float,
    ) -> tuple[bool, int]:
        """Build, sign and send one packet. Returns (sent, sequence).

        Never raises. A send failure on a link that is already misbehaving must
        cost a counter, not the thread -- if this thread dies the car sees a
        silent stream and failsafes, which is safe but undiagnosable.
        """
        with self._tx:
            sock = self._sock
            key = self._key
            session_id = self._session_id
            if sock is None or not key:
                return False, self._sequence
            self._sequence += 1
            sequence = self._sequence
            idx = sequence & _RTT_MASK
            self._seq_ring[idx] = sequence
            self._time_ring[idx] = now
            # The remaining allocation on this path is the packet buffer itself,
            # which telekart_protocol owns and which is frozen. Everything else
            # here is preallocated or an int.
            packet = ControlPacket.from_normalized(
                session_id,
                sequence,
                int(now * 1_000_000.0),
                steering,
                throttle,
                brake,
                flags,
            )
            try:
                sock.send(packet.pack(key))
            except OSError as exc:
                self._errors += 1
                self._last_error = exc.strerror or str(exc)
                _throttle.log(_log, logging.WARNING, "send", "control send failed: %s", exc)
                return False, sequence
            self._sent += 1
            return True, sequence

    def _account(self, now: float, sent: bool) -> None:
        if not sent:
            return
        self._win_count += 1
        if self._win_start == 0.0:
            self._win_start = now
            return
        elapsed = now - self._win_start
        if elapsed >= 1.0:
            self._rate_hz = self._win_count / elapsed
            self._win_start = now
            self._win_count = 0

    def _publish_echo(
        self,
        steering: float,
        throttle: float,
        brake: float,
        flags: ControlFlags,
        sequence: int,
        now: float,
        age: float,
        sent: bool,
    ) -> None:
        # Report the value after quantisation, because that is what the car
        # will actually act on. A HUD showing 0.4137 next to a car that got
        # 0.414 is a HUD that will be trusted once too often.
        self._echo.put(
            InputSnapshot(
                steering=round(steering * 1000.0) / 1000.0,
                throttle=round(throttle * 1000.0) / 1000.0,
                brake=round(brake * 1000.0) / 1000.0,
                flags=flags,
                sequence=sequence,
                sent_at=now,
                command_age=age,
                rate_hz=self._rate_hz,
                transmitting=sent,
                estop_latched=self._estop_latched,
                arm_intent=self._arm_intent,
                device=self._device,
                device_connected=self._device_connected,
            )
        )

    def stats(self) -> ControlTxStats:
        with self._tx:
            transmitting = self._sock is not None and bool(self._key)
            return ControlTxStats(
                sent=self._sent,
                errors=self._errors,
                rate_hz=self._rate_hz,
                sequence=self._sequence,
                last_error=self._last_error,
                transmitting=transmitting,
                estop_latched=self._estop_latched,
                overruns=self._loop.overruns,
            )

    def close(self) -> None:
        self.stop()
        self.clear_target()
