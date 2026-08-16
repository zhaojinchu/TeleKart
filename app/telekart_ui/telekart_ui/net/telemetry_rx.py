"""Telemetry receive thread: decode, statistics, publish."""

from __future__ import annotations

import errno
import logging
import socket
import threading
import time
from dataclasses import dataclass
from typing import Callable

from telekart_protocol import ProtocolError, TelemetryPacket, UDP_TELEMETRY_PORT

# Not re-exported from the package root, so it comes from the module that owns it.
from telekart_protocol.constants import MAX_TELEMETRY_PACKET_LEN

from ..core.latest_box import LatestBox
from ..core.log import Throttle, get_logger

_log = get_logger(__name__)
_throttle = Throttle(5.0)

#: Comfortably larger than a telemetry packet, so an oversized datagram is
#: rejected on its length rather than truncated into something that might parse.
_RECV_BUFFER = 4 * MAX_TELEMETRY_PACKET_LEN

#: Statistics window. Long enough that a single late packet does not make the
#: rate readout jump, short enough that a real stall shows up while the driver
#: still has time to react.
_WINDOW_S = 1.0

#: An RTT outside this is a clock artefact (a resumed laptop, a car that
#: rebooted), not a measurement. Feeding it to the HUD would peg the gauge.
_MAX_PLAUSIBLE_RTT = 2.0


@dataclass(frozen=True, slots=True)
class TelemetrySample:
    """One decoded packet plus the app-side timing around it."""

    packet: TelemetryPacket
    recv_t: float  # perf_counter at recvfrom
    rtt: float  # seconds; 0.0 when unknown


@dataclass(frozen=True, slots=True)
class TelemetryStats:
    packets: int = 0
    lost: int = 0
    bad: int = 0
    reordered: int = 0
    foreign: int = 0
    rate_hz: float = 0.0
    loss: float = 0.0
    rtt: float = 0.0
    last_recv_t: float = 0.0
    bound_port: int = 0


class TelemetryRxThread(threading.Thread):
    """Blocking receive loop.

    Not paced: the packet rate is the car's business, and a paced poll would
    add up to a full period of latency to every packet for no benefit. The
    recv timeout exists only so the thread can notice a shutdown.
    """

    def __init__(
        self,
        box: LatestBox[TelemetrySample],
        *,
        port: int = UDP_TELEMETRY_PORT,
        rtt_lookup: Callable[[int], float | None] | None = None,
        name: str = "TelemetryRx",
    ) -> None:
        super().__init__(name=name, daemon=True)
        self._box = box
        self._requested_port = port
        # Cross-check path: if the car echoes a sequence but not a timestamp,
        # the TX thread's send-time ring still yields an RTT.
        self._rtt_lookup = rtt_lookup

        self._sock: socket.socket | None = None
        self._bound_port = 0
        self._shutdown = threading.Event()

        self._lock = threading.Lock()
        self._session_id = 0

        # Statistics. Only ever written by this thread; the lock protects the
        # coherent read that stats() performs from the supervisor thread.
        self._packets = 0
        self._lost = 0
        self._bad = 0
        self._reordered = 0
        self._foreign = 0
        self._last_seq = -1
        self._last_recv_t = 0.0
        self._rate_hz = 0.0
        self._loss = 0.0
        self._win_start = 0.0
        self._win_recv = 0
        self._win_expected = 0

        # Preallocated so a 50 Hz receive path never touches the allocator.
        self._rtt_last = 0.0

    # -- lifecycle ----------------------------------------------------------

    def bind(self) -> int:
        """Bind the UDP socket and return the port actually taken.

        Called before the handshake, because the app tells the car where to
        send. A port already in use is not fatal: falling back to an ephemeral
        port is exactly what lets a second app run on the same machine, and the
        car is told the real number either way.
        """
        if self._sock is not None:
            return self._bound_port
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self._requested_port))
        except OSError as exc:
            if exc.errno not in (errno.EADDRINUSE, errno.EACCES):
                sock.close()
                raise
            _log.warning(
                "telemetry port %d unavailable (%s); using an ephemeral port",
                self._requested_port,
                exc.strerror,
            )
            try:
                sock.bind(("0.0.0.0", 0))
            except OSError:
                sock.close()
                raise
        # A burst after a scheduling hiccup must not be dropped by the kernel;
        # 256 KiB is about five seconds of telemetry.
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256 * 1024)
        except OSError:
            pass
        sock.settimeout(0.25)
        self._sock = sock
        self._bound_port = sock.getsockname()[1]
        return self._bound_port

    @property
    def port(self) -> int:
        return self._bound_port

    def set_session(self, session_id: int) -> None:
        """Accept packets for this session. Resets per-session statistics."""
        with self._lock:
            self._session_id = session_id
            self._last_seq = -1
            self._win_start = 0.0
            self._win_recv = 0
            self._win_expected = 0

    def clear_session(self) -> None:
        with self._lock:
            self._session_id = 0
            self._rate_hz = 0.0
            self._loss = 0.0

    def stop(self) -> None:
        self._shutdown.set()

    def close(self) -> None:
        self.stop()
        sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()

    # -- loop ---------------------------------------------------------------

    def run(self) -> None:
        if self._sock is None:
            self.bind()
        sock = self._sock
        assert sock is not None
        buf = bytearray(_RECV_BUFFER)
        view = memoryview(buf)

        while not self._shutdown.is_set():
            try:
                nbytes, _addr = sock.recvfrom_into(view)
            except socket.timeout:
                continue
            except OSError as exc:
                if self._shutdown.is_set():
                    break
                # A transient ICMP-driven ECONNREFUSED on a connectionless
                # socket is normal when the car reboots. Never fatal.
                _throttle.log(_log, logging.WARNING, "recv", "telemetry recv failed: %s", exc)
                time.sleep(0.05)
                continue

            recv_t = time.perf_counter()
            with self._lock:
                session_id = self._session_id
            if not session_id:
                continue

            try:
                packet = TelemetryPacket.unpack(view[:nbytes])
            except ProtocolError as exc:
                # Never raise out of a decode path: a malformed or spoofed
                # datagram must cost a counter, not the link.
                self._bad += 1
                _throttle.log(_log, logging.DEBUG, "decode", "telemetry rejected: %s", exc)
                continue
            except Exception as exc:  # defensive: a decoder bug must not kill the link
                self._bad += 1
                _throttle.log(_log, logging.ERROR, "decode!", "telemetry decode error: %s", exc)
                continue

            if session_id and packet.session_id != session_id:
                self._foreign += 1
                continue

            rtt = self._measure_rtt(packet, recv_t)
            self._account(packet.sequence, recv_t)

            self._box.put(TelemetrySample(packet=packet, recv_t=recv_t, rtt=rtt))

        _log.info("telemetry receive thread exiting")

    # -- internals ----------------------------------------------------------

    def _measure_rtt(self, packet: TelemetryPacket, recv_t: float) -> float:
        """Round trip from the echoed send timestamp.

        The car echoes ``client_time_us`` verbatim, so this needs no agreement
        between the two clocks at all -- only that the app's own clock is
        monotonic, which perf_counter guarantees.
        """
        rtt = 0.0
        echo = packet.echo_client_time_us
        if echo:
            rtt = recv_t - echo / 1_000_000.0
        elif self._rtt_lookup is not None and packet.echo_sequence:
            sent = self._rtt_lookup(packet.echo_sequence)
            if sent is not None:
                rtt = recv_t - sent
        if rtt <= 0.0 or rtt > _MAX_PLAUSIBLE_RTT:
            return 0.0
        with self._lock:
            self._rtt_last = rtt
        return rtt

    def _account(self, sequence: int, recv_t: float) -> None:
        with self._lock:
            self._packets += 1
            self._last_recv_t = recv_t

            last = self._last_seq
            if last < 0:
                self._win_expected += 1
            elif sequence > last:
                gap = sequence - last - 1
                if gap > 0:
                    # A gap larger than a window is a session restart or a wrap,
                    # not loss; counting it would show 100 % loss forever.
                    if gap < 1000:
                        self._lost += gap
                        self._win_expected += gap
                    else:
                        self._win_start = 0.0
                self._win_expected += 1
            else:
                self._reordered += 1
            if sequence > last:
                self._last_seq = sequence
            self._win_recv += 1

            if self._win_start == 0.0:
                self._win_start = recv_t
                self._win_recv = 1
                self._win_expected = 1
                return
            elapsed = recv_t - self._win_start
            if elapsed >= _WINDOW_S:
                rate = self._win_recv / elapsed
                expected = self._win_expected if self._win_expected > 0 else 1
                loss = 1.0 - (self._win_recv / expected)
                loss = 0.0 if loss < 0.0 else 1.0 if loss > 1.0 else loss
                # Light smoothing: a single dropped packet in a 1 s window is
                # 2 %, and an unsmoothed readout flickering between 0 and 2 %
                # reads as a problem when it is not one.
                self._rate_hz = rate if self._rate_hz == 0.0 else 0.5 * self._rate_hz + 0.5 * rate
                self._loss = loss if self._loss == 0.0 else 0.7 * self._loss + 0.3 * loss
                self._win_start = recv_t
                self._win_recv = 0
                self._win_expected = 0

    def stats(self) -> TelemetryStats:
        """A coherent read of every counter. Called ~20 Hz by the supervisor.

        Only the latest RTT is kept. The previous station also reported p95 and
        min from a 256-entry ring, which meant sorting on every call for two
        numbers no HUD zone displayed.
        """
        now = time.perf_counter()
        with self._lock:
            rate = self._rate_hz
            # Decay the rate to zero when nothing is arriving; otherwise a dead
            # link keeps advertising the rate it had at the moment it died.
            if self._last_recv_t and now - self._last_recv_t > _WINDOW_S:
                rate = 0.0
            return TelemetryStats(
                packets=self._packets,
                lost=self._lost,
                bad=self._bad,
                reordered=self._reordered,
                foreign=self._foreign,
                rate_hz=rate,
                loss=self._loss,
                rtt=self._rtt_last,
                last_recv_t=self._last_recv_t,
                bound_port=self._bound_port,
            )
