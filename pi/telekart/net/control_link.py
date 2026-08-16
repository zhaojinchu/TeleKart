"""The UDP endpoint the operator's commands arrive on.

Three filters stand between a datagram and the H-bridge, in increasing order of
cost, because the cheap ones are what keep broadcast noise off the CPU:

1. **Length and magic** -- one ``peek_session_id`` call.
2. **Source address pinning** -- the host is taken from the TCP session, not
   from the datagram, and the port is pinned from the first accepted packet.
3. **Strictly increasing sequence** -- a packet whose sequence is not greater
   than the last accepted one is dropped. On a 100 Hz stream where only the
   newest command matters, a reordered packet *is* a stale packet.

There used to be a fourth: a truncated HMAC over a per-session key, sitting
between 2 and 3. It was removed along with the rest of the shared-key layer.
The consequence is worth being plain about -- filters 1-3 stop *accidents*, not
*impersonation*. Any process that knows the session id and sends from the pinned
host can now drive this car, including a copy of the app that you thought had
exited.

Nothing in this module raises. A decode failure increments a counter, updates a
rate-limited log line, and returns. The receive path is one of the two places in
the firmware where an exception would be a runaway car, so it does not have one.
"""

from __future__ import annotations

import asyncio
import socket
from dataclasses import dataclass
from typing import Callable

from telekart_protocol import ControlPacket, ProtocolError, peek_session_id
from telekart_protocol.constants import MAX_CONTROL_PACKET_LEN, UDP_CONTROL_PORT

from ..log import RateLimiter, get_logger
from ..util.clock import Clock

_log = get_logger(__name__)

#: Interval between "we are dropping packets" log lines. Fast enough to notice a
#: problem, slow enough that a misdirected stream cannot write 100 lines a second to
#: an SD card that the control loop is sharing.
_DROP_LOG_INTERVAL_S = 5.0

#: Receive buffer. Sized for a burst of stale datagrams after a WiFi stall
#: rather than for throughput -- 40-byte packets at 100 Hz is 4 kB/s.
_SO_RCVBUF = 64 * 1024

#: ``on_command(packet, received_at)``. Called on the asyncio thread for every
#: packet that survives all four filters.
CommandCallback = Callable[[ControlPacket, float], None]


@dataclass(slots=True)
class LinkStats:
    """Counters for one session, plus the last rejection reason.

    Plain mutable ints rather than a metrics library: incrementing an attribute
    is one bytecode, and this runs on every datagram.
    """

    received: int = 0
    accepted: int = 0
    #: Wrong length or wrong magic -- almost certainly not ours at all.
    malformed: int = 0
    #: Well-formed, but arrived when no session was open.
    no_session: int = 0
    #: Carries a session id we did not issue.
    wrong_session: int = 0
    #: Came from a host or port that is not the operator's.
    wrong_source: int = 0
    #: A malformed packet, or a protocol version this build does not speak.
    rejected: int = 0
    #: Sequence not greater than the last accepted one.
    replayed: int = 0
    #: Most recent rejection detail, for the diagnostics banner. Never formatted
    #: on the hot path -- it is the exception's own message, kept by reference.
    last_error: str = ""

    def reset(self) -> None:
        self.received = 0
        self.accepted = 0
        self.malformed = 0
        self.no_session = 0
        self.wrong_session = 0
        self.wrong_source = 0
        self.rejected = 0
        self.replayed = 0
        self.last_error = ""

    @property
    def dropped(self) -> int:
        return (
            self.malformed
            + self.no_session
            + self.wrong_session
            + self.wrong_source
            + self.rejected
            + self.replayed
        )

    def describe(self) -> str:
        return (
            f"rx={self.received} ok={self.accepted} drop={self.dropped} "
            f"(bad={self.malformed} nosess={self.no_session} "
            f"wrongsess={self.wrong_session} wrongsrc={self.wrong_source} "
            f"bad={self.rejected} replay={self.replayed})"
        )


class ControlLink(asyncio.DatagramProtocol):
    """Receives and de-duplicates the 100 Hz command stream."""

    def __init__(
        self,
        *,
        clock: Clock,
        on_command: CommandCallback,
        host: str = "0.0.0.0",
        port: int = UDP_CONTROL_PORT,
    ) -> None:
        self._clock = clock
        self._on_command = on_command
        self._host = host
        self._port = port

        self._transport: asyncio.DatagramTransport | None = None
        self._session_id = 0
        self._peer_host = ""
        #: Learned from the first accepted datagram rather than from the TCP
        #: connection: the app's UDP source port is not its TCP source port, and
        #: pinning it afterwards closes the "second process on the operator's
        #: own machine" case that host pinning alone leaves open.
        self._peer_port = 0
        self._last_sequence = -1
        self._last_accept_time = 0.0

        self.stats = LinkStats()
        #: Lifetime totals, so a session reset does not erase the history that
        #: explains why the previous session ended.
        self.total_received = 0
        self.total_accepted = 0

        # Echoed verbatim in every telemetry packet so the app can compute true
        # round-trip latency without the two clocks having to agree. Read from
        # the telemetry sender, which runs on this same thread -- no sync.
        self.echo_client_time_us = 0
        self.echo_sequence = 0

        self._drop_log = RateLimiter(_DROP_LOG_INTERVAL_S)

    # -- lifecycle ----------------------------------------------------------

    async def start(self) -> None:
        """Bind the socket. Raises at startup, which is the right time to."""
        if self._transport is not None:
            return
        loop = asyncio.get_running_loop()
        transport, _protocol = await loop.create_datagram_endpoint(
            lambda: self,
            local_addr=(self._host, self._port),
            family=socket.AF_INET,
            reuse_port=False,
        )
        self._transport = transport  # type: ignore[assignment]
        sock = transport.get_extra_info("socket")
        if sock is not None:
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, _SO_RCVBUF)
            except OSError as exc:
                _log.debug("could not enlarge the control receive buffer", error=str(exc))
        _log.info("control link listening", host=self._host, port=self._port)

    async def stop(self) -> None:
        transport, self._transport = self._transport, None
        if transport is not None:
            transport.close()
        self.close_session()
        _log.info("control link closed", accepted=self.total_accepted)

    # -- session ------------------------------------------------------------

    def open_session(self, session_id: int, peer_host: str) -> None:
        """Accept packets for ``session_id`` from ``peer_host`` only.

        Called from the session server once the TCP handshake has completed.
        """
        self._session_id = session_id
        self._peer_host = peer_host
        self._peer_port = 0
        # A fresh session starts a fresh sequence space. Keeping the old high
        # water mark would silently discard the new operator's first thousand
        # packets after a reconnect.
        self._last_sequence = -1
        self._last_accept_time = 0.0
        self.stats.reset()
        self.echo_client_time_us = 0
        self.echo_sequence = 0
        _log.info("control session opened", session_id=session_id, peer=peer_host)

    def close_session(self) -> None:
        if self._session_id:
            _log.info(
                "control session closed",
                session_id=self._session_id,
                stats=self.stats.describe(),
            )
        self._session_id = 0
        self._peer_host = ""
        self._peer_port = 0
        self._last_sequence = -1

    # -- asyncio.DatagramProtocol -------------------------------------------

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self._transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
        stats = self.stats
        stats.received += 1
        self.total_received += 1

        if len(data) > MAX_CONTROL_PACKET_LEN:
            stats.malformed += 1
            return

        # Cheapest possible discriminator: length plus magic, no HMAC. A stray
        # broadcast from an unrelated service dies here for the cost of one
        # struct unpack.
        session_id = peek_session_id(data)
        if session_id is None:
            stats.malformed += 1
            return

        if not self._session_id:
            stats.no_session += 1
            self._maybe_log_drops("control packet with no session open")
            return

        if session_id != self._session_id:
            stats.wrong_session += 1
            self._maybe_log_drops("control packet for an unknown session")
            return

        host, port = addr[0], addr[1]
        if host != self._peer_host or (self._peer_port and port != self._peer_port):
            stats.wrong_source += 1
            self._maybe_log_drops("control packet from an unexpected source")
            return

        try:
            packet = ControlPacket.unpack(data)
        except ProtocolError as exc:
            # A malformed or wrong-version packet. Counted and dropped: raising
            # here would take the receive path down and, with it, the car.
            stats.rejected += 1
            stats.last_error = str(exc)
            self._maybe_log_drops("control packet rejected")
            return
        except Exception as exc:  # noqa: BLE001 - the decode path never raises out
            stats.rejected += 1
            stats.last_error = repr(exc)
            self._maybe_log_drops("control packet decode failed")
            return

        # Strictly increasing, no window. SEQUENCE_REPLAY_WINDOW is 0 by design.
        if packet.sequence <= self._last_sequence:
            stats.replayed += 1
            return

        self._last_sequence = packet.sequence
        if not self._peer_port:
            self._peer_port = port
            _log.info("control stream source pinned", host=host, port=port)

        now = self._clock.monotonic()
        self._last_accept_time = now
        stats.accepted += 1
        self.total_accepted += 1
        self.echo_client_time_us = packet.client_time_us
        self.echo_sequence = packet.sequence

        self._on_command(packet, now)

    def error_received(self, exc: Exception) -> None:
        # ICMP port-unreachable from a telemetry destination that went away
        # surfaces here on Linux. Informational only; UDP has no connection to
        # lose and the next datagram may well arrive.
        _log.debug("control socket error", error=str(exc))

    def connection_lost(self, exc: Exception | None) -> None:
        if exc is not None:
            _log.warning("control socket lost", error=str(exc))
        self._transport = None

    # -- introspection ------------------------------------------------------

    @property
    def session_id(self) -> int:
        return self._session_id

    @property
    def active(self) -> bool:
        return bool(self._session_id) and self._transport is not None

    @property
    def last_accept_time(self) -> float:
        """Monotonic time of the last accepted packet; 0.0 if there has been none."""
        return self._last_accept_time

    @property
    def last_sequence(self) -> int:
        return self._last_sequence

    def _maybe_log_drops(self, message: str) -> None:
        if not self._drop_log.allow(self._clock.monotonic()):
            return
        _log.warning(
            message,
            suppressed=self._drop_log.take_suppressed(),
            stats=self.stats.describe(),
            detail=self.stats.last_error,
        )

    def __repr__(self) -> str:
        return (
            f"ControlLink(port={self._port}, session={self._session_id}, "
            f"{self.stats.describe()})"
        )


__all__ = ["ControlLink", "LinkStats", "CommandCallback"]
