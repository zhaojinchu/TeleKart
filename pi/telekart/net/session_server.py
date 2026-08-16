"""The TCP/JSON control plane, and the car's only notion of "an operator".

Everything that must not be lost lives here: the handshake, parameter push,
arm/disarm, calibration control. The rate is low enough that JSON costs nothing
and its debuggability is worth a great deal during bring-up -- ``nc
telekart.local 4212`` shows you a readable conversation.

The connection is also a **presence signal**. If it drops, the car has lost its
operator, and that is an E-stop condition regardless of whether valid UDP
control packets are still arriving from somewhere. Two consequences that are
easy to miss:

* TCP keepalives are turned on and turned *down*. A laptop that walks out of
  WiFi range never sends a FIN, so without keepalives the socket stays open for
  hours and the presence signal never fires. Eleven seconds is the detection
  budget here, which is slower than the 200 ms control timeout but covers the
  case the control timeout does not: somebody else's stale process still
  spraying valid-looking UDP.
* Exactly one operator at a time. A second connection is refused with
  ``ErrorCode.BUSY`` rather than being allowed to take over, because "which
  laptop is driving?" is not a question anyone should have to answer while the
  car is moving.

The handshake follows docs/protocol.md §5.5, which resolves an ambiguity the
frozen protocol package leaves open: ``session.hello()`` refers to a nonce that
has no constructor in the package. The car sends it unsolicited on accept as
``ack(0, nonce=<32 hex chars>)``.
"""

from __future__ import annotations

import asyncio
import hashlib
import hmac
import math
import os
import socket
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol

from telekart_protocol import (
    ErrorCode,
    LineReader,
    Message,
    MsgType,
    PROTO_VERSION,
    SessionError,
    derive_udp_key,
    make_session_token,
    normalize_shared_key,
)
from telekart_protocol.constants import (
    SESSION_TTL_S,
    TCP_SESSION_PORT,
    TCP_VIDEO_PORT,
    UDP_CONTROL_PORT,
)
from telekart_protocol.params import GROUPS, PARAMS, ParamError
from telekart_protocol.session import ack, error, hello_ack, params as params_msg, state as state_msg

from ..config import VehicleConfig
from ..constants import FIRMWARE_VERSION
from ..log import get_logger
from ..util.clock import Clock

_log = get_logger(__name__)

#: 16 bytes of challenge, hex-encoded on the wire. Big enough that a replayed
#: `auth` from a captured handshake is useless; the point is freshness, not
#: cryptographic drama.
NONCE_LEN = 16

#: An operator who connects and then says nothing holds the slot for this long.
#: Bounded on purpose: the reservation happens at accept time so two laptops
#: cannot both complete a handshake, and an unbounded reservation would let a
#: half-open connection lock the car out.
HANDSHAKE_TIMEOUT_S = 5.0

#: How often the read loop wakes to re-evaluate the session TTL.
IDLE_POLL_S = 15.0

#: A write that has not drained in this long means the peer is gone in a way TCP
#: has not admitted yet. Tearing the session down is the correct response and,
#: because it triggers the presence E-stop, the safe one.
WRITE_TIMEOUT_S = 2.0

#: Connections accepted while one is already being handled. Small: this is a
#: single-operator vehicle, and the only legitimate second connection is a
#: reconnect racing the old socket's teardown.
MAX_CONNECTIONS = 4

_KEEPALIVE_IDLE_S = 5
_KEEPALIVE_INTERVAL_S = 2
_KEEPALIVE_COUNT = 3

DEFAULT_CAPS: tuple[str, ...] = ("video", "calibrate", "params")


# --------------------------------------------------------------------------
# Service seam
# --------------------------------------------------------------------------


class NotAllowedInState(RuntimeError):
    """The vehicle refuses the request in its current state."""


class CalibrationBusy(RuntimeError):
    """A calibration run is already in progress."""


class CalibrationUnavailable(RuntimeError):
    """This build has no calibration routine wired in."""


@dataclass(frozen=True, slots=True)
class CalibrationProgress:
    """One progress report from a calibration run.

    ``done`` is the last message of a run; it carries either ``result`` or
    ``error`` and nothing further will follow it.
    """

    phase: str
    progress: float = 0.0
    done: bool = False
    error: str = ""
    result: dict[str, Any] | None = None


#: Invoked by whatever is running the calibration, from **any** thread. The
#: server hops it onto the event loop itself, so implementers need not care.
ProgressSink = Callable[[CalibrationProgress], None]


class VehicleService(Protocol):
    """What the session channel is allowed to ask of the vehicle.

    Deliberately narrow, and deliberately ``async``: the safety state machine
    and the drive controller live on the control thread, so every mutating call
    here is a cross-thread round trip that the implementation completes on the
    control thread and resolves back on the event loop. Making that explicit in
    the signature stops anyone reaching into the control loop from a socket
    handler.

    The two synchronous methods read published snapshots, never live state.
    """

    def snapshot(self) -> tuple[int, int]:
        """``(VehicleState value, Fault bits)`` as last published."""
        ...

    def is_armed(self) -> bool: ...

    def parameters(self) -> dict[str, Any]:
        """The car's authoritative parameter set."""
        ...

    async def arm(self) -> tuple[bool, str]: ...

    async def disarm(self) -> None: ...

    async def estop(self, reason: str) -> None: ...

    async def clear_estop(self) -> tuple[bool, str]: ...

    async def clear_faults(self) -> None: ...

    async def reset_odometry(self) -> None: ...

    async def apply_parameters(
        self, values: dict[str, Any], *, persist: bool = False
    ) -> dict[str, Any]:
        """Apply and return the authoritative read-back of the requested keys.

        ``persist`` writes the per-vehicle overlay so the change survives a
        restart. Off by default: a tuning session drags a slider a hundred
        times, and a hundred SD-card writes is how cards die.

        Raises :class:`telekart_protocol.params.ParamError` on a bad value and
        :class:`NotAllowedInState` when the vehicle refuses.
        """
        ...

    async def start_calibration(
        self, routine: str, on_ground: bool, progress: ProgressSink
    ) -> None:
        """Raises :class:`CalibrationBusy`, :class:`CalibrationUnavailable`, or
        ``ValueError`` for an unknown routine."""
        ...

    async def cancel_calibration(self) -> None: ...


# --------------------------------------------------------------------------
# Session record
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class SessionInfo:
    """Handed to the app when a handshake completes, so it can point the UDP
    links at the operator and tear them down again when the link drops."""

    session_id: int
    token: bytes
    udp_key: bytes
    peer_host: str
    telemetry_addr: tuple[str, int]
    driver: str
    app_version: str
    opened_at: float

    def describe(self) -> str:
        return (
            f"session {self.session_id} driver={self.driver or '?'} "
            f"app={self.app_version or '?'} peer={self.peer_host}"
        )


class _HandshakeRejected(Exception):
    """Internal: the handshake failed and the reason has already been sent."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


# eq=False so connections keep identity semantics: they live in a set, and two
# peers are never "equal" no matter how alike their fields look.
@dataclass(slots=True, eq=False)
class _Connection:
    """One TCP peer, its framing state, and its write serialisation."""

    reader: asyncio.StreamReader
    writer: asyncio.StreamWriter
    peer_host: str
    peer_port: int
    lines: LineReader = field(default_factory=LineReader)
    pending: deque[Message] = field(default_factory=deque)
    write_lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    last_message_at: float = 0.0
    info: SessionInfo | None = None
    closing: bool = False

    async def read_message(self) -> Message | None:
        """Next complete message, or None when the peer closed.

        Raises :class:`SessionError` on malformed framing, which the caller
        turns into a torn-down session -- resynchronising a JSON stream that has
        already lied to you once is not worth the code.
        """
        while not self.pending:
            chunk = await self.reader.read(4096)
            if not chunk:
                return None
            # No await between feed() and extend(), so a cancellation at the
            # read above can never lose a decoded message.
            self.pending.extend(self.lines.feed(chunk))
        return self.pending.popleft()

    async def send(self, message: Message) -> bool:
        if self.closing:
            return False
        try:
            data = message.encode()
        except (SessionError, ValueError) as exc:
            # allow_nan=False means a stray NaN raises here rather than on the
            # app's parser. Losing one message beats losing the session.
            _log.error("could not encode session message",
                       type=message.type.value, error=str(exc))
            return False
        async with self.write_lock:
            if self.closing:
                return False
            try:
                self.writer.write(data)
                await asyncio.wait_for(self.writer.drain(), WRITE_TIMEOUT_S)
            except (asyncio.TimeoutError, OSError, ConnectionError) as exc:
                self.closing = True
                _log.warning("session write failed", peer=self.peer_host, error=str(exc))
                return False
        return True

    def close(self) -> None:
        self.closing = True
        try:
            self.writer.close()
        except OSError:
            pass

    @property
    def label(self) -> str:
        return f"{self.peer_host}:{self.peer_port}"


# --------------------------------------------------------------------------
# Server
# --------------------------------------------------------------------------


class SessionServer:
    """Accepts one operator, authenticates it, and serves the control plane."""

    def __init__(
        self,
        *,
        config: VehicleConfig,
        clock: Clock,
        service: VehicleService,
        on_open: Callable[[SessionInfo], None],
        on_close: Callable[[SessionInfo, str], None],
        host: str = "0.0.0.0",
        port: int = TCP_SESSION_PORT,
        control_port: int = UDP_CONTROL_PORT,
        video_port: int = TCP_VIDEO_PORT,
        caps: tuple[str, ...] = DEFAULT_CAPS,
        activity: Callable[[], float] | None = None,
        fw_version: str = FIRMWARE_VERSION,
    ) -> None:
        self._config = config
        self._clock = clock
        self._service = service
        self._on_open = on_open
        self._on_close = on_close
        self._host = host
        self._port = port
        self._control_port = control_port
        self._video_port = video_port
        self._caps = list(caps)
        self._fw_version = fw_version
        #: Monotonic time of the last UDP control traffic, so a session that is
        #: quiet on TCP but actively driving does not age out.
        self._activity = activity

        # Derived once: hashing a passphrase per connection would be pointless,
        # and keeping the raw string around longer than necessary is worse.
        self._shared_key = normalize_shared_key(config.shared_key)

        self._server: asyncio.AbstractServer | None = None
        self._connections: set[_Connection] = set()
        #: Reserved at accept time, not at handshake completion, so two laptops
        #: cannot both get through the handshake.
        self._operator: _Connection | None = None
        self._background: set[asyncio.Task[Any]] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._last_published: tuple[int, int] = (-1, -1)

        self.sessions_opened = 0
        self.sessions_rejected = 0

    # -- lifecycle ----------------------------------------------------------

    async def start(self) -> None:
        if self._server is not None:
            return
        self._loop = asyncio.get_running_loop()
        self._server = await asyncio.start_server(
            self._handle_client, self._host, self._port, backlog=8, reuse_address=True
        )
        _log.info("session server listening", host=self._host, port=self._port)

    async def stop(self) -> None:
        server, self._server = self._server, None
        if server is not None:
            server.close()
            try:
                await server.wait_closed()
            except (OSError, asyncio.CancelledError):
                pass
        for conn in list(self._connections):
            conn.close()
        for task in list(self._background):
            task.cancel()
        self._background.clear()
        _log.info("session server stopped", opened=self.sessions_opened)

    # -- unsolicited push ---------------------------------------------------

    def publish_state(self, vehicle_state: int, faults: int, detail: str = "") -> None:
        """Push a STATE message when the vehicle's state or faults change.

        Edge triggered: the caller may invoke this every telemetry tick, and
        only an actual change puts a line on the wire. Silent when there is no
        operator, which is the whole point of the presence rule.
        """
        current = (int(vehicle_state), int(faults))
        if current == self._last_published:
            return
        self._last_published = current
        conn = self._operator
        if conn is None or conn.info is None:
            return
        self._spawn(conn.send(state_msg(current[0], current[1], detail)))

    def _spawn(self, coro: Any) -> None:
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        task = loop.create_task(coro)
        # Keep a strong reference: asyncio only holds a weak one, and a
        # garbage-collected task is a message that silently never went out.
        self._background.add(task)
        task.add_done_callback(self._background.discard)

    # -- connection handling ------------------------------------------------

    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername") or ("unknown", 0)
        conn = _Connection(reader, writer, str(peer[0]), int(peer[1]))
        conn.last_message_at = self._clock.monotonic()
        self._connections.add(conn)
        _configure_socket(writer)

        reason = "closed"
        reserved = False
        try:
            if len(self._connections) > MAX_CONNECTIONS:
                self.sessions_rejected += 1
                await conn.send(error(0, ErrorCode.BUSY, "too many connections"))
                return
            if self._operator is not None:
                self.sessions_rejected += 1
                _log.warning(
                    "refusing a second operator",
                    peer=conn.label,
                    holder=self._operator.label,
                )
                await conn.send(
                    error(0, ErrorCode.BUSY, "another operator is already connected")
                )
                return

            self._operator = conn
            reserved = True

            nonce = os.urandom(NONCE_LEN)
            if not await conn.send(ack(0, nonce=nonce.hex())):
                reason = "nonce send failed"
                return

            try:
                info = await asyncio.wait_for(
                    self._handshake(conn, nonce), HANDSHAKE_TIMEOUT_S
                )
            except asyncio.TimeoutError:
                self.sessions_rejected += 1
                reason = "handshake timed out"
                await conn.send(error(0, ErrorCode.BAD_REQUEST, "handshake timed out"))
                return
            except _HandshakeRejected as exc:
                self.sessions_rejected += 1
                reason = exc.reason
                return

            conn.info = info
            self.sessions_opened += 1
            _log.info("operator connected", detail=info.describe())
            self._on_open(info)
            # Force the next publish_state to emit, so a freshly connected app
            # learns the state without waiting for it to change.
            self._last_published = (-1, -1)

            reason = await self._serve(conn)

        except (ConnectionError, OSError) as exc:
            reason = f"socket error: {exc}"
        except asyncio.CancelledError:
            reason = "server shutting down"
            raise
        except Exception as exc:  # noqa: BLE001 - one bad session must not stop the server
            reason = f"internal error: {exc!r}"
            _log.exception("session handler failed", peer=conn.label)
        finally:
            self._connections.discard(conn)
            if reserved and self._operator is conn:
                self._operator = None
            conn.close()
            info = conn.info
            if info is not None:
                _log.warning("operator link lost", detail=info.describe(), reason=reason)
                # The presence rule. The app is told nothing here because there
                # is no longer anyone to tell; the vehicle side is what matters.
                self._on_close(info, reason)
            else:
                _log.debug("connection closed before handshake",
                           peer=conn.label, reason=reason)
            try:
                await writer.wait_closed()
            except (OSError, ConnectionError, asyncio.CancelledError):
                pass

    async def _handshake(self, conn: _Connection, nonce: bytes) -> SessionInfo:
        try:
            msg = await conn.read_message()
        except SessionError as exc:
            await conn.send(error(0, ErrorCode.BAD_REQUEST, str(exc)))
            raise _HandshakeRejected(f"malformed handshake: {exc}") from exc
        if msg is None:
            raise _HandshakeRejected("peer closed before HELLO")
        if msg.type is not MsgType.HELLO:
            await conn.send(
                error(msg.id, ErrorCode.BAD_REQUEST, f"expected hello, got {msg.type.value}")
            )
            raise _HandshakeRejected(f"first message was {msg.type.value}")

        proto = msg.data.get("proto")
        if proto != PROTO_VERSION:
            # Rejected outright rather than negotiated: a silently misparsed
            # control packet is a runaway car.
            await conn.send(
                error(
                    msg.id,
                    ErrorCode.PROTOCOL_VERSION,
                    f"car speaks protocol {PROTO_VERSION}, app offered {proto!r}",
                )
            )
            raise _HandshakeRejected(f"protocol mismatch: {proto!r}")

        try:
            auth = msg.require("auth", str)
            app_version = msg.require("app_version", str)
            driver = msg.require("driver", str)
            telemetry_port = _require_port(msg, "telemetry_port")
        except SessionError as exc:
            await conn.send(error(msg.id, ErrorCode.BAD_REQUEST, str(exc)))
            raise _HandshakeRejected(str(exc)) from exc

        expected = hmac.new(self._shared_key, nonce, hashlib.sha256).hexdigest()
        if not hmac.compare_digest(expected, auth.strip().lower()):
            await conn.send(
                error(msg.id, ErrorCode.AUTH_FAILED, "shared key does not match")
            )
            raise _HandshakeRejected("authentication failed")

        token = make_session_token()
        session_id = _new_session_id()
        info = SessionInfo(
            session_id=session_id,
            token=token,
            udp_key=derive_udp_key(self._shared_key, token),
            peer_host=conn.peer_host,
            telemetry_addr=(conn.peer_host, telemetry_port),
            driver=driver[:64],
            app_version=app_version[:32],
            opened_at=self._clock.monotonic(),
        )

        sent = await conn.send(
            hello_ack(
                msg_id=msg.id,
                car_id=self._config.car_id,
                fw_version=self._fw_version,
                session_id=session_id,
                session_token=token.hex(),
                caps=self._caps,
                video_port=self._video_port,
                control_port=self._control_port,
            )
        )
        if not sent:
            raise _HandshakeRejected("hello_ack could not be sent")
        conn.last_message_at = self._clock.monotonic()
        return info

    async def _serve(self, conn: _Connection) -> str:
        while True:
            try:
                msg = await asyncio.wait_for(conn.read_message(), IDLE_POLL_S)
            except asyncio.TimeoutError:
                if self._idle_expired(conn):
                    await conn.send(
                        error(0, ErrorCode.BAD_REQUEST, "session idle timeout")
                    )
                    return "idle timeout"
                continue
            except SessionError as exc:
                await conn.send(error(0, ErrorCode.BAD_REQUEST, str(exc)))
                return f"malformed message: {exc}"

            if msg is None:
                return "peer closed the connection"

            conn.last_message_at = self._clock.monotonic()
            if conn.closing:
                return "write failed"
            await self._dispatch(conn, msg)

    def _idle_expired(self, conn: _Connection) -> bool:
        latest = conn.last_message_at
        if self._activity is not None:
            latest = max(latest, self._activity())
        return (self._clock.monotonic() - latest) > SESSION_TTL_S

    # -- message dispatch ---------------------------------------------------

    async def _dispatch(self, conn: _Connection, msg: Message) -> None:
        kind = msg.type
        service = self._service

        if kind is MsgType.PING:
            raw = msg.data.get("t", 0)
            # json.loads accepts NaN and Infinity; Message.encode does not.
            # Echoing an integer or nothing keeps a hostile ping from being able
            # to break the reply path.
            await conn.send(
                Message(MsgType.PONG, msg.id, {"t": raw if _is_plain_int(raw) else 0})
            )
            return

        if kind is MsgType.ARM:
            accepted, why = await service.arm()
            if accepted:
                await conn.send(ack(msg.id, state=service.snapshot()[0]))
            else:
                await conn.send(error(msg.id, ErrorCode.NOT_ALLOWED_IN_STATE, why))
            return

        if kind is MsgType.DISARM:
            await service.disarm()
            await conn.send(ack(msg.id, state=service.snapshot()[0]))
            return

        if kind is MsgType.ESTOP:
            detail = msg.data.get("reason")
            await service.estop(detail if isinstance(detail, str) else "operator estop")
            await conn.send(ack(msg.id, state=service.snapshot()[0]))
            return

        if kind is MsgType.CLEAR_ESTOP:
            cleared, why = await service.clear_estop()
            if cleared:
                await conn.send(ack(msg.id, state=service.snapshot()[0]))
            else:
                await conn.send(error(msg.id, ErrorCode.NOT_ALLOWED_IN_STATE, why))
            return

        if kind is MsgType.CLEAR_FAULTS:
            await service.clear_faults()
            state, faults = service.snapshot()
            await conn.send(ack(msg.id, state=state, faults=faults))
            return

        if kind is MsgType.RESET_ODOM:
            await service.reset_odometry()
            await conn.send(ack(msg.id))
            return

        if kind is MsgType.GET_PARAMS:
            await self._handle_get_params(conn, msg)
            return

        if kind is MsgType.SET_PARAMS:
            await self._handle_set_params(conn, msg)
            return

        if kind is MsgType.CALIBRATE:
            await self._handle_calibrate(conn, msg)
            return

        if kind in (MsgType.PONG, MsgType.ACK):
            # Replies to something we never sent. Harmless; ignore rather than
            # scolding a well-meaning client.
            return

        await conn.send(
            error(msg.id, ErrorCode.BAD_REQUEST, f"{kind.value} is not an app-to-car message")
        )

    async def _handle_get_params(self, conn: _Connection, msg: Message) -> None:
        wanted = msg.data.get("group")
        values = self._service.parameters()
        if wanted is not None:
            if not isinstance(wanted, str) or wanted not in GROUPS:
                await conn.send(
                    error(msg.id, ErrorCode.BAD_REQUEST,
                          f"unknown group {wanted!r}; expected one of {list(GROUPS)}")
                )
                return
            values = {
                name: value
                for name, value in values.items()
                if PARAMS[name].group == wanted
            }
        await conn.send(params_msg(msg.id, values))

    async def _handle_set_params(self, conn: _Connection, msg: Message) -> None:
        try:
            values = msg.require("values", dict)
        except SessionError as exc:
            await conn.send(error(msg.id, ErrorCode.BAD_REQUEST, str(exc)))
            return
        if not values:
            await conn.send(params_msg(msg.id, {}, applied=True))
            return

        unknown = sorted(name for name in values if name not in PARAMS)
        if unknown:
            await conn.send(
                error(msg.id, ErrorCode.UNKNOWN_PARAM, ", ".join(unknown))
            )
            return

        # NaN passes every range check ever written -- `nan < minimum` and
        # `nan > maximum` are both False -- and json.loads produces one happily.
        # Catch it here, before it reaches the registry's validator and settles
        # into the config as a plausible-looking number.
        non_finite = sorted(
            name
            for name, value in values.items()
            if isinstance(value, float) and not math.isfinite(value)
        )
        if non_finite:
            await conn.send(
                error(msg.id, ErrorCode.PARAM_OUT_OF_RANGE,
                      f"not a finite number: {', '.join(non_finite)}")
            )
            return

        locked = VehicleConfig.requires_disarm(values.keys())
        if locked and self._service.is_armed():
            await conn.send(
                error(msg.id, ErrorCode.NOT_ALLOWED_IN_STATE,
                      f"disarm first to change: {', '.join(sorted(locked))}")
            )
            return

        # Opt-in persistence. The app pushes every slider movement live and only
        # asks for a write when the operator commits, because the alternative is
        # a hundred SD-card writes per tuning session.
        persist = msg.data.get("persist") is True
        try:
            applied = await self._service.apply_parameters(dict(values), persist=persist)
        except ParamError as exc:
            await conn.send(error(msg.id, ErrorCode.PARAM_OUT_OF_RANGE, str(exc)))
            return
        except NotAllowedInState as exc:
            await conn.send(error(msg.id, ErrorCode.NOT_ALLOWED_IN_STATE, str(exc)))
            return

        # The authoritative echo. The app renders this, never what it typed.
        await conn.send(params_msg(msg.id, applied, applied=True))
        if persist:
            _log.info("parameters persisted to the local overlay", count=len(applied))

    async def _handle_calibrate(self, conn: _Connection, msg: Message) -> None:
        if msg.data.get("cancel") is True:
            await self._service.cancel_calibration()
            await conn.send(ack(msg.id, cancelled=True))
            return

        routine = msg.data.get("routine", "drive")
        on_ground = msg.data.get("on_ground", False)
        if not isinstance(routine, str):
            await conn.send(error(msg.id, ErrorCode.BAD_REQUEST, "routine must be a string"))
            return
        if not isinstance(on_ground, bool):
            await conn.send(error(msg.id, ErrorCode.BAD_REQUEST, "on_ground must be a boolean"))
            return
        if self._service.is_armed():
            await conn.send(
                error(msg.id, ErrorCode.NOT_ALLOWED_IN_STATE,
                      "disarm before starting a calibration run")
            )
            return

        sink = self._make_progress_sink(conn, msg.id)
        try:
            await self._service.start_calibration(routine, on_ground, sink)
        except CalibrationBusy as exc:
            await conn.send(error(msg.id, ErrorCode.BUSY, str(exc) or "already calibrating"))
            return
        except CalibrationUnavailable as exc:
            await conn.send(error(msg.id, ErrorCode.INTERNAL, str(exc)))
            return
        except NotAllowedInState as exc:
            await conn.send(error(msg.id, ErrorCode.NOT_ALLOWED_IN_STATE, str(exc)))
            return
        except ValueError as exc:
            await conn.send(error(msg.id, ErrorCode.BAD_REQUEST, str(exc)))
            return

        await conn.send(ack(msg.id, routine=routine, on_ground=on_ground))

    def _make_progress_sink(self, conn: _Connection, msg_id: int) -> ProgressSink:
        """A callable the calibration runner may invoke from the control thread.

        ``call_soon_threadsafe`` is the only asyncio API that is safe to call
        from another thread, and it is the whole reason this indirection exists:
        writing to a StreamWriter from the control thread would corrupt the
        stream and, worse, could block a 10 ms deadline on a TCP send.
        """
        loop = self._loop

        def sink(progress: CalibrationProgress) -> None:
            if loop is None or loop.is_closed():
                return
            try:
                loop.call_soon_threadsafe(self._emit_progress, conn, msg_id, progress)
            except RuntimeError:
                # The loop shut down between the check and the call.
                pass

        return sink

    def _emit_progress(
        self, conn: _Connection, msg_id: int, progress: CalibrationProgress
    ) -> None:
        payload: dict[str, Any] = {
            "id": msg_id,
            "phase": progress.phase,
            "progress": round(_finite(progress.progress), 4),
            "done": progress.done,
        }
        if progress.error:
            payload["error"] = progress.error
        if progress.result is not None:
            payload["result"] = _json_safe(progress.result)
        self._spawn(conn.send(Message(MsgType.CALIBRATION_STATUS, msg_id, payload)))

    # -- introspection ------------------------------------------------------

    @property
    def operator_connected(self) -> bool:
        return self._operator is not None and self._operator.info is not None

    @property
    def session(self) -> SessionInfo | None:
        conn = self._operator
        return conn.info if conn is not None else None

    def __repr__(self) -> str:
        return (
            f"SessionServer(port={self._port}, operator="
            f"{'yes' if self.operator_connected else 'no'}, "
            f"opened={self.sessions_opened})"
        )


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def _new_session_id() -> int:
    """A non-zero u32. Zero is reserved for 'no session', and a control packet
    carrying zero must never match a live session by accident."""
    while True:
        value = int.from_bytes(os.urandom(4), "little")
        if value:
            return value


def _require_port(msg: Message, key: str) -> int:
    value = msg.data.get(key)
    # bool is an int in Python, and `telemetry_port: true` is not a port.
    if isinstance(value, bool) or not isinstance(value, int):
        raise SessionError(f"{msg.type.value}: field {key!r} must be an integer port")
    if not 1 <= value <= 65535:
        raise SessionError(f"{msg.type.value}: field {key!r}={value} is not a valid port")
    return value


def _is_plain_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _finite(value: float) -> float:
    return value if isinstance(value, (int, float)) and math.isfinite(value) else 0.0


def _json_safe(payload: dict[str, Any]) -> dict[str, Any]:
    """Strip anything ``json.dumps(allow_nan=False)`` would refuse.

    A calibration that divided by zero somewhere must still be able to report
    that it finished; it must not take the operator's link down on the way out.
    """
    clean: dict[str, Any] = {}
    for key, value in payload.items():
        clean[str(key)] = _json_safe_value(value)
    return clean


def _json_safe_value(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else 0.0
    if isinstance(value, dict):
        return _json_safe(value)
    if isinstance(value, (list, tuple)):
        return [_json_safe_value(item) for item in value]
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


def _configure_socket(writer: asyncio.StreamWriter) -> None:
    """TCP_NODELAY plus aggressive keepalives.

    Nagle would coalesce a 60-byte ACK with whatever comes next and add up to
    40 ms to every round trip on the control plane. The keepalives are the more
    important half: they are what turns "the laptop left the building" into a
    closed socket, and therefore into the presence E-stop, in about eleven
    seconds instead of two hours.
    """
    sock = writer.get_extra_info("socket")
    if sock is None:
        return
    options: tuple[tuple[int, int, int], ...] = (
        (socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),
        (socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1),
    )
    for level, option, value in options:
        try:
            sock.setsockopt(level, option, value)
        except OSError:
            pass
    for name, value in (
        ("TCP_KEEPIDLE", _KEEPALIVE_IDLE_S),
        ("TCP_KEEPINTVL", _KEEPALIVE_INTERVAL_S),
        ("TCP_KEEPCNT", _KEEPALIVE_COUNT),
    ):
        option = getattr(socket, name, None)
        if option is None:  # not Linux
            continue
        try:
            sock.setsockopt(socket.IPPROTO_TCP, option, value)
        except OSError:
            pass


__all__ = [
    "SessionServer",
    "SessionInfo",
    "VehicleService",
    "CalibrationProgress",
    "ProgressSink",
    "NotAllowedInState",
    "CalibrationBusy",
    "CalibrationUnavailable",
    "NONCE_LEN",
    "DEFAULT_CAPS",
]
