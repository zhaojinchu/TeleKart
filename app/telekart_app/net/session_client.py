"""TCP/JSON session channel: handshake, token derivation, params, reconnect.

The session socket is also the *presence* signal. The firmware treats losing it
as an E-stop condition even if UDP control packets are still arriving from
somewhere, so this client's job is not only to carry messages but to be honest
about whether it is really connected -- which is why it pings rather than
trusting TCP to notice a peer that vanished.
"""

from __future__ import annotations

import enum
import errno
import hashlib
import hmac
import json
import math
import queue
import random
import selectors
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from telekart_protocol import (
    PROTO_VERSION,
    TCP_SESSION_PORT,
    UDP_CONTROL_PORT,
    TCP_VIDEO_PORT,
    ErrorCode,
    LineReader,
    Message,
    MsgType,
    SessionError,
    derive_udp_key,
    normalize_shared_key,
)
from telekart_protocol.session import set_params as make_set_params

from .. import __version__
from ..core.log import get_logger

_log = get_logger(__name__)

#: Handshake steps get their own, shorter deadline than the steady-state loop:
#: a car that accepts TCP but never sends its nonce is broken, and waiting the
#: full ping timeout to find that out doubles every reconnect attempt.
_HANDSHAKE_TIMEOUT = 5.0

_SELECT_TIMEOUT = 0.25


class SessionEventKind(enum.Enum):
    CONNECTING = "connecting"
    CONNECTED = "connected"
    DISCONNECTED = "disconnected"
    #: Terminal: the car said no in a way that retrying cannot fix (wrong
    #: passphrase, wrong protocol version). Hammering it would only fill its log.
    FAILED = "failed"
    ERROR = "error"
    PARAMS = "params"
    STATE = "state"
    ACK = "ack"
    CALIBRATION = "calibration"


@dataclass(frozen=True, slots=True)
class SessionInfo:
    """Everything the handshake established. Immutable; replaced on reconnect."""

    session_id: int
    session_token: bytes
    udp_key: bytes
    car_id: str
    fw_version: str
    caps: tuple[str, ...]
    control_port: int
    video_port: int
    host: str
    address: str
    connected_at: float
    #: The car issued the same session id we had before, so the car never
    #: dropped us -- only the socket did.
    resumed: bool = False

    def has(self, capability: str) -> bool:
        return capability in self.caps


@dataclass(frozen=True, slots=True)
class SessionEvent:
    kind: SessionEventKind
    detail: str = ""
    code: str = ""
    info: SessionInfo | None = None
    message: Message | None = None


@dataclass(slots=True)
class _Reconnect:
    """Backoff state, kept together so `reset` cannot forget half of it."""

    minimum: float
    maximum: float
    attempt: int = 0

    def next_delay(self) -> float:
        delay = min(self.maximum, self.minimum * (2.0**self.attempt))
        self.attempt += 1
        # Jitter so two apps (or an app and the sim) restarted together do not
        # retry in lockstep forever.
        return delay * random.uniform(0.8, 1.2)

    def reset(self) -> None:
        self.attempt = 0


class SessionClient(threading.Thread):
    """Owns the TCP session socket for its whole lifetime.

    Runs its own thread with a selector rather than blocking recv, so a queued
    ARM or E-stop goes out immediately instead of waiting for the next read
    timeout. Callers never block: `send` enqueues and wakes the loop through a
    socketpair.
    """

    def __init__(
        self,
        *,
        host: str,
        address: str,
        shared_key: str,
        driver: str,
        telemetry_port: int,
        port: int = TCP_SESSION_PORT,
        on_event: Callable[[SessionEvent], None] | None = None,
        connect_timeout: float = 4.0,
        min_delay: float = 0.5,
        max_delay: float = 8.0,
        ping_interval: float = 1.0,
        ping_timeout: float = 5.0,
        app_version: str = __version__,
        name: str = "Session",
    ) -> None:
        super().__init__(name=name, daemon=True)
        if not shared_key:
            raise ValueError("a shared key is required: the car will reject an empty auth")
        if not address:
            raise ValueError("session client needs a resolved address")
        if telemetry_port <= 0:
            raise ValueError("telemetry port must be bound before the handshake")

        self._host = host or address
        self._address = address
        self._port = port
        self._key = normalize_shared_key(shared_key)
        self._driver = driver or "driver"
        self._app_version = app_version
        self._telemetry_port = telemetry_port
        self._on_event = on_event
        self._connect_timeout = connect_timeout
        self._ping_interval = ping_interval
        self._ping_timeout = ping_timeout
        self._backoff = _Reconnect(min_delay, max_delay)

        self._shutdown = threading.Event()
        self._out: queue.Queue[bytes] = queue.Queue(maxsize=256)
        # Self-pipe so an enqueued message interrupts select() at once. A pure
        # timeout-driven loop would add up to a quarter second to every ARM.
        self._wake_r, self._wake_w = socket.socketpair()
        self._wake_r.setblocking(False)
        self._wake_w.setblocking(False)

        self._state_lock = threading.Lock()
        self._info: SessionInfo | None = None
        self._params: dict[str, Any] = {}
        self._params_generation = 0
        self._params_pending = 0
        self._last_error = ""
        self._last_error_code = ""
        self._last_rx = 0.0
        self._connected = False
        self._fatal = False

        self._next_id = 0
        self._id_lock = threading.Lock()
        self._pending: dict[int, Callable[[Message], None]] = {}
        self._prev_session_id = 0

    # -- public state -------------------------------------------------------

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def info(self) -> SessionInfo | None:
        with self._state_lock:
            return self._info

    @property
    def last_error(self) -> tuple[str, str]:
        with self._state_lock:
            return self._last_error_code, self._last_error

    @property
    def last_rx(self) -> float:
        with self._state_lock:
            return self._last_rx

    def params(self) -> Mapping[str, Any]:
        """A snapshot of the car's authoritative parameter set."""
        with self._state_lock:
            return dict(self._params)

    def params_generation(self) -> tuple[int, int]:
        with self._state_lock:
            return self._params_generation, self._params_pending

    # -- outbound -----------------------------------------------------------

    def send(self, message: Message) -> bool:
        """Queue a message. Never blocks, never raises on a dead link.

        Returns False when the queue is full, which only happens if the socket
        has been stuck for seconds; dropping is right, because everything here
        is either idempotent (ARM) or superseded by the next one (params).
        """
        try:
            payload = message.encode()
        except (SessionError, ValueError) as exc:
            # Encoding is a programming error (a NaN, an oversized payload),
            # so it is loud -- but it is raised at the call site, on the
            # caller's thread, never inside the session loop.
            raise ValueError(f"cannot encode {message.type.value}: {exc}") from exc
        try:
            self._out.put_nowait(payload)
        except queue.Full:
            _log.warning("session output queue full; dropping %s", message.type.value)
            return False
        self._wake()
        return True

    def _wake(self) -> None:
        try:
            self._wake_w.send(b"\x01")
        except OSError:
            pass

    def next_id(self) -> int:
        with self._id_lock:
            self._next_id += 1
            return self._next_id

    def request(
        self,
        msg_type: MsgType,
        data: dict[str, Any] | None = None,
        *,
        on_reply: Callable[[Message], None] | None = None,
    ) -> int:
        """Send a correlated request. Returns its id; the reply arrives as an event."""
        msg_id = self.next_id()
        if on_reply is not None:
            with self._state_lock:
                self._pending[msg_id] = on_reply
        self.send(Message(msg_type, msg_id, dict(data or {})))
        return msg_id

    # -- convenience commands ----------------------------------------------

    def arm(self) -> int:
        return self.request(MsgType.ARM)

    def disarm(self) -> int:
        return self.request(MsgType.DISARM)

    def send_estop(self) -> int:
        """The out-of-band E-stop. Reliable and ordered, unlike the UDP burst,
        and complementary to it rather than a replacement."""
        return self.request(MsgType.ESTOP)

    def clear_estop(self) -> int:
        return self.request(MsgType.CLEAR_ESTOP)

    def clear_faults(self) -> int:
        return self.request(MsgType.CLEAR_FAULTS)

    def reset_odom(self) -> int:
        return self.request(MsgType.RESET_ODOM)

    def get_params(self, group: str = "") -> int:
        return self.request(MsgType.GET_PARAMS, {"group": group} if group else None)

    def set_params(self, values: Mapping[str, Any]) -> int:
        """Push parameters. The reply, not this call, is the truth.

        Non-finite floats are rejected here rather than on the wire:
        ``Message.encode`` uses ``allow_nan=False``, so a NaN reaching this
        point takes the whole session down -- and a NaN in a parameter is a bug
        upstream that deserves to be loud.
        """
        clean: dict[str, Any] = {}
        for name, value in values.items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"parameter {name} is not finite: {value!r}")
            clean[name] = value
        if not clean:
            raise ValueError("set_params called with no values")
        msg_id = self.next_id()
        with self._state_lock:
            self._params_pending += 1
        self.send(make_set_params(msg_id, clean))
        return msg_id

    def calibrate(self, routine: str = "drive", *, on_ground: bool = False) -> int:
        return self.request(MsgType.CALIBRATE, {"routine": routine, "on_ground": on_ground})

    # -- lifecycle ----------------------------------------------------------

    def stop(self) -> None:
        self._shutdown.set()
        self._wake()

    def close(self) -> None:
        self.stop()
        for sock in (self._wake_r, self._wake_w):
            try:
                sock.close()
            except OSError:
                pass

    def run(self) -> None:
        while not self._shutdown.is_set():
            self._emit(SessionEvent(SessionEventKind.CONNECTING, detail=self._address))
            sock = None
            # One LineReader across handshake and steady state: the nonce and
            # the hello_ack can arrive in the same TCP segment as the first
            # STATE message, and a fresh reader would drop the tail of it.
            reader = LineReader()
            try:
                sock = self._connect()
                info = self._handshake(sock, reader)
            except _FatalSessionError as exc:
                with self._state_lock:
                    self._last_error = str(exc)
                    self._last_error_code = exc.code
                self._fatal = True
                self._emit(
                    SessionEvent(SessionEventKind.FAILED, detail=str(exc), code=exc.code)
                )
                _close(sock)
                return
            except (OSError, SessionError, ValueError) as exc:
                _close(sock)
                with self._state_lock:
                    self._last_error = str(exc)
                self._emit(SessionEvent(SessionEventKind.DISCONNECTED, detail=str(exc)))
                if self._sleep_backoff():
                    return
                continue

            self._backoff.reset()
            with self._state_lock:
                self._info = info
                self._connected = True
            self._emit(SessionEvent(SessionEventKind.CONNECTED, info=info))

            reason = ""
            try:
                reason = self._pump(sock, reader)
            except Exception as exc:  # the loop must survive any decode surprise
                reason = str(exc)
                _log.exception("session loop failed")
            finally:
                _close(sock)
                with self._state_lock:
                    self._connected = False
                    self._prev_session_id = info.session_id
                self._emit(SessionEvent(SessionEventKind.DISCONNECTED, detail=reason))

            if self._shutdown.is_set():
                return
            if self._sleep_backoff():
                return

    # -- connect and handshake ---------------------------------------------

    def _connect(self) -> socket.socket:
        sock = socket.create_connection(
            (self._address, self._port), timeout=self._connect_timeout
        )
        # Nagle would coalesce an ARM with whatever comes next and add up to
        # 40 ms to a command a human is waiting on.
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
        return sock

    def _handshake(self, sock: socket.socket, reader: LineReader) -> SessionInfo:
        """Nonce, auth, hello, hello_ack, key derivation.

        The nonce step is not pinned by ``telekart_protocol`` -- the package has
        no NONCE message type. Both ends adopt the convention in
        docs/protocol.md 5.5: the car sends ``ack`` with ``id: 0`` and a
        32-hex-character ``nonce`` field immediately on accept, and ``auth`` is
        the lowercase hex of the *full* HMAC-SHA256, not truncated.
        """
        sock.settimeout(_HANDSHAKE_TIMEOUT)

        nonce_msg = _read_message(sock, reader, MsgType.ACK, "nonce")
        if nonce_msg.type is MsgType.ERROR:
            raise _fatal_from_error(nonce_msg)
        nonce_hex = nonce_msg.data.get("nonce")
        if not isinstance(nonce_hex, str):
            raise SessionError("car did not send a nonce on accept")
        try:
            nonce = bytes.fromhex(nonce_hex)
        except ValueError as exc:
            raise SessionError(f"malformed nonce {nonce_hex!r}") from exc
        if len(nonce) < 8:
            raise SessionError(f"nonce is only {len(nonce)} bytes")

        auth = hmac.new(self._key, nonce, hashlib.sha256).hexdigest()
        hello_id = self.next_id()
        sock.sendall(
            Message(
                MsgType.HELLO,
                hello_id,
                {
                    "proto": PROTO_VERSION,
                    "app_version": self._app_version,
                    "driver": self._driver,
                    "auth": auth,
                    "telemetry_port": self._telemetry_port,
                },
            ).encode()
        )

        reply = _read_message(sock, reader, MsgType.HELLO_ACK, "hello_ack")
        if reply.type is MsgType.ERROR:
            raise _fatal_from_error(reply)

        data = reply.data
        token_hex = data.get("session_token")
        session_id = data.get("session_id")
        if not isinstance(token_hex, str) or not isinstance(session_id, int):
            raise SessionError("hello_ack is missing session_id or session_token")
        try:
            token = bytes.fromhex(token_hex)
        except ValueError as exc:
            raise SessionError("hello_ack session_token is not hex") from exc
        proto = data.get("proto")
        if isinstance(proto, int) and proto != PROTO_VERSION:
            raise _FatalSessionError(
                f"car speaks protocol {proto}, this build speaks {PROTO_VERSION}",
                ErrorCode.PROTOCOL_VERSION.value,
            )

        caps = data.get("caps")
        info = SessionInfo(
            session_id=session_id,
            session_token=token,
            udp_key=derive_udp_key(self._key, token),
            car_id=str(data.get("car_id", "")),
            fw_version=str(data.get("fw_version", "")),
            caps=tuple(str(c) for c in caps) if isinstance(caps, list) else (),
            control_port=_port_or(data.get("control_port"), UDP_CONTROL_PORT),
            video_port=_port_or(data.get("video_port"), TCP_VIDEO_PORT),
            host=self._host,
            address=self._address,
            connected_at=time.time(),
            resumed=session_id != 0 and session_id == self._prev_session_id,
        )
        if info.resumed:
            _log.info("resumed session %d on %s", info.session_id, info.car_id or self._host)
        with self._state_lock:
            self._last_rx = time.perf_counter()
        return info

    # -- steady state -------------------------------------------------------

    def _pump(self, sock: socket.socket, reader: LineReader) -> str:
        """Run until the socket dies or we are asked to stop. Returns the reason.

        The socket keeps a timeout rather than being switched to non-blocking:
        reads only happen after the selector says the socket is readable, so
        they never wait, while writes get a bounded block instead of a partial
        send nobody would notice.
        """
        sock.settimeout(_HANDSHAKE_TIMEOUT)
        selector = selectors.DefaultSelector()
        selector.register(sock, selectors.EVENT_READ)
        selector.register(self._wake_r, selectors.EVENT_READ)

        # Populate the tuning UI immediately; the car's answer is the only
        # value the app is allowed to display.
        self.get_params()
        next_ping = time.perf_counter() + self._ping_interval
        with self._state_lock:
            self._last_rx = time.perf_counter()

        try:
            while not self._shutdown.is_set():
                for key, _events in selector.select(_SELECT_TIMEOUT):
                    if key.fileobj is self._wake_r:
                        _drain(self._wake_r)
                        continue
                    try:
                        chunk = sock.recv(65536)
                    except BlockingIOError:
                        continue
                    except OSError as exc:
                        return f"recv failed: {exc}"
                    if not chunk:
                        return "car closed the session"
                    with self._state_lock:
                        self._last_rx = time.perf_counter()
                    try:
                        for message in reader.feed(chunk):
                            self._dispatch(message)
                    except SessionError as exc:
                        # Malformed session traffic is fatal to the session by
                        # design: LineReader discards the rest of the batch and
                        # there is no defined way to resynchronise a JSON stream.
                        return f"malformed session traffic: {exc}"

                if not self._flush(sock):
                    return "send failed"

                now = time.perf_counter()
                if now >= next_ping:
                    next_ping = now + self._ping_interval
                    self.send(Message(MsgType.PING, self.next_id(), {"t": int(now * 1e6)}))
                    if not self._flush(sock):
                        return "send failed"
                with self._state_lock:
                    silence = now - self._last_rx
                if silence > self._ping_timeout:
                    # TCP will happily hold a half-open connection open for
                    # minutes. Only traffic proves the car is still there, and
                    # a car that is not there means presence is lost.
                    return f"no response for {silence:.1f}s"
            return "stopped"
        finally:
            selector.close()

    def _flush(self, sock: socket.socket) -> bool:
        while True:
            try:
                payload = self._out.get_nowait()
            except queue.Empty:
                return True
            try:
                sock.sendall(payload)
            except OSError as exc:
                if isinstance(exc, TimeoutError) or exc.errno in (
                    errno.EAGAIN,
                    errno.EWOULDBLOCK,
                ):
                    # Seconds of stall on a channel this quiet means the car is
                    # gone; reconnecting is faster and more honest than waiting.
                    _log.warning("session send stalled; dropping the connection")
                else:
                    _log.info("session send failed: %s", exc)
                return False

    def _dispatch(self, message: Message) -> None:
        callback: Callable[[Message], None] | None = None
        if message.id:
            with self._state_lock:
                callback = self._pending.pop(message.id, None)

        if message.type is MsgType.PARAMS:
            values = message.data.get("values")
            if isinstance(values, dict):
                with self._state_lock:
                    self._params.update(values)
                    self._params_generation += 1
                    if self._params_pending > 0:
                        self._params_pending -= 1
            self._emit(SessionEvent(SessionEventKind.PARAMS, message=message))
        elif message.type is MsgType.ERROR:
            code = str(message.data.get("code", ""))
            detail = str(message.data.get("detail", ""))
            with self._state_lock:
                self._last_error = detail
                self._last_error_code = code
                if self._params_pending > 0:
                    self._params_pending -= 1
            _log.warning("car rejected a request: %s %s", code, detail)
            self._emit(
                SessionEvent(SessionEventKind.ERROR, detail=detail, code=code, message=message)
            )
        elif message.type is MsgType.STATE:
            self._emit(SessionEvent(SessionEventKind.STATE, message=message))
        elif message.type is MsgType.CALIBRATION_STATUS:
            self._emit(SessionEvent(SessionEventKind.CALIBRATION, message=message))
        elif message.type in (MsgType.ACK, MsgType.PONG):
            self._emit(SessionEvent(SessionEventKind.ACK, message=message))

        if callback is not None:
            try:
                callback(message)
            except Exception as exc:
                _log.error("session reply handler raised: %s", exc)

    def _emit(self, event: SessionEvent) -> None:
        handler = self._on_event
        if handler is None:
            return
        try:
            handler(event)
        except Exception as exc:
            # A listener that raises must not take the session down with it.
            _log.error("session event listener raised: %s", exc)

    def _sleep_backoff(self) -> bool:
        """Wait before retrying. Returns True if we were told to stop."""
        delay = self._backoff.next_delay()
        _log.info("reconnecting to %s in %.1fs", self._address, delay)
        return self._shutdown.wait(delay)


class _FatalSessionError(Exception):
    """A refusal that retrying cannot fix."""

    def __init__(self, detail: str, code: str) -> None:
        super().__init__(detail)
        self.code = code


def _fatal_from_error(message: Message) -> Exception:
    code = str(message.data.get("code", ""))
    detail = str(message.data.get("detail", "")) or code or "car refused the handshake"
    if code in (ErrorCode.AUTH_FAILED.value, ErrorCode.PROTOCOL_VERSION.value):
        return _FatalSessionError(detail, code)
    return SessionError(detail)


def _read_message(
    sock: socket.socket, reader: LineReader, expect: MsgType, what: str
) -> Message:
    """Read until a message of the expected type (or an ERROR) arrives.

    Anything else is skipped rather than treated as a failure: a car that
    volunteers a STATE line before its hello_ack is not misbehaving. Skipping
    is safe because every unsolicited message during the handshake is
    redundant -- vehicle state and faults arrive authoritatively in telemetry
    at 50 Hz, so a dropped STATE costs at most 20 ms of nothing.

    The LineReader is the caller's, so no *bytes* are lost either way: a
    partial line left in its buffer is picked up by the steady-state loop.
    """
    deadline = time.perf_counter() + _HANDSHAKE_TIMEOUT
    queued: list[Message] = []
    while True:
        for message in queued:
            if message.type is expect or message.type is MsgType.ERROR:
                return message
        queued.clear()
        if time.perf_counter() > deadline:
            raise SessionError(f"timed out waiting for {what}")
        chunk = sock.recv(65536)
        if not chunk:
            raise SessionError(f"car closed the connection before {what}")
        queued = reader.feed(chunk)


def _port_or(value: Any, fallback: int) -> int:
    return value if isinstance(value, int) and 0 < value < 65536 else fallback


def _drain(sock: socket.socket) -> None:
    try:
        while sock.recv(4096):
            pass
    except (BlockingIOError, OSError):
        pass


def _close(sock: socket.socket | None) -> None:
    if sock is None:
        return
    try:
        sock.close()
    except OSError:
        pass


def encode_preview(message: Message) -> str:
    """Human-readable form for the session log pane."""
    try:
        return message.encode().decode("utf-8").rstrip()
    except (SessionError, ValueError, UnicodeDecodeError):
        return json.dumps({"type": message.type.value, "id": message.id})
