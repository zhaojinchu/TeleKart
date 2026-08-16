"""The TCP frame server.

The whole design is one idea: **never let a slow link turn into a growing
backlog of stale frames.** A live view that is four seconds behind is worse
than no view at all, because the operator cannot tell it is behind.

Three mechanisms enforce that, and they are deliberately layered:

* a two-frame bounded queue per client that drops the *oldest whole frame*
  and marks the next one DROPPED_BEFORE, so the receiver knows corruption is
  coming instead of silently rendering garbage;
* TCP_NODELAY plus a modest SO_SNDBUF, because the kernel's send buffer is a
  queue we have no way to drop from -- a large one converts a two-second WiFi
  stall into two seconds of video that arrives late and in order;
* at most two clients, evicting the slowest, so one wedged viewer cannot make
  the encoder's callback the slowest thing in the process.

Combined with the half-second GOP the encoder is configured for, the worst
case after any loss is a bounded 500 ms of corruption, not an indefinite
freeze waiting for a keyframe that will never be requested.
"""

from __future__ import annotations

import asyncio
import collections
import logging
import socket
import time
from dataclasses import dataclass

from .camera import EncodedFrame
from .config import VideoConfig
from .framing import FrameSequencer, header_bytes, is_sendable

_log = logging.getLogger(__name__)

#: Per-offer decay on the drop score used to pick an eviction victim. At 30 fps
#: this is roughly a 1.6 s memory: a client that dropped frames a minute ago and
#: has been healthy since must not lose its slot to one that connected recently.
_DROP_DECAY = 0.98


@dataclass(frozen=True, slots=True)
class ClientStats:
    peer: str
    connected_s: float
    frames_sent: int
    frames_dropped: int
    bytes_sent: int
    queued: int


@dataclass(frozen=True, slots=True)
class ServerStats:
    clients: tuple[ClientStats, ...]
    frames_in: int
    frames_out: int
    frames_dropped: int
    frames_rejected: int
    bytes_out: int
    accepted: int
    evicted: int
    listening: bool


class _ClientSession:
    """One connected viewer, with its own bounded queue and its own drop state."""

    __slots__ = (
        "peer",
        "connected_at",
        "frames_sent",
        "frames_dropped",
        "bytes_sent",
        "drop_score",
        "_reader",
        "_writer",
        "_config",
        "_queue",
        "_wake",
        "_dropped_pending",
        "_closing",
        "_close_reason",
        "_seen_keyframe",
        "_started",
        "_clock",
    )

    def __init__(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
        config: VideoConfig,
        peer: str,
    ) -> None:
        self.peer = peer
        self.connected_at = time.monotonic()
        self.frames_sent = 0
        self.frames_dropped = 0
        self.bytes_sent = 0
        self.drop_score = 0.0
        self._reader = reader
        self._writer = writer
        self._config = config
        self._queue: collections.deque[tuple[int, EncodedFrame]] = collections.deque()
        self._wake = asyncio.Event()
        self._dropped_pending = False
        self._closing = False
        self._close_reason = ""
        self._seen_keyframe = not config.start_on_keyframe
        self._started = False
        self._clock = time.monotonic

    # -- producer side (event loop thread, called once per capture) ---------

    def offer(self, sequence: int, frame: EncodedFrame) -> None:
        if self._closing:
            return
        self.drop_score *= _DROP_DECAY
        if not self._seen_keyframe:
            if not frame.keyframe:
                # Starting mid-GOP hands the decoder a P-frame with no
                # reference and produces a smear until the next IDR. With
                # repeat=True the parameter sets ride along with that IDR, so
                # waiting for one costs at most half a second and gives a clean
                # first picture. Not counted as a drop: nothing was lost, the
                # client had not started yet.
                return
            self._seen_keyframe = True

        if len(self._queue) >= self._config.queue_depth:
            # Drop the OLDEST whole frame. Dropping the newest would keep the
            # stalest picture, and dropping part of a frame would desynchronise
            # the receiver's length-prefixed reader.
            self._queue.popleft()
            self.frames_dropped += 1
            self.drop_score += 1.0
            if not self._started and self._config.start_on_keyframe:
                # Nothing has reached this client yet, so the IDR that just
                # fell out was its only decodable entry point. Re-arm the gate
                # rather than opening the stream on a P-frame with no
                # reference, which decodes to a smear instead of a picture.
                self._queue.clear()
                self._seen_keyframe = frame.keyframe
                if not frame.keyframe:
                    return
            else:
                # The flag lives on the session, not on the queued item, so it
                # survives the dropped frame itself being evicted and lands on
                # whichever frame is actually written next.
                self._dropped_pending = True
        self._queue.append((sequence, frame))
        self._wake.set()

    # -- consumer side -----------------------------------------------------

    async def run(self) -> None:
        """Serve this client until it disconnects, stalls, or is evicted."""
        reader_task = asyncio.create_task(self._drain_incoming())
        try:
            await self._write_loop()
        finally:
            reader_task.cancel()
            try:
                await reader_task
            except asyncio.CancelledError:
                pass
            except Exception:
                _log.debug("client reader raised during teardown", exc_info=True)
            await self._hard_close()

    async def _write_loop(self) -> None:
        while not self._closing:
            if not self._queue:
                self._wake.clear()
                await self._wake.wait()
                continue
            sequence, frame = self._queue.popleft()
            dropped_before = self._dropped_pending
            self._dropped_pending = False

            header = header_bytes(
                sequence,
                frame.pts_us,
                len(frame.data),
                codec=frame.codec,
                keyframe=frame.keyframe,
                dropped_before=dropped_before,
            )
            try:
                # writelines lets the payload stay a single shared object
                # across clients; asyncio joins the pieces into one write, so
                # this is still one segment on the wire with Nagle disabled.
                self._writer.writelines((header, frame.data))
                # Marked before the drain, not after: the frame is already in
                # the transport, so the entry-point re-arm above must stop.
                self._started = True
                await asyncio.wait_for(
                    self._writer.drain(), timeout=self._config.write_timeout_s
                )
            except asyncio.TimeoutError:
                # Must stay ahead of the OSError clause: TimeoutError is an
                # OSError subclass. The socket has not accepted a byte for
                # seconds; waiting longer only builds a larger pile of video
                # nobody will watch.
                self._close_reason = "write stalled"
                return
            except OSError as exc:
                self._close_reason = f"write failed: {exc}"
                return
            except RuntimeError as exc:
                # Transport already closing underneath us.
                self._close_reason = f"transport closed: {exc}"
                return
            self.frames_sent += 1
            self.bytes_sent += len(header) + len(frame.data)

    async def _drain_incoming(self) -> None:
        """Read and discard whatever the client sends.

        The video channel is one-directional, but the socket must still be
        read: it is how a clean FIN is noticed promptly instead of after the
        next write attempt, and it stops a client that babbles from filling
        the receive buffer.
        """
        try:
            while True:
                data = await self._reader.read(4096)
                if not data:
                    break
        except (ConnectionError, OSError):
            pass
        finally:
            if not self._close_reason:
                self._close_reason = "client disconnected"
            self.close_soon(self._close_reason)

    # -- teardown ----------------------------------------------------------

    def close_soon(self, reason: str) -> None:
        """Ask the write loop to finish. Safe to call more than once."""
        if self._closing:
            return
        self._closing = True
        if not self._close_reason:
            self._close_reason = reason
        self._queue.clear()
        self._wake.set()

    async def _hard_close(self) -> None:
        self._closing = True
        try:
            if not self._writer.is_closing():
                self._writer.close()
            await asyncio.wait_for(self._writer.wait_closed(), timeout=1.0)
        except (asyncio.TimeoutError, ConnectionError, OSError):
            pass
        except Exception:
            _log.debug("client close raised", exc_info=True)

    @property
    def close_reason(self) -> str:
        return self._close_reason or "closed"

    def stats(self) -> ClientStats:
        return ClientStats(
            peer=self.peer,
            connected_s=self._clock() - self.connected_at,
            frames_sent=self.frames_sent,
            frames_dropped=self.frames_dropped,
            bytes_sent=self.bytes_sent,
            queued=len(self._queue),
        )


def _tune_socket(sock: socket.socket, send_buffer_bytes: int) -> None:
    """Nagle off, send buffer small, keepalive on. None of this is optional.

    Nagle would coalesce the 24-byte header with the start of the payload and
    add up to 40 ms waiting for an ACK. The send buffer is capped because it is
    the one queue in the path this process cannot drop from: whatever fits in
    it *will* be delivered, however stale it has become by then. Linux doubles
    the requested size for bookkeeping, so the effective depth is about twice
    what is asked for here.
    """
    try:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except OSError as exc:
        _log.warning("TCP_NODELAY failed: %s", exc)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, send_buffer_bytes)
    except OSError as exc:
        _log.warning("SO_SNDBUF failed: %s", exc)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        # A laptop that walks out of WiFi range never sends a FIN. Without
        # keepalive the slot stays occupied until the default two-hour idle
        # timer expires.
        for name, value in (("TCP_KEEPIDLE", 5), ("TCP_KEEPINTVL", 2), ("TCP_KEEPCNT", 3)):
            option = getattr(socket, name, None)
            if option is not None:
                sock.setsockopt(socket.IPPROTO_TCP, option, value)
    except OSError as exc:
        _log.warning("keepalive setup failed: %s", exc)


class VideoServer:
    """Fans one encoded stream out to a bounded number of TCP clients."""

    def __init__(self, config: VideoConfig) -> None:
        self._config = config
        self._sequencer = FrameSequencer()
        self._clients: list[_ClientSession] = []
        self._server: asyncio.AbstractServer | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._closing = False
        self.frames_in = 0
        self.frames_out = 0
        self.frames_dropped = 0
        self.frames_rejected = 0
        self.bytes_out = 0
        self.accepted = 0
        self.evicted = 0

    @property
    def client_count(self) -> int:
        return len(self._clients)

    @property
    def listening(self) -> bool:
        return self._server is not None

    async def start(self) -> None:
        if self._server is not None:
            return
        self._loop = asyncio.get_running_loop()
        self._closing = False
        self._server = await asyncio.start_server(
            self._handle_client,
            host=self._config.bind,
            port=self._config.port,
            # One pending connection is enough for two viewers, and a short
            # backlog means a client that cannot be served is refused quickly
            # rather than sitting in a queue believing it is connected.
            backlog=4,
            reuse_address=True,
        )
        _log.info("video server listening on %s:%d", self._config.bind, self._config.port)

    async def stop(self) -> None:
        self._closing = True
        server, self._server = self._server, None
        for client in list(self._clients):
            client.close_soon("server shutting down")
        if server is not None:
            server.close()
            try:
                await server.wait_closed()
            except Exception:
                _log.debug("server close raised", exc_info=True)
        # Give the write loops a tick to unwind before the loop goes away.
        deadline = time.monotonic() + 2.0
        while self._clients and time.monotonic() < deadline:
            await asyncio.sleep(0.02)
        self._loop = None
        _log.info("video server stopped")

    # -- frame ingress -----------------------------------------------------

    def submit_frame(self, frame: EncodedFrame) -> None:
        """Hand a captured frame to every client. Callable from any thread.

        This runs on libcamera's encoder thread, so it does the minimum
        possible and never raises: the fan-out itself happens on the event
        loop, where the per-client queues live and need no locking.
        """
        loop = self._loop
        if loop is None or self._closing:
            return
        try:
            loop.call_soon_threadsafe(self._dispatch, frame)
        except RuntimeError:
            # The loop closed between the check above and now. A dropped frame
            # during shutdown is not worth an exception on the camera thread.
            pass

    def _dispatch(self, frame: EncodedFrame) -> None:
        self.frames_in += 1
        if not is_sendable(len(frame.data)):
            self.frames_rejected += 1
            return
        # Sequenced once per capture, not once per client, so a gap in what a
        # client received counts exactly the frames its own link lost.
        sequence = self._sequencer.assign()
        for client in self._clients:
            client.offer(sequence, frame)

    # -- connections -------------------------------------------------------

    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer_info = writer.get_extra_info("peername")
        peer = f"{peer_info[0]}:{peer_info[1]}" if peer_info else "?"
        if self._closing:
            writer.close()
            return

        sock = writer.get_extra_info("socket")
        if sock is not None:
            _tune_socket(sock, self._config.send_buffer_bytes)
        # Make drain() actually apply backpressure at the same depth as the
        # kernel buffer, instead of asyncio's much larger default.
        try:
            writer.transport.set_write_buffer_limits(
                high=self._config.send_buffer_bytes,
                low=self._config.send_buffer_bytes // 4,
            )
        except (AttributeError, NotImplementedError):
            _log.debug("transport does not support write buffer limits")

        client = _ClientSession(reader, writer, self._config, peer)
        self._clients.append(client)
        self.accepted += 1
        _log.info("video client connected: %s (%d total)", peer, len(self._clients))

        self._enforce_client_limit()

        try:
            await client.run()
        except asyncio.CancelledError:
            client.close_soon("cancelled")
            raise
        except Exception:
            _log.exception("video client %s failed", peer)
        finally:
            try:
                self._clients.remove(client)
            except ValueError:
                pass
            self.frames_out += client.frames_sent
            self.frames_dropped += client.frames_dropped
            self.bytes_out += client.bytes_sent
            _log.info(
                "video client gone: %s (%s, sent=%d dropped=%d)",
                peer,
                client.close_reason,
                client.frames_sent,
                client.frames_dropped,
            )

    def _enforce_client_limit(self) -> None:
        while len(self._clients) > self._config.max_clients:
            victim = self._slowest()
            victim.close_soon("evicted: slowest client")
            self.evicted += 1
            _log.warning(
                "evicting video client %s (drop score %.1f); %d clients is the limit",
                victim.peer,
                victim.drop_score,
                self._config.max_clients,
            )
            # close_soon only asks; drop it from the roster now so the next
            # capture is not offered to a session that is on its way out.
            try:
                self._clients.remove(victim)
            except ValueError:  # pragma: no cover - defensive
                break

    def _slowest(self) -> _ClientSession:
        """Pick the eviction victim: worst recent drop score wins.

        Ties break towards the oldest connection, because the usual cause of a
        third connection is an app that reconnected before its previous socket
        was reaped -- the new one is the operator, the old one is a ghost.
        """
        return max(
            self._clients,
            key=lambda c: (c.drop_score, c.stats().queued, -c.connected_at),
        )

    # -- reporting ---------------------------------------------------------

    def stats(self) -> ServerStats:
        live = tuple(client.stats() for client in self._clients)
        return ServerStats(
            clients=live,
            frames_in=self.frames_in,
            frames_out=self.frames_out + sum(c.frames_sent for c in live),
            frames_dropped=self.frames_dropped + sum(c.frames_dropped for c in live),
            frames_rejected=self.frames_rejected,
            bytes_out=self.bytes_out + sum(c.bytes_sent for c in live),
            accepted=self.accepted,
            evicted=self.evicted,
            listening=self._server is not None,
        )

    def describe_clients(self) -> str:
        live = self.stats().clients
        if not live:
            return "no clients"
        return ", ".join(
            "%s sent=%d dropped=%d queued=%d"
            % (c.peer, c.frames_sent, c.frames_dropped, c.queued)
            for c in live
        )
