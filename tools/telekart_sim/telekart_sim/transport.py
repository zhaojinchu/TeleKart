"""Sockets, session handling, video serving, mDNS, and the run loop.

Everything a real car exposes on the network, exposed identically: UDP 4210 in,
UDP 4211 out, TCP 4212 for the JSON session, TCP 4213 for framed H.264, and an
mDNS advert on ``_telekart._tcp.local.``. If the desktop app can tell the
difference between this and the car, that is a bug here.

Two clocks run side by side and the distinction matters:

* **Simulated time** advances by a fixed step per control tick. It drives the
  physics, the telemetry timestamps and `--duration`, and it is what makes
  `--seed` reproducible even under `--realtime 0`.
* **Wall time** drives link staleness, the impairment delay line and the video
  thread, because those are properties of the real network and the real
  operator on the other end of it, neither of which fast-forwards.
"""

from __future__ import annotations

import contextlib
import errno
import hashlib
import heapq
import hmac
import logging
import math
import os
import random
import socket
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from telekart_protocol import (
    CONTROL_LOOP_HZ,
    MDNS_SERVICE_TYPE,
    PROTO_VERSION,
    TCP_SESSION_PORT,
    TCP_VIDEO_PORT,
    TELEMETRY_RATE_HZ,
    UDP_CONTROL_PORT,
    ControlPacket,
    ErrorCode,
    Fault,
    LineReader,
    Message,
    MsgType,
    ProtocolError,
    SessionError,
    TelemetryPacket,
    VehicleState,
    VideoCodec,
    VideoFrameFlags,
    derive_udp_key,
    make_session_token,
    normalize_shared_key,
    pack_frame,
)
from telekart_protocol.constants import MAX_CONTROL_PACKET_LEN
from telekart_protocol.params import PARAMS, ParamError
from telekart_protocol.session import ack, error, hello_ack
from telekart_protocol.session import params as params_msg
from telekart_protocol.session import state as state_msg
from telekart_protocol.video import FrameHeader

from .autodrive import PurePursuitDriver, Track
from .physics import NEUTRAL_COMMAND, JitterStats, SimCommand, SimSnapshot, VehicleSim
from .video_gen import FrameEncoder, SceneRenderer, VideoConfig

LOG = logging.getLogger("telekart.sim")

_MDNS_GROUP = "224.0.0.251"
_MDNS_PORT = 5353
#: Bytes a slow video client may fall behind before frames start being dropped.
#: The bound is the point: an unbounded queue turns a slow consumer into
#: unbounded latency, and DROPPED_BEFORE exists so the decoder is told.
_VIDEO_BACKLOG_LIMIT = 512 * 1024

_CAPS = ["video", "params", "calibrate", "sim"]


# --------------------------------------------------------------------------
# Options
# --------------------------------------------------------------------------


@dataclass(slots=True)
class NetworkOptions:
    """Link impairment. All defaults are a perfect network."""

    packet_loss: float = 0.0  # 0..1, applied per datagram in BOTH directions
    latency_s: float = 0.0  # one-way; round trip is roughly twice this
    jitter_s: float = 0.0  # +/- uniform, added per datagram
    tcp_drop_s: float = 0.0  # drop the session connection this often; 0 = never
    video_stall_s: float = 0.0  # length of each induced video stall
    video_stall_period_s: float = 30.0

    def validate(self) -> None:
        if not 0.0 <= self.packet_loss <= 1.0:
            raise ValueError(f"packet loss must be 0..1, got {self.packet_loss}")
        if self.latency_s < 0.0 or self.latency_s > 5.0:
            raise ValueError(
                f"latency must be 0..5000 ms, got {self.latency_s * 1000:.0f} ms"
            )
        if self.jitter_s < 0.0 or self.jitter_s > 5.0:
            raise ValueError(
                f"jitter must be 0..5000 ms, got {self.jitter_s * 1000:.0f} ms"
            )
        if self.tcp_drop_s < 0.0:
            raise ValueError("tcp-drop interval must not be negative")
        if self.video_stall_s < 0.0 or self.video_stall_period_s <= 0.0:
            raise ValueError("video stall duration/period must be positive")
        if self.video_stall_s >= self.video_stall_period_s:
            raise ValueError(
                "video stall duration must be shorter than its period, otherwise "
                "the stream never resumes"
            )

    @property
    def impaired(self) -> bool:
        return self.packet_loss > 0.0 or self.latency_s > 0.0 or self.jitter_s > 0.0


@dataclass(slots=True)
class ServerOptions:
    car_id: str = "telekart-sim"
    fw_version: str = "2.0.0-sim"
    shared_key: str = "telekart"
    bind_host: str = "0.0.0.0"
    advertise_ip: str = ""
    control_port: int = UDP_CONTROL_PORT
    session_port: int = TCP_SESSION_PORT
    video_port: int = TCP_VIDEO_PORT
    proto_version: int = PROTO_VERSION
    mdns: bool = True
    realtime: float = 1.0
    duration_s: float = 0.0
    status_interval_s: float = 5.0
    seed: int = 1

    def validate(self) -> None:
        if not self.car_id:
            raise ValueError("car id must not be empty")
        if self.realtime < 0.0:
            raise ValueError("realtime must be >= 0 (0 means unpaced)")
        if self.duration_s < 0.0:
            raise ValueError("duration must be >= 0 (0 means run forever)")
        for name, port in (
            ("control", self.control_port),
            ("session", self.session_port),
            ("video", self.video_port),
        ):
            if not 1 <= port <= 65535:
                raise ValueError(f"{name} port {port} is out of range")
        if not 0 <= self.proto_version <= 65535:
            raise ValueError(
                f"proto version {self.proto_version} does not fit the wire field"
            )


@dataclass(slots=True)
class SimStats:
    control_rx: int = 0
    control_bad: int = 0
    control_replayed: int = 0
    control_lost: int = 0
    telemetry_tx: int = 0
    telemetry_lost: int = 0
    video_frames: int = 0
    video_bytes: int = 0
    video_dropped: int = 0
    sessions: int = 0
    ticks: int = 0


# --------------------------------------------------------------------------
# Link impairment
# --------------------------------------------------------------------------


class DelayLine:
    """Holds datagrams for a while and then hands them on.

    A single thread with a heap rather than a timer per packet: at 100 Hz a
    timer per packet is thousands of thread wakeups a minute for no benefit.
    Reordering is allowed to happen naturally -- with jitter on, packets do
    arrive out of order, and the protocol's strictly-increasing sequence rule
    is supposed to drop the stale one. Hiding that would hide a real behaviour.
    """

    __slots__ = ("_condition", "_counter", "_heap", "_running", "_thread")

    def __init__(self) -> None:
        self._heap: list[tuple[float, int, Callable[[], None]]] = []
        self._condition = threading.Condition()
        self._thread: threading.Thread | None = None
        self._running = False
        self._counter = 0

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, name="sim-delay", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        with self._condition:
            self._running = False
            self._heap.clear()
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def submit(self, delay: float, action: Callable[[], None]) -> None:
        if delay <= 0.0 or not self._running:
            action()
            return
        with self._condition:
            self._counter += 1
            heapq.heappush(
                self._heap, (time.monotonic() + delay, self._counter, action)
            )
            self._condition.notify()

    def _run(self) -> None:
        while True:
            with self._condition:
                if not self._running:
                    return
                if not self._heap:
                    self._condition.wait(0.25)
                    continue
                due, _, action = self._heap[0]
                remaining = due - time.monotonic()
                if remaining > 0.0:
                    self._condition.wait(min(remaining, 0.25))
                    continue
                heapq.heappop(self._heap)
            try:
                action()
            except OSError:
                # A socket closed underneath a delayed send. Nothing to do and
                # nothing worth logging at 100 Hz.
                pass


# --------------------------------------------------------------------------
# Session state
# --------------------------------------------------------------------------


@dataclass(slots=True)
class SessionState:
    session_id: int
    token: bytes
    udp_key: bytes
    peer_ip: str
    telemetry_port: int
    driver: str
    connection: SessionConnection
    last_sequence: int = 0
    telemetry_sequence: int = 0
    echo_client_time_us: int = 0
    echo_sequence: int = 0


@dataclass(frozen=True, slots=True)
class InboundControl:
    """One accepted control packet, published to the loop as a single slot."""

    command: SimCommand
    client_time_us: int
    sequence: int
    wall_received: float


# --------------------------------------------------------------------------
# The server
# --------------------------------------------------------------------------


class SimServer:
    """One simulated car: physics, network, video, discovery."""

    def __init__(
        self,
        *,
        vehicle: VehicleSim,
        track: Track,
        options: ServerOptions,
        network: NetworkOptions,
        driver: PurePursuitDriver | None = None,
        video: VideoConfig | None = None,
    ) -> None:
        options.validate()
        network.validate()
        self.vehicle = vehicle
        self.track = track
        self.options = options
        self.network = network
        self.driver = driver
        self.video_config = video
        self.stats = SimStats()
        self.jitter = JitterStats()

        self._key = normalize_shared_key(options.shared_key)
        # Impairment gets its own stream so that connecting a second time, or
        # not connecting at all, does not shift which packets get dropped.
        self._rng = random.Random(options.seed ^ 0x51ED0004)
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._link_epoch = time.monotonic()
        self._session: SessionState | None = None
        self._inbound: InboundControl | None = None
        self._latest: SimSnapshot | None = None
        self._threads: list[threading.Thread] = []
        self._delay = DelayLine()
        self._started_wall = 0.0
        self._sim_time = 0.0
        self._last_state: tuple[VehicleState, Fault] | None = None
        self._warned_autodrive_override = False
        self._calibrating = False

        self._control_sock = self._bind_udp(
            options.bind_host, options.control_port, "control"
        )
        self._telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._session_sock = self._bind_tcp(
            options.bind_host, options.session_port, "session"
        )
        self._video_sock: socket.socket | None = None
        self._video_clients: list[_VideoClient] = []
        self._video_lock = threading.Lock()
        self._renderer: SceneRenderer | None = None
        self._encoder: FrameEncoder | None = None

        if video is not None:
            self._video_sock = self._bind_tcp(
                options.bind_host, options.video_port, "video"
            )
            # Constructed here, on the main thread, so a missing PyAV or a bad
            # geometry is a startup failure with a readable message rather than
            # a thread that dies quietly thirty seconds in.
            self._renderer = SceneRenderer(video, track, seed=options.seed)
            self._encoder = FrameEncoder(video)

        self._advertiser: MdnsAdvertiser | None = None
        if options.mdns:
            self._advertiser = MdnsAdvertiser(
                car_id=options.car_id,
                port=options.session_port,
                address=options.advertise_ip or detect_local_ip(),
                properties={
                    "car": options.car_id,
                    "proto": str(options.proto_version),
                    "fw": options.fw_version,
                    "sim": "1",
                },
            )

    # -- socket setup ---------------------------------------------------------

    @staticmethod
    def _bind_udp(host: str, port: int, label: str) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, port))
        except OSError as exc:
            sock.close()
            raise SystemExit(
                f"cannot bind UDP {label} port {port}: {exc}. Another simulator or "
                f"the real car may already be running here."
            ) from exc
        sock.settimeout(0.25)
        return sock

    @staticmethod
    def _bind_tcp(host: str, port: int, label: str) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, port))
            sock.listen(4)
        except OSError as exc:
            sock.close()
            raise SystemExit(f"cannot bind TCP {label} port {port}: {exc}") from exc
        sock.settimeout(0.25)
        return sock

    # -- lifecycle ------------------------------------------------------------

    def run(self) -> int:
        """Run until stopped or `--duration` elapses. Returns a process exit code."""
        self._started_wall = time.monotonic()
        if self.network.impaired:
            self._delay.start()

        if self.driver is not None:
            # The virtual operator counts as presence. Without it the car would
            # refuse to arm with "no session" and autodrive would sit on the
            # grid forever.
            with self.lock:
                self.vehicle.note_session(True)

        self._spawn(self._control_rx_loop, "sim-control-rx")
        self._spawn(self._session_accept_loop, "sim-session")
        if self._video_sock is not None:
            self._spawn(self._video_accept_loop, "sim-video-accept")
            self._spawn(self._video_encode_loop, "sim-video")
        if self._advertiser is not None:
            self._advertiser.start()

        LOG.info(
            "simulator up: control udp/%d  session tcp/%d  video %s  track %s  seed %d",
            self.options.control_port,
            self.options.session_port,
            f"tcp/{self.options.video_port}" if self._video_sock else "disabled",
            self.track.name,
            self.options.seed,
        )
        LOG.info(
            "measured v_max %.3f m/s (%.0f RPM) -- nothing here hardcodes a top speed",
            self.vehicle.v_max,
            self.vehicle.max_rpm_measured,
        )

        try:
            self._physics_loop()
        except KeyboardInterrupt:
            LOG.info("interrupted")
        finally:
            self.stop()
        return 0

    def stop(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        if self._advertiser is not None:
            self._advertiser.stop()
        session = self._session
        if session is not None:
            session.connection.close()
        with self._video_lock:
            for client in self._video_clients:
                client.close()
            self._video_clients.clear()
        for sock in (
            self._control_sock,
            self._telemetry_sock,
            self._session_sock,
            self._video_sock,
        ):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._delay.stop()
        for thread in self._threads:
            thread.join(timeout=1.5)
        if self._encoder is not None:
            self._encoder.close()
        LOG.info(
            "stats: %d ticks, %d control rx (%d bad, %d replay, %d lost), "
            "%d telemetry tx, %d video frames (%.1f MB, %d dropped)",
            self.stats.ticks,
            self.stats.control_rx,
            self.stats.control_bad,
            self.stats.control_replayed,
            self.stats.control_lost,
            self.stats.telemetry_tx,
            self.stats.video_frames,
            self.stats.video_bytes / 1e6,
            self.stats.video_dropped,
        )

    @property
    def stopped(self) -> bool:
        return self._stop.is_set()

    def _spawn(self, target: Callable[[], None], name: str) -> None:
        thread = threading.Thread(target=target, name=name, daemon=True)
        thread.start()
        self._threads.append(thread)

    # -- the loop -------------------------------------------------------------

    def _physics_loop(self) -> None:
        period = 1.0 / CONTROL_LOOP_HZ
        telemetry_divisor = max(1, CONTROL_LOOP_HZ // TELEMETRY_RATE_HZ)
        realtime = self.options.realtime
        next_deadline = time.monotonic() + period
        last_wall = time.monotonic()
        # Status is paced in SIMULATED time so that a `--realtime 0` run still
        # prints a readable trace instead of one line for the whole session.
        next_status = self.options.status_interval_s
        next_tcp_drop = (
            last_wall + self.network.tcp_drop_s
            if self.network.tcp_drop_s > 0.0
            else math.inf
        )
        tick = 0

        while not self._stop.is_set():
            tick += 1
            self.stats.ticks = tick
            # Simulated time always advances by exactly one period. That, not
            # the wall clock, is what makes --seed reproducible.
            self._sim_time += period

            command = self._collect_command(period)
            with self.lock:
                snapshot = self.vehicle.step(period, command)
                self.vehicle.loop_p99_us = self.jitter.p99_us()
            self._latest = snapshot

            if tick % telemetry_divisor == 0:
                self._send_telemetry(snapshot)
            self._publish_state_change(snapshot)

            now = time.monotonic()
            self.jitter.add(now - last_wall)
            last_wall = now

            if self._sim_time >= next_status and self.options.status_interval_s > 0.0:
                next_status = self._sim_time + self.options.status_interval_s
                self._log_status(snapshot)
            if now >= next_tcp_drop:
                next_tcp_drop = now + self.network.tcp_drop_s
                self._force_tcp_drop()
            if (
                self.options.duration_s > 0.0
                and self._sim_time >= self.options.duration_s
            ):
                LOG.info(
                    "reached --duration %.1f s of simulated time",
                    self.options.duration_s,
                )
                break

            if realtime > 0.0:
                # Absolute deadlines, so period error never accumulates. On an
                # overrun we skip ahead rather than trying to catch up, which
                # would just produce a burst of back-to-back ticks.
                next_deadline += period / realtime
                sleep = next_deadline - time.monotonic()
                if sleep > 0.0:
                    time.sleep(sleep)
                elif sleep < -0.5:
                    next_deadline = time.monotonic() + period / realtime

    def _collect_command(self, dt: float) -> SimCommand:
        inbound = self._inbound  # single-slot read; assignment is atomic
        now = time.monotonic()

        if inbound is None:
            # Age from when the link was last known good, so a session that
            # connects and then says nothing still trips the failsafe.
            self.vehicle.note_link_age(now - self._link_epoch)
        else:
            session = self._session
            if session is not None:
                session.echo_client_time_us = inbound.client_time_us
                session.echo_sequence = inbound.sequence
            self.vehicle.note_link_age(now - inbound.wall_received)

        if self.driver is not None:
            if inbound is not None and not self._warned_autodrive_override:
                self._warned_autodrive_override = True
                LOG.warning("--autodrive is on: incoming control packets are ignored")
            snapshot = self._latest
            armed = snapshot is not None and snapshot.state is VehicleState.ARMED
            if snapshot is None:
                pose = (
                    self.track.start_x,
                    self.track.start_y,
                    self.track.start_heading,
                )
                speed = 0.0
            else:
                # The virtual operator drives from GROUND TRUTH, the way a human
                # watching the car does. Feeding it the dead-reckoned pose would
                # make it wander off as odometry drifts, which would corrupt the
                # reference laps and destroy the very signal --track-width-error
                # exists to expose: truth versus what the car believes.
                pose = (snapshot.true_x, snapshot.true_y, snapshot.true_heading)
                speed = snapshot.true_speed
            self.vehicle.note_link_age(0.0)
            command = self.driver.update(pose[0], pose[1], pose[2], speed, dt, armed)
            if not armed and self.vehicle.state is VehicleState.SAFE:
                with self.lock:
                    self.vehicle.request_arm()
            return command

        if inbound is None:
            return NEUTRAL_COMMAND
        return inbound.command

    def _log_status(self, snapshot: SimSnapshot) -> None:
        laps = self.driver.laps if self.driver is not None else 0
        LOG.info(
            "t=%6.1fs %-8s speed %5.2f m/s  rpm %4.0f/%4.0f  duty %+.2f/%+.2f  "
            "pose (%5.2f, %5.2f) %6.1f deg  slip %.3f  pack %.2f V  laps %d  faults %s",
            snapshot.t,
            snapshot.state.name,
            snapshot.speed,
            snapshot.rpm_l,
            snapshot.rpm_r,
            snapshot.duty_l,
            snapshot.duty_r,
            snapshot.x,
            snapshot.y,
            math.degrees(snapshot.heading),
            snapshot.slip,
            snapshot.pack_v,
            laps,
            snapshot.faults.name or "none",
        )

    # -- UDP ------------------------------------------------------------------

    def _control_rx_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self._control_sock.recvfrom(MAX_CONTROL_PACKET_LEN)
            except TimeoutError:
                continue
            except OSError:
                return
            if self._drop_datagram():
                self.stats.control_lost += 1
                continue
            delay = self._one_way_delay()
            if delay > 0.0:
                payload = bytes(data)
                self._delay.submit(
                    delay, lambda p=payload, a=addr: self._accept_control(p, a)
                )
            else:
                self._accept_control(data, addr)

    def _accept_control(self, data: bytes, addr: tuple[str, int]) -> None:
        session = self._session
        if session is None:
            return
        try:
            packet = ControlPacket.unpack(data, session.udp_key)
        except ProtocolError:
            # Never raise out of a decode path. A bad packet is a dropped packet
            # and a counter, nothing more.
            self.stats.control_bad += 1
            return
        if packet.session_id != session.session_id:
            self.stats.control_bad += 1
            return
        if packet.sequence <= session.last_sequence:
            # Strictly increasing, no window: on a 100 Hz stream a reordered
            # packet is a stale packet and acting on it is worse than dropping.
            self.stats.control_replayed += 1
            return
        session.last_sequence = packet.sequence
        self.stats.control_rx += 1
        self._inbound = InboundControl(
            command=SimCommand(
                steering=packet.steering_f,
                throttle=packet.throttle_f,
                brake=packet.brake_f,
                flags=packet.flags,
            ),
            client_time_us=packet.client_time_us,
            sequence=packet.sequence,
            wall_received=time.monotonic(),
        )

    def _send_telemetry(self, snapshot: SimSnapshot) -> None:
        session = self._session
        if session is None:
            return
        session.telemetry_sequence += 1
        packet = TelemetryPacket.from_si(
            session_id=session.session_id,
            sequence=session.telemetry_sequence,
            car_time_us=int(self._sim_time * 1_000_000.0),
            state=snapshot.state,
            faults=snapshot.faults,
            flags=snapshot.flags,
            echo_client_time_us=session.echo_client_time_us,
            echo_sequence=session.echo_sequence,
            rpm_l=snapshot.rpm_l,
            rpm_r=snapshot.rpm_r,
            rpm_target_l=snapshot.rpm_target_l,
            rpm_target_r=snapshot.rpm_target_r,
            duty_l=snapshot.duty_l,
            duty_r=snapshot.duty_r,
            servo_us=snapshot.servo_us,
            steer_angle_deg=math.degrees(snapshot.steer_angle),
            speed_mps=snapshot.speed,
            v_max_mps=snapshot.v_max,
            x_m=snapshot.x,
            y_m=snapshot.y,
            heading_rad=snapshot.heading,
            distance_m=snapshot.distance,
            slip=snapshot.slip,
            pack_volts=snapshot.pack_v,
            cpu_temp_c=snapshot.cpu_temp_c,
            throttled=snapshot.throttled,
            loop_p99_us=snapshot.loop_p99_us,
        )
        if self.options.proto_version != PROTO_VERSION:
            # Only reachable via --proto-version, whose whole purpose is to make
            # the app's version-mismatch path fire.
            packet = _with_version(packet, self.options.proto_version)

        try:
            payload = packet.pack(session.udp_key)
        except (ValueError, struct.error):
            return
        if self._drop_datagram():
            self.stats.telemetry_lost += 1
            return
        target = (session.peer_ip, session.telemetry_port)
        delay = self._one_way_delay()
        if delay > 0.0:
            self._delay.submit(delay, lambda p=payload, t=target: self._sendto(p, t))
        else:
            self._sendto(payload, target)

    def _sendto(self, payload: bytes, target: tuple[str, int]) -> None:
        try:
            self._telemetry_sock.sendto(payload, target)
            self.stats.telemetry_tx += 1
        except OSError:
            pass

    def _drop_datagram(self) -> bool:
        loss = self.network.packet_loss
        return loss > 0.0 and self._rng.random() < loss

    def _one_way_delay(self) -> float:
        base = self.network.latency_s
        jitter = self.network.jitter_s
        if base <= 0.0 and jitter <= 0.0:
            return 0.0
        if jitter > 0.0:
            base += self._rng.uniform(-jitter, jitter)
        return max(0.0, base)

    # -- TCP session ----------------------------------------------------------

    def _session_accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                client, addr = self._session_sock.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            if self._session is not None:
                # One operator at a time, exactly like the car. A second laptop
                # gets a clear refusal rather than a silent fight over the wheel.
                try:
                    client.sendall(
                        error(
                            0, ErrorCode.BUSY, "another session is already connected"
                        ).encode()
                    )
                except OSError:
                    pass
                finally:
                    client.close()
                LOG.warning("refused a second session from %s", addr[0])
                continue
            connection = SessionConnection(self, client, addr)
            thread = threading.Thread(
                target=connection.serve, name=f"sim-session-{addr[1]}", daemon=True
            )
            thread.start()
            self._threads.append(thread)

    def register_session(self, session: SessionState) -> None:
        self._session = session
        self._link_epoch = time.monotonic()
        self.stats.sessions += 1
        with self.lock:
            self.vehicle.note_session(True)
        LOG.info(
            "session %d for driver %r from %s, telemetry to udp/%d",
            session.session_id,
            session.driver,
            session.peer_ip,
            session.telemetry_port,
        )

    def clear_session(self, session: SessionState) -> None:
        if self._session is not session:
            return
        self._session = None
        self._inbound = None
        with self.lock:
            # Losing TCP is losing the operator. The car stops even if UDP is
            # still arriving from somewhere -- that is the point of using the
            # connection as a presence signal.
            self.vehicle.note_session(self.driver is not None)
        LOG.info("session %d ended", session.session_id)

    def _force_tcp_drop(self) -> None:
        session = self._session
        if session is None:
            return
        LOG.warning("--tcp-drop: severing the session connection")
        session.connection.abort()

    def _publish_state_change(self, snapshot: SimSnapshot) -> None:
        current = (snapshot.state, snapshot.faults)
        if current == self._last_state:
            return
        previous = self._last_state
        self._last_state = current
        if previous is None:
            return
        detail = ""
        if snapshot.faults != previous[1]:
            detail = snapshot.faults.name or "cleared"
        session = self._session
        if session is not None:
            session.connection.send(
                state_msg(int(snapshot.state), int(snapshot.faults), detail)
            )
        LOG.info(
            "state %s -> %s%s",
            previous[0].name,
            snapshot.state.name,
            f" ({detail})" if detail else "",
        )

    # -- video ----------------------------------------------------------------

    def _video_accept_loop(self) -> None:
        assert self._video_sock is not None
        while not self._stop.is_set():
            try:
                client, addr = self._video_sock.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            client.setblocking(False)
            with self._video_lock:
                self._video_clients.append(_VideoClient(client, addr[0]))
            LOG.info("video client connected from %s", addr[0])

    def _video_encode_loop(self) -> None:
        assert (
            self._renderer is not None
            and self._encoder is not None
            and self.video_config
        )
        config = self.video_config
        period = 1.0 / config.fps
        sequence = 0
        next_frame = time.monotonic() + period
        stall_until = 0.0
        next_stall = (
            time.monotonic() + self.network.video_stall_period_s
            if self.network.video_stall_s > 0.0
            else math.inf
        )

        while not self._stop.is_set():
            now = time.monotonic()
            sleep = next_frame - now
            if sleep > 0.0:
                time.sleep(min(sleep, 0.25))
                continue
            next_frame += period
            if next_frame < now - 0.5:
                next_frame = now + period

            with self._video_lock:
                clients = list(self._video_clients)
            self.vehicle.video_active = bool(clients)
            if not clients:
                continue

            if now >= next_stall:
                stall_until = now + self.network.video_stall_s
                next_stall = now + self.network.video_stall_period_s
                LOG.warning(
                    "--video-stall: freezing the stream for %.1f s",
                    self.network.video_stall_s,
                )
            if now < stall_until:
                continue

            snapshot = self._latest
            if snapshot is None:
                continue
            yaw_rate = (
                snapshot.true_speed
                * math.tan(snapshot.steer_angle)
                / float(self.vehicle.params["wheelbase_m"])
            )
            pts_us = int((now - self._started_wall) * 1_000_000.0)
            need_key = any(client.needs_keyframe for client in clients)
            image = self._renderer.render(
                x=snapshot.true_x,
                y=snapshot.true_y,
                heading=snapshot.true_heading,
                speed=snapshot.true_speed,
                yaw_rate=yaw_rate,
                timestamp_ms=pts_us // 1000,
                frame_index=sequence,
            )
            for encoded in self._encoder.encode(image, pts_us, force_keyframe=need_key):
                sequence += 1
                self.stats.video_frames += 1
                self.stats.video_bytes += len(encoded.payload)
                self._fan_out(
                    clients, sequence, encoded.payload, encoded.pts_us, encoded.keyframe
                )

        self._flush_video()

    def _fan_out(
        self,
        clients: list[_VideoClient],
        sequence: int,
        payload: bytes,
        pts_us: int,
        keyframe: bool,
    ) -> None:
        assert self._encoder is not None
        codec = self._encoder.codec
        frame_plain = pack_frame(
            sequence, pts_us, payload, codec=codec, keyframe=keyframe
        )
        # The DROPPED_BEFORE variant is packed lazily and at most once per frame,
        # not once per client: packing copies the whole access unit, and on a
        # healthy link no client ever needs the flagged copy at all.
        frame_flagged: bytes | None = None
        extradata = self._encoder.extradata
        dead: list[_VideoClient] = []
        for client in clients:
            if client.needs_keyframe:
                if not keyframe:
                    continue
                if extradata:
                    client.push(
                        _config_frame(sequence, pts_us, extradata, codec), force=True
                    )
                client.needs_keyframe = False
            blob = frame_plain
            if client.take_dropped():
                if frame_flagged is None:
                    frame_flagged = pack_frame(
                        sequence,
                        pts_us,
                        payload,
                        codec=codec,
                        keyframe=keyframe,
                        dropped_before=True,
                    )
                blob = frame_flagged
            if not client.push(blob, force=keyframe):
                self.stats.video_dropped += 1
            if not client.alive:
                dead.append(client)
        if dead:
            with self._video_lock:
                for client in dead:
                    if client in self._video_clients:
                        self._video_clients.remove(client)
                    client.close()
                    LOG.info("video client %s disconnected", client.peer)

    def _flush_video(self) -> None:
        if self._encoder is None:
            return
        for _ in self._encoder.flush():
            pass


def _with_version(packet: TelemetryPacket, version: int) -> TelemetryPacket:
    """Restamp a telemetry packet's version field.

    `dataclasses.replace` would be tidier, but TelemetryPacket has thirty-odd
    fields and this runs at 50 Hz; building the tuple by hand costs nothing.
    """
    return TelemetryPacket(
        session_id=packet.session_id,
        sequence=packet.sequence,
        car_time_us=packet.car_time_us,
        echo_client_time_us=packet.echo_client_time_us,
        echo_sequence=packet.echo_sequence,
        state=packet.state,
        faults=packet.faults,
        flags=packet.flags,
        rpm_l=packet.rpm_l,
        rpm_r=packet.rpm_r,
        rpm_target_l=packet.rpm_target_l,
        rpm_target_r=packet.rpm_target_r,
        duty_l=packet.duty_l,
        duty_r=packet.duty_r,
        servo_us=packet.servo_us,
        steer_angle_cdeg=packet.steer_angle_cdeg,
        speed_mm_s=packet.speed_mm_s,
        v_max_mm_s=packet.v_max_mm_s,
        odom_x_mm=packet.odom_x_mm,
        odom_y_mm=packet.odom_y_mm,
        heading_cdeg=packet.heading_cdeg,
        distance_mm=packet.distance_mm,
        slip_index=packet.slip_index,
        pack_mv=packet.pack_mv,
        cpu_temp_dc=packet.cpu_temp_dc,
        throttled=packet.throttled,
        loop_p99_us=packet.loop_p99_us,
        version=version,
    )


def _config_frame(
    sequence: int, pts_us: int, payload: bytes, codec: VideoCodec
) -> bytes:
    header = FrameHeader(
        sequence=sequence,
        pts_us=pts_us,
        flags=VideoFrameFlags.CONFIG,
        codec=codec,
        length=len(payload),
    )
    return header.pack() + payload


# --------------------------------------------------------------------------
# Video client
# --------------------------------------------------------------------------


class _VideoClient:
    """A connected viewer with a bounded outgoing buffer.

    Bounded on purpose. An unbounded buffer converts a slow consumer into
    unbounded latency, which on a video feed is worse than losing frames -- and
    the protocol has a DROPPED_BEFORE bit precisely so the decoder can be told.
    """

    __slots__ = ("_buffer", "_dropped", "alive", "needs_keyframe", "peer", "sock")

    def __init__(self, sock: socket.socket, peer: str) -> None:
        self.sock = sock
        self.peer = peer
        self._buffer = bytearray()
        self.needs_keyframe = True
        self._dropped = False
        self.alive = True

    def take_dropped(self) -> bool:
        was = self._dropped
        self._dropped = False
        return was

    def push(self, blob: bytes, *, force: bool = False) -> bool:
        if not self.alive:
            return False
        if len(self._buffer) + len(blob) > _VIDEO_BACKLOG_LIMIT:
            if not force:
                self._dropped = True
                self._flush()
                return False
            # A keyframe is worth discarding the backlog for: it is the only
            # thing that resynchronises the decoder.
            self._buffer.clear()
            self._dropped = True
        self._buffer.extend(blob)
        self._flush()
        return True

    def _flush(self) -> None:
        while self._buffer and self.alive:
            try:
                sent = self.sock.send(self._buffer)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    return
                self.alive = False
                return
            if sent <= 0:
                self.alive = False
                return
            del self._buffer[:sent]

    def close(self) -> None:
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass


# --------------------------------------------------------------------------
# Session connection
# --------------------------------------------------------------------------


class SessionConnection:
    """One TCP/JSON conversation with an operator.

    Runs on its own thread and touches the vehicle only under the server's lock.
    Session traffic is event-rate, so a lock is the right tool here -- the
    lock-free single-slot mailbox exists for the 100 Hz control path, where it
    earns its explanation.
    """

    def __init__(
        self, server: SimServer, sock: socket.socket, addr: tuple[str, int]
    ) -> None:
        self._server = server
        self._sock = sock
        self._addr = addr
        self._reader = LineReader()
        self._session: SessionState | None = None
        self._nonce = os.urandom(16)
        self._closed = False
        self._send_lock = threading.Lock()

    # -- io -------------------------------------------------------------------

    def send(self, message: Message) -> None:
        if self._closed:
            return
        try:
            payload = message.encode()
        except (SessionError, ValueError) as exc:
            LOG.error("refusing to send a malformed session message: %s", exc)
            return
        with self._send_lock:
            try:
                self._sock.sendall(payload)
            except OSError:
                self._closed = True

    def close(self) -> None:
        self._closed = True
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass

    def abort(self) -> None:
        """Sever the connection the way a WiFi dropout does: RST, no FIN."""
        try:
            self._sock.setsockopt(
                socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0)
            )
        except OSError:
            pass
        self.close()

    # -- protocol -------------------------------------------------------------

    def serve(self) -> None:
        self._sock.settimeout(0.5)
        # The car greets first with a nonce, before the app says anything. See
        # docs/protocol.md 5.5: the package's `hello()` docstring refers to a
        # nonce message that does not exist in the frozen source, and this ACK
        # form is the resolution both ends are built against.
        self.send(ack(0, nonce=self._nonce.hex()))
        try:
            self._read_loop()
        finally:
            if self._session is not None:
                self._server.clear_session(self._session)
            self.close()

    def _read_loop(self) -> None:
        while not self._closed and not self._server.stopped:
            try:
                chunk = self._sock.recv(4096)
            except TimeoutError:
                continue
            except OSError:
                return
            if not chunk:
                return
            try:
                messages = self._reader.feed(chunk)
            except SessionError as exc:
                # Malformed session traffic is fatal to the session by design;
                # resynchronising a JSON stream is not worth attempting.
                LOG.warning("session from %s: %s", self._addr[0], exc)
                self.send(error(0, ErrorCode.BAD_REQUEST, str(exc)))
                return
            for message in messages:
                if not self._dispatch(message):
                    return

    def _dispatch(self, message: Message) -> bool:
        """Handle one message. Returns False to tear the connection down."""
        server = self._server
        vehicle = server.vehicle

        if self._session is None and message.type is not MsgType.HELLO:
            self.send(error(message.id, ErrorCode.BAD_REQUEST, "expected hello first"))
            return False

        if message.type is MsgType.HELLO:
            return self._handle_hello(message)

        if message.type is MsgType.PING:
            self.send(Message(MsgType.PONG, message.id, dict(message.data)))
            return True

        if message.type is MsgType.ARM:
            with server.lock:
                accepted, reason = vehicle.request_arm()
            if accepted:
                self.send(ack(message.id, state=int(vehicle.state)))
            else:
                self.send(error(message.id, ErrorCode.NOT_ALLOWED_IN_STATE, reason))
            return True

        if message.type is MsgType.DISARM:
            with server.lock:
                vehicle.request_disarm()
            self.send(ack(message.id, state=int(vehicle.state)))
            return True

        if message.type is MsgType.ESTOP:
            with server.lock:
                vehicle.request_estop()
            self.send(ack(message.id, state=int(vehicle.state)))
            return True

        if message.type is MsgType.CLEAR_ESTOP:
            with server.lock:
                accepted, reason = vehicle.clear_estop()
            if accepted:
                self.send(ack(message.id, state=int(vehicle.state)))
            else:
                self.send(error(message.id, ErrorCode.NOT_ALLOWED_IN_STATE, reason))
            return True

        if message.type is MsgType.CLEAR_FAULTS:
            with server.lock:
                vehicle.clear_faults()
            self.send(ack(message.id, faults=int(vehicle.faults)))
            return True

        if message.type is MsgType.RESET_ODOM:
            with server.lock:
                vehicle.reset_odometry()
            self.send(ack(message.id))
            return True

        if message.type is MsgType.GET_PARAMS:
            group = message.data.get("group")
            with server.lock:
                values = dict(vehicle.params)
            if isinstance(group, str):
                values = {k: v for k, v in values.items() if PARAMS[k].group == group}
            self.send(params_msg(message.id, values))
            return True

        if message.type is MsgType.SET_PARAMS:
            return self._handle_set_params(message)

        if message.type is MsgType.CALIBRATE:
            return self._handle_calibrate(message)

        self.send(
            error(
                message.id,
                ErrorCode.BAD_REQUEST,
                f"{message.type.value} is not accepted here",
            )
        )
        return True

    def _handle_hello(self, message: Message) -> bool:
        server = self._server
        options = server.options
        if self._session is not None:
            self.send(
                error(message.id, ErrorCode.BAD_REQUEST, "hello already completed")
            )
            return False

        try:
            proto = message.require("proto", int)
            auth = message.require("auth", str)
            driver = message.require("driver", str)
            telemetry_port = message.require("telemetry_port", int)
        except SessionError as exc:
            self.send(error(message.id, ErrorCode.BAD_REQUEST, str(exc)))
            return False

        if proto != options.proto_version:
            self.send(
                error(
                    message.id,
                    ErrorCode.PROTOCOL_VERSION,
                    f"car speaks protocol {options.proto_version}, app offered {proto}",
                )
            )
            return False
        if not 1 <= telemetry_port <= 65535:
            self.send(
                error(message.id, ErrorCode.BAD_REQUEST, "telemetry_port out of range")
            )
            return False

        expected = hmac.new(server._key, self._nonce, hashlib.sha256).hexdigest()
        if not hmac.compare_digest(expected, auth.lower()):
            self.send(
                error(
                    message.id, ErrorCode.AUTH_FAILED, "nonce signature did not verify"
                )
            )
            return False

        token = make_session_token()
        # session_id 0 is reserved for "no session", so the car never issues it.
        # Drawn from urandom rather than the seeded stream: a session id that
        # depended on --seed would let a stale second app guess it.
        session_id = int.from_bytes(os.urandom(4), "little") | 1
        session = SessionState(
            session_id=session_id,
            token=token,
            udp_key=derive_udp_key(server._key, token),
            peer_ip=self._addr[0],
            telemetry_port=telemetry_port,
            driver=driver,
            connection=self,
        )
        self._session = session
        server.register_session(session)

        reply = hello_ack(
            msg_id=message.id,
            car_id=options.car_id,
            fw_version=options.fw_version,
            session_id=session_id,
            session_token=token.hex(),
            caps=_CAPS,
            video_port=options.video_port,
            control_port=options.control_port,
        )
        if options.proto_version != PROTO_VERSION:
            reply.data["proto"] = options.proto_version
        self.send(reply)
        self.send(
            state_msg(
                int(server.vehicle.state), int(server.vehicle.faults), "session ready"
            )
        )
        return True

    def _handle_set_params(self, message: Message) -> bool:
        server = self._server
        try:
            values = message.require("values", dict)
        except SessionError as exc:
            self.send(error(message.id, ErrorCode.BAD_REQUEST, str(exc)))
            return True
        try:
            with server.lock:
                changed = server.vehicle.apply_params(values)
                echo = dict(server.vehicle.params)
        except ParamError as exc:
            detail = str(exc)
            code = (
                ErrorCode.UNKNOWN_PARAM
                if "unknown parameter" in detail
                else ErrorCode.NOT_ALLOWED_IN_STATE
                if "while armed" in detail
                else ErrorCode.PARAM_OUT_OF_RANGE
            )
            self.send(error(message.id, code, detail))
            return True
        if changed:
            LOG.info("params changed: %s", ", ".join(sorted(changed)))
        # The car's copy is authoritative; the app renders this, never what it
        # optimistically typed.
        self.send(params_msg(message.id, echo))
        return True

    def _handle_calibrate(self, message: Message) -> bool:
        server = self._server
        if server._calibrating:
            self.send(
                error(message.id, ErrorCode.BUSY, "a calibration is already running")
            )
            return True
        if server.vehicle.state is VehicleState.ARMED:
            self.send(
                error(
                    message.id,
                    ErrorCode.NOT_ALLOWED_IN_STATE,
                    "disarm before calibrating",
                )
            )
            return True
        server._calibrating = True
        self.send(ack(message.id))
        thread = threading.Thread(
            target=self._run_calibration,
            args=(message.id, str(message.data.get("routine", "drive"))),
            name="sim-calibrate",
            daemon=True,
        )
        thread.start()
        return True

    def _run_calibration(self, request_id: int, routine: str) -> None:
        server = self._server
        phases = ("left_fwd", "left_rev", "right_fwd", "right_rev")
        try:
            for index, phase in enumerate(phases):
                for step in range(5):
                    if self._closed or server.stopped:
                        return
                    time.sleep(0.1)
                    progress = (index * 5 + step + 1) / (len(phases) * 5)
                    self.send(
                        Message(
                            MsgType.CALIBRATION_STATUS,
                            0,
                            {
                                "id": request_id,
                                "routine": routine,
                                "phase": phase,
                                "progress": round(progress, 3),
                                "done": False,
                            },
                        )
                    )
            with server.lock:
                max_rpm = server.vehicle.max_rpm_measured
                v_max = server.vehicle.v_max
            self.send(
                Message(
                    MsgType.CALIBRATION_STATUS,
                    0,
                    {
                        "id": request_id,
                        "routine": routine,
                        "phase": "done",
                        "progress": 1.0,
                        "done": True,
                        "result": {
                            "max_rpm_measured": round(max_rpm, 2),
                            "v_max_mps": round(v_max, 4),
                            "on_ground": True,
                        },
                    },
                )
            )
        finally:
            server._calibrating = False


# --------------------------------------------------------------------------
# Discovery
# --------------------------------------------------------------------------


def detect_local_ip() -> str:
    """The address this host would use to reach the LAN.

    Connecting a UDP socket sends nothing; it only asks the routing table which
    interface would be used. That is far more reliable than gethostbyname on a
    machine with a VPN, a container bridge, and three virtual interfaces.
    """
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("192.0.2.1", 9))  # TEST-NET-1: routable, never answers
        return str(probe.getsockname()[0])
    except OSError:
        return "127.0.0.1"
    finally:
        probe.close()


class MdnsAdvertiser:
    """Advertises `_telekart._tcp.local.`, with or without the zeroconf package.

    zeroconf is used when it is installed. It is not a declared dependency of
    the simulator, though, and discovery is the first thing the desktop app
    does -- so there is a small built-in responder as well. It answers PTR, SRV,
    TXT and A queries for this one service and nothing else, which is all that
    is needed and all that should be attempted.
    """

    def __init__(
        self, *, car_id: str, port: int, address: str, properties: dict[str, str]
    ) -> None:
        self.car_id = car_id
        self.port = port
        self.address = address
        self.properties = properties
        self.instance = f"{car_id}.{MDNS_SERVICE_TYPE}"
        self.hostname = f"{car_id}.local."
        self._zeroconf: Any = None
        self._info: Any = None
        self._responder: _MdnsResponder | None = None

    def start(self) -> None:
        if self._start_zeroconf():
            return
        self._responder = _MdnsResponder(self)
        self._responder.start()

    def _start_zeroconf(self) -> bool:
        try:
            from zeroconf import ServiceInfo, Zeroconf
        except ImportError:
            return False
        try:
            self._info = ServiceInfo(
                MDNS_SERVICE_TYPE,
                self.instance,
                addresses=[socket.inet_aton(self.address)],
                port=self.port,
                properties={k.encode(): v.encode() for k, v in self.properties.items()},
                server=self.hostname,
            )
            self._zeroconf = Zeroconf()
            self._zeroconf.register_service(self._info, allow_name_change=True)
        except Exception as exc:  # noqa: BLE001 - zeroconf raises broadly
            LOG.warning(
                "zeroconf advert failed (%s); using the built-in responder", exc
            )
            self._zeroconf = None
            return False
        LOG.info(
            "advertising %s at %s:%d via zeroconf",
            self.instance,
            self.address,
            self.port,
        )
        return True

    def stop(self) -> None:
        if self._zeroconf is not None:
            # zeroconf raises a variety of types on a half-torn-down socket and
            # none of them are worth taking the shutdown path down for.
            with contextlib.suppress(Exception):
                self._zeroconf.unregister_service(self._info)
                self._zeroconf.close()
            self._zeroconf = None
        if self._responder is not None:
            self._responder.stop()
            self._responder = None


def _encode_name(name: str) -> bytes:
    out = bytearray()
    for label in name.rstrip(".").split("."):
        raw = label.encode("utf-8")
        if not 0 < len(raw) < 64:
            raise ValueError(f"illegal DNS label {label!r}")
        out.append(len(raw))
        out += raw
    out.append(0)
    return bytes(out)


def _record(
    name: str, rtype: int, rdata: bytes, ttl: int, *, flush: bool = True
) -> bytes:
    rclass = 0x8001 if flush else 0x0001  # cache-flush bit + IN
    return (
        _encode_name(name)
        + struct.pack("!HHIH", rtype, rclass, ttl, len(rdata))
        + rdata
    )


class _MdnsResponder:
    """A deliberately minimal multicast-DNS responder for one service."""

    _TYPE_A = 1
    _TYPE_PTR = 12
    _TYPE_TXT = 16
    _TYPE_SRV = 33
    _TYPE_ANY = 255
    _TTL = 120

    def __init__(self, advert: MdnsAdvertiser) -> None:
        self._advert = advert
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            sock.bind(("", _MDNS_PORT))
            membership = socket.inet_aton(_MDNS_GROUP) + socket.inet_aton("0.0.0.0")
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
            sock.settimeout(0.5)
        except OSError as exc:
            # Not fatal: the operator can always connect by address. But they
            # need to know discovery is off, because "the app cannot see the
            # car" is otherwise a twenty-minute detour.
            LOG.warning(
                "mDNS is unavailable (%s); connect to %s:%d by address instead",
                exc,
                self._advert.address,
                self._advert.port,
            )
            return
        self._sock = sock
        self._thread = threading.Thread(target=self._run, name="sim-mdns", daemon=True)
        self._thread.start()
        LOG.info(
            "advertising %s at %s:%d (built-in responder)",
            self._advert.instance,
            self._advert.address,
            self._advert.port,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._send(self._announcement(ttl=0))  # goodbye
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    # -- records --------------------------------------------------------------

    def _txt_rdata(self) -> bytes:
        out = bytearray()
        for key, value in self._advert.properties.items():
            entry = f"{key}={value}".encode()[:255]
            out.append(len(entry))
            out += entry
        if not out:
            out.append(0)
        return bytes(out)

    def _srv_rdata(self) -> bytes:
        return struct.pack("!HHH", 0, 0, self._advert.port) + _encode_name(
            self._advert.hostname
        )

    def _answers(self, ttl: int) -> tuple[bytes, bytes, bytes, bytes]:
        advert = self._advert
        return (
            _record(
                MDNS_SERVICE_TYPE,
                self._TYPE_PTR,
                _encode_name(advert.instance),
                ttl,
                flush=False,
            ),
            _record(advert.instance, self._TYPE_SRV, self._srv_rdata(), ttl),
            _record(advert.instance, self._TYPE_TXT, self._txt_rdata(), ttl),
            _record(
                advert.hostname, self._TYPE_A, socket.inet_aton(advert.address), ttl
            ),
        )

    def _announcement(self, ttl: int = _TTL) -> bytes:
        ptr, srv, txt, a = self._answers(ttl)
        header = struct.pack("!HHHHHH", 0, 0x8400, 0, 4, 0, 0)
        return header + ptr + srv + txt + a

    def _send(self, payload: bytes, target: tuple[str, int] | None = None) -> None:
        if self._sock is None:
            return
        self._sock.sendto(payload, target or (_MDNS_GROUP, _MDNS_PORT))

    # -- loop -----------------------------------------------------------------

    def _run(self) -> None:
        # Announce three times with increasing spacing, as RFC 6762 asks, so a
        # browser that was already listening notices without waiting for its
        # own query to be due.
        announced = 0
        next_announce = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if announced < 3 and now >= next_announce:
                announced += 1
                next_announce = now + announced
                try:
                    self._send(self._announcement())
                except OSError:
                    pass
            try:
                assert self._sock is not None
                data, addr = self._sock.recvfrom(2048)
            except TimeoutError:
                continue
            except OSError:
                return
            try:
                self._handle_query(data, addr)
            except (struct.error, ValueError, IndexError):
                # Malformed multicast traffic from anything on the LAN must not
                # take the responder down.
                continue

    def _handle_query(self, data: bytes, addr: tuple[str, int]) -> None:
        if len(data) < 12:
            return
        query_id, flags, qdcount = struct.unpack_from("!HHH", data, 0)
        if flags & 0x8000 or qdcount == 0:
            return  # a response, not a question

        advert = self._advert
        offset = 12
        wanted_ptr = False
        wanted_service = False
        wanted_host = False
        unicast = False
        for _ in range(qdcount):
            name, offset = _read_name(data, offset)
            if offset + 4 > len(data):
                return
            qtype, qclass = struct.unpack_from("!HH", data, offset)
            offset += 4
            unicast = unicast or bool(qclass & 0x8000)
            lowered = name.lower()
            if lowered == MDNS_SERVICE_TYPE.lower() and qtype in (
                self._TYPE_PTR,
                self._TYPE_ANY,
            ):
                wanted_ptr = True
            elif lowered == advert.instance.lower() and qtype in (
                self._TYPE_SRV,
                self._TYPE_TXT,
                self._TYPE_ANY,
            ):
                wanted_service = True
            elif lowered == advert.hostname.lower() and qtype in (
                self._TYPE_A,
                self._TYPE_ANY,
            ):
                wanted_host = True

        if not (wanted_ptr or wanted_service or wanted_host):
            return

        ptr, srv, txt, a = self._answers(self._TTL)
        if wanted_ptr:
            answers = [ptr]
            additional = [srv, txt, a]
        elif wanted_service:
            answers = [srv, txt]
            additional = [a]
        else:
            answers = [a]
            additional = []
        header = struct.pack(
            "!HHHHHH", query_id, 0x8400, 0, len(answers), 0, len(additional)
        )
        payload = header + b"".join(answers) + b"".join(additional)
        try:
            self._send(payload, addr if unicast else None)
        except OSError:
            pass


def _read_name(data: bytes, offset: int) -> tuple[str, int]:
    """Read a DNS name. Compression pointers are followed once, non-recursively."""
    labels: list[str] = []
    jumped = False
    end = offset
    guard = 0
    while True:
        guard += 1
        if guard > 128 or offset >= len(data):
            raise ValueError("malformed DNS name")
        length = data[offset]
        if length == 0:
            offset += 1
            if not jumped:
                end = offset
            break
        if length & 0xC0 == 0xC0:
            if offset + 1 >= len(data):
                raise ValueError("truncated DNS pointer")
            pointer = ((length & 0x3F) << 8) | data[offset + 1]
            if not jumped:
                end = offset + 2
            if jumped or pointer >= offset:
                raise ValueError("illegal DNS compression pointer")
            jumped = True
            offset = pointer
            continue
        offset += 1
        labels.append(data[offset : offset + length].decode("utf-8", "replace"))
        offset += length
    return (".".join(labels) + ".", end)


__all__ = [
    "DelayLine",
    "MdnsAdvertiser",
    "NetworkOptions",
    "ServerOptions",
    "SessionConnection",
    "SessionState",
    "SimServer",
    "SimStats",
    "detect_local_ip",
]
