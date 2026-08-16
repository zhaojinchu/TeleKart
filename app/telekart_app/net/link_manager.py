"""The connection state machine, and the owner of every network thread."""

from __future__ import annotations

import threading
import time
from dataclasses import replace
from typing import Any, Mapping

from telekart_protocol import PROTO_VERSION, MsgType, VehicleState, VideoCodec

from .. import __version__
from ..config.settings import Settings
from ..core.latest_box import LatestBox
from ..core.log import get_logger, install_thread_excepthook
from ..core.paced_loop import DEFAULT_SWITCH_INTERVAL, PacedLoop, tune_thread_switching
from ..model.snapshots import (
    TELEMETRY_STALE_S,
    VIDEO_STALE_S,
    InputSnapshot,
    LinkSnapshot,
    LinkState,
    SessionSnapshot,
)
from ..storage.recorder import SessionRecorder
from . import discovery
from .control_tx import ControlCommandLike, ControlTxThread
from .session_client import SessionClient, SessionEvent, SessionEventKind, SessionInfo
from .telemetry_rx import TelemetryRxThread, TelemetrySample

_log = get_logger(__name__)

# --------------------------------------------------------------------------
# The degraded-mode matrix. TELEMETRY_STALE_S and VIDEO_STALE_S are defined in
# model.snapshots so the model and the state machine cannot drift apart; note
# that both concern whether the *display* is trustworthy. The car's own 200 ms
# control timeout is a separate, stricter mechanism that runs on the car and
# does not depend on this app being correct.
# --------------------------------------------------------------------------

#: Supervisor rate. Fast enough that a stale link is flagged within a frame of
#: the HUD noticing, cheap enough to be irrelevant.
SUPERVISOR_HZ = 20.0

#: Age at which the *session* channel is considered gone even though TCP has
#: not said so. The session client's own ping timeout normally gets there
#: first; this is the backstop for a client thread that died.
SESSION_STALE_S = 10.0

#: Grace period after the video target is set, before "no frames yet" counts as
#: a degraded link. picamera2 plus the hardware encoder take a second or two to
#: produce a first keyframe, and a warning that always fires on connect is a
#: warning the driver learns to ignore.
VIDEO_START_GRACE_S = 4.0


class LinkManager:
    """Connects, supervises and tears down the link to one car.

    Owns four channel threads -- session (TCP/JSON), control TX (100 Hz UDP),
    telemetry RX (blocking UDP), video RX (TCP + decode) -- plus a small
    supervisor that runs the watchdogs.

    The watchdogs are deliberately per channel and independent, because the
    channels fail independently and mean different things when they do:

    ======================  ==================  ==========================
    Condition               Link state          Driving
    ======================  ==================  ==========================
    all four healthy        LIVE                allowed
    video stale or absent   DEGRADED            **allowed**
    telemetry stale         STALE               allowed, but nothing on the
                                                HUD can be trusted
    session dropped         RECONNECTING        no -- the key is dropped, so
                                                the TX thread stops sending
    auth or version refused FAILED              no, and no retry
    ======================  ==================  ==========================

    Video loss must not stop the car. A driver who has lost the picture still
    needs to be able to brake and steer the last few feet, and cutting power at
    that moment strands the car wherever it happens to be. Telemetry loss is
    the opposite: it is safety-critical, because everything the HUD asserts
    becomes a guess. Session loss is decisive -- the firmware already treats a
    dropped TCP connection as an E-stop condition, so the app stops
    transmitting rather than pretending otherwise.
    """

    def __init__(
        self,
        settings: Settings,
        *,
        recorder: SessionRecorder | None = None,
        app_version: str = __version__,
    ) -> None:
        self._settings = settings
        self._recorder = recorder
        self._app_version = app_version

        # Written by the input thread, read by the control TX thread.
        self.command_box: LatestBox[ControlCommandLike] = LatestBox()
        self.telemetry_box: LatestBox[TelemetrySample] = LatestBox()
        self.input_box: LatestBox[InputSnapshot] = LatestBox()
        self.link_box: LatestBox[LinkSnapshot] = LatestBox()
        self.session_box: LatestBox[SessionSnapshot] = LatestBox()
        # Typed loosely: FrameBundle lives behind PyAV and QtGui, and this
        # module must stay importable without either.
        self.video_box: LatestBox[Any] = LatestBox()

        self._lock = threading.RLock()
        self._state = LinkState.OFFLINE
        self._detail = ""
        self._host = ""
        self._address = ""
        self._shared_key = ""
        self._driver = settings.driver
        self._info: SessionInfo | None = None
        self._session_started = 0.0
        self._last_error = ""
        self._last_error_code = ""
        self._vehicle_state = VehicleState.BOOT

        self._session: SessionClient | None = None
        self._video: Any = None
        self._video_error = ""
        self._video_since = 0.0
        self._connect_thread: threading.Thread | None = None

        self._telemetry = TelemetryRxThread(
            self.telemetry_box,
            port=settings.link.telemetry_port,
            recorder=recorder,
            rtt_lookup=self._rtt_lookup,
        )
        self._control = ControlTxThread(
            self.command_box,
            self.input_box,
            burst=settings.link.estop_burst,
            burst_gap=settings.link.estop_burst_gap,
            tcp_estop=self._tcp_estop,
        )
        self._shutdown = threading.Event()
        self._supervisor = threading.Thread(
            target=self._supervise, name="LinkSupervisor", daemon=True
        )
        self._threads_started = False

    # -- public state -------------------------------------------------------

    @property
    def state(self) -> LinkState:
        return self._state

    @property
    def session_info(self) -> SessionInfo | None:
        return self._info

    @property
    def connected(self) -> bool:
        return self._state.usable

    def params(self) -> Mapping[str, Any]:
        session = self._session
        return session.params() if session is not None else {}

    # -- lifecycle ----------------------------------------------------------

    def connect(self, host: str = "", *, shared_key: str = "", driver: str = "") -> None:
        """Resolve and connect. Returns immediately; progress arrives as state.

        Resolution runs on its own thread because ``getaddrinfo`` for a car
        that is switched off blocks for seconds, and this is called straight
        from a button handler on the GUI thread.
        """
        with self._lock:
            if self._connect_thread is not None and self._connect_thread.is_alive():
                _log.info("connect already in progress")
                return
            self._shared_key = shared_key or self._shared_key
            if not self._shared_key:
                raise ValueError("a shared key is required to connect")
            self._driver = driver or self._driver or "driver"
            self._host = host or self._settings.link.host
            self._shutdown.clear()
            self._set_state(LinkState.DISCOVERING, self._host or discovery.DEFAULT_HOSTNAME)
            self._connect_thread = threading.Thread(
                target=self._connect_worker, name="LinkConnect", daemon=True
            )
            self._connect_thread.start()

    def _connect_worker(self) -> None:
        link = self._settings.link
        car = discovery.find_car(self._host, timeout=link.discovery_timeout)
        if car is None:
            self._set_state(
                LinkState.FAILED,
                f"no car at {self._host or discovery.DEFAULT_HOSTNAME}",
            )
            return
        self._address = car.address
        if not self._host:
            self._host = car.host or car.address

        try:
            port = self._telemetry.bind()
        except OSError as exc:
            self._set_state(LinkState.FAILED, f"cannot bind a telemetry port: {exc}")
            return

        if not self._threads_started:
            # Do this before the paced threads exist, not after: with CPython's
            # default 5 ms GIL handoff the 100 Hz control stream measures 56 Hz
            # under a CPU-bound repaint. See core.paced_loop for the numbers.
            previous = tune_thread_switching()
            _log.info(
                "GIL switch interval %.1f ms -> %.1f ms for the paced threads",
                previous * 1000.0,
                DEFAULT_SWITCH_INTERVAL * 1000.0,
            )
            # A worker thread that dies prints to stderr and vanishes. The
            # symptom is a car that failsafes for no visible reason, so the
            # traceback has to reach the log file.
            install_thread_excepthook()
            self._telemetry.start()
            self._control.start()
            self._supervisor.start()
            self._threads_started = True

        try:
            session = SessionClient(
                host=self._host,
                address=car.address,
                port=car.port,
                shared_key=self._shared_key,
                driver=self._driver,
                telemetry_port=port,
                on_event=self._on_session_event,
                connect_timeout=link.connect_timeout,
                min_delay=link.reconnect_min_delay,
                max_delay=link.reconnect_max_delay,
                ping_interval=link.ping_interval,
                ping_timeout=link.ping_timeout,
                app_version=self._app_version,
            )
        except ValueError as exc:
            self._set_state(LinkState.FAILED, str(exc))
            return

        with self._lock:
            old, self._session = self._session, session
        if old is not None:
            old.close()
        self._set_state(LinkState.CONNECTING, car.address)
        session.start()

    def disconnect(self) -> None:
        """Tear the link down cleanly. The recording is closed with it.

        The threads keep running so a later ``connect`` works; only ``close``
        is terminal.
        """
        # State first: closing the session makes its thread emit DISCONNECTED,
        # and that handler must be able to tell an operator-requested teardown
        # from a link that fell over and should reconnect.
        self._set_state(LinkState.OFFLINE, "disconnected")
        with self._lock:
            session, self._session = self._session, None
        if session is not None:
            session.close()
        self._control.clear_target()
        self._control.set_arm_intent(False)
        self._telemetry.clear_session()
        self._stop_video()
        self.video_box.clear()
        if self._recorder is not None and self._recorder.active:
            self._recorder.record_event("disconnect", "operator")
            self._recorder.stop()
        with self._lock:
            self._info = None
            self._session_started = 0.0

    def close(self) -> None:
        """Stop every thread. Idempotent."""
        self._shutdown.set()
        self.disconnect()
        self._control.close()
        self._telemetry.close()
        if self._supervisor.is_alive():
            self._supervisor.join(timeout=2.0)
        for thread in (self._telemetry, self._control):
            if thread.is_alive():
                thread.join(timeout=2.0)

    # -- commands (the VehicleController seam AppModel forwards to) ---------

    def arm(self) -> None:
        """Request arming. Never asserts that the car *is* armed.

        The arm intent flag on the control stream is the continuing half of the
        request; the car decides, and telemetry reports what it decided.
        """
        self._control.clear_estop()
        self._control.set_arm_intent(True)
        self._send(MsgType.ARM)
        self._event("arm_request")

    def disarm(self) -> None:
        self._control.set_arm_intent(False)
        self._send(MsgType.DISARM)
        self._event("disarm_request")

    def estop(self) -> None:
        """E-stop over both channels at once. See ControlTxThread.estop."""
        self._control.set_arm_intent(False)
        self._control.estop()
        self._event("estop")
        _log.warning("E-STOP requested by the operator")

    def clear_estop(self) -> None:
        self._control.clear_estop()
        self._send(MsgType.CLEAR_ESTOP)
        self._event("clear_estop")

    def clear_faults(self) -> None:
        self._send(MsgType.CLEAR_FAULTS)
        self._event("clear_faults")

    def reset_odom(self) -> None:
        self._send(MsgType.RESET_ODOM)

    def request_params(self, group: str = "") -> None:
        session = self._session
        if session is not None:
            session.get_params(group)

    def set_params(self, values: Mapping[str, Any]) -> int:
        session = self._session
        if session is None:
            raise RuntimeError("not connected: cannot push parameters")
        return session.set_params(values)

    def calibrate(self, routine: str = "drive", *, on_ground: bool = False) -> None:
        session = self._session
        if session is not None:
            session.calibrate(routine, on_ground=on_ground)

    def set_input_device(self, name: str, connected: bool) -> None:
        """Told to the TX thread so the echoed InputSnapshot carries it."""
        self._control.set_input_device(name, connected)

    def set_video_display_size(self, width: int, height: int) -> None:
        video = self._video
        if video is not None:
            video.set_display_size(width, height)
        self._settings.video.target_width = width
        self._settings.video.target_height = height

    def set_video_enabled(self, enabled: bool) -> None:
        self._settings.video.enabled = enabled
        if not enabled:
            self._stop_video()
            self.video_box.clear()
        elif self._info is not None:
            self._start_video(self._info)

    def _send(self, msg_type: MsgType) -> None:
        session = self._session
        if session is None:
            _log.info("ignoring %s: not connected", msg_type.value)
            return
        session.request(msg_type)

    def _tcp_estop(self) -> None:
        session = self._session
        if session is not None:
            session.send_estop()

    def _rtt_lookup(self, sequence: int) -> float | None:
        return self._control.rtt_for(sequence)

    def _event(self, kind: str, detail: str = "") -> None:
        if self._recorder is not None:
            self._recorder.record_event(kind, detail)

    # -- session events (session thread) ------------------------------------

    def _on_session_event(self, event: SessionEvent) -> None:
        kind = event.kind
        if kind is SessionEventKind.CONNECTING:
            if self._state is not LinkState.RECONNECTING:
                self._set_state(LinkState.CONNECTING, event.detail)
        elif kind is SessionEventKind.CONNECTED and event.info is not None:
            self._on_connected(event.info)
        elif kind is SessionEventKind.DISCONNECTED:
            self._on_disconnected(event.detail)
        elif kind is SessionEventKind.FAILED:
            with self._lock:
                self._last_error = event.detail
                self._last_error_code = event.code
            self._control.clear_target()
            self._telemetry.clear_session()
            self._stop_video()
            self._set_state(LinkState.FAILED, event.detail)
            _log.error("session refused: %s (%s)", event.detail, event.code)
        elif kind is SessionEventKind.ERROR:
            with self._lock:
                self._last_error = event.detail
                self._last_error_code = event.code
            self._event("car_error", f"{event.code}: {event.detail}")
        elif kind is SessionEventKind.STATE and event.message is not None:
            raw = event.message.data.get("state")
            if isinstance(raw, int):
                try:
                    self._vehicle_state = VehicleState(raw)
                except ValueError:
                    self._vehicle_state = VehicleState.FAULT

    def _on_connected(self, info: SessionInfo) -> None:
        with self._lock:
            self._info = info
            if self._session_started == 0.0:
                self._session_started = time.time()
        self._telemetry.set_session(info.session_id, info.udp_key)
        try:
            self._control.set_target(
                info.address, info.session_id, info.udp_key, port=info.control_port
            )
        except OSError as exc:
            # A control socket we cannot open is fatal to driving but not to
            # the session: telemetry and video still work, and the operator
            # needs to see why.
            self._set_state(LinkState.STALE, f"control socket: {exc}")
            _log.error("could not open the control socket: %s", exc)
            return

        if self._settings.video.enabled:
            self._start_video(info)

        if self._recorder is not None and self._settings.recording.enabled:
            if not self._recorder.active:
                self._recorder.start(
                    car_id=info.car_id,
                    driver=self._driver,
                    fw_version=info.fw_version,
                    proto_version=PROTO_VERSION,
                    session_id=info.session_id,
                )
            self._recorder.record_event(
                "connect", f"{info.car_id} session {info.session_id}"
            )

        self._set_state(LinkState.LIVE, "connected")
        _log.info(
            "session %d with %s (fw %s)%s",
            info.session_id,
            info.car_id or info.address,
            info.fw_version or "unknown",
            " [resumed]" if info.resumed else "",
        )

    def _on_disconnected(self, detail: str) -> None:
        # Dropping the key is what stops the TX thread: a session the car has
        # forgotten must not keep receiving authenticated packets, and the
        # firmware treats the lost TCP connection as an E-stop condition
        # regardless of what UDP is still arriving.
        self._control.clear_target()
        self._telemetry.clear_session()
        self._stop_video()
        self.video_box.clear()
        with self._lock:
            self._info = None
        if self._recorder is not None:
            self._recorder.record_event("disconnect", detail)
        if self._state is not LinkState.FAILED and self._state is not LinkState.OFFLINE:
            self._set_state(LinkState.RECONNECTING, detail)

    # -- video --------------------------------------------------------------

    def _start_video(self, info: SessionInfo) -> None:
        video = self._video
        if video is None:
            try:
                # Imported here, not at module scope: PyAV and QtGui are heavy
                # and optional, and a headless integration test drives control
                # and telemetry with neither installed.
                from ..video.receiver import VideoRxThread
            except Exception as exc:
                self._video_error = str(exc)
                _log.error("video unavailable (%s); driving is unaffected", exc)
                return
            codec = _codec_from_params(self._session)
            size = (
                (self._settings.video.target_width, self._settings.video.target_height)
                if self._settings.video.target_width > 0
                else None
            )
            video = VideoRxThread(self.video_box, codec=codec, target_size=size)
            self._video = video
            video.start()
        self._video_since = time.perf_counter()
        video.set_target(info.address, info.video_port)

    def _stop_video(self) -> None:
        self._video_since = 0.0
        video = self._video
        if video is not None:
            video.clear_target()

    # -- supervisor ---------------------------------------------------------

    def _supervise(self) -> None:
        loop = PacedLoop(1.0 / SUPERVISOR_HZ, name="LinkSupervisor")
        loop.start()
        while not self._shutdown.is_set():
            try:
                self._evaluate()
            except Exception:
                # The watchdog is the last thing that may die quietly.
                _log.exception("link supervisor iteration failed")
            if loop.sleep_until_next(self._shutdown):
                break
        _log.info("link supervisor exiting")

    def _evaluate(self) -> None:
        now = time.perf_counter()
        telemetry = self._telemetry.stats()
        control = self._control.stats()
        session = self._session
        info = self._info

        telemetry_age = (now - telemetry.last_recv_t) if telemetry.last_recv_t else 0.0
        session_ok = session is not None and session.connected
        session_age = 0.0
        if session is not None and session.last_rx:
            session_age = now - session.last_rx

        video = self._video
        video_stats = video.stats() if video is not None else None
        video_age = 0.0
        if video_stats is not None and video_stats.last_frame_t:
            video_age = now - video_stats.last_frame_t
        video_wanted = self._settings.video.enabled and info is not None
        # Health is decided by frame recency ALONE, never by socket state.
        # A socket that is up and delivering nothing is not working video, and
        # -- the reason this is written down -- a video process that has died
        # leaves the receiver reconnecting every second, so `connected` flaps
        # true/false several times a second. Feeding that into the state
        # machine made the HUD blink between LIVE and DEGRADED at 20 Hz until
        # the frame age finally crossed the threshold. Frame age is monotone,
        # so it cannot chatter.
        video_ok = bool(
            video_stats is not None
            and video_stats.last_frame_t
            and video_age <= VIDEO_STALE_S
        )
        if video_wanted and not video_ok and self._video_since:
            # The camera process takes a moment to come up after the handshake.
            # Flashing DEGRADED during a normal connect would teach the driver
            # to ignore the indicator, which is worse than not having one.
            starting = (
                video_stats is None or not video_stats.last_frame_t
            ) and now - self._video_since < VIDEO_START_GRACE_S
            if starting:
                video_ok = True

        state, detail = self._classify(
            session_ok=session_ok,
            session_age=session_age,
            has_telemetry=bool(telemetry.last_recv_t),
            telemetry_age=telemetry_age,
            video_wanted=video_wanted,
            video_ok=video_ok,
        )
        if state is not None:
            self._set_state(state, detail, publish=False)

        self.link_box.put(
            LinkSnapshot(
                state=self._state,
                detail=self._detail,
                host=self._host,
                address=info.address if info is not None else self._address,
                car_id=info.car_id if info is not None else "",
                session_id=info.session_id if info is not None else 0,
                telemetry_age=telemetry_age,
                video_age=video_age,
                session_age=session_age,
                rtt=telemetry.rtt,
                rtt_p95=telemetry.rtt_p95,
                rtt_min=telemetry.rtt_min,
                telemetry_hz=telemetry.rate_hz,
                control_hz=control.rate_hz,
                video_fps=video_stats.fps if video_stats else 0.0,
                video_bitrate=video_stats.bitrate if video_stats else 0.0,
                video_latency=video_stats.latency if video_stats else 0.0,
                loss=telemetry.loss,
                telemetry_packets=telemetry.packets,
                telemetry_lost=telemetry.lost,
                telemetry_bad=telemetry.bad,
                control_sent=control.sent,
                control_errors=control.errors,
                video_frames=video_stats.frames if video_stats else 0,
                video_dropped=video_stats.dropped if video_stats else 0,
                video_ok=video_ok,
                session_ok=session_ok,
                estop_latched=control.estop_latched,
            )
        )
        self._publish_session(session)
        self._record_input()

    def _classify(
        self,
        *,
        session_ok: bool,
        session_age: float,
        has_telemetry: bool,
        telemetry_age: float,
        video_wanted: bool,
        video_ok: bool,
    ) -> tuple[LinkState | None, str]:
        """The degraded-mode matrix, in one place and in priority order."""
        current = self._state
        if current in (LinkState.OFFLINE, LinkState.FAILED, LinkState.DISCOVERING):
            return None, ""
        if not session_ok:
            if current is LinkState.CONNECTING:
                return None, ""
            return LinkState.RECONNECTING, "session down"
        if session_age > SESSION_STALE_S:
            return LinkState.RECONNECTING, "session silent"
        if not has_telemetry:
            # Connected but the car has never sent a packet: the handshake
            # succeeded and the UDP path did not. Almost always a firewall.
            return LinkState.STALE, "no telemetry yet"
        if telemetry_age > TELEMETRY_STALE_S:
            return LinkState.STALE, "telemetry stale"
        if video_wanted and not video_ok:
            return LinkState.DEGRADED, self._video_error or "video down"
        return LinkState.LIVE, ""

    def _publish_session(self, session: SessionClient | None) -> None:
        info = self._info
        generation, pending = session.params_generation() if session else (0, 0)
        recorder = self._recorder
        self.session_box.put(
            SessionSnapshot(
                active=info is not None,
                session_id=info.session_id if info else 0,
                car_id=info.car_id if info else "",
                fw_version=info.fw_version if info else "",
                driver=self._driver,
                caps=info.caps if info else (),
                resumed=info.resumed if info else False,
                started_at=self._session_started,
                duration=(
                    time.time() - self._session_started if self._session_started else 0.0
                ),
                params=session.params() if session else {},
                params_generation=generation,
                params_pending=pending,
                recording=bool(recorder is not None and recorder.active),
                record_id=recorder.record_id if recorder is not None else None,
                last_error=self._last_error,
                last_error_code=self._last_error_code,
            )
        )

    def _record_input(self) -> None:
        recorder = self._recorder
        if recorder is None or not recorder.active:
            return
        snapshot = self.input_box.peek()
        if snapshot is not None:
            # peek, never take: AppModel is the box's single consumer, and
            # taking here would steal frames from the HUD.
            recorder.record_input(snapshot)

    # -- state --------------------------------------------------------------

    def _set_state(self, state: LinkState, detail: str = "", *, publish: bool = True) -> None:
        with self._lock:
            changed = state is not self._state
            self._state = state
            self._detail = detail
        if changed:
            _log.info("link state -> %s%s", state.value, f" ({detail})" if detail else "")
            if self._recorder is not None:
                self._recorder.record_event("link_state", f"{state.value} {detail}".strip())
        if publish and changed:
            self._publish_state_only()

    def _publish_state_only(self) -> None:
        """Publish a state-only update from a non-supervisor thread.

        The supervisor overwrites it within 50 ms with full statistics; this
        exists so a state change reaches the HUD on the frame it happened
        rather than up to a supervisor period later. Which matters: the
        transition into STALE is the one the driver has to see immediately.
        """
        previous = self.link_box.peek()
        if previous is None:
            self.link_box.put(LinkSnapshot(state=self._state, detail=self._detail))
        else:
            self.link_box.put(replace(previous, state=self._state, detail=self._detail))


def _codec_from_params(session: SessionClient | None) -> VideoCodec:
    """Use the car's own parameter value if it has told us one yet."""
    if session is None:
        return VideoCodec.H264
    value = session.params().get("video_codec")
    return VideoCodec.MJPEG if value == "mjpeg" else VideoCodec.H264
