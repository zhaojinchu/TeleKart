"""Session recording. On from the first drive, not bolted on later."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ..core.log import get_logger
from ..model.snapshots import InputSnapshot
from .db import Database

if TYPE_CHECKING:  # a runtime import here would make net <-> storage circular
    from ..net.telemetry_rx import TelemetrySample

_log = get_logger(__name__)

_TELEMETRY_SQL = (
    "INSERT INTO telemetry (session,t,seq,state,faults,flags,speed,v_max,rpm_l,rpm_r,"
    "rpm_tl,rpm_tr,duty_l,duty_r,steer,servo_us,x,y,heading,distance,slip,pack_v,cpu_c,rtt)"
    " VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)"
)
_INPUT_SQL = (
    "INSERT INTO inputs (session,t,seq,steering,throttle,brake,flags) VALUES (?,?,?,?,?,?,?)"
)
_EVENT_SQL = "INSERT INTO events (session,t,kind,detail) VALUES (?,?,?,?)"
_LAP_SQL = (
    "INSERT INTO laps (session,idx,t_start,t_end,duration,valid,source)"
    " VALUES (?,?,?,?,?,?,?)"
)

#: Commit every half second. Long enough that a session is a few hundred
#: transactions rather than tens of thousands; short enough that a crash costs
#: half a second of history.
_FLUSH_INTERVAL = 0.5
_FLUSH_ROWS = 256


@dataclass(frozen=True, slots=True)
class RecorderStats:
    active: bool = False
    record_id: int | None = None
    telemetry_rows: int = 0
    input_rows: int = 0
    dropped: int = 0
    queued: int = 0
    distance_m: float = 0.0
    max_speed_mps: float = 0.0
    errors: int = 0


class SessionRecorder:
    """Buffers rows on the producer's thread and writes them on its own.

    The critical property is that ``record_telemetry`` never blocks. It is
    called from the telemetry receive thread, and an fsync there would stall
    the very packets the HUD needs -- turning a slow disk into an apparent link
    fault. The queue is bounded and drops oldest-first on overflow, because a
    recording with a gap is still useful and a wedged receive thread is not.
    """

    def __init__(
        self,
        db: Database,
        *,
        telemetry_hz: float = 25.0,
        record_inputs: bool = True,
        queue_size: int = 8192,
        app_version: str = "",
    ) -> None:
        if telemetry_hz <= 0.0:
            raise ValueError("telemetry_hz must be positive")
        self._db = db
        self._interval = 1.0 / telemetry_hz
        self._record_inputs = record_inputs
        self._app_version = app_version

        self._queue: queue.Queue[tuple[str, tuple[Any, ...]] | None] = queue.Queue(
            maxsize=queue_size
        )
        self._writer: threading.Thread | None = None
        self._lock = threading.Lock()
        self._record_id: int | None = None
        self._active = False

        self._next_telemetry = 0.0
        self._next_input = 0.0
        self._telemetry_rows = 0
        self._input_rows = 0
        self._dropped = 0
        self._errors = 0
        self._distance = 0.0
        self._max_speed = 0.0

    # -- lifecycle ----------------------------------------------------------

    @property
    def active(self) -> bool:
        return self._active

    @property
    def record_id(self) -> int | None:
        return self._record_id

    def start(
        self,
        *,
        car_id: str = "",
        driver: str = "",
        fw_version: str = "",
        proto_version: int = 0,
        session_id: int = 0,
    ) -> int | None:
        """Open a session row and start the writer thread.

        Returns None if the database is unavailable. Recording failing must
        never stop the app from driving, so this is a warning, not an error.
        """
        with self._lock:
            if self._active:
                return self._record_id
            try:
                record_id = self._db.create_session(
                    car_id=car_id,
                    driver=driver,
                    fw_version=fw_version,
                    app_version=self._app_version,
                    proto_version=proto_version,
                    session_id=session_id,
                )
            except Exception as exc:
                _log.error("could not start a recording: %s", exc)
                return None
            self._record_id = record_id
            self._active = True
            self._telemetry_rows = 0
            self._input_rows = 0
            self._dropped = 0
            self._errors = 0
            self._distance = 0.0
            self._max_speed = 0.0
            self._next_telemetry = 0.0
            self._next_input = 0.0
            self._writer = threading.Thread(
                target=self._run, name="Recorder", daemon=True
            )
            self._writer.start()
        _log.info("recording session %d (%s)", record_id, car_id or "unknown car")
        self.record_event("session_start", car_id)
        return record_id

    def stop(self, *, laps: int = 0, best_lap: float | None = None) -> None:
        """Flush, close the session row, and stop the writer."""
        with self._lock:
            if not self._active:
                return
            record_id = self._record_id
            writer = self._writer
            self._active = False
        self._enqueue("event", (record_id, time.time(), "session_stop", ""))
        self._queue.put(None)
        if writer is not None:
            writer.join(timeout=5.0)
            if writer.is_alive():
                _log.warning("recorder writer did not stop in time")
        if record_id is not None:
            try:
                self._db.finish_session(
                    record_id,
                    samples=self._telemetry_rows,
                    distance_m=self._distance,
                    max_speed_mps=self._max_speed,
                    laps=laps,
                    best_lap=best_lap,
                )
            except Exception as exc:
                _log.error("could not close session %d: %s", record_id, exc)
        _log.info(
            "recording %s closed: %d telemetry rows, %d dropped",
            record_id,
            self._telemetry_rows,
            self._dropped,
        )
        self._record_id = None
        self._writer = None

    # -- producers ----------------------------------------------------------

    def record_telemetry(self, sample: "TelemetrySample") -> None:
        """Called from the telemetry receive thread. Must not block."""
        if not self._active:
            return
        # Decimate against the sample's own clock, not the wall clock: the
        # spacing then follows the packets rather than the recorder's timing.
        if sample.recv_t < self._next_telemetry:
            return
        self._next_telemetry = sample.recv_t + self._interval

        packet = sample.packet
        speed = packet.speed_mps
        if abs(speed) > self._max_speed:
            self._max_speed = abs(speed)
        if packet.distance_m > self._distance:
            self._distance = packet.distance_m

        x, y, heading = packet.pose_m
        self._enqueue(
            "telemetry",
            (
                self._record_id,
                sample.wall_t,
                packet.sequence,
                int(packet.state),
                int(packet.faults),
                int(packet.flags),
                speed,
                packet.v_max_mps,
                float(packet.rpm_l),
                float(packet.rpm_r),
                float(packet.rpm_target_l),
                float(packet.rpm_target_r),
                packet.duty_l_f,
                packet.duty_r_f,
                packet.steer_angle_deg,
                packet.servo_us,
                x,
                y,
                heading,
                packet.distance_m,
                packet.slip,
                packet.pack_volts,
                packet.cpu_temp_c,
                sample.rtt,
            ),
        )

    def record_input(self, snapshot: InputSnapshot, *, wall_t: float | None = None) -> None:
        if not self._active or not self._record_inputs:
            return
        now = snapshot.sent_at
        if now < self._next_input:
            return
        self._next_input = now + self._interval
        self._enqueue(
            "input",
            (
                self._record_id,
                wall_t if wall_t is not None else time.time(),
                snapshot.sequence,
                snapshot.steering,
                snapshot.throttle,
                snapshot.brake,
                int(snapshot.flags),
            ),
        )

    def record_event(self, kind: str, detail: str = "") -> None:
        """Arm, disarm, E-stop, fault, connect, disconnect.

        These are what make a recording readable six months later: a telemetry
        trace shows the car braking, the event log says the driver hit E-stop.
        """
        if not self._active:
            return
        self._enqueue("event", (self._record_id, time.time(), kind, detail))

    def record_lap(
        self,
        index: int,
        t_start: float,
        t_end: float,
        duration: float,
        *,
        valid: bool = True,
        source: str = "",
    ) -> None:
        if not self._active:
            return
        self._enqueue(
            "lap",
            (self._record_id, index, t_start, t_end, duration, 1 if valid else 0, source),
        )

    def _enqueue(self, kind: str, row: tuple[Any, ...]) -> None:
        try:
            self._queue.put_nowait((kind, row))
        except queue.Full:
            # Drop the oldest so the newest survives: a stall is transient and
            # what happened just now is the more interesting half.
            self._dropped += 1
            try:
                self._queue.get_nowait()
                self._queue.put_nowait((kind, row))
            except (queue.Empty, queue.Full):
                pass

    # -- writer -------------------------------------------------------------

    def _run(self) -> None:
        telemetry: list[tuple[Any, ...]] = []
        inputs: list[tuple[Any, ...]] = []
        events: list[tuple[Any, ...]] = []
        laps: list[tuple[Any, ...]] = []
        last_flush = time.monotonic()

        while True:
            timeout = max(0.05, _FLUSH_INTERVAL - (time.monotonic() - last_flush))
            try:
                item = self._queue.get(timeout=timeout)
            except queue.Empty:
                item = ("", ())
            if item is None:
                self._flush(telemetry, inputs, events, laps)
                return
            kind, row = item
            if kind == "telemetry":
                telemetry.append(row)
            elif kind == "input":
                inputs.append(row)
            elif kind == "event":
                events.append(row)
            elif kind == "lap":
                laps.append(row)

            pending = len(telemetry) + len(inputs) + len(events) + len(laps)
            if pending >= _FLUSH_ROWS or time.monotonic() - last_flush >= _FLUSH_INTERVAL:
                self._flush(telemetry, inputs, events, laps)
                last_flush = time.monotonic()

    def _flush(
        self,
        telemetry: list[tuple[Any, ...]],
        inputs: list[tuple[Any, ...]],
        events: list[tuple[Any, ...]],
        laps: list[tuple[Any, ...]],
    ) -> None:
        if not (telemetry or inputs or events or laps):
            return
        try:
            with self._db.transaction() as conn:
                if telemetry:
                    conn.executemany(_TELEMETRY_SQL, telemetry)
                if inputs:
                    conn.executemany(_INPUT_SQL, inputs)
                if events:
                    conn.executemany(_EVENT_SQL, events)
                if laps:
                    conn.executemany(_LAP_SQL, laps)
            self._telemetry_rows += len(telemetry)
            self._input_rows += len(inputs)
        except Exception as exc:
            # A failed write costs the batch, never the thread: the recorder
            # must keep trying, because a full disk that clears itself should
            # resume recording rather than need a restart.
            self._errors += 1
            _log.error("recorder write failed, dropping %d rows: %s", len(telemetry), exc)
        finally:
            telemetry.clear()
            inputs.clear()
            events.clear()
            laps.clear()

    # -- introspection ------------------------------------------------------

    def stats(self) -> RecorderStats:
        return RecorderStats(
            active=self._active,
            record_id=self._record_id,
            telemetry_rows=self._telemetry_rows,
            input_rows=self._input_rows,
            dropped=self._dropped,
            queued=self._queue.qsize(),
            distance_m=self._distance,
            max_speed_mps=self._max_speed,
            errors=self._errors,
        )
