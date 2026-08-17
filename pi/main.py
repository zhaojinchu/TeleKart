#!/usr/bin/env python3
"""TeleKart2 Pi-side control loop.

Receives ``cmd`` over UDP, runs the shared ``vehicle`` model at 100 Hz, drives the
hardware through ``car``, and answers with ``tlm`` at 20 Hz. Video runs in its own
thread inside ``video``.

Plain ``while`` + ``time.sleep``. asyncio would buy nothing here: there is one loop, one
socket, and the only concurrent thing in the process is the camera server, which is
already a thread.

Usage::

    python3 main.py                # normal
    python3 main.py --no-camera    # control only, for bench bring-up
"""

from __future__ import annotations

import json
import os
import signal
import socket
import sys
import time

import config
import protocol
import vehicle
from car import Car
from video import VideoServer


class SessionLog:
    """Append-only jsonl of every message in and out.

    Each line wraps the message rather than merging into it: ``tlm`` already has its own
    ``dir`` field (the motor direction), so a flat merge would collide with the rx/tx
    ``dir`` key. ``t_local`` is this machine's clock at receive/send time -- paired with
    the Mac's own log of the same conversation, that is what turns "it worked on the Mac
    but the car didn't move" into a five-second diagnosis.
    """

    def __init__(self, directory: str) -> None:
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, f"session-{time.strftime('%Y%m%d-%H%M%S')}.jsonl")
        # Line buffered: if the Pi loses power mid-drive, everything up to the last
        # completed line survives, which is precisely the run you want to look at.
        self._fh = open(self.path, "a", buffering=1)

    def record(self, direction: str, msg: dict) -> None:
        line = json.dumps(
            {"dir": direction, "t_local": time.time(), "msg": msg}, separators=(",", ":")
        )
        try:
            self._fh.write(line + "\n")
        except OSError:
            pass  # a full disk must not stop the car from being drivable

    def close(self) -> None:
        try:
            self._fh.flush()
            self._fh.close()
        except OSError:
            pass


class RxStats:
    """Receive-side counters for telemetry: rate, inter-arrival gap, lost sequences."""

    def __init__(self) -> None:
        self.drops = 0
        self.last_seq: int | None = None
        self.last_rx: float | None = None
        self.last_gap: float = 0.0
        self._recent: list[float] = []
        self._first: float | None = None

    #: How far below the high-water mark a sequence number may fall and still be read as
    #: ordinary reordering. Anything further back is a sender that restarted counting.
    RESTART_WINDOW = 100

    def new_session(self) -> None:
        """Forget sequence history. Called when the peer address changes."""
        self.last_seq = None

    def note(self, seq: int, now: float) -> None:
        if self._first is None:
            self._first = now
        if self.last_rx is not None:
            self.last_gap = now - self.last_rx
        self.last_rx = now
        self._recent.append(now)

        if self.last_seq is None:
            self.last_seq = seq
            return

        if seq > self.last_seq + 1:
            self.drops += seq - self.last_seq - 1  # gap: datagrams lost in flight
            self.last_seq = seq
        elif seq <= self.last_seq - self.RESTART_WINDOW:
            # The sender restarted and began counting from 1 again. Without this the
            # high-water mark stays pinned above every new sequence number, so *every*
            # packet scores as a duplicate and drops climbs at the full send rate
            # forever -- turning the one number that reports link health into noise.
            self.last_seq = seq
        elif seq <= self.last_seq:
            self.drops += 1  # reordered or duplicated; just as much a link problem
        else:
            # Track the highest seq seen, so one late straggler does not make every
            # subsequent packet look like a gap.
            self.last_seq = seq

    def rate_hz(self, now: float) -> float:
        """Datagrams accepted over the trailing second.

        Counted against the window rather than against the span between the first and
        last arrival: when a backlog is drained they all share one tick's timestamp, and
        a span-based rate would divide by zero and report 0 Hz mid-flood. Pruning here
        rather than only in ``note`` is what makes the number fall to 0 when the Mac
        goes away, instead of freezing at the last healthy rate.
        """
        cutoff = now - 1.0
        while self._recent and self._recent[0] < cutoff:
            self._recent.pop(0)
        if not self._recent or self._first is None:
            return 0.0
        span = min(1.0, now - self._first)
        return 0.0 if span <= 0 else len(self._recent) / span

    def gap_ms(self, now: float) -> float:
        """Inter-arrival gap, floored at the age of the newest packet.

        In steady state this is the real gap between datagrams; when the link stalls it
        keeps growing instead of freezing at whatever the last healthy gap happened to
        be, so the HUD shows the stall rather than hiding it.
        """
        if self.last_rx is None:
            return 0.0
        return max(self.last_gap, now - self.last_rx) * 1000.0


_stop = False


def _request_stop(signum, frame) -> None:
    """Signal handler: set a flag, nothing more.

    Touching pigpio or the socket from inside a handler means re-entering code that the
    interrupted main thread may be halfway through. The loop notices within 10 ms and
    the ``finally`` block does the real shutdown.
    """
    global _stop
    _stop = True


def main() -> int:
    no_camera = "--no-camera" in sys.argv

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)

    log = SessionLog(config.LOG_DIR)
    print(f"[telekart2] logging to {log.path}")

    car = Car()  # raises with a clear message if pigpiod is not running
    print("[telekart2] gpio ready, all bridge pins low")

    video: VideoServer | None = None
    if not no_camera:
        video = VideoServer()
        try:
            video.start()
            print(f"[telekart2] video on :{config.VIDEO_PORT}{protocol.STREAM_PATH}")
        except Exception as exc:
            # A missing or busy camera is not a reason to refuse to drive; the driver
            # can still bench-test motors and steering.
            print(f"[telekart2] camera unavailable ({exc}); continuing without video")
            video = None
    else:
        print("[telekart2] --no-camera: control only")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((config.BIND_HOST, config.CONTROL_PORT))
    sock.setblocking(False)
    print(f"[telekart2] control on udp/{config.CONTROL_PORT}, 100 Hz")

    veh = vehicle.Vehicle(config.vehicle_config())
    rx = RxStats()

    peer: tuple[str, int] | None = None
    ack_seq = 0
    ack_ts = 0.0
    tlm_seq = 0
    cpu_temp: float | None = None

    t0 = time.monotonic()
    next_tick = time.monotonic()
    next_tlm = 0.0
    next_summary = 0.0

    try:
        while not _stop:
            now = time.monotonic()
            uptime = now - t0

            # --- drain the socket -------------------------------------------------
            # Take every queued datagram and keep only the newest. A backlog means we
            # fell behind, and the old commands in it describe a steering wheel and a
            # throttle that have already moved on -- acting on stale input is worse
            # than dropping it.
            newest: dict | None = None
            while True:
                try:
                    data, addr = sock.recvfrom(protocol.MAX_DATAGRAM)
                except BlockingIOError:
                    break
                except OSError:
                    break
                msg = protocol.decode(data)
                if msg is None or msg.get("t") != protocol.MSG_CMD:
                    continue
                log.record("rx", msg)
                if addr != peer:
                    # A restarted Mac app gets a fresh ephemeral port, so a changed peer
                    # is the reliable signal that sequence numbering began again.
                    rx.new_session()
                peer = addr
                rx.note(int(msg.get("seq", 0)), now)
                # Newest by sequence, not by arrival order: UDP can reorder.
                if newest is None or int(msg.get("seq", 0)) >= int(newest.get("seq", 0)):
                    newest = msg

            if newest is not None:
                ack_seq = int(newest.get("seq", 0))
                ack_ts = float(newest.get("ts", 0.0))

            # --- e-stop -----------------------------------------------------------
            # Off until the button's polarity is confirmed on the bench (config.py).
            estop = config.ESTOP_ENABLED and car.read_estop()
            if estop and newest is not None:
                newest = dict(newest, arm=False)  # keep the model's state consistent

            # --- model -> hardware ------------------------------------------------
            out = veh.update(newest, now)
            if estop:
                # Belt and braces: the model has already been told it is disarmed, but
                # a physical stop button must not depend on model state to take effect.
                out.armed = False
                out.throttle_out = 0.0
                out.direction = protocol.DIR_COAST
                out.duty = 0.0
            car.apply(out)
            car.set_servo_us(out.servo_us)
            car.set_led(_led_state(out.state, now))

            # --- telemetry --------------------------------------------------------
            # Answer each command as it arrives rather than on a timer. On a fixed tick the
            # echoed ack_ts belonged to whichever command happened to be newest when the
            # tick fired, which added a uniform 0-20 ms of waiting to every RTT the driver
            # sees -- latency we invented and then reported as if it were the network's.
            # Replying on receipt makes the number mean what it claims to.
            #
            # The heartbeat covers the other case: no commands means nothing to answer, but
            # the Mac still needs to see the car sitting in FAILSAFE rather than nothing.
            if peer is not None and (newest is not None or now >= next_tlm):
                next_tlm = now + 1.0 / config.TLM_HEARTBEAT_HZ
                tlm_seq += 1
                tlm = protocol.make_tlm(
                    seq=tlm_seq,
                    ack_seq=ack_seq,
                    ack_ts=ack_ts,
                    armed=out.armed,
                    state=out.state,
                    throttle_cmd=out.throttle_cmd,
                    throttle_out=out.throttle_out,
                    steer_cmd=out.steer_cmd,
                    servo_us=out.servo_us,
                    duty_pct=out.duty_pct,
                    direction=out.direction,
                    rx_rate_hz=rx.rate_hz(now),
                    rx_gap_ms=rx.gap_ms(now),
                    drops=rx.drops,
                    cam_fps=(video.fps if video is not None else 0.0),
                    cpu_temp=cpu_temp,
                    uptime=uptime,
                )
                try:
                    sock.sendto(protocol.encode(tlm), peer)
                    log.record("tx", tlm)
                except OSError:
                    pass  # the Mac went away; the watchdog handles the consequences

            # --- 1 Hz human-readable summary --------------------------------------
            if now >= next_summary:
                next_summary = now + config.SUMMARY_PERIOD_S
                cpu_temp = car.cpu_temp()
                print(
                    f"[{uptime:7.1f}s] {out.state:8s} "
                    f"thr {out.throttle_cmd:+.2f}->{out.throttle_out:+.2f} "
                    f"duty {out.duty_pct:5.1f}% {out.direction:5s} "
                    f"servo {out.servo_us:4d}us  "
                    f"rx {rx.rate_hz(now):5.1f}Hz drops {rx.drops:4d}  "
                    f"cam {(video.fps if video is not None else 0.0):4.1f}fps"
                    + (f"  cpu {cpu_temp:.1f}C" if cpu_temp is not None else "")
                    + ("  ESTOP" if estop else ""),
                    flush=True,
                )

            # --- pace the loop ----------------------------------------------------
            next_tick += config.CONTROL_PERIOD_S
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # Fell behind (a long log write, a scheduler hiccup). Resync instead of
                # trying to catch up, which would spin the loop with no sleep at all.
                next_tick = time.monotonic()
    finally:
        # Runs on SIGINT/SIGTERM, on an exception, and on a normal exit. Order matters:
        # motors dead first, then everything else.
        try:
            car.coast()
            car.set_servo_us(0)
        except Exception:
            pass
        car.close()
        if video is not None:
            video.stop()
        try:
            sock.close()
        except OSError:
            pass
        log.close()
        print("\n[telekart2] stopped: coasting, servo off, gpio released", flush=True)

    return 0


def _led_state(state: str, now: float) -> bool:
    """Heartbeat when SAFE, solid when DRIVE, off otherwise.

    Readable from across a room, which is the only interface the car has when the Mac
    is not in front of you.
    """
    if state == protocol.STATE_DRIVE:
        return True
    if state == protocol.STATE_SAFE:
        return (now % 1.0) < 0.1  # short 1 Hz flash: alive, not armed
    return False


if __name__ == "__main__":
    sys.exit(main())
