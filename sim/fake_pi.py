#!/usr/bin/env python3
"""Simulated Pi: protocol-identical stand-in for the real vehicle, running on the Mac.

Same control loop rate, same UDP JSON, same MJPEG endpoint, and -- crucially -- the
*same* ``common/vehicle.py`` model the Pi runs. Nothing here reimplements ramping,
deadband or the watchdog; if it did, the simulator would agree with the car right up
until the moment it mattered.

    python sim/fake_pi.py
"""

from __future__ import annotations

import io
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # headless: no window, no display needed

import pygame  # noqa: E402

import protocol  # noqa: E402
from vehicle import Vehicle, VehicleConfig  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
LOG_DIR = REPO / "logs"

CAM_W, CAM_H = 640, 480
CAM_FPS = 40.0  # match the real camera (see pi/config.py)
LOOP_HZ = 100.0
CPU_TEMP_C = 48.0  # the real Pi reads /sys/class/thermal; here, a plausible constant

running = True


# --- frame store -------------------------------------------------------------


class FrameStore:
    """Newest JPEG only. Slow HTTP clients get skipped frames, never a backlog."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._jpg: bytes | None = None
        self._seq = 0

    def put(self, jpg: bytes) -> None:
        with self._cond:
            self._jpg = jpg
            self._seq += 1
            self._cond.notify_all()

    def wait_newer(self, seen: int, timeout: float = 2.0) -> tuple[int, bytes | None]:
        with self._cond:
            if self._seq == seen:
                self._cond.wait(timeout)
            return self._seq, self._jpg


frames = FrameStore()
state_lock = threading.Lock()
shared = {
    "state": protocol.STATE_INIT,
    "armed": False,
    "throttle_cmd": 0.0,
    "throttle_out": 0.0,
    "steer_cmd": 0.0,
    "duty_pct": 0.0,
    "dir": protocol.DIR_COAST,
    "servo_us": 0,
    "cam_fps": 0.0,
    "frames": 0,
    "uptime": 0.0,
}


def snapshot() -> dict:
    with state_lock:
        return dict(shared)


def publish(**kw) -> None:
    with state_lock:
        shared.update(kw)


# --- synthetic camera --------------------------------------------------------


def camera_thread(started: float) -> None:
    """Render a 640x480 scene at ~20 fps and JPEG-encode it.

    The scene is built for diagnosis, not looks: a bar sweeping across at a known rate
    makes glass-to-glass latency visible by eye, and the throttle/steer gauges prove the
    keypress actually reached the "car" rather than just the HUD.
    """
    surf = pygame.Surface((CAM_W, CAM_H))
    f_big = pygame.font.Font(None, 34)
    f_med = pygame.font.Font(None, 26)
    f_small = pygame.font.Font(None, 22)

    stamps: deque[float] = deque(maxlen=60)
    n = 0
    period = 1.0 / CAM_FPS
    next_t = time.monotonic()

    while running:
        now = time.monotonic()
        s = snapshot()
        n += 1

        surf.fill((16, 18, 24))

        # Grid, so the sweeping bar has something to be measured against.
        for x in range(0, CAM_W, 64):
            pygame.draw.line(surf, (34, 38, 48), (x, 0), (x, CAM_H))
        for y in range(0, CAM_H, 64):
            pygame.draw.line(surf, (34, 38, 48), (0, y), (CAM_W, y))

        # Sweeping bar: 2 s left-to-right-to-left. Eyeball the offset between this and
        # the same bar on the Mac window and you have the end-to-end video latency.
        phase = (now - started) % 2.0 / 2.0
        x = int((0.5 - 0.5 * math.cos(phase * 2 * math.pi)) * (CAM_W - 48))
        pygame.draw.rect(surf, (255, 196, 0), (x, 150, 48, 230))
        pygame.draw.rect(surf, (120, 92, 0), (x, 150, 48, 230), 2)

        wall = datetime.now().strftime("%H:%M:%S.") + f"{datetime.now().microsecond // 1000:03d}"
        surf.blit(f_big.render(f"SIM CAM  {wall}", True, (240, 240, 240)), (12, 10))
        surf.blit(
            f_med.render(f"frame {n}   {s['cam_fps']:.1f} fps", True, (170, 180, 200)), (12, 44)
        )

        armed = "ARMED" if s["armed"] else "DISARMED"
        colour = (80, 230, 120) if s["armed"] else (230, 190, 80)
        if s["state"] == protocol.STATE_FAILSAFE:
            colour = (240, 90, 90)
        surf.blit(f_big.render(f"{s['state']}  {armed}", True, colour), (12, 72))
        surf.blit(
            f_med.render(
                f"{s['dir']}  duty {s['duty_pct']:.0f}%  servo {s['servo_us']}us",
                True,
                (200, 205, 215),
            ),
            (12, 108),
        )

        # Throttle: outline is what was commanded, fill is what the slew limiter allows.
        bx, by, bw, bh = CAM_W - 90, 150, 44, 260
        mid = by + bh // 2
        pygame.draw.rect(surf, (60, 66, 80), (bx, by, bw, bh), 2)
        pygame.draw.line(surf, (90, 98, 115), (bx, mid), (bx + bw, mid))
        h_out = int(s["throttle_out"] * (bh // 2 - 2))
        if h_out:
            top = mid - h_out if h_out > 0 else mid
            pygame.draw.rect(surf, (90, 200, 255), (bx + 2, top, bw - 4, abs(h_out)))
        h_cmd = int(s["throttle_cmd"] * (bh // 2 - 2))
        pygame.draw.line(surf, (255, 255, 255), (bx, mid - h_cmd), (bx + bw, mid - h_cmd), 3)
        surf.blit(f_small.render("THR", True, (200, 205, 215)), (bx + 4, by - 22))
        surf.blit(
            f_small.render(f"{s['throttle_out']:+.2f}", True, (200, 205, 215)), (bx - 4, by + bh + 4)
        )

        # Steering.
        sx, sy, sw = 40, CAM_H - 60, CAM_W - 200
        pygame.draw.rect(surf, (60, 66, 80), (sx, sy, sw, 26), 2)
        pygame.draw.line(surf, (90, 98, 115), (sx + sw // 2, sy), (sx + sw // 2, sy + 26))
        knob = int(sx + sw / 2 + s["steer_cmd"] * (sw / 2 - 10))
        pygame.draw.rect(surf, (255, 140, 90), (knob - 8, sy + 2, 16, 22))
        surf.blit(f_small.render("STEER", True, (200, 205, 215)), (sx, sy - 22))
        surf.blit(f_small.render(f"{s['steer_cmd']:+.2f}", True, (200, 205, 215)), (sx + sw + 10, sy + 4))

        buf = io.BytesIO()
        pygame.image.save(surf, buf, "frame.jpg")
        frames.put(buf.getvalue())

        stamps.append(now)
        fps = 0.0
        if len(stamps) > 1 and stamps[-1] > stamps[0]:
            fps = (len(stamps) - 1) / (stamps[-1] - stamps[0])
        publish(cam_fps=fps, frames=n)

        next_t += period
        sleep = next_t - time.monotonic()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.monotonic()


# --- MJPEG HTTP server -------------------------------------------------------


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "telekart2-sim"

    def log_message(self, fmt, *args):  # keep stdout for the 1 Hz summary
        pass

    def handle_one_request(self):
        # A driving app that quits mid-stream resets the connection; that is normal
        # operation, not a traceback worth printing over the telemetry summary.
        try:
            super().handle_one_request()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            self.close_connection = True

    def do_GET(self):  # noqa: N802
        if self.path.startswith(protocol.HEALTH_PATH):
            self._health()
        elif self.path.startswith(protocol.STREAM_PATH):
            self._stream()
        else:
            self.send_error(404)

    def _health(self):
        s = snapshot()
        body = json.dumps(
            {
                "ok": True,
                "sim": True,
                "protocol": protocol.PROTOCOL_VERSION,
                "uptime": round(s["uptime"], 1),
                "cam_fps": round(s["cam_fps"], 1),
                "frames": s["frames"],
                "state": s["state"],
                "armed": s["armed"],
                "cpu_temp": CPU_TEMP_C,
            }
        ).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={protocol.MJPEG_BOUNDARY}"
        )
        self.end_headers()
        seen = 0
        try:
            while running:
                seen, jpg = frames.wait_newer(seen, timeout=1.0)
                if jpg is None:
                    continue
                self.wfile.write(f"--{protocol.MJPEG_BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # client went away mid-frame; normal


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


# --- main --------------------------------------------------------------------


def main() -> int:
    global running

    pygame.init()
    pygame.font.init()

    # Reclaim the signals from SDL. pygame.init() installs its own SIGINT/SIGTERM
    # handlers that turn the signal into an SDL_QUIT *event* -- and this process never
    # pumps the event queue, because it has no window. The result is a simulator that
    # ignores `kill` outright and keeps holding :8090/:8091, so the next `make sim`
    # dies on "Address already in use". Registering after init() overrides SDL's.
    def _handle_signal(signum, frame):
        global running
        running = False

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    LOG_DIR.mkdir(exist_ok=True)
    log_path = LOG_DIR / f"sim-{datetime.now().strftime('%Y%m%d-%H%M%S')}.jsonl"
    log = log_path.open("w", buffering=1)

    def record(direction: str, msg: dict) -> None:
        log.write(json.dumps({"dir": direction, "wall": time.time(), "msg": msg}) + "\n")

    started_m = time.monotonic()
    veh = Vehicle(VehicleConfig())

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", protocol.CONTROL_PORT))
    sock.setblocking(False)

    httpd = Server(("0.0.0.0", protocol.VIDEO_PORT), Handler)
    threading.Thread(target=httpd.serve_forever, name="http", daemon=True).start()
    cam = threading.Thread(target=camera_thread, args=(started_m,), name="cam", daemon=True)
    cam.start()

    print(f"[sim] control udp :{protocol.CONTROL_PORT}  video http :{protocol.VIDEO_PORT}")
    print(f"[sim] stream {protocol.stream_url('localhost')}  health {protocol.health_url('localhost')}")
    print(f"[sim] log {log_path}")

    tlm_seq = 0
    peer: tuple[str, int] | None = None
    last_cmd = None  # newest cmd of this tick
    # Newest cmd ever seen, kept across ticks: telemetry goes out at 20 Hz but commands
    # arrive at 50 Hz, so most tlm ticks have no cmd of their own to acknowledge.
    ack_seq = 0
    ack_ts = 0.0
    rx_stamps: deque[float] = deque()
    last_rx_m: float | None = None
    rx_gap_ms = 0.0
    expect_seq: int | None = None
    drops = 0
    rx_total = 0
    tx_total = 0
    bad = 0

    period = 1.0 / LOOP_HZ
    next_tlm = 0.0  # heartbeat deadline; telemetry is otherwise command-driven
    tick = 0
    next_t = time.monotonic()
    next_print = time.monotonic() + 1.0
    print_rx = print_tx = 0

    try:
        while running:
            now_m = time.monotonic()
            tick += 1

            # Drain the queue and keep only the newest valid cmd: stale inputs are worse
            # than no input, and a backlog of them is worse still.
            last_cmd = None
            while True:
                try:
                    data, addr = sock.recvfrom(protocol.MAX_DATAGRAM)
                except BlockingIOError:
                    break
                except OSError:
                    break
                msg = protocol.decode(data)
                if msg is None or msg.get("t") != protocol.MSG_CMD:
                    bad += 1
                    continue
                peer = addr
                last_cmd = msg
                ack_seq = int(msg.get("seq", 0))
                ack_ts = float(msg.get("ts", 0.0))
                rx_total += 1
                print_rx += 1
                rx_stamps.append(now_m)
                rx_gap_ms = 0.0 if last_rx_m is None else (now_m - last_rx_m) * 1000.0
                last_rx_m = now_m
                seq = int(msg.get("seq", 0))
                if expect_seq is not None and seq > expect_seq:
                    drops += seq - expect_seq
                expect_seq = seq + 1
                record("rx", msg)

            out = veh.update(last_cmd, now_m)
            publish(
                state=out.state,
                armed=out.armed,
                throttle_cmd=out.throttle_cmd,
                throttle_out=out.throttle_out,
                steer_cmd=out.steer_cmd,
                duty_pct=out.duty_pct,
                dir=out.direction,
                servo_us=out.servo_us,
                uptime=now_m - started_m,
            )

            while rx_stamps and now_m - rx_stamps[0] > 1.0:
                rx_stamps.popleft()

            # Answer each command as it arrives, exactly as the Pi does -- a fixed tick
            # would add 0-20 ms of invented latency to every RTT the driver reads. The
            # heartbeat only covers the quiet case, where there is nothing to answer.
            if peer is not None and (last_cmd is not None or now_m >= next_tlm):
                next_tlm = now_m + 1.0 / protocol.TLM_HEARTBEAT_HZ
                s = snapshot()
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
                    rx_rate_hz=len(rx_stamps),
                    rx_gap_ms=rx_gap_ms,
                    drops=drops,
                    cam_fps=s["cam_fps"],
                    cpu_temp=CPU_TEMP_C,
                    uptime=now_m - started_m,
                )
                try:
                    sock.sendto(protocol.encode(tlm), peer)
                    tx_total += 1
                    print_tx += 1
                    record("tx", tlm)
                except OSError:
                    pass

            if now_m >= next_print:
                next_print += 1.0
                s = snapshot()
                print(
                    f"[sim] {s['state']:<8} {'ARM' if s['armed'] else 'safe':<4} "
                    f"thr {s['throttle_cmd']:+.2f}->{s['throttle_out']:+.2f} "
                    f"{s['dir']:<5} duty {s['duty_pct']:5.1f}% servo {s['servo_us']}us "
                    f"| rx {print_rx:3d}/s tx {print_tx:2d}/s drops {drops} bad {bad} "
                    f"| cam {s['cam_fps']:4.1f}fps up {s['uptime']:.0f}s",
                    flush=True,
                )
                print_rx = print_tx = 0

            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()  # overran; resync rather than spin to catch up
    except KeyboardInterrupt:
        print("\n[sim] shutting down", flush=True)
    finally:
        running = False
        httpd.shutdown()
        httpd.server_close()
        sock.close()
        cam.join(timeout=1.0)
        log.close()
        pygame.quit()
        print(f"[sim] rx {rx_total} cmd, tx {tx_total} tlm, drops {drops}, log {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
