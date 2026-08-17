#!/usr/bin/env python3
"""TeleKart2 driving app: video pane, WASD, and a HUD you can read at speed.

    python mac/main.py                 # the simulator on this Mac (the default)
    python mac/main.py telekart.local  # the car -- only during an approved bring-up

This side sends **raw intent only** -- throttle and steer in -1..1, plus brake and the
arm level. It never computes a duty or a pulse width; every actuator number on screen
came back in telemetry from the side that owns the hardware. That separation is what
let the wheel drop in without changing a line on the Pi: keys and pedals produce the
same three floats, saturated for a key and proportional for a pedal.
"""

from __future__ import annotations

import json
import signal
import socket
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import pygame  # noqa: E402

import config  # noqa: E402
import protocol  # noqa: E402
from video_client import VideoClient  # noqa: E402

BG = (14, 15, 20)
PANE_BG = (22, 24, 30)
FG = (226, 230, 238)
DIM = (140, 148, 165)
OK = (90, 220, 130)
WARN = (240, 190, 80)
BAD = (240, 90, 90)
ACCENT = (110, 190, 255)

GEAR_D = "D"
GEAR_R = "R"


def _deadzone(v: float, dz: float) -> float:
    """Apply a deadzone, rescaled so just outside it still reads ~0.

    Without the rescale the control would jump to `dz` the instant it left centre.
    """
    if abs(v) < dz:
        return 0.0
    scaled = (abs(v) - dz) / (1.0 - dz)
    return protocol.clamp(scaled if v > 0 else -scaled, -1.0, 1.0)


def _open_wheel():
    """Return the wheel, or None to fall back to the keyboard.

    Deliberately not pygame: this wheel speaks XInput, which macOS exposes to nothing.
    See mac/xinput.py.
    """
    if not config.USE_WHEEL:
        return None
    try:
        from xinput import XInputDevice
    except ImportError as exc:
        print(f"[mac] wheel support unavailable ({exc}) -- WASD only")
        return None
    try:
        dev = XInputDevice.find()
    except Exception as exc:  # noqa: BLE001 -- no wheel must never stop you driving
        print(f"[mac] wheel unavailable ({exc}) -- WASD only")
        return None
    if dev is None:
        print("[mac] no wheel detected -- WASD only")
        return None
    # Prove the endpoint actually reads before trusting it. Claiming the interface can
    # succeed while reads fail with "Access denied" -- which in practice means another
    # instance of this app is still running and holding it.
    try:
        dev.poll()
    except Exception as exc:  # noqa: BLE001
        print(f"[mac] wheel found but unreadable ({exc})")
        print("[mac] another copy of this app is probably still running -- WASD only")
        dev.close()
        return None
    print(f"[mac] wheel: {dev.name} (XInput)  steer + analog throttle/brake")
    dev.start()
    return dev


def main() -> int:
    pygame.init()

    # Reclaim the signals from SDL. pygame.init() installs handlers that turn SIGINT and
    # SIGTERM into an SDL_QUIT *event*, which only helps if the window has focus and the
    # queue is being drained -- so `kill` on a backgrounded app does nothing and it keeps
    # running. That matters more here than it looks: a surviving instance holds the
    # wheel's USB interface, and the next launch fails with "Access denied" that looks
    # exactly like a macOS permissions problem.
    def _handle_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    pygame.display.set_caption(f"TeleKart2 — {config.HOST}")
    screen = pygame.display.set_mode((config.WINDOW_W, config.WINDOW_H))
    clock = pygame.time.Clock()
    f = pygame.font.SysFont("menlo,monaco,couriernew,dejavusansmono", 14)
    fs = pygame.font.SysFont("menlo,monaco,couriernew,dejavusansmono", 13)
    fb = pygame.font.SysFont("menlo,monaco,couriernew,dejavusansmono", 26, bold=True)

    config.LOG_DIR.mkdir(exist_ok=True)
    log_path = config.LOG_DIR / f"session-{datetime.now().strftime('%Y%m%d-%H%M%S')}.jsonl"
    log = log_path.open("w", buffering=1)

    def record(direction: str, msg: dict) -> None:
        log.write(json.dumps({"dir": direction, "wall": time.time(), "msg": msg}) + "\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    peer = (config.HOST, config.CONTROL_PORT)

    video = VideoClient(config.STREAM_URL, protocol.MJPEG_BOUNDARY).start()
    wheel = _open_wheel()

    print(f"[mac] host {config.HOST}  cmd udp :{config.CONTROL_PORT}  video {config.STREAM_URL}")
    print(f"[mac] log {log_path}")
    if config.IS_REMOTE:
        # Impossible to reach by accident, so impossible to miss when you do.
        print(f"[mac] *** REMOTE HOST {config.HOST} -- this is not the simulator. ***")
        print("[mac] *** If you did not mean to address the car, quit now (Esc). ***")
    else:
        print("[mac] simulator target (localhost). Override with TELEKART_HOST=<host>.")

    seq = 0
    armed = False
    gear = GEAR_D
    tlm: dict | None = None
    tlm_t = 0.0
    rtt_ms = 0.0
    tx_stamps: deque[float] = deque()
    rx_stamps: deque[float] = deque()
    tlm_log: deque[str] = deque(maxlen=6)
    started = time.monotonic()
    next_print = time.monotonic() + 1.0
    running = True

    def send(throttle: float, steer: float, brake: float, arm: bool) -> None:
        nonlocal seq
        seq += 1
        cmd = protocol.make_cmd(seq, throttle, steer, brake, arm)
        try:
            sock.sendto(protocol.encode(cmd), peer)
        except OSError:
            return
        tx_stamps.append(time.monotonic())
        record("tx", cmd)

    try:
        while running:
            now = time.monotonic()

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        running = False
                    elif ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                        # Latched here, restated in every packet -- a one-shot toggle
                        # over UDP would desync the moment a datagram went missing.
                        armed = not armed

            k = pygame.key.get_pressed()
            throttle = (1.0 if k[pygame.K_w] else 0.0) - (1.0 if k[pygame.K_s] else 0.0)
            steer = (1.0 if k[pygame.K_d] else 0.0) - (1.0 if k[pygame.K_a] else 0.0)
            brake = 1.0 if k[pygame.K_SPACE] else 0.0

            if wheel is not None and not wheel.alive:
                # A yanked cable must not take down the app: losing the app means losing
                # the watchdog heartbeat, and the car coasts to a stop far from you.
                # Fall back to the keys and keep the link alive.
                print(f"[mac] wheel lost ({wheel.error}) -- falling back to WASD", flush=True)
                wheel.close()
                wheel = None

            if wheel is not None:
                # Just read the latest state: the reader thread owns the endpoint.
                s = wheel.state
                w_steer = _deadzone(s.lx, config.WHEEL_STEER_DEADZONE)
                if config.WHEEL_STEER_INVERT:
                    w_steer = -w_steer
                if w_steer != 0.0:
                    steer = w_steer
                w_throttle = _deadzone(s.rt, config.WHEEL_PEDAL_DEADZONE)

                # Paddle shifters. One pedal, so direction is a gear: left selects R,
                # right selects D. Only while the throttle is released -- see config.
                if w_throttle <= config.GEAR_CHANGE_MAX_THROTTLE:
                    if s.button(config.WHEEL_GEAR_REVERSE_BIT):
                        gear = GEAR_R
                    elif s.button(config.WHEEL_GEAR_DRIVE_BIT):
                        gear = GEAR_D

                # The wheel adds to the keys rather than replacing them, so S still
                # reverses regardless of the selected gear.
                if w_throttle > 0.0:
                    throttle = w_throttle if gear == GEAR_D else -w_throttle
                brake = max(brake, _deadzone(s.lt, config.WHEEL_PEDAL_DEADZONE))

            send(throttle, steer, brake, armed)

            while True:  # drain: only the newest telemetry is worth looking at
                try:
                    data, _ = sock.recvfrom(protocol.MAX_DATAGRAM)
                except (BlockingIOError, OSError):
                    break
                msg = protocol.decode(data)
                if msg is None or msg.get("t") != protocol.MSG_TLM:
                    continue
                tlm = msg
                tlm_t = now
                rx_stamps.append(now)
                # Read this as an upper bound, not the wire time. The socket is drained
                # once per 50 Hz frame, so a reply can sit in the buffer up to 20 ms
                # before it is stamped here -- roughly 10 ms of overstatement on average.
                # Measured against a threaded receiver: HUD ~24 ms, true wire ~14 ms.
                # Fixing it properly means a receiver thread; the bound is honest enough
                # for driving, and the trend is what you actually watch.
                rtt_ms = max(0.0, (time.time() - float(msg.get("ack_ts", 0.0))) * 1000.0)
                record("rx", msg)
                tlm_log.append(
                    f"{msg['uptime']:6.1f} {msg['state']:<8} {'A' if msg['armed'] else '-'} "
                    f"{msg['throttle_out']:+.2f} {msg['duty_pct']:4.1f}% {msg['dir']:<5}"
                )

            for dq in (tx_stamps, rx_stamps):
                while dq and now - dq[0] > 1.0:
                    dq.popleft()

            stale = tlm is None or (now - tlm_t) > config.TLM_STALE_S

            # -- draw ----------------------------------------------------------
            screen.fill(BG)
            pane = pygame.Rect(8, 8, config.VIDEO_W, config.VIDEO_H)
            pygame.draw.rect(screen, PANE_BG, pane)
            frame = video.latest()
            if frame is not None and video.connected:
                if frame.get_size() != (pane.w, pane.h):
                    frame = pygame.transform.smoothscale(frame, (pane.w, pane.h))
                screen.blit(frame, pane.topleft)
            else:
                msg = "VIDEO DOWN" if not video.connected else "WAITING FOR VIDEO"
                screen.blit(fb.render(msg, True, BAD if not video.connected else WARN),
                            (pane.x + 24, pane.centery - 20))
                if video.error:
                    screen.blit(fs.render(video.error[:70], True, DIM), (pane.x + 24, pane.centery + 14))
            pygame.draw.rect(screen, (60, 66, 80), pane, 1)

            if stale:
                banner = pygame.Rect(pane.x, pane.y, pane.w, 46)
                pygame.draw.rect(screen, BAD, banner)
                screen.blit(fb.render("NO TELEMETRY — LINK LOST", True, (20, 10, 10)),
                            (banner.x + 14, banner.y + 10))

            # key legend under the video, where it costs no HUD space
            legend = (
                "WHEEL steer   PEDALS throttle+brake   PADDLES  L=R  R=D   ENTER arm   ESC quit"
                if wheel is not None
                else "W/S throttle   A/D steer   SPACE brake   ENTER arm   ESC quit"
            )
            screen.blit(f.render(legend, True, DIM), (12, pane.bottom + 12))
            screen.blit(
                f.render(f"video {video.fps:4.1f} fps   frames {video.frames}", True, DIM),
                (12, pane.bottom + 32),
            )

            x = config.PANEL_X
            y = 10

            def line(text: str, colour=FG, font=f, step: int = 15) -> None:
                nonlocal y
                screen.blit(font.render(text, True, colour), (x, y))
                y += step

            line(f"TELEKART2  {config.HOST}", ACCENT)
            if config.IS_REMOTE:
                # Not the simulator. Say so where the driver is already looking.
                line("*** REMOTE — REAL VEHICLE ***", BAD)
            y += 6
            if stale:
                age = "never" if tlm is None else f"{now - tlm_t:5.2f}s"
                line(f"LINK   STALE  {age}", BAD)
            else:
                line(f"LINK   OK  {rtt_ms:6.2f} ms RTT", OK)
            line(f"VIDEO  {'UP' if video.connected else 'DOWN':<4} {video.fps:5.1f} fps",
                 OK if video.connected else BAD)
            y += 6

            state = tlm["state"] if tlm else "—"
            is_armed = bool(tlm and tlm["armed"])
            line(f"STATE  {state}", OK if state == protocol.STATE_DRIVE else
                 BAD if state == protocol.STATE_FAILSAFE else WARN)
            line(f"       {'ARMED' if is_armed else 'DISARMED'}  (latch {'ON' if armed else 'off'})",
                 OK if is_armed else WARN)
            if wheel is not None:
                # Which way the pedal will send you is not visible from the wheel itself,
                # so it has to be on screen -- a reverse you forgot about is how you back
                # into something while looking at a forward-facing camera.
                line(f"GEAR   {'DRIVE' if gear == GEAR_D else 'REVERSE'}",
                     ACCENT if gear == GEAR_D else WARN)
            y += 6

            line(f"seq    {seq}", DIM)
            line(f"tx     {len(tx_stamps):3d} Hz", DIM)
            line(f"rx     {len(rx_stamps):3d} Hz", DIM)
            line(f"RTT    {rtt_ms:6.2f} ms", FG if not stale else DIM)
            y += 6

            if tlm:
                line(f"thr    {tlm['throttle_cmd']:+.2f} -> {tlm['throttle_out']:+.2f}")
                line(f"steer  {tlm['steer_cmd']:+.2f}   servo {tlm['servo_us']} us")
                line(f"duty   {tlm['duty_pct']:5.1f} %   dir {tlm['dir']}")
                line(f"cam    {tlm['cam_fps']:5.1f} fps  drops {tlm['drops']}")
                temp = "—" if tlm["cpu_temp"] is None else f"{tlm['cpu_temp']:.1f} C"
                line(f"cpu    {temp}   up {tlm['uptime']:.0f}s")
                line(f"pi rx  {tlm['rx_rate_hz']:.0f} Hz  gap {tlm['rx_gap_ms']:.0f} ms", DIM)
            else:
                for _ in range(6):
                    line("—", DIM)
            y += 8

            line("TELEMETRY", ACCENT, fs, 16)
            for row in tlm_log:
                line(row, DIM, fs, 14)
            y += 10

            line("KEYS", ACCENT, fs, 16)
            line(f"{config.KEYS['throttle_fwd']}/{config.KEYS['throttle_rev']}    throttle fwd / rev", DIM, fs, 14)
            line(f"{config.KEYS['steer_left']}/{config.KEYS['steer_right']}    steer left / right", DIM, fs, 14)
            line(f"{config.KEYS['brake']}  brake (EN high, IN==IN)", DIM, fs, 14)
            line(f"{config.KEYS['arm_toggle']}  toggle arm latch", DIM, fs, 14)
            line(f"{config.KEYS['quit']}    disarm and quit", DIM, fs, 14)

            pygame.display.flip()

            if now >= next_print:
                next_print += 1.0
                if tlm and not stale:
                    print(
                        f"[mac] {tlm['state']:<8} {'ARM' if tlm['armed'] else 'safe':<4} "
                        f"thr {tlm['throttle_cmd']:+.2f}->{tlm['throttle_out']:+.2f} "
                        f"{tlm['dir']:<5} duty {tlm['duty_pct']:5.1f}% servo {tlm['servo_us']}us "
                        f"| tx {len(tx_stamps):2d}/s rx {len(rx_stamps):2d}/s rtt {rtt_ms:5.2f}ms "
                        f"| video {video.fps:4.1f}fps cam {tlm['cam_fps']:4.1f}fps drops {tlm['drops']}",
                        flush=True,
                    )
                else:
                    print(
                        f"[mac] NO TELEMETRY  tx {len(tx_stamps):2d}/s  "
                        f"video {'up' if video.connected else 'DOWN'} {video.fps:4.1f}fps "
                        f"| up {now - started:.0f}s",
                        flush=True,
                    )

            clock.tick(config.CMD_RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave the car disarmed even if a datagram or two is lost on the way out.
        for _ in range(5):
            send(0.0, 0.0, 0.0, False)
            time.sleep(0.02)
        video.stop()
        sock.close()
        log.close()
        pygame.quit()
        print(f"[mac] sent {seq} cmd, log {log_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
