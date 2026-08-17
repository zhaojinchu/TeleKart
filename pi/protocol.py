"""TeleKart2 v1 wire protocol.

Shared by the Mac app (``mac/``) and the simulator (``sim/``). ``pi/protocol.py`` is a
byte-identical copy, because the Pi is a separate machine with its own install;
``make check-protocol`` diffs the two so they cannot silently drift.

Two channels:

* **Video** -- MJPEG over HTTP on :8090, ``multipart/x-mixed-replace``.
* **Control** -- JSON over UDP on :8091. Mac sends ``cmd`` at 50 Hz, the Pi replies
  ``tlm`` at 20 Hz to whatever address the last ``cmd`` came from.

One JSON object per datagram. Nothing is fragmented, nothing is streamed.
"""

from __future__ import annotations

import json
import time

#: v2 made ``brake`` a 0..1 float (it was a bool). Bumped rather than kept compatible on
#: purpose: a Pi still running v1 rejects v2 datagrams outright, so its watchdog trips
#: and it coasts. Silently reading a float as a truthy bool would mean the lightest touch
#: on the pedal reading as full braking.
PROTOCOL_VERSION = 2

# --- Network -----------------------------------------------------------------

VIDEO_PORT = 8090
CONTROL_PORT = 8091

STREAM_PATH = "/stream.mjpg"
HEALTH_PATH = "/health"

MJPEG_BOUNDARY = "telekart2frame"

#: Datagrams larger than this are a bug, not a big message.
MAX_DATAGRAM = 1500

# --- Timing ------------------------------------------------------------------

CMD_RATE_HZ = 50.0

#: Telemetry is **command-driven**: the Pi answers each ``cmd`` as it arrives rather than on
#: a timer, so this is the effective rate you get when the Mac is sending at CMD_RATE_HZ.
#:
#: Replying on receipt is what makes the reported RTT mean something. On a fixed tick the
#: echoed ``ack_ts`` belonged to whichever command happened to be newest, adding a uniform
#: 0-20 ms of pure waiting to every reading; answering immediately removes that term and
#: leaves the actual network time. Measured: ~28 ms -> ~18 ms median.
TLM_RATE_HZ = 50.0

#: When no commands are arriving there is nothing to answer, but the Mac still needs to see
#: that the car is alive and sitting in FAILSAFE. This slow heartbeat covers that gap
#: without putting a timer back in the latency path.
TLM_HEARTBEAT_HZ = 5.0

#: No ``cmd`` for this long and the Pi drops to FAILSAFE (coast + centre).
WATCHDOG_TIMEOUT_S = 0.30

#: No ``tlm`` for this long and the Mac shows the link as lost.
TLM_STALE_S = 0.50

# --- Message types -----------------------------------------------------------

MSG_CMD = "cmd"
MSG_TLM = "tlm"

# --- Vehicle states ----------------------------------------------------------

STATE_INIT = "INIT"
STATE_SAFE = "SAFE"  # connected, disarmed -- throttle ignored
STATE_DRIVE = "DRIVE"  # armed and receiving commands
STATE_FAILSAFE = "FAILSAFE"  # watchdog tripped -- coasting

DIR_FWD = "FWD"
DIR_REV = "REV"
DIR_BRAKE = "BRAKE"
DIR_COAST = "COAST"


def clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def make_cmd(
    seq: int,
    throttle: float,
    steer: float,
    brake: float,
    arm: bool,
    ts: float | None = None,
) -> dict:
    """Build a control message.

    ``throttle`` and ``steer`` are raw intent in -1..1, ``brake`` is pedal travel in
    0..1. The Mac never computes a PWM duty or a servo pulse width -- the Pi owns all of
    that. A keyboard sends the same fields, just saturated: W is throttle 1.0 and the
    space bar is brake 1.0.

    ``arm`` is a **level, not an edge**. UDP drops packets, so a toggle transmitted once
    would desync; the Mac holds the latch and every packet restates it. That also makes
    recovery after a dropout automatic: packets resume, ``arm`` is still true, driving
    resumes -- and if the driver disarmed, it stays safe.
    """
    return {
        "t": MSG_CMD,
        "v": PROTOCOL_VERSION,
        "seq": int(seq),
        "ts": float(ts if ts is not None else time.time()),
        "throttle": round(clamp(float(throttle), -1.0, 1.0), 4),
        "steer": round(clamp(float(steer), -1.0, 1.0), 4),
        "brake": round(clamp(float(brake), 0.0, 1.0), 4),
        "arm": bool(arm),
    }


def make_tlm(
    seq: int,
    ack_seq: int,
    ack_ts: float,
    armed: bool,
    state: str,
    throttle_cmd: float,
    throttle_out: float,
    steer_cmd: float,
    servo_us: int,
    duty_pct: float,
    direction: str,
    rx_rate_hz: float,
    rx_gap_ms: float,
    drops: int,
    cam_fps: float,
    cpu_temp: float | None = None,
    uptime: float = 0.0,
    ts: float | None = None,
) -> dict:
    """Build a telemetry message.

    ``ack_ts`` is the Mac's own timestamp echoed straight back, so the Mac gets
    round-trip latency for free with no clock synchronisation between the machines.

    ``throttle_cmd`` vs ``throttle_out`` exposes the slew limiter. Without both, "I'm
    holding W and it's crawling" looks like a bug instead of the current limiter doing
    its job.
    """
    return {
        "t": MSG_TLM,
        "v": PROTOCOL_VERSION,
        "seq": int(seq),
        "ts": float(ts if ts is not None else time.time()),
        "ack_seq": int(ack_seq),
        "ack_ts": float(ack_ts),
        "armed": bool(armed),
        "state": state,
        "throttle_cmd": round(float(throttle_cmd), 4),
        "throttle_out": round(float(throttle_out), 4),
        "steer_cmd": round(float(steer_cmd), 4),
        "servo_us": int(servo_us),
        "duty_pct": round(float(duty_pct), 1),
        "dir": direction,
        "rx_rate_hz": round(float(rx_rate_hz), 1),
        "rx_gap_ms": round(float(rx_gap_ms), 1),
        "drops": int(drops),
        "cam_fps": round(float(cam_fps), 1),
        "cpu_temp": (None if cpu_temp is None else round(float(cpu_temp), 1)),
        "uptime": round(float(uptime), 1),
    }


def encode(msg: dict) -> bytes:
    return json.dumps(msg, separators=(",", ":")).encode("utf-8")


def decode(data: bytes) -> dict | None:
    """Parse a datagram, returning ``None`` if it is not a usable message.

    Never raises. A control receiver that dies on one malformed packet is a control
    receiver that dies mid-drive, and on a shared network we will eventually be sent
    something we did not send ourselves.
    """
    try:
        msg = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(msg, dict):
        return None
    if msg.get("t") not in (MSG_CMD, MSG_TLM):
        return None
    if msg.get("v") != PROTOCOL_VERSION:
        return None
    return msg


def stream_url(host: str, port: int = VIDEO_PORT) -> str:
    return f"http://{host}:{port}{STREAM_PATH}"


def health_url(host: str, port: int = VIDEO_PORT) -> str:
    return f"http://{host}:{port}{HEALTH_PATH}"
