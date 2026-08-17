# TeleKart2 v1 — wire protocol

Two independent channels between the Mac and the Pi. Neither depends on the other, which is
deliberate: video can die without the car running away, and control keeps working when the camera
stalls.

| Channel | Transport | Port | Direction | Rate |
|---|---|---|---|---|
| Video | HTTP, MJPEG | 8090 | Pi → Mac | ~20 fps |
| Control | UDP, JSON | 8091 | Mac → Pi | 50 Hz |
| Telemetry | UDP, JSON | 8091 (reply) | Pi → Mac | 20 Hz |

Ports are 8090/8091 rather than the obvious 8080/8081 to reduce the chance of colliding with
whatever else is bound on the Pi.

The authoritative definition is [`common/protocol.py`](../common/protocol.py). `pi/protocol.py` is a
byte-identical copy — `make check-protocol` fails if they drift.

---

## The one rule that shapes everything

**The Mac sends raw driver intent. The Pi computes all vehicle behavior.**

Ramping, deadband, duty mapping, direction guards, arming and limits live in exactly one place:
[`common/vehicle.py`](../common/vehicle.py), on the side that owns the hardware. The Mac never
computes a PWM duty or a servo pulse width — it only ever says "the driver is holding W".

Two payoffs. Safety-critical logic can't be bypassed by a buggy or crashed client. And v2 swaps
key-booleans for steering-wheel-axis floats without the Pi changing at all.

---

## Control message — Mac → Pi

One JSON object per datagram, 50 Hz, ~90 bytes.

```json
{"t":"cmd","v":1,"seq":1234,"ts":1723800000.123,
 "throttle":1.0,"steer":-1.0,"brake":false,"arm":true}
```

| Field | Type | Meaning |
|---|---|---|
| `t` | `"cmd"` | Message type |
| `v` | int | Protocol version; mismatches are dropped |
| `seq` | int | Monotonic. Gaps are counted as `drops` in telemetry |
| `ts` | float | Mac's `time.time()`. Echoed back as `ack_ts` |
| `throttle` | −1..1 | W = +1, S = −1, both or neither = 0 |
| `steer` | −1..1 | A = −1, D = +1 |
| `brake` | bool | Space |
| `arm` | bool | Latched on the Mac by Enter |

### `arm` is a level, not an edge

Every packet restates the arm state rather than transmitting a toggle once. UDP drops packets, and a
lost toggle would leave the two ends disagreeing about whether the car is live — the exact
disagreement you cannot afford.

It also makes dropout recovery fall out for free. Packets resume, `arm` is still true, driving
resumes. If the driver disarmed during the gap, it stays safe. No resynchronisation handshake, no
state to reconcile.

---

## Telemetry message — Pi → Mac

Sent to whatever address the last `cmd` arrived from, 20 Hz. No configuration of the Mac's address
anywhere; the Pi just answers.

```json
{"t":"tlm","v":1,"seq":5678,"ts":1723800000.140,
 "ack_seq":1234,"ack_ts":1723800000.123,
 "armed":true,"state":"DRIVE",
 "throttle_cmd":1.0,"throttle_out":0.42,"steer_cmd":-1.0,
 "servo_us":1200,"duty_pct":42.0,"dir":"FWD",
 "rx_rate_hz":49.8,"rx_gap_ms":21.0,"drops":3,
 "cam_fps":19.7,"cpu_temp":52.1,"uptime":93.2}
```

| Field | Meaning |
|---|---|
| `ack_seq`, `ack_ts` | The newest `cmd` the Pi acted on, and its Mac-side timestamp verbatim |
| `armed`, `state` | `INIT` / `SAFE` / `DRIVE` / `FAILSAFE` |
| `throttle_cmd` | What the driver asked for |
| `throttle_out` | What the slew limiter is actually allowing |
| `servo_us` | Actual pulse width. `0` means pulses stopped entirely |
| `duty_pct`, `dir` | Actual PWM duty and bridge state: `FWD` / `REV` / `BRAKE` / `COAST` |
| `rx_rate_hz`, `rx_gap_ms`, `drops` | Link health as the Pi sees it |
| `cam_fps`, `cpu_temp`, `uptime` | Pi health |

### Two fields that do the debugging work

**`ack_ts`** is the Mac's own timestamp handed straight back. The Mac computes round-trip latency as
`time.time() - ack_ts` — no clock synchronisation between the machines, no NTP dependency, no drift.

**`throttle_cmd` vs `throttle_out`** makes the slew limiter visible. Without both, "I'm holding W and
it's crawling" reads as a bug rather than the current limiter doing exactly its job.

---

## States

```
INIT ──▶ SAFE ◀──▶ DRIVE
          ▲          │
          └── FAILSAFE ◀── watchdog: no cmd for 300 ms
```

| State | Motors | Servo |
|---|---|---|
| `SAFE` | Coast — throttle ignored entirely | Live (steering works disarmed, useful on the bench) |
| `DRIVE` | Armed, slew-limited | Live |
| `FAILSAFE` | Coast | Centred, then **pulses stop after 2 s** |

Servo pulses stop during a sustained failsafe because the HS-311 sits on the Pi's own 5 V rail, and a
stalled servo there can brown out the SoC.

Recovery from `FAILSAFE` is automatic once packets resume — see the `arm`-as-a-level note above.

---

## Video

`http://<host>:8090/stream.mjpg` — `multipart/x-mixed-replace; boundary=telekart2frame`, 640×480,
~20 fps, JPEG quality ~70.

MJPEG was chosen over H.264 purely for simplicity: every frame is independent, there is no decoder
state, no keyframe negotiation, and a dropped frame costs exactly one frame. The Pi's hardware H.264
encoder also cannot produce keyframes on demand — `FORCE_KEY_FRAME` is not honoured by the bcm2835
codec — which makes recovery after packet loss unpleasant. Latency is higher than H.264 and bandwidth
is much worse, and for v1 neither matters.

`http://<host>:8090/health` returns JSON (uptime, frames, fps) so bring-up can be checked with one
`curl`, before any pygame is involved.

---

## Failure modes and what happens

| Failure | Result |
|---|---|
| Mac app quits or crashes | Watchdog trips in 300 ms → coast |
| WiFi drops mid-drive | Watchdog trips → coast; resumes automatically on reconnect |
| Video stream dies | Control unaffected; Mac shows the video as down |
| Malformed / foreign datagram | Dropped silently by `protocol.decode`, which never raises |
| Datagrams arrive in a burst | Only the newest is acted on; stale commands are discarded, not queued |
| Pi process killed | Pins fall to the external 10 kΩ pull-downs → motors off |
