# TeleKart v2 — Wire Protocol Reference

This document describes the contents of
[`packages/telekart_protocol/`](../packages/telekart_protocol/telekart_protocol/). **That
package is the contract.** Every offset, size and constant below was read out of the source
and verified by packing real values and dumping the bytes; where this document and the code
disagree, the code wins and this document is the bug.

The package is imported unmodified by the Pi firmware, the desktop application and the
simulator. That single-source property is the point: the previous generation kept a
hand-written C struct and a hand-written Python `struct.Struct` in sync by hand, which is a
bug waiting for the least convenient moment.

**Companion documents:** [INTERFACES.md](INTERFACES.md) for the *internal* Python interfaces
that do not cross the network · [tuning.md](tuning.md) for the video-latency consequences of
§7 · [bringup.md](bringup.md) for the fault bits in context

---

## 1. Overview

### Four channels

| Channel | Transport | Port | Direction | Rate | Payload |
|---|---|---|---|---|---|
| Control | **UDP** | 4210 (car listens) | app → car | 100 Hz | fixed 40 bytes, HMAC-tagged |
| Telemetry | **UDP** | 4211 (app listens, negotiable) | car → app | 50 Hz | fixed 98 bytes, HMAC-tagged |
| Session | **TCP** | 4212 (car listens) | both | event | newline-delimited JSON |
| Video | **TCP** | 4213 (car listens) | car → app | 30 fps | 24-byte header + Annex-B / JPEG |

Discovery is mDNS: `_telekart._tcp.local.`

### Why UDP for control and telemetry

Latest-wins, on purpose. A control stream wants the *newest* command, never a retransmitted
stale one, and TCP head-of-line blocking during a WiFi hiccup would deliver precisely the
wrong thing — a burst of commands from half a second ago, in order, at the moment you most
need the current one.

### Why TCP+JSON for the session

Everything that must not be lost lives there: the handshake, parameter push, arm/disarm,
calibration control. The rate is low enough that JSON costs nothing, and its debuggability is
worth a great deal during bring-up — `nc telekart.local 4212` shows you a readable
conversation.

The TCP connection also doubles as a **presence** signal. If it drops, the car has lost its
operator, and the firmware treats that as an E-stop condition **regardless of whether UDP
control packets are still arriving from somewhere**.

### Versioning

```python
PROTO_VERSION = 2
```

Bumped on any incompatible change to packet layout or session semantics. **The handshake
rejects a mismatch outright rather than trying to negotiate** — a silently-misparsed control
packet is a runaway car.

### Magic numbers

| Constant | Value | Bytes (little-endian) |
|---|---|---|
| `MAGIC_CONTROL` | `0x314B5443` | `43 54 4B 31` = `CTK1` |
| `MAGIC_TELEMETRY` | `0x314B5454` | `54 54 4B 31` = `TTK1` |
| `MAGIC_VIDEO` | `0x44565854` | `54 58 56 44` = `TXVD` |

They exist so a stray datagram from an unrelated service — or an old v1 ESP32 board still on
the network — is rejected before the version field is even looked at.

### Byte order

**Little-endian, no padding**, everywhere. All `struct` formats begin with `<`.

---

## 2. ControlPacket — app → car, 100 Hz over UDP

```python
struct.Struct("<IHIIQhhhHH8s")     # CONTROL_PACKET_LEN == 40
```

The module asserts `CONTROL_PACKET_LEN == 40` at import time, so a layout drift fails on the
first import rather than on the first corner.

### 2.1 Byte layout

| Offset | Size | Type | Field | Range / meaning |
|---:|---:|---|---|---|
| 0 | 4 | `u32` | `magic` | `MAGIC_CONTROL` |
| 4 | 2 | `u16` | `version` | `PROTO_VERSION` (2) |
| 6 | 4 | `u32` | `session_id` | from the TCP handshake |
| 10 | 4 | `u32` | `sequence` | **strictly increasing**; replay guard |
| 14 | 8 | `u64` | `client_time_us` | app monotonic clock, µs; echoed back for RTT |
| 22 | 2 | `i16` | `steering` | −1000 (full left) … +1000 (full right) |
| 24 | 2 | `i16` | `throttle` | 0 … 1000 |
| 26 | 2 | `i16` | `brake` | 0 … 1000 |
| 28 | 2 | `u16` | `flags` | `ControlFlags` |
| 30 | 2 | `u16` | `reserved` | must be 0 |
| 32 | 8 | `bytes` | `mac_tag` | truncated HMAC-SHA256 over bytes `[0, 32)` |

**Note that `throttle` and `brake` are signed on the wire** (`h`, not `H`) even though they are
semantically 0…1000. Both `pack` and `unpack` clamp them, so a negative value from a buggy or
hostile sender arrives as 0 rather than as 65 000-something.

### 2.2 Scaling

| Constant | Value |
|---|---|
| `STEERING_SCALE` | 1000 |
| `THROTTLE_SCALE` | 1000 |
| `BRAKE_SCALE` | 1000 |

Commands travel as integers so the wire format is exact and endian-safe. `from_normalized()`
converts from floats and **clamps rather than rejects** — a control path is the wrong place to
raise on an out-of-range float from a miscalibrated axis.

Normalized views are available as `steering_f`, `throttle_f`, `brake_f`, plus a convenience
`estop` property.

### 2.3 `ControlFlags`

| Bit | Value | Name | Meaning |
|---:|---:|---|---|
| 0 | `0x0001` | `ESTOP` | Operator E-stop |
| 1 | `0x0002` | `REVERSE_REQ` | Request reverse |
| 2 | `0x0004` | `RESET_ODOM` | Zero the odometry |
| 3 | `0x0008` | `PIT_LIMITER` | Clamp to `pit_duty` |
| 4 | `0x0010` | `ARM_INTENT` | Heartbeat-level "I still want to be armed" |
| 5 | `0x0020` | `HEADLIGHTS` | |
| 6 | `0x0040` | `HORN` | |

### 2.4 Decode order — cheapest first

`ControlPacket.unpack()` checks in this order, and raises `ProtocolError` (a `ValueError`
subclass) on any failure. **Callers drop the datagram; they never raise out of the receive
path.**

1. **Length** must be exactly 40.
2. **Magic** must be `MAGIC_CONTROL`.
3. **Version** must equal `PROTO_VERSION`.
4. **HMAC** must verify.

Garbage traffic therefore costs almost nothing — the HMAC is only spent on something that
already looks like ours.

`peek_session_id(data)` reads the session id **without authenticating**. It exists purely so
that "a packet arrived for a session I don't know about" can be logged at near-zero cost.
**Never use it to make a control decision.**

### 2.5 Worked example

Reproduce this exactly:

```python
from telekart_protocol import ControlPacket, ControlFlags
from telekart_protocol.crypto import normalize_shared_key, derive_udp_key

key   = derive_udp_key(normalize_shared_key("telekart-demo"), bytes(range(16)))
pkt   = ControlPacket.from_normalized(
            session_id=0x0000BEEF, sequence=1234, client_time_us=0x0000000123456789,
            steering=-0.25, throttle=0.60, brake=0.0, flags=ControlFlags.ARM_INTENT)
print(pkt.pack(key).hex())
```

```
offset  bytes                     field
  0     43 54 4b 31               magic          'CTK1'
  4     02 00                     version        2
  6     ef be 00 00               session_id     0x0000BEEF
 10     d2 04 00 00               sequence       1234
 14     89 67 45 23 01 00 00 00   client_time_us 0x123456789
 22     06 ff                     steering       -250   (-0.25)
 24     58 02                     throttle       600    ( 0.60)
 26     00 00                     brake          0
 28     10 00                     flags          ARM_INTENT
 30     00 00                     reserved
 32     e2 20 d3 0b d8 10 5f f7   mac_tag
```

---

## 3. TelemetryPacket — car → app, 50 Hz over UDP

```python
struct.Struct("<IHHIIQQIIBBhhhhhhHhhHiihIHHhIHH8s")   # TELEMETRY_PACKET_LEN == 98
```

Everything the HUD, the safety display and the deferred racing-sim layer need, in **one**
datagram, so the app always renders a self-consistent snapshot rather than stitching together
fields that arrived at different times.

### 3.1 Byte layout

| Offset | Size | Type | Field | Units / scaling |
|---:|---:|---|---|---|
| 0 | 4 | `u32` | `magic` | `MAGIC_TELEMETRY` |
| 4 | 2 | `u16` | `version` | `PROTO_VERSION` (2) |
| 6 | 2 | `u16` | `flags` | `TelemetryFlags`, low 16 bits |
| 8 | 4 | `u32` | `session_id` | |
| 12 | 4 | `u32` | `sequence` | |
| 16 | 8 | `u64` | `car_time_us` | car `CLOCK_MONOTONIC`, µs |
| 24 | 8 | `u64` | `echo_client_time_us` | **verbatim echo** of the last accepted control packet |
| 32 | 4 | `u32` | `echo_sequence` | sequence of that packet |
| 36 | 4 | `u32` | `faults` | `Fault`, low 32 bits |
| 40 | 1 | `u8` | `state` | `VehicleState` |
| 41 | 1 | `u8` | `reserved` | 0 |
| 42 | 2 | `i16` | `rpm_l` | RPM |
| 44 | 2 | `i16` | `rpm_r` | RPM |
| 46 | 2 | `i16` | `rpm_target_l` | RPM |
| 48 | 2 | `i16` | `rpm_target_r` | RPM |
| 50 | 2 | `i16` | `duty_l` | −1000 … +1000 (`DUTY_SCALE`) |
| 52 | 2 | `i16` | `duty_r` | −1000 … +1000 |
| 54 | 2 | `u16` | `servo_us` | µs |
| 56 | 2 | `i16` | `steer_angle_cdeg` | centi-degrees |
| 58 | 2 | `i16` | `speed_mm_s` | mm/s (±32.767 m/s) |
| 60 | 2 | `u16` | `v_max_mm_s` | mm/s — **measured**, see 3.2 |
| 62 | 4 | `i32` | `odom_x_mm` | mm |
| 66 | 4 | `i32` | `odom_y_mm` | mm |
| 70 | 2 | `i16` | `heading_cdeg` | centi-degrees, −18000 … +18000 |
| 72 | 4 | `u32` | `distance_mm` | mm, cumulative |
| 76 | 2 | `u16` | `slip_index` | × `SLIP_SCALE` (1000.0) |
| 78 | 2 | `u16` | `pack_mv` | mV; **0 when no battery sensing is fitted** |
| 80 | 2 | `i16` | `cpu_temp_dc` | deci-Celsius |
| 82 | 4 | `u32` | `throttled` | raw `vcgencmd get_throttled` bitmask |
| 86 | 2 | `u16` | `loop_p99_us` | µs |
| 88 | 2 | `u16` | `reserved` | 0 |
| 90 | 8 | `bytes` | `mac_tag` | HMAC-SHA256 over bytes `[0, 90)` |

Physical quantities travel as **scaled integers** (mm, mm/s, centi-degrees, deci-Celsius):
exact on the wire, no float endianness questions, and a resolution far finer than the sensors
warrant.

### 3.2 `v_max_mm_s` — the field that makes the whole design portable

This is the **measured** top speed from auto-calibration, never a nameplate figure:

```
v_max_mps = max_rpm_measured / 60 × π × wheel_diameter_m
```

The app scales its speedometer off it. That is what lets the same build work with today's
L298N-limited drivetrain and with a MOSFET bridge later, **with no code change on either
side** ([power.md §9.2](power.md), [calibration.md §4.4](calibration.md)).

**Nothing in the codebase hardcodes a top speed.** This field is why.

### 3.3 Saturation, not overflow

`TelemetryPacket.from_si()` builds from SI units and **every field saturates rather than
overflowing**. NaN and infinity are mapped to 0 by an internal `_finite()` helper, and
`heading_rad` is wrapped to (−180°, +180°] before scaling so the `i16` centi-degree field
always fits.

> A NaN or a runaway value from a divide-by-zero upstream must not raise inside the telemetry
> path and take the link down with it. **A pinned gauge is diagnosable; a dead link is not.**

Field limits worth knowing, because they will pin before anything else does:

| Field | Type | Saturates at |
|---|---|---|
| `speed_mm_s` | `i16` | ±32.767 m/s |
| `v_max_mm_s` | `u16` | 65.535 m/s |
| `pack_mv` | `u16` | 65.535 V |
| `loop_p99_us` | `u16` | 65.535 ms |
| `slip_index` | `u16` | 65.535 |
| `distance_mm` | `u32` | 4294 km |

### 3.4 Convenience views

`duty_l_f`, `duty_r_f`, `speed_mps`, `v_max_mps`, `speed_fraction`, `heading_rad`,
`steer_angle_deg`, `distance_m`, `pose_m` (an `(x, y, heading)` tuple in metres and radians),
`slip`, `pack_volts`, `cpu_temp_c`, `armed`, `drivable`.

`speed_fraction` returns 0.0 when `v_max_mm_s` is 0, so a gauge cannot divide by zero before
calibration has run.

### 3.5 Forward compatibility

`unpack()` is deliberately tolerant of a firmware from the future:

- **Unknown `state` values** become `VehicleState.FAULT` via an internal `_safe_state()`, which
  catches the `ValueError` an `IntEnum` would otherwise raise.
- **Unknown `faults` and `flags` bits are preserved verbatim.** `Fault` and `TelemetryFlags`
  are `IntFlag`, which on Python 3.11+ defaults to `boundary=KEEP`, so `Fault(1 << 30)` round
  trips intact.

> An unknown fault bit is still worth showing the driver.

Magic, version and HMAC are still enforced — tolerance applies to *semantics*, never to
*authenticity*.

### 3.6 Worked example

```
offset  bytes                     field                value
  0     54 54 4b 31               magic                'TTK1'
  4     02 00                     version              2
  6     8c 00                     flags                CALIBRATED|CLOSED_LOOP|ODOM_VALID
  8     ef be 00 00               session_id           0x0000BEEF
 12     4d 00 00 00               sequence             77
 16     dd cc bb aa 00 00 00 00   car_time_us          0xAABBCCDD
 24     89 67 45 23 01 00 00 00   echo_client_time_us  0x123456789
 32     d2 04 00 00               echo_sequence        1234
 36     00 00 00 00               faults               NONE
 40     02                        state                ARMED
 41     00                        reserved
 42     97 00                     rpm_l                151
 44     95 00                     rpm_r                149
 46     96 00                     rpm_target_l         150
 48     96 00                     rpm_target_r         150
 50     6c 02                     duty_l               620      (0.620)
 52     62 02                     duty_r               610      (0.610)
 54     dc 05                     servo_us             1500
 56     a8 fd                     steer_angle_cdeg     -600     (-6.00°)
 58     fe 01                     speed_mm_s           510      (0.510 m/s)
 60     a8 02                     v_max_mm_s           680      (0.680 m/s)
 62     d2 04 00 00               odom_x_mm            1234
 66     c9 fd ff ff               odom_y_mm            -567
 70     f0 06                     heading_cdeg         1776     (17.76°)
 72     d4 30 00 00               distance_mm          12500
 76     14 00                     slip_index           20       (0.020)
 78     bc 1b                     pack_mv              7100     (7.100 V)
 80     e3 01                     cpu_temp_dc          483      (48.3 °C)
 82     00 00 00 00               throttled            0x0
 86     a0 28                     loop_p99_us          10400
 88     00 00                     reserved
 90     8d 9a 4e ea a5 7d e2 cf   mac_tag
```

---

## 4. Enumerations

### 4.1 `VehicleState` (`IntEnum`)

| Value | Name |
|---:|---|
| 0 | `BOOT` |
| 1 | `SAFE` |
| 2 | `ARMED` |
| 3 | `FAILSAFE` |
| 4 | `ESTOP` |
| 5 | `FAULT` |

> **The car owns this; the app only displays it. The app must never assume it is armed
> because it sent an `ARM` — it is armed when telemetry says so.**

### 4.2 `TelemetryFlags` (`IntFlag`, 16 bits on the wire)

| Bit | Value | Name | Meaning |
|---:|---:|---|---|
| 0 | `0x0001` | `LIMITER_ACTIVE` | A firmware protection limiter is clipping the command. **If this lights during normal driving, the app's rate limits are looser than the firmware's and the HUD will disagree with the car.** |
| 1 | `0x0002` | `DIRECTION_UNCERTAIN` | Encoders counting while commanded duty is zero, so the sign of motion is a guess — the x2-decode tradeoff |
| 2 | `0x0004` | `CALIBRATED` | |
| 3 | `0x0008` | `CLOSED_LOOP` | |
| 4 | `0x0010` | `REVERSE_ENGAGED` | |
| 5 | `0x0020` | `PIT_LIMITER` | |
| 6 | `0x0040` | `BRAKING` | |
| 7 | `0x0080` | `ODOM_VALID` | |
| 8 | `0x0100` | `VIDEO_ACTIVE` | |

### 4.3 `Fault` (`IntFlag`, 32 bits on the wire) — **sticky until explicitly cleared**

| Bit | Value | Name | Notes |
|---:|---:|---|---|
| 0 | `0x00000001` | `STALL_L` | |
| 1 | `0x00000002` | `STALL_R` | |
| 2 | `0x00000004` | `ENCODER_FAIL_L` | |
| 3 | `0x00000008` | `ENCODER_FAIL_R` | |
| 4 | `0x00000010` | `BROWNOUT` | **Both** encoders hit zero simultaneously while commanded — the boost regulator tripping, **not** a mechanical stall. Distinct code so the two are never confused during diagnosis |
| 5 | `0x00000020` | `CONTROL_TIMEOUT` | |
| 6 | `0x00000040` | `OVERTEMP` | **critical** |
| 7 | `0x00000080` | `LOW_BATTERY` | |
| 8 | `0x00000100` | `CRITICAL_BATTERY` | **critical** |
| 9 | `0x00000200` | `GPIO_ERROR` | **critical** |
| 10 | `0x00000400` | `LOOP_OVERRUN` | |
| 11 | `0x00000800` | `SERVO_FAULT` | |
| 12 | `0x00001000` | `PI_UNDERVOLTAGE` | from `vcgencmd get_throttled` |
| 13 | `0x00002000` | `PI_THROTTLED` | from `vcgencmd get_throttled` |
| 14 | `0x00004000` | `CAMERA_DOWN` | |
| 15 | `0x00008000` | `ESTOP_LATCHED` | **critical** |
| 16 | `0x00010000` | `CALIBRATION_MISSING` | |

```python
CRITICAL_FAULTS = CRITICAL_BATTERY | GPIO_ERROR | OVERTEMP | ESTOP_LATCHED   # 0x8340
```

**Only those four force a disarm.** Everything else warns. In particular `PI_UNDERVOLTAGE` and
`STALL_*` are *not* critical — killing the drive because the Pi dipped once mid-corner would
be its own hazard ([power.md §6](power.md)).

### 4.4 `VideoFrameFlags` and `VideoCodec`

| Bit | Value | Name |
|---:|---:|---|
| 0 | `0x0001` | `KEYFRAME` |
| 1 | `0x0002` | `DROPPED_BEFORE` — frames were dropped by the sender's bounded queue before this one; expect corruption until the next keyframe |
| 2 | `0x0004` | `CONFIG` — SPS/PPS-only frame |

| Value | `VideoCodec` |
|---:|---|
| 0 | `H264` |
| 1 | `MJPEG` |

---

## 5. Authentication and the session-token flow

### 5.1 Threat model, stated plainly

> The risk is a **stale or second controller** injecting commands into a moving car — a laptop
> that was disconnected and reconnected, a leftover process, another student on the same LAN.
> It is **not** a determined attacker with a packet capture.

A truncated HMAC over a strictly-increasing sequence number covers that completely, at roughly
15 µs per packet. Do not over-engineer it later.

`MAC_TAG_LEN = 8` — eight bytes, widened from the old firmware's four.

> **This is worth nothing if `pigpiod` is listening on an open port next to it.** pigpiod
> accepts unauthenticated commands from any host by default; the `-l` flag in
> [pi-setup.md §3](pi-setup.md) is part of this security story, not separate from it.

### 5.2 Key derivation

```python
normalize_shared_key(value: str | bytes) -> bytes      # SHA-256 of the passphrase
derive_udp_key(shared_key: bytes, session_token: bytes) -> bytes
compute_tag(key: bytes, payload: bytes) -> bytes       # HMAC-SHA256, truncated to 8
verify_tag(key: bytes, payload: bytes, tag: bytes) -> bool   # constant-time
make_session_token() -> bytes                          # os.urandom(16)
```

`derive_udp_key` is HMAC-SHA256 used as a one-step KDF — the shared secret keys the MAC, and
the session token plus a domain-separation label form the message:

```
udp_key = HMAC-SHA256(shared_key, b"telekart-udp-v2" + session_token)
```

Both ends run this identically after the handshake. Constraints enforced at call time:
`shared_key` must be non-empty, and `session_token` must be **at least 8 bytes**
(`make_session_token()` returns 16).

A passphrase is hashed rather than used directly so that short, memorable keys still produce
full-width key material. The UDP key is derived from a **per-connection** token rather than
being a constant on a command line, so a key never sits in shell history or a screenshot.

### 5.3 The signed region

The tag is the **trailing** field of both packets, so "the packet with the tag zeroed" and
"the packet minus the tag" are the same bytes. The implementation uses the latter, which
removes any ambiguity about zero-filling:

| Packet | Signed bytes | Tag at |
|---|---|---|
| Control | `[0, 32)` | 32…39 |
| Telemetry | `[0, 90)` | 90…97 |

### 5.4 Replay protection

```python
SEQUENCE_REPLAY_WINDOW = 0      # strictly increasing
```

A control packet whose `sequence` is **not greater** than the last accepted one is dropped. No
window, no reordering tolerance — on a 100 Hz stream where the newest command is the only one
that matters, a reordered packet is a stale packet.

`sequence` is `u32`, so it wraps after 2³² packets. At 100 Hz that is 497 days of continuous
driving; the strictly-increasing rule would reject the wrap, which is not a practical concern
but is worth knowing before someone spends a day on it.

### 5.5 The handshake, end to end

> ### ⚠ Interface gap — read this before implementing either end
>
> `session.hello()`'s docstring says *"`auth` is the HMAC of the server nonce; see `nonce`
> below"*, **but there is no `nonce` constructor and no `NONCE` member of `MsgType` in the
> package.** The nonce delivery step is genuinely unspecified by the frozen source.
>
> The resolution below is what this documentation set assumes. It was chosen because it
> **requires no change to `telekart_protocol`**: `Message` carries an arbitrary `data` dict,
> and the existing `ack(msg_id, **extra)` constructor produces exactly the required line.
> Firmware and app must agree on it or the handshake cannot complete.

**Step 1 — discovery.** The app browses mDNS for `_telekart._tcp.local.` and connects TCP to
the advertised host on `TCP_SESSION_PORT` (4212).

**Step 2 — the car greets with a nonce.** Immediately on accept, before anything else, the car
generates 16 random bytes and sends:

```json
{"type":"ack","id":0,"nonce":"000102030405060708090a0b0c0d0e0f"}
```

Produced by `ack(0, nonce=nonce.hex())`. `id` is 0 because it is unsolicited.

**Step 3 — the app authenticates.**

```python
K     = normalize_shared_key(shared_key)                       # SHA-256 of the passphrase
auth  = hmac.new(K, bytes.fromhex(nonce), hashlib.sha256).hexdigest()
```

```json
{"type":"hello","id":1,"proto":2,"app_version":"2.0.0","driver":"zhaojin",
 "auth":"55a579a805d365562f0a712f1b4ce1c4c38e3ab9ec21f1e779b802fcdb16540d",
 "telemetry_port":4211}
```

Note `telemetry_port`: **the app tells the car where to send telemetry** rather than assuming
the default, so two apps on one machine can coexist.

**Step 4 — the car verifies** with `hmac.compare_digest`.

| Failure | Response | Then |
|---|---|---|
| `proto` ≠ `PROTO_VERSION` | `ERROR` with `protocol_version` | close |
| `auth` mismatch | `ERROR` with `auth_failed` | close |
| Missing / wrong-typed field | `ERROR` with `bad_request` | close |

**Step 5 — the car issues a session.** `session_token = make_session_token()` (16 random
bytes) and a `u32` `session_id`:

```json
{"type":"hello_ack","id":1,"proto":2,"car_id":"telekart-01","fw_version":"2.0.0",
 "session_id":48879,"session_token":"000102030405060708090a0b0c0d0e0f",
 "caps":["video","calibrate","params"],"video_port":4213,"control_port":4210}
```

`id` matches the `HELLO` it answers. `control_port` and `video_port` are given explicitly for
the same reason as `telemetry_port`.

**Step 6 — both ends derive the UDP key.**

```python
udp_key = derive_udp_key(K, bytes.fromhex(session_token))
```

**Step 7 — the streams start.** The app sends `ControlPacket`s to `control_port` and receives
`TelemetryPacket`s on `telemetry_port`, both tagged with `udp_key` and carrying `session_id`.
The app connects TCP to `video_port` for the frame stream.

**Step 8 — teardown.**

- TCP drop → **presence lost → E-stop condition**, even if valid UDP is still arriving.
- `SESSION_TTL_S = 300` — a session with no traffic at all for five minutes is discarded by the
  car.

### 5.6 Conventions this document proposes (not pinned by the source)

Flagged so both ends adopt the same reading:

| Item | Proposal |
|---|---|
| Nonce delivery | `ACK` with `id: 0` and a `nonce` field of 32 hex characters, sent by the car on accept |
| `auth` encoding | lowercase hex of the full 32-byte HMAC-SHA256, **not** truncated to `MAC_TAG_LEN` |
| `session_token` encoding | lowercase hex, 32 characters (16 bytes) |
| `session_id == 0` | reserved for "no session"; the car issues non-zero ids |
| `id` correlation | request `id` ≥ 1 and monotonically increasing per connection; replies echo it; unsolicited messages use `id: 0` |

---

## 6. Session channel

### 6.1 Framing

Newline-delimited UTF-8 JSON. One object per line, `\n` terminated.

```
{"type":"<MsgType value>","id":<int>, ...payload fields at the top level...}
```

`Message.encode()` writes `type` and `id` first, then splats `data` at the **top level** — the
payload is *not* nested under a `data` key. `json.dumps` is called with
`separators=(",", ":")` (compact) and `allow_nan=False`.

`Message.decode()` pops `type` and `id`; **everything else becomes `data`**.

`MAX_SESSION_LINE_LEN = 65536`, enforced on both encode and decode.

`LineReader` reassembles messages from the TCP byte stream, because TCP gives you a byte
stream and not messages — a single `recv` can deliver half a line or three of them. Use it;
do not reinvent it.

### 6.2 Message catalogue

Direction, and whether the package pins the payload shape.

| `MsgType` | Value | Direction | Shape pinned by source? |
|---|---|---|---|
| `HELLO` | `hello` | app → car | **yes** — `hello()` |
| `HELLO_ACK` | `hello_ack` | car → app | **yes** — `hello_ack()` |
| `ERROR` | `error` | car → app | **yes** — `error()` |
| `ACK` | `ack` | car → app | partly — `ack(id, **extra)` |
| `ARM` | `arm` | app → car | no |
| `DISARM` | `disarm` | app → car | no |
| `ESTOP` | `estop` | app → car | no |
| `CLEAR_ESTOP` | `clear_estop` | app → car | no |
| `CLEAR_FAULTS` | `clear_faults` | app → car | no |
| `GET_PARAMS` | `get_params` | app → car | no |
| `SET_PARAMS` | `set_params` | app → car | **yes** — `set_params()` |
| `PARAMS` | `params` | car → app | **yes** — `params()` |
| `CALIBRATE` | `calibrate` | app → car | no |
| `CALIBRATION_STATUS` | `calibration_status` | car → app | no |
| `RESET_ODOM` | `reset_odom` | app → car | no |
| `STATE` | `state` | car → app, unsolicited | **yes** — `state()` |
| `PING` | `ping` | either | no |
| `PONG` | `pong` | either | no |

### 6.3 The pinned shapes, verbatim

```json
{"type":"hello","id":1,"proto":2,"app_version":"2.0.0","driver":"zhaojin",
 "auth":"<64 hex chars>","telemetry_port":4211}

{"type":"hello_ack","id":1,"proto":2,"car_id":"telekart-01","fw_version":"2.0.0",
 "session_id":48879,"session_token":"<32 hex chars>",
 "caps":["video","calibrate","params"],"video_port":4213,"control_port":4210}

{"type":"error","id":7,"code":"param_out_of_range",
 "detail":"pid_kp=0.5 is above the maximum 0.1"}

{"type":"ack","id":12}

{"type":"set_params","id":3,"values":{"pid_kp":0.006,"closed_loop":true}}

{"type":"params","id":3,"values":{"pid_kp":0.006,"closed_loop":true},"applied":true}

{"type":"state","id":0,"state":3,"faults":32,"detail":"link stale"}
```

`state()` always uses `id: 0` — it is unsolicited by construction.

> **`PARAMS` is authoritative.** The app must render **this**, never the value it
> optimistically typed. A UI showing a setting the car never accepted is how you end up
> debugging the wrong vehicle.

### 6.4 Unpinned shapes — suggested minimum

The package does not constrain these. Both ends must agree; these are the shapes this
documentation set assumes.

| Message | Suggested payload |
|---|---|
| `ARM` / `DISARM` / `ESTOP` / `CLEAR_ESTOP` / `CLEAR_FAULTS` / `RESET_ODOM` | none; reply `ACK`, or `ERROR` with `not_allowed_in_state` |
| `GET_PARAMS` | none, or `{"group": "pid"}`; reply is a `PARAMS` message with the same `id` |
| `CALIBRATE` | `{"routine": "drive", "on_ground": false}`; reply `ACK`, then a stream of `CALIBRATION_STATUS` |
| `CALIBRATION_STATUS` | `{"id": <the CALIBRATE id>, "phase": "left_fwd", "progress": 0.35, "done": false}`; final message carries `"done": true` and either a `result` or an `error` |
| `PING` | `{"t": <client monotonic µs>}`; `PONG` echoes `id` and `t` |

### 6.5 `ErrorCode`

| Value | When |
|---|---|
| `protocol_version` | `proto` mismatch in `HELLO` |
| `auth_failed` | `auth` did not verify |
| `bad_request` | malformed or missing field |
| `unknown_param` | `SET_PARAMS` named a parameter not in the registry |
| `param_out_of_range` | value outside the registry's `minimum`/`maximum` |
| `not_allowed_in_state` | e.g. `ARM` while a critical fault is latched, or changing a `requires_disarm` parameter while armed |
| `busy` | e.g. `CALIBRATE` while a calibration is already running |
| `internal` | |

### 6.6 Implementation notes that will bite you

- **`Message.encode()` raises `ValueError`, not `SessionError`, on NaN or infinity**, because
  `json.dumps` is called with `allow_nan=False`. Sanitize floats *before* you build the
  message — a NaN that reaches a params push takes the connection down.
- **`LineReader.feed()` raises `SessionError` mid-batch**, discarding any messages it had
  already decoded in that call. That is consistent with the intent — malformed session traffic
  is fatal to the session — so catch it per connection and tear the connection down rather
  than trying to resynchronise.
- `LineReader` also raises if the buffer exceeds its limit with no newline in it, and clears
  the buffer when it does.
- Blank lines are skipped silently.
- `Message.require(key, kind)` is the typed accessor; it raises `SessionError` with a message
  naming the field and both types.

---

## 7. Video framing

```python
struct.Struct("<IIQHHI")      # VIDEO_HEADER_LEN == 24
```

A 24-byte header ahead of each Annex-B access unit (or JPEG).

### 7.1 Byte layout

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 4 | `u32` | `magic` — `MAGIC_VIDEO` |
| 4 | 4 | `u32` | `sequence` — monotonically increasing; **gaps mean drops** |
| 8 | 8 | `u64` | `pts_us` — capture timestamp, `CLOCK_MONOTONIC` on the car |
| 16 | 2 | `u16` | `flags` — `VideoFrameFlags` |
| 18 | 2 | `u16` | `codec` — `VideoCodec` |
| 20 | 4 | `u32` | `length` — payload bytes that follow |

`MAX_VIDEO_FRAME_LEN = 4 MiB`; a header claiming more is rejected as implausible.

### 7.2 Why a header at all

Raw Annex-B would work — PyAV's parser finds frame boundaries unaided. The header buys two
things worth having every day of tuning:

1. **A per-frame latency readout on the HUD**, from `pts_us`.
2. **An explicit "frames were dropped" bit**, so the decoder knows to expect corruption rather
   than silently rendering garbage.

It also means the stream goes **straight into a `CodecContext` with no container and no
demuxer**, which removes one to two frames of buffering ([tuning.md §5.2](tuning.md)).

### 7.3 `FrameReader` and resynchronisation

`FrameReader.feed(chunk)` returns a list of `(FrameHeader, payload)` tuples. On a corrupted
stream it **drops one byte and hunts for the next magic word** rather than wedging
permanently. When no magic is found it retains the last 3 bytes, since a magic word can be
split across two `recv` calls.

A video link that recovers on its own is worth the twenty lines.

### 7.4 Worked example

`pack_frame(42, 0x11223344, b"\x00\x00\x00\x01\x65payload", keyframe=True)`:

```
offset  bytes                     field
  0     54 58 56 44               magic       'TXVD'
  4     2a 00 00 00               sequence    42
  8     44 33 22 11 00 00 00 00   pts_us      0x11223344
 16     01 00                     flags       KEYFRAME
 18     00 00                     codec       H264
 20     0c 00 00 00               length      12
 24     ... 12 bytes of payload ...
```

### 7.5 `pts_us` is a car-side monotonic clock

Excellent for frame-to-frame jitter and stall detection. **Useless for absolute glass-to-glass
latency** unless the two clocks are disciplined — use the phone-stopwatch method in
[tuning.md §5.1](tuning.md) for that.

---

## 8. Parameter registry

One registry, shared by both ends. The firmware validates incoming `SET_PARAMS` against it;
the desktop app builds its tuning UI **from** it, so adding a parameter here makes it appear in
the app with the right widget, range and units without touching any UI code.

### 8.1 `ParamDef`

| Field | Meaning |
|---|---|
| `name`, `group`, `label` | identity and UI placement |
| `kind` | `"float"` \| `"int"` \| `"bool"` \| `"enum"` |
| `default`, `minimum`, `maximum`, `step`, `unit`, `choices` | widget configuration |
| `description` | tooltip |
| `advanced` | hidden behind an "advanced" toggle |
| **`measured`** | **determined by a bring-up measurement, not by taste** — see [calibration.md](calibration.md) |
| **`requires_disarm`** | **changing it while armed is refused by the firmware** |

`GROUPS = ("drive", "pid", "steering", "geometry", "safety", "pwm", "video")` — 51 parameters
in total.

> Defaults are **conservative starting points, not tuned values.** Anything marked
> `measured=True` is expected to be overwritten by a bring-up procedure — treating a guess as a
> measurement is how you spend an afternoon tuning a PID against the wrong plant gain.

### 8.2 Validation semantics

```python
coerce(name, value) -> Any                # raises ParamError
coerce_all(values)  -> dict               # reports EVERY problem at once
merged_with_defaults(values) -> dict      # overlays onto defaults, ignoring unknown keys
unknown_or_invalid(values) -> list[str]   # for logging
defaults() -> dict
group(name) -> list[ParamDef]
```

Three behaviours worth internalising:

- **Out-of-range values are rejected, not clamped.** *"A control packet's axis value gets
  clamped because dropping it would be worse; a parameter push is deliberate, and silently
  accepting a different number than the operator typed is how the UI ends up lying about the
  car."* This is the exact opposite of `ControlPacket.unpack`'s behaviour, deliberately.
- **`coerce_all` reports every failure**, not just the first, so the app does not make the
  operator fix a form one field per round trip.
- **`merged_with_defaults` ignores unknown keys.** Loading a config written by an older or
  newer build: an unrecognised key is a reason to log, not a reason to refuse to boot. Use
  `unknown_or_invalid()` to produce that log line.

`bool` is checked strictly: `coerce("closed_loop", 1)` raises. And because `isinstance(True,
int)` is true in Python, numeric parameters explicitly reject `bool` as well.

### 8.3 The registry

#### `drive`

| Name | Kind | Default | Min | Max | Step | Unit | Flags |
|---|---|---|---|---|---|---|---|
| `max_duty` | float | 0.85 | 0.1 | 1.0 | 0.05 | — | — |
| `duty_sum_max` | float | 1.4 | 0.2 | 2.0 | 0.05 | — | — |
| `closed_loop` | bool | true | — | — | — | — | — |
| `throttle_expo` | float | 1.8 | 0.5 | 3.0 | 0.1 | — | — |
| `throttle_deadband` | float | 0.04 | 0.0 | 0.2 | 0.01 | — | — |
| `accel_rpm_per_s` | float | 250.0 | 50.0 | 2000.0 | 25.0 | RPM/s | — |
| `decel_rpm_per_s` | float | 700.0 | 50.0 | 3000.0 | 25.0 | RPM/s | — |
| `brake_strength` | float | 0.4 | 0.0 | 1.0 | 0.05 | — | — |
| `reverse_enabled` | bool | true | — | — | — | — | — |
| `pit_duty` | float | 0.25 | 0.05 | 0.6 | 0.05 | — | — |

#### `pid`

| Name | Kind | Default | Min | Max | Step | Unit | Flags |
|---|---|---|---|---|---|---|---|
| `pid_kp` | float | 0.005 | 0.0 | 0.1 | 0.001 | duty/RPM | advanced |
| `pid_ki` | float | 0.02 | 0.0 | 0.5 | 0.001 | duty/(RPM*s) | advanced |
| `pid_kd` | float | 0.0 | 0.0 | 0.05 | 0.001 | duty*s/RPM | advanced |
| `pid_i_clamp` | float | 0.4 | 0.0 | 1.0 | 0.05 | — | advanced |
| `straight_sync_gain` | float | 30.0 | 0.0 | 200.0 | 5.0 | RPM/m | advanced |

#### `steering`

| Name | Kind | Default | Min | Max | Step | Unit | Flags |
|---|---|---|---|---|---|---|---|
| `steer_center_us` | int | 1500 | 900 | 2100 | 1 | us | measured |
| `steer_min_us` | int | 1200 | 700 | 2400 | 1 | us | measured |
| `steer_max_us` | int | 1800 | 700 | 2400 | 1 | us | measured |
| `steer_trim_us` | int | 0 | -200 | 200 | 1 | us | — |
| `steer_max_deg` | float | 24.0 | 5.0 | 45.0 | 1.0 | deg | measured |
| `steer_rate_us_per_s` | float | 2000.0 | 200.0 | 6000.0 | 100.0 | us/s | — |
| `steer_expo` | float | 1.0 | 0.5 | 3.0 | 0.1 | — | — |
| `steer_deadzone` | float | 0.03 | 0.0 | 0.2 | 0.01 | — | — |
| `steer_speed_reduction` | float | 0.45 | 0.0 | 0.8 | 0.05 | — | — |
| `steer_hold_us` | int | 8 | 0 | 40 | 1 | us | advanced |
| `steer_invert` | bool | false | — | — | — | — | — |
| `servo_relax_when_disarmed` | bool | true | — | — | — | — | — |

#### `geometry`

| Name | Kind | Default | Min | Max | Step | Unit | Flags |
|---|---|---|---|---|---|---|---|
| `wheel_diameter_m` | float | 0.065 | 0.02 | 0.3 | 0.001 | m | measured, requires_disarm |
| `wheelbase_m` | float | 0.2 | 0.05 | 1.0 | 0.001 | m | measured, requires_disarm |
| `track_width_m` | float | 0.15 | 0.05 | 1.0 | 0.001 | m | measured, requires_disarm |
| `encoder_cpr` | int | 660 | 1 | 20000 | 1 | — | measured, requires_disarm |
| `invert_left` | bool | false | — | — | — | — | requires_disarm |
| `invert_right` | bool | false | — | — | — | — | requires_disarm |
| `encoder_invert_left` | bool | false | — | — | — | — | requires_disarm |
| `encoder_invert_right` | bool | false | — | — | — | — | requires_disarm |

#### `safety`

| Name | Kind | Default | Min | Max | Step | Unit | Flags |
|---|---|---|---|---|---|---|---|
| `control_timeout_ms` | int | 200 | 50 | 1000 | 10 | ms | — |
| `failsafe_brake_duty` | float | 0.35 | 0.0 | 1.0 | 0.05 | — | — |
| `stall_detect_ms` | int | 600 | 100 | 2000 | 50 | ms | — |
| `stall_rpm_threshold` | float | 5.0 | 0.0 | 50.0 | 1.0 | RPM | advanced |
| `low_battery_v` | float | 6.0 | 0.0 | 30.0 | 0.1 | V | — |
| `critical_battery_v` | float | 5.4 | 0.0 | 30.0 | 0.1 | V | — |
| `arm_neutral_ms` | int | 500 | 0 | 3000 | 50 | ms | — |

#### `pwm`

| Name | Kind | Default | Min | Max | Step | Unit | Flags |
|---|---|---|---|---|---|---|---|
| `pwm_hz` | int | 1000 | 500 | 8000 | 100 | Hz | advanced, requires_disarm |
| `direction_deadtime_ms` | int | 30 | 0 | 200 | 5 | ms | advanced |
| `reverse_allowed_rpm` | float | 15.0 | 0.0 | 100.0 | 1.0 | RPM | advanced |

#### `video`

| Name | Kind | Default | Choices / Min | Max | Step | Unit | Flags |
|---|---|---|---|---|---|---|---|
| `video_codec` | enum | `h264` | `h264`, `mjpeg` | — | — | — | requires_disarm |
| `video_width` | int | 640 | 160 | 1920 | 16 | — | requires_disarm |
| `video_height` | int | 480 | 120 | 1080 | 16 | — | requires_disarm |
| `video_fps` | int | 30 | 5 | 60 | 1 | fps | requires_disarm |
| `video_bitrate` | int | 2000000 | 200000 | 12000000 | 100000 | bps | requires_disarm |
| `video_iperiod` | int | 15 | 1 | 120 | 1 | frames | advanced, requires_disarm |

---

## 9. Timing constants and failsafe schedule

```python
CONTROL_RATE_HZ       = 100
TELEMETRY_RATE_HZ     = 50
CONTROL_LOOP_HZ       = 100

CONTROL_TIMEOUT_MS    = 200      # no valid control packet -> the link is stale
FAILSAFE_BRAKE_AT_MS  = 50       # measured from the moment the link goes stale
FAILSAFE_COAST_AT_MS  = 450
FAILSAFE_DISARM_AT_MS = 1000

SESSION_TTL_S         = 300
```

`CONTROL_TIMEOUT_MS` was **relaxed from the old ESP32 firmware's 120 ms**: Pi Zero 2 W WiFi is
markedly lumpier than the ESP32's, and a spurious failsafe mid-corner is its own hazard.

The `FAILSAFE_*_AT_MS` values are measured **from the moment the link goes stale**, and the
schedule is coast → brake → coast → disarm:

| Since stale | Since last good packet | Action |
|---:|---:|---|
| 0 ms | 200 ms | **coast**, state → `FAILSAFE` |
| 50 ms | 250 ms | **brake** at `failsafe_brake_duty` (0.35) |
| 450 ms | 650 ms | **coast** |
| 1000 ms | 1200 ms | **disarm**, state → `SAFE` |

Coast *then* brake rather than braking immediately, because an unbraked car keeps rolling for
metres — and coast again afterwards so a car left braking does not cook the bridge.
[bringup.md](bringup.md) §8.1 is the test.

---

## 10. Size limits

| Constant | Value | Used for |
|---|---:|---|
| `MAX_CONTROL_PACKET_LEN` | 128 | receive-buffer sizing; the packet is 40 |
| `MAX_TELEMETRY_PACKET_LEN` | 256 | receive-buffer sizing; the packet is 98 |
| `MAX_SESSION_LINE_LEN` | 65 536 | enforced on encode and decode |
| `MAX_VIDEO_FRAME_LEN` | 4 194 304 | a header claiming more is rejected |

The first two are validation bounds, not packet sizes — they leave room for a future field
without every receiver needing a new buffer size.

---

## 11. Error handling contract

| Layer | On bad input |
|---|---|
| `ControlPacket.unpack` / `TelemetryPacket.unpack` | raise `ProtocolError`. **Callers drop the datagram.** Never let it escape the receive path. |
| Out-of-range command values | **clamped**, never rejected |
| `FrameHeader.unpack` | raise `VideoProtocolError`; `FrameReader` resynchronises rather than tearing down |
| Video stream unparseable at a higher level | tear the TCP connection down |
| `Message.decode` / `LineReader.feed` | raise `SessionError`. **Tear the session down.** |
| `coerce` / `coerce_all` | raise `ParamError`. Out-of-range values are **rejected**, never clamped |
| Unknown fault / flag bits in telemetry | **preserved verbatim** (`IntFlag` with `boundary=KEEP`) |
| Unknown `VehicleState` value | mapped to `VehicleState.FAULT` |
| NaN / infinity in `from_si` | mapped to 0 |

The through-line: **fail loudly at configuration time, never inside a control loop or a
packet-decode path.** Hot paths clamp, drop, and set a fault flag instead
([INTERFACES.md §0](INTERFACES.md)).

---

## 12. Testing

Golden-byte tests in both directions, plus `struct.calcsize` assertions, **run by both sides'
suites** — so a layout change that is not reflected on both ends fails immediately rather than
on the first drive. The worked examples in §2.5, §3.6 and §7.4 are reproducible fixtures for
exactly that.

The package has **zero dependencies and targets Python 3.11+** on purpose: it has to install
on a Pi Zero 2 W without a compiler and on macOS without Homebrew, and it must import cleanly
on both the firmware's 3.11 and the desktop app's 3.12. No PEP 695 generics, no `type`
statements.
