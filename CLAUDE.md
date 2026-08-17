# TeleKart2

Teleoperated RC car. A pygame app on the Mac shows the Pi camera feed and drives the car with a USB
steering wheel — analog steering, throttle and brake, plus paddle-shifted gears. WASD still works as
a fallback. Live and driving.

---

## Rules that override convenience

### 1. The car is deployed and live

`telekart.local` (192.168.1.173, user `telekart`) runs our code from `~/telekart2/` as the
**`telekart2` systemd service**, enabled at boot alongside `pigpiod`. Bring-up is finished; the
earlier "do not touch the Pi" isolation rule was lifted 2026-08-16.

```bash
systemctl status telekart2
journalctl -u telekart2 -f          # live 1 Hz summary
sudo systemctl restart telekart2    # after changing code on the Pi
sudo systemctl stop telekart2       # Restart=always, so pkill just respawns it
```

Still true, and still the reason to be careful: **never run a second control process while the
service is up.** Two processes driving the same GPIO pins is a hardware-damaging failure mode, and
two bound to UDP 8091 split the command stream so each sees phantom packet loss.

### 2. Our code lives in `~/telekart2/` on the Pi

Plus `/etc/systemd/system/telekart2.service`, which is a copy of `pi/telekart2.service` — edit the
version in this repo, deploy, then `sudo cp` and `daemon-reload`. Ports are **8090** (video) and
**8091** (control).

### 3. Develop against the simulator

`sim/fake_pi.py` runs on the Mac and speaks the identical protocol with a synthetic camera. All
development and verification happens against it. This is what makes rule 1 workable.

### 4. Keep it small

**Not built, deliberately:** odometry, closed-loop speed control, H.264, video recording, autonomy.
Encoders are read by `pi/encoder_probe.py` for measurement only — the control loop does not use them.
If a change feels like it needs a new abstraction layer, ask first.

### 5. There is no prior version of this project

TeleKart2 is greenfield. Do not look for, reference, or carry anything over from other directories.

### 6. Measure before concluding

This project has produced three confidently-wrong diagnoses that measurement overturned: WiFi power
save, telemetry cadence, and "the car is current-limited". Two came from trusting the hardware brief,
one from an instrument that was adding the latency it measured. **Sanity-check the instrument against
a case where the answer is known** — loopback should read ~0 ms — before trusting it elsewhere.

---

## Architecture

```
MAC (this repo)                                  PI  (~/telekart2/, live)
┌──────────────────────────────┐                 ┌───────────────────────────────┐
│ mac/main.py   pygame window  │  MJPEG /stream  │ video.py   picamera2 → JPEG   │
│  video pane   ◀──────────────┼─── HTTP :8090 ──┤                               │
│  WASD keys   ──── cmd JSON ──┼─── UDP :8091 ──▶│ main.py    control @100 Hz    │
│  debug panel ◀─── tlm JSON ──┼─── UDP  ────────┤ watchdog · slew · arm gate    │
│  logs/*.jsonl                │                 │ car.py     pigpio → L298N     │
└──────────────────────────────┘                 │                    → HS-311   │
        ▲  identical protocol                    └───────────────────────────────┘
┌───────┴──────────────────────┐
│ sim/fake_pi.py  (on the Mac) │
└──────────────────────────────┘
```

**The Mac sends raw intent; the Pi computes all vehicle behavior.** Ramping, deadband, duty mapping,
direction guards and limits live in exactly one place — the side that owns the hardware. The Mac never
computes a PWM value. Keep it that way: it is what lets v2 swap key-booleans for wheel-axis floats
without touching the Pi.

## Layout

| Path | What |
|---|---|
| `common/protocol.py` | Constants + encode/decode. Shared by `mac/` and `sim/`. |
| `pi/protocol.py` | Byte-identical copy — the Pi is a separate install. `make check-protocol` diffs them. |
| `pi/config.py` | Pins, ports, limits, servo calibration. Single source of truth for the Pi. |
| `pi/car.py` | pigpio: motors, servo, failsafe primitives. |
| `pi/video.py` | picamera2 → MJPEG HTTP server. |
| `pi/main.py` | Control loop, watchdog, telemetry, logging, signal handling. |
| `mac/main.py` | pygame loop: input, UDP, HUD, logging. |
| `mac/video_client.py` | MJPEG reader thread. |
| `mac/xinput.py` | Reads the wheel over raw USB — macOS exposes XInput to nothing else. |
| `mac/calibrate_input.py` | Works out what a wheel's axes and buttons actually do. |
| `sim/fake_pi.py` | Synthetic camera + full protocol, runs on the Mac. |
| `pi/servo_probe.py` | Holds the servo at given pulse widths, for steering calibration. |
| `pi/encoder_probe.py` | Wheel speed and encoder calibration. Read-only, safe to run while driving. |
| `pi/telekart2.service` | systemd unit; copy to `/etc/systemd/system/` and `daemon-reload`. |
| `docs/PROTOCOL.md` | Exact wire format. |
| `docs/HARDWARE.md` | **Measured hardware values vs unverified claims. Read this first.** |
| `docs/PI_SETUP.md` | Setup runbook — executed 2026-08-16. |

## Commands

```
make venv             # Mac venv + deps
make sim              # run the simulated Pi
make drive            # run the pygame driving app  -> targets the SIMULATOR
make check-protocol   # verify pi/ copies match common/
make drive-car        # bring-up only: targets the real car
```

`mac/config.py` defaults `HOST` to `127.0.0.1`, never `telekart.local`. Addressing the car has to be
a deliberate act — `TELEKART_HOST=telekart.local` or `make drive-car` — because a default that is
only safe when you remember to override it is not a safe default. Do not change this.

## Measured hardware facts — [docs/HARDWARE.md](docs/HARDWARE.md)

**Read that before trusting any hardware number, including the ones below.** The original hardware
brief has been demonstrably wrong twice (regulator voltage, top speed) and its encoder and steering
figures are wrong too. `docs/HARDWARE.md` separates measured values from unverified claims.

Headlines: regulator is **12 V** (not 9 V), no-load top speed is **307 RPM / ~1.05 m/s** (not
150–200 RPM), encoders give **390** rising edges/rev on one channel (not 660), servo centre is
**1950 µs** (not 1500). The car performs within 3.5% of what its voltage predicts — **nothing is
limiting it.**

## Hardware traps the Pi code must respect

Each of these has bitten someone. Where a claim is **unverified** it now says so — several came from
the same brief that got the regulator voltage and top speed wrong.

- **Coast is `EN` low.** On the L298, `IN1 == IN2` with EN high is a *brake*. Both HIGH or both LOW
  shorts the motor through the bridge either way. This inverts most people's intuition.
- **GPIO5/6 boot HIGH** on BCM283x, so IN1/IN2 are asserted from the instant the Pi has power. External
  10 kΩ pull-downs on ENA/ENB/IN1–IN4 are mandatory, and `car.py` must drive all six pins low as its
  very first action.
- **Use pigpio's DMA PWM** (`set_PWM_frequency`/`set_PWM_dutycycle`) at 1 kHz on GPIO12/13, *not*
  `hardware_PWM()`. The two hardware channels share one clock divider, and `snd_bcm2835` is said to
  claim the PWM peripheral. *(Unverified here — `snd_bcm2835` is not even loaded on this Pi. DMA PWM
  works, so the question never arose.)*
- **The servo is on the Pi's own 5 V rail.** A stalled HS-311 can brown out the SoC. Stop sending pulses
  when disarmed for >2 s. *(Plausible and cheap to respect; never actually triggered here.)*
- **The regulator sustains ~1.5 A for both motors combined** — *unverified*, and it is the stated
  reason for the slew limiter. The car reaches its full voltage-predicted speed at 100% duty with
  `throttled=0x0`, so this limit has never visibly bound. Keep the slew limiter anyway; it costs
  nothing and an INA219 would settle the question.
- **Nothing moves below ~30% duty** — the L298N's Darlington stage drops ~1.4 V. Map throttle into
  `MIN_DUTY..MAX_DUTY`, don't map from zero. *(The ~1.4 V is corroborated indirectly: measured no-load
  RPM implies ~1.8 V total loss including friction.)*
- **Camera FOV:** request `raw={'size': (1640, 1232)}` so picamera2 uses the full-FOV binned mode. A bare
  640×480 request lands on a 1280×960 *crop* — a ~2.6× telephoto that is genuinely hard to drive.
  *(Confirmed: visible in the sensor mode table and libcamera's own log line.)*
- **The CSI ribbon is intermittent.** libcamera reported `Camera frontend has timed out` after 7.5
  minutes of clean running; a reboot cleared it. Reseat the connector before a long session.

## Pin map (BCM)

| Function | Pin |
|---|---|
| ENA / ENB | 12 / 13 |
| Left IN1 / IN2 | 5 / 6 |
| Right IN3 / IN4 | 20 / 21 |
| Steering servo | 18 |
| Status LED | 25 |
| E-stop button | 16 (code present, `ESTOP_ENABLED = False` until polarity is confirmed) |
| Left encoder A / B | 23 / 24 — **working**, 390 rising edges/rev on A |
| Right encoder A / B | 27 / 22 — **working**, same |

Both encoders are **wired and working**; the control loop does not read them.

## Conventions

- Both sides log every message to `logs/session-<ts>.jsonl`, one JSON object per line, plus a 1 Hz
  human-readable summary to stdout. Two independent captures of the same conversation is what turns
  "it worked on the Mac but the car didn't move" into a five-second diagnosis.
- `arm` is transmitted as a **level, not an edge** — UDP drops packets, so a latched toggle sent once
  would desync. The Mac holds the latch; every packet restates it.
- Telemetry echoes the Mac's own timestamp back as `ack_ts`, so round-trip latency needs no clock sync.
- Plain standard-library Python wherever possible. Mac deps: `pygame`, `requests`. Pi deps: `pigpio` and
  `picamera2`, both from **apt** — picamera2 is not reliably pip-installable, so the Pi venv is created
  with `--system-site-packages`.
