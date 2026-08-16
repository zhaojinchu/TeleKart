# TeleKart v2

A teleoperated RC car you drive from a desk. A Raspberry Pi Zero 2 W runs a 100 Hz closed-loop
controller over WiFi; a macOS station renders a low-latency camera feed and a live telemetry HUD
and takes input from a force-feedback wheel and pedals.

The whole system is built around one idea: **the car owns safety, the desk owns feel.** The
firmware clamps duty, limits servo slew, detects stalls and runs the failsafe ramp. The app owns
curves, deadzones, smoothing and expo. Neither side does the other's job, and neither side
filters what the other already filtered — that division is why the loop is tunable at all.

Everything that crosses the network is defined once, in `packages/telekart_protocol/`, and
imported unmodified by all three programs. Its test suite asserts hard-coded byte strings in both
directions and is run by both halves of the system, so a change that isn't reflected on both ends
fails immediately rather than at 3 m/s.

---

## What it actually is

| | |
|---|---|
| **Vehicle** | Raspberry Pi Zero 2 W (Bookworm, Python 3.11) · L298N dual H-bridge · 2× GA37-520 12 V gearmotors with Hall quadrature encoders · HS-311 steering servo · Pi Camera Module 2 (IMX219) |
| **Power** | 7.2 V NiMH → Pololu S18V20ALV boost set to 9 V → L298N. The Pi runs from a separate USB powerbank. |
| **Link** | 2.4 GHz WiFi. UDP for control and telemetry, TCP for the session and the video stream, mDNS for discovery. |
| **Station** | macOS, Python 3.12, PySide6 + PyAV + SDL through pygame-ce. |
| **Control** | Per-wheel feedforward + PID on encoder RPM, with an electronic differential. Feedforward carries the load; the PID only trims. |
| **Odometry** | Kinematic bicycle model, midpoint integration, dead-reckoned from the encoders. |

**Honest performance numbers.** The GA37-520's nameplate is 360 RPM at 12 V. This car will not
see it. The boost regulator sustains about **1.5 A for both motors combined**, and the L298N is a
Darlington bridge that eats ~1.4 V before the motor sees anything, so real top speed is
**150–200 RPM**. Nothing in this codebase hardcodes a top speed: auto-calibration measures it,
telemetry publishes `v_max_mm_s` in every packet, and the speedometer scales itself off that. Fit
a MOSFET bridge later and both sides adapt with no code change.

Dead reckoning without an IMU drifts. Expect **5–15 % closure error on a 5 m square**. There is a
`heading_source` hook so a complementary-filtered MPU-6050 can be dropped in later.

---

## Architecture

```
┌── macOS driving station · app/ ──────────────────────────────────────────────┐
│                                                                              │
│   wheel + pedals ──▶ InputThread 250 Hz ──▶ ControlTxThread 100 Hz           │
│   SDL via pygame-ce  calibrate · deadzone    quantize · sequence · HMAC      │
│                      curve LUT · rate limit  owns the RTT ring               │
│                      one-euro smoothing                                      │
│                                                                              │
│   VideoRxThread ─────┐                                                       │
│   PyAV · LOW_DELAY   │                                                       │
│   thread_type='NONE' ├──▶ AppModel 60 Hz ──▶ Qt HUD · map · tuning UI        │
│                      │    immutable snapshots, one consistent set            │
│   TelemetryRxThread ─┘    per repaint — never a torn mix                     │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
        │ UDP  4210  control    100 Hz     ▲ UDP 4211  telemetry  50 Hz
        │ TCP  4212  session, params, arm  │ TCP 4213  framed H.264
        ▼            dropping it = E-stop  │ mDNS _telekart._tcp.local.
┌── Raspberry Pi Zero 2 W · pi/ ───────────────────────────────────────────────┐
│                                                                              │
│   telekart-control  ── asyncio main thread: UDP · TCP · watchdog             │
│   oom_score -900       SafetyStateMachine: arm · failsafe ramp · faults      │
│                        │                                                     │
│                        │  single-slot mailbox, no locks                      │
│                        ▼                                                     │
│                     ControlThread 100 Hz · SCHED_FIFO 50 · core 3            │
│                        shaping → mixer → feedforward+PID → deadband          │
│                        comp → duty budget → clamp → dead-time seq            │
│                        │                                                     │
│   telekart-video       ▼                                                     │
│   oom_score +500    pigpiod ─── hardware PWM pair ──▶ L298N ──▶ 2x GA37-520  │
│   picamera2 →          │        GPIO12 + GPIO13       │        gearmotors    │
│   H.264 hw encode      │                              │            │         │
│        ▲               └─ DMA servo pulse ──▶ HS-311  │            │         │
│        │                  GPIO18                      │            ▼         │
│   ┌────┴───┐              encoder edges ◀─────────────┴──── GPIO23/24        │
│   │ IMX219 │              GPIO27/22                              x2 decode   │
│   └────────┘                                                                 │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

Two processes on the Pi, not one, because the GIL is real: camera work must not inject jitter
into the 100 Hz loop, and the OOM policy has to be able to guarantee the kernel kills the camera
and never the car.

---

## Repo map

```
TeleKart/
├── packages/telekart_protocol/   The wire format. Standard library only, zero deps,
│   ├── telekart_protocol/          importable on both 3.11 and 3.12.
│   │   ├── constants.py            ports, versions, enums, failsafe schedule
│   │   ├── control.py              40-byte control packet   (app → car, 100 Hz UDP)
│   │   ├── telemetry.py            98-byte telemetry packet (car → app, 50 Hz UDP)
│   │   ├── session.py              newline-delimited JSON over TCP
│   │   ├── video.py                24-byte frame header + resynchronising reader
│   │   ├── params.py               the one parameter registry both ends build from
│   │   └── crypto.py               truncated HMAC + per-session key derivation
│   └── tests/                    Golden byte strings. Run by BOTH sides' suites.
│
├── pi/                           Vehicle firmware.
│   ├── telekart/
│   │   ├── hal/                    GpioBackend: pigpio on the car, MockBackend on a Mac
│   │   ├── drivers/                motors, servo, quadrature encoders
│   │   ├── control/                PID, shaping, differential mixer, safety state machine
│   │   ├── net/                    UDP control/telemetry, TCP session, mDNS advert
│   │   ├── util/                   injected Clock, deadline scheduler, RT scheduling
│   │   ├── odometry.py  calibration.py  config.py  app.py
│   ├── telekart_video/           The camera process.
│   ├── config/                   telekart.yaml (checked in) · config.local.yaml and
│   │                             calibration.yaml (per-vehicle, git-ignored)
│   ├── systemd/                  Units. ExecStopPost= is a safety layer, not tidiness.
│   ├── native/                   Optional C encoder helper
│   └── scripts/                  Bring-up and calibration utilities
│
├── app/telekart_app/             macOS driving station.
│   ├── core/                       LatestBox, PacedLoop — the threading primitives
│   ├── input/                      calibration → curve → filter chain, and force feedback
│   ├── net/                        control TX, telemetry RX, session client, discovery
│   ├── video/                      PyAV decode and FrameBundle lifetime management
│   ├── model/                      AppModel: the only object widgets talk to
│   ├── ui/                         screens, HUD gauges, dialogs, theme tokens
│   ├── race/  storage/             lap timing, session recording
│   └── config/  assets/
│
├── tools/telekart_sim/           A protocol-identical fake car. Same ports, same
│                                 packets, same mDNS, real H.264. Deterministic
│                                 under --seed.
├── docs/                         See below.
└── Makefile                      make help
```

---

## Quickstart — no hardware

You need nothing but a Mac. The simulator speaks the real protocol on the real ports, so the
entire app is exercised end to end: discovery, handshake, arming, control, telemetry, video
decode.

```sh
git clone <this repo> TeleKart && cd TeleKart

make setup-app      # creates app/.venv on Python 3.12 — pulls Qt, give it a minute
make setup-sim      # creates tools/telekart_sim/.venv
```

Then, in two terminals:

```sh
make run-sim ARGS="--seed 1"     # terminal 1: a fake car appears on the network
make run-app                     # terminal 2: it shows up in the discovery list
```

The point of the simulator is that it can be **worse than reality on demand**. Every one of these
is a bug class you would otherwise only meet outdoors, at speed:

```sh
make run-sim ARGS="--seed 1 --packet-loss 0.05 --latency 80 --jitter 30"
make run-sim ARGS="--seed 1 --video-stall 2.0 --tcp-drop 15"
make run-sim ARGS="--seed 1 --encoder-fault left --wheelspin 0.3"
make run-sim ARGS="--seed 1 --reject-arm --battery-drain 0.02"
make run-sim ARGS="--realtime 0 --seed 1"    # as fast as possible, for CI
```

`--seed` makes the physics and the noise deterministic, which is what lets integration tests
assert exact numbers instead of ranges.

Run the tests while you are here — they need no hardware and no display:

```sh
make test
```

---

## Quickstart — the real car

> Read [`docs/wiring.md`](docs/wiring.md) and [`docs/bringup.md`](docs/bringup.md) first. The
> build sheet has a VERIFY block after every subsystem; each one exists because that failure was
> diagnosed the hard way at least once.

**On the Pi**, once it is wired and on the network:

```sh
sudo apt install -y git pigpio python3-picamera2
sudo systemctl enable --now pigpiod

# Mandatory. snd_bcm2835 otherwise claims BOTH hardware PWM channels and the
# motors get neither.
echo 'dtparam=audio=off' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

After the reboot:

```sh
git clone <this repo> TeleKart && cd TeleKart
make setup-pi                 # venv with --system-site-packages, so picamera2 stays visible

# The shared secret. Put it in a file the service reads, not in your shell —
# an `export` on the command line lands in .bash_history and in `ps`.
install -m 600 /dev/null ~/.telekart-env
echo "TELEKART_SHARED_KEY=something only you know" > ~/.telekart-env

set -a; . ~/.telekart-env; set +a
pi/.venv/bin/telekart-control
```

**Wheels off the ground.** Blocks under the chassis, at least 20 mm of clearance under each tyre,
until you have finished bring-up.

Then, from the Mac:

```sh
make run-app          # the car advertises itself over mDNS; pick it from the list
```

The desktop app and the car must share the same secret; enter the same passphrase in the app's
connection dialog. It is hashed into key material and never travels over the wire — the UDP
tagging key is derived per session from a token issued during the TCP handshake, so the secret
itself is never on the network and never in a screenshot of a running app.

Before the first real drive, run the calibration procedure in
[`docs/calibration.md`](docs/calibration.md). It measures servo centre and limits, per-direction
maximum RPM, the duty deadband, and the feedforward table. Every one of those is a measurement,
not a preference — treating a guess as a measurement is how you spend an afternoon tuning a PID
against the wrong plant gain.

To run the firmware as a service, install the units in `pi/systemd/`. Do not skip this in favour
of a `screen` session: the unit's `ExecStopPost=` is the only layer of the panic-stop chain that
survives `SIGKILL`.

---

## Safety

This vehicle has two 12 V gearmotors, a lithium powerbank and a NiMH pack on it. It is heavy
enough to hurt and fast enough to be hard to catch. These are the things that will actually bite
you.

**1. `pigpiod` retains GPIO state after its client dies.** A segfault or a `kill -9` at 80 % duty
leaves the motors running **indefinitely** — the daemon has no idea anything is wrong. Never
`kill -9` the control process. Use `systemctl stop telekart-control`, or Ctrl-C. Four independent
layers exist because no single one covers every case:

1. `hardware.panic_stop()` — idempotent, allocation-free; wired to `atexit`, `SIGTERM`/`SIGINT`/
   `SIGHUP`, `sys.excepthook`, `threading.excepthook`, and a context manager around `main()`.
2. `ExecStopPost=` in the systemd unit, running `pigs` directly. **The only layer that covers
   `SIGKILL`**, and systemd runs it on every stop path including crash-restart.
3. External 10 kΩ pull-downs on ENA/ENB/IN1–IN4 — covers reboot, power loss, loose ribbon cable.
4. The battery-side switch. The human's E-stop, and the only one that still works when everything
   else is on fire. **Know where it is before you power up.**

**2. The pull-down resistors are not optional.** GPIO0–8 default to pull-**up** on BCM283x, so
IN1 (GPIO5) and IN2 (GPIO6) idle HIGH from the moment the Pi powers on until software takes over.
Software cannot fix this; it is not running yet. Without external pull-downs the safe state is
whatever the SoC felt like doing.

**3. On the L298, `IN1 == IN2` with EN high is a BRAKE, not a coast.** Both HIGH or both LOW —
either one shorts the motor through the bridge. Coast requires **EN low**. This is the opposite
of most people's intuition, and getting it backwards shorts a spinning motor every single time
you meant to let it roll.

**4. The steering servo shares the Pi's 5 V rail.** A stalled HS-311 pulls enough to brown the
SoC out. The firmware stops the pulse train when disarmed (the servo goes limp, holding current
drops to near zero) and slew-limits every command, which caps peak draw. Do not remove either
behaviour, and do not hold the wheel against full lock.

**5. Both hardware PWM channels share one clock divider.** Writing GPIO12 at 1 kHz and then
GPIO13 at 4 kHz corrupts channel 0. The HAL exposes only `set_pwm_pair()` — writing them
separately is deliberately unrepresentable.

**6. The app never asserts that it is armed.** Sending an ARM is a request, not a state change;
the HUD shows only what the car reports back in telemetry. So a HUD reading ARMED means the car
said so — but treat a *stale* HUD as unknown, not as safe. Watch the link-age indicator.

**7. Two stalled motors put roughly 7 W into a bridge whose stock heatsink handles 2–3 W.** Stall
detection is hardware protection, not a nicety. If you disable it, watch the heatsink.

---

## Documentation

| Document | What it covers |
|---|---|
| [`docs/INTERFACES.md`](docs/INTERFACES.md) | **Normative.** Every internal Python signature, unit convention and threading rule. If code and this document disagree, the code is the bug. |
| [`docs/wiring.md`](docs/wiring.md) | The build sheet: every connection, in order, with a verification step after each subsystem. |
| [`docs/power.md`](docs/power.md) | Battery, regulator, current budget, and why the duty *sum* is what gets limited. |
| [`docs/pi-setup.md`](docs/pi-setup.md) | OS image, `config.txt`, pigpiod, the I2S/audio overlay conflict. |
| [`docs/bringup.md`](docs/bringup.md) | Bench test sequence with pass/fail gates, wheels off the ground until the last phase. |
| [`docs/calibration.md`](docs/calibration.md) | The numbers you measure: servo limits, max RPM per direction, deadband, feedforward table. |
| [`docs/tuning.md`](docs/tuning.md) | PID and input-chain tuning, and the bandwidth ceiling the encoder resolution imposes. |

For anything about the wire format, read `packages/telekart_protocol/` directly — the source is
the contract, and the tests are its proof.

---

## Development

Three environments, kept separate on purpose:

| Environment | Python | Why pinned |
|---|---|---|
| `app/.venv` | 3.12 (`>=3.12,<3.14`) | PySide6-Essentials 6.11 and pygame-ce wheels. The ceiling is a wheel-availability fact, not a guess. |
| `pi/.venv` | 3.11 | What Bookworm ships. Created with `--system-site-packages` so the apt-installed picamera2/libcamera bindings remain visible. |
| `tools/telekart_sim/.venv` | 3.11+ | Runnable from either side, so the Pi's network stack can be integration-tested with no desktop app in the loop. |

```
make help          list every target with its description

make setup-app     desktop station            make run-app     launch the GUI
make setup-pi      firmware (run on the Pi)   make run-sim     serve a fake car
make setup-sim     simulator                  make run-cli     headless client

make test          every suite that has an environment; the rest are skipped, loudly
make test-protocol golden-byte wire tests     make test-pi     firmware, no hardware
make test-app      pytest-qt, offscreen       make lint        ruff, or a compile check

make clean         caches and build output    make clean-venvs remove the three venvs
```

`ARGS=` is forwarded to the `run-*` targets, `PYTEST_ARGS=` to the `test-*` ones:

```sh
make run-sim ARGS="--seed 7 --latency 120"
make test-protocol PYTEST_ARGS="-k golden -v"
```

### Testing strategy

| Layer | Approach |
|---|---|
| `telekart_protocol` | Golden byte strings asserted in both directions, plus `struct.calcsize` pins, tamper rejection, version-mismatch rejection, saturation/NaN handling, fragmented and coalesced framing. **Run by both sides' suites** so a one-sided change fails. |
| Firmware logic | `MockBackend` + `FakeClock`. A simulated 60-second drive runs in milliseconds and gives the same answer every time. |
| Input chain | Property tests: output always in range, monotonic in input, exact deadzone behaviour. |
| Integration | `telekart-sim --realtime 0 --seed 1`, driving the network stack with no Qt widgets. |
| UI | `pytest-qt` smoke tests: instantiate every screen, drive `AppModel` from a fixture, assert no `paintEvent` exceeds 8 ms. |
| Soak | 60 minutes under `tracemalloc` asserting stable RSS. Guards `FrameBundle` lifetime — wrapping a PyAV plane in a `QImage` without holding the frame alive is a use-after-free that crashes intermittently. |

No module in the firmware calls `time.monotonic()` directly. A `Clock` is constructor-injected
everywhere, which is what makes the whole suite deterministic and instantaneous.

### Things that will surprise you

Each of these is settled, and each cost real debugging time to learn. The reasoning lives in
[`docs/INTERFACES.md`](docs/INTERFACES.md) §12.

- **PyAV must use `thread_type='NONE'`.** `'FRAME'` — which `'AUTO'` selects — buffers 3–5 frames
  before emitting anything: 100–170 ms. It is the single most common cause of "H.264 teleop feels
  laggy" and has nothing to do with the Pi.
- **Encoder decoding is x2**, EITHER_EDGE on channel A only, direction inferred from the commanded
  H-bridge state. True quadrature would cost a pigpiod socket round-trip (~50–100 µs) per edge.
- **M/T velocity estimation, not counts-per-tick.** At 100 Hz and 660 cpr, 20 RPM is 2.2
  counts/tick — ±45 % quantization. The resulting output filter caps closed-loop bandwidth at
  roughly 7 Hz; do not try to tune past it.
- **`pygame-ce`, never `pygame`.** They claim the same module name; installing both breaks
  imports.
- **Headless OpenCV only.** The GUI wheels link their own Qt and Cocoa handling, and two Qt stacks
  in one process crash.
- **Never filter in both places.** An EMA in the app plus an EMA in the firmware is second-order
  lag nobody can reason about. Firmware smoothing is a hard slew clamp, never a filter.
