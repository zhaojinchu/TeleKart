# TeleKart2 — v1

Drive an RC car from a Mac with WASD, watching a live camera feed from the Pi on board.

That is the entire scope of v1. Encoders are wired but ignored — no odometry, no closed loop. A
steering wheel is the eventual goal and explicitly not part of this version.

> **The Raspberry Pi is off-limits right now.** `telekart.local` is running other software that
> controls the kart. Do not SSH in to look around, do not run anything there, do not integrate with
> it. All development happens on the Mac against the simulator. See [CLAUDE.md](CLAUDE.md).

---

## Quick start — no hardware needed

```bash
make venv
make sim      # terminal 1: the simulated Pi
make drive    # terminal 2: the driving app
```

`make drive` targets the **simulator on this Mac**, not the car. Reaching the real thing takes
`make drive-car` or `TELEKART_HOST=telekart.local` — deliberately, since the Pi is off-limits.

**Keys:** `W`/`S` throttle · `A`/`D` steer · `Space` brake · `Enter` arm/disarm · `Esc` quit.

Nothing responds to throttle until you arm it. That is deliberate — a page-load or a reconnect
shouldn't be able to make the car lurch.

## How it fits together

```
MAC (this repo)                                  PI  (~/telekart2/ — at bring-up)
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
direction guards and limits live in one place — the side that owns the hardware. The Mac never
computes a PWM value.

Two consequences worth the design. Safety logic can't be bypassed by a buggy or crashed client. And
v2 swaps key-booleans for wheel-axis floats without the Pi changing at all.

## The simulator is the point

`sim/fake_pi.py` runs on the Mac and speaks the identical protocol with a synthetic camera. Because
it shares `common/vehicle.py` with the real Pi rather than approximating it, the failsafes you test
against it are the same code that runs on the car.

That's what makes it possible to build and verify the whole of v1 without touching hardware — which
is the constraint this project is actually working under.

## Debugging

Both ends write `logs/session-<ts>.jsonl` — one JSON object per line, every message, both
directions, tagged `"dir": "tx"` or `"rx"`. Two independent captures of the same conversation is what
turns "it worked on the Mac but the car didn't move" into a five-second diagnosis.

```bash
jq 'select(.dir=="rx") | .throttle_out' logs/session-*.jsonl   # did the slew limiter engage?
jq 'select(.state=="FAILSAFE")' logs/session-*.jsonl           # when did the link drop?
```

The live debug panel shows state, RTT, `throttle_cmd → throttle_out`, servo µs, duty %, link rates
and drops. If throttle looks stuck, `throttle_cmd` vs `throttle_out` tells you immediately whether
it's the slew limiter or something real.

## Layout

| Path | What |
|---|---|
| `common/protocol.py` | Wire format. `pi/protocol.py` is a copy; `make check-protocol` diffs them |
| `common/vehicle.py` | Vehicle behavior — pure logic, no hardware. Shared with the sim |
| `pi/` | Runs on the Pi: `car.py` (pigpio), `video.py` (picamera2), `main.py` |
| `mac/` | The driving app |
| `sim/fake_pi.py` | Simulated Pi |
| `docs/PROTOCOL.md` | Exact wire format |
| `docs/PI_SETUP.md` | Bring-up runbook — written, **not executed** |

## Status

v1 is developed and verified against the simulator. It has **never been run on the car** — bring-up
is a separate, gated session; the runbook is in [docs/PI_SETUP.md](docs/PI_SETUP.md).
