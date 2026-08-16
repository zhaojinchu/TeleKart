# TeleKart v2 — Tuning

What to turn, in what order, and — more usefully — what the ceilings are so you stop turning
before you start chasing physics.

**Companion documents:** [calibration.md](calibration.md) must be complete first ·
[bringup.md](bringup.md) Phase 7 is the acceptance test for §2 · [power.md](power.md) for why
the frequency arithmetic in §4 matters

---

## 1. Before you tune anything

Tuning against an uncalibrated car is guessing with extra steps. All of these must already be
true:

- [ ] [bringup.md](bringup.md) Phases 1–6 passed, including the Phase 5 CPU gate.
      **A loop that is dropping deadlines cannot be tuned; you will be tuning the deadlines.**
- [ ] `calibration.yaml` exists with `on_ground: false`, four `max_rpm` values within 20 %, and
      monotonic `ff_lut` tables.
- [ ] `encoder_cpr` measured by hand, not assumed ([calibration.md §3](calibration.md)).
- [ ] Geometry measured ([calibration.md §6](calibration.md)).
- [ ] Fully charged pack. Plant gain moves with pack voltage; tuning on a half-flat pack
      produces gains that are wrong on a full one.
- [ ] Wheels off the ground.

### One decision to make consciously: where does *feel* live?

[INTERFACES.md §9](INTERFACES.md) is explicit about the division of responsibility:

> The app owns *feel* (curves, expo, deadzone, smoothing). The firmware owns *protection*
> (duty slew clamp, servo rate limit, stall detection, failsafe ramp). **Never filter in both
> places.**

The firmware nonetheless carries `throttle_expo`, `throttle_deadband`, `steer_expo` and
`steer_deadzone` in its parameter registry, because the simulator and a directly-connected
controller need them.

**Applying a curve in both places composes two curves and produces something neither of you
designed.** Decide once, write it in your build log:

| Setup | Recommended |
|---|---|
| Driven by the desktop app (normal) | app applies expo/deadzone; set the firmware's `throttle_expo` and `steer_expo` to **1.0** and the deadzones to **0.0** |
| Simulator, or a controller wired straight to the car | firmware applies them; app's curve set to linear |

Note that this is about *static curves*. It is doubly true of filters: an EMA in the app plus
an EMA in the firmware is second-order lag nobody can reason about. Firmware smoothing is a
**hard slew clamp only** — transparent when unsaturated, never a filter.

And the app's rate limit must be **tighter** than the firmware's, so the firmware's limiter
never engages during normal driving. When it does, the firmware sets
`TelemetryFlags.LIMITER_ACTIVE` (bit 0), and [bringup.md](bringup.md) Phase 7 gates on that
bit staying dark.

---

## 2. PID

### 2.1 The ~7 Hz ceiling, and where it comes from

**You cannot tune this loop past about 7 Hz of closed-loop bandwidth. Stop trying at 7.**

Here is the whole stack, in frequency:

| Element | Time constant | Corner frequency |
|---|---|---|
| Motor + gearbox, first-order lag | τ ≈ 0.15 s | **1.06 Hz** |
| **M/T velocity estimator output filter** | **τ = 25 ms** | **6.37 Hz** ← the binding constraint |
| Control loop rate | 10 ms | 100 Hz |
| Nyquist | — | 50 Hz |

`f_c = 1 / (2πτ) = 1 / (2π × 0.025) = 6.37 Hz`.

#### Why the 25 ms filter is there and cannot simply be removed

Naive counts-per-tick velocity is unusable at low speed. At 100 Hz and 660 cpr:

```
20 RPM  →  660 × 20/60 / 100  =  2.2 counts per tick
±1 count of quantization       =  ±45 % velocity error
```

You cannot close a loop on a signal with 45 % noise. The M/T estimator fixes the *resolution*
problem by closing its measurement window **on an edge rather than on the loop tick** — which
is why the backend has to expose microsecond ticks at all
([INTERFACES.md §3](INTERFACES.md)) — and the residual noise is then knocked down by a 25 ms
output filter. That filter is the price of a usable low-speed velocity signal, and 7 Hz is
what it costs.

#### Why exceeding it does not work

A first-order filter contributes 45° of phase lag **at its own corner**, approaching 90° above
it. Push the closed-loop bandwidth above 6.4 Hz and you are asking the controller to act on
information that is already 45–90° out of date at the frequency you are trying to control.
That is phase margin you do not have.

The practical tell: raise `pid_kp` until it oscillates, and note the oscillation frequency.

| Oscillation at | Meaning |
|---|---|
| **1–3 Hz** | you are exciting the plant. Normal. Back off to 60 % of that gain. |
| **> 7 Hz** | you are oscillating on **filter phase**, not plant dynamics. No value of `pid_kd` fixes this. Lower `pid_kp`. |

**Target closed-loop bandwidth: 3–7 Hz.** At 7 Hz you still have 14 loop samples per cycle,
which is comfortable. There is nothing above it worth having.

### 2.2 Step 1 — feedforward first, and it does the work

> **Feedforward carries the load; the PID only trims.** That is what makes the loop tunable at
> all when the plant gain is ~2× uncertain. ([INTERFACES.md §4](INTERFACES.md))

Set `closed_loop = false`. Command RPM targets at 20 %, 40 %, 60 %, 80 % and 100 % of
`max_rpm_measured` and record the actual RPM.

> **Gate: open-loop feedforward alone tracks within ±15 % across 20–100 % of
> `max_rpm_measured`.**

If it does not, **stop.** A bad `ff_lut` cannot be rescued by any PID. Go back to
[calibration.md §4](calibration.md) and find out why — the usual answers are a flat pack, a
non-monotonic table, or a calibration run that hit the combined-duty budget.

Put `closed_loop` back to `true` before continuing.

### 2.3 Step 2 — `pid_kp`

Set `pid_ki = 0`, `pid_kd = 0`.

Command a step from 0 to 50 % of `max_rpm_measured`. Raise `pid_kp` until the response **just
begins to overshoot**, then set it to **60 % of that value**.

#### Sanity-check the decade before you start

`pid_kp` is in **duty per RPM**. The default is 0.005, which means a 200 RPM error commands
1.0 duty — full authority.

```
pid_kp × max_rpm_measured  should land in  0.5 … 1.5
```

At 200 RPM: `0.005 × 200 = 1.0`. ✓

If your product is 0.05 or 10, you are in the wrong decade and no amount of careful bisection
will help. Range in the registry is 0.0–0.100.

### 2.4 Step 3 — `pid_ki`

Raise until steady-state error disappears within about a second.

#### The arithmetic that tells you when to stop

```
time to saturate the integrator  =  pid_i_clamp / (pid_ki × typical_error)
```

At the defaults, with a persistent 20 RPM error:

```
0.40 / (0.020 × 20)  =  1.0 s
```

The integrator reaches its clamp in exactly one second. That is the design point.

| Time to saturate | Verdict |
|---|---|
| **< 0.5 s** | `pid_ki` too high — the integrator dominates transients and you get overshoot |
| **0.5 – 2 s** | right |
| **> 2 s** | `pid_ki` too low — steady-state droop never gets corrected |

#### Verify the anti-windup

The controller uses **conditional integration**: it freezes the integrator whenever the output
is saturated, and clamps so that `feedforward + integral` stays inside the output limits.
`reset()` is called on arm, on direction change, and on disarm.

**Test it.** Command 50 % of `max_rpm_measured`, block the wheel with a piece of wood for two
seconds, then release.

> **A burst of overshoot on release means the integrator wound up while saturated and the
> anti-windup is not working.** That is a bug, not a tuning problem. Do not compensate for it
> by lowering `pid_ki`.

### 2.5 Step 4 — `pid_kd`

**Leave it at 0.** The registry says so in as many words: *"Leave at zero unless you also
filter the derivative — encoder velocity is noisy."*

The `PID` constructor's `derivative_lpf_hz` defaults to 10 Hz, which is already **above** the
6.4 Hz sensor corner. So `pid_kd` is differentiating a signal whose useful content stops below
the filter — it is amplifying noise by construction.

If you insist: start at 0.002, increase in 0.002 steps, and stop the instant you hear
high-frequency chatter from the motors. The registry caps it at 0.050. In practice this car
does not need it; the plant is dominated by a single 0.15 s lag and a PI controller handles a
first-order plant perfectly well.

### 2.6 Symptom table

| Symptom | Cause | Fix |
|---|---|---|
| Oscillation at 1–3 Hz | `pid_kp` too high | drop to 60 % |
| Oscillation above 7 Hz | filter phase, not the plant | drop `pid_kp`; `pid_kd` will not help |
| Slow creep up to setpoint | `pid_ki` too low | raise; check §2.4's arithmetic |
| Overshoot on every step | `pid_ki` too high, or integrator saturating in < 0.5 s | lower `pid_ki` |
| Overshoot **only after a stall release** | anti-windup broken | it is a bug — report it |
| Oscillation only at low RPM | encoder quantization | accept it, or lower `pid_kp`. It is real and it is the price of x2 decoding |
| Left and right fighting on a straight | `straight_sync_gain` too high | default 30 RPM/m; it is active **only near centre** so it does not fight the electronic differential |
| Fine on the bench, hunts on the ground | plant gain changed with load | re-run calibration on the ground, then re-tune |
| `LIMITER_ACTIVE` lights during ordinary driving | app rate limits looser than the firmware's | tighten the **app** side |

### 2.7 Re-tune after any of these

All four change the plant gain, and three of them are things you will do without thinking:

- Changing `pwm_hz` (§4)
- Changing the battery, or the pack ageing
- Changing tyres or tyre pressure
- Moving from the bench to the ground

The last one is the big one. Expect `max_rpm_measured` to fall 20–40 % on the ground
([calibration.md §4.5](calibration.md)), and the gains that were right in the air to be
noticeably soft on the floor.

### 2.8 The acceleration and deceleration limits

| Parameter | Default | Why it is what it is |
|---|---|---|
| `accel_rpm_per_s` | 250 RPM/s | Inrush. A step to 150 RPM asks for locked-rotor current until the rotor moves — at `MOTOR_WINDING_OHMS` (3 Ω) and ~7 V at the terminals, roughly **2.3 A per motor**, three times the 1.5 A combined budget. Ramping the RPM **target** at 250 RPM/s makes 150 RPM take 600 ms and keeps current in the running band. See [power.md §4](power.md). |
| `decel_rpm_per_s` | 700 RPM/s | Nearly 3× the acceleration limit, deliberately: braking shorts the winding rather than pulling from the supply, so it is not current-limited the same way — and you always want to be able to stop faster than you can go. |

**These limit the RPM target, not the duty.** Rate-limiting the duty instead would fight the
PID and make the loop untunable.

---

## 3. Deadband compensation

### Why it exists

Two things stack up at the bottom of the throttle:

1. The L298's Darlington output stage needs roughly **2 V** across it before it conducts
   meaningfully.
2. The motor and gearbox have static friction to overcome.

The result is a **stall deadband below about 12 % duty** — real L298N behaviour, modelled
explicitly in the simulator for exactly this reason ([INTERFACES.md §10](INTERFACES.md)).

Without compensation, the first 12 % of throttle travel does nothing, and every time you leave
a stop the integrator has to wind up through that dead region before anything happens. You
feel it as a lag, then a lurch.

### The map

```
u_out = 0                                     if |u| ≤ ε
u_out = sign(u) × (db + (1 − db) × |u|)       otherwise
```

- `db` is the **measured** deadband for **that wheel in that direction** — four different
  values, from `calibration.yaml`. Forward and reverse deadbands on the same motor commonly
  differ by 0.03–0.05 ([calibration.md §4.2](calibration.md)).
- `ε` should be small, around **0.005**. Too large and you get a dead notch at zero; too small
  and PWM dither near zero makes a stationary wheel twitch.
- The `(1 − db)` scaling is what keeps full duty reachable — without it the top of the range
  saturates early.

### The landmine: do not count the deadband twice

The pipeline is, in order ([INTERFACES.md §4](INTERFACES.md)):

```
shaping → mixer → per-wheel (feedforward + PID) → DEADBAND COMPENSATION
        → combined-duty budget → clamp → dead-time sequencing → H-bridge
```

Deadband compensation comes **after** feedforward. Therefore **the `ff_lut` must be a curve
relative to the deadband, not one that already includes it.**

If both include it, commanding 1 % throttle sends `0.12 + 0.12 + 0.01 ≈ 0.25` duty to the
bridge and the car lurches off the line.

### Test it in thirty seconds

`closed_loop = false`, wheels off the ground, command **1 % throttle** from rest.

| Observation | Verdict |
|---|---|
| The wheel *just* creeps | correct |
| The wheel lurches to a visible speed | **the deadband is being counted twice** |
| The wheel does nothing at all | **the compensation is not being applied** |

Test forward and reverse separately — that is the whole point of having four values.

---

## 4. PWM frequency for a Darlington bridge

### The arithmetic

The L298's output stage is a Darlington pair. Its switching times are dominated by the
Darlington storage time on turn-off: roughly `t_on ≈ 1.5–2 µs` and `t_off ≈ 1.5–4 µs`. Call the
total transition **~4 µs**.

Switching loss scales with how large a fraction of each period is spent in transition:

| `pwm_hz` | Period | Transition fraction | Relative switching loss |
|---|---|---|---|
| 500 Hz | 2000 µs | 0.2 % | 0.5× |
| **1000 Hz** | **1000 µs** | **0.4 %** | **1× (default)** |
| 2000 Hz | 500 µs | 0.8 % | 2× |
| 4000 Hz | 250 µs | 1.6 % | 4× |
| 8000 Hz | 125 µs | 3.2 % | **8×** |

That 0.4 % figure is exactly what the registry's own description of `pwm_hz` cites.

**Why the multiplier matters here specifically:** the bridge is already dissipating about 3 W
of *conduction* loss (`2 V × 1.5 A`) into a heatsink rated for 2–3 W
([power.md §3.3](power.md)). You are at the thermal limit before you add a single watt of
switching loss. An 8× increase is not something this package absorbs.

### The lower bound

The motor's electrical time constant sets the floor:

```
τ_e = L / R      with R ≈ 3–5 Ω and L ≈ 1–3 mH   →   τ_e ≈ 0.2–1.0 ms
                                                      corner ≈ 160–800 Hz
```

PWM must sit comfortably **above** that, or winding current becomes discontinuous within each
period: torque ripple grows, low-speed control gets lumpy, and it sounds terrible. 1 kHz is
1.2–6× above the corner.

Below about 500 Hz you will both hear it and feel it.

### The recommendation

**Default 1000 Hz. Usable range 1000–2000 Hz.**

1 kHz sits squarely in a band the ear is sensitive to, so it whines. If the whine bothers you
more than the heat does, **2 kHz** is the move: it costs 0.4 % → 0.8 % switching loss, which
this heatsink can still absorb.

**Do not go to 8 kHz "so it will be ultrasonic".** It will not be — 8 kHz is very audible — and
it costs you eight times the switching loss for the privilege.

### The hard constraint

> **Both hardware PWM channels on BCM283x share a single clock divider.**

`pwm_hz` therefore applies to **ENA and ENB simultaneously**. Writing GPIO12 at 1 kHz and then
GPIO13 at 4 kHz corrupts channel 0. The firmware makes this unrepresentable by exposing only
`set_pwm_pair()` ([INTERFACES.md §2](INTERFACES.md)), and `pwm_hz` is marked
`requires_disarm=True` for the same reason.

When you drive the pins by hand with `pigs`, nothing protects you. Always give both channels
the same frequency.

### Duty resolution is not a limitation

pigpio's hardware PWM takes duty as an integer 0–1 000 000. At 1 kHz there are thousands of
achievable steps. You will never be limited by duty quantization on this car; do not spend
time on it.

### After changing `pwm_hz`

**Re-tune the PID (§2).** Current ripple changes with frequency, and current ripple changes the
effective plant gain. Also re-run the drive calibration — the deadband moves with frequency
too.

---

## 5. Camera and video latency

### The chain, with honest numbers

| Stage | Typical | The lever |
|---|---|---|
| Sensor exposure + readout | 10–33 ms | pick the smallest sensor mode ≥ your output size; shorter exposure in good light |
| ISP / format conversion | a few ms | make the output size the size you actually send |
| Hardware H.264 encode | ~1 frame (33 ms @ 30 fps) | raise fps before raising resolution |
| Sender queue | **0 if bounded, unbounded if not** | must be bounded and **drop-oldest** |
| WiFi + TCP | 5–40 ms, spiky | clean 2.4 GHz channel; power save **off** |
| **Decode** | **1 frame if configured right, 3–5 if not** | **§5.3 — read it** |
| Reformat + blit | a few ms | reformat on the **decode** thread |

**Target: under 150 ms glass-to-glass**, which is the [bringup.md](bringup.md) Phase 9 gate.

### 5.1 Measuring it

Point the camera at a phone running a millisecond stopwatch. Photograph the **phone and the
monitor in the same frame**. Subtract. Five times, take the median.

Anything else measures something other than what you care about. In particular, `pts_us` in the
video frame header is `CLOCK_MONOTONIC` **on the car** — it is excellent for measuring
frame-to-frame jitter and detecting stalls, and it tells you nothing about absolute latency
unless the two clocks are disciplined.

### 5.2 Capture-side settings

**Sensor mode.** Run `rpicam-hello --list-cameras` and read your image's actual mode list
rather than trusting any table. Two rules:

- Prefer a **binned** mode over a **cropped** one, so you keep field of view.
- Prefer the **smallest** mode that is at least your output size, so the ISP is not doing a
  large downscale on every frame.

**`video_iperiod` = 15** (a keyframe every 0.5 s at 30 fps).

> The hardware encoder **cannot honour on-demand keyframe requests**, so a short GOP is the
> only recovery mechanism after packet loss. (`params.py`)

The cost is real: a keyframe is 5–10× the size of a P-frame, so a short GOP raises both the
average bitrate floor and the burst size. 15 is the compromise. 30 halves the keyframe
overhead and doubles the worst-case time-to-recover to one second.

**`video_bitrate` = 2 000 000** at 640×480@30. On a congested 2.4 GHz band, 2 Mbps is already
ambitious.

> **If you see stalls, drop the bitrate before you drop the resolution.** Motion blur beats a
> frozen frame in a teleop loop. A driver can steer through a soft picture; they cannot steer
> through a picture that is three seconds old.

**Framing.** One TCP connection carries the stream, so a single retransmit stalls it. That is
deliberate — a frame you cannot decode is worthless, so there is no point in a lossy transport
here — but it is exactly why the sender's queue must be **bounded and drop-oldest**, never
allowed to grow. `VideoFrameFlags.DROPPED_BEFORE` (bit 1) tells the decoder that frames were
discarded and to expect corruption until the next keyframe, rather than silently rendering
garbage.

Raw Annex-B access units go **straight into a `CodecContext`** with no container and no
demuxer. That removes another one to two frames of buffering, and it is part of why the 24-byte
frame header exists at all ([protocol.md](protocol.md)).

### 5.3 Decoder settings — the ones that are not negotiable

```python
codec.flags |= av.codec.context.Flags.LOW_DELAY
codec.thread_type = 'NONE'     # NEVER 'FRAME' or 'AUTO'
codec.thread_count = 1
```

> # ⚠ **NEVER SET `thread_type` TO `'FRAME'` OR `'AUTO'`.**
>
> ## **`'AUTO'` selects `'FRAME'` for H.264. Frame threading buffers `thread_count` frames before it emits anything — 3 to 5 frames, which at 30 fps is 100–170 ms of pure, unrecoverable latency.**
>
> ## **It is the single most common cause of "H.264 teleop feels laggy", and it has nothing whatsoever to do with the Pi.**
>
> **You can spend a week optimising the capture pipeline, the encoder, the bitrate, the WiFi
> channel and the network stack, and win back less than this one line gives you for free.**

#### How to prove which one you have, unambiguously

Feed the decoder a stream at **1 frame per second**.

| Behaviour | Verdict |
|---|---|
| The first frame appears immediately | `thread_type` is `'NONE'`. Correct. |
| Nothing appears until the 3rd–5th frame arrives, three to five seconds later | **Frame threading is on.** Fix the line. |

There is no ambiguity in that test and it takes a minute.

### 5.4 Where the conversion happens

`frame.reformat()`, sized to the widget, runs **on the decode thread**, so the GUI thread does
a 1:1 blit and nothing else. Doing the reformat on the GUI thread turns a 60 Hz repaint budget
into a 30 Hz one and shows up as UI stutter that looks like a Qt problem.

### 5.5 Frame lifetime is a real crash risk

`FrameBundle` owns **both** the `av.VideoFrame` and the `QImage` that views its buffer.

> Nothing else in the codebase may construct a `QImage` from a raw buffer. Wrapping a PyAV
> plane without holding the frame alive is a use-after-free that crashes intermittently and is
> miserable to diagnose. ([INTERFACES.md §9](INTERFACES.md))

Which is why the 60-minute `tracemalloc` soak in [bringup.md](bringup.md) Phase 10.5 is not
optional.

---

## 6. Whole-system symptom index

The fastest route from a symptom to the right document.

| Symptom | Actual cause | Where |
|---|---|---|
| Both motors dead, **no error message anywhere** | `dtparam=audio=on` — `snd_bcm2835` owns both PWM channels | [pi-setup.md §2](pi-setup.md) |
| Servo dead **and** right motor runs one direction only; left motor perfect | An I2S overlay owns GPIO18/20/21 | [wiring.md §7](wiring.md) |
| Encoder counts appear **only under motor load** | Encoder ground routed to the L298N instead of the Pi | [wiring.md §5](wiring.md) |
| Encoder counts at rest | Brush noise; missing decoupling | [wiring.md §6](wiring.md) |
| One encoder channel never toggles | 5 V-only encoder on a 3.3 V rail | [wiring.md §9](wiring.md) |
| **Both** encoders hit zero simultaneously while commanded | `Fault.BROWNOUT` — the regulator tripping, **not** a stall | [power.md §4](power.md) |
| One encoder hits zero while commanded | `Fault.STALL_L`/`_R` — a real mechanical stall | [bringup.md](bringup.md) 8.5 |
| Motors stutter at a steady 2–5 Hz under load | Regulator hiccup-mode restart cycling | [power.md §4](power.md) |
| Only happens under acceleration | Inrush — lower `accel_rpm_per_s` | §2.8 |
| Only happens in a hard turn | Combined duty — lower `duty_sum_max` | [power.md §4](power.md) |
| Pi reboots when the steering moves | Servo stall on the shared 5 V rail | [power.md §6](power.md) |
| Pi reboots when the master switch is thrown | The L298N `+5V` terminal is back-feeding the Pi | [wiring.md §8](wiring.md) |
| `loop_p99_us` above 12000 | Encoder callback CPU, or the governor | [bringup.md](bringup.md) Phase 5 |
| 1–3 Hz oscillation | `pid_kp` too high | §2.6 |
| Above 7 Hz oscillation | Filter phase — `pid_kd` will not save you | §2.1 |
| Lurch off the line at 1 % throttle | Deadband counted twice | §3 |
| Dead first 12 % of throttle travel | Deadband compensation not applied | §3 |
| Car pulls, left/right RPM **equal** | Steering trim | [calibration.md §5](calibration.md) |
| Car pulls, left/right RPM **differ** | Drivetrain imbalance, not steering | [calibration.md §5.1](calibration.md) |
| Odometry under-reports how much you turned | `steer_max_deg` over-stated (horn measured, not wheel) | [calibration.md §2.5](calibration.md) |
| `max_rpm` above 300 RPM | `encoder_cpr` wrong, probably by an integer factor | [calibration.md §3](calibration.md) |
| Video smooth but **laggy** | `thread_type` | **§5.3** |
| Video stuttery **and corrupt** | Packet loss — lower the bitrate before the resolution | §5.2 |
| UI stutters while video plays | Reformat running on the GUI thread | §5.4 |
| Intermittent crash in the video path | `FrameBundle` lifetime — use-after-free | §5.5 |
| `LIMITER_ACTIVE` during normal driving | App rate limits looser than the firmware's | §1 |
| **Motors keep running after the process exits** | `pigpiod` retains GPIO state — layer 2 of the panic chain is not installed | [pi-setup.md §7.1](pi-setup.md), [bringup.md](bringup.md) 8.4 |
