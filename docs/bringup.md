# TeleKart v2 — Bench Bring-Up

Ten phases, in order, each with a **hard numeric gate**. A phase that does not meet its gate
does not get waived, worked around, or revisited later. You go back and fix it.

**The wheels stay off the ground until Phase 10.** Every earlier phase is designed so that a
total failure of everything you are testing spins a wheel in the air and nothing else.

**Companion documents:** [wiring.md](wiring.md) · [power.md](power.md) ·
[pi-setup.md](pi-setup.md) · [calibration.md](calibration.md) · [tuning.md](tuning.md) ·
[protocol.md](protocol.md)

---

## Instruments

You cannot pass these gates by looking at the car. You need:

| Tool | Used for |
|------|----------|
| **Digital multimeter** | Regulator setting, pull-down verification, loaded rail voltages |
| **Inline RC watt meter** (XT60 power analyser) | The 1.5 A budget. Strongly recommended — everything about §4 of [power.md](power.md) is guesswork without it |
| **IR thermometer** | L298N heatsink. "It feels warm" is not a measurement |
| **Tape measure** (5 m) | Geometry, straight-line tracking, square closure |
| **Protractor or a phone level app** | Steering lock angle |
| **Phone with a millisecond stopwatch** | Glass-to-glass video latency |
| **Masking tape** | Marking wheels and floor for revolution counting |
| **Safety glasses** | Phase 4 onwards. A wheel that liberates its grub screw at 200 RPM leaves the hub at 200 RPM |

---

## Abort criteria — stop immediately, at any phase

- Anything smells hot, or you see smoke.
- A capacitor bulges or a component discolours.
- The L298N heatsink exceeds **80 °C**.
- A motor makes a grinding noise it did not make before.
- A wheel moves when the pull-downs say it must not.
- **Motors keep running after the software that commanded them has exited.**

Hit the master switch. Do not diagnose a running car.

---

## Phase 0 — Bench safety

Not a phase, a prerequisite. Nothing below happens until every line here is true.

- [ ] Car on blocks. **≥20 mm of air under every tyre.** Test it: spin each wheel by hand and
      confirm nothing touches.
- [ ] The car cannot walk off the blocks. Chock it, tape it, or weigh it down.
- [ ] 5 A fuse fitted in the pack positive, at the pack terminal ([power.md §7](power.md)).
- [ ] Master switch fitted, **≥10 A DC rated**, and reachable **without moving your feet**.
- [ ] Nothing loose within 300 mm of a wheel: no cable tails, no tools, no coffee.
- [ ] Hard, non-flammable surface. Not a carpet, not a bed, not a pile of paper.
- [ ] DMM present and working. Test it on a known battery first.
- [ ] Safety glasses within reach.
- [ ] You are alone with the car, or the other person knows where the switch is.

> **GATE 0.** You can reach the master switch, from where you sit, without standing up or
> reaching across a wheel. If you cannot, move the switch. This is the gate you will be
> tempted to skip and the one that matters most.

---

## Phase 1 — OS and daemon

No power to the motor domain at all. Master switch off, and ideally the pack physically
unplugged.

Run the pre-flight block from [pi-setup.md §10](pi-setup.md). Then confirm the pin mux and
pull-downs specifically:

```bash
lsmod | grep snd_bcm2835                 # NOTHING
pinctrl get 5,6,12,13,18,20,21           # no PCM_* / PWM* functions
pigs t; sleep 1; pigs t                  # second value larger than the first
vcgencmd get_throttled                   # 0x0
pgrep -a pigpiod                         # -t 1 -s 5 -l -b 200 -x 0x0BF53060
```

Then, with the ribbon connected and pigpiod running:

```bash
for g in 5 6 12 13 20 21; do printf "GPIO%-3s " $g; pigs r $g; done
```

> **GATE 1.** All of:
> - `uname -m` = `aarch64`, `python3 -V` = 3.11.x
> - `pigpiod -v` = **79**, running with **exactly** the five flags above
> - `vcgencmd get_throttled` = **`0x0`** — not "only the sticky bit", **`0x0`**
> - `snd_bcm2835` **not loaded**
> - no I2S overlay in `config.txt`
> - **all six of GPIO 5, 6, 12, 13, 20, 21 read `0`**
>
> The last one is the pull-down check. GPIO5 and GPIO6 idle **HIGH** without external
> pull-downs, so a `1` there means R2 or R3 is not connected and
> [wiring.md §3](wiring.md) is where you go.
>
> Any `get_throttled` bit set means an unresolved supply problem, and every measurement in
> every phase below will be suspect. Fix it first — [power.md §6](power.md).

---

## Phase 2 — Servo alone

**Physically unscrew the motor leads from OUT1–OUT4.** Not "the battery is off" — unscrewed.
The servo phase is the first thing that draws real current from the Pi's 5 V rail and you want
exactly one variable.

**Disconnect the steering linkage from the servo horn.** A servo commanded past a bound
linkage stalls at 700–800 mA on the same rail as the SoC.

```bash
pigs s 18 1500          # centre
pigs s 18 1400
pigs s 18 1600
pigs s 18 1500

# 20 full sweeps, deliberately slowly
for i in $(seq 1 20); do
  pigs s 18 1200; sleep 0.4
  pigs s 18 1800; sleep 0.4
done
pigs s 18 1500; sleep 0.5
vcgencmd get_throttled

pigs s 18 0             # stop the pulse train -> limp
```

> **GATE 2.** All of:
> - The horn moves to each commanded position and **stops** there. Continuous buzzing at rest
>   means the deadband is not working — but at this stage you are driving `pigs` directly with
>   no deadband, so buzz here means a marginal supply or a servo on its endpoint.
> - `vcgencmd get_throttled` is still **`0x0`** after all 20 sweeps.
> - After `pigs s 18 0`, **the horn turns freely by hand.** This proves that `pulse_us == 0`
>   really does release the servo, which is the mechanism behind
>   `servo_relax_when_disarmed` ([power.md §6](power.md)).
> - No 5 V rail dip you can see on a meter clipped to header pins 4 and 6 — under 100 mV.
>
> If `get_throttled` picks up bit 0 or 16 here, **stop.** Fix the supply chain
> ([power.md §6](power.md), mitigations 4 and 5) before you add motors to the problem.

Do **not** fit the horn and linkage yet. That belongs in
[calibration.md §2](calibration.md), after the endpoints are known.

---

## Phase 3 — Encoders by hand

Motor leads still unscrewed. Motor battery still off. The only power in the system is the
powerbank.

This phase both **verifies** the encoders and **measures `encoder_cpr`** — do not carry a
number over from a datasheet. Full procedure in [calibration.md §3](calibration.md); the
counting script is:

```python
# /tmp/count.py — run with the venv python
import pigpio, time
pi = pigpio.pi()
PINS = {23: "L_A", 24: "L_B", 27: "R_A", 22: "R_B"}
count = {g: 0 for g in PINS}
for g in PINS:
    pi.set_mode(g, pigpio.INPUT)
    pi.set_pull_up_down(g, pigpio.PUD_OFF)
    pi.set_glitch_filter(g, 30)
cbs = [pi.callback(g, pigpio.EITHER_EDGE, lambda g, l, t: count.__setitem__(g, count[g] + 1))
       for g in PINS]
try:
    while True:
        print("  ".join("%s=%6d" % (PINS[g], count[g]) for g in PINS), end="\r", flush=True)
        time.sleep(0.2)
except KeyboardInterrupt:
    print()
finally:
    for c in cbs:
        c.cancel()
    pi.stop()
```

Mark each wheel with tape. Turn **exactly ten revolutions**, slowly — under one revolution per
second — in one direction. Record. Repeat three times per wheel, and once in the opposite
direction.

> **GATE 3.** All of:
> - **All four channels toggle.** A channel that never changes is not wired, or the encoder is
>   5 V-only — see [wiring.md §9](wiring.md).
> - Ten revolutions gives an A-channel edge count reproducible to **±2 % across three trials**.
>   At the default `encoder_cpr` of 660 that is 6600 edges, so 6468–6732.
> - Left and right agree within **1 %**. A larger spread means the two gearboxes are not the
>   same ratio, which is worth knowing now rather than in Phase 7.
> - Turning the opposite direction gives the **same** count. The decoder is x2 on channel A
>   only and is direction-blind by design ([INTERFACES.md §3](INTERFACES.md)); direction comes
>   from the commanded H-bridge state.
> - **Stationary for 30 seconds adds zero counts.** Any count at rest is noise pickup — go
>   back to the grounding rules in [wiring.md §5](wiring.md) and the encoder decoupling cap in
>   [wiring.md §6](wiring.md).
>
> **Diagnostic:** if a channel sits at a constant `0` and never moves, try `PUD_UP` instead of
> `PUD_OFF` in the script — some hall boards have open-collector outputs with no onboard
> pull-up. If that fixes it, say so loudly; the firmware needs to know.

Write the measured `encoder_cpr` into `pi/config/config.local.yaml` now — the git-ignored
per-vehicle overlay, **not** the checked-in `telekart.yaml`
([calibration.md §1](calibration.md)).

---

## Phase 4 — Motors, open loop

The first phase that draws real current. Safety glasses on. Wheels off the ground. Hand on the
master switch.

Reconnect the motor leads to OUT1–OUT4. Fresh, fully-charged pack. Regulator already set to
9.0 V per [power.md §2](power.md).

> **Both hardware PWM channels share one clock divider.** When driving them by hand, **always
> give ENA and ENB the same frequency.** `pigs hp 12 1000 250000` followed by
> `pigs hp 13 4000 250000` corrupts channel 0. This is why the firmware only exposes
> `set_pwm_pair()` ([INTERFACES.md §2](INTERFACES.md)) — but `pigs` will let you do it.

### 4.1 One motor, one direction, briefly

```bash
pigs m 5 w; pigs m 6 w; pigs m 20 w; pigs m 21 w   # direction pins to output
pigs w 5 0; pigs w 6 0; pigs w 20 0; pigs w 21 0   # all low

# LEFT forward, 25 % duty, 1 second
pigs w 5 1; pigs w 6 0
pigs hp 12 1000 250000
sleep 1
pigs hp 12 0 0; pigs w 12 0
pigs w 5 0
```

Repeat for left reverse (`w 5 0; w 6 1`), right forward (`w 20 1; w 21 0`, `hp 13 ...`) and
right reverse. **Four combinations, one second each.** Note which wheels turn backwards —
those become `invert_left` / `invert_right`.

### 4.2 The truth table, with your own eyes

This is the one that everybody gets wrong, so confirm it physically.

```bash
# spin the left wheel up
pigs w 5 1; pigs w 6 0; pigs hp 12 1000 400000; sleep 2

# COAST: enable low. The wheel should free-wheel down slowly.
pigs hp 12 0 0; pigs w 12 0

# spin it up again
pigs w 5 1; pigs w 6 0; pigs hp 12 1000 400000; sleep 2

# BRAKE: enable HIGH, both IN pins EQUAL. The wheel should stop hard.
pigs w 5 1; pigs w 6 1
sleep 1
pigs hp 12 0 0; pigs w 12 0; pigs w 5 0; pigs w 6 0
```

`IN1 == IN2` with EN high is a **brake**. Coast requires EN **low**. It is the opposite of
most people's intuition, and getting it backwards means every time you meant to let the car
roll you instead short a spinning motor into the bridge.

### 4.3 Loaded measurements

Both motors, 40 % duty, 30 seconds. DMM on the L298N `+12V`/`GND` screw terminals.

```bash
pigs w 5 1; pigs w 6 0; pigs w 20 1; pigs w 21 0
pigs hp 12 1000 400000; pigs hp 13 1000 400000
# ... measure ...
pigs hp 12 0 0; pigs hp 13 0 0; pigs w 12 0; pigs w 13 0
pigs w 5 0; pigs w 20 0
```

> **GATE 4.** All of:
> - All four direction combinations work. Both wheels turn, both ways.
> - **Coast and brake behave as described in 4.2.** If brake and coast are swapped, your IN
>   pins are crossed.
> - Regulator output at the L298N terminals **≥ 8.5 V** with both motors at 40 %.
> - Pack voltage under that load **≥ 6.5 V**. Below that, the pack is tired — see
>   [power.md §4](power.md).
> - L298N heatsink **< 60 °C** after 30 s at 40 % on both. Field expedient if you have no IR
>   thermometer: you can hold a finger on it for three seconds. **Abort at 80 °C.**
> - `vcgencmd get_throttled` still **`0x0`**. The motors are on a separate domain and must not
>   touch the Pi's rail at all. If this changes now, you have a ground loop or a back-feed —
>   check the L298N `+5V` terminal really is unconnected ([wiring.md §8](wiring.md)).
> - Encoder counts increase while driving, on the correct side. Left duty must move left
>   counts.
> - **Peak pack current below 3.0 A** on the inline meter during a 40 % start.

---

## Phase 5 — Encoder CPU measurement

The phase that decides whether the x2 decode design actually holds on this hardware. If it
fails here, no amount of PID tuning helps.

### The arithmetic you are testing

```
edges per second, per wheel  =  encoder_cpr × RPM / 60

at 660 cpr and 200 RPM       =  2200 /s per wheel
both wheels                  =  4400 /s

each edge is one pigpio notification and one Python callback body
(the body does nothing but `count += 1; last_tick = tick`)
```

### Procedure

Start the control process, arm, and hold both motors at 60 % for 30 seconds. In another SSH
session:

```bash
top -b -n 6 -d 5 | grep -E 'pigpiod|python'
# or, better:
pidstat -p $(pgrep -f telekart.app),$(pgrep -x pigpiod) 5 6
```

Read `loop_p99_us` out of the telemetry stream (byte offset 86 — see
[protocol.md](protocol.md)) or off the app's HUD.

> **GATE 5.** All of:
> - `pigpiod` CPU **< 25 %** of one core.
> - `pigpiod` + control process combined **< 60 %** of one core.
> - **`loop_p99_us` < 12000.** A 10 ms loop with p99 above 12 ms is failing regardless of its
>   mean ([INTERFACES.md §1](INTERFACES.md)). The mean is not the number; p99 is.
> - **Zero `Fault.LOOP_OVERRUN`** (bit 10, `0x400`) over the full 30 s.
> - Total edges over 30 s within **5 %** of `cpr × mean_RPM × 30 / 60`. A shortfall means you
>   are losing edges, and lost counts corrupt odometry silently.
>
> **If it fails,** two levers, in this order:
> 1. Raise the encoder glitch filter from 30 µs. Every edge the filter suppresses is a
>    callback you do not pay for. Watch that the total edge count does not fall — if it does,
>    you have gone too far and are now eating real edges.
> 2. Switch the CPU governor to `performance` ([pi-setup.md §9](pi-setup.md)), then re-check
>    `get_throttled`, because that costs supply headroom.
>
> Do **not** "fix" it by dropping to x1 decoding. x1 costs a pigpiod socket round-trip of
> 50–100 µs *per edge* to read channel B — 100–200 % of a core. It is strictly worse.

---

## Phase 6 — Drive auto-calibration

Wheels off the ground. Full procedure and rationale in
[calibration.md §4](calibration.md).

Run the calibration through the session channel (`MsgType.CALIBRATE`), watch
`CALIBRATION_STATUS`, and confirm `pi/config/calibration.yaml` is written.

> **GATE 6.** All of:
> - Four `max_rpm` entries (`left_fwd`, `left_rev`, `right_fwd`, `right_rev`) within **20 %**
>   of each other. A larger spread means a mechanical problem on one corner, or one bridge
>   channel is weaker than the other.
> - `max_rpm_measured` (the **minimum** of the four) between **120 and 260 RPM**. This is the
>   band the ~1.5 A budget produces at 9 V. **Above 300 you are not current-limited and your
>   `encoder_cpr` is probably wrong. Below 80 you have a mechanical drag or a flat pack.**
>   Do not "accept" a number outside this band — go and find out why.
> - Every deadband between **0.05 and 0.30**.
> - `ff_lut` strictly monotonic after monotonization, for all four entries.
> - `calibration.yaml` written with **`on_ground: false`**, and it reloads on restart.
> - `Fault.CALIBRATION_MISSING` (bit 16, `0x10000`) clears, and
>   `TelemetryFlags.CALIBRATED` (bit 2) sets.
> - `v_max_mm_s` in telemetry is non-zero and equals
>   `max_rpm_measured / 60 × π × wheel_diameter_m × 1000`, within rounding.

Remember what this number is: a **bench** calibration with no load on the wheels. It will be
optimistic. That is why `on_ground` is recorded, and why you re-run it in Phase 10.

---

## Phase 7 — Closed loop

Wheels off the ground. `closed_loop = true`. Tuning procedure in [tuning.md §2](tuning.md) —
this phase is the *acceptance test* for whatever you tuned there.

Step tests at 30 %, 50 % and 80 % of `max_rpm_measured`, then a direction reversal.

> **GATE 7.** All of:
> - **Step 0 → 50 % of `max_rpm_measured` settles to within ±5 % in under 400 ms.**
> - Steady-state error **< 3 %** at 30 %, 50 % and 80 % setpoints.
> - Over five seconds at a constant setpoint, peak-to-peak RPM ripple **< 8 %** of setpoint.
>   Sustained oscillation is `pid_kp` too high — see the symptom table in
>   [tuning.md §2.6](tuning.md).
> - `TelemetryFlags.CLOSED_LOOP` (bit 3) set.
> - **`TelemetryFlags.LIMITER_ACTIVE` (bit 0) never sets during a normal ramp** at
>   `accel_rpm_per_s`. If it does, a firmware protection clamp is engaging during ordinary
>   driving, the HUD will disagree with the car, and it will feel awful
>   ([INTERFACES.md §9](INTERFACES.md)).
> - **Direction reversal is sequenced.** Command forward, then reverse. You must observe:
>   duty → 0 first, then a pause of `direction_deadtime_ms` (30 ms), then the IN pins flip,
>   then ramp. The IN pins must **never** flip under a live enable. Without a scope, the gate
>   is: no clunk, and no current spike above the straight-line draw on the inline meter.
> - Reversal is refused while `|rpm| > reverse_allowed_rpm` (15 RPM).
> - Anti-windup: hold a wheel for two seconds against a 50 % setpoint, then release. **No
>   overshoot burst on release.** A burst means conditional integration is not freezing the
>   integrator while saturated.

---

## Phase 8 — Safety suite

**None of these are optional and 8.4 is the one that must not be skipped.**

### 8.1 Link loss

Armed, 40 % throttle, wheels spinning in the air. Pull the laptop's network connection (or
`sudo ip link set wlan0 down` on the *laptop*, not the car).

The schedule, from
[`constants.py`](../packages/telekart_protocol/telekart_protocol/constants.py):

| Constant | Value | Meaning |
|---|---|---|
| `CONTROL_TIMEOUT_MS` | 200 | the link is declared **stale** this long after the last accepted packet |
| `FAILSAFE_BRAKE_AT_MS` | 50 | brake, 50 ms after stale |
| `FAILSAFE_COAST_AT_MS` | 450 | coast again, 450 ms after stale |
| `FAILSAFE_DISARM_AT_MS` | 1000 | disarm, 1000 ms after stale |

So, in wall-clock from the last packet you actually sent:

```
   0 ms  last good packet
 200 ms  stale  -> COAST      state -> FAILSAFE
 250 ms           BRAKE at failsafe_brake_duty (0.35)
 650 ms           COAST
1200 ms           DISARM      state -> SAFE
```

Coast *then* brake rather than brake immediately, because an unbraked car keeps rolling for
metres — and coast again afterwards so a car left braking does not cook the bridge.

> **GATE 8.1.** All of:
> - Wheel rotation visibly arrested within **700 ms** of the disconnect.
> - `state` reads `FAILSAFE` (3), then `SAFE` (1).
> - `Fault.CONTROL_TIMEOUT` (bit 5, `0x20`) sets.
> - Reconnecting does **not** silently re-arm. You must send a fresh `ARM`.

### 8.2 TCP session drop

Armed, 40 % throttle. Kill the app's TCP session **only** — leave UDP control packets flowing
(the simulator's `--tcp-drop` flag does exactly this).

The TCP connection doubles as a presence signal: if it drops, the car has lost its operator,
and that is an E-stop condition **regardless of whether UDP control packets are still arriving
from somewhere**
([`session.py`](../packages/telekart_protocol/telekart_protocol/session.py)).

> **GATE 8.2.** Motion stops even though valid, correctly-authenticated control packets are
> still being received. If the car keeps driving on UDP alone, the presence check is not
> implemented and a stale second laptop can drive your car.

### 8.3 Local E-stop button

Armed, 40 % throttle. Press the button on GPIO16.

The button is **normally open** and the firmware reads **LOW as pressed**
([wiring.md §2.4](wiring.md)). Before you arm anything, confirm the resting level, because an
NC button wired here latches `ESTOP` on the first tick and the car simply never arms:

```bash
pinctrl set 16 ip pu; pinctrl get 16     # must read level=1 at rest
```

> **GATE 8.3.** All of:
> - At rest, before arming, `state` is `SAFE` — **not** `ESTOP`. If the car boots straight
>   into `ESTOP`, you have a normally-closed button on GPIO16. Replace it with a
>   normally-open one; there is no inversion parameter.
> - Duty at zero within **one control tick (≤10 ms)** of the press.
> - `state` = `ESTOP` (4).
> - `Fault.ESTOP_LATCHED` (bit 15, `0x8000`) sets, and it is in `CRITICAL_FAULTS`, so it
>   forces a disarm.
> - **Releasing the button does not clear it.** An explicit `CLEAR_ESTOP` is required.
> - **Unplugging the button's connector does *not* produce an E-stop, and that is expected.**
>   A normally-open button is fail-silent by construction. The battery master switch is the
>   fail-safe stop on this car (8.7, and [power.md §7](power.md)) — verify *that* instead, and
>   do not talk yourself into believing the panel button covers a broken wire.

### 8.4 `kill -9` — DO NOT SKIP THIS

`pigpiod` **retains GPIO state after its client dies.** A segfault or a `kill -9` at 80 % duty
leaves the motors running **indefinitely**. This is the single fact that the entire four-layer
panic-stop design exists to handle, and this is the test that proves the design works on your
car.

#### Step 1 — see the hazard, safely

**Unscrew the motor leads from OUT1–OUT4.** Put the DMM on DC volts across OUT1/OUT2.

```bash
sudo systemctl stop telekart-control
pigs w 5 1; pigs w 6 0
pigs hp 12 1000 600000
```

`pigs` has already exited. There is no process left to kill. **Read the meter: the bridge is
still switching.** That is the hazard, in one command, with nothing spinning.

```bash
pigs hp 12 0 0; pigs w 12 0; pigs w 5 0
```

#### Step 2 — prove the mitigation

Reconnect the motor leads. Wheels off the ground. Arm, command ~60 % throttle from the app,
then:

```bash
sudo kill -9 $(systemctl show -p MainPID --value telekart-control)
```

Repeat with the systemd-mediated path:

```bash
sudo systemctl kill -s SIGKILL telekart-control
```

> **GATE 8.4.** For **both** kills:
> - Wheels stop within **1 second**.
> - **They stay stopped.** Watch for ten full seconds.
> - `journalctl -u telekart-control` shows `telekart-panic-stop` ran.
> - After the automatic restart, the car comes up **disarmed** and the wheels do not move.
>
> **If the wheels keep turning: the build is not safe to drive and every gate below this one
> is void.** The `ExecStopPost=` line is layer 2 of the panic chain and **the only layer that
> covers `SIGKILL`** ([INTERFACES.md §8](INTERFACES.md), [pi-setup.md §7.1](pi-setup.md)).
> Fix it before you do anything else.

### 8.5 Stall detection

Armed, 30 % throttle. Stop one wheel with a block of wood. **Not your hand.**

Stall detection is hardware protection, not a nicety: two stalled motors put roughly 7 W into a
bridge whose stock heatsink handles 2–3 W.

> **GATE 8.5.** All of:
> - `Fault.STALL_L` (bit 0) or `STALL_R` (bit 1) sets within `stall_detect_ms` **+150 ms**
>   (i.e. 750 ms at the default 600).
> - Duty on that motor falls to zero.
> - The fault is **sticky**: it stays set after you remove the block, until `CLEAR_FAULTS`.
> - The car does **not** disarm — `STALL_*` is not in `CRITICAL_FAULTS` (`0x8340`), so it
>   inhibits drive and warns rather than forcing a disarm.

### 8.6 Brownout, and that it is not confused with a stall

Set `closed_loop = false` temporarily and command a hard step to 100 % on both motors from
rest, with a partly-discharged pack.

Both encoders reaching zero **simultaneously** while commanded is the boost regulator
tripping, not a mechanical stall. It gets its own fault code so the two are never confused
during diagnosis.

> **GATE 8.6.** Either:
> - The regulator trips, and `Fault.BROWNOUT` (bit 4, `0x10`) sets while `STALL_L`/`STALL_R`
>   do **not**; or
> - It does not trip, in which case your supply beats the design budget — **write down the
>   peak current you measured** and move on.
>
> A trip that reports `STALL_L | STALL_R` instead of `BROWNOUT` is a **fail**. You will spend
> an afternoon looking for a mechanical fault that is not there.

Put `closed_loop` back to `true`.

### 8.7 Master switch under load

Armed, 60 % throttle. Throw the master switch.

> **GATE 8.7.** All of:
> - Motors stop instantly.
> - **The Pi stays up.** This proves the two power domains are genuinely separate
>   ([power.md §1](power.md)). If the Pi reboots, you have back-fed it from the L298N `+5V`
>   terminal.
> - `Fault.BROWNOUT` may set. That is correct behaviour, not a fault in the test.
> - **On switching back on, the wheels do not move.** That is layer 3 — the external
>   pull-downs — doing its job.

### 8.8 Reboot under load

Armed, 60 % throttle. `sudo reboot`.

> **GATE 8.8.** All of:
> - Wheels stop within **2 seconds** (`ExecStopPost` during shutdown).
> - **No twitch at any point during the ~30 s boot.** This is the specific window layer 3
>   covers and nothing else can: firmware is not running, pigpiod is not running, and GPIO5/6
>   would idle HIGH without the external pull-downs.
> - The car comes up **disarmed**.

### 8.9 Arming preconditions

Arming requires **all four**: an explicit `ARM`, throttle at neutral for `arm_neutral_ms`
(500 ms), a valid session, and no active critical fault
([INTERFACES.md §4](INTERFACES.md)).

Deny each one individually and confirm each is enforced:

| Attempt | Expected |
|---|---|
| `ARM` while the throttle is held at 30 % | refused, reason cites neutral |
| `ARM` with throttle neutral for only 100 ms | refused |
| `ARM` before the TCP handshake completes | refused / `NOT_ALLOWED_IN_STATE` |
| `ARM` with `Fault.ESTOP_LATCHED` set | refused |
| All four satisfied | accepted, `state` → `ARMED` |

> **GATE 8.9.** All five rows behave as listed. And: **the desktop app must show `SAFE` until
> the car's telemetry says `ARMED`.** The app never asserts it is armed because it sent an
> `ARM` — it displays the state the car reports.

---

## Phase 9 — Camera

The control process must already be passing Phase 5 and Phase 7, because the whole point of
this phase is proving the camera does not disturb them.

```bash
rpicam-hello --list-cameras
sudo systemctl start telekart-video
```

Run 640×480 @ 30 fps for **five minutes** with the app connected.

**Glass-to-glass latency measurement:** put a phone running a millisecond stopwatch in front of
the camera, then photograph the phone and the monitor showing the video feed **in the same
frame**. Subtract. Do it five times, take the median. Anything else you might do measures
something other than what you care about.

> **GATE 9.** All of:
> - `rpicam-hello --list-cameras` reports `imx219`.
> - Five minutes sustained at 640×480 @ 30 with **≤2 % dropped frames** — count sequence gaps
>   in the video frame header, or occurrences of `VideoFrameFlags.DROPPED_BEFORE` (bit 1).
> - Video process CPU **< 35 %** of one core.
> - **Control loop `loop_p99_us` rises by no more than 1000 µs versus Phase 5.** This is the
>   claim being tested: camera work must not inject jitter into the 100 Hz loop. If it rises
>   more, the two-process split is not doing its job.
> - Glass-to-glass median **< 150 ms**. Higher, and the first thing to check is
>   **`thread_type`** on the decoder — see the boxed warning in [tuning.md §5](tuning.md).
> - RSS of both processes stable over the five minutes.
> - `TelemetryFlags.VIDEO_ACTIVE` (bit 8) set.

---

## Phase 10 — Integration, wheels on the ground

**The first time the car touches the floor.** Everything below has been proven in the air.

### Preconditions

- [ ] Every gate 0–9 passed. Not "mostly".
- [ ] Clear, flat, hard floor. **Minimum 5 × 5 m.** No stairs, no drops, no glass.
- [ ] Nothing breakable in a 10 m radius.
- [ ] Master switch reachable, or a second person holding the car.
- [ ] **`ControlFlags.PIT_LIMITER` engaged** (`pit_duty` 0.25). Take the training wheels off
      later.
- [ ] Fresh pack.

### 10.1 Straight-line tracking

Steer input **exactly** 0. Drive 5 m at a low constant speed. Measure lateral deviation from
the start line.

> **GATE 10.1.** Deviation **< 0.3 m over 5 m**. If more, go to
> [calibration.md §5](calibration.md) — a 1° steering error alone produces over a metre of
> deviation across 5 m with this wheelbase, so this gate is almost always a trim problem, not
> a drivetrain problem.

### 10.2 Odometry closure

Drive a 5 m square, returning to the start. Compare the reported pose to the physical start
point.

Expect drift: with no IMU, heading drifts, and 5–15 % closure error on a 5 m square is the
stated expectation ([INTERFACES.md §5](INTERFACES.md)), not a defect.

> **GATE 10.2.** Closure error **< 15 % of the 20 m perimeter**, i.e. under 3 m. Worse than
> that means geometry ([calibration.md §6](calibration.md)) or `encoder_cpr`.

### 10.3 On-ground re-calibration

Re-run the drive auto-calibration **on the ground**, with `on_ground: true` recorded.

> **GATE 10.3.** `max_rpm_measured` falls **20–40 %** versus the bench figure from Phase 6.
> That drop is the load, and seeing it is how you know both numbers are real. **A figure that
> does not drop at all means the wheels were not touching, or the calibration ran open-loop.**
> `v_max_mm_s` in telemetry updates, and the desktop speedometer rescales with no code change
> on either side.

### 10.4 Sustained drive

Ten minutes of ordinary driving with the pit limiter off.

> **GATE 10.4.** All of:
> - **Zero faults** for the full ten minutes.
> - `slip_index` **< 0.2** during smooth driving. Spikes during hard acceleration are real
>   wheelspin and are fine; a persistently high value means bad calibration.
> - RTT p99 **< 60 ms**, computed from `echo_client_time_us`.
> - `TelemetryFlags.LIMITER_ACTIVE` (bit 0) lights **rarely, and only on deliberate abuse**.
>   Frequent activation means the app's rate limits are looser than the firmware's — the app
>   owns feel, the firmware owns protection, and the app's limits must be the tighter of the
>   two.
> - `vcgencmd get_throttled` still `0x0` at the end.
> - L298N heatsink **< 70 °C** at the end.

### 10.5 Soak

Sixty minutes under `tracemalloc` on the desktop side, asserting stable RSS.

> **GATE 10.5.** Desktop RSS stable over 60 minutes. This is what guards the `FrameBundle`
> lifetime: a `QImage` wrapping a PyAV plane without holding the frame alive is a
> use-after-free that crashes intermittently and is miserable to diagnose
> ([INTERFACES.md §9](INTERFACES.md)). The soak is not optional.

---

## Gate summary

| Phase | The number that matters |
|---|---|
| 0 | Master switch reachable without standing up |
| 1 | `get_throttled` = `0x0`; GPIO 5, 6, 12, 13, 20, 21 all read `0` |
| 2 | `get_throttled` still `0x0` after 20 sweeps; servo limp at `pigs s 18 0` |
| 3 | 10 revs reproducible to ±2 %; **zero** counts at rest for 30 s |
| 4 | Regulator ≥ 8.5 V loaded; heatsink < 60 °C; brake ≠ coast confirmed by eye |
| 5 | `loop_p99_us` < 12000; combined CPU < 60 % of one core |
| 6 | `max_rpm_measured` in 120–260 RPM; four values within 20 % |
| 7 | 50 % step settles ±5 % in < 400 ms; `LIMITER_ACTIVE` never fires |
| 8 | **`kill -9` at 60 % duty → wheels stop within 1 s and stay stopped** |
| 9 | `loop_p99_us` rises ≤ 1000 µs vs Phase 5; glass-to-glass < 150 ms |
| 10 | < 0.3 m deviation over 5 m; square closes within 15 % |
