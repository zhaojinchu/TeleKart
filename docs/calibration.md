# TeleKart v2 — Calibration

Every number in this document is **measured on your car**. None of them are derived from a
datasheet, a gearbox marking, or the previous build. Anything in
[`params.py`](../packages/telekart_protocol/telekart_protocol/params.py) marked
`measured=True` is expected to be overwritten by a procedure here — treating a guess as a
measurement is how you spend an afternoon tuning a PID against the wrong plant gain.

**Companion documents:** [bringup.md](bringup.md) for the phase this belongs to ·
[tuning.md](tuning.md) for what happens after · [wiring.md](wiring.md) · [power.md](power.md)

---

## 1. Where the numbers live, and the order to measure them in

### Files

**There is no `pi/config/config.yaml`** — that name appears nowhere in the firmware. The
firmware loads three layers, in this order, each overriding the last (`VehicleConfig.load` in [`config.py`](../pi/telekart/config.py); the paths
are `DEFAULT_CONFIG_PATH` / `LOCAL_CONFIG_PATH` / `CALIBRATION_PATH` in
[`constants.py`](../pi/telekart/constants.py)):

| File | Contents | Written by | Hand-edit? | In git? |
|---|---|---|---|---|
| `pi/config/telekart.yaml` | Pins and the defaults that are **the same on every car** | You, in an editor | Yes, sparingly | **Yes** |
| `pi/config/config.local.yaml` | **Everything you measure below**, plus `car_id` and `shared_key`. Also where `SET_PARAMS` lands when the firmware calls `save_local()` | You **and** the firmware | **Yes — this is the one** | No, git-ignored |
| `pi/config/calibration.yaml` | `max_rpm`, `deadband`, `ff_lut`, `measured_at`, `on_ground` | The auto-calibration routine | **Never** | No, git-ignored |

**Every number in this document goes in `config.local.yaml`**, not in `telekart.yaml`. That
split is the whole point of the overlay: `telekart.yaml` is checked in and shared, so a servo
centre or a wheel diameter measured on *your* car must never silently become the default on
someone else's. `config.local.yaml` needs only the keys you are overriding — anything absent
falls through to `telekart.yaml`:

```yaml
# pi/config/config.local.yaml
car_id: telekart-01
shared_key: something only you know
params:
  encoder_cpr: 658
  wheel_diameter_m: 0.0642
  steer_center_us: 1500
```

`calibration.yaml` is a **third** file on purpose, so an automated bring-up run can never
clobber a value somebody typed ([INTERFACES.md §6](INTERFACES.md)). If you feel the urge to
hand-edit `calibration.yaml`, what you actually want is to re-run the calibration with the
condition you disagreed with fixed.

### Order

Each step depends on the ones above it. Doing them out of order means doing them twice.

| # | Measure | Section | Bring-up phase | Needs |
|---|---|---|---|---|
| 1 | Geometry: wheel, wheelbase, track | [§6](#6-geometry) | before Phase 3 | a tape measure and no power at all |
| 2 | `encoder_cpr` | [§3](#3-encoder-counts-per-revolution) | Phase 3 | geometry, powerbank only |
| 3 | Servo centre, endpoints, lock angle | [§2](#2-servo-centre-and-endpoints) | Phase 2 | powerbank only |
| 4 | Drive calibration, **on the bench** | [§4](#4-drive-auto-calibration) | Phase 6 | 1–3, motor battery |
| 5 | Steering trim for straight tracking | [§5](#5-steering-trim-for-straight-tracking) | Phase 10.1 | 1–4, wheels **on the ground** |
| 6 | Drive calibration, **on the ground** | [§4.5](#45-re-run-on-the-ground) | Phase 10.3 | 1–5 |

### Keep a build log

Date, pack, state of charge, ambient temperature, and every number you measured. The
`measured_at` field in `DriveCalibration` exists for the same reason. In three weeks, "the car
got slower" is only answerable if you know what it used to be and under what conditions.

---

## 2. Servo centre and endpoints

**Wheels off the ground. Motor leads unscrewed from OUT1–OUT4. The steering linkage
disconnected from the servo horn.**

That last one is not caution, it is the procedure. A servo commanded into a bound linkage
stalls at 700–800 mA on the same 5 V rail as the SoC ([power.md §6](power.md)). You will
command it into a bound linkage during this procedure — that is how you find the endpoint —
so the linkage must not be attached while you are looking for it.

### 2.1 Fit the horn at 1500 µs

```bash
pigs s 18 1500
```

With the servo holding at 1500 µs, fit the horn as close to your intended straight-ahead
position as the spline allows.

**The HS-311 has a 24-tooth spline: 360 / 24 = 15° per tooth.** You physically cannot get
closer than **±7.5°**. Do not fight it. Get within a tooth, then take up the rest with the
tie-rod length and, finally, with `steer_trim_us` in [§5](#5-steering-trim-for-straight-tracking).

Leave `steer_center_us` at 1500 for now. It stays at 1500 unless the endpoints turn out to be
badly asymmetric — see 2.3.

### 2.2 Walk outward from 1500 µs

**Linkage still disconnected.** Step outward in **25 µs** increments, pausing about half a
second at each, in both directions:

```bash
for us in 1500 1475 1450 1425 1400 1375 1350 1325 1300 1275 1250 1225 1200; do
  echo -n "$us  "; pigs s 18 $us; sleep 0.5
done
pigs s 18 1500; sleep 1
for us in 1500 1525 1550 1575 1600 1625 1650 1675 1700 1725 1750 1775 1800; do
  echo -n "$us  "; pigs s 18 $us; sleep 0.5
done
pigs s 18 1500
```

At each step, watch and listen for:

| Sign | Meaning |
|---|---|
| The horn moves cleanly and then goes quiet | fine, keep going |
| A continuous hum or buzz that does not stop | **the servo is straining. You have gone past the mechanical limit.** |
| The horn stops advancing but the servo keeps drawing | same thing |
| Any change in the Pi's status LED / a reboot | you have browned out the rail. Stop, back off 100 µs |

**Back off 50 µs from the first sign of strain** and record that.

> **Never leave the servo at a pulse where it buzzes.** Not for five seconds "just to check".
> That is 700–800 mA into the Pi's rail and it is precisely the condition
> [bringup.md](bringup.md) Phase 2 gates against.

Record:

- `steer_min_us` — the **left** limit
- `steer_max_us` — the **right** limit

Keep `steer_min_us` numerically **below** `steer_max_us`. If that makes the car steer the
wrong way once the linkage is on, the fix is `steer_invert = true`, **not** swapping the two
numbers.

### 2.3 Re-centre if the endpoints are asymmetric

Compute the midpoint of what you found:

```
midpoint = (steer_min_us + steer_max_us) / 2
```

If that is more than about 40 µs from 1500, your horn is a tooth off or the tie rod is the
wrong length. **Fix it mechanically**, then repeat 2.1 and 2.2. Only set `steer_center_us`
away from 1500 if the mechanics genuinely will not centre — a large software offset means the
usable travel is asymmetric and you lose lock in one direction.

### 2.4 Connect the linkage and adjust the tie rod

With the servo commanded to `steer_center_us`, adjust the tie-rod length until the front
wheels point straight ahead by eye against a floor line. Then re-run 2.2 **with the linkage
connected**, because the linkage's own travel limits are usually tighter than the servo's:

- If the wheels reach their steering-rack stop before the servo reaches the endpoint you
  recorded, **shorten the endpoint** to where the wheels stop.
- Never leave an endpoint that pushes the rack into its stop. That is a permanent stall
  condition every time you go to full lock.

### 2.5 Measure `steer_max_deg`

Command full lock. With a protractor or a phone level app against the wheel face, measure the
angle of **each** front wheel relative to straight ahead.

The two will differ — that is Ackermann geometry doing its job, and it is correct. The bicycle
model wants a single virtual steer angle, so **record the average of the inner and outer wheel
angles.**

Repeat at the other lock. If the two locks differ by more than 3°, go back to 2.3.

#### Cross-check it by driving a circle

The protractor measurement is easy to get wrong — the most common error is measuring the servo
horn's angle rather than the wheel's, which over-states lock badly.

```
R = wheelbase_m / tan(steer_max_deg)
```

With the defaults (0.200 m, 24°): `R = 0.200 / 0.4452 = 0.449 m`, so the rear axle centre
should trace a circle about **0.90 m across** at full lock.

Do it on the ground in Phase 10, mark the path, and measure it. If the real circle is much
larger than the formula predicts, `steer_max_deg` is over-stated and both your odometry and
your electronic differential are wrong in the same direction.

### 2.6 `steer_hold_us`

Leave it at the default of **8 µs**.

The servo driver never writes a pulse differing from the last by less than this. That deadband
is the only thing standing between you and a servo that buzzes continuously as the input axis
dithers by a count or two, and it is carried over from the previous firmware deliberately
([INTERFACES.md §3](INTERFACES.md)).

Raising it above ~20 µs makes small steering inputs feel notchy. Lowering it to 0 brings the
buzz back.

---

## 3. Encoder counts per revolution

### What the number actually means

`encoder_cpr` is **A-channel edges per revolution of the output shaft, as this decoder counts
them.** It is not "quadrature counts", not "hall PPR", and not the gear ratio times anything.
The firmware decodes x2 — `EITHER_EDGE` on channel A only — so `cpr` is exactly what you will
count in the procedure below, with no conversion factor.

### Why you measure it instead of computing it

The ratio printed on a GA37-520 gearbox is a rounded marketing figure for a ratio that is
almost never an integer. A "1:30" is frequently 29.86:1 or similar.

A 0.5 % ratio error sounds like nothing. It is 25 mm of odometry error over 5 m — about the
width of your tape measure. But it also biases **every RPM reading and therefore every PID
setpoint** by the same 0.5 %, permanently, in a way that looks like a tuning problem rather
than a measurement problem.

The parameter's own description says it: *"Verified by hand-turning ten revolutions, not
derived from the nameplate gear ratio."*

### Procedure

Motor leads unscrewed. Motor battery off. Powerbank only. Use the counting script in
[bringup.md](bringup.md) Phase 3.

1. Put a strip of masking tape on the wheel and a matching mark on the chassis.
2. Note the count.
3. Turn the wheel **exactly ten revolutions** in one direction. Go slowly — **under one
   revolution per second.** At 660 cpr that is still 660 edges/s, which is well within the
   glitch filter and the sample rate, but rushing it is how you introduce doubt.
4. Note the count. `cpr = (count_after − count_before) / 10`.
5. **Repeat three times per wheel**, plus once in the opposite direction.

### Why ten and not one

- The alignment error at the start and stop marks is a fixed few counts. Over ten revolutions
  it is amortised tenfold.
- A single defective magnet or a burr on one gear tooth averages out.
- Ten revolutions of a 65 mm wheel is 2.04 m of travel, which you can also sanity-check
  against a tape measure once the wheel is on the ground.

### Acceptance

| Check | Threshold |
|---|---|
| Three trials on one wheel | agree within **±2 %** |
| Left vs. right | agree within **±1 %** |
| Forward vs. reverse turn | **identical** — the decoder is direction-blind by design |
| Stationary for 30 s | **exactly zero** additional counts |

A left/right disagreement above 1 % means the two gearboxes are not the same ratio. Record
both and raise it with whoever writes the firmware — a single global `encoder_cpr` cannot
represent two different gearboxes.

Any count at rest is noise pickup. Go back to [wiring.md §5](wiring.md) (grounding) and
[wiring.md §6](wiring.md) (the encoder decoupling cap).

### The resolution you end up with

```
distance per count = π × wheel_diameter_m / encoder_cpr
                   = π × 0.065 / 660
                   = 0.309 mm
```

Sub-millimetre. That is far finer than the uncertainty in the tyre's rolling radius, which is
why there is no reason to chase more resolution — and why the low-speed velocity estimate is
limited by *timing*, not by counts (see [tuning.md §2.1](tuning.md)).

Write the measured value into `config.local.yaml` (see [§1](#1-where-the-numbers-live-and-the-order-to-measure-them-in)). `encoder_cpr` is `requires_disarm=True`, so it
cannot be changed while the car is armed.

---

## 4. Drive auto-calibration

This is the routine that produces `calibration.yaml`. Everything downstream — the speedometer,
the mixer, the feedforward, the speed-sensitive steering — reads from it.

### 4.1 Preconditions

- [ ] Geometry and `encoder_cpr` already measured and in `config.local.yaml`.
- [ ] **Wheels off the ground.** ≥20 mm clearance.
- [ ] Fully charged pack. A calibration on a half-flat pack measures the pack.
- [ ] Regulator confirmed at 9.0 V no-load and ≥8.5 V under a 40 % load
      ([power.md §2](power.md)).
- [ ] `Fault` register clear.
- [ ] Bring-up Phases 1–5 passed. In particular Phase 5, because a calibration run on a loop
      that is dropping deadlines measures the deadlines.

Kick it off with `MsgType.CALIBRATE` on the session channel; progress arrives as
`CALIBRATION_STATUS` messages ([protocol.md](protocol.md)).

### 4.2 What it does, and why

For each of the four combinations {left, right} × {forward, reverse}, **one motor at a time**:

**Step 1 — find the deadband.** Ramp duty up from 0 in **0.02** increments, holding each for
**300 ms**. Record the duty at which `|rpm|` first exceeds `stall_rpm_threshold` (5 RPM). That
is the deadband for that wheel in that direction.

**Step 2 — find `max_rpm`.** Continue to `max_duty` (0.85), hold **1.5 s**, and record the
steady RPM.

**Step 3 — build the feedforward table.** Step back down in **0.05** increments, recording
`(rpm, duty)` pairs. Monotonize the result — a non-monotonic feedforward table produces a
control surface with a fold in it, and the PID will hunt across the fold forever.

#### Why one wheel at a time

Because two motors at once measures **the regulator**, not the motors. The 1.5 A combined
budget is a shared resource; running both means each wheel's `max_rpm` is really "the speed
this wheel reaches while the other one is also drawing". You would then set targets that can
never both be met.

#### Why both directions

The L298's two output stages are not identical, and a brush motor is not electrically
symmetric. Forward and reverse deadbands commonly differ by **0.03–0.05** on the same motor.
Using a single deadband means one direction has a dead spot at the bottom of the throttle and
the other creeps.

#### Why `on_ground` is recorded

A bench calibration has **no load on the wheels** and its `max_rpm` will be optimistic by
20–40 %. Recording which condition produced the number is what makes it interpretable three
weeks later. Everything downstream reads `max_rpm_measured` — the **minimum across all four**
entries — so a single optimistic corner cannot inflate the whole car.

### 4.3 Acceptance

| Check | Threshold | If it fails |
|---|---|---|
| Four `max_rpm` values | within **20 %** of each other | mechanical drag on one corner, or one bridge channel is weak |
| `max_rpm_measured` | **120–260 RPM** | see below |
| Each deadband | **0.05–0.30** | below 0.05, your `stall_rpm_threshold` is firing on noise; above 0.30, check for binding |
| `ff_lut` | strictly monotonic | re-run; if it repeats, the loop is unstable and you have a Phase 5 problem |
| File | written, `on_ground: false`, reloads on restart | |
| Faults | `CALIBRATION_MISSING` (bit 16) clears, `TelemetryFlags.CALIBRATED` (bit 2) sets | |

**On the 120–260 RPM band:** that is what the ~1.5 A budget produces at 9 V through a Darlington
bridge, which is nothing like the 360 RPM nameplate. The nameplate assumes 12 V at the motor
terminals with unlimited current; you have ~7 V and a hard 1.5 A ceiling
([power.md §4](power.md)).

- **Above 300 RPM:** you are not current-limited, which means `encoder_cpr` is probably wrong
  — most likely by an integer factor. Go back to [§3](#3-encoder-counts-per-revolution).
- **Below 80 RPM:** mechanical drag, a flat pack, or the regulator is not actually at 9 V.

Do **not** accept a number outside the band and carry on. Find out why.

### 4.4 What comes out, and who reads it

```
max_rpm_measured  =  min(left_fwd, left_rev, right_fwd, right_rev)

v_max_mps         =  max_rpm_measured / 60 × π × wheel_diameter_m
```

With 200 RPM and a 65 mm wheel: `200/60 × π × 0.065 = 0.681 m/s`.

`v_max_mps` is published in **every telemetry packet** as `v_max_mm_s`, and the desktop
speedometer scales off it. That is the mechanism that lets the same build work with today's
L298N-limited drivetrain and with a MOSFET bridge later, with **no code change on either
side** ([INTERFACES.md §6](INTERFACES.md), [power.md §9.2](power.md)).

> **A landmine worth flagging:** the pipeline applies deadband compensation **after**
> `feedforward + PID` ([INTERFACES.md §4](INTERFACES.md)). So the `ff_lut` must be a curve
> **relative to the deadband**, not one that already includes it. If the table bakes the
> deadband in *and* the compensator adds it again, you get a jump at the bottom of the
> throttle that no amount of PID tuning removes. See [tuning.md §3](tuning.md).

### 4.5 Re-run on the ground

After [bringup.md](bringup.md) Phase 10.1 and 10.2 pass, re-run the whole calibration with the
wheels on the floor and `on_ground: true` recorded.

> **Expect `max_rpm_measured` to fall by 20–40 %.** That drop *is* the load, and seeing it is
> how you know both numbers are real. A figure that does not drop at all means the wheels were
> not actually touching, or the run went open-loop.

Keep both results in your build log. The bench number is the drivetrain's ceiling; the ground
number is the car's.

---

## 5. Steering trim for straight tracking

### Why this parameter matters more than it looks

`steer_trim_us` is described in `params.py` as *"the single biggest lever on odometry heading
drift"*. Here is why, in numbers.

The bicycle model's curvature is `κ = tan(δ) / wheelbase`. Lateral offset accumulated over a
distance `s` is approximately `y ≈ κ·s² / 2`. With this car's 200 mm wheelbase, a **one degree**
steering bias produces:

```
y = tan(1°) × 5² / (2 × 0.200) = 0.01746 × 25 / 0.4 = 1.09 m over 5 m
```

**Over a metre of drift, from one degree.** A short wheelbase amplifies steering error, and
this car has a very short wheelbase.

### 5.1 First: rule out a drivetrain imbalance

A car can pull for two completely different reasons and the fixes are unrelated.

Lift the car, arm it, command a straight-line target at 50 % of `max_rpm_measured`, and compare
`rpm_l` against `rpm_r` in telemetry.

| Observation | Cause | Fix |
|---|---|---|
| RPMs agree within **3 %**, car still pulls | steering bias | this section |
| RPMs differ by more than 3 % | drivetrain imbalance | check `ff_lut`, mechanical drag; `straight_sync_gain` (30 RPM/m, active only near centre so it does not fight the electronic differential) is what corrects residual distance divergence |

Do not trim out a drivetrain problem. It will come back the moment you turn.

### 5.2 Procedure

Wheels on the ground. Clear 6 m of flat, hard floor. Chalk or tape a start line and a line
5.00 m away.

1. Set `steer_trim_us = 0`. Temporarily set `steer_deadzone = 0` so axis noise cannot hide in
   it.
2. Command steering **exactly** 0 — from a script, not a wheel you are holding.
3. Drive 5.00 m at a low constant speed, around 30 % of `max_rpm_measured`.
4. Measure the **lateral deviation** at the 5 m line.

### 5.3 The correction

```
δ_deg      = degrees( atan( 2 × wheelbase_m × deviation_m / distance_m² ) )
µs_per_deg = (steer_max_us − steer_min_us) / (2 × steer_max_deg)
trim_us    = δ_deg × µs_per_deg
```

With the default geometry (`wheelbase 0.200`, `steer_min 1200`, `steer_max 1800`,
`steer_max_deg 24`):

```
µs_per_deg = 600 / 48 = 12.5 µs per degree
```

| Deviation over 5 m | Implied steer error | Trim correction |
|---|---|---|
| 0.10 m | 0.09° | 1.1 µs |
| 0.25 m | 0.23° | 2.9 µs |
| 0.50 m | 0.46° | 5.7 µs |
| 1.00 m | 0.92° | 11.5 µs |

Note how small these are. Trimming in 50 µs steps overshoots wildly; work in **2–5 µs**.

### 5.4 Getting the sign right

**Do not reason about it.** Servo direction, linkage geometry and `steer_invert` all interact
and it is genuinely easy to talk yourself into the wrong answer.

Nudge `steer_trim_us` by **+20 µs**, drive the 5 m again, and see which way the deviation
moved. Now you know the sign, and the magnitude comes from the table.

### 5.5 Converge, then verify

Iterate. Two or three passes should get you there.

> **Target: deviation < 0.10 m over 5 m.** [bringup.md](bringup.md) Phase 10.1 gates at
> 0.30 m; 0.10 m is what you should actually achieve, and the difference shows up directly in
> the Phase 10.2 square-closure number.

**Then verify in reverse.** Drive the same 5 m backwards.

| Result | Meaning |
|---|---|
| Reverse tracks straight too | The trim is correct. Done. |
| Reverse deviates about **twice** as much, the other way | **Linkage slop, not trim.** You have trimmed out a mechanical offset that reverses sign with the load direction. Tighten the tie-rod ends and the rack; then re-do this section. |

That second case is the one that will otherwise haunt your odometry forever, because the
correction is right half the time.

---

## 6. Geometry

Three measurements, a tape measure, and no power. Do these first — everything else depends on
them.

### 6.1 `wheel_diameter_m` — the rolling diameter, under load

**Not the free diameter measured with calipers on a wheel in your hand.**

A foam or pneumatic tyre compresses 1–3 mm under the car's weight. On a 65 mm wheel that is
**3–5 % of the diameter**, which is larger than every other error in the odometry chain
combined. Measure it the way the car experiences it.

**Procedure:**

1. Car on the ground, at its normal weight, with the battery fitted.
2. Tape mark on one rear tyre; chalk mark on the floor at the contact point.
3. Push the car in a straight line for **exactly ten wheel revolutions**, keeping the tape
   mark's return-to-floor as the count.
4. Mark the floor at the finish. Measure the distance.

```
wheel_diameter_m = distance_m / (10 × π)
```

At the default 0.065 m the expected distance is `10 × π × 0.065 = 2.042 m`. Measured to ±5 mm,
that is 0.25 % accuracy — better than you need.

Do both rear wheels. If they differ by more than 2 %, one tyre is under-inflated or has a
different compound, and you should fix that rather than average it.

### 6.2 `wheelbase_m`

Front axle centreline to rear axle centreline, wheels pointing dead straight, measured **along
one side** of the car. Not diagonally.

To **±1 mm.** At 200 mm nominal, 1 mm is 0.5 %, and it multiplies straight into every heading
calculation.

### 6.3 `track_width_m`

Centre of the **left rear** tyre's contact patch to centre of the **right rear** tyre's contact
patch.

**Rear, not front.** This is the driven axle, and it is what the electronic differential uses.
Measuring the front track is the single most common mistake here and it silently corrupts the
mixer.

To **±1 mm**. Easiest method: measure outside-to-outside, then subtract one tyre width (they
are the same width).

### 6.4 Cross-check: the differential split

The electronic differential is not optional — with two motors on a rear axle, Ackermann front
steering and no mechanical diff, the rear wheels **must** run at different speeds in a turn, or
per-wheel PID fights the geometry and scrubs the tyres ([INTERFACES.md §4](INTERFACES.md)).

```
split = track_width_m × tan(steer_max_deg) / (2 × wheelbase_m)
```

With the defaults:

```
0.150 × tan(24°) / (2 × 0.200) = 0.150 × 0.4452 / 0.400 = 0.167
```

**±16.7 % at full lock — a 33 % spread between the two wheels.** Not a rounding error.

Sanity-check your own measurements against this:

| Your split | Verdict |
|---|---|
| 0.10 – 0.25 | plausible for this class of chassis |
| **> 0.30** | you almost certainly measured the **front** track. Re-do 6.3. |
| **< 0.05** | `steer_max_deg` is under-stated, or you measured the wheelbase wrong |

### 6.5 Cross-check: the turning circle

```
R = wheelbase_m / tan(steer_max_deg)
```

Defaults: `0.200 / 0.4452 = 0.449 m`, so a full-lock circle about **0.90 m across**.

Drive one on the ground in Phase 10, chalk the path, measure it. A measured circle noticeably
larger than the prediction means `steer_max_deg` is over-stated — go back to
[§2.5](#25-measure-steer_max_deg). This is the most common source of a car whose odometry
under-reports how much it turned.

---

## 7. Final checklist

Everything measured, in one place. Copy the values into your build log.

| Parameter | File | Source | Cross-check |
|---|---|---|---|
| `wheel_diameter_m` | `config.local.yaml` | §6.1, rolling, ten revolutions | both rear wheels within 2 % |
| `wheelbase_m` | `config.local.yaml` | §6.2, ±1 mm | turning circle §6.5 |
| `track_width_m` | `config.local.yaml` | §6.3, **rear**, ±1 mm | differential split §6.4 |
| `encoder_cpr` | `config.local.yaml` | §3, ten hand revolutions ×3 | L/R within 1 % |
| `steer_center_us` | `config.local.yaml` | §2.1 / §2.3 | midpoint within 40 µs of 1500 |
| `steer_min_us` | `config.local.yaml` | §2.2, backed off 50 µs from bind | linkage does not hit its stop |
| `steer_max_us` | `config.local.yaml` | §2.2, backed off 50 µs from bind | linkage does not hit its stop |
| `steer_max_deg` | `config.local.yaml` | §2.5, protractor, **wheel** not horn | turning circle §6.5 |
| `steer_trim_us` | `config.local.yaml` | §5, iterated on the ground | verified in reverse §5.5 |
| `invert_left` / `invert_right` | `config.local.yaml` | [bringup.md](bringup.md) Phase 4.1 | |
| `encoder_invert_left` / `_right` | `config.local.yaml` | Phase 4, counts must rise on forward | |
| `max_rpm` ×4 | `calibration.yaml` | §4, bench then ground | 120–260 RPM, within 20 % |
| `deadband` ×4 | `calibration.yaml` | §4 | 0.05–0.30 |
| `ff_lut` ×4 | `calibration.yaml` | §4 | strictly monotonic |
| `on_ground` | `calibration.yaml` | §4.5 | both runs recorded in the log |

Then go to [tuning.md](tuning.md).
