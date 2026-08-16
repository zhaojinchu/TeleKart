# TeleKart v2 — Power

The power system is the binding constraint on this whole vehicle. Almost every "the code is
broken" symptom you will chase during bring-up — motors that stop mid-corner, a Pi that
reboots when you turn the wheel, encoders that count when nothing is moving — is a power
problem wearing a software costume.

Read this before you connect the battery to anything.

**Companion documents:** [wiring.md](wiring.md) for the physical build ·
[pi-setup.md](pi-setup.md) for `vcgencmd` and systemd · [bringup.md](bringup.md) for the
loaded measurements that confirm these numbers on your car.

---

## 1. Topology

Two power domains, deliberately separate, meeting only at the star ground point.

```
  MOTOR DOMAIN                                    PI DOMAIN
  ────────────                                    ─────────

  7.2 V NiMH pack                                 USB powerbank
        │                                          ≥2 A, real 2 A
   5 A blade fuse    ← as close to the pack             │
        │              terminal as possible      ≤300 mm, 20–22 AWG
   master switch     ← ≥10 A DC rated                   │
        │              (layer 4 E-stop)          Pi micro-USB PWR IN
  Pololu S18V20ALV                                      │
   set to 9.0 V                                   Pi 5 V rail ──┬── SoC + WiFi + camera
        │                                                       │    400–600 mA
   L298N +12V (VS)                                              └── HS-311 servo
        │                                                            7 mA / 180 mA / 800 mA
   two GA37-520 motors                            Pi 3.3 V rail ──── two hall encoders
   1.5 A COMBINED, hard ceiling
```

The motor pack never touches the Pi's 5 V. The powerbank never touches the L298N. The
L298N's own `+5V` terminal is left unconnected. See [wiring.md §8](wiring.md).

---

## 2. Setting the Pololu S18V20ALV — do this first, and do it with no load

The regulator's output voltage is set by a trimmer potentiometer. It arrives set to *some*
value in its range, and you must not assume which.

### The procedure

1. **Connect nothing to VOUT.** Not the L298N. Not a resistor. Not a motor. Nothing but the
   DMM's voltage probes. This is the whole point of the procedure.
2. Wire the pack to VIN/GND **through the fuse and the master switch**, so you already have a
   kill available.
3. DMM on DC volts, black probe on the regulator's GND, red probe on VOUT.
4. Switch on. Read what the board arrived at.
5. Turn the trim pot **a quarter turn at a time**, watching the meter after each. Note which
   direction raises the output — it varies, and guessing costs you a regulator.
6. Stop at **9.0 V ± 0.1 V**.
7. Switch off. **Wait for VOUT to decay to 0 V** — the output capacitor holds charge for
   several seconds with no load. Only then connect the L298N.

### Why no load

A trim pot on a switching regulator moves the output quickly and non-linearly. It is easy to
blow through 9 V to 12 V — or, if the pot has a wiper discontinuity, momentarily much further
— on a single careless quarter turn.

With nothing connected, the worst case of a slipped screwdriver is an alarming number on a
meter. With the L298N and two motors connected, the worst case is an untested bridge seeing a
voltage transient while you are looking at the meter instead of the car.

**Use a plastic or ceramic trim tool if you have one.** A steel screwdriver blade bridging the
pot body to an adjacent pad is a well-trodden path to a dead regulator.

### The enable pad

If your board revision has an `EN` / `SHDN` pad, **leave it exactly as shipped for bring-up.**
Pololu boards in this family enable by default with the pin unconnected, but the polarity of
the pad is not consistent across the S18V20 range. Check the product page for your exact part
before you wire a switch to it. The master switch on the pack side is the kill you want
anyway, and it works whether or not the regulator cooperates.

> **Re-check under load.** The no-load setting is not the loaded voltage. [bringup.md](bringup.md)
> Phase 4 gates on the output holding **≥ 8.5 V** with both motors at 40 % duty. If it sags
> below that, the problem is the pack or the input wiring, not the trim pot — do not "fix" it
> by turning the pot up.

---

## 3. Why 9 V and not 12 V for bring-up

The motors are 12 V parts and the regulator will happily do 12 V. Start at 9 V anyway, for
four independent reasons.

### 3.1 The pack current is 33 % lower for the same output current

Boost conversion trades voltage for current. At roughly 85 % efficiency:

| Output | Output power at 1.5 A | Input power | Pack current @ 7.2 V | Pack current @ 6.5 V (sagged) |
|--------|----------------------|-------------|----------------------|-------------------------------|
| **9 V**  | 13.5 W | 15.9 W | **2.21 A** | **2.44 A** |
| 12 V | 18.0 W | 21.2 W | 2.94 A | 3.26 A |

Every one of those amps comes out of a NiMH pack with real internal resistance, and the sag it
causes lowers the input voltage, which raises the input current further. At 9 V the boost
ratio is 1.25×; at 12 V it is 1.67×. The regulator, the pack and the wiring all have an
easier day at 9 V.

### 3.2 The bridge drop does not scale, so the motors see proportionally more

The L298 is a Darlington bridge. It drops roughly **2 V** across its output stage more or less
regardless of supply voltage:

| VS | Motor sees | vs. 9 V |
|----|-----------|---------|
| 9 V  | ~7 V  | — |
| 12 V | ~10 V | +43 % |

A 43 % increase in motor voltage is a 43 % increase in no-load speed and a proportional
increase in current at any given load. You would be spending the entire 1.5 A budget to go
faster on a car whose control loop you have not yet closed.

### 3.3 The bridge dissipation is already at its limit

Bridge loss is `V_drop × I`, which is independent of VS:

```
2 V × 1.5 A = 3 W
```

into a stock heatsink rated for **2–3 W**. You are *at* the ceiling at full combined duty by
design — this is precisely why the `duty_sum_max` parameter exists (default 1.40, see
[`params.py`](../packages/telekart_protocol/telekart_protocol/params.py)). Raising VS to 12 V
does not raise this number, but it does make it far easier to reach 1.5 A, so you spend more
of your time there.

### 3.4 A runaway at half speed is survivable

The first time the motors turn under software control you do not yet trust the software. The
wheels are off the ground, but a wheel that liberates its set screw at 200 RPM leaves the hub
at speed. Start slow.

**When to move to 12 V:** after [bringup.md](bringup.md) Phase 10 passes, if you want more
speed and you have measured the actual current draw. Re-run the drive auto-calibration
afterwards — `max_rpm` changes, and everything downstream scales off it automatically
([INTERFACES.md §6](INTERFACES.md)). Nothing in the codebase hardcodes a top speed.

---

## 4. The 1.5 A combined budget

This is the number the entire drivetrain design is organised around.

### The arithmetic

```
Regulator sustained output              1.5 A   at 9 V  =  13.5 W
Bridge conduction loss (2 V × 1.5 A)   −3.0 W
                                       ───────
Delivered to both motors                10.5 W  at ~7 V

Straight-line, both motors equal:       0.75 A each
```

**It is the *simultaneous* demand that trips the regulator, not either motor alone.** One
motor at 1.2 A is fine. Two motors at 0.9 A each is not. That is why the parameter is a
ceiling on the *sum* of the duties rather than a per-motor limit:

| `duty_sum_max` = 1.40 means | left | right |
|---|---|---|
| straight line | 0.70 | 0.70 |
| moderate turn | 0.85 | 0.55 |
| full lock (electronic differential at ±16.7 %) | 0.82 | 0.58 |

The per-motor clamp `max_duty` (default 0.85) sits on top of that, so no single motor can take
the whole budget even when the other is stopped.

### Why acceleration is rate-limited, not just capped

A step from rest to 150 RPM asks the motor for locked-rotor current until it spins up. With
`MOTOR_WINDING_OHMS` at 3 Ω ([`constants.py`](../pi/telekart/constants.py)) and ~7 V at the
terminals that is roughly **2.3 A *per motor*** — over 4.5 A combined, three times the budget,
for as long as the rotor takes to move.

`accel_rpm_per_s` (default 250 RPM/s) rate-limits the **RPM target**, not the duty. Reaching
150 RPM then takes 600 ms and the current never leaves the running band. The parameter's own
description says it plainly: *"Inrush during throttle steps is what trips the regulator."*

Note this is a limit on the *target*, deliberately. Rate-limiting the duty instead would fight
the PID and make the loop untunable.

### Measuring it on your car

Do not trust the arithmetic; it is a starting point. Put a measurement in the pack positive
lead:

- **Inline RC watt meter** (the generic "power analyser" with an XT60 on each end) — the right
  tool. Shows volts, amps, peak amps, watt-hours, all live, for the price of a takeaway.
- **Clamp meter** with a DC range — non-invasive, but most cheap ones are AC-only and the DC
  ones have poor resolution below an amp.
- **DMM in series on the 10 A jack** — works, but you must remember to move the lead back
  afterwards, and the 10 A jack is usually unfused.

Record the peak during a full-throttle step from rest. That number, not the datasheet, is your
budget.

### Signs you are over budget

| Symptom | What it actually is |
|---------|--------------------|
| Both encoders read zero **simultaneously** while commanded | The regulator tripping. The firmware calls this `Fault.BROWNOUT` (bit 4) and gives it a **distinct fault code from a stall** so the two are never confused during diagnosis. |
| One encoder reads zero while commanded | A real mechanical stall. `Fault.STALL_L` / `STALL_R`. |
| Motors stutter at a steady 2–5 Hz under load | Regulator hiccup-mode restart, cycling. |
| It only happens when you accelerate, never at steady speed | Inrush. Lower `accel_rpm_per_s`. |
| It only happens in a hard turn | Combined duty. Lower `duty_sum_max`. |

---

## 5. L298N `5V-EN` jumper policy

Settled: **jumper IN at 9 V, and the `+5V` screw terminal is left unconnected.**

### At VS = 9 V (this build)

With the jumper in, the module's onboard 78M05 generates 5 V from VS to power the L298's own
logic supply (VSS). That load is tens of milliamps:

```
(9 V − 5 V) × ~36 mA = 0.14 W        ← nothing, in a TO-220
```

So it self-powers with no drama and one fewer wire.

### Why the `+5V` terminal stays unconnected

With the jumper in, that terminal becomes an **output** capable of sourcing several hundred
milliamps from the motor battery. Connecting it to the Pi's 5 V rail means:

- Two 5 V sources fighting each other, with the winner decided by whichever sags less.
- Current pushed *backwards* into the powerbank's output stage.
- The Pi browning out when the motor pack switch is thrown, because you have now made the Pi
  depend on the motor battery you were about to kill.

That last point is the serious one: the master switch is layer 4 of the panic-stop chain
([INTERFACES.md §8](INTERFACES.md)) and it must never take the Pi down with it.

### If you back-fed the Pi anyway

For completeness, so you understand why not: at 400 mA of Pi draw the 78M05 dissipates
`(9 − 5) × 0.4 = 1.6 W` in a bare TO-220 with no heatsink. Junction temperature climbs 60 °C+
over ambient, the part enters thermal shutdown, recovers, shuts down again. The Pi sees a 5 V
rail oscillating at a few hertz. The SD card does not enjoy this.

### Above 12 V

**Pull the jumper** and feed 5 V into the `+5V` terminal from a separate source. The 78M05's
dissipation scales with `(Vin − 5) × I`, and the module's guidance is to stop self-powering
above 12 V. This becomes relevant if you take the 3S LiPo upgrade path in §9.

---

## 6. The servo on the Pi's 5 V rail

This is a deliberate choice by the builder, made with the numbers in front of them. Here they
are, honestly.

### The numbers

| Load | Current |
|------|---------|
| HS-311, powered and holding, no load, pulse train present | **~7 mA** |
| HS-311, running under light linkage load | **~180 mA** |
| HS-311, **stalled** (bound linkage, or commanded past its endpoint) | **700–800 mA** |
| Pi Zero 2 W, camera streaming + WiFi transmitting | **400–600 mA** |
| **Worst case sum** | **~1.4 A** |

That is real. A stalled servo plus a busy Pi is 1.4 A out of a powerbank, through a USB cable,
into a board whose undervoltage detector trips at about **4.63 V**.

### Why it is nonetheless the right call for this build

The alternative is a second switching converter, a second ground, and a second thing to get
wrong, on a car that does not yet run. The mitigations below reduce the realistic worst case
to well under 1 A, and the upgrade path in §9 is a twenty-minute job the day you want it.

### Mitigation 1 — bulk capacitance at the servo connector, with honest limits

470–1000 µF electrolytic (≥10 V) plus a 100 nF ceramic, across Pi physical pins 4 and 6, at
the **servo connector end** of the cable.

What it actually does, by `ΔV = I·Δt / C`, for a 700 mA step into 1000 µF:

| Duration of the step | Sag the cap allows |
|----------------------|--------------------|
| 100 µs | 0.07 V — **the cap handles this entirely** |
| 1 ms   | 0.70 V — the cap is losing |
| 5 ms   | 3.5 V — the cap is irrelevant |

**So: the capacitor covers the edge, not the event.** It keeps the fast `di/dt` off the cable
and out of the Pi's rail, which is genuinely worth having — cable inductance plus a sharp
current step is how you get ringing on a supply. It does **not** hold up a sustained stall,
and anyone who tells you 1000 µF "fixes" servo brownout has not done that division.

Sustained stalls are handled by not having them. Which is mitigations 2 and 3.

### Mitigation 2 — slew limiting

`steer_rate_us_per_s`, default 2000 µs/s. A full lock-to-lock sweep of 600 µs then takes
300 ms.

A servo's current draw scales with how fast you ask it to move. A slew limit *is* a current
limit. Under slew limiting the servo draws its **running** figure (~180 mA), not its stall
figure, because it is never asked for more torque than it can produce at that rate.

Note the division of responsibility from [INTERFACES.md §9](INTERFACES.md): the app owns
*feel* and the firmware owns *protection*, and **the app's rate limit must be tighter than the
firmware's** so that the firmware's limiter never engages during normal driving. When it does
engage, the firmware sets `TelemetryFlags.LIMITER_ACTIVE` (bit 0) so you can see it.

### Mitigation 3 — relax when disarmed

`servo_relax_when_disarmed`, default `true`. On disarm the firmware calls
`set_servo_pulse(pin, 0)`, which **stops the pulse train entirely**. The servo goes limp and
its holding current falls to essentially zero.

This matters more than it sounds: the car spends most of its life sitting on a bench,
disarmed, with the operator fiddling. A servo holding position against a slightly
mis-centred linkage for twenty minutes is 180 mA of nothing useful and a warm servo.

The interface makes this explicit — `pulse_us == 0` stops the pulse train, "which is the
correct disarmed state here, because the servo shares the Pi's 5 V rail with the SoC"
([INTERFACES.md §2](INTERFACES.md)).

### Mitigation 4 — a real ≥2 A powerbank

Not a 2 A label on a 1 A bank. If the bank has two ports, the "2.4 A" one is usually shared
across both — use one port only.

### Mitigation 5 — short, thick cable. This is the highest-leverage item on the list.

USB cable resistance, round trip (out and back), at 1.2 A:

| Cable | Ω/m per conductor | Round-trip R | Drop at 1.2 A | Pi sees (from 5.10 V) |
|-------|-------------------|--------------|---------------|------------------------|
| 1 m of 28 AWG (a phone charging cable) | 0.21 | **0.42 Ω** | **0.50 V** | **4.60 V — under the 4.63 V threshold** |
| 0.3 m of 20 AWG | 0.033 | **0.020 Ω** | **0.024 V** | **5.08 V** |

A twenty-fold difference, for the cost of buying the right cable. Most "USB power problems"
on a Pi are this and nothing else.

Two further points:

- **Connector contact resistance is comparable to the wire.** A worn micro-USB socket can add
  100 mΩ on its own. If a cable ever worked and now does not, suspect the socket.
- You *can* bypass the socket by feeding 5 V directly into header pins 2 and 6. This removes
  the connector resistance entirely — and also removes the board's input protection. Do it
  only with a supply you have already measured and trust.

### Mitigation 6 — the `vcgencmd get_throttled` gate

The firmware polls `vcgencmd get_throttled` and publishes the **raw 32-bit bitmask** in every
telemetry packet (field `throttled`, byte offset 82 — see [protocol.md](protocol.md)).

| Bit | Meaning |
|-----|---------|
| 0  | Under-voltage detected **now** |
| 1  | ARM frequency capped now |
| 2  | Currently throttled |
| 3  | Soft temperature limit active |
| 16 | Under-voltage **has occurred** since boot (sticky) |
| 17 | ARM frequency capping has occurred (sticky) |
| 18 | Throttling has occurred (sticky) |
| 19 | Soft temperature limit has occurred (sticky) |

It maps them to `Fault.PI_UNDERVOLTAGE` (bit 12) and `Fault.PI_THROTTLED` (bit 13).

**Be precise about what that does:** neither of those is in `CRITICAL_FAULTS`
(which is `CRITICAL_BATTERY | GPIO_ERROR | OVERTEMP | ESTOP_LATCHED`, `0x8340`), so at runtime
they **warn** — they light the HUD, they do not force a disarm. Killing the drive because the
Pi dipped once mid-corner would be its own hazard.

On the bench the gate is stricter, and it is yours to enforce: **[bringup.md](bringup.md)
Phase 1 does not pass with any bit set.** A car that starts its day with sticky bit 16 already
lit has an unresolved supply problem, and every subsequent measurement is suspect.

```bash
vcgencmd get_throttled          # 0x0 is the only acceptable answer at Phase 1
```

---

## 7. Fuse and master switch

### Fuse

**5 A automotive blade fuse, in the pack positive lead, as close to the pack terminal as
physically possible.**

| Choice | Why |
|--------|-----|
| 5 A | Worst sustained draw is 2.44 A (§3.1). 5 A clears a genuine short in milliseconds while ignoring stall transients. |
| Not 3 A | Nuisance-blows on a two-motor stall, and a fuse you have learned to distrust is worse than none. |
| Not 10 A | 18–20 AWG wire will happily glow for a long time at 10 A. The fuse protects the *wire*, not the load. |
| Blade, not glass | Glass fuses shatter and their holders are unreliable under vibration. |

**Position matters.** The wire between the pack terminal and the fuse is, by definition,
unprotected. Make it as short as you physically can — ideally the fuse holder is crimped
directly onto the pack lead.

### Master switch

**Rated ≥10 A DC, in the pack positive lead, between the fuse and the regulator VIN.**

The DC rating is the part people get wrong. A rocker switch marked "10 A 250 VAC" may be good
for only a couple of amps DC, because a DC arc does not self-extinguish at a zero crossing the
way an AC one does. Use an automotive rocker, or an RC "loop key" (an XT60 pair with a
shorting plug), both of which are specified for DC.

### This is your real E-stop

The master switch is **layer 4 of the four-layer panic-stop chain** and it is the only layer
that still works when everything else is on fire ([INTERFACES.md §8](INTERFACES.md)).

Understand why it has to be on the *motor* battery and not the Pi:

> `pigpiod` retains GPIO state after its client dies. A segfault or a `kill -9` at 80 % duty
> leaves the motors running **indefinitely**.

Killing the Pi does not stop the car. Killing the Pi *removes the only thing that could have
stopped the car*. The battery switch is the guaranteed kill.

Mount it so you can reach it standing next to the car without moving your feet, and where no
wheel, gear or wire can ever reach it.

---

## 8. Powerbank behaviour

### Auto-shutoff below ~100 mA

Many USB powerbanks cut their output when the load drops below roughly 50–100 mA, because they
are designed to stop when a phone finishes charging. There is no standard and no label.

The danger window is narrow but real:

| State | Pi Zero 2 W draw | Risk |
|-------|------------------|------|
| Booted, headless, idle at a login prompt, WiFi associated | **100–150 mA** | **Uncomfortably close to a 100 mA cutoff** |
| Firmware running, WiFi transmitting | 250–400 mA | Safe |
| Firmware + camera streaming | 400–600 mA | Safe |

So the car is safe while you are driving it and vulnerable while it sits waiting for you to
open the app — which is precisely the moment a mysterious shutdown is most confusing.

### Test yours

```bash
sudo systemctl stop telekart-control telekart-video
# leave it for 15 minutes at the login prompt, then check it is still alive
uptime
```

If it dropped, you have your answer.

### Fixes, in order of preference

1. **Buy a different bank.** Look for "low current mode", "trickle charge mode", or "small
   device mode" in the description. Banks that support **pass-through charging** are usually
   the ones that behave, because they are designed to stay awake.
2. **Keep the load up.** In practice the firmware and WiFi keep you above 250 mA whenever the
   car is meant to be usable. Starting the services at boot largely removes the window.
3. **Add a keep-alive load.** A 47 Ω resistor across the 5 V rail draws `5 / 47 = 106 mA` and
   dissipates `5² / 47 = 0.53 W`. Use a **1 W or 2 W** part, mount it where it can breathe,
   and accept that you are burning half a watt and some runtime to work around a design
   decision in a device you did not design. It works.

### Capacity

At an average 500 mA the Pi draws 2.5 Wh per hour. A 10 000 mAh bank nominally holds 37 Wh but
delivers perhaps 26 Wh after boost-conversion losses — call it eight to ten hours of Pi. The
motor pack will be flat long before that.

---

## 9. Upgrade paths

Both of these are worth doing eventually. Neither should be attempted before
[bringup.md](bringup.md) Phase 10 passes, because you cannot debug a change to a system you
have never seen work.

### 9.1 Move the servo to its own UBEC

**What:** a 5 V / 3 A switching BEC (Castle CC BEC, Hobbywing UBEC, etc.) fed from the 7.2 V
motor pack, driving **only** the servo.

**Why:** it removes the servo from the Pi's rail entirely. The 800 mA stall now comes from a
NiMH pack through a switcher instead of from a powerbank through a USB cable. As a bonus,
`get_throttled` becomes a pure Pi-health signal instead of a servo-activity signal, which
makes it far more useful.

**Wiring, and this is the part to get right:**

```
  pack (+) ─── fuse ─── switch ──┬── regulator VIN        (existing)
                                 └── UBEC VIN             (new)

  pack (−) ────────────────────── ★ star point ── UBEC input GND

  UBEC 5 V out ══╗  twisted pair, short and fat
  UBEC GND    ══╝  ─── servo red / servo black

                 └── ONE thin wire ─── any Pi header GND pin
                     (signal reference only, µA)

  Pi GPIO18 (phys 12) ─────────────── servo yellow
```

- The servo's 3 A return path is the short twisted pair between UBEC and servo. It never
  touches the Pi.
- A single thin wire ties the UBEC's output ground to the Pi's ground so the servo's pulse
  input has a reference. It carries microamps. This is the same star-ground discipline from
  [wiring.md §5](wiring.md), applied one level down.
- **Do not also connect the UBEC output to the Pi's 5 V.** One source per rail. Ever.

**After:** you can relax `steer_rate_us_per_s` if you want quicker steering, and
`servo_relax_when_disarmed` becomes a convenience rather than a necessity. Re-check the
straight-line trim in [calibration.md](calibration.md) — a servo held more firmly sits
slightly differently.

### 9.2 MOSFET bridge + 3S LiPo

**What:** replace the L298N with a MOSFET H-bridge (BTS7960 / IBT-2, Cytron MDD10A, or
similar) and feed it directly from a 3S LiPo, deleting the boost regulator entirely.

**What you recover:**

| | L298N @ 9 V | MOSFET bridge @ 11.1 V |
|---|---|---|
| Bridge drop at 1.5 A | ~2 V | ~0.15 V |
| Bridge heat at 1.5 A | 3.0 W | ~0.2 W |
| Motor terminal voltage | ~7 V | ~11 V (**+57 %**) |
| Current ceiling | 1.5 A combined (regulator) | 10–30 A (module dependent) |
| `duty_sum_max` | 1.40 | 2.00 — the budget stops existing |
| Boost regulator | required | **deleted** |

The 2 V you get back is not a rounding error: at 9 V VS it is 28 % of the voltage that
actually reaches the motor.

**What it costs you:**

- A LiPo needs a balance charger, a low-voltage cutoff, and somewhere fireproof to live. NiMH
  is used for bring-up precisely because it is forgiving — a LiPo shorted by a bridge you
  wired at midnight is a fire, not an inconvenience.
- Set `low_battery_v = 10.5` and `critical_battery_v = 9.9` for 3S (3.5 V and 3.3 V per cell).
  **Note these do nothing without battery-sensing hardware** — the telemetry `pack_mv` field
  reads 0 when none is fitted, and the parameter descriptions say so.
- Pull the L298N's `5V-EN` jumper if you keep the module around for anything (§5).
- You still need a 5 V source for the Pi and the servo: keep the powerbank, or fit the UBEC
  from §9.1 fed from the LiPo.

**What changes in software:** the BTS7960 uses `RPWM` / `LPWM` / `R_EN` / `L_EN` instead of
`EN` / `IN1` / `IN2`, so `MotorPins` and `MotorPair` need a variant. That is one driver file —
which is exactly what the `GpioBackend` abstraction in [INTERFACES.md §2](INTERFACES.md)
exists for.

**What does not change: anything downstream of `max_rpm`.** Re-run the drive auto-calibration
from [calibration.md](calibration.md), and `v_max_mm_s` in every telemetry packet updates, and
the desktop speedometer, the mixer and the speed-sensitive steering all follow. Nothing in the
codebase hardcodes a top speed, and this upgrade is the specific reason that rule exists.

---

## 10. Quick reference

| Item | Value |
|------|-------|
| Motor pack | 7.2 V NiMH (6 cells) |
| Fuse | 5 A blade, pack positive, at the pack terminal |
| Master switch | ≥10 A **DC** rated, pack positive, after the fuse |
| Regulator | Pololu S18V20ALV, set to **9.0 V ± 0.1 V, no load** |
| Regulator loaded minimum | ≥ 8.5 V at 40 % combined duty (Phase 4 gate) |
| Combined motor current ceiling | **1.5 A** |
| Pack current at that ceiling | 2.2–2.4 A |
| Bridge dissipation at that ceiling | ~3 W into a 2–3 W heatsink |
| `duty_sum_max` | 1.40 |
| `max_duty` | 0.85 |
| L298N `5V-EN` jumper | **IN** (at 9 V); `+5V` terminal **unconnected** |
| L298N `ENA`/`ENB` jumpers | **REMOVED** |
| Pi supply | USB powerbank, ≥2 A real, **≤300 mm of 20–22 AWG** |
| Pi undervoltage threshold | ~4.63 V |
| Servo bulk cap | 470–1000 µF ≥10 V + 100 nF, at the connector |
| L298N bulk cap | 470–1000 µF ≥16 V + 100 nF, at the screw terminals |
| Acceptable `get_throttled` at Phase 1 | `0x0`, nothing else |
