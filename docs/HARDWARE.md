# Hardware: what we measured vs what we were told

The original hardware brief has been **demonstrably wrong at least twice** about this car's
powertrain, and both errors nearly sent us buying parts we didn't need. This document separates
what has actually been measured on this vehicle from what remains an unverified claim.

**Rule for this project: prefer a number in the MEASURED table over the same number anywhere else,
including the original brief, the datasheets, and comments in older code.**

---

## MEASURED — trust these

All verified on the physical car, 2026-08-16.

### Powertrain

| Value | Measured | How |
|---|---|---|
| Regulator output | **12 V** | Trimpot setting, confirmed by owner |
| Voltage at motors | **~10.2 V** | Inferred from no-load RPM (see below) |
| **No-load top speed** | **307 RPM right / 294 RPM left** | Encoder count, wheels up, 100% duty |
| **No-load ground speed** | **~1.05 m/s** | 307 RPM × 0.204 m circumference |
| Max duty | **1.00 usable** | 45 s at full throttle, `throttled=0x0`, SoC 47 °C |
| Motor mounting | **mirror-image — right side inverted** | Motors fought each other until `INVERT_RIGHT=True` |

**The car performs as its voltage predicts.** 12 V minus the L298N drop gives ~10.2–10.6 V, which
implies 307–318 RPM on a 12 V/360 RPM motor. We measured 307. **Agreement within 3.5%, so there is
no hidden loss and nothing is limiting this car** — a conclusion that took two wrong diagnoses to
reach.

This also *corroborates* the claimed ~1.4 V Darlington drop: working backwards from 307 RPM gives a
total loss of ~1.8 V, and the remainder is plausibly bearing and gearbox friction.

### Encoders

| Value | Measured |
|---|---|
| **Rising edges per output revolution, one channel** | **390** |
| Method | 3905 edges over 10 hand-turned revolutions |
| Both encoders functional | **Yes** — left 294 RPM, right 307 RPM under power |
| Pins | Left A/B = GPIO23/24, Right A/B = GPIO27/22 |

The brief says 660. We measure 390 counting rising edges on channel A. 660 is not 390, nor 2× nor
4× it, so the discrepancy is **unexplained** — treat 660 as wrong until someone shows otherwise.

### Steering — every default was wrong

| | Brief | **Measured** |
|---|---|---|
| Servo centre | 1500 µs | **1950 µs** |
| Full left | 1200 µs | **1550 µs** |
| Full right | 1800 µs | **2400 µs** |
| Steering lock | ±24° | **~±35°** |
| Travel symmetry | symmetric | **asymmetric: 400 µs left, 450 µs right** |

The horn sits roughly 40° round its spline, and the linkage is not symmetric — the same wheel angle
costs a different pulse width each way, which is why `VehicleConfig` carries a span per side.

### Camera

| Value | Measured |
|---|---|
| Sensor | IMX219, confirmed |
| Full-FOV mode | 1640×1232 binned, **41.85 fps ceiling** at 10-bit (83.7 at 8-bit) |
| Running at | **40 fps**, 640×480, ~19.8 KB/frame, **6.6 Mbps** |
| CPU cost | 53% of one core (of four) at 40 fps; 32% at 20 fps |
| **The crop trap is real** | A bare 640×480 request lands on a **1280×960 crop** — confirmed in the mode table and libcamera logs |

### Network

| Value | Measured |
|---|---|
| Signal | **−71 dBm**, 2.4 GHz only (Zero 2 W has no 5 GHz radio) |
| **ICMP packet loss** | **4–8%** |
| ICMP RTT | min 3.8 / avg 15.4 / **max 135 ms**, stddev 21.9 |
| Our control RTT | **p50 13–19 ms** after the command-driven telemetry fix |
| WiFi power save | **already off** — not a tuning opportunity |

**The radio is the weakest part of the whole system.** One packet in twenty is lost. The control
protocol survives only because it retransmits full state 50×/s and coasts on a watchdog.

### System

- `pigpio` DMA PWM works; `snd_bcm2835` **is not loaded**, so the documented audio/PWM contention
  never arose.
- **`dtparam=i2c_arm=on` is NOT set** — `/dev/i2c-1` does not exist. Any IMU or INA219 needs one
  `config.txt` line and a reboot.
- SPI not enabled either.
- The **CSI camera ribbon is intermittent**: libcamera reported `Camera frontend has timed out` after
  7.5 minutes of clean operation, and a reboot cleared it. Reseat the connector.

---

## WRONG — disproven on this car

| Claim | Reality |
|---|---|
| "Pololu … set to **9 V** output" | It is set to **12 V** |
| "Real top speed is **150–200 RPM**" | **307 RPM** no-load — off by ~1.7× |
| "Encoder resolution **660** cpr" | **390** rising edges/rev on one channel |
| "Servo centre 1500, travel 1200–1800" | 1950, travel 1550–2400 |
| "Steering lock ±24°" | ~±35° |

The 150–200 RPM figure is the dangerous one: it produced a confident "you are current-limited, buy a
LiPo" diagnosis that was entirely wrong. Nothing was wrong with the car.

---

## UNVERIFIED — plausible, but nobody has checked

Flagged because several sit underneath numbers we now quote.

| Claim | Why it matters | How to check |
|---|---|---|
| **Wheel diameter 0.065 m** | **Every m/s figure depends on it.** RPM is measured; ground speed is not | `encoder_probe.py --metres 5` — push the car a taped 5 m; edges/m plus edges/rev gives true circumference |
| **Loaded top speed** | We only measured **wheels up**. Real-world speed on the floor is still unknown | Same distance run, driven instead of pushed |
| Regulator sustains ~1.5 A for both motors | Quoted in `vehicle.py` as the reason for the slew limiter | INA219 on I2C |
| ~1.4 V L298N drop | Partially corroborated (see above), never directly measured | Multimeter across the bridge under load |
| Two stalled motors ≈ 7 W into the L298N | Justifies not holding a stall | Thermometer, or just don't |
| Wheelbase 0.200 m, track 0.150 m | Only affects turn-radius arithmetic | Tape measure |
| Encoder low-speed quantisation "±45% at 100 Hz" | Derived from the wrong 660 cpr, so the figure is wrong even if the concern is real | Recompute from 390 |
| Dead reckoning drifts 5–15% over a 5 m square | Never attempted | Needs odometry first |
| GPIO0–8 default to pull-up at boot | Standard BCM283x behaviour and the reason pull-downs are mandatory — but not cleanly observed here, since the pins were already configured when we looked | Read at boot before anything configures them |

---

## Still outstanding, physically

**The external 10 kΩ pull-downs on ENA/ENB/IN1–IN4 are still not fitted.** This is the one open
hardware item, and it matters more now that the car auto-starts its control process at power-on:
boot happens without anyone necessarily watching.
