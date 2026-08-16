# TeleKart v2 — Build Sheet

Everything you physically connect, in the order you should connect it, with a verification
step after each subsystem. Work through it top to bottom. Do not skip a VERIFY block because
the wiring "obviously" worked — every one of them exists because that failure was diagnosed
the hard way at least once.

**Companion documents:** [power.md](power.md) for the battery, regulator and current budget ·
[pi-setup.md](pi-setup.md) for the OS and `config.txt` · [bringup.md](bringup.md) for the
bench test sequence with pass/fail gates · [calibration.md](calibration.md) for the numbers
you measure afterwards.

---

## 0. Rules that apply to the whole build

1. **The motor battery is disconnected for every step in this document except where a VERIFY
   block explicitly says otherwise.** The Pi runs from its powerbank; that is enough to test
   every signal path.
2. **Wheels stay off the ground until [bringup.md](bringup.md) Phase 10.** Blocks under the
   chassis, at least 20 mm of clearance under each tyre.
3. **Colour is scoped per bundle, not globally.** The bundles are physically separate and
   routed differently, so reusing yellow in two of them is fine and keeps each bundle
   readable. Two rules *are* global: black is always ground, and red is always the highest
   voltage in that bundle. Nothing else is.
4. **Strand-count matters more than gauge marketing.** Anything that moves with the chassis
   (motor leads, encoder harness) must be stranded. Solid-core breaks at the solder joint
   after about a hundred corners, and the break is inside the insulation where you cannot see
   it.
5. **Label both ends of every bundle before you route it.** A flag of masking tape with
   `ENC-L` on it costs ten seconds and saves an hour.

---

## 1. System overview

```
                       ┌──────────────────┐
   7.2 V NiMH pack ────┤ 5 A blade fuse   ├──── master switch ──┐
        (─)  │         └──────────────────┘   (≥10 A DC)        │
             │                                                  │ (+)
             │                                          ┌───────┴────────┐
             │                                          │  Pololu        │
             ├──────────────────────────────────────────┤  S18V20ALV     │
             │                                     GND  │  VIN → VOUT    │
             │                                          │  set to 9.0 V  │
             │                                          └───────┬────────┘
             │                                                  │ 9 V
             │                              ┌───────────────────┴──────────────┐
             │                              │  L298N module                    │
             └──────────────────────────────┤  GND        +12V (VS)            │
                        ★ STAR GROUND POINT │                                  │
                              │             │  5V-EN jumper: IN                │
                  ┌───────────┘             │  +5V terminal: NOT CONNECTED     │
                  │                         │  ENA/ENB jumpers: REMOVED        │
                  │  (logic return only,    │                                  │
                  │   one dedicated wire)   │  OUT1 OUT2      OUT3 OUT4        │
                  │                         └───┬────┬────────┬────┬───────────┘
                  │                             │    │        │    │
                  │                         ┌───┴────┴───┐┌───┴────┴───┐
                  │                         │ LEFT motor ││RIGHT motor │
                  │                         │ GA37-520   ││ GA37-520   │
                  │                         └─────┬──────┘└─────┬──────┘
                  │                               │ hall        │ hall
   ┌──────────────┴──────────────┐                │             │
   │  Raspberry Pi Zero 2 W      │                │             │
   │                             │  3V3 + A/B ────┘             │
   │  GPIO12 ENA   GPIO5  IN1    │  3V3 + A/B ──────────────────┘
   │  GPIO13 ENB   GPIO6  IN2    │
   │               GPIO20 IN3    │        ┌──────────────┐
   │               GPIO21 IN4    │  5V ───┤ HS-311 servo │
   │  GPIO18 servo               │  GND ──┤  + 470–1000  │
   │  GPIO23/24  left  encoder   │  sig ──┤    µF here   │
   │  GPIO27/22  right encoder   │        └──────────────┘
   │  GPIO25 LED   GPIO16 E-stop │
   │                             │        ┌──────────────┐
   │  CSI ───────────────────────┼────────┤ Cam Module 2 │
   │                             │        │  (IMX219)    │
   │  micro-USB PWR IN           │        └──────────────┘
   └──────────┬──────────────────┘
              │  short, thick (≤300 mm, 20–22 AWG)
      ┌───────┴────────┐
      │ USB powerbank  │   ≥2 A, separate from the motor pack
      │  ≥2 A          │   on purpose — see power.md
      └────────────────┘
```

Two power domains that meet **only at the star ground point** and nowhere else. The motor
pack never touches the Pi's 5 V, and the powerbank never touches the L298N.

---

## 2. Pi header pinout — the complete table

Physical pin numbers are for the standard 40-pin header, pin 1 nearest the SD card slot,
odd numbers on the row closest to the board edge.

### 2.1 L298N logic — bundle A

| Signal | BCM  | Phys | Colour | L298N pin | Notes |
|--------|------|------|--------|-----------|-------|
| ENA    | 12   | 32   | white  | ENA       | Hardware PWM channel 0 |
| ENB    | 13   | 33   | yellow | ENB       | Hardware PWM channel 1 — **shares one clock divider with ENA** |
| IN1    | 5    | 29   | grey   | IN1       | Left motor direction A. **Idles HIGH without a pull-down** |
| IN2    | 6    | 31   | violet | IN2       | Left motor direction B. **Idles HIGH without a pull-down** |
| IN3    | 20   | 38   | blue   | IN3       | Right motor direction A. Also I2S PCM_DIN — see §7 |
| IN4    | 21   | 40   | green  | IN4       | Right motor direction B. Also I2S PCM_DOUT — see §7 |
| GND    | —    | 39   | black  | GND       | The single logic return. **Motor current must never flow here** |

All seven of these land inside the pins 29–40 block at the far end of the header, which is why
this pinout was chosen: bundle A is one seven-way ribbon off the end of the connector, with
nothing crossing it and nothing to route past the camera ribbon.

ENA and ENB are the two hardware PWM channels on BCM283x, and **they share a single clock
divider.** Writing GPIO12 at 1 kHz and then GPIO13 at 4 kHz corrupts channel 0. The firmware
makes that mistake unrepresentable by exposing only `set_pwm_pair()`
([INTERFACES.md §2](INTERFACES.md)); on the bench, remember it when poking pins with `pigs`.

### 2.2 Steering servo — bundle B

Colours here are fixed by Hitec, not chosen by you.

| Servo lead | Colour | Phys | BCM | Notes |
|------------|--------|------|-----|-------|
| Ground     | black  | 6    | —   | Adjacent to the 5 V pin, so the supply loop is short |
| +5 V       | red    | 4    | —   | Pi's 5 V rail. Deliberate choice — see [power.md](power.md) |
| Signal     | yellow | 12   | 18  | DMA-timed pulses, not hardware PWM |

Futaba leads are black/red/**white** and JR leads are brown/red/orange. If your lead is not
black/red/yellow, identify ground by continuity to the servo case screw and do not guess.

**GPIO18 cannot use hardware PWM here.** GPIO18's PWM alt-function is PWM**0** — the same
channel as GPIO12. Both hardware channels are already spoken for by ENA and ENB, so the servo
gets pigpio's DMA-timed pulse generator instead. This is not a compromise; DMA pulses on this
SoC are accurate to the sample period (5 µs, see [pi-setup.md](pi-setup.md)), which is finer
than an HS-311 can resolve.

The 470–1000 µF bulk capacitor goes **across physical pins 4 and 6, at the servo connector
end of the cable.** See §6.

### 2.3 Encoders — bundles C and D

Encoder VCC comes from the **Pi's 3.3 V rail**, and encoder ground returns to the **Pi's
ground**, never to the L298N. Read §5 before wiring these.

| Signal   | BCM | Phys | Colour | Motor connector pin |
|----------|-----|------|--------|---------------------|
| ENC_L_A  | 23  | 16   | yellow | C1 |
| ENC_L_B  | 24  | 18   | green  | C2 |
| L VCC    | —   | 17   | orange | VCC (3.3 V) |
| L GND    | —   | 14   | black  | GND |
| ENC_R_A  | 27  | 13   | blue   | C1 |
| ENC_R_B  | 22  | 15   | violet | C2 |
| R VCC    | —   | 1    | orange | VCC (3.3 V) |
| R GND    | —   | 9    | black  | GND |

Left and right use different signal colours on purpose. When you inevitably plug the wrong
connector into the wrong motor at 11 pm, a yellow wire going into the right-hand motor is
visible from across the bench; two identical harnesses are not.

Both 3.3 V pins on the header (physical 1 and 17) are the same rail; using both just shortens
the runs.

**Only channel A is edge-driven.** The firmware decodes x2 — `EITHER_EDGE` on channel A only,
with direction taken from the commanded H-bridge state — because reading channel B inside the
callback costs a pigpiod socket round-trip of 50–100 µs *per edge*
([INTERFACES.md §3](INTERFACES.md)). Channel B is still wired: it is polled at loop rate as a
disagreement check, and it is what you will want the day an encoder starts lying.

### 2.4 Panel — bundle F

| Signal      | BCM | Phys | Colour | Notes |
|-------------|-----|------|--------|-------|
| Status LED  | 25  | 22   | white  | Through 330 Ω to the LED anode |
| LED return  | —   | 20   | black  | Adjacent to pin 22 |
| E-stop      | 16  | 36   | grey   | See the polarity discussion below |
| E-stop rtn  | —   | 34   | black  | Adjacent to pin 36 |

**LED resistor:** 330 Ω gives (3.3 − 2.0) / 330 ≈ 4 mA for a red or green LED — bright on any
modern part and a quarter of the 16 mA per-pin limit. Do not fit a blue or white LED: their
forward voltage is 3.0–3.2 V and they will be nearly invisible on a 3.3 V rail.

**E-stop button polarity — fit a NORMALLY-OPEN button.** Wire it pin 36 → button → pin 34.
The firmware enables the internal pull-up and treats **LOW as pressed**
(`LocalControls` in [`app.py`](../pi/telekart/app.py), and the comment on `PIN_ESTOP_BUTTON` in
[`constants.py`](../pi/telekart/constants.py)):

| Condition          | Contact | Pin reads | Firmware sees |
|--------------------|---------|-----------|---------------|
| Normal             | open    | HIGH      | not pressed — OK |
| Button pressed     | closed  | LOW       | **E-STOP** |
| Wire broken        | open    | HIGH      | not pressed |
| Connector fell off | open    | HIGH      | not pressed |

> **Do not fit a normally-closed button.** This is worth the paragraph because NC is the
> textbook answer for an emergency stop, and here it is wired backwards. An NC button holds
> pin 36 at GND at rest, which this firmware reads as *pressed*: the car latches `ESTOP` on
> its first control tick and **can never be armed**, and *pressing* the button is what
> releases it. There is no inversion parameter — `estop_button` in the config is a pin number
> and nothing else, and there is no `estop_invert` in
> [`params.py`](../packages/telekart_protocol/telekart_protocol/params.py) to reach for.

**Be clear-eyed about what this buys you.** A normally-open button is fail-*silent*: a broken
wire or an unplugged connector reads exactly like "everything is fine". That is a deliberate
trade — with a pull-up and an optional button, the fail-safe polarity would mean a car with no
button fitted at all could never arm — but it means the panel button is a *convenience* stop,
not a fail-safe one.

**The battery master switch is the fail-safe E-stop on this vehicle**
([INTERFACES.md §8](INTERFACES.md), panic-stop layer 4, and [power.md §7](power.md)). It is
layer 4 precisely because it is the only one that cannot fail silent. Treat the panel button
as the thing you reach for first and the master switch as the thing that actually guarantees
the car stops.

### 2.5 Pins that must be left alone

| Phys | BCM | Why |
|------|-----|-----|
| 27, 28 | 0, 1 | ID_SD / ID_SC. HAT EEPROM probe at boot. Do not use. |
| 3, 5 | 2, 3 | I²C. Free today, but they are where an MPU-6050 lands when the heading-source hook in [INTERFACES.md §5](INTERFACES.md) gets used. Keep them clear. |
| 35 | 19 | PCM_FS. Unused by us, but it sits inside the I2S block with GPIO18/20/21 — see §7. |

Everything else on the header is unused and available.

---

## 3. The six 10 kΩ pull-down resistors

**These are not optional and software cannot replace them.**

### Why

On BCM283x, GPIO0–8 power up with an internal **pull-UP** enabled; GPIO9–27 power up with an
internal pull-DOWN. IN1 is GPIO5 and IN2 is GPIO6. Both therefore **idle HIGH from the
instant the Pi powers on until firmware runs** — through the bootloader, through the kernel,
through the ninety seconds it takes systemd to reach your service, and again for the whole of
every reboot.

With the L298N powered and IN1 = IN2 = HIGH, the left channel is *primed*. It only takes ENA
going high — a stray pigs command, a half-seated ribbon, the next boot with a different
default — to put the bridge into a hard brake on a coasting motor.

Worse, the internal pulls vanish entirely if the Pi is off or the ribbon is unplugged, leaving
six Darlington inputs floating next to a 9 V switching bridge.

### What to fit

| Resistor | From | To |
|----------|------|----|
| R1 | L298N ENA | L298N GND |
| R2 | L298N IN1 | L298N GND |
| R3 | L298N IN2 | L298N GND |
| R4 | L298N IN3 | L298N GND |
| R5 | L298N IN4 | L298N GND |
| R6 | L298N ENB | L298N GND |

All 10 kΩ, ¼ W, ±5% is fine.

### Why at the L298N end, not the Pi end

Because the failure you are guarding against includes *the ribbon being unplugged*. A
pull-down on the Pi side does nothing once the cable is out. Solder them directly across the
module's input header to its GND pin — on the underside of the board, or on a scrap of
stripboard piggybacked onto the header.

### Why 10 kΩ and not 100 kΩ

10 kΩ has to *win* against the Pi's internal 50–65 kΩ pull-up on GPIO5/6. The divider gives:

```
3.3 V × 10k / (10k + 50k) = 0.55 V
```

The L298's guaranteed input-low threshold is 1.5 V, so 0.55 V is an unambiguous LOW with 1 V
of margin. A 100 kΩ pull-down would give 3.3 × 100/150 = 2.2 V — a solid **HIGH**, i.e. it
does nothing at all. Meanwhile 10 kΩ loads a driving GPIO with only 330 µA, which is nothing.

### What this buys you

This is **layer 3 of the four-layer panic-stop chain** ([INTERFACES.md §8](INTERFACES.md)).
It is the only layer that covers a Pi reboot, a total loss of Pi power, and a disconnected
ribbon. Layers 1 and 2 are software and systemd; they cannot help when there is no software.

> **VERIFY — pull-downs.** Motor battery disconnected, L298N logic unpowered, ribbon
> unplugged from the Pi. Measure resistance from each of ENA, IN1, IN2, IN3, IN4, ENB to the
> L298N GND terminal. **Every one must read 10 kΩ ±5%.** A reading of ∞ means a dry joint; a
> reading of ~5 kΩ means you have bridged two of them.
>
> Then plug the ribbon in, power **only** the Pi, and before starting any firmware run:
> ```
> pinctrl get 5,6,12,13,20,21      # Bookworm
> raspi-gpio get 5,6,12,13,20,21   # older images
> ```
> Every pin must report `level=0`. If GPIO5 or GPIO6 reports `level=1`, your pull-down on that
> pin is not connected. **Stop and fix it before going further.**

---

## 4. Motor harness pinout

GA37-520 gearmotors in this class carry a six-way connector. The pin *order* is fixed; the
wire *colours* vary between suppliers and even between batches. **Trust the silkscreen on the
encoder PCB, never the colours.**

| Pin | Label | Function | Typical colour | Goes to |
|-----|-------|----------|----------------|---------|
| 1 | M1  | Motor winding + | red    | L298N OUT1 (left) / OUT3 (right) |
| 2 | GND | Encoder ground  | black  | **Pi GND** (phys 14 left / phys 9 right) |
| 3 | C1  | Hall channel A  | yellow | GPIO23 (left) / GPIO27 (right) |
| 4 | C2  | Hall channel B  | green  | GPIO24 (left) / GPIO22 (right) |
| 5 | VCC | Encoder supply  | blue   | **Pi 3.3 V** (phys 17 left / phys 1 right) |
| 6 | M2  | Motor winding − | white  | L298N OUT2 (left) / OUT4 (right) |

Note the layout: the two high-current motor wires are at the **outside** of the connector and
the four low-level encoder wires are in the middle. That is deliberate on the motor's part and
you should preserve it — split the harness at the connector into a fat two-wire motor pair
routed one way and a thin four-wire signal bundle routed the other. **Never zip-tie them
together for their whole length.**

### The single most common wiring mistake on this build

Pin 2 is called GND, it is black, and it is 8 mm away from pin 1 which goes to the L298N.
Connecting pin 2 to the **L298N's** ground terminal instead of the Pi's is an entirely natural
mistake and it is the wrong answer. See §5.

> **VERIFY — harness continuity.** Before connecting anything to the Pi, with the connector
> unplugged from the motor: buzz out every wire end-to-end. Then with the connector plugged
> into the motor and nothing else connected, measure M1→M2: you should read the winding
> resistance, a few ohms, and it should change as you turn the shaft (that is the commutator
> moving). Measure VCC→GND: open circuit or a high resistance, never a short. Measure
> M1→VCC and M1→GND: **must be open.** A motor winding shorted to the encoder supply will
> put 9 V onto the Pi's 3.3 V rail and destroy the SoC.

---

## 5. Star ground topology

There are two currents in this car that differ by four orders of magnitude, and the entire
grounding scheme exists to keep them out of each other's way.

- **Motor return current:** up to 1.5 A, switching at 1 kHz, with brush noise on top.
- **Encoder logic current:** a few milliamps, and its *reference* is the Pi's 3.3 V ground.

```
                                ★ = the one and only star point
                                    (L298N GND screw terminal)

     battery (−) ══════════════════════★
                                       ║
     regulator GND ════════════════════║
                                       ║
     L298N GND terminal ═══════════════★
                                       ║
                                       ║  ONE dedicated wire, 22 AWG,
                                       ║  logic return only (µA)
                                       ║
     Pi phys pin 39 ───────────────────╝

     Pi phys 14 ──── left encoder GND    ]  these return to the Pi,
     Pi phys 9  ──── right encoder GND   ]  NOT to the star point
     Pi phys 6  ──── servo GND           ]
     Pi phys 20 ──── LED cathode         ]
     Pi phys 34 ──── E-stop return       ]

     ══ = thick (18–20 AWG), carries motor current
     ── = thin (22–26 AWG), carries logic current only
```

### The rules

1. **Battery negative, regulator ground, and the L298N GND terminal are one node.** Keep those
   three connections short and fat. This is where all 1.5 A of motor return current lives.
2. **The Pi joins that node with exactly one dedicated wire** (physical pin 39 → L298N GND
   terminal). That wire carries only the return for six logic-level inputs — microamps. It
   exists so the Pi and the L298N agree on what 0 V means.
3. **Every other Pi ground stays on the Pi.** Encoder grounds, servo ground, LED, E-stop: all
   return to a header ground pin. Their supply comes from the Pi, so their return belongs to
   the Pi.
4. **Never daisy-chain.** Do not run L298N GND → Pi pin 39 → Pi pin 14 → encoder. Each ground
   connection is its own wire back to its own point.

### Why this matters concretely

Suppose you route the left encoder's GND to the L298N ground terminal, 150 mm of 24 AWG away
from the star point. That wire has about 13 mΩ. Now the motors pull 1.5 A through the shared
copper: the encoder's ground reference lifts by tens of millivolts, modulated at 1 kHz by the
PWM and spiked by brush arcing.

The hall output is referenced to *that* ground. The Pi's input threshold is referenced to the
*Pi's* ground. The difference is now a noise source injected directly into a signal you are
counting edges on.

**The symptom is phantom encoder counts that appear only under motor load.** It looks exactly
like a failing encoder. You will replace the motor, and the new one will do the same thing.

> **VERIFY — grounding.** With everything wired and the motor battery *disconnected*, measure
> resistance from Pi physical pin 39 to the L298N GND terminal: **under 0.5 Ω**. Then measure
> from Pi physical pin 14 (encoder ground) to the L298N GND terminal: it will read low too —
> that is expected, they are connected through pin 39 — but confirm by unplugging the pin 39
> wire that the encoder ground now reads **open** to the L298N. If it still reads a short with
> pin 39 removed, you have a second ground path and you must find it.

---

## 6. Decoupling and noise suppression

| Where | Part | Mandatory? | Purpose |
|-------|------|-----------|---------|
| Across each motor's M1–M2, at the motor terminals | 100 nF ceramic X7R, 50 V | **Yes** | Shunts brush arcing at the source |
| Each motor terminal to the motor can (×2 per motor) | 100 nF ceramic X7R, 50 V | If phantom edges persist | Kills the can-to-terminal RF path |
| Encoder VCC to encoder GND, **at the motor's encoder PCB** | 100 nF ceramic | **Yes** | Local decoupling for the hall sensors |
| L298N +12 V (VS) to GND, at the screw terminals | 470–1000 µF electrolytic, **≥16 V** | **Yes** | Supplies the switching step the wiring cannot |
| In parallel with the above | 100 nF ceramic | **Yes** | Electrolytics have poor ESR above ~10 kHz |
| Servo +5 V to GND, at the servo connector (Pi phys 4 / 6) | 470–1000 µF electrolytic, **≥10 V** | **Yes** | See [power.md](power.md) — and read the honest limits there |
| In parallel with the above | 100 nF ceramic | **Yes** | Same reason |

**Electrolytics are polarised.** The stripe is the negative leg. Backwards, at 9 V, they vent
— loudly, and with a smell you will remember. Check twice.

**Why the bulk cap at the L298N is mandatory and not "nice to have":** the wire from the
regulator to the bridge has inductance. When the bridge turns on at the start of a PWM cycle
it demands a current step the wire physically cannot supply instantly. The resulting dip
appears at the regulator's output, and the regulator's response to a fast dip is to trip. The
capacitor supplies that step locally so the regulator only sees the *average*. Without it, the
car brownouts under acceleration and it looks like a battery problem.

**The can capacitors are the escalation, not the baseline.** Soldering to a motor can needs a
big iron and a fast hand; dwell too long and you cook the adhesive holding the magnets.
Fit the M1–M2 cap first, run [bringup.md](bringup.md) Phase 3, and only add the can caps if
you actually see encoder counts appear when the motors are driven but the shaft is held.

**A note on the L298N module's freewheel diodes.** The module already carries the eight
flyback diodes the L298 needs. Cheaper boards fit 1N4007s, which are slow rectifiers where the
datasheet asks for fast ones. The result is extra heat and reverse-recovery spikes rather than
outright failure. If your bridge runs noticeably hotter than the ~3 W estimate in
[power.md](power.md) predicts, this is a candidate. It is not worth pre-emptively reworking.

---

## 7. The I2S overlay conflict — read this before you debug anything

Four of our pins are the BCM283x I2S/PCM peripheral in ALT0:

| BCM | I2S function | We use it for |
|-----|--------------|---------------|
| 18  | PCM_CLK  | **Servo signal** |
| 19  | PCM_FS   | (unused, but inside the block) |
| 20  | PCM_DIN  | **IN3 — right motor direction A** |
| 21  | PCM_DOUT | **IN4 — right motor direction B** |

Any I2S device-tree overlay reassigns all four to ALT0 **at boot, before pigpiod ever
starts.** pigpio will then accept your commands and report success while the pin mux ignores
you completely.

The offenders, all of which are perfectly normal lines to find in a `config.txt` inherited
from another project:

```
dtoverlay=hifiberry-dac        dtoverlay=hifiberry-dacplus
dtoverlay=iqaudio-dac          dtoverlay=iqaudio-dacplus
dtoverlay=audioinjector-wm8731-audio
dtoverlay=googlevoicehat-soundcard
dtoverlay=rpi-dac              dtoverlay=allo-boss-dac-pcm512x-audio
dtparam=i2s=on
```

**Symptom:** the servo does nothing at all, and the right motor either refuses to run or runs
in one direction only. The left motor works perfectly. It looks precisely like a wiring fault
on the right-hand side, and you will spend an hour re-soldering a harness that was always
fine.

### The separate, second hazard: `dtparam=audio=on`

This one is worse because it is the **default** on a fresh image. It loads `snd_bcm2835`,
which binds the PWM peripheral to drive analogue audio. On a Zero 2 W there is no headphone
jack for it to drive, and it takes both PWM channels anyway.

**Symptom:** `pigs hp 12 1000 500000` returns `0` (success) and nothing happens. Both motors
are dead. There is no error message anywhere. `dtparam=audio=off` is mandatory —
see [pi-setup.md](pi-setup.md).

> **VERIFY — pin mux.** Before anything else, on a booted Pi with pigpiod stopped:
> ```
> pinctrl get 5,6,12,13,18,20,21        # Bookworm
> raspi-gpio get 5,6,12,13,18,20,21     # older
> lsmod | grep snd_bcm2835              # must print nothing
> grep -nE 'i2s|hifiberry|iqaudio|audioinjector|googlevoicehat|allo-|rpi-dac' \
>      /boot/firmware/config.txt        # must print nothing
> ```
> Every pin must report `func=INPUT` or `func=OUTPUT`. **If any pin reports `PCM_CLK`,
> `PCM_DIN`, `PCM_DOUT` or `PWM0`/`PWM1` before pigpiod has started, stop — no amount of
> wiring will fix it.**

---

## 8. L298N module configuration

The red L298N breakout has three sets of jumpers. Get them right before you power it.

| Jumper | Position | Why |
|--------|----------|-----|
| **5V-EN** (next to the screw terminals) | **IN** | With VS at 9 V the onboard 78M05 self-powers the L298's logic. Its load is tens of milliamps, so dissipation is negligible. |
| **ENA** (3-pin header) | **REMOVED** | It shorts ENA to +5 V. We PWM that pin. |
| **ENB** (3-pin header) | **REMOVED** | Same. |

**The `+5V` screw terminal must be left completely unconnected.** With the 5V-EN jumper in,
that terminal is an *output* — it can source about half an amp from the motor battery.
Connecting it to the Pi's 5 V rail back-feeds the Pi from the motor pack, which fights the
powerbank, browns out at power-down, and pushes current backwards through the powerbank's
output stage. There is no version of this that ends well.

**If you ever raise VS above 12 V** (see the upgrade path in [power.md](power.md)): pull the
5V-EN jumper and feed 5 V into the `+5V` terminal from a separate source, because the 78M05
will otherwise dissipate `(Vin − 5) × I` in a TO-220 with no heatsink.

### Terminal assignment

| Terminal | Connects to |
|----------|-------------|
| `+12V` (VS) | Regulator VOUT (9 V) |
| `GND` | ★ star ground point |
| `+5V` | **nothing** |
| `OUT1` / `OUT2` | Left motor M1 / M2 |
| `OUT3` / `OUT4` | Right motor M1 / M2 |

`ENA`/`IN1`/`IN2` drive `OUT1`/`OUT2`. `ENB`/`IN3`/`IN4` drive `OUT3`/`OUT4`.

### The truth table you will get backwards

With EN **HIGH**:

| IN1 | IN2 | Output state |
|-----|-----|--------------|
| H | L | Forward |
| L | H | Reverse |
| **H** | **H** | **BRAKE** — both windings shorted to VS |
| **L** | **L** | **BRAKE** — both windings shorted to GND |

**Coast requires EN LOW.** `IN1 == IN2` is a brake, not a coast. This is the opposite of most
people's intuition, and getting it backwards means every time you meant to let the car roll
you instead short a spinning motor into the bridge. The firmware encodes this correctly in
`MotorPair.brake()` / `MotorPair.coast()` ([INTERFACES.md §3](INTERFACES.md)); when you are
poking pins by hand on the bench, it is on you.

### Heat

The L298 is a Darlington bridge and drops roughly 2 V across its output stage. At the 1.5 A
combined budget that is about 3 W into a heatsink rated for 2–3 W — you are *at* the limit at
full combined duty, by design. This is exactly why `duty_sum_max` exists as a parameter
(default 1.40, see [`params.py`](../packages/telekart_protocol/telekart_protocol/params.py)).
The arithmetic is in [power.md](power.md).

---

## 9. Encoder supply voltage — 3.3 V, with a caveat

The design powers both encoders from the **Pi's 3.3 V rail** so their outputs land inside the
Pi's input range with no level shifting and no chance of a 5 V hall output meeting a 3.3 V-
tolerant-only GPIO.

Most GA37-520 hall encoders are specified 3.3–5 V and work correctly at 3.3 V. **Some
JGB37-class encoders from some suppliers are 5 V-only** — usually because the hall IC's
pull-up network was sized for 5 V, or because the part itself has a 4.5 V minimum. At 3.3 V
they either do not toggle at all, or produce a swing too small to cross the Pi's input
threshold.

### How to test for it

Wire one encoder at 3.3 V, power **only the Pi**, motor battery disconnected, and turn the
output shaft slowly by hand — roughly one revolution every two seconds.

```bash
# watch the raw level while you turn the shaft
watch -n 0.2 'pinctrl get 23,24'
```

| What you see | Verdict |
|--------------|---------|
| Both channels toggle 0 ↔ 1 many times per revolution | **Good.** 3.3 V works. Continue. |
| Levels never change | Encoder is not running at 3.3 V, **or** it is not wired. Check VCC with a meter first. |
| Levels change but erratically / one channel stuck | Marginal swing. Treat as 5 V-only. |

Confirm with a meter before concluding anything: put the DMM on DC volts between C1 and
encoder GND and turn the shaft very slowly. A healthy 3.3 V encoder swings between under
0.4 V and over 2.9 V. If the high side only reaches 1.5–2.2 V, the sensor is browning out and
you have a 5 V part.

### If they are 5 V-only

Power VCC from the Pi's 5 V rail (physical pin 2), keep the ground where it is, and **level
shift both C1 and C2 down before they reach the GPIO.** A 5 V logic output into a Pi GPIO will
damage the pin — the BCM283x inputs are not 5 V tolerant.

The cheap fix is a resistor divider per channel:

```
   C1 (5 V) ──┬── 10 kΩ ──┬── to GPIO
              │           │
              │         18 kΩ
              │           │
             GND ─────────┴── GND

   5 V × 18k / (10k + 18k) = 3.21 V     ← comfortably a logic HIGH, safely under 3.3 V
```

Fit the divider **at the Pi end** of the harness so the long run stays at 5 V swing and is
less susceptible to picking up motor noise.

The better fix is a proper bidirectional level shifter module (TXS0108E or similar) if you
have one — a divider plus the harness capacitance is a low-pass filter, and at 2200 edges per
second that is not a problem, but it does erode your margin.

> **VERIFY — encoders.** With motor power still disconnected, mark the wheel with tape and
> hand-turn one full revolution. Both channels must have toggled. Then run the edge count
> from [bringup.md](bringup.md) Phase 3 before you connect motor power. **Do not skip
> ahead** — an encoder that phantom-counts is far easier to diagnose with the motors dead.

---

## 10. Assembly order and per-step verification

Follow this order. Each step ends in a check you can actually perform.

### Step 1 — Pull-downs, alone

Fit R1–R6 on the L298N. Nothing else connected.

> **VERIFY:** §3. All six read 10 kΩ ±5% to GND.

### Step 2 — L298N jumpers

Set 5V-EN in, remove both ENA/ENB jumpers.

> **VERIFY:** Visually confirm the two 3-pin headers are bare and the 5V-EN shunt is seated.
> With a meter, ENA→+5 V terminal must be **open** (if it reads a short, the ENA jumper is
> still on).

### Step 3 — Bundle A: Pi → L298N logic

Seven wires, physical pins 29/31/32/33/38/39/40.

> **VERIFY:** Buzz each wire Pi-pin-to-L298N-pin. Then, Pi powered from the powerbank only,
> motor battery still disconnected, run the pin-mux check in §7 and confirm all six control
> pins read `level=0`.

### Step 4 — Star ground and the regulator

Battery, fuse, master switch, regulator, and the ground star. **Do not connect the regulator
output to the L298N yet.**

> **VERIFY:** Follow the S18V20ALV procedure in [power.md](power.md) — set the output to
> 9.0 V with a DMM under **no load** before it is connected to anything. Then confirm §5's
> ground checks.

### Step 5 — Motor power

Regulator VOUT → L298N `+12V`. Motor leads to OUT1–OUT4. Bulk cap + 100 nF at the screw
terminals.

> **VERIFY:** Master switch OFF. Check polarity of the bulk cap **again**. Switch on with the
> Pi *unpowered*: with the pull-downs holding all six inputs low, the motors must not twitch,
> and the L298N must draw only its quiescent logic current. Anything that moves means a
> pull-down is not doing its job. Switch off.

### Step 6 — Encoder bundles C and D

Split the harness at the connector: motor pair one way, signal bundle the other.

> **VERIFY:** §9's hand-turn test on both wheels, motor battery disconnected.

### Step 7 — Servo

Bundle B plus the bulk cap across physical pins 4 and 6. **Fit the servo horn later** — see
[calibration.md](calibration.md); the linkage must be disconnected the first time the servo is
commanded.

> **VERIFY:** [bringup.md](bringup.md) Phase 2. The servo is the first thing that draws real
> current from the Pi's rail, so this is where `vcgencmd get_throttled` starts mattering.

### Step 8 — Panel

LED + 330 Ω, E-stop button.

> **VERIFY:** `pinctrl set 25 op dh` lights the LED; `pinctrl set 25 op dl` extinguishes it.
> Then, with the internal pull-up enabled (`pinctrl set 16 ip pu`), `pinctrl get 16` reads
> `level=1` at rest and `level=0` while the normally-open button is held. **If it reads
> `level=0` at rest you have fitted a normally-closed button** — the car will latch `ESTOP` on
> boot and never arm. See §2.4.

### Step 9 — Camera

CSI ribbon into the Zero 2 W's **mini** CSI connector. Contacts face the board; the blue
stiffener faces away. The Zero's connector is a smaller pitch than a full-size Pi's — you need
the narrow ribbon that ships with the Zero-specific camera cable, not the one in the camera's
own box.

> **VERIFY:** `rpicam-hello --list-cameras` reports `imx219`. If it reports nothing, reseat —
> this connector is unforgiving about being a fraction of a millimetre out.

### Step 10 — Strain relief and routing

Every bundle gets a zip tie within 30 mm of both connectors. The motor pairs and the encoder
signal bundles run on **opposite sides** of the chassis where physically possible; where they
must cross, cross them at 90°, never parallel. Nothing may be able to reach a wheel or a
gear.

> **VERIFY:** Lift the car, turn each wheel through a full revolution by hand, and turn the
> steering lock to lock. Nothing snags, nothing stretches, nothing rubs.

---

## 11. Complete pin summary

These are the shipped defaults in `pi/config/telekart.yaml`, and they match
[INTERFACES.md §7](INTERFACES.md) and [`constants.py`](../pi/telekart/constants.py) — they are
physical facts about this car, not preferences, so there is normally nothing here to copy or
change. (Per-vehicle *measurements* go in `pi/config/config.local.yaml` instead; see
[calibration.md §1](calibration.md).)

```
ENA         = GPIO12   phys 32     hardware PWM ch0
IN1         = GPIO5    phys 29     needs external pull-down (idles HIGH)
IN2         = GPIO6    phys 31     needs external pull-down (idles HIGH)
IN3         = GPIO20   phys 38     I2S PCM_DIN — keep overlays off
IN4         = GPIO21   phys 40     I2S PCM_DOUT — keep overlays off
ENB         = GPIO13   phys 33     hardware PWM ch1, shares divider with ch0
SERVO       = GPIO18   phys 12     DMA pulses; GPIO18's PWM is ch0, already used
ENC_L_A     = GPIO23   phys 16     edge callback
ENC_L_B     = GPIO24   phys 18     polled only
ENC_R_A     = GPIO27   phys 13     edge callback
ENC_R_B     = GPIO22   phys 15     polled only
STATUS_LED  = GPIO25   phys 22     330 Ω, red or green only
ESTOP_BTN   = GPIO16   phys 36     normally-OPEN to GND, pull-up, LOW = pressed
L298N GND   =          phys 39     the one logic return wire
SERVO 5V    =          phys 4      bulk cap across phys 4/6
SERVO GND   =          phys 6
ENC L 3V3   =          phys 17     ENC L GND = phys 14
ENC R 3V3   =          phys 1      ENC R GND = phys 9
```

The corresponding pigpiod GPIO mask, which restricts the daemon to exactly these thirteen
pins, is `0x0BF53060`. It is derived and explained in [pi-setup.md](pi-setup.md).

---

## 12. Where to go next

1. [power.md](power.md) — set the regulator, fit the fuse, understand the 1.5 A budget.
2. [pi-setup.md](pi-setup.md) — OS image, `config.txt`, pigpiod flags, systemd.
3. [bringup.md](bringup.md) — the bench sequence, wheels off the ground, with hard gates.
4. [calibration.md](calibration.md) — the numbers you measure once the car runs.
5. [tuning.md](tuning.md) — PID, PWM frequency, camera latency.
