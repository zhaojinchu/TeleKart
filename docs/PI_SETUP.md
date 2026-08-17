# Pi bring-up runbook

> **This document is written, not executed.** Nothing in it has been run. It exists so that the
> bring-up session is a matter of following steps rather than improvising next to a car that can
> drive itself into a wall.

---

## Before anything else

**The Pi is currently running other software that controls the kart.** Until that is deliberately
stopped, none of this runs. Two processes driving the same GPIO pins is the worst failure mode
available here — both will set the same L298N inputs, neither will know, and the bridge is what pays
for it.

Three physical checks first, in this order:

1. **External 10 kΩ pull-downs on ENA, ENB, IN1, IN2, IN3, IN4 are installed.** Not optional.
   GPIO0–8 default to pull-**up** on BCM283x silicon, so IN1 (GPIO5) and IN2 (GPIO6) sit HIGH from
   the instant the Pi receives power. That covers boot, reboot, power loss, and a loose ribbon
   cable — every window in which no software of ours exists to hold the pins down.
2. **The battery-side switch is OFF.** It is the physical disconnect and the last thing that still
   works when everything else has failed. Know where it is before you need it.
3. **The wheels are off the ground.** Books, a box, anything. Every step below assumes this until
   explicitly stated otherwise.

---

## 1. Stop the incumbent

Whatever is currently controlling the kart must be stopped, and confirmed stopped, before our
process starts. Confirm both that its process is gone and that nothing holds our pins.

Also confirm our ports are free:

```bash
ss -tulpn | grep -E '8090|8091'   # expect no output
```

If either is taken, change `VIDEO_PORT` / `CONTROL_PORT` in `common/protocol.py`, re-copy
`pi/protocol.py`, and run `make check-protocol`. Do not work around a collision by killing the other
listener.

## 2. System packages

```bash
sudo apt update
sudo apt install -y pigpio python3-pigpio python3-picamera2
```

`picamera2` is **not reliably pip-installable** — it binds to the system libcamera stack. It comes
from apt, and so does pigpio's Python binding. This is why the venv below needs system packages
visible.

## 3. Our folder

```bash
mkdir -p ~/telekart2
python3 -m venv --system-site-packages ~/telekart2/.venv
```

`--system-site-packages` is load-bearing: without it the venv cannot see the apt-installed
`picamera2` and `pigpio`.

> **Isolation rule lifted 2026-08-16.** During bring-up nothing outside `~/telekart2/` was touched,
> because the Pi was running other software. Testing is done and the car now installs a systemd unit
> in `/etc` — see below.

## 4. Copy our code across

From the Mac:

```bash
rsync -av --exclude='__pycache__' pi/ telekart@telekart.local:~/telekart2/
```

From this point the Pi copy is authoritative and edits happen there; `make pull-pi` snapshots it back
to the Mac repo for git.

## 5. Services (both enabled at boot)

```bash
sudo systemctl enable --now pigpiod
sudo cp ~/telekart2/telekart2.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now telekart2
```

The car now comes up on its own at power-on. It starts in `FAILSAFE` with the bridge coasted, the
servo silent and throttle ignored until you arm, so booting is not the same as being live.

Day to day:

```bash
systemctl status telekart2
journalctl -u telekart2 -f          # the 1 Hz summary, live
sudo systemctl restart telekart2    # after editing code on the Pi
sudo systemctl stop telekart2       # `pkill` no longer works -- Restart=always brings it back
```

`Restart=always` means a crash recovers by itself. It also means `pkill` is not how you stop it.

Both were left manual for most of bring-up, and that cost a debugging session: the Pi rebooted while
idle, nothing restarted, and three launches in a row showed `NO TELEMETRY` while the app happily
transmitted into the void. Enabled is the right default for anything you expect to just work.

pigpio needs its daemon and root-level DMA access. If the incumbent software also used pigpio, the
daemon may already be running — that is fine and shared, but the incumbent *process* still must not be.

### Why pigpio, and why not `hardware_PWM`

We use pigpio's DMA-timed `set_PWM_frequency` / `set_PWM_dutycycle` at 1 kHz on GPIO12/13, not its
`hardware_PWM()`. That sidesteps two documented problems at once:

- The two hardware PWM channels **share a single clock divider**. Setting GPIO12 to one frequency and
  GPIO13 to another corrupts channel 0 — they are not independent peripherals.
- **`snd_bcm2835` claims both hardware PWM channels.** The audio driver and the motor enables contend
  for the same peripheral, and on a desktop image it gets pulled back in through the HDMI codec path
  even after the audio overlay is disabled.

DMA PWM avoids that fight entirely. If the waveform ever needs to be cleaner, `hardware_PWM` is the
upgrade — and then blacklisting `snd_bcm2835` becomes mandatory rather than moot.

---

## 6. Bring-up, in order

Each phase is gated on the previous one. Do not skip ahead because a phase looked boring.

### Phase B — no motion at all

Battery switch **OFF**. Wheels off the ground.

```bash
cd ~/telekart2 && .venv/bin/python main.py
```

From the Mac:

```bash
curl -s http://telekart.local:8090/health          # uptime, frames, fps
make drive-car                                      # the pygame app, pointed at the car
```

Note `drive-car`, not `drive` — the plain target talks to the simulator on the Mac. If the HUD shows
`127.0.0.1` you are driving a test pattern, not a vehicle.

Confirm: video renders, telemetry arrives, RTT is sane on the LAN, `state` reads `SAFE`, and the
debug panel populates. Steer with A/D — **the servo moves even while disarmed**, deliberately, so
this is where you check it. Verify centre sits at 1500 µs and the travel stops inside 1200–1800 µs.
Adjust `SERVO_TRIM_US` in `pi/config.py` if centre is off.

Also confirm the camera FOV looks wide, not telephoto. If it looks zoomed in, the `raw` size in
`pi/video.py` is not taking effect — a bare 640×480 request lands on a 1280×960 crop of the IMX219's
array, about 2.6× telephoto, and the car is genuinely hard to drive that way.

Then pull the WiFi or kill the Mac app: the red banner should appear within 500 ms, and the Pi should
log `FAILSAFE` within 300 ms.

### Phase C — motors live, wheels still off the ground

Battery switch **ON**. Hand on the switch.

Arm with Enter, then the smallest throttle that moves the wheels. Confirm in order:

- Both rear wheels turn the **same** direction, and that direction matches W.
- S reverses them.
- Releasing W coasts rather than braking.
- Space brakes.
- W→S while spinning produces the ~100 ms coast, not an instant reversal.
- Holding W shows `throttle_out` **ramping** toward `throttle_cmd` over ~0.4 s in the debug panel.

Watch `cpu_temp` and touch the L298N heatsink. Two stalled motors put roughly 7 W into a heatsink
rated for 2–3 W, so do not hold a stall to see what happens.

If the Pi resets when the servo hits its limit, that is the HS-311 browning out the 5 V rail it
shares with the SoC — not a software bug.

### Phase D — on the floor

Small space, `MAX_DUTY` still at 0.60, hand near the battery switch. Raise `MAX_DUTY` only once it
drives predictably.

---

## Measured values for this car — bring-up 2026-08-16

These are now in `pi/config.py`. **Every steering default was wrong**, which is exactly what the
"a per-vehicle override supersedes the checked-in numbers" caveat was warning about. They were found
by sweeping with `servo_probe.py` and reading the angle off by eye.

| Setting | Checked-in default | **Measured** | |
|---|---|---|---|
| `servo_centre_us` | 1500 | **1950** | Straight ahead. The horn sits ~40° round its spline |
| `servo_range_left_us` | 300 | **400** | 1550 µs at full left |
| `servo_range_right_us` | 300 | **450** | 2400 µs at full right |
| steering lock | ±24° | **~±35°** | More travel than the HS-311's nominal 900–2100 span |
| `INVERT_RIGHT` | — | **`True`** | The motors ran against each other until one side was inverted |

Two things this car taught us, worth carrying forward:

**The steering linkage is not symmetric.** The same wheel angle costs 400 µs to the left and 450 µs
to the right. A single shared range would give a correct lock one way and a short one the other, so
`VehicleConfig` carries a separate span per side. Equal *angle* is what a driver feels; equal
microseconds is not.

**The motors are mounted mirror-image.** Identical bridge polarity spins them in opposite rotational
directions, so one side must be inverted. Which side is arbitrary — it depends on how the leads were
landed on the L298N terminals — which is why it lives in config as calibration.

### Still on defaults

| Setting | Default | Adjust when |
|---|---|---|
| `min_duty` | 0.30 | The car buzzes without moving, or lurches from standstill |
| `max_duty` | 0.60 | Only after it drives predictably on the floor |
| `slew_per_s` | 2.5 | Brownouts under acceleration (lower it) |
| `servo_trim_us` | 0 | Residual offset after centre is set |
| `ESTOP_ENABLED` | `False` | Once the GPIO16 button's polarity is confirmed |

### The E-stop

The code for GPIO16 is written but **disabled by default**, because the button's wiring polarity has
not been verified. The code assumes active-low with an internal pull-up. If that guess is wrong,
enabling it means the Pi believes the E-stop is permanently pressed and silently refuses to drive —
which looks exactly like a dead motor driver and would cost an hour of debugging in the wrong place.

Confirm the polarity by reading GPIO16 pressed and released, then set `ESTOP_ENABLED = True`.

---

## Reference: pin map (BCM)

| Function | Pin |
|---|---|
| ENA / ENB | 12 / 13 |
| Left IN1 / IN2 | 5 / 6 |
| Right IN3 / IN4 | 20 / 21 |
| Steering servo | 18 |
| Status LED | 25 |
| E-stop button | 16 |
| Encoders | 23/24, 27/22 — **wired but unused in v1** |
