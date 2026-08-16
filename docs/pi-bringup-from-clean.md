# Bringing the Pi back up from a clean OS

The car's Raspberry Pi was reverted to a state where nothing TeleKart-related
runs, starts at boot, or changes a system setting. The **code and the Python
virtualenv were left in place** at `~/telekart` (28 MB) — only the system-level
integration was removed.

This document is the exact path back. It is written against what was actually on
the machine, not against a generic install.

---

## What was removed, and what it did

| Thing | Where | What it was for |
|---|---|---|
| `telekart-control.service` | `/etc/systemd/system/` | The 100 Hz control loop, safety state machine, networking |
| `telekart-video.service` | `/etc/systemd/system/` | Camera capture, H.264 encode, frame server on 4213 |
| `telekart-wifi-nopowersave.service` | `/etc/systemd/system/` | Disables WiFi power save (it adds latency spikes to the control link) |
| `pigpiod.service.d/override.conf` | `/etc/systemd/system/` | Pins pigpiod's GPIO mask and scheduling |
| `telekart-panic-stop` | `/usr/local/sbin/` | The `ExecStopPost` motor kill — **the only layer that survives `SIGKILL`** |
| `telekart-no-audio.conf` | `/etc/modprobe.d/` | `blacklist snd_bcm2835` |
| `dtparam=audio=off` | `/boot/firmware/config.txt` | Frees both hardware PWM channels — restored to `on` |
| `pigpiod` enabled+running | systemd | Now disabled and stopped (Bookworm's default) |

**Every removed file was backed up to `/root/telekart-uninstall-backup/` on the
Pi.** Restoring is mostly a matter of copying them back.

### Not touched

- `~/telekart` — the checkout, the venv, and `pi/config/config.local.yaml`
- apt packages: `pigpio`, `python3-picamera2`, `avahi-daemon` (mDNS still works)
- `dphys-swapfile` — see the caveat at the bottom

---

## Safety before you start

> **The motors are currently held safe only by the external 10 kΩ pull-downs on
> ENA/ENB and IN1–IN4.** No software is running to hold them low. That is exactly
> what those resistors are for, and it is why they are not optional — but it does
> mean that from here until `telekart-control` is running again, the GPIO state is
> a hardware property, not a software one.

Put the car on blocks with at least 20 mm of clearance under each tyre before
the first arm. Know where the battery-side switch is.

---

## Fast path — the installer does all of it

```sh
ssh telekart@telekart.local
cd ~/telekart/pi
sudo ./install.sh
sudo reboot
```

`install.sh` reinstalls the units, the panic-stop helper and the pigpiod
drop-in, and warns about anything it cannot fix itself. The reboot is required
because of the audio/PWM change below.

After the reboot:

```sh
sudo systemctl start telekart-control telekart-video
systemctl status telekart-control --no-pager
```

To have them come back automatically on every boot — they were **not** enabled
before, so this is a deliberate choice, not a restoration:

```sh
sudo systemctl enable telekart-control telekart-video telekart-wifi-nopowersave
```

---

## Manual path — restore exactly what was removed

Run every command on the Pi as `telekart`.

### 1. Restore the files from the backup

```sh
sudo cp /root/telekart-uninstall-backup/telekart-*.service /etc/systemd/system/
sudo mkdir -p /etc/systemd/system/pigpiod.service.d
sudo cp /root/telekart-uninstall-backup/pigpiod-override.conf \
        /etc/systemd/system/pigpiod.service.d/override.conf
sudo cp /root/telekart-uninstall-backup/telekart-panic-stop /usr/local/sbin/
sudo chmod 0755 /usr/local/sbin/telekart-panic-stop
sudo cp /root/telekart-uninstall-backup/telekart-no-audio.conf /etc/modprobe.d/
sudo systemctl daemon-reload
```

### 2. Free the hardware PWM channels — the step that actually matters

Both motor enables (ENA=GPIO12, ENB=GPIO13) are the *only* pins that reach
hardware PWM channels 0 and 1. The kernel audio driver claims both. If you skip
this, **the motors get no PWM at all** and the symptom is a car that arms, reports
healthy, and does not move.

```sh
sudo sed -i 's/^dtparam=audio=on/dtparam=audio=off/' /boot/firmware/config.txt
grep -n dtparam=audio /boot/firmware/config.txt     # expect: dtparam=audio=off
```

`dtparam=audio=off` **is not sufficient on its own** on this image. It stops the
dtoverlay path, but `dtoverlay=vc4-kms-v3d` (which is enabled in your
`config.txt`) pulls `snd_bcm2835` back in through `snd_soc_hdmi_codec`. That is
why the modprobe blacklist in step 1 exists. You need both.

```sh
sudo reboot
```

Verify after the reboot — this must print nothing:

```sh
lsmod | grep '^snd_bcm2835'
```

If it prints a line, the PWM channels are still claimed and the motors will not
turn. (`snd_soc_hdmi_codec` appearing in `lsmod` is fine — that shim does not
claim the PWMs. `snd_bcm2835` is the one that matters.)

### 3. Start pigpiod

```sh
sudo systemctl enable --now pigpiod
systemctl is-active pigpiod            # expect: active
```

Confirm it came up with the TeleKart GPIO mask, which permits exactly the
thirteen pins the car uses and nothing else:

```sh
pgrep -a pigpiod                       # expect: -x 0x0BF53060
```

`0x0BF53060` decodes to GPIO `{5,6,12,13,16,18,20,21,22,23,24,25,27}`.

### 4. Start the car

```sh
sudo systemctl start telekart-control telekart-video
journalctl -u telekart-control -n 20 --no-pager
```

You want to see:

```
motor pair ready  ena=12 enb=13 pwm_hz=1000 max_duty=0.85 duty_sum_max=1.4 invert_left=False invert_right=True
steering servo ready  pin=18 centre_us=1500 min_us=1200 max_us=1800 lock_deg=24
telekart control process ready  car_id=telekart firmware=2.0.0 backend=PigpioBackend
```

`backend=PigpioBackend` is the line to check. If it says `MockBackend`, pigpiod
is not reachable and nothing will move.

---

## Verifying it works

### Ports

```sh
ss -lntu | grep -E '4210|4212|4213'
```

Expect UDP 4210, TCP 4212, TCP 4213.

### GPIO at rest — before you arm anything

```sh
for p in 5 6 12 13 20 21 18; do printf "GPIO$p=$(pigs r $p) "; done; echo
```

All seven must read `0`. Then check the PWM pins are in the right mode:

```sh
pigs mg 12; pigs mg 13      # both must print 4 = ALT0 = hardware PWM
```

Mode `1` (OUTPUT) instead of `4` means pigpio could not get hardware PWM — go
back to step 2.

### From the Mac

```sh
cd ~/Projects/TeleKart
make run-ui ARGS="--host telekart.local"
```

There is no passphrase — the shared-key layer was removed from both ends. The
station connects to whatever answers on port 4212.

---

## Two things that are still outstanding on this car

Neither was caused by the uninstall; both were open before it.

**1. The right encoder is unverified.** The car logged `STALL_R` — "commanded but
not turning" — *after* the motor-direction problem was fixed, so it is not
explained by that. Either the right motor is struggling or its encoder is blind.
Settle it with the wheels off the ground:

```sh
watch -n0.2 'for p in 23 24 27 22; do printf "GPIO$p=$(pigs r $p) "; done; echo'
```

Left is GPIO23/24, right is GPIO27/22. Spin each wheel by hand. If the right pair
never toggles while the left does, that encoder is the fault — and every
right-side stall is phantom, with the motor turning fine and the firmware unable
to see it.

**2. The car has never been calibrated.** There is no `calibration.yaml`, so it
runs on a provisional `max_rpm=120` guess and raises `CALIBRATION_MISSING` every
boot. Speed readings and the feedforward table are guesses until you run:

```sh
cd ~/telekart/pi && ./.venv/bin/python scripts/calibrate_drive.py
```

Do the encoder check first — calibration measures RPM per direction and would
inherit a blind encoder.

`pi/config/config.local.yaml` **was left in place**. It carries the motor
direction fix (`invert_left: false`, `invert_right: true`), which was determined
empirically on this car: with both flags off the wheels drove in opposite
directions, fought each other, and tripped the 1.5 A regulator ceiling.

---

## Caveat: dphys-swapfile

`dphys-swapfile` is currently **disabled and inactive** on this Pi. Raspberry Pi
OS enables it by default, and `install.sh` recommends disabling it (swap on an SD
card under a real-time control loop is a latency hazard) — but the installer
never does it itself.

So I could not tell whether it was disabled for TeleKart or was your own
preference, and **I left it alone rather than guess**. If you want the stock OS
behaviour back:

```sh
sudo systemctl enable --now dphys-swapfile
```

---

## Full teardown, if you ever want it gone completely

```sh
sudo systemctl disable --now telekart-control telekart-video telekart-wifi-nopowersave pigpiod
sudo rm -f /etc/systemd/system/telekart-*.service \
           /usr/local/sbin/telekart-panic-stop \
           /etc/modprobe.d/telekart-no-audio.conf
sudo rm -rf /etc/systemd/system/pigpiod.service.d /etc/telekart
sudo sed -i 's/^dtparam=audio=off/dtparam=audio=on/' /boot/firmware/config.txt
sudo systemctl daemon-reload
rm -rf ~/telekart                                    # the code and venv too
sudo apt remove --purge pigpio python3-picamera2     # optional
sudo reboot
```
