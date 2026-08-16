# TeleKart v2 — Raspberry Pi Setup

From a blank SD card to a Pi that will pass [bringup.md](bringup.md) Phase 1.

Everything here assumes a **Raspberry Pi Zero 2 W** (BCM2710A1, quad Cortex-A53, **512 MB**,
2.4 GHz WiFi only). The memory figure and the single-band radio drive several decisions below
that would be pointless on a Pi 4.

> **pigpio does not work on a Raspberry Pi 5.** The Pi 5's RP1 southbridge moved the GPIO
> block off the SoC and pigpio's direct-register and DMA access no longer applies. If you port
> this project to a Pi 5, the entire `hal/pigpio_backend.py` layer has to be rewritten against
> `libgpiod`. That is exactly the kind of change the `GpioBackend` abstraction in
> [INTERFACES.md §2](INTERFACES.md) exists to contain, but it is not a config change.

**Companion documents:** [wiring.md](wiring.md) — especially §7 on the I2S conflict ·
[power.md](power.md) for `get_throttled` · [bringup.md](bringup.md) for the gates.

---

## 1. OS image

**Raspberry Pi OS Bookworm, 64-bit, Lite.**

| Choice | Why |
|--------|-----|
| **Bookworm** | Ships Python **3.11**, which is what `pi/`, `packages/` and `tools/` target ([INTERFACES.md §0](INTERFACES.md)). It is also where `libcamera` + `picamera2` are the supported camera stack rather than a bolt-on. |
| **64-bit** | The Cortex-A53 is ARMv8. The 64-bit image is where camera and codec support gets tested first, and 64-bit Python is measurably faster on integer work. There is no 32-bit advantage on this SoC. |
| **Lite** | No desktop. A desktop costs 150–250 MB of a 512 MB machine and adds a compositor competing for the same four cores as your 100 Hz control loop. You will never look at it. |

### Flashing with Raspberry Pi Imager

Use the Imager's advanced options (the gear icon) and set all of these before writing:

- **Hostname:** `telekart` — this is what mDNS advertises and what the desktop app resolves.
- **Enable SSH**, with a **public key** rather than a password. A car you can SSH into with a
  password, on a shared network, is a car somebody else can drive.
- **Username:** whatever you like. This document uses `pi` and the path
  `/home/pi/telekart`; adjust the systemd units in §7 to match.
- **WiFi SSID and password.**
- **WLAN country.** Not optional — on some images the radio stays `rfkill`-blocked until a
  regulatory domain is set, which presents as "WiFi just doesn't work" with no error.

### The SSID must be reachable on 2.4 GHz

The Zero 2 W has **no 5 GHz radio.** A router with band steering — one SSID presented on both
bands, clients pushed to 5 GHz — will make the Pi look like it cannot see your network at all.
If you have any trouble associating, create a 2.4 GHz-only SSID and use that.

While you are in the router: 2.4 GHz has exactly **three non-overlapping channels (1, 6, 11)**.
Pick the least congested one and fix the AP to it. Automatic channel selection will move your
AP mid-drive.

### SD card

Buy an **A1 or A2 rated** card from a company that actually makes flash — SanDisk, Samsung,
Kingston. A **High Endurance** card (the dashcam ones) is the right part for a device that
gets its power yanked. Counterfeits are common enough that it is worth running `f3probe` or
`h2testw` on a new card before you trust it with a build you spent an evening on.

---

## 2. `/boot/firmware/config.txt`

On Bookworm this file is at **`/boot/firmware/config.txt`**. On Bullseye and earlier it was
`/boot/config.txt`. Editing the wrong one and wondering why nothing changed is a rite of
passage; skip it.

### The one mandatory line

```ini
dtparam=audio=off
```

**This is not a tidiness setting. Without it, both motors are dead and there is no error
message.**

The default is `dtparam=audio=on`, which loads `snd_bcm2835` to drive analogue audio out of
the PWM peripheral. The Zero 2 W has no headphone jack for it to drive, and the driver claims
**both hardware PWM channels anyway** — which are ENA (GPIO12) and ENB (GPIO13).

The failure mode is the worst kind: `pigs hp 12 1000 500000` returns `0` for success. pigpio
believes it configured the channel. The pin mux disagrees. Nothing moves, nothing logs, and
you go looking at your wiring.

This is settled and listed as such in [INTERFACES.md §12](INTERFACES.md).

### The full working file

Bookworm Lite's stock `config.txt` needs three edits and nothing else. Here it is with the
changes marked:

```ini
[all]
dtparam=audio=off          # CHANGED from 'on' — frees PWM0/PWM1 for ENA/ENB
camera_auto_detect=1       # keep — this is how the IMX219 is found
display_auto_detect=1
auto_initramfs=1
dtoverlay=vc4-kms-v3d      # keep — do not swap to fkms or remove it
max_framebuffers=2
disable_fw_kms_setup=1
arm_64bit=1
disable_overscan=1
arm_boost=1

dtoverlay=disable-bt       # ADDED — see below
disable_splash=1           # ADDED — a second off the boot, and one less thing on the rail
```

### `dtoverlay=disable-bt`

Bluetooth shares the 2.4 GHz radio with WiFi and the two coexist by time-slicing. On a
single-band device carrying a 100 Hz control stream, that is latency you are paying for a
service you do not use. Pair it with:

```bash
sudo systemctl disable --now hciuart bluetooth
```

### What must NOT be in this file

Grep for these before you go any further. Every one of them steals a pin you need:

```bash
grep -nE 'i2s|hifiberry|iqaudio|audioinjector|googlevoicehat|allo-|rpi-dac|dtparam=audio=on' \
     /boot/firmware/config.txt
```

Any I2S overlay reassigns **GPIO18 (servo), GPIO19, GPIO20 (IN3) and GPIO21 (IN4)** to the
PCM peripheral at boot, before pigpiod ever runs. The symptom is a dead servo and a
right-hand motor that will only run one direction, while the left motor is perfect — which
looks exactly like a wiring fault on the right side. [wiring.md §7](wiring.md) has the full
list and the verification command.

### Things people add that you should not

| Setting | Why not |
|---------|---------|
| `gpu_mem=16` | A Buster-era headless habit. On Bookworm with KMS + libcamera, camera and codec memory comes from **CMA**, not the legacy GPU split. Setting it small can starve the hardware encoder. Leave `gpu_mem` unset. |
| `over_voltage=` / `arm_freq=` | Do not overclock. This board is already fighting for supply headroom ([power.md §6](power.md)); an overclock turns a marginal powerbank into a rebooting one, and it poisons every `get_throttled` reading you take afterwards. |
| `dtoverlay=vc4-fkms-v3d` | The legacy stack. Bookworm's camera path expects full KMS. |

**If** — and only if — the video process reports buffer-allocation failures, raise the CMA
pool rather than touching `gpu_mem`:

```ini
dtoverlay=vc4-kms-v3d,cma-128
```

### `cmdline.txt`

Leave `/boot/firmware/cmdline.txt` alone. Disable the serial login shell through
`raspi-config` (Interface Options → Serial Port → login shell: **No**, hardware serial:
**No**) rather than hand-editing it; GPIO14/15 are unused by this build and there is nothing
to gain.

> **VERIFY.** Reboot, then:
> ```
> lsmod | grep snd_bcm2835                    # must print NOTHING
> pinctrl get 5,6,12,13,18,20,21              # all INPUT or OUTPUT, never PCM_* or PWM*
> vcgencmd get_throttled                      # must be 0x0
> uname -m                                    # aarch64
> python3 -V                                  # 3.11.x
> ```
> All five, or stop here.

---

## 3. pigpio v79

### Remove any packaged copy first

```bash
sudo systemctl disable --now pigpiod 2>/dev/null
sudo apt purge -y pigpio pigpiod
```

This matters. Debian's package installs `pigpiod` at `/usr/bin/pigpiod`; building from source
installs it at `/usr/local/bin/pigpiod`. With both present, `which pigpiod` resolves by PATH
while your systemd unit uses an absolute path, and you can spend a genuinely irritating
half-hour convinced your daemon flags are being ignored — because they are, by the *other*
daemon.

### Build and install the daemon

```bash
sudo apt update
sudo apt install -y build-essential unzip wget
cd /tmp
wget -O pigpio-79.tar.gz https://github.com/joan2937/pigpio/archive/refs/tags/v79.tar.gz
tar xf pigpio-79.tar.gz
cd pigpio-79
make -j4
sudo make install
```

`make -j4` is fine on 512 MB — pigpio is a handful of C files, not a kernel.

```bash
/usr/local/bin/pigpiod -v      # must print 79
```

### The Python client comes from pip, not from `make install`

This is the part that trips people on Bookworm.

`sudo make install` tries to run `python3 setup.py install` for the Python module, which hits
PEP 668's *externally-managed-environment* refusal and either fails loudly or installs
somewhere you did not intend.

You do not need it to. **The pigpio Python module is pure Python** — it is a socket client
that talks to `pigpiod` over TCP 8888. There is no C extension. So it installs cleanly into
your venv with pip:

```bash
/home/pi/telekart/pi/.venv/bin/pip install pigpio
```

(`make setup-pi` does this for you through the `[rpi]` extra — see §4.)

Only the *daemon* has to be the C build at v79. Client and daemon negotiate over a stable
socket protocol, so a minor version difference is tolerable — but matching them removes a
variable.

### The daemon flags

```
/usr/local/bin/pigpiod -t 1 -s 5 -l -b 200 -x 0x0BF53060
```

Every flag, and why:

#### `-t 1` — clock peripheral: PCM, not PWM

pigpio needs a hardware timer to drive its DMA sampling and its servo pulse generator. It can
use either the PWM peripheral or the PCM peripheral. **It must use PCM here**, because the PWM
peripheral is spoken for: ENA and ENB are the two hardware PWM channels.

PCM is already pigpio's default. Setting it explicitly costs nothing and means that a
config inherited from elsewhere, or a future change of default, cannot silently take your
motors away. Given that the exact same class of failure (`dtparam=audio=on`) is the number one
bring-up problem on this board, being explicit is cheap insurance.

#### `-s 5` — 5 µs sample rate

Legal values are 1, 2, 4, 5, 8, 10 µs. 5 is the default and the right answer here.

It sets two things at once:

**Servo pulse quantization.** 5 µs on a 600 µs usable span (1200–1800 µs) is 1 part in 120 —
about 0.2° of steering. An HS-311 cannot resolve that, so finer sampling buys nothing.

**Encoder edge timestamp granularity.** ±5 µs. At 660 counts/rev and 200 RPM the edges arrive
2200 times a second, i.e. every 455 µs:

```
5 µs / 455 µs ≈ 1.1 % velocity noise
```

Well below the quantization noise the M/T estimator is already fighting
([INTERFACES.md §3](INTERFACES.md)). Dropping to `-s 1` would buy you 1.1 % → 0.2 % of a term
that is not the limiting one, in exchange for **five times** the DMA and interrupt load on a
board with a control loop to run. Not worth it.

Going the other way, `-s 10` doubles the servo quantization to 0.4° and the timestamp noise to
2.2 %. Also not worth it. 5 is correct.

#### `-l` — disable the remote socket interface

**This is a safety flag, not a hardening nicety.**

By default `pigpiod` listens on TCP 8888 and accepts commands **from any host on the network,
with no authentication of any kind**. Anyone on the same WiFi can write:

```bash
pigs -h telekart.local hp 12 1000 800000
```

and spin your motors at 80 % duty. There is no session, no key, no log.

`-l` binds the socket to localhost only. The firmware runs on the same machine, so it loses
nothing. If you genuinely need remote `pigs` for a debugging session, use `-n <ip>` to allow
one specific address, and take it back out afterwards.

Note that everything the *protocol* does — the truncated HMAC, the session token, the
strictly-increasing sequence number described in [protocol.md](protocol.md) — is worth
precisely nothing if pigpiod is sitting next to it with an open, unauthenticated port.

#### `-b 200` — 200 ms GPIO sample buffer

Default is 120 ms. The buffer is how much GPIO history pigpio holds before it starts dropping
samples, and it is what protects your encoder counts when the notification thread does not get
scheduled promptly.

On a four-core board running a 100 Hz control loop, a camera process and a WiFi stack, a
200 ms scheduling gap is unlikely but not impossible. 200 ms of buffer means such a gap costs
you nothing at all rather than costing you counts — and lost counts corrupt odometry silently,
which is the worst way to lose data.

The cost is memory, proportional to `buffer_ms ÷ sample_us`. At 200 ms and 5 µs that is 40 000
samples: hundreds of kilobytes, not megabytes. Affordable on 512 MB. `-b 1000` would not be.

#### `-x 0x0BF53060` — restrict the daemon to our thirteen pins

The default lets pigpiod touch every user GPIO. This mask permits exactly the pins this car
uses and nothing else:

| GPIO | Function | GPIO | Function |
|------|----------|------|----------|
| 5  | IN1 | 21 | IN4 |
| 6  | IN2 | 22 | ENC_R_B |
| 12 | ENA | 23 | ENC_L_A |
| 13 | ENB | 24 | ENC_L_B |
| 16 | E-stop | 25 | Status LED |
| 18 | Servo | 27 | ENC_R_A |
| 20 | IN3 | | |

```python
>>> hex(sum(1 << p for p in (5,6,12,13,16,18,20,21,22,23,24,25,27)))
'0xbf53060'
```

(The leading zero in `0x0BF53060` is cosmetic — it just pads the mask to eight digits so it
lines up with the 32-bit field it is. pigpiod accepts either spelling.)

With the mask in place, a typo in a `pigs` command — and you will make one, at eleven at night,
with the car on the bench — cannot drive the SD card lines, the camera's I²C, or the ID EEPROM
pins. It fails with an error instead of doing something exciting.

Keep this mask in sync with [wiring.md §11](wiring.md) if the pinout ever changes.

### The pigpiod unit

The repo ships these flags as a **drop-in override**, not as a replacement unit:
[`pi/systemd/pigpiod.service.d/override.conf`](../pi/systemd/pigpiod.service.d/override.conf).

```bash
sudo mkdir -p /etc/systemd/system/pigpiod.service.d
sudo cp /home/pi/telekart/pi/systemd/pigpiod.service.d/override.conf \
        /etc/systemd/system/pigpiod.service.d/override.conf
sudo systemctl daemon-reload
sudo systemctl enable --now pigpiod
pgrep -a pigpiod          # confirm the flags are the ones above
```

Two things about that file worth knowing before you edit it:

- **The bare `ExecStart=` on the line before the real one is mandatory and is not a typo.** In
  a drop-in, a directive that may appear more than once *appends*. Without the reset you add a
  *second* `ExecStart` to whatever the packaged unit had, and systemd refuses to start a
  `Type=forking` unit with two of them.
- It also sets `ExecStop=/bin/systemctl kill -s SIGKILL pigpiod.service`, because pigpiod does
  not shut down cleanly on `SIGTERM` in all versions, and a daemon stuck in shutdown blocks the
  stop transition of everything ordered after it.

The daemon must outlive the firmware's stop transition so that `ExecStopPost=` in
`telekart-control.service` still has something to talk to. That ordering comes from
`Requires=`/`After=` on the control unit (§7.2), not from this file.

> **Stopping pigpiod does not safe the pins.** `pigpiod` retains GPIO state after its client
> dies, and killing the daemon itself is no different — the pin levels persist until something
> changes them or the Pi loses power. This is *the* settled fact that shapes the whole
> panic-stop design ([INTERFACES.md §8](INTERFACES.md)), and §7 below is where it is handled.

---

## 4. picamera2, and why the venv needs `--system-site-packages`

### picamera2 must come from apt

```bash
sudo apt install -y python3-picamera2 --no-install-recommends
```

**Not pip.** `pip install picamera2` will get you the Python package and then fail to import,
because picamera2 depends on the **`libcamera` Python bindings**, which:

- are a **C++ extension** built against the exact `libcamera` version on your image,
- are **not published on PyPI** in any usable form for the Pi, and
- ship as the apt package `python3-libcamera`, alongside `python3-kms++`.

There is no pip route to a working camera on this platform. This is not a preference.

`--no-install-recommends` matters on a 512 MB Lite image: the recommends pull in the GUI
preview stack (PyQt5, `python3-opengl`, and friends) — roughly 200 MB of packages for a
preview window that will never open on a headless car.

### Hardware H.264

The BCM2710 retains the legacy hardware H.264 encoder, exposed through the `bcm2835-codec`
driver as `/dev/video11`, and picamera2's `H264Encoder` uses it. This is why 640×480@30
encoding costs a few percent of one core rather than saturating it — and it is why
[tuning.md](tuning.md) can talk about latency budgets in the tens of milliseconds instead of
hundreds.

```bash
ls -l /dev/video11        # must exist
```

### The venv

Because the camera stack lives in the system site-packages and cannot be reinstalled into a
venv, the venv has to be able to see it:

```bash
cd /home/pi/telekart
make setup-pi                  # creates pi/.venv with --system-site-packages
```

`make setup-pi` is the supported route and it is what the systemd units in §7 point at. It
creates the firmware venv at **`pi/.venv`**, beside the package it installs — `app/`, `pi/`
and `tools/` deliberately keep three separate environments on three different interpreters
(see the table in [README.md](../README.md#development)). By hand it is:

```bash
cd /home/pi/telekart
python3 -m venv --system-site-packages pi/.venv
pi/.venv/bin/pip install --upgrade pip
pi/.venv/bin/pip install -e packages/telekart_protocol
pi/.venv/bin/pip install -e 'pi[rpi]'      # the [rpi] extra is the pigpio client
```

`--system-site-packages` lets the venv import `libcamera`, `picamera2`, `python3-kms++` and
apt's `numpy` (compiled for aarch64) while still giving you an isolated place for the
project's own pure-Python dependencies.

**One trap that comes with it:** venv packages shadow system ones. Never `pip install numpy`
into this venv — you will shadow apt's build with a generic wheel, and picamera2 will hand
buffers to a numpy that was compiled against different headers. If `pip list` shows numpy
twice, you have already done it.

| Comes from **apt** (system) | Comes from **pip** (venv) |
|---|---|
| `python3-picamera2` | `telekart_protocol` (editable, from this repo) |
| `python3-libcamera` | `pigpio` (pure Python socket client) |
| `python3-kms++` | the firmware's own pure-Python deps |
| `python3-numpy` | |

---

## 5. WiFi power save

**Turn it off.** The Zero 2 W's radio enters power-save mode by default, in which the access
point buffers downlink frames until the next beacon. With a typical 100 ms beacon interval
that is up to **100 ms of added latency, in bursts** — landing squarely inside the 200 ms
`CONTROL_TIMEOUT_MS` window and producing failsafe trips that look like a firmware bug.

Bookworm uses NetworkManager. Make it permanent:

```bash
sudo tee /etc/NetworkManager/conf.d/99-wifi-powersave-off.conf >/dev/null <<'EOF'
[connection]
wifi.powersave = 2
EOF
sudo systemctl restart NetworkManager
```

`2` means disabled. (`3` means enabled; `0` means "use the default", which is not what you
want.)

Verify:

```bash
iw dev wlan0 get power_save      # must print: Power save: off
```

### The belt-and-braces unit

The repo also ships
[`pi/systemd/telekart-wifi-nopowersave.service`](../pi/systemd/telekart-wifi-nopowersave.service),
a `Type=oneshot` unit that runs `iw dev wlan0 set power_save off` after NetworkManager on
every boot. The NetworkManager config file above is the durable setting; this unit covers the
case where the connection profile was created *before* that file existed, or where the
interface came up under something other than NetworkManager.

```bash
sudo cp /home/pi/telekart/pi/systemd/telekart-wifi-nopowersave.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now telekart-wifi-nopowersave
systemctl status telekart-wifi-nopowersave   # ExecStartPost logs the resulting state
```

Every `ExecStart` line in it is `-`-prefixed, so a missing `iw` or an absent interface cannot
fail the boot: a car with slightly worse latency still drives, a car that refused to boot does
not.

### Other radio hygiene

- Confirm the regulatory domain is set: `iw reg get` should show your country, not `00`.
- Keep the car and the laptop on the **same AP**, not roaming between two.
- 2.4 GHz only, fixed channel, one of 1/6/11. See §1.

---

## 6. Time, and why you should not worry about it

The Zero 2 W has no real-time clock. `fake-hwclock` restores a plausible time at boot and NTP
corrects it once the network is up.

**This does not affect the control path at all.** Every timestamp that matters is
`CLOCK_MONOTONIC`:

- `car_time_us` and `echo_client_time_us` in the telemetry packet,
- `pts_us` in the video frame header,
- everything behind the injected `Clock` in [INTERFACES.md §1](INTERFACES.md).

Wall-clock time affects log timestamps and the `measured_at` string in
`DriveCalibration`. That is all. Do not go chasing an RTC module to fix a latency problem;
it will not.

---

## 7. systemd

Two units, because the two processes have genuinely different requirements. `telekart-control`
owns motion and networking; `telekart-video` owns the camera. They are separate processes
because the GIL is real — camera work must not inject jitter into the 100 Hz loop — and
because the OOM policy has to be able to guarantee that the kernel kills the camera and never
the car ([INTERFACES.md §8](INTERFACES.md)).

### 7.1 The panic-stop script

This is **layer 2 of the four-layer panic-stop chain, and the only layer that covers
`SIGKILL`.** systemd runs `ExecStopPost=` on every stop path — clean stop, crash, restart,
`kill -9`, OOM kill.

The script ships in the repo as
[`pi/scripts/panic_stop.sh`](../pi/scripts/panic_stop.sh). **Install that file — do not
retype it.** It is deliberately POSIX shell with no Python, no venv and no imports, because
the situation it exists for is the one where Python is the thing that died; it talks to
pigpiod through `pigs`, a small C binary.

```bash
sudo install -m 0755 /home/pi/telekart/pi/scripts/panic_stop.sh \
     /usr/local/sbin/telekart-panic-stop
sudo /usr/local/sbin/telekart-panic-stop && echo ok      # safe to run any time
```

The one thing to understand before you read it:

> **The order is the opposite of intuitive.** The enables come down **first**, which coasts
> the bridge. On an L298 with EN high, `IN1 == IN2` — both high *or* both low — is a **brake**.
> Clearing the direction pins while an enable is still high therefore walks the outputs
> through a hard short across a spinning motor, at the exact moment you were trying to make
> things safe. Coast requires EN LOW and nothing else does
> ([wiring.md §8](wiring.md), [INTERFACES.md §12](INTERFACES.md)).

It also stops the servo pulse train (`pigs s 18 0`, so the HS-311 goes limp instead of holding
against the SoC's own 5 V rail), extinguishes the status LED, and **always exits 0** — a
non-zero `ExecStopPost=` puts the unit into a failed state, and a failed unit does not
restart. The outputs are already low by then; the only thing a non-zero exit could still
achieve is preventing the car from coming back.

### 7.2 `telekart-control.service`

**The unit ships in the repo, fully commented, at
[`pi/systemd/telekart-control.service`](../pi/systemd/telekart-control.service). Install that
file (§7.4) rather than transcribing one from here** — a second copy in a document is a second
copy to drift. What follows is why the directives that surprise people are what they are.

| Directive | Value | Why |
|---|---|---|
| `Requires=pigpiod.service` | — | Not merely `After=`. Without pigpiod the firmware cannot touch a pin and must not start pretending otherwise. It also gives the right **stop** ordering: `systemctl stop pigpiod` stops this unit first, so `ExecStopPost` still has a live daemon to send `pigs` to. |
| `Type=` | **`notify`** | Not `simple`. `WatchdogSec` is only fed by `sd_notify(WATCHDOG=1)`, and with `Type=simple` systemd sets `NotifyAccess=none` and discards every ping — you would have a watchdog line in the unit and no watchdog. |
| `WatchdogSec` | `3` | 3 s against a 10 ms loop is 300 missed deadlines: nothing survivable trips it. The pings come from the **control thread**, not the asyncio loop — a watchdog fed by the event loop stays happy while the control loop is wedged at 60 % duty, which is the exact failure it exists to catch. |
| `ExecStopPost=` | `-/usr/local/sbin/telekart-panic-stop` | Layer 2 of the panic chain, and **the only layer that covers `SIGKILL`**. Note the leading `-`: a failure here must never block the stop transition or the restart that follows it. |
| `TimeoutStopSec` | `5` | If the process wedges, systemd escalates to `SIGKILL` after five seconds — and `ExecStopPost` still runs. That is the design, not a fallback. |
| `Restart=` | **`always`** | Not `on-failure`. This process has no legitimate reason to stop while the vehicle is powered, so a *clean* exit is itself a symptom. `StartLimitBurst=10` over `StartLimitIntervalSec=60` stops a boot-time fault hiding behind an infinite restart loop. |
| `CPUSchedulingPolicy` | **`fifo`**, priority `10` | The process starts at FIFO 10 and the control thread raises *itself* to `RT_PRIORITY_CONTROL` (50, see [`constants.py`](../pi/telekart/constants.py)), so the loop outranks its own networking thread while both outrank everything on CFS. 10 is deliberately low because the asyncio thread inherits it; the kernel's RT throttle is the backstop. |
| `LimitRTPRIO` | **`99`** | This is the ceiling on what `sched_setscheduler` may request. It must be **at or above `RT_PRIORITY_CONTROL` (50)** or the control thread's own promotion fails with `EPERM` and you silently lose real-time scheduling. Anything below 50 is a bug. |
| `CPUAffinity` | `2-3` | Leaves cores 0–1 for kernel IRQ work, the WiFi stack and the camera process. `CONTROL_CPU_AFFINITY` in `constants.py` pins the control thread further, to core 3 alone, leaving core 2 for asyncio. |
| `OOMScoreAdjust` | `-500` | The unit's value applies from `exec`, covering the window before Python has run a line; the process then lowers itself to `OOM_SCORE_CONTROL` (−900) using `CAP_SYS_RESOURCE`. Both, deliberately. |
| `AmbientCapabilities` | `CAP_SYS_NICE CAP_IPC_LOCK CAP_SYS_RESOURCE` | `sched_setscheduler`, `mlockall`, and lowering `oom_score_adj` respectively. Redundant while `User=root`, and set anyway so moving to an unprivileged user later does not silently turn real-time scheduling off. |
| `EnvironmentFile=` | `-/etc/telekart/telekart.env` | 0600, root-owned, not in git — how `TELEKART_SHARED_KEY` stays out of the repository and out of `systemctl show`. The `-` makes it optional, so a car with the key in `config.local.yaml` still starts. |

### 7.3 `telekart-video.service`

Ships at [`pi/systemd/telekart-video.service`](../pi/systemd/telekart-video.service). Same
rule: install the file.

| Directive | Value | Why |
|---|---|---|
| `MemoryMax` / `MemoryHigh` | `220M` / `180M` | About three times a steady-state 640×480 H.264 pipeline: room for a resolution bump, no room for a runaway. With `OOMPolicy=continue`, a leak kills **this** process and systemd restarts it while the control loop — which is steering a moving vehicle — never notices. Without the cap the two processes compete for the same 512 MB and the OOM killer chooses by heuristic. |
| `OOMScoreAdjust` | `500` | `OOM_SCORE_VIDEO`. The counterpart to control's negative score: this is the process the kernel is meant to take. |
| `CPUAffinity` | `0-1` | The two cores the control unit does not use. Camera work must not be able to inject jitter into a 10 ms deadline — that is the entire reason there are two processes. |
| `Nice` | `5` | Explicitly below the control loop. |

Note what is deliberately **absent**:

- No `ExecStopPost`. This process touches no GPIO; there is nothing to safe.
- No `Requires=pigpiod`. Same reason.
- `After=telekart-control.service` but **not** `Requires=`. Video failing must never take the
  car's control process down with it. A car with no picture is recoverable; a car with no
  control loop is not.
- No `DeviceAllow=`. Setting even one entry flips the unit's device policy from `auto` to
  `closed`, and the libcamera stack reaches for `/dev/video*`, `/dev/media*`,
  `/dev/dma_heap/*` and `/dev/dri/*` — an incomplete list presents as "the camera does not
  work" with no useful error. The camera is sandboxed by memory and CPU instead, which is what
  actually protects the control loop.

### 7.4 Install

Everything `pi/systemd/` ships, in one block. Four files: two units, one drop-in for the
packaged `pigpiod.service` (§3), and the WiFi power-save oneshot (§5).

```bash
cd /home/pi/telekart/pi/systemd
sudo cp telekart-control.service telekart-video.service \
        telekart-wifi-nopowersave.service /etc/systemd/system/
sudo mkdir -p /etc/systemd/system/pigpiod.service.d
sudo cp pigpiod.service.d/override.conf /etc/systemd/system/pigpiod.service.d/

sudo systemctl daemon-reload
sudo systemctl enable telekart-control telekart-video telekart-wifi-nopowersave
```

The units reference `/home/pi/telekart/pi/.venv/bin/python`, which is what `make setup-pi`
creates (§4). If your checkout or username differs, edit `WorkingDirectory=`, `PYTHONPATH=`
and `ExecStart=` to match — those three paths and nothing else.

```bash
systemd-analyze verify /etc/systemd/system/telekart-control.service   # catches typos
```

Do **not** start them yet. [bringup.md](bringup.md) runs the subsystems by hand first, in
order, with gates. Starting the full stack against untested wiring is how you find out what
your master switch is for.

### 7.5 mDNS

The desktop app discovers the car through `_telekart._tcp.local.`
(see [`constants.py`](../packages/telekart_protocol/telekart_protocol/constants.py)), which
needs Avahi:

```bash
systemctl is-active avahi-daemon        # 'active' — Bookworm Lite ships it enabled
avahi-browse -rt _telekart._tcp         # once the firmware is running
```

---

## 8. SD card corruption — this car gets its power yanked

Assume it. The powerbank runs out, a USB cable works loose over a kerb, or you pull the plug
because the car is heading for a wall. None of those give the filesystem a chance to flush.

Five mitigations, in the order you should apply them.

### 8.1 Logs to RAM

The single biggest source of writes on an otherwise idle Pi.

`/etc/fstab`:

```
tmpfs  /var/log  tmpfs  defaults,noatime,nosuid,mode=0755,size=32M  0  0
```

`/etc/systemd/journald.conf.d/volatile.conf`:

```ini
[Journal]
Storage=volatile
RuntimeMaxUse=16M
```

**The trade is explicit: logs do not survive a reboot.** Watch them live with
`journalctl -fu telekart-control`, or `rsync` them off before you reboot. If you need
persistence for one specific investigation, set `Storage=auto` temporarily and put it back
afterwards.

### 8.2 `noatime` and a longer commit interval

`/etc/fstab`, on the root entry:

```
PARTUUID=xxxxxxxx-02  /  ext4  defaults,noatime,commit=600  0  1
```

- **`noatime`** removes one metadata write per file *read*. A Python process importing a few
  hundred modules at startup does a lot of reads.
- **`commit=600`** batches ext4 journal commits to every ten minutes instead of every five
  seconds. Far fewer writes, and — more importantly — far fewer partially-completed journal
  transactions sitting on the card at the moment power disappears.

The cost is up to ten minutes of un-flushed data lost on a hard cut. Here that is close to
nothing: the logs are already in RAM, and the only file the car writes in normal operation is
`calibration.yaml`.

> **Note for the firmware workstream:** with `commit=600` in force, a freshly written
> `calibration.yaml` can vanish on a power cut unless the writer `fsync()`s **both the file and
> its containing directory**. `DriveCalibration.save()` ([INTERFACES.md §6](INTERFACES.md))
> should write to a temp file, fsync it, rename, then fsync the directory.

### 8.3 No swap

```bash
sudo systemctl disable --now dphys-swapfile
sudo apt purge -y dphys-swapfile
```

On 512 MB, swapping to an SD card is worse than the alternative in every dimension: it
hammers the card, and it introduces multi-hundred-millisecond stalls into a process that is
supposed to hit a 10 ms deadline. The `OOMScoreAdjust` settings in §7 exist precisely so that
when memory runs out, the kernel makes the right choice quickly instead of the machine dying
slowly.

### 8.4 Read-only root — the escalation

If you keep corrupting cards despite the above:

```bash
sudo raspi-config     # Performance Options -> Overlay File System -> enable
                      # also set the boot partition read-only
```

Everything then writes to a RAM overlay that is discarded at reboot. It is close to
bulletproof.

> **The gotcha that will catch you:** with the overlay enabled, **`calibration.yaml` will not
> persist.** Neither will any config change, SSH key, or `apt install`. You must disable the
> overlay, reboot, make your change, re-enable, reboot. That is why this is the escalation and
> not the default.

### 8.5 Habits

- `sudo shutdown -h now`, wait for the green LED to stop flickering, *then* pull power. Ten
  seconds.
- **`dd` an image of the card the moment the build works.** Restoring a known-good image is a
  five-minute fix. Rebuilding from this document is an evening.

---

## 9. CPU governor — measure before you change it

Bookworm defaults to the `ondemand` governor. Switching to `performance` removes the
frequency-ramp latency at the start of each control tick, at the cost of roughly 50 mA of
continuous draw and a warmer SoC.

On a board that is already supply-constrained ([power.md](power.md)), that is not a free
trade. **Leave it at `ondemand` for bring-up.** Measure loop p99 in [bringup.md](bringup.md)
Phase 5. Only if p99 is marginal against the 12 ms gate:

```bash
sudo apt install -y cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl restart cpufrequtils
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

Then re-measure `vcgencmd get_throttled` and the loop p99, and keep whichever combination
actually passes.

---

## 10. Pre-flight checklist

Everything in this document, in one block. All of it must pass before
[bringup.md](bringup.md) Phase 1.

```bash
uname -m                                    # aarch64
python3 -V                                  # 3.11.x
vcgencmd get_throttled                      # 0x0
lsmod | grep snd_bcm2835                    # (no output)
grep -c 'dtparam=audio=off' /boot/firmware/config.txt          # 1
grep -cE 'i2s|hifiberry|iqaudio|audioinjector|googlevoicehat' \
     /boot/firmware/config.txt              # 0
pinctrl get 5,6,12,13,18,20,21              # no PCM_* / PWM* functions
/usr/local/bin/pigpiod -v                   # 79
pgrep -a pigpiod                            # -t 1 -s 5 -l -b 200 -x 0x0BF53060
pigs t                                      # a tick; run twice, it must increase
iw dev wlan0 get power_save                 # Power save: off
ls -l /dev/video11                          # exists
rpicam-hello --list-cameras                 # imx219
systemctl is-active avahi-daemon            # active
swapon --show                               # (no output)
findmnt -no OPTIONS /var/log                # tmpfs options
findmnt -no OPTIONS /  | grep -o noatime    # noatime
systemctl is-enabled telekart-control telekart-video   # enabled, enabled
test -x /usr/local/sbin/telekart-panic-stop && echo ok  # ok
```
