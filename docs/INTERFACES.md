# TeleKart Internal Interface Contract

**This document is normative.** Every module in this repository is written against it.
If an implementation and this document disagree, this document wins and the implementation
is a bug. Change this file first, then the code.

It exists because the codebase is built in parallel: several independent workstreams write
modules that must fit together on first assembly. The shared wire format lives in
`packages/telekart_protocol/` (read the actual source — it is the contract for anything that
crosses the network). This file covers the *internal* Python interfaces that do not.

---

## 0. Conventions

**Units.** SI everywhere internally, without exception:

| Quantity | Unit | Notes |
|---|---|---|
| Distance | metres | |
| Speed | m/s | |
| Angle | radians | except servo pulse widths (µs) and user-facing degrees |
| Time | seconds (float) | `*_us` / `*_ms` suffixes mark integer exceptions |
| Motor speed | RPM | the natural unit for a gearmotor; converted at the boundary |
| Duty cycle | float −1.0 … +1.0 | sign is direction |
| Normalized input | float −1.0 … +1.0 (steer), 0.0 … 1.0 (throttle/brake) | |

Conversion to the wire's scaled integers happens **only** in `telekart_protocol`. No other
module multiplies by 1000.

**Naming.** `snake_case` for functions and variables, `PascalCase` for classes. A leading
underscore means private and unstable. Suffix `_us`, `_ms`, `_mm` only when the value is an
integer in that unit.

**Typing.** Every public function is fully annotated. `from __future__ import annotations`
at the top of every module. Target **Python 3.11** for `pi/` and `packages/`; **Python 3.12**
for `app/`. No PEP 695 generics anywhere.

**Errors.** Fail loudly at construction and configuration time; never raise inside a control
loop or a packet-decode path. Hot paths clamp, drop, and set a fault flag instead.

**No hot-path allocation.** Anything called at 100 Hz uses `__slots__`, preallocated buffers,
and no f-strings.

---

## 1. `pi/telekart/util/clock.py`

The single most important testability decision in the firmware: **no module ever calls
`time.monotonic()` directly.** A `Clock` is constructor-injected everywhere. `FakeClock`
then makes the whole suite deterministic and instantaneous — a simulated 60-second drive
test runs in milliseconds and gives the same answer every time.

```python
class Clock(Protocol):
    def monotonic(self) -> float: ...
    def monotonic_us(self) -> int: ...
    def sleep(self, seconds: float) -> None: ...

class RealClock(Clock):
    """time.monotonic + time.sleep."""

class FakeClock(Clock):
    def __init__(self, start: float = 0.0) -> None: ...
    def advance(self, seconds: float) -> None: ...
    # sleep() advances virtual time instead of blocking.
```

Also in this module:

```python
class DeadlineScheduler:
    """Fixed-period loop pacing with absolute deadlines, so period error never
    accumulates. Reports overrun rather than silently drifting."""
    def __init__(self, clock: Clock, period: float) -> None: ...
    def start(self) -> None: ...
    def wait_next(self) -> float:
        """Sleep until the next deadline. Returns actual elapsed dt (seconds).
        On overrun, skips missed deadlines and increments `overruns`."""
    overruns: int
    stats: "JitterStats"

class JitterStats:
    """Streaming period statistics. p99 is the number that matters — a 10 ms
    loop with p99 > 12 ms is failing regardless of its mean."""
    def add(self, dt: float) -> None: ...
    def snapshot(self) -> JitterSnapshot: ...   # p50/p95/p99/max/count, in seconds
    def reset(self) -> None: ...
```

---

## 2. `pi/telekart/hal/base.py`

Every hardware access in the firmware goes through this one interface. It is what makes
`MockGPIO` (and therefore Mac-side development of ~60% of the firmware) possible, and what
makes swapping pigpio for sysfs PWM a one-file change.

```python
class Pull(enum.Enum):
    NONE, UP, DOWN

class Edge(enum.Enum):
    RISING, FALLING, BOTH

EdgeCallback = Callable[[int, int, int], None]   # (pin, level, tick_us) — pigpio's shape

class CallbackHandle(Protocol):
    def cancel(self) -> None: ...

class GpioBackend(ABC):
    # --- digital ---
    def setup_output(self, pin: int, initial: bool = False) -> None: ...
    def write(self, pin: int, value: bool) -> None: ...
    def setup_input(self, pin: int, pull: Pull = Pull.NONE, glitch_us: int = 0) -> None: ...
    def read(self, pin: int) -> bool: ...

    # --- PWM ---
    def set_pwm_pair(self, pin_a: int, pin_b: int, freq_hz: int,
                     duty_a: float, duty_b: float) -> None:
        """Set BOTH hardware PWM channels at once. Duties are 0.0..1.0.

        This is a pair operation and not two calls BY DESIGN. On BCM283x the two
        hardware PWM channels share a single clock divider, so writing GPIO12 at
        1 kHz and then GPIO13 at 4 kHz corrupts channel 0. Making the pair the
        only available operation renders that mistake unrepresentable."""

    # --- servo ---
    def set_servo_pulse(self, pin: int, pulse_us: int) -> None:
        """DMA-timed servo pulse. pulse_us == 0 stops the pulse train, which
        lets the servo go limp — that is the correct disarmed state here,
        because the servo shares the Pi's 5 V rail with the SoC."""

    # --- edges ---
    def add_edge_callback(self, pin: int, edge: Edge, callback: EdgeCallback) -> CallbackHandle: ...

    # --- misc ---
    def ticks_us(self) -> int:
        """Backend microsecond tick, same time base as the callback's tick_us.
        Wraps at 2**32 like pigpio's; use `tick_diff` to subtract."""
    def cleanup(self) -> None: ...

def tick_diff(earlier: int, later: int) -> int:
    """Wrap-safe difference of two 32-bit µs ticks, in µs."""
```

Backends: `PigpioBackend` (`hal/pigpio_backend.py`), `MockBackend` (`hal/mock_backend.py`).

`MockBackend` carries a **plant model** so the control loop genuinely closes in simulation:
first-order motor lag (τ ≈ 0.15 s), a stiction deadband, voltage sag under combined load, and
synthesized encoder edges delivered to registered callbacks. It advances in lockstep with a
`FakeClock` via `MockBackend.step(dt)`.

```python
def select_backend(name: str = "auto", **kwargs) -> GpioBackend:
    """'auto' -> pigpio on aarch64/armv7l when importable, else mock.
    Honours the TELEKART_BACKEND env var."""
```

---

## 3. `pi/telekart/drivers/`

### `motor.py`

```python
class MotorPair:
    """Both motors together, because the PWM channels are physically coupled."""
    def __init__(self, gpio: GpioBackend, pins: MotorPins, config: VehicleConfig,
                 clock: Clock) -> None: ...

    def drive(self, duty_l: float, duty_r: float) -> None:
        """Duties in −1..+1. Applies inversion, the combined-duty budget, the
        per-motor clamp, and direction dead-time sequencing."""
    def brake(self, strength: float) -> None:
        """IN pair EQUAL + EN at `strength`.

        NOTE THE TRUTH TABLE. On the L298, with EN high, IN1 == IN2 (both high
        OR both low) is a BRAKE. Coast requires EN LOW. This is the opposite of
        most people's intuition and getting it backwards shorts a spinning
        motor every time you meant to coast."""
    def coast(self) -> None:
        """EN duty = 0 on both. Outputs high-Z."""
    def panic_stop(self) -> None:
        """Idempotent, allocation-free, safe to call from a signal handler."""

    limiter_active: bool   # True when a protection clamp modified the command
```

Direction changes are sequenced: require `|rpm| < reverse_allowed_rpm`, then duty→0, wait
`direction_deadtime_ms`, flip the IN pins, ramp back. Never flip IN pins under a live enable.

### `servo.py`

```python
class SteeringServo:
    def __init__(self, gpio: GpioBackend, pin: int, config: VehicleConfig, clock: Clock) -> None: ...
    def set_angle(self, radians: float) -> None:      # slew-limited; clamped to config limits
    def set_normalized(self, value: float) -> None:   # −1..+1 mapped through centre/limits
    def set_pulse_us(self, us: int) -> None:          # raw, still hard-clamped. Calibration only.
    def relax(self) -> None:                          # stop pulses -> limp, ~zero holding current
    def center(self) -> None:
    def update(self, dt: float) -> None:              # called every control tick; applies slew
    @property
    def applied_us(self) -> int: ...
    @property
    def applied_angle(self) -> float: ...             # radians, after slew and clamping
```

Never writes a pulse that differs from the last by less than `steer_hold_us` — that deadband
is what stops the servo buzzing, and it is carried over from the old firmware deliberately.

### `encoder.py`

```python
class QuadratureEncoder:
    """x2 decoding: EITHER_EDGE on channel A only, direction from the commanded
    H-bridge state. Channel B is wired and polled at loop rate purely as a
    disagreement check.

    Why not x1: 'callback on A rising, read B for direction' costs a socket
    round-trip to pigpiod (~50–100 µs) per edge — 100–200 % of a core. Caching B
    with a second EITHER_EDGE callback fires at 4× the A-rising rate, i.e. the
    same event count as full x4. There is no cheap route to true quadrature
    direction, so we buy resolution (660 cpr) and infer sign.

    The callback body does nothing but `count += 1; last_tick = tick`."""

    def __init__(self, gpio: GpioBackend, pin_a: int, pin_b: int, *,
                 cpr: int, invert: bool = False, glitch_us: int = 30) -> None: ...
    def set_direction_hint(self, sign: int) -> None:   # −1, 0, +1 from the commanded state
    def sample(self) -> EncoderSample: ...             # called once per control tick
    def reset(self) -> None: ...
    @property
    def total_counts(self) -> int: ...
    @property
    def direction_uncertain(self) -> bool: ...

@dataclass(frozen=True, slots=True)
class EncoderSample:
    counts: int          # signed delta since the previous sample
    total: int
    rpm: float           # M/T estimate, filtered
    raw_rpm: float
    edges: int
    stale: bool          # no edges for longer than the stall window

class MTVelocity:
    """M/T velocity estimation. The measurement window closes on an EDGE, not on
    the loop tick, which is why the backend must expose µs ticks.

    Naive counts-per-tick is unusable at low speed: at 100 Hz and 660 cpr, 20 RPM
    is 2.2 counts/tick — ±45 % quantization. The resulting 25 ms output filter
    caps closed-loop bandwidth at roughly 7 Hz; do not try to tune past it."""
```

---

## 4. `pi/telekart/control/`

### `pid.py`

```python
class PID:
    def __init__(self, kp: float, ki: float, kd: float, *,
                 i_clamp: float = 0.4, output_limits: tuple[float, float] = (-1.0, 1.0),
                 derivative_lpf_hz: float = 10.0) -> None: ...
    def update(self, setpoint: float, measured: float, dt: float, *,
               feedforward: float = 0.0) -> float: ...
    def reset(self) -> None: ...
    def set_gains(self, kp: float, ki: float, kd: float) -> None: ...
    saturated: bool
```

Conditional-integration anti-windup: freeze the integrator whenever the output is saturated,
and clamp so `feedforward + integral` stays inside the output limits. `reset()` is called on
arm, on direction change, and on disarm.

### `shaping.py` — pure functions, no state, exhaustively unit-tested

```python
def deadzone(value: float, dz: float) -> float          # rescales so output stays continuous
def expo(value: float, gamma: float) -> float           # sign-preserving
def rate_limit(target: float, current: float, max_rate: float, dt: float) -> float
def clamp(value: float, lo: float, hi: float) -> float
def lerp(a: float, b: float, t: float) -> float
def wrap_pi(angle: float) -> float
def speed_sensitive_scale(speed_frac: float, max_reduction: float) -> float
```

### `mixer.py`

```python
@dataclass(frozen=True, slots=True)
class WheelTargets:
    rpm_l: float
    rpm_r: float

class DifferentialMixer:
    """Electronic differential. NOT optional.

    Two motors on a rear axle with Ackermann front steering and no mechanical
    diff: in a turn the rear wheels MUST run at different speeds. Per-wheel PID
    forcing them equal fights the geometry and scrubs the tyres.

        split = track_width * tan(delta) / (2 * wheelbase)

    which is ±16.7 % at full lock with the default geometry — a 33 % spread
    between wheels, not a rounding error."""
    def mix(self, target_rpm: float, steer_angle: float) -> WheelTargets: ...
```

### `safety.py`

```python
class SafetyStateMachine:
    def __init__(self, config: VehicleConfig, clock: Clock) -> None: ...
    def request_arm(self) -> tuple[bool, str]: ...     # (accepted, reason_if_not)
    def request_disarm(self) -> None: ...
    def request_estop(self) -> None: ...
    def clear_estop(self) -> tuple[bool, str]: ...
    def clear_faults(self) -> None: ...
    def raise_fault(self, fault: Fault, detail: str = "") -> None: ...
    def note_control_packet(self) -> None: ...          # resets the staleness timer
    def update(self, dt: float, *, throttle: float, measured: FeedbackSnapshot) -> SafetyOutput: ...

    @property
    def state(self) -> VehicleState: ...
    @property
    def faults(self) -> Fault: ...

@dataclass(frozen=True, slots=True)
class SafetyOutput:
    allow_drive: bool
    force_brake: float      # 0.0 = no forced brake
    force_coast: bool
    duty_ceiling: float
```

Arming requires **all four**: an explicit ARM, throttle at neutral for `arm_neutral_ms`, a
valid session, and no active critical fault.

Failsafe schedule from the moment the link goes stale — coast, then brake, then coast, then
disarm — per `FAILSAFE_*_AT_MS` in the protocol package. Coast-then-brake rather than coast
alone because an unbraked car keeps rolling for metres.

Stall detection is **hardware protection**, not a nicety: two stalled motors put ~7 W into a
bridge whose stock heatsink handles 2–3 W.

Brownout inference: both encoders reaching zero *simultaneously* while commanded is the boost
regulator tripping, not a mechanical stall. It gets its own fault code so the two are never
confused during diagnosis.

### `drive.py`

```python
class DriveController:
    """The 100 Hz loop body. Owns nothing it did not construct; everything is injected."""
    def __init__(self, *, gpio, config, clock, motors, servo, encoder_l, encoder_r,
                 safety, odometry, calibration) -> None: ...
    def submit_command(self, cmd: DriveCommand) -> None:   # thread-safe, single-slot
    def tick(self, dt: float) -> DriveState: ...           # one control iteration
    def panic_stop(self) -> None: ...

@dataclass(frozen=True, slots=True)
class DriveCommand:
    steering: float      # −1..+1
    throttle: float      # 0..1
    brake: float         # 0..1
    flags: ControlFlags
    received_at: float

@dataclass(frozen=True, slots=True)
class DriveState:
    """Everything the telemetry packet needs, in SI units."""
    rpm_l: float; rpm_r: float
    rpm_target_l: float; rpm_target_r: float
    duty_l: float; duty_r: float
    servo_us: int; steer_angle: float
    speed: float; v_max: float
    pose: tuple[float, float, float]
    distance: float; slip: float
    state: VehicleState; faults: Fault; flags: TelemetryFlags
```

Pipeline, in order: shaping → mixer → per-wheel `feedforward + PID` → deadband compensation →
combined-duty budget → clamp → dead-time sequencing → H-bridge.

**Feedforward carries the load; the PID only trims.** That is what makes the loop tunable at
all when the plant gain is ~2× uncertain.

---

## 5. `pi/telekart/odometry.py`

```python
class BicycleOdometry:
    def __init__(self, config: VehicleConfig) -> None: ...
    def update(self, d_left: float, d_right: float, steer_angle: float, dt: float) -> None: ...
    def reset(self) -> None: ...
    @property
    def pose(self) -> tuple[float, float, float]: ...   # x, y, heading
    @property
    def distance(self) -> float: ...
    @property
    def slip_index(self) -> float: ...
```

Midpoint (second-order) integration — meaningfully better than Euler and exactly as cheap:

```
d   = (d_left + d_right) / 2
δ   = first_order_lag(commanded_steer_angle, τ=0.10)   # HS-311 is ~0.19 s/60°;
                                                        # assuming instant steering is wrong
Δθ  = d * tan(δ) / wheelbase
x  += d * cos(θ + Δθ/2)
y  += d * sin(θ + Δθ/2)
θ   = wrap_pi(θ + Δθ)
```

`slip_index = |Δθ_from_encoders − Δθ_from_bicycle|` — free, and a genuinely good live
wheelspin / bad-calibration signal.

**Known limitation, stated up front:** with no IMU, heading drifts. Expect 5–15 % closure
error on a 5 m square. A `heading_source` hook exists so an MPU-6050 can be complementary-
filtered in later without restructuring anything.

---

## 6. `pi/telekart/calibration.py`

```python
@dataclass
class DriveCalibration:
    max_rpm: dict[str, float]          # {"left_fwd": .., "left_rev": .., "right_fwd": .., "right_rev": ..}
    deadband: dict[str, float]
    ff_lut: dict[str, list[tuple[float, float]]]   # (rpm, duty) breakpoints, monotonized
    measured_at: str
    on_ground: bool

    @property
    def max_rpm_measured(self) -> float: ...   # min across all four
    def feedforward(self, wheel: str, rpm: float) -> float: ...
    def save(self, path: Path) -> None: ...
    @classmethod
    def load(cls, path: Path) -> "DriveCalibration | None": ...
```

Written to `pi/config/calibration.yaml`, which is **a separate file from `config.yaml`** so
machine-written values never clobber hand-edited settings.

Everything downstream reads only `max_rpm_measured`; `v_max_mps = max_rpm/60 * π * wheel_diameter`
is published in every telemetry packet, and the desktop speedometer scales off it. That is
what lets the same build work with today's L298N-limited drivetrain and with a MOSFET bridge
later, with no code change on either side.

---

## 7. `pi/telekart/config.py`

```python
@dataclass
class MotorPins:   ena: int; in1: int; in2: int; in3: int; in4: int; enb: int
@dataclass
class EncoderPins: left_a: int; left_b: int; right_a: int; right_b: int
@dataclass
class HardwarePins:
    motors: MotorPins; encoders: EncoderPins
    servo: int; status_led: int | None; estop_button: int | None

@dataclass
class VehicleConfig:
    """Hardware pins + every tunable from telekart_protocol.params.

    Params are accessed as attributes (`config.max_duty`) and are generated from
    the shared registry, so adding a parameter there makes it available here and
    in the desktop tuning UI with no further work."""
    pins: HardwarePins
    car_id: str; shared_key: str
    # ... one attribute per entry in telekart_protocol.params.PARAMS

    @classmethod
    def load(cls, path: Path, local: Path | None = None) -> "VehicleConfig": ...
    def apply_params(self, values: dict[str, Any]) -> list[str]: ...   # returns changed names
    def as_params(self) -> dict[str, Any]: ...
    def save_local(self, path: Path) -> None: ...
```

**Pin defaults** (also in `pi/telekart/constants.py`, and these are physical facts, not
preferences):

```
ENA=12  IN1=5  IN2=6  IN3=20  IN4=21  ENB=13     # pre-wired by the user
SERVO=18                                          # GPIO12 is PWM0 and GPIO18 is ALSO PWM0,
                                                  # so hardware PWM is impossible here —
                                                  # DMA-timed pulses are the only option
ENC_L_A=23  ENC_L_B=24   ENC_R_A=27  ENC_R_B=22
STATUS_LED=25   ESTOP_BUTTON=16
```

---

## 8. `pi/telekart/app.py` — lifecycle and the panic-stop chain

Two processes. `telekart-control` owns motion and networking; `telekart-video` owns the
camera. Separate because the GIL is real: camera work must not inject jitter into the 100 Hz
loop, and the OOM policy must be able to guarantee the kernel kills the camera and never
the car.

Inside the control process: asyncio on the main thread (UDP, TCP session, watchdog pings);
one `ControlThread` (plain `threading.Thread`, SCHED_FIFO, absolute-deadline paced) running
the whole 100 Hz loop; and pigpio's own notification thread delivering encoder callbacks.

Inter-thread handoff is a **single-slot mailbox**, no queues and no locks: assigning one
attribute to an immutable dataclass reference is atomic under the GIL, and there is exactly
one writer and one reader in each direction. Document that reasoning at the site so nobody
later "fixes" it into a lock.

### The panic-stop chain — the most safety-critical part of the firmware

`pigpiod` retains GPIO state after its client dies. A segfault or `kill -9` at 80 % duty
leaves the motors running **indefinitely**. Four independent layers, because no single one
covers every case:

1. `hardware.panic_stop()` — idempotent and allocation-free; wired to `atexit`,
   `SIGTERM`/`SIGINT`/`SIGHUP`, `sys.excepthook`, `threading.excepthook`, and a context
   manager around `main()`.
2. `ExecStopPost=` in the systemd unit, running `pigs` directly. **This is the only layer
   that covers `SIGKILL`**, and systemd runs it on every stop path including crash-restart.
3. External 10 kΩ pull-downs on ENA/ENB/IN1–IN4 — covers Pi reboot, power loss, loose ribbon.
4. The battery-side switch — the human's E-stop, and the only one that still works when
   everything else is on fire.

---

## 9. Driving station — `app/telekart_ui/`

Superseded `app/telekart_app/`, which remains in the tree for reference only and is not
normative. One window: full-bleed video, a four-zone HUD, and a connect overlay. There is no
parameter UI, no settings screen, no diagnostics screen and no controller calibration — those
were removed deliberately, and §10 records what that cost.

### `core/latest_box.py`

```python
class LatestBox(Generic[T]):
    """Depth-1 cross-thread handoff with a generation counter. The backbone of
    the app's threading model.

    Producers overwrite; consumers take only when the generation changed. This
    is what avoids a Qt signal storm at 50 Hz × N subscribers, and it guarantees
    the UI renders one consistent snapshot per frame rather than a torn mix."""
    def put(self, value: T) -> None: ...
    def take(self) -> tuple[T, int] | None: ...   # None if unchanged since last take
    def peek(self) -> T | None: ...
    @property
    def generation(self) -> int: ...
```

### `core/paced_loop.py`

```python
class PacedLoop:
    """Drift-free perf_counter pacing for the input, TX, and video threads,
    with jitter statistics. Same absolute-deadline discipline as the firmware."""
```

### Threading model — normative

| Thread | Rate | Owns |
|---|---|---|
| Qt main | 60 Hz tick | all widgets; **also pumps SDL events** |
| `InputThread` | 250 Hz | reads cached SDL axes, runs the chain, writes a `LatestBox` |
| `ControlTxThread` | 100 Hz | encodes and sends UDP; owns the sequence counter and RTT ring |
| `TelemetryRxThread` | blocking recv | decodes, writes a `LatestBox` |
| `VideoRxThread` | 30 fps | TCP framing + PyAV decode → `LatestBox[FrameBundle]`, drop-oldest |
| `SessionClient` | event | TCP/JSON; low rate, so per-event Qt signals are fine here |

Plain `threading.Thread`, not `QThread`, for the hot paths: they want drift-free pacing, not
a thread-affine event loop. Cross-thread `Signal` emits to a GUI-thread receiver are
auto-queued and safe.

**The control stream must never depend on repaint timing.** A stalled UI yields a few
milliseconds of stale axis values while the TX thread keeps sending — which is the correct
failure mode, and specifically not a failsafe trip.

### `model/app_model.py`

```python
class AppModel(QObject):
    """The ONLY object widgets talk to. One 60 Hz QTimer drains every LatestBox,
    builds a consistent snapshot, and emits change signals — and only for values
    that actually changed (float comparisons use an epsilon).

    Because widgets never touch the network, the entire UI is testable by
    driving AppModel from a fixture with no sockets at all."""
    vehicleChanged = Signal(VehicleSnapshot)
    linkChanged   = Signal(LinkSnapshot)
    inputChanged  = Signal(InputSnapshot)
    sessionChanged = Signal(SessionSnapshot)
    faultRaised   = Signal(int, str)   # edge-triggered, one per newly-set bit
```

**Every field on a snapshot is rendered somewhere.** The telemetry packet carries more than
the snapshots do — per-wheel RPM and duty, odometry pose, distance, slip — and the station
shows none of it, so none of it is lifted. Adding a field back is one line; carrying twenty
that nothing paints is how the previous station reached a 27-field `LinkSnapshot` behind a
HUD that showed four numbers.

Four immutable snapshot dataclasses, replaced wholesale each tick — `VehicleSnapshot`,
`LinkSnapshot`, `InputSnapshot`, `SessionSnapshot`. Immutability is what prevents a widget
reading half of one update and half of the next.

`InputSnapshot` carries **what was actually sent**, echoed back from the TX thread, not what
the input thread read. A break anywhere in the chain then shows up on the HUD instead of
hiding.

### `input/chain.py` — a pure function, zero I/O, exhaustively tested

```
raw SDL float
 → calibrate   (rest..full → 0..1 for pedals; min..max → −1..+1 for steer; inversion)
 → deadzone    (centre for steer, floor for pedals; rescaled to stay continuous)
 → saturation  (top-end clip, rescaled, so 100 % is reachable without bottoming out)
 → curve       (256-entry precomputed LUT — a table lookup, never pow() at 250 Hz)
 → speed-sensitive steering scale     (optional assist; consumes telemetry speed)
 → rate limit  (separate rise/fall for pedals)
 → smoothing   (one-euro filter)
 → quantize    (steer ±1000, throttle/brake 0..1000)
```

**Division of responsibility, and it matters:** the app owns *feel* (curves, expo, deadzone,
smoothing). The firmware owns *protection* (duty slew clamp, servo rate limit, stall
detection, failsafe ramp). The app's rate limit must be **tighter than** the firmware's so
the firmware limiter never engages during normal driving — and the firmware sets
`TelemetryFlags.LIMITER_ACTIVE` so you can see it when it does.

**Never filter in both places.** An EMA in the app plus an EMA in the firmware is
second-order lag nobody can reason about. Firmware smoothing is a hard slew clamp only:
transparent when unsaturated, never a filter.

### `video/` — frame lifetime is a real crash risk

```python
@dataclass
class FrameBundle:
    """Owns the av.VideoFrame AND the QImage that views its buffer.

    Nothing else in the codebase may construct a QImage from a raw buffer.
    Wrapping a PyAV plane without holding the frame alive is a use-after-free
    that crashes intermittently and is miserable to diagnose — which is why the
    60-minute tracemalloc soak test is not optional."""
```

Decode settings are not negotiable:

```python
codec.flags |= av.codec.context.Flags.LOW_DELAY
codec.thread_type = 'NONE'     # NEVER 'FRAME' or 'AUTO'
codec.thread_count = 1
```

`thread_type='FRAME'` — which `AUTO` selects — buffers `thread_count` frames before emitting
anything: 3–5 frames, i.e. 100–170 ms. It is the single most common cause of "H.264 teleop
feels laggy" and has nothing to do with the Pi.

Conversion happens **on the decode thread** (`frame.reformat()` sized to the widget), so the
GUI thread does a 1:1 blit and nothing else.

### `ui/window.py` — keyboard driving

Four behaviours, each of which has shipped broken once and each of which has a test:

1. **Filter `isAutoRepeat()` on both press and release.** Qt synthesises a release/press pair
   per repeat of a held key; forwarding them makes a held throttle look like tapping to the
   rate limiter, which resolves to no throttle at all.
2. **Alias WASD onto the arrows at the event boundary**, because `ControlBinding` holds one
   ref per direction and both cannot be bound in the map.
3. **Forward only while the driving surface has the window**, and only keys the profile binds.
   Otherwise typing a passphrase steers the car and Space E-stops it mid-word.
4. **Release every key on `WindowDeactivate`.** Alt-tab with the throttle held and the window
   never sees the release; the app keeps transmitting full throttle, so no failsafe fires.

**Escape leaves fullscreen and nothing else.** It must never disarm: an accidental disarm at
speed leaves a car travelling with no drive and no steering authority.

### `ui/theme.py`

One palette and three fonts, consumed by **both** the ~40-line stylesheet and every
custom-painted zone. If QSS and `QPainter` keep separate palettes the app looks like two
applications stitched together.

`app.setStyle("Fusion")` before any widget exists: the macOS native style ignores much of QSS,
and Qt polishes widgets as they are constructed.

**No icons.** The previous station generated 41 SVG icons and scaled the painter by
`devicePixelRatio` while drawing onto a pixmap that already carried it — correct at dpr 1, a
magnified corner at dpr 2, blank at dpr 3. Text and painted shapes cannot have that bug.

---

## 10. Developing with no hardware

**There is no simulator.** `tools/telekart_sim/` was removed. To develop without a car, run
the real firmware on the development machine:

```sh
make run-car      # telekart-control --backend mock --defaults
make run-camera   # telekart-video --synthetic
```

`MockBackend` carries the plant model (first-order motor lag, a stall deadband below ~12 %
duty, voltage sag, encoder counts with ±1 noise, and RPM derived exactly the way the firmware
derives it). `telekart_video`'s synthetic source encodes through PyAV when it is importable,
so the stream is genuine H.264 inside the genuine 24-byte framing.

This is more faithful than the simulator was, because it *is* the firmware: the handshake,
the arming preconditions, the failsafe ladder and the telemetry packet are the shipping
implementations rather than a second one written to match.

**What was lost, and is not replaced.** The simulator could be made worse than reality on
demand — `--packet-loss`, `--latency`, `--jitter`, `--tcp-drop`, `--video-stall`,
`--encoder-fault`, `--reject-arm` — and `--seed` made a run bit-for-bit repeatable, which is
what let integration tests assert exact numbers instead of ranges. It was also a second,
independently written implementation of the protocol, and the two agreeing was the evidence
that this document was unambiguous. Reintroducing a link-impairment layer is the obvious way
to get the first half back; the second half needs a second implementation and nothing else.

---

## 11. Testing

| Layer | Approach |
|---|---|
| `packages/telekart_protocol` | Golden-byte tests, both directions, plus `struct.calcsize` assertions. **Run by both sides' suites** so a change that isn't reflected on both ends fails. |
| Firmware logic | `MockBackend` + `FakeClock`. Deterministic, instant, no hardware. |
| `input/chain.py` | Property tests: output always in range, monotonic in input, exact deadzone behaviour. |
| Keyboard | Every regression from `835ab3d` asserted: auto-repeat filtered on both edges, WASD aliased to the arrows, unbound keys not forwarded, all keys released on window deactivate, Space reaching the E-stop. |
| UI | `pytest-qt`, driving `AppModel` from five bare `LatestBox` objects — no sockets, no SDL, no car. HUD zones asserted against the letterboxed picture rect from 1024×640 to 3840×2160. |
| Soak | 60 minutes under `tracemalloc`, asserting stable RSS. Guards the `FrameBundle` lifetime. |

---

## 12. Things that are settled — do not re-litigate

- **Both hardware PWM channels share one clock divider.** Hence `set_pwm_pair`.
- **`dtparam=audio=off` is mandatory**; `snd_bcm2835` otherwise claims both PWM channels.
- **`IN1 == IN2` with EN high is a BRAKE.** Coast is EN low.
- **GPIO0–8 default to pull-UP**, so IN1 (GPIO5) and IN2 (GPIO6) idle HIGH at boot. External
  pull-downs, not software, are what makes the safe state deterministic.
- **x2 encoder decoding**, direction from command. x1 costs a socket round-trip per edge.
- **`pigpiod` outlives its client.** Four-layer panic stop.
- **PyAV must not use frame threading.**
- **Nothing hardcodes a top speed.** Everything scales off measured `max_rpm`.
- **The app never asserts it is armed** — it displays the state the car reports.
