"""The four immutable views the UI renders from.

Every one is frozen and replaced wholesale. That is not stylistic: a widget that
reads ``speed`` and then ``rpm_l`` off a mutable object can be preempted between
the two and paint a speedometer from packet N next to an RPM bar from packet
N+1. With a frozen object the widget holds one instant, and the worst a race can
do is make it one frame old -- which is invisible, whereas a torn read looks
like a drivetrain fault.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, fields
from types import MappingProxyType
from typing import Any, ClassVar, Mapping

from telekart_protocol import ControlFlags, Fault, TelemetryFlags, VehicleState

_EMPTY_PARAMS: Mapping[str, Any] = MappingProxyType({})

#: Staleness thresholds, defined here rather than in the link layer because
#: both the state machine and the model need them and this package is the one
#: neither depends on. Telemetry runs at 50 Hz, so half a second is twenty-five
#: missed packets -- past that the app has no idea what the car is doing. Video
#: runs at 30 fps with a 15-frame GOP, so a second covers a keyframe gap plus a
#: WiFi hiccup without crying wolf.
TELEMETRY_STALE_S = 0.5
VIDEO_STALE_S = 1.0


class LinkState(enum.Enum):
    """Connection state, and the safety consequence of each.

    The two failure states are deliberately distinct. Video loss is a
    *degradation*: the driver has lost the picture but the car is still
    reporting, still braking, still failsafing correctly -- and taking the
    motors away at that moment could strand it under a table. Telemetry loss is
    safety-critical: without it the app has no idea what the car is doing, and
    anything it displays is a guess.
    """

    OFFLINE = "offline"
    DISCOVERING = "discovering"
    CONNECTING = "connecting"
    LIVE = "live"
    DEGRADED = "degraded"  # video down; driving still permitted
    STALE = "stale"  # telemetry late or absent; safety-critical
    RECONNECTING = "reconnecting"
    FAILED = "failed"

    @property
    def usable(self) -> bool:
        """True when the app is entitled to keep sending control packets."""
        return self in (LinkState.LIVE, LinkState.DEGRADED, LinkState.STALE)

    @property
    def healthy(self) -> bool:
        return self in (LinkState.LIVE, LinkState.DEGRADED)


@dataclass(frozen=True, slots=True)
class VehicleSnapshot:
    """What the car says it is doing. Every field is SI.

    Nothing here is inferred by the app. In particular ``state`` is whatever the
    car reported -- the app never decides that it is armed, it only reports what
    the car claimed in the last packet it received.
    """

    valid: bool = False
    #: The last packet is older than the staleness threshold. The values below
    #: are still the most recent ones seen; they are just not news any more.
    stale: bool = True

    state: VehicleState = VehicleState.BOOT
    faults: Fault = Fault.NONE
    flags: TelemetryFlags = TelemetryFlags.NONE

    rpm_l: float = 0.0
    rpm_r: float = 0.0
    rpm_target_l: float = 0.0
    rpm_target_r: float = 0.0
    duty_l: float = 0.0
    duty_r: float = 0.0

    servo_us: int = 0
    steer_angle: float = 0.0  # radians

    speed: float = 0.0  # m/s
    #: Measured ceiling from calibration, published in every packet. Gauges
    #: scale off this so nothing in the app hardcodes a top speed.
    v_max: float = 0.0
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)  # x m, y m, heading rad
    distance: float = 0.0  # m
    slip: float = 0.0

    pack_volts: float = 0.0
    cpu_temp_c: float = 0.0
    throttled: int = 0
    loop_p99: float = 0.0  # seconds

    sequence: int = 0
    car_time_us: int = 0

    #: Metadata about *when* a value arrived, not about what the car is doing.
    #: They must be excluded or change suppression does nothing at all: the
    #: sequence number increments on every single packet, so comparing it would
    #: report a change fifty times a second on a car standing still, which is
    #: precisely the signal storm the epsilons exist to prevent. Freshness is
    #: reported by `stale` here and by `telemetry_age` on LinkSnapshot.
    _IGNORED_FIELDS: ClassVar[frozenset[str]] = frozenset({"sequence", "car_time_us"})
    _DEFAULT_EPSILON: ClassVar[float] = 1e-6
    _FIELD_EPSILON: ClassVar[Mapping[str, float]] = MappingProxyType(
        {
            # Below these, the change is sensor noise and repainting is waste.
            "rpm_l": 0.25,
            "rpm_r": 0.25,
            "rpm_target_l": 0.25,
            "rpm_target_r": 0.25,
            "duty_l": 0.001,
            "duty_r": 0.001,
            "speed": 0.002,  # 2 mm/s -- the wire resolution is 1 mm/s
            "v_max": 0.001,
            "steer_angle": 0.0005,  # ~0.03 deg
            "distance": 0.001,
            "slip": 0.002,
            "pack_volts": 0.01,
            "cpu_temp_c": 0.1,
            "loop_p99": 5e-5,
            "pose": 0.001,
        }
    )

    @property
    def speed_fraction(self) -> float:
        if self.v_max <= 0.0:
            return 0.0
        ratio = abs(self.speed) / self.v_max
        return 1.0 if ratio > 1.0 else ratio

    @property
    def armed(self) -> bool:
        return self.state is VehicleState.ARMED

    @property
    def calibrated(self) -> bool:
        return bool(self.flags & TelemetryFlags.CALIBRATED)


@dataclass(frozen=True, slots=True)
class LinkSnapshot:
    """Health of the four channels, and the reason the state machine picked its state."""

    state: LinkState = LinkState.OFFLINE
    detail: str = ""
    host: str = ""
    address: str = ""
    car_id: str = ""
    session_id: int = 0

    #: Seconds since the last packet on each channel. These are the fields the
    #: driver has to be able to trust: a HUD reading ARMED with a 3-second-old
    #: telemetry age means "unknown", not "safe".
    telemetry_age: float = 0.0
    video_age: float = 0.0
    session_age: float = 0.0

    rtt: float = 0.0
    rtt_p95: float = 0.0
    rtt_min: float = 0.0

    telemetry_hz: float = 0.0
    control_hz: float = 0.0
    video_fps: float = 0.0
    video_bitrate: float = 0.0  # bits per second
    #: Capture-to-decoded, measured against the car's pts. Only meaningful once
    #: the clocks have been offset-corrected, which the receiver does.
    video_latency: float = 0.0

    loss: float = 0.0  # 0..1, telemetry packets missing
    telemetry_packets: int = 0
    telemetry_lost: int = 0
    telemetry_bad: int = 0
    control_sent: int = 0
    control_errors: int = 0
    video_frames: int = 0
    video_dropped: int = 0

    video_ok: bool = False
    session_ok: bool = False
    estop_latched: bool = False

    _DEFAULT_EPSILON: ClassVar[float] = 1e-6
    _FIELD_EPSILON: ClassVar[Mapping[str, float]] = MappingProxyType(
        {
            # Ages tick continuously; quantising them is what stops a parked,
            # disconnected app from emitting a change signal every frame.
            "telemetry_age": 0.05,
            "video_age": 0.05,
            "session_age": 0.25,
            "rtt": 0.0005,
            "rtt_p95": 0.0005,
            "rtt_min": 0.0005,
            "telemetry_hz": 0.25,
            "control_hz": 0.25,
            "video_fps": 0.25,
            "video_bitrate": 5000.0,
            "video_latency": 0.002,
            "loss": 0.002,
        }
    )

    @property
    def connected(self) -> bool:
        return self.state.usable

    @property
    def degraded(self) -> bool:
        return self.state in (LinkState.DEGRADED, LinkState.STALE)


@dataclass(frozen=True, slots=True)
class InputSnapshot:
    """What was actually put on the wire, echoed back by the TX thread.

    Not what the input thread computed. If the chain breaks anywhere -- a
    crashed input thread, a dropped session, a clamp nobody expected -- the HUD
    shows the stale or zeroed command that the car is really receiving instead
    of cheerfully mirroring the wheel the driver is holding.
    """

    steering: float = 0.0  # -1..+1, dequantised from the packet
    throttle: float = 0.0  # 0..1
    brake: float = 0.0  # 0..1
    flags: ControlFlags = ControlFlags.NONE

    sequence: int = 0
    sent_at: float = 0.0  # perf_counter of the send
    #: Age of the command when it was sent. A rising value means the input
    #: thread is behind; the TX thread keeps sending the last known command,
    #: which is the correct failure mode and must still be visible.
    command_age: float = 0.0
    rate_hz: float = 0.0
    #: False when no session key is held, i.e. the packet was built but never
    #: sent. Without this the HUD would show a live-looking command going
    #: nowhere.
    transmitting: bool = False
    estop_latched: bool = False
    arm_intent: bool = False

    device: str = ""
    device_connected: bool = False

    #: Same reasoning as VehicleSnapshot: the sequence and the send timestamp
    #: advance on every packet and would make every echo a "change".
    _IGNORED_FIELDS: ClassVar[frozenset[str]] = frozenset({"sequence", "sent_at"})
    _DEFAULT_EPSILON: ClassVar[float] = 1e-6
    _FIELD_EPSILON: ClassVar[Mapping[str, float]] = MappingProxyType(
        {
            # The wire quantises to 1/1000, so anything finer is not a change
            # that the car can possibly see.
            "steering": 0.0005,
            "throttle": 0.0005,
            "brake": 0.0005,
            "command_age": 0.002,
            "rate_hz": 0.5,
        }
    )


@dataclass(frozen=True, slots=True)
class SessionSnapshot:
    """The TCP session and the car's authoritative parameter set."""

    active: bool = False
    session_id: int = 0
    car_id: str = ""
    fw_version: str = ""
    driver: str = ""
    caps: tuple[str, ...] = ()
    #: True when a reconnect landed on the same session id -- the car never
    #: dropped us, only the socket did, so the recording continues rather than
    #: being split in two.
    resumed: bool = False
    started_at: float = 0.0  # wall clock
    duration: float = 0.0

    #: The car's echo, never what the operator typed. A UI that renders the
    #: optimistic value is a UI that lies about which vehicle you are tuning.
    params: Mapping[str, Any] = _EMPTY_PARAMS
    params_generation: int = 0
    params_pending: int = 0

    recording: bool = False
    record_id: int | None = None

    last_error: str = ""
    last_error_code: str = ""

    _DEFAULT_EPSILON: ClassVar[float] = 1e-6
    _FIELD_EPSILON: ClassVar[Mapping[str, float]] = MappingProxyType({"duration": 0.5})


EMPTY_VEHICLE = VehicleSnapshot()
EMPTY_LINK = LinkSnapshot()
EMPTY_INPUT = InputSnapshot()
EMPTY_SESSION = SessionSnapshot()


# --------------------------------------------------------------------------
# Change detection
# --------------------------------------------------------------------------

_SPEC_CACHE: dict[type, tuple[tuple[str, float], ...]] = {}


def _spec(cls: type) -> tuple[tuple[str, float], ...]:
    cached = _SPEC_CACHE.get(cls)
    if cached is not None:
        return cached
    per_field: Mapping[str, float] = getattr(cls, "_FIELD_EPSILON", _EMPTY_PARAMS)
    ignored: frozenset[str] = getattr(cls, "_IGNORED_FIELDS", frozenset())
    default: float = getattr(cls, "_DEFAULT_EPSILON", 1e-6)
    spec = tuple(
        (f.name, per_field.get(f.name, default))
        for f in fields(cls)
        if f.name not in ignored
    )
    _SPEC_CACHE[cls] = spec
    return spec


def differs(a: object, b: object) -> bool:
    """True when two snapshots differ enough to be worth a repaint.

    Floats are compared against a per-field epsilon rather than exactly. The
    reason is concrete: a parked car still reports encoder noise, and an exact
    comparison would make every "unchanged" telemetry packet emit a change
    signal to every subscriber -- which is the signal storm the whole
    LatestBox design exists to avoid.
    """
    if a is b:
        return False
    if type(a) is not type(b):
        return True
    for name, eps in _spec(type(a)):
        av = getattr(a, name)
        bv = getattr(b, name)
        if av is bv:
            continue
        if type(av) is float and type(bv) is float:
            if av != bv and (av - bv if av > bv else bv - av) > eps:
                return True
            continue
        if type(av) is tuple and type(bv) is tuple:
            if len(av) != len(bv):
                return True
            for x, y in zip(av, bv):
                if type(x) is float and type(y) is float:
                    if x != y and (x - y if x > y else y - x) > eps:
                        return True
                elif x != y:
                    return True
            continue
        if av != bv:
            return True
    return False
