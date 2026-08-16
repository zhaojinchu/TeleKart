"""SI to display conversion, and the labels the HUD puts next to the numbers.

Everything inside the app is SI. This is the only place that multiplies by 3.6,
and the only place that knows what a "scale km/h" is.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass

from telekart_protocol import Fault, TelemetryFlags, VehicleState

#: 1:10 is the usual RC chassis scale, and the one this car is built to.
DEFAULT_SCALE_FACTOR = 10.0

_MPS_TO_KMH = 3.6
_MPS_TO_MPH = 2.2369362920544025


class SpeedMode(enum.Enum):
    """How the speedometer reads.

    SCALE_KMH is the default and it is not a gimmick. A 1:10 car at 3 m/s
    covers ten body-lengths a second; the driver's eye is calibrated to the
    model's size, not to the metre. Reading "10.8 km/h" understates what is
    actually happening on the track, while "108 km/h" is the speed the scene
    is moving at in the scale the driver is perceiving.

    The real figure never goes away -- REAL_MPS is what every log, every
    telemetry field and the diagnostic overlay use, because scale speed is a
    presentation and a scaled number in an engineering readout is a trap.
    """

    SCALE_KMH = "scale_kmh"
    SCALE_MPH = "scale_mph"
    REAL_KMH = "real_kmh"
    REAL_MPS = "real_mps"

    @classmethod
    def parse(cls, value: str) -> "SpeedMode":
        try:
            return cls(value)
        except ValueError:
            return cls.SCALE_KMH


# --------------------------------------------------------------------------
# Scalar conversions
# --------------------------------------------------------------------------


def mps_to_kmh(mps: float) -> float:
    return mps * _MPS_TO_KMH


def mps_to_mph(mps: float) -> float:
    return mps * _MPS_TO_MPH


def kmh_to_mps(kmh: float) -> float:
    return kmh / _MPS_TO_KMH


def rad_to_deg(radians: float) -> float:
    return math.degrees(radians)


def deg_to_rad(degrees: float) -> float:
    return math.radians(degrees)


def rpm_to_mps(rpm: float, wheel_diameter_m: float) -> float:
    """Wheel RPM to ground speed. No slip term: the encoders measure the wheel,
    and the difference between wheel speed and ground speed *is* the slip index."""
    return rpm / 60.0 * math.pi * wheel_diameter_m


def mps_to_rpm(mps: float, wheel_diameter_m: float) -> float:
    if wheel_diameter_m <= 0.0:
        return 0.0
    return mps * 60.0 / (math.pi * wheel_diameter_m)


def duty_percent(duty: float) -> float:
    return duty * 100.0


def fraction_of(value: float, ceiling: float) -> float:
    """Normalise to 0..1 against a *measured* ceiling.

    Guards the divide because ``v_max`` is zero until the car has been
    calibrated, and a gauge that divides by it would take the HUD down on the
    very first packet from an uncalibrated vehicle.
    """
    if ceiling <= 0.0:
        return 0.0
    ratio = abs(value) / ceiling
    return 1.0 if ratio > 1.0 else ratio


# --------------------------------------------------------------------------
# Formatting
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class UnitFormatter:
    """Bundles the driver's display preferences so widgets carry one object."""

    speed_mode: SpeedMode = SpeedMode.SCALE_KMH
    scale_factor: float = DEFAULT_SCALE_FACTOR

    def speed(self, mps: float) -> tuple[float, str]:
        """Return ``(value, unit)`` for the speedometer."""
        mode = self.speed_mode
        if mode is SpeedMode.REAL_MPS:
            return mps, "m/s"
        if mode is SpeedMode.REAL_KMH:
            return mps_to_kmh(mps), "km/h"
        scaled = mps * self.scale_factor
        if mode is SpeedMode.SCALE_MPH:
            return mps_to_mph(scaled), "mph"
        return mps_to_kmh(scaled), "km/h"

    def speed_text(self, mps: float, decimals: int | None = None) -> str:
        value, unit = self.speed(mps)
        if decimals is None:
            # Scale readings run to three digits, so a decimal place is noise;
            # a real m/s reading is 0..4 and needs two.
            decimals = 0 if self.speed_mode in (SpeedMode.SCALE_KMH, SpeedMode.SCALE_MPH) else 2
        return f"{value:.{decimals}f} {unit}"

    def real_speed_text(self, mps: float) -> str:
        """Always true SI. For the diagnostic overlay and for logs."""
        return f"{mps:.2f} m/s"

    @property
    def scaled(self) -> bool:
        return self.speed_mode in (SpeedMode.SCALE_KMH, SpeedMode.SCALE_MPH)


def format_distance(metres: float) -> str:
    if abs(metres) >= 1000.0:
        return f"{metres / 1000.0:.2f} km"
    if abs(metres) >= 10.0:
        return f"{metres:.1f} m"
    return f"{metres:.2f} m"


def format_duration(seconds: float) -> str:
    """H:MM:SS for session length. Negative clamps to zero rather than printing
    a minus sign the user would have to interpret."""
    total = int(seconds) if seconds > 0.0 else 0
    hours, rem = divmod(total, 3600)
    minutes, secs = divmod(rem, 60)
    if hours:
        return f"{hours}:{minutes:02d}:{secs:02d}"
    return f"{minutes}:{secs:02d}"


def format_lap_time(seconds: float | None) -> str:
    """``M:SS.mmm``. Milliseconds because that is the resolution a lap is won by."""
    if seconds is None or seconds <= 0.0 or not math.isfinite(seconds):
        return "--:--.---"
    minutes = int(seconds // 60)
    rest = seconds - minutes * 60
    return f"{minutes}:{rest:06.3f}"


def format_delta(seconds: float | None) -> str:
    """Signed lap delta. The sign is the whole message, so it is always shown."""
    if seconds is None or not math.isfinite(seconds):
        return "--.---"
    return f"{seconds:+.3f}"


def format_latency(seconds: float) -> str:
    if seconds <= 0.0 or not math.isfinite(seconds):
        return "-- ms"
    return f"{seconds * 1000.0:.0f} ms"


def format_voltage(volts: float) -> str:
    # Zero means "no battery sensing fitted", which is a different statement
    # from "0.0 V" and must not be shown as a flat battery.
    return "--" if volts <= 0.0 else f"{volts:.2f} V"


def format_temperature(celsius: float) -> str:
    return "--" if celsius <= 0.0 else f"{celsius:.1f} °C"


# --------------------------------------------------------------------------
# Enum labels
# --------------------------------------------------------------------------

STATE_LABELS: dict[VehicleState, str] = {
    VehicleState.BOOT: "Booting",
    VehicleState.SAFE: "Safe",
    VehicleState.ARMED: "Armed",
    VehicleState.FAILSAFE: "Failsafe",
    VehicleState.ESTOP: "E-STOP",
    VehicleState.FAULT: "Fault",
}

FAULT_LABELS: dict[Fault, str] = {
    Fault.STALL_L: "Left motor stalled",
    Fault.STALL_R: "Right motor stalled",
    Fault.ENCODER_FAIL_L: "Left encoder silent",
    Fault.ENCODER_FAIL_R: "Right encoder silent",
    Fault.BROWNOUT: "Regulator brownout",
    Fault.CONTROL_TIMEOUT: "Control link timeout",
    Fault.OVERTEMP: "Over temperature",
    Fault.LOW_BATTERY: "Battery low",
    Fault.CRITICAL_BATTERY: "Battery critical",
    Fault.GPIO_ERROR: "GPIO backend error",
    Fault.LOOP_OVERRUN: "Control loop overrun",
    Fault.SERVO_FAULT: "Servo fault",
    Fault.PI_UNDERVOLTAGE: "Pi undervoltage",
    Fault.PI_THROTTLED: "Pi throttled",
    Fault.CAMERA_DOWN: "Camera down",
    Fault.ESTOP_LATCHED: "E-stop latched",
    Fault.CALIBRATION_MISSING: "Not calibrated",
}

FLAG_LABELS: dict[TelemetryFlags, str] = {
    TelemetryFlags.LIMITER_ACTIVE: "Limiter",
    TelemetryFlags.DIRECTION_UNCERTAIN: "Dir?",
    TelemetryFlags.CALIBRATED: "Cal",
    TelemetryFlags.CLOSED_LOOP: "CL",
    TelemetryFlags.REVERSE_ENGAGED: "Rev",
    TelemetryFlags.PIT_LIMITER: "Pit",
    TelemetryFlags.BRAKING: "Brake",
    TelemetryFlags.ODOM_VALID: "Odom",
    TelemetryFlags.VIDEO_ACTIVE: "Video",
}


def state_text(state: VehicleState) -> str:
    return STATE_LABELS.get(state, f"State {int(state)}")


def fault_text(bit: int) -> str:
    """Label for a single fault bit.

    Falls back to the raw bit rather than dropping it: a firmware newer than
    this build can set a bit we have no name for, and an unnamed fault the
    driver can still see is far better than a silent one.
    """
    try:
        fault = Fault(bit)
    except ValueError:
        return f"Fault bit 0x{bit:X}"
    return FAULT_LABELS.get(fault, f"Fault bit 0x{bit:X}")


def fault_texts(faults: Fault) -> list[str]:
    """Every set bit, in bit order."""
    value = int(faults)
    out: list[str] = []
    bit = 1
    while bit <= value:
        if value & bit:
            out.append(fault_text(bit))
        bit <<= 1
    return out


#: Flags whose *set* state is the normal, uninteresting one. A healthy armed car
#: has all four lit continuously, so rendering them on the HUD produces a
#: permanent strip that means "everything is fine" -- which is indistinguishable
#: at a glance from a strip that means something. Their absence is the news, so
#: the HUD reports that instead.
_NOMINAL_WHEN_SET: dict[TelemetryFlags, str] = {
    TelemetryFlags.CALIBRATED: "NO CAL",
    TelemetryFlags.CLOSED_LOOP: "OPEN LOOP",
    TelemetryFlags.ODOM_VALID: "NO ODOM",
    TelemetryFlags.VIDEO_ACTIVE: "NO VIDEO",
}


def flag_texts(flags: TelemetryFlags) -> list[str]:
    """Every active flag. For the diagnostic overlay, which wants the raw truth."""
    return [label for flag, label in FLAG_LABELS.items() if flags & flag]


def hud_flag_texts(flags: TelemetryFlags) -> list[str]:
    """Only what is worth a driver's attention, so an empty strip means nominal.

    Two kinds of entry: a flag that is set when it usually is not (the limiter
    engaged, direction is a guess, reverse is in), and a flag that is *clear*
    when it usually is not (no calibration, open loop). Both are exceptions;
    steady-state normality is deliberately silent.
    """
    out = [
        label
        for flag, label in FLAG_LABELS.items()
        if (flags & flag) and flag not in _NOMINAL_WHEN_SET
    ]
    out.extend(
        label for flag, label in _NOMINAL_WHEN_SET.items() if not (flags & flag)
    )
    return out


def iter_fault_bits(faults: Fault) -> list[int]:
    """The raw set bits. ``AppModel`` uses this for edge-triggered alerts."""
    value = int(faults)
    out: list[int] = []
    bit = 1
    while bit <= value:
        if value & bit:
            out.append(bit)
        bit <<= 1
    return out
