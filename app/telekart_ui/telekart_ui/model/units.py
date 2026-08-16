"""Wire units to words on screen.

The HUD paints numbers and short labels; this is where both come from. Nothing
here knows about Qt, so the tables can be asserted in a plain test.

The car reports SI and this converts once, at the display edge. That direction
is one-way on purpose: SI goes in, a string comes out, and no part of the app
ever reads a value back out of a formatted string.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass

from telekart_protocol import Fault, TelemetryFlags, VehicleState

_MPS_TO_KMH = 3.6
_MPS_TO_MPH = 2.2369362920544025


class SpeedMode(enum.Enum):
    KMH = "kmh"
    MPH = "mph"
    MS = "ms"

    @classmethod
    def parse(cls, value: str) -> "SpeedMode":
        """Never raises. A hand-edited settings file with a typo in it should
        show km/h, not refuse to launch."""
        try:
            return cls(str(value).strip().lower())
        except ValueError:
            return cls.KMH

    @property
    def suffix(self) -> str:
        return {SpeedMode.KMH: "km/h", SpeedMode.MPH: "mph", SpeedMode.MS: "m/s"}[self]


def mps_to_kmh(mps: float) -> float:
    return mps * _MPS_TO_KMH


def mps_to_mph(mps: float) -> float:
    return mps * _MPS_TO_MPH


def rad_to_deg(radians: float) -> float:
    return radians * 57.29577951308232


def fraction_of(value: float, ceiling: float) -> float:
    """``value / ceiling`` clamped to 0..1, and 0.0 when the ceiling is unknown.

    Returning zero rather than guessing is what stops the speed readout scaling
    against a made-up top speed before the car has published its measured one.
    """
    if ceiling <= 0.0:
        return 0.0
    ratio = abs(value) / ceiling
    return 1.0 if ratio > 1.0 else ratio


@dataclass(frozen=True, slots=True)
class UnitFormatter:
    """Speed in whichever unit the driver asked for."""

    speed_mode: SpeedMode = SpeedMode.KMH

    def speed_value(self, mps: float) -> float:
        if self.speed_mode is SpeedMode.KMH:
            return mps_to_kmh(mps)
        if self.speed_mode is SpeedMode.MPH:
            return mps_to_mph(mps)
        return mps

    def speed_text(self, mps: float) -> str:
        value = self.speed_value(mps)
        # No decimal on the hero number. At these speeds the tenths digit
        # changes several times a second and reads as noise, and a driver
        # glancing at it wants a magnitude, not a measurement.
        return f"{value:.0f}" if self.speed_mode is not SpeedMode.MS else f"{value:.1f}"

    @property
    def speed_suffix(self) -> str:
        return self.speed_mode.suffix


def format_latency(seconds: float) -> str:
    return f"{seconds * 1000.0:.0f}"


def format_voltage(volts: float) -> str:
    return f"{volts:.1f}"


def format_temperature(celsius: float) -> str:
    return f"{celsius:.0f}"


# --------------------------------------------------------------------------
# Labels
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
    TelemetryFlags.BRAKING: "Brake",
}

#: Flags that are set in normal operation, so their *absence* is the exception
#: worth showing. Everything else in FLAG_LABELS is reported when set.
_NOMINAL_WHEN_SET: dict[TelemetryFlags, str] = {
    TelemetryFlags.CALIBRATED: "No cal",
    TelemetryFlags.CLOSED_LOOP: "Open loop",
}


def state_text(state: VehicleState) -> str:
    return STATE_LABELS.get(state, "Unknown")


def fault_text(bit: int) -> str:
    """Words for one fault bit.

    Unknown bits are named rather than dropped: the firmware is allowed to grow
    faults this build has never heard of, and "fault bit 19" tells the driver
    there is something wrong and gives them something to grep the firmware for.
    """
    try:
        flag = Fault(bit)
    except ValueError:
        return f"fault bit {bit.bit_length() - 1}"
    return FAULT_LABELS.get(flag, f"fault bit {bit.bit_length() - 1}")


def fault_texts(faults: Fault) -> list[str]:
    return [fault_text(bit) for bit in iter_fault_bits(faults)]


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
    out.extend(label for flag, label in _NOMINAL_WHEN_SET.items() if not (flags & flag))
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


__all__ = [
    "SpeedMode",
    "UnitFormatter",
    "STATE_LABELS",
    "FAULT_LABELS",
    "FLAG_LABELS",
    "fraction_of",
    "mps_to_kmh",
    "mps_to_mph",
    "rad_to_deg",
    "format_latency",
    "format_temperature",
    "format_voltage",
    "state_text",
    "fault_text",
    "fault_texts",
    "hud_flag_texts",
    "iter_fault_bits",
]
