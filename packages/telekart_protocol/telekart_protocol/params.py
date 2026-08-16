"""Canonical vehicle parameter definitions.

One registry, shared by both ends. The firmware validates incoming SET_PARAMS
against it; the desktop app builds its tuning UI *from* it, so adding a
parameter here makes it appear in the app with the right widget, range, and
units without touching any UI code.

Defaults here are conservative starting points, not tuned values. Anything
marked ``measured=True`` is expected to be overwritten by a bring-up procedure
(see docs/calibration.md) -- treating a guess as a measurement is how you spend
an afternoon tuning a PID against the wrong plant gain.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

ParamKind = Literal["float", "int", "bool", "enum"]


@dataclass(frozen=True, slots=True)
class ParamDef:
    name: str
    group: str
    label: str
    kind: ParamKind
    default: Any
    minimum: float | None = None
    maximum: float | None = None
    step: float | None = None
    unit: str = ""
    choices: tuple[str, ...] = ()
    description: str = ""
    #: Hidden behind an "advanced" toggle in the UI.
    advanced: bool = False
    #: Determined by a bring-up measurement, not by taste.
    measured: bool = False
    #: Changing this while armed is refused by the firmware.
    requires_disarm: bool = False


class ParamError(ValueError):
    """A parameter name is unknown or its value is out of range."""


def _p(*args: Any, **kwargs: Any) -> ParamDef:
    return ParamDef(*args, **kwargs)


_DEFS: tuple[ParamDef, ...] = (
    # ---------------------------------------------------------------- drive
    _p("max_duty", "drive", "Power limit", "float", 0.85, 0.10, 1.00, 0.05,
       description="Ceiling on any single motor's duty cycle."),
    _p("duty_sum_max", "drive", "Combined power budget", "float", 1.40, 0.20, 2.00, 0.05,
       description="Ceiling on the SUM of both motors' duty. The boost regulator "
                   "sustains roughly 1.5 A total, and it is simultaneous demand "
                   "that trips it, not either motor alone."),
    _p("closed_loop", "drive", "Closed-loop speed control", "bool", True,
       description="Off falls back to open-loop feedforward duty."),
    _p("throttle_expo", "drive", "Throttle curve", "float", 1.8, 0.5, 3.0, 0.1,
       description="1.0 is linear; higher gives finer control near neutral."),
    _p("throttle_deadband", "drive", "Throttle deadband", "float", 0.04, 0.0, 0.20, 0.01),
    _p("accel_rpm_per_s", "drive", "Acceleration limit", "float", 250.0, 50.0, 2000.0, 25.0,
       unit="RPM/s",
       description="Rate-limits the RPM target, not the duty. Inrush during "
                   "throttle steps is what trips the regulator."),
    _p("decel_rpm_per_s", "drive", "Deceleration limit", "float", 700.0, 50.0, 3000.0, 25.0,
       unit="RPM/s"),
    _p("brake_strength", "drive", "Brake strength", "float", 0.40, 0.0, 1.0, 0.05,
       description="Duty applied to the shorted bridge when braking."),
    _p("reverse_enabled", "drive", "Allow reverse", "bool", True),
    _p("pit_duty", "drive", "Pit limiter", "float", 0.25, 0.05, 0.60, 0.05),

    # ------------------------------------------------------------------ pid
    _p("pid_kp", "pid", "Kp", "float", 0.005, 0.0, 0.100, 0.001, unit="duty/RPM",
       advanced=True),
    _p("pid_ki", "pid", "Ki", "float", 0.020, 0.0, 0.500, 0.001, unit="duty/(RPM*s)",
       advanced=True),
    _p("pid_kd", "pid", "Kd", "float", 0.0, 0.0, 0.050, 0.001, unit="duty*s/RPM",
       advanced=True,
       description="Leave at zero unless you also filter the derivative -- "
                   "encoder velocity is noisy."),
    _p("pid_i_clamp", "pid", "Integrator clamp", "float", 0.40, 0.0, 1.0, 0.05,
       advanced=True),
    _p("straight_sync_gain", "pid", "Straight-line sync", "float", 30.0, 0.0, 200.0, 5.0,
       unit="RPM/m", advanced=True,
       description="Corrects left/right distance divergence. Active only near "
                   "centre, otherwise it fights the electronic differential."),

    # ------------------------------------------------------------- steering
    _p("steer_center_us", "steering", "Centre pulse", "int", 1500, 900, 2100, 1,
       unit="us", measured=True,
       description="Found by servo_calibrate. Never seeded from the old firmware."),
    _p("steer_min_us", "steering", "Left limit", "int", 1200, 700, 2400, 1,
       unit="us", measured=True),
    _p("steer_max_us", "steering", "Right limit", "int", 1800, 700, 2400, 1,
       unit="us", measured=True),
    _p("steer_trim_us", "steering", "Trim", "int", 0, -200, 200, 1, unit="us",
       description="Fine centre adjustment. The single biggest lever on "
                   "odometry heading drift."),
    _p("steer_max_deg", "steering", "Steering lock", "float", 24.0, 5.0, 45.0, 1.0,
       unit="deg", measured=True,
       description="Measured at full lock with a protractor. Feeds the "
                   "odometry model and the electronic differential."),
    _p("steer_rate_us_per_s", "steering", "Steering speed", "float", 2000.0, 200.0, 6000.0,
       100.0, unit="us/s",
       description="Slew limit. Also caps the servo's peak current draw."),
    _p("steer_expo", "steering", "Steering curve", "float", 1.0, 0.5, 3.0, 0.1),
    _p("steer_deadzone", "steering", "Steering deadzone", "float", 0.03, 0.0, 0.20, 0.01),
    _p("steer_speed_reduction", "steering", "Speed-sensitive steering", "float", 0.45,
       0.0, 0.80, 0.05,
       description="Fraction of steering range removed at top speed."),
    _p("steer_hold_us", "steering", "Jitter deadband", "int", 8, 0, 40, 1, unit="us",
       advanced=True,
       description="Suppresses servo buzz by not rewriting tiny changes."),
    _p("steer_invert", "steering", "Invert steering", "bool", False),
    _p("servo_relax_when_disarmed", "steering", "Relax servo when disarmed", "bool", True,
       description="Stops the pulse train so holding current drops to near "
                   "zero. Matters because the servo shares the Pi's 5 V rail."),

    # ------------------------------------------------------------- geometry
    _p("wheel_diameter_m", "geometry", "Wheel diameter", "float", 0.065, 0.02, 0.30, 0.001,
       unit="m", measured=True, requires_disarm=True),
    _p("wheelbase_m", "geometry", "Wheelbase", "float", 0.200, 0.05, 1.00, 0.001,
       unit="m", measured=True, requires_disarm=True),
    _p("track_width_m", "geometry", "Track width", "float", 0.150, 0.05, 1.00, 0.001,
       unit="m", measured=True, requires_disarm=True),
    _p("encoder_cpr", "geometry", "Counts per output rev", "int", 660, 1, 20000, 1,
       measured=True, requires_disarm=True,
       description="Verified by hand-turning ten revolutions, not derived from "
                   "the nameplate gear ratio."),
    _p("invert_left", "geometry", "Invert left motor", "bool", False, requires_disarm=True),
    _p("invert_right", "geometry", "Invert right motor", "bool", False, requires_disarm=True),
    _p("encoder_invert_left", "geometry", "Invert left encoder", "bool", False,
       requires_disarm=True),
    _p("encoder_invert_right", "geometry", "Invert right encoder", "bool", False,
       requires_disarm=True),

    # --------------------------------------------------------------- safety
    _p("control_timeout_ms", "safety", "Control timeout", "int", 200, 50, 1000, 10,
       unit="ms"),
    _p("failsafe_brake_duty", "safety", "Failsafe brake", "float", 0.35, 0.0, 1.0, 0.05),
    _p("stall_detect_ms", "safety", "Stall detect window", "int", 600, 100, 2000, 50,
       unit="ms",
       description="Protects the L298N as much as the motor: two stalled "
                   "motors put ~7 W into a bridge whose heatsink handles 2-3 W."),
    _p("stall_rpm_threshold", "safety", "Stall RPM threshold", "float", 5.0, 0.0, 50.0, 1.0,
       unit="RPM", advanced=True),
    _p("low_battery_v", "safety", "Low battery warning", "float", 6.0, 0.0, 30.0, 0.1,
       unit="V", description="Zero disables. Requires battery sensing hardware."),
    _p("critical_battery_v", "safety", "Critical battery cutoff", "float", 5.4, 0.0, 30.0,
       0.1, unit="V"),
    _p("arm_neutral_ms", "safety", "Neutral hold to arm", "int", 500, 0, 3000, 50,
       unit="ms"),

    # ------------------------------------------------------------------ pwm
    _p("pwm_hz", "pwm", "PWM frequency", "int", 1000, 500, 8000, 100, unit="Hz",
       advanced=True, requires_disarm=True,
       description="1 kHz keeps Darlington switching loss near 0.4% of the "
                   "period. Both channels share one divider and are always "
                   "written together."),
    _p("direction_deadtime_ms", "pwm", "Direction dead time", "int", 30, 0, 200, 5,
       unit="ms", advanced=True),
    _p("reverse_allowed_rpm", "pwm", "Reverse threshold", "float", 15.0, 0.0, 100.0, 1.0,
       unit="RPM", advanced=True),

    # ---------------------------------------------------------------- video
    _p("video_codec", "video", "Codec", "enum", "h264", choices=("h264", "mjpeg"),
       requires_disarm=True),
    _p("video_width", "video", "Width", "int", 640, 160, 1920, 16, requires_disarm=True),
    _p("video_height", "video", "Height", "int", 480, 120, 1080, 16, requires_disarm=True),
    _p("video_fps", "video", "Frame rate", "int", 30, 5, 60, 1, unit="fps",
       requires_disarm=True),
    _p("video_bitrate", "video", "Bitrate", "int", 2_000_000, 200_000, 12_000_000, 100_000,
       unit="bps", requires_disarm=True),
    _p("video_iperiod", "video", "Keyframe interval", "int", 15, 1, 120, 1, unit="frames",
       advanced=True, requires_disarm=True,
       description="The hardware encoder cannot honour on-demand keyframe "
                   "requests, so a short GOP is the only recovery mechanism "
                   "after packet loss."),
)

PARAMS: dict[str, ParamDef] = {d.name: d for d in _DEFS}

GROUPS: tuple[str, ...] = ("drive", "pid", "steering", "geometry", "safety", "pwm", "video")


def defaults() -> dict[str, Any]:
    """The full default parameter set."""
    return {d.name: d.default for d in _DEFS}


def group(name: str) -> list[ParamDef]:
    return [d for d in _DEFS if d.group == name]


def coerce(name: str, value: Any) -> Any:
    """Validate and normalize one parameter. Raises ParamError.

    Out-of-range values are *rejected*, not clamped. A control packet's axis
    value gets clamped because dropping it would be worse; a parameter push is
    deliberate, and silently accepting a different number than the operator
    typed is how the UI ends up lying about the car.
    """
    definition = PARAMS.get(name)
    if definition is None:
        raise ParamError(f"unknown parameter {name!r}")

    if definition.kind == "bool":
        if not isinstance(value, bool):
            raise ParamError(f"{name} must be a boolean, got {type(value).__name__}")
        return value

    if definition.kind == "enum":
        if not isinstance(value, str) or value not in definition.choices:
            raise ParamError(
                f"{name} must be one of {definition.choices}, got {value!r}"
            )
        return value

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParamError(f"{name} must be numeric, got {type(value).__name__}")

    number: float | int = int(value) if definition.kind == "int" else float(value)
    if definition.kind == "int" and float(value) != int(value):
        raise ParamError(f"{name} must be a whole number, got {value}")

    if definition.minimum is not None and number < definition.minimum:
        raise ParamError(f"{name}={number} is below the minimum {definition.minimum}")
    if definition.maximum is not None and number > definition.maximum:
        raise ParamError(f"{name}={number} is above the maximum {definition.maximum}")
    return number


def coerce_all(values: dict[str, Any]) -> dict[str, Any]:
    """Validate a whole parameter dict, reporting every problem at once.

    Reporting all failures rather than the first keeps the app from making the
    operator fix a form one field per round trip.
    """
    result: dict[str, Any] = {}
    problems: list[str] = []
    for name, value in values.items():
        try:
            result[name] = coerce(name, value)
        except ParamError as exc:
            problems.append(str(exc))
    if problems:
        raise ParamError("; ".join(problems))
    return result


def merged_with_defaults(values: dict[str, Any]) -> dict[str, Any]:
    """Overlay `values` onto the defaults, ignoring unknown keys.

    Used when loading a config file written by an older or newer build: an
    unrecognised key is a reason to log, not a reason to refuse to boot.
    """
    merged = defaults()
    for name, value in values.items():
        if name in PARAMS:
            try:
                merged[name] = coerce(name, value)
            except ParamError:
                pass  # keep the default; caller logs via `unknown_or_invalid`
    return merged


def unknown_or_invalid(values: dict[str, Any]) -> list[str]:
    """Names in `values` that are unknown or fail validation. For logging."""
    bad: list[str] = []
    for name, value in values.items():
        if name not in PARAMS:
            bad.append(f"{name} (unknown)")
            continue
        try:
            coerce(name, value)
        except ParamError as exc:
            bad.append(str(exc))
    return bad
