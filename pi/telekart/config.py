"""Vehicle configuration: pin assignment plus every tunable from the shared registry.

The parameter half of :class:`VehicleConfig` is *generated* from
``telekart_protocol.params.PARAMS`` at import time. Adding a parameter to that
registry makes it settable over the session channel, editable in the desktop
tuning UI, loadable from YAML, and readable as ``config.<name>`` here, with no
edit to this file. The previous generation of this project kept four separate
hand-maintained lists of parameter names and they were never all correct at once.

The cost of generating the fields is that a static type checker cannot see them.
That is a real cost, accepted knowingly: an out-of-date hand-written list is a
runtime bug on a moving vehicle, while a missing type stub is a lint warning.

Three layers, applied in order, each overriding the last:

1. ``pi/config/telekart.yaml`` -- checked in, identical on every car.
2. ``pi/config/config.local.yaml`` -- git-ignored, per-vehicle measurements.
3. Live ``set_params`` from the desktop app, and ``save_local`` to persist them.
"""

from __future__ import annotations

import dataclasses
import os
import stat
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, ClassVar, Iterable

from telekart_protocol.params import (
    PARAMS,
    ParamDef,
    ParamError,
    coerce,
    coerce_all,
    defaults,
    merged_with_defaults,
    unknown_or_invalid,
)

from .constants import (
    DEFAULT_CONFIG_PATH,
    ENV_CAR_ID,
    ENV_CONFIG,
    ENV_LOCAL_CONFIG,
    ENV_SHARED_KEY,
    HW_PWM_CHANNEL_0_PINS,
    HW_PWM_CHANNEL_1_PINS,
    LOCAL_CONFIG_PATH,
    PIN_ENA,
    PIN_ENB,
    PIN_ENC_L_A,
    PIN_ENC_L_B,
    PIN_ENC_R_A,
    PIN_ENC_R_B,
    PIN_ESTOP_BUTTON,
    PIN_IN1,
    PIN_IN2,
    PIN_IN3,
    PIN_IN4,
    PIN_SERVO,
    PIN_STATUS_LED,
    rpm_to_mps,
)
from .constants import wheel_circumference_m as _wheel_circumference_m
from .log import get_logger

_log = get_logger(__name__)

DEFAULT_CAR_ID = "telekart"
#: Placeholder only. A car left on this key is refused by `check()`; the real
#: key belongs in config.local.yaml or TELEKART_SHARED_KEY, neither of which is
#: in git.
PLACEHOLDER_SHARED_KEY = "change-me"

#: Highest BCM pin number on a 40-pin header.
_MAX_BCM_PIN = 27


class ConfigError(RuntimeError):
    """The configuration cannot be used. Always raised at load time, never later."""


# --------------------------------------------------------------------------
# Pins
# --------------------------------------------------------------------------


@dataclass(slots=True)
class MotorPins:
    ena: int = PIN_ENA
    in1: int = PIN_IN1
    in2: int = PIN_IN2
    in3: int = PIN_IN3
    in4: int = PIN_IN4
    enb: int = PIN_ENB

    def as_tuple(self) -> tuple[int, ...]:
        return (self.ena, self.in1, self.in2, self.in3, self.in4, self.enb)

    @property
    def left_in(self) -> tuple[int, int]:
        return (self.in1, self.in2)

    @property
    def right_in(self) -> tuple[int, int]:
        return (self.in3, self.in4)


@dataclass(slots=True)
class EncoderPins:
    left_a: int = PIN_ENC_L_A
    left_b: int = PIN_ENC_L_B
    right_a: int = PIN_ENC_R_A
    right_b: int = PIN_ENC_R_B

    def as_tuple(self) -> tuple[int, ...]:
        return (self.left_a, self.left_b, self.right_a, self.right_b)


@dataclass(slots=True)
class HardwarePins:
    motors: MotorPins = field(default_factory=MotorPins)
    encoders: EncoderPins = field(default_factory=EncoderPins)
    servo: int = PIN_SERVO
    status_led: int | None = PIN_STATUS_LED
    estop_button: int | None = PIN_ESTOP_BUTTON

    def all_pins(self) -> tuple[int, ...]:
        pins = list(self.motors.as_tuple()) + list(self.encoders.as_tuple()) + [self.servo]
        if self.status_led is not None:
            pins.append(self.status_led)
        if self.estop_button is not None:
            pins.append(self.estop_button)
        return tuple(pins)

    def validate(self) -> None:
        """Reject a wiring description that cannot physically work.

        Loud and early: a duplicated pin means two drivers fighting over one
        output, and finding that out from the smell of a stalled motor is worse
        than finding it out from a traceback at boot.
        """
        seen: dict[int, str] = {}
        named = [
            ("motors.ena", self.motors.ena),
            ("motors.in1", self.motors.in1),
            ("motors.in2", self.motors.in2),
            ("motors.in3", self.motors.in3),
            ("motors.in4", self.motors.in4),
            ("motors.enb", self.motors.enb),
            ("encoders.left_a", self.encoders.left_a),
            ("encoders.left_b", self.encoders.left_b),
            ("encoders.right_a", self.encoders.right_a),
            ("encoders.right_b", self.encoders.right_b),
            ("servo", self.servo),
        ]
        if self.status_led is not None:
            named.append(("status_led", self.status_led))
        if self.estop_button is not None:
            named.append(("estop_button", self.estop_button))

        for label, pin in named:
            if not isinstance(pin, int) or isinstance(pin, bool):
                raise ConfigError(f"pin {label} must be an integer, got {pin!r}")
            if not 0 <= pin <= _MAX_BCM_PIN:
                raise ConfigError(f"pin {label}={pin} is not a valid BCM pin (0-{_MAX_BCM_PIN})")
            if pin in seen:
                raise ConfigError(f"pin {pin} assigned to both {seen[pin]} and {label}")
            seen[pin] = label

        # ENA and ENB must land on opposite hardware PWM channels. Two pins on
        # channel 0 would make one motor's duty silently mirror the other's.
        if self.motors.ena not in HW_PWM_CHANNEL_0_PINS:
            raise ConfigError(
                f"motors.ena={self.motors.ena} is not a PWM0 pin "
                f"(expected one of {sorted(HW_PWM_CHANNEL_0_PINS)})"
            )
        if self.motors.enb not in HW_PWM_CHANNEL_1_PINS:
            raise ConfigError(
                f"motors.enb={self.motors.enb} is not a PWM1 pin "
                f"(expected one of {sorted(HW_PWM_CHANNEL_1_PINS)})"
            )
        if self.servo in HW_PWM_CHANNEL_0_PINS or self.servo in HW_PWM_CHANNEL_1_PINS:
            # GPIO18 is the default and is PWM0, the same channel as GPIO12.
            # That is fine only because the servo uses DMA-timed pulses, never
            # hardware PWM. Say so, so nobody "optimises" it later.
            _log.debug(
                "servo shares a hardware PWM channel with a motor enable; "
                "DMA-timed pulses are used, hardware PWM is not available here",
                servo=self.servo,
            )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "HardwarePins":
        motors = MotorPins(**_int_fields(data.get("motors", {}), MotorPins, "pins.motors"))
        encoders = EncoderPins(
            **_int_fields(data.get("encoders", {}), EncoderPins, "pins.encoders")
        )
        pins = cls(motors=motors, encoders=encoders)
        if "servo" in data:
            pins.servo = _as_int(data["servo"], "pins.servo")
        if "status_led" in data:
            pins.status_led = _as_optional_int(data["status_led"], "pins.status_led")
        if "estop_button" in data:
            pins.estop_button = _as_optional_int(data["estop_button"], "pins.estop_button")
        return pins

    def as_dict(self) -> dict[str, Any]:
        return {
            "motors": dataclasses.asdict(self.motors),
            "encoders": dataclasses.asdict(self.encoders),
            "servo": self.servo,
            "status_led": self.status_led,
            "estop_button": self.estop_button,
        }


def _as_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{label} must be an integer, got {value!r}")
    return value


def _as_optional_int(value: Any, label: str) -> int | None:
    if value is None:
        return None
    return _as_int(value, label)


def _int_fields(data: Any, kind: type, label: str) -> dict[str, int]:
    if not isinstance(data, dict):
        raise ConfigError(f"{label} must be a mapping, got {type(data).__name__}")
    valid = {f.name for f in dataclasses.fields(kind)}
    unknown = set(data) - valid
    if unknown:
        raise ConfigError(f"{label} has unknown keys: {sorted(unknown)}")
    return {name: _as_int(value, f"{label}.{name}") for name, value in data.items()}


# --------------------------------------------------------------------------
# Generated parameter fields
# --------------------------------------------------------------------------

_PY_TYPES: dict[str, type] = {"float": float, "int": int, "bool": bool, "enum": str}


def _param_field(definition: ParamDef) -> tuple[str, type, Any]:
    # repr=False keeps VehicleConfig's repr readable; there are ~50 parameters
    # and a repr that prints all of them is a repr nobody reads.
    return (
        definition.name,
        _PY_TYPES[definition.kind],
        field(default=definition.default, repr=False),
    )


_ParamsBase = dataclasses.make_dataclass(
    "_ParamsBase",
    [_param_field(d) for d in PARAMS.values()],
    namespace={
        "__doc__": "Generated from telekart_protocol.params.PARAMS. Do not edit by hand.",
    },
)


# --------------------------------------------------------------------------
# VehicleConfig
# --------------------------------------------------------------------------


@dataclass
class VehicleConfig(_ParamsBase):  # type: ignore[valid-type, misc]
    """Everything the firmware needs to know about this particular car.

    Constructing one with no arguments yields the registry defaults and the
    as-wired pin map, which is what every unit test should use -- no YAML, no
    filesystem, no hardware.
    """

    pins: HardwarePins = field(default_factory=HardwarePins)
    car_id: str = DEFAULT_CAR_ID
    shared_key: str = PLACEHOLDER_SHARED_KEY
    #: Where this came from, for the diagnostics banner.
    source_path: Path | None = field(default=None, repr=False)
    local_path: Path | None = field(default=None, repr=False)
    #: Top-level YAML keys this build does not recognise. Retained rather than
    #: dropped so `save_local` cannot silently delete a newer build's settings.
    extra: dict[str, Any] = field(default_factory=dict, repr=False)

    #: Parameter values as loaded from the base file, before local/live
    #: overrides. `save_local` writes the difference against this, so future
    #: changes to the checked-in defaults still reach cars that have a local
    #: overlay. Shadowed by an instance attribute in __post_init__.
    _base_params: ClassVar[dict[str, Any]] = {}

    def __post_init__(self) -> None:
        self._base_params = self.as_params()

    # -- loading ------------------------------------------------------------

    @classmethod
    def load(cls, path: Path, local: Path | None = None) -> "VehicleConfig":
        """Read the base config, then overlay the local file if one is given.

        A missing base file is fatal -- it means a broken deployment. A missing
        local file is normal: most cars never need one.
        """
        path = Path(path)
        if not path.is_file():
            raise ConfigError(f"config file not found: {path}")

        base = _read_yaml(path)
        config = cls.from_dict(base, source_path=path)
        config._base_params = config.as_params()

        if local is not None:
            local = Path(local)
            if local.is_file():
                overlay = _read_yaml(local)
                config.merge_dict(overlay, origin=str(local))
                config.local_path = local
                _log.info("local config overlay applied", path=str(local))
            else:
                _log.debug("no local config overlay", path=str(local))

        config._apply_environment()
        config.check()
        return config

    @classmethod
    def load_default(cls) -> "VehicleConfig":
        """Load the standard pair of files, honouring TELEKART_CONFIG and
        TELEKART_LOCAL_CONFIG. This is what the daemons call; `load` stays
        explicit for tests and tools that point at a fixture."""
        base = Path(os.environ.get(ENV_CONFIG) or DEFAULT_CONFIG_PATH)
        local = Path(os.environ.get(ENV_LOCAL_CONFIG) or LOCAL_CONFIG_PATH)
        return cls.load(base, local)

    @classmethod
    def from_dict(
        cls, data: dict[str, Any], *, source_path: Path | None = None
    ) -> "VehicleConfig":
        if not isinstance(data, dict):
            raise ConfigError("config root must be a mapping")

        pins = HardwarePins.from_dict(data.get("pins", {}) or {})
        config = cls(pins=pins, source_path=source_path)

        raw_params = _collect_params(data, origin=str(source_path or "<dict>"))
        bad = unknown_or_invalid(raw_params)
        if bad:
            # Refusing to boot over one stale key in a config file written by a
            # newer build helps nobody; the default is a safe value by design.
            _log.warning(
                "ignoring unusable parameters from config",
                path=str(source_path or "<dict>"),
                problems=", ".join(bad),
            )
        for name, value in merged_with_defaults(raw_params).items():
            setattr(config, name, value)

        if "car_id" in data:
            config.car_id = _as_str(data["car_id"], "car_id")
        if "shared_key" in data:
            config.shared_key = _as_str(data["shared_key"], "shared_key")

        config.extra = {
            key: value
            for key, value in data.items()
            if key not in _KNOWN_TOP_LEVEL and key not in PARAMS
        }
        if config.extra:
            _log.debug("config has keys this build ignores", keys=sorted(config.extra))
        config._base_params = config.as_params()
        return config

    def merge_dict(self, data: dict[str, Any], *, origin: str = "<overlay>") -> list[str]:
        """Overlay a partial config. Returns the parameter names that changed."""
        if not isinstance(data, dict):
            raise ConfigError(f"{origin}: overlay must be a mapping")

        pin_overlay = data.get("pins")
        if pin_overlay:
            if not isinstance(pin_overlay, dict):
                raise ConfigError(f"{origin}: 'pins' must be a mapping")
            merged = self.pins.as_dict()
            for section in ("motors", "encoders"):
                section_overlay = pin_overlay.get(section)
                if section_overlay is not None:
                    if not isinstance(section_overlay, dict):
                        raise ConfigError(f"{origin}: 'pins.{section}' must be a mapping")
                    merged[section].update(section_overlay)
            for key in ("servo", "status_led", "estop_button"):
                if key in pin_overlay:
                    merged[key] = pin_overlay[key]
            self.pins = HardwarePins.from_dict(merged)

        if "car_id" in data:
            self.car_id = _as_str(data["car_id"], "car_id")
        if "shared_key" in data:
            self.shared_key = _as_str(data["shared_key"], "shared_key")

        raw_params = _collect_params(data, origin=origin)
        bad = unknown_or_invalid(raw_params)
        if bad:
            _log.warning("ignoring unusable parameters from overlay",
                         origin=origin, problems=", ".join(bad))
        changed: list[str] = []
        for name, value in raw_params.items():
            if name not in PARAMS:
                continue
            try:
                coerced = coerce(name, value)
            except ParamError:
                continue
            if getattr(self, name) != coerced:
                setattr(self, name, coerced)
                changed.append(name)

        for key, value in data.items():
            if key not in _KNOWN_TOP_LEVEL and key not in PARAMS:
                self.extra[key] = value
        return changed

    def _apply_environment(self) -> None:
        """Environment beats file. This is how the shared key stays out of git:
        the systemd unit reads it from an EnvironmentFile with 0600 permissions."""
        key = os.environ.get(ENV_SHARED_KEY)
        if key:
            self.shared_key = key
            _log.info("shared key taken from the environment", variable=ENV_SHARED_KEY)
        car_id = os.environ.get(ENV_CAR_ID)
        if car_id:
            self.car_id = car_id

    # -- validation ---------------------------------------------------------

    def check(self) -> None:
        """Fail loudly on anything that cannot work. Load time only."""
        self.pins.validate()

        if not self.car_id or not self.car_id.strip():
            raise ConfigError("car_id must not be empty")
        if not self.shared_key:
            raise ConfigError("shared_key must not be empty")
        if self.shared_key == PLACEHOLDER_SHARED_KEY:
            # Not fatal: bench work with a mock backend is legitimate. But an
            # unattended car on the shared campus network with the published
            # default key is a real hazard, so it is a warning every boot.
            _log.warning(
                "shared_key is still the checked-in placeholder; anyone on the "
                "network can drive this car",
                fix=f"set shared_key in config.local.yaml or export {ENV_SHARED_KEY}",
            )

        if self.steer_min_us >= self.steer_max_us:
            raise ConfigError(
                f"steer_min_us ({self.steer_min_us}) must be below "
                f"steer_max_us ({self.steer_max_us})"
            )
        if not self.steer_min_us <= self.steer_center_us <= self.steer_max_us:
            raise ConfigError(
                f"steer_center_us ({self.steer_center_us}) must lie between "
                f"steer_min_us ({self.steer_min_us}) and steer_max_us ({self.steer_max_us})"
            )
        if self.critical_battery_v > self.low_battery_v > 0.0:
            raise ConfigError(
                f"critical_battery_v ({self.critical_battery_v}) must not exceed "
                f"low_battery_v ({self.low_battery_v})"
            )
        if self.duty_sum_max < self.max_duty:
            raise ConfigError(
                f"duty_sum_max ({self.duty_sum_max}) is below max_duty ({self.max_duty}); "
                "one motor alone would already exceed the combined budget"
            )
        if self.wheel_diameter_m <= 0.0 or self.wheelbase_m <= 0.0 or self.track_width_m <= 0.0:
            raise ConfigError("geometry must be positive")
        if self.encoder_cpr <= 0:
            raise ConfigError("encoder_cpr must be positive")

    # -- parameters ---------------------------------------------------------

    def apply_params(self, values: dict[str, Any]) -> list[str]:
        """Apply a parameter push. Returns the names whose value actually changed.

        Raises :class:`telekart_protocol.params.ParamError` listing *every*
        problem when any value is invalid, and applies nothing -- a half-applied
        parameter set is a car in a state nobody chose. The session handler
        turns that into a PARAM_OUT_OF_RANGE reply.
        """
        coerced = coerce_all(values)
        changed: list[str] = []
        for name, value in coerced.items():
            if getattr(self, name) != value:
                setattr(self, name, value)
                changed.append(name)
        if changed:
            _log.info("parameters applied", count=len(changed), names=",".join(changed))
        return changed

    def as_params(self) -> dict[str, Any]:
        """The full parameter set, in registry order."""
        return {name: getattr(self, name) for name in PARAMS}

    def changed_params(self) -> dict[str, Any]:
        """Parameters differing from the base file this config was loaded from."""
        base = self._base_params
        return {
            name: value
            for name, value in self.as_params().items()
            if name not in base or base[name] != value
        }

    def reset_params(self) -> list[str]:
        """Restore registry defaults. Used by the session channel's factory reset."""
        return self.apply_params(defaults())

    @staticmethod
    def param_names() -> tuple[str, ...]:
        return tuple(PARAMS)

    @staticmethod
    def requires_disarm(names: Iterable[str]) -> list[str]:
        """Which of ``names`` may not be changed while the car is armed.

        Lives here rather than in the session handler so there is one place that
        knows the rule, and it is next to the code that applies the values.
        """
        return [n for n in names if n in PARAMS and PARAMS[n].requires_disarm]

    # -- derived quantities -------------------------------------------------

    @property
    def wheel_circumference_m(self) -> float:
        return _wheel_circumference_m(self.wheel_diameter_m)

    @property
    def steer_max_rad(self) -> float:
        from math import radians

        return radians(self.steer_max_deg)

    @property
    def control_timeout_s(self) -> float:
        return self.control_timeout_ms / 1000.0

    @property
    def arm_neutral_s(self) -> float:
        return self.arm_neutral_ms / 1000.0

    @property
    def stall_detect_s(self) -> float:
        return self.stall_detect_ms / 1000.0

    @property
    def direction_deadtime_s(self) -> float:
        return self.direction_deadtime_ms / 1000.0

    def speed_for_rpm(self, rpm: float) -> float:
        """Wheel RPM to ground speed in m/s, ignoring slip."""
        return rpm_to_mps(rpm, self.wheel_diameter_m)

    # -- persistence --------------------------------------------------------

    def save_local(self, path: Path = LOCAL_CONFIG_PATH) -> None:
        """Write the *difference* from the base config to the local overlay.

        Writing only the delta is what lets a fleet share improvements: a car
        with a measured steer trim in its local file still picks up a new
        default PID gain from the checked-in file on the next update. Writing
        the whole set would pin every value forever.
        """
        path = Path(path)
        payload: dict[str, Any] = {}
        if self.car_id != DEFAULT_CAR_ID:
            payload["car_id"] = self.car_id
        if self.shared_key not in (PLACEHOLDER_SHARED_KEY, ""):
            payload["shared_key"] = self.shared_key
        changed = self.changed_params()
        if changed:
            payload["params"] = changed
        for key, value in self.extra.items():
            payload.setdefault(key, value)

        header = (
            "# TeleKart per-vehicle overlay -- machine written, safe to hand-edit.\n"
            "# Only values that differ from config/telekart.yaml appear here, so\n"
            "# improvements to the checked-in defaults still reach this car.\n"
            "# This file is git-ignored: it holds measurements and the shared key.\n"
        )
        _write_yaml_atomic(path, payload, header=header, secret=bool(payload.get("shared_key")))
        self.local_path = path
        _log.info("local config written", path=str(path), keys=len(payload))

    # -- diagnostics --------------------------------------------------------

    def summary(self) -> str:
        return (
            f"car_id={self.car_id} "
            f"pins(ena={self.pins.motors.ena},enb={self.pins.motors.enb},"
            f"servo={self.pins.servo}) "
            f"cpr={self.encoder_cpr} wheel={self.wheel_diameter_m:.3f}m "
            f"wheelbase={self.wheelbase_m:.3f}m track={self.track_width_m:.3f}m "
            f"max_duty={self.max_duty:.2f} duty_sum_max={self.duty_sum_max:.2f} "
            f"closed_loop={self.closed_loop}"
        )

    def __repr__(self) -> str:
        return f"VehicleConfig({self.summary()})"


_KNOWN_TOP_LEVEL = frozenset({"pins", "params", "car_id", "shared_key"})


def _collect_params(data: dict[str, Any], *, origin: str) -> dict[str, Any]:
    """Pull parameter values out of a config mapping.

    Accepts three shapes, because all three are natural to write by hand and
    rejecting two of them would only produce support questions::

        params: {max_duty: 0.8}                # flat
        params: {drive: {max_duty: 0.8}}       # grouped, as in the shipped file
        max_duty: 0.8                          # top level
    """
    found: dict[str, Any] = {}

    def absorb(mapping: dict[str, Any], allow_groups: bool) -> None:
        for key, value in mapping.items():
            if key in PARAMS:
                found[key] = value
            elif allow_groups and isinstance(value, dict):
                for inner_key, inner_value in value.items():
                    if inner_key in PARAMS:
                        found[inner_key] = inner_value
                    else:
                        _log.debug("ignoring unknown config key",
                                   origin=origin, key=f"{key}.{inner_key}")
            elif allow_groups:
                _log.debug("ignoring unknown config key", origin=origin, key=key)

    params_node = data.get("params")
    if params_node is not None:
        if not isinstance(params_node, dict):
            raise ConfigError(f"{origin}: 'params' must be a mapping")
        absorb(params_node, allow_groups=True)

    absorb({k: v for k, v in data.items() if k not in _KNOWN_TOP_LEVEL}, allow_groups=False)
    return found


def _as_str(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise ConfigError(f"{label} must be a string, got {type(value).__name__}")
    return value


# --------------------------------------------------------------------------
# YAML
# --------------------------------------------------------------------------


def _require_yaml() -> Any:
    """Import PyYAML with an actionable message when it is missing.

    Deferred so that ``import telekart.config`` works on a machine with no
    PyYAML -- ``VehicleConfig()`` with registry defaults is enough for the whole
    Mac-side test suite, and requiring a dependency for that would be rude.
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - environment dependent
        raise ConfigError(
            "PyYAML is required to read or write config files. "
            "Install it with 'sudo apt install python3-yaml' on the Pi, "
            "or 'pip install pyyaml' elsewhere."
        ) from exc
    return yaml


def _read_yaml(path: Path) -> dict[str, Any]:
    yaml = _require_yaml()
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"cannot read {path}: {exc}") from exc
    try:
        data = yaml.safe_load(text)
    except Exception as exc:  # yaml.YAMLError, but do not import for the name
        raise ConfigError(f"{path} is not valid YAML: {exc}") from exc
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f"{path} must contain a mapping at the top level")
    return data


def _write_yaml_atomic(
    path: Path, payload: dict[str, Any], *, header: str = "", secret: bool = False
) -> None:
    """Write via a temporary file and rename.

    The car is powered down by flipping a battery switch. A half-written config
    file is a car that will not boot next time, and rename() is atomic on ext4.
    """
    yaml = _require_yaml()
    path.parent.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump(payload, default_flow_style=False, sort_keys=True)
    handle = tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=str(path.parent), prefix=f".{path.name}.", delete=False
    )
    tmp = Path(handle.name)
    try:
        with handle:
            handle.write(header)
            handle.write(body)
            handle.flush()
            os.fsync(handle.fileno())
        if secret:
            os.chmod(tmp, stat.S_IRUSR | stat.S_IWUSR)
        os.replace(tmp, path)
    except BaseException:
        tmp.unlink(missing_ok=True)
        raise
