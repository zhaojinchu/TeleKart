"""Parameter registry contract tests.

The registry is consumed three ways: the firmware validates SET_PARAMS against
it, `VehicleConfig` generates one attribute per entry, and the desktop tuning UI
builds its widgets from the metadata. A malformed entry therefore fails at three
different times in three different processes, none of them near the typo. These
tests catch it once, here.
"""

from __future__ import annotations

import math

import pytest

from telekart_protocol.params import (
    GROUPS,
    PARAMS,
    ParamDef,
    ParamError,
    coerce,
    coerce_all,
    defaults,
    group,
    merged_with_defaults,
    unknown_or_invalid,
)
from telekart_protocol.session import Message, MsgType, set_params

NUMERIC_KINDS = ("float", "int")
ALL_KINDS = ("float", "int", "bool", "enum")


def _numeric_params() -> list[ParamDef]:
    return [d for d in PARAMS.values() if d.kind in NUMERIC_KINDS]


# ------------------------------------------------------- registry shape


def test_registry_is_not_empty() -> None:
    assert len(PARAMS) > 40


@pytest.mark.parametrize("name", sorted(PARAMS))
def test_key_matches_definition_name(name: str) -> None:
    # PARAMS is built by comprehension, so a mismatch here means a duplicate
    # name silently overwrote an earlier entry.
    assert PARAMS[name].name == name


def test_no_duplicate_names() -> None:
    assert len(PARAMS) == len({d.name for d in PARAMS.values()})


@pytest.mark.parametrize("definition", list(PARAMS.values()), ids=lambda d: d.name)
def test_every_definition_is_well_formed(definition: ParamDef) -> None:
    assert definition.kind in ALL_KINDS
    assert definition.group in GROUPS
    assert definition.label
    assert definition.name.islower()
    assert " " not in definition.name


@pytest.mark.parametrize("definition", _numeric_params(), ids=lambda d: d.name)
def test_numeric_definitions_have_a_usable_range(definition: ParamDef) -> None:
    assert definition.minimum is not None
    assert definition.maximum is not None
    assert definition.minimum < definition.maximum
    assert definition.step is not None and definition.step > 0
    # A UI slider spanning the range in fewer than 2 steps is not a slider.
    assert (definition.maximum - definition.minimum) >= definition.step


@pytest.mark.parametrize("definition", list(PARAMS.values()), ids=lambda d: d.name)
def test_every_default_is_valid_against_its_own_definition(definition: ParamDef) -> None:
    # The single most valuable invariant in this file: a default outside its own
    # range boots the car with a value the operator can never re-enter.
    assert coerce(definition.name, definition.default) == definition.default


def test_defaults_survive_a_full_validation_pass() -> None:
    assert coerce_all(defaults()) == defaults()
    assert unknown_or_invalid(defaults()) == []


def test_enum_definitions_are_coherent() -> None:
    for definition in PARAMS.values():
        if definition.kind != "enum":
            continue
        assert definition.choices, f"{definition.name} has no choices"
        assert definition.default in definition.choices
        assert len(set(definition.choices)) == len(definition.choices)


def test_bool_definitions_carry_no_range() -> None:
    for definition in PARAMS.values():
        if definition.kind != "bool":
            continue
        assert definition.minimum is None and definition.maximum is None
        assert isinstance(definition.default, bool)


def test_groups_partition_the_registry() -> None:
    covered: set[str] = set()
    for name in GROUPS:
        members = group(name)
        assert members, f"group {name!r} is empty"
        covered.update(d.name for d in members)
    assert covered == set(PARAMS)


def test_group_preserves_declaration_order() -> None:
    # The tuning UI renders each group in registry order, which is how related
    # settings stay next to each other without a second ordering table.
    ordered = [d.name for d in PARAMS.values() if d.group == "steering"]
    assert [d.name for d in group("steering")] == ordered


def test_group_of_an_unknown_name_is_empty() -> None:
    assert group("no-such-group") == []


# --------------------------------------------- load-bearing definitions


def test_the_parameters_other_workstreams_depend_on_exist() -> None:
    # Named individually because each one is referenced by name from firmware or
    # UI code that cannot be type-checked against this registry.
    required = {
        "max_duty": "float",
        "duty_sum_max": "float",
        "closed_loop": "bool",
        "accel_rpm_per_s": "float",
        "brake_strength": "float",
        "pid_kp": "float",
        "pid_ki": "float",
        "pid_kd": "float",
        "steer_center_us": "int",
        "steer_min_us": "int",
        "steer_max_us": "int",
        "steer_max_deg": "float",
        "steer_hold_us": "int",
        "wheel_diameter_m": "float",
        "wheelbase_m": "float",
        "track_width_m": "float",
        "encoder_cpr": "int",
        "control_timeout_ms": "int",
        "stall_detect_ms": "int",
        "arm_neutral_ms": "int",
        "pwm_hz": "int",
        "direction_deadtime_ms": "int",
        "reverse_allowed_rpm": "float",
        "video_codec": "enum",
        "video_fps": "int",
        "video_iperiod": "int",
    }
    for name, kind in required.items():
        assert name in PARAMS, f"{name} disappeared from the registry"
        assert PARAMS[name].kind == kind


def test_geometry_and_calibration_values_are_marked_measured() -> None:
    # `measured=True` is what tells the UI to warn instead of pretending the
    # default is a real number. Treating a guess as a measurement is how you
    # tune a PID against the wrong plant gain.
    for name in ("steer_center_us", "steer_max_deg", "wheel_diameter_m", "encoder_cpr"):
        assert PARAMS[name].measured is True


def test_geometry_changes_require_a_disarm() -> None:
    # Changing the wheelbase mid-drive would step the odometry model and the
    # electronic differential at once.
    for name in ("wheel_diameter_m", "wheelbase_m", "track_width_m", "encoder_cpr", "pwm_hz"):
        assert PARAMS[name].requires_disarm is True


def test_video_reconfiguration_requires_a_disarm() -> None:
    for definition in group("video"):
        assert definition.requires_disarm is True


def test_advanced_parameters_are_a_minority() -> None:
    # The advanced toggle stops being useful if most of the registry is behind
    # it; this is a design constraint, not a style preference.
    advanced = sum(1 for d in PARAMS.values() if d.advanced)
    assert 0 < advanced < len(PARAMS) / 2


def test_duty_budget_is_below_the_sum_of_two_full_motors() -> None:
    # The boost regulator sustains roughly 1.5 A for both motors combined, and
    # it is simultaneous demand that trips it. A default budget of 2.0 would
    # mean the parameter does nothing.
    assert PARAMS["duty_sum_max"].default < 2 * PARAMS["max_duty"].default


def test_steering_defaults_bracket_the_centre() -> None:
    assert PARAMS["steer_min_us"].default < PARAMS["steer_center_us"].default
    assert PARAMS["steer_center_us"].default < PARAMS["steer_max_us"].default


def test_battery_thresholds_are_ordered() -> None:
    assert PARAMS["critical_battery_v"].default < PARAMS["low_battery_v"].default


def test_deceleration_is_allowed_to_exceed_acceleration() -> None:
    # Slowing down does not draw current from the regulator, so the two limits
    # are deliberately asymmetric.
    assert PARAMS["decel_rpm_per_s"].default > PARAMS["accel_rpm_per_s"].default


# ---------------------------------------------------- coerce: accepting


def test_coerce_accepts_the_exact_boundaries() -> None:
    for definition in _numeric_params():
        assert definition.minimum is not None and definition.maximum is not None
        assert coerce(definition.name, definition.minimum) == definition.minimum
        assert coerce(definition.name, definition.maximum) == definition.maximum


def test_coerce_normalizes_int_parameters_to_int() -> None:
    value = coerce("encoder_cpr", 660.0)
    assert value == 660
    assert isinstance(value, int) and not isinstance(value, bool)


def test_coerce_normalizes_float_parameters_to_float() -> None:
    value = coerce("max_duty", 1)
    assert isinstance(value, float)
    assert value == 1.0


def test_coerce_accepts_valid_enum_and_bool() -> None:
    assert coerce("video_codec", "mjpeg") == "mjpeg"
    assert coerce("closed_loop", False) is False


# ---------------------------------------------------- coerce: rejecting


def test_unknown_parameter_is_rejected() -> None:
    with pytest.raises(ParamError, match="unknown parameter"):
        coerce("max_dutyy", 0.5)


@pytest.mark.parametrize("name", ["max_duty", "encoder_cpr", "pwm_hz", "steer_max_deg"])
def test_below_minimum_is_rejected(name: str) -> None:
    minimum = PARAMS[name].minimum
    assert minimum is not None
    with pytest.raises(ParamError, match="below the minimum"):
        coerce(name, minimum - 1)


@pytest.mark.parametrize("name", ["max_duty", "encoder_cpr", "pwm_hz", "steer_max_deg"])
def test_above_maximum_is_rejected(name: str) -> None:
    maximum = PARAMS[name].maximum
    assert maximum is not None
    with pytest.raises(ParamError, match="above the maximum"):
        coerce(name, maximum + 1)


def test_out_of_range_is_rejected_rather_than_clamped() -> None:
    # The opposite of the control path on purpose. A control packet's axis value
    # is clamped because dropping it is worse; a parameter push is deliberate,
    # and quietly storing a different number than the operator typed is how a UI
    # ends up lying about which vehicle it is talking to.
    with pytest.raises(ParamError):
        coerce("max_duty", 5.0)


def test_fractional_value_for_an_int_parameter_is_rejected() -> None:
    with pytest.raises(ParamError, match="whole number"):
        coerce("encoder_cpr", 660.5)
    with pytest.raises(ParamError, match="whole number"):
        coerce("pwm_hz", 1000.25)


def test_bool_is_not_accepted_as_a_number() -> None:
    # `isinstance(True, int)` is True in Python, so this guard is the only thing
    # standing between a checkbox bug and max_duty silently becoming 1.0.
    with pytest.raises(ParamError, match="must be numeric"):
        coerce("max_duty", True)
    with pytest.raises(ParamError, match="must be numeric"):
        coerce("encoder_cpr", False)


def test_number_is_not_accepted_as_a_bool() -> None:
    with pytest.raises(ParamError, match="must be a boolean"):
        coerce("closed_loop", 1)
    with pytest.raises(ParamError, match="must be a boolean"):
        coerce("closed_loop", "true")


@pytest.mark.parametrize("bad", ["mp4", "H264", "", 0, None, True])
def test_invalid_enum_choice_is_rejected(bad: object) -> None:
    with pytest.raises(ParamError, match="must be one of"):
        coerce("video_codec", bad)


@pytest.mark.parametrize("bad", ["0.5", None, [], {}, (1,)])
def test_non_numeric_types_are_rejected(bad: object) -> None:
    with pytest.raises(ParamError, match="must be numeric"):
        coerce("max_duty", bad)


@pytest.mark.parametrize("bad", [math.inf, -math.inf])
def test_infinities_are_rejected_by_the_range_check(bad: float) -> None:
    with pytest.raises(ParamError):
        coerce("max_duty", bad)


def test_nan_is_stopped_by_the_encoder_not_by_the_range_check() -> None:
    """A known gap, pinned so it cannot be discovered the hard way.

    NaN compares False against both bounds, so `coerce` accepts it. What stops a
    NaN reaching the car is one layer up: `Message.encode` uses
    ``allow_nan=False`` and refuses to serialise it. The gap is therefore only
    reachable from a locally-parsed source -- YAML spells NaN as ``.nan`` --
    which is why a config loader must screen for finiteness itself rather than
    trusting `coerce` to have done it.
    """
    assert math.isnan(coerce("max_duty", math.nan))
    with pytest.raises(ValueError):
        set_params(1, {"max_duty": math.nan}).encode()


def test_param_error_is_a_value_error() -> None:
    assert issubclass(ParamError, ValueError)


# --------------------------------------------------------- coerce_all


def test_coerce_all_accepts_a_valid_subset() -> None:
    assert coerce_all({"max_duty": 0.6, "closed_loop": True}) == {
        "max_duty": 0.6,
        "closed_loop": True,
    }


def test_coerce_all_reports_every_problem_at_once() -> None:
    # One round trip per broken field would make the operator fix a form one
    # widget at a time.
    with pytest.raises(ParamError) as excinfo:
        coerce_all({"max_duty": 9.0, "encoder_cpr": 0.5, "nope": 1, "closed_loop": True})
    message = str(excinfo.value)
    assert "max_duty" in message
    assert "encoder_cpr" in message
    assert "nope" in message


def test_coerce_all_of_an_empty_dict_is_empty() -> None:
    assert coerce_all({}) == {}


# -------------------------------------------------- merge and reporting


def test_merged_with_defaults_overlays_valid_values() -> None:
    merged = merged_with_defaults({"max_duty": 0.5})
    assert merged["max_duty"] == 0.5
    assert merged["closed_loop"] == PARAMS["closed_loop"].default
    assert set(merged) == set(PARAMS)


def test_merged_with_defaults_ignores_unknown_keys() -> None:
    # A config file written by a newer build is a reason to log, not a reason to
    # refuse to boot a car sitting on a bench.
    merged = merged_with_defaults({"from_the_future": 1, "max_duty": 0.5})
    assert "from_the_future" not in merged
    assert merged["max_duty"] == 0.5


def test_merged_with_defaults_falls_back_on_invalid_values() -> None:
    merged = merged_with_defaults({"max_duty": 99.0, "pwm_hz": "fast"})
    assert merged["max_duty"] == PARAMS["max_duty"].default
    assert merged["pwm_hz"] == PARAMS["pwm_hz"].default


def test_unknown_or_invalid_names_every_problem() -> None:
    problems = unknown_or_invalid({"max_duty": 0.5, "bogus": 1, "encoder_cpr": -3})
    joined = " ".join(problems)
    assert "bogus (unknown)" in problems
    assert "encoder_cpr" in joined
    assert "max_duty" not in joined


def test_defaults_returns_a_fresh_dict() -> None:
    first = defaults()
    first["max_duty"] = 0.0
    assert defaults()["max_duty"] == PARAMS["max_duty"].default


# ------------------------------------------------ session interop


def test_a_validated_param_push_survives_the_session_channel() -> None:
    # The two halves meet here: the app validates locally, encodes, and the car
    # decodes and validates again. Both validations must agree.
    values = coerce_all(
        {
            "max_duty": 0.75,
            "closed_loop": True,
            "video_codec": "mjpeg",
            "encoder_cpr": 660,
            "steer_trim_us": -12,
        }
    )
    decoded = Message.decode(set_params(4, values).encode())
    assert decoded.type is MsgType.SET_PARAMS
    assert coerce_all(decoded.require("values", dict)) == values


def test_the_whole_default_set_fits_in_one_session_line() -> None:
    # GET_PARAMS answers with every parameter at once; if that ever exceeded the
    # line limit the app would fail to load its own tuning page.
    line = Message(MsgType.PARAMS, 1, {"values": defaults(), "applied": True}).encode()
    assert len(line) < 8192
    assert Message.decode(line).require("values", dict).keys() == defaults().keys()
