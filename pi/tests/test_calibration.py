"""The measured drivetrain description, and the file it lives in.

``calibration.yaml`` is machine-written and deliberately separate from
``config.yaml`` so an automated bring-up run can never clobber a value somebody
typed on purpose. Everything downstream reads ``max_rpm_measured`` -- the
speedometer, the mixer, the feedforward -- which is what lets the same build
work with today's L298N-limited drivetrain and with a MOSFET bridge later,
with no code change on either side.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from telekart.calibration import WHEEL_KEYS, DriveCalibration

FF_TABLE: list[tuple[float, float]] = [
    (0.0, 0.0),
    (30.0, 0.14),
    (60.0, 0.28),
    (90.0, 0.42),
    (140.0, 0.63),
]


def build(**overrides: object) -> DriveCalibration:
    values: dict[str, object] = {
        "max_rpm": {"left_fwd": 150.0, "left_rev": 142.0, "right_fwd": 148.0, "right_rev": 139.0},
        "deadband": {key: 0.22 for key in WHEEL_KEYS},
        "ff_lut": {key: list(FF_TABLE) for key in WHEEL_KEYS},
        "measured_at": "2026-08-16T11:02:00Z",
        "on_ground": False,
    }
    values.update(overrides)
    return DriveCalibration(**values)  # type: ignore[arg-type]


# --------------------------------------------------------------------------
# max_rpm_measured
# --------------------------------------------------------------------------


def test_max_rpm_measured_is_the_worst_corner() -> None:
    """The minimum across all four, never the mean. One optimistic corner must
    not be able to inflate a target the other three can never reach."""
    calibration = build()
    assert calibration.max_rpm_measured == pytest.approx(139.0)


def test_max_rpm_measured_survives_a_partial_table() -> None:
    calibration = build(max_rpm={"left_fwd": 120.0, "right_fwd": 130.0})
    assert calibration.max_rpm_measured == pytest.approx(120.0)


def test_an_empty_calibration_does_not_divide_by_zero() -> None:
    """A calibration that failed halfway must not hand the speedometer a NaN."""
    calibration = build(max_rpm={})
    assert math.isfinite(calibration.max_rpm_measured)
    assert calibration.max_rpm_measured >= 0.0


# --------------------------------------------------------------------------
# Feedforward
# --------------------------------------------------------------------------


def test_feedforward_is_zero_at_rest() -> None:
    calibration = build()
    assert calibration.feedforward("left", 0.0) == pytest.approx(0.0, abs=1e-9)


def test_feedforward_interpolates_between_breakpoints() -> None:
    calibration = build()
    midpoint = calibration.feedforward("left", 45.0)
    assert midpoint == pytest.approx(0.21, abs=0.01)


def test_feedforward_hits_the_breakpoints_exactly() -> None:
    calibration = build()
    for rpm, duty in FF_TABLE:
        assert calibration.feedforward("right", rpm) == pytest.approx(duty, abs=1e-6)


def test_feedforward_is_monotonic() -> None:
    """A fold in the feedforward surface makes the PID hunt across it forever,
    which looks exactly like a tuning problem and is not one."""
    calibration = build()
    previous = -1.0
    for step in range(0, 161, 5):
        value = calibration.feedforward("left", float(step))
        assert value >= previous - 1e-9
        previous = value


def test_feedforward_is_bounded_beyond_the_table() -> None:
    calibration = build()
    beyond = calibration.feedforward("left", 10_000.0)
    assert 0.0 <= beyond <= 1.0


def test_feedforward_follows_the_sign_of_the_target() -> None:
    """Reverse is a direction, not a separate mode: a negative RPM target has to
    produce a negative duty or the PID spends the whole reversal saturated."""
    calibration = build()
    forward = calibration.feedforward("left", 60.0)
    backward = calibration.feedforward("left", -60.0)
    assert forward > 0.0
    assert backward < 0.0
    assert abs(backward) == pytest.approx(forward, rel=0.2)


def test_feedforward_of_an_unknown_wheel_is_survivable() -> None:
    """This is read on every control tick. A typo upstream must cost duty, not
    the control thread."""
    calibration = build()
    value = calibration.feedforward("middle", 50.0)
    assert math.isfinite(value)
    assert -1.0 <= value <= 1.0


# --------------------------------------------------------------------------
# Persistence
# --------------------------------------------------------------------------


def test_save_and_load_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "calibration.yaml"
    original = build()
    original.save(path)
    assert path.is_file()

    loaded = DriveCalibration.load(path)
    assert loaded is not None
    assert loaded.max_rpm_measured == pytest.approx(original.max_rpm_measured)
    assert loaded.on_ground is False
    assert loaded.measured_at == original.measured_at
    for rpm, _duty in FF_TABLE:
        assert loaded.feedforward("left", rpm) == pytest.approx(
            original.feedforward("left", rpm), abs=1e-6
        )


def test_on_ground_survives_the_round_trip(tmp_path: Path) -> None:
    """The bench figure is optimistic by 20-40 %. Recording which condition
    produced a number is what makes it interpretable three weeks later."""
    path = tmp_path / "calibration.yaml"
    build(on_ground=True).save(path)
    loaded = DriveCalibration.load(path)
    assert loaded is not None
    assert loaded.on_ground is True


def test_loading_a_missing_file_returns_none(tmp_path: Path) -> None:
    """An uncalibrated car boots, refuses to arm, and says CALIBRATION_MISSING.
    It does not crash on startup."""
    assert DriveCalibration.load(tmp_path / "not-here.yaml") is None


def test_loading_a_corrupt_file_returns_none(tmp_path: Path) -> None:
    """Power is removed from this car by flipping a switch, so a half-written
    file is a real possibility rather than a theoretical one."""
    path = tmp_path / "calibration.yaml"
    path.write_text("max_rpm: {left_fwd: 150.0\nthis is not yaml at all: [", encoding="utf-8")
    assert DriveCalibration.load(path) is None


def test_loading_a_file_of_the_wrong_shape_returns_none(tmp_path: Path) -> None:
    path = tmp_path / "calibration.yaml"
    path.write_text("- just\n- a\n- list\n", encoding="utf-8")
    assert DriveCalibration.load(path) is None


def test_save_creates_missing_directories(tmp_path: Path) -> None:
    path = tmp_path / "nested" / "deeper" / "calibration.yaml"
    build().save(path)
    assert path.is_file()


def test_saving_twice_replaces_rather_than_appends(tmp_path: Path) -> None:
    path = tmp_path / "calibration.yaml"
    build().save(path)
    build(max_rpm={key: 100.0 for key in WHEEL_KEYS}).save(path)
    loaded = DriveCalibration.load(path)
    assert loaded is not None
    assert loaded.max_rpm_measured == pytest.approx(100.0)


# --------------------------------------------------------------------------
# The fixture the rest of the suite drives
# --------------------------------------------------------------------------


def test_the_synthetic_calibration_describes_the_mock_plant(
    calibration: DriveCalibration,
) -> None:
    """If this drifts away from the plant, every closed-loop test in the suite
    is quietly measuring a feedforward table that describes a different car."""
    assert calibration.max_rpm_measured == pytest.approx(139.0)
    for key in WHEEL_KEYS:
        assert 0.05 <= calibration.deadband[key] <= 0.30
    top = calibration.feedforward("left", 139.0)
    assert top == pytest.approx(0.63, abs=0.05)
