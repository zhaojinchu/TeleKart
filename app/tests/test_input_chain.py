"""Property tests for the input chain.

The chain is the one piece of the desktop app that sits directly between the
driver's hands and the car, and it is pure, so it can be tested to a standard
the rest of the app cannot. These are properties rather than examples: for a few
hundred randomly generated configurations, the output is always in range, always
monotonic in the input, exactly zero inside the deadzone, exactly full scale at
saturation, and the lookup table always agrees with the curve it was built from.

Runs under pytest, and also standalone (`python3 test_input_chain.py`) so the
chain can be checked in an environment where nothing is installed yet.
"""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
for _candidate in (
    _HERE.parents[1],  # app/
    _HERE.parents[2] / "packages" / "telekart_protocol",
):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from telekart_app.input.chain import (  # noqa: E402
    MAX_DT,
    AxisCalibration,
    AxisChainConfig,
    ChainConfig,
    ChainConfigError,
    InputChain,
    OneEuroConfig,
    OneEuroFilter,
    calibrate_bipolar,
    calibrate_unipolar,
    center_deadzone,
    default_chain_config,
    floor_deadzone,
    quantize,
    rate_limit,
    saturate,
    speed_scale,
    static_map,
)
from telekart_app.input.curves import (  # noqa: E402
    LUT_SIZE,
    Curve,
    CurveKind,
    CurveSpec,
    build_lut,
    evaluate,
    lut_error_bound,
)
from telekart_app.input.mapping import Control  # noqa: E402
from telekart_app.input.profile import identity_profile, preset_digital_wheel  # noqa: E402
from telekart_app.input.sources import ScriptSource  # noqa: E402
from telekart_protocol.constants import (  # noqa: E402
    BRAKE_SCALE,
    STEERING_SCALE,
    THROTTLE_SCALE,
)
from telekart_protocol.control import ControlPacket  # noqa: E402

SEED = 20260816
CASES = 240
SWEEP = 401


# --------------------------------------------------------------------------
# Generators
# --------------------------------------------------------------------------


def _random_curve(rng: random.Random, *, well_behaved: bool = False) -> Curve:
    kind = rng.choice(list(CurveKind))
    if kind is CurveKind.LINEAR:
        return Curve(CurveSpec.linear())
    if kind is CurveKind.EXPO:
        low = 1.0 if well_behaved else 0.5
        return Curve(CurveSpec.expo(rng.uniform(low, 3.0)))
    if kind is CurveKind.SCURVE:
        return Curve(CurveSpec.scurve(rng.uniform(0.0, 1.0)))
    inner = sorted(rng.uniform(0.0, 1.0) for _ in range(3))
    return Curve(CurveSpec.custom((0.0, inner[0], inner[1], inner[2], 1.0)))


def _random_axis(
    rng: random.Random, control: Control, *, well_behaved: bool = False
) -> AxisChainConfig:
    deadzone = rng.uniform(0.0, 0.30)
    saturation = rng.uniform(0.0, 0.30)
    if control is Control.STEER:
        rest = rng.uniform(-0.2, 0.2)
        lo = rest - rng.uniform(0.3, 1.2)
        hi = rest + rng.uniform(0.3, 1.2)
        cal = AxisCalibration.steering(
            lo, rest, hi, invert=rng.random() < 0.5, pre_gain=rng.uniform(1.0, 3.0)
        )
    else:
        rest = rng.uniform(-1.0, 1.0)
        # Both polarities: a pedal that idles at +1 and reads -1 pressed is the
        # normal case on a Logitech, not an exotic one.
        full = rest + rng.choice((-1.0, 1.0)) * rng.uniform(0.4, 2.0)
        cal = AxisCalibration.pedal(
            rest, full, invert=rng.random() < 0.5, pre_gain=rng.uniform(1.0, 2.0)
        )
    return AxisChainConfig(
        calibration=cal,
        deadzone=deadzone,
        saturation=saturation,
        curve=_random_curve(rng, well_behaved=well_behaved),
        rate_rise=rng.uniform(0.5, 40.0),
        rate_fall=rng.uniform(0.5, 40.0),
        smoothing=OneEuroConfig(
            enabled=rng.random() < 0.7,
            min_cutoff=rng.uniform(1.0, 20.0),
            beta=rng.uniform(0.0, 1.0),
        ),
    )


def _random_chain(rng: random.Random, *, well_behaved: bool = False) -> ChainConfig:
    return ChainConfig(
        steer=_random_axis(rng, Control.STEER, well_behaved=well_behaved),
        throttle=_random_axis(rng, Control.THROTTLE, well_behaved=well_behaved),
        brake=_random_axis(rng, Control.BRAKE, well_behaved=well_behaved),
        speed_sensitive_steering=rng.uniform(0.0, 0.8),
        brake_cuts_throttle=rng.random() < 0.5,
    )


def _raw_span(cfg: AxisChainConfig) -> tuple[float, float]:
    """A raw range that comfortably covers the calibrated one, plus overshoot."""
    cal = cfg.calibration
    lo = min(cal.lo, cal.rest, cal.hi)
    hi = max(cal.lo, cal.rest, cal.hi)
    pad = (hi - lo) * 0.5 + 0.1
    return lo - pad, hi + pad


def _raw_for_value(cfg: AxisChainConfig, value: float) -> float:
    """Invert the calibration: what raw input produces `value` before deadzone?"""
    cal = cfg.calibration
    if cal.invert:
        value = -value if cal.bipolar else 1.0 - value
    value /= cal.pre_gain
    if cal.bipolar:
        if value >= 0.0:
            return cal.rest + value * (cal.hi - cal.rest)
        return cal.rest + value * (cal.rest - cal.lo)
    return cal.rest + value * (cal.hi - cal.rest)


def _direction(cfg: AxisChainConfig) -> float:
    """+1 when static_map increases with raw, -1 when it decreases."""
    cal = cfg.calibration
    if cal.bipolar:
        sign = 1.0
    else:
        sign = 1.0 if cal.hi > cal.rest else -1.0
    return -sign if cal.invert else sign


# --------------------------------------------------------------------------
# Range
# --------------------------------------------------------------------------


def test_static_map_output_always_in_range() -> None:
    rng = random.Random(SEED)
    for _ in range(CASES):
        cfg_all = _random_chain(rng)
        for control in Control:
            cfg = cfg_all.axis(control)
            lo, hi = _raw_span(cfg)
            for i in range(SWEEP):
                raw = lo + (hi - lo) * i / (SWEEP - 1)
                out = static_map(cfg, raw)
                assert math.isfinite(out)
                if cfg.calibration.bipolar:
                    assert -1.0 <= out <= 1.0, (control, raw, out)
                else:
                    assert 0.0 <= out <= 1.0, (control, raw, out)


def test_chain_output_always_in_range() -> None:
    rng = random.Random(SEED + 1)
    for _ in range(60):
        chain = InputChain(_random_chain(rng))
        for _ in range(300):
            out = chain.update(
                rng.uniform(-3.0, 3.0),
                rng.uniform(-3.0, 3.0),
                rng.uniform(-3.0, 3.0),
                rng.uniform(0.0, 0.05),
                speed_frac=rng.uniform(-0.5, 1.5),
                connected=rng.random() < 0.95,
            )
            assert -1.0 <= out.steer <= 1.0
            assert 0.0 <= out.throttle <= 1.0
            assert 0.0 <= out.brake <= 1.0
            assert -STEERING_SCALE <= out.steer_q <= STEERING_SCALE
            assert 0 <= out.throttle_q <= THROTTLE_SCALE
            assert 0 <= out.brake_q <= BRAKE_SCALE


def test_hostile_input_never_escapes_range() -> None:
    """NaN, infinity and absurd magnitudes must clamp, not propagate or raise."""
    chain = InputChain(default_chain_config())
    hostile = (
        float("nan"),
        float("inf"),
        float("-inf"),
        1e308,
        -1e308,
        0.0,
        1.0,
    )
    for steer in hostile:
        for pedal in hostile:
            out = chain.update(steer, pedal, pedal, 0.004)
            assert -1.0 <= out.steer <= 1.0
            assert 0.0 <= out.throttle <= 1.0
            assert 0.0 <= out.brake <= 1.0
            assert math.isfinite(out.steer)
            assert math.isfinite(out.throttle)


def test_nonfinite_input_sets_sanitized_flag() -> None:
    chain = InputChain(default_chain_config())
    assert not chain.update(0.0, 0.0, 0.0, 0.004).sanitized
    assert chain.update(float("nan"), 0.0, 0.0, 0.004).sanitized
    assert chain.update(0.0, float("inf"), 0.0, 0.004).sanitized
    assert chain.update(0.0, 0.0, 0.0, float("nan")).sanitized


# --------------------------------------------------------------------------
# Monotonicity
# --------------------------------------------------------------------------


def test_static_map_is_monotonic_in_input() -> None:
    rng = random.Random(SEED + 2)
    for _ in range(CASES):
        cfg_all = _random_chain(rng)
        for control in Control:
            cfg = cfg_all.axis(control)
            direction = _direction(cfg)
            lo, hi = _raw_span(cfg)
            previous = static_map(cfg, lo) * direction
            for i in range(1, SWEEP):
                raw = lo + (hi - lo) * i / (SWEEP - 1)
                current = static_map(cfg, raw) * direction
                assert current >= previous - 1e-12, (control, raw, previous, current)
                previous = current


def test_curves_are_monotonic() -> None:
    rng = random.Random(SEED + 3)
    for _ in range(200):
        curve = _random_curve(rng)
        previous = curve.apply_unipolar(0.0)
        for i in range(1, 1024):
            value = curve.apply_unipolar(i / 1023.0)
            assert value >= previous - 1e-12, (curve.spec, i, previous, value)
            previous = value


def test_curve_is_sign_symmetric() -> None:
    rng = random.Random(SEED + 4)
    for _ in range(50):
        curve = _random_curve(rng)
        for i in range(0, 101):
            x = i / 100.0
            assert curve.apply(-x) == -curve.apply(x)


# --------------------------------------------------------------------------
# Deadzone
# --------------------------------------------------------------------------


def test_center_deadzone_is_exact_and_continuous() -> None:
    for dz in (0.0, 0.01, 0.05, 0.13, 0.25, 0.5):
        assert center_deadzone(0.0, dz) == 0.0
        assert center_deadzone(dz, dz) == 0.0
        assert center_deadzone(-dz, dz) == 0.0
        assert center_deadzone(1.0, dz) == 1.0
        assert center_deadzone(-1.0, dz) == -1.0
        eps = 1e-9
        just_out = center_deadzone(dz + eps, dz)
        assert 0.0 <= just_out < 1e-6, (dz, just_out)
        assert center_deadzone(-dz - eps, dz) == -just_out


def test_floor_deadzone_is_exact_and_continuous() -> None:
    for dz in (0.0, 0.02, 0.1, 0.3):
        assert floor_deadzone(0.0, dz) == 0.0
        assert floor_deadzone(dz, dz) == 0.0
        assert floor_deadzone(1.0, dz) == 1.0
        just_out = floor_deadzone(dz + 1e-9, dz)
        assert 0.0 <= just_out < 1e-6


def test_deadzone_boundary_in_full_static_map() -> None:
    """Zero inside the deadzone, and no step across its edge.

    Probed strictly inside and strictly outside rather than exactly on the
    boundary: `_raw_for_value` is a floating-point inverse, and a raw value one
    ulp past the edge is a fact about IEEE 754, not about the chain. The exact
    boundary behaviour is pinned directly on the primitives above.
    """
    rng = random.Random(SEED + 5)
    for _ in range(CASES):
        cfg_all = _random_chain(rng)
        for control in Control:
            cfg = cfg_all.axis(control)
            if cfg.deadzone <= 0.0:
                continue
            inside = _raw_for_value(cfg, cfg.deadzone * (1.0 - 1e-9))
            assert static_map(cfg, inside) == 0.0, (control, cfg.deadzone)
            outside = _raw_for_value(cfg, cfg.deadzone + 1e-9)
            just_out = abs(static_map(cfg, outside))
            assert just_out < 1e-3, (control, just_out)
            # Halfway into the deadzone is unambiguously zero.
            assert static_map(cfg, _raw_for_value(cfg, cfg.deadzone * 0.5)) == 0.0
            assert static_map(cfg, _raw_for_value(cfg, -cfg.deadzone * 0.5)) == 0.0


def test_deadzone_rescales_so_full_scale_survives() -> None:
    """A deadzone must not cost the driver the top of the range."""
    rng = random.Random(SEED + 6)
    for _ in range(120):
        cfg_all = _random_chain(rng)
        for control in Control:
            cfg = cfg_all.axis(control)
            full = static_map(cfg, _raw_for_value(cfg, 1.0))
            assert abs(abs(full) - 1.0) < 1e-12, (control, full)


# --------------------------------------------------------------------------
# Saturation
# --------------------------------------------------------------------------


def test_saturate_reaches_exactly_one() -> None:
    for sat in (0.0, 0.01, 0.05, 0.2, 0.4, 0.6):
        assert saturate(1.0 - sat, sat) == 1.0
        assert saturate(-(1.0 - sat), sat) == -1.0
        assert saturate(1.0, sat) == 1.0
        assert saturate(0.0, sat) == 0.0


def test_static_map_reaches_exactly_one_at_saturation_point() -> None:
    rng = random.Random(SEED + 7)
    for _ in range(CASES):
        cfg_all = _random_chain(rng)
        for control in Control:
            cfg = cfg_all.axis(control)
            # Value that, after the deadzone rescale, lands exactly on the
            # saturation threshold.
            value = cfg.deadzone + (1.0 - cfg.saturation) * (1.0 - cfg.deadzone)
            raw = _raw_for_value(cfg, min(value, 1.0))
            out = static_map(cfg, raw)
            assert abs(abs(out) - 1.0) < 1e-9, (control, cfg.saturation, out)
            # And a hair past it is exactly full scale, with no overshoot.
            nudge = (cfg.calibration.hi - cfg.calibration.rest) * 1e-6
            beyond = static_map(cfg, raw + nudge)
            assert abs(beyond) <= 1.0
            beyond = static_map(cfg, raw - nudge)
            assert abs(beyond) <= 1.0


def test_full_travel_quantizes_to_full_scale() -> None:
    """End to end: at the mechanical stop, the wire carries +/-1000."""
    rng = random.Random(SEED + 8)
    for _ in range(60):
        cfg = _random_chain(rng)
        chain = InputChain(cfg)
        steer_raw = _raw_for_value(cfg.steer, 1.0)
        throttle_raw = _raw_for_value(cfg.throttle, 1.0)
        brake_raw = _raw_for_value(cfg.brake, 1.0)
        out = chain.update(steer_raw, throttle_raw, brake_raw, 0.0)
        for _ in range(2000):
            out = chain.update(steer_raw, throttle_raw, brake_raw, 0.004)
        assert abs(out.steer_q) == STEERING_SCALE, out.steer_q
        if not cfg.brake_cuts_throttle:
            assert out.throttle_q == THROTTLE_SCALE, out.throttle_q
        assert out.brake_q == BRAKE_SCALE, out.brake_q


# --------------------------------------------------------------------------
# Curves and the LUT
# --------------------------------------------------------------------------


def test_lut_endpoints_are_exact() -> None:
    rng = random.Random(SEED + 9)
    for _ in range(100):
        curve = _random_curve(rng)
        lut = curve.lut
        assert len(lut) == LUT_SIZE
        assert lut[0] == 0.0
        assert lut[-1] == 1.0
        assert curve.apply_unipolar(0.0) == 0.0
        assert curve.apply_unipolar(1.0) == 1.0


def test_lut_matches_analytic_curve() -> None:
    """The table is a piecewise-linear stand-in for the analytic curve.

    Tolerances are separated because the mathematics is: expo with gamma < 1 has
    infinite slope at the origin, so no evenly spaced table can follow it there.
    Every other family sits around 1e-5.
    """
    specs_tight = [
        CurveSpec.linear(),
        CurveSpec.expo(1.0),
        CurveSpec.expo(1.4),
        CurveSpec.expo(1.8),
        CurveSpec.expo(2.5),
        CurveSpec.expo(3.0),
        CurveSpec.scurve(0.0),
        CurveSpec.scurve(0.5),
        CurveSpec.scurve(1.0),
    ]
    for spec in specs_tight:
        curve = Curve(spec)
        worst = 0.0
        for i in range(4001):
            x = i / 4000.0
            worst = max(worst, abs(curve.apply_unipolar(x) - evaluate(spec, x)))
        assert worst < 1e-4, (spec, worst)
        assert lut_error_bound(spec) < 1e-4, (spec, lut_error_bound(spec))

    for gamma in (0.5, 0.7, 0.9):
        spec = CurveSpec.expo(gamma)
        curve = Curve(spec)
        worst = 0.0
        for i in range(4001):
            x = i / 4000.0
            worst = max(worst, abs(curve.apply_unipolar(x) - evaluate(spec, x)))
        assert worst < 2e-2, (gamma, worst)
        # And the error is concentrated where the theory says it is: away from
        # the origin the table is as good as anywhere else.
        tail = max(
            abs(curve.apply_unipolar(i / 4000.0) - evaluate(spec, i / 4000.0))
            for i in range(400, 4001)
        )
        assert tail < 1e-4, (gamma, tail)


def test_custom_curve_lut_error_stays_under_a_wire_step() -> None:
    """A custom curve's corners do not land on table nodes, and cannot.

    The breakpoints sit at multiples of 0.25, while the 256-entry table samples
    at multiples of 1/255, so the interpolated table rounds every corner off. The
    size of that rounding is bounded by the change in slope at the corner: about
    4e-4 for a curve anyone would actually draw, and about 4e-3 for the most
    extreme legal shape (flat, then vertical, then flat). Both are below the
    1/1000 step the control packet quantizes to, so the table never costs more
    than a count or two on the wire -- which is the number that matters, and the
    reason 256 entries is enough.
    """
    ordinary = CurveSpec.custom((0.0, 0.1, 0.35, 0.7, 1.0))
    assert lut_error_bound(ordinary) < 1.0 / 1000.0

    extreme = CurveSpec.custom((0.0, 0.0, 0.0, 1.0, 1.0))
    assert lut_error_bound(extreme) < 5.0 / 1000.0

    for spec in (ordinary, extreme, CurveSpec.custom((0.0, 0.0, 0.5, 1.0, 1.0))):
        curve = Curve(spec)
        # Whatever the interpolation does between nodes, it is exact on them.
        for i in range(LUT_SIZE):
            x = i / (LUT_SIZE - 1)
            assert abs(curve.apply_unipolar(x) - evaluate(spec, x)) < 1e-12
        assert curve.apply_unipolar(0.0) == 0.0
        assert curve.apply_unipolar(1.0) == 1.0


def test_lut_is_a_sampling_of_the_spec() -> None:
    rng = random.Random(SEED + 10)
    for _ in range(50):
        curve = _random_curve(rng)
        lut = build_lut(curve.spec)
        for i in range(LUT_SIZE):
            assert lut[i] == evaluate(curve.spec, i / (LUT_SIZE - 1))
            assert curve.lut[i] == lut[i]


def test_curve_rejects_nonsense() -> None:
    for bad in (
        lambda: CurveSpec(CurveKind.EXPO, gamma=float("nan")),
        lambda: CurveSpec(CurveKind.EXPO, gamma=0.0),
        lambda: CurveSpec(CurveKind.EXPO, gamma=1e9),
        lambda: CurveSpec(CurveKind.SCURVE, strength=1.5),
        lambda: CurveSpec(CurveKind.CUSTOM, points=(0.0, 0.5, 0.4, 0.8, 1.0)),
        lambda: CurveSpec(CurveKind.CUSTOM, points=(0.1, 0.5, 0.6, 0.8, 1.0)),
        lambda: CurveSpec(CurveKind.CUSTOM, points=(0.0, 0.5, 1.0)),
    ):
        try:
            bad()
        except ValueError:
            continue
        raise AssertionError("invalid curve spec was accepted")


def test_custom_curve_sanitizer_always_produces_a_valid_spec() -> None:
    rng = random.Random(SEED + 11)
    for _ in range(200):
        raw = [rng.uniform(-2.0, 2.0) for _ in range(5)]
        spec = CurveSpec.custom(raw)
        assert spec.points[0] == 0.0
        assert spec.points[-1] == 1.0
        for a, b in zip(spec.points, spec.points[1:]):
            assert b >= a


# --------------------------------------------------------------------------
# Calibration primitives
# --------------------------------------------------------------------------


def test_calibration_endpoints() -> None:
    cal = AxisCalibration.steering(-0.8, 0.05, 0.9)
    assert calibrate_bipolar(0.05, cal) == 0.0
    assert calibrate_bipolar(0.9, cal) == 1.0
    assert calibrate_bipolar(-0.8, cal) == -1.0
    assert calibrate_bipolar(5.0, cal) == 1.0
    assert calibrate_bipolar(-5.0, cal) == -1.0

    pedal = AxisCalibration.pedal(1.0, -1.0)
    assert calibrate_unipolar(1.0, pedal) == 0.0
    assert calibrate_unipolar(-1.0, pedal) == 1.0
    assert calibrate_unipolar(0.0, pedal) == 0.5
    assert calibrate_unipolar(9.0, pedal) == 0.0


def test_asymmetric_steering_scales_each_side_independently() -> None:
    """A wheel whose centre is off-centre must still reach both locks."""
    cal = AxisCalibration.steering(-0.5, 0.1, 1.0)
    assert calibrate_bipolar(-0.5, cal) == -1.0
    assert calibrate_bipolar(1.0, cal) == 1.0
    assert calibrate_bipolar(0.1, cal) == 0.0
    half_right = calibrate_bipolar(0.1 + 0.45, cal)
    half_left = calibrate_bipolar(0.1 - 0.3, cal)
    assert abs(half_right - 0.5) < 1e-12
    assert abs(half_left + 0.5) < 1e-12


def test_calibration_rejects_degenerate_ranges() -> None:
    for bad in (
        lambda: AxisCalibration.steering(0.0, 0.0, 1.0),
        lambda: AxisCalibration.steering(1.0, 0.0, -1.0),
        lambda: AxisCalibration.pedal(0.0, 0.0),
        lambda: AxisCalibration.pedal(0.0, 0.01),
        lambda: AxisCalibration(False, rest=float("nan"), hi=1.0),
        lambda: AxisCalibration.pedal(0.0, 1.0, pre_gain=0.0),
    ):
        try:
            bad()
        except ChainConfigError:
            continue
        raise AssertionError("invalid calibration was accepted")


def test_chain_config_rejects_impossible_combinations() -> None:
    cal = AxisCalibration.pedal(0.0, 1.0)
    for bad in (
        lambda: AxisChainConfig(cal, deadzone=0.6, saturation=0.5),
        lambda: AxisChainConfig(cal, deadzone=-0.1),
        lambda: AxisChainConfig(cal, rate_rise=0.0),
        lambda: AxisChainConfig(cal, rate_fall=float("inf")),
        lambda: AxisChainConfig(cal, saturation=0.9),
    ):
        try:
            bad()
        except ChainConfigError:
            continue
        raise AssertionError("invalid axis config was accepted")

    steer_cfg = AxisChainConfig(AxisCalibration.steering(-1.0, 0.0, 1.0))
    pedal_cfg = AxisChainConfig(cal)
    try:
        ChainConfig(steer=pedal_cfg, throttle=pedal_cfg, brake=pedal_cfg)
    except ChainConfigError:
        pass
    else:
        raise AssertionError("unipolar steering was accepted")
    try:
        ChainConfig(steer=steer_cfg, throttle=steer_cfg, brake=pedal_cfg)
    except ChainConfigError:
        pass
    else:
        raise AssertionError("bipolar throttle was accepted")


# --------------------------------------------------------------------------
# Rate limit
# --------------------------------------------------------------------------


def test_rate_limit_never_exceeds_its_bound_and_never_overshoots() -> None:
    rng = random.Random(SEED + 12)
    for _ in range(2000):
        current = rng.uniform(-1.0, 1.0)
        target = rng.uniform(-1.0, 1.0)
        rise = rng.uniform(0.1, 50.0)
        fall = rng.uniform(0.1, 50.0)
        dt = rng.uniform(0.0, 0.05)
        result = rate_limit(target, current, rise, fall, dt)
        step = max(rise, fall) * dt
        assert abs(result - current) <= step + 1e-12
        # Never past the target: a limiter that overshoots is an oscillator.
        assert min(current, target) - 1e-12 <= result <= max(current, target) + 1e-12


def test_rate_limit_converges_in_the_expected_time() -> None:
    dt = 0.004
    current = 0.0
    rate = 4.0  # units per second: 0 to 1 in 250 ms
    ticks = 0
    while current < 1.0 and ticks < 10_000:
        current = rate_limit(1.0, current, rate, rate, dt)
        ticks += 1
    assert abs(ticks * dt - 0.25) < 0.01, ticks * dt


def test_rate_limit_zero_dt_is_a_hold() -> None:
    assert rate_limit(1.0, 0.3, 10.0, 10.0, 0.0) == 0.3
    assert rate_limit(1.0, 0.3, 10.0, 10.0, -1.0) == 0.3


def test_large_dt_is_capped() -> None:
    """A descheduled input thread must not let the command jump the full range."""
    cfg = default_chain_config()
    chain = InputChain(cfg)
    out = chain.update(0.0, 1.0, 0.0, 5.0)
    assert out.throttle <= cfg.throttle.rate_rise * MAX_DT + 1e-9


# --------------------------------------------------------------------------
# One-euro filter
# --------------------------------------------------------------------------


def test_one_euro_stays_inside_the_input_hull() -> None:
    rng = random.Random(SEED + 13)
    for _ in range(50):
        filt = OneEuroFilter(
            OneEuroConfig(min_cutoff=rng.uniform(0.5, 20.0), beta=rng.uniform(0.0, 1.0))
        )
        lo = hi = None
        for _ in range(500):
            x = rng.uniform(-1.0, 1.0)
            lo = x if lo is None else min(lo, x)
            hi = x if hi is None else max(hi, x)
            y = filt.filter(x, 0.004)
            assert lo - 1e-12 <= y <= hi + 1e-12


def test_one_euro_converges_to_a_constant() -> None:
    filt = OneEuroFilter(OneEuroConfig(min_cutoff=1.0, beta=0.0))
    filt.filter(0.0, 0.004)
    y = 0.0
    for _ in range(5000):
        y = filt.filter(1.0, 0.004)
    assert abs(y - 1.0) < 1e-6


def test_one_euro_first_sample_is_not_a_ramp() -> None:
    """Starting from a held position must not flick the wheel to centre."""
    filt = OneEuroFilter(OneEuroConfig(min_cutoff=1.0, beta=0.0))
    assert filt.filter(-0.83, 0.004) == -0.83


def test_one_euro_disabled_is_a_pass_through() -> None:
    filt = OneEuroFilter(OneEuroConfig(enabled=False))
    for value in (0.0, 0.5, -1.0, 0.25):
        assert filt.filter(value, 0.004) == value


def test_one_euro_beta_reduces_lag_on_fast_movement() -> None:
    slow = OneEuroFilter(OneEuroConfig(min_cutoff=1.0, beta=0.0))
    fast = OneEuroFilter(OneEuroConfig(min_cutoff=1.0, beta=2.0))
    slow.filter(0.0, 0.004)
    fast.filter(0.0, 0.004)
    slow_out = fast_out = 0.0
    for i in range(1, 26):
        target = i / 25.0
        slow_out = slow.filter(target, 0.004)
        fast_out = fast.filter(target, 0.004)
    assert fast_out > slow_out


# --------------------------------------------------------------------------
# Whole-chain behaviour
# --------------------------------------------------------------------------


def test_neutral_input_gives_exactly_zero() -> None:
    rng = random.Random(SEED + 14)
    for _ in range(100):
        cfg = _random_chain(rng)
        chain = InputChain(cfg)
        rest = (
            cfg.steer.calibration.rest,
            cfg.throttle.calibration.rest,
            cfg.brake.calibration.rest,
        )
        out = chain.update(rest[0], rest[1], rest[2], 0.004)
        for _ in range(200):
            out = chain.update(rest[0], rest[1], rest[2], 0.004)
        # An inverted pedal reads full at rest by definition; only the
        # non-inverted ones are expected to sit at zero.
        assert out.steer_q == 0 or cfg.steer.calibration.invert
        if not cfg.throttle.calibration.invert:
            assert out.throttle_q == 0
        if not cfg.brake.calibration.invert:
            assert out.brake_q == 0


def test_quantization_matches_the_protocol() -> None:
    """The chain and `ControlPacket.from_normalized` must round identically."""
    rng = random.Random(SEED + 15)
    for _ in range(5000):
        steer = rng.uniform(-1.2, 1.2)
        throttle = rng.uniform(-0.2, 1.2)
        brake = rng.uniform(-0.2, 1.2)
        packet = ControlPacket.from_normalized(1, 1, 0, steer, throttle, brake)
        assert quantize(steer, STEERING_SCALE, signed=True) == packet.steering
        assert quantize(throttle, THROTTLE_SCALE, signed=False) == packet.throttle
        assert quantize(brake, BRAKE_SCALE, signed=False) == packet.brake


def test_reported_floats_match_what_goes_on_the_wire() -> None:
    rng = random.Random(SEED + 16)
    chain = InputChain(default_chain_config())
    for _ in range(500):
        out = chain.update(
            rng.uniform(-1.0, 1.0), rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0), 0.004
        )
        assert out.steer == out.steer_q / STEERING_SCALE
        assert out.throttle == out.throttle_q / THROTTLE_SCALE
        assert out.brake == out.brake_q / BRAKE_SCALE


def test_disconnect_cuts_throttle_immediately() -> None:
    chain = InputChain(default_chain_config())
    for _ in range(1000):
        chain.update(0.5, 1.0, 0.0, 0.004)
    assert chain.update(0.5, 1.0, 0.0, 0.004).throttle > 0.9
    out = chain.update(0.5, 1.0, 0.0, 0.004, connected=False)
    assert out.throttle == 0.0
    assert out.input_lost
    # Steering ramps rather than snapping: a snap to centre mid-corner is not
    # obviously safer than a controlled return.
    assert out.steer != 0.0


def test_brake_cuts_throttle_when_configured() -> None:
    cfg = default_chain_config(digital_pedals=True)
    assert cfg.brake_cuts_throttle
    chain = InputChain(cfg)
    for _ in range(2000):
        chain.update(0.0, 1.0, 0.0, 0.004)
    assert chain.update(0.0, 1.0, 0.0, 0.004).throttle > 0.9
    for _ in range(2000):
        out = chain.update(0.0, 1.0, 1.0, 0.004)
    assert out.throttle == 0.0
    assert out.brake > 0.9


def test_speed_sensitive_steering_scales_authority() -> None:
    assert speed_scale(0.0, 0.5) == 1.0
    assert speed_scale(1.0, 0.5) == 0.5
    assert speed_scale(2.0, 0.5) == 0.5
    assert speed_scale(-1.0, 0.5) == 1.0
    assert speed_scale(0.5, 0.0) == 1.0

    base = default_chain_config()
    cfg = ChainConfig(
        steer=base.steer,
        throttle=base.throttle,
        brake=base.brake,
        speed_sensitive_steering=0.5,
    )
    slow = InputChain(cfg)
    fast = InputChain(cfg)
    for _ in range(3000):
        slow_out = slow.update(1.0, 0.0, 0.0, 0.004, speed_frac=0.0)
        fast_out = fast.update(1.0, 0.0, 0.0, 0.004, speed_frac=1.0)
    assert abs(slow_out.steer - 1.0) < 1e-6
    assert abs(fast_out.steer - 0.5) < 1e-3
    assert fast_out.steer_assist == 0.5


def test_reset_returns_to_neutral() -> None:
    chain = InputChain(default_chain_config())
    for _ in range(2000):
        chain.update(1.0, 1.0, 0.0, 0.004)
    chain.reset()
    assert chain.state == (0.0, 0.0, 0.0)
    out = chain.update(0.0, 0.0, 0.0, 0.004)
    assert out.steer_q == 0 and out.throttle_q == 0


def test_chain_is_deterministic() -> None:
    rng_a = random.Random(SEED + 17)
    rng_b = random.Random(SEED + 17)
    chain_a = InputChain(default_chain_config())
    chain_b = InputChain(default_chain_config())
    for _ in range(2000):
        a = chain_a.update(
            rng_a.uniform(-1, 1), rng_a.uniform(0, 1), rng_a.uniform(0, 1), 0.004
        )
        b = chain_b.update(
            rng_b.uniform(-1, 1), rng_b.uniform(0, 1), rng_b.uniform(0, 1), 0.004
        )
        assert a == b


def test_pending_config_is_picked_up_on_the_next_tick() -> None:
    chain = InputChain(default_chain_config())
    other = default_chain_config(digital_pedals=True)
    chain.pending_config = other
    chain.update(0.0, 0.0, 0.0, 0.004)
    assert chain.config is other
    assert chain.pending_config is None


# --------------------------------------------------------------------------
# Digital pedals -- the case this app was built around
# --------------------------------------------------------------------------


def test_digital_pedal_ramps_instead_of_switching() -> None:
    """A button pedal must not be an on/off switch at the car."""
    profile = preset_digital_wheel(profile_id="test", name="test wheel")
    chain = InputChain(profile.chain)
    dt = 0.004
    ticks_to_full = 0
    out = chain.update(0.0, 1.0, 0.0, dt)
    while out.throttle < 1.0 and ticks_to_full < 10_000:
        out = chain.update(0.0, 1.0, 0.0, dt)
        ticks_to_full += 1
    rise_s = ticks_to_full * dt
    assert 0.3 < rise_s < 1.5, rise_s

    ticks_to_zero = 0
    while out.throttle > 0.0 and ticks_to_zero < 10_000:
        out = chain.update(0.0, 0.0, 0.0, dt)
        ticks_to_zero += 1
    fall_s = ticks_to_zero * dt
    assert fall_s < rise_s, (rise_s, fall_s)
    assert fall_s < 0.6, fall_s


def test_digital_pedal_reaches_exactly_full_scale() -> None:
    profile = preset_digital_wheel(profile_id="test", name="test wheel")
    chain = InputChain(profile.chain)
    for _ in range(5000):
        out = chain.update(1.0, 1.0, 0.0, 0.004)
    assert out.throttle_q == THROTTLE_SCALE
    assert out.steer_q == STEERING_SCALE


# --------------------------------------------------------------------------
# Scripted lap: the whole chain, no hardware
# --------------------------------------------------------------------------


class _FakeClock:
    __slots__ = ("t",)

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def test_scripted_lap_through_the_chain() -> None:
    clock = _FakeClock()
    source = ScriptSource.full_lap(clock=clock)
    chain = InputChain(identity_profile().chain)
    dt = 1.0 / 250.0

    max_throttle = 0.0
    max_brake = 0.0
    min_steer = 0.0
    max_steer = 0.0
    samples = 0
    while clock.t <= source.duration:
        sample = source.poll()
        out = chain.update_from(sample, dt)
        assert -1.0 <= out.steer <= 1.0
        assert 0.0 <= out.throttle <= 1.0
        assert 0.0 <= out.brake <= 1.0
        assert not out.sanitized
        max_throttle = max(max_throttle, out.throttle)
        max_brake = max(max_brake, out.brake)
        min_steer = min(min_steer, out.steer)
        max_steer = max(max_steer, out.steer)
        samples += 1
        clock.t += dt

    assert samples > 5000, samples
    assert max_throttle == 1.0
    assert max_brake == 1.0
    assert min_steer <= -0.94, min_steer
    assert max_steer >= 0.99, max_steer
    # The lap ends stopped, which is what makes it usable as a soak fixture.
    final = chain.update_from(source.poll(), dt)
    assert source.finished
    assert final.throttle == 0.0
    assert final.brake == 0.0
    assert final.steer == 0.0


def test_scripted_lap_is_deterministic() -> None:
    def run() -> list[tuple[int, int, int]]:
        clock = _FakeClock()
        source = ScriptSource.full_lap(clock=clock)
        chain = InputChain(identity_profile().chain)
        out: list[tuple[int, int, int]] = []
        for _ in range(4000):
            sample = source.poll()
            result = chain.update_from(sample, 0.004)
            out.append((result.steer_q, result.throttle_q, result.brake_q))
            clock.t += 0.004
        return out

    assert run() == run()


# --------------------------------------------------------------------------
# Runner for environments with no pytest
# --------------------------------------------------------------------------


def _main() -> int:
    tests = [
        (name, obj)
        for name, obj in sorted(globals().items())
        if name.startswith("test_") and callable(obj)
    ]
    failures = 0
    for name, test in tests:
        try:
            test()
        except AssertionError as exc:
            failures += 1
            print(f"FAIL {name}: {exc}")
        except Exception as exc:  # noqa: BLE001 - a runner wants everything
            failures += 1
            print(f"ERROR {name}: {type(exc).__name__}: {exc}")
        else:
            print(f"ok   {name}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_main())
