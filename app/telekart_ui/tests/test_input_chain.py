"""The shaping chain: properties that must hold for any input.

Property tests rather than examples, because the chain is a pure function of
(sample, dt, speed) and the interesting failures are at the boundaries -- full
lock, exact centre, a dt that spans a stall.
"""

from __future__ import annotations

import pytest

from telekart_ui.input.chain import (
    MAX_DT,
    AxisCalibration,
    InputChain,
    center_deadzone,
    default_chain_config,
    floor_deadzone,
    quantize,
    rate_limit,
    saturate,
)
from telekart_ui.input.curves import (
    GAMMA_MAX,
    GAMMA_MIN,
    Curve,
    CurveError,
    CurveKind,
    CurveSpec,
    build_lut,
    evaluate,
    lut_error_bound,
)
from telekart_ui.input.defaults import keyboard_profile, wheel_profile
from telekart_ui.input.sources import RawSample

_STEPS = [i / 20.0 for i in range(-20, 21)]


# --------------------------------------------------------------------------
# Curves
# --------------------------------------------------------------------------


@pytest.mark.parametrize("spec", [CurveSpec.linear(), CurveSpec.expo(1.3), CurveSpec.expo(1.8)])
def test_curves_pass_through_zero_and_one_exactly(spec):
    """A hard requirement, not a nicety.

    The driver must always be able to command zero and to command full scale. A
    curve that quietly capped throttle at 0.98 would be invisible on the HUD and
    infuriating on track.
    """
    curve = Curve(spec)
    assert curve.apply_unipolar(0.0) == 0.0
    assert curve.apply_unipolar(1.0) == 1.0
    assert curve.apply(-1.0) == -1.0


@pytest.mark.parametrize("spec", [CurveSpec.linear(), CurveSpec.expo(1.3), CurveSpec.expo(2.5)])
def test_curves_are_monotonic(spec):
    """A control that moves backwards when you push harder is not a control."""
    curve = Curve(spec)
    previous = -1.0
    for x in _STEPS:
        value = curve.apply(x)
        assert value >= previous - 1e-9
        previous = value


def test_curves_preserve_sign():
    """Steering must stay symmetric about centre, or the car pulls to one side."""
    curve = Curve(CurveSpec.expo(1.6))
    for x in _STEPS:
        assert curve.apply(-x) == pytest.approx(-curve.apply(x))


def test_lut_error_is_below_wire_resolution():
    """The table only has to be as good as the 1/1000 the packet quantizes to."""
    for spec in (CurveSpec.linear(), CurveSpec.expo(1.3), CurveSpec.expo(1.8)):
        assert lut_error_bound(spec) < 1e-3


def test_expo_gamma_is_bounded():
    with pytest.raises(CurveError):
        CurveSpec.expo(GAMMA_MAX * 2)
    with pytest.raises(CurveError):
        CurveSpec.expo(GAMMA_MIN / 2)


def test_lut_matches_the_analytic_curve_at_the_nodes():
    spec = CurveSpec.expo(1.8)
    lut = build_lut(spec)
    assert lut[0] == pytest.approx(evaluate(spec, 0.0))
    assert lut[-1] == pytest.approx(evaluate(spec, 1.0))


def test_only_two_curve_kinds_survive():
    assert {k.value for k in CurveKind} == {"linear", "expo"}


# --------------------------------------------------------------------------
# Stages
# --------------------------------------------------------------------------


def test_deadzone_keeps_full_scale_reachable():
    """Rescaled, not merely clipped: full lock must still reach 1.0."""
    for dz in (0.0, 0.05, 0.2, 0.5):
        assert center_deadzone(1.0, dz) == pytest.approx(1.0)
        assert center_deadzone(-1.0, dz) == pytest.approx(-1.0)
        assert center_deadzone(dz * 0.5, dz) == 0.0
        assert floor_deadzone(1.0, dz) == pytest.approx(1.0)


def test_saturation_reaches_full_scale_early():
    assert saturate(1.0, 0.0) == pytest.approx(1.0)
    assert saturate(0.9, 0.1) == pytest.approx(1.0)
    assert saturate(-0.9, 0.1) == pytest.approx(-1.0)


def test_rate_limit_bounds_the_step():
    assert rate_limit(1.0, 0.0, rise=2.0, fall=4.0, dt=0.1) == pytest.approx(0.2)
    # Toward rest uses the fall rate, which is allowed to be faster: releasing
    # a control must never be slower than applying it.
    assert rate_limit(0.0, 1.0, rise=2.0, fall=4.0, dt=0.1) == pytest.approx(0.6)


def test_quantize_matches_the_wire():
    assert quantize(1.0, 1000, signed=True) == 1000
    assert quantize(-1.0, 1000, signed=True) == -1000
    assert quantize(0.0, 1000, signed=True) == 0
    assert quantize(2.0, 1000, signed=False) == 1000  # clamps, never raises


# --------------------------------------------------------------------------
# The whole chain
# --------------------------------------------------------------------------


def _run(chain: InputChain, sample: RawSample, ticks: int = 400, dt: float = 0.004):
    out = None
    for _ in range(ticks):
        out = chain.update_from(sample, dt)
    return out


def test_output_is_always_in_range():
    chain = InputChain(default_chain_config())
    for steer in (-2.0, -1.0, 0.0, 1.0, 2.0):
        for pedal in (-1.0, 0.0, 0.5, 1.0, 2.0):
            out = _run(chain, RawSample(steer=steer, throttle=pedal, brake=pedal, connected=True), 50)
            assert -1.0 <= out.steer <= 1.0
            assert 0.0 <= out.throttle <= 1.0
            assert 0.0 <= out.brake <= 1.0


def test_chain_never_raises_on_garbage():
    """A control path must not die. NaN in must not take the link down."""
    chain = InputChain(default_chain_config())
    for bad in (float("nan"), float("inf"), -float("inf")):
        out = chain.update(bad, bad, bad, 0.004)
        assert -1.0 <= out.steer <= 1.0
        assert 0.0 <= out.throttle <= 1.0


def test_dt_is_clamped_so_a_stall_cannot_jump_full_travel():
    """Resuming after a lid-close must not slam the servo to its stop."""
    chain = InputChain(default_chain_config())
    out = chain.update_from(RawSample(steer=1.0, connected=True), dt=10.0)
    limit = default_chain_config().steer.rate_rise * MAX_DT
    assert abs(out.steer) <= limit + 1e-6


def test_disconnect_cuts_throttle_immediately():
    """The device vanished. Throttle goes now; steering ramps to centre."""
    chain = InputChain(default_chain_config())
    _run(chain, RawSample(steer=0.8, throttle=1.0, connected=True))
    out = chain.update_from(RawSample(connected=False), dt=0.004)
    assert out.throttle == 0.0
    assert out.input_lost is True


def test_keyboard_reaches_full_travel_in_about_half_a_second():
    """0.45 s lock to lock: a key press is an instantaneous demand for full
    lock, and without the ramp the car slams the servo into its stop."""
    profile = keyboard_profile()
    chain = InputChain(profile.chain)
    dt = 0.004
    ticks = 0
    while ticks < 500:
        out = chain.update_from(RawSample(steer=1.0, connected=True), dt)
        ticks += 1
        if out.steer >= 0.99:
            break
    elapsed = ticks * dt
    assert 0.3 <= elapsed <= 0.8, elapsed


def test_wheel_and_keyboard_produce_the_same_output_shape():
    """Everything downstream must be unable to tell them apart."""
    for profile in (keyboard_profile(), wheel_profile()):
        chain = InputChain(profile.chain)
        out = _run(chain, RawSample(steer=1.0, throttle=1.0, connected=True))
        assert -1.0 <= out.steer <= 1.0
        assert 0.0 <= out.throttle <= 1.0
        assert isinstance(out.steer_q, int)


def test_pedal_calibration_handles_an_inverted_rest_point():
    """Logitech pedals idle at +1.0 and read -1.0 fully depressed."""
    cal = AxisCalibration.pedal(1.0, -1.0)
    from telekart_ui.input.chain import calibrate_unipolar

    assert calibrate_unipolar(1.0, cal) == pytest.approx(0.0)
    assert calibrate_unipolar(-1.0, cal) == pytest.approx(1.0)
