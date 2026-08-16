"""Quadrature decoding and the M/T velocity estimate.

The decoder is x2 -- either edge on channel A only -- with the sign taken from
the commanded H-bridge state. It is direction-blind by construction, which is a
deliberate trade: reading channel B per edge costs a socket round trip to
pigpiod, and at 2200 edges a second that is most of a core. So these tests check
that it counts correctly, that it takes its sign from the hint, and that it says
so when the hint is not available.
"""

from __future__ import annotations

import math

import pytest

from telekart.config import VehicleConfig
from telekart.constants import ENCODER_GLITCH_US, ENCODER_STALE_WINDOW_S
from telekart.drivers.encoder import EncoderSample, QuadratureEncoder
from telekart.hal.mock_backend import MockBackend
from telekart.util.clock import FakeClock

#: 1 ms between edges at 660 cpr is 90.9 RPM -- a plausible mid-throttle speed
#: for this drivetrain, and slow enough that nothing is quantisation-limited.
EDGE_SPACING_US = 1000
SPACING_S = EDGE_SPACING_US / 1_000_000.0


def rpm_for_spacing(cpr: int, spacing_us: int) -> float:
    return 60.0 * 1_000_000.0 / (cpr * spacing_us)


def make_encoder(start_s: float) -> tuple[MockBackend, FakeClock, QuadratureEncoder]:
    """An encoder on its own clock, for tests that need a particular epoch."""
    clock = FakeClock(start=start_s)
    config = VehicleConfig()
    gpio = MockBackend(clock=clock)
    encoder = QuadratureEncoder(
        gpio,
        gpio.pins.enc_l_a,
        gpio.pins.enc_l_b,
        cpr=config.encoder_cpr,
    )
    return gpio, clock, encoder


def feed(
    gpio: MockBackend,
    clock: FakeClock,
    encoder: QuadratureEncoder,
    edges: int,
    *,
    spacing_us: int = EDGE_SPACING_US,
    sample_every: int = 10,
) -> EncoderSample:
    """Deliver ``edges`` evenly spaced pulses on the left channel A, sampling at
    loop rate the way the real loop does. Returns the last sample."""
    sample = encoder.sample()
    pin = gpio.pins.enc_l_a
    level = 1
    for index in range(1, edges + 1):
        clock.advance(spacing_us / 1_000_000.0)
        gpio.inject_edge(pin, level, clock.monotonic_us() & 0xFFFFFFFF)
        level ^= 1
        if index % sample_every == 0:
            sample = encoder.sample()
    return sample


# --------------------------------------------------------------------------
# Wiring
# --------------------------------------------------------------------------


def test_encoder_claims_its_pins_with_a_glitch_filter(
    gpio: MockBackend, encoder_l: QuadratureEncoder, config: VehicleConfig
) -> None:
    """The filter runs in the daemon, not in Python. Every edge it rejects is a
    callback and a GIL acquisition that never happens."""
    pins = config.pins.encoders
    assert gpio.pin_mode(pins.left_a) == "in"
    assert gpio.pin_mode(pins.left_b) == "in"
    assert gpio.glitch_filter_us(pins.left_a) == ENCODER_GLITCH_US


def test_a_fresh_encoder_reads_zero(encoder_l: QuadratureEncoder) -> None:
    sample = encoder_l.sample()
    assert sample.counts == 0
    assert sample.total == 0
    assert sample.rpm == pytest.approx(0.0)
    assert encoder_l.total_counts == 0


# --------------------------------------------------------------------------
# Counting
# --------------------------------------------------------------------------


def test_counts_follow_the_direction_hint(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    encoder_l.set_direction_hint(1)
    feed(gpio, clock, encoder_l, 100)
    assert encoder_l.total_counts == 100

    encoder_l.set_direction_hint(-1)
    feed(gpio, clock, encoder_l, 40)
    assert encoder_l.total_counts == 60


def test_sample_returns_the_delta_since_the_previous_call(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    encoder_l.set_direction_hint(1)
    encoder_l.sample()
    sample = feed(gpio, clock, encoder_l, 30, sample_every=30)
    assert sample.counts == 30
    assert sample.total == 30
    assert sample.edges == 30
    # And the next sample sees nothing new.
    clock.advance(0.01)
    assert encoder_l.sample().counts == 0


def test_reverse_hint_makes_counts_and_rpm_negative(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    encoder_l.set_direction_hint(-1)
    sample = feed(gpio, clock, encoder_l, 200)
    assert sample.total == -200
    assert sample.rpm < 0.0
    assert sample.counts < 0


def test_invert_flips_the_reported_sign(
    gpio: MockBackend, clock: FakeClock, config: VehicleConfig
) -> None:
    """One encoder is mounted the other way round on this chassis; the fix is a
    parameter, not a rewiring job."""
    plain = QuadratureEncoder(
        gpio,
        config.pins.encoders.left_a,
        config.pins.encoders.left_b,
        cpr=config.encoder_cpr,
        invert=False,
    )
    inverted = QuadratureEncoder(
        gpio,
        config.pins.encoders.left_a,
        config.pins.encoders.left_b,
        cpr=config.encoder_cpr,
        invert=True,
    )
    plain.set_direction_hint(1)
    inverted.set_direction_hint(1)

    level = 1
    for _ in range(50):
        clock.advance(SPACING_S)
        gpio.inject_edge(config.pins.encoders.left_a, level, clock.monotonic_us() & 0xFFFFFFFF)
        level ^= 1
    plain_sample = plain.sample()
    inverted_sample = inverted.sample()

    assert plain_sample.total == 50
    assert inverted_sample.total == -50
    assert inverted_sample.rpm == pytest.approx(-plain_sample.rpm, rel=1e-6, abs=1e-9)


def test_reset_zeroes_the_totals(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    encoder_l.set_direction_hint(1)
    feed(gpio, clock, encoder_l, 50)
    encoder_l.reset()
    assert encoder_l.total_counts == 0
    assert encoder_l.sample().total == 0


# --------------------------------------------------------------------------
# Velocity
# --------------------------------------------------------------------------


def test_rpm_matches_the_edge_rate(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder, config: VehicleConfig
) -> None:
    encoder_l.set_direction_hint(1)
    sample = feed(gpio, clock, encoder_l, 400)
    expected = rpm_for_spacing(config.encoder_cpr, EDGE_SPACING_US)

    assert sample.raw_rpm == pytest.approx(expected, rel=0.05)
    assert sample.rpm == pytest.approx(expected, rel=0.10)
    assert not sample.stale


def test_low_speed_is_measured_by_timing_not_by_counting(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder, config: VehicleConfig
) -> None:
    """Counts-per-tick is unusable down here: at 100 Hz and 660 cpr, 20 RPM is
    2.2 counts per tick and quantises to +-45 %. Closing the window on an edge
    instead is the whole reason the backend exposes microsecond ticks."""
    spacing_us = 4545  # 20 RPM at 660 cpr
    encoder_l.set_direction_hint(1)
    sample = feed(gpio, clock, encoder_l, 120, spacing_us=spacing_us, sample_every=1)
    expected = rpm_for_spacing(config.encoder_cpr, spacing_us)
    assert expected == pytest.approx(20.0, rel=0.01)
    assert sample.raw_rpm == pytest.approx(expected, rel=0.15)


def test_rpm_decays_to_zero_when_the_edges_stop(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    encoder_l.set_direction_hint(1)
    feed(gpio, clock, encoder_l, 200)

    clock.advance(ENCODER_STALE_WINDOW_S * 2)
    sample = encoder_l.sample()
    assert sample.stale
    assert sample.rpm == pytest.approx(0.0, abs=1e-6)
    assert sample.counts == 0


def test_a_slow_wheel_is_not_reported_as_stale_too_early(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    encoder_l.set_direction_hint(1)
    feed(gpio, clock, encoder_l, 10)
    clock.advance(ENCODER_STALE_WINDOW_S * 0.5)
    assert not encoder_l.sample().stale


# --------------------------------------------------------------------------
# Direction uncertainty
# --------------------------------------------------------------------------


def test_counting_with_no_direction_hint_is_flagged_uncertain(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    """Coasting downhill with zero commanded duty: the wheels turn and the sign
    of that motion is a guess. Telemetry says so rather than inventing one."""
    encoder_l.set_direction_hint(0)
    feed(gpio, clock, encoder_l, 100)
    assert encoder_l.direction_uncertain


def test_a_commanded_wheel_is_not_uncertain(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    encoder_l.set_direction_hint(1)
    feed(gpio, clock, encoder_l, 100)
    assert not encoder_l.direction_uncertain


# --------------------------------------------------------------------------
# Anomalies -- the callback path may never raise
# --------------------------------------------------------------------------


def test_the_tick_counter_wrapping_does_not_corrupt_the_estimate(
    config: VehicleConfig,
) -> None:
    """pigpio's tick is 32 bits of microseconds, so it wraps every 71.6 minutes.

    An unsigned subtraction across that boundary reads as 4294 seconds, which
    the M/T estimator would happily turn into a plausible near-zero RPM -- a
    spinning wheel reported as stopped, once every 71.6 minutes, for one tick.
    """
    # Twenty edges before the wrap, forty after.
    gpio, clock, encoder = make_encoder(((1 << 32) - 20 * EDGE_SPACING_US) / 1_000_000.0)
    encoder.set_direction_hint(1)
    sample = feed(gpio, clock, encoder, 60)

    assert clock.monotonic_us() > (1 << 32), "the clock never crossed the wrap"
    expected = rpm_for_spacing(config.encoder_cpr, EDGE_SPACING_US)
    assert math.isfinite(sample.raw_rpm)
    assert sample.raw_rpm == pytest.approx(expected, rel=0.10)
    assert encoder.total_counts == 60


def test_out_of_order_edges_do_not_produce_a_plausible_lie(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder, config: VehicleConfig
) -> None:
    encoder_l.set_direction_hint(1)
    pin = config.pins.encoders.left_a
    now = clock.monotonic_us() & 0xFFFFFFFF

    gpio.inject_edge(pin, 1, now)
    gpio.inject_edge(pin, 0, (now - 500) & 0xFFFFFFFF)  # delivered late, stamped early
    clock.advance(0.01)
    sample = encoder_l.sample()

    assert math.isfinite(sample.rpm)
    assert abs(sample.rpm) < 10_000.0


def test_a_burst_of_noise_cannot_make_the_wheel_supersonic(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder
) -> None:
    """The glitch filter lives in the daemon, so the driver still has to survive
    whatever gets past it."""
    encoder_l.set_direction_hint(1)
    pin = gpio.pins.enc_l_a
    now = clock.monotonic_us() & 0xFFFFFFFF
    # 40 us apart clears the 30 us filter and implies 2273 RPM, roughly fifteen
    # times anything this drivetrain can reach.
    for index in range(20):
        gpio.inject_edge(pin, index % 2, (now + index * 40) & 0xFFFFFFFF)
    clock.advance(0.01)
    sample = encoder_l.sample()
    assert math.isfinite(sample.rpm)
    assert sample.counts > 0


# --------------------------------------------------------------------------
# Against the plant
# --------------------------------------------------------------------------


def test_the_estimate_tracks_the_plant(
    gpio: MockBackend, clock: FakeClock, encoder_l: QuadratureEncoder, config: VehicleConfig
) -> None:
    """End to end: real synthesised edges from a wheel that is genuinely
    turning, decoded the way the firmware decodes them."""
    pins = config.pins.motors
    gpio.setup_output(pins.in1, False)
    gpio.setup_output(pins.in2, False)
    gpio.write(pins.in1, True)
    gpio.set_pwm_pair(pins.ena, pins.enb, config.pwm_hz, 0.6, 0.0)
    encoder_l.set_direction_hint(1)

    sample = None
    for _ in range(300):  # three seconds at loop rate
        clock.advance(0.01)
        gpio.step(0.01)
        sample = encoder_l.sample()

    assert sample is not None
    truth = gpio.wheel_rpm("left")
    assert truth > 50.0
    assert sample.rpm == pytest.approx(truth, rel=0.12)
    assert encoder_l.total_counts > 0
    assert not sample.stale
