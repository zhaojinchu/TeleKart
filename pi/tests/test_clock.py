"""Pacing and period statistics.

The whole suite's determinism rests on this module, so it gets tested harder
than its size suggests. A ``DeadlineScheduler`` that quietly drifts turns every
timing assertion elsewhere into a coin flip.
"""

from __future__ import annotations

import pytest

from telekart.util.clock import (
    DeadlineScheduler,
    FakeClock,
    JitterSnapshot,
    JitterStats,
    RealClock,
)

PERIOD = 0.010


# --------------------------------------------------------------------------
# FakeClock
# --------------------------------------------------------------------------


def test_fake_clock_sleep_advances_instead_of_blocking() -> None:
    clock = FakeClock(start=5.0)
    clock.sleep(0.25)
    assert clock.monotonic() == pytest.approx(5.25)
    assert clock.slept == pytest.approx(0.25)
    assert clock.sleep_calls == 1


def test_fake_clock_ignores_non_positive_sleep() -> None:
    clock = FakeClock()
    clock.sleep(0.0)
    clock.sleep(-1.0)
    assert clock.monotonic() == 0.0
    assert clock.sleep_calls == 0


def test_fake_clock_microseconds_track_seconds() -> None:
    clock = FakeClock(start=1234.5)
    assert clock.monotonic_us() == 1_234_500_000
    clock.advance(0.000_001)
    assert clock.monotonic_us() == 1_234_500_001


def test_fake_clock_refuses_to_run_backwards() -> None:
    clock = FakeClock(start=10.0)
    with pytest.raises(ValueError):
        clock.advance(-0.001)
    with pytest.raises(ValueError):
        clock.set(9.999)
    assert clock.monotonic() == 10.0


def test_fake_clock_auto_advance_moves_on_every_read() -> None:
    clock = FakeClock(auto_advance=0.001)
    first = clock.monotonic()
    second = clock.monotonic()
    assert second - first == pytest.approx(0.001)


def test_real_clock_is_monotonic() -> None:
    clock = RealClock()
    first = clock.monotonic()
    clock.sleep(0.0)
    assert clock.monotonic() >= first
    assert clock.monotonic_us() > 0


# --------------------------------------------------------------------------
# JitterStats
# --------------------------------------------------------------------------


def test_jitter_stats_empty_snapshot_is_all_zero() -> None:
    snapshot = JitterStats().snapshot()
    assert snapshot == JitterSnapshot(0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0)
    assert not snapshot.exceeds(0.012)


def test_jitter_stats_percentiles_of_a_known_distribution() -> None:
    stats = JitterStats(bucket_s=1e-4, n_buckets=1024)
    # 98 ticks at 10 ms and two at 50 ms: the mean moves by 0.8 ms and says
    # nothing useful; the top 2 % is where the dropped commands live.
    for _ in range(98):
        stats.add(0.010)
    for _ in range(2):
        stats.add(0.050)

    snapshot = stats.snapshot()
    assert snapshot.count == 100
    # Quantiles report the upper edge of the containing bucket, so a 10.0 ms
    # sample in a 0.1 ms histogram reads as 10.1 ms and never as 9.9.
    assert snapshot.p50 == pytest.approx(0.0101, abs=1e-9)
    assert snapshot.p95 == pytest.approx(0.0101, abs=1e-9)
    assert snapshot.max == pytest.approx(0.050)
    assert snapshot.mean == pytest.approx(0.0108, abs=1e-6)
    assert snapshot.min == pytest.approx(0.010, abs=1e-9)
    # This is the entire reason the loop is judged on p99 rather than its mean.
    assert snapshot.p99 == pytest.approx(0.050, abs=1e-3)


def test_jitter_stats_reports_the_upper_bucket_edge() -> None:
    """Never optimistic: a latency budget rounded down is a budget that lies."""
    stats = JitterStats(bucket_s=1e-3, n_buckets=64)
    for _ in range(10):
        stats.add(0.0105)
    snapshot = stats.snapshot()
    assert snapshot.p50 >= 0.0105
    assert snapshot.p50 <= 0.011


def test_jitter_stats_keeps_an_exact_max_beyond_the_histogram() -> None:
    stats = JitterStats(bucket_s=1e-4, n_buckets=8)  # tops out at 800 us
    stats.add(0.0002)
    stats.add(3.0)
    snapshot = stats.snapshot()
    assert snapshot.max == pytest.approx(3.0)
    assert snapshot.p99 == pytest.approx(3.0)


def test_jitter_stats_clamps_negative_periods() -> None:
    stats = JitterStats()
    stats.add(-0.5)
    assert stats.snapshot().max == 0.0


def test_jitter_stats_reset_clears_everything() -> None:
    stats = JitterStats()
    for _ in range(50):
        stats.add(0.01)
    stats.reset()
    assert stats.count == 0
    assert stats.snapshot().count == 0


def test_jitter_snapshot_exceeds_only_on_p99() -> None:
    """A loop that blows its budget on 5 % of ticks is a failing loop.

    Its mean is 13.6 ms under budget and says nothing; p99 is what catches it.
    """
    stats = JitterStats()
    for _ in range(95):
        stats.add(0.009)
    for _ in range(5):
        stats.add(0.100)
    snapshot = stats.snapshot()

    assert snapshot.mean < 0.015
    assert snapshot.exceeds(0.012)
    assert snapshot.p99_us > 12_000
    assert snapshot.max_us >= 100_000
    # And one straggler in a thousand does not: p99 is not p100.
    calm = JitterStats()
    for _ in range(999):
        calm.add(0.009)
    calm.add(0.100)
    assert not calm.snapshot().exceeds(0.012)


# --------------------------------------------------------------------------
# DeadlineScheduler
# --------------------------------------------------------------------------


def test_scheduler_rejects_a_non_positive_period() -> None:
    with pytest.raises(ValueError):
        DeadlineScheduler(FakeClock(), 0.0)
    with pytest.raises(ValueError):
        DeadlineScheduler(FakeClock(), -0.01)


def test_scheduler_does_not_drift_under_a_variable_body() -> None:
    """The property the whole design exists for.

    A body whose cost wanders between 1 ms and 9 ms must still produce ticks on
    exact 10 ms boundaries. Sleeping ``period - elapsed`` instead would bank
    every rounding error and end up milliseconds late by the thousandth tick.
    """
    clock = FakeClock(start=100.0)
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()
    started = clock.monotonic()

    costs = [0.001, 0.004, 0.009, 0.002, 0.007]
    ticks = 1000
    for index in range(ticks):
        clock.advance(costs[index % len(costs)])
        scheduler.wait_next()

    elapsed = clock.monotonic() - started
    assert elapsed == pytest.approx(ticks * PERIOD, abs=1e-9)
    assert scheduler.overruns == 0
    assert scheduler.skipped == 0


def test_scheduler_first_period_is_a_full_period() -> None:
    clock = FakeClock()
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()
    assert scheduler.wait_next() == pytest.approx(PERIOD)


def test_scheduler_starts_itself_if_asked_to_wait_first() -> None:
    clock = FakeClock()
    scheduler = DeadlineScheduler(clock, PERIOD)
    assert scheduler.wait_next() == pytest.approx(PERIOD)


def test_scheduler_skips_missed_deadlines_rather_than_bursting() -> None:
    """One 35 ms hiccup must not be repaid with four back-to-back iterations.

    Catching up that way turns a single overrun into a burst that starves the
    network stack on the same core, which is how a scheduling hiccup becomes
    packet loss.
    """
    clock = FakeClock(start=50.0)
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()

    scheduler.wait_next()
    clock.advance(0.035)  # three and a half periods inside one body
    dt = scheduler.wait_next()

    assert dt == pytest.approx(0.035)
    assert scheduler.overruns == 1
    assert scheduler.skipped >= 2
    # The recovered deadline is in the future, which is what "skip" means: the
    # loop waits for the next grid point instead of running immediately.
    assert scheduler.deadline > clock.monotonic()

    # And the very next iteration sleeps out the remainder rather than firing at once.
    remaining = scheduler.deadline - clock.monotonic()
    assert scheduler.wait_next() == pytest.approx(remaining)
    assert scheduler.overruns == 1


def test_scheduler_recovers_phase_after_a_single_late_tick() -> None:
    clock = FakeClock(start=0.0)
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()
    scheduler.wait_next()

    clock.advance(0.0125)  # 2.5 ms late, but under two full periods
    scheduler.wait_next()
    scheduler.wait_next()

    # Three periods of work have elapsed and the phase is back on the grid.
    assert clock.monotonic() == pytest.approx(3 * PERIOD, abs=1e-9)


def test_scheduler_records_every_period_in_its_stats() -> None:
    clock = FakeClock()
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()
    for _ in range(200):
        clock.advance(0.003)
        scheduler.wait_next()
    snapshot = scheduler.stats.snapshot()
    assert snapshot.count == 200
    assert snapshot.p99 == pytest.approx(PERIOD, abs=5e-5)
    assert not snapshot.exceeds(0.012)


def test_scheduler_time_remaining_goes_negative_when_over_budget() -> None:
    clock = FakeClock()
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()
    assert scheduler.time_remaining() == pytest.approx(PERIOD)
    clock.advance(0.015)
    assert scheduler.time_remaining() < 0.0


def test_scheduler_restart_reanchors_without_reporting_an_overrun() -> None:
    """A deliberate pause -- arming, a calibration run -- is not a missed deadline."""
    clock = FakeClock()
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()
    scheduler.wait_next()

    clock.advance(5.0)
    scheduler.start()
    scheduler.wait_next()

    assert scheduler.overruns == 0


def test_scheduler_reset_clears_counters_and_stats() -> None:
    clock = FakeClock()
    scheduler = DeadlineScheduler(clock, PERIOD)
    scheduler.start()
    clock.advance(0.05)
    scheduler.wait_next()
    assert scheduler.overruns == 1

    scheduler.reset()
    assert scheduler.overruns == 0
    assert scheduler.skipped == 0
    assert scheduler.stats.count == 0

    # After a reset it re-anchors on the next wait rather than using a stale deadline.
    assert scheduler.wait_next() == pytest.approx(PERIOD)


def test_scheduler_period_is_readable() -> None:
    scheduler = DeadlineScheduler(FakeClock(), PERIOD)
    assert scheduler.period == PERIOD
    assert "10.0ms" in repr(scheduler)
