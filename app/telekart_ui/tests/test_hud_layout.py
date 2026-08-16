"""HUD geometry: the zones follow the picture, and never overlap it wrongly.

The rule these enforce is that the HUD is placed over the *letterboxed picture
rect*, not the widget rect. Anchoring to the widget leaves the status cluster
floating half over a black bar on a 4:3 stream in a 16:9 window, and puts every
instrument in the vignetted corner of the image on any stream.
"""

from __future__ import annotations

import pytest
from PySide6.QtCore import QRect

from telekart_protocol import CRITICAL_FAULTS, Fault, TelemetryFlags, VehicleState

from telekart_ui.model.snapshots import InputSnapshot, LinkSnapshot, LinkState, VehicleSnapshot
from telekart_ui.ui.hud import (
    MARGIN,
    MIN_WIDTH_DETAIL,
    MIN_WIDTH_STATUS,
    HudOverlay,
    Severity,
    ValueFollower,
    build_events,
)

_SIZES = [(1024, 640), (1280, 720), (1920, 1080), (3840, 2160)]


@pytest.fixture
def hud(qapp):
    overlay = HudOverlay()
    # Actually shown. Qt reports every child of a hidden widget as invisible,
    # so an unshown overlay would make each visibility assertion below pass
    # vacuously -- which is worse than no assertion.
    overlay.resize(1280, 720)
    overlay.show()
    overlay.set_vehicle(
        VehicleSnapshot(
            valid=True,
            stale=False,
            state=VehicleState.ARMED,
            speed=1.5,
            v_max=2.0,
            pack_volts=7.4,
            flags=TelemetryFlags.CALIBRATED | TelemetryFlags.CLOSED_LOOP,
        )
    )
    overlay.set_link(
        LinkSnapshot(state=LinkState.LIVE, video_ok=True, video_fps=30.0, rtt=0.03)
    )
    overlay.set_input(InputSnapshot(throttle=0.5, steering=-0.2, transmitting=True))
    yield overlay
    overlay.close()


@pytest.mark.parametrize("size", _SIZES)
def test_every_zone_stays_inside_the_overlay(hud, size):
    hud.resize(*size)
    bounds = QRect(0, 0, *size)
    for name in ("wheel", "status", "banner", "detail"):
        zone = getattr(hud, name)
        if not zone.isVisible():
            continue
        assert bounds.contains(zone.geometry()), f"{name} escapes at {size}"


@pytest.mark.parametrize("size", _SIZES)
def test_instruments_respect_the_margin(hud, size):
    hud.resize(*size)
    assert hud.wheel.geometry().bottom() <= size[1] - MARGIN
    assert hud.status.geometry().right() <= size[0] - MARGIN
    assert hud.status.geometry().top() >= MARGIN


def test_zones_hide_rather_than_overlap_when_narrow(hud):
    """A HUD that collides with itself is worse than one missing a panel."""
    hud.set_detail_visible(True)
    hud.resize(MIN_WIDTH_DETAIL - 1, 600)
    assert hud.detail.isVisible() is False

    hud.resize(MIN_WIDTH_STATUS - 1, 600)
    assert hud.status.isVisible() is False

    hud.resize(1280, 720)
    assert hud.status.isVisible() is True
    assert hud.detail.isVisible() is True


def test_the_banner_never_sits_on_the_instruments(hud):
    """It is the one thing that must stay readable when everything else is wrong."""
    hud.set_vehicle(
        VehicleSnapshot(valid=True, state=VehicleState.ESTOP, faults=Fault.STALL_L)
    )
    for size in _SIZES + [(900, 480)]:
        hud.resize(*size)
        if not (hud.banner.isVisible() and hud.wheel.isVisible()):
            continue
        assert hud.banner.geometry().bottom() < hud.wheel.geometry().top()


def test_hud_can_be_hidden_entirely(hud):
    hud.resize(1280, 720)
    hud.set_instruments_visible(False)
    assert hud.wheel.isVisible() is False
    assert hud.status.isVisible() is False
    hud.set_instruments_visible(True)
    assert hud.wheel.isVisible() is True


# --------------------------------------------------------------------------
# Event ranking
# --------------------------------------------------------------------------


def test_estop_message_names_the_key_that_clears_it():
    """An unactionable warning is a bug, not a message.

    The previous station told the driver "e-stop latched, clear it to re-arm"
    and offered only an unshortcutted menu item to do it with.
    """
    events = build_events(
        VehicleSnapshot(valid=True, state=VehicleState.ESTOP), LinkSnapshot()
    )
    assert events
    top = events[0]
    assert top.severity is Severity.CRITICAL
    assert "Ctrl+Shift+E" in top.detail


def test_link_loss_explains_that_the_car_estopped():
    """Reconnecting is only half the recovery; the driver has to know that."""
    events = build_events(
        VehicleSnapshot(valid=True), LinkSnapshot(state=LinkState.FAILED, detail="socket gone")
    )
    assert any("reconnect" in e.detail for e in events)


def test_critical_faults_outrank_ordinary_ones():
    vehicle = VehicleSnapshot(
        valid=True,
        flags=TelemetryFlags.CALIBRATED,
        faults=Fault.STALL_L | Fault.OVERTEMP,
    )
    events = build_events(vehicle, LinkSnapshot(state=LinkState.LIVE))
    severities = [e.severity for e in events]
    assert severities == sorted(severities, reverse=True)
    assert Fault.OVERTEMP & CRITICAL_FAULTS


def test_no_condition_is_reported_twice():
    """Found running against the real firmware: E-STOP and NOT CALIBRATED each
    appeared twice -- once from the dedicated entry, once from the raw fault
    bit -- and the duplicate carried the useless advice "press F to clear",
    which clears neither of them."""
    vehicle = VehicleSnapshot(
        valid=True,
        state=VehicleState.ESTOP,
        faults=Fault.ESTOP_LATCHED | Fault.CALIBRATION_MISSING,
    )
    events = build_events(vehicle, LinkSnapshot(state=LinkState.LIVE))
    titles = [e.title for e in events]
    assert len(titles) == len(set(titles)), titles
    assert not any(e.detail == "press F to clear" for e in events)


def test_a_healthy_car_says_nothing():
    """Steady-state normality is silent, so a banner always means something."""
    vehicle = VehicleSnapshot(
        valid=True,
        state=VehicleState.ARMED,
        flags=TelemetryFlags.CALIBRATED | TelemetryFlags.CLOSED_LOOP,
    )
    assert build_events(vehicle, LinkSnapshot(state=LinkState.LIVE, video_ok=True)) == []


# --------------------------------------------------------------------------
# Smoothing
# --------------------------------------------------------------------------


def test_value_follower_settles_and_stops():
    """The animation timer stops when nothing is moving; a parked car costs
    no repaints at all."""
    follower = ValueFollower(0.0)
    follower.set_target(1.0)
    moved = 0
    for _ in range(500):
        if not follower.advance(1 / 60):
            break
        moved += 1
    assert follower.settled
    assert follower.value == pytest.approx(1.0, abs=1e-3)
    assert 0 < moved < 500


def test_value_follower_snap_is_immediate():
    follower = ValueFollower(0.0)
    follower.snap(0.75)
    assert follower.value == 0.75
    assert follower.settled
