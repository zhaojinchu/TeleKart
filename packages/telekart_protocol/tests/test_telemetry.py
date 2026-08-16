"""Telemetry packet contract tests.

Telemetry is the direction where a raise is unacceptable. The car builds one of
these 50 times a second from live floats produced by an odometry integrator and
a velocity estimator, either of which can hand it a NaN after a bad dt. A
`from_si` that threw would take the link down at precisely the moment the driver
most needs to see what happened -- so every field saturates, and these tests
exist to keep it that way.
"""

from __future__ import annotations

import dataclasses
import math
import random
import struct

import pytest

from telekart_protocol.constants import (
    CRITICAL_FAULTS,
    DUTY_SCALE,
    MAGIC_TELEMETRY,
    PROTO_VERSION,
    Fault,
    TelemetryFlags,
    VehicleState,
)
from telekart_protocol.control import ProtocolError
from telekart_protocol.telemetry import (
    SLIP_SCALE,
    TELEMETRY_PACKET_LEN,
    TelemetryPacket,
)

from . import flip_bit, splice

# Byte offsets derived from the struct format in telemetry.py. Spelled out so a
# layout change fails here with a readable name rather than as a mystery.
OFF_MAGIC = 0
OFF_VERSION = 4
OFF_FLAGS = 6
OFF_SESSION = 8
OFF_SEQUENCE = 12
OFF_CAR_TIME = 16
OFF_ECHO_TIME = 24
OFF_ECHO_SEQ = 32
OFF_FAULTS = 36
OFF_STATE = 40
OFF_RESERVED0 = 41
OFF_RPM_L = 42

GOLDEN_FLAGS = (
    TelemetryFlags.CALIBRATED
    | TelemetryFlags.CLOSED_LOOP
    | TelemetryFlags.ODOM_VALID
    | TelemetryFlags.VIDEO_ACTIVE
)
GOLDEN_FAULTS = Fault.LOW_BATTERY | Fault.PI_THROTTLED

GOLDEN_TELEMETRY = TelemetryPacket(
    session_id=0x11223344,
    sequence=1234,
    car_time_us=0x0000000123456789,
    echo_client_time_us=0x00000000DEADBEEF,
    echo_sequence=1233,
    state=VehicleState.ARMED,
    faults=GOLDEN_FAULTS,
    flags=GOLDEN_FLAGS,
    rpm_l=123,
    rpm_r=-45,
    rpm_target_l=130,
    rpm_target_r=-50,
    duty_l=500,
    duty_r=-250,
    servo_us=1520,
    steer_angle_cdeg=-750,
    speed_mm_s=850,
    v_max_mm_s=1250,
    odom_x_mm=2500,
    odom_y_mm=-1250,
    heading_cdeg=4500,
    distance_mm=13750,
    slip_index=42,
    pack_mv=7200,
    cpu_temp_dc=515,
    throttled=0x00050005,
    loop_p99_us=10250,
)

GOLDEN_TELEMETRY_HEX = (
    "54544b31"  # magic 'TTK1'
    "0300"  # version 3
    "8c01"  # flags CALIBRATED|CLOSED_LOOP|ODOM_VALID|VIDEO_ACTIVE
    "44332211"  # session_id
    "d2040000"  # sequence 1234
    "8967452301000000"  # car_time_us
    "efbeadde00000000"  # echo_client_time_us
    "d1040000"  # echo_sequence 1233
    "80200000"  # faults LOW_BATTERY|PI_THROTTLED
    "02"  # state ARMED
    "00"  # reserved
    "7b00"  # rpm_l 123
    "d3ff"  # rpm_r -45
    "8200"  # rpm_target_l 130
    "ceff"  # rpm_target_r -50
    "f401"  # duty_l 500
    "06ff"  # duty_r -250
    "f005"  # servo_us 1520
    "12fd"  # steer_angle_cdeg -750
    "5203"  # speed_mm_s 850
    "e204"  # v_max_mm_s 1250
    "c4090000"  # odom_x_mm 2500
    "1efbffff"  # odom_y_mm -1250
    "9411"  # heading_cdeg 4500
    "b6350000"  # distance_mm 13750
    "2a00"  # slip_index 42
    "201c"  # pack_mv 7200
    "0302"  # cpu_temp_dc 515
    "05000500"  # throttled 0x00050005
    "0a28"  # loop_p99_us 10250
    "0000"  # reserved
)
GOLDEN_TELEMETRY_BYTES = bytes.fromhex(GOLDEN_TELEMETRY_HEX)


def _wrap_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _angular_error(a: float, b: float) -> float:
    return abs(_wrap_rad(a - b))


# ---------------------------------------------------------------- layout


def test_struct_size_is_pinned() -> None:
    assert TELEMETRY_PACKET_LEN == 90
    assert struct.calcsize("<IHHIIQQIIBBhhhhhhHhhHiihIHHhIHH") == 90


def test_magic_is_the_documented_ascii() -> None:
    assert MAGIC_TELEMETRY.to_bytes(4, "little") == b"TTK1"


def test_slip_scale_is_pinned() -> None:
    assert SLIP_SCALE == 1000.0
    assert DUTY_SCALE == 1000


def test_defined_bits_fit_their_wire_fields() -> None:
    # flags is uint16, faults is uint32. Adding a bit past either width would
    # otherwise fail as a struct.error inside the car's 50 Hz TX path.
    assert max(int(f) for f in TelemetryFlags) < 1 << 16
    assert max(int(f) for f in Fault) < 1 << 32
    assert max(int(s) for s in VehicleState) < 1 << 8


def test_critical_faults_are_a_subset_of_fault() -> None:
    # A critical fault forces a disarm; every bit in the mask must be a real
    # fault the firmware can actually raise.
    assert CRITICAL_FAULTS != Fault.NONE
    for bit in CRITICAL_FAULTS:
        assert bit in Fault


# --------------------------------------------------------------- golden


def test_golden_pack() -> None:
    assert GOLDEN_TELEMETRY.pack().hex() == GOLDEN_TELEMETRY_BYTES.hex()


def test_golden_unpack() -> None:
    pkt = TelemetryPacket.unpack(GOLDEN_TELEMETRY_BYTES)
    assert pkt == GOLDEN_TELEMETRY
    assert pkt.state is VehicleState.ARMED
    assert pkt.faults == GOLDEN_FAULTS
    assert pkt.flags == GOLDEN_FLAGS
    assert pkt.version == PROTO_VERSION


def test_golden_si_views() -> None:
    pkt = TelemetryPacket.unpack(GOLDEN_TELEMETRY_BYTES)
    assert pkt.duty_l_f == pytest.approx(0.5)
    assert pkt.duty_r_f == pytest.approx(-0.25)
    assert pkt.speed_mps == pytest.approx(0.85)
    assert pkt.v_max_mps == pytest.approx(1.25)
    assert pkt.speed_fraction == pytest.approx(0.68)
    assert pkt.heading_rad == pytest.approx(math.radians(45.0))
    assert pkt.steer_angle_deg == pytest.approx(-7.5)
    assert pkt.distance_m == pytest.approx(13.75)
    assert pkt.pose_m[0] == pytest.approx(2.5)
    assert pkt.pose_m[1] == pytest.approx(-1.25)
    assert pkt.pose_m[2] == pytest.approx(math.radians(45.0))
    assert pkt.slip == pytest.approx(0.042)
    assert pkt.pack_volts == pytest.approx(7.2)
    assert pkt.cpu_temp_c == pytest.approx(51.5)
    assert pkt.armed is True
    assert pkt.drivable is True


def test_from_si_reproduces_the_golden_packet() -> None:
    # Pins the SI -> scaled-integer conversion for every field at once.
    assert (
        TelemetryPacket.from_si(
            session_id=0x11223344,
            sequence=1234,
            car_time_us=0x0000000123456789,
            echo_client_time_us=0x00000000DEADBEEF,
            echo_sequence=1233,
            state=VehicleState.ARMED,
            faults=GOLDEN_FAULTS,
            flags=GOLDEN_FLAGS,
            rpm_l=123.0,
            rpm_r=-45.0,
            rpm_target_l=130.0,
            rpm_target_r=-50.0,
            duty_l=0.5,
            duty_r=-0.25,
            servo_us=1520.0,
            steer_angle_deg=-7.5,
            speed_mps=0.85,
            v_max_mps=1.25,
            x_m=2.5,
            y_m=-1.25,
            heading_rad=math.radians(45.0),
            distance_m=13.75,
            slip=0.042,
            pack_volts=7.2,
            cpu_temp_c=51.5,
            throttled=0x00050005,
            loop_p99_us=10250.0,
        )
        == GOLDEN_TELEMETRY
    )


def test_packet_is_immutable() -> None:
    with pytest.raises(dataclasses.FrozenInstanceError):
        GOLDEN_TELEMETRY.rpm_l = 0  # type: ignore[misc]


# ---------------------------------------------------------- round trips


def test_default_packet_round_trips() -> None:
    pkt = TelemetryPacket(session_id=0, sequence=0, car_time_us=0)
    assert TelemetryPacket.unpack(pkt.pack()) == pkt


@pytest.mark.parametrize("state", list(VehicleState))
def test_every_state_round_trips(state: VehicleState) -> None:
    pkt = TelemetryPacket(session_id=1, sequence=1, car_time_us=1, state=state)
    assert TelemetryPacket.unpack(pkt.pack()).state is state


@pytest.mark.parametrize("fault", list(Fault))
def test_every_fault_round_trips(fault: Fault) -> None:
    pkt = TelemetryPacket(session_id=1, sequence=1, car_time_us=1, faults=fault)
    assert TelemetryPacket.unpack(pkt.pack()).faults == fault


def test_all_faults_at_once_round_trip() -> None:
    every = Fault(0)
    for fault in Fault:
        every |= fault
    pkt = TelemetryPacket(session_id=1, sequence=1, car_time_us=1, faults=every)
    assert TelemetryPacket.unpack(pkt.pack()).faults == every


def test_all_flags_at_once_round_trip() -> None:
    every = TelemetryFlags(0)
    for flag in TelemetryFlags:
        every |= flag
    pkt = TelemetryPacket(session_id=1, sequence=1, car_time_us=1, flags=every)
    assert TelemetryPacket.unpack(pkt.pack()).flags == every


def test_randomized_round_trip() -> None:
    rng = random.Random(0xBEEF)
    for _ in range(400):
        pkt = TelemetryPacket(
            session_id=rng.getrandbits(32),
            sequence=rng.getrandbits(32),
            car_time_us=rng.getrandbits(64),
            echo_client_time_us=rng.getrandbits(64),
            echo_sequence=rng.getrandbits(32),
            state=rng.choice(list(VehicleState)),
            faults=Fault(rng.getrandbits(17)),
            flags=TelemetryFlags(rng.getrandbits(9)),
            rpm_l=rng.randint(-32768, 32767),
            rpm_r=rng.randint(-32768, 32767),
            rpm_target_l=rng.randint(-32768, 32767),
            rpm_target_r=rng.randint(-32768, 32767),
            duty_l=rng.randint(-DUTY_SCALE, DUTY_SCALE),
            duty_r=rng.randint(-DUTY_SCALE, DUTY_SCALE),
            servo_us=rng.randint(0, 65535),
            steer_angle_cdeg=rng.randint(-32768, 32767),
            speed_mm_s=rng.randint(-32768, 32767),
            v_max_mm_s=rng.randint(0, 65535),
            odom_x_mm=rng.randint(-2147483648, 2147483647),
            odom_y_mm=rng.randint(-2147483648, 2147483647),
            heading_cdeg=rng.randint(-18000, 18000),
            distance_mm=rng.randint(0, 4294967295),
            slip_index=rng.randint(0, 65535),
            pack_mv=rng.randint(0, 65535),
            cpu_temp_dc=rng.randint(-32768, 32767),
            throttled=rng.getrandbits(32),
            loop_p99_us=rng.randint(0, 65535),
        )
        assert TelemetryPacket.unpack(pkt.pack()) == pkt


# ----------------------------------------------------- saturation / NaN


@pytest.mark.parametrize(
    "field,value,expected",
    [
        ("rpm_l", 1e9, 32767),
        ("rpm_l", -1e9, -32768),
        ("rpm_r", 1e9, 32767),
        ("rpm_target_l", -1e9, -32768),
        ("speed_mps", 1e6, 32767),
        ("speed_mps", -1e6, -32768),
        ("steer_angle_deg", 1e6, 32767),
        ("cpu_temp_c", -1e6, -32768),
    ],
)
def test_signed_fields_saturate(field: str, value: float, expected: int) -> None:
    pkt = TelemetryPacket.from_si(
        session_id=1, sequence=1, car_time_us=1, state=VehicleState.SAFE, **{field: value}
    )
    wire_field = {
        "speed_mps": "speed_mm_s",
        "steer_angle_deg": "steer_angle_cdeg",
        "cpu_temp_c": "cpu_temp_dc",
    }.get(field, field)
    assert getattr(pkt, wire_field) == expected


@pytest.mark.parametrize(
    "field,wire_field,value,expected",
    [
        ("servo_us", "servo_us", -100.0, 0),
        ("servo_us", "servo_us", 1e9, 65535),
        ("v_max_mps", "v_max_mm_s", -1.0, 0),
        ("v_max_mps", "v_max_mm_s", 1e6, 65535),
        ("distance_m", "distance_mm", -5.0, 0),
        ("distance_m", "distance_mm", 1e9, 4294967295),
        ("slip", "slip_index", -1.0, 0),
        ("slip", "slip_index", 1e6, 65535),
        ("pack_volts", "pack_mv", -1.0, 0),
        ("pack_volts", "pack_mv", 1e6, 65535),
        ("loop_p99_us", "loop_p99_us", -1.0, 0),
        ("loop_p99_us", "loop_p99_us", 1e9, 65535),
        ("throttled", "throttled", -1, 0),
        ("throttled", "throttled", 1 << 40, 4294967295),
        ("x_m", "odom_x_mm", 1e9, 2147483647),
        ("x_m", "odom_x_mm", -1e9, -2147483648),
        ("y_m", "odom_y_mm", -1e9, -2147483648),
    ],
)
def test_bounded_fields_saturate(
    field: str, wire_field: str, value: float, expected: int
) -> None:
    pkt = TelemetryPacket.from_si(
        session_id=1, sequence=1, car_time_us=1, state=VehicleState.SAFE, **{field: value}
    )
    assert getattr(pkt, wire_field) == expected


@pytest.mark.parametrize("duty,expected", [(0.0, 0), (1.0, 1000), (-1.0, -1000), (5.0, 1000), (-5.0, -1000)])
def test_duty_is_clamped_to_full_scale(duty: float, expected: int) -> None:
    pkt = TelemetryPacket.from_si(
        session_id=1, sequence=1, car_time_us=1, state=VehicleState.ARMED, duty_l=duty, duty_r=-duty
    )
    assert pkt.duty_l == expected
    assert pkt.duty_r == -expected


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_nonfinite_becomes_zero_not_saturation(bad: float) -> None:
    """Infinity maps to 0, not to the field maximum. That is deliberate.

    A pinned gauge reads as "the car is doing 32 m/s", which is a plausible-
    looking lie. Zero reads as "this channel is broken", which is what a NaN
    upstream actually means.
    """
    pkt = TelemetryPacket.from_si(
        session_id=1,
        sequence=1,
        car_time_us=1,
        state=VehicleState.ARMED,
        rpm_l=bad,
        rpm_r=bad,
        rpm_target_l=bad,
        rpm_target_r=bad,
        duty_l=bad,
        duty_r=bad,
        servo_us=bad,
        steer_angle_deg=bad,
        speed_mps=bad,
        v_max_mps=bad,
        x_m=bad,
        y_m=bad,
        heading_rad=bad,
        distance_m=bad,
        slip=bad,
        pack_volts=bad,
        cpu_temp_c=bad,
        loop_p99_us=bad,
    )
    assert pkt == TelemetryPacket(
        session_id=1, sequence=1, car_time_us=1, state=VehicleState.ARMED
    )
    assert len(pkt.pack()) == TELEMETRY_PACKET_LEN


def test_counters_are_masked_not_overflowed() -> None:
    pkt = TelemetryPacket.from_si(
        session_id=7,
        sequence=(1 << 32) + 9,
        car_time_us=(1 << 64) + 11,
        echo_client_time_us=(1 << 64) + 13,
        echo_sequence=(1 << 32) + 17,
        state=VehicleState.SAFE,
    )
    assert pkt.sequence == 9
    assert pkt.car_time_us == 11
    assert pkt.echo_client_time_us == 13
    assert pkt.echo_sequence == 17


def test_speed_fraction_is_safe_without_calibration() -> None:
    # v_max is zero until auto-calibration has run. The gauge must not divide by
    # it, and must not read full-scale either.
    pkt = TelemetryPacket(session_id=1, sequence=1, car_time_us=1, speed_mm_s=500, v_max_mm_s=0)
    assert pkt.speed_fraction == 0.0


def test_speed_fraction_clamps_above_measured_maximum() -> None:
    pkt = TelemetryPacket(session_id=1, sequence=1, car_time_us=1, speed_mm_s=5000, v_max_mm_s=1000)
    assert pkt.speed_fraction == 1.0
    reverse = TelemetryPacket(
        session_id=1, sequence=1, car_time_us=1, speed_mm_s=-500, v_max_mm_s=1000
    )
    assert reverse.speed_fraction == pytest.approx(0.5)


# ------------------------------------------------------------- heading


@pytest.mark.parametrize(
    "heading",
    [
        0.0,
        0.5,
        -0.5,
        math.pi / 2,
        -math.pi / 2,
        math.pi,
        -math.pi,
        3 * math.pi / 2,
        -3 * math.pi / 2,
        2 * math.pi,
        -2 * math.pi,
        10 * math.pi + 0.3,
        -10 * math.pi - 0.3,
        1000.0,
        -1000.0,
    ],
)
def test_heading_wraps_into_the_int16_field(heading: float) -> None:
    # An odometry integrator accumulates heading without bound. Centi-degrees in
    # an int16 only reach +-327.67 degrees, so wrapping is not cosmetic: without
    # it the field silently saturates and the map view stops turning.
    pkt = TelemetryPacket.from_si(
        session_id=1, sequence=1, car_time_us=1, state=VehicleState.ARMED, heading_rad=heading
    )
    assert -18000 <= pkt.heading_cdeg <= 18000
    # One centi-degree of quantisation is 1.745e-4 rad; allow a little slack.
    assert _angular_error(pkt.heading_rad, heading) < 3e-4


def test_heading_wrap_boundary_is_negative_pi() -> None:
    # (x + 180) % 360 - 180 maps both +180 and -180 onto -180. Pinned because a
    # HUD comparing against +180 would flicker at due south otherwise.
    for heading in (math.pi, -math.pi):
        pkt = TelemetryPacket.from_si(
            session_id=1, sequence=1, car_time_us=1, state=VehicleState.SAFE, heading_rad=heading
        )
        assert pkt.heading_cdeg == -18000


def test_heading_accumulates_without_drift_across_many_wraps() -> None:
    # Simulates 200 laps of a tight circle: the pose the app renders must track
    # the car's unwrapped heading exactly, not walk away from it.
    heading = 0.0
    for _ in range(2000):
        heading += 0.63
        pkt = TelemetryPacket.from_si(
            session_id=1, sequence=1, car_time_us=1, state=VehicleState.ARMED, heading_rad=heading
        )
        assert _angular_error(pkt.heading_rad, heading) < 3e-4


# ----------------------------------------------------------- rejection


def test_bad_magic_is_rejected() -> None:
    from telekart_protocol.constants import MAGIC_CONTROL

    bad = splice(GOLDEN_TELEMETRY_BYTES, OFF_MAGIC, MAGIC_CONTROL.to_bytes(4, "little"))
    with pytest.raises(ProtocolError, match="magic"):
        TelemetryPacket.unpack(bad)


@pytest.mark.parametrize("version", [0, 1, 2, 4, 65535])
def test_version_mismatch_is_rejected(version: int) -> None:
    bad = splice(GOLDEN_TELEMETRY_BYTES, OFF_VERSION, version.to_bytes(2, "little"))
    with pytest.raises(ProtocolError, match="version"):
        TelemetryPacket.unpack(bad)


@pytest.mark.parametrize("length", [0, 1, 40, 97, 99, 256])
def test_wrong_length_is_rejected(length: int) -> None:
    data = (GOLDEN_TELEMETRY_BYTES + bytes(256))[:length]
    with pytest.raises(ProtocolError, match="bytes"):
        TelemetryPacket.unpack(data)


def test_garbage_only_ever_raises_protocol_error() -> None:
    rng = random.Random(4242)
    for _ in range(2000):
        size = rng.choice([0, 1, 50, 89, 90, 91, 300])
        data = bytes(rng.getrandbits(8) for _ in range(size))
        try:
            TelemetryPacket.unpack(data)
        except ProtocolError:
            pass


# ------------------------------------------------ forward compatibility


def test_unknown_state_degrades_to_fault() -> None:
    # A firmware from the future may report a state this app has never heard of.
    # Showing FAULT is honest; raising would blank the whole HUD.
    bad = splice(GOLDEN_TELEMETRY_BYTES, OFF_STATE, b"\x63")
    pkt = TelemetryPacket.unpack(bad)
    assert pkt.state is VehicleState.FAULT


def test_unknown_fault_bit_is_preserved() -> None:
    bad = splice(GOLDEN_TELEMETRY_BYTES, OFF_FAULTS, struct.pack("<I", 1 << 31))
    pkt = TelemetryPacket.unpack(bad)
    assert int(pkt.faults) & (1 << 31)


def test_unknown_telemetry_flag_is_preserved() -> None:
    bad = splice(GOLDEN_TELEMETRY_BYTES, OFF_FLAGS, struct.pack("<H", 1 << 15))
    pkt = TelemetryPacket.unpack(bad)
    assert int(pkt.flags) & (1 << 15)


def test_nonzero_reserved_is_accepted() -> None:
    bad = splice(GOLDEN_TELEMETRY_BYTES, OFF_RESERVED0, b"\x7f")
    assert TelemetryPacket.unpack(bad).rpm_l == 123


# ---------------------------------------------------- state predicates


def test_armed_and_drivable_predicates() -> None:
    def make(state: VehicleState) -> TelemetryPacket:
        return TelemetryPacket(session_id=1, sequence=1, car_time_us=1, state=state)

    assert make(VehicleState.ARMED).armed is True
    assert make(VehicleState.FAILSAFE).armed is False
    # FAILSAFE still counts as drivable: the wheels can be turning, so the HUD
    # must keep treating the car as live rather than greying itself out.
    assert make(VehicleState.FAILSAFE).drivable is True
    assert make(VehicleState.ARMED).drivable is True
    for state in (VehicleState.BOOT, VehicleState.SAFE, VehicleState.ESTOP, VehicleState.FAULT):
        assert make(state).armed is False
        assert make(state).drivable is False
