"""TelemetryPacket -- car to app, 50 Hz over UDP.

Fixed 98 bytes. Everything the HUD, the safety display, and the deferred
racing-sim layer need, in one datagram, so the app always renders a
self-consistent snapshot rather than stitching together fields that arrived at
different times.

Two design notes worth keeping:

* Physical quantities travel as scaled integers (mm, mm/s, centi-degrees,
  deci-Celsius). Exact on the wire, no float endianness questions, and the
  resolution is far finer than the sensors warrant.
* ``v_max_mm_s`` is the *measured* top speed from auto-calibration, not a
  nameplate figure. The app scales its speedometer off it, which is what lets
  the same build work with the current L298N-limited drivetrain and with a
  MOSFET bridge later, with no code change on either side.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

from .constants import (
    DUTY_SCALE,
    MAC_TAG_LEN,
    MAGIC_TELEMETRY,
    PROTO_VERSION,
    Fault,
    TelemetryFlags,
    VehicleState,
)
from .control import ProtocolError
from .crypto import compute_tag, verify_tag

_STRUCT = struct.Struct("<IHHIIQQIIBBhhhhhhHhhHiihIHHhIHH8s")
TELEMETRY_PACKET_LEN = _STRUCT.size
assert TELEMETRY_PACKET_LEN == 98, f"telemetry layout drifted: {TELEMETRY_PACKET_LEN}"

_SIGNED_LEN = TELEMETRY_PACKET_LEN - MAC_TAG_LEN

SLIP_SCALE = 1000.0


def _clamp(value: int, lo: int, hi: int) -> int:
    return lo if value < lo else hi if value > hi else value


def _sat16(value: float) -> int:
    return _clamp(int(round(value)), -32768, 32767)


def _satu16(value: float) -> int:
    return _clamp(int(round(value)), 0, 65535)


def _sat32(value: float) -> int:
    return _clamp(int(round(value)), -2147483648, 2147483647)


def _satu32(value: float) -> int:
    return _clamp(int(round(value)), 0, 4294967295)


@dataclass(frozen=True, slots=True)
class TelemetryPacket:
    # --- identity / timing ---
    session_id: int
    sequence: int
    car_time_us: int
    #: The client_time_us of the most recent accepted control packet, echoed
    #: verbatim. The app subtracts it from its own clock to get true round-trip
    #: latency without needing the two clocks to agree.
    echo_client_time_us: int = 0
    echo_sequence: int = 0

    # --- state ---
    state: VehicleState = VehicleState.BOOT
    faults: Fault = Fault.NONE
    flags: TelemetryFlags = TelemetryFlags.NONE

    # --- drivetrain ---
    rpm_l: int = 0
    rpm_r: int = 0
    rpm_target_l: int = 0
    rpm_target_r: int = 0
    duty_l: int = 0  # -1000 .. +1000
    duty_r: int = 0

    # --- steering ---
    servo_us: int = 0
    steer_angle_cdeg: int = 0  # centi-degrees, signed

    # --- motion ---
    speed_mm_s: int = 0
    v_max_mm_s: int = 0  # measured ceiling; the app scales its gauge to this
    odom_x_mm: int = 0
    odom_y_mm: int = 0
    heading_cdeg: int = 0  # -18000 .. +18000
    distance_mm: int = 0
    slip_index: int = 0  # scaled by SLIP_SCALE

    # --- health ---
    pack_mv: int = 0  # 0 when no battery sensing is fitted
    cpu_temp_dc: int = 0  # deci-Celsius
    throttled: int = 0  # raw vcgencmd get_throttled bitmask
    loop_p99_us: int = 0

    version: int = PROTO_VERSION

    # -- convenience views --------------------------------------------------

    @property
    def duty_l_f(self) -> float:
        return self.duty_l / DUTY_SCALE

    @property
    def duty_r_f(self) -> float:
        return self.duty_r / DUTY_SCALE

    @property
    def speed_mps(self) -> float:
        return self.speed_mm_s / 1000.0

    @property
    def v_max_mps(self) -> float:
        return self.v_max_mm_s / 1000.0

    @property
    def speed_fraction(self) -> float:
        """Speed as a fraction of the measured maximum, for gauge scaling."""
        if self.v_max_mm_s <= 0:
            return 0.0
        return _clamp_f(abs(self.speed_mm_s) / self.v_max_mm_s, 0.0, 1.0)

    @property
    def heading_rad(self) -> float:
        return math.radians(self.heading_cdeg / 100.0)

    @property
    def steer_angle_deg(self) -> float:
        return self.steer_angle_cdeg / 100.0

    @property
    def distance_m(self) -> float:
        return self.distance_mm / 1000.0

    @property
    def pose_m(self) -> tuple[float, float, float]:
        """(x, y, heading) in metres and radians."""
        return (self.odom_x_mm / 1000.0, self.odom_y_mm / 1000.0, self.heading_rad)

    @property
    def slip(self) -> float:
        return self.slip_index / SLIP_SCALE

    @property
    def pack_volts(self) -> float:
        return self.pack_mv / 1000.0

    @property
    def cpu_temp_c(self) -> float:
        return self.cpu_temp_dc / 10.0

    @property
    def armed(self) -> bool:
        return self.state == VehicleState.ARMED

    @property
    def drivable(self) -> bool:
        return self.state in (VehicleState.ARMED, VehicleState.FAILSAFE)

    # -- factory ------------------------------------------------------------

    @classmethod
    def from_si(
        cls,
        *,
        session_id: int,
        sequence: int,
        car_time_us: int,
        state: VehicleState,
        faults: Fault = Fault.NONE,
        flags: TelemetryFlags = TelemetryFlags.NONE,
        echo_client_time_us: int = 0,
        echo_sequence: int = 0,
        rpm_l: float = 0.0,
        rpm_r: float = 0.0,
        rpm_target_l: float = 0.0,
        rpm_target_r: float = 0.0,
        duty_l: float = 0.0,
        duty_r: float = 0.0,
        servo_us: float = 0.0,
        steer_angle_deg: float = 0.0,
        speed_mps: float = 0.0,
        v_max_mps: float = 0.0,
        x_m: float = 0.0,
        y_m: float = 0.0,
        heading_rad: float = 0.0,
        distance_m: float = 0.0,
        slip: float = 0.0,
        pack_volts: float = 0.0,
        cpu_temp_c: float = 0.0,
        throttled: int = 0,
        loop_p99_us: float = 0.0,
    ) -> "TelemetryPacket":
        """Build from SI units. Every field saturates rather than overflowing.

        Saturation matters: a NaN or a runaway value from a divide-by-zero
        upstream must not raise inside the telemetry path and take the link down
        with it. A pinned gauge is diagnosable; a dead link is not.
        """
        heading_deg = math.degrees(_finite(heading_rad))
        # wrap to (-180, 180] so the int16 centi-degree field always fits
        heading_deg = (heading_deg + 180.0) % 360.0 - 180.0
        return cls(
            session_id=session_id,
            sequence=sequence & 0xFFFFFFFF,
            car_time_us=car_time_us & 0xFFFFFFFFFFFFFFFF,
            echo_client_time_us=echo_client_time_us & 0xFFFFFFFFFFFFFFFF,
            echo_sequence=echo_sequence & 0xFFFFFFFF,
            state=state,
            faults=faults,
            flags=flags,
            rpm_l=_sat16(_finite(rpm_l)),
            rpm_r=_sat16(_finite(rpm_r)),
            rpm_target_l=_sat16(_finite(rpm_target_l)),
            rpm_target_r=_sat16(_finite(rpm_target_r)),
            duty_l=_clamp(_sat16(_finite(duty_l) * DUTY_SCALE), -DUTY_SCALE, DUTY_SCALE),
            duty_r=_clamp(_sat16(_finite(duty_r) * DUTY_SCALE), -DUTY_SCALE, DUTY_SCALE),
            servo_us=_satu16(_finite(servo_us)),
            steer_angle_cdeg=_sat16(_finite(steer_angle_deg) * 100.0),
            speed_mm_s=_sat16(_finite(speed_mps) * 1000.0),
            v_max_mm_s=_satu16(_finite(v_max_mps) * 1000.0),
            odom_x_mm=_sat32(_finite(x_m) * 1000.0),
            odom_y_mm=_sat32(_finite(y_m) * 1000.0),
            heading_cdeg=_sat16(heading_deg * 100.0),
            distance_mm=_satu32(_finite(distance_m) * 1000.0),
            slip_index=_satu16(_finite(slip) * SLIP_SCALE),
            pack_mv=_satu16(_finite(pack_volts) * 1000.0),
            cpu_temp_dc=_sat16(_finite(cpu_temp_c) * 10.0),
            throttled=_satu32(throttled),
            loop_p99_us=_satu16(_finite(loop_p99_us)),
        )

    # -- wire format --------------------------------------------------------

    def pack(self, key: bytes) -> bytes:
        body = _STRUCT.pack(
            MAGIC_TELEMETRY,
            self.version,
            int(self.flags) & 0xFFFF,
            self.session_id,
            self.sequence,
            self.car_time_us,
            self.echo_client_time_us,
            self.echo_sequence,
            int(self.faults) & 0xFFFFFFFF,
            int(self.state),
            0,
            self.rpm_l,
            self.rpm_r,
            self.rpm_target_l,
            self.rpm_target_r,
            self.duty_l,
            self.duty_r,
            self.servo_us,
            self.steer_angle_cdeg,
            self.speed_mm_s,
            self.v_max_mm_s,
            self.odom_x_mm,
            self.odom_y_mm,
            self.heading_cdeg,
            self.distance_mm,
            self.slip_index,
            self.pack_mv,
            self.cpu_temp_dc,
            self.throttled,
            self.loop_p99_us,
            0,
            b"\x00" * MAC_TAG_LEN,
        )[:_SIGNED_LEN]
        return body + compute_tag(key, body)

    @classmethod
    def unpack(cls, data: bytes, key: bytes) -> "TelemetryPacket":
        if len(data) != TELEMETRY_PACKET_LEN:
            raise ProtocolError(
                f"telemetry packet is {len(data)} bytes, expected {TELEMETRY_PACKET_LEN}"
            )

        fields = _STRUCT.unpack(data)
        magic, version, flags, session_id, sequence = fields[0:5]

        if magic != MAGIC_TELEMETRY:
            raise ProtocolError(f"bad telemetry magic 0x{magic:08X}")
        if version != PROTO_VERSION:
            raise ProtocolError(
                f"protocol version {version}, this build speaks {PROTO_VERSION}"
            )
        if not verify_tag(key, data[:_SIGNED_LEN], fields[-1]):
            raise ProtocolError("telemetry packet failed authentication")

        (
            car_time_us,
            echo_client_time_us,
            echo_sequence,
            faults,
            state,
            _reserved0,
            rpm_l,
            rpm_r,
            rpm_target_l,
            rpm_target_r,
            duty_l,
            duty_r,
            servo_us,
            steer_angle_cdeg,
            speed_mm_s,
            v_max_mm_s,
            odom_x_mm,
            odom_y_mm,
            heading_cdeg,
            distance_mm,
            slip_index,
            pack_mv,
            cpu_temp_dc,
            throttled,
            loop_p99_us,
        ) = fields[5:30]

        return cls(
            session_id=session_id,
            sequence=sequence,
            car_time_us=car_time_us,
            echo_client_time_us=echo_client_time_us,
            echo_sequence=echo_sequence,
            # A firmware from the future may set state/fault bits this build
            # does not know. Preserve the raw value instead of raising -- an
            # unknown fault bit is still worth showing the driver.
            state=_safe_state(state),
            faults=Fault(faults),
            flags=TelemetryFlags(flags),
            rpm_l=rpm_l,
            rpm_r=rpm_r,
            rpm_target_l=rpm_target_l,
            rpm_target_r=rpm_target_r,
            duty_l=duty_l,
            duty_r=duty_r,
            servo_us=servo_us,
            steer_angle_cdeg=steer_angle_cdeg,
            speed_mm_s=speed_mm_s,
            v_max_mm_s=v_max_mm_s,
            odom_x_mm=odom_x_mm,
            odom_y_mm=odom_y_mm,
            heading_cdeg=heading_cdeg,
            distance_mm=distance_mm,
            slip_index=slip_index,
            pack_mv=pack_mv,
            cpu_temp_dc=cpu_temp_dc,
            throttled=throttled,
            loop_p99_us=loop_p99_us,
            version=version,
        )


def _clamp_f(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def _finite(value: float) -> float:
    """Map NaN/inf to 0. See the note in `from_si` about not raising here."""
    return value if math.isfinite(value) else 0.0


def _safe_state(raw: int) -> VehicleState:
    try:
        return VehicleState(raw)
    except ValueError:
        return VehicleState.FAULT
