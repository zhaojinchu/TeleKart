"""Wire-level constants shared by the firmware, the desktop app, and the simulator.

Nothing in this module may depend on anything outside the standard library.
"""

from __future__ import annotations

import enum

# --------------------------------------------------------------------------
# Versioning
# --------------------------------------------------------------------------

#: Bumped on any incompatible change to packet layout or session semantics.
#: The handshake rejects a mismatch outright rather than trying to negotiate --
#: a silently-misparsed control packet is a runaway car.
PROTO_VERSION = 2

#: Packet magic numbers. Present so a stray datagram from an unrelated service
#: (or an old v1 ESP32 board still on the network) is rejected before we even
#: look at the version field.
MAGIC_CONTROL = 0x314B5443  # 'CTK1' little-endian
MAGIC_TELEMETRY = 0x314B5454  # 'TTK1' little-endian
MAGIC_VIDEO = 0x44565854  # 'TXVD' little-endian


# --------------------------------------------------------------------------
# Ports
# --------------------------------------------------------------------------

UDP_CONTROL_PORT = 4210  # car listens; app -> car
UDP_TELEMETRY_PORT = 4211  # app listens; car -> app
TCP_SESSION_PORT = 4212  # car listens; handshake, params, arm/disarm
TCP_VIDEO_PORT = 4213  # car listens; H.264 / MJPEG frame stream

MDNS_SERVICE_TYPE = "_telekart._tcp.local."


# --------------------------------------------------------------------------
# Rates and timeouts
# --------------------------------------------------------------------------

CONTROL_RATE_HZ = 100
TELEMETRY_RATE_HZ = 50
CONTROL_LOOP_HZ = 100

#: No valid control packet for this long -> the car begins its failsafe ramp.
#: Relaxed from the old ESP32 firmware's 120 ms: Pi Zero 2 W WiFi is markedly
#: lumpier than the ESP32's, and a spurious failsafe mid-corner is its own hazard.
CONTROL_TIMEOUT_MS = 200

#: Failsafe schedule, measured from the moment the link goes stale.
FAILSAFE_BRAKE_AT_MS = 50  # coast first, then brake -- an unbraked car rolls metres
FAILSAFE_COAST_AT_MS = 450
FAILSAFE_DISARM_AT_MS = 1000

#: A session with no traffic at all for this long is discarded by the car.
SESSION_TTL_S = 300


# --------------------------------------------------------------------------
# Command scaling
# --------------------------------------------------------------------------

#: Control commands travel as integers so the wire format is exact and
#: endian-safe. These are the full-scale values.
STEERING_SCALE = 1000  # -1000 .. +1000  (left negative, right positive)
THROTTLE_SCALE = 1000  # 0 .. 1000
BRAKE_SCALE = 1000  # 0 .. 1000
DUTY_SCALE = 1000  # -1000 .. +1000 in telemetry


# --------------------------------------------------------------------------
# Enumerations
# --------------------------------------------------------------------------


class VehicleState(enum.IntEnum):
    """Authoritative vehicle state. The car owns this; the app only displays it.

    The app must never assume it is armed because it sent an ARM -- it is armed
    when telemetry says so.
    """

    BOOT = 0
    SAFE = 1
    ARMED = 2
    FAILSAFE = 3
    ESTOP = 4
    FAULT = 5


class ControlFlags(enum.IntFlag):
    """Bits in ControlPacket.flags (app -> car)."""

    NONE = 0
    ESTOP = 1 << 0
    REVERSE_REQ = 1 << 1
    RESET_ODOM = 1 << 2
    PIT_LIMITER = 1 << 3
    ARM_INTENT = 1 << 4  # heartbeat-level "I still want to be armed"
    HEADLIGHTS = 1 << 5
    HORN = 1 << 6


class TelemetryFlags(enum.IntFlag):
    """Bits in TelemetryPacket.flags (car -> app)."""

    NONE = 0
    #: A firmware protection limiter is clipping the command. If this lights up
    #: during normal driving, the app's rate limits are looser than the
    #: firmware's and the HUD will disagree with the car -- which feels awful.
    LIMITER_ACTIVE = 1 << 0
    #: Encoders are counting but commanded duty is zero, so the sign of motion
    #: is a guess (see the x2-decode tradeoff in docs/tuning.md).
    DIRECTION_UNCERTAIN = 1 << 1
    CALIBRATED = 1 << 2
    CLOSED_LOOP = 1 << 3
    REVERSE_ENGAGED = 1 << 4
    PIT_LIMITER = 1 << 5
    BRAKING = 1 << 6
    ODOM_VALID = 1 << 7
    VIDEO_ACTIVE = 1 << 8


class Fault(enum.IntFlag):
    """Bits in TelemetryPacket.faults. Sticky until explicitly cleared."""

    NONE = 0
    STALL_L = 1 << 0
    STALL_R = 1 << 1
    ENCODER_FAIL_L = 1 << 2
    ENCODER_FAIL_R = 1 << 3
    #: Both encoders hit zero simultaneously while commanded -- that is the
    #: boost regulator tripping, not a mechanical stall. Distinct code so the
    #: two are never confused during diagnosis.
    BROWNOUT = 1 << 4
    CONTROL_TIMEOUT = 1 << 5
    OVERTEMP = 1 << 6
    LOW_BATTERY = 1 << 7
    CRITICAL_BATTERY = 1 << 8
    GPIO_ERROR = 1 << 9
    LOOP_OVERRUN = 1 << 10
    SERVO_FAULT = 1 << 11
    PI_UNDERVOLTAGE = 1 << 12  # from vcgencmd get_throttled
    PI_THROTTLED = 1 << 13
    CAMERA_DOWN = 1 << 14
    ESTOP_LATCHED = 1 << 15
    CALIBRATION_MISSING = 1 << 16


#: Faults that must force a disarm rather than merely warn.
CRITICAL_FAULTS = (
    Fault.CRITICAL_BATTERY
    | Fault.GPIO_ERROR
    | Fault.OVERTEMP
    | Fault.ESTOP_LATCHED
)


class VideoFrameFlags(enum.IntFlag):
    """Bits in the video frame header."""

    NONE = 0
    KEYFRAME = 1 << 0
    #: Frames were dropped by the sender's bounded queue before this one.
    #: The decoder can expect corruption until the next keyframe.
    DROPPED_BEFORE = 1 << 1
    CONFIG = 1 << 2  # SPS/PPS-only frame


class VideoCodec(enum.IntEnum):
    H264 = 0
    MJPEG = 1


# --------------------------------------------------------------------------
# Authentication
# --------------------------------------------------------------------------

#: Truncated HMAC-SHA256 length in bytes. Eight bytes (widened from the old
#: firmware's four) is ample to stop a stale second laptop from commanding the
#: car, which is the actual threat model here -- not a determined attacker.
MAC_TAG_LEN = 8

#: Replay window. A control packet whose sequence is not greater than the last
#: accepted one is dropped.
SEQUENCE_REPLAY_WINDOW = 0  # strictly increasing


# --------------------------------------------------------------------------
# Limits used for validation on both sides
# --------------------------------------------------------------------------

MAX_CONTROL_PACKET_LEN = 128
MAX_TELEMETRY_PACKET_LEN = 256
MAX_SESSION_LINE_LEN = 64 * 1024
MAX_VIDEO_FRAME_LEN = 4 * 1024 * 1024
