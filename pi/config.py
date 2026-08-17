"""Pi-side configuration: pins, ports, limits, camera, servo calibration.

Single source of truth. ``car.py``, ``video.py`` and ``main.py`` read everything from
here; nothing else hard-codes a pin number or a duty limit.

Values that encode a *hardware constraint* are commented with why. If the physical car
ever gets a measured per-vehicle override, its numbers supersede these -- reconcile at
bring-up rather than editing both.
"""

from __future__ import annotations

import os

from protocol import CONTROL_PORT, TLM_HEARTBEAT_HZ, VIDEO_PORT
from vehicle import VehicleConfig

# --- GPIO pin map (BCM numbering) --------------------------------------------
# pigpio always speaks BCM, so there is no numbering mode to get wrong.

ENA = 12  # left bridge enable  -- PWM via pigpio DMA, not hardware_PWM()
ENB = 13  # right bridge enable -- same
IN1 = 5  # left  direction A
IN2 = 6  # left  direction B
IN3 = 20  # right direction A
IN4 = 21  # right direction B

#: GPIO5 and GPIO6 boot HIGH on BCM283x, so IN1/IN2 are asserted from the moment the Pi
#: has power -- before any of our code runs. External 10k pull-downs on all six of these
#: are mandatory, and car.py drives every one of them low as its very first action.
MOTOR_PINS = (ENA, ENB, IN1, IN2, IN3, IN4)

#: The two gearmotors are mounted mirror-image on opposite sides of the chassis, so
#: identical electrical polarity spins them in *opposite* rotational directions. One side
#: has to be inverted for both to push the car the same way. Which side is arbitrary --
#: it depends on how the leads were landed on the L298N screw terminals, so this is
#: calibration, not a constant. Measured at bring-up 2026-08-16: the two motors ran
#: against each other until the right side was inverted.
INVERT_LEFT = False
INVERT_RIGHT = True

# --- Encoders -----------------------------------------------------------------
# Wired since v1 and read by nothing in the control loop. encoder_probe.py reads them
# to measure real wheel speed; because it only ever reads these pins, it can run
# alongside the live service without contending for anything.

ENC_LEFT_A, ENC_LEFT_B = 23, 24
ENC_RIGHT_A, ENC_RIGHT_B = 27, 22

#: **Measured** 2026-08-16: 3905 rising edges on one channel over 10 hand-turned output
#: revolutions of the right wheel. The hardware notes say 660; they also said the
#: regulator was set to 9 V when it is set to 12 V, so they are not a source worth
#: preferring over a direct reading.
#:
#: This counts rising edges on channel A only. Anything that decodes full quadrature will
#: see 4x this, which is probably where 660 came from -- do not mix the two conventions.
ENC_COUNTS_PER_REV = 390

WHEEL_DIAMETER_M = 0.065

SERVO = 18  # HS-311 steering servo, pigpio set_servo_pulsewidth()
LED = 25  # status LED: blink = SAFE, solid = DRIVE, off = FAILSAFE
ESTOP = 16  # momentary button to ground, internal pull-up, active low

#: 1 kHz across the L298N. Low enough that the Darlington outputs actually switch
#: cleanly, high enough to stay out of the audible whine.
PWM_FREQ_HZ = 1000

#: pigpio's duty resolution. Set explicitly with set_PWM_range() so a duty of 255 is
#: unambiguously 100% regardless of the daemon's default.
PWM_RANGE = 255

#: The e-stop button's wiring polarity has not been confirmed on the bench yet, and a
#: mis-read normally-closed switch would look like "permanently pressed" (car never
#: moves) or, worse, "never pressed" (a safety device that does nothing). It stays off
#: until someone puts a meter on it at bring-up. car.py can still read the pin.
ESTOP_ENABLED = False

#: With the internal pull-up on, an unpressed button reads 1 and a pressed one reads 0.
ESTOP_ACTIVE_LOW = True

# --- Network ------------------------------------------------------------------
# Ports come from protocol.py; re-exported so nothing on the Pi imports two modules to
# find out where to listen.

VIDEO_PORT = VIDEO_PORT
CONTROL_PORT = CONTROL_PORT

#: Bind the control socket on all interfaces -- the Mac's address is learned from
#: whatever address the last cmd arrived from, never configured.
BIND_HOST = "0.0.0.0"

# --- Control loop -------------------------------------------------------------

CONTROL_RATE_HZ = 100.0
CONTROL_PERIOD_S = 1.0 / CONTROL_RATE_HZ

#: 1 Hz human-readable summary to stdout, alongside the full jsonl log.
SUMMARY_PERIOD_S = 1.0

#: Telemetry is answered per command, so this only paces the quiet case -- see
#: protocol.TLM_HEARTBEAT_HZ.
TLM_HEARTBEAT_HZ = TLM_HEARTBEAT_HZ

# --- Camera -------------------------------------------------------------------

CAM_WIDTH = 640
CAM_HEIGHT = 480

#: 41.85 fps is the hard ceiling for the full-FOV 1640x1232 10-bit sensor mode below, so
#: 40 is deliberately just under it -- not a round number picked for looks. (v1 ran at 20
#: purely out of caution about CPU and Wi-Fi headroom; measurement showed there was room.)
#:
#: This pins exposure at 25 ms, half of what 20 fps allowed. Indoors that is a stop of
#: light gone and the auto-gain makes it up with noise, which is the accepted cost of a
#: fixed rate: a constant frame interval keeps latency predictable. Going higher needs
#: 8-bit raw, which changes the sensor mode chosen for field of view -- see CAM_RAW_SIZE.
CAM_FPS = 40
CAM_JPEG_QUALITY = 70  # legible at 640x480 while staying well inside the Wi-Fi budget

#: The full-FOV binned sensor mode. A bare 640x480 request lands the IMX219 on a
#: 1280x960 *crop* -- a ~2.6x telephoto that is genuinely hard to drive because you
#: cannot see the ground in front of the wheels. Asking for this raw size forces the
#: binned full-frame mode and the 640x480 main stream is then a downscale of it.
CAM_RAW_SIZE = (1640, 1232)

# --- Paths --------------------------------------------------------------------

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "logs")

# --- Vehicle tunables ---------------------------------------------------------


def vehicle_config() -> VehicleConfig:
    """The Pi's measured values for the shared behaviour model.

    Only the fields this vehicle actually differs on -- or that matter enough to state
    explicitly rather than inherit -- are set here.
    """
    return VehicleConfig(
        # Nothing moves below ~30%: the L298N's Darlington stage drops ~1.4 V, so the
        # motors see far less than the battery. Throttle maps into MIN..MAX, never
        # up from zero.
        min_duty=0.30,
        # 0.60 -> 0.85 -> 1.00 over bring-up on 2026-08-16. A 45 s drive at 0.85 with
        # full-throttle stabs both directions left `vcgencmd get_throttled` at 0x0 and
        # the SoC at 47 C, so the Pololu regulator's ~1.5 A budget was not the binding
        # constraint it was feared to be. The slew limiter below is now the only thing
        # standing between a throttle stab and a step current draw -- do not remove it
        # to "make it more responsive".
        max_duty=1.00,
        # Throttle units per second: 0 -> full in ~0.4 s. A step input into the
        # regulator is how you brown out the SoC mid-drive.
        slew_per_s=2.5,
        # --- Steering, measured on this car 2026-08-16 -------------------------
        # These are measurements, not adjustments to a default. The checked-in defaults
        # (centre 1500, +-300 us, +-24 deg) describe no part of this vehicle: its horn
        # sits far enough round the spline that straight-ahead is 1900 us, and it has
        # more travel than an HS-311's nominal 900-2100 span. Verified by sweeping with
        # the wheels up and reading the angle off by eye.
        servo_centre_us=1950,  # wheels straight
        servo_range_left_us=400,  # 1550 us, full left
        servo_range_right_us=450,  # 2400 us, full right; the linkage is not symmetric
        servo_min_us=1550,  # hard clamps at the measured locks. Past these the linkage
        servo_max_us=2400,  # binds and the servo stalls -- and it sits on the Pi's 5 V
        servo_trim_us=0,  # rail, so a stall browns out the SoC rather than just buzzing
    )
