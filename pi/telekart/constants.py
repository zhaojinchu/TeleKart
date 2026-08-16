"""Physical facts about this specific vehicle, plus the timing budgets derived from them.

Nothing here is a preference. Pin numbers are how the board is soldered, voltages are what
the regulator is set to, and the motor figures come from the datasheet. Anything that is a
matter of taste belongs in ``telekart_protocol.params`` so it can be tuned from the desktop
app instead of edited here.
"""

from __future__ import annotations

import math
from pathlib import Path

from telekart_protocol import (
    CONTROL_LOOP_HZ,
    CONTROL_RATE_HZ,
    TELEMETRY_RATE_HZ,
)

FIRMWARE_VERSION = "2.0.0"

# --------------------------------------------------------------------------
# Filesystem layout
# --------------------------------------------------------------------------

#: ``pi/telekart``
PACKAGE_DIR = Path(__file__).resolve().parent
#: ``pi``
PI_ROOT = PACKAGE_DIR.parent

CONFIG_DIR = PI_ROOT / "config"

#: Checked into git; hand-edited; the same on every car.
DEFAULT_CONFIG_PATH = CONFIG_DIR / "telekart.yaml"
#: Per-vehicle overlay. Git-ignored, so a trim value measured on one car never
#: silently becomes the default for another.
LOCAL_CONFIG_PATH = CONFIG_DIR / "config.local.yaml"
#: Machine-written. Kept apart from the config files precisely so an automated
#: calibration run can never clobber a hand-edited setting.
CALIBRATION_PATH = CONFIG_DIR / "calibration.yaml"

#: Environment variables honoured across the firmware.
ENV_BACKEND = "TELEKART_BACKEND"
ENV_CONFIG = "TELEKART_CONFIG"
ENV_LOCAL_CONFIG = "TELEKART_LOCAL_CONFIG"
ENV_SHARED_KEY = "TELEKART_SHARED_KEY"
ENV_CAR_ID = "TELEKART_CAR_ID"
ENV_LOG_LEVEL = "TELEKART_LOG_LEVEL"


# --------------------------------------------------------------------------
# Pin assignment (BCM numbering) -- how the board is wired, not a suggestion
# --------------------------------------------------------------------------

PIN_ENA = 12  # L298N enable A -> left motor PWM   (BCM PWM0)
PIN_IN1 = 5  # left direction A
PIN_IN2 = 6  # left direction B
PIN_IN3 = 20  # right direction A
PIN_IN4 = 21  # right direction B
PIN_ENB = 13  # L298N enable B -> right motor PWM  (BCM PWM1)

#: HS-311 signal. GPIO18 is *also* PWM0, the same channel as GPIO12, so hardware
#: PWM here is physically impossible while the motors are running. pigpio's
#: DMA-timed ``set_servo_pulsewidth`` is the only option, and it is a good one:
#: the jitter is a couple of microseconds.
PIN_SERVO = 18

PIN_ENC_L_A = 23
PIN_ENC_L_B = 24
PIN_ENC_R_A = 27
PIN_ENC_R_B = 22

PIN_STATUS_LED = 25
PIN_ESTOP_BUTTON = 16  # to ground, internal pull-up, so idle reads HIGH

#: Pins that must be driven to a known-safe level before anything else happens.
#: IN1/IN2 sit on GPIO5/6, which the SoC pulls *up* at boot -- with EN low that
#: is harmless (outputs are high-Z), but the moment EN goes high it is a brake.
#: External 10k pull-downs are the real fix; this list is the software half.
MOTOR_SAFE_LOW_PINS: tuple[int, ...] = (PIN_ENA, PIN_ENB, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4)

#: BCM283x hardware PWM channel map. Both channels share one clock divider,
#: which is why ``GpioBackend.set_pwm_pair`` exists instead of two setters.
HW_PWM_CHANNEL_0_PINS = frozenset({12, 18})
HW_PWM_CHANNEL_1_PINS = frozenset({13, 19})


# --------------------------------------------------------------------------
# Timing budgets
# --------------------------------------------------------------------------

CONTROL_PERIOD_S = 1.0 / CONTROL_LOOP_HZ  # 10 ms
CONTROL_TX_PERIOD_S = 1.0 / CONTROL_RATE_HZ
TELEMETRY_PERIOD_S = 1.0 / TELEMETRY_RATE_HZ  # 20 ms

#: How much of the 10 ms period the loop body may consume. Anything above this
#: and the Pi has no headroom left for the WiFi stack, which shows up as control
#: packet loss long before it shows up as a missed deadline.
LOOP_BUDGET_S = 0.0060

#: p99 above this is a failing loop regardless of the mean. A 10 ms loop whose
#: p99 is 12 ms is dropping a command every hundred ticks.
LOOP_P99_BUDGET_S = 0.0120

#: Consecutive overruns before Fault.LOOP_OVERRUN is raised. One overrun is a
#: scheduling hiccup; twenty in a row means the loop genuinely does not fit.
LOOP_OVERRUN_FAULT_COUNT = 20

#: Cadence of the things that are not the control loop.
WATCHDOG_PERIOD_S = 0.100
STATUS_LED_PERIOD_S = 0.050
HEALTH_POLL_PERIOD_S = 1.000  # vcgencmd is a subprocess; keep it well away from 100 Hz
GC_COLLECT_PERIOD_S = 30.0

#: Encoder edges are filtered in pigpio's notification thread. 30 us rejects
#: contact bounce and ringing without touching real edges: at the 166 RPM
#: ceiling and 660 cpr the shortest genuine gap is ~550 us.
ENCODER_GLITCH_US = 30

#: No edge for this long means the wheel is stopped rather than merely slow.
#: 250 ms at 660 cpr corresponds to under 0.4 RPM, which is below anything the
#: drivetrain can actually hold.
ENCODER_STALE_WINDOW_S = 0.250


# --------------------------------------------------------------------------
# Real-time scheduling
# --------------------------------------------------------------------------

#: SCHED_FIFO priority for the control thread. Deliberately below 80, where the
#: kernel's own IRQ threads live -- outranking the network IRQ thread would
#: starve the very packets the loop is waiting for.
RT_PRIORITY_CONTROL = 50
RT_PRIORITY_NET = 20

#: The Pi Zero 2 W has four A53 cores. Pinning the control loop to the last one
#: keeps it off core 0, where most kernel work lands by default.
CONTROL_CPU_AFFINITY: tuple[int, ...] = (3,)

#: Default GIL switch interval while the control thread is running. Shorter than
#: CPython's 5 ms default so the RT thread reacquires the GIL promptly after a
#: networking thread takes it; the cost is more context switches, which on this
#: workload is a trade worth making.
GIL_SWITCH_INTERVAL_S = 0.001

#: /proc/<pid>/oom_score_adj. The control process must be the last thing the
#: kernel kills; the camera process must be among the first.
OOM_SCORE_CONTROL = -900
OOM_SCORE_VIDEO = 500


# --------------------------------------------------------------------------
# Power train -- measured or datasheet figures for the fitted hardware
# --------------------------------------------------------------------------

#: Pololu S18V20ALV, adjustable output, set to 9 V by the user.
RAIL_NOMINAL_V = 9.0

#: 6-cell NiMH pack.
PACK_NOMINAL_V = 7.2
PACK_CELLS = 6

#: What the boost converter will actually sustain for BOTH motors combined at
#: this input/output ratio. The nameplate says more; the pack's internal
#: resistance says otherwise. Simultaneous demand is what trips it, which is
#: exactly why the firmware budgets the *sum* of the two duties.
REGULATOR_SUSTAINED_A = 1.5
#: Above this for REGULATOR_TRIP_DELAY_S the converter drops into hiccup mode.
REGULATOR_TRIP_A = 2.4
REGULATOR_TRIP_DELAY_S = 0.060
REGULATOR_RECOVERY_S = 0.350

#: L298N is a Darlington bridge, not a MOSFET one: roughly 1.4 V of fixed drop
#: plus a bit of slope. That drop is the single biggest reason this car is slow,
#: and it is also where the ~0.22 duty deadband comes from -- below it the
#: bridge simply does not conduct.
BRIDGE_DROP_V = 1.4
BRIDGE_OHMS = 0.5

#: GA37-520, 12 V, 30:1 gearbox. 360 RPM is the *nameplate* free-running figure
#: at 12 V with an ideal supply. On a 9 V rail through a Darlington bridge with
#: 1.5 A to share, expect 150-200 RPM. Nothing in the firmware may assume either
#: number; everything scales off the measured value in calibration.yaml.
MOTOR_GEAR_RATIO = 30.0
MOTOR_NAMEPLATE_RPM_12V = 360.0
MOTOR_WINDING_OHMS = 3.0

#: Back-EMF constant referred to the *output* shaft, V*s/rad. Numerically equal
#: to the torque constant in N*m/A, as it must be for any DC machine.
MOTOR_KE_V_S_PER_RAD = 0.318
MOTOR_KT_NM_PER_A = 0.318

#: Encoder counts per revolution of the output shaft under x2 decoding:
#: 11 pulses per motor revolution * 2 edges * 30:1 gearbox.
DEFAULT_ENCODER_CPR = 660


# --------------------------------------------------------------------------
# Servo
# --------------------------------------------------------------------------

#: Absolute pulse limits. Wider than any sane calibration; these exist only to
#: stop a corrupt parameter driving the HS-311 into its end stops, where it
#: stalls and draws current from the Pi's own 5 V rail.
SERVO_ABS_MIN_US = 500
SERVO_ABS_MAX_US = 2500
SERVO_NOMINAL_CENTER_US = 1500

#: HS-311 is quoted at 0.19 s per 60 degrees at 6 V. Slower here on 5 V.
SERVO_SLEW_US_PER_S = 2100.0


# --------------------------------------------------------------------------
# Small conversions used across several modules
# --------------------------------------------------------------------------


def wheel_circumference_m(wheel_diameter_m: float) -> float:
    return math.pi * wheel_diameter_m


def rpm_to_mps(rpm: float, wheel_diameter_m: float) -> float:
    return rpm * math.pi * wheel_diameter_m / 60.0


def mps_to_rpm(mps: float, wheel_diameter_m: float) -> float:
    if wheel_diameter_m <= 0.0:
        return 0.0
    return mps * 60.0 / (math.pi * wheel_diameter_m)


def rpm_to_rad_s(rpm: float) -> float:
    return rpm * math.tau / 60.0


def rad_s_to_rpm(rad_s: float) -> float:
    return rad_s * 60.0 / math.tau


def counts_to_metres(counts: int, cpr: int, wheel_diameter_m: float) -> float:
    if cpr <= 0:
        return 0.0
    return (counts / cpr) * math.pi * wheel_diameter_m
