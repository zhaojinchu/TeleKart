"""Mac-side settings. Ports and timeouts come from the protocol, never redefined here."""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "common"))

import protocol  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
LOG_DIR = REPO / "logs"

#: Defaults to the **simulator on this Mac**, never the car.
#:
#: The Pi runs other software and is off-limits, so reaching it has to be a deliberate
#: act rather than what happens when you type ``make drive``. Sending 50 Hz of control
#: traffic at a vehicle you did not mean to address is exactly the accident this default
#: exists to prevent -- and a default that is only safe when you remember to override it
#: is not a safe default.
#:
#: Override, in precedence order::
#:
#:     python mac/main.py telekart.local     # argv wins
#:     TELEKART_HOST=telekart.local make drive
DEFAULT_HOST = "127.0.0.1"

HOST = os.environ.get("TELEKART_HOST", DEFAULT_HOST)
if len(sys.argv) > 1:
    HOST = sys.argv[1]

#: True when we are addressing something other than this machine, i.e. plausibly the real
#: car. main.py says so loudly at startup and on the HUD.
IS_REMOTE = HOST not in ("127.0.0.1", "localhost", "::1")

CONTROL_PORT = protocol.CONTROL_PORT
VIDEO_PORT = protocol.VIDEO_PORT
STREAM_URL = protocol.stream_url(HOST, VIDEO_PORT)

CMD_RATE_HZ = protocol.CMD_RATE_HZ
TLM_STALE_S = protocol.TLM_STALE_S

WINDOW_W, WINDOW_H = 1000, 600
VIDEO_W, VIDEO_H = 640, 480  # the pane; frames are scaled to fit
PANEL_X = VIDEO_W + 16

# --- Wheel / gamepad ----------------------------------------------------------
# Measured with mac/calibrate_input.py on 2026-08-16 against a cheap USB wheel
# ("Wired Wheel", 0x0079:0x189c) read over raw USB -- see mac/xinput.py for why.
#
# The wheel has two personalities. In Switch-emulation mode it appears as HID and pygame
# can read it, but the pedals are reduced to plain buttons: no travel, no proportional
# throttle. In XInput mode it is invisible to HID entirely, and the pedals come through
# as trigger bytes. Measured there: steering 159 distinct values across the full sweep,
# throttle 78, brake 72 -- all genuinely analog. That is the mode worth using, and it is
# why this file maps XInput controls rather than pygame axis indices.

USE_WHEEL = True  # fall back to WASD when no device is present

#: Steering is the left stick X axis, already negative-left, matching our convention.
WHEEL_STEER_INVERT = False
#: Cheap potentiometers wander a little around centre; without this the servo hunts.
WHEEL_STEER_DEADZONE = 0.06
#: Pedals rest at exactly 0 and were measured to 0.99, so only noise needs rejecting.
WHEEL_PEDAL_DEADZONE = 0.02

# --- Paddle shifters ----------------------------------------------------------
# The wheel has one throttle pedal, so direction comes from a gear rather than a second
# pedal: left paddle selects R, right paddle selects D, and the throttle pedal then
# drives whichever way the gear points. Standard wired-XInput bit numbers -- confirm
# with mac/calibrate_input.py if the paddles turn out to sit elsewhere.
WHEEL_GEAR_REVERSE_BIT = 8  # LB / left paddle
WHEEL_GEAR_DRIVE_BIT = 9  # RB / right paddle

#: Refuse to change gear while the throttle is down. Lifting to shift is what you would
#: do anyway, and it keeps a full-throttle direction flip -- the harshest thing you can
#: ask of the bridge -- from being one twitch away at all times. The Pi's reversal guard
#: would survive it; this means it never gets asked to.
GEAR_CHANGE_MAX_THROTTLE = 0.05

# Key bindings, in one place so the HUD legend and the input code cannot disagree.
KEYS = {
    "throttle_fwd": "W",
    "throttle_rev": "S",
    "steer_left": "A",
    "steer_right": "D",
    "brake": "SPACE",
    "arm_toggle": "ENTER",
    "quit": "ESC",
}
