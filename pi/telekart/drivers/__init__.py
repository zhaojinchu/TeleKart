"""Hardware drivers: the layer between a duty cycle and a spinning wheel.

Nothing here is re-exported eagerly beyond the three driver classes and their
value types. Each driver owns exactly the pins it was given, applies the
protections that belong to it -- combined-duty budget and dead-time sequencing
for the bridge, slew and pulse clamps for the servo -- and refuses to have an
opinion about anything above it. The control loop composes them; they never
call back into it.
"""

from __future__ import annotations

from .encoder import EncoderSample, MTVelocity, QuadratureEncoder
from .motor import MotorPair
from .servo import SteeringServo

__all__ = [
    "MotorPair",
    "SteeringServo",
    "QuadratureEncoder",
    "EncoderSample",
    "MTVelocity",
]
