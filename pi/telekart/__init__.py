"""TeleKart vehicle firmware.

Two processes run on the car. ``telekart-control`` owns motion, safety and
networking; ``telekart-video`` owns the camera. They are separate processes and
not threads because the GIL is real: H.264 encoding must not be able to inject
jitter into the 100 Hz control loop, and the OOM policy has to be able to
guarantee that the kernel takes the camera and never the thing that is steering.

Nothing is re-exported here. Submodules import each other in a fixed order --
``constants`` then ``log`` then ``util`` then ``config`` then ``hal`` then the
drivers -- and a convenience re-export at the top would create a cycle the first
time someone imported the package from inside one of them.
"""

from __future__ import annotations

from .constants import FIRMWARE_VERSION

__version__ = FIRMWARE_VERSION

__all__ = ["__version__", "FIRMWARE_VERSION"]
