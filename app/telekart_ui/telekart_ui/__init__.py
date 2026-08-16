"""TeleKart driving station.

One window: full-bleed video with a HUD over it, driven from a wheel or from
WASD. Everything that was a screen in the previous station -- parameters,
settings, diagnostics, calibration, lap timing -- is gone. Parameters live in
``pi/config/config.local.yaml`` where the measured ones already came from, and
calibration is run on the car by ``pi/scripts/``.

Deliberately free of Qt, PyAV and SDL imports: ``import telekart_ui`` must stay
cheap enough that a test can pull in ``telekart_ui.net`` without dragging a GUI
toolkit into the process.
"""

from __future__ import annotations

__version__ = "1.0.0"

#: Used for the window title, the platformdirs application name, and the
#: ``app_version`` field of the session handshake. One constant so a rename
#: cannot leave the car logging a different name than the title bar shows.
APP_NAME = "TeleKart"

#: QCoreApplication identity. Nothing here writes through QSettings -- ``config``
#: owns persistence -- but Qt itself uses these for native dialogs.
ORG_NAME = "TeleKart"
ORG_DOMAIN = "telekart.local"

__all__ = ["__version__", "APP_NAME", "ORG_NAME", "ORG_DOMAIN"]
