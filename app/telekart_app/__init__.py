"""TeleKart desktop driving station.

Deliberately free of Qt, PyAV and SDL imports: ``import telekart_app`` must stay
cheap enough that headless integration tests and the CLI can pull in
``telekart_app.net`` without dragging a GUI toolkit into the process.
"""

from __future__ import annotations

__version__ = "2.0.0"

#: Used for the window title, the platformdirs application name, and the
#: ``app_version`` field of the session handshake. One constant so a rename
#: cannot leave the car logging a different name than the title bar shows.
APP_NAME = "TeleKart"

#: QSettings/QCoreApplication identity. Nothing in this package writes through
#: QSettings -- ``config.settings`` owns persistence -- but Qt itself uses these
#: for native dialogs and crash reports.
ORG_NAME = "TeleKart"
ORG_DOMAIN = "telekart.local"

__all__ = ["__version__", "APP_NAME", "ORG_NAME", "ORG_DOMAIN"]
