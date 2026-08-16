"""Presentation layer.

Layering, and it is enforced by import direction rather than by convention:

``theme``
    Colours, spacing, type, icons, stylesheet. Depends on nothing but Qt.
``widgets``
    Self-contained instruments and containers. Depend on ``theme`` and on
    values passed to them -- never on the model, the network, or the protocol
    package. That is what makes every one of them constructible in a gallery
    with made-up numbers, and what keeps a paint bug from needing a car to
    reproduce.
``hud`` / ``screens`` / ``dialogs``
    Compose the above and subscribe to ``AppModel``. They are the only layer
    that knows a car exists.

Nothing is re-exported here. A screen imports ``telekart_app.ui.widgets``
directly, so that importing one screen does not construct a font database as a
side effect of touching this package.
"""

from __future__ import annotations

__all__: list[str] = []
