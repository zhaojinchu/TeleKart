"""Modal surfaces, of which this application has exactly two.

Everything else is a screen, a panel or an inline message, because a modal
dialog over a live video feed is a dialog covering a moving car. The two
exceptions earn it:

``calibration_wizard``
    Genuinely sequential and genuinely exclusive -- it asks the driver to move
    one control at a time and would produce garbage if anything else were
    reading the device meanwhile.

``about``
    Rare, small, and dismissed immediately.

Nothing is re-exported; the window imports the concrete module.
"""

from __future__ import annotations

__all__: list[str] = []
