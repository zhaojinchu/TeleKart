"""The five screens the navigation rail switches between.

Nothing is re-exported. ``diagnostics`` pulls in pyqtgraph and ``drive`` pulls
in the HUD and the video surface; making this package import them all would mean
that opening the About box constructs a plotting library. The main window
imports the concrete modules it needs, when it needs them.

Every screen follows the same three rules, and they are what keep five
independently-written surfaces feeling like one application:

* A screen reads :class:`~telekart_app.model.app_model.AppModel` and emits
  intent. It never opens a socket, never touches a thread, and never decides
  that the car is armed -- it renders the state the car reported.
* A screen owns no persistent state that the model or the settings file could
  own instead. Reconstructing one must produce the same picture.
* Layout margins and spacings come from ``theme.space``. A screen that invents
  its own 10 px gutter is visibly out of step the moment it sits next to
  another one.
"""

from __future__ import annotations

__all__: list[str] = []
