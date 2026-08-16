"""The window and everything painted in it.

Layering, which the modules are written against: ``theme`` <- ``hud`` <-
``video_view`` <- ``window``, with ``connect_panel`` off to one side depending
only on ``theme``. Nothing here reaches back into ``net`` or ``video``; the
window talks to ``AppModel`` and to nothing else.
"""

from __future__ import annotations

__all__: list[str] = []
