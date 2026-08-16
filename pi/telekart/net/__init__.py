"""Everything on the car that talks to the operator.

Four channels, three of them here: UDP control in (:mod:`control_link`), UDP
telemetry out (:mod:`telemetry_tx`), TCP/JSON session (:mod:`session_server`),
plus mDNS advertisement (:mod:`discovery`). Video lives in its own process.

All of it runs on the asyncio thread. Nothing in this package touches GPIO, the
safety state machine, or the drive controller directly -- the seams are the
callbacks and Protocols each module declares, and ``telekart.app`` is the only
place that wires them to the control thread. That is what lets the whole network
stack be exercised against a fake vehicle with no hardware and no control loop.

Submodules are not re-exported. Importing ``telekart.net`` should not drag in an
asyncio server just because something wanted a constant.
"""

from __future__ import annotations

__all__: list[str] = []
