"""``python -m telekart`` -- the control process.

The systemd unit invokes the package rather than ``telekart.app`` directly, so
the entry point stays stable if the module is ever split. It is also what makes
``python -m telekart --backend mock --duration 5`` a complete smoke test of the
whole stack on a laptop, with no hardware and no daemon.
"""

from __future__ import annotations

import sys

from .app import main

if __name__ == "__main__":
    sys.exit(main())
