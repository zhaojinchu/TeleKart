"""``python -m telekart_video``.

The implementation lives in `telekart_video.app` because that is what the
console script and the systemd unit name, and because a module that is both
``__main__`` and an importable submodule gets executed twice with two copies of
its module-level state.
"""

from __future__ import annotations

from .app import main

if __name__ == "__main__":
    raise SystemExit(main())
