"""Marks the firmware suite as a package.

Not decoration. With this file present, pytest's rootdir walk stops at ``pi/``
rather than ``pi/tests/``, so ``pi/`` is what lands on ``sys.path`` and
``import telekart`` resolves to the source tree being tested instead of to
whatever happens to be installed in the environment. Deleting it makes the
suite silently test the wrong copy of the firmware.
"""

from __future__ import annotations
