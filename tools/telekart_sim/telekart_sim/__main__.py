"""Entry point for `python -m telekart_sim` and the `telekart-sim` console script."""

from __future__ import annotations

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
