"""Entry point for ``python -m examples.demos``."""

from __future__ import annotations

import sys

from .registry import make_demo_scenes
from .runner import run

if __name__ == "__main__":
    sys.exit(run(sys.argv[1:], make_demo_scenes()))
