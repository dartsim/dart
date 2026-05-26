"""Runpy entry for ``pixi run py-ex demos``.

The pixi ``py-ex`` task ``runpy``-loads ``python/examples/<name>/main.py``
directly, so this file gets executed without package context. It puts the
``python/`` directory on ``sys.path`` and then defers to the
``examples.demos`` package's runner (whose scenes use package-relative
imports).
"""

from __future__ import annotations

import pathlib
import sys

_HERE = pathlib.Path(__file__).resolve()
# _HERE = python/examples/demos/main.py; parents[2] = python/
sys.path.insert(0, str(_HERE.parents[2]))

from examples.demos.registry import make_demo_scenes  # noqa: E402
from examples.demos.runner import run  # noqa: E402

if __name__ == "__main__":
    sys.exit(run(sys.argv[1:], make_demo_scenes()))
