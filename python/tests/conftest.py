"""
pytest configuration helpers.

Pytests are executed from within ``python/tests`` which means Python discovers
the source tree (`python/dartpy`) before the freshly built extension under
``build/<env>/cpp/<config>/python``.  Those source packages do not contain the
compiled ``dartpy`` extension, so importing ``dartpy`` during CI fails with
``ModuleNotFoundError: dartpy.dartpy`` even though the extension exists in the
build directory.

To make sure we always import the just-built bindings, re-insert the build
directories specified via ``PYTHONPATH`` at the front of ``sys.path`` before
pytest collects any modules.
"""

from __future__ import annotations

import os
import sys


def _prepend_pythonpath() -> None:
    """Ensure PYTHONPATH entries take precedence on sys.path."""
    extra = os.environ.get("PYTHONPATH")
    if not extra:
        return

    # Split according to platform semantics and add in reverse order so the
    # left-most entry from PYTHONPATH ends up at the front of sys.path.
    paths = [p for p in extra.split(os.pathsep) if p]
    for entry in reversed(paths):
        if entry in sys.path:
            sys.path.remove(entry)
        sys.path.insert(0, entry)


_prepend_pythonpath()
