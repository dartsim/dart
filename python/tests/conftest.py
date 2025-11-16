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
pytest collects any modules.  When ``PYTHONPATH`` is missing (or was stripped
by the calling environment), we infer the expected build tree from the repo
layout so that `dartpy` can still be imported directly from CI artifacts.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _candidate_python_paths() -> list[str]:
    """
    Collect python search paths that should take precedence.

    We start with PYTHONPATH entries (if provided) and then fall back to the
    conventional pixi build directory so CI always sees the freshly built wheel.
    """

    entries: list[str] = []
    seen: set[str] = set()

    def _add(path: str) -> None:
        normalized = os.path.normpath(path)
        if normalized in seen:
            return
        if not os.path.isdir(normalized):
            return
        seen.add(normalized)
        entries.append(normalized)

    extra = os.environ.get("PYTHONPATH")
    if extra:
        for candidate in extra.split(os.pathsep):
            candidate = candidate.strip()
            if candidate:
                _add(candidate)

    # Fall back to the pixi build layout if no explicit PYTHONPATH is provided
    # (or if it missed the dartpy directories).
    repo_root = Path(__file__).resolve().parents[2]
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_type = (
        os.environ.get("BUILD_TYPE")
        or os.environ.get("CMAKE_BUILD_TYPE")
        or "Release"
    )
    build_python = repo_root / "build" / pixi_env / "cpp" / build_type / "python"
    _add(str(build_python))
    _add(str(build_python / "dartpy"))

    return entries


def _prepend_pythonpath() -> None:
    """Ensure PYTHONPATH entries take precedence on sys.path."""
    paths = _candidate_python_paths()
    if not paths:
        return

    # Split according to platform semantics and add in reverse order so the
    # left-most entry from PYTHONPATH ends up at the front of sys.path.
    for entry in reversed(paths):
        if entry in sys.path:
            sys.path.remove(entry)
        sys.path.insert(0, entry)

    # De-prioritize the source tree so compiled bindings are always used.
    source_python = Path(__file__).resolve().parents[1]
    source_str = str(source_python)
    if source_str in sys.path:
        sys.path.remove(source_str)
        sys.path.append(source_str)


_prepend_pythonpath()
