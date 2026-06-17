"""Representative real-render smoke for Python demo scenes.

Runs a small scene subset through the crash-isolated smoke harness in render
mode, which invokes the real Filament viewer headlessly and validates that a
nonblank screenshot is produced. The full-catalog in-process smoke remains in
``test_demos_full_catalog_smoke.py``; this keeps TIER-3 render coverage bounded.
"""

from __future__ import annotations

import pathlib
import subprocess
import sys

# Put python/ on sys.path so the demos package is importable.
_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
_REPO_ROOT = _PYTHON_DIR.parent
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

import pytest


def _viewer_available() -> bool:
    try:
        import dartpy
    except Exception:  # pragma: no cover - dartpy import failure
        return False
    return hasattr(dartpy, "gui") and hasattr(dartpy.gui, "run_demos")


def test_representative_scenes_render_headlessly() -> None:
    if not _viewer_available():
        pytest.skip("dartpy.gui.run_demos unavailable in this build")

    subset = [
        "rigid_body",
        "contact",
        "avbd_demo3d_stack",
        "rigid_ipc_pile",
        "vbd_cloth",
        "ipc_deformable_drape",
    ]
    cmd = [
        sys.executable,
        "scripts/py_demos_smoke.py",
        "--render",
        "--only",
        *subset,
        "--frames",
        "2",
        "--timeout",
        "120",
        "--width",
        "320",
        "--height",
        "240",
    ]
    proc = subprocess.run(
        cmd,
        cwd=_REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    failure = f"demo render smoke failed with exit {proc.returncode}\n{proc.stdout}"
    assert proc.returncode == 0, failure
