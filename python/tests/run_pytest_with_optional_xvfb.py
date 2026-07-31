"""Run pytest with stable Linux OpenGL defaults for GUI tests."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GUARDED_RUNNER = ROOT / "scripts" / "run_pytest.py"
LEGACY_SELECTION_VARIABLES = {
    "DARTPY_PYTEST_ARGS",
    "DARTPY_PYTEST_SOURCES",
}


def _has_linux_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _pytest_env() -> dict[str, str]:
    env = os.environ.copy()
    for name in tuple(env):
        if (
            name.upper().startswith("PYTEST_")
            or name.upper() in LEGACY_SELECTION_VARIABLES
        ):
            env.pop(name)
    env.setdefault("PYTHONUNBUFFERED", "1")
    if sys.platform.startswith("linux"):
        env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
        env.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")
    return env


def main() -> int:
    command = [
        sys.executable,
        "-I",
        str(GUARDED_RUNNER),
        "--repository-conftest",
        *sys.argv[1:],
    ]
    env = _pytest_env()

    if sys.platform.startswith("linux") and not _has_linux_display():
        xvfb_run = shutil.which("xvfb-run")
        if xvfb_run:
            command = [
                xvfb_run,
                "--auto-servernum",
                "--server-args=-screen 0 1024x768x24",
                *command,
            ]

    print("Running pytest by:", " ".join(command), flush=True)
    return subprocess.call(command, env=env)


if __name__ == "__main__":
    raise SystemExit(main())
