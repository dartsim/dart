"""Run pytest with stable Linux OpenGL defaults for GUI tests."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys


def _has_linux_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _pytest_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    if sys.platform.startswith("linux"):
        env.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
        env.setdefault("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe")
    return env


def _pytest_arguments() -> list[str]:
    extra_args = shlex.split(os.environ.get("DARTPY_PYTEST_ARGS", ""))
    source_override = os.environ.get("DARTPY_PYTEST_SOURCES")
    sources = shlex.split(source_override) if source_override else sys.argv[1:]
    return [*extra_args, *sources]


def main() -> int:
    command = [sys.executable, "-m", "pytest", *_pytest_arguments()]
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
