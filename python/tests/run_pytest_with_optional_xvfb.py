"""Run pytest, using xvfb-run for Linux headless GUI tests when available."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys


def _has_linux_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def main() -> int:
    command = [sys.executable, "-m", "pytest", *sys.argv[1:]]

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
    return subprocess.call(command)


if __name__ == "__main__":
    raise SystemExit(main())
