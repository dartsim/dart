#!/usr/bin/env python3
"""Idempotently generate DART AI adapters and install the managed git hook."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def setup_commands() -> tuple[list[str], ...]:
    return (
        [sys.executable, "scripts/sync_ai_commands.py"],
        [sys.executable, "scripts/install_git_hooks.py"],
    )


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    for command in setup_commands():
        result = subprocess.run(command, cwd=root)
        if result.returncode:
            return result.returncode
    print("DART AI setup complete; run the doctor command to verify discovery")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
