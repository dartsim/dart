#!/usr/bin/env python3
"""Wrapper around pytest that dumps macOS crash logs when the test run aborts."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Iterable, List


def _collect_crash_logs(start_ts: float) -> List[Path]:
    """Return crash logs newer than the test start timestamp."""
    if sys.platform != "darwin":
        return []

    crash_dir = Path.home() / "Library" / "Logs" / "DiagnosticReports"
    if not crash_dir.exists():
        return []

    crash_files: List[Path] = []
    for pattern in ("python*.crash", "python*.ips"):
        for path in crash_dir.glob(pattern):
            try:
                if path.stat().st_mtime >= start_ts:
                    crash_files.append(path)
            except OSError:
                continue
    crash_files.sort(key=lambda p: p.stat().st_mtime)
    return crash_files


def _print_crash_log(path: Path) -> None:
    print("::group::macOS crash log:", path)
    try:
        with path.open("r", errors="replace") as handle:
            for line in handle.readlines()[-400:]:
                print(line.rstrip("\n"))
    except OSError as exc:
        print(f"Failed to read crash log {path}: {exc}")
    finally:
        print("::endgroup::")

def _run_lldb_import() -> None:
    if sys.platform != "darwin":
        return
    if not shutil.which("lldb"):
        print("lldb is not available; skipping extra diagnostics.")
        return
    print("::group::lldb dartpy import backtrace")
    cmd = [
        "lldb",
        "--batch",
        "-o",
        "run",
        "-o",
        "bt",
        "-o",
        "thread backtrace all",
        "--",
        sys.executable,
        "-c",
        "import dartpy",
    ]
    try:
        subprocess.run(cmd, check=False)
    finally:
        print("::endgroup::")


def main(argv: Iterable[str]) -> int:
    parser = argparse.ArgumentParser(description="Run pytest and dump macOS crash logs on failure.")
    parser.add_argument("pytest_args", nargs=argparse.REMAINDER, help="Arguments forwarded to pytest")
    args = parser.parse_args(list(argv))

    # argparse keeps the separating "--" in pytest_args; strip it if present.
    pytest_args = args.pytest_args
    if pytest_args and pytest_args[0] == "--":
        pytest_args = pytest_args[1:]

    start_timestamp = time.time()
    cmd = [sys.executable, "-m", "pytest", *pytest_args]
    completed = subprocess.run(cmd)
    exit_code = completed.returncode

    if exit_code != 0:
        crash_logs = _collect_crash_logs(start_timestamp)
        if crash_logs:
            _print_crash_log(crash_logs[-1])
        else:
            print(
                "No macOS crash logs were generated under ~/Library/Logs/DiagnosticReports.",
                file=sys.stderr,
            )
            crash_dir = Path.home() / "Library" / "Logs" / "DiagnosticReports"
            if crash_dir.exists():
                candidates = sorted(crash_dir.glob("python*"), key=lambda p: p.stat().st_mtime)[
                    -3:
                ]
                if candidates:
                    print("Recent diagnostic files:", file=sys.stderr)
                    for cand in candidates:
                        try:
                            ts = time.strftime(
                                "%Y-%m-%d %H:%M:%S", time.localtime(cand.stat().st_mtime)
                            )
                        except OSError:
                            ts = "unknown"
                        print(f"  {cand.name} (modified {ts})", file=sys.stderr)
        _run_lldb_import()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
