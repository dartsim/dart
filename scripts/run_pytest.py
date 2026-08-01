#!/usr/bin/env python3
"""Run repository pytest gates without ambient selection or plugin injection."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
LEGACY_SELECTION_VARIABLES = {
    "DARTPY_PYTEST_ARGS",
    "DARTPY_PYTEST_SOURCES",
}


class _ExecutionGuard:
    """Record whether pytest reached at least one test call."""

    def __init__(self) -> None:
        self.executed = 0

    def pytest_runtest_call(self, item: object) -> None:
        del item
        self.executed += 1


def _clear_pytest_environment() -> None:
    for name in tuple(os.environ):
        if (
            name.upper().startswith("PYTEST_")
            or name.upper() in LEGACY_SELECTION_VARIABLES
        ):
            os.environ.pop(name)
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"


def main(arguments: Sequence[str] | None = None) -> int:
    if not sys.flags.isolated:
        print("ERROR: invoke the pytest runner with Python -I", file=sys.stderr)
        return 2

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--pythonpath", action="append", default=[])
    parser.add_argument(
        "--repository-conftest",
        action="store_true",
        help="load only conftest files below the repository root",
    )
    options, pytest_arguments = parser.parse_known_args(
        arguments if arguments is not None else sys.argv[1:]
    )
    _clear_pytest_environment()

    # Import the trusted pytest installation before adding build/source paths.
    import pytest

    pythonpath_entries = [
        entry
        for value in options.pythonpath
        for entry in value.split(os.pathsep)
        if entry
    ]
    sys.path[:0] = pythonpath_entries
    if pythonpath_entries:
        os.environ["PYTHONPATH"] = os.pathsep.join(pythonpath_entries)
    else:
        os.environ.pop("PYTHONPATH", None)

    conftest_arguments = (
        ["--confcutdir", str(ROOT)] if options.repository_conftest else ["--noconftest"]
    )
    guard = _ExecutionGuard()
    result = pytest.main(
        [
            "-c",
            str(ROOT / "pyproject.toml"),
            "--rootdir",
            str(ROOT),
            *conftest_arguments,
            *pytest_arguments,
        ],
        plugins=[guard],
    )
    if result == pytest.ExitCode.OK and guard.executed == 0:
        print("ERROR: pytest completed without executing a test body", file=sys.stderr)
        return int(pytest.ExitCode.TESTS_FAILED)
    return int(result)


if __name__ == "__main__":
    raise SystemExit(main())
