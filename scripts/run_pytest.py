#!/usr/bin/env python3
"""Run repository pytest gates without ambient control or plugin injection."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]


class _ExecutionGuard:
    """Record whether pytest reached at least one test call."""

    def __init__(self) -> None:
        self.executed = 0

    def pytest_runtest_call(self, item: object) -> None:
        del item
        self.executed += 1


def _clear_pytest_environment() -> None:
    for name in tuple(os.environ):
        if name.upper().startswith("PYTEST_"):
            os.environ.pop(name)
    os.environ["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"


def main(arguments: Sequence[str] | None = None) -> int:
    if not sys.flags.isolated:
        print("ERROR: invoke the pytest runner with Python -I", file=sys.stderr)
        return 2

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--pythonpath", action="append", default=[])
    parser.add_argument("--expect-node")
    options, pytest_arguments = parser.parse_known_args(
        arguments if arguments is not None else sys.argv[1:]
    )
    _clear_pytest_environment()

    import pytest

    class _SelectionGuard:
        error = None

        @pytest.hookimpl(hookwrapper=True, tryfirst=True)
        def pytest_collection_modifyitems(self, config, items):
            original = [item.nodeid for item in items]
            deselected = config.getoption("deselect") or []
            yield
            selected = [item.nodeid for item in items]
            if deselected and (
                len(original) != len(set(original))
                or any(original.count(node) != 1 for node in deselected)
            ):
                self.error = "deselection must name each collected node exactly once"
            elif deselected and selected != [
                node for node in original if node not in deselected
            ]:
                self.error = "selection is not the exact deselection complement"
            elif options.expect_node and selected != [options.expect_node]:
                self.error = (
                    f"expected exactly one collected node: {options.expect_node}"
                )

        def pytest_collection_finish(self):
            if self.error:
                raise pytest.UsageError(self.error)

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

    guard = _ExecutionGuard()
    result = pytest.main(
        [
            "-c",
            str(ROOT / "pyproject.toml"),
            "--noconftest",
            *pytest_arguments,
        ],
        plugins=[guard, _SelectionGuard()],
    )
    if result == pytest.ExitCode.OK and guard.executed == 0:
        print("ERROR: pytest completed without executing a test body", file=sys.stderr)
        return int(pytest.ExitCode.TESTS_FAILED)
    return int(result)


if __name__ == "__main__":
    raise SystemExit(main())
