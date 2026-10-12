#!/usr/bin/env python3
"""Run repository pytest gates without ambient control or plugin injection."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import tempfile
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


def _write(text: str) -> None:
    # Replay text that the parent's output encoding may not represent.
    encoding = sys.stdout.encoding or "utf-8"
    sys.stdout.write(text.encode(encoding, "replace").decode(encoding))
    sys.stdout.flush()


def _stop(child: subprocess.Popen) -> None:
    """Interrupt a child runner and its workers; kill them if they hang."""
    if child.poll() is None:
        if os.name == "posix":
            # Its own session lets the interrupt reach spawned workers too.
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
        else:
            subprocess.run(
                ["taskkill", "/T", "/F", "/PID", str(child.pid)], capture_output=True
            )
    child.wait()


def main(arguments: Sequence[str] | None = None) -> int:
    if not sys.flags.isolated:
        print("ERROR: invoke the pytest runner with Python -I", file=sys.stderr)
        return 2

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--pythonpath", action="append", default=[])
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--expect-node")
    selection.add_argument("--concurrent-node")
    options, pytest_arguments = parser.parse_known_args(
        arguments if arguments is not None else sys.argv[1:]
    )
    _clear_pytest_environment()

    import pytest

    if options.concurrent_node:
        pytest_arguments.extend(["--deselect", options.concurrent_node])

    class _SelectionGuard:
        error = None
        child = None

        @pytest.hookimpl(hookwrapper=True, tryfirst=True)
        def pytest_collection_modifyitems(self, config, items):
            original = [item.nodeid for item in items]
            deselected = config.getoption("deselect") or []
            yield
            selected = [item.nodeid for item in items]
            if options.concurrent_node and deselected != [options.concurrent_node]:
                self.error = "concurrent selection must deselect only its child node"
            elif options.concurrent_node and (
                len(original) != len(set(original))
                or original.count(options.concurrent_node) != 1
            ):
                self.error = "deselection must name each collected node exactly once"
            elif options.concurrent_node and selected != [
                node for node in original if node != options.concurrent_node
            ]:
                self.error = "selection is not the exact deselection complement"
            elif options.expect_node and (
                original != [options.expect_node] or selected != original
            ):
                self.error = (
                    f"expected exactly one collected node: {options.expect_node}"
                )

        def pytest_collection_finish(self, session):
            if self.error:
                raise pytest.UsageError(self.error)
            if options.concurrent_node:
                self.child = subprocess.Popen(
                    [
                        sys.executable,
                        "-I",
                        "-X",
                        "utf8",
                        str(Path(__file__).resolve()),
                        *[
                            argument
                            for entry in options.pythonpath
                            for argument in ("--pythonpath", entry)
                        ],
                        str(session.config.rootpath / options.concurrent_node),
                        "--rootdir",
                        str(session.config.rootpath),
                        "--expect-node",
                        options.concurrent_node,
                        "--basetemp",
                        str(Path(child_directory) / "pytest"),
                        "-v",
                    ],
                    stdout=child_output,
                    stderr=subprocess.STDOUT,
                    **(
                        {"start_new_session": True}
                        if os.name == "posix"
                        else {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
                    ),
                )

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
    selection_guard = _SelectionGuard()
    with (
        tempfile.TemporaryDirectory(prefix="dart-pytest-") as child_directory,
        tempfile.TemporaryFile() as child_output,
    ):
        try:
            result = pytest.main(
                [
                    "-c",
                    str(ROOT / "pyproject.toml"),
                    "--noconftest",
                    *pytest_arguments,
                ],
                plugins=[guard, selection_guard],
            )
            if result == pytest.ExitCode.OK and guard.executed == 0:
                print(
                    "ERROR: pytest completed without executing a test body",
                    file=sys.stderr,
                )
                result = pytest.ExitCode.TESTS_FAILED
            child = selection_guard.child
            child_result = 0
            if child:
                if result == pytest.ExitCode.INTERRUPTED:
                    _stop(child)
                child_result = child.wait()
                child_output.seek(0)
                _write(f"\n=== Concurrent pytest: {options.concurrent_node} ===\n")
                _write(child_output.read().decode("utf-8", errors="replace"))
            return int(result) or child_result
        finally:
            # A second interrupt can escape wait(); never leave the child running.
            if selection_guard.child:
                _stop(selection_guard.child)


if __name__ == "__main__":
    raise SystemExit(main())
