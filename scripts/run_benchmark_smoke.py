#!/usr/bin/env python3
"""Run a Google Benchmark binary as a smoke check that fails on an empty filter.

Google Benchmark exits with status 0 even when ``--benchmark_filter`` matches no
benchmarks; it only prints ``Failed to match any benchmarks against regex`` to
stderr. A smoke invocation therefore passes silently once a benchmark is
renamed, removed, or reparametrized, hiding the fact that the targeted component
is no longer exercised.

This wrapper first lists the benchmarks the filter selects and fails when that
set is empty, then runs the benchmark for real and propagates its exit code.
That keeps the CUDA (and any other) benchmark smoke honest: if the filter stops
matching, the smoke step fails loudly instead of reporting a green pass over
zero benchmarks.

The enumeration step captures the binary's output to count matches. When the
binary instead fails to enumerate at all (a missing executable, or a startup
crash such as a CUDA-init failure during static benchmark registration), the
wrapper re-emits the binary's own stdout/stderr and exits non-zero with an
actionable message, so the real diagnostic still reaches the log rather than
being swallowed behind a Python traceback.

Usage:
    python scripts/run_benchmark_smoke.py <binary> [benchmark args...]

Example:
    python scripts/run_benchmark_smoke.py build/.../bin/bm_vbd_cuda \\
        --benchmark_filter='BM_Vbd(Cpu|Cuda)Step/32$' --benchmark_min_time=0.001s
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from typing import List, Sequence


class BenchmarkEnumerationError(RuntimeError):
    """Raised when a benchmark binary cannot be enumerated for matches."""


def count_matching_benchmarks(binary: str, run_args: Sequence[str]) -> int:
    """Return how many benchmarks ``run_args`` select in ``binary``.

    ``--benchmark_list_tests=true`` prints one matching benchmark name per line
    to stdout and ignores run-only flags such as ``--benchmark_min_time``. When
    nothing matches it writes a diagnostic to stderr and leaves stdout empty, so
    counting non-blank stdout lines yields the match count.

    Raises ``BenchmarkEnumerationError`` when the binary is missing or exits
    non-zero while enumerating, after re-emitting whatever the binary printed so
    its own diagnostic is not lost.
    """
    try:
        result = subprocess.run(
            [binary, "--benchmark_list_tests=true", *run_args],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise BenchmarkEnumerationError(
            f"benchmark binary not found: {binary}"
        ) from exc

    if result.returncode != 0:
        # Surface the binary's own output (e.g. a CUDA startup error) instead of
        # letting capture_output swallow it behind a bare traceback.
        if result.stdout:
            sys.stdout.write(result.stdout)
        if result.stderr:
            sys.stderr.write(result.stderr)
        raise BenchmarkEnumerationError(
            f"benchmark binary '{binary}' exited {result.returncode} while "
            "enumerating benchmarks with --benchmark_list_tests=true"
        )

    return sum(1 for line in result.stdout.splitlines() if line.strip())


def _describe_filter(run_args: Sequence[str]) -> str:
    for arg in run_args:
        if arg.startswith("--benchmark_filter="):
            return arg[len("--benchmark_filter=") :]
    return "<no --benchmark_filter given>"


def run_smoke(binary: str, run_args: Sequence[str]) -> int:
    """List-then-run ``binary``; fail when the filter selects no benchmarks."""
    try:
        matched = count_matching_benchmarks(binary, run_args)
    except BenchmarkEnumerationError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    if matched == 0:
        print(
            f"ERROR: benchmark filter '{_describe_filter(run_args)}' matched no "
            f"benchmarks in {binary}. The smoke target no longer exercises any "
            "benchmark; update the filter or the benchmark registration.",
            file=sys.stderr,
        )
        return 1

    try:
        completed = subprocess.run([binary, *run_args], check=False)
    except FileNotFoundError:
        print(f"ERROR: benchmark binary not found: {binary}", file=sys.stderr)
        return 1
    return completed.returncode


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a Google Benchmark smoke check that fails on an empty filter.",
    )
    parser.add_argument("binary", help="Path to the Google Benchmark executable.")
    parser.add_argument(
        "benchmark_args",
        nargs=argparse.REMAINDER,
        help="Arguments forwarded to the benchmark (e.g. --benchmark_filter=...).",
    )
    return parser.parse_args(list(argv))


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    run_args: List[str] = [arg for arg in args.benchmark_args if arg != "--"]
    return run_smoke(args.binary, run_args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
