#!/usr/bin/env python3
"""Check allocator benchmark results for performance regressions.

Runs bm_allocators (or reads a pre-existing JSON), applies per-category rules
to detect regressions, and exits non-zero if any check fails.

Rules encode the expected performance characteristics of each allocator:
  - Frame: always faster than Std (bump allocator)
  - Pool: faster than Std for batch workloads (count >= 16)
  - FreeList: competitive with Std for batch workloads (count >= 64)
  - RealisticWorkload: DART allocators faster than Std

Pool/FreeList have higher constant overhead than system malloc for single
operations — this is expected and NOT flagged as a regression.

Usage:
    python scripts/check_allocator_benchmarks.py
    python scripts/check_allocator_benchmarks.py --input .benchmark_results/allocators.json
    python scripts/check_allocator_benchmarks.py --verbose
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}

_ALLOC_RE = re.compile(r"BM_\w+?_(Std|Pool|FreeList|Frame)")
_PARAMS_RE = re.compile(r"(BM_\w+?_)(?:Std|Pool|FreeList|Frame)(/.+)")
_COUNT_RE = re.compile(r"/(\d+)(?:/repeats|$)")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Existing benchmark JSON (skips running benchmarks)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(".benchmark_results/allocators_check.json"),
        help="Output JSON path (default: .benchmark_results/allocators_check.json)",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type (default: Release)",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=3,
        help="Benchmark repetitions (default: 3)",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print all comparisons, not just regressions",
    )
    return parser.parse_args(argv)


def _to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def _run_benchmarks(output: Path, build_type: str, repetitions: int) -> Path:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type

    if not build_dir.exists():
        print(
            "Build directory {} not found. Run `pixi run build` first.".format(
                build_dir
            ),
            file=sys.stderr,
        )
        sys.exit(1)

    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(build_dir),
            "--target",
            "bm_allocators",
        ],
        check=True,
    )

    binary_candidates = [
        build_dir / "bin" / "bm_allocators",
        build_dir / "tests" / "benchmark" / "common" / "bm_allocators",
    ]
    binary = None
    for candidate in binary_candidates:
        if candidate.exists():
            binary = candidate
            break

    if binary is None:
        print("bm_allocators binary not found in {}".format(build_dir), file=sys.stderr)
        sys.exit(1)

    output.parent.mkdir(parents=True, exist_ok=True)

    subprocess.run(
        [
            str(binary),
            "--benchmark_repetitions={}".format(repetitions),
            "--benchmark_out={}".format(output),
            "--benchmark_out_format=json",
        ],
        check=True,
    )

    return output


def _extract_allocator(name: str) -> str | None:
    m = _ALLOC_RE.match(name)
    return m.group(1) if m else None


def _make_comparison_key(name: str) -> str | None:
    m = _PARAMS_RE.match(name)
    if m:
        return m.group(1) + "ALLOC" + m.group(2)
    return None


def _extract_count(params: str) -> int | None:
    parts = params.strip("/").split("/")
    if len(parts) >= 2:
        try:
            return int(parts[1].split("/")[0])
        except ValueError:
            pass
    m = _COUNT_RE.search(params)
    return int(m.group(1)) if m else None


def _load_medians(filepath: Path) -> list[dict]:
    with open(filepath) as f:
        data = json.load(f)
    benchmarks = data.get("benchmarks", [])

    medians = [
        b
        for b in benchmarks
        if b.get("run_type") == "aggregate" and b.get("aggregate_name") == "median"
    ]
    if medians:
        return medians

    return [b for b in benchmarks if b.get("run_type") != "aggregate"]


def _get_threshold(benchmark_key: str, allocator: str) -> float | None:
    """Return max allowed ratio (DART/Std), or None to skip the check.

    Returns None for cases where DART allocators are expected to be slower
    than system malloc (known tradeoff for memory control benefits).
    """
    m = _PARAMS_RE.match(benchmark_key.replace("ALLOC", allocator))
    if not m:
        return None

    prefix = m.group(1)
    params = m.group(2)
    count = _extract_count(params)

    if allocator == "Frame":
        return 1.3

    if "RealisticWorkload" in prefix:
        return 1.3

    if "StlContainer" in prefix:
        return 1.3

    if "FrameBulkReset" in prefix:
        return 1.3

    if "MixedSteadyState" in prefix:
        return None

    if "PoolBoundary" in prefix:
        return None

    if "MMDispatch" in prefix or "DirectFreeList" in prefix:
        return None

    if "MMConstruct" in prefix:
        return None

    if "SingleAlloc" in prefix:
        if count is not None and count <= 1:
            return None

        size_match = re.search(r"/(\d+)/", params)
        alloc_size = int(size_match.group(1)) if size_match else 0

        if allocator == "Pool" and count is not None and count >= 16:
            return 1.5
        if allocator == "FreeList":
            if alloc_size < 128 or (count is not None and count < 64):
                return None
            return 1.5
        return None

    return 1.5


def check_regressions(
    filepath: Path, verbose: bool
) -> tuple[list[dict], list[dict], int]:
    entries = _load_medians(filepath)

    by_key: dict[str, dict[str, float]] = defaultdict(dict)
    for entry in entries:
        name = entry.get("run_name", entry.get("name", ""))
        allocator = _extract_allocator(name)
        if allocator is None:
            continue
        key = _make_comparison_key(name)
        if key is None:
            continue
        time_ns = _to_ns(entry.get("cpu_time", 0), entry.get("time_unit", "ns"))
        by_key[key][allocator] = time_ns

    regressions = []
    passes = []
    skipped = 0

    dart_allocators = {"Pool", "FreeList", "Frame"}

    for key in sorted(by_key.keys()):
        alloc_times = by_key[key]
        std_time = alloc_times.get("Std")
        if std_time is None or std_time <= 0:
            continue

        for alloc in sorted(dart_allocators & set(alloc_times.keys())):
            threshold = _get_threshold(key, alloc)
            if threshold is None:
                skipped += 1
                continue

            dart_time = alloc_times[alloc]
            ratio = dart_time / std_time

            display_key = key.replace("ALLOC", alloc)
            result = {
                "benchmark": display_key,
                "allocator": alloc,
                "dart_ns": round(dart_time, 1),
                "std_ns": round(std_time, 1),
                "ratio": round(ratio, 3),
                "threshold": threshold,
            }

            if ratio > threshold:
                result["status"] = "REGRESSION"
                regressions.append(result)
            else:
                result["status"] = "PASS"
                passes.append(result)

    return regressions, passes, skipped


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if args.input and args.input.exists():
        json_path = args.input
        print("Using existing results: {}".format(json_path))
    else:
        print("Running bm_allocators...")
        json_path = _run_benchmarks(args.output, args.build_type, args.repetitions)
        print("Results saved to: {}".format(json_path))

    regressions, passes, skipped = check_regressions(json_path, args.verbose)

    checked = len(regressions) + len(passes)
    print(
        "\n{} checks performed, {} skipped (expected tradeoffs)".format(
            checked, skipped
        )
    )

    if args.verbose:
        for p in passes:
            speedup = (1.0 / p["ratio"] - 1) * 100 if p["ratio"] > 0 else 0
            print(
                "  PASS  {} [<={}x]: {:.1f} ns vs Std {:.1f} ns (ratio {:.2f})".format(
                    p["benchmark"],
                    p["threshold"],
                    p["dart_ns"],
                    p["std_ns"],
                    p["ratio"],
                )
            )

    if regressions:
        print("\nREGRESSIONS DETECTED ({}):\n".format(len(regressions)))
        for r in regressions:
            slowdown = (r["ratio"] - 1) * 100
            print(
                "  FAIL  {} [{}]: {:.1f} ns vs Std {:.1f} ns "
                "(ratio {:.2f}, threshold {:.1f}, {:.0f}% slower)".format(
                    r["benchmark"],
                    r["allocator"],
                    r["dart_ns"],
                    r["std_ns"],
                    r["ratio"],
                    r["threshold"],
                    slowdown,
                )
            )
        print("\n{} regression(s) detected.".format(len(regressions)))
        return 1

    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
