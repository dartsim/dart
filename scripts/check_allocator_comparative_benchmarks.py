#!/usr/bin/env python3
"""Check allocator benchmarks against foonathan/memory and std::pmr.

Runs bm_allocators_comparative (or reads an existing Google Benchmark JSON)
and fails when DART allocator timings do not beat the selected baseline
allocator on matching workloads.

The default foonathan rule is intentionally strict: DART must be faster than
the corresponding foonathan/memory row. This script is a manual evidence gate
for deciding whether DART's in-house allocators are good enough for broader
simulation-loop adoption.

Usage:
    python scripts/check_allocator_comparative_benchmarks.py
    python scripts/check_allocator_comparative_benchmarks.py --input result.json
    python scripts/check_allocator_comparative_benchmarks.py --baseline stdpmr
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}

DEFAULT_FILTER = (
    "BM_(Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk)_"
    "(DART|Foonathan|StdPmr)"
)

_COMPARATIVE_RE = re.compile(
    r"^(BM_(?:Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk))"
    r"_(DART|Foonathan|StdPmr)(/.*)?$"
)
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_BASELINE_ALLOCATORS = {
    "foonathan": "Foonathan",
    "stdpmr": "StdPmr",
}


class BenchmarkCheckError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="Existing Google Benchmark JSON file. Skips running the benchmark.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(".benchmark_results/allocator_comparative_check.json"),
        help="Output JSON path when running the benchmark.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use when running the benchmark.",
    )
    parser.add_argument(
        "--benchmark-min-time",
        default="10ms",
        help="Minimum benchmark time passed to Google Benchmark.",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=5,
        help="Benchmark repetitions when running the benchmark.",
    )
    parser.add_argument(
        "--baseline",
        action="append",
        choices=sorted(_BASELINE_ALLOCATORS),
        default=None,
        help=(
            "Baseline allocator family to compare against. May be passed more "
            "than once. Defaults to foonathan."
        ),
    )
    parser.add_argument(
        "--max-ratio",
        type=float,
        default=1.0,
        help=(
            "Maximum allowed DART/baseline timing ratio. The default 1.0 means "
            "DART must be faster than the baseline median."
        ),
    )
    parser.add_argument(
        "--metric",
        choices=("cpu_time", "real_time"),
        default="cpu_time",
        help="Timing metric to compare.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print passing comparisons in addition to failures.",
    )
    return parser.parse_args(argv)


def _to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def _row_name(row: dict) -> str:
    name = row.get("run_name", row.get("name", ""))
    return name if isinstance(name, str) else ""


def _canonical_name(name: str) -> str:
    return _AGGREGATE_SUFFIX_RE.sub("", _REPEATS_SUFFIX_RE.sub("", name))


def _timing_ns(row: dict, metric: str) -> float:
    value = row.get(metric)
    if value is None:
        return math.nan
    try:
        return _to_ns(float(value), row.get("time_unit", "ns"))
    except (TypeError, ValueError):
        return math.nan


def load_benchmark_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    rows = data.get("benchmarks", [])
    if not isinstance(rows, list):
        raise BenchmarkCheckError("benchmark JSON has no benchmark row list")
    return rows


def _select_rows(rows: list[dict]) -> list[dict]:
    medians = [
        row
        for row in rows
        if row.get("run_type") == "aggregate" and row.get("aggregate_name") == "median"
    ]
    if medians:
        return medians

    return [row for row in rows if row.get("aggregate_name") not in {"stddev", "cv"}]


def collect_timings(
    rows: list[dict], metric: str = "cpu_time"
) -> dict[str, dict[str, float]]:
    timings: dict[str, dict[str, float]] = defaultdict(dict)

    for row in _select_rows(rows):
        name = _canonical_name(_row_name(row))
        match = _COMPARATIVE_RE.match(name)
        if match is None:
            continue

        family, allocator, args = match.groups()
        key = family + (args or "")
        timing = _timing_ns(row, metric)
        if math.isfinite(timing) and timing > 0.0:
            timings[key][allocator] = timing

    return dict(timings)


def evaluate_comparisons(
    rows: list[dict],
    *,
    baseline_allocators: list[str],
    max_ratio: float = 1.0,
    metric: str = "cpu_time",
) -> tuple[list[dict], list[dict]]:
    if not baseline_allocators:
        baseline_allocators = ["Foonathan"]

    timings = collect_timings(rows, metric)
    failures = []
    passes = []

    if not timings:
        raise BenchmarkCheckError("no comparative allocator benchmark rows found")

    for key in sorted(timings):
        allocs = timings[key]
        dart_time = allocs.get("DART")
        if dart_time is None:
            failures.append(
                {
                    "benchmark": key,
                    "baseline": "<none>",
                    "status": "MISSING_DART",
                }
            )
            continue

        for baseline in baseline_allocators:
            baseline_time = allocs.get(baseline)
            if baseline_time is None:
                failures.append(
                    {
                        "benchmark": key,
                        "baseline": baseline,
                        "status": "MISSING_BASELINE",
                    }
                )
                continue

            ratio = dart_time / baseline_time
            result = {
                "benchmark": key,
                "baseline": baseline,
                "dart_ns": round(dart_time, 3),
                "baseline_ns": round(baseline_time, 3),
                "ratio": round(ratio, 4),
                "max_ratio": max_ratio,
            }
            if ratio < max_ratio:
                result["status"] = "PASS"
                passes.append(result)
            else:
                result["status"] = "FAIL"
                failures.append(result)

    return failures, passes


def run_benchmark(
    output: Path,
    *,
    build_type: str,
    benchmark_min_time: str,
    repetitions: int,
) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    subprocess.run(
        [
            sys.executable,
            "scripts/run_cpp_benchmark.py",
            "allocators-comparative",
            "--build-type",
            build_type,
            "--benchmark_filter={}".format(DEFAULT_FILTER),
            "--benchmark_min_time={}".format(benchmark_min_time),
            "--benchmark_repetitions={}".format(repetitions),
            "--benchmark_report_aggregates_only=true",
            "--benchmark_out={}".format(output),
            "--benchmark_out_format=json",
        ],
        check=True,
        env=env,
    )
    return output


def _print_result(prefix: str, result: dict) -> None:
    if result["status"].startswith("MISSING"):
        print(
            "  {}  {} vs {} ({})".format(
                prefix, result["benchmark"], result["baseline"], result["status"]
            )
        )
        return

    print(
        "  {}  {} vs {}: DART {:.1f} ns, baseline {:.1f} ns, "
        "ratio {:.3f} (max {:.3f})".format(
            prefix,
            result["benchmark"],
            result["baseline"],
            result["dart_ns"],
            result["baseline_ns"],
            result["ratio"],
            result["max_ratio"],
        )
    )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.max_ratio <= 0.0 or not math.isfinite(args.max_ratio):
        raise SystemExit("--max-ratio must be a finite positive number")

    baseline_allocators = [
        _BASELINE_ALLOCATORS[name] for name in (args.baseline or ["foonathan"])
    ]

    if args.input is not None:
        json_path = args.input
        print("Using existing results: {}".format(json_path))
    else:
        print("Running bm_allocators_comparative...")
        json_path = run_benchmark(
            args.output,
            build_type=args.build_type,
            benchmark_min_time=args.benchmark_min_time,
            repetitions=args.repetitions,
        )
        print("Results saved to: {}".format(json_path))

    try:
        failures, passes = evaluate_comparisons(
            load_benchmark_rows(json_path),
            baseline_allocators=baseline_allocators,
            max_ratio=args.max_ratio,
            metric=args.metric,
        )
    except BenchmarkCheckError as exc:
        print("Allocator comparative check failed: {}".format(exc), file=sys.stderr)
        return 1

    print(
        "\n{} comparative allocator checks performed against {}.".format(
            len(failures) + len(passes),
            ", ".join(baseline_allocators),
        )
    )

    if args.verbose:
        for result in passes:
            _print_result("PASS", result)

    if failures:
        print("\nCOMPARATIVE FAILURES ({}):".format(len(failures)))
        for result in failures:
            _print_result("FAIL", result)
        return 1

    print("\nAll comparative allocator checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
