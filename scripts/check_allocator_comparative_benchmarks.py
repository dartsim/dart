#!/usr/bin/env python3
"""Check allocator benchmarks against foonathan/memory and standard allocators.

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
    python scripts/check_allocator_comparative_benchmarks.py --baseline std
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
    "BM_(Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk|StlVector)_"
    "(DART|Foonathan|StdPmr)"
)

ENTT_REGISTRY_FILTER = (
    "BM_(Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk|StlVector|"
    "EnttRegistry|EnttRegistryBuild)_(DART|Foonathan|StdPmr|Std)"
)
ENTT_REGISTRY_ONLY_FILTER = "BM_(EnttRegistry|EnttRegistryBuild)_(DART|Foonathan|Std)"

_COMPARATIVE_RE = re.compile(
    r"^(BM_(?:Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk|StlVector|"
    r"EnttRegistry|EnttRegistryBuild))_(DART|Foonathan|StdPmr|Std)(/.*)?$"
)
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_BASELINE_ALLOCATORS = {
    "foonathan": ("Foonathan",),
    "std": ("StdPmr", "Std"),
    "stdpmr": ("StdPmr", "Std"),
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
        default="1.0s",
        help=(
            "Minimum benchmark time passed to Google Benchmark. The default "
            "keeps the strict manual comparison gate focused on sustained "
            "allocator timings instead of short-run noise."
        ),
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
    entt_mode = parser.add_mutually_exclusive_group()
    entt_mode.add_argument(
        "--include-entt-registry",
        action="store_true",
        help=(
            "Include allocator-aware EnTT registry/component storage rows. "
            "These rows are an evidence surface for allocator-policy work and "
            "are intentionally opt-in until the production registry allocator "
            "policy consistently beats the baselines."
        ),
    )
    entt_mode.add_argument(
        "--only-entt-registry",
        action="store_true",
        help=(
            "Run only allocator-aware EnTT registry/component storage rows. "
            "Use this for focused registry allocator optimization loops without "
            "rerunning the broader allocator benchmark set."
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
        "--max-cv",
        type=float,
        default=0.10,
        help=(
            "Maximum allowed coefficient of variation for compared aggregate "
            "rows. The default 0.10 rejects rows noisier than 10%% before "
            "treating timing ratios as evidence."
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
    try:
        with path.open(encoding="utf-8") as f:
            data = json.load(f)
    except FileNotFoundError as exc:
        raise BenchmarkCheckError(f"benchmark JSON not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise BenchmarkCheckError(
            f"benchmark JSON is empty or invalid: {path}"
        ) from exc
    rows = data.get("benchmarks", [])
    if not isinstance(rows, list):
        raise BenchmarkCheckError("benchmark JSON has no benchmark row list")
    return rows


def filter_benchmark_rows_for_mode(
    rows: list[dict], *, include_entt_registry: bool, only_entt_registry: bool
) -> list[dict]:
    benchmark_filter = re.compile(
        benchmark_filter_for_mode(
            include_entt_registry=include_entt_registry,
            only_entt_registry=only_entt_registry,
        )
    )
    return [
        row for row in rows if benchmark_filter.search(_canonical_name(_row_name(row)))
    ]


def require_requested_entt_registry_rows(
    rows: list[dict], *, include_entt_registry: bool, only_entt_registry: bool
) -> None:
    if not include_entt_registry and not only_entt_registry:
        return

    if any(
        _canonical_name(_row_name(row)).startswith(
            ("BM_EnttRegistry_", "BM_EnttRegistryBuild_")
        )
        for row in rows
    ):
        return

    raise BenchmarkCheckError(
        "EnTT registry benchmark rows were requested but are unavailable. "
        "Configure this benchmark in an environment where EnTT::EnTT is already "
        "available, for example with DART_BUILD_SIMULATION_EXPERIMENTAL=ON."
    )


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


def collect_coefficients_of_variation(
    rows: list[dict], metric: str = "cpu_time"
) -> dict[str, dict[str, float]]:
    cvs: dict[str, dict[str, float]] = defaultdict(dict)

    for row in rows:
        if row.get("run_type") != "aggregate" or row.get("aggregate_name") != "cv":
            continue

        name = _canonical_name(_row_name(row))
        match = _COMPARATIVE_RE.match(name)
        if match is None:
            continue

        family, allocator, args = match.groups()
        key = family + (args or "")
        try:
            cv = float(row.get(metric))
        except (TypeError, ValueError):
            continue
        if math.isfinite(cv) and cv >= 0.0:
            cvs[key][allocator] = cv

    return dict(cvs)


def collect_dart_counters(rows: list[dict]) -> dict[str, dict[str, float]]:
    counters: dict[str, dict[str, float]] = defaultdict(dict)

    for row in _select_rows(rows):
        name = _canonical_name(_row_name(row))
        match = _COMPARATIVE_RE.match(name)
        if match is None:
            continue

        family, allocator, args = match.groups()
        if allocator != "DART":
            continue

        key = family + (args or "")
        for counter_name, value in row.items():
            if not counter_name.startswith("dart_"):
                continue
            try:
                counter = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(counter):
                counters[key][counter_name] = counter

    return dict(counters)


def evaluate_comparisons(
    rows: list[dict],
    *,
    baseline_allocators: list[tuple[str, ...]],
    max_ratio: float = 1.0,
    max_cv: float | None = 0.10,
    metric: str = "cpu_time",
) -> tuple[list[dict], list[dict]]:
    if not baseline_allocators:
        baseline_allocators = [("Foonathan",)]

    timings = collect_timings(rows, metric)
    cvs = collect_coefficients_of_variation(rows, metric)
    dart_counters = collect_dart_counters(rows)
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

        for candidates in baseline_allocators:
            baseline = next(
                (candidate for candidate in candidates if candidate in allocs),
                None,
            )
            baseline_time = allocs.get(baseline) if baseline is not None else None
            if baseline_time is None:
                failures.append(
                    {
                        "benchmark": key,
                        "baseline": "/".join(candidates),
                        "status": "MISSING_BASELINE",
                    }
                )
                continue

            noisy_allocators = []
            if max_cv is not None:
                dart_cv = cvs.get(key, {}).get("DART")
                baseline_cv = cvs.get(key, {}).get(baseline)
                if dart_cv is not None and dart_cv > max_cv:
                    noisy_allocators.append(("DART", dart_cv))
                if baseline_cv is not None and baseline_cv > max_cv:
                    noisy_allocators.append((baseline, baseline_cv))

            if noisy_allocators:
                failures.append(
                    {
                        "benchmark": key,
                        "baseline": baseline,
                        "noisy_allocators": noisy_allocators,
                        "max_cv": max_cv,
                        "status": "NOISY",
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
            if key in dart_counters:
                result["dart_counters"] = dart_counters[key]
            if ratio < max_ratio:
                result["status"] = "PASS"
                passes.append(result)
            else:
                result["status"] = "FAIL"
                failures.append(result)

    return failures, passes


def benchmark_filter_for_mode(
    *, include_entt_registry: bool, only_entt_registry: bool
) -> str:
    if only_entt_registry:
        return ENTT_REGISTRY_ONLY_FILTER
    if include_entt_registry:
        return ENTT_REGISTRY_FILTER
    return DEFAULT_FILTER


def run_benchmark(
    output: Path,
    *,
    build_type: str,
    benchmark_min_time: str,
    include_entt_registry: bool,
    only_entt_registry: bool,
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
            "--benchmark_filter={}".format(
                benchmark_filter_for_mode(
                    include_entt_registry=include_entt_registry,
                    only_entt_registry=only_entt_registry,
                )
            ),
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


def _format_counter_value(value: float) -> str:
    rounded = round(value)
    if math.isclose(value, rounded, rel_tol=0.0, abs_tol=1e-9):
        return str(int(rounded))
    return "{:.3g}".format(value)


def _format_dart_counters(result: dict) -> str:
    counters = result.get("dart_counters")
    if not counters:
        return ""
    return ", ".join(
        "{}={}".format(name, _format_counter_value(value))
        for name, value in sorted(counters.items())
    )


def _print_result(prefix: str, result: dict) -> None:
    if result["status"].startswith("MISSING"):
        print(
            "  {}  {} vs {} ({})".format(
                prefix, result["benchmark"], result["baseline"], result["status"]
            )
        )
        return

    if result["status"] == "NOISY":
        details = ", ".join(
            "{} cv {:.2%}".format(allocator, cv)
            for allocator, cv in result["noisy_allocators"]
        )
        print(
            "  {}  {} vs {} (NOISY: {}, max {:.2%})".format(
                prefix,
                result["benchmark"],
                result["baseline"],
                details,
                result["max_cv"],
            )
        )
        return

    message = (
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
    counters = _format_dart_counters(result)
    if counters:
        message += "; DART counters: {}".format(counters)
    print(message)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.max_ratio <= 0.0 or not math.isfinite(args.max_ratio):
        raise SystemExit("--max-ratio must be a finite positive number")
    if args.max_cv <= 0.0 or not math.isfinite(args.max_cv):
        raise SystemExit("--max-cv must be a finite positive number")

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
            include_entt_registry=args.include_entt_registry,
            only_entt_registry=args.only_entt_registry,
            repetitions=args.repetitions,
        )
        print("Results saved to: {}".format(json_path))

    try:
        rows = filter_benchmark_rows_for_mode(
            load_benchmark_rows(json_path),
            include_entt_registry=args.include_entt_registry,
            only_entt_registry=args.only_entt_registry,
        )
        require_requested_entt_registry_rows(
            rows,
            include_entt_registry=args.include_entt_registry,
            only_entt_registry=args.only_entt_registry,
        )
        failures, passes = evaluate_comparisons(
            rows,
            baseline_allocators=baseline_allocators,
            max_ratio=args.max_ratio,
            max_cv=args.max_cv,
            metric=args.metric,
        )
    except BenchmarkCheckError as exc:
        print("Allocator comparative check failed: {}".format(exc), file=sys.stderr)
        return 1

    print(
        "\n{} comparative allocator checks performed against {}.".format(
            len(failures) + len(passes),
            ", ".join("/".join(group) for group in baseline_allocators),
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
