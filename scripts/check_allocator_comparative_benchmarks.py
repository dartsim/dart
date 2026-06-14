#!/usr/bin/env python3
"""Check allocator benchmarks against foonathan/memory and standard allocators.

Runs bm_allocators_comparative (or reads an existing Google Benchmark JSON)
and fails when DART allocator timings do not beat the selected baseline
allocator on matching workloads.

The default foonathan rule is intentionally strict: DART must be faster than
the corresponding foonathan/memory row. This script is a manual evidence gate
for deciding whether DART's in-house allocators are good enough for broader
simulation-loop adoption. The checker also requires the benchmark rows expected
for the selected mode, so missing foonathan/memory coverage cannot be mistaken
for a pass.

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
    "BM_(Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk|StlVector|"
    "StaticStack|Temporary|Iteration|RawHeap|RawMalloc|RawNew|"
    "AlignedStack|FallbackStack|Segregator|TrackedStack|DeepTrackedPool)_"
    "(DART|Foonathan|StdPmr)"
)

ENTT_REGISTRY_FILTER = (
    "BM_(Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk|StlVector|"
    "StaticStack|Temporary|Iteration|RawHeap|RawMalloc|RawNew|"
    "AlignedStack|FallbackStack|Segregator|TrackedStack|DeepTrackedPool|"
    "EnttRegistry|EnttRegistryBuild)_"
    "(DART|Foonathan|StdPmr|Std)"
)
ENTT_REGISTRY_ONLY_FILTER = "BM_(EnttRegistry|EnttRegistryBuild)_(DART|Foonathan|Std)"

_COMPARATIVE_RE = re.compile(
    r"^(BM_(?:Pool|Stack|MultiPool|Realistic|SteadyState|FrameBulk|StlVector|"
    r"StaticStack|Temporary|Iteration|RawHeap|RawMalloc|RawNew|"
    r"AlignedStack|FallbackStack|Segregator|TrackedStack|DeepTrackedPool|"
    r"EnttRegistry|EnttRegistryBuild))_"
    r"(DART|Foonathan|StdPmr|Std)(/.*)?$"
)
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
# Normal-approximation 95% confidence interval over benchmark repetitions.
_NOISY_SEPARATION_Z = 1.96
_BASELINE_ALLOCATORS = {
    "foonathan": ("Foonathan",),
    "std": ("StdPmr", "Std"),
    "stdpmr": ("StdPmr", "Std"),
}

_DEFAULT_REQUIRED_KEYS = (
    "BM_Pool/32/64",
    "BM_Pool/256/256",
    "BM_Pool/32/1024",
    "BM_Stack/32/64",
    "BM_Stack/256/256",
    "BM_Stack/256/1024",
    "BM_Stack/32/4096",
    "BM_MultiPool",
    "BM_Realistic",
    "BM_SteadyState/64/1024",
    "BM_SteadyState/256/512",
    "BM_FrameBulk/256",
    "BM_FrameBulk/1024",
    "BM_FrameBulk/4096",
    "BM_StlVector/1000",
    "BM_StlVector/10000",
    "BM_StaticStack/256",
    "BM_StaticStack/1024",
    "BM_StaticStack/4096",
    "BM_Temporary/256",
    "BM_Temporary/1024",
    "BM_Temporary/4096",
    "BM_Iteration/256",
    "BM_Iteration/1024",
    "BM_Iteration/4096",
    "BM_RawHeap/256",
    "BM_RawHeap/1024",
    "BM_RawMalloc/256",
    "BM_RawMalloc/1024",
    "BM_RawNew/256",
    "BM_RawNew/1024",
    "BM_AlignedStack/256",
    "BM_AlignedStack/1024",
    "BM_FallbackStack/256",
    "BM_FallbackStack/1024",
    "BM_Segregator/256",
    "BM_Segregator/1024",
    "BM_TrackedStack/256",
    "BM_TrackedStack/1024",
    "BM_DeepTrackedPool/256",
    "BM_DeepTrackedPool/1024",
)
_ENTT_REQUIRED_KEYS = (
    "BM_EnttRegistry/256",
    "BM_EnttRegistry/512",
    "BM_EnttRegistry/2048",
    "BM_EnttRegistryBuild/256",
    "BM_EnttRegistryBuild/512",
    "BM_EnttRegistryBuild/2048",
)


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
        "--benchmark-min-warmup-time",
        default=None,
        help=(
            "Optional Google Benchmark warmup time in seconds. Pass a bare "
            "number such as 0.1; this host's Google Benchmark rejects an "
            "'s'-suffixed warmup value."
        ),
    )
    parser.add_argument(
        "--benchmark-random-interleaving",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Enable Google Benchmark random interleaving for repetitions. This "
            "can reduce order and thermal bias in strict allocator comparisons."
        ),
    )
    parser.add_argument(
        "--cpu-affinity",
        default=None,
        help=(
            "Pin only the benchmark binary to one CPU via run_cpp_benchmark.py. "
            "Pass a CPU index or 'auto' to choose a nonzero allowed CPU."
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
    except TypeError, ValueError:
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
        "Configure this benchmark in an environment where EnTT::EnTT is "
        "available through the DART 7 World stack."
    )


def required_keys_for_mode(
    *, include_entt_registry: bool, only_entt_registry: bool
) -> tuple[str, ...]:
    if only_entt_registry:
        return _ENTT_REQUIRED_KEYS
    if include_entt_registry:
        return (*_DEFAULT_REQUIRED_KEYS, *_ENTT_REQUIRED_KEYS)
    return _DEFAULT_REQUIRED_KEYS


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
        except TypeError, ValueError:
            continue
        if math.isfinite(cv) and cv >= 0.0:
            cvs[key][allocator] = cv

    return dict(cvs)


def collect_timing_statistics(
    rows: list[dict], metric: str = "cpu_time"
) -> dict[str, dict[str, dict[str, float]]]:
    statistics: dict[str, dict[str, dict[str, float]]] = defaultdict(
        lambda: defaultdict(dict)
    )

    for row in rows:
        if row.get("run_type") != "aggregate":
            continue
        aggregate = row.get("aggregate_name")
        if aggregate not in {"mean", "stddev"}:
            continue

        name = _canonical_name(_row_name(row))
        match = _COMPARATIVE_RE.match(name)
        if match is None:
            continue

        family, allocator, args = match.groups()
        key = family + (args or "")
        timing = _timing_ns(row, metric)
        if not math.isfinite(timing) or timing < 0.0:
            continue

        statistics[key][allocator][aggregate] = timing
        try:
            repetitions = int(row.get("repetitions", 0))
        except TypeError, ValueError:
            repetitions = 0
        if repetitions > 0:
            statistics[key][allocator]["repetitions"] = float(repetitions)

    return {
        key: {allocator: dict(values) for allocator, values in allocs.items()}
        for key, allocs in statistics.items()
    }


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
            except TypeError, ValueError:
                continue
            if math.isfinite(counter):
                counters[key][counter_name] = counter

    return dict(counters)


def _mean_confidence_interval(
    statistics: dict[str, float],
) -> tuple[float, float] | None:
    mean = statistics.get("mean")
    stddev = statistics.get("stddev")
    repetitions = statistics.get("repetitions")
    if mean is None or stddev is None or repetitions is None:
        return None
    if not all(math.isfinite(value) for value in (mean, stddev, repetitions)):
        return None
    if mean <= 0.0 or stddev < 0.0 or repetitions < 2.0:
        return None

    half_width = _NOISY_SEPARATION_Z * stddev / math.sqrt(repetitions)
    return max(0.0, mean - half_width), mean + half_width


def _separated_noisy_timing_evidence(
    timing_statistics: dict[str, dict[str, dict[str, float]]],
    *,
    key: str,
    baseline: str,
    max_ratio: float,
) -> dict | None:
    dart_interval = _mean_confidence_interval(
        timing_statistics.get(key, {}).get("DART", {})
    )
    baseline_interval = _mean_confidence_interval(
        timing_statistics.get(key, {}).get(baseline, {})
    )
    if dart_interval is None or baseline_interval is None:
        return None

    dart_lower, dart_upper = dart_interval
    baseline_lower, baseline_upper = baseline_interval
    if dart_upper < baseline_lower * max_ratio:
        return {
            "evidence": "MEAN_CI_SEPARATED",
            "dart_mean_ci_ns": [round(dart_lower, 3), round(dart_upper, 3)],
            "baseline_mean_ci_ns": [
                round(baseline_lower, 3),
                round(baseline_upper, 3),
            ],
            "confidence_z": _NOISY_SEPARATION_Z,
        }
    return None


def evaluate_comparisons(
    rows: list[dict],
    *,
    baseline_allocators: list[str | tuple[str, ...]],
    required_keys: tuple[str, ...] = (),
    max_ratio: float = 1.0,
    max_cv: float | None = 0.10,
    metric: str = "cpu_time",
) -> tuple[list[dict], list[dict]]:
    if not baseline_allocators:
        baseline_groups = [("Foonathan",)]
    else:
        baseline_groups = [
            (candidates,) if isinstance(candidates, str) else tuple(candidates)
            for candidates in baseline_allocators
        ]

    timings = collect_timings(rows, metric)
    cvs = collect_coefficients_of_variation(rows, metric)
    timing_statistics = collect_timing_statistics(rows, metric)
    dart_counters = collect_dart_counters(rows)
    failures = []
    passes = []

    if not timings:
        raise BenchmarkCheckError("no comparative allocator benchmark rows found")

    for key in required_keys:
        allocs = timings.get(key)
        if allocs is None:
            failures.append(
                {
                    "benchmark": key,
                    "baseline": "<required>",
                    "status": "MISSING_BENCHMARK",
                }
            )

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

        for candidates in baseline_groups:
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

            noisy_allocators = []
            if max_cv is not None:
                dart_cv = cvs.get(key, {}).get("DART")
                baseline_cv = cvs.get(key, {}).get(baseline)
                if dart_cv is not None and dart_cv > max_cv:
                    noisy_allocators.append(("DART", dart_cv))
                if baseline_cv is not None and baseline_cv > max_cv:
                    noisy_allocators.append((baseline, baseline_cv))

            if noisy_allocators:
                separated_evidence = _separated_noisy_timing_evidence(
                    timing_statistics,
                    key=key,
                    baseline=baseline,
                    max_ratio=max_ratio,
                )
                if ratio < max_ratio and separated_evidence is not None:
                    result.update(separated_evidence)
                    result["noisy_allocators"] = noisy_allocators
                    result["max_cv"] = max_cv
                    result["status"] = "PASS"
                    passes.append(result)
                    continue

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
    benchmark_min_warmup_time: str | None,
    benchmark_random_interleaving: bool,
    cpu_affinity: str | None,
    include_entt_registry: bool,
    only_entt_registry: bool,
    repetitions: int,
) -> Path:
    output.parent.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "allocators-comparative",
        "--build-type",
        build_type,
    ]
    if cpu_affinity is not None:
        command.extend(["--cpu-affinity", cpu_affinity])
    command.append("--")
    command.extend(
        [
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
        ]
    )
    if benchmark_min_warmup_time is not None:
        command.append(
            "--benchmark_min_warmup_time={}".format(benchmark_min_warmup_time)
        )
    if benchmark_random_interleaving:
        command.append("--benchmark_enable_random_interleaving=true")

    subprocess.run(
        command,
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
    if result.get("evidence") == "MEAN_CI_SEPARATED":
        message += (
            "; noisy but separated mean CI: DART [{:.1f}, {:.1f}] ns, "
            "baseline [{:.1f}, {:.1f}] ns (z={:.2f})"
        ).format(
            result["dart_mean_ci_ns"][0],
            result["dart_mean_ci_ns"][1],
            result["baseline_mean_ci_ns"][0],
            result["baseline_mean_ci_ns"][1],
            result["confidence_z"],
        )
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
            benchmark_min_warmup_time=args.benchmark_min_warmup_time,
            benchmark_random_interleaving=args.benchmark_random_interleaving,
            cpu_affinity=args.cpu_affinity,
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
            required_keys=required_keys_for_mode(
                include_entt_registry=args.include_entt_registry,
                only_entt_registry=args.only_entt_registry,
            ),
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
