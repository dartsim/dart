#!/usr/bin/env python3
"""Smoke-check the experimental compute-graph benchmark surface.

This checker is primarily a correctness/coverage gate. It verifies that the
compute-graph, world-step, rigid-body-step, contact-shaped,
contact-island-shaped, and Phase 5 CPU-baseline smoke benchmark rows run and
report finite positive timings. Ratio summaries are printed for review. The
current Euler-only rigid-body workload is known to be overhead-bound and must
not be used to choose CPU/GPU backends, but the contact-island surface is
compute-bound enough to require a bounded parallel speedup sanity check in the
dedicated benchmark gate.

Usage:
    python scripts/check_compute_graph_benchmarks.py
    python scripts/check_compute_graph_benchmarks.py --input result.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}

DEFAULT_FILTER = (
    "BM_(ComputeGraph(Build|Sequential|Parallel)"
    "|WorldStep(Sequential|Parallel)"
    "|RigidBodyStep(Sequential|Parallel)"
    "|ContactShaped(Sequential|Parallel)"
    "|ContactIslandShaped(Sequential|Parallel))/.*"
    "|BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10"
)


REQUIRED_BENCHMARKS = (
    "BM_ComputeGraphBuild/1024/1",
    "BM_ComputeGraphBuild/1024/32",
    "BM_ComputeGraphBuild/4096/64",
    "BM_ComputeGraphSequential/1024/1",
    "BM_ComputeGraphSequential/1024/32",
    "BM_ComputeGraphSequential/4096/64",
    "BM_ComputeGraphParallel/1024/1",
    "BM_ComputeGraphParallel/1024/32",
    "BM_ComputeGraphParallel/4096/64",
    "BM_WorldStepSequential/32/8",
    "BM_WorldStepSequential/128/8",
    "BM_WorldStepSequential/128/32",
    "BM_WorldStepParallel/32/8",
    "BM_WorldStepParallel/128/8",
    "BM_WorldStepParallel/128/32",
    "BM_RigidBodyStepSequential/128",
    "BM_RigidBodyStepSequential/1024",
    "BM_RigidBodyStepSequential/4096",
    "BM_RigidBodyStepParallel/128",
    "BM_RigidBodyStepParallel/1024",
    "BM_RigidBodyStepParallel/4096",
    "BM_ContactShapedSequential/1024/16",
    "BM_ContactShapedSequential/4096/16",
    "BM_ContactShapedSequential/1024/64",
    "BM_ContactShapedParallel/1024/16",
    "BM_ContactShapedParallel/4096/16",
    "BM_ContactShapedParallel/1024/64",
    "BM_ContactIslandShapedSequential/4/512/64",
    "BM_ContactIslandShapedSequential/8/512/64",
    "BM_ContactIslandShapedSequential/16/512/64",
    "BM_ContactIslandShapedParallel/4/512/64",
    "BM_ContactIslandShapedParallel/8/512/64",
    "BM_ContactIslandShapedParallel/16/512/64",
    "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
)

_PAIR_RE = re.compile(
    r"^(BM_(?:ComputeGraph|WorldStep|RigidBodyStep|ContactShaped|"
    r"ContactIslandShaped))"
    r"(Sequential|Parallel)(/.*)$"
)
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_CONTACT_ISLAND_SPEEDUP_BENCHMARK = "BM_ContactIslandShaped/16/512/64"
_CONTACT_ISLAND_MAX_PARALLEL_OVER_SEQUENTIAL = 0.95


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
        default=Path(".benchmark_results/compute_graph_check.json"),
        help="Output JSON path when running the benchmark.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use when running the benchmark.",
    )
    parser.add_argument(
        "--benchmark-min-time",
        default="1ms",
        help="Minimum benchmark time passed to Google Benchmark.",
    )
    parser.add_argument(
        "--skip-contact-island-speedup-check",
        action="store_true",
        help=(
            "Only validate required rows and finite timings. Use this for noisy "
            "dashboard publication runs; the default bm-compute-check gate keeps "
            "the contact-island speedup sanity check enabled."
        ),
    )
    return parser.parse_args(argv)


def _to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def _timing_ns(row: dict) -> float:
    unit = row.get("time_unit", "ns")
    value = row.get("real_time", row.get("cpu_time"))
    if value is None:
        return math.nan
    try:
        return _to_ns(float(value), unit)
    except TypeError, ValueError:
        return math.nan


def _row_name(row: dict) -> str:
    name = row.get("run_name", row.get("name", ""))
    return name if isinstance(name, str) else ""


def _canonical_name(name: str) -> str:
    return _AGGREGATE_SUFFIX_RE.sub("", _REPEATS_SUFFIX_RE.sub("", name))


def _is_timing_row(row: dict) -> bool:
    return row.get("aggregate_name") not in {"stddev", "cv"}


def load_benchmark_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    rows = data.get("benchmarks", [])
    if not isinstance(rows, list):
        raise BenchmarkCheckError("benchmark JSON has no benchmark row list")
    return rows


def validate_benchmark_rows(
    rows: list[dict],
    *,
    check_contact_island_speedup: bool = True,
) -> dict:
    missing = []
    invalid = []
    matched_rows: list[dict] = []
    rows_by_name: dict[str, list[dict]] = {}

    for row in rows:
        if not _is_timing_row(row):
            continue
        name = _row_name(row)
        if name:
            rows_by_name.setdefault(_canonical_name(name), []).append(row)

    for name in REQUIRED_BENCHMARKS:
        matches = rows_by_name.get(name, [])
        if not matches:
            missing.append(name)
            continue
        matched_rows.extend(matches)

    for row in matched_rows:
        name = _row_name(row) or "<unnamed>"
        timing = _timing_ns(row)
        if not math.isfinite(timing) or timing <= 0.0:
            invalid.append(name)

    if missing or invalid:
        details = []
        if missing:
            details.append("missing surfaces: " + ", ".join(missing))
        if invalid:
            details.append("invalid timings: " + ", ".join(invalid))
        raise BenchmarkCheckError("; ".join(details))

    ratios = compute_parallel_ratios(matched_rows)
    if check_contact_island_speedup:
        validate_contact_island_speedup(ratios)
    return {
        "row_count": len(matched_rows),
        "ratio_count": len(ratios),
        "parallel_ratios": ratios,
    }


def compute_parallel_ratios(rows: list[dict]) -> list[dict]:
    grouped: dict[tuple[str, str], dict[str, float]] = {}
    for row in rows:
        name = _canonical_name(_row_name(row))
        match = _PAIR_RE.match(name)
        if not match:
            continue
        family, mode, args = match.groups()
        grouped.setdefault((family, args), {})[mode] = _timing_ns(row)

    ratios = []
    for (family, args), timings in sorted(grouped.items()):
        sequential = timings.get("Sequential")
        parallel = timings.get("Parallel")
        if (
            sequential is None
            or parallel is None
            or not math.isfinite(sequential)
            or not math.isfinite(parallel)
            or sequential <= 0.0
        ):
            continue
        ratios.append(
            {
                "benchmark": f"{family}{args}",
                "parallel_over_sequential": parallel / sequential,
            }
        )
    return ratios


def validate_contact_island_speedup(ratios: list[dict]) -> None:
    for row in ratios:
        if row["benchmark"] != _CONTACT_ISLAND_SPEEDUP_BENCHMARK:
            continue
        ratio = row["parallel_over_sequential"]
        if ratio >= _CONTACT_ISLAND_MAX_PARALLEL_OVER_SEQUENTIAL:
            raise BenchmarkCheckError(
                "{} parallel/sequential {:.3f} does not beat the "
                "required threshold {:.3f}".format(
                    _CONTACT_ISLAND_SPEEDUP_BENCHMARK,
                    ratio,
                    _CONTACT_ISLAND_MAX_PARALLEL_OVER_SEQUENTIAL,
                )
            )
        return

    raise BenchmarkCheckError(f"missing ratio for {_CONTACT_ISLAND_SPEEDUP_BENCHMARK}")


def _find_binary(build_dir: Path, target: str) -> Path:
    candidates = [
        build_dir / "bin" / target,
        build_dir / "tests" / "benchmark" / "simulation" / "experimental" / target,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise SystemExit(f"{target} binary not found in {build_dir}")


def run_benchmark(output: Path, build_type: str, benchmark_min_time: str) -> Path:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type
    if not build_dir.exists():
        raise SystemExit(
            f"Build directory {build_dir} does not exist. Run `pixi run config` first."
        )

    env = os.environ.copy()
    env["BUILD_TYPE"] = build_type
    env["CMAKE_BUILD_DIR"] = str(build_dir)

    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(build_dir),
            "--target",
            "bm_compute_graph",
        ],
        check=True,
        env=env,
    )

    binary = _find_binary(build_dir, "bm_compute_graph")
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            str(binary),
            f"--benchmark_filter={DEFAULT_FILTER}",
            f"--benchmark_min_time={benchmark_min_time}",
            f"--benchmark_out={output}",
            "--benchmark_out_format=json",
        ],
        check=True,
        env=env,
    )
    return output


def _print_summary(summary: dict) -> None:
    print(f"Checked {summary['row_count']} compute benchmark rows.")
    for ratio in summary["parallel_ratios"]:
        print(
            "{}: parallel/sequential {:.3f}".format(
                ratio["benchmark"], ratio["parallel_over_sequential"]
            )
        )


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    path = args.input
    if path is None:
        path = run_benchmark(args.output, args.build_type, args.benchmark_min_time)

    try:
        summary = validate_benchmark_rows(
            load_benchmark_rows(path),
            check_contact_island_speedup=not args.skip_contact_island_speedup_check,
        )
    except BenchmarkCheckError as exc:
        raise SystemExit(str(exc)) from exc

    _print_summary(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
