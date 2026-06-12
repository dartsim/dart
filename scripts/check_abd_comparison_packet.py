#!/usr/bin/env python3
"""Run and validate the first PLAN-083 ABD comparison packet."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

from benchmark_packet_utils import (
    benchmark_row_name,
    benchmark_timing_field_errors,
    canonical_benchmark_name,
)

EXPECTED_BENCHMARKS = {
    "BM_AffineBodyPointTriangleBarrier",
    "BM_AffineBodyPointTriangleMicroSolve",
    "BM_AffineBodyRigidIpcPointTriangleOracle",
    "BM_AffineBodyOrthogonalityEnergy",
}

MICRO_SOLVE_BENCHMARK = "BM_AffineBodyPointTriangleMicroSolve"
DEFAULT_OUTPUT = Path(".benchmark_results/abd_comparison_packet.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Benchmark JSON output path (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type for the benchmark target (default: Release)",
    )
    parser.add_argument(
        "--benchmark-min-time",
        default="0.01s",
        help="Google Benchmark --benchmark_min_time value (default: 0.01s)",
    )
    parser.add_argument(
        "--benchmark-repetitions",
        type=int,
        default=3,
        help="Google Benchmark --benchmark_repetitions value (default: 3)",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Validate an existing JSON packet instead of running the benchmark",
    )
    return parser.parse_args()


def run_benchmark(args: argparse.Namespace) -> None:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    filter_expr = "^({})$".format("|".join(sorted(EXPECTED_BENCHMARKS)))
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_affine_body_dynamics",
        "--build-type",
        args.build_type,
        "--",
        f"--benchmark_filter={filter_expr}",
        f"--benchmark_min_time={args.benchmark_min_time}",
        f"--benchmark_repetitions={args.benchmark_repetitions}",
        "--benchmark_report_aggregates_only=true",
        f"--benchmark_out={args.output.as_posix()}",
        "--benchmark_out_format=json",
    ]
    subprocess.run(command, check=True)


def _finite_number(row: dict[str, object], field: str) -> float | None:
    value = row.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _micro_solve_counter_errors(row: dict[str, object]) -> list[str]:
    name = benchmark_row_name(row) or MICRO_SOLVE_BENCHMARK
    errors: list[str] = []

    required_fields = (
        "row_abd_alg_affine_body",
        "paper_scale",
        "affine_dynamic_body_count",
        "static_triangle_body_count",
        "point_triangle_pair_count",
        "valid_solve",
        "converged",
        "barrier_active",
        "solver_iterations",
        "initial_objective",
        "final_objective",
        "objective_decrease",
        "initial_gradient_norm",
        "final_gradient_norm",
        "initial_squared_distance",
        "final_squared_distance",
        "squared_activation_distance",
    )
    counters: dict[str, float] = {}
    for field in required_fields:
        value = _finite_number(row, field)
        if value is None:
            errors.append(f"{name} has missing or non-finite counter {field}")
        else:
            counters[field] = value

    if errors:
        return errors

    expected_flags = {
        "row_abd_alg_affine_body": 1.0,
        "paper_scale": 0.0,
        "affine_dynamic_body_count": 1.0,
        "static_triangle_body_count": 1.0,
        "point_triangle_pair_count": 1.0,
        "valid_solve": 1.0,
        "converged": 1.0,
        "barrier_active": 1.0,
    }
    for field, expected in expected_flags.items():
        if counters[field] != expected:
            errors.append(f"{name} expected {field}={expected}, got {counters[field]}")

    if counters["solver_iterations"] <= 0.0:
        errors.append(f"{name} solver_iterations must be positive")
    if counters["objective_decrease"] <= 0.0:
        errors.append(f"{name} objective_decrease must be positive")
    if counters["final_objective"] >= counters["initial_objective"]:
        errors.append(f"{name} final_objective must be below initial_objective")
    if counters["final_gradient_norm"] >= counters["initial_gradient_norm"]:
        errors.append(f"{name} final_gradient_norm must be below initial_gradient_norm")
    if counters["final_squared_distance"] <= 0.0:
        errors.append(f"{name} final_squared_distance must be positive")
    if counters["final_squared_distance"] <= counters["initial_squared_distance"]:
        errors.append(
            f"{name} final_squared_distance must exceed initial_squared_distance"
        )
    if counters["final_squared_distance"] >= counters["squared_activation_distance"]:
        errors.append(
            f"{name} final_squared_distance must stay inside activation distance"
        )
    return errors


def validate_packet(path: Path) -> None:
    with path.open(encoding="utf-8") as stream:
        payload = json.load(stream)

    rows = payload.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise SystemExit(f"{path} has no benchmark rows")

    seen: set[str] = set()
    errors: list[str] = []
    rows_by_base: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        if not isinstance(row, dict):
            errors.append("benchmark row is not an object")
            continue
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a string name")
            continue
        base_name = canonical_benchmark_name(name)
        if base_name not in EXPECTED_BENCHMARKS:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        seen.add(base_name)
        rows_by_base.setdefault(base_name, []).append(row)
        errors.extend(benchmark_timing_field_errors(row, name))

    missing = sorted(EXPECTED_BENCHMARKS - seen)
    if missing:
        errors.append("missing benchmark rows: {}".format(", ".join(missing)))

    micro_solve_rows = rows_by_base.get(MICRO_SOLVE_BENCHMARK, [])
    micro_solve_median = next(
        (
            row
            for row in micro_solve_rows
            if row.get("aggregate_name") in {None, "median"}
        ),
        None,
    )
    if micro_solve_median is None:
        errors.append(f"missing {MICRO_SOLVE_BENCHMARK} median row")
    else:
        errors.extend(_micro_solve_counter_errors(micro_solve_median))

    if errors:
        raise SystemExit("\n".join(errors))

    print(
        "ABD comparison packet OK: {} rows validated in {}".format(
            ", ".join(sorted(seen)), path
        )
    )


def main() -> int:
    args = parse_args()
    if not args.skip_run:
        run_benchmark(args)
    validate_packet(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
