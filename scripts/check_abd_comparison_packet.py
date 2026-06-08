#!/usr/bin/env python3
"""Run and validate the first PLAN-083 ABD comparison packet."""

from __future__ import annotations

import argparse
import json
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
    "BM_AffineBodyRigidIpcPointTriangleOracle",
    "BM_AffineBodyOrthogonalityEnergy",
}

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


def validate_packet(path: Path) -> None:
    with path.open(encoding="utf-8") as stream:
        payload = json.load(stream)

    rows = payload.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise SystemExit(f"{path} has no benchmark rows")

    seen: set[str] = set()
    errors: list[str] = []
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
        errors.extend(benchmark_timing_field_errors(row, name))

    missing = sorted(EXPECTED_BENCHMARKS - seen)
    if missing:
        errors.append("missing benchmark rows: {}".format(", ".join(missing)))

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
