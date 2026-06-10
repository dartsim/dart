#!/usr/bin/env python3
"""Run and validate PLAN-083's private GPU PSD projection packet."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from benchmark_packet_utils import (  # noqa: E402
    benchmark_row_name,
    benchmark_timing_field_errors,
    benchmark_timing_ns,
    canonical_benchmark_name,
)

DEFAULT_BENCHMARK_OUTPUT = Path(
    ".benchmark_results/plan083/gpu/psd_projection_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/gpu/psd_projection_parity.json"
)

DEFAULT_BLOCK_COUNT = 4096
DEFAULT_TOLERANCE = 1e-9
DEFAULT_MIN_SPEEDUP = 1.25
REQUIRED_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "solve",
    "device_to_host",
    "readback",
}


class Plan083GpuPsdPacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        default=DEFAULT_BENCHMARK_OUTPUT,
        help="Google Benchmark JSON path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_PACKET_OUTPUT,
        help="Validated packet output path.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type for the benchmark target.",
    )
    parser.add_argument(
        "--benchmark-min-time",
        default="0.01s",
        help="Google Benchmark --benchmark_min_time value.",
    )
    parser.add_argument(
        "--benchmark-repetitions",
        type=int,
        default=3,
        help="Google Benchmark --benchmark_repetitions value.",
    )
    parser.add_argument(
        "--block-count",
        type=int,
        default=DEFAULT_BLOCK_COUNT,
        help="Representative 12x12 block count for the packet row.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="Maximum CPU/GPU absolute error.",
    )
    parser.add_argument(
        "--min-speedup",
        type=float,
        default=DEFAULT_MIN_SPEEDUP,
        help="Minimum GPU speedup over the CPU row.",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Validate an existing Google Benchmark JSON file.",
    )
    return parser.parse_args(argv)


def run_benchmark(args: argparse.Namespace) -> None:
    args.benchmark_json.parent.mkdir(parents=True, exist_ok=True)
    filter_expr = f"^BM_Plan083PsdProjection(Cpu|Cuda)/{args.block_count}(/real_time)?$"
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_gpu_psd_projection",
        "--build-type",
        args.build_type,
        "--",
        f"--benchmark_filter={filter_expr}",
        f"--benchmark_min_time={args.benchmark_min_time}",
        f"--benchmark_repetitions={args.benchmark_repetitions}",
        "--benchmark_report_aggregates_only=true",
        f"--benchmark_out={args.benchmark_json.as_posix()}",
        "--benchmark_out_format=json",
    ]
    subprocess.run(command, check=True)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as stream:
            data = json.load(stream)
    except json.JSONDecodeError as exc:
        raise Plan083GpuPsdPacketError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise Plan083GpuPsdPacketError(f"{path}: JSON root must be an object")
    return data


def _finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _counter(row: Mapping[str, Any], key: str) -> float:
    value = _finite_number(row.get(key))
    if value is None:
        raise Plan083GpuPsdPacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _representative_rows(
    rows: list[Mapping[str, Any]], block_count: int
) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    cpu_name = f"BM_Plan083PsdProjectionCpu/{block_count}"
    gpu_name = f"BM_Plan083PsdProjectionCuda/{block_count}"
    found: dict[str, Mapping[str, Any]] = {}
    errors: list[str] = []

    for row in rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical not in {cpu_name, gpu_name}:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found[canonical] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for expected in (cpu_name, gpu_name):
        if expected not in found:
            errors.append(f"missing median benchmark row: {expected}")

    if errors:
        raise Plan083GpuPsdPacketError("\n".join(errors))

    return found[cpu_name], found[gpu_name]


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    block_count: int,
    tolerance: float,
    min_speedup: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise Plan083GpuPsdPacketError("benchmark JSON has no benchmark rows")
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise Plan083GpuPsdPacketError("benchmark JSON has non-object rows")

    cpu_row, gpu_row = _representative_rows(typed_rows, block_count)
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuPsdPacketError("CPU benchmark timing is not positive")
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuPsdPacketError("GPU benchmark timing is not positive")

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuPsdPacketError(
            f"PSD projection max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    speedup = cpu_ns / gpu_ns
    if speedup < min_speedup:
        raise Plan083GpuPsdPacketError(
            f"PSD projection speedup {speedup:.3f} is below required {min_speedup:.3f}"
        )

    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = REQUIRED_TIMING_KEYS - timing_ns.keys()
    if missing:
        raise Plan083GpuPsdPacketError(f"packet timing is missing {sorted(missing)}")

    return {
        "plan083_gpu_psd_projection_packet": {
            "row_id": "psd-projection",
            "same_scene_cpu_gpu": True,
            "block_dimension": int(_counter(gpu_row, "dimension")),
            "block_count": block_count,
            "max_result_abs_error": max_error,
            "result_abs_error_tolerance": tolerance,
            "speedup": speedup,
            "min_speedup": min_speedup,
            "timing_ns": timing_ns,
            "cpu_benchmark_row": _packet_row_name(cpu_row),
            "gpu_benchmark_row": _packet_row_name(gpu_row),
        },
        "benchmarks": rows,
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if not args.skip_run:
        run_benchmark(args)
    try:
        packet = make_packet(
            _load_json(args.benchmark_json),
            block_count=args.block_count,
            tolerance=args.tolerance,
            min_speedup=args.min_speedup,
        )
    except Plan083GpuPsdPacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_gpu_psd_projection_packet"]
    print(
        "PLAN-083 GPU PSD packet OK: "
        f"count={row['block_count']} max_error={row['max_result_abs_error']:.3g} "
        f"speedup={row['speedup']:.3f}x"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
