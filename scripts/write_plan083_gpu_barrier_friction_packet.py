#!/usr/bin/env python3
"""Run and validate PLAN-083's private GPU barrier/friction local packet."""

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
    ".benchmark_results/plan083/gpu/barrier_friction_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/gpu/barrier_friction_parity.json"
)

DEFAULT_SAMPLE_COUNT = 65536
DEFAULT_TOLERANCE = 1e-10
DEFAULT_SPEEDUP_GATE = 1.25
REQUIRED_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "solve",
    "device_to_host",
    "readback",
}


class Plan083GpuBarrierFrictionPacketError(RuntimeError):
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
        "--sample-count",
        type=int,
        default=DEFAULT_SAMPLE_COUNT,
        help="Representative barrier/friction local sample count.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="Maximum CPU/GPU local-kernel absolute error.",
    )
    parser.add_argument(
        "--speedup-gate",
        type=float,
        default=DEFAULT_SPEEDUP_GATE,
        help="Policy speedup gate recorded in the packet; parity does not require it.",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Validate an existing Google Benchmark JSON file.",
    )
    return parser.parse_args(argv)


def run_benchmark(args: argparse.Namespace) -> None:
    args.benchmark_json.parent.mkdir(parents=True, exist_ok=True)
    filter_expr = (
        "^BM_Plan083("
        "BarrierFrictionLocal(Cpu|Cuda)"
        "|PointTriangleBarrierGradient(Cpu|Cuda)"
        f")/{args.sample_count}(/real_time)?$"
    )
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_gpu_barrier_friction",
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
        raise Plan083GpuBarrierFrictionPacketError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise Plan083GpuBarrierFrictionPacketError(
            f"{path}: JSON root must be an object"
        )
    return data


def _finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _counter(row: Mapping[str, Any], key: str) -> float:
    value = _finite_number(row.get(key))
    if value is None:
        raise Plan083GpuBarrierFrictionPacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _representative_rows(
    rows: list[Mapping[str, Any]], sample_count: int
) -> dict[str, Mapping[str, Any]]:
    expected_names = {
        "scalar_cpu": f"BM_Plan083BarrierFrictionLocalCpu/{sample_count}",
        "scalar_gpu": f"BM_Plan083BarrierFrictionLocalCuda/{sample_count}",
        "point_triangle_cpu": (
            f"BM_Plan083PointTriangleBarrierGradientCpu/{sample_count}"
        ),
        "point_triangle_gpu": (
            f"BM_Plan083PointTriangleBarrierGradientCuda/{sample_count}"
        ),
    }
    expected_by_name = {name: key for key, name in expected_names.items()}
    found: dict[str, Mapping[str, Any]] = {}
    errors: list[str] = []

    for row in rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical not in expected_by_name:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found[expected_by_name[canonical]] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for key, expected in expected_names.items():
        if key not in found:
            errors.append(f"missing median benchmark row: {expected}")

    if errors:
        raise Plan083GpuBarrierFrictionPacketError("\n".join(errors))

    return found


def _timing_ns(row: Mapping[str, Any]) -> dict[str, float]:
    timing_ns = {
        "setup": _counter(row, "host_setup_ns"),
        "host_to_device": _counter(row, "host_to_device_ns"),
        "kernel": _counter(row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = REQUIRED_TIMING_KEYS - timing_ns.keys()
    if missing:
        raise Plan083GpuBarrierFrictionPacketError(
            f"packet timing is missing {sorted(missing)}"
        )
    return timing_ns


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    sample_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise Plan083GpuBarrierFrictionPacketError(
            "benchmark JSON has no benchmark rows"
        )
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise Plan083GpuBarrierFrictionPacketError("benchmark JSON has non-object rows")

    representative_rows = _representative_rows(typed_rows, sample_count)
    scalar_cpu_row = representative_rows["scalar_cpu"]
    scalar_gpu_row = representative_rows["scalar_gpu"]
    point_triangle_cpu_row = representative_rows["point_triangle_cpu"]
    point_triangle_gpu_row = representative_rows["point_triangle_gpu"]

    scalar_cpu_ns = benchmark_timing_ns(scalar_cpu_row)
    scalar_gpu_ns = benchmark_timing_ns(scalar_gpu_row)
    point_triangle_cpu_ns = benchmark_timing_ns(point_triangle_cpu_row)
    point_triangle_gpu_ns = benchmark_timing_ns(point_triangle_gpu_row)
    for label, timing in {
        "scalar CPU": scalar_cpu_ns,
        "scalar GPU": scalar_gpu_ns,
        "point-triangle CPU": point_triangle_cpu_ns,
        "point-triangle GPU": point_triangle_gpu_ns,
    }.items():
        if not math.isfinite(timing) or timing <= 0.0:
            raise Plan083GpuBarrierFrictionPacketError(
                f"{label} benchmark timing is not positive"
            )

    scalar_max_error = _counter(scalar_gpu_row, "max_result_abs_error")
    point_triangle_max_error = _counter(point_triangle_gpu_row, "max_result_abs_error")
    max_error = max(scalar_max_error, point_triangle_max_error)
    if max_error > tolerance:
        raise Plan083GpuBarrierFrictionPacketError(
            f"local-kernel max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    count_pairs = {
        "active_barriers": "gpu_active_barriers",
        "active_friction": "gpu_active_friction",
        "dynamic_friction": "gpu_dynamic_friction",
    }
    counts: dict[str, int] = {}
    for cpu_key, gpu_key in count_pairs.items():
        cpu_count = int(_counter(scalar_cpu_row, cpu_key))
        gpu_count = int(_counter(scalar_gpu_row, gpu_key))
        if cpu_count != gpu_count:
            raise Plan083GpuBarrierFrictionPacketError(
                f"{cpu_key} count {cpu_count} != {gpu_key} count {gpu_count}"
            )
        counts[cpu_key] = cpu_count

    point_triangle_cpu_active = int(_counter(point_triangle_cpu_row, "active_barriers"))
    point_triangle_gpu_active = int(
        _counter(point_triangle_gpu_row, "gpu_active_barriers")
    )
    if point_triangle_cpu_active != point_triangle_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-triangle active_barriers count "
            f"{point_triangle_cpu_active} != gpu_active_barriers count "
            f"{point_triangle_gpu_active}"
        )

    sample_rows = {
        "scalar CPU": scalar_cpu_row,
        "scalar GPU": scalar_gpu_row,
        "point-triangle CPU": point_triangle_cpu_row,
        "point-triangle GPU": point_triangle_gpu_row,
    }
    for label, row in sample_rows.items():
        row_samples = int(_counter(row, "samples"))
        if row_samples != sample_count:
            raise Plan083GpuBarrierFrictionPacketError(
                f"expected {sample_count} samples, got {label}={row_samples}"
            )

    scalar_speedup = scalar_cpu_ns / scalar_gpu_ns
    point_triangle_speedup = point_triangle_cpu_ns / point_triangle_gpu_ns
    speedup = min(scalar_speedup, point_triangle_speedup)
    scalar_timing_ns = _timing_ns(scalar_gpu_row)
    point_triangle_timing_ns = _timing_ns(point_triangle_gpu_row)

    cpu_samples = int(_counter(scalar_cpu_row, "samples"))
    gpu_samples = int(_counter(scalar_gpu_row, "samples"))
    if cpu_samples != sample_count or gpu_samples != sample_count:
        raise Plan083GpuBarrierFrictionPacketError(
            f"expected {sample_count} samples, got CPU={cpu_samples}, GPU={gpu_samples}"
        )

    return {
        "plan083_gpu_barrier_friction_packet": {
            "row_id": "barrier-friction-local-kernels",
            "same_scene_cpu_gpu": True,
            "sample_count": sample_count,
            "active_barrier_count": counts["active_barriers"],
            "active_friction_count": counts["active_friction"],
            "dynamic_friction_count": counts["dynamic_friction"],
            "max_barrier_value": _counter(scalar_cpu_row, "max_barrier_value"),
            "max_friction_work": _counter(scalar_cpu_row, "max_friction_work"),
            "max_result_abs_error": max_error,
            "result_abs_error_tolerance": tolerance,
            "speedup": speedup,
            "speedup_gate": speedup_gate,
            "meets_speedup_gate": speedup >= speedup_gate,
            "timing_ns": scalar_timing_ns,
            "cpu_benchmark_row": _packet_row_name(scalar_cpu_row),
            "gpu_benchmark_row": _packet_row_name(scalar_gpu_row),
            "scalar_local": {
                "sample_count": sample_count,
                "active_barrier_count": counts["active_barriers"],
                "active_friction_count": counts["active_friction"],
                "dynamic_friction_count": counts["dynamic_friction"],
                "max_barrier_value": _counter(scalar_cpu_row, "max_barrier_value"),
                "max_friction_work": _counter(scalar_cpu_row, "max_friction_work"),
                "max_result_abs_error": scalar_max_error,
                "speedup": scalar_speedup,
                "meets_speedup_gate": scalar_speedup >= speedup_gate,
                "timing_ns": scalar_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scalar_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scalar_gpu_row),
            },
            "point_triangle_barrier_gradient": {
                "sample_count": sample_count,
                "active_barrier_count": point_triangle_cpu_active,
                "max_barrier_value": _counter(
                    point_triangle_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": point_triangle_max_error,
                "speedup": point_triangle_speedup,
                "meets_speedup_gate": point_triangle_speedup >= speedup_gate,
                "timing_ns": point_triangle_timing_ns,
                "cpu_benchmark_row": _packet_row_name(point_triangle_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_triangle_gpu_row),
            },
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
            sample_count=args.sample_count,
            tolerance=args.tolerance,
            speedup_gate=args.speedup_gate,
        )
    except Plan083GpuBarrierFrictionPacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_gpu_barrier_friction_packet"]
    print(
        "PLAN-083 GPU barrier/friction packet OK: "
        f"samples={row['sample_count']} max_error={row['max_result_abs_error']:.3g} "
        f"speedup={row['speedup']:.3f}x "
        f"meets_gate={row['meets_speedup_gate']}"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
