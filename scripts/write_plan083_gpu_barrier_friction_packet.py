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
        "|PointTriangleBarrierHessian(Cpu|Cuda)"
        "|SceneRuntimePointTriangleBarrierHessian(Cpu|Cuda)"
        "|SceneRuntimePointEdgeBarrierHessian(Cpu|Cuda)"
        "|SceneRuntimePointPointBarrierHessian(Cpu|Cuda)"
        "|SceneRuntimeEdgeEdgeBarrierHessian(Cpu|Cuda)"
        "|SceneRuntimeCombinedBarrierHessian(Cpu|Cuda)"
        "|PointTriangleBarrierHessianPsd(Cpu|Cuda)"
        "|PointPointBarrierHessian(Cpu|Cuda)"
        "|PointPointBarrierHessianPsd(Cpu|Cuda)"
        "|PointEdgeBarrierHessian(Cpu|Cuda)"
        "|PointEdgeBarrierHessianPsd(Cpu|Cuda)"
        "|EdgeEdgeBarrierHessian(Cpu|Cuda)"
        "|EdgeEdgeBarrierHessianPsd(Cpu|Cuda)"
        "|PointTriangleTangentStencil(Cpu|Cuda)"
        "|EdgeEdgeTangentStencil(Cpu|Cuda)"
        "|PointEdgeTangentStencil(Cpu|Cuda)"
        "|PointPointTangentStencil(Cpu|Cuda)"
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
        "point_triangle_hessian_cpu": (
            f"BM_Plan083PointTriangleBarrierHessianCpu/{sample_count}"
        ),
        "point_triangle_hessian_gpu": (
            f"BM_Plan083PointTriangleBarrierHessianCuda/{sample_count}"
        ),
        "scene_runtime_point_triangle_hessian_cpu": (
            f"BM_Plan083SceneRuntimePointTriangleBarrierHessianCpu/{sample_count}"
        ),
        "scene_runtime_point_triangle_hessian_gpu": (
            f"BM_Plan083SceneRuntimePointTriangleBarrierHessianCuda/{sample_count}"
        ),
        "scene_runtime_point_edge_hessian_cpu": (
            f"BM_Plan083SceneRuntimePointEdgeBarrierHessianCpu/{sample_count}"
        ),
        "scene_runtime_point_edge_hessian_gpu": (
            f"BM_Plan083SceneRuntimePointEdgeBarrierHessianCuda/{sample_count}"
        ),
        "scene_runtime_point_point_hessian_cpu": (
            f"BM_Plan083SceneRuntimePointPointBarrierHessianCpu/{sample_count}"
        ),
        "scene_runtime_point_point_hessian_gpu": (
            f"BM_Plan083SceneRuntimePointPointBarrierHessianCuda/{sample_count}"
        ),
        "scene_runtime_edge_edge_hessian_cpu": (
            f"BM_Plan083SceneRuntimeEdgeEdgeBarrierHessianCpu/{sample_count}"
        ),
        "scene_runtime_edge_edge_hessian_gpu": (
            f"BM_Plan083SceneRuntimeEdgeEdgeBarrierHessianCuda/{sample_count}"
        ),
        "scene_runtime_combined_hessian_cpu": (
            f"BM_Plan083SceneRuntimeCombinedBarrierHessianCpu/{sample_count}"
        ),
        "scene_runtime_combined_hessian_gpu": (
            f"BM_Plan083SceneRuntimeCombinedBarrierHessianCuda/{sample_count}"
        ),
        "point_triangle_hessian_psd_cpu": (
            f"BM_Plan083PointTriangleBarrierHessianPsdCpu/{sample_count}"
        ),
        "point_triangle_hessian_psd_gpu": (
            f"BM_Plan083PointTriangleBarrierHessianPsdCuda/{sample_count}"
        ),
        "point_point_hessian_cpu": (
            f"BM_Plan083PointPointBarrierHessianCpu/{sample_count}"
        ),
        "point_point_hessian_gpu": (
            f"BM_Plan083PointPointBarrierHessianCuda/{sample_count}"
        ),
        "point_point_hessian_psd_cpu": (
            f"BM_Plan083PointPointBarrierHessianPsdCpu/{sample_count}"
        ),
        "point_point_hessian_psd_gpu": (
            f"BM_Plan083PointPointBarrierHessianPsdCuda/{sample_count}"
        ),
        "point_edge_hessian_cpu": (
            f"BM_Plan083PointEdgeBarrierHessianCpu/{sample_count}"
        ),
        "point_edge_hessian_gpu": (
            f"BM_Plan083PointEdgeBarrierHessianCuda/{sample_count}"
        ),
        "point_edge_hessian_psd_cpu": (
            f"BM_Plan083PointEdgeBarrierHessianPsdCpu/{sample_count}"
        ),
        "point_edge_hessian_psd_gpu": (
            f"BM_Plan083PointEdgeBarrierHessianPsdCuda/{sample_count}"
        ),
        "edge_edge_hessian_cpu": (
            f"BM_Plan083EdgeEdgeBarrierHessianCpu/{sample_count}"
        ),
        "edge_edge_hessian_gpu": (
            f"BM_Plan083EdgeEdgeBarrierHessianCuda/{sample_count}"
        ),
        "edge_edge_hessian_psd_cpu": (
            f"BM_Plan083EdgeEdgeBarrierHessianPsdCpu/{sample_count}"
        ),
        "edge_edge_hessian_psd_gpu": (
            f"BM_Plan083EdgeEdgeBarrierHessianPsdCuda/{sample_count}"
        ),
        "point_triangle_tangent_cpu": (
            f"BM_Plan083PointTriangleTangentStencilCpu/{sample_count}"
        ),
        "point_triangle_tangent_gpu": (
            f"BM_Plan083PointTriangleTangentStencilCuda/{sample_count}"
        ),
        "edge_edge_tangent_cpu": (
            f"BM_Plan083EdgeEdgeTangentStencilCpu/{sample_count}"
        ),
        "edge_edge_tangent_gpu": (
            f"BM_Plan083EdgeEdgeTangentStencilCuda/{sample_count}"
        ),
        "point_edge_tangent_cpu": (
            f"BM_Plan083PointEdgeTangentStencilCpu/{sample_count}"
        ),
        "point_edge_tangent_gpu": (
            f"BM_Plan083PointEdgeTangentStencilCuda/{sample_count}"
        ),
        "point_point_tangent_cpu": (
            f"BM_Plan083PointPointTangentStencilCpu/{sample_count}"
        ),
        "point_point_tangent_gpu": (
            f"BM_Plan083PointPointTangentStencilCuda/{sample_count}"
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
    point_triangle_hessian_cpu_row = representative_rows["point_triangle_hessian_cpu"]
    point_triangle_hessian_gpu_row = representative_rows["point_triangle_hessian_gpu"]
    scene_runtime_point_triangle_hessian_cpu_row = representative_rows[
        "scene_runtime_point_triangle_hessian_cpu"
    ]
    scene_runtime_point_triangle_hessian_gpu_row = representative_rows[
        "scene_runtime_point_triangle_hessian_gpu"
    ]
    scene_runtime_point_edge_hessian_cpu_row = representative_rows[
        "scene_runtime_point_edge_hessian_cpu"
    ]
    scene_runtime_point_edge_hessian_gpu_row = representative_rows[
        "scene_runtime_point_edge_hessian_gpu"
    ]
    scene_runtime_point_point_hessian_cpu_row = representative_rows[
        "scene_runtime_point_point_hessian_cpu"
    ]
    scene_runtime_point_point_hessian_gpu_row = representative_rows[
        "scene_runtime_point_point_hessian_gpu"
    ]
    scene_runtime_edge_edge_hessian_cpu_row = representative_rows[
        "scene_runtime_edge_edge_hessian_cpu"
    ]
    scene_runtime_edge_edge_hessian_gpu_row = representative_rows[
        "scene_runtime_edge_edge_hessian_gpu"
    ]
    scene_runtime_combined_hessian_cpu_row = representative_rows[
        "scene_runtime_combined_hessian_cpu"
    ]
    scene_runtime_combined_hessian_gpu_row = representative_rows[
        "scene_runtime_combined_hessian_gpu"
    ]
    point_triangle_hessian_psd_cpu_row = representative_rows[
        "point_triangle_hessian_psd_cpu"
    ]
    point_triangle_hessian_psd_gpu_row = representative_rows[
        "point_triangle_hessian_psd_gpu"
    ]
    point_point_hessian_cpu_row = representative_rows["point_point_hessian_cpu"]
    point_point_hessian_gpu_row = representative_rows["point_point_hessian_gpu"]
    point_point_hessian_psd_cpu_row = representative_rows["point_point_hessian_psd_cpu"]
    point_point_hessian_psd_gpu_row = representative_rows["point_point_hessian_psd_gpu"]
    point_edge_hessian_cpu_row = representative_rows["point_edge_hessian_cpu"]
    point_edge_hessian_gpu_row = representative_rows["point_edge_hessian_gpu"]
    point_edge_hessian_psd_cpu_row = representative_rows["point_edge_hessian_psd_cpu"]
    point_edge_hessian_psd_gpu_row = representative_rows["point_edge_hessian_psd_gpu"]
    edge_edge_hessian_cpu_row = representative_rows["edge_edge_hessian_cpu"]
    edge_edge_hessian_gpu_row = representative_rows["edge_edge_hessian_gpu"]
    edge_edge_hessian_psd_cpu_row = representative_rows["edge_edge_hessian_psd_cpu"]
    edge_edge_hessian_psd_gpu_row = representative_rows["edge_edge_hessian_psd_gpu"]
    point_triangle_tangent_cpu_row = representative_rows["point_triangle_tangent_cpu"]
    point_triangle_tangent_gpu_row = representative_rows["point_triangle_tangent_gpu"]
    edge_edge_tangent_cpu_row = representative_rows["edge_edge_tangent_cpu"]
    edge_edge_tangent_gpu_row = representative_rows["edge_edge_tangent_gpu"]
    point_edge_tangent_cpu_row = representative_rows["point_edge_tangent_cpu"]
    point_edge_tangent_gpu_row = representative_rows["point_edge_tangent_gpu"]
    point_point_tangent_cpu_row = representative_rows["point_point_tangent_cpu"]
    point_point_tangent_gpu_row = representative_rows["point_point_tangent_gpu"]

    scalar_cpu_ns = benchmark_timing_ns(scalar_cpu_row)
    scalar_gpu_ns = benchmark_timing_ns(scalar_gpu_row)
    point_triangle_cpu_ns = benchmark_timing_ns(point_triangle_cpu_row)
    point_triangle_gpu_ns = benchmark_timing_ns(point_triangle_gpu_row)
    point_triangle_hessian_cpu_ns = benchmark_timing_ns(point_triangle_hessian_cpu_row)
    point_triangle_hessian_gpu_ns = benchmark_timing_ns(point_triangle_hessian_gpu_row)
    scene_runtime_point_triangle_hessian_cpu_ns = benchmark_timing_ns(
        scene_runtime_point_triangle_hessian_cpu_row
    )
    scene_runtime_point_triangle_hessian_gpu_ns = benchmark_timing_ns(
        scene_runtime_point_triangle_hessian_gpu_row
    )
    scene_runtime_point_edge_hessian_cpu_ns = benchmark_timing_ns(
        scene_runtime_point_edge_hessian_cpu_row
    )
    scene_runtime_point_edge_hessian_gpu_ns = benchmark_timing_ns(
        scene_runtime_point_edge_hessian_gpu_row
    )
    scene_runtime_point_point_hessian_cpu_ns = benchmark_timing_ns(
        scene_runtime_point_point_hessian_cpu_row
    )
    scene_runtime_point_point_hessian_gpu_ns = benchmark_timing_ns(
        scene_runtime_point_point_hessian_gpu_row
    )
    scene_runtime_edge_edge_hessian_cpu_ns = benchmark_timing_ns(
        scene_runtime_edge_edge_hessian_cpu_row
    )
    scene_runtime_edge_edge_hessian_gpu_ns = benchmark_timing_ns(
        scene_runtime_edge_edge_hessian_gpu_row
    )
    scene_runtime_combined_hessian_cpu_ns = benchmark_timing_ns(
        scene_runtime_combined_hessian_cpu_row
    )
    scene_runtime_combined_hessian_gpu_ns = benchmark_timing_ns(
        scene_runtime_combined_hessian_gpu_row
    )
    point_triangle_hessian_psd_cpu_ns = benchmark_timing_ns(
        point_triangle_hessian_psd_cpu_row
    )
    point_triangle_hessian_psd_gpu_ns = benchmark_timing_ns(
        point_triangle_hessian_psd_gpu_row
    )
    point_point_hessian_cpu_ns = benchmark_timing_ns(point_point_hessian_cpu_row)
    point_point_hessian_gpu_ns = benchmark_timing_ns(point_point_hessian_gpu_row)
    point_point_hessian_psd_cpu_ns = benchmark_timing_ns(
        point_point_hessian_psd_cpu_row
    )
    point_point_hessian_psd_gpu_ns = benchmark_timing_ns(
        point_point_hessian_psd_gpu_row
    )
    point_edge_hessian_cpu_ns = benchmark_timing_ns(point_edge_hessian_cpu_row)
    point_edge_hessian_gpu_ns = benchmark_timing_ns(point_edge_hessian_gpu_row)
    point_edge_hessian_psd_cpu_ns = benchmark_timing_ns(point_edge_hessian_psd_cpu_row)
    point_edge_hessian_psd_gpu_ns = benchmark_timing_ns(point_edge_hessian_psd_gpu_row)
    edge_edge_hessian_cpu_ns = benchmark_timing_ns(edge_edge_hessian_cpu_row)
    edge_edge_hessian_gpu_ns = benchmark_timing_ns(edge_edge_hessian_gpu_row)
    edge_edge_hessian_psd_cpu_ns = benchmark_timing_ns(edge_edge_hessian_psd_cpu_row)
    edge_edge_hessian_psd_gpu_ns = benchmark_timing_ns(edge_edge_hessian_psd_gpu_row)
    point_triangle_tangent_cpu_ns = benchmark_timing_ns(point_triangle_tangent_cpu_row)
    point_triangle_tangent_gpu_ns = benchmark_timing_ns(point_triangle_tangent_gpu_row)
    edge_edge_tangent_cpu_ns = benchmark_timing_ns(edge_edge_tangent_cpu_row)
    edge_edge_tangent_gpu_ns = benchmark_timing_ns(edge_edge_tangent_gpu_row)
    point_edge_tangent_cpu_ns = benchmark_timing_ns(point_edge_tangent_cpu_row)
    point_edge_tangent_gpu_ns = benchmark_timing_ns(point_edge_tangent_gpu_row)
    point_point_tangent_cpu_ns = benchmark_timing_ns(point_point_tangent_cpu_row)
    point_point_tangent_gpu_ns = benchmark_timing_ns(point_point_tangent_gpu_row)
    for label, timing in {
        "scalar CPU": scalar_cpu_ns,
        "scalar GPU": scalar_gpu_ns,
        "point-triangle CPU": point_triangle_cpu_ns,
        "point-triangle GPU": point_triangle_gpu_ns,
        "point-triangle Hessian CPU": point_triangle_hessian_cpu_ns,
        "point-triangle Hessian GPU": point_triangle_hessian_gpu_ns,
        "scene-runtime point-triangle Hessian CPU": (
            scene_runtime_point_triangle_hessian_cpu_ns
        ),
        "scene-runtime point-triangle Hessian GPU": (
            scene_runtime_point_triangle_hessian_gpu_ns
        ),
        "scene-runtime point-edge Hessian CPU": scene_runtime_point_edge_hessian_cpu_ns,
        "scene-runtime point-edge Hessian GPU": scene_runtime_point_edge_hessian_gpu_ns,
        "scene-runtime point-point Hessian CPU": (
            scene_runtime_point_point_hessian_cpu_ns
        ),
        "scene-runtime point-point Hessian GPU": (
            scene_runtime_point_point_hessian_gpu_ns
        ),
        "scene-runtime edge-edge Hessian CPU": (scene_runtime_edge_edge_hessian_cpu_ns),
        "scene-runtime edge-edge Hessian GPU": (scene_runtime_edge_edge_hessian_gpu_ns),
        "scene-runtime combined Hessian CPU": scene_runtime_combined_hessian_cpu_ns,
        "scene-runtime combined Hessian GPU": scene_runtime_combined_hessian_gpu_ns,
        "point-triangle Hessian PSD CPU": point_triangle_hessian_psd_cpu_ns,
        "point-triangle Hessian PSD GPU": point_triangle_hessian_psd_gpu_ns,
        "point-point Hessian CPU": point_point_hessian_cpu_ns,
        "point-point Hessian GPU": point_point_hessian_gpu_ns,
        "point-point Hessian PSD CPU": point_point_hessian_psd_cpu_ns,
        "point-point Hessian PSD GPU": point_point_hessian_psd_gpu_ns,
        "point-edge Hessian CPU": point_edge_hessian_cpu_ns,
        "point-edge Hessian GPU": point_edge_hessian_gpu_ns,
        "point-edge Hessian PSD CPU": point_edge_hessian_psd_cpu_ns,
        "point-edge Hessian PSD GPU": point_edge_hessian_psd_gpu_ns,
        "edge-edge Hessian CPU": edge_edge_hessian_cpu_ns,
        "edge-edge Hessian GPU": edge_edge_hessian_gpu_ns,
        "edge-edge Hessian PSD CPU": edge_edge_hessian_psd_cpu_ns,
        "edge-edge Hessian PSD GPU": edge_edge_hessian_psd_gpu_ns,
        "point-triangle tangent CPU": point_triangle_tangent_cpu_ns,
        "point-triangle tangent GPU": point_triangle_tangent_gpu_ns,
        "edge-edge tangent CPU": edge_edge_tangent_cpu_ns,
        "edge-edge tangent GPU": edge_edge_tangent_gpu_ns,
        "point-edge tangent CPU": point_edge_tangent_cpu_ns,
        "point-edge tangent GPU": point_edge_tangent_gpu_ns,
        "point-point tangent CPU": point_point_tangent_cpu_ns,
        "point-point tangent GPU": point_point_tangent_gpu_ns,
    }.items():
        if not math.isfinite(timing) or timing <= 0.0:
            raise Plan083GpuBarrierFrictionPacketError(
                f"{label} benchmark timing is not positive"
            )

    scalar_max_error = _counter(scalar_gpu_row, "max_result_abs_error")
    point_triangle_max_error = _counter(point_triangle_gpu_row, "max_result_abs_error")
    point_triangle_hessian_max_error = _counter(
        point_triangle_hessian_gpu_row, "max_result_abs_error"
    )
    scene_runtime_point_triangle_hessian_max_error = _counter(
        scene_runtime_point_triangle_hessian_gpu_row, "max_result_abs_error"
    )
    scene_runtime_point_edge_hessian_max_error = _counter(
        scene_runtime_point_edge_hessian_gpu_row, "max_result_abs_error"
    )
    scene_runtime_point_point_hessian_max_error = _counter(
        scene_runtime_point_point_hessian_gpu_row, "max_result_abs_error"
    )
    scene_runtime_edge_edge_hessian_max_error = _counter(
        scene_runtime_edge_edge_hessian_gpu_row, "max_result_abs_error"
    )
    scene_runtime_combined_hessian_max_error = _counter(
        scene_runtime_combined_hessian_gpu_row, "max_result_abs_error"
    )
    point_triangle_hessian_psd_max_error = _counter(
        point_triangle_hessian_psd_gpu_row, "max_result_abs_error"
    )
    point_point_hessian_max_error = _counter(
        point_point_hessian_gpu_row, "max_result_abs_error"
    )
    point_point_hessian_psd_max_error = _counter(
        point_point_hessian_psd_gpu_row, "max_result_abs_error"
    )
    point_edge_hessian_max_error = _counter(
        point_edge_hessian_gpu_row, "max_result_abs_error"
    )
    point_edge_hessian_psd_max_error = _counter(
        point_edge_hessian_psd_gpu_row, "max_result_abs_error"
    )
    edge_edge_hessian_max_error = _counter(
        edge_edge_hessian_gpu_row, "max_result_abs_error"
    )
    edge_edge_hessian_psd_max_error = _counter(
        edge_edge_hessian_psd_gpu_row, "max_result_abs_error"
    )
    point_triangle_tangent_max_error = _counter(
        point_triangle_tangent_gpu_row, "max_result_abs_error"
    )
    edge_edge_tangent_max_error = _counter(
        edge_edge_tangent_gpu_row, "max_result_abs_error"
    )
    point_edge_tangent_max_error = _counter(
        point_edge_tangent_gpu_row, "max_result_abs_error"
    )
    point_point_tangent_max_error = _counter(
        point_point_tangent_gpu_row, "max_result_abs_error"
    )
    max_error = max(
        scalar_max_error,
        point_triangle_max_error,
        point_triangle_hessian_max_error,
        scene_runtime_point_triangle_hessian_max_error,
        scene_runtime_point_edge_hessian_max_error,
        scene_runtime_point_point_hessian_max_error,
        scene_runtime_edge_edge_hessian_max_error,
        scene_runtime_combined_hessian_max_error,
        point_triangle_hessian_psd_max_error,
        point_point_hessian_max_error,
        point_point_hessian_psd_max_error,
        point_edge_hessian_max_error,
        point_edge_hessian_psd_max_error,
        edge_edge_hessian_max_error,
        edge_edge_hessian_psd_max_error,
        point_triangle_tangent_max_error,
        edge_edge_tangent_max_error,
        point_edge_tangent_max_error,
        point_point_tangent_max_error,
    )
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

    point_triangle_hessian_cpu_active = int(
        _counter(point_triangle_hessian_cpu_row, "active_barriers")
    )
    point_triangle_hessian_gpu_active = int(
        _counter(point_triangle_hessian_gpu_row, "gpu_active_barriers")
    )
    if point_triangle_hessian_cpu_active != point_triangle_hessian_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-triangle Hessian active_barriers count "
            f"{point_triangle_hessian_cpu_active} != gpu_active_barriers count "
            f"{point_triangle_hessian_gpu_active}"
        )

    scene_runtime_point_triangle_hessian_cpu_active = int(
        _counter(scene_runtime_point_triangle_hessian_cpu_row, "active_barriers")
    )
    scene_runtime_point_triangle_hessian_gpu_active = int(
        _counter(scene_runtime_point_triangle_hessian_gpu_row, "gpu_active_barriers")
    )
    if (
        scene_runtime_point_triangle_hessian_cpu_active
        != scene_runtime_point_triangle_hessian_gpu_active
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-triangle Hessian active_barriers count "
            f"{scene_runtime_point_triangle_hessian_cpu_active} "
            "!= gpu_active_barriers count "
            f"{scene_runtime_point_triangle_hessian_gpu_active}"
        )
    scene_runtime_point_triangle_candidates = int(
        _counter(
            scene_runtime_point_triangle_hessian_cpu_row,
            "runtime_point_triangle_candidates",
        )
    )
    if scene_runtime_point_triangle_candidates <= 0:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-triangle Hessian row has no candidates"
        )
    scene_runtime_gpu_candidates = int(
        _counter(
            scene_runtime_point_triangle_hessian_gpu_row,
            "runtime_point_triangle_candidates",
        )
    )
    if scene_runtime_point_triangle_candidates != scene_runtime_gpu_candidates:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-triangle candidate count "
            f"{scene_runtime_point_triangle_candidates} != "
            f"{scene_runtime_gpu_candidates}"
        )
    scene_runtime_body_count = int(
        _counter(scene_runtime_point_triangle_hessian_cpu_row, "scene_bodies")
    )
    scene_runtime_gpu_body_count = int(
        _counter(scene_runtime_point_triangle_hessian_gpu_row, "scene_bodies")
    )
    if scene_runtime_body_count != scene_runtime_gpu_body_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-triangle scene_bodies count "
            f"{scene_runtime_body_count} != {scene_runtime_gpu_body_count}"
        )
    scene_runtime_node_count = int(
        _counter(scene_runtime_point_triangle_hessian_cpu_row, "scene_nodes")
    )
    scene_runtime_gpu_node_count = int(
        _counter(scene_runtime_point_triangle_hessian_gpu_row, "scene_nodes")
    )
    scene_runtime_triangle_count = int(
        _counter(scene_runtime_point_triangle_hessian_cpu_row, "scene_triangles")
    )
    scene_runtime_gpu_triangle_count = int(
        _counter(scene_runtime_point_triangle_hessian_gpu_row, "scene_triangles")
    )
    if scene_runtime_node_count != scene_runtime_gpu_node_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-triangle scene_nodes count "
            f"{scene_runtime_node_count} != {scene_runtime_gpu_node_count}"
        )
    if scene_runtime_triangle_count != scene_runtime_gpu_triangle_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-triangle scene_triangles count "
            f"{scene_runtime_triangle_count} != "
            f"{scene_runtime_gpu_triangle_count}"
        )

    scene_runtime_point_edge_hessian_cpu_active = int(
        _counter(scene_runtime_point_edge_hessian_cpu_row, "active_barriers")
    )
    scene_runtime_point_edge_hessian_gpu_active = int(
        _counter(scene_runtime_point_edge_hessian_gpu_row, "gpu_active_barriers")
    )
    if (
        scene_runtime_point_edge_hessian_cpu_active
        != scene_runtime_point_edge_hessian_gpu_active
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge Hessian active_barriers count "
            f"{scene_runtime_point_edge_hessian_cpu_active} "
            "!= gpu_active_barriers count "
            f"{scene_runtime_point_edge_hessian_gpu_active}"
        )
    scene_runtime_point_edge_candidates = int(
        _counter(
            scene_runtime_point_edge_hessian_cpu_row,
            "runtime_point_edge_candidates",
        )
    )
    if scene_runtime_point_edge_candidates <= 0:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge Hessian row has no candidates"
        )
    scene_runtime_point_edge_gpu_candidates = int(
        _counter(
            scene_runtime_point_edge_hessian_gpu_row,
            "runtime_point_edge_candidates",
        )
    )
    if scene_runtime_point_edge_candidates != scene_runtime_point_edge_gpu_candidates:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge candidate count "
            f"{scene_runtime_point_edge_candidates} != "
            f"{scene_runtime_point_edge_gpu_candidates}"
        )
    scene_runtime_point_edge_source_candidates = int(
        _counter(
            scene_runtime_point_edge_hessian_cpu_row,
            "source_point_triangle_candidates",
        )
    )
    scene_runtime_point_edge_gpu_source_candidates = int(
        _counter(
            scene_runtime_point_edge_hessian_gpu_row,
            "source_point_triangle_candidates",
        )
    )
    if (
        scene_runtime_point_edge_source_candidates
        != scene_runtime_point_edge_gpu_source_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge source candidate count "
            f"{scene_runtime_point_edge_source_candidates} != "
            f"{scene_runtime_point_edge_gpu_source_candidates}"
        )
    if (
        scene_runtime_point_edge_source_candidates
        != scene_runtime_point_triangle_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge source point-triangle count "
            f"{scene_runtime_point_edge_source_candidates} != "
            f"{scene_runtime_point_triangle_candidates}"
        )
    scene_runtime_point_edge_body_count = int(
        _counter(scene_runtime_point_edge_hessian_cpu_row, "scene_bodies")
    )
    scene_runtime_point_edge_gpu_body_count = int(
        _counter(scene_runtime_point_edge_hessian_gpu_row, "scene_bodies")
    )
    if scene_runtime_point_edge_body_count != scene_runtime_point_edge_gpu_body_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge scene_bodies count "
            f"{scene_runtime_point_edge_body_count} != "
            f"{scene_runtime_point_edge_gpu_body_count}"
        )
    if scene_runtime_point_edge_body_count != scene_runtime_body_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge scene_bodies count "
            f"{scene_runtime_point_edge_body_count} != "
            f"{scene_runtime_body_count}"
        )
    scene_runtime_point_edge_node_count = int(
        _counter(scene_runtime_point_edge_hessian_cpu_row, "scene_nodes")
    )
    scene_runtime_point_edge_gpu_node_count = int(
        _counter(scene_runtime_point_edge_hessian_gpu_row, "scene_nodes")
    )
    scene_runtime_point_edge_triangle_count = int(
        _counter(scene_runtime_point_edge_hessian_cpu_row, "scene_triangles")
    )
    scene_runtime_point_edge_gpu_triangle_count = int(
        _counter(scene_runtime_point_edge_hessian_gpu_row, "scene_triangles")
    )
    if scene_runtime_point_edge_node_count != scene_runtime_point_edge_gpu_node_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge scene_nodes count "
            f"{scene_runtime_point_edge_node_count} != "
            f"{scene_runtime_point_edge_gpu_node_count}"
        )
    if (
        scene_runtime_point_edge_triangle_count
        != scene_runtime_point_edge_gpu_triangle_count
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge scene_triangles count "
            f"{scene_runtime_point_edge_triangle_count} != "
            f"{scene_runtime_point_edge_gpu_triangle_count}"
        )
    if (
        scene_runtime_point_edge_node_count != scene_runtime_node_count
        or scene_runtime_point_edge_triangle_count != scene_runtime_triangle_count
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-edge scene topology does not match "
            "the point-triangle runtime row"
        )

    scene_runtime_point_point_hessian_cpu_active = int(
        _counter(scene_runtime_point_point_hessian_cpu_row, "active_barriers")
    )
    scene_runtime_point_point_hessian_gpu_active = int(
        _counter(scene_runtime_point_point_hessian_gpu_row, "gpu_active_barriers")
    )
    if (
        scene_runtime_point_point_hessian_cpu_active
        != scene_runtime_point_point_hessian_gpu_active
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point Hessian active_barriers count "
            f"{scene_runtime_point_point_hessian_cpu_active} "
            "!= gpu_active_barriers count "
            f"{scene_runtime_point_point_hessian_gpu_active}"
        )
    scene_runtime_point_point_candidates = int(
        _counter(
            scene_runtime_point_point_hessian_cpu_row,
            "runtime_point_point_candidates",
        )
    )
    if scene_runtime_point_point_candidates <= 0:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point Hessian row has no candidates"
        )
    scene_runtime_point_point_gpu_candidates = int(
        _counter(
            scene_runtime_point_point_hessian_gpu_row,
            "runtime_point_point_candidates",
        )
    )
    if scene_runtime_point_point_candidates != scene_runtime_point_point_gpu_candidates:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point candidate count "
            f"{scene_runtime_point_point_candidates} != "
            f"{scene_runtime_point_point_gpu_candidates}"
        )
    scene_runtime_point_point_source_candidates = int(
        _counter(
            scene_runtime_point_point_hessian_cpu_row,
            "source_point_triangle_candidates",
        )
    )
    scene_runtime_point_point_gpu_source_candidates = int(
        _counter(
            scene_runtime_point_point_hessian_gpu_row,
            "source_point_triangle_candidates",
        )
    )
    if (
        scene_runtime_point_point_source_candidates
        != scene_runtime_point_point_gpu_source_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point source candidate count "
            f"{scene_runtime_point_point_source_candidates} != "
            f"{scene_runtime_point_point_gpu_source_candidates}"
        )
    if (
        scene_runtime_point_point_source_candidates
        != scene_runtime_point_triangle_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point source point-triangle count "
            f"{scene_runtime_point_point_source_candidates} != "
            f"{scene_runtime_point_triangle_candidates}"
        )
    if (
        scene_runtime_point_point_candidates
        != 3 * scene_runtime_point_point_source_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point candidate count "
            f"{scene_runtime_point_point_candidates} != 3 * "
            f"{scene_runtime_point_point_source_candidates}"
        )
    scene_runtime_point_point_body_count = int(
        _counter(scene_runtime_point_point_hessian_cpu_row, "scene_bodies")
    )
    scene_runtime_point_point_gpu_body_count = int(
        _counter(scene_runtime_point_point_hessian_gpu_row, "scene_bodies")
    )
    if scene_runtime_point_point_body_count != scene_runtime_point_point_gpu_body_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point scene_bodies count "
            f"{scene_runtime_point_point_body_count} != "
            f"{scene_runtime_point_point_gpu_body_count}"
        )
    if scene_runtime_point_point_body_count != scene_runtime_body_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point scene_bodies count "
            f"{scene_runtime_point_point_body_count} != "
            f"{scene_runtime_body_count}"
        )
    scene_runtime_point_point_node_count = int(
        _counter(scene_runtime_point_point_hessian_cpu_row, "scene_nodes")
    )
    scene_runtime_point_point_gpu_node_count = int(
        _counter(scene_runtime_point_point_hessian_gpu_row, "scene_nodes")
    )
    scene_runtime_point_point_triangle_count = int(
        _counter(scene_runtime_point_point_hessian_cpu_row, "scene_triangles")
    )
    scene_runtime_point_point_gpu_triangle_count = int(
        _counter(scene_runtime_point_point_hessian_gpu_row, "scene_triangles")
    )
    if scene_runtime_point_point_node_count != scene_runtime_point_point_gpu_node_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point scene_nodes count "
            f"{scene_runtime_point_point_node_count} != "
            f"{scene_runtime_point_point_gpu_node_count}"
        )
    if (
        scene_runtime_point_point_triangle_count
        != scene_runtime_point_point_gpu_triangle_count
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point scene_triangles count "
            f"{scene_runtime_point_point_triangle_count} != "
            f"{scene_runtime_point_point_gpu_triangle_count}"
        )
    if (
        scene_runtime_point_point_node_count != scene_runtime_node_count
        or scene_runtime_point_point_triangle_count != scene_runtime_triangle_count
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime point-point scene topology does not match "
            "the point-triangle runtime row"
        )

    scene_runtime_edge_edge_hessian_cpu_active = int(
        _counter(scene_runtime_edge_edge_hessian_cpu_row, "active_barriers")
    )
    scene_runtime_edge_edge_hessian_gpu_active = int(
        _counter(scene_runtime_edge_edge_hessian_gpu_row, "gpu_active_barriers")
    )
    if (
        scene_runtime_edge_edge_hessian_cpu_active
        != scene_runtime_edge_edge_hessian_gpu_active
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge Hessian active_barriers count "
            f"{scene_runtime_edge_edge_hessian_cpu_active} "
            "!= gpu_active_barriers count "
            f"{scene_runtime_edge_edge_hessian_gpu_active}"
        )
    scene_runtime_edge_edge_candidates = int(
        _counter(
            scene_runtime_edge_edge_hessian_cpu_row,
            "runtime_edge_edge_candidates",
        )
    )
    if scene_runtime_edge_edge_candidates <= 0:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge Hessian row has no candidates"
        )
    scene_runtime_edge_edge_gpu_candidates = int(
        _counter(
            scene_runtime_edge_edge_hessian_gpu_row,
            "runtime_edge_edge_candidates",
        )
    )
    if scene_runtime_edge_edge_candidates != scene_runtime_edge_edge_gpu_candidates:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge candidate count "
            f"{scene_runtime_edge_edge_candidates} != "
            f"{scene_runtime_edge_edge_gpu_candidates}"
        )
    scene_runtime_edge_edge_source_candidates = int(
        _counter(
            scene_runtime_edge_edge_hessian_cpu_row,
            "source_edge_edge_candidates",
        )
    )
    scene_runtime_edge_edge_gpu_source_candidates = int(
        _counter(
            scene_runtime_edge_edge_hessian_gpu_row,
            "source_edge_edge_candidates",
        )
    )
    if (
        scene_runtime_edge_edge_source_candidates
        != scene_runtime_edge_edge_gpu_source_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge source candidate count "
            f"{scene_runtime_edge_edge_source_candidates} != "
            f"{scene_runtime_edge_edge_gpu_source_candidates}"
        )
    if scene_runtime_edge_edge_candidates != scene_runtime_edge_edge_source_candidates:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge candidate count "
            f"{scene_runtime_edge_edge_candidates} != "
            f"{scene_runtime_edge_edge_source_candidates}"
        )
    scene_runtime_edge_edge_body_count = int(
        _counter(scene_runtime_edge_edge_hessian_cpu_row, "scene_bodies")
    )
    scene_runtime_edge_edge_gpu_body_count = int(
        _counter(scene_runtime_edge_edge_hessian_gpu_row, "scene_bodies")
    )
    if scene_runtime_edge_edge_body_count != scene_runtime_edge_edge_gpu_body_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge scene_bodies count "
            f"{scene_runtime_edge_edge_body_count} != "
            f"{scene_runtime_edge_edge_gpu_body_count}"
        )
    if scene_runtime_edge_edge_body_count != scene_runtime_body_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge scene_bodies count "
            f"{scene_runtime_edge_edge_body_count} != "
            f"{scene_runtime_body_count}"
        )
    scene_runtime_edge_edge_node_count = int(
        _counter(scene_runtime_edge_edge_hessian_cpu_row, "scene_nodes")
    )
    scene_runtime_edge_edge_gpu_node_count = int(
        _counter(scene_runtime_edge_edge_hessian_gpu_row, "scene_nodes")
    )
    scene_runtime_edge_edge_triangle_count = int(
        _counter(scene_runtime_edge_edge_hessian_cpu_row, "scene_triangles")
    )
    scene_runtime_edge_edge_gpu_triangle_count = int(
        _counter(scene_runtime_edge_edge_hessian_gpu_row, "scene_triangles")
    )
    if scene_runtime_edge_edge_node_count != scene_runtime_edge_edge_gpu_node_count:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge scene_nodes count "
            f"{scene_runtime_edge_edge_node_count} != "
            f"{scene_runtime_edge_edge_gpu_node_count}"
        )
    if (
        scene_runtime_edge_edge_triangle_count
        != scene_runtime_edge_edge_gpu_triangle_count
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge scene_triangles count "
            f"{scene_runtime_edge_edge_triangle_count} != "
            f"{scene_runtime_edge_edge_gpu_triangle_count}"
        )
    if (
        scene_runtime_edge_edge_node_count != scene_runtime_node_count
        or scene_runtime_edge_edge_triangle_count != scene_runtime_triangle_count
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime edge-edge scene topology does not match "
            "the point-triangle runtime row"
        )

    scene_runtime_combined_cpu_active = int(
        _counter(scene_runtime_combined_hessian_cpu_row, "active_barriers")
    )
    scene_runtime_combined_gpu_active = int(
        _counter(scene_runtime_combined_hessian_gpu_row, "gpu_active_barriers")
    )
    if scene_runtime_combined_cpu_active != scene_runtime_combined_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime combined Hessian active_barriers count "
            f"{scene_runtime_combined_cpu_active} != gpu_active_barriers count "
            f"{scene_runtime_combined_gpu_active}"
        )
    scene_runtime_combined_point_triangle_candidates = int(
        _counter(
            scene_runtime_combined_hessian_cpu_row,
            "runtime_point_triangle_candidates",
        )
    )
    scene_runtime_combined_point_edge_candidates = int(
        _counter(
            scene_runtime_combined_hessian_cpu_row,
            "runtime_point_edge_candidates",
        )
    )
    scene_runtime_combined_point_point_candidates = int(
        _counter(
            scene_runtime_combined_hessian_cpu_row,
            "runtime_point_point_candidates",
        )
    )
    scene_runtime_combined_edge_edge_candidates = int(
        _counter(
            scene_runtime_combined_hessian_cpu_row,
            "runtime_edge_edge_candidates",
        )
    )
    scene_runtime_combined_total_candidates = int(
        _counter(scene_runtime_combined_hessian_cpu_row, "samples")
    )
    scene_runtime_combined_gpu_total_candidates = int(
        _counter(scene_runtime_combined_hessian_gpu_row, "samples")
    )
    if (
        scene_runtime_combined_total_candidates
        != scene_runtime_combined_gpu_total_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime combined total candidate count "
            f"{scene_runtime_combined_total_candidates} != "
            f"{scene_runtime_combined_gpu_total_candidates}"
        )
    if (
        scene_runtime_combined_total_candidates
        != scene_runtime_combined_point_triangle_candidates
        + scene_runtime_combined_point_edge_candidates
        + scene_runtime_combined_point_point_candidates
        + scene_runtime_combined_edge_edge_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime combined total candidate count does not match "
            "the primitive-family candidate sum"
        )
    combined_candidate_pairs = {
        "runtime_point_triangle_candidates": (
            scene_runtime_point_triangle_candidates,
            scene_runtime_combined_point_triangle_candidates,
        ),
        "runtime_point_edge_candidates": (
            scene_runtime_point_edge_candidates,
            scene_runtime_combined_point_edge_candidates,
        ),
        "runtime_point_point_candidates": (
            scene_runtime_point_point_candidates,
            scene_runtime_combined_point_point_candidates,
        ),
        "runtime_edge_edge_candidates": (
            scene_runtime_edge_edge_candidates,
            scene_runtime_combined_edge_edge_candidates,
        ),
    }
    for label, (expected, actual) in combined_candidate_pairs.items():
        if expected != actual:
            raise Plan083GpuBarrierFrictionPacketError(
                f"scene-runtime combined {label} count {actual} != {expected}"
            )
        gpu_actual = int(_counter(scene_runtime_combined_hessian_gpu_row, label))
        if actual != gpu_actual:
            raise Plan083GpuBarrierFrictionPacketError(
                f"scene-runtime combined {label} CPU/GPU count "
                f"{actual} != {gpu_actual}"
            )
    scene_runtime_combined_source_point_triangle_candidates = int(
        _counter(
            scene_runtime_combined_hessian_cpu_row,
            "source_point_triangle_candidates",
        )
    )
    scene_runtime_combined_source_edge_edge_candidates = int(
        _counter(scene_runtime_combined_hessian_cpu_row, "source_edge_edge_candidates")
    )
    if (
        scene_runtime_combined_source_point_triangle_candidates
        != scene_runtime_point_triangle_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime combined source point-triangle candidate count "
            f"{scene_runtime_combined_source_point_triangle_candidates} != "
            f"{scene_runtime_point_triangle_candidates}"
        )
    if (
        scene_runtime_combined_source_edge_edge_candidates
        != scene_runtime_edge_edge_candidates
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime combined source edge-edge candidate count "
            f"{scene_runtime_combined_source_edge_edge_candidates} != "
            f"{scene_runtime_edge_edge_candidates}"
        )
    scene_runtime_combined_body_count = int(
        _counter(scene_runtime_combined_hessian_cpu_row, "scene_bodies")
    )
    scene_runtime_combined_node_count = int(
        _counter(scene_runtime_combined_hessian_cpu_row, "scene_nodes")
    )
    scene_runtime_combined_triangle_count = int(
        _counter(scene_runtime_combined_hessian_cpu_row, "scene_triangles")
    )
    if (
        scene_runtime_combined_body_count != scene_runtime_body_count
        or scene_runtime_combined_node_count != scene_runtime_node_count
        or scene_runtime_combined_triangle_count != scene_runtime_triangle_count
    ):
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime combined scene topology does not match "
            "the per-family runtime rows"
        )
    for label in ("scene_bodies", "scene_nodes", "scene_triangles"):
        cpu_count = int(_counter(scene_runtime_combined_hessian_cpu_row, label))
        gpu_count = int(_counter(scene_runtime_combined_hessian_gpu_row, label))
        if cpu_count != gpu_count:
            raise Plan083GpuBarrierFrictionPacketError(
                f"scene-runtime combined {label} count {cpu_count} != {gpu_count}"
            )
    combined_active_pairs = {
        "point_triangle_active_barriers": scene_runtime_point_triangle_hessian_cpu_active,
        "point_edge_active_barriers": scene_runtime_point_edge_hessian_cpu_active,
        "point_point_active_barriers": scene_runtime_point_point_hessian_cpu_active,
        "edge_edge_active_barriers": scene_runtime_edge_edge_hessian_cpu_active,
    }
    combined_active_total = 0
    for label, expected in combined_active_pairs.items():
        cpu_count = int(_counter(scene_runtime_combined_hessian_cpu_row, label))
        gpu_count = int(
            _counter(scene_runtime_combined_hessian_gpu_row, f"gpu_{label}")
        )
        if cpu_count != expected:
            raise Plan083GpuBarrierFrictionPacketError(
                f"scene-runtime combined {label} count {cpu_count} != {expected}"
            )
        if cpu_count != gpu_count:
            raise Plan083GpuBarrierFrictionPacketError(
                f"scene-runtime combined {label} CPU/GPU count "
                f"{cpu_count} != {gpu_count}"
            )
        combined_active_total += cpu_count
    if scene_runtime_combined_cpu_active != combined_active_total:
        raise Plan083GpuBarrierFrictionPacketError(
            "scene-runtime combined active_barriers total "
            f"{scene_runtime_combined_cpu_active} != {combined_active_total}"
        )

    point_triangle_hessian_psd_cpu_active = int(
        _counter(point_triangle_hessian_psd_cpu_row, "active_barriers")
    )
    point_triangle_hessian_psd_gpu_active = int(
        _counter(point_triangle_hessian_psd_gpu_row, "gpu_active_barriers")
    )
    if point_triangle_hessian_psd_cpu_active != point_triangle_hessian_psd_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-triangle Hessian PSD active_barriers count "
            f"{point_triangle_hessian_psd_cpu_active} != gpu_active_barriers count "
            f"{point_triangle_hessian_psd_gpu_active}"
        )

    point_point_hessian_cpu_active = int(
        _counter(point_point_hessian_cpu_row, "active_barriers")
    )
    point_point_hessian_gpu_active = int(
        _counter(point_point_hessian_gpu_row, "gpu_active_barriers")
    )
    if point_point_hessian_cpu_active != point_point_hessian_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-point Hessian active_barriers count "
            f"{point_point_hessian_cpu_active} != gpu_active_barriers count "
            f"{point_point_hessian_gpu_active}"
        )

    point_point_hessian_psd_cpu_active = int(
        _counter(point_point_hessian_psd_cpu_row, "active_barriers")
    )
    point_point_hessian_psd_gpu_active = int(
        _counter(point_point_hessian_psd_gpu_row, "gpu_active_barriers")
    )
    if point_point_hessian_psd_cpu_active != point_point_hessian_psd_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-point Hessian PSD active_barriers count "
            f"{point_point_hessian_psd_cpu_active} != gpu_active_barriers count "
            f"{point_point_hessian_psd_gpu_active}"
        )

    point_edge_hessian_cpu_active = int(
        _counter(point_edge_hessian_cpu_row, "active_barriers")
    )
    point_edge_hessian_gpu_active = int(
        _counter(point_edge_hessian_gpu_row, "gpu_active_barriers")
    )
    if point_edge_hessian_cpu_active != point_edge_hessian_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-edge Hessian active_barriers count "
            f"{point_edge_hessian_cpu_active} != gpu_active_barriers count "
            f"{point_edge_hessian_gpu_active}"
        )

    point_edge_hessian_psd_cpu_active = int(
        _counter(point_edge_hessian_psd_cpu_row, "active_barriers")
    )
    point_edge_hessian_psd_gpu_active = int(
        _counter(point_edge_hessian_psd_gpu_row, "gpu_active_barriers")
    )
    if point_edge_hessian_psd_cpu_active != point_edge_hessian_psd_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-edge Hessian PSD active_barriers count "
            f"{point_edge_hessian_psd_cpu_active} != gpu_active_barriers count "
            f"{point_edge_hessian_psd_gpu_active}"
        )

    edge_edge_hessian_cpu_active = int(
        _counter(edge_edge_hessian_cpu_row, "active_barriers")
    )
    edge_edge_hessian_gpu_active = int(
        _counter(edge_edge_hessian_gpu_row, "gpu_active_barriers")
    )
    if edge_edge_hessian_cpu_active != edge_edge_hessian_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "edge-edge Hessian active_barriers count "
            f"{edge_edge_hessian_cpu_active} != gpu_active_barriers count "
            f"{edge_edge_hessian_gpu_active}"
        )

    edge_edge_hessian_psd_cpu_active = int(
        _counter(edge_edge_hessian_psd_cpu_row, "active_barriers")
    )
    edge_edge_hessian_psd_gpu_active = int(
        _counter(edge_edge_hessian_psd_gpu_row, "gpu_active_barriers")
    )
    if edge_edge_hessian_psd_cpu_active != edge_edge_hessian_psd_gpu_active:
        raise Plan083GpuBarrierFrictionPacketError(
            "edge-edge Hessian PSD active_barriers count "
            f"{edge_edge_hessian_psd_cpu_active} != gpu_active_barriers count "
            f"{edge_edge_hessian_psd_gpu_active}"
        )

    point_triangle_tangent_cpu_fallbacks = int(
        _counter(point_triangle_tangent_cpu_row, "fallback_bases")
    )
    point_triangle_tangent_gpu_fallbacks = int(
        _counter(point_triangle_tangent_gpu_row, "gpu_fallback_bases")
    )
    if point_triangle_tangent_cpu_fallbacks != point_triangle_tangent_gpu_fallbacks:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-triangle fallback_bases count "
            f"{point_triangle_tangent_cpu_fallbacks} != gpu_fallback_bases count "
            f"{point_triangle_tangent_gpu_fallbacks}"
        )

    edge_edge_tangent_cpu_fallbacks = int(
        _counter(edge_edge_tangent_cpu_row, "fallback_bases")
    )
    edge_edge_tangent_gpu_fallbacks = int(
        _counter(edge_edge_tangent_gpu_row, "gpu_fallback_bases")
    )
    if edge_edge_tangent_cpu_fallbacks != edge_edge_tangent_gpu_fallbacks:
        raise Plan083GpuBarrierFrictionPacketError(
            "edge-edge fallback_bases count "
            f"{edge_edge_tangent_cpu_fallbacks} != gpu_fallback_bases count "
            f"{edge_edge_tangent_gpu_fallbacks}"
        )

    point_edge_tangent_cpu_fallbacks = int(
        _counter(point_edge_tangent_cpu_row, "fallback_bases")
    )
    point_edge_tangent_gpu_fallbacks = int(
        _counter(point_edge_tangent_gpu_row, "gpu_fallback_bases")
    )
    if point_edge_tangent_cpu_fallbacks != point_edge_tangent_gpu_fallbacks:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-edge fallback_bases count "
            f"{point_edge_tangent_cpu_fallbacks} != gpu_fallback_bases count "
            f"{point_edge_tangent_gpu_fallbacks}"
        )

    point_point_tangent_cpu_fallbacks = int(
        _counter(point_point_tangent_cpu_row, "fallback_bases")
    )
    point_point_tangent_gpu_fallbacks = int(
        _counter(point_point_tangent_gpu_row, "gpu_fallback_bases")
    )
    if point_point_tangent_cpu_fallbacks != point_point_tangent_gpu_fallbacks:
        raise Plan083GpuBarrierFrictionPacketError(
            "point-point fallback_bases count "
            f"{point_point_tangent_cpu_fallbacks} != gpu_fallback_bases count "
            f"{point_point_tangent_gpu_fallbacks}"
        )

    sample_rows = {
        "scalar CPU": scalar_cpu_row,
        "scalar GPU": scalar_gpu_row,
        "point-triangle CPU": point_triangle_cpu_row,
        "point-triangle GPU": point_triangle_gpu_row,
        "point-triangle Hessian CPU": point_triangle_hessian_cpu_row,
        "point-triangle Hessian GPU": point_triangle_hessian_gpu_row,
        "point-triangle Hessian PSD CPU": point_triangle_hessian_psd_cpu_row,
        "point-triangle Hessian PSD GPU": point_triangle_hessian_psd_gpu_row,
        "point-point Hessian CPU": point_point_hessian_cpu_row,
        "point-point Hessian GPU": point_point_hessian_gpu_row,
        "point-point Hessian PSD CPU": point_point_hessian_psd_cpu_row,
        "point-point Hessian PSD GPU": point_point_hessian_psd_gpu_row,
        "point-edge Hessian CPU": point_edge_hessian_cpu_row,
        "point-edge Hessian GPU": point_edge_hessian_gpu_row,
        "point-edge Hessian PSD CPU": point_edge_hessian_psd_cpu_row,
        "point-edge Hessian PSD GPU": point_edge_hessian_psd_gpu_row,
        "edge-edge Hessian CPU": edge_edge_hessian_cpu_row,
        "edge-edge Hessian GPU": edge_edge_hessian_gpu_row,
        "edge-edge Hessian PSD CPU": edge_edge_hessian_psd_cpu_row,
        "edge-edge Hessian PSD GPU": edge_edge_hessian_psd_gpu_row,
        "point-triangle tangent CPU": point_triangle_tangent_cpu_row,
        "point-triangle tangent GPU": point_triangle_tangent_gpu_row,
        "edge-edge tangent CPU": edge_edge_tangent_cpu_row,
        "edge-edge tangent GPU": edge_edge_tangent_gpu_row,
        "point-edge tangent CPU": point_edge_tangent_cpu_row,
        "point-edge tangent GPU": point_edge_tangent_gpu_row,
        "point-point tangent CPU": point_point_tangent_cpu_row,
        "point-point tangent GPU": point_point_tangent_gpu_row,
    }
    for label, row in sample_rows.items():
        row_samples = int(_counter(row, "samples"))
        if row_samples != sample_count:
            raise Plan083GpuBarrierFrictionPacketError(
                f"expected {sample_count} samples, got {label}={row_samples}"
            )

    scalar_speedup = scalar_cpu_ns / scalar_gpu_ns
    point_triangle_speedup = point_triangle_cpu_ns / point_triangle_gpu_ns
    point_triangle_hessian_speedup = (
        point_triangle_hessian_cpu_ns / point_triangle_hessian_gpu_ns
    )
    scene_runtime_point_triangle_hessian_speedup = (
        scene_runtime_point_triangle_hessian_cpu_ns
        / scene_runtime_point_triangle_hessian_gpu_ns
    )
    scene_runtime_point_edge_hessian_speedup = (
        scene_runtime_point_edge_hessian_cpu_ns
        / scene_runtime_point_edge_hessian_gpu_ns
    )
    scene_runtime_point_point_hessian_speedup = (
        scene_runtime_point_point_hessian_cpu_ns
        / scene_runtime_point_point_hessian_gpu_ns
    )
    scene_runtime_edge_edge_hessian_speedup = (
        scene_runtime_edge_edge_hessian_cpu_ns / scene_runtime_edge_edge_hessian_gpu_ns
    )
    scene_runtime_combined_hessian_speedup = (
        scene_runtime_combined_hessian_cpu_ns / scene_runtime_combined_hessian_gpu_ns
    )
    point_triangle_hessian_psd_speedup = (
        point_triangle_hessian_psd_cpu_ns / point_triangle_hessian_psd_gpu_ns
    )
    point_point_hessian_speedup = (
        point_point_hessian_cpu_ns / point_point_hessian_gpu_ns
    )
    point_point_hessian_psd_speedup = (
        point_point_hessian_psd_cpu_ns / point_point_hessian_psd_gpu_ns
    )
    point_edge_hessian_speedup = point_edge_hessian_cpu_ns / point_edge_hessian_gpu_ns
    point_edge_hessian_psd_speedup = (
        point_edge_hessian_psd_cpu_ns / point_edge_hessian_psd_gpu_ns
    )
    edge_edge_hessian_speedup = edge_edge_hessian_cpu_ns / edge_edge_hessian_gpu_ns
    edge_edge_hessian_psd_speedup = (
        edge_edge_hessian_psd_cpu_ns / edge_edge_hessian_psd_gpu_ns
    )
    point_triangle_tangent_speedup = (
        point_triangle_tangent_cpu_ns / point_triangle_tangent_gpu_ns
    )
    edge_edge_tangent_speedup = edge_edge_tangent_cpu_ns / edge_edge_tangent_gpu_ns
    point_edge_tangent_speedup = point_edge_tangent_cpu_ns / point_edge_tangent_gpu_ns
    point_point_tangent_speedup = (
        point_point_tangent_cpu_ns / point_point_tangent_gpu_ns
    )
    speedup = min(
        scalar_speedup,
        point_triangle_speedup,
        point_triangle_hessian_speedup,
        scene_runtime_point_triangle_hessian_speedup,
        scene_runtime_point_edge_hessian_speedup,
        scene_runtime_point_point_hessian_speedup,
        scene_runtime_edge_edge_hessian_speedup,
        scene_runtime_combined_hessian_speedup,
        point_triangle_hessian_psd_speedup,
        point_point_hessian_speedup,
        point_point_hessian_psd_speedup,
        point_edge_hessian_speedup,
        point_edge_hessian_psd_speedup,
        edge_edge_hessian_speedup,
        edge_edge_hessian_psd_speedup,
        point_triangle_tangent_speedup,
        edge_edge_tangent_speedup,
        point_edge_tangent_speedup,
        point_point_tangent_speedup,
    )
    scalar_timing_ns = _timing_ns(scalar_gpu_row)
    point_triangle_timing_ns = _timing_ns(point_triangle_gpu_row)
    point_triangle_hessian_timing_ns = _timing_ns(point_triangle_hessian_gpu_row)
    scene_runtime_point_triangle_hessian_timing_ns = _timing_ns(
        scene_runtime_point_triangle_hessian_gpu_row
    )
    scene_runtime_point_edge_hessian_timing_ns = _timing_ns(
        scene_runtime_point_edge_hessian_gpu_row
    )
    scene_runtime_point_point_hessian_timing_ns = _timing_ns(
        scene_runtime_point_point_hessian_gpu_row
    )
    scene_runtime_edge_edge_hessian_timing_ns = _timing_ns(
        scene_runtime_edge_edge_hessian_gpu_row
    )
    scene_runtime_combined_hessian_timing_ns = _timing_ns(
        scene_runtime_combined_hessian_gpu_row
    )
    point_triangle_hessian_psd_timing_ns = _timing_ns(
        point_triangle_hessian_psd_gpu_row
    )
    point_point_hessian_timing_ns = _timing_ns(point_point_hessian_gpu_row)
    point_point_hessian_psd_timing_ns = _timing_ns(point_point_hessian_psd_gpu_row)
    point_edge_hessian_timing_ns = _timing_ns(point_edge_hessian_gpu_row)
    point_edge_hessian_psd_timing_ns = _timing_ns(point_edge_hessian_psd_gpu_row)
    edge_edge_hessian_timing_ns = _timing_ns(edge_edge_hessian_gpu_row)
    edge_edge_hessian_psd_timing_ns = _timing_ns(edge_edge_hessian_psd_gpu_row)
    point_triangle_tangent_timing_ns = _timing_ns(point_triangle_tangent_gpu_row)
    edge_edge_tangent_timing_ns = _timing_ns(edge_edge_tangent_gpu_row)
    point_edge_tangent_timing_ns = _timing_ns(point_edge_tangent_gpu_row)
    point_point_tangent_timing_ns = _timing_ns(point_point_tangent_gpu_row)

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
            "point_triangle_barrier_hessian": {
                "sample_count": sample_count,
                "active_barrier_count": point_triangle_hessian_cpu_active,
                "max_barrier_value": _counter(
                    point_triangle_hessian_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": point_triangle_hessian_max_error,
                "speedup": point_triangle_hessian_speedup,
                "meets_speedup_gate": point_triangle_hessian_speedup >= speedup_gate,
                "timing_ns": point_triangle_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(point_triangle_hessian_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_triangle_hessian_gpu_row),
            },
            "point_triangle_scene_runtime_barrier_hessian": {
                "candidate_count": scene_runtime_point_triangle_candidates,
                "active_barrier_count": (
                    scene_runtime_point_triangle_hessian_cpu_active
                ),
                "scene_body_count": scene_runtime_body_count,
                "scene_node_count": scene_runtime_node_count,
                "scene_triangle_count": scene_runtime_triangle_count,
                "max_barrier_value": _counter(
                    scene_runtime_point_triangle_hessian_cpu_row,
                    "max_barrier_value",
                ),
                "max_result_abs_error": (
                    scene_runtime_point_triangle_hessian_max_error
                ),
                "speedup": scene_runtime_point_triangle_hessian_speedup,
                "meets_speedup_gate": (
                    scene_runtime_point_triangle_hessian_speedup >= speedup_gate
                ),
                "timing_ns": scene_runtime_point_triangle_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_runtime_point_triangle_hessian_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_runtime_point_triangle_hessian_gpu_row
                ),
            },
            "point_edge_scene_runtime_barrier_hessian": {
                "candidate_count": scene_runtime_point_edge_candidates,
                "source_point_triangle_candidate_count": (
                    scene_runtime_point_edge_source_candidates
                ),
                "active_barrier_count": scene_runtime_point_edge_hessian_cpu_active,
                "scene_body_count": scene_runtime_point_edge_body_count,
                "scene_node_count": scene_runtime_point_edge_node_count,
                "scene_triangle_count": scene_runtime_point_edge_triangle_count,
                "max_barrier_value": _counter(
                    scene_runtime_point_edge_hessian_cpu_row,
                    "max_barrier_value",
                ),
                "max_result_abs_error": scene_runtime_point_edge_hessian_max_error,
                "speedup": scene_runtime_point_edge_hessian_speedup,
                "meets_speedup_gate": (
                    scene_runtime_point_edge_hessian_speedup >= speedup_gate
                ),
                "timing_ns": scene_runtime_point_edge_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_runtime_point_edge_hessian_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_runtime_point_edge_hessian_gpu_row
                ),
            },
            "point_point_scene_runtime_barrier_hessian": {
                "candidate_count": scene_runtime_point_point_candidates,
                "source_point_triangle_candidate_count": (
                    scene_runtime_point_point_source_candidates
                ),
                "active_barrier_count": (scene_runtime_point_point_hessian_cpu_active),
                "scene_body_count": scene_runtime_point_point_body_count,
                "scene_node_count": scene_runtime_point_point_node_count,
                "scene_triangle_count": scene_runtime_point_point_triangle_count,
                "max_barrier_value": _counter(
                    scene_runtime_point_point_hessian_cpu_row,
                    "max_barrier_value",
                ),
                "max_result_abs_error": scene_runtime_point_point_hessian_max_error,
                "speedup": scene_runtime_point_point_hessian_speedup,
                "meets_speedup_gate": (
                    scene_runtime_point_point_hessian_speedup >= speedup_gate
                ),
                "timing_ns": scene_runtime_point_point_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_runtime_point_point_hessian_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_runtime_point_point_hessian_gpu_row
                ),
            },
            "edge_edge_scene_runtime_barrier_hessian": {
                "candidate_count": scene_runtime_edge_edge_candidates,
                "source_edge_edge_candidate_count": (
                    scene_runtime_edge_edge_source_candidates
                ),
                "active_barrier_count": scene_runtime_edge_edge_hessian_cpu_active,
                "scene_body_count": scene_runtime_edge_edge_body_count,
                "scene_node_count": scene_runtime_edge_edge_node_count,
                "scene_triangle_count": scene_runtime_edge_edge_triangle_count,
                "max_barrier_value": _counter(
                    scene_runtime_edge_edge_hessian_cpu_row,
                    "max_barrier_value",
                ),
                "max_result_abs_error": scene_runtime_edge_edge_hessian_max_error,
                "speedup": scene_runtime_edge_edge_hessian_speedup,
                "meets_speedup_gate": (
                    scene_runtime_edge_edge_hessian_speedup >= speedup_gate
                ),
                "timing_ns": scene_runtime_edge_edge_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_runtime_edge_edge_hessian_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_runtime_edge_edge_hessian_gpu_row
                ),
            },
            "combined_scene_runtime_barrier_hessian": {
                "candidate_count": scene_runtime_combined_total_candidates,
                "source_point_triangle_candidate_count": (
                    scene_runtime_combined_source_point_triangle_candidates
                ),
                "source_edge_edge_candidate_count": (
                    scene_runtime_combined_source_edge_edge_candidates
                ),
                "point_triangle_candidate_count": (
                    scene_runtime_combined_point_triangle_candidates
                ),
                "point_edge_candidate_count": (
                    scene_runtime_combined_point_edge_candidates
                ),
                "point_point_candidate_count": (
                    scene_runtime_combined_point_point_candidates
                ),
                "edge_edge_candidate_count": (
                    scene_runtime_combined_edge_edge_candidates
                ),
                "active_barrier_count": scene_runtime_combined_cpu_active,
                "point_triangle_active_barrier_count": (
                    scene_runtime_point_triangle_hessian_cpu_active
                ),
                "point_edge_active_barrier_count": (
                    scene_runtime_point_edge_hessian_cpu_active
                ),
                "point_point_active_barrier_count": (
                    scene_runtime_point_point_hessian_cpu_active
                ),
                "edge_edge_active_barrier_count": (
                    scene_runtime_edge_edge_hessian_cpu_active
                ),
                "scene_body_count": scene_runtime_combined_body_count,
                "scene_node_count": scene_runtime_combined_node_count,
                "scene_triangle_count": scene_runtime_combined_triangle_count,
                "max_barrier_value": _counter(
                    scene_runtime_combined_hessian_cpu_row,
                    "max_barrier_value",
                ),
                "max_result_abs_error": scene_runtime_combined_hessian_max_error,
                "speedup": scene_runtime_combined_hessian_speedup,
                "meets_speedup_gate": (
                    scene_runtime_combined_hessian_speedup >= speedup_gate
                ),
                "timing_ns": scene_runtime_combined_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_runtime_combined_hessian_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_runtime_combined_hessian_gpu_row
                ),
            },
            "point_triangle_barrier_hessian_psd_projection": {
                "sample_count": sample_count,
                "active_barrier_count": point_triangle_hessian_psd_cpu_active,
                "max_barrier_value": _counter(
                    point_triangle_hessian_psd_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": point_triangle_hessian_psd_max_error,
                "speedup": point_triangle_hessian_psd_speedup,
                "meets_speedup_gate": (
                    point_triangle_hessian_psd_speedup >= speedup_gate
                ),
                "timing_ns": point_triangle_hessian_psd_timing_ns,
                "psd_projection_ns": _counter(
                    point_triangle_hessian_psd_gpu_row, "psd_projection_ns"
                ),
                "cpu_benchmark_row": _packet_row_name(
                    point_triangle_hessian_psd_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    point_triangle_hessian_psd_gpu_row
                ),
            },
            "point_point_barrier_hessian": {
                "sample_count": sample_count,
                "active_barrier_count": point_point_hessian_cpu_active,
                "max_barrier_value": _counter(
                    point_point_hessian_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": point_point_hessian_max_error,
                "speedup": point_point_hessian_speedup,
                "meets_speedup_gate": point_point_hessian_speedup >= speedup_gate,
                "timing_ns": point_point_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(point_point_hessian_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_point_hessian_gpu_row),
            },
            "point_point_barrier_hessian_psd_projection": {
                "sample_count": sample_count,
                "active_barrier_count": point_point_hessian_psd_cpu_active,
                "max_barrier_value": _counter(
                    point_point_hessian_psd_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": point_point_hessian_psd_max_error,
                "speedup": point_point_hessian_psd_speedup,
                "meets_speedup_gate": point_point_hessian_psd_speedup >= speedup_gate,
                "timing_ns": point_point_hessian_psd_timing_ns,
                "psd_projection_ns": _counter(
                    point_point_hessian_psd_gpu_row, "psd_projection_ns"
                ),
                "cpu_benchmark_row": _packet_row_name(point_point_hessian_psd_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_point_hessian_psd_gpu_row),
            },
            "point_edge_barrier_hessian": {
                "sample_count": sample_count,
                "active_barrier_count": point_edge_hessian_cpu_active,
                "max_barrier_value": _counter(
                    point_edge_hessian_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": point_edge_hessian_max_error,
                "speedup": point_edge_hessian_speedup,
                "meets_speedup_gate": point_edge_hessian_speedup >= speedup_gate,
                "timing_ns": point_edge_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(point_edge_hessian_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_edge_hessian_gpu_row),
            },
            "point_edge_barrier_hessian_psd_projection": {
                "sample_count": sample_count,
                "active_barrier_count": point_edge_hessian_psd_cpu_active,
                "max_barrier_value": _counter(
                    point_edge_hessian_psd_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": point_edge_hessian_psd_max_error,
                "speedup": point_edge_hessian_psd_speedup,
                "meets_speedup_gate": point_edge_hessian_psd_speedup >= speedup_gate,
                "timing_ns": point_edge_hessian_psd_timing_ns,
                "psd_projection_ns": _counter(
                    point_edge_hessian_psd_gpu_row, "psd_projection_ns"
                ),
                "cpu_benchmark_row": _packet_row_name(point_edge_hessian_psd_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_edge_hessian_psd_gpu_row),
            },
            "edge_edge_barrier_hessian": {
                "sample_count": sample_count,
                "active_barrier_count": edge_edge_hessian_cpu_active,
                "max_barrier_value": _counter(
                    edge_edge_hessian_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": edge_edge_hessian_max_error,
                "speedup": edge_edge_hessian_speedup,
                "meets_speedup_gate": edge_edge_hessian_speedup >= speedup_gate,
                "timing_ns": edge_edge_hessian_timing_ns,
                "cpu_benchmark_row": _packet_row_name(edge_edge_hessian_cpu_row),
                "gpu_benchmark_row": _packet_row_name(edge_edge_hessian_gpu_row),
            },
            "edge_edge_barrier_hessian_psd_projection": {
                "sample_count": sample_count,
                "active_barrier_count": edge_edge_hessian_psd_cpu_active,
                "max_barrier_value": _counter(
                    edge_edge_hessian_psd_cpu_row, "max_barrier_value"
                ),
                "max_result_abs_error": edge_edge_hessian_psd_max_error,
                "speedup": edge_edge_hessian_psd_speedup,
                "meets_speedup_gate": edge_edge_hessian_psd_speedup >= speedup_gate,
                "timing_ns": edge_edge_hessian_psd_timing_ns,
                "psd_projection_ns": _counter(
                    edge_edge_hessian_psd_gpu_row, "psd_projection_ns"
                ),
                "cpu_benchmark_row": _packet_row_name(edge_edge_hessian_psd_cpu_row),
                "gpu_benchmark_row": _packet_row_name(edge_edge_hessian_psd_gpu_row),
            },
            "point_triangle_tangent_stencil": {
                "sample_count": sample_count,
                "fallback_basis_count": point_triangle_tangent_cpu_fallbacks,
                "max_result_abs_error": point_triangle_tangent_max_error,
                "speedup": point_triangle_tangent_speedup,
                "meets_speedup_gate": point_triangle_tangent_speedup >= speedup_gate,
                "timing_ns": point_triangle_tangent_timing_ns,
                "cpu_benchmark_row": _packet_row_name(point_triangle_tangent_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_triangle_tangent_gpu_row),
            },
            "edge_edge_tangent_stencil": {
                "sample_count": sample_count,
                "fallback_basis_count": edge_edge_tangent_cpu_fallbacks,
                "max_result_abs_error": edge_edge_tangent_max_error,
                "speedup": edge_edge_tangent_speedup,
                "meets_speedup_gate": edge_edge_tangent_speedup >= speedup_gate,
                "timing_ns": edge_edge_tangent_timing_ns,
                "cpu_benchmark_row": _packet_row_name(edge_edge_tangent_cpu_row),
                "gpu_benchmark_row": _packet_row_name(edge_edge_tangent_gpu_row),
            },
            "point_edge_tangent_stencil": {
                "sample_count": sample_count,
                "fallback_basis_count": point_edge_tangent_cpu_fallbacks,
                "max_result_abs_error": point_edge_tangent_max_error,
                "speedup": point_edge_tangent_speedup,
                "meets_speedup_gate": point_edge_tangent_speedup >= speedup_gate,
                "timing_ns": point_edge_tangent_timing_ns,
                "cpu_benchmark_row": _packet_row_name(point_edge_tangent_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_edge_tangent_gpu_row),
            },
            "point_point_tangent_stencil": {
                "sample_count": sample_count,
                "fallback_basis_count": point_point_tangent_cpu_fallbacks,
                "max_result_abs_error": point_point_tangent_max_error,
                "speedup": point_point_tangent_speedup,
                "meets_speedup_gate": point_point_tangent_speedup >= speedup_gate,
                "timing_ns": point_point_tangent_timing_ns,
                "cpu_benchmark_row": _packet_row_name(point_point_tangent_cpu_row),
                "gpu_benchmark_row": _packet_row_name(point_point_tangent_gpu_row),
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
