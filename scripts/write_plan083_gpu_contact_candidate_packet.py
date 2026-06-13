#!/usr/bin/env python3
"""Run and validate PLAN-083's private GPU contact-candidate packet."""

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
    ".benchmark_results/plan083/gpu/contact_candidates_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/gpu/contact_candidates_parity.json"
)

DEFAULT_STENCIL_COUNT = 65536
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


class Plan083GpuContactCandidatePacketError(RuntimeError):
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
        "--stencil-count",
        type=int,
        default=DEFAULT_STENCIL_COUNT,
        help="Representative point-triangle stencil count for the packet row.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="Maximum CPU/GPU squared-distance absolute error.",
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
        "(EdgeEdge)?ContactCandidate|"
        "PointTriangleCandidateMask|"
        "EdgeEdgeCandidateMask|"
        "SweptPointTriangleCandidateMask|"
        "SweptEdgeEdgeCandidateMask|"
        "SweptPointTriangleSweep|"
        "SweptEdgeEdgeSweep|"
        "SceneRuntimePointTriangleSweep|"
        "SceneRuntimeEdgeEdgeSweep|"
        "SceneRuntimeCombinedSweepFilter|"
        "RuntimePointTriangleCandidateBuffer|"
        "RuntimeEdgeEdgeCandidateBuffer|"
        "SceneRuntimePointTriangleCandidateBuffer|"
        "SceneRuntimeEdgeEdgeCandidateBuffer|"
        "SceneRuntimeCombinedCandidateBuffer"
        ")(Cpu|Cuda)"
        f"/{args.stencil_count}(/real_time)?$"
    )
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_gpu_contact_candidates",
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
        raise Plan083GpuContactCandidatePacketError(
            f"{path}: invalid JSON: {exc}"
        ) from exc
    if not isinstance(data, dict):
        raise Plan083GpuContactCandidatePacketError(
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
        raise Plan083GpuContactCandidatePacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _expected_row_names(stencil_count: int) -> dict[str, tuple[str, str]]:
    return {
        "point_triangle": (
            f"BM_Plan083ContactCandidateCpu/{stencil_count}",
            f"BM_Plan083ContactCandidateCuda/{stencil_count}",
        ),
        "edge_edge": (
            f"BM_Plan083EdgeEdgeContactCandidateCpu/{stencil_count}",
            f"BM_Plan083EdgeEdgeContactCandidateCuda/{stencil_count}",
        ),
    }


def _expected_candidate_mask_row_names(pair_count: int) -> tuple[str, str]:
    return (
        f"BM_Plan083PointTriangleCandidateMaskCpu/{pair_count}",
        f"BM_Plan083PointTriangleCandidateMaskCuda/{pair_count}",
    )


def _expected_edge_edge_candidate_mask_row_names(pair_count: int) -> tuple[str, str]:
    return (
        f"BM_Plan083EdgeEdgeCandidateMaskCpu/{pair_count}",
        f"BM_Plan083EdgeEdgeCandidateMaskCuda/{pair_count}",
    )


def _expected_swept_candidate_mask_row_names(pair_count: int) -> tuple[str, str]:
    return (
        f"BM_Plan083SweptPointTriangleCandidateMaskCpu/{pair_count}",
        f"BM_Plan083SweptPointTriangleCandidateMaskCuda/{pair_count}",
    )


def _expected_swept_edge_edge_candidate_mask_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SweptEdgeEdgeCandidateMaskCpu/{pair_count}",
        f"BM_Plan083SweptEdgeEdgeCandidateMaskCuda/{pair_count}",
    )


def _expected_swept_point_triangle_sweep_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SweptPointTriangleSweepCpu/{pair_count}",
        f"BM_Plan083SweptPointTriangleSweepCuda/{pair_count}",
    )


def _expected_swept_edge_edge_sweep_row_names(pair_count: int) -> tuple[str, str]:
    return (
        f"BM_Plan083SweptEdgeEdgeSweepCpu/{pair_count}",
        f"BM_Plan083SweptEdgeEdgeSweepCuda/{pair_count}",
    )


def _expected_runtime_point_triangle_candidate_buffer_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083RuntimePointTriangleCandidateBufferCpu/{pair_count}",
        f"BM_Plan083RuntimePointTriangleCandidateBufferCuda/{pair_count}",
    )


def _expected_runtime_edge_edge_candidate_buffer_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083RuntimeEdgeEdgeCandidateBufferCpu/{pair_count}",
        f"BM_Plan083RuntimeEdgeEdgeCandidateBufferCuda/{pair_count}",
    )


def _expected_scene_runtime_point_triangle_sweep_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SceneRuntimePointTriangleSweepCpu/{pair_count}",
        f"BM_Plan083SceneRuntimePointTriangleSweepCuda/{pair_count}",
    )


def _expected_scene_runtime_edge_edge_sweep_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SceneRuntimeEdgeEdgeSweepCpu/{pair_count}",
        f"BM_Plan083SceneRuntimeEdgeEdgeSweepCuda/{pair_count}",
    )


def _expected_scene_runtime_combined_sweep_filter_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SceneRuntimeCombinedSweepFilterCpu/{pair_count}",
        f"BM_Plan083SceneRuntimeCombinedSweepFilterCuda/{pair_count}",
    )


def _expected_scene_runtime_point_triangle_candidate_buffer_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SceneRuntimePointTriangleCandidateBufferCpu/{pair_count}",
        f"BM_Plan083SceneRuntimePointTriangleCandidateBufferCuda/{pair_count}",
    )


def _expected_scene_runtime_edge_edge_candidate_buffer_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SceneRuntimeEdgeEdgeCandidateBufferCpu/{pair_count}",
        f"BM_Plan083SceneRuntimeEdgeEdgeCandidateBufferCuda/{pair_count}",
    )


def _expected_scene_runtime_combined_candidate_buffer_row_names(
    pair_count: int,
) -> tuple[str, str]:
    return (
        f"BM_Plan083SceneRuntimeCombinedCandidateBufferCpu/{pair_count}",
        f"BM_Plan083SceneRuntimeCombinedCandidateBufferCuda/{pair_count}",
    )


def _representative_rows(
    rows: list[Mapping[str, Any]], stencil_count: int
) -> dict[str, Mapping[str, Any]]:
    expected_rows = _expected_row_names(stencil_count)
    expected_names = {name for names in expected_rows.values() for name in names}
    expected_names.update(_expected_candidate_mask_row_names(stencil_count))
    expected_names.update(_expected_edge_edge_candidate_mask_row_names(stencil_count))
    expected_names.update(_expected_swept_candidate_mask_row_names(stencil_count))
    expected_names.update(
        _expected_swept_edge_edge_candidate_mask_row_names(stencil_count)
    )
    expected_names.update(_expected_swept_point_triangle_sweep_row_names(stencil_count))
    expected_names.update(_expected_swept_edge_edge_sweep_row_names(stencil_count))
    expected_names.update(
        _expected_scene_runtime_point_triangle_sweep_row_names(stencil_count)
    )
    expected_names.update(
        _expected_scene_runtime_edge_edge_sweep_row_names(stencil_count)
    )
    expected_names.update(
        _expected_scene_runtime_combined_sweep_filter_row_names(stencil_count)
    )
    expected_names.update(
        _expected_runtime_point_triangle_candidate_buffer_row_names(stencil_count)
    )
    expected_names.update(
        _expected_runtime_edge_edge_candidate_buffer_row_names(stencil_count)
    )
    expected_names.update(
        _expected_scene_runtime_point_triangle_candidate_buffer_row_names(stencil_count)
    )
    expected_names.update(
        _expected_scene_runtime_edge_edge_candidate_buffer_row_names(stencil_count)
    )
    expected_names.update(
        _expected_scene_runtime_combined_candidate_buffer_row_names(stencil_count)
    )
    found: dict[str, Mapping[str, Any]] = {}
    errors: list[str] = []

    for row in rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical not in expected_names:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found[canonical] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for expected in expected_names:
        if expected not in found:
            errors.append(f"missing median benchmark row: {expected}")

    if errors:
        raise Plan083GpuContactCandidatePacketError("\n".join(errors))

    return found


def _validate_primitive_family(
    *,
    family: str,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    stencil_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} candidate max error {max_error:.3g} exceeds tolerance "
            f"{tolerance:.3g}"
        )

    cpu_accepted = _counter(cpu_row, "accepted_count")
    gpu_accepted = _counter(gpu_row, "gpu_accepted_count")
    if int(cpu_accepted) != int(gpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{family} CPU accepted count {cpu_accepted:g} != GPU accepted "
            f"count {gpu_accepted:g}"
        )

    cpu_stencils = int(_counter(cpu_row, "stencils"))
    gpu_stencils = int(_counter(gpu_row, "stencils"))
    if cpu_stencils != stencil_count or gpu_stencils != stencil_count:
        raise Plan083GpuContactCandidatePacketError(
            f"{family} expected {stencil_count} stencils, got "
            f"CPU={cpu_stencils}, GPU={gpu_stencils}"
        )

    speedup = cpu_ns / gpu_ns
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
        raise Plan083GpuContactCandidatePacketError(
            f"{family} packet timing is missing {sorted(missing)}"
        )

    return {
        "stencil_count": stencil_count,
        "accepted_count": int(cpu_accepted),
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def _validate_candidate_mask(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    pair_count: int,
    tolerance: float,
    speedup_gate: float,
    label: str = "point-triangle candidate mask",
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_accepted = _counter(cpu_row, "accepted_count")
    gpu_accepted = _counter(gpu_row, "gpu_accepted_count")
    if int(cpu_accepted) != int(gpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU accepted count "
            f"{cpu_accepted:g} != GPU accepted count {gpu_accepted:g}"
        )
    gpu_compacted = _counter(gpu_row, "gpu_compacted_count")
    if int(gpu_compacted) != int(cpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU compacted count "
            f"{gpu_compacted:g} != accepted count {cpu_accepted:g}"
        )
    gpu_compacted_triangles = _counter(gpu_row, "gpu_compacted_triangle_count")
    if int(gpu_compacted_triangles) != int(cpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU compacted triangle count "
            f"{gpu_compacted_triangles:g} != accepted count {cpu_accepted:g}"
        )
    gpu_compacted_distances = _counter(gpu_row, "gpu_compacted_distance_count")
    if int(gpu_compacted_distances) != int(cpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU compacted distance count "
            f"{gpu_compacted_distances:g} != accepted count {cpu_accepted:g}"
        )

    cpu_pairs = int(_counter(cpu_row, "pairs"))
    gpu_pairs = int(_counter(gpu_row, "gpu_pairs"))
    if cpu_pairs != pair_count or gpu_pairs != pair_count:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} expected "
            f"{pair_count} pairs, got CPU={cpu_pairs}, GPU={gpu_pairs}"
        )

    cpu_points = int(_counter(cpu_row, "points"))
    gpu_points = int(_counter(gpu_row, "gpu_points"))
    cpu_triangles = int(_counter(cpu_row, "triangles"))
    gpu_triangles = int(_counter(gpu_row, "gpu_triangles"))
    if cpu_points != gpu_points or cpu_triangles != gpu_triangles:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: "
            f"points {cpu_points}/{gpu_points}, "
            f"triangles {cpu_triangles}/{gpu_triangles}"
        )

    speedup = cpu_ns / gpu_ns
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
        raise Plan083GpuContactCandidatePacketError(
            f"{label} timing is missing {sorted(missing)}"
        )

    return {
        "pair_count": pair_count,
        "point_count": cpu_points,
        "triangle_count": cpu_triangles,
        "accepted_count": int(cpu_accepted),
        "compacted_count": int(gpu_compacted),
        "compacted_triangle_count": int(gpu_compacted_triangles),
        "compacted_distance_count": int(gpu_compacted_distances),
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def _validate_edge_edge_candidate_mask(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    pair_count: int,
    tolerance: float,
    speedup_gate: float,
    label: str = "edge-edge candidate mask",
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_accepted = _counter(cpu_row, "accepted_count")
    gpu_accepted = _counter(gpu_row, "gpu_accepted_count")
    if int(cpu_accepted) != int(gpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU accepted count "
            f"{cpu_accepted:g} != GPU accepted count {gpu_accepted:g}"
        )
    gpu_compacted_a = _counter(gpu_row, "gpu_compacted_edge_a_count")
    if int(gpu_compacted_a) != int(cpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU compacted edge-a count "
            f"{gpu_compacted_a:g} != accepted count {cpu_accepted:g}"
        )
    gpu_compacted_b = _counter(gpu_row, "gpu_compacted_edge_b_count")
    if int(gpu_compacted_b) != int(cpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU compacted edge-b count "
            f"{gpu_compacted_b:g} != accepted count {cpu_accepted:g}"
        )
    gpu_compacted_distances = _counter(gpu_row, "gpu_compacted_distance_count")
    if int(gpu_compacted_distances) != int(cpu_accepted):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU compacted distance count "
            f"{gpu_compacted_distances:g} != accepted count {cpu_accepted:g}"
        )

    cpu_pairs = int(_counter(cpu_row, "pairs"))
    gpu_pairs = int(_counter(gpu_row, "gpu_pairs"))
    if cpu_pairs != pair_count or gpu_pairs != pair_count:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} expected "
            f"{pair_count} pairs, got CPU={cpu_pairs}, GPU={gpu_pairs}"
        )

    cpu_edges = int(_counter(cpu_row, "edges"))
    gpu_edges = int(_counter(gpu_row, "gpu_edges"))
    if cpu_edges != gpu_edges:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: " f"edges {cpu_edges}/{gpu_edges}"
        )

    speedup = cpu_ns / gpu_ns
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
        raise Plan083GpuContactCandidatePacketError(
            f"{label} timing is missing {sorted(missing)}"
        )

    return {
        "pair_count": pair_count,
        "edge_count": cpu_edges,
        "accepted_count": int(cpu_accepted),
        "compacted_edge_a_count": int(gpu_compacted_a),
        "compacted_edge_b_count": int(gpu_compacted_b),
        "compacted_distance_count": int(gpu_compacted_distances),
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def _validate_swept_point_triangle_sweep(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    pair_capacity: int | None,
    tolerance: float,
    speedup_gate: float,
    label: str = "swept point-triangle sweep broad phase",
    include_scene_bodies: bool = False,
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_accepted = int(_counter(cpu_row, "accepted_count"))
    gpu_accepted = int(_counter(gpu_row, "gpu_accepted_count"))
    if cpu_accepted != gpu_accepted:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU accepted count {cpu_accepted} != GPU accepted count "
            f"{gpu_accepted}"
        )

    gpu_compacted = int(_counter(gpu_row, "gpu_compacted_count"))
    gpu_compacted_triangles = int(_counter(gpu_row, "gpu_compacted_triangle_count"))
    gpu_compacted_distances = int(_counter(gpu_row, "gpu_compacted_distance_count"))
    if not (
        gpu_compacted
        == gpu_compacted_triangles
        == gpu_compacted_distances
        == cpu_accepted
    ):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} compacted count mismatch: points={gpu_compacted}, "
            f"triangles={gpu_compacted_triangles}, "
            f"distances={gpu_compacted_distances}, accepted={cpu_accepted}"
        )

    cpu_capacity = int(_counter(cpu_row, "pair_capacity"))
    gpu_capacity = int(_counter(gpu_row, "gpu_pair_capacity"))
    if gpu_capacity != cpu_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU pair capacity mismatch: " f"{cpu_capacity}/{gpu_capacity}"
        )
    if pair_capacity is not None and cpu_capacity != pair_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} expected {pair_capacity} pair capacity, got {cpu_capacity}"
        )

    cpu_points = int(_counter(cpu_row, "points"))
    gpu_points = int(_counter(gpu_row, "gpu_points"))
    cpu_triangles = int(_counter(cpu_row, "triangles"))
    gpu_triangles = int(_counter(gpu_row, "gpu_triangles"))
    if cpu_points != gpu_points or cpu_triangles != gpu_triangles:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: points {cpu_points}/{gpu_points}, "
            f"triangles {cpu_triangles}/{gpu_triangles}"
        )
    scene_body_count: int | None = None
    if include_scene_bodies:
        cpu_scene_bodies = int(_counter(cpu_row, "scene_bodies"))
        gpu_scene_bodies = int(_counter(gpu_row, "scene_bodies"))
        if cpu_scene_bodies != gpu_scene_bodies:
            raise Plan083GpuContactCandidatePacketError(
                f"{label} CPU/GPU scene body mismatch: "
                f"{cpu_scene_bodies}/{gpu_scene_bodies}"
            )
        scene_body_count = cpu_scene_bodies

    speedup = cpu_ns / gpu_ns
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }

    row = {
        "pair_capacity": cpu_capacity,
        "point_count": cpu_points,
        "triangle_count": cpu_triangles,
        "accepted_count": cpu_accepted,
        "compacted_count": gpu_compacted,
        "compacted_triangle_count": gpu_compacted_triangles,
        "compacted_distance_count": gpu_compacted_distances,
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }
    if scene_body_count is not None:
        row["scene_body_count"] = scene_body_count
    return row


def _validate_swept_edge_edge_sweep(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    pair_capacity: int | None,
    tolerance: float,
    speedup_gate: float,
    label: str = "swept edge-edge sweep broad phase",
    include_scene_bodies: bool = False,
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_accepted = int(_counter(cpu_row, "accepted_count"))
    gpu_accepted = int(_counter(gpu_row, "gpu_accepted_count"))
    if cpu_accepted != gpu_accepted:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU accepted count {cpu_accepted} != GPU accepted count "
            f"{gpu_accepted}"
        )

    gpu_compacted_a = int(_counter(gpu_row, "gpu_compacted_edge_a_count"))
    gpu_compacted_b = int(_counter(gpu_row, "gpu_compacted_edge_b_count"))
    gpu_compacted_distances = int(_counter(gpu_row, "gpu_compacted_distance_count"))
    if not (
        gpu_compacted_a == gpu_compacted_b == gpu_compacted_distances == cpu_accepted
    ):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} compacted count mismatch: edge_a={gpu_compacted_a}, "
            f"edge_b={gpu_compacted_b}, distances={gpu_compacted_distances}, "
            f"accepted={cpu_accepted}"
        )

    cpu_capacity = int(_counter(cpu_row, "pair_capacity"))
    gpu_capacity = int(_counter(gpu_row, "gpu_pair_capacity"))
    if gpu_capacity != cpu_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU pair capacity mismatch: " f"{cpu_capacity}/{gpu_capacity}"
        )
    if pair_capacity is not None and cpu_capacity != pair_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} expected {pair_capacity} pair capacity, got {cpu_capacity}"
        )

    cpu_edges = int(_counter(cpu_row, "edges"))
    gpu_edges = int(_counter(gpu_row, "gpu_edges"))
    if cpu_edges != gpu_edges:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: edges {cpu_edges}/{gpu_edges}"
        )
    scene_body_count: int | None = None
    if include_scene_bodies:
        cpu_scene_bodies = int(_counter(cpu_row, "scene_bodies"))
        gpu_scene_bodies = int(_counter(gpu_row, "scene_bodies"))
        if cpu_scene_bodies != gpu_scene_bodies:
            raise Plan083GpuContactCandidatePacketError(
                f"{label} CPU/GPU scene body mismatch: "
                f"{cpu_scene_bodies}/{gpu_scene_bodies}"
            )
        scene_body_count = cpu_scene_bodies

    speedup = cpu_ns / gpu_ns
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }

    row = {
        "pair_capacity": cpu_capacity,
        "edge_count": cpu_edges,
        "accepted_count": cpu_accepted,
        "compacted_edge_a_count": gpu_compacted_a,
        "compacted_edge_b_count": gpu_compacted_b,
        "compacted_distance_count": gpu_compacted_distances,
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }
    if scene_body_count is not None:
        row["scene_body_count"] = scene_body_count
    return row


def _validate_scene_runtime_combined_sweep_filter(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    label = "combined scene runtime sweep filter"
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_capacity = int(_counter(cpu_row, "pair_capacity"))
    gpu_capacity = int(_counter(gpu_row, "gpu_pair_capacity"))
    if cpu_capacity != gpu_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU pair capacity mismatch: {cpu_capacity}/{gpu_capacity}"
        )
    cpu_point_capacity = int(_counter(cpu_row, "point_triangle_pair_capacity"))
    gpu_point_capacity = int(_counter(gpu_row, "gpu_point_triangle_pair_capacity"))
    cpu_edge_capacity = int(_counter(cpu_row, "edge_edge_pair_capacity"))
    gpu_edge_capacity = int(_counter(gpu_row, "gpu_edge_edge_pair_capacity"))
    if cpu_point_capacity != gpu_point_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU point-triangle pair capacity mismatch: "
            f"{cpu_point_capacity}/{gpu_point_capacity}"
        )
    if cpu_edge_capacity != gpu_edge_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU edge-edge pair capacity mismatch: "
            f"{cpu_edge_capacity}/{gpu_edge_capacity}"
        )
    if cpu_capacity != cpu_point_capacity + cpu_edge_capacity:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} pair capacity {cpu_capacity} does not match "
            f"point-triangle plus edge-edge capacity "
            f"{cpu_point_capacity + cpu_edge_capacity}"
        )

    cpu_accepted = int(_counter(cpu_row, "accepted_count"))
    gpu_accepted = int(_counter(gpu_row, "gpu_accepted_count"))
    if cpu_accepted != gpu_accepted:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU accepted count mismatch: {cpu_accepted}/{gpu_accepted}"
        )
    cpu_point_accepted = int(_counter(cpu_row, "point_triangle_accepted_count"))
    gpu_point_accepted = int(_counter(gpu_row, "gpu_point_triangle_accepted_count"))
    cpu_edge_accepted = int(_counter(cpu_row, "edge_edge_accepted_count"))
    gpu_edge_accepted = int(_counter(gpu_row, "gpu_edge_edge_accepted_count"))
    if cpu_point_accepted != gpu_point_accepted:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU point-triangle accepted count mismatch: "
            f"{cpu_point_accepted}/{gpu_point_accepted}"
        )
    if cpu_edge_accepted != gpu_edge_accepted:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU edge-edge accepted count mismatch: "
            f"{cpu_edge_accepted}/{gpu_edge_accepted}"
        )
    if cpu_accepted != cpu_point_accepted + cpu_edge_accepted:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} accepted total {cpu_accepted} does not match "
            f"point-triangle plus edge-edge count "
            f"{cpu_point_accepted + cpu_edge_accepted}"
        )

    gpu_point_compacted = int(_counter(gpu_row, "gpu_point_triangle_compacted_count"))
    gpu_point_compacted_triangles = int(
        _counter(gpu_row, "gpu_point_triangle_compacted_triangle_count")
    )
    gpu_point_compacted_distances = int(
        _counter(gpu_row, "gpu_point_triangle_compacted_distance_count")
    )
    if not (
        gpu_point_compacted
        == gpu_point_compacted_triangles
        == gpu_point_compacted_distances
        == cpu_point_accepted
    ):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} point-triangle compacted count mismatch: "
            f"points={gpu_point_compacted}, "
            f"triangles={gpu_point_compacted_triangles}, "
            f"distances={gpu_point_compacted_distances}, "
            f"accepted={cpu_point_accepted}"
        )

    gpu_edge_compacted_a = int(
        _counter(gpu_row, "gpu_edge_edge_compacted_edge_a_count")
    )
    gpu_edge_compacted_b = int(
        _counter(gpu_row, "gpu_edge_edge_compacted_edge_b_count")
    )
    gpu_edge_compacted_distances = int(
        _counter(gpu_row, "gpu_edge_edge_compacted_distance_count")
    )
    if not (
        gpu_edge_compacted_a
        == gpu_edge_compacted_b
        == gpu_edge_compacted_distances
        == cpu_edge_accepted
    ):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} edge-edge compacted count mismatch: "
            f"edge_a={gpu_edge_compacted_a}, "
            f"edge_b={gpu_edge_compacted_b}, "
            f"distances={gpu_edge_compacted_distances}, "
            f"accepted={cpu_edge_accepted}"
        )

    cpu_points = int(_counter(cpu_row, "points"))
    gpu_points = int(_counter(gpu_row, "gpu_points"))
    cpu_triangles = int(_counter(cpu_row, "triangles"))
    gpu_triangles = int(_counter(gpu_row, "gpu_triangles"))
    cpu_edges = int(_counter(cpu_row, "edges"))
    gpu_edges = int(_counter(gpu_row, "gpu_edges"))
    if (
        cpu_points != gpu_points
        or cpu_triangles != gpu_triangles
        or cpu_edges != gpu_edges
    ):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: "
            f"points {cpu_points}/{gpu_points}, "
            f"triangles {cpu_triangles}/{gpu_triangles}, "
            f"edges {cpu_edges}/{gpu_edges}"
        )

    cpu_scene_bodies = int(_counter(cpu_row, "scene_bodies"))
    gpu_scene_bodies = int(_counter(gpu_row, "scene_bodies"))
    if cpu_scene_bodies != gpu_scene_bodies:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU scene body mismatch: "
            f"{cpu_scene_bodies}/{gpu_scene_bodies}"
        )

    speedup = cpu_ns / gpu_ns
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }

    return {
        "pair_capacity": cpu_capacity,
        "point_triangle_pair_capacity": cpu_point_capacity,
        "edge_edge_pair_capacity": cpu_edge_capacity,
        "accepted_count": cpu_accepted,
        "point_triangle_accepted_count": cpu_point_accepted,
        "edge_edge_accepted_count": cpu_edge_accepted,
        "point_triangle_compacted_count": gpu_point_compacted,
        "point_triangle_compacted_triangle_count": gpu_point_compacted_triangles,
        "point_triangle_compacted_distance_count": gpu_point_compacted_distances,
        "edge_edge_compacted_edge_a_count": gpu_edge_compacted_a,
        "edge_edge_compacted_edge_b_count": gpu_edge_compacted_b,
        "edge_edge_compacted_distance_count": gpu_edge_compacted_distances,
        "point_count": cpu_points,
        "triangle_count": cpu_triangles,
        "edge_count": cpu_edges,
        "scene_body_count": cpu_scene_bodies,
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def _validate_runtime_point_triangle_candidate_buffer(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    tolerance: float,
    speedup_gate: float,
    label: str = "runtime point-triangle candidate buffer",
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_candidates = int(_counter(cpu_row, "candidates"))
    gpu_candidates = int(_counter(gpu_row, "gpu_candidates"))
    if cpu_candidates != gpu_candidates:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU candidate count mismatch: "
            f"{cpu_candidates}/{gpu_candidates}"
        )

    cpu_points = int(_counter(cpu_row, "points"))
    gpu_points = int(_counter(gpu_row, "gpu_points"))
    cpu_triangles = int(_counter(cpu_row, "triangles"))
    gpu_triangles = int(_counter(gpu_row, "gpu_triangles"))
    if cpu_points != gpu_points or cpu_triangles != gpu_triangles:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: "
            f"points {cpu_points}/{gpu_points}, "
            f"triangles {cpu_triangles}/{gpu_triangles}"
        )
    cpu_scene_bodies = int(_counter(cpu_row, "scene_bodies"))
    gpu_scene_bodies = int(_counter(gpu_row, "scene_bodies"))
    if cpu_scene_bodies != gpu_scene_bodies:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU scene body mismatch: "
            f"{cpu_scene_bodies}/{gpu_scene_bodies}"
        )

    speedup = cpu_ns / gpu_ns
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }

    return {
        "candidate_count": cpu_candidates,
        "point_count": cpu_points,
        "triangle_count": cpu_triangles,
        "scene_body_count": cpu_scene_bodies,
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def _validate_runtime_edge_edge_candidate_buffer(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    tolerance: float,
    speedup_gate: float,
    label: str = "runtime edge-edge candidate buffer",
) -> dict[str, Any]:
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_candidates = int(_counter(cpu_row, "candidates"))
    gpu_candidates = int(_counter(gpu_row, "gpu_candidates"))
    if cpu_candidates != gpu_candidates:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU candidate count mismatch: "
            f"{cpu_candidates}/{gpu_candidates}"
        )

    cpu_edges = int(_counter(cpu_row, "edges"))
    gpu_edges = int(_counter(gpu_row, "gpu_edges"))
    if cpu_edges != gpu_edges:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: edges {cpu_edges}/{gpu_edges}"
        )
    cpu_scene_bodies = int(_counter(cpu_row, "scene_bodies"))
    gpu_scene_bodies = int(_counter(gpu_row, "scene_bodies"))
    if cpu_scene_bodies != gpu_scene_bodies:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU scene body mismatch: "
            f"{cpu_scene_bodies}/{gpu_scene_bodies}"
        )

    speedup = cpu_ns / gpu_ns
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }

    return {
        "candidate_count": cpu_candidates,
        "edge_count": cpu_edges,
        "scene_body_count": cpu_scene_bodies,
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def _validate_scene_runtime_combined_candidate_buffer(
    *,
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    label = "combined scene runtime candidate filter"
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU benchmark timing is not positive"
        )
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} GPU benchmark timing is not positive"
        )

    max_error = _counter(gpu_row, "max_result_abs_error")
    if max_error > tolerance:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    cpu_candidates = int(_counter(cpu_row, "candidates"))
    gpu_candidates = int(_counter(gpu_row, "gpu_candidates"))
    if cpu_candidates != gpu_candidates:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU candidate count mismatch: "
            f"{cpu_candidates}/{gpu_candidates}"
        )

    cpu_point_triangle_candidates = int(_counter(cpu_row, "point_triangle_candidates"))
    gpu_point_triangle_candidates = int(
        _counter(gpu_row, "gpu_point_triangle_candidates")
    )
    if cpu_point_triangle_candidates != gpu_point_triangle_candidates:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU point-triangle candidate count mismatch: "
            f"{cpu_point_triangle_candidates}/{gpu_point_triangle_candidates}"
        )

    cpu_edge_edge_candidates = int(_counter(cpu_row, "edge_edge_candidates"))
    gpu_edge_edge_candidates = int(_counter(gpu_row, "gpu_edge_edge_candidates"))
    if cpu_edge_edge_candidates != gpu_edge_edge_candidates:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU edge-edge candidate count mismatch: "
            f"{cpu_edge_edge_candidates}/{gpu_edge_edge_candidates}"
        )

    if cpu_candidates != cpu_point_triangle_candidates + cpu_edge_edge_candidates:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} candidate total {cpu_candidates} does not match "
            f"point-triangle plus edge-edge count "
            f"{cpu_point_triangle_candidates + cpu_edge_edge_candidates}"
        )

    cpu_points = int(_counter(cpu_row, "points"))
    gpu_points = int(_counter(gpu_row, "gpu_points"))
    cpu_triangles = int(_counter(cpu_row, "triangles"))
    gpu_triangles = int(_counter(gpu_row, "gpu_triangles"))
    cpu_edges = int(_counter(cpu_row, "edges"))
    gpu_edges = int(_counter(gpu_row, "gpu_edges"))
    if (
        cpu_points != gpu_points
        or cpu_triangles != gpu_triangles
        or cpu_edges != gpu_edges
    ):
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU shape mismatch: "
            f"points {cpu_points}/{gpu_points}, "
            f"triangles {cpu_triangles}/{gpu_triangles}, "
            f"edges {cpu_edges}/{gpu_edges}"
        )

    cpu_scene_bodies = int(_counter(cpu_row, "scene_bodies"))
    gpu_scene_bodies = int(_counter(gpu_row, "scene_bodies"))
    if cpu_scene_bodies != gpu_scene_bodies:
        raise Plan083GpuContactCandidatePacketError(
            f"{label} CPU/GPU scene body mismatch: "
            f"{cpu_scene_bodies}/{gpu_scene_bodies}"
        )

    speedup = cpu_ns / gpu_ns
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "kernel_ns"),
        "solve": 0.0,
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }

    return {
        "candidate_count": cpu_candidates,
        "point_triangle_candidate_count": cpu_point_triangle_candidates,
        "edge_edge_candidate_count": cpu_edge_edge_candidates,
        "point_count": cpu_points,
        "triangle_count": cpu_triangles,
        "edge_count": cpu_edges,
        "scene_body_count": cpu_scene_bodies,
        "max_result_abs_error": max_error,
        "speedup": speedup,
        "meets_speedup_gate": speedup >= speedup_gate,
        "timing_ns": timing_ns,
        "cpu_benchmark_row": _packet_row_name(cpu_row),
        "gpu_benchmark_row": _packet_row_name(gpu_row),
    }


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    stencil_count: int,
    tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise Plan083GpuContactCandidatePacketError(
            "benchmark JSON has no benchmark rows"
        )
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise Plan083GpuContactCandidatePacketError(
            "benchmark JSON has non-object rows"
        )

    representative_rows = _representative_rows(typed_rows, stencil_count)
    primitive_families = {}
    for family, (cpu_name, gpu_name) in _expected_row_names(stencil_count).items():
        primitive_families[family] = _validate_primitive_family(
            family=family,
            cpu_row=representative_rows[cpu_name],
            gpu_row=representative_rows[gpu_name],
            stencil_count=stencil_count,
            tolerance=tolerance,
            speedup_gate=speedup_gate,
        )

    candidate_mask_cpu, candidate_mask_gpu = _expected_candidate_mask_row_names(
        stencil_count
    )
    point_triangle_candidate_construction = _validate_candidate_mask(
        cpu_row=representative_rows[candidate_mask_cpu],
        gpu_row=representative_rows[candidate_mask_gpu],
        pair_count=stencil_count,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
    )
    edge_mask_cpu, edge_mask_gpu = _expected_edge_edge_candidate_mask_row_names(
        stencil_count
    )
    edge_edge_candidate_construction = _validate_edge_edge_candidate_mask(
        cpu_row=representative_rows[edge_mask_cpu],
        gpu_row=representative_rows[edge_mask_gpu],
        pair_count=stencil_count,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
    )
    swept_point_cpu, swept_point_gpu = _expected_swept_candidate_mask_row_names(
        stencil_count
    )
    swept_point_triangle_candidate_construction = _validate_candidate_mask(
        cpu_row=representative_rows[swept_point_cpu],
        gpu_row=representative_rows[swept_point_gpu],
        pair_count=stencil_count,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
        label="swept point-triangle candidate mask",
    )
    swept_edge_cpu, swept_edge_gpu = _expected_swept_edge_edge_candidate_mask_row_names(
        stencil_count
    )
    swept_edge_edge_candidate_construction = _validate_edge_edge_candidate_mask(
        cpu_row=representative_rows[swept_edge_cpu],
        gpu_row=representative_rows[swept_edge_gpu],
        pair_count=stencil_count,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
        label="swept edge-edge candidate mask",
    )
    sweep_point_cpu, sweep_point_gpu = _expected_swept_point_triangle_sweep_row_names(
        stencil_count
    )
    swept_point_triangle_sweep = _validate_swept_point_triangle_sweep(
        cpu_row=representative_rows[sweep_point_cpu],
        gpu_row=representative_rows[sweep_point_gpu],
        pair_capacity=stencil_count,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
    )
    sweep_edge_cpu, sweep_edge_gpu = _expected_swept_edge_edge_sweep_row_names(
        stencil_count
    )
    swept_edge_edge_sweep = _validate_swept_edge_edge_sweep(
        cpu_row=representative_rows[sweep_edge_cpu],
        gpu_row=representative_rows[sweep_edge_gpu],
        pair_capacity=stencil_count,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
    )
    scene_sweep_point_cpu, scene_sweep_point_gpu = (
        _expected_scene_runtime_point_triangle_sweep_row_names(stencil_count)
    )
    scene_runtime_point_triangle_sweep = _validate_swept_point_triangle_sweep(
        cpu_row=representative_rows[scene_sweep_point_cpu],
        gpu_row=representative_rows[scene_sweep_point_gpu],
        pair_capacity=None,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
        label="scene runtime point-triangle sweep broad phase",
        include_scene_bodies=True,
    )
    scene_sweep_edge_cpu, scene_sweep_edge_gpu = (
        _expected_scene_runtime_edge_edge_sweep_row_names(stencil_count)
    )
    scene_runtime_edge_edge_sweep = _validate_swept_edge_edge_sweep(
        cpu_row=representative_rows[scene_sweep_edge_cpu],
        gpu_row=representative_rows[scene_sweep_edge_gpu],
        pair_capacity=None,
        tolerance=tolerance,
        speedup_gate=speedup_gate,
        label="scene runtime edge-edge sweep broad phase",
        include_scene_bodies=True,
    )
    combined_sweep_cpu, combined_sweep_gpu = (
        _expected_scene_runtime_combined_sweep_filter_row_names(stencil_count)
    )
    combined_scene_runtime_sweep_filter = _validate_scene_runtime_combined_sweep_filter(
        cpu_row=representative_rows[combined_sweep_cpu],
        gpu_row=representative_rows[combined_sweep_gpu],
        tolerance=tolerance,
        speedup_gate=speedup_gate,
    )
    if (
        combined_scene_runtime_sweep_filter["point_triangle_pair_capacity"]
        != scene_runtime_point_triangle_sweep["pair_capacity"]
        or combined_scene_runtime_sweep_filter["point_triangle_accepted_count"]
        != scene_runtime_point_triangle_sweep["accepted_count"]
    ):
        raise Plan083GpuContactCandidatePacketError(
            "combined scene runtime sweep filter point-triangle counts do not "
            "match the point-triangle scene runtime sweep row"
        )
    if (
        combined_scene_runtime_sweep_filter["edge_edge_pair_capacity"]
        != scene_runtime_edge_edge_sweep["pair_capacity"]
        or combined_scene_runtime_sweep_filter["edge_edge_accepted_count"]
        != scene_runtime_edge_edge_sweep["accepted_count"]
    ):
        raise Plan083GpuContactCandidatePacketError(
            "combined scene runtime sweep filter edge-edge counts do not match "
            "the edge-edge scene runtime sweep row"
        )
    if (
        combined_scene_runtime_sweep_filter["scene_body_count"]
        != scene_runtime_point_triangle_sweep["scene_body_count"]
        or combined_scene_runtime_sweep_filter["scene_body_count"]
        != scene_runtime_edge_edge_sweep["scene_body_count"]
    ):
        raise Plan083GpuContactCandidatePacketError(
            "combined scene runtime sweep filter scene body count does not match "
            "the per-family scene runtime sweep rows"
        )
    runtime_point_cpu, runtime_point_gpu = (
        _expected_runtime_point_triangle_candidate_buffer_row_names(stencil_count)
    )
    runtime_point_triangle_candidate_buffer = (
        _validate_runtime_point_triangle_candidate_buffer(
            cpu_row=representative_rows[runtime_point_cpu],
            gpu_row=representative_rows[runtime_point_gpu],
            tolerance=tolerance,
            speedup_gate=speedup_gate,
        )
    )
    runtime_edge_cpu, runtime_edge_gpu = (
        _expected_runtime_edge_edge_candidate_buffer_row_names(stencil_count)
    )
    runtime_edge_edge_candidate_buffer = _validate_runtime_edge_edge_candidate_buffer(
        cpu_row=representative_rows[runtime_edge_cpu],
        gpu_row=representative_rows[runtime_edge_gpu],
        tolerance=tolerance,
        speedup_gate=speedup_gate,
    )
    scene_runtime_point_cpu, scene_runtime_point_gpu = (
        _expected_scene_runtime_point_triangle_candidate_buffer_row_names(stencil_count)
    )
    scene_runtime_point_triangle_candidate_buffer = (
        _validate_runtime_point_triangle_candidate_buffer(
            cpu_row=representative_rows[scene_runtime_point_cpu],
            gpu_row=representative_rows[scene_runtime_point_gpu],
            tolerance=tolerance,
            speedup_gate=speedup_gate,
            label="scene runtime point-triangle candidate buffer",
        )
    )
    scene_runtime_edge_cpu, scene_runtime_edge_gpu = (
        _expected_scene_runtime_edge_edge_candidate_buffer_row_names(stencil_count)
    )
    scene_runtime_edge_edge_candidate_buffer = (
        _validate_runtime_edge_edge_candidate_buffer(
            cpu_row=representative_rows[scene_runtime_edge_cpu],
            gpu_row=representative_rows[scene_runtime_edge_gpu],
            tolerance=tolerance,
            speedup_gate=speedup_gate,
            label="scene runtime edge-edge candidate buffer",
        )
    )
    scene_combined_cpu, scene_combined_gpu = (
        _expected_scene_runtime_combined_candidate_buffer_row_names(stencil_count)
    )
    combined_scene_runtime_candidate_filter = (
        _validate_scene_runtime_combined_candidate_buffer(
            cpu_row=representative_rows[scene_combined_cpu],
            gpu_row=representative_rows[scene_combined_gpu],
            tolerance=tolerance,
            speedup_gate=speedup_gate,
        )
    )
    if (
        combined_scene_runtime_candidate_filter["point_triangle_candidate_count"]
        != scene_runtime_point_triangle_candidate_buffer["candidate_count"]
    ):
        raise Plan083GpuContactCandidatePacketError(
            "combined scene runtime candidate filter point-triangle count does "
            "not match the point-triangle scene runtime buffer row"
        )
    if (
        combined_scene_runtime_candidate_filter["edge_edge_candidate_count"]
        != scene_runtime_edge_edge_candidate_buffer["candidate_count"]
    ):
        raise Plan083GpuContactCandidatePacketError(
            "combined scene runtime candidate filter edge-edge count does not "
            "match the edge-edge scene runtime buffer row"
        )
    if (
        combined_scene_runtime_candidate_filter["scene_body_count"]
        != scene_runtime_point_triangle_candidate_buffer["scene_body_count"]
        or combined_scene_runtime_candidate_filter["scene_body_count"]
        != scene_runtime_edge_edge_candidate_buffer["scene_body_count"]
    ):
        raise Plan083GpuContactCandidatePacketError(
            "combined scene runtime candidate filter scene body count does not "
            "match the per-family scene runtime buffer rows"
        )
    candidate_construction = {
        "point_triangle_all_pairs_mask": point_triangle_candidate_construction,
        "edge_edge_all_pairs_mask": edge_edge_candidate_construction,
        "point_triangle_swept_aabb_candidates": (
            swept_point_triangle_candidate_construction
        ),
        "edge_edge_swept_aabb_candidates": swept_edge_edge_candidate_construction,
        "point_triangle_sweep_broad_phase": swept_point_triangle_sweep,
        "edge_edge_sweep_broad_phase": swept_edge_edge_sweep,
        "point_triangle_scene_runtime_sweep": scene_runtime_point_triangle_sweep,
        "edge_edge_scene_runtime_sweep": scene_runtime_edge_edge_sweep,
        "combined_scene_runtime_sweep_filter": combined_scene_runtime_sweep_filter,
        "point_triangle_runtime_sweep_buffer": (
            runtime_point_triangle_candidate_buffer
        ),
        "edge_edge_runtime_sweep_buffer": runtime_edge_edge_candidate_buffer,
        "point_triangle_scene_runtime_buffer": (
            scene_runtime_point_triangle_candidate_buffer
        ),
        "edge_edge_scene_runtime_buffer": scene_runtime_edge_edge_candidate_buffer,
        "combined_scene_runtime_candidate_filter": (
            combined_scene_runtime_candidate_filter
        ),
    }

    point_triangle = primitive_families["point_triangle"]
    max_error = max(
        [family["max_result_abs_error"] for family in primitive_families.values()]
        + [family["max_result_abs_error"] for family in candidate_construction.values()]
    )
    speedup = min(
        [family["speedup"] for family in primitive_families.values()]
        + [family["speedup"] for family in candidate_construction.values()]
    )
    meets_speedup_gate = all(
        [family["meets_speedup_gate"] for family in primitive_families.values()]
        + [family["meets_speedup_gate"] for family in candidate_construction.values()]
    )

    return {
        "plan083_gpu_contact_candidate_packet": {
            "row_id": "contact-stencils-candidate-filtering",
            "same_scene_cpu_gpu": True,
            "primitive_families": primitive_families,
            "candidate_construction": candidate_construction,
            "stencil_count": stencil_count * len(primitive_families),
            "candidate_pair_count": sum(
                family.get("pair_count", family.get("pair_capacity", 0))
                for family in candidate_construction.values()
            ),
            "accepted_count": sum(
                family["accepted_count"] for family in primitive_families.values()
            ),
            "max_result_abs_error": max_error,
            "result_abs_error_tolerance": tolerance,
            "speedup": speedup,
            "speedup_gate": speedup_gate,
            "meets_speedup_gate": meets_speedup_gate,
            "timing_ns": point_triangle["timing_ns"],
            "cpu_benchmark_row": point_triangle["cpu_benchmark_row"],
            "gpu_benchmark_row": point_triangle["gpu_benchmark_row"],
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
            stencil_count=args.stencil_count,
            tolerance=args.tolerance,
            speedup_gate=args.speedup_gate,
        )
    except Plan083GpuContactCandidatePacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_gpu_contact_candidate_packet"]
    print(
        "PLAN-083 GPU contact candidate packet OK: "
        f"stencils={row['stencil_count']} max_error={row['max_result_abs_error']:.3g} "
        f"speedup={row['speedup']:.3f}x "
        f"meets_gate={row['meets_speedup_gate']}"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
