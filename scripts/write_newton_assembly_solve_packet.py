#!/usr/bin/env python3
"""Run and validate the private GPU Newton assembly/solve packet."""

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
    ".benchmark_results/plan083/gpu/assembly_linear_solve_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/gpu/assembly_linear_solve_parity.json"
)

DEFAULT_ROW_COUNT = 65536
DEFAULT_TOLERANCE = 1e-10
DEFAULT_RESIDUAL_TOLERANCE = 1e-8
DEFAULT_SPEEDUP_GATE = 1.25
REQUIRED_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "solve",
    "device_to_host",
    "readback",
}
EQUALITY_TIMING_KEYS = REQUIRED_TIMING_KEYS | {"reduction", "expansion"}
SPARSE_RESIDUAL_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "gradient_seed",
    "diagonal",
    "off_diagonal",
    "device_to_host",
    "readback",
}
SPARSE_JACOBI_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "iterations",
    "final_residual",
    "device_to_host",
    "readback",
}
SPARSE_CG_TIMING_KEYS = SPARSE_JACOBI_TIMING_KEYS


class NewtonAssemblySolvePacketError(RuntimeError):
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
        "--row-count",
        type=int,
        default=DEFAULT_ROW_COUNT,
        help="Representative Newton assembly row count.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help="Maximum CPU/GPU output absolute error.",
    )
    parser.add_argument(
        "--residual-tolerance",
        type=float,
        default=DEFAULT_RESIDUAL_TOLERANCE,
        help="Maximum accepted regularized residual norm.",
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
        "^BM_Newton(AssemblySolve|SceneRuntimeAssemblySolve|"
        "OffDiagonalAssembly|SparseResidual|SparseJacobiSolve|"
        "SparseCgSolve|EqualityReducedSolve)(Cpu|Cuda)"
        f"/{args.row_count}(/real_time)?$"
    )
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_newton_assembly_solve_cuda",
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
        raise NewtonAssemblySolvePacketError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise NewtonAssemblySolvePacketError(f"{path}: JSON root must be an object")
    return data


def _finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _counter(row: Mapping[str, Any], key: str) -> float:
    value = _finite_number(row.get(key))
    if value is None:
        raise NewtonAssemblySolvePacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _representative_rows(
    rows: list[Mapping[str, Any]], row_count: int
) -> dict[str, Mapping[str, Any]]:
    expected_names = {
        "diagonal_cpu": f"BM_NewtonAssemblySolveCpu/{row_count}",
        "diagonal_gpu": f"BM_NewtonAssemblySolveCuda/{row_count}",
        "scene_assembly_cpu": f"BM_NewtonSceneRuntimeAssemblySolveCpu/{row_count}",
        "scene_assembly_gpu": f"BM_NewtonSceneRuntimeAssemblySolveCuda/{row_count}",
        "off_diagonal_cpu": f"BM_NewtonOffDiagonalAssemblyCpu/{row_count}",
        "off_diagonal_gpu": f"BM_NewtonOffDiagonalAssemblyCuda/{row_count}",
        "sparse_residual_cpu": f"BM_NewtonSparseResidualCpu/{row_count}",
        "sparse_residual_gpu": f"BM_NewtonSparseResidualCuda/{row_count}",
        "sparse_jacobi_cpu": f"BM_NewtonSparseJacobiSolveCpu/{row_count}",
        "sparse_jacobi_gpu": f"BM_NewtonSparseJacobiSolveCuda/{row_count}",
        "sparse_cg_cpu": f"BM_NewtonSparseCgSolveCpu/{row_count}",
        "sparse_cg_gpu": f"BM_NewtonSparseCgSolveCuda/{row_count}",
        "equality_cpu": f"BM_NewtonEqualityReducedSolveCpu/{row_count}",
        "equality_gpu": f"BM_NewtonEqualityReducedSolveCuda/{row_count}",
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
        raise NewtonAssemblySolvePacketError("\n".join(errors))

    return found


def _matching_int_counter(
    cpu_row: Mapping[str, Any],
    gpu_row: Mapping[str, Any],
    cpu_key: str,
    gpu_key: str,
) -> int:
    cpu_count = int(_counter(cpu_row, cpu_key))
    gpu_count = int(_counter(gpu_row, gpu_key))
    if cpu_count != gpu_count:
        raise NewtonAssemblySolvePacketError(
            f"{cpu_key} count {cpu_count} != {gpu_key} count {gpu_count}"
        )
    return cpu_count


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    row_count: int,
    tolerance: float,
    residual_tolerance: float,
    speedup_gate: float,
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise NewtonAssemblySolvePacketError("benchmark JSON has no benchmark rows")
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise NewtonAssemblySolvePacketError("benchmark JSON has non-object rows")

    representative_rows = _representative_rows(typed_rows, row_count)
    cpu_row = representative_rows["diagonal_cpu"]
    gpu_row = representative_rows["diagonal_gpu"]
    scene_assembly_cpu_row = representative_rows["scene_assembly_cpu"]
    scene_assembly_gpu_row = representative_rows["scene_assembly_gpu"]
    off_diagonal_cpu_row = representative_rows["off_diagonal_cpu"]
    off_diagonal_gpu_row = representative_rows["off_diagonal_gpu"]
    sparse_residual_cpu_row = representative_rows["sparse_residual_cpu"]
    sparse_residual_gpu_row = representative_rows["sparse_residual_gpu"]
    sparse_jacobi_cpu_row = representative_rows["sparse_jacobi_cpu"]
    sparse_jacobi_gpu_row = representative_rows["sparse_jacobi_gpu"]
    sparse_cg_cpu_row = representative_rows["sparse_cg_cpu"]
    sparse_cg_gpu_row = representative_rows["sparse_cg_gpu"]
    equality_cpu_row = representative_rows["equality_cpu"]
    equality_gpu_row = representative_rows["equality_gpu"]
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    scene_assembly_cpu_ns = benchmark_timing_ns(scene_assembly_cpu_row)
    scene_assembly_gpu_ns = benchmark_timing_ns(scene_assembly_gpu_row)
    off_diagonal_cpu_ns = benchmark_timing_ns(off_diagonal_cpu_row)
    off_diagonal_gpu_ns = benchmark_timing_ns(off_diagonal_gpu_row)
    sparse_residual_cpu_ns = benchmark_timing_ns(sparse_residual_cpu_row)
    sparse_residual_gpu_ns = benchmark_timing_ns(sparse_residual_gpu_row)
    sparse_jacobi_cpu_ns = benchmark_timing_ns(sparse_jacobi_cpu_row)
    sparse_jacobi_gpu_ns = benchmark_timing_ns(sparse_jacobi_gpu_row)
    sparse_cg_cpu_ns = benchmark_timing_ns(sparse_cg_cpu_row)
    sparse_cg_gpu_ns = benchmark_timing_ns(sparse_cg_gpu_row)
    equality_cpu_ns = benchmark_timing_ns(equality_cpu_row)
    equality_gpu_ns = benchmark_timing_ns(equality_gpu_row)
    if not math.isfinite(cpu_ns) or cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError("CPU benchmark timing is not positive")
    if not math.isfinite(gpu_ns) or gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError("GPU benchmark timing is not positive")
    if not math.isfinite(scene_assembly_cpu_ns) or scene_assembly_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime assembly CPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_assembly_gpu_ns) or scene_assembly_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime assembly GPU benchmark timing is not positive"
        )
    if not math.isfinite(off_diagonal_cpu_ns) or off_diagonal_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "off-diagonal CPU benchmark timing is not positive"
        )
    if not math.isfinite(off_diagonal_gpu_ns) or off_diagonal_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "off-diagonal GPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_residual_cpu_ns) or sparse_residual_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse residual CPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_residual_gpu_ns) or sparse_residual_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse residual GPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_jacobi_cpu_ns) or sparse_jacobi_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse Jacobi CPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_jacobi_gpu_ns) or sparse_jacobi_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse Jacobi GPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_cg_cpu_ns) or sparse_cg_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse CG CPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_cg_gpu_ns) or sparse_cg_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse CG GPU benchmark timing is not positive"
        )
    if not math.isfinite(equality_cpu_ns) or equality_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "equality-reduced CPU benchmark timing is not positive"
        )
    if not math.isfinite(equality_gpu_ns) or equality_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "equality-reduced GPU benchmark timing is not positive"
        )

    diagonal_max_error = _counter(gpu_row, "max_result_abs_error")
    scene_assembly_max_error = _counter(scene_assembly_gpu_row, "max_result_abs_error")
    off_diagonal_max_error = _counter(off_diagonal_gpu_row, "max_result_abs_error")
    sparse_residual_max_error = _counter(
        sparse_residual_gpu_row, "max_result_abs_error"
    )
    sparse_jacobi_max_error = _counter(sparse_jacobi_gpu_row, "max_result_abs_error")
    sparse_cg_max_error = _counter(sparse_cg_gpu_row, "max_result_abs_error")
    equality_max_error = _counter(equality_gpu_row, "max_result_abs_error")
    max_error = max(
        diagonal_max_error,
        scene_assembly_max_error,
        off_diagonal_max_error,
        sparse_residual_max_error,
        sparse_jacobi_max_error,
        sparse_cg_max_error,
        equality_max_error,
    )
    if max_error > tolerance:
        raise NewtonAssemblySolvePacketError(
            f"assembly/solve max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    residual_norm = _counter(gpu_row, "gpu_residual_norm")
    scene_assembly_residual_norm = _counter(scene_assembly_gpu_row, "gpu_residual_norm")
    sparse_jacobi_residual_norm = _counter(sparse_jacobi_gpu_row, "gpu_residual_norm")
    sparse_cg_residual_norm = _counter(sparse_cg_gpu_row, "gpu_residual_norm")
    equality_residual_norm = _counter(equality_gpu_row, "gpu_residual_norm")
    max_residual_norm = max(
        residual_norm,
        scene_assembly_residual_norm,
        sparse_jacobi_residual_norm,
        sparse_cg_residual_norm,
        equality_residual_norm,
    )
    if max_residual_norm > residual_tolerance:
        raise NewtonAssemblySolvePacketError(
            f"GPU residual norm {max_residual_norm:.3g} exceeds "
            f"{residual_tolerance:.3g}"
        )

    rows_count = _matching_int_counter(cpu_row, gpu_row, "rows", "gpu_rows")
    if rows_count != row_count:
        raise NewtonAssemblySolvePacketError(
            f"expected {row_count} rows, got {rows_count}"
        )
    body_count = _matching_int_counter(cpu_row, gpu_row, "bodies", "gpu_bodies")
    active_dofs = _matching_int_counter(
        cpu_row, gpu_row, "active_dofs", "gpu_active_dofs"
    )
    dofs = int(_counter(cpu_row, "dofs"))

    step_norm = _counter(gpu_row, "gpu_step_norm")
    scene_assembly_rows = _matching_int_counter(
        scene_assembly_cpu_row, scene_assembly_gpu_row, "rows", "gpu_rows"
    )
    if scene_assembly_rows <= 0:
        raise NewtonAssemblySolvePacketError("scene runtime assembly row count is zero")
    scene_assembly_bodies = _matching_int_counter(
        scene_assembly_cpu_row,
        scene_assembly_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_assembly_dofs = _matching_int_counter(
        scene_assembly_cpu_row, scene_assembly_gpu_row, "dofs", "gpu_dofs"
    )
    scene_assembly_active_dofs = _matching_int_counter(
        scene_assembly_cpu_row,
        scene_assembly_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    scene_assembly_scene_bodies = _matching_int_counter(
        scene_assembly_cpu_row,
        scene_assembly_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_assembly_scene_nodes = _matching_int_counter(
        scene_assembly_cpu_row,
        scene_assembly_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_assembly_scene_triangles = _matching_int_counter(
        scene_assembly_cpu_row,
        scene_assembly_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_assembly_step_norm = _counter(scene_assembly_gpu_row, "gpu_step_norm")
    off_diagonal_rows = _matching_int_counter(
        off_diagonal_cpu_row, off_diagonal_gpu_row, "rows", "gpu_rows"
    )
    if off_diagonal_rows != row_count:
        raise NewtonAssemblySolvePacketError(
            f"expected {row_count} off-diagonal rows, got {off_diagonal_rows}"
        )
    pair_count = _matching_int_counter(
        off_diagonal_cpu_row, off_diagonal_gpu_row, "pairs", "gpu_pairs"
    )
    active_blocks = _matching_int_counter(
        off_diagonal_cpu_row,
        off_diagonal_gpu_row,
        "active_blocks",
        "gpu_active_blocks",
    )
    block_entries = int(_counter(off_diagonal_cpu_row, "block_entries"))
    sparse_residual_rows = _matching_int_counter(
        sparse_residual_cpu_row, sparse_residual_gpu_row, "rows", "gpu_rows"
    )
    if sparse_residual_rows != row_count:
        raise NewtonAssemblySolvePacketError(
            f"expected {row_count} sparse residual rows, got {sparse_residual_rows}"
        )
    sparse_residual_bodies = _matching_int_counter(
        sparse_residual_cpu_row,
        sparse_residual_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    sparse_residual_dofs = _matching_int_counter(
        sparse_residual_cpu_row, sparse_residual_gpu_row, "dofs", "gpu_dofs"
    )
    sparse_residual_blocks = _matching_int_counter(
        sparse_residual_cpu_row,
        sparse_residual_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    sparse_residual_active_dofs = _matching_int_counter(
        sparse_residual_cpu_row,
        sparse_residual_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    sparse_residual_block_entries = int(
        _counter(sparse_residual_cpu_row, "block_entries")
    )
    sparse_residual_output_norm = _counter(sparse_residual_gpu_row, "gpu_output_norm")
    sparse_residual_max_output_abs = _counter(
        sparse_residual_gpu_row, "gpu_max_output_abs"
    )
    sparse_jacobi_rows = _matching_int_counter(
        sparse_jacobi_cpu_row, sparse_jacobi_gpu_row, "rows", "gpu_rows"
    )
    if sparse_jacobi_rows != row_count:
        raise NewtonAssemblySolvePacketError(
            f"expected {row_count} sparse Jacobi rows, got {sparse_jacobi_rows}"
        )
    sparse_jacobi_bodies = _matching_int_counter(
        sparse_jacobi_cpu_row,
        sparse_jacobi_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    sparse_jacobi_dofs = _matching_int_counter(
        sparse_jacobi_cpu_row, sparse_jacobi_gpu_row, "dofs", "gpu_dofs"
    )
    sparse_jacobi_blocks = _matching_int_counter(
        sparse_jacobi_cpu_row,
        sparse_jacobi_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    sparse_jacobi_iterations = _matching_int_counter(
        sparse_jacobi_cpu_row,
        sparse_jacobi_gpu_row,
        "iterations",
        "gpu_iterations",
    )
    sparse_jacobi_active_dofs = _matching_int_counter(
        sparse_jacobi_cpu_row,
        sparse_jacobi_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    sparse_jacobi_block_entries = int(_counter(sparse_jacobi_cpu_row, "block_entries"))
    sparse_jacobi_step_norm = _counter(sparse_jacobi_gpu_row, "gpu_step_norm")
    sparse_jacobi_max_residual_abs = _counter(
        sparse_jacobi_gpu_row, "gpu_max_residual_abs"
    )
    sparse_cg_rows = _matching_int_counter(
        sparse_cg_cpu_row, sparse_cg_gpu_row, "rows", "gpu_rows"
    )
    if sparse_cg_rows != row_count:
        raise NewtonAssemblySolvePacketError(
            f"expected {row_count} sparse CG rows, got {sparse_cg_rows}"
        )
    sparse_cg_bodies = _matching_int_counter(
        sparse_cg_cpu_row,
        sparse_cg_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    sparse_cg_dofs = _matching_int_counter(
        sparse_cg_cpu_row, sparse_cg_gpu_row, "dofs", "gpu_dofs"
    )
    sparse_cg_blocks = _matching_int_counter(
        sparse_cg_cpu_row,
        sparse_cg_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    sparse_cg_max_iterations = _matching_int_counter(
        sparse_cg_cpu_row,
        sparse_cg_gpu_row,
        "max_iterations",
        "gpu_max_iterations",
    )
    sparse_cg_completed_iterations = _matching_int_counter(
        sparse_cg_cpu_row,
        sparse_cg_gpu_row,
        "completed_iterations",
        "gpu_completed_iterations",
    )
    sparse_cg_active_dofs = _matching_int_counter(
        sparse_cg_cpu_row,
        sparse_cg_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    sparse_cg_block_entries = int(_counter(sparse_cg_cpu_row, "block_entries"))
    sparse_cg_converged = int(_counter(sparse_cg_gpu_row, "gpu_converged"))
    if sparse_cg_converged != 1:
        raise NewtonAssemblySolvePacketError("sparse CG GPU row did not converge")
    sparse_cg_residual_tolerance = _counter(sparse_cg_gpu_row, "gpu_residual_tolerance")
    if sparse_cg_residual_norm > sparse_cg_residual_tolerance:
        raise NewtonAssemblySolvePacketError(
            f"sparse CG residual norm {sparse_cg_residual_norm:.3g} exceeds "
            f"{sparse_cg_residual_tolerance:.3g}"
        )
    sparse_cg_initial_residual_norm = _counter(
        sparse_cg_gpu_row, "gpu_initial_residual_norm"
    )
    sparse_cg_step_norm = _counter(sparse_cg_gpu_row, "gpu_step_norm")
    sparse_cg_max_residual_abs = _counter(sparse_cg_gpu_row, "gpu_max_residual_abs")
    equality_rows = _matching_int_counter(
        equality_cpu_row, equality_gpu_row, "rows", "gpu_rows"
    )
    if equality_rows != row_count:
        raise NewtonAssemblySolvePacketError(
            f"expected {row_count} equality-reduced rows, got {equality_rows}"
        )
    equality_bodies = _matching_int_counter(
        equality_cpu_row, equality_gpu_row, "bodies", "gpu_bodies"
    )
    equality_dofs = _matching_int_counter(
        equality_cpu_row, equality_gpu_row, "dofs", "gpu_dofs"
    )
    reduction_entries = _matching_int_counter(
        equality_cpu_row,
        equality_gpu_row,
        "reduction_entries",
        "gpu_reduction_entries",
    )
    reduced_dofs = _matching_int_counter(
        equality_cpu_row, equality_gpu_row, "reduced_dofs", "gpu_reduced_dofs"
    )
    active_reduced_dofs = _matching_int_counter(
        equality_cpu_row,
        equality_gpu_row,
        "active_reduced_dofs",
        "gpu_active_reduced_dofs",
    )
    equality_step_norm = _counter(equality_gpu_row, "gpu_step_norm")
    off_diagonal_speedup = off_diagonal_cpu_ns / off_diagonal_gpu_ns
    diagonal_speedup = cpu_ns / gpu_ns
    scene_assembly_speedup = scene_assembly_cpu_ns / scene_assembly_gpu_ns
    sparse_residual_speedup = sparse_residual_cpu_ns / sparse_residual_gpu_ns
    sparse_jacobi_speedup = sparse_jacobi_cpu_ns / sparse_jacobi_gpu_ns
    sparse_cg_speedup = sparse_cg_cpu_ns / sparse_cg_gpu_ns
    equality_speedup = equality_cpu_ns / equality_gpu_ns
    speedup = min(
        diagonal_speedup,
        scene_assembly_speedup,
        off_diagonal_speedup,
        sparse_residual_speedup,
        sparse_jacobi_speedup,
        sparse_cg_speedup,
        equality_speedup,
    )
    timing_ns = {
        "setup": _counter(gpu_row, "host_setup_ns"),
        "host_to_device": _counter(gpu_row, "host_to_device_ns"),
        "kernel": _counter(gpu_row, "assembly_kernel_ns"),
        "solve": _counter(gpu_row, "solve_kernel_ns"),
        "device_to_host": _counter(gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = REQUIRED_TIMING_KEYS - timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"packet timing is missing {sorted(missing)}"
        )
    scene_assembly_timing_ns = {
        "setup": _counter(scene_assembly_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_assembly_gpu_row, "host_to_device_ns"),
        "kernel": _counter(scene_assembly_gpu_row, "assembly_kernel_ns"),
        "solve": _counter(scene_assembly_gpu_row, "solve_kernel_ns"),
        "device_to_host": _counter(scene_assembly_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = REQUIRED_TIMING_KEYS - scene_assembly_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime assembly packet timing is missing {sorted(missing)}"
        )
    off_diagonal_timing_ns = {
        "setup": _counter(off_diagonal_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(off_diagonal_gpu_row, "host_to_device_ns"),
        "kernel": _counter(off_diagonal_gpu_row, "assembly_kernel_ns"),
        "solve": _counter(off_diagonal_gpu_row, "solve_kernel_ns"),
        "device_to_host": _counter(off_diagonal_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = REQUIRED_TIMING_KEYS - off_diagonal_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"off-diagonal packet timing is missing {sorted(missing)}"
        )
    sparse_residual_timing_ns = {
        "setup": _counter(sparse_residual_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(sparse_residual_gpu_row, "host_to_device_ns"),
        "kernel": _counter(sparse_residual_gpu_row, "assembly_kernel_ns"),
        "gradient_seed": _counter(sparse_residual_gpu_row, "gradient_seed_ns"),
        "diagonal": _counter(sparse_residual_gpu_row, "diagonal_kernel_ns"),
        "off_diagonal": _counter(sparse_residual_gpu_row, "off_diagonal_kernel_ns"),
        "device_to_host": _counter(sparse_residual_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = SPARSE_RESIDUAL_TIMING_KEYS - sparse_residual_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"sparse residual packet timing is missing {sorted(missing)}"
        )
    sparse_jacobi_timing_ns = {
        "setup": _counter(sparse_jacobi_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(sparse_jacobi_gpu_row, "host_to_device_ns"),
        "kernel": _counter(sparse_jacobi_gpu_row, "assembly_kernel_ns"),
        "iterations": _counter(sparse_jacobi_gpu_row, "iteration_kernel_ns"),
        "final_residual": _counter(sparse_jacobi_gpu_row, "final_residual_kernel_ns"),
        "device_to_host": _counter(sparse_jacobi_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = SPARSE_JACOBI_TIMING_KEYS - sparse_jacobi_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"sparse Jacobi packet timing is missing {sorted(missing)}"
        )
    sparse_cg_timing_ns = {
        "setup": _counter(sparse_cg_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(sparse_cg_gpu_row, "host_to_device_ns"),
        "kernel": _counter(sparse_cg_gpu_row, "assembly_kernel_ns"),
        "iterations": _counter(sparse_cg_gpu_row, "iteration_kernel_ns"),
        "final_residual": _counter(sparse_cg_gpu_row, "final_residual_kernel_ns"),
        "device_to_host": _counter(sparse_cg_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = SPARSE_CG_TIMING_KEYS - sparse_cg_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"sparse CG packet timing is missing {sorted(missing)}"
        )
    equality_timing_ns = {
        "setup": _counter(equality_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(equality_gpu_row, "host_to_device_ns"),
        "kernel": _counter(equality_gpu_row, "assembly_kernel_ns"),
        "reduction": _counter(equality_gpu_row, "reduction_kernel_ns"),
        "solve": _counter(equality_gpu_row, "solve_kernel_ns"),
        "expansion": _counter(equality_gpu_row, "expansion_kernel_ns"),
        "device_to_host": _counter(equality_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = EQUALITY_TIMING_KEYS - equality_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"equality-reduced packet timing is missing {sorted(missing)}"
        )

    return {
        "newton_assembly_solve_packet": {
            "row_id": "assembly-linear-solve",
            "same_scene_cpu_gpu": True,
            "row_count": row_count,
            "body_count": body_count,
            "dof_count": dofs,
            "active_dof_count": active_dofs,
            "max_result_abs_error": max_error,
            "result_abs_error_tolerance": tolerance,
            "residual_norm": max_residual_norm,
            "residual_norm_tolerance": residual_tolerance,
            "step_norm": step_norm,
            "speedup": speedup,
            "speedup_gate": speedup_gate,
            "meets_speedup_gate": speedup >= speedup_gate,
            "timing_ns": timing_ns,
            "cpu_benchmark_row": _packet_row_name(cpu_row),
            "gpu_benchmark_row": _packet_row_name(gpu_row),
            "diagonal_assembly_solve": {
                "row_count": row_count,
                "body_count": body_count,
                "dof_count": dofs,
                "active_dof_count": active_dofs,
                "max_result_abs_error": diagonal_max_error,
                "residual_norm": residual_norm,
                "step_norm": step_norm,
                "speedup": diagonal_speedup,
                "meets_speedup_gate": diagonal_speedup >= speedup_gate,
                "timing_ns": timing_ns,
                "cpu_benchmark_row": _packet_row_name(cpu_row),
                "gpu_benchmark_row": _packet_row_name(gpu_row),
            },
            "scene_runtime_diagonal_assembly_solve": {
                "row_count": scene_assembly_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_assembly_scene_bodies,
                "scene_node_count": scene_assembly_scene_nodes,
                "scene_triangle_count": scene_assembly_scene_triangles,
                "body_count": scene_assembly_bodies,
                "dof_count": scene_assembly_dofs,
                "active_dof_count": scene_assembly_active_dofs,
                "max_result_abs_error": scene_assembly_max_error,
                "residual_norm": scene_assembly_residual_norm,
                "step_norm": scene_assembly_step_norm,
                "speedup": scene_assembly_speedup,
                "meets_speedup_gate": scene_assembly_speedup >= speedup_gate,
                "timing_ns": scene_assembly_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_assembly_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_assembly_gpu_row),
            },
            "off_diagonal_sparse_block_assembly": {
                "row_count": row_count,
                "pair_count": pair_count,
                "block_entry_count": block_entries,
                "active_block_count": active_blocks,
                "max_block_abs": _counter(off_diagonal_gpu_row, "gpu_max_block_abs"),
                "max_result_abs_error": off_diagonal_max_error,
                "speedup": off_diagonal_speedup,
                "meets_speedup_gate": off_diagonal_speedup >= speedup_gate,
                "timing_ns": off_diagonal_timing_ns,
                "cpu_benchmark_row": _packet_row_name(off_diagonal_cpu_row),
                "gpu_benchmark_row": _packet_row_name(off_diagonal_gpu_row),
            },
            "sparse_block_residual": {
                "row_count": row_count,
                "body_count": sparse_residual_bodies,
                "dof_count": sparse_residual_dofs,
                "block_count": sparse_residual_blocks,
                "block_entry_count": sparse_residual_block_entries,
                "active_dof_count": sparse_residual_active_dofs,
                "max_result_abs_error": sparse_residual_max_error,
                "output_norm": sparse_residual_output_norm,
                "max_output_abs": sparse_residual_max_output_abs,
                "speedup": sparse_residual_speedup,
                "meets_speedup_gate": sparse_residual_speedup >= speedup_gate,
                "timing_ns": sparse_residual_timing_ns,
                "cpu_benchmark_row": _packet_row_name(sparse_residual_cpu_row),
                "gpu_benchmark_row": _packet_row_name(sparse_residual_gpu_row),
            },
            "sparse_block_jacobi_solve": {
                "row_count": row_count,
                "body_count": sparse_jacobi_bodies,
                "dof_count": sparse_jacobi_dofs,
                "block_count": sparse_jacobi_blocks,
                "block_entry_count": sparse_jacobi_block_entries,
                "iteration_count": sparse_jacobi_iterations,
                "active_dof_count": sparse_jacobi_active_dofs,
                "max_result_abs_error": sparse_jacobi_max_error,
                "residual_norm": sparse_jacobi_residual_norm,
                "max_residual_abs": sparse_jacobi_max_residual_abs,
                "step_norm": sparse_jacobi_step_norm,
                "speedup": sparse_jacobi_speedup,
                "meets_speedup_gate": sparse_jacobi_speedup >= speedup_gate,
                "timing_ns": sparse_jacobi_timing_ns,
                "cpu_benchmark_row": _packet_row_name(sparse_jacobi_cpu_row),
                "gpu_benchmark_row": _packet_row_name(sparse_jacobi_gpu_row),
            },
            "sparse_block_cg_solve": {
                "row_count": row_count,
                "body_count": sparse_cg_bodies,
                "dof_count": sparse_cg_dofs,
                "block_count": sparse_cg_blocks,
                "block_entry_count": sparse_cg_block_entries,
                "max_iteration_count": sparse_cg_max_iterations,
                "completed_iteration_count": sparse_cg_completed_iterations,
                "active_dof_count": sparse_cg_active_dofs,
                "converged": bool(sparse_cg_converged),
                "residual_tolerance": sparse_cg_residual_tolerance,
                "initial_residual_norm": sparse_cg_initial_residual_norm,
                "max_result_abs_error": sparse_cg_max_error,
                "residual_norm": sparse_cg_residual_norm,
                "max_residual_abs": sparse_cg_max_residual_abs,
                "step_norm": sparse_cg_step_norm,
                "speedup": sparse_cg_speedup,
                "meets_speedup_gate": sparse_cg_speedup >= speedup_gate,
                "timing_ns": sparse_cg_timing_ns,
                "cpu_benchmark_row": _packet_row_name(sparse_cg_cpu_row),
                "gpu_benchmark_row": _packet_row_name(sparse_cg_gpu_row),
            },
            "equality_reduced_diagonal_solve": {
                "row_count": row_count,
                "body_count": equality_bodies,
                "full_dof_count": equality_dofs,
                "reduction_entry_count": reduction_entries,
                "reduced_dof_count": reduced_dofs,
                "active_reduced_dof_count": active_reduced_dofs,
                "max_result_abs_error": equality_max_error,
                "residual_norm": equality_residual_norm,
                "step_norm": equality_step_norm,
                "speedup": equality_speedup,
                "meets_speedup_gate": equality_speedup >= speedup_gate,
                "timing_ns": equality_timing_ns,
                "cpu_benchmark_row": _packet_row_name(equality_cpu_row),
                "gpu_benchmark_row": _packet_row_name(equality_gpu_row),
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
            row_count=args.row_count,
            tolerance=args.tolerance,
            residual_tolerance=args.residual_tolerance,
            speedup_gate=args.speedup_gate,
        )
    except NewtonAssemblySolvePacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["newton_assembly_solve_packet"]
    print(
        "Newton assembly/solve packet OK: "
        f"rows={row['row_count']} dofs={row['dof_count']} "
        f"max_error={row['max_result_abs_error']:.3g} "
        f"residual={row['residual_norm']:.3g} "
        f"speedup={row['speedup']:.3f}x "
        f"meets_gate={row['meets_speedup_gate']}"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
