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
DIRECT_SPARSE_DOFS_PER_BODY = 6
DIRECT_SPARSE_BLOCK_ENTRY_COUNT = DIRECT_SPARSE_DOFS_PER_BODY**2
DIRECT_SPARSE_MAX_DOF_COUNT = 48
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
SCENE_SPARSE_GRAPH_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "incidence",
    "diagonal",
    "sparse_blocks",
    "device_to_host",
    "readback",
}
SCENE_SPARSE_GRAPH_UNIQUE_TIMING_KEYS = SCENE_SPARSE_GRAPH_TIMING_KEYS | {
    "edge_mark",
}
SCENE_NONLINEAR_EQUALITY_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "constraints",
    "device_to_host",
    "readback",
}
SCENE_NONLINEAR_EQUALITY_SOLVE_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "solve",
    "post_residual",
    "device_to_host",
    "readback",
}
SCENE_NONLINEAR_EQUALITY_CONVERGENCE_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "iterations",
    "residuals",
    "convergence_readback",
    "final_assembly",
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
DIRECT_SPARSE_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "dense_matrix",
    "factor_solve",
    "final_residual",
    "device_to_host",
    "readback",
}


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
        "OffDiagonalAssembly|SceneRuntimeOffDiagonalAssembly|"
        "SceneRuntimeSparseGraphAssembly|"
        "SceneRuntimeSparseGraphUniqueAssembly|"
        "SceneRuntimeNonlinearEqualityAssembly|"
        "SceneRuntimeNonlinearEqualitySolve|"
        "SceneRuntimeNonlinearEqualityConvergence|SparseResidual|"
        "SceneRuntimeSparseResidual|SparseJacobiSolve|SceneRuntimeSparseJacobiSolve|"
        "SparseCgSolve|SceneRuntimeSparseCgSolve|DirectSparseSolve|"
        "SceneRuntimeDirectSparseSolve|EqualityReducedSolve|"
        "SceneRuntimeEqualityReducedSolve)(Cpu|Cuda)"
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
        "scene_off_diagonal_cpu": (
            f"BM_NewtonSceneRuntimeOffDiagonalAssemblyCpu/{row_count}"
        ),
        "scene_off_diagonal_gpu": (
            f"BM_NewtonSceneRuntimeOffDiagonalAssemblyCuda/{row_count}"
        ),
        "scene_sparse_graph_cpu": (
            f"BM_NewtonSceneRuntimeSparseGraphAssemblyCpu/{row_count}"
        ),
        "scene_sparse_graph_gpu": (
            f"BM_NewtonSceneRuntimeSparseGraphAssemblyCuda/{row_count}"
        ),
        "scene_sparse_graph_unique_cpu": (
            f"BM_NewtonSceneRuntimeSparseGraphUniqueAssemblyCpu/{row_count}"
        ),
        "scene_sparse_graph_unique_gpu": (
            f"BM_NewtonSceneRuntimeSparseGraphUniqueAssemblyCuda/{row_count}"
        ),
        "scene_nonlinear_equality_cpu": (
            f"BM_NewtonSceneRuntimeNonlinearEqualityAssemblyCpu/{row_count}"
        ),
        "scene_nonlinear_equality_gpu": (
            f"BM_NewtonSceneRuntimeNonlinearEqualityAssemblyCuda/{row_count}"
        ),
        "scene_nonlinear_equality_solve_cpu": (
            f"BM_NewtonSceneRuntimeNonlinearEqualitySolveCpu/{row_count}"
        ),
        "scene_nonlinear_equality_solve_gpu": (
            f"BM_NewtonSceneRuntimeNonlinearEqualitySolveCuda/{row_count}"
        ),
        "scene_nonlinear_equality_convergence_cpu": (
            f"BM_NewtonSceneRuntimeNonlinearEqualityConvergenceCpu/{row_count}"
        ),
        "scene_nonlinear_equality_convergence_gpu": (
            f"BM_NewtonSceneRuntimeNonlinearEqualityConvergenceCuda/{row_count}"
        ),
        "sparse_residual_cpu": f"BM_NewtonSparseResidualCpu/{row_count}",
        "sparse_residual_gpu": f"BM_NewtonSparseResidualCuda/{row_count}",
        "scene_sparse_residual_cpu": (
            f"BM_NewtonSceneRuntimeSparseResidualCpu/{row_count}"
        ),
        "scene_sparse_residual_gpu": (
            f"BM_NewtonSceneRuntimeSparseResidualCuda/{row_count}"
        ),
        "sparse_jacobi_cpu": f"BM_NewtonSparseJacobiSolveCpu/{row_count}",
        "sparse_jacobi_gpu": f"BM_NewtonSparseJacobiSolveCuda/{row_count}",
        "scene_sparse_jacobi_cpu": (
            f"BM_NewtonSceneRuntimeSparseJacobiSolveCpu/{row_count}"
        ),
        "scene_sparse_jacobi_gpu": (
            f"BM_NewtonSceneRuntimeSparseJacobiSolveCuda/{row_count}"
        ),
        "sparse_cg_cpu": f"BM_NewtonSparseCgSolveCpu/{row_count}",
        "sparse_cg_gpu": f"BM_NewtonSparseCgSolveCuda/{row_count}",
        "scene_sparse_cg_cpu": (f"BM_NewtonSceneRuntimeSparseCgSolveCpu/{row_count}"),
        "scene_sparse_cg_gpu": (f"BM_NewtonSceneRuntimeSparseCgSolveCuda/{row_count}"),
        "direct_sparse_cpu": f"BM_NewtonDirectSparseSolveCpu/{row_count}",
        "direct_sparse_gpu": f"BM_NewtonDirectSparseSolveCuda/{row_count}",
        "scene_direct_sparse_cpu": (
            f"BM_NewtonSceneRuntimeDirectSparseSolveCpu/{row_count}"
        ),
        "scene_direct_sparse_gpu": (
            f"BM_NewtonSceneRuntimeDirectSparseSolveCuda/{row_count}"
        ),
        "equality_cpu": f"BM_NewtonEqualityReducedSolveCpu/{row_count}",
        "equality_gpu": f"BM_NewtonEqualityReducedSolveCuda/{row_count}",
        "scene_equality_cpu": (
            f"BM_NewtonSceneRuntimeEqualityReducedSolveCpu/{row_count}"
        ),
        "scene_equality_gpu": (
            f"BM_NewtonSceneRuntimeEqualityReducedSolveCuda/{row_count}"
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
    scene_off_diagonal_cpu_row = representative_rows["scene_off_diagonal_cpu"]
    scene_off_diagonal_gpu_row = representative_rows["scene_off_diagonal_gpu"]
    scene_sparse_graph_cpu_row = representative_rows["scene_sparse_graph_cpu"]
    scene_sparse_graph_gpu_row = representative_rows["scene_sparse_graph_gpu"]
    scene_sparse_graph_unique_cpu_row = representative_rows[
        "scene_sparse_graph_unique_cpu"
    ]
    scene_sparse_graph_unique_gpu_row = representative_rows[
        "scene_sparse_graph_unique_gpu"
    ]
    scene_nonlinear_equality_cpu_row = representative_rows[
        "scene_nonlinear_equality_cpu"
    ]
    scene_nonlinear_equality_gpu_row = representative_rows[
        "scene_nonlinear_equality_gpu"
    ]
    scene_nonlinear_equality_solve_cpu_row = representative_rows[
        "scene_nonlinear_equality_solve_cpu"
    ]
    scene_nonlinear_equality_solve_gpu_row = representative_rows[
        "scene_nonlinear_equality_solve_gpu"
    ]
    scene_nonlinear_equality_convergence_cpu_row = representative_rows[
        "scene_nonlinear_equality_convergence_cpu"
    ]
    scene_nonlinear_equality_convergence_gpu_row = representative_rows[
        "scene_nonlinear_equality_convergence_gpu"
    ]
    sparse_residual_cpu_row = representative_rows["sparse_residual_cpu"]
    sparse_residual_gpu_row = representative_rows["sparse_residual_gpu"]
    scene_sparse_residual_cpu_row = representative_rows["scene_sparse_residual_cpu"]
    scene_sparse_residual_gpu_row = representative_rows["scene_sparse_residual_gpu"]
    sparse_jacobi_cpu_row = representative_rows["sparse_jacobi_cpu"]
    sparse_jacobi_gpu_row = representative_rows["sparse_jacobi_gpu"]
    scene_sparse_jacobi_cpu_row = representative_rows["scene_sparse_jacobi_cpu"]
    scene_sparse_jacobi_gpu_row = representative_rows["scene_sparse_jacobi_gpu"]
    sparse_cg_cpu_row = representative_rows["sparse_cg_cpu"]
    sparse_cg_gpu_row = representative_rows["sparse_cg_gpu"]
    scene_sparse_cg_cpu_row = representative_rows["scene_sparse_cg_cpu"]
    scene_sparse_cg_gpu_row = representative_rows["scene_sparse_cg_gpu"]
    direct_sparse_cpu_row = representative_rows["direct_sparse_cpu"]
    direct_sparse_gpu_row = representative_rows["direct_sparse_gpu"]
    scene_direct_sparse_cpu_row = representative_rows["scene_direct_sparse_cpu"]
    scene_direct_sparse_gpu_row = representative_rows["scene_direct_sparse_gpu"]
    equality_cpu_row = representative_rows["equality_cpu"]
    equality_gpu_row = representative_rows["equality_gpu"]
    scene_equality_cpu_row = representative_rows["scene_equality_cpu"]
    scene_equality_gpu_row = representative_rows["scene_equality_gpu"]
    cpu_ns = benchmark_timing_ns(cpu_row)
    gpu_ns = benchmark_timing_ns(gpu_row)
    scene_assembly_cpu_ns = benchmark_timing_ns(scene_assembly_cpu_row)
    scene_assembly_gpu_ns = benchmark_timing_ns(scene_assembly_gpu_row)
    off_diagonal_cpu_ns = benchmark_timing_ns(off_diagonal_cpu_row)
    off_diagonal_gpu_ns = benchmark_timing_ns(off_diagonal_gpu_row)
    scene_off_diagonal_cpu_ns = benchmark_timing_ns(scene_off_diagonal_cpu_row)
    scene_off_diagonal_gpu_ns = benchmark_timing_ns(scene_off_diagonal_gpu_row)
    scene_sparse_graph_cpu_ns = benchmark_timing_ns(scene_sparse_graph_cpu_row)
    scene_sparse_graph_gpu_ns = benchmark_timing_ns(scene_sparse_graph_gpu_row)
    scene_sparse_graph_unique_cpu_ns = benchmark_timing_ns(
        scene_sparse_graph_unique_cpu_row
    )
    scene_sparse_graph_unique_gpu_ns = benchmark_timing_ns(
        scene_sparse_graph_unique_gpu_row
    )
    scene_nonlinear_equality_cpu_ns = benchmark_timing_ns(
        scene_nonlinear_equality_cpu_row
    )
    scene_nonlinear_equality_gpu_ns = benchmark_timing_ns(
        scene_nonlinear_equality_gpu_row
    )
    scene_nonlinear_equality_solve_cpu_ns = benchmark_timing_ns(
        scene_nonlinear_equality_solve_cpu_row
    )
    scene_nonlinear_equality_solve_gpu_ns = benchmark_timing_ns(
        scene_nonlinear_equality_solve_gpu_row
    )
    scene_nonlinear_equality_convergence_cpu_ns = benchmark_timing_ns(
        scene_nonlinear_equality_convergence_cpu_row
    )
    scene_nonlinear_equality_convergence_gpu_ns = benchmark_timing_ns(
        scene_nonlinear_equality_convergence_gpu_row
    )
    sparse_residual_cpu_ns = benchmark_timing_ns(sparse_residual_cpu_row)
    sparse_residual_gpu_ns = benchmark_timing_ns(sparse_residual_gpu_row)
    scene_sparse_residual_cpu_ns = benchmark_timing_ns(scene_sparse_residual_cpu_row)
    scene_sparse_residual_gpu_ns = benchmark_timing_ns(scene_sparse_residual_gpu_row)
    sparse_jacobi_cpu_ns = benchmark_timing_ns(sparse_jacobi_cpu_row)
    sparse_jacobi_gpu_ns = benchmark_timing_ns(sparse_jacobi_gpu_row)
    scene_sparse_jacobi_cpu_ns = benchmark_timing_ns(scene_sparse_jacobi_cpu_row)
    scene_sparse_jacobi_gpu_ns = benchmark_timing_ns(scene_sparse_jacobi_gpu_row)
    sparse_cg_cpu_ns = benchmark_timing_ns(sparse_cg_cpu_row)
    sparse_cg_gpu_ns = benchmark_timing_ns(sparse_cg_gpu_row)
    scene_sparse_cg_cpu_ns = benchmark_timing_ns(scene_sparse_cg_cpu_row)
    scene_sparse_cg_gpu_ns = benchmark_timing_ns(scene_sparse_cg_gpu_row)
    direct_sparse_cpu_ns = benchmark_timing_ns(direct_sparse_cpu_row)
    direct_sparse_gpu_ns = benchmark_timing_ns(direct_sparse_gpu_row)
    scene_direct_sparse_cpu_ns = benchmark_timing_ns(scene_direct_sparse_cpu_row)
    scene_direct_sparse_gpu_ns = benchmark_timing_ns(scene_direct_sparse_gpu_row)
    equality_cpu_ns = benchmark_timing_ns(equality_cpu_row)
    equality_gpu_ns = benchmark_timing_ns(equality_gpu_row)
    scene_equality_cpu_ns = benchmark_timing_ns(scene_equality_cpu_row)
    scene_equality_gpu_ns = benchmark_timing_ns(scene_equality_gpu_row)
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
    if not math.isfinite(scene_off_diagonal_cpu_ns) or scene_off_diagonal_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime off-diagonal CPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_off_diagonal_gpu_ns) or scene_off_diagonal_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime off-diagonal GPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_sparse_graph_cpu_ns) or scene_sparse_graph_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse graph CPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_sparse_graph_gpu_ns) or scene_sparse_graph_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse graph GPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_sparse_graph_unique_cpu_ns)
        or scene_sparse_graph_unique_cpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph CPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_sparse_graph_unique_gpu_ns)
        or scene_sparse_graph_unique_gpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph GPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_nonlinear_equality_cpu_ns)
        or scene_nonlinear_equality_cpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality CPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_nonlinear_equality_gpu_ns)
        or scene_nonlinear_equality_gpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality GPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_nonlinear_equality_solve_cpu_ns)
        or scene_nonlinear_equality_solve_cpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve CPU benchmark timing is not "
            "positive"
        )
    if (
        not math.isfinite(scene_nonlinear_equality_solve_gpu_ns)
        or scene_nonlinear_equality_solve_gpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve GPU benchmark timing is not "
            "positive"
        )
    if (
        not math.isfinite(scene_nonlinear_equality_convergence_cpu_ns)
        or scene_nonlinear_equality_convergence_cpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence CPU benchmark timing "
            "is not positive"
        )
    if (
        not math.isfinite(scene_nonlinear_equality_convergence_gpu_ns)
        or scene_nonlinear_equality_convergence_gpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence GPU benchmark timing "
            "is not positive"
        )
    if not math.isfinite(sparse_residual_cpu_ns) or sparse_residual_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse residual CPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_residual_gpu_ns) or sparse_residual_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse residual GPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_sparse_residual_cpu_ns)
        or scene_sparse_residual_cpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse residual CPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_sparse_residual_gpu_ns)
        or scene_sparse_residual_gpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse residual GPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_jacobi_cpu_ns) or sparse_jacobi_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse Jacobi CPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_jacobi_gpu_ns) or sparse_jacobi_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse Jacobi GPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_sparse_jacobi_cpu_ns)
        or scene_sparse_jacobi_cpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse Jacobi CPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_sparse_jacobi_gpu_ns)
        or scene_sparse_jacobi_gpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse Jacobi GPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_cg_cpu_ns) or sparse_cg_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse CG CPU benchmark timing is not positive"
        )
    if not math.isfinite(sparse_cg_gpu_ns) or sparse_cg_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "sparse CG GPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_sparse_cg_cpu_ns) or scene_sparse_cg_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse CG CPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_sparse_cg_gpu_ns) or scene_sparse_cg_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse CG GPU benchmark timing is not positive"
        )
    if not math.isfinite(direct_sparse_cpu_ns) or direct_sparse_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "direct sparse CPU benchmark timing is not positive"
        )
    if not math.isfinite(direct_sparse_gpu_ns) or direct_sparse_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "direct sparse GPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_direct_sparse_cpu_ns)
        or scene_direct_sparse_cpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse CPU benchmark timing is not positive"
        )
    if (
        not math.isfinite(scene_direct_sparse_gpu_ns)
        or scene_direct_sparse_gpu_ns <= 0.0
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse GPU benchmark timing is not positive"
        )
    if not math.isfinite(equality_cpu_ns) or equality_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "equality-reduced CPU benchmark timing is not positive"
        )
    if not math.isfinite(equality_gpu_ns) or equality_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "equality-reduced GPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_equality_cpu_ns) or scene_equality_cpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime equality-reduced CPU benchmark timing is not positive"
        )
    if not math.isfinite(scene_equality_gpu_ns) or scene_equality_gpu_ns <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime equality-reduced GPU benchmark timing is not positive"
        )

    diagonal_max_error = _counter(gpu_row, "max_result_abs_error")
    scene_assembly_max_error = _counter(scene_assembly_gpu_row, "max_result_abs_error")
    off_diagonal_max_error = _counter(off_diagonal_gpu_row, "max_result_abs_error")
    scene_off_diagonal_max_error = _counter(
        scene_off_diagonal_gpu_row, "max_result_abs_error"
    )
    scene_sparse_graph_max_error = _counter(
        scene_sparse_graph_gpu_row, "max_result_abs_error"
    )
    scene_sparse_graph_unique_max_error = _counter(
        scene_sparse_graph_unique_gpu_row, "max_result_abs_error"
    )
    scene_nonlinear_equality_max_error = _counter(
        scene_nonlinear_equality_gpu_row, "max_result_abs_error"
    )
    scene_nonlinear_equality_solve_max_error = _counter(
        scene_nonlinear_equality_solve_gpu_row, "max_result_abs_error"
    )
    scene_nonlinear_equality_convergence_max_error = _counter(
        scene_nonlinear_equality_convergence_gpu_row, "max_result_abs_error"
    )
    sparse_residual_max_error = _counter(
        sparse_residual_gpu_row, "max_result_abs_error"
    )
    scene_sparse_residual_max_error = _counter(
        scene_sparse_residual_gpu_row, "max_result_abs_error"
    )
    sparse_jacobi_max_error = _counter(sparse_jacobi_gpu_row, "max_result_abs_error")
    scene_sparse_jacobi_max_error = _counter(
        scene_sparse_jacobi_gpu_row, "max_result_abs_error"
    )
    sparse_cg_max_error = _counter(sparse_cg_gpu_row, "max_result_abs_error")
    scene_sparse_cg_max_error = _counter(
        scene_sparse_cg_gpu_row, "max_result_abs_error"
    )
    direct_sparse_max_error = _counter(direct_sparse_gpu_row, "max_result_abs_error")
    scene_direct_sparse_max_error = _counter(
        scene_direct_sparse_gpu_row, "max_result_abs_error"
    )
    equality_max_error = _counter(equality_gpu_row, "max_result_abs_error")
    scene_equality_max_error = _counter(scene_equality_gpu_row, "max_result_abs_error")
    max_error = max(
        diagonal_max_error,
        scene_assembly_max_error,
        off_diagonal_max_error,
        scene_off_diagonal_max_error,
        scene_sparse_graph_max_error,
        scene_sparse_graph_unique_max_error,
        scene_nonlinear_equality_max_error,
        scene_nonlinear_equality_solve_max_error,
        scene_nonlinear_equality_convergence_max_error,
        sparse_residual_max_error,
        scene_sparse_residual_max_error,
        sparse_jacobi_max_error,
        scene_sparse_jacobi_max_error,
        sparse_cg_max_error,
        scene_sparse_cg_max_error,
        direct_sparse_max_error,
        scene_direct_sparse_max_error,
        equality_max_error,
        scene_equality_max_error,
    )
    if max_error > tolerance:
        raise NewtonAssemblySolvePacketError(
            f"assembly/solve max error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    residual_norm = _counter(gpu_row, "gpu_residual_norm")
    scene_assembly_residual_norm = _counter(scene_assembly_gpu_row, "gpu_residual_norm")
    sparse_jacobi_residual_norm = _counter(sparse_jacobi_gpu_row, "gpu_residual_norm")
    scene_sparse_jacobi_residual_norm = _counter(
        scene_sparse_jacobi_gpu_row, "gpu_residual_norm"
    )
    sparse_cg_residual_norm = _counter(sparse_cg_gpu_row, "gpu_residual_norm")
    scene_sparse_cg_residual_norm = _counter(
        scene_sparse_cg_gpu_row, "gpu_residual_norm"
    )
    direct_sparse_residual_norm = _counter(direct_sparse_gpu_row, "gpu_residual_norm")
    scene_direct_sparse_residual_norm = _counter(
        scene_direct_sparse_gpu_row, "gpu_residual_norm"
    )
    equality_residual_norm = _counter(equality_gpu_row, "gpu_residual_norm")
    scene_equality_residual_norm = _counter(scene_equality_gpu_row, "gpu_residual_norm")
    max_residual_norm = max(
        residual_norm,
        scene_assembly_residual_norm,
        sparse_jacobi_residual_norm,
        scene_sparse_jacobi_residual_norm,
        sparse_cg_residual_norm,
        scene_sparse_cg_residual_norm,
        direct_sparse_residual_norm,
        scene_direct_sparse_residual_norm,
        equality_residual_norm,
        scene_equality_residual_norm,
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
    scene_off_diagonal_rows = _matching_int_counter(
        scene_off_diagonal_cpu_row,
        scene_off_diagonal_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_off_diagonal_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime off-diagonal row count is zero"
        )
    scene_off_diagonal_pairs = _matching_int_counter(
        scene_off_diagonal_cpu_row,
        scene_off_diagonal_gpu_row,
        "pairs",
        "gpu_pairs",
    )
    scene_off_diagonal_active_blocks = _matching_int_counter(
        scene_off_diagonal_cpu_row,
        scene_off_diagonal_gpu_row,
        "active_blocks",
        "gpu_active_blocks",
    )
    scene_off_diagonal_block_entries = int(
        _counter(scene_off_diagonal_cpu_row, "block_entries")
    )
    scene_off_diagonal_scene_bodies = _matching_int_counter(
        scene_off_diagonal_cpu_row,
        scene_off_diagonal_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_off_diagonal_scene_nodes = _matching_int_counter(
        scene_off_diagonal_cpu_row,
        scene_off_diagonal_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_off_diagonal_scene_triangles = _matching_int_counter(
        scene_off_diagonal_cpu_row,
        scene_off_diagonal_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_off_diagonal_scene_edge_pairs = _matching_int_counter(
        scene_off_diagonal_cpu_row,
        scene_off_diagonal_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if scene_off_diagonal_scene_edge_pairs != scene_off_diagonal_pairs:
        raise NewtonAssemblySolvePacketError(
            "scene runtime off-diagonal edge-pair count "
            f"{scene_off_diagonal_scene_edge_pairs} != pairs "
            f"{scene_off_diagonal_pairs}"
        )
    scene_sparse_graph_rows = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_sparse_graph_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse graph row count is zero"
        )
    scene_sparse_graph_bodies = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_sparse_graph_dofs = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_sparse_graph_blocks = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_sparse_graph_block_entries = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "block_entries",
        "gpu_block_entries",
    )
    scene_sparse_graph_scene_bodies = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_sparse_graph_scene_nodes = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_sparse_graph_scene_triangles = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_sparse_graph_scene_edge_pairs = _matching_int_counter(
        scene_sparse_graph_cpu_row,
        scene_sparse_graph_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if scene_sparse_graph_scene_nodes != scene_sparse_graph_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse graph node count "
            f"{scene_sparse_graph_scene_nodes} != bodies "
            f"{scene_sparse_graph_bodies}"
        )
    if scene_sparse_graph_scene_edge_pairs != scene_sparse_graph_blocks:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse graph edge-pair count "
            f"{scene_sparse_graph_scene_edge_pairs} != blocks "
            f"{scene_sparse_graph_blocks}"
        )
    if scene_sparse_graph_scene_triangles * 3 != scene_sparse_graph_scene_edge_pairs:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse graph edge pairs do not match three "
            "oriented edges per triangle"
        )
    scene_sparse_graph_max_diagonal = _counter(
        scene_sparse_graph_gpu_row, "gpu_max_diagonal"
    )
    scene_sparse_graph_max_gradient_abs = _counter(
        scene_sparse_graph_gpu_row, "gpu_max_gradient_abs"
    )
    scene_sparse_graph_max_block_abs = _counter(
        scene_sparse_graph_gpu_row, "gpu_max_block_abs"
    )
    scene_sparse_graph_unique_rows = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_sparse_graph_unique_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph row count is zero"
        )
    scene_sparse_graph_unique_bodies = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_sparse_graph_unique_dofs = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_sparse_graph_unique_blocks = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_sparse_graph_unique_block_entries = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "block_entries",
        "gpu_block_entries",
    )
    scene_sparse_graph_unique_edge_slots = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "edge_slots",
        "gpu_edge_slots",
    )
    scene_sparse_graph_unique_unique_edges = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "unique_edges",
        "gpu_unique_edges",
    )
    scene_sparse_graph_unique_duplicate_edge_slots = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "duplicate_edge_slots",
        "gpu_duplicate_edge_slots",
    )
    scene_sparse_graph_unique_scene_bodies = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_sparse_graph_unique_scene_nodes = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_sparse_graph_unique_scene_triangles = _matching_int_counter(
        scene_sparse_graph_unique_cpu_row,
        scene_sparse_graph_unique_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    if scene_sparse_graph_unique_scene_nodes != scene_sparse_graph_unique_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph node count "
            f"{scene_sparse_graph_unique_scene_nodes} != bodies "
            f"{scene_sparse_graph_unique_bodies}"
        )
    if scene_sparse_graph_unique_edge_slots != (
        scene_sparse_graph_unique_scene_triangles * 3
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph edge slots do not match three "
            "oriented edges per triangle"
        )
    if (
        scene_sparse_graph_unique_duplicate_edge_slots
        != scene_sparse_graph_unique_edge_slots - scene_sparse_graph_unique_unique_edges
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph duplicate edge slots do not match "
            "edge slots minus unique edges"
        )
    if scene_sparse_graph_unique_unique_edges != scene_sparse_graph_unique_blocks:
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph unique edge count "
            f"{scene_sparse_graph_unique_unique_edges} != blocks "
            f"{scene_sparse_graph_unique_blocks}"
        )
    if scene_sparse_graph_unique_block_entries != (
        scene_sparse_graph_unique_blocks * 36
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph block entries do not match "
            "36 entries per 6x6 block"
        )
    if scene_sparse_graph_unique_blocks >= scene_sparse_graph_unique_edge_slots:
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph row did not deduplicate any "
            "oriented edge slots"
        )
    scene_sparse_graph_unique_max_diagonal = _counter(
        scene_sparse_graph_unique_gpu_row, "gpu_max_diagonal"
    )
    scene_sparse_graph_unique_max_gradient_abs = _counter(
        scene_sparse_graph_unique_gpu_row, "gpu_max_gradient_abs"
    )
    scene_sparse_graph_unique_max_block_abs = _counter(
        scene_sparse_graph_unique_gpu_row, "gpu_max_block_abs"
    )
    scene_nonlinear_equality_rows = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_nonlinear_equality_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality row count is zero"
        )
    scene_nonlinear_equality_bodies = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_nonlinear_equality_dofs = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_nonlinear_equality_constraints = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "constraints",
        "gpu_constraints",
    )
    scene_nonlinear_equality_blocks = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_nonlinear_equality_block_entries = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "block_entries",
        "gpu_block_entries",
    )
    scene_nonlinear_equality_scene_bodies = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_nonlinear_equality_scene_nodes = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_nonlinear_equality_scene_triangles = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_nonlinear_equality_scene_edge_pairs = _matching_int_counter(
        scene_nonlinear_equality_cpu_row,
        scene_nonlinear_equality_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if scene_nonlinear_equality_scene_nodes != scene_nonlinear_equality_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality node count "
            f"{scene_nonlinear_equality_scene_nodes} != bodies "
            f"{scene_nonlinear_equality_bodies}"
        )
    if (
        scene_nonlinear_equality_scene_edge_pairs
        != scene_nonlinear_equality_constraints
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality edge-pair count "
            f"{scene_nonlinear_equality_scene_edge_pairs} != constraints "
            f"{scene_nonlinear_equality_constraints}"
        )
    if scene_nonlinear_equality_constraints != scene_nonlinear_equality_blocks:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality constraint count "
            f"{scene_nonlinear_equality_constraints} != blocks "
            f"{scene_nonlinear_equality_blocks}"
        )
    if 2 * scene_nonlinear_equality_constraints != scene_nonlinear_equality_rows:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality rows do not match two rows per "
            "constraint"
        )
    if (
        scene_nonlinear_equality_scene_triangles * 3
        != scene_nonlinear_equality_scene_edge_pairs
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality edge pairs do not match three "
            "oriented edges per triangle"
        )
    scene_nonlinear_equality_max_constraint_residual_abs = _counter(
        scene_nonlinear_equality_gpu_row, "gpu_max_constraint_residual_abs"
    )
    scene_nonlinear_equality_max_diagonal = _counter(
        scene_nonlinear_equality_gpu_row, "gpu_max_diagonal"
    )
    scene_nonlinear_equality_max_gradient_abs = _counter(
        scene_nonlinear_equality_gpu_row, "gpu_max_gradient_abs"
    )
    scene_nonlinear_equality_max_block_abs = _counter(
        scene_nonlinear_equality_gpu_row, "gpu_max_block_abs"
    )
    scene_nonlinear_equality_solve_rows = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_nonlinear_equality_solve_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve row count is zero"
        )
    scene_nonlinear_equality_solve_bodies = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_nonlinear_equality_solve_dofs = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_nonlinear_equality_solve_constraints = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "constraints",
        "gpu_constraints",
    )
    scene_nonlinear_equality_solve_blocks = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_nonlinear_equality_solve_block_entries = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "block_entries",
        "gpu_block_entries",
    )
    scene_nonlinear_equality_solve_active_dofs = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    if scene_nonlinear_equality_solve_active_dofs <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve active dof count is zero"
        )
    scene_nonlinear_equality_solve_scene_bodies = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_nonlinear_equality_solve_scene_nodes = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_nonlinear_equality_solve_scene_triangles = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_nonlinear_equality_solve_scene_edge_pairs = _matching_int_counter(
        scene_nonlinear_equality_solve_cpu_row,
        scene_nonlinear_equality_solve_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if (
        scene_nonlinear_equality_solve_scene_nodes
        != scene_nonlinear_equality_solve_bodies
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve node count "
            f"{scene_nonlinear_equality_solve_scene_nodes} != bodies "
            f"{scene_nonlinear_equality_solve_bodies}"
        )
    if (
        scene_nonlinear_equality_solve_scene_edge_pairs
        != scene_nonlinear_equality_solve_constraints
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve edge-pair count "
            f"{scene_nonlinear_equality_solve_scene_edge_pairs} != constraints "
            f"{scene_nonlinear_equality_solve_constraints}"
        )
    if (
        scene_nonlinear_equality_solve_constraints
        != scene_nonlinear_equality_solve_blocks
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve constraint count "
            f"{scene_nonlinear_equality_solve_constraints} != blocks "
            f"{scene_nonlinear_equality_solve_blocks}"
        )
    if (
        2 * scene_nonlinear_equality_solve_constraints
        != scene_nonlinear_equality_solve_rows
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve rows do not match two "
            "rows per constraint"
        )
    if (
        scene_nonlinear_equality_solve_scene_triangles * 3
        != scene_nonlinear_equality_solve_scene_edge_pairs
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve edge pairs do not match "
            "three oriented edges per triangle"
        )
    scene_nonlinear_equality_solve_regularization = _counter(
        scene_nonlinear_equality_solve_cpu_row, "regularization"
    )
    scene_nonlinear_equality_solve_gpu_regularization = _counter(
        scene_nonlinear_equality_solve_gpu_row, "gpu_regularization"
    )
    if (
        abs(
            scene_nonlinear_equality_solve_regularization
            - scene_nonlinear_equality_solve_gpu_regularization
        )
        > 1e-15
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve regularization mismatch"
        )
    scene_nonlinear_equality_solve_max_constraint_residual_abs = _counter(
        scene_nonlinear_equality_solve_gpu_row,
        "gpu_max_constraint_residual_abs",
    )
    scene_nonlinear_equality_solve_max_post_residual_abs = _counter(
        scene_nonlinear_equality_solve_gpu_row,
        "gpu_max_post_solve_linearized_residual_abs",
    )
    scene_nonlinear_equality_solve_max_diagonal = _counter(
        scene_nonlinear_equality_solve_gpu_row, "gpu_max_diagonal"
    )
    scene_nonlinear_equality_solve_max_gradient_abs = _counter(
        scene_nonlinear_equality_solve_gpu_row, "gpu_max_gradient_abs"
    )
    scene_nonlinear_equality_solve_max_block_abs = _counter(
        scene_nonlinear_equality_solve_gpu_row, "gpu_max_block_abs"
    )
    scene_nonlinear_equality_solve_step_norm = _counter(
        scene_nonlinear_equality_solve_gpu_row, "gpu_step_norm"
    )
    scene_nonlinear_equality_convergence_rows = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_nonlinear_equality_convergence_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence row count is zero"
        )
    scene_nonlinear_equality_convergence_bodies = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_nonlinear_equality_convergence_dofs = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_nonlinear_equality_convergence_constraints = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "constraints",
        "gpu_constraints",
    )
    scene_nonlinear_equality_convergence_blocks = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_nonlinear_equality_convergence_block_entries = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "block_entries",
        "gpu_block_entries",
    )
    scene_nonlinear_equality_convergence_active_dofs = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    if scene_nonlinear_equality_convergence_active_dofs <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence active dof count is zero"
        )
    scene_nonlinear_equality_convergence_max_iterations = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "max_iterations",
        "gpu_max_iterations",
    )
    scene_nonlinear_equality_convergence_completed_iterations = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "completed_iterations",
        "gpu_completed_iterations",
    )
    if scene_nonlinear_equality_convergence_max_iterations <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence iteration cap is zero"
        )
    if (
        scene_nonlinear_equality_convergence_completed_iterations
        > scene_nonlinear_equality_convergence_max_iterations
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence completed iterations "
            "exceed the cap"
        )
    scene_nonlinear_equality_convergence_scene_bodies = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_nonlinear_equality_convergence_scene_nodes = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_nonlinear_equality_convergence_scene_triangles = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_nonlinear_equality_convergence_scene_edge_pairs = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if (
        scene_nonlinear_equality_convergence_scene_nodes
        != scene_nonlinear_equality_convergence_bodies
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence node count "
            f"{scene_nonlinear_equality_convergence_scene_nodes} != bodies "
            f"{scene_nonlinear_equality_convergence_bodies}"
        )
    if (
        scene_nonlinear_equality_convergence_scene_edge_pairs
        != scene_nonlinear_equality_convergence_constraints
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence edge-pair count "
            f"{scene_nonlinear_equality_convergence_scene_edge_pairs} != "
            f"constraints {scene_nonlinear_equality_convergence_constraints}"
        )
    if (
        scene_nonlinear_equality_convergence_constraints
        != scene_nonlinear_equality_convergence_blocks
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence constraint count "
            f"{scene_nonlinear_equality_convergence_constraints} != blocks "
            f"{scene_nonlinear_equality_convergence_blocks}"
        )
    if (
        2 * scene_nonlinear_equality_convergence_constraints
        != scene_nonlinear_equality_convergence_rows
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence rows do not match "
            "two rows per constraint"
        )
    if (
        scene_nonlinear_equality_convergence_scene_triangles * 3
        != scene_nonlinear_equality_convergence_scene_edge_pairs
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence edge pairs do not "
            "match three oriented edges per triangle"
        )
    scene_nonlinear_equality_convergence_regularization = _counter(
        scene_nonlinear_equality_convergence_cpu_row, "regularization"
    )
    scene_nonlinear_equality_convergence_gpu_regularization = _counter(
        scene_nonlinear_equality_convergence_gpu_row, "gpu_regularization"
    )
    if (
        abs(
            scene_nonlinear_equality_convergence_regularization
            - scene_nonlinear_equality_convergence_gpu_regularization
        )
        > 1e-15
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence regularization mismatch"
        )
    scene_nonlinear_equality_convergence_residual_tolerance = _counter(
        scene_nonlinear_equality_convergence_cpu_row, "residual_tolerance"
    )
    scene_nonlinear_equality_convergence_gpu_residual_tolerance = _counter(
        scene_nonlinear_equality_convergence_gpu_row, "gpu_residual_tolerance"
    )
    if (
        abs(
            scene_nonlinear_equality_convergence_residual_tolerance
            - scene_nonlinear_equality_convergence_gpu_residual_tolerance
        )
        > 1e-15
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence tolerance mismatch"
        )
    scene_nonlinear_equality_convergence_converged = _matching_int_counter(
        scene_nonlinear_equality_convergence_cpu_row,
        scene_nonlinear_equality_convergence_gpu_row,
        "converged",
        "gpu_converged",
    )
    scene_nonlinear_equality_convergence_initial_max_residual_abs = _counter(
        scene_nonlinear_equality_convergence_gpu_row,
        "gpu_initial_max_constraint_residual_abs",
    )
    scene_nonlinear_equality_convergence_final_max_residual_abs = _counter(
        scene_nonlinear_equality_convergence_gpu_row,
        "gpu_final_max_constraint_residual_abs",
    )
    scene_nonlinear_equality_convergence_max_diagonal = _counter(
        scene_nonlinear_equality_convergence_gpu_row, "gpu_max_diagonal"
    )
    scene_nonlinear_equality_convergence_max_gradient_abs = _counter(
        scene_nonlinear_equality_convergence_gpu_row, "gpu_max_gradient_abs"
    )
    scene_nonlinear_equality_convergence_max_block_abs = _counter(
        scene_nonlinear_equality_convergence_gpu_row, "gpu_max_block_abs"
    )
    scene_nonlinear_equality_convergence_step_norm = _counter(
        scene_nonlinear_equality_convergence_gpu_row, "gpu_step_norm"
    )
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
    scene_sparse_residual_rows = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_sparse_residual_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse residual row count is zero"
        )
    scene_sparse_residual_bodies = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_sparse_residual_dofs = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_sparse_residual_blocks = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_sparse_residual_active_dofs = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    scene_sparse_residual_block_entries = int(
        _counter(scene_sparse_residual_cpu_row, "block_entries")
    )
    scene_sparse_residual_scene_bodies = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_sparse_residual_scene_nodes = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_sparse_residual_scene_triangles = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_sparse_residual_scene_edge_pairs = _matching_int_counter(
        scene_sparse_residual_cpu_row,
        scene_sparse_residual_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if scene_sparse_residual_scene_nodes != scene_sparse_residual_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse residual node count "
            f"{scene_sparse_residual_scene_nodes} != bodies "
            f"{scene_sparse_residual_bodies}"
        )
    if scene_sparse_residual_scene_edge_pairs != scene_sparse_residual_blocks:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse residual edge-pair count "
            f"{scene_sparse_residual_scene_edge_pairs} != blocks "
            f"{scene_sparse_residual_blocks}"
        )
    scene_sparse_residual_output_norm = _counter(
        scene_sparse_residual_gpu_row, "gpu_output_norm"
    )
    scene_sparse_residual_max_output_abs = _counter(
        scene_sparse_residual_gpu_row, "gpu_max_output_abs"
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
    scene_sparse_jacobi_rows = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_sparse_jacobi_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse Jacobi row count is zero"
        )
    scene_sparse_jacobi_bodies = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_sparse_jacobi_dofs = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_sparse_jacobi_blocks = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_sparse_jacobi_iterations = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "iterations",
        "gpu_iterations",
    )
    scene_sparse_jacobi_active_dofs = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    scene_sparse_jacobi_block_entries = int(
        _counter(scene_sparse_jacobi_cpu_row, "block_entries")
    )
    scene_sparse_jacobi_scene_bodies = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_sparse_jacobi_scene_nodes = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_sparse_jacobi_scene_triangles = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_sparse_jacobi_scene_edge_pairs = _matching_int_counter(
        scene_sparse_jacobi_cpu_row,
        scene_sparse_jacobi_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if scene_sparse_jacobi_scene_nodes != scene_sparse_jacobi_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse Jacobi node count "
            f"{scene_sparse_jacobi_scene_nodes} != bodies "
            f"{scene_sparse_jacobi_bodies}"
        )
    if scene_sparse_jacobi_scene_edge_pairs != scene_sparse_jacobi_blocks:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse Jacobi edge-pair count "
            f"{scene_sparse_jacobi_scene_edge_pairs} != blocks "
            f"{scene_sparse_jacobi_blocks}"
        )
    scene_sparse_jacobi_step_norm = _counter(
        scene_sparse_jacobi_gpu_row, "gpu_step_norm"
    )
    scene_sparse_jacobi_max_residual_abs = _counter(
        scene_sparse_jacobi_gpu_row, "gpu_max_residual_abs"
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
    scene_sparse_cg_rows = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_sparse_cg_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse CG row count is zero"
        )
    scene_sparse_cg_bodies = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    scene_sparse_cg_dofs = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    scene_sparse_cg_blocks = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    scene_sparse_cg_max_iterations = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "max_iterations",
        "gpu_max_iterations",
    )
    scene_sparse_cg_completed_iterations = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "completed_iterations",
        "gpu_completed_iterations",
    )
    scene_sparse_cg_active_dofs = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    scene_sparse_cg_block_entries = int(
        _counter(scene_sparse_cg_cpu_row, "block_entries")
    )
    scene_sparse_cg_scene_bodies = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_sparse_cg_scene_nodes = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_sparse_cg_scene_triangles = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_sparse_cg_scene_edge_pairs = _matching_int_counter(
        scene_sparse_cg_cpu_row,
        scene_sparse_cg_gpu_row,
        "scene_edge_pairs",
        "gpu_scene_edge_pairs",
    )
    if scene_sparse_cg_scene_nodes != scene_sparse_cg_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse CG node count "
            f"{scene_sparse_cg_scene_nodes} != bodies {scene_sparse_cg_bodies}"
        )
    if scene_sparse_cg_scene_edge_pairs != scene_sparse_cg_blocks:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse CG edge-pair count "
            f"{scene_sparse_cg_scene_edge_pairs} != blocks "
            f"{scene_sparse_cg_blocks}"
        )
    scene_sparse_cg_converged = int(_counter(scene_sparse_cg_gpu_row, "gpu_converged"))
    if scene_sparse_cg_converged != 1:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse CG GPU row did not converge"
        )
    scene_sparse_cg_residual_tolerance = _counter(
        scene_sparse_cg_gpu_row, "gpu_residual_tolerance"
    )
    if scene_sparse_cg_residual_norm > scene_sparse_cg_residual_tolerance:
        raise NewtonAssemblySolvePacketError(
            "scene runtime sparse CG residual norm "
            f"{scene_sparse_cg_residual_norm:.3g} exceeds "
            f"{scene_sparse_cg_residual_tolerance:.3g}"
        )
    scene_sparse_cg_initial_residual_norm = _counter(
        scene_sparse_cg_gpu_row, "gpu_initial_residual_norm"
    )
    scene_sparse_cg_step_norm = _counter(scene_sparse_cg_gpu_row, "gpu_step_norm")
    scene_sparse_cg_max_residual_abs = _counter(
        scene_sparse_cg_gpu_row, "gpu_max_residual_abs"
    )
    direct_sparse_rows = _matching_int_counter(
        direct_sparse_cpu_row, direct_sparse_gpu_row, "rows", "gpu_rows"
    )
    if direct_sparse_rows != row_count:
        raise NewtonAssemblySolvePacketError(
            f"expected {row_count} direct sparse rows, got {direct_sparse_rows}"
        )
    direct_sparse_bodies = _matching_int_counter(
        direct_sparse_cpu_row,
        direct_sparse_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    direct_sparse_dofs = _matching_int_counter(
        direct_sparse_cpu_row, direct_sparse_gpu_row, "dofs", "gpu_dofs"
    )
    if direct_sparse_dofs != direct_sparse_bodies * DIRECT_SPARSE_DOFS_PER_BODY:
        raise NewtonAssemblySolvePacketError(
            "direct sparse dofs do not match six dofs per body"
        )
    if direct_sparse_dofs > DIRECT_SPARSE_MAX_DOF_COUNT:
        raise NewtonAssemblySolvePacketError(
            f"direct sparse dofs {direct_sparse_dofs} exceed "
            f"{DIRECT_SPARSE_MAX_DOF_COUNT}"
        )
    direct_sparse_blocks = _matching_int_counter(
        direct_sparse_cpu_row,
        direct_sparse_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    direct_sparse_active_dofs = _matching_int_counter(
        direct_sparse_cpu_row,
        direct_sparse_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    if direct_sparse_active_dofs != direct_sparse_dofs:
        raise NewtonAssemblySolvePacketError(
            "direct sparse active dofs do not cover the bounded direct system"
        )
    direct_sparse_factorized = _matching_int_counter(
        direct_sparse_cpu_row,
        direct_sparse_gpu_row,
        "factorized",
        "factorized",
    )
    if direct_sparse_factorized != 1:
        raise NewtonAssemblySolvePacketError("direct sparse row did not factorize")
    direct_sparse_block_entries = int(_counter(direct_sparse_cpu_row, "block_entries"))
    if direct_sparse_block_entries != (
        direct_sparse_blocks * DIRECT_SPARSE_BLOCK_ENTRY_COUNT
    ):
        raise NewtonAssemblySolvePacketError(
            "direct sparse block entries do not match 36 entries per 6x6 block"
        )
    direct_sparse_regularization = _counter(direct_sparse_cpu_row, "regularization")
    direct_sparse_gpu_regularization = _counter(
        direct_sparse_gpu_row, "gpu_regularization"
    )
    if abs(direct_sparse_regularization - direct_sparse_gpu_regularization) > 1e-15:
        raise NewtonAssemblySolvePacketError("direct sparse regularization mismatch")
    direct_sparse_min_factor_pivot = _counter(
        direct_sparse_gpu_row, "gpu_min_factor_pivot"
    )
    if direct_sparse_min_factor_pivot <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "direct sparse GPU minimum factor pivot is not positive"
        )
    direct_sparse_cpu_min_factor_pivot = _counter(
        direct_sparse_cpu_row, "min_factor_pivot"
    )
    if (
        abs(direct_sparse_min_factor_pivot - direct_sparse_cpu_min_factor_pivot)
        > tolerance
    ):
        raise NewtonAssemblySolvePacketError(
            "direct sparse GPU minimum factor pivot does not match CPU"
        )
    if direct_sparse_residual_norm > residual_tolerance:
        raise NewtonAssemblySolvePacketError(
            f"direct sparse residual norm {direct_sparse_residual_norm:.3g} "
            f"exceeds {residual_tolerance:.3g}"
        )
    direct_sparse_step_norm = _counter(direct_sparse_gpu_row, "gpu_step_norm")
    direct_sparse_max_residual_abs = _counter(
        direct_sparse_gpu_row, "gpu_max_residual_abs"
    )
    scene_direct_sparse_rows = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "rows",
        "gpu_rows",
    )
    if scene_direct_sparse_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse row count is zero"
        )
    scene_direct_sparse_scene_bodies = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_direct_sparse_scene_nodes = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_direct_sparse_scene_triangles = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    scene_direct_sparse_selected_nodes = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "selected_scene_nodes",
        "gpu_selected_scene_nodes",
    )
    if scene_direct_sparse_selected_nodes <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse selected node count is zero"
        )
    if scene_direct_sparse_selected_nodes > scene_direct_sparse_scene_nodes:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse selected nodes exceed scene nodes"
        )
    scene_direct_sparse_selected_edge_pairs = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "selected_scene_edge_pairs",
        "gpu_selected_scene_edge_pairs",
    )
    if scene_direct_sparse_selected_edge_pairs <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse selected edge-pair count is zero"
        )
    scene_direct_sparse_bodies = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "bodies",
        "gpu_bodies",
    )
    if scene_direct_sparse_selected_nodes != scene_direct_sparse_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse selected node count "
            f"{scene_direct_sparse_selected_nodes} != bodies "
            f"{scene_direct_sparse_bodies}"
        )
    scene_direct_sparse_dofs = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "dofs",
        "gpu_dofs",
    )
    if scene_direct_sparse_dofs != (
        scene_direct_sparse_selected_nodes * DIRECT_SPARSE_DOFS_PER_BODY
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse dofs do not match six dofs per "
            "selected node"
        )
    if scene_direct_sparse_dofs > DIRECT_SPARSE_MAX_DOF_COUNT:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime direct sparse dofs {scene_direct_sparse_dofs} "
            f"exceed {DIRECT_SPARSE_MAX_DOF_COUNT}"
        )
    scene_direct_sparse_blocks = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "blocks",
        "gpu_blocks",
    )
    if scene_direct_sparse_selected_edge_pairs != scene_direct_sparse_blocks:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse selected edge-pair count "
            f"{scene_direct_sparse_selected_edge_pairs} != blocks "
            f"{scene_direct_sparse_blocks}"
        )
    scene_direct_sparse_active_dofs = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "active_dofs",
        "gpu_active_dofs",
    )
    if scene_direct_sparse_active_dofs != scene_direct_sparse_dofs:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse active dofs do not cover the "
            "selected scene system"
        )
    scene_direct_sparse_factorized = _matching_int_counter(
        scene_direct_sparse_cpu_row,
        scene_direct_sparse_gpu_row,
        "factorized",
        "factorized",
    )
    if scene_direct_sparse_factorized != 1:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse row did not factorize"
        )
    scene_direct_sparse_block_entries = int(
        _counter(scene_direct_sparse_cpu_row, "block_entries")
    )
    if scene_direct_sparse_block_entries != (
        scene_direct_sparse_blocks * DIRECT_SPARSE_BLOCK_ENTRY_COUNT
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse block entries do not match 36 entries "
            "per 6x6 block"
        )
    scene_direct_sparse_regularization = _counter(
        scene_direct_sparse_cpu_row, "regularization"
    )
    scene_direct_sparse_gpu_regularization = _counter(
        scene_direct_sparse_gpu_row, "gpu_regularization"
    )
    if (
        abs(scene_direct_sparse_regularization - scene_direct_sparse_gpu_regularization)
        > 1e-15
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse regularization mismatch"
        )
    scene_direct_sparse_min_factor_pivot = _counter(
        scene_direct_sparse_gpu_row, "gpu_min_factor_pivot"
    )
    if scene_direct_sparse_min_factor_pivot <= 0.0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse GPU minimum factor pivot is not positive"
        )
    scene_direct_sparse_cpu_min_factor_pivot = _counter(
        scene_direct_sparse_cpu_row, "min_factor_pivot"
    )
    if (
        abs(
            scene_direct_sparse_min_factor_pivot
            - scene_direct_sparse_cpu_min_factor_pivot
        )
        > tolerance
    ):
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse GPU minimum factor pivot does not match CPU"
        )
    if scene_direct_sparse_residual_norm > residual_tolerance:
        raise NewtonAssemblySolvePacketError(
            "scene runtime direct sparse residual norm "
            f"{scene_direct_sparse_residual_norm:.3g} exceeds "
            f"{residual_tolerance:.3g}"
        )
    scene_direct_sparse_step_norm = _counter(
        scene_direct_sparse_gpu_row, "gpu_step_norm"
    )
    scene_direct_sparse_max_residual_abs = _counter(
        scene_direct_sparse_gpu_row, "gpu_max_residual_abs"
    )
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
    scene_equality_rows = _matching_int_counter(
        scene_equality_cpu_row, scene_equality_gpu_row, "rows", "gpu_rows"
    )
    if scene_equality_rows <= 0:
        raise NewtonAssemblySolvePacketError(
            "scene runtime equality-reduced row count is zero"
        )
    scene_equality_bodies = _matching_int_counter(
        scene_equality_cpu_row, scene_equality_gpu_row, "bodies", "gpu_bodies"
    )
    scene_equality_dofs = _matching_int_counter(
        scene_equality_cpu_row, scene_equality_gpu_row, "dofs", "gpu_dofs"
    )
    scene_equality_reduction_entries = _matching_int_counter(
        scene_equality_cpu_row,
        scene_equality_gpu_row,
        "reduction_entries",
        "gpu_reduction_entries",
    )
    scene_equality_reduced_dofs = _matching_int_counter(
        scene_equality_cpu_row,
        scene_equality_gpu_row,
        "reduced_dofs",
        "gpu_reduced_dofs",
    )
    scene_equality_active_reduced_dofs = _matching_int_counter(
        scene_equality_cpu_row,
        scene_equality_gpu_row,
        "active_reduced_dofs",
        "gpu_active_reduced_dofs",
    )
    scene_equality_scene_bodies = _matching_int_counter(
        scene_equality_cpu_row,
        scene_equality_gpu_row,
        "scene_bodies",
        "gpu_scene_bodies",
    )
    scene_equality_scene_nodes = _matching_int_counter(
        scene_equality_cpu_row,
        scene_equality_gpu_row,
        "scene_nodes",
        "gpu_scene_nodes",
    )
    scene_equality_scene_triangles = _matching_int_counter(
        scene_equality_cpu_row,
        scene_equality_gpu_row,
        "scene_triangles",
        "gpu_scene_triangles",
    )
    if scene_equality_scene_nodes != scene_equality_bodies:
        raise NewtonAssemblySolvePacketError(
            "scene runtime equality-reduced node count "
            f"{scene_equality_scene_nodes} != bodies {scene_equality_bodies}"
        )
    if scene_equality_dofs != scene_equality_scene_nodes * 6:
        raise NewtonAssemblySolvePacketError(
            "scene runtime equality-reduced dofs do not match six dofs per node"
        )
    if scene_equality_reduction_entries != scene_equality_dofs:
        raise NewtonAssemblySolvePacketError(
            "scene runtime equality-reduced reduction entries do not match full dofs"
        )
    if scene_equality_reduced_dofs != scene_equality_scene_nodes * 3:
        raise NewtonAssemblySolvePacketError(
            "scene runtime equality-reduced dofs do not match three per node"
        )
    scene_equality_step_norm = _counter(scene_equality_gpu_row, "gpu_step_norm")
    off_diagonal_speedup = off_diagonal_cpu_ns / off_diagonal_gpu_ns
    scene_off_diagonal_speedup = scene_off_diagonal_cpu_ns / scene_off_diagonal_gpu_ns
    scene_sparse_graph_speedup = scene_sparse_graph_cpu_ns / scene_sparse_graph_gpu_ns
    scene_sparse_graph_unique_speedup = (
        scene_sparse_graph_unique_cpu_ns / scene_sparse_graph_unique_gpu_ns
    )
    scene_nonlinear_equality_speedup = (
        scene_nonlinear_equality_cpu_ns / scene_nonlinear_equality_gpu_ns
    )
    scene_nonlinear_equality_solve_speedup = (
        scene_nonlinear_equality_solve_cpu_ns / scene_nonlinear_equality_solve_gpu_ns
    )
    scene_nonlinear_equality_convergence_speedup = (
        scene_nonlinear_equality_convergence_cpu_ns
        / scene_nonlinear_equality_convergence_gpu_ns
    )
    diagonal_speedup = cpu_ns / gpu_ns
    scene_assembly_speedup = scene_assembly_cpu_ns / scene_assembly_gpu_ns
    sparse_residual_speedup = sparse_residual_cpu_ns / sparse_residual_gpu_ns
    scene_sparse_residual_speedup = (
        scene_sparse_residual_cpu_ns / scene_sparse_residual_gpu_ns
    )
    sparse_jacobi_speedup = sparse_jacobi_cpu_ns / sparse_jacobi_gpu_ns
    scene_sparse_jacobi_speedup = (
        scene_sparse_jacobi_cpu_ns / scene_sparse_jacobi_gpu_ns
    )
    sparse_cg_speedup = sparse_cg_cpu_ns / sparse_cg_gpu_ns
    scene_sparse_cg_speedup = scene_sparse_cg_cpu_ns / scene_sparse_cg_gpu_ns
    direct_sparse_speedup = direct_sparse_cpu_ns / direct_sparse_gpu_ns
    scene_direct_sparse_speedup = (
        scene_direct_sparse_cpu_ns / scene_direct_sparse_gpu_ns
    )
    equality_speedup = equality_cpu_ns / equality_gpu_ns
    scene_equality_speedup = scene_equality_cpu_ns / scene_equality_gpu_ns
    speedup = min(
        diagonal_speedup,
        scene_assembly_speedup,
        off_diagonal_speedup,
        scene_off_diagonal_speedup,
        scene_sparse_graph_speedup,
        scene_sparse_graph_unique_speedup,
        scene_nonlinear_equality_speedup,
        scene_nonlinear_equality_solve_speedup,
        scene_nonlinear_equality_convergence_speedup,
        sparse_residual_speedup,
        scene_sparse_residual_speedup,
        sparse_jacobi_speedup,
        scene_sparse_jacobi_speedup,
        sparse_cg_speedup,
        scene_sparse_cg_speedup,
        direct_sparse_speedup,
        scene_direct_sparse_speedup,
        equality_speedup,
        scene_equality_speedup,
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
    scene_off_diagonal_timing_ns = {
        "setup": _counter(scene_off_diagonal_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_off_diagonal_gpu_row, "host_to_device_ns"),
        "kernel": _counter(scene_off_diagonal_gpu_row, "assembly_kernel_ns"),
        "solve": _counter(scene_off_diagonal_gpu_row, "solve_kernel_ns"),
        "device_to_host": _counter(scene_off_diagonal_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = REQUIRED_TIMING_KEYS - scene_off_diagonal_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime off-diagonal packet timing is missing {sorted(missing)}"
        )
    scene_sparse_graph_timing_ns = {
        "setup": _counter(scene_sparse_graph_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_sparse_graph_gpu_row, "host_to_device_ns"),
        "incidence": _counter(scene_sparse_graph_gpu_row, "incidence_kernel_ns"),
        "diagonal": _counter(scene_sparse_graph_gpu_row, "diagonal_kernel_ns"),
        "sparse_blocks": _counter(scene_sparse_graph_gpu_row, "sparse_block_kernel_ns"),
        "device_to_host": _counter(scene_sparse_graph_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = SCENE_SPARSE_GRAPH_TIMING_KEYS - scene_sparse_graph_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime sparse graph packet timing is missing {sorted(missing)}"
        )
    scene_sparse_graph_unique_timing_ns = {
        "setup": _counter(scene_sparse_graph_unique_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(
            scene_sparse_graph_unique_gpu_row, "host_to_device_ns"
        ),
        "incidence": _counter(scene_sparse_graph_unique_gpu_row, "incidence_kernel_ns"),
        "diagonal": _counter(scene_sparse_graph_unique_gpu_row, "diagonal_kernel_ns"),
        "edge_mark": _counter(
            scene_sparse_graph_unique_gpu_row, "unique_edge_mark_kernel_ns"
        ),
        "sparse_blocks": _counter(
            scene_sparse_graph_unique_gpu_row, "sparse_block_kernel_ns"
        ),
        "device_to_host": _counter(
            scene_sparse_graph_unique_gpu_row, "device_to_host_ns"
        ),
        "readback": 0.0,
    }
    missing = (
        SCENE_SPARSE_GRAPH_UNIQUE_TIMING_KEYS
        - scene_sparse_graph_unique_timing_ns.keys()
    )
    if missing:
        raise NewtonAssemblySolvePacketError(
            "scene runtime unique sparse graph packet timing is missing "
            f"{sorted(missing)}"
        )
    scene_nonlinear_equality_timing_ns = {
        "setup": _counter(scene_nonlinear_equality_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(
            scene_nonlinear_equality_gpu_row, "host_to_device_ns"
        ),
        "constraints": _counter(
            scene_nonlinear_equality_gpu_row, "constraint_kernel_ns"
        ),
        "device_to_host": _counter(
            scene_nonlinear_equality_gpu_row, "device_to_host_ns"
        ),
        "readback": 0.0,
    }
    missing = (
        SCENE_NONLINEAR_EQUALITY_TIMING_KEYS - scene_nonlinear_equality_timing_ns.keys()
    )
    if missing:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality packet timing is missing "
            f"{sorted(missing)}"
        )
    scene_nonlinear_equality_solve_timing_ns = {
        "setup": _counter(scene_nonlinear_equality_solve_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(
            scene_nonlinear_equality_solve_gpu_row, "host_to_device_ns"
        ),
        "solve": _counter(scene_nonlinear_equality_solve_gpu_row, "solve_kernel_ns"),
        "post_residual": _counter(
            scene_nonlinear_equality_solve_gpu_row,
            "post_residual_kernel_ns",
        ),
        "device_to_host": _counter(
            scene_nonlinear_equality_solve_gpu_row, "device_to_host_ns"
        ),
        "readback": 0.0,
    }
    missing = (
        SCENE_NONLINEAR_EQUALITY_SOLVE_TIMING_KEYS
        - scene_nonlinear_equality_solve_timing_ns.keys()
    )
    if missing:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality solve packet timing is missing "
            f"{sorted(missing)}"
        )
    scene_nonlinear_equality_convergence_timing_ns = {
        "setup": _counter(
            scene_nonlinear_equality_convergence_gpu_row, "host_setup_ns"
        ),
        "host_to_device": _counter(
            scene_nonlinear_equality_convergence_gpu_row, "host_to_device_ns"
        ),
        "iterations": _counter(
            scene_nonlinear_equality_convergence_gpu_row, "iteration_kernel_ns"
        ),
        "residuals": _counter(
            scene_nonlinear_equality_convergence_gpu_row, "residual_kernel_ns"
        ),
        "convergence_readback": _counter(
            scene_nonlinear_equality_convergence_gpu_row,
            "convergence_readback_ns",
        ),
        "final_assembly": _counter(
            scene_nonlinear_equality_convergence_gpu_row,
            "final_assembly_kernel_ns",
        ),
        "device_to_host": _counter(
            scene_nonlinear_equality_convergence_gpu_row, "device_to_host_ns"
        ),
        "readback": 0.0,
    }
    missing = (
        SCENE_NONLINEAR_EQUALITY_CONVERGENCE_TIMING_KEYS
        - scene_nonlinear_equality_convergence_timing_ns.keys()
    )
    if missing:
        raise NewtonAssemblySolvePacketError(
            "scene runtime nonlinear equality convergence packet timing is "
            f"missing {sorted(missing)}"
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
    scene_sparse_residual_timing_ns = {
        "setup": _counter(scene_sparse_residual_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_sparse_residual_gpu_row, "host_to_device_ns"),
        "kernel": _counter(scene_sparse_residual_gpu_row, "assembly_kernel_ns"),
        "gradient_seed": _counter(scene_sparse_residual_gpu_row, "gradient_seed_ns"),
        "diagonal": _counter(scene_sparse_residual_gpu_row, "diagonal_kernel_ns"),
        "off_diagonal": _counter(
            scene_sparse_residual_gpu_row, "off_diagonal_kernel_ns"
        ),
        "device_to_host": _counter(scene_sparse_residual_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = SPARSE_RESIDUAL_TIMING_KEYS - scene_sparse_residual_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime sparse residual packet timing is missing {sorted(missing)}"
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
    scene_sparse_jacobi_timing_ns = {
        "setup": _counter(scene_sparse_jacobi_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_sparse_jacobi_gpu_row, "host_to_device_ns"),
        "kernel": _counter(scene_sparse_jacobi_gpu_row, "assembly_kernel_ns"),
        "iterations": _counter(scene_sparse_jacobi_gpu_row, "iteration_kernel_ns"),
        "final_residual": _counter(
            scene_sparse_jacobi_gpu_row, "final_residual_kernel_ns"
        ),
        "device_to_host": _counter(scene_sparse_jacobi_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = SPARSE_JACOBI_TIMING_KEYS - scene_sparse_jacobi_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime sparse Jacobi packet timing is missing {sorted(missing)}"
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
    scene_sparse_cg_timing_ns = {
        "setup": _counter(scene_sparse_cg_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_sparse_cg_gpu_row, "host_to_device_ns"),
        "kernel": _counter(scene_sparse_cg_gpu_row, "assembly_kernel_ns"),
        "iterations": _counter(scene_sparse_cg_gpu_row, "iteration_kernel_ns"),
        "final_residual": _counter(scene_sparse_cg_gpu_row, "final_residual_kernel_ns"),
        "device_to_host": _counter(scene_sparse_cg_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = SPARSE_CG_TIMING_KEYS - scene_sparse_cg_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime sparse CG packet timing is missing {sorted(missing)}"
        )
    direct_sparse_timing_ns = {
        "setup": _counter(direct_sparse_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(direct_sparse_gpu_row, "host_to_device_ns"),
        "kernel": _counter(direct_sparse_gpu_row, "assembly_kernel_ns"),
        "dense_matrix": _counter(direct_sparse_gpu_row, "dense_matrix_kernel_ns"),
        "factor_solve": _counter(direct_sparse_gpu_row, "factor_solve_kernel_ns"),
        "final_residual": _counter(direct_sparse_gpu_row, "final_residual_kernel_ns"),
        "device_to_host": _counter(direct_sparse_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = DIRECT_SPARSE_TIMING_KEYS - direct_sparse_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"direct sparse packet timing is missing {sorted(missing)}"
        )
    scene_direct_sparse_timing_ns = {
        "setup": _counter(scene_direct_sparse_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_direct_sparse_gpu_row, "host_to_device_ns"),
        "kernel": _counter(scene_direct_sparse_gpu_row, "assembly_kernel_ns"),
        "dense_matrix": _counter(scene_direct_sparse_gpu_row, "dense_matrix_kernel_ns"),
        "factor_solve": _counter(scene_direct_sparse_gpu_row, "factor_solve_kernel_ns"),
        "final_residual": _counter(
            scene_direct_sparse_gpu_row, "final_residual_kernel_ns"
        ),
        "device_to_host": _counter(scene_direct_sparse_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = DIRECT_SPARSE_TIMING_KEYS - scene_direct_sparse_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene runtime direct sparse packet timing is missing {sorted(missing)}"
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
    scene_equality_timing_ns = {
        "setup": _counter(scene_equality_gpu_row, "host_setup_ns"),
        "host_to_device": _counter(scene_equality_gpu_row, "host_to_device_ns"),
        "kernel": _counter(scene_equality_gpu_row, "assembly_kernel_ns"),
        "reduction": _counter(scene_equality_gpu_row, "reduction_kernel_ns"),
        "solve": _counter(scene_equality_gpu_row, "solve_kernel_ns"),
        "expansion": _counter(scene_equality_gpu_row, "expansion_kernel_ns"),
        "device_to_host": _counter(scene_equality_gpu_row, "device_to_host_ns"),
        "readback": 0.0,
    }
    missing = EQUALITY_TIMING_KEYS - scene_equality_timing_ns.keys()
    if missing:
        raise NewtonAssemblySolvePacketError(
            f"scene equality-reduced packet timing is missing {sorted(missing)}"
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
            "scene_runtime_off_diagonal_sparse_block_assembly": {
                "row_count": scene_off_diagonal_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_off_diagonal_scene_bodies,
                "scene_node_count": scene_off_diagonal_scene_nodes,
                "scene_triangle_count": scene_off_diagonal_scene_triangles,
                "scene_edge_pair_count": scene_off_diagonal_scene_edge_pairs,
                "pair_count": scene_off_diagonal_pairs,
                "block_entry_count": scene_off_diagonal_block_entries,
                "active_block_count": scene_off_diagonal_active_blocks,
                "max_block_abs": _counter(
                    scene_off_diagonal_gpu_row, "gpu_max_block_abs"
                ),
                "max_result_abs_error": scene_off_diagonal_max_error,
                "speedup": scene_off_diagonal_speedup,
                "meets_speedup_gate": scene_off_diagonal_speedup >= speedup_gate,
                "timing_ns": scene_off_diagonal_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_off_diagonal_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_off_diagonal_gpu_row),
            },
            "scene_runtime_sparse_graph_assembly": {
                "row_count": scene_sparse_graph_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_sparse_graph_scene_bodies,
                "scene_node_count": scene_sparse_graph_scene_nodes,
                "scene_triangle_count": scene_sparse_graph_scene_triangles,
                "scene_edge_pair_count": scene_sparse_graph_scene_edge_pairs,
                "body_count": scene_sparse_graph_bodies,
                "dof_count": scene_sparse_graph_dofs,
                "block_count": scene_sparse_graph_blocks,
                "block_entry_count": scene_sparse_graph_block_entries,
                "max_diagonal": scene_sparse_graph_max_diagonal,
                "max_gradient_abs": scene_sparse_graph_max_gradient_abs,
                "max_block_abs": scene_sparse_graph_max_block_abs,
                "max_result_abs_error": scene_sparse_graph_max_error,
                "speedup": scene_sparse_graph_speedup,
                "meets_speedup_gate": scene_sparse_graph_speedup >= speedup_gate,
                "timing_ns": scene_sparse_graph_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_sparse_graph_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_sparse_graph_gpu_row),
            },
            "scene_runtime_sparse_graph_unique_assembly": {
                "row_count": scene_sparse_graph_unique_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_sparse_graph_unique_scene_bodies,
                "scene_node_count": scene_sparse_graph_unique_scene_nodes,
                "scene_triangle_count": scene_sparse_graph_unique_scene_triangles,
                "edge_slot_count": scene_sparse_graph_unique_edge_slots,
                "unique_edge_count": scene_sparse_graph_unique_unique_edges,
                "duplicate_edge_slot_count": (
                    scene_sparse_graph_unique_duplicate_edge_slots
                ),
                "body_count": scene_sparse_graph_unique_bodies,
                "dof_count": scene_sparse_graph_unique_dofs,
                "block_count": scene_sparse_graph_unique_blocks,
                "block_entry_count": scene_sparse_graph_unique_block_entries,
                "max_diagonal": scene_sparse_graph_unique_max_diagonal,
                "max_gradient_abs": scene_sparse_graph_unique_max_gradient_abs,
                "max_block_abs": scene_sparse_graph_unique_max_block_abs,
                "max_result_abs_error": scene_sparse_graph_unique_max_error,
                "speedup": scene_sparse_graph_unique_speedup,
                "meets_speedup_gate": (
                    scene_sparse_graph_unique_speedup >= speedup_gate
                ),
                "timing_ns": scene_sparse_graph_unique_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_sparse_graph_unique_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_sparse_graph_unique_gpu_row
                ),
            },
            "scene_runtime_nonlinear_equality_assembly": {
                "row_count": scene_nonlinear_equality_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_nonlinear_equality_scene_bodies,
                "scene_node_count": scene_nonlinear_equality_scene_nodes,
                "scene_triangle_count": scene_nonlinear_equality_scene_triangles,
                "scene_edge_pair_count": scene_nonlinear_equality_scene_edge_pairs,
                "body_count": scene_nonlinear_equality_bodies,
                "dof_count": scene_nonlinear_equality_dofs,
                "constraint_count": scene_nonlinear_equality_constraints,
                "block_count": scene_nonlinear_equality_blocks,
                "block_entry_count": scene_nonlinear_equality_block_entries,
                "max_constraint_residual_abs": (
                    scene_nonlinear_equality_max_constraint_residual_abs
                ),
                "max_diagonal": scene_nonlinear_equality_max_diagonal,
                "max_gradient_abs": scene_nonlinear_equality_max_gradient_abs,
                "max_block_abs": scene_nonlinear_equality_max_block_abs,
                "max_result_abs_error": scene_nonlinear_equality_max_error,
                "speedup": scene_nonlinear_equality_speedup,
                "meets_speedup_gate": (
                    scene_nonlinear_equality_speedup >= speedup_gate
                ),
                "timing_ns": scene_nonlinear_equality_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_nonlinear_equality_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_nonlinear_equality_gpu_row),
            },
            "scene_runtime_nonlinear_equality_solve": {
                "row_count": scene_nonlinear_equality_solve_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_nonlinear_equality_solve_scene_bodies,
                "scene_node_count": scene_nonlinear_equality_solve_scene_nodes,
                "scene_triangle_count": (
                    scene_nonlinear_equality_solve_scene_triangles
                ),
                "scene_edge_pair_count": (
                    scene_nonlinear_equality_solve_scene_edge_pairs
                ),
                "body_count": scene_nonlinear_equality_solve_bodies,
                "dof_count": scene_nonlinear_equality_solve_dofs,
                "constraint_count": scene_nonlinear_equality_solve_constraints,
                "block_count": scene_nonlinear_equality_solve_blocks,
                "block_entry_count": scene_nonlinear_equality_solve_block_entries,
                "active_dof_count": scene_nonlinear_equality_solve_active_dofs,
                "regularization": scene_nonlinear_equality_solve_regularization,
                "max_constraint_residual_abs": (
                    scene_nonlinear_equality_solve_max_constraint_residual_abs
                ),
                "max_post_solve_linearized_residual_abs": (
                    scene_nonlinear_equality_solve_max_post_residual_abs
                ),
                "max_diagonal": scene_nonlinear_equality_solve_max_diagonal,
                "max_gradient_abs": (scene_nonlinear_equality_solve_max_gradient_abs),
                "max_block_abs": scene_nonlinear_equality_solve_max_block_abs,
                "step_norm": scene_nonlinear_equality_solve_step_norm,
                "max_result_abs_error": scene_nonlinear_equality_solve_max_error,
                "speedup": scene_nonlinear_equality_solve_speedup,
                "meets_speedup_gate": (
                    scene_nonlinear_equality_solve_speedup >= speedup_gate
                ),
                "timing_ns": scene_nonlinear_equality_solve_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_nonlinear_equality_solve_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_nonlinear_equality_solve_gpu_row
                ),
            },
            "scene_runtime_nonlinear_equality_convergence": {
                "row_count": scene_nonlinear_equality_convergence_rows,
                "nominal_row_count": row_count,
                "scene_body_count": (scene_nonlinear_equality_convergence_scene_bodies),
                "scene_node_count": (scene_nonlinear_equality_convergence_scene_nodes),
                "scene_triangle_count": (
                    scene_nonlinear_equality_convergence_scene_triangles
                ),
                "scene_edge_pair_count": (
                    scene_nonlinear_equality_convergence_scene_edge_pairs
                ),
                "body_count": scene_nonlinear_equality_convergence_bodies,
                "dof_count": scene_nonlinear_equality_convergence_dofs,
                "constraint_count": (scene_nonlinear_equality_convergence_constraints),
                "block_count": scene_nonlinear_equality_convergence_blocks,
                "block_entry_count": (
                    scene_nonlinear_equality_convergence_block_entries
                ),
                "active_dof_count": (scene_nonlinear_equality_convergence_active_dofs),
                "regularization": (scene_nonlinear_equality_convergence_regularization),
                "residual_tolerance": (
                    scene_nonlinear_equality_convergence_residual_tolerance
                ),
                "max_iteration_count": (
                    scene_nonlinear_equality_convergence_max_iterations
                ),
                "completed_iteration_count": (
                    scene_nonlinear_equality_convergence_completed_iterations
                ),
                "converged": bool(scene_nonlinear_equality_convergence_converged),
                "initial_max_constraint_residual_abs": (
                    scene_nonlinear_equality_convergence_initial_max_residual_abs
                ),
                "final_max_constraint_residual_abs": (
                    scene_nonlinear_equality_convergence_final_max_residual_abs
                ),
                "max_diagonal": scene_nonlinear_equality_convergence_max_diagonal,
                "max_gradient_abs": (
                    scene_nonlinear_equality_convergence_max_gradient_abs
                ),
                "max_block_abs": scene_nonlinear_equality_convergence_max_block_abs,
                "step_norm": scene_nonlinear_equality_convergence_step_norm,
                "max_result_abs_error": (
                    scene_nonlinear_equality_convergence_max_error
                ),
                "speedup": scene_nonlinear_equality_convergence_speedup,
                "meets_speedup_gate": (
                    scene_nonlinear_equality_convergence_speedup >= speedup_gate
                ),
                "timing_ns": scene_nonlinear_equality_convergence_timing_ns,
                "cpu_benchmark_row": _packet_row_name(
                    scene_nonlinear_equality_convergence_cpu_row
                ),
                "gpu_benchmark_row": _packet_row_name(
                    scene_nonlinear_equality_convergence_gpu_row
                ),
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
            "scene_runtime_sparse_block_residual": {
                "row_count": scene_sparse_residual_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_sparse_residual_scene_bodies,
                "scene_node_count": scene_sparse_residual_scene_nodes,
                "scene_triangle_count": scene_sparse_residual_scene_triangles,
                "scene_edge_pair_count": scene_sparse_residual_scene_edge_pairs,
                "body_count": scene_sparse_residual_bodies,
                "dof_count": scene_sparse_residual_dofs,
                "block_count": scene_sparse_residual_blocks,
                "block_entry_count": scene_sparse_residual_block_entries,
                "active_dof_count": scene_sparse_residual_active_dofs,
                "max_result_abs_error": scene_sparse_residual_max_error,
                "output_norm": scene_sparse_residual_output_norm,
                "max_output_abs": scene_sparse_residual_max_output_abs,
                "speedup": scene_sparse_residual_speedup,
                "meets_speedup_gate": scene_sparse_residual_speedup >= speedup_gate,
                "timing_ns": scene_sparse_residual_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_sparse_residual_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_sparse_residual_gpu_row),
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
            "scene_runtime_sparse_jacobi_solve": {
                "row_count": scene_sparse_jacobi_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_sparse_jacobi_scene_bodies,
                "scene_node_count": scene_sparse_jacobi_scene_nodes,
                "scene_triangle_count": scene_sparse_jacobi_scene_triangles,
                "scene_edge_pair_count": scene_sparse_jacobi_scene_edge_pairs,
                "body_count": scene_sparse_jacobi_bodies,
                "dof_count": scene_sparse_jacobi_dofs,
                "block_count": scene_sparse_jacobi_blocks,
                "block_entry_count": scene_sparse_jacobi_block_entries,
                "iteration_count": scene_sparse_jacobi_iterations,
                "active_dof_count": scene_sparse_jacobi_active_dofs,
                "max_result_abs_error": scene_sparse_jacobi_max_error,
                "residual_norm": scene_sparse_jacobi_residual_norm,
                "max_residual_abs": scene_sparse_jacobi_max_residual_abs,
                "step_norm": scene_sparse_jacobi_step_norm,
                "speedup": scene_sparse_jacobi_speedup,
                "meets_speedup_gate": scene_sparse_jacobi_speedup >= speedup_gate,
                "timing_ns": scene_sparse_jacobi_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_sparse_jacobi_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_sparse_jacobi_gpu_row),
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
            "scene_runtime_sparse_cg_solve": {
                "row_count": scene_sparse_cg_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_sparse_cg_scene_bodies,
                "scene_node_count": scene_sparse_cg_scene_nodes,
                "scene_triangle_count": scene_sparse_cg_scene_triangles,
                "scene_edge_pair_count": scene_sparse_cg_scene_edge_pairs,
                "body_count": scene_sparse_cg_bodies,
                "dof_count": scene_sparse_cg_dofs,
                "block_count": scene_sparse_cg_blocks,
                "block_entry_count": scene_sparse_cg_block_entries,
                "max_iteration_count": scene_sparse_cg_max_iterations,
                "completed_iteration_count": scene_sparse_cg_completed_iterations,
                "active_dof_count": scene_sparse_cg_active_dofs,
                "converged": bool(scene_sparse_cg_converged),
                "residual_tolerance": scene_sparse_cg_residual_tolerance,
                "initial_residual_norm": scene_sparse_cg_initial_residual_norm,
                "max_result_abs_error": scene_sparse_cg_max_error,
                "residual_norm": scene_sparse_cg_residual_norm,
                "max_residual_abs": scene_sparse_cg_max_residual_abs,
                "step_norm": scene_sparse_cg_step_norm,
                "speedup": scene_sparse_cg_speedup,
                "meets_speedup_gate": scene_sparse_cg_speedup >= speedup_gate,
                "timing_ns": scene_sparse_cg_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_sparse_cg_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_sparse_cg_gpu_row),
            },
            "direct_sparse_factor_solve": {
                "row_count": direct_sparse_rows,
                "body_count": direct_sparse_bodies,
                "dof_count": direct_sparse_dofs,
                "max_supported_dof_count": DIRECT_SPARSE_MAX_DOF_COUNT,
                "block_count": direct_sparse_blocks,
                "block_entry_count": direct_sparse_block_entries,
                "active_dof_count": direct_sparse_active_dofs,
                "regularization": direct_sparse_regularization,
                "factorized": bool(direct_sparse_factorized),
                "minimum_factor_pivot": direct_sparse_min_factor_pivot,
                "max_result_abs_error": direct_sparse_max_error,
                "residual_norm": direct_sparse_residual_norm,
                "max_residual_abs": direct_sparse_max_residual_abs,
                "step_norm": direct_sparse_step_norm,
                "speedup": direct_sparse_speedup,
                "meets_speedup_gate": direct_sparse_speedup >= speedup_gate,
                "timing_ns": direct_sparse_timing_ns,
                "cpu_benchmark_row": _packet_row_name(direct_sparse_cpu_row),
                "gpu_benchmark_row": _packet_row_name(direct_sparse_gpu_row),
            },
            "scene_runtime_direct_sparse_factor_solve": {
                "row_count": scene_direct_sparse_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_direct_sparse_scene_bodies,
                "scene_node_count": scene_direct_sparse_scene_nodes,
                "scene_triangle_count": scene_direct_sparse_scene_triangles,
                "selected_scene_node_count": scene_direct_sparse_selected_nodes,
                "selected_scene_edge_pair_count": (
                    scene_direct_sparse_selected_edge_pairs
                ),
                "body_count": scene_direct_sparse_bodies,
                "dof_count": scene_direct_sparse_dofs,
                "max_supported_dof_count": DIRECT_SPARSE_MAX_DOF_COUNT,
                "block_count": scene_direct_sparse_blocks,
                "block_entry_count": scene_direct_sparse_block_entries,
                "active_dof_count": scene_direct_sparse_active_dofs,
                "regularization": scene_direct_sparse_regularization,
                "factorized": bool(scene_direct_sparse_factorized),
                "minimum_factor_pivot": scene_direct_sparse_min_factor_pivot,
                "max_result_abs_error": scene_direct_sparse_max_error,
                "residual_norm": scene_direct_sparse_residual_norm,
                "max_residual_abs": scene_direct_sparse_max_residual_abs,
                "step_norm": scene_direct_sparse_step_norm,
                "speedup": scene_direct_sparse_speedup,
                "meets_speedup_gate": scene_direct_sparse_speedup >= speedup_gate,
                "timing_ns": scene_direct_sparse_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_direct_sparse_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_direct_sparse_gpu_row),
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
            "scene_runtime_equality_reduced_diagonal_solve": {
                "row_count": scene_equality_rows,
                "nominal_row_count": row_count,
                "scene_body_count": scene_equality_scene_bodies,
                "scene_node_count": scene_equality_scene_nodes,
                "scene_triangle_count": scene_equality_scene_triangles,
                "body_count": scene_equality_bodies,
                "full_dof_count": scene_equality_dofs,
                "reduction_entry_count": scene_equality_reduction_entries,
                "reduced_dof_count": scene_equality_reduced_dofs,
                "active_reduced_dof_count": scene_equality_active_reduced_dofs,
                "max_result_abs_error": scene_equality_max_error,
                "residual_norm": scene_equality_residual_norm,
                "step_norm": scene_equality_step_norm,
                "speedup": scene_equality_speedup,
                "meets_speedup_gate": scene_equality_speedup >= speedup_gate,
                "timing_ns": scene_equality_timing_ns,
                "cpu_benchmark_row": _packet_row_name(scene_equality_cpu_row),
                "gpu_benchmark_row": _packet_row_name(scene_equality_gpu_row),
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
