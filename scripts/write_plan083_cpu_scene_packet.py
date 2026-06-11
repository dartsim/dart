#!/usr/bin/env python3
"""Run and validate PLAN-083's reduced CPU scene corpus packet."""

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
    ".benchmark_results/plan083/cpu_scene_corpus/hanging_bridge_benchmark.json"
)
DEFAULT_PACKET_OUTPUT = Path(
    ".benchmark_results/plan083/cpu_scene_corpus/hanging_bridge.json"
)

DEFAULT_SCENE = "hanging_bridge"
SCENE_ROWS = {
    "hanging_bridge": "BM_Plan083CpuScene_hanging_bridge_reduced_world_step",
    "nunchaku_single": "BM_Plan083CpuScene_nunchaku_single_reduced_world_step",
    "precession": "BM_Plan083CpuScene_precession_reduced_world_step",
    "ragdoll_reduced": "BM_Plan083CpuScene_ragdoll_reduced_world_step",
    "terrain_vehicle": "BM_Plan083CpuScene_terrain_vehicle_reduced_world_step",
    "windmill": "BM_Plan083CpuScene_windmill_reduced_world_step",
}
SCENE_BENCHMARK_OUTPUTS = {
    "hanging_bridge": DEFAULT_BENCHMARK_OUTPUT,
    "nunchaku_single": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/nunchaku_single_benchmark.json"
    ),
    "precession": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/precession_benchmark.json"
    ),
    "ragdoll_reduced": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/ragdolls_benchmark.json"
    ),
    "terrain_vehicle": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/terrain_vehicle_benchmark.json"
    ),
    "windmill": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/windmill_benchmark.json"
    ),
}
SCENE_PACKET_OUTPUTS = {
    "hanging_bridge": DEFAULT_PACKET_OUTPUT,
    "nunchaku_single": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/nunchaku_single.json"
    ),
    "precession": Path(".benchmark_results/plan083/cpu_scene_corpus/precession.json"),
    "ragdoll_reduced": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/ragdolls.json"
    ),
    "terrain_vehicle": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/terrain_vehicle.json"
    ),
    "windmill": Path(".benchmark_results/plan083/cpu_scene_corpus/windmill.json"),
}
DEFAULT_MAX_EQUALITY_RESIDUAL = 1e-8


class Plan083CpuScenePacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        choices=sorted(SCENE_ROWS),
        default=DEFAULT_SCENE,
        help="Reduced CPU scene packet to run and validate.",
    )
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        default=None,
        help="Google Benchmark JSON path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
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
        "--max-equality-residual",
        type=float,
        default=DEFAULT_MAX_EQUALITY_RESIDUAL,
        help="Maximum accepted final equality residual for the reduced scene.",
    )
    parser.add_argument(
        "--skip-run",
        action="store_true",
        help="Validate an existing Google Benchmark JSON file.",
    )
    args = parser.parse_args(argv)
    if args.benchmark_json is None:
        args.benchmark_json = SCENE_BENCHMARK_OUTPUTS[args.scene]
    if args.output is None:
        args.output = SCENE_PACKET_OUTPUTS[args.scene]
    return args


def run_benchmark(args: argparse.Namespace) -> None:
    args.benchmark_json.parent.mkdir(parents=True, exist_ok=True)
    benchmark_row = SCENE_ROWS[args.scene]
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_cpu_scene_corpus",
        "--build-type",
        args.build_type,
        "--",
        f"--benchmark_filter=^{benchmark_row}$",
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
        raise Plan083CpuScenePacketError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise Plan083CpuScenePacketError(f"{path}: JSON root must be an object")
    return data


def _finite_number(row: Mapping[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise Plan083CpuScenePacketError(
            f"{benchmark_row_name(row)} is missing finite counter {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise Plan083CpuScenePacketError(
            f"{benchmark_row_name(row)} has non-finite counter {key}"
        )
    return value


def _packet_row_name(row: Mapping[str, Any]) -> str:
    name = canonical_benchmark_name(benchmark_row_name(row))
    if name.endswith("/real_time"):
        return name[: -len("/real_time")]
    return name


def _representative_row(
    rows: list[Mapping[str, Any]], expected_row: str
) -> Mapping[str, Any]:
    found: Mapping[str, Any] | None = None
    errors: list[str] = []

    for row in rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical != expected_row:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found = row
        errors.extend(benchmark_timing_field_errors(row, name))

    if found is None:
        errors.append(f"missing median benchmark row: {expected_row}")

    if errors:
        raise Plan083CpuScenePacketError("\n".join(errors))
    return found


def make_packet(
    benchmark_data: dict[str, Any],
    *,
    max_equality_residual: float,
    scene: str = DEFAULT_SCENE,
) -> dict[str, Any]:
    if scene not in SCENE_ROWS:
        raise Plan083CpuScenePacketError(f"unsupported scene: {scene}")

    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list) or not rows:
        raise Plan083CpuScenePacketError("benchmark JSON has no benchmark rows")
    typed_rows = [row for row in rows if isinstance(row, Mapping)]
    if len(typed_rows) != len(rows):
        raise Plan083CpuScenePacketError("benchmark JSON has non-object rows")

    row = _representative_row(typed_rows, SCENE_ROWS[scene])
    timing_ns = benchmark_timing_ns(row)
    if not math.isfinite(timing_ns) or timing_ns <= 0.0:
        raise Plan083CpuScenePacketError("benchmark timing is not positive")

    scene_label = scene.replace("_", " ")
    failed_steps = _finite_number(row, "failed_steps")
    if failed_steps != 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced {scene_label} reported {failed_steps:g} failed steps"
        )

    final_residual = _finite_number(row, "final_equality_residual_norm")
    if final_residual > max_equality_residual:
        raise Plan083CpuScenePacketError(
            f"reduced {scene_label} equality residual "
            f"{final_residual:.3g} exceeds {max_equality_residual:.3g}"
        )

    if scene == "hanging_bridge":
        return _make_hanging_bridge_packet(
            row,
            rows,
            timing_ns=timing_ns,
            final_residual=final_residual,
            max_equality_residual=max_equality_residual,
        )
    if scene == "nunchaku_single":
        return _make_nunchaku_packet(
            row,
            rows,
            timing_ns=timing_ns,
            final_residual=final_residual,
            max_equality_residual=max_equality_residual,
        )
    if scene == "precession":
        return _make_precession_packet(
            row,
            rows,
            timing_ns=timing_ns,
            final_residual=final_residual,
            max_equality_residual=max_equality_residual,
        )
    if scene == "ragdoll_reduced":
        return _make_ragdoll_packet(
            row,
            rows,
            timing_ns=timing_ns,
            final_residual=final_residual,
            max_equality_residual=max_equality_residual,
        )
    if scene == "terrain_vehicle":
        return _make_terrain_vehicle_packet(
            row,
            rows,
            timing_ns=timing_ns,
            final_residual=final_residual,
            max_equality_residual=max_equality_residual,
        )
    if scene == "windmill":
        return _make_windmill_packet(
            row,
            rows,
            timing_ns=timing_ns,
            final_residual=final_residual,
            max_equality_residual=max_equality_residual,
        )
    raise Plan083CpuScenePacketError(f"unsupported scene: {scene}")


def _make_hanging_bridge_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
    final_residual: float,
    max_equality_residual: float,
) -> dict[str, Any]:
    body_count = int(_finite_number(row, "body_count"))
    dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
    fixed_joint_count = int(_finite_number(row, "fixed_joint_count"))
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 7:
        raise Plan083CpuScenePacketError(f"expected 7 rigid bodies, got {body_count}")
    if dynamic_body_count != 5:
        raise Plan083CpuScenePacketError(
            f"expected 5 dynamic rigid bodies, got {dynamic_body_count}"
        )
    if fixed_joint_count != 4:
        raise Plan083CpuScenePacketError(
            f"expected 4 point-connection joints, got {fixed_joint_count}"
        )
    if active_articulation_constraints < 12:
        raise Plan083CpuScenePacketError(
            "expected fixed-joint articulation rows for the reduced bridge"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-02",
            "scene_id": "plan083_hanging_bridge",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "fixed_joint_count": fixed_joint_count,
            "active_articulation_constraints": active_articulation_constraints,
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "traveler_height_m": _finite_number(row, "traveler_height_m"),
            "max_board_sag_m": _finite_number(row, "max_board_sag_m"),
            "limitation_status": (
                "Reduced runtime smoke packet only; paper-scale rods, "
                "codimensional deformables, and full Table 2 counts remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_nunchaku_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
    final_residual: float,
    max_equality_residual: float,
) -> dict[str, Any]:
    body_count = int(_finite_number(row, "body_count"))
    dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
    revolute_joint_count = int(_finite_number(row, "revolute_joint_count"))
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 2:
        raise Plan083CpuScenePacketError(f"expected 2 rigid bodies, got {body_count}")
    if dynamic_body_count != 1:
        raise Plan083CpuScenePacketError(
            f"expected 1 dynamic rigid body, got {dynamic_body_count}"
        )
    if revolute_joint_count != 1:
        raise Plan083CpuScenePacketError(
            f"expected 1 revolute joint, got {revolute_joint_count}"
        )
    if active_articulation_constraints < 2:
        raise Plan083CpuScenePacketError(
            "expected revolute-joint articulation rows for the reduced nunchaku"
        )

    tip_radius = _finite_number(row, "swinging_tip_radius_m")
    if tip_radius <= 0.0:
        raise Plan083CpuScenePacketError("nunchaku swinging tip radius is not positive")

    free_axis_velocity = _finite_number(row, "free_axis_angular_velocity_rad_s")
    if abs(free_axis_velocity) <= 1e-9:
        raise Plan083CpuScenePacketError("nunchaku free-axis angular velocity was lost")

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-13",
            "scene_id": "plan083_nunchaku",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "revolute_joint_count": revolute_joint_count,
            "active_articulation_constraints": active_articulation_constraints,
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "swinging_tip_radius_m": tip_radius,
            "free_axis_angular_velocity_rad_s": free_axis_velocity,
            "limitation_status": (
                "Reduced single-hinge runtime smoke packet only; cone-twist "
                "ranges, sparse N-by-N scaling, and Fig. 25 remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_windmill_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
    final_residual: float,
    max_equality_residual: float,
) -> dict[str, Any]:
    body_count = int(_finite_number(row, "body_count"))
    dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
    revolute_joint_count = int(_finite_number(row, "revolute_joint_count"))
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 3:
        raise Plan083CpuScenePacketError(f"expected 3 rigid bodies, got {body_count}")
    if dynamic_body_count != 2:
        raise Plan083CpuScenePacketError(
            f"expected 2 dynamic rigid bodies, got {dynamic_body_count}"
        )
    if revolute_joint_count != 1:
        raise Plan083CpuScenePacketError(
            f"expected 1 revolute joint, got {revolute_joint_count}"
        )
    if active_articulation_constraints < 2:
        raise Plan083CpuScenePacketError(
            "expected revolute-joint articulation rows for the reduced windmill"
        )

    blade_tip_radius = _finite_number(row, "blade_tip_radius_m")
    if blade_tip_radius <= 0.0:
        raise Plan083CpuScenePacketError("windmill blade tip radius is not positive")

    striker_clearance = _finite_number(row, "striker_blade_clearance_m")
    if striker_clearance < -1e-5:
        raise Plan083CpuScenePacketError(
            "windmill striker penetrated the reduced blade smoke scene"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-20",
            "scene_id": "plan083_windmill",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "revolute_joint_count": revolute_joint_count,
            "active_constraints": int(_finite_number(row, "active_constraints")),
            "active_friction_constraints": int(
                _finite_number(row, "active_friction_constraints")
            ),
            "active_articulation_constraints": active_articulation_constraints,
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "blade_tip_radius_m": blade_tip_radius,
            "striker_height_m": _finite_number(row, "striker_height_m"),
            "striker_blade_clearance_m": striker_clearance,
            "limitation_status": (
                "Reduced hinge/contact smoke packet only; Bullet/reference "
                "comparison, cube piles, and paper-scale timing remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_terrain_vehicle_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
    final_residual: float,
    max_equality_residual: float,
) -> dict[str, Any]:
    body_count = int(_finite_number(row, "body_count"))
    dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
    wheel_count = int(_finite_number(row, "wheel_count"))
    revolute_joint_count = int(_finite_number(row, "revolute_joint_count"))
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 6:
        raise Plan083CpuScenePacketError(f"expected 6 rigid bodies, got {body_count}")
    if dynamic_body_count != 5:
        raise Plan083CpuScenePacketError(
            f"expected 5 dynamic rigid bodies, got {dynamic_body_count}"
        )
    if wheel_count != 4:
        raise Plan083CpuScenePacketError(
            f"expected 4 passive wheels, got {wheel_count}"
        )
    if revolute_joint_count != 4:
        raise Plan083CpuScenePacketError(
            f"expected 4 revolute wheel joints, got {revolute_joint_count}"
        )
    if active_articulation_constraints < 8:
        raise Plan083CpuScenePacketError(
            "expected wheel-hinge articulation rows for the reduced terrain vehicle"
        )

    ground_clearance = _finite_number(row, "min_wheel_ground_clearance_m")
    if ground_clearance < -1e-5:
        raise Plan083CpuScenePacketError(
            "terrain vehicle wheel penetrated the reduced terrain smoke scene"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-10",
            "scene_id": "plan083_terrain_vehicle",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "wheel_count": wheel_count,
            "revolute_joint_count": revolute_joint_count,
            "active_constraints": int(_finite_number(row, "active_constraints")),
            "active_friction_constraints": int(
                _finite_number(row, "active_friction_constraints")
            ),
            "active_articulation_constraints": active_articulation_constraints,
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "chassis_height_m": _finite_number(row, "chassis_height_m"),
            "min_wheel_ground_clearance_m": ground_clearance,
            "limitation_status": (
                "Reduced chassis/passive-wheel terrain smoke packet only; "
                "paper-scale terrain mesh, navigation controls, and Table 2 "
                "timings remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_precession_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
    final_residual: float,
    max_equality_residual: float,
) -> dict[str, Any]:
    body_count = int(_finite_number(row, "body_count"))
    dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
    active_constraints = int(_finite_number(row, "active_constraints"))
    active_friction_constraints = int(
        _finite_number(row, "active_friction_constraints")
    )
    if body_count != 2:
        raise Plan083CpuScenePacketError(f"expected 2 rigid bodies, got {body_count}")
    if dynamic_body_count != 1:
        raise Plan083CpuScenePacketError(
            f"expected 1 dynamic rigid body, got {dynamic_body_count}"
        )
    if active_constraints <= 0:
        raise Plan083CpuScenePacketError(
            "expected active wheel-ground contact rows for the reduced precession scene"
        )
    if active_friction_constraints <= 0:
        raise Plan083CpuScenePacketError(
            "expected active wheel-ground friction rows for the reduced precession scene"
        )

    ground_clearance = _finite_number(row, "wheel_ground_clearance_m")
    if ground_clearance < -1e-5:
        raise Plan083CpuScenePacketError(
            "precession wheel penetrated the reduced ground smoke scene"
        )

    spin_rate = _finite_number(row, "spin_rate_rad_s")
    if spin_rate <= 1e-9:
        raise Plan083CpuScenePacketError("precession spin rate was lost")

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-23",
            "scene_id": "plan083_precession",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "active_constraints": active_constraints,
            "active_friction_constraints": active_friction_constraints,
            "solver_iterations": int(_finite_number(row, "solver_iterations")),
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "wheel_height_m": _finite_number(row, "wheel_height_m"),
            "wheel_ground_clearance_m": ground_clearance,
            "spin_rate_rad_s": spin_rate,
            "limitation_status": (
                "Reduced rolling-wheel runtime smoke packet only; angular-velocity "
                "sweep, rolling-contact model, and Table 2 timing remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_ragdoll_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
    final_residual: float,
    max_equality_residual: float,
) -> dict[str, Any]:
    body_count = int(_finite_number(row, "body_count"))
    dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
    ragdoll_body_count = int(_finite_number(row, "ragdoll_body_count"))
    revolute_joint_count = int(_finite_number(row, "revolute_joint_count"))
    active_constraints = int(_finite_number(row, "active_constraints"))
    active_friction_constraints = int(
        _finite_number(row, "active_friction_constraints")
    )
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 7:
        raise Plan083CpuScenePacketError(f"expected 7 rigid bodies, got {body_count}")
    if dynamic_body_count != 6:
        raise Plan083CpuScenePacketError(
            f"expected 6 dynamic rigid bodies, got {dynamic_body_count}"
        )
    if ragdoll_body_count != 6:
        raise Plan083CpuScenePacketError(
            f"expected 6 reduced ragdoll bodies, got {ragdoll_body_count}"
        )
    if revolute_joint_count != 5:
        raise Plan083CpuScenePacketError(
            f"expected 5 reduced ragdoll revolute joints, got {revolute_joint_count}"
        )
    if active_articulation_constraints < 10:
        raise Plan083CpuScenePacketError(
            "expected revolute-joint articulation rows for the reduced ragdoll"
        )
    if active_constraints <= 0:
        raise Plan083CpuScenePacketError(
            "expected active ground-contact rows for the reduced ragdoll"
        )
    if active_friction_constraints <= 0:
        raise Plan083CpuScenePacketError(
            "expected active ground-friction rows for the reduced ragdoll"
        )

    ground_clearance = _finite_number(row, "min_leg_ground_clearance_m")
    if ground_clearance < -1e-5:
        raise Plan083CpuScenePacketError(
            "ragdoll legs penetrated the reduced ground smoke scene"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-11",
            "scene_id": "plan083_ragdolls",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "ragdoll_body_count": ragdoll_body_count,
            "revolute_joint_count": revolute_joint_count,
            "active_constraints": active_constraints,
            "active_friction_constraints": active_friction_constraints,
            "active_articulation_constraints": active_articulation_constraints,
            "solver_iterations": int(_finite_number(row, "solver_iterations")),
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "torso_height_m": _finite_number(row, "torso_height_m"),
            "min_leg_ground_clearance_m": ground_clearance,
            "limitation_status": (
                "Reduced six-body revolute-chain runtime smoke packet only; "
                "cone-twist joints, 60-ragdoll scale, and Table 2 timing remain planned."
            ),
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
            max_equality_residual=args.max_equality_residual,
            scene=args.scene,
        )
    except Plan083CpuScenePacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    row = packet["plan083_cpu_scene_packet"]
    print(
        "PLAN-083 CPU scene packet OK: "
        f"{row['scene_id']} bodies={row['body_count']} "
        f"residual={row['final_equality_residual_norm']:.3g} "
        f"time={row['wall_time_ns']:.3g} ns"
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
