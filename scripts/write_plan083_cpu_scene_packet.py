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
NUNCHAKU_SCALING_SIZES = (20, 40, 60, 80, 100)
TIMING_BREAKDOWN_SCENES = (
    "hanging_bridge",
    "lying_flat",
    "pulley_system",
    "umbrella",
    "nunchaku_single",
    "terrain_vehicle",
    "ragdoll_reduced",
    "windmill",
    "candy",
    "precession",
)
TABLE2_REDUCED_SCENES = (
    "lying_flat",
    "hanging_bridge",
    "pulley_system",
    "umbrella",
    "terrain_vehicle",
    "ragdoll_reduced",
    "windmill",
    "candy",
    "precession",
)
ABD_CHAIN_SCENES = {
    "abd_chain_8": {
        "row_id": "abd-chain-8",
        "scene_id": "plan083_abd_chain_8",
        "pair_count": 8,
        "label": "8x8 chain net",
        "paper_gap": "63k-triangle chain-net assets, rigid IPC comparison timing, and paper-scale reproduction remain planned.",
    },
    "abd_chain_16": {
        "row_id": "abd-chain-16",
        "scene_id": "plan083_abd_chain_16",
        "pair_count": 16,
        "label": "16x16 chain net",
        "paper_gap": "445k-triangle chain-net assets, rigid IPC comparison timing, and paper-scale reproduction remain planned.",
    },
    "abd_chain_96": {
        "row_id": "abd-chain-96",
        "scene_id": "plan083_abd_chain_96",
        "pair_count": 96,
        "label": "96x96 chain net",
        "paper_gap": "12M-triangle chain-net assets, stress-machine policy, GPU packet, and paper-scale reproduction remain planned.",
    },
}
ABD_COMPARISON_SCENES = {
    "abd_gears": {
        "row_id": "abd-gears",
        "scene_id": "plan083_abd_gears",
        "pair_count": 28,
        "paper_body_count": 28,
        "paper_triangle_count": 2_500_000,
        "paper_gap": "2.5M-triangle gear assets, contact-force visualization, GPU parity, and paper-scale reproduction remain planned.",
        "reference_gap": "Deck GPU speedup is not measured; this packet records only reduced CPU ABD runtime-step evidence.",
    },
    "abd_bullet_small": {
        "row_id": "abd-bullet-small",
        "scene_id": "plan083_abd_bullet_small",
        "pair_count": 16,
        "paper_body_count": 16,
        "paper_triangle_count": 1_200,
        "paper_gap": "1.2k-triangle Bullet comparison asset and accepted Bullet/reference baseline command remain planned.",
        "reference_gap": "Deck Bullet timings are not reproduced; this packet records only reduced CPU ABD runtime-step evidence.",
    },
    "abd_bullet_medium": {
        "row_id": "abd-bullet-medium",
        "scene_id": "plan083_abd_bullet_medium",
        "pair_count": 48,
        "paper_body_count": 142,
        "paper_triangle_count": 3_500,
        "paper_gap": "3.5k-triangle Bullet comparison asset and accepted Bullet/reference baseline command remain planned.",
        "reference_gap": "Deck Bullet timings are not reproduced; this packet records only reduced CPU ABD runtime-step evidence.",
    },
    "abd_bullet_large": {
        "row_id": "abd-bullet-large",
        "scene_id": "plan083_abd_bullet_large",
        "pair_count": 96,
        "paper_body_count": 562,
        "paper_triangle_count": 11_000,
        "paper_gap": "11k-triangle Bullet comparison asset and accepted Bullet/reference baseline command remain planned.",
        "reference_gap": "Deck Bullet timings are not reproduced; this packet records only reduced CPU ABD runtime-step evidence.",
    },
    "abd_complex_geometry": {
        "row_id": "abd-complex-geometry",
        "scene_id": "plan083_abd_complex_geometry",
        "pair_count": 29,
        "paper_body_count": 29,
        "paper_triangle_count": 1_200_000,
        "paper_gap": "1.2M-triangle complex-geometry assets, contact-force visualization, and paper-scale reproduction remain planned.",
        "reference_gap": "Deck 2.8 s/step timing is not reproduced; this packet records only reduced CPU ABD runtime-step evidence.",
    },
    "abd_fem_coupling": {
        "row_id": "abd-fem-coupling",
        "scene_id": "plan083_abd_fem_coupling",
        "pair_count": 27,
        "paper_body_count": 27,
        "paper_triangle_count": 1_100_000,
        "paper_gap": "1.1M-triangle FEM-coupling assets, full runtime affine/FEM coupling, and paper-scale reproduction remain planned.",
        "reference_gap": "Deck 16.4 s/step timing is not reproduced; this packet records only reduced ABD/FEM micro-solve evidence.",
    },
}
SCENE_IDS = {
    "abd_chain_8": "plan083_abd_chain_8",
    "abd_chain_16": "plan083_abd_chain_16",
    "abd_chain_96": "plan083_abd_chain_96",
    "abd_gears": "plan083_abd_gears",
    "abd_bullet_small": "plan083_abd_bullet_small",
    "abd_bullet_medium": "plan083_abd_bullet_medium",
    "abd_bullet_large": "plan083_abd_bullet_large",
    "abd_complex_geometry": "plan083_abd_complex_geometry",
    "abd_fem_coupling": "plan083_abd_fem_coupling",
    "abd_house_of_cards": "plan083_abd_house_of_cards",
    "abd_wrecking_ball": "plan083_abd_wrecking_ball",
    "candy": "plan083_candy",
    "external_surface_ccd": "plan083_external_surface_ccd",
    "hanging_bridge": "plan083_hanging_bridge",
    "lying_flat": "plan083_lying_flat",
    "nunchaku_single": "plan083_nunchaku",
    "pulley_system": "plan083_pulley_system",
    "precession": "plan083_precession",
    "ragdoll_reduced": "plan083_ragdolls",
    "table_2": "plan083_reduced_table_2",
    "terrain_vehicle": "plan083_terrain_vehicle",
    "timing_breakdown": "plan083_reduced_timing_breakdown",
    "umbrella": "plan083_umbrella",
    "windmill": "plan083_windmill",
}
SCENE_ROW_IDS = {
    "abd_chain_8": "abd-chain-8",
    "abd_chain_16": "abd-chain-16",
    "abd_chain_96": "abd-chain-96",
    "abd_gears": "abd-gears",
    "abd_bullet_small": "abd-bullet-small",
    "abd_bullet_medium": "abd-bullet-medium",
    "abd_bullet_large": "abd-bullet-large",
    "abd_complex_geometry": "abd-complex-geometry",
    "abd_fem_coupling": "abd-fem-coupling",
    "abd_house_of_cards": "abd-vs-rigid-cards",
    "abd_wrecking_ball": "abd-vs-rigid-wreck",
    "candy": "unb-fig-22",
    "external_surface_ccd": "unb-alg-barriers",
    "hanging_bridge": "unb-fig-02",
    "lying_flat": "unb-fig-01",
    "nunchaku_single": "unb-fig-13",
    "pulley_system": "unb-fig-03",
    "precession": "unb-fig-23",
    "ragdoll_reduced": "unb-fig-11",
    "table_2": "unb-table-02",
    "terrain_vehicle": "unb-fig-10",
    "timing_breakdown": "unb-fig-24",
    "umbrella": "unb-fig-04",
    "windmill": "unb-fig-20",
}
SCENE_ROWS = {
    "abd_chain_8": "BM_Plan083CpuScene_abd_chain_8_reduced_pair_runtime_step",
    "abd_chain_16": "BM_Plan083CpuScene_abd_chain_16_reduced_pair_runtime_step",
    "abd_chain_96": "BM_Plan083CpuScene_abd_chain_96_reduced_pair_runtime_step",
    "abd_gears": "BM_Plan083CpuScene_abd_gears_reduced_pair_runtime_step",
    "abd_bullet_small": "BM_Plan083CpuScene_abd_bullet_small_reduced_pair_runtime_step",
    "abd_bullet_medium": "BM_Plan083CpuScene_abd_bullet_medium_reduced_pair_runtime_step",
    "abd_bullet_large": "BM_Plan083CpuScene_abd_bullet_large_reduced_pair_runtime_step",
    "abd_complex_geometry": "BM_Plan083CpuScene_abd_complex_geometry_reduced_pair_runtime_step",
    "abd_fem_coupling": "BM_Plan083CpuScene_abd_fem_coupling_reduced_side_by_side_step",
    "abd_house_of_cards": "BM_Plan083CpuScene_abd_house_of_cards_reduced_runtime_step",
    "abd_wrecking_ball": "BM_Plan083CpuScene_abd_wrecking_ball_reduced_pair_runtime_step",
    "candy": "BM_Plan083CpuScene_candy_reduced_world_step",
    "external_surface_ccd": "BM_Plan083CpuScene_external_surface_ccd_diagnostics",
    "hanging_bridge": "BM_Plan083CpuScene_hanging_bridge_reduced_world_step",
    "lying_flat": "BM_Plan083CpuScene_lying_flat_reduced_world_step",
    "nunchaku_single": "BM_Plan083CpuScene_nunchaku_single_reduced_world_step",
    "nunchaku_scaling": "BM_Plan083CpuScene_nunchaku_scaling_reduced_world_step",
    "pulley_system": "BM_Plan083CpuScene_pulley_system_reduced_world_step",
    "precession": "BM_Plan083CpuScene_precession_reduced_world_step",
    "ragdoll_reduced": "BM_Plan083CpuScene_ragdoll_reduced_world_step",
    "table_2": "BM_Plan083CpuScene_reduced_table_2",
    "terrain_vehicle": "BM_Plan083CpuScene_terrain_vehicle_reduced_world_step",
    "timing_breakdown": "BM_Plan083CpuScene_reduced_timing_breakdown",
    "umbrella": "BM_Plan083CpuScene_umbrella_reduced_world_step",
    "windmill": "BM_Plan083CpuScene_windmill_reduced_world_step",
}
TIMING_BREAKDOWN_ROWS = {scene: SCENE_ROWS[scene] for scene in TIMING_BREAKDOWN_SCENES}
TABLE2_REDUCED_ROWS = {scene: SCENE_ROWS[scene] for scene in TABLE2_REDUCED_SCENES}
SCENE_BENCHMARK_OUTPUTS = {
    "abd_chain_8": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_chain_8_benchmark.json"
    ),
    "abd_chain_16": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_chain_16_benchmark.json"
    ),
    "abd_chain_96": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_chain_96_benchmark.json"
    ),
    "abd_gears": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_gears_benchmark.json"
    ),
    "abd_bullet_small": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_bullet_small_benchmark.json"
    ),
    "abd_bullet_medium": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_bullet_medium_benchmark.json"
    ),
    "abd_bullet_large": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_bullet_large_benchmark.json"
    ),
    "abd_complex_geometry": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_complex_geometry_benchmark.json"
    ),
    "abd_fem_coupling": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_fem_coupling_benchmark.json"
    ),
    "abd_house_of_cards": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_house_of_cards_benchmark.json"
    ),
    "abd_wrecking_ball": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_wrecking_ball_benchmark.json"
    ),
    "candy": Path(".benchmark_results/plan083/cpu_scene_corpus/candy_benchmark.json"),
    "external_surface_ccd": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/external_surface_ccd_benchmark.json"
    ),
    "hanging_bridge": DEFAULT_BENCHMARK_OUTPUT,
    "lying_flat": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/lying_flat_benchmark.json"
    ),
    "nunchaku_single": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/nunchaku_single_benchmark.json"
    ),
    "nunchaku_scaling": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/nunchaku_scaling_benchmark.json"
    ),
    "pulley_system": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/pulley_system_benchmark.json"
    ),
    "precession": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/precession_benchmark.json"
    ),
    "ragdoll_reduced": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/ragdolls_benchmark.json"
    ),
    "table_2": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/table_2_benchmark.json"
    ),
    "terrain_vehicle": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/terrain_vehicle_benchmark.json"
    ),
    "timing_breakdown": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/timing_breakdown_benchmark.json"
    ),
    "umbrella": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/umbrella_benchmark.json"
    ),
    "windmill": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/windmill_benchmark.json"
    ),
}
SCENE_PACKET_OUTPUTS = {
    "abd_chain_8": Path(".benchmark_results/plan083/cpu_scene_corpus/abd_chain_8.json"),
    "abd_chain_16": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_chain_16.json"
    ),
    "abd_chain_96": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_chain_96.json"
    ),
    "abd_gears": Path(".benchmark_results/plan083/cpu_scene_corpus/abd_gears.json"),
    "abd_bullet_small": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_bullet_small.json"
    ),
    "abd_bullet_medium": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_bullet_medium.json"
    ),
    "abd_bullet_large": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_bullet_large.json"
    ),
    "abd_complex_geometry": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_complex_geometry.json"
    ),
    "abd_fem_coupling": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_fem_coupling.json"
    ),
    "abd_house_of_cards": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_house_of_cards.json"
    ),
    "abd_wrecking_ball": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/abd_wrecking_ball.json"
    ),
    "candy": Path(".benchmark_results/plan083/cpu_scene_corpus/candy.json"),
    "external_surface_ccd": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/external_surface_ccd.json"
    ),
    "hanging_bridge": DEFAULT_PACKET_OUTPUT,
    "lying_flat": Path(".benchmark_results/plan083/cpu_scene_corpus/lying_flat.json"),
    "nunchaku_single": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/nunchaku_single.json"
    ),
    "nunchaku_scaling": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/nunchaku_scaling.json"
    ),
    "pulley_system": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/pulley_system.json"
    ),
    "precession": Path(".benchmark_results/plan083/cpu_scene_corpus/precession.json"),
    "ragdoll_reduced": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/ragdolls.json"
    ),
    "table_2": Path(".benchmark_results/plan083/cpu_scene_corpus/table_2.json"),
    "terrain_vehicle": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/terrain_vehicle.json"
    ),
    "timing_breakdown": Path(
        ".benchmark_results/plan083/cpu_scene_corpus/timing_breakdown.json"
    ),
    "umbrella": Path(".benchmark_results/plan083/cpu_scene_corpus/umbrella.json"),
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
    benchmark_filter = f"^{benchmark_row}$"
    if args.scene == "timing_breakdown":
        rows = "|".join(TIMING_BREAKDOWN_ROWS.values())
        benchmark_filter = f"^({rows})$"
    if args.scene == "table_2":
        rows = "|".join(TABLE2_REDUCED_ROWS.values())
        benchmark_filter = f"^({rows})$"
    if args.scene == "nunchaku_scaling":
        benchmark_filter = f"^{benchmark_row}/"
    command = [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        "bm_plan083_cpu_scene_corpus",
        "--build-type",
        args.build_type,
        "--",
        f"--benchmark_filter={benchmark_filter}",
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


def _optional_finite_number(row: Mapping[str, Any], key: str) -> float:
    if key not in row:
        return 0.0
    return _finite_number(row, key)


SURFACE_CONTACT_RUNTIME_COUNTER_KEYS = (
    "line_search_trials",
    "surface_contact_candidate_builds",
    "surface_contact_point_triangle_candidates",
    "surface_contact_edge_edge_candidates",
    "surface_contact_ccd_point_triangle_checks",
    "surface_contact_ccd_edge_edge_checks",
    "surface_contact_ccd_hits",
    "surface_contact_ccd_limited_steps",
    "surface_contact_ccd_zero_step_count",
)

EXTERNAL_SURFACE_CONTACT_RUNTIME_COUNTER_KEYS = (
    "inter_body_surface_contact_candidate_builds",
    "inter_body_surface_contact_point_triangle_candidates",
    "inter_body_surface_contact_edge_edge_candidates",
    "inter_body_surface_contact_ccd_point_triangle_checks",
    "inter_body_surface_contact_ccd_edge_edge_checks",
    "inter_body_surface_contact_ccd_hits",
    "inter_body_surface_contact_ccd_limited_steps",
    "inter_body_surface_contact_ccd_zero_step_count",
    "static_rigid_surface_ccd_snapshot_builds",
    "static_rigid_surface_ccd_box_count",
    "static_rigid_surface_ccd_sphere_count",
    "static_rigid_surface_ccd_triangle_count",
    "static_rigid_surface_ccd_edge_count",
    "static_rigid_surface_ccd_candidate_builds",
    "static_rigid_surface_ccd_point_triangle_candidates",
    "static_rigid_surface_ccd_edge_edge_candidates",
    "static_rigid_surface_ccd_point_triangle_checks",
    "static_rigid_surface_ccd_edge_edge_checks",
    "static_rigid_surface_ccd_hits",
    "static_rigid_surface_ccd_limited_steps",
    "static_rigid_surface_ccd_zero_step_count",
    "moving_rigid_surface_ccd_snapshot_builds",
    "moving_rigid_surface_ccd_box_count",
    "moving_rigid_surface_ccd_sample_count",
    "moving_rigid_surface_ccd_inflated_box_count",
    "moving_rigid_surface_ccd_triangle_count",
    "moving_rigid_surface_ccd_edge_count",
    "moving_rigid_surface_ccd_candidate_builds",
    "moving_rigid_surface_ccd_point_triangle_candidates",
    "moving_rigid_surface_ccd_edge_edge_candidates",
    "moving_rigid_surface_ccd_point_triangle_checks",
    "moving_rigid_surface_ccd_edge_edge_checks",
    "moving_rigid_surface_ccd_hits",
    "moving_rigid_surface_ccd_limited_steps",
    "moving_rigid_surface_ccd_zero_step_count",
)

DEFORMABLE_RUNTIME_CONTACT_COUNTER_KEYS = (
    SURFACE_CONTACT_RUNTIME_COUNTER_KEYS + EXTERNAL_SURFACE_CONTACT_RUNTIME_COUNTER_KEYS
)

EXTERNAL_SURFACE_CCD_REQUIRED_COUNTER_KEYS = (
    "inter_body_surface_contact_candidate_builds",
    "inter_body_surface_contact_point_triangle_candidates",
    "inter_body_surface_contact_ccd_point_triangle_checks",
    "inter_body_surface_contact_ccd_hits",
    "inter_body_surface_contact_ccd_limited_steps",
    "static_rigid_surface_ccd_snapshot_builds",
    "static_rigid_surface_ccd_box_count",
    "static_rigid_surface_ccd_triangle_count",
    "static_rigid_surface_ccd_edge_count",
    "static_rigid_surface_ccd_candidate_builds",
    "static_rigid_surface_ccd_point_triangle_candidates",
    "static_rigid_surface_ccd_point_triangle_checks",
    "static_rigid_surface_ccd_hits",
    "static_rigid_surface_ccd_limited_steps",
    "moving_rigid_surface_ccd_snapshot_builds",
    "moving_rigid_surface_ccd_box_count",
    "moving_rigid_surface_ccd_sample_count",
    "moving_rigid_surface_ccd_triangle_count",
    "moving_rigid_surface_ccd_edge_count",
    "moving_rigid_surface_ccd_candidate_builds",
    "moving_rigid_surface_ccd_point_triangle_candidates",
    "moving_rigid_surface_ccd_point_triangle_checks",
    "moving_rigid_surface_ccd_hits",
    "moving_rigid_surface_ccd_limited_steps",
)

MIXED_EXTERNAL_SURFACE_CCD_REQUIRED_COUNTER_KEYS = (
    "mixed_external_surface_ccd_scene_count",
    "mixed_external_surface_ccd_family_count",
    "mixed_inter_body_surface_contact_candidate_builds",
    "mixed_inter_body_surface_contact_point_triangle_candidates",
    "mixed_inter_body_surface_contact_ccd_point_triangle_checks",
    "mixed_inter_body_surface_contact_ccd_hits",
    "mixed_inter_body_surface_contact_ccd_limited_steps",
    "mixed_static_rigid_surface_ccd_candidate_builds",
    "mixed_static_rigid_surface_ccd_point_triangle_candidates",
    "mixed_static_rigid_surface_ccd_point_triangle_checks",
    "mixed_static_rigid_surface_ccd_hits",
    "mixed_static_rigid_surface_ccd_limited_steps",
    "mixed_moving_rigid_surface_ccd_candidate_builds",
    "mixed_moving_rigid_surface_ccd_point_triangle_candidates",
    "mixed_moving_rigid_surface_ccd_point_triangle_checks",
    "mixed_moving_rigid_surface_ccd_hits",
    "mixed_moving_rigid_surface_ccd_limited_steps",
)


def _surface_contact_runtime_counters(row: Mapping[str, Any]) -> dict[str, int]:
    counters = {
        key: int(_finite_number(row, key))
        for key in DEFORMABLE_RUNTIME_CONTACT_COUNTER_KEYS
    }
    for key, value in counters.items():
        if value < 0:
            raise Plan083CpuScenePacketError(
                f"{benchmark_row_name(row)} has negative counter {key}"
            )

    for label, point_key, edge_key, hits_key in (
        (
            "surface-contact CCD",
            "surface_contact_ccd_point_triangle_checks",
            "surface_contact_ccd_edge_edge_checks",
            "surface_contact_ccd_hits",
        ),
        (
            "inter-body surface-contact CCD",
            "inter_body_surface_contact_ccd_point_triangle_checks",
            "inter_body_surface_contact_ccd_edge_edge_checks",
            "inter_body_surface_contact_ccd_hits",
        ),
        (
            "static-rigid surface CCD",
            "static_rigid_surface_ccd_point_triangle_checks",
            "static_rigid_surface_ccd_edge_edge_checks",
            "static_rigid_surface_ccd_hits",
        ),
        (
            "moving-rigid surface CCD",
            "moving_rigid_surface_ccd_point_triangle_checks",
            "moving_rigid_surface_ccd_edge_edge_checks",
            "moving_rigid_surface_ccd_hits",
        ),
    ):
        ccd_checks = counters[point_key] + counters[edge_key]
        if counters[hits_key] > ccd_checks:
            row_name = benchmark_row_name(row)
            raise Plan083CpuScenePacketError(
                f"{row_name} has more {label} hits than CCD checks"
            )
    return counters


def _external_surface_ccd_sidecar_payload(
    row: Mapping[str, Any], *, scene_label: str
) -> dict[str, int]:
    external_sidecar_scene_count = int(
        _finite_number(row, "external_surface_ccd_sidecar_scene_count")
    )
    deformable_sidecar_body_count = int(
        _finite_number(row, "deformable_sidecar_body_count")
    )
    deformable_sidecar_node_count = int(
        _finite_number(row, "deformable_sidecar_node_count")
    )
    deformable_sidecar_edge_count = int(
        _finite_number(row, "deformable_sidecar_edge_count")
    )
    deformable_sidecar_surface_triangle_count = int(
        _finite_number(row, "deformable_sidecar_surface_triangle_count")
    )
    if external_sidecar_scene_count != 1:
        raise Plan083CpuScenePacketError(
            f"reduced {scene_label} packet needs one external surface CCD sidecar"
        )
    if deformable_sidecar_body_count != 3:
        raise Plan083CpuScenePacketError(
            f"expected reduced {scene_label} external surface CCD sidecar to "
            f"step 3 deformable bodies, got {deformable_sidecar_body_count}"
        )
    if deformable_sidecar_node_count != 31:
        raise Plan083CpuScenePacketError(
            f"expected reduced {scene_label} external surface CCD sidecar to "
            f"step 31 deformable nodes, got {deformable_sidecar_node_count}"
        )
    if deformable_sidecar_edge_count < 68:
        raise Plan083CpuScenePacketError(
            f"expected reduced {scene_label} external surface CCD sidecar to "
            "include structural and shear spring edges"
        )
    if deformable_sidecar_surface_triangle_count != 32:
        raise Plan083CpuScenePacketError(
            f"expected reduced {scene_label} external surface CCD sidecar to "
            "carry 32 surface triangles"
        )
    counters = _surface_contact_runtime_counters(row)
    for key in EXTERNAL_SURFACE_CCD_REQUIRED_COUNTER_KEYS:
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                f"reduced {scene_label} packet needs positive external surface "
                f"CCD sidecar counter {key}"
            )

    return {
        "external_surface_ccd_sidecar_scene_count": external_sidecar_scene_count,
        "deformable_sidecar_body_count": deformable_sidecar_body_count,
        "deformable_sidecar_node_count": deformable_sidecar_node_count,
        "deformable_sidecar_edge_count": deformable_sidecar_edge_count,
        "deformable_sidecar_surface_triangle_count": (
            deformable_sidecar_surface_triangle_count
        ),
        **counters,
    }


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

    if scene == "nunchaku_scaling":
        return _make_nunchaku_scaling_packet(
            typed_rows,
            rows,
            max_equality_residual=max_equality_residual,
        )
    if scene == "timing_breakdown":
        return _make_timing_breakdown_packet(
            typed_rows,
            rows,
            max_equality_residual=max_equality_residual,
        )
    if scene == "table_2":
        return _make_table2_packet(
            typed_rows,
            rows,
            max_equality_residual=max_equality_residual,
        )

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

    if scene == "lying_flat":
        return _make_lying_flat_packet(row, rows, timing_ns=timing_ns)
    if scene == "candy":
        return _make_candy_packet(row, rows, timing_ns=timing_ns)
    if scene == "abd_house_of_cards":
        return _make_abd_house_of_cards_packet(row, rows, timing_ns=timing_ns)
    if scene == "abd_wrecking_ball":
        return _make_abd_wrecking_ball_packet(row, rows, timing_ns=timing_ns)
    if scene in ABD_CHAIN_SCENES:
        return _make_abd_chain_packet(scene, row, rows, timing_ns=timing_ns)
    if scene == "abd_fem_coupling":
        return _make_abd_fem_coupling_packet(row, rows, timing_ns=timing_ns)
    if scene in ABD_COMPARISON_SCENES:
        return _make_abd_comparison_packet(scene, row, rows, timing_ns=timing_ns)
    if scene == "external_surface_ccd":
        return _make_external_surface_ccd_packet(row, rows, timing_ns=timing_ns)

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
    if scene == "pulley_system":
        return _make_pulley_packet(
            row,
            rows,
            timing_ns=timing_ns,
            final_residual=final_residual,
            max_equality_residual=max_equality_residual,
        )
    if scene == "umbrella":
        return _make_umbrella_packet(
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


def _make_lying_flat_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    rigid_obstacle_count = int(_finite_number(row, "rigid_obstacle_count"))
    deformable_body_count = int(_finite_number(row, "deformable_body_count"))
    deformable_node_count = int(_finite_number(row, "deformable_node_count"))
    deformable_edge_count = int(_finite_number(row, "deformable_edge_count"))
    surface_triangle_count = int(_finite_number(row, "surface_triangle_count"))
    if rigid_obstacle_count < 4:
        raise Plan083CpuScenePacketError(
            f"expected at least 4 reduced lying-flat obstacles, got {rigid_obstacle_count}"
        )
    if deformable_body_count != 3:
        raise Plan083CpuScenePacketError(
            "expected reduced lying-flat cloth plus two inter-body witness "
            f"deformable bodies, got {deformable_body_count}"
        )
    if deformable_node_count != 31:
        raise Plan083CpuScenePacketError(
            "expected 24 cloth nodes plus 7 inter-body witness nodes, "
            f"got {deformable_node_count}"
        )
    if deformable_edge_count < 68:
        raise Plan083CpuScenePacketError(
            "expected structural and shear spring edges for reduced lying-flat cloth"
        )
    if surface_triangle_count != 32:
        raise Plan083CpuScenePacketError(
            "expected 30 cloth surface triangles plus 2 inter-body witness "
            f"triangles, got {surface_triangle_count}"
        )

    min_cloth_height = _finite_number(row, "min_cloth_height_m")
    cloth_span_x = _finite_number(row, "cloth_span_x_m")
    cloth_span_y = _finite_number(row, "cloth_span_y_m")
    if min_cloth_height <= 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced lying-flat cloth penetrated below z=0: {min_cloth_height:.3g}"
        )
    if cloth_span_x <= 0.0 or cloth_span_y <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced lying-flat cloth span is not positive"
        )
    counters = _surface_contact_runtime_counters(row)
    for key in (
        "inter_body_surface_contact_candidate_builds",
        "inter_body_surface_contact_point_triangle_candidates",
        "inter_body_surface_contact_ccd_point_triangle_checks",
        "inter_body_surface_contact_ccd_hits",
        "inter_body_surface_contact_ccd_limited_steps",
    ):
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                "reduced lying-flat packet needs positive inter-body "
                f"surface CCD witness counter {key}"
            )
    if counters["static_rigid_surface_ccd_box_count"] <= 0:
        raise Plan083CpuScenePacketError(
            "reduced lying-flat packet needs a static-rigid surface CCD witness box"
        )
    for key in (
        "static_rigid_surface_ccd_candidate_builds",
        "static_rigid_surface_ccd_point_triangle_candidates",
        "static_rigid_surface_ccd_point_triangle_checks",
        "static_rigid_surface_ccd_hits",
        "static_rigid_surface_ccd_limited_steps",
    ):
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                "reduced lying-flat packet needs positive static-rigid "
                f"surface CCD witness counter {key}"
            )
    if counters["moving_rigid_surface_ccd_box_count"] <= 0:
        raise Plan083CpuScenePacketError(
            "reduced lying-flat packet needs a moving-rigid surface CCD witness box"
        )
    for key in (
        "moving_rigid_surface_ccd_sample_count",
        "moving_rigid_surface_ccd_candidate_builds",
        "moving_rigid_surface_ccd_point_triangle_candidates",
        "moving_rigid_surface_ccd_point_triangle_checks",
        "moving_rigid_surface_ccd_hits",
        "moving_rigid_surface_ccd_limited_steps",
    ):
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                "reduced lying-flat packet needs positive moving-rigid "
                f"surface CCD witness counter {key}"
            )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-01",
            "scene_id": "plan083_lying_flat",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "deformable IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "rigid_obstacle_count": rigid_obstacle_count,
            "deformable_body_count": deformable_body_count,
            "deformable_node_count": deformable_node_count,
            "deformable_edge_count": deformable_edge_count,
            "surface_triangle_count": surface_triangle_count,
            "solver_iterations": int(_finite_number(row, "solver_iterations")),
            "active_contact_count": int(_finite_number(row, "active_contact_count")),
            **counters,
            "friction_dissipation": _finite_number(row, "friction_dissipation"),
            "min_active_contact_distance_m": _finite_number(
                row, "min_active_contact_distance_m"
            ),
            "min_cloth_height_m": min_cloth_height,
            "cloth_span_x_m": cloth_span_x,
            "cloth_span_y_m": cloth_span_y,
            "limitation_status": (
                "Reduced deformable-cloth/inter-body/static and moving "
                "obstacle smoke packet only; rigid rings, deformable tori, "
                "rods, articulated ragdoll, cloth self-contact, production "
                "mixed coupling, and paper-scale mixed coupling remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_candy_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    rigid_obstacle_count = int(_finite_number(row, "rigid_obstacle_count"))
    deformable_body_count = int(_finite_number(row, "deformable_body_count"))
    deformable_node_count = int(_finite_number(row, "deformable_node_count"))
    deformable_edge_count = int(_finite_number(row, "deformable_edge_count"))
    surface_triangle_count = int(_finite_number(row, "surface_triangle_count"))
    if rigid_obstacle_count != 3:
        raise Plan083CpuScenePacketError(
            f"expected 3 rigid obstacles, got {rigid_obstacle_count}"
        )
    if deformable_body_count != 3:
        raise Plan083CpuScenePacketError(
            f"expected 3 deformable bodies, got {deformable_body_count}"
        )
    if deformable_node_count != 27:
        raise Plan083CpuScenePacketError(
            f"expected 27 deformable nodes, got {deformable_node_count}"
        )
    if deformable_edge_count < 70:
        raise Plan083CpuScenePacketError(
            "expected structural and shear spring edges for reduced candy cloth"
        )
    if surface_triangle_count != 32:
        raise Plan083CpuScenePacketError(
            f"expected 32 cloth surface triangles, got {surface_triangle_count}"
        )

    min_cloth_height = _finite_number(row, "min_cloth_height_m")
    cloth_span_x = _finite_number(row, "cloth_span_x_m")
    if min_cloth_height <= 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced candy cloth penetrated below z=0: {min_cloth_height:.3g}"
        )
    if cloth_span_x <= 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced candy cloth span is not positive: {cloth_span_x:.3g}"
        )
    counters = _surface_contact_runtime_counters(row)
    if counters["static_rigid_surface_ccd_box_count"] <= 0:
        raise Plan083CpuScenePacketError(
            "reduced candy packet needs a static-rigid surface CCD witness box"
        )
    for key in (
        "static_rigid_surface_ccd_candidate_builds",
        "static_rigid_surface_ccd_point_triangle_candidates",
        "static_rigid_surface_ccd_point_triangle_checks",
        "static_rigid_surface_ccd_hits",
        "static_rigid_surface_ccd_limited_steps",
    ):
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                "reduced candy packet needs positive static-rigid "
                f"surface CCD witness counter {key}"
            )
    if counters["moving_rigid_surface_ccd_box_count"] <= 0:
        raise Plan083CpuScenePacketError(
            "reduced candy packet needs a moving-rigid surface CCD witness box"
        )
    for key in (
        "moving_rigid_surface_ccd_sample_count",
        "moving_rigid_surface_ccd_candidate_builds",
        "moving_rigid_surface_ccd_point_triangle_candidates",
        "moving_rigid_surface_ccd_point_triangle_checks",
        "moving_rigid_surface_ccd_hits",
        "moving_rigid_surface_ccd_limited_steps",
    ):
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                "reduced candy packet needs positive moving-rigid "
                f"surface CCD witness counter {key}"
            )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-22",
            "scene_id": "plan083_candy",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "deformable IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "rigid_obstacle_count": rigid_obstacle_count,
            "deformable_body_count": deformable_body_count,
            "deformable_node_count": deformable_node_count,
            "deformable_edge_count": deformable_edge_count,
            "surface_triangle_count": surface_triangle_count,
            "solver_iterations": int(_finite_number(row, "solver_iterations")),
            "active_contact_count": int(_finite_number(row, "active_contact_count")),
            **counters,
            "friction_dissipation": _finite_number(row, "friction_dissipation"),
            "min_active_contact_distance_m": _finite_number(
                row, "min_active_contact_distance_m"
            ),
            "min_cloth_height_m": min_cloth_height,
            "cloth_span_x_m": cloth_span_x,
            "limitation_status": (
                "Reduced deformable-cloth/static-shell packet with a "
                "static-rigid and moving-rigid surface CCD witness only; "
                "affine body packing, twisted shell, cloth self-contact parity, "
                "and paper scale "
                "remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_external_surface_ccd_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    scene_count = int(_finite_number(row, "external_surface_ccd_scene_count"))
    deformable_body_count = int(_finite_number(row, "deformable_body_count"))
    deformable_node_count = int(_finite_number(row, "deformable_node_count"))
    deformable_edge_count = int(_finite_number(row, "deformable_edge_count"))
    surface_triangle_count = int(_finite_number(row, "surface_triangle_count"))
    rigid_obstacle_count = int(_finite_number(row, "rigid_obstacle_count"))
    static_rigid_obstacle_count = int(
        _finite_number(row, "static_rigid_obstacle_count")
    )
    moving_rigid_obstacle_count = int(
        _finite_number(row, "moving_rigid_obstacle_count")
    )
    if scene_count != 4:
        raise Plan083CpuScenePacketError(
            f"expected 4 external surface CCD diagnostic scenes, got {scene_count}"
        )
    if deformable_body_count != 8:
        raise Plan083CpuScenePacketError(
            f"expected 8 deformable bodies across external CCD scenes, got {deformable_body_count}"
        )
    if deformable_node_count < 18:
        raise Plan083CpuScenePacketError(
            "expected at least 18 deformable nodes across external CCD scenes"
        )
    if surface_triangle_count < 4:
        raise Plan083CpuScenePacketError(
            "expected at least 4 deformable surface triangles across external CCD scenes"
        )
    if rigid_obstacle_count != 4:
        raise Plan083CpuScenePacketError(
            f"expected 4 rigid surface CCD obstacles, got {rigid_obstacle_count}"
        )
    if static_rigid_obstacle_count != 2 or moving_rigid_obstacle_count != 2:
        raise Plan083CpuScenePacketError(
            "expected two static-rigid and two moving-rigid surface CCD obstacles"
        )

    counters = _surface_contact_runtime_counters(row)
    for key in EXTERNAL_SURFACE_CCD_REQUIRED_COUNTER_KEYS:
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                f"external surface CCD diagnostic row requires positive {key}"
            )
    mixed_counters = {
        key: int(_finite_number(row, key))
        for key in MIXED_EXTERNAL_SURFACE_CCD_REQUIRED_COUNTER_KEYS
    }
    if mixed_counters["mixed_external_surface_ccd_scene_count"] != 1:
        raise Plan083CpuScenePacketError(
            "external surface CCD diagnostic row requires one mixed scene"
        )
    if mixed_counters["mixed_external_surface_ccd_family_count"] != 3:
        raise Plan083CpuScenePacketError(
            "external surface CCD mixed scene must exercise all three external families"
        )
    for key, value in mixed_counters.items():
        if value <= 0:
            raise Plan083CpuScenePacketError(
                f"external surface CCD mixed scene requires positive {key}"
            )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-alg-barriers",
            "scene_id": "plan083_external_surface_ccd",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "deformable IPC World::step external surface CCD diagnostics",
            "step_count": scene_count,
            "wall_time_ns": timing_ns,
            "external_surface_ccd_scene_count": scene_count,
            "rigid_obstacle_count": rigid_obstacle_count,
            "static_rigid_obstacle_count": static_rigid_obstacle_count,
            "moving_rigid_obstacle_count": moving_rigid_obstacle_count,
            "deformable_body_count": deformable_body_count,
            "deformable_node_count": deformable_node_count,
            "deformable_edge_count": deformable_edge_count,
            "surface_triangle_count": surface_triangle_count,
            **counters,
            **mixed_counters,
            "limitation_status": (
                "Reduced external surface CCD diagnostic packet only; the packet "
                "includes one mixed World::step scene plus isolated witnesses, "
                "but paper-scale external contact, production runtime scene "
                "filtering, GPU World::step, analytic curved CCD, and speedup "
                "evidence remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_abd_house_of_cards_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    affine_body_count = int(_finite_number(row, "affine_body_count"))
    static_triangle_body_count = int(_finite_number(row, "static_triangle_body_count"))
    point_triangle_pair_count = int(_finite_number(row, "point_triangle_pair_count"))
    valid_step_count = int(_finite_number(row, "valid_step_count"))
    failed_steps = _finite_number(row, "failed_steps")
    converged_solve_count = int(_finite_number(row, "converged_solve_count"))
    barrier_active_count = int(_finite_number(row, "barrier_active_count"))
    solver_iterations = int(_finite_number(row, "solver_iterations"))
    total_objective_decrease = _finite_number(row, "total_objective_decrease")
    max_final_gradient_norm = _finite_number(row, "max_final_gradient_norm")
    min_target_squared_distance = _finite_number(row, "min_target_squared_distance")
    min_final_squared_distance = _finite_number(row, "min_final_squared_distance")
    squared_activation_distance = _finite_number(row, "squared_activation_distance")
    max_linear_speed = _finite_number(row, "max_linear_speed_m_s")
    max_affine_velocity_norm = _finite_number(row, "max_affine_velocity_norm")
    max_displacement_norm = _finite_number(row, "max_displacement_norm_m")

    if affine_body_count < 4:
        raise Plan083CpuScenePacketError(
            f"expected at least 4 reduced ABD cards, got {affine_body_count}"
        )
    if static_triangle_body_count != 1:
        raise Plan083CpuScenePacketError(
            "expected one static triangle support for reduced ABD cards"
        )
    if point_triangle_pair_count != affine_body_count:
        raise Plan083CpuScenePacketError(
            "expected one point-triangle runtime solve per reduced ABD card"
        )
    if failed_steps != 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced ABD cards reported {failed_steps:g} failed steps"
        )
    if valid_step_count != affine_body_count:
        raise Plan083CpuScenePacketError(
            f"expected {affine_body_count} valid ABD steps, got {valid_step_count}"
        )
    if converged_solve_count != affine_body_count:
        raise Plan083CpuScenePacketError(
            f"expected {affine_body_count} converged ABD solves, got {converged_solve_count}"
        )
    if barrier_active_count != affine_body_count:
        raise Plan083CpuScenePacketError(
            f"expected {affine_body_count} active ABD barrier rows, got {barrier_active_count}"
        )
    if solver_iterations <= 0:
        raise Plan083CpuScenePacketError("reduced ABD cards need solver iterations")
    if total_objective_decrease <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD cards need positive objective decrease"
        )
    if max_final_gradient_norm > 1e-5:
        raise Plan083CpuScenePacketError(
            f"reduced ABD cards final gradient is too large: {max_final_gradient_norm:.3g}"
        )
    if min_final_squared_distance <= min_target_squared_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD cards must move farther from contact than the inertial target"
        )
    if min_final_squared_distance >= squared_activation_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD cards should remain inside the activation distance"
        )
    if max_linear_speed <= 0.0 or max_affine_velocity_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD cards need linear and affine velocity updates"
        )
    if max_displacement_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD cards need a nonzero runtime displacement"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "abd-vs-rigid-cards",
            "scene_id": "plan083_abd_house_of_cards",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "detail affine point-triangle runtime step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "affine_body_count": affine_body_count,
            "static_triangle_body_count": static_triangle_body_count,
            "point_triangle_pair_count": point_triangle_pair_count,
            "valid_step_count": valid_step_count,
            "converged_solve_count": converged_solve_count,
            "barrier_active_count": barrier_active_count,
            "solver_iterations": solver_iterations,
            "total_objective_decrease": total_objective_decrease,
            "max_final_gradient_norm": max_final_gradient_norm,
            "min_target_squared_distance": min_target_squared_distance,
            "min_final_squared_distance": min_final_squared_distance,
            "squared_activation_distance": squared_activation_distance,
            "max_linear_speed_m_s": max_linear_speed,
            "max_affine_velocity_norm": max_affine_velocity_norm,
            "max_displacement_norm_m": max_displacement_norm,
            "limitation_status": (
                "Reduced affine point-triangle runtime-step packet only; "
                "card-stack assets, rigid IPC comparison timing, and paper-scale "
                "house-of-cards reproduction remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_abd_wrecking_ball_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    affine_body_count = int(_finite_number(row, "affine_body_count"))
    dynamic_pair_count = int(_finite_number(row, "dynamic_pair_count"))
    valid_step_count = int(_finite_number(row, "valid_step_count"))
    failed_steps = _finite_number(row, "failed_steps")
    converged_solve_count = int(_finite_number(row, "converged_solve_count"))
    barrier_active_count = int(_finite_number(row, "barrier_active_count"))
    solver_iterations = int(_finite_number(row, "solver_iterations"))
    total_objective_decrease = _finite_number(row, "total_objective_decrease")
    max_final_gradient_norm = _finite_number(row, "max_final_gradient_norm")
    min_target_squared_distance = _finite_number(row, "min_target_squared_distance")
    min_final_squared_distance = _finite_number(row, "min_final_squared_distance")
    squared_activation_distance = _finite_number(row, "squared_activation_distance")
    max_linear_speed = _finite_number(row, "max_linear_speed_m_s")
    max_affine_velocity_norm = _finite_number(row, "max_affine_velocity_norm")
    max_displacement_norm = _finite_number(row, "max_displacement_norm_m")

    if affine_body_count != 2:
        raise Plan083CpuScenePacketError(
            f"expected 2 reduced ABD wrecking-ball bodies, got {affine_body_count}"
        )
    if dynamic_pair_count != 1:
        raise Plan083CpuScenePacketError(
            "expected one dynamic point-triangle pair for reduced ABD wrecking ball"
        )
    if failed_steps != 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced ABD wrecking ball reported {failed_steps:g} failed steps"
        )
    if valid_step_count != dynamic_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {dynamic_pair_count} valid ABD pair steps, got {valid_step_count}"
        )
    if converged_solve_count != dynamic_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {dynamic_pair_count} converged ABD pair solves, got {converged_solve_count}"
        )
    if barrier_active_count != dynamic_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {dynamic_pair_count} active ABD pair barriers, got {barrier_active_count}"
        )
    if solver_iterations <= 0:
        raise Plan083CpuScenePacketError(
            "reduced ABD wrecking ball needs solver iterations"
        )
    if total_objective_decrease <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD wrecking ball needs positive objective decrease"
        )
    if max_final_gradient_norm > 1e-5:
        raise Plan083CpuScenePacketError(
            "reduced ABD wrecking ball final gradient is too large: "
            f"{max_final_gradient_norm:.3g}"
        )
    if min_final_squared_distance <= min_target_squared_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD wrecking ball must move farther from contact than the inertial target"
        )
    if min_final_squared_distance >= squared_activation_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD wrecking ball should remain inside the activation distance"
        )
    if max_linear_speed <= 0.0 or max_affine_velocity_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD wrecking ball needs linear and affine velocity updates"
        )
    if max_displacement_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD wrecking ball needs a nonzero runtime displacement"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "abd-vs-rigid-wreck",
            "scene_id": "plan083_abd_wrecking_ball",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "detail affine point-triangle pair runtime step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "affine_body_count": affine_body_count,
            "dynamic_pair_count": dynamic_pair_count,
            "valid_step_count": valid_step_count,
            "converged_solve_count": converged_solve_count,
            "barrier_active_count": barrier_active_count,
            "solver_iterations": solver_iterations,
            "total_objective_decrease": total_objective_decrease,
            "max_final_gradient_norm": max_final_gradient_norm,
            "min_target_squared_distance": min_target_squared_distance,
            "min_final_squared_distance": min_final_squared_distance,
            "squared_activation_distance": squared_activation_distance,
            "max_linear_speed_m_s": max_linear_speed,
            "max_affine_velocity_norm": max_affine_velocity_norm,
            "max_displacement_norm_m": max_displacement_norm,
            "limitation_status": (
                "Reduced two-body affine point-triangle runtime-step packet only; "
                "14k-triangle wrecking-ball assets, rigid IPC comparison timing, "
                "and paper-scale reproduction remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_abd_chain_packet(
    scene: str,
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    config = ABD_CHAIN_SCENES[scene]
    expected_pair_count = int(config["pair_count"])
    affine_body_count = int(_finite_number(row, "affine_body_count"))
    dynamic_pair_count = int(_finite_number(row, "dynamic_pair_count"))
    valid_step_count = int(_finite_number(row, "valid_step_count"))
    failed_steps = _finite_number(row, "failed_steps")
    converged_solve_count = int(_finite_number(row, "converged_solve_count"))
    barrier_active_count = int(_finite_number(row, "barrier_active_count"))
    solver_iterations = int(_finite_number(row, "solver_iterations"))
    total_objective_decrease = _finite_number(row, "total_objective_decrease")
    max_final_gradient_norm = _finite_number(row, "max_final_gradient_norm")
    min_target_squared_distance = _finite_number(row, "min_target_squared_distance")
    min_final_squared_distance = _finite_number(row, "min_final_squared_distance")
    squared_activation_distance = _finite_number(row, "squared_activation_distance")
    max_linear_speed = _finite_number(row, "max_linear_speed_m_s")
    max_affine_velocity_norm = _finite_number(row, "max_affine_velocity_norm")
    max_displacement_norm = _finite_number(row, "max_displacement_norm_m")

    expected_affine_body_count = 2 * expected_pair_count
    if affine_body_count != expected_affine_body_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_affine_body_count} reduced ABD chain bodies, "
            f"got {affine_body_count}"
        )
    if dynamic_pair_count != expected_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} dynamic ABD chain pairs, "
            f"got {dynamic_pair_count}"
        )
    if failed_steps != 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced ABD chain packet reported {failed_steps:g} failed steps"
        )
    if valid_step_count != expected_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} valid ABD chain steps, got {valid_step_count}"
        )
    if converged_solve_count != expected_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} converged ABD chain solves, "
            f"got {converged_solve_count}"
        )
    if barrier_active_count != expected_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} active ABD chain barriers, "
            f"got {barrier_active_count}"
        )
    if solver_iterations <= 0:
        raise Plan083CpuScenePacketError("reduced ABD chain needs solver iterations")
    if total_objective_decrease <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD chain needs positive objective decrease"
        )
    if max_final_gradient_norm > 1e-5:
        raise Plan083CpuScenePacketError(
            "reduced ABD chain final gradient is too large: "
            f"{max_final_gradient_norm:.3g}"
        )
    if min_final_squared_distance <= min_target_squared_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD chain must move farther from contact than the inertial target"
        )
    if min_final_squared_distance >= squared_activation_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD chain should remain inside the activation distance"
        )
    if max_linear_speed <= 0.0 or max_affine_velocity_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD chain needs linear and affine velocity updates"
        )
    if max_displacement_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD chain needs a nonzero runtime displacement"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": config["row_id"],
            "scene_id": config["scene_id"],
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "detail affine point-triangle pair runtime step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "affine_body_count": affine_body_count,
            "dynamic_pair_count": dynamic_pair_count,
            "valid_step_count": valid_step_count,
            "converged_solve_count": converged_solve_count,
            "barrier_active_count": barrier_active_count,
            "solver_iterations": solver_iterations,
            "total_objective_decrease": total_objective_decrease,
            "max_final_gradient_norm": max_final_gradient_norm,
            "min_target_squared_distance": min_target_squared_distance,
            "min_final_squared_distance": min_final_squared_distance,
            "squared_activation_distance": squared_activation_distance,
            "max_linear_speed_m_s": max_linear_speed,
            "max_affine_velocity_norm": max_affine_velocity_norm,
            "max_displacement_norm_m": max_displacement_norm,
            "limitation_status": (
                f"Reduced {expected_pair_count}-pair affine chain-net "
                f"runtime-step packet only; {config['paper_gap']}"
            ),
        },
        "benchmarks": rows,
    }


def _make_abd_comparison_packet(
    scene: str,
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    config = ABD_COMPARISON_SCENES[scene]
    expected_pair_count = int(config["pair_count"])
    expected_paper_body_count = int(config["paper_body_count"])
    expected_paper_triangle_count = int(config["paper_triangle_count"])
    affine_body_count = int(_finite_number(row, "affine_body_count"))
    dynamic_pair_count = int(_finite_number(row, "dynamic_pair_count"))
    reduced_pair_count = int(_finite_number(row, "reduced_pair_count"))
    paper_body_count = int(_finite_number(row, "paper_body_count"))
    paper_triangle_count = int(_finite_number(row, "paper_triangle_count"))
    reference_baseline_measured = _finite_number(row, "reference_baseline_measured")
    valid_step_count = int(_finite_number(row, "valid_step_count"))
    failed_steps = _finite_number(row, "failed_steps")
    converged_solve_count = int(_finite_number(row, "converged_solve_count"))
    barrier_active_count = int(_finite_number(row, "barrier_active_count"))
    solver_iterations = int(_finite_number(row, "solver_iterations"))
    total_objective_decrease = _finite_number(row, "total_objective_decrease")
    max_final_gradient_norm = _finite_number(row, "max_final_gradient_norm")
    min_target_squared_distance = _finite_number(row, "min_target_squared_distance")
    min_final_squared_distance = _finite_number(row, "min_final_squared_distance")
    squared_activation_distance = _finite_number(row, "squared_activation_distance")
    max_linear_speed = _finite_number(row, "max_linear_speed_m_s")
    max_affine_velocity_norm = _finite_number(row, "max_affine_velocity_norm")
    max_displacement_norm = _finite_number(row, "max_displacement_norm_m")

    expected_affine_body_count = 2 * expected_pair_count
    if affine_body_count != expected_affine_body_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_affine_body_count} reduced ABD comparison bodies, "
            f"got {affine_body_count}"
        )
    if (
        dynamic_pair_count != expected_pair_count
        or reduced_pair_count != expected_pair_count
    ):
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} dynamic ABD comparison pairs, "
            f"got dynamic={dynamic_pair_count}, reduced={reduced_pair_count}"
        )
    if paper_body_count != expected_paper_body_count:
        raise Plan083CpuScenePacketError(
            f"expected paper body count {expected_paper_body_count}, got {paper_body_count}"
        )
    if paper_triangle_count != expected_paper_triangle_count:
        raise Plan083CpuScenePacketError(
            "expected paper triangle count "
            f"{expected_paper_triangle_count}, got {paper_triangle_count}"
        )
    if reference_baseline_measured != 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison packet must not claim a measured reference baseline"
        )
    if failed_steps != 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced ABD comparison packet reported {failed_steps:g} failed steps"
        )
    if valid_step_count != expected_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} valid ABD comparison steps, "
            f"got {valid_step_count}"
        )
    if converged_solve_count != expected_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} converged ABD comparison solves, "
            f"got {converged_solve_count}"
        )
    if barrier_active_count != expected_pair_count:
        raise Plan083CpuScenePacketError(
            f"expected {expected_pair_count} active ABD comparison barriers, "
            f"got {barrier_active_count}"
        )
    if solver_iterations <= 0:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison packet needs solver iterations"
        )
    if total_objective_decrease <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison packet needs positive objective decrease"
        )
    if max_final_gradient_norm > 1e-5:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison final gradient is too large: "
            f"{max_final_gradient_norm:.3g}"
        )
    if min_final_squared_distance <= min_target_squared_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison must move farther from contact than the inertial target"
        )
    if min_final_squared_distance >= squared_activation_distance:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison should remain inside the activation distance"
        )
    if max_linear_speed <= 0.0 or max_affine_velocity_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison needs linear and affine velocity updates"
        )
    if max_displacement_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD comparison needs a nonzero runtime displacement"
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": config["row_id"],
            "scene_id": config["scene_id"],
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "detail affine point-triangle pair runtime step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "affine_body_count": affine_body_count,
            "dynamic_pair_count": dynamic_pair_count,
            "reduced_pair_count": reduced_pair_count,
            "paper_body_count": paper_body_count,
            "paper_triangle_count": paper_triangle_count,
            "reference_baseline_measured": False,
            "valid_step_count": valid_step_count,
            "converged_solve_count": converged_solve_count,
            "barrier_active_count": barrier_active_count,
            "solver_iterations": solver_iterations,
            "total_objective_decrease": total_objective_decrease,
            "max_final_gradient_norm": max_final_gradient_norm,
            "min_target_squared_distance": min_target_squared_distance,
            "min_final_squared_distance": min_final_squared_distance,
            "squared_activation_distance": squared_activation_distance,
            "max_linear_speed_m_s": max_linear_speed,
            "max_affine_velocity_norm": max_affine_velocity_norm,
            "max_displacement_norm_m": max_displacement_norm,
            "limitation_status": (
                f"Reduced {expected_pair_count}-pair ABD comparison "
                f"runtime-step packet only; {config['paper_gap']}"
            ),
            "reference_baseline_status": config["reference_gap"],
        },
        "benchmarks": rows,
    }


def _make_abd_fem_coupling_packet(
    row: Mapping[str, Any],
    rows: list[Any],
    *,
    timing_ns: float,
) -> dict[str, Any]:
    packet = _make_abd_comparison_packet(
        "abd_fem_coupling", row, rows, timing_ns=timing_ns
    )
    packet_row = packet["plan083_cpu_scene_packet"]

    deformable_body_count = int(_finite_number(row, "deformable_body_count"))
    deformable_node_count = int(_finite_number(row, "deformable_node_count"))
    deformable_edge_count = int(_finite_number(row, "deformable_edge_count"))
    surface_triangle_count = int(_finite_number(row, "surface_triangle_count"))
    deformable_solver_iterations = int(
        _finite_number(row, "deformable_solver_iterations")
    )
    min_cloth_height = _finite_number(row, "min_cloth_height_m")
    candidate_diagnostics_measured = _finite_number(
        row, "affine_fem_candidate_diagnostics_measured"
    )
    mixed_candidate_count = int(_finite_number(row, "affine_fem_mixed_candidate_count"))
    mixed_active_barrier_count = int(
        _finite_number(row, "affine_fem_mixed_active_barrier_count")
    )
    mixed_min_squared_distance = _finite_number(
        row, "affine_fem_mixed_min_squared_distance"
    )
    mixed_barrier_value = _finite_number(row, "affine_fem_mixed_barrier_value")
    coupled_contact_measured = _finite_number(
        row, "affine_fem_coupled_contact_measured"
    )
    coupled_solve_converged = _finite_number(row, "affine_fem_coupled_solve_converged")
    coupled_objective_decrease = _finite_number(
        row, "affine_fem_coupled_objective_decrease"
    )
    coupled_initial_gradient_norm = _finite_number(
        row, "affine_fem_coupled_initial_gradient_norm"
    )
    coupled_final_gradient_norm = _finite_number(
        row, "affine_fem_coupled_final_gradient_norm"
    )
    coupled_affine_displacement_norm = _finite_number(
        row, "affine_fem_coupled_affine_displacement_norm"
    )
    coupled_deformable_displacement_norm = _finite_number(
        row, "affine_fem_coupled_deformable_displacement_norm"
    )

    if deformable_body_count != 3:
        raise Plan083CpuScenePacketError(
            "expected reduced FEM cloth plus two inter-body witness "
            f"deformable bodies, got {deformable_body_count}"
        )
    if deformable_node_count != 31:
        raise Plan083CpuScenePacketError(
            "expected 24 reduced FEM cloth nodes plus 7 inter-body witness "
            f"nodes, got {deformable_node_count}"
        )
    if deformable_edge_count < 68:
        raise Plan083CpuScenePacketError(
            "expected structural and shear spring edges for reduced FEM cloth"
        )
    if surface_triangle_count != 30:
        raise Plan083CpuScenePacketError(
            f"expected 30 reduced FEM surface triangles, got {surface_triangle_count}"
        )
    if deformable_solver_iterations < 0:
        raise Plan083CpuScenePacketError(
            "reduced FEM sidecar reported invalid solver iterations"
        )
    if min_cloth_height <= 0.0:
        raise Plan083CpuScenePacketError(
            f"reduced FEM cloth penetrated below z=0: {min_cloth_height:.3g}"
        )
    counters = _surface_contact_runtime_counters(row)
    for key in EXTERNAL_SURFACE_CCD_REQUIRED_COUNTER_KEYS:
        if counters[key] <= 0:
            raise Plan083CpuScenePacketError(
                "reduced ABD/FEM packet needs positive external surface CCD "
                f"witness counter {key}"
            )
    if candidate_diagnostics_measured != 1.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM packet must measure mixed candidate diagnostics"
        )
    if mixed_candidate_count <= 0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM packet needs mixed candidate rows"
        )
    if mixed_active_barrier_count <= 0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM packet needs active mixed barrier rows"
        )
    if mixed_min_squared_distance < 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM mixed distance must be non-negative"
        )
    if mixed_barrier_value <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM mixed barrier value must be positive"
        )
    if coupled_contact_measured != 1.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM packet must measure coupled affine/FEM contact"
        )
    if coupled_solve_converged != 1.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM coupled micro-solve must converge"
        )
    if coupled_objective_decrease <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM coupled micro-solve must decrease the objective"
        )
    if coupled_final_gradient_norm > coupled_initial_gradient_norm:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM coupled micro-solve must reduce gradient norm"
        )
    if coupled_affine_displacement_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM coupled micro-solve must move the affine state"
        )
    if coupled_deformable_displacement_norm <= 0.0:
        raise Plan083CpuScenePacketError(
            "reduced ABD/FEM coupled micro-solve must move deformable vertices"
        )

    packet_row.update(
        {
            "runtime_path": (
                "detail affine point-triangle runtime step plus deformable IPC "
                "World::step external surface CCD sidecar"
            ),
            "deformable_body_count": deformable_body_count,
            "deformable_node_count": deformable_node_count,
            "deformable_edge_count": deformable_edge_count,
            "surface_triangle_count": surface_triangle_count,
            "deformable_solver_iterations": deformable_solver_iterations,
            **counters,
            "min_cloth_height_m": min_cloth_height,
            "affine_fem_candidate_diagnostics_measured": True,
            "affine_fem_mixed_candidate_count": mixed_candidate_count,
            "affine_fem_mixed_active_barrier_count": mixed_active_barrier_count,
            "affine_fem_mixed_min_squared_distance": mixed_min_squared_distance,
            "affine_fem_mixed_barrier_value": mixed_barrier_value,
            "affine_fem_coupled_contact_measured": True,
            "affine_fem_coupled_solve_converged": True,
            "affine_fem_coupled_objective_decrease": coupled_objective_decrease,
            "affine_fem_coupled_initial_gradient_norm": coupled_initial_gradient_norm,
            "affine_fem_coupled_final_gradient_norm": coupled_final_gradient_norm,
            "affine_fem_coupled_affine_displacement_norm": coupled_affine_displacement_norm,
            "affine_fem_coupled_deformable_displacement_norm": coupled_deformable_displacement_norm,
            "limitation_status": (
                "Reduced affine runtime-step, deformable IPC smoke, and "
                "affine/deformable coupled contact micro-solve evidence with "
                "a reduced external surface CCD witness sidecar only; full "
                "runtime affine/FEM coupling, 1.1M-triangle assets, and "
                "paper-scale reproduction remain planned."
            ),
        }
    )
    return packet


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
    sidecar_payload = _external_surface_ccd_sidecar_payload(
        row, scene_label="hanging-bridge"
    )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-02",
            "scene_id": "plan083_hanging_bridge",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": (
                "rigid IPC World::step plus deformable IPC World::step "
                "external surface CCD sidecar"
            ),
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
            **sidecar_payload,
            "limitation_status": (
                "Reduced rigid runtime smoke packet plus external surface CCD "
                "sidecar only; paper-scale rods, codimensional deformables, "
                "bridge-owned deformable contact, and full Table 2 counts "
                "remain planned."
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


def _make_pulley_packet(
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
    revolute_joint_count = int(_finite_number(row, "revolute_joint_count"))
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 4:
        raise Plan083CpuScenePacketError(f"expected 4 rigid bodies, got {body_count}")
    if dynamic_body_count != 3:
        raise Plan083CpuScenePacketError(
            f"expected 3 dynamic rigid bodies, got {dynamic_body_count}"
        )
    if fixed_joint_count != 2:
        raise Plan083CpuScenePacketError(
            f"expected 2 point-connection joints, got {fixed_joint_count}"
        )
    if revolute_joint_count != 1:
        raise Plan083CpuScenePacketError(
            f"expected 1 revolute joint, got {revolute_joint_count}"
        )
    if active_articulation_constraints < 8:
        raise Plan083CpuScenePacketError(
            "expected pulley hinge and point-connection articulation rows"
        )

    separation = _finite_number(row, "load_separation_m")
    if separation <= 0.0:
        raise Plan083CpuScenePacketError("pulley load separation is not positive")

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-03",
            "scene_id": "plan083_pulley_system",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "fixed_joint_count": fixed_joint_count,
            "revolute_joint_count": revolute_joint_count,
            "active_articulation_constraints": active_articulation_constraints,
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "left_load_height_m": _finite_number(row, "left_load_height_m"),
            "right_load_height_m": _finite_number(row, "right_load_height_m"),
            "load_height_difference_m": _finite_number(row, "load_height_difference_m"),
            "load_separation_m": separation,
            "wheel_spin_rad_s": _finite_number(row, "wheel_spin_rad_s"),
            "limitation_status": (
                "Reduced hinged-wheel and point-connection smoke packet only; "
                "analytical force comparison, rope/rod coupling, and paper-scale "
                "timing remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_umbrella_packet(
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
    revolute_joint_count = int(_finite_number(row, "revolute_joint_count"))
    active_articulation_constraints = int(
        _finite_number(row, "active_articulation_constraints")
    )
    if body_count != 4:
        raise Plan083CpuScenePacketError(f"expected 4 rigid bodies, got {body_count}")
    if dynamic_body_count != 3:
        raise Plan083CpuScenePacketError(
            f"expected 3 dynamic rigid bodies, got {dynamic_body_count}"
        )
    if fixed_joint_count != 2:
        raise Plan083CpuScenePacketError(
            f"expected 2 point-connected ribs, got {fixed_joint_count}"
        )
    if revolute_joint_count != 1:
        raise Plan083CpuScenePacketError(
            f"expected 1 umbrella hinge, got {revolute_joint_count}"
        )
    if active_articulation_constraints < 8:
        raise Plan083CpuScenePacketError(
            "expected umbrella hinge and rib articulation rows"
        )

    canopy_span = _finite_number(row, "canopy_span_m")
    if canopy_span <= 0.0:
        raise Plan083CpuScenePacketError("umbrella canopy span is not positive")
    sidecar_payload = _external_surface_ccd_sidecar_payload(row, scene_label="umbrella")

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-04",
            "scene_id": "plan083_umbrella",
            "benchmark_row": _packet_row_name(row),
            "paper_scale": False,
            "runtime_path": (
                "rigid IPC World::step plus deformable IPC World::step "
                "external surface CCD sidecar"
            ),
            "step_count": 1,
            "wall_time_ns": timing_ns,
            "body_count": body_count,
            "dynamic_body_count": dynamic_body_count,
            "fixed_joint_count": fixed_joint_count,
            "revolute_joint_count": revolute_joint_count,
            "active_articulation_constraints": active_articulation_constraints,
            "final_equality_residual_norm": final_residual,
            "max_equality_residual": max_equality_residual,
            "canopy_span_m": canopy_span,
            "hinge_angular_velocity_rad_s": _finite_number(
                row, "hinge_angular_velocity_rad_s"
            ),
            **sidecar_payload,
            "limitation_status": (
                "Reduced hinged-rib smoke packet plus external surface CCD "
                "sidecar only; cloth shrinking, wrinkling, sliding constraints, "
                "umbrella-owned deformable contact, and paper-scale rod "
                "coupling remain planned."
            ),
        },
        "benchmarks": rows,
    }


def _make_nunchaku_scaling_packet(
    typed_rows: list[Mapping[str, Any]],
    rows: list[Any],
    *,
    max_equality_residual: float,
) -> dict[str, Any]:
    expected_row = SCENE_ROWS["nunchaku_scaling"]
    found: dict[int, Mapping[str, Any]] = {}
    errors: list[str] = []
    for row in typed_rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        prefix = f"{expected_row}/"
        if not canonical.startswith(prefix):
            errors.append(f"unexpected benchmark row: {name}")
            continue
        size_text = canonical[len(prefix) :]
        try:
            size = int(size_text)
        except ValueError:
            errors.append(f"{name}: expected integer nunchaku scaling size")
            continue
        if size not in NUNCHAKU_SCALING_SIZES:
            errors.append(f"{name}: unexpected nunchaku scaling size {size}")
            continue
        if row.get("aggregate_name") == "median":
            found[size] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for size in NUNCHAKU_SCALING_SIZES:
        if size not in found:
            errors.append(f"missing median benchmark row: {expected_row}/{size}")

    if errors:
        raise Plan083CpuScenePacketError("\n".join(errors))

    samples = []
    per_pair_times = []
    for size in NUNCHAKU_SCALING_SIZES:
        row = found[size]
        timing_ns = benchmark_timing_ns(row)
        if not math.isfinite(timing_ns) or timing_ns <= 0.0:
            raise Plan083CpuScenePacketError(
                f"{benchmark_row_name(row)} timing is not positive"
            )
        failed_steps = _finite_number(row, "failed_steps")
        if failed_steps != 0.0:
            raise Plan083CpuScenePacketError(
                f"reduced nunchaku scaling size {size} reported "
                f"{failed_steps:g} failed steps"
            )
        final_residual = _finite_number(row, "final_equality_residual_norm")
        if final_residual > max_equality_residual:
            raise Plan083CpuScenePacketError(
                f"reduced nunchaku scaling size {size} equality residual "
                f"{final_residual:.3g} exceeds {max_equality_residual:.3g}"
            )

        body_count = int(_finite_number(row, "body_count"))
        dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
        revolute_joint_count = int(_finite_number(row, "revolute_joint_count"))
        active_articulation_constraints = int(
            _finite_number(row, "active_articulation_constraints")
        )
        if body_count != 2 * size:
            raise Plan083CpuScenePacketError(
                f"size {size}: expected {2 * size} rigid bodies, got {body_count}"
            )
        if dynamic_body_count != size:
            raise Plan083CpuScenePacketError(
                f"size {size}: expected {size} dynamic handles, got {dynamic_body_count}"
            )
        if revolute_joint_count != size:
            raise Plan083CpuScenePacketError(
                f"size {size}: expected {size} revolute joints, got {revolute_joint_count}"
            )
        if active_articulation_constraints < 2 * size:
            raise Plan083CpuScenePacketError(
                f"size {size}: expected active hinge articulation rows"
            )

        free_axis_velocity = _finite_number(row, "free_axis_angular_velocity_rad_s")
        if abs(free_axis_velocity) <= 1e-9:
            raise Plan083CpuScenePacketError(
                f"size {size}: nunchaku free-axis angular velocity was lost"
            )
        per_pair_time = timing_ns / size
        per_pair_times.append(per_pair_time)
        samples.append(
            {
                "nunchaku_pair_count": size,
                "benchmark_row": _packet_row_name(row),
                "wall_time_ns": timing_ns,
                "time_per_pair_ns": per_pair_time,
                "body_count": body_count,
                "dynamic_body_count": dynamic_body_count,
                "revolute_joint_count": revolute_joint_count,
                "active_articulation_constraints": active_articulation_constraints,
                "solver_iterations": int(_finite_number(row, "solver_iterations")),
                "final_equality_residual_norm": final_residual,
                "max_equality_residual": max_equality_residual,
                "free_axis_angular_velocity_rad_s": free_axis_velocity,
            }
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-25",
            "scene_id": "plan083_nunchaku",
            "paper_scale": False,
            "runtime_path": "rigid IPC World::step",
            "sample_sizes": list(NUNCHAKU_SCALING_SIZES),
            "sample_count": len(samples),
            "min_time_per_pair_ns": min(per_pair_times),
            "max_time_per_pair_ns": max(per_pair_times),
            "limitation_status": (
                "Reduced independent-hinge scaling packet only; cone-twist "
                "ranges, coupled N-by-N contact, and paper-scale linear-growth "
                "claims remain planned."
            ),
            "samples": samples,
        },
        "benchmarks": rows,
    }


def _make_timing_breakdown_packet(
    typed_rows: list[Mapping[str, Any]],
    rows: list[Any],
    *,
    max_equality_residual: float,
) -> dict[str, Any]:
    expected_to_scene = {
        row_name: scene for scene, row_name in TIMING_BREAKDOWN_ROWS.items()
    }
    found: dict[str, Mapping[str, Any]] = {}
    errors: list[str] = []

    for row in typed_rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical not in expected_to_scene:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found[canonical] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for row_name in TIMING_BREAKDOWN_ROWS.values():
        if row_name not in found:
            errors.append(f"missing median benchmark row: {row_name}")

    if errors:
        raise Plan083CpuScenePacketError("\n".join(errors))

    samples = []
    total_wall_time_ns = 0.0
    max_residual = 0.0
    total_bodies = 0
    total_dynamic_bodies = 0
    total_active_constraints = 0
    total_active_friction_constraints = 0
    total_active_articulation_constraints = 0

    for scene in TIMING_BREAKDOWN_SCENES:
        row_name = TIMING_BREAKDOWN_ROWS[scene]
        row = found[row_name]
        timing_ns = benchmark_timing_ns(row)
        if not math.isfinite(timing_ns) or timing_ns <= 0.0:
            raise Plan083CpuScenePacketError(
                f"{benchmark_row_name(row)} timing is not positive"
            )
        failed_steps = _finite_number(row, "failed_steps")
        if failed_steps != 0.0:
            raise Plan083CpuScenePacketError(
                f"reduced timing scene {scene} reported {failed_steps:g} failed steps"
            )
        if scene in {"candy", "lying_flat"}:
            final_residual = 0.0
            body_count = int(_finite_number(row, "rigid_obstacle_count")) + int(
                _finite_number(row, "deformable_body_count")
            )
            dynamic_body_count = int(_finite_number(row, "deformable_body_count"))
            active_constraints = int(_finite_number(row, "active_contact_count"))
            active_friction_constraints = 0
            active_articulation_constraints = 0
            deformable_node_count = int(_finite_number(row, "deformable_node_count"))
        else:
            final_residual = _finite_number(row, "final_equality_residual_norm")
            if final_residual > max_equality_residual:
                raise Plan083CpuScenePacketError(
                    f"reduced timing scene {scene} equality residual "
                    f"{final_residual:.3g} exceeds {max_equality_residual:.3g}"
                )
            body_count = int(_finite_number(row, "body_count"))
            dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
            active_constraints = int(_optional_finite_number(row, "active_constraints"))
            active_friction_constraints = int(
                _optional_finite_number(row, "active_friction_constraints")
            )
            active_articulation_constraints = int(
                _optional_finite_number(row, "active_articulation_constraints")
            )
            deformable_node_count = 0

        total_wall_time_ns += timing_ns
        max_residual = max(max_residual, final_residual)
        total_bodies += body_count
        total_dynamic_bodies += dynamic_body_count
        total_active_constraints += active_constraints
        total_active_friction_constraints += active_friction_constraints
        total_active_articulation_constraints += active_articulation_constraints
        samples.append(
            {
                "row_id": SCENE_ROW_IDS[scene],
                "scene_id": SCENE_IDS[scene],
                "benchmark_row": _packet_row_name(row),
                "wall_time_ns": timing_ns,
                "body_count": body_count,
                "dynamic_body_count": dynamic_body_count,
                "active_constraints": active_constraints,
                "active_friction_constraints": active_friction_constraints,
                "active_articulation_constraints": active_articulation_constraints,
                "solver_iterations": int(_finite_number(row, "solver_iterations")),
                "final_equality_residual_norm": final_residual,
                "deformable_node_count": deformable_node_count,
            }
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-fig-24",
            "scene_id": SCENE_IDS["timing_breakdown"],
            "paper_scale": False,
            "runtime_path": "World::step reduced corpus rows",
            "scene_count": len(samples),
            "sample_scene_ids": [sample["scene_id"] for sample in samples],
            "total_wall_time_ns": total_wall_time_ns,
            "max_scene_wall_time_ns": max(sample["wall_time_ns"] for sample in samples),
            "total_body_count": total_bodies,
            "total_dynamic_body_count": total_dynamic_bodies,
            "total_active_constraints": total_active_constraints,
            "total_active_friction_constraints": total_active_friction_constraints,
            "total_active_articulation_constraints": (
                total_active_articulation_constraints
            ),
            "max_final_equality_residual_norm": max_residual,
            "max_equality_residual": max_equality_residual,
            "available_timing_fields": ["wall_time_ns"],
            "missing_paper_timing_fields": [
                "gradient",
                "hessian",
                "contact_pairs",
                "ccd",
                "linear_solve",
            ],
            "limitation_status": (
                "Reduced aggregate wall-time packet only; paper gradient, "
                "Hessian, contact-pair, CCD, linear-solve, CPU/GPU parity, "
                "and paper-scale scene timing breakdowns remain planned."
            ),
            "samples": samples,
        },
        "benchmarks": rows,
    }


def _make_table2_packet(
    typed_rows: list[Mapping[str, Any]],
    rows: list[Any],
    *,
    max_equality_residual: float,
) -> dict[str, Any]:
    expected_to_scene = {
        row_name: scene for scene, row_name in TABLE2_REDUCED_ROWS.items()
    }
    found: dict[str, Mapping[str, Any]] = {}
    errors: list[str] = []

    for row in typed_rows:
        name = benchmark_row_name(row)
        if not name:
            errors.append("benchmark row is missing a name")
            continue
        canonical = _packet_row_name(row)
        if canonical not in expected_to_scene:
            errors.append(f"unexpected benchmark row: {name}")
            continue
        if row.get("aggregate_name") == "median":
            found[canonical] = row
        errors.extend(benchmark_timing_field_errors(row, name))

    for row_name in TABLE2_REDUCED_ROWS.values():
        if row_name not in found:
            errors.append(f"missing median benchmark row: {row_name}")

    if errors:
        raise Plan083CpuScenePacketError("\n".join(errors))

    samples = []
    total_wall_time_ns = 0.0
    total_bodies = 0
    total_dynamic_bodies = 0
    max_active_constraints = 0
    max_solver_iterations = 0
    max_residual = 0.0

    for scene in TABLE2_REDUCED_SCENES:
        row_name = TABLE2_REDUCED_ROWS[scene]
        row = found[row_name]
        timing_ns = benchmark_timing_ns(row)
        if not math.isfinite(timing_ns) or timing_ns <= 0.0:
            raise Plan083CpuScenePacketError(
                f"{benchmark_row_name(row)} timing is not positive"
            )
        failed_steps = _finite_number(row, "failed_steps")
        if failed_steps != 0.0:
            raise Plan083CpuScenePacketError(
                f"reduced Table 2 scene {scene} reported {failed_steps:g} failed steps"
            )
        if scene in {"candy", "lying_flat"}:
            final_residual = 0.0
            body_count = int(_finite_number(row, "rigid_obstacle_count")) + int(
                _finite_number(row, "deformable_body_count")
            )
            dynamic_body_count = int(_finite_number(row, "deformable_body_count"))
            active_constraints = int(_finite_number(row, "active_contact_count"))
            active_friction_constraints = 0
            active_articulation_constraints = 0
            deformable_node_count = int(_finite_number(row, "deformable_node_count"))
        else:
            final_residual = _finite_number(row, "final_equality_residual_norm")
            if final_residual > max_equality_residual:
                raise Plan083CpuScenePacketError(
                    f"reduced Table 2 scene {scene} equality residual "
                    f"{final_residual:.3g} exceeds {max_equality_residual:.3g}"
                )
            body_count = int(_finite_number(row, "body_count"))
            dynamic_body_count = int(_finite_number(row, "dynamic_body_count"))
            active_constraints = int(_optional_finite_number(row, "active_constraints"))
            active_friction_constraints = int(
                _optional_finite_number(row, "active_friction_constraints")
            )
            active_articulation_constraints = int(
                _optional_finite_number(row, "active_articulation_constraints")
            )
            deformable_node_count = 0
        solver_iterations = int(_finite_number(row, "solver_iterations"))

        total_wall_time_ns += timing_ns
        total_bodies += body_count
        total_dynamic_bodies += dynamic_body_count
        max_active_constraints = max(max_active_constraints, active_constraints)
        max_solver_iterations = max(max_solver_iterations, solver_iterations)
        max_residual = max(max_residual, final_residual)
        samples.append(
            {
                "row_id": SCENE_ROW_IDS[scene],
                "scene_id": SCENE_IDS[scene],
                "benchmark_row": _packet_row_name(row),
                "paper_scale": False,
                "wall_time_ns": timing_ns,
                "body_count": body_count,
                "dynamic_body_count": dynamic_body_count,
                "active_constraints": active_constraints,
                "active_friction_constraints": active_friction_constraints,
                "active_articulation_constraints": active_articulation_constraints,
                "solver_iterations": solver_iterations,
                "final_equality_residual_norm": final_residual,
                "deformable_node_count": deformable_node_count,
            }
        )

    return {
        "plan083_cpu_scene_packet": {
            "row_id": "unb-table-02",
            "scene_id": SCENE_IDS["table_2"],
            "paper_scale": False,
            "runtime_path": "World::step reduced Table 2 rows",
            "scene_count": len(samples),
            "covered_paper_rows": [sample["row_id"] for sample in samples],
            "missing_paper_rows": [],
            "total_wall_time_ns": total_wall_time_ns,
            "total_body_count": total_bodies,
            "total_dynamic_body_count": total_dynamic_bodies,
            "max_active_constraints": max_active_constraints,
            "max_solver_iterations": max_solver_iterations,
            "max_final_equality_residual_norm": max_residual,
            "max_equality_residual": max_equality_residual,
            "limitation_status": (
                "Reduced Table 2 setup/statistics packet only; paper "
                "body/node/contact counts, paper timesteps, and paper timing "
                "rows remain planned."
            ),
            "samples": samples,
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
    if "body_count" in row:
        print(
            "PLAN-083 CPU scene packet OK: "
            f"{row['scene_id']} bodies={row['body_count']} "
            f"residual={row['final_equality_residual_norm']:.3g} "
            f"time={row['wall_time_ns']:.3g} ns"
        )
    elif "sample_sizes" in row:
        print(
            "PLAN-083 CPU scene packet OK: "
            f"{row['scene_id']} samples={row['sample_sizes']} "
            f"max_time_per_pair={row['max_time_per_pair_ns']:.3g} ns"
        )
    elif "deformable_body_count" in row:
        print(
            "PLAN-083 CPU scene packet OK: "
            f"{row['scene_id']} deformable_bodies={row['deformable_body_count']} "
            f"nodes={row['deformable_node_count']} "
            f"time={row['wall_time_ns']:.3g} ns"
        )
    elif "affine_body_count" in row:
        print(
            "PLAN-083 CPU scene packet OK: "
            f"{row['scene_id']} affine_bodies={row['affine_body_count']} "
            f"time={row['wall_time_ns']:.3g} ns"
        )
    else:
        print(
            "PLAN-083 CPU scene packet OK: "
            f"{row['scene_id']} scenes={row['scene_count']} "
            f"total_time={row['total_wall_time_ns']:.3g} ns"
        )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
