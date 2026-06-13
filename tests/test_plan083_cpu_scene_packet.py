import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_cpu_scene_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_cpu_scene_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _benchmark_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_hanging_bridge_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_hanging_bridge_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 2.0,
        "cpu_time": 2.0,
        "time_unit": "ms",
        "body_count": 7,
        "dynamic_body_count": 5,
        "fixed_joint_count": 4,
        "active_articulation_constraints": 12,
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "traveler_height_m": 0.81,
        "max_board_sag_m": 0.01,
        "external_surface_ccd_sidecar_scene_count": 1,
        "deformable_sidecar_body_count": 3,
        "deformable_sidecar_node_count": 31,
        "deformable_sidecar_edge_count": 68,
        "deformable_sidecar_surface_triangle_count": 32,
        **_external_surface_ccd_sidecar_counters(),
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _surface_contact_counters() -> dict[str, object]:
    return {
        "line_search_trials": 6,
        "surface_contact_candidate_builds": 6,
        "surface_contact_point_triangle_candidates": 4,
        "surface_contact_edge_edge_candidates": 2,
        "surface_contact_ccd_point_triangle_checks": 4,
        "surface_contact_ccd_edge_edge_checks": 2,
        "surface_contact_ccd_hits": 1,
        "surface_contact_ccd_limited_steps": 1,
        "surface_contact_ccd_zero_step_count": 0,
        "inter_body_surface_contact_candidate_builds": 0,
        "inter_body_surface_contact_point_triangle_candidates": 0,
        "inter_body_surface_contact_edge_edge_candidates": 0,
        "inter_body_surface_contact_ccd_point_triangle_checks": 0,
        "inter_body_surface_contact_ccd_edge_edge_checks": 0,
        "inter_body_surface_contact_ccd_hits": 0,
        "inter_body_surface_contact_ccd_limited_steps": 0,
        "inter_body_surface_contact_ccd_zero_step_count": 0,
        "static_rigid_surface_ccd_snapshot_builds": 0,
        "static_rigid_surface_ccd_box_count": 0,
        "static_rigid_surface_ccd_sphere_count": 0,
        "static_rigid_surface_ccd_triangle_count": 0,
        "static_rigid_surface_ccd_edge_count": 0,
        "static_rigid_surface_ccd_candidate_builds": 0,
        "static_rigid_surface_ccd_point_triangle_candidates": 0,
        "static_rigid_surface_ccd_edge_edge_candidates": 0,
        "static_rigid_surface_ccd_point_triangle_checks": 0,
        "static_rigid_surface_ccd_edge_edge_checks": 0,
        "static_rigid_surface_ccd_hits": 0,
        "static_rigid_surface_ccd_limited_steps": 0,
        "static_rigid_surface_ccd_zero_step_count": 0,
        "moving_rigid_surface_ccd_snapshot_builds": 0,
        "moving_rigid_surface_ccd_box_count": 0,
        "moving_rigid_surface_ccd_sample_count": 0,
        "moving_rigid_surface_ccd_inflated_box_count": 0,
        "moving_rigid_surface_ccd_triangle_count": 0,
        "moving_rigid_surface_ccd_edge_count": 0,
        "moving_rigid_surface_ccd_candidate_builds": 0,
        "moving_rigid_surface_ccd_point_triangle_candidates": 0,
        "moving_rigid_surface_ccd_edge_edge_candidates": 0,
        "moving_rigid_surface_ccd_point_triangle_checks": 0,
        "moving_rigid_surface_ccd_edge_edge_checks": 0,
        "moving_rigid_surface_ccd_hits": 0,
        "moving_rigid_surface_ccd_limited_steps": 0,
        "moving_rigid_surface_ccd_zero_step_count": 0,
    }


def _external_surface_ccd_sidecar_counters() -> dict[str, object]:
    counters = _surface_contact_counters()
    counters.update(
        {
            "line_search_trials": 68,
            "surface_contact_candidate_builds": 68,
            "surface_contact_point_triangle_candidates": 660,
            "surface_contact_edge_edge_candidates": 1224,
            "surface_contact_ccd_point_triangle_checks": 660,
            "surface_contact_ccd_edge_edge_checks": 1224,
            "surface_contact_ccd_hits": 1,
            "surface_contact_ccd_limited_steps": 0,
            "inter_body_surface_contact_candidate_builds": 67,
            "inter_body_surface_contact_point_triangle_candidates": 33,
            "inter_body_surface_contact_ccd_point_triangle_checks": 33,
            "inter_body_surface_contact_ccd_hits": 33,
            "inter_body_surface_contact_ccd_limited_steps": 1,
            "inter_body_surface_contact_ccd_zero_step_count": 32,
            "static_rigid_surface_ccd_snapshot_builds": 1,
            "static_rigid_surface_ccd_box_count": 1,
            "static_rigid_surface_ccd_triangle_count": 12,
            "static_rigid_surface_ccd_edge_count": 12,
            "static_rigid_surface_ccd_candidate_builds": 35,
            "static_rigid_surface_ccd_point_triangle_candidates": 68,
            "static_rigid_surface_ccd_edge_edge_candidates": 102,
            "static_rigid_surface_ccd_point_triangle_checks": 68,
            "static_rigid_surface_ccd_edge_edge_checks": 102,
            "static_rigid_surface_ccd_hits": 34,
            "static_rigid_surface_ccd_limited_steps": 1,
            "static_rigid_surface_ccd_zero_step_count": 62,
            "moving_rigid_surface_ccd_snapshot_builds": 1,
            "moving_rigid_surface_ccd_box_count": 1,
            "moving_rigid_surface_ccd_sample_count": 10,
            "moving_rigid_surface_ccd_triangle_count": 120,
            "moving_rigid_surface_ccd_edge_count": 120,
            "moving_rigid_surface_ccd_candidate_builds": 3,
            "moving_rigid_surface_ccd_point_triangle_candidates": 2,
            "moving_rigid_surface_ccd_edge_edge_candidates": 41,
            "moving_rigid_surface_ccd_point_triangle_checks": 2,
            "moving_rigid_surface_ccd_edge_edge_checks": 41,
            "moving_rigid_surface_ccd_hits": 1,
            "moving_rigid_surface_ccd_limited_steps": 1,
        }
    )
    return counters


def _abd_fem_surface_contact_counters() -> dict[str, object]:
    return _external_surface_ccd_sidecar_counters()


def _nunchaku_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_nunchaku_single_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_nunchaku_single_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 3.0,
        "cpu_time": 3.0,
        "time_unit": "ms",
        "body_count": 2,
        "dynamic_body_count": 1,
        "revolute_joint_count": 1,
        "active_articulation_constraints": 2,
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "swinging_tip_radius_m": 0.36,
        "free_axis_angular_velocity_rad_s": 1.5,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _pulley_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_pulley_system_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_pulley_system_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 8.0,
        "cpu_time": 8.0,
        "time_unit": "ms",
        "body_count": 4,
        "dynamic_body_count": 3,
        "fixed_joint_count": 2,
        "revolute_joint_count": 1,
        "active_articulation_constraints": 8,
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "left_load_height_m": 0.48,
        "right_load_height_m": 0.42,
        "load_height_difference_m": 0.06,
        "load_separation_m": 0.40,
        "wheel_spin_rad_s": 0.0,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _umbrella_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_umbrella_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_umbrella_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 9.0,
        "cpu_time": 9.0,
        "time_unit": "ms",
        "body_count": 4,
        "dynamic_body_count": 3,
        "fixed_joint_count": 2,
        "revolute_joint_count": 1,
        "active_articulation_constraints": 8,
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "canopy_span_m": 0.36,
        "hinge_angular_velocity_rad_s": 0.9,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _candy_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_candy_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_candy_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 10.0,
        "cpu_time": 10.0,
        "time_unit": "ms",
        "rigid_obstacle_count": 3,
        "deformable_body_count": 3,
        "deformable_node_count": 27,
        "deformable_edge_count": 72,
        "surface_triangle_count": 32,
        "solver_iterations": 8,
        "active_contact_count": 0,
        "friction_dissipation": 1e-6,
        "min_active_contact_distance_m": 0.0,
        "min_cloth_height_m": 0.06,
        "cloth_span_x_m": 0.22,
        "failed_steps": 0,
        **_surface_contact_counters(),
        "line_search_trials": 69,
        "surface_contact_candidate_builds": 3,
        "surface_contact_point_triangle_candidates": 64,
        "surface_contact_edge_edge_candidates": 207,
        "surface_contact_ccd_point_triangle_checks": 64,
        "surface_contact_ccd_edge_edge_checks": 207,
        "surface_contact_ccd_hits": 0,
        "surface_contact_ccd_limited_steps": 0,
        "static_rigid_surface_ccd_snapshot_builds": 1,
        "static_rigid_surface_ccd_box_count": 1,
        "static_rigid_surface_ccd_sphere_count": 0,
        "static_rigid_surface_ccd_triangle_count": 12,
        "static_rigid_surface_ccd_edge_count": 12,
        "static_rigid_surface_ccd_candidate_builds": 69,
        "static_rigid_surface_ccd_point_triangle_candidates": 72,
        "static_rigid_surface_ccd_edge_edge_candidates": 0,
        "static_rigid_surface_ccd_point_triangle_checks": 72,
        "static_rigid_surface_ccd_edge_edge_checks": 0,
        "static_rigid_surface_ccd_hits": 72,
        "static_rigid_surface_ccd_limited_steps": 1,
        "static_rigid_surface_ccd_zero_step_count": 96,
        "moving_rigid_surface_ccd_snapshot_builds": 1,
        "moving_rigid_surface_ccd_box_count": 1,
        "moving_rigid_surface_ccd_sample_count": 6,
        "moving_rigid_surface_ccd_inflated_box_count": 0,
        "moving_rigid_surface_ccd_triangle_count": 72,
        "moving_rigid_surface_ccd_edge_count": 72,
        "moving_rigid_surface_ccd_candidate_builds": 37,
        "moving_rigid_surface_ccd_point_triangle_candidates": 184,
        "moving_rigid_surface_ccd_edge_edge_candidates": 0,
        "moving_rigid_surface_ccd_point_triangle_checks": 184,
        "moving_rigid_surface_ccd_edge_edge_checks": 0,
        "moving_rigid_surface_ccd_hits": 184,
        "moving_rigid_surface_ccd_limited_steps": 1,
        "moving_rigid_surface_ccd_zero_step_count": 64,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _abd_house_cards_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_abd_house_of_cards_reduced_runtime_step_median",
        "run_name": "BM_Plan083CpuScene_abd_house_of_cards_reduced_runtime_step",
        "aggregate_name": "median",
        "real_time": 7.0,
        "cpu_time": 7.0,
        "time_unit": "ms",
        "affine_body_count": 8,
        "static_triangle_body_count": 1,
        "point_triangle_pair_count": 8,
        "valid_step_count": 8,
        "failed_steps": 0,
        "converged_solve_count": 8,
        "barrier_active_count": 8,
        "solver_iterations": 64,
        "total_objective_decrease": 0.12,
        "max_final_gradient_norm": 1e-8,
        "min_target_squared_distance": 1e-4,
        "min_final_squared_distance": 2e-3,
        "squared_activation_distance": 0.25,
        "max_linear_speed_m_s": 0.8,
        "max_affine_velocity_norm": 1.2,
        "max_displacement_norm_m": 0.03,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _abd_wrecking_ball_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_abd_wrecking_ball_reduced_pair_runtime_step_median",
        "run_name": "BM_Plan083CpuScene_abd_wrecking_ball_reduced_pair_runtime_step",
        "aggregate_name": "median",
        "real_time": 5.0,
        "cpu_time": 5.0,
        "time_unit": "ms",
        "affine_body_count": 2,
        "dynamic_pair_count": 1,
        "valid_step_count": 1,
        "failed_steps": 0,
        "converged_solve_count": 1,
        "barrier_active_count": 1,
        "solver_iterations": 18,
        "total_objective_decrease": 0.04,
        "max_final_gradient_norm": 1e-8,
        "min_target_squared_distance": 1e-4,
        "min_final_squared_distance": 2e-3,
        "squared_activation_distance": 0.25,
        "max_linear_speed_m_s": 0.6,
        "max_affine_velocity_norm": 0.9,
        "max_displacement_norm_m": 0.02,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _abd_chain_packet(scene: str, pair_count: int, **overrides):
    row_name = f"BM_Plan083CpuScene_{scene}_reduced_pair_runtime_step"
    row = {
        "name": f"{row_name}_median",
        "run_name": row_name,
        "aggregate_name": "median",
        "real_time": 6.0,
        "cpu_time": 6.0,
        "time_unit": "ms",
        "affine_body_count": 2 * pair_count,
        "dynamic_pair_count": pair_count,
        "valid_step_count": pair_count,
        "failed_steps": 0,
        "converged_solve_count": pair_count,
        "barrier_active_count": pair_count,
        "solver_iterations": 18 * pair_count,
        "total_objective_decrease": 0.04 * pair_count,
        "max_final_gradient_norm": 1e-8,
        "min_target_squared_distance": 1e-4,
        "min_final_squared_distance": 2e-3,
        "squared_activation_distance": 0.25,
        "max_linear_speed_m_s": 0.6,
        "max_affine_velocity_norm": 0.9,
        "max_displacement_norm_m": 0.02,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _abd_comparison_packet(scene: str, pair_count: int, **overrides):
    suffix = (
        "reduced_side_by_side_step"
        if scene == "abd_fem_coupling"
        else "reduced_pair_runtime_step"
    )
    row_name = f"BM_Plan083CpuScene_{scene}_{suffix}"
    paper_counts = {
        "abd_gears": (28, 2_500_000),
        "abd_bullet_small": (16, 1_200),
        "abd_bullet_medium": (142, 3_500),
        "abd_bullet_large": (562, 11_000),
        "abd_complex_geometry": (29, 1_200_000),
        "abd_fem_coupling": (27, 1_100_000),
    }
    paper_body_count, paper_triangle_count = paper_counts[scene]
    row = {
        "name": f"{row_name}_median",
        "run_name": row_name,
        "aggregate_name": "median",
        "real_time": 6.0,
        "cpu_time": 6.0,
        "time_unit": "ms",
        "affine_body_count": 2 * pair_count,
        "dynamic_pair_count": pair_count,
        "reduced_pair_count": pair_count,
        "paper_body_count": paper_body_count,
        "paper_triangle_count": paper_triangle_count,
        "reference_baseline_measured": 0,
        "valid_step_count": pair_count,
        "failed_steps": 0,
        "converged_solve_count": pair_count,
        "barrier_active_count": pair_count,
        "solver_iterations": 18 * pair_count,
        "total_objective_decrease": 0.04 * pair_count,
        "max_final_gradient_norm": 1e-8,
        "min_target_squared_distance": 1e-4,
        "min_final_squared_distance": 2e-3,
        "squared_activation_distance": 0.25,
        "max_linear_speed_m_s": 0.6,
        "max_affine_velocity_norm": 0.9,
        "max_displacement_norm_m": 0.02,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _lying_flat_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_lying_flat_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_lying_flat_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 11.0,
        "cpu_time": 11.0,
        "time_unit": "ms",
        "rigid_obstacle_count": 6,
        "deformable_body_count": 3,
        "deformable_node_count": 31,
        "deformable_edge_count": 68,
        "surface_triangle_count": 32,
        "solver_iterations": 6,
        "active_contact_count": 6,
        "friction_dissipation": 0.945552,
        "min_active_contact_distance_m": 0.0,
        "min_cloth_height_m": 0.0763259,
        "cloth_span_x_m": 0.27488,
        "cloth_span_y_m": 0.167135,
        "failed_steps": 0,
        **_surface_contact_counters(),
        "line_search_trials": 68,
        "surface_contact_candidate_builds": 68,
        "surface_contact_point_triangle_candidates": 660,
        "surface_contact_edge_edge_candidates": 1224,
        "surface_contact_ccd_point_triangle_checks": 660,
        "surface_contact_ccd_edge_edge_checks": 1224,
        "surface_contact_ccd_hits": 1,
        "surface_contact_ccd_limited_steps": 0,
        "inter_body_surface_contact_candidate_builds": 67,
        "inter_body_surface_contact_point_triangle_candidates": 33,
        "inter_body_surface_contact_edge_edge_candidates": 0,
        "inter_body_surface_contact_ccd_point_triangle_checks": 33,
        "inter_body_surface_contact_ccd_edge_edge_checks": 0,
        "inter_body_surface_contact_ccd_hits": 33,
        "inter_body_surface_contact_ccd_limited_steps": 1,
        "inter_body_surface_contact_ccd_zero_step_count": 32,
        "static_rigid_surface_ccd_snapshot_builds": 1,
        "static_rigid_surface_ccd_box_count": 1,
        "static_rigid_surface_ccd_sphere_count": 0,
        "static_rigid_surface_ccd_triangle_count": 12,
        "static_rigid_surface_ccd_edge_count": 12,
        "static_rigid_surface_ccd_candidate_builds": 35,
        "static_rigid_surface_ccd_point_triangle_candidates": 68,
        "static_rigid_surface_ccd_edge_edge_candidates": 102,
        "static_rigid_surface_ccd_point_triangle_checks": 68,
        "static_rigid_surface_ccd_edge_edge_checks": 102,
        "static_rigid_surface_ccd_hits": 34,
        "static_rigid_surface_ccd_limited_steps": 1,
        "static_rigid_surface_ccd_zero_step_count": 62,
        "moving_rigid_surface_ccd_snapshot_builds": 1,
        "moving_rigid_surface_ccd_box_count": 1,
        "moving_rigid_surface_ccd_sample_count": 10,
        "moving_rigid_surface_ccd_inflated_box_count": 0,
        "moving_rigid_surface_ccd_triangle_count": 120,
        "moving_rigid_surface_ccd_edge_count": 120,
        "moving_rigid_surface_ccd_candidate_builds": 3,
        "moving_rigid_surface_ccd_point_triangle_candidates": 2,
        "moving_rigid_surface_ccd_edge_edge_candidates": 41,
        "moving_rigid_surface_ccd_point_triangle_checks": 2,
        "moving_rigid_surface_ccd_edge_edge_checks": 41,
        "moving_rigid_surface_ccd_hits": 1,
        "moving_rigid_surface_ccd_limited_steps": 1,
        "moving_rigid_surface_ccd_zero_step_count": 0,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _external_surface_ccd_packet(**overrides):
    counters = {key: 0 for key in _surface_contact_counters()}
    counters.update(
        {
            "line_search_trials": 3,
            "inter_body_surface_contact_candidate_builds": 2,
            "inter_body_surface_contact_point_triangle_candidates": 2,
            "inter_body_surface_contact_edge_edge_candidates": 0,
            "inter_body_surface_contact_ccd_point_triangle_checks": 2,
            "inter_body_surface_contact_ccd_edge_edge_checks": 0,
            "inter_body_surface_contact_ccd_hits": 2,
            "inter_body_surface_contact_ccd_limited_steps": 2,
            "inter_body_surface_contact_ccd_zero_step_count": 0,
            "static_rigid_surface_ccd_snapshot_builds": 2,
            "static_rigid_surface_ccd_box_count": 2,
            "static_rigid_surface_ccd_sphere_count": 0,
            "static_rigid_surface_ccd_triangle_count": 24,
            "static_rigid_surface_ccd_edge_count": 24,
            "static_rigid_surface_ccd_candidate_builds": 2,
            "static_rigid_surface_ccd_point_triangle_candidates": 2,
            "static_rigid_surface_ccd_edge_edge_candidates": 0,
            "static_rigid_surface_ccd_point_triangle_checks": 2,
            "static_rigid_surface_ccd_edge_edge_checks": 0,
            "static_rigid_surface_ccd_hits": 2,
            "static_rigid_surface_ccd_limited_steps": 2,
            "static_rigid_surface_ccd_zero_step_count": 0,
            "moving_rigid_surface_ccd_snapshot_builds": 2,
            "moving_rigid_surface_ccd_box_count": 2,
            "moving_rigid_surface_ccd_sample_count": 4,
            "moving_rigid_surface_ccd_inflated_box_count": 2,
            "moving_rigid_surface_ccd_triangle_count": 48,
            "moving_rigid_surface_ccd_edge_count": 48,
            "moving_rigid_surface_ccd_candidate_builds": 2,
            "moving_rigid_surface_ccd_point_triangle_candidates": 2,
            "moving_rigid_surface_ccd_edge_edge_candidates": 0,
            "moving_rigid_surface_ccd_point_triangle_checks": 2,
            "moving_rigid_surface_ccd_edge_edge_checks": 0,
            "moving_rigid_surface_ccd_hits": 2,
            "moving_rigid_surface_ccd_limited_steps": 2,
            "moving_rigid_surface_ccd_zero_step_count": 0,
            "mixed_external_surface_ccd_scene_count": 1,
            "mixed_external_surface_ccd_family_count": 3,
            "mixed_inter_body_surface_contact_candidate_builds": 1,
            "mixed_inter_body_surface_contact_point_triangle_candidates": 1,
            "mixed_inter_body_surface_contact_ccd_point_triangle_checks": 1,
            "mixed_inter_body_surface_contact_ccd_hits": 1,
            "mixed_inter_body_surface_contact_ccd_limited_steps": 1,
            "mixed_static_rigid_surface_ccd_candidate_builds": 1,
            "mixed_static_rigid_surface_ccd_point_triangle_candidates": 1,
            "mixed_static_rigid_surface_ccd_point_triangle_checks": 1,
            "mixed_static_rigid_surface_ccd_hits": 1,
            "mixed_static_rigid_surface_ccd_limited_steps": 1,
            "mixed_moving_rigid_surface_ccd_candidate_builds": 1,
            "mixed_moving_rigid_surface_ccd_point_triangle_candidates": 1,
            "mixed_moving_rigid_surface_ccd_point_triangle_checks": 1,
            "mixed_moving_rigid_surface_ccd_hits": 1,
            "mixed_moving_rigid_surface_ccd_limited_steps": 1,
        }
    )
    row = {
        "name": "BM_Plan083CpuScene_external_surface_ccd_diagnostics_median",
        "run_name": "BM_Plan083CpuScene_external_surface_ccd_diagnostics",
        "aggregate_name": "median",
        "real_time": 12.0,
        "cpu_time": 12.0,
        "time_unit": "ms",
        "external_surface_ccd_scene_count": 4,
        "mixed_external_surface_ccd_scene_count": 1,
        "rigid_obstacle_count": 4,
        "static_rigid_obstacle_count": 2,
        "moving_rigid_obstacle_count": 2,
        "deformable_body_count": 8,
        "deformable_node_count": 18,
        "deformable_edge_count": 0,
        "surface_triangle_count": 4,
        "failed_steps": 0,
        **counters,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _nunchaku_scaling_packet(**overrides):
    rows = []
    for size in (20, 40, 60, 80, 100):
        row = {
            "name": f"BM_Plan083CpuScene_nunchaku_scaling_reduced_world_step/{size}_median",
            "run_name": f"BM_Plan083CpuScene_nunchaku_scaling_reduced_world_step/{size}",
            "aggregate_name": "median",
            "real_time": float(size),
            "cpu_time": float(size),
            "time_unit": "ms",
            "nunchaku_pair_count": size,
            "body_count": 2 * size,
            "dynamic_body_count": size,
            "revolute_joint_count": size,
            "active_articulation_constraints": 2 * size,
            "solver_iterations": 0,
            "failed_steps": 0,
            "final_equality_residual_norm": 0.0,
            "free_axis_angular_velocity_rad_s": 1.5,
        }
        if overrides.get("size") == size:
            row.update({k: v for k, v in overrides.items() if k != "size"})
        rows.append(row)
    return {"benchmarks": rows}


def _windmill_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_windmill_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_windmill_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 4.0,
        "cpu_time": 4.0,
        "time_unit": "ms",
        "body_count": 3,
        "dynamic_body_count": 2,
        "revolute_joint_count": 1,
        "active_constraints": 4,
        "active_friction_constraints": 2,
        "active_articulation_constraints": 2,
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "blade_tip_radius_m": 0.36,
        "striker_height_m": 0.95,
        "striker_blade_clearance_m": 0.01,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _terrain_vehicle_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_terrain_vehicle_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_terrain_vehicle_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 5.0,
        "cpu_time": 5.0,
        "time_unit": "ms",
        "body_count": 6,
        "dynamic_body_count": 5,
        "wheel_count": 4,
        "revolute_joint_count": 4,
        "active_constraints": 18,
        "active_friction_constraints": 10,
        "active_articulation_constraints": 8,
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "chassis_height_m": 0.17,
        "min_wheel_ground_clearance_m": 0.01,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _precession_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_precession_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_precession_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 6.0,
        "cpu_time": 6.0,
        "time_unit": "ms",
        "body_count": 2,
        "dynamic_body_count": 1,
        "active_constraints": 8,
        "active_friction_constraints": 8,
        "solver_iterations": 3,
        "failed_steps": 0,
        "final_equality_residual_norm": 0.0,
        "wheel_height_m": 0.16,
        "wheel_ground_clearance_m": 0.0,
        "spin_rate_rad_s": 8.1,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _ragdoll_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_ragdoll_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_ragdoll_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 7.0,
        "cpu_time": 7.0,
        "time_unit": "ms",
        "body_count": 7,
        "dynamic_body_count": 6,
        "ragdoll_body_count": 6,
        "revolute_joint_count": 5,
        "active_constraints": 12,
        "active_friction_constraints": 12,
        "active_articulation_constraints": 10,
        "solver_iterations": 3,
        "failed_steps": 0,
        "final_equality_residual_norm": 0.0,
        "torso_height_m": 0.42,
        "min_leg_ground_clearance_m": 0.0,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _timing_breakdown_packet(**overrides):
    rows = [
        _benchmark_packet()["benchmarks"][0],
        _lying_flat_packet()["benchmarks"][0],
        _pulley_packet()["benchmarks"][0],
        _umbrella_packet()["benchmarks"][0],
        _nunchaku_packet()["benchmarks"][0],
        _terrain_vehicle_packet()["benchmarks"][0],
        _ragdoll_packet()["benchmarks"][0],
        _windmill_packet()["benchmarks"][0],
        _candy_packet()["benchmarks"][0],
        _precession_packet()["benchmarks"][0],
    ]
    if "row_index" in overrides:
        row_index = overrides.pop("row_index")
        rows[row_index].update(overrides)
    return {"benchmarks": rows}


def _table2_packet(**overrides):
    rows = [
        _lying_flat_packet()["benchmarks"][0],
        _benchmark_packet()["benchmarks"][0],
        _pulley_packet()["benchmarks"][0],
        _umbrella_packet()["benchmarks"][0],
        _terrain_vehicle_packet()["benchmarks"][0],
        _ragdoll_packet()["benchmarks"][0],
        _windmill_packet()["benchmarks"][0],
        _candy_packet()["benchmarks"][0],
        _precession_packet()["benchmarks"][0],
    ]
    if "row_index" in overrides:
        row_index = overrides.pop("row_index")
        rows[row_index].update(overrides)
    return {"benchmarks": rows}


def test_plan083_cpu_scene_packet_accepts_reduced_hanging_bridge() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_packet(),
        max_equality_residual=1e-8,
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-02"
    assert row["scene_id"] == "plan083_hanging_bridge"
    assert row["paper_scale"] is False
    assert row["body_count"] == 7
    assert row["dynamic_body_count"] == 5
    assert row["fixed_joint_count"] == 4
    assert row["active_articulation_constraints"] == 12
    assert row["wall_time_ns"] == 2.0e6
    assert (
        row["runtime_path"]
        == "rigid IPC World::step plus deformable IPC World::step external surface CCD sidecar"
    )
    assert row["external_surface_ccd_sidecar_scene_count"] == 1
    assert row["inter_body_surface_contact_ccd_hits"] == 33
    assert row["static_rigid_surface_ccd_hits"] == 34
    assert row["moving_rigid_surface_ccd_hits"] == 1


def test_plan083_cpu_scene_packet_rejects_hanging_bridge_without_sidecar() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="external surface CCD"):
        module.make_packet(
            _benchmark_packet(external_surface_ccd_sidecar_scene_count=0),
            max_equality_residual=1e-8,
        )


def test_plan083_cpu_scene_packet_rejects_hanging_bridge_without_external_ccd() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="external surface CCD"):
        module.make_packet(
            _benchmark_packet(inter_body_surface_contact_ccd_hits=0),
            max_equality_residual=1e-8,
        )


def test_plan083_cpu_scene_packet_accepts_reduced_nunchaku() -> None:
    module = _load_module()

    packet = module.make_packet(
        _nunchaku_packet(),
        max_equality_residual=1e-8,
        scene="nunchaku_single",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-13"
    assert row["scene_id"] == "plan083_nunchaku"
    assert row["paper_scale"] is False
    assert row["body_count"] == 2
    assert row["dynamic_body_count"] == 1
    assert row["revolute_joint_count"] == 1
    assert row["active_articulation_constraints"] == 2
    assert row["wall_time_ns"] == 3.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_pulley() -> None:
    module = _load_module()

    packet = module.make_packet(
        _pulley_packet(),
        max_equality_residual=1e-8,
        scene="pulley_system",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-03"
    assert row["scene_id"] == "plan083_pulley_system"
    assert row["paper_scale"] is False
    assert row["body_count"] == 4
    assert row["dynamic_body_count"] == 3
    assert row["fixed_joint_count"] == 2
    assert row["revolute_joint_count"] == 1
    assert row["active_articulation_constraints"] == 8
    assert row["wall_time_ns"] == 8.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_umbrella() -> None:
    module = _load_module()

    packet = module.make_packet(
        _umbrella_packet(),
        max_equality_residual=1e-8,
        scene="umbrella",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-04"
    assert row["scene_id"] == "plan083_umbrella"
    assert row["paper_scale"] is False
    assert row["body_count"] == 4
    assert row["dynamic_body_count"] == 3
    assert row["fixed_joint_count"] == 2
    assert row["revolute_joint_count"] == 1
    assert row["active_articulation_constraints"] == 8
    assert row["wall_time_ns"] == 9.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_candy() -> None:
    module = _load_module()

    packet = module.make_packet(
        _candy_packet(),
        max_equality_residual=1e-8,
        scene="candy",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-22"
    assert row["scene_id"] == "plan083_candy"
    assert row["paper_scale"] is False
    assert row["rigid_obstacle_count"] == 3
    assert row["deformable_body_count"] == 3
    assert row["deformable_node_count"] == 27
    assert row["surface_triangle_count"] == 32
    assert row["line_search_trials"] == 69
    assert row["surface_contact_candidate_builds"] == 3
    assert row["surface_contact_point_triangle_candidates"] == 64
    assert row["surface_contact_edge_edge_candidates"] == 207
    assert row["surface_contact_ccd_hits"] == 0
    assert row["surface_contact_ccd_limited_steps"] == 0
    assert row["inter_body_surface_contact_candidate_builds"] == 0
    assert row["static_rigid_surface_ccd_box_count"] == 1
    assert row["static_rigid_surface_ccd_candidate_builds"] == 69
    assert row["static_rigid_surface_ccd_point_triangle_candidates"] == 72
    assert row["static_rigid_surface_ccd_point_triangle_checks"] == 72
    assert row["static_rigid_surface_ccd_hits"] == 72
    assert row["static_rigid_surface_ccd_limited_steps"] == 1
    assert row["moving_rigid_surface_ccd_box_count"] == 1
    assert row["moving_rigid_surface_ccd_sample_count"] == 6
    assert row["moving_rigid_surface_ccd_candidate_builds"] == 37
    assert row["moving_rigid_surface_ccd_point_triangle_candidates"] == 184
    assert row["moving_rigid_surface_ccd_point_triangle_checks"] == 184
    assert row["moving_rigid_surface_ccd_hits"] == 184
    assert row["moving_rigid_surface_ccd_limited_steps"] == 1
    assert row["wall_time_ns"] == 10.0e6


def test_plan083_cpu_scene_packet_rejects_surface_contact_hit_count_mismatch() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="more surface-contact CCD hits than CCD checks",
    ):
        module.make_packet(
            _lying_flat_packet(surface_contact_ccd_hits=9999),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_rejects_inter_body_surface_contact_hits() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="more inter-body surface-contact CCD hits than CCD checks",
    ):
        module.make_packet(
            _lying_flat_packet(inter_body_surface_contact_ccd_hits=999),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_rejects_static_rigid_surface_ccd_hits() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="more static-rigid surface CCD hits than CCD checks",
    ):
        module.make_packet(
            _lying_flat_packet(static_rigid_surface_ccd_hits=999),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_rejects_moving_rigid_surface_ccd_hits() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="more moving-rigid surface CCD hits than CCD checks",
    ):
        module.make_packet(
            _lying_flat_packet(moving_rigid_surface_ccd_hits=999),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_accepts_reduced_abd_house_cards() -> None:
    module = _load_module()

    packet = module.make_packet(
        _abd_house_cards_packet(),
        max_equality_residual=1e-8,
        scene="abd_house_of_cards",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "abd-vs-rigid-cards"
    assert row["scene_id"] == "plan083_abd_house_of_cards"
    assert row["paper_scale"] is False
    assert row["runtime_path"] == "detail affine point-triangle runtime step"
    assert row["affine_body_count"] == 8
    assert row["converged_solve_count"] == 8
    assert row["barrier_active_count"] == 8
    assert row["wall_time_ns"] == 7.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_abd_wrecking_ball() -> None:
    module = _load_module()

    packet = module.make_packet(
        _abd_wrecking_ball_packet(),
        max_equality_residual=1e-8,
        scene="abd_wrecking_ball",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "abd-vs-rigid-wreck"
    assert row["scene_id"] == "plan083_abd_wrecking_ball"
    assert row["paper_scale"] is False
    assert row["runtime_path"] == "detail affine point-triangle pair runtime step"
    assert row["affine_body_count"] == 2
    assert row["dynamic_pair_count"] == 1
    assert row["converged_solve_count"] == 1
    assert row["barrier_active_count"] == 1
    assert row["wall_time_ns"] == 5.0e6


@pytest.mark.parametrize(
    ("scene", "row_id", "scene_id", "pair_count"),
    [
        ("abd_chain_8", "abd-chain-8", "plan083_abd_chain_8", 8),
        ("abd_chain_16", "abd-chain-16", "plan083_abd_chain_16", 16),
        ("abd_chain_96", "abd-chain-96", "plan083_abd_chain_96", 96),
    ],
)
def test_plan083_cpu_scene_packet_accepts_reduced_abd_chain(
    scene: str,
    row_id: str,
    scene_id: str,
    pair_count: int,
) -> None:
    module = _load_module()

    packet = module.make_packet(
        _abd_chain_packet(scene, pair_count),
        max_equality_residual=1e-8,
        scene=scene,
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == row_id
    assert row["scene_id"] == scene_id
    assert row["paper_scale"] is False
    assert row["runtime_path"] == "detail affine point-triangle pair runtime step"
    assert row["affine_body_count"] == 2 * pair_count
    assert row["dynamic_pair_count"] == pair_count
    assert row["converged_solve_count"] == pair_count
    assert row["barrier_active_count"] == pair_count
    assert row["wall_time_ns"] == 6.0e6


@pytest.mark.parametrize(
    ("scene", "row_id", "scene_id", "pair_count", "paper_body_count"),
    [
        ("abd_gears", "abd-gears", "plan083_abd_gears", 28, 28),
        (
            "abd_bullet_small",
            "abd-bullet-small",
            "plan083_abd_bullet_small",
            16,
            16,
        ),
        (
            "abd_bullet_medium",
            "abd-bullet-medium",
            "plan083_abd_bullet_medium",
            48,
            142,
        ),
        (
            "abd_bullet_large",
            "abd-bullet-large",
            "plan083_abd_bullet_large",
            96,
            562,
        ),
        (
            "abd_complex_geometry",
            "abd-complex-geometry",
            "plan083_abd_complex_geometry",
            29,
            29,
        ),
    ],
)
def test_plan083_cpu_scene_packet_accepts_reduced_abd_comparison(
    scene: str,
    row_id: str,
    scene_id: str,
    pair_count: int,
    paper_body_count: int,
) -> None:
    module = _load_module()

    packet = module.make_packet(
        _abd_comparison_packet(scene, pair_count),
        max_equality_residual=1e-8,
        scene=scene,
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == row_id
    assert row["scene_id"] == scene_id
    assert row["paper_scale"] is False
    assert row["runtime_path"] == "detail affine point-triangle pair runtime step"
    assert row["affine_body_count"] == 2 * pair_count
    assert row["dynamic_pair_count"] == pair_count
    assert row["reduced_pair_count"] == pair_count
    assert row["paper_body_count"] == paper_body_count
    assert row["reference_baseline_measured"] is False
    assert row["converged_solve_count"] == pair_count
    assert row["barrier_active_count"] == pair_count
    assert row["wall_time_ns"] == 6.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_abd_fem_coupling() -> None:
    module = _load_module()

    packet = module.make_packet(
        _abd_comparison_packet(
            "abd_fem_coupling",
            27,
            deformable_body_count=3,
            deformable_node_count=31,
            deformable_edge_count=68,
            surface_triangle_count=30,
            deformable_solver_iterations=6,
            min_cloth_height_m=0.0763259,
            affine_fem_candidate_diagnostics_measured=1,
            affine_fem_mixed_candidate_count=26,
            affine_fem_mixed_active_barrier_count=26,
            affine_fem_mixed_min_squared_distance=2.87308e-4,
            affine_fem_mixed_barrier_value=0.00167175,
            affine_fem_coupled_contact_measured=1,
            affine_fem_coupled_solve_converged=1,
            affine_fem_coupled_objective_decrease=5.6378e-5,
            affine_fem_coupled_initial_gradient_norm=0.0106202,
            affine_fem_coupled_final_gradient_norm=3.27139e-8,
            affine_fem_coupled_affine_displacement_norm=0.0100439,
            affine_fem_coupled_deformable_displacement_norm=0.0034389,
            **_abd_fem_surface_contact_counters(),
        ),
        max_equality_residual=1e-8,
        scene="abd_fem_coupling",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "abd-fem-coupling"
    assert row["scene_id"] == "plan083_abd_fem_coupling"
    assert row["paper_scale"] is False
    assert (
        row["runtime_path"]
        == "detail affine point-triangle runtime step plus deformable IPC World::step external surface CCD sidecar"
    )
    assert row["affine_body_count"] == 54
    assert row["dynamic_pair_count"] == 27
    assert row["deformable_body_count"] == 3
    assert row["deformable_node_count"] == 31
    assert row["line_search_trials"] == 68
    assert row["surface_contact_candidate_builds"] == 68
    assert row["surface_contact_ccd_hits"] == 1
    assert row["inter_body_surface_contact_candidate_builds"] == 67
    assert row["inter_body_surface_contact_ccd_hits"] == 33
    assert row["inter_body_surface_contact_ccd_limited_steps"] == 1
    assert row["static_rigid_surface_ccd_candidate_builds"] == 35
    assert row["static_rigid_surface_ccd_hits"] == 34
    assert row["static_rigid_surface_ccd_limited_steps"] == 1
    assert row["moving_rigid_surface_ccd_candidate_builds"] == 3
    assert row["moving_rigid_surface_ccd_edge_edge_checks"] == 41
    assert row["moving_rigid_surface_ccd_hits"] == 1
    assert row["moving_rigid_surface_ccd_limited_steps"] == 1
    assert row["affine_fem_candidate_diagnostics_measured"] is True
    assert row["affine_fem_mixed_candidate_count"] == 26
    assert row["affine_fem_mixed_active_barrier_count"] == 26
    assert row["affine_fem_coupled_contact_measured"] is True
    assert row["affine_fem_coupled_solve_converged"] is True
    assert row["wall_time_ns"] == 6.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_lying_flat() -> None:
    module = _load_module()

    packet = module.make_packet(
        _lying_flat_packet(),
        max_equality_residual=1e-8,
        scene="lying_flat",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-01"
    assert row["scene_id"] == "plan083_lying_flat"
    assert row["paper_scale"] is False
    assert row["rigid_obstacle_count"] == 6
    assert row["deformable_body_count"] == 3
    assert row["deformable_node_count"] == 31
    assert row["surface_triangle_count"] == 32
    assert row["line_search_trials"] == 68
    assert row["surface_contact_candidate_builds"] == 68
    assert row["surface_contact_point_triangle_candidates"] == 660
    assert row["surface_contact_edge_edge_candidates"] == 1224
    assert row["surface_contact_ccd_hits"] == 1
    assert row["surface_contact_ccd_limited_steps"] == 0
    assert row["inter_body_surface_contact_candidate_builds"] == 67
    assert row["inter_body_surface_contact_point_triangle_candidates"] == 33
    assert row["inter_body_surface_contact_ccd_point_triangle_checks"] == 33
    assert row["inter_body_surface_contact_ccd_hits"] == 33
    assert row["inter_body_surface_contact_ccd_limited_steps"] == 1
    assert row["inter_body_surface_contact_ccd_zero_step_count"] == 32
    assert row["static_rigid_surface_ccd_box_count"] == 1
    assert row["static_rigid_surface_ccd_candidate_builds"] == 35
    assert row["static_rigid_surface_ccd_point_triangle_checks"] == 68
    assert row["static_rigid_surface_ccd_edge_edge_checks"] == 102
    assert row["static_rigid_surface_ccd_hits"] == 34
    assert row["static_rigid_surface_ccd_limited_steps"] == 1
    assert row["moving_rigid_surface_ccd_box_count"] == 1
    assert row["moving_rigid_surface_ccd_sample_count"] == 10
    assert row["moving_rigid_surface_ccd_candidate_builds"] == 3
    assert row["moving_rigid_surface_ccd_point_triangle_checks"] == 2
    assert row["moving_rigid_surface_ccd_edge_edge_checks"] == 41
    assert row["moving_rigid_surface_ccd_hits"] == 1
    assert row["moving_rigid_surface_ccd_limited_steps"] == 1
    assert row["wall_time_ns"] == 11.0e6


def test_plan083_cpu_scene_packet_accepts_external_surface_ccd() -> None:
    module = _load_module()

    packet = module.make_packet(
        _external_surface_ccd_packet(),
        max_equality_residual=1e-8,
        scene="external_surface_ccd",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-alg-barriers"
    assert row["scene_id"] == "plan083_external_surface_ccd"
    assert row["paper_scale"] is False
    assert row["external_surface_ccd_scene_count"] == 4
    assert row["mixed_external_surface_ccd_scene_count"] == 1
    assert row["mixed_external_surface_ccd_family_count"] == 3
    assert row["deformable_body_count"] == 8
    assert row["deformable_node_count"] == 18
    assert row["surface_triangle_count"] == 4
    assert row["rigid_obstacle_count"] == 4
    assert row["static_rigid_obstacle_count"] == 2
    assert row["moving_rigid_obstacle_count"] == 2
    assert row["surface_contact_candidate_builds"] == 0
    assert row["inter_body_surface_contact_candidate_builds"] == 2
    assert row["inter_body_surface_contact_ccd_hits"] == 2
    assert row["static_rigid_surface_ccd_box_count"] == 2
    assert row["static_rigid_surface_ccd_triangle_count"] == 24
    assert row["static_rigid_surface_ccd_hits"] == 2
    assert row["moving_rigid_surface_ccd_box_count"] == 2
    assert row["moving_rigid_surface_ccd_sample_count"] == 4
    assert row["moving_rigid_surface_ccd_hits"] == 2
    assert row["mixed_inter_body_surface_contact_ccd_hits"] == 1
    assert row["mixed_static_rigid_surface_ccd_hits"] == 1
    assert row["mixed_moving_rigid_surface_ccd_hits"] == 1
    assert row["wall_time_ns"] == 12.0e6


def test_plan083_cpu_scene_packet_rejects_missing_mixed_external_surface_ccd() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="mixed scene requires positive mixed_static_rigid_surface_ccd_hits",
    ):
        module.make_packet(
            _external_surface_ccd_packet(mixed_static_rigid_surface_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="external_surface_ccd",
        )


def test_plan083_cpu_scene_packet_accepts_reduced_nunchaku_scaling() -> None:
    module = _load_module()

    packet = module.make_packet(
        _nunchaku_scaling_packet(),
        max_equality_residual=1e-8,
        scene="nunchaku_scaling",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-25"
    assert row["scene_id"] == "plan083_nunchaku"
    assert row["paper_scale"] is False
    assert row["sample_sizes"] == [20, 40, 60, 80, 100]
    assert row["sample_count"] == 5
    assert row["samples"][0]["body_count"] == 40
    assert row["samples"][-1]["revolute_joint_count"] == 100


def test_plan083_cpu_scene_packet_accepts_reduced_windmill() -> None:
    module = _load_module()

    packet = module.make_packet(
        _windmill_packet(),
        max_equality_residual=1e-8,
        scene="windmill",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-20"
    assert row["scene_id"] == "plan083_windmill"
    assert row["paper_scale"] is False
    assert row["body_count"] == 3
    assert row["dynamic_body_count"] == 2
    assert row["revolute_joint_count"] == 1
    assert row["active_articulation_constraints"] == 2
    assert row["wall_time_ns"] == 4.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_terrain_vehicle() -> None:
    module = _load_module()

    packet = module.make_packet(
        _terrain_vehicle_packet(),
        max_equality_residual=1e-8,
        scene="terrain_vehicle",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-10"
    assert row["scene_id"] == "plan083_terrain_vehicle"
    assert row["paper_scale"] is False
    assert row["body_count"] == 6
    assert row["dynamic_body_count"] == 5
    assert row["wheel_count"] == 4
    assert row["revolute_joint_count"] == 4
    assert row["active_articulation_constraints"] == 8
    assert row["wall_time_ns"] == 5.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_precession() -> None:
    module = _load_module()

    packet = module.make_packet(
        _precession_packet(),
        max_equality_residual=1e-8,
        scene="precession",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-23"
    assert row["scene_id"] == "plan083_precession"
    assert row["paper_scale"] is False
    assert row["body_count"] == 2
    assert row["dynamic_body_count"] == 1
    assert row["active_constraints"] == 8
    assert row["active_friction_constraints"] == 8
    assert row["wall_time_ns"] == 6.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_ragdoll() -> None:
    module = _load_module()

    packet = module.make_packet(
        _ragdoll_packet(),
        max_equality_residual=1e-8,
        scene="ragdoll_reduced",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-11"
    assert row["scene_id"] == "plan083_ragdolls"
    assert row["paper_scale"] is False
    assert row["body_count"] == 7
    assert row["dynamic_body_count"] == 6
    assert row["ragdoll_body_count"] == 6
    assert row["revolute_joint_count"] == 5
    assert row["active_articulation_constraints"] == 10
    assert row["wall_time_ns"] == 7.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_timing_breakdown() -> None:
    module = _load_module()

    packet = module.make_packet(
        _timing_breakdown_packet(),
        max_equality_residual=1e-8,
        scene="timing_breakdown",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-24"
    assert row["scene_id"] == "plan083_reduced_timing_breakdown"
    assert row["paper_scale"] is False
    assert row["scene_count"] == 10
    assert row["total_body_count"] == 50
    assert row["total_dynamic_body_count"] == 32
    assert row["total_wall_time_ns"] == 65.0e6
    assert row["available_timing_fields"] == ["wall_time_ns"]
    assert "linear_solve" in row["missing_paper_timing_fields"]


def test_plan083_cpu_scene_packet_accepts_reduced_table2() -> None:
    module = _load_module()

    packet = module.make_packet(
        _table2_packet(),
        max_equality_residual=1e-8,
        scene="table_2",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-table-02"
    assert row["scene_id"] == "plan083_reduced_table_2"
    assert row["paper_scale"] is False
    assert row["scene_count"] == 9
    assert row["covered_paper_rows"] == [
        "unb-fig-01",
        "unb-fig-02",
        "unb-fig-03",
        "unb-fig-04",
        "unb-fig-10",
        "unb-fig-11",
        "unb-fig-20",
        "unb-fig-22",
        "unb-fig-23",
    ]
    assert row["missing_paper_rows"] == []
    assert row["total_body_count"] == 48
    assert row["total_dynamic_body_count"] == 31
    assert row["total_wall_time_ns"] == 62.0e6


def test_plan083_cpu_scene_packet_rejects_failed_bridge_step() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="failed steps"):
        module.make_packet(
            _benchmark_packet(failed_steps=1),
            max_equality_residual=1e-8,
        )


def test_plan083_cpu_scene_packet_rejects_high_equality_residual() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="equality residual",
    ):
        module.make_packet(
            _benchmark_packet(final_equality_residual_norm=1e-5),
            max_equality_residual=1e-8,
        )


def test_plan083_cpu_scene_packet_requires_representative_row() -> None:
    module = _load_module()
    packet = _benchmark_packet(
        run_name="BM_Plan083CpuScene_other_scene",
        name="BM_Plan083CpuScene_other_scene_median",
    )

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="unexpected benchmark row",
    ):
        module.make_packet(packet, max_equality_residual=1e-8)


def test_plan083_cpu_scene_packet_rejects_nunchaku_without_hinge() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="revolute joint"):
        module.make_packet(
            _nunchaku_packet(revolute_joint_count=0),
            max_equality_residual=1e-8,
            scene="nunchaku_single",
        )


def test_plan083_cpu_scene_packet_rejects_pulley_without_point_connections() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="point-connection"):
        module.make_packet(
            _pulley_packet(fixed_joint_count=1),
            max_equality_residual=1e-8,
            scene="pulley_system",
        )


def test_plan083_cpu_scene_packet_rejects_umbrella_without_hinge() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="umbrella hinge"):
        module.make_packet(
            _umbrella_packet(revolute_joint_count=0),
            max_equality_residual=1e-8,
            scene="umbrella",
        )


def test_plan083_cpu_scene_packet_rejects_candy_penetration() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="penetrated"):
        module.make_packet(
            _candy_packet(min_cloth_height_m=-1e-3),
            max_equality_residual=1e-8,
            scene="candy",
        )


def test_plan083_cpu_scene_packet_rejects_candy_without_static_ccd_witness() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive static-rigid surface CCD witness counter",
    ):
        module.make_packet(
            _candy_packet(static_rigid_surface_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="candy",
        )


def test_plan083_cpu_scene_packet_rejects_candy_without_moving_ccd_witness() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive moving-rigid surface CCD witness counter",
    ):
        module.make_packet(
            _candy_packet(moving_rigid_surface_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="candy",
        )


def test_plan083_cpu_scene_packet_rejects_abd_cards_without_convergence() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="converged"):
        module.make_packet(
            _abd_house_cards_packet(converged_solve_count=7),
            max_equality_residual=1e-8,
            scene="abd_house_of_cards",
        )


def test_plan083_cpu_scene_packet_rejects_abd_cards_with_large_gradient() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="gradient"):
        module.make_packet(
            _abd_house_cards_packet(max_final_gradient_norm=1.1e-5),
            max_equality_residual=1e-8,
            scene="abd_house_of_cards",
        )


def test_plan083_cpu_scene_packet_rejects_abd_wreck_without_pair() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="dynamic"):
        module.make_packet(
            _abd_wrecking_ball_packet(dynamic_pair_count=0),
            max_equality_residual=1e-8,
            scene="abd_wrecking_ball",
        )


def test_plan083_cpu_scene_packet_rejects_abd_chain_wrong_pair_count() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="dynamic"):
        module.make_packet(
            _abd_chain_packet("abd_chain_16", 16, dynamic_pair_count=15),
            max_equality_residual=1e-8,
            scene="abd_chain_16",
        )


def test_plan083_cpu_scene_packet_rejects_abd_comparison_reference_claim() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="reference"):
        module.make_packet(
            _abd_comparison_packet(
                "abd_bullet_medium",
                48,
                reference_baseline_measured=1,
            ),
            max_equality_residual=1e-8,
            scene="abd_bullet_medium",
        )


def test_plan083_cpu_scene_packet_rejects_abd_comparison_wrong_pair_count() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="dynamic"):
        module.make_packet(
            _abd_comparison_packet("abd_gears", 28, dynamic_pair_count=27),
            max_equality_residual=1e-8,
            scene="abd_gears",
        )


def test_plan083_cpu_scene_packet_rejects_abd_fem_without_coupled_contact() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="coupled"):
        module.make_packet(
            _abd_comparison_packet(
                "abd_fem_coupling",
                27,
                deformable_body_count=3,
                deformable_node_count=31,
                deformable_edge_count=68,
                surface_triangle_count=30,
                deformable_solver_iterations=6,
                min_cloth_height_m=0.0763259,
                affine_fem_candidate_diagnostics_measured=1,
                affine_fem_mixed_candidate_count=26,
                affine_fem_mixed_active_barrier_count=26,
                affine_fem_mixed_min_squared_distance=2.87308e-4,
                affine_fem_mixed_barrier_value=0.00167175,
                affine_fem_coupled_contact_measured=0,
                affine_fem_coupled_solve_converged=1,
                affine_fem_coupled_objective_decrease=5.6378e-5,
                affine_fem_coupled_initial_gradient_norm=0.0106202,
                affine_fem_coupled_final_gradient_norm=3.27139e-8,
                affine_fem_coupled_affine_displacement_norm=0.0100439,
                affine_fem_coupled_deformable_displacement_norm=0.0034389,
                **_abd_fem_surface_contact_counters(),
            ),
            max_equality_residual=1e-8,
            scene="abd_fem_coupling",
        )


def test_plan083_cpu_scene_packet_rejects_abd_fem_without_external_ccd() -> None:
    module = _load_module()

    counters = _abd_fem_surface_contact_counters()
    counters["inter_body_surface_contact_ccd_hits"] = 0
    with pytest.raises(module.Plan083CpuScenePacketError, match="external surface CCD"):
        module.make_packet(
            _abd_comparison_packet(
                "abd_fem_coupling",
                27,
                deformable_body_count=3,
                deformable_node_count=31,
                deformable_edge_count=68,
                surface_triangle_count=30,
                deformable_solver_iterations=6,
                min_cloth_height_m=0.0763259,
                affine_fem_candidate_diagnostics_measured=1,
                affine_fem_mixed_candidate_count=26,
                affine_fem_mixed_active_barrier_count=26,
                affine_fem_mixed_min_squared_distance=2.87308e-4,
                affine_fem_mixed_barrier_value=0.00167175,
                affine_fem_coupled_contact_measured=1,
                affine_fem_coupled_solve_converged=1,
                affine_fem_coupled_objective_decrease=5.6378e-5,
                affine_fem_coupled_initial_gradient_norm=0.0106202,
                affine_fem_coupled_final_gradient_norm=3.27139e-8,
                affine_fem_coupled_affine_displacement_norm=0.0100439,
                affine_fem_coupled_deformable_displacement_norm=0.0034389,
                **counters,
            ),
            max_equality_residual=1e-8,
            scene="abd_fem_coupling",
        )


def test_plan083_cpu_scene_packet_rejects_lying_flat_without_obstacles() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="obstacles"):
        module.make_packet(
            _lying_flat_packet(rigid_obstacle_count=2),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_rejects_lying_flat_without_inter_body_ccd_witness() -> (
    None
):
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive inter-body surface CCD witness counter",
    ):
        module.make_packet(
            _lying_flat_packet(inter_body_surface_contact_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_rejects_lying_flat_without_static_ccd_witness() -> (
    None
):
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive static-rigid surface CCD witness counter",
    ):
        module.make_packet(
            _lying_flat_packet(static_rigid_surface_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_rejects_lying_flat_without_moving_ccd_witness() -> (
    None
):
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive moving-rigid surface CCD witness counter",
    ):
        module.make_packet(
            _lying_flat_packet(moving_rigid_surface_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="lying_flat",
        )


def test_plan083_cpu_scene_packet_rejects_external_surface_ccd_without_inter_body_hit() -> (
    None
):
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive inter_body_surface_contact_ccd_hits",
    ):
        module.make_packet(
            _external_surface_ccd_packet(inter_body_surface_contact_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="external_surface_ccd",
        )


def test_plan083_cpu_scene_packet_rejects_external_surface_ccd_without_static_hit() -> (
    None
):
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive static_rigid_surface_ccd_hits",
    ):
        module.make_packet(
            _external_surface_ccd_packet(static_rigid_surface_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="external_surface_ccd",
        )


def test_plan083_cpu_scene_packet_rejects_external_surface_ccd_without_moving_hit() -> (
    None
):
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="positive moving_rigid_surface_ccd_hits",
    ):
        module.make_packet(
            _external_surface_ccd_packet(moving_rigid_surface_ccd_hits=0),
            max_equality_residual=1e-8,
            scene="external_surface_ccd",
        )


def test_plan083_cpu_scene_packet_rejects_nunchaku_scaling_without_size() -> None:
    module = _load_module()
    packet = _nunchaku_scaling_packet()
    packet["benchmarks"] = packet["benchmarks"][:-1]

    with pytest.raises(module.Plan083CpuScenePacketError, match="missing median"):
        module.make_packet(
            packet,
            max_equality_residual=1e-8,
            scene="nunchaku_scaling",
        )


def test_plan083_cpu_scene_packet_rejects_windmill_penetration() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="penetrated"):
        module.make_packet(
            _windmill_packet(striker_blade_clearance_m=-1e-3),
            max_equality_residual=1e-8,
            scene="windmill",
        )


def test_plan083_cpu_scene_packet_rejects_terrain_vehicle_without_wheels() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="passive wheels"):
        module.make_packet(
            _terrain_vehicle_packet(wheel_count=3),
            max_equality_residual=1e-8,
            scene="terrain_vehicle",
        )


def test_plan083_cpu_scene_packet_rejects_precession_without_spin() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="spin rate"):
        module.make_packet(
            _precession_packet(spin_rate_rad_s=0.0),
            max_equality_residual=1e-8,
            scene="precession",
        )


def test_plan083_cpu_scene_packet_rejects_ragdoll_without_ground_contact() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="ground-contact"):
        module.make_packet(
            _ragdoll_packet(active_constraints=0),
            max_equality_residual=1e-8,
            scene="ragdoll_reduced",
        )


def test_plan083_cpu_scene_packet_rejects_timing_breakdown_without_scene() -> None:
    module = _load_module()
    packet = _timing_breakdown_packet()
    packet["benchmarks"] = packet["benchmarks"][:-1]

    with pytest.raises(module.Plan083CpuScenePacketError, match="missing median"):
        module.make_packet(
            packet,
            max_equality_residual=1e-8,
            scene="timing_breakdown",
        )


def test_plan083_cpu_scene_packet_rejects_table2_without_scene() -> None:
    module = _load_module()
    packet = _table2_packet()
    packet["benchmarks"] = packet["benchmarks"][:-1]

    with pytest.raises(module.Plan083CpuScenePacketError, match="missing median"):
        module.make_packet(
            packet,
            max_equality_residual=1e-8,
            scene="table_2",
        )
