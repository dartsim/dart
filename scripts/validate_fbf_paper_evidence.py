#!/usr/bin/env python3
"""Validate the exact-Coulomb FBF paper-evidence ledger.

The manifest is intentionally stricter than a generic JSON schema.  It fixes
the canonical paper and video coverage, checks every referenced repository
artifact, and prevents a requirement from being called complete unless every
declared deliverable is present, validated, and fallback-free.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shlex
import shutil
import statistics
import sys
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.fbf_paper_evidence/v1"
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "docs"
    / "dev_tasks"
    / "fbf_exact_coulomb_friction"
    / "paper-evidence-manifest.json"
)

VIDEO_SEGMENTS = {
    "video.01_title": (0, 2),
    "video.02_backspin": (2, 24),
    "video.03_incline": (24, 35),
    "video.04_turntable": (35, 50),
    "video.05_painleve": (50, 60),
    "video.06_card_house": (60, 67),
    "video.07_arch_25": (67, 74),
    "video.08_arch_101": (74, 80),
    "video.09_closing": (80, 82),
}

CANONICAL_REQUIREMENT_IDS = (
    "teaser",
    *(f"fig.{number:02d}" for number in range(1, 11)),
    *(f"table.{number:02d}" for number in range(1, 8)),
    "large_scale.arch_101",
    "large_scale.card_house_10",
    *VIDEO_SEGMENTS,
)

ALLOWED_STATUSES = {"not_started", "partial", "blocked", "complete"}
ALLOWED_KINDS = {
    "figure",
    "table",
    "large_scale",
    "teaser",
    "video_segment",
}
ALLOWED_DELIVERABLES = {
    "approved_golden",
    "capture_sidecar",
    "citation_record",
    "claim_map",
    "comparison_plot",
    "exact_fixture",
    "external_baseline",
    "golden_diff",
    "outcome_report",
    "performance_csv",
    "performance_report",
    "raw_data",
    "residual_plot",
    "still_image",
    "trace_csv",
    "video_clip",
}

# The v1 manifest is a closed claim ledger, not an open-ended JSON annotation
# document.  Unknown keys are dangerous here because a reviewer can reasonably
# read an ignored field such as ``paper_parity_complete: true`` as a validated
# claim.  Keep every claim-bearing object closed and update these contracts in
# the same change as any deliberate schema extension.
MANIFEST_KEYS = {
    "capture_profiles",
    "completion_rule",
    "configuration_profiles",
    "current_truth",
    "overall_status",
    "requirements",
    "schema_version",
    "snapshot_date",
    "source_audit",
    "sources",
    "task_id",
}
SOURCE_CATALOG_KEYS = {
    "author_code",
    "paper",
    "pr3374",
    "pr3377",
    "project_page",
    "video",
}
SOURCE_RECORD_KEYS = {
    "paper": {"retrieved", "title", "url", "venue"},
    "project_page": {"code_status", "locator", "retrieved", "url"},
    "author_code": {
        "availability_boundary",
        "branch",
        "commit",
        "license",
        "license_sha256",
        "readme_sha256",
        "retrieved",
        "scene_source_sha256",
        "scope",
        "solver_source_sha256",
        "url",
    },
    "video": {"duration_seconds", "retrieved", "sha256", "timeline_basis", "url"},
    "pr3374": {
        "integration_detail",
        "live_state",
        "merge_commit",
        "merged_at",
        "remaining_limit",
        "scope",
        "status",
        "upstream_head",
        "url",
    },
    "pr3377": {
        "base_state",
        "ci_snapshot",
        "live_state",
        "observed",
        "remaining_limit",
        "scope",
        "status",
        "upstream_head",
        "url",
    },
}
AUTHOR_SCENE_SOURCE_KEYS = {
    "backspin-ball/run.py",
    "card-house/run.py",
    "cube-on-incline/run.py",
    "masonry-arch/run.py",
    "painleve/run.py",
    "turntable/run.py",
}
AUTHOR_SOLVER_SOURCE_KEYS = {
    "fbf_solver/config.py",
    "fbf_solver/inner/block_gs.py",
    "fbf_solver/solver_fbf.py",
}
SOURCE_AUDIT_KEYS = {
    "combined_video_sha256",
    "coverage_seconds",
    "local_capture_status",
    "paper_pdf_sha256",
    "segment_count",
    "teaser_sha256",
}
CURRENT_TRUTH_KEYS = {
    "author_card_house_5_construction_only_v1",
    "backspin_visual_v3_nonpaper",
    "card_house_manifold_sensitivity_v2_nonpaper",
    "cpu_evidence_schema",
    "current_small_paper_cpu_v1",
    "historical_scale1_literal_arch_diagnostic",
    "incline_visual_v1_nonpaper",
    "literal_arch_101_v1_nonpaper",
    "literal_wedge_collision_audit",
    "literal_wedge_crown_impact_v1_nonpaper",
    "literal_wedge_exact_dynamics_nonpaper",
    "literal_wedge_visual_nonpaper",
    "overall",
    "painleve_proxy_visual_v1_nonpaper",
    "paper_comparability",
    "prior_source_external_evidence",
    "prior_source_strict_card_paper_cpu",
    "production_arch_geometry",
    "turntable_author_visual_v1_nonpaper",
    "visual_evidence_workflow",
}
CURRENT_TRUTH_RECORD_KEYS = {
    "incline_visual_v1_nonpaper": set(
        "accepted_at_cap actual_simulator aggregate_count_projection_equivalent "
        "aggregate_count_projection_sha256 aggregate_counts_only "
        "approved_source_golden approved_source_golden_diff artifact_count "
        "artifact_hashes artifact_valid automated_semantic_outcome_validated "
        "boxed_lcp_fallbacks bundle capture_contacts_per_post_initial_step "
        "capture_max_residual capture_sidecar_deliverable_validated "
        "capture_trace_contact_count_equivalent captured_frames claim_boundary "
        "claim_scope claim_valid exact_attempts exact_failures exact_solves "
        "external_solver_parity full_friction_sweep full_state_trace_equivalence "
        "generated_imagery manual_inspected manual_visual_outcome_validated "
        "maximum_penetration_proven mp4_dimensions mp4_fps mp4_frames "
        "panel_dimensions paper_comparable paper_parity "
        "paper_reference_contact_count_match per_cell_trace_equivalence "
        "physical_outcome_valid realtime_verdict requirement_ids "
        "slide_downhill_displacement_m slide_expected_downhill_displacement_m "
        "slide_final_downhill_speed_m_s slide_minus_stick_m solver_contract_valid "
        "status steps stick_downhill_displacement_m stick_maximum_speed_m_s "
        "timing_verdict trace_aggregate_contacts_per_post_initial_step "
        "trace_equivalence_to_rendered_demo trace_max_residual "
        "trace_rows_per_scenario tracked_trace_continuous_contact_proven".split()
    ),
    "backspin_visual_v3_nonpaper": set(
        "accepted_at_cap actual_simulator approved_source_golden artifact_count "
        "artifact_hashes artifact_valid automated_semantic_outcome_validated "
        "boxed_lcp_fallbacks bundle capture_max_residual "
        "capture_sidecar_deliverable_validated captured_frames claim_boundary "
        "claim_valid contact_free_post_initial_steps continuous_contact_proven "
        "exact_attempts exact_failures exact_solves external_solver_parity "
        "final_vx_m_s final_x_m first_negative_vx_step generated_imagery "
        "gif_dimensions gif_fps gif_frames manual_inspected "
        "manual_visual_outcome_validated maximum_forward_step maximum_forward_x_m "
        "mp4_dimensions mp4_fps mp4_frames panel_dimensions paper_comparable "
        "paper_parity physical_outcome_valid realtime_verdict "
        "signed_angular_direction_proven solver_contract_valid "
        "solver_projection_equivalent solver_projection_sha256 status steps "
        "strict_rigid_body_rest_proven timing_verdict "
        "trace_equivalence_to_rendered_demo trace_max_residual trace_rows "
        "warm_starts".split()
    ),
    "prior_source_strict_card_paper_cpu": set(
        "accepted_cap_steps artifact_hashes boxed_lcp_fallbacks bundle "
        "completed_steps contact_range convergence_verdict exact_fbf_failures "
        "mean_step_ms_before_failure original_output_path "
        "outer_cap_per_exact_group requested_steps source_status terminal_residual "
        "terminal_step_fbf_iterations timing_verdict tolerance "
        "trace_executable_sha256 trajectory_verdict".split()
    ),
    "card_house_manifold_sensitivity_v2_nonpaper": set(
        "artifact_hashes bundle claim_boundary compact "
        "comparison_artifact_integrity_valid "
        "directional_contact_multiplicity_hypothesis_supported four_point_planar "
        "four_point_planar_minus_compact manifold_is_only_intended_factor "
        "paper_parity physical_verdict protocol runner runtime_provenance status "
        "timing_used_in_verdict timing_verdict".split()
    ),
    "author_card_house_5_construction_only_v1": set(
        "actual_simulator artifact_count artifact_hashes "
        "artifact_index_exclusions artifact_valid author_commit "
        "author_repository author_tree bundle canonical_fig06_deliverable "
        "canonical_video06_deliverable capture_steps cards claim_boundary "
        "claim_scope completed_steps configuration_port_valid construction_only "
        "contract_payload_sha256 cubes durable_still_path durable_still_sha256 "
        "dynamics_executed fig06_parity generated_imagery indexed_artifact_count "
        "levels manual_configuration_inspection_valid panel_steps paper_comparable "
        "paper_timing_valid physical_outcome_valid requirement_ids solver_valid "
        "source_identity_sha256 status total_steps trajectory_valid "
        "video06_parity".split()
    ),
    "historical_scale1_literal_arch_diagnostic": set(
        "cold_outer_iterations historical_only local_diagonal_seed_outer_iterations "
        "max_2000_outer_continuation paper_comparable "
        "standard_200_outer_continuation".split()
    ),
    "production_arch_geometry": set(
        "author_faithful_literal_wedge kind stones_101_full_contact_cap_reached "
        "stones_101_gui_contacts stones_101_uncapped_contacts stones_25_gui_contacts "
        "stones_25_natural_contacts".split()
    ),
    "literal_arch_101_v1_nonpaper": set(
        "artifact_hashes artifact_valid boxed_lcp_fallbacks bundle claim_boundary "
        "colliding_body_pairs collision_probe colored_schedule contacts "
        "emitted_steps exact_failures first_failed_step historical_invalid_bundles "
        "historical_superseded_bundles max_arch_body_displacement_m "
        "min_arch_body_orientation_alignment normalized_fingerprint_sha256 "
        "outer_iterations protocol protocol_contract_sha256 requested_steps runner "
        "runtime_provenance scene_graph_evidence_scope standing_claim_passed status "
        "terminal_residual terminal_status timing_evidence_eligible".split()
    ),
    "literal_wedge_collision_audit": set(
        "collision_only dynamic_claim exact_polyhedral_inertia inertia "
        "paper_25_stone_100_contact_contract stones_25_nominal_adjacent_pairs "
        "stones_25_pairs_with_closure_and_ground".split()
    ),
    "literal_wedge_exact_dynamics_nonpaper": set(
        "accepted_caps artifact_hashes boxed_lcp_fallbacks bundle claim_boundary "
        "closure_meters colliding_body_pairs_each_step collision_frontend "
        "colored_schedule contacts_each_step evidence_binary_sha256 exact_failures "
        "four_threads max_arch_body_displacement_from_initial_m max_residual "
        "measured_steps_per_thread_count measured_trajectories_per_thread_count "
        "measured_workload_fingerprint_sha256 "
        "min_arch_body_orientation_alignment_from_initial one_thread "
        "paper_comparable physical_outcome_valid runtime_provenance scene_contract "
        "solver_options steps_per_trajectory validated_speedup "
        "warmup_trajectories_per_thread_count".split()
    ),
    "painleve_proxy_visual_v1_nonpaper": set(
        "actual_simulator approved_source_golden artifact_count artifact_hashes "
        "automated_semantic_outcome_validated boxed_lcp_fallbacks bundle "
        "capture_max_residual capture_sidecar_deliverable_validated claim_boundary "
        "exact_failures external_solver_parity generated_imagery group_dimensions "
        "group_video_fps group_video_frames manual_visual_outcome_validated "
        "member_frames mu05 mu055 paper_comparable paper_parity realtime_verdict "
        "shorter_pre_tumble_margin_m status steps strict_rigid_body_rest_proven "
        "timing_verdict trace_equivalence_to_rendered_demo trace_max_residual "
        "trace_rows_per_scenario".split()
    ),
    "turntable_author_visual_v1_nonpaper": set(
        "actual_simulator approved_source_golden artifact_count artifact_hashes "
        "artifact_valid author_configuration_port author_source_commit "
        "author_source_pinned automated_semantic_outcome_validated bundle "
        "claim_boundary claim_scope core_solver_contact_projection_equivalent "
        "cross_lane_substitution_allowed external_solver_parity generated_imagery "
        "group_dimensions group_panel_dimensions group_video_fps group_video_frames "
        "manual_inspected manual_visual_outcome_validated member_frames "
        "paper_comparable paper_cpu_native_all_solver_contract_valid paper_parity "
        "physical_outcome_valid realtime_verdict render_binding_lane "
        "separate_diagnostic_lane solver_contract_valid "
        "solver_projection_equivalent source_order_validated status steps "
        "timing_verdict trace_equivalence_to_rendered_demo "
        "trace_rows_per_scenario".split()
    ),
    "literal_wedge_visual_nonpaper": set(
        "artifact_hashes boxed_lcp_fallbacks bundle claim_boundary "
        "colliding_body_pairs_each_step contacts_each_step distinct_frames "
        "exact_attempts exact_failures exact_solves final_artifacts_valid "
        "frame_dimensions manual_inspected max_residual paper_parity "
        "playback_acceleration_factor playback_duration_seconds projectile_present "
        "simulation_duration_seconds status steps trace_mismatches "
        "trace_rows_compared video_codec video_fps video_frames".split()
    ),
    "literal_wedge_crown_impact_v1_nonpaper": set(
        "accepted_caps artifact_hashes boxed_lcp_fallbacks bundle claim_boundary "
        "exact_failures failed_preregistered_gates final_max_arch_displacement_m "
        "final_max_far_field_displacement_m finite_state "
        "first_projectile_arch_contact_step first_projectile_ground_contact_step "
        "impact_claim_passed normalized_fingerprint_sha256 paper_parity "
        "preregistration_contract_sha256 projectile_count runner runtime_provenance "
        "standing_prefix_fields_compared standing_prefix_mismatches "
        "standing_prefix_steps standing_reference_fields "
        "standing_reference_mismatches status steps_completed "
        "worst_exact_residual".split()
    ),
    "paper_comparability": {"blockers", "paper_code_status", "verdict"},
    "cpu_evidence_schema": {
        "comparison_guards",
        "paper_cpu_requires",
        "policy",
        "realtime_threshold_seconds",
        "trace_contracts",
        "version",
    },
    "current_small_paper_cpu_v1": set(
        "artifact_count artifact_hashes bundle claim_boundary collision_frontend "
        "contract cpu_list evidence_binary_sha256 overall_strict_matrix_valid "
        "paper_comparable physical_classifier_passes "
        "realtime_contract_valid_scenarios repetitions_per_scenario rows "
        "runtime_provenance scenario_count schema_version status "
        "strict_solver_valid_scenarios".split()
    ),
    "visual_evidence_workflow": {
        "completion_attestation_status",
        "contract",
        "repository_artifact_status",
        "runner",
        "schedule_count",
    },
    "prior_source_external_evidence": set(
        "long_visual_blockers long_visual_sidecars_valid multicore "
        "painleve_repository_current_v1 paper_cpu_card_fail_fast "
        "paper_cpu_small_matrix promotion_status small_visual_post_review "
        "source_status turntable_author_repository_current_v1".split()
    ),
}
CURRENT_TRUTH_NESTED_OBJECT_KEYS = {
    "card_house_manifold_sensitivity_v2_nonpaper.compact": set(
        "accepted_cap_exact_groups boxed_lcp_fallbacks contact_range emitted_steps "
        "exact_attempts exact_failures mean_contacts mean_pair_multiplicity "
        "pair_identity_transition_rows pair_multiplicity_transition_rows pair_range "
        "process_exit_class strict_success_rows strict_trajectory_valid "
        "terminal_last_group_residual terminal_last_group_status".split()
    ),
    "card_house_manifold_sensitivity_v2_nonpaper.four_point_planar": set(
        "accepted_cap_exact_groups boxed_lcp_fallbacks contact_range emitted_steps "
        "exact_attempts exact_failures mean_contacts mean_pair_multiplicity "
        "pair_identity_transition_rows pair_multiplicity_transition_rows pair_range "
        "process_exit_class strict_success_rows strict_trajectory_valid "
        "terminal_last_group_residual terminal_last_group_status".split()
    ),
    "card_house_manifold_sensitivity_v2_nonpaper.four_point_planar_minus_compact": set(
        "accepted_cap_exact_groups exact_attempts mean_contacts "
        "mean_pair_multiplicity pair_identity_transition_rows "
        "pair_multiplicity_transition_rows terminal_last_group_residual".split()
    ),
    "card_house_manifold_sensitivity_v2_nonpaper.runtime_provenance": set(
        "identity_rechecks ldd_tool_sha256 resolved_build_libdart_sha256 scope "
        "taskset_tool_sha256 trace_ldd_map_sha256 "
        "trace_resolved_regular_shared_libraries".split()
    ),
    "literal_arch_101_v1_nonpaper.colored_schedule": {
        "colors",
        "manifolds",
        "max_manifolds_per_color",
        "participants",
    },
    "literal_arch_101_v1_nonpaper.scene_graph_evidence_scope": {
        "constructed_initial_scene",
        "dynamic_trace",
        "promotion_boundary",
    },
    "literal_arch_101_v1_nonpaper.collision_probe": set(
        "adjacent_stone_pairs contacts dynamic_pair_identity_evidence "
        "genuine_contact_graph nonadjacent_stone_pairs nonfinite_contacts "
        "repeat_count repeated_collision_stable scope springer_ground_pairs "
        "unexpected_ground_pairs unique_pairs".split()
    ),
    "literal_arch_101_v1_nonpaper.runtime_provenance": set(
        "collision_probe_ldd_map_sha256 "
        "collision_probe_resolved_regular_shared_libraries identity_rechecks "
        "ldd_tool_sha256 resolved_build_libdart_sha256 scope taskset_tool_sha256 "
        "trace_ldd_map_sha256 trace_resolved_regular_shared_libraries".split()
    ),
    "literal_arch_101_v1_nonpaper.historical_invalid_bundles[]": {
        "bundle",
        "reason",
    },
    "literal_arch_101_v1_nonpaper.historical_superseded_bundles[]": {
        "bundle",
        "reason",
    },
    "literal_wedge_exact_dynamics_nonpaper.colored_schedule": {
        "colors",
        "manifolds",
        "max_manifolds_per_color",
    },
    "literal_wedge_exact_dynamics_nonpaper.solver_options": set(
        "diagonal_seed inner_fixed_sweeps matrix_free_seed outer_iterations "
        "outer_relaxation step_size_persistence step_size_scale".split()
    ),
    "literal_wedge_exact_dynamics_nonpaper.one_thread": {
        "all_steps_realtime",
        "max_ms",
        "mean_ms",
        "mean_realtime",
        "median_ms",
        "p95_ms",
    },
    "literal_wedge_exact_dynamics_nonpaper.four_threads": {
        "all_steps_realtime",
        "max_ms",
        "mean_ms",
        "mean_realtime",
        "median_ms",
        "p95_ms",
    },
    "literal_wedge_exact_dynamics_nonpaper.runtime_provenance": set(
        "executed_tool_identity_rechecks identity_recheck_stage ldd_tool_sha256 "
        "resolved_build_libdart_sha256 resolved_regular_file_count "
        "resolved_regular_files_sha256 scope taskset_tool_sha256".split()
    ),
    "painleve_proxy_visual_v1_nonpaper.mu05": set(
        "exact_solves final_up_z final_x_m final_z_m first_tumble "
        "max_tail_horizontal_speed_m_s tail_horizontal_drift_m tail_up_z_range "
        "tail_z_excursion_m".split()
    ),
    "painleve_proxy_visual_v1_nonpaper.mu055": set(
        "exact_solves final_up_z final_x_m final_z_m first_tumble_step "
        "first_tumble_time_s first_tumble_x_m max_tail_horizontal_speed_m_s "
        "tail_horizontal_drift_m tail_up_z_range tail_z_excursion_m".split()
    ),
    "literal_wedge_crown_impact_v1_nonpaper.runtime_provenance": set(
        "identity_rechecks ldd_tool_sha256 resolved_build_libdart_sha256 scope "
        "selected_cpu taskset_tool_sha256 trace_ldd_map_sha256 "
        "trace_resolved_regular_shared_libraries".split()
    ),
    "current_small_paper_cpu_v1.rows": set(
        "backspin incline_mu_0_4 incline_mu_0_5 painleve_mu_0_5 "
        "painleve_mu_0_55 turntable_mu_0_2_omega_2 "
        "turntable_mu_0_2_omega_5 turntable_mu_0_5_omega_2 "
        "turntable_mu_0_5_omega_5".split()
    ),
    **{
        f"current_small_paper_cpu_v1.rows.{scenario}": set(
            "failed_processes max_iterations_accepted_rows max_residual "
            "max_step_ms mean_step_ms median_step_ms p95_step_ms "
            "physical_outcome_valid residual_pass_fraction strict_solver_valid".split()
        )
        for scenario in (
            "backspin",
            "incline_mu_0_4",
            "incline_mu_0_5",
            "painleve_mu_0_5",
            "painleve_mu_0_55",
            "turntable_mu_0_2_omega_2",
            "turntable_mu_0_2_omega_5",
            "turntable_mu_0_5_omega_2",
            "turntable_mu_0_5_omega_5",
        )
    },
    "current_small_paper_cpu_v1.runtime_provenance": set(
        "executed_tool_identity_rechecks identity_recheck_stage ldd_tool_sha256 "
        "resolved_build_libdart_sha256 resolved_regular_file_count "
        "resolved_regular_files_sha256 scope taskset_tool_sha256".split()
    ),
    "prior_source_external_evidence.paper_cpu_small_matrix": set(
        "durability local_realtime_contract_rows paper_comparable_rows path reason "
        "repetitions_per_row rows".split()
    ),
    "prior_source_external_evidence.paper_cpu_card_fail_fast": set(
        "completed_steps durability mean_step_ms original_output_path path "
        "requested_steps terminal_residual verdict".split()
    ),
    "prior_source_external_evidence.multicore": {"bundle", "reason", "verdict"},
    "prior_source_external_evidence.small_visual_post_review": set(
        "dynamic_schedules manual_inspection painleve_mp4_sha256 paper_comparable "
        "path recorded_binary_verification recorded_demo_binary_sha256 "
        "repository_deliverable static_artifact_schedules summary "
        "turntable_mp4_sha256 validated_dynamic_solver_schedules".split()
    ),
    "prior_source_external_evidence.painleve_repository_current_v1": set(
        "artifact_count durability manual_visual_outcome paired_clip_sha256 "
        "paired_panel_sha256 paper_comparable path repository_deliverable "
        "trace_corroboration trace_equivalence_to_rendered_demo".split()
    ),
    "prior_source_external_evidence.turntable_author_repository_current_v1": set(
        "artifact_count durability group_clip_sha256 group_panel_sha256 "
        "manual_visual_outcome paper_comparable paper_cpu_native_lane path "
        "render_binding_lane repository_deliverable "
        "trace_equivalence_to_rendered_demo".split()
    ),
}

# These maps are not extensibility escape hatches. Their exact membership and
# values are already tied to bundle-specific target constants by dedicated
# validators, so the generic manifest-shape walker only enforces object type.
CURRENT_TRUTH_DELEGATED_OBJECT_PATHS = {
    f"{record}.artifact_hashes"
    for record, keys in CURRENT_TRUTH_RECORD_KEYS.items()
    if "artifact_hashes" in keys
} | {"author_card_house_5_construction_only_v1.source_identity_sha256"}
CURRENT_TRUTH_OBJECT_KEYS = {
    **CURRENT_TRUTH_RECORD_KEYS,
    **CURRENT_TRUTH_NESTED_OBJECT_KEYS,
}
REQUIREMENT_KEYS = {
    "blockers",
    "capture_plan",
    "claim",
    "commands",
    "configuration",
    "current_evidence",
    "deliverables",
    "expected_behavior",
    "fallback_count",
    "id",
    "kind",
    "required_deliverables",
    "source",
    "status",
    "title",
}
REQUIREMENT_SOURCE_KEYS = {"locator", "url"}
VIDEO_REQUIREMENT_SOURCE_KEYS = {
    "end_seconds",
    "locator",
    "start_seconds",
    "url",
}
REQUIREMENT_CONFIGURATION_KEYS = {
    **{requirement_id: {"profile"} for requirement_id in CANONICAL_REQUIREMENT_IDS},
    "fig.01": {"profile", "required_sweep"},
    "fig.02": {"profile", "snapshot_cells"},
    "fig.03": {"orientation_cue", "profile"},
    "fig.09": {"panels", "profile"},
    "large_scale.arch_101": {"performance_profiles", "profile"},
    "large_scale.card_house_10": {
        "levels_override",
        "performance_profiles",
        "profile",
    },
    "video.02_backspin": {"orientation_cue", "profile"},
    "video.03_incline": {"profile", "video_cells"},
}
REQUIREMENT_CONFIGURATION_LIST_FIELDS = {
    "panels",
    "performance_profiles",
    "snapshot_cells",
    "video_cells",
}
CURRENT_EVIDENCE_KEYS = {"path", "supports"}
CAPTURE_PLAN_KEYS = {"blockers", "commands", "profile", "shots"}
VALIDATED_DELIVERABLE_KEYS = {"kind", "path", "sha256", "validated"}
STALE_DELIVERABLE_KEYS = {"kind", "path", "stale_reason", "validated"}

CAPTURE_PROFILE_KEYS = {
    "performance",
    "plot_comparison",
    "still_comparison",
    "video",
}
CAPTURE_PROFILE_RECORD_KEYS = {"gate", "pr3374_status", "toolchain"}

# Configuration profiles contain source/paper contracts and therefore use a
# closed recursive key shape as well.  Scalar/list values remain free to change
# when evidence is refreshed; only object membership is fixed by schema v1.
CONFIGURATION_PROFILE_OBJECT_KEYS = {
    "teaser": {"claim_scope", "paper_setup"},
    "incline": {
        "analytic_transition_mu",
        "comparison_solvers",
        "duration_seconds",
        "friction_sweep",
        "reported_quantity",
        "slope_degrees",
        "slope_radians",
        "time_step_seconds",
    },
    "backspin": {
        "analytic_limit_angular_velocity_rad_s",
        "analytic_limit_linear_velocity_m_s",
        "body",
        "comparison_solvers",
        "friction_coefficient",
        "initial_angular_velocity_rad_s",
        "initial_linear_velocity_m_s",
        "radius_m",
        "snapshot_seconds",
        "time_step_seconds",
    },
    "turntable": {
        "cells",
        "comparison_boundary",
        "control",
        "cube",
        "horizon_seconds",
        "initial_drop_m",
        "initial_gap_m",
        "initial_radial_position_m",
        "ramp_duration_seconds",
        "ramp_profile",
        "rendering",
        "settle_duration_seconds",
        "source",
        "support",
        "time_step_seconds",
    },
    "turntable.support": {"height_m", "mobile", "radius_m", "shape"},
    "turntable.cube": {"density_kg_m3", "edge_m", "mass_kg"},
    "turntable.cells[]": {"mu", "omega_rad_s", "outcome"},
    "painleve": {"body", "cells", "paper_unspecified"},
    "painleve.cells[]": {"fbf_kamino", "mu", "mujoco"},
    "card_house_26": {
        "comparison_solvers",
        "friction_coefficient",
        "levels",
        "paper_contact_count_for_timing",
        "paper_precision",
        "plates",
        "time_step_seconds",
        "timeline",
    },
    "author_card_house_5_default": {
        "cards",
        "contact",
        "cubes",
        "evidence_scope",
        "provenance",
        "scene",
        "schedule",
        "solver",
    },
    "author_card_house_5_default.provenance": {
        "author_commit",
        "author_repository",
        "author_tree",
    },
    "author_card_house_5_default.cards": {
        "bridge_angle_degrees",
        "bridge_size_m",
        "density_kg_m3",
        "lean_from_horizontal_degrees",
        "lean_from_vertical_degrees",
        "lean_size_m",
        "mass_kg",
        "source_mobile",
        "tent_half_gap_m",
        "tent_height_m",
        "tent_width_m",
    },
    "author_card_house_5_default.contact": {"friction", "gap_m"},
    "author_card_house_5_default.cubes": {
        "density_kg_m3",
        "initial_height_m",
        "mass_kg",
        "size_m",
        "source_initially_kinematic",
    },
    "author_card_house_5_default.scene": {
        "bridges",
        "cards",
        "cubes",
        "demo_scene_id",
        "leaning_cards",
        "levels",
    },
    "author_card_house_5_default.schedule": {
        "display_time_step_seconds",
        "release_frame",
        "release_substep",
        "runtime_time_step_seconds",
        "substeps_per_frame",
        "total_frames",
        "total_substeps",
    },
    "author_card_house_5_default.solver": {
        "adaptive_gamma",
        "armijo_max_backtracks",
        "armijo_rho_high",
        "armijo_shrink",
        "baumgarte_erp",
        "gamma",
        "gamma_c",
        "gamma_max",
        "inner_gs_sweeps",
        "inner_max_iter",
        "inner_solver",
        "inner_tol",
        "max_contacts",
        "max_outer",
        "outer_tol",
        "plateau_patience",
        "plateau_rtol",
        "project_after_correction",
        "residual_check_interval",
        "termination_residual",
        "termination_tol",
        "type",
        "warm_start",
        "warm_start_gamma_cap",
        "warm_start_match_radius",
        "warm_start_max_age",
        "warm_start_normal_cosine",
    },
    "masonry_arch_25": {
        "comparison_solvers",
        "dynamic_interior_stones",
        "friction_coefficient_for_contact_rich_results",
        "local_wedge_collision_audit",
        "paper_contact_count_for_timing",
        "paper_precision",
        "paper_unspecified",
        "pinned_end_stones",
        "production_dynamics",
        "timeline",
        "voussoirs",
    },
    "masonry_arch_25.production_dynamics": {
        "geometry",
        "gui_reduced_contacts",
        "literal_tapered_voussoirs",
        "natural_contacts",
    },
    "masonry_arch_25.local_wedge_collision_audit": {
        "collision_only",
        "detector",
        "dynamic_claim",
        "exact_uniform_prism_inertia",
        "exact_volume_total_m3",
        "friction_coefficient",
        "mobile_interior_stones",
        "nominal_adjacent_pairs",
        "pairs_with_closure_and_ground",
        "paper_100_contact_contract",
        "pinned_springers",
        "repeat_count",
        "repeated_collision_stable",
    },
    "masonry_arch_101": {
        "friction_coefficient",
        "local_wedge_collision_audit",
        "outcome",
        "paper_precision",
        "paper_unspecified",
        "production_dynamics",
        "voussoirs",
    },
    "masonry_arch_101.production_dynamics": {
        "full_contact_cap_reached",
        "geometry",
        "gui_reduced_contacts",
        "literal_tapered_voussoirs",
        "uncapped_natural_contacts",
    },
    "masonry_arch_101.local_wedge_collision_audit": {
        "collision_only",
        "current_pair_count_claim",
        "detector",
        "dynamic_claim",
        "exact_uniform_prism_inertia",
        "exact_volume_total_m3",
        "friction_coefficient",
        "mobile_interior_stones",
        "paper_25_stone_100_contact_contract",
        "pinned_springers",
        "repeat_count",
        "repeated_collision_stable",
    },
    "convergence": {
        "friction_coefficient_contact_rich",
        "inner_sweeps_arch",
        "inner_sweeps_cards",
        "outer_cap",
        "precision",
        "reported",
        "residual",
        "scenes",
        "tolerance",
    },
    "convergence.reported": {
        "arch_median",
        "arch_share_at_tolerance",
        "cards_median",
        "cards_share_at_tolerance",
    },
    "gamma_sweep": {"arch_rows", "card_rows", "friction_coefficient", "outer_cap"},
    "gamma_sweep.arch_rows[]": {"gamma", "median_residual"},
    "gamma_sweep.card_rows[]": {"gamma", "max_drop_m", "median_residual"},
    "table_1_contact_rich": {
        "execution",
        "friction_coefficient",
        "inner_sweeps",
        "outer_cap",
        "precision",
        "rows",
        "tolerance",
    },
    "table_1_contact_rich.inner_sweeps": {"arch", "cards"},
    "table_1_contact_rich.rows[]": {
        "contacts",
        "fbf_ms",
        "kamino_relative",
        "median_outer",
        "mujoco_relative",
        "scene",
    },
    "table_2_convergence": {"profile", "rows"},
    "table_2_convergence.rows[]": {
        "median_residual",
        "scene",
        "share_at_tolerance",
    },
    "table_3_outer_budget": {"rows", "standard_outer_cap", "ten_x_outer_cap"},
    "table_3_outer_budget.rows[]": {
        "median_iterations",
        "median_residual",
        "ms_per_frame",
        "run",
        "share_at_tolerance",
    },
    "table_4_gamma": {"metric", "profile"},
    "table_5_cpu": {"execution", "precision", "rows", "tolerance"},
    "table_5_cpu.rows[]": {
        "benchmark",
        "contacts",
        "fbf_ms",
        "inner",
        "kamino_relative",
        "mujoco_relative",
        "outer_median_p95",
    },
    "table_6_large_scale": {
        "execution",
        "friction_coefficient",
        "precision",
        "rows",
        "tolerance",
    },
    "table_6_large_scale.rows[]": {"fbf_ms", "kamino", "mujoco", "scene"},
    "table_7_gpu": {
        "device",
        "friction_coefficient",
        "metric",
        "rows",
        "tolerance",
    },
    "table_7_gpu.rows[]": {"fbf", "kamino", "mujoco", "scene"},
    "video_title": {"content", "simulation_configuration"},
    "video_closing": {"content", "simulation_configuration"},
}
CONFIGURATION_PROFILE_KEYS = {
    path for path in CONFIGURATION_PROFILE_OBJECT_KEYS if "." not in path
}

DELIVERABLE_EXTENSIONS = {
    "approved_golden": {".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"},
    "capture_sidecar": {".json"},
    "citation_record": {".json"},
    "claim_map": {".json"},
    "comparison_plot": {".jpeg", ".jpg", ".pdf", ".png", ".svg", ".webp"},
    "exact_fixture": {".cc", ".cpp", ".cxx", ".py"},
    "external_baseline": {".json"},
    "golden_diff": {".json"},
    "outcome_report": {".json"},
    "performance_csv": {".csv"},
    "performance_report": {".json"},
    "raw_data": {".csv", ".json"},
    "residual_plot": {".jpeg", ".jpg", ".pdf", ".png", ".svg", ".webp"},
    "still_image": {".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"},
    "trace_csv": {".csv"},
    "video_clip": {".gif", ".mov", ".mp4", ".webm"},
}

STRUCTURED_SCHEMAS = {
    "capture_sidecar": "dart.demos_headless_timeline/v1",
    "citation_record": "dart.fbf_citation_record/v1",
    "claim_map": "dart.fbf_paper_claim_map/v1",
    "external_baseline": "dart.fbf_external_baseline/v1",
    "golden_diff": "dart.image_diff/v1",
    "outcome_report": "dart.fbf_outcome_report/v1",
    "performance_report": "dart.fbf_performance_report/v1",
    "raw_data": "dart.fbf_raw_data/v1",
}

FULL_TRAJECTORY_SOLVER_SCHEMA = "dart.fbf_full_trajectory_solver_contract/v1"
PAPER_RESIDUAL_TOLERANCE = 1e-6
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")

CARD_V2_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "card_house_26_manifold_sensitivity_v2_r3"
)
CARD_V2_PROTOCOL = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/" "CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md"
)
CARD_V2_RUNNER = "scripts/run_fbf_card_manifold_sensitivity_v2.py"
VISUAL_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig07_arch25_literal"
)
PAINLEVE_V1_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig05_painleve_proxy_current_v1"
)
INCLINE_V1_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig01_02_incline_current_v1"
)
BACKSPIN_V3_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig03_backspin_current_v3"
)
TURNTABLE_V1_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig04_turntable_author_current_v1"
)
IMPACT_V7_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig07_arch25_literal_impact_v1_negative_final_v9"
)
IMPACT_V7_RUNNER = "scripts/run_fbf_literal_crown_impact_negative.py"
ARCH101_V5_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "fig08_arch101_literal_v1_negative_final_v6"
)
ARCH101_V1_PROTOCOL = "docs/dev_tasks/fbf_exact_coulomb_friction/LITERAL_ARCH_101_V1.md"
ARCH101_V1_RUNNER = "scripts/run_fbf_literal_arch101_v1.py"
TRACE_SOURCE = "tests/benchmark/integration/fbf_paper_trace.cpp"
COLLISION_PROBE_SOURCE = (
    "tests/benchmark/integration/fbf_paper_arch_wedge_collision_probe.cpp"
)
PRIOR_SOURCE_ARCHIVE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/dart_cpu_evidence/"
    "2026-07-12_prior_source_paper_cpu_card600_negative"
)
CPU_EVIDENCE_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/dart_cpu_evidence/"
    "2026-07-19_mark26_native25_colored_v10_archwide_pcore"
)
CURRENT_SMALL_CPU_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/dart_cpu_evidence/"
    "2026-07-19_current_source_paper_cpu_small_r7"
)
AUTHOR_CARD_HOUSE_V1_BUNDLE = (
    "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/"
    "card_house_author_5_construction_current_v1"
)
AUTHOR_CARD_HOUSE_V1_CAPTURE = "card_house_author_5_construction"
AUTHOR_CARD_HOUSE_V1_SCENE = "fbf_author_card_house_5_construction"
AUTHOR_CARD_HOUSE_V1_PROFILE = "author_card_house_5_default"
AUTHOR_CARD_HOUSE_V1_AUTHOR = {
    "repository": "https://github.com/matthcsong/fbf-sca-2026",
    "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
    "tree": "ffcdafb61adeda2239c8366d054b548b50d26685e",
    "card_house_run_blob": "35f33651bc9674a259071ac723e47755504152db",
    "card_house_run_py_sha256": (
        "18c58c85eaad865aeef480b46e880a52088f266b79c90226f624637221ee36f8"
    ),
    "fbf_config_py_sha256": (
        "88f3f9ffd758eccce8496f7897192587a05907109e313c7a86bcf8f9de8cc248"
    ),
    "solver_fbf_py_sha256": (
        "8ec32aa20bf8d6c1173ed6c7f3735e2926fbb4b5059ee2236e26ad27eb22f941"
    ),
}
AUTHOR_CARD_HOUSE_V1_SOURCE_CONFIGURATION = {
    "cards": {
        "bridge_angle_degrees": -1,
        "bridge_size_m": [2.5, 1.25, 0.04],
        "density_kg_m3": 200,
        "lean_from_horizontal_degrees": 65,
        "lean_from_vertical_degrees": 25,
        "lean_size_m": [0.04, 1.25, 2.5],
        "mass_kg": 25,
        "source_mobile": True,
        "tent_half_gap_m": 0.55,
        "tent_height_m": 2.41660616977186,
        "tent_width_m": 2.2,
    },
    "contact": {"friction": 0.8, "gap_m": 0.005},
    "cubes": {
        "density_kg_m3": 500,
        "initial_height_m": 13.0830308488593,
        "mass_kg": 256,
        "size_m": [0.8, 0.8, 0.8],
        "source_initially_kinematic": True,
    },
    "scene": {
        "bridges": 10,
        "cards": 40,
        "cubes": 4,
        "demo_scene_id": AUTHOR_CARD_HOUSE_V1_SCENE,
        "leaning_cards": 30,
        "levels": 5,
    },
    "schedule": {
        "display_time_step_seconds": 1.0 / 60.0,
        "release_frame": 400,
        "release_substep": 1600,
        "runtime_time_step_seconds": 1.0 / 240.0,
        "substeps_per_frame": 4,
        "total_frames": 800,
        "total_substeps": 3200,
    },
    "solver": {
        "adaptive_gamma": True,
        "armijo_max_backtracks": 8,
        "armijo_rho_high": 0.9,
        "armijo_shrink": 0.7,
        "baumgarte_erp": 0,
        "gamma": None,
        "gamma_c": 5,
        "gamma_max": 1000000,
        "inner_gs_sweeps": 10,
        "inner_max_iter": 200,
        "inner_solver": "block_gs",
        "inner_tol": 1.0e-6,
        "max_contacts": 4096,
        "max_outer": 200,
        "outer_tol": 1.0e-6,
        "plateau_patience": 5,
        "plateau_rtol": 0.01,
        "project_after_correction": False,
        "residual_check_interval": 5,
        "termination_residual": "coulomb_rel",
        "termination_tol": 1.0e-6,
        "type": "fbf_exact_coulomb",
        "warm_start": True,
        "warm_start_gamma_cap": 10000,
        "warm_start_match_radius": 0.02,
        "warm_start_max_age": 3,
        "warm_start_normal_cosine": 0.9,
    },
}
AUTHOR_CARD_HOUSE_V1_PROFILE_PAYLOAD = {
    "provenance": {
        "author_repository": AUTHOR_CARD_HOUSE_V1_AUTHOR["repository"],
        "author_commit": AUTHOR_CARD_HOUSE_V1_AUTHOR["commit"],
        "author_tree": AUTHOR_CARD_HOUSE_V1_AUTHOR["tree"],
    },
    "evidence_scope": "configuration_only",
    **AUTHOR_CARD_HOUSE_V1_SOURCE_CONFIGURATION,
}
AUTHOR_CARD_HOUSE_V1_ARTIFACT_PATHS = (
    "REPORT.md",
    "artifact-index.json",
    "capture-provenance.json",
    f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/construction-step-0.png",
    f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/metadata.json",
    f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/panel.compose.json",
    f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/panel.png",
    f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/timeline.json",
    "contract.json",
    "invocations.json",
    "manual-inspection.json",
    "metadata.json",
    "run-summary.json",
    "verification.json",
)
AUTHOR_CARD_HOUSE_V1_ARTIFACT_TARGETS = {
    relative: f"{AUTHOR_CARD_HOUSE_V1_BUNDLE}/{relative}"
    for relative in AUTHOR_CARD_HOUSE_V1_ARTIFACT_PATHS
}
AUTHOR_CARD_HOUSE_V1_SOURCE_TARGETS = {
    "configuration_spec": "examples/demos/scenes/FbfAuthorCardHouseSpec.hpp",
    "demo_binary": "build/default/cpp/Release/bin/dart-demos",
    "demo_cmake": "examples/demos/CMakeLists.txt",
    "demo_dependency_examples__demos__libdartDemosWamIk.so": (
        "build/default/cpp/Release/examples/demos/libdartDemosWamIk.so"
    ),
    "demo_dependency_lib__libdart-collision-bullet.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-collision-bullet.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-collision-ode.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-collision-ode.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-external-imgui.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-external-imgui.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-gui-osg.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-gui-osg.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-utils-urdf.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-utils-urdf.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-utils.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-utils.so.6.19.3"
    ),
    "demo_dependency_lib__libdart.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart.so.6.19.3"
    ),
    "demo_host_header": "examples/demos/DemoHost.hpp",
    "demo_host_source": "examples/demos/DemoHost.cpp",
    "demo_main": "examples/demos/main.cpp",
    "demo_source": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
    "ffmpeg_binary": ".pixi/envs/gazebo/bin/ffmpeg",
    "ffprobe_binary": ".pixi/envs/gazebo/bin/ffprobe",
    "finalizer": "scripts/finalize_fbf_author_card_house_construction.py",
    "finalizer_test": (
        "python/tests/unit/test_finalize_fbf_author_card_house_construction.py"
    ),
    "fixture_cmake": "tests/integration/CMakeLists.txt",
    "fixture_test": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
    "image_compose": "scripts/image_compose.py",
    "image_tools": "scripts/_image_tools.py",
    "image_verdict": "scripts/image_verdict.py",
    "python_binary": ".pixi/envs/default/bin/python3.14",
    "registry_source": "examples/demos/Registry.cpp",
    "scenes_header": "examples/demos/scenes/Scenes.hpp",
    "visual_runner": "scripts/run_fbf_visual_evidence.py",
    "visual_runner_test": "python/tests/unit/test_run_fbf_visual_evidence.py",
}
AUTHOR_CARD_HOUSE_V1_CAPTURE_ONLY_SOURCE_KEYS = {
    "demo_binary",
    "demo_dependency_examples__demos__libdartDemosWamIk.so",
    "demo_dependency_lib__libdart-collision-bullet.so.6.19.3",
    "demo_dependency_lib__libdart-collision-ode.so.6.19.3",
    "demo_dependency_lib__libdart-external-imgui.so.6.19.3",
    "demo_dependency_lib__libdart-gui-osg.so.6.19.3",
    "demo_dependency_lib__libdart-utils-urdf.so.6.19.3",
    "demo_dependency_lib__libdart-utils.so.6.19.3",
    "demo_dependency_lib__libdart.so.6.19.3",
    "ffmpeg_binary",
    "ffprobe_binary",
    "python_binary",
}
AUTHOR_CARD_HOUSE_V1_NEGATIVE_FLAGS = {
    "dynamics_executed": False,
    "trajectory_valid": False,
    "solver_valid": False,
    "physical_outcome_valid": False,
    "fig06_parity": False,
    "video06_parity": False,
    "paper_timing_valid": False,
    "paper_comparable": False,
    "canonical_fig06_deliverable": False,
    "canonical_video06_deliverable": False,
    "generated_imagery": False,
}
AUTHOR_CARD_HOUSE_V1_CLAIM_SCOPE = (
    "Current-source DART step-zero inspection of the author-pinned five-level "
    "card-house geometry, initial poses, and construction schedule."
)
AUTHOR_CARD_HOUSE_V1_CLAIM_BOUNDARY = (
    "Configuration only. The capture executes zero simulation substeps and does "
    "not validate release, standing, trajectory, solver behavior, contact "
    "dynamics, physical outcome, Fig. 6 parity, video parity, source renderer "
    "colors, paper timing, or performance. It is not a canonical Fig. 6 or video "
    "deliverable."
)

PRIOR_SOURCE_ARTIFACT_TARGETS = {
    "ARCHIVE_NOTE.md": f"{PRIOR_SOURCE_ARCHIVE}/ARCHIVE_NOTE.md",
    "REPORT.md": f"{PRIOR_SOURCE_ARCHIVE}/REPORT.md",
    "invocations.json": f"{PRIOR_SOURCE_ARCHIVE}/invocations.json",
    "metadata.json": f"{PRIOR_SOURCE_ARCHIVE}/metadata.json",
    "raw.csv": f"{PRIOR_SOURCE_ARCHIVE}/raw.csv",
    "raw/card_house_26_settle_projectile_full-n600-t1-rep001.csv": (
        f"{PRIOR_SOURCE_ARCHIVE}/raw/"
        "card_house_26_settle_projectile_full-n600-t1-rep001.csv"
    ),
    "raw/card_house_26_settle_projectile_full-n600-t1-rep001.stderr.txt": (
        f"{PRIOR_SOURCE_ARCHIVE}/raw/"
        "card_house_26_settle_projectile_full-n600-t1-rep001.stderr.txt"
    ),
    "summary.csv": f"{PRIOR_SOURCE_ARCHIVE}/summary.csv",
    "summary.json": f"{PRIOR_SOURCE_ARCHIVE}/summary.json",
    "artifact-index.json": f"{PRIOR_SOURCE_ARCHIVE}/artifact-index.json",
}

CPU_EVIDENCE_ARTIFACT_TARGETS = {
    "runner": "scripts/run_fbf_cpu_evidence.py",
    "runner_test": "python/tests/unit/test_run_fbf_cpu_evidence.py",
    "trace_source": TRACE_SOURCE,
    "artifact-index.json": f"{CPU_EVIDENCE_BUNDLE}/artifact-index.json",
    "metadata.json": f"{CPU_EVIDENCE_BUNDLE}/metadata.json",
    "invocations.json": f"{CPU_EVIDENCE_BUNDLE}/invocations.json",
    "raw.csv": f"{CPU_EVIDENCE_BUNDLE}/raw.csv",
    "summary.csv": f"{CPU_EVIDENCE_BUNDLE}/summary.csv",
    "summary.json": f"{CPU_EVIDENCE_BUNDLE}/summary.json",
    "REPORT.md": f"{CPU_EVIDENCE_BUNDLE}/REPORT.md",
}

CURRENT_SMALL_CPU_ARTIFACT_TARGETS = {
    "runner": "scripts/run_fbf_cpu_evidence.py",
    "runner_test": "python/tests/unit/test_run_fbf_cpu_evidence.py",
    "trace_source": TRACE_SOURCE,
    "artifact-index.json": f"{CURRENT_SMALL_CPU_BUNDLE}/artifact-index.json",
    "metadata.json": f"{CURRENT_SMALL_CPU_BUNDLE}/metadata.json",
    "invocations.json": f"{CURRENT_SMALL_CPU_BUNDLE}/invocations.json",
    "raw.csv": f"{CURRENT_SMALL_CPU_BUNDLE}/raw.csv",
    "summary.csv": f"{CURRENT_SMALL_CPU_BUNDLE}/summary.csv",
    "summary.json": f"{CURRENT_SMALL_CPU_BUNDLE}/summary.json",
    "REPORT.md": f"{CURRENT_SMALL_CPU_BUNDLE}/REPORT.md",
}

CARD_V2_ARTIFACT_TARGETS = {
    "runner": CARD_V2_RUNNER,
    "runner_test": "python/tests/unit/test_run_fbf_card_manifold_sensitivity_v2.py",
    "identity_helper_source": ARCH101_V1_RUNNER,
    "trace_contract_test": (
        "python/tests/unit/test_fbf_card_manifold_sensitivity_trace.py"
    ),
    "trace_source": TRACE_SOURCE,
    "compact/raw.csv": f"{CARD_V2_BUNDLE}/compact/raw.csv",
    "four_point_planar/raw.csv": (f"{CARD_V2_BUNDLE}/four_point_planar/raw.csv"),
    "invocation.json": f"{CARD_V2_BUNDLE}/invocation.json",
    "summary.json": f"{CARD_V2_BUNDLE}/summary.json",
    "comparison.json": f"{CARD_V2_BUNDLE}/comparison.json",
    "metadata.json": f"{CARD_V2_BUNDLE}/metadata.json",
    "artifact-index.json": f"{CARD_V2_BUNDLE}/artifact-index.json",
    "REPORT.md": f"{CARD_V2_BUNDLE}/REPORT.md",
}
VISUAL_ARTIFACT_TARGETS = {
    "driver": "scripts/run_fbf_literal_wedge_visual_capture.py",
    "driver_test": "python/tests/unit/test_run_fbf_literal_wedge_visual_capture.py",
    "current_trace_source": TRACE_SOURCE,
    "metadata.json": f"{VISUAL_BUNDLE}/metadata.json",
    "provenance.json": f"{VISUAL_BUNDLE}/provenance.json",
    "artifact-index.json": f"{VISUAL_BUNDLE}/artifact-index.json",
    "pending-metadata.json": f"{VISUAL_BUNDLE}/pending-metadata.json",
    "manual-inspection.json": f"{VISUAL_BUNDLE}/manual-inspection.json",
    "reference-fbf-paper-trace.csv": (f"{VISUAL_BUNDLE}/reference-fbf-paper-trace.csv"),
    "fig07_literal_wedge_stability.mp4": (
        f"{VISUAL_BUNDLE}/fig07_literal_wedge_stability.mp4"
    ),
    "fig07_literal_wedge_timeline.png": (
        f"{VISUAL_BUNDLE}/fig07_literal_wedge_timeline.png"
    ),
    "decoded/video_midpoint_t3.0.png": (
        f"{VISUAL_BUNDLE}/decoded/video_midpoint_t3.0.png"
    ),
}
PAINLEVE_V1_ARTIFACT_TARGETS = {
    "finalizer": "scripts/finalize_fbf_painleve_visual.py",
    "finalizer_test": "python/tests/unit/test_finalize_fbf_painleve_visual.py",
    "visual_runner": "scripts/run_fbf_visual_evidence.py",
    "visual_runner_test": "python/tests/unit/test_run_fbf_visual_evidence.py",
    "trace_source": TRACE_SOURCE,
    "fixture_source": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
    "demo_source": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
    "metadata.json": f"{PAINLEVE_V1_BUNDLE}/metadata.json",
    "artifact-index.json": f"{PAINLEVE_V1_BUNDLE}/artifact-index.json",
    "manual-inspection.json": f"{PAINLEVE_V1_BUNDLE}/manual-inspection.json",
    "trace-summary.json": f"{PAINLEVE_V1_BUNDLE}/trace-summary.json",
    "verification.json": f"{PAINLEVE_V1_BUNDLE}/verification.json",
    "invocations.json": f"{PAINLEVE_V1_BUNDLE}/invocations.json",
    "run-summary.json": f"{PAINLEVE_V1_BUNDLE}/run-summary.json",
    "REPORT.md": f"{PAINLEVE_V1_BUNDLE}/REPORT.md",
    "traces/painleve_mu_0_5.csv": (f"{PAINLEVE_V1_BUNDLE}/traces/painleve_mu_0_5.csv"),
    "traces/painleve_mu_0_55.csv": (
        f"{PAINLEVE_V1_BUNDLE}/traces/painleve_mu_0_55.csv"
    ),
    "painleve_mu05/metadata.json": (
        f"{PAINLEVE_V1_BUNDLE}/painleve_mu05/metadata.json"
    ),
    "painleve_mu05/timeline.json": (
        f"{PAINLEVE_V1_BUNDLE}/painleve_mu05/timeline.json"
    ),
    "painleve_mu05/panel.png": f"{PAINLEVE_V1_BUNDLE}/painleve_mu05/panel.png",
    "painleve_mu05/clip.mp4": f"{PAINLEVE_V1_BUNDLE}/painleve_mu05/clip.mp4",
    "painleve_mu055/metadata.json": (
        f"{PAINLEVE_V1_BUNDLE}/painleve_mu055/metadata.json"
    ),
    "painleve_mu055/timeline.json": (
        f"{PAINLEVE_V1_BUNDLE}/painleve_mu055/timeline.json"
    ),
    "painleve_mu055/panel.png": f"{PAINLEVE_V1_BUNDLE}/painleve_mu055/panel.png",
    "painleve_mu055/clip.mp4": f"{PAINLEVE_V1_BUNDLE}/painleve_mu055/clip.mp4",
    "groups/painleve/metadata.json": (
        f"{PAINLEVE_V1_BUNDLE}/groups/painleve/metadata.json"
    ),
    "groups/painleve/panel.png": (f"{PAINLEVE_V1_BUNDLE}/groups/painleve/panel.png"),
    "groups/painleve/clip.mp4": (f"{PAINLEVE_V1_BUNDLE}/groups/painleve/clip.mp4"),
}
INCLINE_V1_ARTIFACT_TARGETS = {
    "finalizer": "scripts/finalize_fbf_incline_visual.py",
    "finalizer_test": "python/tests/unit/test_finalize_fbf_incline_visual.py",
    "visual_runner": "scripts/run_fbf_visual_evidence.py",
    "visual_runner_test": "python/tests/unit/test_run_fbf_visual_evidence.py",
    "image_compose": "scripts/image_compose.py",
    "image_tools": "scripts/_image_tools.py",
    "image_verdict": "scripts/image_verdict.py",
    "trace_source": TRACE_SOURCE,
    "fixture_source": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
    "demo_source": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
    "demo_host_source": "examples/demos/DemoHost.cpp",
    "demo_host_header": "examples/demos/DemoHost.hpp",
    "registry_source": "examples/demos/Registry.cpp",
    "scenes_header": "examples/demos/scenes/Scenes.hpp",
    "metadata.json": f"{INCLINE_V1_BUNDLE}/metadata.json",
    "artifact-index.json": f"{INCLINE_V1_BUNDLE}/artifact-index.json",
    "manual-inspection.json": f"{INCLINE_V1_BUNDLE}/manual-inspection.json",
    "trace-summary.json": f"{INCLINE_V1_BUNDLE}/trace-summary.json",
    "verification.json": f"{INCLINE_V1_BUNDLE}/verification.json",
    "invocations.json": f"{INCLINE_V1_BUNDLE}/invocations.json",
    "run-summary.json": f"{INCLINE_V1_BUNDLE}/run-summary.json",
    "capture-provenance.json": f"{INCLINE_V1_BUNDLE}/capture-provenance.json",
    "REPORT.md": f"{INCLINE_V1_BUNDLE}/REPORT.md",
    "traces/incline_mu_0_4.csv": f"{INCLINE_V1_BUNDLE}/traces/incline_mu_0_4.csv",
    "traces/incline_mu_0_4.stderr.txt": (
        f"{INCLINE_V1_BUNDLE}/traces/incline_mu_0_4.stderr.txt"
    ),
    "traces/incline_mu_0_5.csv": f"{INCLINE_V1_BUNDLE}/traces/incline_mu_0_5.csv",
    "traces/incline_mu_0_5.stderr.txt": (
        f"{INCLINE_V1_BUNDLE}/traces/incline_mu_0_5.stderr.txt"
    ),
    "incline/metadata.json": f"{INCLINE_V1_BUNDLE}/incline/metadata.json",
    "incline/timeline.json": f"{INCLINE_V1_BUNDLE}/incline/timeline.json",
    "incline/panel.compose.json": f"{INCLINE_V1_BUNDLE}/incline/panel.compose.json",
    "incline/panel.png": f"{INCLINE_V1_BUNDLE}/incline/panel.png",
    "incline/clip.mp4": f"{INCLINE_V1_BUNDLE}/incline/clip.mp4",
}
for _incline_still_step in (0, 30, 60, 90, 120):
    _incline_still_key = f"incline/stills/step_{_incline_still_step:06d}.png"
    INCLINE_V1_ARTIFACT_TARGETS[_incline_still_key] = (
        f"{INCLINE_V1_BUNDLE}/{_incline_still_key}"
    )
INCLINE_V1_RECORDED_HASH_KEYS = {
    "trace_binary",
    "demo_binary",
    "ffmpeg_binary",
    "ffprobe_binary",
    "python_binary",
    "demo_dependency_examples__demos__libdartDemosWamIk.so",
    "demo_dependency_lib__libdart-collision-bullet.so.6.19.3",
    "demo_dependency_lib__libdart-collision-ode.so.6.19.3",
    "demo_dependency_lib__libdart-external-imgui.so.6.19.3",
    "demo_dependency_lib__libdart-gui-osg.so.6.19.3",
    "demo_dependency_lib__libdart-utils-urdf.so.6.19.3",
    "demo_dependency_lib__libdart-utils.so.6.19.3",
    "demo_dependency_lib__libdart.so.6.19.3",
    "trace_dependency_lib__libdart.so.6.19.3",
    "demo_source_query_payload",
    "trace_source_query_payload",
}
INCLINE_V1_SOURCE_IDENTITY_PATHS = {
    **INCLINE_V1_ARTIFACT_TARGETS,
    "trace_binary": (
        "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
    ),
    "demo_binary": "build/default/cpp/Release/bin/dart-demos",
    "ffmpeg_binary": ".pixi/envs/gazebo/bin/ffmpeg",
    "ffprobe_binary": ".pixi/envs/gazebo/bin/ffprobe",
    "python_binary": ".pixi/envs/default/bin/python3.14",
    "demo_dependency_examples__demos__libdartDemosWamIk.so": (
        "build/default/cpp/Release/examples/demos/libdartDemosWamIk.so"
    ),
    "demo_dependency_lib__libdart-collision-bullet.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-collision-bullet.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-collision-ode.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-collision-ode.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-external-imgui.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-external-imgui.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-gui-osg.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-gui-osg.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-utils-urdf.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-utils-urdf.so.6.19.3"
    ),
    "demo_dependency_lib__libdart-utils.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart-utils.so.6.19.3"
    ),
    "demo_dependency_lib__libdart.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart.so.6.19.3"
    ),
    "trace_dependency_lib__libdart.so.6.19.3": (
        "build/default/cpp/Release/lib/libdart.so.6.19.3"
    ),
}
INCLINE_V1_CAPTURE_RUNTIME_KEYS = {
    "demo_binary",
    "demo_dependency_examples__demos__libdartDemosWamIk.so",
    "demo_dependency_lib__libdart-collision-bullet.so.6.19.3",
    "demo_dependency_lib__libdart-collision-ode.so.6.19.3",
    "demo_dependency_lib__libdart-external-imgui.so.6.19.3",
    "demo_dependency_lib__libdart-gui-osg.so.6.19.3",
    "demo_dependency_lib__libdart-utils-urdf.so.6.19.3",
    "demo_dependency_lib__libdart-utils.so.6.19.3",
    "demo_dependency_lib__libdart.so.6.19.3",
    "demo_host_header",
    "demo_host_source",
    "demo_source",
    "ffmpeg_binary",
    "ffprobe_binary",
    "image_compose",
    "image_tools",
    "image_verdict",
    "python_binary",
    "registry_source",
    "scenes_header",
    "visual_runner",
}
INCLINE_V1_SOURCE_IDENTITY_KEYS = INCLINE_V1_CAPTURE_RUNTIME_KEYS | {
    "binary_source_bindings",
    "finalizer",
    "finalizer_test",
    "fixture_source",
    "trace_binary",
    "trace_dependency_lib__libdart.so.6.19.3",
    "trace_source",
    "visual_runner_test",
}
INCLINE_V1_BINDING_CONTRACTS = {
    "demo_source_binding": {
        "binary": "demo_binary",
        "source": "demo_source",
        "query_hash": "demo_source_query_payload",
        "argv_tail": [
            "--fbf-author-turntable-contract",
            "fbf_author_turntable_mu_0_2_omega_2",
        ],
        "role": "dart_demos",
    },
    "trace_source_binding": {
        "binary": "trace_binary",
        "source": "trace_source",
        "query_hash": "trace_source_query_payload",
        "argv_tail": [
            "--author-turntable-contract",
            "turntable_author_mu_0_2_omega_2",
            "dart_best",
        ],
        "role": "fbf_paper_trace",
    },
}
INCLINE_V1_REPRESENTATIVE_ARTIFACTS = (
    "incline/panel.png",
    "incline/clip.mp4",
    "incline/stills/step_000000.png",
    "incline/stills/step_000030.png",
    "incline/stills/step_000060.png",
    "incline/stills/step_000090.png",
    "incline/stills/step_000120.png",
)
INCLINE_V1_SCENARIOS = {
    "incline_mu_0_4": {
        "mu": 0.4,
        "trace": "traces/incline_mu_0_4.csv",
        "expected_outcome": "slide",
    },
    "incline_mu_0_5": {
        "mu": 0.5,
        "trace": "traces/incline_mu_0_5.csv",
        "expected_outcome": "stick",
    },
}
INCLINE_V1_CLAIM_SCOPE = (
    "Current-source DART two-second incline threshold: manual combined visual "
    "slide/stick classification independently supported by two tracked physical "
    "outcome traces and an additive exact-solve/fallback count-only capture/trace "
    "projection."
)
INCLINE_V1_CLAIM_BOUNDARY = (
    "The combined rendered scene and the two tracked CSV exporters use different "
    "placements. Only additive exact-solve/fallback counts are equivalent; capture "
    "contacts are eight per post-initial step while the traces total six, so contact "
    "counts are not equivalent. State, residual, status, warm-start, per-cell, and "
    "full-trace equivalence are not claimed. This is not a full friction sweep, "
    "paper contact-count match, maximum-penetration proof, author-scene or paper "
    "parity, faithful external-solver parity, approved source golden/diff, paper "
    "timing, or real-time evidence. The runner timeline is not promoted as a "
    "standalone manifest capture-sidecar deliverable."
)
BACKSPIN_V3_ARTIFACT_TARGETS = {
    "finalizer": "scripts/finalize_fbf_backspin_visual.py",
    "finalizer_test": "python/tests/unit/test_finalize_fbf_backspin_visual.py",
    "visual_runner": "scripts/run_fbf_visual_evidence.py",
    "visual_runner_test": "python/tests/unit/test_run_fbf_visual_evidence.py",
    "image_compose": "scripts/image_compose.py",
    "image_tools": "scripts/_image_tools.py",
    "image_verdict": "scripts/image_verdict.py",
    "trace_source": TRACE_SOURCE,
    "fixture_source": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
    "demo_source": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
    "backspin_checker_mesh": "data/obj/fbf_backspin_checker_sphere.obj",
    "backspin_checker_material": "data/obj/fbf_backspin_checker_sphere.mtl",
    "backspin_checker_texture": "data/obj/fbf_backspin_checker.ppm",
    "demo_host_source": "examples/demos/DemoHost.cpp",
    "demo_host_header": "examples/demos/DemoHost.hpp",
    "registry_source": "examples/demos/Registry.cpp",
    "scenes_header": "examples/demos/scenes/Scenes.hpp",
    "metadata.json": f"{BACKSPIN_V3_BUNDLE}/metadata.json",
    "artifact-index.json": f"{BACKSPIN_V3_BUNDLE}/artifact-index.json",
    "manual-inspection.json": f"{BACKSPIN_V3_BUNDLE}/manual-inspection.json",
    "trace-summary.json": f"{BACKSPIN_V3_BUNDLE}/trace-summary.json",
    "verification.json": f"{BACKSPIN_V3_BUNDLE}/verification.json",
    "invocations.json": f"{BACKSPIN_V3_BUNDLE}/invocations.json",
    "run-summary.json": f"{BACKSPIN_V3_BUNDLE}/run-summary.json",
    "capture-provenance.json": f"{BACKSPIN_V3_BUNDLE}/capture-provenance.json",
    "REPORT.md": f"{BACKSPIN_V3_BUNDLE}/REPORT.md",
    "traces/backspin.csv": f"{BACKSPIN_V3_BUNDLE}/traces/backspin.csv",
    "traces/backspin.stderr.txt": (f"{BACKSPIN_V3_BUNDLE}/traces/backspin.stderr.txt"),
    "backspin/metadata.json": f"{BACKSPIN_V3_BUNDLE}/backspin/metadata.json",
    "backspin/timeline.json": f"{BACKSPIN_V3_BUNDLE}/backspin/timeline.json",
    "backspin/panel.compose.json": (
        f"{BACKSPIN_V3_BUNDLE}/backspin/panel.compose.json"
    ),
    "backspin/panel.png": f"{BACKSPIN_V3_BUNDLE}/backspin/panel.png",
    "backspin/clip.mp4": f"{BACKSPIN_V3_BUNDLE}/backspin/clip.mp4",
    "backspin/clip.gif": f"{BACKSPIN_V3_BUNDLE}/backspin/clip.gif",
}
for _backspin_still_step in (0, 1, 2):
    _backspin_still_key = f"backspin/stills/step_{_backspin_still_step:06d}.png"
    BACKSPIN_V3_ARTIFACT_TARGETS[_backspin_still_key] = (
        f"{BACKSPIN_V3_BUNDLE}/{_backspin_still_key}"
    )
TURNTABLE_V1_ARTIFACT_TARGETS = {
    "finalizer": "scripts/finalize_fbf_turntable_visual.py",
    "finalizer_test": "python/tests/unit/test_finalize_fbf_turntable_visual.py",
    "visual_runner": "scripts/run_fbf_visual_evidence.py",
    "visual_runner_test": "python/tests/unit/test_run_fbf_visual_evidence.py",
    "image_compose": "scripts/image_compose.py",
    "image_tools": "scripts/_image_tools.py",
    "image_verdict": "scripts/image_verdict.py",
    "trace_source": TRACE_SOURCE,
    "trace_cmake_source": "tests/benchmark/integration/CMakeLists.txt",
    "fixture_source": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
    "exact_fbf_source": "dart/constraint/ExactCoulombFbfConstraintSolver.cpp",
    "exact_fbf_header": "dart/constraint/ExactCoulombFbfConstraintSolver.hpp",
    "dart_collision_source": "dart/collision/dart/DARTCollisionDetector.cpp",
    "dart_collision_header": "dart/collision/dart/DARTCollisionDetector.hpp",
    "native_collision_source": "dart/collision/native/NativeCollisionDetector.cpp",
    "native_collision_header": "dart/collision/native/NativeCollisionDetector.hpp",
    "native_cylinder_source": (
        "dart/collision/native/narrow_phase/CylinderCollision.cpp"
    ),
    "native_cylinder_header": (
        "dart/collision/native/narrow_phase/CylinderCollision.hpp"
    ),
    "native_narrow_phase_source": (
        "dart/collision/native/narrow_phase/NarrowPhase.cpp"
    ),
    "native_narrow_phase_header": (
        "dart/collision/native/narrow_phase/NarrowPhase.hpp"
    ),
    "demo_source": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
    "author_turntable_spec_source": (
        "examples/demos/scenes/FbfAuthorTurntableSpec.hpp"
    ),
    "demo_cmake_source": "examples/demos/CMakeLists.txt",
    "demo_main_source": "examples/demos/main.cpp",
    "demo_scene_header": "examples/demos/DemoScene.hpp",
    "demo_host_source": "examples/demos/DemoHost.cpp",
    "demo_host_header": "examples/demos/DemoHost.hpp",
    "registry_source": "examples/demos/Registry.cpp",
    "scenes_header": "examples/demos/scenes/Scenes.hpp",
    "author_turntable_visual_obj": "data/obj/fbf_author_turntable_disc.obj",
    "author_turntable_visual_mtl": "data/obj/fbf_author_turntable_disc.mtl",
    "metadata.json": f"{TURNTABLE_V1_BUNDLE}/metadata.json",
    "artifact-index.json": f"{TURNTABLE_V1_BUNDLE}/artifact-index.json",
    "manual-inspection.json": f"{TURNTABLE_V1_BUNDLE}/manual-inspection.json",
    "trace-summary.json": f"{TURNTABLE_V1_BUNDLE}/trace-summary.json",
    "verification.json": f"{TURNTABLE_V1_BUNDLE}/verification.json",
    "invocations.json": f"{TURNTABLE_V1_BUNDLE}/invocations.json",
    "run-summary.json": f"{TURNTABLE_V1_BUNDLE}/run-summary.json",
    "capture-provenance.json": f"{TURNTABLE_V1_BUNDLE}/capture-provenance.json",
    "REPORT.md": f"{TURNTABLE_V1_BUNDLE}/REPORT.md",
    "groups/turntable_author/metadata.json": (
        f"{TURNTABLE_V1_BUNDLE}/groups/turntable_author/metadata.json"
    ),
    "groups/turntable_author/panel.png": (
        f"{TURNTABLE_V1_BUNDLE}/groups/turntable_author/panel.png"
    ),
    "groups/turntable_author/clip.mp4": (
        f"{TURNTABLE_V1_BUNDLE}/groups/turntable_author/clip.mp4"
    ),
}
TURNTABLE_V1_SCENARIOS = {
    "turntable_author_mu_0_2_omega_2": {
        "capture_id": "turntable_author_mu02_omega2",
        "scene": "fbf_author_turntable_mu_0_2_omega_2",
        "mu": 0.2,
        "omega": 2,
        "expected_outcome": "ejected",
    },
    "turntable_author_mu_0_2_omega_5": {
        "capture_id": "turntable_author_mu02_omega5",
        "scene": "fbf_author_turntable_mu_0_2_omega_5",
        "mu": 0.2,
        "omega": 5,
        "expected_outcome": "ejected",
    },
    "turntable_author_mu_0_5_omega_2": {
        "capture_id": "turntable_author_mu05_omega2",
        "scene": "fbf_author_turntable_mu_0_5_omega_2",
        "mu": 0.5,
        "omega": 2,
        "expected_outcome": "retained_through_6s",
    },
    "turntable_author_mu_0_5_omega_5": {
        "capture_id": "turntable_author_mu05_omega5",
        "scene": "fbf_author_turntable_mu_0_5_omega_5",
        "mu": 0.5,
        "omega": 5,
        "expected_outcome": "ejected",
    },
}
TURNTABLE_V1_CAPTURE_IDS = tuple(
    scenario["capture_id"] for scenario in TURNTABLE_V1_SCENARIOS.values()
)
TURNTABLE_V1_LANES = {
    "current_visual": {
        "solver_contract": "dart_best",
        "collision_frontend": "native",
        "role": "visual_compatible_author_source_port",
    },
    "paper_cpu_native": {
        "solver_contract": "paper_cpu",
        "collision_frontend": "native",
        "role": "separate_strict_diagnostic_not_render_binding",
    },
}
TURNTABLE_V1_GROUP_LABELS = (
    "mu=0.2, omega=2 rad/s",
    "mu=0.2, omega=5 rad/s",
    "mu=0.5, omega=2 rad/s",
    "mu=0.5, omega=5 rad/s",
)
TURNTABLE_V1_GROUP_PANEL_STEPS = (136, 120, 360, 90)
TURNTABLE_V1_GROUP_PANEL_LABELS = (
    "MU 0.2 OMEGA 2 T 2.27S",
    "MU 0.2 OMEGA 5 T 2.00S",
    "MU 0.5 OMEGA 2 T 6.00S",
    "MU 0.5 OMEGA 5 T 1.50S",
)
TURNTABLE_V1_MANUAL_VERDICTS = {
    "four_cell_source_order_visible": True,
    "parameter_labels_legible": True,
    "synchronized_motion_visible": True,
    "three_ejections_visible": True,
    "mu05_omega2_retained_through_6s_visible": True,
    "author_cylinder_support_geometry_visible": True,
    "paper_style_segmented_turntable_visible": True,
    "turntable_rotation_visible": True,
    "outcome_aware_still_steps_bound": True,
    "retained_wording_does_not_claim_zero_slip": True,
    "source_port_and_backend_semantic_boundary_disclosed": True,
    "physical_outcome_claim_requires_current_visual_traces": True,
    "paper_cpu_native_lane_kept_separate": True,
    "rendered_demo_and_trace_full_state_equivalent": False,
    "paper_parity": False,
    "external_solver_parity": False,
    "approved_source_golden": False,
    "timing_verdict": None,
    "realtime_verdict": None,
}
TURNTABLE_V1_CLAIM_SCOPE = (
    "DART port of the public author turntable configuration: manual 2x2 visual "
    "classification independently corroborated by four dart_best/Native-collision "
    "traces."
)
TURNTABLE_V1_CLAIM_BOUNDARY = (
    "The paper_cpu/Native lane remains separate diagnostic evidence. The public "
    "author configuration is ported, but DART gap/collision semantics and state "
    "traces are not Warp/Newton equivalent; the render/trace pairing is not "
    "full-state equivalence; no historical approved golden, external-solver, "
    "timing, or real-time claim is made."
)
TURNTABLE_V1_TRACE_COLUMNS = (
    "step",
    "time",
    "scenario",
    "solver",
    "body",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "up_z",
    "contacts",
    "exact_solves",
    "warm_starts",
    "fallbacks",
    "residual",
    "status",
)
for _turntable_lane in TURNTABLE_V1_LANES:
    for _turntable_scenario in TURNTABLE_V1_SCENARIOS:
        _turntable_trace_key = f"traces/{_turntable_lane}/{_turntable_scenario}.csv"
        TURNTABLE_V1_ARTIFACT_TARGETS[_turntable_trace_key] = (
            f"{TURNTABLE_V1_BUNDLE}/{_turntable_trace_key}"
        )
for _turntable_capture_id in TURNTABLE_V1_CAPTURE_IDS:
    for _turntable_member_file in (
        "metadata.json",
        "timeline.json",
        "panel.png",
        "clip.mp4",
    ):
        _turntable_member_key = f"{_turntable_capture_id}/{_turntable_member_file}"
        TURNTABLE_V1_ARTIFACT_TARGETS[_turntable_member_key] = (
            f"{TURNTABLE_V1_BUNDLE}/{_turntable_member_key}"
        )
IMPACT_V7_ARTIFACT_TARGETS = {
    "runner": IMPACT_V7_RUNNER,
    "runner_test": ("python/tests/unit/test_run_fbf_literal_crown_impact_negative.py"),
    "trace_source": TRACE_SOURCE,
    "raw.csv": f"{IMPACT_V7_BUNDLE}/raw.csv",
    "stderr.txt": f"{IMPACT_V7_BUNDLE}/stderr.txt",
    "standing-reference.csv": f"{IMPACT_V7_BUNDLE}/standing-reference.csv",
    "standing-reference.stderr.txt": (
        f"{IMPACT_V7_BUNDLE}/standing-reference.stderr.txt"
    ),
    "summary.json": f"{IMPACT_V7_BUNDLE}/summary.json",
    "metadata.json": f"{IMPACT_V7_BUNDLE}/metadata.json",
    "REPORT.md": f"{IMPACT_V7_BUNDLE}/REPORT.md",
}
ARCH101_V5_ARTIFACT_TARGETS = {
    "runner": ARCH101_V1_RUNNER,
    "runner_test": "python/tests/unit/test_run_fbf_literal_arch101_v1.py",
    "trace_source": TRACE_SOURCE,
    "collision_probe_source": COLLISION_PROBE_SOURCE,
    "raw.csv": f"{ARCH101_V5_BUNDLE}/raw.csv",
    "stderr.txt": f"{ARCH101_V5_BUNDLE}/stderr.txt",
    "collision_probe_stdout.txt": (f"{ARCH101_V5_BUNDLE}/collision_probe_stdout.txt"),
    "collision_probe_stderr.txt": (f"{ARCH101_V5_BUNDLE}/collision_probe_stderr.txt"),
    "invocation.json": f"{ARCH101_V5_BUNDLE}/invocation.json",
    "summary.json": f"{ARCH101_V5_BUNDLE}/summary.json",
    "metadata.json": f"{ARCH101_V5_BUNDLE}/metadata.json",
    "REPORT.md": f"{ARCH101_V5_BUNDLE}/REPORT.md",
}


def _nonempty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _check_string_list(
    value: Any, location: str, errors: list[str], *, allow_empty: bool = False
) -> list[str]:
    if not isinstance(value, list):
        errors.append(f"{location}: expected a list")
        return []
    if not allow_empty and not value:
        errors.append(f"{location}: must not be empty")
    for index, item in enumerate(value):
        if not _nonempty_string(item):
            errors.append(f"{location}[{index}]: expected a non-empty string")
    return [item for item in value if _nonempty_string(item)]


def _repository_path_without_symlinks(
    relative: Path, location: str, repo_root: Path, errors: list[str]
) -> tuple[Path, Path] | None:
    root = repo_root.resolve()
    candidate = root
    for part in relative.parts:
        candidate /= part
        if candidate.is_symlink():
            errors.append(
                f"{location}: repository evidence paths must not contain "
                f"symlinks: {relative.as_posix()}"
            )
            return None
    resolved = candidate.resolve()
    if resolved != candidate:
        errors.append(
            f"{location}: repository evidence paths must not contain "
            f"symlinks: {relative.as_posix()}"
        )
        return None
    return root, resolved


def _artifact_path(
    value: Any, location: str, repo_root: Path, errors: list[str]
) -> Path | None:
    if not _nonempty_string(value):
        errors.append(f"{location}: expected a non-empty repository-relative path")
        return None

    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        errors.append(f"{location}: path must remain inside the repository")
        return None

    lexical = _repository_path_without_symlinks(relative, location, repo_root, errors)
    if lexical is None:
        return None
    root, candidate = lexical
    if candidate != root and root not in candidate.parents:
        errors.append(f"{location}: path resolves outside the repository")
        return None
    if not candidate.exists():
        errors.append(f"{location}: local evidence does not exist: {value}")
        return None
    if not candidate.is_file():
        errors.append(f"{location}: local evidence is not a regular file: {value}")
        return None
    return candidate


def _evidence_directory(
    value: Any, location: str, repo_root: Path, errors: list[str]
) -> Path | None:
    if not _nonempty_string(value):
        errors.append(f"{location}: expected a non-empty repository-relative path")
        return None

    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        errors.append(f"{location}: path must remain inside the repository")
        return None

    lexical = _repository_path_without_symlinks(relative, location, repo_root, errors)
    if lexical is None:
        return None
    root, candidate = lexical
    if candidate != root and root not in candidate.parents:
        errors.append(f"{location}: path resolves outside the repository")
        return None
    if not candidate.exists():
        errors.append(f"{location}: current evidence bundle does not exist: {value}")
        return None
    if not candidate.is_dir():
        errors.append(
            f"{location}: current evidence bundle is not a directory: {value}"
        )
        return None
    return candidate


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_sha256(payload: Any) -> str:
    encoded = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _read_json_object(
    path: Path, location: str, errors: list[str]
) -> dict[str, Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        errors.append(f"{location}: invalid JSON: {error}")
        return None
    if not isinstance(data, dict):
        errors.append(f"{location}: structured deliverable must be a JSON object")
        return None
    return data


def _read_json_array(path: Path, location: str, errors: list[str]) -> list[Any] | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        errors.append(f"{location}: invalid JSON: {error}")
        return None
    if not isinstance(data, list):
        errors.append(f"{location}: structured deliverable must be a JSON array")
        return None
    return data


def _object(value: Any, location: str, errors: list[str]) -> dict[str, Any] | None:
    if not isinstance(value, dict):
        errors.append(f"{location}: expected an object")
        return None
    return value


def _expect_exact_keys(
    data: Any,
    expected_keys: set[str],
    location: str,
    errors: list[str],
) -> None:
    """Require exact object membership without interpreting field values."""

    if not isinstance(data, dict):
        return
    actual_keys = set(data)
    missing = sorted(expected_keys - actual_keys)
    unexpected = sorted(actual_keys - expected_keys)
    if missing:
        errors.append(f"{location}: missing keys {missing}")
    if unexpected:
        errors.append(f"{location}: unexpected keys {unexpected}")


def _validate_closed_object_tree(
    value: Any,
    schema_path: str,
    location: str,
    object_keys: dict[str, set[str]],
    errors: list[str],
    *,
    delegated_object_paths: set[str] | None = None,
) -> None:
    """Validate object keys and reject objects smuggled into scalar/list leaves."""

    delegated_object_paths = delegated_object_paths or set()
    expected_keys = object_keys.get(schema_path)
    if not isinstance(value, dict):
        errors.append(f"{location}: expected an object")
        return
    if expected_keys is None:
        errors.append(f"{location}: no schema-v1 object-key contract")
        return
    _expect_exact_keys(value, expected_keys, location, errors)
    for field in sorted(expected_keys & set(value)):
        child = value[field]
        child_schema_path = f"{schema_path}.{field}"
        child_location = f"{location}.{field}"
        if child_schema_path in delegated_object_paths:
            if not isinstance(child, dict):
                errors.append(f"{child_location}: expected an object")
            continue
        if child_schema_path in object_keys:
            _validate_closed_object_tree(
                child,
                child_schema_path,
                child_location,
                object_keys,
                errors,
                delegated_object_paths=delegated_object_paths,
            )
            continue
        item_schema_path = f"{child_schema_path}[]"
        if item_schema_path in object_keys:
            if not isinstance(child, list):
                errors.append(f"{child_location}: expected a list of objects")
                continue
            for index, item in enumerate(child):
                _validate_closed_object_tree(
                    item,
                    item_schema_path,
                    f"{child_location}[{index}]",
                    object_keys,
                    errors,
                    delegated_object_paths=delegated_object_paths,
                )
            continue
        if isinstance(child, dict):
            errors.append(f"{child_location}: unexpected object in a scalar field")
        elif isinstance(child, list) and any(
            isinstance(item, (dict, list)) for item in child
        ):
            errors.append(
                f"{child_location}: expected a flat list without nested objects"
            )


def _validate_configuration_profile_key_contract(
    profile_id: str,
    payload: Any,
    errors: list[str],
) -> None:
    """Validate every object node in one versioned configuration profile."""
    _validate_closed_object_tree(
        payload,
        profile_id,
        f"configuration_profiles.{profile_id}",
        CONFIGURATION_PROFILE_OBJECT_KEYS,
        errors,
    )


def _expect_fields(
    data: Any,
    expected: dict[str, Any],
    location: str,
    errors: list[str],
) -> None:
    if not isinstance(data, dict):
        errors.append(f"{location}: expected an object")
        return
    for field, expected_value in expected.items():
        actual = data.get(field)
        if type(actual) is not type(expected_value) or actual != expected_value:
            errors.append(
                f"{location}.{field}: expected {expected_value!r}, got {actual!r}"
            )


def _expect_exact_payload(
    actual: Any, expected: Any, location: str, errors: list[str]
) -> None:
    """Recursively require an exact JSON-compatible payload with useful paths."""

    if type(actual) is not type(expected):
        errors.append(
            f"{location}: expected {type(expected).__name__}, "
            f"got {type(actual).__name__}"
        )
        return
    if isinstance(expected, dict):
        actual_keys = set(actual)
        expected_keys = set(expected)
        missing = sorted(expected_keys - actual_keys)
        unexpected = sorted(actual_keys - expected_keys)
        if missing:
            errors.append(f"{location}: missing keys {missing}")
        if unexpected:
            errors.append(f"{location}: unexpected keys {unexpected}")
        for key in sorted(expected_keys & actual_keys):
            _expect_exact_payload(
                actual[key], expected[key], f"{location}.{key}", errors
            )
        return
    if isinstance(expected, list):
        if len(actual) != len(expected):
            errors.append(
                f"{location}: expected {len(expected)} entries, got {len(actual)}"
            )
        for index, (actual_item, expected_item) in enumerate(zip(actual, expected)):
            _expect_exact_payload(
                actual_item,
                expected_item,
                f"{location}[{index}]",
                errors,
            )
        return
    if actual != expected:
        errors.append(f"{location}: expected {expected!r}, got {actual!r}")


def _validate_live_absolute_file_identity(
    identity: dict[str, Any],
    path_field: str,
    location: str,
    errors: list[str],
) -> Path | None:
    path_value = identity.get(path_field)
    if not _nonempty_string(path_value):
        errors.append(f"{location}.{path_field}: expected an absolute file path")
        return None
    path = Path(path_value)
    try:
        if not path.is_absolute() or not path.is_file():
            errors.append(f"{location}.{path_field}: recorded file is unavailable")
            return None
        expected = {"sha256": _sha256(path)}
        if "size_bytes" in identity:
            expected["size_bytes"] = path.stat().st_size
    except OSError as error:
        errors.append(
            f"{location}.{path_field}: recorded file could not be read: {error}"
        )
        return None
    _expect_fields(identity, expected, location, errors)
    return path


def _validate_tool_identity(
    value: Any, expected_name: str, location: str, errors: list[str]
) -> dict[str, Any] | None:
    identity = _object(value, location, errors)
    if identity is None:
        return None
    expected_keys = {"name", "path", "resolved_path", "sha256", "size_bytes"}
    if set(identity) != expected_keys:
        errors.append(f"{location}: exact tool-identity contract changed")
    _expect_fields(identity, {"name": expected_name}, location, errors)
    resolved = _validate_live_absolute_file_identity(
        identity, "resolved_path", location, errors
    )
    original_value = identity.get("path")
    if not _nonempty_string(original_value) or not Path(original_value).is_absolute():
        errors.append(f"{location}.path: expected an absolute tool path")
    elif resolved is not None:
        try:
            current_resolved = Path(original_value).resolve(strict=True)
        except OSError:
            errors.append(f"{location}.path: recorded tool path is unavailable")
        else:
            if current_resolved != resolved:
                errors.append(
                    f"{location}.path: expected to resolve to {resolved}, "
                    f"got {current_resolved}"
                )
    return identity


def _validate_recorded_tool_command(
    argument: Any,
    tool_identity: dict[str, Any],
    location: str,
    errors: list[str],
) -> None:
    if not _nonempty_string(argument):
        errors.append(f"{location}: expected a non-empty executable")
        return
    recorded_resolved = tool_identity.get("resolved_path")
    if not _nonempty_string(recorded_resolved):
        errors.append(f"{location}: recorded tool closure has no resolved path")
        return
    if Path(argument).is_absolute():
        try:
            invoked_resolved = Path(argument).resolve(strict=True)
        except OSError:
            errors.append(f"{location}: invoked executable is unavailable")
            return
    elif argument == tool_identity.get("name"):
        discovered = shutil.which(argument)
        if discovered is None:
            errors.append(f"{location}: invoked executable is unavailable")
            return
        invoked_resolved = Path(discovered).resolve()
    else:
        errors.append(
            f"{location}: executable is not bound to the recorded tool identity"
        )
        return
    if invoked_resolved != Path(recorded_resolved):
        errors.append(
            f"{location}: expected recorded executable {recorded_resolved!r}, "
            f"got {str(invoked_resolved)!r}"
        )


def _validate_structured_executable_identity(
    value: Any,
    location: str,
    repo_root: Path,
    errors: list[str],
    *,
    expected_sha256: str | None,
) -> dict[str, Any] | None:
    identity = _object(value, location, errors)
    if identity is None:
        return None
    expected_keys = {
        "path",
        "size_bytes",
        "sha256",
        "ldd_tool",
        "ldd_resolution_output_normalized",
        "ldd_resolution_output_normalized_sha256",
        "ldd_stderr_sha256",
        "resolved_regular_shared_libraries",
        "resolved_regular_shared_library_count",
        "resolved_build_libdart",
    }
    if set(identity) != expected_keys:
        errors.append(f"{location}: exact executable-identity contract changed")

    _validate_live_absolute_file_identity(identity, "path", location, errors)
    if expected_sha256 is not None:
        _expect_fields(identity, {"sha256": expected_sha256}, location, errors)
    _validate_tool_identity(
        identity.get("ldd_tool"), "ldd", f"{location}.ldd_tool", errors
    )

    normalized = identity.get("ldd_resolution_output_normalized")
    if not _nonempty_string(normalized):
        errors.append(
            f"{location}.ldd_resolution_output_normalized: expected non-empty text"
        )
    else:
        _expect_fields(
            identity,
            {
                "ldd_resolution_output_normalized_sha256": hashlib.sha256(
                    normalized.encode("utf-8")
                ).hexdigest()
            },
            location,
            errors,
        )
    stderr_hash = identity.get("ldd_stderr_sha256")
    if not isinstance(stderr_hash, str) or not SHA256_PATTERN.fullmatch(stderr_hash):
        errors.append(f"{location}.ldd_stderr_sha256: expected lowercase SHA-256")

    libraries = identity.get("resolved_regular_shared_libraries")
    if not isinstance(libraries, list) or not libraries:
        errors.append(
            f"{location}.resolved_regular_shared_libraries: "
            "expected a non-empty list"
        )
        return identity
    _expect_fields(
        identity,
        {"resolved_regular_shared_library_count": len(libraries)},
        location,
        errors,
    )

    normalized_rows: list[str] = []
    build_root = (repo_root / "build").resolve()
    build_libdart: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    for index, item in enumerate(libraries):
        item_location = f"{location}.resolved_regular_shared_libraries[{index}]"
        library = _object(item, item_location, errors)
        if library is None:
            continue
        if set(library) != {
            "soname",
            "reported_path",
            "resolved_path",
            "sha256",
            "size_bytes",
        }:
            errors.append(f"{item_location}: exact library-identity contract changed")
        soname = library.get("soname")
        reported_value = library.get("reported_path")
        if not _nonempty_string(soname):
            errors.append(f"{item_location}.soname: expected a non-empty string")
        resolved = _validate_live_absolute_file_identity(
            library, "resolved_path", item_location, errors
        )
        if (
            not _nonempty_string(reported_value)
            or not Path(reported_value).is_absolute()
        ):
            errors.append(
                f"{item_location}.reported_path: expected an absolute file path"
            )
        elif resolved is not None:
            try:
                current_resolved = Path(reported_value).resolve(strict=True)
            except OSError:
                errors.append(
                    f"{item_location}.reported_path: recorded file is unavailable"
                )
            else:
                if current_resolved != resolved:
                    errors.append(
                        f"{item_location}.reported_path: expected to resolve to "
                        f"{resolved}, got {current_resolved}"
                    )
        if _nonempty_string(soname) and resolved is not None:
            key = (soname, str(resolved))
            if key in seen:
                errors.append(f"{item_location}: duplicate resolved library identity")
            seen.add(key)
            normalized_rows.append(f"{soname} => {reported_value} => {resolved}")
            if soname.startswith("libdart.so") and (
                resolved == build_root or build_root in resolved.parents
            ):
                build_libdart.append(library)

    expected_normalized = "\n".join(normalized_rows) + "\n"
    if isinstance(normalized, str) and normalized != expected_normalized:
        errors.append(
            f"{location}.ldd_resolution_output_normalized: "
            "resolved-library map drifted"
        )
    if len(build_libdart) != 1:
        errors.append(
            f"{location}.resolved_build_libdart: expected exactly one "
            f"build-tree libdart, got {len(build_libdart)}"
        )
    elif identity.get("resolved_build_libdart") != build_libdart[0]:
        errors.append(
            f"{location}.resolved_build_libdart: bound library identity drifted"
        )
    return identity


def _validate_cpu_runtime_identity(
    value: Any, location: str, repo_root: Path, errors: list[str]
) -> dict[str, Any] | None:
    identity = _object(value, location, errors)
    if identity is None:
        return None
    expected_keys = {
        "ldd_path",
        "ldd_sha256",
        "resolved_regular_files",
        "resolved_regular_files_sha256",
    }
    if set(identity) != expected_keys:
        errors.append(f"{location}: exact runtime-identity contract changed")

    ldd_path_value = identity.get("ldd_path")
    if not _nonempty_string(ldd_path_value):
        errors.append(f"{location}.ldd_path: expected an absolute file path")
    else:
        ldd_path = Path(ldd_path_value)
        if not ldd_path.is_absolute() or not ldd_path.is_file():
            errors.append(f"{location}.ldd_path: recorded file is unavailable")
        else:
            _expect_fields(
                identity,
                {"ldd_sha256": _sha256(ldd_path)},
                location,
                errors,
            )

    resolved = _object(
        identity.get("resolved_regular_files"),
        f"{location}.resolved_regular_files",
        errors,
    )
    if resolved is None:
        return identity
    if not resolved:
        errors.append(f"{location}.resolved_regular_files: must not be empty")
    expected_map_hash = hashlib.sha256(
        json.dumps(resolved, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    _expect_fields(
        identity,
        {"resolved_regular_files_sha256": expected_map_hash},
        location,
        errors,
    )

    build_root = (repo_root / "build").resolve()
    build_libdart: list[str] = []
    for path_value, item in resolved.items():
        item_location = f"{location}.resolved_regular_files[{path_value!r}]"
        record = _object(item, item_location, errors)
        if record is None:
            continue
        if set(record) != {"sha256", "size_bytes"}:
            errors.append(f"{item_location}: exact file-identity contract changed")
        if not isinstance(path_value, str):
            errors.append(f"{item_location}: path key must be a string")
            continue
        raw_path = Path(path_value)
        try:
            canonical_path = raw_path.resolve(strict=True)
        except OSError:
            canonical_path = None
        if canonical_path is not None and raw_path != canonical_path:
            errors.append(
                f"{item_location}: recorded runtime path must be canonical; "
                f"got {path_value!r}, resolved {str(canonical_path)!r}"
            )
        path = _validate_live_absolute_file_identity(
            {**record, "path": path_value}, "path", item_location, errors
        )
        if (
            path is not None
            and path.name.startswith("libdart.so.")
            and (path == build_root or build_root in path.parents)
        ):
            build_libdart.append(path_value)
    if len(build_libdart) != 1:
        errors.append(
            f"{location}: expected exactly one resolved build-tree libdart, "
            f"got {build_libdart}"
        )
    return identity


def _validate_cpu_executed_tool_closure(
    metadata: dict[str, Any],
    configuration: dict[str, Any] | None,
    location: str,
    errors: list[str],
) -> dict[str, Any] | None:
    closure = _object(
        metadata.get("executed_tool_closure"),
        f"{location}.executed_tool_closure",
        errors,
    )
    cpu_list = configuration.get("cpu_list") if configuration is not None else None
    cpu_lists = (
        configuration.get("cpu_lists_by_threads") if configuration is not None else None
    )
    pinning_requested = bool(cpu_list) or bool(cpu_lists)
    expected_keys = {"taskset"} if pinning_requested else set()
    if closure is not None and set(closure) != expected_keys:
        errors.append(
            f"{location}.executed_tool_closure: exact executed-tool "
            "membership changed"
        )
    rechecks = metadata.get("executed_tool_identity_rechecks")
    if not isinstance(rechecks, list) or any(
        not _nonempty_string(stage) for stage in rechecks
    ):
        errors.append(
            f"{location}.executed_tool_identity_rechecks: expected a string list"
        )
    if not pinning_requested:
        if rechecks != []:
            errors.append(
                f"{location}.executed_tool_identity_rechecks: "
                "unpinned evidence must have no tool rechecks"
            )
        return None
    if closure is None:
        return None
    return _validate_tool_identity(
        closure.get("taskset"),
        "taskset",
        f"{location}.executed_tool_closure.taskset",
        errors,
    )


def _validate_cpu_runtime_provenance(
    value: Any,
    runtime_identity: dict[str, Any] | None,
    taskset_tool: dict[str, Any] | None,
    executed_tool_identity_rechecks: Any,
    location: str,
    repo_root: Path,
    errors: list[str],
) -> None:
    provenance = _object(value, location, errors)
    if provenance is None or runtime_identity is None:
        return
    resolved = runtime_identity.get("resolved_regular_files")
    resolved = resolved if isinstance(resolved, dict) else {}
    build_root = (repo_root / "build").resolve()
    libdart = [
        record
        for path_value, record in resolved.items()
        if isinstance(path_value, str)
        and isinstance(record, dict)
        and Path(path_value).name.startswith("libdart.so.")
        and (Path(path_value) == build_root or build_root in Path(path_value).parents)
    ]
    expected = {
        "scope": (
            "Executed trace binary, ldd tool, resolved regular runtime files, "
            "and the exact taskset executable used for CPU affinity were "
            "recorded and rechecked after every invocation; this is not a claim "
            "over all host runtime state."
        ),
        "identity_recheck_stage": "after_all_invocations",
        "executed_tool_identity_rechecks": executed_tool_identity_rechecks,
        "resolved_regular_file_count": len(resolved),
        "resolved_regular_files_sha256": runtime_identity.get(
            "resolved_regular_files_sha256"
        ),
        "resolved_build_libdart_sha256": (
            libdart[0].get("sha256") if len(libdart) == 1 else None
        ),
        "ldd_tool_sha256": runtime_identity.get("ldd_sha256"),
        "taskset_tool_sha256": (
            taskset_tool.get("sha256") if taskset_tool is not None else None
        ),
    }
    if set(provenance) != set(expected):
        errors.append(f"{location}: exact runtime-provenance contract changed")
    _expect_fields(provenance, expected, location, errors)


def _validate_current_path(
    data: dict[str, Any],
    field: str,
    expected: str,
    location: str,
    repo_root: Path,
    errors: list[str],
    *,
    directory: bool = False,
) -> Path | None:
    value = data.get(field)
    if value != expected:
        errors.append(
            f"{location}.{field}: expected current path {expected!r}, got {value!r}"
        )
    resolver = _evidence_directory if directory else _artifact_path
    return resolver(value, f"{location}.{field}", repo_root, errors)


def _contract_sha256(
    path: Path | None,
    marker: str,
    location: str,
    errors: list[str],
) -> str | None:
    if path is None:
        return None
    try:
        text = path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        errors.append(f"{location}: protocol could not be read: {error}")
        return None
    if marker not in text:
        errors.append(f"{location}: protocol result marker {marker!r} is missing")
        return None
    frozen = (text.split(marker, 1)[0].rstrip() + "\n").encode()
    return hashlib.sha256(frozen).hexdigest()


def _validate_artifact_hashes(
    data: dict[str, Any],
    targets: dict[str, str],
    location: str,
    repo_root: Path,
    errors: list[str],
    *,
    computed_hashes: dict[str, str | None] | None = None,
    recorded_hash_keys: set[str] | None = None,
) -> dict[str, str]:
    hashes = data.get("artifact_hashes")
    if not isinstance(hashes, dict):
        errors.append(f"{location}.artifact_hashes: expected an object")
        return {}

    computed_hashes = computed_hashes or {}
    recorded_hash_keys = recorded_hash_keys or set()
    expected_keys = set(targets) | set(computed_hashes) | recorded_hash_keys
    invalid_keys = [key for key in hashes if not isinstance(key, str)]
    if invalid_keys:
        errors.append(
            f"{location}.artifact_hashes: keys must be strings; got "
            f"{[repr(key) for key in invalid_keys]}"
        )
    actual_keys = {key for key in hashes if isinstance(key, str)}
    missing = sorted(expected_keys - actual_keys)
    unexpected = sorted(actual_keys - expected_keys)
    if missing:
        errors.append(f"{location}.artifact_hashes: missing current keys {missing}")
    if unexpected:
        errors.append(
            f"{location}.artifact_hashes: unexpected current keys {unexpected}"
        )

    actual_hashes: dict[str, str | None] = dict(computed_hashes)
    for key, target in targets.items():
        path = _artifact_path(
            target,
            f"{location}.artifact_hashes[{key!r}].path",
            repo_root,
            errors,
        )
        actual_hashes[key] = _sha256(path) if path is not None else None

    validated: dict[str, str] = {}
    for key in sorted(expected_keys):
        declared = hashes.get(key)
        hash_location = f"{location}.artifact_hashes[{key!r}]"
        if not isinstance(declared, str) or not SHA256_PATTERN.fullmatch(declared):
            errors.append(f"{hash_location}: expected lowercase SHA-256")
            continue
        validated[key] = declared
        actual = actual_hashes.get(key)
        if actual is not None and declared != actual:
            errors.append(
                f"{hash_location}: digest mismatch; expected {declared}, "
                f"computed {actual}"
            )
    return validated


def _read_bundle_json(
    bundle: Path | None, name: str, location: str, errors: list[str]
) -> dict[str, Any] | None:
    if bundle is None:
        return None
    path = bundle / name
    if not path.is_file():
        errors.append(f"{location}: current bundle JSON does not exist: {name}")
        return None
    return _read_json_object(path, location, errors)


def _validate_pruned_staging(
    bundle: Path,
    value: Any,
    *,
    schema_version: str,
    artifact_count: int,
    location: str,
    errors: list[str],
) -> dict[str, dict[str, Any]]:
    staging = _object(value, location, errors)
    if staging is None:
        return {}
    if set(staging) != {
        "schema_version",
        "disposition",
        "artifact_count",
        "artifacts",
    }:
        errors.append(f"{location}: exact contract changed")
    _expect_fields(
        staging,
        {
            "schema_version": schema_version,
            "disposition": "pruned_before_seal",
            "artifact_count": artifact_count,
        },
        location,
        errors,
    )
    artifacts = staging.get("artifacts")
    if not isinstance(artifacts, list) or len(artifacts) != artifact_count:
        errors.append(f"{location}.artifacts: expected {artifact_count} records")
        return {}
    by_path: dict[str, dict[str, Any]] = {}
    for index, item in enumerate(artifacts):
        item_location = f"{location}.artifacts[{index}]"
        if not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}:
            errors.append(f"{item_location}: exact contract changed")
            continue
        relative = item.get("path")
        size = _integer(item.get("bytes"))
        digest = item.get("sha256")
        if not isinstance(relative, str) or not relative or relative in by_path:
            errors.append(f"{item_location}.path: expected a unique relative path")
            continue
        if Path(relative).is_absolute() or ".." in Path(relative).parts:
            errors.append(f"{item_location}.path: unsafe path")
            continue
        if size is None or size < 0:
            errors.append(f"{item_location}.bytes: expected a non-negative integer")
        if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest):
            errors.append(f"{item_location}.sha256: expected lowercase SHA-256")
        if (bundle / relative).exists() or (bundle / relative).is_symlink():
            errors.append(f"{item_location}.path: staging artifact was not pruned")
        by_path[relative] = item
    return by_path


def _validate_durable_stills(
    bundle: Path,
    value: Any,
    expected: tuple[tuple[int, str, str], ...],
    location: str,
    errors: list[str],
) -> None:
    if not isinstance(value, list) or len(value) != len(expected):
        errors.append(f"{location}: expected {len(expected)} durable stills")
        return
    for index, (item, (step, path, source_path)) in enumerate(zip(value, expected)):
        item_location = f"{location}[{index}]"
        if not isinstance(item, dict) or set(item) != {
            "path",
            "run_summary_frame_sha256",
            "sha256",
            "source_frame_path",
            "step",
            "timeline_frame_sha256",
        }:
            errors.append(f"{item_location}: exact contract changed")
            continue
        artifact = bundle / path
        digest = _sha256(artifact) if artifact.is_file() else None
        _expect_fields(
            item,
            {
                "step": step,
                "path": path,
                "source_frame_path": source_path,
                "sha256": digest,
                "run_summary_frame_sha256": digest,
                "timeline_frame_sha256": digest,
            },
            item_location,
            errors,
        )


def _read_csv(
    path: Path, location: str, errors: list[str]
) -> tuple[list[str], list[dict[str, str]]] | None:
    try:
        with path.open(encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames or [])
            rows = list(reader)
    except (OSError, UnicodeDecodeError, csv.Error) as error:
        errors.append(f"{location}: invalid CSV: {error}")
        return None
    if not fieldnames or any(not _nonempty_string(field) for field in fieldnames):
        errors.append(f"{location}: CSV requires a non-empty header")
        return None
    if len(fieldnames) != len(set(fieldnames)):
        errors.append(f"{location}: CSV header contains duplicate columns")
        return None
    if not rows:
        errors.append(f"{location}: CSV requires at least one data row")
        return None
    if any(None in row for row in rows):
        errors.append(f"{location}: CSV row has more values than the header")
        return None
    if any(any(value is None for value in row.values()) for row in rows):
        errors.append(f"{location}: CSV row has fewer values than the header")
        return None
    return fieldnames, rows


def _has_image_signature(path: Path) -> bool:
    with path.open("rb") as stream:
        header = stream.read(32)
    suffix = path.suffix.lower()
    if suffix == ".png":
        return header.startswith(b"\x89PNG\r\n\x1a\n")
    if suffix in {".jpg", ".jpeg"}:
        return header.startswith(b"\xff\xd8\xff")
    if suffix == ".webp":
        return header.startswith(b"RIFF") and header[8:12] == b"WEBP"
    if suffix in {".tif", ".tiff"}:
        return header.startswith((b"II*\x00", b"MM\x00*"))
    return False


def _has_plot_signature(path: Path) -> bool:
    suffix = path.suffix.lower()
    if suffix == ".pdf":
        with path.open("rb") as stream:
            return stream.read(5) == b"%PDF-"
    if suffix == ".svg":
        try:
            with path.open(encoding="utf-8") as stream:
                prefix = stream.read(4096).lower()
        except (OSError, UnicodeDecodeError):
            return False
        return "<svg" in prefix
    return _has_image_signature(path)


def _has_video_signature(path: Path) -> bool:
    with path.open("rb") as stream:
        header = stream.read(32)
    suffix = path.suffix.lower()
    if suffix == ".gif":
        return header.startswith((b"GIF87a", b"GIF89a"))
    if suffix in {".mov", ".mp4"}:
        return len(header) >= 12 and header[4:8] == b"ftyp"
    if suffix == ".webm":
        return header.startswith(b"\x1a\x45\xdf\xa3")
    return False


def _validate_deliverable_type(
    kind: Any,
    path: Path,
    location: str,
    errors: list[str],
) -> dict[str, Any] | tuple[list[str], list[dict[str, str]]] | None:
    if not isinstance(kind, str) or kind not in DELIVERABLE_EXTENSIONS:
        return None
    suffix = path.suffix.lower()
    allowed = DELIVERABLE_EXTENSIONS[kind]
    if suffix not in allowed:
        errors.append(
            f"{location}: {kind} requires one of {sorted(allowed)}, got {suffix!r}"
        )
        return None

    if suffix == ".json":
        return _read_json_object(path, location, errors)
    if suffix == ".csv":
        return _read_csv(path, location, errors)
    if kind in {"approved_golden", "still_image"}:
        valid = _has_image_signature(path)
    elif kind in {"comparison_plot", "residual_plot"}:
        valid = _has_plot_signature(path)
    elif kind == "video_clip":
        valid = _has_video_signature(path)
    elif kind == "exact_fixture":
        try:
            valid = bool(path.read_text(encoding="utf-8").strip())
        except (OSError, UnicodeDecodeError):
            valid = False
    else:
        valid = True
    if not valid:
        errors.append(f"{location}: file signature/content does not match {kind}")
    return None


def _require_requirement_binding(
    data: dict[str, Any],
    requirement_id: str,
    location: str,
    errors: list[str],
) -> None:
    requirement_ids = data.get("requirement_ids")
    if not isinstance(requirement_ids, list) or requirement_id not in requirement_ids:
        errors.append(f"{location}.requirement_ids: must bind {requirement_id!r}")
    elif any(not _nonempty_string(item) for item in requirement_ids):
        errors.append(f"{location}.requirement_ids: expected non-empty strings")


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        result = float(value)
    except OverflowError:
        return None
    return result if math.isfinite(result) else None


def _integer(value: Any) -> int | None:
    if isinstance(value, bool) or not isinstance(value, int):
        return None
    return value


def _validate_runtime_solver_diagnostics(
    diagnostics: Any,
    location: str,
    errors: list[str],
    previous_counters: dict[str, int] | None,
) -> dict[str, int] | None:
    if not isinstance(diagnostics, dict):
        errors.append(f"{location}: native DemoHost diagnostics are required")
        return None
    if diagnostics.get("available") is not True:
        errors.append(f"{location}.available: expected true")
    if diagnostics.get("solver") != "ExactCoulombFbfConstraintSolver":
        errors.append(f"{location}.solver: exact FBF solver is required")

    counters: dict[str, int] = {}
    for field in (
        "total_iterations",
        "exact_solves",
        "exact_attempts",
        "accepted_at_cap",
        "exact_failures",
        "boxed_lcp_fallbacks",
        "warm_starts",
    ):
        value = _integer(diagnostics.get(field))
        if value is None or value < 0:
            errors.append(f"{location}.{field}: expected a non-negative integer")
            continue
        counters[field] = value
        if previous_counters is not None and value < previous_counters.get(field, 0):
            errors.append(f"{location}.{field}: cumulative counter regressed")

    contacts = _integer(diagnostics.get("contacts"))
    if contacts is None or contacts < 0:
        errors.append(f"{location}.contacts: expected a non-negative integer")
    elif contacts > 0 and (
        diagnostics.get("status") != "success"
        or diagnostics.get("fbf_status") != "success"
    ):
        errors.append(f"{location}: contact-step solver statuses must be 'success'")

    for field in ("accepted_at_cap", "exact_failures", "boxed_lcp_fallbacks"):
        if counters.get(field) != 0:
            errors.append(f"{location}.{field}: validated evidence requires 0")

    worst_residual = diagnostics.get("worst_residual")
    if worst_residual is not None:
        residual = _number(worst_residual)
        if residual is None or residual < 0.0 or residual > PAPER_RESIDUAL_TOLERANCE:
            errors.append(
                f"{location}.worst_residual: expected a finite value <= "
                f"{PAPER_RESIDUAL_TOLERANCE}"
            )
    return counters


def _validate_capture_sidecar(
    data: dict[str, Any],
    requirement_id: str,
    location: str,
    repo_root: Path,
    errors: list[str],
) -> None:
    # DemoHost emits runtime state, but it does not provide an external,
    # non-self-attesting provenance signature. A hand-authored JSON document
    # can therefore imitate every runtime field and bind itself to arbitrary
    # media. Keep capture completion fail-closed until a production attestation
    # channel exists; the checks below still report structural defects and are
    # useful for partial evidence preparation.
    errors.append(
        f"{location}: validated capture completion is unavailable because no "
        "non-self-attesting production attestation exists"
    )
    _require_requirement_binding(data, requirement_id, location, errors)
    if "solver_contract" in data:
        errors.append(
            f"{location}.solver_contract: hand-authored solver records are not "
            "accepted; use the native DemoHost runtime timeline"
        )

    command = data.get("runtime_command")
    if not _nonempty_string(command) or any(
        token not in command
        for token in ("dart-demos", "--headless", "--headless-sidecar")
    ):
        errors.append(
            f"{location}.runtime_command: expected the recorded DemoHost headless "
            "sidecar command"
        )
    build = data.get("build")
    if not isinstance(build, dict) or not all(
        _nonempty_string(build.get(field)) for field in ("dart_version", "mode")
    ):
        errors.append(f"{location}.build: runtime DART version and mode are required")
    scene = data.get("scene")
    if not _nonempty_string(scene) or data.get("active_scene") != scene:
        errors.append(f"{location}: scene and active_scene must name the runtime scene")

    total_steps = _integer(data.get("total_steps"))
    completed_steps = _integer(data.get("completed_steps"))
    if total_steps is None or total_steps < 2:
        errors.append(
            f"{location}.total_steps: full-trajectory evidence requires at least "
            "two completed runtime steps"
        )
        return
    if completed_steps != total_steps:
        errors.append(
            f"{location}: incomplete trajectory "
            f"({completed_steps!r} of {total_steps} steps)"
        )

    trajectory = data.get("steps")
    if not isinstance(trajectory, list):
        errors.append(
            f"{location}.steps: native DemoHost diagnostics for every runtime step "
            "are required"
        )
        return
    actual_steps = [
        step.get("step") if isinstance(step, dict) else None for step in trajectory
    ]
    expected_steps = list(range(total_steps + 1))
    if actual_steps != expected_steps:
        errors.append(
            f"{location}.steps: full runtime trajectory must cover 0 through "
            f"{total_steps} exactly"
        )

    diagnostics_by_step: dict[int, dict[str, Any]] = {}
    previous_counters: dict[str, int] | None = None
    for index, step in enumerate(trajectory):
        step_location = f"{location}.steps[{index}]"
        if not isinstance(step, dict):
            errors.append(f"{step_location}: expected an object")
            continue
        step_number = _integer(step.get("step"))
        diagnostics = step.get("solver_diagnostics")
        previous_counters = _validate_runtime_solver_diagnostics(
            diagnostics,
            f"{step_location}.solver_diagnostics",
            errors,
            previous_counters,
        )
        if step_number is not None and isinstance(diagnostics, dict):
            diagnostics_by_step[step_number] = diagnostics

    if trajectory and isinstance(trajectory[-1], dict):
        if data.get("solver_diagnostics") != trajectory[-1].get("solver_diagnostics"):
            errors.append(
                f"{location}.solver_diagnostics: final summary must equal the "
                "last runtime step diagnostics"
            )

    actions = data.get("actions")
    if not isinstance(actions, list) or any(
        not isinstance(action, dict) or action.get("success") is not True
        for action in actions
    ):
        errors.append(f"{location}.actions: every recorded action must succeed")
    shots = data.get("shots")
    if not isinstance(shots, list) or not shots:
        errors.append(f"{location}.shots: successful media captures are required")
        return
    for index, shot in enumerate(shots):
        shot_location = f"{location}.shots[{index}]"
        if not isinstance(shot, dict) or shot.get("success") is not True:
            errors.append(f"{shot_location}: expected a successful runtime capture")
            continue
        shot_step = _integer(shot.get("step"))
        if shot_step not in diagnostics_by_step:
            errors.append(f"{shot_location}.step: expected a captured runtime step")
        elif shot.get("solver_diagnostics") != diagnostics_by_step[shot_step]:
            errors.append(
                f"{shot_location}.solver_diagnostics: must equal the captured "
                "runtime step diagnostics"
            )
        media_path = _artifact_path(
            shot.get("path"), f"{shot_location}.path", repo_root, errors
        )
        expected_hash = shot.get("sha256")
        if not isinstance(expected_hash, str) or not SHA256_PATTERN.fullmatch(
            expected_hash
        ):
            errors.append(
                f"{shot_location}.sha256: runtime media requires lowercase SHA-256"
            )
        elif media_path is not None and _sha256(media_path) != expected_hash:
            errors.append(f"{shot_location}.sha256: runtime media digest mismatch")


def _validate_golden_diff(
    data: dict[str, Any],
    requirement_id: str,
    location: str,
    errors: list[str],
) -> None:
    _require_requirement_binding(data, requirement_id, location, errors)
    different_pixels = _integer(data.get("different_pixels"))
    total_pixels = _integer(data.get("total_pixels"))
    allowed = _number(data.get("allowed_difference_fraction"))
    declared_fraction = _number(data.get("difference_fraction"))
    if different_pixels is None or different_pixels < 0:
        errors.append(f"{location}.different_pixels: expected a non-negative integer")
        return
    if total_pixels is None or total_pixels <= 0 or different_pixels > total_pixels:
        errors.append(f"{location}.total_pixels: expected a valid positive total")
        return
    if allowed is None or not 0.0 <= allowed <= 1.0:
        errors.append(
            f"{location}.allowed_difference_fraction: expected a value in [0, 1]"
        )
        return
    computed_fraction = different_pixels / total_pixels
    if declared_fraction is None or not math.isclose(
        declared_fraction, computed_fraction, rel_tol=1e-12, abs_tol=1e-15
    ):
        errors.append(
            f"{location}.difference_fraction: expected computed value "
            f"{computed_fraction}"
        )
    computed_pass = computed_fraction <= allowed
    if data.get("pass") is not computed_pass:
        errors.append(f"{location}.pass: expected computed value {computed_pass!r}")
    if not computed_pass:
        errors.append(f"{location}: golden comparison did not pass")


def _validate_pass_collection(
    data: dict[str, Any],
    field: str,
    location: str,
    errors: list[str],
) -> bool:
    items = data.get(field)
    if not isinstance(items, list) or not items:
        errors.append(f"{location}.{field}: expected a non-empty list")
        return False
    item_passes = [
        item.get("pass") if isinstance(item, dict) else None for item in items
    ]
    if any(not isinstance(value, bool) for value in item_passes):
        errors.append(f"{location}.{field}: every item requires a boolean pass")
        return False
    computed_pass = all(item_passes)
    if data.get("pass") is not computed_pass:
        errors.append(f"{location}.pass: expected computed value {computed_pass!r}")
    if not computed_pass:
        errors.append(f"{location}: computed payload verdict failed")
    return computed_pass


def _validate_solver_validation(
    data: dict[str, Any], location: str, errors: list[str]
) -> None:
    solver = data.get("solver_validation")
    if not isinstance(solver, dict):
        errors.append(f"{location}.solver_validation: expected an object")
        return
    if solver.get("status") != "success":
        errors.append(f"{location}.solver_validation.status: expected 'success'")
    if _integer(solver.get("fallback_count")) != 0:
        errors.append(f"{location}.solver_validation.fallback_count: expected 0")
    if _integer(solver.get("exact_failure_count")) != 0:
        errors.append(f"{location}.solver_validation.exact_failure_count: expected 0")
    residual = _number(solver.get("maximum_residual"))
    if residual is None or residual < 0.0 or residual > PAPER_RESIDUAL_TOLERANCE:
        errors.append(
            f"{location}.solver_validation.maximum_residual: expected a finite "
            f"value <= {PAPER_RESIDUAL_TOLERANCE}"
        )


def _validate_structured_completion(
    kind: str,
    data: dict[str, Any],
    requirement_id: str,
    expected_claim: str,
    location: str,
    repo_root: Path,
    require_complete: bool,
    errors: list[str],
) -> None:
    expected_schema = STRUCTURED_SCHEMAS[kind]
    if data.get("schema_version") != expected_schema:
        errors.append(
            f"{location}.schema_version: expected {expected_schema!r}, "
            f"got {data.get('schema_version')!r}"
        )
        return
    if kind == "capture_sidecar":
        _validate_capture_sidecar(data, requirement_id, location, repo_root, errors)
        return
    if kind == "golden_diff":
        _validate_golden_diff(data, requirement_id, location, errors)
        return

    if kind == "claim_map":
        requirement_ids = data.get("requirement_ids")
        if requirement_ids != [requirement_id]:
            errors.append(
                f"{location}.requirement_ids: claim map must bind exactly "
                f"[{requirement_id!r}]"
            )
        if data.get("manifest_claim") != expected_claim:
            errors.append(
                f"{location}.manifest_claim: must exactly match the manifest claim"
            )
        status = data.get("status")
        if status not in {"partial", "complete"}:
            errors.append(f"{location}.status: expected 'partial' or 'complete'")
        if require_complete and status != "complete":
            errors.append(f"{location}.status: complete claim map is required")
        remaining_blockers = data.get("remaining_blockers")
        if not isinstance(remaining_blockers, list) or any(
            not _nonempty_string(blocker) for blocker in remaining_blockers
        ):
            errors.append(
                f"{location}.remaining_blockers: expected a list of non-empty strings"
            )
        if require_complete and remaining_blockers != []:
            errors.append(f"{location}.remaining_blockers: expected an empty list")
        claims = data.get("claims", data.get("visual_claims"))
        if not isinstance(claims, list) or not claims:
            errors.append(f"{location}: claim map requires at least one claim")
        else:
            exact_claim_has_support = False
            for index, claim in enumerate(claims):
                if not isinstance(claim, dict):
                    errors.append(f"{location}.claims[{index}]: expected an object")
                    continue
                claim_text = claim.get("claim")
                support = claim.get("support")
                if not _nonempty_string(claim_text):
                    errors.append(
                        f"{location}.claims[{index}].claim: expected non-empty text"
                    )
                if not _nonempty_string(support):
                    errors.append(
                        f"{location}.claims[{index}].support: expected non-empty "
                        "evidence semantics"
                    )
                if claim_text == expected_claim and _nonempty_string(support):
                    exact_claim_has_support = True
            if not exact_claim_has_support:
                errors.append(
                    f"{location}: expected the exact manifest claim with non-empty "
                    "support semantics"
                )
        return

    _require_requirement_binding(data, requirement_id, location, errors)
    if kind == "citation_record":
        status = data.get("status")
        if status not in {"partial", "complete"}:
            errors.append(f"{location}.status: expected 'partial' or 'complete'")
        if require_complete and status != "complete":
            errors.append(f"{location}.status: expected 'complete'")
        sources = data.get("sources")
        if not isinstance(sources, list) or not sources:
            errors.append(f"{location}.sources: expected verified citations")
        else:
            for index, source in enumerate(sources):
                if (
                    not isinstance(source, dict)
                    or not _nonempty_string(source.get("url"))
                    or not _nonempty_string(source.get("locator"))
                    or source.get("verified") is not True
                ):
                    errors.append(
                        f"{location}.sources[{index}]: expected a verified URL and locator"
                    )
    elif kind == "raw_data":
        records = data.get("records")
        if not isinstance(records, list) or not records:
            errors.append(f"{location}.records: expected non-empty raw records")
    else:
        status = data.get("status")
        if status not in {"partial", "complete"}:
            errors.append(f"{location}.status: expected 'partial' or 'complete'")
        if require_complete and status != "complete":
            errors.append(f"{location}.status: expected 'complete'")
        collection_field = {
            "external_baseline": "comparisons",
            "outcome_report": "outcomes",
            "performance_report": "benchmarks",
        }[kind]
        _validate_pass_collection(data, collection_field, location, errors)
        if kind in {"outcome_report", "performance_report"}:
            _validate_solver_validation(data, location, errors)
        if (
            kind == "external_baseline"
            and require_complete
            and data.get("paper_matched") is not True
        ):
            errors.append(f"{location}.paper_matched: expected true")


def _parse_csv_int(value: str | None, location: str, errors: list[str]) -> int | None:
    try:
        result = int(value or "")
    except ValueError:
        errors.append(f"{location}: expected an integer, got {value!r}")
        return None
    return result


def _parse_csv_number(
    value: str | None, location: str, errors: list[str]
) -> float | None:
    try:
        result = float(value or "")
    except ValueError:
        errors.append(f"{location}: expected a number, got {value!r}")
        return None
    if not math.isfinite(result):
        errors.append(f"{location}: expected a finite number, got {value!r}")
        return None
    return result


def _csv_bool(value: str | None) -> bool | None:
    normalized = (value or "").strip().lower()
    if normalized in {"1", "true"}:
        return True
    if normalized in {"0", "false"}:
        return False
    return None


def _validate_trace_csv(
    parsed: tuple[list[str], list[dict[str, str]]],
    location: str,
    errors: list[str],
) -> None:
    fieldnames, rows = parsed
    required = {"contacts", "residual", "step"}
    missing = sorted(required - set(fieldnames))
    fallback_field = next(
        (
            field
            for field in (
                "fallbacks",
                "step_fallbacks",
                "boxed_lcp_fallbacks_delta",
            )
            if field in fieldnames
        ),
        None,
    )
    status_field = next(
        (field for field in ("status", "exact_status") if field in fieldnames),
        None,
    )
    if fallback_field is None:
        missing.append("fallbacks|step_fallbacks|boxed_lcp_fallbacks_delta")
    if status_field is None:
        missing.append("status|exact_status")
    if missing:
        errors.append(f"{location}: trace CSV is missing columns {missing}")
        return
    assert fallback_field is not None
    assert status_field is not None
    steps: list[int] = []
    for index, row in enumerate(rows):
        row_location = f"{location}.rows[{index}]"
        step = _parse_csv_int(row.get("step"), f"{row_location}.step", errors)
        contacts = _parse_csv_int(
            row.get("contacts"), f"{row_location}.contacts", errors
        )
        fallbacks = _parse_csv_int(
            row.get(fallback_field),
            f"{row_location}.{fallback_field}",
            errors,
        )
        if step is None or contacts is None or fallbacks is None:
            continue
        steps.append(step)
        if contacts < 0:
            errors.append(f"{row_location}.contacts: expected a non-negative value")
        if fallbacks != 0:
            errors.append(f"{row_location}.fallbacks: completion requires 0")
        status = row.get(status_field)
        if step == 0 and status == "not_run":
            continue
        if contacts > 0 and status != "success":
            errors.append(
                f"{row_location}.{status_field}: contact step must be successful"
            )
        residual_text = row.get("residual")
        if contacts > 0 or (residual_text or "").strip().lower() not in {"", "nan"}:
            residual = _parse_csv_number(
                residual_text, f"{row_location}.residual", errors
            )
            if residual is not None and (
                residual < 0.0 or residual > PAPER_RESIDUAL_TOLERANCE
            ):
                errors.append(
                    f"{row_location}.residual: {residual} exceeds "
                    f"{PAPER_RESIDUAL_TOLERANCE}"
                )
    if steps != list(range(len(rows))):
        errors.append(
            f"{location}: trace must contain every step in order from 0; got {steps}"
        )


def _validate_performance_csv(
    parsed: tuple[list[str], list[dict[str, str]]],
    location: str,
    errors: list[str],
) -> None:
    fieldnames, rows = parsed
    required = {
        "all_solver_steps_successful",
        "complete_requested_trajectory_evidence",
        "exact_failures",
        "fallbacks",
        "full_trajectory_evidence",
        "max_residual",
        "paper_mean_target_met",
        "paper_target_evaluated",
        "paper_timing_comparable",
        "paper_workload_contract_valid",
        "scenario",
    }
    missing = sorted(required - set(fieldnames))
    if missing:
        errors.append(f"{location}: performance CSV is missing columns {missing}")
        return
    for index, row in enumerate(rows):
        row_location = f"{location}.rows[{index}]"
        if not _nonempty_string(row.get("scenario")):
            errors.append(f"{row_location}.scenario: expected a non-empty value")
        for field in (
            "all_solver_steps_successful",
            "complete_requested_trajectory_evidence",
            "full_trajectory_evidence",
            "paper_mean_target_met",
            "paper_target_evaluated",
            "paper_timing_comparable",
            "paper_workload_contract_valid",
        ):
            value = _csv_bool(row.get(field))
            if value is not True:
                errors.append(f"{row_location}.{field}: completion requires true")
        for field in ("exact_failures", "fallbacks"):
            value = _parse_csv_int(row.get(field), f"{row_location}.{field}", errors)
            if value is not None and value != 0:
                errors.append(f"{row_location}.{field}: completion requires 0")
        residual = _parse_csv_number(
            row.get("max_residual"), f"{row_location}.max_residual", errors
        )
        if residual is not None and (
            residual < 0.0 or residual > PAPER_RESIDUAL_TOLERANCE
        ):
            errors.append(
                f"{row_location}.max_residual: {residual} exceeds "
                f"{PAPER_RESIDUAL_TOLERANCE}"
            )


def _validate_validated_deliverable(
    kind: str,
    deliverable: dict[str, Any],
    path: Path | None,
    parsed: dict[str, Any] | tuple[list[str], list[dict[str, str]]] | None,
    requirement_id: str,
    expected_claim: str,
    location: str,
    repo_root: Path,
    require_complete: bool,
    errors: list[str],
) -> None:
    expected_hash = deliverable.get("sha256")
    if not isinstance(expected_hash, str) or not SHA256_PATTERN.fullmatch(
        expected_hash
    ):
        errors.append(
            f"{location}.sha256: validated deliverable requires lowercase SHA-256"
        )
    elif path is not None:
        actual_hash = _sha256(path)
        if actual_hash != expected_hash:
            errors.append(
                f"{location}.sha256: digest mismatch; expected {expected_hash}, "
                f"computed {actual_hash}"
            )

    if kind == "raw_data" and isinstance(parsed, tuple):
        return
    if kind in STRUCTURED_SCHEMAS:
        if not isinstance(parsed, dict):
            errors.append(f"{location}: validated structured deliverable is unreadable")
        else:
            _validate_structured_completion(
                kind,
                parsed,
                requirement_id,
                expected_claim,
                location,
                repo_root,
                require_complete,
                errors,
            )
    elif kind == "trace_csv":
        if not isinstance(parsed, tuple):
            errors.append(f"{location}: validated trace CSV is unreadable")
        else:
            _validate_trace_csv(parsed, location, errors)
    elif kind == "performance_csv":
        if not isinstance(parsed, tuple):
            errors.append(f"{location}: validated performance CSV is unreadable")
        else:
            _validate_performance_csv(parsed, location, errors)


def _validate_prior_source_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.prior_source_strict_card_paper_cpu"
    data = _object(
        current_truth.get("prior_source_strict_card_paper_cpu"), location, errors
    )
    if data is None:
        return

    bundle = _validate_current_path(
        data,
        "bundle",
        PRIOR_SOURCE_ARCHIVE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    source_status = data.get("source_status")
    if not _nonempty_string(source_status) or not all(
        label in source_status.casefold() for label in ("historical", "prior-source")
    ):
        errors.append(
            f"{location}.source_status: historical prior-source labeling is required"
        )
    requested = _integer(data.get("requested_steps"))
    completed = _integer(data.get("completed_steps"))
    if requested is None or requested <= 0:
        errors.append(f"{location}.requested_steps: expected a positive integer")
    if completed is None or completed < 0:
        errors.append(f"{location}.completed_steps: expected a non-negative integer")
    elif requested is not None and completed >= requested:
        errors.append(f"{location}: prior-source trajectory must remain incomplete")
    failures = _integer(data.get("exact_fbf_failures"))
    if failures is None or failures < 1:
        errors.append(f"{location}.exact_fbf_failures: expected at least one failure")
    _expect_fields(
        data,
        {
            "boxed_lcp_fallbacks": 0,
            "convergence_verdict": False,
            "timing_verdict": None,
        },
        location,
        errors,
    )
    trajectory = data.get("trajectory_verdict")
    if not _nonempty_string(trajectory) or not all(
        label in trajectory.casefold() for label in ("failed", "incomplete")
    ):
        errors.append(
            f"{location}.trajectory_verdict: expected failed and incomplete labeling"
        )

    trace_hash = data.get("trace_executable_sha256")
    if not isinstance(trace_hash, str) or not SHA256_PATTERN.fullmatch(trace_hash):
        errors.append(f"{location}.trace_executable_sha256: expected lowercase SHA-256")
    hashes = _validate_artifact_hashes(
        data, PRIOR_SOURCE_ARTIFACT_TARGETS, location, repo_root, errors
    )
    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    if metadata is not None:
        binary = _object(metadata.get("binary"), f"{location}.metadata.binary", errors)
        if binary is not None and isinstance(trace_hash, str):
            _expect_fields(
                binary,
                {"sha256": trace_hash},
                f"{location}.metadata.binary",
                errors,
            )
        configuration = _object(
            metadata.get("configuration"),
            f"{location}.metadata.configuration",
            errors,
        )
        if configuration is not None:
            _expect_fields(
                configuration,
                {"contract": "paper_cpu", "solver": "exact_fbf"},
                f"{location}.metadata.configuration",
                errors,
            )
        argv = metadata.get("argv")
        if not isinstance(argv, list) or any(
            not _nonempty_string(argument) for argument in argv
        ):
            errors.append(f"{location}.metadata.argv: expected non-empty strings")
        else:
            output_flags = [
                index
                for index, argument in enumerate(argv)
                if argument == "--output-dir"
            ]
            if len(output_flags) != 1 or output_flags[0] + 1 >= len(argv):
                errors.append(
                    f"{location}.metadata.argv: expected one --output-dir value"
                )
            else:
                _expect_fields(
                    data,
                    {"original_output_path": argv[output_flags[0] + 1]},
                    location,
                    errors,
                )

    artifact_index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )
    if artifact_index is not None:
        _expect_fields(
            artifact_index,
            {"schema_version": "dart.fbf_prior_source_archive/v1"},
            f"{location}.artifact_index",
            errors,
        )
        files = _object(
            artifact_index.get("files"), f"{location}.artifact_index.files", errors
        )
        if files is not None:
            expected_files = set(PRIOR_SOURCE_ARTIFACT_TARGETS) - {
                "artifact-index.json"
            }
            indexed_files = {key for key in files if isinstance(key, str)}
            if indexed_files != expected_files:
                errors.append(
                    f"{location}.artifact_index.files: expected exact membership "
                    f"{sorted(expected_files)}, got {sorted(indexed_files)}"
                )
            if any(not isinstance(key, str) for key in files):
                errors.append(f"{location}.artifact_index.files: keys must be strings")
            if bundle is not None:
                disk_files = {
                    path.relative_to(bundle).as_posix()
                    for path in bundle.rglob("*")
                    if path.is_file()
                    and path.relative_to(bundle).as_posix() != "artifact-index.json"
                }
                if disk_files != expected_files:
                    errors.append(
                        f"{location}.archive: expected exact file membership "
                        f"{sorted(expected_files)}, got {sorted(disk_files)}"
                    )
            for key in sorted(expected_files):
                expected_hash = hashes.get(key)
                entry = _object(
                    files.get(key), f"{location}.artifact_index.files[{key!r}]", errors
                )
                if entry is not None:
                    expected = {"sha256": expected_hash}
                    path = bundle / key if bundle is not None else None
                    if path is not None and path.is_file():
                        expected["size_bytes"] = path.stat().st_size
                    _expect_fields(
                        entry,
                        expected,
                        f"{location}.artifact_index.files[{key!r}]",
                        errors,
                    )

    if bundle is not None:
        invocations = _read_json_array(
            bundle / "invocations.json", f"{location}.invocations", errors
        )
        if invocations is not None:
            if len(invocations) != 1 or not isinstance(invocations[0], dict):
                errors.append(f"{location}.invocations: expected one process record")
            else:
                invocation = invocations[0]
                _expect_fields(
                    invocation,
                    {
                        "complete_rows": False,
                        "expected_rows": data.get("requested_steps"),
                        "returncode": 1,
                        "rows": data.get("completed_steps"),
                        "steps": data.get("requested_steps"),
                        "timed_out": False,
                        "warmup": False,
                    },
                    f"{location}.invocations[0]",
                    errors,
                )

        summary = _read_json_array(
            bundle / "summary.json", f"{location}.summary", errors
        )
        if summary is not None:
            if len(summary) != 1 or not isinstance(summary[0], dict):
                errors.append(f"{location}.summary: expected one result row")
            else:
                row = summary[0]
                contact_range = data.get("contact_range")
                if not isinstance(contact_range, list) or len(contact_range) != 2:
                    contact_range = [None, None]
                _expect_fields(
                    row,
                    {
                        "sample_steps": data.get("completed_steps"),
                        "steps_per_repetition": str(data.get("completed_steps")),
                        "min_contacts": contact_range[0],
                        "max_contacts": contact_range[1],
                        "max_outer_iterations": str(
                            data.get("outer_cap_per_exact_group")
                        ),
                        "exact_failures": data.get("exact_fbf_failures"),
                        "fallbacks": data.get("boxed_lcp_fallbacks"),
                        "failed_processes": 1,
                        "all_solver_steps_accepted": False,
                        "all_solver_steps_successful": False,
                        "complete_requested_trajectory_evidence": False,
                    },
                    f"{location}.summary[0]",
                    errors,
                )
                summary_mean = _number(row.get("mean_step_ms"))
                declared_mean = _number(data.get("mean_step_ms_before_failure"))
                if (
                    summary_mean is None
                    or declared_mean is None
                    or not math.isclose(
                        summary_mean, declared_mean, rel_tol=1e-6, abs_tol=1e-6
                    )
                ):
                    errors.append(
                        f"{location}.mean_step_ms_before_failure: bound summary "
                        "value drifted"
                    )

        raw = _read_csv(bundle / "raw.csv", f"{location}.raw", errors)
        if raw is not None:
            _, rows = raw
            completed_steps = _integer(data.get("completed_steps"))
            if completed_steps is None or len(rows) != completed_steps:
                errors.append(
                    f"{location}.raw: expected {completed_steps!r} rows, got {len(rows)}"
                )
            elif rows:
                steps = [
                    _parse_csv_int(row.get("step"), f"{location}.raw.step", errors)
                    for row in rows
                ]
                if steps != list(range(1, completed_steps + 1)):
                    errors.append(
                        f"{location}.raw.step: expected contiguous failed prefix"
                    )
                contacts = [
                    _parse_csv_int(
                        row.get("contacts"), f"{location}.raw.contacts", errors
                    )
                    for row in rows
                ]
                valid_contacts = [value for value in contacts if value is not None]
                declared_range = data.get("contact_range")
                if (
                    not isinstance(declared_range, list)
                    or len(declared_range) != 2
                    or not valid_contacts
                    or [min(valid_contacts), max(valid_contacts)] != declared_range
                ):
                    errors.append(f"{location}.contact_range: bound raw range drifted")
                for field, expected in (
                    (
                        "max_outer_iterations",
                        str(data.get("outer_cap_per_exact_group")),
                    ),
                    ("step_fallbacks", str(data.get("boxed_lcp_fallbacks"))),
                ):
                    observed = {row.get(field) for row in rows}
                    if observed != {expected}:
                        errors.append(
                            f"{location}.raw.{field}: expected only {expected!r}, "
                            f"got {sorted(repr(value) for value in observed)}"
                        )
                tolerances = [
                    _parse_csv_number(
                        row.get("tolerance"), f"{location}.raw.tolerance", errors
                    )
                    for row in rows
                ]
                declared_tolerance = _number(data.get("tolerance"))
                if declared_tolerance is None or any(
                    value is None
                    or not math.isclose(
                        value,
                        declared_tolerance,
                        rel_tol=1e-12,
                        abs_tol=1e-15,
                    )
                    for value in tolerances
                ):
                    errors.append(f"{location}.tolerance: bound raw value drifted")
                failures = sum(
                    _parse_csv_int(
                        row.get("step_exact_failures"),
                        f"{location}.raw.step_exact_failures",
                        errors,
                    )
                    or 0
                    for row in rows
                )
                caps = sum(
                    row.get("status") == "max_iterations_accepted" for row in rows
                )
                if failures != data.get("exact_fbf_failures"):
                    errors.append(
                        f"{location}.exact_fbf_failures: bound raw total drifted"
                    )
                if caps != data.get("accepted_cap_steps"):
                    errors.append(
                        f"{location}.accepted_cap_steps: bound raw total drifted"
                    )
                terminal = rows[-1]
                terminal_iterations = _parse_csv_int(
                    terminal.get("step_fbf_iterations"),
                    f"{location}.raw.terminal.step_fbf_iterations",
                    errors,
                )
                terminal_residual = _parse_csv_number(
                    terminal.get("residual"),
                    f"{location}.raw.terminal.residual",
                    errors,
                )
                if terminal_iterations != data.get("terminal_step_fbf_iterations"):
                    errors.append(
                        f"{location}.terminal_step_fbf_iterations: bound raw value drifted"
                    )
                declared_residual = _number(data.get("terminal_residual"))
                if (
                    terminal_residual is None
                    or declared_residual is None
                    or not math.isclose(
                        terminal_residual,
                        declared_residual,
                        rel_tol=1e-12,
                        abs_tol=1e-15,
                    )
                ):
                    errors.append(
                        f"{location}.terminal_residual: bound raw value drifted"
                    )
                wall_values = [
                    _parse_csv_number(
                        row.get("wall_ms"), f"{location}.raw.wall_ms", errors
                    )
                    for row in rows
                ]
                valid_wall = [value for value in wall_values if value is not None]
                declared_mean = _number(data.get("mean_step_ms_before_failure"))
                if (
                    len(valid_wall) != len(rows)
                    or declared_mean is None
                    or not math.isclose(
                        sum(valid_wall) / len(valid_wall),
                        declared_mean,
                        rel_tol=1e-6,
                        abs_tol=1e-6,
                    )
                ):
                    errors.append(
                        f"{location}.mean_step_ms_before_failure: bound raw mean drifted"
                    )


def _validate_card_v2_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.card_house_manifold_sensitivity_v2_nonpaper"
    data = _object(
        current_truth.get("card_house_manifold_sensitivity_v2_nonpaper"),
        location,
        errors,
    )
    if data is None:
        return

    bundle = _validate_current_path(
        data,
        "bundle",
        CARD_V2_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    protocol = _validate_current_path(
        data,
        "protocol",
        CARD_V2_PROTOCOL,
        location,
        repo_root,
        errors,
    )
    _validate_current_path(data, "runner", CARD_V2_RUNNER, location, repo_root, errors)
    _expect_fields(
        data,
        {
            "status": "valid_current_source_comparison_both_trajectories_nonstrict",
            "paper_parity": False,
            "comparison_artifact_integrity_valid": True,
            "manifold_is_only_intended_factor": True,
            "directional_contact_multiplicity_hypothesis_supported": True,
            "timing_used_in_verdict": False,
            "timing_verdict": None,
            "physical_verdict": None,
        },
        location,
        errors,
    )
    for mode in ("compact", "four_point_planar"):
        mode_data = _object(data.get(mode), f"{location}.{mode}", errors)
        if mode_data is not None:
            _expect_fields(
                mode_data,
                {
                    "emitted_steps": 600,
                    "strict_trajectory_valid": False,
                    "strict_success_rows": 0,
                    "exact_failures": 0,
                    "boxed_lcp_fallbacks": 0,
                },
                f"{location}.{mode}",
                errors,
            )

    protocol_hash = _contract_sha256(
        protocol, "## Frozen v2 result", f"{location}.protocol", errors
    )
    hashes = _validate_artifact_hashes(
        data,
        CARD_V2_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        computed_hashes={"protocol_contract": protocol_hash},
        recorded_hash_keys={"trace_binary"},
    )
    expected_identity_rechecks = ["after_compact", "after_four_point_planar"]
    invocation = _read_bundle_json(
        bundle, "invocation.json", f"{location}.invocation", errors
    )
    if invocation is not None:
        _expect_fields(
            invocation,
            {"identity_rechecks": expected_identity_rechecks},
            f"{location}.invocation",
            errors,
        )

    summary = _read_bundle_json(bundle, "summary.json", f"{location}.summary", errors)
    if summary is not None:
        _expect_fields(
            summary,
            {"comparison_artifact_integrity_valid": True, "error": None},
            f"{location}.summary",
            errors,
        )
        modes = _object(summary.get("modes"), f"{location}.summary.modes", errors)
        if modes is not None:
            for mode in ("compact", "four_point_planar"):
                summary_mode = _object(
                    modes.get(mode), f"{location}.summary.modes.{mode}", errors
                )
                if summary_mode is not None:
                    _expect_fields(
                        summary_mode,
                        {
                            "strict_trajectory_valid": False,
                            "strict_success_rows": 0,
                            "physical_outcome_verdict": None,
                            "timing_verdict": None,
                            "exact_failures": 0,
                            "boxed_lcp_fallbacks": 0,
                        },
                        f"{location}.summary.modes.{mode}",
                        errors,
                    )
                    truth_mode = _object(data.get(mode), f"{location}.{mode}", errors)
                    if truth_mode is not None:
                        contacts = _object(
                            summary_mode.get("contacts"),
                            f"{location}.summary.modes.{mode}.contacts",
                            errors,
                        )
                        pairs = _object(
                            summary_mode.get("unique_pairs"),
                            f"{location}.summary.modes.{mode}.unique_pairs",
                            errors,
                        )
                        pair_graph = _object(
                            summary_mode.get("pair_graph"),
                            f"{location}.summary.modes.{mode}.pair_graph",
                            errors,
                        )
                        terminal = _object(
                            summary_mode.get("terminal"),
                            f"{location}.summary.modes.{mode}.terminal",
                            errors,
                        )
                        expected = {
                            "process_exit_class": summary_mode.get(
                                "process_exit_class"
                            ),
                            "emitted_steps": summary_mode.get("emitted_steps"),
                            "strict_trajectory_valid": summary_mode.get(
                                "strict_trajectory_valid"
                            ),
                            "strict_success_rows": summary_mode.get(
                                "strict_success_rows"
                            ),
                            "accepted_cap_exact_groups": summary_mode.get(
                                "accepted_cap_exact_groups"
                            ),
                            "exact_attempts": summary_mode.get("exact_attempts"),
                            "exact_failures": summary_mode.get("exact_failures"),
                            "boxed_lcp_fallbacks": summary_mode.get(
                                "boxed_lcp_fallbacks"
                            ),
                        }
                        if contacts is not None:
                            expected.update(
                                {
                                    "contact_range": [
                                        contacts.get("minimum"),
                                        contacts.get("maximum"),
                                    ],
                                    "mean_contacts": contacts.get("mean"),
                                }
                            )
                        if pairs is not None:
                            expected["pair_range"] = [
                                pairs.get("minimum"),
                                pairs.get("maximum"),
                            ]
                        if terminal is not None:
                            expected.update(
                                {
                                    "terminal_last_group_status": terminal.get(
                                        "internal_fbf_status"
                                    ),
                                    "terminal_last_group_residual": terminal.get(
                                        "residual"
                                    ),
                                }
                            )
                        if pair_graph is not None:
                            expected.update(
                                {
                                    "pair_identity_transition_rows": pair_graph.get(
                                        "identity_transition_rows"
                                    ),
                                    "pair_multiplicity_transition_rows": pair_graph.get(
                                        "multiplicity_transition_rows"
                                    ),
                                }
                            )
                        _expect_fields(
                            truth_mode,
                            expected,
                            f"{location}.{mode}",
                            errors,
                        )
                        multiplicity = (
                            pair_graph.get("multiplicity")
                            if isinstance(pair_graph, dict)
                            else None
                        )
                        artifact_mean = (
                            multiplicity.get("mean")
                            if isinstance(multiplicity, dict)
                            else None
                        )
                        declared_mean = _number(
                            truth_mode.get("mean_pair_multiplicity")
                        )
                        if (
                            _number(artifact_mean) is None
                            or declared_mean is None
                            or not math.isclose(
                                declared_mean,
                                float(artifact_mean),
                                rel_tol=1e-9,
                                abs_tol=1e-12,
                            )
                        ):
                            errors.append(
                                f"{location}.{mode}.mean_pair_multiplicity: "
                                "bound summary value drifted"
                            )

    comparison = _read_bundle_json(
        bundle, "comparison.json", f"{location}.comparison", errors
    )
    if comparison is not None:
        _expect_fields(
            comparison,
            {
                "comparison_artifact_integrity_valid": True,
                "manifold_is_only_intended_factor": True,
                "paper_parity": False,
                "timing_used_in_verdict": False,
                "strict_trajectory_valid": {
                    "compact": False,
                    "four_point_planar": False,
                },
            },
            f"{location}.comparison",
            errors,
        )
        deltas = _object(
            comparison.get("deltas"), f"{location}.comparison.deltas", errors
        )
        truth_deltas = _object(
            data.get("four_point_planar_minus_compact"),
            f"{location}.four_point_planar_minus_compact",
            errors,
        )
        if deltas is not None and truth_deltas is not None:
            _expect_fields(
                truth_deltas,
                {
                    "mean_contacts": deltas.get("mean_contacts"),
                    "accepted_cap_exact_groups": deltas.get(
                        "accepted_cap_exact_groups"
                    ),
                    "exact_attempts": deltas.get("exact_attempts"),
                    "terminal_last_group_residual": deltas.get("terminal_residual"),
                    "pair_identity_transition_rows": deltas.get(
                        "pair_identity_transition_rows"
                    ),
                    "pair_multiplicity_transition_rows": deltas.get(
                        "pair_multiplicity_transition_rows"
                    ),
                },
                f"{location}.four_point_planar_minus_compact",
                errors,
            )
            multiplicity_delta = _number(deltas.get("mean_pair_multiplicity"))
            declared_delta = _number(truth_deltas.get("mean_pair_multiplicity"))
            if (
                multiplicity_delta is None
                or declared_delta is None
                or not math.isclose(
                    declared_delta,
                    multiplicity_delta,
                    rel_tol=1e-12,
                    abs_tol=1e-12,
                )
            ):
                errors.append(
                    f"{location}.four_point_planar_minus_compact."
                    "mean_pair_multiplicity: bound comparison value drifted"
                )
            if (
                _number(deltas.get("mean_contacts")) is None
                or float(deltas["mean_contacts"]) <= 0.0
                or multiplicity_delta is None
                or multiplicity_delta <= 0.0
            ):
                errors.append(
                    f"{location}.comparison: directional contact-multiplicity "
                    "hypothesis requires both deltas to be positive"
                )

    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    if metadata is not None:
        _expect_fields(
            metadata,
            {
                "protocol": CARD_V2_PROTOCOL,
                "runner": CARD_V2_RUNNER,
                "comparison_artifact_integrity_valid": True,
                "paper_parity": False,
                "timing_verdict": None,
            },
            f"{location}.metadata",
            errors,
        )
        source_identity = _object(
            metadata.get("source_identity"),
            f"{location}.metadata.source_identity",
            errors,
        )
        if source_identity is not None:
            trace_executable = _validate_structured_executable_identity(
                source_identity.get("trace_executable"),
                f"{location}.metadata.source_identity.trace_executable",
                repo_root,
                errors,
                expected_sha256=hashes.get("trace_binary"),
            )
            expected = {
                "identity_helper_source_sha256": hashes.get("identity_helper_source"),
                "protocol_contract_sha256": hashes.get("protocol_contract"),
                "runner_source_sha256": hashes.get("runner"),
                "trace_source_sha256": hashes.get("trace_source"),
            }
            _expect_fields(
                source_identity,
                {key: value for key, value in expected.items() if value is not None},
                f"{location}.metadata.source_identity",
                errors,
            )
            taskset_tool = _validate_tool_identity(
                source_identity.get("taskset_tool"),
                "taskset",
                f"{location}.metadata.source_identity.taskset_tool",
                errors,
            )
            if trace_executable is not None:
                _expect_fields(
                    metadata,
                    {
                        "binary": trace_executable.get("path"),
                        "trace_source": TRACE_SOURCE,
                    },
                    f"{location}.metadata",
                    errors,
                )
            if (
                invocation is not None
                and trace_executable is not None
                and taskset_tool is not None
            ):
                commands = _object(
                    invocation.get("commands"),
                    f"{location}.invocation.commands",
                    errors,
                )
                if commands is not None:
                    if set(commands) != {"compact", "four_point_planar"}:
                        errors.append(
                            f"{location}.invocation.commands: exact mode "
                            "membership changed"
                        )
                    for mode in ("compact", "four_point_planar"):
                        command = commands.get(mode)
                        command_location = f"{location}.invocation.commands.{mode}"
                        if not isinstance(command, list) or len(command) != 19:
                            errors.append(f"{command_location}: expected 19 arguments")
                            continue
                        _validate_recorded_tool_command(
                            command[0],
                            taskset_tool,
                            f"{command_location}[0]",
                            errors,
                        )
                        _expect_exact_payload(
                            command[1:],
                            [
                                "--cpu-list",
                                "8",
                                trace_executable.get("path"),
                                "card_house_26_settle_projectile_full",
                                "exact_fbf",
                                "1",
                                "600",
                                "nan",
                                "performance",
                                "default",
                                "default",
                                "1",
                                "paper_cpu",
                                "native",
                                "default",
                                "0",
                                "0",
                                mode,
                            ],
                            f"{command_location}[1:]",
                            errors,
                        )
            runtime = _object(
                data.get("runtime_provenance"),
                f"{location}.runtime_provenance",
                errors,
            )
            if runtime is not None and set(runtime) != {
                "scope",
                "identity_rechecks",
                "trace_ldd_map_sha256",
                "resolved_build_libdart_sha256",
                "ldd_tool_sha256",
                "taskset_tool_sha256",
                "trace_resolved_regular_shared_libraries",
            }:
                errors.append(
                    f"{location}.runtime_provenance: exact runtime-provenance "
                    "contract changed"
                )
            if (
                runtime is not None
                and trace_executable is not None
                and taskset_tool is not None
            ):
                libdart = trace_executable.get("resolved_build_libdart")
                ldd_tool = trace_executable.get("ldd_tool")
                _expect_fields(
                    runtime,
                    {
                        "scope": (
                            "Executed trace binary, ldd, taskset, and resolved "
                            "regular shared-library identities were recorded and "
                            "rechecked after both manifold traces; this is not a "
                            "claim over all host runtime state."
                        ),
                        "identity_rechecks": (
                            invocation.get("identity_rechecks")
                            if invocation is not None
                            else expected_identity_rechecks
                        ),
                        "trace_ldd_map_sha256": trace_executable.get(
                            "ldd_resolution_output_normalized_sha256"
                        ),
                        "resolved_build_libdart_sha256": (
                            libdart.get("sha256") if isinstance(libdart, dict) else None
                        ),
                        "ldd_tool_sha256": (
                            ldd_tool.get("sha256")
                            if isinstance(ldd_tool, dict)
                            else None
                        ),
                        "taskset_tool_sha256": taskset_tool.get("sha256"),
                        "trace_resolved_regular_shared_libraries": (
                            trace_executable.get(
                                "resolved_regular_shared_library_count"
                            )
                        ),
                    },
                    f"{location}.runtime_provenance",
                    errors,
                )
        artifact_sha256 = _object(
            metadata.get("artifact_sha256"),
            f"{location}.metadata.artifact_sha256",
            errors,
        )
        if artifact_sha256 is not None:
            for key in (
                "compact/raw.csv",
                "four_point_planar/raw.csv",
                "invocation.json",
                "summary.json",
                "comparison.json",
                "REPORT.md",
            ):
                if key in hashes:
                    _expect_fields(
                        artifact_sha256,
                        {key: hashes[key]},
                        f"{location}.metadata.artifact_sha256",
                        errors,
                    )


def _validate_cpu_artifact_index(
    bundle: Path, location: str, errors: list[str]
) -> None:
    index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )
    if index is None:
        return
    _expect_fields(
        index,
        {"schema_version": "dart.fbf_cpu_evidence_artifact_index/v1"},
        f"{location}.artifact_index",
        errors,
    )
    files = _object(index.get("files"), f"{location}.artifact_index.files", errors)
    if files is None:
        return
    indexed_files = {key for key in files if isinstance(key, str)}
    if any(not isinstance(key, str) for key in files):
        errors.append(f"{location}.artifact_index.files: keys must be strings")
    disk_files = {
        path.relative_to(bundle).as_posix()
        for path in bundle.rglob("*")
        if path.is_file()
        and path.relative_to(bundle).as_posix() != "artifact-index.json"
    }
    if indexed_files != disk_files:
        errors.append(
            f"{location}.artifact_index.files: expected exact disk membership "
            f"{sorted(disk_files)}, got {sorted(indexed_files)}"
        )
    for key in sorted(indexed_files & disk_files):
        entry = _object(
            files.get(key), f"{location}.artifact_index.files[{key!r}]", errors
        )
        if entry is None:
            continue
        path = bundle / key
        _expect_fields(
            entry,
            {"sha256": _sha256(path), "size_bytes": path.stat().st_size},
            f"{location}.artifact_index.files[{key!r}]",
            errors,
        )


def _validate_cpu_raw_claims(
    bundle: Path,
    data: dict[str, Any],
    metadata: dict[str, Any] | None,
    taskset_tool: dict[str, Any] | None,
    location: str,
    errors: list[str],
) -> None:
    expected_affinity: dict[int, dict[str, str]] = {}
    binary_path: str | None = None
    if metadata is not None:
        binary = metadata.get("binary")
        if isinstance(binary, dict) and _nonempty_string(binary.get("path")):
            binary_path = binary["path"]
    if binary_path is None:
        errors.append(f"{location}.metadata.binary.path: expected a non-empty string")
    invocations = _read_json_array(
        bundle / "invocations.json", f"{location}.invocations", errors
    )
    if invocations is not None:
        if len(invocations) != 8:
            errors.append(
                f"{location}.invocations: expected 8 processes, got {len(invocations)}"
            )
        invocation_groups: dict[tuple[int, bool], list[dict[str, Any]]] = {}
        expected_tool_rechecks: list[str] = []
        for index, invocation in enumerate(invocations):
            invocation_location = f"{location}.invocations[{index}]"
            if not isinstance(invocation, dict):
                errors.append(f"{invocation_location}: expected an object")
                continue
            threads = _integer(invocation.get("threads"))
            warmup = invocation.get("warmup")
            if threads not in {1, 4} or not isinstance(warmup, bool):
                errors.append(
                    f"{invocation_location}: expected threads 1/4 and boolean warmup"
                )
                continue
            invocation_groups.setdefault((threads, warmup), []).append(invocation)
            repetition = _integer(invocation.get("repetition"))
            scenario = invocation.get("scenario")
            if repetition is not None and _nonempty_string(scenario):
                run_label = (
                    f"warmup{repetition:03d}" if warmup else f"rep{repetition:03d}"
                )
                expected_tool_rechecks.append(
                    f"after_{scenario}-n600-t{threads}-{run_label}"
                )
            _expect_fields(
                invocation,
                {
                    "complete_rows": True,
                    "expected_rows": 600,
                    "returncode": 0,
                    "rows": 600,
                    "scenario": "masonry_arch_25_literal_wedge",
                    "steps": 600,
                    "timed_out": False,
                },
                invocation_location,
                errors,
            )
            affinity = _object(
                invocation.get("cpu_affinity"),
                f"{invocation_location}.cpu_affinity",
                errors,
            )
            if affinity is None:
                continue
            logical_cpus = affinity.get("logical_cpus")
            physical_keys = affinity.get("physical_core_keys")
            if (
                not isinstance(logical_cpus, list)
                or any(_integer(cpu) is None for cpu in logical_cpus)
                or len(logical_cpus) != threads
                or len(set(logical_cpus)) != threads
                or not isinstance(physical_keys, list)
                or any(not _nonempty_string(key) for key in physical_keys)
                or len(physical_keys) != threads
                or len(set(physical_keys)) != threads
            ):
                errors.append(
                    f"{invocation_location}.cpu_affinity: expected {threads} distinct "
                    "logical CPUs and physical cores"
                )
                continue
            _expect_fields(
                affinity,
                {
                    "logical_cpu_count": threads,
                    "one_logical_per_physical_core": True,
                    "physical_core_count": threads,
                    "source": "explicit_taskset",
                },
                f"{invocation_location}.cpu_affinity",
                errors,
            )
            logical_to_physical = _object(
                affinity.get("logical_cpu_physical_core_keys"),
                (
                    f"{invocation_location}.cpu_affinity."
                    "logical_cpu_physical_core_keys"
                ),
                errors,
            )
            package_ids = affinity.get("package_ids")
            smt_counts = affinity.get("smt_sibling_counts")
            core_classes = affinity.get("core_classes_khz")
            governors = affinity.get("scaling_governors")
            if (
                logical_to_physical is None
                or set(logical_to_physical) != {str(cpu) for cpu in logical_cpus}
                or any(
                    not _nonempty_string(logical_to_physical.get(str(cpu)))
                    for cpu in logical_cpus
                )
                or [logical_to_physical[str(cpu)] for cpu in logical_cpus]
                != physical_keys
                or not isinstance(package_ids, list)
                or not package_ids
                or any(not _nonempty_string(value) for value in package_ids)
                or len(package_ids) != len(set(package_ids))
                or _integer(affinity.get("package_count")) != len(package_ids)
                or not isinstance(smt_counts, list)
                or not smt_counts
                or any(
                    _integer(value) is None or _integer(value) <= 0
                    for value in smt_counts
                )
                or len(smt_counts) != len(set(smt_counts))
                or not isinstance(core_classes, list)
                or not core_classes
                or any(
                    _integer(value) is None or _integer(value) <= 0
                    for value in core_classes
                )
                or len(core_classes) != len(set(core_classes))
                or not isinstance(governors, list)
                or not governors
                or any(not _nonempty_string(value) for value in governors)
                or len(governors) != len(set(governors))
            ):
                errors.append(
                    f"{invocation_location}.cpu_affinity: incomplete or inconsistent "
                    "topology provenance"
                )
                continue
            logical_topology = _object(
                affinity.get("logical_cpu_topology"),
                f"{invocation_location}.cpu_affinity.logical_cpu_topology",
                errors,
            )
            if logical_topology is None or set(logical_topology) != {
                str(cpu) for cpu in logical_cpus
            }:
                errors.append(
                    f"{invocation_location}.cpu_affinity.logical_cpu_topology: "
                    "expected exact logical CPU membership"
                )
                continue
            derived_packages: list[str] = []
            derived_smt_counts: list[int] = []
            derived_core_classes: list[int] = []
            derived_governors: list[str] = []
            topology_valid = True
            for cpu in logical_cpus:
                cpu_location = (
                    f"{invocation_location}.cpu_affinity."
                    f"logical_cpu_topology[{str(cpu)!r}]"
                )
                cpu_topology = _object(
                    logical_topology.get(str(cpu)), cpu_location, errors
                )
                if cpu_topology is None:
                    topology_valid = False
                    continue
                package_id = cpu_topology.get("physical_package_id")
                core_id = cpu_topology.get("core_id")
                sibling_count = _integer(cpu_topology.get("thread_sibling_count"))
                core_class = _integer(cpu_topology.get("cpuinfo_max_frequency_khz"))
                governor = cpu_topology.get("scaling_governor")
                sibling_list = cpu_topology.get("thread_siblings_list")
                if (
                    not _nonempty_string(package_id)
                    or not _nonempty_string(core_id)
                    or sibling_count is None
                    or sibling_count <= 0
                    or core_class is None
                    or core_class <= 0
                    or not _nonempty_string(governor)
                    or not _nonempty_string(sibling_list)
                ):
                    errors.append(
                        f"{cpu_location}: incomplete per-CPU topology provenance"
                    )
                    topology_valid = False
                    continue
                derived_physical_key = f"{package_id}:{core_id}"
                if logical_to_physical[str(cpu)] != derived_physical_key:
                    errors.append(
                        f"{cpu_location}: derived physical key "
                        f"{derived_physical_key!r} does not match "
                        f"{logical_to_physical[str(cpu)]!r}"
                    )
                    topology_valid = False
                derived_packages.append(package_id)
                derived_smt_counts.append(sibling_count)
                derived_core_classes.append(core_class)
                derived_governors.append(governor)
            if not topology_valid:
                continue
            if (
                package_ids != sorted(set(derived_packages))
                or smt_counts != sorted(set(derived_smt_counts))
                or core_classes != sorted(set(derived_core_classes))
                or governors != sorted(set(derived_governors))
            ):
                errors.append(
                    f"{invocation_location}.cpu_affinity: aggregate topology "
                    "does not match per-CPU records"
                )
                continue
            raw_affinity = {
                "affinity_source": "explicit_taskset",
                "affinity_logical_cpus": ",".join(str(cpu) for cpu in logical_cpus),
                "affinity_logical_cpu_count": str(threads),
                "affinity_logical_cpu_physical_core_keys": ";".join(
                    f"{cpu}={logical_to_physical[str(cpu)]}" for cpu in logical_cpus
                ),
                "affinity_physical_core_keys": ";".join(physical_keys),
                "affinity_physical_core_count": str(threads),
                "affinity_one_logical_per_physical_core": "true",
                "affinity_package_ids": ",".join(package_ids),
                "affinity_package_count": str(len(package_ids)),
                "affinity_smt_sibling_counts": ",".join(
                    str(value) for value in smt_counts
                ),
                "affinity_core_classes_khz": ",".join(
                    str(value) for value in core_classes
                ),
                "affinity_scaling_governors": ",".join(governors),
            }
            expected_cpu_list = raw_affinity["affinity_logical_cpus"]
            _expect_fields(
                invocation,
                {"cpu_list": expected_cpu_list},
                invocation_location,
                errors,
            )
            command = invocation.get("command")
            if (
                not isinstance(command, list)
                or len(command) != 16
                or any(not _nonempty_string(argument) for argument in command)
            ):
                errors.append(
                    f"{invocation_location}.command: expected 16 non-empty arguments"
                )
            else:
                expected_taskset = (
                    taskset_tool.get("resolved_path")
                    if taskset_tool is not None
                    else None
                )
                if command[0] != expected_taskset:
                    errors.append(
                        f"{invocation_location}.command[0]: expected recorded taskset "
                        f"path {expected_taskset!r}, got {command[0]!r}"
                    )
                expected_arguments = {
                    1: "--cpu-list",
                    2: expected_cpu_list,
                    3: binary_path,
                    4: "masonry_arch_25_literal_wedge",
                    5: "exact_fbf",
                    6: "1",
                    7: "600",
                    8: "nan",
                    9: "performance",
                    10: "default",
                    11: "default",
                    12: str(threads),
                    13: "dart_best_colored_bgs",
                    14: "native",
                    15: "default",
                }
                for command_index, expected in expected_arguments.items():
                    if command[command_index] != expected:
                        errors.append(
                            f"{invocation_location}.command[{command_index}]: "
                            f"expected {expected!r}, got {command[command_index]!r}"
                        )
            previous = expected_affinity.setdefault(threads, raw_affinity)
            if previous != raw_affinity:
                errors.append(
                    f"{invocation_location}.cpu_affinity: topology changed across runs"
                )
        for threads in (1, 4):
            warmup_runs = invocation_groups.get((threads, True), [])
            measured_runs = invocation_groups.get((threads, False), [])
            if len(warmup_runs) != 1 or len(measured_runs) != 3:
                errors.append(
                    f"{location}.invocations[{threads}]: expected one warmup and "
                    f"three measured runs, got {len(warmup_runs)} and "
                    f"{len(measured_runs)}"
                )
            measured_repetitions = {
                _integer(run.get("repetition")) for run in measured_runs
            }
            if measured_repetitions != {1, 2, 3}:
                errors.append(
                    f"{location}.invocations[{threads}].repetition: expected 1, 2, 3"
                )
            if (
                len(warmup_runs) == 1
                and _integer(warmup_runs[0].get("repetition")) != 1
            ):
                errors.append(
                    f"{location}.invocations[{threads}].warmup.repetition: expected 1"
                )
        if (
            metadata is not None
            and metadata.get("executed_tool_identity_rechecks")
            != expected_tool_rechecks
        ):
            errors.append(
                f"{location}.metadata.executed_tool_identity_rechecks: expected "
                "one ordered taskset identity recheck after every invocation"
            )

    raw_path = bundle / "raw.csv"
    parsed = _read_csv(raw_path, f"{location}.raw", errors)
    if parsed is None:
        return
    fieldnames, rows = parsed
    required_fields = {
        "actual_threads",
        "affinity_core_classes_khz",
        "affinity_logical_cpus",
        "affinity_logical_cpu_count",
        "affinity_logical_cpu_physical_core_keys",
        "affinity_one_logical_per_physical_core",
        "affinity_package_count",
        "affinity_package_ids",
        "affinity_physical_core_keys",
        "affinity_physical_core_count",
        "affinity_scaling_governors",
        "affinity_smt_sibling_counts",
        "affinity_source",
        "contacts",
        "diagonal_seed_enabled",
        "exact_colored_bgs_logical_cpus",
        "fixed_inner_sweeps_requested",
        "inner_bgs_schedule_contract",
        "inner_sweeps_requested",
        "last_exact_colored_bgs_colors",
        "last_exact_colored_bgs_dispatches",
        "last_exact_colored_bgs_manifolds",
        "last_exact_colored_bgs_max_manifolds_per_color",
        "last_exact_colored_bgs_used",
        "matrix_free_seed_enabled",
        "max_arch_body_displacement_from_initial",
        "max_outer_iterations",
        "max_phase_exact_colored_bgs_logical_cpus",
        "min_arch_body_orientation_alignment_from_initial",
        "outer_relaxation",
        "process_returncode",
        "repetition",
        "requested_threads",
        "residual",
        "split_impulse_enabled",
        "status",
        "step",
        "step_exact_failures",
        "step_fallbacks",
        "step_size_persistence_enabled",
        "step_size_scale",
        "unique_colliding_body_pairs",
        "wall_ms",
        "warmup",
    }
    missing = sorted(required_fields - set(fieldnames))
    if missing:
        errors.append(f"{location}.raw: missing claim fields {missing}")
        return
    if len(rows) != 4800:
        errors.append(f"{location}.raw: expected 4800 total rows, got {len(rows)}")

    measured = [row for row in rows if row.get("warmup") == "0"]
    warmups = [row for row in rows if row.get("warmup") == "1"]
    if len(measured) != 3600 or len(warmups) != 1200:
        errors.append(
            f"{location}.raw: expected 3600 measured and 1200 warmup rows, got "
            f"{len(measured)} and {len(warmups)}"
        )

    schedule = _object(
        data.get("colored_schedule"), f"{location}.colored_schedule", errors
    )
    options = _object(data.get("solver_options"), f"{location}.solver_options", errors)
    if schedule is None or options is None:
        return

    def expect_uniform(
        selected: list[dict[str, str]], field: str, expected: str, row_location: str
    ) -> None:
        observed = {row.get(field) for row in selected}
        if observed != {expected}:
            errors.append(
                f"{row_location}.{field}: expected only {expected!r}, got "
                f"{sorted(repr(value) for value in observed)}"
            )

    for threads, timing_field in ((1, "one_thread"), (4, "four_threads")):
        selected = [
            row for row in measured if row.get("requested_threads") == str(threads)
        ]
        row_location = f"{location}.raw[{threads}]"
        if len(selected) != 1800:
            errors.append(
                f"{row_location}: expected 1800 measured rows, got {len(selected)}"
            )
            continue
        step_counts = {step: 0 for step in range(1, 601)}
        for index, row in enumerate(selected):
            step = _parse_csv_int(
                row.get("step"), f"{row_location}.rows[{index}].step", errors
            )
            if step in step_counts:
                step_counts[step] += 1
        if set(step_counts.values()) != {3}:
            errors.append(
                f"{row_location}.step: every step 1..600 must occur three times"
            )
        for repetition in (1, 2, 3):
            repetition_rows = [
                row for row in selected if row.get("repetition") == str(repetition)
            ]
            repetition_steps = {
                _parse_csv_int(
                    row.get("step"),
                    f"{row_location}.repetition[{repetition}].step",
                    errors,
                )
                for row in repetition_rows
            }
            if len(repetition_rows) != 600 or repetition_steps != set(range(1, 601)):
                errors.append(
                    f"{row_location}.repetition[{repetition}]: expected one complete "
                    "600-step trajectory"
                )

        warmup_rows = [
            row for row in warmups if row.get("requested_threads") == str(threads)
        ]
        warmup_steps = {
            _parse_csv_int(row.get("step"), f"{row_location}.warmup.step", errors)
            for row in warmup_rows
        }
        if (
            len(warmup_rows) != 600
            or {row.get("repetition") for row in warmup_rows} != {"1"}
            or warmup_steps != set(range(1, 601))
        ):
            errors.append(
                f"{row_location}.warmup: expected one complete 600-step trajectory"
            )

        affinity_fields = expected_affinity.get(threads)
        if affinity_fields is None:
            errors.append(
                f"{row_location}.affinity: no valid process topology record available"
            )
            affinity_fields = {}

        uniform_fields = {
            "actual_threads": str(threads),
            "process_returncode": "0",
            "contacts": str(data.get("contacts_each_step")),
            "unique_colliding_body_pairs": str(
                data.get("colliding_body_pairs_each_step")
            ),
            "last_exact_colored_bgs_dispatches": "0" if threads == 1 else "1",
            "last_exact_colored_bgs_manifolds": str(schedule.get("manifolds")),
            "last_exact_colored_bgs_colors": str(schedule.get("colors")),
            "last_exact_colored_bgs_max_manifolds_per_color": str(
                schedule.get("max_manifolds_per_color")
            ),
            "inner_bgs_schedule_contract": (
                "dart_deterministic_manifold_colored_bgs_nonpaper"
            ),
            "inner_sweeps_requested": str(options.get("inner_fixed_sweeps")),
            "fixed_inner_sweeps_requested": "1",
            "step_size_persistence_enabled": "0",
            "diagonal_seed_enabled": "0",
            "matrix_free_seed_enabled": "0",
            "max_outer_iterations": str(options.get("outer_iterations")),
            "split_impulse_enabled": "1",
            "status": "success",
            "step_exact_failures": "0",
            "step_fallbacks": "0",
        }
        uniform_fields.update(affinity_fields)
        for field, expected in uniform_fields.items():
            expect_uniform(selected, field, expected, row_location)
            expect_uniform(warmup_rows, field, expected, f"{row_location}.warmup")

        expected_phase_cpu_ids = (
            []
            if threads == 1
            else affinity_fields.get("affinity_logical_cpus", "").split(",")
        )
        expected_residency = (
            "none" if not expected_phase_cpu_ids else ";".join(expected_phase_cpu_ids)
        )
        for residency_field in (
            "exact_colored_bgs_logical_cpus",
            "max_phase_exact_colored_bgs_logical_cpus",
        ):
            expect_uniform(selected, residency_field, expected_residency, row_location)
            expect_uniform(
                warmup_rows,
                residency_field,
                expected_residency,
                f"{row_location}.warmup",
            )
        phase_cpu_counts: set[int] = set()
        for index, row in enumerate(selected):
            phase_cpus = row.get("max_phase_exact_colored_bgs_logical_cpus")
            if phase_cpus == "none":
                phase_cpu_counts.add(0)
                continue
            if not _nonempty_string(phase_cpus):
                errors.append(
                    f"{row_location}.rows[{index}]."
                    "max_phase_exact_colored_bgs_logical_cpus: expected 'none' "
                    "or semicolon-separated CPU ids"
                )
                phase_cpu_counts.add(-1)
                continue
            cpu_ids = phase_cpus.split(";")
            if any(not cpu_id for cpu_id in cpu_ids) or len(cpu_ids) != len(
                set(cpu_ids)
            ):
                errors.append(
                    f"{row_location}.rows[{index}]."
                    "max_phase_exact_colored_bgs_logical_cpus: expected distinct "
                    "semicolon-separated CPU ids"
                )
                phase_cpu_counts.add(-1)
                continue
            if cpu_ids != expected_phase_cpu_ids:
                errors.append(
                    f"{row_location}.rows[{index}]."
                    "max_phase_exact_colored_bgs_logical_cpus: expected affinity "
                    f"CPUs {expected_phase_cpu_ids!r}, got {cpu_ids!r}"
                )
                phase_cpu_counts.add(-1)
                continue
            phase_cpu_counts.add(len(cpu_ids))
        expected_phase_cpu_count = 0 if threads == 1 else 4
        if phase_cpu_counts != {expected_phase_cpu_count}:
            errors.append(
                f"{row_location}.max_phase_exact_colored_bgs_logical_cpus: "
                f"expected {expected_phase_cpu_count}, got {sorted(phase_cpu_counts)}"
            )
        colored_used = sum(
            row.get("last_exact_colored_bgs_used") == "1" for row in selected
        )
        if colored_used != 1785:
            errors.append(
                f"{row_location}.last_exact_colored_bgs_used: expected 1785, "
                f"got {colored_used}"
            )

        numeric_columns: dict[str, list[float]] = {}
        for field in (
            "wall_ms",
            "residual",
            "max_arch_body_displacement_from_initial",
            "min_arch_body_orientation_alignment_from_initial",
            "step_size_scale",
            "outer_relaxation",
        ):
            values: list[float] = []
            for index, row in enumerate(selected):
                value = _parse_csv_number(
                    row.get(field), f"{row_location}.rows[{index}].{field}", errors
                )
                if value is not None:
                    values.append(value)
            numeric_columns[field] = values
        for field, expected in (
            ("step_size_scale", _number(options.get("step_size_scale"))),
            ("outer_relaxation", _number(options.get("outer_relaxation"))),
        ):
            values = numeric_columns[field]
            if expected is not None and (
                len(values) != len(selected)
                or any(
                    not math.isclose(value, expected, rel_tol=1e-12, abs_tol=1e-12)
                    for value in values
                )
            ):
                errors.append(f"{row_location}.{field}: solver option drifted")

        wall = sorted(numeric_columns["wall_ms"])
        if len(wall) == len(selected):
            position = 0.95 * (len(wall) - 1)
            lower = math.floor(position)
            upper = math.ceil(position)
            p95 = wall[lower] + (wall[upper] - wall[lower]) * (position - lower)
            middle = len(wall) // 2
            computed_timing = {
                "mean_ms": sum(wall) / len(wall),
                "median_ms": (wall[middle - 1] + wall[middle]) / 2.0,
                "p95_ms": p95,
                "max_ms": wall[-1],
            }
            timing = _object(
                data.get(timing_field), f"{location}.{timing_field}", errors
            )
            if timing is not None:
                for field, expected in computed_timing.items():
                    actual = _number(timing.get(field))
                    if actual is None or not math.isclose(
                        actual, expected, rel_tol=1e-12, abs_tol=1e-12
                    ):
                        errors.append(
                            f"{location}.{timing_field}.{field}: expected "
                            f"{expected!r}, got {timing.get(field)!r}"
                        )

        for field, expected in (
            ("residual", _number(data.get("max_residual"))),
            (
                "max_arch_body_displacement_from_initial",
                _number(data.get("max_arch_body_displacement_from_initial_m")),
            ),
        ):
            values = numeric_columns[field]
            if expected is not None and (
                not values
                or not math.isclose(max(values), expected, rel_tol=1e-12, abs_tol=1e-15)
            ):
                errors.append(f"{row_location}.{field}: claim value drifted")
        orientations = numeric_columns[
            "min_arch_body_orientation_alignment_from_initial"
        ]
        expected_orientation = _number(
            data.get("min_arch_body_orientation_alignment_from_initial")
        )
        if expected_orientation is not None and (
            not orientations
            or not math.isclose(
                min(orientations), expected_orientation, rel_tol=1e-12, abs_tol=1e-15
            )
        ):
            errors.append(
                f"{row_location}.min_arch_body_orientation_alignment_from_initial: "
                "claim value drifted"
            )


def _validate_cpu_evidence_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.literal_wedge_exact_dynamics_nonpaper"
    data = _object(
        current_truth.get("literal_wedge_exact_dynamics_nonpaper"), location, errors
    )
    if data is None:
        return

    bundle = _validate_current_path(
        data,
        "bundle",
        CPU_EVIDENCE_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    _expect_fields(
        data,
        {
            "scene_contract": (
                "reconstructed_literal_wedge_arch_nonpaper_native_collision_frontend"
            ),
            "collision_frontend": "Native FourPointPlanar",
            "closure_meters": 1e-6,
            "evidence_binary_sha256": (
                "0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff"
            ),
            "steps_per_trajectory": 600,
            "warmup_trajectories_per_thread_count": 1,
            "measured_trajectories_per_thread_count": 3,
            "measured_steps_per_thread_count": 1800,
            "contacts_each_step": 96,
            "colliding_body_pairs_each_step": 24,
            "exact_failures": 0,
            "accepted_caps": 0,
            "boxed_lcp_fallbacks": 0,
            "physical_outcome_valid": True,
            "paper_comparable": False,
        },
        location,
        errors,
    )
    residual = _number(data.get("max_residual"))
    if residual is None or not 0.0 <= residual <= PAPER_RESIDUAL_TOLERANCE:
        errors.append(
            f"{location}.max_residual: expected a finite value <= "
            f"{PAPER_RESIDUAL_TOLERANCE}"
        )

    thread_rows: dict[int, dict[str, Any]] = {}
    for field, threads in (("one_thread", 1), ("four_threads", 4)):
        timing = _object(data.get(field), f"{location}.{field}", errors)
        if timing is None:
            continue
        thread_rows[threads] = timing
        _expect_fields(
            timing,
            {"mean_realtime": True, "all_steps_realtime": False},
            f"{location}.{field}",
            errors,
        )
        mean_ms = _number(timing.get("mean_ms"))
        maximum_ms = _number(timing.get("max_ms"))
        if mean_ms is None or not 0.0 < mean_ms < 1000.0 / 60.0:
            errors.append(
                f"{location}.{field}.mean_ms: expected positive mean below 60 Hz"
            )
        if maximum_ms is None or maximum_ms <= 1000.0 / 60.0:
            errors.append(
                f"{location}.{field}.max_ms: all-step 60 Hz must remain false"
            )

    declared_speedup = _number(data.get("validated_speedup"))
    if 1 in thread_rows and 4 in thread_rows:
        one_mean = _number(thread_rows[1].get("mean_ms"))
        four_mean = _number(thread_rows[4].get("mean_ms"))
        if one_mean is not None and four_mean is not None and four_mean > 0.0:
            computed_speedup = one_mean / four_mean
            if declared_speedup is None or not math.isclose(
                declared_speedup, computed_speedup, rel_tol=1e-12, abs_tol=1e-12
            ):
                errors.append(
                    f"{location}.validated_speedup: expected {computed_speedup}, "
                    f"got {data.get('validated_speedup')!r}"
                )

    claim_boundary = data.get("claim_boundary")
    if not _nonempty_string(claim_boundary) or not all(
        label in claim_boundary.casefold()
        for label in ("mean-throughput", "same-machine scaling", "not an all-step")
    ):
        errors.append(
            f"{location}.claim_boundary: expected narrow mean-throughput, "
            "same-machine scaling, and not-all-step labeling"
        )
    if (
        _nonempty_string(claim_boundary)
        and "paper timing" not in claim_boundary.casefold()
    ):
        errors.append(f"{location}.claim_boundary: paper timing exclusion is required")

    hashes = _validate_artifact_hashes(
        data,
        CPU_EVIDENCE_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        recorded_hash_keys={"trace_binary"},
    )
    configuration: dict[str, Any] | None = None
    taskset_tool: dict[str, Any] | None = None
    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    if metadata is not None:
        _expect_fields(metadata, {"schema_version": 8}, f"{location}.metadata", errors)
        binary = _object(metadata.get("binary"), f"{location}.metadata.binary", errors)
        if binary is not None:
            _validate_live_absolute_file_identity(
                binary, "path", f"{location}.metadata.binary", errors
            )
            _expect_fields(
                binary,
                {"sha256": data.get("evidence_binary_sha256")},
                f"{location}.metadata.binary",
                errors,
            )
        configuration = _object(
            metadata.get("configuration"),
            f"{location}.metadata.configuration",
            errors,
        )
        if configuration is not None:
            _expect_fields(
                configuration,
                {
                    "collision_frontend": "native",
                    "contract": "dart_best_colored_bgs",
                    "cpu_list": None,
                    "cpu_lists_by_threads": {"1": "8", "4": "8,10,12,14"},
                    "initial_gamma": "nan",
                    "local_solver": "default",
                    "repetitions": 3,
                    "solver": "exact_fbf",
                    "split_impulse": "default",
                    "threads": [1, 4],
                    "timeout_seconds": 600.0,
                    "warm_start": "default",
                    "warmup_repetitions": 1,
                },
                f"{location}.metadata.configuration",
                errors,
            )
        taskset_tool = _validate_cpu_executed_tool_closure(
            metadata,
            configuration,
            f"{location}.metadata",
            errors,
        )
        arch_contract = _object(
            metadata.get("literal_wedge_arch_contract"),
            f"{location}.metadata.literal_wedge_arch_contract",
            errors,
        )
        if arch_contract is not None:
            solver_options = _object(
                data.get("solver_options"), f"{location}.solver_options", errors
            )
            _expect_fields(
                arch_contract,
                {
                    "paper_parity": False,
                    "scenario": "masonry_arch_25_literal_wedge",
                    "scene_contract": data.get("scene_contract"),
                    "trajectory_steps": 600,
                    "expected_contacts_per_step": 96,
                    "expected_colliding_body_pairs_per_step": 24,
                    "max_body_displacement_from_constructed_initial_pose": 0.001,
                    "min_body_orientation_alignment_from_constructed_initial_pose": (
                        0.999
                    ),
                    "split_impulse": True,
                },
                f"{location}.metadata.literal_wedge_arch_contract",
                errors,
            )
            if solver_options is not None:
                _expect_fields(
                    arch_contract,
                    {
                        "colored_exact_options": {
                            "diagonal_seed": solver_options.get("diagonal_seed"),
                            "fixed_inner_sweeps": True,
                            "inner_sweeps": solver_options.get("inner_fixed_sweeps"),
                            "matrix_free_seed": solver_options.get("matrix_free_seed"),
                            "max_outer_iterations": solver_options.get(
                                "outer_iterations"
                            ),
                            "outer_relaxation": solver_options.get("outer_relaxation"),
                            "step_size_persistence": solver_options.get(
                                "step_size_persistence"
                            ),
                            "step_size_scale": solver_options.get("step_size_scale"),
                        }
                    },
                    f"{location}.metadata.literal_wedge_arch_contract",
                    errors,
                )
        source_identity = _object(
            metadata.get("source_identity"),
            f"{location}.metadata.source_identity",
            errors,
        )
        if source_identity is not None:
            _expect_fields(
                source_identity,
                {
                    "runner_path": "scripts/run_fbf_cpu_evidence.py",
                    "runner_sha256": hashes.get("runner"),
                    "trace_source_path": TRACE_SOURCE,
                    "trace_source_sha256": hashes.get("trace_source"),
                },
                f"{location}.metadata.source_identity",
                errors,
            )
        runtime_identity = _validate_cpu_runtime_identity(
            metadata.get("runtime_identity"),
            f"{location}.metadata.runtime_identity",
            repo_root,
            errors,
        )
        recheck = _object(
            metadata.get("identity_recheck"),
            f"{location}.metadata.identity_recheck",
            errors,
        )
        if recheck is not None:
            _expect_fields(
                recheck,
                {
                    "stage": "after_all_invocations",
                    "source_identity": metadata.get("source_identity"),
                    "runtime_identity": metadata.get("runtime_identity"),
                    "binary_sha256": data.get("evidence_binary_sha256"),
                },
                f"{location}.metadata.identity_recheck",
                errors,
            )
        _validate_cpu_runtime_provenance(
            data.get("runtime_provenance"),
            runtime_identity,
            taskset_tool,
            metadata.get("executed_tool_identity_rechecks"),
            f"{location}.runtime_provenance",
            repo_root,
            errors,
        )

    if bundle is None:
        return
    _validate_cpu_artifact_index(bundle, location, errors)
    _validate_cpu_raw_claims(bundle, data, metadata, taskset_tool, location, errors)
    summary_path = bundle / "summary.json"
    summary = _read_json_array(summary_path, f"{location}.summary", errors)
    if summary is None:
        return
    if len(summary) != 2 or any(not isinstance(row, dict) for row in summary):
        errors.append(f"{location}.summary: expected exactly two thread rows")
        return
    by_threads = {
        _integer(row.get("requested_threads")): row
        for row in summary
        if isinstance(row, dict)
    }
    if set(by_threads) != {1, 4}:
        errors.append(f"{location}.summary: expected requested_threads 1 and 4")
        return
    schedule = data.get("colored_schedule")
    schedule = schedule if isinstance(schedule, dict) else {}
    options = data.get("solver_options")
    options = options if isinstance(options, dict) else {}
    for threads, field in ((1, "one_thread"), (4, "four_threads")):
        row = by_threads[threads]
        _expect_fields(
            row,
            {
                "all_solver_steps_accepted": True,
                "all_solver_steps_successful": True,
                "all_states_finite": True,
                "all_steps_realtime_target_met": False,
                "actual_threads": str(threads),
                "affinity_logical_cpu_count": threads,
                "affinity_one_logical_per_physical_core": True,
                "affinity_physical_core_count": threads,
                "colored_bgs_colors": str(schedule.get("colors")),
                "colored_bgs_manifolds": str(schedule.get("manifolds")),
                "colored_bgs_max_manifolds_per_color": schedule.get(
                    "max_manifolds_per_color"
                ),
                "colored_bgs_used_steps": 1785,
                "complete_measured_trajectories": 3,
                "complete_requested_trajectory_evidence": True,
                "completed_warmup_trajectories": 1,
                "controlled_affinity_valid": True,
                "diagonal_seed_enabled": "0",
                "exact_failures": 0,
                "fallbacks": 0,
                "fixed_inner_sweeps_requested": "1",
                "full_trajectory_evidence": True,
                "inner_bgs_schedule_contract": (
                    "dart_deterministic_manifold_colored_bgs_nonpaper"
                ),
                "inner_sweeps_requested": str(options.get("inner_fixed_sweeps")),
                "matrix_free_seed_enabled": "0",
                "max_contacts": data.get("contacts_each_step"),
                "max_outer_iterations": str(options.get("outer_iterations")),
                "max_unique_colliding_body_pairs": data.get(
                    "colliding_body_pairs_each_step"
                ),
                "mean_realtime_target_met": True,
                "measured_trajectories": 3,
                "min_contacts": data.get("contacts_each_step"),
                "min_unique_colliding_body_pairs": data.get(
                    "colliding_body_pairs_each_step"
                ),
                "outer_relaxation": "1.1000000000000001",
                "paper_target_evaluated": False,
                "paper_timing_comparable": False,
                "paper_workload_contract_valid": False,
                "physical_outcome_reasons": "valid",
                "physical_outcome_valid": True,
                "realtime_contract_valid": True,
                "sample_steps": 1800,
                "scenario": "masonry_arch_25_literal_wedge",
                "split_impulse_enabled": "1",
                "step_size_persistence_enabled": "0",
                "step_size_scale": "35",
            },
            f"{location}.summary[{threads}]",
            errors,
        )
        timing = thread_rows.get(threads)
        if timing is not None:
            _expect_fields(
                row,
                {
                    "mean_step_ms": timing.get("mean_ms"),
                    "median_step_ms": timing.get("median_ms"),
                    "p95_step_ms": timing.get("p95_ms"),
                    "max_step_ms": timing.get("max_ms"),
                },
                f"{location}.summary[{threads}]",
                errors,
            )
        _expect_fields(
            row,
            {
                "measured_workload_fingerprint_sha256": data.get(
                    "measured_workload_fingerprint_sha256"
                )
            },
            f"{location}.summary[{threads}]",
            errors,
        )
        details = row.get("physical_outcome_details")
        required_detail_labels = (
            "rep1:passed=true",
            "rep2:passed=true",
            "rep3:passed=true",
            "whole_arch_stable_from_initial=true",
            "contact_manifold_valid=true",
            "colliding_body_pairs_valid=true",
        )
        if not _nonempty_string(details) or not all(
            label in details for label in required_detail_labels
        ):
            errors.append(
                f"{location}.summary[{threads}].physical_outcome_details: "
                "expected three passing repetitions and explicit arch/contact gates"
            )
    _expect_fields(
        by_threads[1],
        {
            "single_core_claim_valid": True,
            "multicore_claim_valid": False,
            "runtime_cpu_residency_valid": False,
            "observed_exact_colored_bgs_logical_cpu_count": 0,
            "phase_residency_min_logical_cpu_count": 0,
            "phase_residency_min_physical_core_count": 0,
            "colored_bgs_dispatches": 0,
        },
        f"{location}.summary[1]",
        errors,
    )
    _expect_fields(
        by_threads[4],
        {
            "single_core_claim_valid": False,
            "multicore_claim_valid": True,
            "runtime_cpu_residency_valid": True,
            "observed_exact_colored_bgs_logical_cpu_count": 4,
            "phase_residency_min_logical_cpu_count": 4,
            "phase_residency_min_physical_core_count": 4,
            "colored_bgs_dispatch_valid": True,
            "colored_bgs_dispatches": 1800,
            "scaling_pair_valid": True,
            "validated_speedup_vs_one_thread": declared_speedup,
        },
        f"{location}.summary[4]",
        errors,
    )
    if "summary.json" in hashes and _sha256(summary_path) != hashes["summary.json"]:
        errors.append(f"{location}.summary: hash binding changed during validation")


def _validate_current_small_cpu_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.current_small_paper_cpu_v1"
    data = _object(current_truth.get("current_small_paper_cpu_v1"), location, errors)
    if data is None:
        return

    bundle = _validate_current_path(
        data,
        "bundle",
        CURRENT_SMALL_CPU_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    _expect_fields(
        data,
        {
            "status": "valid_current_source_partial_strict_matrix",
            "schema_version": 8,
            "contract": "paper_cpu",
            "collision_frontend": "native",
            "cpu_list": "4",
            "repetitions_per_scenario": 3,
            "scenario_count": 9,
            "physical_classifier_passes": 9,
            "strict_solver_valid_scenarios": 7,
            "realtime_contract_valid_scenarios": 7,
            "artifact_count": 60,
            "overall_strict_matrix_valid": False,
            "paper_comparable": False,
        },
        location,
        errors,
    )
    binary_sha256 = data.get("evidence_binary_sha256")
    if not isinstance(binary_sha256, str) or not SHA256_PATTERN.fullmatch(
        binary_sha256
    ):
        errors.append(f"{location}.evidence_binary_sha256: expected lowercase SHA-256")

    claim_boundary = data.get("claim_boundary")
    if not _nonempty_string(claim_boundary) or not all(
        label in claim_boundary.casefold()
        for label in (
            "7/9",
            "9/9 physical",
            "not paper-comparable",
            "zero warmups",
        )
    ):
        errors.append(
            f"{location}.claim_boundary: expected 7/9 strict, 9/9 physical, "
            "not-paper-comparable, and zero-warmup boundaries"
        )

    hashes = _validate_artifact_hashes(
        data, CURRENT_SMALL_CPU_ARTIFACT_TARGETS, location, repo_root, errors
    )
    if bundle is None:
        return
    _validate_cpu_artifact_index(bundle, location, errors)

    configuration: dict[str, Any] | None = None
    taskset_tool: dict[str, Any] | None = None
    binary_path: str | None = None
    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    if metadata is not None:
        _expect_fields(metadata, {"schema_version": 8}, f"{location}.metadata", errors)
        configuration = _object(
            metadata.get("configuration"), f"{location}.metadata.configuration", errors
        )
        if configuration is not None:
            _expect_fields(
                configuration,
                {
                    "cases": [
                        {"scenario": "backspin", "steps": 240},
                        {"scenario": "incline_mu_0_4", "steps": 120},
                        {"scenario": "incline_mu_0_5", "steps": 120},
                        {"scenario": "painleve_mu_0_5", "steps": 150},
                        {"scenario": "painleve_mu_0_55", "steps": 150},
                        {"scenario": "turntable_mu_0_2_omega_2", "steps": 240},
                        {"scenario": "turntable_mu_0_2_omega_5", "steps": 240},
                        {"scenario": "turntable_mu_0_5_omega_2", "steps": 240},
                        {"scenario": "turntable_mu_0_5_omega_5", "steps": 240},
                    ],
                    "collision_frontend": "native",
                    "contract": "paper_cpu",
                    "cpu_list": "4",
                    "cpu_lists_by_threads": {},
                    "initial_gamma": "nan",
                    "local_solver": "default",
                    "repetitions": 3,
                    "resolved_local_solver": "exact_metric",
                    "solver": "exact_fbf",
                    "split_impulse": "default",
                    "threads": [1],
                    "timeout_seconds": 3600.0,
                    "warm_start": "default",
                    "warmup_repetitions": 0,
                },
                f"{location}.metadata.configuration",
                errors,
            )
        taskset_tool = _validate_cpu_executed_tool_closure(
            metadata,
            configuration,
            f"{location}.metadata",
            errors,
        )
        binary = _object(metadata.get("binary"), f"{location}.metadata.binary", errors)
        if binary is not None:
            if _nonempty_string(binary.get("path")):
                binary_path = binary["path"]
            _validate_live_absolute_file_identity(
                binary, "path", f"{location}.metadata.binary", errors
            )
            _expect_fields(
                binary,
                {"sha256": binary_sha256},
                f"{location}.metadata.binary",
                errors,
            )
        source_identity = _object(
            metadata.get("source_identity"),
            f"{location}.metadata.source_identity",
            errors,
        )
        if source_identity is not None:
            _expect_fields(
                source_identity,
                {
                    "runner_path": "scripts/run_fbf_cpu_evidence.py",
                    "runner_sha256": hashes.get("runner"),
                    "trace_source_path": TRACE_SOURCE,
                    "trace_source_sha256": hashes.get("trace_source"),
                },
                f"{location}.metadata.source_identity",
                errors,
            )
        runtime_identity = _validate_cpu_runtime_identity(
            metadata.get("runtime_identity"),
            f"{location}.metadata.runtime_identity",
            repo_root,
            errors,
        )
        recheck = _object(
            metadata.get("identity_recheck"),
            f"{location}.metadata.identity_recheck",
            errors,
        )
        if recheck is not None:
            _expect_fields(
                recheck,
                {
                    "stage": "after_all_invocations",
                    "source_identity": metadata.get("source_identity"),
                    "runtime_identity": metadata.get("runtime_identity"),
                    "binary_sha256": binary_sha256,
                },
                f"{location}.metadata.identity_recheck",
                errors,
            )
        _validate_cpu_runtime_provenance(
            data.get("runtime_provenance"),
            runtime_identity,
            taskset_tool,
            metadata.get("executed_tool_identity_rechecks"),
            f"{location}.runtime_provenance",
            repo_root,
            errors,
        )

    expected_steps = {
        "backspin": 240,
        "incline_mu_0_4": 120,
        "incline_mu_0_5": 120,
        "painleve_mu_0_5": 150,
        "painleve_mu_0_55": 150,
        "turntable_mu_0_2_omega_2": 240,
        "turntable_mu_0_2_omega_5": 240,
        "turntable_mu_0_5_omega_2": 240,
        "turntable_mu_0_5_omega_5": 240,
    }
    failed_scenario = "turntable_mu_0_5_omega_5"
    invocations = _read_json_array(
        bundle / "invocations.json", f"{location}.invocations", errors
    )
    if invocations is not None:
        if len(invocations) != 27:
            errors.append(f"{location}.invocations: expected 27 records")
        seen: set[tuple[str, int]] = set()
        expected_tool_rechecks: list[str] = []
        for index, record in enumerate(invocations):
            record_location = f"{location}.invocations[{index}]"
            if not isinstance(record, dict):
                errors.append(f"{record_location}: expected an object")
                continue
            scenario = record.get("scenario")
            repetition = _integer(record.get("repetition"))
            if scenario not in expected_steps or repetition not in {1, 2, 3}:
                errors.append(f"{record_location}: unexpected scenario/repetition")
                continue
            seen.add((scenario, repetition))
            steps = expected_steps[scenario]
            expected_returncode = 1 if scenario == failed_scenario else 0
            stem = f"{scenario}-n{steps}-t1-rep{repetition:03d}"
            expected_tool_rechecks.append(f"after_{stem}")
            _expect_fields(
                record,
                {
                    "collision_frontend": "native",
                    "complete_rows": True,
                    "cpu_list": "4",
                    "expected_rows": steps,
                    "returncode": expected_returncode,
                    "rows": steps,
                    "scenario": scenario,
                    "stderr_file": f"raw/{stem}.stderr.txt",
                    "stdout_file": f"raw/{stem}.csv",
                    "steps": steps,
                    "threads": 1,
                    "timed_out": False,
                    "warmup": False,
                },
                record_location,
                errors,
            )
            affinity = _object(
                record.get("cpu_affinity"), f"{record_location}.cpu_affinity", errors
            )
            if affinity is not None:
                _expect_fields(
                    affinity,
                    {
                        "logical_cpus": [4],
                        "logical_cpu_count": 1,
                        "physical_core_count": 1,
                        "one_logical_per_physical_core": True,
                        "source": "explicit_taskset",
                    },
                    f"{record_location}.cpu_affinity",
                    errors,
                )
            command = record.get("command")
            expected_suffix = [
                scenario,
                "exact_fbf",
                "1",
                str(steps),
                "nan",
                "performance",
                "default",
                "default",
                "1",
                "paper_cpu",
                "native",
                "default",
            ]
            if (
                not isinstance(command, list)
                or len(command) != 16
                or any(not _nonempty_string(argument) for argument in command)
                or command[0]
                != (
                    taskset_tool.get("resolved_path")
                    if taskset_tool is not None
                    else None
                )
                or command[1:3] != ["--cpu-list", "4"]
                or command[3] != binary_path
                or command[4:] != expected_suffix
            ):
                errors.append(f"{record_location}.command: unexpected invocation")
            stderr_path = bundle / str(record.get("stderr_file"))
            if not stderr_path.is_file() or stderr_path.stat().st_size != 0:
                errors.append(f"{record_location}.stderr_file: expected empty file")
        expected_seen = {
            (scenario, repetition)
            for scenario in expected_steps
            for repetition in (1, 2, 3)
        }
        if seen != expected_seen:
            errors.append(f"{location}.invocations: scenario/repetition set drifted")
        if (
            metadata is not None
            and metadata.get("executed_tool_identity_rechecks")
            != expected_tool_rechecks
        ):
            errors.append(
                f"{location}.metadata.executed_tool_identity_rechecks: expected "
                "one ordered taskset identity recheck after every invocation"
            )

    declared_rows = _object(data.get("rows"), f"{location}.rows", errors)
    summary = _read_json_array(bundle / "summary.json", f"{location}.summary", errors)
    summary_by_scenario: dict[str, dict[str, Any]] = {}
    if summary is not None:
        if len(summary) != 9 or any(not isinstance(row, dict) for row in summary):
            errors.append(f"{location}.summary: expected nine object rows")
        else:
            summary_by_scenario = {str(row.get("scenario")): row for row in summary}
            if set(summary_by_scenario) != set(expected_steps):
                errors.append(f"{location}.summary: scenario set drifted")
    if declared_rows is not None and set(declared_rows) != set(expected_steps):
        errors.append(f"{location}.rows: scenario set drifted")

    strict_count = 0
    physical_count = 0
    realtime_count = 0
    if declared_rows is not None:
        for scenario, steps in expected_steps.items():
            row_location = f"{location}.rows[{scenario!r}]"
            declared = _object(declared_rows.get(scenario), row_location, errors)
            summary_row = summary_by_scenario.get(scenario)
            if declared is None or summary_row is None:
                continue
            strict_valid = declared.get("strict_solver_valid") is True
            expected_successful = strict_valid
            expected_accepted = scenario != failed_scenario
            expected_failed_processes = 3 if scenario == failed_scenario else 0
            expected_accepted_cap_rows = (
                3 if scenario in {"incline_mu_0_5", "turntable_mu_0_5_omega_5"} else 0
            )
            _expect_fields(
                declared,
                {
                    "physical_outcome_valid": True,
                    "failed_processes": expected_failed_processes,
                    "max_iterations_accepted_rows": expected_accepted_cap_rows,
                },
                row_location,
                errors,
            )
            _expect_fields(
                summary_row,
                {
                    "scenario": scenario,
                    "steps_per_repetition": ";".join([str(steps)] * 3),
                    "physical_outcome_valid": True,
                    "all_solver_steps_successful": expected_successful,
                    "all_solver_steps_accepted": expected_accepted,
                    "failed_processes": expected_failed_processes,
                    "max_residual": declared.get("max_residual"),
                    "residual_pass_fraction": declared.get("residual_pass_fraction"),
                    "exact_failures": 0,
                    "fallbacks": 0,
                    "realtime_contract_valid": strict_valid,
                    "single_core_claim_valid": strict_valid,
                    "mean_step_ms": declared.get("mean_step_ms"),
                    "median_step_ms": declared.get("median_step_ms"),
                    "p95_step_ms": declared.get("p95_step_ms"),
                    "max_step_ms": declared.get("max_step_ms"),
                    "paper_target_evaluated": False,
                    "paper_timing_comparable": False,
                    "paper_workload_contract_valid": False,
                },
                f"{location}.summary[{scenario!r}]",
                errors,
            )
            details = summary_row.get("physical_outcome_details")
            if not _nonempty_string(details) or any(
                f"rep{repetition}:passed=true" not in details
                for repetition in (1, 2, 3)
            ):
                errors.append(
                    f"{location}.summary[{scenario!r}].physical_outcome_details: "
                    "expected three passing classifiers"
                )
            max_residual = _number(declared.get("max_residual"))
            if max_residual is None:
                errors.append(f"{row_location}.max_residual: expected finite number")
            elif strict_valid and max_residual > PAPER_RESIDUAL_TOLERANCE:
                errors.append(f"{row_location}: strict row exceeds residual tolerance")
            elif not strict_valid and max_residual <= PAPER_RESIDUAL_TOLERANCE:
                errors.append(
                    f"{row_location}: negative row no longer exceeds tolerance"
                )
            strict_count += int(strict_valid)
            physical_count += 1
            realtime_count += int(summary_row.get("realtime_contract_valid") is True)

    _expect_fields(
        data,
        {
            "strict_solver_valid_scenarios": strict_count,
            "physical_classifier_passes": physical_count,
            "realtime_contract_valid_scenarios": realtime_count,
        },
        location,
        errors,
    )

    parsed_raw = _read_csv(bundle / "raw.csv", f"{location}.raw", errors)
    if parsed_raw is not None and declared_rows is not None:
        _, raw_rows = parsed_raw
        for scenario, steps in expected_steps.items():
            row_location = f"{location}.raw[{scenario!r}]"
            selected = [row for row in raw_rows if row.get("scenario") == scenario]
            if len(selected) != steps * 3:
                errors.append(
                    f"{row_location}: expected {steps * 3} rows, got {len(selected)}"
                )
                continue
            expected_returncode = "1" if scenario == failed_scenario else "0"
            for index, row in enumerate(selected):
                expected_fields = {
                    "process_returncode": expected_returncode,
                    "warmup": "0",
                    "affinity_logical_cpus": "4",
                    "collision_frontend": "native",
                    "requested_threads": "1",
                    "actual_threads": "1",
                    "solver": "exact_fbf",
                    "solver_contract": "paper_cpu",
                    "inner_local_solver": "exact_metric",
                    "step_fallbacks": "0",
                    "step_exact_failures": "0",
                    "finite_state": "1",
                }
                for field, expected in expected_fields.items():
                    if row.get(field) != expected:
                        errors.append(
                            f"{row_location}.rows[{index}].{field}: expected "
                            f"{expected!r}, got {row.get(field)!r}"
                        )

            accepted_cap_rows = sum(
                row.get("status") == "max_iterations_accepted" for row in selected
            )
            declared = declared_rows.get(scenario)
            if isinstance(declared, dict):
                _expect_fields(
                    declared,
                    {"max_iterations_accepted_rows": accepted_cap_rows},
                    f"{location}.rows[{scenario!r}]",
                    errors,
                )

            def finite_values(field: str) -> list[float]:
                values: list[float] = []
                for index, row in enumerate(selected):
                    raw = row.get(field)
                    try:
                        value = float(raw) if raw is not None else math.nan
                    except ValueError:
                        errors.append(
                            f"{row_location}.rows[{index}].{field}: invalid number"
                        )
                        continue
                    if math.isfinite(value):
                        values.append(value)
                return values

            wall = sorted(finite_values("wall_ms"))
            residuals = finite_values("residual")
            declared = declared_rows.get(scenario)
            if not isinstance(declared, dict):
                continue
            if len(wall) == len(selected):
                position = 0.95 * (len(wall) - 1)
                lower = math.floor(position)
                upper = math.ceil(position)
                computed = {
                    "mean_step_ms": sum(wall) / len(wall),
                    "median_step_ms": statistics.median(wall),
                    "p95_step_ms": wall[lower]
                    + (wall[upper] - wall[lower]) * (position - lower),
                    "max_step_ms": wall[-1],
                }
                for field, expected in computed.items():
                    actual = _number(declared.get(field))
                    if actual is None or not math.isclose(
                        actual, expected, rel_tol=1e-12, abs_tol=1e-15
                    ):
                        errors.append(
                            f"{row_location}.{field}: expected {expected}, got "
                            f"{declared.get(field)!r}"
                        )
            if residuals:
                residual_max = max(residuals)
                residual_fraction = sum(
                    value <= PAPER_RESIDUAL_TOLERANCE for value in residuals
                ) / len(residuals)
                for field, expected in (
                    ("max_residual", residual_max),
                    ("residual_pass_fraction", residual_fraction),
                ):
                    actual = _number(declared.get(field))
                    if actual is None or not math.isclose(
                        actual, expected, rel_tol=1e-12, abs_tol=1e-15
                    ):
                        errors.append(
                            f"{row_location}.{field}: expected {expected}, got "
                            f"{declared.get(field)!r}"
                        )

    if (
        "summary.json" in hashes
        and _sha256(bundle / "summary.json") != hashes["summary.json"]
    ):
        errors.append(f"{location}.summary: hash binding changed during validation")


def _backspin_trace_metrics(
    parsed: tuple[list[str], list[dict[str, str]]] | None,
    location: str,
    errors: list[str],
) -> dict[str, Any] | None:
    if parsed is None:
        return None
    fieldnames, rows = parsed
    expected_fields = [
        "step",
        "time",
        "scenario",
        "solver",
        "body",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "up_z",
        "contacts",
        "exact_solves",
        "warm_starts",
        "fallbacks",
        "residual",
        "status",
    ]
    if fieldnames != expected_fields:
        errors.append(
            f"{location}: expected exact tracked schema {expected_fields}, got "
            f"{fieldnames}"
        )
        return None
    _validate_trace_csv(parsed, location, errors)
    if len(rows) != 131:
        errors.append(
            f"{location}: expected 131 rows including step 0, got {len(rows)}"
        )
        return None

    typed: list[dict[str, Any]] = []
    previous = {"exact_solves": 0, "warm_starts": 0, "fallbacks": 0}
    valid = True
    max_residual = 0.0
    for index, row in enumerate(rows):
        row_location = f"{location}.rows[{index}]"
        step = _parse_csv_int(row.get("step"), f"{row_location}.step", errors)
        time_seconds = _parse_csv_number(
            row.get("time"), f"{row_location}.time", errors
        )
        if step != index:
            errors.append(f"{row_location}.step: expected {index}, got {step}")
            valid = False
        if time_seconds is None or not math.isclose(
            time_seconds, index / 60.0, rel_tol=0.0, abs_tol=2e-14
        ):
            errors.append(f"{row_location}.time: expected step/60")
            valid = False
        if row.get("scenario") != "backspin":
            errors.append(
                f"{row_location}.scenario: expected 'backspin', got "
                f"{row.get('scenario')!r}"
            )
            valid = False
        if (
            row.get("solver") != "exact_fbf"
            or row.get("body") != "backspin_sphere_body"
        ):
            errors.append(f"{row_location}: solver/body identity changed")
            valid = False

        values: dict[str, Any] = {
            "step": step,
            "time": time_seconds,
            "status": row.get("status"),
        }
        for field in ("x", "y", "z", "vx", "vy", "vz", "up_z"):
            value = _parse_csv_number(row.get(field), f"{row_location}.{field}", errors)
            values[field] = value
            valid = valid and value is not None
        for field in ("contacts", "exact_solves", "warm_starts", "fallbacks"):
            value = _parse_csv_int(row.get(field), f"{row_location}.{field}", errors)
            values[field] = value
            if value is None:
                valid = False
                continue
            if value < 0:
                errors.append(f"{row_location}.{field}: expected non-negative")
                valid = False
            if field in previous:
                if value < previous[field]:
                    errors.append(f"{row_location}.{field}: cumulative value decreased")
                    valid = False
                previous[field] = value
        if index == 0:
            if (row.get("residual") or "").casefold() != "nan" or row.get(
                "status"
            ) != "not_run":
                errors.append(f"{row_location}: step 0 must be nan/not_run")
                valid = False
            if any(values.get(field) != 0 for field in previous):
                errors.append(f"{row_location}: cumulative counters must be zero")
                valid = False
            values["residual"] = None
        else:
            residual = _parse_csv_number(
                row.get("residual"), f"{row_location}.residual", errors
            )
            values["residual"] = residual
            if residual is None:
                valid = False
            else:
                max_residual = max(max_residual, residual)
            if row.get("status") != "success":
                errors.append(f"{row_location}.status: expected success")
                valid = False
        typed.append(values)

    if not valid:
        return None
    if any(row["fallbacks"] != 0 for row in typed):
        errors.append(f"{location}: boxed-LCP fallback observed")
    if typed[-1]["exact_solves"] <= 0:
        errors.append(f"{location}: no exact-FBF solve recorded")
    if not any(row["contacts"] > 0 for row in typed[1:]):
        errors.append(f"{location}: no contact was recorded")

    initial = typed[0]
    final = typed[-1]
    maximum = max(typed, key=lambda row: row["x"])
    reversal = next((row for row in typed[1:] if row["vx"] < 0.0), None)
    if initial["vx"] <= 0.0:
        errors.append(f"{location}: initial translational velocity is not positive")
    if maximum["x"] <= initial["x"] or maximum["step"] == 0:
        errors.append(f"{location}: positive translational advance was not recorded")
    if reversal is None:
        errors.append(f"{location}: translational velocity never reversed")
    elif reversal["step"] <= maximum["step"]:
        errors.append(f"{location}: reversal does not follow the forward maximum")
    if final["vx"] >= 0.0 or final["x"] >= initial["x"]:
        errors.append(f"{location}: final state did not travel backward past its start")
    if final["contacts"] <= 0:
        errors.append(f"{location}: final state is not in contact")
    if reversal is None:
        return None

    selected = []
    selected_fields = (
        "step",
        "time",
        "x",
        "z",
        "vx",
        "vz",
        "up_z",
        "contacts",
        "exact_solves",
        "warm_starts",
        "fallbacks",
        "residual",
        "status",
    )
    for step in (0, 10, 50, 120, 130):
        row = typed[step]
        selected.append({key: row[key] for key in selected_fields})
    projection = [
        {
            "step": row["step"],
            "contacts": row["contacts"],
            "exact_solves": row["exact_solves"],
            "warm_starts": row["warm_starts"],
            "boxed_lcp_fallbacks": row["fallbacks"],
            "status": row["status"],
        }
        for row in typed
    ]
    contact_free_steps = [row["step"] for row in typed[1:] if row["contacts"] == 0]
    summary = {
        "scenario": "backspin",
        "row_count": len(typed),
        "completed_steps": 130,
        "exact_solves": final["exact_solves"],
        "warm_starts": final["warm_starts"],
        "boxed_lcp_fallbacks": final["fallbacks"],
        "max_residual": max_residual,
        "initial": {
            key: initial[key] for key in ("step", "time", "x", "z", "vx", "up_z")
        },
        "maximum_forward_travel": {
            key: maximum[key] for key in ("step", "time", "x", "vx", "contacts")
        },
        "first_negative_vx": {
            key: reversal[key] for key in ("step", "time", "x", "vx", "contacts")
        },
        "final": {
            key: final[key]
            for key in ("step", "time", "x", "z", "vx", "vz", "up_z", "contacts")
        },
        "selected_panel_states": selected,
        "contact_free_post_initial_steps": contact_free_steps,
        "continuous_contact_proven": False,
        "strict_rigid_body_rest_proven": False,
        "signed_angular_direction_proven": False,
        "angular_velocity_exported": False,
        "physical_outcome_valid": True,
        "claim_scope": "DART translational advance and reversal through 2.17 seconds",
    }
    return {
        "summary": summary,
        "solver_projection": projection,
        "solver_projection_sha256": _payload_sha256(projection),
    }


def _backspin_expected_bundle_paths() -> set[str]:
    return {
        "metadata.json",
        "artifact-index.json",
        "manual-inspection.json",
        "run-summary.json",
        "capture-provenance.json",
        "trace-summary.json",
        "verification.json",
        "invocations.json",
        "REPORT.md",
        "traces/backspin.csv",
        "traces/backspin.stderr.txt",
        "backspin/metadata.json",
        "backspin/timeline.json",
        "backspin/panel.png",
        "backspin/panel.compose.json",
        "backspin/clip.mp4",
        "backspin/clip.gif",
        "backspin/stills/step_000000.png",
        "backspin/stills/step_000001.png",
        "backspin/stills/step_000002.png",
    }


def _validate_backspin_checker_asset_structure(
    mesh: str, texture: str
) -> dict[str, Any]:
    vertices: list[tuple[float, float, float]] = []
    texture_coordinates: list[tuple[float, float]] = []
    normals: list[tuple[float, float, float]] = []
    faces: list[tuple[tuple[int, int, int], ...]] = []

    for line_number, raw_line in enumerate(mesh.splitlines(), start=1):
        fields = raw_line.split()
        if not fields or fields[0].startswith("#"):
            continue
        tag = fields[0]
        try:
            if tag == "v":
                if len(fields) != 4:
                    raise ValueError
                vertices.append(tuple(float(value) for value in fields[1:]))
            elif tag == "vt":
                if len(fields) != 3:
                    raise ValueError
                texture_coordinates.append(tuple(float(value) for value in fields[1:]))
            elif tag == "vn":
                if len(fields) != 4:
                    raise ValueError
                normals.append(tuple(float(value) for value in fields[1:]))
            elif tag == "f":
                if len(fields) != 4:
                    raise ValueError
                corners: list[tuple[int, int, int]] = []
                for token in fields[1:]:
                    indices = token.split("/")
                    if len(indices) != 3 or any(not index for index in indices):
                        raise ValueError
                    corners.append(tuple(int(index) for index in indices))
                faces.append(tuple(corners))
        except ValueError as error:
            raise ValueError(
                f"checker OBJ record is invalid at line {line_number}"
            ) from error

    counts = (
        len(vertices),
        len(texture_coordinates),
        len(normals),
        len(faces),
    )
    if counts != (559, 559, 559, 960):
        raise ValueError("checker OBJ topology changed")

    for vertex, normal in zip(vertices, normals):
        if not all(math.isfinite(value) for value in (*vertex, *normal)):
            raise ValueError("checker OBJ contains non-finite geometry")
        vertex_length = math.sqrt(sum(value * value for value in vertex))
        normal_length = math.sqrt(sum(value * value for value in normal))
        alignment = sum(a * b for a, b in zip(vertex, normal))
        if abs(vertex_length - 1.0) > 1.0e-6:
            raise ValueError("checker OBJ radius changed")
        if abs(normal_length - 1.0) > 1.0e-6 or alignment < 1.0 - 1.0e-6:
            raise ValueError("checker OBJ outward normals changed")

    if any(
        not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in uv)
        for uv in texture_coordinates
    ):
        raise ValueError("checker OBJ UV bounds changed")
    u_values = [uv[0] for uv in texture_coordinates]
    v_values = [uv[1] for uv in texture_coordinates]
    if (
        min(u_values) != 0.0
        or max(u_values) != 1.0
        or min(v_values) != 0.0
        or max(v_values) != 1.0
    ):
        raise ValueError("checker OBJ UV extent changed")

    used_vertices: set[int] = set()
    used_texture_coordinates: set[int] = set()
    used_normals: set[int] = set()
    for face in faces:
        for vertex_index, uv_index, normal_index in face:
            if (
                not 1 <= vertex_index <= len(vertices)
                or not 1 <= uv_index <= len(texture_coordinates)
                or not 1 <= normal_index <= len(normals)
            ):
                raise ValueError("checker OBJ face index is out of range")
            if not vertex_index == uv_index == normal_index:
                raise ValueError("checker OBJ face index mapping changed")
            used_vertices.add(vertex_index)
            used_texture_coordinates.add(uv_index)
            used_normals.add(normal_index)

        points = [vertices[corner[0] - 1] for corner in face]
        edge_ab = tuple(points[1][axis] - points[0][axis] for axis in range(3))
        edge_ac = tuple(points[2][axis] - points[0][axis] for axis in range(3))
        cross = (
            edge_ab[1] * edge_ac[2] - edge_ab[2] * edge_ac[1],
            edge_ab[2] * edge_ac[0] - edge_ab[0] * edge_ac[2],
            edge_ab[0] * edge_ac[1] - edge_ab[1] * edge_ac[0],
        )
        centroid_sum = tuple(sum(point[axis] for point in points) for axis in range(3))
        outward_measure = sum(a * b for a, b in zip(cross, centroid_sum))
        if outward_measure <= 1.0e-10:
            raise ValueError("checker OBJ face winding is not outward")

        uvs = [texture_coordinates[corner[1] - 1] for corner in face]
        uv_area_twice = (uvs[1][0] - uvs[0][0]) * (uvs[2][1] - uvs[0][1]) - (
            uvs[1][1] - uvs[0][1]
        ) * (uvs[2][0] - uvs[0][0])
        if abs(uv_area_twice) <= 1.0e-12:
            raise ValueError("checker OBJ has a degenerate UV triangle")

    expected_indices = set(range(1, 560))
    if (
        used_vertices != expected_indices
        or used_texture_coordinates != expected_indices
        or used_normals != expected_indices
    ):
        raise ValueError("checker OBJ topology is not fully referenced")

    texture_tokens = [
        token
        for raw_line in texture.splitlines()
        if (line := raw_line.strip()) and not line.startswith("#")
        for token in line.split()
    ]
    if texture_tokens[:4] != ["P3", "64", "32", "255"]:
        raise ValueError("checker PPM header changed")
    try:
        texels = [int(value) for value in texture_tokens[4:]]
    except ValueError as error:
        raise ValueError("checker PPM contains non-integer texels") from error
    if len(texels) != 64 * 32 * 3:
        raise ValueError("checker PPM texel count changed")
    if any(not 0 <= value <= 255 for value in texels):
        raise ValueError("checker PPM texel range changed")

    ivory = (244, 241, 228)
    charcoal = (32, 36, 43)
    coral = (255, 93, 115)
    pixels = [tuple(texels[index : index + 3]) for index in range(0, len(texels), 3)]
    for y in range(32):
        for x in range(64):
            tile = (min(5, x * 6 // 64), min(3, y * 4 // 32))
            expected = (
                coral
                if tile == (0, 1)
                else ivory if (tile[0] + tile[1]) % 2 == 0 else charcoal
            )
            if pixels[y * 64 + x] != expected:
                raise ValueError("checker PPM tile layout changed")

    return {
        "mesh_vertex_count": len(vertices),
        "mesh_uv_count": len(texture_coordinates),
        "mesh_normal_count": len(normals),
        "mesh_triangle_count": len(faces),
        "mesh_outward_winding": True,
        "mesh_uv_triangles_non_degenerate": True,
        "texture_column_width_range": [10, 11],
        "texture_row_height_range": [8, 8],
        "checker_layout_validated": True,
        "registration_tile_coordinates": [0, 1],
        "registration_tile_count": 1,
    }


def _backspin_visual_only_source_contract(
    repo_root: Path, location: str, errors: list[str]
) -> dict[str, Any] | None:
    initial_error_count = len(errors)
    source_path = _artifact_path(
        "examples/demos/scenes/FbfPaperFrictionScene.cpp",
        f"{location}.path",
        repo_root,
        errors,
    )
    mesh_path = _artifact_path(
        "data/obj/fbf_backspin_checker_sphere.obj",
        f"{location}.mesh_path",
        repo_root,
        errors,
    )
    material_path = _artifact_path(
        "data/obj/fbf_backspin_checker_sphere.mtl",
        f"{location}.material_path",
        repo_root,
        errors,
    )
    texture_path = _artifact_path(
        "data/obj/fbf_backspin_checker.ppm",
        f"{location}.texture_path",
        repo_root,
        errors,
    )
    if None in (source_path, mesh_path, material_path, texture_path):
        return None
    try:
        source = source_path.read_text(encoding="utf-8")
        mesh = mesh_path.read_text(encoding="utf-8")
        material = material_path.read_text(encoding="utf-8")
        texture = texture_path.read_text(encoding="ascii")
        helper_start = source.index("void addBackspinCheckerTexture")
        helper_end = source.index("//================================", helper_start)
        helper = source[helper_start:helper_end]
        sphere_start = source.index("SkeletonPtr createBackspinSphere()")
        sphere_end = source.index("//================================", sphere_start)
        sphere = source[sphere_start:sphere_end]
        scene_start = source.index("DemoScene makeFbfPaperBackspinScene()")
        scene_end = source.index("//================================", scene_start)
        scene = source[scene_start:scene_end]
    except (OSError, UnicodeDecodeError, ValueError) as error:
        errors.append(f"{location}: checker-texture source is unreadable: {error}")
        return None

    required = (
        '"dart://sample/obj/fbf_backspin_checker_sphere.obj"',
        "DartResourceRetriever::create()",
        "MeshShape::loadMesh(meshUri, retriever)",
        "Eigen::Vector3d::Constant(kBackspinRadius)",
        "createShapeNodeWith<VisualAspect>(checker)",
    )
    if any(fragment not in helper for fragment in required):
        errors.append(f"{location}: checker-texture source contract changed")
    if "CollisionAspect" in helper or "DynamicsAspect" in helper:
        errors.append(f"{location}: checker mesh gained a physics aspect")
    physical_node = "createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape)"
    if physical_node not in sphere or "VisualAspect>(shape)" in sphere:
        errors.append(f"{location}: physical sphere aspect contract changed")
    inertia = "setShapeInertia(body, shape);"
    call = "addBackspinCheckerTexture(body);"
    if (
        inertia not in sphere
        or call not in sphere
        or sphere.index(inertia) > sphere.index(call)
    ):
        errors.append(f"{location}: inertia/checker-mesh ordering changed")
    elif inertia in sphere[sphere.index(call) :]:
        errors.append(f"{location}: inertia changes after the checker mesh")
    if source.count(call) != 1:
        errors.append(f"{location}: checker mesh attachment count changed")
    camera_home = (
        "::osg::Vec3d(-0.5, -1.25, 5.5)",
        "::osg::Vec3d(-0.5, 0.0, 0.2)",
        "::osg::Vec3d(0.0, 1.0, 0.0)",
    )
    if any(fragment not in scene for fragment in camera_home):
        errors.append(f"{location}: checker-legibility camera contract changed")
    if (
        mesh.count("mtllib fbf_backspin_checker_sphere.mtl") != 1
        or mesh.count("usemtl FbfBackspinChecker") != 1
        or "\nvt " not in mesh
    ):
        errors.append(f"{location}: checker OBJ material/UV contract changed")
    if (
        material.count("map_Kd fbf_backspin_checker.ppm") != 1
        or material.count("Ke 0.180000 0.180000 0.180000") != 1
    ):
        errors.append(f"{location}: checker MTL texture binding changed")
    try:
        asset_structure = _validate_backspin_checker_asset_structure(mesh, texture)
    except ValueError as error:
        errors.append(f"{location}: {error}")
        asset_structure = None
    if len(errors) != initial_error_count or asset_structure is None:
        return None
    return {
        "pass": True,
        "shape": "MeshShape",
        "aspect": "VisualAspect_only",
        "mesh_uri": "dart://sample/obj/fbf_backspin_checker_sphere.obj",
        "texture_binding": "fbf_backspin_checker_sphere.mtl:map_Kd",
        "texture_dimensions": [64, 32],
        "checker_grid": [6, 4],
        "palette": {
            "ivory": [244, 241, 228],
            "charcoal": [32, 36, 43],
            "coral": [255, 93, 115],
        },
        "registration_tile": "coral",
        "camera_home": {
            "eye": [-0.5, -1.25, 5.5],
            "center": [-0.5, 0.0, 0.2],
            "up": [0.0, 1.0, 0.0],
            "nearly_perpendicular_to_spin_axis": True,
        },
        **asset_structure,
        "physical_shape_aspects": ["CollisionAspect", "DynamicsAspect"],
        "physics_neutral_by_source_contract": True,
        "full_state_equivalence_proven": False,
        "source_sha256": _sha256(source_path),
        "mesh_sha256": _sha256(mesh_path),
        "material_sha256": _sha256(material_path),
        "texture_sha256": _sha256(texture_path),
    }


def _painleve_trace_metrics(
    parsed: tuple[list[str], list[dict[str, str]]] | None,
    scenario: str,
    location: str,
    errors: list[str],
) -> dict[str, Any] | None:
    if parsed is None:
        return None
    fieldnames, rows = parsed
    expected_fields = [
        "step",
        "time",
        "scenario",
        "solver",
        "body",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "up_z",
        "contacts",
        "exact_solves",
        "warm_starts",
        "fallbacks",
        "residual",
        "status",
    ]
    if fieldnames != expected_fields:
        errors.append(
            f"{location}: expected exact tracked schema {expected_fields}, got "
            f"{fieldnames}"
        )
        return None
    _validate_trace_csv(parsed, location, errors)
    if len(rows) != 151:
        errors.append(
            f"{location}: expected 151 rows including step 0, got {len(rows)}"
        )
        return None

    typed: list[dict[str, Any]] = []
    previous = {"exact_solves": 0, "warm_starts": 0, "fallbacks": 0}
    valid = True
    max_residual = 0.0
    for index, row in enumerate(rows):
        row_location = f"{location}.rows[{index}]"
        step = _parse_csv_int(row.get("step"), f"{row_location}.step", errors)
        time_seconds = _parse_csv_number(
            row.get("time"), f"{row_location}.time", errors
        )
        if step != index:
            errors.append(f"{row_location}.step: expected {index}, got {step}")
            valid = False
        if time_seconds is None or not math.isclose(
            time_seconds, index / 60.0, rel_tol=0.0, abs_tol=2e-14
        ):
            errors.append(f"{row_location}.time: expected step/60")
            valid = False
        if row.get("scenario") != scenario:
            errors.append(
                f"{row_location}.scenario: expected {scenario!r}, got "
                f"{row.get('scenario')!r}"
            )
            valid = False
        if row.get("solver") != "exact_fbf" or row.get("body") != "painleve_box_body":
            errors.append(f"{row_location}: solver/body identity changed")
            valid = False

        values: dict[str, Any] = {"step": step, "time": time_seconds}
        for field in ("x", "y", "z", "vx", "vy", "vz", "up_z"):
            value = _parse_csv_number(row.get(field), f"{row_location}.{field}", errors)
            values[field] = value
            valid = valid and value is not None
        for field in ("contacts", "exact_solves", "warm_starts", "fallbacks"):
            value = _parse_csv_int(row.get(field), f"{row_location}.{field}", errors)
            values[field] = value
            if value is None:
                valid = False
                continue
            if value < 0:
                errors.append(f"{row_location}.{field}: expected non-negative")
                valid = False
            if field in previous and value < previous[field]:
                errors.append(f"{row_location}.{field}: cumulative value decreased")
                valid = False
            if field in previous:
                previous[field] = value
        if index == 0:
            if (row.get("residual") or "").casefold() != "nan" or row.get(
                "status"
            ) != "not_run":
                errors.append(f"{row_location}: step 0 must be nan/not_run")
                valid = False
        else:
            residual = _parse_csv_number(
                row.get("residual"), f"{row_location}.residual", errors
            )
            if residual is None:
                valid = False
            else:
                max_residual = max(max_residual, residual)
            if row.get("status") != "success":
                errors.append(f"{row_location}.status: expected success")
                valid = False
        typed.append(values)
    if not valid:
        return None
    if any(row["fallbacks"] != 0 for row in typed):
        errors.append(f"{location}: boxed-LCP fallback observed")
    if typed[-1]["exact_solves"] <= 0:
        errors.append(f"{location}: no exact-FBF solve recorded")

    initial = typed[0]
    final = typed[-1]
    first_tumble = next(
        (row for row in typed[1:] if row["up_z"] < 0.55 or row["z"] < 0.35),
        None,
    )
    tail = typed[120:]
    max_tail_horizontal_speed = max(math.hypot(row["vx"], row["vy"]) for row in tail)
    tail_horizontal_drift = math.hypot(
        final["x"] - tail[0]["x"], final["y"] - tail[0]["y"]
    )
    tail_up_z_range = max(row["up_z"] for row in tail) - min(
        row["up_z"] for row in tail
    )
    tail_z_excursion = max(row["z"] for row in tail) - min(row["z"] for row in tail)
    tail_x_excursion = max(row["x"] for row in tail) - min(row["x"] for row in tail)
    tail_y_excursion = max(row["y"] for row in tail) - min(row["y"] for row in tail)
    if max_tail_horizontal_speed > 1e-4:
        errors.append(f"{location}: tail horizontal speed exceeds 1e-4")
    if max(tail_x_excursion, tail_y_excursion) > 1e-4:
        errors.append(f"{location}: tail horizontal excursion exceeds 1e-4")
    if tail_z_excursion > 1e-3:
        errors.append(f"{location}: tail vertical excursion exceeds 1e-3")
    if tail_up_z_range > 1e-4:
        errors.append(f"{location}: tail up_z range exceeds 1e-4")

    if scenario == "painleve_mu_0_5":
        if first_tumble is not None:
            errors.append(f"{location}: mu=.50 crossed the tumble threshold")
        if not (final["up_z"] > 0.85 and final["z"] > 0.35):
            errors.append(f"{location}: mu=.50 did not finish upright")
        if final["contacts"] <= 0 or final["x"] <= initial["x"]:
            errors.append(f"{location}: mu=.50 lacks positive in-contact travel")
    else:
        if first_tumble is None:
            errors.append(f"{location}: mu=.55 never crossed the tumble threshold")
        if not (
            abs(final["up_z"]) <= 0.1 and final["z"] < 0.35 and final["contacts"] > 0
        ):
            errors.append(f"{location}: mu=.55 did not finish horizontal/in contact")

    return {
        "exact_solves": final["exact_solves"],
        "final_x_m": final["x"],
        "final_z_m": final["z"],
        "final_up_z": final["up_z"],
        "first_tumble": first_tumble,
        "max_residual": max_residual,
        "max_tail_horizontal_speed_m_s": max_tail_horizontal_speed,
        "tail_horizontal_drift_m": tail_horizontal_drift,
        "tail_up_z_range": tail_up_z_range,
        "tail_z_excursion_m": tail_z_excursion,
        "initial_x_m": initial["x"],
    }


def _validate_backspin_v3_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.backspin_visual_v3_nonpaper"
    data = _object(current_truth.get("backspin_visual_v3_nonpaper"), location, errors)
    if data is None:
        return
    bundle = _validate_current_path(
        data,
        "bundle",
        BACKSPIN_V3_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    if bundle is not None:
        actual_paths: set[str] = set()
        actual_directories: set[str] = set()
        unsafe_paths: list[str] = []
        for path in bundle.rglob("*"):
            relative = path.relative_to(bundle).as_posix()
            if path.is_symlink():
                unsafe_paths.append(relative)
            elif path.is_file():
                actual_paths.add(relative)
            elif path.is_dir():
                actual_directories.add(relative)
        expected_paths = _backspin_expected_bundle_paths()
        expected_directories = {
            "backspin",
            "backspin/stills",
            "traces",
        }
        missing = sorted(expected_paths - actual_paths)
        unexpected = sorted(actual_paths - expected_paths)
        missing_directories = sorted(expected_directories - actual_directories)
        unexpected_directories = sorted(actual_directories - expected_directories)
        if unsafe_paths:
            errors.append(
                f"{location}.bundle: symlinks are forbidden: {sorted(unsafe_paths)}"
            )
        if missing or unexpected or missing_directories or unexpected_directories:
            errors.append(
                f"{location}.bundle: exact membership changed; missing={missing}, "
                f"unexpected={unexpected}, missing_directories={missing_directories}, "
                f"unexpected_directories={unexpected_directories}"
            )
        if (
            unsafe_paths
            or missing
            or unexpected
            or missing_directories
            or unexpected_directories
        ):
            # The deep checks below assume the exact finalized layout. Fail closed
            # without dereferencing absent or unsafe members.
            bundle = None
    _expect_fields(
        data,
        {
            "status": "valid_current_source_nonpaper_backspin",
            "artifact_valid": True,
            "solver_contract_valid": True,
            "physical_outcome_valid": True,
            "manual_inspected": True,
            "claim_valid": False,
            "paper_parity": False,
            "paper_comparable": False,
            "external_solver_parity": False,
            "approved_source_golden": False,
            "timing_verdict": None,
            "realtime_verdict": None,
            "actual_simulator": True,
            "generated_imagery": False,
            "automated_semantic_outcome_validated": False,
            "manual_visual_outcome_validated": True,
            "trace_equivalence_to_rendered_demo": False,
            "solver_projection_equivalent": True,
            "strict_rigid_body_rest_proven": False,
            "signed_angular_direction_proven": False,
            "continuous_contact_proven": False,
            "capture_sidecar_deliverable_validated": False,
            "steps": 130,
            "trace_rows": 131,
            "captured_frames": 131,
            "panel_dimensions": [6548, 538],
            "mp4_dimensions": [1300, 506],
            "mp4_frames": 66,
            "mp4_fps": 30,
            "gif_dimensions": [960, 374],
            "gif_frames": 33,
            "gif_fps": 15,
            "accepted_at_cap": 0,
            "exact_attempts": 129,
            "exact_solves": 129,
            "exact_failures": 0,
            "boxed_lcp_fallbacks": 0,
            "contact_free_post_initial_steps": [120],
            "artifact_count": 18,
        },
        location,
        errors,
    )
    for field in ("trace_max_residual", "capture_max_residual"):
        residual = _number(data.get(field))
        if residual is None or not 0.0 <= residual <= PAPER_RESIDUAL_TOLERANCE:
            errors.append(f"{location}.{field}: expected finite value <= 1e-6")
    for field in (
        "exact_attempts",
        "exact_solves",
        "warm_starts",
        "maximum_forward_step",
        "first_negative_vx_step",
    ):
        value = _integer(data.get(field))
        if value is None or value < 0:
            errors.append(f"{location}.{field}: expected a non-negative integer")
    for field in ("maximum_forward_x_m", "final_x_m", "final_vx_m_s"):
        if _number(data.get(field)) is None:
            errors.append(f"{location}.{field}: expected a finite number")
    projection_digest = data.get("solver_projection_sha256")
    if not isinstance(projection_digest, str) or not SHA256_PATTERN.fullmatch(
        projection_digest
    ):
        errors.append(
            f"{location}.solver_projection_sha256: expected lowercase SHA-256"
        )
    if not _nonempty_string(data.get("claim_boundary")):
        errors.append(f"{location}.claim_boundary: expected non-empty text")

    recorded_hash_keys = {
        "trace_binary",
        "demo_binary",
        "ffmpeg_binary",
        "ffprobe_binary",
        "python_binary",
    }
    hashes = _validate_artifact_hashes(
        data,
        BACKSPIN_V3_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        recorded_hash_keys=recorded_hash_keys,
    )
    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    trace_summary = _read_bundle_json(
        bundle, "trace-summary.json", f"{location}.trace_summary", errors
    )
    capture_metadata = _read_bundle_json(
        bundle, "backspin/metadata.json", f"{location}.capture_metadata", errors
    )
    run_summary = _read_bundle_json(
        bundle, "run-summary.json", f"{location}.run_summary", errors
    )
    capture_provenance_file = _read_bundle_json(
        bundle,
        "capture-provenance.json",
        f"{location}.capture_provenance_file",
        errors,
    )
    verification = _read_bundle_json(
        bundle, "verification.json", f"{location}.verification", errors
    )
    inspection = _read_bundle_json(
        bundle, "manual-inspection.json", f"{location}.manual_inspection", errors
    )
    invocations = _read_bundle_json(
        bundle, "invocations.json", f"{location}.invocations", errors
    )
    artifact_index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )

    if metadata is not None:
        expected_metadata_keys = {
            "schema_version",
            "status",
            "pass",
            "evidence_date",
            "requirement_ids",
            "artifact_valid",
            "solver_contract_valid",
            "physical_outcome_valid",
            "paper_parity",
            "paper_comparable",
            "external_solver_parity",
            "approved_source_golden",
            "timing_verdict",
            "realtime_verdict",
            "actual_simulator",
            "generated_imagery",
            "automated_semantic_outcome_validated",
            "manual_visual_outcome_validated",
            "trace_equivalence_to_rendered_demo",
            "solver_projection_equivalent",
            "strict_rigid_body_rest_proven",
            "signed_angular_direction_proven",
            "continuous_contact_proven",
            "capture_sidecar_deliverable_validated",
            "claim_scope",
            "trace_summary",
            "capture_summary",
            "capture_provenance",
            "visual_only_source_contract",
            "manual_inspection",
            "run_summary",
            "verification",
            "invocations",
            "report",
            "artifact_index",
            "source_identity",
            "claim_boundary",
        }
        if set(metadata) != expected_metadata_keys:
            errors.append(f"{location}.metadata: top-level contract changed")
        _expect_fields(
            metadata,
            {
                "schema_version": "dart.fbf_backspin_visual_bundle/v3",
                "status": data.get("status"),
                "pass": True,
                "requirement_ids": ["fig.03", "video.02_backspin"],
                "artifact_valid": data.get("artifact_valid"),
                "solver_contract_valid": data.get("solver_contract_valid"),
                "physical_outcome_valid": data.get("physical_outcome_valid"),
                "paper_parity": data.get("paper_parity"),
                "paper_comparable": data.get("paper_comparable"),
                "external_solver_parity": data.get("external_solver_parity"),
                "approved_source_golden": data.get("approved_source_golden"),
                "timing_verdict": data.get("timing_verdict"),
                "realtime_verdict": data.get("realtime_verdict"),
                "actual_simulator": data.get("actual_simulator"),
                "generated_imagery": data.get("generated_imagery"),
                "automated_semantic_outcome_validated": data.get(
                    "automated_semantic_outcome_validated"
                ),
                "manual_visual_outcome_validated": data.get(
                    "manual_visual_outcome_validated"
                ),
                "trace_equivalence_to_rendered_demo": data.get(
                    "trace_equivalence_to_rendered_demo"
                ),
                "solver_projection_equivalent": data.get(
                    "solver_projection_equivalent"
                ),
                "strict_rigid_body_rest_proven": data.get(
                    "strict_rigid_body_rest_proven"
                ),
                "signed_angular_direction_proven": data.get(
                    "signed_angular_direction_proven"
                ),
                "continuous_contact_proven": data.get("continuous_contact_proven"),
                "capture_sidecar_deliverable_validated": data.get(
                    "capture_sidecar_deliverable_validated"
                ),
                "claim_boundary": data.get("claim_boundary"),
            },
            f"{location}.metadata",
            errors,
        )
        if trace_summary is not None and metadata.get("trace_summary") != trace_summary:
            errors.append(f"{location}.metadata.trace_summary: file binding changed")
        bindings = {
            "manual_inspection": ("manual-inspection.json", "manual-inspection.json"),
            "run_summary": ("run-summary.json", "run-summary.json"),
            "verification": ("verification.json", "verification.json"),
            "invocations": ("invocations.json", "invocations.json"),
            "report": ("REPORT.md", "REPORT.md"),
        }
        for field, (relative, hash_key) in bindings.items():
            binding = _object(
                metadata.get(field), f"{location}.metadata.{field}", errors
            )
            if binding is not None:
                expected = {"path": relative, "sha256": hashes.get(hash_key)}
                if field == "manual_inspection":
                    expected["pass"] = True
                _expect_fields(
                    binding, expected, f"{location}.metadata.{field}", errors
                )
        index_binding = _object(
            metadata.get("artifact_index"),
            f"{location}.metadata.artifact_index",
            errors,
        )
        if index_binding is not None:
            _expect_fields(
                index_binding,
                {
                    "path": "artifact-index.json",
                    "sha256": hashes.get("artifact-index.json"),
                    "artifact_count": data.get("artifact_count"),
                    "excluded": ["artifact-index.json", "metadata.json"],
                },
                f"{location}.metadata.artifact_index",
                errors,
            )

        source_identity = _object(
            metadata.get("source_identity"),
            f"{location}.metadata.source_identity",
            errors,
        )
        expected_source_keys = set(BACKSPIN_V3_ARTIFACT_TARGETS) & {
            "finalizer",
            "finalizer_test",
            "visual_runner",
            "visual_runner_test",
            "image_compose",
            "image_tools",
            "image_verdict",
            "trace_source",
            "fixture_source",
            "demo_source",
            "backspin_checker_mesh",
            "backspin_checker_material",
            "backspin_checker_texture",
            "demo_host_source",
            "demo_host_header",
            "registry_source",
            "scenes_header",
        }
        expected_source_keys |= recorded_hash_keys
        if source_identity is not None:
            if set(source_identity) != expected_source_keys:
                errors.append(
                    f"{location}.metadata.source_identity: expected exact keys "
                    f"{sorted(expected_source_keys)}"
                )
            for source_key in sorted(expected_source_keys):
                identity = _object(
                    source_identity.get(source_key),
                    f"{location}.metadata.source_identity.{source_key}",
                    errors,
                )
                if identity is None:
                    continue
                _expect_fields(
                    identity,
                    {"sha256": hashes.get(source_key)},
                    f"{location}.metadata.source_identity.{source_key}",
                    errors,
                )
                if source_key in BACKSPIN_V3_ARTIFACT_TARGETS and source_key in (
                    expected_source_keys - recorded_hash_keys
                ):
                    expected_path = str(
                        (repo_root / BACKSPIN_V3_ARTIFACT_TARGETS[source_key]).resolve()
                    )
                    _expect_fields(
                        identity,
                        {"path": expected_path},
                        f"{location}.metadata.source_identity.{source_key}",
                        errors,
                    )
                elif not _nonempty_string(identity.get("path")):
                    errors.append(
                        f"{location}.metadata.source_identity.{source_key}.path: "
                        "expected a recorded binary path"
                    )

        capture_provenance = _object(
            metadata.get("capture_provenance"),
            f"{location}.metadata.capture_provenance",
            errors,
        )
        if (
            capture_provenance_file is not None
            and capture_provenance != capture_provenance_file
        ):
            errors.append(
                f"{location}.metadata.capture_provenance: file binding changed"
            )
        if capture_provenance is not None and bundle is not None:
            expected_provenance_keys = {
                "argv",
                "durable_stills",
                "pruned_staging",
                "returncode",
                "run_summary_path",
                "run_summary_sha256",
                "run_summary_validated",
                "stdout_path",
                "stdout_sha256",
                "stderr_path",
                "stderr_sha256",
                "runtime_resources_before",
                "runtime_resources_after",
            }
            if set(capture_provenance) != expected_provenance_keys:
                errors.append(
                    f"{location}.metadata.capture_provenance: top-level "
                    "contract changed"
                )

            def capture_identity_path(key: str) -> Any:
                identity = (
                    source_identity.get(key)
                    if isinstance(source_identity, dict)
                    else None
                )
                return identity.get("path") if isinstance(identity, dict) else None

            runtime_resources = {
                key: {
                    "path": str(
                        (repo_root / BACKSPIN_V3_ARTIFACT_TARGETS[key]).resolve()
                    ),
                    "sha256": hashes.get(key),
                }
                for key in (
                    "backspin_checker_mesh",
                    "backspin_checker_material",
                    "backspin_checker_texture",
                )
            }
            pruned = _validate_pruned_staging(
                bundle,
                capture_provenance.get("pruned_staging"),
                schema_version="dart.fbf_backspin_pruned_staging/v1",
                artifact_count=140,
                location=f"{location}.metadata.capture_provenance.pruned_staging",
                errors=errors,
            )
            _validate_durable_stills(
                bundle,
                capture_provenance.get("durable_stills"),
                (
                    (
                        0,
                        "backspin/stills/step_000000.png",
                        "backspin/frames/step_000000.png",
                    ),
                    (
                        1,
                        "backspin/stills/step_000001.png",
                        "backspin/frames/step_000001.png",
                    ),
                    (
                        2,
                        "backspin/stills/step_000002.png",
                        "backspin/frames/step_000002.png",
                    ),
                ),
                f"{location}.metadata.capture_provenance.durable_stills",
                errors,
            )
            _expect_fields(
                capture_provenance,
                {
                    "argv": [
                        capture_identity_path("python_binary"),
                        capture_identity_path("visual_runner"),
                        "run",
                        "--scenario",
                        "backspin",
                        "--demo",
                        capture_identity_path("demo_binary"),
                        "--output-root",
                        str(bundle),
                        "--ffmpeg",
                        capture_identity_path("ffmpeg_binary"),
                        "--ffprobe",
                        capture_identity_path("ffprobe_binary"),
                        "--python",
                        capture_identity_path("python_binary"),
                        "--out",
                        str(bundle / "run-summary.json"),
                    ],
                    "returncode": 0,
                    "run_summary_path": "run-summary.json",
                    "run_summary_sha256": hashes.get("run-summary.json"),
                    "run_summary_validated": True,
                    "stdout_path": "capture.stdout.txt",
                    "stdout_sha256": pruned.get("capture.stdout.txt", {}).get("sha256"),
                    "stderr_path": "capture.stderr.txt",
                    "stderr_sha256": pruned.get("capture.stderr.txt", {}).get("sha256"),
                    "runtime_resources_before": runtime_resources,
                    "runtime_resources_after": runtime_resources,
                },
                f"{location}.metadata.capture_provenance",
                errors,
            )

        source_contract = _backspin_visual_only_source_contract(
            repo_root, f"{location}.visual_only_source_contract", errors
        )
        if (
            source_contract is not None
            and metadata.get("visual_only_source_contract") != source_contract
        ):
            errors.append(
                f"{location}.metadata.visual_only_source_contract: source binding "
                "changed"
            )

    trace_metrics = None
    if bundle is not None:
        trace_path = bundle / "traces/backspin.csv"
        parsed_trace = _read_csv(trace_path, f"{location}.trace", errors)
        trace_metrics = _backspin_trace_metrics(
            parsed_trace, f"{location}.trace", errors
        )
    if trace_metrics is not None:
        trace = trace_metrics["summary"]
        _expect_fields(
            data,
            {
                "trace_rows": trace["row_count"],
                "exact_solves": trace["exact_solves"],
                "warm_starts": trace["warm_starts"],
                "trace_max_residual": trace["max_residual"],
                "maximum_forward_step": trace["maximum_forward_travel"]["step"],
                "maximum_forward_x_m": trace["maximum_forward_travel"]["x"],
                "first_negative_vx_step": trace["first_negative_vx"]["step"],
                "final_x_m": trace["final"]["x"],
                "final_vx_m_s": trace["final"]["vx"],
                "contact_free_post_initial_steps": trace[
                    "contact_free_post_initial_steps"
                ],
                "solver_projection_sha256": trace_metrics["solver_projection_sha256"],
            },
            location,
            errors,
        )
        if trace_summary is not None:
            _expect_fields(
                trace_summary,
                {
                    "schema_version": "dart.fbf_backspin_trace_summary/v1",
                    "pass": True,
                    "trace_path": "traces/backspin.csv",
                    "trace_sha256": hashes.get("traces/backspin.csv"),
                    "trace": trace,
                },
                f"{location}.trace_summary",
                errors,
            )

    capture_projection: list[dict[str, Any]] | None = None
    capture_summary: dict[str, Any] | None = None
    if capture_metadata is not None and bundle is not None:
        expected_capture_metadata_keys = {
            "schema_version",
            "kind",
            "pass",
            "schedule",
            "runtime",
            "timeline_validation",
            "panel_validation",
            "media_validation",
            "known_mismatches",
            "actual_simulator",
            "generated_imagery",
            "paper_comparable",
            "automated_semantic_outcome_validated",
            "semantic_outcome_gate",
        }
        if set(capture_metadata) != expected_capture_metadata_keys:
            errors.append(f"{location}.capture_metadata: top-level contract changed")
        _expect_fields(
            capture_metadata,
            {
                "schema_version": "dart.fbf_visual_evidence/v1",
                "kind": "capture_result",
                "pass": True,
                "actual_simulator": True,
                "generated_imagery": False,
                "paper_comparable": False,
                "automated_semantic_outcome_validated": False,
            },
            f"{location}.capture_metadata",
            errors,
        )
        if not _nonempty_string(capture_metadata.get("semantic_outcome_gate")):
            errors.append(
                f"{location}.capture_metadata.semantic_outcome_gate: expected "
                "non-empty text"
            )
        schedule = _object(
            capture_metadata.get("schedule"),
            f"{location}.capture_metadata.schedule",
            errors,
        )
        expected_output = {
            "directory": str(bundle / "backspin"),
            "timeline": str(bundle / "backspin/timeline.json"),
            "panel": str(bundle / "backspin/panel.png"),
            "mp4": str(bundle / "backspin/clip.mp4"),
            "gif": str(bundle / "backspin/clip.gif"),
        }
        if schedule is not None:
            _expect_fields(
                schedule,
                {
                    "id": "backspin",
                    "scene": "fbf_paper_backspin",
                    "source_segment": "backspin",
                    "total_steps": 130,
                    "time_step_seconds": 1.0 / 60.0,
                    "capture_steps": list(range(131)),
                    "panel_steps": [0, 10, 50, 120, 130],
                    "panel_labels": [
                        "t=0.00s",
                        "t=0.17s",
                        "t=0.83s",
                        "t=2.00s",
                        "t=2.17s",
                    ],
                    "actions": [],
                    "configuration": {
                        "radius_m": "0.25",
                        "initial_linear_velocity_m_s": "4",
                        "initial_angular_velocity_rad_s": "-200",
                        "mu": "0.5",
                        "orientation_cue": (
                            "renderer-applied high-contrast 6x4 checker texture "
                            "with coral registration tile"
                        ),
                    },
                    "output": expected_output,
                    "runnable": True,
                    "adapter_gap": None,
                    "long_run": False,
                    "actual_simulator_required": True,
                    "generated_imagery_allowed": False,
                    "paper_comparable": False,
                    "known_gate_blockers": [],
                    "evidence_ready": True,
                },
                f"{location}.capture_metadata.schedule",
                errors,
            )
        mismatches = capture_metadata.get("known_mismatches")
        if (
            not isinstance(mismatches, list)
            or not all(_nonempty_string(item) for item in mismatches)
            or not any(
                "checker-textured UV mesh is VisualAspect-only" in item
                for item in mismatches
            )
        ):
            errors.append(
                f"{location}.capture_metadata.known_mismatches: visual-only checker "
                "disclosure is required"
            )
        elif schedule is not None and schedule.get("known_mismatches") != mismatches:
            errors.append(
                f"{location}.capture_metadata.known_mismatches: schedule binding "
                "changed"
            )

        runtime = _object(
            capture_metadata.get("runtime"),
            f"{location}.capture_metadata.runtime",
            errors,
        )
        if runtime is not None:
            _expect_fields(
                runtime,
                {"demo_sha256": hashes.get("demo_binary")},
                f"{location}.capture_metadata.runtime",
                errors,
            )
            if schedule is not None and runtime.get("demo_argv") != schedule.get(
                "demo_argv"
            ):
                errors.append(
                    f"{location}.capture_metadata.runtime.demo_argv: schedule "
                    "binding changed"
                )
            for field in ("demo_path", "ffmpeg", "ffprobe"):
                if not _nonempty_string(runtime.get(field)):
                    errors.append(
                        f"{location}.capture_metadata.runtime.{field}: expected a "
                        "recorded executable path"
                    )

        timeline = _object(
            capture_metadata.get("timeline_validation"),
            f"{location}.capture_metadata.timeline_validation",
            errors,
        )
        if timeline is not None:
            _expect_fields(
                timeline,
                {
                    "pass": True,
                    "step_count": 131,
                    "shot_count": 131,
                    "action_count": 0,
                    "unique_frame_hashes": 131,
                },
                f"{location}.capture_metadata.timeline_validation",
                errors,
            )
            steps = timeline.get("steps")
            frames = timeline.get("frames")
            expected_step_keys = {str(step) for step in range(131)}
            if not isinstance(steps, dict) or set(steps) != expected_step_keys:
                errors.append(
                    f"{location}.capture_metadata.timeline_validation.steps: "
                    "expected every step 0 through 130"
                )
            if not isinstance(frames, dict) or set(frames) != expected_step_keys:
                errors.append(
                    f"{location}.capture_metadata.timeline_validation.frames: "
                    "expected every frame 0 through 130"
                )
            if isinstance(steps, dict) and set(steps) == expected_step_keys:
                capture_projection = []
                previous_counters = {
                    "exact_solves": 0,
                    "warm_starts": 0,
                    "boxed_lcp_fallbacks": 0,
                }
                for step in range(131):
                    step_location = (
                        f"{location}.capture_metadata.timeline_validation.steps[{step}]"
                    )
                    item = _object(steps[str(step)], step_location, errors)
                    if item is None:
                        continue
                    sim_time = _number(item.get("sim_time"))
                    if sim_time is None or not math.isclose(
                        sim_time, step / 60.0, rel_tol=0.0, abs_tol=1e-9
                    ):
                        errors.append(f"{step_location}.sim_time: expected step/60")
                    diagnostics = _object(
                        item.get("solver_diagnostics"),
                        f"{step_location}.solver_diagnostics",
                        errors,
                    )
                    if diagnostics is None:
                        continue
                    projection = {
                        "step": step,
                        "contacts": diagnostics.get("contacts"),
                        "exact_solves": diagnostics.get("exact_solves"),
                        "warm_starts": diagnostics.get("warm_starts"),
                        "boxed_lcp_fallbacks": diagnostics.get("boxed_lcp_fallbacks"),
                        "status": diagnostics.get("status"),
                    }
                    capture_projection.append(projection)
                    for field in (
                        "contacts",
                        "exact_solves",
                        "warm_starts",
                        "boxed_lcp_fallbacks",
                    ):
                        value = _integer(projection[field])
                        if value is None or value < 0:
                            errors.append(
                                f"{step_location}.solver_diagnostics.{field}: "
                                "expected a non-negative integer"
                            )
                            continue
                        if field in previous_counters:
                            if value < previous_counters[field]:
                                errors.append(
                                    f"{step_location}.solver_diagnostics.{field}: "
                                    "cumulative value decreased"
                                )
                            previous_counters[field] = value
                    for field in (
                        "accepted_at_cap",
                        "exact_failures",
                        "boxed_lcp_fallbacks",
                    ):
                        if diagnostics.get(field) != 0:
                            errors.append(
                                f"{step_location}.solver_diagnostics.{field}: "
                                "validated evidence requires 0"
                            )
                    if step == 0:
                        if diagnostics.get("status") != "not_run":
                            errors.append(
                                f"{step_location}.solver_diagnostics.status: "
                                "expected not_run"
                            )
                    elif diagnostics.get("status") != "success":
                        errors.append(
                            f"{step_location}.solver_diagnostics.status: expected "
                            "success"
                        )
                    worst = diagnostics.get("worst_residual")
                    if worst is not None:
                        value = _number(worst)
                        if (
                            value is None
                            or value < 0.0
                            or value > PAPER_RESIDUAL_TOLERANCE
                        ):
                            errors.append(
                                f"{step_location}.solver_diagnostics.worst_residual: "
                                "expected finite value <= 1e-6"
                            )
            if isinstance(frames, dict) and set(frames) == expected_step_keys:
                world_hashes = set()
                for step in range(131):
                    frame_location = f"{location}.capture_metadata.timeline_validation.frames[{step}]"
                    frame = _object(frames[str(step)], frame_location, errors)
                    if frame is None:
                        continue
                    frame_path = bundle / "backspin/frames" / f"step_{step:06d}.png"
                    _expect_fields(
                        frame,
                        {"path": str(frame_path)},
                        frame_location,
                        errors,
                    )
                    frame_sha = frame.get("sha256")
                    if not isinstance(frame_sha, str) or not SHA256_PATTERN.fullmatch(
                        frame_sha
                    ):
                        errors.append(
                            f"{frame_location}.sha256: expected lowercase SHA-256"
                        )
                    viewport = frame.get("world_viewport")
                    if isinstance(viewport, dict):
                        world_hash = viewport.get("sha256")
                        if isinstance(world_hash, str):
                            world_hashes.add(world_hash)
                        else:
                            errors.append(
                                f"{frame_location}.world_viewport.sha256: expected "
                                "a string"
                            )
                if None in world_hashes or len(world_hashes) != 131:
                    errors.append(
                        f"{location}.capture_metadata.timeline_validation.frames: "
                        "expected 131 distinct world-view hashes"
                    )

            final_diagnostics = _object(
                timeline.get("final_solver_diagnostics"),
                (
                    f"{location}.capture_metadata.timeline_validation."
                    "final_solver_diagnostics"
                ),
                errors,
            )
            if final_diagnostics is not None:
                _expect_fields(
                    final_diagnostics,
                    {
                        "available": True,
                        "accepted_at_cap": 0,
                        "exact_failures": 0,
                        "boxed_lcp_fallbacks": 0,
                        "status": "success",
                    },
                    (
                        f"{location}.capture_metadata.timeline_validation."
                        "final_solver_diagnostics"
                    ),
                    errors,
                )
                for field in ("exact_attempts", "exact_solves"):
                    value = _integer(final_diagnostics.get(field))
                    if value is None or value <= 0:
                        errors.append(
                            f"{location}.capture_metadata.timeline_validation."
                            f"final_solver_diagnostics.{field}: expected positive"
                        )
                worst = _number(final_diagnostics.get("worst_residual"))
                if worst is None or worst < 0.0 or worst > PAPER_RESIDUAL_TOLERANCE:
                    errors.append(
                        f"{location}.capture_metadata.timeline_validation."
                        "final_solver_diagnostics.worst_residual: expected finite "
                        "value <= 1e-6"
                    )
                if isinstance(steps, dict) and isinstance(steps.get("130"), dict):
                    if final_diagnostics != steps["130"].get("solver_diagnostics"):
                        errors.append(
                            f"{location}.capture_metadata.timeline_validation."
                            "final_solver_diagnostics: final-step binding changed"
                        )

        panel = _object(
            capture_metadata.get("panel_validation"),
            f"{location}.capture_metadata.panel_validation",
            errors,
        )
        panel_path = bundle / "backspin/panel.png"
        compose_path = bundle / "backspin/panel.compose.json"
        if panel is not None:
            compose = _read_json_object(
                compose_path, f"{location}.panel_compose", errors
            )
            _expect_fields(
                panel,
                {
                    "path": str(panel_path),
                    "sha256": hashes.get("backspin/panel.png"),
                    "compose_manifest_path": str(compose_path),
                    "compose_manifest_sha256": _sha256(compose_path),
                    "compose_manifest": compose,
                },
                f"{location}.capture_metadata.panel_validation",
                errors,
            )
            source_frames = panel.get("source_frames")
            expected_source_paths = [
                str(bundle / "backspin/panel_frames" / f"step_{step:06d}.png")
                for step in (0, 10, 50, 120, 130)
            ]
            if not isinstance(source_frames, list) or len(source_frames) != 5:
                errors.append(
                    f"{location}.capture_metadata.panel_validation.source_frames: "
                    "expected five records"
                )
            else:
                for index, (item, expected_path) in enumerate(
                    zip(source_frames, expected_source_paths)
                ):
                    item_location = (
                        f"{location}.capture_metadata.panel_validation."
                        f"source_frames[{index}]"
                    )
                    if not isinstance(item, dict):
                        errors.append(f"{item_location}: expected object")
                        continue
                    _expect_fields(item, {"path": expected_path}, item_location, errors)
                    digest = item.get("sha256")
                    if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(
                        digest
                    ):
                        errors.append(
                            f"{item_location}.sha256: expected lowercase SHA-256"
                        )
            verdict = _object(
                panel.get("verdict"),
                f"{location}.capture_metadata.panel_validation.verdict",
                errors,
            )
            if verdict is not None:
                image = _object(
                    verdict.get("image"),
                    f"{location}.capture_metadata.panel_validation.verdict.image",
                    errors,
                )
                _expect_fields(
                    verdict,
                    {"pass": True},
                    f"{location}.capture_metadata.panel_validation.verdict",
                    errors,
                )
                if image is not None:
                    _expect_fields(
                        image,
                        {"width": 6548, "height": 538, "path": str(panel_path)},
                        (
                            f"{location}.capture_metadata.panel_validation.verdict."
                            "image"
                        ),
                        errors,
                    )
            if not _has_image_signature(panel_path):
                errors.append(
                    f"{location}.capture_metadata.panel_validation: invalid PNG"
                )

        media_summaries: dict[str, dict[str, Any]] = {}
        media_items = capture_metadata.get("media_validation")
        if not isinstance(media_items, list) or any(
            not isinstance(item, dict) for item in media_items
        ):
            errors.append(
                f"{location}.capture_metadata.media_validation: expected objects"
            )
            media_items = []
        media_by_kind: dict[str, dict[str, Any]] = {}
        for item in media_items:
            kind = item.get("kind")
            if not isinstance(kind, str):
                errors.append(
                    f"{location}.capture_metadata.media_validation.kind: expected "
                    "a string"
                )
            else:
                media_by_kind[kind] = item
        if set(media_by_kind) != {"mp4", "gif"}:
            errors.append(
                f"{location}.capture_metadata.media_validation: expected mp4 and gif"
            )
        media_contracts = {
            "mp4": {
                "width": 1300,
                "height": 506,
                "frame_rate": "30/1",
                "frame_rate_rational": "30/1",
                "frame_count": 66,
            },
            "gif": {
                "width": 960,
                "height": 374,
                "frame_rate": "15/1",
                "frame_rate_rational": "15/1",
                "frame_count": 33,
            },
        }
        for kind, contract in media_contracts.items():
            item = media_by_kind.get(kind)
            if not isinstance(item, dict):
                continue
            path = bundle / "backspin" / f"clip.{kind}"
            _expect_fields(
                item,
                {
                    "kind": kind,
                    "path": str(path),
                    "sha256": hashes.get(f"backspin/clip.{kind}"),
                    "size_bytes": path.stat().st_size,
                    "full_decode": "pass",
                    "stream_contract": contract,
                },
                f"{location}.capture_metadata.media_validation.{kind}",
                errors,
            )
            probe = _object(
                item.get("probe"),
                f"{location}.capture_metadata.media_validation.{kind}.probe",
                errors,
            )
            if probe is not None:
                streams = probe.get("streams")
                if (
                    not isinstance(streams, list)
                    or len(streams) != 1
                    or not isinstance(streams[0], dict)
                ):
                    errors.append(
                        f"{location}.capture_metadata.media_validation.{kind}."
                        "probe.streams: expected one stream"
                    )
                else:
                    _expect_fields(
                        streams[0],
                        {
                            "width": contract["width"],
                            "height": contract["height"],
                            "r_frame_rate": contract["frame_rate"],
                            "nb_frames": str(contract["frame_count"]),
                        },
                        (
                            f"{location}.capture_metadata.media_validation.{kind}."
                            "probe.streams[0]"
                        ),
                        errors,
                    )
            if not _has_video_signature(path):
                errors.append(
                    f"{location}.capture_metadata.media_validation.{kind}: invalid "
                    "media signature"
                )
            media_summaries[kind] = {
                "path": str(path),
                "sha256": _sha256(path),
                "bytes": path.stat().st_size,
                "full_decode": "pass",
                "stream_contract": contract,
            }

        if run_summary is not None:
            _expect_fields(
                run_summary,
                {
                    "schema_version": "dart.fbf_visual_evidence/v1",
                    "kind": "capture_run",
                    "pass": True,
                    "failures": [],
                    "group_outputs": [],
                    "group_skips": [],
                    "results": [capture_metadata],
                },
                f"{location}.run_summary",
                errors,
            )
        if verification is not None:
            _expect_fields(
                verification,
                {
                    "schema_version": "dart.fbf_visual_evidence/v1",
                    "kind": "verification",
                    "pass": True,
                    "group_outputs": [],
                },
                f"{location}.verification",
                errors,
            )
            results = verification.get("results")
            if (
                not isinstance(results, list)
                or len(results) != 1
                or not isinstance(results[0], dict)
            ):
                errors.append(f"{location}.verification.results: expected one object")
            else:
                verified = results[0]
                _expect_fields(
                    verified,
                    {
                        "schedule": "backspin",
                        "pass": True,
                        "metadata_path": str(bundle / "backspin/metadata.json"),
                        "metadata_sha256": hashes.get("backspin/metadata.json"),
                    },
                    f"{location}.verification.results[0]",
                    errors,
                )
                if timeline is not None and verified.get("timeline") != timeline:
                    errors.append(
                        f"{location}.verification.results[0].timeline: stored/fresh "
                        "binding changed"
                    )
                verified_media = verified.get("media")
                if not isinstance(verified_media, list) or any(
                    not isinstance(item, dict) for item in verified_media
                ):
                    errors.append(
                        f"{location}.verification.results[0].media: expected objects"
                    )
                else:
                    by_kind: dict[str, dict[str, Any]] = {}
                    for item in verified_media:
                        kind = item.get("kind")
                        if not isinstance(kind, str):
                            errors.append(
                                f"{location}.verification.results[0].media.kind: "
                                "expected a string"
                            )
                        else:
                            by_kind[kind] = item
                    if set(by_kind) != {"mp4", "gif"}:
                        errors.append(
                            f"{location}.verification.results[0].media: expected mp4 "
                            "and gif"
                        )
                    for kind, contract in media_contracts.items():
                        item = by_kind.get(kind)
                        if isinstance(item, dict):
                            stored_item = media_by_kind.get(kind)
                            _expect_fields(
                                item,
                                {
                                    "kind": kind,
                                    "path": str(bundle / "backspin" / f"clip.{kind}"),
                                    "sha256": hashes.get(f"backspin/clip.{kind}"),
                                    "size_bytes": (bundle / "backspin" / f"clip.{kind}")
                                    .stat()
                                    .st_size,
                                    "full_decode": "pass",
                                    "stream_contract": contract,
                                    "probe": (
                                        stored_item.get("probe")
                                        if isinstance(stored_item, dict)
                                        else None
                                    ),
                                },
                                (
                                    f"{location}.verification.results[0].media."
                                    f"{kind}"
                                ),
                                errors,
                            )

        if timeline is not None:
            final_diagnostics = timeline.get("final_solver_diagnostics")
            if isinstance(final_diagnostics, dict) and capture_projection is not None:
                contact_free_steps = [
                    item["step"]
                    for item in capture_projection[1:]
                    if item.get("contacts") == 0
                ]
                capture_summary = {
                    "pass": True,
                    "metadata_sha256": hashes.get("backspin/metadata.json"),
                    "timeline_sha256": hashes.get("backspin/timeline.json"),
                    "panel_sha256": hashes.get("backspin/panel.png"),
                    "captured_frames": 131,
                    "unique_frames": 131,
                    "panel_steps": [0, 10, 50, 120, 130],
                    "exact_attempts": final_diagnostics.get("exact_attempts"),
                    "exact_solves": final_diagnostics.get("exact_solves"),
                    "accepted_at_cap": final_diagnostics.get("accepted_at_cap"),
                    "exact_failures": final_diagnostics.get("exact_failures"),
                    "boxed_lcp_fallbacks": final_diagnostics.get("boxed_lcp_fallbacks"),
                    "worst_residual": final_diagnostics.get("worst_residual"),
                    "contact_free_post_initial_steps": contact_free_steps,
                    "continuous_contact_proven": False,
                    "media": media_summaries,
                    "run_summary_sha256": hashes.get("run-summary.json"),
                    "verification_sha256": hashes.get("verification.json"),
                    "actual_simulator": True,
                    "generated_imagery": False,
                    "automated_semantic_outcome_validated": False,
                    "paper_comparable": False,
                    "solver_projection_sha256": _payload_sha256(capture_projection),
                }
                _expect_fields(
                    data,
                    {
                        "captured_frames": capture_summary["captured_frames"],
                        "exact_attempts": capture_summary["exact_attempts"],
                        "exact_solves": capture_summary["exact_solves"],
                        "accepted_at_cap": capture_summary["accepted_at_cap"],
                        "exact_failures": capture_summary["exact_failures"],
                        "boxed_lcp_fallbacks": capture_summary["boxed_lcp_fallbacks"],
                        "capture_max_residual": capture_summary["worst_residual"],
                        "contact_free_post_initial_steps": contact_free_steps,
                        "solver_projection_sha256": capture_summary[
                            "solver_projection_sha256"
                        ],
                    },
                    location,
                    errors,
                )
                if (
                    metadata is not None
                    and metadata.get("capture_summary") != capture_summary
                ):
                    errors.append(
                        f"{location}.metadata.capture_summary: recomputed binding "
                        "changed"
                    )

    comparison: dict[str, Any] | None = None
    if trace_metrics is not None and capture_projection is not None:
        trace_projection = trace_metrics["solver_projection"]
        if trace_projection != capture_projection:
            mismatch = next(
                (
                    index
                    for index, (trace_item, capture_item) in enumerate(
                        zip(trace_projection, capture_projection)
                    )
                    if trace_item != capture_item
                ),
                min(len(trace_projection), len(capture_projection)),
            )
            errors.append(
                f"{location}: trace/capture solver projection differs at {mismatch}"
            )
        else:
            digest = _payload_sha256(trace_projection)
            comparison = {
                "pass": True,
                "row_count": 131,
                "fields": [
                    "step",
                    "contacts",
                    "exact_solves",
                    "warm_starts",
                    "boxed_lcp_fallbacks",
                    "status",
                ],
                "trace_sha256": digest,
                "capture_sha256": digest,
                "byte_identical": True,
                "full_state_trace_equivalence": False,
            }
            _expect_fields(
                data,
                {
                    "solver_projection_equivalent": True,
                    "solver_projection_sha256": digest,
                },
                location,
                errors,
            )

    if trace_summary is not None:
        if not _nonempty_string(trace_summary.get("claim_boundary")):
            errors.append(
                f"{location}.trace_summary.claim_boundary: expected non-empty text"
            )
        stored_comparison = _object(
            trace_summary.get("solver_projection_comparison"),
            f"{location}.trace_summary.solver_projection_comparison",
            errors,
        )
        if stored_comparison is not None:
            if comparison is not None:
                _expect_fields(
                    stored_comparison,
                    comparison,
                    f"{location}.trace_summary.solver_projection_comparison",
                    errors,
                )
            if not _nonempty_string(stored_comparison.get("limitation")):
                errors.append(
                    f"{location}.trace_summary.solver_projection_comparison."
                    "limitation: expected non-empty text"
                )

    expected_verdicts = {
        "checker_texture_legible": True,
        "coral_registration_tile_visible": True,
        "orientation_change_visible_in_consecutive_source_frames": True,
        "texture_loading_warnings_observed": False,
        "translational_advance_then_reversal_visible": True,
        "translational_reversal_claim_requires_physical_trace": True,
        "physics_neutrality_requires_source_contract": True,
        "media_sampling_alias_disclosed": True,
        "signed_angular_direction_proven_by_media": False,
        "continuous_contact_proven": False,
        "strict_rigid_body_rest_proven": False,
        "paper_parity": False,
        "external_solver_parity": False,
        "approved_source_golden": False,
        "timing_verdict": None,
        "realtime_verdict": None,
    }
    if inspection is not None and bundle is not None:
        expected_top_keys = {
            "schema_version",
            "manual_inspected",
            "pass",
            "verdicts",
            "representative_artifacts",
        }
        if set(inspection) != expected_top_keys:
            errors.append(f"{location}.manual_inspection: top-level contract changed")
        _expect_fields(
            inspection,
            {
                "schema_version": "dart.fbf_backspin_manual_inspection/v3",
                "manual_inspected": True,
                "pass": True,
                "verdicts": expected_verdicts,
            },
            f"{location}.manual_inspection",
            errors,
        )
        representatives = inspection.get("representative_artifacts")
        expected_representatives = {
            "backspin/panel.png",
            "backspin/clip.mp4",
            "backspin/clip.gif",
            "backspin/stills/step_000000.png",
            "backspin/stills/step_000001.png",
            "backspin/stills/step_000002.png",
        }
        if not isinstance(representatives, list) or any(
            not isinstance(item, dict) for item in representatives
        ):
            errors.append(
                f"{location}.manual_inspection.representative_artifacts: "
                "expected objects"
            )
        else:
            paths = [item.get("path") for item in representatives]
            if any(not isinstance(path, str) for path in paths):
                errors.append(
                    f"{location}.manual_inspection.representative_artifacts: "
                    "paths must be strings"
                )
            elif set(paths) != expected_representatives or len(paths) != len(
                set(paths)
            ):
                errors.append(
                    f"{location}.manual_inspection.representative_artifacts: "
                    "exact paths changed"
                )
            for index, item in enumerate(representatives):
                item_location = (
                    f"{location}.manual_inspection.representative_artifacts[{index}]"
                )
                if set(item) != {"path", "sha256", "observation"}:
                    errors.append(f"{item_location}: contract changed")
                relative = item.get("path")
                if (
                    not isinstance(relative, str)
                    or relative not in expected_representatives
                ):
                    continue
                artifact = bundle / relative
                _expect_fields(
                    item,
                    {"sha256": _sha256(artifact)},
                    item_location,
                    errors,
                )
                if not _nonempty_string(item.get("observation")):
                    errors.append(
                        f"{item_location}.observation: expected non-empty text"
                    )

    if invocations is not None and bundle is not None:
        if set(invocations) != {"schema_version", "trace", "capture_verification"}:
            errors.append(f"{location}.invocations: top-level contract changed")
        _expect_fields(
            invocations,
            {"schema_version": "dart.fbf_backspin_invocations/v1"},
            f"{location}.invocations",
            errors,
        )
        identities = metadata.get("source_identity") if metadata is not None else None
        identities = identities if isinstance(identities, dict) else {}

        def identity_path(key: str) -> Any:
            identity = identities.get(key)
            return identity.get("path") if isinstance(identity, dict) else None

        trace_invocation = _object(
            invocations.get("trace"), f"{location}.invocations.trace", errors
        )
        if trace_invocation is not None:
            _expect_fields(
                trace_invocation,
                {
                    "argv": [
                        identity_path("trace_binary"),
                        "backspin",
                        "exact_fbf",
                        "1",
                        "130",
                        "nan",
                        "tracked",
                        "default",
                        "default",
                        "1",
                        "dart_best",
                        "dart",
                        "default",
                        "0",
                        "0",
                        "default",
                    ],
                    "returncode": 0,
                    "stdout_path": "traces/backspin.csv",
                    "stdout_sha256": hashes.get("traces/backspin.csv"),
                    "stderr_path": "traces/backspin.stderr.txt",
                    "stderr_sha256": _sha256(bundle / "traces/backspin.stderr.txt"),
                },
                f"{location}.invocations.trace",
                errors,
            )
        verification_invocation = _object(
            invocations.get("capture_verification"),
            f"{location}.invocations.capture_verification",
            errors,
        )
        if verification_invocation is not None:
            _expect_fields(
                verification_invocation,
                {
                    "argv": [
                        identity_path("python_binary"),
                        identity_path("visual_runner"),
                        "verify",
                        "--scenario",
                        "backspin",
                        "--demo",
                        identity_path("demo_binary"),
                        "--output-root",
                        str(bundle),
                        "--ffmpeg",
                        identity_path("ffmpeg_binary"),
                        "--ffprobe",
                        identity_path("ffprobe_binary"),
                    ],
                    "returncode": 0,
                    "stdout_path": "verification.json",
                    "stdout_sha256": hashes.get("verification.json"),
                    "stderr_path": "verification.stderr.txt",
                    "stderr_sha256": hashlib.sha256(b"").hexdigest(),
                },
                f"{location}.invocations.capture_verification",
                errors,
            )

    if artifact_index is not None and bundle is not None:
        expected_index_paths = _backspin_expected_bundle_paths() - {
            "artifact-index.json",
            "metadata.json",
        }
        _expect_fields(
            artifact_index,
            {
                "schema_version": "dart.fbf_backspin_artifact_index/v1",
                "artifact_count": len(expected_index_paths),
                "excluded": ["artifact-index.json", "metadata.json"],
            },
            f"{location}.artifact_index",
            errors,
        )
        artifacts = artifact_index.get("artifacts")
        if not isinstance(artifacts, list) or any(
            not isinstance(item, dict) for item in artifacts
        ):
            errors.append(f"{location}.artifact_index.artifacts: expected objects")
        else:
            listed = [item.get("path") for item in artifacts]
            valid_listed = all(isinstance(path, str) for path in listed)
            if not valid_listed:
                errors.append(
                    f"{location}.artifact_index.artifacts: paths must be strings"
                )
            elif listed != sorted(set(listed)):
                errors.append(
                    f"{location}.artifact_index.artifacts: paths must be unique "
                    "and sorted"
                )
            if valid_listed and set(listed) != expected_index_paths:
                errors.append(
                    f"{location}.artifact_index.artifacts: exact membership " "changed"
                )
            if artifact_index.get("artifact_count") != len(artifacts):
                errors.append(
                    f"{location}.artifact_index.artifact_count: list binding changed"
                )
            for index, item in enumerate(artifacts):
                item_location = f"{location}.artifact_index.artifacts[{index}]"
                if set(item) != {"path", "bytes", "sha256"}:
                    errors.append(f"{item_location}: contract changed")
                relative = item.get("path")
                if (
                    not isinstance(relative, str)
                    or relative not in expected_index_paths
                ):
                    continue
                path = bundle / relative
                _expect_fields(
                    item,
                    {"bytes": path.stat().st_size, "sha256": _sha256(path)},
                    item_location,
                    errors,
                )

    if metadata is not None:
        if not _nonempty_string(metadata.get("evidence_date")):
            errors.append(f"{location}.metadata.evidence_date: expected non-empty text")
        if not _nonempty_string(metadata.get("claim_scope")):
            errors.append(f"{location}.metadata.claim_scope: expected non-empty text")


def _validate_incline_v1_trace(
    bundle: Path,
    scenario: str,
    location: str,
    errors: list[str],
) -> dict[str, Any] | None:
    contract = INCLINE_V1_SCENARIOS[scenario]
    parsed = _read_csv(bundle / contract["trace"], location, errors)
    if parsed is None:
        return None
    fields, rows = parsed
    expected_fields = [
        "step",
        "time",
        "scenario",
        "solver",
        "body",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "up_z",
        "contacts",
        "exact_solves",
        "warm_starts",
        "fallbacks",
        "residual",
        "status",
    ]
    if fields != expected_fields:
        errors.append(f"{location}: expected columns {expected_fields!r}")
    if len(rows) != 121:
        errors.append(f"{location}: expected 121 rows including step 0")
        return None

    numeric_rows: list[dict[str, float | int]] = []
    max_residual = 0.0
    max_cross_speed = 0.0
    max_velocity_position_error = 0.0
    for index, row in enumerate(rows):
        row_location = f"{location}.rows[{index}]"
        step = _parse_csv_int(row.get("step"), f"{row_location}.step", errors)
        time = _parse_csv_number(row.get("time"), f"{row_location}.time", errors)
        contacts = _parse_csv_int(
            row.get("contacts"), f"{row_location}.contacts", errors
        )
        exact_solves = _parse_csv_int(
            row.get("exact_solves"), f"{row_location}.exact_solves", errors
        )
        warm_starts = _parse_csv_int(
            row.get("warm_starts"), f"{row_location}.warm_starts", errors
        )
        fallbacks = _parse_csv_int(
            row.get("fallbacks"), f"{row_location}.fallbacks", errors
        )
        state = {
            field: _parse_csv_number(row.get(field), f"{row_location}.{field}", errors)
            for field in ("x", "y", "z", "vx", "vy", "vz", "up_z")
        }
        if any(value is None for value in state.values()):
            continue
        if step != index:
            errors.append(f"{row_location}.step: expected {index}, got {step!r}")
        if time is not None and not math.isclose(
            time, index / 60.0, rel_tol=0.0, abs_tol=1e-12
        ):
            errors.append(f"{row_location}.time: expected step/60")
        _expect_fields(
            row,
            {
                "scenario": scenario,
                "solver": "exact_fbf",
                "body": "incline_cube_body",
            },
            row_location,
            errors,
        )
        if index == 0:
            if (contacts, exact_solves, warm_starts, fallbacks) != (0, 0, 0, 0):
                errors.append(f"{row_location}: step 0 counters changed")
            if row.get("status") != "not_run":
                errors.append(f"{row_location}.status: expected 'not_run'")
            try:
                residual = float(row.get("residual", ""))
            except ValueError:
                residual = 0.0
                errors.append(f"{row_location}.residual: expected NaN")
            if not math.isnan(residual):
                errors.append(f"{row_location}.residual: expected NaN")
        else:
            if contacts != 3:
                errors.append(f"{row_location}.contacts: expected 3")
            if exact_solves != index:
                errors.append(f"{row_location}.exact_solves: expected {index}")
            if warm_starts != index - 1:
                errors.append(f"{row_location}.warm_starts: expected {index - 1}")
            if fallbacks != 0:
                errors.append(f"{row_location}.fallbacks: expected 0")
            if row.get("status") != "success":
                errors.append(f"{row_location}.status: expected 'success'")
            residual = _parse_csv_number(
                row.get("residual"), f"{row_location}.residual", errors
            )
            if residual is not None:
                max_residual = max(max_residual, residual)
                if not 0.0 <= residual <= PAPER_RESIDUAL_TOLERANCE:
                    errors.append(f"{row_location}.residual: {residual} exceeds 1e-06")
        numeric = {field: float(value) for field, value in state.items()}
        numeric["step"] = index
        numeric_rows.append(numeric)
        max_cross_speed = max(max_cross_speed, abs(numeric["vy"]))
        if index > 0 and len(numeric_rows) >= 2:
            previous = numeric_rows[-2]
            dt = 1.0 / 60.0
            for position, velocity in (("x", "vx"), ("y", "vy"), ("z", "vz")):
                error = abs(
                    (numeric[position] - previous[position]) / dt - numeric[velocity]
                )
                max_velocity_position_error = max(max_velocity_position_error, error)
                if error > 1e-10:
                    errors.append(
                        f"{row_location}.{velocity}: finite-difference mismatch"
                    )

    if len(numeric_rows) != 121:
        return None
    initial = numeric_rows[0]
    final = numeric_rows[-1]
    cosine = 2.0 / math.sqrt(5.0)
    sine = 1.0 / math.sqrt(5.0)
    downhill_displacement = -(
        (final["x"] - initial["x"]) * cosine + (final["z"] - initial["z"]) * sine
    )
    final_downhill_speed = -(final["vx"] * cosine + final["vz"] * sine)
    maximum_cross_slope_displacement = max(
        abs(row["y"] - initial["y"]) for row in numeric_rows
    )
    initial_normal_coordinate = -sine * initial["x"] + cosine * initial["z"]
    maximum_normal_offset_error = max(
        abs(-sine * row["x"] + cosine * row["z"] - initial_normal_coordinate)
        for row in numeric_rows
    )
    maximum_normal_speed = max(
        abs(-sine * row["vx"] + cosine * row["vz"]) for row in numeric_rows
    )
    maximum_stick_speed = max(
        math.sqrt(row["vx"] ** 2 + row["vy"] ** 2 + row["vz"] ** 2)
        for row in numeric_rows
    )
    if contract["expected_outcome"] == "slide":
        if downhill_displacement <= 0.5:
            errors.append(f"{location}: slide displacement does not exceed 0.5 m")
        if final_downhill_speed < 0.5:
            errors.append(f"{location}: final downhill speed is below 0.5 m/s")
    else:
        if abs(downhill_displacement) > 0.02:
            errors.append(f"{location}: stick displacement exceeds 0.02 m")
        if maximum_stick_speed > 0.02:
            errors.append(f"{location}: stick speed exceeds 0.02 m/s")
    if max_cross_speed > 0.02:
        errors.append(f"{location}: cross-slope speed exceeds 0.02 m/s")
    if maximum_cross_slope_displacement > 0.02:
        errors.append(f"{location}: cross-slope displacement exceeds 0.02 m")
    if maximum_normal_offset_error > 0.02:
        errors.append(f"{location}: incline-normal offset exceeds 0.02 m")
    if maximum_normal_speed > 0.02:
        errors.append(f"{location}: incline-normal speed exceeds 0.02 m/s")
    return {
        "row_count": len(rows),
        "completed_steps": 120,
        "exact_solves": 120,
        "boxed_lcp_fallbacks": 0,
        "observed_contacts_per_step": 3,
        "downhill_displacement_m": downhill_displacement,
        "final_downhill_speed_m_s": final_downhill_speed,
        "maximum_stick_speed_m_s": maximum_stick_speed,
        "maximum_cross_slope_speed_m_s": max_cross_speed,
        "maximum_cross_slope_displacement_m": maximum_cross_slope_displacement,
        "maximum_incline_normal_center_offset_error_m": (maximum_normal_offset_error),
        "maximum_incline_normal_speed_m_s": maximum_normal_speed,
        "maximum_velocity_position_component_error_m_s": (max_velocity_position_error),
        "max_residual": max_residual,
    }


def _validate_incline_v1_capture_provenance(
    provenance: dict[str, Any] | None,
    source_identity: dict[str, Any] | None,
    run_summary: dict[str, Any] | None,
    bundle: Path,
    repo_root: Path,
    hashes: dict[str, str],
    location: str,
    errors: list[str],
) -> None:
    if provenance is None:
        return
    provenance_location = f"{location}.capture_provenance"
    expected_keys = {
        "argv",
        "binary_source_bindings_after",
        "binary_source_bindings_before",
        "durable_stills",
        "pruned_staging",
        "returncode",
        "run_summary_path",
        "run_summary_sha256",
        "run_summary_validated",
        "runtime_inputs_after",
        "runtime_inputs_before",
        "stderr_path",
        "stderr_sha256",
        "stdout_ends_with_run_summary",
        "stdout_path",
        "stdout_sha256",
    }
    if set(provenance) != expected_keys:
        errors.append(f"{provenance_location}: exact contract changed")

    python = str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["python_binary"])
    demo = str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["demo_binary"])
    ffmpeg = str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["ffmpeg_binary"])
    ffprobe = str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["ffprobe_binary"])
    expected_argv = [
        python,
        str(repo_root / "scripts/run_fbf_visual_evidence.py"),
        "run",
        "--scenario",
        "incline",
        "--demo",
        demo,
        "--output-root",
        str(bundle),
        "--ffmpeg",
        ffmpeg,
        "--ffprobe",
        ffprobe,
        "--python",
        python,
        "--out",
        str(bundle / "run-summary.json"),
    ]
    pruned = _validate_pruned_staging(
        bundle,
        provenance.get("pruned_staging"),
        schema_version="dart.fbf_incline_pruned_staging/v1",
        artifact_count=70,
        location=f"{provenance_location}.pruned_staging",
        errors=errors,
    )
    _validate_durable_stills(
        bundle,
        provenance.get("durable_stills"),
        tuple(
            (
                step,
                f"incline/stills/step_{step:06d}.png",
                f"incline/frames/step_{step:06d}.png",
            )
            for step in (0, 30, 60, 90, 120)
        ),
        f"{provenance_location}.durable_stills",
        errors,
    )
    _expect_fields(
        provenance,
        {
            "argv": expected_argv,
            "returncode": 0,
            "run_summary_path": "run-summary.json",
            "run_summary_sha256": hashes.get("run-summary.json"),
            "run_summary_validated": True,
            "stdout_path": "capture.stdout.txt",
            "stdout_sha256": pruned.get("capture.stdout.txt", {}).get("sha256"),
            "stderr_path": "capture.stderr.txt",
            "stderr_sha256": pruned.get("capture.stderr.txt", {}).get("sha256"),
            "stdout_ends_with_run_summary": True,
        },
        provenance_location,
        errors,
    )

    before = _object(
        provenance.get("runtime_inputs_before"),
        f"{provenance_location}.runtime_inputs_before",
        errors,
    )
    after = _object(
        provenance.get("runtime_inputs_after"),
        f"{provenance_location}.runtime_inputs_after",
        errors,
    )
    if before is not None and after is not None and before != after:
        errors.append(f"{provenance_location}.runtime_inputs: changed during capture")
    if source_identity is not None:
        expected_runtime = {
            key: source_identity.get(key)
            for key in sorted(INCLINE_V1_CAPTURE_RUNTIME_KEYS)
        }
        if before is not None and before != expected_runtime:
            errors.append(
                f"{provenance_location}.runtime_inputs_before: "
                "source identity binding changed"
            )
        if after is not None and after != expected_runtime:
            errors.append(
                f"{provenance_location}.runtime_inputs_after: "
                "source identity binding changed"
            )

        expected_bindings = source_identity.get("binary_source_bindings")
        for phase in ("before", "after"):
            field = f"binary_source_bindings_{phase}"
            if provenance.get(field) != expected_bindings:
                errors.append(
                    f"{provenance_location}.{field}: source identity binding changed"
                )

    stderr_record = pruned.get("capture.stderr.txt")
    if stderr_record is not None and stderr_record.get("bytes") != 0:
        errors.append(f"{provenance_location}.stderr_path: expected empty stream")
    if run_summary is not None and provenance.get("run_summary_validated") is not True:
        errors.append(f"{provenance_location}.run_summary_validated: expected true")


def _validate_incline_v1_invocations(
    invocations: dict[str, Any] | None,
    bundle: Path,
    repo_root: Path,
    hashes: dict[str, str],
    location: str,
    errors: list[str],
) -> None:
    if invocations is None:
        return
    invocation_location = f"{location}.invocations"
    if set(invocations) != {"schema_version", "capture_verification", "traces"}:
        errors.append(f"{invocation_location}: exact contract changed")
    _expect_fields(
        invocations,
        {"schema_version": "dart.fbf_incline_invocations/v1"},
        invocation_location,
        errors,
    )

    verification = _object(
        invocations.get("capture_verification"),
        f"{invocation_location}.capture_verification",
        errors,
    )
    if verification is not None:
        expected_argv = [
            str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["python_binary"]),
            str(repo_root / "scripts/run_fbf_visual_evidence.py"),
            "verify",
            "--scenario",
            "incline",
            "--demo",
            str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["demo_binary"]),
            "--output-root",
            str(bundle),
            "--ffmpeg",
            str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["ffmpeg_binary"]),
            "--ffprobe",
            str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["ffprobe_binary"]),
        ]
        _expect_fields(
            verification,
            {
                "argv": expected_argv,
                "returncode": 0,
                "stdout_path": "verification.json",
                "stdout_sha256": hashes.get("verification.json"),
                "stderr_path": "verification.stderr.txt",
                "stderr_sha256": hashlib.sha256(b"").hexdigest(),
            },
            f"{invocation_location}.capture_verification",
            errors,
        )
        if set(verification) != {
            "argv",
            "returncode",
            "stdout_path",
            "stdout_sha256",
            "stderr_path",
            "stderr_sha256",
        }:
            errors.append(
                f"{invocation_location}.capture_verification: contract changed"
            )

    traces = invocations.get("traces")
    if not isinstance(traces, list) or len(traces) != 2:
        errors.append(f"{invocation_location}.traces: expected two invocations")
        return
    by_scenario = {
        item.get("scenario"): item
        for item in traces
        if isinstance(item, dict) and isinstance(item.get("scenario"), str)
    }
    if set(by_scenario) != set(INCLINE_V1_SCENARIOS):
        errors.append(f"{invocation_location}.traces: identities changed")
    trace_binary = str(repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["trace_binary"])
    for scenario, contract in INCLINE_V1_SCENARIOS.items():
        invocation = by_scenario.get(scenario)
        if not isinstance(invocation, dict):
            continue
        trace_path = contract["trace"]
        invocation_item_location = f"{invocation_location}.traces.{scenario}"
        expected = {
            "scenario": scenario,
            "argv": [
                trace_binary,
                scenario,
                "exact_fbf",
                "1",
                "120",
                "nan",
                "tracked",
                "default",
                "default",
                "1",
                "dart_best",
                "dart",
                "default",
                "0",
                "0",
                "default",
            ],
            "returncode": 0,
            "stdout_path": trace_path,
            "stdout_sha256": hashes.get(trace_path),
            "stderr_path": f"traces/{scenario}.stderr.txt",
            "stderr_sha256": hashlib.sha256(b"").hexdigest(),
        }
        _expect_fields(invocation, expected, invocation_item_location, errors)
        if set(invocation) != set(expected):
            errors.append(f"{invocation_item_location}: contract changed")


def _validate_incline_v1_run_summary(
    run_summary: dict[str, Any] | None,
    bundle: Path,
    repo_root: Path,
    hashes: dict[str, str],
    projection_sha: str | None,
    location: str,
    errors: list[str],
) -> None:
    if run_summary is None:
        return
    summary_location = f"{location}.run_summary"
    if set(run_summary) != {
        "schema_version",
        "kind",
        "pass",
        "failures",
        "group_outputs",
        "group_skips",
        "results",
    }:
        errors.append(f"{summary_location}: exact contract changed")
    _expect_fields(
        run_summary,
        {
            "schema_version": "dart.fbf_visual_evidence/v1",
            "kind": "capture_run",
            "pass": True,
            "failures": [],
            "group_outputs": [],
            "group_skips": [],
        },
        summary_location,
        errors,
    )
    results = run_summary.get("results")
    if not isinstance(results, list) or len(results) != 1:
        errors.append(f"{summary_location}.results: expected one result")
        return
    result = _object(results[0], f"{summary_location}.results[0]", errors)
    if result is None:
        return
    _expect_fields(
        result,
        {
            "schema_version": "dart.fbf_visual_evidence/v1",
            "kind": "capture_result",
            "pass": True,
            "actual_simulator": True,
            "generated_imagery": False,
            "automated_semantic_outcome_validated": False,
            "paper_comparable": False,
        },
        f"{summary_location}.results[0]",
        errors,
    )
    schedule = _object(
        result.get("schedule"), f"{summary_location}.results[0].schedule", errors
    )
    if schedule is not None:
        _expect_fields(
            schedule,
            {
                "id": "incline",
                "scene": "fbf_paper_incline",
                "total_steps": 120,
                "time_step_seconds": 1.0 / 60.0,
                "collision_detector": "dart",
                "collision_detector_override": True,
                "actual_simulator_required": True,
                "configuration": {"mu_cells": "0.4,0.5", "tan_theta": "0.5"},
                "panel_steps": [0, 30, 60, 90, 120],
                "panel_labels": [
                    "t=0.00s",
                    "t=0.50s",
                    "t=1.00s",
                    "t=1.50s",
                    "t=2.00s",
                ],
            },
            f"{summary_location}.results[0].schedule",
            errors,
        )
    runtime = _object(
        result.get("runtime"), f"{summary_location}.results[0].runtime", errors
    )
    if runtime is not None:
        _expect_fields(
            runtime,
            {
                "demo_path": str(
                    repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["demo_binary"]
                ),
                "demo_sha256": hashes.get("demo_binary"),
                "ffmpeg": str(
                    repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["ffmpeg_binary"]
                ),
                "ffprobe": str(
                    repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS["ffprobe_binary"]
                ),
            },
            f"{summary_location}.results[0].runtime",
            errors,
        )
    timeline = _object(
        result.get("timeline_validation"),
        f"{summary_location}.results[0].timeline_validation",
        errors,
    )
    if timeline is not None:
        timeline_location = f"{summary_location}.results[0].timeline_validation"
        if set(timeline) != {
            "action_count",
            "final_solver_diagnostics",
            "frames",
            "pass",
            "shot_count",
            "sidecar",
            "step_count",
            "steps",
            "unique_frame_hashes",
        }:
            errors.append(f"{timeline_location}: exact contract changed")
        _expect_fields(
            timeline,
            {
                "pass": True,
                "step_count": 121,
                "shot_count": 61,
                "action_count": 0,
                "unique_frame_hashes": 61,
                "sidecar": str(bundle / "incline/timeline.json"),
            },
            timeline_location,
            errors,
        )
        steps = timeline.get("steps")
        expected_step_keys = {str(step) for step in range(121)}
        step_120_diagnostics: dict[str, Any] | None = None
        if not isinstance(steps, dict) or set(steps) != expected_step_keys:
            errors.append(
                f"{timeline_location}.steps: expected every step 0 through 120"
            )
        else:
            capture_projection = []
            previous_total_iterations = 0
            diagnostic_keys = {
                "accepted_at_cap",
                "available",
                "best_residual",
                "boxed_lcp_fallbacks",
                "contacts",
                "exact_attempts",
                "exact_failures",
                "exact_solves",
                "fbf_status",
                "iterations",
                "residual",
                "solver",
                "status",
                "total_iterations",
                "warm_starts",
                "worst_residual",
            }
            for step in range(121):
                step_location = f"{timeline_location}.steps[{step}]"
                item = _object(steps[str(step)], step_location, errors)
                if item is None:
                    continue
                if set(item) != {"sim_time", "solver_diagnostics"}:
                    errors.append(f"{step_location}: exact contract changed")
                sim_time = _number(item.get("sim_time"))
                if sim_time is None or not math.isclose(
                    sim_time, step / 60.0, rel_tol=0.0, abs_tol=1e-9
                ):
                    errors.append(f"{step_location}.sim_time: expected step/60")
                diagnostics = _object(
                    item.get("solver_diagnostics"),
                    f"{step_location}.solver_diagnostics",
                    errors,
                )
                if diagnostics is None:
                    continue
                if set(diagnostics) != diagnostic_keys:
                    errors.append(
                        f"{step_location}.solver_diagnostics: exact contract changed"
                    )
                common = {
                    "available": True,
                    "solver": "ExactCoulombFbfConstraintSolver",
                    "accepted_at_cap": 0,
                    "exact_attempts": 2 * step,
                    "exact_solves": 2 * step,
                    "exact_failures": 0,
                    "boxed_lcp_fallbacks": 0,
                    "contacts": 0 if step == 0 else 8,
                    "warm_starts": 0 if step == 0 else 2 * (step - 1),
                    "status": "not_run" if step == 0 else "success",
                    "fbf_status": "not_run" if step == 0 else "success",
                }
                _expect_fields(
                    diagnostics,
                    common,
                    f"{step_location}.solver_diagnostics",
                    errors,
                )
                for field in ("iterations", "total_iterations"):
                    value = _integer(diagnostics.get(field))
                    if value is None or value < 0:
                        errors.append(
                            f"{step_location}.solver_diagnostics.{field}: "
                            "expected a non-negative integer"
                        )
                total_iterations = _integer(diagnostics.get("total_iterations"))
                if total_iterations is not None:
                    if total_iterations < previous_total_iterations:
                        errors.append(
                            f"{step_location}.solver_diagnostics.total_iterations: "
                            "cumulative value decreased"
                        )
                    previous_total_iterations = total_iterations
                for field in (
                    "best_residual",
                    "residual",
                    "worst_residual",
                ):
                    value = diagnostics.get(field)
                    if step == 0:
                        if value is not None:
                            errors.append(
                                f"{step_location}.solver_diagnostics.{field}: "
                                "expected None before the first solve"
                            )
                    else:
                        number = _number(value)
                        if (
                            number is None
                            or number < 0.0
                            or number > PAPER_RESIDUAL_TOLERANCE
                        ):
                            errors.append(
                                f"{step_location}.solver_diagnostics.{field}: "
                                "expected finite value <= 1e-6"
                            )
                capture_projection.append(
                    {
                        "step": step,
                        "exact_solves": diagnostics.get("exact_solves"),
                        "boxed_lcp_fallbacks": diagnostics.get("boxed_lcp_fallbacks"),
                    }
                )
                if step == 120:
                    step_120_diagnostics = diagnostics
            capture_projection_sha = hashlib.sha256(
                json.dumps(
                    capture_projection,
                    sort_keys=True,
                    separators=(",", ":"),
                    ensure_ascii=True,
                    allow_nan=False,
                ).encode("utf-8")
            ).hexdigest()
            if capture_projection_sha != projection_sha:
                errors.append(
                    f"{timeline_location}.steps: aggregate count projection changed"
                )

        frames = timeline.get("frames")
        expected_frame_steps = tuple(range(0, 121, 2))
        expected_frame_keys = {str(step) for step in expected_frame_steps}
        if not isinstance(frames, dict) or set(frames) != expected_frame_keys:
            errors.append(
                f"{timeline_location}.frames: expected every captured even step "
                "0 through 120"
            )
        else:
            world_hashes = set()
            for step in expected_frame_steps:
                frame_location = f"{timeline_location}.frames[{step}]"
                frame = _object(frames[str(step)], frame_location, errors)
                if frame is None:
                    continue
                frame_path = bundle / "incline/frames" / f"step_{step:06d}.png"
                _expect_fields(
                    frame,
                    {"path": str(frame_path)},
                    frame_location,
                    errors,
                )
                frame_sha = frame.get("sha256")
                if not isinstance(frame_sha, str) or not SHA256_PATTERN.fullmatch(
                    frame_sha
                ):
                    errors.append(
                        f"{frame_location}.sha256: expected lowercase SHA-256"
                    )
                non_blank = _object(
                    frame.get("non_blank"), f"{frame_location}.non_blank", errors
                )
                if non_blank is not None and non_blank.get("pass") is not True:
                    errors.append(f"{frame_location}.non_blank.pass: expected true")
                viewport = _object(
                    frame.get("world_viewport"),
                    f"{frame_location}.world_viewport",
                    errors,
                )
                if viewport is not None:
                    _expect_fields(
                        viewport,
                        {"x": 260, "y": 58, "width": 660, "height": 506},
                        f"{frame_location}.world_viewport",
                        errors,
                    )
                    world_hash = viewport.get("sha256")
                    if not isinstance(world_hash, str) or not SHA256_PATTERN.fullmatch(
                        world_hash
                    ):
                        errors.append(
                            f"{frame_location}.world_viewport.sha256: expected "
                            "lowercase SHA-256"
                        )
                    else:
                        world_hashes.add(world_hash)
                    viewport_non_blank = _object(
                        viewport.get("non_blank"),
                        f"{frame_location}.world_viewport.non_blank",
                        errors,
                    )
                    if (
                        viewport_non_blank is not None
                        and viewport_non_blank.get("pass") is not True
                    ):
                        errors.append(
                            f"{frame_location}.world_viewport.non_blank.pass: "
                            "expected true"
                        )
            if len(world_hashes) != 61:
                errors.append(
                    f"{timeline_location}.frames: expected 61 distinct "
                    "world-view hashes"
                )
        final = _object(
            timeline.get("final_solver_diagnostics"),
            f"{timeline_location}.final_solver_diagnostics",
            errors,
        )
        if final is not None:
            _expect_fields(
                final,
                {
                    "available": True,
                    "solver": "ExactCoulombFbfConstraintSolver",
                    "status": "success",
                    "fbf_status": "success",
                    "accepted_at_cap": 0,
                    "exact_attempts": 240,
                    "exact_solves": 240,
                    "exact_failures": 0,
                    "boxed_lcp_fallbacks": 0,
                    "contacts": 8,
                    "worst_residual": 9.999836962261359e-7,
                },
                f"{timeline_location}.final_solver_diagnostics",
                errors,
            )
            if step_120_diagnostics is not None and final != step_120_diagnostics:
                errors.append(
                    f"{timeline_location}.final_solver_diagnostics: "
                    "step 120 binding changed"
                )
    output = schedule.get("output") if schedule is not None else None
    if isinstance(output, dict):
        _expect_fields(
            output,
            {
                "directory": str(bundle / "incline"),
                "timeline": str(bundle / "incline/timeline.json"),
                "panel": str(bundle / "incline/panel.png"),
                "mp4": str(bundle / "incline/clip.mp4"),
                "gif": None,
            },
            f"{summary_location}.results[0].schedule.output",
            errors,
        )


def _validate_incline_v1_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    key = "incline_visual_v1_nonpaper"
    location = f"current_truth.{key}"
    data = _object(current_truth.get(key), location, errors)
    if data is None:
        return
    bundle = _validate_current_path(
        data,
        "bundle",
        INCLINE_V1_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    expected_truth = {
        "status": "valid_current_source_nonpaper_incline",
        "artifact_valid": True,
        "solver_contract_valid": True,
        "physical_outcome_valid": True,
        "manual_inspected": True,
        "claim_valid": False,
        "requirement_ids": ["fig.01", "fig.02", "video.03_incline"],
        "paper_parity": False,
        "paper_comparable": False,
        "external_solver_parity": False,
        "approved_source_golden": False,
        "approved_source_golden_diff": False,
        "timing_verdict": None,
        "realtime_verdict": None,
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "manual_visual_outcome_validated": True,
        "trace_equivalence_to_rendered_demo": False,
        "aggregate_count_projection_equivalent": True,
        "aggregate_counts_only": True,
        "capture_trace_contact_count_equivalent": False,
        "per_cell_trace_equivalence": False,
        "full_state_trace_equivalence": False,
        "tracked_trace_continuous_contact_proven": True,
        "paper_reference_contact_count_match": False,
        "maximum_penetration_proven": False,
        "full_friction_sweep": False,
        "capture_sidecar_deliverable_validated": False,
        "steps": 120,
        "trace_rows_per_scenario": 121,
        "captured_frames": 61,
        "panel_dimensions": [3348, 538],
        "mp4_dimensions": [660, 506],
        "mp4_frames": 61,
        "mp4_fps": 30,
        "accepted_at_cap": 0,
        "exact_attempts": 240,
        "exact_solves": 240,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
        "trace_max_residual": 9.997210606407098e-7,
        "capture_max_residual": 9.999836962261359e-7,
        "slide_downhill_displacement_m": 1.7686892884927794,
        "slide_expected_downhill_displacement_m": 1.7548661487418349,
        "slide_final_downhill_speed_m_s": 1.7544655347780056,
        "stick_downhill_displacement_m": 0.0008905412965980523,
        "stick_maximum_speed_m_s": 0.001116442058867632,
        "slide_minus_stick_m": 1.7677987471961814,
        "capture_contacts_per_post_initial_step": 8,
        "trace_aggregate_contacts_per_post_initial_step": 6,
        "artifact_count": 21,
        "claim_scope": INCLINE_V1_CLAIM_SCOPE,
        "claim_boundary": INCLINE_V1_CLAIM_BOUNDARY,
    }
    _expect_fields(data, expected_truth, location, errors)
    expected_truth_keys = {
        "bundle",
        "aggregate_count_projection_sha256",
        "artifact_hashes",
        *expected_truth,
    }
    if set(data) != expected_truth_keys:
        errors.append(f"{location}: exact truth schema changed")
    projection_sha = data.get("aggregate_count_projection_sha256")
    if not isinstance(projection_sha, str) or not SHA256_PATTERN.fullmatch(
        projection_sha
    ):
        errors.append(
            f"{location}.aggregate_count_projection_sha256: expected lowercase SHA-256"
        )
    hashes = _validate_artifact_hashes(
        data,
        INCLINE_V1_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        recorded_hash_keys=INCLINE_V1_RECORDED_HASH_KEYS,
    )
    if bundle is None:
        return

    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    trace_summary = _read_bundle_json(
        bundle, "trace-summary.json", f"{location}.trace_summary", errors
    )
    invocations = _read_bundle_json(
        bundle, "invocations.json", f"{location}.invocations", errors
    )
    run_summary = _read_bundle_json(
        bundle, "run-summary.json", f"{location}.run_summary", errors
    )
    capture_provenance = _read_bundle_json(
        bundle,
        "capture-provenance.json",
        f"{location}.capture_provenance",
        errors,
    )
    source_identity: dict[str, Any] | None = None
    if metadata is not None:
        _expect_fields(
            metadata,
            {
                "schema_version": "dart.fbf_incline_visual_bundle/v1",
                "status": data.get("status"),
                "pass": True,
                "requirement_ids": ["fig.01", "fig.02", "video.03_incline"],
                "artifact_valid": data.get("artifact_valid"),
                "solver_contract_valid": data.get("solver_contract_valid"),
                "physical_outcome_valid": data.get("physical_outcome_valid"),
                "paper_parity": data.get("paper_parity"),
                "paper_comparable": data.get("paper_comparable"),
                "external_solver_parity": data.get("external_solver_parity"),
                "approved_source_golden": data.get("approved_source_golden"),
                "approved_source_golden_diff": data.get("approved_source_golden_diff"),
                "actual_simulator": data.get("actual_simulator"),
                "generated_imagery": data.get("generated_imagery"),
                "manual_visual_outcome_validated": data.get(
                    "manual_visual_outcome_validated"
                ),
                "automated_semantic_outcome_validated": data.get(
                    "automated_semantic_outcome_validated"
                ),
                "trace_equivalence_to_rendered_demo": data.get(
                    "trace_equivalence_to_rendered_demo"
                ),
                "aggregate_count_projection_equivalent": data.get(
                    "aggregate_count_projection_equivalent"
                ),
                "aggregate_counts_only": data.get("aggregate_counts_only"),
                "capture_trace_contact_count_equivalent": data.get(
                    "capture_trace_contact_count_equivalent"
                ),
                "per_cell_trace_equivalence": data.get("per_cell_trace_equivalence"),
                "full_state_trace_equivalence": data.get(
                    "full_state_trace_equivalence"
                ),
                "tracked_trace_continuous_contact_proven": data.get(
                    "tracked_trace_continuous_contact_proven"
                ),
                "paper_reference_contact_count_match": data.get(
                    "paper_reference_contact_count_match"
                ),
                "maximum_penetration_proven": data.get("maximum_penetration_proven"),
                "full_friction_sweep": data.get("full_friction_sweep"),
                "capture_sidecar_deliverable_validated": data.get(
                    "capture_sidecar_deliverable_validated"
                ),
                "timing_verdict": data.get("timing_verdict"),
                "realtime_verdict": data.get("realtime_verdict"),
                "claim_scope": data.get("claim_scope"),
                "claim_boundary": data.get("claim_boundary"),
            },
            f"{location}.metadata",
            errors,
        )
        if trace_summary is not None and metadata.get("trace_summary") != trace_summary:
            errors.append(f"{location}.metadata.trace_summary: file binding changed")
        capture_summary = _object(
            metadata.get("capture_summary"),
            f"{location}.metadata.capture_summary",
            errors,
        )
        if capture_summary is not None:
            _expect_fields(
                capture_summary,
                {
                    "pass": True,
                    "actual_simulator": True,
                    "generated_imagery": False,
                    "automated_semantic_outcome_validated": False,
                    "accepted_at_cap": 0,
                    "exact_attempts": 240,
                    "exact_solves": 240,
                    "exact_failures": 0,
                    "boxed_lcp_fallbacks": 0,
                    "captured_frames": 61,
                    "unique_frames": 61,
                    "observed_contacts_per_post_initial_step": 8,
                    "worst_residual": data.get("capture_max_residual"),
                    "count_projection_sha256": projection_sha,
                    "panel_sha256": hashes.get("incline/panel.png"),
                    "panel_steps": [0, 30, 60, 90, 120],
                    "metadata_sha256": hashes.get("incline/metadata.json"),
                    "timeline_sha256": hashes.get("incline/timeline.json"),
                    "run_summary_sha256": hashes.get("run-summary.json"),
                    "verification_sha256": hashes.get("verification.json"),
                    "paper_comparable": False,
                },
                f"{location}.metadata.capture_summary",
                errors,
            )
            media = _object(
                capture_summary.get("media"),
                f"{location}.metadata.capture_summary.media",
                errors,
            )
            if media is not None:
                _expect_fields(
                    media,
                    {
                        "sha256": hashes.get("incline/clip.mp4"),
                        "bytes": (bundle / "incline/clip.mp4").stat().st_size,
                        "full_decode": "pass",
                        "stream_contract": {
                            "width": 660,
                            "height": 506,
                            "frame_count": 61,
                            "frame_rate": "30/1",
                            "frame_rate_rational": "30/1",
                        },
                    },
                    f"{location}.metadata.capture_summary.media",
                    errors,
                )
        for field, filename in (
            ("artifact_index", "artifact-index.json"),
            ("manual_inspection", "manual-inspection.json"),
            ("verification", "verification.json"),
            ("invocations", "invocations.json"),
            ("run_summary", "run-summary.json"),
            ("report", "REPORT.md"),
        ):
            binding = _object(
                metadata.get(field), f"{location}.metadata.{field}", errors
            )
            if binding is not None:
                expected = {"path": filename, "sha256": hashes.get(filename)}
                if field == "artifact_index":
                    expected.update(
                        {
                            "artifact_count": 21,
                            "excluded": ["artifact-index.json", "metadata.json"],
                        }
                    )
                _expect_fields(
                    binding, expected, f"{location}.metadata.{field}", errors
                )
        if (
            capture_provenance is not None
            and metadata.get("capture_provenance") != capture_provenance
        ):
            errors.append(
                f"{location}.metadata.capture_provenance: file binding changed"
            )
        source_identity = _object(
            metadata.get("source_identity"),
            f"{location}.metadata.source_identity",
            errors,
        )
        if source_identity is not None:
            if set(source_identity) != INCLINE_V1_SOURCE_IDENTITY_KEYS:
                errors.append(
                    f"{location}.metadata.source_identity: identity closure changed"
                )
            for source_key in sorted(
                INCLINE_V1_SOURCE_IDENTITY_KEYS - {"binary_source_bindings"}
            ):
                identity = _object(
                    source_identity.get(source_key),
                    f"{location}.metadata.source_identity.{source_key}",
                    errors,
                )
                if identity is not None:
                    expected_path = str(
                        repo_root / INCLINE_V1_SOURCE_IDENTITY_PATHS[source_key]
                    )
                    _expect_fields(
                        identity,
                        {
                            "path": expected_path,
                            "sha256": hashes.get(source_key),
                        },
                        f"{location}.metadata.source_identity.{source_key}",
                        errors,
                    )
                    if set(identity) != {"path", "sha256"}:
                        errors.append(
                            f"{location}.metadata.source_identity.{source_key}: "
                            "contract changed"
                        )
            bindings = _object(
                source_identity.get("binary_source_bindings"),
                f"{location}.metadata.source_identity.binary_source_bindings",
                errors,
            )
            if bindings is not None:
                if set(bindings) != {"demo_source_binding", "trace_source_binding"}:
                    errors.append(
                        f"{location}.metadata.source_identity.binary_source_bindings: "
                        "expected demo and trace bindings"
                    )
                for binding_key, contract in INCLINE_V1_BINDING_CONTRACTS.items():
                    binding = _object(
                        bindings.get(binding_key),
                        f"{location}.metadata.source_identity.binary_source_bindings."
                        f"{binding_key}",
                        errors,
                    )
                    binary = source_identity.get(contract["binary"])
                    source = source_identity.get(contract["source"])
                    if (
                        binding is None
                        or not isinstance(binary, dict)
                        or not isinstance(source, dict)
                    ):
                        continue
                    if set(binding) != {
                        "query_argv",
                        "query_payload_sha256",
                        "binary_path",
                        "binary_sha256",
                        "implementation_source_path",
                        "implementation_source_sha256",
                        "validated_binary_binding",
                    }:
                        errors.append(
                            f"{location}.metadata.source_identity."
                            f"binary_source_bindings.{binding_key}: contract changed"
                        )
                    _expect_fields(
                        binding,
                        {
                            "query_argv": [
                                binary.get("path"),
                                *contract["argv_tail"],
                            ],
                            "binary_path": binary.get("path"),
                            "binary_sha256": binary.get("sha256"),
                            "implementation_source_path": source.get("path"),
                            "implementation_source_sha256": source.get("sha256"),
                            "validated_binary_binding": {
                                "implementation_source_sha256": source.get("sha256"),
                                "role": contract["role"],
                            },
                            "query_payload_sha256": hashes.get(contract["query_hash"]),
                        },
                        f"{location}.metadata.source_identity.binary_source_bindings."
                        f"{binding_key}",
                        errors,
                    )

    _validate_incline_v1_capture_provenance(
        capture_provenance,
        source_identity,
        run_summary,
        bundle,
        repo_root,
        hashes,
        location,
        errors,
    )
    _validate_incline_v1_invocations(
        invocations, bundle, repo_root, hashes, location, errors
    )
    _validate_incline_v1_run_summary(
        run_summary, bundle, repo_root, hashes, projection_sha, location, errors
    )

    computed: dict[str, dict[str, Any]] = {}
    for scenario in INCLINE_V1_SCENARIOS:
        result = _validate_incline_v1_trace(
            bundle,
            scenario,
            f"{location}.traces.{scenario}",
            errors,
        )
        if result is not None:
            computed[scenario] = result
    if set(computed) == set(INCLINE_V1_SCENARIOS):
        slide = computed["incline_mu_0_4"]
        stick = computed["incline_mu_0_5"]
        recomputed_truth_metrics = {
            "trace_max_residual": max(slide["max_residual"], stick["max_residual"]),
            "slide_downhill_displacement_m": slide["downhill_displacement_m"],
            "slide_final_downhill_speed_m_s": slide["final_downhill_speed_m_s"],
            "stick_downhill_displacement_m": stick["downhill_displacement_m"],
            "stick_maximum_speed_m_s": stick["maximum_stick_speed_m_s"],
            "slide_minus_stick_m": (
                slide["downhill_displacement_m"] - stick["downhill_displacement_m"]
            ),
        }
        for field, recomputed in recomputed_truth_metrics.items():
            declared = _number(data.get(field))
            if declared is None or not math.isclose(
                declared, float(recomputed), rel_tol=0.0, abs_tol=1e-12
            ):
                errors.append(f"{location}.{field}: raw traces changed")
    projection = [
        {"step": step, "exact_solves": step * 2, "boxed_lcp_fallbacks": 0}
        for step in range(121)
    ]
    computed_projection_sha = hashlib.sha256(
        json.dumps(
            projection,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    if projection_sha != computed_projection_sha:
        errors.append(
            f"{location}.aggregate_count_projection_sha256: raw traces changed"
        )
    if trace_summary is not None:
        _expect_fields(
            trace_summary,
            {
                "schema_version": "dart.fbf_incline_trace_summary/v1",
                "pass": True,
                "physical_outcome_valid": True,
                "aggregate_count_projection_sha256": computed_projection_sha,
                "aggregate_projection_fields": [
                    "step",
                    "exact_solves",
                    "boxed_lcp_fallbacks",
                ],
                "trace_aggregate_contacts_per_post_initial_step": 6,
                "full_state_trace_equivalence": False,
            },
            f"{location}.trace_summary",
            errors,
        )
        scenarios = trace_summary.get("scenarios")
        if not isinstance(scenarios, list) or len(scenarios) != 2:
            errors.append(f"{location}.trace_summary.scenarios: expected two objects")
        else:
            by_name = {
                item.get("scenario"): item
                for item in scenarios
                if isinstance(item, dict) and isinstance(item.get("scenario"), str)
            }
            if set(by_name) != set(INCLINE_V1_SCENARIOS):
                errors.append(f"{location}.trace_summary.scenarios: identities changed")
            for scenario, result in computed.items():
                reported = by_name.get(scenario)
                if not isinstance(reported, dict):
                    continue
                for field in (
                    "row_count",
                    "completed_steps",
                    "exact_solves",
                    "boxed_lcp_fallbacks",
                    "observed_contacts_per_step",
                ):
                    if reported.get(field) != result[field]:
                        errors.append(
                            f"{location}.trace_summary.{scenario}.{field}: raw trace changed"
                        )
                if reported.get("continuous_contact_post_initial") is not True:
                    errors.append(
                        f"{location}.trace_summary.{scenario}."
                        "continuous_contact_post_initial: expected true"
                    )
                for field in ("downhill_displacement_m", "max_residual"):
                    value = _number(reported.get(field))
                    if value is None or not math.isclose(
                        value, float(result[field]), rel_tol=0.0, abs_tol=1e-12
                    ):
                        errors.append(
                            f"{location}.trace_summary.{scenario}.{field}: raw trace changed"
                        )
                invariants = _object(
                    reported.get("trajectory_invariants"),
                    f"{location}.trace_summary.{scenario}.trajectory_invariants",
                    errors,
                )
                if invariants is not None:
                    invariant_fields = (
                        "final_downhill_speed_m_s",
                        "maximum_cross_slope_displacement_m",
                        "maximum_cross_slope_speed_m_s",
                        "maximum_incline_normal_center_offset_error_m",
                        "maximum_incline_normal_speed_m_s",
                        "maximum_velocity_position_component_error_m_s",
                    )
                    for field in invariant_fields:
                        value = _number(invariants.get(field))
                        if value is None or not math.isclose(
                            value,
                            float(result[field]),
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        ):
                            errors.append(
                                f"{location}.trace_summary.{scenario}."
                                f"trajectory_invariants.{field}: raw trace changed"
                            )
                    if scenario == "incline_mu_0_5":
                        value = _number(invariants.get("maximum_stick_speed_m_s"))
                        if value is None or not math.isclose(
                            value,
                            float(result["maximum_stick_speed_m_s"]),
                            rel_tol=0.0,
                            abs_tol=1e-12,
                        ):
                            errors.append(
                                f"{location}.trace_summary.{scenario}."
                                "trajectory_invariants.maximum_stick_speed_m_s: "
                                "raw trace changed"
                            )
        comparison = _object(
            trace_summary.get("aggregate_count_projection_comparison"),
            f"{location}.trace_summary.aggregate_count_projection_comparison",
            errors,
        )
        if comparison is not None:
            _expect_fields(
                comparison,
                {
                    "pass": True,
                    "aggregate_counts_only": True,
                    "byte_identical": True,
                    "fields": [
                        "step",
                        "exact_solves",
                        "boxed_lcp_fallbacks",
                    ],
                    "capture_sha256": computed_projection_sha,
                    "trace_sha256": computed_projection_sha,
                    "capture_contacts_per_post_initial_step": 8,
                    "trace_aggregate_contacts_per_post_initial_step": 6,
                    "contact_counts_compared": False,
                    "contact_count_match": False,
                    "per_cell_trace_equivalence": False,
                    "full_state_trace_equivalence": False,
                    "row_count": 121,
                },
                f"{location}.trace_summary.aggregate_count_projection_comparison",
                errors,
            )

    artifact_index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )
    if artifact_index is not None:
        _expect_fields(
            artifact_index,
            {
                "schema_version": "dart.fbf_incline_artifact_index/v1",
                "artifact_count": 21,
                "excluded": ["artifact-index.json", "metadata.json"],
            },
            f"{location}.artifact_index",
            errors,
        )
        artifacts = artifact_index.get("artifacts")
        if not isinstance(artifacts, list) or any(
            not isinstance(item, dict) for item in artifacts
        ):
            errors.append(f"{location}.artifact_index.artifacts: expected objects")
        else:
            listed = [item.get("path") for item in artifacts]
            disk_paths = {
                path.relative_to(bundle).as_posix()
                for path in bundle.rglob("*")
                if path.is_file()
                and path.relative_to(bundle).as_posix()
                not in {"artifact-index.json", "metadata.json"}
            }
            if listed != sorted(set(listed)):
                errors.append(
                    f"{location}.artifact_index.artifacts: paths must be sorted/unique"
                )
            if set(listed) != disk_paths:
                errors.append(
                    f"{location}.artifact_index.artifacts: exact membership changed"
                )
            for index, item in enumerate(artifacts):
                relative = item.get("path")
                if not isinstance(relative, str) or relative not in disk_paths:
                    continue
                path = bundle / relative
                item_location = f"{location}.artifact_index.artifacts[{index}]"
                if path.is_symlink():
                    errors.append(f"{item_location}: symlink forbidden")
                    continue
                if set(item) != {"path", "bytes", "sha256"}:
                    errors.append(f"{item_location}: contract changed")
                _expect_fields(
                    item,
                    {"bytes": path.stat().st_size, "sha256": _sha256(path)},
                    item_location,
                    errors,
                )

    manual = _read_bundle_json(
        bundle, "manual-inspection.json", f"{location}.manual_inspection", errors
    )
    if manual is not None:
        _expect_fields(
            manual,
            {
                "schema_version": "dart.fbf_incline_manual_inspection/v1",
                "manual_inspected": True,
                "pass": True,
            },
            f"{location}.manual_inspection",
            errors,
        )
        verdicts = _object(
            manual.get("verdicts"), f"{location}.manual_inspection.verdicts", errors
        )
        if verdicts is not None:
            _expect_fields(
                verdicts,
                {
                    "two_cell_threshold_comparison_visible": True,
                    "mu_0_4_downhill_slide_visible": True,
                    "mu_0_5_near_threshold_stick_visible": True,
                    "physical_outcome_claim_requires_independent_traces": True,
                    "aggregate_count_projection_only": True,
                    "full_state_trace_equivalence": False,
                    "maximum_penetration_proven": False,
                    "paper_reference_contact_count_match": False,
                    "paper_parity": False,
                    "external_solver_parity": False,
                    "approved_source_golden": False,
                    "approved_source_golden_diff": False,
                    "capture_sidecar_deliverable_validated": False,
                    "full_friction_sweep": False,
                    "timing_verdict": None,
                    "realtime_verdict": None,
                },
                f"{location}.manual_inspection.verdicts",
                errors,
            )
        representatives = manual.get("representative_artifacts")
        if not isinstance(representatives, list) or len(representatives) != 7:
            errors.append(
                f"{location}.manual_inspection.representative_artifacts: expected seven"
            )
        else:
            representative_paths = [
                item.get("path") if isinstance(item, dict) else None
                for item in representatives
            ]
            if representative_paths != list(INCLINE_V1_REPRESENTATIVE_ARTIFACTS):
                errors.append(
                    f"{location}.manual_inspection.representative_artifacts: "
                    "expected exact ordered unique paths"
                )
            for index, item in enumerate(representatives):
                item_location = (
                    f"{location}.manual_inspection.representative_artifacts[{index}]"
                )
                if not isinstance(item, dict):
                    errors.append(f"{item_location}: expected object")
                    continue
                relative = item.get("path")
                if not isinstance(relative, str) or not (bundle / relative).is_file():
                    errors.append(f"{item_location}.path: missing artifact")
                    continue
                _expect_fields(
                    item,
                    {"sha256": _sha256(bundle / relative)},
                    item_location,
                    errors,
                )
                if not _nonempty_string(item.get("observation")):
                    errors.append(
                        f"{item_location}.observation: expected non-empty text"
                    )

    verification = _read_bundle_json(
        bundle, "verification.json", f"{location}.verification", errors
    )
    if verification is not None:
        _expect_fields(
            verification,
            {
                "schema_version": "dart.fbf_visual_evidence/v1",
                "kind": "verification",
                "pass": True,
                "group_outputs": [],
            },
            f"{location}.verification",
            errors,
        )
        results = verification.get("results")
        if not isinstance(results, list) or len(results) != 1:
            errors.append(f"{location}.verification.results: expected one")
        elif not isinstance(results[0], dict):
            errors.append(f"{location}.verification.results[0]: expected object")
        else:
            result = results[0]
            _expect_fields(
                result,
                {
                    "pass": True,
                    "schedule": "incline",
                    "metadata_sha256": hashes.get("incline/metadata.json"),
                },
                f"{location}.verification.results[0]",
                errors,
            )
            panel = _object(
                result.get("panel"),
                f"{location}.verification.results[0].panel",
                errors,
            )
            if panel is not None:
                _expect_fields(
                    panel,
                    {
                        "pass": True,
                        "image": {
                            "path": str(bundle / "incline/panel.png"),
                            "width": 3348,
                            "height": 538,
                        },
                    },
                    f"{location}.verification.results[0].panel",
                    errors,
                )
            media = result.get("media")
            if not isinstance(media, list) or len(media) != 1:
                errors.append(f"{location}.verification.results[0].media: expected one")
            elif isinstance(media[0], dict):
                _expect_fields(
                    media[0],
                    {
                        "kind": "mp4",
                        "sha256": hashes.get("incline/clip.mp4"),
                        "size_bytes": (bundle / "incline/clip.mp4").stat().st_size,
                        "full_decode": "pass",
                        "stream_contract": {
                            "width": 660,
                            "height": 506,
                            "frame_count": 61,
                            "frame_rate": "30/1",
                            "frame_rate_rational": "30/1",
                        },
                    },
                    f"{location}.verification.results[0].media[0]",
                    errors,
                )
            else:
                errors.append(
                    f"{location}.verification.results[0].media[0]: expected object"
                )


def _validate_painleve_v1_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.painleve_proxy_visual_v1_nonpaper"
    data = _object(
        current_truth.get("painleve_proxy_visual_v1_nonpaper"), location, errors
    )
    if data is None:
        return
    bundle = _validate_current_path(
        data,
        "bundle",
        PAINLEVE_V1_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    _expect_fields(
        data,
        {
            "status": "valid_current_source_nonpaper_proxy",
            "paper_parity": False,
            "paper_comparable": False,
            "external_solver_parity": False,
            "approved_source_golden": False,
            "timing_verdict": None,
            "realtime_verdict": None,
            "actual_simulator": True,
            "generated_imagery": False,
            "manual_visual_outcome_validated": True,
            "automated_semantic_outcome_validated": False,
            "trace_equivalence_to_rendered_demo": False,
            "strict_rigid_body_rest_proven": False,
            "capture_sidecar_deliverable_validated": False,
            "steps": 150,
            "trace_rows_per_scenario": 151,
            "member_frames": 76,
            "group_dimensions": [1320, 530],
            "group_video_frames": 76,
            "group_video_fps": 30,
            "exact_failures": 0,
            "boxed_lcp_fallbacks": 0,
            "artifact_count": 27,
        },
        location,
        errors,
    )
    for field in ("trace_max_residual", "capture_max_residual"):
        residual = _number(data.get(field))
        if residual is None or not 0.0 <= residual <= PAPER_RESIDUAL_TOLERANCE:
            errors.append(f"{location}.{field}: expected finite value <= 1e-6")
    margin = _number(data.get("shorter_pre_tumble_margin_m"))
    if margin is None or margin <= 0.0:
        errors.append(f"{location}.shorter_pre_tumble_margin_m: expected positive")

    hashes = _validate_artifact_hashes(
        data,
        PAINLEVE_V1_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        recorded_hash_keys={"trace_binary", "demo_binary"},
    )
    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    trace_summary = _read_bundle_json(
        bundle, "trace-summary.json", f"{location}.trace_summary", errors
    )
    if metadata is not None:
        _expect_fields(
            metadata,
            {
                "schema_version": "dart.fbf_painleve_proxy_visual_bundle/v1",
                "status": data.get("status"),
                "pass": True,
                "paper_parity": data.get("paper_parity"),
                "actual_simulator": data.get("actual_simulator"),
                "generated_imagery": data.get("generated_imagery"),
                "automated_semantic_outcome_validated": data.get(
                    "automated_semantic_outcome_validated"
                ),
                "manual_visual_outcome_validated": data.get(
                    "manual_visual_outcome_validated"
                ),
                "trace_equivalence_to_rendered_demo": data.get(
                    "trace_equivalence_to_rendered_demo"
                ),
            },
            f"{location}.metadata",
            errors,
        )
        if trace_summary is not None and metadata.get("trace_summary") != trace_summary:
            errors.append(f"{location}.metadata.trace_summary: file binding changed")
        index_binding = _object(
            metadata.get("artifact_index"),
            f"{location}.metadata.artifact_index",
            errors,
        )
        if index_binding is not None:
            _expect_fields(
                index_binding,
                {
                    "path": "artifact-index.json",
                    "sha256": hashes.get("artifact-index.json"),
                    "artifact_count": data.get("artifact_count"),
                    "excluded": ["artifact-index.json", "metadata.json"],
                },
                f"{location}.metadata.artifact_index",
                errors,
            )
        source_identity = _object(
            metadata.get("source_identity"),
            f"{location}.metadata.source_identity",
            errors,
        )
        if source_identity is not None:
            for source_key, hash_key in (
                ("finalizer", "finalizer"),
                ("finalizer_test", "finalizer_test"),
                ("visual_runner", "visual_runner"),
                ("visual_runner_test", "visual_runner_test"),
                ("trace_source", "trace_source"),
                ("fixture_source", "fixture_source"),
                ("demo_source", "demo_source"),
                ("trace_binary", "trace_binary"),
                ("demo_binary", "demo_binary"),
            ):
                identity = _object(
                    source_identity.get(source_key),
                    f"{location}.metadata.source_identity.{source_key}",
                    errors,
                )
                if identity is not None:
                    _expect_fields(
                        identity,
                        {"sha256": hashes.get(hash_key)},
                        f"{location}.metadata.source_identity.{source_key}",
                        errors,
                    )

    trace_by_scenario: dict[str, dict[str, Any]] = {}
    if trace_summary is not None:
        _expect_fields(
            trace_summary,
            {
                "schema_version": "dart.fbf_painleve_proxy_trace_summary/v1",
                "pass": True,
            },
            f"{location}.trace_summary",
            errors,
        )
        scenarios = trace_summary.get("scenarios")
        if not isinstance(scenarios, list) or len(scenarios) != 2:
            errors.append(f"{location}.trace_summary.scenarios: expected two objects")
        else:
            for index, scenario_data in enumerate(scenarios):
                if not isinstance(scenario_data, dict):
                    errors.append(
                        f"{location}.trace_summary.scenarios[{index}]: expected object"
                    )
                    continue
                name = scenario_data.get("scenario")
                if not isinstance(name, str) or name in trace_by_scenario:
                    errors.append(
                        f"{location}.trace_summary.scenarios[{index}]: bad identity"
                    )
                    continue
                trace_by_scenario[name] = scenario_data
        pair = _object(
            trace_summary.get("paired_contract"),
            f"{location}.trace_summary.paired_contract",
            errors,
        )
        if pair is not None:
            _expect_fields(
                pair,
                {
                    "mu055_first_tumble_step": data.get("mu055", {}).get(
                        "first_tumble_step"
                    ),
                    "mu055_first_tumble_time_seconds": data.get("mu055", {}).get(
                        "first_tumble_time_s"
                    ),
                    "mu055_first_tumble_travel": data.get("mu055", {}).get(
                        "first_tumble_x_m"
                    ),
                    "mu05_rest_travel": data.get("mu05", {}).get("final_x_m"),
                    "shorter_by": data.get("shorter_pre_tumble_margin_m"),
                    "pass": True,
                },
                f"{location}.trace_summary.paired_contract",
                errors,
            )

    computed_metrics: dict[str, dict[str, Any] | None] = {}
    if bundle is not None:
        for scenario in ("painleve_mu_0_5", "painleve_mu_0_55"):
            path = bundle / "traces" / f"{scenario}.csv"
            parsed = _read_csv(path, f"{location}.traces.{scenario}", errors)
            computed_metrics[scenario] = _painleve_trace_metrics(
                parsed, scenario, f"{location}.traces.{scenario}", errors
            )

    for scenario, ledger_key in (
        ("painleve_mu_0_5", "mu05"),
        ("painleve_mu_0_55", "mu055"),
    ):
        ledger = _object(data.get(ledger_key), f"{location}.{ledger_key}", errors)
        summary = trace_by_scenario.get(scenario)
        metrics = computed_metrics.get(scenario)
        if ledger is None or summary is None or metrics is None:
            continue
        settled = _object(
            summary.get("settled_proxy"),
            f"{location}.trace_summary.{scenario}.settled_proxy",
            errors,
        )
        if settled is None:
            continue
        common = {
            "exact_solves": metrics["exact_solves"],
            "final_x_m": metrics["final_x_m"],
            "final_z_m": metrics["final_z_m"],
            "final_up_z": metrics["final_up_z"],
            "max_tail_horizontal_speed_m_s": metrics["max_tail_horizontal_speed_m_s"],
            "tail_horizontal_drift_m": metrics["tail_horizontal_drift_m"],
            "tail_up_z_range": metrics["tail_up_z_range"],
            "tail_z_excursion_m": metrics["tail_z_excursion_m"],
        }
        if scenario == "painleve_mu_0_5":
            common["first_tumble"] = None
        else:
            first = metrics["first_tumble"]
            if not isinstance(first, dict):
                errors.append(
                    f"{location}.traces.{scenario}: missing recomputed first tumble"
                )
            else:
                common.update(
                    {
                        "first_tumble_step": first["step"],
                        "first_tumble_time_s": first["time"],
                        "first_tumble_x_m": first["x"] - metrics["initial_x_m"],
                    }
                )
        _expect_fields(ledger, common, f"{location}.{ledger_key}", errors)
        _expect_fields(
            summary,
            {
                "row_count": data.get("trace_rows_per_scenario"),
                "completed_steps": data.get("steps"),
                "exact_solves": metrics["exact_solves"],
                "boxed_lcp_fallbacks": 0,
                "max_residual": metrics["max_residual"],
                "physical_outcome_valid": True,
            },
            f"{location}.trace_summary.{scenario}",
            errors,
        )
        _expect_fields(
            settled,
            {
                "strict_rigid_body_rest_proven": False,
                "pass": True,
                "max_window_horizontal_speed": metrics["max_tail_horizontal_speed_m_s"],
                "window_horizontal_drift": metrics["tail_horizontal_drift_m"],
                "window_up_z_range": metrics["tail_up_z_range"],
                "window_z_excursion": metrics["tail_z_excursion_m"],
            },
            f"{location}.trace_summary.{scenario}.settled_proxy",
            errors,
        )

    expected_scenarios = {"painleve_mu_0_5", "painleve_mu_0_55"}
    if set(computed_metrics) == expected_scenarios and all(
        computed_metrics.get(name) is not None for name in expected_scenarios
    ):
        slide = computed_metrics["painleve_mu_0_5"]
        tumble = computed_metrics["painleve_mu_0_55"]
        if slide is None or tumble is None:
            errors.append(f"{location}.traces: incomplete recomputed metrics")
            return
        first = tumble["first_tumble"]
        if isinstance(first, dict):
            recomputed_margin = (slide["final_x_m"] - slide["initial_x_m"]) - (
                first["x"] - tumble["initial_x_m"]
            )
            declared_margin = _number(data.get("shorter_pre_tumble_margin_m"))
            if declared_margin is not None and not math.isclose(
                declared_margin, recomputed_margin, rel_tol=1e-12, abs_tol=1e-15
            ):
                errors.append(
                    f"{location}.shorter_pre_tumble_margin_m: expected "
                    f"{recomputed_margin}, got {declared_margin}"
                )
        trace_max = max(slide["max_residual"], tumble["max_residual"])
        if data.get("trace_max_residual") != trace_max:
            errors.append(
                f"{location}.trace_max_residual: expected {trace_max}, got "
                f"{data.get('trace_max_residual')}"
            )

    if metadata is not None:
        capture = _object(
            metadata.get("capture_summary"),
            f"{location}.metadata.capture_summary",
            errors,
        )
        if capture is not None:
            _expect_fields(
                capture,
                {
                    "pass": True,
                    "actual_simulator": data.get("actual_simulator"),
                    "generated_imagery": data.get("generated_imagery"),
                    "automated_semantic_outcome_validated": data.get(
                        "automated_semantic_outcome_validated"
                    ),
                    "paper_comparable": False,
                },
                f"{location}.metadata.capture_summary",
                errors,
            )
            capture_scenarios = capture.get("scenarios")
            if not isinstance(capture_scenarios, list) or len(capture_scenarios) != 2:
                errors.append(
                    f"{location}.metadata.capture_summary.scenarios: expected two"
                )
            else:
                residuals = []
                for index, item in enumerate(capture_scenarios):
                    item_location = (
                        f"{location}.metadata.capture_summary.scenarios[{index}]"
                    )
                    if not isinstance(item, dict):
                        errors.append(f"{item_location}: expected object")
                        continue
                    _expect_fields(
                        item,
                        {
                            "captured_frames": data.get("member_frames"),
                            "unique_frames": data.get("member_frames"),
                            "decoded_video_frames": data.get("member_frames"),
                            "exact_failures": 0,
                            "boxed_lcp_fallbacks": 0,
                            "contrast_heuristic_pass": False,
                            "contrast_required": False,
                            "pass": True,
                        },
                        item_location,
                        errors,
                    )
                    residual = _number(item.get("worst_residual"))
                    if residual is not None:
                        residuals.append(residual)
                if residuals and data.get("capture_max_residual") != max(residuals):
                    errors.append(
                        f"{location}.capture_max_residual: expected {max(residuals)}, "
                        f"got {data.get('capture_max_residual')}"
                    )
            group = _object(
                capture.get("group"),
                f"{location}.metadata.capture_summary.group",
                errors,
            )
            if group is not None:
                dimensions = data.get("group_dimensions")
                width = dimensions[0] if isinstance(dimensions, list) else None
                height = dimensions[1] if isinstance(dimensions, list) else None
                _expect_fields(
                    group,
                    {
                        "width": width,
                        "height": height,
                        "frame_rate": f"{data.get('group_video_fps')}/1",
                        "frame_count": data.get("group_video_frames"),
                        "full_decode": "pass",
                        "panel_sha256": hashes.get("groups/painleve/panel.png"),
                        "clip_sha256": hashes.get("groups/painleve/clip.mp4"),
                        "metadata_sha256": hashes.get("groups/painleve/metadata.json"),
                    },
                    f"{location}.metadata.capture_summary.group",
                    errors,
                )

    inspection = _read_bundle_json(
        bundle, "manual-inspection.json", f"{location}.manual_inspection", errors
    )
    if inspection is not None:
        _expect_fields(
            inspection,
            {
                "schema_version": "dart.fbf_painleve_proxy_manual_inspection/v1",
                "manual_inspected": True,
                "pass": True,
            },
            f"{location}.manual_inspection",
            errors,
        )
        verdicts = _object(
            inspection.get("verdicts"),
            f"{location}.manual_inspection.verdicts",
            errors,
        )
        if verdicts is not None:
            _expect_fields(
                verdicts,
                {
                    "paired_proxy_outcome_supported": True,
                    "mu05_upright_return_visible": True,
                    "mu055_tumble_and_visually_settled_horizontal_visible": True,
                    "shorter_travel_requires_physical_trace": True,
                    "paper_parity": False,
                    "external_solver_parity": False,
                    "approved_source_golden": False,
                    "timing_verdict": None,
                    "realtime_verdict": None,
                },
                f"{location}.manual_inspection.verdicts",
                errors,
            )

    invocations = _read_bundle_json(
        bundle, "invocations.json", f"{location}.invocations", errors
    )
    if invocations is not None:
        trace_invocations = invocations.get("trace")
        if not isinstance(trace_invocations, list) or len(trace_invocations) != 2:
            errors.append(f"{location}.invocations.trace: expected two invocations")
        else:
            expected_suffix = [
                "exact_fbf",
                "1",
                "150",
                "nan",
                "tracked",
                "default",
                "default",
                "1",
                "dart_best",
                "dart",
                "default",
                "0",
                "0",
                "default",
            ]
            for index, item in enumerate(trace_invocations):
                item_location = f"{location}.invocations.trace[{index}]"
                if not isinstance(item, dict):
                    errors.append(f"{item_location}: expected object")
                    continue
                scenario = item.get("scenario")
                argv = item.get("argv")
                if (
                    not isinstance(argv, list)
                    or len(argv) != 16
                    or argv[1] != scenario
                    or argv[2:] != expected_suffix
                ):
                    errors.append(
                        f"{item_location}.argv: explicit tracked contract changed"
                    )
                expected_trace_hash = hashes.get(f"traces/{scenario}.csv")
                _expect_fields(
                    item,
                    {"returncode": 0, "stdout_sha256": expected_trace_hash},
                    item_location,
                    errors,
                )
        verification_invocation = _object(
            invocations.get("capture_verification"),
            f"{location}.invocations.capture_verification",
            errors,
        )
        if verification_invocation is not None:
            _expect_fields(
                verification_invocation,
                {
                    "returncode": 0,
                    "stdout_path": "verification.json",
                    "stdout_sha256": hashes.get("verification.json"),
                },
                f"{location}.invocations.capture_verification",
                errors,
            )

    verification = _read_bundle_json(
        bundle, "verification.json", f"{location}.verification", errors
    )
    if verification is not None:
        _expect_fields(
            verification,
            {"kind": "verification", "pass": True},
            f"{location}.verification",
            errors,
        )
        results = verification.get("results")
        groups = verification.get("group_outputs")
        if not isinstance(results, list) or len(results) != 2:
            errors.append(f"{location}.verification.results: expected two")
        if not isinstance(groups, list) or len(groups) != 1:
            errors.append(f"{location}.verification.group_outputs: expected one")

    artifact_index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )
    if artifact_index is not None:
        _expect_fields(
            artifact_index,
            {
                "schema_version": "dart.fbf_painleve_proxy_artifact_index/v1",
                "artifact_count": data.get("artifact_count"),
                "excluded": ["artifact-index.json", "metadata.json"],
            },
            f"{location}.artifact_index",
            errors,
        )
        artifacts = artifact_index.get("artifacts")
        if not isinstance(artifacts, list) or len(artifacts) != data.get(
            "artifact_count"
        ):
            errors.append(
                f"{location}.artifact_index.artifacts: expected "
                f"{data.get('artifact_count')} artifacts"
            )
        elif bundle is not None:
            indexed_paths = [
                entry.get("path") for entry in artifacts if isinstance(entry, dict)
            ]
            if indexed_paths != sorted(set(indexed_paths)):
                errors.append(
                    f"{location}.artifact_index.artifacts: paths must be sorted/unique"
                )
            disk_paths = {
                path.relative_to(bundle).as_posix()
                for path in bundle.rglob("*")
                if path.is_file()
                and path.relative_to(bundle).as_posix()
                not in {"artifact-index.json", "metadata.json"}
            }
            if set(indexed_paths) != disk_paths:
                errors.append(
                    f"{location}.artifact_index.artifacts: expected exact disk "
                    "membership"
                )
            for index, entry in enumerate(artifacts):
                if not isinstance(entry, dict):
                    errors.append(
                        f"{location}.artifact_index.artifacts[{index}]: expected object"
                    )
                    continue
                relative = entry.get("path")
                if not isinstance(relative, str) or relative not in disk_paths:
                    continue
                path = bundle / relative
                if path.is_symlink():
                    errors.append(
                        f"{location}.artifact_index.artifacts[{index}]: symlink forbidden"
                    )
                    continue
                _expect_fields(
                    entry,
                    {"sha256": _sha256(path), "bytes": path.stat().st_size},
                    f"{location}.artifact_index.artifacts[{index}]",
                    errors,
                )


def _validate_visual_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.literal_wedge_visual_nonpaper"
    data = _object(current_truth.get("literal_wedge_visual_nonpaper"), location, errors)
    if data is None:
        return

    bundle = _validate_current_path(
        data,
        "bundle",
        VISUAL_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    _expect_fields(
        data,
        {
            "status": "valid_current_source_nonpaper_reconstruction",
            "paper_parity": False,
            "projectile_present": False,
            "steps": 600,
            "exact_attempts": 600,
            "exact_solves": 600,
            "exact_failures": 0,
            "boxed_lcp_fallbacks": 0,
            "trace_rows_compared": 600,
            "trace_mismatches": 0,
            "distinct_frames": 61,
            "frame_dimensions": [1280, 720],
            "video_codec": "H.264",
            "video_frames": 61,
            "video_fps": 10,
            "manual_inspected": True,
            "final_artifacts_valid": 19,
        },
        location,
        errors,
    )
    dimensions = data.get("frame_dimensions")
    if not isinstance(dimensions, list) or len(dimensions) != 2:
        dimensions = [None, None]
    residual = _number(data.get("max_residual"))
    if residual is None or not 0.0 <= residual <= PAPER_RESIDUAL_TOLERANCE:
        errors.append(
            f"{location}.max_residual: expected a finite value <= "
            f"{PAPER_RESIDUAL_TOLERANCE}"
        )
    acceleration = _number(data.get("playback_acceleration_factor"))
    if acceleration is None or acceleration <= 1.0:
        errors.append(
            f"{location}.playback_acceleration_factor: finalized visual must be "
            "labeled as time-lapse"
        )

    hashes = _validate_artifact_hashes(
        data,
        VISUAL_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        recorded_hash_keys={"current_trace_binary"},
    )
    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    if metadata is not None:
        _expect_fields(
            metadata,
            {
                "schema_version": "dart.fbf_literal_wedge_visual_bundle/v2",
                "status": "valid_current_source_nonpaper_reconstruction",
                "claim_valid": True,
                "paper_parity": False,
                "manual_inspected": True,
                "manual_inspection_bound": True,
                "manual_inspection_required": True,
            },
            f"{location}.metadata",
            errors,
        )
        for field, hash_key in (
            ("artifact_index_sha256", "artifact-index.json"),
            ("pending_metadata_sha256", "pending-metadata.json"),
            ("manual_inspection_sha256", "manual-inspection.json"),
            ("provenance_sha256", "provenance.json"),
            ("video_sha256", "fig07_literal_wedge_stability.mp4"),
            ("timeline_panel_sha256", "fig07_literal_wedge_timeline.png"),
            ("decoded_midpoint_sha256", "decoded/video_midpoint_t3.0.png"),
        ):
            if hash_key in hashes:
                _expect_fields(
                    metadata,
                    {field: hashes[hash_key]},
                    f"{location}.metadata",
                    errors,
                )
        timing = _object(metadata.get("timing"), f"{location}.metadata.timing", errors)
        if timing is not None:
            _expect_fields(
                timing,
                {
                    "captured_frames": 61,
                    "playback_acceleration_factor": data.get(
                        "playback_acceleration_factor"
                    ),
                    "playback_duration_seconds": data.get("playback_duration_seconds"),
                    "simulation_duration_seconds": data.get(
                        "simulation_duration_seconds"
                    ),
                },
                f"{location}.metadata.timing",
                errors,
            )
            meaning = timing.get("meaning")
            if not _nonempty_string(meaning) or not all(
                label in meaning.casefold() for label in ("time-lapse", "not real time")
            ):
                errors.append(
                    f"{location}.metadata.timing.meaning: explicit time-lapse and "
                    "not-real-time labeling is required"
                )

    provenance = _read_bundle_json(
        bundle, "provenance.json", f"{location}.provenance", errors
    )
    if provenance is not None:
        _expect_fields(
            provenance,
            {
                "schema_version": "dart.fbf_literal_wedge_visual_evidence/v2",
                "evidence_state": "valid_current_source_nonpaper_reconstruction",
            },
            f"{location}.provenance",
            errors,
        )
        source = _object(
            provenance.get("source"), f"{location}.provenance.source", errors
        )
        if source is not None and "current_trace_binary" in hashes:
            _expect_fields(
                source,
                {"trace_executable_sha256": hashes["current_trace_binary"]},
                f"{location}.provenance.source",
                errors,
            )
            continuity = _object(
                source.get("binary_and_raw_source_continuity"),
                f"{location}.provenance.source.binary_and_raw_source_continuity",
                errors,
            )
            if continuity is not None:
                _expect_fields(
                    continuity,
                    {"current_trace_executable_sha256": hashes["current_trace_binary"]},
                    (
                        f"{location}.provenance.source."
                        "binary_and_raw_source_continuity"
                    ),
                    errors,
                )
        finalization = _object(
            provenance.get("finalization"),
            f"{location}.provenance.finalization",
            errors,
        )
        if finalization is not None:
            _expect_fields(
                finalization,
                {
                    "claim_valid_after_finalization": True,
                    "paper_parity": False,
                },
                f"{location}.provenance.finalization",
                errors,
            )
        validation = _object(
            provenance.get("validation"),
            f"{location}.provenance.validation",
            errors,
        )
        if validation is not None:
            _expect_fields(
                validation,
                {
                    "automated_validation_pass": True,
                    "manual_inspection_bound": True,
                },
                f"{location}.provenance.validation",
                errors,
            )

    inspection = _read_bundle_json(
        bundle, "manual-inspection.json", f"{location}.manual_inspection", errors
    )
    if inspection is not None:
        _expect_fields(
            inspection,
            {
                "schema_version": "dart.fbf_literal_wedge_manual_inspection/v2",
                "manual_inspected": True,
                "pass": True,
            },
            f"{location}.manual_inspection",
            errors,
        )

    runtime = _read_bundle_json(
        bundle, "capture-runtime.json", f"{location}.runtime", errors
    )
    if runtime is not None:
        _expect_fields(
            runtime,
            {
                "steps_requested": data.get("steps"),
                "steps_completed": data.get("steps"),
                "width": dimensions[0],
                "height": dimensions[1],
            },
            f"{location}.runtime",
            errors,
        )
        outcome = _object(runtime.get("outcome"), f"{location}.runtime.outcome", errors)
        if outcome is not None:
            _expect_fields(
                outcome,
                {
                    "exact_attempts": data.get("exact_attempts"),
                    "exact_solves": data.get("exact_solves"),
                    "exact_failures": data.get("exact_failures"),
                    "boxed_lcp_fallbacks": data.get("boxed_lcp_fallbacks"),
                    "worst_exact_residual": data.get("max_residual"),
                },
                f"{location}.runtime.outcome",
                errors,
            )

    frame_validation = _read_bundle_json(
        bundle, "frame-validation.json", f"{location}.frame_validation", errors
    )
    if frame_validation is not None:
        _expect_fields(
            frame_validation,
            {
                "frame_count": data.get("distinct_frames"),
                "unique_pixel_frames": data.get("distinct_frames"),
                "pass": True,
            },
            f"{location}.frame_validation",
            errors,
        )

    video_probe = _read_bundle_json(
        bundle, "video-probe.json", f"{location}.video_probe", errors
    )
    if video_probe is not None:
        streams = video_probe.get("streams")
        if (
            not isinstance(streams, list)
            or len(streams) != 1
            or not isinstance(streams[0], dict)
        ):
            errors.append(f"{location}.video_probe.streams: expected one video stream")
        else:
            _expect_fields(
                streams[0],
                {
                    "codec_name": "h264",
                    "width": dimensions[0],
                    "height": dimensions[1],
                    "nb_read_frames": str(data.get("video_frames")),
                    "avg_frame_rate": f"{data.get('video_fps')}/1",
                },
                f"{location}.video_probe.streams[0]",
                errors,
            )
        _expect_fields(
            video_probe,
            {"pass": True},
            f"{location}.video_probe",
            errors,
        )

    equivalence = _read_bundle_json(
        bundle, "trace-equivalence.json", f"{location}.trace_equivalence", errors
    )
    if equivalence is not None:
        _expect_fields(
            equivalence,
            {
                "pass": True,
                "rows_compared": data.get("trace_rows_compared"),
                "mismatches": [],
            },
            f"{location}.trace_equivalence",
            errors,
        )
        reference_int = _object(
            equivalence.get("expected_reference_int"),
            f"{location}.trace_equivalence.expected_reference_int",
            errors,
        )
        if reference_int is not None:
            _expect_fields(
                reference_int,
                {
                    "contacts": data.get("contacts_each_step"),
                    "unique_colliding_body_pairs": data.get(
                        "colliding_body_pairs_each_step"
                    ),
                },
                f"{location}.trace_equivalence.expected_reference_int",
                errors,
            )

    artifact_index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )
    if artifact_index is not None:
        _expect_fields(
            artifact_index,
            {
                "schema_version": "dart.fbf_literal_wedge_artifact_index/v2",
                "artifact_count": 19,
                "excluded": ["artifact-index.json", "metadata.json"],
            },
            f"{location}.artifact_index",
            errors,
        )
        artifacts = artifact_index.get("artifacts")
        if not isinstance(artifacts, list) or len(artifacts) != 19:
            errors.append(
                f"{location}.artifact_index.artifacts: expected 19 finalized artifacts"
            )
        else:
            indexed_paths = {
                entry.get("path")
                for entry in artifacts
                if isinstance(entry, dict) and isinstance(entry.get("path"), str)
            }
            if len(indexed_paths) != 19:
                errors.append(
                    f"{location}.artifact_index.artifacts: paths must be unique strings"
                )
            if bundle is not None:
                disk_paths = {
                    path.relative_to(bundle).as_posix()
                    for path in bundle.rglob("*")
                    if path.is_file()
                    and path.relative_to(bundle).as_posix()
                    not in {"artifact-index.json", "metadata.json"}
                }
                if indexed_paths != disk_paths:
                    errors.append(
                        f"{location}.artifact_index.artifacts: expected exact disk "
                        f"membership {sorted(disk_paths)}, got {sorted(indexed_paths)}"
                    )
                for index, entry in enumerate(artifacts):
                    if not isinstance(entry, dict):
                        errors.append(
                            f"{location}.artifact_index.artifacts[{index}]: "
                            "expected an object"
                        )
                        continue
                    relative = entry.get("path")
                    if not isinstance(relative, str) or relative not in disk_paths:
                        continue
                    path = bundle / relative
                    _expect_fields(
                        entry,
                        {"sha256": _sha256(path), "bytes": path.stat().st_size},
                        f"{location}.artifact_index.artifacts[{index}]",
                        errors,
                    )


def _validate_impact_v7_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.literal_wedge_crown_impact_v1_nonpaper"
    data = _object(
        current_truth.get("literal_wedge_crown_impact_v1_nonpaper"), location, errors
    )
    if data is None:
        return

    bundle = _validate_current_path(
        data,
        "bundle",
        IMPACT_V7_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    _validate_current_path(
        data, "runner", IMPACT_V7_RUNNER, location, repo_root, errors
    )
    _expect_fields(
        data,
        {
            "status": "valid_scientific_negative",
            "impact_claim_passed": False,
            "paper_parity": False,
            "steps_completed": 720,
            "standing_prefix_steps": 600,
            "standing_prefix_mismatches": 0,
            "projectile_count": 3,
            "first_projectile_arch_contact_step": 607,
            "first_projectile_ground_contact_step": 616,
            "finite_state": True,
            "exact_failures": 0,
            "boxed_lcp_fallbacks": 0,
            "accepted_caps": 5,
            "failed_preregistered_gates": [
                "accepted caps must equal zero",
                "worst exact residual must be at most 1e-6",
                "final maximum arch displacement must be at most 0.07 m",
                "final far-field displacement must be at most 0.007 m",
            ],
        },
        location,
        errors,
    )
    failed_gates = data.get("failed_preregistered_gates")
    if not isinstance(failed_gates, list) or not failed_gates:
        errors.append(
            f"{location}.failed_preregistered_gates: negative evidence requires "
            "failed gates"
        )

    hashes = _validate_artifact_hashes(
        data,
        IMPACT_V7_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        recorded_hash_keys={"trace_binary"},
    )
    summary = _read_bundle_json(bundle, "summary.json", f"{location}.summary", errors)
    if summary is not None:
        _expect_fields(
            summary,
            {
                "schema_version": "dart.fbf_literal_crown_impact_negative/v1",
                "artifact_valid": True,
                "classification": "valid_scientific_negative",
                "expected_fail_closed_process_observed": True,
                "impact_claim_passed": False,
            },
            f"{location}.summary",
            errors,
        )
        summary_failed_gates = summary.get("failed_gates")
        if not isinstance(summary_failed_gates, list) or not summary_failed_gates:
            errors.append(
                f"{location}.summary.failed_gates: negative evidence requires "
                "failed gates"
            )
        if _nonempty_string(data.get("normalized_fingerprint_sha256")):
            _expect_fields(
                summary,
                {"normalized_trace_fingerprint": data["normalized_fingerprint_sha256"]},
                f"{location}.summary",
                errors,
            )
        _expect_fields(
            summary,
            {
                "trace_rows": data.get("steps_completed"),
                "failed_gates": [
                    "impact_exact_gate",
                    "impact_residual_gate",
                    "final_all_body_displacement_gate",
                    "final_far_field_displacement_gate",
                    "final_impact_acceptance_gate",
                ],
            },
            f"{location}.summary",
            errors,
        )
        contact_order = _object(
            summary.get("contact_order"), f"{location}.summary.contact_order", errors
        )
        if contact_order is not None:
            _expect_fields(
                contact_order,
                {
                    "first_projectile_arch_contact_step": data.get(
                        "first_projectile_arch_contact_step"
                    ),
                    "first_projectile_ground_contact_step": data.get(
                        "first_projectile_ground_contact_step"
                    ),
                    "gate_passed": True,
                },
                f"{location}.summary.contact_order",
                errors,
            )
        comparison = _object(
            summary.get("standing_prefix_comparison"),
            f"{location}.summary.standing_prefix_comparison",
            errors,
        )
        if comparison is not None:
            _expect_fields(
                comparison,
                {
                    "steps_compared": data.get("standing_prefix_steps"),
                    "fields_compared": data.get("standing_prefix_fields_compared"),
                    "mismatches": data.get("standing_prefix_mismatches"),
                    "pass": True,
                },
                f"{location}.summary.standing_prefix_comparison",
                errors,
            )
            _expect_fields(
                data,
                {
                    "standing_reference_fields": comparison.get("fields_compared"),
                    "standing_reference_mismatches": comparison.get("mismatches"),
                },
                location,
                errors,
            )
        preservation = _object(
            summary.get("preservation"), f"{location}.summary.preservation", errors
        )
        if preservation is not None:
            _expect_fields(
                preservation,
                {
                    "final_maximum_arch_displacement": data.get(
                        "final_max_arch_displacement_m"
                    ),
                    "final_maximum_far_field_displacement": data.get(
                        "final_max_far_field_displacement_m"
                    ),
                },
                f"{location}.summary.preservation",
                errors,
            )

    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    if metadata is not None:
        expected_identity_rechecks = [
            "after_impact_trace",
            "after_standing_reference_trace",
        ]
        runtime_scope = (
            "Recorded the executed trace binary, ldd tool, every resolved regular "
            "shared-library file, and exactly one build-tree libdart with resolved "
            "paths, sizes, and SHA-256 digests. When CPU affinity was requested, "
            "the exact resolved taskset executable was used and recorded with its "
            "size and SHA-256 digest. The full closure was unchanged after both "
            "child traces."
        )
        _expect_fields(
            metadata,
            {
                "schema_version": "dart.fbf_literal_crown_impact_negative/v1",
                "classification": "valid_scientific_negative",
                "impact_claim_passed": False,
                "runner_source": IMPACT_V7_RUNNER,
                "identity_rechecks": expected_identity_rechecks,
                "runtime_provenance_scope": runtime_scope,
            },
            f"{location}.metadata",
            errors,
        )
        runtime_identity = _validate_structured_executable_identity(
            metadata.get("runtime_identity"),
            f"{location}.metadata.runtime_identity",
            repo_root,
            errors,
            expected_sha256=hashes.get("trace_binary"),
        )
        selected_cpu_value = metadata.get("selected_cpu")
        selected_cpu: int | None
        if selected_cpu_value is None:
            selected_cpu = None
        else:
            selected_cpu = _integer(selected_cpu_value)
            if selected_cpu is None or selected_cpu < 0:
                errors.append(
                    f"{location}.metadata.selected_cpu: expected null or a "
                    "non-negative integer"
                )
                selected_cpu = None
        executed_tool_closure = _object(
            metadata.get("executed_tool_closure"),
            f"{location}.metadata.executed_tool_closure",
            errors,
        )
        taskset_tool: dict[str, Any] | None = None
        if executed_tool_closure is not None:
            expected_tool_keys = (
                {"taskset"} if selected_cpu_value is not None else set()
            )
            if set(executed_tool_closure) != expected_tool_keys:
                errors.append(
                    f"{location}.metadata.executed_tool_closure: exact executed-"
                    "tool membership changed"
                )
            if selected_cpu is not None:
                taskset_tool = _validate_tool_identity(
                    executed_tool_closure.get("taskset"),
                    "taskset",
                    f"{location}.metadata.executed_tool_closure.taskset",
                    errors,
                )
        bindings = {
            "runner_source_sha256": hashes.get("runner"),
            "trace_source_sha256": hashes.get("trace_source"),
            "binary_sha256": hashes.get("trace_binary"),
            "raw_csv_sha256": hashes.get("raw.csv"),
            "stderr_sha256": hashes.get("stderr.txt"),
            "standing_reference_csv_sha256": hashes.get("standing-reference.csv"),
            "standing_reference_stderr_sha256": hashes.get(
                "standing-reference.stderr.txt"
            ),
            "preregistration_contract_sha256": data.get(
                "preregistration_contract_sha256"
            ),
        }
        _expect_fields(
            metadata,
            {key: value for key, value in bindings.items() if value is not None},
            f"{location}.metadata",
            errors,
        )
        if runtime_identity is not None:
            _expect_fields(
                metadata,
                {
                    "binary": runtime_identity.get("path"),
                    "binary_size_bytes": runtime_identity.get("size_bytes"),
                    "binary_sha256": runtime_identity.get("sha256"),
                    "trace_source": TRACE_SOURCE,
                    "runner_source": IMPACT_V7_RUNNER,
                },
                f"{location}.metadata",
                errors,
            )
            prefix: list[Any] = []
            if selected_cpu is not None and taskset_tool is not None:
                prefix = [
                    taskset_tool.get("resolved_path"),
                    "-c",
                    str(selected_cpu),
                ]
            _expect_exact_payload(
                metadata.get("command"),
                prefix
                + [
                    runtime_identity.get("path"),
                    "masonry_arch_25_literal_wedge_crown_impact_v1",
                    "exact_fbf",
                    "1",
                    "720",
                    "nan",
                    "performance",
                    "default",
                    "default",
                    "1",
                    "dart_best_colored_bgs",
                    "native",
                    "default",
                    "0",
                    "0",
                ],
                f"{location}.metadata.command",
                errors,
            )
            _expect_exact_payload(
                metadata.get("standing_reference_command"),
                prefix
                + [
                    runtime_identity.get("path"),
                    "masonry_arch_25_literal_wedge",
                    "exact_fbf",
                    "1",
                    "600",
                    "nan",
                    "performance",
                    "default",
                    "default",
                    "1",
                    "dart_best_colored_bgs",
                    "native",
                    "default",
                    "0",
                    "0",
                ],
                f"{location}.metadata.standing_reference_command",
                errors,
            )
        runtime = _object(
            data.get("runtime_provenance"),
            f"{location}.runtime_provenance",
            errors,
        )
        if runtime is not None and runtime_identity is not None:
            libdart = runtime_identity.get("resolved_build_libdart")
            ldd_tool = runtime_identity.get("ldd_tool")
            expected_runtime = {
                "scope": runtime_scope,
                "identity_rechecks": metadata.get("identity_rechecks"),
                "selected_cpu": selected_cpu_value,
                "trace_ldd_map_sha256": runtime_identity.get(
                    "ldd_resolution_output_normalized_sha256"
                ),
                "resolved_build_libdart_sha256": (
                    libdart.get("sha256") if isinstance(libdart, dict) else None
                ),
                "ldd_tool_sha256": (
                    ldd_tool.get("sha256") if isinstance(ldd_tool, dict) else None
                ),
                "trace_resolved_regular_shared_libraries": (
                    runtime_identity.get("resolved_regular_shared_library_count")
                ),
            }
            if taskset_tool is not None:
                expected_runtime["taskset_tool_sha256"] = taskset_tool.get("sha256")
            _expect_exact_payload(
                runtime,
                expected_runtime,
                f"{location}.runtime_provenance",
                errors,
            )

    if bundle is not None:
        raw = _read_csv(bundle / "raw.csv", f"{location}.raw", errors)
        if raw is not None:
            _, rows = raw
            if len(rows) != 720:
                errors.append(f"{location}.raw: expected 720 rows, got {len(rows)}")
            elif rows:
                final = rows[-1]
                for field, truth_field in (
                    ("impact_projectile_count", "projectile_count"),
                    ("exact_failures_to_date", "exact_failures"),
                    ("boxed_fallbacks_to_date", "boxed_lcp_fallbacks"),
                    ("max_iterations_accepted_to_date", "accepted_caps"),
                    (
                        "first_projectile_arch_contact_step",
                        "first_projectile_arch_contact_step",
                    ),
                    (
                        "first_projectile_ground_contact_step",
                        "first_projectile_ground_contact_step",
                    ),
                ):
                    value = _parse_csv_int(
                        final.get(field), f"{location}.raw.final.{field}", errors
                    )
                    if value is not None and value != data.get(truth_field):
                        errors.append(
                            f"{location}.raw.final.{field}: expected "
                            f"{data.get(truth_field)!r}, got {value!r}"
                        )
                for field, truth_field in (
                    ("worst_exact_residual_to_date", "worst_exact_residual"),
                    (
                        "max_arch_body_displacement_from_preimpact",
                        "final_max_arch_displacement_m",
                    ),
                    (
                        "max_far_field_displacement_from_preimpact",
                        "final_max_far_field_displacement_m",
                    ),
                ):
                    value = _parse_csv_number(
                        final.get(field), f"{location}.raw.final.{field}", errors
                    )
                    expected = _number(data.get(truth_field))
                    if (
                        value is not None
                        and expected is not None
                        and not math.isclose(
                            value, expected, rel_tol=1e-12, abs_tol=1e-15
                        )
                    ):
                        errors.append(
                            f"{location}.raw.final.{field}: expected {expected}, "
                            f"got {value}"
                        )
                if final.get("finite_state_to_date") != "1":
                    errors.append(
                        f"{location}.raw.final.finite_state_to_date: expected '1'"
                    )


def _validate_arch101_v5_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.literal_arch_101_v1_nonpaper"
    data = _object(current_truth.get("literal_arch_101_v1_nonpaper"), location, errors)
    if data is None:
        return

    bundle = _validate_current_path(
        data,
        "bundle",
        ARCH101_V5_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    protocol = _validate_current_path(
        data,
        "protocol",
        ARCH101_V1_PROTOCOL,
        location,
        repo_root,
        errors,
    )
    _validate_current_path(
        data, "runner", ARCH101_V1_RUNNER, location, repo_root, errors
    )
    _expect_fields(
        data,
        {
            "status": "failed_prefix_scientific_negative",
            "artifact_valid": False,
            "standing_claim_passed": False,
            "timing_evidence_eligible": False,
            "requested_steps": 600,
            "emitted_steps": 1,
            "first_failed_step": 1,
            "contacts": 400,
            "colliding_body_pairs": 100,
            "outer_iterations": 5000,
            "terminal_status": "fbf_failed",
            "exact_failures": 1,
            "boxed_lcp_fallbacks": 0,
        },
        location,
        errors,
    )
    protocol_hash = _contract_sha256(
        protocol, "## Frozen v1 result", f"{location}.protocol", errors
    )
    if protocol_hash is not None:
        _expect_fields(
            data,
            {"protocol_contract_sha256": protocol_hash},
            location,
            errors,
        )
    hashes = _validate_artifact_hashes(
        data,
        ARCH101_V5_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
        recorded_hash_keys={"trace_binary", "collision_probe_binary"},
    )

    invocation = _read_bundle_json(
        bundle, "invocation.json", f"{location}.invocation", errors
    )
    expected_identity_rechecks = ["after_collision_probe", "after_trace"]
    if invocation is not None:
        _expect_fields(
            invocation,
            {"identity_rechecks": expected_identity_rechecks},
            f"{location}.invocation",
            errors,
        )

    summary = _read_bundle_json(bundle, "summary.json", f"{location}.summary", errors)
    if summary is not None:
        _expect_fields(
            summary,
            {
                "schema_version": "dart.fbf_literal_arch101_v1/v1",
                "artifact_valid": False,
                "classification": "failed_prefix_scientific_negative",
                "standing_claim_passed": False,
                "timing_evidence_eligible": False,
                "timing_statistics": None,
                "positive_long_run_promotion_eligible": False,
                "requested_steps": 600,
                "emitted_steps": 1,
                "first_failed_step": 1,
            },
            f"{location}.summary",
            errors,
        )
        if protocol_hash is not None:
            _expect_fields(
                summary,
                {"protocol_contract_sha256": protocol_hash},
                f"{location}.summary",
                errors,
            )
        if _nonempty_string(data.get("normalized_fingerprint_sha256")):
            _expect_fields(
                summary,
                {
                    "normalized_scene_work_fingerprint_sha256": data[
                        "normalized_fingerprint_sha256"
                    ]
                },
                f"{location}.summary",
                errors,
            )
        _expect_fields(
            summary,
            {
                "terminal_status": data.get("terminal_status"),
                "terminal_residual": data.get("terminal_residual"),
                "dynamic_pair_identity_evidence": False,
            },
            f"{location}.summary",
            errors,
        )
        summary_probe = _object(
            summary.get("collision_probe"),
            f"{location}.summary.collision_probe",
            errors,
        )
        truth_probe = _object(
            data.get("collision_probe"), f"{location}.collision_probe", errors
        )
        if summary_probe is not None and truth_probe is not None:
            _expect_fields(
                summary_probe,
                truth_probe,
                f"{location}.summary.collision_probe",
                errors,
            )

    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    if metadata is not None:
        _expect_fields(
            metadata,
            {
                "schema_version": "dart.fbf_literal_arch101_v1/v1",
                "classification": "failed_prefix_scientific_negative",
                "artifact_valid": False,
                "standing_claim_passed": False,
                "timing_evidence_eligible": False,
                "positive_long_run_promotion_eligible": False,
                "protocol": ARCH101_V1_PROTOCOL,
                "runner_source": ARCH101_V1_RUNNER,
                "runtime_provenance_scope": (
                    "Recorded executable, ldd, taskset, and resolved regular "
                    "shared-library file identities; this is not a claim over "
                    "all host runtime state"
                ),
            },
            f"{location}.metadata",
            errors,
        )
        if _nonempty_string(data.get("normalized_fingerprint_sha256")):
            _expect_fields(
                metadata,
                {
                    "normalized_scene_work_fingerprint_sha256": data[
                        "normalized_fingerprint_sha256"
                    ]
                },
                f"{location}.metadata",
                errors,
            )
        source_identity = _object(
            metadata.get("source_identity"),
            f"{location}.metadata.source_identity",
            errors,
        )
        if source_identity is not None:
            bindings = {
                "protocol_contract_sha256": protocol_hash,
                "runner_source_sha256": hashes.get("runner"),
                "trace_source_sha256": hashes.get("trace_source"),
                "collision_probe_source_sha256": hashes.get("collision_probe_source"),
            }
            _expect_fields(
                source_identity,
                {key: value for key, value in bindings.items() if value is not None},
                f"{location}.metadata.source_identity",
                errors,
            )
            trace_executable = _validate_structured_executable_identity(
                source_identity.get("trace_executable"),
                f"{location}.metadata.source_identity.trace_executable",
                repo_root,
                errors,
                expected_sha256=hashes.get("trace_binary"),
            )
            collision_executable = _validate_structured_executable_identity(
                source_identity.get("collision_probe_executable"),
                f"{location}.metadata.source_identity.collision_probe_executable",
                repo_root,
                errors,
                expected_sha256=hashes.get("collision_probe_binary"),
            )
            taskset_tool = _validate_tool_identity(
                source_identity.get("taskset_tool"),
                "taskset",
                f"{location}.metadata.source_identity.taskset_tool",
                errors,
            )
            if (
                trace_executable is not None
                and collision_executable is not None
                and taskset_tool is not None
            ):
                _expect_fields(
                    metadata,
                    {
                        "binary": trace_executable.get("path"),
                        "trace_source": TRACE_SOURCE,
                        "collision_probe_binary": collision_executable.get("path"),
                        "collision_probe_source": COLLISION_PROBE_SOURCE,
                        "taskset": {
                            "path": taskset_tool.get("path"),
                            "sha256": taskset_tool.get("sha256"),
                        },
                    },
                    f"{location}.metadata",
                    errors,
                )
                _expect_exact_payload(
                    collision_executable.get("ldd_tool"),
                    trace_executable.get("ldd_tool"),
                    f"{location}.metadata.source_identity."
                    "collision_probe_executable.ldd_tool",
                    errors,
                )
                _expect_exact_payload(
                    collision_executable.get("resolved_build_libdart"),
                    trace_executable.get("resolved_build_libdart"),
                    f"{location}.metadata.source_identity."
                    "collision_probe_executable.resolved_build_libdart",
                    errors,
                )
            if (
                invocation is not None
                and trace_executable is not None
                and collision_executable is not None
                and taskset_tool is not None
            ):
                command = invocation.get("command")
                if not isinstance(command, list) or len(command) != 18:
                    errors.append(
                        f"{location}.invocation.command: expected 18 arguments"
                    )
                else:
                    _validate_recorded_tool_command(
                        command[0],
                        taskset_tool,
                        f"{location}.invocation.command[0]",
                        errors,
                    )
                    _expect_exact_payload(
                        command[1:],
                        [
                            "--cpu-list",
                            "8,10,12,14",
                            trace_executable.get("path"),
                            "masonry_arch_101_literal_wedge",
                            "exact_fbf",
                            "1",
                            "600",
                            "nan",
                            "performance",
                            "default",
                            "default",
                            "4",
                            "dart_best_colored_bgs",
                            "native",
                            "default",
                            "0",
                            "0",
                        ],
                        f"{location}.invocation.command[1:]",
                        errors,
                    )
                _expect_exact_payload(
                    invocation.get("collision_probe_command"),
                    [collision_executable.get("path"), "2"],
                    f"{location}.invocation.collision_probe_command",
                    errors,
                )
            runtime = _object(
                data.get("runtime_provenance"),
                f"{location}.runtime_provenance",
                errors,
            )
            if runtime is not None and set(runtime) != {
                "scope",
                "identity_rechecks",
                "trace_ldd_map_sha256",
                "collision_probe_ldd_map_sha256",
                "resolved_build_libdart_sha256",
                "ldd_tool_sha256",
                "taskset_tool_sha256",
                "trace_resolved_regular_shared_libraries",
                "collision_probe_resolved_regular_shared_libraries",
            }:
                errors.append(
                    f"{location}.runtime_provenance: exact runtime-provenance "
                    "contract changed"
                )
            if runtime is not None:
                expected_scope = (
                    "Executable, ldd, taskset, and resolved regular shared-library "
                    "file identities were recorded and rechecked after the collision "
                    "probe and trace; this is not a claim over all host runtime state."
                )
                _expect_fields(
                    runtime,
                    {
                        "scope": expected_scope,
                        "identity_rechecks": (
                            invocation.get("identity_rechecks")
                            if invocation is not None
                            else expected_identity_rechecks
                        ),
                    },
                    f"{location}.runtime_provenance",
                    errors,
                )
                if (
                    isinstance(trace_executable, dict)
                    and isinstance(collision_executable, dict)
                    and isinstance(taskset_tool, dict)
                ):
                    trace_libdart = trace_executable.get("resolved_build_libdart")
                    _expect_fields(
                        runtime,
                        {
                            "trace_ldd_map_sha256": trace_executable.get(
                                "ldd_resolution_output_normalized_sha256"
                            ),
                            "collision_probe_ldd_map_sha256": collision_executable.get(
                                "ldd_resolution_output_normalized_sha256"
                            ),
                            "resolved_build_libdart_sha256": (
                                trace_libdart.get("sha256")
                                if isinstance(trace_libdart, dict)
                                else None
                            ),
                            "ldd_tool_sha256": (
                                trace_executable.get("ldd_tool", {}).get("sha256")
                                if isinstance(trace_executable.get("ldd_tool"), dict)
                                else None
                            ),
                            "taskset_tool_sha256": taskset_tool.get("sha256"),
                            "trace_resolved_regular_shared_libraries": (
                                trace_executable.get(
                                    "resolved_regular_shared_library_count"
                                )
                            ),
                            "collision_probe_resolved_regular_shared_libraries": (
                                collision_executable.get(
                                    "resolved_regular_shared_library_count"
                                )
                            ),
                        },
                        f"{location}.runtime_provenance",
                        errors,
                    )
        artifact_sha256 = _object(
            metadata.get("artifact_sha256"),
            f"{location}.metadata.artifact_sha256",
            errors,
        )
        if artifact_sha256 is not None:
            for key in (
                "raw.csv",
                "stderr.txt",
                "collision_probe_stdout.txt",
                "collision_probe_stderr.txt",
                "invocation.json",
                "summary.json",
                "REPORT.md",
            ):
                if key in hashes:
                    _expect_fields(
                        artifact_sha256,
                        {key: hashes[key]},
                        f"{location}.metadata.artifact_sha256",
                        errors,
                    )

    if bundle is not None:
        raw = _read_csv(bundle / "raw.csv", f"{location}.raw", errors)
        if raw is not None:
            _, rows = raw
            if len(rows) != 1:
                errors.append(
                    f"{location}.raw: expected one failed row, got {len(rows)}"
                )
            elif rows:
                row = rows[0]
                schedule = data.get("colored_schedule")
                schedule = schedule if isinstance(schedule, dict) else {}
                for field, truth_value in (
                    ("step", data.get("first_failed_step")),
                    ("contacts", data.get("contacts")),
                    (
                        "unique_colliding_body_pairs",
                        data.get("colliding_body_pairs"),
                    ),
                    ("last_exact_colored_bgs_manifolds", schedule.get("manifolds")),
                    ("last_exact_colored_bgs_colors", schedule.get("colors")),
                    (
                        "last_exact_colored_bgs_max_manifolds_per_color",
                        schedule.get("max_manifolds_per_color"),
                    ),
                    (
                        "last_exact_colored_bgs_max_participants",
                        schedule.get("participants"),
                    ),
                    ("max_outer_iterations", data.get("outer_iterations")),
                    ("step_exact_failures", data.get("exact_failures")),
                    ("step_fallbacks", data.get("boxed_lcp_fallbacks")),
                ):
                    value = _parse_csv_int(
                        row.get(field), f"{location}.raw[0].{field}", errors
                    )
                    if value is not None and value != truth_value:
                        errors.append(
                            f"{location}.raw[0].{field}: expected {truth_value!r}, "
                            f"got {value!r}"
                        )
                for field, truth_field in (
                    ("residual", "terminal_residual"),
                    (
                        "max_arch_body_displacement_from_initial",
                        "max_arch_body_displacement_m",
                    ),
                    (
                        "min_arch_body_orientation_alignment_from_initial",
                        "min_arch_body_orientation_alignment",
                    ),
                ):
                    value = _parse_csv_number(
                        row.get(field), f"{location}.raw[0].{field}", errors
                    )
                    expected = _number(data.get(truth_field))
                    if (
                        value is not None
                        and expected is not None
                        and not math.isclose(
                            value, expected, rel_tol=1e-12, abs_tol=1e-15
                        )
                    ):
                        errors.append(
                            f"{location}.raw[0].{field}: expected {expected}, got {value}"
                        )


def _turntable_v1_expected_bundle_paths() -> set[str]:
    paths = {
        "run-summary.json",
        "capture-provenance.json",
        "manual-inspection.json",
        "groups/turntable_author/metadata.json",
        "groups/turntable_author/panel.png",
        "groups/turntable_author/clip.mp4",
        "groups/turntable_author/panel_top.png",
        "groups/turntable_author/panel_bottom.png",
        "groups/turntable_author/panel_row_0.compose.json",
        "groups/turntable_author/panel_row_1.compose.json",
        "trace-summary.json",
        "verification.json",
        "invocations.json",
        "REPORT.md",
        "artifact-index.json",
        "metadata.json",
    }
    for capture_id, outcome_step in zip(
        TURNTABLE_V1_CAPTURE_IDS, TURNTABLE_V1_GROUP_PANEL_STEPS
    ):
        paths.update(
            {
                f"{capture_id}/metadata.json",
                f"{capture_id}/timeline.json",
                f"{capture_id}/panel.png",
                f"{capture_id}/panel.compose.json",
                f"{capture_id}/clip.mp4",
                f"{capture_id}/stills/outcome_step_{outcome_step:06d}.png",
                f"groups/turntable_author/panel_members/{capture_id}.png",
            }
        )
    for lane in TURNTABLE_V1_LANES:
        for scenario in TURNTABLE_V1_SCENARIOS:
            paths.add(f"traces/{lane}/{scenario}.csv")
            paths.add(f"traces/{lane}/{scenario}.stderr.txt")
    return paths


def _validate_turntable_v1_visual_resources(
    obj_text: str, mtl_text: str
) -> dict[str, Any]:
    """Validate that the segmented disc is a renderer-only unit cylinder."""

    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[str, tuple[int, ...]]] = []
    material_switches: list[str] = []
    mtllibs: list[str] = []
    object_names: list[str] = []
    active_material: str | None = None
    for line_number, raw in enumerate(obj_text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        command = fields[0]
        if command not in {"mtllib", "o", "v", "usemtl", "f"}:
            raise ValueError(
                f"turntable OBJ line {line_number}: unsupported command {command!r}"
            )
        if command == "mtllib":
            if len(fields) != 2:
                raise ValueError("turntable OBJ mtllib changed")
            mtllibs.append(fields[1])
        elif command == "o":
            if len(fields) != 2:
                raise ValueError("turntable OBJ object changed")
            object_names.append(fields[1])
        elif command == "v":
            if len(fields) != 4:
                raise ValueError("turntable OBJ vertex changed")
            try:
                vertex = tuple(float(value) for value in fields[1:])
            except (ValueError, OverflowError) as error:
                raise ValueError("turntable OBJ vertex changed") from error
            if not all(math.isfinite(value) for value in vertex):
                raise ValueError("turntable OBJ vertex is non-finite")
            vertices.append(vertex)
        elif command == "usemtl":
            if len(fields) != 2:
                raise ValueError("turntable OBJ material changed")
            active_material = fields[1]
            material_switches.append(active_material)
        else:
            if active_material is None or len(fields) not in (4, 5):
                raise ValueError("turntable OBJ face changed")
            indices: list[int] = []
            for token in fields[1:]:
                if "/" in token:
                    raise ValueError("turntable OBJ texture/normal indices appeared")
                try:
                    index = int(token)
                except (ValueError, OverflowError) as error:
                    raise ValueError("turntable OBJ index changed") from error
                if index <= 0 or str(index) != token:
                    raise ValueError("turntable OBJ index changed")
                indices.append(index)
            faces.append((active_material, tuple(indices)))

    top_materials = (
        "FbfTurntableCoral",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
    )
    if mtllibs != ["fbf_author_turntable_disc.mtl"] or object_names != [
        "FbfAuthorTurntableDisc"
    ]:
        raise ValueError("turntable OBJ resource linkage changed")
    if len(vertices) != 130 or len(faces) != 192:
        raise ValueError("turntable OBJ geometry counts changed")
    if tuple(material_switches) != top_materials + (
        "FbfTurntableSide",
        "FbfTurntableBottom",
    ):
        raise ValueError("turntable OBJ material sector order changed")

    expected_vertices = [(0.0, 0.0, 0.5)]
    expected_vertices.extend(
        (
            math.cos(2.0 * math.pi * index / 64.0),
            math.sin(2.0 * math.pi * index / 64.0),
            0.5,
        )
        for index in range(64)
    )
    expected_vertices.append((0.0, 0.0, -0.5))
    expected_vertices.extend(
        (
            math.cos(2.0 * math.pi * index / 64.0),
            math.sin(2.0 * math.pi * index / 64.0),
            -0.5,
        )
        for index in range(64)
    )
    if any(
        any(
            not math.isclose(actual, expected, rel_tol=0.0, abs_tol=5.0e-10)
            for actual, expected in zip(vertex, expected_vertex)
        )
        for vertex, expected_vertex in zip(vertices, expected_vertices)
    ):
        raise ValueError("turntable OBJ unit-cylinder vertices changed")

    expected_faces: list[tuple[str, tuple[int, ...]]] = []
    expected_faces.extend(
        (
            top_materials[index // 8],
            (1, 2 + index, 2 + ((index + 1) % 64)),
        )
        for index in range(64)
    )
    expected_faces.extend(
        (
            "FbfTurntableSide",
            (
                2 + index,
                67 + index,
                67 + ((index + 1) % 64),
                2 + ((index + 1) % 64),
            ),
        )
        for index in range(64)
    )
    expected_faces.extend(
        (
            "FbfTurntableBottom",
            (66, 67 + ((index + 1) % 64), 67 + index),
        )
        for index in range(64)
    )
    if faces != expected_faces:
        raise ValueError("turntable OBJ segmented faces changed")

    materials: dict[str, dict[str, tuple[float, ...] | float]] = {}
    active_name: str | None = None
    allowed_fields = {"Ka", "Kd", "Ks", "Ke", "Ns", "d", "illum"}
    for line_number, raw in enumerate(mtl_text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if fields[0] == "newmtl":
            if len(fields) != 2 or fields[1] in materials:
                raise ValueError("turntable MTL material set changed")
            active_name = fields[1]
            materials[active_name] = {}
            continue
        if active_name is None or fields[0] not in allowed_fields:
            raise ValueError(f"turntable MTL line {line_number}: field changed")
        field = fields[0]
        if field in materials[active_name]:
            raise ValueError("turntable MTL duplicate field")
        count = 3 if field in {"Ka", "Kd", "Ks", "Ke"} else 1
        if len(fields) != count + 1:
            raise ValueError("turntable MTL value count changed")
        try:
            values = tuple(float(value) for value in fields[1:])
        except (ValueError, OverflowError) as error:
            raise ValueError("turntable MTL value changed") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError("turntable MTL value is non-finite")
        materials[active_name][field] = values if count == 3 else values[0]

    diffuse = {
        "FbfTurntableCoral": (0.94, 0.31, 0.27),
        "FbfTurntableLight": (0.76, 0.78, 0.80),
        "FbfTurntableDark": (0.39, 0.42, 0.45),
        "FbfTurntableSide": (0.50, 0.52, 0.54),
        "FbfTurntableBottom": (0.32, 0.34, 0.36),
    }
    if set(materials) != set(diffuse) or any(
        set(properties) != allowed_fields for properties in materials.values()
    ):
        raise ValueError("turntable MTL material set changed")
    if any(materials[name]["Kd"] != color for name, color in diffuse.items()):
        raise ValueError("turntable MTL diffuse colors changed")
    return {
        "unit_geometry": "radius_1_height_1_cylinder",
        "vertex_count": 130,
        "face_count": 192,
        "top_wedge_count": 8,
        "top_triangles_per_wedge": 8,
        "top_material_order": list(top_materials),
        "registration_material": "FbfTurntableCoral",
        "material_diffuse_rgb": {name: list(color) for name, color in diffuse.items()},
    }


def _turntable_v1_trace_metrics(
    parsed: tuple[list[str], list[dict[str, str]]] | None,
    scenario: str,
    lane: str,
    location: str,
    errors: list[str],
) -> dict[str, Any] | None:
    if parsed is None:
        return None
    fields, raw_rows = parsed
    if tuple(fields) != TURNTABLE_V1_TRACE_COLUMNS:
        errors.append(
            f"{location}: expected columns {list(TURNTABLE_V1_TRACE_COLUMNS)!r}"
        )
        return None
    if len(raw_rows) != 361:
        errors.append(f"{location}: expected 361 rows, got {len(raw_rows)}")
        return None

    rows: list[dict[str, Any]] = []
    cumulative = {field: 0 for field in ("exact_solves", "warm_starts", "fallbacks")}
    failure_steps: list[int] = []
    accepted_steps: list[int] = []
    max_residual = 0.0
    solver_started = False
    for index, raw in enumerate(raw_rows):
        row_location = f"{location}.rows[{index}]"
        typed: dict[str, Any] = {}
        try:
            step = int(raw["step"])
        except (KeyError, TypeError, ValueError, OverflowError):
            errors.append(f"{row_location}.step: expected a canonical integer")
            return None
        if str(step) != raw["step"] or step != index:
            errors.append(f"{row_location}.step: expected {index}, got {raw['step']!r}")
            return None
        try:
            time_seconds = float(raw["time"])
        except (KeyError, TypeError, ValueError, OverflowError):
            errors.append(f"{row_location}.time: expected a finite number")
            return None
        if not math.isfinite(time_seconds) or not math.isclose(
            time_seconds, step / 60.0, rel_tol=0.0, abs_tol=2.0e-14
        ):
            errors.append(f"{row_location}.time: expected step/60")
            return None
        if raw.get("scenario") != scenario:
            errors.append(f"{row_location}.scenario: expected {scenario!r}")
            return None
        if raw.get("solver") != "exact_fbf" or raw.get("body") != (
            "turntable_rider_body"
        ):
            errors.append(f"{row_location}: solver/body identity changed")
            return None
        typed.update(
            {
                "step": step,
                "time": time_seconds,
                "scenario": scenario,
                "solver": raw["solver"],
                "body": raw["body"],
                "status": raw.get("status"),
            }
        )
        for field in ("x", "y", "z", "vx", "vy", "vz", "up_z"):
            try:
                value = float(raw[field])
            except (KeyError, TypeError, ValueError, OverflowError):
                errors.append(f"{row_location}.{field}: expected a finite number")
                return None
            if not math.isfinite(value):
                errors.append(f"{row_location}.{field}: expected a finite number")
                return None
            typed[field] = value
        for field in ("contacts", "exact_solves", "warm_starts", "fallbacks"):
            try:
                value = int(raw[field])
            except (KeyError, TypeError, ValueError, OverflowError):
                errors.append(f"{row_location}.{field}: expected a canonical integer")
                return None
            if str(value) != raw[field] or value < 0:
                errors.append(f"{row_location}.{field}: expected non-negative integer")
                return None
            if field in cumulative:
                if value < cumulative[field]:
                    errors.append(f"{row_location}.{field}: cumulative value regressed")
                    return None
                cumulative[field] = value
            typed[field] = value

        residual_text = raw.get("residual", "")
        try:
            residual = float(residual_text)
        except (TypeError, ValueError, OverflowError):
            errors.append(f"{row_location}.residual: expected a number")
            return None
        if typed["status"] == "not_run":
            if not math.isnan(residual):
                errors.append(f"{row_location}.residual: not_run requires nan")
                return None
            if solver_started or typed["contacts"] != 0 or any(cumulative.values()):
                errors.append(
                    f"{row_location}: not_run must be the zero-contact counter prefix"
                )
                return None
        else:
            if step == 0 or typed["status"] not in {
                "success",
                "max_iterations_accepted",
            }:
                errors.append(f"{row_location}.status: solver status changed")
                return None
            if not math.isfinite(residual) or residual < 0.0:
                errors.append(f"{row_location}.residual: expected finite non-negative")
                return None
            solver_started = True
            max_residual = max(max_residual, residual)
            if typed["status"] != "success" or residual > PAPER_RESIDUAL_TOLERANCE:
                failure_steps.append(step)
            if typed["status"] == "max_iterations_accepted":
                accepted_steps.append(step)
        typed["residual"] = residual
        rows.append(typed)

    if any(row["fallbacks"] != 0 for row in rows):
        errors.append(f"{location}: boxed-LCP fallback observed")
    if rows[-1]["exact_solves"] <= 0 or not any(
        row["contacts"] > 0 for row in rows[1:]
    ):
        errors.append(f"{location}: exact-contact trajectory is missing")

    support_fallthrough_steps: list[int] = []
    radial_exit_steps: list[int] = []
    source_exit_steps: list[int] = []
    for row in rows:
        radius = math.hypot(row["x"], row["y"])
        radial_exit = radius > 2.15
        if radial_exit:
            radial_exit_steps.append(row["step"])
        if radius <= 1.85 and row["z"] < -0.05:
            support_fallthrough_steps.append(row["step"])
        if radial_exit or row["z"] < -0.4:
            source_exit_steps.append(row["step"])
    final = rows[-1]
    final_radius = math.hypot(final["x"], final["y"])
    if support_fallthrough_steps:
        outcome = "invalid_support_fallthrough"
    elif (
        final["contacts"] > 0
        and final["z"] > 0.1
        and 0.75 <= final_radius <= 1.35
        and not source_exit_steps
    ):
        outcome = "captured"
    elif final["contacts"] == 0 and final_radius > 2.15:
        outcome = "ejected"
    else:
        outcome = "indeterminate"
    expected = TURNTABLE_V1_SCENARIOS[scenario]["expected_outcome"]
    expected_classification = (
        "captured" if expected == "retained_through_6s" else "ejected"
    )
    outcome_matches = outcome == expected_classification
    contract_valid = not failure_steps
    if lane == "current_visual" and not contract_valid:
        errors.append(
            f"{location}: current visual solver contract failed at {failure_steps[0]}"
        )
    if lane == "current_visual" and not outcome_matches:
        errors.append(
            f"{location}: current visual outcome {outcome!r} does not match "
            f"{expected!r}"
        )
    projection = [
        {
            "step": row["step"],
            "contacts": row["contacts"],
            "exact_solves": row["exact_solves"],
            "warm_starts": row["warm_starts"],
            "boxed_lcp_fallbacks": row["fallbacks"],
            "status": row["status"],
        }
        for row in rows
    ]
    return {
        "scenario": scenario,
        "capture_id": TURNTABLE_V1_SCENARIOS[scenario]["capture_id"],
        "lane": lane,
        "solver_contract": TURNTABLE_V1_LANES[lane]["solver_contract"],
        "collision_frontend": "native",
        "row_count": 361,
        "completed_steps": 360,
        "exact_solves": final["exact_solves"],
        "warm_starts": final["warm_starts"],
        "boxed_lcp_fallbacks": final["fallbacks"],
        "max_residual": max_residual,
        "accepted_at_cap_steps": accepted_steps,
        "solver_contract_failure_steps": failure_steps,
        "solver_contract_valid": contract_valid,
        "expected_outcome": expected,
        "classified_outcome": outcome,
        "expected_outcome_match": outcome_matches,
        "physical_outcome_authority": lane == "current_visual",
        "support_history_valid": not support_fallthrough_steps,
        "support_fallthrough_steps": support_fallthrough_steps,
        "radial_exit_steps": radial_exit_steps,
        "source_exit_steps": source_exit_steps,
        "projection": projection,
        "projection_sha256": _payload_sha256(projection),
    }


def _turntable_v1_capture_projection(
    timeline: dict[str, Any], location: str, errors: list[str]
) -> list[dict[str, Any]] | None:
    steps = timeline.get("steps")
    if isinstance(steps, list):
        if len(steps) != 361 or [
            entry.get("step") if isinstance(entry, dict) else None for entry in steps
        ] != list(range(361)):
            errors.append(f"{location}.steps: expected exactly steps 0 through 360")
            return None
        by_step = {str(entry["step"]): entry for entry in steps}
    elif isinstance(steps, dict) and set(steps) == {str(step) for step in range(361)}:
        by_step = steps
    else:
        errors.append(f"{location}.steps: expected exactly steps 0 through 360")
        return None
    projection = []
    for step in range(361):
        entry = by_step.get(str(step))
        diagnostics = (
            entry.get("solver_diagnostics") if isinstance(entry, dict) else None
        )
        if not isinstance(diagnostics, dict):
            errors.append(f"{location}.steps[{step}]: solver diagnostics missing")
            return None
        projection.append(
            {
                "step": step,
                "contacts": diagnostics.get("contacts"),
                "exact_solves": diagnostics.get("exact_solves"),
                "warm_starts": diagnostics.get("warm_starts"),
                "boxed_lcp_fallbacks": diagnostics.get("boxed_lcp_fallbacks"),
                "status": diagnostics.get("status"),
            }
        )
    return projection


def _validate_turntable_v1_lane_separation(
    trace_summary: dict[str, Any], location: str, errors: list[str]
) -> None:
    _expect_fields(
        trace_summary,
        {
            "render_binding_lane": "current_visual",
            "separate_diagnostic_lane": "paper_cpu_native",
            "cross_lane_substitution_allowed": False,
            "paper_cpu_native_projection_compared_to_capture": False,
        },
        location,
        errors,
    )
    lanes = _object(trace_summary.get("lanes"), f"{location}.lanes", errors)
    if lanes is None:
        return
    if tuple(lanes) != tuple(TURNTABLE_V1_LANES):
        errors.append(f"{location}.lanes: expected current then diagnostic lane")
    for lane, contract in TURNTABLE_V1_LANES.items():
        lane_data = _object(lanes.get(lane), f"{location}.lanes.{lane}", errors)
        if lane_data is None:
            continue
        _expect_fields(
            lane_data,
            {
                "contract": contract,
                "physical_outcome_authority": lane == "current_visual",
            },
            f"{location}.lanes.{lane}",
            errors,
        )


def _validate_turntable_v1_strict_diagnostic(
    parsed_traces: dict[str, dict[str, dict[str, Any]]],
    location: str,
    errors: list[str],
) -> None:
    strict = parsed_traces.get("paper_cpu_native", {})
    strict_failures = {
        scenario: metrics
        for scenario, metrics in strict.items()
        if not metrics["solver_contract_valid"]
    }
    if set(strict_failures) != {"turntable_author_mu_0_5_omega_2"}:
        errors.append(f"{location}: expected only mu=.5/omega=2 strict failure")
        return
    failure = strict_failures["turntable_author_mu_0_5_omega_2"]
    if (
        failure["accepted_at_cap_steps"] != [40]
        or failure["solver_contract_failure_steps"] != [40]
        or not math.isclose(
            failure["max_residual"],
            7.407835021099202e-06,
            rel_tol=1e-12,
            abs_tol=1e-15,
        )
    ):
        errors.append(f"{location}: strict diagnostic failure changed")


def _validate_turntable_v1_physics_contract(
    payload: Any,
    scenario: str,
    solver_contract: str,
    binary_role: str,
    source_hashes: dict[str, str],
    location: str,
    errors: list[str],
) -> dict[str, Any] | None:
    contract = _object(payload, location, errors)
    if contract is None:
        return None
    expected_fields = {
        "schema_version",
        "kind",
        "author_source",
        "physics_spec_source_sha256",
        "binary_binding",
        "scenario",
        "world",
        "collision",
        "solver",
        "support",
        "rider",
        "control",
        "visual_asset_identity",
    }
    if set(contract) != expected_fields:
        errors.append(f"{location}: physics fields changed")
    implementation_key = (
        "demo_source" if binary_role == "dart_demos" else "trace_source"
    )
    _expect_fields(
        contract,
        {
            "schema_version": "dart.fbf_author_turntable_physics_contract/v1",
            "kind": "physics_control",
            "author_source": {
                "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
                "turntable_run_py_sha256": (
                    "5dd330d2e430585fc3e61eb9c11d2d9aa00b518170f7b64b809c69587cf7db53"
                ),
            },
            "physics_spec_source_sha256": source_hashes.get(
                "author_turntable_spec_source"
            ),
            "binary_binding": {
                "role": binary_role,
                "implementation_source_sha256": source_hashes.get(implementation_key),
            },
            "visual_asset_identity": None,
        },
        location,
        errors,
    )
    scenario_contract = TURNTABLE_V1_SCENARIOS[scenario]
    _expect_fields(
        _object(contract.get("scenario"), f"{location}.scenario", errors) or {},
        {
            "trace_id": scenario,
            "demo_scene_id": scenario_contract["scene"],
            "friction": scenario_contract["mu"],
            "angular_velocity_rad_s": scenario_contract["omega"],
            "expected_outcome": scenario_contract["expected_outcome"],
        },
        f"{location}.scenario",
        errors,
    )
    _expect_fields(
        _object(contract.get("world"), f"{location}.world", errors) or {},
        {
            "time_step_seconds": 1.0 / 60.0,
            "gravity_m_s2": [0.0, 0.0, -9.81],
            "simulation_threads": 1,
            "deactivation_enabled": False,
        },
        f"{location}.world",
        errors,
    )
    _expect_fields(
        _object(contract.get("collision"), f"{location}.collision", errors) or {},
        {
            "detector": "native",
            "contact_manifold": "four_point_planar",
            "max_contacts": 4,
            "max_contacts_per_pair": 4,
        },
        f"{location}.collision",
        errors,
    )
    expected_solvers = {
        "dart_best": {
            "type": "exact_fbf",
            "contract": "dart_best",
            "split_impulse_enabled": False,
            "max_outer_iterations": 500,
            "tolerance": 1.0e-6,
            "inner_max_sweeps": 120,
            "inner_local_iterations": 32,
            "step_size_scale": 2.0,
            "warm_start_enabled": True,
            "fallback_to_boxed_lcp_enabled": True,
            "projected_gradient_retry_enabled": True,
            "dense_residual_polish_enabled": True,
            "contact_row_operator_enabled": True,
            "dense_contact_row_snapshot_enabled": True,
        },
        "paper_cpu": {
            "type": "exact_fbf",
            "contract": "paper_cpu",
            "split_impulse_enabled": False,
            "max_outer_iterations": 200,
            "tolerance": 1.0e-6,
            "inner_max_sweeps": 10,
            "inner_local_iterations": 1,
            "step_size_scale": 1.0,
            "warm_start_enabled": True,
            "fallback_to_boxed_lcp_enabled": False,
            "projected_gradient_retry_enabled": False,
            "dense_residual_polish_enabled": False,
            "contact_row_operator_enabled": True,
            "dense_contact_row_snapshot_enabled": False,
        },
    }
    solver = _object(contract.get("solver"), f"{location}.solver", errors)
    if solver != expected_solvers[solver_contract]:
        errors.append(f"{location}.solver: {solver_contract} contract changed")
    _expect_fields(
        _object(contract.get("support"), f"{location}.support", errors) or {},
        {
            "shape": "cylinder",
            "mobile": False,
            "radius_m": 2,
            "height_m": 0.1,
            "friction": scenario_contract["mu"],
            "initial_pose": {
                "translation": [0.0, 0.0, 0.05],
                "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
        },
        f"{location}.support",
        errors,
    )
    rider = _object(contract.get("rider"), f"{location}.rider", errors) or {}
    mass = 13.5
    moment = mass * (0.3 * 0.3 + 0.3 * 0.3) / 12.0
    _expect_fields(
        rider,
        {
            "shape": "box",
            "size_m": [0.3, 0.3, 0.3],
            "friction": scenario_contract["mu"],
            "mass_kg": mass,
            "moment_kg_m2": [
                [moment, 0.0, 0.0],
                [0.0, moment, 0.0],
                [0.0, 0.0, moment],
            ],
            "initial_pose": {
                "translation": [1.0, 0.0, 0.455],
                "rotation": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            },
            "initial_linear_velocity_m_s": [0.0, 0.0, 0.0],
            "initial_angular_velocity_rad_s": [0.0, 0.0, 0.0],
        },
        f"{location}.rider",
        errors,
    )
    _expect_fields(
        _object(contract.get("control"), f"{location}.control", errors) or {},
        {
            "settle_duration_seconds": 0.5,
            "ramp_duration_seconds": 0.5,
            "duration_seconds": 6,
            "ramp": "cubic_smoothstep",
        },
        f"{location}.control",
        errors,
    )
    return contract


def _validate_turntable_v1_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    key = "turntable_author_visual_v1_nonpaper"
    if key not in current_truth:
        # Manifest integration is intentionally separable from validator/test work.
        return
    location = f"current_truth.{key}"
    data = _object(current_truth.get(key), location, errors)
    if data is None:
        return
    bundle = _validate_current_path(
        data,
        "bundle",
        TURNTABLE_V1_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    _expect_fields(
        data,
        {
            "status": "valid_author_source_pinned_nonpaper_turntable_matrix",
            "artifact_valid": True,
            "solver_contract_valid": True,
            "physical_outcome_valid": True,
            "render_binding_lane": "current_visual",
            "separate_diagnostic_lane": "paper_cpu_native",
            "cross_lane_substitution_allowed": False,
            "paper_cpu_native_all_solver_contract_valid": False,
            "paper_parity": False,
            "paper_comparable": False,
            "external_solver_parity": False,
            "approved_source_golden": False,
            "timing_verdict": None,
            "realtime_verdict": None,
            "actual_simulator": True,
            "generated_imagery": False,
            "automated_semantic_outcome_validated": False,
            "manual_visual_outcome_validated": True,
            "trace_equivalence_to_rendered_demo": False,
            "solver_projection_equivalent": True,
            "core_solver_contact_projection_equivalent": True,
            "source_order_validated": True,
            "author_source_pinned": True,
            "author_source_commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
            "author_configuration_port": True,
            "steps": 360,
            "trace_rows_per_scenario": 361,
            "member_frames": 181,
            "group_video_frames": 181,
            "group_video_fps": 30,
            "group_dimensions": [1320, 1060],
            "group_panel_dimensions": [1344, 1076],
            "manual_inspected": True,
            "artifact_count": 58,
            "claim_scope": TURNTABLE_V1_CLAIM_SCOPE,
            "claim_boundary": TURNTABLE_V1_CLAIM_BOUNDARY,
        },
        location,
        errors,
    )
    hashes = _validate_artifact_hashes(
        data, TURNTABLE_V1_ARTIFACT_TARGETS, location, repo_root, errors
    )
    if bundle is None:
        return

    actual_paths: set[str] = set()
    actual_directories: set[str] = set()
    unsafe: list[str] = []
    for path in bundle.rglob("*"):
        relative = path.relative_to(bundle).as_posix()
        if path.is_symlink():
            unsafe.append(relative)
        elif path.is_file():
            actual_paths.add(relative)
        elif path.is_dir():
            actual_directories.add(relative)
    expected_paths = _turntable_v1_expected_bundle_paths()
    expected_directories = {
        "groups",
        "groups/turntable_author",
        "groups/turntable_author/panel_members",
        "traces",
        *(f"traces/{lane}" for lane in TURNTABLE_V1_LANES),
        *TURNTABLE_V1_CAPTURE_IDS,
        *(f"{capture_id}/stills" for capture_id in TURNTABLE_V1_CAPTURE_IDS),
    }
    if unsafe:
        errors.append(f"{location}.bundle: symlinks are forbidden: {sorted(unsafe)}")
    missing = sorted(expected_paths - actual_paths)
    unexpected = sorted(actual_paths - expected_paths)
    missing_directories = sorted(expected_directories - actual_directories)
    unexpected_directories = sorted(actual_directories - expected_directories)
    if missing or unexpected or missing_directories or unexpected_directories:
        errors.append(
            f"{location}.bundle: exact membership changed; missing={missing}, "
            f"unexpected={unexpected}, missing_directories={missing_directories}, "
            f"unexpected_directories={unexpected_directories}"
        )

    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    trace_summary = _read_bundle_json(
        bundle, "trace-summary.json", f"{location}.trace_summary", errors
    )
    group = _read_bundle_json(
        bundle,
        "groups/turntable_author/metadata.json",
        f"{location}.group",
        errors,
    )
    if metadata is not None:
        _expect_fields(
            metadata,
            {
                "schema_version": "dart.fbf_turntable_visual_bundle/v1",
                "status": data.get("status"),
                "pass": True,
                "requirement_ids": ["fig.04", "video.04_turntable"],
                "artifact_valid": data.get("artifact_valid"),
                "solver_contract_valid": data.get("solver_contract_valid"),
                "physical_outcome_valid": data.get("physical_outcome_valid"),
                "render_binding_lane": data.get("render_binding_lane"),
                "separate_diagnostic_lane": data.get("separate_diagnostic_lane"),
                "cross_lane_substitution_allowed": data.get(
                    "cross_lane_substitution_allowed"
                ),
                "paper_cpu_native_all_solver_contract_valid": data.get(
                    "paper_cpu_native_all_solver_contract_valid"
                ),
                "paper_parity": False,
                "paper_comparable": False,
                "external_solver_parity": False,
                "approved_source_golden": False,
                "timing_verdict": None,
                "realtime_verdict": None,
                "actual_simulator": True,
                "generated_imagery": False,
                "automated_semantic_outcome_validated": False,
                "manual_visual_outcome_validated": True,
                "trace_equivalence_to_rendered_demo": False,
                "solver_projection_equivalent": True,
                "core_solver_contact_projection_equivalent": True,
                "source_order_validated": True,
                "author_source_pinned": True,
                "author_source_commit": data.get("author_source_commit"),
                "author_configuration_port": True,
                "claim_scope": data.get("claim_scope"),
                "claim_boundary": data.get("claim_boundary"),
                "trace_summary": trace_summary,
            },
            f"{location}.metadata",
            errors,
        )
        if not _nonempty_string(metadata.get("evidence_date")):
            errors.append(f"{location}.metadata.evidence_date: expected a date")
        for field, relative in (
            ("manual_inspection", "manual-inspection.json"),
            ("run_summary", "run-summary.json"),
            ("verification", "verification.json"),
            ("invocations", "invocations.json"),
            ("report", "REPORT.md"),
        ):
            expected_binding = {
                "path": relative,
                "sha256": hashes.get(relative),
            }
            if field == "manual_inspection":
                expected_binding["pass"] = True
            _expect_fields(
                metadata,
                {field: expected_binding},
                f"{location}.metadata",
                errors,
            )
        artifact_binding = _object(
            metadata.get("artifact_index"),
            f"{location}.metadata.artifact_index",
            errors,
        )
        if artifact_binding is not None:
            _expect_fields(
                artifact_binding,
                {
                    "path": "artifact-index.json",
                    "sha256": hashes.get("artifact-index.json"),
                    "artifact_count": 58,
                    "excluded": ["artifact-index.json", "metadata.json"],
                },
                f"{location}.metadata.artifact_index",
                errors,
            )

    parsed_traces: dict[str, dict[str, dict[str, Any]]] = {
        lane: {} for lane in TURNTABLE_V1_LANES
    }
    for lane in TURNTABLE_V1_LANES:
        for scenario in TURNTABLE_V1_SCENARIOS:
            relative = f"traces/{lane}/{scenario}.csv"
            parsed = _read_csv(bundle / relative, f"{location}.{relative}", errors)
            metrics = _turntable_v1_trace_metrics(
                parsed,
                scenario,
                lane,
                f"{location}.{relative}",
                errors,
            )
            if metrics is not None:
                parsed_traces[lane][scenario] = metrics

    if trace_summary is not None:
        _validate_turntable_v1_lane_separation(
            trace_summary, f"{location}.trace_summary", errors
        )
        _expect_fields(
            trace_summary,
            {
                "schema_version": "dart.fbf_turntable_trace_summary/v1",
                "pass": True,
                "render_binding_lane": "current_visual",
                "separate_diagnostic_lane": "paper_cpu_native",
                "cross_lane_substitution_allowed": False,
                "all_visual_capture_core_projections_equal": True,
                "all_visual_capture_full_projections_byte_identical": True,
                "full_projection_byte_identical_count": 4,
                "paper_cpu_native_projection_compared_to_capture": False,
            },
            f"{location}.trace_summary",
            errors,
        )
        lanes = _object(
            trace_summary.get("lanes"), f"{location}.trace_summary.lanes", errors
        )
        if lanes is not None:
            if tuple(lanes) != tuple(TURNTABLE_V1_LANES):
                errors.append(
                    f"{location}.trace_summary.lanes: lane order/separation changed"
                )
            for lane, lane_contract in TURNTABLE_V1_LANES.items():
                lane_data = _object(
                    lanes.get(lane), f"{location}.trace_summary.lanes.{lane}", errors
                )
                if lane_data is None:
                    continue
                expected_all_valid = lane == "current_visual"
                _expect_fields(
                    lane_data,
                    {
                        "contract": lane_contract,
                        "scenario_order": list(TURNTABLE_V1_SCENARIOS),
                        "all_solver_contract_valid": expected_all_valid,
                        "all_expected_outcomes_match": True,
                        "physical_outcome_authority": lane == "current_visual",
                    },
                    f"{location}.trace_summary.lanes.{lane}",
                    errors,
                )
                scenarios = lane_data.get("scenarios")
                if not isinstance(scenarios, list) or len(scenarios) != 4:
                    errors.append(
                        f"{location}.trace_summary.lanes.{lane}.scenarios: expected four"
                    )
                    continue
                if [
                    item.get("scenario") for item in scenarios if isinstance(item, dict)
                ] != list(TURNTABLE_V1_SCENARIOS):
                    errors.append(
                        f"{location}.trace_summary.lanes.{lane}.scenarios: "
                        "source order changed"
                    )
                for item in scenarios:
                    if not isinstance(item, dict):
                        continue
                    scenario = item.get("scenario")
                    computed = parsed_traces.get(lane, {}).get(scenario)
                    if computed is None:
                        continue
                    expected_fields = {
                        field: computed[field]
                        for field in (
                            "scenario",
                            "capture_id",
                            "lane",
                            "solver_contract",
                            "collision_frontend",
                            "row_count",
                            "completed_steps",
                            "exact_solves",
                            "warm_starts",
                            "boxed_lcp_fallbacks",
                            "max_residual",
                            "accepted_at_cap_steps",
                            "solver_contract_failure_steps",
                            "solver_contract_valid",
                            "expected_outcome",
                            "classified_outcome",
                            "expected_outcome_match",
                            "physical_outcome_authority",
                            "support_history_valid",
                            "support_fallthrough_steps",
                            "radial_exit_steps",
                            "source_exit_steps",
                        )
                    }
                    _expect_fields(
                        item,
                        expected_fields,
                        f"{location}.trace_summary.lanes.{lane}.{scenario}",
                        errors,
                    )

    _validate_turntable_v1_strict_diagnostic(
        parsed_traces, f"{location}.paper_cpu_native", errors
    )

    for scenario, scenario_contract in TURNTABLE_V1_SCENARIOS.items():
        capture_id = scenario_contract["capture_id"]
        member = _read_bundle_json(
            bundle,
            f"{capture_id}/metadata.json",
            f"{location}.members.{capture_id}.metadata",
            errors,
        )
        timeline = _read_bundle_json(
            bundle,
            f"{capture_id}/timeline.json",
            f"{location}.members.{capture_id}.timeline",
            errors,
        )
        if member is not None:
            _expect_fields(
                member,
                {
                    "schema_version": "dart.fbf_visual_evidence/v1",
                    "kind": "capture_result",
                    "pass": True,
                    "actual_simulator": True,
                    "generated_imagery": False,
                    "paper_comparable": False,
                    "automated_semantic_outcome_validated": False,
                },
                f"{location}.members.{capture_id}.metadata",
                errors,
            )
            schedule = _object(
                member.get("schedule"),
                f"{location}.members.{capture_id}.schedule",
                errors,
            )
            if schedule is not None:
                _expect_fields(
                    schedule,
                    {
                        "id": capture_id,
                        "scene": scenario_contract["scene"],
                        "source_segment": "turntable",
                        "collision_detector": "native",
                        "collision_detector_override": False,
                        "total_steps": 360,
                        "time_step_seconds": 1.0 / 60.0,
                        "capture_steps": list(range(0, 361, 2)),
                        "panel_steps": [0, 30, 60, 180, 360],
                        "panel_labels": [
                            "t=0s",
                            "settled t=.5s",
                            "ramped t=1s",
                            "t=3s",
                            "t=6s",
                        ],
                        "actions": [],
                        "runnable": True,
                        "adapter_gap": None,
                        "long_run": False,
                        "actual_simulator_required": True,
                        "generated_imagery_allowed": False,
                        "paper_comparable": False,
                        "known_gate_blockers": [],
                        "evidence_ready": True,
                    },
                    f"{location}.members.{capture_id}.schedule",
                    errors,
                )
            media = member.get("media_validation")
            if not isinstance(media, list) or len(media) != 1:
                errors.append(
                    f"{location}.members.{capture_id}.media_validation: expected one MP4"
                )
            else:
                _expect_fields(
                    media[0],
                    {
                        "kind": "mp4",
                        "sha256": hashes.get(f"{capture_id}/clip.mp4"),
                        "full_decode": "pass",
                        "stream_contract": {
                            "width": 660,
                            "height": 506,
                            "frame_rate": "30/1",
                            "frame_rate_rational": "30/1",
                            "frame_count": 181,
                        },
                    },
                    f"{location}.members.{capture_id}.media_validation[0]",
                    errors,
                )
        if timeline is not None:
            physics = timeline.get("physics_contract")
            _validate_turntable_v1_physics_contract(
                physics,
                scenario,
                "dart_best",
                "dart_demos",
                hashes,
                f"{location}.members.{capture_id}.timeline.physics_contract",
                errors,
            )
            capture_projection = _turntable_v1_capture_projection(
                timeline, f"{location}.members.{capture_id}.timeline", errors
            )
            trace = parsed_traces.get("current_visual", {}).get(scenario)
            if trace is not None and capture_projection is not None:
                if trace["projection"] != capture_projection:
                    errors.append(
                        f"{location}.{scenario}: current trace/capture full projection differs"
                    )

    _validate_turntable_v1_group(group, bundle, hashes, location, errors)
    _validate_turntable_v1_manual(bundle, hashes, location, errors)
    _validate_turntable_v1_sources_and_physics(
        metadata, repo_root, hashes, location, errors
    )
    _validate_turntable_v1_index(bundle, data, location, errors)


def _validate_turntable_v1_group(
    group: dict[str, Any] | None,
    bundle: Path,
    hashes: dict[str, str],
    location: str,
    errors: list[str],
) -> None:
    group_location = f"{location}.group"
    if group is None:
        return
    _expect_fields(
        group,
        {
            "schema_version": "dart.fbf_visual_evidence/v1",
            "kind": "group_capture_result",
            "group_id": "turntable_author",
            "source_segment": "turntable",
            "layout": "2x2",
            "member_order": list(TURNTABLE_V1_CAPTURE_IDS),
            "labels": list(TURNTABLE_V1_GROUP_LABELS),
            "actual_simulator": True,
            "generated_imagery": False,
            "paper_comparable": False,
            "automated_semantic_outcome_validated": False,
            "pass": True,
        },
        group_location,
        errors,
    )
    synchronization = _object(
        group.get("synchronization"), f"{group_location}.synchronization", errors
    )
    if synchronization is not None:
        _expect_fields(
            synchronization,
            {
                "total_steps": 360,
                "video_steps": list(range(0, 361, 2)),
                "output_fps": 30,
                "frame_count": 181,
                "frame_rate": "30/1",
            },
            f"{group_location}.synchronization",
            errors,
        )
        duration = _number(synchronization.get("duration_seconds"))
        tolerance = _number(synchronization.get("duration_tolerance_seconds"))
        if (
            duration is None
            or tolerance is None
            or abs(duration - 181 / 30) > tolerance
        ):
            errors.append(f"{group_location}.synchronization: duration changed")

    members = group.get("members")
    if not isinstance(members, list) or [
        item.get("id") for item in members if isinstance(item, dict)
    ] != list(TURNTABLE_V1_CAPTURE_IDS):
        errors.append(f"{group_location}.members: source order changed")
    else:
        for capture_id, panel_step, member in zip(
            TURNTABLE_V1_CAPTURE_IDS, TURNTABLE_V1_GROUP_PANEL_STEPS, members
        ):
            if not isinstance(member, dict):
                continue
            expected = {
                "metadata_path": str(bundle / capture_id / "metadata.json"),
                "metadata_sha256": hashes.get(f"{capture_id}/metadata.json"),
                "timeline_path": str(bundle / capture_id / "timeline.json"),
                "timeline_sha256": hashes.get(f"{capture_id}/timeline.json"),
                "panel_source_step": panel_step,
                "panel_source_time_seconds": panel_step / 60.0,
                "panel_source_frame": str(
                    bundle / capture_id / "frames" / f"step_{panel_step:06d}.png"
                ),
                "panel_source_frame_sha256": _sha256(
                    bundle
                    / capture_id
                    / "stills"
                    / f"outcome_step_{panel_step:06d}.png"
                ),
                "clip_path": str(bundle / capture_id / "clip.mp4"),
                "clip_sha256": hashes.get(f"{capture_id}/clip.mp4"),
            }
            _expect_fields(
                member, expected, f"{group_location}.members.{capture_id}", errors
            )

    panel = _object(
        group.get("panel_validation"), f"{group_location}.panel_validation", errors
    )
    if panel is not None:
        if "panel_step" in panel:
            errors.append(
                f"{group_location}.panel_validation.panel_step: shared step forbidden"
            )
        _expect_fields(
            panel,
            {
                "member_order": list(TURNTABLE_V1_CAPTURE_IDS),
                "labels": list(TURNTABLE_V1_GROUP_PANEL_LABELS),
                "path": str(bundle / "groups/turntable_author/panel.png"),
                "sha256": hashes.get("groups/turntable_author/panel.png"),
                "full_decode": "pass",
            },
            f"{group_location}.panel_validation",
            errors,
        )
        sources = panel.get("source_frames")
        expected_sources = []
        for capture_id, panel_step in zip(
            TURNTABLE_V1_CAPTURE_IDS, TURNTABLE_V1_GROUP_PANEL_STEPS
        ):
            frame = bundle / capture_id / "frames" / f"step_{panel_step:06d}.png"
            durable = (
                bundle / capture_id / "stills" / f"outcome_step_{panel_step:06d}.png"
            )
            expected_sources.append(
                {
                    "member": capture_id,
                    "step": panel_step,
                    "time_seconds": panel_step / 60.0,
                    "path": str(frame),
                    "sha256": _sha256(durable),
                }
            )
        if sources != expected_sources:
            errors.append(
                f"{group_location}.panel_validation.source_frames: "
                "outcome-aware step/order binding changed"
            )
        verdict = panel.get("verdict")
        image = verdict.get("image") if isinstance(verdict, dict) else None
        if (
            not isinstance(verdict, dict)
            or verdict.get("pass") is not True
            or not isinstance(image, dict)
            or image.get("width") != 1344
            or image.get("height") != 1076
        ):
            errors.append(
                f"{group_location}.panel_validation.verdict: dimensions changed"
            )

    media = _object(
        group.get("media_validation"), f"{group_location}.media_validation", errors
    )
    if media is not None:
        _expect_fields(
            media,
            {
                "kind": "mp4",
                "path": str(bundle / "groups/turntable_author/clip.mp4"),
                "sha256": hashes.get("groups/turntable_author/clip.mp4"),
                "full_decode": "pass",
            },
            f"{group_location}.media_validation",
            errors,
        )
        streams = media.get("probe", {}).get("streams", [])
        videos = (
            [
                stream
                for stream in streams
                if isinstance(stream, dict) and stream.get("width")
            ]
            if isinstance(streams, list)
            else []
        )
        if len(videos) != 1:
            errors.append(
                f"{group_location}.media_validation.probe: expected one video"
            )
        else:
            _expect_fields(
                videos[0],
                {
                    "width": 1320,
                    "height": 1060,
                    "r_frame_rate": "30/1",
                    "nb_frames": "181",
                },
                f"{group_location}.media_validation.probe.streams[0]",
                errors,
            )

    run_summary = _read_bundle_json(
        bundle, "run-summary.json", f"{location}.run_summary", errors
    )
    if run_summary is not None:
        _expect_fields(
            run_summary,
            {
                "schema_version": "dart.fbf_visual_evidence/v1",
                "kind": "capture_run",
                "pass": True,
                "failures": [],
                "group_skips": [],
            },
            f"{location}.run_summary",
            errors,
        )
        results = run_summary.get("results")
        result_ids = []
        if isinstance(results, list):
            for item in results:
                schedule = item.get("schedule") if isinstance(item, dict) else None
                result_ids.append(
                    schedule.get("id") if isinstance(schedule, dict) else None
                )
        if result_ids != list(TURNTABLE_V1_CAPTURE_IDS):
            errors.append(f"{location}.run_summary.results: source order changed")
        groups = run_summary.get("group_outputs")
        if not isinstance(groups, list) or len(groups) != 1 or groups[0] != group:
            errors.append(
                f"{location}.run_summary.group_outputs: group binding changed"
            )

    verification = _read_bundle_json(
        bundle, "verification.json", f"{location}.verification", errors
    )
    if verification is not None:
        _expect_fields(
            verification,
            {
                "schema_version": "dart.fbf_visual_evidence/v1",
                "kind": "verification",
                "pass": True,
            },
            f"{location}.verification",
            errors,
        )
        results = verification.get("results")
        verified_ids = []
        if isinstance(results, list):
            for item in results:
                metadata_path = (
                    item.get("metadata_path") if isinstance(item, dict) else None
                )
                verified_ids.append(
                    Path(metadata_path).parent.name
                    if isinstance(metadata_path, str)
                    else None
                )
        if verified_ids != list(TURNTABLE_V1_CAPTURE_IDS):
            errors.append(f"{location}.verification.results: source order changed")
        groups = verification.get("group_outputs")
        if (
            not isinstance(groups, list)
            or len(groups) != 1
            or groups[0].get("group_id") != "turntable_author"
            or groups[0].get("pass") is not True
        ):
            errors.append(f"{location}.verification.group_outputs: group changed")


def _validate_turntable_v1_manual(
    bundle: Path, hashes: dict[str, str], location: str, errors: list[str]
) -> None:
    record = _read_bundle_json(
        bundle, "manual-inspection.json", f"{location}.manual_inspection", errors
    )
    if record is None:
        return
    if set(record) != {
        "schema_version",
        "manual_inspected",
        "pass",
        "verdicts",
        "representative_artifacts",
    }:
        errors.append(f"{location}.manual_inspection: top-level fields changed")
    _expect_fields(
        record,
        {
            "schema_version": "dart.fbf_turntable_manual_inspection/v1",
            "manual_inspected": True,
            "pass": True,
            "verdicts": TURNTABLE_V1_MANUAL_VERDICTS,
        },
        f"{location}.manual_inspection",
        errors,
    )
    expected_paths = {
        "groups/turntable_author/panel.png",
        "groups/turntable_author/clip.mp4",
        *(
            f"{capture_id}/stills/outcome_step_{step:06d}.png"
            for capture_id, step in zip(
                TURNTABLE_V1_CAPTURE_IDS, TURNTABLE_V1_GROUP_PANEL_STEPS
            )
        ),
    }
    artifacts = record.get("representative_artifacts")
    if not isinstance(artifacts, list):
        errors.append(
            f"{location}.manual_inspection.representative_artifacts: expected list"
        )
        return
    paths = [item.get("path") for item in artifacts if isinstance(item, dict)]
    if set(paths) != expected_paths or len(paths) != len(set(paths)):
        errors.append(
            f"{location}.manual_inspection.representative_artifacts: paths changed"
        )
    for index, item in enumerate(artifacts):
        item_location = (
            f"{location}.manual_inspection.representative_artifacts[{index}]"
        )
        if not isinstance(item, dict) or set(item) != {"path", "sha256", "observation"}:
            errors.append(f"{item_location}: malformed artifact")
            continue
        relative = item.get("path")
        if not isinstance(relative, str) or relative not in expected_paths:
            continue
        path = bundle / relative
        if item.get("sha256") != _sha256(path):
            errors.append(f"{item_location}.sha256: representative artifact changed")
        if not _nonempty_string(item.get("observation")):
            errors.append(f"{item_location}.observation: expected non-empty text")


def _validate_turntable_v1_index(
    bundle: Path, data: dict[str, Any], location: str, errors: list[str]
) -> None:
    index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )
    if index is None:
        return
    _expect_fields(
        index,
        {
            "schema_version": "dart.fbf_turntable_artifact_index/v1",
            "artifact_count": data.get("artifact_count"),
            "excluded": ["artifact-index.json", "metadata.json"],
        },
        f"{location}.artifact_index",
        errors,
    )
    artifacts = index.get("artifacts")
    if not isinstance(artifacts, list) or len(artifacts) != 58:
        errors.append(f"{location}.artifact_index.artifacts: expected 58 entries")
        return
    listed = [item.get("path") for item in artifacts if isinstance(item, dict)]
    if listed != sorted(set(listed)):
        errors.append(
            f"{location}.artifact_index.artifacts: paths must be sorted/unique"
        )
    disk_paths = {
        path.relative_to(bundle).as_posix()
        for path in bundle.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.relative_to(bundle).as_posix()
        not in {"artifact-index.json", "metadata.json"}
    }
    if set(listed) != disk_paths:
        errors.append(f"{location}.artifact_index.artifacts: exact membership changed")
    for index_number, item in enumerate(artifacts):
        item_location = f"{location}.artifact_index.artifacts[{index_number}]"
        if not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}:
            errors.append(f"{item_location}: malformed entry")
            continue
        relative = item.get("path")
        if not isinstance(relative, str) or relative not in disk_paths:
            continue
        path = bundle / relative
        _expect_fields(
            item,
            {"bytes": path.stat().st_size, "sha256": _sha256(path)},
            item_location,
            errors,
        )


def _validate_turntable_v1_sources_and_physics(
    metadata: dict[str, Any] | None,
    repo_root: Path,
    hashes: dict[str, str],
    location: str,
    errors: list[str],
) -> None:
    if metadata is None:
        return
    source_identity = _object(
        metadata.get("source_identity"),
        f"{location}.metadata.source_identity",
        errors,
    )
    source_keys = {
        key
        for key, path in TURNTABLE_V1_ARTIFACT_TARGETS.items()
        if not path.startswith(TURNTABLE_V1_BUNDLE)
    }
    if source_identity is not None:
        for key in sorted(source_keys):
            identity = _object(
                source_identity.get(key),
                f"{location}.metadata.source_identity.{key}",
                errors,
            )
            if identity is None:
                continue
            if key in {
                "author_turntable_visual_obj",
                "author_turntable_visual_mtl",
            }:
                source_path = repo_root / TURNTABLE_V1_ARTIFACT_TARGETS[key]
                expected_identity = {
                    "relative_path": TURNTABLE_V1_ARTIFACT_TARGETS[key],
                    "sha256": hashes.get(key),
                    "size_bytes": source_path.stat().st_size,
                }
            else:
                expected_identity = {
                    "path": str(
                        (repo_root / TURNTABLE_V1_ARTIFACT_TARGETS[key]).resolve()
                    ),
                    "sha256": hashes.get(key),
                }
            _expect_fields(
                identity,
                expected_identity,
                f"{location}.metadata.source_identity.{key}",
                errors,
            )

    obj_path = repo_root / TURNTABLE_V1_ARTIFACT_TARGETS["author_turntable_visual_obj"]
    mtl_path = repo_root / TURNTABLE_V1_ARTIFACT_TARGETS["author_turntable_visual_mtl"]
    try:
        visual_structure = _validate_turntable_v1_visual_resources(
            obj_path.read_text(encoding="utf-8"),
            mtl_path.read_text(encoding="utf-8"),
        )
    except (OSError, UnicodeDecodeError, ValueError) as error:
        errors.append(f"{location}.renderer_resources: {error}")
        visual_structure = None

    contract = _object(
        metadata.get("capture_source_contract"),
        f"{location}.metadata.capture_source_contract",
        errors,
    )
    if contract is None:
        return
    _expect_fields(
        contract,
        {
            "pass": True,
            "member_order": list(TURNTABLE_V1_CAPTURE_IDS),
            "labels": list(TURNTABLE_V1_GROUP_LABELS),
            "layout": "2x2",
            "panel_steps": list(TURNTABLE_V1_GROUP_PANEL_STEPS),
            "panel_labels": list(TURNTABLE_V1_GROUP_PANEL_LABELS),
            "support_geometry": "author_pinned_cylinder_radius_2_height_0.1",
            "source_support_geometry": "cylinder_radius_2_height_0.1",
            "cube_edge_length_m": 0.3,
            "cube_density_kg_m3": 500.0,
            "initial_geometric_gap_m": 0.005,
            "drop_height_m": 0.2,
            "settle_duration_seconds": 0.5,
            "ramp_duration_seconds": 0.5,
            "capture_duration_seconds": 6.0,
            "capture_collision_frontend": "native_four_point_planar",
            "collision_override_applied_after_scene_install": False,
            "author_numerical_source_available": True,
            "author_turntable_renderer_assets_available": True,
            "physics_contract_visual_asset_identity": None,
            "source_golden_available": False,
            "author_turntable_spec_source_sha256": hashes.get(
                "author_turntable_spec_source"
            ),
            "demo_source_sha256": hashes.get("demo_source"),
            "demo_query_source_sha256": hashes.get("demo_main_source"),
            "trace_query_source_sha256": hashes.get("trace_source"),
            "demo_cmake_source_sha256": hashes.get("demo_cmake_source"),
            "trace_cmake_source_sha256": hashes.get("trace_cmake_source"),
            "demo_host_source_sha256": hashes.get("demo_host_source"),
            "runner_source_sha256": hashes.get("visual_runner"),
        },
        f"{location}.metadata.capture_source_contract",
        errors,
    )
    author_source = _object(
        contract.get("author_source"),
        f"{location}.metadata.capture_source_contract.author_source",
        errors,
    )
    if author_source is not None:
        _expect_fields(
            author_source,
            {
                "url": "https://github.com/matthcsong/fbf-sca-2026",
                "commit": "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0",
                "license": "MIT",
            },
            f"{location}.metadata.capture_source_contract.author_source",
            errors,
        )
        scene_hashes = author_source.get("scene_source_sha256")
        if not isinstance(scene_hashes, dict) or scene_hashes.get(
            "turntable/run.py"
        ) != ("5dd330d2e430585fc3e61eb9c11d2d9aa00b518170f7b64b809c69587cf7db53"):
            errors.append(
                f"{location}.metadata.capture_source_contract.author_source: "
                "turntable source hash changed"
            )
        if contract.get("author_source_payload_sha256") != _payload_sha256(
            author_source
        ):
            errors.append(
                f"{location}.metadata.capture_source_contract.author_source_payload_sha256: "
                "payload binding changed"
            )

    renderer = _object(
        contract.get("renderer_resources"),
        f"{location}.metadata.capture_source_contract.renderer_resources",
        errors,
    )
    if renderer is not None and visual_structure is not None:
        _expect_fields(
            renderer,
            {"pass": True, **visual_structure},
            f"{location}.metadata.capture_source_contract.renderer_resources",
            errors,
        )
        resources = _object(
            renderer.get("resources"),
            f"{location}.metadata.capture_source_contract.renderer_resources.resources",
            errors,
        )
        if resources is not None:
            for key in (
                "author_turntable_visual_obj",
                "author_turntable_visual_mtl",
            ):
                identity = _object(
                    resources.get(key),
                    f"{location}.metadata.capture_source_contract.renderer_resources.resources.{key}",
                    errors,
                )
                if identity is not None:
                    _expect_fields(
                        identity,
                        {
                            "relative_path": TURNTABLE_V1_ARTIFACT_TARGETS[key],
                            "sha256": hashes.get(key),
                            "size_bytes": (
                                repo_root / TURNTABLE_V1_ARTIFACT_TARGETS[key]
                            )
                            .stat()
                            .st_size,
                        },
                        f"{location}.metadata.capture_source_contract.renderer_resources.resources.{key}",
                        errors,
                    )

    attachment = _object(
        contract.get("visual_attachment"),
        f"{location}.metadata.capture_source_contract.visual_attachment",
        errors,
    )
    if attachment is not None:
        _expect_fields(
            attachment,
            {
                "pass": True,
                "mesh_uri": "dart://sample/obj/fbf_author_turntable_disc.obj",
                "collision_geometry": "shared_spec_cylinder",
                "visual_geometry": "scaled_obj_mesh",
                "visual_aspect_only": True,
                "scale": [
                    "shared_spec_support_radius",
                    "shared_spec_support_radius",
                    "shared_spec_support_height",
                ],
                "demo_source_sha256": hashes.get("demo_source"),
            },
            f"{location}.metadata.capture_source_contract.visual_attachment",
            errors,
        )
    try:
        demo_source = (
            repo_root / TURNTABLE_V1_ARTIFACT_TARGETS["demo_source"]
        ).read_text(encoding="utf-8")
        compact = " ".join(demo_source.split())
    except (OSError, UnicodeDecodeError) as error:
        errors.append(
            f"{location}.demo_source: could not read visual attachment: {error}"
        )
        compact = ""
    required_fragments = (
        '"dart://sample/obj/fbf_author_turntable_disc.obj"',
        "createShapeNodeWith<CollisionAspect, DynamicsAspect>( collisionShape)",
        "createShapeNodeWith<VisualAspect>(visualShape)",
        "MeshShape::loadMesh(meshUri, retriever)",
    )
    if compact and any(fragment not in compact for fragment in required_fragments):
        errors.append(f"{location}.demo_source: renderer attachment changed")
    if compact and any(
        fragment in compact
        for fragment in (
            "createShapeNodeWith<CollisionAspect>(visualShape)",
            "createShapeNodeWith<DynamicsAspect>(visualShape)",
            "createShapeNodeWith<VisualAspect, CollisionAspect>(visualShape)",
            "createShapeNodeWith<VisualAspect, DynamicsAspect>(visualShape)",
        )
    ):
        errors.append(f"{location}.demo_source: visual mesh is no longer renderer-only")

    queries = _object(
        contract.get("physics_contract_queries"),
        f"{location}.metadata.capture_source_contract.physics_contract_queries",
        errors,
    )
    if queries is None:
        return
    if contract.get("physics_contract_queries_payload_sha256") != _payload_sha256(
        queries
    ):
        errors.append(
            f"{location}.metadata.capture_source_contract."
            "physics_contract_queries_payload_sha256: payload binding changed"
        )
    demo_queries = _object(
        queries.get("demo"),
        f"{location}.metadata.capture_source_contract.physics_contract_queries.demo",
        errors,
    )
    trace_queries = _object(
        queries.get("trace"),
        f"{location}.metadata.capture_source_contract.physics_contract_queries.trace",
        errors,
    )
    if demo_queries is None or trace_queries is None:
        return
    if tuple(demo_queries) != tuple(TURNTABLE_V1_SCENARIOS):
        errors.append(
            f"{location}.physics_contract_queries.demo: scenario order changed"
        )
    if tuple(trace_queries) != ("dart_best", "paper_cpu"):
        errors.append(f"{location}.physics_contract_queries.trace: lane order changed")
    trace_best = _object(
        trace_queries.get("dart_best"),
        f"{location}.physics_contract_queries.trace.dart_best",
        errors,
    )
    trace_paper = _object(
        trace_queries.get("paper_cpu"),
        f"{location}.physics_contract_queries.trace.paper_cpu",
        errors,
    )
    if trace_best is None or trace_paper is None:
        return
    for scenario in TURNTABLE_V1_SCENARIOS:
        demo_contract = _validate_turntable_v1_physics_contract(
            demo_queries.get(scenario),
            scenario,
            "dart_best",
            "dart_demos",
            hashes,
            f"{location}.physics_contract_queries.demo.{scenario}",
            errors,
        )
        best_contract = _validate_turntable_v1_physics_contract(
            trace_best.get(scenario),
            scenario,
            "dart_best",
            "fbf_paper_trace",
            hashes,
            f"{location}.physics_contract_queries.trace.dart_best.{scenario}",
            errors,
        )
        paper_contract = _validate_turntable_v1_physics_contract(
            trace_paper.get(scenario),
            scenario,
            "paper_cpu",
            "fbf_paper_trace",
            hashes,
            f"{location}.physics_contract_queries.trace.paper_cpu.{scenario}",
            errors,
        )
        if demo_contract is None or best_contract is None or paper_contract is None:
            continue
        normalized_demo = dict(demo_contract)
        normalized_best = dict(best_contract)
        normalized_paper = dict(paper_contract)
        normalized_demo.pop("binary_binding", None)
        normalized_best.pop("binary_binding", None)
        normalized_paper.pop("binary_binding", None)
        if normalized_demo != normalized_best:
            errors.append(
                f"{location}.{scenario}: demo/current physics contracts differ"
            )
        normalized_best.pop("solver", None)
        normalized_paper.pop("solver", None)
        if normalized_best != normalized_paper:
            errors.append(
                f"{location}.{scenario}: diagnostic lane changed non-solver physics"
            )


def _validate_author_card_house_v1_profile(
    profiles: dict[str, Any], errors: list[str]
) -> None:
    location = f"configuration_profiles.{AUTHOR_CARD_HOUSE_V1_PROFILE}"
    profile = profiles.get(AUTHOR_CARD_HOUSE_V1_PROFILE)
    _expect_exact_payload(
        profile, AUTHOR_CARD_HOUSE_V1_PROFILE_PAYLOAD, location, errors
    )


def _validate_author_card_house_v1_contract(
    contract: dict[str, Any] | None,
    data: dict[str, Any],
    source_hashes: dict[str, str],
    location: str,
    errors: list[str],
) -> None:
    if contract is None:
        return
    expected_keys = {
        "schema_version",
        "kind",
        "author_source",
        "configuration_spec_source_sha256",
        "binary_binding",
        "source_configuration",
        "dart_observation",
        "adapter_boundaries",
        "claim_boundary",
        "visual_style",
    }
    actual_keys = set(contract)
    if actual_keys != expected_keys:
        missing = sorted(expected_keys - actual_keys)
        unexpected = sorted(actual_keys - expected_keys)
        if missing:
            errors.append(f"{location}: missing keys {missing}")
        if unexpected:
            errors.append(f"{location}: unexpected keys {unexpected}")
    _expect_fields(
        contract,
        {
            "schema_version": ("dart.fbf_author_card_house_configuration_contract/v1"),
            "kind": "configuration_only",
        },
        location,
        errors,
    )
    _expect_exact_payload(
        contract.get("author_source"),
        AUTHOR_CARD_HOUSE_V1_AUTHOR,
        f"{location}.author_source",
        errors,
    )
    expected_spec_hash = source_hashes.get("configuration_spec")
    if contract.get("configuration_spec_source_sha256") != expected_spec_hash:
        errors.append(
            f"{location}.configuration_spec_source_sha256: "
            "does not match the captured configuration spec identity"
        )
    _expect_exact_payload(
        contract.get("binary_binding"),
        {
            "role": "dart_demos",
            "implementation_source_sha256": source_hashes.get("demo_source"),
        },
        f"{location}.binary_binding",
        errors,
    )
    _expect_exact_payload(
        contract.get("source_configuration"),
        AUTHOR_CARD_HOUSE_V1_SOURCE_CONFIGURATION,
        f"{location}.source_configuration",
        errors,
    )
    _expect_exact_payload(
        contract.get("adapter_boundaries"),
        {
            "source_contact_gap_recorded_m": 0.005,
            "source_contact_gap_semantics_implemented": False,
            "source_solver_backend_semantics_implemented": False,
        },
        f"{location}.adapter_boundaries",
        errors,
    )
    _expect_exact_payload(
        contract.get("claim_boundary"),
        {
            "construction_only": True,
            "source_geometry_and_initial_pose_port": True,
            "dart_cards_mobile": True,
            "dart_cubes_initially_immobile": True,
            "dynamics_executed": False,
            "configuration_port_valid": True,
            "trajectory_valid": False,
            "solver_valid": False,
            "physical_outcome_valid": False,
            "trajectory_equivalence": False,
            "solver_equivalence": False,
            "physical_outcome_equivalence": False,
            "fig06_parity": False,
            "video06_parity": False,
            "paper_timing_valid": False,
            "timing_comparability": False,
            "renderer_colors_source_parity": False,
        },
        f"{location}.claim_boundary",
        errors,
    )
    _expect_exact_payload(
        contract.get("visual_style"),
        {
            "description": ("restrained alternating paper colors for layer legibility"),
            "renderer_only": True,
        },
        f"{location}.visual_style",
        errors,
    )
    declared_payload_hash = data.get("contract_payload_sha256")
    actual_payload_hash = _payload_sha256(contract)
    if declared_payload_hash != actual_payload_hash:
        errors.append(
            f"{location}: payload digest mismatch; expected "
            f"{declared_payload_hash}, computed {actual_payload_hash}"
        )

    observation = _object(
        contract.get("dart_observation"), f"{location}.dart_observation", errors
    )
    if observation is None:
        return
    expected_observation_keys = {
        "world",
        "collision",
        "solver_adapter",
        "ground",
        "cards",
        "cubes",
    }
    if set(observation) != expected_observation_keys:
        errors.append(
            f"{location}.dart_observation: expected exact keys "
            f"{sorted(expected_observation_keys)}"
        )
    _expect_exact_payload(
        observation.get("world"),
        {
            "time_step_seconds": 1.0 / 240.0,
            "gravity_m_s2": [0, 0, -9.81],
            "simulation_threads": 1,
            "deactivation_enabled": False,
        },
        f"{location}.dart_observation.world",
        errors,
    )
    _expect_exact_payload(
        observation.get("collision"),
        {
            "detector": "native",
            "contact_manifold": "four_point_planar",
            "max_contacts": 4096,
            "max_contacts_per_pair": 4,
        },
        f"{location}.dart_observation.collision",
        errors,
    )
    _expect_exact_payload(
        observation.get("solver_adapter"),
        {
            "type": "exact_fbf",
            "split_impulse_enabled": False,
            "max_outer_iterations": 200,
            "tolerance": 1.0e-6,
            "inner_max_sweeps": 10,
            "inner_tolerance": 1.0e-6,
            "step_size_scale": 10,
            "warm_start_enabled": True,
            "fallback_to_boxed_lcp_enabled": False,
            "projected_gradient_retry_enabled": False,
            "dense_residual_polish_enabled": False,
        },
        f"{location}.dart_observation.solver_adapter",
        errors,
    )
    _expect_exact_payload(
        observation.get("ground"),
        {"shape": "plane", "friction": 0.8, "mobile": False},
        f"{location}.dart_observation.ground",
        errors,
    )

    expected_cards: list[tuple[str, str]] = []
    for level in range(5):
        for tent in range(5 - level):
            if level > 0:
                expected_cards.append((f"bridge_L{level}_T{tent}", "bridge"))
            expected_cards.extend(
                [
                    (f"tent_left_L{level}_T{tent}", "tent_left"),
                    (f"tent_right_L{level}_T{tent}", "tent_right"),
                ]
            )
    cards = observation.get("cards")
    if not isinstance(cards, list) or len(cards) != len(expected_cards):
        errors.append(f"{location}.dart_observation.cards: expected 40 ordered cards")
    else:
        for index, (card, (expected_name, expected_role)) in enumerate(
            zip(cards, expected_cards)
        ):
            card_location = f"{location}.dart_observation.cards[{index}]"
            if not isinstance(card, dict):
                errors.append(f"{card_location}: expected an object")
                continue
            expected_size = (
                [2.5, 1.25, 0.04] if expected_role == "bridge" else [0.04, 1.25, 2.5]
            )
            _expect_fields(
                card,
                {
                    "name": expected_name,
                    "role": expected_role,
                    "size_m": expected_size,
                    "mass_kg": 25,
                    "friction": 0.8,
                    "mobile": True,
                    "initial_linear_velocity_m_s": [0, 0, 0],
                    "initial_angular_velocity_rad_s": [0, 0, 0],
                },
                card_location,
                errors,
            )
            pose = card.get("initial_pose")
            if (
                not isinstance(pose, dict)
                or set(pose) != {"translation", "rotation"}
                or not isinstance(pose.get("translation"), list)
                or len(pose["translation"]) != 3
                or not isinstance(pose.get("rotation"), list)
                or len(pose["rotation"]) != 3
                or any(
                    not isinstance(row, list) or len(row) != 3
                    for row in pose["rotation"]
                )
            ):
                errors.append(f"{card_location}.initial_pose: malformed rigid pose")

    cubes = observation.get("cubes")
    if not isinstance(cubes, list) or len(cubes) != 4:
        errors.append(f"{location}.dart_observation.cubes: expected four cubes")
    else:
        for index, cube in enumerate(cubes):
            cube_location = f"{location}.dart_observation.cubes[{index}]"
            if not isinstance(cube, dict):
                errors.append(f"{cube_location}: expected an object")
                continue
            _expect_fields(
                cube,
                {
                    "name": f"cube_{index}",
                    "role": "cube",
                    "size_m": [0.8, 0.8, 0.8],
                    "mass_kg": 256,
                    "friction": 0.8,
                    "mobile": False,
                    "initial_linear_velocity_m_s": [0, 0, 0],
                    "initial_angular_velocity_rad_s": [0, 0, 0],
                },
                cube_location,
                errors,
            )


def _validate_author_card_house_v1_source_identity(
    data: dict[str, Any],
    metadata: dict[str, Any] | None,
    provenance: dict[str, Any] | None,
    repo_root: Path,
    location: str,
    errors: list[str],
) -> dict[str, str]:
    declared = _object(
        data.get("source_identity_sha256"),
        f"{location}.source_identity_sha256",
        errors,
    )
    if declared is None:
        return {}
    expected_keys = set(AUTHOR_CARD_HOUSE_V1_SOURCE_TARGETS)
    actual_keys = set(declared)
    if actual_keys != expected_keys:
        missing = sorted(expected_keys - actual_keys)
        unexpected = sorted(actual_keys - expected_keys)
        if missing:
            errors.append(
                f"{location}.source_identity_sha256: missing current keys {missing}"
            )
        if unexpected:
            errors.append(
                f"{location}.source_identity_sha256: unexpected current keys "
                f"{unexpected}"
            )
    valid_declared: dict[str, str] = {}
    for key in sorted(expected_keys & actual_keys):
        digest = declared.get(key)
        if not isinstance(digest, str) or not SHA256_PATTERN.fullmatch(digest):
            errors.append(
                f"{location}.source_identity_sha256[{key!r}]: "
                "expected lowercase SHA-256"
            )
        else:
            valid_declared[key] = digest

    identity = None if metadata is None else metadata.get("source_identity")
    if not isinstance(identity, dict):
        errors.append(f"{location}.metadata.source_identity: expected an object")
        return valid_declared
    if set(identity) != expected_keys:
        errors.append(
            f"{location}.metadata.source_identity: exact identity membership changed"
        )
    for key, relative in AUTHOR_CARD_HOUSE_V1_SOURCE_TARGETS.items():
        entry = identity.get(key)
        entry_location = f"{location}.metadata.source_identity[{key!r}]"
        if not isinstance(entry, dict) or set(entry) != {"path", "sha256"}:
            errors.append(f"{entry_location}: malformed source identity")
            continue
        captured_path = entry.get("path")
        suffix = Path(relative).parts
        if (
            not isinstance(captured_path, str)
            or tuple(Path(captured_path).parts[-len(suffix) :]) != suffix
        ):
            errors.append(f"{entry_location}.path: captured path suffix changed")
        if entry.get("sha256") != declared.get(key):
            errors.append(f"{entry_location}.sha256: manifest identity differs")
        if key not in AUTHOR_CARD_HOUSE_V1_CAPTURE_ONLY_SOURCE_KEYS:
            path = _artifact_path(
                relative, f"{entry_location}.current_path", repo_root, errors
            )
            if path is not None and declared.get(key) != _sha256(path):
                errors.append(f"{entry_location}.sha256: current source changed")

    if provenance is not None:
        before = provenance.get("source_identity_before")
        after = provenance.get("source_identity_after")
        if before != identity or after != identity:
            errors.append(
                f"{location}.capture_provenance.source_identity: "
                "before/after identities differ from finalized metadata"
            )
    return valid_declared


def _validate_author_card_house_v1_artifact_index(
    bundle: Path,
    index: dict[str, Any] | None,
    location: str,
    errors: list[str],
) -> None:
    expected_files = set(AUTHOR_CARD_HOUSE_V1_ARTIFACT_PATHS)
    actual_files: set[str] = set()
    actual_directories: set[str] = set()
    for path in bundle.rglob("*"):
        relative = path.relative_to(bundle).as_posix()
        if path.is_symlink():
            errors.append(f"{location}.membership: symlink is forbidden: {relative}")
        elif path.is_file():
            actual_files.add(relative)
        elif path.is_dir():
            actual_directories.add(relative)
    if actual_files != expected_files:
        missing = sorted(expected_files - actual_files)
        unexpected = sorted(actual_files - expected_files)
        if missing:
            errors.append(f"{location}.membership: missing files {missing}")
        if unexpected:
            errors.append(f"{location}.membership: unexpected files {unexpected}")
    expected_directories = {AUTHOR_CARD_HOUSE_V1_CAPTURE}
    if actual_directories != expected_directories:
        errors.append(
            f"{location}.membership: expected directories "
            f"{sorted(expected_directories)}, got {sorted(actual_directories)}"
        )
    if index is None:
        return
    indexed_paths = sorted(expected_files - {"artifact-index.json", "metadata.json"})
    expected_index = {
        "schema_version": ("dart.fbf_author_card_house_construction_artifact_index/v1"),
        "artifact_count": 12,
        "excluded": ["artifact-index.json", "metadata.json"],
        "artifacts": [
            {
                "path": relative,
                "sha256": _sha256(bundle / relative),
                "size_bytes": (bundle / relative).stat().st_size,
            }
            for relative in indexed_paths
            if (bundle / relative).is_file()
        ],
    }
    _expect_exact_payload(index, expected_index, f"{location}.artifact_index", errors)


def _author_card_house_v1_zero_diagnostics() -> dict[str, Any]:
    return {
        "available": True,
        "solver": "ExactCoulombFbfConstraintSolver",
        "iterations": 0,
        "total_iterations": 0,
        "exact_solves": 0,
        "exact_attempts": 0,
        "accepted_at_cap": 0,
        "exact_failures": 0,
        "boxed_lcp_fallbacks": 0,
        "warm_starts": 0,
        "contacts": 0,
        "residual": None,
        "best_residual": None,
        "worst_residual": None,
        "status": "not_run",
        "fbf_status": "not_run",
    }


def _validate_author_card_house_v1_timeline(
    timeline: dict[str, Any] | None,
    standalone_contract: dict[str, Any] | None,
    data: dict[str, Any],
    source_hashes: dict[str, str],
    durable_still_sha256: Any,
    location: str,
    errors: list[str],
) -> None:
    if timeline is None:
        return
    _expect_fields(
        timeline,
        {
            "schema_version": "dart.demos_headless_timeline/v1",
            "scene": AUTHOR_CARD_HOUSE_V1_SCENE,
            "active_scene": AUTHOR_CARD_HOUSE_V1_SCENE,
            "total_steps": 0,
            "completed_steps": 0,
            "width": 1280,
            "height": 720,
            "collision_detector": "native",
            "event_order": "captures_before_actions_at_each_completed_step",
            "actions": [],
            "solver_diagnostics": _author_card_house_v1_zero_diagnostics(),
        },
        location,
        errors,
    )
    physics_contract = _object(
        timeline.get("physics_contract"), f"{location}.physics_contract", errors
    )
    if physics_contract is not None:
        _validate_author_card_house_v1_contract(
            physics_contract,
            data,
            source_hashes,
            f"{location}.physics_contract",
            errors,
        )
        if physics_contract != standalone_contract:
            errors.append(
                f"{location}.physics_contract: differs semantically from the "
                "separately queried contract"
            )
        if standalone_contract is None or _payload_sha256(
            physics_contract
        ) != _payload_sha256(standalone_contract):
            errors.append(
                f"{location}.physics_contract: canonical payload digest differs "
                "from the separately queried contract"
            )
    runtime_command = timeline.get("runtime_command")
    try:
        argv = shlex.split(runtime_command) if isinstance(runtime_command, str) else []
    except ValueError:
        argv = []
    required_pairs = {
        "--scene": AUTHOR_CARD_HOUSE_V1_SCENE,
        "--steps": "0",
        "--width": "1280",
        "--height": "720",
        "--threads": "1",
    }
    for option, expected in required_pairs.items():
        if option not in argv or argv.index(option) + 1 >= len(argv):
            errors.append(f"{location}.runtime_command: missing {option}")
        elif argv[argv.index(option) + 1] != expected:
            errors.append(f"{location}.runtime_command: {option} must be {expected!r}")
    steps = timeline.get("steps")
    if not isinstance(steps, list) or len(steps) != 1:
        errors.append(f"{location}.steps: expected exactly one step-zero record")
    else:
        _expect_exact_payload(
            steps[0],
            {
                "step": 0,
                "sim_time": 0,
                "solver_diagnostics": _author_card_house_v1_zero_diagnostics(),
            },
            f"{location}.steps[0]",
            errors,
        )
    shots = timeline.get("shots")
    if not isinstance(shots, list) or len(shots) != 1:
        errors.append(f"{location}.shots: expected exactly one step-zero shot")
    else:
        shot = shots[0]
        if not isinstance(shot, dict):
            errors.append(f"{location}.shots[0]: expected an object")
        else:
            _expect_fields(
                shot,
                {
                    "sequence": 0,
                    "step": 0,
                    "sim_time": 0,
                    "success": True,
                    "solver_diagnostics": _author_card_house_v1_zero_diagnostics(),
                },
                f"{location}.shots[0]",
                errors,
            )
            expected_suffix = f"/{AUTHOR_CARD_HOUSE_V1_CAPTURE}/frames/step_000000.png"
            if not isinstance(shot.get("path"), str) or not shot["path"].endswith(
                expected_suffix
            ):
                errors.append(f"{location}.shots[0].path: staging capture path changed")
    events = timeline.get("events")
    if not isinstance(events, list) or len(events) != 1:
        errors.append(f"{location}.events: expected exactly one shot event")
    else:
        if not isinstance(events[0], dict):
            errors.append(f"{location}.events[0]: expected an object")
        else:
            _expect_fields(
                events[0],
                {"sequence": 0, "step": 0, "success": True, "type": "shot"},
                f"{location}.events[0]",
                errors,
            )
    if not isinstance(durable_still_sha256, str) or not SHA256_PATTERN.fullmatch(
        durable_still_sha256
    ):
        errors.append(f"{location}: durable still identity is malformed")


def _validate_author_card_house_v1_manual(
    bundle: Path,
    manual: dict[str, Any] | None,
    location: str,
    errors: list[str],
) -> None:
    if manual is None:
        return
    verdicts = {
        "five_level_card_house_configuration_visible": True,
        "four_projectile_cubes_visible": True,
        "cards_and_cubes_are_not_visibly_clipped": True,
        "step_zero_only_confirmed": True,
        "configuration_only_nonparity_boundary_confirmed": True,
    }
    _expect_fields(
        manual,
        {
            "schema_version": (
                "dart.fbf_author_card_house_construction_manual_inspection/v1"
            ),
            "manual_inspected": True,
            "pass": True,
            "verdicts": verdicts,
        },
        location,
        errors,
    )
    expected_paths = [
        f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/construction-step-0.png",
        f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/panel.png",
    ]
    artifacts = manual.get("representative_artifacts")
    if not isinstance(artifacts, list) or len(artifacts) != 2:
        errors.append(f"{location}.representative_artifacts: expected two artifacts")
        return
    if [
        item.get("path") for item in artifacts if isinstance(item, dict)
    ] != expected_paths:
        errors.append(
            f"{location}.representative_artifacts: exact artifact order changed"
        )
    for index, artifact in enumerate(artifacts):
        artifact_location = f"{location}.representative_artifacts[{index}]"
        if not isinstance(artifact, dict) or set(artifact) != {
            "path",
            "sha256",
            "observation",
        }:
            errors.append(f"{artifact_location}: malformed manual artifact")
            continue
        relative = artifact.get("path")
        if not _nonempty_string(relative):
            errors.append(f"{artifact_location}.path: expected a relative path")
            continue
        path = bundle / relative
        if not path.is_file() or path.is_symlink():
            errors.append(f"{artifact_location}.path: artifact is unavailable")
        elif artifact.get("sha256") != _sha256(path):
            errors.append(f"{artifact_location}.sha256: artifact digest changed")
        if not _nonempty_string(artifact.get("observation")):
            errors.append(f"{artifact_location}.observation: required")


def _validate_author_card_house_v1_capture_records(
    bundle: Path,
    data: dict[str, Any],
    metadata: dict[str, Any] | None,
    provenance: dict[str, Any] | None,
    run_summary: dict[str, Any] | None,
    verification: dict[str, Any] | None,
    invocations: dict[str, Any] | None,
    location: str,
    errors: list[str],
) -> None:
    durable_path = f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/construction-step-0.png"
    durable_hash = data.get("durable_still_sha256")
    file_hashes = data.get("artifact_hashes", {})
    if not isinstance(file_hashes, dict):
        file_hashes = {}
    negative = AUTHOR_CARD_HOUSE_V1_NEGATIVE_FLAGS
    if metadata is not None:
        expected_metadata = {
            "schema_version": ("dart.fbf_author_card_house_construction_bundle/v1"),
            "status": "valid_current_source_construction_only",
            "pass": True,
            "requirement_ids": ["fig.06", "video.06_card_house"],
            "artifact_valid": True,
            "configuration_port_valid": True,
            "construction_only": True,
            "actual_simulator": True,
            "manual_configuration_inspection_valid": True,
            "claim_scope": AUTHOR_CARD_HOUSE_V1_CLAIM_SCOPE,
            "claim_boundary": AUTHOR_CARD_HOUSE_V1_CLAIM_BOUNDARY,
            **negative,
        }
        _expect_fields(metadata, expected_metadata, f"{location}.metadata", errors)
        links = {
            "manual_inspection": "manual-inspection.json",
            "run_summary": "run-summary.json",
            "verification": "verification.json",
            "invocations": "invocations.json",
            "report": "REPORT.md",
        }
        for field, path in links.items():
            payload = metadata.get(field)
            if not isinstance(payload, dict):
                errors.append(f"{location}.metadata.{field}: expected an object")
                continue
            _expect_fields(
                payload,
                {"path": path, "sha256": file_hashes.get(path)},
                f"{location}.metadata.{field}",
                errors,
            )
        _expect_fields(
            metadata.get("configuration_contract", {}),
            {
                "schema_version": (
                    "dart.fbf_author_card_house_configuration_contract/v1"
                ),
                "kind": "configuration_only",
                "path": "contract.json",
                "sha256": file_hashes.get("contract.json"),
                "payload_sha256": data.get("contract_payload_sha256"),
                "validated": True,
            },
            f"{location}.metadata.configuration_contract",
            errors,
        )
        _expect_fields(
            metadata.get("artifact_index", {}),
            {
                "path": "artifact-index.json",
                "sha256": file_hashes.get("artifact-index.json"),
                "artifact_count": 12,
                "excluded": ["artifact-index.json", "metadata.json"],
            },
            f"{location}.metadata.artifact_index",
            errors,
        )
        summary = metadata.get("capture_summary")
        if not isinstance(summary, dict):
            errors.append(f"{location}.metadata.capture_summary: expected an object")
        else:
            _expect_fields(
                summary,
                {
                    "schedule_id": AUTHOR_CARD_HOUSE_V1_CAPTURE,
                    "scene_id": AUTHOR_CARD_HOUSE_V1_SCENE,
                    "capture_step": 0,
                    "simulation_substeps_executed": 0,
                    "configuration_only": True,
                    "trajectory_valid": False,
                    "solver_valid": False,
                    "physical_outcome_valid": False,
                },
                f"{location}.metadata.capture_summary",
                errors,
            )
            frame = summary.get("step_zero_frame", {})
            _expect_fields(
                frame,
                {
                    "path": durable_path,
                    "sha256": durable_hash,
                    "durable_promoted_copy": True,
                },
                f"{location}.metadata.capture_summary.step_zero_frame",
                errors,
            )
            timeline = summary.get("timeline", {})
            _expect_fields(
                timeline,
                {
                    "path": f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/timeline.json",
                    "total_steps": 0,
                    "completed_steps": 0,
                    "shot_count": 1,
                    "dynamics_executed": False,
                    "solver_diagnostics": _author_card_house_v1_zero_diagnostics(),
                },
                f"{location}.metadata.capture_summary.timeline",
                errors,
            )

    if provenance is not None:
        _expect_fields(
            provenance,
            {
                "capture_returncode": 0,
                "run_summary_path": "run-summary.json",
                "run_summary_sha256": file_hashes.get("run-summary.json"),
                "run_summary_validated": True,
                "capture_stderr_sha256": hashlib.sha256(b"").hexdigest(),
                "contract_query_returncode": 0,
                "contract_path": "contract.json",
                "contract_sha256": file_hashes.get("contract.json"),
                "contract_payload_sha256_before": data.get("contract_payload_sha256"),
                "contract_payload_sha256_after": data.get("contract_payload_sha256"),
                "contract_stdout_byte_identical_before_after": True,
                "verification_returncode": 0,
                "verification_path": "verification.json",
                "verification_sha256": file_hashes.get("verification.json"),
                "durable_still_path": durable_path,
                "durable_still_sha256": durable_hash,
                "staging_pruned": True,
            },
            f"{location}.capture_provenance",
            errors,
        )
        query = provenance.get("contract_query_argv")
        if not isinstance(query, list) or query[-2:] != [
            "--fbf-author-card-house-contract",
            AUTHOR_CARD_HOUSE_V1_SCENE,
        ]:
            errors.append(
                f"{location}.capture_provenance.contract_query_argv: "
                "runtime contract query changed"
            )

    if run_summary is not None:
        _expect_fields(
            run_summary,
            {
                "schema_version": "dart.fbf_visual_evidence/v1",
                "kind": "capture_run",
                "failures": [],
                "group_outputs": [],
                "group_skips": [],
                "pass": True,
            },
            f"{location}.run_summary",
            errors,
        )
        results = run_summary.get("results")
        if not isinstance(results, list) or len(results) != 1:
            errors.append(f"{location}.run_summary.results: expected one result")
        else:
            result = results[0]
            if not isinstance(result, dict):
                errors.append(f"{location}.run_summary.results[0]: expected an object")
                result = {}
            _expect_fields(
                result,
                {
                    "schema_version": "dart.fbf_visual_evidence/v1",
                    "kind": "capture_result",
                    "actual_simulator": True,
                    "generated_imagery": False,
                    "paper_comparable": False,
                    "automated_semantic_outcome_validated": False,
                    "media_validation": [],
                    "pass": True,
                },
                f"{location}.run_summary.results[0]",
                errors,
            )
            schedule_payload = result.get("schedule")
            if not isinstance(schedule_payload, dict):
                errors.append(
                    f"{location}.run_summary.results[0].schedule: expected an object"
                )
            else:
                _expect_fields(
                    schedule_payload,
                    {
                        "id": AUTHOR_CARD_HOUSE_V1_CAPTURE,
                        "total_steps": 0,
                        "time_step_seconds": 1.0 / 240.0,
                        "capture_steps": [0],
                        "panel_steps": [0],
                        "actions": [],
                        "collision_detector": "native",
                        "collision_detector_override": False,
                        "paper_comparable": False,
                        "generated_imagery_allowed": False,
                    },
                    f"{location}.run_summary.results[0].schedule",
                    errors,
                )
            timeline_validation = result.get("timeline_validation")
            if not isinstance(timeline_validation, dict):
                errors.append(
                    f"{location}.run_summary.results[0].timeline_validation: "
                    "expected an object"
                )
            else:
                _expect_fields(
                    timeline_validation,
                    {
                        "pass": True,
                        "step_count": 1,
                        "shot_count": 1,
                        "action_count": 0,
                        "unique_frame_hashes": 1,
                        "final_solver_diagnostics": (
                            _author_card_house_v1_zero_diagnostics()
                        ),
                    },
                    f"{location}.run_summary.results[0].timeline_validation",
                    errors,
                )
                frame = timeline_validation.get("frames", {}).get("0", {})
                if frame.get("sha256") != durable_hash:
                    errors.append(
                        f"{location}.run_summary.results[0].timeline_validation."
                        "frames[0].sha256: durable still differs from capture"
                    )

    if verification is not None:
        _expect_fields(
            verification,
            {
                "schema_version": "dart.fbf_visual_evidence/v1",
                "kind": "verification",
                "group_outputs": [],
                "pass": True,
            },
            f"{location}.verification",
            errors,
        )
        results = verification.get("results")
        if not isinstance(results, list) or len(results) != 1:
            errors.append(f"{location}.verification.results: expected one result")
        else:
            result = results[0]
            if not isinstance(result, dict):
                errors.append(f"{location}.verification.results[0]: expected an object")
                result = {}
            _expect_fields(
                result,
                {
                    "schedule": AUTHOR_CARD_HOUSE_V1_CAPTURE,
                    "media": [],
                    "metadata_sha256": file_hashes.get(
                        f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/metadata.json"
                    ),
                    "pass": True,
                },
                f"{location}.verification.results[0]",
                errors,
            )
            timeline_result = result.get("timeline", {})
            _expect_fields(
                timeline_result,
                {
                    "pass": True,
                    "step_count": 1,
                    "shot_count": 1,
                    "action_count": 0,
                    "unique_frame_hashes": 1,
                    "final_solver_diagnostics": (
                        _author_card_house_v1_zero_diagnostics()
                    ),
                },
                f"{location}.verification.results[0].timeline",
                errors,
            )

    if invocations is not None:
        _expect_fields(
            invocations,
            {
                "schema_version": (
                    "dart.fbf_author_card_house_construction_invocations/v1"
                )
            },
            f"{location}.invocations",
            errors,
        )
        contract_query = invocations.get("contract_query", {})
        _expect_fields(
            contract_query,
            {
                "returncode": 0,
                "payload_path": "contract.json",
                "payload_sha256": file_hashes.get("contract.json"),
            },
            f"{location}.invocations.contract_query",
            errors,
        )
        argv = contract_query.get("argv")
        if not isinstance(argv, list) or argv[-2:] != [
            "--fbf-author-card-house-contract",
            AUTHOR_CARD_HOUSE_V1_SCENE,
        ]:
            errors.append(f"{location}.invocations.contract_query.argv: query changed")

    durable = bundle / durable_path
    panel = bundle / AUTHOR_CARD_HOUSE_V1_CAPTURE / "panel.png"
    for path, label in ((durable, "durable_still"), (panel, "panel")):
        if not path.is_file() or path.is_symlink() or not _has_image_signature(path):
            errors.append(f"{location}.{label}: expected a regular PNG image")


def _validate_author_card_house_v1_truth(
    current_truth: dict[str, Any], repo_root: Path, errors: list[str]
) -> None:
    location = "current_truth.author_card_house_5_construction_only_v1"
    data = _object(
        current_truth.get("author_card_house_5_construction_only_v1"), location, errors
    )
    if data is None:
        return
    expected_keys = {
        "bundle",
        "status",
        "requirement_ids",
        "artifact_valid",
        "configuration_port_valid",
        "construction_only",
        "dynamics_executed",
        "trajectory_valid",
        "solver_valid",
        "physical_outcome_valid",
        "fig06_parity",
        "video06_parity",
        "paper_timing_valid",
        "paper_comparable",
        "canonical_fig06_deliverable",
        "canonical_video06_deliverable",
        "actual_simulator",
        "generated_imagery",
        "manual_configuration_inspection_valid",
        "author_repository",
        "author_commit",
        "author_tree",
        "levels",
        "cards",
        "cubes",
        "total_steps",
        "completed_steps",
        "capture_steps",
        "panel_steps",
        "artifact_count",
        "indexed_artifact_count",
        "artifact_index_exclusions",
        "contract_payload_sha256",
        "durable_still_path",
        "durable_still_sha256",
        "artifact_hashes",
        "source_identity_sha256",
        "claim_scope",
        "claim_boundary",
    }
    if set(data) != expected_keys:
        missing = sorted(expected_keys - set(data))
        unexpected = sorted(set(data) - expected_keys)
        if missing:
            errors.append(f"{location}: missing keys {missing}")
        if unexpected:
            errors.append(f"{location}: unexpected keys {unexpected}")
    expected_fields = {
        "status": "valid_current_source_construction_only",
        "requirement_ids": ["fig.06", "video.06_card_house"],
        "artifact_valid": True,
        "configuration_port_valid": True,
        "construction_only": True,
        "actual_simulator": True,
        "manual_configuration_inspection_valid": True,
        "author_repository": AUTHOR_CARD_HOUSE_V1_AUTHOR["repository"],
        "author_commit": AUTHOR_CARD_HOUSE_V1_AUTHOR["commit"],
        "author_tree": AUTHOR_CARD_HOUSE_V1_AUTHOR["tree"],
        "levels": 5,
        "cards": 40,
        "cubes": 4,
        "total_steps": 0,
        "completed_steps": 0,
        "capture_steps": [0],
        "panel_steps": [0],
        "artifact_count": 14,
        "indexed_artifact_count": 12,
        "artifact_index_exclusions": ["artifact-index.json", "metadata.json"],
        "durable_still_path": (
            f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/construction-step-0.png"
        ),
        "claim_scope": AUTHOR_CARD_HOUSE_V1_CLAIM_SCOPE,
        "claim_boundary": AUTHOR_CARD_HOUSE_V1_CLAIM_BOUNDARY,
        **AUTHOR_CARD_HOUSE_V1_NEGATIVE_FLAGS,
    }
    _expect_fields(data, expected_fields, location, errors)
    for field in ("contract_payload_sha256", "durable_still_sha256"):
        value = data.get(field)
        if not isinstance(value, str) or not SHA256_PATTERN.fullmatch(value):
            errors.append(f"{location}.{field}: expected lowercase SHA-256")

    bundle = _validate_current_path(
        data,
        "bundle",
        AUTHOR_CARD_HOUSE_V1_BUNDLE,
        location,
        repo_root,
        errors,
        directory=True,
    )
    hashes = _validate_artifact_hashes(
        data,
        AUTHOR_CARD_HOUSE_V1_ARTIFACT_TARGETS,
        location,
        repo_root,
        errors,
    )
    if bundle is None:
        return
    metadata = _read_bundle_json(
        bundle, "metadata.json", f"{location}.metadata", errors
    )
    provenance = _read_bundle_json(
        bundle,
        "capture-provenance.json",
        f"{location}.capture_provenance",
        errors,
    )
    contract = _read_bundle_json(
        bundle, "contract.json", f"{location}.contract", errors
    )
    timeline = _read_bundle_json(
        bundle,
        f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/timeline.json",
        f"{location}.timeline",
        errors,
    )
    manual = _read_bundle_json(
        bundle, "manual-inspection.json", f"{location}.manual_inspection", errors
    )
    index = _read_bundle_json(
        bundle, "artifact-index.json", f"{location}.artifact_index", errors
    )
    run_summary = _read_bundle_json(
        bundle, "run-summary.json", f"{location}.run_summary", errors
    )
    verification = _read_bundle_json(
        bundle, "verification.json", f"{location}.verification", errors
    )
    invocations = _read_bundle_json(
        bundle, "invocations.json", f"{location}.invocations", errors
    )
    source_hashes = _validate_author_card_house_v1_source_identity(
        data, metadata, provenance, repo_root, location, errors
    )
    _validate_author_card_house_v1_contract(
        contract, data, source_hashes, f"{location}.contract", errors
    )
    _validate_author_card_house_v1_timeline(
        timeline,
        contract,
        data,
        source_hashes,
        data.get("durable_still_sha256"),
        f"{location}.timeline",
        errors,
    )
    _validate_author_card_house_v1_manual(
        bundle, manual, f"{location}.manual_inspection", errors
    )
    _validate_author_card_house_v1_artifact_index(bundle, index, location, errors)
    _validate_author_card_house_v1_capture_records(
        bundle,
        data,
        metadata,
        provenance,
        run_summary,
        verification,
        invocations,
        location,
        errors,
    )
    durable_relative = data.get("durable_still_path")
    durable_path = (
        bundle / durable_relative if isinstance(durable_relative, str) else None
    )
    if (
        durable_path is not None
        and durable_path.is_file()
        and hashes.get(f"{AUTHOR_CARD_HOUSE_V1_CAPTURE}/construction-step-0.png")
        != data.get("durable_still_sha256")
    ):
        errors.append(f"{location}.durable_still_sha256: artifact hash differs")


def _validate_current_truth(value: Any, repo_root: Path, errors: list[str]) -> None:
    current_truth = _object(value, "current_truth", errors)
    if current_truth is None:
        return
    _expect_exact_keys(current_truth, CURRENT_TRUTH_KEYS, "current_truth", errors)
    for field in sorted(CURRENT_TRUTH_KEYS & set(current_truth)):
        value = current_truth[field]
        if field in CURRENT_TRUTH_OBJECT_KEYS:
            _validate_closed_object_tree(
                value,
                field,
                f"current_truth.{field}",
                CURRENT_TRUTH_OBJECT_KEYS,
                errors,
                delegated_object_paths=CURRENT_TRUTH_DELEGATED_OBJECT_PATHS,
            )
        elif isinstance(value, (dict, list)):
            errors.append(f"current_truth.{field}: expected a scalar value")
    _validate_prior_source_truth(current_truth, repo_root, errors)
    _validate_card_v2_truth(current_truth, repo_root, errors)
    _validate_cpu_evidence_truth(current_truth, repo_root, errors)
    _validate_current_small_cpu_truth(current_truth, repo_root, errors)
    _validate_author_card_house_v1_truth(current_truth, repo_root, errors)
    _validate_backspin_v3_truth(current_truth, repo_root, errors)
    _validate_turntable_v1_truth(current_truth, repo_root, errors)
    _validate_incline_v1_truth(current_truth, repo_root, errors)
    _validate_painleve_v1_truth(current_truth, repo_root, errors)
    _validate_visual_truth(current_truth, repo_root, errors)
    _validate_impact_v7_truth(current_truth, repo_root, errors)
    _validate_arch101_v5_truth(current_truth, repo_root, errors)


def _expected_kind(requirement_id: str) -> str:
    if requirement_id == "teaser":
        return "teaser"
    if requirement_id.startswith("fig."):
        return "figure"
    if requirement_id.startswith("table."):
        return "table"
    if requirement_id.startswith("large_scale."):
        return "large_scale"
    return "video_segment"


def _validate_painleve_requirement_boundaries(
    by_id: dict[str, dict[str, Any]], errors: list[str]
) -> None:
    expected_current = {
        "fig.05": {
            "trace_csv": (f"{PAINLEVE_V1_BUNDLE}/traces/painleve_mu_0_55.csv"),
            "still_image": f"{PAINLEVE_V1_BUNDLE}/groups/painleve/panel.png",
        },
        "video.05_painleve": {
            "video_clip": f"{PAINLEVE_V1_BUNDLE}/groups/painleve/clip.mp4",
        },
    }
    forbidden_promotions = {
        "external_baseline",
        "approved_golden",
        "golden_diff",
        "capture_sidecar",
        "claim_map",
    }
    for requirement_id, expected_deliverables in expected_current.items():
        requirement = by_id.get(requirement_id)
        if not isinstance(requirement, dict):
            continue
        location = f"{requirement_id}.painleve_current_boundary"
        if requirement.get("status") != "partial":
            errors.append(f"{location}.status: expected 'partial'")
        deliverables = requirement.get("deliverables")
        if not isinstance(deliverables, list):
            continue
        by_kind = {
            item.get("kind"): item for item in deliverables if isinstance(item, dict)
        }
        for kind, path in expected_deliverables.items():
            deliverable = by_kind.get(kind)
            if not isinstance(deliverable, dict):
                errors.append(f"{location}.{kind}: missing current deliverable")
                continue
            _expect_fields(
                deliverable,
                {"path": path, "validated": True},
                f"{location}.{kind}",
                errors,
            )
        for kind in forbidden_promotions:
            deliverable = by_kind.get(kind)
            if isinstance(deliverable, dict) and deliverable.get("validated") is True:
                errors.append(f"{location}.{kind}: promotion remains unsupported")


def _validate_author_card_house_requirement_boundaries(
    by_id: dict[str, dict[str, Any]], errors: list[str]
) -> None:
    bundle_report = f"{AUTHOR_CARD_HOUSE_V1_BUNDLE}/REPORT.md"
    expected = {
        "fig.06": {
            "status": "partial",
            "fallback_count": 24,
            "required_deliverables": [
                "exact_fixture",
                "trace_csv",
                "still_image",
                "video_clip",
                "external_baseline",
                "approved_golden",
                "golden_diff",
                "capture_sidecar",
                "outcome_report",
                "claim_map",
            ],
        },
        "video.06_card_house": {
            "status": "blocked",
            "fallback_count": 24,
            "required_deliverables": [
                "exact_fixture",
                "video_clip",
                "external_baseline",
                "capture_sidecar",
                "approved_golden",
                "golden_diff",
                "outcome_report",
                "claim_map",
            ],
        },
    }
    for requirement_id, expected_fields in expected.items():
        requirement = by_id.get(requirement_id)
        if not isinstance(requirement, dict):
            continue
        location = f"{requirement_id}.author_card_house_current_boundary"
        _expect_fields(requirement, expected_fields, location, errors)
        _expect_exact_payload(
            requirement.get("configuration"),
            {"profile": "card_house_26"},
            f"{location}.configuration",
            errors,
        )
        if requirement.get("deliverables") != []:
            errors.append(
                f"{location}.deliverables: canonical deliverables must remain empty"
            )
        evidence = requirement.get("current_evidence")
        evidence_paths = (
            {item.get("path") for item in evidence if isinstance(item, dict)}
            if isinstance(evidence, list)
            else set()
        )
        if bundle_report not in evidence_paths:
            errors.append(
                f"{location}.current_evidence: missing construction-only report"
            )
        command_lists = [requirement.get("commands")]
        capture_plan = requirement.get("capture_plan")
        if isinstance(capture_plan, dict):
            command_lists.append(capture_plan.get("commands"))
        commands = [
            command
            for command_list in command_lists
            if isinstance(command_list, list)
            for command in command_list
            if isinstance(command, str)
        ]
        if not any(
            "finalize_fbf_author_card_house_construction.py --verify-only" in command
            for command in commands
        ):
            errors.append(f"{location}.commands: missing bundle verification command")
        blockers = requirement.get("blockers")
        capture_blockers = (
            capture_plan.get("blockers") if isinstance(capture_plan, dict) else []
        )
        blocker_text = " ".join(
            item
            for values in (blockers, capture_blockers)
            if isinstance(values, list)
            for item in values
            if isinstance(item, str)
        ).lower()
        for phrase in (
            "five-level",
            "40-card",
            "four-level",
            "26-card",
            "zero",
        ):
            if phrase not in blocker_text:
                errors.append(
                    f"{location}.blockers: missing construction boundary {phrase!r}"
                )


def _validate_incline_requirement_boundaries(
    by_id: dict[str, dict[str, Any]], errors: list[str]
) -> None:
    expected_current = {
        "fig.01": {
            "exact_fixture": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
            "trace_csv": f"{INCLINE_V1_BUNDLE}/traces/incline_mu_0_4.csv",
        },
        "fig.02": {
            "exact_fixture": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
            "trace_csv": f"{INCLINE_V1_BUNDLE}/traces/incline_mu_0_5.csv",
            "still_image": f"{INCLINE_V1_BUNDLE}/incline/panel.png",
        },
        "video.03_incline": {
            "exact_fixture": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
            "video_clip": f"{INCLINE_V1_BUNDLE}/incline/clip.mp4",
        },
    }
    forbidden_promotions = {
        "external_baseline",
        "approved_golden",
        "golden_diff",
        "capture_sidecar",
        "claim_map",
    }
    for requirement_id, expected_deliverables in expected_current.items():
        requirement = by_id.get(requirement_id)
        if not isinstance(requirement, dict):
            continue
        location = f"{requirement_id}.incline_current_boundary"
        if requirement.get("status") != "partial":
            errors.append(f"{location}.status: expected 'partial'")
        if requirement.get("fallback_count") != 0:
            errors.append(f"{location}.fallback_count: expected 0")
        deliverables = requirement.get("deliverables")
        if not isinstance(deliverables, list):
            continue
        by_kind = {
            item.get("kind"): item for item in deliverables if isinstance(item, dict)
        }
        for kind, path in expected_deliverables.items():
            deliverable = by_kind.get(kind)
            if not isinstance(deliverable, dict):
                errors.append(f"{location}.{kind}: missing current deliverable")
                continue
            _expect_fields(
                deliverable,
                {"path": path, "validated": True},
                f"{location}.{kind}",
                errors,
            )
        promotions = set(forbidden_promotions)
        if requirement_id == "fig.01":
            promotions.add("comparison_plot")
        for kind in promotions:
            deliverable = by_kind.get(kind)
            if isinstance(deliverable, dict) and deliverable.get("validated") is True:
                errors.append(f"{location}.{kind}: promotion remains unsupported")


def _validate_backspin_requirement_boundaries(
    by_id: dict[str, dict[str, Any]], errors: list[str]
) -> None:
    expected_current = {
        "fig.03": {
            "exact_fixture": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
            "trace_csv": f"{BACKSPIN_V3_BUNDLE}/traces/backspin.csv",
            "still_image": f"{BACKSPIN_V3_BUNDLE}/backspin/panel.png",
        },
        "video.02_backspin": {
            "exact_fixture": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
            "video_clip": f"{BACKSPIN_V3_BUNDLE}/backspin/clip.mp4",
        },
    }
    forbidden_promotions = {
        "external_baseline",
        "approved_golden",
        "golden_diff",
        "capture_sidecar",
        "claim_map",
    }
    for requirement_id, expected_deliverables in expected_current.items():
        requirement = by_id.get(requirement_id)
        if not isinstance(requirement, dict):
            continue
        location = f"{requirement_id}.backspin_current_boundary"
        if requirement.get("status") != "partial":
            errors.append(f"{location}.status: expected 'partial'")
        if requirement.get("fallback_count") != 0:
            errors.append(f"{location}.fallback_count: expected 0")
        deliverables = requirement.get("deliverables")
        if not isinstance(deliverables, list):
            continue
        by_kind = {
            item.get("kind"): item for item in deliverables if isinstance(item, dict)
        }
        for kind, path in expected_deliverables.items():
            deliverable = by_kind.get(kind)
            if not isinstance(deliverable, dict):
                errors.append(f"{location}.{kind}: missing current deliverable")
                continue
            _expect_fields(
                deliverable,
                {"path": path, "validated": True},
                f"{location}.{kind}",
                errors,
            )
        for kind in forbidden_promotions:
            deliverable = by_kind.get(kind)
            if isinstance(deliverable, dict) and deliverable.get("validated") is True:
                errors.append(f"{location}.{kind}: promotion remains unsupported")


def _validate_turntable_requirement_boundaries(
    by_id: dict[str, dict[str, Any]], errors: list[str]
) -> None:
    expected_current = {
        "fig.04": {
            "exact_fixture": "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp",
            "trace_csv": (
                f"{TURNTABLE_V1_BUNDLE}/traces/current_visual/"
                "turntable_author_mu_0_5_omega_2.csv"
            ),
            "still_image": (f"{TURNTABLE_V1_BUNDLE}/groups/turntable_author/panel.png"),
        },
        "video.04_turntable": {
            "exact_fixture": "examples/demos/scenes/FbfPaperFrictionScene.cpp",
            "video_clip": (f"{TURNTABLE_V1_BUNDLE}/groups/turntable_author/clip.mp4"),
        },
    }
    forbidden_promotions = {
        "external_baseline",
        "approved_golden",
        "golden_diff",
        "capture_sidecar",
        "claim_map",
    }
    for requirement_id, expected_deliverables in expected_current.items():
        requirement = by_id.get(requirement_id)
        if not isinstance(requirement, dict):
            continue
        location = f"{requirement_id}.turntable_current_boundary"
        if requirement.get("status") != "partial":
            errors.append(f"{location}.status: expected 'partial'")
        if requirement.get("fallback_count") != 0:
            errors.append(f"{location}.fallback_count: expected 0")
        claim = requirement.get("claim")
        if isinstance(claim, str) and any(
            phrase in claim.lower()
            for phrase in ("zero slip", "zero-slip", "co-rotation", "corotation")
        ):
            errors.append(f"{location}.claim: retained case cannot claim zero slip")
        deliverables = requirement.get("deliverables")
        if not isinstance(deliverables, list):
            continue
        by_kind = {
            item.get("kind"): item for item in deliverables if isinstance(item, dict)
        }
        for kind, path in expected_deliverables.items():
            deliverable = by_kind.get(kind)
            if not isinstance(deliverable, dict):
                errors.append(f"{location}.{kind}: missing current deliverable")
                continue
            _expect_fields(
                deliverable,
                {"path": path, "validated": True},
                f"{location}.{kind}",
                errors,
            )
        for kind in forbidden_promotions:
            deliverable = by_kind.get(kind)
            if isinstance(deliverable, dict) and deliverable.get("validated") is True:
                errors.append(f"{location}.{kind}: promotion remains unsupported")


def validate_manifest(data: Any, repo_root: Path = REPO_ROOT) -> list[str]:
    """Return every validation error found in *data*."""

    errors: list[str] = []
    if not isinstance(data, dict):
        return ["manifest: expected a JSON object"]
    _expect_exact_keys(data, MANIFEST_KEYS, "manifest", errors)
    for field in ("completion_rule", "snapshot_date"):
        if not _nonempty_string(data.get(field)):
            errors.append(f"{field}: expected a non-empty string")

    if data.get("schema_version") != SCHEMA_VERSION:
        errors.append(
            "schema_version: expected "
            f"{SCHEMA_VERSION!r}, got {data.get('schema_version')!r}"
        )
    if data.get("task_id") != "fbf_exact_coulomb_friction":
        errors.append("task_id: expected 'fbf_exact_coulomb_friction'")

    overall_status = data.get("overall_status")
    if not isinstance(overall_status, str) or overall_status not in {
        "partial",
        "complete",
    }:
        errors.append("overall_status: expected 'partial' or 'complete'")

    _validate_current_truth(data.get("current_truth"), repo_root, errors)

    sources = data.get("sources")
    if not isinstance(sources, dict):
        errors.append("sources: expected an object")
    else:
        _expect_exact_keys(sources, SOURCE_CATALOG_KEYS, "sources", errors)
        for source_id, expected_keys in SOURCE_RECORD_KEYS.items():
            source = sources.get(source_id)
            if not isinstance(source, dict):
                errors.append(f"sources.{source_id}: expected an object")
                continue
            _expect_exact_keys(
                source,
                expected_keys,
                f"sources.{source_id}",
                errors,
            )
            for field in expected_keys & set(source):
                if source_id == "author_code" and field in {
                    "scene_source_sha256",
                    "solver_source_sha256",
                }:
                    continue
                if isinstance(source[field], (dict, list)):
                    errors.append(
                        f"sources.{source_id}.{field}: expected a scalar value"
                    )
            if not _nonempty_string(source.get("url")):
                errors.append(f"sources.{source_id}.url: expected a non-empty URL")
        author_code = sources.get("author_code")
        if isinstance(author_code, dict):
            scene_hashes = author_code.get("scene_source_sha256")
            if not isinstance(scene_hashes, dict):
                errors.append(
                    "sources.author_code.scene_source_sha256: expected an object"
                )
            else:
                _expect_exact_keys(
                    scene_hashes,
                    AUTHOR_SCENE_SOURCE_KEYS,
                    "sources.author_code.scene_source_sha256",
                    errors,
                )
                for field in AUTHOR_SCENE_SOURCE_KEYS & set(scene_hashes):
                    if isinstance(scene_hashes[field], (dict, list)):
                        errors.append(
                            "sources.author_code.scene_source_sha256"
                            f".{field}: expected a scalar value"
                        )
            solver_hashes = author_code.get("solver_source_sha256")
            if not isinstance(solver_hashes, dict):
                errors.append(
                    "sources.author_code.solver_source_sha256: expected an object"
                )
            else:
                _expect_exact_keys(
                    solver_hashes,
                    AUTHOR_SOLVER_SOURCE_KEYS,
                    "sources.author_code.solver_source_sha256",
                    errors,
                )
                for field in AUTHOR_SOLVER_SOURCE_KEYS & set(solver_hashes):
                    if isinstance(solver_hashes[field], (dict, list)):
                        errors.append(
                            "sources.author_code.solver_source_sha256"
                            f".{field}: expected a scalar value"
                        )
        video = sources.get("video")
        if isinstance(video, dict) and video.get("duration_seconds") != 82:
            errors.append("sources.video.duration_seconds: expected 82")
        pr3374 = sources.get("pr3374")
        if isinstance(pr3374, dict) and pr3374.get("status") != "integrated":
            errors.append("sources.pr3374.status: expected 'integrated'")

    source_audit = data.get("source_audit")
    if not isinstance(source_audit, dict):
        errors.append("source_audit: expected an object")
    else:
        _expect_exact_keys(source_audit, SOURCE_AUDIT_KEYS, "source_audit", errors)
        for field in SOURCE_AUDIT_KEYS & set(source_audit):
            value = source_audit[field]
            if field == "coverage_seconds":
                if not isinstance(value, list) or any(
                    isinstance(item, (dict, list)) for item in value
                ):
                    errors.append("source_audit.coverage_seconds: expected a flat list")
            elif isinstance(value, (dict, list)):
                errors.append(f"source_audit.{field}: expected a scalar value")

    profiles = data.get("configuration_profiles")
    if not isinstance(profiles, dict) or not profiles:
        errors.append("configuration_profiles: expected a non-empty object")
        profiles = {}
    else:
        _expect_exact_keys(
            profiles,
            CONFIGURATION_PROFILE_KEYS,
            "configuration_profiles",
            errors,
        )
        for profile_id in sorted(CONFIGURATION_PROFILE_KEYS & set(profiles)):
            profile = profiles[profile_id]
            if not isinstance(profile, dict):
                errors.append(
                    f"configuration_profiles.{profile_id}: expected an object"
                )
                continue
            _validate_configuration_profile_key_contract(profile_id, profile, errors)
    _validate_author_card_house_v1_profile(profiles, errors)

    capture_profiles = data.get("capture_profiles")
    if not isinstance(capture_profiles, dict) or not capture_profiles:
        errors.append("capture_profiles: expected a non-empty object")
        capture_profiles = {}
    else:
        _expect_exact_keys(
            capture_profiles,
            CAPTURE_PROFILE_KEYS,
            "capture_profiles",
            errors,
        )
        for profile_id in sorted(CAPTURE_PROFILE_KEYS & set(capture_profiles)):
            profile = capture_profiles[profile_id]
            if not isinstance(profile, dict):
                errors.append(f"capture_profiles.{profile_id}: expected an object")
                continue
            _expect_exact_keys(
                profile,
                CAPTURE_PROFILE_RECORD_KEYS,
                f"capture_profiles.{profile_id}",
                errors,
            )
            for field in CAPTURE_PROFILE_RECORD_KEYS & set(profile):
                value = profile[field]
                location = f"capture_profiles.{profile_id}.{field}"
                if field == "toolchain":
                    if not isinstance(value, list) or any(
                        isinstance(item, (dict, list)) for item in value
                    ):
                        errors.append(f"{location}: expected a flat list")
                elif isinstance(value, (dict, list)):
                    errors.append(f"{location}: expected a scalar value")

    requirements = data.get("requirements")
    if not isinstance(requirements, list):
        errors.append("requirements: expected a list")
        return errors

    ids: list[str] = []
    statuses: list[str] = []
    by_id: dict[str, dict[str, Any]] = {}
    for index, requirement in enumerate(requirements):
        location = f"requirements[{index}]"
        if not isinstance(requirement, dict):
            errors.append(f"{location}: expected an object")
            continue
        _expect_exact_keys(requirement, REQUIREMENT_KEYS, location, errors)

        requirement_id = requirement.get("id")
        if not _nonempty_string(requirement_id):
            errors.append(f"{location}.id: expected a non-empty string")
            continue
        ids.append(requirement_id)
        by_id.setdefault(requirement_id, requirement)

        expected_kind = _expected_kind(requirement_id)
        kind = requirement.get("kind")
        if not isinstance(kind, str) or kind not in ALLOWED_KINDS:
            errors.append(f"{location}.kind: unsupported kind {kind!r}")
        elif kind != expected_kind:
            errors.append(
                f"{location}.kind: {requirement_id} must use {expected_kind!r}"
            )

        if not _nonempty_string(requirement.get("title")):
            errors.append(f"{location}.title: expected a non-empty string")

        source = requirement.get("source")
        if not isinstance(source, dict):
            errors.append(f"{location}.source: expected an object")
        else:
            expected_source_keys = (
                VIDEO_REQUIREMENT_SOURCE_KEYS
                if requirement_id in VIDEO_SEGMENTS
                else REQUIREMENT_SOURCE_KEYS
            )
            _expect_exact_keys(
                source,
                expected_source_keys,
                f"{location}.source",
                errors,
            )
            for field in ("url", "locator"):
                if not _nonempty_string(source.get(field)):
                    errors.append(
                        f"{location}.source.{field}: expected a non-empty string"
                    )

        if not _nonempty_string(requirement.get("claim")):
            errors.append(f"{location}.claim: expected a non-empty string")

        configuration = requirement.get("configuration")
        if not isinstance(configuration, dict) or not configuration:
            errors.append(f"{location}.configuration: expected a non-empty object")
        else:
            expected_configuration_keys = REQUIREMENT_CONFIGURATION_KEYS.get(
                requirement_id
            )
            if expected_configuration_keys is not None:
                _expect_exact_keys(
                    configuration,
                    expected_configuration_keys,
                    f"{location}.configuration",
                    errors,
                )
                for field in expected_configuration_keys & set(configuration):
                    value = configuration[field]
                    field_location = f"{location}.configuration.{field}"
                    if field in REQUIREMENT_CONFIGURATION_LIST_FIELDS:
                        if not isinstance(value, list) or any(
                            isinstance(item, (dict, list)) for item in value
                        ):
                            errors.append(f"{field_location}: expected a flat list")
                    elif isinstance(value, (dict, list)):
                        errors.append(f"{field_location}: expected a scalar value")
            profile = configuration.get("profile")
            if not _nonempty_string(profile) or profile not in profiles:
                errors.append(
                    f"{location}.configuration.profile: unknown profile {profile!r}"
                )

        required_deliverables = _check_string_list(
            requirement.get("required_deliverables"),
            f"{location}.required_deliverables",
            errors,
        )
        if len(required_deliverables) != len(set(required_deliverables)):
            errors.append(f"{location}.required_deliverables: duplicate kind")
        for deliverable in required_deliverables:
            if deliverable not in ALLOWED_DELIVERABLES:
                errors.append(
                    f"{location}.required_deliverables: unsupported kind "
                    f"{deliverable!r}"
                )

        _check_string_list(
            requirement.get("expected_behavior"),
            f"{location}.expected_behavior",
            errors,
        )

        status = requirement.get("status")
        if not isinstance(status, str) or status not in ALLOWED_STATUSES:
            errors.append(f"{location}.status: unsupported status {status!r}")
        else:
            statuses.append(status)

        current_evidence = requirement.get("current_evidence")
        if not isinstance(current_evidence, list):
            errors.append(f"{location}.current_evidence: expected a list")
            current_evidence = []
        for evidence_index, evidence in enumerate(current_evidence):
            evidence_location = f"{location}.current_evidence[{evidence_index}]"
            if not isinstance(evidence, dict):
                errors.append(f"{evidence_location}: expected an object")
                continue
            _expect_exact_keys(
                evidence,
                CURRENT_EVIDENCE_KEYS,
                evidence_location,
                errors,
            )
            _artifact_path(
                evidence.get("path"),
                f"{evidence_location}.path",
                repo_root,
                errors,
            )
            if not _nonempty_string(evidence.get("supports")):
                errors.append(
                    f"{evidence_location}.supports: expected a non-empty string"
                )
        if status == "partial" and not current_evidence:
            errors.append(f"{location}: partial status requires current_evidence")

        capture_plan = requirement.get("capture_plan")
        capture_blockers: list[str] = []
        if not isinstance(capture_plan, dict):
            errors.append(f"{location}.capture_plan: expected an object")
        else:
            _expect_exact_keys(
                capture_plan,
                CAPTURE_PLAN_KEYS,
                f"{location}.capture_plan",
                errors,
            )
            capture_profile = capture_plan.get("profile")
            if (
                not _nonempty_string(capture_profile)
                or capture_profile not in capture_profiles
            ):
                errors.append(
                    f"{location}.capture_plan.profile: unknown profile "
                    f"{capture_profile!r}"
                )
            _check_string_list(
                capture_plan.get("shots"),
                f"{location}.capture_plan.shots",
                errors,
            )
            capture_commands = _check_string_list(
                capture_plan.get("commands"),
                f"{location}.capture_plan.commands",
                errors,
                allow_empty=True,
            )
            if status in ("partial", "complete") and not capture_commands:
                errors.append(
                    f"{location}.capture_plan.commands: {status} status requires "
                    "at least one reproduction command"
                )
            capture_blockers = _check_string_list(
                capture_plan.get("blockers"),
                f"{location}.capture_plan.blockers",
                errors,
                allow_empty=True,
            )

        commands = _check_string_list(
            requirement.get("commands"),
            f"{location}.commands",
            errors,
            allow_empty=True,
        )
        if status in ("partial", "complete") and not commands:
            errors.append(f"{location}.commands: {status} status requires a command")

        delivered = requirement.get("deliverables")
        if not isinstance(delivered, list):
            errors.append(f"{location}.deliverables: expected a list")
            delivered = []
        delivered_kinds: set[str] = set()
        delivered_paths: set[Path] = set()
        all_deliverables_validated = True
        for deliverable_index, deliverable in enumerate(delivered):
            deliverable_location = f"{location}.deliverables[{deliverable_index}]"
            if not isinstance(deliverable, dict):
                errors.append(f"{deliverable_location}: expected an object")
                all_deliverables_validated = False
                continue
            validated = deliverable.get("validated")
            if validated is True:
                expected_deliverable_keys = VALIDATED_DELIVERABLE_KEYS
            elif validated is False:
                expected_deliverable_keys = STALE_DELIVERABLE_KEYS
                if not _nonempty_string(deliverable.get("stale_reason")):
                    errors.append(
                        f"{deliverable_location}.stale_reason: expected a non-empty "
                        "string"
                    )
            else:
                expected_deliverable_keys = VALIDATED_DELIVERABLE_KEYS
                errors.append(f"{deliverable_location}.validated: expected a boolean")
            _expect_exact_keys(
                deliverable,
                expected_deliverable_keys,
                deliverable_location,
                errors,
            )
            deliverable_kind = deliverable.get("kind")
            if deliverable_kind not in required_deliverables:
                errors.append(f"{deliverable_location}.kind: not declared as required")
            elif deliverable_kind in delivered_kinds:
                errors.append(f"{deliverable_location}.kind: duplicate delivered kind")
            else:
                delivered_kinds.add(deliverable_kind)
            deliverable_path = _artifact_path(
                deliverable.get("path"),
                f"{deliverable_location}.path",
                repo_root,
                errors,
            )
            parsed_deliverable = None
            if deliverable_path is not None:
                parsed_deliverable = _validate_deliverable_type(
                    deliverable_kind,
                    deliverable_path,
                    f"{deliverable_location}.path",
                    errors,
                )
                if status == "complete":
                    if deliverable_path in delivered_paths:
                        errors.append(
                            f"{deliverable_location}.path: complete deliverables "
                            "must use distinct artifacts"
                        )
                    delivered_paths.add(deliverable_path)
            is_validated = deliverable.get("validated") is True
            if not is_validated:
                all_deliverables_validated = False
            if (
                is_validated
                and isinstance(deliverable_kind, str)
                and deliverable_kind in ALLOWED_DELIVERABLES
            ):
                _validate_validated_deliverable(
                    deliverable_kind,
                    deliverable,
                    deliverable_path,
                    parsed_deliverable,
                    requirement_id,
                    requirement.get("claim", ""),
                    deliverable_location,
                    repo_root,
                    status == "complete",
                    errors,
                )

        fallback_count = requirement.get("fallback_count")
        if fallback_count is not None and (
            isinstance(fallback_count, bool)
            or not isinstance(fallback_count, int)
            or fallback_count < 0
        ):
            errors.append(
                f"{location}.fallback_count: expected null or a non-negative integer"
            )

        blockers = _check_string_list(
            requirement.get("blockers"),
            f"{location}.blockers",
            errors,
            allow_empty=True,
        )
        if status == "blocked" and not blockers:
            errors.append(f"{location}: blocked status requires a blocker")

        if status == "complete":
            missing = set(required_deliverables) - delivered_kinds
            if missing:
                errors.append(
                    f"{location}: complete status is missing deliverables "
                    f"{sorted(missing)}"
                )
            if not all_deliverables_validated:
                errors.append(
                    f"{location}: complete status requires validated deliverables"
                )
            if fallback_count != 0:
                errors.append(
                    f"{location}: complete status requires fallback_count == 0"
                )
            if blockers or capture_blockers:
                errors.append(f"{location}: complete status cannot retain blockers")

    duplicates = sorted({item for item in ids if ids.count(item) > 1})
    if duplicates:
        errors.append(f"requirements: duplicate ids {duplicates}")

    canonical = set(CANONICAL_REQUIREMENT_IDS)
    actual = set(ids)
    missing = sorted(canonical - actual)
    extra = sorted(actual - canonical)
    if missing:
        errors.append(f"requirements: missing canonical ids {missing}")
    if extra:
        errors.append(f"requirements: unexpected ids {extra}")

    _validate_backspin_requirement_boundaries(by_id, errors)
    _validate_painleve_requirement_boundaries(by_id, errors)
    current_truth = data.get("current_truth")
    if (
        isinstance(current_truth, dict)
        and "incline_visual_v1_nonpaper" in current_truth
    ):
        _validate_incline_requirement_boundaries(by_id, errors)
    if isinstance(current_truth, dict) and (
        "turntable_author_visual_v1_nonpaper" in current_truth
    ):
        _validate_turntable_requirement_boundaries(by_id, errors)
    if isinstance(current_truth, dict) and (
        "author_card_house_5_construction_only_v1" in current_truth
    ):
        _validate_author_card_house_requirement_boundaries(by_id, errors)

    for requirement_id, expected_range in VIDEO_SEGMENTS.items():
        requirement = by_id.get(requirement_id)
        if not isinstance(requirement, dict):
            continue
        source = requirement.get("source")
        if not isinstance(source, dict):
            continue
        actual_range = (source.get("start_seconds"), source.get("end_seconds"))
        if actual_range != expected_range:
            errors.append(
                f"{requirement_id}.source: expected timeline {expected_range}, "
                f"got {actual_range}"
            )

    ordered_ranges = [VIDEO_SEGMENTS[item] for item in VIDEO_SEGMENTS]
    for previous, following in zip(ordered_ranges, ordered_ranges[1:]):
        if previous[1] != following[0]:
            errors.append("validator: canonical video segments are not contiguous")
    if ordered_ranges[0][0] != 0 or ordered_ranges[-1][1] != 82:
        errors.append("validator: canonical video segments must cover 0 through 82")

    all_complete = bool(statuses) and all(status == "complete" for status in statuses)
    if overall_status == "complete" and not all_complete:
        errors.append(
            "overall_status: complete requires every requirement to be complete"
        )
    if overall_status == "partial" and all_complete:
        errors.append(
            "overall_status: partial is inconsistent with all requirements complete"
        )

    return errors


def load_manifest(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "manifest",
        nargs="?",
        type=Path,
        default=DEFAULT_MANIFEST,
        help=f"manifest path (default: {DEFAULT_MANIFEST.relative_to(REPO_ROOT)})",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=REPO_ROOT,
        help="repository root used to resolve local evidence paths",
    )
    args = parser.parse_args(argv)

    try:
        manifest = load_manifest(args.manifest)
    except (OSError, json.JSONDecodeError) as error:
        print(f"paper-evidence manifest could not be read: {error}", file=sys.stderr)
        return 2

    errors = validate_manifest(manifest, args.repo_root)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        print(
            f"paper-evidence manifest invalid: {len(errors)} error(s)",
            file=sys.stderr,
        )
        return 1

    print(
        "paper-evidence manifest valid: "
        f"{len(manifest['requirements'])} canonical requirements, "
        f"status={manifest['overall_status']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
