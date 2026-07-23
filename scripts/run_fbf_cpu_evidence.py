#!/usr/bin/env python3
"""Collect reproducible full-trajectory CPU evidence for FBF paper scenes.

The C++ ``fbf_paper_trace`` executable owns scene construction, physics, and
per-step timing. This wrapper repeats complete trajectories, preserves every
raw CSV row, records the machine/build context, and writes aggregate summaries.
Raw timing is always retained, but realtime and paper-target verdicts remain
unevaluated until their explicit trajectory, physical-outcome, convergence,
fallback, threading, affinity, scene, precision, frontend, and hardware
contracts are satisfied.
Evidence schema v8 preserves the v7 default trace and adds an opt-in extended
schema for DART's non-paper deterministic manifold-colored inner BGS. The
extended fields keep colored scheduling and CPU residency distinct from the
contact-row ``W * x`` kernel and add whole-arch metrics against the constructed
initial pose. Evidence schema v7 appended exact-kernel
runtime CPU-residency observations plus reconstruction-specific card-scene
metrics. Dispatch counters distinguish configured world threads from actual
multi-participant ``W * x`` work. Runtime CPU IDs are residency observations;
only the per-phase set can support a multicore claim, and it is not a proof of
perfect simultaneity.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
import platform
import re
import shutil
import statistics
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

SCHEMA_VERSION = 8

TRACE_COLUMNS_V4 = (
    "step",
    "time",
    "scenario",
    "solver",
    "solver_contract",
    "precision_contract",
    "scene_contract",
    "baumgarte_contract",
    "collision_frontend",
    "inner_local_solver",
    "inner_sweeps_requested",
    "fixed_inner_sweeps_requested",
    "step_size_persistence_enabled",
    "step_size_recovery_growth_factor",
    "step_size_persistence_used",
    "step_size_persistence_request",
    "row_operator_request",
    "row_operator_mode",
    "wall_ms",
    "requested_threads",
    "actual_threads",
    "contacts",
    "unique_colliding_body_pairs",
    "penetration_depth_min",
    "penetration_depth_median",
    "penetration_depth_p95",
    "penetration_depth_max",
    "exact_diagnostics_contract",
    "step_exact_solves",
    "step_warm_starts",
    "step_exact_failures",
    "step_fallbacks",
    "step_fbf_iterations",
    "residual",
    "status",
    "residual_primal_feasibility",
    "residual_dual_feasibility",
    "residual_complementarity",
    "accepted_gamma",
    "safe_gamma",
    "shrink_iterations",
    "coupling_variation_ratio",
    "warm_start_matched_contacts",
    "warm_start_matched_fraction",
    "phase",
    "card_count",
    "projectile_count",
    "finite_state",
    "min_card_axis_up",
    "min_center_height",
    "max_card_horizontal_travel",
    "max_projectile_speed",
    "tracked_body",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "up_z",
)

TRACE_CONTRACT_COLUMNS_V5 = (
    "max_outer_iterations",
    "tolerance",
    "accept_outer_max_iterations",
    "inner_local_iterations",
    "adaptive_step_size_enabled",
    "warm_start_enabled",
    "projected_gradient_retry_enabled",
    "dense_residual_polish_enabled",
    "fallback_to_boxed_lcp_enabled",
    "diagonal_seed_enabled",
    "matrix_free_seed_enabled",
    "step_size_scale",
    "outer_relaxation",
    "initial_gamma_contract",
    "split_impulse_enabled",
)

TRACE_PARALLEL_COLUMNS_V6 = (
    "step_contact_row_delassus_products",
    "step_parallel_contact_row_delassus_products",
    "max_contact_row_participants_to_date",
)

TRACE_RESIDENCY_AND_CARD_COLUMNS_V7 = (
    "exact_contact_row_logical_cpus_to_date",
    "max_phase_contact_row_logical_cpus_to_date",
    "max_card_center_displacement_from_initial",
    "min_card_orientation_alignment_from_initial",
    "projectile_card_contacts",
)

TRACE_COLUMNS = (
    *TRACE_COLUMNS_V4,
    *TRACE_CONTRACT_COLUMNS_V5,
    *TRACE_PARALLEL_COLUMNS_V6,
    *TRACE_RESIDENCY_AND_CARD_COLUMNS_V7,
)

TRACE_COLORED_BGS_COLUMNS_V8 = (
    "inner_bgs_schedule_contract",
    "last_exact_colored_bgs_used",
    "last_exact_colored_bgs_solves",
    "last_exact_colored_bgs_dispatches",
    "last_exact_colored_bgs_max_participants",
    "last_exact_colored_bgs_manifolds",
    "last_exact_colored_bgs_colors",
    "last_exact_colored_bgs_max_manifolds_per_color",
    "exact_colored_bgs_logical_cpus",
    "max_phase_exact_colored_bgs_logical_cpus",
    "max_arch_body_displacement_from_initial",
    "min_arch_body_orientation_alignment_from_initial",
)

TRACE_COLORED_COLUMNS = (*TRACE_COLUMNS, *TRACE_COLORED_BGS_COLUMNS_V8)

SUMMARY_COLUMNS = (
    "scenario",
    "solver",
    "solver_contract",
    "collision_frontend",
    "requested_threads",
    "actual_threads",
    "repetitions",
    "steps_per_repetition",
    "sample_steps",
    "failed_processes",
    "mean_step_ms",
    "median_step_ms",
    "p95_step_ms",
    "max_step_ms",
    "mean_repetition_ms",
    "steps_per_second",
    "realtime_factor",
    "realtime_threshold_ms",
    "raw_mean_below_realtime",
    "raw_all_steps_below_realtime",
    "realtime_contract_valid",
    "realtime_contract_reasons",
    "mean_realtime_target_met",
    "all_steps_realtime_target_met",
    "affinity_source",
    "affinity_logical_cpus",
    "affinity_logical_cpu_count",
    "affinity_physical_core_count",
    "affinity_one_logical_per_physical_core",
    "affinity_physical_core_keys",
    "affinity_logical_cpu_physical_core_keys",
    "affinity_package_ids",
    "affinity_package_count",
    "affinity_smt_sibling_counts",
    "affinity_core_classes_khz",
    "affinity_scaling_governors",
    "single_core_claim_valid",
    "multicore_claim_valid",
    "parallelism_contract",
    "parallel_dispatch_valid",
    "parallel_dispatch_reasons",
    "contact_row_delassus_products",
    "parallel_contact_row_delassus_products",
    "parallel_contact_row_product_fraction",
    "parallel_contact_row_steps",
    "max_contact_row_participants",
    "inner_bgs_schedule_contract",
    "colored_bgs_used_steps",
    "colored_bgs_solves",
    "colored_bgs_dispatches",
    "max_colored_bgs_participants",
    "colored_bgs_manifolds",
    "colored_bgs_colors",
    "colored_bgs_max_manifolds_per_color",
    "colored_bgs_dispatch_valid",
    "colored_bgs_dispatch_reasons",
    "observed_exact_colored_bgs_logical_cpus",
    "observed_exact_colored_bgs_logical_cpu_count",
    "observed_exact_contact_row_logical_cpus",
    "observed_exact_contact_row_logical_cpu_count",
    "observed_exact_contact_row_physical_core_count",
    "phase_residency_logical_cpus_by_repetition",
    "phase_residency_min_logical_cpu_count",
    "phase_residency_min_physical_core_count",
    "runtime_cpu_residency_valid",
    "runtime_cpu_residency_reasons",
    "raw_speedup_vs_one_thread",
    "validated_speedup_vs_one_thread",
    "measured_workload_fingerprint_sha256",
    "scaling_pair_valid",
    "scaling_pair_reasons",
    "configured_warmup_repetitions",
    "completed_warmup_trajectories",
    "failed_warmup_processes",
    "warmup_contract_valid",
    "warmup_contract_reasons",
    "measured_trajectories",
    "complete_measured_trajectories",
    "complete_requested_trajectory_evidence",
    "full_trajectory_evidence",
    "scenario_trajectory_steps",
    "scenario_trajectory_contract",
    "physical_outcome_valid",
    "physical_outcome_reasons",
    "physical_outcome_details",
    "controlled_affinity_valid",
    "controlled_affinity_reasons",
    "paper_trajectory_steps",
    "paper_trajectory_contract",
    "exact_failures",
    "fallbacks",
    "solver_statuses",
    "all_solver_steps_accepted",
    "max_residual",
    "median_residual",
    "p95_residual",
    "residual_sample_steps",
    "contact_steps_without_comparable_residual",
    "min_contacts",
    "max_contacts",
    "min_unique_colliding_body_pairs",
    "max_unique_colliding_body_pairs",
    "max_penetration_depth",
    "max_penetration_depth_p95",
    "max_residual_primal_feasibility",
    "max_residual_dual_feasibility",
    "max_residual_complementarity",
    "min_accepted_gamma",
    "max_accepted_gamma",
    "min_safe_gamma",
    "max_safe_gamma",
    "total_shrink_iterations",
    "max_coupling_variation_ratio",
    "mean_warm_start_matched_fraction",
    "total_fbf_iterations",
    "median_step_fbf_iterations",
    "p95_step_fbf_iterations",
    "warm_start_step_fraction",
    "all_states_finite",
    "all_solver_steps_successful",
    "residual_pass_fraction",
    "paper_reference_ms",
    "paper_residual_pass_fraction",
    "paper_median_outer_iterations",
    "paper_p95_outer_iterations",
    "paper_reference_contacts",
    "contacts_match_paper_reference",
    "paper_workload_contract_valid",
    "paper_workload_contract_reasons",
    "paper_hardware_contract",
    "paper_timing_comparable",
    "paper_target_evaluated",
    "paper_ratio_to_reference",
    "paper_mean_target_met",
    "paper_comparison_note",
    "precision_contract",
    "scene_contract",
    "baumgarte_contract",
    "inner_local_solver",
    "inner_sweeps_requested",
    "fixed_inner_sweeps_requested",
    "step_size_persistence_enabled",
    "step_size_recovery_growth_factor",
    "step_size_persistence_used_steps",
    "min_step_size_persistence_request",
    "max_step_size_persistence_request",
    "row_operator_request",
    "row_operator_modes",
    "exact_diagnostics_contract",
    "max_outer_iterations",
    "tolerance",
    "accept_outer_max_iterations",
    "inner_local_iterations",
    "adaptive_step_size_enabled",
    "warm_start_enabled",
    "projected_gradient_retry_enabled",
    "dense_residual_polish_enabled",
    "fallback_to_boxed_lcp_enabled",
    "diagonal_seed_enabled",
    "matrix_free_seed_enabled",
    "step_size_scale",
    "outer_relaxation",
    "initial_gamma_contract",
    "split_impulse_enabled",
)

RUN_COLUMNS = (
    "repetition",
    "process_returncode",
    "warmup",
    "affinity_source",
    "affinity_logical_cpus",
    "affinity_logical_cpu_count",
    "affinity_physical_core_count",
    "affinity_one_logical_per_physical_core",
    "affinity_physical_core_keys",
    "affinity_logical_cpu_physical_core_keys",
    "affinity_package_ids",
    "affinity_package_count",
    "affinity_smt_sibling_counts",
    "affinity_core_classes_khz",
    "affinity_scaling_governors",
    "paper_hardware_contract",
)

_UINT64_MAX = (1 << 64) - 1
_INT32_MIN = -(1 << 31)
_INT32_MAX = (1 << 31) - 1

# Integer-valued trace fields are evidence-bearing counters or discrete solver
# contract values. Parse them all before validation or aggregation so corrupt
# text cannot silently become a favorable zero.
_TRACE_INTEGER_RANGES: dict[str, tuple[int, int]] = {
    "step": (1, _UINT64_MAX),
    "inner_sweeps_requested": (-1, _UINT64_MAX),
    "fixed_inner_sweeps_requested": (-1, 1),
    "step_size_persistence_enabled": (-1, 1),
    "step_size_persistence_used": (-1, 1),
    "requested_threads": (1, _UINT64_MAX),
    "actual_threads": (1, _UINT64_MAX),
    "contacts": (0, _UINT64_MAX),
    "unique_colliding_body_pairs": (0, _UINT64_MAX),
    "step_exact_solves": (0, _UINT64_MAX),
    "step_warm_starts": (0, _UINT64_MAX),
    "step_exact_failures": (0, _UINT64_MAX),
    "step_fallbacks": (0, _UINT64_MAX),
    "step_fbf_iterations": (0, _UINT64_MAX),
    "shrink_iterations": (-1, _UINT64_MAX),
    "warm_start_matched_contacts": (-1, _UINT64_MAX),
    "card_count": (0, _UINT64_MAX),
    "projectile_count": (0, _UINT64_MAX),
    "finite_state": (0, 1),
    "max_outer_iterations": (-1, _UINT64_MAX),
    "accept_outer_max_iterations": (-1, 1),
    "inner_local_iterations": (-1, _UINT64_MAX),
    "adaptive_step_size_enabled": (-1, 1),
    "warm_start_enabled": (-1, 1),
    "projected_gradient_retry_enabled": (-1, 1),
    "dense_residual_polish_enabled": (-1, 1),
    "fallback_to_boxed_lcp_enabled": (-1, 1),
    "diagonal_seed_enabled": (-1, 1),
    "matrix_free_seed_enabled": (-1, 1),
    "split_impulse_enabled": (-1, 1),
    "step_contact_row_delassus_products": (0, _UINT64_MAX),
    "step_parallel_contact_row_delassus_products": (0, _UINT64_MAX),
    "max_contact_row_participants_to_date": (0, _UINT64_MAX),
    "projectile_card_contacts": (0, _UINT64_MAX),
    "last_exact_colored_bgs_used": (-1, 1),
    "last_exact_colored_bgs_solves": (0, _UINT64_MAX),
    "last_exact_colored_bgs_dispatches": (0, _UINT64_MAX),
    "last_exact_colored_bgs_max_participants": (0, _UINT64_MAX),
    "last_exact_colored_bgs_manifolds": (0, _UINT64_MAX),
    "last_exact_colored_bgs_colors": (0, _UINT64_MAX),
    "last_exact_colored_bgs_max_manifolds_per_color": (0, _UINT64_MAX),
}

_RUN_INTEGER_RANGES: dict[str, tuple[int, int]] = {
    "repetition": (1, _UINT64_MAX),
    "process_returncode": (_INT32_MIN, _INT32_MAX),
    "warmup": (0, 1),
}

REALTIME_THRESHOLD_MS = 1000.0 / 60.0

SCALING_WORKLOAD_OPTION_FIELDS = (
    "inner_bgs_schedule_contract",
    "colored_bgs_manifolds",
    "colored_bgs_colors",
    "colored_bgs_max_manifolds_per_color",
    "precision_contract",
    "scene_contract",
    "baumgarte_contract",
    "inner_local_solver",
    "inner_sweeps_requested",
    "fixed_inner_sweeps_requested",
    "step_size_persistence_enabled",
    "step_size_recovery_growth_factor",
    "row_operator_request",
    "max_outer_iterations",
    "tolerance",
    "accept_outer_max_iterations",
    "inner_local_iterations",
    "adaptive_step_size_enabled",
    "warm_start_enabled",
    "projected_gradient_retry_enabled",
    "dense_residual_polish_enabled",
    "fallback_to_boxed_lcp_enabled",
    "diagonal_seed_enabled",
    "matrix_free_seed_enabled",
    "step_size_scale",
    "outer_relaxation",
    "initial_gamma_contract",
    "split_impulse_enabled",
    "steps_per_repetition",
    "repetitions",
    "configured_warmup_repetitions",
)

SCALING_MEASURED_WORK_FIELDS = (
    "repetition",
    "step",
    "time",
    "contacts",
    "unique_colliding_body_pairs",
    "penetration_depth_min",
    "penetration_depth_median",
    "penetration_depth_p95",
    "penetration_depth_max",
    "step_size_persistence_used",
    "step_size_persistence_request",
    "row_operator_mode",
    "exact_diagnostics_contract",
    "step_exact_solves",
    "step_warm_starts",
    "step_exact_failures",
    "step_fallbacks",
    "step_fbf_iterations",
    "step_contact_row_delassus_products",
    "step_parallel_contact_row_delassus_products",
    "max_contact_row_participants_to_date",
    "exact_contact_row_logical_cpus_to_date",
    "max_phase_contact_row_logical_cpus_to_date",
    "last_exact_colored_bgs_used",
    "last_exact_colored_bgs_solves",
    "last_exact_colored_bgs_manifolds",
    "last_exact_colored_bgs_colors",
    "last_exact_colored_bgs_max_manifolds_per_color",
    "residual",
    "status",
    "residual_primal_feasibility",
    "residual_dual_feasibility",
    "residual_complementarity",
    "accepted_gamma",
    "safe_gamma",
    "shrink_iterations",
    "coupling_variation_ratio",
    "warm_start_matched_contacts",
    "warm_start_matched_fraction",
    "phase",
    "card_count",
    "projectile_count",
    "finite_state",
    "min_card_axis_up",
    "min_center_height",
    "max_card_horizontal_travel",
    "max_projectile_speed",
    "max_card_center_displacement_from_initial",
    "min_card_orientation_alignment_from_initial",
    "projectile_card_contacts",
    "max_arch_body_displacement_from_initial",
    "min_arch_body_orientation_alignment_from_initial",
    "tracked_body",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "up_z",
)

# These diagnostics are invariant across a valid one-thread/four-thread
# colored-inner-BGS pair. They are intentionally excluded from legacy
# contact-row-kernel pairs, where parallel counters and CPU residency are
# execution evidence that necessarily differ with the thread count.
SCALING_COLORED_MEASURED_WORK_FIELDS = frozenset(
    {
        "step_parallel_contact_row_delassus_products",
        "max_contact_row_participants_to_date",
        "exact_contact_row_logical_cpus_to_date",
        "max_phase_contact_row_logical_cpus_to_date",
        "last_exact_colored_bgs_used",
        "last_exact_colored_bgs_solves",
        "last_exact_colored_bgs_manifolds",
        "last_exact_colored_bgs_colors",
        "last_exact_colored_bgs_max_manifolds_per_color",
        "max_arch_body_displacement_from_initial",
        "min_arch_body_orientation_alignment_from_initial",
    }
)

PAPER_CPU_SPLIT_IMPULSE_SCENARIOS = frozenset(
    {
        "card_house_26_settle_projectile_full",
        "masonry_arch_25_full_manifold",
        "masonry_arch_101_full_manifold",
    }
)

LITERAL_WEDGE_ARCH_SCENARIO = "masonry_arch_25_literal_wedge"
LITERAL_WEDGE_ARCH_SCENE_CONTRACT = (
    "reconstructed_literal_wedge_arch_nonpaper_native_collision_frontend"
)
TRACE_SPLIT_IMPULSE_SCENARIOS = frozenset(
    {*PAPER_CPU_SPLIT_IMPULSE_SCENARIOS, LITERAL_WEDGE_ARCH_SCENARIO}
)

TRACE_SCENARIOS = frozenset(
    {
        "backspin",
        "incline_mu_0_5",
        "incline_mu_0_4",
        "turntable_mu_0_2_omega_2",
        "turntable_mu_0_2_omega_5",
        "turntable_mu_0_5_omega_2",
        "turntable_mu_0_5_omega_5",
        "painleve_mu_0_5",
        "painleve_mu_0_55",
        "card_house_26_reduced_contact",
        "card_house_26_settle_projectile_reduced_contact",
        "card_house_26_settle_projectile_full",
        "masonry_arch_25_reduced_contact",
        "masonry_arch_25_projectile_reduced_contact",
        "masonry_arch_101_reduced_contact",
        "masonry_arch_25_full_manifold",
        LITERAL_WEDGE_ARCH_SCENARIO,
        "masonry_arch_101_full_manifold",
    }
)

PAPER_REFERENCE_MS = {
    "backspin": 6.0,
    "incline_mu_0_5": 5.5,
    "incline_mu_0_4": 5.3,
    "painleve_mu_0_5": 7.0,
    "painleve_mu_0_55": 6.4,
    "card_house_26_reduced_contact": 199.0,
    "card_house_26_settle_projectile_reduced_contact": 199.0,
    "card_house_26_settle_projectile_full": 199.0,
    "masonry_arch_25_reduced_contact": 595.0,
    "masonry_arch_25_projectile_reduced_contact": 595.0,
    "masonry_arch_25_full_manifold": 595.0,
    "masonry_arch_101_reduced_contact": 1234.0,
    "masonry_arch_101_full_manifold": 1234.0,
    "card_house_10_level": 853.0,
    "turntable_mu_0_5_omega_2": 6.8,
    "turntable_mu_0_5_omega_5": 3.1,
    "turntable_mu_0_2_omega_2": 3.7,
    "turntable_mu_0_2_omega_5": 3.1,
}

PAPER_REFERENCE_CONTACTS = {
    "backspin": 1,
    "incline_mu_0_5": 4,
    "incline_mu_0_4": 4,
    "painleve_mu_0_5": 4,
    "painleve_mu_0_55": 4,
    "card_house_26_reduced_contact": 214,
    "card_house_26_settle_projectile_reduced_contact": 214,
    "card_house_26_settle_projectile_full": 214,
    "masonry_arch_25_reduced_contact": 100,
    "masonry_arch_25_projectile_reduced_contact": 100,
    "masonry_arch_25_full_manifold": 100,
    "turntable_mu_0_5_omega_2": 4,
    "turntable_mu_0_5_omega_5": 0,
    "turntable_mu_0_2_omega_2": 0,
    "turntable_mu_0_2_omega_5": 0,
}

# A per-step mean can only be compared when it covers the same trajectory.
# The paper publishes an exact duration for the incline and the 26-card visual
# sequence.  It gives a last figure snapshot for backspin, but does not say
# that Appendix B timed exactly that interval.  The other timing-run lengths
# are not disclosed, so the runner must retain their timings as raw evidence.
PAPER_TRAJECTORY_CONTRACT = {
    "incline_mu_0_5": (120, "published_2_seconds_at_60_hz"),
    "incline_mu_0_4": (120, "published_2_seconds_at_60_hz"),
    "backspin": (130, "figure_endpoint_only_timing_run_length_unpublished"),
    "card_house_26_settle_projectile_full": (
        600,
        "published_10_seconds_at_60_hz",
    ),
}

# Canonical complete trajectories exposed by fbf_paper_trace. Single-step
# contact-rich probes are intentionally absent: they can measure one frame but
# cannot prove sustained realtime behavior.
SCENARIO_TRAJECTORY_CONTRACT = {
    "backspin": (240, "trace_full_4_seconds_at_60_hz"),
    "incline_mu_0_5": (120, "paper_full_2_seconds_at_60_hz"),
    "incline_mu_0_4": (120, "paper_full_2_seconds_at_60_hz"),
    "painleve_mu_0_5": (150, "trace_full_2_5_seconds_at_60_hz"),
    "painleve_mu_0_55": (150, "trace_full_2_5_seconds_at_60_hz"),
    "turntable_mu_0_5_omega_2": (240, "trace_full_4_seconds_at_60_hz"),
    "turntable_mu_0_5_omega_5": (240, "trace_full_4_seconds_at_60_hz"),
    "turntable_mu_0_2_omega_2": (240, "trace_full_4_seconds_at_60_hz"),
    "turntable_mu_0_2_omega_5": (240, "trace_full_4_seconds_at_60_hz"),
    "card_house_26_settle_projectile_full": (
        600,
        "paper_full_10_seconds_at_60_hz",
    ),
    LITERAL_WEDGE_ARCH_SCENARIO: (
        600,
        "reconstructed_literal_wedge_full_10_seconds_at_60_hz_nonpaper",
    ),
}

# The small-scene values mirror physical assertions in
# tests/integration/test_ExactCoulombFbfPaperFixtures.cpp. The card-house gates
# are reconstruction-specific because the fixture does not publish an
# equivalent full-trajectory stability/contact contract.
INCLINE_TAN = 0.5
INCLINE_INITIAL_NORMAL_OFFSET = 0.5 - 0.01
INCLINE_STICK_DISPLACEMENT_TOLERANCE = 2e-2
INCLINE_SLIDE_FRICTION = 0.4
INCLINE_SLIDE_DISPLACEMENT_TOLERANCE = 0.2
INCLINE_MAX_PENETRATION = 2e-2
GRAVITY = 9.81
BACKSPIN_RADIUS = 0.25
BACKSPIN_INITIAL_LINEAR_VELOCITY = 4.0
BACKSPIN_INITIAL_ANGULAR_VELOCITY = -200.0
BACKSPIN_LINEAR_VELOCITY_TOLERANCE = 0.5
BACKSPIN_HEIGHT_TOLERANCE = 3e-2
TURNTABLE_CAPTURE_RADIUS_MAX = 1.25
TURNTABLE_CAPTURE_HEIGHT_MIN = 0.05
TURNTABLE_CAPTURE_RADIAL_VELOCITY_MAX = 0.2
TURNTABLE_CAPTURE_COROTATION_SPEED_ERROR_MAX = 0.05
TURNTABLE_EJECTION_RADIUS_MIN = 1.75
PAINLEVE_UPRIGHT_UP_Z_MIN = 0.85
PAINLEVE_TUMBLED_UP_Z_MAX = 0.55
PAINLEVE_HEIGHT_THRESHOLD = 0.35
CARD_HOUSE_FULL_CARD_COUNT = 26
CARD_HOUSE_FULL_PROJECTILE_COUNT = 4
CARD_HOUSE_FULL_SETTLE_STEPS = 402
# The aggregate card metrics include horizontal support cards whose local
# z-axis is horizontal in the intended construction. These broad relative
# gates therefore reject premature settle collapse, then require a visible
# post-impact response, without pretending to recover unpublished author
# trajectory tolerances.
CARD_HOUSE_SETTLE_MIN_AXIS_UP = -0.25
CARD_HOUSE_SETTLE_MIN_CENTER_HEIGHT = 0.35
CARD_HOUSE_SETTLE_MAX_HORIZONTAL_DRIFT = 0.25
CARD_HOUSE_SETTLE_MAX_CENTER_DISPLACEMENT = 0.05
CARD_HOUSE_SETTLE_MIN_ORIENTATION_ALIGNMENT = 0.95
CARD_HOUSE_PROJECTILE_MIN_SPEED = 0.1
CARD_HOUSE_PROJECTILE_MIN_AXIS_RESPONSE = 0.5
CARD_HOUSE_PROJECTILE_MIN_HEIGHT_RESPONSE = 0.15
CARD_HOUSE_PROJECTILE_MIN_HORIZONTAL_RESPONSE = 0.25
LITERAL_WEDGE_ARCH_CONTACT_COUNT = 96
LITERAL_WEDGE_ARCH_COLLIDING_BODY_PAIR_COUNT = 24
LITERAL_WEDGE_ARCH_MAX_CROWN_DISPLACEMENT = 1e-3
LITERAL_WEDGE_ARCH_MIN_CROWN_UP_Z = 0.999
LITERAL_WEDGE_ARCH_MAX_BODY_DISPLACEMENT = 1e-3
LITERAL_WEDGE_ARCH_MIN_BODY_ORIENTATION_ALIGNMENT = 0.999

PHYSICAL_OUTCOME_TRACKED_BODIES = {
    "incline_mu_0_5": "incline_cube_body",
    "incline_mu_0_4": "incline_cube_body",
    "backspin": "backspin_sphere_body",
    "turntable_mu_0_5_omega_2": "turntable_rider_body",
    "turntable_mu_0_5_omega_5": "turntable_rider_body",
    "turntable_mu_0_2_omega_2": "turntable_rider_body",
    "turntable_mu_0_2_omega_5": "turntable_rider_body",
    "painleve_mu_0_5": "painleve_box_body",
    "painleve_mu_0_55": "painleve_box_body",
    "card_house_26_settle_projectile_full": "card_house_l3_f0_left_body",
    LITERAL_WEDGE_ARCH_SCENARIO: "masonry_arch_stone_12_body",
}

PAPER_CONVERGENCE_REFERENCE = {
    "backspin": (math.nan, 25.0, 30.0),
    "incline_mu_0_5": (math.nan, 5.0, 5.0),
    "incline_mu_0_4": (math.nan, 5.0, 5.0),
    "painleve_mu_0_5": (math.nan, 5.0, 25.0),
    "painleve_mu_0_55": (math.nan, 5.0, 30.0),
    "turntable_mu_0_5_omega_2": (math.nan, 10.0, 15.0),
    "turntable_mu_0_5_omega_5": (math.nan, 0.0, 15.0),
    "turntable_mu_0_2_omega_2": (math.nan, 0.0, 15.0),
    "turntable_mu_0_2_omega_5": (math.nan, 0.0, 10.0),
    "card_house_26_reduced_contact": (0.93, 5.0, 115.0),
    "card_house_26_settle_projectile_reduced_contact": (0.93, 5.0, 115.0),
    "card_house_26_settle_projectile_full": (0.93, 5.0, 115.0),
    "masonry_arch_25_reduced_contact": (0.47, 200.0, 200.0),
    "masonry_arch_25_projectile_reduced_contact": (0.47, 200.0, 200.0),
    "masonry_arch_25_full_manifold": (0.47, 200.0, 200.0),
}

PAPER_MEDIAN_RESIDUAL_REFERENCE = {
    "card_house_26_reduced_contact": 6.3e-7,
    "card_house_26_settle_projectile_reduced_contact": 6.3e-7,
    "card_house_26_settle_projectile_full": 6.3e-7,
    "masonry_arch_25_reduced_contact": 1.1e-6,
    "masonry_arch_25_projectile_reduced_contact": 1.1e-6,
    "masonry_arch_25_full_manifold": 1.1e-6,
}


@dataclass(frozen=True)
class Case:
    scenario: str
    steps: int


@dataclass(frozen=True)
class Invocation:
    case: Case
    threads: int
    repetition: int
    warmup: bool


def _parse_case(value: str) -> Case:
    try:
        scenario, raw_steps = value.rsplit(":", 1)
        steps = int(raw_steps)
    except (ValueError, TypeError) as error:
        raise argparse.ArgumentTypeError(
            "case must have the form SCENARIO:STEPS"
        ) from error
    if scenario not in TRACE_SCENARIOS or steps <= 0:
        raise argparse.ArgumentTypeError(
            "case must name a canonical fbf_paper_trace scenario and use a "
            "positive step count"
        )
    return Case(scenario=scenario, steps=steps)


def _parse_threads(value: str) -> tuple[int, ...]:
    try:
        threads = tuple(int(item) for item in value.split(","))
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "threads must be a comma-separated list of positive integers"
        ) from error
    if not threads or any(item <= 0 for item in threads):
        raise argparse.ArgumentTypeError(
            "threads must be a comma-separated list of positive integers"
        )
    return tuple(dict.fromkeys(threads))


def _expand_cpu_list(value: str) -> tuple[int, ...]:
    cpus: list[int] = []
    try:
        for item in value.split(","):
            if not item:
                raise ValueError
            if "-" in item:
                if item.count("-") != 1:
                    raise ValueError
                raw_first, raw_last = item.split("-", 1)
                first = int(raw_first)
                last = int(raw_last)
                if first < 0 or last < first:
                    raise ValueError
                cpus.extend(range(first, last + 1))
            else:
                cpu = int(item)
                if cpu < 0:
                    raise ValueError
                cpus.append(cpu)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "CPU list must use taskset syntax such as 2-5 or 2,4,6,8"
        ) from error
    if not cpus or len(set(cpus)) != len(cpus):
        raise argparse.ArgumentTypeError(
            "CPU list must be nonempty and must not repeat logical CPUs"
        )
    return tuple(cpus)


def _parse_cpu_list_for(value: str) -> tuple[int, str]:
    try:
        raw_threads, cpu_list = value.split(":", 1)
        threads = int(raw_threads)
    except (ValueError, TypeError) as error:
        raise argparse.ArgumentTypeError(
            "CPU mapping must have the form THREADS:CPU_LIST"
        ) from error
    if threads <= 0 or not cpu_list:
        raise argparse.ArgumentTypeError(
            "CPU mapping must use positive THREADS and a taskset CPU list"
        )
    _expand_cpu_list(cpu_list)
    return threads, cpu_list


def _parse_initial_gamma(value: str) -> str:
    if value == "nan":
        return value
    try:
        gamma = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            "initial gamma must be a positive finite number or nan"
        ) from error
    if not math.isfinite(gamma) or gamma <= 0.0:
        raise argparse.ArgumentTypeError(
            "initial gamma must be a positive finite number or nan"
        )
    return value


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binary",
        type=Path,
        required=True,
        help="Path to the built fbf_paper_trace executable.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="New or empty directory for metadata, raw rows, and summaries.",
    )
    parser.add_argument(
        "--case",
        action="append",
        type=_parse_case,
        required=True,
        help="Trajectory as SCENARIO:STEPS; repeat for additional scenes.",
    )
    parser.add_argument(
        "--threads",
        type=_parse_threads,
        default=(1,),
        help="Comma-separated requested simulation-thread counts (default: 1).",
    )
    parser.add_argument(
        "--repetitions",
        type=int,
        default=3,
        help="Measured complete trajectories per case/thread pair (default: 3).",
    )
    parser.add_argument(
        "--warmup-repetitions",
        type=int,
        default=0,
        help="Discarded complete trajectories before measurement (default: 0).",
    )
    parser.add_argument(
        "--solver",
        choices=("exact_fbf", "boxed_lcp"),
        default="exact_fbf",
    )
    parser.add_argument(
        "--contract",
        choices=("dart_best", "dart_best_colored_bgs", "paper_cpu"),
        default="dart_best",
        help=(
            "dart_best_colored_bgs is an explicitly non-paper inner-schedule "
            "diagnostic; paper_cpu is fail-closed to one simulation thread "
            "and the designated native reconstructed-paper frontend"
        ),
    )
    parser.add_argument(
        "--collision-frontend",
        choices=("dart", "native"),
        default="dart",
        help="Collision frontend installed in the trace world (default: dart).",
    )
    parser.add_argument(
        "--local-solver",
        choices=(
            "default",
            "exact_metric",
            "inverse_euclidean",
            "projected_gradient",
        ),
        default="default",
        help=(
            "Diagnostic dart_best local block solver override. paper_cpu fixes "
            "exact_metric and rejects overrides."
        ),
    )
    parser.add_argument(
        "--warm-start",
        choices=("default", "0", "1"),
        default="default",
    )
    parser.add_argument(
        "--split-impulse",
        choices=("default", "0", "1"),
        default="default",
    )
    parser.add_argument(
        "--initial-gamma",
        type=_parse_initial_gamma,
        default="nan",
        help="Positive gamma or nan for the trace's adaptive default.",
    )
    parser.add_argument(
        "--cpu-list",
        type=lambda value: (value, _expand_cpu_list(value))[0],
        help="Optional taskset CPU list, for example 2-5 or 2,4,6,8.",
    )
    parser.add_argument(
        "--cpu-list-for",
        action="append",
        type=_parse_cpu_list_for,
        default=[],
        metavar="THREADS:CPU_LIST",
        help=(
            "Thread-specific taskset override; repeat for a controlled scaling "
            "matrix, for example 1:4 and 4:4,6,8,10."
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3600.0,
        help="Per-trajectory timeout in seconds (default: 3600).",
    )
    parser.add_argument(
        "--accept-nonzero",
        action="store_true",
        help="Return success after recording failed strict-contract trajectories.",
    )
    args = parser.parse_args(argv)
    if args.repetitions <= 0:
        parser.error("--repetitions must be positive")
    if args.warmup_repetitions < 0:
        parser.error("--warmup-repetitions cannot be negative")
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        parser.error("--timeout must be positive and finite")
    if args.local_solver != "default" and (
        args.contract not in {"dart_best", "dart_best_colored_bgs"}
        or args.solver != "exact_fbf"
    ):
        parser.error(
            "--local-solver overrides require a dart_best contract and "
            "--solver exact_fbf"
        )
    if args.contract == "dart_best_colored_bgs" and args.solver != "exact_fbf":
        parser.error("--contract dart_best_colored_bgs requires --solver exact_fbf")
    if args.contract == "paper_cpu" and (
        args.initial_gamma != "nan"
        or args.warm_start != "default"
        or args.split_impulse != "default"
        or args.local_solver != "default"
    ):
        parser.error(
            "--contract paper_cpu rejects initial-gamma, warm-start, "
            "split-impulse, and local-solver overrides"
        )
    if args.contract == "paper_cpu" and args.threads != (1,):
        parser.error(
            "--contract paper_cpu requires --threads 1; use dart_best for "
            "thread-count diagnostics"
        )
    if args.contract == "paper_cpu" and args.solver != "exact_fbf":
        parser.error(
            "--contract paper_cpu requires --solver exact_fbf; use dart_best "
            "for boxed-LCP diagnostics"
        )
    if args.contract == "paper_cpu" and args.collision_frontend != "native":
        parser.error(
            "--contract paper_cpu requires --collision-frontend native; use "
            "dart_best for frontend diagnostics"
        )
    scenarios = [case.scenario for case in args.case]
    if len(set(scenarios)) != len(scenarios):
        parser.error("--case scenario names must be unique")
    if LITERAL_WEDGE_ARCH_SCENARIO in scenarios and args.collision_frontend != "native":
        parser.error(
            f"--case {LITERAL_WEDGE_ARCH_SCENARIO} requires "
            "--collision-frontend native"
        )
    cpu_mapping_keys = [threads for threads, _ in args.cpu_list_for]
    if len(set(cpu_mapping_keys)) != len(cpu_mapping_keys):
        parser.error("--cpu-list-for thread counts must be unique")
    unexpected_mappings = sorted(set(cpu_mapping_keys) - set(args.threads))
    if unexpected_mappings:
        parser.error(
            "--cpu-list-for has mappings outside --threads: "
            + ",".join(str(value) for value in unexpected_mappings)
        )
    args.cpu_lists_by_threads = dict(args.cpu_list_for)
    args.executed_tool_closure = {}
    args.executed_tool_identity_rechecks = []
    return args


def _run_text(command: Sequence[str], cwd: Path | None = None) -> str | None:
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=10.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if completed.returncode != 0:
        return None
    return completed.stdout.strip()


def _read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _current_affinity() -> tuple[int, ...]:
    if hasattr(os, "sched_getaffinity"):
        return tuple(sorted(os.sched_getaffinity(0)))
    logical_cpus = os.cpu_count()
    return tuple(range(logical_cpus or 0))


def _logical_cpu_topology(logical_cpus: Sequence[int]) -> dict[int, dict[str, object]]:
    topology: dict[int, dict[str, object]] = {}
    for cpu in logical_cpus:
        base = Path(f"/sys/devices/system/cpu/cpu{cpu}/topology")
        frequency_base = Path(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq")
        package = _read_text(base / "physical_package_id")
        core = _read_text(base / "core_id")
        siblings = _read_text(base / "thread_siblings_list")
        sibling_count: int | None = None
        if siblings is not None:
            try:
                sibling_count = len(_expand_cpu_list(siblings))
            except argparse.ArgumentTypeError:
                sibling_count = None
        raw_max_frequency = _read_text(frequency_base / "cpuinfo_max_freq")
        max_frequency_khz = (
            int(raw_max_frequency)
            if raw_max_frequency is not None
            and raw_max_frequency.isdigit()
            and int(raw_max_frequency) > 0
            else None
        )
        topology[cpu] = {
            "physical_package_id": package,
            "core_id": core,
            "thread_siblings_list": siblings,
            "thread_sibling_count": sibling_count,
            "cpuinfo_max_frequency_khz": max_frequency_khz,
            "scaling_governor": _read_text(frequency_base / "scaling_governor"),
        }
    return topology


def _physical_core_keys(
    topology: dict[int, dict[str, object]],
) -> tuple[tuple[str, str], ...] | None:
    keys: set[tuple[str, str]] = set()
    for values in topology.values():
        package = values.get("physical_package_id")
        core = values.get("core_id")
        if not isinstance(package, str) or not isinstance(core, str):
            return None
        keys.add((package, core))
    return tuple(sorted(keys))


def _paper_hardware_contract() -> str:
    machine = platform.machine().lower()
    if platform.system() == "Darwin" and machine in {"arm64", "aarch64"}:
        return "apple_silicon_family_exact_model_unpublished"
    return (
        f"{platform.system().lower()}_{machine or 'unknown'}" "_vs_paper_apple_silicon"
    )


def _invocation_affinity(
    args: argparse.Namespace, invocation: Invocation
) -> dict[str, object]:
    configured = args.cpu_lists_by_threads.get(invocation.threads, args.cpu_list)
    if configured:
        logical_cpus = _expand_cpu_list(configured)
        source = "explicit_taskset"
    else:
        logical_cpus = _current_affinity()
        source = "inherited_process_affinity"
    topology = _logical_cpu_topology(logical_cpus)
    physical_keys = _physical_core_keys(topology)
    physical_count = len(physical_keys) if physical_keys is not None else None
    package_ids = {
        values["physical_package_id"]
        for values in topology.values()
        if isinstance(values.get("physical_package_id"), str)
    }
    sibling_counts = {
        values["thread_sibling_count"]
        for values in topology.values()
        if isinstance(values.get("thread_sibling_count"), int)
    }
    core_classes_khz = {
        values["cpuinfo_max_frequency_khz"]
        for values in topology.values()
        if isinstance(values.get("cpuinfo_max_frequency_khz"), int)
    }
    governors = {
        values["scaling_governor"]
        for values in topology.values()
        if isinstance(values.get("scaling_governor"), str)
        and bool(values["scaling_governor"])
    }
    logical_cpu_physical_core_keys = {
        str(cpu): (
            f"{values['physical_package_id']}:{values['core_id']}"
            if isinstance(values.get("physical_package_id"), str)
            and isinstance(values.get("core_id"), str)
            else None
        )
        for cpu, values in topology.items()
    }
    package_complete = bool(topology) and all(
        isinstance(values.get("physical_package_id"), str)
        for values in topology.values()
    )
    sibling_complete = bool(topology) and all(
        isinstance(values.get("thread_sibling_count"), int)
        for values in topology.values()
    )
    core_class_complete = bool(topology) and all(
        isinstance(values.get("cpuinfo_max_frequency_khz"), int)
        for values in topology.values()
    )
    governor_complete = bool(topology) and all(
        isinstance(values.get("scaling_governor"), str)
        and bool(values["scaling_governor"])
        for values in topology.values()
    )
    return {
        "source": source,
        "logical_cpus": list(logical_cpus),
        "logical_cpu_count": len(logical_cpus),
        "physical_core_count": physical_count,
        "one_logical_per_physical_core": (
            len(logical_cpus) == physical_count if physical_count is not None else None
        ),
        "logical_cpu_topology": {str(cpu): values for cpu, values in topology.items()},
        "logical_cpu_physical_core_keys": logical_cpu_physical_core_keys,
        "physical_core_keys": (
            [f"{package}:{core}" for package, core in physical_keys]
            if physical_keys is not None
            else None
        ),
        "package_ids": sorted(package_ids) if package_complete else None,
        "package_count": len(package_ids) if package_complete else None,
        "smt_sibling_counts": (sorted(sibling_counts) if sibling_complete else None),
        "core_classes_khz": (sorted(core_classes_khz) if core_class_complete else None),
        "scaling_governors": (sorted(governors) if governor_complete else None),
    }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _tool_identity(name: str) -> dict[str, object]:
    """Resolve and bind one executable tool to an exact regular file."""

    path_text = shutil.which(name)
    if path_text is None:
        raise RuntimeError(f"required tool is unavailable: {name}")
    path = Path(path_text)
    try:
        resolved_path = path.resolve(strict=True)
    except (OSError, RuntimeError) as error:
        raise RuntimeError(f"{name} tool cannot be resolved: {path}") from error
    if not resolved_path.is_file():
        raise RuntimeError(f"resolved {name} tool is not a regular file")
    try:
        size_bytes = resolved_path.stat().st_size
        sha256 = _sha256(resolved_path)
    except OSError as error:
        raise RuntimeError(
            f"resolved {name} tool identity cannot be read: {resolved_path}"
        ) from error
    return {
        "name": name,
        "path": str(path),
        "resolved_path": str(resolved_path),
        "size_bytes": size_bytes,
        "sha256": sha256,
    }


def _bind_executed_tool_closure(args: argparse.Namespace) -> dict[str, object]:
    if args.cpu_list is None and not args.cpu_lists_by_threads:
        return {}
    return {"taskset": _tool_identity("taskset")}


def _require_executed_tool_closure_unchanged(
    args: argparse.Namespace,
    invocation: Invocation,
) -> None:
    closure = args.executed_tool_closure
    if not closure:
        return
    taskset_identity = closure.get("taskset")
    if not isinstance(taskset_identity, dict):
        raise RuntimeError("bound taskset executable identity is missing")
    stage = _slug(invocation)
    try:
        current_identity = _tool_identity("taskset")
    except (OSError, RuntimeError) as error:
        raise RuntimeError(
            "taskset executable identity cannot be revalidated after "
            f"{stage}: {error}"
        ) from error
    if current_identity != taskset_identity:
        raise RuntimeError(f"taskset executable identity drifted after {stage}")
    args.executed_tool_identity_rechecks.append(f"after_{stage}")


def _source_identity(root: Path) -> dict[str, str]:
    runner = Path(__file__).resolve()
    trace_source = root / "tests/benchmark/integration/fbf_paper_trace.cpp"
    return {
        "runner_path": runner.relative_to(root).as_posix(),
        "runner_sha256": _sha256(runner),
        "trace_source_path": trace_source.relative_to(root).as_posix(),
        "trace_source_sha256": _sha256(trace_source),
    }


def _runtime_identity(binary: Path) -> dict[str, object]:
    ldd = shutil.which("ldd")
    if ldd is None:
        raise RuntimeError("ldd is required to bind the evidence runtime")
    ldd_path = Path(ldd).resolve()
    stdout = _run_text([str(ldd_path), str(binary.resolve())])
    if stdout is None:
        raise RuntimeError(f"ldd failed for evidence binary: {binary}")

    resolved: dict[str, dict[str, object]] = {}
    for line in stdout.splitlines():
        text = line.strip()
        if not text or "not found" in text:
            if "not found" in text:
                raise RuntimeError(f"unresolved evidence runtime dependency: {text}")
            continue
        candidate = (
            text.split("=>", 1)[1].strip().split()[0]
            if "=>" in text
            else text.split()[0]
        )
        path = Path(candidate)
        if not path.is_absolute() or not path.is_file():
            continue
        resolved_path = path.resolve()
        resolved[str(resolved_path)] = {
            "sha256": _sha256(resolved_path),
            "size_bytes": resolved_path.stat().st_size,
        }
    if not resolved:
        raise RuntimeError(f"ldd resolved no regular runtime files for: {binary}")
    resolved_digest = hashlib.sha256(
        json.dumps(resolved, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return {
        "ldd_path": str(ldd_path),
        "ldd_sha256": _sha256(ldd_path),
        "resolved_regular_files_sha256": resolved_digest,
        "resolved_regular_files": resolved,
    }


def _artifact_index(root: Path) -> dict[str, object]:
    """Describe the exact generated bundle membership before indexing it."""

    files: dict[str, dict[str, object]] = {}
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.name == "artifact-index.json":
            continue
        relative = path.relative_to(root).as_posix()
        files[relative] = {
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
    return {
        "schema_version": "dart.fbf_cpu_evidence_artifact_index/v1",
        "files": files,
    }


def _cpu_metadata() -> dict[str, object]:
    affinity = list(_current_affinity())
    model_names: list[str] = []
    cpuinfo = _read_text(Path("/proc/cpuinfo")) or ""
    for line in cpuinfo.splitlines():
        if line.startswith("model name") and ":" in line:
            model = line.split(":", 1)[1].strip()
            if model not in model_names:
                model_names.append(model)

    governors: dict[str, str] = {}
    frequencies_khz: dict[str, str] = {}
    maximum_frequencies_khz: dict[str, int] = {}
    sibling_counts: dict[str, int] = {}
    topology = _logical_cpu_topology(affinity)
    for cpu in affinity:
        base = Path(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq")
        governor = _read_text(base / "scaling_governor")
        frequency = _read_text(base / "scaling_cur_freq")
        if governor is not None:
            governors[str(cpu)] = governor
        if frequency is not None:
            frequencies_khz[str(cpu)] = frequency
        cpu_topology = topology.get(cpu, {})
        maximum_frequency = cpu_topology.get("cpuinfo_max_frequency_khz")
        sibling_count = cpu_topology.get("thread_sibling_count")
        if isinstance(maximum_frequency, int):
            maximum_frequencies_khz[str(cpu)] = maximum_frequency
        if isinstance(sibling_count, int):
            sibling_counts[str(cpu)] = sibling_count

    physical_cores: set[tuple[str, str]] = set()
    current: dict[str, str] = {}
    for line in [*cpuinfo.splitlines(), ""]:
        if not line.strip():
            if "physical id" in current and "core id" in current:
                physical_cores.add((current["physical id"], current["core id"]))
            current = {}
            continue
        if ":" in line:
            key, value = line.split(":", 1)
            current[key.strip()] = value.strip()

    return {
        "model_names": model_names,
        "logical_cpus": os.cpu_count(),
        "physical_cores_detected": len(physical_cores) or None,
        "process_affinity": affinity,
        "logical_cpu_topology": {str(cpu): values for cpu, values in topology.items()},
        "governors_by_cpu": governors,
        "cpuinfo_max_frequency_khz_by_cpu": maximum_frequencies_khz,
        "smt_sibling_count_by_cpu": sibling_counts,
        "sampled_frequency_khz_by_cpu": frequencies_khz,
        "load_average": list(os.getloadavg()) if hasattr(os, "getloadavg") else None,
    }


def _git_metadata(root: Path) -> dict[str, object]:
    status = _run_text(["git", "status", "--short"], cwd=root)
    return {
        "root": str(root),
        "sha": _run_text(["git", "rev-parse", "HEAD"], cwd=root),
        "branch": _run_text(["git", "branch", "--show-current"], cwd=root),
        "describe": _run_text(
            ["git", "describe", "--always", "--dirty", "--tags"], cwd=root
        ),
        "dirty": bool(status),
        "status_short": status.splitlines() if status else [],
    }


def _metadata(args: argparse.Namespace, root: Path) -> dict[str, object]:
    return {
        "schema_version": SCHEMA_VERSION,
        "schema_compatibility": (
            "v8 preserves the 83-column v7 default trace byte-for-byte and "
            "uses an opt-in extended schema for non-paper colored-inner-BGS "
            "schedule, dispatch, utilization-width, CPU-residency, and "
            "whole-arch initial-pose outcome fields"
        ),
        "generated_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "timing_scope": {
            "boundary": "steady_clock around each World::step",
            "includes": "collision detection and constraint solve inside World::step",
            "excludes": "scene construction and process startup",
            "aggregation": "mean over every measured step of every complete trajectory",
        },
        "execution_order": "sequential trajectory processes; no concurrent samples",
        "paper_comparison_policy": (
            "Paper target verdicts remain null unless the disclosed workload, "
            "full-trajectory, physical-outcome, convergence, sequential-thread, "
            "affinity, and Apple-silicon-family contracts all pass. The paper "
            "does not identify the Mac model or fully delimit its per-step timer."
        ),
        "physical_outcome_policy": {
            "source": (
                "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp "
                "fixture assertions for the small scenes; reconstructed "
                "card-house stability, projectile-contact, and response gates "
                "plus the published card-house phase timing encoded by "
                "fbf_paper_trace; explicit non-paper literal-wedge arch "
                "whole-arch initial-pose, crown-stability, and native "
                "collision-manifold gates"
            ),
            "contracted_scenarios": sorted(PHYSICAL_OUTCOME_TRACKED_BODIES),
            "uncontracted_scenarios": sorted(
                TRACE_SCENARIOS - frozenset(PHYSICAL_OUTCOME_TRACKED_BODIES)
            ),
            "evaluation": (
                "scene-specific trajectory and final-state gates for every "
                "complete measured repetition; card-house gates are explicitly "
                "reconstruction-specific rather than fixture-matched; the "
                "literal-wedge arch gate is explicitly reconstructed/non-paper"
            ),
        },
        "literal_wedge_arch_contract": {
            "scenario": LITERAL_WEDGE_ARCH_SCENARIO,
            "scene_contract": LITERAL_WEDGE_ARCH_SCENE_CONTRACT,
            "paper_parity": False,
            "collision_frontend": "native",
            "trajectory_steps": SCENARIO_TRAJECTORY_CONTRACT[
                LITERAL_WEDGE_ARCH_SCENARIO
            ][0],
            "split_impulse": True,
            "expected_contacts_per_step": LITERAL_WEDGE_ARCH_CONTACT_COUNT,
            "expected_colliding_body_pairs_per_step": (
                LITERAL_WEDGE_ARCH_COLLIDING_BODY_PAIR_COUNT
            ),
            "max_body_displacement_from_constructed_initial_pose": (
                LITERAL_WEDGE_ARCH_MAX_BODY_DISPLACEMENT
            ),
            "min_body_orientation_alignment_from_constructed_initial_pose": (
                LITERAL_WEDGE_ARCH_MIN_BODY_ORIENTATION_ALIGNMENT
            ),
            "tracked_body": PHYSICAL_OUTCOME_TRACKED_BODIES[
                LITERAL_WEDGE_ARCH_SCENARIO
            ],
            "colored_exact_options": {
                "max_outer_iterations": 5000,
                "inner_sweeps": 30,
                "fixed_inner_sweeps": True,
                "step_size_scale": 35.0,
                "outer_relaxation": 1.1,
                "step_size_persistence": False,
                "diagonal_seed": False,
                "matrix_free_seed": False,
            },
        },
        "paper_cpu_contract": {
            "source": "Splitting Exact Coulomb Friction, Appendix B and Table 5",
            "host": "one Apple-silicon Mac; exact model not published",
            "backend": "Warp and Newton CPU backend",
            "precision": "float32",
            "execution": "sequential and uncontended",
            "requested_simulation_threads": 1,
            "designated_reconstructed_frontend": "native",
            "solver": "exact_fbf",
            "time_step_seconds": 1.0 / 60.0,
            "timing_statistic": "mean wall time per step over the run",
            "timer_boundary_caveat": (
                "the paper does not state a boundary more precise than wall "
                "time per step"
            ),
            "tolerance": 1e-6,
            "outer_iteration_cap": 200,
            "accept_outer_iteration_cap": True,
            "adaptive_step_size": True,
            "inner_block_gauss_seidel_sweeps": {
                "default": 10,
                "masonry_arch": 30,
            },
            "inner_local_solver": {
                "mode": "exact_metric",
                "contract": (
                    "rigorous DART local cone-QP solve; the paper's local 3x3 "
                    "kernel formula is unpublished, so kernel equivalence and "
                    "apples-to-apples timing remain unverified"
                ),
            },
            "step_size_persistence": {
                "enabled": True,
                "growth_factor": 1.05,
                "contract": (
                    "the paper says gamma is slowly restored after stable steps "
                    "but does not publish its recovery rule"
                ),
            },
            "warm_start": "previous solution when contacts are unchanged",
            "projected_gradient_retry": False,
            "dense_residual_polish": False,
            "boxed_lcp_fallback": False,
            "diagonal_seed": False,
            "matrix_free_seed": False,
            "step_size_scale": 1.0,
            "outer_relaxation": 1.0,
            "initial_gamma_contract": "automatic_safe_bound",
            "split_impulse": {
                "enabled_scenarios": sorted(PAPER_CPU_SPLIT_IMPULSE_SCENARIOS),
                "contract": (
                    "scenario-specific DART stepping choice; equivalence to "
                    "the paper's position treatment is unpublished"
                ),
            },
            "warmup_exclusion": (
                "none reported for FBF; only Kamino's first card step is "
                "explicitly excluded"
            ),
            "references_ms": PAPER_REFERENCE_MS,
            "references_contacts": PAPER_REFERENCE_CONTACTS,
            "trace_scene_gaps": {
                "card_house_10_level": (
                    "paper CPU target is recorded, but fbf_paper_trace has no "
                    "ten-level card-house scenario"
                )
            },
            "trajectory_contracts": {
                scenario: {"steps": steps, "basis": basis}
                for scenario, (steps, basis) in PAPER_TRAJECTORY_CONTRACT.items()
            },
        },
        "trace_full_trajectory_contracts": {
            scenario: {"steps": steps, "basis": basis}
            for scenario, (steps, basis) in SCENARIO_TRAJECTORY_CONTRACT.items()
        },
        "exact_diagnostics_scope": (
            "Public getters describe the last exact constrained group in each "
            "World step; warm_start_matched_fraction divides matched contacts "
            "by the step collision-result contact count."
        ),
        "threading_policy": (
            "requested/actual World thread counts do not prove multicore "
            "execution. Exact counters separately establish contact-row kernel "
            "parallel dispatch. The lifetime CPU-ID union is audit-only because "
            "migration across phases can populate it. A multicore verdict "
            "requires every repetition's largest within-phase CPU-ID set to map "
            "to the requested number of distinct physical cores. CPU IDs are "
            "runtime residency observations, not proof of perfect simultaneity."
        ),
        "scaling_policy": (
            "Validated 1-vs-N speedup additionally requires a nested baseline "
            "CPU list with matching physical-core mapping, one package, matching "
            "SMT sibling topology, cpuinfo maximum-frequency core class, scaling "
            "governor, identical solver/workload options, an exact measured-work "
            "trajectory fingerprint, and successful completion of every "
            "configured warmup. Raw speedup is retained when any gate fails."
        ),
        "argv": list(sys.argv),
        "source_identity": _source_identity(root),
        "binary": {
            "path": str(args.binary.resolve()),
            "sha256": _sha256(args.binary),
        },
        "runtime_identity": _runtime_identity(args.binary),
        "executed_tool_closure": dict(args.executed_tool_closure),
        "executed_tool_identity_rechecks": list(args.executed_tool_identity_rechecks),
        "host": {
            "platform": platform.platform(),
            "uname": list(platform.uname()),
            "python": platform.python_version(),
            "cpu": _cpu_metadata(),
            "memory_kib": _read_text(Path("/proc/meminfo")),
        },
        "toolchain": {
            "cxx_version": _run_text(["c++", "--version"]),
            "cmake_version": _run_text(["cmake", "--version"]),
            "ninja_version": _run_text(["ninja", "--version"]),
        },
        "git": _git_metadata(root),
        "configuration": {
            "cases": [case.__dict__ for case in args.case],
            "threads": list(args.threads),
            "repetitions": args.repetitions,
            "warmup_repetitions": args.warmup_repetitions,
            "solver": args.solver,
            "contract": args.contract,
            "collision_frontend": args.collision_frontend,
            "local_solver": args.local_solver,
            "resolved_local_solver": _resolved_local_solver(args),
            "warm_start": args.warm_start,
            "split_impulse": args.split_impulse,
            "initial_gamma": args.initial_gamma,
            "cpu_list": args.cpu_list,
            "cpu_lists_by_threads": args.cpu_lists_by_threads,
            "timeout_seconds": args.timeout,
        },
    }


def _command(args: argparse.Namespace, invocation: Invocation) -> list[str]:
    command = [
        str(args.binary),
        invocation.case.scenario,
        args.solver,
        "1",
        str(invocation.case.steps),
        args.initial_gamma,
        "performance",
        args.warm_start,
        args.split_impulse,
        str(invocation.threads),
        args.contract,
        args.collision_frontend,
        args.local_solver,
    ]
    cpu_list = args.cpu_lists_by_threads.get(invocation.threads, args.cpu_list)
    if cpu_list:
        taskset_identity = args.executed_tool_closure.get("taskset")
        if not isinstance(taskset_identity, dict):
            raise RuntimeError("CPU pinning requires a bound taskset identity")
        taskset = taskset_identity.get("resolved_path")
        if not isinstance(taskset, str) or not taskset:
            raise RuntimeError("bound taskset identity has no resolved executable path")
        command = [taskset, "--cpu-list", cpu_list, *command]
    return command


def _slug(invocation: Invocation) -> str:
    scenario = re.sub(r"[^A-Za-z0-9_-]", "_", invocation.case.scenario)
    scenario = scenario.strip("_") or "scenario"
    kind = (
        f"warmup{invocation.repetition:03d}"
        if invocation.warmup
        else f"rep{invocation.repetition:03d}"
    )
    return f"{scenario}-n{invocation.case.steps}" f"-t{invocation.threads}-{kind}"


def _raw_output_path(raw_dir: Path, filename: str) -> Path:
    if Path(filename).is_absolute():
        raise ValueError(f"raw evidence filename must be relative: {filename!r}")
    resolved_dir = raw_dir.resolve()
    resolved_path = (resolved_dir / filename).resolve()
    if resolved_path.parent != resolved_dir:
        raise ValueError(f"raw evidence path escapes output directory: {filename!r}")
    return resolved_path


def _strict_integer(
    row: dict[str, str],
    key: str,
    minimum: int,
    maximum: int,
) -> int:
    if key not in row:
        raise ValueError(f"integer field {key} is missing")
    raw = row[key]
    if not isinstance(raw, str) or re.fullmatch(r"-?[0-9]+", raw) is None:
        raise ValueError(f"integer field {key} is invalid: {raw!r}")
    value = int(raw)
    if value < minimum or value > maximum:
        raise ValueError(
            f"integer field {key} is outside [{minimum}, {maximum}]: {raw!r}"
        )
    return value


def _validate_integer_fields(
    rows: Sequence[dict[str, str]], *, include_run_fields: bool = False
) -> None:
    ranges = dict(_TRACE_INTEGER_RANGES)
    if include_run_fields:
        ranges.update(_RUN_INTEGER_RANGES)
    for row_index, row in enumerate(rows, start=1):
        for key, (minimum, maximum) in ranges.items():
            if key in TRACE_COLORED_BGS_COLUMNS_V8 and key not in row:
                continue
            try:
                _strict_integer(row, key, minimum, maximum)
            except ValueError as error:
                raise ValueError(f"row {row_index}: {error}") from error


def _strict_logical_cpu_ids(row: dict[str, str], key: str) -> tuple[int, ...]:
    if key not in row:
        raise ValueError(f"logical CPU residency field {key} is missing")
    raw = row[key]
    if raw == "none":
        return ()
    if (
        not isinstance(raw, str)
        or re.fullmatch(r"(?:0|[1-9][0-9]*)(?:;(?:0|[1-9][0-9]*))*", raw) is None
    ):
        raise ValueError(f"logical CPU residency field {key} is invalid: {raw!r}")
    logical_cpus = tuple(int(value) for value in raw.split(";"))
    if logical_cpus != tuple(sorted(set(logical_cpus))):
        raise ValueError(
            f"logical CPU residency field {key} must be sorted and unique: {raw!r}"
        )
    return logical_cpus


def _validate_logical_cpu_residency_fields(
    rows: Sequence[dict[str, str]],
) -> None:
    for row_index, row in enumerate(rows, start=1):
        keys = [
            "exact_contact_row_logical_cpus_to_date",
            "max_phase_contact_row_logical_cpus_to_date",
        ]
        if "exact_colored_bgs_logical_cpus" in row:
            keys.extend(
                (
                    "exact_colored_bgs_logical_cpus",
                    "max_phase_exact_colored_bgs_logical_cpus",
                )
            )
        for key in keys:
            try:
                _strict_logical_cpu_ids(row, key)
            except ValueError as error:
                raise ValueError(f"row {row_index}: {error}") from error


def _parse_trace(stdout: str, contract: str = "dart_best") -> list[dict[str, str]]:
    reader = csv.DictReader(stdout.splitlines())
    expected_columns = (
        TRACE_COLORED_COLUMNS if contract == "dart_best_colored_bgs" else TRACE_COLUMNS
    )
    if tuple(reader.fieldnames or ()) != expected_columns:
        raise ValueError(
            "unexpected fbf_paper_trace performance schema for evidence v"
            f"{SCHEMA_VERSION}: {reader.fieldnames!r}"
        )
    rows = list(reader)
    if any(None in row for row in rows):
        raise ValueError("malformed performance CSV row")
    _validate_integer_fields(rows)
    _validate_logical_cpu_residency_fields(rows)
    return rows


def _expected_solver_contract(contract: str, threads: int) -> str:
    if contract == "dart_best_colored_bgs":
        return (
            "dart_best_nonpaper_colored_inner_bgs"
            if threads == 1
            else "dart_best_nonpaper_colored_inner_bgs_threaded_world"
        )
    if threads == 1:
        return contract
    if contract == "paper_cpu":
        return "paper_cpu_parameters_threaded_world_serial_fbf"
    return "dart_best_threaded_world_exact_contact_row_parallel_capable"


def _expected_split_impulse(args: argparse.Namespace, scenario: str) -> str:
    if args.split_impulse in {"0", "1"}:
        return args.split_impulse
    return "1" if scenario in TRACE_SPLIT_IMPULSE_SCENARIOS else "0"


def _resolved_local_solver(args: argparse.Namespace) -> str:
    if args.solver != "exact_fbf":
        return "not_applicable"
    return "exact_metric" if args.local_solver == "default" else args.local_solver


def _validate_colored_bgs_rows(
    rows: Sequence[dict[str, str]],
    args: argparse.Namespace,
    invocation: Invocation,
) -> None:
    if args.contract != "dart_best_colored_bgs":
        return

    allowed_logical_cpus = set(_invocation_affinity(args, invocation)["logical_cpus"])
    for row in rows:
        if (
            row["inner_bgs_schedule_contract"]
            != "dart_deterministic_manifold_colored_bgs_nonpaper"
        ):
            raise ValueError("trace colored inner-BGS schedule contract mismatch")

        if _int(row, "step_parallel_contact_row_delassus_products") != 0:
            raise ValueError(
                "trace colored contract reports legacy parallel contact-row work"
            )
        if _int(row, "max_contact_row_participants_to_date") > 1:
            raise ValueError(
                "trace colored contract reports multiple legacy contact-row participants"
            )
        if _strict_logical_cpu_ids(
            row, "exact_contact_row_logical_cpus_to_date"
        ) or _strict_logical_cpu_ids(row, "max_phase_contact_row_logical_cpus_to_date"):
            raise ValueError(
                "trace colored contract reports legacy contact-row CPU residency"
            )

        exact_attempts = _int(row, "step_exact_solves") + _int(
            row, "step_exact_failures"
        )
        used = _int(row, "last_exact_colored_bgs_used")
        solves = _int(row, "last_exact_colored_bgs_solves")
        dispatches = _int(row, "last_exact_colored_bgs_dispatches")
        participants = _int(row, "last_exact_colored_bgs_max_participants")
        manifolds = _int(row, "last_exact_colored_bgs_manifolds")
        colors = _int(row, "last_exact_colored_bgs_colors")
        max_manifolds_per_color = _int(
            row, "last_exact_colored_bgs_max_manifolds_per_color"
        )
        logical_cpus = set(
            _strict_logical_cpu_ids(row, "exact_colored_bgs_logical_cpus")
        )
        max_phase_cpus = set(
            _strict_logical_cpu_ids(row, "max_phase_exact_colored_bgs_logical_cpus")
        )
        work_counters = (solves, dispatches, participants)
        schedule_counters = (
            manifolds,
            colors,
            max_manifolds_per_color,
        )
        counters = (*work_counters, *schedule_counters)

        if exact_attempts != 1:
            if used != -1 or any(counters) or logical_cpus or max_phase_cpus:
                raise ValueError(
                    "trace colored diagnostics must be unavailable unless "
                    "exactly one exact group ran"
                )
            continue
        if (
            invocation.case.scenario == LITERAL_WEDGE_ARCH_SCENARIO
            and manifolds != LITERAL_WEDGE_ARCH_COLLIDING_BODY_PAIR_COUNT
        ):
            raise ValueError(
                "trace literal-wedge exact attempt must retain its 24-manifold "
                "colored schedule"
            )
        if used == -1:
            raise ValueError(
                "trace colored diagnostics unavailable after exact attempt"
            )
        if used == 0:
            # A warm start can satisfy the outer residual before the first
            # frozen-cone solve. The colored schedule is still eligible and
            # therefore remains observable. A multi-thread attempt enters its
            # persistent participant dispatch before the math layer performs
            # that initial residual check, so dispatch/residency can be real
            # even though no colored frozen-cone solve ran.
            if _int(row, "step_fbf_iterations") != 0:
                raise ValueError(
                    "trace unused colored path on positive-iteration exact attempt"
                )
            if solves != 0:
                raise ValueError("trace unused colored path reports frozen-cone solves")
            schedule_eligible = any(schedule_counters)
            if schedule_eligible and (
                manifolds <= 0
                or colors <= 0
                or max_manifolds_per_color <= 0
                or colors > manifolds
                or max_manifolds_per_color > manifolds
            ):
                raise ValueError(
                    "trace unused colored path reports inconsistent schedule dimensions"
                )
            actual_threads = _int(row, "actual_threads")
            if not schedule_eligible:
                if (
                    dispatches != 0
                    or participants != 0
                    or logical_cpus
                    or max_phase_cpus
                ):
                    raise ValueError(
                        "trace ineligible colored schedule reports dispatch diagnostics"
                    )
            elif actual_threads == 1:
                if (
                    dispatches != 0
                    or participants != 0
                    or logical_cpus
                    or max_phase_cpus
                ):
                    raise ValueError(
                        "trace zero-iteration one-thread colored attempt reports dispatch"
                    )
            else:
                if dispatches != 1 or participants != actual_threads:
                    raise ValueError(
                        "trace zero-iteration colored dispatch is inconsistent"
                    )
                if not logical_cpus.issubset(allowed_logical_cpus):
                    raise ValueError("trace colored logical CPUs fall outside affinity")
                if not max_phase_cpus.issubset(logical_cpus):
                    raise ValueError(
                        "trace colored max-phase CPUs are not a residency subset"
                    )
                if (
                    len(logical_cpus) > participants
                    or len(max_phase_cpus) > participants
                ):
                    raise ValueError("trace colored logical CPUs exceed participants")
            continue

        actual_threads = _int(row, "actual_threads")
        if _int(row, "step_fbf_iterations") <= 0:
            raise ValueError("trace colored path used without outer iterations")
        if solves <= 0:
            raise ValueError("trace colored path used without frozen-cone solves")
        if participants != actual_threads:
            raise ValueError("trace colored participants do not match actual threads")
        if manifolds <= 0 or colors <= 0 or max_manifolds_per_color <= 0:
            raise ValueError("trace colored schedule dimensions must be positive")
        if colors > manifolds or max_manifolds_per_color > manifolds:
            raise ValueError("trace colored schedule dimensions are inconsistent")
        if max_manifolds_per_color < invocation.threads:
            raise ValueError(
                "trace colored schedule width is below the requested thread count"
            )
        if actual_threads == 1:
            if dispatches != 0 or logical_cpus or max_phase_cpus:
                raise ValueError(
                    "trace one-thread colored baseline must not report pool residency"
                )
        elif dispatches != 1:
            raise ValueError(
                "trace colored persistent-dispatch count must be exactly one"
            )
        if not logical_cpus.issubset(allowed_logical_cpus):
            raise ValueError("trace colored logical CPUs fall outside affinity")
        if not max_phase_cpus.issubset(logical_cpus):
            raise ValueError("trace colored max-phase CPUs are not a residency subset")
        if len(logical_cpus) > participants:
            raise ValueError("trace colored logical CPUs exceed participants")
        if len(max_phase_cpus) > participants:
            raise ValueError("trace colored max-phase CPUs exceed participants")


def _validate_invocation_rows(
    rows: Sequence[dict[str, str]],
    args: argparse.Namespace,
    invocation: Invocation,
) -> None:
    _validate_integer_fields(rows)
    _validate_logical_cpu_residency_fields(rows)
    if len(rows) != invocation.case.steps:
        raise ValueError(
            f"trace emitted {len(rows)} rows for {invocation.case.steps} steps"
        )
    actual_steps = [_int(row, "step") for row in rows]
    expected_steps = list(range(1, invocation.case.steps + 1))
    if actual_steps != expected_steps:
        raise ValueError(
            f"trace step sequence mismatch: {actual_steps!r} != {expected_steps!r}"
        )

    expected_fields = {
        "scenario": invocation.case.scenario,
        "solver": args.solver,
        "solver_contract": _expected_solver_contract(args.contract, invocation.threads),
        "collision_frontend": args.collision_frontend,
        "requested_threads": str(invocation.threads),
    }
    if invocation.case.scenario == LITERAL_WEDGE_ARCH_SCENARIO:
        expected_fields["scene_contract"] = LITERAL_WEDGE_ARCH_SCENE_CONTRACT
    for key, expected in expected_fields.items():
        actual = {row[key] for row in rows}
        if actual != {expected}:
            raise ValueError(
                f"trace {key} mismatch: {sorted(actual)!r} != {expected!r}"
            )

    if args.solver == "exact_fbf":
        if args.contract == "paper_cpu":
            expected_local_solver = "exact_metric"
            expected_inner_sweeps = (
                "30" if invocation.case.scenario.startswith("masonry_arch_") else "10"
            )
            expected_fixed_inner_sweeps = "1"
            expected_step_size_persistence = "1"
            expected_row_operator_request = "contact_row_no_dense_snapshot"
            expected_contract_fields = {
                "max_outer_iterations": "200",
                "accept_outer_max_iterations": "1",
                "inner_local_iterations": "1",
                "adaptive_step_size_enabled": "1",
                "warm_start_enabled": "1",
                "projected_gradient_retry_enabled": "0",
                "dense_residual_polish_enabled": "0",
                "fallback_to_boxed_lcp_enabled": "0",
                "diagonal_seed_enabled": "0",
                "matrix_free_seed_enabled": "0",
                "initial_gamma_contract": "automatic_safe_bound",
                "split_impulse_enabled": _expected_split_impulse(
                    args, invocation.case.scenario
                ),
            }
        elif (
            args.contract == "dart_best_colored_bgs"
            and invocation.case.scenario == LITERAL_WEDGE_ARCH_SCENARIO
        ):
            expected_local_solver = "exact_metric"
            expected_inner_sweeps = "30"
            expected_fixed_inner_sweeps = "1"
            expected_step_size_persistence = "0"
            expected_row_operator_request = "contact_row_no_dense_snapshot"
            expected_contract_fields = {
                "max_outer_iterations": "5000",
                "accept_outer_max_iterations": "1",
                "inner_local_iterations": "1",
                "adaptive_step_size_enabled": "1",
                "warm_start_enabled": "1",
                "projected_gradient_retry_enabled": "0",
                "dense_residual_polish_enabled": "0",
                "fallback_to_boxed_lcp_enabled": "0",
                "diagonal_seed_enabled": "0",
                "matrix_free_seed_enabled": "0",
                "initial_gamma_contract": "automatic_safe_bound",
                "split_impulse_enabled": _expected_split_impulse(
                    args, invocation.case.scenario
                ),
            }
        else:
            expected_local_solver = _resolved_local_solver(args)
            expected_inner_sweeps = "120"
            expected_fixed_inner_sweeps = "0"
            expected_step_size_persistence = "1"
            expected_row_operator_request = "contact_row_no_dense_snapshot"
            expected_contract_fields = (
                {
                    "accept_outer_max_iterations": "1",
                    "projected_gradient_retry_enabled": "0",
                    "dense_residual_polish_enabled": "0",
                    "fallback_to_boxed_lcp_enabled": "0",
                    "diagonal_seed_enabled": "1",
                    "matrix_free_seed_enabled": "1",
                }
                if args.contract == "dart_best_colored_bgs"
                else {}
            )
        expected_solver_fields = {
            "inner_local_solver": expected_local_solver,
            "inner_sweeps_requested": expected_inner_sweeps,
            "fixed_inner_sweeps_requested": expected_fixed_inner_sweeps,
            "step_size_persistence_enabled": expected_step_size_persistence,
            "row_operator_request": expected_row_operator_request,
            "split_impulse_enabled": _expected_split_impulse(
                args, invocation.case.scenario
            ),
        }
    else:
        expected_solver_fields = {
            "inner_local_solver": "not_applicable",
            "inner_sweeps_requested": "-1",
            "fixed_inner_sweeps_requested": "-1",
            "step_size_persistence_enabled": "-1",
            "row_operator_request": "not_applicable",
            "row_operator_mode": "boxed_lcp",
            "max_outer_iterations": "-1",
            "tolerance": "nan",
            "accept_outer_max_iterations": "-1",
            "inner_local_iterations": "-1",
            "adaptive_step_size_enabled": "-1",
            "warm_start_enabled": "-1",
            "projected_gradient_retry_enabled": "-1",
            "dense_residual_polish_enabled": "-1",
            "fallback_to_boxed_lcp_enabled": "-1",
            "diagonal_seed_enabled": "-1",
            "matrix_free_seed_enabled": "-1",
            "step_size_scale": "nan",
            "outer_relaxation": "nan",
            "initial_gamma_contract": "not_applicable",
            "split_impulse_enabled": _expected_split_impulse(
                args, invocation.case.scenario
            ),
            "step_contact_row_delassus_products": "0",
            "step_parallel_contact_row_delassus_products": "0",
            "max_contact_row_participants_to_date": "0",
            "exact_contact_row_logical_cpus_to_date": "none",
            "max_phase_contact_row_logical_cpus_to_date": "none",
        }
        expected_contract_fields = {}

    for key, expected in expected_solver_fields.items():
        actual = {row[key] for row in rows}
        if actual != {expected}:
            raise ValueError(
                f"trace {key} mismatch: {sorted(actual)!r} != {expected!r}"
            )

    for key, expected in expected_contract_fields.items():
        actual = {row[key] for row in rows}
        if actual != {expected}:
            raise ValueError(
                f"trace configured contract {key} mismatch: "
                f"{sorted(actual)!r} != {expected!r}"
            )

    _validate_colored_bgs_rows(rows, args, invocation)

    if args.solver == "exact_fbf":
        previous_max_participants = 0
        previous_logical_cpus: set[int] = set()
        previous_max_phase_cpu_count = 0
        previous_max_phase_cpus: set[int] = set()
        invocation_affinity = _invocation_affinity(args, invocation)
        allowed_logical_cpus = set(invocation_affinity["logical_cpus"])
        for key in ("tolerance", "step_size_scale", "outer_relaxation"):
            values = [_float(row, key) for row in rows]
            if any(not math.isfinite(value) or value <= 0.0 for value in values):
                raise ValueError(f"trace {key} must be positive and finite: {values!r}")
        if args.contract == "paper_cpu":
            paper_float_fields = {
                "tolerance": 1e-6,
                "step_size_scale": 1.0,
                "outer_relaxation": 1.0,
            }
            for key, expected in paper_float_fields.items():
                values = [_float(row, key) for row in rows]
                if any(
                    not math.isclose(value, expected, rel_tol=1e-12, abs_tol=0.0)
                    for value in values
                ):
                    raise ValueError(
                        f"trace paper_cpu {key} mismatch: "
                        f"{values!r} != {expected!r}"
                    )
        elif (
            args.contract == "dart_best_colored_bgs"
            and invocation.case.scenario == LITERAL_WEDGE_ARCH_SCENARIO
        ):
            literal_float_fields = {
                "tolerance": 1e-6,
                "step_size_scale": 35.0,
                "outer_relaxation": 1.1,
            }
            for key, expected in literal_float_fields.items():
                values = [_float(row, key) for row in rows]
                if any(
                    not math.isclose(value, expected, rel_tol=1e-12, abs_tol=0.0)
                    for value in values
                ):
                    raise ValueError(
                        f"trace literal-wedge colored {key} mismatch: "
                        f"{values!r} != {expected!r}"
                    )

    if args.solver == "exact_fbf":
        growth_factors = [
            _float(row, "step_size_recovery_growth_factor") for row in rows
        ]
        if any(not math.isclose(value, 1.05) for value in growth_factors):
            raise ValueError(
                "trace step_size_recovery_growth_factor mismatch: "
                f"{growth_factors!r} != 1.05"
            )
        for row in rows:
            exact_attempts = _int(row, "step_exact_solves") + _int(
                row, "step_exact_failures"
            )
            row_products = _int(row, "step_contact_row_delassus_products")
            parallel_products = _int(row, "step_parallel_contact_row_delassus_products")
            max_participants = _int(row, "max_contact_row_participants_to_date")
            actual_threads = _int(row, "actual_threads")
            logical_cpus = set(
                _strict_logical_cpu_ids(row, "exact_contact_row_logical_cpus_to_date")
            )
            max_phase_cpus = set(
                _strict_logical_cpu_ids(
                    row, "max_phase_contact_row_logical_cpus_to_date"
                )
            )
            if parallel_products > row_products:
                raise ValueError(
                    "trace parallel contact-row products exceed total products"
                )
            if max_participants > actual_threads:
                raise ValueError("trace contact-row participants exceed actual threads")
            if parallel_products > 0 and max_participants <= 1:
                raise ValueError(
                    "trace reports parallel contact-row products without "
                    "multiple participants"
                )
            if actual_threads == 1 and (parallel_products > 0 or max_participants > 1):
                raise ValueError(
                    "trace reports exact-kernel parallelism with one actual thread"
                )
            if max_participants < previous_max_participants:
                raise ValueError(
                    "trace cumulative contact-row participant maximum decreased"
                )
            previous_max_participants = max_participants
            if not previous_logical_cpus.issubset(logical_cpus):
                raise ValueError(
                    "trace cumulative exact contact-row logical CPU set decreased"
                )
            if logical_cpus - previous_logical_cpus and parallel_products == 0:
                raise ValueError(
                    "trace cumulative exact contact-row logical CPU set grew "
                    "without a parallel contact-row product"
                )
            if not logical_cpus.issubset(allowed_logical_cpus):
                unexpected = sorted(logical_cpus - allowed_logical_cpus)
                raise ValueError(
                    "trace exact contact-row logical CPUs fall outside invocation "
                    f"affinity: {unexpected!r}"
                )
            if not max_phase_cpus.issubset(logical_cpus):
                raise ValueError(
                    "trace max-phase logical CPU set is not a subset of the "
                    "cumulative residency set"
                )
            if len(max_phase_cpus) < previous_max_phase_cpu_count:
                raise ValueError(
                    "trace cumulative max-phase logical CPU count decreased"
                )
            if max_phase_cpus != previous_max_phase_cpus and parallel_products == 0:
                raise ValueError(
                    "trace cumulative max-phase logical CPU set changed without "
                    "a parallel contact-row product"
                )
            if len(max_phase_cpus) > max_participants:
                raise ValueError(
                    "trace max-phase logical CPU count exceeds participant maximum"
                )
            previous_logical_cpus = logical_cpus
            previous_max_phase_cpu_count = len(max_phase_cpus)
            previous_max_phase_cpus = max_phase_cpus
            persistence_used = _int(row, "step_size_persistence_used")
            persistence_request = _float(row, "step_size_persistence_request")
            if exact_attempts == 0:
                if row_products != 0 or parallel_products != 0:
                    raise ValueError(
                        "trace reports contact-row products when no exact group ran"
                    )
                if persistence_used != -1 or math.isfinite(persistence_request):
                    raise ValueError(
                        "trace persisted step-size diagnostics must be unavailable "
                        "when no exact group ran"
                    )
                if row["row_operator_mode"] != "not_run":
                    raise ValueError(
                        "trace row_operator_mode must be not_run when no exact group ran"
                    )
            else:
                if persistence_used not in {0, 1}:
                    raise ValueError(
                        "trace step_size_persistence_used must be 0 or 1 after an "
                        "exact attempt"
                    )
                if persistence_used == 1 and not (
                    math.isfinite(persistence_request) and persistence_request > 0.0
                ):
                    raise ValueError(
                        "trace used persisted step size without a positive finite request"
                    )
                if persistence_used == 0 and math.isfinite(persistence_request):
                    raise ValueError(
                        "trace emitted a persisted step-size request when persistence "
                        "was not used"
                    )
                if (
                    args.contract == "dart_best_colored_bgs"
                    and invocation.case.scenario == LITERAL_WEDGE_ARCH_SCENARIO
                    and persistence_used != 0
                ):
                    raise ValueError(
                        "trace literal-wedge colored contract used disabled "
                        "step-size persistence"
                    )
                if row["row_operator_mode"] in {"not_run", "boxed_lcp"}:
                    raise ValueError(
                        "trace row_operator_mode is unavailable after an exact attempt"
                    )
                if row_products > 0 and not row["row_operator_mode"].startswith(
                    "contact_row_"
                ):
                    raise ValueError(
                        "trace reports contact-row products for a non-contact-row mode"
                    )


def _run_invocation(
    args: argparse.Namespace,
    invocation: Invocation,
    raw_dir: Path,
) -> tuple[list[dict[str, str]], dict[str, object]]:
    command = _command(args, invocation)
    affinity = _invocation_affinity(args, invocation)
    paper_hardware_contract = _paper_hardware_contract()
    timed_out = False
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=args.timeout,
        )
        returncode = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except subprocess.TimeoutExpired as error:
        timed_out = True
        returncode = 124
        stdout = error.stdout or ""
        stderr = error.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="replace")

    _require_executed_tool_closure_unchanged(args, invocation)

    slug = _slug(invocation)
    stdout_path = _raw_output_path(raw_dir, f"{slug}.csv")
    stderr_path = _raw_output_path(raw_dir, f"{slug}.stderr.txt")
    stdout_path.write_text(stdout, encoding="utf-8")
    stderr_path.write_text(stderr, encoding="utf-8")
    rows = _parse_trace(stdout, args.contract) if stdout.strip() else []
    if len(rows) == invocation.case.steps:
        _validate_invocation_rows(rows, args, invocation)
    elif returncode == 0:
        _validate_invocation_rows(rows, args, invocation)
    for row in rows:
        row["repetition"] = str(invocation.repetition)
        row["process_returncode"] = str(returncode)
        row["warmup"] = "1" if invocation.warmup else "0"
        row["affinity_source"] = str(affinity["source"])
        row["affinity_logical_cpus"] = ",".join(
            str(cpu) for cpu in affinity["logical_cpus"]
        )
        row["affinity_logical_cpu_count"] = str(affinity["logical_cpu_count"])
        row["affinity_physical_core_count"] = (
            str(affinity["physical_core_count"])
            if affinity["physical_core_count"] is not None
            else "unknown"
        )
        row["affinity_one_logical_per_physical_core"] = (
            str(affinity["one_logical_per_physical_core"]).lower()
            if affinity["one_logical_per_physical_core"] is not None
            else "unknown"
        )
        row["affinity_physical_core_keys"] = (
            ";".join(str(value) for value in affinity["physical_core_keys"])
            if affinity["physical_core_keys"] is not None
            else "unknown"
        )
        logical_cpu_physical_core_keys = affinity["logical_cpu_physical_core_keys"]
        row["affinity_logical_cpu_physical_core_keys"] = (
            ";".join(
                f"{cpu}={logical_cpu_physical_core_keys[str(cpu)]}"
                for cpu in affinity["logical_cpus"]
            )
            if isinstance(logical_cpu_physical_core_keys, dict)
            and all(
                isinstance(logical_cpu_physical_core_keys.get(str(cpu)), str)
                for cpu in affinity["logical_cpus"]
            )
            else "unknown"
        )
        row["affinity_package_ids"] = (
            ";".join(str(value) for value in affinity["package_ids"])
            if affinity["package_ids"] is not None
            else "unknown"
        )
        row["affinity_package_count"] = (
            str(affinity["package_count"])
            if affinity["package_count"] is not None
            else "unknown"
        )
        row["affinity_smt_sibling_counts"] = (
            ";".join(str(value) for value in affinity["smt_sibling_counts"])
            if affinity["smt_sibling_counts"] is not None
            else "unknown"
        )
        row["affinity_core_classes_khz"] = (
            ";".join(str(value) for value in affinity["core_classes_khz"])
            if affinity["core_classes_khz"] is not None
            else "unknown"
        )
        row["affinity_scaling_governors"] = (
            ";".join(str(value) for value in affinity["scaling_governors"])
            if affinity["scaling_governors"] is not None
            else "unknown"
        )
        row["paper_hardware_contract"] = paper_hardware_contract

    record = {
        "scenario": invocation.case.scenario,
        "collision_frontend": args.collision_frontend,
        "cpu_list": args.cpu_lists_by_threads.get(invocation.threads, args.cpu_list),
        "cpu_affinity": affinity,
        "paper_hardware_contract": paper_hardware_contract,
        "steps": invocation.case.steps,
        "threads": invocation.threads,
        "repetition": invocation.repetition,
        "warmup": invocation.warmup,
        "command": command,
        "returncode": returncode,
        "timed_out": timed_out,
        "rows": len(rows),
        "expected_rows": invocation.case.steps,
        "complete_rows": len(rows) == invocation.case.steps,
        "stdout_file": f"raw/{slug}.csv",
        "stderr_file": f"raw/{slug}.stderr.txt",
    }
    return rows, record


def _float(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return math.nan


def _int(row: dict[str, str], key: str) -> int:
    ranges = _TRACE_INTEGER_RANGES | _RUN_INTEGER_RANGES
    try:
        minimum, maximum = ranges[key]
    except KeyError as error:
        raise ValueError(f"no strict integer contract for field {key}") from error
    return _strict_integer(row, key, minimum, maximum)


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _unique(rows: Sequence[dict[str, str]], key: str) -> str:
    values = sorted({row.get(key, "unknown") for row in rows})
    return ";".join(values)


def _consistent_row_value(rows: Sequence[dict[str, str]], key: str) -> str | None:
    values = {row.get(key, "unknown") for row in rows}
    return next(iter(values)) if len(values) == 1 else None


def _consistent_measured_affinity(
    records: Sequence[dict[str, object]],
) -> dict[str, object] | None:
    affinities = [
        record.get("cpu_affinity")
        for record in records
        if not bool(record.get("warmup"))
        and isinstance(record.get("cpu_affinity"), dict)
    ]
    if not affinities:
        return None
    first = affinities[0]
    return first if all(affinity == first for affinity in affinities) else None


def _physical_core_mapping(
    affinity: dict[str, object] | None,
) -> dict[int, str] | None:
    if affinity is None:
        return None
    raw_mapping = affinity.get("logical_cpu_physical_core_keys")
    if not isinstance(raw_mapping, dict):
        return None
    mapping: dict[int, str] = {}
    for raw_cpu, raw_key in raw_mapping.items():
        if not isinstance(raw_cpu, str) or not raw_cpu.isdigit():
            return None
        if not isinstance(raw_key, str) or not raw_key:
            return None
        mapping[int(raw_cpu)] = raw_key
    return mapping


def _parse_encoded_physical_core_mapping(value: object) -> dict[int, str] | None:
    if not isinstance(value, str) or not value or value == "unknown":
        return None
    mapping: dict[int, str] = {}
    for item in value.split(";"):
        match = re.fullmatch(r"(0|[1-9][0-9]*)=([^:=;]+):([^:=;]+)", item)
        if match is None:
            return None
        cpu = int(match.group(1))
        if cpu in mapping:
            return None
        mapping[cpu] = f"{match.group(2)}:{match.group(3)}"
    return mapping


def _measured_workload_fingerprint(rows: Sequence[dict[str, str]]) -> str:
    ordered_rows = sorted(
        rows,
        key=lambda row: (_int(row, "repetition"), _int(row, "step")),
    )
    colored_contract = any(
        row.get("solver_contract", "").startswith(
            "dart_best_nonpaper_colored_inner_bgs"
        )
        for row in ordered_rows
    )
    fields = tuple(
        field
        for field in SCALING_MEASURED_WORK_FIELDS
        if colored_contract or field not in SCALING_COLORED_MEASURED_WORK_FIELDS
    )
    payload = [[row.get(field, "") for field in fields] for row in ordered_rows]
    canonical = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _finite_values(rows: Sequence[dict[str, str]], key: str) -> list[float]:
    values = [_float(row, key) for row in rows]
    return [value for value in values if math.isfinite(value)]


def _incline_expected_state() -> tuple[float, float, float, float]:
    theta = math.atan(INCLINE_TAN)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    initial_x = -sin_theta * INCLINE_INITIAL_NORMAL_OFFSET
    initial_z = cos_theta * INCLINE_INITIAL_NORMAL_OFFSET
    return initial_x, initial_z, -cos_theta, -sin_theta


def _incline_sliding_displacement() -> float:
    theta = math.atan(INCLINE_TAN)
    acceleration = GRAVITY * (
        math.sin(theta) - INCLINE_SLIDE_FRICTION * math.cos(theta)
    )
    return 0.5 * acceleration * 2.0 * 2.0


def _backspin_expected_linear_velocity() -> float:
    sphere_inertia_over_mass_radius_squared = 2.0 / 5.0
    impulse_over_mass = (
        BACKSPIN_RADIUS * BACKSPIN_INITIAL_ANGULAR_VELOCITY
        - BACKSPIN_INITIAL_LINEAR_VELOCITY
    ) / (1.0 + 1.0 / sphere_inertia_over_mass_radius_squared)
    return BACKSPIN_INITIAL_LINEAR_VELOCITY + impulse_over_mass


def _physical_outcome_for_final_row(
    scenario: str, row: dict[str, str]
) -> tuple[bool, str]:
    tracked_body = row.get("tracked_body", "")
    expected_body = PHYSICAL_OUTCOME_TRACKED_BODIES[scenario]
    if tracked_body != expected_body:
        return False, f"tracked_body={tracked_body},expected={expected_body}"

    x = _float(row, "x")
    y = _float(row, "y")
    z = _float(row, "z")
    vx = _float(row, "vx")
    vy = _float(row, "vy")
    up_z = _float(row, "up_z")
    contacts = _int(row, "contacts")

    if scenario.startswith("incline_mu_"):
        initial_x, initial_z, downhill_x, downhill_z = _incline_expected_state()
        displacement = (x - initial_x) * downhill_x + (z - initial_z) * downhill_z
        if scenario == "incline_mu_0_5":
            expected = 0.0
            tolerance = INCLINE_STICK_DISPLACEMENT_TOLERANCE
        else:
            expected = _incline_sliding_displacement()
            tolerance = INCLINE_SLIDE_DISPLACEMENT_TOLERANCE
        passed = (
            math.isfinite(displacement) and abs(displacement - expected) <= tolerance
        )
        return (
            passed,
            f"displacement={displacement:.9g},expected={expected:.9g},"
            f"tolerance={tolerance:.9g}",
        )

    if scenario == "backspin":
        expected_vx = _backspin_expected_linear_velocity()
        passed = (
            math.isfinite(vx)
            and abs(vx - expected_vx) <= BACKSPIN_LINEAR_VELOCITY_TOLERANCE
            and math.isfinite(z)
            and abs(z - BACKSPIN_RADIUS) <= BACKSPIN_HEIGHT_TOLERANCE
            and contacts > 0
        )
        return (
            passed,
            f"vx={vx:.9g},expected_vx={expected_vx:.9g},"
            f"vx_tolerance={BACKSPIN_LINEAR_VELOCITY_TOLERANCE:.9g},"
            f"height={z:.9g},height_expected={BACKSPIN_RADIUS:.9g},"
            f"height_tolerance={BACKSPIN_HEIGHT_TOLERANCE:.9g},"
            f"contacts={contacts}",
        )

    if scenario.startswith("turntable_"):
        radius = math.hypot(x, y)
        captured_expected = scenario == "turntable_mu_0_5_omega_2"
        radial_velocity = math.nan
        tangential_velocity = math.nan
        corotation_speed_error = math.nan
        if captured_expected:
            if radius > 1e-12 and all(math.isfinite(value) for value in (x, y, vx, vy)):
                radial_velocity = (x * vx + y * vy) / radius
                tangential_velocity = (-y * vx + x * vy) / radius
                expected_tangential_velocity = 2.0 * radius
                corotation_speed_error = abs(
                    tangential_velocity - expected_tangential_velocity
                )
            passed = (
                math.isfinite(radius)
                and radius < TURNTABLE_CAPTURE_RADIUS_MAX
                and math.isfinite(z)
                and z > TURNTABLE_CAPTURE_HEIGHT_MIN
                and contacts > 0
                and math.isfinite(radial_velocity)
                and abs(radial_velocity) <= TURNTABLE_CAPTURE_RADIAL_VELOCITY_MAX
                and math.isfinite(corotation_speed_error)
                and corotation_speed_error
                <= TURNTABLE_CAPTURE_COROTATION_SPEED_ERROR_MAX
            )
            expected = "captured"
        else:
            passed = (
                math.isfinite(radius)
                and math.isfinite(z)
                and (radius > TURNTABLE_EJECTION_RADIUS_MIN or z < 0.0)
            )
            expected = "ejected"
        return (
            passed,
            f"expected={expected},radius={radius:.9g},height={z:.9g},"
            f"contacts={contacts},radial_velocity={radial_velocity:.9g},"
            f"tangential_velocity={tangential_velocity:.9g},"
            f"corotation_speed_error={corotation_speed_error:.9g}",
        )

    if scenario == "painleve_mu_0_5":
        passed = (
            math.isfinite(up_z)
            and up_z > PAINLEVE_UPRIGHT_UP_Z_MIN
            and math.isfinite(z)
            and z > PAINLEVE_HEIGHT_THRESHOLD
        )
        return (
            passed,
            f"expected=upright,up_z={up_z:.9g},height={z:.9g}",
        )

    passed = (
        math.isfinite(up_z)
        and math.isfinite(z)
        and (up_z < PAINLEVE_TUMBLED_UP_Z_MAX or z < PAINLEVE_HEIGHT_THRESHOLD)
    )
    return passed, f"expected=tumbled,up_z={up_z:.9g},height={z:.9g}"


def _card_house_full_outcome(
    rows: Sequence[dict[str, str]],
) -> tuple[bool, str]:
    settle_rows = rows[:CARD_HOUSE_FULL_SETTLE_STEPS]
    projectile_rows = rows[CARD_HOUSE_FULL_SETTLE_STEPS:]
    if not settle_rows or not projectile_rows:
        return False, "missing_settle_or_projectile_phase"

    card_count_valid = all(
        _int(row, "card_count") == CARD_HOUSE_FULL_CARD_COUNT for row in rows
    )
    finite_state_valid = all(_int(row, "finite_state") == 1 for row in rows)
    tracked_body_valid = all(
        row.get("tracked_body", "")
        == PHYSICAL_OUTCOME_TRACKED_BODIES["card_house_26_settle_projectile_full"]
        for row in rows
    )
    settle_phase_valid = all(
        row.get("phase", "") == "settle" and _int(row, "projectile_count") == 0
        for row in settle_rows
    )
    projectile_phase_valid = all(
        row.get("phase", "") == "projectile"
        and _int(row, "projectile_count") == CARD_HOUSE_FULL_PROJECTILE_COUNT
        for row in projectile_rows
    )

    metric_fields = (
        "min_card_axis_up",
        "min_center_height",
        "max_card_horizontal_travel",
        "max_projectile_speed",
        "max_card_center_displacement_from_initial",
        "min_card_orientation_alignment_from_initial",
    )
    finite_metrics_valid = all(
        all(math.isfinite(_float(row, field)) for field in metric_fields)
        for row in rows
    )

    first_settle = settle_rows[0]
    final_settle = settle_rows[-1]
    final = projectile_rows[-1]
    initial_horizontal = _float(first_settle, "max_card_horizontal_travel")
    settle_axis = _float(final_settle, "min_card_axis_up")
    settle_height = _float(final_settle, "min_center_height")
    settle_horizontal = _float(final_settle, "max_card_horizontal_travel")
    final_axis = _float(final, "min_card_axis_up")
    final_height = _float(final, "min_center_height")
    final_horizontal = _float(final, "max_card_horizontal_travel")
    max_projectile_speed = max(
        _float(row, "max_projectile_speed") for row in projectile_rows
    )

    settle_stable = finite_metrics_valid and all(
        _float(row, "min_card_axis_up") > CARD_HOUSE_SETTLE_MIN_AXIS_UP
        and _float(row, "min_center_height") > CARD_HOUSE_SETTLE_MIN_CENTER_HEIGHT
        and abs(_float(row, "max_card_horizontal_travel") - initial_horizontal)
        <= CARD_HOUSE_SETTLE_MAX_HORIZONTAL_DRIFT
        for row in settle_rows
    )
    reconstructed_settle_stable = finite_metrics_valid and all(
        _float(row, "max_card_center_displacement_from_initial")
        <= CARD_HOUSE_SETTLE_MAX_CENTER_DISPLACEMENT
        and _float(row, "min_card_orientation_alignment_from_initial")
        >= CARD_HOUSE_SETTLE_MIN_ORIENTATION_ALIGNMENT
        for row in settle_rows
    )
    projectile_motion = (
        math.isfinite(max_projectile_speed)
        and max_projectile_speed > CARD_HOUSE_PROJECTILE_MIN_SPEED
    )
    axis_response = settle_axis - final_axis >= CARD_HOUSE_PROJECTILE_MIN_AXIS_RESPONSE
    height_response = (
        settle_height - final_height >= CARD_HOUSE_PROJECTILE_MIN_HEIGHT_RESPONSE
    )
    horizontal_response = (
        final_horizontal - settle_horizontal
        >= CARD_HOUSE_PROJECTILE_MIN_HORIZONTAL_RESPONSE
    )
    impact_response = axis_response or height_response or horizontal_response
    first_projectile_card_contact = next(
        (
            index
            for index, row in enumerate(projectile_rows)
            if _int(row, "projectile_card_contacts") > 0
        ),
        None,
    )
    projectile_card_contact_observed = first_projectile_card_contact is not None
    post_contact_rows = (
        projectile_rows[first_projectile_card_contact:]
        if first_projectile_card_contact is not None
        else []
    )
    post_contact_reconstructed_response = any(
        _float(row, "max_card_center_displacement_from_initial")
        >= CARD_HOUSE_PROJECTILE_MIN_HORIZONTAL_RESPONSE
        or _float(row, "min_card_orientation_alignment_from_initial")
        <= 1.0 - CARD_HOUSE_PROJECTILE_MIN_AXIS_RESPONSE
        for row in post_contact_rows
    )

    passed = (
        card_count_valid
        and finite_state_valid
        and tracked_body_valid
        and settle_phase_valid
        and projectile_phase_valid
        and finite_metrics_valid
        and settle_stable
        and reconstructed_settle_stable
        and projectile_motion
        and impact_response
        and projectile_card_contact_observed
        and post_contact_reconstructed_response
    )
    detail = (
        f"cards={_int(final, 'card_count')},"
        f"projectiles={_int(final, 'projectile_count')},"
        f"card_count_valid={str(card_count_valid).lower()},"
        f"finite_state_valid={str(finite_state_valid).lower()},"
        f"tracked_body_valid={str(tracked_body_valid).lower()},"
        f"settle_phase_valid={str(settle_phase_valid).lower()},"
        f"projectile_phase_valid={str(projectile_phase_valid).lower()},"
        f"finite_metrics_valid={str(finite_metrics_valid).lower()},"
        f"settle_stable={str(settle_stable).lower()},"
        "reconstruction_specific_card_gate=true,"
        f"reconstructed_settle_stable="
        f"{str(reconstructed_settle_stable).lower()},"
        f"settle_axis={settle_axis:.9g},final_axis={final_axis:.9g},"
        f"settle_height={settle_height:.9g},final_height={final_height:.9g},"
        f"settle_horizontal={settle_horizontal:.9g},"
        f"final_horizontal={final_horizontal:.9g},"
        f"max_projectile_speed={max_projectile_speed:.9g},"
        f"projectile_motion={str(projectile_motion).lower()},"
        f"impact_response={str(impact_response).lower()},"
        f"projectile_card_contact_observed="
        f"{str(projectile_card_contact_observed).lower()},"
        f"post_contact_reconstructed_response="
        f"{str(post_contact_reconstructed_response).lower()}"
    )
    return passed, detail


def _literal_wedge_arch_outcome(
    rows: Sequence[dict[str, str]],
) -> tuple[bool, str]:
    expected_body = PHYSICAL_OUTCOME_TRACKED_BODIES[LITERAL_WEDGE_ARCH_SCENARIO]
    tracked_body_valid = all(
        row.get("tracked_body", "") == expected_body for row in rows
    )
    scene_contract_valid = all(
        row.get("scene_contract", "") == LITERAL_WEDGE_ARCH_SCENE_CONTRACT
        for row in rows
    )
    native_frontend_valid = all(
        row.get("collision_frontend", "") == "native" for row in rows
    )
    split_impulse_valid = all(
        row.get("split_impulse_enabled", "") == "1" for row in rows
    )
    finite_state_valid = all(_int(row, "finite_state") == 1 for row in rows)

    positions = [(_float(row, "x"), _float(row, "y"), _float(row, "z")) for row in rows]
    finite_positions = all(
        all(math.isfinite(value) for value in position) for position in positions
    )
    if positions and finite_positions:
        initial = positions[0]
        crown_displacements = [
            math.sqrt(
                sum((value - origin) ** 2 for value, origin in zip(position, initial))
            )
            for position in positions
        ]
        max_crown_displacement = max(crown_displacements)
    else:
        max_crown_displacement = math.nan
    crown_stable = (
        math.isfinite(max_crown_displacement)
        and max_crown_displacement <= LITERAL_WEDGE_ARCH_MAX_CROWN_DISPLACEMENT
    )

    up_values = [_float(row, "up_z") for row in rows]
    min_crown_up_z = min(up_values, default=math.nan)
    crown_upright = all(math.isfinite(value) for value in up_values) and (
        min_crown_up_z >= LITERAL_WEDGE_ARCH_MIN_CROWN_UP_Z
    )
    arch_displacements = [
        (
            _float(row, "max_arch_body_displacement_from_initial")
            if "max_arch_body_displacement_from_initial" in row
            else math.nan
        )
        for row in rows
    ]
    arch_orientation_alignments = [
        (
            _float(row, "min_arch_body_orientation_alignment_from_initial")
            if "min_arch_body_orientation_alignment_from_initial" in row
            else math.nan
        )
        for row in rows
    ]
    finite_arch_metrics = all(
        math.isfinite(value)
        for value in (*arch_displacements, *arch_orientation_alignments)
    )
    max_arch_displacement = max(arch_displacements, default=math.nan)
    min_arch_orientation_alignment = min(arch_orientation_alignments, default=math.nan)
    whole_arch_stable_from_initial = (
        finite_arch_metrics
        and all(
            0.0 <= value <= LITERAL_WEDGE_ARCH_MAX_BODY_DISPLACEMENT
            for value in arch_displacements
        )
        and all(
            LITERAL_WEDGE_ARCH_MIN_BODY_ORIENTATION_ALIGNMENT <= value <= 1.0
            for value in arch_orientation_alignments
        )
    )
    contact_manifold_valid = all(
        _int(row, "contacts") == LITERAL_WEDGE_ARCH_CONTACT_COUNT for row in rows
    )
    colliding_body_pairs_valid = all(
        _int(row, "unique_colliding_body_pairs")
        == LITERAL_WEDGE_ARCH_COLLIDING_BODY_PAIR_COUNT
        for row in rows
    )

    passed = (
        tracked_body_valid
        and scene_contract_valid
        and native_frontend_valid
        and split_impulse_valid
        and finite_state_valid
        and finite_positions
        and crown_stable
        and crown_upright
        and whole_arch_stable_from_initial
        and contact_manifold_valid
        and colliding_body_pairs_valid
    )
    detail = (
        "reconstructed_literal_wedge_nonpaper=true,"
        f"tracked_body_valid={str(tracked_body_valid).lower()},"
        f"scene_contract_valid={str(scene_contract_valid).lower()},"
        f"native_frontend_valid={str(native_frontend_valid).lower()},"
        f"split_impulse_valid={str(split_impulse_valid).lower()},"
        f"finite_state_valid={str(finite_state_valid).lower()},"
        f"finite_positions={str(finite_positions).lower()},"
        f"max_crown_displacement={max_crown_displacement:.9g},"
        f"max_crown_displacement_limit="
        f"{LITERAL_WEDGE_ARCH_MAX_CROWN_DISPLACEMENT:.9g},"
        f"min_crown_up_z={min_crown_up_z:.9g},"
        f"min_crown_up_z_limit={LITERAL_WEDGE_ARCH_MIN_CROWN_UP_Z:.9g},"
        f"finite_arch_initial_pose_metrics={str(finite_arch_metrics).lower()},"
        f"max_arch_body_displacement_from_initial={max_arch_displacement:.9g},"
        f"max_arch_body_displacement_limit="
        f"{LITERAL_WEDGE_ARCH_MAX_BODY_DISPLACEMENT:.9g},"
        f"min_arch_body_orientation_alignment_from_initial="
        f"{min_arch_orientation_alignment:.9g},"
        f"min_arch_body_orientation_alignment_limit="
        f"{LITERAL_WEDGE_ARCH_MIN_BODY_ORIENTATION_ALIGNMENT:.9g},"
        f"whole_arch_stable_from_initial="
        f"{str(whole_arch_stable_from_initial).lower()},"
        f"contacts_expected={LITERAL_WEDGE_ARCH_CONTACT_COUNT},"
        f"contact_manifold_valid={str(contact_manifold_valid).lower()},"
        f"colliding_body_pairs_expected="
        f"{LITERAL_WEDGE_ARCH_COLLIDING_BODY_PAIR_COUNT},"
        f"colliding_body_pairs_valid={str(colliding_body_pairs_valid).lower()}"
    )
    return passed, detail


def _evaluate_physical_outcome(
    rows: Sequence[dict[str, str]],
    scenario: str,
    expected_repetitions: int,
) -> tuple[bool | None, str, str]:
    if scenario not in PHYSICAL_OUTCOME_TRACKED_BODIES:
        return (
            None,
            "physical_outcome_contract_unavailable",
            "no fixture-derived trajectory outcome contract",
        )

    expected_steps = SCENARIO_TRAJECTORY_CONTRACT[scenario][0]
    reasons: list[str] = []
    details: list[str] = []
    for repetition in range(1, expected_repetitions + 1):
        repetition_rows = sorted(
            (row for row in rows if _int(row, "repetition") == repetition),
            key=lambda row: _int(row, "step"),
        )
        actual_steps = [_int(row, "step") for row in repetition_rows]
        if actual_steps != list(range(1, expected_steps + 1)):
            reasons.append(f"physical_outcome_incomplete_repetition:{repetition}")
            details.append(
                f"rep{repetition}:steps={len(actual_steps)},expected={expected_steps}"
            )
            continue
        if scenario == "card_house_26_settle_projectile_full":
            passed, detail = _card_house_full_outcome(repetition_rows)
        elif scenario == LITERAL_WEDGE_ARCH_SCENARIO:
            passed, detail = _literal_wedge_arch_outcome(repetition_rows)
        else:
            passed, detail = _physical_outcome_for_final_row(
                scenario, repetition_rows[-1]
            )
        if scenario.startswith("incline_mu_"):
            contact_counts = [_int(row, "contacts") for row in repetition_rows]
            penetration_depths = [
                _float(row, "penetration_depth_max") for row in repetition_rows
            ]
            sustained_contact = all(count > 0 for count in contact_counts)
            bounded_penetration = all(
                math.isfinite(depth) and depth <= INCLINE_MAX_PENETRATION
                for depth in penetration_depths
            )
            maximum_penetration = (
                max(penetration_depths) if penetration_depths else math.nan
            )
            passed = passed and sustained_contact and bounded_penetration
            detail += (
                f",sustained_contact={str(sustained_contact).lower()},"
                f"max_penetration={maximum_penetration:.9g},"
                f"max_penetration_limit={INCLINE_MAX_PENETRATION:.9g}"
            )
        details.append(f"rep{repetition}:passed={str(passed).lower()},{detail}")
        if not passed:
            reasons.append(f"physical_outcome_mismatch:rep{repetition}")

    if expected_repetitions <= 0:
        reasons.append("physical_outcome_no_measured_repetitions")
    return not reasons, ";".join(reasons) or "valid", "|".join(details)


def _summarize_group(
    rows: Sequence[dict[str, str]],
    failed_processes: int,
    *,
    records: Sequence[dict[str, object]] = (),
    expected_repetitions: int | None = None,
    configured_warmup_repetitions: int = 0,
) -> dict[str, object]:
    _validate_integer_fields(rows, include_run_fields=True)
    first = rows[0]
    scenario = first["scenario"]
    requested_threads = _int(first, "requested_threads")
    all_wall = [_float(row, "wall_ms") for row in rows]
    wall = [value for value in all_wall if math.isfinite(value) and value > 0.0]
    repetitions = sorted({_int(row, "repetition") for row in rows})
    steps_by_repetition = [
        sum(_int(row, "repetition") == repetition for row in rows)
        for repetition in repetitions
    ]
    repetition_totals = [
        sum(
            _float(row, "wall_ms")
            for row in rows
            if _int(row, "repetition") == repetition
        )
        for repetition in repetitions
    ]
    mean_ms = statistics.fmean(wall) if wall else math.nan
    reference = PAPER_REFERENCE_MS.get(scenario, math.nan)
    reference_contacts = PAPER_REFERENCE_CONTACTS.get(scenario)
    exact_diagnostic_rows = [
        row
        for row in rows
        if row["exact_diagnostics_contract"].startswith("last_exact_group_")
        and "multi_group_noncomparable" not in row["exact_diagnostics_contract"]
    ]
    finite_residuals = _finite_values(exact_diagnostic_rows, "residual")
    penetration_maxima = _finite_values(rows, "penetration_depth_max")
    penetration_p95 = _finite_values(rows, "penetration_depth_p95")
    residual_primals = _finite_values(
        exact_diagnostic_rows, "residual_primal_feasibility"
    )
    residual_duals = _finite_values(exact_diagnostic_rows, "residual_dual_feasibility")
    residual_complementarities = _finite_values(
        exact_diagnostic_rows, "residual_complementarity"
    )
    accepted_gammas = _finite_values(exact_diagnostic_rows, "accepted_gamma")
    safe_gammas = _finite_values(exact_diagnostic_rows, "safe_gamma")
    coupling_variations = _finite_values(
        exact_diagnostic_rows, "coupling_variation_ratio"
    )
    warm_start_matched_fractions = _finite_values(
        exact_diagnostic_rows, "warm_start_matched_fraction"
    )
    precision = _unique(rows, "precision_contract")
    scene = _unique(rows, "scene_contract")
    baumgarte = _unique(rows, "baumgarte_contract")
    collision_frontend = _unique(rows, "collision_frontend")
    inner_local_solver = _unique(rows, "inner_local_solver")
    inner_sweeps_requested = _unique(rows, "inner_sweeps_requested")
    fixed_inner_sweeps_requested = _unique(rows, "fixed_inner_sweeps_requested")
    step_size_persistence_enabled = _unique(rows, "step_size_persistence_enabled")
    step_size_recovery_growth_factor = _unique(rows, "step_size_recovery_growth_factor")
    step_size_persistence_requests = _finite_values(
        exact_diagnostic_rows, "step_size_persistence_request"
    )
    step_size_persistence_used_steps = sum(
        _int(row, "step_size_persistence_used") == 1 for row in exact_diagnostic_rows
    )
    row_operator_request = _unique(rows, "row_operator_request")
    row_operator_modes = _unique(rows, "row_operator_mode")
    max_outer_iterations = _unique(rows, "max_outer_iterations")
    tolerance = _unique(rows, "tolerance")
    accept_outer_max_iterations = _unique(rows, "accept_outer_max_iterations")
    inner_local_iterations = _unique(rows, "inner_local_iterations")
    adaptive_step_size_enabled = _unique(rows, "adaptive_step_size_enabled")
    warm_start_enabled = _unique(rows, "warm_start_enabled")
    projected_gradient_retry_enabled = _unique(rows, "projected_gradient_retry_enabled")
    dense_residual_polish_enabled = _unique(rows, "dense_residual_polish_enabled")
    fallback_to_boxed_lcp_enabled = _unique(rows, "fallback_to_boxed_lcp_enabled")
    diagonal_seed_enabled = _unique(rows, "diagonal_seed_enabled")
    matrix_free_seed_enabled = _unique(rows, "matrix_free_seed_enabled")
    step_size_scale = _unique(rows, "step_size_scale")
    outer_relaxation = _unique(rows, "outer_relaxation")
    initial_gamma_contract = _unique(rows, "initial_gamma_contract")
    split_impulse_enabled = _unique(rows, "split_impulse_enabled")
    actual_threads = sorted({_int(row, "actual_threads") for row in rows})
    contact_row_products = sum(
        _int(row, "step_contact_row_delassus_products") for row in rows
    )
    parallel_contact_row_products = sum(
        _int(row, "step_parallel_contact_row_delassus_products") for row in rows
    )
    parallel_contact_row_product_fraction = (
        parallel_contact_row_products / contact_row_products
        if contact_row_products > 0
        else math.nan
    )
    parallel_contact_row_steps = sum(
        _int(row, "step_parallel_contact_row_delassus_products") > 0 for row in rows
    )
    max_contact_row_participants = max(
        _int(row, "max_contact_row_participants_to_date") for row in rows
    )
    parallel_counters_consistent = (
        parallel_contact_row_products <= contact_row_products
        and (parallel_contact_row_products == 0 or max_contact_row_participants > 1)
        and max_contact_row_participants <= requested_threads
    )
    parallel_dispatch_reasons: list[str] = []
    if first["solver"] != "exact_fbf":
        parallel_dispatch_reasons.append("solver_not_exact_fbf")
    if requested_threads <= 1:
        parallel_dispatch_reasons.append("not_applicable_single_thread")
    if contact_row_products <= 0:
        parallel_dispatch_reasons.append("exact_contact_row_work_not_observed")
    if parallel_contact_row_products <= 0:
        parallel_dispatch_reasons.append("parallel_contact_row_work_not_observed")
    if max_contact_row_participants <= 1:
        parallel_dispatch_reasons.append("multiple_pool_participants_not_observed")
    if not parallel_counters_consistent:
        parallel_dispatch_reasons.append("exact_parallel_counters_inconsistent")
    parallel_dispatch_valid = not parallel_dispatch_reasons
    observed_logical_cpus = sorted(
        {
            cpu
            for row in rows
            for cpu in _strict_logical_cpu_ids(
                row, "exact_contact_row_logical_cpus_to_date"
            )
        }
    )
    observed_max_phase_logical_cpus = sorted(
        {
            cpu
            for row in rows
            for cpu in _strict_logical_cpu_ids(
                row, "max_phase_contact_row_logical_cpus_to_date"
            )
        }
    )
    colored_contract = first["solver_contract"] in {
        "dart_best_nonpaper_colored_inner_bgs",
        "dart_best_nonpaper_colored_inner_bgs_threaded_world",
    }
    inner_bgs_schedule_contract = (
        _unique(rows, "inner_bgs_schedule_contract")
        if colored_contract
        else "dart_legacy_serial_bgs"
    )
    colored_comparable_rows = (
        [
            row
            for row in rows
            if _int(row, "step_exact_solves") + _int(row, "step_exact_failures") == 1
        ]
        if colored_contract
        else []
    )
    colored_work_rows = [
        row for row in colored_comparable_rows if _int(row, "step_fbf_iterations") > 0
    ]
    colored_used_rows = [
        row
        for row in colored_work_rows
        if _int(row, "last_exact_colored_bgs_used") == 1
    ]
    colored_bgs_used_steps = len(colored_used_rows)
    colored_bgs_solves = sum(
        _int(row, "last_exact_colored_bgs_solves") for row in colored_used_rows
    )
    colored_bgs_dispatches = sum(
        _int(row, "last_exact_colored_bgs_dispatches")
        for row in colored_comparable_rows
    )
    max_colored_bgs_participants = max(
        (
            _int(row, "last_exact_colored_bgs_max_participants")
            for row in colored_comparable_rows
        ),
        default=0,
    )
    colored_schedule_rows = [
        row
        for row in colored_comparable_rows
        if any(
            _int(row, field) > 0
            for field in (
                "last_exact_colored_bgs_manifolds",
                "last_exact_colored_bgs_colors",
                "last_exact_colored_bgs_max_manifolds_per_color",
            )
        )
    ]
    colored_schedule_row_ids = {id(row) for row in colored_schedule_rows}
    literal_colored_schedule_observable = (
        scenario != LITERAL_WEDGE_ARCH_SCENARIO
        or all(
            _int(row, "last_exact_colored_bgs_manifolds")
            == LITERAL_WEDGE_ARCH_COLLIDING_BODY_PAIR_COUNT
            for row in colored_comparable_rows
        )
    )
    colored_bgs_manifolds = (
        _unique(colored_schedule_rows, "last_exact_colored_bgs_manifolds")
        if colored_schedule_rows
        else "unavailable"
    )
    colored_bgs_colors = (
        _unique(colored_schedule_rows, "last_exact_colored_bgs_colors")
        if colored_schedule_rows
        else "unavailable"
    )
    colored_bgs_max_manifolds_per_color = max(
        (
            _int(row, "last_exact_colored_bgs_max_manifolds_per_color")
            for row in colored_schedule_rows
        ),
        default=0,
    )
    colored_schedule_dimensions_valid = all(
        (
            _int(row, "last_exact_colored_bgs_manifolds") > 0
            and _int(row, "last_exact_colored_bgs_colors") > 0
            and _int(row, "last_exact_colored_bgs_max_manifolds_per_color") > 0
            and _int(row, "last_exact_colored_bgs_colors")
            <= _int(row, "last_exact_colored_bgs_manifolds")
            and _int(row, "last_exact_colored_bgs_max_manifolds_per_color")
            <= _int(row, "last_exact_colored_bgs_manifolds")
        )
        for row in colored_schedule_rows
    )
    colored_work_schedule_valid = (
        all(id(row) in colored_schedule_row_ids for row in colored_work_rows)
        and colored_schedule_dimensions_valid
    )
    colored_work_schedule_width_valid = all(
        _int(row, "last_exact_colored_bgs_max_manifolds_per_color") >= requested_threads
        for row in colored_work_rows
    )
    colored_dispatch_rows_valid = all(
        (
            _int(row, "last_exact_colored_bgs_dispatches") == 0
            if requested_threads == 1 or id(row) not in colored_schedule_row_ids
            else _int(row, "last_exact_colored_bgs_dispatches") == 1
        )
        for row in colored_comparable_rows
    )
    colored_participant_rows_valid = all(
        (
            _int(row, "last_exact_colored_bgs_max_participants") == 0
            if id(row) not in colored_schedule_row_ids
            else (
                _int(row, "last_exact_colored_bgs_max_participants")
                == requested_threads
                if _int(row, "last_exact_colored_bgs_used") == 1
                or requested_threads > 1
                else _int(row, "last_exact_colored_bgs_max_participants") == 0
            )
        )
        for row in colored_comparable_rows
    )
    colored_legacy_contact_row_isolation_valid = (
        parallel_contact_row_products == 0
        and max_contact_row_participants <= 1
        and not observed_logical_cpus
        and not observed_max_phase_logical_cpus
    )
    colored_dispatch_reasons: list[str] = []
    if not colored_contract:
        colored_dispatch_reasons.append("colored_contract_not_requested")
    if requested_threads <= 1:
        colored_dispatch_reasons.append("not_applicable_single_thread")
    if not colored_work_rows:
        colored_dispatch_reasons.append("colored_positive_iteration_work_not_observed")
    if len(colored_used_rows) != len(colored_work_rows):
        colored_dispatch_reasons.append(
            "colored_path_not_used_on_every_positive_iteration_attempt"
        )
    if not colored_schedule_dimensions_valid or not colored_work_schedule_valid:
        colored_dispatch_reasons.append("colored_schedule_dimensions_inconsistent")
    if not literal_colored_schedule_observable:
        colored_dispatch_reasons.append("literal_contact_schedule_unobservable")
    if not colored_dispatch_rows_valid:
        colored_dispatch_reasons.append("colored_persistent_dispatch_count_mismatch")
    if not colored_participant_rows_valid:
        colored_dispatch_reasons.append("colored_participant_count_mismatch")
    if not colored_work_schedule_width_valid:
        colored_dispatch_reasons.append("colored_schedule_width_below_thread_count")
    if parallel_contact_row_products != 0:
        colored_dispatch_reasons.append("legacy_parallel_contact_row_work_observed")
    if max_contact_row_participants > 1:
        colored_dispatch_reasons.append("legacy_contact_row_participants_observed")
    if observed_logical_cpus or observed_max_phase_logical_cpus:
        colored_dispatch_reasons.append("legacy_contact_row_cpu_residency_observed")
    colored_bgs_dispatch_valid = not colored_dispatch_reasons
    colored_path_valid = (
        colored_contract
        and bool(colored_work_rows)
        and len(colored_used_rows) == len(colored_work_rows)
        and colored_work_schedule_valid
        and colored_work_schedule_width_valid
        and literal_colored_schedule_observable
        and colored_legacy_contact_row_isolation_valid
        and max_colored_bgs_participants == requested_threads
        and colored_bgs_max_manifolds_per_color >= 1
    )
    observed_colored_logical_cpus = sorted(
        {
            cpu
            for row in colored_comparable_rows
            for cpu in _strict_logical_cpu_ids(row, "exact_colored_bgs_logical_cpus")
        }
    )
    step_iterations = [_int(row, "step_fbf_iterations") for row in rows]
    convergence_reference = PAPER_CONVERGENCE_REFERENCE.get(
        scenario, (math.nan, math.nan, math.nan)
    )
    residual_pass_fraction = (
        sum(value <= 1e-6 for value in finite_residuals) / len(finite_residuals)
        if finite_residuals
        else math.nan
    )
    median_residual = (
        statistics.median(finite_residuals) if finite_residuals else math.nan
    )
    exact_failures = sum(_int(row, "step_exact_failures") for row in rows)
    fallbacks = sum(_int(row, "step_fallbacks") for row in rows)
    min_contacts = min(_int(row, "contacts") for row in rows)
    max_contacts = max(_int(row, "contacts") for row in rows)
    contact_steps_without_comparable_residual = sum(
        _int(row, "contacts") > 0 and row not in exact_diagnostic_rows for row in rows
    )

    def solver_step_accepted(row: dict[str, str]) -> bool:
        if row["solver"] != "exact_fbf":
            return row["status"] == "boxed_lcp"
        if row["status"] in {"success", "max_iterations_accepted"}:
            return _int(row, "step_fallbacks") == 0
        return (
            row["status"] == "no_exact_group"
            and _int(row, "step_exact_solves") == 0
            and _int(row, "step_exact_failures") == 0
            and _int(row, "step_fallbacks") == 0
        )

    def solver_step_succeeded(row: dict[str, str]) -> bool:
        return solver_step_accepted(row) and row["status"] != "max_iterations_accepted"

    all_solver_steps_accepted = failed_processes == 0 and all(
        solver_step_accepted(row) for row in rows
    )
    all_solver_steps_successful = failed_processes == 0 and all(
        solver_step_succeeded(row) for row in rows
    )
    status_counts: dict[str, int] = {}
    for row in rows:
        status_counts[row["status"]] = status_counts.get(row["status"], 0) + 1
    solver_statuses = ";".join(
        f"{status}:{count}" for status, count in sorted(status_counts.items())
    )

    measured_records = [record for record in records if not record["warmup"]]
    warmup_records = [record for record in records if record["warmup"]]
    measured_trajectories = len(measured_records) if records else len(repetitions)
    complete_measured_trajectories = (
        sum(bool(record["complete_rows"]) for record in measured_records)
        if records
        else len(repetitions)
    )
    completed_warmup_trajectories = sum(
        bool(record["complete_rows"]) and int(record["returncode"]) == 0
        for record in warmup_records
    )
    failed_warmup_processes = sum(
        int(record["returncode"]) != 0 or not bool(record["complete_rows"])
        for record in warmup_records
    )
    warmup_contract_reasons: list[str] = []
    if len(warmup_records) != configured_warmup_repetitions:
        warmup_contract_reasons.append("configured_warmup_count_mismatch")
    if completed_warmup_trajectories != configured_warmup_repetitions:
        warmup_contract_reasons.append("warmup_trajectory_failure_or_incomplete")
    warmup_contract_valid = not warmup_contract_reasons
    if expected_repetitions is None:
        expected_repetitions = measured_trajectories
    expected_steps = {
        int(record["expected_rows"])
        for record in measured_records
        if "expected_rows" in record
    }
    expected_steps_per_trajectory = (
        next(iter(expected_steps))
        if len(expected_steps) == 1
        else (steps_by_repetition[0] if len(set(steps_by_repetition)) == 1 else None)
    )
    complete_requested_trajectory_evidence = (
        measured_trajectories == expected_repetitions
        and complete_measured_trajectories == expected_repetitions
        and len(repetitions) == expected_repetitions
        and expected_steps_per_trajectory is not None
        and all(steps == expected_steps_per_trajectory for steps in steps_by_repetition)
    )
    scenario_steps, scenario_trajectory_contract = SCENARIO_TRAJECTORY_CONTRACT.get(
        scenario, (None, "single_step_probe_or_full_length_not_defined")
    )
    full_trajectory_evidence = (
        complete_requested_trajectory_evidence
        and scenario_steps is not None
        and expected_steps_per_trajectory == scenario_steps
    )
    (
        physical_outcome_valid,
        physical_outcome_reasons,
        physical_outcome_details,
    ) = _evaluate_physical_outcome(rows, scenario, expected_repetitions)

    affinity_source = _unique(rows, "affinity_source")
    affinity_logical_cpus = _unique(rows, "affinity_logical_cpus")
    raw_logical_counts = {
        row.get("affinity_logical_cpu_count", "unknown") for row in rows
    }
    affinity_logical_counts = {
        int(value) for value in raw_logical_counts if value.isdigit()
    }
    raw_physical_counts = {
        row.get("affinity_physical_core_count", "unknown") for row in rows
    }
    affinity_physical_counts = {
        int(value) for value in raw_physical_counts if value.isdigit()
    }
    affinity_logical_cpu_count = (
        next(iter(affinity_logical_counts))
        if len(affinity_logical_counts) == 1 and len(raw_logical_counts) == 1
        else None
    )
    affinity_physical_core_count = (
        next(iter(affinity_physical_counts))
        if len(affinity_physical_counts) == 1 and len(raw_physical_counts) == 1
        else None
    )
    one_logical_values = {
        row.get("affinity_one_logical_per_physical_core", "unknown") for row in rows
    }
    affinity_one_logical_per_physical_core = (
        next(iter(one_logical_values)) == "true"
        if len(one_logical_values) == 1
        and next(iter(one_logical_values)) in {"true", "false"}
        else None
    )
    affinity_physical_core_keys = _unique(rows, "affinity_physical_core_keys")
    affinity_logical_cpu_physical_core_keys = _unique(
        rows, "affinity_logical_cpu_physical_core_keys"
    )
    affinity_package_ids = _unique(rows, "affinity_package_ids")
    raw_package_counts = {row.get("affinity_package_count", "unknown") for row in rows}
    affinity_package_counts = {
        int(value) for value in raw_package_counts if value.isdigit()
    }
    affinity_package_count = (
        next(iter(affinity_package_counts))
        if len(affinity_package_counts) == 1 and len(raw_package_counts) == 1
        else None
    )
    affinity_smt_sibling_counts = _unique(rows, "affinity_smt_sibling_counts")
    affinity_core_classes_khz = _unique(rows, "affinity_core_classes_khz")
    affinity_scaling_governors = _unique(rows, "affinity_scaling_governors")
    paper_hardware_contract = _unique(rows, "paper_hardware_contract")
    explicitly_controlled_affinity = affinity_source == "explicit_taskset"
    controlled_affinity_reasons: list[str] = []
    if not explicitly_controlled_affinity:
        controlled_affinity_reasons.append("affinity_not_explicit_taskset")
    if affinity_logical_cpu_count != requested_threads:
        controlled_affinity_reasons.append("affinity_logical_cpu_count_mismatch")
    if affinity_physical_core_count != requested_threads:
        controlled_affinity_reasons.append("affinity_physical_core_count_mismatch")
    if affinity_one_logical_per_physical_core is not True:
        controlled_affinity_reasons.append("one_logical_per_core_unproven")
    raw_physical_core_keys = _consistent_row_value(rows, "affinity_physical_core_keys")
    encoded_physical_mapping = _consistent_row_value(
        rows, "affinity_logical_cpu_physical_core_keys"
    )
    parsed_physical_mapping = _parse_encoded_physical_core_mapping(
        encoded_physical_mapping
    )
    if (
        raw_physical_core_keys in {None, "unknown"}
        or len(raw_physical_core_keys.split(";")) != requested_threads
    ):
        controlled_affinity_reasons.append("physical_core_keys_unproven")
    if (
        parsed_physical_mapping is None
        or len(parsed_physical_mapping) != requested_threads
        or len(set(parsed_physical_mapping.values())) != requested_threads
        or raw_physical_core_keys in {None, "unknown"}
        or set(parsed_physical_mapping.values())
        != set(raw_physical_core_keys.split(";"))
    ):
        controlled_affinity_reasons.append("logical_cpu_core_mapping_unproven")
    if requested_threads > 1:
        raw_package_ids = _consistent_row_value(rows, "affinity_package_ids")
        if affinity_package_count != 1 or (
            raw_package_ids in {None, "unknown"} or ";" in raw_package_ids
        ):
            controlled_affinity_reasons.append("single_package_unproven")
        raw_core_classes = _consistent_row_value(rows, "affinity_core_classes_khz")
        if raw_core_classes in {None, "unknown"} or ";" in raw_core_classes:
            controlled_affinity_reasons.append("single_core_class_unproven")
        raw_governors = _consistent_row_value(rows, "affinity_scaling_governors")
        if raw_governors in {None, "unknown"} or ";" in raw_governors:
            controlled_affinity_reasons.append("single_scaling_governor_unproven")
    controlled_affinity_valid = not controlled_affinity_reasons

    measured_affinity = _consistent_measured_affinity(records)
    physical_core_mapping = _physical_core_mapping(measured_affinity)
    allowed_observed_cpus: set[int] | None = None
    if measured_affinity is not None and isinstance(
        measured_affinity.get("logical_cpus"), list
    ):
        raw_allowed_cpus = measured_affinity["logical_cpus"]
        if all(isinstance(cpu, int) and cpu >= 0 for cpu in raw_allowed_cpus):
            allowed_observed_cpus = set(raw_allowed_cpus)
    if allowed_observed_cpus is None:
        raw_affinity_cpus = _consistent_row_value(rows, "affinity_logical_cpus")
        if raw_affinity_cpus is not None and re.fullmatch(
            r"[0-9]+(?:,[0-9]+)*", raw_affinity_cpus
        ):
            allowed_observed_cpus = {
                int(value) for value in raw_affinity_cpus.split(",")
            }

    if not observed_logical_cpus:
        observed_physical_core_count: int | None = 0
    elif physical_core_mapping is not None and all(
        cpu in physical_core_mapping for cpu in observed_logical_cpus
    ):
        observed_physical_core_count = len(
            {physical_core_mapping[cpu] for cpu in observed_logical_cpus}
        )
    else:
        observed_physical_core_count = None

    phase_sets_by_repetition: dict[int, tuple[int, ...]] = {}
    for repetition in repetitions:
        repetition_rows = sorted(
            (row for row in rows if _int(row, "repetition") == repetition),
            key=lambda row: _int(row, "step"),
        )
        if not repetition_rows:
            continue
        if colored_contract:
            candidates = [
                _strict_logical_cpu_ids(row, "max_phase_exact_colored_bgs_logical_cpus")
                for row in repetition_rows
                if _int(row, "last_exact_colored_bgs_used") == 1
            ]
            phase_sets_by_repetition[repetition] = (
                min(candidates, key=lambda cpus: (-len(cpus), cpus))
                if candidates
                else ()
            )
        else:
            phase_sets_by_repetition[repetition] = _strict_logical_cpu_ids(
                repetition_rows[-1], "max_phase_contact_row_logical_cpus_to_date"
            )
    phase_residency_logical_cpus_by_repetition = "|".join(
        f"rep{repetition}=" + (";".join(str(cpu) for cpu in cpus) if cpus else "none")
        for repetition, cpus in sorted(phase_sets_by_repetition.items())
    )
    phase_residency_min_logical_cpu_count = (
        min(len(cpus) for cpus in phase_sets_by_repetition.values())
        if phase_sets_by_repetition
        else 0
    )
    phase_physical_counts: dict[int, int] = {}
    if physical_core_mapping is not None:
        for repetition, cpus in phase_sets_by_repetition.items():
            if all(cpu in physical_core_mapping for cpu in cpus):
                phase_physical_counts[repetition] = len(
                    {physical_core_mapping[cpu] for cpu in cpus}
                )
    phase_residency_min_physical_core_count = (
        min(phase_physical_counts.values())
        if len(phase_physical_counts) == len(phase_sets_by_repetition)
        and phase_physical_counts
        else (0 if not phase_sets_by_repetition else None)
    )

    runtime_cpu_residency_details: list[str] = []
    if requested_threads <= 1:
        runtime_cpu_residency_details.append("not_applicable_single_thread")
    else:
        selected_dispatch_valid = (
            colored_bgs_dispatch_valid if colored_contract else parallel_dispatch_valid
        )
        if not selected_dispatch_valid:
            runtime_cpu_residency_details.append("parallel_dispatch_not_valid")
        if not complete_requested_trajectory_evidence:
            runtime_cpu_residency_details.append("incomplete_measured_trajectory")
        if physical_core_mapping is None:
            runtime_cpu_residency_details.append("logical_cpu_topology_unavailable")
        if allowed_observed_cpus is None:
            runtime_cpu_residency_details.append("invocation_affinity_unavailable")
        expected_repetition_ids = set(range(1, expected_repetitions + 1))
        if set(phase_sets_by_repetition) != expected_repetition_ids:
            runtime_cpu_residency_details.append("phase_residency_repetition_missing")
        for repetition in sorted(expected_repetition_ids):
            phase_cpus = phase_sets_by_repetition.get(repetition, ())
            if len(phase_cpus) != requested_threads:
                runtime_cpu_residency_details.append(
                    f"phase_logical_cpu_count_mismatch:rep{repetition}"
                )
            if allowed_observed_cpus is not None and not set(phase_cpus).issubset(
                allowed_observed_cpus
            ):
                runtime_cpu_residency_details.append(
                    f"phase_cpu_outside_affinity:rep{repetition}"
                )
            if phase_physical_counts.get(repetition) != requested_threads:
                runtime_cpu_residency_details.append(
                    f"phase_physical_core_count_mismatch:rep{repetition}"
                )
    runtime_cpu_residency_valid = (
        requested_threads > 1 and not runtime_cpu_residency_details
    )
    if runtime_cpu_residency_valid:
        runtime_cpu_residency_reasons = "valid"
    elif requested_threads <= 1:
        runtime_cpu_residency_reasons = "not_applicable_single_thread"
    else:
        runtime_cpu_residency_reasons = ";".join(
            [
                "per_parallel_phase_core_residency_unobserved",
                *runtime_cpu_residency_details,
            ]
        )

    realtime_reasons: list[str] = []
    if first["solver"] != "exact_fbf":
        realtime_reasons.append("solver_not_exact_fbf")
    if failed_processes:
        realtime_reasons.append("measured_process_failure")
    if not warmup_contract_valid:
        realtime_reasons.extend(warmup_contract_reasons)
    if not complete_requested_trajectory_evidence:
        realtime_reasons.append("incomplete_measured_trajectory")
    elif not full_trajectory_evidence:
        realtime_reasons.append("not_full_scenario_trajectory")
    if len(wall) != len(rows):
        realtime_reasons.append("nonfinite_or_negative_wall_time")
    if not all(_int(row, "finite_state") == 1 for row in rows):
        realtime_reasons.append("nonfinite_simulation_state")
    if exact_failures:
        realtime_reasons.append("exact_solver_failure")
    if fallbacks:
        realtime_reasons.append("fallback_present")
    if not all_solver_steps_accepted:
        realtime_reasons.append("unaccepted_solver_status")
    if contact_steps_without_comparable_residual:
        realtime_reasons.append("contact_step_residual_not_comparable")
    if len(finite_residuals) != len(exact_diagnostic_rows):
        realtime_reasons.append("nonfinite_residual")
    if finite_residuals and max(finite_residuals) > 1e-6:
        realtime_reasons.append("residual_above_1e-6")
    if max_contacts > 0 and not finite_residuals:
        realtime_reasons.append("no_comparable_contact_residual")
    if actual_threads != [requested_threads]:
        realtime_reasons.append("actual_thread_count_mismatch")
    if colored_contract:
        if not colored_path_valid:
            realtime_reasons.append("colored_inner_bgs_path_not_consistently_observed")
        if requested_threads > 1 and not colored_bgs_dispatch_valid:
            realtime_reasons.append("colored_inner_bgs_dispatch_invalid")
    elif not parallel_counters_consistent:
        realtime_reasons.append("exact_parallel_evidence_inconsistent")
    if physical_outcome_valid is not True:
        realtime_reasons.append(physical_outcome_reasons)
    if not controlled_affinity_valid:
        realtime_reasons.append(
            "controlled_affinity_contract_invalid:"
            + ",".join(controlled_affinity_reasons)
        )
    realtime_contract_valid = not realtime_reasons

    solver_contract = first["solver_contract"]
    if colored_contract and requested_threads == 1 and colored_path_valid:
        parallelism_contract = "exact_fbf_nonpaper_colored_inner_bgs_single_thread"
    elif colored_contract and colored_bgs_dispatch_valid:
        parallelism_contract = "exact_fbf_nonpaper_colored_inner_bgs_observed"
    elif colored_contract:
        parallelism_contract = "exact_fbf_nonpaper_colored_inner_bgs_unproven"
    elif requested_threads == 1:
        parallelism_contract = "single_simulation_thread"
    elif first["solver"] != "exact_fbf":
        parallelism_contract = "threaded_world_non_exact_solver"
    elif parallel_contact_row_products > 0 and max_contact_row_participants > 1:
        parallelism_contract = "exact_fbf_contact_row_parallel_observed"
    elif contact_row_products > 0:
        parallelism_contract = "exact_fbf_contact_row_serial_observed"
    else:
        parallelism_contract = "exact_fbf_contact_row_work_not_observed"
    single_core_claim_valid = (
        realtime_contract_valid
        and requested_threads == 1
        and actual_threads == [1]
        and affinity_physical_core_count == 1
        and parallel_contact_row_products == 0
        and max_contact_row_participants <= 1
        and (not colored_contract or colored_path_valid)
    )
    if colored_contract:
        multicore_claim_valid = (
            realtime_contract_valid
            and requested_threads > 1
            and actual_threads == [requested_threads]
            and first["solver"] == "exact_fbf"
            and collision_frontend == "native"
            and colored_bgs_dispatch_valid
            and runtime_cpu_residency_valid
            and controlled_affinity_valid
        )
    else:
        multicore_claim_valid = (
            realtime_contract_valid
            and requested_threads > 1
            and actual_threads == [requested_threads]
            and first["solver"] == "exact_fbf"
            and collision_frontend == "native"
            and contact_row_products > 0
            and parallel_dispatch_valid
            and runtime_cpu_residency_valid
            and controlled_affinity_valid
        )

    paper_steps, paper_trajectory_contract = PAPER_TRAJECTORY_CONTRACT.get(
        scenario, (None, "timing_run_length_not_published")
    )
    paper_trajectory_exact = (
        paper_steps is not None
        and paper_trajectory_contract.startswith("published_")
        and expected_steps_per_trajectory == paper_steps
        and complete_requested_trajectory_evidence
    )
    contacts_match: bool | None
    if reference_contacts is None:
        contacts_match = None
    else:
        contacts_match = all(
            _int(row, "contacts") == reference_contacts for row in rows
        )

    workload_reasons: list[str] = []
    if not warmup_contract_valid:
        workload_reasons.extend(warmup_contract_reasons)
    if not math.isfinite(reference):
        workload_reasons.append("paper_timing_reference_unavailable")
    if first["solver"] != "exact_fbf":
        workload_reasons.append("solver_not_exact_fbf")
    if solver_contract != "paper_cpu":
        workload_reasons.append("solver_contract_not_paper_cpu")
    if colored_contract:
        workload_reasons.append("nonpaper_colored_inner_bgs_schedule")
    expected_paper_inner_sweeps = 30 if scenario.startswith("masonry_arch_") else 10
    if inner_local_solver != "exact_metric":
        workload_reasons.append(f"inner_local_solver_mismatch:{inner_local_solver}")
    if inner_sweeps_requested != str(expected_paper_inner_sweeps):
        workload_reasons.append(f"inner_sweep_budget_mismatch:{inner_sweeps_requested}")
    if fixed_inner_sweeps_requested != "1":
        workload_reasons.append("fixed_inner_sweeps_not_requested")
    if step_size_persistence_enabled != "1":
        workload_reasons.append("step_size_persistence_not_enabled")
    growth_factors = _finite_values(rows, "step_size_recovery_growth_factor")
    if len(growth_factors) != len(rows) or any(
        not math.isclose(value, 1.05) for value in growth_factors
    ):
        workload_reasons.append(
            f"step_size_recovery_growth_factor_mismatch:"
            f"{step_size_recovery_growth_factor}"
        )
    if row_operator_request != "contact_row_no_dense_snapshot":
        workload_reasons.append(f"row_operator_request_mismatch:{row_operator_request}")
    exact_attempt_row_modes = {
        row["row_operator_mode"]
        for row in rows
        if _int(row, "step_exact_solves") + _int(row, "step_exact_failures") > 0
    }
    if exact_attempt_row_modes - {"contact_row_no_dense_snapshot"}:
        workload_reasons.append(
            "row_operator_mode_mismatch:" + ";".join(sorted(exact_attempt_row_modes))
        )
    expected_paper_cpu_fields = {
        "max_outer_iterations": (max_outer_iterations, "200"),
        "accept_outer_max_iterations": (accept_outer_max_iterations, "1"),
        "inner_local_iterations": (inner_local_iterations, "1"),
        "adaptive_step_size_enabled": (adaptive_step_size_enabled, "1"),
        "warm_start_enabled": (warm_start_enabled, "1"),
        "projected_gradient_retry_enabled": (
            projected_gradient_retry_enabled,
            "0",
        ),
        "dense_residual_polish_enabled": (dense_residual_polish_enabled, "0"),
        "fallback_to_boxed_lcp_enabled": (fallback_to_boxed_lcp_enabled, "0"),
        "diagonal_seed_enabled": (diagonal_seed_enabled, "0"),
        "matrix_free_seed_enabled": (matrix_free_seed_enabled, "0"),
        "initial_gamma_contract": (
            initial_gamma_contract,
            "automatic_safe_bound",
        ),
        "split_impulse_enabled": (
            split_impulse_enabled,
            "1" if scenario in PAPER_CPU_SPLIT_IMPULSE_SCENARIOS else "0",
        ),
    }
    for name, (actual, expected) in expected_paper_cpu_fields.items():
        if actual != expected:
            workload_reasons.append(f"{name}_mismatch:{actual}")
    expected_paper_cpu_float_fields = {
        "tolerance": 1e-6,
        "step_size_scale": 1.0,
        "outer_relaxation": 1.0,
    }
    for name, expected in expected_paper_cpu_float_fields.items():
        values = _finite_values(rows, name)
        if len(values) != len(rows) or any(
            not math.isclose(value, expected, rel_tol=1e-12, abs_tol=0.0)
            for value in values
        ):
            workload_reasons.append(f"{name}_mismatch:{_unique(rows, name)}")
    # These two details are not disclosed by the paper. Even a rigorously
    # configured DART run therefore cannot claim exact algorithm/timing parity.
    workload_reasons.append("paper_inner_block_formula_unpublished")
    workload_reasons.append("paper_step_size_recovery_rule_unpublished")
    if scenario in PAPER_CPU_SPLIT_IMPULSE_SCENARIOS:
        workload_reasons.append("paper_split_impulse_contract_unpublished")
    if requested_threads != 1 or actual_threads != [1]:
        workload_reasons.append("paper_cpu_run_not_sequential_single_thread")
    if physical_outcome_valid is not True:
        workload_reasons.append(physical_outcome_reasons)
    if not controlled_affinity_valid:
        workload_reasons.append("controlled_affinity_contract_invalid")
    if configured_warmup_repetitions != 0:
        workload_reasons.append("fbf_warmup_exclusion_not_reported_by_paper")
    if not complete_requested_trajectory_evidence:
        workload_reasons.append("incomplete_measured_trajectory")
    if paper_steps is None:
        workload_reasons.append("paper_timing_run_length_not_published")
    elif not paper_trajectory_contract.startswith("published_"):
        workload_reasons.append("paper_timing_run_length_only_inferred")
    elif not paper_trajectory_exact:
        workload_reasons.append("trajectory_steps_do_not_match_paper")
    if precision != "paper_float32":
        workload_reasons.append(f"precision_mismatch:{precision}")
    exact_scene_contracts = {
        "dart": {"paper_exact", "paper_exact_dart_collision_frontend"},
        "native": {"paper_exact_native_collision_frontend"},
    }
    if scene not in exact_scene_contracts.get(collision_frontend, set()):
        workload_reasons.append(
            f"scene_or_frontend_mismatch:{collision_frontend}:{scene}"
        )
    if baumgarte != "paper_exact":
        workload_reasons.append(f"baumgarte_mismatch:{baumgarte}")
    if contacts_match is not True:
        workload_reasons.append("published_contact_count_not_matched")
    if failed_processes:
        workload_reasons.append("measured_process_failure")
    if len(wall) != len(rows):
        workload_reasons.append("nonfinite_or_nonpositive_wall_time")
    if exact_failures:
        workload_reasons.append("exact_solver_failure")
    if fallbacks:
        workload_reasons.append("fallback_present")
    if not all_solver_steps_accepted:
        workload_reasons.append("unaccepted_solver_status")
    if not all(_int(row, "finite_state") == 1 for row in rows):
        workload_reasons.append("nonfinite_simulation_state")
    if contact_steps_without_comparable_residual:
        workload_reasons.append("contact_step_residual_not_comparable")
    if len(finite_residuals) != len(exact_diagnostic_rows):
        workload_reasons.append("nonfinite_residual")
    published_pass_fraction = convergence_reference[0]
    published_median_residual = PAPER_MEDIAN_RESIDUAL_REFERENCE.get(scenario)
    if math.isfinite(published_pass_fraction):
        if (
            not math.isfinite(residual_pass_fraction)
            or residual_pass_fraction < published_pass_fraction
        ):
            workload_reasons.append("residual_pass_fraction_below_paper")
        if published_median_residual is not None and (
            not math.isfinite(median_residual)
            or median_residual > published_median_residual
        ):
            workload_reasons.append("median_residual_above_paper")
    elif finite_residuals and max(finite_residuals) > 1e-6:
        workload_reasons.append("residual_above_matched_1e-6_tolerance")
    elif max_contacts > 0 and not finite_residuals:
        workload_reasons.append("no_comparable_contact_residual")

    paper_workload_contract_valid = not workload_reasons
    hardware_comparable = paper_hardware_contract.startswith("apple_silicon_family_")
    paper_timing_comparable = paper_workload_contract_valid and hardware_comparable
    paper_target_evaluated = paper_timing_comparable and math.isfinite(mean_ms)
    paper_ratio_to_reference = mean_ms / reference if paper_target_evaluated else None
    paper_mean_target_met = mean_ms < reference if paper_target_evaluated else None
    comparison_reasons = list(workload_reasons)
    if not hardware_comparable:
        comparison_reasons.append(f"hardware_mismatch:{paper_hardware_contract}")
    paper_comparison_note = (
        "comparable_within_disclosed_contract_exact_mac_model_unpublished"
        if not comparison_reasons
        else ";".join(comparison_reasons)
    )

    measured_workload_fingerprint = _measured_workload_fingerprint(rows)
    summary = {
        "scenario": scenario,
        "solver": first["solver"],
        "solver_contract": solver_contract,
        "collision_frontend": collision_frontend,
        "requested_threads": requested_threads,
        "actual_threads": ";".join(str(value) for value in actual_threads),
        "repetitions": len(repetitions),
        "steps_per_repetition": ";".join(str(value) for value in steps_by_repetition),
        "sample_steps": len(rows),
        "failed_processes": failed_processes,
        "mean_step_ms": mean_ms,
        "median_step_ms": statistics.median(wall) if wall else math.nan,
        "p95_step_ms": _percentile(wall, 0.95),
        "max_step_ms": max(wall, default=math.nan),
        "mean_repetition_ms": (
            statistics.fmean(repetition_totals) if repetition_totals else math.nan
        ),
        "steps_per_second": 1000.0 / mean_ms if mean_ms > 0.0 else math.nan,
        "realtime_factor": (
            REALTIME_THRESHOLD_MS / mean_ms if mean_ms > 0.0 else math.nan
        ),
        "realtime_threshold_ms": REALTIME_THRESHOLD_MS,
        "raw_mean_below_realtime": bool(wall) and mean_ms < REALTIME_THRESHOLD_MS,
        "raw_all_steps_below_realtime": bool(wall)
        and max(wall) < REALTIME_THRESHOLD_MS,
        "realtime_contract_valid": realtime_contract_valid,
        "realtime_contract_reasons": ";".join(realtime_reasons) or "valid",
        "mean_realtime_target_met": (
            mean_ms < REALTIME_THRESHOLD_MS if realtime_contract_valid else None
        ),
        "all_steps_realtime_target_met": (
            max(wall) < REALTIME_THRESHOLD_MS if realtime_contract_valid else None
        ),
        "affinity_source": affinity_source,
        "affinity_logical_cpus": affinity_logical_cpus,
        "affinity_logical_cpu_count": affinity_logical_cpu_count,
        "affinity_physical_core_count": affinity_physical_core_count,
        "affinity_one_logical_per_physical_core": (
            affinity_one_logical_per_physical_core
        ),
        "affinity_physical_core_keys": affinity_physical_core_keys,
        "affinity_logical_cpu_physical_core_keys": (
            affinity_logical_cpu_physical_core_keys
        ),
        "affinity_package_ids": affinity_package_ids,
        "affinity_package_count": affinity_package_count,
        "affinity_smt_sibling_counts": affinity_smt_sibling_counts,
        "affinity_core_classes_khz": affinity_core_classes_khz,
        "affinity_scaling_governors": affinity_scaling_governors,
        "single_core_claim_valid": single_core_claim_valid,
        "multicore_claim_valid": multicore_claim_valid,
        "parallelism_contract": parallelism_contract,
        "parallel_dispatch_valid": parallel_dispatch_valid,
        "parallel_dispatch_reasons": (";".join(parallel_dispatch_reasons) or "valid"),
        "contact_row_delassus_products": contact_row_products,
        "parallel_contact_row_delassus_products": parallel_contact_row_products,
        "parallel_contact_row_product_fraction": (
            parallel_contact_row_product_fraction
        ),
        "parallel_contact_row_steps": parallel_contact_row_steps,
        "max_contact_row_participants": max_contact_row_participants,
        "inner_bgs_schedule_contract": inner_bgs_schedule_contract,
        "colored_bgs_used_steps": colored_bgs_used_steps,
        "colored_bgs_solves": colored_bgs_solves,
        "colored_bgs_dispatches": colored_bgs_dispatches,
        "max_colored_bgs_participants": max_colored_bgs_participants,
        "colored_bgs_manifolds": colored_bgs_manifolds,
        "colored_bgs_colors": colored_bgs_colors,
        "colored_bgs_max_manifolds_per_color": (colored_bgs_max_manifolds_per_color),
        "colored_bgs_dispatch_valid": colored_bgs_dispatch_valid,
        "colored_bgs_dispatch_reasons": (";".join(colored_dispatch_reasons) or "valid"),
        "observed_exact_colored_bgs_logical_cpus": (
            ";".join(str(cpu) for cpu in observed_colored_logical_cpus)
            if observed_colored_logical_cpus
            else "none"
        ),
        "observed_exact_colored_bgs_logical_cpu_count": len(
            observed_colored_logical_cpus
        ),
        "observed_exact_contact_row_logical_cpus": (
            ";".join(str(cpu) for cpu in observed_logical_cpus)
            if observed_logical_cpus
            else "none"
        ),
        "observed_exact_contact_row_logical_cpu_count": len(observed_logical_cpus),
        "observed_exact_contact_row_physical_core_count": (
            observed_physical_core_count
        ),
        "phase_residency_logical_cpus_by_repetition": (
            phase_residency_logical_cpus_by_repetition
        ),
        "phase_residency_min_logical_cpu_count": (
            phase_residency_min_logical_cpu_count
        ),
        "phase_residency_min_physical_core_count": (
            phase_residency_min_physical_core_count
        ),
        "runtime_cpu_residency_valid": runtime_cpu_residency_valid,
        "runtime_cpu_residency_reasons": runtime_cpu_residency_reasons,
        "raw_speedup_vs_one_thread": None,
        "validated_speedup_vs_one_thread": None,
        "measured_workload_fingerprint_sha256": measured_workload_fingerprint,
        "scaling_pair_valid": None,
        "scaling_pair_reasons": "not_evaluated",
        "configured_warmup_repetitions": configured_warmup_repetitions,
        "completed_warmup_trajectories": completed_warmup_trajectories,
        "failed_warmup_processes": failed_warmup_processes,
        "warmup_contract_valid": warmup_contract_valid,
        "warmup_contract_reasons": (";".join(warmup_contract_reasons) or "valid"),
        "measured_trajectories": measured_trajectories,
        "complete_measured_trajectories": complete_measured_trajectories,
        "complete_requested_trajectory_evidence": (
            complete_requested_trajectory_evidence
        ),
        "full_trajectory_evidence": full_trajectory_evidence,
        "scenario_trajectory_steps": scenario_steps,
        "scenario_trajectory_contract": scenario_trajectory_contract,
        "physical_outcome_valid": physical_outcome_valid,
        "physical_outcome_reasons": physical_outcome_reasons,
        "physical_outcome_details": physical_outcome_details,
        "controlled_affinity_valid": controlled_affinity_valid,
        "controlled_affinity_reasons": (
            ";".join(controlled_affinity_reasons) or "valid"
        ),
        "paper_trajectory_steps": paper_steps,
        "paper_trajectory_contract": paper_trajectory_contract,
        "exact_failures": exact_failures,
        "fallbacks": fallbacks,
        "solver_statuses": solver_statuses,
        "all_solver_steps_accepted": all_solver_steps_accepted,
        "max_residual": max(finite_residuals, default=math.nan),
        "median_residual": median_residual,
        "p95_residual": _percentile(finite_residuals, 0.95),
        "residual_sample_steps": len(finite_residuals),
        "contact_steps_without_comparable_residual": (
            contact_steps_without_comparable_residual
        ),
        "min_contacts": min_contacts,
        "max_contacts": max_contacts,
        "min_unique_colliding_body_pairs": min(
            _int(row, "unique_colliding_body_pairs") for row in rows
        ),
        "max_unique_colliding_body_pairs": max(
            _int(row, "unique_colliding_body_pairs") for row in rows
        ),
        "max_penetration_depth": max(penetration_maxima, default=math.nan),
        "max_penetration_depth_p95": max(penetration_p95, default=math.nan),
        "max_residual_primal_feasibility": max(residual_primals, default=math.nan),
        "max_residual_dual_feasibility": max(residual_duals, default=math.nan),
        "max_residual_complementarity": max(
            residual_complementarities, default=math.nan
        ),
        "min_accepted_gamma": min(accepted_gammas, default=math.nan),
        "max_accepted_gamma": max(accepted_gammas, default=math.nan),
        "min_safe_gamma": min(safe_gammas, default=math.nan),
        "max_safe_gamma": max(safe_gammas, default=math.nan),
        "total_shrink_iterations": (
            sum(_int(row, "shrink_iterations") for row in exact_diagnostic_rows)
            if exact_diagnostic_rows
            else math.nan
        ),
        "max_coupling_variation_ratio": max(coupling_variations, default=math.nan),
        "mean_warm_start_matched_fraction": (
            statistics.fmean(warm_start_matched_fractions)
            if warm_start_matched_fractions
            else math.nan
        ),
        "total_fbf_iterations": sum(_int(row, "step_fbf_iterations") for row in rows),
        "median_step_fbf_iterations": statistics.median(step_iterations),
        "p95_step_fbf_iterations": _percentile(step_iterations, 0.95),
        "warm_start_step_fraction": sum(
            _int(row, "step_warm_starts") > 0 for row in rows
        )
        / len(rows),
        "all_states_finite": all(_int(row, "finite_state") == 1 for row in rows),
        "all_solver_steps_successful": all_solver_steps_successful,
        "residual_pass_fraction": residual_pass_fraction,
        "paper_reference_ms": reference,
        "paper_residual_pass_fraction": convergence_reference[0],
        "paper_median_outer_iterations": convergence_reference[1],
        "paper_p95_outer_iterations": convergence_reference[2],
        "paper_reference_contacts": reference_contacts,
        "contacts_match_paper_reference": contacts_match,
        "paper_workload_contract_valid": paper_workload_contract_valid,
        "paper_workload_contract_reasons": ";".join(workload_reasons) or "valid",
        "paper_hardware_contract": paper_hardware_contract,
        "paper_timing_comparable": paper_timing_comparable,
        "paper_target_evaluated": paper_target_evaluated,
        "paper_ratio_to_reference": paper_ratio_to_reference,
        "paper_mean_target_met": paper_mean_target_met,
        "paper_comparison_note": paper_comparison_note,
        "precision_contract": precision,
        "scene_contract": scene,
        "baumgarte_contract": baumgarte,
        "inner_local_solver": inner_local_solver,
        "inner_sweeps_requested": inner_sweeps_requested,
        "fixed_inner_sweeps_requested": fixed_inner_sweeps_requested,
        "step_size_persistence_enabled": step_size_persistence_enabled,
        "step_size_recovery_growth_factor": step_size_recovery_growth_factor,
        "step_size_persistence_used_steps": step_size_persistence_used_steps,
        "min_step_size_persistence_request": min(
            step_size_persistence_requests, default=math.nan
        ),
        "max_step_size_persistence_request": max(
            step_size_persistence_requests, default=math.nan
        ),
        "row_operator_request": row_operator_request,
        "row_operator_modes": row_operator_modes,
        "exact_diagnostics_contract": _unique(rows, "exact_diagnostics_contract"),
        "max_outer_iterations": max_outer_iterations,
        "tolerance": tolerance,
        "accept_outer_max_iterations": accept_outer_max_iterations,
        "inner_local_iterations": inner_local_iterations,
        "adaptive_step_size_enabled": adaptive_step_size_enabled,
        "warm_start_enabled": warm_start_enabled,
        "projected_gradient_retry_enabled": projected_gradient_retry_enabled,
        "dense_residual_polish_enabled": dense_residual_polish_enabled,
        "fallback_to_boxed_lcp_enabled": fallback_to_boxed_lcp_enabled,
        "diagonal_seed_enabled": diagonal_seed_enabled,
        "matrix_free_seed_enabled": matrix_free_seed_enabled,
        "step_size_scale": step_size_scale,
        "outer_relaxation": outer_relaxation,
        "initial_gamma_contract": initial_gamma_contract,
        "split_impulse_enabled": split_impulse_enabled,
    }
    return summary


def _empty_summary(
    key: tuple[str, str, str, str, str],
    failed_processes: int,
    records: Sequence[dict[str, object]],
    *,
    expected_repetitions: int | None,
    configured_warmup_repetitions: int,
) -> dict[str, object]:
    scenario, solver, solver_contract, collision_frontend, raw_threads = key
    requested_threads = int(raw_threads)
    measured_records = [record for record in records if not record["warmup"]]
    warmup_records = [record for record in records if record["warmup"]]
    if expected_repetitions is None:
        expected_repetitions = len(measured_records)
    completed_warmup_trajectories = sum(
        bool(record.get("complete_rows")) and int(record.get("returncode", 1)) == 0
        for record in warmup_records
    )
    failed_warmup_processes = sum(
        int(record.get("returncode", 1)) != 0 or not bool(record.get("complete_rows"))
        for record in warmup_records
    )
    warmup_contract_reasons: list[str] = []
    if len(warmup_records) != configured_warmup_repetitions:
        warmup_contract_reasons.append("configured_warmup_count_mismatch")
    if completed_warmup_trajectories != configured_warmup_repetitions:
        warmup_contract_reasons.append("warmup_trajectory_failure_or_incomplete")
    warmup_contract_valid = not warmup_contract_reasons

    affinities = [
        record.get("cpu_affinity")
        for record in records
        if isinstance(record.get("cpu_affinity"), dict)
    ]

    def unique_affinity_value(name: str) -> object | None:
        values = [affinity.get(name) for affinity in affinities]
        if not values:
            return None
        first = values[0]
        return first if all(value == first for value in values) else None

    raw_logical_cpus = unique_affinity_value("logical_cpus")
    affinity_logical_cpus = (
        ",".join(str(cpu) for cpu in raw_logical_cpus)
        if isinstance(raw_logical_cpus, list)
        else ""
    )
    raw_logical_cpu_mapping = unique_affinity_value("logical_cpu_physical_core_keys")
    affinity_logical_cpu_physical_core_keys = (
        ";".join(
            f"{cpu}={raw_logical_cpu_mapping[str(cpu)]}"
            for cpu in sorted(int(value) for value in raw_logical_cpu_mapping)
        )
        if isinstance(raw_logical_cpu_mapping, dict)
        and all(
            isinstance(key, str) and key.isdigit() and isinstance(value, str)
            for key, value in raw_logical_cpu_mapping.items()
        )
        else "unknown"
    )
    hardware_values = sorted(
        {
            str(record["paper_hardware_contract"])
            for record in records
            if record.get("paper_hardware_contract") is not None
        }
    )
    paper_hardware_contract = ";".join(hardware_values) or "unknown"
    reasons = ["no_measured_rows", "incomplete_measured_trajectory"]
    if failed_processes:
        reasons.append("measured_process_failure")
    reasons.extend(warmup_contract_reasons)
    reason_text = ";".join(reasons)
    comparison_reasons = list(reasons)
    if not paper_hardware_contract.startswith("apple_silicon_family_"):
        comparison_reasons.append(f"hardware_mismatch:{paper_hardware_contract}")

    scenario_steps, scenario_contract = SCENARIO_TRAJECTORY_CONTRACT.get(
        scenario, (None, "single_step_probe_or_full_length_not_defined")
    )
    paper_steps, paper_contract = PAPER_TRAJECTORY_CONTRACT.get(
        scenario, (None, "paper_timing_run_length_unpublished")
    )
    convergence_reference = PAPER_CONVERGENCE_REFERENCE.get(
        scenario, (math.nan, math.nan, math.nan)
    )
    if scenario in PHYSICAL_OUTCOME_TRACKED_BODIES:
        physical_outcome_valid: bool | None = False
        physical_outcome_reasons = "physical_outcome_no_measured_repetitions"
        physical_outcome_details = "no measured rows"
    else:
        physical_outcome_valid = None
        physical_outcome_reasons = "physical_outcome_contract_unavailable"
        physical_outcome_details = "no fixture-derived trajectory outcome contract"
    summary: dict[str, object] = {column: None for column in SUMMARY_COLUMNS}
    summary.update(
        {
            "scenario": scenario,
            "solver": solver,
            "solver_contract": solver_contract,
            "collision_frontend": collision_frontend,
            "requested_threads": requested_threads,
            "actual_threads": "",
            "repetitions": 0,
            "steps_per_repetition": "",
            "sample_steps": 0,
            "failed_processes": failed_processes,
            "mean_step_ms": math.nan,
            "median_step_ms": math.nan,
            "p95_step_ms": math.nan,
            "max_step_ms": math.nan,
            "mean_repetition_ms": math.nan,
            "steps_per_second": math.nan,
            "realtime_factor": math.nan,
            "realtime_threshold_ms": REALTIME_THRESHOLD_MS,
            "raw_mean_below_realtime": False,
            "raw_all_steps_below_realtime": False,
            "realtime_contract_valid": False,
            "realtime_contract_reasons": reason_text,
            "mean_realtime_target_met": None,
            "all_steps_realtime_target_met": None,
            "affinity_source": unique_affinity_value("source") or "unknown",
            "affinity_logical_cpus": affinity_logical_cpus,
            "affinity_logical_cpu_count": unique_affinity_value("logical_cpu_count"),
            "affinity_physical_core_count": unique_affinity_value(
                "physical_core_count"
            ),
            "affinity_one_logical_per_physical_core": unique_affinity_value(
                "one_logical_per_physical_core"
            ),
            "affinity_physical_core_keys": (
                ";".join(
                    str(value)
                    for value in (unique_affinity_value("physical_core_keys") or [])
                )
                or "unknown"
            ),
            "affinity_logical_cpu_physical_core_keys": (
                affinity_logical_cpu_physical_core_keys
            ),
            "affinity_package_ids": (
                ";".join(
                    str(value) for value in (unique_affinity_value("package_ids") or [])
                )
                or "unknown"
            ),
            "affinity_package_count": unique_affinity_value("package_count"),
            "affinity_smt_sibling_counts": (
                ";".join(
                    str(value)
                    for value in (unique_affinity_value("smt_sibling_counts") or [])
                )
                or "unknown"
            ),
            "affinity_core_classes_khz": (
                ";".join(
                    str(value)
                    for value in (unique_affinity_value("core_classes_khz") or [])
                )
                or "unknown"
            ),
            "affinity_scaling_governors": (
                ";".join(
                    str(value)
                    for value in (unique_affinity_value("scaling_governors") or [])
                )
                or "unknown"
            ),
            "single_core_claim_valid": False,
            "multicore_claim_valid": False,
            "parallelism_contract": "unavailable_no_measured_rows",
            "parallel_dispatch_valid": False,
            "parallel_dispatch_reasons": "no_measured_rows",
            "contact_row_delassus_products": 0,
            "parallel_contact_row_delassus_products": 0,
            "parallel_contact_row_product_fraction": math.nan,
            "parallel_contact_row_steps": 0,
            "max_contact_row_participants": 0,
            "inner_bgs_schedule_contract": (
                "dart_deterministic_manifold_colored_bgs_nonpaper"
                if solver_contract.startswith("dart_best_nonpaper_colored_inner_bgs")
                else "dart_legacy_serial_bgs"
            ),
            "colored_bgs_used_steps": 0,
            "colored_bgs_solves": 0,
            "colored_bgs_dispatches": 0,
            "max_colored_bgs_participants": 0,
            "colored_bgs_manifolds": "unavailable",
            "colored_bgs_colors": "unavailable",
            "colored_bgs_max_manifolds_per_color": 0,
            "colored_bgs_dispatch_valid": False,
            "colored_bgs_dispatch_reasons": "no_measured_rows",
            "observed_exact_colored_bgs_logical_cpus": "none",
            "observed_exact_colored_bgs_logical_cpu_count": 0,
            "observed_exact_contact_row_logical_cpus": "none",
            "observed_exact_contact_row_logical_cpu_count": 0,
            "observed_exact_contact_row_physical_core_count": 0,
            "phase_residency_logical_cpus_by_repetition": "",
            "phase_residency_min_logical_cpu_count": 0,
            "phase_residency_min_physical_core_count": 0,
            "runtime_cpu_residency_valid": False,
            "runtime_cpu_residency_reasons": (
                "per_parallel_phase_core_residency_unobserved;no_measured_rows"
                if requested_threads > 1
                else "not_applicable_single_thread"
            ),
            "raw_speedup_vs_one_thread": None,
            "validated_speedup_vs_one_thread": None,
            "measured_workload_fingerprint_sha256": None,
            "scaling_pair_valid": None,
            "scaling_pair_reasons": "not_evaluated",
            "configured_warmup_repetitions": configured_warmup_repetitions,
            "completed_warmup_trajectories": completed_warmup_trajectories,
            "failed_warmup_processes": failed_warmup_processes,
            "warmup_contract_valid": warmup_contract_valid,
            "warmup_contract_reasons": (";".join(warmup_contract_reasons) or "valid"),
            "measured_trajectories": len(measured_records),
            "complete_measured_trajectories": 0,
            "complete_requested_trajectory_evidence": False,
            "full_trajectory_evidence": False,
            "scenario_trajectory_steps": scenario_steps,
            "scenario_trajectory_contract": scenario_contract,
            "physical_outcome_valid": physical_outcome_valid,
            "physical_outcome_reasons": physical_outcome_reasons,
            "physical_outcome_details": physical_outcome_details,
            "controlled_affinity_valid": False,
            "controlled_affinity_reasons": "no_measured_rows",
            "paper_trajectory_steps": paper_steps,
            "paper_trajectory_contract": paper_contract,
            "exact_failures": None,
            "fallbacks": None,
            "solver_statuses": (
                "no_rows_process_failed" if failed_processes else "no_rows_incomplete"
            ),
            "all_solver_steps_accepted": False,
            "all_solver_steps_successful": False,
            "all_states_finite": False,
            "paper_reference_ms": PAPER_REFERENCE_MS.get(scenario, math.nan),
            "paper_residual_pass_fraction": convergence_reference[0],
            "paper_median_outer_iterations": convergence_reference[1],
            "paper_p95_outer_iterations": convergence_reference[2],
            "paper_reference_contacts": PAPER_REFERENCE_CONTACTS.get(scenario),
            "contacts_match_paper_reference": None,
            "paper_workload_contract_valid": False,
            "paper_workload_contract_reasons": reason_text,
            "paper_hardware_contract": paper_hardware_contract,
            "paper_timing_comparable": False,
            "paper_target_evaluated": False,
            "paper_ratio_to_reference": None,
            "paper_mean_target_met": None,
            "paper_comparison_note": ";".join(comparison_reasons),
            "precision_contract": "unavailable_no_measured_rows",
            "scene_contract": "unavailable_no_measured_rows",
            "baumgarte_contract": "unavailable_no_measured_rows",
            "inner_local_solver": "unavailable_no_measured_rows",
            "inner_sweeps_requested": "unavailable_no_measured_rows",
            "fixed_inner_sweeps_requested": "unavailable_no_measured_rows",
            "step_size_persistence_enabled": "unavailable_no_measured_rows",
            "step_size_recovery_growth_factor": "unavailable_no_measured_rows",
            "step_size_persistence_used_steps": None,
            "row_operator_request": "unavailable_no_measured_rows",
            "row_operator_modes": "unavailable_no_measured_rows",
            "exact_diagnostics_contract": "unavailable_no_measured_rows",
            "max_outer_iterations": "unavailable_no_measured_rows",
            "tolerance": "unavailable_no_measured_rows",
            "accept_outer_max_iterations": "unavailable_no_measured_rows",
            "inner_local_iterations": "unavailable_no_measured_rows",
            "adaptive_step_size_enabled": "unavailable_no_measured_rows",
            "warm_start_enabled": "unavailable_no_measured_rows",
            "projected_gradient_retry_enabled": "unavailable_no_measured_rows",
            "dense_residual_polish_enabled": "unavailable_no_measured_rows",
            "fallback_to_boxed_lcp_enabled": "unavailable_no_measured_rows",
            "diagonal_seed_enabled": "unavailable_no_measured_rows",
            "matrix_free_seed_enabled": "unavailable_no_measured_rows",
            "step_size_scale": "unavailable_no_measured_rows",
            "outer_relaxation": "unavailable_no_measured_rows",
            "initial_gamma_contract": "unavailable_no_measured_rows",
            "split_impulse_enabled": "unavailable_no_measured_rows",
        }
    )
    return summary


def _summaries(
    rows: Sequence[dict[str, str]],
    records: Sequence[dict[str, object]],
    args: argparse.Namespace | None = None,
) -> list[dict[str, object]]:
    row_keys = {
        (
            row["scenario"],
            row["solver"],
            row["solver_contract"],
            row["collision_frontend"],
            row["requested_threads"],
        )
        for row in rows
        if row["warmup"] == "0"
    }
    requested_keys = (
        {
            (
                case.scenario,
                args.solver,
                _expected_solver_contract(args.contract, threads),
                args.collision_frontend,
                str(threads),
            )
            for case in args.case
            for threads in args.threads
        }
        if args
        else set()
    )
    keys = sorted(row_keys | requested_keys)
    output: list[dict[str, object]] = []
    for key in keys:
        group = [
            row
            for row in rows
            if row["warmup"] == "0"
            and (
                row["scenario"],
                row["solver"],
                row["solver_contract"],
                row["collision_frontend"],
                row["requested_threads"],
            )
            == key
        ]
        group_records = [
            record
            for record in records
            if record["scenario"] == key[0]
            and record["collision_frontend"] == key[3]
            and int(record["threads"]) == int(key[4])
        ]
        failures = sum(
            int(record["returncode"]) != 0 or not bool(record["complete_rows"])
            for record in group_records
            if not record["warmup"]
        )
        if group:
            output.append(
                _summarize_group(
                    group,
                    failures,
                    records=group_records,
                    expected_repetitions=(args.repetitions if args else None),
                    configured_warmup_repetitions=(
                        args.warmup_repetitions
                        if args
                        else len(
                            [record for record in group_records if record["warmup"]]
                        )
                    ),
                )
            )
        else:
            output.append(
                _empty_summary(
                    key,
                    failures,
                    group_records,
                    expected_repetitions=(args.repetitions if args else None),
                    configured_warmup_repetitions=(
                        args.warmup_repetitions
                        if args
                        else len(
                            [record for record in group_records if record["warmup"]]
                        )
                    ),
                )
            )

    one_thread = {
        (
            row["scenario"],
            row["solver"],
            row["collision_frontend"],
        ): row
        for row in output
        if row["requested_threads"] == 1
    }
    for row in output:
        if row["requested_threads"] == 1:
            row["scaling_pair_valid"] = None
            row["scaling_pair_reasons"] = "not_applicable_one_thread_baseline"
            mean_step_ms = row["mean_step_ms"]
            if (
                isinstance(mean_step_ms, (int, float))
                and math.isfinite(float(mean_step_ms))
                and mean_step_ms > 0.0
            ):
                row["raw_speedup_vs_one_thread"] = 1.0
                row["validated_speedup_vs_one_thread"] = (
                    1.0 if row["single_core_claim_valid"] else None
                )
            continue
        baseline = one_thread.get(
            (row["scenario"], row["solver"], row["collision_frontend"])
        )
        if baseline is None:
            row["scaling_pair_valid"] = False
            row["scaling_pair_reasons"] = "one_thread_baseline_unavailable"
            continue
        row_mean = row["mean_step_ms"]
        baseline_mean = baseline["mean_step_ms"]
        if (
            isinstance(row_mean, (int, float))
            and isinstance(baseline_mean, (int, float))
            and math.isfinite(float(row_mean))
            and math.isfinite(float(baseline_mean))
            and row_mean > 0.0
            and baseline_mean > 0.0
        ):
            row["raw_speedup_vs_one_thread"] = baseline_mean / row_mean

        scaling_pair_reasons: list[str] = []
        if not baseline["single_core_claim_valid"]:
            scaling_pair_reasons.append("baseline_single_core_claim_invalid")
        if not row["multicore_claim_valid"]:
            scaling_pair_reasons.append("candidate_multicore_claim_invalid")

        baseline_cpu_text = baseline["affinity_logical_cpus"]
        candidate_cpu_text = row["affinity_logical_cpus"]
        baseline_cpus = (
            {int(value) for value in baseline_cpu_text.split(",")}
            if isinstance(baseline_cpu_text, str)
            and re.fullmatch(r"[0-9]+(?:,[0-9]+)*", baseline_cpu_text)
            else None
        )
        candidate_cpus = (
            {int(value) for value in candidate_cpu_text.split(",")}
            if isinstance(candidate_cpu_text, str)
            and re.fullmatch(r"[0-9]+(?:,[0-9]+)*", candidate_cpu_text)
            else None
        )
        if (
            baseline_cpus is None
            or candidate_cpus is None
            or not baseline_cpus.issubset(candidate_cpus)
        ):
            scaling_pair_reasons.append("baseline_cpu_list_not_nested")

        for field, reason in (
            ("affinity_package_ids", "scaling_package_mismatch"),
            ("affinity_smt_sibling_counts", "scaling_smt_topology_mismatch"),
            ("affinity_core_classes_khz", "scaling_core_class_mismatch"),
            ("affinity_scaling_governors", "scaling_governor_mismatch"),
        ):
            baseline_value = baseline[field]
            candidate_value = row[field]
            if (
                baseline_value in {None, "", "unknown"}
                or candidate_value in {None, "", "unknown"}
                or baseline_value != candidate_value
            ):
                scaling_pair_reasons.append(reason)

        baseline_mapping = _parse_encoded_physical_core_mapping(
            baseline["affinity_logical_cpu_physical_core_keys"]
        )
        candidate_mapping = _parse_encoded_physical_core_mapping(
            row["affinity_logical_cpu_physical_core_keys"]
        )
        if (
            baseline_cpus is None
            or baseline_mapping is None
            or candidate_mapping is None
            or any(
                cpu not in baseline_mapping
                or cpu not in candidate_mapping
                or baseline_mapping[cpu] != candidate_mapping[cpu]
                for cpu in baseline_cpus
            )
        ):
            scaling_pair_reasons.append("nested_cpu_physical_mapping_mismatch")

        if baseline["solver_contract"] == "dart_best":
            expected_candidate_contract = (
                "dart_best_threaded_world_exact_contact_row_parallel_capable"
            )
        elif baseline["solver_contract"] == "dart_best_nonpaper_colored_inner_bgs":
            expected_candidate_contract = (
                "dart_best_nonpaper_colored_inner_bgs_threaded_world"
            )
        else:
            expected_candidate_contract = baseline["solver_contract"]
        if row["solver_contract"] != expected_candidate_contract:
            scaling_pair_reasons.append("solver_contract_pair_mismatch")
        for field in SCALING_WORKLOAD_OPTION_FIELDS:
            if baseline[field] != row[field]:
                scaling_pair_reasons.append(f"workload_option_mismatch:{field}")
        if (
            baseline["measured_workload_fingerprint_sha256"] in {None, ""}
            or row["measured_workload_fingerprint_sha256"] in {None, ""}
            or baseline["measured_workload_fingerprint_sha256"]
            != row["measured_workload_fingerprint_sha256"]
        ):
            scaling_pair_reasons.append("measured_workload_trace_mismatch")

        row["scaling_pair_valid"] = not scaling_pair_reasons
        row["scaling_pair_reasons"] = ";".join(scaling_pair_reasons) or "valid"
        if row["scaling_pair_valid"]:
            row["validated_speedup_vs_one_thread"] = row["raw_speedup_vs_one_thread"]
    return output


def _write_csv(
    path: Path, columns: Sequence[str], rows: Iterable[dict[str, object]]
) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _json_safe(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def _strict_json_text(value: object) -> str:
    return (
        json.dumps(
            _json_safe(value),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    )


def _report_number(value: object, format_spec: str) -> str:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return format(value, format_spec)
    return ""


def _write_report(path: Path, summaries: Sequence[dict[str, object]]) -> None:
    lines = [
        "# FBF CPU evidence summary",
        "",
        "Raw timings cover `World::step()` only. A realtime or paper-target "
        "verdict is emitted only when its corresponding evidence contract is "
        "valid. Blank target cells mean unevaluated, not failed.",
        "",
        "`Mean realtime met` is a throughput-average verdict. `Every-step "
        "deadline met` is the stricter latency verdict: every measured "
        f"`World::step()` must finish below {REALTIME_THRESHOLD_MS:.9g} ms.",
        "",
        "A requested case/thread group with no measured rows remains visible "
        "and is marked incomplete; `--accept-nonzero` never turns a failed "
        "process into successful evidence.",
        "",
        "| Scenario | Frontend | Threads | Affinity | Samples | Failed "
        "measured | Failed warmups | Complete | Outcome | Controlled affinity | Mean ms | "
        "p95 ms | RTF | Realtime contract valid | Mean realtime met | "
        "Every-step deadline met | Residual pass | Fallbacks | Paper ref ms | "
        "Paper evaluated/met |",
        "| --- | --- | ---: | --- | ---: | ---: | ---: | --- | --- | --- | "
        "---: | ---: | ---: | --- | --- | --- | ---: | ---: | ---: | --- |",
    ]
    for row in summaries:
        lines.append(
            f"| {row['scenario']} | {row['collision_frontend']} | "
            f"{row['requested_threads']} | {row['affinity_logical_cpus']} | "
            f"{row['sample_steps']} | {row['failed_processes']} | "
            f"{row['failed_warmup_processes']} | "
            f"{row['complete_requested_trajectory_evidence']} | "
            f"{row['physical_outcome_valid']} | "
            f"{row['controlled_affinity_valid']} | "
            f"{_report_number(row['mean_step_ms'], '.6g')} | "
            f"{_report_number(row['p95_step_ms'], '.6g')} | "
            f"{_report_number(row['realtime_factor'], '.6g')} | "
            f"{row['realtime_contract_valid']} | "
            f"{row['mean_realtime_target_met']} | "
            f"{row['all_steps_realtime_target_met']} | "
            f"{_report_number(row['residual_pass_fraction'], '.3f')} | "
            f"{'' if row['fallbacks'] is None else row['fallbacks']} | "
            f"{_report_number(row['paper_reference_ms'], '.6g')} | "
            f"{row['paper_target_evaluated']}/{row['paper_mean_target_met']} |"
        )
    lines.extend(
        [
            "",
            "## Physical outcome audit",
            "",
            "| Scenario | Threads | Valid | Reasons | Details |",
            "| --- | ---: | --- | --- | --- |",
        ]
    )
    for row in summaries:
        details = str(row["physical_outcome_details"]).replace("|", "<br>")
        lines.append(
            f"| {row['scenario']} | {row['requested_threads']} | "
            f"{row['physical_outcome_valid']} | "
            f"{row['physical_outcome_reasons']} | {details} |"
        )
    lines.extend(
        [
            "",
            "## Exact-kernel parallelism audit",
            "",
            "| Scenario | Threads | Contract | Row products | Parallel products | "
            "Parallel fraction | Dispatch valid | Lifetime CPU residency | "
            "Per-phase-best CPUs | Phase residency valid/reasons | Multicore valid | "
            "Scaling pair valid/reasons | Raw/validated speedup |",
            "| --- | ---: | --- | ---: | ---: | ---: | --- | --- | --- | --- | "
            "--- | --- | --- |",
        ]
    )
    for row in summaries:
        phase_residency = str(
            row["phase_residency_logical_cpus_by_repetition"]
        ).replace("|", "<br>")
        lines.append(
            f"| {row['scenario']} | {row['requested_threads']} | "
            f"{row['parallelism_contract']} | "
            f"{row['contact_row_delassus_products']} | "
            f"{row['parallel_contact_row_delassus_products']} | "
            f"{_report_number(row['parallel_contact_row_product_fraction'], '.3f')} | "
            f"{row['parallel_dispatch_valid']} | "
            f"{row['observed_exact_contact_row_logical_cpus']} "
            f"({row['observed_exact_contact_row_physical_core_count']} cores) | "
            f"{phase_residency} | "
            f"{row['runtime_cpu_residency_valid']}<br>"
            f"{row['runtime_cpu_residency_reasons']} | "
            f"{row['multicore_claim_valid']} | "
            f"{row['scaling_pair_valid']}<br>{row['scaling_pair_reasons']} | "
            f"{_report_number(row['raw_speedup_vs_one_thread'], '.4g')}/"
            f"{_report_number(row['validated_speedup_vs_one_thread'], '.4g')} |"
        )
    colored_rows = [
        row
        for row in summaries
        if str(row["solver_contract"]).startswith(
            "dart_best_nonpaper_colored_inner_bgs"
        )
    ]
    if colored_rows:
        lines.extend(
            [
                "",
                "## Non-paper colored inner-BGS audit",
                "",
                "| Scenario | Threads | Used steps | Frozen solves | Persistent "
                "dispatches | Participants | Manifolds/colors/max width | "
                "Dispatch valid/reasons | Runtime CPUs |",
                "| --- | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |",
            ]
        )
        for row in colored_rows:
            lines.append(
                f"| {row['scenario']} | {row['requested_threads']} | "
                f"{row['colored_bgs_used_steps']} | {row['colored_bgs_solves']} | "
                f"{row['colored_bgs_dispatches']} | "
                f"{row['max_colored_bgs_participants']} | "
                f"{row['colored_bgs_manifolds']}/"
                f"{row['colored_bgs_colors']}/"
                f"{row['colored_bgs_max_manifolds_per_color']} | "
                f"{row['colored_bgs_dispatch_valid']}<br>"
                f"{row['colored_bgs_dispatch_reasons']} | "
                f"{row['observed_exact_colored_bgs_logical_cpus']} |"
            )
    lines.extend(
        [
            "",
            "Contract rejection reasons are preserved in `summary.csv` and "
            "`summary.json`. The legacy dart_best multicore verdict requires "
            "measured parallel contact-row products. The explicitly non-paper "
            "colored contract instead requires colored path use, exactly one "
            "persistent dispatch per eligible multi-thread exact attempt, "
            "per-positive-work-step schedule width at least equal to the "
            "requested threads, no legacy parallel contact-row activity, and "
            "colored-dispatch CPU residency. In both cases every "
            "repetition's largest within-phase CPU-ID set must map to the "
            "requested number of pinned physical cores. Lifetime CPU-ID unions "
            "are audit-only because migration across phases can populate them. "
            "Runtime CPU IDs are residency observations, not proof of perfect "
            "simultaneity.",
            "",
            "Validated scaling additionally requires a nested one-thread CPU "
            "list and physical mapping, matching package, SMT topology, cpuinfo "
            "maximum-frequency core class, scaling governor, solver/workload "
            "options, and an exact measured-work trajectory fingerprint. Every "
            "configured warmup must also complete successfully. Raw speedup "
            "remains visible when a scaling-pair gate fails.",
            "",
            "Schema v8 preserves the 83-column v7 default trace byte-for-byte "
            "and selects a 95-column extended trace only for the non-paper "
            "colored contract. The paper's "
            "local block formula, "
            "gamma-recovery rule, and DART split-impulse equivalence are not "
            "published, so those gaps keep an apples-to-apples paper verdict "
            "unevaluated.",
            "",
            "See `metadata.json`, `invocations.json`, `raw.csv`, and the "
            "per-process files under `raw/` for the complete evidence chain.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(list(argv) if argv is not None else sys.argv[1:])
    args.binary = args.binary.resolve()
    args.output_dir = args.output_dir.resolve()
    if not args.binary.is_file() or not os.access(args.binary, os.X_OK):
        raise SystemExit(f"executable not found: {args.binary}")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise SystemExit(f"output directory is not empty: {args.output_dir}")
    try:
        args.executed_tool_closure = _bind_executed_tool_closure(args)
    except (OSError, RuntimeError) as error:
        raise SystemExit(str(error)) from error
    args.executed_tool_identity_rechecks = []
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = args.output_dir / "raw"
    raw_dir.mkdir()

    root_text = _run_text(
        [
            "git",
            "-C",
            str(Path(__file__).resolve().parent),
            "rev-parse",
            "--show-toplevel",
        ]
    )
    root = Path(root_text) if root_text else Path.cwd()
    metadata = _metadata(args, root)
    (args.output_dir / "metadata.json").write_text(
        _strict_json_text(metadata), encoding="utf-8"
    )

    all_rows: list[dict[str, str]] = []
    records: list[dict[str, object]] = []
    for case in args.case:
        for threads in args.threads:
            for warmup_index in range(args.warmup_repetitions):
                invocation = Invocation(case, threads, warmup_index + 1, True)
                rows, record = _run_invocation(args, invocation, raw_dir)
                all_rows.extend(rows)
                records.append(record)
            for repetition in range(1, args.repetitions + 1):
                invocation = Invocation(case, threads, repetition, False)
                rows, record = _run_invocation(args, invocation, raw_dir)
                all_rows.extend(rows)
                records.append(record)

    (args.output_dir / "invocations.json").write_text(
        _strict_json_text(records),
        encoding="utf-8",
    )
    trace_columns = (
        TRACE_COLORED_COLUMNS
        if args.contract == "dart_best_colored_bgs"
        else TRACE_COLUMNS
    )
    raw_columns = (*RUN_COLUMNS, *trace_columns)
    _write_csv(args.output_dir / "raw.csv", raw_columns, all_rows)
    summaries = _summaries(all_rows, records, args)
    _write_csv(args.output_dir / "summary.csv", SUMMARY_COLUMNS, summaries)
    (args.output_dir / "summary.json").write_text(
        _strict_json_text(summaries),
        encoding="utf-8",
    )
    _write_report(args.output_dir / "REPORT.md", summaries)

    source_identity_after = _source_identity(root)
    runtime_identity_after = _runtime_identity(args.binary)
    binary_sha256_after = _sha256(args.binary)
    if source_identity_after != metadata["source_identity"]:
        raise RuntimeError("evidence source identity changed during execution")
    if runtime_identity_after != metadata["runtime_identity"]:
        raise RuntimeError("evidence runtime identity changed during execution")
    if binary_sha256_after != metadata["binary"]["sha256"]:
        raise RuntimeError("evidence binary changed during execution")
    metadata["executed_tool_identity_rechecks"] = list(
        args.executed_tool_identity_rechecks
    )
    metadata["identity_recheck"] = {
        "stage": "after_all_invocations",
        "source_identity": source_identity_after,
        "runtime_identity": runtime_identity_after,
        "binary_sha256": binary_sha256_after,
    }
    (args.output_dir / "metadata.json").write_text(
        _strict_json_text(metadata), encoding="utf-8"
    )
    (args.output_dir / "artifact-index.json").write_text(
        _strict_json_text(_artifact_index(args.output_dir)),
        encoding="utf-8",
    )

    failed = any(int(record["returncode"]) != 0 for record in records)
    return 0 if args.accept_nonzero or not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
