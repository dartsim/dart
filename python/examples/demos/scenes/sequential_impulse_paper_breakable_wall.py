"""Sequential-impulse row for the shared DART Figure 13 reconstruction."""

from __future__ import annotations

from typing import Any

import dartpy as sx

from ..runner import PythonDemoScene, SceneSetup
from .avbd_paper_breakable_wall import (
    OUTCOME_METRIC_THRESHOLDS,
    PAPER_REFERENCE,
    build_solver_variant,
)

SEQUENTIAL_IMPULSE_OUTCOME_ORACLE: dict[str, Any] = {
    **OUTCOME_METRIC_THRESHOLDS,
    "evaluation_frame": 14,
    "collapse_evaluation_frame": 120,
    "joint_evidence_frames": (14, 120, 600),
    "minimum_initial_broken_joints": 3,
    "maximum_initial_broken_joints": 20,
    "minimum_initial_unbroken_joints": 690,
    "minimum_initial_total_retained_fraction": 0.95,
    "minimum_initial_broken_joints_per_impact_region": 1,
    "maximum_initial_outside_joint_linear_residual": 0.04,
    "maximum_initial_outside_joint_angular_residual_radians": 0.13,
    "expected_broken_joints": 5,
    "expected_broken_joint_ids_sha256": (
        "c85184879b1b9036ff582731031fc49c56b4149e93ded45780b06352bd94d61d"
    ),
    "expected_outside_impact_unbroken_joint_count": 484,
    "minimum_final_broken_joints": 3,
    "maximum_final_broken_joints": 30,
    "minimum_final_unbroken_joints": 680,
    "minimum_displaced_bricks_per_impact_band": 10,
    "minimum_collapse_outside_joint_maximum_linear_residual": 0.05,
    "minimum_collapse_outside_joint_rms_linear_residual": 0.015,
    "minimum_collapse_outside_joint_maximum_angular_residual_radians": 0.60,
    "minimum_collapse_outside_joint_rms_angular_residual_radians": 0.18,
    "maximum_collapse_outside_retained_fraction": 0.35,
    "maximum_collapse_total_retained_fraction": 0.25,
    "minimum_collapse_wall_normal_displacement": 2.0,
}


def build() -> SceneSetup:
    """Build the public SI row of the shared DART reconstruction."""
    return build_solver_variant(
        rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
        solver_key="sequential_impulse",
        solver_display="Sequential Impulse",
        paper_reference=PAPER_REFERENCE,
        outcome_oracle=SEQUENTIAL_IMPULSE_OUTCOME_ORACLE,
        capture_assessment_frames=(
            int(SEQUENTIAL_IMPULSE_OUTCOME_ORACLE["evaluation_frame"]),
            int(
                SEQUENTIAL_IMPULSE_OUTCOME_ORACLE[
                    "collapse_evaluation_frame"
                ]
            ),
            600,
        ),
        resolved_solver_key="sequential-impulse",
    )


SCENE = PythonDemoScene(
    id="sequential_impulse_paper_breakable_wall",
    title="Sequential Impulse Figure 13 Reconstruction (sx)",
    category="Sequential Impulse Rigid Constraints (sx)",
    summary="The SI row of DART's shared publication-shaped Figure 13 "
    "reconstruction: 20 PGS sweeps, "
    "impulse-derived fracture, and non-velocity post-stabilization.",
    build=build,
)
