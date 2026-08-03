"""Matched fixed-penalty VBD row for the Figure 13 breakable wall."""

from __future__ import annotations

from typing import Any

import dartpy as sx

from ..runner import PythonDemoScene, SceneSetup
from .avbd_paper_breakable_wall import (
    OUTCOME_METRIC_THRESHOLDS,
    PAPER_REFERENCE as _SHARED_PAPER_REFERENCE,
    build_solver_variant,
)

PAPER_REFERENCE: dict[str, Any] = {
    **_SHARED_PAPER_REFERENCE,
    "published_facts": {
        **_SHARED_PAPER_REFERENCE["published_facts"],
        "vbd_outcome": "the wall bends under impact but does not break",
    },
    "dart_reconstruction": dict(_SHARED_PAPER_REFERENCE["dart_reconstruction"]),
}

OUTCOME_ORACLE: dict[str, Any] = {
    **OUTCOME_METRIC_THRESHOLDS,
    "evaluation_frame": 18,
    "retention_evaluation_frame": 120,
    "joint_evidence_frames": (18, 120),
    "maximum_broken_joints": 0,
    "minimum_unbroken_joints": 712,
    "minimum_maximum_wall_normal_displacement": 0.10,
    "minimum_rms_wall_normal_displacement": 0.05,
    "bent_brick_displacement_threshold": 0.05,
    "minimum_bent_bricks": 100,
    "minimum_total_retained_fraction": 0.99,
    "maximum_unbroken_joint_linear_residual": 0.025,
    "maximum_unbroken_joint_angular_residual_radians": 0.02,
}


def build() -> SceneSetup:
    return build_solver_variant(
        rigid_body_solver=sx.RigidBodySolver.VBD,
        solver_key="vbd",
        solver_display="VBD",
        paper_reference=PAPER_REFERENCE,
        outcome_oracle=OUTCOME_ORACLE,
        capture_assessment_frames=(
            int(OUTCOME_ORACLE["evaluation_frame"]),
            int(OUTCOME_ORACLE["retention_evaluation_frame"]),
        ),
    )


SCENE = PythonDemoScene(
    id="vbd_paper_breakable_wall",
    title="VBD Paper Breakable Wall (sx)",
    category="VBD Rigid Constraints (sx)",
    summary="The matched fixed-penalty Figure 13 wall row: 20 sweeps, retained "
    "topology, and a quantitative bending oracle.",
    build=build,
)
