#!/usr/bin/env python3
"""Validate committed AVBD evidence packets against the shared schema.

Enforces PLAN-091 WP-091.1: AVBD packets at the current schema version
must machine-record the resolved solver configuration and the rigid-contact
selection source that actually ran (``resolved_solver_identity``). Packets
committed before the identity contract stay readable through a legacy
allowlist, but new packet files must be written at the current schema version.
The field contract lives in ``scripts/avbd_packet_schema.py``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import struct
import sys
import uuid
from collections.abc import Iterator, Mapping
from dataclasses import dataclass, field
from datetime import datetime
from fractions import Fraction
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    MULTIBODY_IDENTITY_MIN_SCHEMA_VERSION,
    PAPER_PACKET_SOURCE_PATHS,
    PLAN104_CLAIMS_KEY,
    PLAN104_CLAIMS_MIN_SCHEMA_VERSION,
    SOLVER_CONFIGURATION_MIN_SCHEMA_VERSION,
    SOURCE_PROVENANCE_ALGORITHM,
    evidence_definition_matches,
    packet_schema_version_errors,
    plan104_claims_errors,
    resolved_solver_identity_errors,
    stable_counter_stddev_is_noise,
)

__all__ = [
    "AVBD_PACKET_SCHEMA_VERSION",
    "PacketValidationContext",
    "packet_errors",
]
from capture_source_provenance import (  # noqa: E402
    CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
    CAPTURE_LOADER_ENVIRONMENT_PREFIXES,
    CAPTURE_LOADER_POLICY_ALGORITHM,
    CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM,
    CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM,
    CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
    CAPTURE_SCREENSHOT_BINDING_ALGORITHM,
    CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
    CAPTURE_VIDEO_CONTENT_CORRESPONDENCE_ALGORITHM,
    CAPTURE_VIDEO_ENCODER,
    CAPTURE_VIDEO_PROBE_ALGORITHM,
    DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
    CaptureSourceProvenanceError,
    compute_capture_source_provenance,
)
from run_figure13_benchmark import (  # noqa: E402
    BUILD_CONFIGURATION_ALGORITHM,
    BUILD_CONFIGURATION_KEYS,
    EVIDENCE_CMAKE_DEFINITIONS,
    LOADER_ENVIRONMENT_PREFIXES,
    LOADER_POLICY_ALGORITHM,
    REQUIRED_RUNTIME_IMAGE_ROLES,
    RUNTIME_IMAGE_INVENTORY_ALGORITHM,
)

REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_PACKET_DIR = REPO_ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"
PACKET_GLOB = "avbd-*-packet.json"
LINKED_PACKET_KEYS = ("linked_avbd_evidence", "linked_avbd_vbd_evidence")
BENCHMARK_SOURCE_PATH = Path("tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp")
FIGURE13_BENCHMARK_FILTER = (
    "^BM_(Avbd|Vbd|SequentialImpulse)PaperBreakableWallStep/iterations:120$"
)
FIGURE13_BENCHMARK_RUN_SCHEMA = "dart.figure13_benchmark_run/v3"
BENCHMARK_BUILD_IDENTITY_ALGORITHM = "sha256-canonical-compiled-benchmark-identity-v3"
BENCHMARK_SOURCE_PROVENANCE_ALGORITHM = (
    "sha256-canonical-benchmark-source-and-executable-v3"
)
SEMANTIC_CLAIM_ASSESSMENTS = {
    "capture_checkpoint_semantics": "supported",
    "cuda_parity": "not_proven",
    "exact_unpublished_source_scene": "not_proven",
    "full_interval_video_semantics": "supported",
    "other_figure13_solver_rows": "not_proven",
    "paper_figure_visual_agreement": "supported",
    "published_timing_parity": "not_proven",
    "text_oracle_agreement": "supported",
    "view_report_agreement": "supported",
    "xpbd_parity": "not_proven",
}
SEMANTIC_TERMINAL_BEHAVIOR = {
    "avbd-paper-breakable-wall-packet.json": "retained_damaged_wall",
    "avbd-paper-vbd-comparison-packet.json": "bent_retained_wall",
    "avbd-paper-sequential-impulse-comparison-packet.json": "collapsed_wall",
}
# The public AVBD reconstruction under the immutable paper profile keeps the
# wall standing with three small joint-break clusters and no displaced bricks;
# Figure 13(d) shows the wall broken open with flying bricks, so visual
# agreement with the paper figure is not proven for that row.
SEMANTIC_CLAIM_ASSESSMENTS_BY_TERMINAL_BEHAVIOR = {
    "retained_damaged_wall": {
        **SEMANTIC_CLAIM_ASSESSMENTS,
        "paper_figure_visual_agreement": "not_proven",
    },
    "bent_retained_wall": dict(SEMANTIC_CLAIM_ASSESSMENTS),
    "collapsed_wall": dict(SEMANTIC_CLAIM_ASSESSMENTS),
}
SEMANTIC_STRUCTURED_OBSERVATIONS = {
    "retained_damaged_wall": {
        "checkpoint_sequence": [
            "three_localized_joint_break_clusters_at_frame_60",
            "retained_damaged_wall_at_frame_120",
            "retained_damaged_wall_at_frame_600",
        ],
        "paper_figure_relationship": "retained_wall_without_the_paper_fracture",
        "temporal_behavior": "retained_damaged_wall",
        "text_oracle_relationship": "agrees",
        "view_report_relationship": "agrees",
    },
    "bent_retained_wall": {
        "checkpoint_sequence": [
            "distributed_bend_at_frame_18",
            "bent_retained_wall_at_frame_120",
            "bent_retained_wall_at_frame_600",
        ],
        "paper_figure_relationship": "qualitative_agreement",
        "temporal_behavior": "bent_retained_wall",
        "text_oracle_relationship": "agrees",
        "view_report_relationship": "agrees",
    },
    "collapsed_wall": {
        "checkpoint_sequence": [
            "localized_three_region_fracture_at_frame_14",
            "collapsed_wall_at_frame_120",
            "collapsed_wall_at_frame_600",
        ],
        "paper_figure_relationship": "qualitative_agreement",
        "temporal_behavior": "collapsed_wall",
        "text_oracle_relationship": "agrees",
        "view_report_relationship": "agrees",
    },
}
PAPER_CAPTURE_ROLES = {
    "avbd-paper-breakable-wall-packet.json": ("impact", "outcome"),
    "avbd-paper-vbd-comparison-packet.json": ("bend", "retention"),
    "avbd-paper-sequential-impulse-comparison-packet.json": (
        "fracture",
        "collapse",
    ),
}
PAPER_LONG_HORIZON_ROLE = "long_horizon"
PAPER_LONG_HORIZON_FRAME = 600

# Checker-owned copies of the Figure 13 outcome oracles. A packet grades its
# own physical outcome against the oracle it ships, so the thresholds have to
# be pinned here as well: without an independent copy a regenerated packet
# could relax `minimum_broken_joints` to 0 and still report a passing
# `physical_outcome_valid`. These must stay byte-identical to the writers'
# `OUTCOME_ORACLE` constants, which
# tests/test_check_avbd_packets.py::test_paper_outcome_oracles_match_writer_constants
# proves on every run.
_BREAKABLE_WALL_OUTCOME_ORACLE: dict[str, object] = {
    "evaluation_frame": 120,
    "expected_broken_joint_ids_sha256": (
        "e746389411f654ea64f2836db35c704443b2dac09186fc73d4a9341a18890fab"
    ),
    "impact_band_radius": 0.85,
    "impact_damage_displacement_threshold": 0.1,
    "joint_evidence_frames": [60, 120, 600],
    "maximum_broken_joints": 60,
    "maximum_broken_joints_outside_impact_regions": 21,
    "maximum_unbroken_joint_angular_residual_radians": 0.002,
    "maximum_unbroken_joint_linear_residual": 0.002,
    "minimum_broken_joints": 30,
    "minimum_broken_joints_per_impact_region": 4,
    "minimum_outside_retained_fraction": 0.95,
    "minimum_total_retained_fraction": 0.95,
    "minimum_unbroken_joints": 650,
    "outside_radius": 1.15,
    "retained_displacement_threshold": 0.5,
}
_VBD_COMPARISON_OUTCOME_ORACLE: dict[str, object] = {
    "bent_brick_displacement_threshold": 0.05,
    "evaluation_frame": 18,
    "impact_band_radius": 0.85,
    "impact_damage_displacement_threshold": 0.1,
    "joint_evidence_frames": [18, 120, 600],
    "maximum_broken_joints": 0,
    "maximum_unbroken_joint_angular_residual_radians": 0.02,
    "maximum_unbroken_joint_linear_residual": 0.025,
    "minimum_bent_bricks": 100,
    "minimum_maximum_wall_normal_displacement": 0.1,
    "minimum_rms_wall_normal_displacement": 0.05,
    "minimum_total_retained_fraction": 0.99,
    "minimum_unbroken_joints": 712,
    "outside_radius": 1.15,
    "retained_displacement_threshold": 0.5,
    "retention_evaluation_frame": 120,
}
_SEQUENTIAL_IMPULSE_OUTCOME_ORACLE: dict[str, object] = {
    "collapse_evaluation_frame": 120,
    "evaluation_frame": 14,
    "expected_broken_joint_ids_sha256": (
        "c85184879b1b9036ff582731031fc49c56b4149e93ded45780b06352bd94d61d"
    ),
    "expected_broken_joints": 5,
    "expected_outside_impact_unbroken_joint_count": 484,
    "impact_band_radius": 0.85,
    "impact_damage_displacement_threshold": 0.1,
    "joint_evidence_frames": [14, 120, 600],
    "maximum_collapse_outside_retained_fraction": 0.35,
    "maximum_collapse_total_retained_fraction": 0.25,
    "maximum_final_broken_joints": 30,
    "maximum_initial_broken_joints": 20,
    "maximum_initial_outside_joint_angular_residual_radians": 0.13,
    "maximum_initial_outside_joint_linear_residual": 0.04,
    "minimum_collapse_outside_joint_maximum_angular_residual_radians": 0.6,
    "minimum_collapse_outside_joint_maximum_linear_residual": 0.05,
    "minimum_collapse_outside_joint_rms_angular_residual_radians": 0.18,
    "minimum_collapse_outside_joint_rms_linear_residual": 0.015,
    "minimum_collapse_wall_normal_displacement": 2.0,
    "minimum_displaced_bricks_per_impact_band": 10,
    "minimum_final_broken_joints": 3,
    "minimum_final_unbroken_joints": 680,
    "minimum_initial_broken_joints": 3,
    "minimum_initial_broken_joints_per_impact_region": 1,
    "minimum_initial_total_retained_fraction": 0.95,
    "minimum_initial_unbroken_joints": 690,
    "outside_radius": 1.15,
    "retained_displacement_threshold": 0.5,
}
PAPER_OUTCOME_ORACLES: dict[str, dict[str, dict[str, object]]] = {
    packet_name: {
        checkpoint: oracle for checkpoint in (*roles, PAPER_LONG_HORIZON_ROLE)
    }
    for packet_name, roles, oracle in (
        (
            "avbd-paper-breakable-wall-packet.json",
            PAPER_CAPTURE_ROLES["avbd-paper-breakable-wall-packet.json"],
            _BREAKABLE_WALL_OUTCOME_ORACLE,
        ),
        (
            "avbd-paper-vbd-comparison-packet.json",
            PAPER_CAPTURE_ROLES["avbd-paper-vbd-comparison-packet.json"],
            _VBD_COMPARISON_OUTCOME_ORACLE,
        ),
        (
            "avbd-paper-sequential-impulse-comparison-packet.json",
            PAPER_CAPTURE_ROLES["avbd-paper-sequential-impulse-comparison-packet.json"],
            _SEQUENTIAL_IMPULSE_OUTCOME_ORACLE,
        ),
    )
}
PAPER_REQUIRED_MEDIAN_RATIO_KEYS = {
    "avbd-paper-vbd-comparison-packet.json": frozenset(
        {"vbd_to_avbd_median_cpu_cost_ratio"}
    ),
    "avbd-paper-sequential-impulse-comparison-packet.json": frozenset(
        {
            "sequential_impulse_to_avbd_median_cpu_cost_ratio",
            "sequential_impulse_to_vbd_median_cpu_cost_ratio",
        }
    ),
}
PAPER_FIGURE13_SPECS: dict[str, dict[str, Any]] = {
    "avbd-paper-breakable-wall-packet.json": {
        "packet": "avbd_paper_breakable_wall",
        "scene": "avbd_paper_breakable_wall",
        "identity_solver": "avbd",
        "capture_solver": "avbd",
        "selection": "world_solver_family",
        "public_solver": "AVBD",
        "scene_solver": "public_avbd",
        "captures": {
            "impact": {
                "review_role": "impact_frame_60",
                "frame": 60,
                "checkpoint": "outcome",
                "evaluated": False,
                "status": "pre-evaluation",
                "thresholds_pass": False,
                "evaluation_key": "evaluation_frame",
                "evaluation_frame": 120,
                "threshold_checks": (
                    "finite_state",
                    "fracture_activated",
                    "fracture_count_bounded",
                    "fracture_identity_matches",
                    "fracture_in_three_impact_regions",
                    "outside_breaks_bounded",
                    "outside_wall_retained",
                    "retained_joint_rows_satisfied",
                    "total_wall_retained",
                ),
            },
            "outcome": {
                "review_role": "outcome_frame_120",
                "frame": 120,
                "checkpoint": "outcome",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "evaluation_frame",
                "evaluation_frame": 120,
                "threshold_checks": (
                    "finite_state",
                    "fracture_activated",
                    "fracture_count_bounded",
                    "fracture_identity_matches",
                    "fracture_in_three_impact_regions",
                    "outside_breaks_bounded",
                    "outside_wall_retained",
                    "retained_joint_rows_satisfied",
                    "total_wall_retained",
                ),
            },
            "long_horizon": {
                "frame": PAPER_LONG_HORIZON_FRAME,
                "long_horizon": True,
                "review_role": "long_horizon_frame_600",
                "checkpoint": "outcome",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "evaluation_frame",
                "evaluation_frame": 120,
                "threshold_checks": (
                    "finite_state",
                    "fracture_activated",
                    "fracture_count_bounded",
                    "fracture_identity_matches",
                    "fracture_in_three_impact_regions",
                    "outside_breaks_bounded",
                    "outside_wall_retained",
                    "retained_joint_rows_satisfied",
                    "total_wall_retained",
                ),
            },
        },
        "benchmark_methods": {
            "avbd": {
                "location": "root",
                "benchmark": "BM_AvbdPaperBreakableWallStep",
                "runtime_solver": "avbd",
                "family_index": None,
            }
        },
    },
    "avbd-paper-vbd-comparison-packet.json": {
        "packet": "avbd_paper_vbd_comparison",
        "scene": "vbd_paper_breakable_wall",
        "identity_solver": "vbd",
        "capture_solver": "vbd",
        "selection": "world_solver_family",
        "public_solver": "VBD",
        "scene_solver": "public_vbd",
        "captures": {
            "bend": {
                "review_role": "bend_frame_18",
                "frame": 18,
                "checkpoint": "bend",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "evaluation_frame",
                "evaluation_frame": 18,
                "threshold_checks": (
                    "bend_is_spatially_resolved",
                    "finite_state",
                    "no_fracture",
                    "retained_joint_rows_satisfied",
                    "topology_retained",
                    "wall_bend_is_distributed",
                    "wall_bends",
                ),
            },
            "retention": {
                "review_role": "retention_frame_120",
                "frame": 120,
                "checkpoint": "retention",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "retention_evaluation_frame",
                "evaluation_frame": 120,
                "threshold_checks": (
                    "finite_state",
                    "no_fracture",
                    "retained_joint_rows_satisfied",
                    "topology_retained",
                    "wall_retained",
                ),
            },
            "long_horizon": {
                "frame": PAPER_LONG_HORIZON_FRAME,
                "long_horizon": True,
                "review_role": "long_horizon_frame_600",
                "checkpoint": "retention",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "retention_evaluation_frame",
                "evaluation_frame": 120,
                "threshold_checks": (
                    "finite_state",
                    "no_fracture",
                    "retained_joint_rows_satisfied",
                    "topology_retained",
                    "wall_retained",
                ),
            },
        },
        "benchmark_methods": {
            "avbd": {
                "location": "methods",
                "benchmark": "BM_AvbdPaperBreakableWallStep",
                "runtime_solver": "avbd",
                "family_index": 0,
            },
            "vbd": {
                "location": "methods",
                "benchmark": "BM_VbdPaperBreakableWallStep",
                "runtime_solver": "vbd",
                "family_index": 1,
            },
        },
    },
    "avbd-paper-sequential-impulse-comparison-packet.json": {
        "packet": "avbd_paper_sequential_impulse_comparison",
        "scene": "sequential_impulse_paper_breakable_wall",
        "identity_solver": "sequential_impulse",
        "capture_solver": "sequential-impulse",
        "selection": "contact_solver_method",
        "public_solver": "SEQUENTIAL_IMPULSE",
        "scene_solver": "public_sequential-impulse",
        "captures": {
            "fracture": {
                "review_role": "fracture_frame_14",
                "frame": 14,
                "checkpoint": "fracture",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "evaluation_frame",
                "evaluation_frame": 14,
                "threshold_checks": (
                    "finite_state",
                    "fracture_activated",
                    "initial_fracture_confined_to_impact_regions",
                    "initial_fracture_covers_three_impacts",
                    "initial_fracture_identity_matches",
                    "initial_retained_joint_rows_bounded",
                    "wall_initially_retained",
                ),
            },
            "collapse": {
                "review_role": "collapse_frame_120",
                "frame": 120,
                "checkpoint": "collapse",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "collapse_evaluation_frame",
                "evaluation_frame": 120,
                "threshold_checks": (
                    "damage_in_three_impact_bands",
                    "finite_state",
                    "fracture_identity_unchanged",
                    "initial_fracture_remains_visible",
                    "outside_wall_collapses",
                    "retained_rows_fail_outside_impacts",
                    "wall_collapses",
                ),
            },
            "long_horizon": {
                "frame": PAPER_LONG_HORIZON_FRAME,
                "long_horizon": True,
                "review_role": "long_horizon_frame_600",
                "checkpoint": "collapse",
                "evaluated": True,
                "status": "pass",
                "thresholds_pass": True,
                "evaluation_key": "collapse_evaluation_frame",
                "evaluation_frame": 120,
                "threshold_checks": (
                    "damage_in_three_impact_bands",
                    "finite_state",
                    "fracture_identity_unchanged",
                    "initial_fracture_remains_visible",
                    "outside_wall_collapses",
                    "retained_rows_fail_outside_impacts",
                    "wall_collapses",
                ),
            },
        },
        "benchmark_methods": {
            "sequential_impulse": {
                "location": "method",
                "benchmark": "BM_SequentialImpulsePaperBreakableWallStep",
                "runtime_solver": "sequential-impulse",
                "family_index": 2,
            }
        },
    },
}

_PAPER_FINGERPRINT_HEX_LENGTH = 16
_PAPER_TRAJECTORY_FRAMES = 120
_PAPER_RIGID_CONSTRAINT_ITERATIONS = 20
_PAPER_BENCHMARK_REPETITIONS = 5
_PAPER_BENCHMARK_ITERATIONS = 5
_PAPER_CAPTURE_VIDEO_FPS = 60
_PAPER_SCENE_COUNTERS = {
    "breakable_joints": 712,
    "collision_shapes": 256,
    "impacting_balls": 3,
    "rigid_bodies": 256,
    "rigid_body_joints": 712,
}


def _paper_requires_long_horizon(packet: Mapping[str, object]) -> bool:
    version = packet.get("schema_version")
    return (
        isinstance(version, int)
        and not isinstance(version, bool)
        and version >= SOLVER_CONFIGURATION_MIN_SCHEMA_VERSION
    )


def _paper_capture_roles(
    packet: Mapping[str, object], packet_name: str
) -> tuple[str, ...] | None:
    roles = PAPER_CAPTURE_ROLES.get(packet_name)
    if roles is None:
        return None
    if _paper_requires_long_horizon(packet):
        return (*roles, PAPER_LONG_HORIZON_ROLE)
    return roles


HISTORICAL_HIGH_RATIO_BOUNDARIES = {
    "avbd-articulated-high-ratio-chain-packet.json": {
        "supported_scope": (
            "historical_variational_multibody_capture_hash_and_cpu_metadata"
        ),
        "visual_boundary": True,
    },
    "avbd-paper-scale-high-ratio-chain-packet.json": {
        "supported_scope": (
            "historical_variational_multibody_capture_hash_and_cpu_metadata"
        ),
        "visual_boundary": True,
    },
    "avbd-paper-scale-high-ratio-iteration-sweep-packet.json": {
        "supported_scope": ("historical_variational_multibody_cpu_metadata_and_plot"),
        "visual_boundary": False,
    },
}

# Packets committed before the resolved-solver-identity contract
# (WP-091.1). They remain readable at schema_version 1; their
# sequential-impulse contact rows are relabeled in prose instead of
# being rewritten. Do not add new packets here: new packet files must use the
# current AVBD_PACKET_SCHEMA_VERSION with a recorded identity.
LEGACY_IDENTITY_EXEMPT_PACKETS = frozenset(
    {
        "avbd-articulated-breakable-joint-packet.json",
        "avbd-articulated-breakable-motor-packet.json",
        "avbd-articulated-fixed-pair-breakable-joint-packet.json",
        "avbd-articulated-high-ratio-chain-packet.json",
        "avbd-articulated-prismatic-motor-packet.json",
        "avbd-articulated-prismatic-pair-breakable-motor-packet.json",
        "avbd-articulated-revolute-motor-packet.json",
        "avbd-articulated-spherical-breakable-joint-packet.json",
        "avbd-articulated-spherical-pair-breakable-joint-packet.json",
        "avbd-articulated-world-prismatic-breakable-motor-packet.json",
        "avbd-articulated-world-revolute-breakable-motor-packet.json",
        "avbd-demo2d-cards-packet.json",
        "avbd-demo2d-dynamic-friction-packet.json",
        "avbd-demo2d-fracture-packet.json",
        "avbd-demo2d-ground-packet.json",
        "avbd-demo2d-hanging-rope-packet.json",
        "avbd-demo2d-heavy-rope-packet.json",
        "avbd-demo2d-joint-grid-packet.json",
        "avbd-demo2d-motor-packet.json",
        "avbd-demo2d-net-packet.json",
        "avbd-demo2d-pyramid-packet.json",
        "avbd-demo2d-rod-packet.json",
        "avbd-demo2d-rope-packet.json",
        "avbd-demo2d-soft-body-packet.json",
        "avbd-demo2d-spring-packet.json",
        "avbd-demo2d-spring-ratio-packet.json",
        "avbd-demo2d-stack-packet.json",
        "avbd-demo2d-stack-ratio-packet.json",
        "avbd-demo2d-static-friction-packet.json",
        "avbd-demo3d-breakable-packet.json",
        "avbd-demo3d-bridge-packet.json",
        "avbd-demo3d-dynamic-friction-packet.json",
        "avbd-demo3d-ground-packet.json",
        "avbd-demo3d-heavy-rope-packet.json",
        "avbd-demo3d-pyramid-packet.json",
        "avbd-demo3d-rope-packet.json",
        "avbd-demo3d-soft-body-packet.json",
        "avbd-demo3d-spring-packet.json",
        "avbd-demo3d-spring-ratio-packet.json",
        "avbd-demo3d-stack-packet.json",
        "avbd-demo3d-stack-ratio-packet.json",
        "avbd-demo3d-static-friction-packet.json",
        "avbd-empty-baseline-packet.json",
        "avbd-paper-scale-high-ratio-chain-packet.json",
        "avbd-rigid-breakable-joint-packet.json",
        "avbd-rigid-prismatic-motor-packet.json",
        "avbd-rigid-revolute-motor-packet.json",
        "avbd-rigid-spherical-breakable-joint-packet.json",
    }
)

# These schema-version 3 packets were committed before schema version 4 moved
# public hard pair rows from the private AVBD compatibility projection into the
# Sequential Impulse family. Their writers now emit the current schema when
# regenerated. Keep only these exact historical names readable; a new filename
# must use AVBD_PACKET_SCHEMA_VERSION so it cannot claim the retired v3 solver
# identity contract. A filename leaves this set the moment its packet is
# regenerated at the current schema (the breakable scale packets left at
# schema 6), so a later downgrade of current evidence is rejected outright.
LEGACY_PRE_SI_PAIR_ROW_PACKETS = frozenset(
    {
        "avbd-articulated-compliant-fracture-packet.json",
        "avbd-articulated-compliant-joints-packet.json",
        "avbd-articulated-compliant-motors-packet.json",
        "avbd-friction-coefficient-sweep-packet.json",
        "avbd-paper-scale-high-ratio-iteration-sweep-packet.json",
    }
)

# The schema-version 5 Figure 13 packets predate the solver-configuration
# fingerprint, multibody identity, and numeric Table 2 binding introduced by
# schema version 6. All three were regenerated at schema 6, so no committed
# packet remains at version 5 and that contract is retired: a Figure 13
# filename at schema 5 is rejected like any other stale version.

# Pin each historical filename to the one legacy version it was committed
# with. A legacy filename may move directly to the current schema when its
# packet is regenerated; once it has, it leaves this map so the current
# evidence cannot be downgraded back to a readable historical contract.
LEGACY_PACKET_SCHEMA_VERSIONS = {
    **{name: 1 for name in LEGACY_IDENTITY_EXEMPT_PACKETS},
    **{name: 3 for name in LEGACY_PRE_SI_PAIR_ROW_PACKETS},
}
LEGACY_SCHEMA_EXEMPT_PACKETS = frozenset(LEGACY_PACKET_SCHEMA_VERSIONS)

# Identity-free schema-v1 packets and the five schema-v3 packets whose
# historical labels did not identify the runtime solver remain readable only
# as explicitly bounded historical artifacts. The boundary lives in the
# packet, not just this allowlist, so downstream readers cannot accidentally
# promote a bare legacy skeleton into current AVBD evidence.
LEGACY_NON_EVIDENCE_BOUNDARY_SCOPES = {
    **{
        name: "historical_artifact_or_topology_metadata_only"
        for name in LEGACY_IDENTITY_EXEMPT_PACKETS
    },
    "avbd-articulated-high-ratio-chain-packet.json": (
        "historical_variational_multibody_capture_hash_and_cpu_metadata"
    ),
    "avbd-paper-scale-high-ratio-chain-packet.json": (
        "historical_variational_multibody_capture_hash_and_cpu_metadata"
    ),
    "avbd-articulated-compliant-fracture-packet.json": (
        "historical_variational_multibody_artifact_and_cpu_metadata_only"
    ),
    "avbd-articulated-compliant-joints-packet.json": (
        "historical_variational_multibody_artifact_and_cpu_metadata_only"
    ),
    "avbd-articulated-compliant-motors-packet.json": (
        "historical_variational_multibody_artifact_and_cpu_metadata_only"
    ),
    "avbd-friction-coefficient-sweep-packet.json": (
        "historical_sequential_impulse_cpu_and_visual_metadata_only"
    ),
    "avbd-paper-scale-high-ratio-iteration-sweep-packet.json": (
        "historical_variational_multibody_cpu_metadata_and_plot"
    ),
}


@dataclass
class PacketLoadResult:
    packet: dict[str, object] | None
    errors: tuple[str, ...]
    payload: bytes | None
    sha256: str | None


@dataclass
class PacketValidationContext:
    """Shared parse and validation state for one AVBD packet batch."""

    packet_dir: Path = field(default_factory=lambda: DEFAULT_PACKET_DIR.resolve())
    in_progress: set[Path] = field(default_factory=set)
    loaded: dict[Path, PacketLoadResult] = field(default_factory=dict)
    artifact_errors: dict[Path, tuple[str, ...]] = field(default_factory=dict)
    initialization_error: str | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        try:
            self.packet_dir = self.packet_dir.resolve()
        except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
            self.initialization_error = f"packet directory cannot be resolved ({exc})"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--packet",
        action="append",
        type=Path,
        default=None,
        help="Explicit packet file to validate (repeatable); defaults to "
        "every avbd-*-packet.json under --packet-dir.",
    )
    parser.add_argument(
        "--packet-dir",
        type=Path,
        default=DEFAULT_PACKET_DIR,
        help="Directory scanned for avbd-*-packet.json files.",
    )
    parser.add_argument(
        "--stale-source",
        choices=("error", "report"),
        default="error",
        help="How to treat sealed evidence whose recorded source state no "
        "longer matches the working tree. 'error' (default) fails closed, "
        "which is the bar for any parity or performance claim; 'report' "
        "prints the stale seals as advisories so repository-wide lint does "
        "not fail on unrelated source or dependency changes. Structural "
        "and hash defects inside a packet always fail.",
    )
    return parser.parse_args(argv)


# Findings that only say the sealed evidence predates the current working tree.
# They never describe a corrupt or self-inconsistent packet.
STALE_SOURCE_PATTERNS = (
    re.compile(
        r"\.source_provenance\.(digest|file_count|ignored_paths|roots) does not "
        r"match current source state$"
    ),
    re.compile(r"benchmark source capture digest does not match current source state$"),
    re.compile(
        r"benchmark source hash does not match current benchmark translation unit$"
    ),
    re.compile(r"source_provenance\.files\[\d+\]\.sha256 drifted for "),
    re.compile(r"source_provenance\.files paths must exactly match "),
)
# The packet-level digest is recomputed from the listed files, so it is a stale
# seal only when a listed file also drifted; on its own it is a mutated packet.
PACKET_DIGEST_PATTERN = re.compile(
    r": source_provenance\.digest does not match current listed source contents$"
)
PACKET_FILE_DRIFT_PATTERN = re.compile(
    r"source_provenance\.files(\[\d+\]\.sha256 drifted for | paths must exactly match )"
)


def split_stale_source_findings(errors: list[str]) -> tuple[list[str], list[str]]:
    """Split validator output into hard errors and stale-seal advisories."""
    drifted_packets = {
        error.split(": ", 1)[0]
        for error in errors
        if PACKET_FILE_DRIFT_PATTERN.search(error)
    }
    hard: list[str] = []
    stale: list[str] = []
    for error in errors:
        packet_name = error.split(": ", 1)[0]
        if any(pattern.search(error) for pattern in STALE_SOURCE_PATTERNS):
            stale.append(error)
        elif PACKET_DIGEST_PATTERN.search(error) and packet_name in drifted_packets:
            stale.append(error)
        else:
            hard.append(error)
    return hard, stale


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_relative_path(value: object) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    if "\x00" in value:
        return None
    try:
        value.encode("utf-8")
        path = Path(value)
    except OSError, RuntimeError, TypeError, ValueError, UnicodeError:
        return None
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value:
        return None
    return path


def _reject_duplicate_json_keys(
    pairs: list[tuple[str, object]],
) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate object key {key!r}")
        result[key] = value
    return result


def _reject_nonstandard_json_constant(value: str) -> None:
    raise ValueError(f"non-standard numeric constant {value!r}")


def _parse_finite_json_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite JSON number {value!r}")
    return parsed


def _strict_json_loads(payload: bytes) -> object:
    text = payload.decode("utf-8")
    return json.loads(
        text,
        object_pairs_hook=_reject_duplicate_json_keys,
        parse_constant=_reject_nonstandard_json_constant,
        parse_float=_parse_finite_json_float,
    )


def _source_provenance_errors(
    packet: Mapping[str, object], packet_name: str
) -> list[str]:
    provenance = packet.get("source_provenance")
    if provenance is None:
        if packet_name in PAPER_CAPTURE_ROLES:
            return [
                f"{packet_name}: source_provenance must be an object for a "
                "current paper packet"
            ]
        # Every packet written at the current schema contract must name the
        # source it was produced from, and so must any packet that closes a
        # PLAN-104 row, whatever its version. Only packets still validated at
        # their pinned legacy version keep their allowlisted provenance-free
        # shape, because they predate the contract.
        version = packet.get("schema_version")
        current_schema = (
            isinstance(version, int)
            and not isinstance(version, bool)
            and version >= AVBD_PACKET_SCHEMA_VERSION
        )
        if current_schema:
            return [
                f"{packet_name}: source_provenance must be an object for a "
                f"packet written at schema_version {AVBD_PACKET_SCHEMA_VERSION}"
            ]
        if PLAN104_CLAIMS_KEY in packet:
            return [
                f"{packet_name}: source_provenance must be an object for a "
                f"packet that records {PLAN104_CLAIMS_KEY}"
            ]
        return []
    if not isinstance(provenance, Mapping):
        return [f"{packet_name}: source_provenance must be an object"]

    errors: list[str] = []
    if provenance.get("algorithm") != SOURCE_PROVENANCE_ALGORITHM:
        errors.append(
            f"{packet_name}: source_provenance.algorithm must be "
            f"{SOURCE_PROVENANCE_ALGORITHM!r}"
        )
    files = provenance.get("files")
    if not isinstance(files, list) or not files:
        return errors + [
            f"{packet_name}: source_provenance.files must be a non-empty list"
        ]

    required_paths = PAPER_PACKET_SOURCE_PATHS.get(packet_name)
    actual_paths = [
        entry.get("path") if isinstance(entry, Mapping) else None for entry in files
    ]
    if required_paths is not None and actual_paths != list(required_paths):
        errors.append(
            f"{packet_name}: source_provenance.files paths must exactly match "
            "the canonical ordered paper-packet source contract"
        )

    combined = hashlib.sha256()
    seen: set[str] = set()
    try:
        repository_root = REPO_ROOT.resolve()
    except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
        return errors + [f"{packet_name}: repository root cannot be resolved ({exc})"]
    for index, entry in enumerate(files):
        label = f"{packet_name}: source_provenance.files[{index}]"
        if not isinstance(entry, Mapping):
            errors.append(f"{label} must be an object")
            continue
        relative = _safe_relative_path(entry.get("path"))
        if relative is None:
            errors.append(f"{label}.path must be a safe repository-relative path")
            continue
        relative_text = relative.as_posix()
        if relative_text in seen:
            errors.append(f"{label}.path duplicates {relative_text!r}")
            continue
        seen.add(relative_text)
        try:
            lexical_source_path = REPO_ROOT / relative
            if lexical_source_path.is_symlink():
                errors.append(
                    f"{label}.path cannot be a symbolic link: {relative_text}"
                )
                continue
            source_path = lexical_source_path.resolve()
        except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
            errors.append(f"{label}.path cannot be resolved: {relative_text} ({exc})")
            continue
        try:
            source_path.relative_to(repository_root)
        except ValueError:
            errors.append(
                f"{label}.path resolves outside the repository: {relative_text}"
            )
            continue
        try:
            is_file = source_path.is_file()
        except (OSError, ValueError, UnicodeError) as exc:
            errors.append(f"{label}.path cannot be inspected: {relative_text} ({exc})")
            continue
        if not is_file:
            errors.append(f"{label}.path must be a regular file: {relative_text}")
            continue
        try:
            payload = source_path.read_bytes()
        except (OSError, ValueError, UnicodeError) as exc:
            errors.append(f"{label}.path cannot be read: {relative_text} ({exc})")
            continue

        current_hash = hashlib.sha256(payload).hexdigest()
        if entry.get("sha256") != current_hash:
            errors.append(
                f"{label}.sha256 drifted for {relative_text}: expected "
                f"{current_hash}"
            )
        try:
            encoded_path = relative_text.encode("utf-8")
        except UnicodeError as exc:
            errors.append(f"{label}.path cannot be encoded as UTF-8 ({exc})")
            continue
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)

    if provenance.get("digest") != combined.hexdigest():
        errors.append(
            f"{packet_name}: source_provenance.digest does not match current "
            "listed source contents"
        )
    return errors


def _paper_capture_source_binding_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    roles = _paper_capture_roles(packet, packet_name)
    if roles is None:
        return []

    try:
        current = compute_capture_source_provenance(REPO_ROOT)
    except (
        CaptureSourceProvenanceError,
        OSError,
        RuntimeError,
        ValueError,
        UnicodeError,
    ) as exc:
        return [
            f"{packet_name}: cannot resolve current capture source provenance: "
            f"{exc}"
        ]

    errors: list[str] = []
    capture_build_identities: set[tuple[object, object]] = set()
    visual = packet.get("visual_evidence")
    if not isinstance(visual, Mapping):
        return [f"{packet_name}: visual_evidence must be an object"]
    for role in roles:
        capture = visual.get(role)
        label = f"{packet_name}: visual_evidence.{role}.source_provenance"
        if not isinstance(capture, Mapping):
            errors.append(f"{packet_name}: visual_evidence.{role} must be an object")
            continue
        provenance = capture.get("source_provenance")
        if not isinstance(provenance, Mapping):
            errors.append(f"{label} must be an object")
            continue
        if provenance.get("algorithm") != CAPTURE_SOURCE_PROVENANCE_ALGORITHM:
            errors.append(
                f"{label}.algorithm must be " f"{CAPTURE_SOURCE_PROVENANCE_ALGORITHM!r}"
            )
        for key in ("digest", "file_count", "ignored_paths", "roots"):
            if provenance.get(key) != current[key]:
                errors.append(f"{label}.{key} does not match current source state")
        # A recorded Git HEAD describes the evidence only when nothing in the
        # attested scopes was modified and no ignored file inside a capture root
        # escaped the digest. Both are recorded by the producer; a row-closing
        # packet must show both clean.
        if provenance.get("working_tree_clean") is not True:
            errors.append(
                f"{label}.working_tree_clean must be true; a dirty capture "
                "source tree makes the recorded Git HEAD unverifiable"
            )
        if provenance.get("ignored_paths") != []:
            errors.append(
                f"{label}.ignored_paths must be empty; an ignored file inside a "
                "capture root is unhashed capture input"
            )
        git_head = provenance.get("git_head")
        if (
            not isinstance(git_head, str)
            or len(git_head) != 40
            or any(character not in "0123456789abcdef" for character in git_head)
        ):
            errors.append(f"{label}.git_head must be a lowercase Git object ID")
        capture_build_identities.add((provenance.get("digest"), git_head))
        errors.extend(
            _paper_capture_runtime_errors(
                capture,
                expected_source_digest=provenance.get("digest"),
                expected_source_git_head=provenance.get("git_head"),
                label=f"{packet_name}: visual_evidence.{role}",
            )
        )

    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, Mapping):
        return errors + [f"{packet_name}: benchmark must be an object"]
    benchmark_provenance = benchmark.get("source_provenance")
    if not isinstance(benchmark_provenance, Mapping):
        errors.append(f"{packet_name}: benchmark.source_provenance must be an object")
        return errors
    expected_provenance_keys = {
        "algorithm",
        "benchmark_source_sha256",
        "build_identity",
        "capture_source_git_head",
        "capture_source_provenance_digest",
        "digest",
        "executable",
        "loaded_dart_libraries",
        "runtime_image_inventory",
    }
    if set(benchmark_provenance) != expected_provenance_keys:
        errors.append(
            f"{packet_name}: benchmark.source_provenance must contain exactly "
            f"{sorted(expected_provenance_keys)!r}"
        )
    if benchmark_provenance.get("algorithm") != BENCHMARK_SOURCE_PROVENANCE_ALGORITHM:
        errors.append(
            f"{packet_name}: benchmark.source_provenance.algorithm must be "
            f"{BENCHMARK_SOURCE_PROVENANCE_ALGORITHM!r}"
        )
    if (
        benchmark_provenance.get("capture_source_provenance_digest")
        != current["digest"]
    ):
        errors.append(
            f"{packet_name}: benchmark source capture digest does not match "
            "current source state"
        )
    try:
        benchmark_source_hash = _sha256(REPO_ROOT / BENCHMARK_SOURCE_PATH)
    except (OSError, ValueError, UnicodeError) as exc:
        errors.append(
            f"{packet_name}: benchmark source file cannot be read: "
            f"{BENCHMARK_SOURCE_PATH.as_posix()} ({exc})"
        )
        return errors
    if benchmark_provenance.get("benchmark_source_sha256") != benchmark_source_hash:
        errors.append(
            f"{packet_name}: benchmark source hash does not match current "
            "benchmark translation unit"
        )

    capture_git_head = benchmark_provenance.get("capture_source_git_head")
    if (
        not isinstance(capture_git_head, str)
        or len(capture_git_head) != 40
        or any(character not in "0123456789abcdef" for character in capture_git_head)
    ):
        errors.append(
            f"{packet_name}: benchmark source Git HEAD must be a lowercase object ID"
        )
    expected_build_identity = (
        benchmark_provenance.get("capture_source_provenance_digest"),
        capture_git_head,
    )
    if capture_build_identities != {expected_build_identity}:
        errors.append(
            f"{packet_name}: every capture and benchmark must share one compiled "
            "source digest and Git HEAD"
        )
    executable = benchmark_provenance.get("executable")
    executable_errors = _paper_benchmark_executable_errors(
        executable,
        f"{packet_name}: benchmark.source_provenance.executable",
    )
    errors.extend(executable_errors)
    loaded_dart_libraries = benchmark_provenance.get("loaded_dart_libraries")
    errors.extend(
        _paper_benchmark_library_inventory_errors(
            loaded_dart_libraries,
            f"{packet_name}: benchmark.source_provenance.loaded_dart_libraries",
        )
    )
    runtime_image_inventory = benchmark_provenance.get("runtime_image_inventory")
    errors.extend(
        _paper_runtime_image_inventory_errors(
            runtime_image_inventory,
            executable=executable,
            label=(
                f"{packet_name}: " "benchmark.source_provenance.runtime_image_inventory"
            ),
        )
    )
    if isinstance(runtime_image_inventory, Mapping):
        runtime_images = runtime_image_inventory.get("images")
        runtime_by_path = (
            {
                image.get("path"): image
                for image in runtime_images
                if isinstance(image, Mapping)
            }
            if isinstance(runtime_images, list)
            else {}
        )
        if isinstance(loaded_dart_libraries, list):
            for index, library in enumerate(loaded_dart_libraries):
                if not isinstance(library, Mapping):
                    continue
                base_library = {
                    key: library.get(key)
                    for key in ("file", "path", "sha256", "size_bytes")
                }
                if runtime_by_path.get(library.get("path")) != base_library:
                    errors.append(
                        f"{packet_name}: benchmark loaded DART library {index} "
                        "must match runtime image inventory"
                    )
            declared_dart_paths = sorted(
                library.get("path")
                for library in loaded_dart_libraries
                if isinstance(library, Mapping) and isinstance(library.get("path"), str)
            )
            runtime_dart_paths = sorted(
                image.get("path")
                for image in runtime_by_path.values()
                if isinstance(image.get("file"), str)
                and image["file"].lower().startswith("libdart")
                and isinstance(image.get("path"), str)
            )
            if declared_dart_paths != runtime_dart_paths:
                errors.append(
                    f"{packet_name}: benchmark loaded DART libraries must "
                    "exactly cover every mapped libdart runtime image"
                )
    build_identity = benchmark_provenance.get("build_identity")
    errors.extend(
        _paper_benchmark_build_identity_errors(
            build_identity,
            benchmark_provenance=benchmark_provenance,
            label=f"{packet_name}: benchmark.source_provenance.build_identity",
        )
    )
    source_payload = {
        key: benchmark_provenance.get(key)
        for key in (
            "benchmark_source_sha256",
            "build_identity",
            "capture_source_git_head",
            "capture_source_provenance_digest",
            "executable",
            "loaded_dart_libraries",
            "runtime_image_inventory",
        )
    }
    if benchmark_provenance.get("digest") != _canonical_json_digest(source_payload):
        errors.append(
            f"{packet_name}: benchmark.source_provenance.digest must bind its "
            "source, build, and executable identity"
        )

    context = benchmark.get("context")
    if not isinstance(context, Mapping):
        errors.append(f"{packet_name}: benchmark.context must be an object")
        return errors
    context_bindings = {
        "dart_benchmark_source_sha256": "benchmark_source_sha256",
        "dart_capture_source_git_head": "capture_source_git_head",
        "dart_capture_source_provenance_digest": ("capture_source_provenance_digest"),
    }
    for context_key, provenance_key in context_bindings.items():
        if context.get(context_key) != benchmark_provenance.get(provenance_key):
            errors.append(
                f"{packet_name}: benchmark.context.{context_key} does not match "
                "benchmark.source_provenance"
            )
    expected_build_context = {
        "dart_cmake_build_type": "Release",
        "dart_ndebug": "1",
        "dart_optimization_enabled": "1",
    }
    for key, expected in expected_build_context.items():
        if context.get(key) != expected:
            errors.append(
                f"{packet_name}: benchmark.context.{key} must be {expected!r}"
            )
    for key in ("dart_compiler_id", "dart_compiler_version"):
        if not isinstance(context.get(key), str) or not context[key]:
            errors.append(f"{packet_name}: benchmark.context.{key} must be non-empty")
    if isinstance(build_identity, Mapping):
        build_configuration = build_identity.get("build_configuration")
        build_configuration_digest = (
            build_configuration.get("digest")
            if isinstance(build_configuration, Mapping)
            else None
        )
        if (
            not _is_lowercase_sha256(context.get("dart_build_configuration_digest"))
            or context.get("dart_build_configuration_digest")
            != build_configuration_digest
        ):
            errors.append(
                f"{packet_name}: benchmark.context."
                "dart_build_configuration_digest must match compiled build identity"
            )
        for context_key, build_key in (
            ("dart_cmake_build_type", "cmake_build_type"),
            ("dart_compiler_id", "compiler_id"),
            ("dart_compiler_version", "compiler_version"),
            ("dart_ndebug", "ndebug"),
            ("dart_optimization_enabled", "optimization_enabled"),
        ):
            if context.get(context_key) != build_identity.get(build_key):
                errors.append(
                    f"{packet_name}: benchmark.context.{context_key} must match "
                    "compiled build identity"
                )
    if isinstance(executable, Mapping):
        context_executable = context.get("dart_benchmark_executable_path")
        if context_executable != executable.get("path"):
            errors.append(
                f"{packet_name}: benchmark context executable path must match "
                "benchmark.source_provenance"
            )
        standard_executable = context.get("executable")
        if not isinstance(standard_executable, str) or Path(
            standard_executable
        ).name != executable.get("file"):
            errors.append(
                f"{packet_name}: benchmark context standard executable must "
                "identify the bound binary"
            )
    errors.extend(
        _paper_benchmark_run_evidence_errors(
            benchmark,
            context=context,
            build_identity=build_identity,
            packet_name=packet_name,
        )
    )
    return errors


def _is_lowercase_sha256(value: object) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _canonical_json_digest(value: object) -> str | None:
    try:
        encoded = json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except TypeError, ValueError, UnicodeError:
        return None
    return hashlib.sha256(encoded).hexdigest()


def _paper_benchmark_executable_errors(value: object, label: str) -> list[str]:
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    expected_keys = {"file", "path", "sha256", "size_bytes"}
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    file_value = value.get("file")
    path_value = value.get("path")
    if file_value != "bm_avbd_rigid_fixed_joint":
        errors.append(f"{label}.file must identify bm_avbd_rigid_fixed_joint")
    if (
        not isinstance(path_value, str)
        or not path_value
        or not Path(path_value).is_absolute()
        or Path(path_value).name != file_value
    ):
        errors.append(f"{label}.path must be an absolute path to the named executable")
    if not _is_lowercase_sha256(value.get("sha256")):
        errors.append(f"{label}.sha256 must be a lowercase SHA-256 digest")
    size = value.get("size_bytes")
    if (
        not isinstance(size, int)
        or isinstance(size, bool)
        or not 1 <= size <= 0xFFFFFFFFFFFFFFFF
    ):
        errors.append(f"{label}.size_bytes must be a positive uint64")
    return errors


def _paper_runtime_image_inventory_errors(
    value: object,
    *,
    executable: object,
    label: str,
) -> list[str]:
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    expected_keys = {"algorithm", "digest", "images", "required_roles"}
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    if value.get("algorithm") != RUNTIME_IMAGE_INVENTORY_ALGORITHM:
        errors.append(
            f"{label}.algorithm must be {RUNTIME_IMAGE_INVENTORY_ALGORITHM!r}"
        )
    images = value.get("images")
    if not isinstance(images, list) or not images:
        errors.append(f"{label}.images must be a non-empty list")
        images = []
    paths: list[str] = []
    role_matches: dict[str, list[str]] = {
        role: [] for role in REQUIRED_RUNTIME_IMAGE_ROLES
    }
    for index, image in enumerate(images):
        image_label = f"{label}.images[{index}]"
        if not isinstance(image, Mapping):
            errors.append(f"{image_label} must be an object")
            continue
        image_keys = {"file", "path", "sha256", "size_bytes"}
        if set(image) != image_keys:
            errors.append(f"{image_label} must contain exactly {sorted(image_keys)!r}")
        file_value = image.get("file")
        path_value = image.get("path")
        if not isinstance(file_value, str) or not file_value:
            errors.append(f"{image_label}.file must be non-empty")
        if (
            not isinstance(path_value, str)
            or not Path(path_value).is_absolute()
            or Path(path_value).name != file_value
        ):
            errors.append(f"{image_label}.path must be absolute and match its file")
            continue
        paths.append(path_value)
        name = file_value.lower()
        if name.startswith(("ld-linux", "ld-musl")) or name == "ld.so":
            role_matches["dynamic_loader"].append(path_value)
        if name == "libbenchmark.so" or name.startswith("libbenchmark.so."):
            role_matches["google_benchmark"].append(path_value)
        if name == "libc.so" or name.startswith("libc.so."):
            role_matches["libc"].append(path_value)
        if name == "libm.so" or name.startswith("libm.so."):
            role_matches["libm"].append(path_value)
        if name == "libstdc++.so" or name.startswith("libstdc++.so."):
            role_matches["libstdcxx"].append(path_value)
        if not _is_lowercase_sha256(image.get("sha256")):
            errors.append(f"{image_label}.sha256 must be a lowercase SHA-256 digest")
        size = image.get("size_bytes")
        if (
            not isinstance(size, int)
            or isinstance(size, bool)
            or not 1 <= size <= 0xFFFFFFFFFFFFFFFF
        ):
            errors.append(f"{image_label}.size_bytes must be a positive uint64")
    if paths != sorted(set(paths)) or len(paths) != len(images):
        errors.append(f"{label}.images must be unique and path-sorted")
    roles = value.get("required_roles")
    if not isinstance(roles, Mapping) or set(roles) != set(
        REQUIRED_RUNTIME_IMAGE_ROLES
    ):
        errors.append(f"{label}.required_roles must contain exactly the required roles")
    else:
        for role in REQUIRED_RUNTIME_IMAGE_ROLES:
            matches = role_matches[role]
            if len(matches) != 1:
                errors.append(f"{label}.images must contain exactly one {role} image")
            elif roles.get(role) != matches[0]:
                errors.append(
                    f"{label}.required_roles.{role} must bind its mapped image"
                )
    executable_path = (
        executable.get("path") if isinstance(executable, Mapping) else None
    )
    if paths.count(executable_path) != 1:
        errors.append(f"{label}.images must contain the exact executable")
    payload = {
        "images": value.get("images"),
        "required_roles": value.get("required_roles"),
    }
    if value.get("digest") != _canonical_json_digest(payload):
        errors.append(f"{label}.digest must bind every mapped ELF image")
    return errors


def _paper_benchmark_library_inventory_errors(value: object, label: str) -> list[str]:
    if not isinstance(value, list) or not value:
        return [f"{label} must be a non-empty list"]
    errors: list[str] = []
    paths: list[str] = []
    for index, entry in enumerate(value):
        entry_label = f"{label}[{index}]"
        if not isinstance(entry, Mapping):
            errors.append(f"{entry_label} must be an object")
            continue
        expected_keys = {"build_identity", "file", "path", "sha256", "size_bytes"}
        if set(entry) != expected_keys:
            errors.append(
                f"{entry_label} must contain exactly {sorted(expected_keys)!r}"
            )
        file_value = entry.get("file")
        path_value = entry.get("path")
        if not isinstance(file_value, str) or not file_value.lower().startswith(
            "libdart"
        ):
            errors.append(f"{entry_label}.file must identify a libdart image")
        if (
            not isinstance(path_value, str)
            or not Path(path_value).is_absolute()
            or Path(path_value).name != file_value
        ):
            errors.append(
                f"{entry_label}.path must be an absolute path to the named library"
            )
        else:
            paths.append(path_value)
        if not _is_lowercase_sha256(entry.get("sha256")):
            errors.append(f"{entry_label}.sha256 must be a lowercase SHA-256 digest")
        size = entry.get("size_bytes")
        if (
            not isinstance(size, int)
            or isinstance(size, bool)
            or not 1 <= size <= 0xFFFFFFFFFFFFFFFF
        ):
            errors.append(f"{entry_label}.size_bytes must be a positive uint64")
        errors.extend(
            _paper_dart_library_build_identity_errors(
                entry.get("build_identity"), f"{entry_label}.build_identity"
            )
        )
    if paths != sorted(set(paths)) or len(paths) != len(value):
        errors.append(f"{label} must be unique and path-sorted")
    return errors


def _paper_dart_library_build_identity_errors(
    value: object,
    label: str,
    *,
    expected_source_digest: object | None = None,
    expected_source_git_head: object | None = None,
) -> list[str]:
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    payload_keys = {
        "build_configuration_digest",
        "build_target",
        "cmake_build_type",
        "compiler_id",
        "compiler_version",
        "ndebug",
        "optimization_enabled",
        "source_git_head",
        "source_provenance_digest",
    }
    expected_keys = payload_keys | {"algorithm", "digest"}
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    if value.get("algorithm") != DART_LIBRARY_BUILD_IDENTITY_ALGORITHM:
        errors.append(
            f"{label}.algorithm must be {DART_LIBRARY_BUILD_IDENTITY_ALGORITHM!r}"
        )
    for key in ("build_target", "compiler_id", "compiler_version"):
        if not isinstance(value.get(key), str) or not value[key]:
            errors.append(f"{label}.{key} must be non-empty")
    if value.get("cmake_build_type") != "Release":
        errors.append(f"{label}.cmake_build_type must be 'Release'")
    for key in ("ndebug", "optimization_enabled"):
        if value.get(key) is not True:
            errors.append(f"{label}.{key} must be true")
    if not _is_lowercase_sha256(value.get("build_configuration_digest")):
        errors.append(f"{label}.build_configuration_digest must be SHA-256")
    if not _is_lowercase_sha256(value.get("source_provenance_digest")):
        errors.append(f"{label}.source_provenance_digest must be SHA-256")
    source_head = value.get("source_git_head")
    if (
        not isinstance(source_head, str)
        or len(source_head) != 40
        or any(character not in "0123456789abcdef" for character in source_head)
    ):
        errors.append(f"{label}.source_git_head must be a Git object ID")
    if (
        expected_source_digest is not None
        and value.get("source_provenance_digest") != expected_source_digest
    ):
        errors.append(f"{label}.source_provenance_digest must match source")
    if (
        expected_source_git_head is not None
        and value.get("source_git_head") != expected_source_git_head
    ):
        errors.append(f"{label}.source_git_head must match source")
    payload = {key: value.get(key) for key in payload_keys}
    if value.get("digest") != _canonical_json_digest(payload):
        errors.append(f"{label}.digest must bind the library build identity")
    return errors


def _paper_build_configuration_errors(value: object, label: str) -> list[str]:
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    expected_keys = {"algorithm", "digest", "values"}
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    if value.get("algorithm") != BUILD_CONFIGURATION_ALGORITHM:
        errors.append(f"{label}.algorithm must be {BUILD_CONFIGURATION_ALGORITHM!r}")
    values = value.get("values")
    if not isinstance(values, Mapping) or set(values) != set(BUILD_CONFIGURATION_KEYS):
        errors.append(f"{label}.values must contain exactly the canonical build keys")
        values = {}
    elif any(not isinstance(item, str) for item in values.values()):
        errors.append(f"{label}.values must contain only strings")
    record = "".join(
        [f"algorithm={BUILD_CONFIGURATION_ALGORITHM}\n"]
        + [f"{key}={values.get(key)}\n" for key in BUILD_CONFIGURATION_KEYS]
    )
    if (
        not _is_lowercase_sha256(value.get("digest"))
        or value.get("digest") != hashlib.sha256(record.encode("utf-8")).hexdigest()
    ):
        errors.append(f"{label}.digest must bind the canonical build record")
    for definition in EVIDENCE_CMAKE_DEFINITIONS:
        name, expected = definition.split("=", maxsplit=1)
        if name not in BUILD_CONFIGURATION_KEYS:
            continue
        if not evidence_definition_matches(name, expected, values.get(name)):
            errors.append(f"{label}.values.{name} must be {expected!r}")
    if values.get("CMAKE_GENERATOR") != "Ninja":
        errors.append(f"{label}.values.CMAKE_GENERATOR must be 'Ninja'")
    return errors


def _paper_benchmark_build_identity_errors(
    value: object,
    *,
    benchmark_provenance: Mapping[str, object],
    label: str,
) -> list[str]:
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    payload_keys = {
        "benchmark_source_sha256",
        "build_configuration",
        "capture_source_git_head",
        "capture_source_provenance_digest",
        "cmake_build_type",
        "compiler_id",
        "compiler_version",
        "executable_file",
        "executable_path",
        "executable_sha256",
        "executable_size_bytes",
        "loaded_dart_libraries",
        "runtime_image_inventory",
        "ndebug",
        "optimization_enabled",
    }
    expected_keys = payload_keys | {"algorithm", "digest"}
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    if value.get("algorithm") != BENCHMARK_BUILD_IDENTITY_ALGORITHM:
        errors.append(
            f"{label}.algorithm must be {BENCHMARK_BUILD_IDENTITY_ALGORITHM!r}"
        )
    build_configuration = value.get("build_configuration")
    errors.extend(
        _paper_build_configuration_errors(
            build_configuration, f"{label}.build_configuration"
        )
    )
    configuration_digest = (
        build_configuration.get("digest")
        if isinstance(build_configuration, Mapping)
        else None
    )
    for build_key, provenance_key in (
        ("benchmark_source_sha256", "benchmark_source_sha256"),
        ("capture_source_git_head", "capture_source_git_head"),
        ("capture_source_provenance_digest", "capture_source_provenance_digest"),
    ):
        if value.get(build_key) != benchmark_provenance.get(provenance_key):
            errors.append(f"{label}.{build_key} must match benchmark provenance")
    executable = benchmark_provenance.get("executable")
    if isinstance(executable, Mapping):
        for build_key, executable_key in (
            ("executable_file", "file"),
            ("executable_path", "path"),
            ("executable_sha256", "sha256"),
            ("executable_size_bytes", "size_bytes"),
        ):
            if value.get(build_key) != executable.get(executable_key):
                errors.append(f"{label}.{build_key} must match executable identity")
    if not _json_values_equal_exact(
        value.get("loaded_dart_libraries"),
        benchmark_provenance.get("loaded_dart_libraries"),
    ):
        errors.append(f"{label}.loaded_dart_libraries must match provenance")
    if not _json_values_equal_exact(
        value.get("runtime_image_inventory"),
        benchmark_provenance.get("runtime_image_inventory"),
    ):
        errors.append(f"{label}.runtime_image_inventory must match provenance")
    libraries = value.get("loaded_dart_libraries")
    runtime_inventory = value.get("runtime_image_inventory")
    if isinstance(libraries, list) and isinstance(runtime_inventory, Mapping):
        runtime_images = runtime_inventory.get("images")
        if isinstance(runtime_images, list):
            declared_dart_paths = sorted(
                library.get("path")
                for library in libraries
                if isinstance(library, Mapping) and isinstance(library.get("path"), str)
            )
            runtime_dart_paths = sorted(
                image.get("path")
                for image in runtime_images
                if isinstance(image, Mapping)
                and isinstance(image.get("file"), str)
                and image["file"].lower().startswith("libdart")
                and isinstance(image.get("path"), str)
            )
            if declared_dart_paths != runtime_dart_paths:
                errors.append(
                    f"{label}.loaded_dart_libraries must exactly cover every "
                    "mapped libdart runtime image"
                )
    if isinstance(libraries, list):
        for index, library in enumerate(libraries):
            if not isinstance(library, Mapping):
                continue
            identity = library.get("build_identity")
            identity_label = f"{label}.loaded_dart_libraries[{index}].build_identity"
            errors.extend(
                _paper_dart_library_build_identity_errors(
                    identity,
                    identity_label,
                    expected_source_digest=value.get(
                        "capture_source_provenance_digest"
                    ),
                    expected_source_git_head=value.get("capture_source_git_head"),
                )
            )
            if isinstance(identity, Mapping):
                for identity_key, build_key, expected in (
                    (
                        "build_configuration_digest",
                        "build_configuration",
                        configuration_digest,
                    ),
                    ("cmake_build_type", "cmake_build_type", None),
                    ("compiler_id", "compiler_id", None),
                    ("compiler_version", "compiler_version", None),
                    ("ndebug", "ndebug", "1"),
                    ("optimization_enabled", "optimization_enabled", "1"),
                ):
                    build_value = value.get(build_key)
                    if identity_key == "build_configuration_digest":
                        build_value = expected
                    elif expected is not None:
                        build_value = build_value == expected
                    if identity.get(identity_key) != build_value:
                        errors.append(
                            f"{identity_label}.{identity_key} must match the "
                            "benchmark build identity"
                        )
    for key in ("benchmark_source_sha256", "capture_source_provenance_digest"):
        if not _is_lowercase_sha256(value.get(key)):
            errors.append(f"{label}.{key} must be a lowercase SHA-256 digest")
    if isinstance(build_configuration, Mapping):
        build_values = build_configuration.get("values")
        if isinstance(build_values, Mapping):
            for build_key, configuration_key in (
                ("cmake_build_type", "CMAKE_BUILD_TYPE"),
                ("compiler_id", "CMAKE_CXX_COMPILER_ID"),
                ("compiler_version", "CMAKE_CXX_COMPILER_VERSION"),
            ):
                if value.get(build_key) != build_values.get(configuration_key):
                    errors.append(f"{label}.{build_key} must match build configuration")
    git_head = value.get("capture_source_git_head")
    if (
        not isinstance(git_head, str)
        or len(git_head) != 40
        or any(character not in "0123456789abcdef" for character in git_head)
    ):
        errors.append(f"{label}.capture_source_git_head must be a Git object ID")
    for key in ("compiler_id", "compiler_version"):
        if not isinstance(value.get(key), str) or not value[key]:
            errors.append(f"{label}.{key} must be non-empty")
    for key, expected in (
        ("cmake_build_type", "Release"),
        ("ndebug", "1"),
        ("optimization_enabled", "1"),
    ):
        if value.get(key) != expected:
            errors.append(f"{label}.{key} must be {expected!r}")
    payload = {key: value.get(key) for key in payload_keys}
    if value.get("digest") != _canonical_json_digest(payload):
        errors.append(f"{label}.digest must bind the compiled build identity")
    return errors


def _paper_load_gate_errors(
    value: object,
    *,
    label: str,
    minimum_duration_seconds: float | None,
) -> list[str]:
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    expected_keys = {
        "elapsed_seconds",
        "finished_at",
        "max_normalized_load",
        "normalized_load_limit",
        "passed",
        "sample_count",
        "sample_interval_seconds",
        "started_at",
    }
    if minimum_duration_seconds is not None:
        expected_keys.add("duration_seconds")
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    if value.get("passed") is not True:
        errors.append(f"{label}.passed must be true")
    for key in ("started_at", "finished_at"):
        timestamp = value.get(key)
        if not isinstance(timestamp, str) or not timestamp.endswith("Z"):
            errors.append(f"{label}.{key} must be a UTC ISO-8601 timestamp")
            continue
        try:
            datetime.fromisoformat(timestamp.removesuffix("Z") + "+00:00")
        except ValueError:
            errors.append(f"{label}.{key} must be a UTC ISO-8601 timestamp")
    interval = _finite_number(value.get("sample_interval_seconds"))
    limit = _finite_number(value.get("normalized_load_limit"))
    maximum = _finite_number(value.get("max_normalized_load"))
    elapsed = _finite_number(value.get("elapsed_seconds"))
    if interval is None or not 0.0 < interval <= 1.0:
        errors.append(f"{label}.sample_interval_seconds must be in (0, 1]")
    if limit is None or not 0.0 < limit <= 0.25:
        errors.append(f"{label}.normalized_load_limit must be in (0, 0.25]")
    if maximum is None or limit is None or not 0.0 <= maximum <= limit:
        errors.append(f"{label}.max_normalized_load must not exceed its limit")
    if elapsed is None or elapsed <= 0.0:
        errors.append(f"{label}.elapsed_seconds must be positive")
    sample_count = value.get("sample_count")
    if (
        not isinstance(sample_count, int)
        or isinstance(sample_count, bool)
        or sample_count < 2
    ):
        errors.append(f"{label}.sample_count must be an integer of at least two")
    if minimum_duration_seconds is not None:
        duration = _finite_number(value.get("duration_seconds"))
        if (
            duration is None
            or duration < minimum_duration_seconds
            or elapsed is None
            or elapsed < duration
        ):
            errors.append(
                f"{label} must cover at least {minimum_duration_seconds:g} seconds"
            )
        elif (
            interval is not None
            and isinstance(sample_count, int)
            and not isinstance(sample_count, bool)
            and sample_count < math.floor(duration / interval)
        ):
            errors.append(f"{label}.sample_count cannot cover the declared duration")
    return errors


def _paper_benchmark_run_evidence_errors(
    benchmark: Mapping[str, object],
    *,
    context: Mapping[str, object],
    build_identity: object,
    packet_name: str,
) -> list[str]:
    label = f"{packet_name}: benchmark.run_evidence"
    evidence = benchmark.get("run_evidence")
    if not isinstance(evidence, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    expected_keys = {
        "benchmark_context_date",
        "benchmark_policy",
        "build_identity",
        "capture_ignored_paths",
        "capture_working_tree_clean",
        "digest",
        "host_identity",
        "host_token",
        "loader_environment",
        "quiet_host",
        "run_token",
        "schema_version",
        "watchdog",
    }
    if set(evidence) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    if evidence.get("schema_version") != FIGURE13_BENCHMARK_RUN_SCHEMA:
        errors.append(
            f"{label}.schema_version must be {FIGURE13_BENCHMARK_RUN_SCHEMA!r}"
        )
    run_payload = {
        key: evidence.get(key) for key in expected_keys - {"digest", "schema_version"}
    }
    if evidence.get("digest") != _canonical_json_digest(run_payload):
        errors.append(f"{label}.digest must bind the complete run evidence")
    expected_policy = {
        "filter": FIGURE13_BENCHMARK_FILTER,
        "min_warmup_time_seconds": 1.0,
        "repetitions": 5,
        "report_aggregates_only": True,
    }
    if not _json_values_equal_exact(evidence.get("benchmark_policy"), expected_policy):
        errors.append(f"{label}.benchmark_policy must match Figure 13 policy")
    if evidence.get("capture_working_tree_clean") is not True:
        errors.append(
            f"{label}.capture_working_tree_clean must be true; a dirty capture "
            "source tree makes the recorded Git HEAD unverifiable"
        )
    if evidence.get("capture_ignored_paths") != []:
        errors.append(
            f"{label}.capture_ignored_paths must be empty; an ignored file inside "
            "a capture root is unhashed benchmark input"
        )
    if evidence.get("benchmark_context_date") != context.get("date") or not isinstance(
        context.get("date"), str
    ):
        errors.append(f"{label}.benchmark_context_date must match benchmark context")
    if not _json_values_equal_exact(evidence.get("build_identity"), build_identity):
        errors.append(f"{label}.build_identity must match compiled provenance")
    expected_loader_environment = {
        "algorithm": LOADER_POLICY_ALGORITHM,
        "forbidden_environment_prefixes": list(LOADER_ENVIRONMENT_PREFIXES),
        "passed": True,
        "present_environment_variables": [],
    }
    if not _json_values_equal_exact(
        evidence.get("loader_environment"), expected_loader_environment
    ):
        errors.append(
            f"{label}.loader_environment must prove an empty loader-control "
            "environment"
        )
    host = evidence.get("host_identity")
    host_token: object = None
    if not isinstance(host, Mapping):
        errors.append(f"{label}.host_identity must be an object")
    else:
        host_keys = {
            "cpu_count",
            "cpu_model",
            "host_token",
            "hostname",
            "machine",
            "platform",
            "system",
        }
        if set(host) != host_keys:
            errors.append(
                f"{label}.host_identity must contain exactly {sorted(host_keys)!r}"
            )
        cpu_count = host.get("cpu_count")
        if (
            not isinstance(cpu_count, int)
            or isinstance(cpu_count, bool)
            or cpu_count < 1
        ):
            errors.append(f"{label}.host_identity.cpu_count must be positive")
        for key in ("cpu_model", "hostname", "machine", "platform", "system"):
            if not isinstance(host.get(key), str) or not host[key]:
                errors.append(f"{label}.host_identity.{key} must be non-empty")
        host_payload = {key: host.get(key) for key in host_keys - {"host_token"}}
        host_token = _canonical_json_digest(host_payload)
        if host.get("host_token") != host_token:
            errors.append(f"{label}.host_identity.host_token must bind host identity")
        if context.get("host_name") != host.get("hostname"):
            errors.append(f"{label}.host_identity.hostname must match context")
    if evidence.get("host_token") != host_token:
        errors.append(f"{label}.host_token must match host_identity")
    run_token = evidence.get("run_token")
    try:
        parsed = uuid.UUID(run_token, version=4)
    except AttributeError, TypeError, ValueError:
        errors.append(f"{label}.run_token must be a canonical UUIDv4")
    else:
        if str(parsed) != run_token:
            errors.append(f"{label}.run_token must be a canonical UUIDv4")
    errors.extend(
        _paper_load_gate_errors(
            evidence.get("quiet_host"),
            label=f"{label}.quiet_host",
            minimum_duration_seconds=120.0,
        )
    )
    errors.extend(
        _paper_load_gate_errors(
            evidence.get("watchdog"),
            label=f"{label}.watchdog",
            minimum_duration_seconds=None,
        )
    )
    quiet = evidence.get("quiet_host")
    watchdog = evidence.get("watchdog")
    if isinstance(quiet, Mapping) and isinstance(watchdog, Mapping):
        for key in ("normalized_load_limit", "sample_interval_seconds"):
            if not _json_values_equal_exact(quiet.get(key), watchdog.get(key)):
                errors.append(f"{label}.watchdog.{key} must match quiet_host")
    return errors


def _iter_image_verdict_containers(
    node: object, path: str
) -> Iterator[tuple[str, Mapping[str, object]]]:
    """Yield every container in *node* that records an ``image_verdict``."""
    if isinstance(node, Mapping):
        if isinstance(node.get("image_verdict"), Mapping):
            yield path, node
        for key, value in node.items():
            yield from _iter_image_verdict_containers(
                value, f"{path}.{key}" if path else str(key)
            )
    elif isinstance(node, list):
        for index, value in enumerate(node):
            yield from _iter_image_verdict_containers(value, f"{path}[{index}]")


def _image_verdict_binding_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    """Bind every recorded image verdict to the screenshot bytes it judged.

    A verdict that records only width and height would accept any image of the
    same size, so every packet that stores one must also store the screenshot
    digest the verdict was rendered from. The Figure 13 roles are validated by
    ``_paper_image_verdict_binding_errors``; this covers every other capture
    that carries a verdict, whatever the packet shape.
    """
    roles = _paper_capture_roles(packet, packet_name)
    covered = (
        {f"visual_evidence.{role}" for role in roles} if roles is not None else set()
    )
    # Pinned legacy packets predate the `image_sha256` verdict field and never
    # recorded the judged image digest. Their own evidence_boundary (validated
    # by `_legacy_non_evidence_boundary_errors`) declares every capture they
    # hold non-evidence, so there is no image claim left to bind here.
    version = packet.get("schema_version")
    if (
        packet_name in LEGACY_NON_EVIDENCE_BOUNDARY_SCOPES
        and isinstance(version, int)
        and version < PLAN104_CLAIMS_MIN_SCHEMA_VERSION
        and isinstance(packet.get("evidence_boundary"), Mapping)
    ):
        return []
    errors: list[str] = []
    for path, container in _iter_image_verdict_containers(packet, ""):
        if path in covered:
            continue
        label = f"{packet_name}: {path}" if path else packet_name
        verdict = container["image_verdict"]
        assert isinstance(verdict, Mapping)
        screenshot = container.get("screenshot")
        screenshot_sha256 = (
            screenshot.get("sha256") if isinstance(screenshot, Mapping) else None
        )
        if not _is_lowercase_sha256(screenshot_sha256):
            errors.append(
                f"{label}.screenshot.sha256 must be a lowercase SHA-256 digest "
                "for a capture that records an image verdict"
            )
        image_sha256 = verdict.get("image_sha256")
        if not _is_lowercase_sha256(image_sha256):
            errors.append(
                f"{label}.image_verdict.image_sha256 must be a lowercase "
                "SHA-256 digest"
            )
        elif (
            _is_lowercase_sha256(screenshot_sha256)
            and image_sha256 != screenshot_sha256
        ):
            errors.append(
                f"{label}.image_verdict.image_sha256 must match "
                f"{label}.screenshot.sha256"
            )
    return errors


def _paper_image_verdict_binding_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    """Validate durable screenshot-to-verdict bindings for Figure 13 packets."""
    roles = _paper_capture_roles(packet, packet_name)
    if roles is None:
        return []

    visual = packet.get("visual_evidence")
    if not isinstance(visual, Mapping):
        return [f"{packet_name}: visual_evidence must be an object"]

    errors: list[str] = []
    capture_specs = PAPER_FIGURE13_SPECS[packet_name]["captures"]
    assert isinstance(capture_specs, Mapping)
    for role in roles:
        label = f"{packet_name}: visual_evidence.{role}"
        capture = visual.get(role)
        if not isinstance(capture, Mapping):
            errors.append(f"{label} must be an object")
            continue
        screenshot = capture.get("screenshot")
        if not isinstance(screenshot, Mapping):
            errors.append(f"{label}.screenshot must be an object")
            continue
        screenshot_sha256 = screenshot.get("sha256")
        if not _is_lowercase_sha256(screenshot_sha256):
            errors.append(
                f"{label}.screenshot.sha256 must be a lowercase SHA-256 digest"
            )

        image_verdict = capture.get("image_verdict")
        if not isinstance(image_verdict, Mapping):
            errors.append(f"{label}.image_verdict must be an object")
            continue
        image_sha256 = image_verdict.get("image_sha256")
        if not _is_lowercase_sha256(image_sha256):
            errors.append(
                f"{label}.image_verdict.image_sha256 must be a lowercase "
                "SHA-256 digest"
            )
        if _is_lowercase_sha256(screenshot_sha256) and _is_lowercase_sha256(
            image_sha256
        ):
            if image_sha256 != screenshot_sha256:
                errors.append(
                    f"{label}.image_verdict.image_sha256 must match "
                    f"{label}.screenshot.sha256"
                )
        if _paper_requires_long_horizon(packet):
            capture_spec = capture_specs[role]
            assert isinstance(capture_spec, Mapping)
            metadata = image_verdict.get("metadata")
            if not isinstance(metadata, Mapping):
                errors.append(f"{label}.image_verdict.metadata must be an object")
            else:
                expected_frame = str(capture_spec["frame"])
                if metadata.get("frame") != expected_frame:
                    errors.append(
                        f"{label}.image_verdict.metadata.frame must be "
                        f"{expected_frame!r}"
                    )
                expected_scene = PAPER_FIGURE13_SPECS[packet_name]["scene"]
                if metadata.get("scene") != expected_scene:
                    errors.append(
                        f"{label}.image_verdict.metadata.scene must be "
                        f"{expected_scene!r}"
                    )
    return errors


def _ordered_png_manifest_digest(
    files: list[Mapping[str, object]],
) -> str | None:
    try:
        digest = hashlib.sha256()
        digest.update(struct.pack("<Q", len(files)))
        for entry in files:
            index = entry["index"]
            name = entry["file"]
            size_bytes = entry["size_bytes"]
            file_digest = entry["sha256"]
            if (
                not isinstance(index, int)
                or isinstance(index, bool)
                or not isinstance(name, str)
                or not isinstance(size_bytes, int)
                or isinstance(size_bytes, bool)
                or not _is_lowercase_sha256(file_digest)
            ):
                return None
            encoded_name = name.encode("utf-8")
            digest.update(struct.pack("<Q", index))
            digest.update(struct.pack("<Q", len(encoded_name)))
            digest.update(encoded_name)
            digest.update(struct.pack("<Q", size_bytes))
            digest.update(bytes.fromhex(file_digest))
        return digest.hexdigest()
    except KeyError, OverflowError, struct.error, UnicodeError, ValueError:
        return None


def _capture_artifact_manifest_digest(
    provenance: Mapping[str, object],
) -> str | None:
    payload = {
        key: provenance.get(key)
        for key in (
            "artifact_count",
            "png_frames",
            "scene_metrics_events_sha256",
            "screenshot_png_frame_binding",
            "screenshot_sha256",
            "video",
        )
    }
    try:
        encoded = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except TypeError, ValueError, UnicodeError:
        return None
    return hashlib.sha256(encoded).hexdigest()


CAPTURE_RUNTIME_PROVENANCE_PAYLOAD_KEYS = (
    "dart_library_linkage",
    "loader_environment",
    "loaded_dart_libraries",
    "native_extension",
    "runtime_image_inventory",
    "source_git_head",
    "source_provenance_digest",
)
CAPTURE_RUNTIME_PROVENANCE_KEYS = frozenset(
    {"algorithm", "digest", *CAPTURE_RUNTIME_PROVENANCE_PAYLOAD_KEYS}
)


def _capture_runtime_manifest_digest(
    provenance: Mapping[str, object],
) -> str | None:
    """Recompute the digest ``capture_runtime_provenance`` binds.

    The payload must stay byte-identical to the producer's, which digests the
    complete manifest minus ``algorithm``/``digest`` — loader attestation and
    the mapped-image inventory included.
    """
    payload = {
        key: provenance.get(key) for key in CAPTURE_RUNTIME_PROVENANCE_PAYLOAD_KEYS
    }
    try:
        encoded = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except TypeError, ValueError, UnicodeError:
        return None
    return hashlib.sha256(encoded).hexdigest()


def _capture_loader_environment_errors(value: object, label: str) -> list[str]:
    """Require the capture-side empty loader-control attestation.

    The invariant fields use the same constants the benchmark loader policy is
    held to; ``trusted_image_roots`` names machine-local absolute directories,
    so it is checked for shape rather than compared to this host's roots.
    """
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    expected_keys = {
        "algorithm",
        "forbidden_environment_prefixes",
        "passed",
        "present_environment_variables",
        "trusted_image_roots",
    }
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    expected_policy = {
        "algorithm": CAPTURE_LOADER_POLICY_ALGORITHM,
        "forbidden_environment_prefixes": list(CAPTURE_LOADER_ENVIRONMENT_PREFIXES),
        "passed": True,
        "present_environment_variables": [],
    }
    for key, expected in expected_policy.items():
        if not _json_values_equal_exact(value.get(key), expected):
            errors.append(
                f"{label}.{key} must prove an empty loader-control environment"
            )
    roots = value.get("trusted_image_roots")
    if (
        not isinstance(roots, list)
        or not roots
        or any(
            not isinstance(root, str) or not Path(root).is_absolute() for root in roots
        )
        or roots != sorted(set(roots))
    ):
        errors.append(
            f"{label}.trusted_image_roots must be a unique sorted list of "
            "absolute paths"
        )
    return errors


def _capture_runtime_image_inventory_errors(
    value: object,
    *,
    extension: object,
    library_paths: list[str],
    label: str,
) -> list[str]:
    """Require the complete mapped-image inventory the capture recorded."""
    if not isinstance(value, Mapping):
        return [f"{label} must be an object"]
    errors: list[str] = []
    expected_keys = {"algorithm", "digest", "images", "required_images"}
    if set(value) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    if value.get("algorithm") != CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM:
        errors.append(
            f"{label}.algorithm must be {CAPTURE_RUNTIME_IMAGE_INVENTORY_ALGORITHM!r}"
        )
    images = value.get("images")
    if not isinstance(images, list) or not images:
        errors.append(f"{label}.images must be a non-empty list")
        images = []
    paths: list[str] = []
    image_by_path: dict[str, Mapping[str, object]] = {}
    dart_paths: list[str] = []
    for index, image in enumerate(images):
        image_label = f"{label}.images[{index}]"
        if not isinstance(image, Mapping):
            errors.append(f"{image_label} must be an object")
            continue
        image_keys = {"file", "path", "sha256", "size_bytes"}
        if set(image) != image_keys:
            errors.append(f"{image_label} must contain exactly {sorted(image_keys)!r}")
        file_value = image.get("file")
        path_value = image.get("path")
        if (
            not isinstance(file_value, str)
            or not file_value
            or not isinstance(path_value, str)
            or not Path(path_value).is_absolute()
            or Path(path_value).name != file_value
        ):
            errors.append(f"{image_label}.path must be absolute and match its file")
            continue
        paths.append(path_value)
        image_by_path[path_value] = image
        if _is_dart_runtime_image_name(file_value):
            dart_paths.append(path_value)
        if not _is_lowercase_sha256(image.get("sha256")):
            errors.append(f"{image_label}.sha256 must be a lowercase SHA-256 digest")
        size = image.get("size_bytes")
        if (
            not isinstance(size, int)
            or isinstance(size, bool)
            or not 1 <= size <= 0xFFFFFFFFFFFFFFFF
        ):
            errors.append(f"{image_label}.size_bytes must be a positive uint64")
    if paths != sorted(set(paths)) or len(paths) != len(images):
        errors.append(f"{label}.images must be unique and path-sorted")
    extension_path = extension.get("path") if isinstance(extension, Mapping) else None
    if not _json_values_equal_exact(
        value.get("required_images"),
        {"dartpy_native_extension": extension_path},
    ):
        errors.append(
            f"{label}.required_images must bind the imported dartpy extension"
        )
    if isinstance(extension, Mapping) and isinstance(extension_path, str):
        recorded = image_by_path.get(extension_path)
        expected_image = {
            key: extension.get(key) for key in ("file", "path", "sha256", "size_bytes")
        }
        if recorded is None or not _json_values_equal_exact(recorded, expected_image):
            errors.append(
                f"{label}.images must contain the dartpy extension bytes verbatim"
            )
    if sorted(dart_paths) != sorted(library_paths):
        errors.append(
            f"{label}.images must contain exactly the loaded DART libraries "
            "named by loaded_dart_libraries"
        )
    payload = {
        "images": value.get("images"),
        "required_images": value.get("required_images"),
    }
    if not _is_lowercase_sha256(value.get("digest")) or value.get(
        "digest"
    ) != _canonical_json_digest(payload):
        errors.append(f"{label}.digest must be derived from the complete inventory")
    return errors


def _is_dart_runtime_image_name(file_name: str) -> bool:
    name = file_name.lower()
    return name.startswith("libdart") or (
        name.startswith("dart") and name.endswith(".dll")
    )


def _paper_capture_runtime_errors(
    capture: Mapping[str, object],
    *,
    expected_source_digest: object,
    expected_source_git_head: object,
    label: str,
) -> list[str]:
    provenance = capture.get("runtime_provenance")
    if not isinstance(provenance, Mapping):
        return [f"{label}.runtime_provenance must be an object"]
    errors: list[str] = []
    expected_keys = set(CAPTURE_RUNTIME_PROVENANCE_KEYS)
    if set(provenance) != expected_keys:
        errors.append(
            f"{label}.runtime_provenance must contain exactly "
            f"{sorted(expected_keys)!r}"
        )
    errors.extend(
        _capture_loader_environment_errors(
            provenance.get("loader_environment"),
            f"{label}.runtime_provenance.loader_environment",
        )
    )
    if provenance.get("algorithm") != CAPTURE_RUNTIME_PROVENANCE_ALGORITHM:
        errors.append(
            f"{label}.runtime_provenance.algorithm must be "
            f"{CAPTURE_RUNTIME_PROVENANCE_ALGORITHM!r}"
        )
    if (
        not _is_lowercase_sha256(provenance.get("source_provenance_digest"))
        or provenance.get("source_provenance_digest") != expected_source_digest
    ):
        errors.append(
            f"{label}.runtime_provenance.source_provenance_digest must match "
            "capture source provenance"
        )
    if provenance.get("source_git_head") != expected_source_git_head:
        errors.append(
            f"{label}.runtime_provenance.source_git_head must match capture "
            "source provenance"
        )

    def binary_errors(
        value: object, binary_label: str, *, extension: bool
    ) -> list[str]:
        result: list[str] = []
        if not isinstance(value, Mapping):
            return [f"{binary_label} must be an object"]
        keys = {"file", "path", "sha256", "size_bytes"}
        if extension:
            keys.update(
                {
                    "build_configuration_digest",
                    "module",
                    "source_git_head",
                    "source_provenance_digest",
                }
            )
        else:
            keys.add("build_identity")
        if set(value) != keys:
            result.append(f"{binary_label} must contain exactly {sorted(keys)!r}")
        file_value = value.get("file")
        path_value = value.get("path")
        if (
            not isinstance(file_value, str)
            or not file_value
            or not isinstance(path_value, str)
            or not path_value
            or Path(path_value).name != file_value
        ):
            result.append(f"{binary_label} file/path identity is invalid")
        if not _is_lowercase_sha256(value.get("sha256")):
            result.append(f"{binary_label}.sha256 must be a lowercase SHA-256 digest")
        size_bytes = value.get("size_bytes")
        if (
            not isinstance(size_bytes, int)
            or isinstance(size_bytes, bool)
            or size_bytes < 1
            or size_bytes > 0xFFFFFFFFFFFFFFFF
        ):
            result.append(f"{binary_label}.size_bytes must be a positive uint64")
        if extension:
            if value.get("module") != "dartpy._dartpy":
                result.append(f"{binary_label}.module must be 'dartpy._dartpy'")
            if value.get("source_provenance_digest") != expected_source_digest:
                result.append(
                    f"{binary_label}.source_provenance_digest must match capture source"
                )
            if value.get("source_git_head") != expected_source_git_head:
                result.append(
                    f"{binary_label}.source_git_head must match capture source"
                )
            if not _is_lowercase_sha256(value.get("build_configuration_digest")):
                result.append(
                    f"{binary_label}.build_configuration_digest must be SHA-256"
                )
        else:
            result.extend(
                _paper_dart_library_build_identity_errors(
                    value.get("build_identity"),
                    f"{binary_label}.build_identity",
                    expected_source_digest=expected_source_digest,
                    expected_source_git_head=expected_source_git_head,
                )
            )
        return result

    extension = provenance.get("native_extension")
    errors.extend(
        binary_errors(
            extension,
            f"{label}.runtime_provenance.native_extension",
            extension=True,
        )
    )
    libraries = provenance.get("loaded_dart_libraries")
    extension_configuration_digest = (
        extension.get("build_configuration_digest")
        if isinstance(extension, Mapping)
        else None
    )
    library_paths: list[str] = []
    if not isinstance(libraries, list) or not libraries:
        errors.append(
            f"{label}.runtime_provenance.loaded_dart_libraries must be a "
            "non-empty shared-library inventory"
        )
        libraries = []
    for index, library in enumerate(libraries):
        errors.extend(
            binary_errors(
                library,
                f"{label}.runtime_provenance.loaded_dart_libraries[{index}]",
                extension=False,
            )
        )
        if isinstance(library, Mapping) and isinstance(library.get("path"), str):
            library_paths.append(library["path"])
        library_identity = (
            library.get("build_identity") if isinstance(library, Mapping) else None
        )
        if (
            isinstance(library_identity, Mapping)
            and library_identity.get("build_configuration_digest")
            != extension_configuration_digest
        ):
            errors.append(
                f"{label}.runtime_provenance.loaded_dart_libraries[{index}]."
                "build_identity.build_configuration_digest must match the "
                "dartpy native extension"
            )
    if library_paths != sorted(set(library_paths)):
        errors.append(
            f"{label}.runtime_provenance.loaded_dart_libraries must be unique "
            "and path-sorted"
        )
    errors.extend(
        _capture_runtime_image_inventory_errors(
            provenance.get("runtime_image_inventory"),
            extension=extension,
            library_paths=library_paths,
            label=f"{label}.runtime_provenance.runtime_image_inventory",
        )
    )
    if provenance.get("dart_library_linkage") != "shared":
        errors.append(
            f"{label}.runtime_provenance.dart_library_linkage must be " "'shared'"
        )
    derived_digest = _capture_runtime_manifest_digest(provenance)
    if (
        not _is_lowercase_sha256(provenance.get("digest"))
        or provenance.get("digest") != derived_digest
    ):
        errors.append(
            f"{label}.runtime_provenance.digest must be derived from the "
            "complete runtime inventory"
        )
    return errors


def _paper_capture_artifact_errors(
    capture: Mapping[str, object],
    *,
    expected_frame: int,
    label: str,
    role: str,
    scene: str,
) -> list[str]:
    errors: list[str] = []
    capture_contract = capture.get("capture")
    expected_width = (
        capture_contract.get("width") if isinstance(capture_contract, Mapping) else None
    )
    expected_height = (
        capture_contract.get("height")
        if isinstance(capture_contract, Mapping)
        else None
    )
    provenance = capture.get("artifact_provenance")
    if not isinstance(provenance, Mapping):
        return [f"{label}.artifact_provenance must be an object"]
    expected_keys = {
        "algorithm",
        "artifact_count",
        "digest",
        "png_frames",
        "scene_metrics_events_sha256",
        "screenshot_png_frame_binding",
        "screenshot_sha256",
        "video",
    }
    if set(provenance) != expected_keys:
        errors.append(
            f"{label}.artifact_provenance must contain exactly "
            f"{sorted(expected_keys)!r}"
        )
    if provenance.get("algorithm") != CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM:
        errors.append(
            f"{label}.artifact_provenance.algorithm must be "
            f"{CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM!r}"
        )
    expected_artifact_count = expected_frame + 3
    if not _json_values_equal_exact(
        provenance.get("artifact_count"), expected_artifact_count
    ):
        errors.append(
            f"{label}.artifact_provenance.artifact_count must be "
            f"{expected_artifact_count}"
        )

    screenshot = capture.get("screenshot")
    screenshot_sha = (
        screenshot.get("sha256") if isinstance(screenshot, Mapping) else None
    )
    expected_screenshot_name = f"{scene}_{role}.png"
    if not isinstance(screenshot, Mapping) or not _json_values_equal_exact(
        screenshot.get("file"), expected_screenshot_name
    ):
        errors.append(f"{label}.screenshot.file must be {expected_screenshot_name!r}")
    if not _is_lowercase_sha256(provenance.get("screenshot_sha256")) or (
        provenance.get("screenshot_sha256") != screenshot_sha
    ):
        errors.append(
            f"{label}.artifact_provenance.screenshot_sha256 must match the "
            "packet screenshot digest"
        )
    screenshot_binding = provenance.get("screenshot_png_frame_binding")
    screenshot_binding_label = (
        f"{label}.artifact_provenance.screenshot_png_frame_binding"
    )
    if not isinstance(screenshot_binding, Mapping):
        errors.append(f"{screenshot_binding_label} must be an object")
    else:
        screenshot_binding_keys = {
            "algorithm",
            "passed",
            "png_frame_file",
            "png_frame_index",
            "png_frame_sha256",
        }
        if set(screenshot_binding) != screenshot_binding_keys:
            errors.append(
                f"{screenshot_binding_label} must contain exactly "
                f"{sorted(screenshot_binding_keys)!r}"
            )
        expected_bound_frame = f"frame_{expected_frame:06d}.png"
        for key, expected in (
            ("algorithm", CAPTURE_SCREENSHOT_BINDING_ALGORITHM),
            ("passed", True),
            ("png_frame_file", expected_bound_frame),
            ("png_frame_index", expected_frame),
        ):
            if not _json_values_equal_exact(screenshot_binding.get(key), expected):
                errors.append(f"{screenshot_binding_label}.{key} must be {expected!r}")
    events = capture.get("scene_metrics_events")
    events_sha = events.get("sha256") if isinstance(events, Mapping) else None
    if not isinstance(events, Mapping) or not _json_values_equal_exact(
        events.get("file"), "scene_metrics.jsonl"
    ):
        errors.append(
            f"{label}.scene_metrics_events.file must be 'scene_metrics.jsonl'"
        )
    if (
        not _is_lowercase_sha256(provenance.get("scene_metrics_events_sha256"))
        or provenance.get("scene_metrics_events_sha256") != events_sha
    ):
        errors.append(
            f"{label}.artifact_provenance.scene_metrics_events_sha256 must "
            "match the packet event-log digest"
        )
    manifest = capture.get("manifest")
    if not isinstance(manifest, Mapping):
        errors.append(f"{label}.manifest must be an object")
    else:
        if not _json_values_equal_exact(manifest.get("file"), "manifest.json"):
            errors.append(f"{label}.manifest.file must be 'manifest.json'")
        if not _is_lowercase_sha256(manifest.get("sha256")):
            errors.append(f"{label}.manifest.sha256 must be a lowercase SHA-256 digest")

    png_frames = provenance.get("png_frames")
    if not isinstance(png_frames, Mapping):
        errors.append(f"{label}.artifact_provenance.png_frames must be an object")
    else:
        png_keys = {"algorithm", "count", "digest", "files", "height", "width"}
        if set(png_frames) != png_keys:
            errors.append(
                f"{label}.artifact_provenance.png_frames must contain exactly "
                f"{sorted(png_keys)!r}"
            )
        if png_frames.get("algorithm") != CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM:
            errors.append(
                f"{label}.artifact_provenance.png_frames.algorithm must be "
                f"{CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM!r}"
            )
        if not _json_values_equal_exact(png_frames.get("count"), expected_frame):
            errors.append(
                f"{label}.artifact_provenance.png_frames.count must be "
                f"{expected_frame}"
            )
        for key, expected in (
            ("width", expected_width),
            ("height", expected_height),
        ):
            if not _json_values_equal_exact(png_frames.get(key), expected):
                errors.append(
                    f"{label}.artifact_provenance.png_frames.{key} must be "
                    f"{expected!r}"
                )
        files = png_frames.get("files")
        valid_files: list[Mapping[str, object]] = []
        if not isinstance(files, list) or len(files) != expected_frame:
            errors.append(
                f"{label}.artifact_provenance.png_frames.files must contain "
                f"exactly {expected_frame} ordered entries"
            )
        else:
            for index, entry in enumerate(files, start=1):
                entry_label = (
                    f"{label}.artifact_provenance.png_frames.files[{index - 1}]"
                )
                if not isinstance(entry, Mapping):
                    errors.append(f"{entry_label} must be an object")
                    continue
                valid_files.append(entry)
                if set(entry) != {"file", "index", "sha256", "size_bytes"}:
                    errors.append(
                        f"{entry_label} must contain exactly file, index, "
                        "sha256, and size_bytes"
                    )
                if not _json_values_equal_exact(entry.get("index"), index):
                    errors.append(f"{entry_label}.index must be {index}")
                expected_name = f"frame_{index:06d}.png"
                if entry.get("file") != expected_name:
                    errors.append(f"{entry_label}.file must be {expected_name!r}")
                if not _is_lowercase_sha256(entry.get("sha256")):
                    errors.append(
                        f"{entry_label}.sha256 must be a lowercase SHA-256 digest"
                    )
                size_bytes = entry.get("size_bytes")
                if (
                    not isinstance(size_bytes, int)
                    or isinstance(size_bytes, bool)
                    or size_bytes < 1
                    or size_bytes > 0xFFFFFFFFFFFFFFFF
                ):
                    errors.append(f"{entry_label}.size_bytes must be a positive uint64")
            derived_digest = _ordered_png_manifest_digest(valid_files)
            if (
                not _is_lowercase_sha256(png_frames.get("digest"))
                or png_frames.get("digest") != derived_digest
            ):
                errors.append(
                    f"{label}.artifact_provenance.png_frames.digest must be "
                    "derived from the ordered frame entries"
                )
            if isinstance(screenshot_binding, Mapping) and valid_files:
                if screenshot_binding.get("png_frame_sha256") != valid_files[-1].get(
                    "sha256"
                ):
                    errors.append(
                        f"{screenshot_binding_label}.png_frame_sha256 must match "
                        "the terminal PNG entry"
                    )

    video = provenance.get("video")
    if not isinstance(video, Mapping):
        errors.append(f"{label}.artifact_provenance.video must be an object")
    else:
        video_keys = {
            "codec_name",
            "content_correspondence",
            "decoded_frame_count",
            "duration_seconds",
            "file",
            "fps",
            "height",
            "pixel_format",
            "probe_algorithm",
            "sha256",
            "size_bytes",
            "width",
        }
        if set(video) != video_keys:
            errors.append(
                f"{label}.artifact_provenance.video must contain exactly "
                f"{sorted(video_keys)!r}"
            )
        expected_video_name = f"{scene}_{role}.mp4"
        for key, expected in (
            ("file", expected_video_name),
            ("fps", f"{_PAPER_CAPTURE_VIDEO_FPS}/1"),
            ("decoded_frame_count", expected_frame),
            ("width", expected_width),
            ("height", expected_height),
            (
                "duration_seconds",
                (
                    f"{Fraction(expected_frame, _PAPER_CAPTURE_VIDEO_FPS).numerator}/"
                    f"{Fraction(expected_frame, _PAPER_CAPTURE_VIDEO_FPS).denominator}"
                ),
            ),
            ("probe_algorithm", CAPTURE_VIDEO_PROBE_ALGORITHM),
        ):
            if not _json_values_equal_exact(video.get(key), expected):
                errors.append(
                    f"{label}.artifact_provenance.video.{key} must be " f"{expected!r}"
                )
        codec = video.get("codec_name")
        if codec != "h264":
            errors.append(
                f"{label}.artifact_provenance.video.codec_name must be 'h264'"
            )
        if video.get("pixel_format") != "yuv420p":
            errors.append(
                f"{label}.artifact_provenance.video.pixel_format must be 'yuv420p'"
            )
        correspondence = video.get("content_correspondence")
        correspondence_label = (
            f"{label}.artifact_provenance.video.content_correspondence"
        )
        correspondence_keys = {
            "algorithm",
            "encoder",
            "expected_reencoded_sha256",
            "passed",
            "source_png_sequence_digest",
        }
        if not isinstance(correspondence, Mapping):
            errors.append(f"{correspondence_label} must be an object")
        else:
            if set(correspondence) != correspondence_keys:
                errors.append(
                    f"{correspondence_label} must contain exactly "
                    f"{sorted(correspondence_keys)!r}"
                )
            encoder = correspondence.get("encoder")
            if not isinstance(encoder, Mapping) or set(encoder) != {
                *CAPTURE_VIDEO_ENCODER,
                "ffmpeg_version",
                "libx264_version",
            }:
                errors.append(
                    f"{correspondence_label}.encoder must record the pinned "
                    "settings plus the ffmpeg and libx264 versions the bytes "
                    "depend on"
                )
            else:
                for key, expected in CAPTURE_VIDEO_ENCODER.items():
                    if not _json_values_equal_exact(encoder.get(key), expected):
                        errors.append(
                            f"{correspondence_label}.encoder.{key} must be "
                            f"{expected!r}"
                        )
                for key in ("ffmpeg_version", "libx264_version"):
                    value = encoder.get(key)
                    if not isinstance(value, str) or not value.strip():
                        errors.append(
                            f"{correspondence_label}.encoder.{key} must be a "
                            "non-empty toolchain version"
                        )
            for key, expected in (
                ("algorithm", CAPTURE_VIDEO_CONTENT_CORRESPONDENCE_ALGORITHM),
                ("expected_reencoded_sha256", video.get("sha256")),
                ("passed", True),
                (
                    "source_png_sequence_digest",
                    (
                        png_frames.get("digest")
                        if isinstance(png_frames, Mapping)
                        else None
                    ),
                ),
            ):
                if not _json_values_equal_exact(correspondence.get(key), expected):
                    errors.append(f"{correspondence_label}.{key} must be {expected!r}")
        if not _is_lowercase_sha256(video.get("sha256")):
            errors.append(
                f"{label}.artifact_provenance.video.sha256 must be a "
                "lowercase SHA-256 digest"
            )
        video_size = video.get("size_bytes")
        if (
            not isinstance(video_size, int)
            or isinstance(video_size, bool)
            or video_size < 1
            or video_size > 0xFFFFFFFFFFFFFFFF
        ):
            errors.append(
                f"{label}.artifact_provenance.video.size_bytes must be a "
                "positive uint64"
            )

    derived_manifest_digest = _capture_artifact_manifest_digest(provenance)
    if (
        not _is_lowercase_sha256(provenance.get("digest"))
        or provenance.get("digest") != derived_manifest_digest
    ):
        errors.append(
            f"{label}.artifact_provenance.digest must be derived from the "
            "complete capture artifact manifest"
        )
    return errors


def _finite_number(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    try:
        converted = float(value)
    except OverflowError, TypeError, ValueError:
        return None
    return converted if math.isfinite(converted) else None


def _numbers_match(actual: object, expected: object) -> bool:
    actual_number = _finite_number(actual)
    expected_number = _finite_number(expected)
    if actual_number is None or expected_number is None:
        return False
    return math.isclose(actual_number, expected_number, rel_tol=1e-12, abs_tol=1e-12)


def _numbers_close(actual: object, expected: object) -> bool:
    """Exact for integers; a 1e-12 relative bound for real-valued counters."""
    actual_number = _finite_number(actual)
    expected_number = _finite_number(expected)
    if actual_number is None or expected_number is None:
        return False
    if float(expected_number).is_integer():
        return actual_number == expected_number
    return math.isclose(actual_number, expected_number, rel_tol=1e-12, abs_tol=0.0)


def _numbers_equal(actual: object, expected: object) -> bool:
    actual_number = _finite_number(actual)
    expected_number = _finite_number(expected)
    return (
        actual_number is not None
        and expected_number is not None
        and actual_number == expected_number
    )


def _json_values_equal_exact(actual: object, expected: object) -> bool:
    """Compare JSON-shaped values without Python's bool/int equivalence."""
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _json_values_equal_exact(actual[key], value)
            for key, value in expected.items()
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _json_values_equal_exact(actual_value, expected_value)
            for actual_value, expected_value in zip(actual, expected)
        )
    return actual == expected


def _finite_ratio(numerator: float, denominator: float) -> float | None:
    try:
        ratio = numerator / denominator
    except OverflowError, ZeroDivisionError:
        return None
    return ratio if math.isfinite(ratio) and ratio > 0.0 else None


def _paper_threshold_number(
    errors: list[str],
    *,
    outcome: Mapping[str, object],
    oracle: Mapping[str, object],
    outcome_key: str,
    oracle_key: str,
    label: str,
    relation: str,
) -> None:
    actual = _finite_number(outcome.get(outcome_key))
    threshold = _finite_number(oracle.get(oracle_key))
    if actual is None:
        errors.append(f"{label}.{outcome_key} must be a finite number")
        return
    if threshold is None:
        errors.append(f"{label}.outcome_oracle.{oracle_key} must be a finite number")
        return
    passed = actual >= threshold if relation == "minimum" else actual <= threshold
    if not passed:
        operator = ">=" if relation == "minimum" else "<="
        errors.append(
            f"{label}.{outcome_key} must be {operator} "
            f"outcome_oracle.{oracle_key} ({threshold!r}), got {actual!r}"
        )


def _paper_outcome_threshold_errors(
    packet_name: str,
    role: str,
    outcome: Mapping[str, object],
    oracle: Mapping[str, object],
) -> list[str]:
    """Re-evaluate the numeric predicates behind one Figure 13 checkpoint."""
    label = f"{packet_name}: visual_evidence.{role}.scene_metrics.outcome"
    errors: list[str] = []

    broken = outcome.get("broken_joints")
    unbroken = outcome.get("unbroken_joints")
    if (
        not isinstance(broken, int)
        or isinstance(broken, bool)
        or not isinstance(unbroken, int)
        or isinstance(unbroken, bool)
        or broken < 0
        or unbroken < 0
        or broken + unbroken != 712
    ):
        errors.append(
            f"{label}.broken_joints and unbroken_joints must be non-negative "
            "integers summing to 712"
        )
    else:
        identity_count = outcome.get("broken_joint_identity_count")
        if (
            not isinstance(identity_count, int)
            or isinstance(identity_count, bool)
            or identity_count != broken
        ):
            errors.append(
                f"{label}.broken_joint_identity_count must be an integer "
                "equal to broken_joints"
            )
        residual_count = outcome.get("unbroken_joint_residual_count")
        if (
            not isinstance(residual_count, int)
            or isinstance(residual_count, bool)
            or residual_count != unbroken
        ):
            errors.append(
                f"{label}.unbroken_joint_residual_count must be an integer "
                "equal to unbroken_joints"
            )

    records = outcome.get("broken_joint_records")
    record_ids: list[str] = []
    if not isinstance(records, list):
        errors.append(f"{label}.broken_joint_records must be a list")
    else:
        if (
            isinstance(broken, int)
            and not isinstance(broken, bool)
            and len(records) != broken
        ):
            errors.append(
                f"{label}.broken_joint_records length must equal broken_joints"
            )
        for index, record in enumerate(records):
            joint_id = record.get("id") if isinstance(record, Mapping) else None
            if not isinstance(joint_id, str) or not joint_id:
                errors.append(
                    f"{label}.broken_joint_records[{index}].id must be a "
                    "non-empty string"
                )
            else:
                record_ids.append(joint_id)
        if len(record_ids) != len(set(record_ids)):
            errors.append(f"{label}.broken_joint_records IDs must be unique")
        try:
            identity_digest = hashlib.sha256()
            for joint_id in sorted(record_ids):
                encoded = joint_id.encode("utf-8")
                identity_digest.update(struct.pack("<Q", len(encoded)))
                identity_digest.update(encoded)
        except (OverflowError, UnicodeError) as exc:
            errors.append(f"{label}.broken_joint_records IDs cannot be hashed ({exc})")
        else:
            if outcome.get("broken_joint_ids_sha256") != identity_digest.hexdigest():
                errors.append(
                    f"{label}.broken_joint_ids_sha256 must be derived from "
                    "broken_joint_records"
                )

    region_counts = outcome.get("broken_joint_impact_region_counts")
    outside_count = outcome.get("broken_joints_outside_impact_regions")
    if (
        not isinstance(region_counts, list)
        or len(region_counts) != 3
        or any(
            not isinstance(value, int) or isinstance(value, bool) or value < 0
            for value in region_counts
        )
        or not isinstance(outside_count, int)
        or isinstance(outside_count, bool)
        or outside_count < 0
        or not isinstance(broken, int)
        or isinstance(broken, bool)
        or sum(region_counts) + outside_count != broken
    ):
        errors.append(
            f"{label}.broken_joint_impact_region_counts plus the outside count "
            "must reconcile with broken_joints"
        )

    expected_outside_residual_count = (
        463 if packet_name == "avbd-paper-breakable-wall-packet.json" else 484
    )
    outside_residual_count = outcome.get("outside_impact_unbroken_joint_residual_count")
    if (
        not isinstance(outside_residual_count, int)
        or isinstance(outside_residual_count, bool)
        or outside_residual_count != expected_outside_residual_count
    ):
        errors.append(
            f"{label}.outside_impact_unbroken_joint_residual_count must be "
            f"{expected_outside_residual_count}"
        )

    for maximum_key, rms_key in (
        (
            "maximum_unbroken_joint_linear_residual",
            "rms_unbroken_joint_linear_residual",
        ),
        (
            "maximum_unbroken_joint_angular_residual_radians",
            "rms_unbroken_joint_angular_residual_radians",
        ),
        (
            "maximum_outside_impact_unbroken_joint_linear_residual",
            "rms_outside_impact_unbroken_joint_linear_residual",
        ),
        (
            "maximum_outside_impact_unbroken_joint_angular_residual_radians",
            "rms_outside_impact_unbroken_joint_angular_residual_radians",
        ),
    ):
        maximum = _finite_number(outcome.get(maximum_key))
        rms = _finite_number(outcome.get(rms_key))
        if (
            maximum is None
            or rms is None
            or maximum < 0.0
            or rms < 0.0
            or rms > maximum
        ):
            errors.append(
                f"{label}.{maximum_key}/{rms_key} must be finite, non-negative, "
                "and RMS cannot exceed the maximum"
            )

    expected_identity = oracle.get("expected_broken_joint_ids_sha256")
    identity_oracle_required = packet_name != "avbd-paper-vbd-comparison-packet.json"
    if (identity_oracle_required and not _is_lowercase_sha256(expected_identity)) or (
        expected_identity is not None
        and outcome.get("broken_joint_ids_sha256") != expected_identity
    ):
        errors.append(f"{label}.broken_joint_ids_sha256 must match the outcome oracle")
    if outcome.get("joint_residuals_finite") is not True:
        errors.append(f"{label}.joint_residuals_finite must be true")

    if packet_name == "avbd-paper-breakable-wall-packet.json":
        for outcome_key, oracle_key, relation in (
            ("broken_joints", "minimum_broken_joints", "minimum"),
            ("broken_joints", "maximum_broken_joints", "maximum"),
            ("unbroken_joints", "minimum_unbroken_joints", "minimum"),
            (
                "total_retained_fraction",
                "minimum_total_retained_fraction",
                "minimum",
            ),
            (
                "outside_retained_fraction",
                "minimum_outside_retained_fraction",
                "minimum",
            ),
            (
                "maximum_unbroken_joint_linear_residual",
                "maximum_unbroken_joint_linear_residual",
                "maximum",
            ),
            (
                "maximum_unbroken_joint_angular_residual_radians",
                "maximum_unbroken_joint_angular_residual_radians",
                "maximum",
            ),
        ):
            _paper_threshold_number(
                errors,
                outcome=outcome,
                oracle=oracle,
                outcome_key=outcome_key,
                oracle_key=oracle_key,
                label=label,
                relation=relation,
            )
        region_counts = outcome.get("broken_joint_impact_region_counts")
        minimum = _finite_number(oracle.get("minimum_broken_joints_per_impact_region"))
        if (
            not isinstance(region_counts, list)
            or len(region_counts) != 3
            or minimum is None
            or any(
                _finite_number(value) is None or _finite_number(value) < minimum
                for value in region_counts
            )
        ):
            errors.append(
                f"{label}.broken_joint_impact_region_counts must satisfy all "
                "three outcome-oracle minima"
            )
    elif packet_name == "avbd-paper-vbd-comparison-packet.json":
        for outcome_key, oracle_key, relation in (
            ("broken_joints", "maximum_broken_joints", "maximum"),
            ("unbroken_joints", "minimum_unbroken_joints", "minimum"),
            (
                "maximum_unbroken_joint_linear_residual",
                "maximum_unbroken_joint_linear_residual",
                "maximum",
            ),
            (
                "maximum_unbroken_joint_angular_residual_radians",
                "maximum_unbroken_joint_angular_residual_radians",
                "maximum",
            ),
        ):
            _paper_threshold_number(
                errors,
                outcome=outcome,
                oracle=oracle,
                outcome_key=outcome_key,
                oracle_key=oracle_key,
                label=label,
                relation=relation,
            )
        if outcome.get("checkpoint") == "bend":
            for outcome_key, oracle_key in (
                (
                    "maximum_wall_normal_displacement",
                    "minimum_maximum_wall_normal_displacement",
                ),
                (
                    "rms_wall_normal_displacement",
                    "minimum_rms_wall_normal_displacement",
                ),
                ("bent_brick_count", "minimum_bent_bricks"),
            ):
                _paper_threshold_number(
                    errors,
                    outcome=outcome,
                    oracle=oracle,
                    outcome_key=outcome_key,
                    oracle_key=oracle_key,
                    label=label,
                    relation="minimum",
                )
        else:
            _paper_threshold_number(
                errors,
                outcome=outcome,
                oracle=oracle,
                outcome_key="total_retained_fraction",
                oracle_key="minimum_total_retained_fraction",
                label=label,
                relation="minimum",
            )
    else:
        if outcome.get("checkpoint") == "fracture":
            predicates = (
                ("broken_joints", "minimum_initial_broken_joints", "minimum"),
                ("broken_joints", "maximum_initial_broken_joints", "maximum"),
                ("unbroken_joints", "minimum_initial_unbroken_joints", "minimum"),
                (
                    "total_retained_fraction",
                    "minimum_initial_total_retained_fraction",
                    "minimum",
                ),
                (
                    "maximum_outside_impact_unbroken_joint_linear_residual",
                    "maximum_initial_outside_joint_linear_residual",
                    "maximum",
                ),
                (
                    "maximum_outside_impact_unbroken_joint_angular_residual_radians",
                    "maximum_initial_outside_joint_angular_residual_radians",
                    "maximum",
                ),
            )
        else:
            predicates = (
                ("broken_joints", "minimum_final_broken_joints", "minimum"),
                ("broken_joints", "maximum_final_broken_joints", "maximum"),
                ("unbroken_joints", "minimum_final_unbroken_joints", "minimum"),
                (
                    "total_retained_fraction",
                    "maximum_collapse_total_retained_fraction",
                    "maximum",
                ),
                (
                    "outside_retained_fraction",
                    "maximum_collapse_outside_retained_fraction",
                    "maximum",
                ),
                (
                    "maximum_wall_normal_displacement",
                    "minimum_collapse_wall_normal_displacement",
                    "minimum",
                ),
                (
                    "maximum_outside_impact_unbroken_joint_linear_residual",
                    "minimum_collapse_outside_joint_maximum_linear_residual",
                    "minimum",
                ),
                (
                    "rms_outside_impact_unbroken_joint_linear_residual",
                    "minimum_collapse_outside_joint_rms_linear_residual",
                    "minimum",
                ),
                (
                    "maximum_outside_impact_unbroken_joint_angular_residual_radians",
                    "minimum_collapse_outside_joint_maximum_angular_residual_radians",
                    "minimum",
                ),
                (
                    "rms_outside_impact_unbroken_joint_angular_residual_radians",
                    "minimum_collapse_outside_joint_rms_angular_residual_radians",
                    "minimum",
                ),
            )
        for outcome_key, oracle_key, relation in predicates:
            _paper_threshold_number(
                errors,
                outcome=outcome,
                oracle=oracle,
                outcome_key=outcome_key,
                oracle_key=oracle_key,
                label=label,
                relation=relation,
            )
        impact_counts = outcome.get("broken_joint_impact_region_counts")
        minimum_impact = _finite_number(
            oracle.get("minimum_initial_broken_joints_per_impact_region")
        )
        if (
            not isinstance(impact_counts, list)
            or len(impact_counts) != 3
            or minimum_impact is None
            or any(
                _finite_number(value) is None or _finite_number(value) < minimum_impact
                for value in impact_counts
            )
        ):
            errors.append(
                f"{label}.broken_joint_impact_region_counts must satisfy all "
                "three outcome-oracle minima"
            )
        if outcome.get("broken_joints_outside_impact_regions") != 0:
            errors.append(f"{label}.broken_joints_outside_impact_regions must be zero")
        if outcome.get("checkpoint") == "collapse":
            displaced = outcome.get("impact_band_displaced_counts")
            minimum = _finite_number(
                oracle.get("minimum_displaced_bricks_per_impact_band")
            )
            if (
                not isinstance(displaced, list)
                or len(displaced) != 3
                or minimum is None
                or any(
                    _finite_number(value) is None or _finite_number(value) < minimum
                    for value in displaced
                )
            ):
                errors.append(
                    f"{label}.impact_band_displaced_counts must satisfy all "
                    "three collapse minima"
                )
    return errors


def _paper_capture_consistency_errors(
    packet: Mapping[str, object],
    packet_name: str,
    spec: Mapping[str, object],
) -> list[str]:
    errors: list[str] = []
    visual = packet.get("visual_evidence")
    if not isinstance(visual, Mapping):
        return [f"{packet_name}: visual_evidence must be an object"]
    benchmark = packet.get("benchmark")
    fingerprint = (
        benchmark.get("scene_spec_fingerprint")
        if isinstance(benchmark, Mapping)
        else None
    )
    if (
        not isinstance(fingerprint, str)
        or len(fingerprint) != _PAPER_FINGERPRINT_HEX_LENGTH
        or any(character not in "0123456789abcdef" for character in fingerprint)
    ):
        errors.append(
            f"{packet_name}: benchmark.scene_spec_fingerprint must be a "
            "16-character lowercase hexadecimal value"
        )

    captures = spec["captures"]
    assert isinstance(captures, Mapping)
    requires_long_horizon = _paper_requires_long_horizon(packet)
    reference_oracle: Mapping[str, object] | None = None
    reference_oracle_role: str | None = None
    checkpoint_frames = sorted(
        {
            raw_capture_spec["frame"]
            for raw_capture_spec in captures.values()
            if isinstance(raw_capture_spec, Mapping)
        }
    )
    event_prefixes: dict[str, Mapping[str, object]] = {}
    for role, raw_capture_spec in captures.items():
        assert isinstance(role, str)
        assert isinstance(raw_capture_spec, Mapping)
        capture_spec = raw_capture_spec
        if capture_spec.get("long_horizon") is True and not requires_long_horizon:
            continue
        label = f"{packet_name}: visual_evidence.{role}"
        capture = visual.get(role)
        if not isinstance(capture, Mapping):
            errors.append(f"{label} must be an object")
            continue
        if not _json_values_equal_exact(capture.get("label"), role):
            errors.append(f"{label}.label must be {role!r}")
        expected_frame = capture_spec["frame"]
        capture_config = capture.get("capture")
        if not isinstance(capture_config, Mapping):
            errors.append(f"{label}.capture must be an object")
        else:
            for key in ("requested_frames", "converted_frames"):
                if not _json_values_equal_exact(
                    capture_config.get(key), expected_frame
                ):
                    errors.append(f"{label}.capture.{key} must be {expected_frame}")
        errors.extend(
            _paper_capture_artifact_errors(
                capture,
                expected_frame=expected_frame,
                label=label,
                role=role,
                scene=spec["scene"],
            )
        )
        event_summary = capture.get("scene_metrics_events")
        if not isinstance(event_summary, Mapping):
            errors.append(f"{label}.scene_metrics_events must be an object")
        else:
            expected_event_keys = {
                "event_count",
                "file",
                "prefix_sha256",
                "sha256",
            }
            if set(event_summary) != expected_event_keys:
                errors.append(
                    f"{label}.scene_metrics_events must contain exactly "
                    f"{sorted(expected_event_keys)!r}"
                )
            if not _json_values_equal_exact(
                event_summary.get("event_count"), expected_frame
            ):
                errors.append(
                    f"{label}.scene_metrics_events.event_count must be "
                    f"{expected_frame}"
                )
            if event_summary.get("file") != "scene_metrics.jsonl":
                errors.append(
                    f"{label}.scene_metrics_events.file must be "
                    "'scene_metrics.jsonl'"
                )
            if not _is_lowercase_sha256(event_summary.get("sha256")):
                errors.append(f"{label}.scene_metrics_events.sha256 must be SHA-256")
            prefixes = event_summary.get("prefix_sha256")
            expected_prefix_keys = {
                str(frame) for frame in checkpoint_frames if frame <= expected_frame
            }
            if (
                not isinstance(prefixes, Mapping)
                or set(prefixes) != expected_prefix_keys
            ):
                errors.append(
                    f"{label}.scene_metrics_events.prefix_sha256 must contain "
                    f"exactly {sorted(expected_prefix_keys)!r}"
                )
            else:
                for frame, digest in prefixes.items():
                    if not _is_lowercase_sha256(digest):
                        errors.append(
                            f"{label}.scene_metrics_events.prefix_sha256[{frame!r}] "
                            "must be SHA-256"
                        )
                event_prefixes[role] = prefixes
        metrics = capture.get("scene_metrics")
        if not isinstance(metrics, Mapping):
            errors.append(f"{label}.scene_metrics must be an object")
            continue
        for key in ("event_count", "frame"):
            if not _json_values_equal_exact(metrics.get(key), expected_frame):
                errors.append(f"{label}.scene_metrics.{key} must be {expected_frame}")
        if metrics.get("scene_spec_fingerprint") != fingerprint:
            errors.append(
                f"{label}.scene_metrics.scene_spec_fingerprint must match "
                "benchmark.scene_spec_fingerprint"
            )

        scene_contract = metrics.get("scene_contract")
        if not isinstance(scene_contract, Mapping):
            errors.append(f"{label}.scene_metrics.scene_contract must be an object")
        else:
            if scene_contract.get("effective_scene_contract_passed") is not True:
                errors.append(
                    f"{label}.scene_metrics.scene_contract."
                    "effective_scene_contract_passed must be true"
                )
            if not _json_values_equal_exact(
                scene_contract.get("rigid_body_solver"), spec["public_solver"]
            ):
                errors.append(
                    f"{label}.scene_metrics.scene_contract.rigid_body_solver "
                    "contradicts the filename-specific solver"
                )
            if not _json_values_equal_exact(
                scene_contract.get("solver"), spec["scene_solver"]
            ):
                errors.append(
                    f"{label}.scene_metrics.scene_contract.solver contradicts "
                    "the filename-specific solver"
                )
            options = scene_contract.get("rigid_constraint_options")
            if not isinstance(options, Mapping) or not _json_values_equal_exact(
                options.get("iterations"), 20
            ):
                errors.append(
                    f"{label}.scene_metrics.scene_contract rigid constraint "
                    "iterations must be 20"
                )

        resolved = metrics.get("resolved_configuration")
        if not isinstance(resolved, list):
            errors.append(
                f"{label}.scene_metrics.resolved_configuration must be a list"
            )
        else:
            default_reason = "as requested"
            pair_reason = (
                "hard public rigid pair constraints use solver-owned "
                "sequential-impulse rows"
                if packet_name == "avbd-paper-sequential-impulse-comparison-packet.json"
                else default_reason
            )
            expected_notes = {
                "rigid-body": (
                    spec["capture_solver"],
                    spec["capture_solver"],
                    default_reason,
                ),
                "rigid-contact": (
                    spec["capture_solver"],
                    spec["capture_solver"],
                    default_reason,
                ),
                "rigid-pair-constraint": (
                    spec["capture_solver"],
                    spec["capture_solver"],
                    pair_reason,
                ),
                "rigid-constraint-iterations": ("20", "20", default_reason),
                "multibody": (
                    "semi-implicit",
                    "semi-implicit",
                    default_reason,
                ),
                "deformable-inner-solver": (
                    "inactive",
                    "inactive",
                    "no deformable bodies configured",
                ),
                "deformable-psd": ("cpu", "cpu", default_reason),
            }
            if spec["capture_solver"] == "avbd":
                # World.step records the immutable public AVBD parameter
                # profile whenever the rigid solver family is AVBD.
                expected_notes["rigid-avbd-parameter-profile"] = (
                    "paper-2025-table-2",
                    "paper-2025-table-2",
                    "immutable public AVBD parameters: "
                    "alpha=0.95, beta=10, gamma=0.99",
                )
            if len(resolved) != len(expected_notes):
                errors.append(
                    f"{label}.scene_metrics.resolved_configuration must contain "
                    f"exactly the {len(expected_notes)} filename-specific notes"
                )
            for domain, (requested, actual, reason) in expected_notes.items():
                domain_notes = [
                    note
                    for note in resolved
                    if isinstance(note, Mapping) and note.get("domain") == domain
                ]
                matches = [
                    note
                    for note in domain_notes
                    if note.get("requested") == requested
                    and note.get("resolved") == actual
                    and note.get("reason") == reason
                ]
                if len(domain_notes) != 1 or len(matches) != 1:
                    errors.append(
                        f"{label}.scene_metrics.resolved_configuration must "
                        f"contain exactly one {domain} {requested}->{actual} "
                        f"note with reason {reason!r}"
                    )
            unexpected_notes = [
                index
                for index, note in enumerate(resolved)
                if not isinstance(note, Mapping)
                or note.get("domain") not in expected_notes
            ]
            if unexpected_notes:
                errors.append(
                    f"{label}.scene_metrics.resolved_configuration contains "
                    f"unexpected notes at indices {unexpected_notes!r}"
                )

        oracle = metrics.get("outcome_oracle")
        outcome = metrics.get("outcome")
        if not isinstance(oracle, Mapping):
            errors.append(f"{label}.scene_metrics.outcome_oracle must be an object")
            continue
        expected_oracle = PAPER_OUTCOME_ORACLES.get(packet_name, {}).get(role)
        if expected_oracle is None:
            errors.append(
                f"{label}.scene_metrics.outcome_oracle has no checker-owned "
                "oracle for this checkpoint"
            )
        else:
            for oracle_key, oracle_value in expected_oracle.items():
                if not _json_values_equal_exact(oracle.get(oracle_key), oracle_value):
                    errors.append(
                        f"{label}.scene_metrics.outcome_oracle.{oracle_key} must "
                        f"be {oracle_value!r}; the packet cannot choose the "
                        "threshold it is graded against"
                    )
        if reference_oracle is None:
            reference_oracle = oracle
            reference_oracle_role = role
        elif not _json_values_equal_exact(oracle, reference_oracle):
            errors.append(
                f"{label}.scene_metrics.outcome_oracle must exactly match "
                f"visual_evidence.{reference_oracle_role}.scene_metrics."
                "outcome_oracle"
            )
        if not isinstance(outcome, Mapping):
            errors.append(f"{label}.scene_metrics.outcome must be an object")
            continue
        evaluation_key = capture_spec["evaluation_key"]
        if not _json_values_equal_exact(
            oracle.get(evaluation_key), capture_spec["evaluation_frame"]
        ):
            errors.append(
                f"{label}.scene_metrics.outcome_oracle.{evaluation_key} must be "
                f"{capture_spec['evaluation_frame']}"
            )
        for key in ("frame", "checkpoint", "evaluated", "status", "thresholds_pass"):
            expected = capture_spec[key]
            if not _json_values_equal_exact(outcome.get(key), expected):
                errors.append(
                    f"{label}.scene_metrics.outcome.{key} must be {expected!r}"
                )
        if not _json_values_equal_exact(outcome.get("last_step_iterations"), 20):
            errors.append(
                f"{label}.scene_metrics.outcome.last_step_iterations must be 20"
            )
        expected_time = float(expected_frame) / 60.0
        if not _numbers_match(outcome.get("world_time"), expected_time):
            errors.append(
                f"{label}.scene_metrics.outcome.world_time must equal frame / 60"
            )
        expected_checks = {key: True for key in capture_spec["threshold_checks"]}
        if not _json_values_equal_exact(
            outcome.get("threshold_checks"), expected_checks
        ):
            errors.append(
                f"{label}.scene_metrics.outcome.threshold_checks must exactly "
                "match the filename-specific passing checkpoint predicates"
            )
        errors.extend(
            _paper_outcome_threshold_errors(packet_name, role, outcome, oracle)
        )
    for frame in checkpoint_frames:
        matching = [
            prefixes[str(frame)]
            for prefixes in event_prefixes.values()
            if str(frame) in prefixes
        ]
        if len(matching) > 1 and len(set(matching)) != 1:
            errors.append(
                f"{packet_name}: independent captures must have an exact shared "
                f"scene-metric event prefix through frame {frame}"
            )
    if requires_long_horizon:
        review = visual.get("semantic_review")
        if not isinstance(review, Mapping):
            errors.append(
                f"{packet_name}: visual_evidence.semantic_review must be an object"
            )
        else:
            expected_review_keys = {
                "assessment_assertions",
                "claim_assessments",
                "file",
                "inspected_images",
                "inspected_videos",
                "reviewer_capabilities",
                "sha256",
                "structured_observations",
                "temporal_assessment",
                "verdict",
            }
            if set(review) != expected_review_keys:
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review must contain "
                    f"exactly {sorted(expected_review_keys)!r}"
                )
            if review.get("verdict") != "pass":
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review.verdict "
                    "must be 'pass'"
                )
            expected_capabilities = {
                "image_semantic_review": True,
                "video_semantic_review": True,
            }
            if not _json_values_equal_exact(
                review.get("reviewer_capabilities"), expected_capabilities
            ):
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review."
                    "reviewer_capabilities must positively attest image and video review"
                )
            expected_assertions = {
                "capture_images_assessed": True,
                "long_horizon_video_assessed": True,
                "no_contradictions_found": True,
                "paper_reference_assessed": True,
                "text_oracle_agrees": True,
                "view_reports_agree": True,
            }
            if not _json_values_equal_exact(
                review.get("assessment_assertions"), expected_assertions
            ):
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review."
                    "assessment_assertions must be the exact positive contract"
                )
            if not _json_values_equal_exact(
                review.get("claim_assessments"),
                SEMANTIC_CLAIM_ASSESSMENTS_BY_TERMINAL_BEHAVIOR[
                    SEMANTIC_TERMINAL_BEHAVIOR[packet_name]
                ],
            ):
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review."
                    "claim_assessments must use the exact authoritative "
                    "supported/not_proven contract"
                )
            expected_temporal_assessment = {
                "checkpoint_sequence_agrees": True,
                "full_interval_viewed": True,
                "still_frames_only": False,
                "terminal_behavior": SEMANTIC_TERMINAL_BEHAVIOR[packet_name],
            }
            if not _json_values_equal_exact(
                review.get("temporal_assessment"), expected_temporal_assessment
            ):
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review."
                    "temporal_assessment must bind full-interval review and "
                    "the filename-specific terminal behavior"
                )
            expected_observations = SEMANTIC_STRUCTURED_OBSERVATIONS[
                SEMANTIC_TERMINAL_BEHAVIOR[packet_name]
            ]
            if not _json_values_equal_exact(
                review.get("structured_observations"), expected_observations
            ):
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review."
                    "structured_observations must be the exact authoritative "
                    "checkpoint/paper/oracle/ViewReport contract"
                )
            entries = review.get("inspected_images")
            if not isinstance(entries, list):
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review."
                    "inspected_images must be a list"
                )
            else:
                expected_image_roles = {
                    raw_capture_spec["review_role"]
                    for raw_capture_spec in captures.values()
                    if isinstance(raw_capture_spec, Mapping)
                }
                expected_image_roles.add("paper_figure_13_reference")
                actual_image_roles = [
                    entry.get("role") for entry in entries if isinstance(entry, Mapping)
                ]
                if (
                    len(actual_image_roles) != len(entries)
                    or set(actual_image_roles) != expected_image_roles
                    or len(actual_image_roles) != len(expected_image_roles)
                ):
                    errors.append(
                        f"{packet_name}: visual_evidence.semantic_review must inspect "
                        "the exact capture-role set plus paper_figure_13_reference"
                    )
                for role, raw_capture_spec in captures.items():
                    assert isinstance(role, str)
                    assert isinstance(raw_capture_spec, Mapping)
                    review_role = raw_capture_spec["review_role"]
                    matches = [
                        entry
                        for entry in entries
                        if isinstance(entry, Mapping)
                        and entry.get("role") == review_role
                    ]
                    capture = visual.get(role)
                    screenshot = (
                        capture.get("screenshot")
                        if isinstance(capture, Mapping)
                        else None
                    )
                    if len(matches) != 1:
                        errors.append(
                            f"{packet_name}: visual_evidence.semantic_review must "
                            f"inspect {review_role!r} exactly once"
                        )
                        continue
                    if not isinstance(screenshot, Mapping):
                        continue
                    entry = matches[0]
                    for key in ("file", "sha256"):
                        if not _json_values_equal_exact(
                            entry.get(key), screenshot.get(key)
                        ):
                            errors.append(
                                f"{packet_name}: visual_evidence.semantic_review "
                                f"{review_role}.{key} must match "
                                f"visual_evidence.{role}.screenshot.{key}"
                            )
                paper_reference = packet.get("paper_reference")
                paper_figure = (
                    paper_reference.get("figure")
                    if isinstance(paper_reference, Mapping)
                    else None
                )
                paper_matches = [
                    entry
                    for entry in entries
                    if isinstance(entry, Mapping)
                    and entry.get("role") == "paper_figure_13_reference"
                ]
                if len(paper_matches) != 1 or not isinstance(paper_figure, Mapping):
                    errors.append(
                        f"{packet_name}: visual_evidence.semantic_review must bind "
                        "the paper Figure 13 reference exactly once"
                    )
                else:
                    for key in ("file", "sha256"):
                        if not _json_values_equal_exact(
                            paper_matches[0].get(key), paper_figure.get(key)
                        ):
                            errors.append(
                                f"{packet_name}: visual_evidence.semantic_review "
                                f"paper_figure_13_reference.{key} must match "
                                f"paper_reference.figure.{key}"
                            )
            video_entries = review.get("inspected_videos")
            long_capture = visual.get(PAPER_LONG_HORIZON_ROLE)
            long_artifacts = (
                long_capture.get("artifact_provenance")
                if isinstance(long_capture, Mapping)
                else None
            )
            long_video = (
                long_artifacts.get("video")
                if isinstance(long_artifacts, Mapping)
                else None
            )
            if not isinstance(video_entries, list) or len(video_entries) != 1:
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review must inspect "
                    "exactly one long-horizon video"
                )
            elif not isinstance(video_entries[0], Mapping) or not isinstance(
                long_video, Mapping
            ):
                errors.append(
                    f"{packet_name}: visual_evidence.semantic_review long-horizon "
                    "video binding is invalid"
                )
            else:
                video_entry = video_entries[0]
                expected_video_entry = {
                    "decoded_frame_count": long_video.get("decoded_frame_count"),
                    "duration_seconds": long_video.get("duration_seconds"),
                    "file": long_video.get("file"),
                    "role": "long_horizon_video_600",
                    "sha256": long_video.get("sha256"),
                }
                if not _json_values_equal_exact(video_entry, expected_video_entry):
                    errors.append(
                        f"{packet_name}: visual_evidence.semantic_review inspected "
                        "video must bind the exact long-horizon MP4 bytes"
                    )
    return errors


def _paper_figure13_consistency_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    """Reconcile every durable Figure 13 identity and checkpoint surface."""
    spec = PAPER_FIGURE13_SPECS.get(packet_name)
    if spec is None:
        return []

    errors: list[str] = []
    for key in ("packet", "scene"):
        if packet.get(key) != spec[key]:
            errors.append(f"{packet_name}: {key} must be {spec[key]!r}")

    identity = packet.get("resolved_solver_identity")
    if not isinstance(identity, Mapping):
        errors.append(f"{packet_name}: resolved_solver_identity must be an object")
    else:
        expected_identity = [
            ("rigid_contact_solver", spec["identity_solver"]),
            ("rigid_point_joint_solver", spec["identity_solver"]),
            ("rigid_contact_selection", spec["selection"]),
            ("avbd_rigid_contact_config_emplaced", False),
        ]
        version = packet.get("schema_version")
        if (
            isinstance(version, int)
            and version >= MULTIBODY_IDENTITY_MIN_SCHEMA_VERSION
        ) or "multibody_integration_family" in identity:
            expected_identity.append(("multibody_integration_family", "none"))
        for key, expected in expected_identity:
            if identity.get(key) != expected:
                errors.append(
                    f"{packet_name}: resolved_solver_identity.{key} must be "
                    f"{expected!r} for this Figure 13 filename"
                )

    correctness = packet.get("correctness")
    public_configuration = (
        correctness.get("public_configuration")
        if isinstance(correctness, Mapping)
        else None
    )
    benchmark = packet.get("benchmark")
    fingerprint = (
        benchmark.get("scene_spec_fingerprint")
        if isinstance(benchmark, Mapping)
        else None
    )
    if not isinstance(public_configuration, Mapping):
        errors.append(
            f"{packet_name}: correctness.public_configuration must be an object"
        )
    else:
        if public_configuration.get("rigid_body_solver") != spec["public_solver"]:
            errors.append(
                f"{packet_name}: correctness.public_configuration.rigid_body_solver "
                "contradicts the filename-specific solver"
            )
        if public_configuration.get("scene_spec_fingerprint") != fingerprint:
            errors.append(
                f"{packet_name}: correctness.public_configuration."
                "scene_spec_fingerprint must match benchmark.scene_spec_fingerprint"
            )
        options = public_configuration.get("rigid_constraint_options")
        if not isinstance(options, Mapping) or options.get("iterations") != 20:
            errors.append(
                f"{packet_name}: correctness.public_configuration rigid "
                "constraint iterations must be 20"
            )
    required_link = {
        "avbd-paper-vbd-comparison-packet.json": "linked_avbd_evidence",
        "avbd-paper-sequential-impulse-comparison-packet.json": (
            "linked_avbd_vbd_evidence"
        ),
    }.get(packet_name)
    if required_link is not None and not isinstance(packet.get(required_link), Mapping):
        errors.append(f"{packet_name}: {required_link} must be an object")
    errors.extend(_paper_capture_consistency_errors(packet, packet_name, spec))
    return errors


def _historical_high_ratio_boundary_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    """Keep unavailable legacy high-ratio media and timings claim-bounded."""
    spec = HISTORICAL_HIGH_RATIO_BOUNDARIES.get(packet_name)
    if spec is None:
        return []
    boundary = packet.get("evidence_boundary")
    if not isinstance(boundary, Mapping):
        return [f"{packet_name}: evidence_boundary must be an object"]
    expected: dict[str, object] = {
        "current_build_bound": False,
        "measurement_runtime_identity_recorded": False,
        "plan104_avbd_row_closure_supported": False,
        "avbd_solver_evidence": False,
        "avbd_performance_claim_supported": False,
        "supported_scope": spec["supported_scope"],
    }
    if spec["visual_boundary"]:
        expected.update(
            {
                "capture_artifacts_accessible": False,
                "semantic_visual_review_recorded": False,
            }
        )
    errors: list[str] = []
    for key, value in expected.items():
        if not _json_values_equal_exact(boundary.get(key), value):
            errors.append(f"{packet_name}: evidence_boundary.{key} must be {value!r}")
    return errors


def _legacy_non_evidence_boundary_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    """Require dangerous legacy packets to carry their own claim boundary."""
    supported_scope = LEGACY_NON_EVIDENCE_BOUNDARY_SCOPES.get(packet_name)
    version = packet.get("schema_version")
    if supported_scope is None or not isinstance(version, int):
        return []
    if version >= PLAN104_CLAIMS_MIN_SCHEMA_VERSION:
        return []

    boundary = packet.get("evidence_boundary")
    if not isinstance(boundary, Mapping):
        return [
            f"{packet_name}: legacy schema_version {version} requires an "
            "evidence_boundary object"
        ]
    expected: dict[str, object] = {
        "artifact_status": "legacy_unbound",
        "avbd_performance_claim_supported": False,
        "avbd_solver_evidence": False,
        "current_build_bound": False,
        "historical_identifiers_retained": True,
        "historical_measurements_preserved": True,
        "measurement_runtime_identity_recorded": False,
        "plan104_avbd_row_closure_supported": False,
        "supported_scope": supported_scope,
    }
    errors: list[str] = []
    for key, value in expected.items():
        if not _json_values_equal_exact(boundary.get(key), value):
            errors.append(f"{packet_name}: evidence_boundary.{key} must be {value!r}")
    reason = boundary.get("reason")
    if not isinstance(reason, str) or not reason.strip():
        errors.append(
            f"{packet_name}: evidence_boundary.reason must be a non-empty string"
        )
    return errors


def _packet_target(
    path: Path,
    context: PacketValidationContext,
) -> tuple[Path | None, tuple[str, ...]]:
    try:
        path_text = str(path)
        path_text.encode("utf-8")
    except TypeError, ValueError, UnicodeError:
        return None, ("packet target path must be UTF-8-encodable text",)
    if "\x00" in path_text:
        return None, ("packet target path must not contain NUL",)
    name = path.name
    if context.initialization_error is not None:
        return None, (f"{name}: {context.initialization_error}",)
    try:
        if path.is_symlink():
            return None, (f"{name}: packet target cannot be a symbolic link",)
        resolved = path.resolve()
    except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
        return None, (f"{name}: packet target cannot be resolved ({exc})",)
    if resolved.parent != context.packet_dir:
        return None, (
            f"{name}: packet target must resolve directly under "
            f"{context.packet_dir}, got {resolved}",
        )
    try:
        is_file = resolved.is_file()
    except (OSError, ValueError, UnicodeError) as exc:
        return None, (f"{name}: packet target cannot be inspected ({exc})",)
    if not is_file:
        return None, (f"{name}: packet target must be a regular file: {resolved}",)
    return resolved, ()


def _load_packet(
    path: Path,
    context: PacketValidationContext,
) -> PacketLoadResult:
    cached = context.loaded.get(path)
    if cached is not None:
        return cached

    name = path.name
    try:
        payload = path.read_bytes()
    except (OSError, ValueError, UnicodeError) as exc:
        result = PacketLoadResult(
            packet=None,
            errors=(f"{name}: packet cannot be read ({exc})",),
            payload=None,
            sha256=None,
        )
    else:
        digest = hashlib.sha256(payload).hexdigest()
        try:
            packet = _strict_json_loads(payload)
        except (OverflowError, UnicodeError, ValueError) as exc:
            result = PacketLoadResult(
                packet=None,
                errors=(f"{name}: invalid JSON ({exc})",),
                payload=payload,
                sha256=digest,
            )
        else:
            if not isinstance(packet, dict):
                result = PacketLoadResult(
                    packet=None,
                    errors=(f"{name}: packet must be a JSON object",),
                    payload=payload,
                    sha256=digest,
                )
            else:
                result = PacketLoadResult(
                    packet=packet,
                    errors=(),
                    payload=payload,
                    sha256=digest,
                )
    context.loaded[path] = result
    return result


def _paper_visual_summary(
    packet: Mapping[str, object],
    first_role: str,
    second_role: str,
    *,
    include_review_file: bool,
) -> dict[str, object]:
    visual = packet.get("visual_evidence")
    if not isinstance(visual, Mapping):
        return {}

    def screenshot(role: str) -> object:
        capture = visual.get(role)
        value = capture.get("screenshot") if isinstance(capture, Mapping) else None
        if not isinstance(value, Mapping):
            return None
        return {key: value.get(key) for key in ("file", "sha256")}

    review = visual.get("semantic_review")
    if isinstance(review, Mapping):
        review_keys = (
            ("file", "sha256", "verdict")
            if include_review_file
            else (
                "sha256",
                "verdict",
            )
        )
        review_summary: object = {key: review.get(key) for key in review_keys}
    else:
        review_summary = None
    summary = {
        f"{first_role}_screenshot": screenshot(first_role),
        f"{second_role}_screenshot": screenshot(second_role),
        "semantic_review": review_summary,
    }
    if isinstance(visual.get(PAPER_LONG_HORIZON_ROLE), Mapping):
        summary["long_horizon_screenshot"] = screenshot(PAPER_LONG_HORIZON_ROLE)
    return summary


def _paper_link_summary_errors(
    packet: Mapping[str, object],
    packet_name: str,
    key: str,
    link: Mapping[str, object],
    linked_packet: Mapping[str, object],
    linked_path: Path,
    context: PacketValidationContext,
) -> list[str]:
    """Bind copied Figure 13 summaries to the parsed linked packet chain."""
    errors: list[str] = []
    label = f"{packet_name}: {key}"
    own_benchmark = packet.get("benchmark")
    linked_benchmark_common = linked_packet.get("benchmark")
    own_run_evidence = (
        own_benchmark.get("run_evidence")
        if isinstance(own_benchmark, Mapping)
        else None
    )
    linked_run_evidence = (
        linked_benchmark_common.get("run_evidence")
        if isinstance(linked_benchmark_common, Mapping)
        else None
    )
    if not _json_values_equal_exact(own_run_evidence, linked_run_evidence):
        errors.append(
            f"{label}: linked packet must use the identical benchmark "
            "run/host/build evidence"
        )
    if (
        packet_name == "avbd-paper-vbd-comparison-packet.json"
        and key == "linked_avbd_evidence"
    ):
        if linked_path.name != "avbd-paper-breakable-wall-packet.json":
            return [f"{label}.file must be 'avbd-paper-breakable-wall-packet.json'"]
        expected_keys = {
            "file",
            "resolved_solver_identity",
            "scene_spec_fingerprint",
            "sha256",
            "visual_evidence",
        }
        if set(link) != expected_keys:
            errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
        linked_benchmark = linked_packet.get("benchmark")
        expected_fingerprint = (
            linked_benchmark.get("scene_spec_fingerprint")
            if isinstance(linked_benchmark, Mapping)
            else None
        )
        for field_name, expected in (
            ("resolved_solver_identity", linked_packet.get("resolved_solver_identity")),
            ("scene_spec_fingerprint", expected_fingerprint),
            (
                "visual_evidence",
                _paper_visual_summary(
                    linked_packet,
                    "impact",
                    "outcome",
                    include_review_file=True,
                ),
            ),
        ):
            if not _json_values_equal_exact(link.get(field_name), expected):
                errors.append(
                    f"{label}.{field_name} must match the parsed linked AVBD packet"
                )
        return errors

    if (
        packet_name != "avbd-paper-sequential-impulse-comparison-packet.json"
        or key != "linked_avbd_vbd_evidence"
    ):
        return []
    if linked_path.name != "avbd-paper-vbd-comparison-packet.json":
        return [f"{label}.file must be 'avbd-paper-vbd-comparison-packet.json'"]
    expected_keys = {
        "benchmark_method_timings",
        "file",
        "nested_avbd_resolved_solver_identity",
        "nested_avbd_sha256",
        "nested_avbd_source_provenance_digest",
        "resolved_solver_identity",
        "scene_spec_fingerprint",
        "sha256",
        "source_provenance_digest",
        "visual_evidence",
    }
    if set(link) != expected_keys:
        errors.append(f"{label} must contain exactly {sorted(expected_keys)!r}")
    linked_benchmark = linked_packet.get("benchmark")
    linked_fingerprint = (
        linked_benchmark.get("scene_spec_fingerprint")
        if isinstance(linked_benchmark, Mapping)
        else None
    )
    linked_provenance = linked_packet.get("source_provenance")
    linked_provenance_digest = (
        linked_provenance.get("digest")
        if isinstance(linked_provenance, Mapping)
        else None
    )
    for field_name, expected in (
        ("resolved_solver_identity", linked_packet.get("resolved_solver_identity")),
        ("scene_spec_fingerprint", linked_fingerprint),
        ("source_provenance_digest", linked_provenance_digest),
        (
            "visual_evidence",
            _paper_visual_summary(
                linked_packet,
                "bend",
                "retention",
                include_review_file=False,
            ),
        ),
    ):
        if not _json_values_equal_exact(link.get(field_name), expected):
            errors.append(
                f"{label}.{field_name} must match the parsed linked VBD packet"
            )

    _, linked_medians = _paper_benchmark_evidence(linked_packet, linked_path.name)
    expected_timings = {
        method_name: {_MEDIAN_CPU_TIME_KEY: median}
        for method_name, median in linked_medians.items()
    }
    if not _json_values_equal_exact(
        link.get("benchmark_method_timings"), expected_timings
    ):
        errors.append(
            f"{label}.benchmark_method_timings must be derived from the "
            "linked packet's embedded median aggregate rows"
        )

    nested_link = linked_packet.get("linked_avbd_evidence")
    if not isinstance(nested_link, Mapping):
        errors.append(f"{label} linked VBD packet lacks linked_avbd_evidence")
    else:
        for field_name, nested_field in (
            ("nested_avbd_resolved_solver_identity", "resolved_solver_identity"),
            ("nested_avbd_sha256", "sha256"),
        ):
            if not _json_values_equal_exact(
                link.get(field_name), nested_link.get(nested_field)
            ):
                errors.append(
                    f"{label}.{field_name} must match linked VBD's nested AVBD "
                    "summary"
                )
        nested_relative = _safe_relative_path(nested_link.get("file"))
        nested_packet: Mapping[str, object] | None = None
        if nested_relative is None or len(nested_relative.parts) != 1:
            errors.append(f"{label} linked VBD nested AVBD file is unsafe")
        else:
            nested_path, target_errors = _packet_target(
                linked_path.parent / nested_relative, context
            )
            if nested_path is None:
                errors.extend(f"{label}: {error}" for error in target_errors)
            else:
                nested_packet = _load_packet(nested_path, context).packet
        nested_provenance = (
            nested_packet.get("source_provenance")
            if isinstance(nested_packet, Mapping)
            else None
        )
        nested_digest = (
            nested_provenance.get("digest")
            if isinstance(nested_provenance, Mapping)
            else None
        )
        if link.get("nested_avbd_source_provenance_digest") != nested_digest:
            errors.append(
                f"{label}.nested_avbd_source_provenance_digest must match the "
                "parsed nested AVBD packet"
            )
        nested_benchmark = (
            nested_packet.get("benchmark")
            if isinstance(nested_packet, Mapping)
            else None
        )
        nested_run_evidence = (
            nested_benchmark.get("run_evidence")
            if isinstance(nested_benchmark, Mapping)
            else None
        )
        if not _json_values_equal_exact(own_run_evidence, nested_run_evidence):
            errors.append(
                f"{label}: nested AVBD packet must use the identical benchmark "
                "run/host/build evidence"
            )

    _, self_medians = _paper_benchmark_evidence(packet, packet_name)
    numerator = self_medians.get("sequential_impulse")
    comparison = (
        packet.get("benchmark", {}).get("comparison")
        if isinstance(packet.get("benchmark"), Mapping)
        else None
    )
    if isinstance(comparison, Mapping) and numerator is not None:
        for denominator_name in ("avbd", "vbd"):
            denominator = linked_medians.get(denominator_name)
            ratio_key = (
                f"sequential_impulse_to_{denominator_name}" f"{_MEDIAN_RATIO_SUFFIX}"
            )
            actual = _finite_number(comparison.get(ratio_key))
            if denominator is None or actual is None:
                errors.append(
                    f"{label}: benchmark.comparison.{ratio_key} cannot be "
                    "derived from embedded aggregate rows"
                )
                continue
            expected = _finite_ratio(numerator, denominator)
            if expected is None:
                errors.append(
                    f"{label}: benchmark.comparison.{ratio_key} cannot be "
                    "represented as a positive finite ratio"
                )
                continue
            if not math.isclose(
                actual,
                expected,
                rel_tol=_MEDIAN_RATIO_RELATIVE_TOLERANCE,
                abs_tol=0.0,
            ):
                errors.append(
                    f"{label}: benchmark.comparison.{ratio_key} is {actual!r} "
                    f"but embedded aggregate rows record {expected!r}"
                )
    return errors


def _linked_packet_errors(
    packet: Mapping[str, object],
    path: Path,
    context: PacketValidationContext,
) -> list[str]:
    errors: list[str] = []
    for key in LINKED_PACKET_KEYS:
        link = packet.get(key)
        if link is None:
            continue
        if not isinstance(link, Mapping):
            errors.append(f"{path.name}: {key} must be an object")
            continue
        relative = _safe_relative_path(link.get("file"))
        if relative is None or len(relative.parts) != 1:
            errors.append(f"{path.name}: {key}.file must name one sibling packet file")
            continue
        linked_path, target_errors = _packet_target(path.parent / relative, context)
        if linked_path is None:
            errors.extend(f"{path.name}: {key}: {error}" for error in target_errors)
            continue
        loaded = _load_packet(linked_path, context)
        if loaded.sha256 is None:
            errors.append(
                f"{path.name}: {key}.file cannot be read: " f"{relative.as_posix()}"
            )
        elif link.get("sha256") != loaded.sha256:
            errors.append(
                f"{path.name}: {key}.sha256 drifted for {relative.as_posix()}"
            )
        linked_packet = loaded.packet
        expected_provenance_digest = link.get("source_provenance_digest")
        if expected_provenance_digest is not None and linked_packet is not None:
            linked_provenance = linked_packet.get("source_provenance")
            actual_provenance_digest = (
                linked_provenance.get("digest")
                if isinstance(linked_provenance, Mapping)
                else None
            )
            if expected_provenance_digest != actual_provenance_digest:
                errors.append(
                    f"{path.name}: {key}.source_provenance_digest does not "
                    f"match {relative.as_posix()}"
                )
        if linked_packet is not None:
            errors.extend(
                _paper_link_summary_errors(
                    packet,
                    path.name,
                    key,
                    link,
                    linked_packet,
                    linked_path,
                    context,
                )
            )
        errors.extend(_packet_errors(linked_path, context))
    return errors


def _packet_errors(path: Path, context: PacketValidationContext) -> list[str]:
    resolved, target_errors = _packet_target(path, context)
    if resolved is None:
        return list(target_errors)
    cached_errors = context.artifact_errors.get(resolved)
    if cached_errors is not None:
        return list(cached_errors)
    if resolved in context.in_progress:
        return [f"{resolved.name}: linked packet cycle detected"]
    context.in_progress.add(resolved)
    try:
        name = resolved.name
        loaded = _load_packet(resolved, context)
        packet = loaded.packet
        if packet is None:
            errors = list(loaded.errors)
        else:
            errors = packet_schema_version_errors(packet, name)
            if not errors:
                version = packet["schema_version"]
                if version < AVBD_PACKET_SCHEMA_VERSION:
                    expected_legacy_version = LEGACY_PACKET_SCHEMA_VERSIONS.get(name)
                    if expected_legacy_version is None:
                        errors.append(
                            f"{name}: new AVBD packets must be written at "
                            f"schema_version {AVBD_PACKET_SCHEMA_VERSION} with a "
                            "recorded resolved_solver_identity (legacy allowlist "
                            "covers only packets committed before the current "
                            "schema contract)"
                        )
                    elif version != expected_legacy_version:
                        errors.append(
                            f"{name}: legacy allowlist requires schema_version "
                            f"{expected_legacy_version}, got {version}; otherwise "
                            "regenerate the packet at current schema_version "
                            f"{AVBD_PACKET_SCHEMA_VERSION}"
                        )
                errors.extend(resolved_solver_identity_errors(packet, name))
                errors.extend(plan104_claims_errors(packet, name))
                errors.extend(_source_provenance_errors(packet, name))
                errors.extend(_paper_capture_source_binding_errors(packet, name))
                errors.extend(_image_verdict_binding_errors(packet, name))
                errors.extend(_paper_image_verdict_binding_errors(packet, name))
                errors.extend(_paper_figure13_consistency_errors(packet, name))
                errors.extend(_legacy_non_evidence_boundary_errors(packet, name))
                errors.extend(_historical_high_ratio_boundary_errors(packet, name))
                errors.extend(_paper_benchmark_timing_errors(packet, name))
                errors.extend(_linked_packet_errors(packet, resolved, context))
    finally:
        context.in_progress.remove(resolved)

    result = tuple(errors)
    context.artifact_errors[resolved] = result
    return list(result)


_MEDIAN_CPU_TIME_KEY = "median_cpu_time_per_step_ns"
_MEDIAN_RATIO_SUFFIX = "_median_cpu_cost_ratio"
_MEDIAN_RATIO_RELATIVE_TOLERANCE = 1e-9


def _finite_positive_number(value: object) -> bool:
    number = _finite_number(value)
    return number is not None and number > 0.0


def _paper_benchmark_method_entry(
    benchmark: Mapping[str, object],
    method_name: str,
    method_spec: Mapping[str, object],
) -> object:
    location = method_spec["location"]
    if location == "root":
        return benchmark
    if location == "method":
        return benchmark.get("method")
    methods = benchmark.get("methods")
    return methods.get(method_name) if isinstance(methods, Mapping) else None


def _paper_benchmark_evidence(
    packet: Mapping[str, object],
    packet_name: str,
) -> tuple[list[str], dict[str, float]]:
    """Validate raw aggregate rows and derive their CPU medians."""
    spec = PAPER_FIGURE13_SPECS.get(packet_name)
    if spec is None:
        return [], {}
    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, Mapping):
        return [f"{packet_name}: benchmark must be an object"], {}

    errors: list[str] = []
    medians: dict[str, float] = {}
    requires_solver_configuration = (
        isinstance(packet.get("schema_version"), int)
        and packet["schema_version"] >= SOLVER_CONFIGURATION_MIN_SCHEMA_VERSION
    )
    fingerprint = benchmark.get("scene_spec_fingerprint")
    if (
        not isinstance(fingerprint, str)
        or len(fingerprint) != _PAPER_FINGERPRINT_HEX_LENGTH
        or any(character not in "0123456789abcdef" for character in fingerprint)
    ):
        errors.append(
            f"{packet_name}: benchmark.scene_spec_fingerprint must be a "
            "16-character lowercase hexadecimal value"
        )
        fingerprint_value = None
    else:
        fingerprint_value = int(fingerprint, 16)

    method_specs = spec["benchmark_methods"]
    assert isinstance(method_specs, Mapping)
    for method_name, raw_method_spec in method_specs.items():
        assert isinstance(method_name, str)
        assert isinstance(raw_method_spec, Mapping)
        method_spec = raw_method_spec
        location = method_spec["location"]
        location_label = (
            "benchmark"
            if location == "root"
            else (
                f"benchmark.{location}"
                if location == "method"
                else f"benchmark.methods.{method_name}"
            )
        )
        entry = _paper_benchmark_method_entry(benchmark, method_name, method_spec)
        if not isinstance(entry, Mapping):
            errors.append(f"{packet_name}: {location_label} must be an object")
            continue
        configuration_fingerprint_value = None
        if requires_solver_configuration:
            configuration_fingerprint = entry.get("solver_configuration_fingerprint")
            if (
                not isinstance(configuration_fingerprint, str)
                or len(configuration_fingerprint) != _PAPER_FINGERPRINT_HEX_LENGTH
                or any(
                    character not in "0123456789abcdef"
                    for character in configuration_fingerprint
                )
            ):
                errors.append(
                    f"{packet_name}: {location_label}."
                    "solver_configuration_fingerprint must be a 16-character "
                    "lowercase hexadecimal value"
                )
            else:
                configuration_fingerprint_value = int(configuration_fingerprint, 16)
        expected_benchmark = method_spec["benchmark"]
        if entry.get("benchmark") != expected_benchmark:
            errors.append(
                f"{packet_name}: {location_label}.benchmark must be "
                f"{expected_benchmark!r}"
            )
        rows = entry.get("rows")
        if not isinstance(rows, list):
            errors.append(f"{packet_name}: {location_label}.rows must be a list")
            continue
        expected_aggregates = ("mean", "median", "stddev", "cv")
        aggregate_names = [
            row.get("aggregate_name") if isinstance(row, Mapping) else None
            for row in rows
        ]
        if tuple(aggregate_names) != expected_aggregates:
            errors.append(
                f"{packet_name}: {location_label}.rows must contain exactly, "
                f"in order: {', '.join(expected_aggregates)}"
            )
            continue
        by_aggregate = dict(zip(expected_aggregates, rows))
        runtime_solver = method_spec["runtime_solver"]
        expected_run = f"{expected_benchmark}/iterations:120"
        stable_counter_keys = {
            *_PAPER_SCENE_COUNTERS,
            f"public_{runtime_solver}_family",
            f"resolved_rigid_body_{runtime_solver}",
            "resolved_rigid_constraint_iterations",
            f"resolved_rigid_contact_{runtime_solver}",
            f"resolved_rigid_pair_constraint_{runtime_solver}",
            "rigid_constraint_iterations",
            "runtime_contract_passed",
            "trajectory_frames",
            "scene_spec_fingerprint_hi",
            "scene_spec_fingerprint_lo",
        }
        if requires_solver_configuration:
            stable_counter_keys.update(
                {
                    "contact_method_sequential_impulse",
                    "effective_scene_contract_passed",
                    "effective_scene_mutation_audit_passed",
                    "rigid_avbd_alpha",
                    "rigid_avbd_beta",
                    "rigid_avbd_gamma",
                    "rigid_avbd_parameter_profile_paper_2025",
                    "runtime_identity_recorded",
                    "runtime_identity_applicable",
                    "runtime_identity_not_applicable",
                    "runtime_identity_public_avbd_rigid",
                    "runtime_identity_variational_multibody",
                    "runtime_identity_contract_passed",
                    "scene_spec_matches_python",
                    "solver_projection_policies_match",
                    "solver_configuration_fingerprint_hi",
                    "solver_configuration_fingerprint_lo",
                }
            )
        for aggregate, row in by_aggregate.items():
            assert isinstance(row, Mapping)
            row_label = f"{packet_name}: {location_label}.rows.{aggregate}"
            for key, expected in (
                ("aggregate_name", aggregate),
                ("run_type", "aggregate"),
                ("run_name", expected_run),
                ("repetitions", _PAPER_BENCHMARK_REPETITIONS),
                ("iterations", _PAPER_BENCHMARK_ITERATIONS),
                ("time_unit", "ns"),
            ):
                if row.get(key) != expected:
                    errors.append(f"{row_label}.{key} must be {expected!r}")
            expected_unit = "percentage" if aggregate == "cv" else "time"
            if row.get("aggregate_unit") != expected_unit:
                errors.append(f"{row_label}.aggregate_unit must be {expected_unit!r}")
            cpu_time = _finite_number(row.get("cpu_time"))
            real_time = _finite_number(row.get("real_time"))
            if cpu_time is None or real_time is None:
                errors.append(f"{row_label} timings must be finite numbers")
            elif aggregate in ("mean", "median") and (
                cpu_time <= 0.0 or real_time <= 0.0
            ):
                errors.append(f"{row_label} representative timings must be positive")
            elif aggregate in ("stddev", "cv") and (cpu_time < 0.0 or real_time < 0.0):
                errors.append(f"{row_label} dispersion timings must be non-negative")
            if aggregate == "cv" and (
                cpu_time is None
                or real_time is None
                or cpu_time > 0.10
                or real_time > 0.10
            ):
                errors.append(
                    f"{row_label} timing coefficients of variation must be "
                    "between 0 and 0.10"
                )
            if aggregate in ("mean", "median"):
                expected_counters = {
                    **_PAPER_SCENE_COUNTERS,
                    f"public_{runtime_solver}_family": 1,
                    f"resolved_rigid_body_{runtime_solver}": 1,
                    "resolved_rigid_constraint_iterations": 1,
                    f"resolved_rigid_contact_{runtime_solver}": 1,
                    f"resolved_rigid_pair_constraint_{runtime_solver}": 1,
                    "rigid_constraint_iterations": (_PAPER_RIGID_CONSTRAINT_ITERATIONS),
                    "runtime_contract_passed": 1,
                    "trajectory_frames": _PAPER_TRAJECTORY_FRAMES,
                }
                if requires_solver_configuration:
                    expected_counters.update(
                        {
                            "contact_method_sequential_impulse": 1,
                            "effective_scene_contract_passed": 1,
                            "effective_scene_mutation_audit_passed": 1,
                            "rigid_avbd_alpha": (
                                0.95 if runtime_solver == "avbd" else 0.0
                            ),
                            "rigid_avbd_beta": (
                                10.0 if runtime_solver == "avbd" else 0.0
                            ),
                            "rigid_avbd_gamma": (
                                0.99 if runtime_solver == "avbd" else 0.0
                            ),
                            "rigid_avbd_parameter_profile_paper_2025": (
                                1 if runtime_solver == "avbd" else 0
                            ),
                            "runtime_identity_recorded": 1,
                            "runtime_identity_applicable": 1,
                            "runtime_identity_not_applicable": 0,
                            "runtime_identity_public_avbd_rigid": (
                                1 if runtime_solver == "avbd" else 0
                            ),
                            "runtime_identity_variational_multibody": 0,
                            "runtime_identity_contract_passed": 1,
                            "scene_spec_matches_python": 1,
                            "solver_projection_policies_match": 1,
                        }
                    )
                if fingerprint_value is not None:
                    expected_counters.update(
                        {
                            "scene_spec_fingerprint_hi": fingerprint_value >> 32,
                            "scene_spec_fingerprint_lo": (
                                fingerprint_value & 0xFFFFFFFF
                            ),
                        }
                    )
                if configuration_fingerprint_value is not None:
                    expected_counters.update(
                        {
                            "solver_configuration_fingerprint_hi": (
                                configuration_fingerprint_value >> 32
                            ),
                            "solver_configuration_fingerprint_lo": (
                                configuration_fingerprint_value & 0xFFFFFFFF
                            ),
                        }
                    )
                for key, expected in expected_counters.items():
                    # Google Benchmark averages repetition counters in double
                    # precision, so the mean of five identical 0.95 values is
                    # 0.9500000000000001; integer-valued counters and the
                    # median stay exact.
                    if not _numbers_close(row.get(key), expected):
                        errors.append(
                            f"{row_label}.{key} must be {expected!r} for the "
                            "filename-specific runtime contract"
                        )
                family_index = method_spec["family_index"]
                if family_index is not None:
                    for key, expected in (
                        ("family_index", family_index),
                        ("per_family_instance_index", 0),
                        ("threads", 1),
                    ):
                        if not _numbers_equal(row.get(key), expected):
                            errors.append(f"{row_label}.{key} must be {expected!r}")
                expected_identity_keys = {
                    f"public_{runtime_solver}_family",
                    f"resolved_rigid_body_{runtime_solver}",
                    f"resolved_rigid_contact_{runtime_solver}",
                    f"resolved_rigid_pair_constraint_{runtime_solver}",
                }
                for key, value in row.items():
                    if not isinstance(key, str):
                        continue
                    identity_counter = (
                        (key.startswith("public_") and key.endswith("_family"))
                        or key.startswith("resolved_rigid_body_")
                        or key.startswith("resolved_rigid_contact_")
                        or key.startswith("resolved_rigid_pair_constraint_")
                    )
                    number = _finite_number(value)
                    if identity_counter and key not in expected_identity_keys:
                        if number is None:
                            errors.append(
                                f"{row_label}.{key} must be a finite zero for "
                                "an inactive runtime solver identity"
                            )
                        elif number != 0.0:
                            errors.append(
                                f"{row_label}.{key} contradicts the "
                                "filename-specific runtime solver identity"
                            )
            elif aggregate == "stddev" and requires_solver_configuration:
                median_row = by_aggregate.get("median", {})
                for key in sorted(stable_counter_keys):
                    if not stable_counter_stddev_is_noise(
                        key,
                        _finite_number(row.get(key)),
                        _finite_number(median_row.get(key)),
                    ):
                        errors.append(
                            f"{row_label}.{key} must be 0 (fingerprint words: "
                            "within double-precision aggregation noise) to prove "
                            "identical configuration across benchmark repetitions"
                        )
        mean = by_aggregate["mean"]
        median = by_aggregate["median"]
        cv = by_aggregate["cv"]
        assert isinstance(mean, Mapping)
        assert isinstance(median, Mapping)
        assert isinstance(cv, Mapping)
        timing = entry.get("timing")
        if not isinstance(timing, Mapping):
            errors.append(f"{packet_name}: {location_label}.timing must be an object")
        else:
            for key, row, row_key in (
                ("mean_cpu_time_per_step_ns", mean, "cpu_time"),
                ("mean_real_time_per_step_ns", mean, "real_time"),
                ("median_cpu_time_per_step_ns", median, "cpu_time"),
                ("median_real_time_per_step_ns", median, "real_time"),
            ):
                if not _numbers_equal(timing.get(key), row.get(row_key)):
                    errors.append(
                        f"{packet_name}: {location_label}.timing.{key} must "
                        "be derived from the embedded aggregate row"
                    )
        stability = entry.get("stability")
        if not isinstance(stability, Mapping):
            errors.append(
                f"{packet_name}: {location_label}.stability must be an object"
            )
        else:
            for key, row_key in (
                ("cpu_time_cv_fraction", "cpu_time"),
                ("real_time_cv_fraction", "real_time"),
            ):
                if not _numbers_equal(stability.get(key), cv.get(row_key)):
                    errors.append(
                        f"{packet_name}: {location_label}.stability.{key} must "
                        "be derived from the embedded cv aggregate row"
                    )
            if stability.get("repetitions") != _PAPER_BENCHMARK_REPETITIONS:
                errors.append(
                    f"{packet_name}: {location_label}.stability.repetitions "
                    f"must be {_PAPER_BENCHMARK_REPETITIONS}"
                )
        median_cpu = _finite_number(median.get("cpu_time"))
        if median_cpu is not None and median_cpu > 0.0:
            medians[method_name] = median_cpu
    return errors, medians


def _paper_benchmark_timing_errors(
    packet: Mapping[str, object],
    packet_name: str,
) -> list[str]:
    """Derive durable medians/CVs from aggregate rows and validate ratios."""
    errors, medians = _paper_benchmark_evidence(packet, packet_name)
    if packet_name not in PAPER_CAPTURE_ROLES:
        return errors
    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, Mapping):
        return errors
    comparison = benchmark.get("comparison")
    required_ratio_keys = PAPER_REQUIRED_MEDIAN_RATIO_KEYS.get(packet_name, frozenset())
    if not isinstance(comparison, Mapping):
        if required_ratio_keys:
            errors.append(
                f"{packet_name}: benchmark.comparison must contain exactly "
                f"the required median ratio keys {sorted(required_ratio_keys)!r}"
            )
        return errors
    actual_ratio_keys = {
        key
        for key in comparison
        if isinstance(key, str) and key.endswith(_MEDIAN_RATIO_SUFFIX)
    }
    if actual_ratio_keys != required_ratio_keys:
        errors.append(
            f"{packet_name}: benchmark.comparison median ratio keys must be "
            f"exactly {sorted(required_ratio_keys)!r}, got "
            f"{sorted(actual_ratio_keys)!r}"
        )
    for key, value in comparison.items():
        if not isinstance(key, str) or not key.endswith(_MEDIAN_RATIO_SUFFIX):
            continue
        if not _finite_positive_number(value):
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} must be a "
                "positive finite number"
            )
            continue
        stem = key[: -len(_MEDIAN_RATIO_SUFFIX)]
        if "_to_" not in stem:
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} does not name a "
                "'<numerator>_to_<denominator>' median pair"
            )
            continue
        numerator_name, denominator_name = stem.rsplit("_to_", 1)
        numerator = medians.get(numerator_name)
        denominator = medians.get(denominator_name)
        if packet_name == "avbd-paper-sequential-impulse-comparison-packet.json":
            # The AVBD/VBD denominators live in the linked packet. They are
            # checked against that packet's raw aggregate rows by
            # _paper_link_summary_errors, never trusted from this summary map.
            continue
        if numerator is None or denominator is None:
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} references a "
                "median the packet does not embed"
            )
            continue
        expected = _finite_ratio(numerator, denominator)
        actual = _finite_number(value)
        if expected is None:
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} cannot be "
                "represented as a positive finite ratio"
            )
            continue
        if actual is None or not math.isclose(
            actual,
            expected,
            rel_tol=_MEDIAN_RATIO_RELATIVE_TOLERANCE,
            abs_tol=0.0,
        ):
            errors.append(
                f"{packet_name}: benchmark.comparison.{key} is {value!r} but "
                f"the embedded medians record {expected!r}"
            )
    return errors


def packet_errors(
    path: Path,
    *,
    context: PacketValidationContext | None = None,
    packet_dir: Path | None = None,
) -> list[str]:
    """Validate one packet, sharing parse/visit state when context is supplied."""
    if context is None:
        context = PacketValidationContext(
            packet_dir=DEFAULT_PACKET_DIR if packet_dir is None else packet_dir
        )
    elif packet_dir is not None:
        raise ValueError("packet_dir cannot be combined with a validation context")
    return _packet_errors(path, context)


def collect_packets(args: argparse.Namespace) -> list[Path]:
    if args.packet:
        return list(args.packet)
    return sorted(args.packet_dir.glob(PACKET_GLOB))


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packets = collect_packets(args)
    except (OSError, RuntimeError, ValueError, UnicodeError) as exc:
        print(f"ERROR: packet directory cannot be scanned ({exc})")
        return 1
    if not packets:
        print(f"No {PACKET_GLOB} packets found under {args.packet_dir}")
        return 1

    all_errors: list[str] = []
    reported_errors: set[str] = set()
    context = PacketValidationContext(packet_dir=args.packet_dir)
    for path in packets:
        for error in packet_errors(path, context=context):
            if error not in reported_errors:
                reported_errors.add(error)
                all_errors.append(error)

    stale: list[str] = []
    if args.stale_source == "report":
        all_errors, stale = split_stale_source_findings(all_errors)
    if all_errors:
        for error in all_errors:
            print(f"ERROR: {error}")
        return 1

    if stale:
        for finding in stale:
            print(f"STALE: {finding}", file=sys.stderr)
        stale_packets = sorted({finding.split(": ", 1)[0] for finding in stale})
        print(
            f"Validated {len(packets)} AVBD packet(s); {len(stale_packets)} sealed "
            "evidence packet(s) predate the current source state and cannot "
            "support a parity or performance claim until regenerated: "
            + ", ".join(stale_packets)
        )
        return 0

    print(f"Validated {len(packets)} AVBD packet(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
