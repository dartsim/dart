"""Publication-shaped AVBD breakable-wall reconstruction.

Figure 13 of Giles et al. shows a brick wall hit by three high-momentum balls
and compares the retained-wall outcome across four solver families. The paper
publishes the 1/60 s step, 20 iterations/substeps, impact count, and qualitative
outcome, but not the exact wall geometry, masses, launch speed, or break
threshold. This scene keeps those two classes of facts separate: published
facts are source claims, while every numeric reconstruction choice is recorded
as a visually adjudicated DART parameter.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import math
import struct
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 1.0 / 60.0
_GRAVITY = -9.81
_RIGID_CONSTRAINT_ITERATIONS = 20
_WALL_COLUMNS = 21
_WALL_ROWS = 12
_BRICK_COUNT = _WALL_COLUMNS * _WALL_ROWS
_BRICK_SIZE = np.array([0.60, 0.30, 0.25])
_BRICK_DENSITY = 200.0
_BRICK_FRICTION = 0.50
_BRICK_SPACING_X = 0.62
_BRICK_SPACING_Z = 0.27
_BRICK_BASE_CLEARANCE = 0.02
_BALL_RADIUS = 0.48
_BALL_MASS = 40.0
_BALL_FRICTION = 0.30
_BALL_START_Y = -5.0
_BALL_LAUNCH_SPEED = 24.0
_IMPACT_TARGETS_XZ = (
    (-3.10, 1.55),
    (-0.31, 1.75),
    (3.10, 2.35),
)
_GROUND_SIZE = np.array([16.0, 16.0, 0.50])
_GROUND_FRICTION = 0.60
_BREAK_FORCE = 5000.0
_HORIZONTAL_JOINTS = _WALL_ROWS * (_WALL_COLUMNS - 1)
_VERTICAL_JOINTS = (_WALL_ROWS - 1) * (2 * _WALL_COLUMNS - 1)
_BASE_JOINTS = _WALL_COLUMNS
_BREAKABLE_JOINTS = _HORIZONTAL_JOINTS + _VERTICAL_JOINTS + _BASE_JOINTS
_OUTCOME_FRAME = 120
_IMPACT_DAMAGE_DISPLACEMENT_THRESHOLD = 0.10
_RETAINED_DISPLACEMENT_THRESHOLD = 0.50
_IMPACT_BAND_RADIUS = 0.85
_OUTSIDE_RADIUS = 1.15
_CAPTURE_SIZE = (1280, 720)
_CAPTURE_CAMERA_AZIMUTH = -5.0 * math.pi / 8.0
_CAPTURE_CAMERA_ELEVATION = 0.62
_CAPTURE_CAMERA_DISTANCE = 22.0
_CAPTURE_CAMERA_TARGET = (0.0, 0.45, 1.6)
_FNV1A_64_OFFSET_BASIS = 14695981039346656037
_FNV1A_64_PRIME = 1099511628211
_FNV1A_64_MASK = (1 << 64) - 1
_SCENE_SPEC_FINGERPRINT_TAG = "avbd-paper-breakable-wall/v3"

PAPER_REFERENCE: dict[str, Any] = {
    "paper": {
        "authors": ("Chris Giles", "Elie Diaz", "Cem Yuksel"),
        "title": "Augmented Vertex Block Descent",
        "venue": "ACM Transactions on Graphics 44(4), Article 90",
        "year": 2025,
        "doi": "10.1145/3731195",
    },
    "source_locator": "Section 5.4 and Figure 13, PDF page 10",
    "published_facts": {
        "time_step": _TIME_STEP,
        "iterations_or_substeps": _RIGID_CONSTRAINT_ITERATIONS,
        "impacting_balls": 3,
        "comparison_methods": (
            "sequential impulse",
            "XPBD",
            "VBD",
            "AVBD",
        ),
        "avbd_outcome": (
            "localized displacement damage in three impact bands while large "
            "retained wall regions remain attached and standing"
        ),
        "sequential_impulse_outcome": (
            "clean initial fracture followed by retained-constraint failure, "
            "slow wall bending, and eventual collapse"
        ),
    },
    "unpublished_scene_constants": (
        "wall dimensions and brick geometry",
        "brick density and friction",
        "ball radius, mass, launch speed, and exact impact locations",
        "break force",
    ),
    "dart_reconstruction": {
        "wall_columns": _WALL_COLUMNS,
        "wall_rows": _WALL_ROWS,
        "brick_size": tuple(float(value) for value in _BRICK_SIZE),
        "brick_density": _BRICK_DENSITY,
        "brick_spacing_x": _BRICK_SPACING_X,
        "brick_spacing_z": _BRICK_SPACING_Z,
        "ball_radius": _BALL_RADIUS,
        "ball_mass": _BALL_MASS,
        "ball_start_y": _BALL_START_Y,
        "ball_launch_speed": _BALL_LAUNCH_SPEED,
        "impact_targets_xz": _IMPACT_TARGETS_XZ,
        "break_force": _BREAK_FORCE,
        "selection": "visual adjudication against the paper and project video",
    },
}

OUTCOME_METRIC_THRESHOLDS: dict[str, Any] = {
    "impact_damage_displacement_threshold": (
        _IMPACT_DAMAGE_DISPLACEMENT_THRESHOLD
    ),
    "retained_displacement_threshold": _RETAINED_DISPLACEMENT_THRESHOLD,
    "impact_band_radius": _IMPACT_BAND_RADIUS,
    "outside_radius": _OUTSIDE_RADIUS,
}

OUTCOME_ORACLE: dict[str, Any] = {
    **OUTCOME_METRIC_THRESHOLDS,
    "evaluation_frame": _OUTCOME_FRAME,
    "joint_evidence_frames": (60, _OUTCOME_FRAME),
    "minimum_displaced_bricks_per_impact_band": 4,
    "minimum_outside_retained_fraction": 0.95,
    "minimum_total_retained_fraction": 0.95,
    "minimum_broken_joints": 150,
    "maximum_broken_joints": 250,
    "minimum_unbroken_joints": 450,
    "expected_broken_joint_ids_sha256": (
        "31b187538ac1549563be368a4d7e304d1caef6ea11ba65b624d48f5f27468503"
    ),
    "maximum_unbroken_joint_linear_residual": 0.002,
    "maximum_unbroken_joint_angular_residual_radians": 0.001,
}


@dataclass(frozen=True)
class _FixedJointEvidence:
    joint: sx.Joint
    kind: str
    parent: sx.RigidBody
    child: sx.RigidBody
    parent_index: int
    child_index: int
    local_anchor_parent: np.ndarray
    local_anchor_child: np.ndarray
    target_relative_rotation: np.ndarray
    initial_anchor: np.ndarray


def _fnv1a_64_update(state: int, payload: bytes) -> int:
    for value in payload:
        state ^= value
        state = (state * _FNV1A_64_PRIME) & _FNV1A_64_MASK
    return state


def _scene_spec_fingerprint(
    topology_records: Sequence[tuple[int, int, int]],
) -> str:
    """Hash the exact reconstructed scene contract shared with the benchmark."""
    state = _FNV1A_64_OFFSET_BASIS

    def update_u64(value: int) -> None:
        nonlocal state
        state = _fnv1a_64_update(state, struct.pack("<Q", value))

    def update_f64(value: float) -> None:
        nonlocal state
        state = _fnv1a_64_update(state, struct.pack("<d", value))

    encoded_tag = _SCENE_SPEC_FINGERPRINT_TAG.encode("utf-8")
    update_u64(len(encoded_tag))
    state = _fnv1a_64_update(state, encoded_tag)
    update_f64(_TIME_STEP)
    update_f64(_GRAVITY)
    update_u64(_RIGID_CONSTRAINT_ITERATIONS)
    update_u64(_WALL_COLUMNS)
    update_u64(_WALL_ROWS)
    for value in _BRICK_SIZE:
        update_f64(float(value))
    for value in (
        _BRICK_DENSITY,
        _BRICK_FRICTION,
        _BRICK_SPACING_X,
        _BRICK_SPACING_Z,
        _BRICK_BASE_CLEARANCE,
        _BALL_RADIUS,
        _BALL_MASS,
        _BALL_FRICTION,
        _BALL_START_Y,
        _BALL_LAUNCH_SPEED,
    ):
        update_f64(value)
    update_u64(len(_IMPACT_TARGETS_XZ))
    for target_x, target_z in _IMPACT_TARGETS_XZ:
        update_f64(target_x)
        update_f64(target_z)
    for value in _GROUND_SIZE:
        update_f64(float(value))
    update_f64(_GROUND_FRICTION)
    update_f64(_BREAK_FORCE)
    update_u64(len(topology_records))
    for kind, parent_index, child_index in topology_records:
        update_u64(kind)
        update_u64(parent_index)
        update_u64(child_index)
    return f"{state:016x}"


def _serialize_and_validate_resolved_configuration(
    world: sx.World,
    solver_key: str,
) -> list[dict[str, str]]:
    notes = [
        {
            "domain": note.domain,
            "requested": note.requested,
            "resolved": note.resolved,
            "reason": note.reason,
        }
        for note in world.resolved_configuration.notes
    ]
    expected = {
        "rigid-body": (solver_key, solver_key),
        "rigid-contact": (solver_key, solver_key),
        "rigid-pair-constraint": (solver_key, solver_key),
        "rigid-constraint-iterations": (
            str(_RIGID_CONSTRAINT_ITERATIONS),
            str(_RIGID_CONSTRAINT_ITERATIONS),
        ),
    }
    for domain, (requested, resolved) in expected.items():
        matching = [
            note
            for note in notes
            if note["domain"] == domain
            and note["requested"] == requested
            and note["resolved"] == resolved
        ]
        if len(matching) != 1:
            raise RuntimeError(
                f"{solver_key.upper()} breakable-wall resolved configuration "
                "drifted: "
                f"expected one {domain} {requested}->{resolved} note, "
                f"got {matching!r}"
            )
    return notes


def _view_assessment_available() -> bool:
    return all(
        hasattr(dart.gui, name)
        for name in ("OrbitCamera", "ProjectionOptions", "assess_view_quality")
    )


def _full_box_inertia(size: np.ndarray, mass: float) -> np.ndarray:
    return np.diag(
        [
            mass * float(size[1] * size[1] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[1] * size[1]) / 12.0,
        ]
    )


def _brick_position(row: int, column: int) -> np.ndarray:
    course_offset = 0.5 if row % 2 else 0.0
    return np.array(
        [
            (column - 0.5 * (_WALL_COLUMNS - 1) + course_offset)
            * _BRICK_SPACING_X,
            0.0,
            _BRICK_BASE_CLEARANCE
            + 0.5 * float(_BRICK_SIZE[2])
            + row * _BRICK_SPACING_Z,
        ]
    )


def _add_box(
    world: sx.World,
    name: str,
    *,
    size: np.ndarray,
    position: np.ndarray,
    mass: float | None,
    friction: float,
) -> sx.RigidBody:
    body = world.add_rigid_body(name, position=tuple(position))
    body.is_static = mass is None
    body.friction = friction
    body.set_collision_shape(sx.CollisionShape.box(0.5 * size))
    if mass is not None:
        body.mass = mass
        body.inertia = _full_box_inertia(size, mass)
    return body


def _add_breakable_fixed_joint(
    world: sx.World,
    name: str,
    parent: sx.RigidBody,
    child: sx.RigidBody,
) -> sx.Joint:
    spec = sx.JointSpec(name=name, type=sx.JointType.FIXED)
    joint = world.add_joint(parent, child, spec)
    joint.break_force = _BREAK_FORCE
    return joint


def _positions(bodies: Sequence[sx.RigidBody]) -> np.ndarray:
    return np.asarray(
        [
            np.asarray(body.translation, dtype=float).reshape(3)
            for body in bodies
        ],
        dtype=float,
    )


def _fixed_joint_evidence(
    joints: Sequence[sx.Joint],
    topology_records: Sequence[tuple[int, int, int]],
) -> tuple[_FixedJointEvidence, ...]:
    kind_names = {1: "horizontal", 2: "vertical", 3: "base"}
    evidence = []
    for joint, (kind, parent_index, child_index) in zip(
        joints,
        topology_records,
        strict=True,
    ):
        parent = joint.parent_rigid_body
        child = joint.child_rigid_body
        parent_transform = np.asarray(parent.transform, dtype=float).reshape(4, 4)
        child_transform = np.asarray(child.transform, dtype=float).reshape(4, 4)
        initial_anchor = child_transform[:3, 3].copy()
        local_anchor_parent = (
            parent_transform[:3, :3].T
            @ (initial_anchor - parent_transform[:3, 3])
        )
        local_anchor_child = (
            child_transform[:3, :3].T
            @ (initial_anchor - child_transform[:3, 3])
        )
        evidence.append(
            _FixedJointEvidence(
                joint=joint,
                kind=kind_names[kind],
                parent=parent,
                child=child,
                parent_index=parent_index,
                child_index=child_index,
                local_anchor_parent=local_anchor_parent,
                local_anchor_child=local_anchor_child,
                target_relative_rotation=(
                    parent_transform[:3, :3].T
                    @ child_transform[:3, :3]
                ),
                initial_anchor=initial_anchor,
            )
        )
    return tuple(evidence)


def _endpoint_grid_location(index: int) -> dict[str, Any]:
    if index == 0:
        return {"body": "ground", "column": None, "row": None}
    brick_index = index - 1
    return {
        "body": "brick",
        "column": int(brick_index % _WALL_COLUMNS),
        "row": int(brick_index // _WALL_COLUMNS),
    }


def _rotation_angle(rotation: np.ndarray) -> float:
    cosine = float(np.clip(0.5 * (np.trace(rotation) - 1.0), -1.0, 1.0))
    return float(math.acos(cosine))


def _joint_constraint_evidence(
    records: Sequence[_FixedJointEvidence],
    *,
    include_broken_records: bool,
) -> dict[str, Any]:
    unbroken_linear_residuals = []
    unbroken_angular_residuals = []
    outside_unbroken_linear_residuals = []
    outside_unbroken_angular_residuals = []
    broken_records = []
    broken_joint_ids = []
    broken_impact_band_counts = [0] * len(_IMPACT_TARGETS_XZ)
    broken_outside_impact_regions = 0

    for record in records:
        parent_rotation = np.asarray(
            record.parent.rotation, dtype=float
        ).reshape(3, 3)
        child_rotation = np.asarray(
            record.child.rotation, dtype=float
        ).reshape(3, 3)
        parent_anchor = (
            np.asarray(record.parent.translation, dtype=float).reshape(3)
            + parent_rotation @ record.local_anchor_parent
        )
        child_anchor = (
            np.asarray(record.child.translation, dtype=float).reshape(3)
            + child_rotation @ record.local_anchor_child
        )
        linear_residual = float(np.linalg.norm(child_anchor - parent_anchor))
        current_relative_rotation = parent_rotation.T @ child_rotation
        angular_residual = _rotation_angle(
            record.target_relative_rotation.T @ current_relative_rotation
        )
        impact_distances = [
            float(
                np.linalg.norm(
                    record.initial_anchor[[0, 2]] - np.asarray(target)
                )
            )
            for target in _IMPACT_TARGETS_XZ
        ]
        nearest_impact_index = int(np.argmin(impact_distances))
        nearest_impact_distance = impact_distances[nearest_impact_index]
        within_impact_band = nearest_impact_distance <= _IMPACT_BAND_RADIUS
        within_impact_region = nearest_impact_distance <= _OUTSIDE_RADIUS

        if not record.joint.is_broken:
            unbroken_linear_residuals.append(linear_residual)
            unbroken_angular_residuals.append(angular_residual)
            if not within_impact_region:
                outside_unbroken_linear_residuals.append(linear_residual)
                outside_unbroken_angular_residuals.append(angular_residual)
            continue

        broken_joint_ids.append(record.joint.name)
        if within_impact_region:
            broken_impact_band_counts[nearest_impact_index] += 1
        else:
            broken_outside_impact_regions += 1
        if include_broken_records:
            broken_records.append(
                {
                    "angular_residual_radians": angular_residual,
                    "child": _endpoint_grid_location(record.child_index),
                    "id": record.joint.name,
                    "initial_anchor": record.initial_anchor.tolist(),
                    "kind": record.kind,
                    "linear_residual": linear_residual,
                    "nearest_impact_distance": nearest_impact_distance,
                    "nearest_impact_index": nearest_impact_index,
                    "parent": _endpoint_grid_location(record.parent_index),
                    "within_impact_band": within_impact_band,
                    "within_impact_region": within_impact_region,
                }
            )

    def maximum(values: Sequence[float]) -> float:
        return float(max(values, default=0.0))

    def rms(values: Sequence[float]) -> float:
        return (
            float(np.sqrt(np.mean(np.square(values))))
            if values
            else 0.0
        )

    broken_joint_ids_digest = hashlib.sha256()
    for joint_id in sorted(broken_joint_ids):
        encoded_id = joint_id.encode("utf-8")
        broken_joint_ids_digest.update(struct.pack("<Q", len(encoded_id)))
        broken_joint_ids_digest.update(encoded_id)

    return {
        "broken_joint_identity_count": len(broken_joint_ids),
        "broken_joint_ids_sha256": broken_joint_ids_digest.hexdigest(),
        "broken_joint_impact_region_counts": broken_impact_band_counts,
        "broken_joint_records": broken_records,
        "broken_joints_outside_impact_regions": broken_outside_impact_regions,
        "joint_residuals_finite": bool(
            np.all(np.isfinite(unbroken_linear_residuals))
            and np.all(np.isfinite(unbroken_angular_residuals))
            and np.all(np.isfinite(outside_unbroken_linear_residuals))
            and np.all(np.isfinite(outside_unbroken_angular_residuals))
            and all(
                math.isfinite(record["linear_residual"])
                and math.isfinite(record["angular_residual_radians"])
                and math.isfinite(record["nearest_impact_distance"])
                for record in broken_records
            )
        ),
        "maximum_unbroken_joint_angular_residual_radians": maximum(
            unbroken_angular_residuals
        ),
        "maximum_unbroken_joint_linear_residual": maximum(
            unbroken_linear_residuals
        ),
        "maximum_outside_impact_unbroken_joint_angular_residual_radians": (
            maximum(outside_unbroken_angular_residuals)
        ),
        "maximum_outside_impact_unbroken_joint_linear_residual": maximum(
            outside_unbroken_linear_residuals
        ),
        "rms_unbroken_joint_angular_residual_radians": rms(
            unbroken_angular_residuals
        ),
        "rms_unbroken_joint_linear_residual": rms(
            unbroken_linear_residuals
        ),
        "rms_outside_impact_unbroken_joint_angular_residual_radians": rms(
            outside_unbroken_angular_residuals
        ),
        "rms_outside_impact_unbroken_joint_linear_residual": rms(
            outside_unbroken_linear_residuals
        ),
        "outside_impact_unbroken_joint_residual_count": len(
            outside_unbroken_linear_residuals
        ),
        "unbroken_joint_residual_count": len(unbroken_linear_residuals),
    }


def compute_outcome_metrics(
    *,
    world: sx.World,
    bricks: Sequence[sx.RigidBody],
    joints: Sequence[sx.Joint],
    balls: Sequence[sx.RigidBody],
    initial_brick_positions: np.ndarray,
    joint_evidence: Sequence[_FixedJointEvidence],
    solver_family: str = "avbd",
    outcome_oracle: dict[str, Any] = OUTCOME_ORACLE,
) -> dict[str, Any]:
    """Evaluate the fail-closed Figure 13 outcome oracle at the current state."""
    brick_positions = _positions(bricks)
    ball_positions = _positions(balls)
    displacements = np.linalg.norm(brick_positions - initial_brick_positions, axis=1)
    wall_normal_displacements = np.abs(
        brick_positions[:, 1] - initial_brick_positions[:, 1]
    )
    initial_xz = initial_brick_positions[:, (0, 2)]
    impact_damage_displacement_threshold = float(
        outcome_oracle.get(
            "impact_damage_displacement_threshold",
            _IMPACT_DAMAGE_DISPLACEMENT_THRESHOLD,
        )
    )
    retained_displacement_threshold = float(
        outcome_oracle.get(
            "retained_displacement_threshold",
            _RETAINED_DISPLACEMENT_THRESHOLD,
        )
    )

    impact_band_displaced_counts = []
    for target in _IMPACT_TARGETS_XZ:
        distances = np.linalg.norm(initial_xz - np.asarray(target), axis=1)
        impact_band_displaced_counts.append(
            int(
                np.count_nonzero(
                    (distances <= _IMPACT_BAND_RADIUS)
                    & (displacements > impact_damage_displacement_threshold)
                )
            )
        )

    outside_mask = np.ones(_BRICK_COUNT, dtype=bool)
    for target in _IMPACT_TARGETS_XZ:
        outside_mask &= (
            np.linalg.norm(initial_xz - np.asarray(target), axis=1)
            > _OUTSIDE_RADIUS
        )
    retained_mask = displacements < retained_displacement_threshold
    outside_count = int(np.count_nonzero(outside_mask))
    outside_retained_fraction = float(
        np.count_nonzero(retained_mask & outside_mask) / outside_count
    )
    total_retained_fraction = float(np.count_nonzero(retained_mask) / _BRICK_COUNT)

    frame = int(round(float(world.time) / _TIME_STEP))
    joint_constraint_evidence = _joint_constraint_evidence(
        joint_evidence,
        include_broken_records=frame
        in {
            int(value)
            for value in outcome_oracle.get("joint_evidence_frames", ())
        },
    )
    broken_joints = sum(1 for joint in joints if joint.is_broken)
    unbroken_joints = len(joints) - broken_joints
    include_broken_records = frame in {
        int(value)
        for value in outcome_oracle.get("joint_evidence_frames", ())
    }
    if (
        joint_constraint_evidence["unbroken_joint_residual_count"]
        != unbroken_joints
    ):
        raise RuntimeError(
            "breakable-wall joint evidence count drifted: "
            f"expected {unbroken_joints} unbroken records, got "
            f"{joint_constraint_evidence['unbroken_joint_residual_count']}"
        )
    if joint_constraint_evidence["broken_joint_identity_count"] != broken_joints:
        raise RuntimeError(
            "breakable-wall broken-joint identity count drifted: "
            f"expected {broken_joints} identities, got "
            f"{joint_constraint_evidence['broken_joint_identity_count']}"
        )
    if (
        include_broken_records
        and len(joint_constraint_evidence["broken_joint_records"])
        != broken_joints
    ):
        raise RuntimeError(
            "breakable-wall detailed broken-joint evidence count drifted: "
            f"expected {broken_joints} records, got "
            f"{len(joint_constraint_evidence['broken_joint_records'])}"
        )
    bent_brick_displacement_threshold = float(
        outcome_oracle.get("bent_brick_displacement_threshold", 0.05)
    )
    finite_state = bool(
        np.all(np.isfinite(brick_positions))
        and np.all(np.isfinite(ball_positions))
        and joint_constraint_evidence["joint_residuals_finite"]
        and all(
            np.all(
                np.isfinite(
                    np.asarray(body.linear_velocity, dtype=float).reshape(3)
                )
            )
            and np.all(
                np.isfinite(
                    np.asarray(body.angular_velocity, dtype=float).reshape(3)
                )
            )
            for body in (*bricks, *balls)
        )
    )
    evaluation_frame = int(outcome_oracle["evaluation_frame"])
    checkpoint = "outcome"
    if solver_family == "avbd":
        threshold_checks = {
            "finite_state": finite_state,
            "damage_in_three_impact_bands": all(
                count
                >= outcome_oracle["minimum_displaced_bricks_per_impact_band"]
                for count in impact_band_displaced_counts
            ),
            "outside_wall_retained": (
                outside_retained_fraction
                >= outcome_oracle["minimum_outside_retained_fraction"]
            ),
            "total_wall_retained": (
                total_retained_fraction
                >= outcome_oracle["minimum_total_retained_fraction"]
            ),
            "fracture_activated": (
                broken_joints >= outcome_oracle["minimum_broken_joints"]
            ),
            "fracture_count_bounded": (
                broken_joints <= outcome_oracle["maximum_broken_joints"]
                and unbroken_joints >= outcome_oracle["minimum_unbroken_joints"]
            ),
            "fracture_identity_matches": (
                joint_constraint_evidence["broken_joint_ids_sha256"]
                == outcome_oracle["expected_broken_joint_ids_sha256"]
            ),
            "retained_joint_rows_satisfied": (
                joint_constraint_evidence[
                    "maximum_unbroken_joint_linear_residual"
                ]
                <= outcome_oracle[
                    "maximum_unbroken_joint_linear_residual"
                ]
                and joint_constraint_evidence[
                    "maximum_unbroken_joint_angular_residual_radians"
                ]
                <= outcome_oracle[
                    "maximum_unbroken_joint_angular_residual_radians"
                ]
            ),
        }
        evaluated = frame >= evaluation_frame
    elif solver_family == "vbd":
        retention_frame = int(outcome_oracle["retention_evaluation_frame"])
        if frame >= retention_frame:
            checkpoint = "retention"
            threshold_checks = {
                "finite_state": finite_state,
                "no_fracture": (
                    broken_joints <= outcome_oracle["maximum_broken_joints"]
                ),
                "topology_retained": (
                    unbroken_joints >= outcome_oracle["minimum_unbroken_joints"]
                ),
                "retained_joint_rows_satisfied": (
                    joint_constraint_evidence[
                        "maximum_unbroken_joint_linear_residual"
                    ]
                    <= outcome_oracle[
                        "maximum_unbroken_joint_linear_residual"
                    ]
                    and joint_constraint_evidence[
                        "maximum_unbroken_joint_angular_residual_radians"
                    ]
                    <= outcome_oracle[
                        "maximum_unbroken_joint_angular_residual_radians"
                    ]
                ),
                "wall_retained": (
                    total_retained_fraction
                    >= outcome_oracle["minimum_total_retained_fraction"]
                ),
            }
            evaluated = True
        else:
            checkpoint = "bend"
            threshold_checks = {
                "finite_state": finite_state,
                "no_fracture": (
                    broken_joints <= outcome_oracle["maximum_broken_joints"]
                ),
                "topology_retained": (
                    unbroken_joints >= outcome_oracle["minimum_unbroken_joints"]
                ),
                "retained_joint_rows_satisfied": (
                    joint_constraint_evidence[
                        "maximum_unbroken_joint_linear_residual"
                    ]
                    <= outcome_oracle[
                        "maximum_unbroken_joint_linear_residual"
                    ]
                    and joint_constraint_evidence[
                        "maximum_unbroken_joint_angular_residual_radians"
                    ]
                    <= outcome_oracle[
                        "maximum_unbroken_joint_angular_residual_radians"
                    ]
                ),
                "wall_bends": (
                    float(np.max(wall_normal_displacements))
                    >= outcome_oracle[
                        "minimum_maximum_wall_normal_displacement"
                    ]
                ),
                "wall_bend_is_distributed": (
                    float(np.sqrt(np.mean(np.square(wall_normal_displacements))))
                    >= outcome_oracle["minimum_rms_wall_normal_displacement"]
                ),
                "bend_is_spatially_resolved": (
                    int(
                        np.count_nonzero(
                            wall_normal_displacements
                            >= outcome_oracle[
                                "bent_brick_displacement_threshold"
                            ]
                        )
                    )
                    >= outcome_oracle["minimum_bent_bricks"]
                ),
            }
            evaluated = frame == evaluation_frame
    elif solver_family == "sequential_impulse":
        collapse_frame = int(outcome_oracle["collapse_evaluation_frame"])
        if frame >= collapse_frame:
            checkpoint = "collapse"
            threshold_checks = {
                "finite_state": finite_state,
                "initial_fracture_remains_visible": (
                    broken_joints
                    >= outcome_oracle["minimum_final_broken_joints"]
                ),
                "fracture_identity_unchanged": (
                    broken_joints
                    == outcome_oracle["expected_broken_joints"]
                    and joint_constraint_evidence[
                        "broken_joint_ids_sha256"
                    ]
                    == outcome_oracle["expected_broken_joint_ids_sha256"]
                    and joint_constraint_evidence[
                        "broken_joints_outside_impact_regions"
                    ]
                    == 0
                    and joint_constraint_evidence[
                        "outside_impact_unbroken_joint_residual_count"
                    ]
                    == outcome_oracle[
                        "expected_outside_impact_unbroken_joint_count"
                    ]
                ),
                "damage_in_three_impact_bands": all(
                    count
                    >= outcome_oracle[
                        "minimum_displaced_bricks_per_impact_band"
                    ]
                    for count in impact_band_displaced_counts
                ),
                "retained_rows_fail_outside_impacts": (
                    joint_constraint_evidence[
                        "maximum_outside_impact_unbroken_joint_linear_residual"
                    ]
                    >= outcome_oracle[
                        "minimum_collapse_outside_joint_maximum_linear_residual"
                    ]
                    and joint_constraint_evidence[
                        "rms_outside_impact_unbroken_joint_linear_residual"
                    ]
                    >= outcome_oracle[
                        "minimum_collapse_outside_joint_rms_linear_residual"
                    ]
                    and joint_constraint_evidence[
                        "maximum_outside_impact_unbroken_joint_angular_residual_radians"
                    ]
                    >= outcome_oracle[
                        "minimum_collapse_outside_joint_maximum_angular_residual_radians"
                    ]
                    and joint_constraint_evidence[
                        "rms_outside_impact_unbroken_joint_angular_residual_radians"
                    ]
                    >= outcome_oracle[
                        "minimum_collapse_outside_joint_rms_angular_residual_radians"
                    ]
                ),
                "outside_wall_collapses": (
                    outside_retained_fraction
                    <= outcome_oracle[
                        "maximum_collapse_outside_retained_fraction"
                    ]
                ),
                "wall_collapses": (
                    total_retained_fraction
                    <= outcome_oracle[
                        "maximum_collapse_total_retained_fraction"
                    ]
                    and float(np.max(wall_normal_displacements))
                    >= outcome_oracle[
                        "minimum_collapse_wall_normal_displacement"
                    ]
                ),
            }
            evaluated = True
        else:
            checkpoint = "fracture"
            threshold_checks = {
                "finite_state": finite_state,
                "fracture_activated": (
                    broken_joints
                    >= outcome_oracle["minimum_initial_broken_joints"]
                ),
                "initial_fracture_confined_to_impact_regions": (
                    broken_joints
                    <= outcome_oracle["maximum_initial_broken_joints"]
                    and unbroken_joints
                    >= outcome_oracle["minimum_initial_unbroken_joints"]
                    and joint_constraint_evidence[
                        "broken_joints_outside_impact_regions"
                    ]
                    == 0
                ),
                "initial_fracture_covers_three_impacts": all(
                    count
                    >= outcome_oracle[
                        "minimum_initial_broken_joints_per_impact_region"
                    ]
                    for count in joint_constraint_evidence[
                        "broken_joint_impact_region_counts"
                    ]
                ),
                "initial_fracture_identity_matches": (
                    broken_joints == outcome_oracle["expected_broken_joints"]
                    and joint_constraint_evidence[
                        "broken_joint_ids_sha256"
                    ]
                    == outcome_oracle["expected_broken_joint_ids_sha256"]
                    and joint_constraint_evidence[
                        "outside_impact_unbroken_joint_residual_count"
                    ]
                    == outcome_oracle[
                        "expected_outside_impact_unbroken_joint_count"
                    ]
                ),
                "initial_retained_joint_rows_bounded": (
                    joint_constraint_evidence[
                        "maximum_outside_impact_unbroken_joint_linear_residual"
                    ]
                    <= outcome_oracle[
                        "maximum_initial_outside_joint_linear_residual"
                    ]
                    and joint_constraint_evidence[
                        "maximum_outside_impact_unbroken_joint_angular_residual_radians"
                    ]
                    <= outcome_oracle[
                        "maximum_initial_outside_joint_angular_residual_radians"
                    ]
                ),
                "wall_initially_retained": (
                    total_retained_fraction
                    >= outcome_oracle[
                        "minimum_initial_total_retained_fraction"
                    ]
                ),
            }
            evaluated = frame == evaluation_frame
    else:
        raise ValueError(f"unsupported breakable-wall solver family: {solver_family}")
    thresholds_pass = evaluated and all(threshold_checks.values())
    step_metrics = world.compute_step_metrics()
    return {
        "frame": frame,
        "world_time": float(world.time),
        "evaluated": evaluated,
        "checkpoint": checkpoint,
        "status": (
            "pass"
            if thresholds_pass
            else "fail"
            if evaluated
            else "pre-evaluation"
            if frame < evaluation_frame
            else "between-checkpoints"
        ),
        "thresholds_pass": thresholds_pass,
        "threshold_checks": threshold_checks,
        "impact_damage_displacement_threshold": (
            impact_damage_displacement_threshold
        ),
        "retained_displacement_threshold": retained_displacement_threshold,
        "impact_band_displaced_counts": impact_band_displaced_counts,
        "outside_brick_count": outside_count,
        "outside_retained_fraction": outside_retained_fraction,
        "total_retained_fraction": total_retained_fraction,
        "broken_joints": broken_joints,
        "unbroken_joints": unbroken_joints,
        **joint_constraint_evidence,
        "max_brick_displacement": float(np.max(displacements)),
        "maximum_wall_normal_displacement": float(
            np.max(wall_normal_displacements)
        ),
        "rms_wall_normal_displacement": float(
            np.sqrt(np.mean(np.square(wall_normal_displacements)))
        ),
        "bent_brick_count": int(
            np.count_nonzero(
                wall_normal_displacements
                >= bent_brick_displacement_threshold
            )
        ),
        "ball_positions": ball_positions.tolist(),
        "ball_velocities": [
            np.asarray(ball.linear_velocity, dtype=float).reshape(3).tolist()
            for ball in balls
        ],
        "last_step_iterations": int(step_metrics.last_step_iterations),
        "contact_count": int(step_metrics.active_contact_count),
    }


def build_solver_variant(
    *,
    rigid_body_solver: sx.RigidBodySolver,
    solver_key: str,
    solver_display: str,
    paper_reference: dict[str, Any],
    outcome_oracle: dict[str, Any],
    capture_assessment_frames: tuple[int, ...],
    resolved_solver_key: str | None = None,
) -> SceneSetup:
    """Build one matched Figure 13 solver row from the shared reconstruction."""
    scene_id = f"{solver_key}_paper_breakable_wall"
    object_prefix = f"{solver_key}_paper"
    world = sx.World(
        time_step=_TIME_STEP,
        gravity=(0.0, 0.0, _GRAVITY),
        rigid_body_solver=rigid_body_solver,
        rigid_constraint_options=sx.RigidConstraintOptions(
            iterations=_RIGID_CONSTRAINT_ITERATIONS
        ),
    )

    ground = _add_box(
        world,
        f"{object_prefix}_breakable_wall_ground",
        size=_GROUND_SIZE,
        position=np.array([0.0, 0.0, -0.5 * float(_GROUND_SIZE[2])]),
        mass=None,
        friction=_GROUND_FRICTION,
    )

    brick_mass = float(np.prod(_BRICK_SIZE) * _BRICK_DENSITY)
    brick_rows: list[list[sx.RigidBody]] = []
    bricks: list[sx.RigidBody] = []
    for row in range(_WALL_ROWS):
        course = []
        for column in range(_WALL_COLUMNS):
            brick = _add_box(
                world,
                f"{object_prefix}_wall_brick_{row:02d}_{column:02d}",
                size=_BRICK_SIZE,
                position=_brick_position(row, column),
                mass=brick_mass,
                friction=_BRICK_FRICTION,
            )
            course.append(brick)
            bricks.append(brick)
        brick_rows.append(course)

    joints: list[sx.Joint] = []
    topology_records: list[tuple[int, int, int]] = []
    for row, course in enumerate(brick_rows):
        for column, (left, right) in enumerate(zip(course, course[1:])):
            joints.append(
                _add_breakable_fixed_joint(
                    world,
                    f"{object_prefix}_wall_horizontal_{row:02d}_{column:02d}",
                    left,
                    right,
                )
            )
            topology_records.append(
                (
                    1,
                    1 + row * _WALL_COLUMNS + column,
                    1 + row * _WALL_COLUMNS + column + 1,
                )
            )

    for row in range(1, _WALL_ROWS):
        lower = brick_rows[row - 1]
        upper = brick_rows[row]
        for upper_column, upper_brick in enumerate(upper):
            upper_x = _brick_position(row, upper_column)[0]
            for lower_column, lower_brick in enumerate(lower):
                lower_x = _brick_position(row - 1, lower_column)[0]
                if abs(float(upper_x - lower_x)) < float(_BRICK_SIZE[0]):
                    joints.append(
                        _add_breakable_fixed_joint(
                            world,
                            f"{object_prefix}_wall_vertical_"
                            f"{row - 1:02d}_{lower_column:02d}_"
                            f"{row:02d}_{upper_column:02d}",
                            lower_brick,
                            upper_brick,
                        )
                    )
                    topology_records.append(
                        (
                            2,
                            1 + (row - 1) * _WALL_COLUMNS + lower_column,
                            1 + row * _WALL_COLUMNS + upper_column,
                        )
                    )

    for column, brick in enumerate(brick_rows[0]):
        joints.append(
            _add_breakable_fixed_joint(
                world,
                f"{object_prefix}_wall_base_{column:02d}",
                ground,
                brick,
            )
        )
        topology_records.append((3, 0, 1 + column))

    balls: list[sx.RigidBody] = []
    sphere_inertia = (
        2.0 / 5.0 * _BALL_MASS * _BALL_RADIUS * _BALL_RADIUS * np.eye(3)
    )
    for index, (target_x, target_z) in enumerate(_IMPACT_TARGETS_XZ):
        ball = world.add_rigid_body(
            f"{object_prefix}_wall_ball_{index}",
            position=(target_x, _BALL_START_Y, target_z),
            linear_velocity=(0.0, _BALL_LAUNCH_SPEED, 0.0),
        )
        ball.mass = _BALL_MASS
        ball.inertia = sphere_inertia
        ball.friction = _BALL_FRICTION
        ball.set_collision_shape(sx.CollisionShape.sphere(_BALL_RADIUS))
        balls.append(ball)

    if len(joints) != _BREAKABLE_JOINTS:
        raise RuntimeError(
            "breakable-wall topology drifted: "
            f"expected {_BREAKABLE_JOINTS} joints, built {len(joints)}"
        )
    if len(topology_records) != len(joints):
        raise RuntimeError("breakable-wall topology fingerprint record drifted")

    initial_brick_positions = _positions(bricks)
    joint_evidence = _fixed_joint_evidence(joints, topology_records)
    world.enter_simulation_mode()
    resolved_configuration = _serialize_and_validate_resolved_configuration(
        world, resolved_solver_key or solver_key
    )
    scene_spec_fingerprint = _scene_spec_fingerprint(topology_records)
    view_assessment_available = _view_assessment_available()
    capture_camera = (
        dart.gui.orbit_camera(
            azimuth=_CAPTURE_CAMERA_AZIMUTH,
            elevation=_CAPTURE_CAMERA_ELEVATION,
            distance=_CAPTURE_CAMERA_DISTANCE,
            target=_CAPTURE_CAMERA_TARGET,
        )
        if view_assessment_available
        else None
    )
    capture_focus = tuple(f"{brick.name}_visual" for brick in bricks)

    bridge = WorldRenderBridge(world, name=f"{scene_id}_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.42, 0.45),
        name=f"{object_prefix}_breakable_wall_ground_visual",
    )
    brick_colors = (
        (0.72, 0.25, 0.13),
        (0.86, 0.38, 0.18),
        (0.64, 0.19, 0.10),
        (0.80, 0.31, 0.14),
    )
    for index, brick in enumerate(bricks):
        row, column = divmod(index, _WALL_COLUMNS)
        bridge.add_rigid_body_visual(
            brick,
            dart.BoxShape(_BRICK_SIZE),
            brick_colors[(row + column) % len(brick_colors)],
            name=f"{brick.name}_visual",
        )
    ball_colors = (
        (0.96, 0.82, 0.28),
        (0.41, 0.76, 0.95),
        (0.88, 0.54, 0.84),
    )
    for ball, color in zip(balls, ball_colors):
        bridge.add_rigid_body_visual(
            ball,
            dart.SphereShape(_BALL_RADIUS),
            color,
            name=f"{ball.name}_visual",
        )

    _last_metrics: dict[str, Any] = {}
    broken_history: deque[float] = deque(maxlen=180)
    retained_history: deque[float] = deque(maxlen=180)
    impact_band_history: tuple[deque[float], ...] = tuple(
        deque(maxlen=180) for _ in _IMPACT_TARGETS_XZ
    )

    def sample_metrics() -> dict[str, Any]:
        return compute_outcome_metrics(
            world=world,
            bricks=bricks,
            joints=joints,
            balls=balls,
            initial_brick_positions=initial_brick_positions,
            joint_evidence=joint_evidence,
            solver_family=solver_key,
            outcome_oracle=outcome_oracle,
        )

    def record_metrics() -> dict[str, Any]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        broken_history.append(float(_last_metrics["broken_joints"]))
        retained_history.append(float(_last_metrics["outside_retained_fraction"]))
        for history, count in zip(
            impact_band_history,
            _last_metrics["impact_band_displaced_counts"],
        ):
            history.append(float(count))
        return _last_metrics

    def capture_metrics() -> dict[str, Any]:
        metrics = record_metrics()
        rigid_constraint_options = world.rigid_constraint_options
        resolved_rigid_body = next(
            note["resolved"]
            for note in resolved_configuration
            if note["domain"] == "rigid-body"
        )
        view_report = None
        assessed_frames = {int(frame) for frame in capture_assessment_frames}
        if (
            view_assessment_available
            and metrics["frame"] in assessed_frames
        ):
            # capture_metrics() is also a public text-oracle callback, so it
            # can be invoked after direct World.step(n=...) calls that bypass
            # the demo runner's render-provider synchronization.
            bridge.sync()
            assert capture_camera is not None
            assessed_view = dart.gui.assess_view(
                bridge.render_world,
                capture_camera,
                _CAPTURE_SIZE,
                focus=capture_focus,
            )
            if not assessed_view.acceptable:
                raise RuntimeError(
                    f"{solver_display} breakable-wall capture view failed "
                    "assessment: "
                    f"{assessed_view.issues!r}"
                )
            view_report = assessed_view.to_json()
        return {
            "row": scene_id,
            "solver": f"public_{resolved_rigid_body}",
            "executor": "World.step default",
            "paper_locator": paper_reference["source_locator"],
            "time_step_ms": 1000.0 * _TIME_STEP,
            "rigid_constraint_options": {
                "iterations": int(rigid_constraint_options.iterations)
            },
            "rigid_body_solver": world.rigid_body_solver.name,
            "resolved_configuration": resolved_configuration,
            "scene_spec_fingerprint": scene_spec_fingerprint,
            "view_assessment_available": view_assessment_available,
            "view_report": view_report,
            "rigid_bodies": world.num_rigid_bodies,
            "collision_shapes": 1 + len(bricks) + len(balls),
            "brick_count": len(bricks),
            "ball_count": len(balls),
            "breakable_joints": len(joints),
            "break_force": _BREAK_FORCE,
            "outcome_oracle": dict(outcome_oracle),
            "outcome": metrics,
        }

    def replay_sync() -> None:
        bridge.sync()
        record_metrics()

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()

    bridge.sync()
    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        metrics = record_metrics()
        impact_bands = ", ".join(
            str(value) for value in metrics["impact_band_displaced_counts"]
        )
        builder.text("paper: Giles et al. 2025, Figure 13")
        builder.text(f"solver: public rigid {solver_display}")
        builder.text(f"iterations: {_RIGID_CONSTRAINT_ITERATIONS}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"bricks / breakable joints: {len(bricks)} / {len(joints)}")
        builder.text(
            f"broken / retained joints: {metrics['broken_joints']} / "
            f"{metrics['unbroken_joints']}"
        )
        builder.text(f"displaced bricks in impact bands: {impact_bands}")
        builder.text(
            "outside retained: "
            f"{100.0 * metrics['outside_retained_fraction']:.1f}%"
        )
        builder.text(
            "total retained: "
            f"{100.0 * metrics['total_retained_fraction']:.1f}%"
        )
        builder.text(f"outcome oracle: {metrics['status']}")
        builder.plot_lines("Broken joints", list(broken_history))
        builder.plot_lines("Outside retained", list(retained_history))
        for index, history in enumerate(impact_band_history, start=1):
            builder.plot_lines(f"Impact {index} displaced", list(history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(f"{solver_display} Paper Breakable Wall", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "bricks": tuple(bricks),
            "balls": tuple(balls),
            "joints": tuple(joints),
            "initial_brick_positions": initial_brick_positions.copy(),
            "paper_reference": paper_reference,
            "outcome_oracle": outcome_oracle,
            "resolved_configuration": tuple(resolved_configuration),
            "scene_spec_fingerprint": scene_spec_fingerprint,
            "outcome_metrics": sample_metrics,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


def build() -> SceneSetup:
    return build_solver_variant(
        rigid_body_solver=sx.RigidBodySolver.AVBD,
        solver_key="avbd",
        solver_display="AVBD",
        paper_reference=PAPER_REFERENCE,
        outcome_oracle=OUTCOME_ORACLE,
        capture_assessment_frames=(60, _OUTCOME_FRAME),
    )


SCENE = PythonDemoScene(
    id="avbd_paper_breakable_wall",
    title="AVBD Paper Breakable Wall (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A publication-shaped Figure 13 wall hit by three balls, with public "
    "AVBD contacts, 20 sweeps, and a quantitative retained-wall oracle.",
    build=build,
)
