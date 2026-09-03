#!/usr/bin/env python3
"""Write the SI row for DART's publication-shaped Figure 13 reconstruction."""

from __future__ import annotations

import argparse
import json
import struct
import sys
from hashlib import sha256
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import write_avbd_paper_breakable_wall_packet as shared  # noqa: E402
import write_avbd_paper_vbd_comparison_packet as comparison  # noqa: E402
from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    PAPER_PACKET_SOURCE_PATHS,
    make_resolved_solver_identity,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-sequential-impulse-comparison-packet.json"
)
DEFAULT_AVBD_VBD_PACKET = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-vbd-comparison-packet.json"
)

SCENE_ID = "sequential_impulse_paper_breakable_wall"
LINKED_PACKET_ID = "avbd_paper_vbd_comparison"
LINKED_SCENE_ID = comparison.SCENE_ID
PAPER_LOCATOR = comparison.PAPER_LOCATOR
TIME_STEP = comparison.TIME_STEP
RIGID_CONSTRAINT_ITERATIONS = comparison.RIGID_CONSTRAINT_ITERATIONS
BRICK_COUNT = comparison.BRICK_COUNT
RIGID_BODIES = comparison.RIGID_BODIES
COLLISION_SHAPES = comparison.COLLISION_SHAPES
BREAKABLE_JOINTS = comparison.BREAKABLE_JOINTS
IMPACTING_BALLS = comparison.IMPACTING_BALLS
BREAK_FORCE = comparison.BREAK_FORCE
CAMERA_VIEW = shared.CAMERA_VIEW
VIEW_FOCUS = tuple(
    f"sequential_impulse_paper_wall_brick_{row:02d}_{column:02d}_visual"
    for row in range(12)
    for column in range(21)
)

CHECKPOINTS = {
    14: {
        "label": "fracture",
        "checkpoint": "fracture",
        "threshold_checks": {
            "finite_state": True,
            "fracture_activated": True,
            "initial_fracture_confined_to_impact_regions": True,
            "initial_fracture_covers_three_impacts": True,
            "initial_fracture_identity_matches": True,
            "initial_retained_joint_rows_bounded": True,
            "wall_initially_retained": True,
        },
    },
    120: {
        "label": "collapse",
        "checkpoint": "collapse",
        "threshold_checks": {
            "damage_in_three_impact_bands": True,
            "finite_state": True,
            "fracture_identity_unchanged": True,
            "initial_fracture_remains_visible": True,
            "outside_wall_collapses": True,
            "retained_rows_fail_outside_impacts": True,
            "wall_collapses": True,
        },
    },
    600: {
        "label": "long_horizon",
        "checkpoint": "collapse",
        "threshold_checks": {
            "damage_in_three_impact_bands": True,
            "finite_state": True,
            "fracture_identity_unchanged": True,
            "initial_fracture_remains_visible": True,
            "outside_wall_collapses": True,
            "retained_rows_fail_outside_impacts": True,
            "wall_collapses": True,
        },
    },
}

OUTCOME_ORACLE = {
    "impact_damage_displacement_threshold": 0.1,
    "retained_displacement_threshold": 0.5,
    "impact_band_radius": 0.85,
    "outside_radius": 1.15,
    "evaluation_frame": 14,
    "collapse_evaluation_frame": 120,
    "joint_evidence_frames": [14, 120, 600],
    "minimum_initial_broken_joints": 3,
    "minimum_initial_broken_joints_per_impact_region": 1,
    "maximum_initial_broken_joints": 20,
    "minimum_initial_unbroken_joints": 690,
    "minimum_initial_total_retained_fraction": 0.95,
    "maximum_initial_outside_joint_angular_residual_radians": 0.13,
    "maximum_initial_outside_joint_linear_residual": 0.04,
    "expected_broken_joints": 5,
    "expected_broken_joint_ids_sha256": (
        "c85184879b1b9036ff582731031fc49c56b4149e93ded45780b06352bd94d61d"
    ),
    "expected_outside_impact_unbroken_joint_count": 484,
    "minimum_final_broken_joints": 3,
    "maximum_final_broken_joints": 30,
    "minimum_final_unbroken_joints": 680,
    "minimum_displaced_bricks_per_impact_band": 10,
    "minimum_collapse_outside_joint_maximum_angular_residual_radians": 0.60,
    "minimum_collapse_outside_joint_maximum_linear_residual": 0.05,
    "minimum_collapse_outside_joint_rms_angular_residual_radians": 0.18,
    "minimum_collapse_outside_joint_rms_linear_residual": 0.015,
    "maximum_collapse_outside_retained_fraction": 0.35,
    "maximum_collapse_total_retained_fraction": 0.25,
    "minimum_collapse_wall_normal_displacement": 2.0,
}

EXPECTED_OUTCOMES = {
    14: {
        "broken_joints": 5,
        "impact_band_displaced_counts": [13, 14, 13],
        "outside_retained_fraction": 1.0,
        "total_retained_fraction": 1.0,
        "unbroken_joints": 707,
    },
    120: {
        "broken_joints": 5,
        "impact_band_displaced_counts": [13, 14, 13],
        "outside_retained_fraction": 0.26704545454545453,
        "total_retained_fraction": 0.1865079365079365,
        "unbroken_joints": 707,
    },
}

RESOLVED_SOLVER_IDENTITY = make_resolved_solver_identity(
    resolved_rigid_contact_family="sequential-impulse",
    rigid_point_joint_solver="sequential_impulse",
    avbd_rigid_contact_config_emplaced=False,
    rigid_contact_selection="contact_solver_method",
    recorded_from=(
        "engine World.resolved_configuration in both Sequential Impulse "
        "captures plus benchmark runtime resolved-configuration counters"
    ),
)

REPO_ROOT = SCRIPT_DIR.parent
SOURCE_PATHS = tuple(
    Path(path) for path in PAPER_PACKET_SOURCE_PATHS[DEFAULT_OUTPUT.name]
)


class AvbdPaperSequentialImpulseComparisonPacketError(RuntimeError):
    """Raised when inputs cannot support the cross-solver SI comparison."""


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument(
        "--fracture-capture-manifest",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--fracture-image-verdict-json",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--collapse-capture-manifest",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--collapse-image-verdict-json",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--long-horizon-capture-manifest",
        type=Path,
        required=True,
    )
    parser.add_argument(
        "--long-horizon-image-verdict-json",
        type=Path,
        required=True,
    )
    parser.add_argument("--visual-review-json", type=Path, required=True)
    parser.add_argument("--paper-pdf", type=Path, required=True)
    parser.add_argument("--paper-figure-image", type=Path, required=True)
    parser.add_argument(
        "--avbd-vbd-packet",
        type=Path,
        default=DEFAULT_AVBD_VBD_PACKET,
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _source_provenance() -> dict[str, Any]:
    combined = sha256()
    files = []
    for relative_path in SOURCE_PATHS:
        path = REPO_ROOT / relative_path
        try:
            payload = path.read_bytes()
        except FileNotFoundError as exc:
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"{relative_path}: source file not found"
            ) from exc
        encoded_path = relative_path.as_posix().encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)
        files.append(
            {
                "path": relative_path.as_posix(),
                "sha256": sha256(payload).hexdigest(),
            }
        )
    return {
        "algorithm": "sha256-length-prefixed-path-and-content-v1",
        "digest": combined.hexdigest(),
        "files": files,
    }


def _validate_resolved_configuration(
    metrics: dict[str, Any],
) -> list[dict[str, str]]:
    notes = metrics.get("resolved_configuration")
    if not isinstance(notes, list):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "scene metrics missing engine resolved_configuration"
        )
    normalized = []
    for index, note in enumerate(notes):
        if not isinstance(note, dict):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"resolved_configuration[{index}] must be an object"
            )
        normalized_note = {}
        for key in ("domain", "requested", "resolved", "reason"):
            value = note.get(key)
            if not isinstance(value, str) or not value:
                raise AvbdPaperSequentialImpulseComparisonPacketError(
                    f"resolved_configuration[{index}].{key} must be non-empty"
                )
            normalized_note[key] = value
        normalized.append(normalized_note)

    expected = {
        "rigid-body": ("sequential-impulse", "sequential-impulse"),
        "rigid-contact": ("sequential-impulse", "sequential-impulse"),
        "rigid-pair-constraint": (
            "sequential-impulse",
            "sequential-impulse",
        ),
        "rigid-constraint-iterations": (
            str(RIGID_CONSTRAINT_ITERATIONS),
            str(RIGID_CONSTRAINT_ITERATIONS),
        ),
    }
    for domain, (requested, resolved) in expected.items():
        matching = [
            note
            for note in normalized
            if note["domain"] == domain
            and note["requested"] == requested
            and note["resolved"] == resolved
        ]
        if len(matching) != 1:
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                "engine resolved_configuration must contain exactly one "
                f"{domain} {requested}->{resolved} note"
            )
    return normalized


def _validate_outcome(
    outcome: dict[str, Any],
    *,
    expected_frame: int,
) -> dict[str, Any]:
    checkpoint = CHECKPOINTS[expected_frame]
    expected_outcome = EXPECTED_OUTCOMES.get(expected_frame)
    shared._validate_finite_tree(outcome, f"frame {expected_frame} outcome")
    exact_fields: list[tuple[str, Any]] = [
        ("frame", expected_frame),
        ("evaluated", True),
        ("checkpoint", checkpoint["checkpoint"]),
        ("status", "pass"),
        ("thresholds_pass", True),
        ("threshold_checks", checkpoint["threshold_checks"]),
        ("last_step_iterations", RIGID_CONSTRAINT_ITERATIONS),
    ]
    if expected_outcome is not None:
        exact_fields.extend(
            (
                ("broken_joints", expected_outcome["broken_joints"]),
                ("unbroken_joints", expected_outcome["unbroken_joints"]),
                (
                    "impact_band_displaced_counts",
                    expected_outcome["impact_band_displaced_counts"],
                ),
            )
        )
    else:
        exact_fields.extend(
            (
                ("broken_joints", OUTCOME_ORACLE["expected_broken_joints"]),
                (
                    "unbroken_joints",
                    BREAKABLE_JOINTS - OUTCOME_ORACLE["expected_broken_joints"],
                ),
            )
        )
    for key, expected in exact_fields:
        shared._require_exact(
            outcome.get(key),
            expected,
            f"frame {expected_frame} outcome {key}",
        )
    if expected_outcome is not None:
        for key in ("outside_retained_fraction", "total_retained_fraction"):
            shared._require_close(
                outcome.get(key),
                expected_outcome[key],
                f"frame {expected_frame} outcome {key}",
            )
    shared._require_close(
        outcome.get("world_time"),
        expected_frame * TIME_STEP,
        f"frame {expected_frame} outcome world_time",
    )
    shared._validate_joint_evidence(
        outcome,
        expected_broken_count=OUTCOME_ORACLE["expected_broken_joints"],
        expected_broken_ids_sha256=OUTCOME_ORACLE["expected_broken_joint_ids_sha256"],
        expected_outside_unbroken_count=OUTCOME_ORACLE[
            "expected_outside_impact_unbroken_joint_count"
        ],
        label=f"frame {expected_frame} outcome",
    )
    shared._require_exact(
        outcome.get("broken_joint_impact_region_counts"),
        [2, 1, 2],
        f"frame {expected_frame} broken_joint_impact_region_counts",
    )
    shared._require_exact(
        outcome.get("broken_joints_outside_impact_regions"),
        0,
        f"frame {expected_frame} broken_joints_outside_impact_regions",
    )

    contact_count = shared._finite_number(
        outcome.get("contact_count"),
        f"frame {expected_frame} outcome contact_count",
    )
    if contact_count < 0.0:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"frame {expected_frame} outcome contact_count must be non-negative"
        )
    if expected_frame == 14:
        if (
            float(outcome["total_retained_fraction"])
            < OUTCOME_ORACLE["minimum_initial_total_retained_fraction"]
        ):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                "frame 14 initial wall-retention oracle failed"
            )
        initial_maximum_linear = shared._finite_number(
            outcome.get("maximum_outside_impact_unbroken_joint_linear_residual"),
            "frame 14 outside-impact maximum linear residual",
        )
        if (
            initial_maximum_linear
            > OUTCOME_ORACLE["maximum_initial_outside_joint_linear_residual"]
        ):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                "frame 14 outside-impact retained-joint linear residual "
                "exceeds the initial oracle"
            )
        initial_maximum_angular = shared._finite_number(
            outcome.get(
                "maximum_outside_impact_unbroken_joint_angular_residual_radians"
            ),
            "frame 14 outside-impact maximum angular residual",
        )
        if (
            initial_maximum_angular
            > OUTCOME_ORACLE["maximum_initial_outside_joint_angular_residual_radians"]
        ):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                "frame 14 outside-impact retained-joint angular residual "
                "exceeds the initial oracle"
            )
    else:
        wall_normal_displacement = shared._finite_number(
            outcome.get("maximum_wall_normal_displacement"),
            "frame 120 outcome maximum_wall_normal_displacement",
        )
        if (
            wall_normal_displacement
            < OUTCOME_ORACLE["minimum_collapse_wall_normal_displacement"]
        ):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                "frame 120 wall-collapse displacement oracle failed"
            )
        collapse_minimums = {
            "maximum_outside_impact_unbroken_joint_linear_residual": (
                OUTCOME_ORACLE["minimum_collapse_outside_joint_maximum_linear_residual"]
            ),
            "rms_outside_impact_unbroken_joint_linear_residual": (
                OUTCOME_ORACLE["minimum_collapse_outside_joint_rms_linear_residual"]
            ),
            (
                "maximum_outside_impact_unbroken_joint_angular_" "residual_radians"
            ): OUTCOME_ORACLE[
                "minimum_collapse_outside_joint_maximum_angular_residual_radians"
            ],
            (
                "rms_outside_impact_unbroken_joint_angular_residual_radians"
            ): OUTCOME_ORACLE[
                "minimum_collapse_outside_joint_rms_angular_residual_radians"
            ],
        }
        for key, minimum in collapse_minimums.items():
            actual = shared._finite_number(
                outcome.get(key),
                f"frame 120 outcome {key}",
            )
            if actual < minimum:
                raise AvbdPaperSequentialImpulseComparisonPacketError(
                    f"frame {expected_frame} outcome {key} must be >= "
                    f"{minimum}, got {actual}"
                )
        for key, maximum in (
            (
                "outside_retained_fraction",
                OUTCOME_ORACLE["maximum_collapse_outside_retained_fraction"],
            ),
            (
                "total_retained_fraction",
                OUTCOME_ORACLE["maximum_collapse_total_retained_fraction"],
            ),
        ):
            actual = shared._finite_number(
                outcome.get(key), f"frame {expected_frame} outcome {key}"
            )
            if actual > maximum:
                raise AvbdPaperSequentialImpulseComparisonPacketError(
                    f"frame {expected_frame} outcome {key} must be <= "
                    f"{maximum}, got {actual}"
                )
        displaced = outcome.get("impact_band_displaced_counts")
        minimum_displaced = OUTCOME_ORACLE["minimum_displaced_bricks_per_impact_band"]
        if (
            not isinstance(displaced, list)
            or len(displaced) != 3
            or any(
                not isinstance(value, int)
                or isinstance(value, bool)
                or value < minimum_displaced
                for value in displaced
            )
        ):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"frame {expected_frame} displaced-brick counts must satisfy "
                "all three collapse minima"
            )
    return outcome


def _validate_scene_metrics(
    manifest: dict[str, Any],
    *,
    expected_frame: int,
    height: int,
    logged_latest: dict[str, Any],
    width: int,
) -> dict[str, Any]:
    summary = manifest.get("scene_metrics")
    if not isinstance(summary, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture manifest missing scene_metrics"
        )
    shared._require_exact(
        summary.get("event_count"),
        expected_frame,
        "scene metric event_count",
    )
    latest = summary.get("latest")
    if not isinstance(latest, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture scene_metrics missing latest event"
        )
    shared._require_exact(
        latest,
        logged_latest,
        "manifest latest scene metric event",
    )
    shared._require_exact(latest.get("scene"), SCENE_ID, "latest scene")
    shared._require_exact(
        latest.get("frame"),
        expected_frame,
        "latest scene frame",
    )
    metrics = latest.get("metrics")
    if not isinstance(metrics, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture latest scene metric payload must be an object"
        )
    shared._validate_finite_tree(metrics, "scene metrics")

    exact_metrics = {
        "ball_count": IMPACTING_BALLS,
        "breakable_joints": BREAKABLE_JOINTS,
        "brick_count": BRICK_COUNT,
        "collision_shapes": COLLISION_SHAPES,
        "executor": "World.step default",
        "effective_scene_contract_passed": True,
        "paper_locator": PAPER_LOCATOR,
        "rigid_bodies": RIGID_BODIES,
        "rigid_body_solver": "SEQUENTIAL_IMPULSE",
        "row": SCENE_ID,
        "solver": "public_sequential-impulse",
    }
    for key, expected in exact_metrics.items():
        shared._require_exact(metrics.get(key), expected, f"scene metrics {key}")
    shared._require_close(
        metrics.get("break_force"),
        BREAK_FORCE,
        "scene break_force",
    )
    shared._require_close(
        metrics.get("time_step_ms"),
        TIME_STEP * 1000.0,
        "scene time_step_ms",
    )
    shared._require_exact(
        metrics.get("rigid_constraint_options"),
        {"iterations": RIGID_CONSTRAINT_ITERATIONS},
        "scene rigid_constraint_options",
    )

    oracle = metrics.get("outcome_oracle")
    if not isinstance(oracle, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "scene metrics missing outcome_oracle"
        )
    for key, expected in OUTCOME_ORACLE.items():
        if isinstance(expected, float):
            shared._require_close(oracle.get(key), expected, f"oracle {key}")
        else:
            shared._require_exact(oracle.get(key), expected, f"oracle {key}")

    outcome = metrics.get("outcome")
    if not isinstance(outcome, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "scene metrics missing outcome"
        )
    _validate_outcome(outcome, expected_frame=expected_frame)
    resolved = _validate_resolved_configuration(metrics)
    view_report = shared._validate_view_report(
        metrics,
        expected_focus=VIEW_FOCUS,
        width=width,
        height=height,
    )
    fingerprint = comparison._validate_scene_spec_fingerprint(
        metrics.get("scene_spec_fingerprint"),
        "scene metrics scene_spec_fingerprint",
    )
    return {
        "event_count": expected_frame,
        "frame": expected_frame,
        "outcome": outcome,
        "outcome_oracle": oracle,
        "resolved_configuration": resolved,
        "scene_contract": {
            key: metrics[key]
            for key in (
                "ball_count",
                "break_force",
                "breakable_joints",
                "brick_count",
                "collision_shapes",
                "executor",
                "effective_scene_contract_passed",
                "paper_locator",
                "rigid_bodies",
                "rigid_body_solver",
                "solver",
                "time_step_ms",
            )
        }
        | {"rigid_constraint_options": metrics["rigid_constraint_options"]},
        "scene_spec_fingerprint": fingerprint,
        "view_report": view_report,
    }


def _validate_capture(
    manifest_path: Path,
    *,
    expected_frame: int,
) -> tuple[dict[str, Any], Path, Path]:
    expected_label = CHECKPOINTS[expected_frame]["label"]
    manifest = shared._load_json(manifest_path)
    shared._require_exact(manifest.get("schema_version"), 1, "capture schema")
    shared._require_exact(manifest.get("scene"), SCENE_ID, "capture scene")
    shared._require_exact(
        manifest.get("capture_label"),
        expected_label,
        "capture label",
    )
    shared._require_exact(manifest.get("force_drag"), None, "capture force_drag")
    shared._require_exact(
        manifest.get("resolved_solver_identity"),
        {
            "executor": "World.step default",
            "solver": "public_sequential-impulse",
            "source": "scene_capture_metrics.latest.metrics",
        },
        "capture resolved_solver_identity",
    )
    capture = manifest.get("capture")
    if not isinstance(capture, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture manifest missing dimensions"
        )
    for key in ("requested_frames", "converted_frames"):
        shared._require_exact(
            capture.get(key),
            expected_frame,
            f"capture {key}",
        )
    width = capture.get("width")
    height = capture.get("height")
    if (
        not isinstance(width, int)
        or isinstance(width, bool)
        or not isinstance(height, int)
        or isinstance(height, bool)
        or width < 1
        or height < 1
    ):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture width and height must be positive integers"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture manifest missing artifacts"
        )
    screenshot = shared._artifact_path(
        manifest_path,
        artifacts.get("screenshot"),
        "artifacts.screenshot",
    )
    metrics_events = shared._artifact_path(
        manifest_path,
        artifacts.get("scene_metrics_events"),
        "artifacts.scene_metrics_events",
    )
    frames = shared._artifact_directory(
        manifest_path,
        artifacts.get("frames"),
        "artifacts.frames",
    )
    video = shared._artifact_path(
        manifest_path,
        artifacts.get("video"),
        "artifacts.video",
    )
    shared._require_exact(
        video.name,
        f"{SCENE_ID}_{expected_label}.mp4",
        "capture video filename",
    )
    if not screenshot.is_file() or not metrics_events.is_file():
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture screenshot and scene metric log must both exist"
        )
    if shared._png_dimensions(screenshot) != (width, height):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "capture screenshot dimensions do not match manifest"
        )
    events = shared._read_scene_metric_events(
        metrics_events,
        expected_frame=expected_frame,
        expected_focus=VIEW_FOCUS,
        expected_scene_id=SCENE_ID,
        height=height,
        width=width,
        assessed_frames=tuple(
            int(frame) for frame in OUTCOME_ORACLE["joint_evidence_frames"]
        ),
    )
    for checkpoint_frame in sorted(CHECKPOINTS):
        if checkpoint_frame > expected_frame:
            continue
        checkpoint_event = events[checkpoint_frame - 1]
        _validate_scene_metrics(
            {
                "scene_metrics": {
                    "event_count": checkpoint_frame,
                    "latest": checkpoint_event,
                }
            },
            expected_frame=checkpoint_frame,
            height=height,
            logged_latest=checkpoint_event,
            width=width,
        )
    scene_metrics = _validate_scene_metrics(
        manifest,
        expected_frame=expected_frame,
        height=height,
        logged_latest=events[-1],
        width=width,
    )
    (
        capture_source_provenance,
        capture_runtime_provenance,
        capture_artifacts,
    ) = shared._validate_capture_provenance(
        manifest,
        expected_frame_count=expected_frame,
        expected_width=width,
        expected_height=height,
        frames=frames,
        metrics_events=metrics_events,
        screenshot=screenshot,
        video=video,
    )
    return (
        {
            "artifact_provenance": capture_artifacts,
            "camera": shared._validate_camera(manifest),
            "capture": {
                "converted_frames": expected_frame,
                "height": height,
                "requested_frames": expected_frame,
                "width": width,
            },
            "label": expected_label,
            "manifest": {
                "file": manifest_path.name,
                "sha256": shared._sha256(manifest_path),
            },
            "scene_metrics": scene_metrics,
            "source_provenance": capture_source_provenance,
            "runtime_provenance": capture_runtime_provenance,
            "scene_metrics_events": {
                "event_count": len(events),
                "file": metrics_events.name,
                "prefix_sha256": {
                    str(frame): shared._scene_metric_prefix_digest(events, frame)
                    for frame in sorted(CHECKPOINTS)
                    if frame <= expected_frame
                },
                "sha256": shared._sha256(metrics_events),
            },
            "screenshot": {
                "file": screenshot.name,
                "sha256": shared._sha256(screenshot),
            },
        },
        screenshot,
        video,
    )


def _validate_benchmark(
    benchmark_path: Path,
    *,
    expected_fingerprint: str,
) -> dict[str, Any]:
    data = shared._load_json(benchmark_path)
    method = comparison._benchmark_rows(
        data,
        solver_key="sequential-impulse",
        expected_fingerprint=expected_fingerprint,
    )
    context = data.get("context")
    if not isinstance(context, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "benchmark JSON missing context object"
        )
    executable = context.get("executable")
    if (
        not isinstance(executable, str)
        or Path(executable).name != "bm_avbd_rigid_fixed_joint"
    ):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "benchmark context identifies the wrong executable"
        )
    shared._require_exact(
        context.get("library_build_type"),
        "release",
        "benchmark library_build_type",
    )
    raw_evidence = data.get("dart_evidence_run")
    raw_build_identity = (
        raw_evidence.get("build_identity") if isinstance(raw_evidence, dict) else None
    )
    loaded_dart_libraries = (
        raw_build_identity.get("loaded_dart_libraries")
        if isinstance(raw_build_identity, dict)
        else None
    )
    build_configuration = (
        raw_build_identity.get("build_configuration")
        if isinstance(raw_build_identity, dict)
        else None
    )
    runtime_image_inventory = (
        raw_build_identity.get("runtime_image_inventory")
        if isinstance(raw_build_identity, dict)
        else None
    )
    benchmark_source_provenance = shared._validate_benchmark_source_provenance(
        context,
        loaded_dart_libraries=loaded_dart_libraries,
        runtime_image_inventory=runtime_image_inventory,
        build_configuration=build_configuration,
    )
    run_evidence = shared._validate_benchmark_run_evidence(
        data,
        context=context,
        source_provenance=benchmark_source_provenance,
    )
    return {
        "context": {
            key: context[key]
            for key in (
                "executable",
                "host_name",
                "json_schema_version",
                "library_build_type",
                "library_version",
                "mhz_per_cpu",
                "num_cpus",
                "date",
                "dart_benchmark_executable_path",
                "dart_benchmark_source_sha256",
                "dart_build_configuration_digest",
                "dart_capture_source_git_head",
                "dart_capture_source_provenance_digest",
                "dart_cmake_build_type",
                "dart_compiler_id",
                "dart_compiler_version",
                "dart_ndebug",
                "dart_optimization_enabled",
            )
            if key in context
        },
        "json_sha256": shared._sha256(benchmark_path),
        "method": method,
        "scene_spec_fingerprint": expected_fingerprint,
        "source_provenance": benchmark_source_provenance,
        "run_evidence": run_evidence,
    }


def _validate_linked_source_provenance(
    provenance: object,
    *,
    repo_root: Path = comparison.REPO_ROOT,
    source_paths: tuple[Path, ...] = comparison.SOURCE_PATHS,
    label: str = "linked AVBD/VBD",
) -> dict[str, Any]:
    if not isinstance(provenance, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"{label} packet lacks source provenance"
        )
    shared._require_exact(
        provenance.get("algorithm"),
        "sha256-length-prefixed-path-and-content-v1",
        f"{label} source provenance algorithm",
    )
    files = provenance.get("files")
    if not isinstance(files, list):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"{label} source provenance must list files"
        )
    expected_paths = [relative.as_posix() for relative in source_paths]
    actual_paths = [
        entry.get("path") if isinstance(entry, dict) else None for entry in files
    ]
    shared._require_exact(
        actual_paths,
        expected_paths,
        f"{label} source provenance paths",
    )

    combined = sha256()
    normalized_files = []
    for entry, relative in zip(files, source_paths):
        if not isinstance(entry, dict):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"{label} source provenance entry must be an object"
            )
        path = repo_root / relative
        try:
            payload = path.read_bytes()
        except FileNotFoundError as exc:
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"{label} source provenance file not found: {relative}"
            ) from exc
        expected_hash = sha256(payload).hexdigest()
        shared._require_exact(
            entry.get("sha256"),
            expected_hash,
            f"{label} source provenance {relative} sha256",
        )
        encoded_path = relative.as_posix().encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)
        normalized_files.append(
            {
                "path": relative.as_posix(),
                "sha256": expected_hash,
            }
        )
    shared._require_exact(
        provenance.get("digest"),
        combined.hexdigest(),
        f"{label} source provenance digest",
    )
    return {
        "algorithm": "sha256-length-prefixed-path-and-content-v1",
        "digest": combined.hexdigest(),
        "files": normalized_files,
    }


def _validate_linked_image_binding(
    capture: object,
    *,
    label: str,
) -> dict[str, str]:
    if not isinstance(capture, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"{label} visual evidence is missing"
        )
    screenshot = capture.get("screenshot")
    if not isinstance(screenshot, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"{label} evidence lacks a screenshot"
        )
    screenshot_file = screenshot.get("file")
    if not isinstance(screenshot_file, str) or not screenshot_file:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"{label} screenshot file must be non-empty"
        )
    screenshot_hash = comparison._validate_sha256_value(
        screenshot.get("sha256"),
        f"{label} screenshot sha256",
    )
    image_verdict = capture.get("image_verdict")
    if not isinstance(image_verdict, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"{label} image verdict is missing"
        )
    shared._require_exact(
        image_verdict.get("pass"),
        True,
        f"{label} image verdict pass",
    )
    comparison._validate_sha256_value(
        image_verdict.get("sha256"),
        f"{label} image verdict sha256",
    )
    image_sha256 = comparison._validate_sha256_value(
        image_verdict.get("image_sha256"),
        f"{label} image verdict image_sha256",
    )
    shared._require_exact(
        image_sha256,
        screenshot_hash,
        f"{label} image verdict screenshot binding",
    )
    return {
        "file": screenshot_file,
        "sha256": screenshot_hash,
    }


def _validate_linked_visual_capture(
    capture: object,
    *,
    expected_frame: int,
    expected_fingerprint: str,
    label: str,
) -> dict[str, Any]:
    screenshot = _validate_linked_image_binding(
        capture,
        label=f"linked AVBD/VBD {label}",
    )
    assert isinstance(capture, dict)
    scene_metrics = capture.get("scene_metrics")
    if not isinstance(scene_metrics, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"linked AVBD/VBD {label} evidence lacks scene metrics"
        )
    shared._require_exact(
        scene_metrics.get("frame"),
        expected_frame,
        f"linked AVBD/VBD {label} frame",
    )
    shared._require_exact(
        scene_metrics.get("scene_spec_fingerprint"),
        expected_fingerprint,
        f"linked AVBD/VBD {label} scene fingerprint",
    )
    outcome = scene_metrics.get("outcome")
    if not isinstance(outcome, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"linked AVBD/VBD {label} evidence lacks outcome"
        )
    try:
        comparison._validate_outcome(outcome, expected_frame=expected_frame)
    except (
        comparison.AvbdPaperVbdComparisonPacketError,
        shared.AvbdPaperBreakableWallPacketError,
    ) as error:
        detail = str(error).removeprefix(f"frame {expected_frame} outcome ")
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"linked AVBD/VBD {label} outcome {detail}"
        ) from error
    return {
        "screenshot": screenshot,
        "frame": expected_frame,
    }


def _validate_linked_avbd_vbd_packet(
    packet_path: Path,
    *,
    benchmark_sha256: str,
    expected_run_evidence: dict[str, Any],
    expected_fingerprint: str,
) -> dict[str, Any]:
    packet = shared._load_json(packet_path)
    shared._require_exact(
        packet.get("schema_version"),
        AVBD_PACKET_SCHEMA_VERSION,
        "linked AVBD/VBD packet schema_version",
    )
    shared._require_exact(
        packet.get("packet"),
        LINKED_PACKET_ID,
        "linked AVBD/VBD packet identity",
    )
    shared._require_exact(
        packet.get("scene"),
        LINKED_SCENE_ID,
        "linked AVBD/VBD packet scene",
    )

    identity = packet.get("resolved_solver_identity")
    if not isinstance(identity, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD packet lacks resolved_solver_identity"
        )
    shared._require_exact(
        identity.get("rigid_contact_solver"),
        "vbd",
        "linked AVBD/VBD rigid contact solver",
    )
    shared._require_exact(
        identity.get("rigid_point_joint_solver"),
        "vbd",
        "linked AVBD/VBD point-joint solver",
    )

    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD packet lacks benchmark"
        )
    shared._require_exact(
        benchmark.get("json_sha256"),
        benchmark_sha256,
        "linked AVBD/VBD benchmark JSON hash",
    )
    shared._require_exact(
        benchmark.get("scene_spec_fingerprint"),
        expected_fingerprint,
        "linked AVBD/VBD scene fingerprint",
    )
    shared._require_exact(
        benchmark.get("run_evidence"),
        expected_run_evidence,
        "linked AVBD/VBD benchmark run/host/build evidence",
    )
    methods = benchmark.get("methods")
    if not isinstance(methods, dict) or set(methods) != {"avbd", "vbd"}:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD benchmark must contain AVBD and VBD methods"
        )
    method_timings = {}
    for solver_key in ("avbd", "vbd"):
        method = methods.get(solver_key)
        if not isinstance(method, dict):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"linked {solver_key.upper()} benchmark method is missing"
            )
        shared._require_exact(
            method.get("benchmark"),
            comparison.BENCHMARK_RUNS[solver_key].removesuffix("/iterations:120"),
            f"linked {solver_key.upper()} benchmark identity",
        )
        timing = method.get("timing")
        if not isinstance(timing, dict):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"linked {solver_key.upper()} benchmark timing is missing"
            )
        median_cpu = shared._finite_number(
            timing.get("median_cpu_time_per_step_ns"),
            f"linked {solver_key.upper()} median CPU time",
        )
        if median_cpu <= 0.0:
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"linked {solver_key.upper()} median CPU time must be positive"
            )
        method_timings[solver_key] = {
            "median_cpu_time_per_step_ns": median_cpu,
        }

    correctness = packet.get("correctness")
    if not isinstance(correctness, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD packet lacks correctness evidence"
        )
    shared._require_exact(
        correctness.get("determinism_test"),
        "test_vbd_paper_breakable_wall_checkpoints_are_deterministic",
        "linked AVBD/VBD determinism test",
    )
    shared._require_exact(
        correctness.get("outcome_test"),
        "test_vbd_paper_breakable_wall_matches_figure13_contract",
        "linked AVBD/VBD outcome test",
    )

    linked_avbd = packet.get("linked_avbd_evidence")
    if not isinstance(linked_avbd, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD packet lacks linked AVBD evidence"
        )
    linked_avbd_file = linked_avbd.get("file")
    if (
        not isinstance(linked_avbd_file, str)
        or not linked_avbd_file
        or Path(linked_avbd_file).name != linked_avbd_file
    ):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "nested linked AVBD file must name one sibling packet"
        )
    linked_avbd_path = packet_path.parent / linked_avbd_file
    if not linked_avbd_path.is_file():
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            f"nested linked AVBD packet not found: {linked_avbd_file}"
        )
    linked_avbd_hash = shared._sha256(linked_avbd_path)
    shared._require_exact(
        comparison._validate_sha256_value(
            linked_avbd.get("sha256"),
            "nested linked AVBD packet sha256",
        ),
        linked_avbd_hash,
        "nested linked AVBD current file sha256",
    )
    linked_avbd_packet = shared._load_json(linked_avbd_path)
    shared._require_exact(
        linked_avbd_packet.get("schema_version"),
        AVBD_PACKET_SCHEMA_VERSION,
        "nested linked AVBD packet schema_version",
    )
    shared._require_exact(
        linked_avbd_packet.get("packet"),
        "avbd_paper_breakable_wall",
        "nested linked AVBD packet identity",
    )
    shared._require_exact(
        linked_avbd_packet.get("scene"),
        shared.SCENE_ID,
        "nested linked AVBD scene",
    )
    linked_avbd_visual = linked_avbd_packet.get("visual_evidence")
    if not isinstance(linked_avbd_visual, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "nested linked AVBD packet lacks visual evidence"
        )
    for label in ("impact", "outcome", "long_horizon"):
        _validate_linked_image_binding(
            linked_avbd_visual.get(label),
            label=f"nested linked AVBD {label}",
        )
    linked_avbd_benchmark = linked_avbd_packet.get("benchmark")
    if not isinstance(linked_avbd_benchmark, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "nested linked AVBD packet lacks benchmark"
        )
    shared._require_exact(
        linked_avbd_benchmark.get("scene_spec_fingerprint"),
        expected_fingerprint,
        "nested linked AVBD scene fingerprint",
    )
    shared._require_exact(
        linked_avbd_benchmark.get("run_evidence"),
        expected_run_evidence,
        "nested linked AVBD benchmark run/host/build evidence",
    )
    shared._require_exact(
        linked_avbd.get("scene_spec_fingerprint"),
        expected_fingerprint,
        "nested linked AVBD scene fingerprint",
    )
    linked_avbd_identity = linked_avbd.get("resolved_solver_identity")
    if not isinstance(linked_avbd_identity, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "nested linked AVBD evidence lacks resolved_solver_identity"
        )
    shared._require_exact(
        linked_avbd_identity.get("rigid_contact_solver"),
        "avbd",
        "nested linked AVBD rigid contact solver",
    )
    shared._require_exact(
        linked_avbd_identity.get("rigid_point_joint_solver"),
        "avbd",
        "nested linked AVBD point-joint solver",
    )
    shared._require_exact(
        linked_avbd_packet.get("resolved_solver_identity"),
        linked_avbd_identity,
        "nested linked AVBD packet resolved_solver_identity",
    )
    linked_avbd_provenance = _validate_linked_source_provenance(
        linked_avbd_packet.get("source_provenance"),
        repo_root=shared.REPO_ROOT,
        source_paths=shared.SOURCE_PATHS,
        label="nested linked AVBD",
    )

    visual = packet.get("visual_evidence")
    if not isinstance(visual, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD packet lacks visual evidence"
        )
    validated_visuals = {
        "bend": _validate_linked_visual_capture(
            visual.get("bend"),
            expected_frame=18,
            expected_fingerprint=expected_fingerprint,
            label="bend",
        ),
        "retention": _validate_linked_visual_capture(
            visual.get("retention"),
            expected_frame=120,
            expected_fingerprint=expected_fingerprint,
            label="retention",
        ),
        "long_horizon": _validate_linked_visual_capture(
            visual.get("long_horizon"),
            expected_frame=600,
            expected_fingerprint=expected_fingerprint,
            label="long_horizon",
        ),
    }
    semantic = visual.get("semantic_review")
    if not isinstance(semantic, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD semantic review is missing"
        )
    shared._require_exact(
        semantic.get("verdict"),
        "pass",
        "linked AVBD/VBD semantic review verdict",
    )
    semantic_hash = comparison._validate_sha256_value(
        semantic.get("sha256"),
        "linked AVBD/VBD semantic review sha256",
    )
    inspected_images = semantic.get("inspected_images")
    if not isinstance(inspected_images, list):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD semantic review must list inspected images"
        )
    by_role = {
        entry.get("role"): entry
        for entry in inspected_images
        if isinstance(entry, dict) and isinstance(entry.get("role"), str)
    }
    paper_reference = packet.get("paper_reference")
    paper_figure = (
        paper_reference.get("figure") if isinstance(paper_reference, dict) else None
    )
    if not isinstance(paper_figure, dict):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD packet lacks paper figure evidence"
        )
    paper_figure_file = paper_figure.get("file")
    if not isinstance(paper_figure_file, str) or not paper_figure_file:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD paper figure file must be non-empty"
        )
    paper_figure_hash = comparison._validate_sha256_value(
        paper_figure.get("sha256"),
        "linked AVBD/VBD paper figure sha256",
    )
    expected_roles = {
        "bend_frame_18",
        "retention_frame_120",
        "long_horizon_frame_600",
        "paper_figure_13_reference",
    }
    if set(by_role) != expected_roles:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "linked AVBD/VBD semantic review must inspect bend, retention, "
            "long-horizon, and paper Figure 13 images"
        )
    for role, visual_key in (
        ("bend_frame_18", "bend"),
        ("retention_frame_120", "retention"),
        ("long_horizon_frame_600", "long_horizon"),
    ):
        expected_screenshot = validated_visuals[visual_key]["screenshot"]
        entry = by_role.get(role)
        if not isinstance(entry, dict):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"linked AVBD/VBD semantic review lacks {role}"
            )
        shared._require_exact(
            entry.get("file"),
            expected_screenshot["file"],
            f"linked AVBD/VBD semantic review {role} file",
        )
        shared._require_exact(
            entry.get("sha256"),
            expected_screenshot["sha256"],
            f"linked AVBD/VBD semantic review {role} sha256",
        )
    paper_entry = by_role["paper_figure_13_reference"]
    shared._require_exact(
        paper_entry.get("file"),
        paper_figure_file,
        "linked AVBD/VBD semantic review paper figure file",
    )
    shared._require_exact(
        paper_entry.get("sha256"),
        paper_figure_hash,
        "linked AVBD/VBD semantic review paper figure sha256",
    )

    provenance = _validate_linked_source_provenance(packet.get("source_provenance"))
    return {
        "benchmark_method_timings": method_timings,
        "file": packet_path.name,
        "nested_avbd_resolved_solver_identity": linked_avbd_identity,
        "nested_avbd_sha256": linked_avbd_hash,
        "nested_avbd_source_provenance_digest": linked_avbd_provenance["digest"],
        "resolved_solver_identity": identity,
        "scene_spec_fingerprint": expected_fingerprint,
        "sha256": shared._sha256(packet_path),
        "source_provenance_digest": provenance["digest"],
        "visual_evidence": {
            "bend_screenshot": validated_visuals["bend"]["screenshot"],
            "retention_screenshot": validated_visuals["retention"]["screenshot"],
            "long_horizon_screenshot": validated_visuals["long_horizon"]["screenshot"],
            "semantic_review": {
                "sha256": semantic_hash,
                "verdict": "pass",
            },
        },
    }


def _validate_visual_review(
    review_path: Path,
    *,
    fracture_screenshot: Path,
    collapse_screenshot: Path,
    long_horizon_screenshot: Path,
    long_horizon_video: dict[str, Any],
    long_horizon_video_path: Path,
    paper_figure: Path,
) -> dict[str, Any]:
    review = shared._load_json(review_path)
    expected_review_keys = {
        "assessment_assertions",
        "claim_assessments",
        "inspected_images",
        "inspected_videos",
        "reviewer_capabilities",
        "scene",
        "schema_version",
        "structured_observations",
        "temporal_assessment",
        "verdict",
    }
    if set(review) != expected_review_keys:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "visual review must use the exact structured semantic-review fields"
        )
    shared._require_exact(
        review.get("schema_version"),
        "dart.visual_semantic_review/v1",
        "visual review schema_version",
    )
    shared._require_exact(review.get("scene"), SCENE_ID, "visual review scene")
    shared._require_exact(
        review.get("verdict"),
        "pass",
        "visual review verdict",
    )
    expected_capabilities = {
        "image_semantic_review": True,
        "video_semantic_review": True,
    }
    shared._require_exact(
        review.get("reviewer_capabilities"),
        expected_capabilities,
        "visual review reviewer_capabilities",
    )
    expected_assertions = {
        "capture_images_assessed": True,
        "long_horizon_video_assessed": True,
        "no_contradictions_found": True,
        "paper_reference_assessed": True,
        "text_oracle_agrees": True,
        "view_reports_agree": True,
    }
    shared._require_exact(
        review.get("assessment_assertions"),
        expected_assertions,
        "visual review assessment_assertions",
    )
    claim_assessments, temporal_assessment, structured_observations = (
        shared._validate_semantic_claim_contract(
            review,
            expected_terminal_behavior="collapsed_wall",
            error_type=AvbdPaperSequentialImpulseComparisonPacketError,
        )
    )

    expected = {
        "fracture_frame_14": fracture_screenshot,
        "collapse_frame_120": collapse_screenshot,
        "long_horizon_frame_600": long_horizon_screenshot,
        "paper_figure_13_reference": paper_figure,
    }
    entries = review.get("inspected_images")
    if not isinstance(entries, list):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "visual review inspected_images must be a list"
        )
    by_role = {
        entry.get("role"): entry
        for entry in entries
        if isinstance(entry, dict) and isinstance(entry.get("role"), str)
    }
    if set(by_role) != set(expected):
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "visual review must inspect fracture, collapse, long-horizon, "
            "and paper images"
        )
    inspected = []
    for role, path in expected.items():
        entry = by_role[role]
        file_value = entry.get("file")
        if (
            not isinstance(file_value, str)
            or Path(file_value).resolve() != path.resolve()
        ):
            raise AvbdPaperSequentialImpulseComparisonPacketError(
                f"visual review {role} file does not match inspected image"
            )
        expected_hash = shared._sha256(path)
        shared._require_exact(
            entry.get("sha256"),
            expected_hash,
            f"visual review {role} sha256",
        )
        inspected.append(
            {
                "file": path.name,
                "role": role,
                "sha256": expected_hash,
            }
        )
    video_entries = review.get("inspected_videos")
    if not isinstance(video_entries, list) or len(video_entries) != 1:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "visual review must inspect exactly one long-horizon video"
        )
    video_entry = video_entries[0]
    if not isinstance(video_entry, dict) or set(video_entry) != {
        "decoded_frame_count",
        "duration_seconds",
        "file",
        "role",
        "sha256",
    }:
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "visual review long-horizon video entry has unexpected fields"
        )
    expected_video_review = {
        "decoded_frame_count": long_horizon_video["decoded_frame_count"],
        "duration_seconds": long_horizon_video["duration_seconds"],
        "file": long_horizon_video["file"],
        "role": "long_horizon_video_600",
        "sha256": long_horizon_video["sha256"],
    }
    video_entry_path = Path(str(video_entry.get("file")))
    if video_entry_path.resolve() != long_horizon_video_path.resolve():
        raise AvbdPaperSequentialImpulseComparisonPacketError(
            "visual review long-horizon video file does not match capture"
        )
    video_entry = dict(video_entry)
    video_entry["file"] = Path(str(video_entry["file"])).name
    shared._require_exact(
        video_entry,
        expected_video_review,
        "visual review long-horizon video",
    )
    return {
        "assessment_assertions": expected_assertions,
        "claim_assessments": claim_assessments,
        "file": review_path.name,
        "inspected_images": inspected,
        "inspected_videos": [expected_video_review],
        "reviewer_capabilities": expected_capabilities,
        "sha256": shared._sha256(review_path),
        "structured_observations": structured_observations,
        "temporal_assessment": temporal_assessment,
        "verdict": "pass",
    }


def make_packet(
    *,
    benchmark_json: Path,
    fracture_capture_manifest: Path,
    fracture_image_verdict_json: Path,
    collapse_capture_manifest: Path,
    collapse_image_verdict_json: Path,
    long_horizon_capture_manifest: Path,
    long_horizon_image_verdict_json: Path,
    visual_review_json: Path,
    paper_pdf: Path,
    paper_figure_image: Path,
    avbd_vbd_packet: Path,
) -> dict[str, Any]:
    fracture, fracture_screenshot, _fracture_video = _validate_capture(
        fracture_capture_manifest,
        expected_frame=14,
    )
    fracture["image_verdict"] = shared._validate_image_verdict(
        fracture_image_verdict_json,
        fracture_screenshot,
        expected_frame=14,
        expected_scene_id=SCENE_ID,
    )
    collapse, collapse_screenshot, _collapse_video = _validate_capture(
        collapse_capture_manifest,
        expected_frame=120,
    )
    collapse["image_verdict"] = shared._validate_image_verdict(
        collapse_image_verdict_json,
        collapse_screenshot,
        expected_frame=120,
        expected_scene_id=SCENE_ID,
    )
    (
        long_horizon,
        long_horizon_screenshot,
        long_horizon_video_path,
    ) = _validate_capture(
        long_horizon_capture_manifest,
        expected_frame=600,
    )
    long_horizon["image_verdict"] = shared._validate_image_verdict(
        long_horizon_image_verdict_json,
        long_horizon_screenshot,
        expected_frame=600,
        expected_scene_id=SCENE_ID,
    )
    shared._require_exact(
        collapse["scene_metrics_events"]["prefix_sha256"]["14"],
        fracture["scene_metrics_events"]["prefix_sha256"]["14"],
        "fracture/collapse exact scene-metric event prefix",
    )
    for frame in (14, 120):
        shared._require_exact(
            long_horizon["scene_metrics_events"]["prefix_sha256"][str(frame)],
            collapse["scene_metrics_events"]["prefix_sha256"][str(frame)],
            f"collapse/long-horizon exact {frame}-frame scene-metric prefix",
        )
    fingerprint = fracture["scene_metrics"]["scene_spec_fingerprint"]
    shared._require_exact(
        collapse["scene_metrics"]["scene_spec_fingerprint"],
        fingerprint,
        "fracture/collapse scene fingerprint",
    )
    shared._require_exact(
        long_horizon["scene_metrics"]["scene_spec_fingerprint"],
        fingerprint,
        "fracture/long-horizon scene fingerprint",
    )
    shared._require_exact(
        long_horizon["scene_metrics"]["outcome_oracle"],
        fracture["scene_metrics"]["outcome_oracle"],
        "fracture/long-horizon outcome oracle",
    )
    benchmark = _validate_benchmark(
        benchmark_json,
        expected_fingerprint=fingerprint,
    )
    shared._require_shared_capture_benchmark_build(
        benchmark,
        fracture,
        collapse,
        long_horizon,
    )
    linked_avbd_vbd = _validate_linked_avbd_vbd_packet(
        avbd_vbd_packet,
        benchmark_sha256=benchmark["json_sha256"],
        expected_run_evidence=benchmark["run_evidence"],
        expected_fingerprint=fingerprint,
    )
    paper_reference = shared._validate_paper_artifacts(
        paper_pdf,
        paper_figure_image,
    )
    semantic_review = _validate_visual_review(
        visual_review_json,
        fracture_screenshot=fracture_screenshot,
        collapse_screenshot=collapse_screenshot,
        long_horizon_screenshot=long_horizon_screenshot,
        long_horizon_video=long_horizon["artifact_provenance"]["video"],
        long_horizon_video_path=long_horizon_video_path,
        paper_figure=paper_figure_image,
    )

    si_median = float(benchmark["method"]["timing"]["median_cpu_time_per_step_ns"])
    linked_timings = linked_avbd_vbd["benchmark_method_timings"]
    benchmark["comparison"] = {
        "basis": (
            "same benchmark JSON, executable, host, reconstructed scene "
            "fingerprint, trajectory length, time step, and 20-sweep budget"
        ),
        "sequential_impulse_to_avbd_median_cpu_cost_ratio": (
            si_median / float(linked_timings["avbd"]["median_cpu_time_per_step_ns"])
        ),
        "sequential_impulse_to_vbd_median_cpu_cost_ratio": (
            si_median / float(linked_timings["vbd"]["median_cpu_time_per_step_ns"])
        ),
    }

    return {
        "benchmark": benchmark,
        "claim_boundary": (
            "This packet covers the Sequential Impulse row of DART's "
            "cross-solver-matched, publication-shaped Figure 13 reconstruction "
            "and links the same-host AVBD/VBD comparison packet. Together "
            "they establish three DART solver variants on one reconstructed "
            "scene, not three source-equivalent published rows. The packet "
            "does not reproduce XPBD, claim exact unpublished source-scene "
            "constants, establish CUDA parity, or compare against a published/"
            "source timing."
        ),
        "correctness": {
            "determinism_test": (
                "test_sequential_impulse_paper_breakable_wall_is_deterministic"
            ),
            "outcome_test": (
                "test_sequential_impulse_paper_breakable_wall_matches_"
                "figure13_contract"
            ),
            "public_configuration": {
                "rigid_body_solver": "SEQUENTIAL_IMPULSE",
                "rigid_constraint_options": {
                    "iterations": RIGID_CONSTRAINT_ITERATIONS,
                },
                "scene_spec_fingerprint": fingerprint,
                "time_step": TIME_STEP,
            },
            "solver_kernel_tests": [
                ("SequentialImpulseContact." "HardFixedJointCancelsRelativeVelocity"),
                (
                    "SequentialImpulseContact."
                    "ContactAndFixedJointRowsConvergeTogether"
                ),
                (
                    "SequentialImpulseContact."
                    "FixedJointBreaksFromImpulseLoadAndStaysExcluded"
                ),
                (
                    "SequentialImpulseContact."
                    "PublicFixedJointPostStabilizesWithoutVelocityInjection"
                ),
            ],
        },
        "linked_avbd_vbd_evidence": linked_avbd_vbd,
        "packet": "avbd_paper_sequential_impulse_comparison",
        "paper_reference": paper_reference,
        "performance_claim_boundary": (
            "The ratios are descriptive same-host CPU costs for three public "
            "DART families running the identical reconstructed scene and "
            "20-sweep budget. They are not paper/reference speedup claims."
        ),
        "publication_observation": (
            "The Figure 13 Sequential Impulse row shows a clean initial "
            "fracture followed by retained-constraint failure, slow bending, "
            "and collapse."
        ),
        "reconstruction": {
            "matched_with_avbd_and_vbd": {
                "break_force": BREAK_FORCE,
                "breakable_joints": BREAKABLE_JOINTS,
                "bricks": BRICK_COUNT,
                "impacting_balls": IMPACTING_BALLS,
                "scene_spec_fingerprint": fingerprint,
            },
            "published": {
                "constraint_iterations_or_substeps": (RIGID_CONSTRAINT_ITERATIONS),
                "paper_locator": PAPER_LOCATOR,
                "time_step": TIME_STEP,
            },
            "unpublished": (
                "wall dimensions, body materials, ball properties and launch "
                "conditions, break threshold, and exact source scene"
            ),
        },
        "reproduction": {
            "benchmark_command": shared._benchmark_reproduction_command(
                "^BM_(Avbd|Vbd|SequentialImpulse)"
                "PaperBreakableWallStep/iterations:120$"
            ),
            "capture_commands": [
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 14 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label fracture --video --fps 60 "
                    "--output-dir <fracture-capture-dir>"
                ),
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 120 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label collapse --video --fps 60 "
                    "--output-dir <collapse-capture-dir>"
                ),
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 600 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label long_horizon --video --fps 60 "
                    "--output-dir <long-horizon-capture-dir>"
                ),
            ],
            "image_verdict_commands": [
                (
                    "pixi run image-verdict -- "
                    f"<fracture-capture-dir>/{SCENE_ID}_fracture.png "
                    f"--meta scene={SCENE_ID} --meta frame=14 "
                    f"--meta view={CAMERA_VIEW} "
                    "--out <fracture-capture-dir>/image_verdict.json"
                ),
                (
                    "pixi run image-verdict -- "
                    f"<collapse-capture-dir>/{SCENE_ID}_collapse.png "
                    f"--meta scene={SCENE_ID} --meta frame=120 "
                    f"--meta view={CAMERA_VIEW} "
                    "--out <collapse-capture-dir>/image_verdict.json"
                ),
                (
                    "pixi run image-verdict -- "
                    f"<long-horizon-capture-dir>/{SCENE_ID}_long_horizon.png "
                    f"--meta scene={SCENE_ID} --meta frame=600 "
                    f"--meta view={CAMERA_VIEW} "
                    "--out <long-horizon-capture-dir>/image_verdict.json"
                ),
            ],
        },
        "resolved_solver_identity": RESOLVED_SOLVER_IDENTITY,
        "schema_version": AVBD_PACKET_SCHEMA_VERSION,
        "scene": SCENE_ID,
        "source_provenance": _source_provenance(),
        "target": {
            "complete_paper_reproduction": False,
            "contract_rows": ["avbd.paper.fig.13"],
            "covered_slice": (
                "matched CPU Sequential Impulse fracture/collapse row plus "
                "linked AVBD and VBD numeric, performance, and assessed "
                "visual evidence"
            ),
        },
        "visual_evidence": {
            "collapse": collapse,
            "fracture": fracture,
            "long_horizon": long_horizon,
            "semantic_review": semantic_review,
        },
    }


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(
            benchmark_json=args.benchmark_json,
            fracture_capture_manifest=args.fracture_capture_manifest,
            fracture_image_verdict_json=args.fracture_image_verdict_json,
            collapse_capture_manifest=args.collapse_capture_manifest,
            collapse_image_verdict_json=args.collapse_image_verdict_json,
            long_horizon_capture_manifest=args.long_horizon_capture_manifest,
            long_horizon_image_verdict_json=(args.long_horizon_image_verdict_json),
            visual_review_json=args.visual_review_json,
            paper_pdf=args.paper_pdf,
            paper_figure_image=args.paper_figure_image,
            avbd_vbd_packet=args.avbd_vbd_packet,
        )
    except (
        OSError,
        ValueError,
        shared.AvbdPaperBreakableWallPacketError,
        comparison.AvbdPaperVbdComparisonPacketError,
        AvbdPaperSequentialImpulseComparisonPacketError,
    ) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(packet, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
