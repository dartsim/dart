#!/usr/bin/env python3
"""Write the matched fixed-penalty VBD row for AVBD paper Figure 13."""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
from hashlib import sha256
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import write_avbd_paper_breakable_wall_packet as shared  # noqa: E402
from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    PAPER_PACKET_SOURCE_PATHS,
    make_resolved_solver_identity,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-vbd-comparison-packet.json"
)
DEFAULT_AVBD_PACKET = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-breakable-wall-packet.json"
)

SCENE_ID = "vbd_paper_breakable_wall"
AVBD_SCENE_ID = "avbd_paper_breakable_wall"
PAPER_LOCATOR = "Section 5.4 and Figure 13, PDF page 10"
TIME_STEP = 1.0 / 60.0
RIGID_CONSTRAINT_ITERATIONS = 20
BRICK_COUNT = 252
RIGID_BODIES = 256
COLLISION_SHAPES = 256
BREAKABLE_JOINTS = 712
IMPACTING_BALLS = 3
BREAK_FORCE = shared.BREAK_FORCE
CAMERA_DISTANCE = shared.CAMERA_DISTANCE
CAMERA_TARGET = shared.CAMERA_TARGET
CAMERA_VIEW = shared.CAMERA_VIEW
VIEW_FOCUS = tuple(
    f"vbd_paper_wall_brick_{row:02d}_{column:02d}_visual"
    for row in range(12)
    for column in range(21)
)

CHECKPOINTS = {
    18: {
        "label": "bend",
        "checkpoint": "bend",
        "threshold_checks": {
            "bend_is_spatially_resolved": True,
            "finite_state": True,
            "no_fracture": True,
            "retained_joint_rows_satisfied": True,
            "topology_retained": True,
            "wall_bend_is_distributed": True,
            "wall_bends": True,
        },
    },
    120: {
        "label": "retention",
        "checkpoint": "retention",
        "threshold_checks": {
            "finite_state": True,
            "no_fracture": True,
            "retained_joint_rows_satisfied": True,
            "topology_retained": True,
            "wall_retained": True,
        },
    },
}

OUTCOME_ORACLE = {
    "impact_damage_displacement_threshold": 0.1,
    "retained_displacement_threshold": 0.5,
    "impact_band_radius": 0.85,
    "outside_radius": 1.15,
    "evaluation_frame": 18,
    "retention_evaluation_frame": 120,
    "joint_evidence_frames": [18, 120],
    "maximum_broken_joints": 0,
    "maximum_unbroken_joint_angular_residual_radians": 0.02,
    "maximum_unbroken_joint_linear_residual": 0.025,
    "minimum_unbroken_joints": BREAKABLE_JOINTS,
    "minimum_maximum_wall_normal_displacement": 0.10,
    "minimum_rms_wall_normal_displacement": 0.05,
    "bent_brick_displacement_threshold": 0.05,
    "minimum_bent_bricks": 100,
    "minimum_total_retained_fraction": 0.99,
}

BENCHMARK_RUNS = {
    "avbd": "BM_AvbdPaperBreakableWallStep/iterations:120",
    "vbd": "BM_VbdPaperBreakableWallStep/iterations:120",
    "sequential-impulse": ("BM_SequentialImpulsePaperBreakableWallStep/iterations:120"),
}

RESOLVED_SOLVER_IDENTITY = make_resolved_solver_identity(
    resolved_rigid_contact_family="vbd",
    rigid_point_joint_solver="vbd",
    avbd_rigid_contact_config_emplaced=False,
    rigid_contact_selection="world_solver_family",
    recorded_from=(
        "engine World.resolved_configuration in both VBD captures plus "
        "benchmark runtime resolved-configuration counters"
    ),
)

REPO_ROOT = SCRIPT_DIR.parent
SOURCE_PATHS = tuple(
    Path(path) for path in PAPER_PACKET_SOURCE_PATHS[DEFAULT_OUTPUT.name]
)


class AvbdPaperVbdComparisonPacketError(RuntimeError):
    """Raised when inputs cannot support the matched comparison claim."""


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--bend-capture-manifest", type=Path, required=True)
    parser.add_argument("--bend-image-verdict-json", type=Path, required=True)
    parser.add_argument("--retention-capture-manifest", type=Path, required=True)
    parser.add_argument(
        "--retention-image-verdict-json",
        type=Path,
        required=True,
    )
    parser.add_argument("--visual-review-json", type=Path, required=True)
    parser.add_argument("--paper-pdf", type=Path, required=True)
    parser.add_argument("--paper-figure-image", type=Path, required=True)
    parser.add_argument(
        "--avbd-packet",
        type=Path,
        default=DEFAULT_AVBD_PACKET,
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
            raise AvbdPaperVbdComparisonPacketError(
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


def _validate_scene_spec_fingerprint(value: object, label: str) -> str:
    if not isinstance(value, str) or len(value) != 16:
        raise AvbdPaperVbdComparisonPacketError(
            f"{label} must be a 16-character lowercase hexadecimal string"
        )
    try:
        parsed = int(value, 16)
    except ValueError as exc:
        raise AvbdPaperVbdComparisonPacketError(f"{label} must be hexadecimal") from exc
    if f"{parsed:016x}" != value:
        raise AvbdPaperVbdComparisonPacketError(
            f"{label} must use canonical lowercase hexadecimal"
        )
    return value


def _validate_sha256_value(value: object, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise AvbdPaperVbdComparisonPacketError(
            f"{label} must be a 64-character lowercase hexadecimal string"
        )
    try:
        parsed = int(value, 16)
    except ValueError as exc:
        raise AvbdPaperVbdComparisonPacketError(f"{label} must be hexadecimal") from exc
    if f"{parsed:064x}" != value:
        raise AvbdPaperVbdComparisonPacketError(
            f"{label} must use canonical lowercase hexadecimal"
        )
    return value


def _validate_resolved_configuration(
    metrics: dict[str, Any],
) -> list[dict[str, str]]:
    notes = metrics.get("resolved_configuration")
    if not isinstance(notes, list):
        raise AvbdPaperVbdComparisonPacketError(
            "scene metrics missing engine resolved_configuration"
        )
    normalized = []
    for index, note in enumerate(notes):
        if not isinstance(note, dict):
            raise AvbdPaperVbdComparisonPacketError(
                f"resolved_configuration[{index}] must be an object"
            )
        normalized_note = {}
        for key in ("domain", "requested", "resolved", "reason"):
            value = note.get(key)
            if not isinstance(value, str) or not value:
                raise AvbdPaperVbdComparisonPacketError(
                    f"resolved_configuration[{index}].{key} must be non-empty"
                )
            normalized_note[key] = value
        normalized.append(normalized_note)

    expected = {
        "rigid-body": ("vbd", "vbd"),
        "rigid-contact": ("vbd", "vbd"),
        "rigid-pair-constraint": ("vbd", "vbd"),
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
            raise AvbdPaperVbdComparisonPacketError(
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
    shared._validate_finite_tree(outcome, f"frame {expected_frame} outcome")
    for key, expected in (
        ("frame", expected_frame),
        ("evaluated", True),
        ("checkpoint", checkpoint["checkpoint"]),
        ("status", "pass"),
        ("thresholds_pass", True),
        ("threshold_checks", checkpoint["threshold_checks"]),
        ("broken_joints", 0),
        ("unbroken_joints", BREAKABLE_JOINTS),
        ("last_step_iterations", RIGID_CONSTRAINT_ITERATIONS),
    ):
        shared._require_exact(
            outcome.get(key),
            expected,
            f"frame {expected_frame} outcome {key}",
        )
    shared._require_close(
        outcome.get("world_time"),
        expected_frame * TIME_STEP,
        f"frame {expected_frame} outcome world_time",
    )
    shared._validate_joint_evidence(
        outcome,
        expected_broken_count=0,
        expected_broken_ids_sha256=(
            "e3b0c44298fc1c149afbf4c8996fb924" "27ae41e4649b934ca495991b7852b855"
        ),
        expected_outside_unbroken_count=484,
        label=f"frame {expected_frame} outcome",
    )
    shared._require_exact(
        outcome.get("broken_joint_impact_region_counts"),
        [0, 0, 0],
        f"frame {expected_frame} broken_joint_impact_region_counts",
    )
    shared._require_exact(
        outcome.get("broken_joints_outside_impact_regions"),
        0,
        f"frame {expected_frame} broken_joints_outside_impact_regions",
    )
    maximum_linear_residual = shared._finite_number(
        outcome.get("maximum_unbroken_joint_linear_residual"),
        f"frame {expected_frame} maximum_unbroken_joint_linear_residual",
    )
    if (
        maximum_linear_residual
        > OUTCOME_ORACLE["maximum_unbroken_joint_linear_residual"]
    ):
        raise AvbdPaperVbdComparisonPacketError(
            f"frame {expected_frame} retained-joint linear residual exceeds "
            "the VBD oracle"
        )
    maximum_angular_residual = shared._finite_number(
        outcome.get("maximum_unbroken_joint_angular_residual_radians"),
        f"frame {expected_frame} " "maximum_unbroken_joint_angular_residual_radians",
    )
    if (
        maximum_angular_residual
        > OUTCOME_ORACLE["maximum_unbroken_joint_angular_residual_radians"]
    ):
        raise AvbdPaperVbdComparisonPacketError(
            f"frame {expected_frame} retained-joint angular residual exceeds "
            "the VBD oracle"
        )

    if expected_frame == 18:
        minimums = {
            "maximum_wall_normal_displacement": (
                OUTCOME_ORACLE["minimum_maximum_wall_normal_displacement"]
            ),
            "rms_wall_normal_displacement": (
                OUTCOME_ORACLE["minimum_rms_wall_normal_displacement"]
            ),
            "bent_brick_count": OUTCOME_ORACLE["minimum_bent_bricks"],
        }
        for key, minimum in minimums.items():
            value = shared._finite_number(
                outcome.get(key),
                f"frame {expected_frame} outcome {key}",
            )
            if value < float(minimum):
                raise AvbdPaperVbdComparisonPacketError(
                    f"frame {expected_frame} outcome {key} must be >= "
                    f"{minimum}, got {value}"
                )
        contact_count = shared._finite_number(
            outcome.get("contact_count"),
            "frame 18 outcome contact_count",
        )
        if contact_count < 0.0:
            raise AvbdPaperVbdComparisonPacketError(
                "frame 18 outcome contact_count must be non-negative"
            )
    else:
        retained = shared._finite_number(
            outcome.get("total_retained_fraction"),
            "frame 120 outcome total_retained_fraction",
        )
        if retained < OUTCOME_ORACLE["minimum_total_retained_fraction"]:
            raise AvbdPaperVbdComparisonPacketError(
                "frame 120 wall-retention oracle failed"
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
        raise AvbdPaperVbdComparisonPacketError(
            "capture manifest missing scene_metrics"
        )
    shared._require_exact(
        summary.get("event_count"),
        expected_frame,
        "scene metric event_count",
    )
    latest = summary.get("latest")
    if not isinstance(latest, dict):
        raise AvbdPaperVbdComparisonPacketError(
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
        raise AvbdPaperVbdComparisonPacketError(
            "capture latest scene metric payload must be an object"
        )
    shared._validate_finite_tree(metrics, "scene metrics")

    exact_metrics = {
        "ball_count": IMPACTING_BALLS,
        "breakable_joints": BREAKABLE_JOINTS,
        "brick_count": BRICK_COUNT,
        "collision_shapes": COLLISION_SHAPES,
        "executor": "World.step default",
        "paper_locator": PAPER_LOCATOR,
        "rigid_bodies": RIGID_BODIES,
        "rigid_body_solver": "VBD",
        "row": SCENE_ID,
        "solver": "public_vbd",
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
        raise AvbdPaperVbdComparisonPacketError("scene metrics missing outcome_oracle")
    for key, expected in OUTCOME_ORACLE.items():
        if isinstance(expected, float):
            shared._require_close(oracle.get(key), expected, f"oracle {key}")
        else:
            shared._require_exact(oracle.get(key), expected, f"oracle {key}")

    outcome = metrics.get("outcome")
    if not isinstance(outcome, dict):
        raise AvbdPaperVbdComparisonPacketError("scene metrics missing outcome")
    _validate_outcome(outcome, expected_frame=expected_frame)
    resolved = _validate_resolved_configuration(metrics)
    view_report = shared._validate_view_report(
        metrics,
        expected_focus=VIEW_FOCUS,
        width=width,
        height=height,
    )
    fingerprint = _validate_scene_spec_fingerprint(
        metrics.get("scene_spec_fingerprint"),
        "scene metrics scene_spec_fingerprint",
    )
    return {
        "event_count": expected_frame,
        "frame": expected_frame,
        "outcome": outcome,
        "outcome_oracle": oracle,
        "resolved_configuration": resolved,
        "scene_spec_fingerprint": fingerprint,
        "view_report": view_report,
        "scene_contract": {
            key: metrics[key]
            for key in (
                "ball_count",
                "break_force",
                "breakable_joints",
                "brick_count",
                "collision_shapes",
                "executor",
                "paper_locator",
                "rigid_bodies",
                "rigid_body_solver",
                "solver",
                "time_step_ms",
            )
        }
        | {"rigid_constraint_options": metrics["rigid_constraint_options"]},
    }


def _validate_capture(
    manifest_path: Path,
    *,
    expected_frame: int,
) -> tuple[dict[str, Any], Path]:
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
            "solver": "public_vbd",
            "source": "scene_capture_metrics.latest.metrics",
        },
        "capture resolved_solver_identity",
    )
    capture = manifest.get("capture")
    if not isinstance(capture, dict):
        raise AvbdPaperVbdComparisonPacketError("capture manifest missing dimensions")
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
        raise AvbdPaperVbdComparisonPacketError(
            "capture width and height must be positive integers"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdPaperVbdComparisonPacketError("capture manifest missing artifacts")
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
    if not screenshot.is_file() or not metrics_events.is_file():
        raise AvbdPaperVbdComparisonPacketError(
            "capture screenshot and scene metric log must both exist"
        )
    if shared._png_dimensions(screenshot) != (width, height):
        raise AvbdPaperVbdComparisonPacketError(
            "capture screenshot dimensions do not match manifest"
        )
    events = shared._read_scene_metric_events(
        metrics_events,
        expected_frame=expected_frame,
        expected_scene_id=SCENE_ID,
    )
    scene_metrics = _validate_scene_metrics(
        manifest,
        expected_frame=expected_frame,
        height=height,
        logged_latest=events[-1],
        width=width,
    )
    capture_source_provenance = shared._validate_capture_provenance(
        manifest,
        metrics_events=metrics_events,
        screenshot=screenshot,
    )
    return (
        {
            "camera": shared._validate_camera(manifest),
            "capture": {
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
            "scene_metrics_events": {
                "event_count": len(events),
                "file": metrics_events.name,
                "sha256": shared._sha256(metrics_events),
            },
            "screenshot": {
                "file": screenshot.name,
                "sha256": shared._sha256(screenshot),
            },
        },
        screenshot,
    )


def _benchmark_rows(
    data: dict[str, Any],
    *,
    solver_key: str,
    expected_fingerprint: str,
) -> dict[str, Any]:
    benchmark_run = BENCHMARK_RUNS[solver_key]
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdPaperVbdComparisonPacketError(
            "benchmark JSON missing benchmarks list"
        )
    matching = [
        row
        for row in rows
        if isinstance(row, dict)
        and shared._canonical_benchmark_name(row) == benchmark_run
    ]
    by_aggregate: dict[str, dict[str, Any]] = {}
    for row in matching:
        aggregate = row.get("aggregate_name")
        if not isinstance(aggregate, str):
            raise AvbdPaperVbdComparisonPacketError(
                f"{benchmark_run}: every row must be an aggregate"
            )
        if aggregate in by_aggregate:
            raise AvbdPaperVbdComparisonPacketError(
                f"{benchmark_run}: duplicate {aggregate} aggregate"
            )
        by_aggregate[aggregate] = row
    expected_aggregates = {"mean", "median", "stddev", "cv"}
    if set(by_aggregate) != expected_aggregates:
        raise AvbdPaperVbdComparisonPacketError(
            f"{benchmark_run}: expected aggregates "
            f"{sorted(expected_aggregates)}, got {sorted(by_aggregate)}"
        )

    counters = {
        "breakable_joints": BREAKABLE_JOINTS,
        "collision_shapes": COLLISION_SHAPES,
        "impacting_balls": IMPACTING_BALLS,
        f"public_{solver_key}_family": 1,
        f"resolved_rigid_body_{solver_key}": 1,
        "resolved_rigid_constraint_iterations": 1,
        f"resolved_rigid_contact_{solver_key}": 1,
        f"resolved_rigid_pair_constraint_{solver_key}": 1,
        "rigid_constraint_iterations": RIGID_CONSTRAINT_ITERATIONS,
        "rigid_bodies": RIGID_BODIES,
        "rigid_body_joints": BREAKABLE_JOINTS,
        "runtime_contract_passed": 1,
        "trajectory_frames": 120,
    }
    fingerprint = int(expected_fingerprint, 16)
    fingerprint_counters = {
        "scene_spec_fingerprint_hi": fingerprint >> 32,
        "scene_spec_fingerprint_lo": fingerprint & 0xFFFFFFFF,
    }
    packet_rows = []
    for aggregate in ("mean", "median", "stddev", "cv"):
        row = by_aggregate[aggregate]
        shared._require_exact(
            row.get("run_type"),
            "aggregate",
            f"{benchmark_run} run_type",
        )
        shared._require_exact(
            row.get("repetitions"),
            5,
            f"{benchmark_run} repetitions",
        )
        shared._require_exact(
            row.get("iterations"),
            5,
            f"{benchmark_run} iterations",
        )
        shared._require_exact(
            row.get("time_unit"),
            "ns",
            f"{benchmark_run} time_unit",
        )
        real_time = shared._finite_number(
            row.get("real_time"),
            f"{benchmark_run} {aggregate} real_time",
        )
        cpu_time = shared._finite_number(
            row.get("cpu_time"),
            f"{benchmark_run} {aggregate} cpu_time",
        )
        if real_time < 0.0 or cpu_time < 0.0:
            raise AvbdPaperVbdComparisonPacketError(
                f"{benchmark_run}: timings must be non-negative"
            )
        if aggregate in ("mean", "median"):
            if real_time <= 0.0 or cpu_time <= 0.0:
                raise AvbdPaperVbdComparisonPacketError(
                    f"{benchmark_run}: representative timings must be positive"
                )
            for key, expected in counters.items():
                shared._require_close(
                    row.get(key),
                    float(expected),
                    f"{benchmark_run} {aggregate} {key}",
                )
            for key, expected in fingerprint_counters.items():
                shared._require_close(
                    row.get(key),
                    float(expected),
                    f"{benchmark_run} {aggregate} {key}",
                )
        packet_rows.append(dict(row))

    cv_row = by_aggregate["cv"]
    real_cv = shared._finite_number(
        cv_row.get("real_time"),
        f"{benchmark_run} real-time CV",
    )
    cpu_cv = shared._finite_number(
        cv_row.get("cpu_time"),
        f"{benchmark_run} CPU-time CV",
    )
    if not 0.0 <= real_cv <= 0.10 or not 0.0 <= cpu_cv <= 0.10:
        raise AvbdPaperVbdComparisonPacketError(
            f"{benchmark_run}: timing CV must be between 0 and 10 percent"
        )
    return {
        "benchmark": benchmark_run.removesuffix("/iterations:120"),
        "rows": packet_rows,
        "stability": {
            "cpu_time_cv_fraction": cpu_cv,
            "real_time_cv_fraction": real_cv,
            "repetitions": 5,
        },
        "timing": {
            "mean_cpu_time_per_step_ns": by_aggregate["mean"]["cpu_time"],
            "mean_real_time_per_step_ns": by_aggregate["mean"]["real_time"],
            "median_cpu_time_per_step_ns": by_aggregate["median"]["cpu_time"],
            "median_real_time_per_step_ns": by_aggregate["median"]["real_time"],
        },
    }


def _validate_benchmark(
    benchmark_path: Path,
    *,
    expected_fingerprint: str,
) -> dict[str, Any]:
    data = shared._load_json(benchmark_path)
    methods = {
        solver_key: _benchmark_rows(
            data,
            solver_key=solver_key,
            expected_fingerprint=expected_fingerprint,
        )
        for solver_key in ("avbd", "vbd")
    }
    context = data.get("context")
    if not isinstance(context, dict):
        raise AvbdPaperVbdComparisonPacketError("benchmark JSON missing context object")
    executable = context.get("executable")
    if (
        not isinstance(executable, str)
        or Path(executable).name != "bm_avbd_rigid_fixed_joint"
    ):
        raise AvbdPaperVbdComparisonPacketError(
            "benchmark context identifies the wrong executable"
        )
    shared._require_exact(
        context.get("library_build_type"),
        "release",
        "benchmark library_build_type",
    )
    benchmark_source_provenance = shared._validate_benchmark_source_provenance(context)
    avbd_median = float(methods["avbd"]["timing"]["median_cpu_time_per_step_ns"])
    vbd_median = float(methods["vbd"]["timing"]["median_cpu_time_per_step_ns"])
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
                "benchmark_source_sha256",
                "capture_source_provenance_digest",
            )
            if key in context
        },
        "json_sha256": shared._sha256(benchmark_path),
        "methods": methods,
        "scene_spec_fingerprint": expected_fingerprint,
        "source_provenance": benchmark_source_provenance,
        "comparison": {
            "basis": (
                "same executable, host, reconstructed scene fingerprint, "
                "trajectory length, time step, and 20-sweep budget"
            ),
            "vbd_to_avbd_median_cpu_cost_ratio": vbd_median / avbd_median,
        },
    }


def _validate_avbd_packet(
    packet_path: Path,
    *,
    benchmark_sha256: str,
    expected_fingerprint: str,
) -> dict[str, Any]:
    packet = shared._load_json(packet_path)
    shared._require_exact(
        packet.get("schema_version"),
        AVBD_PACKET_SCHEMA_VERSION,
        "linked AVBD packet schema_version",
    )
    shared._require_exact(
        packet.get("packet"),
        "avbd_paper_breakable_wall",
        "linked AVBD packet identity",
    )
    shared._require_exact(
        packet.get("scene"),
        AVBD_SCENE_ID,
        "linked AVBD packet scene",
    )
    identity = packet.get("resolved_solver_identity")
    if not isinstance(identity, dict):
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD packet lacks resolved_solver_identity"
        )
    shared._require_exact(
        identity.get("rigid_contact_solver"),
        "avbd",
        "linked AVBD rigid contact solver",
    )
    shared._require_exact(
        identity.get("rigid_point_joint_solver"),
        "avbd",
        "linked AVBD point-joint solver",
    )
    benchmark = packet.get("benchmark")
    if not isinstance(benchmark, dict):
        raise AvbdPaperVbdComparisonPacketError("linked AVBD packet lacks benchmark")
    shared._require_exact(
        benchmark.get("json_sha256"),
        benchmark_sha256,
        "linked AVBD benchmark JSON hash",
    )
    shared._require_exact(
        benchmark.get("scene_spec_fingerprint"),
        expected_fingerprint,
        "linked AVBD scene fingerprint",
    )

    correctness = packet.get("correctness")
    if not isinstance(correctness, dict):
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD packet lacks correctness evidence"
        )
    shared._require_exact(
        correctness.get("determinism_test"),
        "test_avbd_paper_breakable_wall_outcome_is_deterministic",
        "linked AVBD determinism test",
    )
    shared._require_exact(
        correctness.get("outcome_test"),
        "test_avbd_paper_breakable_wall_matches_figure13_contract",
        "linked AVBD outcome test",
    )

    visual = packet.get("visual_evidence")
    if not isinstance(visual, dict):
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD packet lacks visual evidence"
        )

    validated_screenshots: dict[str, dict[str, str]] = {}
    for label, expected_frame in (("impact", 60), ("outcome", 120)):
        capture = visual.get(label)
        if not isinstance(capture, dict):
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD packet lacks {label} visual evidence"
            )
        screenshot = capture.get("screenshot")
        if not isinstance(screenshot, dict):
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD {label} evidence lacks a screenshot"
            )
        screenshot_file = screenshot.get("file")
        if not isinstance(screenshot_file, str) or not screenshot_file:
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD {label} screenshot file must be non-empty"
            )
        screenshot_hash = _validate_sha256_value(
            screenshot.get("sha256"),
            f"linked AVBD {label} screenshot sha256",
        )
        validated_screenshots[label] = {
            "file": screenshot_file,
            "sha256": screenshot_hash,
        }

        image_verdict = capture.get("image_verdict")
        if not isinstance(image_verdict, dict) or image_verdict.get("pass") is not True:
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD {label} image verdict must pass"
            )
        _validate_sha256_value(
            image_verdict.get("sha256"),
            f"linked AVBD {label} image verdict sha256",
        )

        scene_metrics = capture.get("scene_metrics")
        if not isinstance(scene_metrics, dict):
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD {label} evidence lacks scene metrics"
            )
        shared._require_exact(
            scene_metrics.get("frame"),
            expected_frame,
            f"linked AVBD {label} frame",
        )
        shared._require_exact(
            scene_metrics.get("scene_spec_fingerprint"),
            expected_fingerprint,
            f"linked AVBD {label} scene fingerprint",
        )
        oracle = scene_metrics.get("outcome_oracle")
        if not isinstance(oracle, dict):
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD {label} evidence lacks outcome oracle"
            )
        for key, expected in shared.OUTCOME_ORACLE.items():
            if isinstance(expected, float):
                shared._require_close(
                    oracle.get(key),
                    expected,
                    f"linked AVBD {label} outcome oracle {key}",
                )
            else:
                shared._require_exact(
                    oracle.get(key),
                    expected,
                    f"linked AVBD {label} outcome oracle {key}",
                )
        outcome = scene_metrics.get("outcome")
        if not isinstance(outcome, dict):
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD {label} evidence lacks outcome"
            )
        expected_outcome = shared.EXPECTED_OUTCOMES[expected_frame]
        for key in (
            "broken_joints",
            "evaluated",
            "impact_band_displaced_counts",
            "status",
            "threshold_checks",
            "thresholds_pass",
            "unbroken_joints",
        ):
            shared._require_exact(
                outcome.get(key),
                expected_outcome[key],
                f"linked AVBD frame {expected_frame} outcome {key}",
            )
        for key in ("outside_retained_fraction", "total_retained_fraction"):
            shared._require_close(
                outcome.get(key),
                expected_outcome[key],
                f"linked AVBD frame {expected_frame} outcome {key}",
            )
        shared._require_exact(
            outcome.get("frame"),
            expected_frame,
            f"linked AVBD frame {expected_frame} outcome frame",
        )
        shared._require_exact(
            outcome.get("last_step_iterations"),
            shared.RIGID_CONSTRAINT_ITERATIONS,
            f"linked AVBD frame {expected_frame} outcome iterations",
        )
        shared._validate_joint_evidence(
            outcome,
            expected_broken_count=expected_outcome["broken_joints"],
            expected_broken_ids_sha256=shared.OUTCOME_ORACLE[
                "expected_broken_joint_ids_sha256"
            ],
            expected_outside_unbroken_count=405,
            label=f"linked AVBD frame {expected_frame} outcome",
        )
        shared._require_exact(
            outcome.get("broken_joint_impact_region_counts"),
            [18, 44, 13],
            f"linked AVBD frame {expected_frame} impact-region counts",
        )
        shared._require_exact(
            outcome.get("broken_joints_outside_impact_regions"),
            79,
            f"linked AVBD frame {expected_frame} outside-region count",
        )

    semantic = visual.get("semantic_review")
    if not isinstance(semantic, dict) or semantic.get("verdict") != "pass":
        raise AvbdPaperVbdComparisonPacketError("linked AVBD semantic review must pass")
    semantic_file = semantic.get("file")
    if not isinstance(semantic_file, str) or not semantic_file:
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD semantic review file must be non-empty"
        )
    semantic_hash = _validate_sha256_value(
        semantic.get("sha256"),
        "linked AVBD semantic review sha256",
    )
    inspected_images = semantic.get("inspected_images")
    if not isinstance(inspected_images, list):
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD semantic review must list inspected images"
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
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD packet lacks paper figure evidence"
        )
    expected_inspected = {
        "impact_frame_60": validated_screenshots["impact"],
        "outcome_frame_120": validated_screenshots["outcome"],
        "paper_figure_13_reference": {
            "file": paper_figure.get("file"),
            "sha256": _validate_sha256_value(
                paper_figure.get("sha256"),
                "linked AVBD paper figure sha256",
            ),
        },
    }
    if set(by_role) != set(expected_inspected):
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD semantic review must inspect impact, outcome, and "
            "paper Figure 13 images"
        )
    for role, expected in expected_inspected.items():
        shared._require_exact(
            by_role[role].get("file"),
            expected["file"],
            f"linked AVBD semantic review {role} file",
        )
        shared._require_exact(
            by_role[role].get("sha256"),
            expected["sha256"],
            f"linked AVBD semantic review {role} sha256",
        )

    provenance = packet.get("source_provenance")
    if not isinstance(provenance, dict):
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD packet lacks source provenance"
        )
    files = provenance.get("files")
    if not isinstance(files, list) or not files:
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD source provenance must list files"
        )
    expected_source_paths = {relative.as_posix() for relative in shared.SOURCE_PATHS}
    actual_source_paths = {
        entry.get("path") for entry in files if isinstance(entry, dict)
    }
    if actual_source_paths != expected_source_paths:
        raise AvbdPaperVbdComparisonPacketError(
            "linked AVBD source provenance must list the canonical writer " "source set"
        )
    combined = sha256()
    for entry in files:
        if not isinstance(entry, dict):
            raise AvbdPaperVbdComparisonPacketError(
                "linked AVBD source provenance entry must be an object"
            )
        relative = entry.get("path")
        expected_hash = entry.get("sha256")
        if not isinstance(relative, str) or not isinstance(expected_hash, str):
            raise AvbdPaperVbdComparisonPacketError(
                "linked AVBD source provenance entry is incomplete"
            )
        current_hash = shared._sha256(REPO_ROOT / relative)
        if current_hash != expected_hash:
            raise AvbdPaperVbdComparisonPacketError(
                f"linked AVBD source provenance drifted for {relative}"
            )
        payload = (REPO_ROOT / relative).read_bytes()
        encoded_path = relative.encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)
    shared._require_exact(
        provenance.get("algorithm"),
        "sha256-length-prefixed-path-and-content-v1",
        "linked AVBD source provenance algorithm",
    )
    shared._require_exact(
        provenance.get("digest"),
        combined.hexdigest(),
        "linked AVBD source provenance digest",
    )

    return {
        "file": packet_path.name,
        "sha256": shared._sha256(packet_path),
        "resolved_solver_identity": identity,
        "scene_spec_fingerprint": expected_fingerprint,
        "visual_evidence": {
            "impact_screenshot": validated_screenshots["impact"],
            "outcome_screenshot": validated_screenshots["outcome"],
            "semantic_review": {
                "file": semantic_file,
                "sha256": semantic_hash,
                "verdict": "pass",
            },
        },
    }


def _validate_visual_review(
    review_path: Path,
    *,
    bend_screenshot: Path,
    retention_screenshot: Path,
    paper_figure: Path,
) -> dict[str, Any]:
    review = shared._load_json(review_path)
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
    fields = (
        "reviewer_capability",
        "claim_and_expected_observation",
        "text_oracle",
        "visible_observation",
        "reconciliation_and_verdict",
        "not_proven_and_limitations",
    )
    for key in fields:
        value = review.get(key)
        if not isinstance(value, str) or not value.strip():
            raise AvbdPaperVbdComparisonPacketError(
                f"visual review {key} must be non-empty"
            )
    if "image" not in review["reviewer_capability"].lower():
        raise AvbdPaperVbdComparisonPacketError(
            "visual review must name an image-capable reviewer"
        )
    if (
        "viewreport"
        not in review["reconciliation_and_verdict"].replace(" ", "").lower()
    ):
        raise AvbdPaperVbdComparisonPacketError(
            "visual review reconciliation must account for ViewReports"
        )
    limitations = review["not_proven_and_limitations"].lower()
    for required in ("cuda", "sequential impulse", "unpublished", "xpbd"):
        if required not in limitations:
            raise AvbdPaperVbdComparisonPacketError(
                f"visual review limitations must name {required}"
            )

    expected = {
        "bend_frame_18": bend_screenshot,
        "retention_frame_120": retention_screenshot,
        "paper_figure_13_reference": paper_figure,
    }
    entries = review.get("inspected_images")
    if not isinstance(entries, list):
        raise AvbdPaperVbdComparisonPacketError(
            "visual review inspected_images must be a list"
        )
    by_role = {
        entry.get("role"): entry
        for entry in entries
        if isinstance(entry, dict) and isinstance(entry.get("role"), str)
    }
    if set(by_role) != set(expected):
        raise AvbdPaperVbdComparisonPacketError(
            "visual review must inspect bend, retention, and paper images"
        )
    inspected = []
    for role, path in expected.items():
        entry = by_role[role]
        file_value = entry.get("file")
        if (
            not isinstance(file_value, str)
            or Path(file_value).resolve() != path.resolve()
        ):
            raise AvbdPaperVbdComparisonPacketError(
                f"visual review {role} file does not match inspected image"
            )
        expected_hash = shared._sha256(path)
        shared._require_exact(
            entry.get("sha256"),
            expected_hash,
            f"visual review {role} sha256",
        )
        inspected.append({"file": path.name, "role": role, "sha256": expected_hash})
    return {
        "claim_and_expected_observation": review["claim_and_expected_observation"],
        "file": review_path.name,
        "inspected_images": inspected,
        "not_proven_and_limitations": review["not_proven_and_limitations"],
        "reconciliation_and_verdict": review["reconciliation_and_verdict"],
        "reviewer_capability": review["reviewer_capability"],
        "sha256": shared._sha256(review_path),
        "text_oracle": review["text_oracle"],
        "verdict": "pass",
        "visible_observation": review["visible_observation"],
    }


def make_packet(
    *,
    benchmark_json: Path,
    bend_capture_manifest: Path,
    bend_image_verdict_json: Path,
    retention_capture_manifest: Path,
    retention_image_verdict_json: Path,
    visual_review_json: Path,
    paper_pdf: Path,
    paper_figure_image: Path,
    avbd_packet: Path,
) -> dict[str, Any]:
    bend, bend_screenshot = _validate_capture(
        bend_capture_manifest,
        expected_frame=18,
    )
    bend["image_verdict"] = shared._validate_image_verdict(
        bend_image_verdict_json,
        bend_screenshot,
        expected_frame=18,
        expected_scene_id=SCENE_ID,
    )
    retention, retention_screenshot = _validate_capture(
        retention_capture_manifest,
        expected_frame=120,
    )
    retention["image_verdict"] = shared._validate_image_verdict(
        retention_image_verdict_json,
        retention_screenshot,
        expected_frame=120,
        expected_scene_id=SCENE_ID,
    )
    fingerprint = bend["scene_metrics"]["scene_spec_fingerprint"]
    shared._require_exact(
        retention["scene_metrics"]["scene_spec_fingerprint"],
        fingerprint,
        "bend/retention scene fingerprint",
    )
    benchmark = _validate_benchmark(
        benchmark_json,
        expected_fingerprint=fingerprint,
    )
    linked_avbd = _validate_avbd_packet(
        avbd_packet,
        benchmark_sha256=benchmark["json_sha256"],
        expected_fingerprint=fingerprint,
    )
    paper_reference = shared._validate_paper_artifacts(
        paper_pdf,
        paper_figure_image,
    )
    semantic_review = _validate_visual_review(
        visual_review_json,
        bend_screenshot=bend_screenshot,
        retention_screenshot=retention_screenshot,
        paper_figure=paper_figure_image,
    )

    return {
        "benchmark": benchmark,
        "claim_boundary": (
            "This packet covers the matched fixed-penalty VBD row and its "
            "same-host CPU comparison with the linked public AVBD Figure 13 "
            "packet. It does not reproduce the Sequential Impulse or XPBD "
            "rows, claim exact unpublished source constants, establish CUDA "
            "parity, or compare against a published/source timing."
        ),
        "correctness": {
            "allocation_scene": (
                "configurePublicRigidVbdContactAndBreakableJointRowsScene"
            ),
            "allocation_tests": [
                "World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths",
                "World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap",
                "World.BakedAvbdVbdRowsDoNotMallocOnHeap",
            ],
            "determinism_test": (
                "test_vbd_paper_breakable_wall_checkpoints_are_deterministic"
            ),
            "outcome_test": ("test_vbd_paper_breakable_wall_matches_figure13_contract"),
            "public_configuration": {
                "rigid_body_solver": "VBD",
                "rigid_constraint_options": {
                    "iterations": RIGID_CONSTRAINT_ITERATIONS,
                },
                "scene_spec_fingerprint": fingerprint,
                "time_step": TIME_STEP,
            },
        },
        "linked_avbd_evidence": linked_avbd,
        "packet": "avbd_paper_vbd_comparison",
        "paper_reference": paper_reference,
        "performance_claim_boundary": (
            "The ratio is a descriptive same-host cost comparison for two "
            "public DART families running an identical reconstructed scene and "
            "iteration budget. It is not a paper/reference speedup claim."
        ),
        "publication_observation": (
            "The Figure 13 VBD row bends under the three impacts but does not " "break."
        ),
        "reconstruction": {
            "matched_with_avbd": {
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
                "^BM_(Avbd|Vbd)PaperBreakableWallStep/iterations:120$"
            ),
            "capture_commands": [
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 18 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label bend "
                    "--output-dir <bend-capture-dir>"
                ),
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 120 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label retention "
                    "--output-dir <retention-capture-dir>"
                ),
            ],
            "image_verdict_commands": [
                (
                    "pixi run image-verdict -- "
                    f"<bend-capture-dir>/{SCENE_ID}_bend.png "
                    f"--meta scene={SCENE_ID} --meta frame=18 "
                    f"--meta view={CAMERA_VIEW} "
                    "--out <bend-capture-dir>/image_verdict.json"
                ),
                (
                    "pixi run image-verdict -- "
                    f"<retention-capture-dir>/{SCENE_ID}_retention.png "
                    f"--meta scene={SCENE_ID} --meta frame=120 "
                    f"--meta view={CAMERA_VIEW} "
                    "--out <retention-capture-dir>/image_verdict.json"
                ),
            ],
        },
        "resolved_solver_identity": RESOLVED_SOLVER_IDENTITY,
        "schema_version": AVBD_PACKET_SCHEMA_VERSION,
        "scene": SCENE_ID,
        "source_provenance": _source_provenance(),
        "target": {
            "complete_paper_reproduction": False,
            "contract_rows": [
                "avbd.paper.fig.13",
            ],
            "covered_slice": (
                "matched CPU VBD bend/no-break row with numeric, allocation, "
                "performance, and assessed visual evidence"
            ),
        },
        "visual_evidence": {
            "bend": bend,
            "retention": retention,
            "semantic_review": semantic_review,
        },
    }


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(
            benchmark_json=args.benchmark_json,
            bend_capture_manifest=args.bend_capture_manifest,
            bend_image_verdict_json=args.bend_image_verdict_json,
            retention_capture_manifest=args.retention_capture_manifest,
            retention_image_verdict_json=(args.retention_image_verdict_json),
            visual_review_json=args.visual_review_json,
            paper_pdf=args.paper_pdf,
            paper_figure_image=args.paper_figure_image,
            avbd_packet=args.avbd_packet,
        )
    except (
        OSError,
        ValueError,
        shared.AvbdPaperBreakableWallPacketError,
        AvbdPaperVbdComparisonPacketError,
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
