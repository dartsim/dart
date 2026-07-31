#!/usr/bin/env python3
"""Write the validated AVBD paper Figure 13 breakable-wall packet."""

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

from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    make_resolved_solver_identity,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-breakable-wall-packet.json"
)
SCENE_ID = "avbd_paper_breakable_wall"
BENCHMARK_NAME = "BM_AvbdPaperBreakableWallStep"
BENCHMARK_RUN = f"{BENCHMARK_NAME}/iterations:120"
PAPER_LOCATOR = "Section 5.4 and Figure 13, PDF page 10"
PAPER_PDF_SHA256 = "7957d116b9130cfb0aa5a48ab7cd0d74a64ad79f75c99acd291bdece2be3f2d6"
PAPER_FIGURE_SHA256 = "040361603d4e986de4d2da570593b9dc8295f8def2f57fd200de9a3e3836c61b"

TIME_STEP = 1.0 / 60.0
RIGID_CONSTRAINT_ITERATIONS = 20
BRICK_COUNT = 252
RIGID_BODIES = 256
COLLISION_SHAPES = 256
BREAKABLE_JOINTS = 712
IMPACTING_BALLS = 3
BREAK_FORCE = 8500.0
CAMERA_DISTANCE = 11.0
CAMERA_TARGET = (0.0, 0.0, 1.4)
CAMERA_VIEW = "front"
VIEW_FOCUS = (
    "avbd_paper_wall_brick_00_00_visual",
    "avbd_paper_wall_brick_00_20_visual",
    "avbd_paper_wall_brick_11_00_visual",
    "avbd_paper_wall_brick_11_20_visual",
)

OUTCOME_ORACLE = {
    "displacement_threshold": 0.5,
    "evaluation_frame": 120,
    "impact_band_radius": 0.85,
    "maximum_broken_joints": 500,
    "minimum_broken_joints": 300,
    "minimum_displaced_bricks_per_impact_band": 4,
    "minimum_outside_retained_fraction": 0.8,
    "minimum_total_retained_fraction": 0.7,
    "minimum_unbroken_joints": 200,
    "outside_radius": 1.15,
}

EXPECTED_OUTCOMES = {
    60: {
        "broken_joints": 359,
        "evaluated": False,
        "impact_band_displaced_counts": [3, 9, 4],
        "outside_retained_fraction": 0.9337016574585635,
        "status": "pre-evaluation",
        "threshold_checks": {
            "damage_in_three_impact_bands": False,
            "finite_state": True,
            "fracture_activated": True,
            "fracture_localized": True,
            "outside_wall_retained": True,
            "total_wall_retained": True,
        },
        "thresholds_pass": False,
        "total_retained_fraction": 0.8611111111111112,
        "unbroken_joints": 353,
    },
    120: {
        "broken_joints": 359,
        "evaluated": True,
        "impact_band_displaced_counts": [4, 10, 6],
        "outside_retained_fraction": 0.9116022099447514,
        "status": "pass",
        "threshold_checks": {
            "damage_in_three_impact_bands": True,
            "finite_state": True,
            "fracture_activated": True,
            "fracture_localized": True,
            "outside_wall_retained": True,
            "total_wall_retained": True,
        },
        "thresholds_pass": True,
        "total_retained_fraction": 0.8253968253968254,
        "unbroken_joints": 353,
    },
}

RESOLVED_SOLVER_IDENTITY = make_resolved_solver_identity(
    resolved_rigid_contact_family="avbd",
    rigid_point_joint_solver="avbd",
    avbd_rigid_contact_config_emplaced=False,
    rigid_contact_selection="world_solver_family",
    recorded_from=(
        "engine World.resolved_configuration in both captures plus benchmark "
        "runtime resolved-configuration counters"
    ),
)

REPO_ROOT = SCRIPT_DIR.parent
SOURCE_PATHS = (
    Path("dart/gui/view_quality.cpp"),
    Path("dart/gui/view_quality.hpp"),
    Path("dart/simulation/world.cpp"),
    Path("dart/simulation/world.hpp"),
    Path("dart/simulation/world_options.hpp"),
    Path("dart/simulation/compute/rigid_body_contact_stage.cpp"),
    Path("dart/simulation/detail/rigid_avbd/rigid_block_kernel.hpp"),
    Path("dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp"),
    Path("python/dartpy/_view_quality.py"),
    Path("python/examples/demos/scenes/avbd_paper_breakable_wall.py"),
    Path("tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp"),
    Path("scripts/write_avbd_paper_breakable_wall_packet.py"),
)


class AvbdPaperBreakableWallPacketError(RuntimeError):
    """Raised when an input cannot support the packet's claims."""


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--impact-capture-manifest", type=Path, required=True)
    parser.add_argument("--impact-image-verdict-json", type=Path, required=True)
    parser.add_argument("--outcome-capture-manifest", type=Path, required=True)
    parser.add_argument("--outcome-image-verdict-json", type=Path, required=True)
    parser.add_argument("--visual-review-json", type=Path, required=True)
    parser.add_argument("--paper-pdf", type=Path, required=True)
    parser.add_argument("--paper-figure-image", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise AvbdPaperBreakableWallPacketError(f"{path}: file not found") from exc
    except json.JSONDecodeError as exc:
        raise AvbdPaperBreakableWallPacketError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise AvbdPaperBreakableWallPacketError(
            f"{path}: top-level JSON value must be an object"
        )
    return data


def _sha256(path: Path) -> str:
    digest = sha256()
    try:
        with path.open("rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(chunk)
    except FileNotFoundError as exc:
        raise AvbdPaperBreakableWallPacketError(f"{path}: file not found") from exc
    return digest.hexdigest()


def _source_provenance() -> dict[str, Any]:
    combined = sha256()
    files = []
    for relative_path in SOURCE_PATHS:
        path = REPO_ROOT / relative_path
        try:
            payload = path.read_bytes()
        except FileNotFoundError as exc:
            raise AvbdPaperBreakableWallPacketError(
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


def _png_dimensions(path: Path) -> tuple[int, int]:
    try:
        with path.open("rb") as file:
            header = file.read(24)
    except FileNotFoundError as exc:
        raise AvbdPaperBreakableWallPacketError(f"{path}: image not found") from exc
    if (
        len(header) != 24
        or header[:8] != b"\x89PNG\r\n\x1a\n"
        or header[12:16] != b"IHDR"
    ):
        raise AvbdPaperBreakableWallPacketError(f"{path}: expected a PNG image")
    width, height = struct.unpack(">II", header[16:24])
    if width < 1 or height < 1:
        raise AvbdPaperBreakableWallPacketError(f"{path}: invalid PNG dimensions")
    return width, height


def _finite_number(value: object, label: str) -> float:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
    ):
        raise AvbdPaperBreakableWallPacketError(f"{label} must be a finite number")
    return float(value)


def _require_close(
    value: object,
    expected: float,
    label: str,
    *,
    absolute_tolerance: float = 1e-12,
) -> None:
    actual = _finite_number(value, label)
    if not math.isclose(
        actual,
        expected,
        rel_tol=0.0,
        abs_tol=absolute_tolerance,
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must be {expected:g}, got {actual:g}"
        )


def _require_exact(value: object, expected: object, label: str) -> None:
    if value != expected:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must be {expected!r}, got {value!r}"
        )


def _artifact_path(manifest_path: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AvbdPaperBreakableWallPacketError(f"capture manifest missing {label}")
    path = Path(value)
    if not path.is_absolute():
        path = manifest_path.parent / path
    return path


def _validate_finite_tree(value: object, label: str) -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        _finite_number(value, label)
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_finite_tree(item, f"{label}[{index}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            _validate_finite_tree(item, f"{label}.{key}")
        return
    raise AvbdPaperBreakableWallPacketError(
        f"{label} contains unsupported value {value!r}"
    )


def _validate_camera(manifest: dict[str, Any]) -> dict[str, Any]:
    camera = manifest.get("camera")
    if not isinstance(camera, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture manifest missing serialized camera"
        )
    _require_exact(camera.get("view"), CAMERA_VIEW, "camera view")
    _require_close(camera.get("distance"), CAMERA_DISTANCE, "camera distance")
    target = camera.get("target")
    if not isinstance(target, list) or len(target) != 3:
        raise AvbdPaperBreakableWallPacketError(
            "camera target must contain three coordinates"
        )
    for index, expected in enumerate(CAMERA_TARGET):
        _require_close(target[index], expected, f"camera target[{index}]")
    return {
        "distance": CAMERA_DISTANCE,
        "target": list(CAMERA_TARGET),
        "view": CAMERA_VIEW,
    }


def _validate_scene_spec_fingerprint(value: object, label: str) -> str:
    if not isinstance(value, str) or len(value) != 16:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must be a 16-character lowercase hexadecimal string"
        )
    try:
        parsed = int(value, 16)
    except ValueError as exc:
        raise AvbdPaperBreakableWallPacketError(f"{label} must be hexadecimal") from exc
    if f"{parsed:016x}" != value:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must use canonical lowercase hexadecimal"
        )
    return value


def _validate_resolved_configuration(metrics: dict[str, Any]) -> list[dict[str, str]]:
    notes = metrics.get("resolved_configuration")
    if not isinstance(notes, list):
        raise AvbdPaperBreakableWallPacketError(
            "scene metrics missing engine resolved_configuration"
        )
    normalized = []
    for index, note in enumerate(notes):
        if not isinstance(note, dict):
            raise AvbdPaperBreakableWallPacketError(
                f"resolved_configuration[{index}] must be an object"
            )
        normalized_note = {}
        for key in ("domain", "requested", "resolved", "reason"):
            value = note.get(key)
            if not isinstance(value, str) or not value:
                raise AvbdPaperBreakableWallPacketError(
                    f"resolved_configuration[{index}].{key} must be non-empty"
                )
            normalized_note[key] = value
        normalized.append(normalized_note)

    expected = {
        "rigid-body": ("avbd", "avbd"),
        "rigid-contact": ("avbd", "avbd"),
        "rigid-pair-constraint": ("avbd", "avbd"),
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
            raise AvbdPaperBreakableWallPacketError(
                "engine resolved_configuration must contain exactly one "
                f"{domain} {requested}->{resolved} note"
            )
    return normalized


def _validate_view_report(
    metrics: dict[str, Any],
    *,
    width: int,
    height: int,
) -> dict[str, Any]:
    report = metrics.get("view_report")
    if not isinstance(report, dict):
        raise AvbdPaperBreakableWallPacketError(
            "scene metrics missing engine ViewReport"
        )
    _require_exact(
        report.get("schema_version"),
        "dart.view_report/v1",
        "ViewReport schema_version",
    )
    _require_exact(report.get("pass"), True, "ViewReport pass")
    _require_exact(report.get("issues"), [], "ViewReport issues")
    _require_exact(report.get("size"), [width, height], "ViewReport size")
    _require_exact(report.get("focus"), list(VIEW_FOCUS), "ViewReport focus")
    camera = report.get("camera")
    if not isinstance(camera, dict):
        raise AvbdPaperBreakableWallPacketError("ViewReport missing camera")
    _require_close(
        camera.get("azimuth"),
        -0.5 * math.pi,
        "ViewReport front-camera azimuth",
    )
    _require_close(
        camera.get("elevation"),
        0.0,
        "ViewReport front-camera elevation",
    )
    _require_close(camera.get("distance"), CAMERA_DISTANCE, "ViewReport distance")
    target = camera.get("target")
    if not isinstance(target, list) or len(target) != 3:
        raise AvbdPaperBreakableWallPacketError(
            "ViewReport camera target must contain three coordinates"
        )
    for index, expected in enumerate(CAMERA_TARGET):
        _require_close(target[index], expected, f"ViewReport target[{index}]")
    report_metrics = report.get("metrics")
    if not isinstance(report_metrics, dict):
        raise AvbdPaperBreakableWallPacketError("ViewReport missing metrics")
    _require_exact(
        report_metrics.get("center_visible"),
        True,
        "ViewReport center_visible",
    )
    corner_coverage = _finite_number(
        report_metrics.get("corner_coverage"),
        "ViewReport corner_coverage",
    )
    subject_fraction = _finite_number(
        report_metrics.get("subject_fraction"),
        "ViewReport subject_fraction",
    )
    occlusion_fraction = _finite_number(
        report_metrics.get("occlusion_fraction"),
        "ViewReport occlusion_fraction",
    )
    ambiguity_iou = _finite_number(
        report_metrics.get("ambiguity_iou"),
        "ViewReport ambiguity_iou",
    )
    for key, value in (
        ("corner_coverage", corner_coverage),
        ("subject_fraction", subject_fraction),
        ("occlusion_fraction", occlusion_fraction),
        ("ambiguity_iou", ambiguity_iou),
    ):
        if not 0.0 <= value <= 1.0:
            raise AvbdPaperBreakableWallPacketError(
                f"ViewReport {key} must be in [0, 1]"
            )
    if corner_coverage < 0.999:
        raise AvbdPaperBreakableWallPacketError(
            "ViewReport corner_coverage fails the engine acceptance threshold"
        )
    if not 0.015 <= subject_fraction <= 0.75:
        raise AvbdPaperBreakableWallPacketError(
            "ViewReport subject_fraction fails the engine acceptance band"
        )
    if occlusion_fraction > 0.35:
        raise AvbdPaperBreakableWallPacketError(
            "ViewReport occlusion_fraction fails the engine acceptance threshold"
        )
    if ambiguity_iou > 0.55:
        raise AvbdPaperBreakableWallPacketError(
            "ViewReport ambiguity_iou fails the engine acceptance threshold"
        )
    score = _finite_number(report.get("score"), "ViewReport score")
    if not 0.0 <= score <= 1.0:
        raise AvbdPaperBreakableWallPacketError("ViewReport score must be in [0, 1]")
    return report


def _read_scene_metric_events(
    path: Path,
    *,
    expected_frame: int,
) -> list[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"{path}: scene metrics event log not found"
        ) from exc

    events = []
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError as exc:
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: invalid scene metric JSON: {exc}"
            ) from exc
        if not isinstance(event, dict):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: scene metric event must be an object"
            )
        _require_exact(
            event.get("event"),
            "scene_capture_metrics",
            f"{path}:{line_number} event",
        )
        _require_exact(
            event.get("scene"),
            SCENE_ID,
            f"{path}:{line_number} scene",
        )
        _require_exact(
            event.get("source"),
            "py-demo-scene",
            f"{path}:{line_number} source",
        )
        metrics = event.get("metrics")
        if not isinstance(metrics, dict):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: metrics must be an object"
            )
        _validate_finite_tree(metrics, f"{path}:{line_number}.metrics")
        outcome = metrics.get("outcome")
        if not isinstance(outcome, dict):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: metrics.outcome must be an object"
            )
        event_frame = event.get("frame")
        if not isinstance(event_frame, int) or isinstance(event_frame, bool):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: frame must be an integer"
            )
        _require_exact(
            outcome.get("frame"),
            event_frame,
            f"{path}:{line_number} outcome frame",
        )
        _require_close(
            outcome.get("world_time"),
            event_frame * TIME_STEP,
            f"{path}:{line_number} outcome world_time",
        )
        events.append(event)

    if len(events) != expected_frame:
        raise AvbdPaperBreakableWallPacketError(
            f"{path}: expected {expected_frame} scene metric events, "
            f"got {len(events)}"
        )
    frames = [event.get("frame") for event in events]
    if frames != list(range(1, expected_frame + 1)):
        raise AvbdPaperBreakableWallPacketError(
            f"{path}: scene metric frames must be exactly 1..{expected_frame}"
        )
    return events


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
        raise AvbdPaperBreakableWallPacketError(
            "capture manifest missing scene_metrics"
        )
    _require_exact(
        summary.get("event_count"),
        expected_frame,
        "scene metric event_count",
    )
    latest = summary.get("latest")
    if not isinstance(latest, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture scene_metrics missing latest event"
        )
    _require_exact(latest, logged_latest, "manifest latest scene metric event")
    _require_exact(latest.get("scene"), SCENE_ID, "latest scene metric scene")
    _require_exact(latest.get("frame"), expected_frame, "latest scene metric frame")
    metrics = latest.get("metrics")
    if not isinstance(metrics, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture latest scene metric payload must be an object"
        )

    exact_metrics = {
        "ball_count": IMPACTING_BALLS,
        "breakable_joints": BREAKABLE_JOINTS,
        "brick_count": BRICK_COUNT,
        "collision_shapes": COLLISION_SHAPES,
        "executor": "World.step default",
        "paper_locator": PAPER_LOCATOR,
        "rigid_bodies": RIGID_BODIES,
        "rigid_body_solver": "AVBD",
        "row": SCENE_ID,
        "solver": "public_avbd",
    }
    for key, expected in exact_metrics.items():
        _require_exact(metrics.get(key), expected, f"scene metrics {key}")
    _require_close(metrics.get("break_force"), BREAK_FORCE, "scene break_force")
    _require_close(
        metrics.get("time_step_ms"),
        TIME_STEP * 1000.0,
        "scene time_step_ms",
    )
    rigid_constraint_options = metrics.get("rigid_constraint_options")
    if not isinstance(rigid_constraint_options, dict):
        raise AvbdPaperBreakableWallPacketError(
            "scene metrics missing rigid_constraint_options"
        )
    _require_exact(
        rigid_constraint_options,
        {"iterations": RIGID_CONSTRAINT_ITERATIONS},
        "scene rigid_constraint_options",
    )
    resolved_configuration = _validate_resolved_configuration(metrics)
    view_report = _validate_view_report(metrics, width=width, height=height)
    scene_spec_fingerprint = _validate_scene_spec_fingerprint(
        metrics.get("scene_spec_fingerprint"),
        "scene metrics scene_spec_fingerprint",
    )

    oracle = metrics.get("outcome_oracle")
    if not isinstance(oracle, dict):
        raise AvbdPaperBreakableWallPacketError("scene metrics missing outcome_oracle")
    for key, expected in OUTCOME_ORACLE.items():
        if isinstance(expected, float):
            _require_close(oracle.get(key), expected, f"outcome oracle {key}")
        else:
            _require_exact(oracle.get(key), expected, f"outcome oracle {key}")

    outcome = metrics.get("outcome")
    if not isinstance(outcome, dict):
        raise AvbdPaperBreakableWallPacketError("scene metrics missing outcome")
    _validate_finite_tree(outcome, "outcome")
    expected_outcome = EXPECTED_OUTCOMES[expected_frame]
    for key in (
        "broken_joints",
        "evaluated",
        "impact_band_displaced_counts",
        "status",
        "thresholds_pass",
        "unbroken_joints",
    ):
        _require_exact(
            outcome.get(key),
            expected_outcome[key],
            f"frame {expected_frame} outcome {key}",
        )
    for key in ("outside_retained_fraction", "total_retained_fraction"):
        _require_close(
            outcome.get(key),
            expected_outcome[key],
            f"frame {expected_frame} outcome {key}",
        )
    _require_exact(
        outcome.get("last_step_iterations"),
        RIGID_CONSTRAINT_ITERATIONS,
        f"frame {expected_frame} last_step_iterations",
    )
    _require_exact(
        outcome.get("frame"),
        expected_frame,
        f"frame {expected_frame} outcome frame",
    )
    threshold_checks = outcome.get("threshold_checks")
    if not isinstance(threshold_checks, dict):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} outcome missing threshold_checks"
        )
    _require_exact(
        threshold_checks,
        expected_outcome["threshold_checks"],
        f"frame {expected_frame} outcome threshold_checks",
    )
    if (
        _finite_number(
            outcome.get("max_brick_displacement"),
            f"frame {expected_frame} max_brick_displacement",
        )
        <= 0.0
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} max_brick_displacement must be positive"
        )
    if (
        _finite_number(
            outcome.get("contact_count"),
            f"frame {expected_frame} contact_count",
        )
        <= 0.0
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} contact_count must be positive"
        )

    return {
        "event_count": expected_frame,
        "frame": expected_frame,
        "outcome": outcome,
        "outcome_oracle": oracle,
        "resolved_configuration": resolved_configuration,
        "scene_spec_fingerprint": scene_spec_fingerprint,
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
        | {"rigid_constraint_options": rigid_constraint_options},
    }


def _validate_capture(
    manifest_path: Path,
    *,
    expected_frame: int,
    expected_label: str,
) -> tuple[dict[str, Any], Path]:
    manifest = _load_json(manifest_path)
    _require_exact(manifest.get("schema_version"), 1, "capture schema_version")
    _require_exact(manifest.get("scene"), SCENE_ID, "capture scene")
    _require_exact(
        manifest.get("capture_label"),
        expected_label,
        "capture label",
    )
    _require_exact(manifest.get("force_drag"), None, "capture force_drag")

    capture = manifest.get("capture")
    if not isinstance(capture, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture manifest missing capture dimensions"
        )
    _require_exact(
        capture.get("requested_frames"),
        expected_frame,
        "capture requested_frames",
    )
    _require_exact(
        capture.get("converted_frames"),
        expected_frame,
        "capture converted_frames",
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
        raise AvbdPaperBreakableWallPacketError(
            "capture width and height must be positive integers"
        )

    identity = manifest.get("resolved_solver_identity")
    if identity != {
        "executor": "World.step default",
        "solver": "public_avbd",
        "source": "scene_capture_metrics.latest.metrics",
    }:
        raise AvbdPaperBreakableWallPacketError(
            "capture must resolve public_avbd through World.step default"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdPaperBreakableWallPacketError("capture manifest missing artifacts")
    screenshot = _artifact_path(
        manifest_path,
        artifacts.get("screenshot"),
        "artifacts.screenshot",
    )
    metrics_events = _artifact_path(
        manifest_path,
        artifacts.get("scene_metrics_events"),
        "artifacts.scene_metrics_events",
    )
    if not screenshot.is_file():
        raise AvbdPaperBreakableWallPacketError(f"{screenshot}: screenshot not found")
    if not metrics_events.is_file():
        raise AvbdPaperBreakableWallPacketError(
            f"{metrics_events}: scene metrics event log not found"
        )
    if _png_dimensions(screenshot) != (width, height):
        raise AvbdPaperBreakableWallPacketError(
            "capture screenshot dimensions do not match manifest"
        )
    metric_events = _read_scene_metric_events(
        metrics_events,
        expected_frame=expected_frame,
    )
    scene_metrics = _validate_scene_metrics(
        manifest,
        expected_frame=expected_frame,
        height=height,
        logged_latest=metric_events[-1],
        width=width,
    )

    return (
        {
            "camera": _validate_camera(manifest),
            "capture": {
                "height": height,
                "requested_frames": expected_frame,
                "width": width,
            },
            "label": expected_label,
            "manifest": {
                "file": manifest_path.name,
                "sha256": _sha256(manifest_path),
            },
            "scene_metrics": scene_metrics,
            "scene_metrics_events": {
                "event_count": len(metric_events),
                "file": metrics_events.name,
                "sha256": _sha256(metrics_events),
            },
            "screenshot": {
                "file": screenshot.name,
                "sha256": _sha256(screenshot),
            },
        },
        screenshot,
    )


def _validate_image_verdict(
    verdict_path: Path,
    screenshot: Path,
    *,
    expected_frame: int,
) -> dict[str, Any]:
    verdict = _load_json(verdict_path)
    _require_exact(
        verdict.get("schema_version"),
        "dart.image_verdict/v1",
        "image verdict schema_version",
    )
    _require_exact(verdict.get("pass"), True, "image verdict pass")
    _require_exact(
        verdict.get("machine_scope"),
        "pixel-integrity",
        "image verdict machine_scope",
    )
    checks = verdict.get("checks")
    if not isinstance(checks, dict):
        raise AvbdPaperBreakableWallPacketError("image verdict missing checks")
    non_blank = checks.get("non_blank")
    if not isinstance(non_blank, dict) or non_blank.get("pass") is not True:
        raise AvbdPaperBreakableWallPacketError(
            "image verdict non_blank check must pass"
        )
    semantic_review = verdict.get("semantic_review")
    if (
        not isinstance(semantic_review, dict)
        or semantic_review.get("required") is not True
        or semantic_review.get("performed_by_this_tool") is not False
    ):
        raise AvbdPaperBreakableWallPacketError(
            "image verdict must require a separate semantic review"
        )
    metadata = verdict.get("metadata")
    expected_metadata = {
        "frame": str(expected_frame),
        "scene": SCENE_ID,
        "view": CAMERA_VIEW,
    }
    _require_exact(metadata, expected_metadata, "image verdict metadata")

    image = verdict.get("image")
    if not isinstance(image, dict):
        raise AvbdPaperBreakableWallPacketError("image verdict missing image metadata")
    image_path = image.get("path")
    if (
        not isinstance(image_path, str)
        or Path(image_path).resolve() != screenshot.resolve()
    ):
        raise AvbdPaperBreakableWallPacketError(
            "image verdict path does not match capture screenshot"
        )
    width, height = _png_dimensions(screenshot)
    _require_exact(image.get("width"), width, "image verdict width")
    _require_exact(image.get("height"), height, "image verdict height")
    return {
        "checks": checks,
        "file": verdict_path.name,
        "machine_scope": "pixel-integrity",
        "metadata": expected_metadata,
        "pass": True,
        "sha256": _sha256(verdict_path),
    }


def _canonical_benchmark_name(row: dict[str, Any]) -> str:
    name = row.get("run_name", row.get("name"))
    if not isinstance(name, str):
        return ""
    for suffix in ("_mean", "_median", "_stddev", "_cv"):
        if name.endswith(suffix):
            name = name[: -len(suffix)]
            break
    return name


def _validate_benchmark(
    benchmark_path: Path,
    *,
    expected_scene_spec_fingerprint: str,
) -> dict[str, Any]:
    data = _load_json(benchmark_path)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark JSON missing benchmarks list"
        )
    matching = [
        row
        for row in rows
        if isinstance(row, dict) and _canonical_benchmark_name(row) == BENCHMARK_RUN
    ]
    by_aggregate: dict[str, dict[str, Any]] = {}
    for row in matching:
        aggregate = row.get("aggregate_name")
        if not isinstance(aggregate, str):
            raise AvbdPaperBreakableWallPacketError(
                f"{BENCHMARK_RUN}: every row must be an aggregate"
            )
        if aggregate in by_aggregate:
            raise AvbdPaperBreakableWallPacketError(
                f"{BENCHMARK_RUN}: duplicate {aggregate} aggregate"
            )
        by_aggregate[aggregate] = row
    expected_aggregates = {"mean", "median", "stddev", "cv"}
    if set(by_aggregate) != expected_aggregates:
        raise AvbdPaperBreakableWallPacketError(
            f"{BENCHMARK_RUN}: expected aggregates {sorted(expected_aggregates)}, "
            f"got {sorted(by_aggregate)}"
        )

    counters = {
        "breakable_joints": BREAKABLE_JOINTS,
        "collision_shapes": COLLISION_SHAPES,
        "impacting_balls": IMPACTING_BALLS,
        "public_avbd_family": 1,
        "resolved_rigid_body_avbd": 1,
        "resolved_rigid_constraint_iterations": 1,
        "resolved_rigid_contact_avbd": 1,
        "resolved_rigid_pair_constraint_avbd": 1,
        "rigid_constraint_iterations": RIGID_CONSTRAINT_ITERATIONS,
        "rigid_bodies": RIGID_BODIES,
        "rigid_body_joints": BREAKABLE_JOINTS,
        "runtime_contract_passed": 1,
        "trajectory_frames": 120,
    }
    expected_fingerprint_value = int(expected_scene_spec_fingerprint, 16)
    fingerprint_counters = {
        "scene_spec_fingerprint_hi": expected_fingerprint_value >> 32,
        "scene_spec_fingerprint_lo": expected_fingerprint_value & 0xFFFFFFFF,
    }
    packet_rows = []
    for aggregate in ("mean", "median", "stddev", "cv"):
        row = by_aggregate[aggregate]
        _require_exact(row.get("run_type"), "aggregate", "benchmark run_type")
        _require_exact(row.get("repetitions"), 5, "benchmark repetitions")
        _require_exact(row.get("iterations"), 5, "benchmark iterations")
        _require_exact(row.get("time_unit"), "ns", "benchmark time_unit")
        real_time = _finite_number(
            row.get("real_time"),
            f"{BENCHMARK_RUN} {aggregate} real_time",
        )
        cpu_time = _finite_number(
            row.get("cpu_time"),
            f"{BENCHMARK_RUN} {aggregate} cpu_time",
        )
        if real_time < 0.0 or cpu_time < 0.0:
            raise AvbdPaperBreakableWallPacketError(
                f"{BENCHMARK_RUN}: aggregate timings must be non-negative"
            )
        if aggregate in ("mean", "median"):
            if real_time <= 0.0 or cpu_time <= 0.0:
                raise AvbdPaperBreakableWallPacketError(
                    f"{BENCHMARK_RUN}: representative timings must be positive"
                )
            for key, expected in counters.items():
                _require_close(
                    row.get(key),
                    float(expected),
                    f"{BENCHMARK_RUN} {aggregate} {key}",
                )
            for key, expected in fingerprint_counters.items():
                _require_close(
                    row.get(key),
                    float(expected),
                    f"{BENCHMARK_RUN} {aggregate} {key}",
                )
        packet_rows.append(
            {
                key: row[key]
                for key in (
                    "aggregate_name",
                    "aggregate_unit",
                    "cpu_time",
                    "iterations",
                    "real_time",
                    "repetitions",
                    "run_name",
                    "run_type",
                    "time_unit",
                )
                if key in row
            }
            | (
                {key: row[key] for key in (*counters, *fingerprint_counters)}
                if aggregate in ("mean", "median")
                else {}
            )
        )

    cv_row = by_aggregate["cv"]
    real_cv = _finite_number(cv_row.get("real_time"), "benchmark real-time CV")
    cpu_cv = _finite_number(cv_row.get("cpu_time"), "benchmark CPU-time CV")
    if not 0.0 <= real_cv <= 0.10 or not 0.0 <= cpu_cv <= 0.10:
        raise AvbdPaperBreakableWallPacketError(
            f"{BENCHMARK_RUN}: timing CV must be between 0 and 10 percent"
        )

    context = data.get("context")
    if not isinstance(context, dict):
        raise AvbdPaperBreakableWallPacketError("benchmark JSON missing context object")
    executable = context.get("executable")
    if not isinstance(executable, str) or Path(executable).name != (
        "bm_avbd_rigid_fixed_joint"
    ):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark context identifies the wrong executable"
        )
    _require_exact(
        context.get("library_build_type"),
        "release",
        "benchmark library_build_type",
    )
    return {
        "benchmark": BENCHMARK_NAME,
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
            )
            if key in context
        },
        "json_sha256": _sha256(benchmark_path),
        "rows": packet_rows,
        "scene_spec_fingerprint": expected_scene_spec_fingerprint,
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


def _validate_paper_artifacts(
    paper_pdf: Path,
    paper_figure: Path,
) -> dict[str, Any]:
    pdf_hash = _sha256(paper_pdf)
    figure_hash = _sha256(paper_figure)
    if pdf_hash != PAPER_PDF_SHA256:
        raise AvbdPaperBreakableWallPacketError(
            f"paper PDF sha256 must be {PAPER_PDF_SHA256}, got {pdf_hash}"
        )
    if figure_hash != PAPER_FIGURE_SHA256:
        raise AvbdPaperBreakableWallPacketError(
            f"paper figure sha256 must be {PAPER_FIGURE_SHA256}, got {figure_hash}"
        )
    width, height = _png_dimensions(paper_figure)
    return {
        "figure": {
            "file": paper_figure.name,
            "height": height,
            "sha256": figure_hash,
            "width": width,
        },
        "locator": PAPER_LOCATOR,
        "paper_pdf": {
            "file": paper_pdf.name,
            "sha256": pdf_hash,
        },
    }


def _validate_visual_review(
    review_path: Path,
    *,
    impact_screenshot: Path,
    outcome_screenshot: Path,
    paper_figure: Path,
) -> dict[str, Any]:
    review = _load_json(review_path)
    _require_exact(
        review.get("schema_version"),
        "dart.visual_semantic_review/v1",
        "visual review schema_version",
    )
    _require_exact(review.get("scene"), SCENE_ID, "visual review scene")
    _require_exact(review.get("verdict"), "pass", "visual review verdict")
    review_fields = (
        "reviewer_capability",
        "claim_and_expected_observation",
        "text_oracle",
        "visible_observation",
        "reconciliation_and_verdict",
        "not_proven_and_limitations",
    )
    for key in review_fields:
        value = review.get(key)
        if not isinstance(value, str) or not value.strip():
            raise AvbdPaperBreakableWallPacketError(
                f"visual review {key} must be non-empty"
            )
    if "image" not in review["reviewer_capability"].lower():
        raise AvbdPaperBreakableWallPacketError(
            "visual review must name an image-capable reviewer"
        )
    if (
        "viewreport"
        not in review["reconciliation_and_verdict"].replace(" ", "").lower()
    ):
        raise AvbdPaperBreakableWallPacketError(
            "visual review reconciliation must account for the engine ViewReports"
        )

    expected = {
        "impact_frame_60": impact_screenshot,
        "outcome_frame_120": outcome_screenshot,
        "paper_figure_13_reference": paper_figure,
    }
    entries = review.get("inspected_images")
    if not isinstance(entries, list):
        raise AvbdPaperBreakableWallPacketError(
            "visual review inspected_images must be a list"
        )
    by_role = {
        entry.get("role"): entry
        for entry in entries
        if isinstance(entry, dict) and isinstance(entry.get("role"), str)
    }
    if set(by_role) != set(expected):
        raise AvbdPaperBreakableWallPacketError(
            "visual review must inspect the impact, outcome, and paper images"
        )
    inspected_images = []
    for role, path in expected.items():
        entry = by_role[role]
        file_value = entry.get("file")
        if (
            not isinstance(file_value, str)
            or Path(file_value).resolve() != path.resolve()
        ):
            raise AvbdPaperBreakableWallPacketError(
                f"visual review {role} file does not match the inspected image"
            )
        expected_hash = _sha256(path)
        _require_exact(
            entry.get("sha256"),
            expected_hash,
            f"visual review {role} sha256",
        )
        inspected_images.append(
            {
                "file": path.name,
                "role": role,
                "sha256": expected_hash,
            }
        )
    return {
        "claim_and_expected_observation": review["claim_and_expected_observation"],
        "file": review_path.name,
        "inspected_images": inspected_images,
        "not_proven_and_limitations": review["not_proven_and_limitations"],
        "reconciliation_and_verdict": review["reconciliation_and_verdict"],
        "reviewer_capability": review["reviewer_capability"],
        "sha256": _sha256(review_path),
        "text_oracle": review["text_oracle"],
        "verdict": "pass",
        "visible_observation": review["visible_observation"],
    }


def make_packet(
    *,
    benchmark_json: Path,
    impact_capture_manifest: Path,
    impact_image_verdict_json: Path,
    outcome_capture_manifest: Path,
    outcome_image_verdict_json: Path,
    visual_review_json: Path,
    paper_pdf: Path,
    paper_figure_image: Path,
) -> dict[str, Any]:
    impact_capture, impact_screenshot = _validate_capture(
        impact_capture_manifest,
        expected_frame=60,
        expected_label="impact",
    )
    impact_capture["image_verdict"] = _validate_image_verdict(
        impact_image_verdict_json,
        impact_screenshot,
        expected_frame=60,
    )
    outcome_capture, outcome_screenshot = _validate_capture(
        outcome_capture_manifest,
        expected_frame=120,
        expected_label="outcome",
    )
    outcome_capture["image_verdict"] = _validate_image_verdict(
        outcome_image_verdict_json,
        outcome_screenshot,
        expected_frame=120,
    )
    paper_reference = _validate_paper_artifacts(paper_pdf, paper_figure_image)
    semantic_review = _validate_visual_review(
        visual_review_json,
        impact_screenshot=impact_screenshot,
        outcome_screenshot=outcome_screenshot,
        paper_figure=paper_figure_image,
    )
    scene_spec_fingerprint = impact_capture["scene_metrics"]["scene_spec_fingerprint"]
    _require_exact(
        outcome_capture["scene_metrics"]["scene_spec_fingerprint"],
        scene_spec_fingerprint,
        "impact/outcome scene_spec_fingerprint",
    )
    benchmark = _validate_benchmark(
        benchmark_json,
        expected_scene_spec_fingerprint=scene_spec_fingerprint,
    )

    return {
        "benchmark": benchmark,
        "claim_boundary": (
            "This packet covers the AVBD row of the publication-shaped "
            "Figure 13 breakable-wall outcome on the public CPU solver family. "
            "It does not claim exact source-scene replay, reproduce the "
            "Sequential Impulse, XPBD, and VBD comparison rows, or establish "
            "public CUDA AVBD parity."
        ),
        "correctness": {
            "allocation_tests": [
                "World.BakedStepsDoNotGrowWorldBaseAllocatorForReservedEcsPaths",
                "World.BakedRigidBodyContactStepsDoNotAllocateGlobalHeap",
                "World.BakedAvbdVbdRowsDoNotMallocOnHeap",
            ],
            "allocation_scene": (
                "configurePublicRigidAvbdContactAndBreakableJointRowsScene"
            ),
            "determinism_test": (
                "test_avbd_paper_breakable_wall_outcome_is_deterministic"
            ),
            "outcome_test": (
                "test_avbd_paper_breakable_wall_matches_figure13_contract"
            ),
            "public_configuration": {
                "rigid_body_solver": "AVBD",
                "rigid_constraint_options": {
                    "iterations": RIGID_CONSTRAINT_ITERATIONS,
                },
                "scene_spec_fingerprint": scene_spec_fingerprint,
                "time_step": TIME_STEP,
            },
        },
        "packet": "avbd_paper_breakable_wall",
        "paper_reference": paper_reference,
        "performance_claim_boundary": (
            "The benchmark records absolute DART Release-build CPU step cost "
            "and stability for this trajectory. The paper and source artifacts "
            "publish no directly comparable timing for this scene, so no "
            "reference ratio or speedup claim is made."
        ),
        "reconstruction": {
            "published": {
                "description": (
                    "Three high-momentum balls fracture a brick wall assembled "
                    "with breakable attachments."
                ),
                "constraint_iterations_or_substeps": (RIGID_CONSTRAINT_ITERATIONS),
                "paper_locator": PAPER_LOCATOR,
                "time_step": TIME_STEP,
            },
            "reconstructed_because_unpublished": {
                "ball_mass": 100.0,
                "ball_radius": 0.48,
                "ball_speed": 24.0,
                "break_force": BREAK_FORCE,
                "brick_density": 4.0,
                "brick_size": [0.6, 0.3, 0.25],
                "target_coordinates_xz": [
                    [-3.1, 1.55],
                    [0.0, 1.75],
                    [3.1, 2.35],
                ],
                "wall_layout": {
                    "columns": 21,
                    "rows": 12,
                    "staggered": True,
                },
            },
        },
        "resolved_solver_identity": RESOLVED_SOLVER_IDENTITY,
        "schema_version": AVBD_PACKET_SCHEMA_VERSION,
        "scene": SCENE_ID,
        "source_provenance": _source_provenance(),
        "target": {
            "complete_paper_reproduction": False,
            "contract_rows": [
                "avbd.method.joints_and_attachments",
                "avbd.method.fracture",
                "avbd.paper.fig.13",
            ],
            "covered_slice": (
                "AVBD Figure 13 publication-shaped breakable-wall row with "
                "numeric, allocation, performance, and visual evidence"
            ),
        },
        "visual_evidence": {
            "impact": impact_capture,
            "outcome": outcome_capture,
            "semantic_review": semantic_review,
        },
        "reproduction": {
            "benchmark_command": (
                "pixi run bm -- bm_avbd_rigid_fixed_joint -- "
                f"--benchmark_filter='^{BENCHMARK_RUN}$' "
                "--benchmark_repetitions=5 "
                "--benchmark_report_aggregates_only=true "
                "--benchmark_out=<benchmark-json> "
                "--benchmark_out_format=json"
            ),
            "capture_commands": [
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 60 --width 1280 "
                    "--height 720 --view front --camera-distance 11 "
                    "--camera-target 0,0,1.4 --capture-label impact "
                    "--output-dir <impact-capture-dir>"
                ),
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 120 --width 1280 "
                    "--height 720 --view front --camera-distance 11 "
                    "--camera-target 0,0,1.4 --capture-label outcome "
                    "--output-dir <outcome-capture-dir>"
                ),
            ],
            "image_verdict_commands": [
                (
                    "pixi run image-verdict -- "
                    f"<impact-capture-dir>/{SCENE_ID}_impact.png "
                    f"--meta scene={SCENE_ID} --meta frame=60 "
                    "--meta view=front "
                    "--out <impact-capture-dir>/image_verdict.json"
                ),
                (
                    "pixi run image-verdict -- "
                    f"<outcome-capture-dir>/{SCENE_ID}_outcome.png "
                    f"--meta scene={SCENE_ID} --meta frame=120 "
                    "--meta view=front "
                    "--out <outcome-capture-dir>/image_verdict.json"
                ),
            ],
        },
    }


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(
            benchmark_json=args.benchmark_json,
            impact_capture_manifest=args.impact_capture_manifest,
            impact_image_verdict_json=args.impact_image_verdict_json,
            outcome_capture_manifest=args.outcome_capture_manifest,
            outcome_image_verdict_json=args.outcome_image_verdict_json,
            visual_review_json=args.visual_review_json,
            paper_pdf=args.paper_pdf,
            paper_figure_image=args.paper_figure_image,
        )
    except (OSError, ValueError, AvbdPaperBreakableWallPacketError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
