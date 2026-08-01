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
    PAPER_PACKET_SOURCE_PATHS,
    make_resolved_solver_identity,
)
from capture_source_provenance import (  # noqa: E402
    CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
    CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
    CAPTURE_SOURCE_ROOTS,
    CaptureSourceProvenanceError,
    compute_capture_source_provenance,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-paper-breakable-wall-packet.json"
)
SCENE_ID = "avbd_paper_breakable_wall"
BENCHMARK_NAME = "BM_AvbdPaperBreakableWallStep"
BENCHMARK_RUN = f"{BENCHMARK_NAME}/iterations:120"
BENCHMARK_SOURCE_PATH = Path("tests/benchmark/simulation/bm_avbd_rigid_fixed_joint.cpp")
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
BREAK_FORCE = 5000.0
CAMERA_AZIMUTH = -5.0 * math.pi / 8.0
CAMERA_ELEVATION = 0.62
CAMERA_DISTANCE = 22.0
CAMERA_TARGET = (0.0, 0.45, 1.6)
CAMERA_PRESET = "front"
CAMERA_VIEW = "front-oblique"
VIEW_FOCUS = tuple(
    f"avbd_paper_wall_brick_{row:02d}_{column:02d}_visual"
    for row in range(12)
    for column in range(21)
)

OUTCOME_ORACLE = {
    "evaluation_frame": 120,
    "impact_band_radius": 0.85,
    "impact_damage_displacement_threshold": 0.1,
    "joint_evidence_frames": [60, 120],
    "maximum_broken_joints": 250,
    "maximum_unbroken_joint_angular_residual_radians": 0.001,
    "maximum_unbroken_joint_linear_residual": 0.002,
    "minimum_broken_joints": 150,
    "minimum_displaced_bricks_per_impact_band": 4,
    "minimum_outside_retained_fraction": 0.95,
    "minimum_total_retained_fraction": 0.95,
    "minimum_unbroken_joints": 450,
    "outside_radius": 1.15,
    "retained_displacement_threshold": 0.5,
    "expected_broken_joint_ids_sha256": (
        "31b187538ac1549563be368a4d7e304d1caef6ea11ba65b624d48f5f27468503"
    ),
}

EXPECTED_OUTCOMES = {
    60: {
        "broken_joints": 154,
        "evaluated": False,
        "impact_band_displaced_counts": [13, 14, 12],
        "outside_retained_fraction": 1.0,
        "status": "pre-evaluation",
        "threshold_checks": {
            "damage_in_three_impact_bands": True,
            "finite_state": True,
            "fracture_activated": True,
            "fracture_count_bounded": True,
            "fracture_identity_matches": True,
            "outside_wall_retained": True,
            "retained_joint_rows_satisfied": True,
            "total_wall_retained": True,
        },
        "thresholds_pass": False,
        "total_retained_fraction": 0.996031746031746,
        "unbroken_joints": 558,
    },
    120: {
        "broken_joints": 154,
        "evaluated": True,
        "impact_band_displaced_counts": [11, 14, 11],
        "outside_retained_fraction": 0.9943181818181818,
        "status": "pass",
        "threshold_checks": {
            "damage_in_three_impact_bands": True,
            "finite_state": True,
            "fracture_activated": True,
            "fracture_count_bounded": True,
            "fracture_identity_matches": True,
            "outside_wall_retained": True,
            "retained_joint_rows_satisfied": True,
            "total_wall_retained": True,
        },
        "thresholds_pass": True,
        "total_retained_fraction": 0.9920634920634921,
        "unbroken_joints": 558,
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
SOURCE_PATHS = tuple(
    Path(path) for path in PAPER_PACKET_SOURCE_PATHS[DEFAULT_OUTPUT.name]
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


def _benchmark_reproduction_command(filter_pattern: str) -> str:
    return (
        "capture_source_digest=$(pixi run python "
        "scripts/capture_source_provenance.py --digest-only) && "
        "benchmark_source_digest=$(sha256sum "
        f"{BENCHMARK_SOURCE_PATH.as_posix()} | cut -d' ' -f1) && "
        "pixi run bm -- bm_avbd_rigid_fixed_joint -- "
        f"--benchmark_filter='{filter_pattern}' "
        "--benchmark_repetitions=5 "
        "--benchmark_report_aggregates_only=true "
        "--benchmark_context=capture_source_provenance_digest="
        "$capture_source_digest "
        "--benchmark_context=benchmark_source_sha256="
        "$benchmark_source_digest "
        "--benchmark_out=<benchmark-json> "
        "--benchmark_out_format=json"
    )


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


def _validate_sha256_hex(value: object, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must be a 64-character lowercase hexadecimal string"
        )
    try:
        parsed = int(value, 16)
    except ValueError as exc:
        raise AvbdPaperBreakableWallPacketError(f"{label} must be hexadecimal") from exc
    if f"{parsed:064x}" != value:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must use canonical lowercase hexadecimal"
        )
    return value


def _validate_capture_provenance(
    manifest: dict[str, Any],
    *,
    metrics_events: Path,
    screenshot: Path,
) -> dict[str, Any]:
    recorded_source = manifest.get("capture_source_provenance")
    if not isinstance(recorded_source, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture manifest missing capture_source_provenance"
        )
    _require_exact(
        recorded_source.get("algorithm"),
        CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
        "capture source provenance algorithm",
    )
    _require_exact(
        recorded_source.get("roots"),
        list(CAPTURE_SOURCE_ROOTS),
        "capture source provenance roots",
    )
    recorded_digest = _validate_sha256_hex(
        recorded_source.get("digest"),
        "capture source provenance digest",
    )
    recorded_file_count = recorded_source.get("file_count")
    if (
        not isinstance(recorded_file_count, int)
        or isinstance(recorded_file_count, bool)
        or recorded_file_count < 1
    ):
        raise AvbdPaperBreakableWallPacketError(
            "capture source provenance file_count must be a positive integer"
        )
    recorded_head = recorded_source.get("git_head")
    if (
        not isinstance(recorded_head, str)
        or len(recorded_head) != 40
        or any(character not in "0123456789abcdef" for character in recorded_head)
    ):
        raise AvbdPaperBreakableWallPacketError(
            "capture source provenance git_head must be a lowercase Git object ID"
        )
    try:
        current_source = compute_capture_source_provenance(REPO_ROOT)
    except CaptureSourceProvenanceError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot validate capture source provenance: {exc}"
        ) from exc
    _require_exact(
        recorded_file_count,
        current_source["file_count"],
        "capture source provenance file_count",
    )
    _require_exact(
        recorded_digest,
        current_source["digest"],
        "capture source provenance digest",
    )

    recorded_artifacts = manifest.get("capture_artifact_provenance")
    if not isinstance(recorded_artifacts, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture manifest missing capture_artifact_provenance"
        )
    _require_exact(
        recorded_artifacts.get("algorithm"),
        CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
        "capture artifact provenance algorithm",
    )
    _require_exact(
        _validate_sha256_hex(
            recorded_artifacts.get("scene_metrics_events_sha256"),
            "capture artifact scene_metrics_events_sha256",
        ),
        _sha256(metrics_events),
        "capture artifact scene_metrics_events_sha256",
    )
    _require_exact(
        _validate_sha256_hex(
            recorded_artifacts.get("screenshot_sha256"),
            "capture artifact screenshot_sha256",
        ),
        _sha256(screenshot),
        "capture artifact screenshot_sha256",
    )
    return {
        "algorithm": CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
        "digest": recorded_digest,
        "file_count": recorded_file_count,
        "git_head": recorded_head,
        "roots": list(CAPTURE_SOURCE_ROOTS),
    }


def _validate_benchmark_source_provenance(
    context: dict[str, Any],
) -> dict[str, str]:
    try:
        current_source = compute_capture_source_provenance(REPO_ROOT)
    except CaptureSourceProvenanceError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot validate benchmark source provenance: {exc}"
        ) from exc
    capture_digest = _validate_sha256_hex(
        context.get("capture_source_provenance_digest"),
        "benchmark capture_source_provenance_digest",
    )
    _require_exact(
        capture_digest,
        current_source["digest"],
        "benchmark capture_source_provenance_digest",
    )
    benchmark_source_hash = _validate_sha256_hex(
        context.get("benchmark_source_sha256"),
        "benchmark benchmark_source_sha256",
    )
    _require_exact(
        benchmark_source_hash,
        _sha256(REPO_ROOT / BENCHMARK_SOURCE_PATH),
        "benchmark benchmark_source_sha256",
    )
    return {
        "benchmark_source_sha256": benchmark_source_hash,
        "capture_source_provenance_digest": capture_digest,
    }


def _validate_joint_endpoint(value: object, label: str) -> None:
    if not isinstance(value, dict):
        raise AvbdPaperBreakableWallPacketError(f"{label} must be an object")
    if set(value) != {"body", "column", "row"}:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must contain only body, column, and row"
        )
    body = value.get("body")
    column = value.get("column")
    row = value.get("row")
    if body == "ground":
        if column is not None or row is not None:
            raise AvbdPaperBreakableWallPacketError(
                f"{label} ground endpoint must use null grid coordinates"
            )
        return
    if body != "brick":
        raise AvbdPaperBreakableWallPacketError(f"{label}.body must be ground or brick")
    if (
        not isinstance(column, int)
        or isinstance(column, bool)
        or not 0 <= column < 21
        or not isinstance(row, int)
        or isinstance(row, bool)
        or not 0 <= row < 12
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"{label} brick grid coordinates are out of range"
        )


def _validate_joint_evidence(
    outcome: dict[str, Any],
    *,
    expected_broken_count: int,
    expected_broken_ids_sha256: str,
    expected_outside_unbroken_count: int,
    label: str,
) -> None:
    _require_exact(
        outcome.get("joint_residuals_finite"),
        True,
        f"{label} joint_residuals_finite",
    )
    _require_exact(
        outcome.get("broken_joint_identity_count"),
        expected_broken_count,
        f"{label} broken_joint_identity_count",
    )
    _require_exact(
        outcome.get("unbroken_joint_residual_count"),
        BREAKABLE_JOINTS - expected_broken_count,
        f"{label} unbroken_joint_residual_count",
    )
    _require_exact(
        outcome.get("outside_impact_unbroken_joint_residual_count"),
        expected_outside_unbroken_count,
        f"{label} outside_impact_unbroken_joint_residual_count",
    )

    records = outcome.get("broken_joint_records")
    if not isinstance(records, list):
        raise AvbdPaperBreakableWallPacketError(
            f"{label} broken_joint_records must be a list"
        )
    _require_exact(
        len(records),
        expected_broken_count,
        f"{label} broken_joint_records count",
    )
    record_ids = []
    derived_impact_counts = [0, 0, 0]
    derived_outside_count = 0
    for index, record in enumerate(records):
        record_label = f"{label} broken_joint_records[{index}]"
        if not isinstance(record, dict):
            raise AvbdPaperBreakableWallPacketError(f"{record_label} must be an object")
        expected_keys = {
            "angular_residual_radians",
            "child",
            "id",
            "initial_anchor",
            "kind",
            "linear_residual",
            "nearest_impact_distance",
            "nearest_impact_index",
            "parent",
            "within_impact_band",
            "within_impact_region",
        }
        if set(record) != expected_keys:
            raise AvbdPaperBreakableWallPacketError(
                f"{record_label} has an unexpected field set"
            )
        joint_id = record.get("id")
        if not isinstance(joint_id, str) or not joint_id:
            raise AvbdPaperBreakableWallPacketError(
                f"{record_label}.id must be non-empty"
            )
        record_ids.append(joint_id)
        _require_exact(
            record.get("kind") in {"horizontal", "vertical", "base"},
            True,
            f"{record_label}.kind validity",
        )
        _validate_joint_endpoint(record.get("parent"), f"{record_label}.parent")
        _validate_joint_endpoint(record.get("child"), f"{record_label}.child")
        anchor = record.get("initial_anchor")
        if not isinstance(anchor, list) or len(anchor) != 3:
            raise AvbdPaperBreakableWallPacketError(
                f"{record_label}.initial_anchor must have three coordinates"
            )
        for coordinate_index, coordinate in enumerate(anchor):
            _finite_number(
                coordinate,
                f"{record_label}.initial_anchor[{coordinate_index}]",
            )
        for residual_key in (
            "angular_residual_radians",
            "linear_residual",
            "nearest_impact_distance",
        ):
            if (
                _finite_number(
                    record.get(residual_key),
                    f"{record_label}.{residual_key}",
                )
                < 0.0
            ):
                raise AvbdPaperBreakableWallPacketError(
                    f"{record_label}.{residual_key} must be non-negative"
                )
        impact_index = record.get("nearest_impact_index")
        if (
            not isinstance(impact_index, int)
            or isinstance(impact_index, bool)
            or not 0 <= impact_index < 3
        ):
            raise AvbdPaperBreakableWallPacketError(
                f"{record_label}.nearest_impact_index must be 0, 1, or 2"
            )
        within_band = record.get("within_impact_band")
        within_region = record.get("within_impact_region")
        if not isinstance(within_band, bool) or not isinstance(within_region, bool):
            raise AvbdPaperBreakableWallPacketError(
                f"{record_label} impact-membership fields must be booleans"
            )
        if within_band and not within_region:
            raise AvbdPaperBreakableWallPacketError(
                f"{record_label} impact-band membership must imply region membership"
            )
        if within_region:
            derived_impact_counts[impact_index] += 1
        else:
            derived_outside_count += 1

    if len(set(record_ids)) != len(record_ids):
        raise AvbdPaperBreakableWallPacketError(
            f"{label} broken joint IDs must be unique"
        )
    identity_digest = sha256()
    for joint_id in sorted(record_ids):
        encoded_id = joint_id.encode("utf-8")
        identity_digest.update(struct.pack("<Q", len(encoded_id)))
        identity_digest.update(encoded_id)
    recorded_digest = _validate_sha256_hex(
        outcome.get("broken_joint_ids_sha256"),
        f"{label} broken_joint_ids_sha256",
    )
    _require_exact(
        recorded_digest,
        identity_digest.hexdigest(),
        f"{label} broken-joint identity digest",
    )
    _require_exact(
        recorded_digest,
        expected_broken_ids_sha256,
        f"{label} expected broken-joint identity digest",
    )
    _require_exact(
        outcome.get("broken_joint_impact_region_counts"),
        derived_impact_counts,
        f"{label} broken_joint_impact_region_counts",
    )
    _require_exact(
        outcome.get("broken_joints_outside_impact_regions"),
        derived_outside_count,
        f"{label} broken_joints_outside_impact_regions",
    )

    residual_pairs = (
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
    )
    for maximum_key, rms_key in residual_pairs:
        maximum = _finite_number(outcome.get(maximum_key), f"{label} {maximum_key}")
        rms = _finite_number(outcome.get(rms_key), f"{label} {rms_key}")
        if maximum < 0.0 or rms < 0.0 or rms > maximum:
            raise AvbdPaperBreakableWallPacketError(
                f"{label} {maximum_key}/{rms_key} residual summary is invalid"
            )


def _validate_camera(manifest: dict[str, Any]) -> dict[str, Any]:
    camera = manifest.get("camera")
    if not isinstance(camera, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture manifest missing serialized camera"
        )
    _require_exact(camera.get("view"), CAMERA_PRESET, "camera preset")
    _require_close(
        camera.get("azimuth"),
        math.degrees(CAMERA_AZIMUTH),
        "camera azimuth",
    )
    _require_close(
        camera.get("elevation"),
        math.degrees(CAMERA_ELEVATION),
        "camera elevation",
    )
    _require_close(camera.get("distance"), CAMERA_DISTANCE, "camera distance")
    target = camera.get("target")
    if not isinstance(target, list) or len(target) != 3:
        raise AvbdPaperBreakableWallPacketError(
            "camera target must contain three coordinates"
        )
    for index, expected in enumerate(CAMERA_TARGET):
        _require_close(target[index], expected, f"camera target[{index}]")
    return {
        "azimuth": CAMERA_AZIMUTH,
        "distance": CAMERA_DISTANCE,
        "elevation": CAMERA_ELEVATION,
        "preset": CAMERA_PRESET,
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
    expected_focus: tuple[str, ...] = VIEW_FOCUS,
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
    _require_exact(
        report.get("focus"),
        list(expected_focus),
        "ViewReport focus",
    )
    camera = report.get("camera")
    if not isinstance(camera, dict):
        raise AvbdPaperBreakableWallPacketError("ViewReport missing camera")
    _require_close(
        camera.get("azimuth"),
        CAMERA_AZIMUTH,
        "ViewReport camera azimuth",
    )
    _require_close(
        camera.get("elevation"),
        CAMERA_ELEVATION,
        "ViewReport camera elevation",
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
    expected_scene_id: str = SCENE_ID,
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
            expected_scene_id,
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
    _validate_joint_evidence(
        outcome,
        expected_broken_count=expected_outcome["broken_joints"],
        expected_broken_ids_sha256=OUTCOME_ORACLE["expected_broken_joint_ids_sha256"],
        expected_outside_unbroken_count=405,
        label=f"frame {expected_frame} outcome",
    )
    _require_exact(
        outcome.get("broken_joint_impact_region_counts"),
        [18, 44, 13],
        f"frame {expected_frame} broken_joint_impact_region_counts",
    )
    _require_exact(
        outcome.get("broken_joints_outside_impact_regions"),
        79,
        f"frame {expected_frame} broken_joints_outside_impact_regions",
    )
    maximum_linear_residual = _finite_number(
        outcome.get("maximum_unbroken_joint_linear_residual"),
        f"frame {expected_frame} maximum_unbroken_joint_linear_residual",
    )
    if (
        maximum_linear_residual
        > OUTCOME_ORACLE["maximum_unbroken_joint_linear_residual"]
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} retained-joint linear residual exceeds "
            "the AVBD oracle"
        )
    maximum_angular_residual = _finite_number(
        outcome.get("maximum_unbroken_joint_angular_residual_radians"),
        f"frame {expected_frame} " "maximum_unbroken_joint_angular_residual_radians",
    )
    if (
        maximum_angular_residual
        > OUTCOME_ORACLE["maximum_unbroken_joint_angular_residual_radians"]
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} retained-joint angular residual exceeds "
            "the AVBD oracle"
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
    capture_source_provenance = _validate_capture_provenance(
        manifest,
        metrics_events=metrics_events,
        screenshot=screenshot,
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
            "source_provenance": capture_source_provenance,
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
    expected_scene_id: str = SCENE_ID,
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
        "scene": expected_scene_id,
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
    benchmark_source_provenance = _validate_benchmark_source_provenance(context)
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
                "benchmark_source_sha256",
                "capture_source_provenance_digest",
            )
            if key in context
        },
        "source_provenance": benchmark_source_provenance,
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
                "ball_mass": 40.0,
                "ball_radius": 0.48,
                "ball_speed": 24.0,
                "break_force": BREAK_FORCE,
                "brick_density": 200.0,
                "brick_size": [0.6, 0.3, 0.25],
                "target_coordinates_xz": [
                    [-3.1, 1.55],
                    [-0.31, 1.75],
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
            "benchmark_command": _benchmark_reproduction_command(f"^{BENCHMARK_RUN}$"),
            "capture_commands": [
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 60 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label impact "
                    "--output-dir <impact-capture-dir>"
                ),
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 120 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label outcome "
                    "--output-dir <outcome-capture-dir>"
                ),
            ],
            "image_verdict_commands": [
                (
                    "pixi run image-verdict -- "
                    f"<impact-capture-dir>/{SCENE_ID}_impact.png "
                    f"--meta scene={SCENE_ID} --meta frame=60 "
                    f"--meta view={CAMERA_VIEW} "
                    "--out <impact-capture-dir>/image_verdict.json"
                ),
                (
                    "pixi run image-verdict -- "
                    f"<outcome-capture-dir>/{SCENE_ID}_outcome.png "
                    f"--meta scene={SCENE_ID} --meta frame=120 "
                    f"--meta view={CAMERA_VIEW} "
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
