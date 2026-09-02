#!/usr/bin/env python3
"""Write the validated AVBD row for DART's Figure 13 reconstruction."""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import uuid
from datetime import datetime
from hashlib import sha256
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    PAPER_PACKET_SOURCE_PATHS,
    evidence_definition_matches,
    make_resolved_solver_identity,
    stable_counter_stddev_is_noise,
)
from capture_source_provenance import (  # noqa: E402
    CAPTURE_ARTIFACT_PROVENANCE_ALGORITHM,
    CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM,
    CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
    CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
    CAPTURE_SOURCE_ROOTS,
    DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
    CaptureSourceProvenanceError,
    capture_artifact_provenance,
    compute_capture_source_provenance,
    dart_library_build_identity,
    validate_capture_runtime_provenance,
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
RIGID_AVBD_ALPHA = 0.95
RIGID_AVBD_BETA = 10.0
RIGID_AVBD_GAMMA = 0.99
CAPTURE_VIDEO_FPS = 60
FIGURE13_BENCHMARK_FILTER = (
    "^BM_(Avbd|Vbd|SequentialImpulse)PaperBreakableWallStep/iterations:120$"
)
FIGURE13_BENCHMARK_MIN_WARMUP_SECONDS = 1.0
FIGURE13_BENCHMARK_MIN_QUIET_SECONDS = 120.0
FIGURE13_BENCHMARK_MAX_NORMALIZED_LOAD = 0.25
FIGURE13_BENCHMARK_MAX_SAMPLE_INTERVAL_SECONDS = 1.0
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
    "expected_broken_joint_ids_sha256": (
        "e746389411f654ea64f2836db35c704443b2dac09186fc73d4a9341a18890fab"
    ),
}

EXPECTED_OUTCOMES = {
    60: {
        "broken_joints": 36,
        "evaluated": False,
        "impact_band_displaced_counts": [0, 0, 0],
        "outside_retained_fraction": 1.0,
        "status": "pre-evaluation",
        "threshold_checks": {
            "finite_state": True,
            "fracture_activated": True,
            "fracture_count_bounded": True,
            "fracture_identity_matches": True,
            "fracture_in_three_impact_regions": True,
            "outside_breaks_bounded": True,
            "outside_wall_retained": True,
            "retained_joint_rows_satisfied": True,
            "total_wall_retained": True,
        },
        "thresholds_pass": False,
        "total_retained_fraction": 1.0,
        "unbroken_joints": 676,
    },
    120: {
        "broken_joints": 36,
        "evaluated": True,
        "impact_band_displaced_counts": [0, 0, 0],
        "outside_retained_fraction": 1.0,
        "status": "pass",
        "threshold_checks": {
            "finite_state": True,
            "fracture_activated": True,
            "fracture_count_bounded": True,
            "fracture_identity_matches": True,
            "fracture_in_three_impact_regions": True,
            "outside_breaks_bounded": True,
            "outside_wall_retained": True,
            "retained_joint_rows_satisfied": True,
            "total_wall_retained": True,
        },
        "thresholds_pass": True,
        "total_retained_fraction": 1.0,
        "unbroken_joints": 676,
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


def _validate_long_horizon_outcome(
    outcome: dict[str, Any], *, expected_frame: int = 600
) -> None:
    """Validate a terminal AVBD wall state without pinning transient metrics."""
    _validate_finite_tree(outcome, f"frame {expected_frame} outcome")
    terminal = EXPECTED_OUTCOMES[120]
    for key, expected in (
        ("frame", expected_frame),
        ("evaluated", True),
        ("checkpoint", "outcome"),
        ("status", "pass"),
        ("thresholds_pass", True),
        ("threshold_checks", terminal["threshold_checks"]),
        ("broken_joints", terminal["broken_joints"]),
        ("unbroken_joints", terminal["unbroken_joints"]),
        ("last_step_iterations", RIGID_CONSTRAINT_ITERATIONS),
    ):
        _require_exact(
            outcome.get(key), expected, f"frame {expected_frame} outcome {key}"
        )
    _require_close(
        outcome.get("world_time"),
        expected_frame * TIME_STEP,
        f"frame {expected_frame} outcome world_time",
    )
    _validate_joint_evidence(
        outcome,
        expected_broken_count=terminal["broken_joints"],
        expected_broken_ids_sha256=OUTCOME_ORACLE["expected_broken_joint_ids_sha256"],
        expected_outside_unbroken_count=463,
        label=f"frame {expected_frame} outcome",
    )
    _require_exact(
        outcome.get("broken_joint_impact_region_counts"),
        [5, 5, 5],
        f"frame {expected_frame} broken_joint_impact_region_counts",
    )
    _require_exact(
        outcome.get("broken_joints_outside_impact_regions"),
        21,
        f"frame {expected_frame} broken_joints_outside_impact_regions",
    )
    for key, oracle_key in (
        (
            "maximum_unbroken_joint_linear_residual",
            "maximum_unbroken_joint_linear_residual",
        ),
        (
            "maximum_unbroken_joint_angular_residual_radians",
            "maximum_unbroken_joint_angular_residual_radians",
        ),
    ):
        actual = _finite_number(
            outcome.get(key), f"frame {expected_frame} outcome {key}"
        )
        if actual > OUTCOME_ORACLE[oracle_key]:
            raise AvbdPaperBreakableWallPacketError(
                f"frame {expected_frame} outcome {key} exceeds the AVBD oracle"
            )
    for key, oracle_key in (
        ("outside_retained_fraction", "minimum_outside_retained_fraction"),
        ("total_retained_fraction", "minimum_total_retained_fraction"),
    ):
        actual = _finite_number(
            outcome.get(key), f"frame {expected_frame} outcome {key}"
        )
        if actual < OUTCOME_ORACLE[oracle_key]:
            raise AvbdPaperBreakableWallPacketError(
                f"frame {expected_frame} outcome {key} fails the AVBD oracle"
            )
    region_counts = outcome.get("broken_joint_impact_region_counts")
    minimum = OUTCOME_ORACLE["minimum_broken_joints_per_impact_region"]
    if (
        not isinstance(region_counts, list)
        or len(region_counts) != 3
        or any(
            not isinstance(value, int) or isinstance(value, bool) or value < minimum
            for value in region_counts
        )
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} broken-joint impact-region counts must "
            "satisfy all three outcome minima"
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
            outcome.get("contact_count"), f"frame {expected_frame} contact_count"
        )
        < 0.0
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} contact_count must be non-negative"
        )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--impact-capture-manifest", type=Path, required=True)
    parser.add_argument("--impact-image-verdict-json", type=Path, required=True)
    parser.add_argument("--outcome-capture-manifest", type=Path, required=True)
    parser.add_argument("--outcome-image-verdict-json", type=Path, required=True)
    parser.add_argument("--long-horizon-capture-manifest", type=Path, required=True)
    parser.add_argument("--long-horizon-image-verdict-json", type=Path, required=True)
    parser.add_argument("--visual-review-json", type=Path, required=True)
    parser.add_argument("--paper-pdf", type=Path, required=True)
    parser.add_argument("--paper-figure-image", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _benchmark_reproduction_command(_filter_pattern: str) -> str:
    return (
        "pixi run python scripts/run_figure13_benchmark.py --output "
        "<benchmark-json> -- "
        f"--benchmark_filter='{FIGURE13_BENCHMARK_FILTER}' "
        "--benchmark_repetitions=5 "
        "--benchmark_report_aggregates_only=true "
        "--benchmark_out_format=json "
        f"--benchmark_min_warmup_time={FIGURE13_BENCHMARK_MIN_WARMUP_SECONDS}"
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
    if not _json_values_equal_exact(value, expected):
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must be {expected!r}, got {value!r}"
        )


def _json_values_equal_exact(value: object, expected: object) -> bool:
    if type(value) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(value) == set(expected) and all(
            _json_values_equal_exact(value[key], item) for key, item in expected.items()
        )
    if isinstance(expected, list):
        return len(value) == len(expected) and all(
            _json_values_equal_exact(actual_item, expected_item)
            for actual_item, expected_item in zip(value, expected)
        )
    return value == expected


def _artifact_path(manifest_path: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AvbdPaperBreakableWallPacketError(f"capture manifest missing {label}")
    try:
        value.encode("utf-8")
        recorded_path = Path(value)
    except (TypeError, ValueError, UnicodeError) as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} path must be UTF-8-encodable text"
        ) from exc
    if "\x00" in value:
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} path must not contain NUL"
        )
    if recorded_path.is_absolute():
        candidates = [recorded_path]
    else:
        candidates = [
            manifest_path.parent / recorded_path,
            REPO_ROOT / recorded_path,
        ]

    matches: dict[Path, Path] = {}
    checked: list[Path] = []
    for candidate in candidates:
        try:
            lexical_path = candidate.absolute()
            if lexical_path in checked:
                continue
            checked.append(lexical_path)
            if lexical_path.is_symlink():
                raise AvbdPaperBreakableWallPacketError(
                    f"capture {label} path must not be a symlink: {lexical_path}"
                )
            if not lexical_path.exists():
                continue
            if not lexical_path.is_file():
                raise AvbdPaperBreakableWallPacketError(
                    f"capture {label} path must be a regular file: {lexical_path}"
                )
            resolved_path = lexical_path.resolve(strict=True)
        except AvbdPaperBreakableWallPacketError:
            raise
        except (OSError, ValueError, UnicodeError) as exc:
            raise AvbdPaperBreakableWallPacketError(
                f"cannot inspect capture {label} path {candidate}: {exc}"
            ) from exc
        matches.setdefault(resolved_path, lexical_path)

    if not matches:
        searched = ", ".join(str(path) for path in checked)
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} path does not resolve to a regular file; "
            f"checked: {searched}"
        )
    if len(matches) > 1:
        matched = ", ".join(str(path) for path in matches.values())
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} path is ambiguous; matched: {matched}"
        )
    return next(iter(matches))


def _artifact_directory(manifest_path: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AvbdPaperBreakableWallPacketError(f"capture manifest missing {label}")
    try:
        value.encode("utf-8")
        recorded_path = Path(value)
    except (TypeError, ValueError, UnicodeError) as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} path must be UTF-8-encodable text"
        ) from exc
    if "\x00" in value:
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} path must not contain NUL"
        )
    candidates = (
        [recorded_path]
        if recorded_path.is_absolute()
        else [manifest_path.parent / recorded_path, REPO_ROOT / recorded_path]
    )

    matches: dict[Path, Path] = {}
    checked: list[Path] = []
    for candidate in candidates:
        try:
            lexical_path = candidate.absolute()
            if lexical_path in checked:
                continue
            checked.append(lexical_path)
            if lexical_path.is_symlink():
                raise AvbdPaperBreakableWallPacketError(
                    f"capture {label} path must not be a symlink: {lexical_path}"
                )
            if not lexical_path.exists():
                continue
            if not lexical_path.is_dir():
                raise AvbdPaperBreakableWallPacketError(
                    f"capture {label} path must be a directory: {lexical_path}"
                )
            resolved_path = lexical_path.resolve(strict=True)
        except AvbdPaperBreakableWallPacketError:
            raise
        except (OSError, ValueError, UnicodeError) as exc:
            raise AvbdPaperBreakableWallPacketError(
                f"cannot inspect capture {label} path {candidate}: {exc}"
            ) from exc
        matches.setdefault(resolved_path, lexical_path)

    if not matches:
        searched = ", ".join(str(path) for path in checked)
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} directory does not resolve; checked: {searched}"
        )
    if len(matches) > 1:
        matched = ", ".join(str(path) for path in matches.values())
        raise AvbdPaperBreakableWallPacketError(
            f"capture {label} path is ambiguous; matched: {matched}"
        )
    return next(iter(matches))


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
    expected_frame_count: int,
    expected_width: int,
    expected_height: int,
    frames: Path,
    metrics_events: Path,
    screenshot: Path,
    video: Path,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
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
    if recorded_source.get("working_tree_clean") is not True:
        raise AvbdPaperBreakableWallPacketError(
            "capture source provenance working_tree_clean must be true; a dirty "
            "capture tree cannot be reproduced from git_head"
        )
    if recorded_source.get("ignored_paths") != []:
        raise AvbdPaperBreakableWallPacketError(
            "capture source provenance ignored_paths must be empty; an ignored "
            "file inside a sealed root cannot be reproduced from git_head"
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

    try:
        runtime_provenance = validate_capture_runtime_provenance(
            manifest.get("capture_runtime_provenance"),
            expected_source_digest=recorded_digest,
            expected_source_git_head=recorded_head,
            repo_root=REPO_ROOT,
        )
    except CaptureSourceProvenanceError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot validate capture runtime provenance: {exc}"
        ) from exc
    _require_exact(
        runtime_provenance.get("algorithm"),
        CAPTURE_RUNTIME_PROVENANCE_ALGORITHM,
        "capture runtime provenance algorithm",
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
    _validate_sha256_hex(
        recorded_artifacts.get("digest"), "capture artifact provenance digest"
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
    recorded_png_frames = recorded_artifacts.get("png_frames")
    if not isinstance(recorded_png_frames, dict):
        raise AvbdPaperBreakableWallPacketError(
            "capture artifact provenance missing png_frames"
        )
    _require_exact(
        recorded_png_frames.get("algorithm"),
        CAPTURE_PNG_SEQUENCE_PROVENANCE_ALGORITHM,
        "capture PNG sequence provenance algorithm",
    )
    try:
        frame_paths = sorted(frames.iterdir(), key=lambda path: path.name)
    except (OSError, ValueError, UnicodeError) as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot inspect capture final PNG sequence: {exc}"
        ) from exc
    expected_frame_names = [
        f"frame_{index:06d}.png" for index in range(1, expected_frame_count + 1)
    ]
    if [path.name for path in frame_paths] != expected_frame_names:
        raise AvbdPaperBreakableWallPacketError(
            "capture final PNG sequence must contain exactly "
            f"{expected_frame_names!r}"
        )
    try:
        expected_artifacts = capture_artifact_provenance(
            scene_metrics_events=metrics_events,
            screenshot=screenshot,
            png_frames=frame_paths,
            video=video,
            video_fps=CAPTURE_VIDEO_FPS,
            video_width=expected_width,
            video_height=expected_height,
            screenshot_png_frame_index=expected_frame_count,
        )
    except CaptureSourceProvenanceError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot validate capture artifact provenance: {exc}"
        ) from exc
    _require_exact(
        recorded_artifacts,
        expected_artifacts,
        "capture artifact provenance",
    )
    source_provenance = {
        "algorithm": CAPTURE_SOURCE_PROVENANCE_ALGORITHM,
        "digest": recorded_digest,
        "file_count": recorded_file_count,
        "git_head": recorded_head,
        "ignored_paths": [],
        "roots": list(CAPTURE_SOURCE_ROOTS),
        "working_tree_clean": True,
    }
    return source_provenance, runtime_provenance, expected_artifacts


def _validate_build_configuration(
    value: object, *, context: dict[str, Any]
) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != {
        "algorithm",
        "digest",
        "values",
    }:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark build configuration has an unexpected field set"
        )
    _require_exact(
        value.get("algorithm"),
        BUILD_CONFIGURATION_ALGORITHM,
        "benchmark build configuration algorithm",
    )
    values = value.get("values")
    if not isinstance(values, dict) or set(values) != set(BUILD_CONFIGURATION_KEYS):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark build configuration has an unexpected value key set"
        )
    if any(not isinstance(item, str) for item in values.values()):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark build configuration values must be strings"
        )
    record = "".join(
        [f"algorithm={BUILD_CONFIGURATION_ALGORITHM}\n"]
        + [f"{key}={values[key]}\n" for key in BUILD_CONFIGURATION_KEYS]
    )
    digest = _validate_sha256_hex(
        value.get("digest"), "benchmark build configuration digest"
    )
    _require_exact(
        digest,
        sha256(record.encode("utf-8")).hexdigest(),
        "benchmark build configuration digest",
    )
    for definition in EVIDENCE_CMAKE_DEFINITIONS:
        name, expected = definition.split("=", maxsplit=1)
        if name not in BUILD_CONFIGURATION_KEYS:
            continue
        if not evidence_definition_matches(name, expected, values.get(name)):
            raise AvbdPaperBreakableWallPacketError(
                f"benchmark build configuration {name} must be {expected!r}, "
                f"got {values.get(name)!r}"
            )
    _require_exact(
        values.get("CMAKE_GENERATOR"),
        "Ninja",
        "benchmark build configuration generator",
    )
    for value_key, context_key in (
        ("CMAKE_CXX_COMPILER_ID", "dart_compiler_id"),
        ("CMAKE_CXX_COMPILER_VERSION", "dart_compiler_version"),
        ("CMAKE_BUILD_TYPE", "dart_cmake_build_type"),
    ):
        _require_exact(
            values.get(value_key),
            context.get(context_key),
            f"benchmark build configuration {value_key}",
        )
    _require_exact(
        context.get("dart_build_configuration_digest"),
        digest,
        "benchmark compiled build configuration digest",
    )
    return {
        "algorithm": BUILD_CONFIGURATION_ALGORITHM,
        "digest": digest,
        "values": {key: values[key] for key in BUILD_CONFIGURATION_KEYS},
    }


def _runtime_image_roles(
    images: list[dict[str, Any]],
) -> dict[str, str]:
    matches: dict[str, list[str]] = {role: [] for role in REQUIRED_RUNTIME_IMAGE_ROLES}
    for image in images:
        name = image["file"].lower()
        path = image["path"]
        if name.startswith(("ld-linux", "ld-musl")) or name == "ld.so":
            matches["dynamic_loader"].append(path)
        if name == "libbenchmark.so" or name.startswith("libbenchmark.so."):
            matches["google_benchmark"].append(path)
        if name == "libc.so" or name.startswith("libc.so."):
            matches["libc"].append(path)
        if name == "libm.so" or name.startswith("libm.so."):
            matches["libm"].append(path)
        if name == "libstdc++.so" or name.startswith("libstdc++.so."):
            matches["libstdcxx"].append(path)
    missing = [role for role, paths in matches.items() if not paths]
    if missing:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark runtime-image inventory is missing required roles: "
            f"{missing!r}"
        )
    ambiguous = {role: paths for role, paths in matches.items() if len(paths) != 1}
    if ambiguous:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark runtime-image inventory has ambiguous required roles: "
            f"{ambiguous!r}"
        )
    return {role: matches[role][0] for role in REQUIRED_RUNTIME_IMAGE_ROLES}


def _validate_runtime_image_inventory(
    value: object, *, executable: Path
) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != {
        "algorithm",
        "digest",
        "images",
        "required_roles",
    }:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark runtime-image inventory has an unexpected field set"
        )
    _require_exact(
        value.get("algorithm"),
        RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        "benchmark runtime-image inventory algorithm",
    )
    images = value.get("images")
    if not isinstance(images, list) or not images:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark runtime-image inventory must contain mapped ELF images"
        )
    normalized_images: list[dict[str, Any]] = []
    for index, image in enumerate(images):
        label = f"benchmark runtime image {index}"
        if not isinstance(image, dict) or set(image) != {
            "file",
            "path",
            "sha256",
            "size_bytes",
        }:
            raise AvbdPaperBreakableWallPacketError(
                f"{label} has an unexpected field set"
            )
        path_value = image.get("path")
        if not isinstance(path_value, str) or not Path(path_value).is_absolute():
            raise AvbdPaperBreakableWallPacketError(f"{label} path must be absolute")
        candidate = Path(path_value)
        try:
            path = candidate.resolve(strict=True)
        except (OSError, RuntimeError, ValueError) as exc:
            raise AvbdPaperBreakableWallPacketError(
                f"cannot resolve {label}: {exc}"
            ) from exc
        if candidate.is_symlink() or candidate != path or not path.is_file():
            raise AvbdPaperBreakableWallPacketError(
                f"{label} must be canonical, regular, and non-symlink"
            )
        try:
            with path.open("rb") as file:
                if file.read(4) != b"\x7fELF":
                    raise AvbdPaperBreakableWallPacketError(
                        f"{label} must be an ELF image"
                    )
        except OSError as exc:
            raise AvbdPaperBreakableWallPacketError(
                f"cannot read {label}: {exc}"
            ) from exc
        normalized = {
            "file": path.name,
            "path": str(path),
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
        if not 1 <= normalized["size_bytes"] <= 0xFFFFFFFFFFFFFFFF:
            raise AvbdPaperBreakableWallPacketError(
                f"{label} size must be a positive uint64"
            )
        _require_exact(image, normalized, label)
        normalized_images.append(normalized)
    paths = [image["path"] for image in normalized_images]
    if paths != sorted(set(paths)):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark runtime images must be unique and path-sorted"
        )
    if paths.count(str(executable)) != 1:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark runtime images must contain the exact executable"
        )
    roles = _runtime_image_roles(normalized_images)
    _require_exact(
        value.get("required_roles"),
        roles,
        "benchmark runtime-image required-role binding",
    )
    payload = {"images": normalized_images, "required_roles": roles}
    _require_exact(
        _validate_sha256_hex(
            value.get("digest"), "benchmark runtime-image inventory digest"
        ),
        _canonical_json_digest(payload),
        "benchmark runtime-image inventory digest",
    )
    return {
        "algorithm": RUNTIME_IMAGE_INVENTORY_ALGORITHM,
        **payload,
        "digest": _canonical_json_digest(payload),
    }


def _validate_benchmark_source_provenance(
    context: dict[str, Any],
    *,
    loaded_dart_libraries: object,
    runtime_image_inventory: object,
    build_configuration: object,
) -> dict[str, Any]:
    try:
        current_source = compute_capture_source_provenance(REPO_ROOT)
    except CaptureSourceProvenanceError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot validate benchmark source provenance: {exc}"
        ) from exc
    capture_digest = _validate_sha256_hex(
        context.get("dart_capture_source_provenance_digest"),
        "benchmark compiled capture source provenance digest",
    )
    _require_exact(
        capture_digest,
        current_source["digest"],
        "benchmark compiled capture source provenance digest",
    )
    benchmark_source_hash = _validate_sha256_hex(
        context.get("dart_benchmark_source_sha256"),
        "benchmark compiled benchmark source sha256",
    )
    _require_exact(
        benchmark_source_hash,
        _sha256(REPO_ROOT / BENCHMARK_SOURCE_PATH),
        "benchmark compiled benchmark source sha256",
    )
    capture_git_head = context.get("dart_capture_source_git_head")
    if (
        not isinstance(capture_git_head, str)
        or len(capture_git_head) != 40
        or any(character not in "0123456789abcdef" for character in capture_git_head)
    ):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark compiled capture source Git HEAD must be a lowercase "
            "object ID"
        )
    build_type = context.get("dart_cmake_build_type")
    _require_exact(build_type, "Release", "benchmark compiled CMake build type")
    compiler_id = context.get("dart_compiler_id")
    compiler_version = context.get("dart_compiler_version")
    if not isinstance(compiler_id, str) or not compiler_id.strip():
        raise AvbdPaperBreakableWallPacketError(
            "benchmark compiled compiler ID must be non-empty"
        )
    if not isinstance(compiler_version, str) or not compiler_version.strip():
        raise AvbdPaperBreakableWallPacketError(
            "benchmark compiled compiler version must be non-empty"
        )
    _require_exact(context.get("dart_ndebug"), "1", "benchmark compiled NDEBUG")
    _require_exact(
        context.get("dart_optimization_enabled"),
        "1",
        "benchmark compiled optimization state",
    )
    normalized_build_configuration = _validate_build_configuration(
        build_configuration, context=context
    )
    build_configuration_digest = normalized_build_configuration["digest"]
    executable_value = context.get("dart_benchmark_executable_path")
    if not isinstance(executable_value, str) or not executable_value:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark compiled context is missing actual executable path"
        )
    try:
        executable = Path(executable_value).resolve(strict=True)
    except (OSError, RuntimeError, ValueError) as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot resolve benchmark executable: {exc}"
        ) from exc
    if executable.is_symlink() or not executable.is_file():
        raise AvbdPaperBreakableWallPacketError(
            "benchmark executable must be a regular non-symlink file"
        )
    if executable.name != "bm_avbd_rigid_fixed_joint":
        raise AvbdPaperBreakableWallPacketError(
            "benchmark compiled context identifies the wrong executable"
        )
    executable_size = executable.stat().st_size
    if executable_size < 1 or executable_size > 0xFFFFFFFFFFFFFFFF:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark executable size must be a positive uint64"
        )
    normalized_runtime_inventory = _validate_runtime_image_inventory(
        runtime_image_inventory, executable=executable
    )
    runtime_images_by_path = {
        image["path"]: image for image in normalized_runtime_inventory["images"]
    }
    if not isinstance(loaded_dart_libraries, list) or not loaded_dart_libraries:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark evidence must inventory loaded DART shared libraries"
        )
    normalized_libraries: list[dict[str, Any]] = []
    for index, entry in enumerate(loaded_dart_libraries):
        label = f"benchmark loaded DART library {index}"
        if not isinstance(entry, dict) or set(entry) != {
            "build_identity",
            "file",
            "path",
            "sha256",
            "size_bytes",
        }:
            raise AvbdPaperBreakableWallPacketError(
                f"{label} has an unexpected field set"
            )
        path_value = entry.get("path")
        if not isinstance(path_value, str) or not Path(path_value).is_absolute():
            raise AvbdPaperBreakableWallPacketError(f"{label} path must be absolute")
        try:
            path = Path(path_value).resolve(strict=True)
        except (OSError, RuntimeError, ValueError) as exc:
            raise AvbdPaperBreakableWallPacketError(
                f"cannot resolve {label}: {exc}"
            ) from exc
        if (
            path.is_symlink()
            or not path.is_file()
            or not path.name.lower().startswith("libdart")
        ):
            raise AvbdPaperBreakableWallPacketError(
                f"{label} must be a regular non-symlink libdart image"
            )
        normalized = {
            "build_identity": dart_library_build_identity(
                path,
                expected_source_digest=capture_digest,
                expected_source_git_head=capture_git_head,
            ),
            "file": path.name,
            "path": str(path),
            "sha256": _sha256(path),
            "size_bytes": path.stat().st_size,
        }
        _require_exact(entry, normalized, label)
        runtime_image = runtime_images_by_path.get(str(path))
        if runtime_image != {
            key: normalized[key] for key in ("file", "path", "sha256", "size_bytes")
        }:
            raise AvbdPaperBreakableWallPacketError(
                f"{label} must match the complete runtime-image inventory"
            )
        library_identity = normalized["build_identity"]
        _require_exact(
            library_identity["algorithm"],
            DART_LIBRARY_BUILD_IDENTITY_ALGORITHM,
            f"{label} build identity algorithm",
        )
        for key, expected in {
            "build_configuration_digest": build_configuration_digest,
            "cmake_build_type": build_type,
            "compiler_id": compiler_id,
            "compiler_version": compiler_version,
            "ndebug": True,
            "optimization_enabled": True,
        }.items():
            _require_exact(
                library_identity[key], expected, f"{label} build identity {key}"
            )
        normalized_libraries.append(normalized)
    library_paths = [entry["path"] for entry in normalized_libraries]
    if library_paths != sorted(set(library_paths)):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark loaded DART libraries must be unique and path-sorted"
        )
    runtime_dart_paths = sorted(
        image["path"]
        for image in normalized_runtime_inventory["images"]
        if image["file"].lower().startswith("libdart")
    )
    if library_paths != runtime_dart_paths:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark loaded DART libraries must exactly cover every mapped "
            "libdart runtime image"
        )
    executable_payload = {
        "executable_file": executable.name,
        "executable_path": str(executable),
        "executable_sha256": _sha256(executable),
        "executable_size_bytes": executable_size,
        "loaded_dart_libraries": normalized_libraries,
        "runtime_image_inventory": normalized_runtime_inventory,
    }
    build_payload = {
        "benchmark_source_sha256": benchmark_source_hash,
        "build_configuration": normalized_build_configuration,
        "capture_source_git_head": capture_git_head,
        "capture_source_provenance_digest": capture_digest,
        "cmake_build_type": build_type,
        "compiler_id": compiler_id,
        "compiler_version": compiler_version,
        **executable_payload,
        "ndebug": "1",
        "optimization_enabled": "1",
    }
    build_identity = {
        "algorithm": BENCHMARK_BUILD_IDENTITY_ALGORITHM,
        **build_payload,
        "digest": _canonical_json_digest(build_payload),
    }
    source_payload = {
        "benchmark_source_sha256": benchmark_source_hash,
        "build_identity": build_identity,
        "capture_source_git_head": capture_git_head,
        "capture_source_provenance_digest": capture_digest,
        "executable": {
            "file": executable.name,
            "path": str(executable),
            "sha256": executable_payload["executable_sha256"],
            "size_bytes": executable_size,
        },
        "loaded_dart_libraries": normalized_libraries,
        "runtime_image_inventory": normalized_runtime_inventory,
    }
    return {
        "algorithm": BENCHMARK_SOURCE_PROVENANCE_ALGORITHM,
        **source_payload,
        "digest": _canonical_json_digest(source_payload),
    }


def _canonical_json_digest(value: object) -> str:
    return sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()


def _require_iso_timestamp(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.endswith("Z"):
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must be a UTC ISO-8601 timestamp"
        )
    try:
        datetime.fromisoformat(value.removesuffix("Z") + "+00:00")
    except ValueError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} must be a UTC ISO-8601 timestamp"
        ) from exc
    return value


def _validate_load_gate(
    value: object,
    *,
    label: str,
    minimum_duration_seconds: float | None,
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AvbdPaperBreakableWallPacketError(f"{label} must be an object")
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
        raise AvbdPaperBreakableWallPacketError(f"{label} has an unexpected field set")
    _require_exact(value.get("passed"), True, f"{label} passed")
    _require_iso_timestamp(value.get("started_at"), f"{label} started_at")
    _require_iso_timestamp(value.get("finished_at"), f"{label} finished_at")
    interval = _finite_number(
        value.get("sample_interval_seconds"), f"{label} sample interval"
    )
    if not 0.0 < interval <= FIGURE13_BENCHMARK_MAX_SAMPLE_INTERVAL_SECONDS:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} sample interval must be in (0, 1] seconds"
        )
    limit = _finite_number(value.get("normalized_load_limit"), f"{label} load limit")
    if not 0.0 < limit <= FIGURE13_BENCHMARK_MAX_NORMALIZED_LOAD:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} normalized load limit is too permissive"
        )
    maximum = _finite_number(value.get("max_normalized_load"), f"{label} maximum load")
    if not 0.0 <= maximum <= limit:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} maximum normalized load exceeds its limit"
        )
    elapsed = _finite_number(value.get("elapsed_seconds"), f"{label} elapsed")
    if elapsed <= 0.0:
        raise AvbdPaperBreakableWallPacketError(
            f"{label} elapsed time must be positive"
        )
    sample_count = value.get("sample_count")
    if (
        not isinstance(sample_count, int)
        or isinstance(sample_count, bool)
        or sample_count < 2
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"{label} sample_count must be an integer of at least two"
        )
    if minimum_duration_seconds is not None:
        duration = _finite_number(value.get("duration_seconds"), f"{label} duration")
        if duration < minimum_duration_seconds or elapsed < duration:
            raise AvbdPaperBreakableWallPacketError(
                f"{label} must cover at least {minimum_duration_seconds:g} seconds"
            )
        if sample_count < math.floor(duration / interval):
            raise AvbdPaperBreakableWallPacketError(
                f"{label} sample_count cannot cover its declared duration"
            )
    return dict(value)


def _validate_benchmark_run_evidence(
    data: dict[str, Any],
    *,
    context: dict[str, Any],
    source_provenance: dict[str, Any],
) -> dict[str, Any]:
    evidence = data.get("dart_evidence_run")
    if not isinstance(evidence, dict):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark JSON missing dart_evidence_run wrapper attestation"
        )
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
        raise AvbdPaperBreakableWallPacketError(
            "benchmark dart_evidence_run has an unexpected field set"
        )
    _require_exact(
        evidence.get("schema_version"),
        FIGURE13_BENCHMARK_RUN_SCHEMA,
        "benchmark evidence schema_version",
    )
    digest = _validate_sha256_hex(evidence.get("digest"), "benchmark evidence digest")
    run_payload = {
        key: evidence[key]
        for key in sorted(expected_keys - {"digest", "schema_version"})
    }
    _require_exact(
        digest,
        _canonical_json_digest(run_payload),
        "benchmark evidence digest",
    )
    policy = evidence.get("benchmark_policy")
    expected_policy = {
        "filter": FIGURE13_BENCHMARK_FILTER,
        "min_warmup_time_seconds": FIGURE13_BENCHMARK_MIN_WARMUP_SECONDS,
        "repetitions": 5,
        "report_aggregates_only": True,
    }
    _require_exact(policy, expected_policy, "benchmark evidence policy")
    # The recorded Git HEAD only describes the evidence if the capture roots
    # were clean and no ignored file inside them escaped the digest.
    _require_exact(
        evidence.get("capture_working_tree_clean"),
        True,
        "benchmark evidence capture_working_tree_clean",
    )
    _require_exact(
        evidence.get("capture_ignored_paths"),
        [],
        "benchmark evidence capture_ignored_paths",
    )
    context_date = context.get("date")
    if not isinstance(context_date, str) or not context_date:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark context date must be non-empty"
        )
    _require_exact(
        evidence.get("benchmark_context_date"),
        context_date,
        "benchmark evidence context date",
    )
    _require_exact(
        evidence.get("build_identity"),
        source_provenance["build_identity"],
        "benchmark evidence compiled build identity",
    )
    expected_loader_environment = {
        "algorithm": LOADER_POLICY_ALGORITHM,
        "forbidden_environment_prefixes": list(LOADER_ENVIRONMENT_PREFIXES),
        "passed": True,
        "present_environment_variables": [],
    }
    _require_exact(
        evidence.get("loader_environment"),
        expected_loader_environment,
        "benchmark evidence loader environment",
    )
    host = evidence.get("host_identity")
    if not isinstance(host, dict):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark evidence host_identity must be an object"
        )
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
        raise AvbdPaperBreakableWallPacketError(
            "benchmark evidence host_identity has an unexpected field set"
        )
    for key in ("cpu_model", "hostname", "machine", "platform", "system"):
        if not isinstance(host.get(key), str) or not host[key]:
            raise AvbdPaperBreakableWallPacketError(
                f"benchmark evidence host_identity.{key} must be non-empty"
            )
    cpu_count = host.get("cpu_count")
    if not isinstance(cpu_count, int) or isinstance(cpu_count, bool) or cpu_count < 1:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark evidence host cpu_count must be positive"
        )
    host_payload = {key: host[key] for key in sorted(host_keys - {"host_token"})}
    expected_host_token = _canonical_json_digest(host_payload)
    _require_exact(
        host.get("host_token"), expected_host_token, "benchmark evidence host token"
    )
    _require_exact(
        evidence.get("host_token"), expected_host_token, "benchmark evidence host token"
    )
    _require_exact(
        context.get("host_name"), host.get("hostname"), "benchmark evidence hostname"
    )
    run_token = evidence.get("run_token")
    try:
        parsed_run_token = uuid.UUID(run_token, version=4)
    except (AttributeError, TypeError, ValueError) as exc:
        raise AvbdPaperBreakableWallPacketError(
            "benchmark evidence run_token must be a canonical UUIDv4"
        ) from exc
    _require_exact(str(parsed_run_token), run_token, "benchmark evidence run_token")
    quiet = _validate_load_gate(
        evidence.get("quiet_host"),
        label="benchmark quiet-host gate",
        minimum_duration_seconds=FIGURE13_BENCHMARK_MIN_QUIET_SECONDS,
    )
    watchdog = _validate_load_gate(
        evidence.get("watchdog"),
        label="benchmark in-run watchdog",
        minimum_duration_seconds=None,
    )
    _require_exact(
        watchdog["normalized_load_limit"],
        quiet["normalized_load_limit"],
        "benchmark watchdog load limit",
    )
    _require_exact(
        watchdog["sample_interval_seconds"],
        quiet["sample_interval_seconds"],
        "benchmark watchdog sample interval",
    )
    return dict(evidence)


def _require_shared_capture_benchmark_build(
    benchmark: dict[str, Any],
    *captures: dict[str, Any],
) -> None:
    benchmark_source = benchmark.get("source_provenance")
    if not isinstance(benchmark_source, dict):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark source provenance is missing"
        )
    expected = (
        benchmark_source.get("capture_source_provenance_digest"),
        benchmark_source.get("capture_source_git_head"),
    )
    for index, capture in enumerate(captures):
        capture_source = capture.get("source_provenance")
        if not isinstance(capture_source, dict):
            raise AvbdPaperBreakableWallPacketError(
                f"capture {index} source provenance is missing"
            )
        actual = (capture_source.get("digest"), capture_source.get("git_head"))
        _require_exact(
            actual,
            expected,
            "capture and benchmark compiled source/build identity",
        )


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
    height: int,
    width: int,
    expected_focus: tuple[str, ...] = VIEW_FOCUS,
    expected_scene_id: str = SCENE_ID,
    assessed_frames: tuple[int, ...] | None = None,
) -> list[dict[str, Any]]:
    # The scene runs the engine view assessment only at its capture
    # checkpoint frames; every other event records `view_report: null`.
    if assessed_frames is None:
        assessed_frames = tuple(
            int(frame) for frame in OUTCOME_ORACLE["joint_evidence_frames"]
        )
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except FileNotFoundError as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"{path}: scene metrics event log not found"
        ) from exc

    events = []
    invariant_metrics: dict[str, Any] | None = None
    invariant_keys = (
        "ball_count",
        "break_force",
        "breakable_joints",
        "brick_count",
        "collision_shapes",
        "executor",
        "outcome_oracle",
        "paper_locator",
        "resolved_configuration",
        "rigid_bodies",
        "rigid_body_solver",
        "rigid_constraint_options",
        "row",
        "scene_spec_fingerprint",
        "solver",
        "time_step_ms",
    )
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
        _require_exact(
            metrics.get("effective_scene_contract_passed"),
            True,
            f"{path}:{line_number} effective scene contract",
        )
        if metrics.get("view_report") is not None or event.get("frame") in set(
            assessed_frames
        ):
            _validate_view_report(
                metrics,
                expected_focus=expected_focus,
                width=width,
                height=height,
            )
        current_invariants = {key: metrics.get(key) for key in invariant_keys}
        if invariant_metrics is None:
            invariant_metrics = current_invariants
        else:
            _require_exact(
                current_invariants,
                invariant_metrics,
                f"{path}:{line_number} full-stream scene/solver/oracle invariants",
            )
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
        _require_exact(
            outcome.get("last_step_iterations"),
            RIGID_CONSTRAINT_ITERATIONS,
            f"{path}:{line_number} outcome solver iterations",
        )
        evaluated = outcome.get("evaluated")
        thresholds_pass = outcome.get("thresholds_pass")
        status = outcome.get("status")
        if not isinstance(evaluated, bool) or not isinstance(thresholds_pass, bool):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: evaluated and thresholds_pass must be booleans"
            )
        if not isinstance(status, str) or not status:
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: outcome status must be non-empty"
            )
        oracle = metrics.get("outcome_oracle")
        evaluation_frame = (
            oracle.get("evaluation_frame") if isinstance(oracle, dict) else None
        )
        if (
            not isinstance(evaluation_frame, int)
            or isinstance(evaluation_frame, bool)
            or evaluation_frame < 1
        ):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: outcome oracle evaluation_frame is invalid"
            )
        secondary_frame = None
        if isinstance(oracle, dict):
            for key in ("retention_evaluation_frame", "collapse_evaluation_frame"):
                candidate = oracle.get(key)
                if candidate is not None:
                    if (
                        not isinstance(candidate, int)
                        or isinstance(candidate, bool)
                        or candidate <= evaluation_frame
                    ):
                        raise AvbdPaperBreakableWallPacketError(
                            f"{path}:{line_number}: outcome oracle {key} is invalid"
                        )
                    secondary_frame = candidate
        if event_frame < evaluation_frame:
            _require_exact(
                (evaluated, thresholds_pass, status),
                (False, False, "pre-evaluation"),
                f"{path}:{line_number} pre-evaluation outcome state",
            )
        elif (
            secondary_frame is not None
            and event_frame != evaluation_frame
            and event_frame < secondary_frame
        ):
            _require_exact(
                (evaluated, thresholds_pass, status),
                (False, False, "between-checkpoints"),
                f"{path}:{line_number} between-checkpoint outcome state",
            )
        else:
            _require_exact(
                (evaluated, thresholds_pass, status),
                (True, True, "pass"),
                f"{path}:{line_number} evaluated outcome state",
            )
        threshold_checks = outcome.get("threshold_checks")
        if not isinstance(threshold_checks, dict) or not threshold_checks:
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: threshold_checks must be a non-empty object"
            )
        if any(not isinstance(value, bool) for value in threshold_checks.values()):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: threshold_checks values must be booleans"
            )
        if thresholds_pass and not all(threshold_checks.values()):
            raise AvbdPaperBreakableWallPacketError(
                f"{path}:{line_number}: passing outcome has a false threshold check"
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


def _scene_metric_prefix_digest(events: list[dict[str, Any]], frame_count: int) -> str:
    if not 1 <= frame_count <= len(events):
        raise AvbdPaperBreakableWallPacketError(
            "scene metric prefix frame count is out of range"
        )
    return _canonical_json_digest(events[:frame_count])


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
        "effective_scene_contract_passed": True,
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
    if expected_frame == 600:
        _validate_long_horizon_outcome(outcome, expected_frame=expected_frame)
    expected_outcome = EXPECTED_OUTCOMES.get(expected_frame)
    terminal_outcome = EXPECTED_OUTCOMES[120]
    expected_common = expected_outcome or {
        "broken_joints": terminal_outcome["broken_joints"],
        "evaluated": True,
        "status": "pass",
        "threshold_checks": terminal_outcome["threshold_checks"],
        "thresholds_pass": True,
        "unbroken_joints": terminal_outcome["unbroken_joints"],
    }
    for key in (
        "broken_joints",
        "evaluated",
        "status",
        "thresholds_pass",
        "unbroken_joints",
    ):
        _require_exact(
            outcome.get(key),
            expected_common[key],
            f"frame {expected_frame} outcome {key}",
        )
    if expected_outcome is not None:
        _require_exact(
            outcome.get("impact_band_displaced_counts"),
            expected_outcome["impact_band_displaced_counts"],
            f"frame {expected_frame} outcome impact_band_displaced_counts",
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
    _require_exact(
        outcome.get("checkpoint"),
        "outcome",
        f"frame {expected_frame} outcome checkpoint",
    )
    _require_close(
        outcome.get("world_time"),
        expected_frame * TIME_STEP,
        f"frame {expected_frame} outcome world_time",
    )
    threshold_checks = outcome.get("threshold_checks")
    if not isinstance(threshold_checks, dict):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} outcome missing threshold_checks"
        )
    _require_exact(
        threshold_checks,
        expected_common["threshold_checks"],
        f"frame {expected_frame} outcome threshold_checks",
    )
    _validate_joint_evidence(
        outcome,
        expected_broken_count=expected_common["broken_joints"],
        expected_broken_ids_sha256=OUTCOME_ORACLE["expected_broken_joint_ids_sha256"],
        expected_outside_unbroken_count=463,
        label=f"frame {expected_frame} outcome",
    )
    _require_exact(
        outcome.get("broken_joint_impact_region_counts"),
        [5, 5, 5],
        f"frame {expected_frame} broken_joint_impact_region_counts",
    )
    _require_exact(
        outcome.get("broken_joints_outside_impact_regions"),
        21,
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
    for key, oracle_key in (
        ("outside_retained_fraction", "minimum_outside_retained_fraction"),
        ("total_retained_fraction", "minimum_total_retained_fraction"),
    ):
        actual = _finite_number(
            outcome.get(key), f"frame {expected_frame} outcome {key}"
        )
        if actual < OUTCOME_ORACLE[oracle_key]:
            raise AvbdPaperBreakableWallPacketError(
                f"frame {expected_frame} outcome {key} must be >= "
                f"{OUTCOME_ORACLE[oracle_key]}, got {actual}"
            )
    region_counts = outcome.get("broken_joint_impact_region_counts")
    minimum_region_breaks = OUTCOME_ORACLE["minimum_broken_joints_per_impact_region"]
    if (
        not isinstance(region_counts, list)
        or len(region_counts) != 3
        or any(
            not isinstance(value, int)
            or isinstance(value, bool)
            or value < minimum_region_breaks
            for value in region_counts
        )
    ):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} broken-joint impact-region counts must "
            "satisfy all three outcome minima"
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
    contact_count = _finite_number(
        outcome.get("contact_count"), f"frame {expected_frame} contact_count"
    )
    if contact_count < 0.0 or (expected_frame != 600 and contact_count == 0.0):
        raise AvbdPaperBreakableWallPacketError(
            f"frame {expected_frame} contact_count must be non-negative and "
            "positive at the paper checkpoints"
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
                "effective_scene_contract_passed",
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
) -> tuple[dict[str, Any], Path, Path]:
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
    frames = _artifact_directory(
        manifest_path,
        artifacts.get("frames"),
        "artifacts.frames",
    )
    video = _artifact_path(
        manifest_path,
        artifacts.get("video"),
        "artifacts.video",
    )
    _require_exact(
        video.name,
        f"{SCENE_ID}_{expected_label}.mp4",
        "capture video filename",
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
        height=height,
        width=width,
    )
    for checkpoint_frame in sorted(EXPECTED_OUTCOMES):
        if checkpoint_frame > expected_frame:
            continue
        checkpoint_event = metric_events[checkpoint_frame - 1]
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
    if expected_frame == 600:
        checkpoint_event = metric_events[599]
        _validate_scene_metrics(
            {
                "scene_metrics": {
                    "event_count": 600,
                    "latest": checkpoint_event,
                }
            },
            expected_frame=600,
            height=height,
            logged_latest=checkpoint_event,
            width=width,
        )
    scene_metrics = _validate_scene_metrics(
        manifest,
        expected_frame=expected_frame,
        height=height,
        logged_latest=metric_events[-1],
        width=width,
    )
    (
        capture_source_provenance,
        capture_runtime_provenance,
        capture_artifacts,
    ) = _validate_capture_provenance(
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
            "camera": _validate_camera(manifest),
            "capture": {
                "converted_frames": expected_frame,
                "height": height,
                "requested_frames": expected_frame,
                "width": width,
            },
            "artifact_provenance": capture_artifacts,
            "label": expected_label,
            "manifest": {
                "file": manifest_path.name,
                "sha256": _sha256(manifest_path),
            },
            "scene_metrics": scene_metrics,
            "source_provenance": capture_source_provenance,
            "runtime_provenance": capture_runtime_provenance,
            "scene_metrics_events": {
                "event_count": len(metric_events),
                "file": metrics_events.name,
                "prefix_sha256": {
                    str(frame): _scene_metric_prefix_digest(metric_events, frame)
                    for frame in sorted({60, 120, 600})
                    if frame <= expected_frame
                },
                "sha256": _sha256(metrics_events),
            },
            "screenshot": {
                "file": screenshot.name,
                "sha256": _sha256(screenshot),
            },
        },
        screenshot,
        video,
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
    screenshot_sha256 = _sha256(screenshot)
    _require_exact(
        _validate_sha256_hex(
            image.get("sha256"),
            "image verdict image sha256",
        ),
        screenshot_sha256,
        "image verdict image sha256",
    )
    return {
        "checks": checks,
        "file": verdict_path.name,
        "image_sha256": screenshot_sha256,
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


def _require_zero_stddev_counters(
    row: dict[str, Any],
    keys: tuple[str, ...],
    benchmark_run: str,
    *,
    reference_row: dict[str, Any],
) -> None:
    """Prove that repetition-invariant benchmark counters did not drift.

    Boolean and small counters must show an exact zero spread; 32-bit
    fingerprint words are compared against the double-precision aggregation
    noise bound documented on `stable_counter_stddev_is_noise`.
    """
    for key in keys:
        value = _finite_number(row.get(key), f"{benchmark_run} stddev {key}")
        reference = _finite_number(
            reference_row.get(key), f"{benchmark_run} median {key}"
        )
        if not stable_counter_stddev_is_noise(key, value, reference):
            raise AvbdPaperBreakableWallPacketError(
                f"{benchmark_run}: stddev {key} must be 0 (fingerprint words: "
                "within double-precision aggregation noise) to prove identical "
                "configuration across benchmark repetitions"
            )


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
        "contact_method_sequential_impulse": 1,
        "effective_scene_contract_passed": 1,
        "effective_scene_mutation_audit_passed": 1,
        "impacting_balls": IMPACTING_BALLS,
        "public_avbd_family": 1,
        "resolved_rigid_body_avbd": 1,
        "resolved_rigid_constraint_iterations": 1,
        "resolved_rigid_contact_avbd": 1,
        "resolved_rigid_pair_constraint_avbd": 1,
        "rigid_constraint_iterations": RIGID_CONSTRAINT_ITERATIONS,
        "rigid_bodies": RIGID_BODIES,
        "rigid_body_joints": BREAKABLE_JOINTS,
        "rigid_avbd_alpha": RIGID_AVBD_ALPHA,
        "rigid_avbd_beta": RIGID_AVBD_BETA,
        "rigid_avbd_gamma": RIGID_AVBD_GAMMA,
        "rigid_avbd_parameter_profile_paper_2025": 1,
        "runtime_contract_passed": 1,
        "runtime_identity_recorded": 1,
        "runtime_identity_applicable": 1,
        "runtime_identity_not_applicable": 0,
        "runtime_identity_public_avbd_rigid": 1,
        "runtime_identity_variational_multibody": 0,
        "scene_spec_matches_python": 1,
        "runtime_identity_contract_passed": 1,
        "solver_projection_policies_match": 1,
        "trajectory_frames": 120,
    }
    expected_fingerprint_value = int(expected_scene_spec_fingerprint, 16)
    representative_rows = (by_aggregate["mean"], by_aggregate["median"])
    configuration_words: list[int] = []
    for key in (
        "solver_configuration_fingerprint_hi",
        "solver_configuration_fingerprint_lo",
    ):
        values = []
        for row in representative_rows:
            value = _finite_number(row.get(key), f"{BENCHMARK_RUN} {key}")
            if value < 0.0 or value > 0xFFFFFFFF or value != math.floor(value):
                raise AvbdPaperBreakableWallPacketError(
                    f"{BENCHMARK_RUN} {key} must be an unsigned 32-bit integer"
                )
            values.append(int(value))
        if values[0] != values[1]:
            raise AvbdPaperBreakableWallPacketError(
                f"{BENCHMARK_RUN} mean/median {key} counters must match"
            )
        configuration_words.append(values[0])
    solver_configuration_fingerprint = (
        f"{configuration_words[0]:08x}{configuration_words[1]:08x}"
    )
    fingerprint_counters = {
        "scene_spec_fingerprint_hi": expected_fingerprint_value >> 32,
        "scene_spec_fingerprint_lo": expected_fingerprint_value & 0xFFFFFFFF,
        "solver_configuration_fingerprint_hi": configuration_words[0],
        "solver_configuration_fingerprint_lo": configuration_words[1],
    }
    stable_counter_keys = tuple((*counters, *fingerprint_counters))
    _require_zero_stddev_counters(
        by_aggregate["stddev"],
        stable_counter_keys,
        BENCHMARK_RUN,
        reference_row=by_aggregate["median"],
    )
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
                {key: row[key] for key in stable_counter_keys}
                if aggregate in ("mean", "median", "stddev")
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
    raw_evidence = data.get("dart_evidence_run")
    if not isinstance(raw_evidence, dict):
        raise AvbdPaperBreakableWallPacketError(
            "benchmark JSON missing dart_evidence_run wrapper attestation"
        )
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
    benchmark_source_provenance = _validate_benchmark_source_provenance(
        context,
        loaded_dart_libraries=loaded_dart_libraries,
        runtime_image_inventory=runtime_image_inventory,
        build_configuration=build_configuration,
    )
    try:
        reported_executable = Path(executable).resolve(strict=True)
    except (OSError, RuntimeError, ValueError) as exc:
        raise AvbdPaperBreakableWallPacketError(
            f"cannot resolve benchmark context executable: {exc}"
        ) from exc
    _require_exact(
        str(reported_executable),
        benchmark_source_provenance["executable"]["path"],
        "benchmark executable identity",
    )
    run_evidence = _validate_benchmark_run_evidence(
        data,
        context=context,
        source_provenance=benchmark_source_provenance,
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
        "source_provenance": benchmark_source_provenance,
        "run_evidence": run_evidence,
        "json_sha256": _sha256(benchmark_path),
        "rows": packet_rows,
        "scene_spec_fingerprint": expected_scene_spec_fingerprint,
        "solver_configuration_fingerprint": (solver_configuration_fingerprint),
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


def _validate_semantic_claim_contract(
    review: dict[str, Any],
    *,
    expected_terminal_behavior: str,
    error_type: type[Exception] = AvbdPaperBreakableWallPacketError,
) -> tuple[dict[str, str], dict[str, Any], dict[str, Any]]:
    expected_claims = SEMANTIC_CLAIM_ASSESSMENTS_BY_TERMINAL_BEHAVIOR[
        expected_terminal_behavior
    ]
    if review.get("claim_assessments") != expected_claims:
        raise error_type(
            "visual review claim_assessments must use the exact authoritative "
            "supported/not_proven contract"
        )
    expected_temporal = {
        "checkpoint_sequence_agrees": True,
        "full_interval_viewed": True,
        "still_frames_only": False,
        "terminal_behavior": expected_terminal_behavior,
    }
    if review.get("temporal_assessment") != expected_temporal:
        raise error_type(
            "visual review temporal_assessment must positively bind the full "
            "video interval and expected terminal behavior"
        )
    expected_observations = SEMANTIC_STRUCTURED_OBSERVATIONS[expected_terminal_behavior]
    if review.get("structured_observations") != expected_observations:
        raise error_type(
            "visual review structured_observations must use the exact "
            "authoritative checkpoint/paper/oracle/ViewReport contract"
        )
    return (
        dict(expected_claims),
        expected_temporal,
        dict(expected_observations),
    )


def _validate_visual_review(
    review_path: Path,
    *,
    impact_screenshot: Path,
    outcome_screenshot: Path,
    long_horizon_screenshot: Path,
    long_horizon_video: dict[str, Any],
    long_horizon_video_path: Path,
    paper_figure: Path,
) -> dict[str, Any]:
    review = _load_json(review_path)
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
        raise AvbdPaperBreakableWallPacketError(
            "visual review must use the exact structured semantic-review fields"
        )
    _require_exact(
        review.get("schema_version"),
        "dart.visual_semantic_review/v1",
        "visual review schema_version",
    )
    _require_exact(review.get("scene"), SCENE_ID, "visual review scene")
    _require_exact(review.get("verdict"), "pass", "visual review verdict")
    capabilities = review.get("reviewer_capabilities")
    expected_capabilities = {
        "image_semantic_review": True,
        "video_semantic_review": True,
    }
    _require_exact(
        capabilities,
        expected_capabilities,
        "visual review reviewer_capabilities",
    )
    assertions = review.get("assessment_assertions")
    expected_assertions = {
        "capture_images_assessed": True,
        "long_horizon_video_assessed": True,
        "no_contradictions_found": True,
        "paper_reference_assessed": True,
        "text_oracle_agrees": True,
        "view_reports_agree": True,
    }
    _require_exact(
        assertions,
        expected_assertions,
        "visual review assessment_assertions",
    )
    (
        claim_assessments,
        temporal_assessment,
        structured_observations,
    ) = _validate_semantic_claim_contract(
        review,
        expected_terminal_behavior="retained_damaged_wall",
    )
    expected = {
        "impact_frame_60": impact_screenshot,
        "outcome_frame_120": outcome_screenshot,
        "long_horizon_frame_600": long_horizon_screenshot,
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
            "visual review must inspect the impact, outcome, long-horizon, "
            "and paper images"
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
    video_entries = review.get("inspected_videos")
    if not isinstance(video_entries, list) or len(video_entries) != 1:
        raise AvbdPaperBreakableWallPacketError(
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
        raise AvbdPaperBreakableWallPacketError(
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
        raise AvbdPaperBreakableWallPacketError(
            "visual review long-horizon video file does not match capture"
        )
    video_entry = dict(video_entry)
    video_entry["file"] = Path(str(video_entry["file"])).name
    _require_exact(
        video_entry,
        expected_video_review,
        "visual review long-horizon video",
    )
    return {
        "assessment_assertions": expected_assertions,
        "claim_assessments": claim_assessments,
        "file": review_path.name,
        "inspected_images": inspected_images,
        "inspected_videos": [expected_video_review],
        "reviewer_capabilities": expected_capabilities,
        "sha256": _sha256(review_path),
        "structured_observations": structured_observations,
        "temporal_assessment": temporal_assessment,
        "verdict": "pass",
    }


def make_packet(
    *,
    benchmark_json: Path,
    impact_capture_manifest: Path,
    impact_image_verdict_json: Path,
    outcome_capture_manifest: Path,
    outcome_image_verdict_json: Path,
    long_horizon_capture_manifest: Path,
    long_horizon_image_verdict_json: Path,
    visual_review_json: Path,
    paper_pdf: Path,
    paper_figure_image: Path,
) -> dict[str, Any]:
    impact_capture, impact_screenshot, _impact_video = _validate_capture(
        impact_capture_manifest,
        expected_frame=60,
        expected_label="impact",
    )
    impact_capture["image_verdict"] = _validate_image_verdict(
        impact_image_verdict_json,
        impact_screenshot,
        expected_frame=60,
    )
    outcome_capture, outcome_screenshot, _outcome_video = _validate_capture(
        outcome_capture_manifest,
        expected_frame=120,
        expected_label="outcome",
    )
    outcome_capture["image_verdict"] = _validate_image_verdict(
        outcome_image_verdict_json,
        outcome_screenshot,
        expected_frame=120,
    )
    (
        long_horizon_capture,
        long_horizon_screenshot,
        long_horizon_video_path,
    ) = _validate_capture(
        long_horizon_capture_manifest,
        expected_frame=600,
        expected_label="long_horizon",
    )
    long_horizon_capture["image_verdict"] = _validate_image_verdict(
        long_horizon_image_verdict_json,
        long_horizon_screenshot,
        expected_frame=600,
    )
    _require_exact(
        outcome_capture["scene_metrics_events"]["prefix_sha256"]["60"],
        impact_capture["scene_metrics_events"]["prefix_sha256"]["60"],
        "impact/outcome exact scene-metric event prefix",
    )
    for frame in (60, 120):
        _require_exact(
            long_horizon_capture["scene_metrics_events"]["prefix_sha256"][str(frame)],
            outcome_capture["scene_metrics_events"]["prefix_sha256"][str(frame)],
            f"outcome/long-horizon exact {frame}-frame scene-metric prefix",
        )
    paper_reference = _validate_paper_artifacts(paper_pdf, paper_figure_image)
    semantic_review = _validate_visual_review(
        visual_review_json,
        impact_screenshot=impact_screenshot,
        outcome_screenshot=outcome_screenshot,
        long_horizon_screenshot=long_horizon_screenshot,
        long_horizon_video=long_horizon_capture["artifact_provenance"]["video"],
        long_horizon_video_path=long_horizon_video_path,
        paper_figure=paper_figure_image,
    )
    scene_spec_fingerprint = impact_capture["scene_metrics"]["scene_spec_fingerprint"]
    _require_exact(
        outcome_capture["scene_metrics"]["scene_spec_fingerprint"],
        scene_spec_fingerprint,
        "impact/outcome scene_spec_fingerprint",
    )
    _require_exact(
        long_horizon_capture["scene_metrics"]["scene_spec_fingerprint"],
        scene_spec_fingerprint,
        "impact/long-horizon scene_spec_fingerprint",
    )
    _require_exact(
        long_horizon_capture["scene_metrics"]["outcome_oracle"],
        impact_capture["scene_metrics"]["outcome_oracle"],
        "impact/long-horizon outcome_oracle",
    )
    benchmark = _validate_benchmark(
        benchmark_json,
        expected_scene_spec_fingerprint=scene_spec_fingerprint,
    )
    _require_shared_capture_benchmark_build(
        benchmark,
        impact_capture,
        outcome_capture,
        long_horizon_capture,
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
            "long_horizon": long_horizon_capture,
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
                    "--capture-label impact --video --fps 60 "
                    "--output-dir <impact-capture-dir>"
                ),
                (
                    "pixi run py-demo-capture -- "
                    f"--scene {SCENE_ID} --frames 120 --width 1280 "
                    "--height 720 --view front --camera-azimuth -112.5 "
                    "--camera-elevation 35.52338329811104 "
                    "--camera-distance 22 --camera-target 0,0.45,1.6 "
                    "--capture-label outcome --video --fps 60 "
                    "--output-dir <outcome-capture-dir>"
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
                (
                    "pixi run image-verdict -- "
                    f"<long-horizon-capture-dir>/{SCENE_ID}_long_horizon.png "
                    f"--meta scene={SCENE_ID} --meta frame=600 "
                    f"--meta view={CAMERA_VIEW} "
                    "--out <long-horizon-capture-dir>/image_verdict.json"
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
            long_horizon_capture_manifest=args.long_horizon_capture_manifest,
            long_horizon_image_verdict_json=(args.long_horizon_image_verdict_json),
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
