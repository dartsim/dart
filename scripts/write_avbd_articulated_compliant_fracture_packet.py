#!/usr/bin/env python3
"""Write a validated AVBD articulated finite-load fracture evidence packet."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from avbd_packet_schema import (  # noqa: E402
    AVBD_PACKET_SCHEMA_VERSION,
    make_resolved_solver_identity,
    make_resolved_solver_identity_from_benchmark_row,
)
from write_avbd_demo3d_static_friction_packet import (  # noqa: E402
    _artifact_label,
    _artifact_path,
    _canonical_name,
    _load_json,
    _png_dimensions,
    _row_name,
    _sha256,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-articulated-compliant-fracture-packet.json"
)
SCENE_ID = "avbd_articulated_compliant_breakable_motor"
BENCHMARK_NAME = "BM_AvbdArticulatedCompliantBreakableMotorStep"
BENCHMARK_ARGS = (1, 4, 16)
EXACT_PARENT_COMMIT = "9ebd9b895b17e982bd1ed9287287d589b029409f"
LOAD_TEST = (
    "VariationalIntegration."
    "AvbdCompliantPublicArticulatedJointAggregatesFiniteAndMotorBreakLoads"
)
LIFECYCLE_TEST = (
    "VariationalIntegration."
    "AvbdCompliantPublicArticulatedJointBreakResetRearmsFiniteRows"
)
SERIALIZATION_TEST = (
    "VariationalIntegration."
    "AvbdCompliantPublicArticulatedFiniteBreakageSurvivesSaveLoadAndReset"
)
TIME_STEP = 0.005
START_STIFFNESS = 100.0
LINEAR_STIFFNESS = 100.0
ANGULAR_STIFFNESS = 100.0
EFFORT_LIMIT = 6.0
WEAK_BREAK_FORCE = 9.0
STRONG_BREAK_FORCE = 100.0
PRESTRAIN = 0.08
RESOLVED_SOLVER_IDENTITY = make_resolved_solver_identity(
    resolved_rigid_contact_family="sequential-impulse",
    rigid_point_joint_solver="sequential_impulse",
    avbd_rigid_contact_config_emplaced=False,
    recorded_from=(
        "articulated compliant-fracture benchmark runtime identity counters"
    ),
    multibody_integration_family="variational",
)


class AvbdArticulatedCompliantFracturePacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-manifest", type=Path, required=True)
    parser.add_argument("--image-verdict-json", type=Path, required=True)
    parser.add_argument("--visual-review-json", type=Path, required=True)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--exact-parent-mutation-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _capture_artifact_path(
    manifest_dir: Path,
    value: object,
    key: str,
) -> Path:
    if isinstance(value, str) and value:
        path = Path(value)
        if path.is_absolute():
            return path
        repo_relative = SCRIPT_DIR.parent / path
        if repo_relative.exists():
            return repo_relative
    return _artifact_path(manifest_dir, value, key)


def _finite_number(value: object, label: str) -> float:
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
    ):
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{label} must be a finite number"
        )
    return float(value)


def _require_close(value: object, expected: float, label: str) -> None:
    actual = _finite_number(value, label)
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12):
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{label} must be {expected:g}, got {actual:g}"
        )


def _validate_scene_metrics(manifest: dict[str, Any]) -> dict[str, Any]:
    summary = manifest.get("scene_metrics")
    if not isinstance(summary, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture manifest missing scene_metrics"
        )
    event_count = summary.get("event_count")
    if (
        not isinstance(event_count, int)
        or isinstance(event_count, bool)
        or event_count < 100
    ):
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture must record at least 100 scene metric events"
        )
    latest = summary.get("latest")
    if not isinstance(latest, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture scene_metrics missing latest event"
        )
    metrics = latest.get("metrics")
    if not isinstance(metrics, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture latest scene metric payload must be an object"
        )
    if metrics.get("constraint") != "finite_and_motor_physical_load_fracture_lifecycle":
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture metrics identify the wrong fracture lifecycle"
        )
    if metrics.get("phase") not in {
        "weak: break",
        "strong reset: intact",
        "weak re-arm: break",
    }:
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture metrics contain an unknown lifecycle phase"
        )
    for key, expected in (
        ("time_step_ms", TIME_STEP * 1000.0),
        ("start_stiffness", START_STIFFNESS),
        ("linear_stiffness", LINEAR_STIFFNESS),
        ("angular_stiffness", ANGULAR_STIFFNESS),
        ("max_effort", EFFORT_LIMIT),
        ("weak_break_force", WEAK_BREAK_FORCE),
        ("strong_break_force", STRONG_BREAK_FORCE),
        ("prestrain", PRESTRAIN),
    ):
        _require_close(metrics.get(key), expected, f"capture metrics {key}")

    history = metrics.get("history")
    if not isinstance(history, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture metrics missing lifecycle history"
        )
    _require_close(history.get("saw_broken"), 1.0, "history saw_broken")
    _require_close(history.get("saw_intact"), 1.0, "history saw_intact")
    transitions = _finite_number(history.get("transitions"), "history transitions")
    if transitions < 3.0:
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture must include weak break, strong reset, and weak re-arm"
        )
    max_residual = _finite_number(
        history.get("max_transverse_residual"),
        "history max_transverse_residual",
    )
    if max_residual <= PRESTRAIN:
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture must show released transverse drift beyond the preload"
        )
    return {
        "event_count": event_count,
        "latest_frame": latest.get("frame"),
        "latest_metrics": metrics,
        "metric_key_counts": summary.get("metric_key_counts"),
        "numeric_ranges": summary.get("numeric_ranges"),
    }


def _validate_capture(
    manifest_path: Path,
) -> tuple[dict[str, Any], Path, dict[str, Path]]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture manifest schema_version must be 1"
        )
    if manifest.get("scene") != SCENE_ID:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"capture scene must be {SCENE_ID}"
        )
    if manifest.get("switch_scene") is not None:
        raise AvbdArticulatedCompliantFracturePacketError(
            "fracture capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdArticulatedCompliantFracturePacketError(
            "fracture capture must not force-drag"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture manifest missing artifacts"
        )
    manifest_dir = manifest_path.parent
    screenshot = _capture_artifact_path(
        manifest_dir, artifacts.get("screenshot"), "screenshot"
    )
    frames_dir = _capture_artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    metrics_events = _capture_artifact_path(
        manifest_dir,
        artifacts.get("scene_metrics_events"),
        "scene_metrics_events",
    )
    if not screenshot.is_file():
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{screenshot}: screenshot not found"
        )
    if not frames_dir.is_dir():
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{frames_dir}: frame directory not found"
        )
    if not metrics_events.is_file():
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{metrics_events}: scene metrics event log not found"
        )

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if len(frame_paths) < 100:
        raise AvbdArticulatedCompliantFracturePacketError(
            "fracture capture must include at least 100 PNG frames"
        )
    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdArticulatedCompliantFracturePacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdArticulatedCompliantFracturePacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "capture manifest missing ui_ready"
        )
    if manifest.get("show_ui") is not True or ui_ready.get("required") is not True:
        raise AvbdArticulatedCompliantFracturePacketError(
            "fracture capture must show the required docked workspace"
        )
    visual_evidence = manifest.get("visual_evidence")
    if not isinstance(visual_evidence, dict) or any(
        not isinstance(visual_evidence.get(role), dict)
        or visual_evidence[role].get("docked_workspace") is not True
        for role in ("first_frame", "screenshot")
    ):
        raise AvbdArticulatedCompliantFracturePacketError(
            "fracture capture must verify the docked workspace"
        )

    middle_frame = frame_paths[(len(frame_paths) - 1) // 2]
    return (
        {
            "manifest_sha256": _sha256(manifest_path),
            "scene": SCENE_ID,
            "show_ui": True,
            "ui_ready": ui_ready,
            "screenshot": {
                "file": _artifact_label(manifest_dir, screenshot),
                "sha256": _sha256(screenshot),
                "width": width,
                "height": height,
            },
            "frames": {
                "directory": _artifact_label(manifest_dir, frames_dir),
                "count": len(frame_paths),
                "first_frame": {
                    "file": _artifact_label(manifest_dir, frame_paths[0]),
                    "sha256": _sha256(frame_paths[0]),
                },
                "last_frame": {
                    "file": _artifact_label(manifest_dir, frame_paths[-1]),
                    "sha256": _sha256(frame_paths[-1]),
                },
            },
            "scene_metrics_events": {
                "file": _artifact_label(manifest_dir, metrics_events),
                "sha256": _sha256(metrics_events),
            },
            "scene_metrics": _validate_scene_metrics(manifest),
        },
        screenshot,
        {
            "start": frame_paths[0],
            "middle": middle_frame,
            "end": frame_paths[-1],
        },
    )


def _validate_sha256_hex(value: object, label: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{label} must be a 64-character lowercase hexadecimal string"
        )
    try:
        parsed = int(value, 16)
    except ValueError as exc:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{label} must be hexadecimal"
        ) from exc
    if f"{parsed:064x}" != value:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{label} must use canonical lowercase hexadecimal"
        )
    return value


def _validate_image_verdict(
    verdict_path: Path,
    screenshot: Path,
) -> dict[str, Any]:
    verdict = _load_json(verdict_path)
    if verdict.get("schema_version") != "dart.image_verdict/v1":
        raise AvbdArticulatedCompliantFracturePacketError(
            "image verdict schema_version must be dart.image_verdict/v1"
        )
    if verdict.get("pass") is not True:
        raise AvbdArticulatedCompliantFracturePacketError("image verdict must pass")
    checks = verdict.get("checks")
    if not isinstance(checks, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "image verdict missing checks"
        )
    non_blank = checks.get("non_blank")
    if not isinstance(non_blank, dict) or non_blank.get("pass") is not True:
        raise AvbdArticulatedCompliantFracturePacketError(
            "image verdict non_blank check must pass"
        )
    image = verdict.get("image")
    if not isinstance(image, dict):
        raise AvbdArticulatedCompliantFracturePacketError(
            "image verdict missing image metadata"
        )
    width, height = _png_dimensions(screenshot)
    if image.get("width") != width or image.get("height") != height:
        raise AvbdArticulatedCompliantFracturePacketError(
            "image verdict dimensions do not match capture screenshot"
        )
    screenshot_sha256 = _sha256(screenshot)
    recorded_sha256 = _validate_sha256_hex(
        image.get("sha256"),
        "image verdict image sha256",
    )
    if recorded_sha256 != screenshot_sha256:
        raise AvbdArticulatedCompliantFracturePacketError(
            "image verdict image sha256 does not match capture screenshot"
        )
    return {
        "file": verdict_path.name,
        "sha256": _sha256(verdict_path),
        "image_sha256": screenshot_sha256,
        "machine_scope": verdict.get("machine_scope"),
        "metadata": verdict.get("metadata"),
        "checks": checks,
        "pass": True,
    }


def _validate_visual_review(
    review_path: Path,
    manifest_path: Path,
    inspected_frames: dict[str, Path],
) -> dict[str, Any]:
    review = _load_json(review_path)
    if review.get("schema_version") != "dart.visual_semantic_review/v1":
        raise AvbdArticulatedCompliantFracturePacketError(
            "visual review schema_version must be dart.visual_semantic_review/v1"
        )
    if review.get("scene") != SCENE_ID:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"visual review scene must be {SCENE_ID}"
        )
    if review.get("verdict") != "pass":
        raise AvbdArticulatedCompliantFracturePacketError(
            "visual semantic review must pass"
        )
    for key in ("observations", "limitations"):
        value = review.get(key)
        if not isinstance(value, str) or not value.strip():
            raise AvbdArticulatedCompliantFracturePacketError(
                f"visual review {key} must be non-empty"
            )

    entries = review.get("inspected_frames")
    if not isinstance(entries, list):
        raise AvbdArticulatedCompliantFracturePacketError(
            "visual review inspected_frames must be a list"
        )
    by_role = {
        entry.get("role"): entry
        for entry in entries
        if isinstance(entry, dict) and isinstance(entry.get("role"), str)
    }
    manifest_dir = manifest_path.parent
    validated_frames = []
    for role, frame_path in inspected_frames.items():
        entry = by_role.get(role)
        if entry is None:
            raise AvbdArticulatedCompliantFracturePacketError(
                f"visual review missing {role} frame"
            )
        expected_file = _artifact_label(manifest_dir, frame_path)
        expected_sha = _sha256(frame_path)
        if entry.get("file") != expected_file:
            raise AvbdArticulatedCompliantFracturePacketError(
                f"visual review {role} frame must be {expected_file}"
            )
        if entry.get("sha256") != expected_sha:
            raise AvbdArticulatedCompliantFracturePacketError(
                f"visual review {role} frame sha256 mismatch"
            )
        validated_frames.append(
            {
                "role": role,
                "file": expected_file,
                "sha256": expected_sha,
            }
        )
    return {
        "file": review_path.name,
        "sha256": _sha256(review_path),
        "reviewer_capability": review.get("reviewer_capability"),
        "inspected_frames": validated_frames,
        "observations": review["observations"],
        "limitations": review["limitations"],
        "verdict": "pass",
    }


def _benchmark_arg(row: dict[str, Any]) -> int | None:
    identity = _canonical_name(_row_name(row))
    prefix = f"{BENCHMARK_NAME}/"
    if not identity.startswith(prefix):
        return None
    arg_text = identity[len(prefix) :]
    try:
        arg = int(arg_text)
    except ValueError as exc:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{BENCHMARK_NAME}: non-integer benchmark argument {arg_text!r}"
        ) from exc
    if arg not in BENCHMARK_ARGS:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"{BENCHMARK_NAME}: unexpected family_instances argument {arg}"
        )
    return arg


def _is_representative(row: dict[str, Any]) -> bool:
    run_type = row.get("run_type", "iteration")
    aggregate_name = row.get("aggregate_name")
    return run_type == "iteration" or aggregate_name in ("mean", "median")


def _timing_row(rows: list[dict[str, Any]]) -> dict[str, Any]:
    median_rows = [row for row in rows if row.get("aggregate_name") == "median"]
    if median_rows:
        return median_rows[0]
    mean_rows = [row for row in rows if row.get("aggregate_name") == "mean"]
    if mean_rows:
        return mean_rows[0]
    return rows[0]


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdArticulatedCompliantFracturePacketError(
            "benchmark JSON missing benchmarks list"
        )

    representative: dict[int, list[dict[str, Any]]] = {
        arg: [] for arg in BENCHMARK_ARGS
    }
    packet_rows: list[dict[str, Any]] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        arg = _benchmark_arg(row)
        if arg is None:
            continue
        _finite_number(row.get("real_time"), f"{BENCHMARK_NAME} real_time")
        _finite_number(row.get("cpu_time"), f"{BENCHMARK_NAME} cpu_time")
        packet_rows.append(row)
        if not _is_representative(row):
            continue
        for key, expected in (
            ("family_instances", float(arg)),
            ("revolute_motors", float(arg)),
            ("prismatic_motors", float(arg)),
            ("breakable_motors", float(2 * arg)),
        ):
            _require_close(row.get(key), expected, f"{BENCHMARK_NAME} {key}")
        representative[arg].append(row)

    missing = [arg for arg in BENCHMARK_ARGS if not representative[arg]]
    if missing:
        raise AvbdArticulatedCompliantFracturePacketError(
            "benchmark JSON missing breakable finite-motor scale rows: "
            + ", ".join(str(arg) for arg in missing)
        )

    scale_data = []
    for arg in BENCHMARK_ARGS:
        row = _timing_row(representative[arg])
        try:
            runtime_identity = make_resolved_solver_identity_from_benchmark_row(
                row,
                recorded_from=RESOLVED_SOLVER_IDENTITY["recorded_from"],
            )
        except ValueError as exc:
            raise AvbdArticulatedCompliantFracturePacketError(str(exc)) from exc
        if runtime_identity != RESOLVED_SOLVER_IDENTITY:
            raise AvbdArticulatedCompliantFracturePacketError(
                f"{BENCHMARK_NAME}/{arg}: runtime solver identity is not the "
                "Variational multibody packet identity"
            )
        motor_count = 2 * arg
        cpu_time = _finite_number(
            row.get("cpu_time"), f"{BENCHMARK_NAME}/{arg} cpu_time"
        )
        real_time = _finite_number(
            row.get("real_time"), f"{BENCHMARK_NAME}/{arg} real_time"
        )
        scale_data.append(
            {
                "benchmark": f"{BENCHMARK_NAME}/{arg}",
                "family_instances": arg,
                "breakable_motors": motor_count,
                "cpu_time_per_step_ns": cpu_time,
                "real_time_per_step_ns": real_time,
                "cpu_time_per_motor_ns": cpu_time / motor_count,
                "time_unit": row.get("time_unit", "ns"),
            }
        )

    context = data.get("context", {})
    if not isinstance(context, dict):
        context = {}
    return {
        "json_sha256": _sha256(benchmark_json),
        "benchmark": BENCHMARK_NAME,
        "context": {
            key: context[key]
            for key in (
                "executable",
                "num_cpus",
                "mhz_per_cpu",
                "library_version",
                "library_build_type",
                "json_schema_version",
            )
            if key in context
        },
        "rows": packet_rows,
        "scale_data": scale_data,
        "invariants": {
            "family_instances": list(BENCHMARK_ARGS),
            "motor_types": ["revolute", "prismatic"],
            "endpoint_topology": "same-multibody movable-link pairs",
            "breakable": True,
            "break_force": 1.0e12,
            "time_step": TIME_STEP,
        },
    }


def _validate_exact_parent_mutation(path: Path) -> dict[str, Any]:
    evidence = _load_json(path)
    if evidence.get("schema_version") != "dart.exact_parent_mutation/v1":
        raise AvbdArticulatedCompliantFracturePacketError(
            "exact-parent mutation schema_version must be "
            "dart.exact_parent_mutation/v1"
        )
    if evidence.get("parent_commit") != EXACT_PARENT_COMMIT:
        raise AvbdArticulatedCompliantFracturePacketError(
            f"exact-parent mutation parent_commit must be {EXACT_PARENT_COMMIT}"
        )
    if evidence.get("tests") != [LOAD_TEST, LIFECYCLE_TEST]:
        raise AvbdArticulatedCompliantFracturePacketError(
            "exact-parent mutation must run the load and lifecycle tests"
        )
    if evidence.get("exit_code") != 1:
        raise AvbdArticulatedCompliantFracturePacketError(
            "exact-parent mutation exit_code must be 1"
        )
    if (
        evidence.get("expected_result") != "fail"
        or evidence.get("observed_result") != "fail"
    ):
        raise AvbdArticulatedCompliantFracturePacketError(
            "exact-parent mutation must record expected and observed failure"
        )

    assertions = evidence.get("failed_assertions")
    if not isinstance(assertions, list):
        raise AvbdArticulatedCompliantFracturePacketError(
            "exact-parent mutation failed_assertions must be a list"
        )
    required_cases = {
        "finite_only",
        "motor_dt_0.005",
        "motor_dt_0.01",
        "combined_load",
        "reset_lifecycle",
    }
    observed_cases = {
        assertion.get("case")
        for assertion in assertions
        if isinstance(assertion, dict)
        and assertion.get("expected") is True
        and assertion.get("actual") is False
    }
    missing_cases = sorted(required_cases - observed_cases)
    if missing_cases:
        raise AvbdArticulatedCompliantFracturePacketError(
            "exact-parent mutation missing failed cases: " + ", ".join(missing_cases)
        )

    for key in ("mutation", "command", "interpretation"):
        value = evidence.get(key)
        if not isinstance(value, str) or not value.strip():
            raise AvbdArticulatedCompliantFracturePacketError(
                f"exact-parent mutation {key} must be non-empty"
            )
    return {
        "file": path.name,
        "sha256": _sha256(path),
        "parent_commit": EXACT_PARENT_COMMIT,
        "tests": [LOAD_TEST, LIFECYCLE_TEST],
        "mutation": evidence["mutation"],
        "command": evidence["command"],
        "exit_code": 1,
        "observed_result": "fail",
        "failed_assertions": assertions,
        "interpretation": evidence["interpretation"],
    }


def make_packet(
    capture_manifest: Path,
    image_verdict_json: Path,
    visual_review_json: Path,
    benchmark_json: Path,
    exact_parent_mutation_json: Path,
) -> dict[str, Any]:
    visual_capture, screenshot, inspected_frames = _validate_capture(capture_manifest)
    visual_capture["image_verdict"] = _validate_image_verdict(
        image_verdict_json, screenshot
    )
    visual_capture["semantic_review"] = _validate_visual_review(
        visual_review_json, capture_manifest, inspected_frames
    )
    return {
        "schema_version": AVBD_PACKET_SCHEMA_VERSION,
        "resolved_solver_identity": RESOLVED_SOLVER_IDENTITY,
        "packet": "avbd_articulated_compliant_fracture",
        "scene": SCENE_ID,
        "target": {
            "contract_rows": [
                "avbd.method.joints_and_attachments",
                "avbd.method.finite_stiffness_ramping",
                "avbd.method.motors",
                "avbd.method.fracture",
            ],
            "scope": (
                "solver-row break-metric accounting across finite masked rows "
                "and bounded motor projection rows for same-multibody "
                "articulated point joints on CPU"
            ),
            "complete_paper_reproduction": False,
        },
        "load_contract": {
            "finite_rows": "accepted stiffness times accepted residual",
            "hard_and_motor_projection_rows": (
                "position-level projection lambda divided by time_step squared"
            ),
            "joint_aggregation": "L2 norm across finite and projection rows",
            "public_threshold_units": "force or torque",
            "fracture_action": (
                "mark the source joint broken and clear its finite row "
                "lambda/stiffness state before later-step extraction skips it"
            ),
        },
        "correctness": {
            "cpp_load_test": LOAD_TEST,
            "cpp_lifecycle_test": LIFECYCLE_TEST,
            "cpp_serialization_test": SERIALIZATION_TEST,
            "python_scene_test": (
                "test_avbd_articulated_compliant_breakable_motor_demo_"
                "rearms_finite_rows"
            ),
            "allocation_tests": [
                "World.BakedVariationalCompliantArticulatedPointJointRowsDoNotGrowWorldBaseAllocator",
                "World.BakedVariationalCompliantArticulatedPointJointRowsDoNotAllocateGlobalHeap",
                "World.BakedVariationalCompliantArticulatedPointJointRowsDoNotMallocOnHeap",
            ],
            "exact_parent_mutation": _validate_exact_parent_mutation(
                exact_parent_mutation_json
            ),
        },
        "visual_capture": visual_capture,
        "benchmark": _validate_benchmark(benchmark_json),
        "performance_claim_boundary": (
            "Candidate-only finite breakable-mechanism cost and scaling. The "
            "exact parent did not account for these finite or bounded-motor "
            "loads, so this packet makes no before/after speedup claim."
        ),
        "claim_boundary": (
            "This closes the narrow CPU articulated finite-row load-accounting "
            "gap. It does not reproduce the paper wall, close the broad "
            "fracture corpus, unify rigid and soft rows, provide CUDA parity, "
            "or establish a paper/reference performance win."
        ),
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 "
                "MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "PYTHONPATH=build/default/cpp/Release-docking/python:python "
                "BUILD_TYPE=Release-docking python scripts/capture_py_demo.py "
                f"--scene {SCENE_ID} --frames 150 --width 960 --height 540 "
                "--show-ui --view three-quarter --camera-distance 5 "
                "--camera-target 0,0,1 --output-dir <capture-dir>"
            ),
            "image_verdict_command": (
                "pixi run image-verdict -- "
                f"<capture-dir>/{SCENE_ID}.png "
                "--out <capture-dir>/image_verdict.json "
                f"--meta scene={SCENE_ID} --meta view=three-quarter"
            ),
            "benchmark_command": (
                "taskset -c 18 "
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter="
                f"'{BENCHMARK_NAME}/(1|4|16)$' "
                "--benchmark_min_time=0.05s --benchmark_repetitions=5 "
                "--benchmark_report_aggregates_only=true "
                "--benchmark_out=<benchmark-json> "
                "--benchmark_out_format=json"
            ),
        },
    }


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(
            args.capture_manifest,
            args.image_verdict_json,
            args.visual_review_json,
            args.benchmark_json,
            args.exact_parent_mutation_json,
        )
    except (OSError, ValueError, AvbdArticulatedCompliantFracturePacketError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
