#!/usr/bin/env python3
"""Write a validated AVBD articulated compliant-motor evidence packet."""

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
    "avbd-articulated-compliant-motors-packet.json"
)
SCENE_ID = "avbd_articulated_compliant_motors"
BENCHMARK_NAME = "BM_AvbdArticulatedCompliantMotorStep"
BENCHMARK_ARGS = (1, 4, 16)
EXACT_PARENT_COMMIT = "761263bbd41190536ce699439ebc64f5776757b6"
BEHAVIOR_TEST = (
    "VariationalIntegration."
    "AvbdCompliantPublicArticulatedOneDofMotorsDriveMovableLinkPairs"
)
PASSIVE_PAIR_TEST = (
    "VariationalIntegration."
    "AvbdCompliantPublicArticulatedJointMasksSupportMovableLinkPairs"
)
TIME_STEP = 0.005
START_STIFFNESS = 20.0
LINEAR_STIFFNESS = 2000.0
ANGULAR_STIFFNESS = 2000.0
EFFORT_LIMIT = 800.0
RESOLVED_SOLVER_IDENTITY = make_resolved_solver_identity(
    resolved_rigid_contact_family="sequential-impulse",
    rigid_point_joint_solver="sequential_impulse",
    avbd_rigid_contact_config_emplaced=False,
    recorded_from=("articulated compliant-motor benchmark runtime identity counters"),
    multibody_integration_family="variational",
)


class AvbdArticulatedCompliantMotorsPacketError(RuntimeError):
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


def _validate_capture(
    manifest_path: Path,
) -> tuple[dict[str, Any], Path, dict[str, Path]]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "capture manifest schema_version must be 1"
        )
    if manifest.get("scene") != SCENE_ID:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"capture scene must be {SCENE_ID}"
        )
    if manifest.get("switch_scene") is not None:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "compliant-motor capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "compliant-motor capture must not force-drag"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "capture manifest missing artifacts"
        )
    manifest_dir = manifest_path.parent
    screenshot = _capture_artifact_path(
        manifest_dir, artifacts.get("screenshot"), "screenshot"
    )
    frames_dir = _capture_artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{screenshot}: screenshot not found"
        )
    if not frames_dir.is_dir():
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{frames_dir}: frame directory not found"
        )

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{frames_dir}: no PNG frames found"
        )
    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "capture manifest missing ui_ready"
        )
    if manifest.get("show_ui") is not True or ui_ready.get("required") is not True:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "compliant-motor capture must show the required docked workspace"
        )
    visual_evidence = manifest.get("visual_evidence")
    if not isinstance(visual_evidence, dict) or any(
        not isinstance(visual_evidence.get(role), dict)
        or visual_evidence[role].get("docked_workspace") is not True
        for role in ("first_frame", "screenshot")
    ):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "compliant-motor capture must verify the docked workspace"
        )

    middle_frame = frame_paths[(len(frame_paths) - 1) // 2]
    return (
        {
            "manifest_sha256": _sha256(manifest_path),
            "scene": SCENE_ID,
            "show_ui": bool(manifest.get("show_ui")),
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
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{label} must be a 64-character lowercase hexadecimal string"
        )
    try:
        parsed = int(value, 16)
    except ValueError as exc:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{label} must be hexadecimal"
        ) from exc
    if f"{parsed:064x}" != value:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{label} must use canonical lowercase hexadecimal"
        )
    return value


def _validate_image_verdict(
    verdict_path: Path,
    screenshot: Path,
) -> dict[str, Any]:
    verdict = _load_json(verdict_path)
    if verdict.get("schema_version") != "dart.image_verdict/v1":
        raise AvbdArticulatedCompliantMotorsPacketError(
            "image verdict schema_version must be dart.image_verdict/v1"
        )
    if verdict.get("pass") is not True:
        raise AvbdArticulatedCompliantMotorsPacketError("image verdict must pass")
    checks = verdict.get("checks")
    if not isinstance(checks, dict):
        raise AvbdArticulatedCompliantMotorsPacketError("image verdict missing checks")
    non_blank = checks.get("non_blank")
    if not isinstance(non_blank, dict) or non_blank.get("pass") is not True:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "image verdict non_blank check must pass"
        )
    image = verdict.get("image")
    if not isinstance(image, dict):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "image verdict missing image metadata"
        )
    if (
        image.get("width") != _png_dimensions(screenshot)[0]
        or image.get("height") != _png_dimensions(screenshot)[1]
    ):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "image verdict dimensions do not match capture screenshot"
        )
    screenshot_sha256 = _sha256(screenshot)
    recorded_sha256 = _validate_sha256_hex(
        image.get("sha256"),
        "image verdict image sha256",
    )
    if recorded_sha256 != screenshot_sha256:
        raise AvbdArticulatedCompliantMotorsPacketError(
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
        raise AvbdArticulatedCompliantMotorsPacketError(
            "visual review schema_version must be " "dart.visual_semantic_review/v1"
        )
    if review.get("scene") != SCENE_ID:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"visual review scene must be {SCENE_ID}"
        )
    if review.get("verdict") != "pass":
        raise AvbdArticulatedCompliantMotorsPacketError(
            "visual semantic review must pass"
        )
    observations = review.get("observations")
    limitations = review.get("limitations")
    if not isinstance(observations, str) or not observations.strip():
        raise AvbdArticulatedCompliantMotorsPacketError(
            "visual review observations must be non-empty"
        )
    if not isinstance(limitations, str) or not limitations.strip():
        raise AvbdArticulatedCompliantMotorsPacketError(
            "visual review limitations must be non-empty"
        )

    entries = review.get("inspected_frames")
    if not isinstance(entries, list):
        raise AvbdArticulatedCompliantMotorsPacketError(
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
            raise AvbdArticulatedCompliantMotorsPacketError(
                f"visual review missing {role} frame"
            )
        expected_file = _artifact_label(manifest_dir, frame_path)
        if entry.get("file") != expected_file:
            raise AvbdArticulatedCompliantMotorsPacketError(
                f"visual review {role} frame must be {expected_file}"
            )
        expected_sha = _sha256(frame_path)
        if entry.get("sha256") != expected_sha:
            raise AvbdArticulatedCompliantMotorsPacketError(
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
        "observations": observations,
        "limitations": limitations,
        "verdict": "pass",
    }


def _finite_counter(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{BENCHMARK_NAME}: missing {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{BENCHMARK_NAME}: non-finite {key}"
        )
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=1e-15):
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _benchmark_arg(row: dict[str, Any]) -> int | None:
    identity = _canonical_name(_row_name(row))
    prefix = f"{BENCHMARK_NAME}/"
    if not identity.startswith(prefix):
        return None
    arg_text = identity[len(prefix) :]
    try:
        arg = int(arg_text)
    except ValueError as exc:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"{BENCHMARK_NAME}: non-integer benchmark argument {arg_text!r}"
        ) from exc
    if arg not in BENCHMARK_ARGS:
        raise AvbdArticulatedCompliantMotorsPacketError(
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
        raise AvbdArticulatedCompliantMotorsPacketError(
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
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)
        if not _is_representative(row):
            continue
        _require_counter(row, "family_instances", float(arg))
        _require_counter(row, "revolute_motors", float(arg))
        _require_counter(row, "prismatic_motors", float(arg))
        _require_counter(row, "compliant_motors", float(2 * arg))
        representative[arg].append(row)

    missing = [arg for arg in BENCHMARK_ARGS if not representative[arg]]
    if missing:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "benchmark JSON missing compliant-motor scale rows: "
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
            raise AvbdArticulatedCompliantMotorsPacketError(str(exc)) from exc
        if runtime_identity != RESOLVED_SOLVER_IDENTITY:
            raise AvbdArticulatedCompliantMotorsPacketError(
                f"{BENCHMARK_NAME}/{arg}: runtime solver identity is not the "
                "Variational multibody packet identity"
            )
        compliant_motors = 2 * arg
        cpu_time = _finite_counter(row, "cpu_time")
        real_time = _finite_counter(row, "real_time")
        scale_data.append(
            {
                "benchmark": f"{BENCHMARK_NAME}/{arg}",
                "family_instances": arg,
                "compliant_motors": compliant_motors,
                "cpu_time_per_step_ns": cpu_time,
                "real_time_per_step_ns": real_time,
                "cpu_time_per_motor_ns": cpu_time / compliant_motors,
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
            "time_step": TIME_STEP,
            "start_stiffness": START_STIFFNESS,
            "linear_stiffness": LINEAR_STIFFNESS,
            "angular_stiffness": ANGULAR_STIFFNESS,
            "effort_limit": EFFORT_LIMIT,
        },
    }


def _validate_exact_parent_mutation(path: Path) -> dict[str, Any]:
    evidence = _load_json(path)
    if evidence.get("schema_version") != "dart.exact_parent_mutation/v1":
        raise AvbdArticulatedCompliantMotorsPacketError(
            "exact-parent mutation schema_version must be "
            "dart.exact_parent_mutation/v1"
        )
    if evidence.get("parent_commit") != EXACT_PARENT_COMMIT:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"exact-parent mutation parent_commit must be {EXACT_PARENT_COMMIT}"
        )
    if evidence.get("test") != BEHAVIOR_TEST:
        raise AvbdArticulatedCompliantMotorsPacketError(
            f"exact-parent mutation test must be {BEHAVIOR_TEST}"
        )
    if evidence.get("exit_code") != 1:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "exact-parent mutation exit_code must be 1"
        )
    if (
        evidence.get("expected_result") != "fail"
        or evidence.get("observed_result") != "fail"
    ):
        raise AvbdArticulatedCompliantMotorsPacketError(
            "exact-parent mutation must record expected and observed failure"
        )

    assertions = evidence.get("failed_assertions")
    if not isinstance(assertions, list) or len(assertions) != 2:
        raise AvbdArticulatedCompliantMotorsPacketError(
            "exact-parent mutation must record both strong-motor failures"
        )
    validated_assertions = []
    for assertion in assertions:
        if not isinstance(assertion, dict):
            raise AvbdArticulatedCompliantMotorsPacketError(
                "exact-parent mutation assertions must be objects"
            )
        coordinate = assertion.get("coordinate")
        actual = assertion.get("actual")
        expected = assertion.get("expected")
        if not isinstance(coordinate, str) or not coordinate.strip():
            raise AvbdArticulatedCompliantMotorsPacketError(
                "exact-parent mutation assertion coordinate must be non-empty"
            )
        if (
            not isinstance(actual, (int, float))
            or isinstance(actual, bool)
            or not math.isfinite(float(actual))
            or not isinstance(expected, (int, float))
            or isinstance(expected, bool)
            or not math.isfinite(float(expected))
            or float(expected) <= 0.0
            or abs(float(actual)) >= 0.1 * float(expected)
        ):
            raise AvbdArticulatedCompliantMotorsPacketError(
                "exact-parent mutation assertions must record finite near-zero "
                "actual values against positive driven expectations"
            )
        validated_assertions.append(
            {
                "coordinate": coordinate,
                "actual": float(actual),
                "expected": float(expected),
            }
        )

    mutation = evidence.get("mutation")
    command = evidence.get("command")
    interpretation = evidence.get("interpretation")
    for label, value in (
        ("mutation", mutation),
        ("command", command),
        ("interpretation", interpretation),
    ):
        if not isinstance(value, str) or not value.strip():
            raise AvbdArticulatedCompliantMotorsPacketError(
                f"exact-parent mutation {label} must be non-empty"
            )
    return {
        "file": path.name,
        "sha256": _sha256(path),
        "parent_commit": EXACT_PARENT_COMMIT,
        "test": BEHAVIOR_TEST,
        "mutation": mutation,
        "command": command,
        "exit_code": 1,
        "observed_result": "fail",
        "failed_assertions": validated_assertions,
        "interpretation": interpretation,
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
        "packet": "avbd_articulated_compliant_motors",
        "scene": SCENE_ID,
        "target": {
            "contract_rows": [
                "avbd.method.joints_and_attachments",
                "avbd.method.finite_stiffness_ramping",
                "avbd.method.motors",
            ],
            "scope": (
                "bounded finite-stiffness articulated revolute and prismatic "
                "velocity motors between movable links of one multibody on CPU"
            ),
            "complete_paper_reproduction": False,
        },
        "scene_invariants": {
            "motor_types": ["revolute", "prismatic"],
            "endpoint_topology": "same-multibody movable-link pairs",
            "free_coordinates": {
                "revolute": "rotation about the joint axis",
                "prismatic": "translation along the joint axis",
            },
            "time_step": TIME_STEP,
            "start_stiffness": START_STIFFNESS,
            "linear_stiffness": LINEAR_STIFFNESS,
            "angular_stiffness": ANGULAR_STIFFNESS,
            "effort_limit": EFFORT_LIMIT,
        },
        "correctness": {
            "cpp_behavior_test": BEHAVIOR_TEST,
            "cpp_passive_pair_test": PASSIVE_PAIR_TEST,
            "python_scene_test": (
                "test_avbd_articulated_compliant_motors_demo_" "drives_free_coordinates"
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
            "Candidate-only finite mechanism cost and scaling. The exact "
            "parent intentionally skipped these non-Fixed finite rows, so "
            "this packet makes no before/after speedup claim."
        ),
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "PYTHONPATH=build/default/cpp/Release-docking/python:python "
                "BUILD_TYPE=Release-docking python scripts/capture_py_demo.py "
                "--scene avbd_articulated_compliant_motors --frames 120 "
                "--width 960 --height 540 --show-ui --view three-quarter "
                "--camera-distance 7 --camera-target 0,0,1.05 "
                "--output-dir <capture-dir>"
            ),
            "image_verdict_command": (
                "pixi run image-verdict -- <capture-dir>/"
                "avbd_articulated_compliant_motors.png "
                "--out <capture-dir>/image_verdict.json "
                "--meta scene=avbd_articulated_compliant_motors "
                "--meta view=three-quarter"
            ),
            "benchmark_command": (
                "taskset -c 18 "
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter='BM_AvbdArticulatedCompliantMotorStep/"
                "(1|4|16)$' --benchmark_min_time=0.05s "
                "--benchmark_repetitions=5 "
                "--benchmark_report_aggregates_only=true "
                "--benchmark_out=<benchmark-json> "
                "--benchmark_out_format=json"
            ),
        },
        "remaining_gates": [
            "finite-row break-force accounting and fracture lifecycle",
            "unified soft/rigid row solve",
            "CUDA AVBD row parity and same-hardware benchmark packets",
            "source-demo, paper/site/video scene, and achieved-accuracy performance closure",
        ],
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


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
    except AvbdArticulatedCompliantMotorsPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD articulated compliant-motors packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
