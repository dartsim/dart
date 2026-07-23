#!/usr/bin/env python3
"""Finalize and revalidate the current-source DART backspin visual bundle.

This driver deliberately keeps the scientific claim narrow. It binds one
runner-native DART capture to a fresh exact-FBF translational trace, a recorded
manual inspection, and the current source/binary identities. It does not claim
paper parity, external-solver parity, an approved image golden, uninterrupted
contact, rest, signed angular direction from sampled media, or real-time
performance.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import hashlib
import io
import json
import math
import os
import shutil
import stat
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any, Callable, Iterator, Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "fig03_backspin_current_v3"
)
DEFAULT_TRACE_BINARY = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
)
DEFAULT_DEMO = ROOT / "build/default/cpp/Release/bin/dart-demos"
DEFAULT_RUNNER = ROOT / "scripts/run_fbf_visual_evidence.py"
DEFAULT_FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
DEFAULT_FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"
DEFAULT_PYTHON = Path(sys.executable)

TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
FIXTURE_SOURCE = ROOT / "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp"
DEMO_SOURCE = ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
CHECKER_MESH = ROOT / "data/obj/fbf_backspin_checker_sphere.obj"
CHECKER_MATERIAL = ROOT / "data/obj/fbf_backspin_checker_sphere.mtl"
CHECKER_TEXTURE = ROOT / "data/obj/fbf_backspin_checker.ppm"
DEMO_HOST_SOURCE = ROOT / "examples/demos/DemoHost.cpp"
DEMO_HOST_HEADER = ROOT / "examples/demos/DemoHost.hpp"
REGISTRY_SOURCE = ROOT / "examples/demos/Registry.cpp"
SCENES_HEADER = ROOT / "examples/demos/scenes/Scenes.hpp"
RUNNER_TEST = ROOT / "python/tests/unit/test_run_fbf_visual_evidence.py"
FINALIZER_TEST = ROOT / "python/tests/unit/test_finalize_fbf_backspin_visual.py"
IMAGE_COMPOSE = ROOT / "scripts/image_compose.py"
IMAGE_TOOLS = ROOT / "scripts/_image_tools.py"
IMAGE_VERDICT = ROOT / "scripts/image_verdict.py"

SCHEMA_VERSION = "dart.fbf_backspin_visual_bundle/v3"
INDEX_SCHEMA_VERSION = "dart.fbf_backspin_artifact_index/v1"
MANUAL_SCHEMA_VERSION = "dart.fbf_backspin_manual_inspection/v3"
TRACE_SUMMARY_SCHEMA_VERSION = "dart.fbf_backspin_trace_summary/v1"
INVOCATIONS_SCHEMA_VERSION = "dart.fbf_backspin_invocations/v1"
RUNNER_SCHEMA_VERSION = "dart.fbf_visual_evidence/v1"
STAGING_SCHEMA_VERSION = "dart.fbf_backspin_pruned_staging/v1"

TRACE_COLUMNS = (
    "step",
    "time",
    "scenario",
    "solver",
    "body",
    "x",
    "y",
    "z",
    "vx",
    "vy",
    "vz",
    "up_z",
    "contacts",
    "exact_solves",
    "warm_starts",
    "fallbacks",
    "residual",
    "status",
)
TOTAL_STEPS = 130
DT_SECONDS = 1.0 / 60.0
RESIDUAL_TOLERANCE = 1.0e-6
PANEL_STEPS = (0, 10, 50, 120, 130)
PANEL_LABELS = (
    "t=0.00s",
    "t=0.17s",
    "t=0.83s",
    "t=2.00s",
    "t=2.17s",
)
EXPECTED_CONFIGURATION = {
    "radius_m": "0.25",
    "initial_linear_velocity_m_s": "4",
    "initial_angular_velocity_rad_s": "-200",
    "mu": "0.5",
    "orientation_cue": (
        "renderer-applied high-contrast 6x4 checker texture with coral "
        "registration tile"
    ),
}
INDEX_EXCLUSIONS = {"artifact-index.json", "metadata.json"}

MANUAL_VERDICTS = {
    "checker_texture_legible": True,
    "coral_registration_tile_visible": True,
    "orientation_change_visible_in_consecutive_source_frames": True,
    "texture_loading_warnings_observed": False,
    "translational_advance_then_reversal_visible": True,
    "translational_reversal_claim_requires_physical_trace": True,
    "physics_neutrality_requires_source_contract": True,
    "media_sampling_alias_disclosed": True,
    "signed_angular_direction_proven_by_media": False,
    "continuous_contact_proven": False,
    "strict_rigid_body_rest_proven": False,
    "paper_parity": False,
    "external_solver_parity": False,
    "approved_source_golden": False,
    "timing_verdict": None,
    "realtime_verdict": None,
}
DURABLE_STILL_STEPS = (0, 1, 2)
DURABLE_STILL_PATHS = {
    f"backspin/stills/step_{step:06d}.png" for step in DURABLE_STILL_STEPS
}
MANUAL_ARTIFACT_PATHS = {
    "backspin/panel.png",
    "backspin/clip.mp4",
    "backspin/clip.gif",
    *DURABLE_STILL_PATHS,
}


def _capture_paths() -> set[str]:
    paths = {
        "run-summary.json",
        "capture-provenance.json",
        "capture.stdout.txt",
        "capture.stderr.txt",
        "manual-inspection.json",
        "backspin/metadata.json",
        "backspin/timeline.json",
        "backspin/panel.png",
        "backspin/panel.compose.json",
        "backspin/clip.mp4",
        "backspin/clip.gif",
        "backspin/video_frames.ffconcat",
    }
    paths.update(
        f"backspin/frames/step_{step:06d}.png" for step in range(TOTAL_STEPS + 1)
    )
    paths.update(f"backspin/panel_frames/step_{step:06d}.png" for step in PANEL_STEPS)
    return paths


CAPTURE_PATHS = _capture_paths()
GENERATED_PATHS = {
    "traces/backspin.csv",
    "traces/backspin.stderr.txt",
    "trace-summary.json",
    "verification.json",
    "verification.stderr.txt",
    "invocations.json",
    "REPORT.md",
    "artifact-index.json",
    "metadata.json",
}
STAGING_PATHS = {
    "capture.stdout.txt",
    "capture.stderr.txt",
    "verification.stderr.txt",
    "backspin/video_frames.ffconcat",
    *(f"backspin/frames/step_{step:06d}.png" for step in range(TOTAL_STEPS + 1)),
    *(f"backspin/panel_frames/step_{step:06d}.png" for step in PANEL_STEPS),
}
DURABLE_CAPTURE_PATHS = CAPTURE_PATHS - STAGING_PATHS
DURABLE_GENERATED_PATHS = GENERATED_PATHS - STAGING_PATHS
EXPECTED_FINAL_PATHS = (
    DURABLE_CAPTURE_PATHS | DURABLE_STILL_PATHS | DURABLE_GENERATED_PATHS
)
WORKING_PATHS = CAPTURE_PATHS | DURABLE_STILL_PATHS | GENERATED_PATHS
WORKING_DIRECTORIES = {
    "backspin",
    "backspin/frames",
    "backspin/panel_frames",
    "backspin/stills",
    "traces",
}
FINAL_DIRECTORIES = {"backspin", "backspin/stills", "traces"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_sha256(payload: Any) -> str:
    encoded = json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    def reject_nonfinite(value: str) -> None:
        raise ValueError(f"{path}: non-finite JSON number {value}")

    payload = json.loads(
        path.read_text(encoding="utf-8"), parse_constant=reject_nonfinite
    )
    if not isinstance(payload, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return payload


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _require_file(path: Path, label: str) -> Path:
    path = path.resolve()
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"{label} is not a regular file: {path}")
    return path


def _require_bundle_root(path: Path, *, create: bool) -> Path:
    original = Path(path)
    absolute = Path(os.path.abspath(original))
    if original.is_symlink():
        raise ValueError(f"bundle root is a symlink: {original}")
    for ancestor in absolute.parents:
        if ancestor.is_symlink():
            raise ValueError(f"bundle root passes through a symlink: {original}")
    if original.exists():
        if not original.is_dir():
            raise ValueError(f"bundle is not a regular directory: {original}")
    elif create:
        original.mkdir(parents=True)
    else:
        raise ValueError(f"bundle directory does not exist: {original}")
    if original.is_symlink():
        raise ValueError(f"bundle root became a symlink: {original}")
    resolved = original.resolve(strict=True)
    if resolved != absolute:
        raise ValueError(f"bundle root passes through a symlink: {original}")
    return resolved


def _validate_bundle_paths(
    root: Path,
    *,
    complete: bool,
    allow_missing_capture_provenance: bool = False,
    allow_missing_capture: bool = False,
    allow_missing_manual: bool = False,
) -> None:
    root = _require_bundle_root(root, create=False)

    files: set[str] = set()
    directories: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise ValueError(f"bundle contains a symlink: {relative}")
        if path.is_dir():
            directories.add(relative)
        elif path.is_file():
            files.add(relative)
        else:
            raise ValueError(f"bundle contains a non-regular entry: {relative}")

    allowed_files = EXPECTED_FINAL_PATHS if complete else WORKING_PATHS
    allowed_directories = FINAL_DIRECTORIES if complete else WORKING_DIRECTORIES
    unexpected_files = files - allowed_files
    unexpected_directories = directories - allowed_directories
    if unexpected_files or unexpected_directories:
        raise ValueError(
            "backspin bundle membership has unexpected entries: "
            f"files={sorted(unexpected_files)}, "
            f"directories={sorted(unexpected_directories)}"
        )
    if complete:
        missing = EXPECTED_FINAL_PATHS - files
        if missing:
            raise ValueError(
                f"backspin finalized bundle is incomplete: {sorted(missing)}"
            )
        missing_directories = FINAL_DIRECTORIES - directories
        if missing_directories:
            raise ValueError(
                "backspin finalized bundle directories are incomplete: "
                f"{sorted(missing_directories)}"
            )
    else:
        missing_capture = CAPTURE_PATHS - files
        if allow_missing_capture_provenance:
            missing_capture -= {"capture-provenance.json"}
        if allow_missing_manual:
            missing_capture -= {"manual-inspection.json"}
        if missing_capture and not allow_missing_capture:
            raise ValueError(
                f"backspin capture bundle is incomplete: {sorted(missing_capture)}"
            )


def _int_field(row: dict[str, str], field: str, *, line: int) -> int:
    value = row[field]
    try:
        parsed = int(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(
            f"trace line {line}: invalid integer {field}={value!r}"
        ) from error
    if str(parsed) != value:
        raise ValueError(f"trace line {line}: noncanonical integer {field}={value!r}")
    return parsed


def _float_field(
    row: dict[str, str], field: str, *, line: int, allow_nan: bool = False
) -> float:
    value = row[field]
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(
            f"trace line {line}: invalid float {field}={value!r}"
        ) from error
    if math.isnan(parsed) and allow_nan:
        return parsed
    if not math.isfinite(parsed):
        raise ValueError(f"trace line {line}: nonfinite {field}={value!r}")
    return parsed


def parse_trace_text(text: str) -> dict[str, Any]:
    reader = csv.DictReader(io.StringIO(text))
    if tuple(reader.fieldnames or ()) != TRACE_COLUMNS:
        raise ValueError(
            f"unexpected trace columns: {reader.fieldnames!r}; expected {TRACE_COLUMNS!r}"
        )
    raw_rows = list(reader)
    if len(raw_rows) != TOTAL_STEPS + 1:
        raise ValueError(
            f"backspin: expected {TOTAL_STEPS + 1} rows including step 0, "
            f"got {len(raw_rows)}"
        )

    rows: list[dict[str, Any]] = []
    cumulative_fields = ("exact_solves", "warm_starts", "fallbacks")
    previous_cumulative = {field: 0 for field in cumulative_fields}
    max_residual = 0.0
    for index, raw in enumerate(raw_rows):
        line = index + 2
        if None in raw or set(raw) != set(TRACE_COLUMNS):
            raise ValueError(f"trace line {line}: malformed CSV row")
        step = _int_field(raw, "step", line=line)
        if step != index:
            raise ValueError(f"backspin: noncontiguous step at row {index}: {step}")
        time_seconds = _float_field(raw, "time", line=line)
        if not math.isclose(
            time_seconds, step * DT_SECONDS, rel_tol=0.0, abs_tol=2.0e-14
        ):
            raise ValueError(
                f"backspin: step {step} time {time_seconds} is not step/60"
            )
        if raw["scenario"] != "backspin":
            raise ValueError(
                f"trace line {line}: scenario {raw['scenario']!r} != 'backspin'"
            )
        if raw["solver"] != "exact_fbf" or raw["body"] != "backspin_sphere_body":
            raise ValueError(f"trace line {line}: solver/body identity mismatch")

        typed: dict[str, Any] = {
            "step": step,
            "time": time_seconds,
            "scenario": raw["scenario"],
            "solver": raw["solver"],
            "body": raw["body"],
            "status": raw["status"],
        }
        for field in ("x", "y", "z", "vx", "vy", "vz", "up_z"):
            typed[field] = _float_field(raw, field, line=line)
        for field in ("contacts", *cumulative_fields):
            typed[field] = _int_field(raw, field, line=line)
            if typed[field] < 0:
                raise ValueError(f"trace line {line}: negative {field}")
        for field in cumulative_fields:
            if typed[field] < previous_cumulative[field]:
                raise ValueError(f"trace line {line}: decreasing cumulative {field}")
            previous_cumulative[field] = typed[field]

        if step == 0:
            residual = _float_field(raw, "residual", line=line, allow_nan=True)
            if not math.isnan(residual) or typed["status"] != "not_run":
                raise ValueError("backspin: step 0 must be nan/not_run")
            if any(typed[field] != 0 for field in cumulative_fields):
                raise ValueError("backspin: step 0 cumulative counters must be zero")
        else:
            residual = _float_field(raw, "residual", line=line)
            if typed["status"] != "success":
                raise ValueError(f"backspin: step {step} status {typed['status']!r}")
            if residual > RESIDUAL_TOLERANCE:
                raise ValueError(
                    f"backspin: step {step} residual {residual} exceeds "
                    f"{RESIDUAL_TOLERANCE}"
                )
            max_residual = max(max_residual, residual)
        typed["residual"] = residual
        rows.append(typed)

    if any(row["fallbacks"] != 0 for row in rows):
        raise ValueError("backspin: boxed-LCP fallback observed")
    if rows[-1]["exact_solves"] <= 0:
        raise ValueError("backspin: no exact-FBF solve was recorded")
    if not any(row["contacts"] > 0 for row in rows[1:]):
        raise ValueError("backspin: no contact was recorded")

    initial = rows[0]
    final = rows[-1]
    maximum = max(rows, key=lambda row: row["x"])
    reversal = next((row for row in rows[1:] if row["vx"] < 0.0), None)
    if initial["vx"] <= 0.0:
        raise ValueError("backspin: initial translational velocity is not positive")
    if maximum["x"] <= initial["x"] or maximum["step"] == 0:
        raise ValueError("backspin: positive translational advance was not recorded")
    if reversal is None:
        raise ValueError("backspin: translational velocity never reversed")
    if reversal["step"] <= maximum["step"]:
        raise ValueError("backspin: reversal does not follow the forward maximum")
    if final["vx"] >= 0.0 or final["x"] >= initial["x"]:
        raise ValueError("backspin: final state did not travel backward past its start")
    if final["contacts"] <= 0:
        raise ValueError("backspin: final state is not in contact")

    selected = []
    for step in PANEL_STEPS:
        row = rows[step]
        state = {
            key: row[key]
            for key in (
                "step",
                "time",
                "x",
                "z",
                "vx",
                "vz",
                "up_z",
                "contacts",
                "exact_solves",
                "warm_starts",
                "fallbacks",
                "residual",
                "status",
            )
        }
        if not math.isfinite(state["residual"]):
            state["residual"] = None
        selected.append(state)
    projection = [
        {
            "step": row["step"],
            "contacts": row["contacts"],
            "exact_solves": row["exact_solves"],
            "warm_starts": row["warm_starts"],
            "boxed_lcp_fallbacks": row["fallbacks"],
            "status": row["status"],
        }
        for row in rows
    ]
    contact_free_steps = [row["step"] for row in rows[1:] if row["contacts"] == 0]
    summary = {
        "scenario": "backspin",
        "row_count": len(rows),
        "completed_steps": TOTAL_STEPS,
        "exact_solves": final["exact_solves"],
        "warm_starts": final["warm_starts"],
        "boxed_lcp_fallbacks": final["fallbacks"],
        "max_residual": max_residual,
        "initial": {
            key: initial[key] for key in ("step", "time", "x", "z", "vx", "up_z")
        },
        "maximum_forward_travel": {
            key: maximum[key] for key in ("step", "time", "x", "vx", "contacts")
        },
        "first_negative_vx": {
            key: reversal[key] for key in ("step", "time", "x", "vx", "contacts")
        },
        "final": {
            key: final[key]
            for key in (
                "step",
                "time",
                "x",
                "z",
                "vx",
                "vz",
                "up_z",
                "contacts",
            )
        },
        "selected_panel_states": selected,
        "contact_free_post_initial_steps": contact_free_steps,
        "continuous_contact_proven": False,
        "strict_rigid_body_rest_proven": False,
        "signed_angular_direction_proven": False,
        "angular_velocity_exported": False,
        "physical_outcome_valid": True,
        "claim_scope": "DART translational advance and reversal through 2.17 seconds",
    }
    return {
        "summary": summary,
        "solver_projection": projection,
        "solver_projection_sha256": _payload_sha256(projection),
    }


def artifact_index(root: Path) -> dict[str, Any]:
    root = root.resolve()
    artifacts = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"artifact bundle contains a symlink: {path}")
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if relative in INDEX_EXCLUSIONS:
            continue
        artifacts.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    return {
        "schema_version": INDEX_SCHEMA_VERSION,
        "artifact_count": len(artifacts),
        "excluded": sorted(INDEX_EXCLUSIONS),
        "artifacts": artifacts,
    }


def validate_artifact_index(root: Path, payload: dict[str, Any]) -> None:
    root = root.resolve()
    if payload.get("schema_version") != INDEX_SCHEMA_VERSION:
        raise ValueError("unexpected backspin artifact-index schema")
    if payload.get("excluded") != sorted(INDEX_EXCLUSIONS):
        raise ValueError("backspin artifact-index exclusions changed")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("backspin artifact-index artifacts must be a list")
    if payload.get("artifact_count") != len(artifacts):
        raise ValueError("backspin artifact-index count mismatch")

    listed = []
    for item in artifacts:
        if not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("malformed backspin artifact-index entry")
        relative = item["path"]
        if (
            not isinstance(relative, str)
            or relative.startswith(("/", "../"))
            or "/../" in relative
        ):
            raise ValueError(f"unsafe backspin artifact-index path: {relative!r}")
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"backspin artifact-index file missing: {relative}")
        if path.stat().st_size != item["bytes"]:
            raise ValueError(f"backspin artifact-index byte mismatch: {relative}")
        if sha256(path) != item["sha256"]:
            raise ValueError(f"backspin artifact-index hash mismatch: {relative}")
        listed.append(relative)
    if listed != sorted(set(listed)):
        raise ValueError("backspin artifact-index paths are duplicate or unsorted")

    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.relative_to(root).as_posix() not in INDEX_EXCLUSIONS
    }
    if actual != set(listed):
        raise ValueError(
            "backspin artifact-index membership mismatch: "
            f"missing={sorted(actual - set(listed))}, "
            f"extra={sorted(set(listed) - actual)}"
        )


def validate_manual_inspection(root: Path) -> dict[str, Any]:
    root = root.resolve()
    path = root / "manual-inspection.json"
    record = read_json(path)
    if set(record) != {
        "schema_version",
        "manual_inspected",
        "pass",
        "verdicts",
        "representative_artifacts",
    }:
        raise ValueError("backspin manual-inspection top-level contract changed")
    if record.get("schema_version") != MANUAL_SCHEMA_VERSION:
        raise ValueError("unexpected backspin manual-inspection schema")
    if record.get("pass") is not True or record.get("manual_inspected") is not True:
        raise ValueError("backspin manual inspection is not passing")
    if record.get("verdicts") != MANUAL_VERDICTS:
        raise ValueError("backspin manual-inspection verdict contract changed")

    artifacts = record.get("representative_artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("backspin manual-inspection artifacts must be a list")
    if any(
        not isinstance(item, dict) or set(item) != {"path", "sha256", "observation"}
        for item in artifacts
    ):
        raise ValueError("malformed backspin manual-inspection artifact")
    paths = [item["path"] for item in artifacts]
    if set(paths) != MANUAL_ARTIFACT_PATHS or len(paths) != len(set(paths)):
        raise ValueError("backspin manual-inspection artifact paths changed")
    for item in artifacts:
        relative = item["path"]
        artifact = root / relative
        if (
            not artifact.is_file()
            or artifact.is_symlink()
            or sha256(artifact) != item["sha256"]
        ):
            raise ValueError(f"backspin manual-inspection artifact changed: {relative}")
        observation = item["observation"]
        if not isinstance(observation, str) or not observation.strip():
            raise ValueError(
                f"backspin manual-inspection observation missing: {relative}"
            )
    return record


def _capture_result_by_id(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    results = payload.get("results")
    if not isinstance(results, list):
        raise ValueError("capture results must be a list")
    by_id = {}
    for result in results:
        if not isinstance(result, dict):
            raise ValueError("capture result must be an object")
        schedule = result.get("schedule")
        schedule_id = (
            schedule.get("id") if isinstance(schedule, dict) else result.get("schedule")
        )
        if not isinstance(schedule_id, str) or schedule_id in by_id:
            raise ValueError("capture result identity is missing or duplicated")
        by_id[schedule_id] = result
    return by_id


def _capture_still_bindings(root: Path) -> list[dict[str, Any]]:
    """Recompute the selected still hashes from both stored capture records."""

    root = root.resolve()
    metadata = read_json(root / "backspin/metadata.json")
    run_results = _capture_result_by_id(read_json(root / "run-summary.json"))
    if set(run_results) != {"backspin"} or run_results["backspin"] != metadata:
        raise ValueError("backspin durable still run-summary binding changed")
    metadata_timeline = metadata.get("timeline_validation")
    run_timeline = run_results["backspin"].get("timeline_validation")
    if not isinstance(metadata_timeline, dict) or not isinstance(run_timeline, dict):
        raise ValueError("backspin durable still timeline validation is missing")
    metadata_frames = metadata_timeline.get("frames")
    run_frames = run_timeline.get("frames")
    if not isinstance(metadata_frames, dict) or not isinstance(run_frames, dict):
        raise ValueError("backspin durable still timeline frames are missing")

    sidecar = read_json(root / "backspin/timeline.json")
    shots = sidecar.get("shots")
    if not isinstance(shots, list):
        raise ValueError("backspin durable still timeline shots are missing")
    shots_by_step: dict[int, dict[str, Any]] = {}
    for shot in shots:
        if (
            not isinstance(shot, dict)
            or not isinstance(shot.get("step"), int)
            or isinstance(shot.get("step"), bool)
        ):
            raise ValueError("backspin durable still timeline shot is malformed")
        step = shot["step"]
        if step in shots_by_step:
            raise ValueError("backspin durable still timeline steps are duplicated")
        shots_by_step[step] = shot

    bindings = []
    for step in DURABLE_STILL_STEPS:
        source_relative = f"backspin/frames/step_{step:06d}.png"
        source_path = root / source_relative
        still_relative = f"backspin/stills/step_{step:06d}.png"
        metadata_frame = metadata_frames.get(str(step))
        run_frame = run_frames.get(str(step))
        shot = shots_by_step.get(step)
        if (
            not isinstance(metadata_frame, dict)
            or not isinstance(run_frame, dict)
            or not isinstance(shot, dict)
            or metadata_frame.get("path") != str(source_path)
            or run_frame.get("path") != str(source_path)
            or shot.get("path") != str(source_path)
            or metadata_frame.get("sha256") != run_frame.get("sha256")
            or not isinstance(metadata_frame.get("sha256"), str)
            or len(metadata_frame["sha256"]) != 64
        ):
            raise ValueError(
                f"backspin durable still capture binding changed at step {step}"
            )
        bindings.append(
            {
                "step": step,
                "path": still_relative,
                "source_frame_path": source_relative,
                "timeline_frame_sha256": metadata_frame["sha256"],
                "run_summary_frame_sha256": run_frame["sha256"],
            }
        )
    return bindings


def _materialize_durable_stills(root: Path) -> list[dict[str, Any]]:
    root = root.resolve()
    records = []
    for binding in _capture_still_bindings(root):
        source = root / binding["source_frame_path"]
        destination = root / binding["path"]
        if not source.is_file() or source.is_symlink():
            raise ValueError(
                f"backspin durable still source is missing: {binding['source_frame_path']}"
            )
        if sha256(source) != binding["timeline_frame_sha256"]:
            raise ValueError(
                "backspin durable still source hash changed: "
                f"{binding['source_frame_path']}"
            )
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            raise ValueError(f"backspin durable still is a symlink: {binding['path']}")
        shutil.copyfile(source, destination)
        records.append({**binding, "sha256": sha256(destination)})
    return records


def _validate_durable_stills(root: Path, records: Any) -> list[dict[str, Any]]:
    root = root.resolve()
    expected = []
    for binding in _capture_still_bindings(root):
        still = root / binding["path"]
        if not still.is_file() or still.is_symlink():
            raise ValueError(f"backspin durable still is missing: {binding['path']}")
        still_sha = sha256(still)
        if (
            still_sha != binding["timeline_frame_sha256"]
            or still_sha != binding["run_summary_frame_sha256"]
        ):
            raise ValueError(
                f"backspin durable still hash changed at step {binding['step']}"
            )
        expected.append({**binding, "sha256": still_sha})
    if records != expected:
        raise ValueError("backspin durable still provenance changed")
    return expected


def _build_staging_manifest(root: Path) -> dict[str, Any]:
    root = root.resolve()
    artifacts = []
    for relative in sorted(STAGING_PATHS):
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"backspin staging artifact is missing: {relative}")
        artifacts.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    return {
        "schema_version": STAGING_SCHEMA_VERSION,
        "disposition": "pruned_before_seal",
        "artifact_count": len(artifacts),
        "artifacts": artifacts,
    }


def _validate_staging_manifest(
    root: Path, payload: Any, *, require_absent: bool
) -> dict[str, dict[str, Any]]:
    root = root.resolve()
    if not isinstance(payload, dict) or set(payload) != {
        "schema_version",
        "disposition",
        "artifact_count",
        "artifacts",
    }:
        raise ValueError("malformed backspin pruned-staging provenance")
    if (
        payload.get("schema_version") != STAGING_SCHEMA_VERSION
        or payload.get("disposition") != "pruned_before_seal"
    ):
        raise ValueError("backspin pruned-staging contract changed")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list) or payload.get("artifact_count") != len(
        artifacts
    ):
        raise ValueError("backspin pruned-staging count changed")
    expected_paths = sorted(STAGING_PATHS)
    paths = [item.get("path") for item in artifacts if isinstance(item, dict)]
    if paths != expected_paths or len(paths) != len(artifacts):
        raise ValueError("backspin pruned-staging membership changed")
    by_path = {}
    for item in artifacts:
        if set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("malformed backspin pruned-staging artifact")
        relative = item["path"]
        if (
            not isinstance(item["bytes"], int)
            or isinstance(item["bytes"], bool)
            or item["bytes"] < 0
            or not isinstance(item["sha256"], str)
            or len(item["sha256"]) != 64
            or any(character not in "0123456789abcdef" for character in item["sha256"])
        ):
            raise ValueError(f"malformed backspin pruned-staging identity: {relative}")
        path = root / relative
        if require_absent:
            if path.exists() or path.is_symlink():
                raise ValueError(f"backspin staging artifact survived seal: {relative}")
        elif (
            not path.is_file()
            or path.is_symlink()
            or path.stat().st_size != item["bytes"]
            or sha256(path) != item["sha256"]
        ):
            raise ValueError(f"backspin staging artifact changed: {relative}")
        by_path[relative] = item
    return by_path


def _prune_staging_outputs(root: Path, manifest: dict[str, Any]) -> None:
    root = root.resolve()
    _validate_staging_manifest(root, manifest, require_absent=False)
    for relative in sorted(STAGING_PATHS):
        (root / relative).unlink()
    for relative in ("backspin/panel_frames", "backspin/frames"):
        (root / relative).rmdir()
    _validate_staging_manifest(root, manifest, require_absent=True)


def _capture_solver_projection(timeline: dict[str, Any]) -> list[dict[str, Any]]:
    steps = timeline.get("steps")
    if not isinstance(steps, dict):
        raise ValueError("backspin capture timeline steps are unavailable")
    projection = []
    for step in range(TOTAL_STEPS + 1):
        entry = steps.get(str(step))
        if not isinstance(entry, dict):
            raise ValueError(f"backspin capture timeline step {step} is missing")
        diagnostics = entry.get("solver_diagnostics")
        if not isinstance(diagnostics, dict):
            raise ValueError(f"backspin capture diagnostics missing at step {step}")
        projection.append(
            {
                "step": step,
                "contacts": diagnostics.get("contacts"),
                "exact_solves": diagnostics.get("exact_solves"),
                "warm_starts": diagnostics.get("warm_starts"),
                "boxed_lcp_fallbacks": diagnostics.get("boxed_lcp_fallbacks"),
                "status": diagnostics.get("status"),
            }
        )
    return projection


def _compare_solver_projections(
    trace_projection: Sequence[dict[str, Any]],
    capture_projection: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    trace_projection = list(trace_projection)
    capture_projection = list(capture_projection)
    if trace_projection != capture_projection:
        mismatch = next(
            (
                index
                for index, (trace, capture) in enumerate(
                    zip(trace_projection, capture_projection)
                )
                if trace != capture
            ),
            min(len(trace_projection), len(capture_projection)),
        )
        raise ValueError(
            f"backspin trace/capture solver projection differs at {mismatch}"
        )
    digest = _payload_sha256(trace_projection)
    return {
        "pass": True,
        "row_count": len(trace_projection),
        "fields": [
            "step",
            "contacts",
            "exact_solves",
            "warm_starts",
            "boxed_lcp_fallbacks",
            "status",
        ],
        "trace_sha256": digest,
        "capture_sha256": digest,
        "byte_identical": True,
        "full_state_trace_equivalence": False,
        "limitation": (
            "This compares discrete solver/contact diagnostics only. The rendered "
            "demo and CSV exporter are separate scene implementations, and neither "
            "projection includes angular velocity or a complete state trajectory."
        ),
    }


def _validate_media_item(
    root: Path,
    stored: dict[str, Any],
    verified: dict[str, Any],
    *,
    kind: str,
    expected_stream: dict[str, Any],
) -> dict[str, Any]:
    path = root / "backspin" / f"clip.{kind}"
    expected_path = str(path)
    for item, label in ((stored, "stored"), (verified, "verified")):
        if (
            item.get("kind") != kind
            or item.get("path") != expected_path
            or item.get("sha256") != sha256(path)
            or item.get("full_decode") != "pass"
            or item.get("stream_contract") != expected_stream
        ):
            raise ValueError(f"backspin {kind} {label} contract changed")
    return {
        "path": expected_path,
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
        "full_decode": "pass",
        "stream_contract": expected_stream,
    }


def validate_capture_bundle(
    root: Path,
    *,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
    verification: dict[str, Any] | None = None,
    staging_manifest: dict[str, Any] | None = None,
) -> dict[str, Any]:
    root = root.resolve()
    demo = _require_file(demo, "DART demo binary")
    ffmpeg = _require_file(ffmpeg, "ffmpeg")
    ffprobe = _require_file(ffprobe, "ffprobe")
    staging_by_path = (
        _validate_staging_manifest(root, staging_manifest, require_absent=True)
        if staging_manifest is not None
        else None
    )
    verification = verification or read_json(root / "verification.json")
    if (
        verification.get("schema_version") != RUNNER_SCHEMA_VERSION
        or verification.get("kind") != "verification"
        or verification.get("pass") is not True
        or verification.get("group_outputs") != []
    ):
        raise ValueError("backspin capture verification is not a passing singleton")
    verify_results = _capture_result_by_id(verification)
    if set(verify_results) != {"backspin"}:
        raise ValueError("backspin capture verification member set changed")

    run_summary = read_json(root / "run-summary.json")
    if (
        run_summary.get("schema_version") != RUNNER_SCHEMA_VERSION
        or run_summary.get("kind") != "capture_run"
        or run_summary.get("pass") is not True
        or run_summary.get("failures") != []
        or run_summary.get("group_outputs") != []
        or run_summary.get("group_skips") != []
    ):
        raise ValueError("backspin run-summary is not a complete passing singleton")
    run_results = _capture_result_by_id(run_summary)
    if set(run_results) != {"backspin"}:
        raise ValueError("backspin run-summary member set changed")

    metadata_path = root / "backspin/metadata.json"
    metadata = read_json(metadata_path)
    if run_results["backspin"] != metadata:
        raise ValueError("backspin run-summary no longer equals metadata.json")
    if (
        metadata.get("schema_version") != RUNNER_SCHEMA_VERSION
        or metadata.get("kind") != "capture_result"
        or metadata.get("pass") is not True
        or metadata.get("actual_simulator") is not True
        or metadata.get("generated_imagery") is not False
        or metadata.get("paper_comparable") is not False
        or metadata.get("automated_semantic_outcome_validated") is not False
    ):
        raise ValueError("backspin capture claim flags changed")
    if metadata.get("known_mismatches") != metadata.get("schedule", {}).get(
        "known_mismatches"
    ):
        raise ValueError("backspin capture mismatch disclosures changed")
    if not any(
        "checker-textured UV mesh is VisualAspect-only" in mismatch
        for mismatch in metadata.get("known_mismatches", [])
    ):
        raise ValueError("backspin visual-only checker disclosure is missing")

    schedule = metadata.get("schedule")
    if not isinstance(schedule, dict):
        raise ValueError("backspin capture schedule is missing")
    expected_output = {
        "directory": str(root / "backspin"),
        "timeline": str(root / "backspin/timeline.json"),
        "panel": str(root / "backspin/panel.png"),
        "mp4": str(root / "backspin/clip.mp4"),
        "gif": str(root / "backspin/clip.gif"),
    }
    if (
        schedule.get("id") != "backspin"
        or schedule.get("scene") != "fbf_paper_backspin"
        or schedule.get("source_segment") != "backspin"
        or schedule.get("total_steps") != TOTAL_STEPS
        or schedule.get("time_step_seconds") != DT_SECONDS
        or schedule.get("capture_steps") != list(range(TOTAL_STEPS + 1))
        or schedule.get("panel_steps") != list(PANEL_STEPS)
        or schedule.get("panel_labels") != list(PANEL_LABELS)
        or schedule.get("actions") != []
        or schedule.get("configuration") != EXPECTED_CONFIGURATION
        or schedule.get("output") != expected_output
        or schedule.get("runnable") is not True
        or schedule.get("adapter_gap") is not None
        or schedule.get("long_run") is not False
        or schedule.get("actual_simulator_required") is not True
        or schedule.get("generated_imagery_allowed") is not False
        or schedule.get("paper_comparable") is not False
        or schedule.get("known_gate_blockers") != []
        or schedule.get("evidence_ready") is not True
    ):
        raise ValueError("backspin capture schedule contract changed")

    runtime = metadata.get("runtime")
    if (
        not isinstance(runtime, dict)
        or runtime.get("demo_path") != str(demo)
        or runtime.get("demo_sha256") != sha256(demo)
        or runtime.get("demo_argv") != schedule.get("demo_argv")
        or runtime.get("ffmpeg") != str(ffmpeg)
        or runtime.get("ffprobe") != str(ffprobe)
    ):
        raise ValueError("backspin capture runtime identity changed")

    verified = verify_results["backspin"]
    if (
        verified.get("pass") is not True
        or verified.get("metadata_path") != str(metadata_path)
        or verified.get("metadata_sha256") != sha256(metadata_path)
    ):
        raise ValueError("backspin verification metadata binding changed")
    timeline = metadata.get("timeline_validation")
    if not isinstance(timeline, dict) or verified.get("timeline") != timeline:
        raise ValueError("backspin stored/fresh timeline validation differs")
    expected_steps = {str(step) for step in range(TOTAL_STEPS + 1)}
    if (
        timeline.get("pass") is not True
        or timeline.get("step_count") != TOTAL_STEPS + 1
        or timeline.get("shot_count") != TOTAL_STEPS + 1
        or timeline.get("action_count") != 0
        or timeline.get("unique_frame_hashes") != TOTAL_STEPS + 1
        or set(timeline.get("steps", {})) != expected_steps
        or set(timeline.get("frames", {})) != expected_steps
    ):
        raise ValueError("backspin capture timeline contract changed")
    for step in range(TOTAL_STEPS + 1):
        frame_path = root / "backspin/frames" / f"step_{step:06d}.png"
        frame_relative = frame_path.relative_to(root).as_posix()
        expected_frame_sha = (
            staging_by_path[frame_relative]["sha256"]
            if staging_by_path is not None
            else sha256(frame_path)
        )
        frame = timeline["frames"][str(step)]
        if (
            frame.get("path") != str(frame_path)
            or frame.get("sha256") != expected_frame_sha
        ):
            raise ValueError(f"backspin capture frame binding changed at step {step}")
    world_hashes = {
        timeline["frames"][str(step)].get("world_viewport", {}).get("sha256")
        for step in range(TOTAL_STEPS + 1)
    }
    if None in world_hashes or len(world_hashes) != TOTAL_STEPS + 1:
        raise ValueError("backspin capture world frames are not all distinct")

    final_diagnostics = timeline.get("final_solver_diagnostics")
    if (
        not isinstance(final_diagnostics, dict)
        or final_diagnostics.get("available") is not True
        or final_diagnostics.get("exact_attempts", 0) <= 0
        or final_diagnostics.get("exact_solves", 0) <= 0
        or final_diagnostics.get("accepted_at_cap") != 0
        or final_diagnostics.get("exact_failures") != 0
        or final_diagnostics.get("boxed_lcp_fallbacks") != 0
        or final_diagnostics.get("status") != "success"
        or not math.isfinite(float(final_diagnostics.get("worst_residual", math.nan)))
        or float(final_diagnostics["worst_residual"]) > RESIDUAL_TOLERANCE
    ):
        raise ValueError("backspin capture solver diagnostics changed")

    panel = metadata.get("panel_validation")
    panel_path = root / "backspin/panel.png"
    if not isinstance(panel, dict):
        raise ValueError("backspin panel validation is missing")
    expected_panel_sources = []
    for step in PANEL_STEPS:
        source = root / "backspin/panel_frames" / f"step_{step:06d}.png"
        relative = source.relative_to(root).as_posix()
        expected_panel_sources.append(
            {
                "path": str(source),
                "sha256": (
                    staging_by_path[relative]["sha256"]
                    if staging_by_path is not None
                    else sha256(source)
                ),
            }
        )
    compose_path = root / "backspin/panel.compose.json"
    if (
        panel.get("path") != str(panel_path)
        or panel.get("sha256") != sha256(panel_path)
        or panel.get("source_frames") != expected_panel_sources
        or panel.get("compose_manifest_path") != str(compose_path)
        or panel.get("compose_manifest_sha256") != sha256(compose_path)
        or panel.get("compose_manifest") != read_json(compose_path)
        or panel.get("verdict", {}).get("pass") is not True
        or panel.get("verdict", {}).get("image", {}).get("width") != 6548
        or panel.get("verdict", {}).get("image", {}).get("height") != 538
        or verified.get("panel", {}).get("pass") is not True
        or verified.get("panel", {}).get("image", {}).get("path") != str(panel_path)
    ):
        raise ValueError("backspin panel contract changed")

    stored_media = {
        item.get("kind"): item for item in metadata.get("media_validation", [])
    }
    verified_media = {item.get("kind"): item for item in verified.get("media", [])}
    if set(stored_media) != {"mp4", "gif"} or set(verified_media) != {
        "mp4",
        "gif",
    }:
        raise ValueError("backspin media member set changed")
    media = {
        "mp4": _validate_media_item(
            root,
            stored_media["mp4"],
            verified_media["mp4"],
            kind="mp4",
            expected_stream={
                "width": 1300,
                "height": 506,
                "frame_rate": "30/1",
                "frame_rate_rational": "30/1",
                "frame_count": 66,
            },
        ),
        "gif": _validate_media_item(
            root,
            stored_media["gif"],
            verified_media["gif"],
            kind="gif",
            expected_stream={
                "width": 960,
                "height": 374,
                "frame_rate": "15/1",
                "frame_rate_rational": "15/1",
                "frame_count": 33,
            },
        ),
    }

    projection = _capture_solver_projection(timeline)
    contact_free_steps = [
        item["step"] for item in projection[1:] if item.get("contacts") == 0
    ]
    return {
        "pass": True,
        "metadata_sha256": sha256(metadata_path),
        "timeline_sha256": sha256(root / "backspin/timeline.json"),
        "panel_sha256": sha256(panel_path),
        "captured_frames": TOTAL_STEPS + 1,
        "unique_frames": TOTAL_STEPS + 1,
        "panel_steps": list(PANEL_STEPS),
        "exact_attempts": final_diagnostics["exact_attempts"],
        "exact_solves": final_diagnostics["exact_solves"],
        "accepted_at_cap": final_diagnostics["accepted_at_cap"],
        "exact_failures": final_diagnostics["exact_failures"],
        "boxed_lcp_fallbacks": final_diagnostics["boxed_lcp_fallbacks"],
        "worst_residual": final_diagnostics["worst_residual"],
        "contact_free_post_initial_steps": contact_free_steps,
        "continuous_contact_proven": False,
        "media": media,
        "run_summary_sha256": sha256(root / "run-summary.json"),
        "verification_sha256": sha256(root / "verification.json"),
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "paper_comparable": False,
        "solver_projection": projection,
        "solver_projection_sha256": _payload_sha256(projection),
    }


def _run_command(
    argv: Sequence[str], *, timeout: float
) -> subprocess.CompletedProcess[str]:
    process = subprocess.run(
        [str(item) for item in argv],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    if process.returncode != 0:
        raise ValueError(
            f"command failed with exit {process.returncode}: {list(argv)!r}\n"
            f"stderr:\n{process.stderr}"
        )
    return process


def _trace_argv(trace_binary: Path) -> list[str]:
    return [
        str(trace_binary),
        "backspin",
        "exact_fbf",
        "1",
        str(TOTAL_STEPS),
        "nan",
        "tracked",
        "default",
        "default",
        "1",
        "dart_best",
        "dart",
        "default",
        "0",
        "0",
        "default",
    ]


def _verification_argv(
    python: Path,
    runner: Path,
    demo: Path,
    bundle: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> list[str]:
    return [
        str(python),
        str(runner),
        "verify",
        "--scenario",
        "backspin",
        "--demo",
        str(demo),
        "--output-root",
        str(bundle),
        "--ffmpeg",
        str(ffmpeg),
        "--ffprobe",
        str(ffprobe),
    ]


def _capture_argv(
    python: Path,
    runner: Path,
    demo: Path,
    bundle: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> list[str]:
    return [
        str(python),
        str(runner),
        "run",
        "--scenario",
        "backspin",
        "--demo",
        str(demo),
        "--output-root",
        str(bundle),
        "--ffmpeg",
        str(ffmpeg),
        "--ffprobe",
        str(ffprobe),
        "--python",
        str(python),
        "--out",
        str(bundle / "run-summary.json"),
    ]


def _checker_runtime_resource_identity() -> dict[str, dict[str, str]]:
    return {
        name: {"path": str(_require_file(path, name)), "sha256": sha256(path)}
        for name, path in {
            "backspin_checker_mesh": CHECKER_MESH,
            "backspin_checker_material": CHECKER_MATERIAL,
            "backspin_checker_texture": CHECKER_TEXTURE,
        }.items()
    }


def _validate_checker_asset_structure(mesh: str, texture: str) -> dict[str, Any]:
    vertices: list[tuple[float, float, float]] = []
    texture_coordinates: list[tuple[float, float]] = []
    normals: list[tuple[float, float, float]] = []
    faces: list[tuple[tuple[int, int, int], ...]] = []

    for line_number, raw_line in enumerate(mesh.splitlines(), start=1):
        fields = raw_line.split()
        if not fields or fields[0].startswith("#"):
            continue
        tag = fields[0]
        try:
            if tag == "v":
                if len(fields) != 4:
                    raise ValueError
                vertices.append(tuple(float(value) for value in fields[1:]))
            elif tag == "vt":
                if len(fields) != 3:
                    raise ValueError
                texture_coordinates.append(tuple(float(value) for value in fields[1:]))
            elif tag == "vn":
                if len(fields) != 4:
                    raise ValueError
                normals.append(tuple(float(value) for value in fields[1:]))
            elif tag == "f":
                if len(fields) != 4:
                    raise ValueError
                corners: list[tuple[int, int, int]] = []
                for token in fields[1:]:
                    indices = token.split("/")
                    if len(indices) != 3 or any(not index for index in indices):
                        raise ValueError
                    corners.append(tuple(int(index) for index in indices))
                faces.append(tuple(corners))
        except ValueError as error:
            raise ValueError(
                f"backspin checker OBJ record is invalid at line {line_number}"
            ) from error

    counts = (
        len(vertices),
        len(texture_coordinates),
        len(normals),
        len(faces),
    )
    if counts != (559, 559, 559, 960):
        raise ValueError("backspin checker OBJ topology changed")

    for vertex, normal in zip(vertices, normals):
        if not all(math.isfinite(value) for value in (*vertex, *normal)):
            raise ValueError("backspin checker OBJ contains non-finite geometry")
        vertex_length = math.sqrt(sum(value * value for value in vertex))
        normal_length = math.sqrt(sum(value * value for value in normal))
        alignment = sum(a * b for a, b in zip(vertex, normal))
        if abs(vertex_length - 1.0) > 1.0e-6:
            raise ValueError("backspin checker OBJ radius changed")
        if abs(normal_length - 1.0) > 1.0e-6 or alignment < 1.0 - 1.0e-6:
            raise ValueError("backspin checker OBJ outward normals changed")

    if any(
        not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in uv)
        for uv in texture_coordinates
    ):
        raise ValueError("backspin checker OBJ UV bounds changed")
    u_values = [uv[0] for uv in texture_coordinates]
    v_values = [uv[1] for uv in texture_coordinates]
    if (
        min(u_values) != 0.0
        or max(u_values) != 1.0
        or min(v_values) != 0.0
        or max(v_values) != 1.0
    ):
        raise ValueError("backspin checker OBJ UV extent changed")

    used_vertices: set[int] = set()
    used_texture_coordinates: set[int] = set()
    used_normals: set[int] = set()
    for face in faces:
        for vertex_index, uv_index, normal_index in face:
            if (
                not 1 <= vertex_index <= len(vertices)
                or not 1 <= uv_index <= len(texture_coordinates)
                or not 1 <= normal_index <= len(normals)
            ):
                raise ValueError("backspin checker OBJ face index is out of range")
            if not vertex_index == uv_index == normal_index:
                raise ValueError("backspin checker OBJ face index mapping changed")
            used_vertices.add(vertex_index)
            used_texture_coordinates.add(uv_index)
            used_normals.add(normal_index)

        points = [vertices[corner[0] - 1] for corner in face]
        edge_ab = tuple(points[1][axis] - points[0][axis] for axis in range(3))
        edge_ac = tuple(points[2][axis] - points[0][axis] for axis in range(3))
        cross = (
            edge_ab[1] * edge_ac[2] - edge_ab[2] * edge_ac[1],
            edge_ab[2] * edge_ac[0] - edge_ab[0] * edge_ac[2],
            edge_ab[0] * edge_ac[1] - edge_ab[1] * edge_ac[0],
        )
        centroid_sum = tuple(sum(point[axis] for point in points) for axis in range(3))
        outward_measure = sum(a * b for a, b in zip(cross, centroid_sum))
        if outward_measure <= 1.0e-10:
            raise ValueError("backspin checker OBJ face winding is not outward")

        uvs = [texture_coordinates[corner[1] - 1] for corner in face]
        uv_area_twice = (uvs[1][0] - uvs[0][0]) * (uvs[2][1] - uvs[0][1]) - (
            uvs[1][1] - uvs[0][1]
        ) * (uvs[2][0] - uvs[0][0])
        if abs(uv_area_twice) <= 1.0e-12:
            raise ValueError("backspin checker OBJ has a degenerate UV triangle")

    expected_indices = set(range(1, 560))
    if (
        used_vertices != expected_indices
        or used_texture_coordinates != expected_indices
        or used_normals != expected_indices
    ):
        raise ValueError("backspin checker OBJ topology is not fully referenced")

    texture_tokens = [
        token
        for raw_line in texture.splitlines()
        if (line := raw_line.strip()) and not line.startswith("#")
        for token in line.split()
    ]
    if texture_tokens[:4] != ["P3", "64", "32", "255"]:
        raise ValueError("backspin checker PPM header changed")
    try:
        texels = [int(value) for value in texture_tokens[4:]]
    except ValueError as error:
        raise ValueError("backspin checker PPM contains non-integer texels") from error
    if len(texels) != 64 * 32 * 3:
        raise ValueError("backspin checker PPM texel count changed")
    if any(not 0 <= value <= 255 for value in texels):
        raise ValueError("backspin checker PPM texel range changed")

    ivory = (244, 241, 228)
    charcoal = (32, 36, 43)
    coral = (255, 93, 115)
    pixels = [tuple(texels[index : index + 3]) for index in range(0, len(texels), 3)]
    for y in range(32):
        for x in range(64):
            tile = (min(5, x * 6 // 64), min(3, y * 4 // 32))
            expected = (
                coral
                if tile == (0, 1)
                else ivory if (tile[0] + tile[1]) % 2 == 0 else charcoal
            )
            if pixels[y * 64 + x] != expected:
                raise ValueError("backspin checker PPM tile layout changed")

    return {
        "mesh_vertex_count": len(vertices),
        "mesh_uv_count": len(texture_coordinates),
        "mesh_normal_count": len(normals),
        "mesh_triangle_count": len(faces),
        "mesh_outward_winding": True,
        "mesh_uv_triangles_non_degenerate": True,
        "texture_column_width_range": [10, 11],
        "texture_row_height_range": [8, 8],
        "checker_layout_validated": True,
        "registration_tile_coordinates": [0, 1],
        "registration_tile_count": 1,
    }


def _validate_visual_only_source() -> dict[str, Any]:
    source = _require_file(DEMO_SOURCE, "backspin demo source").read_text(
        encoding="utf-8"
    )
    helper_start = source.index("void addBackspinCheckerTexture")
    helper_end = source.index("//================================", helper_start)
    helper = source[helper_start:helper_end]
    sphere_start = source.index("SkeletonPtr createBackspinSphere()")
    sphere_end = source.index("//================================", sphere_start)
    sphere = source[sphere_start:sphere_end]
    scene_start = source.index("DemoScene makeFbfPaperBackspinScene()")
    scene_end = source.index("//================================", scene_start)
    scene = source[scene_start:scene_end]
    required_helper_fragments = (
        "void addBackspinCheckerTexture(BodyNode* body, double radius)",
        '"dart://sample/obj/fbf_backspin_checker_sphere.obj"',
        "DartResourceRetriever::create()",
        "MeshShape::loadMesh(meshUri, retriever)",
        "Eigen::Vector3d::Constant(radius)",
        "createShapeNodeWith<VisualAspect>(checker)",
    )
    if any(fragment not in helper for fragment in required_helper_fragments):
        raise ValueError("backspin checker-texture source contract changed")
    if "CollisionAspect" in helper or "DynamicsAspect" in helper:
        raise ValueError("backspin checker mesh gained a physics aspect")
    physical_node = "createShapeNodeWith<CollisionAspect, DynamicsAspect>(shape)"
    if physical_node not in sphere or "VisualAspect>(shape)" in sphere:
        raise ValueError("backspin physical sphere aspect contract changed")
    inertia = "setShapeInertia(body, shape);"
    call = "addBackspinCheckerTexture(body, kBackspinRadius);"
    if (
        inertia not in sphere
        or call not in sphere
        or sphere.index(inertia) > sphere.index(call)
    ):
        raise ValueError("backspin inertia/checker-mesh ordering changed")
    if inertia in sphere[sphere.index(call) :]:
        raise ValueError("backspin inertia changes after the checker mesh")
    if source.count(call) != 1:
        raise ValueError("backspin checker mesh attachment count changed")
    camera_home = (
        "::osg::Vec3d(-0.5, -1.25, 5.5)",
        "::osg::Vec3d(-0.5, 0.0, 0.2)",
        "::osg::Vec3d(0.0, 1.0, 0.0)",
    )
    if any(fragment not in scene for fragment in camera_home):
        raise ValueError("backspin checker-legibility camera contract changed")

    mesh = _require_file(CHECKER_MESH, "backspin checker mesh").read_text(
        encoding="utf-8"
    )
    material = _require_file(CHECKER_MATERIAL, "backspin checker material").read_text(
        encoding="utf-8"
    )
    texture = _require_file(CHECKER_TEXTURE, "backspin checker texture").read_text(
        encoding="ascii"
    )
    if (
        mesh.count("mtllib fbf_backspin_checker_sphere.mtl") != 1
        or mesh.count("usemtl FbfBackspinChecker") != 1
        or "\nvt " not in mesh
    ):
        raise ValueError("backspin checker OBJ material/UV contract changed")
    if (
        material.count("map_Kd fbf_backspin_checker.ppm") != 1
        or material.count("Ke 0.180000 0.180000 0.180000") != 1
    ):
        raise ValueError("backspin checker MTL texture binding changed")
    asset_structure = _validate_checker_asset_structure(mesh, texture)
    return {
        "pass": True,
        "shape": "MeshShape",
        "aspect": "VisualAspect_only",
        "mesh_uri": "dart://sample/obj/fbf_backspin_checker_sphere.obj",
        "texture_binding": "fbf_backspin_checker_sphere.mtl:map_Kd",
        "texture_dimensions": [64, 32],
        "checker_grid": [6, 4],
        "palette": {
            "ivory": [244, 241, 228],
            "charcoal": [32, 36, 43],
            "coral": [255, 93, 115],
        },
        "registration_tile": "coral",
        "camera_home": {
            "eye": [-0.5, -1.25, 5.5],
            "center": [-0.5, 0.0, 0.2],
            "up": [0.0, 1.0, 0.0],
            "nearly_perpendicular_to_spin_axis": True,
        },
        **asset_structure,
        "physical_shape_aspects": ["CollisionAspect", "DynamicsAspect"],
        "physics_neutral_by_source_contract": True,
        "full_state_equivalence_proven": False,
        "source_sha256": sha256(DEMO_SOURCE),
        "mesh_sha256": sha256(CHECKER_MESH),
        "material_sha256": sha256(CHECKER_MATERIAL),
        "texture_sha256": sha256(CHECKER_TEXTURE),
    }


def _source_identity(
    *,
    trace_binary: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, Any]:
    sources = {
        "finalizer": Path(__file__).resolve(),
        "finalizer_test": FINALIZER_TEST,
        "visual_runner": runner,
        "visual_runner_test": RUNNER_TEST,
        "image_compose": IMAGE_COMPOSE,
        "image_tools": IMAGE_TOOLS,
        "image_verdict": IMAGE_VERDICT,
        "trace_source": TRACE_SOURCE,
        "fixture_source": FIXTURE_SOURCE,
        "demo_source": DEMO_SOURCE,
        "backspin_checker_mesh": CHECKER_MESH,
        "backspin_checker_material": CHECKER_MATERIAL,
        "backspin_checker_texture": CHECKER_TEXTURE,
        "demo_host_source": DEMO_HOST_SOURCE,
        "demo_host_header": DEMO_HOST_HEADER,
        "registry_source": REGISTRY_SOURCE,
        "scenes_header": SCENES_HEADER,
        "trace_binary": trace_binary,
        "demo_binary": demo,
        "ffmpeg_binary": ffmpeg,
        "ffprobe_binary": ffprobe,
        "python_binary": python,
    }
    return {
        name: {
            "path": str(_require_file(path, name)),
            "sha256": sha256(_require_file(path, name)),
        }
        for name, path in sources.items()
    }


def _report_markdown(
    trace_summary: dict[str, Any],
    capture_summary: dict[str, Any],
    projection: dict[str, Any],
) -> str:
    trace = trace_summary["trace"]
    maximum = trace["maximum_forward_travel"]
    reversal = trace["first_negative_vx"]
    final = trace["final"]
    mp4 = capture_summary["media"]["mp4"]["stream_contract"]
    gif = capture_summary["media"]["gif"]["stream_contract"]
    return f"""# Figure 3 backspin current-source DART evidence

Status: valid current-source DART backspin evidence; not paper parity.

The fresh 131-row exact-FBF trace advances to x={maximum['x']:.17g} m at
step {maximum['step']}, first records negative translational velocity at step
{reversal['step']} (t={reversal['time']:.17g} s), and reaches
x={final['x']:.17g} m with vx={final['vx']:.17g} m/s at step {TOTAL_STEPS}.
This establishes translational advance and reversal for the reconstructed DART
fixture. It does not establish rest.

The capture completes {TOTAL_STEPS} steps with
{capture_summary['captured_frames']} distinct world-view frames, zero accepted
caps, zero exact failures, zero boxed-LCP fallbacks, and worst residual
{capture_summary['worst_residual']:.17g}. The MP4 is {mp4['width']}x{mp4['height']}
at {mp4['frame_rate']} with {mp4['frame_count']} decoded frames; the GIF is
{gif['width']}x{gif['height']} at {gif['frame_rate']} with
{gif['frame_count']} decoded frames.

The high-contrast 6x4 ivory/charcoal checker texture and coral registration tile
are renderer-applied through a UV MeshShape attached as a VisualAspect-only
shape node. The camera is nearly perpendicular to the Y spin axis, so the
checker cells remain large and quadrilateral in the captured viewport. They
make orientation changes legible without adding collision or dynamics aspects.
The physical SphereShape retains only CollisionAspect and DynamicsAspect.
The trace and capture have byte-identical {projection['row_count']}-row
solver/contact projections over {', '.join(projection['fields'])}. That is not
full-state trace equivalence.

Claim boundary: the rendered demo and CSV exporter are separate scene
implementations. The trace exports translation and pose alignment but not
angular velocity. At the encoded 30/15 fps rates, the -200 rad/s motion can
alias, so sampled media do not prove signed angular direction. A contact-free
post-initial step is retained and uninterrupted contact is not claimed. No
landing phase, faithful external-solver row, approved source golden/diff,
author-scene parity, paper timing, or real-time performance verdict is supplied.
"""


def _validate_invocations(
    bundle: Path,
    *,
    trace_binary: Path,
    python: Path,
    runner: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    payload = read_json(bundle / "invocations.json")
    if set(payload) != {"schema_version", "trace", "capture_verification"}:
        raise ValueError("malformed backspin invocations payload")
    if payload.get("schema_version") != INVOCATIONS_SCHEMA_VERSION:
        raise ValueError("unexpected backspin invocations schema")
    trace = payload.get("trace")
    if trace != {
        "argv": _trace_argv(trace_binary),
        "returncode": 0,
        "stdout_path": "traces/backspin.csv",
        "stdout_sha256": sha256(bundle / "traces/backspin.csv"),
        "stderr_path": "traces/backspin.stderr.txt",
        "stderr_sha256": sha256(bundle / "traces/backspin.stderr.txt"),
    }:
        raise ValueError("backspin trace invocation binding changed")
    capture_provenance = read_json(bundle / "capture-provenance.json")
    staging = _validate_staging_manifest(
        bundle, capture_provenance.get("pruned_staging"), require_absent=True
    )
    verification = payload.get("capture_verification")
    if verification != {
        "argv": _verification_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "returncode": 0,
        "stdout_path": "verification.json",
        "stdout_sha256": sha256(bundle / "verification.json"),
        "stderr_path": "verification.stderr.txt",
        "stderr_sha256": staging["verification.stderr.txt"]["sha256"],
    }:
        raise ValueError("backspin verification invocation binding changed")
    return payload


def _validate_capture_provenance(
    payload: Any,
    *,
    bundle: Path,
    python: Path,
    runner: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
    sealed: bool = False,
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("backspin capture provenance is missing")
    expected_resources = _checker_runtime_resource_identity()
    expected = {
        "argv": _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "stdout_path": "capture.stdout.txt",
        "stdout_sha256": None,
        "stderr_path": "capture.stderr.txt",
        "stderr_sha256": None,
        "runtime_resources_before": expected_resources,
        "runtime_resources_after": expected_resources,
    }
    if sealed:
        staging = _validate_staging_manifest(
            bundle, payload.get("pruned_staging"), require_absent=True
        )
        expected["stdout_sha256"] = staging["capture.stdout.txt"]["sha256"]
        expected["stderr_sha256"] = staging["capture.stderr.txt"]["sha256"]
        expected["durable_stills"] = _validate_durable_stills(
            bundle, payload.get("durable_stills")
        )
        expected["pruned_staging"] = payload["pruned_staging"]
    else:
        expected["stdout_sha256"] = sha256(bundle / "capture.stdout.txt")
        expected["stderr_sha256"] = sha256(bundle / "capture.stderr.txt")
    if payload != expected:
        raise ValueError("backspin capture-time provenance changed")
    return payload


def _build_trace_summary(
    parsed_trace: dict[str, Any],
    capture_summary: dict[str, Any],
    trace_path: Path,
) -> dict[str, Any]:
    comparison = _compare_solver_projections(
        parsed_trace["solver_projection"], capture_summary["solver_projection"]
    )
    return {
        "schema_version": TRACE_SUMMARY_SCHEMA_VERSION,
        "pass": True,
        "trace_path": "traces/backspin.csv",
        "trace_sha256": sha256(trace_path),
        "trace": parsed_trace["summary"],
        "solver_projection_comparison": comparison,
        "claim_boundary": (
            "Current-source DART translational advance/reversal only. The CSV "
            "does not export angular velocity; the sampled media do not prove "
            "signed angular direction, rest, or uninterrupted contact."
        ),
    }


def _clean_generated_outputs(bundle: Path) -> None:
    for relative in GENERATED_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()


def _clean_capture_outputs(bundle: Path) -> None:
    for relative in CAPTURE_PATHS | DURABLE_STILL_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()


def _finalize_transaction(
    args: argparse.Namespace, *, commit: Callable[[], None]
) -> dict[str, Any]:
    bundle = _require_bundle_root(args.bundle, create=False)
    _validate_bundle_paths(
        bundle,
        complete=False,
        allow_missing_capture_provenance=not args.reuse_current_capture,
        allow_missing_capture=not args.reuse_current_capture,
    )
    _clean_generated_outputs(bundle)
    if not args.reuse_current_capture:
        _clean_capture_outputs(bundle)
    _validate_bundle_paths(
        bundle,
        complete=False,
        allow_missing_capture_provenance=not args.reuse_current_capture,
        allow_missing_capture=not args.reuse_current_capture,
    )

    trace_binary = _require_file(args.trace_binary, "trace binary")
    demo = _require_file(args.demo, "DART demo binary")
    runner = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")
    python = _require_file(args.python, "Python interpreter")
    visual_source_contract = _validate_visual_only_source()

    if args.reuse_current_capture:
        capture_provenance = read_json(bundle / "capture-provenance.json")
    else:
        capture_resources_before = _checker_runtime_resource_identity()
        capture_argv = _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe)
        capture_process = _run_command(capture_argv, timeout=args.capture_timeout)
        capture_resources_after = _checker_runtime_resource_identity()
        if capture_resources_after != capture_resources_before:
            raise ValueError("backspin checker resources changed during capture")
        if _validate_visual_only_source() != visual_source_contract:
            raise ValueError("backspin visual source contract changed during capture")
        capture_stdout_path = bundle / "capture.stdout.txt"
        capture_stderr_path = bundle / "capture.stderr.txt"
        capture_stdout_path.write_text(capture_process.stdout, encoding="utf-8")
        capture_stderr_path.write_text(capture_process.stderr, encoding="utf-8")
        run_summary_path = bundle / "run-summary.json"
        capture_payload = read_json(run_summary_path)
        if (
            capture_payload.get("kind") != "capture_run"
            or capture_payload.get("pass") is not True
            or len(capture_payload.get("results", [])) != 1
            or capture_payload["results"][0].get("schedule", {}).get("id") != "backspin"
        ):
            raise ValueError(
                "visual capture did not produce one passing backspin result"
            )
        capture_provenance = {
            "argv": capture_argv,
            "returncode": capture_process.returncode,
            "run_summary_path": "run-summary.json",
            "run_summary_sha256": sha256(run_summary_path),
            "run_summary_validated": True,
            "stdout_path": "capture.stdout.txt",
            "stdout_sha256": sha256(capture_stdout_path),
            "stderr_path": "capture.stderr.txt",
            "stderr_sha256": sha256(capture_stderr_path),
            "runtime_resources_before": capture_resources_before,
            "runtime_resources_after": capture_resources_after,
        }
        write_json(bundle / "capture-provenance.json", capture_provenance)
    _validate_capture_provenance(
        capture_provenance,
        bundle=bundle,
        python=python,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    durable_stills = _materialize_durable_stills(bundle)
    _validate_bundle_paths(
        bundle,
        complete=False,
        allow_missing_manual=not args.reuse_current_capture,
    )
    if not (bundle / "manual-inspection.json").is_file():
        commit()
        raise ValueError(
            "fresh backspin capture is complete; manually inspect the bound "
            "artifacts, add manual-inspection.json, then rerun with "
            "--reuse-current-capture"
        )

    trace_argv = _trace_argv(trace_binary)
    trace_process = _run_command(trace_argv, timeout=args.trace_timeout)
    trace_path = bundle / "traces/backspin.csv"
    trace_stderr_path = bundle / "traces/backspin.stderr.txt"
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    trace_path.write_text(trace_process.stdout, encoding="utf-8")
    trace_stderr_path.write_text(trace_process.stderr, encoding="utf-8")
    parsed_trace = parse_trace_text(trace_process.stdout)

    verify_argv = _verification_argv(python, runner, demo, bundle, ffmpeg, ffprobe)
    verify_process = _run_command(verify_argv, timeout=args.verification_timeout)
    try:
        verification = json.loads(verify_process.stdout)
    except json.JSONDecodeError as error:
        raise ValueError("visual verifier did not emit one JSON document") from error
    if not isinstance(verification, dict):
        raise ValueError("visual verifier did not emit a JSON object")
    write_json(bundle / "verification.json", verification)
    (bundle / "verification.stderr.txt").write_text(
        verify_process.stderr, encoding="utf-8"
    )

    capture_summary = validate_capture_bundle(
        bundle,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        verification=verification,
    )
    trace_summary = _build_trace_summary(parsed_trace, capture_summary, trace_path)
    write_json(bundle / "trace-summary.json", trace_summary)
    manual = validate_manual_inspection(bundle)

    invocations = {
        "schema_version": INVOCATIONS_SCHEMA_VERSION,
        "trace": {
            "argv": trace_argv,
            "returncode": trace_process.returncode,
            "stdout_path": "traces/backspin.csv",
            "stdout_sha256": sha256(trace_path),
            "stderr_path": "traces/backspin.stderr.txt",
            "stderr_sha256": sha256(trace_stderr_path),
        },
        "capture_verification": {
            "argv": verify_argv,
            "returncode": verify_process.returncode,
            "stdout_path": "verification.json",
            "stdout_sha256": sha256(bundle / "verification.json"),
            "stderr_path": "verification.stderr.txt",
            "stderr_sha256": sha256(bundle / "verification.stderr.txt"),
        },
    }
    write_json(bundle / "invocations.json", invocations)
    report = _report_markdown(
        trace_summary,
        capture_summary,
        trace_summary["solver_projection_comparison"],
    )
    (bundle / "REPORT.md").write_text(report, encoding="utf-8")

    staging_manifest = _build_staging_manifest(bundle)
    capture_provenance = {
        **capture_provenance,
        "durable_stills": durable_stills,
        "pruned_staging": staging_manifest,
    }
    write_json(bundle / "capture-provenance.json", capture_provenance)
    _prune_staging_outputs(bundle, staging_manifest)
    _validate_capture_provenance(
        capture_provenance,
        bundle=bundle,
        python=python,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        sealed=True,
    )

    files_before_index = {
        path.relative_to(bundle).as_posix()
        for path in bundle.rglob("*")
        if path.is_file() and not path.is_symlink()
    }
    expected_before_index = EXPECTED_FINAL_PATHS - INDEX_EXCLUSIONS
    if files_before_index != expected_before_index:
        raise ValueError(
            "backspin bundle is not ready for indexing: "
            f"missing={sorted(expected_before_index - files_before_index)}, "
            f"extra={sorted(files_before_index - expected_before_index)}"
        )
    index = artifact_index(bundle)
    write_json(bundle / "artifact-index.json", index)

    capture_metadata = dict(capture_summary)
    capture_metadata.pop("solver_projection")
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "status": "valid_current_source_nonpaper_backspin",
        "pass": True,
        "evidence_date": args.evidence_date,
        "requirement_ids": ["fig.03", "video.02_backspin"],
        "artifact_valid": True,
        "solver_contract_valid": True,
        "physical_outcome_valid": True,
        "paper_parity": False,
        "paper_comparable": False,
        "external_solver_parity": False,
        "approved_source_golden": False,
        "timing_verdict": None,
        "realtime_verdict": None,
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "manual_visual_outcome_validated": True,
        "trace_equivalence_to_rendered_demo": False,
        "solver_projection_equivalent": True,
        "strict_rigid_body_rest_proven": False,
        "signed_angular_direction_proven": False,
        "continuous_contact_proven": False,
        "capture_sidecar_deliverable_validated": False,
        "claim_scope": (
            "Current-source DART translational advance/reversal and manual "
            "high-contrast checker-texture orientation legibility through "
            "2.17 seconds."
        ),
        "trace_summary": trace_summary,
        "capture_summary": capture_metadata,
        "capture_provenance": capture_provenance,
        "visual_only_source_contract": visual_source_contract,
        "manual_inspection": {
            "path": "manual-inspection.json",
            "sha256": sha256(bundle / "manual-inspection.json"),
            "pass": manual["pass"],
        },
        "run_summary": {
            "path": "run-summary.json",
            "sha256": sha256(bundle / "run-summary.json"),
        },
        "verification": {
            "path": "verification.json",
            "sha256": sha256(bundle / "verification.json"),
        },
        "invocations": {
            "path": "invocations.json",
            "sha256": sha256(bundle / "invocations.json"),
        },
        "report": {"path": "REPORT.md", "sha256": sha256(bundle / "REPORT.md")},
        "artifact_index": {
            "path": "artifact-index.json",
            "sha256": sha256(bundle / "artifact-index.json"),
            "artifact_count": index["artifact_count"],
            "excluded": index["excluded"],
        },
        "source_identity": _source_identity(
            trace_binary=trace_binary,
            demo=demo,
            runner=runner,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
        ),
        "claim_boundary": (
            "The rendered demo and CSV exporter are separate scene implementations. "
            "This bundle does not establish full-state trace equivalence, signed "
            "angular direction from sampled media, uninterrupted contact, rest, an "
            "airborne landing phase, author-scene or paper parity, faithful external-"
            "solver parity, an approved source golden/diff, paper timing, or real-time "
            "performance."
        ),
    }
    write_json(bundle / "metadata.json", metadata)
    try:
        result = verify_finalized(
            bundle,
            trace_binary=trace_binary,
            demo=demo,
            runner=runner,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
        )
    except (OSError, ValueError):
        (bundle / "metadata.json").unlink(missing_ok=True)
        (bundle / "artifact-index.json").unlink(missing_ok=True)
        raise
    commit()
    return result


def _remove_bundle_root(bundle: Path) -> None:
    """Remove a bundle path without following a replacement symlink."""
    try:
        mode = bundle.lstat().st_mode
    except FileNotFoundError:
        return
    if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
        bundle.unlink()
        return
    shutil.rmtree(bundle)


@contextlib.contextmanager
def _bundle_transaction(bundle: Path) -> Iterator[Callable[[], None]]:
    bundle = _require_bundle_root(bundle, create=False)
    committed = False

    def commit() -> None:
        nonlocal committed
        committed = True

    with tempfile.TemporaryDirectory(
        prefix=f".{bundle.name}.backup-", dir=bundle.parent
    ) as temporary_directory:
        snapshot = Path(temporary_directory) / "bundle"
        shutil.copytree(bundle, snapshot)
        try:
            yield commit
            if not committed:
                raise ValueError("backspin bundle transaction was not committed")
        except BaseException:
            if not committed:
                _remove_bundle_root(bundle)
                shutil.copytree(snapshot, bundle)
            raise


def finalize(args: argparse.Namespace) -> dict[str, Any]:
    bundle = _require_bundle_root(args.bundle, create=False)
    _validate_bundle_paths(
        bundle,
        complete=False,
        allow_missing_capture_provenance=not args.reuse_current_capture,
        allow_missing_capture=not args.reuse_current_capture,
    )
    with _bundle_transaction(bundle) as commit:
        return _finalize_transaction(args, commit=commit)


def verify_finalized(
    bundle: Path,
    *,
    trace_binary: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, Any]:
    bundle = _require_bundle_root(bundle, create=False)
    _validate_bundle_paths(bundle, complete=True)
    metadata = read_json(bundle / "metadata.json")
    expected_flags = {
        "schema_version": SCHEMA_VERSION,
        "status": "valid_current_source_nonpaper_backspin",
        "pass": True,
        "requirement_ids": ["fig.03", "video.02_backspin"],
        "artifact_valid": True,
        "solver_contract_valid": True,
        "physical_outcome_valid": True,
        "paper_parity": False,
        "paper_comparable": False,
        "external_solver_parity": False,
        "approved_source_golden": False,
        "timing_verdict": None,
        "realtime_verdict": None,
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "manual_visual_outcome_validated": True,
        "trace_equivalence_to_rendered_demo": False,
        "solver_projection_equivalent": True,
        "strict_rigid_body_rest_proven": False,
        "signed_angular_direction_proven": False,
        "continuous_contact_proven": False,
        "capture_sidecar_deliverable_validated": False,
    }
    if any(metadata.get(key) != value for key, value in expected_flags.items()):
        raise ValueError("backspin finalized metadata claim flags changed")
    if (
        not isinstance(metadata.get("evidence_date"), str)
        or not metadata["evidence_date"]
    ):
        raise ValueError("backspin evidence date is missing")

    trace_path = bundle / "traces/backspin.csv"
    parsed_trace = parse_trace_text(trace_path.read_text(encoding="utf-8"))
    capture_provenance = read_json(bundle / "capture-provenance.json")
    if metadata.get("capture_provenance") != capture_provenance:
        raise ValueError("backspin capture-provenance file binding is stale")
    _validate_capture_provenance(
        capture_provenance,
        bundle=bundle,
        python=python,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        sealed=True,
    )
    capture_summary = validate_capture_bundle(
        bundle,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        staging_manifest=capture_provenance["pruned_staging"],
    )
    trace_summary = _build_trace_summary(parsed_trace, capture_summary, trace_path)
    if read_json(bundle / "trace-summary.json") != trace_summary:
        raise ValueError("backspin trace-summary.json is stale")
    if metadata.get("trace_summary") != trace_summary:
        raise ValueError("backspin metadata trace summary is stale")
    capture_metadata = dict(capture_summary)
    capture_metadata.pop("solver_projection")
    if metadata.get("capture_summary") != capture_metadata:
        raise ValueError("backspin metadata capture summary is stale")

    visual_source_contract = _validate_visual_only_source()
    if metadata.get("visual_only_source_contract") != visual_source_contract:
        raise ValueError("backspin visual-only source contract is stale")
    manual = validate_manual_inspection(bundle)
    if metadata.get("manual_inspection") != {
        "path": "manual-inspection.json",
        "sha256": sha256(bundle / "manual-inspection.json"),
        "pass": manual["pass"],
    }:
        raise ValueError("backspin metadata manual-inspection binding is stale")

    _validate_invocations(
        bundle,
        trace_binary=trace_binary,
        python=python,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    for key, relative in (
        ("run_summary", "run-summary.json"),
        ("verification", "verification.json"),
        ("invocations", "invocations.json"),
        ("report", "REPORT.md"),
    ):
        if metadata.get(key) != {"path": relative, "sha256": sha256(bundle / relative)}:
            raise ValueError(f"backspin metadata {key} binding is stale")
    expected_report = _report_markdown(
        trace_summary,
        capture_summary,
        trace_summary["solver_projection_comparison"],
    )
    if (bundle / "REPORT.md").read_text(encoding="utf-8") != expected_report:
        raise ValueError("backspin REPORT.md is stale")

    index = read_json(bundle / "artifact-index.json")
    validate_artifact_index(bundle, index)
    if metadata.get("artifact_index") != {
        "path": "artifact-index.json",
        "sha256": sha256(bundle / "artifact-index.json"),
        "artifact_count": index["artifact_count"],
        "excluded": index["excluded"],
    }:
        raise ValueError("backspin metadata artifact-index binding is stale")
    sources = _source_identity(
        trace_binary=_require_file(trace_binary, "trace binary"),
        demo=_require_file(demo, "DART demo binary"),
        runner=_require_file(runner, "visual evidence runner"),
        ffmpeg=_require_file(ffmpeg, "ffmpeg"),
        ffprobe=_require_file(ffprobe, "ffprobe"),
        python=_require_file(python, "Python interpreter"),
    )
    if metadata.get("source_identity") != sources:
        raise ValueError("backspin source/binary identity changed")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": metadata["status"],
        "pass": True,
        "artifact_count": index["artifact_count"],
        "trace_summary_sha256": sha256(bundle / "trace-summary.json"),
        "verification_sha256": sha256(bundle / "verification.json"),
        "manual_inspection_sha256": sha256(bundle / "manual-inspection.json"),
        "artifact_index_sha256": sha256(bundle / "artifact-index.json"),
        "metadata_sha256": sha256(bundle / "metadata.json"),
        "solver_projection_sha256": trace_summary["solver_projection_comparison"][
            "trace_sha256"
        ],
    }


def verify_only(args: argparse.Namespace) -> dict[str, Any]:
    bundle = _require_bundle_root(args.bundle, create=False)
    trace_binary = _require_file(args.trace_binary, "trace binary")
    demo = _require_file(args.demo, "DART demo binary")
    runner = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")
    python = _require_file(args.python, "Python interpreter")
    result = verify_finalized(
        bundle,
        trace_binary=trace_binary,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )

    trace_process = _run_command(_trace_argv(trace_binary), timeout=args.trace_timeout)
    stored_trace = (bundle / "traces/backspin.csv").read_text(encoding="utf-8")
    if trace_process.stdout != stored_trace:
        raise ValueError("live backspin trace replay differs from the finalized trace")
    if trace_process.stderr != (bundle / "traces/backspin.stderr.txt").read_text(
        encoding="utf-8"
    ):
        raise ValueError("live backspin trace stderr differs from the finalized trace")

    return {
        **result,
        "live_trace_replay": True,
        "live_capture_reverification": False,
        "durable_capture_reverification": True,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--trace-binary", type=Path, default=DEFAULT_TRACE_BINARY)
    parser.add_argument("--demo", type=Path, default=DEFAULT_DEMO)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--ffmpeg", type=Path, default=DEFAULT_FFMPEG)
    parser.add_argument("--ffprobe", type=Path, default=DEFAULT_FFPROBE)
    parser.add_argument("--python", type=Path, default=DEFAULT_PYTHON)
    parser.add_argument("--capture-timeout", type=float, default=900.0)
    parser.add_argument("--trace-timeout", type=float, default=120.0)
    parser.add_argument("--verification-timeout", type=float, default=600.0)
    parser.add_argument("--evidence-date", default="2026-07-18")
    parser.add_argument(
        "--reuse-current-capture",
        action="store_true",
        help=(
            "finalize an already captured bundle only after validating its "
            "persisted capture-time checker-resource provenance"
        ),
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help=(
            "recompute finalized durable capture contracts and replay the current "
            "trace without requiring pruned capture staging"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.verify_only and args.reuse_current_capture:
            raise ValueError(
                "--verify-only and --reuse-current-capture are mutually exclusive"
            )
        result = verify_only(args) if args.verify_only else finalize(args)
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
