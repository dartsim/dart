#!/usr/bin/env python3
"""Finalize and revalidate author-source-pinned DART turntable evidence.

The visual claim is deliberately narrow.  The four source-ordered DART cells
are captured together and are bound only to fresh ``dart_best`` traces using
the same DART collision frontend as ``dart-demos``.  A second four-trace
``paper_cpu``/Native lane is retained as separate diagnostic evidence and is
never substituted for, or compared as if it were, the rendered trajectory.

Fresh capture is the default. Reusing media requires persisted capture-time
provenance, and ``--verify-only`` replays all eight traces while revalidating
the sealed panels, clips, timelines, and outcome stills without ignored staging.
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
BUILD_RUNTIME_ROOT = ROOT / "build/default/cpp/Release"
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "fig04_turntable_author_current_v1"
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
EXACT_FBF_SOURCE = ROOT / "dart/constraint/ExactCoulombFbfConstraintSolver.cpp"
EXACT_FBF_HEADER = ROOT / "dart/constraint/ExactCoulombFbfConstraintSolver.hpp"
DART_COLLISION_SOURCE = ROOT / "dart/collision/dart/DARTCollisionDetector.cpp"
DART_COLLISION_HEADER = ROOT / "dart/collision/dart/DARTCollisionDetector.hpp"
NATIVE_COLLISION_SOURCE = ROOT / "dart/collision/native/NativeCollisionDetector.cpp"
NATIVE_COLLISION_HEADER = ROOT / "dart/collision/native/NativeCollisionDetector.hpp"
NATIVE_CYLINDER_SOURCE = (
    ROOT / "dart/collision/native/narrow_phase/CylinderCollision.cpp"
)
NATIVE_CYLINDER_HEADER = (
    ROOT / "dart/collision/native/narrow_phase/CylinderCollision.hpp"
)
NATIVE_NARROW_PHASE_SOURCE = ROOT / "dart/collision/native/narrow_phase/NarrowPhase.cpp"
NATIVE_NARROW_PHASE_HEADER = ROOT / "dart/collision/native/narrow_phase/NarrowPhase.hpp"
DEMO_SOURCE = ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
AUTHOR_TURNTABLE_SPEC_SOURCE = ROOT / "examples/demos/scenes/FbfAuthorTurntableSpec.hpp"
DEMO_CMAKE_SOURCE = ROOT / "examples/demos/CMakeLists.txt"
DEMO_MAIN_SOURCE = ROOT / "examples/demos/main.cpp"
DEMO_SCENE_HEADER = ROOT / "examples/demos/DemoScene.hpp"
DEMO_HOST_SOURCE = ROOT / "examples/demos/DemoHost.cpp"
DEMO_HOST_HEADER = ROOT / "examples/demos/DemoHost.hpp"
REGISTRY_SOURCE = ROOT / "examples/demos/Registry.cpp"
SCENES_HEADER = ROOT / "examples/demos/scenes/Scenes.hpp"
AUTHOR_TURNTABLE_VISUAL_OBJ = ROOT / "data/obj/fbf_author_turntable_disc.obj"
AUTHOR_TURNTABLE_VISUAL_MTL = ROOT / "data/obj/fbf_author_turntable_disc.mtl"
TRACE_CMAKE_SOURCE = ROOT / "tests/benchmark/integration/CMakeLists.txt"
RUNNER_TEST = ROOT / "python/tests/unit/test_run_fbf_visual_evidence.py"
FINALIZER_TEST = ROOT / "python/tests/unit/test_finalize_fbf_turntable_visual.py"
IMAGE_COMPOSE = ROOT / "scripts/image_compose.py"
IMAGE_TOOLS = ROOT / "scripts/_image_tools.py"
IMAGE_VERDICT = ROOT / "scripts/image_verdict.py"

SCHEMA_VERSION = "dart.fbf_turntable_visual_bundle/v1"
INDEX_SCHEMA_VERSION = "dart.fbf_turntable_artifact_index/v1"
MANUAL_SCHEMA_VERSION = "dart.fbf_turntable_manual_inspection/v1"
TRACE_SUMMARY_SCHEMA_VERSION = "dart.fbf_turntable_trace_summary/v1"
INVOCATIONS_SCHEMA_VERSION = "dart.fbf_turntable_invocations/v1"
RUNNER_SCHEMA_VERSION = "dart.fbf_visual_evidence/v1"

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
AUTHOR_COMMIT = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
AUTHOR_REPOSITORY = "https://github.com/matthcsong/fbf-sca-2026"
AUTHOR_TURNTABLE_SOURCE_SHA256 = (
    "5dd330d2e430585fc3e61eb9c11d2d9aa00b518170f7b64b809c69587cf7db53"
)
AUTHOR_TURNTABLE_RADIUS = 2.0
AUTHOR_TURNTABLE_CUBE_HALF_SIZE = 0.15
AUTHOR_TURNTABLE_TABLE_TOP = 0.1
AUTHOR_TURNTABLE_FULLY_SUPPORTED_RADIUS = (
    AUTHOR_TURNTABLE_RADIUS - AUTHOR_TURNTABLE_CUBE_HALF_SIZE
)
AUTHOR_TURNTABLE_FALLTHROUGH_HEIGHT = (
    AUTHOR_TURNTABLE_TABLE_TOP - AUTHOR_TURNTABLE_CUBE_HALF_SIZE
)
AUTHOR_TURNTABLE_SOURCE_VERTICAL_EXIT_HEIGHT = AUTHOR_TURNTABLE_TABLE_TOP - 0.5
PHYSICS_CONTRACT_SCHEMA_VERSION = "dart.fbf_author_turntable_physics_contract/v1"
PHYSICS_CONTRACT_KIND = "physics_control"
PHYSICS_EXPECTED_OUTCOMES = (
    "ejected",
    "ejected",
    "retained_through_6s",
    "ejected",
)
TOTAL_STEPS = 360
DT_SECONDS = 1.0 / 60.0
RESIDUAL_TOLERANCE = 1.0e-6
FRAME_STEPS = tuple(range(0, TOTAL_STEPS + 1, 2))
OUTPUT_FPS = 30
VIDEO_FRAME_COUNT = len(FRAME_STEPS)
VIDEO_DURATION_SECONDS = VIDEO_FRAME_COUNT / OUTPUT_FPS
PANEL_STEPS = (0, 30, 60, 180, 360)
PANEL_LABELS = ("t=0s", "settled t=.5s", "ramped t=1s", "t=3s", "t=6s")
MOTION_KEYFRAME_PAIRS = ((0, 30), (30, 60), (60, 180))
GROUP_ID = "turntable_author"
SOURCE_SEGMENT = "turntable"
GROUP_LABELS = (
    "mu=0.2, omega=2 rad/s",
    "mu=0.2, omega=5 rad/s",
    "mu=0.5, omega=2 rad/s",
    "mu=0.5, omega=5 rad/s",
)
GROUP_PANEL_STEPS = (136, 120, 360, 90)
GROUP_PANEL_LABELS = (
    "MU 0.2 OMEGA 2 T 2.27S",
    "MU 0.2 OMEGA 5 T 2.00S",
    "MU 0.5 OMEGA 2 T 6.00S",
    "MU 0.5 OMEGA 5 T 1.50S",
)

LANES = {
    "current_visual": {
        "solver_contract": "dart_best",
        "collision_frontend": "native",
        "role": "visual_compatible_author_source_port",
    },
    "paper_cpu_native": {
        "solver_contract": "paper_cpu",
        "collision_frontend": "native",
        "role": "separate_strict_diagnostic_not_render_binding",
    },
}

SCENARIOS = {
    "turntable_author_mu_0_2_omega_2": {
        "capture_id": "turntable_author_mu02_omega2",
        "scene": "fbf_author_turntable_mu_0_2_omega_2",
        "mu": "0.2",
        "omega": "2",
        "expected_outcome": "ejected",
    },
    "turntable_author_mu_0_2_omega_5": {
        "capture_id": "turntable_author_mu02_omega5",
        "scene": "fbf_author_turntable_mu_0_2_omega_5",
        "mu": "0.2",
        "omega": "5",
        "expected_outcome": "ejected",
    },
    "turntable_author_mu_0_5_omega_2": {
        "capture_id": "turntable_author_mu05_omega2",
        "scene": "fbf_author_turntable_mu_0_5_omega_2",
        "mu": "0.5",
        "omega": "2",
        "expected_outcome": "retained_through_6s",
    },
    "turntable_author_mu_0_5_omega_5": {
        "capture_id": "turntable_author_mu05_omega5",
        "scene": "fbf_author_turntable_mu_0_5_omega_5",
        "mu": "0.5",
        "omega": "5",
        "expected_outcome": "ejected",
    },
}
TRACE_CLASSIFICATION_BY_EXPECTED_OUTCOME = {
    "ejected": "ejected",
    "retained_through_6s": "captured",
}
CAPTURE_IDS = tuple(item["capture_id"] for item in SCENARIOS.values())
GROUP_PANEL_STEP_BY_CAPTURE = dict(zip(CAPTURE_IDS, GROUP_PANEL_STEPS))

MANUAL_VERDICTS = {
    "four_cell_source_order_visible": True,
    "parameter_labels_legible": True,
    "synchronized_motion_visible": True,
    "three_ejections_visible": True,
    "mu05_omega2_retained_through_6s_visible": True,
    "author_cylinder_support_geometry_visible": True,
    "paper_style_segmented_turntable_visible": True,
    "turntable_rotation_visible": True,
    "outcome_aware_still_steps_bound": True,
    "retained_wording_does_not_claim_zero_slip": True,
    "source_port_and_backend_semantic_boundary_disclosed": True,
    "physical_outcome_claim_requires_current_visual_traces": True,
    "paper_cpu_native_lane_kept_separate": True,
    "rendered_demo_and_trace_full_state_equivalent": False,
    "paper_parity": False,
    "external_solver_parity": False,
    "approved_source_golden": False,
    "timing_verdict": None,
    "realtime_verdict": None,
}
MANUAL_ARTIFACT_PATHS = {
    "groups/turntable_author/panel.png",
    "groups/turntable_author/clip.mp4",
    *(
        f"{capture_id}/stills/outcome_step_{step:06d}.png"
        for capture_id, step in zip(CAPTURE_IDS, GROUP_PANEL_STEPS)
    ),
}
OUTCOME_STILL_PATH_BY_CAPTURE = {
    capture_id: f"{capture_id}/stills/outcome_step_{step:06d}.png"
    for capture_id, step in zip(CAPTURE_IDS, GROUP_PANEL_STEPS)
}
INDEX_EXCLUSIONS = {"artifact-index.json", "metadata.json"}


def _capture_paths() -> set[str]:
    paths = {
        "run-summary.json",
        "capture-provenance.json",
        "manual-inspection.json",
        "groups/turntable_author/metadata.json",
        "groups/turntable_author/panel.png",
        "groups/turntable_author/clip.mp4",
        "groups/turntable_author/panel_top.png",
        "groups/turntable_author/panel_bottom.png",
        "groups/turntable_author/panel_row_0.compose.json",
        "groups/turntable_author/panel_row_1.compose.json",
    }
    for capture_id in CAPTURE_IDS:
        paths.update(
            {
                f"{capture_id}/metadata.json",
                f"{capture_id}/timeline.json",
                f"{capture_id}/panel.png",
                f"{capture_id}/panel.compose.json",
                f"{capture_id}/clip.mp4",
                OUTCOME_STILL_PATH_BY_CAPTURE[capture_id],
                f"groups/turntable_author/panel_members/{capture_id}.png",
            }
        )
    return paths


CAPTURE_PATHS = _capture_paths()
STAGING_PATHS = {
    "capture.stdout.txt",
    "capture.stderr.txt",
    "verification.stderr.txt",
}
for _capture_id in CAPTURE_IDS:
    STAGING_PATHS.add(f"{_capture_id}/video_frames.ffconcat")
    STAGING_PATHS.update(
        f"{_capture_id}/frames/step_{step:06d}.png" for step in FRAME_STEPS
    )
    STAGING_PATHS.update(
        f"{_capture_id}/panel_frames/step_{step:06d}.png" for step in PANEL_STEPS
    )
TRACE_PATHS = {
    f"traces/{lane}/{scenario}{suffix}"
    for lane in LANES
    for scenario in SCENARIOS
    for suffix in (".csv", ".stderr.txt")
}
GENERATED_PATHS = TRACE_PATHS | {
    "trace-summary.json",
    "verification.json",
    "invocations.json",
    "REPORT.md",
    "artifact-index.json",
    "metadata.json",
}
EXPECTED_FINAL_PATHS = CAPTURE_PATHS | GENERATED_PATHS
ALLOWED_DIRECTORIES = {
    "groups",
    "groups/turntable_author",
    "groups/turntable_author/panel_members",
    "traces",
    *(f"traces/{lane}" for lane in LANES),
    *CAPTURE_IDS,
    *(f"{capture_id}/stills" for capture_id in CAPTURE_IDS),
}
STAGING_DIRECTORIES = {
    *(f"{capture_id}/frames" for capture_id in CAPTURE_IDS),
    *(f"{capture_id}/panel_frames" for capture_id in CAPTURE_IDS),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


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
        raise ValueError(f"turntable bundle root is a symlink: {original}")
    for ancestor in absolute.parents:
        if ancestor.is_symlink():
            raise ValueError(
                f"turntable bundle root passes through a symlink: {original}"
            )
    if original.exists():
        if not original.is_dir():
            raise ValueError(f"turntable bundle is not a regular directory: {original}")
    elif create:
        original.mkdir(parents=True)
    else:
        raise ValueError(f"turntable bundle directory does not exist: {original}")
    if original.is_symlink():
        raise ValueError(f"turntable bundle root became a symlink: {original}")
    resolved = original.resolve(strict=True)
    if resolved != absolute:
        raise ValueError(f"turntable bundle root passes through a symlink: {original}")
    return resolved


def _validate_bundle_paths(
    root: Path,
    *,
    complete: bool,
    require_capture: bool,
    allow_missing_manual: bool = False,
    allow_staging: bool = False,
) -> None:
    root = _require_bundle_root(root, create=False)

    files: set[str] = set()
    directories: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise ValueError(f"turntable bundle contains a symlink: {relative}")
        if path.is_dir():
            directories.add(relative)
        elif path.is_file():
            files.add(relative)
        else:
            raise ValueError(f"turntable bundle has a non-regular entry: {relative}")

    allowed_files = EXPECTED_FINAL_PATHS | (STAGING_PATHS if allow_staging else set())
    allowed_directories = ALLOWED_DIRECTORIES | (
        STAGING_DIRECTORIES if allow_staging else set()
    )
    unexpected_files = files - allowed_files
    unexpected_directories = directories - allowed_directories
    if unexpected_files or unexpected_directories:
        raise ValueError(
            "turntable bundle membership has unexpected entries: "
            f"files={sorted(unexpected_files)}, "
            f"directories={sorted(unexpected_directories)}"
        )
    if require_capture:
        missing_capture = CAPTURE_PATHS - files
        if allow_missing_manual:
            missing_capture.discard("manual-inspection.json")
        if missing_capture:
            raise ValueError(
                f"turntable capture bundle is incomplete: {sorted(missing_capture)}"
            )
    if complete:
        surviving_staging = files & STAGING_PATHS
        surviving_staging_directories = directories & STAGING_DIRECTORIES
        if surviving_staging or surviving_staging_directories:
            raise ValueError(
                "turntable ignored staging survived sealing: "
                f"files={sorted(surviving_staging)}, "
                f"directories={sorted(surviving_staging_directories)}"
            )
        missing = EXPECTED_FINAL_PATHS - files
        if missing:
            raise ValueError(
                f"turntable finalized bundle is incomplete: {sorted(missing)}"
            )
        missing_directories = ALLOWED_DIRECTORIES - directories
        if missing_directories:
            raise ValueError(
                "turntable finalized directories are incomplete: "
                f"{sorted(missing_directories)}"
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


def _turntable_history(rows: Sequence[dict[str, Any]]) -> dict[str, list[int]]:
    fallthrough_steps: list[int] = []
    radial_exit_steps: list[int] = []
    source_exit_steps: list[int] = []
    for row in rows:
        radius = math.hypot(row["x"], row["y"])
        radial_exit = radius > AUTHOR_TURNTABLE_RADIUS + AUTHOR_TURNTABLE_CUBE_HALF_SIZE
        if radial_exit:
            radial_exit_steps.append(row["step"])
        if (
            radius <= AUTHOR_TURNTABLE_FULLY_SUPPORTED_RADIUS
            and row["z"] < AUTHOR_TURNTABLE_FALLTHROUGH_HEIGHT
        ):
            fallthrough_steps.append(row["step"])
        if radial_exit or row["z"] < AUTHOR_TURNTABLE_SOURCE_VERTICAL_EXIT_HEIGHT:
            source_exit_steps.append(row["step"])
    return {
        "support_fallthrough_steps": fallthrough_steps,
        "radial_exit_steps": radial_exit_steps,
        "source_exit_steps": source_exit_steps,
    }


def _classify_turntable_outcome(rows: Sequence[dict[str, Any]]) -> str:
    history = _turntable_history(rows)
    if history["support_fallthrough_steps"]:
        return "invalid_support_fallthrough"

    final = rows[-1]
    horizontal_radius = math.hypot(final["x"], final["y"])
    if (
        final["contacts"] > 0
        and final["z"] > AUTHOR_TURNTABLE_TABLE_TOP
        and 0.75 <= horizontal_radius <= 1.35
        and not history["source_exit_steps"]
    ):
        return "captured"
    if (
        final["contacts"] == 0
        and horizontal_radius
        > AUTHOR_TURNTABLE_RADIUS + AUTHOR_TURNTABLE_CUBE_HALF_SIZE
    ):
        return "ejected"
    return "indeterminate"


def parse_trace_text(text: str, expected_scenario: str, *, lane: str) -> dict[str, Any]:
    if expected_scenario not in SCENARIOS:
        raise ValueError(f"unsupported turntable scenario: {expected_scenario}")
    if lane not in LANES:
        raise ValueError(f"unsupported turntable trace lane: {lane}")
    reader = csv.DictReader(io.StringIO(text))
    if tuple(reader.fieldnames or ()) != TRACE_COLUMNS:
        raise ValueError(
            f"unexpected trace columns: {reader.fieldnames!r}; expected {TRACE_COLUMNS!r}"
        )
    raw_rows = list(reader)
    if len(raw_rows) != TOTAL_STEPS + 1:
        raise ValueError(
            f"{expected_scenario}: expected {TOTAL_STEPS + 1} rows including step 0, "
            f"got {len(raw_rows)}"
        )

    rows: list[dict[str, Any]] = []
    previous_cumulative = {
        field: 0 for field in ("exact_solves", "warm_starts", "fallbacks")
    }
    max_residual = 0.0
    contract_failure_steps: list[int] = []
    accepted_at_cap_steps: list[int] = []
    solver_started = False
    for index, raw in enumerate(raw_rows):
        line = index + 2
        if None in raw or set(raw) != set(TRACE_COLUMNS):
            raise ValueError(f"trace line {line}: malformed CSV row")
        step = _int_field(raw, "step", line=line)
        if step != index:
            raise ValueError(
                f"{expected_scenario}: noncontiguous step at row {index}: {step}"
            )
        time_seconds = _float_field(raw, "time", line=line)
        if not math.isclose(
            time_seconds, step * DT_SECONDS, rel_tol=0.0, abs_tol=2.0e-14
        ):
            raise ValueError(
                f"{expected_scenario}: step {step} time {time_seconds} is not step/60"
            )
        if raw["scenario"] != expected_scenario:
            raise ValueError(
                f"trace line {line}: scenario {raw['scenario']!r} != "
                f"{expected_scenario!r}"
            )
        if raw["solver"] != "exact_fbf" or raw["body"] != "turntable_rider_body":
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
        for field in ("contacts", "exact_solves", "warm_starts", "fallbacks"):
            typed[field] = _int_field(raw, field, line=line)
            if typed[field] < 0:
                raise ValueError(f"trace line {line}: negative {field}")
        for field in ("exact_solves", "warm_starts", "fallbacks"):
            if typed[field] < previous_cumulative[field]:
                raise ValueError(f"trace line {line}: decreasing cumulative {field}")
            previous_cumulative[field] = typed[field]

        if raw["status"] == "not_run":
            residual = _float_field(raw, "residual", line=line, allow_nan=True)
            if not math.isnan(residual):
                raise ValueError(
                    f"{expected_scenario}: not_run step {step} must have nan residual"
                )
            if solver_started:
                raise ValueError(
                    f"{expected_scenario}: not_run step {step} follows an exact solve"
                )
            if typed["contacts"] != 0 or any(
                typed[field] != 0 for field in previous_cumulative
            ):
                raise ValueError(
                    f"{expected_scenario}: not_run step {step} must be a "
                    "zero-contact zero-counter prefix"
                )
        else:
            if step == 0:
                raise ValueError(f"{expected_scenario}: step 0 must be nan/not_run")
            solver_started = True
            residual = _float_field(raw, "residual", line=line)
            if residual < 0.0:
                raise ValueError(
                    f"{expected_scenario}: step {step} residual is negative"
                )
            if typed["status"] not in {"success", "max_iterations_accepted"}:
                raise ValueError(
                    f"{expected_scenario}: unknown step {step} status "
                    f"{typed['status']!r}"
                )
            max_residual = max(max_residual, residual)
            if typed["status"] != "success" or residual > RESIDUAL_TOLERANCE:
                contract_failure_steps.append(step)
            if typed["status"] == "max_iterations_accepted":
                accepted_at_cap_steps.append(step)
        typed["residual"] = residual
        rows.append(typed)

    if any(row["fallbacks"] != 0 for row in rows):
        raise ValueError(f"{expected_scenario}: boxed-LCP fallback observed")
    if rows[-1]["exact_solves"] <= 0:
        raise ValueError(f"{expected_scenario}: no exact-FBF solve was recorded")
    if not any(row["contacts"] > 0 for row in rows[1:]):
        raise ValueError(f"{expected_scenario}: no contact was recorded")

    history = _turntable_history(rows)
    outcome = _classify_turntable_outcome(rows)
    expected_outcome = SCENARIOS[expected_scenario]["expected_outcome"]
    expected_classification = TRACE_CLASSIFICATION_BY_EXPECTED_OUTCOME[expected_outcome]
    outcome_matches = outcome == expected_classification
    contract_valid = not contract_failure_steps
    if lane == "current_visual":
        if not contract_valid:
            first = contract_failure_steps[0]
            raise ValueError(
                f"{expected_scenario}: current visual trace solver contract failed "
                f"at step {first}"
            )
        if not outcome_matches:
            raise ValueError(
                f"{expected_scenario}: current visual trace outcome {outcome!r} "
                f"does not match {expected_outcome!r}"
            )

    final = rows[-1]
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
    selected = []
    for step in PANEL_STEPS:
        row = rows[step]
        state = {
            key: row[key]
            for key in (
                "step",
                "time",
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
        }
        if not math.isfinite(state["residual"]):
            state["residual"] = None
        selected.append(state)
    return {
        "summary": {
            "scenario": expected_scenario,
            "capture_id": SCENARIOS[expected_scenario]["capture_id"],
            "lane": lane,
            "solver_contract": LANES[lane]["solver_contract"],
            "collision_frontend": LANES[lane]["collision_frontend"],
            "row_count": len(rows),
            "completed_steps": TOTAL_STEPS,
            "exact_solves": final["exact_solves"],
            "warm_starts": final["warm_starts"],
            "boxed_lcp_fallbacks": final["fallbacks"],
            "max_residual": max_residual,
            "accepted_at_cap_steps": accepted_at_cap_steps,
            "solver_contract_failure_steps": contract_failure_steps,
            "solver_contract_valid": contract_valid,
            "expected_outcome": expected_outcome,
            "classified_outcome": outcome,
            "expected_outcome_match": outcome_matches,
            "physical_outcome_authority": lane == "current_visual",
            "support_history_valid": not history["support_fallthrough_steps"],
            **history,
            "selected_panel_states": selected,
            "final": {
                key: final[key]
                for key in (
                    "step",
                    "time",
                    "x",
                    "y",
                    "z",
                    "vx",
                    "vy",
                    "vz",
                    "up_z",
                    "contacts",
                )
            },
        },
        "solver_projection": projection,
        "solver_projection_sha256": _payload_sha256(projection),
    }


def summarize_trace_lanes(
    parsed: dict[str, dict[str, dict[str, Any]]],
) -> dict[str, Any]:
    if set(parsed) != set(LANES):
        raise ValueError("turntable trace lanes are incomplete")
    lane_reports: dict[str, Any] = {}
    for lane, contract in LANES.items():
        by_scenario = parsed[lane]
        if set(by_scenario) != set(SCENARIOS):
            raise ValueError(f"turntable {lane} scenario matrix is incomplete")
        summaries = [by_scenario[name]["summary"] for name in SCENARIOS]
        if any(summary["lane"] != lane for summary in summaries):
            raise ValueError(f"turntable {lane} contains a cross-lane trace")
        lane_reports[lane] = {
            "contract": contract,
            "scenario_order": list(SCENARIOS),
            "scenarios": summaries,
            "all_solver_contract_valid": all(
                summary["solver_contract_valid"] for summary in summaries
            ),
            "all_expected_outcomes_match": all(
                summary["expected_outcome_match"] for summary in summaries
            ),
            "physical_outcome_authority": lane == "current_visual",
        }
    if not lane_reports["current_visual"]["all_solver_contract_valid"]:
        raise ValueError("turntable current visual trace matrix is not solver-valid")
    if not lane_reports["current_visual"]["all_expected_outcomes_match"]:
        raise ValueError("turntable current visual outcome matrix is invalid")
    return {
        "schema_version": TRACE_SUMMARY_SCHEMA_VERSION,
        "pass": True,
        "lanes": lane_reports,
        "render_binding_lane": "current_visual",
        "separate_diagnostic_lane": "paper_cpu_native",
        "cross_lane_substitution_allowed": False,
        "claim_boundary": (
            "Only the four dart_best/Native-collision traces may corroborate the "
            "rendered cells. The paper_cpu/Native traces are separately indexed "
            "diagnostics and do not establish render equivalence or paper parity."
        ),
    }


def artifact_index(root: Path) -> dict[str, Any]:
    root = root.resolve()
    artifacts = []
    for path in sorted(root.rglob("*")):
        if path.is_symlink():
            raise ValueError(f"turntable bundle contains a symlink: {path}")
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
        raise ValueError("unexpected turntable artifact-index schema")
    if payload.get("excluded") != sorted(INDEX_EXCLUSIONS):
        raise ValueError("turntable artifact-index exclusions changed")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("turntable artifact-index artifacts must be a list")
    if payload.get("artifact_count") != len(artifacts):
        raise ValueError("turntable artifact-index count mismatch")
    listed = []
    for item in artifacts:
        if not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("malformed turntable artifact-index entry")
        relative = item["path"]
        if (
            not isinstance(relative, str)
            or relative.startswith(("/", "../"))
            or "/../" in relative
        ):
            raise ValueError(f"unsafe turntable artifact-index path: {relative!r}")
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"turntable artifact-index file missing: {relative}")
        if path.stat().st_size != item["bytes"] or sha256(path) != item["sha256"]:
            raise ValueError(f"turntable artifact-index content changed: {relative}")
        listed.append(relative)
    if listed != sorted(set(listed)):
        raise ValueError("turntable artifact-index paths are duplicate or unsorted")
    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.relative_to(root).as_posix() not in INDEX_EXCLUSIONS
    }
    if actual != set(listed):
        raise ValueError("turntable artifact-index membership changed")


def validate_manual_inspection(root: Path) -> dict[str, Any]:
    root = root.resolve()
    record = read_json(root / "manual-inspection.json")
    if set(record) != {
        "schema_version",
        "manual_inspected",
        "pass",
        "verdicts",
        "representative_artifacts",
    }:
        raise ValueError("turntable manual-inspection top-level contract changed")
    if record.get("schema_version") != MANUAL_SCHEMA_VERSION:
        raise ValueError("unexpected turntable manual-inspection schema")
    if record.get("pass") is not True or record.get("manual_inspected") is not True:
        raise ValueError("turntable manual inspection is not passing")
    if record.get("verdicts") != MANUAL_VERDICTS:
        raise ValueError("turntable manual-inspection verdict contract changed")
    artifacts = record.get("representative_artifacts")
    if not isinstance(artifacts, list) or any(
        not isinstance(item, dict) or set(item) != {"path", "sha256", "observation"}
        for item in artifacts
    ):
        raise ValueError("malformed turntable manual-inspection artifact")
    paths = [item["path"] for item in artifacts]
    if set(paths) != MANUAL_ARTIFACT_PATHS or len(paths) != len(set(paths)):
        raise ValueError("turntable manual-inspection artifact paths changed")
    for item in artifacts:
        artifact = root / item["path"]
        if (
            not artifact.is_file()
            or artifact.is_symlink()
            or sha256(artifact) != item["sha256"]
        ):
            raise ValueError(
                f"turntable manual-inspection artifact changed: {item['path']}"
            )
        if not isinstance(item["observation"], str) or not item["observation"].strip():
            raise ValueError(
                f"turntable manual-inspection observation missing: {item['path']}"
            )
    return record


def _timeline_frame_record(
    root: Path,
    *,
    capture_id: str,
    step: int,
    timeline: dict[str, Any],
    require_staging: bool,
) -> dict[str, Any]:
    frame_path = root / capture_id / "frames" / f"step_{step:06d}.png"
    frames = timeline.get("frames")
    frame = frames.get(str(step)) if isinstance(frames, dict) else None
    if (
        not isinstance(frame, dict)
        or frame.get("path") != str(frame_path)
        or not _is_sha256(frame.get("sha256"))
    ):
        raise ValueError(f"{capture_id}: frame binding changed at step {step}")
    if require_staging:
        if (
            not frame_path.is_file()
            or frame_path.is_symlink()
            or sha256(frame_path) != frame["sha256"]
        ):
            raise ValueError(f"{capture_id}: staging frame changed at step {step}")
    return frame


def _validate_outcome_still(
    root: Path,
    *,
    capture_id: str,
    timeline: dict[str, Any],
) -> dict[str, Any]:
    step = GROUP_PANEL_STEP_BY_CAPTURE[capture_id]
    frame = _timeline_frame_record(
        root,
        capture_id=capture_id,
        step=step,
        timeline=timeline,
        require_staging=False,
    )
    relative = OUTCOME_STILL_PATH_BY_CAPTURE[capture_id]
    still = root / relative
    if not still.is_file() or still.is_symlink() or sha256(still) != frame["sha256"]:
        raise ValueError(
            f"{capture_id}: durable outcome still differs from timeline step {step}"
        )
    return {
        "path": relative,
        "source_frame_path": frame["path"],
        "step": step,
        "sha256": frame["sha256"],
        "timeline_frame_sha256": frame["sha256"],
    }


def _promote_outcome_stills(root: Path) -> list[dict[str, Any]]:
    root = root.resolve()
    bindings = []
    for capture_id in CAPTURE_IDS:
        metadata = read_json(root / capture_id / "metadata.json")
        timeline = metadata.get("timeline_validation")
        if not isinstance(timeline, dict):
            raise ValueError(f"{capture_id}: timeline validation is missing")
        step = GROUP_PANEL_STEP_BY_CAPTURE[capture_id]
        frame = _timeline_frame_record(
            root,
            capture_id=capture_id,
            step=step,
            timeline=timeline,
            require_staging=True,
        )
        source = Path(frame["path"])
        destination = root / OUTCOME_STILL_PATH_BY_CAPTURE[capture_id]
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            raise ValueError(f"{capture_id}: durable outcome still is a symlink")
        shutil.copyfile(source, destination)
        bindings.append(
            _validate_outcome_still(
                root,
                capture_id=capture_id,
                timeline=timeline,
            )
        )
    return bindings


def _prune_capture_staging(root: Path) -> None:
    root = root.resolve()
    for relative in STAGING_PATHS:
        path = root / relative
        if path.is_file() or path.is_symlink():
            path.unlink()
    for relative in STAGING_DIRECTORIES:
        path = root / relative
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
    surviving = [
        relative
        for relative in sorted(STAGING_PATHS)
        if (root / relative).exists() or (root / relative).is_symlink()
    ]
    if surviving:
        raise ValueError(f"turntable ignored staging survived pruning: {surviving}")


def _capture_result_by_id(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    results = payload.get("results")
    if not isinstance(results, list):
        raise ValueError("turntable capture results must be a list")
    by_id: dict[str, dict[str, Any]] = {}
    for result in results:
        if not isinstance(result, dict):
            raise ValueError("turntable capture result must be an object")
        schedule = result.get("schedule")
        schedule_id = (
            schedule.get("id") if isinstance(schedule, dict) else result.get("schedule")
        )
        if not isinstance(schedule_id, str) or schedule_id in by_id:
            raise ValueError("turntable capture identity is missing or duplicated")
        by_id[schedule_id] = result
    return by_id


def _capture_group_by_id(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    groups = payload.get("group_outputs")
    if not isinstance(groups, list):
        raise ValueError("turntable capture groups must be a list")
    by_id: dict[str, dict[str, Any]] = {}
    for group in groups:
        if not isinstance(group, dict):
            raise ValueError("turntable capture group must be an object")
        group_id = group.get("group_id")
        if not isinstance(group_id, str) or group_id in by_id:
            raise ValueError("turntable group identity is missing or duplicated")
        by_id[group_id] = group
    return by_id


def _validate_capture_run_shape(payload: dict[str, Any]) -> None:
    if (
        payload.get("schema_version") != RUNNER_SCHEMA_VERSION
        or payload.get("kind") != "capture_run"
        or payload.get("pass") is not True
        or payload.get("failures") != []
        or payload.get("group_skips") != []
    ):
        raise ValueError("turntable run-summary is not a complete passing capture")
    if tuple(_capture_result_by_id(payload)) != CAPTURE_IDS:
        raise ValueError("turntable run-summary members are not in source order")
    groups = _capture_group_by_id(payload)
    if tuple(groups) != (GROUP_ID,):
        raise ValueError("turntable run-summary group set changed")
    group = groups[GROUP_ID]
    if (
        group.get("member_order") != list(CAPTURE_IDS)
        or group.get("labels") != list(GROUP_LABELS)
        or group.get("layout") != "2x2"
    ):
        raise ValueError("turntable run-summary source-order contract changed")


def _capture_solver_projection(timeline: dict[str, Any]) -> list[dict[str, Any]]:
    steps = timeline.get("steps")
    if not isinstance(steps, dict):
        raise ValueError("turntable capture timeline steps are unavailable")
    projection = []
    for step in range(TOTAL_STEPS + 1):
        entry = steps.get(str(step))
        if not isinstance(entry, dict):
            raise ValueError(f"turntable capture timeline step {step} is missing")
        diagnostics = entry.get("solver_diagnostics")
        if not isinstance(diagnostics, dict):
            raise ValueError(f"turntable capture diagnostics missing at step {step}")
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


def _validate_world_view_motion_hashes(
    frame_hashes: dict[int, Any], *, capture_id: str
) -> int:
    if set(frame_hashes) != set(FRAME_STEPS) or any(
        not isinstance(value, str) or not value for value in frame_hashes.values()
    ):
        raise ValueError(f"{capture_id}: world-view frame hashes are incomplete")
    unique_count = len(set(frame_hashes.values()))
    if unique_count < 2:
        raise ValueError(f"{capture_id}: world-view capture is entirely static")
    for first, second in MOTION_KEYFRAME_PAIRS:
        if frame_hashes[first] == frame_hashes[second]:
            raise ValueError(
                f"{capture_id}: world-view motion keyframes {first}/{second} "
                "are byte-identical"
            )
    return unique_count


def _compare_solver_projections(
    trace_projection: Sequence[dict[str, Any]],
    capture_projection: Sequence[dict[str, Any]],
    *,
    scenario: str,
) -> dict[str, Any]:
    trace_projection = list(trace_projection)
    capture_projection = list(capture_projection)
    if len(trace_projection) != TOTAL_STEPS + 1 or len(capture_projection) != (
        TOTAL_STEPS + 1
    ):
        raise ValueError(f"{scenario}: current trace/capture projection length changed")
    fields = [
        "step",
        "contacts",
        "exact_solves",
        "warm_starts",
        "boxed_lcp_fallbacks",
        "status",
    ]
    core_fields = [
        "step",
        "contacts",
        "exact_solves",
        "boxed_lcp_fallbacks",
        "status",
    ]
    mismatches = []
    core_mismatches = []
    for index, (trace, capture) in enumerate(zip(trace_projection, capture_projection)):
        if set(trace) != set(fields) or set(capture) != set(fields):
            raise ValueError(f"{scenario}: projection fields changed at step {index}")
        differing = [field for field in fields if trace[field] != capture[field]]
        if differing:
            mismatches.append(
                {
                    "step": index,
                    "fields": differing,
                    "trace": {field: trace[field] for field in differing},
                    "capture": {field: capture[field] for field in differing},
                }
            )
        differing_core = [
            field for field in core_fields if trace[field] != capture[field]
        ]
        if differing_core:
            core_mismatches.append((index, differing_core))
    if core_mismatches:
        step, differing = core_mismatches[0]
        raise ValueError(
            f"{scenario}: current trace/capture core projection differs at "
            f"step {step}: {differing}"
        )
    mismatch_fields = sorted(
        {field for mismatch in mismatches for field in mismatch["fields"]}
    )
    if any(field != "warm_starts" for field in mismatch_fields):
        raise ValueError(
            f"{scenario}: unsupported non-core projection drift: {mismatch_fields}"
        )
    trace_digest = _payload_sha256(trace_projection)
    capture_digest = _payload_sha256(capture_projection)
    return {
        "pass": True,
        "scenario": scenario,
        "lane": "current_visual",
        "row_count": len(trace_projection),
        "fields": fields,
        "core_fields": core_fields,
        "trace_sha256": trace_digest,
        "capture_sha256": capture_digest,
        "byte_identical": not mismatches,
        "core_projection_equivalent": True,
        "mismatch_count": len(mismatches),
        "mismatch_steps": [item["step"] for item in mismatches],
        "mismatch_fields": mismatch_fields,
        "first_mismatch": mismatches[0] if mismatches else None,
        "paper_cpu_native_compared": False,
        "full_state_trace_equivalence": False,
        "limitation": (
            "The core comparison covers contacts, exact solves, fallbacks, and "
            "status for the visual-compatible dart_best/Native-collision lane. "
            "Warm-start-count drift is retained explicitly. This is not full-state "
            "equivalence."
        ),
    }


def _expected_schedule_output(root: Path, capture_id: str) -> dict[str, str]:
    member = root / capture_id
    return {
        "directory": str(member),
        "timeline": str(member / "timeline.json"),
        "panel": str(member / "panel.png"),
        "mp4": str(member / "clip.mp4"),
        "gif": None,
    }


def _expected_demo_argv(
    root: Path, *, capture_id: str, scene: str, demo: Path
) -> list[str]:
    member_root = root / capture_id
    argv = [
        str(demo),
        "--scene",
        scene,
        "--headless",
        "--steps",
        str(TOTAL_STEPS),
        "--width",
        "1280",
        "--height",
        "720",
        "--threads",
        "1",
        "--headless-sidecar",
        str(member_root / "timeline.json"),
    ]
    for step in FRAME_STEPS:
        argv.extend(
            (
                "--headless-shot-at",
                f"{step}:{member_root / 'frames' / f'step_{step:06d}.png'}",
            )
        )
    return argv


def _validate_final_solver_diagnostics(
    diagnostics: Any, *, capture_id: str
) -> dict[str, Any]:
    if not isinstance(diagnostics, dict):
        raise ValueError(f"{capture_id}: solver diagnostics changed")
    integer_fields = (
        "exact_attempts",
        "exact_solves",
        "accepted_at_cap",
        "exact_failures",
        "boxed_lcp_fallbacks",
    )
    if any(
        not isinstance(diagnostics.get(field), int)
        or isinstance(diagnostics.get(field), bool)
        for field in integer_fields
    ):
        raise ValueError(f"{capture_id}: solver diagnostics changed")
    residual = diagnostics.get("worst_residual")
    if (
        diagnostics.get("available") is not True
        or diagnostics["exact_attempts"] <= 0
        or diagnostics["exact_solves"] <= 0
        or diagnostics["accepted_at_cap"] != 0
        or diagnostics["exact_failures"] != 0
        or diagnostics["boxed_lcp_fallbacks"] != 0
        or diagnostics.get("status") != "success"
        or not isinstance(residual, (int, float))
        or isinstance(residual, bool)
        or not math.isfinite(float(residual))
        or float(residual) < 0.0
        or float(residual) > RESIDUAL_TOLERANCE
    ):
        raise ValueError(f"{capture_id}: solver diagnostics changed")
    return diagnostics


def _validate_member_capture(
    root: Path,
    *,
    scenario: str,
    metadata: dict[str, Any],
    verified: dict[str, Any],
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
    require_staging: bool,
) -> dict[str, Any]:
    contract = SCENARIOS[scenario]
    capture_id = contract["capture_id"]
    member_root = root / capture_id
    metadata_path = member_root / "metadata.json"
    if (
        metadata.get("schema_version") != RUNNER_SCHEMA_VERSION
        or metadata.get("kind") != "capture_result"
        or metadata.get("pass") is not True
        or metadata.get("actual_simulator") is not True
        or metadata.get("generated_imagery") is not False
        or metadata.get("paper_comparable") is not False
        or metadata.get("automated_semantic_outcome_validated") is not False
    ):
        raise ValueError(f"{capture_id}: capture claim flags changed")
    schedule = metadata.get("schedule")
    expected_configuration = {
        "author_commit": AUTHOR_COMMIT,
        "mu": contract["mu"],
        "omega_rad_s": contract["omega"],
        "outcome": contract["expected_outcome"],
        "drop_height_m": "0.2",
        "settle_seconds": "0.5",
        "smoothstep_ramp_seconds": "0.5",
    }
    if (
        not isinstance(schedule, dict)
        or schedule.get("id") != capture_id
        or schedule.get("scene") != contract["scene"]
        or schedule.get("source_segment") != SOURCE_SEGMENT
        or schedule.get("collision_detector") != "native"
        or schedule.get("collision_detector_override") is not False
        or schedule.get("total_steps") != TOTAL_STEPS
        or schedule.get("time_step_seconds") != DT_SECONDS
        or schedule.get("capture_steps") != list(FRAME_STEPS)
        or schedule.get("panel_steps") != list(PANEL_STEPS)
        or schedule.get("panel_labels") != list(PANEL_LABELS)
        or schedule.get("actions") != []
        or schedule.get("configuration") != expected_configuration
        or schedule.get("output") != _expected_schedule_output(root, capture_id)
        or schedule.get("demo_argv")
        != _expected_demo_argv(
            root, capture_id=capture_id, scene=contract["scene"], demo=demo
        )
        or schedule.get("runnable") is not True
        or schedule.get("adapter_gap") is not None
        or schedule.get("long_run") is not False
        or schedule.get("actual_simulator_required") is not True
        or schedule.get("generated_imagery_allowed") is not False
        or schedule.get("paper_comparable") is not False
        or schedule.get("known_gate_blockers") != []
        or schedule.get("evidence_ready") is not True
    ):
        raise ValueError(f"{capture_id}: capture schedule contract changed")
    mismatches = metadata.get("known_mismatches")
    if mismatches != schedule.get("known_mismatches") or not all(
        any(fragment in mismatch for mismatch in mismatches)
        for fragment in (
            "ports the public author geometry and schedule",
            "5 mm author collision-gap setting",
            "faithful synchronized external-solver renderings",
        )
    ):
        raise ValueError(f"{capture_id}: source-gap disclosures changed")

    runtime = metadata.get("runtime")
    if (
        not isinstance(runtime, dict)
        or runtime.get("demo_path") != str(demo)
        or runtime.get("demo_sha256") != sha256(demo)
        or runtime.get("demo_argv") != schedule.get("demo_argv")
        or runtime.get("ffmpeg") != str(ffmpeg)
        or runtime.get("ffprobe") != str(ffprobe)
    ):
        raise ValueError(f"{capture_id}: runtime identity changed")

    timeline = metadata.get("timeline_validation")
    if not isinstance(timeline, dict):
        raise ValueError(f"{capture_id}: timeline validation is missing")
    expected_step_keys = {str(step) for step in range(TOTAL_STEPS + 1)}
    expected_frame_keys = {str(step) for step in FRAME_STEPS}
    unique_frame_hashes = timeline.get("unique_frame_hashes")
    if (
        timeline.get("pass") is not True
        or timeline.get("step_count") != TOTAL_STEPS + 1
        or timeline.get("shot_count") != len(FRAME_STEPS)
        or timeline.get("action_count") != 0
        or type(unique_frame_hashes) is not int
        or not 2 <= unique_frame_hashes <= len(FRAME_STEPS)
        or set(timeline.get("steps", {})) != expected_step_keys
        or set(timeline.get("frames", {})) != expected_frame_keys
    ):
        raise ValueError(f"{capture_id}: timeline structure changed")
    sidecar_path = member_root / "timeline.json"
    sidecar_physics_contract = _validate_capture_sidecar_physics_contract(
        sidecar_path, demo=demo, scenario=scenario
    )
    for step in FRAME_STEPS:
        _timeline_frame_record(
            root,
            capture_id=capture_id,
            step=step,
            timeline=timeline,
            require_staging=require_staging,
        )
    outcome_still = _validate_outcome_still(
        root,
        capture_id=capture_id,
        timeline=timeline,
    )
    world_hashes = {
        step: timeline["frames"][str(step)].get("world_viewport", {}).get("sha256")
        for step in FRAME_STEPS
    }
    unique_world_frames = _validate_world_view_motion_hashes(
        world_hashes, capture_id=capture_id
    )

    diagnostics = _validate_final_solver_diagnostics(
        timeline.get("final_solver_diagnostics"), capture_id=capture_id
    )

    panel = metadata.get("panel_validation")
    panel_path = member_root / "panel.png"
    compose_path = member_root / "panel.compose.json"
    panel_sources = panel.get("source_frames") if isinstance(panel, dict) else None
    expected_panel_source_paths = [
        str(member_root / "panel_frames" / f"step_{step:06d}.png")
        for step in PANEL_STEPS
    ]
    if (
        not isinstance(panel_sources, list)
        or len(panel_sources) != len(PANEL_STEPS)
        or [item.get("path") for item in panel_sources] != expected_panel_source_paths
        or any(not _is_sha256(item.get("sha256")) for item in panel_sources)
    ):
        raise ValueError(f"{capture_id}: panel source-frame contract changed")
    if require_staging:
        for source in panel_sources:
            path = Path(source["path"])
            if (
                not path.is_file()
                or path.is_symlink()
                or sha256(path) != source["sha256"]
            ):
                raise ValueError(f"{capture_id}: panel staging source changed")
    if (
        not isinstance(panel, dict)
        or panel.get("path") != str(panel_path)
        or panel.get("sha256") != sha256(panel_path)
        or panel.get("compose_manifest_path") != str(compose_path)
        or panel.get("compose_manifest_sha256") != sha256(compose_path)
        or panel.get("compose_manifest") != read_json(compose_path)
        or panel.get("verdict", {}).get("pass") is not True
        or panel.get("verdict", {}).get("image", {}).get("width") != 3348
        or panel.get("verdict", {}).get("image", {}).get("height") != 538
    ):
        raise ValueError(f"{capture_id}: panel contract changed")

    media = metadata.get("media_validation")
    if not isinstance(media, list) or len(media) != 1:
        raise ValueError(f"{capture_id}: expected exactly one member MP4")
    clip = media[0]
    clip_path = member_root / "clip.mp4"
    expected_stream = {
        "width": 660,
        "height": 506,
        "frame_rate": f"{OUTPUT_FPS}/1",
        "frame_rate_rational": f"{OUTPUT_FPS}/1",
        "frame_count": VIDEO_FRAME_COUNT,
    }
    if (
        clip.get("kind") != "mp4"
        or clip.get("path") != str(clip_path)
        or clip.get("sha256") != sha256(clip_path)
        or clip.get("full_decode") != "pass"
        or clip.get("stream_contract") != expected_stream
    ):
        raise ValueError(f"{capture_id}: member MP4 contract changed")

    if (
        verified.get("pass") is not True
        or verified.get("metadata_path") != str(metadata_path)
        or verified.get("metadata_sha256") != sha256(metadata_path)
        or verified.get("timeline") != timeline
        or verified.get("panel", {}).get("pass") is not True
        or verified.get("panel", {}).get("image", {}).get("path") != str(panel_path)
    ):
        raise ValueError(f"{capture_id}: fresh verification binding changed")
    verified_media = verified.get("media")
    if not isinstance(verified_media, list) or len(verified_media) != 1:
        raise ValueError(f"{capture_id}: verified media set changed")
    if any(
        verified_media[0].get(key) != clip.get(key)
        for key in ("kind", "path", "sha256", "full_decode", "stream_contract")
    ):
        raise ValueError(f"{capture_id}: stored/fresh media binding differs")

    projection = _capture_solver_projection(timeline)
    return {
        "scenario": scenario,
        "capture_id": capture_id,
        "metadata_sha256": sha256(metadata_path),
        "timeline_sha256": sha256(sidecar_path),
        "physics_contract_sha256": _payload_sha256(sidecar_physics_contract),
        "physics_contract_visual_asset_identity": None,
        "panel_sha256": sha256(panel_path),
        "clip_sha256": sha256(clip_path),
        "outcome_still": outcome_still,
        "captured_frames": len(FRAME_STEPS),
        "unique_frames": unique_world_frames,
        "decoded_video_frames": VIDEO_FRAME_COUNT,
        "exact_attempts": diagnostics["exact_attempts"],
        "exact_solves": diagnostics["exact_solves"],
        "accepted_at_cap": diagnostics["accepted_at_cap"],
        "exact_failures": diagnostics["exact_failures"],
        "boxed_lcp_fallbacks": diagnostics["boxed_lcp_fallbacks"],
        "worst_residual": diagnostics["worst_residual"],
        "solver_projection": projection,
        "solver_projection_sha256": _payload_sha256(projection),
        "pass": True,
    }


def _expected_group_crop_commands(root: Path, ffmpeg: Path) -> list[list[str]]:
    group_root = root / "groups" / GROUP_ID
    return [
        [
            str(ffmpeg),
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(
                root
                / capture_id
                / "frames"
                / f"step_{GROUP_PANEL_STEP_BY_CAPTURE[capture_id]:06d}.png"
            ),
            "-vf",
            "crop=660:506:260:58",
            "-frames:v",
            "1",
            str(group_root / "panel_members" / f"{capture_id}.png"),
        ]
        for capture_id in CAPTURE_IDS
    ]


def _expected_group_row_command(root: Path, python: Path, row: int) -> list[str]:
    if row not in (0, 1):
        raise ValueError(f"unsupported turntable group row: {row}")
    group_root = root / "groups" / GROUP_ID
    start = row * 2
    capture_ids = CAPTURE_IDS[start : start + 2]
    labels = GROUP_PANEL_LABELS[start : start + 2]
    destination = group_root / ("panel_top.png" if row == 0 else "panel_bottom.png")
    return [
        str(python),
        str(IMAGE_COMPOSE),
        "side-by-side",
        *(
            str(group_root / "panel_members" / f"{capture_id}.png")
            for capture_id in capture_ids
        ),
        "--out",
        str(destination),
        "--labels",
        *labels,
        "--gap",
        "8",
        "--manifest",
        str(group_root / f"panel_row_{row}.compose.json"),
    ]


def _expected_group_stack_command(root: Path, ffmpeg: Path) -> list[str]:
    group_root = root / "groups" / GROUP_ID
    return [
        str(ffmpeg),
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(group_root / "panel_top.png"),
        "-i",
        str(group_root / "panel_bottom.png"),
        "-filter_complex",
        "[0:v][1:v]vstack=inputs=2[outv]",
        "-map",
        "[outv]",
        "-frames:v",
        "1",
        str(group_root / "panel.png"),
    ]


def _expected_group_media_command(root: Path, ffmpeg: Path) -> list[str]:
    group_root = root / "groups" / GROUP_ID
    filters = [
        (
            f"[{index}:v]setpts=PTS-STARTPTS,"
            "pad=iw:ih+24:0:24:color=0x181c20,"
            f"drawtext=text='{label}':x=8:y=5:fontsize=14:fontcolor=white"
            f"[v{index}]"
        )
        for index, label in enumerate(GROUP_LABELS)
    ]
    filters.extend(
        (
            "[v0][v1]hstack=inputs=2:shortest=1[top]",
            "[v2][v3]hstack=inputs=2:shortest=1[bottom]",
            "[top][bottom]vstack=inputs=2:shortest=1[outv]",
        )
    )
    command = [str(ffmpeg), "-hide_banner", "-loglevel", "error", "-y"]
    for capture_id in CAPTURE_IDS:
        command.extend(("-i", str(root / capture_id / "clip.mp4")))
    command.extend(
        (
            "-filter_complex",
            ";".join(filters),
            "-map",
            "[outv]",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "24",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(group_root / "clip.mp4"),
        )
    )
    return command


def _validate_group_media_command(
    command: Any, *, root: Path, ffmpeg: Path
) -> list[str]:
    expected = _expected_group_media_command(root, ffmpeg)
    if command != expected:
        raise ValueError("turntable group MP4 source/label order changed")
    return expected


def _validate_group_capture(
    root: Path,
    *,
    group: dict[str, Any],
    verified: dict[str, Any],
    python: Path,
    ffmpeg: Path,
    require_staging: bool,
) -> dict[str, Any]:
    group_root = root / "groups" / GROUP_ID
    metadata_path = group_root / "metadata.json"
    if (
        group.get("schema_version") != RUNNER_SCHEMA_VERSION
        or group.get("kind") != "group_capture_result"
        or group.get("group_id") != GROUP_ID
        or group.get("source_segment") != SOURCE_SEGMENT
        or group.get("layout") != "2x2"
        or group.get("member_order") != list(CAPTURE_IDS)
        or group.get("labels") != list(GROUP_LABELS)
        or group.get("actual_simulator") is not True
        or group.get("generated_imagery") is not False
        or group.get("paper_comparable") is not False
        or group.get("automated_semantic_outcome_validated") is not False
        or group.get("pass") is not True
    ):
        raise ValueError("turntable group identity/order/claim flags changed")

    synchronization = group.get("synchronization")
    if (
        not isinstance(synchronization, dict)
        or synchronization.get("total_steps") != TOTAL_STEPS
        or synchronization.get("video_steps") != list(FRAME_STEPS)
        or synchronization.get("output_fps") != OUTPUT_FPS
        or synchronization.get("frame_count") != VIDEO_FRAME_COUNT
        or synchronization.get("frame_rate") != f"{OUTPUT_FPS}/1"
        or not math.isfinite(float(synchronization.get("duration_seconds", math.nan)))
        or abs(float(synchronization["duration_seconds"]) - VIDEO_DURATION_SECONDS)
        > float(synchronization.get("duration_tolerance_seconds", -1.0))
    ):
        raise ValueError("turntable group synchronization contract changed")

    members = group.get("members")
    if not isinstance(members, list) or [
        member.get("id") for member in members
    ] != list(CAPTURE_IDS):
        raise ValueError("turntable group members are out of source order")
    for capture_id, member in zip(CAPTURE_IDS, members):
        member_root = root / capture_id
        panel_step = GROUP_PANEL_STEP_BY_CAPTURE[capture_id]
        panel_time = panel_step * DT_SECONDS
        member_metadata = read_json(member_root / "metadata.json")
        member_timeline = member_metadata.get("timeline_validation")
        if not isinstance(member_timeline, dict):
            raise ValueError(f"{capture_id}: group member timeline is missing")
        source_frame = _timeline_frame_record(
            root,
            capture_id=capture_id,
            step=panel_step,
            timeline=member_timeline,
            require_staging=require_staging,
        )
        _validate_outcome_still(
            root,
            capture_id=capture_id,
            timeline=member_timeline,
        )
        expected = {
            "metadata_path": str(member_root / "metadata.json"),
            "metadata_sha256": sha256(member_root / "metadata.json"),
            "timeline_path": str(member_root / "timeline.json"),
            "timeline_sha256": sha256(member_root / "timeline.json"),
            "panel_source_step": panel_step,
            "panel_source_time_seconds": panel_time,
            "panel_source_frame": str(
                member_root / "frames" / f"step_{panel_step:06d}.png"
            ),
            "panel_source_frame_sha256": source_frame["sha256"],
            "clip_path": str(member_root / "clip.mp4"),
            "clip_sha256": sha256(member_root / "clip.mp4"),
        }
        if any(member.get(key) != value for key, value in expected.items()):
            raise ValueError(f"turntable group member binding changed: {capture_id}")

    panel = group.get("panel_validation")
    panel_path = group_root / "panel.png"
    expected_sources = []
    for capture_id in CAPTURE_IDS:
        step = GROUP_PANEL_STEP_BY_CAPTURE[capture_id]
        member_root = root / capture_id
        timeline = read_json(member_root / "metadata.json").get("timeline_validation")
        if not isinstance(timeline, dict):
            raise ValueError(f"{capture_id}: group panel timeline is missing")
        frame = _timeline_frame_record(
            root,
            capture_id=capture_id,
            step=step,
            timeline=timeline,
            require_staging=require_staging,
        )
        expected_sources.append(
            {
                "member": capture_id,
                "step": step,
                "time_seconds": step * DT_SECONDS,
                "path": frame["path"],
                "sha256": frame["sha256"],
            }
        )
    expected_tiles = [
        str(group_root / "panel_members" / f"{capture_id}.png")
        for capture_id in CAPTURE_IDS
    ]
    if (
        not isinstance(panel, dict)
        or "panel_step" in panel
        or panel.get("member_order") != list(CAPTURE_IDS)
        or panel.get("labels") != list(GROUP_PANEL_LABELS)
        or panel.get("source_frames") != expected_sources
        or [tile.get("path") for tile in panel.get("cropped_tiles", [])]
        != expected_tiles
        or panel.get("path") != str(panel_path)
        or panel.get("sha256") != sha256(panel_path)
        or panel.get("verdict", {}).get("pass") is not True
        or panel.get("verdict", {}).get("image", {}).get("width") != 1344
        or panel.get("verdict", {}).get("image", {}).get("height") != 1076
        or panel.get("full_decode") != "pass"
        or panel.get("crop_commands") != _expected_group_crop_commands(root, ffmpeg)
        or panel.get("stack_command") != _expected_group_stack_command(root, ffmpeg)
    ):
        raise ValueError("turntable group panel contract changed")
    for tile in panel["cropped_tiles"]:
        if tile.get("sha256") != sha256(Path(tile["path"])):
            raise ValueError("turntable group panel tile hash changed")
    rows = panel.get("row_compositions")
    if not isinstance(rows, list) or len(rows) != 2:
        raise ValueError("turntable group panel rows changed")
    flattened_inputs: list[str] = []
    flattened_labels: list[str] = []
    for index, row in enumerate(rows):
        manifest_path = group_root / f"panel_row_{index}.compose.json"
        row_path = group_root / ("panel_top.png" if index == 0 else "panel_bottom.png")
        manifest = read_json(manifest_path)
        if (
            row.get("manifest_path") != str(manifest_path)
            or row.get("manifest_sha256") != sha256(manifest_path)
            or row.get("manifest") != manifest
            or row.get("path") != str(row_path)
            or row.get("sha256") != sha256(row_path)
            or row.get("command") != _expected_group_row_command(root, python, index)
        ):
            raise ValueError(f"turntable group panel row {index} binding changed")
        flattened_inputs.extend(manifest.get("inputs", []))
        flattened_labels.extend(manifest.get("labels", []))
    if flattened_inputs != expected_tiles or flattened_labels != list(
        GROUP_PANEL_LABELS
    ):
        raise ValueError("turntable group panel source order changed")

    media = group.get("media_validation")
    clip_path = group_root / "clip.mp4"
    probe_streams = (
        media.get("probe", {}).get("streams", []) if isinstance(media, dict) else []
    )
    video_streams = [stream for stream in probe_streams if stream.get("width")]
    if (
        not isinstance(media, dict)
        or media.get("kind") != "mp4"
        or media.get("path") != str(clip_path)
        or media.get("sha256") != sha256(clip_path)
        or media.get("full_decode") != "pass"
        or len(video_streams) != 1
        or video_streams[0].get("width") != 1320
        or video_streams[0].get("height") != 1060
        or video_streams[0].get("r_frame_rate") != f"{OUTPUT_FPS}/1"
        or video_streams[0].get("nb_frames") != str(VIDEO_FRAME_COUNT)
    ):
        raise ValueError("turntable group MP4 contract changed")
    _validate_group_media_command(media.get("command"), root=root, ffmpeg=ffmpeg)

    if (
        verified.get("group_id") != GROUP_ID
        or verified.get("member_order") != list(CAPTURE_IDS)
        or verified.get("pass") is not True
        or verified.get("metadata_path") != str(metadata_path)
        or verified.get("metadata_sha256") != sha256(metadata_path)
        or verified.get("panel")
        != {"path": str(panel_path), "sha256": sha256(panel_path)}
        or verified.get("media", {}).get("path") != str(clip_path)
        or verified.get("media", {}).get("sha256") != sha256(clip_path)
    ):
        raise ValueError("turntable group fresh verification binding changed")
    return {
        "metadata_sha256": sha256(metadata_path),
        "panel_sha256": sha256(panel_path),
        "clip_sha256": sha256(clip_path),
        "width": 1320,
        "height": 1060,
        "frame_rate": f"{OUTPUT_FPS}/1",
        "frame_count": VIDEO_FRAME_COUNT,
        "outcome_panel_steps": list(GROUP_PANEL_STEPS),
        "outcome_panel_times_seconds": [
            step * DT_SECONDS for step in GROUP_PANEL_STEPS
        ],
        "member_order": list(CAPTURE_IDS),
        "labels": list(GROUP_LABELS),
        "panel_labels": list(GROUP_PANEL_LABELS),
        "layout": "2x2",
        "full_decode": "pass",
        "pass": True,
    }


def validate_capture_bundle(
    root: Path,
    *,
    demo: Path,
    python: Path,
    ffmpeg: Path,
    ffprobe: Path,
    verification: dict[str, Any] | None = None,
    require_staging: bool = False,
) -> dict[str, Any]:
    root = root.resolve()
    demo = _require_file(demo, "DART demo binary")
    python = _require_file(python, "Python interpreter")
    ffmpeg = _require_file(ffmpeg, "ffmpeg")
    ffprobe = _require_file(ffprobe, "ffprobe")

    run_summary = read_json(root / "run-summary.json")
    _validate_capture_run_shape(run_summary)
    run_results = _capture_result_by_id(run_summary)
    run_groups = _capture_group_by_id(run_summary)

    verification = verification or read_json(root / "verification.json")
    if (
        verification.get("schema_version") != RUNNER_SCHEMA_VERSION
        or verification.get("kind") != "verification"
        or verification.get("pass") is not True
    ):
        raise ValueError("turntable capture verification is not passing")
    verify_results = _capture_result_by_id(verification)
    verify_groups = _capture_group_by_id(verification)
    if tuple(verify_results) != CAPTURE_IDS or tuple(verify_groups) != (GROUP_ID,):
        raise ValueError("turntable capture verification member/group set changed")

    scenarios = []
    capture_projections: dict[str, list[dict[str, Any]]] = {}
    for scenario, contract in SCENARIOS.items():
        capture_id = contract["capture_id"]
        metadata = read_json(root / capture_id / "metadata.json")
        if run_results[capture_id] != metadata:
            raise ValueError(
                f"{capture_id}: run-summary no longer equals member metadata"
            )
        report = _validate_member_capture(
            root,
            scenario=scenario,
            metadata=metadata,
            verified=verify_results[capture_id],
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            require_staging=require_staging,
        )
        capture_projections[scenario] = report.pop("solver_projection")
        scenarios.append(report)

    group = read_json(root / "groups" / GROUP_ID / "metadata.json")
    if run_groups[GROUP_ID] != group:
        raise ValueError("turntable run-summary no longer equals group metadata")
    group_report = _validate_group_capture(
        root,
        group=group,
        verified=verify_groups[GROUP_ID],
        python=python,
        ffmpeg=ffmpeg,
        require_staging=require_staging,
    )
    return {
        "pass": True,
        "scenarios": scenarios,
        "group": group_report,
        "run_summary_sha256": sha256(root / "run-summary.json"),
        "verification_sha256": sha256(root / "verification.json"),
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "paper_comparable": False,
        "capture_solver_projections": capture_projections,
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


def _run_trace_command(
    argv: Sequence[str], *, lane: str, timeout: float
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
    allowed_returncodes = {0} if lane == "current_visual" else {0, 1}
    if process.returncode not in allowed_returncodes:
        raise ValueError(
            f"trace command failed with exit {process.returncode}: {list(argv)!r}\n"
            f"stderr:\n{process.stderr}"
        )
    return process


def _expected_trace_returncode(summary: dict[str, Any]) -> int:
    if summary.get("lane") == "current_visual":
        if summary.get("solver_contract_valid") is not True:
            raise ValueError("current visual trace cannot carry a failing result")
        return 0
    if summary.get("lane") == "paper_cpu_native":
        return 0 if summary.get("solver_contract_valid") is True else 1
    raise ValueError("trace summary has an unknown lane")


def _trace_argv(trace_binary: Path, scenario: str, lane: str) -> list[str]:
    if scenario not in SCENARIOS:
        raise ValueError(f"unsupported turntable scenario: {scenario}")
    if lane not in LANES:
        raise ValueError(f"unsupported turntable trace lane: {lane}")
    contract = LANES[lane]
    return [
        str(trace_binary),
        scenario,
        "exact_fbf",
        "1",
        str(TOTAL_STEPS),
        "nan",
        "tracked",
        "default",
        "default",
        "1",
        contract["solver_contract"],
        contract["collision_frontend"],
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
    argv = [str(python), str(runner), "verify"]
    for capture_id in CAPTURE_IDS:
        argv.extend(("--scenario", capture_id))
    argv.extend(
        (
            "--demo",
            str(demo),
            "--output-root",
            str(bundle),
            "--ffmpeg",
            str(ffmpeg),
            "--ffprobe",
            str(ffprobe),
        )
    )
    return argv


def _capture_argv(
    python: Path,
    runner: Path,
    demo: Path,
    bundle: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> list[str]:
    argv = [str(python), str(runner), "run"]
    for capture_id in CAPTURE_IDS:
        argv.extend(("--scenario", capture_id))
    argv.extend(
        (
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
        )
    )
    return argv


def _identity(paths: dict[str, Path]) -> dict[str, dict[str, str]]:
    return {
        name: {
            "path": str(_require_file(path, name)),
            "sha256": sha256(_require_file(path, name)),
        }
        for name, path in paths.items()
    }


def _repository_resource_identity(path: Path, *, label: str) -> dict[str, Any]:
    path = _require_file(path, label)
    try:
        relative = path.relative_to(ROOT.resolve(strict=True)).as_posix()
    except ValueError as error:
        raise ValueError(f"{label} is outside the repository: {path}") from error
    return {
        "relative_path": relative,
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def _turntable_visual_resource_identity() -> dict[str, dict[str, Any]]:
    return {
        "author_turntable_visual_obj": _repository_resource_identity(
            AUTHOR_TURNTABLE_VISUAL_OBJ, label="author turntable visual OBJ"
        ),
        "author_turntable_visual_mtl": _repository_resource_identity(
            AUTHOR_TURNTABLE_VISUAL_MTL, label="author turntable visual MTL"
        ),
    }


def _parse_obj_index(token: str, *, label: str) -> int:
    if "/" in token:
        raise ValueError(f"{label}: texture/normal OBJ indices are not expected")
    try:
        value = int(token)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{label}: invalid OBJ index {token!r}") from error
    if value <= 0 or str(value) != token:
        raise ValueError(f"{label}: noncanonical OBJ index {token!r}")
    return value


def _validate_turntable_visual_resources() -> dict[str, Any]:
    obj_path = _require_file(AUTHOR_TURNTABLE_VISUAL_OBJ, "author turntable visual OBJ")
    mtl_path = _require_file(AUTHOR_TURNTABLE_VISUAL_MTL, "author turntable visual MTL")

    vertices: list[tuple[float, float, float]] = []
    faces: list[tuple[str, tuple[int, ...]]] = []
    material_switches: list[str] = []
    mtllibs: list[str] = []
    object_names: list[str] = []
    active_material: str | None = None
    allowed_obj_commands = {"mtllib", "o", "v", "usemtl", "f"}
    for line_number, raw in enumerate(
        obj_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        command = fields[0]
        if command not in allowed_obj_commands:
            raise ValueError(
                f"author turntable visual OBJ line {line_number}: "
                f"unsupported command {command!r}"
            )
        if command == "mtllib":
            if len(fields) != 2:
                raise ValueError("author turntable visual OBJ mtllib changed")
            mtllibs.append(fields[1])
        elif command == "o":
            if len(fields) != 2:
                raise ValueError("author turntable visual OBJ object changed")
            object_names.append(fields[1])
        elif command == "v":
            if len(fields) != 4:
                raise ValueError("author turntable visual OBJ vertex changed")
            try:
                vertex = tuple(float(value) for value in fields[1:])
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError(
                    "author turntable visual OBJ has an invalid vertex"
                ) from error
            if len(vertex) != 3 or not all(math.isfinite(value) for value in vertex):
                raise ValueError("author turntable visual OBJ has a non-finite vertex")
            vertices.append(vertex)
        elif command == "usemtl":
            if len(fields) != 2:
                raise ValueError("author turntable visual OBJ material changed")
            active_material = fields[1]
            material_switches.append(active_material)
        else:
            if active_material is None or len(fields) not in (4, 5):
                raise ValueError("author turntable visual OBJ face changed")
            indices = tuple(
                _parse_obj_index(
                    token,
                    label=f"author turntable visual OBJ line {line_number}",
                )
                for token in fields[1:]
            )
            faces.append((active_material, indices))

    expected_top_materials = (
        "FbfTurntableCoral",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
        "FbfTurntableDark",
        "FbfTurntableLight",
    )
    expected_switches = expected_top_materials + (
        "FbfTurntableSide",
        "FbfTurntableBottom",
    )
    if mtllibs != [mtl_path.name] or object_names != ["FbfAuthorTurntableDisc"]:
        raise ValueError("author turntable visual OBJ resource linkage changed")
    if len(vertices) != 130 or len(faces) != 192:
        raise ValueError("author turntable visual OBJ geometry counts changed")
    if tuple(material_switches) != expected_switches:
        raise ValueError("author turntable visual OBJ material sectors changed")

    expected_vertices = [(0.0, 0.0, 0.5)]
    expected_vertices.extend(
        (
            math.cos(2.0 * math.pi * index / 64.0),
            math.sin(2.0 * math.pi * index / 64.0),
            0.5,
        )
        for index in range(64)
    )
    expected_vertices.append((0.0, 0.0, -0.5))
    expected_vertices.extend(
        (
            math.cos(2.0 * math.pi * index / 64.0),
            math.sin(2.0 * math.pi * index / 64.0),
            -0.5,
        )
        for index in range(64)
    )
    if any(
        not all(
            math.isclose(actual, expected, rel_tol=0.0, abs_tol=5.0e-10)
            for actual, expected in zip(vertex, expected_vertex)
        )
        for vertex, expected_vertex in zip(vertices, expected_vertices)
    ):
        raise ValueError("author turntable visual OBJ unit-cylinder vertices changed")

    expected_top_faces = []
    for index in range(64):
        expected_top_faces.append(
            (
                expected_top_materials[index // 8],
                (1, 2 + index, 2 + ((index + 1) % 64)),
            )
        )
    expected_side_faces = [
        (
            "FbfTurntableSide",
            (
                2 + index,
                67 + index,
                67 + ((index + 1) % 64),
                2 + ((index + 1) % 64),
            ),
        )
        for index in range(64)
    ]
    expected_bottom_faces = [
        (
            "FbfTurntableBottom",
            (66, 67 + ((index + 1) % 64), 67 + index),
        )
        for index in range(64)
    ]
    if faces != expected_top_faces + expected_side_faces + expected_bottom_faces:
        raise ValueError("author turntable visual OBJ segmented faces changed")

    materials: dict[str, dict[str, tuple[float, ...] | float | int]] = {}
    active_name: str | None = None
    allowed_mtl_fields = {"Ka", "Kd", "Ks", "Ke", "Ns", "d", "illum"}
    for line_number, raw in enumerate(
        mtl_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if fields[0] == "newmtl":
            if len(fields) != 2 or fields[1] in materials:
                raise ValueError("author turntable visual MTL material changed")
            active_name = fields[1]
            materials[active_name] = {}
            continue
        if active_name is None or fields[0] not in allowed_mtl_fields:
            raise ValueError(
                f"author turntable visual MTL line {line_number}: field changed"
            )
        field = fields[0]
        if field in materials[active_name]:
            raise ValueError("author turntable visual MTL duplicate field")
        expected_count = 3 if field in {"Ka", "Kd", "Ks", "Ke"} else 1
        if len(fields) != expected_count + 1:
            raise ValueError("author turntable visual MTL value count changed")
        try:
            values = tuple(float(value) for value in fields[1:])
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("author turntable visual MTL value changed") from error
        if not all(math.isfinite(value) for value in values):
            raise ValueError("author turntable visual MTL has a non-finite value")
        materials[active_name][field] = values if expected_count == 3 else values[0]

    expected_materials = {
        "FbfTurntableCoral": (0.94, 0.31, 0.27),
        "FbfTurntableLight": (0.76, 0.78, 0.80),
        "FbfTurntableDark": (0.39, 0.42, 0.45),
        "FbfTurntableSide": (0.50, 0.52, 0.54),
        "FbfTurntableBottom": (0.32, 0.34, 0.36),
    }
    if set(materials) != set(expected_materials) or any(
        set(properties) != allowed_mtl_fields for properties in materials.values()
    ):
        raise ValueError("author turntable visual MTL material set changed")
    if any(
        materials[name]["Kd"] != expected
        for name, expected in expected_materials.items()
    ):
        raise ValueError("author turntable visual MTL diffuse colors changed")
    coral = materials["FbfTurntableCoral"]["Kd"]
    if not isinstance(coral, tuple) or not (
        coral[0] >= 0.9 and 0.2 <= coral[1] <= 0.4 and coral[2] < 0.35
    ):
        raise ValueError("author turntable visual coral registration color changed")

    return {
        "pass": True,
        "resources": _turntable_visual_resource_identity(),
        "unit_geometry": "radius_1_height_1_cylinder",
        "vertex_count": len(vertices),
        "face_count": len(faces),
        "top_wedge_count": 8,
        "top_triangles_per_wedge": 8,
        "top_material_order": list(expected_top_materials),
        "registration_material": "FbfTurntableCoral",
        "material_diffuse_rgb": {
            name: list(color) for name, color in expected_materials.items()
        },
    }


def _extract_cpp_function(source: str, signature: str) -> str:
    start = source.find(signature)
    if start < 0:
        raise ValueError(f"turntable demo visual attachment is missing {signature}")
    brace = source.find("{", start + len(signature))
    if brace < 0:
        raise ValueError("turntable demo visual attachment function is malformed")
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise ValueError("turntable demo visual attachment function is unterminated")


def _validate_turntable_visual_attachment() -> dict[str, Any]:
    source = _require_file(DEMO_SOURCE, "turntable demo source").read_text(
        encoding="utf-8"
    )
    function = _extract_cpp_function(
        source, "SkeletonPtr createAuthorTurntableSupport(double friction)"
    )
    compact = " ".join(function.split())
    required = (
        "std::make_shared<CylinderShape>( fbf_author_turntable::kSupportRadius, "
        "2.0 * fbf_author_turntable::kSupportHalfHeight)",
        "createShapeNodeWith<CollisionAspect, DynamicsAspect>( collisionShape)",
        '"dart://sample/obj/fbf_author_turntable_disc.obj"',
        "MeshShape::loadMesh(meshUri, retriever)",
        "fbf_author_turntable::kSupportRadius, "
        "fbf_author_turntable::kSupportRadius, "
        "2.0 * fbf_author_turntable::kSupportHalfHeight",
        "createShapeNodeWith<VisualAspect>(visualShape)",
    )
    if any(fragment not in compact for fragment in required):
        raise ValueError("turntable demo visual resource attachment contract changed")
    forbidden = (
        "createShapeNodeWith<CollisionAspect>(visualShape)",
        "createShapeNodeWith<DynamicsAspect>(visualShape)",
        "createShapeNodeWith<VisualAspect, CollisionAspect>(visualShape)",
        "createShapeNodeWith<VisualAspect, DynamicsAspect>(visualShape)",
    )
    if any(fragment in compact for fragment in forbidden):
        raise ValueError("turntable visual mesh is no longer VisualAspect-only")
    return {
        "pass": True,
        "mesh_uri": "dart://sample/obj/fbf_author_turntable_disc.obj",
        "collision_geometry": "shared_spec_cylinder",
        "visual_geometry": "scaled_obj_mesh",
        "visual_aspect_only": True,
        "scale": [
            "shared_spec_support_radius",
            "shared_spec_support_radius",
            "shared_spec_support_height",
        ],
        "demo_source_sha256": sha256(DEMO_SOURCE),
    }


def _query_json_command(argv: Sequence[str], *, label: str) -> dict[str, Any]:
    process = _run_command(argv, timeout=30.0)
    if process.stderr:
        raise ValueError(f"{label} emitted unexpected stderr")
    return _parse_json_stdout(process.stdout, label=label)


def _query_demo_author_turntable_physics_contract(
    demo: Path, *, scenario: str
) -> dict[str, Any]:
    if scenario not in SCENARIOS:
        raise ValueError(f"unsupported author turntable scenario: {scenario}")
    return _query_json_command(
        [
            str(_require_file(demo, "DART demo binary")),
            "--fbf-author-turntable-contract",
            SCENARIOS[scenario]["scene"],
        ],
        label=f"demo author-turntable contract {scenario}",
    )


def _query_trace_author_turntable_physics_contract(
    trace_binary: Path, *, scenario: str, solver_contract: str
) -> dict[str, Any]:
    if scenario not in SCENARIOS or solver_contract not in {"dart_best", "paper_cpu"}:
        raise ValueError("unsupported trace author-turntable contract query")
    return _query_json_command(
        [
            str(_require_file(trace_binary, "trace binary")),
            "--author-turntable-contract",
            scenario,
            solver_contract,
        ],
        label=f"trace author-turntable contract {scenario}/{solver_contract}",
    )


def _query_author_turntable_physics_contracts(
    *, demo: Path, trace_binary: Path
) -> dict[str, Any]:
    demo = _require_file(demo, "DART demo binary")
    trace_binary = _require_file(trace_binary, "trace binary")
    demo_contracts: dict[str, Any] = {}
    trace_contracts: dict[str, dict[str, Any]] = {
        "dart_best": {},
        "paper_cpu": {},
    }
    for scenario in SCENARIOS:
        demo_contracts[scenario] = _query_demo_author_turntable_physics_contract(
            demo, scenario=scenario
        )
        for solver_contract in trace_contracts:
            trace_contracts[solver_contract][scenario] = (
                _query_trace_author_turntable_physics_contract(
                    trace_binary,
                    scenario=scenario,
                    solver_contract=solver_contract,
                )
            )
    return {"demo": demo_contracts, "trace": trace_contracts}


def _physics_contract_without_binary_binding(payload: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(payload)
    normalized.pop("binary_binding", None)
    return normalized


def _require_finite_number(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} is not a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} is not finite")
    return result


def _require_close(value: Any, expected: float, *, label: str) -> None:
    if not math.isclose(
        _require_finite_number(value, label=label),
        expected,
        rel_tol=1.0e-12,
        abs_tol=1.0e-12,
    ):
        raise ValueError(f"{label} changed")


def _require_vector(value: Any, expected: Sequence[float], *, label: str) -> None:
    if not isinstance(value, list) or len(value) != len(expected):
        raise ValueError(f"{label} shape changed")
    for index, (actual, target) in enumerate(zip(value, expected)):
        _require_close(actual, target, label=f"{label}[{index}]")


def _require_pose(value: Any, translation: Sequence[float], *, label: str) -> None:
    if not isinstance(value, dict) or set(value) != {"translation", "rotation"}:
        raise ValueError(f"{label} fields changed")
    _require_vector(value["translation"], translation, label=f"{label}.translation")
    rotation = value["rotation"]
    if not isinstance(rotation, list) or len(rotation) != 3:
        raise ValueError(f"{label}.rotation shape changed")
    for row, expected_row in zip(rotation, ((1, 0, 0), (0, 1, 0), (0, 0, 1))):
        _require_vector(row, expected_row, label=f"{label}.rotation")


def _validate_author_turntable_physics_contract(
    payload: Any,
    *,
    scenario: str,
    solver_contract: str,
    binary_role: str,
    implementation_source: Path,
) -> dict[str, Any]:
    if scenario not in SCENARIOS or not isinstance(payload, dict):
        raise ValueError("author turntable physics contract is missing")
    expected_top_fields = {
        "schema_version",
        "kind",
        "author_source",
        "physics_spec_source_sha256",
        "binary_binding",
        "scenario",
        "world",
        "collision",
        "solver",
        "support",
        "rider",
        "control",
        "visual_asset_identity",
    }
    if set(payload) != expected_top_fields:
        raise ValueError("author turntable physics contract fields changed")
    if (
        payload.get("schema_version") != PHYSICS_CONTRACT_SCHEMA_VERSION
        or payload.get("kind") != PHYSICS_CONTRACT_KIND
        or payload.get("author_source")
        != {
            "commit": AUTHOR_COMMIT,
            "turntable_run_py_sha256": AUTHOR_TURNTABLE_SOURCE_SHA256,
        }
        or payload.get("physics_spec_source_sha256")
        != sha256(AUTHOR_TURNTABLE_SPEC_SOURCE)
        or payload.get("visual_asset_identity") is not None
    ):
        raise ValueError("author turntable shared physics identity changed")
    if payload.get("binary_binding") != {
        "role": binary_role,
        "implementation_source_sha256": sha256(implementation_source),
    }:
        raise ValueError("author turntable source-to-binary binding changed")

    index = list(SCENARIOS).index(scenario)
    configuration = SCENARIOS[scenario]
    expected_scenario = {
        "trace_id": scenario,
        "demo_scene_id": configuration["scene"],
        "friction": float(configuration["mu"]),
        "angular_velocity_rad_s": float(configuration["omega"]),
        "expected_outcome": PHYSICS_EXPECTED_OUTCOMES[index],
    }
    if payload.get("scenario") != expected_scenario:
        raise ValueError(f"{scenario}: physics scenario contract changed")

    world = payload.get("world")
    if not isinstance(world, dict) or set(world) != {
        "time_step_seconds",
        "gravity_m_s2",
        "simulation_threads",
        "deactivation_enabled",
    }:
        raise ValueError(f"{scenario}: physics world contract changed")
    _require_close(world["time_step_seconds"], DT_SECONDS, label="world time step")
    _require_vector(world["gravity_m_s2"], (0.0, 0.0, -9.81), label="gravity")
    if world["simulation_threads"] != 1 or world["deactivation_enabled"] is not False:
        raise ValueError(f"{scenario}: physics world runtime settings changed")

    if payload.get("collision") != {
        "detector": "native",
        "contact_manifold": "four_point_planar",
        "max_contacts": 4,
        "max_contacts_per_pair": 4,
    }:
        raise ValueError(f"{scenario}: physics collision contract changed")

    solver = payload.get("solver")
    solver_fields = {
        "type",
        "contract",
        "split_impulse_enabled",
        "max_outer_iterations",
        "tolerance",
        "inner_max_sweeps",
        "inner_local_iterations",
        "step_size_scale",
        "warm_start_enabled",
        "fallback_to_boxed_lcp_enabled",
        "projected_gradient_retry_enabled",
        "dense_residual_polish_enabled",
        "contact_row_operator_enabled",
        "dense_contact_row_snapshot_enabled",
    }
    if not isinstance(solver, dict) or set(solver) != solver_fields:
        raise ValueError(f"{scenario}: physics solver fields changed")
    expected_solver = {
        "dart_best": {
            "type": "exact_fbf",
            "contract": "dart_best",
            "split_impulse_enabled": False,
            "max_outer_iterations": 500,
            "tolerance": 1.0e-6,
            "inner_max_sweeps": 120,
            "inner_local_iterations": 32,
            "step_size_scale": 2.0,
            "warm_start_enabled": True,
            "fallback_to_boxed_lcp_enabled": True,
            "projected_gradient_retry_enabled": True,
            "dense_residual_polish_enabled": True,
            "contact_row_operator_enabled": True,
            "dense_contact_row_snapshot_enabled": True,
        },
        "paper_cpu": {
            "type": "exact_fbf",
            "contract": "paper_cpu",
            "split_impulse_enabled": False,
            "max_outer_iterations": 200,
            "tolerance": 1.0e-6,
            "inner_max_sweeps": 10,
            "inner_local_iterations": 1,
            "step_size_scale": 1.0,
            "warm_start_enabled": True,
            "fallback_to_boxed_lcp_enabled": False,
            "projected_gradient_retry_enabled": False,
            "dense_residual_polish_enabled": False,
            "contact_row_operator_enabled": True,
            "dense_contact_row_snapshot_enabled": False,
        },
    }
    if (
        solver_contract not in expected_solver
        or solver != expected_solver[solver_contract]
    ):
        raise ValueError(f"{scenario}: physics solver contract changed")

    friction = float(configuration["mu"])
    support = payload.get("support")
    if not isinstance(support, dict) or set(support) != {
        "shape",
        "mobile",
        "radius_m",
        "height_m",
        "friction",
        "initial_pose",
    }:
        raise ValueError(f"{scenario}: physics support fields changed")
    if support["shape"] != "cylinder" or support["mobile"] is not False:
        raise ValueError(f"{scenario}: physics support type changed")
    _require_close(support["radius_m"], 2.0, label="support radius")
    _require_close(support["height_m"], 0.1, label="support height")
    _require_close(support["friction"], friction, label="support friction")
    _require_pose(support["initial_pose"], (0.0, 0.0, 0.05), label="support pose")

    rider = payload.get("rider")
    if not isinstance(rider, dict) or set(rider) != {
        "shape",
        "size_m",
        "friction",
        "mass_kg",
        "moment_kg_m2",
        "initial_pose",
        "initial_linear_velocity_m_s",
        "initial_angular_velocity_rad_s",
    }:
        raise ValueError(f"{scenario}: physics rider fields changed")
    if rider["shape"] != "box":
        raise ValueError(f"{scenario}: physics rider shape changed")
    _require_vector(rider["size_m"], (0.3, 0.3, 0.3), label="rider size")
    _require_close(rider["friction"], friction, label="rider friction")
    mass = 500.0 * 0.3 * 0.3 * 0.3
    moment = mass * (0.3 * 0.3 + 0.3 * 0.3) / 12.0
    _require_close(rider["mass_kg"], mass, label="rider mass")
    moment_matrix = rider["moment_kg_m2"]
    if not isinstance(moment_matrix, list) or len(moment_matrix) != 3:
        raise ValueError(f"{scenario}: rider moment shape changed")
    for row, expected_row in zip(
        moment_matrix,
        ((moment, 0.0, 0.0), (0.0, moment, 0.0), (0.0, 0.0, moment)),
    ):
        _require_vector(row, expected_row, label="rider moment")
    _require_pose(rider["initial_pose"], (1.0, 0.0, 0.455), label="rider pose")
    _require_vector(
        rider["initial_linear_velocity_m_s"],
        (0.0, 0.0, 0.0),
        label="rider linear velocity",
    )
    _require_vector(
        rider["initial_angular_velocity_rad_s"],
        (0.0, 0.0, 0.0),
        label="rider angular velocity",
    )

    if payload.get("control") != {
        "settle_duration_seconds": 0.5,
        "ramp_duration_seconds": 0.5,
        "duration_seconds": 6.0,
        "ramp": "cubic_smoothstep",
    }:
        raise ValueError(f"{scenario}: physics control schedule changed")
    return payload


def _validate_author_turntable_physics_contracts(
    payload: Any, *, demo: Path, trace_binary: Path
) -> dict[str, Any]:
    if not isinstance(payload, dict) or set(payload) != {"demo", "trace"}:
        raise ValueError("author turntable physics query matrix changed")
    demo_contracts = payload["demo"]
    trace_contracts = payload["trace"]
    if (
        not isinstance(demo_contracts, dict)
        or tuple(demo_contracts) != tuple(SCENARIOS)
        or not isinstance(trace_contracts, dict)
        or tuple(trace_contracts) != ("dart_best", "paper_cpu")
    ):
        raise ValueError("author turntable physics query order changed")
    for contracts in trace_contracts.values():
        if not isinstance(contracts, dict) or tuple(contracts) != tuple(SCENARIOS):
            raise ValueError("author turntable trace contract matrix changed")

    for scenario in SCENARIOS:
        demo_contract = _validate_author_turntable_physics_contract(
            demo_contracts[scenario],
            scenario=scenario,
            solver_contract="dart_best",
            binary_role="dart_demos",
            implementation_source=DEMO_SOURCE,
        )
        trace_best = _validate_author_turntable_physics_contract(
            trace_contracts["dart_best"][scenario],
            scenario=scenario,
            solver_contract="dart_best",
            binary_role="fbf_paper_trace",
            implementation_source=TRACE_SOURCE,
        )
        trace_paper = _validate_author_turntable_physics_contract(
            trace_contracts["paper_cpu"][scenario],
            scenario=scenario,
            solver_contract="paper_cpu",
            binary_role="fbf_paper_trace",
            implementation_source=TRACE_SOURCE,
        )
        if _physics_contract_without_binary_binding(
            demo_contract
        ) != _physics_contract_without_binary_binding(trace_best):
            raise ValueError(
                f"{scenario}: demo/trace dart_best physics contracts differ"
            )
        best_non_solver = _physics_contract_without_binary_binding(trace_best)
        paper_non_solver = _physics_contract_without_binary_binding(trace_paper)
        best_non_solver.pop("solver")
        paper_non_solver.pop("solver")
        if best_non_solver != paper_non_solver:
            raise ValueError(
                f"{scenario}: strict trace changed geometry/control contract"
            )
    return payload


def _validate_capture_sidecar_physics_contract(
    sidecar_path: Path, *, demo: Path, scenario: str
) -> dict[str, Any]:
    sidecar = read_json(_require_file(sidecar_path, "turntable capture sidecar"))
    persisted = _validate_author_turntable_physics_contract(
        sidecar.get("physics_contract"),
        scenario=scenario,
        solver_contract="dart_best",
        binary_role="dart_demos",
        implementation_source=DEMO_SOURCE,
    )
    current = _validate_author_turntable_physics_contract(
        _query_demo_author_turntable_physics_contract(demo, scenario=scenario),
        scenario=scenario,
        solver_contract="dart_best",
        binary_role="dart_demos",
        implementation_source=DEMO_SOURCE,
    )
    if persisted != current:
        raise ValueError(
            f"{SCENARIOS[scenario]['capture_id']}: timeline physics contract changed"
        )
    return persisted


def _query_runner_author_group_contract(
    *, python: Path, runner: Path, demo: Path
) -> dict[str, Any]:
    argv: list[str] = [str(python), str(runner), "plan"]
    for capture_id in CAPTURE_IDS:
        argv.extend(("--scenario", capture_id))
    argv.extend(("--demo", str(demo), "--output-root", str(ROOT)))
    plan = _query_json_command(argv, label="turntable visual runner plan")
    schedules = plan.get("schedules")
    groups = plan.get("group_outputs")
    group = groups.get(GROUP_ID) if isinstance(groups, dict) else None
    expected_panel_sources = [
        {
            "member": capture_id,
            "step": step,
            "time_seconds": step * DT_SECONDS,
        }
        for capture_id, step in zip(CAPTURE_IDS, GROUP_PANEL_STEPS)
    ]
    if (
        plan.get("schema_version") != RUNNER_SCHEMA_VERSION
        or plan.get("kind") != "capture_plan"
        or plan.get("pass") is not True
        or not isinstance(schedules, list)
        or [schedule.get("id") for schedule in schedules] != list(CAPTURE_IDS)
        or not isinstance(group, dict)
        or group.get("members") != list(CAPTURE_IDS)
        or group.get("labels") != list(GROUP_LABELS)
        or group.get("layout") != "2x2 in source order"
        or "panel_step" in group
        or group.get("panel_sources") != expected_panel_sources
        or group.get("panel_labels") != list(GROUP_PANEL_LABELS)
    ):
        raise ValueError("turntable visual-runner group contract changed")
    return {
        "member_order": list(CAPTURE_IDS),
        "labels": list(GROUP_LABELS),
        "layout": "2x2",
        "panel_steps": list(GROUP_PANEL_STEPS),
        "panel_labels": list(GROUP_PANEL_LABELS),
        "plan_payload_sha256": _payload_sha256(plan),
    }


def _parse_ldd_in_tree_paths(output: str, *, build_root: Path) -> list[Path]:
    paths: set[Path] = set()
    for line in output.splitlines():
        if "not found" in line:
            raise ValueError(f"unresolved runtime dependency: {line.strip()}")
        candidate = ""
        if "=>" in line:
            fields = line.split("=>", 1)[1].strip().split()
            if fields:
                candidate = fields[0]
        else:
            fields = line.strip().split()
            if fields:
                candidate = fields[0]
        if not candidate.startswith("/"):
            continue
        path = Path(candidate).resolve(strict=True)
        try:
            path.relative_to(build_root.resolve(strict=True))
        except ValueError:
            continue
        paths.add(path)
    return sorted(paths, key=lambda path: path.as_posix())


def _in_tree_runtime_dependency_identity(
    executable: Path, *, label: str
) -> dict[str, dict[str, str]]:
    executable = _require_file(executable, f"{label} executable")
    process = subprocess.run(
        ["ldd", str(executable)],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    dependencies = _parse_ldd_in_tree_paths(
        process.stdout, build_root=BUILD_RUNTIME_ROOT
    )
    if not any(path.name.startswith("libdart.so") for path in dependencies):
        raise ValueError(f"{label}: resolved in-tree libdart dependency is missing")
    return {
        f"{label}_dependency_{path.relative_to(BUILD_RUNTIME_ROOT.resolve()).as_posix().replace('/', '__')}": {
            "path": str(path),
            "sha256": sha256(path),
        }
        for path in dependencies
    }


def _capture_runtime_identity(
    *, demo: Path, ffmpeg: Path, ffprobe: Path, python: Path
) -> dict[str, dict[str, str]]:
    return {
        **_identity(
            {
                "demo_binary": demo,
                "ffmpeg_binary": ffmpeg,
                "ffprobe_binary": ffprobe,
                "python_binary": python,
            }
        ),
        **_in_tree_runtime_dependency_identity(demo, label="demo"),
    }


def _capture_source_identity(*, runner: Path) -> dict[str, dict[str, Any]]:
    return {
        **_identity(
            {
                "visual_runner": runner,
                "visual_runner_test": RUNNER_TEST,
                "image_compose": IMAGE_COMPOSE,
                "image_tools": IMAGE_TOOLS,
                "image_verdict": IMAGE_VERDICT,
                "demo_source": DEMO_SOURCE,
                "author_turntable_spec_source": AUTHOR_TURNTABLE_SPEC_SOURCE,
                "demo_cmake_source": DEMO_CMAKE_SOURCE,
                "demo_main_source": DEMO_MAIN_SOURCE,
                "demo_scene_header": DEMO_SCENE_HEADER,
                "demo_host_source": DEMO_HOST_SOURCE,
                "demo_host_header": DEMO_HOST_HEADER,
                "registry_source": REGISTRY_SOURCE,
                "scenes_header": SCENES_HEADER,
                "native_cylinder_source": NATIVE_CYLINDER_SOURCE,
                "native_cylinder_header": NATIVE_CYLINDER_HEADER,
                "native_narrow_phase_source": NATIVE_NARROW_PHASE_SOURCE,
                "native_narrow_phase_header": NATIVE_NARROW_PHASE_HEADER,
            }
        ),
        **_turntable_visual_resource_identity(),
    }


def _trace_runtime_identity(*, trace_binary: Path) -> dict[str, dict[str, str]]:
    return {
        **_identity({"trace_binary": trace_binary}),
        **_in_tree_runtime_dependency_identity(trace_binary, label="trace"),
    }


def _trace_source_identity() -> dict[str, dict[str, str]]:
    return _identity(
        {
            "trace_source": TRACE_SOURCE,
            "trace_cmake_source": TRACE_CMAKE_SOURCE,
            "fixture_source": FIXTURE_SOURCE,
            "author_turntable_spec_source": AUTHOR_TURNTABLE_SPEC_SOURCE,
            "exact_fbf_source": EXACT_FBF_SOURCE,
            "exact_fbf_header": EXACT_FBF_HEADER,
            "dart_collision_source": DART_COLLISION_SOURCE,
            "dart_collision_header": DART_COLLISION_HEADER,
            "native_collision_source": NATIVE_COLLISION_SOURCE,
            "native_collision_header": NATIVE_COLLISION_HEADER,
            "native_cylinder_source": NATIVE_CYLINDER_SOURCE,
            "native_cylinder_header": NATIVE_CYLINDER_HEADER,
            "native_narrow_phase_source": NATIVE_NARROW_PHASE_SOURCE,
            "native_narrow_phase_header": NATIVE_NARROW_PHASE_HEADER,
        }
    )


def _validate_trace_matrix_provenance(
    payload: Any, *, trace_binary: Path
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("turntable trace-matrix provenance is missing")
    runtime_identity = _trace_runtime_identity(trace_binary=trace_binary)
    source_identity = _trace_source_identity()
    expected = {
        "cwd": str(ROOT),
        "runtime_resources_before": runtime_identity,
        "runtime_resources_after": runtime_identity,
        "trace_sources_before": source_identity,
        "trace_sources_after": source_identity,
    }
    if payload != expected:
        raise ValueError("turntable trace-matrix provenance changed")
    return payload


def _source_identity(
    *,
    trace_binary: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, dict[str, Any]]:
    return {
        **_identity(
            {
                "finalizer": Path(__file__).resolve(),
                "finalizer_test": FINALIZER_TEST,
                "visual_runner": runner,
                "visual_runner_test": RUNNER_TEST,
                "image_compose": IMAGE_COMPOSE,
                "image_tools": IMAGE_TOOLS,
                "image_verdict": IMAGE_VERDICT,
                "trace_source": TRACE_SOURCE,
                "trace_cmake_source": TRACE_CMAKE_SOURCE,
                "fixture_source": FIXTURE_SOURCE,
                "exact_fbf_source": EXACT_FBF_SOURCE,
                "exact_fbf_header": EXACT_FBF_HEADER,
                "dart_collision_source": DART_COLLISION_SOURCE,
                "dart_collision_header": DART_COLLISION_HEADER,
                "native_collision_source": NATIVE_COLLISION_SOURCE,
                "native_collision_header": NATIVE_COLLISION_HEADER,
                "native_cylinder_source": NATIVE_CYLINDER_SOURCE,
                "native_cylinder_header": NATIVE_CYLINDER_HEADER,
                "native_narrow_phase_source": NATIVE_NARROW_PHASE_SOURCE,
                "native_narrow_phase_header": NATIVE_NARROW_PHASE_HEADER,
                "demo_source": DEMO_SOURCE,
                "author_turntable_spec_source": AUTHOR_TURNTABLE_SPEC_SOURCE,
                "demo_cmake_source": DEMO_CMAKE_SOURCE,
                "demo_main_source": DEMO_MAIN_SOURCE,
                "demo_scene_header": DEMO_SCENE_HEADER,
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
        ),
        **_turntable_visual_resource_identity(),
        **_in_tree_runtime_dependency_identity(demo, label="demo"),
        **_in_tree_runtime_dependency_identity(trace_binary, label="trace"),
    }


def _validate_capture_source_contract(
    runner: Path = DEFAULT_RUNNER,
    *,
    demo: Path = DEFAULT_DEMO,
    trace_binary: Path = DEFAULT_TRACE_BINARY,
    python: Path = DEFAULT_PYTHON,
) -> dict[str, Any]:
    runner = _require_file(runner, "turntable visual runner source")
    demo = _require_file(demo, "DART demo binary")
    trace_binary = _require_file(trace_binary, "trace binary")
    python = _require_file(python, "Python interpreter")
    author_source = {
        "url": AUTHOR_REPOSITORY,
        "commit": AUTHOR_COMMIT,
        "license": "MIT",
        "scene_source_sha256": {
            "turntable/run.py": AUTHOR_TURNTABLE_SOURCE_SHA256,
        },
    }
    renderer_resources = _validate_turntable_visual_resources()
    visual_attachment = _validate_turntable_visual_attachment()
    runner_contract = _query_runner_author_group_contract(
        python=python, runner=runner, demo=demo
    )
    physics_contracts = _validate_author_turntable_physics_contracts(
        _query_author_turntable_physics_contracts(demo=demo, trace_binary=trace_binary),
        demo=demo,
        trace_binary=trace_binary,
    )
    return {
        "pass": True,
        "member_order": list(CAPTURE_IDS),
        "labels": list(GROUP_LABELS),
        "layout": "2x2",
        "panel_steps": list(GROUP_PANEL_STEPS),
        "panel_labels": list(GROUP_PANEL_LABELS),
        "support_geometry": "author_pinned_cylinder_radius_2_height_0.1",
        "source_support_geometry": "cylinder_radius_2_height_0.1",
        "cube_edge_length_m": 0.3,
        "cube_density_kg_m3": 500.0,
        "initial_geometric_gap_m": 0.005,
        "drop_height_m": 0.2,
        "settle_duration_seconds": 0.5,
        "ramp_duration_seconds": 0.5,
        "capture_duration_seconds": 6.0,
        "capture_collision_frontend": "native_four_point_planar",
        "collision_override_applied_after_scene_install": False,
        "author_numerical_source_available": True,
        "author_turntable_renderer_assets_available": True,
        "renderer_resources": renderer_resources,
        "visual_attachment": visual_attachment,
        "runner_group_contract": runner_contract,
        "physics_contract_queries": physics_contracts,
        "physics_contract_queries_payload_sha256": _payload_sha256(physics_contracts),
        "physics_contract_visual_asset_identity": None,
        "source_golden_available": False,
        "author_source": author_source,
        "author_source_payload_sha256": _payload_sha256(author_source),
        "author_turntable_spec_source_sha256": sha256(AUTHOR_TURNTABLE_SPEC_SOURCE),
        "demo_source_sha256": sha256(DEMO_SOURCE),
        "demo_query_source_sha256": sha256(DEMO_MAIN_SOURCE),
        "trace_query_source_sha256": sha256(TRACE_SOURCE),
        "demo_cmake_source_sha256": sha256(DEMO_CMAKE_SOURCE),
        "trace_cmake_source_sha256": sha256(TRACE_CMAKE_SOURCE),
        "demo_host_source_sha256": sha256(DEMO_HOST_SOURCE),
        "runner_source_sha256": sha256(runner),
    }


def _validate_capture_provenance(
    payload: Any,
    *,
    bundle: Path,
    python: Path,
    runner: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("turntable capture provenance is missing")
    run_summary = read_json(bundle / "run-summary.json")
    _validate_capture_run_shape(run_summary)
    runtime_identity = _capture_runtime_identity(
        demo=demo, ffmpeg=ffmpeg, ffprobe=ffprobe, python=python
    )
    source_identity = _capture_source_identity(runner=runner)
    outcome_stills = []
    for capture_id in CAPTURE_IDS:
        metadata = read_json(bundle / capture_id / "metadata.json")
        timeline = metadata.get("timeline_validation")
        if not isinstance(timeline, dict):
            raise ValueError(f"{capture_id}: timeline validation is missing")
        outcome_stills.append(
            _validate_outcome_still(
                bundle,
                capture_id=capture_id,
                timeline=timeline,
            )
        )
    if set(payload) != {
        "argv",
        "cwd",
        "returncode",
        "run_summary_path",
        "run_summary_sha256",
        "run_summary_validated",
        "capture_stdout_sha256",
        "capture_stdout_ends_with_run_summary",
        "capture_stderr_sha256",
        "verification_stderr_sha256",
        "outcome_stills",
        "staging_pruned",
        "runtime_resources_before",
        "runtime_resources_after",
        "capture_sources_before",
        "capture_sources_after",
    }:
        raise ValueError("turntable capture provenance fields changed")
    expected = {
        "argv": _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "cwd": str(ROOT),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "capture_stdout_ends_with_run_summary": True,
        "outcome_stills": outcome_stills,
        "runtime_resources_before": runtime_identity,
        "runtime_resources_after": runtime_identity,
        "capture_sources_before": source_identity,
        "capture_sources_after": source_identity,
    }
    if any(payload.get(key) != value for key, value in expected.items()):
        raise ValueError("turntable capture-time provenance changed")
    if (
        not _is_sha256(payload.get("capture_stdout_sha256"))
        or not _is_sha256(payload.get("capture_stderr_sha256"))
        or type(payload.get("staging_pruned")) is not bool
    ):
        raise ValueError("turntable capture log provenance is malformed")
    if payload["staging_pruned"]:
        if not _is_sha256(payload.get("verification_stderr_sha256")):
            raise ValueError("turntable verification stderr provenance is malformed")
    elif payload.get("verification_stderr_sha256") is not None:
        raise ValueError("turntable transitional verification provenance changed")
    if not payload["staging_pruned"]:
        stdout = bundle / "capture.stdout.txt"
        stderr = bundle / "capture.stderr.txt"
        expected_summary_stdout = (
            json.dumps(run_summary, indent=2, sort_keys=True) + "\n"
        )
        if (
            not stdout.is_file()
            or stdout.is_symlink()
            or sha256(stdout) != payload["capture_stdout_sha256"]
            or not stdout.read_text(encoding="utf-8").endswith(expected_summary_stdout)
            or not stderr.is_file()
            or stderr.is_symlink()
            or sha256(stderr) != payload["capture_stderr_sha256"]
        ):
            raise ValueError("turntable transitional capture logs changed")
    return payload


def _trace_relative_path(lane: str, scenario: str, suffix: str) -> str:
    return f"traces/{lane}/{scenario}{suffix}"


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
    if set(payload) != {
        "schema_version",
        "trace_matrix_provenance",
        "traces",
        "capture_verification",
    }:
        raise ValueError("turntable invocation fields changed")
    if payload.get("schema_version") != INVOCATIONS_SCHEMA_VERSION:
        raise ValueError("unexpected turntable invocations schema")
    _validate_trace_matrix_provenance(
        payload.get("trace_matrix_provenance"), trace_binary=trace_binary
    )
    traces = payload.get("traces")
    if not isinstance(traces, list) or len(traces) != len(LANES) * len(SCENARIOS):
        raise ValueError("turntable trace invocation matrix changed")
    expected = []
    for lane in LANES:
        for scenario in SCENARIOS:
            stdout_path = _trace_relative_path(lane, scenario, ".csv")
            stderr_path = _trace_relative_path(lane, scenario, ".stderr.txt")
            parsed = parse_trace_text(
                (bundle / stdout_path).read_text(encoding="utf-8"),
                scenario,
                lane=lane,
            )
            expected.append(
                {
                    "lane": lane,
                    "scenario": scenario,
                    "argv": _trace_argv(trace_binary, scenario, lane),
                    "returncode": _expected_trace_returncode(parsed["summary"]),
                    "stdout_path": stdout_path,
                    "stdout_sha256": sha256(bundle / stdout_path),
                    "stderr_path": stderr_path,
                    "stderr_sha256": sha256(bundle / stderr_path),
                }
            )
    if traces != expected:
        raise ValueError("turntable trace invocation ordering/binding changed")
    verification = payload.get("capture_verification")
    provenance = read_json(bundle / "capture-provenance.json")
    if verification != {
        "argv": _verification_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "returncode": 0,
        "stdout_path": "verification.json",
        "stdout_sha256": sha256(bundle / "verification.json"),
        "stderr_sha256": provenance.get("verification_stderr_sha256"),
        "stderr_retained": False,
    }:
        raise ValueError("turntable verification invocation binding changed")
    return payload


def _build_trace_summary(
    parsed: dict[str, dict[str, dict[str, Any]]],
    capture_summary: dict[str, Any],
) -> dict[str, Any]:
    summary = summarize_trace_lanes(parsed)
    comparisons = []
    capture_projections = capture_summary["capture_solver_projections"]
    for scenario in SCENARIOS:
        comparisons.append(
            _compare_solver_projections(
                parsed["current_visual"][scenario]["solver_projection"],
                capture_projections[scenario],
                scenario=scenario,
            )
        )
    if set(capture_projections) != set(SCENARIOS):
        raise ValueError("turntable capture projection matrix is incomplete")
    byte_identical_count = sum(item["byte_identical"] for item in comparisons)
    return {
        **summary,
        "visual_capture_projection_comparisons": comparisons,
        "all_visual_capture_core_projections_equal": True,
        "all_visual_capture_full_projections_byte_identical": (
            byte_identical_count == len(SCENARIOS)
        ),
        "full_projection_byte_identical_count": byte_identical_count,
        "paper_cpu_native_projection_compared_to_capture": False,
    }


def _report_markdown(
    trace_summary: dict[str, Any], capture_summary: dict[str, Any]
) -> str:
    current = trace_summary["lanes"]["current_visual"]
    strict = trace_summary["lanes"]["paper_cpu_native"]
    strict_failures = [
        scenario["scenario"]
        for scenario in strict["scenarios"]
        if not scenario["solver_contract_valid"]
    ]
    strict_failure_text = (
        ", ".join(strict_failures) if strict_failures else "none in this replay"
    )
    comparisons = trace_summary["visual_capture_projection_comparisons"]
    nonidentical = [item for item in comparisons if not item["byte_identical"]]
    warm_start_disclosure = (
        "none"
        if not nonidentical
        else "; ".join(
            f"{item['scenario']} at {item['mismatch_count']} steps "
            f"({','.join(item['mismatch_fields'])})"
            for item in nonidentical
        )
    )
    group = capture_summary["group"]
    return f"""# Figure 4 author-source-pinned DART turntable evidence

Status: valid DART port of the public author turntable configuration at
`{AUTHOR_COMMIT}`; not Warp/Newton trace parity or historical paper-render parity.

The capture contains the source-ordered 2x2 matrix: top row mu=.2 at omega=2
and 5 rad/s, then bottom row mu=.5 at omega=2 and 5 rad/s. The synchronized
group clip is {group['width']}x{group['height']} at {group['frame_rate']} with
{group['frame_count']} decoded frames. Manual inspection records the three
ejections and the mu=.5, omega=2 cell retained on the support through 6 s.
Retention is not a zero-slip or co-rotation claim.

Four fresh current_visual traces use exact FBF with the dart_best contract and
the Native FourPointPlanar collision frontend. They complete {TOTAL_STEPS} steps with no boxed-LCP
fallbacks, all solved residuals at or below {RESIDUAL_TOLERANCE:.1e}, and the
expected ejected/ejected/retained-through-6s/ejected physical outcome order.
Their four complete
core projections (contacts, exact solves, fallbacks, and status) equal the
corresponding capture sidecars. Full six-field projection byte identity holds
for {trace_summary['full_projection_byte_identical_count']} of four cells. The
retained warm-start-only divergence is: {warm_start_disclosure}. This is not
full-state trace equivalence.

Four additional paper_cpu/Native traces are retained in a separate directory,
invocation list, and trace-summary lane. Their strict-invalid scenarios are:
{strict_failure_text}. They are not used to validate the rendered trajectory,
cannot replace a current_visual trace, and do not make this reconstruction
paper-comparable. The current visual lane is solver-valid={str(current['all_solver_contract_valid']).lower()};
the separate strict lane is solver-valid={str(strict['all_solver_contract_valid']).lower()}.

Claim boundary: geometry, mass/density, friction cells, drop height, settle,
smoothstep ramp, timestep, and six-second horizon are pinned to the public author
source. DART represents the author's 5 mm collision gap as initial geometric
separation; backend margin semantics are not equivalent. The rendered demo and
CSV exporter are separate DART implementations. No Warp/Newton state-trace
equivalence, historical camera/material/lighting golden, synchronized external-
solver row, historical Apple timing, or real-time verdict is supplied.
"""


def _clean_generated_outputs(bundle: Path) -> None:
    for relative in GENERATED_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()


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
                raise ValueError("turntable bundle transaction was not committed")
        except BaseException:
            if not committed:
                _remove_bundle_root(bundle)
                shutil.copytree(snapshot, bundle)
            raise


def _parse_json_stdout(stdout: str, *, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(stdout)
    except json.JSONDecodeError as error:
        raise ValueError(f"{label} did not emit one JSON document") from error
    if not isinstance(payload, dict):
        raise ValueError(f"{label} did not emit a JSON object")
    return payload


def _finalize_transaction(
    args: argparse.Namespace,
    *,
    bundle: Path,
    commit: Callable[[], None],
) -> dict[str, Any]:
    _clean_generated_outputs(bundle)
    _validate_bundle_paths(
        bundle,
        complete=False,
        require_capture=args.reuse_current_capture,
        allow_staging=True,
    )

    trace_binary = _require_file(args.trace_binary, "trace binary")
    demo = _require_file(args.demo, "DART demo binary")
    runner = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")
    python = _require_file(args.python, "Python interpreter")
    source_contract = _validate_capture_source_contract(
        runner, demo=demo, trace_binary=trace_binary, python=python
    )

    if args.reuse_current_capture:
        capture_provenance = read_json(bundle / "capture-provenance.json")
    else:
        runtime_before = _capture_runtime_identity(
            demo=demo, ffmpeg=ffmpeg, ffprobe=ffprobe, python=python
        )
        sources_before = _capture_source_identity(runner=runner)
        capture_argv = _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe)
        capture_process = _run_command(capture_argv, timeout=args.capture_timeout)
        runtime_after = _capture_runtime_identity(
            demo=demo, ffmpeg=ffmpeg, ffprobe=ffprobe, python=python
        )
        sources_after = _capture_source_identity(runner=runner)
        if runtime_after != runtime_before:
            raise ValueError("turntable runtime resources changed during capture")
        if sources_after != sources_before:
            raise ValueError("turntable capture sources changed during capture")
        if (
            _validate_capture_source_contract(
                runner, demo=demo, trace_binary=trace_binary, python=python
            )
            != source_contract
        ):
            raise ValueError("turntable source contract changed during capture")

        capture_stdout_path = bundle / "capture.stdout.txt"
        capture_stderr_path = bundle / "capture.stderr.txt"
        capture_stdout_path.write_text(capture_process.stdout, encoding="utf-8")
        capture_stderr_path.write_text(capture_process.stderr, encoding="utf-8")
        run_summary_path = bundle / "run-summary.json"
        run_summary = read_json(run_summary_path)
        _validate_capture_run_shape(run_summary)
        expected_summary_stdout = (
            json.dumps(run_summary, indent=2, sort_keys=True) + "\n"
        )
        if not capture_process.stdout.endswith(expected_summary_stdout):
            raise ValueError("turntable capture stdout does not end with run-summary")
        outcome_stills = _promote_outcome_stills(bundle)
        capture_provenance = {
            "argv": capture_argv,
            "cwd": str(ROOT),
            "returncode": capture_process.returncode,
            "run_summary_path": "run-summary.json",
            "run_summary_sha256": sha256(run_summary_path),
            "run_summary_validated": True,
            "capture_stdout_sha256": sha256(capture_stdout_path),
            "capture_stdout_ends_with_run_summary": True,
            "capture_stderr_sha256": sha256(capture_stderr_path),
            "verification_stderr_sha256": None,
            "outcome_stills": outcome_stills,
            "staging_pruned": False,
            "runtime_resources_before": runtime_before,
            "runtime_resources_after": runtime_after,
            "capture_sources_before": sources_before,
            "capture_sources_after": sources_after,
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
    _validate_bundle_paths(
        bundle,
        complete=False,
        require_capture=True,
        allow_missing_manual=True,
        allow_staging=True,
    )
    if not (bundle / "manual-inspection.json").is_file():
        commit()
        raise ValueError(
            "fresh turntable capture is complete; manually inspect the bound "
            "artifacts, add manual-inspection.json, then rerun with "
            "--reuse-current-capture"
        )

    trace_runtime_before = _trace_runtime_identity(trace_binary=trace_binary)
    trace_sources_before = _trace_source_identity()
    parsed: dict[str, dict[str, dict[str, Any]]] = {lane: {} for lane in LANES}
    trace_invocations = []
    for lane in LANES:
        for scenario in SCENARIOS:
            argv = _trace_argv(trace_binary, scenario, lane)
            process = _run_trace_command(argv, lane=lane, timeout=args.trace_timeout)
            stdout_relative = _trace_relative_path(lane, scenario, ".csv")
            stderr_relative = _trace_relative_path(lane, scenario, ".stderr.txt")
            stdout_path = bundle / stdout_relative
            stderr_path = bundle / stderr_relative
            stdout_path.parent.mkdir(parents=True, exist_ok=True)
            stdout_path.write_text(process.stdout, encoding="utf-8")
            stderr_path.write_text(process.stderr, encoding="utf-8")
            parsed[lane][scenario] = parse_trace_text(
                process.stdout, scenario, lane=lane
            )
            expected_returncode = _expected_trace_returncode(
                parsed[lane][scenario]["summary"]
            )
            if process.returncode != expected_returncode:
                raise ValueError(
                    f"{lane}/{scenario}: trace exit {process.returncode} does not "
                    f"match parsed solver result {expected_returncode}"
                )
            trace_invocations.append(
                {
                    "lane": lane,
                    "scenario": scenario,
                    "argv": argv,
                    "returncode": process.returncode,
                    "stdout_path": stdout_relative,
                    "stdout_sha256": sha256(stdout_path),
                    "stderr_path": stderr_relative,
                    "stderr_sha256": sha256(stderr_path),
                }
            )
    trace_runtime_after = _trace_runtime_identity(trace_binary=trace_binary)
    trace_sources_after = _trace_source_identity()
    if trace_runtime_after != trace_runtime_before:
        raise ValueError("turntable trace binary changed during trace matrix")
    if trace_sources_after != trace_sources_before:
        raise ValueError("turntable trace sources changed during trace matrix")
    trace_matrix_provenance = {
        "cwd": str(ROOT),
        "runtime_resources_before": trace_runtime_before,
        "runtime_resources_after": trace_runtime_after,
        "trace_sources_before": trace_sources_before,
        "trace_sources_after": trace_sources_after,
    }
    _validate_trace_matrix_provenance(
        trace_matrix_provenance, trace_binary=trace_binary
    )

    verify_argv = _verification_argv(python, runner, demo, bundle, ffmpeg, ffprobe)
    verify_process = _run_command(verify_argv, timeout=args.verification_timeout)
    verification = _parse_json_stdout(verify_process.stdout, label="visual verifier")
    write_json(bundle / "verification.json", verification)
    (bundle / "verification.stderr.txt").write_text(
        verify_process.stderr, encoding="utf-8"
    )
    capture_summary = validate_capture_bundle(
        bundle,
        demo=demo,
        python=python,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        verification=verification,
        require_staging=True,
    )
    trace_summary = _build_trace_summary(parsed, capture_summary)
    write_json(bundle / "trace-summary.json", trace_summary)
    manual = validate_manual_inspection(bundle)
    capture_provenance["verification_stderr_sha256"] = sha256(
        bundle / "verification.stderr.txt"
    )
    capture_provenance["staging_pruned"] = True
    write_json(bundle / "capture-provenance.json", capture_provenance)

    invocations = {
        "schema_version": INVOCATIONS_SCHEMA_VERSION,
        "trace_matrix_provenance": trace_matrix_provenance,
        "traces": trace_invocations,
        "capture_verification": {
            "argv": verify_argv,
            "returncode": verify_process.returncode,
            "stdout_path": "verification.json",
            "stdout_sha256": sha256(bundle / "verification.json"),
            "stderr_sha256": capture_provenance["verification_stderr_sha256"],
            "stderr_retained": False,
        },
    }
    write_json(bundle / "invocations.json", invocations)
    report = _report_markdown(trace_summary, capture_summary)
    (bundle / "REPORT.md").write_text(report, encoding="utf-8")

    _prune_capture_staging(bundle)
    _validate_capture_provenance(
        capture_provenance,
        bundle=bundle,
        python=python,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    sealed_capture_summary = validate_capture_bundle(
        bundle,
        demo=demo,
        python=python,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        verification=verification,
    )
    if sealed_capture_summary != capture_summary:
        raise ValueError("turntable durable capture summary changed after pruning")
    _validate_bundle_paths(bundle, complete=False, require_capture=True)

    files_before_index = {
        path.relative_to(bundle).as_posix()
        for path in bundle.rglob("*")
        if path.is_file() and not path.is_symlink()
    }
    expected_before_index = EXPECTED_FINAL_PATHS - INDEX_EXCLUSIONS
    if files_before_index != expected_before_index:
        raise ValueError(
            "turntable bundle is not ready for indexing: "
            f"missing={sorted(expected_before_index - files_before_index)}, "
            f"extra={sorted(files_before_index - expected_before_index)}"
        )
    index = artifact_index(bundle)
    write_json(bundle / "artifact-index.json", index)

    capture_metadata = dict(capture_summary)
    capture_metadata.pop("capture_solver_projections")
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "status": "valid_author_source_pinned_nonpaper_turntable_matrix",
        "pass": True,
        "evidence_date": args.evidence_date,
        "requirement_ids": ["fig.04", "video.04_turntable"],
        "artifact_valid": True,
        "solver_contract_valid": True,
        "physical_outcome_valid": True,
        "render_binding_lane": "current_visual",
        "separate_diagnostic_lane": "paper_cpu_native",
        "cross_lane_substitution_allowed": False,
        "paper_cpu_native_all_solver_contract_valid": trace_summary["lanes"][
            "paper_cpu_native"
        ]["all_solver_contract_valid"],
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
        "solver_projection_equivalent": trace_summary[
            "all_visual_capture_full_projections_byte_identical"
        ],
        "core_solver_contact_projection_equivalent": True,
        "source_order_validated": True,
        "author_source_pinned": True,
        "author_source_commit": AUTHOR_COMMIT,
        "author_configuration_port": True,
        "claim_scope": (
            "DART port of the public author turntable configuration: manual "
            "2x2 visual classification independently corroborated by four "
            "dart_best/Native-collision traces."
        ),
        "trace_summary": trace_summary,
        "capture_summary": capture_metadata,
        "capture_provenance": capture_provenance,
        "capture_source_contract": source_contract,
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
            "The paper_cpu/Native lane remains separate diagnostic evidence. "
            "The public author configuration is ported, but DART gap/collision "
            "semantics and state traces are not Warp/Newton equivalent; the "
            "render/trace pairing is not full-state equivalence; no historical "
            "approved golden, external-solver, timing, or real-time claim is made."
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


def finalize(args: argparse.Namespace) -> dict[str, Any]:
    bundle = _require_bundle_root(args.bundle, create=True)
    _validate_bundle_paths(
        bundle,
        complete=False,
        require_capture=args.reuse_current_capture,
        allow_staging=True,
    )
    if args.reuse_current_capture:
        persisted_provenance = read_json(bundle / "capture-provenance.json")
        if persisted_provenance.get("staging_pruned") is True:
            raise ValueError(
                "sealed turntable evidence cannot be re-finalized without a "
                "fresh capture; use --verify-only"
            )
    with _bundle_transaction(bundle) as commit:
        return _finalize_transaction(
            args,
            bundle=bundle,
            commit=commit,
        )


def _read_parsed_traces(
    bundle: Path,
) -> dict[str, dict[str, dict[str, Any]]]:
    parsed: dict[str, dict[str, dict[str, Any]]] = {lane: {} for lane in LANES}
    for lane in LANES:
        for scenario in SCENARIOS:
            path = bundle / _trace_relative_path(lane, scenario, ".csv")
            parsed[lane][scenario] = parse_trace_text(
                path.read_text(encoding="utf-8"), scenario, lane=lane
            )
    return parsed


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
    _validate_bundle_paths(bundle, complete=True, require_capture=True)
    trace_binary = _require_file(trace_binary, "trace binary")
    demo = _require_file(demo, "DART demo binary")
    runner = _require_file(runner, "visual evidence runner")
    ffmpeg = _require_file(ffmpeg, "ffmpeg")
    ffprobe = _require_file(ffprobe, "ffprobe")
    python = _require_file(python, "Python interpreter")

    metadata = read_json(bundle / "metadata.json")
    strict_valid = metadata.get("paper_cpu_native_all_solver_contract_valid")
    expected_flags = {
        "schema_version": SCHEMA_VERSION,
        "status": "valid_author_source_pinned_nonpaper_turntable_matrix",
        "pass": True,
        "requirement_ids": ["fig.04", "video.04_turntable"],
        "artifact_valid": True,
        "solver_contract_valid": True,
        "physical_outcome_valid": True,
        "render_binding_lane": "current_visual",
        "separate_diagnostic_lane": "paper_cpu_native",
        "cross_lane_substitution_allowed": False,
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
        "core_solver_contact_projection_equivalent": True,
        "source_order_validated": True,
        "author_source_pinned": True,
        "author_source_commit": AUTHOR_COMMIT,
        "author_configuration_port": True,
    }
    if any(metadata.get(key) != value for key, value in expected_flags.items()):
        raise ValueError("turntable finalized metadata claim flags changed")
    if not isinstance(strict_valid, bool):
        raise ValueError("turntable strict diagnostic status is missing")
    if (
        not isinstance(metadata.get("evidence_date"), str)
        or not metadata["evidence_date"]
    ):
        raise ValueError("turntable evidence date is missing")

    parsed = _read_parsed_traces(bundle)
    capture_summary = validate_capture_bundle(
        bundle,
        demo=demo,
        python=python,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    trace_summary = _build_trace_summary(parsed, capture_summary)
    if read_json(bundle / "trace-summary.json") != trace_summary:
        raise ValueError("turntable trace-summary.json is stale")
    if metadata.get("trace_summary") != trace_summary:
        raise ValueError("turntable metadata trace summary is stale")
    if (
        metadata.get("solver_projection_equivalent")
        != trace_summary["all_visual_capture_full_projections_byte_identical"]
    ):
        raise ValueError("turntable full projection identity flag is stale")
    if (
        strict_valid
        != trace_summary["lanes"]["paper_cpu_native"]["all_solver_contract_valid"]
    ):
        raise ValueError("turntable strict diagnostic status is stale")
    capture_metadata = dict(capture_summary)
    capture_metadata.pop("capture_solver_projections")
    if metadata.get("capture_summary") != capture_metadata:
        raise ValueError("turntable metadata capture summary is stale")

    capture_provenance = read_json(bundle / "capture-provenance.json")
    if metadata.get("capture_provenance") != capture_provenance:
        raise ValueError("turntable capture-provenance file binding is stale")
    _validate_capture_provenance(
        capture_provenance,
        bundle=bundle,
        python=python,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    source_contract = _validate_capture_source_contract(
        runner, demo=demo, trace_binary=trace_binary, python=python
    )
    if metadata.get("capture_source_contract") != source_contract:
        raise ValueError("turntable capture source contract is stale")

    manual = validate_manual_inspection(bundle)
    if metadata.get("manual_inspection") != {
        "path": "manual-inspection.json",
        "sha256": sha256(bundle / "manual-inspection.json"),
        "pass": manual["pass"],
    }:
        raise ValueError("turntable metadata manual-inspection binding is stale")
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
        if metadata.get(key) != {
            "path": relative,
            "sha256": sha256(bundle / relative),
        }:
            raise ValueError(f"turntable metadata {key} binding is stale")
    expected_report = _report_markdown(trace_summary, capture_summary)
    if (bundle / "REPORT.md").read_text(encoding="utf-8") != expected_report:
        raise ValueError("turntable REPORT.md is stale")

    index = read_json(bundle / "artifact-index.json")
    validate_artifact_index(bundle, index)
    if metadata.get("artifact_index") != {
        "path": "artifact-index.json",
        "sha256": sha256(bundle / "artifact-index.json"),
        "artifact_count": index["artifact_count"],
        "excluded": index["excluded"],
    }:
        raise ValueError("turntable metadata artifact-index binding is stale")
    sources = _source_identity(
        trace_binary=trace_binary,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    if metadata.get("source_identity") != sources:
        raise ValueError("turntable source/binary identity changed")
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
        "current_visual_projection_sha256": {
            item["scenario"]: {
                "trace": item["trace_sha256"],
                "capture": item["capture_sha256"],
                "byte_identical": item["byte_identical"],
                "core_projection_equivalent": item["core_projection_equivalent"],
            }
            for item in trace_summary["visual_capture_projection_comparisons"]
        },
        "paper_cpu_native_all_solver_contract_valid": strict_valid,
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
    persisted_trace_provenance = read_json(bundle / "invocations.json").get(
        "trace_matrix_provenance"
    )
    _validate_trace_matrix_provenance(
        persisted_trace_provenance, trace_binary=trace_binary
    )
    trace_runtime_before = _trace_runtime_identity(trace_binary=trace_binary)
    trace_sources_before = _trace_source_identity()
    for lane in LANES:
        for scenario in SCENARIOS:
            process = _run_trace_command(
                _trace_argv(trace_binary, scenario, lane),
                lane=lane,
                timeout=args.trace_timeout,
            )
            stdout_path = bundle / _trace_relative_path(lane, scenario, ".csv")
            stderr_path = bundle / _trace_relative_path(lane, scenario, ".stderr.txt")
            if process.stdout != stdout_path.read_text(encoding="utf-8"):
                raise ValueError(f"live {lane}/{scenario} trace replay changed")
            if process.stderr != stderr_path.read_text(encoding="utf-8"):
                raise ValueError(f"live {lane}/{scenario} trace stderr changed")
            parsed = parse_trace_text(process.stdout, scenario, lane=lane)
            expected_returncode = _expected_trace_returncode(parsed["summary"])
            if process.returncode != expected_returncode:
                raise ValueError(f"live {lane}/{scenario} trace exit changed")
    trace_runtime_after = _trace_runtime_identity(trace_binary=trace_binary)
    trace_sources_after = _trace_source_identity()
    if trace_runtime_after != trace_runtime_before:
        raise ValueError("turntable trace binary changed during live replay")
    if trace_sources_after != trace_sources_before:
        raise ValueError("turntable trace sources changed during live replay")

    return {
        **result,
        "live_current_visual_trace_replay": True,
        "live_paper_cpu_native_trace_replay": True,
        "live_trace_provenance_stable": True,
        "durable_capture_record_reverification": True,
        "ignored_staging_required": False,
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
    parser.add_argument("--capture-timeout", type=float, default=1800.0)
    parser.add_argument("--trace-timeout", type=float, default=120.0)
    parser.add_argument("--verification-timeout", type=float, default=900.0)
    parser.add_argument("--evidence-date", default="2026-07-18")
    parser.add_argument(
        "--reuse-current-capture",
        action="store_true",
        help=(
            "finalize an existing capture only after validating persisted "
            "capture-time runtime/source provenance"
        ),
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help=(
            "recompute sealed durable contracts and replay all eight traces "
            "without requiring ignored capture staging"
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
