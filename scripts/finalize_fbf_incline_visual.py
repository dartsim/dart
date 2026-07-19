#!/usr/bin/env python3
"""Finalize and revalidate the current-source DART incline visual bundle.

The bundle deliberately supports a narrow claim.  It binds one runner-native
two-cell DART capture to two fresh, independent exact-FBF tracked traces, a
recorded manual inspection, and the exact source/runtime identities used to
produce it.  The only capture/trace comparison is an additive projection of
contact, exact-solve, and fallback counts.  The rendered and traced scenes
have different placements, so this driver never claims state or full-trace
equivalence, paper parity, external-solver parity, or real-time performance.
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
import shlex
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Iterator, Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "fig01_02_incline_current_v1"
)
DEFAULT_TRACE_BINARY = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
)
DEFAULT_DEMO = ROOT / "build/default/cpp/Release/bin/dart-demos"
DEFAULT_RUNNER = ROOT / "scripts/run_fbf_visual_evidence.py"
DEFAULT_FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
DEFAULT_FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"
DEFAULT_PYTHON = Path(sys.executable)
BUILD_RUNTIME_ROOT = ROOT / "build/default/cpp/Release"

TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
FIXTURE_SOURCE = ROOT / "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp"
DEMO_SOURCE = ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
DEMO_HOST_SOURCE = ROOT / "examples/demos/DemoHost.cpp"
DEMO_HOST_HEADER = ROOT / "examples/demos/DemoHost.hpp"
REGISTRY_SOURCE = ROOT / "examples/demos/Registry.cpp"
SCENES_HEADER = ROOT / "examples/demos/scenes/Scenes.hpp"
RUNNER_TEST = ROOT / "python/tests/unit/test_run_fbf_visual_evidence.py"
FINALIZER_TEST = ROOT / "python/tests/unit/test_finalize_fbf_incline_visual.py"
IMAGE_COMPOSE = ROOT / "scripts/image_compose.py"
IMAGE_TOOLS = ROOT / "scripts/_image_tools.py"
IMAGE_VERDICT = ROOT / "scripts/image_verdict.py"

SCHEMA_VERSION = "dart.fbf_incline_visual_bundle/v1"
INDEX_SCHEMA_VERSION = "dart.fbf_incline_artifact_index/v1"
MANUAL_SCHEMA_VERSION = "dart.fbf_incline_manual_inspection/v1"
TRACE_SUMMARY_SCHEMA_VERSION = "dart.fbf_incline_trace_summary/v1"
INVOCATIONS_SCHEMA_VERSION = "dart.fbf_incline_invocations/v1"
RUNNER_SCHEMA_VERSION = "dart.fbf_visual_evidence/v1"
STAGING_SCHEMA_VERSION = "dart.fbf_incline_pruned_staging/v1"

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
SCENARIOS = ("incline_mu_0_4", "incline_mu_0_5")
SCENARIO_FRICTION = {"incline_mu_0_4": 0.4, "incline_mu_0_5": 0.5}
TOTAL_STEPS = 120
DT_SECONDS = 1.0 / 60.0
EXPECTED_CAPTURE_EXACT_GROUPS = 2 * TOTAL_STEPS
CAPTURE_STEPS = tuple(range(0, TOTAL_STEPS + 1, 2))
PANEL_STEPS = (0, 30, 60, 90, 120)
PANEL_LABELS = (
    "t=0.00s",
    "t=0.50s",
    "t=1.00s",
    "t=1.50s",
    "t=2.00s",
)
EXPECTED_CONFIGURATION = {"mu_cells": "0.4,0.5", "tan_theta": "0.5"}
EXPECTED_MISMATCHES = [
    (
        "DART renders its own float64 scene reconstructions under DART collision/"
        "scene contracts, not outputs from the authors' public Warp/Newton "
        "reference implementation or historical paper rendering setup."
    ),
    (
        "Only the DART exact-FBF row is captured; faithful synchronized external-"
        "solver renderings and approved source goldens remain separate evidence."
    ),
    (
        "Both DART cells share one view, but the current collision frontend reports "
        "three contacts per cell versus four in the paper timing row."
    ),
    (
        "The paper comparison edit and external panels last 11 seconds; this "
        "artifact is the underlying two-second DART trajectory."
    ),
]
SEMANTIC_OUTCOME_GATE = (
    "manual inspection plus the separate physical trace/test contract is required; "
    "image motion/nonblank checks do not prove the expected outcome"
)

INCLINE_TAN = 0.5
INCLINE_INITIAL_NORMAL_OFFSET = 0.5 - 0.01
INCLINE_STICK_DISPLACEMENT_TOLERANCE = 2.0e-2
INCLINE_SLIDE_FRICTION = 0.4
INCLINE_SLIDE_DISPLACEMENT_TOLERANCE = 0.2
INCLINE_CROSS_SLOPE_DISPLACEMENT_TOLERANCE = 2.0e-2
INCLINE_NORMAL_CENTER_OFFSET_TOLERANCE = 2.0e-2
INCLINE_UP_Z_ALIGNMENT_TOLERANCE = 5.0e-2
INCLINE_SLIDE_REGRESSION_TOLERANCE = 1.0e-4
INCLINE_VELOCITY_POSITION_CONSISTENCY_TOLERANCE = 1.0e-10
INCLINE_CROSS_SLOPE_SPEED_TOLERANCE = 2.0e-2
INCLINE_NORMAL_SPEED_TOLERANCE = 2.0e-2
INCLINE_STICK_SPEED_TOLERANCE = 2.0e-2
INCLINE_SLIDE_FINAL_DOWNHILL_SPEED_MINIMUM = 0.5
PAPER_REFERENCE_CONTACTS = 4
CURRENT_DART_CONTACTS = 3
CURRENT_CAPTURE_TOTAL_CONTACTS = 8
GRAVITY = 9.81
RESIDUAL_TOLERANCE = 1.0e-6
PROCESS_TERMINATION_GRACE_SECONDS = 1.0

COUNT_PROJECTION_FIELDS = (
    "step",
    "exact_solves",
    "boxed_lcp_fallbacks",
)
INDEX_EXCLUSIONS = {"artifact-index.json", "metadata.json"}

MANUAL_VERDICTS = {
    "two_cell_threshold_comparison_visible": True,
    "mu_0_4_downhill_slide_visible": True,
    "mu_0_5_near_threshold_stick_visible": True,
    "panel_time_labels_legible": True,
    "physical_outcome_claim_requires_independent_traces": True,
    "aggregate_count_projection_only": True,
    "full_state_trace_equivalence": False,
    "maximum_penetration_proven": False,
    "paper_reference_contact_count_match": False,
    "paper_parity": False,
    "external_solver_parity": False,
    "approved_source_golden": False,
    "approved_source_golden_diff": False,
    "capture_sidecar_deliverable_validated": False,
    "full_friction_sweep": False,
    "timing_verdict": None,
    "realtime_verdict": None,
}
DURABLE_STILL_STEPS = PANEL_STEPS
DURABLE_STILL_PATHS = {
    f"incline/stills/step_{step:06d}.png" for step in DURABLE_STILL_STEPS
}
MANUAL_ARTIFACT_PATHS = {
    "incline/panel.png",
    "incline/clip.mp4",
    *DURABLE_STILL_PATHS,
}


def _capture_paths() -> set[str]:
    paths = {
        "run-summary.json",
        "capture-provenance.json",
        "capture.stdout.txt",
        "capture.stderr.txt",
        "manual-inspection.json",
        "incline/metadata.json",
        "incline/timeline.json",
        "incline/panel.png",
        "incline/panel.compose.json",
        "incline/clip.mp4",
        "incline/video_frames.ffconcat",
    }
    paths.update(f"incline/frames/step_{step:06d}.png" for step in CAPTURE_STEPS)
    paths.update(f"incline/panel_frames/step_{step:06d}.png" for step in PANEL_STEPS)
    return paths


CAPTURE_PATHS = _capture_paths()
GENERATED_PATHS = {
    *(f"traces/{scenario}.csv" for scenario in SCENARIOS),
    *(f"traces/{scenario}.stderr.txt" for scenario in SCENARIOS),
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
    "incline/video_frames.ffconcat",
    *(f"incline/frames/step_{step:06d}.png" for step in CAPTURE_STEPS),
    *(f"incline/panel_frames/step_{step:06d}.png" for step in PANEL_STEPS),
}
DURABLE_CAPTURE_PATHS = CAPTURE_PATHS - STAGING_PATHS
DURABLE_GENERATED_PATHS = GENERATED_PATHS - STAGING_PATHS
EXPECTED_FINAL_PATHS = (
    DURABLE_CAPTURE_PATHS | DURABLE_STILL_PATHS | DURABLE_GENERATED_PATHS
)
WORKING_PATHS = CAPTURE_PATHS | DURABLE_STILL_PATHS | GENERATED_PATHS
WORKING_DIRECTORIES = {
    "incline",
    "incline/frames",
    "incline/panel_frames",
    "incline/stills",
    "traces",
}
FINAL_DIRECTORIES = {"incline", "incline/stills", "traces"}


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
            "incline bundle membership has unexpected entries: "
            f"files={sorted(unexpected_files)}, "
            f"directories={sorted(unexpected_directories)}"
        )
    if complete:
        missing = EXPECTED_FINAL_PATHS - files
        if missing:
            raise ValueError(
                f"incline finalized bundle is incomplete: {sorted(missing)}"
            )
        missing_directories = FINAL_DIRECTORIES - directories
        if missing_directories:
            raise ValueError(
                "incline finalized bundle directories are incomplete: "
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
                f"incline capture bundle is incomplete: {sorted(missing_capture)}"
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


def _incline_geometry() -> dict[str, float]:
    theta = math.atan(INCLINE_TAN)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    return {
        "theta": theta,
        "sin_theta": sin_theta,
        "cos_theta": cos_theta,
        "initial_x": -sin_theta * INCLINE_INITIAL_NORMAL_OFFSET,
        "initial_z": cos_theta * INCLINE_INITIAL_NORMAL_OFFSET,
        "downhill_x": -cos_theta,
        "downhill_z": -sin_theta,
    }


def _analytical_slide_displacement() -> float:
    geometry = _incline_geometry()
    acceleration = GRAVITY * (
        geometry["sin_theta"] - INCLINE_SLIDE_FRICTION * geometry["cos_theta"]
    )
    duration = TOTAL_STEPS * DT_SECONDS
    return 0.5 * acceleration * duration * duration


def parse_trace_text(text: str, expected_scenario: str) -> dict[str, Any]:
    if expected_scenario not in SCENARIOS:
        raise ValueError(f"unsupported incline trace scenario: {expected_scenario}")
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
    cumulative_fields = ("exact_solves", "warm_starts", "fallbacks")
    previous_cumulative = {field: 0 for field in cumulative_fields}
    max_residual = 0.0
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
                f"trace line {line}: scenario {raw['scenario']!r} "
                f"!= {expected_scenario!r}"
            )
        if raw["solver"] != "exact_fbf" or raw["body"] != "incline_cube_body":
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
        if typed["warm_starts"] > typed["exact_solves"]:
            raise ValueError(
                f"trace line {line}: warm starts exceed cumulative exact solves"
            )

        if step == 0:
            residual = _float_field(raw, "residual", line=line, allow_nan=True)
            if not math.isnan(residual) or typed["status"] != "not_run":
                raise ValueError(f"{expected_scenario}: step 0 must be nan/not_run")
            if typed["contacts"] != 0 or any(
                typed[field] != 0 for field in cumulative_fields
            ):
                raise ValueError(
                    f"{expected_scenario}: step 0 contacts/counters must be zero"
                )
        else:
            residual = _float_field(raw, "residual", line=line)
            if typed["status"] != "success":
                raise ValueError(
                    f"{expected_scenario}: step {step} status {typed['status']!r}"
                )
            if residual < 0.0 or residual > RESIDUAL_TOLERANCE:
                raise ValueError(
                    f"{expected_scenario}: step {step} residual {residual} exceeds "
                    f"the [0, {RESIDUAL_TOLERANCE}] contract"
                )
            if typed["contacts"] != CURRENT_DART_CONTACTS:
                raise ValueError(
                    f"{expected_scenario}: step {step} contacts "
                    f"{typed['contacts']} != {CURRENT_DART_CONTACTS}"
                )
            if typed["exact_solves"] != step:
                raise ValueError(
                    f"{expected_scenario}: step {step} cumulative exact solves "
                    f"{typed['exact_solves']} != {step}"
                )
            max_residual = max(max_residual, residual)
        typed["residual"] = residual
        rows.append(typed)

    if any(row["fallbacks"] != 0 for row in rows):
        raise ValueError(f"{expected_scenario}: boxed-LCP fallback observed")

    geometry = _incline_geometry()
    initial = rows[0]
    final = rows[-1]
    for field, expected in (
        ("x", geometry["initial_x"]),
        ("y", 0.0),
        ("z", geometry["initial_z"]),
        ("vx", 0.0),
        ("vy", 0.0),
        ("vz", 0.0),
        ("up_z", geometry["cos_theta"]),
    ):
        if not math.isclose(initial[field], expected, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(
                f"{expected_scenario}: initial {field} {initial[field]} != {expected}"
            )

    displacement = (final["x"] - initial["x"]) * geometry["downhill_x"] + (
        final["z"] - initial["z"]
    ) * geometry["downhill_z"]
    trajectory_displacements = [
        (row["x"] - initial["x"]) * geometry["downhill_x"]
        + (row["z"] - initial["z"]) * geometry["downhill_z"]
        for row in rows
    ]
    maximum_cross_slope_displacement = max(abs(row["y"] - initial["y"]) for row in rows)
    maximum_up_z_alignment_error = max(
        abs(row["up_z"] - geometry["cos_theta"]) for row in rows
    )
    normal_x = -geometry["sin_theta"]
    normal_z = geometry["cos_theta"]
    initial_normal_center_offset = initial["x"] * normal_x + initial["z"] * normal_z
    maximum_normal_center_offset_error = max(
        abs(row["x"] * normal_x + row["z"] * normal_z - initial_normal_center_offset)
        for row in rows
    )
    if maximum_cross_slope_displacement > INCLINE_CROSS_SLOPE_DISPLACEMENT_TOLERANCE:
        raise ValueError(
            f"{expected_scenario}: cross-slope displacement "
            f"{maximum_cross_slope_displacement} exceeds "
            f"{INCLINE_CROSS_SLOPE_DISPLACEMENT_TOLERANCE}"
        )
    if maximum_up_z_alignment_error > INCLINE_UP_Z_ALIGNMENT_TOLERANCE:
        raise ValueError(
            f"{expected_scenario}: up_z alignment error "
            f"{maximum_up_z_alignment_error} exceeds "
            f"{INCLINE_UP_Z_ALIGNMENT_TOLERANCE}"
        )
    if maximum_normal_center_offset_error > INCLINE_NORMAL_CENTER_OFFSET_TOLERANCE:
        raise ValueError(
            f"{expected_scenario}: incline-normal center-offset error "
            f"{maximum_normal_center_offset_error} exceeds "
            f"{INCLINE_NORMAL_CENTER_OFFSET_TOLERANCE}"
        )
    if expected_scenario == "incline_mu_0_5":
        expected_displacement = 0.0
        tolerance = INCLINE_STICK_DISPLACEMENT_TOLERANCE
        maximum_abs_displacement = max(abs(value) for value in trajectory_displacements)
        if maximum_abs_displacement > tolerance:
            raise ValueError(
                f"{expected_scenario}: stick trajectory displacement "
                f"{maximum_abs_displacement} exceeds {tolerance}"
            )
    else:
        expected_displacement = _analytical_slide_displacement()
        tolerance = INCLINE_SLIDE_DISPLACEMENT_TOLERANCE
        maximum_abs_displacement = None
        for step, (previous, current) in enumerate(
            zip(trajectory_displacements, trajectory_displacements[1:]), start=1
        ):
            if current + INCLINE_SLIDE_REGRESSION_TOLERANCE < previous:
                raise ValueError(
                    f"{expected_scenario}: downhill displacement regressed at "
                    f"step {step}"
                )
    if not math.isfinite(displacement) or abs(displacement - expected_displacement) > (
        tolerance
    ):
        raise ValueError(
            f"{expected_scenario}: downhill displacement {displacement} differs "
            f"from {expected_displacement} by more than {tolerance}"
        )

    velocity_position_component_errors = []
    for previous, current in zip(rows, rows[1:]):
        for position_field, velocity_field in (
            ("x", "vx"),
            ("y", "vy"),
            ("z", "vz"),
        ):
            finite_difference_velocity = (
                current[position_field] - previous[position_field]
            ) / DT_SECONDS
            velocity_position_component_errors.append(
                abs(current[velocity_field] - finite_difference_velocity)
            )
    maximum_velocity_position_component_error = max(velocity_position_component_errors)
    if (
        maximum_velocity_position_component_error
        > INCLINE_VELOCITY_POSITION_CONSISTENCY_TOLERANCE
    ):
        raise ValueError(
            f"{expected_scenario}: velocity-position component consistency error "
            f"{maximum_velocity_position_component_error} exceeds "
            f"{INCLINE_VELOCITY_POSITION_CONSISTENCY_TOLERANCE}"
        )

    downhill_speeds = [
        row["vx"] * geometry["downhill_x"] + row["vz"] * geometry["downhill_z"]
        for row in rows
    ]
    cross_slope_speeds = [abs(row["vy"]) for row in rows]
    incline_normal_speeds = [
        abs(row["vx"] * normal_x + row["vz"] * normal_z) for row in rows
    ]
    total_speeds = [
        math.sqrt(row["vx"] ** 2 + row["vy"] ** 2 + row["vz"] ** 2) for row in rows
    ]
    maximum_cross_slope_speed = max(cross_slope_speeds)
    maximum_incline_normal_speed = max(incline_normal_speeds)
    if maximum_cross_slope_speed > INCLINE_CROSS_SLOPE_SPEED_TOLERANCE:
        raise ValueError(
            f"{expected_scenario}: cross-slope speed {maximum_cross_slope_speed} "
            f"exceeds {INCLINE_CROSS_SLOPE_SPEED_TOLERANCE}"
        )
    if maximum_incline_normal_speed > INCLINE_NORMAL_SPEED_TOLERANCE:
        raise ValueError(
            f"{expected_scenario}: incline-normal speed "
            f"{maximum_incline_normal_speed} exceeds "
            f"{INCLINE_NORMAL_SPEED_TOLERANCE}"
        )

    maximum_stick_speed = None
    final_stick_speed = None
    if expected_scenario == "incline_mu_0_5":
        maximum_stick_speed = max(total_speeds)
        final_stick_speed = total_speeds[-1]
        if maximum_stick_speed > INCLINE_STICK_SPEED_TOLERANCE:
            raise ValueError(
                f"{expected_scenario}: stick speed {maximum_stick_speed} exceeds "
                f"{INCLINE_STICK_SPEED_TOLERANCE}"
            )
        if final_stick_speed > INCLINE_STICK_SPEED_TOLERANCE:
            raise ValueError(
                f"{expected_scenario}: final stick speed {final_stick_speed} "
                f"exceeds {INCLINE_STICK_SPEED_TOLERANCE}"
            )
    elif downhill_speeds[-1] < INCLINE_SLIDE_FINAL_DOWNHILL_SPEED_MINIMUM:
        raise ValueError(
            f"{expected_scenario}: final downhill speed {downhill_speeds[-1]} is "
            f"below {INCLINE_SLIDE_FINAL_DOWNHILL_SPEED_MINIMUM}"
        )

    selected_states = []
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
        selected_states.append(state)

    count_projection = [
        {
            "step": row["step"],
            "contacts": row["contacts"],
            "exact_solves": row["exact_solves"],
            "boxed_lcp_fallbacks": row["fallbacks"],
        }
        for row in rows
    ]
    summary = {
        "scenario": expected_scenario,
        "mu": SCENARIO_FRICTION[expected_scenario],
        "row_count": len(rows),
        "completed_steps": TOTAL_STEPS,
        "exact_solves": final["exact_solves"],
        "warm_starts": final["warm_starts"],
        "boxed_lcp_fallbacks": final["fallbacks"],
        "max_residual": max_residual,
        "initial": {
            key: initial[key] for key in ("step", "time", "x", "y", "z", "up_z")
        },
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
        "downhill_displacement_m": displacement,
        "expected_downhill_displacement_m": expected_displacement,
        "displacement_tolerance_m": tolerance,
        "trajectory_invariants": {
            "maximum_cross_slope_displacement_m": (maximum_cross_slope_displacement),
            "cross_slope_displacement_tolerance_m": (
                INCLINE_CROSS_SLOPE_DISPLACEMENT_TOLERANCE
            ),
            "maximum_incline_normal_center_offset_error_m": (
                maximum_normal_center_offset_error
            ),
            "incline_normal_center_offset_tolerance_m": (
                INCLINE_NORMAL_CENTER_OFFSET_TOLERANCE
            ),
            "maximum_up_z_alignment_error": maximum_up_z_alignment_error,
            "up_z_alignment_tolerance": INCLINE_UP_Z_ALIGNMENT_TOLERANCE,
            "maximum_velocity_position_component_error_m_s": (
                maximum_velocity_position_component_error
            ),
            "velocity_position_component_consistency_tolerance_m_s": (
                INCLINE_VELOCITY_POSITION_CONSISTENCY_TOLERANCE
            ),
            "maximum_cross_slope_speed_m_s": maximum_cross_slope_speed,
            "cross_slope_speed_tolerance_m_s": (INCLINE_CROSS_SLOPE_SPEED_TOLERANCE),
            "maximum_incline_normal_speed_m_s": maximum_incline_normal_speed,
            "incline_normal_speed_tolerance_m_s": INCLINE_NORMAL_SPEED_TOLERANCE,
            "final_downhill_speed_m_s": downhill_speeds[-1],
            "slide_final_downhill_speed_minimum_m_s": (
                INCLINE_SLIDE_FINAL_DOWNHILL_SPEED_MINIMUM
                if expected_scenario == "incline_mu_0_4"
                else None
            ),
            "maximum_stick_speed_m_s": maximum_stick_speed,
            "final_stick_speed_m_s": final_stick_speed,
            "stick_speed_tolerance_m_s": (
                INCLINE_STICK_SPEED_TOLERANCE
                if expected_scenario == "incline_mu_0_5"
                else None
            ),
            "maximum_abs_stick_displacement_m": maximum_abs_displacement,
            "slide_displacement_regression_tolerance_m": (
                INCLINE_SLIDE_REGRESSION_TOLERANCE
                if expected_scenario == "incline_mu_0_4"
                else None
            ),
            "pass": True,
        },
        "selected_panel_states": selected_states,
        "continuous_contact_post_initial": True,
        "observed_contacts_per_step": CURRENT_DART_CONTACTS,
        "paper_reference_contacts": PAPER_REFERENCE_CONTACTS,
        "paper_reference_contact_count_match": False,
        "maximum_penetration_proven": False,
        "physical_outcome_valid": True,
        "claim_scope": (
            "DART two-second downhill displacement threshold and uninterrupted "
            "post-initial tracked-trace contact"
        ),
    }
    return {
        "summary": summary,
        "count_projection": count_projection,
        "count_projection_sha256": _payload_sha256(count_projection),
    }


def summarize_trace_pair(parsed: Sequence[dict[str, Any]]) -> dict[str, Any]:
    if len(parsed) != len(SCENARIOS):
        raise ValueError("incline trace pair is incomplete")
    by_scenario = {item.get("summary", {}).get("scenario"): item for item in parsed}
    if set(by_scenario) != set(SCENARIOS):
        raise ValueError("incline trace pair is missing or duplicated")

    slide_displacement = by_scenario["incline_mu_0_4"]["summary"][
        "downhill_displacement_m"
    ]
    stick_displacement = by_scenario["incline_mu_0_5"]["summary"][
        "downhill_displacement_m"
    ]
    threshold_separation = slide_displacement - stick_displacement
    if not threshold_separation > 0.5:
        raise ValueError(
            "incline slide/stick downhill displacement separation does not exceed "
            "0.5 m"
        )

    aggregate_projection = []
    for step in range(TOTAL_STEPS + 1):
        rows = [by_scenario[name]["count_projection"][step] for name in SCENARIOS]
        aggregate_projection.append(
            {
                "step": step,
                "exact_solves": sum(row["exact_solves"] for row in rows),
                "boxed_lcp_fallbacks": sum(row["boxed_lcp_fallbacks"] for row in rows),
            }
        )
    return {
        "schema_version": TRACE_SUMMARY_SCHEMA_VERSION,
        "pass": True,
        "scenarios": [by_scenario[name]["summary"] for name in SCENARIOS],
        "aggregate_count_projection": aggregate_projection,
        "aggregate_count_projection_sha256": _payload_sha256(aggregate_projection),
        "aggregate_projection_fields": list(COUNT_PROJECTION_FIELDS),
        "trace_aggregate_contacts_per_post_initial_step": (
            len(SCENARIOS) * CURRENT_DART_CONTACTS
        ),
        "physical_outcome_valid": True,
        "threshold_separation": {
            "slide_downhill_displacement_m": slide_displacement,
            "stick_downhill_displacement_m": stick_displacement,
            "slide_minus_stick_m": threshold_separation,
            "minimum_exclusive_m": 0.5,
            "pass": True,
        },
        "full_state_trace_equivalence": False,
        "claim_boundary": (
            "The two tracked traces are independent DART runs. Their additive "
            "exact-solve/fallback counts may be compared with the combined capture. "
            "Contact counts differ and are excluded; state, residual, status, "
            "warm-start, and per-cell equivalence are not claimed."
        ),
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
    if set(payload) != {"schema_version", "artifact_count", "excluded", "artifacts"}:
        raise ValueError("malformed incline artifact-index payload")
    if payload.get("schema_version") != INDEX_SCHEMA_VERSION:
        raise ValueError("unexpected incline artifact-index schema")
    if payload.get("excluded") != sorted(INDEX_EXCLUSIONS):
        raise ValueError("incline artifact-index exclusions changed")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("incline artifact-index artifacts must be a list")
    if payload.get("artifact_count") != len(artifacts):
        raise ValueError("incline artifact-index count mismatch")

    listed = []
    for item in artifacts:
        if not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("malformed incline artifact-index entry")
        relative = item["path"]
        if (
            not isinstance(relative, str)
            or relative.startswith(("/", "../"))
            or "/../" in relative
        ):
            raise ValueError(f"unsafe incline artifact-index path: {relative!r}")
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"incline artifact-index file missing: {relative}")
        if path.stat().st_size != item["bytes"]:
            raise ValueError(f"incline artifact-index byte mismatch: {relative}")
        if sha256(path) != item["sha256"]:
            raise ValueError(f"incline artifact-index hash mismatch: {relative}")
        listed.append(relative)
    if listed != sorted(set(listed)):
        raise ValueError("incline artifact-index paths are duplicate or unsorted")

    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.relative_to(root).as_posix() not in INDEX_EXCLUSIONS
    }
    if actual != set(listed):
        raise ValueError(
            "incline artifact-index membership mismatch: "
            f"missing={sorted(actual - set(listed))}, "
            f"extra={sorted(set(listed) - actual)}"
        )


def validate_manual_inspection(root: Path) -> dict[str, Any]:
    record = read_json(root / "manual-inspection.json")
    if set(record) != {
        "schema_version",
        "manual_inspected",
        "pass",
        "verdicts",
        "representative_artifacts",
    }:
        raise ValueError("malformed incline manual-inspection payload")
    if record.get("schema_version") != MANUAL_SCHEMA_VERSION:
        raise ValueError("unexpected incline manual-inspection schema")
    if record.get("manual_inspected") is not True or record.get("pass") is not True:
        raise ValueError("incline manual inspection did not pass")
    if record.get("verdicts") != MANUAL_VERDICTS:
        raise ValueError("incline manual-inspection verdicts changed")
    artifacts = record.get("representative_artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("incline manual-inspection artifacts must be a list")
    if any(
        not isinstance(item, dict) or set(item) != {"path", "sha256", "observation"}
        for item in artifacts
    ):
        raise ValueError("malformed incline manual-inspection artifact")
    paths = [item["path"] for item in artifacts]
    if set(paths) != MANUAL_ARTIFACT_PATHS or len(paths) != len(set(paths)):
        raise ValueError("incline manual-inspection artifact paths changed")
    for item in artifacts:
        relative = item["path"]
        artifact = root / relative
        if (
            not artifact.is_file()
            or artifact.is_symlink()
            or sha256(artifact) != item["sha256"]
        ):
            raise ValueError(f"incline manual-inspection artifact changed: {relative}")
        observation = item["observation"]
        if not isinstance(observation, str) or not observation.strip():
            raise ValueError(
                f"incline manual-inspection observation missing: {relative}"
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

    root = _require_bundle_root(root, create=False)
    metadata = read_json(root / "incline/metadata.json")
    run_results = _capture_result_by_id(read_json(root / "run-summary.json"))
    if set(run_results) != {"incline"} or run_results["incline"] != metadata:
        raise ValueError("incline durable still run-summary binding changed")
    metadata_timeline = metadata.get("timeline_validation")
    run_timeline = run_results["incline"].get("timeline_validation")
    if not isinstance(metadata_timeline, dict) or not isinstance(run_timeline, dict):
        raise ValueError("incline durable still timeline validation is missing")
    metadata_frames = metadata_timeline.get("frames")
    run_frames = run_timeline.get("frames")
    if not isinstance(metadata_frames, dict) or not isinstance(run_frames, dict):
        raise ValueError("incline durable still timeline frames are missing")

    sidecar = read_json(root / "incline/timeline.json")
    shots = sidecar.get("shots")
    if not isinstance(shots, list):
        raise ValueError("incline durable still timeline shots are missing")
    shots_by_step: dict[int, dict[str, Any]] = {}
    for shot in shots:
        if (
            not isinstance(shot, dict)
            or not isinstance(shot.get("step"), int)
            or isinstance(shot.get("step"), bool)
        ):
            raise ValueError("incline durable still timeline shot is malformed")
        step = shot["step"]
        if step in shots_by_step:
            raise ValueError("incline durable still timeline steps are duplicated")
        shots_by_step[step] = shot

    bindings = []
    for step in DURABLE_STILL_STEPS:
        source_relative = f"incline/frames/step_{step:06d}.png"
        source_path = root / source_relative
        still_relative = f"incline/stills/step_{step:06d}.png"
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
                f"incline durable still capture binding changed at step {step}"
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
    root = _require_bundle_root(root, create=False)
    records = []
    for binding in _capture_still_bindings(root):
        source = root / binding["source_frame_path"]
        destination = root / binding["path"]
        if not source.is_file() or source.is_symlink():
            raise ValueError(
                f"incline durable still source is missing: {binding['source_frame_path']}"
            )
        if sha256(source) != binding["timeline_frame_sha256"]:
            raise ValueError(
                "incline durable still source hash changed: "
                f"{binding['source_frame_path']}"
            )
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.is_symlink():
            raise ValueError(f"incline durable still is a symlink: {binding['path']}")
        shutil.copyfile(source, destination)
        records.append({**binding, "sha256": sha256(destination)})
    return records


def _validate_durable_stills(root: Path, records: Any) -> list[dict[str, Any]]:
    root = _require_bundle_root(root, create=False)
    expected = []
    for binding in _capture_still_bindings(root):
        still = root / binding["path"]
        if not still.is_file() or still.is_symlink():
            raise ValueError(f"incline durable still is missing: {binding['path']}")
        still_sha = sha256(still)
        if (
            still_sha != binding["timeline_frame_sha256"]
            or still_sha != binding["run_summary_frame_sha256"]
        ):
            raise ValueError(
                f"incline durable still hash changed at step {binding['step']}"
            )
        expected.append({**binding, "sha256": still_sha})
    if records != expected:
        raise ValueError("incline durable still provenance changed")
    return expected


def _build_staging_manifest(root: Path) -> dict[str, Any]:
    root = _require_bundle_root(root, create=False)
    artifacts = []
    for relative in sorted(STAGING_PATHS):
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"incline staging artifact is missing: {relative}")
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
    root = _require_bundle_root(root, create=False)
    if not isinstance(payload, dict) or set(payload) != {
        "schema_version",
        "disposition",
        "artifact_count",
        "artifacts",
    }:
        raise ValueError("malformed incline pruned-staging provenance")
    if (
        payload.get("schema_version") != STAGING_SCHEMA_VERSION
        or payload.get("disposition") != "pruned_before_seal"
    ):
        raise ValueError("incline pruned-staging contract changed")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list) or payload.get("artifact_count") != len(
        artifacts
    ):
        raise ValueError("incline pruned-staging count changed")
    expected_paths = sorted(STAGING_PATHS)
    paths = [item.get("path") for item in artifacts if isinstance(item, dict)]
    if paths != expected_paths or len(paths) != len(artifacts):
        raise ValueError("incline pruned-staging membership changed")
    by_path = {}
    for item in artifacts:
        if set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("malformed incline pruned-staging artifact")
        relative = item["path"]
        if (
            not isinstance(item["bytes"], int)
            or isinstance(item["bytes"], bool)
            or item["bytes"] < 0
            or not isinstance(item["sha256"], str)
            or len(item["sha256"]) != 64
            or any(character not in "0123456789abcdef" for character in item["sha256"])
        ):
            raise ValueError(f"malformed incline pruned-staging identity: {relative}")
        path = root / relative
        if require_absent:
            if path.exists() or path.is_symlink():
                raise ValueError(f"incline staging artifact survived seal: {relative}")
        elif (
            not path.is_file()
            or path.is_symlink()
            or path.stat().st_size != item["bytes"]
            or sha256(path) != item["sha256"]
        ):
            raise ValueError(f"incline staging artifact changed: {relative}")
        by_path[relative] = item
    return by_path


def _prune_staging_outputs(root: Path, manifest: dict[str, Any]) -> None:
    root = _require_bundle_root(root, create=False)
    _validate_staging_manifest(root, manifest, require_absent=False)
    for relative in sorted(STAGING_PATHS):
        (root / relative).unlink()
    for relative in ("incline/panel_frames", "incline/frames"):
        (root / relative).rmdir()
    _validate_staging_manifest(root, manifest, require_absent=True)


def _strict_nonnegative_int(value: Any, *, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} is unavailable/invalid")
    return value


def _capture_count_projection(timeline: dict[str, Any]) -> list[dict[str, Any]]:
    steps = timeline.get("steps")
    if not isinstance(steps, dict) or set(steps) != {
        str(step) for step in range(TOTAL_STEPS + 1)
    }:
        raise ValueError("incline capture timeline steps are unavailable/incomplete")
    projection = []
    for step in range(TOTAL_STEPS + 1):
        entry = steps[str(step)]
        if not isinstance(entry, dict):
            raise ValueError(f"incline capture timeline step {step} is malformed")
        diagnostics = entry.get("solver_diagnostics")
        if not isinstance(diagnostics, dict):
            raise ValueError(f"incline capture diagnostics missing at step {step}")
        contacts = _strict_nonnegative_int(
            diagnostics.get("contacts"), label=f"step {step} contacts"
        )
        expected_contacts = 0 if step == 0 else CURRENT_CAPTURE_TOTAL_CONTACTS
        if contacts != expected_contacts:
            raise ValueError(
                f"incline capture step {step} contacts {contacts} != "
                f"{expected_contacts}"
            )
        projection.append(
            {
                "step": step,
                "exact_solves": _strict_nonnegative_int(
                    diagnostics.get("exact_solves"),
                    label=f"step {step} exact_solves",
                ),
                "boxed_lcp_fallbacks": _strict_nonnegative_int(
                    diagnostics.get("boxed_lcp_fallbacks"),
                    label=f"step {step} boxed_lcp_fallbacks",
                ),
            }
        )
    return projection


def _compare_aggregate_count_projections(
    trace_projection: Sequence[dict[str, Any]],
    capture_projection: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    trace_projection = list(trace_projection)
    capture_projection = list(capture_projection)
    expected_fields = set(COUNT_PROJECTION_FIELDS)
    if any(set(row) != expected_fields for row in trace_projection) or any(
        set(row) != expected_fields for row in capture_projection
    ):
        raise ValueError("incline shared count projection fields changed")
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
            "incline aggregate trace/capture count projection differs at " f"{mismatch}"
        )
    digest = _payload_sha256(trace_projection)
    return {
        "pass": True,
        "row_count": len(trace_projection),
        "fields": list(COUNT_PROJECTION_FIELDS),
        "trace_sha256": digest,
        "capture_sha256": digest,
        "byte_identical": True,
        "aggregate_counts_only": True,
        "contact_counts_compared": False,
        "contact_count_match": False,
        "trace_aggregate_contacts_per_post_initial_step": (
            len(SCENARIOS) * CURRENT_DART_CONTACTS
        ),
        "capture_contacts_per_post_initial_step": CURRENT_CAPTURE_TOTAL_CONTACTS,
        "full_state_trace_equivalence": False,
        "per_cell_trace_equivalence": False,
        "limitation": (
            "Only additive cumulative exact-solve and cumulative fallback counts are "
            "compared. The combined capture reports eight contacts per post-initial "
            "step while the two traces report six in aggregate, so contact counts are "
            "explicitly excluded. The rendered scene and independent tracked traces "
            "use different placements; state, residual, status, warm-start, and "
            "per-cell equivalence are not established."
        ),
    }


def _expected_demo_argv(demo: Path, root: Path) -> list[str]:
    output_dir = root / "incline"
    argv = [
        str(demo),
        "--scene",
        "fbf_paper_incline",
        "--headless",
        "--steps",
        str(TOTAL_STEPS),
        "--width",
        "1280",
        "--height",
        "720",
        "--collision-detector",
        "dart",
        "--threads",
        "1",
        "--headless-sidecar",
        str(output_dir / "timeline.json"),
    ]
    for step in CAPTURE_STEPS:
        argv.extend(
            (
                "--headless-shot-at",
                f"{step}:{output_dir / 'frames' / f'step_{step:06d}.png'}",
            )
        )
    return argv


def _validate_media_item(
    root: Path,
    stored: dict[str, Any],
    verified: dict[str, Any],
) -> dict[str, Any]:
    path = root / "incline/clip.mp4"
    expected_path = str(path)
    expected_stream = {
        "width": 660,
        "height": 506,
        "frame_rate": "30/1",
        "frame_rate_rational": "30/1",
        "frame_count": 61,
    }
    for item, label in ((stored, "stored"), (verified, "verified")):
        if (
            not isinstance(item, dict)
            or item.get("kind") != "mp4"
            or item.get("path") != expected_path
            or item.get("sha256") != sha256(path)
            or item.get("full_decode") != "pass"
            or item.get("stream_contract") != expected_stream
        ):
            raise ValueError(f"incline MP4 {label} contract changed")
    return {
        "path": expected_path,
        "sha256": sha256(path),
        "bytes": path.stat().st_size,
        "full_decode": "pass",
        "stream_contract": expected_stream,
    }


def _validate_capture_final_diagnostics(diagnostics: Any) -> dict[str, Any]:
    if not isinstance(diagnostics, dict):
        raise ValueError("incline capture final solver diagnostics are unavailable")
    exact_attempts = _strict_nonnegative_int(
        diagnostics.get("exact_attempts"), label="final exact_attempts"
    )
    exact_solves = _strict_nonnegative_int(
        diagnostics.get("exact_solves"), label="final exact_solves"
    )
    worst_residual = diagnostics.get("worst_residual")
    if (
        diagnostics.get("available") is not True
        or exact_attempts != EXPECTED_CAPTURE_EXACT_GROUPS
        or exact_solves != EXPECTED_CAPTURE_EXACT_GROUPS
        or diagnostics.get("accepted_at_cap") != 0
        or diagnostics.get("exact_failures") != 0
        or diagnostics.get("boxed_lcp_fallbacks") != 0
        or diagnostics.get("status") != "success"
        or diagnostics.get("fbf_status") != "success"
        or isinstance(worst_residual, bool)
        or not isinstance(worst_residual, (int, float))
        or not math.isfinite(float(worst_residual))
        or float(worst_residual) < 0.0
        or float(worst_residual) > RESIDUAL_TOLERANCE
    ):
        raise ValueError("incline capture solver diagnostics changed")
    return diagnostics


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
        set(verification)
        != {"schema_version", "kind", "results", "group_outputs", "pass"}
        or verification.get("schema_version") != RUNNER_SCHEMA_VERSION
        or verification.get("kind") != "verification"
        or verification.get("pass") is not True
        or verification.get("group_outputs") != []
    ):
        raise ValueError("incline capture verification is not a passing singleton")
    verify_results = _capture_result_by_id(verification)
    if set(verify_results) != {"incline"}:
        raise ValueError("incline capture verification member set changed")

    run_summary = read_json(root / "run-summary.json")
    if (
        set(run_summary)
        != {
            "schema_version",
            "kind",
            "results",
            "group_outputs",
            "group_skips",
            "failures",
            "pass",
        }
        or run_summary.get("schema_version") != RUNNER_SCHEMA_VERSION
        or run_summary.get("kind") != "capture_run"
        or run_summary.get("pass") is not True
        or run_summary.get("failures") != []
        or run_summary.get("group_outputs") != []
        or run_summary.get("group_skips") != []
    ):
        raise ValueError("incline run-summary is not a complete passing singleton")
    run_results = _capture_result_by_id(run_summary)
    if set(run_results) != {"incline"}:
        raise ValueError("incline run-summary member set changed")

    metadata_path = root / "incline/metadata.json"
    metadata = read_json(metadata_path)
    if run_results["incline"] != metadata:
        raise ValueError("incline run-summary no longer equals metadata.json")
    if (
        set(metadata)
        != {
            "schema_version",
            "kind",
            "schedule",
            "runtime",
            "timeline_validation",
            "panel_validation",
            "media_validation",
            "actual_simulator",
            "generated_imagery",
            "paper_comparable",
            "automated_semantic_outcome_validated",
            "semantic_outcome_gate",
            "known_mismatches",
            "pass",
        }
        or metadata.get("schema_version") != RUNNER_SCHEMA_VERSION
        or metadata.get("kind") != "capture_result"
        or metadata.get("pass") is not True
        or metadata.get("actual_simulator") is not True
        or metadata.get("generated_imagery") is not False
        or metadata.get("paper_comparable") is not False
        or metadata.get("automated_semantic_outcome_validated") is not False
        or metadata.get("semantic_outcome_gate") != SEMANTIC_OUTCOME_GATE
        or metadata.get("known_mismatches") != EXPECTED_MISMATCHES
    ):
        raise ValueError("incline capture claim flags changed")

    schedule = metadata.get("schedule")
    if not isinstance(schedule, dict):
        raise ValueError("incline capture schedule is missing")
    expected_output = {
        "directory": str(root / "incline"),
        "timeline": str(root / "incline/timeline.json"),
        "panel": str(root / "incline/panel.png"),
        "mp4": str(root / "incline/clip.mp4"),
        "gif": None,
    }
    expected_demo_argv = _expected_demo_argv(demo, root)
    expected_schedule = {
        "id": "incline",
        "title": "Incline threshold pair",
        "scene": "fbf_paper_incline",
        "source_segment": "incline",
        "collision_detector": "dart",
        "collision_detector_override": True,
        "configuration": EXPECTED_CONFIGURATION,
        "total_steps": TOTAL_STEPS,
        "time_step_seconds": DT_SECONDS,
        "capture_steps": list(CAPTURE_STEPS),
        "panel_steps": list(PANEL_STEPS),
        "panel_labels": list(PANEL_LABELS),
        "actions": [],
        "output": expected_output,
        "runnable": True,
        "adapter_gap": None,
        "long_run": False,
        "actual_simulator_required": True,
        "generated_imagery_allowed": False,
        "paper_comparable": False,
        "known_mismatches": EXPECTED_MISMATCHES,
        "known_gate_blockers": [],
        "evidence_ready": True,
        "demo_argv": expected_demo_argv,
        "demo_command": shlex.join(expected_demo_argv),
    }
    if schedule != expected_schedule:
        raise ValueError("incline capture schedule contract changed")
    if metadata.get("known_mismatches") != schedule["known_mismatches"]:
        raise ValueError("incline capture mismatch disclosures changed")

    runtime = metadata.get("runtime")
    if (
        not isinstance(runtime, dict)
        or set(runtime)
        != {"demo_argv", "demo_path", "demo_sha256", "ffmpeg", "ffprobe"}
        or runtime.get("demo_path") != str(demo)
        or runtime.get("demo_sha256") != sha256(demo)
        or runtime.get("demo_argv") != expected_demo_argv
        or runtime.get("ffmpeg") != str(ffmpeg)
        or runtime.get("ffprobe") != str(ffprobe)
    ):
        raise ValueError("incline capture runtime identity changed")

    verified = verify_results["incline"]
    if (
        set(verified)
        != {
            "schedule",
            "timeline",
            "panel",
            "media",
            "metadata_path",
            "metadata_sha256",
            "pass",
        }
        or verified.get("pass") is not True
        or verified.get("schedule") != "incline"
        or verified.get("metadata_path") != str(metadata_path)
        or verified.get("metadata_sha256") != sha256(metadata_path)
    ):
        raise ValueError("incline verification metadata binding changed")
    timeline = metadata.get("timeline_validation")
    if not isinstance(timeline, dict) or verified.get("timeline") != timeline:
        raise ValueError("incline stored/fresh timeline validation differs")
    expected_steps = {str(step) for step in range(TOTAL_STEPS + 1)}
    expected_frames = {str(step) for step in CAPTURE_STEPS}
    if (
        timeline.get("pass") is not True
        or timeline.get("sidecar") != str(root / "incline/timeline.json")
        or timeline.get("step_count") != TOTAL_STEPS + 1
        or timeline.get("shot_count") != len(CAPTURE_STEPS)
        or timeline.get("action_count") != 0
        or timeline.get("unique_frame_hashes") != len(CAPTURE_STEPS)
        or set(timeline.get("steps", {})) != expected_steps
        or set(timeline.get("frames", {})) != expected_frames
    ):
        raise ValueError("incline capture timeline contract changed")
    for step in CAPTURE_STEPS:
        frame_path = root / "incline/frames" / f"step_{step:06d}.png"
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
            raise ValueError(f"incline capture frame binding changed at step {step}")
    world_hashes = {
        timeline["frames"][str(step)].get("world_viewport", {}).get("sha256")
        for step in CAPTURE_STEPS
    }
    if None in world_hashes or len(world_hashes) != len(CAPTURE_STEPS):
        raise ValueError("incline capture world frames are not all distinct")

    final_diagnostics = _validate_capture_final_diagnostics(
        timeline.get("final_solver_diagnostics")
    )

    panel = metadata.get("panel_validation")
    panel_path = root / "incline/panel.png"
    if not isinstance(panel, dict):
        raise ValueError("incline panel validation is missing")
    expected_panel_sources = []
    for step in PANEL_STEPS:
        source = root / "incline/panel_frames" / f"step_{step:06d}.png"
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
    compose_path = root / "incline/panel.compose.json"
    if (
        panel.get("path") != str(panel_path)
        or panel.get("sha256") != sha256(panel_path)
        or panel.get("source_frames") != expected_panel_sources
        or panel.get("compose_manifest_path") != str(compose_path)
        or panel.get("compose_manifest_sha256") != sha256(compose_path)
        or panel.get("compose_manifest") != read_json(compose_path)
        or panel.get("verdict", {}).get("pass") is not True
        or panel.get("verdict", {}).get("image", {}).get("width") != 3348
        or panel.get("verdict", {}).get("image", {}).get("height") != 538
        or verified.get("panel", {}).get("pass") is not True
        or verified.get("panel", {}).get("image", {}).get("path") != str(panel_path)
    ):
        raise ValueError("incline panel contract changed")

    stored_media = metadata.get("media_validation")
    verified_media = verified.get("media")
    if (
        not isinstance(stored_media, list)
        or len(stored_media) != 1
        or not isinstance(verified_media, list)
        or len(verified_media) != 1
    ):
        raise ValueError("incline media member set changed")
    media = _validate_media_item(root, stored_media[0], verified_media[0])

    projection = _capture_count_projection(timeline)
    return {
        "pass": True,
        "metadata_sha256": sha256(metadata_path),
        "timeline_sha256": sha256(root / "incline/timeline.json"),
        "panel_sha256": sha256(panel_path),
        "captured_frames": len(CAPTURE_STEPS),
        "unique_frames": len(CAPTURE_STEPS),
        "panel_steps": list(PANEL_STEPS),
        "exact_attempts": final_diagnostics["exact_attempts"],
        "exact_solves": final_diagnostics["exact_solves"],
        "accepted_at_cap": final_diagnostics["accepted_at_cap"],
        "exact_failures": final_diagnostics["exact_failures"],
        "boxed_lcp_fallbacks": final_diagnostics["boxed_lcp_fallbacks"],
        "observed_contacts_per_post_initial_step": CURRENT_CAPTURE_TOTAL_CONTACTS,
        "worst_residual": final_diagnostics["worst_residual"],
        "media": media,
        "run_summary_sha256": sha256(root / "run-summary.json"),
        "verification_sha256": sha256(root / "verification.json"),
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "paper_comparable": False,
        "count_projection": projection,
        "count_projection_sha256": _payload_sha256(projection),
    }


def _process_group_exists(process_group_id: int) -> bool:
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    return True


def _kill_process_group(process_group_id: int, signal_number: int) -> None:
    try:
        os.killpg(process_group_id, signal_number)
    except ProcessLookupError:
        pass


def _terminate_process_group(
    process: subprocess.Popen[str],
) -> tuple[str, str]:
    termination_started = time.monotonic()
    _kill_process_group(process.pid, signal.SIGTERM)
    try:
        try:
            stdout, stderr = process.communicate(
                timeout=PROCESS_TERMINATION_GRACE_SECONDS
            )
        except subprocess.TimeoutExpired:
            _kill_process_group(process.pid, signal.SIGKILL)
            return process.communicate()

        deadline = termination_started + PROCESS_TERMINATION_GRACE_SECONDS
        while _process_group_exists(process.pid) and time.monotonic() < deadline:
            time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
        return stdout, stderr
    finally:
        _kill_process_group(process.pid, signal.SIGKILL)


def _run_command(
    argv: Sequence[str], *, timeout: float
) -> subprocess.CompletedProcess[str]:
    command = [str(item) for item in argv]
    process = subprocess.Popen(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as error:
        try:
            stdout, stderr = _terminate_process_group(process)
        except BaseException:
            _kill_process_group(process.pid, signal.SIGKILL)
            raise
        raise subprocess.TimeoutExpired(
            command,
            timeout,
            output=stdout,
            stderr=stderr,
        ) from error
    except BaseException:
        try:
            _terminate_process_group(process)
        except BaseException:
            _kill_process_group(process.pid, signal.SIGKILL)
        raise
    if _process_group_exists(process.pid):
        _kill_process_group(process.pid, signal.SIGKILL)
    completed = subprocess.CompletedProcess(
        command,
        process.returncode,
        stdout,
        stderr,
    )
    if completed.returncode != 0:
        raise ValueError(
            f"command failed with exit {completed.returncode}: {list(argv)!r}\n"
            f"stderr:\n{completed.stderr}"
        )
    return completed


def _trace_argv(trace_binary: Path, scenario: str) -> list[str]:
    if scenario not in SCENARIOS:
        raise ValueError(f"unsupported incline trace scenario: {scenario}")
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
        "incline",
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
        "incline",
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


def _parse_json_stdout(text: str, *, label: str) -> dict[str, Any]:
    def reject_nonfinite(value: str) -> None:
        raise ValueError(f"{label}: non-finite JSON number {value}")

    try:
        payload = json.loads(text, parse_constant=reject_nonfinite)
    except json.JSONDecodeError as error:
        raise ValueError(f"{label} did not emit one JSON document") from error
    if not isinstance(payload, dict):
        raise ValueError(f"{label} did not emit a JSON object")
    return payload


def _binary_source_binding_identity(
    *, demo: Path, trace_binary: Path
) -> dict[str, dict[str, Any]]:
    demo = _require_file(demo, "DART demo binary")
    trace_binary = _require_file(trace_binary, "trace binary")
    query_specs = (
        (
            "demo_source_binding",
            demo,
            [
                str(demo),
                "--fbf-author-turntable-contract",
                "fbf_author_turntable_mu_0_2_omega_2",
            ],
            "dart_demos",
            DEMO_SOURCE,
        ),
        (
            "trace_source_binding",
            trace_binary,
            [
                str(trace_binary),
                "--author-turntable-contract",
                "turntable_author_mu_0_2_omega_2",
                "dart_best",
            ],
            "fbf_paper_trace",
            TRACE_SOURCE,
        ),
    )
    identity: dict[str, dict[str, Any]] = {}
    for label, binary, argv, role, implementation_source in query_specs:
        process = _run_command(argv, timeout=30.0)
        if process.stderr:
            raise ValueError(f"{label} query emitted unexpected stderr")
        payload = _parse_json_stdout(process.stdout, label=label)
        expected_binding = {
            "role": role,
            "implementation_source_sha256": sha256(implementation_source),
        }
        scenario = payload.get("scenario")
        if (
            payload.get("schema_version")
            != "dart.fbf_author_turntable_physics_contract/v1"
            or payload.get("kind") != "physics_control"
            or not isinstance(scenario, dict)
            or scenario.get("trace_id") != "turntable_author_mu_0_2_omega_2"
            or payload.get("binary_binding") != expected_binding
        ):
            raise ValueError(f"{label} does not bind the current implementation source")
        identity[label] = {
            "query_argv": argv,
            "query_payload_sha256": _payload_sha256(payload),
            "binary_path": str(binary),
            "binary_sha256": sha256(binary),
            "implementation_source_path": str(implementation_source.resolve()),
            "implementation_source_sha256": sha256(implementation_source),
            "validated_binary_binding": expected_binding,
        }
    return identity


def _parse_ldd_in_tree_paths(output: str, *, build_root: Path) -> list[Path]:
    paths: set[Path] = set()
    resolved_build_root = build_root.resolve(strict=True)
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
            path.relative_to(resolved_build_root)
        except ValueError:
            continue
        paths.add(path)
    return sorted(paths, key=lambda path: path.as_posix())


def _in_tree_runtime_dependency_identity(
    executable: Path, *, label: str
) -> dict[str, dict[str, str]]:
    executable = _require_file(executable, f"{label} executable")
    process = _run_command(["ldd", str(executable)], timeout=30.0)
    dependencies = _parse_ldd_in_tree_paths(
        process.stdout, build_root=BUILD_RUNTIME_ROOT
    )
    if not any(path.name.startswith("libdart.so") for path in dependencies):
        raise ValueError(f"{label}: resolved in-tree libdart dependency is missing")
    build_root = BUILD_RUNTIME_ROOT.resolve(strict=True)
    return {
        f"{label}_dependency_{path.relative_to(build_root).as_posix().replace('/', '__')}": {
            "path": str(path),
            "sha256": sha256(path),
        }
        for path in dependencies
    }


def _capture_runtime_identity(
    *,
    python: Path,
    runner: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, dict[str, str]]:
    inputs = {
        "visual_runner": runner,
        "image_compose": IMAGE_COMPOSE,
        "image_tools": IMAGE_TOOLS,
        "image_verdict": IMAGE_VERDICT,
        "demo_source": DEMO_SOURCE,
        "demo_host_source": DEMO_HOST_SOURCE,
        "demo_host_header": DEMO_HOST_HEADER,
        "registry_source": REGISTRY_SOURCE,
        "scenes_header": SCENES_HEADER,
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
        for name, path in inputs.items()
    } | _in_tree_runtime_dependency_identity(demo, label="demo")


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
    identity: dict[str, Any] = {
        name: {
            "path": str(_require_file(path, name)),
            "sha256": sha256(_require_file(path, name)),
        }
        for name, path in sources.items()
    }
    identity.update(_in_tree_runtime_dependency_identity(demo, label="demo"))
    identity.update(_in_tree_runtime_dependency_identity(trace_binary, label="trace"))
    identity["binary_source_bindings"] = _binary_source_binding_identity(
        demo=demo, trace_binary=trace_binary
    )
    return identity


def _validate_capture_provenance(
    payload: Any,
    *,
    bundle: Path,
    trace_binary: Path,
    python: Path,
    runner: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
    sealed: bool = False,
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("incline capture provenance is missing")
    if not sealed:
        run_summary = read_json(bundle / "run-summary.json")
        expected_summary_stdout = (
            json.dumps(run_summary, indent=2, sort_keys=True) + "\n"
        )
        if (
            not (bundle / "capture.stdout.txt")
            .read_text(encoding="utf-8")
            .endswith(expected_summary_stdout)
        ):
            raise ValueError(
                "incline capture stdout does not end with run-summary.json"
            )
    expected_identity = _capture_runtime_identity(
        python=python,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    expected_binary_bindings = _binary_source_binding_identity(
        demo=demo, trace_binary=trace_binary
    )
    expected = {
        "argv": _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "stdout_path": "capture.stdout.txt",
        "stdout_sha256": None,
        "stdout_ends_with_run_summary": True,
        "stderr_path": "capture.stderr.txt",
        "stderr_sha256": None,
        "runtime_inputs_before": expected_identity,
        "runtime_inputs_after": expected_identity,
        "binary_source_bindings_before": expected_binary_bindings,
        "binary_source_bindings_after": expected_binary_bindings,
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
        raise ValueError("incline capture-time provenance changed")
    return payload


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
    if set(payload) != {"schema_version", "traces", "capture_verification"}:
        raise ValueError("malformed incline invocations payload")
    if payload.get("schema_version") != INVOCATIONS_SCHEMA_VERSION:
        raise ValueError("unexpected incline invocations schema")
    expected_traces = []
    for scenario in SCENARIOS:
        stdout_path = bundle / "traces" / f"{scenario}.csv"
        stderr_path = bundle / "traces" / f"{scenario}.stderr.txt"
        expected_traces.append(
            {
                "scenario": scenario,
                "argv": _trace_argv(trace_binary, scenario),
                "returncode": 0,
                "stdout_path": stdout_path.relative_to(bundle).as_posix(),
                "stdout_sha256": sha256(stdout_path),
                "stderr_path": stderr_path.relative_to(bundle).as_posix(),
                "stderr_sha256": sha256(stderr_path),
            }
        )
    if payload.get("traces") != expected_traces:
        raise ValueError("incline trace invocation bindings changed")
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
        raise ValueError("incline verification invocation binding changed")
    return payload


def _build_trace_summary(
    parsed: Sequence[dict[str, Any]],
    capture_summary: dict[str, Any],
    bundle: Path,
) -> dict[str, Any]:
    pair = summarize_trace_pair(parsed)
    aggregate_projection = pair.pop("aggregate_count_projection")
    comparison = _compare_aggregate_count_projections(
        aggregate_projection, capture_summary["count_projection"]
    )
    pair["trace_artifacts"] = [
        {
            "scenario": scenario,
            "path": f"traces/{scenario}.csv",
            "sha256": sha256(bundle / "traces" / f"{scenario}.csv"),
        }
        for scenario in SCENARIOS
    ]
    pair["aggregate_count_projection_comparison"] = comparison
    return pair


def _report_markdown(
    trace_summary: dict[str, Any], capture_summary: dict[str, Any]
) -> str:
    slide, stick = trace_summary["scenarios"]
    separation = trace_summary["threshold_separation"]
    comparison = trace_summary["aggregate_count_projection_comparison"]
    stream = capture_summary["media"]["stream_contract"]
    return f"""# Figures 1-2 incline current-source DART evidence

Status: valid current-source DART incline evidence; not paper parity.

Two fresh 121-row exact-FBF traces independently validate the reconstructed
two-second threshold outcomes. At `mu=.4`, the downhill displacement is
{slide['downhill_displacement_m']:.17g} m versus the analytical
{slide['expected_downhill_displacement_m']:.17g} m target with a
{slide['displacement_tolerance_m']:.17g} m fixture tolerance. At `mu=.5`, the
downhill displacement is {stick['downhill_displacement_m']:.17g} m with a
{stick['displacement_tolerance_m']:.17g} m stick tolerance. The slide-minus-stick
separation is {separation['slide_minus_stick_m']:.17g} m and exceeds the frozen
{separation['minimum_exclusive_m']:.1f} m threshold. Both traces retain
three DART contacts at every post-initial step, complete without boxed-LCP
fallback, and keep every solved-step residual at or below
{RESIDUAL_TOLERANCE:.1e}. Full-trajectory gates also bound cross-slope motion,
incline-normal center-offset drift, and the exported `up_z` alignment; the
exported velocity components must match step-to-step position differences,
cross-slope and incline-normal speeds remain bounded, the stick cell stays in
both its displacement and speed bands, and the slide finishes with positive
downhill speed without regressing beyond its frozen numerical tolerance.

The combined actual-simulator capture completes {TOTAL_STEPS} steps with
{capture_summary['captured_frames']} distinct world-view frames, zero accepted
caps, zero exact failures, zero boxed-LCP fallbacks, and worst residual
{capture_summary['worst_residual']:.17g}. Its MP4 is {stream['width']}x
{stream['height']} at {stream['frame_rate']} with {stream['frame_count']} fully
decoded frames. Recorded manual inspection binds the five-time panel, durable
source-frame copies, and clip to the visible low-friction slide versus
near-threshold stick.
The combined timeline records
{capture_summary['observed_contacts_per_post_initial_step']} contacts per
post-initial step, whereas the two independent traces record six in aggregate
(three per trace). The capture does not export a per-cell contact split.

The traces and combined capture have a byte-identical
{comparison['row_count']}-row additive count projection over
{', '.join(comparison['fields'])}. Contact counts are explicitly excluded from
that projection because the measured eight-versus-six counts differ. This is
explicitly not state or full-trace equivalence: the combined renderer offsets
the two cells for presentation, while each tracked CSV is an independent
single-cell run. Residual, status, warm-start, state, and per-cell capture
equivalence are not compared.

Claim boundary: this is current-source DART reconstruction evidence. The DART
tracked traces report three contacts per cell, the combined capture reports
eight contacts in aggregate without a per-cell split, and the paper timing row
reports four. The raw runner schedule
retains a legacy three-contacts-per-cell mismatch note; the measured aggregate
timeline and traces reported above supersede that note for this bundle. The
tracked CSV does not directly export maximum penetration. No full friction
sweep, source-matched analytical plot, faithful external-solver media, approved
source golden/diff, paper timing, or real-time verdict is supplied. The
validated runner timeline remains internal bundle evidence and is not promoted
as a standalone manifest capture-sidecar deliverable.
"""


CLAIM_SCOPE = (
    "Current-source DART two-second incline threshold: manual combined visual "
    "slide/stick classification independently supported by two tracked physical "
    "outcome traces and an additive exact-solve/fallback count-only capture/trace "
    "projection."
)
CLAIM_BOUNDARY = (
    "The combined rendered scene and the two tracked CSV exporters use different "
    "placements. Only additive exact-solve/fallback counts are equivalent; capture "
    "contacts are eight per post-initial step while the traces total six, so contact "
    "counts are not equivalent. State, residual, status, warm-start, per-cell, and "
    "full-trace equivalence are not claimed. This is not a full friction sweep, paper "
    "contact-count match, maximum-penetration proof, author-scene or paper parity, "
    "faithful external-solver parity, approved source golden/diff, paper timing, or "
    "real-time evidence. The runner timeline is not promoted as a standalone "
    "manifest capture-sidecar deliverable."
)
EXPECTED_METADATA_FLAGS = {
    "schema_version": SCHEMA_VERSION,
    "status": "valid_current_source_nonpaper_incline",
    "pass": True,
    "requirement_ids": ["fig.01", "fig.02", "video.03_incline"],
    "artifact_valid": True,
    "solver_contract_valid": True,
    "physical_outcome_valid": True,
    "paper_parity": False,
    "paper_comparable": False,
    "external_solver_parity": False,
    "approved_source_golden": False,
    "approved_source_golden_diff": False,
    "capture_sidecar_deliverable_validated": False,
    "full_friction_sweep": False,
    "timing_verdict": None,
    "realtime_verdict": None,
    "actual_simulator": True,
    "generated_imagery": False,
    "automated_semantic_outcome_validated": False,
    "manual_visual_outcome_validated": True,
    "aggregate_count_projection_equivalent": True,
    "aggregate_counts_only": True,
    "capture_trace_contact_count_equivalent": False,
    "trace_equivalence_to_rendered_demo": False,
    "full_state_trace_equivalence": False,
    "per_cell_trace_equivalence": False,
    "tracked_trace_continuous_contact_proven": True,
    "maximum_penetration_proven": False,
    "paper_reference_contact_count_match": False,
}
EXPECTED_FINAL_METADATA_KEYS = set(EXPECTED_METADATA_FLAGS) | {
    "evidence_date",
    "claim_scope",
    "trace_summary",
    "capture_summary",
    "capture_provenance",
    "manual_inspection",
    "run_summary",
    "verification",
    "invocations",
    "report",
    "artifact_index",
    "source_identity",
    "claim_boundary",
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
def _bundle_transaction(
    bundle: Path,
) -> Iterator[Callable[[], None]]:
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
                raise ValueError("incline bundle transaction was not committed")
        except BaseException:
            if not committed:
                _remove_bundle_root(bundle)
                shutil.copytree(snapshot, bundle)
            raise


def _finalize_transaction(
    args: argparse.Namespace,
    *,
    bundle: Path,
    trace_binary: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
    source_identity_before: dict[str, Any],
    commit: Callable[[], None],
) -> dict[str, Any]:
    _clean_generated_outputs(bundle)
    if not args.reuse_current_capture:
        _clean_capture_outputs(bundle)
    _validate_bundle_paths(
        bundle,
        complete=False,
        allow_missing_capture_provenance=not args.reuse_current_capture,
        allow_missing_capture=not args.reuse_current_capture,
    )

    if args.reuse_current_capture:
        capture_provenance = read_json(bundle / "capture-provenance.json")
    else:
        binary_source_bindings_before = _binary_source_binding_identity(
            demo=demo, trace_binary=trace_binary
        )
        runtime_before = _capture_runtime_identity(
            python=python,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
        capture_argv = _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe)
        capture_process = _run_command(capture_argv, timeout=args.capture_timeout)
        runtime_after = _capture_runtime_identity(
            python=python,
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        )
        binary_source_bindings_after = _binary_source_binding_identity(
            demo=demo, trace_binary=trace_binary
        )
        if runtime_after != runtime_before:
            raise ValueError("incline capture runtime inputs changed during capture")
        if binary_source_bindings_after != binary_source_bindings_before:
            raise ValueError("incline binary/source bindings changed during capture")
        capture_stdout_path = bundle / "capture.stdout.txt"
        capture_stderr_path = bundle / "capture.stderr.txt"
        capture_stdout_path.write_text(capture_process.stdout, encoding="utf-8")
        capture_stderr_path.write_text(capture_process.stderr, encoding="utf-8")
        run_summary_path = bundle / "run-summary.json"
        capture_payload = read_json(run_summary_path)
        if (
            capture_payload.get("kind") != "capture_run"
            or capture_payload.get("pass") is not True
            or capture_payload.get("failures") != []
            or capture_payload.get("group_outputs") != []
            or capture_payload.get("group_skips") != []
            or len(capture_payload.get("results", [])) != 1
            or capture_payload["results"][0].get("schedule", {}).get("id") != "incline"
        ):
            raise ValueError(
                "visual capture did not produce one passing incline result"
            )
        expected_summary_stdout = (
            json.dumps(capture_payload, indent=2, sort_keys=True) + "\n"
        )
        if not capture_process.stdout.endswith(expected_summary_stdout):
            raise ValueError("incline capture stdout does not end with run-summary")
        capture_provenance = {
            "argv": capture_argv,
            "returncode": capture_process.returncode,
            "run_summary_path": "run-summary.json",
            "run_summary_sha256": sha256(run_summary_path),
            "run_summary_validated": True,
            "stdout_path": "capture.stdout.txt",
            "stdout_sha256": sha256(capture_stdout_path),
            "stdout_ends_with_run_summary": True,
            "stderr_path": "capture.stderr.txt",
            "stderr_sha256": sha256(capture_stderr_path),
            "runtime_inputs_before": runtime_before,
            "runtime_inputs_after": runtime_after,
            "binary_source_bindings_before": binary_source_bindings_before,
            "binary_source_bindings_after": binary_source_bindings_after,
        }
        write_json(bundle / "capture-provenance.json", capture_provenance)
    _validate_capture_provenance(
        capture_provenance,
        bundle=bundle,
        trace_binary=trace_binary,
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
            "fresh incline capture is complete; manually inspect the bound "
            "artifacts, add manual-inspection.json, then rerun with "
            "--reuse-current-capture"
        )

    parsed_traces = []
    trace_invocations = []
    for scenario in SCENARIOS:
        argv = _trace_argv(trace_binary, scenario)
        process = _run_command(argv, timeout=args.trace_timeout)
        stdout_path = bundle / "traces" / f"{scenario}.csv"
        stderr_path = bundle / "traces" / f"{scenario}.stderr.txt"
        stdout_path.parent.mkdir(parents=True, exist_ok=True)
        stdout_path.write_text(process.stdout, encoding="utf-8")
        stderr_path.write_text(process.stderr, encoding="utf-8")
        parsed_traces.append(parse_trace_text(process.stdout, scenario))
        trace_invocations.append(
            {
                "scenario": scenario,
                "argv": argv,
                "returncode": process.returncode,
                "stdout_path": stdout_path.relative_to(bundle).as_posix(),
                "stdout_sha256": sha256(stdout_path),
                "stderr_path": stderr_path.relative_to(bundle).as_posix(),
                "stderr_sha256": sha256(stderr_path),
            }
        )

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
    trace_summary = _build_trace_summary(parsed_traces, capture_summary, bundle)
    write_json(bundle / "trace-summary.json", trace_summary)
    manual = validate_manual_inspection(bundle)

    invocations = {
        "schema_version": INVOCATIONS_SCHEMA_VERSION,
        "traces": trace_invocations,
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
    report = _report_markdown(trace_summary, capture_summary)
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
        trace_binary=trace_binary,
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
            "incline bundle is not ready for indexing: "
            f"missing={sorted(expected_before_index - files_before_index)}, "
            f"extra={sorted(files_before_index - expected_before_index)}"
        )
    index = artifact_index(bundle)
    write_json(bundle / "artifact-index.json", index)

    capture_metadata = dict(capture_summary)
    capture_metadata.pop("count_projection")
    source_identity_after = _source_identity(
        trace_binary=trace_binary,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    if source_identity_after != source_identity_before:
        raise ValueError("incline source/binary/runtime inputs changed during reseal")
    metadata = {
        **EXPECTED_METADATA_FLAGS,
        "evidence_date": args.evidence_date,
        "claim_scope": CLAIM_SCOPE,
        "trace_summary": trace_summary,
        "capture_summary": capture_metadata,
        "capture_provenance": capture_provenance,
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
        "source_identity": source_identity_after,
        "claim_boundary": CLAIM_BOUNDARY,
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
    bundle = _require_bundle_root(args.bundle, create=not args.reuse_current_capture)
    _validate_bundle_paths(
        bundle,
        complete=False,
        allow_missing_capture_provenance=not args.reuse_current_capture,
        allow_missing_capture=not args.reuse_current_capture,
    )
    if not args.reuse_current_capture and (bundle / "metadata.json").is_file():
        raise ValueError(
            "fresh incline capture refuses to replace an existing finalized "
            "bundle; choose a separate --bundle path"
        )
    trace_binary = _require_file(args.trace_binary, "trace binary")
    demo = _require_file(args.demo, "DART demo binary")
    runner = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")
    python = _require_file(args.python, "Python interpreter")
    source_identity_before = _source_identity(
        trace_binary=trace_binary,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    with _bundle_transaction(bundle) as commit:
        return _finalize_transaction(
            args,
            bundle=bundle,
            trace_binary=trace_binary,
            demo=demo,
            runner=runner,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
            source_identity_before=source_identity_before,
            commit=commit,
        )


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
    trace_binary = _require_file(trace_binary, "trace binary")
    demo = _require_file(demo, "DART demo binary")
    runner = _require_file(runner, "visual evidence runner")
    ffmpeg = _require_file(ffmpeg, "ffmpeg")
    ffprobe = _require_file(ffprobe, "ffprobe")
    python = _require_file(python, "Python interpreter")
    metadata = read_json(bundle / "metadata.json")
    if set(metadata) != EXPECTED_FINAL_METADATA_KEYS:
        raise ValueError("malformed incline finalized metadata payload")
    if any(
        metadata.get(key) != value for key, value in EXPECTED_METADATA_FLAGS.items()
    ):
        raise ValueError("incline finalized metadata claim flags changed")
    if (
        not isinstance(metadata.get("evidence_date"), str)
        or not metadata["evidence_date"]
    ):
        raise ValueError("incline evidence date is missing")
    if metadata.get("claim_scope") != CLAIM_SCOPE:
        raise ValueError("incline claim scope changed")
    if metadata.get("claim_boundary") != CLAIM_BOUNDARY:
        raise ValueError("incline claim boundary changed")

    parsed_traces = []
    for scenario in SCENARIOS:
        trace_path = bundle / "traces" / f"{scenario}.csv"
        parsed_traces.append(
            parse_trace_text(trace_path.read_text(encoding="utf-8"), scenario)
        )
    capture_provenance = read_json(bundle / "capture-provenance.json")
    if metadata.get("capture_provenance") != capture_provenance:
        raise ValueError("incline capture-provenance file binding is stale")
    _validate_capture_provenance(
        capture_provenance,
        bundle=bundle,
        trace_binary=trace_binary,
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
    trace_summary = _build_trace_summary(parsed_traces, capture_summary, bundle)
    if read_json(bundle / "trace-summary.json") != trace_summary:
        raise ValueError("incline trace-summary.json is stale")
    if metadata.get("trace_summary") != trace_summary:
        raise ValueError("incline metadata trace summary is stale")
    capture_metadata = dict(capture_summary)
    capture_metadata.pop("count_projection")
    if metadata.get("capture_summary") != capture_metadata:
        raise ValueError("incline metadata capture summary is stale")

    manual = validate_manual_inspection(bundle)
    if metadata.get("manual_inspection") != {
        "path": "manual-inspection.json",
        "sha256": sha256(bundle / "manual-inspection.json"),
        "pass": manual["pass"],
    }:
        raise ValueError("incline metadata manual-inspection binding is stale")

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
            raise ValueError(f"incline metadata {key} binding is stale")
    expected_report = _report_markdown(trace_summary, capture_summary)
    if (bundle / "REPORT.md").read_text(encoding="utf-8") != expected_report:
        raise ValueError("incline REPORT.md is stale")

    index = read_json(bundle / "artifact-index.json")
    validate_artifact_index(bundle, index)
    if metadata.get("artifact_index") != {
        "path": "artifact-index.json",
        "sha256": sha256(bundle / "artifact-index.json"),
        "artifact_count": index["artifact_count"],
        "excluded": index["excluded"],
    }:
        raise ValueError("incline metadata artifact-index binding is stale")
    sources = _source_identity(
        trace_binary=trace_binary,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    if metadata.get("source_identity") != sources:
        raise ValueError("incline source/binary identity changed")
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
        "aggregate_count_projection_sha256": trace_summary[
            "aggregate_count_projection_comparison"
        ]["trace_sha256"],
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

    for scenario in SCENARIOS:
        trace_process = _run_command(
            _trace_argv(trace_binary, scenario), timeout=args.trace_timeout
        )
        trace_path = bundle / "traces" / f"{scenario}.csv"
        stderr_path = bundle / "traces" / f"{scenario}.stderr.txt"
        if trace_process.stdout != trace_path.read_text(encoding="utf-8"):
            raise ValueError(
                f"live {scenario} trace replay differs from the finalized trace"
            )
        if trace_process.stderr != stderr_path.read_text(encoding="utf-8"):
            raise ValueError(f"live {scenario} trace stderr differs from finalized")

    return {
        **result,
        "live_trace_replays": {scenario: True for scenario in SCENARIOS},
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
            "persisted capture-time source/runtime provenance"
        ),
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help=(
            "recompute finalized durable capture contracts and replay both current "
            "traces without requiring pruned capture staging"
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
