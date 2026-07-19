#!/usr/bin/env python3
"""Finalize and revalidate the current-source Painleve proxy visual bundle.

This driver deliberately keeps the scientific claim narrow. It binds the
current DART captures to fresh exact-FBF trajectory traces and a recorded
manual inspection. It does not claim author-scene parity, external-solver
parity, an approved image golden, or real-time performance.
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
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "fig05_painleve_proxy_current_v1"
)
DEFAULT_TRACE_BINARY = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
)
DEFAULT_DEMO = ROOT / "build/default/cpp/Release/bin/dart-demos"
DEFAULT_RUNNER = ROOT / "scripts/run_fbf_visual_evidence.py"
DEFAULT_FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
DEFAULT_FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"

TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
FIXTURE_SOURCE = ROOT / "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp"
DEMO_SOURCE = ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
RUNNER_TEST = ROOT / "python/tests/unit/test_run_fbf_visual_evidence.py"
FINALIZER_TEST = ROOT / "python/tests/unit/test_finalize_fbf_painleve_visual.py"

SCHEMA_VERSION = "dart.fbf_painleve_proxy_visual_bundle/v1"
INDEX_SCHEMA_VERSION = "dart.fbf_painleve_proxy_artifact_index/v1"
MANUAL_SCHEMA_VERSION = "dart.fbf_painleve_proxy_manual_inspection/v1"
TRACE_SUMMARY_SCHEMA_VERSION = "dart.fbf_painleve_proxy_trace_summary/v1"
CAPTURE_PROVENANCE_SCHEMA_VERSION = "dart.fbf_painleve_proxy_capture_provenance/v1"

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
SCENARIOS = {
    "painleve_mu_0_5": {
        "capture_id": "painleve_mu05",
        "mu": 0.5,
        "expected_outcome": "slide_then_translationally_and_tilt_settled_upright",
    },
    "painleve_mu_0_55": {
        "capture_id": "painleve_mu055",
        "mu": 0.55,
        "expected_outcome": (
            "shorter_travel_to_tumble_then_translationally_and_tilt_settled"
        ),
    },
}
CAPTURE_IDS = tuple(spec["capture_id"] for spec in SCENARIOS.values())

TOTAL_STEPS = 150
DT_SECONDS = 1.0 / 60.0
FRAME_STEPS = tuple(range(0, TOTAL_STEPS + 1, 2))
PANEL_STEPS = (0, 30, 60, 90, 120, 150)
RESIDUAL_TOLERANCE = 1.0e-6
UPRIGHT_UP_Z_MIN = 0.85
TUMBLED_UP_Z_MAX = 0.55
HEIGHT_THRESHOLD = 0.35
HORIZONTAL_REST_SPEED_MAX = 1.0e-5
HORIZONTAL_REST_DRIFT_MAX = 1.0e-5
REST_WINDOW_START_STEP = 120
TAIL_HORIZONTAL_SPEED_MAX = 1.0e-4
TAIL_HORIZONTAL_EXCURSION_MAX = 1.0e-4
TAIL_VERTICAL_EXCURSION_MAX = 1.0e-3
TAIL_UP_Z_RANGE_MAX = 1.0e-4
INDEX_EXCLUSIONS = {"artifact-index.json", "metadata.json"}

CAPTURE_PATHS = {
    "capture-provenance.json",
    "manual-inspection.json",
    "run-summary.json",
    "groups/painleve/clip.mp4",
    "groups/painleve/metadata.json",
    "groups/painleve/panel.compose.json",
    "groups/painleve/panel.png",
    *(f"groups/painleve/panel_members/{capture_id}.png" for capture_id in CAPTURE_IDS),
}
for _capture_id in CAPTURE_IDS:
    CAPTURE_PATHS.update(
        {
            f"{_capture_id}/clip.mp4",
            f"{_capture_id}/metadata.json",
            f"{_capture_id}/panel.compose.json",
            f"{_capture_id}/panel.png",
            f"{_capture_id}/timeline.json",
        }
    )

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
    f"traces/{scenario}{suffix}"
    for scenario in SCENARIOS
    for suffix in (".csv", ".stderr.txt")
}
FINALIZER_OUTPUT_PATHS = TRACE_PATHS | {
    "REPORT.md",
    "artifact-index.json",
    "invocations.json",
    "metadata.json",
    "trace-summary.json",
    "verification.json",
}
EXPECTED_FINAL_PATHS = CAPTURE_PATHS | FINALIZER_OUTPUT_PATHS
ALLOWED_DIRECTORIES = {
    "groups",
    "groups/painleve",
    "groups/painleve/panel_members",
    "traces",
    *CAPTURE_IDS,
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
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
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
        raise ValueError(f"{label} is not a file: {path}")
    return path


def _require_bundle_root(path: Path, *, create: bool) -> Path:
    original = Path(path)
    absolute = Path(os.path.abspath(original))
    if original.is_symlink():
        raise ValueError(f"Painleve bundle root is a symlink: {original}")
    for ancestor in absolute.parents:
        if ancestor.is_symlink():
            raise ValueError(
                f"Painleve bundle root passes through a symlink: {original}"
            )
    if original.exists():
        if not original.is_dir():
            raise ValueError(f"Painleve bundle is not a regular directory: {original}")
    elif create:
        original.mkdir(parents=True)
    else:
        raise ValueError(f"Painleve bundle directory does not exist: {original}")
    if original.is_symlink():
        raise ValueError(f"Painleve bundle root became a symlink: {original}")
    resolved = original.resolve(strict=True)
    if resolved != absolute:
        raise ValueError(f"Painleve bundle root passes through a symlink: {original}")
    return resolved


def _bundle_entries(root: Path) -> tuple[set[str], set[str]]:
    root = _require_bundle_root(root, create=False)
    files: set[str] = set()
    directories: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise ValueError(f"Painleve bundle contains a symlink: {relative}")
        if path.is_dir():
            directories.add(relative)
        elif path.is_file():
            files.add(relative)
        else:
            raise ValueError(f"Painleve bundle has a non-regular entry: {relative}")
    return files, directories


def _validate_bundle_paths(
    root: Path, *, complete: bool, allow_staging: bool = False
) -> None:
    files, directories = _bundle_entries(root)
    allowed_files = EXPECTED_FINAL_PATHS | (STAGING_PATHS if allow_staging else set())
    allowed_directories = ALLOWED_DIRECTORIES | (
        STAGING_DIRECTORIES if allow_staging else set()
    )
    unexpected_files = files - allowed_files
    unexpected_directories = directories - allowed_directories
    if unexpected_files or unexpected_directories:
        raise ValueError(
            "Painleve bundle membership has unexpected entries: "
            f"files={sorted(unexpected_files)}, "
            f"directories={sorted(unexpected_directories)}"
        )
    if complete:
        missing = EXPECTED_FINAL_PATHS - files
        missing_directories = ALLOWED_DIRECTORIES - directories
        if missing or missing_directories:
            raise ValueError(
                "Painleve sealed bundle is incomplete: "
                f"files={sorted(missing)}, "
                f"directories={sorted(missing_directories)}"
            )


def _clean_finalizer_outputs(bundle: Path) -> None:
    for relative in FINALIZER_OUTPUT_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()


def _prune_capture_staging(bundle: Path) -> None:
    for relative in STAGING_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()
    for relative in STAGING_DIRECTORIES:
        path = bundle / relative
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
    files, directories = _bundle_entries(bundle)
    if files & STAGING_PATHS or directories & STAGING_DIRECTORIES:
        raise ValueError("Painleve ignored staging survived pruning")


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
def _bundle_transaction(bundle: Path):
    bundle = _require_bundle_root(bundle, create=False)
    with tempfile.TemporaryDirectory(
        prefix=f".{bundle.name}.backup-", dir=bundle.parent
    ) as temporary:
        snapshot = Path(temporary) / "bundle"
        shutil.copytree(bundle, snapshot)
        try:
            yield
        except BaseException:
            _remove_bundle_root(bundle)
            shutil.copytree(snapshot, bundle)
            raise


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


def parse_trace_text(text: str, expected_scenario: str) -> dict[str, Any]:
    if expected_scenario not in SCENARIOS:
        raise ValueError(f"unsupported Painleve trace scenario: {expected_scenario}")
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
                f"trace line {line}: scenario {raw['scenario']!r} != {expected_scenario!r}"
            )
        if raw["solver"] != "exact_fbf" or raw["body"] != "painleve_box_body":
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
                raise ValueError(f"{expected_scenario}: step 0 must be nan/not_run")
        else:
            residual = _float_field(raw, "residual", line=line)
            if typed["status"] != "success":
                raise ValueError(
                    f"{expected_scenario}: step {step} status {typed['status']!r}"
                )
            if residual > RESIDUAL_TOLERANCE:
                raise ValueError(
                    f"{expected_scenario}: step {step} residual {residual} exceeds "
                    f"{RESIDUAL_TOLERANCE}"
                )
            max_residual = max(max_residual, residual)
        typed["residual"] = residual
        rows.append(typed)

    if any(row["fallbacks"] != 0 for row in rows):
        raise ValueError(f"{expected_scenario}: boxed-LCP fallback observed")
    if rows[-1]["exact_solves"] <= 0:
        raise ValueError(f"{expected_scenario}: no exact-FBF solve was recorded")

    initial = rows[0]
    final = rows[-1]
    first_tumble = next(
        (
            row
            for row in rows[1:]
            if row["up_z"] < TUMBLED_UP_Z_MAX or row["z"] < HEIGHT_THRESHOLD
        ),
        None,
    )
    rest_start = rows[REST_WINDOW_START_STEP]
    tail = rows[REST_WINDOW_START_STEP:]
    final_horizontal_speed = math.hypot(final["vx"], final["vy"])
    horizontal_rest_drift = math.hypot(
        final["x"] - rest_start["x"], final["y"] - rest_start["y"]
    )
    max_tail_horizontal_speed = max(math.hypot(row["vx"], row["vy"]) for row in tail)
    tail_x_excursion = max(row["x"] for row in tail) - min(row["x"] for row in tail)
    tail_y_excursion = max(row["y"] for row in tail) - min(row["y"] for row in tail)
    tail_z_excursion = max(row["z"] for row in tail) - min(row["z"] for row in tail)
    tail_up_z_range = max(row["up_z"] for row in tail) - min(
        row["up_z"] for row in tail
    )
    if final_horizontal_speed > HORIZONTAL_REST_SPEED_MAX:
        raise ValueError(
            f"{expected_scenario}: final horizontal speed {final_horizontal_speed} "
            f"exceeds {HORIZONTAL_REST_SPEED_MAX}"
        )
    if horizontal_rest_drift > HORIZONTAL_REST_DRIFT_MAX:
        raise ValueError(
            f"{expected_scenario}: last-0.5s horizontal drift {horizontal_rest_drift} "
            f"exceeds {HORIZONTAL_REST_DRIFT_MAX}"
        )
    if max_tail_horizontal_speed > TAIL_HORIZONTAL_SPEED_MAX:
        raise ValueError(
            f"{expected_scenario}: tail horizontal speed {max_tail_horizontal_speed} "
            f"exceeds {TAIL_HORIZONTAL_SPEED_MAX}"
        )
    if max(tail_x_excursion, tail_y_excursion) > TAIL_HORIZONTAL_EXCURSION_MAX:
        raise ValueError(
            f"{expected_scenario}: tail horizontal excursion exceeds "
            f"{TAIL_HORIZONTAL_EXCURSION_MAX}"
        )
    if tail_z_excursion > TAIL_VERTICAL_EXCURSION_MAX:
        raise ValueError(
            f"{expected_scenario}: tail vertical excursion {tail_z_excursion} "
            f"exceeds {TAIL_VERTICAL_EXCURSION_MAX}"
        )
    if tail_up_z_range > TAIL_UP_Z_RANGE_MAX:
        raise ValueError(
            f"{expected_scenario}: tail up_z range {tail_up_z_range} "
            f"exceeds {TAIL_UP_Z_RANGE_MAX}"
        )

    if expected_scenario == "painleve_mu_0_5":
        if first_tumble is not None:
            raise ValueError("mu=.50 proxy crossed the tumble threshold")
        if not (final["up_z"] > UPRIGHT_UP_Z_MIN and final["z"] > HEIGHT_THRESHOLD):
            raise ValueError("mu=.50 proxy did not finish upright")
        if final["contacts"] <= 0 or final["x"] <= initial["x"]:
            raise ValueError(
                "mu=.50 proxy did not finish in contact after positive travel"
            )
    else:
        if first_tumble is None:
            raise ValueError("mu=.55 proxy never crossed the tumble threshold")
        if not (
            abs(final["up_z"]) <= 0.1
            and final["z"] < HEIGHT_THRESHOLD
            and final["contacts"] > 0
        ):
            raise ValueError("mu=.55 proxy did not finish horizontal and in contact")

    return {
        "scenario": expected_scenario,
        "capture_id": SCENARIOS[expected_scenario]["capture_id"],
        "mu": SCENARIOS[expected_scenario]["mu"],
        "expected_outcome": SCENARIOS[expected_scenario]["expected_outcome"],
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
            for key in ("step", "time", "x", "y", "z", "vx", "vy", "vz", "up_z")
        },
        "first_tumble": (
            None
            if first_tumble is None
            else {
                key: first_tumble[key]
                for key in ("step", "time", "x", "y", "z", "up_z")
            }
        ),
        "settled_proxy": {
            "window_start_step": REST_WINDOW_START_STEP,
            "window_duration_seconds": (TOTAL_STEPS - REST_WINDOW_START_STEP)
            * DT_SECONDS,
            "final_horizontal_speed": final_horizontal_speed,
            "window_horizontal_drift": horizontal_rest_drift,
            "max_window_horizontal_speed": max_tail_horizontal_speed,
            "window_x_excursion": tail_x_excursion,
            "window_y_excursion": tail_y_excursion,
            "window_z_excursion": tail_z_excursion,
            "window_up_z_range": tail_up_z_range,
            "speed_threshold": HORIZONTAL_REST_SPEED_MAX,
            "drift_threshold": HORIZONTAL_REST_DRIFT_MAX,
            "max_window_horizontal_speed_threshold": TAIL_HORIZONTAL_SPEED_MAX,
            "max_window_horizontal_excursion_threshold": TAIL_HORIZONTAL_EXCURSION_MAX,
            "max_window_vertical_excursion_threshold": TAIL_VERTICAL_EXCURSION_MAX,
            "max_window_up_z_range_threshold": TAIL_UP_Z_RANGE_MAX,
            "strict_rigid_body_rest_proven": False,
            "limitation": "The tracked CSV does not export angular velocity.",
            "pass": True,
        },
        "physical_outcome_valid": True,
    }


def summarize_trace_pair(summaries: Sequence[dict[str, Any]]) -> dict[str, Any]:
    by_scenario = {summary["scenario"]: summary for summary in summaries}
    if set(by_scenario) != set(SCENARIOS):
        raise ValueError("Painleve trace pair is incomplete")
    slide = by_scenario["painleve_mu_0_5"]
    tumble = by_scenario["painleve_mu_0_55"]
    first_tumble = tumble["first_tumble"]
    if first_tumble is None:
        raise ValueError("mu=.55 first-tumble record is missing")
    slide_rest_travel = slide["final"]["x"] - slide["initial"]["x"]
    tumble_threshold_travel = first_tumble["x"] - tumble["initial"]["x"]
    shorter_by = slide_rest_travel - tumble_threshold_travel
    if not shorter_by > 0.0:
        raise ValueError(
            "mu=.55 tumble-threshold travel is not shorter than mu=.50 rest travel"
        )
    return {
        "schema_version": TRACE_SUMMARY_SCHEMA_VERSION,
        "pass": True,
        "scenarios": [by_scenario[name] for name in SCENARIOS],
        "paired_contract": {
            "shorter_travel_definition": (
                "mu=.55 x travel at the first fixture tumble threshold "
                "(up_z < 0.55 or z < 0.35), compared with mu=.50 final rest travel"
            ),
            "mu05_rest_travel": slide_rest_travel,
            "mu055_first_tumble_travel": tumble_threshold_travel,
            "mu055_first_tumble_step": first_tumble["step"],
            "mu055_first_tumble_time_seconds": first_tumble["time"],
            "shorter_by": shorter_by,
            "pass": True,
        },
        "claim_boundary": (
            "Current-source DART proxy physical outcome only; the paper omits the "
            "box geometry, mass, launch state, and absolute timestamps."
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
    if payload.get("schema_version") != INDEX_SCHEMA_VERSION:
        raise ValueError("unexpected Painleve artifact-index schema")
    if payload.get("excluded") != sorted(INDEX_EXCLUSIONS):
        raise ValueError("Painleve artifact-index exclusions changed")
    artifacts = payload.get("artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("Painleve artifact-index artifacts must be a list")
    if payload.get("artifact_count") != len(artifacts):
        raise ValueError("Painleve artifact-index count mismatch")
    listed = []
    for item in artifacts:
        if not isinstance(item, dict) or set(item) != {"path", "bytes", "sha256"}:
            raise ValueError("malformed Painleve artifact-index entry")
        relative = item["path"]
        if not isinstance(relative, str) or relative.startswith(("/", "../")):
            raise ValueError(f"unsafe Painleve artifact-index path: {relative!r}")
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"Painleve artifact-index file missing: {relative}")
        if path.stat().st_size != item["bytes"]:
            raise ValueError(f"Painleve artifact-index byte mismatch: {relative}")
        if sha256(path) != item["sha256"]:
            raise ValueError(f"Painleve artifact-index hash mismatch: {relative}")
        listed.append(relative)
    if listed != sorted(set(listed)):
        raise ValueError("Painleve artifact-index paths are duplicate or unsorted")
    actual = {
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.relative_to(root).as_posix() not in INDEX_EXCLUSIONS
    }
    if actual != set(listed):
        raise ValueError(
            "Painleve artifact-index membership mismatch: "
            f"missing={sorted(actual - set(listed))}, "
            f"extra={sorted(set(listed) - actual)}"
        )


def validate_manual_inspection(root: Path) -> dict[str, Any]:
    path = root / "manual-inspection.json"
    record = read_json(path)
    if record.get("schema_version") != MANUAL_SCHEMA_VERSION:
        raise ValueError("unexpected Painleve manual-inspection schema")
    if record.get("pass") is not True or record.get("manual_inspected") is not True:
        raise ValueError("Painleve manual inspection is not passing")
    verdicts = record.get("verdicts")
    required_verdicts = {
        "paired_proxy_outcome_supported": True,
        "mu05_upright_return_visible": True,
        "mu055_tumble_and_visually_settled_horizontal_visible": True,
        "shorter_travel_requires_physical_trace": True,
        "paper_parity": False,
        "external_solver_parity": False,
        "approved_source_golden": False,
        "timing_verdict": None,
        "realtime_verdict": None,
    }
    if verdicts != required_verdicts:
        raise ValueError("Painleve manual-inspection verdict contract changed")
    expected_paths = {
        "painleve_mu05/panel.png",
        "painleve_mu055/panel.png",
        "groups/painleve/panel.png",
    }
    images = record.get("representative_images")
    if (
        not isinstance(images, list)
        or {item.get("path") for item in images if isinstance(item, dict)}
        != expected_paths
    ):
        raise ValueError("Painleve manual-inspection representative images changed")
    for item in images:
        relative = item.get("path")
        image_path = root / relative
        if not image_path.is_file() or sha256(image_path) != item.get("sha256"):
            raise ValueError(f"Painleve manual-inspection image changed: {relative}")
        if not isinstance(item.get("observation"), str) or not item["observation"]:
            raise ValueError(
                f"Painleve manual-inspection observation missing: {relative}"
            )
    return record


def _capture_result_by_id(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    results = payload.get("results")
    if not isinstance(results, list):
        raise ValueError("capture results must be a list")
    by_id = {}
    for result in results:
        schedule = result.get("schedule")
        schedule_id = (
            schedule.get("id") if isinstance(schedule, dict) else result.get("schedule")
        )
        if not isinstance(schedule_id, str) or schedule_id in by_id:
            raise ValueError("capture result identity is missing or duplicated")
        by_id[schedule_id] = result
    return by_id


def validate_capture_bundle(
    root: Path,
    *,
    demo: Path,
    verification: dict[str, Any] | None = None,
    require_staging: bool = False,
) -> dict[str, Any]:
    root = root.resolve()
    demo = _require_file(demo, "DART demo binary")
    verification = verification or read_json(root / "verification.json")
    if (
        verification.get("kind") != "verification"
        or verification.get("pass") is not True
    ):
        raise ValueError("Painleve capture verification is not passing")
    verify_results = _capture_result_by_id(verification)
    if set(verify_results) != {spec["capture_id"] for spec in SCENARIOS.values()}:
        raise ValueError("Painleve capture verification member set changed")

    run_summary = read_json(root / "run-summary.json")
    if (
        run_summary.get("kind") != "capture_run"
        or run_summary.get("pass") is not True
        or run_summary.get("failures") != []
        or run_summary.get("group_skips") != []
    ):
        raise ValueError("Painleve run-summary is not a complete passing capture")
    run_results = _capture_result_by_id(run_summary)
    if set(run_results) != set(verify_results):
        raise ValueError("Painleve run-summary member set changed")

    scenario_reports = []
    for trace_scenario, spec in SCENARIOS.items():
        capture_id = spec["capture_id"]
        metadata_path = root / capture_id / "metadata.json"
        metadata = read_json(metadata_path)
        if run_results[capture_id] != metadata:
            raise ValueError(
                f"{capture_id}: run-summary no longer equals metadata.json"
            )
        if (
            metadata.get("kind") != "capture_result"
            or metadata.get("pass") is not True
            or metadata.get("actual_simulator") is not True
            or metadata.get("generated_imagery") is not False
            or metadata.get("automated_semantic_outcome_validated") is not False
            or metadata.get("paper_comparable") is not False
        ):
            raise ValueError(f"{capture_id}: capture claim flags changed")
        schedule = metadata.get("schedule")
        if not isinstance(schedule, dict) or schedule.get("id") != capture_id:
            raise ValueError(f"{capture_id}: schedule identity changed")
        if (
            schedule.get("total_steps") != TOTAL_STEPS
            or schedule.get("time_step_seconds") != DT_SECONDS
        ):
            raise ValueError(f"{capture_id}: capture duration changed")
        if schedule.get("configuration", {}).get("mu") != str(spec["mu"]):
            raise ValueError(f"{capture_id}: friction coefficient changed")
        runtime = metadata.get("runtime")
        if (
            not isinstance(runtime, dict)
            or Path(runtime.get("demo_path", "")).resolve() != demo
            or runtime.get("demo_sha256") != sha256(demo)
            or runtime.get("demo_argv") != schedule.get("demo_argv")
        ):
            raise ValueError(f"{capture_id}: demo identity/argv changed")

        timeline = metadata.get("timeline_validation")
        expected_frame_steps = {str(step) for step in FRAME_STEPS}
        if (
            not isinstance(timeline, dict)
            or timeline.get("pass") is not True
            or timeline.get("step_count") != TOTAL_STEPS + 1
            or timeline.get("shot_count") != len(expected_frame_steps)
            or timeline.get("unique_frame_hashes") != len(expected_frame_steps)
            or set(timeline.get("steps", {}))
            != {str(step) for step in range(TOTAL_STEPS + 1)}
            or set(timeline.get("frames", {})) != expected_frame_steps
        ):
            raise ValueError(f"{capture_id}: timeline contract changed")
        sidecar = root / capture_id / "timeline.json"
        if timeline.get("sidecar") != str(sidecar):
            raise ValueError(f"{capture_id}: timeline sidecar binding changed")
        for step in FRAME_STEPS:
            frame_path = root / capture_id / "frames" / f"step_{step:06d}.png"
            frame = timeline["frames"].get(str(step))
            if (
                not isinstance(frame, dict)
                or frame.get("path") != str(frame_path)
                or not _is_sha256(frame.get("sha256"))
                or not _is_sha256(frame.get("world_viewport", {}).get("sha256"))
            ):
                raise ValueError(f"{capture_id}: frame binding changed at step {step}")
            if require_staging and (
                not frame_path.is_file()
                or frame_path.is_symlink()
                or sha256(frame_path) != frame["sha256"]
            ):
                raise ValueError(f"{capture_id}: staging frame changed at step {step}")
        diagnostics = timeline.get("final_solver_diagnostics")
        if (
            not isinstance(diagnostics, dict)
            or diagnostics.get("available") is not True
            or diagnostics.get("exact_attempts", 0) <= 0
            or diagnostics.get("exact_failures") != 0
            or diagnostics.get("boxed_lcp_fallbacks") != 0
            or diagnostics.get("status") != "success"
            or not math.isfinite(float(diagnostics.get("worst_residual", math.nan)))
            or float(diagnostics["worst_residual"]) > RESIDUAL_TOLERANCE
        ):
            raise ValueError(f"{capture_id}: solver diagnostics changed")

        media = metadata.get("media_validation")
        if not isinstance(media, list) or len(media) != 1:
            raise ValueError(f"{capture_id}: expected one MP4 validation")
        clip = media[0]
        clip_path = root / capture_id / "clip.mp4"
        stream = clip.get("stream_contract", {})
        if (
            clip.get("kind") != "mp4"
            or clip.get("path") != str(clip_path)
            or clip.get("full_decode") != "pass"
            or clip.get("sha256") != sha256(clip_path)
            or stream
            != {
                "width": 660,
                "height": 506,
                "frame_rate": "30/1",
                "frame_rate_rational": "30/1",
                "frame_count": 76,
            }
        ):
            raise ValueError(f"{capture_id}: MP4 contract changed")
        panel = metadata.get("panel_validation")
        panel_path = root / capture_id / "panel.png"
        compose_path = root / capture_id / "panel.compose.json"
        contrast = panel.get("verdict", {}).get("checks", {}).get("contrast", {})
        panel_sources = panel.get("source_frames") if isinstance(panel, dict) else None
        expected_panel_source_paths = [
            str(root / capture_id / "panel_frames" / f"step_{step:06d}.png")
            for step in PANEL_STEPS
        ]
        if (
            not isinstance(panel_sources, list)
            or [item.get("path") for item in panel_sources]
            != expected_panel_source_paths
            or any(not _is_sha256(item.get("sha256")) for item in panel_sources)
        ):
            raise ValueError(f"{capture_id}: panel source-frame contract changed")
        if require_staging:
            for source in panel_sources:
                source_path = Path(source["path"])
                if (
                    not source_path.is_file()
                    or source_path.is_symlink()
                    or sha256(source_path) != source["sha256"]
                ):
                    raise ValueError(f"{capture_id}: panel staging source changed")
        if (
            not isinstance(panel, dict)
            or panel.get("sha256") != sha256(panel_path)
            or panel.get("compose_manifest_path") != str(compose_path)
            or panel.get("compose_manifest_sha256") != sha256(compose_path)
            or panel.get("compose_manifest") != read_json(compose_path)
            or panel.get("verdict", {}).get("pass") is not True
            or contrast.get("pass") is not False
            or panel.get("verdict", {})
            .get("thresholds_used", {})
            .get("require_contrast")
            is not False
        ):
            raise ValueError(f"{capture_id}: panel/contrast disclosure changed")
        verified = verify_results[capture_id]
        verified_media = verified.get("media")
        if (
            verified.get("pass") is not True
            or verified.get("metadata_sha256") != sha256(metadata_path)
            or verified.get("timeline") != timeline
            or verified.get("panel", {}).get("image", {}).get("path") != str(panel_path)
            or verified.get("panel", {}).get("pass") is not True
            or not isinstance(verified_media, list)
            or len(verified_media) != 1
            or any(
                verified_media[0].get(key) != clip.get(key)
                for key in ("kind", "path", "sha256", "full_decode", "stream_contract")
            )
        ):
            raise ValueError(f"{capture_id}: verification metadata binding changed")
        scenario_reports.append(
            {
                "trace_scenario": trace_scenario,
                "capture_id": capture_id,
                "metadata_sha256": sha256(metadata_path),
                "timeline_sha256": sha256(root / capture_id / "timeline.json"),
                "panel_sha256": sha256(panel_path),
                "clip_sha256": sha256(clip_path),
                "captured_frames": 76,
                "unique_frames": 76,
                "decoded_video_frames": 76,
                "exact_attempts": diagnostics["exact_attempts"],
                "exact_failures": diagnostics["exact_failures"],
                "boxed_lcp_fallbacks": diagnostics["boxed_lcp_fallbacks"],
                "worst_residual": diagnostics["worst_residual"],
                "contrast_heuristic_pass": False,
                "contrast_required": False,
                "pass": True,
            }
        )

    groups = run_summary.get("group_outputs")
    if not isinstance(groups, list) or len(groups) != 1:
        raise ValueError("Painleve run-summary group output changed")
    group_path = root / "groups/painleve/metadata.json"
    group = read_json(group_path)
    if groups[0] != group:
        raise ValueError("Painleve run-summary group no longer equals metadata.json")
    if (
        group.get("group_id") != "painleve"
        or group.get("pass") is not True
        or group.get("member_order") != ["painleve_mu05", "painleve_mu055"]
        or group.get("labels") != ["MU .5 SLIDE REST", "MU .55 SHORT TRAVEL TUMBLE"]
        or group.get("actual_simulator") is not True
        or group.get("generated_imagery") is not False
        or group.get("automated_semantic_outcome_validated") is not False
        or group.get("paper_comparable") is not False
    ):
        raise ValueError("Painleve group identity/claim flags changed")
    members = group.get("members")
    if not isinstance(members, list) or [
        member.get("id") for member in members
    ] != list(CAPTURE_IDS):
        raise ValueError("Painleve group member order changed")
    expected_group_sources = []
    for capture_id, member in zip(CAPTURE_IDS, members):
        member_root = root / capture_id
        member_metadata_path = member_root / "metadata.json"
        member_timeline_path = member_root / "timeline.json"
        member_clip_path = member_root / "clip.mp4"
        member_metadata = read_json(member_metadata_path)
        timeline = member_metadata.get("timeline_validation")
        frame = (
            timeline.get("frames", {}).get(str(TOTAL_STEPS))
            if isinstance(timeline, dict)
            else None
        )
        expected_frame_path = member_root / "frames" / f"step_{TOTAL_STEPS:06d}.png"
        if (
            not isinstance(frame, dict)
            or frame.get("path") != str(expected_frame_path)
            or not _is_sha256(frame.get("sha256"))
        ):
            raise ValueError(f"{capture_id}: group source frame binding changed")
        if require_staging and (
            not expected_frame_path.is_file()
            or expected_frame_path.is_symlink()
            or sha256(expected_frame_path) != frame["sha256"]
        ):
            raise ValueError(f"{capture_id}: group source staging frame changed")
        expected_member = {
            "metadata_path": str(member_metadata_path),
            "metadata_sha256": sha256(member_metadata_path),
            "timeline_path": str(member_timeline_path),
            "timeline_sha256": sha256(member_timeline_path),
            "panel_source_frame": str(expected_frame_path),
            "panel_source_frame_sha256": frame["sha256"],
            "clip_path": str(member_clip_path),
            "clip_sha256": sha256(member_clip_path),
        }
        if any(member.get(key) != value for key, value in expected_member.items()):
            raise ValueError(f"{capture_id}: Painleve group member binding changed")
        expected_group_sources.append(
            {"path": str(expected_frame_path), "sha256": frame["sha256"]}
        )
    group_panel = root / "groups/painleve/panel.png"
    group_clip = root / "groups/painleve/clip.mp4"
    group_compose = root / "groups/painleve/panel.compose.json"
    group_panel_validation = group.get("panel_validation", {})
    group_rows = group_panel_validation.get("row_compositions")
    if (
        group_panel_validation.get("sha256") != sha256(group_panel)
        or group_panel_validation.get("source_frames") != expected_group_sources
        or not isinstance(group_panel_validation.get("cropped_tiles"), list)
        or [tile.get("path") for tile in group_panel_validation["cropped_tiles"]]
        != [
            str(root / "groups/painleve/panel_members" / f"{capture_id}.png")
            for capture_id in CAPTURE_IDS
        ]
        or any(
            not Path(tile["path"]).is_file()
            or Path(tile["path"]).is_symlink()
            or tile.get("sha256") != sha256(Path(tile["path"]))
            for tile in group_panel_validation["cropped_tiles"]
        )
        or not isinstance(group_rows, list)
        or len(group_rows) != 1
        or group_rows[0].get("manifest_path") != str(group_compose)
        or group_rows[0].get("manifest_sha256") != sha256(group_compose)
        or group_rows[0].get("manifest") != read_json(group_compose)
        or group.get("media_validation", {}).get("sha256") != sha256(group_clip)
        or group.get("media_validation", {}).get("full_decode") != "pass"
        or group.get("synchronization", {}).get("frame_count") != 76
        or group.get("synchronization", {}).get("frame_rate") != "30/1"
        or group.get("synchronization", {}).get("total_steps") != TOTAL_STEPS
    ):
        raise ValueError("Painleve group panel/media contract changed")
    verify_groups = verification.get("group_outputs")
    if not isinstance(verify_groups, list) or len(verify_groups) != 1:
        raise ValueError("Painleve verification group output changed")
    verified_group = verify_groups[0]
    if (
        verified_group.get("group_id") != "painleve"
        or verified_group.get("pass") is not True
        or verified_group.get("metadata_sha256") != sha256(group_path)
        or verified_group.get("panel", {}).get("sha256") != sha256(group_panel)
        or verified_group.get("media", {}).get("sha256") != sha256(group_clip)
    ):
        raise ValueError("Painleve verification group binding changed")

    return {
        "pass": True,
        "scenarios": scenario_reports,
        "group": {
            "metadata_sha256": sha256(group_path),
            "panel_sha256": sha256(group_panel),
            "clip_sha256": sha256(group_clip),
            "width": 1320,
            "height": 530,
            "frame_rate": "30/1",
            "frame_count": 76,
            "full_decode": "pass",
        },
        "run_summary_sha256": sha256(root / "run-summary.json"),
        "verification_sha256": sha256(root / "verification.json"),
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "paper_comparable": False,
    }


def _run_command(
    argv: Sequence[str], *, timeout: float
) -> subprocess.CompletedProcess[str]:
    process = subprocess.run(
        list(argv),
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


def _trace_argv(trace_binary: Path, scenario: str) -> list[str]:
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
    runner: Path,
    demo: Path,
    bundle: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> list[str]:
    return [
        sys.executable,
        str(runner),
        "verify",
        "--scenario",
        "painleve_mu05",
        "--scenario",
        "painleve_mu055",
        "--demo",
        str(demo),
        "--output-root",
        str(bundle),
        "--ffmpeg",
        str(ffmpeg),
        "--ffprobe",
        str(ffprobe),
    ]


def _report_markdown(
    trace_summary: dict[str, Any], capture_summary: dict[str, Any]
) -> str:
    slide, tumble = trace_summary["scenarios"]
    pair = trace_summary["paired_contract"]
    group = capture_summary["group"]
    return f"""# Figure 5 Painleve proxy current-source evidence

Status: valid current-source DART proxy evidence; not paper parity.

The paired 2.5-second exact-FBF captures and fresh 151-row traces independently
support the qualitative proxy transition. At `mu=.50`, the box remains upright
and is translationally and tilt-settled over the final 0.5-second trace window
at x={slide['final']['x']:.17g} m. At `mu=.55`, the first
fixture-defined tumble threshold occurs at step {pair['mu055_first_tumble_step']}
(t={pair['mu055_first_tumble_time_seconds']:.17g} s,
x={pair['mu055_first_tumble_travel']:.17g} m),
{pair['shorter_by']:.17g} m before the `mu=.50` rest distance. The `mu=.55`
box is visually horizontal and passes the same translational/tilt-settled tail
gates; its later final x position is not the definition of the shorter-travel
claim. Angular velocity is not exported, so strict rigid-body rest is not
established.

Both traces complete {TOTAL_STEPS} steps without boxed-LCP fallback and keep
every solved-step residual at or below {RESIDUAL_TOLERANCE:.1e}. Both capture
streams contain 76 distinct frames and decode fully. The synchronized group
clip is {group['width']}x{group['height']} at {group['frame_rate']} with
{group['frame_count']} frames.

The panels use a deliberately dark palette. Their generic contrast heuristic
is false and contrast is not a required automated gate; nonblank, motion,
hash, composition, stream, full-decode, physical-trace, and recorded manual
inspection gates are the evidence used here.

Claim boundary: this is DART-side proxy evidence. The rendered demo and tracked
trace are separate scene implementations and are not trace-equivalent; the
trace corroborates the classifier but does not automate the visual semantic
verdict. The paper does not publish the box dimensions, mass, launch state, or
absolute timestamps. No faithful external-solver panels, approved source
golden/diff, paper timing, or real-time performance verdict is supplied.
"""


def _source_identity(*, trace_binary: Path, demo: Path, runner: Path) -> dict[str, Any]:
    sources = {
        "finalizer": Path(__file__).resolve(),
        "finalizer_test": FINALIZER_TEST,
        "visual_runner": runner,
        "visual_runner_test": RUNNER_TEST,
        "trace_source": TRACE_SOURCE,
        "fixture_source": FIXTURE_SOURCE,
        "demo_source": DEMO_SOURCE,
        "trace_binary": trace_binary,
        "demo_binary": demo,
    }
    return {
        name: {"path": str(path.resolve()), "sha256": sha256(_require_file(path, name))}
        for name, path in sources.items()
    }


def _durable_capture_records(bundle: Path) -> dict[str, Any]:
    def bound_sha(relative: str) -> str:
        return sha256(_require_file(bundle / relative, relative))

    members = []
    for capture_id in CAPTURE_IDS:
        members.append(
            {
                "capture_id": capture_id,
                "metadata_sha256": bound_sha(f"{capture_id}/metadata.json"),
                "timeline_sha256": bound_sha(f"{capture_id}/timeline.json"),
                "panel_sha256": bound_sha(f"{capture_id}/panel.png"),
                "clip_sha256": bound_sha(f"{capture_id}/clip.mp4"),
            }
        )
    return {
        "members": members,
        "group": {
            "metadata_sha256": bound_sha("groups/painleve/metadata.json"),
            "panel_sha256": bound_sha("groups/painleve/panel.png"),
            "clip_sha256": bound_sha("groups/painleve/clip.mp4"),
        },
    }


def _verification_resource_identity(
    *, runner: Path, demo: Path, ffmpeg: Path, ffprobe: Path
) -> dict[str, dict[str, str]]:
    return {
        name: {
            "path": str(path.resolve()),
            "sha256": sha256(_require_file(path, name)),
        }
        for name, path in {
            "visual_runner": runner,
            "demo_binary": demo,
            "ffmpeg_binary": ffmpeg,
            "ffprobe_binary": ffprobe,
        }.items()
    }


def _validate_capture_provenance(
    payload: Any,
    *,
    bundle: Path,
    runner: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    verification = read_json(bundle / "verification.json")
    expected = {
        "schema_version": CAPTURE_PROVENANCE_SCHEMA_VERSION,
        "run_summary": {
            "path": "run-summary.json",
            "sha256": sha256(bundle / "run-summary.json"),
        },
        "verification": {
            "argv": _verification_argv(runner, demo, bundle, ffmpeg, ffprobe),
            "returncode": 0,
            "path": "verification.json",
            "sha256": sha256(bundle / "verification.json"),
            "payload_sha256": _payload_sha256(verification),
        },
        "verification_resources": _verification_resource_identity(
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        ),
        "durable_capture_records": _durable_capture_records(bundle),
        "staging_pruned": True,
    }
    if not isinstance(payload, dict) or any(
        payload.get(key) != value for key, value in expected.items()
    ):
        raise ValueError("Painleve capture provenance changed")
    if (
        set(payload)
        != {
            *expected,
            "verification_stdout_sha256",
            "verification_stderr_sha256",
        }
        or not _is_sha256(payload.get("verification_stdout_sha256"))
        or not _is_sha256(payload.get("verification_stderr_sha256"))
    ):
        raise ValueError("Painleve capture provenance hashes are malformed")
    return payload


def _validate_invocations(
    bundle: Path,
    *,
    trace_binary: Path,
    runner: Path,
    demo: Path,
    ffmpeg: Path,
    ffprobe: Path,
) -> dict[str, Any]:
    provenance = read_json(bundle / "capture-provenance.json")
    traces = []
    for scenario in SCENARIOS:
        stdout_path = bundle / "traces" / f"{scenario}.csv"
        stderr_path = bundle / "traces" / f"{scenario}.stderr.txt"
        traces.append(
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
    expected = {
        "schema_version": "dart.fbf_painleve_proxy_invocations/v1",
        "trace": traces,
        "capture_verification": {
            "argv": _verification_argv(runner, demo, bundle, ffmpeg, ffprobe),
            "returncode": 0,
            "stdout_path": "verification.json",
            "stdout_sha256": sha256(bundle / "verification.json"),
            "raw_stdout_sha256": provenance["verification_stdout_sha256"],
            "stderr_sha256": provenance["verification_stderr_sha256"],
            "stderr_retained": False,
        },
    }
    payload = read_json(bundle / "invocations.json")
    if payload != expected:
        raise ValueError("Painleve invocation bindings changed")
    return payload


def _finalize_in_place(args: argparse.Namespace) -> dict[str, Any]:
    bundle = _require_bundle_root(args.bundle, create=False)
    _validate_bundle_paths(bundle, complete=False, allow_staging=True)
    _clean_finalizer_outputs(bundle)
    _validate_bundle_paths(bundle, complete=False, allow_staging=True)
    trace_binary = _require_file(args.trace_binary, "trace binary")
    demo = _require_file(args.demo, "DART demo binary")
    runner = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")

    trace_summaries = []
    trace_invocations = []
    for scenario in SCENARIOS:
        argv = _trace_argv(trace_binary, scenario)
        process = _run_command(argv, timeout=args.trace_timeout)
        stdout_path = bundle / "traces" / f"{scenario}.csv"
        stderr_path = bundle / "traces" / f"{scenario}.stderr.txt"
        stdout_path.parent.mkdir(parents=True, exist_ok=True)
        stdout_path.write_text(process.stdout, encoding="utf-8")
        stderr_path.write_text(process.stderr, encoding="utf-8")
        trace_summaries.append(parse_trace_text(process.stdout, scenario))
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
    trace_summary = summarize_trace_pair(trace_summaries)
    write_json(bundle / "trace-summary.json", trace_summary)

    verify_argv = _verification_argv(runner, demo, bundle, ffmpeg, ffprobe)
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
        verification=verification,
        require_staging=True,
    )
    manual = validate_manual_inspection(bundle)
    provenance = {
        "schema_version": CAPTURE_PROVENANCE_SCHEMA_VERSION,
        "run_summary": {
            "path": "run-summary.json",
            "sha256": sha256(bundle / "run-summary.json"),
        },
        "verification": {
            "argv": verify_argv,
            "returncode": verify_process.returncode,
            "path": "verification.json",
            "sha256": sha256(bundle / "verification.json"),
            "payload_sha256": _payload_sha256(verification),
        },
        "verification_resources": _verification_resource_identity(
            runner=runner,
            demo=demo,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
        ),
        "verification_stdout_sha256": hashlib.sha256(
            verify_process.stdout.encode("utf-8")
        ).hexdigest(),
        "verification_stderr_sha256": sha256(bundle / "verification.stderr.txt"),
        "durable_capture_records": _durable_capture_records(bundle),
        "staging_pruned": True,
    }
    write_json(bundle / "capture-provenance.json", provenance)
    invocations = {
        "schema_version": "dart.fbf_painleve_proxy_invocations/v1",
        "trace": trace_invocations,
        "capture_verification": {
            "argv": verify_argv,
            "returncode": verify_process.returncode,
            "stdout_path": "verification.json",
            "stdout_sha256": sha256(bundle / "verification.json"),
            "raw_stdout_sha256": provenance["verification_stdout_sha256"],
            "stderr_sha256": provenance["verification_stderr_sha256"],
            "stderr_retained": False,
        },
    }
    write_json(bundle / "invocations.json", invocations)
    (bundle / "REPORT.md").write_text(
        _report_markdown(trace_summary, capture_summary), encoding="utf-8"
    )

    _prune_capture_staging(bundle)
    _validate_capture_provenance(
        provenance,
        bundle=bundle,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    sealed_capture_summary = validate_capture_bundle(
        bundle,
        demo=demo,
        verification=verification,
    )
    if sealed_capture_summary != capture_summary:
        raise ValueError("Painleve durable capture summary changed after pruning")
    _validate_bundle_paths(bundle, complete=False)

    files_before_index, _ = _bundle_entries(bundle)
    expected_before_index = EXPECTED_FINAL_PATHS - INDEX_EXCLUSIONS
    if files_before_index != expected_before_index:
        raise ValueError(
            "Painleve bundle is not ready for indexing: "
            f"missing={sorted(expected_before_index - files_before_index)}, "
            f"extra={sorted(files_before_index - expected_before_index)}"
        )

    index = artifact_index(bundle)
    write_json(bundle / "artifact-index.json", index)
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "status": "valid_current_source_nonpaper_proxy",
        "pass": True,
        "evidence_date": args.evidence_date,
        "paper_parity": False,
        "actual_simulator": True,
        "generated_imagery": False,
        "automated_semantic_outcome_validated": False,
        "manual_visual_outcome_validated": True,
        "trace_equivalence_to_rendered_demo": False,
        "claim_scope": (
            "Current-source DART Painleve-style proxy: manual visual upright versus "
            "horizontal classification, independently corroborated by a separate "
            "tracked proxy trace and its shorter pre-tumble travel threshold."
        ),
        "trace_summary": trace_summary,
        "capture_summary": capture_summary,
        "capture_provenance": {
            "path": "capture-provenance.json",
            "sha256": sha256(bundle / "capture-provenance.json"),
            "staging_pruned": True,
        },
        "manual_inspection": {
            "path": "manual-inspection.json",
            "sha256": sha256(bundle / "manual-inspection.json"),
            "pass": manual["pass"],
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
            trace_binary=trace_binary, demo=demo, runner=runner
        ),
        "claim_boundary": (
            "The rendered demo and tracked trace are not trace-equivalent. This is "
            "not author-scene parity, faithful external-solver parity, an approved "
            "source golden/diff, strict rigid-body rest, paper timing, or real-time "
            "evidence."
        ),
    }
    write_json(bundle / "metadata.json", metadata)
    return verify_finalized(
        bundle,
        trace_binary=trace_binary,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )


def finalize(args: argparse.Namespace) -> dict[str, Any]:
    bundle = _require_bundle_root(args.bundle, create=False)
    _validate_bundle_paths(bundle, complete=False, allow_staging=True)
    provenance_path = bundle / "capture-provenance.json"
    if provenance_path.is_file():
        persisted = read_json(provenance_path)
        has_staging_frames = any(
            (bundle / capture_id / "frames").is_dir() for capture_id in CAPTURE_IDS
        )
        if persisted.get("staging_pruned") is True and not has_staging_frames:
            raise ValueError(
                "sealed Painleve evidence cannot be re-finalized without a fresh "
                "capture; use --verify-only"
            )
    with _bundle_transaction(bundle):
        return _finalize_in_place(args)


def verify_finalized(
    bundle: Path,
    *,
    trace_binary: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path = DEFAULT_FFMPEG,
    ffprobe: Path = DEFAULT_FFPROBE,
) -> dict[str, Any]:
    bundle = _require_bundle_root(bundle, create=False)
    _validate_bundle_paths(bundle, complete=True)
    trace_binary = _require_file(trace_binary, "trace binary")
    demo = _require_file(demo, "DART demo binary")
    runner = _require_file(runner, "visual evidence runner")
    ffmpeg = _require_file(ffmpeg, "ffmpeg")
    ffprobe = _require_file(ffprobe, "ffprobe")
    metadata = read_json(bundle / "metadata.json")
    if (
        metadata.get("schema_version") != SCHEMA_VERSION
        or metadata.get("status") != "valid_current_source_nonpaper_proxy"
        or metadata.get("pass") is not True
        or metadata.get("paper_parity") is not False
        or metadata.get("actual_simulator") is not True
        or metadata.get("generated_imagery") is not False
        or metadata.get("automated_semantic_outcome_validated") is not False
        or metadata.get("manual_visual_outcome_validated") is not True
        or metadata.get("trace_equivalence_to_rendered_demo") is not False
    ):
        raise ValueError("Painleve finalized metadata claim flags changed")

    summaries = []
    for scenario in SCENARIOS:
        trace_path = bundle / "traces" / f"{scenario}.csv"
        summaries.append(
            parse_trace_text(trace_path.read_text(encoding="utf-8"), scenario)
        )
    trace_summary = summarize_trace_pair(summaries)
    if metadata.get("trace_summary") != trace_summary:
        raise ValueError("Painleve metadata trace summary is stale")
    if read_json(bundle / "trace-summary.json") != trace_summary:
        raise ValueError("Painleve trace-summary.json is stale")

    capture_summary = validate_capture_bundle(bundle, demo=demo)
    if metadata.get("capture_summary") != capture_summary:
        raise ValueError("Painleve metadata capture summary is stale")
    provenance = read_json(bundle / "capture-provenance.json")
    _validate_capture_provenance(
        provenance,
        bundle=bundle,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    if metadata.get("capture_provenance") != {
        "path": "capture-provenance.json",
        "sha256": sha256(bundle / "capture-provenance.json"),
        "staging_pruned": True,
    }:
        raise ValueError("Painleve metadata capture provenance is stale")
    manual = validate_manual_inspection(bundle)
    if metadata.get("manual_inspection") != {
        "path": "manual-inspection.json",
        "sha256": sha256(bundle / "manual-inspection.json"),
        "pass": manual["pass"],
    }:
        raise ValueError("Painleve metadata manual-inspection binding is stale")

    index = read_json(bundle / "artifact-index.json")
    validate_artifact_index(bundle, index)
    if metadata.get("artifact_index") != {
        "path": "artifact-index.json",
        "sha256": sha256(bundle / "artifact-index.json"),
        "artifact_count": index["artifact_count"],
        "excluded": index["excluded"],
    }:
        raise ValueError("Painleve metadata artifact-index binding is stale")
    _validate_invocations(
        bundle,
        trace_binary=trace_binary,
        runner=runner,
        demo=demo,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
    )
    for key, relative in (("invocations", "invocations.json"), ("report", "REPORT.md")):
        stored = metadata.get(key)
        if stored != {"path": relative, "sha256": sha256(bundle / relative)}:
            raise ValueError(f"Painleve metadata {key} binding is stale")

    sources = _source_identity(
        trace_binary=trace_binary,
        demo=demo,
        runner=runner,
    )
    if metadata.get("source_identity") != sources:
        raise ValueError("Painleve source/binary identity changed")
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
    parser.add_argument("--trace-timeout", type=float, default=120.0)
    parser.add_argument("--verification-timeout", type=float, default=600.0)
    parser.add_argument("--evidence-date", default="2026-07-12")
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help=(
            "recompute every sealed durable contract without requiring ignored "
            "capture staging"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.verify_only:
            result = verify_finalized(
                args.bundle,
                trace_binary=args.trace_binary,
                demo=args.demo,
                runner=args.runner,
                ffmpeg=args.ffmpeg,
                ffprobe=args.ffprobe,
            )
        else:
            result = finalize(args)
    except (OSError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
