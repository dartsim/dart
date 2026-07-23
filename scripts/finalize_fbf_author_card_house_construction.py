#!/usr/bin/env python3
"""Finalize the construction-only author card-house capture bundle.

This lane validates one DART step-zero render of the source-configuration
port. It intentionally executes no dynamics and cannot establish a trajectory,
solver, physical-outcome, Fig. 6, video, or paper-timing claim.
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import shlex
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
from collections.abc import Callable, Iterator, Sequence
from pathlib import Path, PurePosixPath
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from fbf_scene_provenance import (  # noqa: E402
    SEMANTIC_PROVENANCE_SCHEMA_VERSION,
    build_semantic_physics_provenance,
)
from image_verdict import build_verdict  # noqa: E402

DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "card_house_author_5_construction_current_v1"
)
DEFAULT_DEMO = ROOT / "build/default/cpp/Release/bin/dart-demos"
DEFAULT_RUNNER = ROOT / "scripts/run_fbf_visual_evidence.py"
DEFAULT_FFMPEG = ROOT / ".pixi/envs/gazebo/bin/ffmpeg"
DEFAULT_FFPROBE = ROOT / ".pixi/envs/gazebo/bin/ffprobe"
DEFAULT_PYTHON = Path(sys.executable)
BUILD_RUNTIME_ROOT = ROOT / "build/default/cpp/Release"

DEMO_CMAKE = ROOT / "examples/demos/CMakeLists.txt"
DEMO_MAIN = ROOT / "examples/demos/main.cpp"
DEMO_SOURCE = ROOT / "examples/demos/scenes/FbfPaperFrictionScene.cpp"
CONFIG_SPEC = ROOT / "examples/demos/scenes/FbfAuthorCardHouseSpec.hpp"
DEMO_HOST_SOURCE = ROOT / "examples/demos/DemoHost.cpp"
DEMO_HOST_HEADER = ROOT / "examples/demos/DemoHost.hpp"
REGISTRY_SOURCE = ROOT / "examples/demos/Registry.cpp"
SCENES_HEADER = ROOT / "examples/demos/scenes/Scenes.hpp"
RUNNER_TEST = ROOT / "python/tests/unit/test_run_fbf_visual_evidence.py"
FIXTURE_CMAKE = ROOT / "tests/integration/CMakeLists.txt"
FIXTURE_TEST = ROOT / "tests/integration/test_ExactCoulombFbfPaperFixtures.cpp"
FINALIZER_TEST = (
    ROOT / "python/tests/unit/test_finalize_fbf_author_card_house_construction.py"
)
IMAGE_COMPOSE = ROOT / "scripts/image_compose.py"
IMAGE_TOOLS = ROOT / "scripts/_image_tools.py"
IMAGE_VERDICT = ROOT / "scripts/image_verdict.py"

SCHEMA_VERSION = "dart.fbf_author_card_house_construction_bundle/v1"
INDEX_SCHEMA_VERSION = "dart.fbf_author_card_house_construction_artifact_index/v1"
MANUAL_SCHEMA_VERSION = "dart.fbf_author_card_house_construction_manual_inspection/v1"
INVOCATIONS_SCHEMA_VERSION = "dart.fbf_author_card_house_construction_invocations/v1"
CONTRACT_SCHEMA_VERSION = "dart.fbf_author_card_house_configuration_contract/v1"
RUNNER_SCHEMA_VERSION = "dart.fbf_visual_evidence/v1"
RUNNER_CAPTURE_RESULT_SCHEMA_VERSION = "dart.fbf_visual_evidence/v2"
RUNNER_CAPTURE_RESULT_SCHEMA_VERSIONS = (
    RUNNER_SCHEMA_VERSION,
    RUNNER_CAPTURE_RESULT_SCHEMA_VERSION,
)

SCHEDULE_ID = "card_house_author_5_construction"
SCENE_ID = "fbf_author_card_house_5_construction"
CAPTURE_DIR = SCHEDULE_ID
EXPECTED_CONFIGURATION = {
    "levels": "5",
    "mode": "static source-configuration port",
    "simulation_time_step_seconds": "1/240",
    "display_time_step_seconds": "1/60",
    "substeps_per_display_frame": "4",
    "release_substep": "1600",
    "total_substeps": "3200",
    "evidence_scope": "configuration-only",
    "parity_scope": "non-parity",
}
EXPECTED_PANEL_LABEL = "configuration-only substep 0"
EXPECTED_MISMATCHES = [
    (
        "DART renders its own float64 scene reconstructions under DART "
        "collision/scene contracts, not outputs from the authors' public "
        "Warp/Newton reference implementation or historical paper rendering setup."
    ),
    (
        "The step-zero capture validates only the source-configuration port; "
        "it supplies no release, standing, dynamics, outcome, performance, or "
        "paper-parity evidence."
    ),
    (
        "DART Native collision and the configured DART solver are not the "
        "authors' Warp/Newton collision backend or solver implementation."
    ),
    (
        "The paper video does not show the five-level GPU benchmark, and no "
        "approved source render golden is available."
    ),
]
EXPECTED_SEMANTIC_OUTCOME_GATE = (
    "manual inspection plus the separate physical trace/test contract is "
    "required; image motion/nonblank checks do not prove the expected outcome"
)
EXPECTED_AUTHOR_REPOSITORY = "https://github.com/matthcsong/fbf-sca-2026"
EXPECTED_AUTHOR_COMMIT = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
EXPECTED_AUTHOR_TREE = "ffcdafb61adeda2239c8366d054b548b50d26685"
EXPECTED_CLAIM_BOUNDARY = {
    "construction_only": True,
    "source_geometry_and_initial_pose_port": True,
    "dart_cards_mobile": True,
    "dart_cubes_initially_immobile": True,
    "dynamics_executed": False,
    "configuration_port_valid": True,
    "trajectory_valid": False,
    "solver_valid": False,
    "physical_outcome_valid": False,
    "trajectory_equivalence": False,
    "solver_equivalence": False,
    "physical_outcome_equivalence": False,
    "fig06_parity": False,
    "video06_parity": False,
    "paper_timing_valid": False,
    "timing_comparability": False,
    "renderer_colors_source_parity": False,
}

CLAIM_SCOPE = (
    "Current-source DART step-zero inspection of the author-pinned five-level "
    "card-house geometry, initial poses, and construction schedule."
)
CLAIM_BOUNDARY = (
    "Configuration only. The capture executes zero simulation substeps and does "
    "not validate release, standing, trajectory, solver behavior, contact "
    "dynamics, physical outcome, Fig. 6 parity, video parity, source renderer "
    "colors, paper timing, or performance. It is not a canonical Fig. 6 or video "
    "deliverable."
)

EXPECTED_METADATA_FLAGS = {
    "schema_version": SCHEMA_VERSION,
    "status": "valid_current_source_construction_only",
    "pass": True,
    "requirement_ids": ["fig.06", "video.06_card_house"],
    "artifact_valid": True,
    "configuration_port_valid": True,
    "construction_only": True,
    "trajectory_valid": False,
    "solver_valid": False,
    "physical_outcome_valid": False,
    "fig06_parity": False,
    "video06_parity": False,
    "paper_timing_valid": False,
    "paper_comparable": False,
    "canonical_fig06_deliverable": False,
    "canonical_video06_deliverable": False,
    "dynamics_executed": False,
    "actual_simulator": True,
    "generated_imagery": False,
    "manual_configuration_inspection_valid": True,
}

MANUAL_VERDICTS = {
    "five_level_card_house_configuration_visible": True,
    "four_projectile_cubes_visible": True,
    "cards_and_cubes_are_not_visibly_clipped": True,
    "step_zero_only_confirmed": True,
    "configuration_only_nonparity_boundary_confirmed": True,
}
DURABLE_STILL = f"{CAPTURE_DIR}/construction-step-0.png"
MANUAL_ARTIFACT_PATHS = (
    DURABLE_STILL,
    f"{CAPTURE_DIR}/panel.png",
)

CAPTURE_PATHS = {
    "capture-provenance.json",
    "contract.json",
    "run-summary.json",
    "verification.json",
    DURABLE_STILL,
    f"{CAPTURE_DIR}/metadata.json",
    f"{CAPTURE_DIR}/panel.compose.json",
    f"{CAPTURE_DIR}/panel.png",
    f"{CAPTURE_DIR}/timeline.json",
}
STAGING_PATHS = {
    "capture.stderr.txt",
    "capture.stdout.txt",
    "contract.stderr.txt",
    "contract.stdout.txt",
    "verification.stderr.txt",
    f"{CAPTURE_DIR}/frames/step_000000.png",
    f"{CAPTURE_DIR}/panel_frames/step_000000.png",
}
HUMAN_PATHS = {"manual-inspection.json"}
FINALIZER_OUTPUT_PATHS = {
    "REPORT.md",
    "artifact-index.json",
    "invocations.json",
    "metadata.json",
}
EXPECTED_FINAL_PATHS = CAPTURE_PATHS | HUMAN_PATHS | FINALIZER_OUTPUT_PATHS
INDEX_EXCLUSIONS = {"artifact-index.json", "metadata.json"}
ALLOWED_DIRECTORIES = {
    CAPTURE_DIR,
}

PROCESS_TERMINATION_GRACE_SECONDS = 1.0
EMPTY_SHA256 = hashlib.sha256(b"").hexdigest()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _payload_sha256(payload: Any) -> str:
    return hashlib.sha256(
        json.dumps(
            payload,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    def reject_nonfinite(value: str) -> None:
        raise ValueError(f"{path}: non-finite JSON number {value}")

    try:
        payload = json.loads(
            path.read_text(encoding="utf-8"),
            parse_constant=reject_nonfinite,
        )
    except json.JSONDecodeError as error:
        raise ValueError(f"{path}: invalid JSON") from error
    if not isinstance(payload, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return payload


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _require_file(path: Path, label: str) -> Path:
    try:
        resolved = path.expanduser().resolve(strict=True)
    except FileNotFoundError as error:
        raise FileNotFoundError(f"{label}: {path}") from error
    if not resolved.is_file():
        raise ValueError(f"{label} is not a regular file: {resolved}")
    return resolved


def _require_bundle_root(path: Path, *, create: bool) -> Path:
    original = Path(path).expanduser()
    absolute = Path(os.path.abspath(original))
    if original.is_symlink():
        raise ValueError(f"bundle root is a symlink: {original}")
    for ancestor in absolute.parents:
        if ancestor.is_symlink():
            raise ValueError(f"bundle root passes through a symlink: {original}")
    if original.exists():
        if not original.is_dir():
            raise ValueError(f"bundle root is not a directory: {original}")
    elif create:
        original.mkdir(parents=True)
    else:
        raise FileNotFoundError(f"bundle root does not exist: {original}")
    if original.is_symlink():
        raise ValueError(f"bundle root became a symlink: {original}")
    resolved = original.resolve(strict=True)
    if resolved != absolute:
        raise ValueError(f"bundle root passes through a symlink: {original}")
    return resolved


def _validate_relative_path(value: str, *, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label}: expected a nonempty relative path")
    pure = PurePosixPath(value)
    if pure.is_absolute() or ".." in pure.parts or pure.as_posix() != value:
        raise ValueError(f"{label}: unsafe relative path {value!r}")
    return value


def _bundle_files_and_directories(root: Path) -> tuple[set[str], set[str]]:
    files: set[str] = set()
    directories: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"bundle contains a symlink: {relative}")
        if stat.S_ISREG(mode):
            files.add(relative)
        elif stat.S_ISDIR(mode):
            directories.add(relative)
        else:
            raise ValueError(f"bundle contains a non-regular entry: {relative}")
    return files, directories


def _validate_bundle_paths(
    root: Path,
    *,
    complete: bool,
    allow_missing_manual: bool = False,
    allow_missing_capture: bool = False,
) -> set[str]:
    files, directories = _bundle_files_and_directories(root)
    if not directories <= ALLOWED_DIRECTORIES:
        raise ValueError(
            "card-house bundle has unexpected directories: "
            f"{sorted(directories - ALLOWED_DIRECTORIES)}"
        )
    if not files <= EXPECTED_FINAL_PATHS:
        raise ValueError(
            "card-house bundle has unexpected files: "
            f"{sorted(files - EXPECTED_FINAL_PATHS)}"
        )
    required = set(EXPECTED_FINAL_PATHS if complete else CAPTURE_PATHS)
    if not complete:
        required -= FINALIZER_OUTPUT_PATHS
        if not allow_missing_manual:
            required |= HUMAN_PATHS
        if allow_missing_capture:
            required -= CAPTURE_PATHS
    missing = required - files
    if missing:
        raise ValueError(f"card-house bundle is missing files: {sorted(missing)}")
    if complete and files != EXPECTED_FINAL_PATHS:
        raise ValueError(
            "card-house bundle membership changed: "
            f"missing={sorted(EXPECTED_FINAL_PATHS - files)}, "
            f"extra={sorted(files - EXPECTED_FINAL_PATHS)}"
        )
    return files


def artifact_index(root: Path) -> dict[str, Any]:
    files, _ = _bundle_files_and_directories(root)
    indexed = sorted(files - INDEX_EXCLUSIONS)
    return {
        "schema_version": INDEX_SCHEMA_VERSION,
        "artifact_count": len(indexed),
        "excluded": sorted(INDEX_EXCLUSIONS),
        "artifacts": [
            {
                "path": relative,
                "sha256": sha256(root / relative),
                "size_bytes": (root / relative).stat().st_size,
            }
            for relative in indexed
        ],
    }


def validate_artifact_index(root: Path, payload: dict[str, Any]) -> None:
    if set(payload) != {
        "schema_version",
        "artifact_count",
        "excluded",
        "artifacts",
    }:
        raise ValueError("malformed card-house artifact index")
    if payload.get("schema_version") != INDEX_SCHEMA_VERSION:
        raise ValueError("unexpected card-house artifact-index schema")
    if payload.get("excluded") != sorted(INDEX_EXCLUSIONS):
        raise ValueError("card-house artifact-index exclusions changed")
    if payload != artifact_index(root):
        raise ValueError("card-house artifact index is stale")


def _manual_template(root: Path) -> dict[str, Any]:
    return {
        "schema_version": MANUAL_SCHEMA_VERSION,
        "manual_inspected": False,
        "pass": False,
        "verdicts": {key: False for key in MANUAL_VERDICTS},
        "representative_artifacts": [
            {
                "path": relative,
                "sha256": sha256(root / relative),
                "observation": "",
            }
            for relative in MANUAL_ARTIFACT_PATHS
        ],
    }


def validate_manual_inspection(root: Path) -> dict[str, Any]:
    record = read_json(root / "manual-inspection.json")
    if set(record) != {
        "schema_version",
        "manual_inspected",
        "pass",
        "verdicts",
        "representative_artifacts",
    }:
        raise ValueError("malformed card-house manual-inspection payload")
    if record.get("schema_version") != MANUAL_SCHEMA_VERSION:
        raise ValueError("unexpected card-house manual-inspection schema")
    if record.get("manual_inspected") is not True or record.get("pass") is not True:
        raise ValueError("card-house manual inspection did not pass")
    if record.get("verdicts") != MANUAL_VERDICTS:
        raise ValueError("card-house manual-inspection verdicts changed")
    artifacts = record.get("representative_artifacts")
    if not isinstance(artifacts, list):
        raise ValueError("card-house manual-inspection artifacts must be a list")
    if any(
        not isinstance(item, dict) or set(item) != {"path", "sha256", "observation"}
        for item in artifacts
    ):
        raise ValueError("malformed card-house manual-inspection artifact")
    paths = [item["path"] for item in artifacts]
    if paths != list(MANUAL_ARTIFACT_PATHS) or len(paths) != len(set(paths)):
        raise ValueError("card-house manual-inspection artifact paths changed")
    for item in artifacts:
        relative = _validate_relative_path(
            item["path"], label="manual-inspection artifact"
        )
        artifact = root / relative
        if (
            not artifact.is_file()
            or artifact.is_symlink()
            or sha256(artifact) != item["sha256"]
        ):
            raise ValueError(
                f"card-house manual-inspection artifact changed: {relative}"
            )
        observation = item["observation"]
        if not isinstance(observation, str) or not observation.strip():
            raise ValueError(
                f"card-house manual-inspection observation missing: {relative}"
            )
    return record


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


def _is_sha256(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _is_git_object_id(value: Any) -> bool:
    return (
        isinstance(value, str)
        and len(value) == 40
        and all(character in "0123456789abcdef" for character in value)
    )


def _require_exact_keys(payload: Any, keys: set[str], *, label: str) -> dict[str, Any]:
    if not isinstance(payload, dict) or set(payload) != keys:
        actual = (
            sorted(payload) if isinstance(payload, dict) else type(payload).__name__
        )
        raise ValueError(f"{label}: exact keys changed: {actual}")
    return payload


def validate_configuration_contract(payload: dict[str, Any]) -> dict[str, Any]:
    _require_exact_keys(
        payload,
        {
            "schema_version",
            "kind",
            "author_source",
            "configuration_spec_source_sha256",
            "binary_binding",
            "source_configuration",
            "dart_observation",
            "adapter_boundaries",
            "claim_boundary",
            "visual_style",
        },
        label="card-house configuration contract",
    )
    if (
        payload.get("schema_version") != CONTRACT_SCHEMA_VERSION
        or payload.get("kind") != "configuration_only"
    ):
        raise ValueError("unexpected card-house configuration contract identity")

    author = _require_exact_keys(
        payload.get("author_source"),
        {
            "repository",
            "commit",
            "tree",
            "card_house_run_blob",
            "card_house_run_py_sha256",
            "fbf_config_py_sha256",
            "solver_fbf_py_sha256",
        },
        label="card-house author source",
    )
    if (
        author.get("repository") != EXPECTED_AUTHOR_REPOSITORY
        or author.get("commit") != EXPECTED_AUTHOR_COMMIT
        or author.get("tree") != EXPECTED_AUTHOR_TREE
        or not _is_git_object_id(author.get("card_house_run_blob"))
        or any(
            not _is_sha256(author.get(key))
            for key in (
                "card_house_run_py_sha256",
                "fbf_config_py_sha256",
                "solver_fbf_py_sha256",
            )
        )
    ):
        raise ValueError("card-house author-source identity changed")

    if payload.get("configuration_spec_source_sha256") != sha256(CONFIG_SPEC):
        raise ValueError("card-house contract is not bound to the current spec")
    binding = _require_exact_keys(
        payload.get("binary_binding"),
        {"role", "implementation_source_sha256"},
        label="card-house binary binding",
    )
    if binding != {
        "role": "dart_demos",
        "implementation_source_sha256": sha256(DEMO_SOURCE),
    }:
        raise ValueError("card-house contract is not bound to the current demo source")

    source = _require_exact_keys(
        payload.get("source_configuration"),
        {"scene", "cards", "cubes", "contact", "schedule", "solver"},
        label="card-house source configuration",
    )
    if source["scene"] != {
        "demo_scene_id": SCENE_ID,
        "levels": 5,
        "leaning_cards": 30,
        "bridges": 10,
        "cards": 40,
        "cubes": 4,
    }:
        raise ValueError("card-house source scene contract changed")
    schedule = source["schedule"]
    if schedule != {
        "display_time_step_seconds": 1.0 / 60.0,
        "substeps_per_frame": 4,
        "runtime_time_step_seconds": 1.0 / 240.0,
        "release_frame": 400,
        "release_substep": 1600,
        "total_frames": 800,
        "total_substeps": 3200,
    }:
        raise ValueError("card-house source schedule changed")
    cards = source["cards"]
    if (
        not isinstance(cards, dict)
        or cards.get("source_mobile") is not True
        or cards.get("mass_kg") != 25.0
        or cards.get("density_kg_m3") != 200.0
        or cards.get("lean_from_horizontal_degrees") != 65.0
        or cards.get("lean_from_vertical_degrees") != 25.0
        or cards.get("bridge_angle_degrees") != -1.0
    ):
        raise ValueError("card-house source card contract changed")
    cubes = source["cubes"]
    if (
        not isinstance(cubes, dict)
        or cubes.get("source_initially_kinematic") is not True
        or cubes.get("size_m") != [0.8, 0.8, 0.8]
        or cubes.get("density_kg_m3") != 500.0
        or cubes.get("mass_kg") != 256.0
    ):
        raise ValueError("card-house source cube contract changed")
    if source["contact"] != {"friction": 0.8, "gap_m": 0.005}:
        raise ValueError("card-house source contact contract changed")
    solver = source["solver"]
    if (
        not isinstance(solver, dict)
        or solver.get("type") != "fbf_exact_coulomb"
        or solver.get("max_contacts") != 4096
        or solver.get("max_outer") != 200
        or solver.get("outer_tol") != 1.0e-6
        or solver.get("warm_start") is not True
        or solver.get("warm_start_match_radius") != 0.02
        or solver.get("warm_start_normal_cosine") != 0.9
        or solver.get("warm_start_max_age") != 3
        or solver.get("warm_start_gamma_cap") != 1.0e4
        or solver.get("termination_residual") != "coulomb_rel"
        or solver.get("termination_tol") != 1.0e-6
    ):
        raise ValueError("card-house source solver contract changed")

    observation = _require_exact_keys(
        payload.get("dart_observation"),
        {"world", "collision", "solver_adapter", "ground", "cards", "cubes"},
        label="card-house DART observation",
    )
    if not isinstance(observation["world"], dict):
        raise ValueError("card-house world observation is malformed")
    if not isinstance(observation["collision"], dict):
        raise ValueError("card-house collision observation is malformed")
    if not isinstance(observation["solver_adapter"], dict):
        raise ValueError("card-house solver-adapter observation is malformed")
    if not isinstance(observation["ground"], dict):
        raise ValueError("card-house ground observation is malformed")
    if (
        not isinstance(observation["cards"], list)
        or len(observation["cards"]) != 40
        or not isinstance(observation["cubes"], list)
        or len(observation["cubes"]) != 4
    ):
        raise ValueError("card-house observation must contain 40 cards and 4 cubes")
    for label, bodies, expected_mobile in (
        ("card", observation["cards"], True),
        ("cube", observation["cubes"], False),
    ):
        for index, body in enumerate(bodies):
            if not isinstance(body, dict):
                raise ValueError(f"card-house {label} {index} is malformed")
            if body.get("mobile") is not expected_mobile:
                raise ValueError(
                    f"card-house {label} {index} mobility contract changed"
                )
            for velocity_key in (
                "initial_linear_velocity_m_s",
                "initial_angular_velocity_rad_s",
            ):
                velocity = body.get(velocity_key)
                if velocity != [0.0, 0.0, 0.0]:
                    raise ValueError(
                        f"card-house {label} {index} has nonzero {velocity_key}"
                    )

    if payload.get("adapter_boundaries") != {
        "source_contact_gap_recorded_m": 0.005,
        "source_contact_gap_semantics_implemented": False,
        "source_solver_backend_semantics_implemented": False,
    }:
        raise ValueError("card-house adapter boundary changed")
    if payload.get("claim_boundary") != EXPECTED_CLAIM_BOUNDARY:
        raise ValueError("card-house configuration claim boundary changed")
    if payload.get("visual_style") != {
        "renderer_only": True,
        "description": "restrained alternating paper colors for layer legibility",
    }:
        raise ValueError("card-house visual-style contract changed")
    return payload


def _contract_argv(demo: Path) -> list[str]:
    return [
        str(demo),
        "--fbf-author-card-house-contract",
        SCENE_ID,
    ]


def _runner_contract_argv(demo: Path) -> list[str]:
    return [
        str(demo),
        "--threads",
        "1",
        "--scene-physics-contract",
        SCENE_ID,
    ]


def _parse_ldd_in_tree_paths(output: str, *, build_root: Path) -> list[Path]:
    paths: set[Path] = set()
    resolved_build_root = build_root.resolve(strict=True)
    for line in output.splitlines():
        if "not found" in line:
            raise ValueError(f"unresolved runtime dependency: {line.strip()}")
        if "=>" in line:
            fields = line.split("=>", 1)[1].strip().split()
        else:
            fields = line.strip().split()
        if not fields or not fields[0].startswith("/"):
            continue
        path = Path(fields[0]).resolve(strict=True)
        try:
            path.relative_to(resolved_build_root)
        except ValueError:
            continue
        paths.add(path)
    return sorted(paths, key=lambda path: path.as_posix())


def _run_command(
    argv: Sequence[str],
    *,
    timeout: float,
    cwd: Path = ROOT,
) -> subprocess.CompletedProcess[str]:
    process = subprocess.Popen(
        list(argv),
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            stdout, stderr = process.communicate(
                timeout=PROCESS_TERMINATION_GRACE_SECONDS
            )
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            stdout, stderr = process.communicate()
        raise subprocess.TimeoutExpired(argv, timeout, output=stdout, stderr=stderr)
    completed = subprocess.CompletedProcess(argv, process.returncode, stdout, stderr)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise ValueError(
            f"command failed with exit {completed.returncode}: "
            f"{' '.join(argv)}: {detail}"
        )
    return completed


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


def _source_identity(
    *,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, Any]:
    inputs = {
        "finalizer": Path(__file__).resolve(),
        "finalizer_test": FINALIZER_TEST,
        "visual_runner": runner,
        "visual_runner_test": RUNNER_TEST,
        "fixture_cmake": FIXTURE_CMAKE,
        "fixture_test": FIXTURE_TEST,
        "image_compose": IMAGE_COMPOSE,
        "image_tools": IMAGE_TOOLS,
        "image_verdict": IMAGE_VERDICT,
        "demo_cmake": DEMO_CMAKE,
        "demo_main": DEMO_MAIN,
        "configuration_spec": CONFIG_SPEC,
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
    identity: dict[str, Any] = {
        name: {
            "path": str(_require_file(path, name)),
            "sha256": sha256(_require_file(path, name)),
        }
        for name, path in inputs.items()
    }
    identity.update(_in_tree_runtime_dependency_identity(demo, label="demo"))
    return identity


def _query_contract(
    demo: Path, *, timeout: float
) -> tuple[subprocess.CompletedProcess[str], dict[str, Any]]:
    process = _run_command(_contract_argv(demo), timeout=timeout)
    if process.stderr:
        raise ValueError("card-house contract query emitted unexpected stderr")
    payload = _parse_json_stdout(
        process.stdout, label="card-house configuration contract"
    )
    validate_configuration_contract(payload)
    return process, payload


def _expected_demo_argv(demo: Path, root: Path) -> list[str]:
    output = root / CAPTURE_DIR
    return [
        str(demo),
        "--scene",
        SCENE_ID,
        "--headless",
        "--steps",
        "0",
        "--width",
        "1280",
        "--height",
        "720",
        "--threads",
        "1",
        "--headless-sidecar",
        str(output / "timeline.json"),
        "--headless-shot-at",
        f"0:{output / 'frames/step_000000.png'}",
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
        SCHEDULE_ID,
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
        SCHEDULE_ID,
        "--demo",
        str(demo),
        "--output-root",
        str(bundle),
        "--ffmpeg",
        str(ffmpeg),
        "--ffprobe",
        str(ffprobe),
    ]


def _validate_zero_step_diagnostics(payload: Any, *, label: str) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError(f"{label}: solver diagnostics are unavailable")
    for key in (
        "iterations",
        "total_iterations",
        "exact_solves",
        "exact_attempts",
        "accepted_at_cap",
        "exact_failures",
        "boxed_lcp_fallbacks",
        "warm_starts",
        "contacts",
    ):
        if payload.get(key) != 0:
            raise ValueError(f"{label}: {key} proves unexpected execution")
    for key in ("residual", "best_residual", "worst_residual"):
        if payload.get(key) is not None:
            raise ValueError(f"{label}: {key} exists before simulation")
    if payload.get("status") != "not_run" or payload.get("fbf_status") != "not_run":
        raise ValueError(f"{label}: solver status is not step-zero/not-run")
    if not isinstance(payload.get("solver"), str) or not payload["solver"]:
        raise ValueError(f"{label}: configured solver identity is unavailable")
    if payload.get("available") is not True:
        raise ValueError(f"{label}: configured solver diagnostics are unavailable")
    return payload


def _validate_schedule(plan: Any, *, demo: Path, bundle: Path) -> dict[str, Any]:
    if not isinstance(plan, dict):
        raise ValueError("card-house capture schedule is missing")
    if set(plan) != {
        "id",
        "title",
        "scene",
        "source_segment",
        "collision_detector",
        "collision_detector_override",
        "configuration",
        "total_steps",
        "time_step_seconds",
        "capture_steps",
        "panel_steps",
        "panel_labels",
        "actions",
        "output",
        "runnable",
        "adapter_gap",
        "long_run",
        "actual_simulator_required",
        "generated_imagery_allowed",
        "paper_comparable",
        "known_mismatches",
        "known_gate_blockers",
        "evidence_ready",
        "demo_argv",
        "demo_command",
    }:
        raise ValueError("card-house capture schedule exact keys changed")
    expected_fields = {
        "id": SCHEDULE_ID,
        "scene": SCENE_ID,
        "source_segment": "paper_tables_6_7_no_video_segment",
        "collision_detector": "native",
        "collision_detector_override": False,
        "configuration": EXPECTED_CONFIGURATION,
        "total_steps": 0,
        "time_step_seconds": 1.0 / 240.0,
        "capture_steps": [0],
        "panel_steps": [0],
        "panel_labels": [EXPECTED_PANEL_LABEL],
        "actions": [],
        "runnable": True,
        "adapter_gap": None,
        "long_run": False,
        "actual_simulator_required": True,
        "generated_imagery_allowed": False,
        "paper_comparable": False,
        "evidence_ready": True,
        "demo_argv": _expected_demo_argv(demo, bundle),
    }
    for key, expected in expected_fields.items():
        if plan.get(key) != expected:
            raise ValueError(f"card-house capture schedule field {key} changed")
    output = plan.get("output")
    expected_output = {
        "directory": str(bundle / CAPTURE_DIR),
        "timeline": str(bundle / CAPTURE_DIR / "timeline.json"),
        "panel": str(bundle / CAPTURE_DIR / "panel.png"),
        "mp4": None,
        "gif": None,
    }
    if output != expected_output:
        raise ValueError("card-house capture schedule output contract changed")
    if plan.get("known_mismatches") != EXPECTED_MISMATCHES:
        raise ValueError("card-house capture mismatch boundary changed")
    if plan.get("known_gate_blockers") != []:
        raise ValueError("card-house construction schedule gained a gate blocker")
    return plan


def _validate_capture_physics_contract(
    payload: Any, *, queried_contract: dict[str, Any]
) -> dict[str, Any]:
    if not isinstance(payload, dict):
        raise ValueError("card-house timeline physics contract is missing")
    validate_configuration_contract(payload)
    validate_configuration_contract(queried_contract)
    if payload != queried_contract:
        raise ValueError(
            "card-house timeline physics contract differs from the separately "
            "queried contract"
        )
    if _payload_sha256(payload) != _payload_sha256(queried_contract):
        raise ValueError(
            "card-house timeline physics contract payload digest differs from "
            "the separately queried contract"
        )
    return payload


def _validate_timeline(
    root: Path, *, demo: Path, queried_contract: dict[str, Any]
) -> dict[str, Any]:
    path = root / CAPTURE_DIR / "timeline.json"
    timeline = read_json(path)
    for key, expected in {
        "schema_version": "dart.demos_headless_timeline/v1",
        "scene": SCENE_ID,
        "active_scene": SCENE_ID,
        "total_steps": 0,
        "completed_steps": 0,
        "width": 1280,
        "height": 720,
        "collision_detector": "native",
        "event_order": "captures_before_actions_at_each_completed_step",
        "actions": [],
    }.items():
        if timeline.get(key) != expected:
            raise ValueError(f"card-house timeline field {key} changed")
    _validate_capture_physics_contract(
        timeline.get("physics_contract"), queried_contract=queried_contract
    )
    runtime_command = timeline.get("runtime_command")
    if not isinstance(runtime_command, str):
        raise ValueError("card-house runtime command is unavailable")
    if shlex.split(runtime_command) != _expected_demo_argv(demo, root):
        raise ValueError("card-house timeline runtime command changed")
    steps = timeline.get("steps")
    if (
        not isinstance(steps, list)
        or len(steps) != 1
        or steps[0].get("step") != 0
        or steps[0].get("sim_time") != 0.0
    ):
        raise ValueError("card-house timeline is not exactly step zero")
    diagnostics = _validate_zero_step_diagnostics(
        steps[0].get("solver_diagnostics"),
        label="card-house timeline step zero",
    )
    if timeline.get("solver_diagnostics") != diagnostics:
        raise ValueError("card-house final diagnostics differ from step zero")
    shots = timeline.get("shots")
    frame = root / CAPTURE_DIR / "frames/step_000000.png"
    if (
        not isinstance(shots, list)
        or len(shots) != 1
        or shots[0].get("step") != 0
        or shots[0].get("sim_time") != 0.0
        or shots[0].get("path") != str(frame)
        or shots[0].get("success") is not True
        or shots[0].get("solver_diagnostics") != diagnostics
    ):
        raise ValueError("card-house step-zero shot contract changed")
    events = timeline.get("events")
    if (
        not isinstance(events, list)
        or len(events) != 1
        or events[0].get("sequence") != 0
        or events[0].get("step") != 0
        or events[0].get("type") != "shot"
    ):
        raise ValueError("card-house step-zero event contract changed")
    return {
        "path": f"{CAPTURE_DIR}/timeline.json",
        "sha256": sha256(path),
        "total_steps": 0,
        "completed_steps": 0,
        "shot_count": 1,
        "dynamics_executed": False,
        "solver_diagnostics": diagnostics,
    }


def _strict_image_verdict(path: Path, *, label: str) -> dict[str, Any]:
    if not path.is_file() or path.is_symlink() or path.stat().st_size == 0:
        raise ValueError(f"{label}: image is missing, linked, or empty")
    verdict = build_verdict(path)
    checks = verdict.get("checks")
    if not isinstance(checks, dict):
        raise ValueError(f"{label}: decoded image checks are unavailable")
    non_blank = checks.get("non_blank", {})
    contrast = checks.get("contrast", {})
    thresholds = contrast.get("thresholds", {})
    # This restrained renderer palette intentionally tops out below the generic
    # image-verdict bright threshold. Gate real contrast using the same decoded
    # central-region spread plus dark/midtone populations, and record the
    # scene-specific decision explicitly instead of pretending the generic
    # bright-pixel heuristic passed.
    construction_contrast = {
        "pass": (
            contrast.get("spread", -1.0) >= thresholds.get("min_spread", 90.0)
            and contrast.get("dark_pixels", 0) >= thresholds.get("min_dark_pixels", 1)
            and contrast.get("mid_pixels", 0) >= thresholds.get("min_mid_pixels", 1)
        ),
        "basis": "decoded central-region luminance spread plus dark/midtone populations",
        "bright_population_required": False,
        "spread": contrast.get("spread"),
        "minimum_spread": thresholds.get("min_spread"),
        "dark_pixels": contrast.get("dark_pixels"),
        "minimum_dark_pixels": thresholds.get("min_dark_pixels"),
        "mid_pixels": contrast.get("mid_pixels"),
        "minimum_mid_pixels": thresholds.get("min_mid_pixels"),
    }
    verdict["construction_contrast_gate"] = construction_contrast
    verdict["thresholds_used"]["construction_contrast_gate"] = {
        "minimum_spread": thresholds.get("min_spread"),
        "minimum_dark_pixels": thresholds.get("min_dark_pixels"),
        "minimum_mid_pixels": thresholds.get("min_mid_pixels"),
        "bright_population_required": False,
    }
    verdict["pass"] = (
        non_blank.get("pass") is True and construction_contrast["pass"] is True
    )
    verdict["reasons"] = []
    if non_blank.get("pass") is not True:
        verdict["reasons"].extend(non_blank.get("reasons", ["nonblank failed"]))
    if construction_contrast["pass"] is not True:
        verdict["reasons"].append("construction contrast gate failed")
    if verdict["pass"] is not True:
        raise ValueError(f"{label}: strict image verdict failed: {verdict['reasons']}")
    return verdict


def _verification_result(payload: Any) -> dict[str, Any]:
    if (
        not isinstance(payload, dict)
        or set(payload)
        != {"schema_version", "kind", "results", "group_outputs", "pass"}
        or payload.get("schema_version") != RUNNER_SCHEMA_VERSION
        or payload.get("kind") != "verification"
        or payload.get("pass") is not True
        or payload.get("group_outputs") != []
        or not isinstance(payload.get("results"), list)
        or len(payload["results"]) != 1
    ):
        raise ValueError("card-house runner verification is malformed")
    result = payload["results"][0]
    if (
        not isinstance(result, dict)
        or set(result)
        != {
            "schedule",
            "timeline",
            "panel",
            "media",
            "metadata_path",
            "metadata_sha256",
            "pass",
        }
        or result.get("schedule") != SCHEDULE_ID
        or result.get("pass") is not True
        or result.get("media") != []
    ):
        raise ValueError("card-house runner verification result changed")
    return result


def validate_capture_bundle(
    root: Path,
    *,
    demo: Path,
    verification: dict[str, Any] | None = None,
) -> dict[str, Any]:
    summary = read_json(root / "run-summary.json")
    if (
        set(summary)
        != {
            "schema_version",
            "kind",
            "results",
            "failures",
            "group_outputs",
            "group_skips",
            "pass",
        }
        or summary.get("schema_version") != RUNNER_SCHEMA_VERSION
        or summary.get("kind") != "capture_run"
        or summary.get("pass") is not True
        or summary.get("failures") != []
        or summary.get("group_outputs") != []
        or summary.get("group_skips") != []
        or not isinstance(summary.get("results"), list)
        or len(summary["results"]) != 1
    ):
        raise ValueError("card-house capture run summary is malformed")
    result = summary["results"][0]
    if not isinstance(result, dict):
        raise ValueError("card-house capture result is malformed")
    if set(result) != {
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
    }:
        raise ValueError("card-house capture result exact keys changed")
    if result.get("schema_version") not in RUNNER_CAPTURE_RESULT_SCHEMA_VERSIONS:
        raise ValueError("card-house capture result schema changed")
    plan = _validate_schedule(result.get("schedule"), demo=demo, bundle=root)
    expected_claims = {
        "kind": "capture_result",
        "actual_simulator": True,
        "generated_imagery": False,
        "paper_comparable": False,
        "automated_semantic_outcome_validated": False,
        "semantic_outcome_gate": EXPECTED_SEMANTIC_OUTCOME_GATE,
        "known_mismatches": EXPECTED_MISMATCHES,
        "pass": True,
    }
    for key, expected in expected_claims.items():
        if result.get(key) != expected:
            raise ValueError(f"card-house capture claim {key} changed")
    if result.get("media_validation") != []:
        raise ValueError("card-house construction capture unexpectedly has media")
    stored_metadata = read_json(root / CAPTURE_DIR / "metadata.json")
    if stored_metadata != result:
        raise ValueError("card-house capture metadata differs from run summary")
    runtime = result.get("runtime")
    expected_runtime_keys = {
        "demo_argv",
        "demo_path",
        "demo_sha256",
        "ffmpeg",
        "ffprobe",
    }
    if result.get("schema_version") == RUNNER_CAPTURE_RESULT_SCHEMA_VERSION:
        expected_runtime_keys.add("scene_physics_provenance")
    if (
        not isinstance(runtime, dict)
        or set(runtime) != expected_runtime_keys
        or runtime.get("demo_argv") != _expected_demo_argv(demo, root)
        or runtime.get("demo_path") != str(demo)
        or runtime.get("demo_sha256") != sha256(demo)
    ):
        raise ValueError("card-house capture runtime binding changed")

    queried_contract = read_json(root / "contract.json")
    validate_configuration_contract(queried_contract)
    if result.get("schema_version") == RUNNER_CAPTURE_RESULT_SCHEMA_VERSION:
        provenance = build_semantic_physics_provenance(queried_contract)
        expected_provenance = {
            "schema_version": SEMANTIC_PROVENANCE_SCHEMA_VERSION,
            "contract_schema_version": provenance.schema_version,
            "family": provenance.family,
            "query_argv": _runner_contract_argv(demo),
            "contract_sha256": _payload_sha256(queried_contract),
            "semantic_physics_sha256": provenance.semantic_sha256,
            "broad_implementation_sha256": (provenance.broad_implementation_sha256),
            "sidecar_contract_match": True,
        }
        if runtime.get("scene_physics_provenance") != expected_provenance:
            raise ValueError("card-house semantic physics provenance changed")
    timeline = _validate_timeline(root, demo=demo, queried_contract=queried_contract)
    stored_timeline = result.get("timeline_validation")
    if (
        not isinstance(stored_timeline, dict)
        or stored_timeline.get("sidecar") != str(root / CAPTURE_DIR / "timeline.json")
        or stored_timeline.get("shot_count") != 1
        or stored_timeline.get("action_count") != 0
        or stored_timeline.get("step_count") != 1
        or set(stored_timeline.get("frames", {})) != {"0"}
        or stored_timeline.get("final_solver_diagnostics")
        != timeline["solver_diagnostics"]
    ):
        raise ValueError("card-house stored timeline validation changed")

    recorded_frame = root / CAPTURE_DIR / "frames/step_000000.png"
    durable_frame = root / DURABLE_STILL
    if durable_frame.is_file() and not durable_frame.is_symlink():
        frame = durable_frame
        durable = True
    elif recorded_frame.is_file() and not recorded_frame.is_symlink():
        frame = recorded_frame
        durable = False
    else:
        raise ValueError(
            "card-house durable step-zero still is missing and staging is unavailable"
        )
    panel_frame = root / CAPTURE_DIR / "panel_frames/step_000000.png"
    panel = root / CAPTURE_DIR / "panel.png"
    frame_binding = stored_timeline["frames"]["0"]
    if frame_binding.get("path") != str(recorded_frame) or frame_binding.get(
        "sha256"
    ) != sha256(frame):
        raise ValueError("card-house step-zero frame binding changed")
    panel_validation = result.get("panel_validation")
    compose = root / CAPTURE_DIR / "panel.compose.json"
    compose_payload = read_json(compose)
    if compose_payload != {
        "schema_version": "dart.image_compose/v1",
        "mode": "side-by-side",
        "inputs": [str(panel_frame)],
        "out": str(panel),
        "labels": [EXPECTED_PANEL_LABEL],
        "width": 676,
        "height": 538,
        "stats": {},
        "pass": True,
    }:
        raise ValueError("card-house panel compose manifest changed")
    source_frames = (
        [{"path": str(panel_frame), "sha256": sha256(panel_frame)}]
        if panel_frame.is_file() and not panel_frame.is_symlink()
        else (
            panel_validation.get("source_frames")
            if isinstance(panel_validation, dict)
            else None
        )
    )
    if (
        not isinstance(panel_validation, dict)
        or panel_validation.get("path") != str(panel)
        or panel_validation.get("sha256") != sha256(panel)
        or panel_validation.get("source_frames") != source_frames
        or not isinstance(source_frames, list)
        or len(source_frames) != 1
        or source_frames[0].get("path") != str(panel_frame)
        or not _is_sha256(source_frames[0].get("sha256"))
        or panel_validation.get("compose_manifest_path") != str(compose)
        or panel_validation.get("compose_manifest_sha256") != sha256(compose)
        or panel_validation.get("compose_manifest") != compose_payload
    ):
        raise ValueError("card-house panel composition binding changed")

    frame_verdict = _strict_image_verdict(frame, label="card-house step-zero frame")
    panel_verdict = _strict_image_verdict(panel, label="card-house panel")
    if frame_verdict.get("image") != {
        "path": str(frame),
        "width": 1280,
        "height": 720,
    }:
        raise ValueError("card-house step-zero dimensions changed")
    if panel_verdict.get("image") != {
        "path": str(panel),
        "width": 676,
        "height": 538,
    }:
        raise ValueError("card-house panel dimensions changed")
    if verification is None:
        verification = read_json(root / "verification.json")
    verified = _verification_result(verification)
    verified_panel = verified.get("panel")
    verified_timeline = verified.get("timeline")
    if (
        verified.get("metadata_path") != str(root / CAPTURE_DIR / "metadata.json")
        or verified.get("metadata_sha256")
        != sha256(root / CAPTURE_DIR / "metadata.json")
        or not isinstance(verified_panel, dict)
        or verified_panel.get("pass") is not True
        or verified_panel.get("image")
        != {"path": str(panel), "width": 676, "height": 538}
        or verified_panel.get("checks", {}).get("non_blank", {}).get("pass") is not True
        or not isinstance(verified_timeline, dict)
        or verified_timeline.get("sidecar") != str(root / CAPTURE_DIR / "timeline.json")
        or verified_timeline.get("shot_count") != 1
        or verified_timeline.get("step_count") != 1
        or verified_timeline.get("final_solver_diagnostics")
        != timeline["solver_diagnostics"]
        or verified_timeline.get("frames", {}).get("0", {}).get("path")
        != str(recorded_frame)
        or verified_timeline.get("frames", {}).get("0", {}).get("sha256")
        != frame_binding["sha256"]
    ):
        raise ValueError("card-house live verification binding changed")
    return {
        "schedule_id": SCHEDULE_ID,
        "scene_id": SCENE_ID,
        "configuration": plan["configuration"],
        "capture_step": 0,
        "simulation_substeps_executed": 0,
        "timeline": timeline,
        "step_zero_frame": {
            "path": (
                DURABLE_STILL if durable else f"{CAPTURE_DIR}/frames/step_000000.png"
            ),
            "original_capture_path": f"{CAPTURE_DIR}/frames/step_000000.png",
            "sha256": sha256(frame),
            "size_bytes": frame.stat().st_size,
            "durable_promoted_copy": durable,
            "strict_nonblank_contrast_verdict": frame_verdict,
        },
        "panel": {
            "path": f"{CAPTURE_DIR}/panel.png",
            "sha256": sha256(panel),
            "size_bytes": panel.stat().st_size,
            "strict_nonblank_contrast_verdict": panel_verdict,
            "compose_manifest_path": f"{CAPTURE_DIR}/panel.compose.json",
            "compose_manifest_sha256": sha256(compose),
        },
        "runner_verification": {
            "path": "verification.json",
            "sha256": sha256(root / "verification.json"),
            "pass": True,
        },
        "configuration_only": True,
        "trajectory_valid": False,
        "solver_valid": False,
        "physical_outcome_valid": False,
    }


def _validate_capture_provenance(
    payload: Any,
    *,
    bundle: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, Any]:
    expected_keys = {
        "capture_argv",
        "capture_returncode",
        "run_summary_path",
        "run_summary_sha256",
        "run_summary_validated",
        "capture_stdout_sha256",
        "capture_stdout_ends_with_run_summary",
        "capture_stderr_sha256",
        "contract_query_argv",
        "contract_query_returncode",
        "contract_path",
        "contract_sha256",
        "contract_payload_sha256_before",
        "contract_payload_sha256_after",
        "contract_stdout_sha256_before",
        "contract_stdout_sha256_after",
        "contract_stdout_byte_identical_before_after",
        "contract_stderr_sha256_before",
        "contract_stderr_sha256_after",
        "verification_argv",
        "verification_returncode",
        "verification_path",
        "verification_sha256",
        "verification_payload_sha256",
        "verification_stdout_sha256",
        "verification_stderr_sha256",
        "durable_still_path",
        "durable_still_sha256",
        "staging_pruned",
        "source_identity_before",
        "source_identity_after",
    }
    if not isinstance(payload, dict) or set(payload) != expected_keys:
        raise ValueError("card-house capture provenance is malformed")
    contract = read_json(bundle / "contract.json")
    validate_configuration_contract(contract)
    verification = read_json(bundle / "verification.json")
    _verification_result(verification)
    identity = _source_identity(
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    exact_fields = {
        "capture_argv": _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        "capture_returncode": 0,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "capture_stdout_ends_with_run_summary": True,
        "capture_stderr_sha256": EMPTY_SHA256,
        "contract_query_argv": _contract_argv(demo),
        "contract_query_returncode": 0,
        "contract_path": "contract.json",
        "contract_sha256": sha256(bundle / "contract.json"),
        "contract_payload_sha256_before": _payload_sha256(contract),
        "contract_payload_sha256_after": _payload_sha256(contract),
        "contract_stdout_byte_identical_before_after": True,
        "contract_stderr_sha256_before": EMPTY_SHA256,
        "contract_stderr_sha256_after": EMPTY_SHA256,
        "verification_argv": _verification_argv(
            python, runner, demo, bundle, ffmpeg, ffprobe
        ),
        "verification_returncode": 0,
        "verification_path": "verification.json",
        "verification_sha256": sha256(bundle / "verification.json"),
        "verification_payload_sha256": _payload_sha256(verification),
        "verification_stderr_sha256": EMPTY_SHA256,
        "durable_still_path": DURABLE_STILL,
        "durable_still_sha256": sha256(bundle / DURABLE_STILL),
        "staging_pruned": True,
        "source_identity_before": identity,
        "source_identity_after": identity,
    }
    if any(payload.get(key) != value for key, value in exact_fields.items()):
        raise ValueError("card-house capture-time provenance changed")
    for key in (
        "capture_stdout_sha256",
        "contract_stdout_sha256_before",
        "contract_stdout_sha256_after",
        "verification_stdout_sha256",
    ):
        if not _is_sha256(payload.get(key)):
            raise ValueError(f"card-house capture provenance {key} is malformed")
    if (
        payload["contract_stdout_sha256_before"]
        != payload["contract_stdout_sha256_after"]
    ):
        raise ValueError("card-house contract stdout hashes changed during capture")
    frame_binding = read_json(bundle / "run-summary.json")["results"][0][
        "timeline_validation"
    ]["frames"]["0"]
    if frame_binding.get("sha256") != payload["durable_still_sha256"]:
        raise ValueError("card-house durable still differs from capture record")
    return payload


def _validate_invocations(
    bundle: Path,
    *,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, Any]:
    payload = read_json(bundle / "invocations.json")
    provenance = read_json(bundle / "capture-provenance.json")
    expected = {
        "schema_version": INVOCATIONS_SCHEMA_VERSION,
        "contract_query": {
            "argv": _contract_argv(demo),
            "returncode": 0,
            "stdout_sha256": provenance["contract_stdout_sha256_after"],
            "stderr_sha256": provenance["contract_stderr_sha256_after"],
            "payload_path": "contract.json",
            "payload_sha256": sha256(bundle / "contract.json"),
        },
        "capture": {
            "argv": _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
            "returncode": 0,
            "stdout_sha256": provenance["capture_stdout_sha256"],
            "stderr_sha256": provenance["capture_stderr_sha256"],
            "run_summary_path": "run-summary.json",
            "run_summary_sha256": sha256(bundle / "run-summary.json"),
        },
        "capture_verification": {
            "argv": _verification_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
            "returncode": 0,
            "stdout_path": "verification.json",
            "stdout_sha256": sha256(bundle / "verification.json"),
            "raw_stdout_sha256": provenance["verification_stdout_sha256"],
            "stderr_sha256": provenance["verification_stderr_sha256"],
        },
    }
    if payload != expected:
        raise ValueError("card-house invocation bindings changed")
    return payload


def _report_markdown(
    *,
    contract: dict[str, Any],
    capture: dict[str, Any],
    manual: dict[str, Any],
) -> str:
    author = contract["author_source"]
    return f"""# Author-pinned five-level card-house construction

Status: valid current-source configuration-only evidence

## What is validated

- The queried DART configuration contract is bound to the current shared spec,
  production scene source, executable, and in-tree DART runtime dependencies.
- The contract pins author source commit {author['commit']} and reports 40 card
  bodies plus four projectile cubes.
- The real DART demo produced one decoded step-zero frame and one panel. Both
  pass strict nonblank and contrast gates.
- Manual inspection passed for the hash-bound frame and panel.

## Exact execution boundary

- Schedule: {SCHEDULE_ID}
- Scene: {SCENE_ID}
- Captured substep: {capture['capture_step']}
- Simulation substeps executed: {capture['simulation_substeps_executed']}
- Dynamics executed: false
- Configuration port valid: true

## Claims deliberately not made

- Trajectory valid: false
- Solver valid: false
- Physical outcome valid: false
- Fig. 6 parity: false
- Video 06 parity: false
- Paper timing valid: false
- Canonical Fig. 6 or video deliverable: false

{CLAIM_BOUNDARY}

## Manual inspection

The record is manual-inspection.json with schema {manual['schema_version']}.
Its representative artifacts are exactly the step-zero frame and panel.
"""


EXPECTED_FINAL_METADATA_KEYS = set(EXPECTED_METADATA_FLAGS) | {
    "evidence_date",
    "claim_scope",
    "claim_boundary",
    "configuration_contract",
    "capture_summary",
    "capture_provenance",
    "manual_inspection",
    "run_summary",
    "verification",
    "invocations",
    "report",
    "artifact_index",
    "source_identity",
}


def _clean_finalizer_outputs(bundle: Path) -> None:
    for relative in FINALIZER_OUTPUT_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()


def _clean_capture_outputs(bundle: Path) -> None:
    for relative in CAPTURE_PATHS | STAGING_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()
    capture_dir = bundle / CAPTURE_DIR
    if capture_dir.is_dir() and not capture_dir.is_symlink():
        shutil.rmtree(capture_dir)


def _promote_and_prune_capture_staging(bundle: Path) -> None:
    source = bundle / CAPTURE_DIR / "frames/step_000000.png"
    destination = bundle / DURABLE_STILL
    if not source.is_file() or source.is_symlink():
        raise ValueError("card-house captured step-zero staging frame is unavailable")
    if destination.exists() or destination.is_symlink():
        raise ValueError("card-house durable step-zero destination already exists")
    shutil.copyfile(source, destination)
    if sha256(destination) != sha256(source):
        raise ValueError("card-house durable step-zero copy changed bytes")
    for relative in STAGING_PATHS:
        path = bundle / relative
        if path.is_file() or path.is_symlink():
            path.unlink()
    for relative in (
        f"{CAPTURE_DIR}/frames",
        f"{CAPTURE_DIR}/panel_frames",
    ):
        path = bundle / relative
        if path.is_dir() and not path.is_symlink():
            shutil.rmtree(path)
    files, directories = _bundle_files_and_directories(bundle)
    if files & STAGING_PATHS:
        raise ValueError("card-house ignored staging files survived pruning")
    if directories - ALLOWED_DIRECTORIES:
        raise ValueError("card-house ignored staging directories survived pruning")
    if not destination.is_file() or destination.is_symlink():
        raise ValueError("card-house durable step-zero still was not promoted")


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
    ) as temporary:
        snapshot = Path(temporary) / "bundle"
        shutil.copytree(bundle, snapshot)
        try:
            yield commit
            if not committed:
                raise ValueError("card-house bundle transaction was not committed")
        except BaseException:
            if not committed:
                _remove_bundle_root(bundle)
                shutil.copytree(snapshot, bundle)
            raise


def _run_verification(
    *,
    bundle: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
    timeout: float,
) -> tuple[subprocess.CompletedProcess[str], dict[str, Any]]:
    process = _run_command(
        _verification_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
        timeout=timeout,
    )
    payload = _parse_json_stdout(process.stdout, label="card-house visual verification")
    _verification_result(payload)
    write_json(bundle / "verification.json", payload)
    (bundle / "verification.stderr.txt").write_text(process.stderr, encoding="utf-8")
    return process, payload


def _write_fresh_capture(
    args: argparse.Namespace,
    *,
    bundle: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
    source_identity_before: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    contract_before_process, contract_before = _query_contract(
        demo, timeout=args.contract_timeout
    )
    capture_argv = _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe)
    capture_process = _run_command(capture_argv, timeout=args.capture_timeout)
    contract_after_process, contract_after = _query_contract(
        demo, timeout=args.contract_timeout
    )
    source_identity_after = _source_identity(
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    if source_identity_after != source_identity_before:
        raise ValueError(
            "card-house source/binary/runtime identity changed during capture"
        )
    if contract_before != contract_after:
        raise ValueError("card-house contract payload changed during capture")
    if contract_before_process.stdout != contract_after_process.stdout:
        raise ValueError("card-house contract stdout changed during capture")
    if contract_before_process.stderr != contract_after_process.stderr:
        raise ValueError("card-house contract stderr changed during capture")

    capture_stdout = bundle / "capture.stdout.txt"
    capture_stderr = bundle / "capture.stderr.txt"
    capture_stdout.write_text(capture_process.stdout, encoding="utf-8")
    capture_stderr.write_text(capture_process.stderr, encoding="utf-8")
    contract_stdout = bundle / "contract.stdout.txt"
    contract_stderr = bundle / "contract.stderr.txt"
    contract_stdout.write_text(contract_after_process.stdout, encoding="utf-8")
    contract_stderr.write_text(contract_after_process.stderr, encoding="utf-8")
    write_json(bundle / "contract.json", contract_after)

    run_summary = read_json(bundle / "run-summary.json")
    expected_stdout_suffix = json.dumps(run_summary, indent=2, sort_keys=True) + "\n"
    if not capture_process.stdout.endswith(expected_stdout_suffix):
        raise ValueError("card-house capture stdout does not end with run summary")
    if (
        run_summary.get("kind") != "capture_run"
        or run_summary.get("pass") is not True
        or run_summary.get("failures") != []
        or run_summary.get("group_outputs") != []
        or run_summary.get("group_skips") != []
        or not isinstance(run_summary.get("results"), list)
        or len(run_summary["results"]) != 1
        or run_summary["results"][0].get("schedule", {}).get("id") != SCHEDULE_ID
    ):
        raise ValueError("visual capture did not produce one passing card-house result")

    provenance = {
        "capture_argv": capture_argv,
        "capture_returncode": capture_process.returncode,
        "run_summary_path": "run-summary.json",
        "run_summary_sha256": sha256(bundle / "run-summary.json"),
        "run_summary_validated": True,
        "capture_stdout_sha256": sha256(capture_stdout),
        "capture_stdout_ends_with_run_summary": True,
        "capture_stderr_sha256": sha256(capture_stderr),
        "contract_query_argv": _contract_argv(demo),
        "contract_query_returncode": contract_after_process.returncode,
        "contract_path": "contract.json",
        "contract_sha256": sha256(bundle / "contract.json"),
        "contract_payload_sha256_before": _payload_sha256(contract_before),
        "contract_payload_sha256_after": _payload_sha256(contract_after),
        "contract_stdout_sha256_before": hashlib.sha256(
            contract_before_process.stdout.encode("utf-8")
        ).hexdigest(),
        "contract_stdout_sha256_after": sha256(contract_stdout),
        "contract_stdout_byte_identical_before_after": True,
        "contract_stderr_sha256_before": hashlib.sha256(
            contract_before_process.stderr.encode("utf-8")
        ).hexdigest(),
        "contract_stderr_sha256_after": sha256(contract_stderr),
        "source_identity_before": source_identity_before,
        "source_identity_after": source_identity_after,
    }
    write_json(bundle / "capture-provenance.json", provenance)
    return provenance, contract_after


def _finalize_transaction(
    args: argparse.Namespace,
    *,
    bundle: Path,
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
    source_identity_before: dict[str, Any],
    commit: Callable[[], None],
) -> dict[str, Any]:
    _clean_finalizer_outputs(bundle)
    if not args.reuse_current_capture:
        _clean_capture_outputs(bundle)
        (bundle / "manual-inspection.json").unlink(missing_ok=True)
        provenance, contract = _write_fresh_capture(
            args,
            bundle=bundle,
            demo=demo,
            runner=runner,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
            source_identity_before=source_identity_before,
        )
    else:
        provenance = read_json(bundle / "capture-provenance.json")
        contract = read_json(bundle / "contract.json")

    live_contract_process, live_contract = _query_contract(
        demo, timeout=args.contract_timeout
    )
    if (
        live_contract != contract
        or hashlib.sha256(live_contract_process.stdout.encode("utf-8")).hexdigest()
        != provenance["contract_stdout_sha256_after"]
        or hashlib.sha256(live_contract_process.stderr.encode("utf-8")).hexdigest()
        != provenance["contract_stderr_sha256_after"]
    ):
        raise ValueError("live card-house contract differs from captured contract")

    if not args.reuse_current_capture:
        verification_process, verification = _run_verification(
            bundle=bundle,
            demo=demo,
            runner=runner,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
            timeout=args.verification_timeout,
        )
        validate_capture_bundle(
            bundle,
            demo=demo,
            verification=verification,
        )
        staging_frame = bundle / CAPTURE_DIR / "frames/step_000000.png"
        provenance.update(
            {
                "verification_argv": _verification_argv(
                    python, runner, demo, bundle, ffmpeg, ffprobe
                ),
                "verification_returncode": verification_process.returncode,
                "verification_path": "verification.json",
                "verification_sha256": sha256(bundle / "verification.json"),
                "verification_payload_sha256": _payload_sha256(verification),
                "verification_stdout_sha256": hashlib.sha256(
                    verification_process.stdout.encode("utf-8")
                ).hexdigest(),
                "verification_stderr_sha256": hashlib.sha256(
                    verification_process.stderr.encode("utf-8")
                ).hexdigest(),
                "durable_still_path": DURABLE_STILL,
                "durable_still_sha256": sha256(staging_frame),
                "staging_pruned": True,
            }
        )
        write_json(bundle / "capture-provenance.json", provenance)
        _promote_and_prune_capture_staging(bundle)
        capture = validate_capture_bundle(
            bundle,
            demo=demo,
            verification=verification,
        )
        _validate_capture_provenance(
            provenance,
            bundle=bundle,
            demo=demo,
            runner=runner,
            ffmpeg=ffmpeg,
            ffprobe=ffprobe,
            python=python,
        )
        write_json(bundle / "manual-inspection.json", _manual_template(bundle))
        _validate_bundle_paths(bundle, complete=False, allow_missing_manual=False)
        commit()
        raise ValueError(
            "fresh card-house step-zero capture and automated verification are "
            "complete; inspect the two hash-bound artifacts, set every "
            "manual-inspection.json verdict plus manual_inspected/pass to true, "
            "add nonempty observations, then rerun with --reuse-current-capture"
        )

    _validate_capture_provenance(
        provenance,
        bundle=bundle,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    verification = read_json(bundle / "verification.json")
    capture = validate_capture_bundle(
        bundle,
        demo=demo,
        verification=verification,
    )
    _validate_bundle_paths(bundle, complete=False, allow_missing_manual=False)
    manual = validate_manual_inspection(bundle)
    invocations = {
        "schema_version": INVOCATIONS_SCHEMA_VERSION,
        "contract_query": {
            "argv": _contract_argv(demo),
            "returncode": live_contract_process.returncode,
            "stdout_sha256": provenance["contract_stdout_sha256_after"],
            "stderr_sha256": provenance["contract_stderr_sha256_after"],
            "payload_path": "contract.json",
            "payload_sha256": sha256(bundle / "contract.json"),
        },
        "capture": {
            "argv": _capture_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
            "returncode": 0,
            "stdout_sha256": provenance["capture_stdout_sha256"],
            "stderr_sha256": provenance["capture_stderr_sha256"],
            "run_summary_path": "run-summary.json",
            "run_summary_sha256": sha256(bundle / "run-summary.json"),
        },
        "capture_verification": {
            "argv": _verification_argv(python, runner, demo, bundle, ffmpeg, ffprobe),
            "returncode": provenance["verification_returncode"],
            "stdout_path": "verification.json",
            "stdout_sha256": sha256(bundle / "verification.json"),
            "raw_stdout_sha256": provenance["verification_stdout_sha256"],
            "stderr_sha256": provenance["verification_stderr_sha256"],
        },
    }
    write_json(bundle / "invocations.json", invocations)
    report = _report_markdown(
        contract=contract,
        capture=capture,
        manual=manual,
    )
    (bundle / "REPORT.md").write_text(report, encoding="utf-8")

    files_before_index, _ = _bundle_files_and_directories(bundle)
    expected_before_index = EXPECTED_FINAL_PATHS - INDEX_EXCLUSIONS
    if files_before_index != expected_before_index:
        raise ValueError(
            "card-house bundle is not ready for indexing: "
            f"missing={sorted(expected_before_index - files_before_index)}, "
            f"extra={sorted(files_before_index - expected_before_index)}"
        )
    index = artifact_index(bundle)
    write_json(bundle / "artifact-index.json", index)

    source_identity_after = _source_identity(
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    if source_identity_after != source_identity_before:
        raise ValueError(
            "card-house source/binary/runtime identity changed during finalization"
        )
    metadata = {
        **EXPECTED_METADATA_FLAGS,
        "evidence_date": args.evidence_date,
        "claim_scope": CLAIM_SCOPE,
        "claim_boundary": CLAIM_BOUNDARY,
        "configuration_contract": {
            "path": "contract.json",
            "sha256": sha256(bundle / "contract.json"),
            "payload_sha256": _payload_sha256(contract),
            "schema_version": contract["schema_version"],
            "kind": contract["kind"],
            "validated": True,
        },
        "capture_summary": capture,
        "capture_provenance": provenance,
        "manual_inspection": {
            "path": "manual-inspection.json",
            "sha256": sha256(bundle / "manual-inspection.json"),
            "pass": True,
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
        "report": {
            "path": "REPORT.md",
            "sha256": sha256(bundle / "REPORT.md"),
        },
        "artifact_index": {
            "path": "artifact-index.json",
            "sha256": sha256(bundle / "artifact-index.json"),
            "artifact_count": index["artifact_count"],
            "excluded": index["excluded"],
        },
        "source_identity": source_identity_after,
    }
    write_json(bundle / "metadata.json", metadata)
    try:
        result = verify_finalized(
            bundle,
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
    if args.reuse_current_capture:
        _validate_bundle_paths(
            bundle,
            complete=False,
            allow_missing_manual=False,
        )
    else:
        files, _ = _bundle_files_and_directories(bundle)
        if files:
            raise ValueError(
                "fresh card-house capture requires an empty bundle; use "
                "--reuse-current-capture for a completed capture"
            )
    demo = _require_file(args.demo, "DART demo binary")
    runner = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")
    python = _require_file(args.python, "Python interpreter")
    source_identity_before = _source_identity(
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
    demo: Path,
    runner: Path,
    ffmpeg: Path,
    ffprobe: Path,
    python: Path,
) -> dict[str, Any]:
    bundle = _require_bundle_root(bundle, create=False)
    _validate_bundle_paths(bundle, complete=True)
    demo = _require_file(demo, "DART demo binary")
    runner = _require_file(runner, "visual evidence runner")
    ffmpeg = _require_file(ffmpeg, "ffmpeg")
    ffprobe = _require_file(ffprobe, "ffprobe")
    python = _require_file(python, "Python interpreter")
    metadata = read_json(bundle / "metadata.json")
    if set(metadata) != EXPECTED_FINAL_METADATA_KEYS:
        raise ValueError("malformed card-house finalized metadata payload")
    if any(
        metadata.get(key) != value for key, value in EXPECTED_METADATA_FLAGS.items()
    ):
        raise ValueError("card-house finalized metadata claim flags changed")
    if (
        not isinstance(metadata.get("evidence_date"), str)
        or not metadata["evidence_date"]
        or metadata.get("claim_scope") != CLAIM_SCOPE
        or metadata.get("claim_boundary") != CLAIM_BOUNDARY
    ):
        raise ValueError("card-house finalized claim text changed")

    contract = read_json(bundle / "contract.json")
    validate_configuration_contract(contract)
    expected_contract_binding = {
        "path": "contract.json",
        "sha256": sha256(bundle / "contract.json"),
        "payload_sha256": _payload_sha256(contract),
        "schema_version": CONTRACT_SCHEMA_VERSION,
        "kind": "configuration_only",
        "validated": True,
    }
    if metadata.get("configuration_contract") != expected_contract_binding:
        raise ValueError("card-house metadata contract binding is stale")

    capture = validate_capture_bundle(bundle, demo=demo)
    if metadata.get("capture_summary") != capture:
        raise ValueError("card-house metadata capture summary is stale")
    provenance = read_json(bundle / "capture-provenance.json")
    if metadata.get("capture_provenance") != provenance:
        raise ValueError("card-house metadata capture provenance is stale")
    _validate_capture_provenance(
        provenance,
        bundle=bundle,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    manual = validate_manual_inspection(bundle)
    if metadata.get("manual_inspection") != {
        "path": "manual-inspection.json",
        "sha256": sha256(bundle / "manual-inspection.json"),
        "pass": manual["pass"],
    }:
        raise ValueError("card-house metadata manual inspection is stale")
    _validate_invocations(
        bundle,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
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
            raise ValueError(f"card-house metadata {key} binding is stale")
    expected_report = _report_markdown(
        contract=contract,
        capture=capture,
        manual=manual,
    )
    if (bundle / "REPORT.md").read_text(encoding="utf-8") != expected_report:
        raise ValueError("card-house REPORT.md is stale")
    index = read_json(bundle / "artifact-index.json")
    validate_artifact_index(bundle, index)
    if metadata.get("artifact_index") != {
        "path": "artifact-index.json",
        "sha256": sha256(bundle / "artifact-index.json"),
        "artifact_count": index["artifact_count"],
        "excluded": index["excluded"],
    }:
        raise ValueError("card-house metadata artifact index is stale")
    sources = _source_identity(
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    if metadata.get("source_identity") != sources:
        raise ValueError("card-house source/binary/runtime identity changed")
    return {
        "schema_version": SCHEMA_VERSION,
        "status": metadata["status"],
        "pass": True,
        "artifact_count": index["artifact_count"],
        "configuration_port_valid": True,
        "trajectory_valid": False,
        "solver_valid": False,
        "physical_outcome_valid": False,
        "fig06_parity": False,
        "video06_parity": False,
        "paper_timing_valid": False,
        "contract_sha256": sha256(bundle / "contract.json"),
        "manual_inspection_sha256": sha256(bundle / "manual-inspection.json"),
        "artifact_index_sha256": sha256(bundle / "artifact-index.json"),
        "metadata_sha256": sha256(bundle / "metadata.json"),
    }


def verify_only(args: argparse.Namespace) -> dict[str, Any]:
    bundle = _require_bundle_root(args.bundle, create=False)
    demo = _require_file(args.demo, "DART demo binary")
    runner = _require_file(args.runner, "visual evidence runner")
    ffmpeg = _require_file(args.ffmpeg, "ffmpeg")
    ffprobe = _require_file(args.ffprobe, "ffprobe")
    python = _require_file(args.python, "Python interpreter")
    result = verify_finalized(
        bundle,
        demo=demo,
        runner=runner,
        ffmpeg=ffmpeg,
        ffprobe=ffprobe,
        python=python,
    )
    contract_process, contract = _query_contract(demo, timeout=args.contract_timeout)
    provenance = read_json(bundle / "capture-provenance.json")
    if (
        contract != read_json(bundle / "contract.json")
        or hashlib.sha256(contract_process.stdout.encode("utf-8")).hexdigest()
        != provenance["contract_stdout_sha256_after"]
        or hashlib.sha256(contract_process.stderr.encode("utf-8")).hexdigest()
        != provenance["contract_stderr_sha256_after"]
    ):
        raise ValueError("live card-house contract replay changed")
    return {
        **result,
        "live_contract_replay": True,
        "durable_capture_record_reverification": True,
        "ignored_staging_required": False,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--demo", type=Path, default=DEFAULT_DEMO)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--ffmpeg", type=Path, default=DEFAULT_FFMPEG)
    parser.add_argument("--ffprobe", type=Path, default=DEFAULT_FFPROBE)
    parser.add_argument("--python", type=Path, default=DEFAULT_PYTHON)
    parser.add_argument("--capture-timeout", type=float, default=600.0)
    parser.add_argument("--contract-timeout", type=float, default=30.0)
    parser.add_argument("--verification-timeout", type=float, default=300.0)
    parser.add_argument("--evidence-date", default="2026-07-18")
    parser.add_argument(
        "--reuse-current-capture",
        action="store_true",
        help=(
            "finalize the captured step-zero bundle after completing its "
            "hash-bound manual-inspection.json template"
        ),
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help=(
            "revalidate the sealed bundle, replay the configuration contract, "
            "and verify the durable still against the original capture record "
            "without requiring ignored runner staging"
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
