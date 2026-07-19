#!/usr/bin/env python3
"""Seal and verify the pinned current-author masonry-arch diagnostic.

This bundle is deliberately fail-closed.  It preserves the completed 500-frame
author-source run and a lossless projection of every claim-bearing solver
history field, while binding the much larger raw history by exact size and
digest.  The run is a scientific negative because most substeps carry a
negative author convergence flag.  Integrity of the run is not evidence of
paper, historical-invocation, or DART dynamics parity.
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import math
import os
import shutil
import stat
import subprocess
import sys
import uuid
import zipfile
import zlib
from pathlib import Path, PurePosixPath
from typing import IO, Any, Callable, Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_REPO = Path("/tmp/fbf-sca-2026-author")
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "author_masonry_arch_reference_v1"
)

SCHEMA_VERSION = "dart.fbf_author_masonry_arch_reference/v1"
SUMMARY_SCHEMA_VERSION = "dart.fbf_author_masonry_arch_summary/v1"
INDEX_SCHEMA_VERSION = "dart.fbf_author_masonry_arch_artifact_index/v1"
VERIFICATION_SCHEMA_VERSION = "dart.fbf_author_masonry_arch_reference_validation/v1"
HISTORY_PROJECTION_SCHEMA_VERSION = (
    "dart.fbf_author_masonry_arch_history_claim_projection/v1"
)

SOURCE_REPOSITORY = "https://github.com/matthcsong/fbf-sca-2026.git"
SOURCE_COMMIT = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
SOURCE_TREE = "ffcdafb61adeda2239c8366d054b548b50d26685"
SOURCE_SHORT_COMMIT = "b3f3c5c"
RUNNER_PATH = "paper_examples/masonry-arch/run.py"
RUNNER_SHA256 = "7e9158240267bb0ec1d0316b1badd4f3c8e1cd10270322de2e205cfea96f6f73"
RUNNER_GIT_BLOB = "35a052d7ef0975e7c828c9678d163054dfbb3ef2"
REQUIREMENTS_PATH = "requirements.txt"
REQUIREMENTS_SHA256 = "436d6a280e62f0cf5b670bf0475cbed6e95fa87e2fb011bc30334355a3ed9688"
REQUIREMENTS_GIT_BLOB = "f42eadca6be4869162675340427315aca01bd634"

RUN_ID = "20260719T113826Z"
SOURCE_RUN_DIRECTORY = f"paper_examples/masonry-arch/results/{RUN_ID}"
RUN_PREFIX = f"runs/{RUN_ID}"
METADATA_PATH = f"{RUN_PREFIX}/metadata.json"
RESULT_PATH = f"{RUN_PREFIX}/fbf/result.json"
TRAJECTORY_PATH = f"{RUN_PREFIX}/fbf/trajectory.npz"
HISTORY_PATH = f"{RUN_PREFIX}/fbf/history-claim-projection.json.gz"

INVOCATION = (
    "/tmp/fbf-author-venv/bin/python",
    "paper_examples/masonry-arch/run.py",
    "--solvers",
    "fbf",
    "--frames",
    "500",
    "--drop-frame",
    "400",
    "--device",
    "cpu",
    "--profile",
)

TOTAL_FRAMES = 500
SOURCE_DEFAULT_FRAMES = 400
DROP_FRAME = 400
SUBSTEPS = 4
TOTAL_SUBSTEPS = TOTAL_FRAMES * SUBSTEPS
RELEASE_SUBSTEP = DROP_FRAME * SUBSTEPS
TERMINATION_TOLERANCE = 1.0e-6
MAX_OUTER = 200
JSON_CHUNK_CHARS = 1024 * 1024
MAX_JSON_VALUE_CHARS = 32 * 1024 * 1024
GZIP_CHUNK_BYTES = 1024 * 1024
MAX_NPZ_MEMBER_BYTES = 2 * 1024 * 1024
MAX_NPZ_TOTAL_BYTES = 8 * 1024 * 1024

SOURCE_ARTIFACTS = (
    {
        "source_path": f"{SOURCE_RUN_DIRECTORY}/metadata.json",
        "bundle_path": METADATA_PATH,
        "bytes": 566,
        "sha256": ("ac135b4320c4f771db4fd9126e634ff475d932fdfd7912d335bf3d9ff28230fe"),
        "encoding": "raw",
    },
    {
        "source_path": f"{SOURCE_RUN_DIRECTORY}/fbf/result.json",
        "bundle_path": RESULT_PATH,
        "bytes": 13584,
        "sha256": ("1edb551142eec1d6c58f67e45c8e73fe95d4e22c0c78f477fed507c49d5c8eb4"),
        "encoding": "raw",
    },
    {
        "source_path": f"{SOURCE_RUN_DIRECTORY}/fbf/trajectory.npz",
        "bundle_path": TRAJECTORY_PATH,
        "bytes": 9222,
        "sha256": ("250ba12a329b41d4710929faaa7af2b5a3bdd2c3f28d6f1dcfa3b59cc22d570d"),
        "encoding": "raw",
    },
    {
        "source_path": f"{SOURCE_RUN_DIRECTORY}/fbf/history.json",
        "bundle_path": HISTORY_PATH,
        "encoding": "claim-projection-json-gzip-n9-mtime0",
        "source_identity": {
            "bytes": 382753953,
            "sha256": (
                "cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1"
            ),
            "retained": False,
        },
        "projection": {
            "schema_version": HISTORY_PROJECTION_SCHEMA_VERSION,
            "stored_bytes": 8080187,
            "stored_sha256": (
                "8a19be46feae8bfbbfc57f3aff194bc18a9fa695f286cc4d13f3c819f9dc701b"
            ),
            "decompressed_bytes": 28606169,
            "decompressed_sha256": (
                "8c9d245b5820950b36fe9b319cde0c256ab1cb404d9dfa1fc7af5fb53a9e0f98"
            ),
            "step_fields": [
                "step_idx",
                "num_contacts",
                "outer_iters",
                "converged",
                "initial_residual",
                "final_residual",
                "warmup",
                "outer",
            ],
            "outer_fields": ["k", "residual", "r_coulomb"],
        },
    },
)
SOURCE_ARTIFACT_BY_BUNDLE_PATH = {
    item["bundle_path"]: item for item in SOURCE_ARTIFACTS
}

OMITTED_SOURCE_ARTIFACT = {
    "source_path": f"{SOURCE_RUN_DIRECTORY}/sweep_results.json",
    "bytes": 14714,
    "sha256": "610ce014544e549e144bea77ebab50011b4caedf070ebf132dbb8ce203ad7442",
    "reason": "semantic duplicate of the retained single-case result.json",
}

INSTALLED_DISTRIBUTIONS = (
    "GitPython==3.1.46",
    "PyOpenGL==3.1.10",
    "PyYAML==6.0.3",
    "Pygments==2.20.0",
    "absl-py==2.4.0",
    "alphashape==1.3.1",
    "cbor2==5.9.0",
    "certifi==2026.2.25",
    "cffi==2.0.0",
    "charset-normalizer==3.4.7",
    "clarabel==0.11.1",
    "click-log==0.4.0",
    "click==8.3.2",
    "coacd==1.0.9",
    "contourpy==1.3.3",
    "cycler==0.12.1",
    "etils==1.14.0",
    "fast_simplification==0.1.13",
    "fonttools==4.62.1",
    "fsspec==2026.3.0",
    "gitdb==4.0.12",
    "glfw==2.10.0",
    "idna==3.11",
    "ijson==3.5.0",
    "imgui-bundle==1.92.601",
    "iniconfig==2.3.0",
    "kiwisolver==1.5.0",
    "manifold3d==3.4.1",
    "mapbox_earcut==2.0.0",
    "markdown-it-py==4.0.0",
    "matplotlib==3.10.8",
    "mdurl==0.1.2",
    "meshio==5.3.5",
    "mujoco-warp==3.5.0.2",
    "mujoco==3.5.0",
    "networkx==3.6.1",
    "newton-actuators==0.1.0",
    "newton-usd-schemas==0.1.0",
    "newton==1.0.0",
    "numpy==2.4.4",
    "packaging==26.0",
    "pandas==3.0.2",
    "pillow==12.2.0",
    "pluggy==1.6.0",
    "pycollada==0.9.3",
    "pycparser==3.0",
    "pyglet==2.1.14",
    "pyparsing==3.3.2",
    "pytest==9.0.3",
    "python-dateutil==2.9.0.post0",
    "requests==2.33.1",
    "resolve-robotics-uri-py==0.4.1",
    "rich==14.3.3",
    "rtree==1.4.1",
    "scipy==1.17.1",
    "scs==3.2.11",
    "shapely==2.1.2",
    "six==1.17.0",
    "smmap==5.0.3",
    "trimesh==4.11.5",
    "typing_extensions==4.15.0",
    "urllib3==2.6.3",
    "usd-core==26.3",
    "warp-lang==1.12.1",
    "zipp==3.23.0",
)

EXPECTED_ENVIRONMENT = {
    "invoked_interpreter": "/tmp/fbf-author-venv/bin/python",
    "resolved_interpreter": (
        "/home/js/.local/share/uv/python/"
        "cpython-3.11.15-linux-x86_64-gnu/bin/python3.11"
    ),
    "resolved_interpreter_sha256": (
        "6759dcc6aa5a3fe3377349092f92f6700f1f79017294c1d0085009eb8802cb1e"
    ),
    "python_implementation": "CPython",
    "python_version": "3.11.15 (main, May 10 2026, 19:28:18) [Clang 22.1.3 ]",
    "platform": "Linux-7.0.0-27-generic-x86_64-with-glibc2.43",
    "machine": "x86_64",
    "installed_distribution_count": 65,
    "installed_distributions": list(INSTALLED_DISTRIBUTIONS),
    "installed_distributions_sha256": (
        "d86b1f09415cdd3fa849dd468284068f2889b1b7bb47397ff20b99a424d099b3"
    ),
    "requirements_exact_match": True,
    "observation_timing": "post_run_only_at_finalization",
}

EXPECTED_METADATA = {
    "experiment": "masonry_arch_drop",
    "mu": 0.8,
    "num_stones": 25,
    "num_cubes": 3,
    "cube_half_size": 1.5,
    "cube_density": 2000.0,
    "stone_density": 2000.0,
    "drop_height": 10.0,
    "scale": 1.0,
    "num_frames": TOTAL_FRAMES,
    "drop_frame": DROP_FRAME,
    "dt": 1.0 / 60.0,
    "substeps": SUBSTEPS,
    "sub_dt": 1.0 / 240.0,
    "T_seconds": 25.0 / 3.0,
    "gap": 0.005,
    "mesh_dir": (
        "/tmp/fbf-sca-2026-author/paper_examples/masonry-arch/"
        "../../meshes/arch/num_stones=25"
    ),
    "solvers": ["fbf"],
    "device": "cpu",
    "timestamp": RUN_ID,
}

EXPECTED_CONFIG = {
    "gamma": None,
    "max_outer": MAX_OUTER,
    "outer_tol": TERMINATION_TOLERANCE,
    "residual_check_interval": 1,
    "inner_max_iter": 1000,
    "inner_tol": TERMINATION_TOLERANCE,
    "inner_step_size": None,
    "inner_solver": "block_gs",
    "inner_gs_sweeps": 30,
    "max_contacts": 4096,
    "baumgarte_erp": 0.0,
    "project_after_correction": True,
    "warm_start": True,
    "warm_start_match_radius": 0.02,
    "warm_start_normal_cos": 0.9,
    "warm_start_max_age": 3,
    "gamma_min": 1.0e-6,
    "gamma_max": 1.0e6,
    "gamma_c": 15.5,
    "adaptive_gamma": True,
    "armijo_rho_high": 0.9,
    "armijo_shrink": 0.7,
    "armijo_grow": 1.4285714285714286,
    "armijo_skip_threshold": 1.0e-10,
    "armijo_max_backtracks": 8,
    "plateau_patience": 0,
    "plateau_rtol": 0.01,
    "warm_start_gamma_threshold": 0.0001,
    "warm_start_gamma_cap": 10000.0,
    "history_path": (
        "/tmp/fbf-sca-2026-author/paper_examples/masonry-arch/results/"
        f"{RUN_ID}/fbf/history.json"
    ),
    "termination_residual": "coulomb_rel",
    "termination_tol": TERMINATION_TOLERANCE,
    "per_contact_diag_dir": None,
    "per_contact_diag_topk": 20,
    "dump_step_state_idx": None,
    "dump_step_state_path": None,
}

EXPECTED_RESULT_STATIC = {
    "solver": "fbf",
    "mu": 0.8,
    "num_stones": 25,
    "num_cubes": 3,
    "cube_half_size": 1.5,
    "cube_density": 2000.0,
    "drop_height": 10.0,
    "drop_z": 75.27337646484375,
    "drop_frame": DROP_FRAME,
    "scale": 1.0,
    "dt": 1.0 / 60.0,
    "substeps": SUBSTEPS,
    "sub_dt": 1.0 / 240.0,
    "num_frames": TOTAL_FRAMES,
    "keystone_idx": 12,
    "max_height_change": 1.7435989379882812,
    "keystone_z_init": 61.92774963378906,
    "keystone_z_final": 60.18415069580078,
    "avg_step_time_ms": 1670.4299479899346,
    "usd": None,
    "T_seconds": 25.0 / 3.0,
}
RESULT_KEYS = set(EXPECTED_RESULT_STATIC) | {"step_times_ms", "config"}

EXPECTED_HISTORY_META = {
    "solver": "FBF",
    "inner_solver": "block_gs",
    "num_bodies": 28,
    "config": {
        **EXPECTED_CONFIG,
        "history_path": (
            f"paper_examples/masonry-arch/results/{RUN_ID}/fbf/history.json"
        ),
    },
    "schema_version": 2,
    "bench": {
        "scene": "masonry-arch",
        "scene_params": {
            "mu": 0.8,
            "num_stones": 25,
            "num_cubes": 3,
            "drop_frame": DROP_FRAME,
            "drop_height": 10.0,
            "scale": 1.0,
        },
        "warmup_steps": 0,
        "total_steps": TOTAL_SUBSTEPS,
        "git_sha": SOURCE_SHORT_COMMIT,
    },
    "dt": 1.0 / 240.0,
}

STEP_KEYS = {
    "step_idx",
    "num_contacts",
    "W_lambda_max",
    "gamma_init",
    "warm_started",
    "converged",
    "outer_iters",
    "final_residual",
    "t_total_ms",
    "t_collide_ms",
    "t_jacobian_ms",
    "t_power_iter_ms",
    "t_recover_ms",
    "t_warm_start_ms",
    "t_save_warm_cache_ms",
    "t_init_residual_ms",
    "t_integrate_free_ms",
    "gamma_final",
    "warm_start_matched",
    "warm_start_total",
    "initial_residual",
    "warmup",
    "t_total_gpu_ms",
    "t_collide_gpu_ms",
    "t_jacobian_gpu_ms",
    "t_power_iter_gpu_ms",
    "t_recover_gpu_ms",
    "t_warm_start_gpu_ms",
    "t_save_warm_cache_gpu_ms",
    "t_init_residual_gpu_ms",
    "t_integrate_free_gpu_ms",
    "t_unattributed_gpu_ms",
    "outer",
}
OUTER_KEYS = {
    "k",
    "residual",
    "inner_iters",
    "inner_last_change",
    "gamma",
    "gamma_event",
    "rho",
    "t_fwd_bwd_ms",
    "t_correction_ms",
    "t_fwd_eval_ms",
    "t_residual_check_ms",
    "backtracks",
    "gamma_start",
    "r_primal",
    "r_dual",
    "r_gap",
    "r_kkt",
    "eps_vel",
    "eps_force",
    "eps_gap",
    "r_coulomb",
    "t_fwd_eval_gpu_ms",
    "t_fwd_bwd_gpu_ms",
    "t_correction_gpu_ms",
    "t_residual_check_gpu_ms",
}
PROJECTED_STEP_KEYS = {
    "step_idx",
    "num_contacts",
    "outer_iters",
    "converged",
    "initial_residual",
    "final_residual",
    "warmup",
    "outer",
}
PROJECTED_OUTER_KEYS = {"k", "residual", "r_coulomb"}

EXPECTED_HISTORY_METRICS = {
    "total_substeps": 2000,
    "author_converged_flag_count": 157,
    "author_nonconverged_flag_count": 1843,
    "initial_natural_residual_shortcut_converged_count": 40,
    "configured_coulomb_rel_outer_gate_converged_count": 117,
    "pre_release": {
        "substeps": 1600,
        "converged": 142,
        "nonconverged": 1458,
        "initial_natural_residual_shortcut_converged": 40,
        "configured_coulomb_rel_outer_gate_converged": 102,
    },
    "post_release": {
        "substeps": 400,
        "converged": 15,
        "nonconverged": 385,
        "initial_natural_residual_shortcut_converged": 0,
        "configured_coulomb_rel_outer_gate_converged": 15,
    },
    "natural_final_residual_at_or_below_tolerance_count": 47,
    "convergence_natural_threshold_cross_table": {
        "converged_and_natural_at_or_below": 46,
        "converged_and_natural_above": 111,
        "nonconverged_and_natural_at_or_below": 1,
        "nonconverged_and_natural_above": 1842,
    },
    "first_nonconverged_step_idx": 53,
    "outer_cap_hit_count": 1843,
    "zero_outer_iteration_count": 40,
    "contact_positive_step_count": 2000,
    "warmup_step_count": 0,
    "max_natural_final_residual": {"step_idx": 226, "value": 4.1130565788445415},
    "release_substep": {
        "step_idx": 1600,
        "num_contacts": 100,
        "converged": False,
        "outer_iters": 200,
        "natural_final_residual": 0.017456069692858667,
    },
    "first_inferred_contact_count_increase_after_release": {
        "step_idx": 1944,
        "previous_count": 100,
        "count": 102,
    },
    "peak_contact_count": {"step_idx": 1947, "count": 109},
    "final_substep": {
        "step_idx": 1999,
        "num_contacts": 108,
        "converged": False,
        "outer_iters": 200,
        "natural_final_residual": 0.5161195175386001,
    },
    "contact_pair_identity_available": False,
}

NONCLAIMS = (
    "not_a_historical_or_paper_invocation",
    "not_the_source_default_400_frame_no_release_protocol",
    "not_an_all_substeps_solver_valid_result",
    "not_dart_equivalence_or_dynamics_parity",
    "not_cross_solver_parity",
    "not_timing_or_real_time_evidence",
    "not_repeatability_or_statistical_evidence",
    "contact_count_change_is_not_definitive_cube_arch_pair_contact",
    "not_visual_renderer_or_approved_golden_evidence",
    "not_a_contemporaneous_pre_and_post_runtime_attestation",
    "raw_source_history_not_retained_beyond_claim_projection",
)

CORE_FILES = {
    "REPORT.md",
    "manifest.json",
    "summary.json",
    METADATA_PATH,
    RESULT_PATH,
    TRAJECTORY_PATH,
    HISTORY_PATH,
}
INDEX_EXCLUSIONS = {"artifact-index.json", "verification.json"}
EXPECTED_FILES = CORE_FILES | INDEX_EXCLUSIONS
EXPECTED_DIRECTORIES = {
    "runs",
    RUN_PREFIX,
    f"{RUN_PREFIX}/fbf",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _reject_nonfinite_json(token: str) -> None:
    raise ValueError(f"non-finite JSON constant: {token}")


def read_json(path: Path) -> Any:
    try:
        return json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_nonfinite_json,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{path}: invalid UTF-8 JSON") from error


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def exact_json_equal(actual: Any, expected: Any) -> bool:
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            exact_json_equal(actual[key], value) for key, value in expected.items()
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            exact_json_equal(left, right)
            for left, right in zip(actual, expected, strict=True)
        )
    return actual == expected


def _require_exact(payload: Any, expected: Any, *, label: str) -> Any:
    if not exact_json_equal(payload, expected):
        raise ValueError(f"{label} changed")
    return payload


def _require_finite_number(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label}: expected a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label}: expected a finite number")
    return result


def _initial_natural_residual_shortcut_passes(value: Any) -> bool:
    residual = _require_finite_number(
        value, label="initial natural-residual shortcut value"
    )
    return residual < TERMINATION_TOLERANCE


def _configured_coulomb_rel_gate_passes(value: Any) -> bool:
    residual = _require_finite_number(value, label="configured coulomb_rel value")
    return 0.0 <= residual < TERMINATION_TOLERANCE


def _safe_relative_path(value: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if (
        not value
        or path.is_absolute()
        or "\\" in value
        or any(part in {"", ".", ".."} for part in path.parts)
        or path.as_posix() != value
    ):
        raise ValueError(f"unsafe or noncanonical bundle path: {value!r}")
    return path


def reject_symlink_path_components(path: Path) -> None:
    absolute = Path(os.path.abspath(path.expanduser()))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if current.is_symlink():
            raise ValueError(f"path contains symlink: {current}")


def _remove_path(path: Path) -> None:
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError:
        return
    if stat.S_ISDIR(mode) and not stat.S_ISLNK(mode):
        shutil.rmtree(path)
    else:
        path.unlink()


def _bundle_entries(root: Path) -> tuple[set[str], set[str]]:
    if root.is_symlink() or not root.is_dir():
        raise ValueError("author masonry-arch bundle must be a regular directory")
    files: set[str] = set()
    directories: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"author masonry-arch bundle contains symlink: {relative}")
        if stat.S_ISREG(mode):
            files.add(relative)
        elif stat.S_ISDIR(mode):
            directories.add(relative)
        else:
            raise ValueError(
                "author masonry-arch bundle contains non-regular entry: " f"{relative}"
            )
    return files, directories


def validate_membership(bundle: Path) -> None:
    reject_symlink_path_components(bundle)
    files, directories = _bundle_entries(bundle)
    if files != EXPECTED_FILES:
        raise ValueError(
            "author masonry-arch bundle file membership changed: "
            f"missing={sorted(EXPECTED_FILES - files)}, "
            f"extra={sorted(files - EXPECTED_FILES)}"
        )
    if directories != EXPECTED_DIRECTORIES:
        raise ValueError(
            "author masonry-arch bundle directory membership changed: "
            f"missing={sorted(EXPECTED_DIRECTORIES - directories)}, "
            f"extra={sorted(directories - EXPECTED_DIRECTORIES)}"
        )


class _JsonCursor:
    """Incremental JSON cursor for one bounded object at a time."""

    def __init__(self, stream: IO[str], *, label: str):
        self.stream = stream
        self.label = label
        self.buffer = ""
        self.position = 0
        self.eof = False
        self.decoder = json.JSONDecoder(
            object_pairs_hook=_unique_object,
            parse_constant=_reject_nonfinite_json,
        )

    def _compact(self) -> None:
        if self.position:
            self.buffer = self.buffer[self.position :]
            self.position = 0

    def _fill(self) -> bool:
        if self.eof:
            return False
        chunk = self.stream.read(JSON_CHUNK_CHARS)
        if chunk == "":
            self.eof = True
            return False
        self.buffer += chunk
        return True

    def skip_whitespace(self) -> None:
        if self.position >= JSON_CHUNK_CHARS:
            self._compact()
        while True:
            while (
                self.position < len(self.buffer)
                and self.buffer[self.position].isspace()
            ):
                self.position += 1
            if self.position < len(self.buffer) or not self._fill():
                return

    def peek(self) -> str:
        self.skip_whitespace()
        if self.position >= len(self.buffer):
            raise ValueError(f"{self.label}: unexpected end of JSON")
        return self.buffer[self.position]

    def expect(self, token: str) -> None:
        if self.peek() != token:
            raise ValueError(f"{self.label}: expected {token!r}")
        self.position += 1

    def decode(self) -> Any:
        self.skip_whitespace()
        start = self.position
        while True:
            try:
                value, end = self.decoder.raw_decode(self.buffer, self.position)
            except json.JSONDecodeError as error:
                if self.eof:
                    raise ValueError(f"{self.label}: malformed JSON") from error
                if len(self.buffer) - start > MAX_JSON_VALUE_CHARS:
                    raise ValueError(f"{self.label}: JSON value exceeds size bound")
                self._compact()
                start = 0
                self._fill()
                continue
            self.position = end
            return value

    def finish(self) -> None:
        self.skip_whitespace()
        while not self.eof:
            if self.position < len(self.buffer):
                raise ValueError(f"{self.label}: trailing JSON data")
            self._compact()
            self._fill()
            self.skip_whitespace()
        if self.position != len(self.buffer):
            raise ValueError(f"{self.label}: trailing JSON data")


def _validate_history_meta(meta: Any) -> None:
    _require_exact(meta, EXPECTED_HISTORY_META, label="author history metadata")


def _validate_history_convergence(step: dict[str, Any], expected_index: int) -> None:
    outer = step["outer"]
    if outer and step["final_residual"] != outer[-1]["residual"]:
        raise ValueError(f"history step {expected_index}: final residual changed")
    if outer:
        converged_from_configured_residual = _configured_coulomb_rel_gate_passes(
            outer[-1]["r_coulomb"]
        )
        if step["converged"] is not converged_from_configured_residual:
            raise ValueError(
                f"history step {expected_index}: configured convergence flag changed"
            )
    else:
        if step["final_residual"] != step["initial_residual"]:
            raise ValueError(
                f"history step {expected_index}: zero-outer residual changed"
            )
        shortcut_passes = _initial_natural_residual_shortcut_passes(
            step["initial_residual"]
        )
        if step["converged"] is not shortcut_passes:
            raise ValueError(
                f"history step {expected_index}: initial shortcut flag changed"
            )


def _validate_history_step(step: Any, expected_index: int) -> None:
    if not isinstance(step, dict) or set(step) != STEP_KEYS:
        raise ValueError(f"history step {expected_index}: key set changed")
    if type(step["step_idx"]) is not int or step["step_idx"] != expected_index:
        raise ValueError(f"history step {expected_index}: nonsequential step_idx")
    for name in (
        "num_contacts",
        "outer_iters",
        "warm_start_matched",
        "warm_start_total",
    ):
        if type(step[name]) is not int or step[name] < 0:
            raise ValueError(f"history step {expected_index}: invalid {name}")
    for name in ("warm_started", "converged", "warmup"):
        if type(step[name]) is not bool:
            raise ValueError(f"history step {expected_index}: invalid {name}")
    for name in STEP_KEYS - {
        "step_idx",
        "num_contacts",
        "outer_iters",
        "warm_start_matched",
        "warm_start_total",
        "warm_started",
        "converged",
        "warmup",
        "outer",
    }:
        _require_finite_number(
            step[name], label=f"history step {expected_index}.{name}"
        )
    outer = step["outer"]
    if not isinstance(outer, list) or len(outer) != step["outer_iters"]:
        raise ValueError(f"history step {expected_index}: outer history length changed")
    for outer_index, item in enumerate(outer):
        if not isinstance(item, dict) or set(item) != OUTER_KEYS:
            raise ValueError(
                f"history step {expected_index} outer {outer_index}: key set changed"
            )
        if type(item["k"]) is not int or item["k"] != outer_index:
            raise ValueError(
                f"history step {expected_index} outer {outer_index}: invalid k"
            )
        if type(item["inner_iters"]) is not int or item["inner_iters"] < 0:
            raise ValueError(
                f"history step {expected_index} outer {outer_index}: "
                "invalid inner_iters"
            )
        if type(item["backtracks"]) is not int or item["backtracks"] < 0:
            raise ValueError(
                f"history step {expected_index} outer {outer_index}: "
                "invalid backtracks"
            )
        if not isinstance(item["gamma_event"], str):
            raise ValueError(
                f"history step {expected_index} outer {outer_index}: "
                "invalid gamma_event"
            )
        for name in OUTER_KEYS - {"k", "inner_iters", "backtracks", "gamma_event"}:
            _require_finite_number(
                item[name],
                label=f"history step {expected_index} outer {outer_index}.{name}",
            )
    _validate_history_convergence(step, expected_index)


def _validate_projected_history_step(step: Any, expected_index: int) -> None:
    if not isinstance(step, dict) or set(step) != PROJECTED_STEP_KEYS:
        raise ValueError(f"projected history step {expected_index}: key set changed")
    if type(step["step_idx"]) is not int or step["step_idx"] != expected_index:
        raise ValueError(
            f"projected history step {expected_index}: nonsequential step_idx"
        )
    for name in ("num_contacts", "outer_iters"):
        if type(step[name]) is not int or step[name] < 0:
            raise ValueError(f"projected history step {expected_index}: invalid {name}")
    for name in ("converged", "warmup"):
        if type(step[name]) is not bool:
            raise ValueError(f"projected history step {expected_index}: invalid {name}")
    for name in ("initial_residual", "final_residual"):
        _require_finite_number(
            step[name], label=f"projected history step {expected_index}.{name}"
        )
    outer = step["outer"]
    if not isinstance(outer, list) or len(outer) != step["outer_iters"]:
        raise ValueError(
            f"projected history step {expected_index}: outer history length changed"
        )
    for outer_index, item in enumerate(outer):
        if not isinstance(item, dict) or set(item) != PROJECTED_OUTER_KEYS:
            raise ValueError(
                f"projected history step {expected_index} outer {outer_index}: "
                "key set changed"
            )
        if type(item["k"]) is not int or item["k"] != outer_index:
            raise ValueError(
                f"projected history step {expected_index} outer {outer_index}: "
                "invalid k"
            )
        for name in ("residual", "r_coulomb"):
            _require_finite_number(
                item[name],
                label=(
                    f"projected history step {expected_index} "
                    f"outer {outer_index}.{name}"
                ),
            )
    _validate_history_convergence(step, expected_index)


def _project_history_step(step: dict[str, Any]) -> dict[str, Any]:
    return {
        "step_idx": step["step_idx"],
        "num_contacts": step["num_contacts"],
        "outer_iters": step["outer_iters"],
        "converged": step["converged"],
        "initial_residual": step["initial_residual"],
        "final_residual": step["final_residual"],
        "warmup": step["warmup"],
        "outer": [
            {
                "k": item["k"],
                "residual": item["residual"],
                "r_coulomb": item["r_coulomb"],
            }
            for item in step["outer"]
        ],
    }


def summarize_history(
    stream: IO[str],
    *,
    label: str,
    projected: bool = False,
    projection_stream: IO[str] | None = None,
) -> dict[str, Any]:
    cursor = _JsonCursor(stream, label=label)
    cursor.expect("{")
    if projected:
        if cursor.decode() != "schema_version":
            raise ValueError(f"{label}: first root key must be 'schema_version'")
        cursor.expect(":")
        _require_exact(
            cursor.decode(),
            HISTORY_PROJECTION_SCHEMA_VERSION,
            label="history projection schema",
        )
        cursor.expect(",")
        if cursor.decode() != "source_history":
            raise ValueError(f"{label}: second root key must be 'source_history'")
        cursor.expect(":")
        source_record = SOURCE_ARTIFACT_BY_BUNDLE_PATH[HISTORY_PATH]
        source_identity = source_record["source_identity"]
        _require_exact(
            cursor.decode(),
            {
                "bytes": source_identity["bytes"],
                "sha256": source_identity["sha256"],
            },
            label="history projection source identity",
        )
        cursor.expect(",")
    if cursor.decode() != "meta":
        ordinal = "third" if projected else "first"
        raise ValueError(f"{label}: {ordinal} root key must be 'meta'")
    cursor.expect(":")
    meta = cursor.decode()
    _validate_history_meta(meta)
    cursor.expect(",")
    if cursor.decode() != "steps":
        ordinal = "fourth" if projected else "second"
        raise ValueError(f"{label}: {ordinal} root key must be 'steps'")
    cursor.expect(":")
    cursor.expect("[")

    if projection_stream is not None:
        if projected:
            raise ValueError("cannot project an already projected history")
        source_record = SOURCE_ARTIFACT_BY_BUNDLE_PATH[HISTORY_PATH]
        source_identity = source_record["source_identity"]
        projection_stream.write(
            json.dumps(
                {
                    "schema_version": HISTORY_PROJECTION_SCHEMA_VERSION,
                    "source_history": {
                        "bytes": source_identity["bytes"],
                        "sha256": source_identity["sha256"],
                    },
                    "meta": meta,
                },
                separators=(",", ":"),
                allow_nan=False,
            )[:-1]
        )
        projection_stream.write(',"steps":[')
    first_projected_step = True

    converged = 0
    nonconverged = 0
    pre_converged = 0
    pre_nonconverged = 0
    post_converged = 0
    post_nonconverged = 0
    pre_shortcut_converged = 0
    post_shortcut_converged = 0
    pre_configured_gate_converged = 0
    post_configured_gate_converged = 0
    natural_at_or_below = 0
    cross = {
        "converged_and_natural_at_or_below": 0,
        "converged_and_natural_above": 0,
        "nonconverged_and_natural_at_or_below": 0,
        "nonconverged_and_natural_above": 0,
    }
    first_nonconverged: int | None = None
    outer_cap_hits = 0
    zero_outer = 0
    configured_gate_converged = 0
    contact_positive = 0
    warmup_steps = 0
    max_residual = -math.inf
    max_residual_step: int | None = None
    release_record: dict[str, Any] | None = None
    first_contact_increase: dict[str, int] | None = None
    peak_contacts = -1
    peak_contact_step: int | None = None
    previous_contacts: int | None = None
    final_record: dict[str, Any] | None = None
    step_count = 0

    if cursor.peek() != "]":
        while True:
            step = cursor.decode()
            if projected:
                _validate_projected_history_step(step, step_count)
            else:
                _validate_history_step(step, step_count)
            if projection_stream is not None:
                if not first_projected_step:
                    projection_stream.write(",")
                projection_stream.write(
                    json.dumps(
                        _project_history_step(step),
                        separators=(",", ":"),
                        allow_nan=False,
                    )
                )
                first_projected_step = False
            is_converged = step["converged"]
            natural_small = step["final_residual"] <= TERMINATION_TOLERANCE
            if is_converged:
                converged += 1
            else:
                nonconverged += 1
                if first_nonconverged is None:
                    first_nonconverged = step_count
            phase = "pre" if step_count < RELEASE_SUBSTEP else "post"
            if phase == "pre" and is_converged:
                pre_converged += 1
            elif phase == "pre":
                pre_nonconverged += 1
            elif is_converged:
                post_converged += 1
            else:
                post_nonconverged += 1
            if is_converged and step["outer_iters"] == 0:
                if phase == "pre":
                    pre_shortcut_converged += 1
                else:
                    post_shortcut_converged += 1
            elif is_converged:
                if phase == "pre":
                    pre_configured_gate_converged += 1
                else:
                    post_configured_gate_converged += 1
            if natural_small:
                natural_at_or_below += 1
            cross[
                ("converged" if is_converged else "nonconverged")
                + "_and_natural_"
                + ("at_or_below" if natural_small else "above")
            ] += 1
            outer_cap_hits += step["outer_iters"] == MAX_OUTER
            zero_outer += step["outer_iters"] == 0
            configured_gate_converged += is_converged and step["outer_iters"] > 0
            contact_positive += step["num_contacts"] > 0
            warmup_steps += step["warmup"]
            if step["final_residual"] > max_residual:
                max_residual = step["final_residual"]
                max_residual_step = step_count
            compact = {
                "step_idx": step_count,
                "num_contacts": step["num_contacts"],
                "converged": is_converged,
                "outer_iters": step["outer_iters"],
                "natural_final_residual": step["final_residual"],
            }
            if step_count == RELEASE_SUBSTEP:
                release_record = compact
            if (
                step_count >= RELEASE_SUBSTEP
                and previous_contacts is not None
                and step["num_contacts"] > previous_contacts
                and first_contact_increase is None
            ):
                first_contact_increase = {
                    "step_idx": step_count,
                    "previous_count": previous_contacts,
                    "count": step["num_contacts"],
                }
            previous_contacts = step["num_contacts"]
            if step["num_contacts"] > peak_contacts:
                peak_contacts = step["num_contacts"]
                peak_contact_step = step_count
            final_record = compact
            step_count += 1
            delimiter = cursor.peek()
            if delimiter == ",":
                cursor.expect(",")
                continue
            if delimiter == "]":
                break
            raise ValueError(f"{label}: malformed steps array")
    cursor.expect("]")
    cursor.expect("}")
    cursor.finish()
    if projection_stream is not None:
        projection_stream.write("]}\n")

    return {
        "total_substeps": step_count,
        "author_converged_flag_count": converged,
        "author_nonconverged_flag_count": nonconverged,
        "initial_natural_residual_shortcut_converged_count": zero_outer,
        "configured_coulomb_rel_outer_gate_converged_count": (
            configured_gate_converged
        ),
        "pre_release": {
            "substeps": RELEASE_SUBSTEP,
            "converged": pre_converged,
            "nonconverged": pre_nonconverged,
            "initial_natural_residual_shortcut_converged": pre_shortcut_converged,
            "configured_coulomb_rel_outer_gate_converged": (
                pre_configured_gate_converged
            ),
        },
        "post_release": {
            "substeps": TOTAL_SUBSTEPS - RELEASE_SUBSTEP,
            "converged": post_converged,
            "nonconverged": post_nonconverged,
            "initial_natural_residual_shortcut_converged": post_shortcut_converged,
            "configured_coulomb_rel_outer_gate_converged": (
                post_configured_gate_converged
            ),
        },
        "natural_final_residual_at_or_below_tolerance_count": natural_at_or_below,
        "convergence_natural_threshold_cross_table": cross,
        "first_nonconverged_step_idx": first_nonconverged,
        "outer_cap_hit_count": outer_cap_hits,
        "zero_outer_iteration_count": zero_outer,
        "contact_positive_step_count": contact_positive,
        "warmup_step_count": warmup_steps,
        "max_natural_final_residual": {
            "step_idx": max_residual_step,
            "value": max_residual,
        },
        "release_substep": release_record,
        "first_inferred_contact_count_increase_after_release": (first_contact_increase),
        "peak_contact_count": {
            "step_idx": peak_contact_step,
            "count": peak_contacts,
        },
        "final_substep": final_record,
        "contact_pair_identity_available": False,
    }


def summarize_raw_history(path: Path) -> dict[str, Any]:
    with path.open("rt", encoding="utf-8", newline="") as stream:
        return summarize_history(stream, label=str(path))


def summarize_projected_gzip_history(path: Path) -> dict[str, Any]:
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        return summarize_history(stream, label=str(path), projected=True)


def write_history_projection(source: Path, destination: Path) -> dict[str, Any]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rt", encoding="utf-8", newline="") as source_stream:
        with destination.open("wb") as raw_output:
            with gzip.GzipFile(
                filename="",
                mode="wb",
                fileobj=raw_output,
                compresslevel=9,
                mtime=0,
            ) as compressed:
                with io.TextIOWrapper(
                    compressed, encoding="utf-8", newline=""
                ) as projection_stream:
                    return summarize_history(
                        source_stream,
                        label=str(source),
                        projection_stream=projection_stream,
                    )


def validate_gzip_source_identity(
    path: Path, *, raw_bytes: int, raw_sha256: str
) -> None:
    with path.open("rb") as stream:
        header = stream.read(10)
    if len(header) != 10 or header[:4] != b"\x1f\x8b\x08\x00":
        raise ValueError(f"{path}: expected deterministic gzip without options")
    if header[4:8] != b"\x00\x00\x00\x00":
        raise ValueError(f"{path}: gzip mtime must be zero")
    digest = hashlib.sha256()
    count = 0
    decompressor = zlib.decompressobj(16 + zlib.MAX_WBITS)
    with path.open("rb") as stream:
        while chunk := stream.read(GZIP_CHUNK_BYTES):
            try:
                output = decompressor.decompress(chunk)
            except zlib.error as error:
                raise ValueError(f"{path}: invalid gzip stream") from error
            count += len(output)
            digest.update(output)
            if count > raw_bytes:
                raise ValueError(f"{path}: decompressed history exceeds size bound")
            if decompressor.unused_data:
                raise ValueError(f"{path}: gzip stream has trailing data or members")
    try:
        output = decompressor.flush()
    except zlib.error as error:
        raise ValueError(f"{path}: invalid gzip stream") from error
    count += len(output)
    digest.update(output)
    if not decompressor.eof:
        raise ValueError(f"{path}: truncated gzip stream")
    if decompressor.unused_data or decompressor.unconsumed_tail:
        raise ValueError(f"{path}: gzip stream has trailing data or members")
    if count != raw_bytes or digest.hexdigest() != raw_sha256:
        raise ValueError(f"{path}: decompressed source identity changed")


def _gzip_n9_mtime0(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as input_stream, destination.open("wb") as raw_output:
        with gzip.GzipFile(
            filename="", mode="wb", fileobj=raw_output, compresslevel=9, mtime=0
        ) as output_stream:
            shutil.copyfileobj(input_stream, output_stream, GZIP_CHUNK_BYTES)


def load_trajectory(path: Path) -> dict[str, np.ndarray]:
    expected = {
        "t.npy": (np.dtype("float64"), (TOTAL_FRAMES,)),
        "keystone_z.npy": (np.dtype("float64"), (TOTAL_FRAMES,)),
        "stone_z_init.npy": (np.dtype("float32"), (25,)),
        "stone_z_final.npy": (np.dtype("float32"), (25,)),
    }
    try:
        with zipfile.ZipFile(path) as archive:
            members = archive.infolist()
            if {item.filename for item in members} != set(expected):
                raise ValueError(f"{path}: unexpected NPZ member set")
            if any(item.is_dir() for item in members):
                raise ValueError(f"{path}: NPZ contains directory member")
            if any(item.file_size > MAX_NPZ_MEMBER_BYTES for item in members):
                raise ValueError(f"{path}: NPZ member exceeds size bound")
            if sum(item.file_size for item in members) > MAX_NPZ_TOTAL_BYTES:
                raise ValueError(f"{path}: NPZ exceeds total size bound")
        with np.load(path, allow_pickle=False) as archive:
            arrays = {key: archive[key].copy() for key in archive.files}
    except (OSError, ValueError, zipfile.BadZipFile) as error:
        if isinstance(error, ValueError) and str(error).startswith(str(path)):
            raise
        raise ValueError(f"{path}: invalid NPZ") from error
    if set(arrays) != {name.removesuffix(".npy") for name in expected}:
        raise ValueError(f"{path}: unexpected NPZ array set")
    for member, (dtype, shape) in expected.items():
        key = member.removesuffix(".npy")
        array = arrays[key]
        if array.dtype != dtype:
            raise ValueError(f"{path}: {key} dtype must be {dtype}")
        if array.shape != shape:
            raise ValueError(f"{path}: {key} shape must be {shape}")
        if not np.isfinite(array).all():
            raise ValueError(f"{path}: {key} contains non-finite values")
    return arrays


def summarize_trajectory(arrays: dict[str, np.ndarray]) -> dict[str, Any]:
    expected_time = np.arange(1, TOTAL_FRAMES + 1, dtype=np.float64) * (1.0 / 60.0)
    if not np.array_equal(arrays["t"], expected_time):
        raise ValueError("author trajectory time grid changed")
    init = arrays["stone_z_init"]
    final = arrays["stone_z_final"]
    keystone = arrays["keystone_z"]
    max_height_change = float(np.max(np.abs(init.astype(np.float64) - final)))
    summary = {
        "sample_count": TOTAL_FRAMES,
        "stone_count": 25,
        "keystone_idx": 12,
        "keystone_z_init": float(init[12]),
        "keystone_z_first_recorded": float(keystone[0]),
        "keystone_z_final": float(keystone[-1]),
        "max_height_change": max_height_change,
    }
    expected = {
        "sample_count": 500,
        "stone_count": 25,
        "keystone_idx": 12,
        "keystone_z_init": 61.92774963378906,
        "keystone_z_first_recorded": 61.92604446411133,
        "keystone_z_final": 60.18415069580078,
        "max_height_change": 1.7435989379882812,
    }
    _require_exact(summary, expected, label="author trajectory summary")
    return summary


def validate_result(payload: Any, trajectory: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(payload, dict) or set(payload) != RESULT_KEYS:
        raise ValueError("author masonry-arch result key set changed")
    for key, expected in EXPECTED_RESULT_STATIC.items():
        if not exact_json_equal(payload[key], expected):
            raise ValueError(f"author masonry-arch result field changed: {key}")
    _require_exact(payload["config"], EXPECTED_CONFIG, label="author solver config")
    times = payload["step_times_ms"]
    if not isinstance(times, list) or len(times) != TOTAL_FRAMES:
        raise ValueError("author result step_times_ms length changed")
    for index, value in enumerate(times):
        if _require_finite_number(value, label=f"step_times_ms[{index}]") <= 0.0:
            raise ValueError(f"step_times_ms[{index}] must be positive")
    if (
        float(np.mean(np.asarray(times, dtype=np.float64)))
        != payload["avg_step_time_ms"]
    ):
        raise ValueError("author result average timing does not recompute")
    if payload["max_height_change"] != trajectory["max_height_change"]:
        raise ValueError("author result max_height_change does not match trajectory")
    if payload["keystone_z_init"] != trajectory["keystone_z_init"]:
        raise ValueError(
            "author result initial keystone height does not match trajectory"
        )
    if payload["keystone_z_final"] != trajectory["keystone_z_final"]:
        raise ValueError(
            "author result final keystone height does not match trajectory"
        )
    return payload


def expected_summary() -> dict[str, Any]:
    return {
        "schema_version": SUMMARY_SCHEMA_VERSION,
        "status": "valid_current_source_scientific_negative",
        "protocol": {
            "source_commit": SOURCE_COMMIT,
            "run_id": RUN_ID,
            "frames": TOTAL_FRAMES,
            "drop_frame": DROP_FRAME,
            "substeps_per_frame": SUBSTEPS,
            "release_substep": RELEASE_SUBSTEP,
            "total_substeps": TOTAL_SUBSTEPS,
            "pre_release_substeps": RELEASE_SUBSTEP,
            "post_release_substeps": TOTAL_SUBSTEPS - RELEASE_SUBSTEP,
            "source_default_frames": SOURCE_DEFAULT_FRAMES,
            "source_default_drop_frame": DROP_FRAME,
            "source_default_releases_cubes": False,
            "observed_invocation_is_source_default": False,
        },
        "history": dict(EXPECTED_HISTORY_METRICS),
        "trajectory": {
            "sample_count": 500,
            "stone_count": 25,
            "keystone_idx": 12,
            "keystone_z_init": 61.92774963378906,
            "keystone_z_first_recorded": 61.92604446411133,
            "keystone_z_final": 60.18415069580078,
            "max_height_change": 1.7435989379882812,
        },
        "residual_semantics": {
            "configured_termination_residual": "coulomb_rel",
            "configured_termination_tolerance": TERMINATION_TOLERANCE,
            "author_converged_flag_is_exclusively_configured_gate": False,
            "initial_natural_residual_shortcut_converged_count": 40,
            "configured_coulomb_rel_outer_gate_converged_count": 117,
            "final_residual_field_is_natural_residual": True,
            "natural_threshold_count_is_not_configured_convergence_count": True,
        },
        "predicates": {
            "artifact_integrity_valid": True,
            "claim_history_projection_valid": True,
            "raw_source_history_retained": False,
            "current_source_diagnostic_valid": True,
            "scientific_negative": True,
            "all_substeps_solver_contract_valid": False,
            "historical_or_paper_invocation_valid": False,
            "dart_dynamics_parity_valid": False,
            "cross_solver_parity_valid": False,
            "timing_verdict": None,
            "runtime_attestation_complete": False,
            "contact_increase_is_pair_identified_cube_arch_contact": False,
        },
        "nonclaims": list(NONCLAIMS),
    }


def build_summary(
    history: dict[str, Any], trajectory: dict[str, Any]
) -> dict[str, Any]:
    payload = expected_summary()
    payload["history"] = history
    payload["trajectory"] = trajectory
    _require_exact(payload, expected_summary(), label="author masonry-arch summary")
    return payload


def expected_manifest() -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "valid_current_source_scientific_negative",
        "source": {
            "repository": SOURCE_REPOSITORY,
            "commit": SOURCE_COMMIT,
            "tree": SOURCE_TREE,
            "runner_path": RUNNER_PATH,
            "runner_sha256": RUNNER_SHA256,
            "runner_git_blob": RUNNER_GIT_BLOB,
            "requirements_path": REQUIREMENTS_PATH,
            "requirements_sha256": REQUIREMENTS_SHA256,
            "requirements_git_blob": REQUIREMENTS_GIT_BLOB,
            "clean_pinned_checkout_observed_at_finalization": True,
            "pre_and_post_run_cleanliness_captured": False,
        },
        "run": {
            "run_id": RUN_ID,
            "source_directory": SOURCE_RUN_DIRECTORY,
            "source_default": {
                "frames": SOURCE_DEFAULT_FRAMES,
                "drop_frame": DROP_FRAME,
                "releases_cubes": False,
            },
            "observed_override": {
                "frames": TOTAL_FRAMES,
                "drop_frame": DROP_FRAME,
                "releases_cubes": True,
            },
        },
        "invocation": {
            "argv": list(INVOCATION),
            "cwd": "/tmp/fbf-sca-2026-author",
            "returncode": 0,
            "observation_source": "orchestration_record",
            "argv_embedded_in_source_result": False,
            "stdout_retained": False,
            "stderr_retained": False,
            "warmup_steps": 0,
            "repetitions": 1,
            "timing_includes_first_use_compilation": True,
        },
        "environment": EXPECTED_ENVIRONMENT,
        "retained_source_artifacts": [dict(item) for item in SOURCE_ARTIFACTS],
        "omitted_redundant_source_artifact": OMITTED_SOURCE_ARTIFACT,
        "claim_boundary": {
            "current_source_diagnostic": True,
            "scientific_negative": True,
            "claim_history_projection_valid": True,
            "raw_source_history_retained": False,
            "all_substeps_solver_contract_valid": False,
            "historical_or_paper_invocation": False,
            "dart_dynamics_parity": False,
            "cross_solver_parity": False,
            "contact_count_change_only_inferred": True,
            "contact_pairs_recorded": False,
            "timing_verdict": None,
            "runtime_attestation_complete": False,
        },
        "nonclaims": list(NONCLAIMS),
    }


def validate_manifest(payload: Any) -> dict[str, Any]:
    _require_exact(
        payload,
        expected_manifest(),
        label="author masonry-arch manifest or claim boundary",
    )
    return payload


def report_markdown() -> str:
    return f"""# Current-author masonry-arch reference v1

Status: **valid current-source scientific negative**.

This sealed bundle records the completed pinned author-source run at
`{SOURCE_COMMIT}`. The observed command used 500 frames and released the three
cubes at frame 400 (solver substep {RELEASE_SUBSTEP}). The runner's source
default is 400 frames with `drop_frame=400`, which never releases the cubes;
therefore this is a newly declared current-source diagnostic, not a historical
or paper invocation.

## Solver result

- A deterministic projection represents all {TOTAL_SUBSTEPS} solver substeps
  and is lossless with respect to every declared claim-bearing natural and
  configured residual field. The 382,753,953-byte raw history is bound by
  SHA-256 but intentionally omitted; the projection is about 8.1 MB instead of
  a 49 MB raw archive.
- Author convergence flags: 157 converged, 1,843 nonconverged.
- Of the 157 true flags, 40 are zero-outer-iteration accepts from the initial
  natural-residual shortcut and 117 are outer solves accepted by the configured
  nonnegative `coulomb_rel < 1e-6` gate.
- Before release: 142 converged and 1,458 nonconverged substeps.
- After release: 15 converged and 385 nonconverged substeps.
- Only 47 `final_residual` values are at or below `1e-6`. That field is the
  natural residual; it is not the configured `coulomb_rel` convergence gate.
- First nonconverged substep: 53. Maximum natural residual:
  4.1130565788445415 at substep 226.
- Release substep 1600 has 100 contacts and natural residual
  0.017456069692858667. The final substep has 108 contacts and natural residual
  0.5161195175386001; both carry negative convergence flags.

The first post-release contact-count increase is inferred at substep 1944
(100 to 102), and the peak is 109 at substep 1947. The projection does not
record contact-pair identities, so this is not definitive cube-arch contact
evidence.

## Claim boundary

Exit code 0 and intact artifacts validate sealing of this current-source
diagnostic. They do **not** validate all-substep solver success, DART or
cross-solver dynamics parity, paper timing, repeatability, or a historical
paper invocation.
"""


def build_artifact_index(root: Path) -> dict[str, Any]:
    artifacts = []
    for relative in sorted(CORE_FILES):
        _safe_relative_path(relative)
        path = root / relative
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"indexed artifact is not a regular file: {relative}")
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
        "stored_bytes": sum(item["bytes"] for item in artifacts),
        "excluded": sorted(INDEX_EXCLUSIONS),
        "artifacts": artifacts,
    }


def validate_artifact_index(root: Path, payload: Any) -> dict[str, Any]:
    expected = build_artifact_index(root)
    _require_exact(payload, expected, label="author masonry-arch artifact index")
    return payload


def expected_verification(root: Path) -> dict[str, Any]:
    index = read_json(root / "artifact-index.json")
    return {
        "schema_version": VERIFICATION_SCHEMA_VERSION,
        "pass": True,
        "status": "valid_current_source_scientific_negative",
        "source_commit": SOURCE_COMMIT,
        "artifact_count": index["artifact_count"],
        "stored_bytes": index["stored_bytes"],
        "artifact_index_sha256": sha256(root / "artifact-index.json"),
        "manifest_sha256": sha256(root / "manifest.json"),
        "summary_sha256": sha256(root / "summary.json"),
        "finalizer": {
            "path": "scripts/finalize_fbf_author_masonry_arch_reference.py",
            "sha256": sha256(Path(__file__).resolve()),
        },
        "checks": [
            "exact_bundle_membership",
            "no_symlinks_or_nonregular_entries",
            "raw_source_history_hash_and_size_bound",
            "single_member_deterministic_gzip",
            "bounded_streaming_claim_projection_validation",
            "claim_projection_exact_field_boundary",
            "bounded_pickle_disabled_npz_validation",
            "result_trajectory_metric_recomputation",
            "author_convergence_and_natural_residual_semantics_separated",
            "fail_closed_claim_boundary",
            "artifact_index_current",
            "no_reserved_transient_directories",
        ],
    }


def _validate_raw_identity(path: Path, record: dict[str, Any]) -> None:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"source artifact is not a regular file: {path}")
    identity = record.get("source_identity", record)
    if path.stat().st_size != identity["bytes"] or sha256(path) != identity["sha256"]:
        raise ValueError(f"source artifact identity changed: {path}")


def _validate_retained_artifacts(bundle: Path) -> None:
    for record in SOURCE_ARTIFACTS:
        destination = bundle / record["bundle_path"]
        if record["encoding"] == "raw":
            _validate_raw_identity(destination, record)
        elif record["encoding"] == "claim-projection-json-gzip-n9-mtime0":
            projection = record["projection"]
            if (
                destination.stat().st_size != projection["stored_bytes"]
                or sha256(destination) != projection["stored_sha256"]
            ):
                raise ValueError(
                    f"stored history projection identity changed: {destination}"
                )
            validate_gzip_source_identity(
                destination,
                raw_bytes=projection["decompressed_bytes"],
                raw_sha256=projection["decompressed_sha256"],
            )
        else:
            raise ValueError(f"unsupported retained artifact encoding: {record}")


def _validate_semantics(
    *,
    metadata_path: Path,
    result_path: Path,
    trajectory_path: Path,
    history_path: Path,
    projected_history: bool,
) -> dict[str, Any]:
    metadata = read_json(metadata_path)
    _require_exact(metadata, EXPECTED_METADATA, label="author run metadata")
    arrays = load_trajectory(trajectory_path)
    trajectory = summarize_trajectory(arrays)
    result = read_json(result_path)
    validate_result(result, trajectory)
    history = (
        summarize_projected_gzip_history(history_path)
        if projected_history
        else summarize_raw_history(history_path)
    )
    summary = build_summary(history, trajectory)
    return {"result": result, "summary": summary}


def validate_bundle(bundle: Path = DEFAULT_BUNDLE) -> dict[str, Any]:
    bundle = bundle.expanduser()
    validate_membership(bundle)
    manifest = validate_manifest(read_json(bundle / "manifest.json"))
    index = validate_artifact_index(bundle, read_json(bundle / "artifact-index.json"))
    _validate_retained_artifacts(bundle)
    semantics = _validate_semantics(
        metadata_path=bundle / METADATA_PATH,
        result_path=bundle / RESULT_PATH,
        trajectory_path=bundle / TRAJECTORY_PATH,
        history_path=bundle / HISTORY_PATH,
        projected_history=True,
    )
    _require_exact(
        read_json(bundle / "summary.json"),
        semantics["summary"],
        label="author masonry-arch stored summary",
    )
    if (bundle / "REPORT.md").read_text(encoding="utf-8") != report_markdown():
        raise ValueError("author masonry-arch report changed")
    _require_exact(
        read_json(bundle / "verification.json"),
        expected_verification(bundle),
        label="author masonry-arch verification record",
    )
    return {
        "schema_version": VERIFICATION_SCHEMA_VERSION,
        "pass": True,
        "status": semantics["summary"]["status"],
        "source_commit": manifest["source"]["commit"],
        "artifact_count": index["artifact_count"],
        "stored_bytes": index["stored_bytes"],
        "author_converged_flag_count": semantics["summary"]["history"][
            "author_converged_flag_count"
        ],
        "author_nonconverged_flag_count": semantics["summary"]["history"][
            "author_nonconverged_flag_count"
        ],
        "dart_dynamics_parity_valid": False,
    }


def _git_output(source_repo: Path, *arguments: str) -> str:
    process = subprocess.run(
        ["git", "-C", str(source_repo), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return process.stdout.strip()


def _validate_source_repo(source_repo: Path) -> None:
    if source_repo.is_symlink() or not source_repo.is_dir():
        raise ValueError("author source checkout must be a regular directory")
    if _git_output(source_repo, "rev-parse", "HEAD") != SOURCE_COMMIT:
        raise ValueError("author source checkout is not at the pinned commit")
    if _git_output(source_repo, "rev-parse", "HEAD^{tree}") != SOURCE_TREE:
        raise ValueError("author source tree identity changed")
    if _git_output(source_repo, "remote", "get-url", "origin") != SOURCE_REPOSITORY:
        raise ValueError("author source remote changed")
    if _git_output(source_repo, "status", "--porcelain=v1"):
        raise ValueError("author source checkout must be clean")
    if sha256(source_repo / RUNNER_PATH) != RUNNER_SHA256:
        raise ValueError("author masonry-arch runner content changed")
    if _git_output(source_repo, "hash-object", RUNNER_PATH) != RUNNER_GIT_BLOB:
        raise ValueError("author masonry-arch runner git blob changed")
    if sha256(source_repo / REQUIREMENTS_PATH) != REQUIREMENTS_SHA256:
        raise ValueError("author requirements content changed")
    if (
        _git_output(source_repo, "hash-object", REQUIREMENTS_PATH)
        != REQUIREMENTS_GIT_BLOB
    ):
        raise ValueError("author requirements git blob changed")


def _validate_source_environment(source_repo: Path) -> None:
    interpreter = Path(INVOCATION[0])
    program = """
import hashlib
import importlib.metadata
import json
import pathlib
import platform
import sys

resolved = pathlib.Path(sys.executable).resolve()
distributions = sorted(
    f"{item.metadata['Name']}=={item.version}"
    for item in importlib.metadata.distributions()
)
payload = {
    "invoked_interpreter": sys.executable,
    "resolved_interpreter": str(resolved),
    "resolved_interpreter_sha256": hashlib.sha256(resolved.read_bytes()).hexdigest(),
    "python_implementation": platform.python_implementation(),
    "python_version": sys.version,
    "platform": platform.platform(),
    "machine": platform.machine(),
    "installed_distribution_count": len(distributions),
    "installed_distributions": distributions,
    "installed_distributions_sha256": hashlib.sha256(
        ("\\n".join(distributions) + "\\n").encode("utf-8")
    ).hexdigest(),
    "requirements_exact_match": True,
    "observation_timing": "post_run_only_at_finalization",
}
print(json.dumps(payload, allow_nan=False))
"""
    process = subprocess.run(
        [str(interpreter), "-c", program],
        cwd=source_repo,
        check=True,
        capture_output=True,
        text=True,
    )
    try:
        payload = json.loads(process.stdout, object_pairs_hook=_unique_object)
    except json.JSONDecodeError as error:
        raise ValueError(
            "author runtime environment query returned invalid JSON"
        ) from error
    _require_exact(
        payload,
        EXPECTED_ENVIRONMENT,
        label="author post-run environment observation",
    )
    requirement_lines = [
        line.strip()
        for line in (source_repo / REQUIREMENTS_PATH)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    if sorted(requirement_lines) != sorted(INSTALLED_DISTRIBUTIONS):
        raise ValueError("author environment no longer exactly matches requirements")


def _validate_source_run(source_repo: Path) -> dict[str, Any]:
    source_run = source_repo / SOURCE_RUN_DIRECTORY
    expected_files = {
        "metadata.json",
        "sweep_results.json",
        "fbf/result.json",
        "fbf/trajectory.npz",
        "fbf/history.json",
    }
    expected_directories = {"fbf"}
    files: set[str] = set()
    directories: set[str] = set()
    for path in source_run.rglob("*"):
        relative = path.relative_to(source_run).as_posix()
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"author source run contains symlink: {relative}")
        if stat.S_ISREG(mode):
            files.add(relative)
        elif stat.S_ISDIR(mode):
            directories.add(relative)
        else:
            raise ValueError(f"author source run contains non-regular path: {relative}")
    if files != expected_files or directories != expected_directories:
        raise ValueError("author source run membership changed")
    for record in SOURCE_ARTIFACTS:
        _validate_raw_identity(source_repo / record["source_path"], record)
    _validate_raw_identity(
        source_repo / OMITTED_SOURCE_ARTIFACT["source_path"],
        OMITTED_SOURCE_ARTIFACT,
    )
    semantics = _validate_semantics(
        metadata_path=source_run / "metadata.json",
        result_path=source_run / "fbf/result.json",
        trajectory_path=source_run / "fbf/trajectory.npz",
        history_path=source_run / "fbf/history.json",
        projected_history=False,
    )
    sweep = read_json(source_run / "sweep_results.json")
    _require_exact(
        sweep,
        [semantics["result"]],
        label="author single-case sweep result",
    )
    return semantics["summary"]


def _write_stage(source_repo: Path, stage: Path, summary: dict[str, Any]) -> None:
    stage.mkdir(parents=True)
    for record in SOURCE_ARTIFACTS:
        source = source_repo / record["source_path"]
        destination = stage / record["bundle_path"]
        destination.parent.mkdir(parents=True, exist_ok=True)
        if record["encoding"] == "raw":
            shutil.copyfile(source, destination)
        elif record["encoding"] == "claim-projection-json-gzip-n9-mtime0":
            projected_summary = write_history_projection(source, destination)
            _require_exact(
                projected_summary,
                summary["history"],
                label="author history projection summary",
            )
        else:
            raise ValueError(f"unsupported retained artifact encoding: {record}")
    write_json(stage / "manifest.json", expected_manifest())
    write_json(stage / "summary.json", summary)
    (stage / "REPORT.md").write_text(report_markdown(), encoding="utf-8")
    write_json(stage / "artifact-index.json", build_artifact_index(stage))
    write_json(stage / "verification.json", expected_verification(stage))


def _publish_staged_bundle(
    stage: Path,
    bundle: Path,
    *,
    validator: Callable[[Path], dict[str, Any]] = validate_bundle,
) -> dict[str, Any]:
    reject_symlink_path_components(bundle)
    token = uuid.uuid4().hex
    backup = bundle.parent / f".{bundle.name}.backup-{token}"
    had_existing = bundle.exists()
    if had_existing and (bundle.is_symlink() or not bundle.is_dir()):
        raise ValueError("existing bundle is not a regular directory")
    try:
        if had_existing:
            os.replace(bundle, backup)
        os.replace(stage, bundle)
        return validator(bundle)
    except Exception:
        _remove_path(bundle)
        if backup.exists() or backup.is_symlink():
            os.replace(backup, bundle)
        raise
    finally:
        _remove_path(stage)
        _remove_path(backup)


def finalize_bundle(
    source_repo: Path = DEFAULT_SOURCE_REPO,
    bundle: Path = DEFAULT_BUNDLE,
) -> dict[str, Any]:
    bundle = bundle.expanduser()
    reject_symlink_path_components(bundle)
    source_repo = source_repo.expanduser().resolve(strict=True)
    _validate_source_repo(source_repo)
    _validate_source_environment(source_repo)
    summary = _validate_source_run(source_repo)

    bundle.parent.mkdir(parents=True, exist_ok=True)
    staging_parent = bundle.parent / ".staging"
    reject_symlink_path_components(staging_parent)
    staging_parent.mkdir(exist_ok=True)
    stage_container = staging_parent / uuid.uuid4().hex
    stage = stage_container / bundle.name
    try:
        _write_stage(source_repo, stage, summary)
        validate_bundle(stage)
        _validate_source_repo(source_repo)
        _validate_source_environment(source_repo)
        _validate_source_run(source_repo)
        return _publish_staged_bundle(stage, bundle)
    finally:
        _remove_path(stage_container)
        try:
            staging_parent.rmdir()
        except OSError:
            pass


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Finalize or verify the pinned current-author masonry-arch "
            "scientific-negative bundle"
        )
    )
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--source-repo", type=Path, default=DEFAULT_SOURCE_REPO)
    parser.add_argument("--verify-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = (
            validate_bundle(args.bundle)
            if args.verify_only
            else finalize_bundle(args.source_repo, args.bundle)
        )
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
