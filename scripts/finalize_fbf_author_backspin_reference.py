#!/usr/bin/env python3
"""Seal and verify the pinned current-author backspin output.

This is a fail-closed, source-specific numeric reference.  It records one
author FBF run and the analytic rolling projection for the same initial state;
it does not claim DART equivalence, paper-wide parity, timing validity, or
renderer equivalence.
"""

from __future__ import annotations

import argparse
import copy
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
from typing import Any, Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_REPO = Path("/tmp/fbf-sca-2026-author")
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "author_backspin_reference_v1"
)

SCHEMA_VERSION = "dart.fbf_author_backspin_reference/v1"
VALIDATION_SCHEMA_VERSION = "dart.fbf_author_backspin_validation/v1"
STATUS = "valid_current_source_numeric_reference"

SOURCE_REPOSITORY = "https://github.com/matthcsong/fbf-sca-2026.git"
SOURCE_COMMIT = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
SOURCE_TREE = "ffcdafb61adeda2239c8366d054b548b50d26685"
SOURCE_SHORT_COMMIT = "b3f3c5c"
RUNNER_PATH = "paper_examples/backspin-ball/run.py"
RUNNER_SHA256 = "c9174e88bf18dbe050d72568639de50b8477f2bc57ea9558637087da4268409a"
RUNNER_GIT_BLOB = "82d8916233df8db1cacc5915699dc53a5d08ea17"
REQUIREMENTS_PATH = "requirements.txt"
REQUIREMENTS_SHA256 = "436d6a280e62f0cf5b670bf0475cbed6e95fa87e2fb011bc30334355a3ed9688"
REQUIREMENTS_GIT_BLOB = "f42eadca6be4869162675340427315aca01bd634"

RUN_ID = "20260722T185956Z"
SOURCE_RUN_DIRECTORY = f"paper_examples/backspin-ball/results/{RUN_ID}"
SOURCE_HISTORY_PATH = f"{SOURCE_RUN_DIRECTORY}/fbf/history.json"
RESULT_HISTORY_PATH = f"/tmp/fbf-sca-2026-author/{SOURCE_HISTORY_PATH}"

RECORDED_INVOCATION = (
    "/tmp/fbf-author-venv/bin/python",
    RUNNER_PATH,
    "--solvers",
    "fbf",
    "--mu",
    "0.5",
    "--v0",
    "4",
    "--omega0",
    "-200",
    "--time",
    "4",
    "--device",
    "cpu",
    "--profile",
    "--residual-check-interval",
    "5",
    "--max-outer",
    "200",
    "--termination-residual",
    "coulomb_rel",
    "--termination-tol",
    "1e-6",
    "--gamma-c",
    "5",
    "--gamma-max",
    "1000000",
    "--inner-solver",
    "block_gs",
    "--inner-gs-sweeps",
    "10",
    "--inner-max-iter",
    "200",
    "--inner-tol",
    "1e-6",
    "--warm-start-gamma-cap",
    "10000",
    "--baumgarte-erp",
    "0",
    "--no-project-after-correction",
)

DT_SECONDS = 1.0 / 60.0
TOTAL_STEPS = 240
RADIUS = 0.25
MASS_NORMALIZED_INERTIA = 2.0 / 5.0
TERMINATION_TOLERANCE = 1.0e-6
TRAJECTORY_KEYS = ("t", "vx", "wy", "vc", "z")
MAX_NPZ_MEMBER_BYTES = 16 * 1024
MAX_NPZ_TOTAL_BYTES = 128 * 1024
GZIP_CHUNK_BYTES = 64 * 1024

SOURCE_TERMINAL = {
    "vx": -11.428571701049805,
    "wy": -45.71428680419922,
    "vc": 0.0,
    "z": -1.3714094161987305,
}
ANALYTIC_STATE = {
    "model": "solid_sphere_impulsive_rolling_projection",
    "mass_normalized_inertia": MASS_NORMALIZED_INERTIA,
    "impulse_per_mass": -15.428571428571429,
    "vx": -11.428571428571429,
    "wy": -45.71428571428572,
    "vc": 0.0,
}

METADATA_KEYS = {
    "T_seconds",
    "density",
    "device",
    "dt",
    "experiment",
    "gap",
    "gravity",
    "ground_hx",
    "ground_hy",
    "ground_hz",
    "mu",
    "num_steps",
    "omega0",
    "radius",
    "solvers",
    "timestamp",
    "v0",
}
RESULT_KEYS = {
    "T_seconds",
    "avg_step_time_ms",
    "config",
    "dt",
    "mu",
    "num_steps",
    "omega0",
    "solver",
    "step_times_ms",
    "usd",
    "v0",
    "vx_final",
    "wy_final",
}
CONFIG_KEYS = {
    "adaptive_gamma",
    "armijo_grow",
    "armijo_max_backtracks",
    "armijo_rho_high",
    "armijo_shrink",
    "armijo_skip_threshold",
    "baumgarte_erp",
    "dump_step_state_idx",
    "dump_step_state_path",
    "gamma",
    "gamma_c",
    "gamma_max",
    "gamma_min",
    "history_path",
    "inner_gs_sweeps",
    "inner_max_iter",
    "inner_solver",
    "inner_step_size",
    "inner_tol",
    "max_contacts",
    "max_outer",
    "outer_tol",
    "per_contact_diag_dir",
    "per_contact_diag_topk",
    "plateau_patience",
    "plateau_rtol",
    "project_after_correction",
    "residual_check_interval",
    "termination_residual",
    "termination_tol",
    "warm_start",
    "warm_start_gamma_cap",
    "warm_start_gamma_threshold",
    "warm_start_match_radius",
    "warm_start_max_age",
    "warm_start_normal_cos",
}
HISTORY_META_KEYS = {
    "bench",
    "config",
    "dt",
    "inner_solver",
    "num_bodies",
    "schema_version",
    "solver",
}
HISTORY_BENCH_KEYS = {
    "git_sha",
    "scene",
    "scene_params",
    "total_steps",
    "warmup_steps",
}
HISTORY_STEP_KEYS = {
    "W_lambda_max",
    "converged",
    "final_residual",
    "gamma_final",
    "gamma_init",
    "initial_residual",
    "num_contacts",
    "outer",
    "outer_iters",
    "step_idx",
    "t_collide_gpu_ms",
    "t_collide_ms",
    "t_init_residual_gpu_ms",
    "t_init_residual_ms",
    "t_integrate_free_gpu_ms",
    "t_integrate_free_ms",
    "t_jacobian_gpu_ms",
    "t_jacobian_ms",
    "t_power_iter_gpu_ms",
    "t_power_iter_ms",
    "t_recover_gpu_ms",
    "t_recover_ms",
    "t_save_warm_cache_gpu_ms",
    "t_save_warm_cache_ms",
    "t_total_gpu_ms",
    "t_total_ms",
    "t_unattributed_gpu_ms",
    "t_warm_start_gpu_ms",
    "t_warm_start_ms",
    "warm_start_matched",
    "warm_start_total",
    "warm_started",
    "warmup",
}
HISTORY_OUTER_KEYS = {
    "backtracks",
    "eps_force",
    "eps_gap",
    "eps_vel",
    "gamma",
    "gamma_event",
    "gamma_start",
    "inner_iters",
    "inner_last_change",
    "k",
    "r_coulomb",
    "r_dual",
    "r_gap",
    "r_kkt",
    "r_primal",
    "residual",
    "rho",
    "t_correction_gpu_ms",
    "t_correction_ms",
    "t_fwd_bwd_gpu_ms",
    "t_fwd_bwd_ms",
    "t_fwd_eval_gpu_ms",
    "t_fwd_eval_ms",
    "t_residual_check_gpu_ms",
    "t_residual_check_ms",
}
STEP_INTEGER_KEYS = {
    "num_contacts",
    "outer_iters",
    "step_idx",
    "warm_start_matched",
    "warm_start_total",
}
STEP_BOOLEAN_KEYS = {"converged", "warm_started", "warmup"}
STEP_CPU_TIME_KEYS = {
    "t_collide_ms",
    "t_init_residual_ms",
    "t_integrate_free_ms",
    "t_jacobian_ms",
    "t_power_iter_ms",
    "t_recover_ms",
    "t_save_warm_cache_ms",
    "t_total_ms",
    "t_warm_start_ms",
}
STEP_GPU_TIME_KEYS = {
    "t_collide_gpu_ms",
    "t_init_residual_gpu_ms",
    "t_integrate_free_gpu_ms",
    "t_jacobian_gpu_ms",
    "t_power_iter_gpu_ms",
    "t_recover_gpu_ms",
    "t_save_warm_cache_gpu_ms",
    "t_total_gpu_ms",
    "t_unattributed_gpu_ms",
    "t_warm_start_gpu_ms",
}
OUTER_INTEGER_KEYS = {"backtracks", "inner_iters", "k"}
OUTER_CPU_TIME_KEYS = {
    "t_correction_ms",
    "t_fwd_bwd_ms",
    "t_fwd_eval_ms",
    "t_residual_check_ms",
}
OUTER_GPU_TIME_KEYS = {
    "t_correction_gpu_ms",
    "t_fwd_bwd_gpu_ms",
    "t_fwd_eval_gpu_ms",
    "t_residual_check_gpu_ms",
}

EXPECTED_METADATA = {
    "T_seconds": 4.0,
    "density": 15.278874536821954,
    "device": "cpu",
    "dt": DT_SECONDS,
    "experiment": "backspin_ball",
    "gap": 0.001,
    "gravity": 9.81,
    "ground_hx": 15.0,
    "ground_hy": 0.5,
    "ground_hz": 0.05,
    "mu": 0.5,
    "num_steps": TOTAL_STEPS,
    "omega0": -200.0,
    "radius": RADIUS,
    "solvers": ["fbf"],
    "timestamp": RUN_ID,
    "v0": 4.0,
}


def _expected_config(history_path: str) -> dict[str, Any]:
    return {
        "adaptive_gamma": True,
        "armijo_grow": 1.4285714285714286,
        "armijo_max_backtracks": 8,
        "armijo_rho_high": 0.9,
        "armijo_shrink": 0.7,
        "armijo_skip_threshold": 1.0e-10,
        "baumgarte_erp": 0.0,
        "dump_step_state_idx": None,
        "dump_step_state_path": None,
        "gamma": None,
        "gamma_c": 5.0,
        "gamma_max": 1.0e6,
        "gamma_min": 1.0e-6,
        "history_path": history_path,
        "inner_gs_sweeps": 10,
        "inner_max_iter": 200,
        "inner_solver": "block_gs",
        "inner_step_size": None,
        "inner_tol": 1.0e-6,
        "max_contacts": 4096,
        "max_outer": 200,
        "outer_tol": 1.0e-6,
        "per_contact_diag_dir": None,
        "per_contact_diag_topk": 20,
        "plateau_patience": 5,
        "plateau_rtol": 0.01,
        "project_after_correction": False,
        "residual_check_interval": 5,
        "termination_residual": "coulomb_rel",
        "termination_tol": TERMINATION_TOLERANCE,
        "warm_start": True,
        "warm_start_gamma_cap": 10000.0,
        "warm_start_gamma_threshold": 0.0001,
        "warm_start_match_radius": 0.02,
        "warm_start_max_age": 3,
        "warm_start_normal_cos": 0.9,
    }


EXPECTED_HISTORY_SUMMARY = {
    "step_count": 240,
    "converged_steps": 240,
    "nonconverged_steps": [],
    "contact_steps": 207,
    "first_contact_step": 0,
    "last_contact_step": 206,
    "max_contacts": 1,
    "outer_solved_steps": 206,
    "initial_shortcut_steps": 1,
    "no_contact_steps": 33,
    "total_outer_iterations": 5255,
    "max_outer_iterations": 40,
    "outer_cap_steps": [],
    "natural_residual_above_tolerance_converged_steps": 183,
    "max_final_natural_residual": 3.290394367853205e-05,
    "max_final_natural_residual_step": 0,
    "max_terminal_coulomb_residual": 9.94422217909298e-07,
    "max_terminal_coulomb_residual_step": 182,
    "warm_started_steps": 0,
}

RAW_ARTIFACTS = {
    "metadata.json": {
        "bytes": 383,
        "sha256": "0d919084b7ef8dcc42e9c89c7b7a3c9256e50c4eac664b6b861c407b5ed9eb84",
    },
    "sweep_results.json": {
        "bytes": 7768,
        "sha256": "830e0db3b81406ab3ac305383f811daa45680b4bd2a8a94f198250b8b4cbb2ca",
    },
    "fbf/result.json": {
        "bytes": 7178,
        "sha256": "29a63de1277ec96c6e4f23c4650634c28c1a783a56995a5138c2d3bd15066adc",
    },
    "fbf/trajectory.npz": {
        "bytes": 10798,
        "sha256": "77a28f87962bef54132b98fa1f37d9e82807d33b8e5c3bb154e814bb02462133",
    },
    "fbf/history.json": {
        "bytes": 4951663,
        "sha256": "2d0d6fc73923d227e748002785ee11543f8aa292bf7926a03c21a9b4a7f7a3c9",
    },
}

EXPECTED_ARTIFACTS = (
    {
        "path": f"runs/{RUN_ID}/metadata.json",
        "bytes": 383,
        "sha256": RAW_ARTIFACTS["metadata.json"]["sha256"],
        "encoding": "raw",
    },
    {
        "path": f"runs/{RUN_ID}/fbf/result.json",
        "bytes": 7178,
        "sha256": RAW_ARTIFACTS["fbf/result.json"]["sha256"],
        "encoding": "raw",
    },
    {
        "path": f"runs/{RUN_ID}/fbf/trajectory.npz",
        "bytes": 10798,
        "sha256": RAW_ARTIFACTS["fbf/trajectory.npz"]["sha256"],
        "encoding": "raw",
    },
    {
        "path": f"runs/{RUN_ID}/fbf/history.json.gz",
        "bytes": 370014,
        "sha256": "216ad72d9c8a2c971e98e843bf88e7503ccdda98fd73405ec1fec5fd053c9f28",
        "encoding": "gzip-n9",
        "raw_bytes": RAW_ARTIFACTS["fbf/history.json"]["bytes"],
        "raw_sha256": RAW_ARTIFACTS["fbf/history.json"]["sha256"],
    },
)
EXPECTED_ARTIFACT_PATHS = {record["path"] for record in EXPECTED_ARTIFACTS}

NONCLAIMS = (
    "not_paper_wide_reproduction_or_parity",
    "not_dart_equivalence_or_cross_solver_parity",
    "not_timing_or_realtime_evidence",
    "not_renderer_camera_material_or_video_evidence",
    "not_infinite_ground_or_infinite_horizon_evidence",
    "terminal_z_is_source_output_after_the_ball_leaves_the_finite_ground",
)


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key!r}")
        result[key] = value
    return result


def _parse_json_bytes(data: bytes, location: str) -> Any:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError(f"{location}: expected UTF-8 JSON") from error

    def reject_constant(token: str) -> float:
        raise ValueError(f"{location}: nonstandard JSON constant {token!r}")

    try:
        return json.loads(
            text,
            object_pairs_hook=_unique_object,
            parse_constant=reject_constant,
        )
    except json.JSONDecodeError as error:
        raise ValueError(f"{location}: malformed JSON: {error}") from error


def _read_json(path: Path) -> Any:
    return _parse_json_bytes(path.read_bytes(), str(path))


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode(
        "utf-8"
    )


def _write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def _write_json(path: Path, value: Any) -> None:
    _write_bytes(path, _json_bytes(value))


def _exact_json_equal(actual: Any, expected: Any) -> bool:
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _exact_json_equal(actual[key], value) for key, value in expected.items()
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _exact_json_equal(left, right)
            for left, right in zip(actual, expected, strict=True)
        )
    return actual == expected


def _expect_exact(actual: Any, expected: Any, location: str) -> None:
    if not _exact_json_equal(actual, expected):
        raise ValueError(f"{location}: exact JSON value or type changed")


def _expect_keys(value: Any, expected: set[str], location: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != expected:
        actual = sorted(value) if isinstance(value, dict) else type(value).__name__
        raise ValueError(
            f"{location}: exact keys changed; expected {sorted(expected)}, got {actual}"
        )
    return value


def _finite_number(value: Any, location: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(float(value)):
        raise ValueError(f"{location}: expected a finite JSON number")
    return float(value)


def _finite_float(value: Any, location: str) -> float:
    if type(value) is not float or not math.isfinite(value):
        raise ValueError(f"{location}: expected a finite JSON float")
    return value


def _integer(value: Any, location: str) -> int:
    if type(value) is not int:
        raise ValueError(f"{location}: expected a JSON integer")
    return value


def _safe_relative(value: str, location: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if (
        not value
        or value == "."
        or path.is_absolute()
        or "\\" in value
        or any(part in {"", ".", ".."} for part in path.parts)
        or path.as_posix() != value
    ):
        raise ValueError(f"{location}: unsafe or noncanonical relative path {value!r}")
    return path


def _reject_symlink_components(path: Path, location: str) -> Path:
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if current.is_symlink():
            raise ValueError(f"{location}: path contains symlink component {current}")
    return absolute


def _remove_path(path: Path) -> None:
    try:
        mode = path.lstat().st_mode
    except FileNotFoundError:
        return
    if stat.S_ISDIR(mode) and not stat.S_ISLNK(mode):
        shutil.rmtree(path)
    else:
        path.unlink()


def _run_git(source_repo: Path, *arguments: str) -> str:
    try:
        result = subprocess.run(
            ["git", *arguments],
            cwd=source_repo,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        raise ValueError(
            f"source git command failed: git {' '.join(arguments)}"
        ) from error
    return result.stdout.strip()


def _validate_source_identity(source_repo: Path) -> dict[str, Any]:
    source_repo = _reject_symlink_components(source_repo, "source repository")
    if not source_repo.is_dir():
        raise ValueError(f"source repository does not exist: {source_repo}")
    if _run_git(source_repo, "rev-parse", "HEAD") != SOURCE_COMMIT:
        raise ValueError("source repository HEAD changed")
    if _run_git(source_repo, "rev-parse", "HEAD^{tree}") != SOURCE_TREE:
        raise ValueError("source repository tree changed")
    if _run_git(source_repo, "remote", "get-url", "origin") != SOURCE_REPOSITORY:
        raise ValueError("source repository origin changed")
    if _run_git(source_repo, "status", "--porcelain=v1", "--untracked-files=no"):
        raise ValueError("source repository has tracked changes")

    for relative, expected_sha, expected_blob in (
        (RUNNER_PATH, RUNNER_SHA256, RUNNER_GIT_BLOB),
        (REQUIREMENTS_PATH, REQUIREMENTS_SHA256, REQUIREMENTS_GIT_BLOB),
    ):
        path = source_repo / relative
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"source file is not regular: {relative}")
        if _sha256(path) != expected_sha:
            raise ValueError(f"source file digest changed: {relative}")
        if _run_git(source_repo, "hash-object", relative) != expected_blob:
            raise ValueError(f"source file git blob changed: {relative}")

    return {
        "repository": SOURCE_REPOSITORY,
        "commit": SOURCE_COMMIT,
        "tree": SOURCE_TREE,
        "runner_git_blob": RUNNER_GIT_BLOB,
        "runner_sha256": RUNNER_SHA256,
    }


def _expected_manifest() -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": STATUS,
        "source": {
            "repository": SOURCE_REPOSITORY,
            "commit": SOURCE_COMMIT,
            "tree": SOURCE_TREE,
            "history_reported_short_commit": SOURCE_SHORT_COMMIT,
            "runner_path": RUNNER_PATH,
            "runner_sha256": RUNNER_SHA256,
            "runner_git_blob": RUNNER_GIT_BLOB,
            "requirements_path": REQUIREMENTS_PATH,
            "requirements_sha256": REQUIREMENTS_SHA256,
            "requirements_git_blob": REQUIREMENTS_GIT_BLOB,
            "run_id": RUN_ID,
            "source_run_directory": SOURCE_RUN_DIRECTORY,
        },
        "invocation": {
            "argv": list(RECORDED_INVOCATION),
            "cwd": "/tmp/fbf-sca-2026-author",
            "returncode": 0,
            "observation_source": "orchestration_record",
            "argv_embedded_in_source_result": False,
            "stdout_retained": False,
            "stderr_retained": False,
        },
        "configuration": {
            "metadata": copy.deepcopy(EXPECTED_METADATA),
            "result": _expected_config(RESULT_HISTORY_PATH),
            "history": _expected_config(SOURCE_HISTORY_PATH),
        },
        "outcome": {
            "trajectory_convention": "240_post_step_states",
            "source_terminal": copy.deepcopy(SOURCE_TERMINAL),
            "analytic_state": copy.deepcopy(ANALYTIC_STATE),
            "source_minus_analytic": {
                "vx": SOURCE_TERMINAL["vx"] - ANALYTIC_STATE["vx"],
                "wy": SOURCE_TERMINAL["wy"] - ANALYTIC_STATE["wy"],
                "vc": 0.0,
            },
        },
        "diagnostics": copy.deepcopy(EXPECTED_HISTORY_SUMMARY),
        "predicates": {
            "artifact_valid": True,
            "source_identity_bound": True,
            "literal_invocation_recorded": True,
            "configuration_bound": True,
            "trajectory_contract_valid": True,
            "all_runner_converged_flags_true": True,
            "all_configured_terminal_gates_pass": True,
            "timing_verdict": None,
            "paper_parity": False,
            "dart_equivalence": False,
            "current_source_numeric_reference_valid": True,
        },
        "nonclaims": list(NONCLAIMS),
        "omitted_redundant_artifact": {
            "path": f"runs/{RUN_ID}/sweep_results.json",
            "source_bytes": RAW_ARTIFACTS["sweep_results.json"]["bytes"],
            "source_sha256": RAW_ARTIFACTS["sweep_results.json"]["sha256"],
            "reason": "semantic duplicate of the retained result.json payload",
        },
        "artifact_count": len(EXPECTED_ARTIFACTS),
        "stored_artifact_bytes": sum(
            int(record["bytes"]) for record in EXPECTED_ARTIFACTS
        ),
        "artifacts": copy.deepcopy(list(EXPECTED_ARTIFACTS)),
    }


def _validate_manifest(value: Any) -> dict[str, Any]:
    expected = _expected_manifest()
    if not _exact_json_equal(value, expected):
        raise ValueError(
            "author backspin manifest changed or promotes a forbidden claim"
        )
    return value


def _validate_metadata(value: Any) -> dict[str, Any]:
    _expect_keys(value, METADATA_KEYS, "metadata")
    _expect_exact(value, EXPECTED_METADATA, "metadata")
    return value


def _validate_result(value: Any) -> dict[str, Any]:
    result = _expect_keys(value, RESULT_KEYS, "result")
    expected_scalars = {
        "T_seconds": 4.0,
        "dt": DT_SECONDS,
        "mu": 0.5,
        "num_steps": TOTAL_STEPS,
        "omega0": -200.0,
        "solver": "fbf",
        "usd": None,
        "v0": 4.0,
        "vx_final": SOURCE_TERMINAL["vx"],
        "wy_final": SOURCE_TERMINAL["wy"],
    }
    for key, expected in expected_scalars.items():
        _expect_exact(result[key], expected, f"result.{key}")
    _expect_keys(result["config"], CONFIG_KEYS, "result.config")
    _expect_exact(
        result["config"], _expected_config(RESULT_HISTORY_PATH), "result.config"
    )
    times = result["step_times_ms"]
    if not isinstance(times, list) or len(times) != TOTAL_STEPS:
        raise ValueError("result.step_times_ms: expected 240 samples")
    checked_times = [
        _finite_float(value, f"result.step_times_ms[{index}]")
        for index, value in enumerate(times)
    ]
    if any(value <= 0.0 for value in checked_times):
        raise ValueError("result.step_times_ms: samples must be positive")
    average = _finite_float(result["avg_step_time_ms"], "result.avg_step_time_ms")
    if average <= 0.0 or not math.isclose(
        average, math.fsum(checked_times) / TOTAL_STEPS, rel_tol=1.0e-15
    ):
        raise ValueError("result.avg_step_time_ms: inconsistent with step samples")
    return result


def _validate_npz_container(path: Path) -> None:
    try:
        with zipfile.ZipFile(path) as archive:
            infos = archive.infolist()
            expected_names = [f"{key}.npy" for key in TRAJECTORY_KEYS]
            if [info.filename for info in infos] != expected_names:
                raise ValueError(
                    f"{path}: missing, reordered, or unexpected trajectory arrays"
                )
            if len({info.filename for info in infos}) != len(infos):
                raise ValueError(f"{path}: duplicate trajectory archive member")
            total = 0
            for info in infos:
                if info.is_dir() or PurePosixPath(info.filename).name != info.filename:
                    raise ValueError(f"{path}: unsafe trajectory archive member")
                if info.file_size > MAX_NPZ_MEMBER_BYTES:
                    raise ValueError(f"{path}: trajectory member exceeds size bound")
                total += info.file_size
            if total > MAX_NPZ_TOTAL_BYTES:
                raise ValueError(f"{path}: trajectory archive exceeds size bound")
    except zipfile.BadZipFile as error:
        raise ValueError(f"{path}: invalid trajectory NPZ") from error


def load_trajectory(path: Path) -> dict[str, np.ndarray]:
    """Load and validate the 240 post-step source trajectory arrays."""

    _validate_npz_container(path)
    try:
        with np.load(path, allow_pickle=False) as archive:
            if tuple(archive.files) != TRAJECTORY_KEYS:
                raise ValueError(
                    f"{path}: missing, reordered, or unexpected trajectory arrays"
                )
            arrays = {key: np.array(archive[key], copy=True) for key in TRAJECTORY_KEYS}
    except (OSError, ValueError, TypeError) as error:
        if isinstance(error, ValueError) and "trajectory arrays" in str(error):
            raise
        raise ValueError(f"{path}: could not load trajectory arrays") from error

    for key, array in arrays.items():
        if array.dtype != np.dtype("float64"):
            raise ValueError(f"{path}:{key}: expected float64 array")
        if array.shape != (TOTAL_STEPS,):
            raise ValueError(f"{path}:{key}: expected shape ({TOTAL_STEPS},)")
        if not bool(np.isfinite(array).all()):
            raise ValueError(f"{path}:{key}: array contains non-finite values")

    expected_times = np.cumsum(np.full(TOTAL_STEPS, DT_SECONDS, dtype=np.float64))
    if not np.array_equal(arrays["t"], expected_times):
        raise ValueError(f"{path}: t is not the exact post-step time grid")
    if not np.array_equal(arrays["vc"], arrays["vx"] - RADIUS * arrays["wy"]):
        raise ValueError(f"{path}: vc is inconsistent with vx - radius * wy")
    for key, expected in SOURCE_TERMINAL.items():
        if float(arrays[key][-1]) != expected:
            raise ValueError(f"{path}:{key}: source terminal value changed")
    return arrays


def _validate_history(value: Any) -> dict[str, Any]:
    history = _expect_keys(value, {"meta", "steps"}, "history")
    meta = _expect_keys(history["meta"], HISTORY_META_KEYS, "history.meta")
    bench = _expect_keys(meta["bench"], HISTORY_BENCH_KEYS, "history.meta.bench")
    _expect_exact(
        bench,
        {
            "git_sha": SOURCE_SHORT_COMMIT,
            "scene": "backspin-ball",
            "scene_params": {"mu": 0.5, "omega0": -200.0, "v0": 4.0},
            "total_steps": TOTAL_STEPS,
            "warmup_steps": 0,
        },
        "history.meta.bench",
    )
    for key, expected in {
        "dt": DT_SECONDS,
        "inner_solver": "block_gs",
        "num_bodies": 1,
        "schema_version": 2,
        "solver": "FBF",
    }.items():
        _expect_exact(meta[key], expected, f"history.meta.{key}")
    _expect_keys(meta["config"], CONFIG_KEYS, "history.meta.config")
    _expect_exact(
        meta["config"], _expected_config(SOURCE_HISTORY_PATH), "history.meta.config"
    )

    steps = history["steps"]
    if not isinstance(steps, list) or len(steps) != TOTAL_STEPS:
        raise ValueError("history.steps: expected exactly 240 steps")

    contact_indices: list[int] = []
    nonconverged: list[int] = []
    cap_steps: list[int] = []
    terminal_residuals: list[tuple[float, int]] = []
    final_residuals: list[tuple[float, int]] = []
    total_outer = 0
    max_outer = 0
    outer_solved = 0
    initial_shortcuts = 0
    no_contact = 0
    natural_above = 0
    warm_started = 0

    for index, raw_step in enumerate(steps):
        location = f"history.steps[{index}]"
        step = _expect_keys(raw_step, HISTORY_STEP_KEYS, location)
        for key in STEP_INTEGER_KEYS:
            _integer(step[key], f"{location}.{key}")
        for key in STEP_BOOLEAN_KEYS:
            if type(step[key]) is not bool:
                raise ValueError(f"{location}.{key}: expected a JSON boolean")
        for key in STEP_CPU_TIME_KEYS:
            if _finite_float(step[key], f"{location}.{key}") < 0.0:
                raise ValueError(f"{location}.{key}: expected nonnegative CPU time")
        for key in STEP_GPU_TIME_KEYS:
            if _finite_float(step[key], f"{location}.{key}") != -1.0:
                raise ValueError(f"{location}.{key}: expected CPU-run -1 sentinel")
        for key in (
            "W_lambda_max",
            "final_residual",
            "gamma_final",
            "gamma_init",
            "initial_residual",
        ):
            _finite_float(step[key], f"{location}.{key}")

        if step["step_idx"] != index:
            raise ValueError(f"{location}.step_idx: expected {index}")
        contacts = step["num_contacts"]
        if contacts not in (0, 1):
            raise ValueError(f"{location}.num_contacts: expected 0 or 1")
        outer_iters = step["outer_iters"]
        outer = step["outer"]
        if not isinstance(outer, list) or len(outer) != outer_iters:
            raise ValueError(f"{location}.outer: length disagrees with outer_iters")
        if not 0 <= outer_iters <= 200:
            raise ValueError(f"{location}.outer_iters: outside configured bound")
        if not step["converged"]:
            nonconverged.append(index)
        if step["warmup"]:
            raise ValueError(f"{location}.warmup: recorded run has no warmup steps")
        if step["warm_started"]:
            warm_started += 1
        if outer_iters == 200:
            cap_steps.append(index)
        total_outer += outer_iters
        max_outer = max(max_outer, outer_iters)

        if contacts:
            contact_indices.append(index)
            if step["warm_start_total"] != 1 or step["warm_start_matched"] != 0:
                raise ValueError(f"{location}: warm-start contact counts changed")
            if step["warm_started"]:
                raise ValueError(f"{location}: unexpected warm-start acceptance")
            if min(step["W_lambda_max"], step["gamma_init"]) <= 0.0:
                raise ValueError(f"{location}: invalid contact scaling diagnostics")
            if min(step["initial_residual"], step["final_residual"]) < 0.0:
                raise ValueError(f"{location}: natural residuals must be nonnegative")
            if outer_iters:
                outer_solved += 1
                if step["initial_residual"] < TERMINATION_TOLERANCE:
                    raise ValueError(
                        f"{location}: outer solve has a passing initial residual"
                    )
            else:
                initial_shortcuts += 1
                if not (
                    step["initial_residual"] < TERMINATION_TOLERANCE
                    and step["final_residual"] < TERMINATION_TOLERANCE
                ):
                    raise ValueError(f"{location}: invalid initial residual shortcut")
        else:
            no_contact += 1
            if outer_iters != 0 or outer:
                raise ValueError(f"{location}: no-contact step has outer iterations")
            for key, expected in {
                "W_lambda_max": -1.0,
                "gamma_init": 0.0,
                "gamma_final": -1.0,
                "initial_residual": -1.0,
                "final_residual": 0.0,
                "warm_start_matched": 0,
                "warm_start_total": 0,
                "warm_started": False,
            }.items():
                _expect_exact(step[key], expected, f"{location}.{key}")

        for outer_index, raw_outer in enumerate(outer):
            outer_location = f"{location}.outer[{outer_index}]"
            record = _expect_keys(raw_outer, HISTORY_OUTER_KEYS, outer_location)
            for key in OUTER_INTEGER_KEYS:
                _integer(record[key], f"{outer_location}.{key}")
            if record["k"] != outer_index:
                raise ValueError(f"{outer_location}.k: expected {outer_index}")
            if not 0 <= record["backtracks"] <= 8:
                raise ValueError(
                    f"{outer_location}.backtracks: outside configured bound"
                )
            if not 0 <= record["inner_iters"] <= 200:
                raise ValueError(
                    f"{outer_location}.inner_iters: outside configured bound"
                )
            if type(record["gamma_event"]) is not str or record["gamma_event"] not in {
                "",
                "carry",
                "restore",
                "shrink",
            }:
                raise ValueError(f"{outer_location}.gamma_event: unexpected value")
            for key in OUTER_CPU_TIME_KEYS:
                if _finite_float(record[key], f"{outer_location}.{key}") < 0.0:
                    raise ValueError(
                        f"{outer_location}.{key}: expected nonnegative CPU time"
                    )
            for key in OUTER_GPU_TIME_KEYS:
                if _finite_float(record[key], f"{outer_location}.{key}") != -1.0:
                    raise ValueError(
                        f"{outer_location}.{key}: expected CPU-run -1 sentinel"
                    )
            for key in (
                HISTORY_OUTER_KEYS
                - OUTER_INTEGER_KEYS
                - {
                    "gamma_event",
                    *OUTER_CPU_TIME_KEYS,
                    *OUTER_GPU_TIME_KEYS,
                }
            ):
                value = _finite_float(record[key], f"{outer_location}.{key}")
                if value < -1.0:
                    raise ValueError(
                        f"{outer_location}.{key}: invalid numeric sentinel"
                    )
            if record["gamma"] <= 0.0 or record["gamma_start"] <= 0.0:
                raise ValueError(f"{outer_location}: gamma must be positive")

        if outer:
            terminal = outer[-1]
            if terminal["r_coulomb"] < 0.0:
                raise ValueError(
                    f"{location}: terminal Coulomb residual is unavailable"
                )
            if step["converged"] != (terminal["r_coulomb"] < TERMINATION_TOLERANCE):
                raise ValueError(
                    f"{location}: converged flag disagrees with configured terminal gate"
                )
            if step["final_residual"] != terminal["residual"]:
                raise ValueError(f"{location}: final natural residual changed")
            if step["gamma_final"] != terminal["gamma"]:
                raise ValueError(f"{location}: final gamma changed")
            terminal_residuals.append((terminal["r_coulomb"], index))
        elif contacts and not step["converged"]:
            raise ValueError(f"{location}: passing initial shortcut is not converged")

        if step["converged"] and step["final_residual"] > TERMINATION_TOLERANCE:
            natural_above += 1
        final_residuals.append((step["final_residual"], index))

    max_final, max_final_step = max(final_residuals)
    max_terminal, max_terminal_step = max(terminal_residuals)
    summary = {
        "step_count": len(steps),
        "converged_steps": len(steps) - len(nonconverged),
        "nonconverged_steps": nonconverged,
        "contact_steps": len(contact_indices),
        "first_contact_step": min(contact_indices),
        "last_contact_step": max(contact_indices),
        "max_contacts": max(step["num_contacts"] for step in steps),
        "outer_solved_steps": outer_solved,
        "initial_shortcut_steps": initial_shortcuts,
        "no_contact_steps": no_contact,
        "total_outer_iterations": total_outer,
        "max_outer_iterations": max_outer,
        "outer_cap_steps": cap_steps,
        "natural_residual_above_tolerance_converged_steps": natural_above,
        "max_final_natural_residual": max_final,
        "max_final_natural_residual_step": max_final_step,
        "max_terminal_coulomb_residual": max_terminal,
        "max_terminal_coulomb_residual_step": max_terminal_step,
        "warm_started_steps": warm_started,
    }
    _expect_exact(summary, EXPECTED_HISTORY_SUMMARY, "history diagnostics")
    return summary


def _validate_sweep(value: Any, result: dict[str, Any]) -> None:
    if not isinstance(value, list) or len(value) != 1:
        raise ValueError("sweep_results.json: expected one result")
    _expect_exact(value[0], result, "sweep_results.json[0]")


def _validate_payloads(
    metadata_bytes: bytes,
    result_bytes: bytes,
    trajectory_path: Path,
    history_bytes: bytes,
    sweep_bytes: bytes | None = None,
) -> dict[str, Any]:
    metadata = _validate_metadata(_parse_json_bytes(metadata_bytes, "metadata"))
    result = _validate_result(_parse_json_bytes(result_bytes, "result"))
    trajectory = load_trajectory(trajectory_path)
    history = _parse_json_bytes(history_bytes, "history")
    diagnostics = _validate_history(history)
    if sweep_bytes is not None:
        _validate_sweep(_parse_json_bytes(sweep_bytes, "sweep_results.json"), result)
    if result["vx_final"] != float(trajectory["vx"][-1]):
        raise ValueError("result.vx_final disagrees with trajectory")
    if result["wy_final"] != float(trajectory["wy"][-1]):
        raise ValueError("result.wy_final disagrees with trajectory")
    if metadata["num_steps"] != len(trajectory["t"]):
        raise ValueError("metadata.num_steps disagrees with trajectory")
    return diagnostics


def _source_run_members(root: Path) -> set[str]:
    members: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"source run contains symlink: {relative}")
        if stat.S_ISREG(mode):
            members.add(relative)
        elif not stat.S_ISDIR(mode):
            raise ValueError(f"source run contains non-regular entry: {relative}")
    return members


def _load_source_run(source_repo: Path) -> dict[str, bytes]:
    run_root = source_repo / SOURCE_RUN_DIRECTORY
    expected_members = set(RAW_ARTIFACTS)
    if not run_root.is_dir() or run_root.is_symlink():
        raise ValueError(f"source run directory is missing: {run_root}")
    actual_members = _source_run_members(run_root)
    if actual_members != expected_members:
        raise ValueError(
            "source run membership changed: "
            f"expected {sorted(expected_members)}, got {sorted(actual_members)}"
        )

    payloads: dict[str, bytes] = {}
    for relative, record in RAW_ARTIFACTS.items():
        path = run_root / relative
        data = path.read_bytes()
        if len(data) != record["bytes"] or _sha256_bytes(data) != record["sha256"]:
            raise ValueError(f"source artifact identity changed: {relative}")
        payloads[relative] = data

    _validate_payloads(
        payloads["metadata.json"],
        payloads["fbf/result.json"],
        run_root / "fbf/trajectory.npz",
        payloads["fbf/history.json"],
        payloads["sweep_results.json"],
    )
    return payloads


def _gzip_n9(data: bytes) -> bytes:
    output = io.BytesIO()
    with gzip.GzipFile(
        filename="", mode="wb", fileobj=output, compresslevel=9, mtime=0
    ) as stream:
        stream.write(data)
    return output.getvalue()


def _decompress_history(path: Path, record: dict[str, Any]) -> bytes:
    compressed = path.read_bytes()
    if compressed[:10] != b"\x1f\x8b\x08\x00\x00\x00\x00\x00\x02\xff":
        raise ValueError(f"{path}: expected deterministic gzip-n9 header")
    raw_bytes = int(record["raw_bytes"])
    output = bytearray()
    decoder = zlib.decompressobj(wbits=16 + zlib.MAX_WBITS)
    try:
        for offset in range(0, len(compressed), GZIP_CHUNK_BYTES):
            pending = compressed[offset : offset + GZIP_CHUNK_BYTES]
            while pending:
                remaining = raw_bytes - len(output)
                output.extend(decoder.decompress(pending, remaining + 1))
                if len(output) > raw_bytes:
                    raise ValueError(f"{path}: decompressed history exceeds size bound")
                pending = decoder.unconsumed_tail
        output.extend(decoder.flush(raw_bytes - len(output) + 1))
    except zlib.error as error:
        raise ValueError(f"{path}: invalid gzip history") from error
    if not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise ValueError(f"{path}: truncated or concatenated gzip history")
    raw = bytes(output)
    if len(raw) != raw_bytes or _sha256_bytes(raw) != record["raw_sha256"]:
        raise ValueError(f"{path}: decompressed history identity changed")
    return raw


def _expected_directories() -> set[str]:
    directories: set[str] = set()
    for relative in EXPECTED_ARTIFACT_PATHS:
        parent = PurePosixPath(relative).parent
        while parent != PurePosixPath("."):
            directories.add(parent.as_posix())
            parent = parent.parent
    return directories


def _validate_membership(bundle: Path) -> None:
    if bundle.is_symlink() or not bundle.is_dir():
        raise ValueError("author backspin bundle must be a regular directory")
    files: set[str] = set()
    directories: set[str] = set()
    for path in bundle.rglob("*"):
        relative = path.relative_to(bundle).as_posix()
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"author backspin bundle contains symlink: {relative}")
        if stat.S_ISREG(mode):
            files.add(relative)
        elif stat.S_ISDIR(mode):
            directories.add(relative)
        else:
            raise ValueError(
                f"author backspin bundle contains non-regular entry: {relative}"
            )
    expected_files = EXPECTED_ARTIFACT_PATHS | {"manifest.json"}
    if files != expected_files:
        raise ValueError(
            "author backspin bundle file membership changed: "
            f"expected {sorted(expected_files)}, got {sorted(files)}"
        )
    expected_directories = _expected_directories()
    if directories != expected_directories:
        raise ValueError(
            "author backspin bundle directory membership changed: "
            f"expected {sorted(expected_directories)}, got {sorted(directories)}"
        )


def validate_bundle(bundle: Path = DEFAULT_BUNDLE) -> dict[str, Any]:
    bundle = _reject_symlink_components(bundle, "bundle")
    _validate_membership(bundle)
    manifest = _validate_manifest(_read_json(bundle / "manifest.json"))
    by_path = {record["path"]: record for record in manifest["artifacts"]}
    for relative in EXPECTED_ARTIFACT_PATHS:
        _safe_relative(relative, "manifest artifact path")
        record = by_path[relative]
        path = bundle / relative
        if path.stat().st_size != record["bytes"] or _sha256(path) != record["sha256"]:
            raise ValueError(f"bundle artifact identity changed: {relative}")

    prefix = bundle / "runs" / RUN_ID
    history_record = by_path[f"runs/{RUN_ID}/fbf/history.json.gz"]
    diagnostics = _validate_payloads(
        (prefix / "metadata.json").read_bytes(),
        (prefix / "fbf/result.json").read_bytes(),
        prefix / "fbf/trajectory.npz",
        _decompress_history(prefix / "fbf/history.json.gz", history_record),
    )
    return {
        "schema_version": VALIDATION_SCHEMA_VERSION,
        "status": STATUS,
        "bundle": str(bundle),
        "artifact_count": len(EXPECTED_ARTIFACTS),
        "trajectory_state_count": TOTAL_STEPS,
        "converged_steps": diagnostics["converged_steps"],
        "terminal": copy.deepcopy(SOURCE_TERMINAL),
        "analytic_state": copy.deepcopy(ANALYTIC_STATE),
        "paper_parity": False,
        "dart_equivalence": False,
    }


def _publish_staged_bundle(staging: Path, bundle: Path) -> dict[str, Any]:
    backup = bundle.with_name(f".{bundle.name}.backup-{uuid.uuid4().hex}")
    had_existing = bundle.exists() or bundle.is_symlink()
    try:
        if had_existing:
            os.replace(bundle, backup)
        os.replace(staging, bundle)
        report = validate_bundle(bundle)
    except BaseException:
        _remove_path(bundle)
        if had_existing and backup.exists():
            os.replace(backup, bundle)
        raise
    else:
        _remove_path(backup)
        return report
    finally:
        _remove_path(staging)


def finalize_bundle(
    source_repo: Path = DEFAULT_SOURCE_REPO, bundle: Path = DEFAULT_BUNDLE
) -> dict[str, Any]:
    source_repo = _reject_symlink_components(source_repo, "source repository")
    bundle = _reject_symlink_components(bundle, "bundle")
    _validate_source_identity(source_repo)
    payloads = _load_source_run(source_repo)

    bundle.parent.mkdir(parents=True, exist_ok=True)
    staging = bundle.with_name(f".{bundle.name}.staging-{uuid.uuid4().hex}")
    if staging.exists() or staging.is_symlink():
        raise ValueError(f"staging path unexpectedly exists: {staging}")
    staging.mkdir()
    try:
        prefix = staging / "runs" / RUN_ID
        _write_bytes(prefix / "metadata.json", payloads["metadata.json"])
        _write_bytes(prefix / "fbf/result.json", payloads["fbf/result.json"])
        _write_bytes(prefix / "fbf/trajectory.npz", payloads["fbf/trajectory.npz"])
        _write_bytes(
            prefix / "fbf/history.json.gz",
            _gzip_n9(payloads["fbf/history.json"]),
        )
        _write_json(staging / "manifest.json", _expected_manifest())
        validate_bundle(staging)
        _validate_source_identity(source_repo)
        return _publish_staged_bundle(staging, bundle)
    except BaseException:
        _remove_path(staging)
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", type=Path, default=DEFAULT_SOURCE_REPO)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument(
        "--verify-only", action="store_true", help="validate an existing sealed bundle"
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        report = (
            validate_bundle(args.bundle)
            if args.verify_only
            else finalize_bundle(args.source_repo, args.bundle)
        )
    except (OSError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
