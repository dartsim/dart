#!/usr/bin/env python3
"""Seal and verify the pinned current-author incline comparison sweep.

This packet is a current-source numeric diagnostic.  It deliberately does not
turn independent FBF, MuJoCo, and Kamino runs into a cross-solver parity or
timing claim.  In particular, the author FBF ``converged`` flag is validated
against its configured ``coulomb_rel`` gate instead of being inferred from the
separately emitted natural ``final_residual`` value.
"""

from __future__ import annotations

import argparse
import csv
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
from pathlib import Path, PurePosixPath
from typing import Any, Sequence

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_REPO = Path("/tmp/fbf-sca-2026-author")
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "author_incline_sweep_reference_v1"
)

SCHEMA_VERSION = "dart.fbf_author_incline_sweep_reference/v1"
VALIDATION_SCHEMA_VERSION = "dart.fbf_author_incline_sweep_validation/v1"
STATUS = "valid_current_source_scientific_negative"

SOURCE_REPOSITORY = "https://github.com/matthcsong/fbf-sca-2026.git"
SOURCE_COMMIT = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
SOURCE_TREE = "ffcdafb61adeda2239c8366d054b548b50d26685"
SOURCE_SHORT_COMMIT = "b3f3c5c"
RUNNER_PATH = "paper_examples/cube-on-incline/run.py"
RUNNER_SHA256 = "881d486f25d85f9ae197bf4164e110b6bc39775c498433f27ec4407ca09ebf82"
RUNNER_GIT_BLOB = "63cfc28dca1f6c65ce4a27dbfa239cba154580c6"
REQUIREMENTS_PATH = "requirements.txt"
REQUIREMENTS_SHA256 = "436d6a280e62f0cf5b670bf0475cbed6e95fa87e2fb011bc30334355a3ed9688"
RECORDED_PYTHON = Path("/tmp/fbf-author-venv/bin/python")
RECORDED_PYTHON_RESOLVED_PATH = (
    "/home/js/.local/share/uv/python/cpython-3.11.15-linux-x86_64-gnu/bin/python3.11"
)
RECORDED_PYTHON_SHA256 = (
    "6759dcc6aa5a3fe3377349092f92f6700f1f79017294c1d0085009eb8802cb1e"
)
RECORDED_RUNTIME_PROBE = {
    "platform": "Linux-7.0.0-27-generic-x86_64-with-glibc2.43",
    "python": "3.11.15 (main, May 10 2026, 19:28:18) [Clang 22.1.3 ]",
}

MU_VALUES = (0.3, 0.4, 0.45, 0.5, 0.55, 0.6, 0.8)
DT_SECONDS = 1.0 / 60.0
STEPS_PER_CELL = 120
CONTACTS_PER_STEP = 4
TERMINATION_TOLERANCE = 1.0e-6

METADATA_KEYS = {
    "T_seconds",
    "cube_half_extent",
    "device",
    "dt",
    "experiment",
    "gap",
    "gravity",
    "mu_star",
    "mu_values",
    "num_steps",
    "plane_hx",
    "plane_hy",
    "plane_hz",
    "solvers",
    "theta_deg",
    "timestamp",
}
RESULT_KEYS = {
    "T_seconds",
    "avg_outer_iters_per_step",
    "avg_step_time_ms",
    "dt",
    "final_residual",
    "mu",
    "normal_dist",
    "num_steps",
    "solver",
    "step_times_ms",
    "tangential_disp",
    "tangential_vel",
    "total_outer_iters",
    "usd",
}
FBF_CONFIG_KEYS = {
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
HISTORY_STEP_INTEGER_KEYS = {
    "num_contacts",
    "outer_iters",
    "step_idx",
    "warm_start_matched",
    "warm_start_total",
}
HISTORY_STEP_BOOLEAN_KEYS = {"converged", "warm_started", "warmup"}
HISTORY_STEP_CPU_TIME_KEYS = {
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
HISTORY_STEP_GPU_TIME_KEYS = {
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
HISTORY_OUTER_INTEGER_KEYS = {"backtracks", "inner_iters", "k"}
HISTORY_OUTER_CPU_TIME_KEYS = {
    "t_correction_ms",
    "t_fwd_bwd_ms",
    "t_fwd_eval_ms",
    "t_residual_check_ms",
}
HISTORY_OUTER_GPU_TIME_KEYS = {
    "t_correction_gpu_ms",
    "t_fwd_bwd_gpu_ms",
    "t_fwd_eval_gpu_ms",
    "t_residual_check_gpu_ms",
}
HISTORY_OUTER_RESIDUAL_KEYS = {
    "eps_force",
    "eps_gap",
    "eps_vel",
    "r_coulomb",
    "r_dual",
    "r_gap",
    "r_kkt",
    "r_primal",
    "residual",
}

RUN_SPECS = (
    {
        "solver": "fbf",
        "run_id": "20260719T151337Z",
        "profile": True,
    },
    {
        "solver": "mujoco",
        "run_id": "20260719T151857Z",
        "profile": False,
    },
    {
        "solver": "kamino",
        "run_id": "20260719T152022Z",
        "profile": False,
    },
)

EXPECTED_DISPLACEMENTS = {
    "fbf": (
        3.5392831695743054,
        1.7698922978656797,
        0.8851976117115778,
        0.0005018926371855115,
        0.0004281487924409106,
        0.00035627086822120473,
        0.00006883913936487685,
    ),
    "mujoco": (
        3.5794330878127263,
        2.1118162364510042,
        1.8350010889171866,
        0.35929869745695187,
        1.8135318873257669,
        1.2524510782262557,
        0.18109838262409816,
    ),
    "kamino": (
        3.5392822099580354,
        1.7698918846975635,
        0.8851977916396285,
        0.0005009063649080669,
        0.0004281487924409106,
        0.0003563241802362017,
        0.00006887912337612457,
    ),
}

EXPECTED_FBF_COUNTS = {
    "history_steps": 840,
    "contact_records": 3360,
    "converged_flags": 839,
    "nonconverged_flags": 1,
    "initial_natural_shortcut_accepts": 235,
    "configured_outer_gate_accepts": 604,
    "true_flags_natural_residual_at_or_below_tolerance": 456,
    "true_flags_natural_residual_above_tolerance": 383,
    "outer_cap_hits": 1,
}

EXPECTED_FBF_PER_MU = {
    0.3: (120, 0, 6, 114, 20, 100, 45),
    0.4: (120, 0, 0, 120, 8, 112, 60),
    0.45: (120, 0, 10, 110, 35, 85, 70),
    0.5: (120, 0, 13, 107, 35, 85, 180),
    0.55: (119, 1, 64, 55, 119, 0, 200),
    0.6: (120, 0, 70, 50, 120, 0, 130),
    0.8: (120, 0, 72, 48, 119, 1, 180),
}

CLAIM_SCOPE = (
    "Pinned current-author CPU diagnostic over all seven friction cells exposed "
    "by the public cube-on-incline runner, preserving independent FBF, MuJoCo, "
    "and Kamino results plus every FBF residual-history step."
)
CLAIM_BOUNDARY = (
    "The three solver lanes are independent current-source runs. FBF has one "
    "configured nonconvergence at mu=.55 step 1; natural final_residual and the "
    "configured nonnegative coulomb_rel gate are distinct. Source step times "
    "include uncontrolled first-use/JIT and instrumentation effects and are "
    "diagnostic only. This is not a historical paper invocation, full published "
    "sweep, DART comparison, full-state or cross-solver parity result, approved "
    "golden, renderer/media evidence, paper timing, real-time, or performance "
    "evidence."
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def _write_json(path: Path, value: Any) -> None:
    _write_bytes(path, _json_bytes(value))


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON object key {key!r}")
        result[key] = value
    return result


def _json_constant(value: str) -> float:
    if value == "NaN":
        return math.nan
    raise ValueError(f"unsupported JSON constant {value!r}")


def _load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(
            stream,
            object_pairs_hook=_unique_object,
            parse_constant=_json_constant,
        )


def _run_git(source_repo: Path, *args: str) -> str:
    process = subprocess.run(
        ["git", "-C", str(source_repo), *args],
        check=False,
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise ValueError(
            f"git {' '.join(args)} failed: {process.stderr.strip() or process.stdout.strip()}"
        )
    return process.stdout.strip()


def _finite_number(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{location}: expected a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{location}: expected a finite number")
    return result


def _close(actual: Any, expected: float, location: str, tol: float = 1.0e-12) -> None:
    value = _finite_number(actual, location)
    if not math.isclose(value, expected, rel_tol=tol, abs_tol=tol):
        raise ValueError(f"{location}: expected {expected!r}, got {value!r}")


def _finite_float(value: Any, location: str) -> float:
    if type(value) is not float or not math.isfinite(value):
        raise ValueError(f"{location}: expected a finite JSON float")
    return value


def _integer(value: Any, location: str) -> int:
    if type(value) is not int:
        raise ValueError(f"{location}: expected a JSON integer")
    return value


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
    if isinstance(expected, float) and math.isnan(expected):
        return math.isnan(actual)
    return actual == expected


def _expect_exact(value: Any, expected: Any, location: str) -> None:
    if not _exact_json_equal(value, expected):
        raise ValueError(f"{location}: exact JSON value or type changed")


def _expect_exact_keys(value: Any, expected: set[str], location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{location}: expected an object")
    actual = set(value)
    if actual != expected:
        raise ValueError(
            f"{location}: exact keys changed; missing={sorted(expected - actual)}, extra={sorted(actual - expected)}"
        )
    return value


def _safe_relative(value: str, location: str) -> PurePosixPath:
    if not isinstance(value, str):
        raise ValueError(f"{location}: expected a path string")
    path = PurePosixPath(value)
    if (
        not value
        or not path.parts
        or path.is_absolute()
        or "\\" in value
        or "\x00" in value
        or any(part in {"", ".", ".."} for part in path.parts)
        or path.as_posix() != value
    ):
        raise ValueError(f"{location}: unsafe or noncanonical relative path {value!r}")
    return path


def _absolute_without_symlink_components(path: Path, location: str) -> Path:
    absolute = Path(os.path.abspath(path.expanduser()))
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


def _source_identity(source_repo: Path) -> dict[str, Any]:
    source_repo = _absolute_without_symlink_components(source_repo, "source repository")
    if source_repo.is_symlink() or not source_repo.is_dir():
        raise ValueError(f"source repository is not a regular directory: {source_repo}")
    if _run_git(source_repo, "rev-parse", "HEAD") != SOURCE_COMMIT:
        raise ValueError("source repository commit changed")
    if _run_git(source_repo, "rev-parse", "HEAD^{tree}") != SOURCE_TREE:
        raise ValueError("source repository tree changed")
    if _run_git(source_repo, "remote", "get-url", "origin") != SOURCE_REPOSITORY:
        raise ValueError("source repository origin changed")
    if _run_git(source_repo, "status", "--porcelain", "--untracked-files=no"):
        raise ValueError("source repository has tracked modifications")

    runner = source_repo / RUNNER_PATH
    requirements = source_repo / REQUIREMENTS_PATH
    if runner.is_symlink() or requirements.is_symlink():
        raise ValueError("source identity files must not be symlinks")
    if _sha256(runner) != RUNNER_SHA256:
        raise ValueError("source runner digest changed")
    if _run_git(source_repo, "hash-object", RUNNER_PATH) != RUNNER_GIT_BLOB:
        raise ValueError("source runner git blob changed")
    if _sha256(requirements) != REQUIREMENTS_SHA256:
        raise ValueError("source requirements digest changed")

    if RECORDED_PYTHON.is_symlink():
        python_resolved = RECORDED_PYTHON.resolve(strict=True)
    else:
        python_resolved = RECORDED_PYTHON
    if not python_resolved.is_file():
        raise ValueError(
            f"recorded Python executable is unavailable: {RECORDED_PYTHON}"
        )
    process = subprocess.run(
        [
            str(RECORDED_PYTHON),
            "-c",
            "import json,platform,sys; print(json.dumps({'platform': platform.platform(), 'python': sys.version}, sort_keys=True))",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if process.returncode != 0:
        raise ValueError(f"recorded Python probe failed: {process.stderr.strip()}")
    runtime = json.loads(
        process.stdout,
        object_pairs_hook=_unique_object,
        parse_constant=_json_constant,
    )
    identity = {
        "repository": SOURCE_REPOSITORY,
        "commit": SOURCE_COMMIT,
        "tree": SOURCE_TREE,
        "tracked_clean_before": True,
        "tracked_clean_after": True,
        "runner_path": RUNNER_PATH,
        "runner_sha256": RUNNER_SHA256,
        "runner_git_blob": RUNNER_GIT_BLOB,
        "requirements_path": REQUIREMENTS_PATH,
        "requirements_sha256": REQUIREMENTS_SHA256,
        "recorded_python_path": str(RECORDED_PYTHON),
        "recorded_python_resolved_path": str(python_resolved),
        "recorded_python_sha256": _sha256(python_resolved),
        "recorded_runtime_probe": runtime,
        "runtime_attestation_complete": False,
    }
    _expect_exact(identity, _expected_source_identity(), "source identity")
    return identity


def _expected_source_identity() -> dict[str, Any]:
    return {
        "repository": SOURCE_REPOSITORY,
        "commit": SOURCE_COMMIT,
        "tree": SOURCE_TREE,
        "tracked_clean_before": True,
        "tracked_clean_after": True,
        "runner_path": RUNNER_PATH,
        "runner_sha256": RUNNER_SHA256,
        "runner_git_blob": RUNNER_GIT_BLOB,
        "requirements_path": REQUIREMENTS_PATH,
        "requirements_sha256": REQUIREMENTS_SHA256,
        "recorded_python_path": str(RECORDED_PYTHON),
        "recorded_python_resolved_path": RECORDED_PYTHON_RESOLVED_PATH,
        "recorded_python_sha256": RECORDED_PYTHON_SHA256,
        "recorded_runtime_probe": dict(RECORDED_RUNTIME_PROBE),
        "runtime_attestation_complete": False,
    }


def _argv(solver: str, profile: bool) -> list[str]:
    argv = [
        str(RECORDED_PYTHON),
        RUNNER_PATH,
        "--solvers",
        solver,
        "--mu",
        *(str(value) for value in MU_VALUES),
        "--device",
        "cpu",
    ]
    if profile:
        argv.append("--profile")
    return argv


def _source_run_root(source_repo: Path, run_id: str) -> Path:
    return source_repo / "paper_examples/cube-on-incline/results" / run_id


def _result_relative(run_id: str, solver: str, mu: float) -> str:
    return f"runs/{run_id}/mu{mu:.4f}_{solver}/result.json"


def _history_relative(run_id: str, mu: float) -> str:
    return f"runs/{run_id}/mu{mu:.4f}_fbf/history.json.gz"


def _source_artifact_paths() -> tuple[str, ...]:
    paths: list[str] = []
    for spec in RUN_SPECS:
        run_id = str(spec["run_id"])
        solver = str(spec["solver"])
        paths.extend(
            (f"runs/{run_id}/metadata.json", f"runs/{run_id}/sweep_results.json")
        )
        for mu in MU_VALUES:
            paths.append(_result_relative(run_id, solver, mu))
            if solver == "fbf":
                paths.append(_history_relative(run_id, mu))
    return tuple(paths)


SOURCE_ARTIFACT_PATHS = _source_artifact_paths()
DERIVED_ARTIFACT_PATHS = ("comparison.csv", "comparison.svg", "REPORT.md")
EXACT_BUNDLE_MEMBERS = tuple(
    sorted(
        (
            *SOURCE_ARTIFACT_PATHS,
            *DERIVED_ARTIFACT_PATHS,
            "manifest.json",
            "verification.json",
        )
    )
)


def _expected_source_path(bundle_relative: str) -> str:
    if bundle_relative not in SOURCE_ARTIFACT_PATHS or not bundle_relative.startswith(
        "runs/"
    ):
        raise ValueError(f"unexpected source artifact path {bundle_relative!r}")
    run_relative = bundle_relative.removeprefix("runs/")
    if run_relative.endswith(".json.gz"):
        run_relative = run_relative.removesuffix(".gz")
    return f"paper_examples/cube-on-incline/results/{run_relative}"


def _expected_source_members(solver: str) -> set[str]:
    members = {"metadata.json", "sweep_results.json"}
    for mu in MU_VALUES:
        directory = f"mu{mu:.4f}_{solver}"
        members.add(f"{directory}/result.json")
        if solver == "fbf":
            members.add(f"{directory}/history.json")
    return members


def _regular_members(root: Path) -> set[str]:
    root = _absolute_without_symlink_components(root, "bundle/run root")
    if root.is_symlink() or not root.is_dir():
        raise ValueError(f"bundle/run root is not a regular directory: {root}")
    members: set[str] = set()
    for path in root.rglob("*"):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise ValueError(f"contains symlink {relative!r}")
        if path.is_file():
            members.add(relative)
        elif not path.is_dir():
            raise ValueError(f"contains non-regular entry {relative!r}")
    return members


def _expected_fbf_config(history_path: str) -> dict[str, Any]:
    return {
        "gamma": None,
        "max_outer": 200,
        "outer_tol": TERMINATION_TOLERANCE,
        "residual_check_interval": 5,
        "inner_max_iter": 200,
        "inner_tol": TERMINATION_TOLERANCE,
        "inner_step_size": None,
        "inner_solver": "block_gs",
        "inner_gs_sweeps": 10,
        "max_contacts": 4096,
        "baumgarte_erp": 0.0,
        "project_after_correction": False,
        "warm_start": True,
        "warm_start_match_radius": 0.02,
        "warm_start_normal_cos": 0.9,
        "warm_start_max_age": 3,
        "gamma_min": 1.0e-6,
        "gamma_max": 1.0e6,
        "gamma_c": 0.5,
        "adaptive_gamma": True,
        "armijo_rho_high": 0.9,
        "armijo_shrink": 0.7,
        "armijo_grow": 1.4285714285714286,
        "armijo_skip_threshold": 1.0e-10,
        "armijo_max_backtracks": 8,
        "plateau_patience": 5,
        "plateau_rtol": 0.01,
        "warm_start_gamma_threshold": 1.0e-4,
        "warm_start_gamma_cap": 1.0e4,
        "history_path": history_path,
        "termination_residual": "coulomb_rel",
        "termination_tol": TERMINATION_TOLERANCE,
        "per_contact_diag_dir": None,
        "per_contact_diag_topk": 20,
        "dump_step_state_idx": None,
        "dump_step_state_path": None,
    }


def _history_source_path(run_id: str, mu: float) -> str:
    return (
        f"paper_examples/cube-on-incline/results/{run_id}/"
        f"mu{mu:.4f}_fbf/history.json"
    )


def _validate_metadata(metadata: Any, solver: str, run_id: str) -> None:
    location = f"{run_id}.metadata"
    _expect_exact_keys(metadata, METADATA_KEYS, location)
    expected = {
        "experiment": "cube_on_incline_sweep",
        "mu_values": list(MU_VALUES),
        "num_steps": STEPS_PER_CELL,
        "solvers": [solver],
        "device": "cpu",
        "timestamp": run_id,
        "theta_deg": 26.56505117707799,
        "mu_star": 0.5,
        "cube_half_extent": 0.5,
        "gap": 0.001,
        "gravity": 9.81,
        "dt": DT_SECONDS,
        "T_seconds": 2.0,
        "plane_hx": 5.0,
        "plane_hy": 1.5,
        "plane_hz": 0.05,
    }
    _expect_exact(metadata, expected, location)


def _validate_result(
    result: Any, solver: str, mu: float, run_id: str, location: str
) -> None:
    expected_keys = RESULT_KEYS | ({"config"} if solver == "fbf" else set())
    _expect_exact_keys(result, expected_keys, location)
    for key, expected in {
        "solver": solver,
        "mu": mu,
        "dt": DT_SECONDS,
        "num_steps": STEPS_PER_CELL,
        "T_seconds": 2.0,
        "usd": None,
    }.items():
        _expect_exact(result[key], expected, f"{location}.{key}")
    index = MU_VALUES.index(mu)
    _finite_float(result["tangential_disp"], f"{location}.tangential_disp")
    _close(
        result["tangential_disp"],
        EXPECTED_DISPLACEMENTS[solver][index],
        f"{location}.tangential_disp",
    )
    for key in ("tangential_vel", "normal_dist", "avg_step_time_ms"):
        _finite_float(result[key], f"{location}.{key}")
    step_times = result["step_times_ms"]
    if not isinstance(step_times, list) or len(step_times) != STEPS_PER_CELL:
        raise ValueError(f"{location}.step_times_ms: expected 120 samples")
    for index_, value in enumerate(step_times):
        _finite_float(value, f"{location}.step_times_ms[{index_}]")
    _close(
        result["avg_step_time_ms"],
        sum(step_times) / STEPS_PER_CELL,
        f"{location}.avg_step_time_ms",
    )
    total_outer = _integer(result["total_outer_iters"], f"{location}.total_outer_iters")
    if total_outer < 0:
        raise ValueError(
            f"{location}.total_outer_iters: expected a nonnegative integer"
        )
    if solver == "fbf":
        _finite_float(result["final_residual"], f"{location}.final_residual")
        _finite_float(
            result["avg_outer_iters_per_step"],
            f"{location}.avg_outer_iters_per_step",
        )
        _close(
            result["avg_outer_iters_per_step"],
            total_outer / STEPS_PER_CELL,
            f"{location}.avg_outer_iters_per_step",
        )
        config = _expect_exact_keys(
            result["config"], FBF_CONFIG_KEYS, f"{location}.config"
        )
        _expect_exact(
            config,
            _expected_fbf_config(
                str(DEFAULT_SOURCE_REPO / _history_source_path(run_id, mu))
            ),
            f"{location}.config",
        )
    else:
        final_residual = result["final_residual"]
        if type(final_residual) is not float or not math.isnan(final_residual):
            raise ValueError(f"{location}.final_residual: expected JSON NaN")
        _expect_exact(
            result["avg_outer_iters_per_step"],
            0,
            f"{location}.avg_outer_iters_per_step",
        )
        if total_outer != 0:
            raise ValueError(f"{location}.total_outer_iters: expected 0")


def _terminal_coulomb_residual(step: dict[str, Any], location: str) -> float | None:
    outer = step.get("outer")
    if not isinstance(outer, list):
        raise ValueError(f"{location}.outer: expected a list")
    for index in range(len(outer) - 1, -1, -1):
        record = outer[index]
        if not isinstance(record, dict):
            raise ValueError(f"{location}.outer[{index}]: expected an object")
        value = _finite_number(
            record.get("r_coulomb"), f"{location}.outer[{index}].r_coulomb"
        )
        if value >= 0.0:
            return value
    return None


def _validate_history(
    history: Any, mu: float, run_id: str, location: str
) -> dict[str, Any]:
    _expect_exact_keys(history, {"meta", "steps"}, location)
    meta = history["meta"]
    steps = history["steps"]
    _expect_exact_keys(meta, HISTORY_META_KEYS, f"{location}.meta")
    if not isinstance(steps, list):
        raise ValueError(f"{location}.steps: expected a list")
    if len(steps) != STEPS_PER_CELL:
        raise ValueError(f"{location}.steps: expected 120 rows")
    for key, expected in {
        "schema_version": 2,
        "solver": "FBF",
        "inner_solver": "block_gs",
        "num_bodies": 1,
        "dt": DT_SECONDS,
    }.items():
        _expect_exact(meta[key], expected, f"{location}.meta.{key}")
    config = _expect_exact_keys(
        meta["config"], FBF_CONFIG_KEYS, f"{location}.meta.config"
    )
    _expect_exact(
        config,
        _expected_fbf_config(_history_source_path(run_id, mu)),
        f"{location}.meta.config",
    )
    bench = _expect_exact_keys(
        meta["bench"], HISTORY_BENCH_KEYS, f"{location}.meta.bench"
    )
    _expect_exact(
        bench,
        {
            "scene": "cube-on-incline",
            "scene_params": {
                "mu": mu,
                "theta_deg": 26.56505117707799,
            },
            "warmup_steps": 0,
            "total_steps": STEPS_PER_CELL,
            "git_sha": SOURCE_SHORT_COMMIT,
        },
        f"{location}.meta.bench",
    )

    counts = {
        "converged": 0,
        "nonconverged": 0,
        "initial": 0,
        "outer": 0,
        "natural_at_or_below_true": 0,
        "natural_above_true": 0,
        "outer_cap": 0,
        "max_outer_iterations": 0,
    }
    nonconverged_steps: list[int] = []
    for index, raw_step in enumerate(steps):
        step_location = f"{location}.steps[{index}]"
        _expect_exact_keys(raw_step, HISTORY_STEP_KEYS, step_location)
        for key in HISTORY_STEP_INTEGER_KEYS:
            _integer(raw_step[key], f"{step_location}.{key}")
        for key in HISTORY_STEP_BOOLEAN_KEYS:
            if type(raw_step[key]) is not bool:
                raise ValueError(f"{step_location}.{key}: expected a JSON boolean")
        for key in (
            HISTORY_STEP_KEYS
            - HISTORY_STEP_INTEGER_KEYS
            - HISTORY_STEP_BOOLEAN_KEYS
            - {"outer"}
        ):
            _finite_float(raw_step[key], f"{step_location}.{key}")
        for key in HISTORY_STEP_CPU_TIME_KEYS:
            if raw_step[key] < 0.0:
                raise ValueError(f"{step_location}.{key}: expected a nonnegative time")
        for key in HISTORY_STEP_GPU_TIME_KEYS:
            _expect_exact(raw_step[key], -1.0, f"{step_location}.{key}")
        if raw_step["warmup"] is not False:
            raise ValueError(f"{step_location}.warmup: expected false")
        if raw_step["step_idx"] != index:
            raise ValueError(f"{step_location}.step_idx: expected {index}")
        if raw_step["num_contacts"] != CONTACTS_PER_STEP:
            raise ValueError(f"{step_location}.num_contacts: expected 4")
        if raw_step["warm_started"] != (raw_step["warm_start_matched"] > 0):
            raise ValueError(f"{step_location}: warm-start flag/count mismatch")
        if not (
            0
            <= raw_step["warm_start_matched"]
            <= raw_step["warm_start_total"]
            == raw_step["num_contacts"]
        ):
            raise ValueError(f"{step_location}: warm-start contact counts changed")
        converged = raw_step["converged"]
        outer_iters = raw_step["outer_iters"]
        if not 0 <= outer_iters <= 200:
            raise ValueError(f"{step_location}.outer_iters: expected 0..200")
        outer = raw_step["outer"]
        if not isinstance(outer, list) or len(outer) != outer_iters:
            raise ValueError(
                f"{step_location}.outer: expected exactly {outer_iters} records"
            )
        for outer_index, record in enumerate(outer):
            outer_location = f"{step_location}.outer[{outer_index}]"
            _expect_exact_keys(record, HISTORY_OUTER_KEYS, outer_location)
            for key in HISTORY_OUTER_INTEGER_KEYS:
                _integer(record[key], f"{outer_location}.{key}")
            for key in (
                HISTORY_OUTER_KEYS - HISTORY_OUTER_INTEGER_KEYS - {"gamma_event"}
            ):
                _finite_float(record[key], f"{outer_location}.{key}")
            if not isinstance(record["gamma_event"], str):
                raise ValueError(f"{outer_location}.gamma_event: expected a string")
            if record["gamma_event"] not in {"", "carry", "restore", "shrink"}:
                raise ValueError(f"{outer_location}.gamma_event: unexpected value")
            if record["k"] != outer_index:
                raise ValueError(f"{outer_location}.k: expected {outer_index}")
            if not 0 <= record["backtracks"] <= 8:
                raise ValueError(f"{outer_location}.backtracks: expected 0..8")
            if not 1 <= record["inner_iters"] <= 10:
                raise ValueError(f"{outer_location}.inner_iters: expected 1..10")
            for key in HISTORY_OUTER_CPU_TIME_KEYS:
                if record[key] < 0.0:
                    raise ValueError(
                        f"{outer_location}.{key}: expected a nonnegative time"
                    )
            for key in HISTORY_OUTER_GPU_TIME_KEYS:
                _expect_exact(record[key], -1.0, f"{outer_location}.{key}")
            residual_checked = (outer_index + 1) % 5 == 0 or outer_index == 199
            if residual_checked:
                for key in HISTORY_OUTER_RESIDUAL_KEYS:
                    if record[key] < 0.0:
                        raise ValueError(
                            f"{outer_location}.{key}: expected a checked residual"
                        )
                _expect_exact(
                    record["r_kkt"],
                    max(record["r_primal"], record["r_dual"], record["r_gap"]),
                    f"{outer_location}.r_kkt",
                )
                _expect_exact(
                    record["r_coulomb"],
                    max(record["eps_vel"], record["eps_force"], record["eps_gap"]),
                    f"{outer_location}.r_coulomb",
                )
                if (
                    outer_index + 1 < outer_iters
                    and record["r_coulomb"] < TERMINATION_TOLERANCE
                ):
                    raise ValueError(
                        f"{outer_location}.r_coulomb: preterminal configured gate passes"
                    )
            else:
                for key in HISTORY_OUTER_RESIDUAL_KEYS:
                    _expect_exact(record[key], -1.0, f"{outer_location}.{key}")
                _expect_exact(
                    record["t_residual_check_ms"],
                    0.0,
                    f"{outer_location}.t_residual_check_ms",
                )
        if outer_iters == 0:
            _expect_exact(
                raw_step["final_residual"],
                raw_step["initial_residual"],
                f"{step_location}.final_residual",
            )
            _expect_exact(
                raw_step["gamma_final"],
                raw_step["gamma_init"],
                f"{step_location}.gamma_final",
            )
        else:
            _expect_exact(
                raw_step["final_residual"],
                outer[-1]["residual"],
                f"{step_location}.final_residual",
            )
            _expect_exact(
                raw_step["gamma_final"],
                outer[-1]["gamma"],
                f"{step_location}.gamma_final",
            )
            if (outer_iters % 5 != 0) and outer_iters != 200:
                raise ValueError(
                    f"{step_location}.outer: terminal residual is unchecked"
                )
        counts["max_outer_iterations"] = max(
            counts["max_outer_iterations"], outer_iters
        )
        natural = raw_step["final_residual"]
        initial = raw_step["initial_residual"]
        if natural < 0.0 or initial < 0.0:
            raise ValueError(f"{step_location}: natural residuals must be nonnegative")
        if outer_iters == 0 and initial >= TERMINATION_TOLERANCE:
            raise ValueError(
                f"{step_location}: invalid initial natural-residual shortcut"
            )
        if outer_iters > 0 and initial < TERMINATION_TOLERANCE:
            raise ValueError(
                f"{step_location}: outer solve has a passing initial residual"
            )
        terminal = _terminal_coulomb_residual(raw_step, step_location)
        if converged:
            counts["converged"] += 1
            if natural <= TERMINATION_TOLERANCE:
                counts["natural_at_or_below_true"] += 1
            else:
                counts["natural_above_true"] += 1
            if outer_iters == 0:
                counts["initial"] += 1
                if terminal is not None:
                    raise ValueError(
                        f"{step_location}: zero-outer accept unexpectedly has configured residual"
                    )
            else:
                counts["outer"] += 1
                if terminal is None or not terminal < TERMINATION_TOLERANCE:
                    raise ValueError(
                        f"{step_location}: configured outer accept lacks passing coulomb_rel"
                    )
        else:
            counts["nonconverged"] += 1
            nonconverged_steps.append(index)
            if outer_iters == 200:
                counts["outer_cap"] += 1
            if terminal is None or terminal < TERMINATION_TOLERANCE:
                raise ValueError(
                    f"{step_location}: configured failure lacks failing coulomb_rel"
                )
    return {
        "mu": mu,
        "converged_flags": counts["converged"],
        "nonconverged_flags": counts["nonconverged"],
        "initial_natural_shortcut_accepts": counts["initial"],
        "configured_outer_gate_accepts": counts["outer"],
        "true_flags_natural_residual_at_or_below_tolerance": counts[
            "natural_at_or_below_true"
        ],
        "true_flags_natural_residual_above_tolerance": counts["natural_above_true"],
        "outer_cap_hits": counts["outer_cap"],
        "max_outer_iterations": counts["max_outer_iterations"],
        "nonconverged_steps": nonconverged_steps,
    }


def _load_source_runs(source_repo: Path) -> dict[str, Any]:
    runs: dict[str, Any] = {}
    for spec in RUN_SPECS:
        solver = str(spec["solver"])
        run_id = str(spec["run_id"])
        root = _source_run_root(source_repo, run_id)
        actual = _regular_members(root)
        expected = _expected_source_members(solver)
        if actual != expected:
            raise ValueError(
                f"{run_id}: source membership changed; missing={sorted(expected - actual)}, extra={sorted(actual - expected)}"
            )
        metadata = _load_json(root / "metadata.json")
        _validate_metadata(metadata, solver, run_id)
        sweep = _load_json(root / "sweep_results.json")
        if not isinstance(sweep, list) or len(sweep) != len(MU_VALUES):
            raise ValueError(f"{run_id}.sweep_results: expected seven rows")
        results: list[dict[str, Any]] = []
        histories: list[dict[str, Any]] = []
        for index, mu in enumerate(MU_VALUES):
            directory = root / f"mu{mu:.4f}_{solver}"
            result = _load_json(directory / "result.json")
            _validate_result(result, solver, mu, run_id, f"{run_id}.{solver}.{mu}")
            if not _exact_json_equal(sweep[index], result):
                raise ValueError(
                    f"{run_id}.sweep_results[{index}]: differs from result.json"
                )
            results.append(result)
            if solver == "fbf":
                history = _load_json(directory / "history.json")
                histories.append(history)
        runs[solver] = {
            "spec": dict(spec),
            "metadata": metadata,
            "sweep": sweep,
            "results": results,
            "histories": histories,
        }
    return runs


def _metrics(runs: dict[str, Any]) -> dict[str, Any]:
    fbf_run_id = str(runs["fbf"]["spec"]["run_id"])
    fbf_results = runs["fbf"]["results"]
    fbf_histories = runs["fbf"]["histories"]
    if len(fbf_results) != len(MU_VALUES) or len(fbf_histories) != len(MU_VALUES):
        raise ValueError("FBF result/history cell membership changed")
    per_mu = [
        _validate_history(history, mu, fbf_run_id, f"fbf.history.mu={mu}")
        for mu, history in zip(MU_VALUES, fbf_histories, strict=True)
    ]
    for mu, result, history in zip(MU_VALUES, fbf_results, fbf_histories, strict=True):
        history_outer_iters = sum(step["outer_iters"] for step in history["steps"])
        if result["total_outer_iters"] != history_outer_iters:
            raise ValueError(
                f"fbf.result.mu={mu}.total_outer_iters: differs from retained history"
            )
        if not _exact_json_equal(
            result["final_residual"], history["steps"][-1]["final_residual"]
        ):
            raise ValueError(
                f"fbf.result.mu={mu}.final_residual: differs from retained history"
            )
    for record in per_mu:
        expected = EXPECTED_FBF_PER_MU[record["mu"]]
        actual = (
            record["converged_flags"],
            record["nonconverged_flags"],
            record["initial_natural_shortcut_accepts"],
            record["configured_outer_gate_accepts"],
            record["true_flags_natural_residual_at_or_below_tolerance"],
            record["true_flags_natural_residual_above_tolerance"],
            record["max_outer_iterations"],
        )
        if actual != expected:
            raise ValueError(
                f"fbf.history.mu={record['mu']}: expected counts {expected}, got {actual}"
            )
    aggregate = {
        "history_steps": len(MU_VALUES) * STEPS_PER_CELL,
        "contact_records": len(MU_VALUES) * STEPS_PER_CELL * CONTACTS_PER_STEP,
        "converged_flags": sum(item["converged_flags"] for item in per_mu),
        "nonconverged_flags": sum(item["nonconverged_flags"] for item in per_mu),
        "initial_natural_shortcut_accepts": sum(
            item["initial_natural_shortcut_accepts"] for item in per_mu
        ),
        "configured_outer_gate_accepts": sum(
            item["configured_outer_gate_accepts"] for item in per_mu
        ),
        "true_flags_natural_residual_at_or_below_tolerance": sum(
            item["true_flags_natural_residual_at_or_below_tolerance"] for item in per_mu
        ),
        "true_flags_natural_residual_above_tolerance": sum(
            item["true_flags_natural_residual_above_tolerance"] for item in per_mu
        ),
        "outer_cap_hits": sum(item["outer_cap_hits"] for item in per_mu),
    }
    if aggregate != EXPECTED_FBF_COUNTS:
        raise ValueError(
            f"FBF aggregate counts changed: expected {EXPECTED_FBF_COUNTS}, got {aggregate}"
        )
    if per_mu[4]["nonconverged_steps"] != [1]:
        raise ValueError("mu=.55 step 1 must be the sole configured nonconvergence")
    for index, record in enumerate(per_mu):
        if index != 4 and record["nonconverged_steps"]:
            raise ValueError("unexpected configured nonconvergence outside mu=.55")

    displacements = {
        solver: [float(result["tangential_disp"]) for result in runs[solver]["results"]]
        for solver in ("fbf", "mujoco", "kamino")
    }
    fbf_kamino_deltas = [
        abs(a - b) for a, b in zip(displacements["fbf"], displacements["kamino"])
    ]
    fbf_mujoco_deltas = [
        abs(a - b) for a, b in zip(displacements["fbf"], displacements["mujoco"])
    ]
    fk_index = max(range(len(MU_VALUES)), key=fbf_kamino_deltas.__getitem__)
    fm_index = max(range(len(MU_VALUES)), key=fbf_mujoco_deltas.__getitem__)
    mujoco_monotone = all(
        following <= previous
        for previous, following in zip(
            displacements["mujoco"], displacements["mujoco"][1:]
        )
    )
    if not max(fbf_kamino_deltas) <= 1.0e-6:
        raise ValueError("FBF/Kamino displacement diagnostic changed")
    if mujoco_monotone:
        raise ValueError("MuJoCo displacement diagnostic unexpectedly became monotone")
    return {
        "cell_count": 21,
        "mu_values": list(MU_VALUES),
        "steps_per_cell": STEPS_PER_CELL,
        "fbf_contacts_per_step": CONTACTS_PER_STEP,
        "fbf": {**aggregate, "per_mu": per_mu},
        "displacements_m": displacements,
        "fbf_kamino_max_absolute_delta_m": max(fbf_kamino_deltas),
        "fbf_kamino_max_absolute_delta_mu": MU_VALUES[fk_index],
        "fbf_mujoco_max_absolute_delta_m": max(fbf_mujoco_deltas),
        "fbf_mujoco_max_absolute_delta_mu": MU_VALUES[fm_index],
        "mujoco_displacement_monotone_nonincreasing": mujoco_monotone,
    }


def _comparison_csv(runs: dict[str, Any], metrics: dict[str, Any]) -> bytes:
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(
        (
            "solver",
            "mu",
            "tangential_displacement_m",
            "steps",
            "fbf_contacts_per_step",
            "configured_converged_flags",
            "configured_nonconverged_flags",
            "timing_evidence_eligible",
        )
    )
    per_mu = {item["mu"]: item for item in metrics["fbf"]["per_mu"]}
    for solver in ("fbf", "mujoco", "kamino"):
        for mu, result in zip(MU_VALUES, runs[solver]["results"]):
            convergence = per_mu[mu] if solver == "fbf" else None
            writer.writerow(
                (
                    solver,
                    format(mu, ".17g"),
                    format(float(result["tangential_disp"]), ".17g"),
                    STEPS_PER_CELL,
                    CONTACTS_PER_STEP if solver == "fbf" else "",
                    convergence["converged_flags"] if convergence else "",
                    convergence["nonconverged_flags"] if convergence else "",
                    "false",
                )
            )
    return output.getvalue().encode("utf-8")


def _comparison_svg(metrics: dict[str, Any]) -> bytes:
    width, height = 1000, 620
    left, right, top, bottom = 105.0, 955.0, 80.0, 520.0
    x_min, x_max, y_max = 0.25, 0.85, 4.0

    def x(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * (right - left)

    def y(value: float) -> float:
        return bottom - value / y_max * (bottom - top)

    def points(values: Sequence[float]) -> str:
        return " ".join(
            f"{x(mu):.3f},{y(value):.3f}" for mu, value in zip(MU_VALUES, values)
        )

    analytic = [max(0.0, 2.0 * 9.81 / math.sqrt(1.25) * (0.5 - mu)) for mu in MU_VALUES]
    colors = {"fbf": "#c94747", "mujoco": "#3465a4", "kamino": "#3f8f55"}
    labels = {"fbf": "FBF", "mujoco": "MuJoCo", "kamino": "Kamino"}
    marker = {"fbf": "circle", "mujoco": "square", "kamino": "triangle"}
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fbfaf6"/>',
        '<text x="500" y="34" text-anchor="middle" font-family="sans-serif" font-size="22" font-weight="700">Pinned current-author incline sweep</text>',
        '<text x="500" y="58" text-anchor="middle" font-family="sans-serif" font-size="14" fill="#555">numeric diagnostic — not historical paper, DART, timing, or cross-solver parity</text>',
    ]
    for tick in range(5):
        value = float(tick)
        yy = y(value)
        lines.append(
            f'<line x1="{left}" y1="{yy:.3f}" x2="{right}" y2="{yy:.3f}" stroke="#ddd" stroke-width="1"/>'
        )
        lines.append(
            f'<text x="{left - 14}" y="{yy + 5:.3f}" text-anchor="end" font-family="sans-serif" font-size="13">{tick}</text>'
        )
    for mu in MU_VALUES:
        xx = x(mu)
        lines.append(
            f'<line x1="{xx:.3f}" y1="{top}" x2="{xx:.3f}" y2="{bottom}" stroke="#eee" stroke-width="1"/>'
        )
        lines.append(
            f'<text x="{xx:.3f}" y="{bottom + 25}" text-anchor="middle" font-family="sans-serif" font-size="13">{mu:g}</text>'
        )
    lines.extend(
        (
            f'<line x1="{left}" y1="{bottom}" x2="{right}" y2="{bottom}" stroke="#222" stroke-width="2"/>',
            f'<line x1="{left}" y1="{top}" x2="{left}" y2="{bottom}" stroke="#222" stroke-width="2"/>',
            f'<text x="{(left + right) / 2:.3f}" y="575" text-anchor="middle" font-family="sans-serif" font-size="16">friction coefficient μ</text>',
            f'<text x="30" y="{(top + bottom) / 2:.3f}" transform="rotate(-90 30 {(top + bottom) / 2:.3f})" text-anchor="middle" font-family="sans-serif" font-size="16">downslope displacement (m)</text>',
            f'<polyline fill="none" stroke="#444" stroke-width="2" stroke-dasharray="8 6" points="{points(analytic)}"/>',
        )
    )
    # Draw FBF last so its near-identical points remain visible over Kamino.
    for solver in ("mujoco", "kamino", "fbf"):
        values = metrics["displacements_m"][solver]
        color = colors[solver]
        lines.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2.5" points="{points(values)}"/>'
        )
        for mu, value in zip(MU_VALUES, values):
            xx, yy = x(mu), y(value)
            if marker[solver] == "circle":
                lines.append(
                    f'<circle cx="{xx:.3f}" cy="{yy:.3f}" r="6" fill="none" stroke="{color}" stroke-width="2.5"/>'
                )
            elif marker[solver] == "square":
                lines.append(
                    f'<rect x="{xx - 5:.3f}" y="{yy - 5:.3f}" width="10" height="10" fill="{color}" stroke="#fff" stroke-width="1.5"/>'
                )
            else:
                lines.append(
                    f'<path d="M {xx:.3f} {yy - 6:.3f} L {xx - 6:.3f} {yy + 5:.3f} L {xx + 6:.3f} {yy + 5:.3f} Z" fill="{color}" stroke="#fff" stroke-width="1.5"/>'
                )
    legend_x, legend_y = 690.0, 105.0
    lines.append(
        f'<rect x="{legend_x}" y="{legend_y}" width="245" height="112" rx="6" fill="#fff" stroke="#bbb"/>'
    )
    lines.append(
        f'<line x1="{legend_x + 16}" y1="{legend_y + 24}" x2="{legend_x + 56}" y2="{legend_y + 24}" stroke="#444" stroke-width="2" stroke-dasharray="8 6"/>'
    )
    lines.append(
        f'<text x="{legend_x + 68}" y="{legend_y + 29}" font-family="sans-serif" font-size="13">analytic threshold</text>'
    )
    for row, solver in enumerate(("fbf", "mujoco", "kamino"), start=1):
        yy = legend_y + 24 + row * 24
        lines.append(
            f'<line x1="{legend_x + 16}" y1="{yy}" x2="{legend_x + 56}" y2="{yy}" stroke="{colors[solver]}" stroke-width="3"/>'
        )
        lines.append(
            f'<text x="{legend_x + 68}" y="{yy + 5}" font-family="sans-serif" font-size="13">{labels[solver]}</text>'
        )
    lines.append("</svg>")
    return ("\n".join(lines) + "\n").encode("utf-8")


def _report(metrics: dict[str, Any]) -> bytes:
    per_mu = {item["mu"]: item for item in metrics["fbf"]["per_mu"]}
    lines = [
        "# Pinned current-author incline sweep reference",
        "",
        f"Status: `{STATUS}`.",
        "",
        CLAIM_SCOPE,
        "",
        "## Recorded grid",
        "",
        "| mu | FBF displacement (m) | MuJoCo displacement (m) | Kamino displacement (m) | FBF configured flags | FBF initial / outer accepts | Natural <= 1e-6 / > 1e-6 among true flags |",
        "|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for index, mu in enumerate(MU_VALUES):
        item = per_mu[mu]
        lines.append(
            "| "
            + " | ".join(
                (
                    f"{mu:g}",
                    f"{metrics['displacements_m']['fbf'][index]:.17g}",
                    f"{metrics['displacements_m']['mujoco'][index]:.17g}",
                    f"{metrics['displacements_m']['kamino'][index]:.17g}",
                    f"{item['converged_flags']}/{STEPS_PER_CELL}",
                    f"{item['initial_natural_shortcut_accepts']} / {item['configured_outer_gate_accepts']}",
                    f"{item['true_flags_natural_residual_at_or_below_tolerance']} / {item['true_flags_natural_residual_above_tolerance']}",
                )
            )
            + " |"
        )
    lines.extend(
        (
            "",
            "Every solver cell records 120 steps. The retained FBF histories record four contacts per FBF step; the MuJoCo and Kamino result records contain no contact-count field. FBF has 839/840 true configured convergence flags. The sole false flag is `mu=.55`, step 1, at the 200-outer cap. Its natural `final_residual` is `3.273267262002487e-8`, while the configured terminal `r_coulomb` is `1.5311460572898186e-6`.",
            "",
            "Of the 839 true FBF flags, 235 use the initial natural-residual shortcut and 604 use the configured nonnegative `coulomb_rel < 1e-6` outer gate. Only 456 true flags also have natural `final_residual <= 1e-6`; 383 true flags have a larger natural value. The two residual fields are not interchangeable.",
            "",
            "FBF and Kamino displacement values differ by at most `9.86272277444535e-7 m` in this packet. MuJoCo displacement is nonmonotone over the recorded grid. These are qualitative numeric observations, not full-state or cross-solver parity.",
            "",
            "## Timing and parity boundary",
            "",
            CLAIM_BOUNDARY,
            "",
            "No manual image inspection is recorded. `comparison.svg` is a deterministic rendering of `comparison.csv`; the numeric CSV is authoritative.",
            "",
            "## Verification",
            "",
            "```bash",
            "python3 scripts/finalize_fbf_author_incline_reference.py --verify-only",
            "```",
            "",
        )
    )
    return ("\n".join(lines)).encode("utf-8")


def _gzip_bytes(data: bytes) -> bytes:
    output = io.BytesIO()
    with gzip.GzipFile(
        filename="", mode="wb", fileobj=output, compresslevel=9, mtime=0
    ) as stream:
        stream.write(data)
    return output.getvalue()


def _copy_source_artifacts(source_repo: Path, staging: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for spec in RUN_SPECS:
        solver = str(spec["solver"])
        run_id = str(spec["run_id"])
        source_root = _source_run_root(source_repo, run_id)
        for name in ("metadata.json", "sweep_results.json"):
            source = source_root / name
            relative = f"runs/{run_id}/{name}"
            data = source.read_bytes()
            _write_bytes(staging / relative, data)
            records.append(
                {
                    "bundle_path": relative,
                    "source_path": str(source.relative_to(source_repo)),
                    "source_sha256": _sha256_bytes(data),
                    "source_bytes": len(data),
                    "stored_sha256": _sha256_bytes(data),
                    "stored_bytes": len(data),
                    "compression": None,
                }
            )
        for mu in MU_VALUES:
            source_dir = source_root / f"mu{mu:.4f}_{solver}"
            source = source_dir / "result.json"
            relative = _result_relative(run_id, solver, mu)
            data = source.read_bytes()
            _write_bytes(staging / relative, data)
            records.append(
                {
                    "bundle_path": relative,
                    "source_path": str(source.relative_to(source_repo)),
                    "source_sha256": _sha256_bytes(data),
                    "source_bytes": len(data),
                    "stored_sha256": _sha256_bytes(data),
                    "stored_bytes": len(data),
                    "compression": None,
                }
            )
            if solver == "fbf":
                history = source_dir / "history.json"
                raw = history.read_bytes()
                compressed = _gzip_bytes(raw)
                relative = _history_relative(run_id, mu)
                _write_bytes(staging / relative, compressed)
                records.append(
                    {
                        "bundle_path": relative,
                        "source_path": str(history.relative_to(source_repo)),
                        "source_sha256": _sha256_bytes(raw),
                        "source_bytes": len(raw),
                        "stored_sha256": _sha256_bytes(compressed),
                        "stored_bytes": len(compressed),
                        "compression": "gzip-mtime0-level9",
                    }
                )
    records.sort(key=lambda item: item["bundle_path"])
    if [item["bundle_path"] for item in records] != sorted(SOURCE_ARTIFACT_PATHS):
        raise ValueError("internal source-artifact membership mismatch")
    return records


def _artifact_record(root: Path, relative: str) -> dict[str, Any]:
    path = root / _safe_relative(relative, "artifact path")
    return {
        "path": relative,
        "sha256": _sha256(path),
        "bytes": path.stat().st_size,
    }


def _manifest(
    source_identity: dict[str, Any],
    source_artifacts: list[dict[str, Any]],
    metrics: dict[str, Any],
    staging: Path,
) -> dict[str, Any]:
    run_records = [
        {
            "solver": spec["solver"],
            "run_id": spec["run_id"],
            "device": "cpu",
            "argv": _argv(str(spec["solver"]), bool(spec["profile"])),
            "argv_provenance": "operator_recorded_not_embedded_by_source_runner",
            "cwd": str(DEFAULT_SOURCE_REPO),
            "profile_flag": bool(spec["profile"]),
            "timing_evidence_eligible": False,
        }
        for spec in RUN_SPECS
    ]
    derived = [
        _artifact_record(staging, relative) for relative in DERIVED_ARTIFACT_PATHS
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "status": STATUS,
        "source": source_identity,
        "runs": run_records,
        "workload": {
            "scene": "cube-on-incline",
            "mu_values": list(MU_VALUES),
            "dt_seconds": DT_SECONDS,
            "steps_per_cell": STEPS_PER_CELL,
            "fbf_contacts_per_step": CONTACTS_PER_STEP,
            "device": "cpu",
            "independent_solver_runs": True,
        },
        "metrics": metrics,
        "predicates": {
            "artifact_integrity_valid": True,
            "qualitative_source_grid_outcome_valid": True,
            "fbf_configured_solver_valid": False,
            "fbf_kamino_displacement_close_all_cells": True,
            "mujoco_displacement_monotone_nonincreasing": False,
            "current_source_scientific_negative": True,
            "historical_or_paper_invocation_valid": False,
            "full_published_sweep_valid": False,
            "source_equivalent_dart_dynamics_executed": False,
            "dart_dynamics_parity_valid": False,
            "cross_solver_full_state_parity_valid": False,
            "fig01_parity": False,
            "fig02_parity": False,
            "video03_incline_parity": False,
            "renderer_or_media_evidence_valid": False,
            "approved_source_golden_valid": False,
            "timing_verdict": None,
            "paper_timing_valid": False,
            "realtime_verdict": None,
            "paper_comparable": False,
            "manual_visual_inspection_valid": False,
        },
        "timing_boundary": {
            "eligible_for_verdict": False,
            "reason": "First-use/JIT work, always-on history instrumentation, ineffective warmup exclusion, and scene-dependent source timer boundaries are uncontrolled.",
        },
        "retained_source_artifacts": source_artifacts,
        "derived_artifacts": derived,
        "exact_bundle_members": list(EXACT_BUNDLE_MEMBERS),
        "claim_scope": CLAIM_SCOPE,
        "claim_boundary": CLAIM_BOUNDARY,
    }


def _load_bundle_runs(bundle: Path) -> dict[str, Any]:
    runs: dict[str, Any] = {}
    for spec in RUN_SPECS:
        solver = str(spec["solver"])
        run_id = str(spec["run_id"])
        metadata = _load_json(bundle / f"runs/{run_id}/metadata.json")
        _validate_metadata(metadata, solver, run_id)
        sweep = _load_json(bundle / f"runs/{run_id}/sweep_results.json")
        if not isinstance(sweep, list) or len(sweep) != len(MU_VALUES):
            raise ValueError(f"{run_id}.sweep_results: expected seven rows")
        results: list[dict[str, Any]] = []
        histories: list[dict[str, Any]] = []
        for index, mu in enumerate(MU_VALUES):
            result = _load_json(bundle / _result_relative(run_id, solver, mu))
            _validate_result(result, solver, mu, run_id, f"{run_id}.{solver}.{mu}")
            if not _exact_json_equal(sweep[index], result):
                raise ValueError(
                    f"{run_id}.sweep_results[{index}]: differs from result.json"
                )
            results.append(result)
            if solver == "fbf":
                relative = _history_relative(run_id, mu)
                try:
                    with gzip.open(bundle / relative, "rt", encoding="utf-8") as stream:
                        histories.append(
                            json.load(
                                stream,
                                object_pairs_hook=_unique_object,
                                parse_constant=_json_constant,
                            )
                        )
                except (OSError, json.JSONDecodeError) as error:
                    raise ValueError(
                        f"{relative}: invalid deterministic history: {error}"
                    ) from error
        runs[solver] = {
            "spec": dict(spec),
            "metadata": metadata,
            "sweep": sweep,
            "results": results,
            "histories": histories,
        }
    return runs


def validate_bundle(bundle: Path) -> dict[str, Any]:
    """Validate exact membership, provenance bindings, and numeric semantics."""
    bundle = _absolute_without_symlink_components(bundle, "bundle")
    actual_members = _regular_members(bundle)
    expected_members = set(EXACT_BUNDLE_MEMBERS)
    if actual_members != expected_members:
        raise ValueError(
            f"bundle membership changed; missing={sorted(expected_members - actual_members)}, extra={sorted(actual_members - expected_members)}"
        )
    manifest = _expect_exact_keys(
        _load_json(bundle / "manifest.json"),
        {
            "schema_version",
            "status",
            "source",
            "runs",
            "workload",
            "metrics",
            "predicates",
            "timing_boundary",
            "retained_source_artifacts",
            "derived_artifacts",
            "exact_bundle_members",
            "claim_scope",
            "claim_boundary",
        },
        "manifest",
    )
    for key, expected in {
        "schema_version": SCHEMA_VERSION,
        "status": STATUS,
        "claim_scope": CLAIM_SCOPE,
        "claim_boundary": CLAIM_BOUNDARY,
        "exact_bundle_members": list(EXACT_BUNDLE_MEMBERS),
    }.items():
        _expect_exact(manifest[key], expected, f"manifest.{key}")
    expected_source = _expected_source_identity()
    source = _expect_exact_keys(
        manifest["source"], set(expected_source), "manifest.source"
    )
    _expect_exact(source, expected_source, "manifest.source")
    expected_runs = [
        {
            "solver": spec["solver"],
            "run_id": spec["run_id"],
            "device": "cpu",
            "argv": _argv(str(spec["solver"]), bool(spec["profile"])),
            "argv_provenance": "operator_recorded_not_embedded_by_source_runner",
            "cwd": str(DEFAULT_SOURCE_REPO),
            "profile_flag": bool(spec["profile"]),
            "timing_evidence_eligible": False,
        }
        for spec in RUN_SPECS
    ]
    _expect_exact(manifest["runs"], expected_runs, "manifest.runs")
    expected_workload = {
        "scene": "cube-on-incline",
        "mu_values": list(MU_VALUES),
        "dt_seconds": DT_SECONDS,
        "steps_per_cell": STEPS_PER_CELL,
        "fbf_contacts_per_step": CONTACTS_PER_STEP,
        "device": "cpu",
        "independent_solver_runs": True,
    }
    _expect_exact(manifest["workload"], expected_workload, "manifest.workload")

    source_records = manifest["retained_source_artifacts"]
    if not isinstance(source_records, list):
        raise ValueError("manifest.retained_source_artifacts: expected list")
    source_by_path: dict[str, dict[str, Any]] = {}
    ordered_source_paths: list[str] = []
    for index, raw_record in enumerate(source_records):
        record = _expect_exact_keys(
            raw_record,
            {
                "bundle_path",
                "source_path",
                "source_sha256",
                "source_bytes",
                "stored_sha256",
                "stored_bytes",
                "compression",
            },
            f"manifest.retained_source_artifacts[{index}]",
        )
        relative = record["bundle_path"]
        if not isinstance(relative, str):
            raise ValueError("source artifact bundle path must be a string")
        _safe_relative(relative, "source artifact bundle path")
        source_relative = record["source_path"]
        if not isinstance(source_relative, str):
            raise ValueError("source artifact source path must be a string")
        _safe_relative(source_relative, "source artifact source path")
        expected_source_relative = _expected_source_path(relative)
        if source_relative != expected_source_relative:
            raise ValueError(
                f"{relative}: source path differs; expected {expected_source_relative!r}"
            )
        if relative in source_by_path:
            raise ValueError(f"duplicate source artifact {relative!r}")
        source_by_path[relative] = record
        ordered_source_paths.append(relative)
    if set(source_by_path) != set(SOURCE_ARTIFACT_PATHS):
        raise ValueError("manifest source artifact membership changed")
    if ordered_source_paths != sorted(SOURCE_ARTIFACT_PATHS):
        raise ValueError("manifest source artifact order changed")
    for relative, record in source_by_path.items():
        path = bundle / relative
        location = f"manifest.retained_source_artifacts[{relative}]"
        _expect_exact(
            record["stored_sha256"], _sha256(path), f"{location}.stored_sha256"
        )
        _expect_exact(
            record["stored_bytes"], path.stat().st_size, f"{location}.stored_bytes"
        )
        if relative.endswith(".json.gz"):
            _expect_exact(
                record["compression"],
                "gzip-mtime0-level9",
                f"{location}.compression",
            )
            with gzip.open(path, "rb") as stream:
                raw = stream.read()
            _expect_exact(
                record["source_sha256"],
                _sha256_bytes(raw),
                f"{location}.source_sha256",
            )
            _expect_exact(record["source_bytes"], len(raw), f"{location}.source_bytes")
            if path.read_bytes() != _gzip_bytes(raw):
                raise ValueError(f"{relative}: gzip stream is not deterministic")
        else:
            _expect_exact(record["compression"], None, f"{location}.compression")
            _expect_exact(
                record["source_sha256"],
                record["stored_sha256"],
                f"{location}.source_sha256",
            )
            _expect_exact(
                record["source_bytes"],
                record["stored_bytes"],
                f"{location}.source_bytes",
            )

    runs = _load_bundle_runs(bundle)
    metrics = _metrics(runs)
    _expect_exact(manifest["metrics"], metrics, "manifest.metrics")
    expected_predicates = {
        "artifact_integrity_valid": True,
        "qualitative_source_grid_outcome_valid": True,
        "fbf_configured_solver_valid": False,
        "fbf_kamino_displacement_close_all_cells": True,
        "mujoco_displacement_monotone_nonincreasing": False,
        "current_source_scientific_negative": True,
        "historical_or_paper_invocation_valid": False,
        "full_published_sweep_valid": False,
        "source_equivalent_dart_dynamics_executed": False,
        "dart_dynamics_parity_valid": False,
        "cross_solver_full_state_parity_valid": False,
        "fig01_parity": False,
        "fig02_parity": False,
        "video03_incline_parity": False,
        "renderer_or_media_evidence_valid": False,
        "approved_source_golden_valid": False,
        "timing_verdict": None,
        "paper_timing_valid": False,
        "realtime_verdict": None,
        "paper_comparable": False,
        "manual_visual_inspection_valid": False,
    }
    _expect_exact(manifest["predicates"], expected_predicates, "manifest.predicates")
    expected_timing_boundary = {
        "eligible_for_verdict": False,
        "reason": "First-use/JIT work, always-on history instrumentation, ineffective warmup exclusion, and scene-dependent source timer boundaries are uncontrolled.",
    }
    _expect_exact(
        manifest["timing_boundary"],
        expected_timing_boundary,
        "manifest.timing_boundary",
    )

    expected_derived_bytes = {
        "comparison.csv": _comparison_csv(runs, metrics),
        "comparison.svg": _comparison_svg(metrics),
        "REPORT.md": _report(metrics),
    }
    derived = manifest["derived_artifacts"]
    if not isinstance(derived, list) or len(derived) != len(DERIVED_ARTIFACT_PATHS):
        raise ValueError("manifest.derived_artifacts: exact list length changed")
    derived_by_path: dict[str, dict[str, Any]] = {}
    for index, raw_record in enumerate(derived):
        record = _expect_exact_keys(
            raw_record,
            {"path", "sha256", "bytes"},
            f"manifest.derived_artifacts[{index}]",
        )
        relative = record["path"]
        if not isinstance(relative, str):
            raise ValueError(
                f"manifest.derived_artifacts[{index}].path: expected string"
            )
        _safe_relative(relative, f"manifest.derived_artifacts[{index}].path")
        if relative in derived_by_path:
            raise ValueError(f"duplicate derived artifact {relative!r}")
        derived_by_path[relative] = record
    if list(derived_by_path) != list(DERIVED_ARTIFACT_PATHS):
        raise ValueError("manifest derived artifact membership/order changed")
    for relative, data in expected_derived_bytes.items():
        record = derived_by_path[relative]
        if (bundle / relative).read_bytes() != data:
            raise ValueError(f"{relative}: deterministic derived content changed")
        _expect_exact(
            record,
            {"path": relative, "sha256": _sha256_bytes(data), "bytes": len(data)},
            f"manifest.derived_artifacts[{relative}]",
        )

    verification = _expect_exact_keys(
        _load_json(bundle / "verification.json"),
        {
            "schema_version",
            "status",
            "manifest_sha256",
            "finalizer_sha256",
            "artifact_count",
            "physical_file_count",
            "checks",
        },
        "verification",
    )
    expected_checks = [
        "exact_bundle_membership",
        "no_symlinks_or_nonregular_entries",
        "source_identity_and_operator_recorded_invocations",
        "all_21_result_cells_and_7_fbf_histories",
        "four_contacts_for_all_840_fbf_steps",
        "configured_coulomb_rel_distinct_from_natural_final_residual",
        "sole_mu_0_55_step_1_configured_nonconvergence",
        "deterministic_comparison_csv_svg_report",
        "timing_and_all_parity_promotions_rejected",
    ]
    _expect_exact(
        verification,
        {
            "schema_version": VALIDATION_SCHEMA_VERSION,
            "status": STATUS,
            "manifest_sha256": _sha256(bundle / "manifest.json"),
            "finalizer_sha256": _sha256(Path(__file__)),
            "artifact_count": 37,
            "physical_file_count": 39,
            "checks": expected_checks,
        },
        "verification",
    )
    return {
        "status": STATUS,
        "bundle": str(bundle),
        "artifact_count": 37,
        "physical_file_count": 39,
        "cell_count": metrics["cell_count"],
        "fbf_configured_converged_flags": metrics["fbf"]["converged_flags"],
        "fbf_configured_nonconverged_flags": metrics["fbf"]["nonconverged_flags"],
        "timing_verdict": None,
        "paper_comparable": False,
    }


def _publish_staged_bundle(staging: Path, bundle: Path) -> dict[str, Any]:
    staging = _absolute_without_symlink_components(staging, "staged bundle")
    bundle = _absolute_without_symlink_components(bundle, "bundle destination")
    backup = bundle.parent / f".{bundle.name}.backup-{uuid.uuid4().hex}"
    if backup.exists() or backup.is_symlink():
        raise ValueError("transaction backup collision")
    backed_up = False
    promoted = False
    try:
        if bundle.exists() or bundle.is_symlink():
            if bundle.is_symlink() or not bundle.is_dir():
                raise ValueError(
                    f"existing bundle is not a regular directory: {bundle}"
                )
            os.replace(bundle, backup)
            backed_up = True
        _absolute_without_symlink_components(bundle.parent, "bundle parent")
        os.replace(staging, bundle)
        promoted = True
        report = validate_bundle(bundle)
        if backed_up:
            _remove_path(backup)
            backed_up = False
        return report
    except Exception:
        if promoted or backed_up:
            _remove_path(bundle)
        if backed_up:
            os.replace(backup, bundle)
            backed_up = False
        raise
    finally:
        _remove_path(staging)
        if not backed_up:
            _remove_path(backup)


def finalize_bundle(source_repo: Path, bundle: Path) -> dict[str, Any]:
    """Build the deterministic bundle transactionally and verify it."""
    source_repo = _absolute_without_symlink_components(source_repo, "source repository")
    bundle = _absolute_without_symlink_components(bundle, "bundle destination")
    source_identity = _source_identity(source_repo)
    runs = _load_source_runs(source_repo)
    metrics = _metrics(runs)
    if _source_identity(source_repo) != source_identity:
        raise ValueError("source identity changed while reading run artifacts")

    bundle.parent.mkdir(parents=True, exist_ok=True)
    _absolute_without_symlink_components(bundle.parent, "bundle parent")
    staging = bundle.parent / f".{bundle.name}.staging-{uuid.uuid4().hex}"
    if staging.exists() or staging.is_symlink():
        raise ValueError("transaction staging collision")
    staging.mkdir()
    try:
        source_artifacts = _copy_source_artifacts(source_repo, staging)
        _write_bytes(staging / "comparison.csv", _comparison_csv(runs, metrics))
        _write_bytes(staging / "comparison.svg", _comparison_svg(metrics))
        _write_bytes(staging / "REPORT.md", _report(metrics))
        manifest = _manifest(source_identity, source_artifacts, metrics, staging)
        _write_json(staging / "manifest.json", manifest)
        _write_json(
            staging / "verification.json",
            {
                "schema_version": VALIDATION_SCHEMA_VERSION,
                "status": STATUS,
                "manifest_sha256": _sha256(staging / "manifest.json"),
                "finalizer_sha256": _sha256(Path(__file__)),
                "artifact_count": 37,
                "physical_file_count": 39,
                "checks": [
                    "exact_bundle_membership",
                    "no_symlinks_or_nonregular_entries",
                    "source_identity_and_operator_recorded_invocations",
                    "all_21_result_cells_and_7_fbf_histories",
                    "four_contacts_for_all_840_fbf_steps",
                    "configured_coulomb_rel_distinct_from_natural_final_residual",
                    "sole_mu_0_55_step_1_configured_nonconvergence",
                    "deterministic_comparison_csv_svg_report",
                    "timing_and_all_parity_promotions_rejected",
                ],
            },
        )
        validate_bundle(staging)
        if _source_identity(source_repo) != source_identity:
            raise ValueError("source identity changed before bundle promotion")
        return _publish_staged_bundle(staging, bundle)
    finally:
        _remove_path(staging)


def main(argv: Sequence[str] | None = None) -> int:
    """Finalize a source run or verify an already sealed bundle."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repo", type=Path, default=DEFAULT_SOURCE_REPO)
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        report = (
            validate_bundle(args.bundle)
            if args.verify_only
            else finalize_bundle(args.source_repo, args.bundle)
        )
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
