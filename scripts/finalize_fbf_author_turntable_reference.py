#!/usr/bin/env python3
"""Finalize and validate the pinned author FBF turntable reference bundle.

The bundle preserves four public-reference trajectories and their complete
solver histories.  It deliberately separates artifact integrity, physical
outcome classification, solver convergence, and provenance completeness.  In
particular, the mu=0.2, omega=2 history contains one expected nonconverged
contact step, so an intact bundle is not an all-cases solver-valid result.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import stat
import subprocess
import sys
import zipfile
import zlib
from pathlib import Path, PurePosixPath
from typing import Any, NamedTuple, Sequence

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BUNDLE = (
    ROOT
    / "docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence"
    / "author_turntable_reference_v1"
)
DEFAULT_SOURCE_REPO = Path("/tmp/fbf-sca-2026-author")

SCHEMA_VERSION = "dart.fbf_author_turntable_reference/v1"
VALIDATION_SCHEMA_VERSION = "dart.fbf_author_turntable_reference_validation/v1"
SOURCE_REPOSITORY = "https://github.com/matthcsong/fbf-sca-2026.git"
SOURCE_COMMIT = "b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0"
SOURCE_TREE = "ffcdafb61adeda2239c8366d054b548b50d26685"
SOURCE_SHORT_COMMIT = "b3f3c5c"
RUNNER_PATH = "paper_examples/turntable/run.py"
RUNNER_SHA256 = "5dd330d2e430585fc3e61eb9c11d2d9aa00b518170f7b64b809c69587cf7db53"
RUNNER_GIT_BLOB = "34980c07ab7723a116d2c65fc1614ecb8f9ff539"
REQUIREMENTS_SHA256 = "436d6a280e62f0cf5b670bf0475cbed6e95fa87e2fb011bc30334355a3ed9688"

DT_SECONDS = 1.0 / 60.0
TOTAL_STEPS = 360
STEADY_WINDOW_STEPS = 60
TURN_TABLE_RADIUS = 2.0
CUBE_HALF_SIZE = 0.15
CUBE_OFFSET_RADIUS = 1.0
TURN_TABLE_HALF_HEIGHT = 0.05
GRAVITY = 9.81
EXIT_RADIUS = TURN_TABLE_RADIUS + CUBE_HALF_SIZE
EXIT_Z = 2.0 * TURN_TABLE_HALF_HEIGHT - 0.5
TERMINATION_TOLERANCE = 1.0e-6
MAX_NPZ_MEMBER_BYTES = 16 * 1024
MAX_NPZ_TOTAL_BYTES = 128 * 1024
GZIP_CHUNK_BYTES = 64 * 1024

TRAJECTORY_KEYS = (
    "t",
    "r",
    "radial_disp",
    "tangential_arc",
    "z",
    "slip_tangential",
    "slip_radial",
    "omega_command",
    "omega_turntable_actual",
)

RESULT_KEYS = {
    "solver",
    "mu",
    "omega",
    "drop_height",
    "omega_over_crit",
    "dt",
    "num_steps",
    "ramp_time",
    "steady_window",
    "v_slip_tan_mean_last_window",
    "v_slip_tan_max_last_window",
    "v_slip_rad_mean_last_window",
    "v_slip_rad_max_last_window",
    "final_tangential_arc",
    "final_radial_disp",
    "avg_step_time_ms",
    "step_times_ms",
    "usd",
    "config",
    "T_seconds",
}

CONFIG_KEYS = {
    "gamma",
    "max_outer",
    "outer_tol",
    "residual_check_interval",
    "inner_max_iter",
    "inner_tol",
    "inner_step_size",
    "inner_solver",
    "inner_gs_sweeps",
    "max_contacts",
    "baumgarte_erp",
    "project_after_correction",
    "warm_start",
    "warm_start_match_radius",
    "warm_start_normal_cos",
    "warm_start_max_age",
    "gamma_min",
    "gamma_max",
    "gamma_c",
    "adaptive_gamma",
    "armijo_rho_high",
    "armijo_shrink",
    "armijo_grow",
    "armijo_skip_threshold",
    "armijo_max_backtracks",
    "plateau_patience",
    "plateau_rtol",
    "warm_start_gamma_threshold",
    "warm_start_gamma_cap",
    "history_path",
    "termination_residual",
    "termination_tol",
    "per_contact_diag_dir",
    "per_contact_diag_topk",
    "dump_step_state_idx",
    "dump_step_state_path",
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

EXPECTED_CONFIG = {
    "gamma": None,
    "max_outer": 200,
    "outer_tol": 1.0e-6,
    "residual_check_interval": 5,
    "inner_max_iter": 200,
    "inner_tol": 1.0e-6,
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
    "gamma_c": 5.0,
    "adaptive_gamma": True,
    "armijo_rho_high": 0.9,
    "armijo_shrink": 0.7,
    "armijo_grow": 1.4285714285714286,
    "armijo_skip_threshold": 1.0e-10,
    "armijo_max_backtracks": 8,
    "plateau_patience": 5,
    "plateau_rtol": 0.01,
    "warm_start_gamma_threshold": 0.0001,
    "warm_start_gamma_cap": 10000.0,
    "termination_residual": "coulomb_rel",
    "termination_tol": TERMINATION_TOLERANCE,
    "per_contact_diag_dir": None,
    "per_contact_diag_topk": 20,
    "dump_step_state_idx": None,
    "dump_step_state_path": None,
}


class CaseSpec(NamedTuple):
    case_id: str
    run_id: str
    directory: str
    mu: float
    omega: float
    expected_outcome: str
    first_exit_index: int | None
    first_tangential_nan_index: int | None
    final_radial_displacement: float
    final_tangential_arc: float | None
    contact_steps: int
    max_outer_iterations: int
    nonconverged_steps: tuple[int, ...]
    max_converged_coulomb_residual: float
    initial_converged_steps: int
    natural_residual_over_tolerance_converged_steps: int

    @property
    def prefix(self) -> str:
        return f"runs/{self.run_id}/{self.directory}"

    @property
    def result_path(self) -> str:
        return f"{self.prefix}/result.json"

    @property
    def trajectory_path(self) -> str:
        return f"{self.prefix}/trajectory.npz"

    @property
    def history_path(self) -> str:
        return f"{self.prefix}/history.json.gz"


CASES = (
    CaseSpec(
        "mu_0_5_omega_2",
        "20260718T214609Z",
        "omega2.00_fbf",
        0.5,
        2.0,
        "retained_through_horizon",
        None,
        None,
        0.21715261973480904,
        0.05770821335911861,
        351,
        40,
        (),
        9.992237902278174e-7,
        17,
        3,
    ),
    CaseSpec(
        "mu_0_5_omega_5",
        "20260718T214609Z",
        "omega5.00_fbf",
        0.5,
        5.0,
        "left_table_boundary",
        95,
        96,
        18.19300319827592,
        None,
        91,
        95,
        (),
        9.834467093070385e-7,
        21,
        26,
    ),
    CaseSpec(
        "mu_0_2_omega_2",
        "20260718T214657Z",
        "omega2.00_fbf",
        0.2,
        2.0,
        "left_table_boundary",
        146,
        147,
        9.154311327087953,
        None,
        146,
        45,
        (130,),
        9.95604688398705e-7,
        23,
        20,
    ),
    CaseSpec(
        "mu_0_2_omega_5",
        "20260718T214657Z",
        "omega5.00_fbf",
        0.2,
        5.0,
        "left_table_boundary",
        128,
        129,
        10.757564838421219,
        None,
        125,
        20,
        (),
        7.284644639693557e-7,
        21,
        61,
    ),
)
CASE_BY_ID = {case.case_id: case for case in CASES}

RUNS = {
    "20260718T214609Z": {
        "mu": 0.5,
        "metadata_path": "runs/20260718T214609Z/metadata.json",
        "source_directory": ("paper_examples/turntable/results/20260718T214609Z"),
        "invocation": [
            "/tmp/fbf-author-venv/bin/python",
            "paper_examples/turntable/run.py",
            "--solvers",
            "fbf",
            "--mu",
            "0.5",
            "--omega",
            "2",
            "5",
            "--time",
            "6",
            "--device",
            "cpu",
            "--drop-height",
            "0.2",
        ],
    },
    "20260718T214657Z": {
        "mu": 0.2,
        "metadata_path": "runs/20260718T214657Z/metadata.json",
        "source_directory": ("paper_examples/turntable/results/20260718T214657Z"),
        "invocation": [
            "/tmp/fbf-author-venv/bin/python",
            "paper_examples/turntable/run.py",
            "--solvers",
            "fbf",
            "--mu",
            "0.2",
            "--omega",
            "2",
            "5",
            "--time",
            "6",
            "--device",
            "cpu",
            "--drop-height",
            "0.2",
        ],
    },
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

NONCLAIMS = (
    "not_all_four_cases_are_solver_contract_valid",
    "not_paper_timing_or_real_time_evidence",
    "not_an_exact_critical_speed_or_bifurcation_measurement",
    "not_a_sticking_or_exact_corotation_classification",
    "post_exit_slip_is_not_contact_sliding_evidence",
    "not_infinite_horizon_stability_or_repeatability_evidence",
    "not_dart_equivalence_or_cross_solver_parity",
    "not_renderer_camera_material_usd_or_approved_golden_evidence",
    "not_a_contemporaneous_pre_and_post_runtime_attestation",
)

EXPECTED_PREDICATES = {
    "artifact_valid": True,
    "physical_outcome_matrix_valid": True,
    "solver_contract_by_case": {
        "mu_0_5_omega_2": True,
        "mu_0_5_omega_5": True,
        "mu_0_2_omega_2": False,
        "mu_0_2_omega_5": True,
    },
    "all_solver_contract_valid": False,
    "literal_invocations_recorded": True,
    "source_commit_consistent": True,
    "runtime_attestation_complete": False,
    "timing_verdict": None,
    "realtime_verdict": None,
    "paper_parity": False,
    "dart_equivalence": False,
    "claim_valid": False,
}

EXPECTED_ARTIFACTS = (
    {
        "path": "runs/20260718T214609Z/metadata.json",
        "bytes": 663,
        "sha256": "873f033619ff1ba9973baf8a1558cb758cb49d2a2d01fb840cb2ceb90612fe0c",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214609Z/omega2.00_fbf/history.json.gz",
        "bytes": 370294,
        "sha256": "141f03741200c20d767db641b65dd737813260b2664fd135b5a4446822a9a87e",
        "encoding": "gzip-n9",
        "raw_bytes": 4218667,
        "raw_sha256": "6443d0a70efcee6df699726db77bbdbc83af4f8bd951c4c82230f14534309157",
    },
    {
        "path": "runs/20260718T214609Z/omega2.00_fbf/result.json",
        "bytes": 10179,
        "sha256": "ddf8f8825b784431c70078a3e6e598af71fcb6e5909a17b9485765f764a93b88",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214609Z/omega2.00_fbf/trajectory.npz",
        "bytes": 28208,
        "sha256": "820f0444b7650504fcf025750ea2c0097b7fb51aa8f25cb8c0e181ec8813be32",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214609Z/omega5.00_fbf/history.json.gz",
        "bytes": 111095,
        "sha256": "17569cef8da4fb9c2c3a10edebb803387fac8f15c7ca4d0411a7c4e7a163eba8",
        "encoding": "gzip-n9",
        "raw_bytes": 1373538,
        "raw_sha256": "ada0da194b8386d187c4861ebec31ab2b799cd39110d20d26cd621bd66ea3b0b",
    },
    {
        "path": "runs/20260718T214609Z/omega5.00_fbf/result.json",
        "bytes": 10301,
        "sha256": "3989e4fda7c63621e25f102cdb692351af76d89b746d35f334cf4333011fe434",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214609Z/omega5.00_fbf/trajectory.npz",
        "bytes": 28208,
        "sha256": "ba1fc63651cf81458343980223f2c337c18338961ed884d0b4a01de351b2e917",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214657Z/metadata.json",
        "bytes": 663,
        "sha256": "2a7e52e1f5ce2557ad2bcd0806a8f6c7ec52e5330ccae414f6b679dc2a1c3706",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214657Z/omega2.00_fbf/history.json.gz",
        "bytes": 113880,
        "sha256": "fe68c5d397bb6097a95e292b673a567fd1abe959ea4c2deb40749f9863f1322b",
        "encoding": "gzip-n9",
        "raw_bytes": 1359991,
        "raw_sha256": "11c88c1b515fc3e4a5a58b0df62edd6ff55f24506e726b57ab05b9eec917729a",
    },
    {
        "path": "runs/20260718T214657Z/omega2.00_fbf/result.json",
        "bytes": 10249,
        "sha256": "e01e36c346c76316832b0737c0e4a7802d7c52532beecf7db7562ed189bb09fb",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214657Z/omega2.00_fbf/trajectory.npz",
        "bytes": 28208,
        "sha256": "59a91e1e15a3c3c3feb6f102b0507a3ab294613ffb06c0c6175f21797dc338a2",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214657Z/omega5.00_fbf/history.json.gz",
        "bytes": 81968,
        "sha256": "b5bd31f689c757961c6288788e8e995da8b494647383f209304cd7c0eab9c7dd",
        "encoding": "gzip-n9",
        "raw_bytes": 1029951,
        "raw_sha256": "9dd569ce3ea23229abd82a1bf063923c99cf9df26c928538b4bf2665cbec1b20",
    },
    {
        "path": "runs/20260718T214657Z/omega5.00_fbf/result.json",
        "bytes": 10264,
        "sha256": "2ad9a336c17e2b1aa9f260408cfa3b614e69c2afbe743e03da1cbdc7dabdc069",
        "encoding": "raw",
    },
    {
        "path": "runs/20260718T214657Z/omega5.00_fbf/trajectory.npz",
        "bytes": 28208,
        "sha256": "041201d16b2784d09d5613bc1dfa4fd254f3f975670c2935a65cbd28a89fb015",
        "encoding": "raw",
    },
)
EXPECTED_ARTIFACT_PATHS = {item["path"] for item in EXPECTED_ARTIFACTS}
EXPECTED_STORED_BYTES = sum(int(item["bytes"]) for item in EXPECTED_ARTIFACTS)


class JsonConstant:
    """Sentinel used to retain the location of nonstandard JSON constants."""

    def __init__(self, token: str):
        self.token = token


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256(path: Path) -> str:
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


def parse_json_bytes(data: bytes, *, label: str, allow_constants: bool = False) -> Any:
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError(f"{label}: expected UTF-8 JSON") from error

    def parse_constant(token: str) -> JsonConstant:
        if not allow_constants:
            raise ValueError(f"{label}: nonstandard JSON constant {token!r}")
        return JsonConstant(token)

    try:
        return json.loads(
            text,
            object_pairs_hook=_unique_object,
            parse_constant=parse_constant,
        )
    except json.JSONDecodeError as error:
        raise ValueError(f"{label}: malformed JSON: {error}") from error


def read_json(path: Path, *, allow_constants: bool = False) -> Any:
    return parse_json_bytes(
        path.read_bytes(), label=str(path), allow_constants=allow_constants
    )


def write_json(path: Path, payload: Any) -> None:
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


def _manifest_static() -> dict[str, Any]:
    cases = []
    for case in CASES:
        cases.append(
            {
                "case_id": case.case_id,
                "run_id": case.run_id,
                "mu": case.mu,
                "omega": case.omega,
                "expected_outcome": case.expected_outcome,
                "first_exit_index": case.first_exit_index,
                "first_tangential_nan_index": (case.first_tangential_nan_index),
                "contact_steps": case.contact_steps,
                "max_outer_iterations": case.max_outer_iterations,
                "nonconverged_steps": list(case.nonconverged_steps),
                "initial_converged_steps": case.initial_converged_steps,
                "natural_residual_over_tolerance_converged_steps": (
                    case.natural_residual_over_tolerance_converged_steps
                ),
                "solver_contract_valid": not case.nonconverged_steps,
            }
        )
    invocations = []
    for run_id, run in RUNS.items():
        invocations.append(
            {
                "run_id": run_id,
                "argv": run["invocation"],
                "cwd": "/tmp/fbf-sca-2026-author",
                "returncode": 0,
                "stdout_retained": False,
                "stderr_retained": False,
                "warmup_steps": 0,
                "repetitions": 1,
                "timing_includes_first_use_compilation": True,
                "source_identity_captured_before_and_after": False,
                "environment_identity_captured_before_and_after": False,
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "source": {
            "repository": SOURCE_REPOSITORY,
            "commit": SOURCE_COMMIT,
            "tree": SOURCE_TREE,
            "history_reported_short_commit": SOURCE_SHORT_COMMIT,
            "runner_path": RUNNER_PATH,
            "runner_sha256": RUNNER_SHA256,
            "runner_git_blob": RUNNER_GIT_BLOB,
            "requirements_path": "requirements.txt",
            "requirements_sha256": REQUIREMENTS_SHA256,
            "clean_commit_observed_after_runs": True,
            "pre_and_post_run_cleanliness_captured": False,
            "observation_timestamp_utc": "2026-07-18T21:59:00Z",
        },
        "environment": {
            "invoked_interpreter": "/tmp/fbf-author-venv/bin/python",
            "resolved_interpreter": (
                "/home/js/.local/share/uv/python/"
                "cpython-3.11.15-linux-x86_64-gnu/bin/python3.11"
            ),
            "resolved_interpreter_sha256": (
                "6759dcc6aa5a3fe3377349092f92f6700f1f79017294c1d0085009eb8802cb1e"
            ),
            "python_implementation": "CPython",
            "python_version": ("3.11.15 (main, May 10 2026, 19:28:18) [Clang 22.1.3 ]"),
            "platform": "Linux-7.0.0-27-generic-x86_64-with-glibc2.43",
            "machine": "x86_64",
            "installed_distribution_count": 65,
            "installed_distributions": list(INSTALLED_DISTRIBUTIONS),
            "installed_distributions_sha256": (
                "d86b1f09415cdd3fa849dd468284068f2889b1b7bb47397ff20b99a424d099b3"
            ),
            "requirements_exact_match": True,
            "observation_timing": "post_run",
        },
        "invocations": invocations,
        "cases": cases,
        "predicates": copy.deepcopy(EXPECTED_PREDICATES),
        "nonclaims": list(NONCLAIMS),
        "omitted_redundant_artifacts": [
            {
                "path": "runs/20260718T214609Z/sweep_results.json",
                "source_bytes": 22166,
                "source_sha256": (
                    "c11e81cbebaaab684d01e5c7ad9a30bec582c649c4e792f4eebd622be14ef561"
                ),
                "reason": "semantic duplicate of the two retained result files",
            },
            {
                "path": "runs/20260718T214657Z/sweep_results.json",
                "source_bytes": 22199,
                "source_sha256": (
                    "d996e97ddd9e3ff5ba1bdb50097bacdcf720fe984fea0e680b73cc7284e88811"
                ),
                "reason": "semantic duplicate of the two retained result files",
            },
        ],
    }


def expected_manifest() -> dict[str, Any]:
    payload = _manifest_static()
    payload.update(
        {
            "artifact_count": len(EXPECTED_ARTIFACTS),
            "stored_artifact_bytes": EXPECTED_STORED_BYTES,
            "artifacts": copy.deepcopy(list(EXPECTED_ARTIFACTS)),
        }
    )
    return payload


def validate_manifest_contract(payload: Any) -> dict[str, Any]:
    if not exact_json_equal(payload, expected_manifest()):
        raise ValueError(
            "author turntable manifest changed or promotes a forbidden claim"
        )
    return payload


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


def _expected_directories() -> set[str]:
    directories: set[str] = set()
    for relative in EXPECTED_ARTIFACT_PATHS:
        parent = PurePosixPath(relative).parent
        while parent != PurePosixPath("."):
            directories.add(parent.as_posix())
            parent = parent.parent
    return directories


def validate_membership(bundle: Path) -> None:
    if bundle.is_symlink() or not bundle.is_dir():
        raise ValueError("author turntable bundle must be a regular directory")
    actual_files: set[str] = set()
    actual_directories: set[str] = set()
    for path in bundle.rglob("*"):
        relative = path.relative_to(bundle).as_posix()
        mode = path.lstat().st_mode
        if stat.S_ISLNK(mode):
            raise ValueError(f"author turntable bundle contains symlink: {relative}")
        if stat.S_ISDIR(mode):
            actual_directories.add(relative)
        elif stat.S_ISREG(mode):
            actual_files.add(relative)
        else:
            raise ValueError(
                f"author turntable bundle contains non-regular entry: {relative}"
            )
    expected_files = EXPECTED_ARTIFACT_PATHS | {"manifest.json"}
    if actual_files != expected_files:
        raise ValueError(
            "author turntable bundle file membership changed: "
            f"expected {sorted(expected_files)}, got {sorted(actual_files)}"
        )
    expected_directories = _expected_directories()
    if actual_directories != expected_directories:
        raise ValueError(
            "author turntable bundle directory membership changed: "
            f"expected {sorted(expected_directories)}, "
            f"got {sorted(actual_directories)}"
        )


def reject_symlink_path_components(path: Path) -> None:
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if current.is_symlink():
            raise ValueError(
                f"author turntable bundle path contains symlink: {current}"
            )


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


def decompress_history(path: Path, *, raw_bytes: int, raw_sha256: str) -> bytes:
    compressed = path.read_bytes()
    header = compressed[:10]
    if len(header) != 10 or header[:4] != b"\x1f\x8b\x08\x00":
        raise ValueError(f"{path}: expected deterministic gzip without optional fields")
    if header[4:8] != b"\x00\x00\x00\x00":
        raise ValueError(f"{path}: gzip mtime must be zero")
    output = bytearray()
    decompressor = zlib.decompressobj(wbits=16 + zlib.MAX_WBITS)
    try:
        for offset in range(0, len(compressed), GZIP_CHUNK_BYTES):
            pending = compressed[offset : offset + GZIP_CHUNK_BYTES]
            while pending:
                remaining = raw_bytes - len(output)
                if remaining < 0:
                    raise ValueError(f"{path}: decompressed history exceeds size bound")
                output.extend(decompressor.decompress(pending, remaining + 1))
                if len(output) > raw_bytes:
                    raise ValueError(f"{path}: decompressed history exceeds size bound")
                pending = decompressor.unconsumed_tail
        remaining = raw_bytes - len(output)
        output.extend(decompressor.flush(remaining + 1))
    except zlib.error as error:
        raise ValueError(f"{path}: invalid gzip history") from error
    if not decompressor.eof:
        raise ValueError(f"{path}: truncated gzip history")
    if decompressor.unused_data or decompressor.unconsumed_tail:
        raise ValueError(f"{path}: gzip history contains trailing data")
    raw = bytes(output)
    if len(raw) != raw_bytes:
        raise ValueError(
            f"{path}: decompressed history size changed: {len(raw)} != {raw_bytes}"
        )
    if sha256_bytes(raw) != raw_sha256:
        raise ValueError(f"{path}: decompressed history content changed")
    return raw


def validate_artifacts(bundle: Path, manifest: dict[str, Any]) -> dict[str, bytes]:
    histories: dict[str, bytes] = {}
    records = manifest["artifacts"]
    if not exact_json_equal(records, list(EXPECTED_ARTIFACTS)):
        raise ValueError("author turntable artifact index changed")
    for record in records:
        relative = record["path"]
        _safe_relative_path(relative)
        path = bundle / relative
        if path.stat().st_size != record["bytes"]:
            raise ValueError(f"{relative}: artifact size changed")
        if sha256(path) != record["sha256"]:
            raise ValueError(f"{relative}: artifact content changed")
        if record["encoding"] == "gzip-n9":
            histories[relative] = decompress_history(
                path,
                raw_bytes=record["raw_bytes"],
                raw_sha256=record["raw_sha256"],
            )
        elif record["encoding"] != "raw":
            raise ValueError(f"{relative}: unsupported artifact encoding")
    return histories


def validate_metadata(data: Any, *, run_id: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError(f"{run_id}: metadata must be an object")
    run = RUNS[run_id]
    expected = {
        "experiment": "turntable_friction_sweep",
        "mu": run["mu"],
        "cube_offset_r": CUBE_OFFSET_RADIUS,
        "cube_half_size": CUBE_HALF_SIZE,
        "turntable_radius": TURN_TABLE_RADIUS,
        "turntable_half_h": TURN_TABLE_HALF_HEIGHT,
        "omega_crit_ref": math.sqrt(run["mu"] * GRAVITY / CUBE_OFFSET_RADIUS),
        "omega_crit_note": (
            "sqrt(mu*g/r) is a point-mass reference scale; the finite cube on "
            "a finite contact patch does not have an exact bifurcation at this "
            "value."
        ),
        "omega_values": [2.0, 5.0],
        "gravity": GRAVITY,
        "dt": DT_SECONDS,
        "T_seconds": 6.0,
        "num_steps": TOTAL_STEPS,
        "ramp_time": 0.5,
        "steady_window": 1.0,
        "solvers": ["fbf"],
        "device": "cpu",
        "gap": 0.005,
        "drop_height": 0.2,
        "timestamp": run_id,
    }
    if not exact_json_equal(data, expected):
        raise ValueError(f"{run_id}: metadata contract changed")
    return data


def _find_json_constants(value: Any, prefix: str = "$") -> list[tuple[str, str]]:
    found: list[tuple[str, str]] = []
    if isinstance(value, JsonConstant):
        found.append((prefix, value.token))
    elif isinstance(value, dict):
        for key, item in value.items():
            found.extend(_find_json_constants(item, f"{prefix}.{key}"))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            found.extend(_find_json_constants(item, f"{prefix}[{index}]"))
    return found


def read_result(path: Path, case: CaseSpec) -> dict[str, Any]:
    data = read_json(path, allow_constants=True)
    if not isinstance(data, dict) or set(data) != RESULT_KEYS:
        raise ValueError(f"{case.case_id}: result fields changed")
    constants = _find_json_constants(data)
    if case.first_exit_index is None:
        if constants:
            raise ValueError(f"{case.case_id}: result contains nonfinite JSON")
    else:
        if constants != [("$.final_tangential_arc", "NaN")]:
            raise ValueError(
                f"{case.case_id}: only final_tangential_arc may be JSON NaN"
            )
        data["final_tangential_arc"] = math.nan
    return data


def load_trajectory(path: Path) -> dict[str, np.ndarray]:
    try:
        with zipfile.ZipFile(path) as archive:
            infos = archive.infolist()
            names = [info.filename for info in infos]
            if len(names) != len(set(names)):
                raise ValueError(f"{path}: duplicate NPZ member")
            if tuple(names) != tuple(f"{key}.npy" for key in TRAJECTORY_KEYS):
                raise ValueError(f"{path}: unexpected NPZ member set")
            total = 0
            for info in infos:
                member = PurePosixPath(info.filename)
                if (
                    info.is_dir()
                    or len(member.parts) != 1
                    or member.name != info.filename
                ):
                    raise ValueError(f"{path}: unsafe NPZ member path")
                if info.flag_bits & 0x1:
                    raise ValueError(f"{path}: encrypted NPZ member")
                if info.compress_type != zipfile.ZIP_STORED:
                    raise ValueError(f"{path}: NPZ members must be stored")
                if info.file_size != 3008 or info.compress_size != 3008:
                    raise ValueError(f"{path}: NPZ member size changed")
                if info.file_size > MAX_NPZ_MEMBER_BYTES:
                    raise ValueError(f"{path}: NPZ member exceeds size bound")
                total += info.file_size
                if len(archive.read(info)) != info.file_size:
                    raise ValueError(f"{path}: incomplete NPZ member")
            if total > MAX_NPZ_TOTAL_BYTES:
                raise ValueError(f"{path}: NPZ payload exceeds size bound")
    except zipfile.BadZipFile as error:
        raise ValueError(f"{path}: invalid NPZ container") from error

    arrays: dict[str, np.ndarray] = {}
    try:
        with np.load(path, allow_pickle=False) as archive:
            if tuple(archive.files) != TRAJECTORY_KEYS:
                raise ValueError(f"{path}: NPZ key order changed")
            for key in TRAJECTORY_KEYS:
                array = archive[key]
                if array.dtype != np.dtype(np.float64):
                    raise ValueError(f"{path}: {key} dtype must be float64")
                if array.shape != (TOTAL_STEPS,):
                    raise ValueError(f"{path}: {key} shape must be ({TOTAL_STEPS},)")
                arrays[key] = array.copy()
    except (OSError, ValueError) as error:
        if isinstance(error, ValueError) and str(error).startswith(str(path)):
            raise
        raise ValueError(f"{path}: invalid non-pickle NPZ payload") from error
    return arrays


def _close(actual: float, expected: float, *, tolerance: float = 1.0e-12) -> bool:
    return math.isclose(actual, expected, rel_tol=1.0e-12, abs_tol=tolerance)


def _expected_omega_command(omega: float) -> np.ndarray:
    values = []
    for step in range(TOTAL_STEPS):
        sim_time = step * DT_SECONDS
        if sim_time < 0.5:
            value = 0.0
        elif sim_time < 1.0:
            fraction = (sim_time - 0.5) / 0.5
            value = omega * fraction * fraction * (3.0 - 2.0 * fraction)
        else:
            value = omega
        values.append(value)
    return np.asarray(values, dtype=np.float64)


def summarize_trajectory(
    arrays: dict[str, np.ndarray], case: CaseSpec
) -> dict[str, Any]:
    for key, array in arrays.items():
        finite = np.isfinite(array)
        if key == "tangential_arc":
            continue
        if not bool(np.all(finite)):
            raise ValueError(f"{case.case_id}: nonfinite trajectory array {key}")
    expected_time = (np.arange(TOTAL_STEPS, dtype=np.float64) + 1.0) / 60.0
    if not np.allclose(arrays["t"], expected_time, rtol=0.0, atol=2.0e-14):
        raise ValueError(f"{case.case_id}: trajectory time grid changed")
    if not np.array_equal(arrays["radial_disp"], arrays["r"] - CUBE_OFFSET_RADIUS):
        raise ValueError(f"{case.case_id}: radial displacement is inconsistent")
    expected_command = _expected_omega_command(case.omega)
    if not np.allclose(
        arrays["omega_command"], expected_command, rtol=0.0, atol=2.0e-14
    ):
        raise ValueError(f"{case.case_id}: turntable command schedule changed")
    if not np.allclose(
        arrays["omega_turntable_actual"],
        arrays["omega_command"],
        rtol=0.0,
        atol=2.0e-7,
    ):
        raise ValueError(f"{case.case_id}: actual turntable speed drifted")

    exit_mask = (arrays["r"] > EXIT_RADIUS) | (arrays["z"] < EXIT_Z)
    exit_indices = np.flatnonzero(exit_mask)
    first_exit = int(exit_indices[0]) if exit_indices.size else None
    if first_exit != case.first_exit_index:
        raise ValueError(
            f"{case.case_id}: first author exit index {first_exit} does not "
            f"match {case.first_exit_index}"
        )
    nan_indices = np.flatnonzero(np.isnan(arrays["tangential_arc"]))
    first_nan = int(nan_indices[0]) if nan_indices.size else None
    if first_nan != case.first_tangential_nan_index:
        raise ValueError(
            f"{case.case_id}: first tangential NaN {first_nan} does not "
            f"match {case.first_tangential_nan_index}"
        )
    if first_nan is None:
        if not np.all(np.isfinite(arrays["tangential_arc"])):
            raise ValueError(f"{case.case_id}: unexpected tangential nonfinite value")
    else:
        if not np.all(np.isfinite(arrays["tangential_arc"][:first_nan])):
            raise ValueError(f"{case.case_id}: premature tangential NaN")
        if not np.all(np.isnan(arrays["tangential_arc"][first_nan:])):
            raise ValueError(f"{case.case_id}: post-exit tangential latch changed")

    tail_tan = np.abs(arrays["slip_tangential"][-STEADY_WINDOW_STEPS:])
    tail_rad = np.abs(arrays["slip_radial"][-STEADY_WINDOW_STEPS:])
    tail_tan = tail_tan[~np.isnan(tail_tan)]
    tail_rad = tail_rad[~np.isnan(tail_rad)]
    final_tangential = float(arrays["tangential_arc"][-1])
    summary = {
        "classified_outcome": (
            "retained_through_horizon" if first_exit is None else "left_table_boundary"
        ),
        "first_exit_index": first_exit,
        "first_exit_time_seconds": (
            None if first_exit is None else float(arrays["t"][first_exit])
        ),
        "first_exit_reason": (
            None
            if first_exit is None
            else ("radial" if arrays["r"][first_exit] > EXIT_RADIUS else "vertical")
        ),
        "first_tangential_nan_index": first_nan,
        "final_radial_disp": float(arrays["radial_disp"][-1]),
        "final_tangential_arc": (
            None if math.isnan(final_tangential) else final_tangential
        ),
        "v_slip_tan_mean_last_window": float(np.mean(tail_tan)),
        "v_slip_tan_max_last_window": float(np.max(tail_tan)),
        "v_slip_rad_mean_last_window": float(np.mean(tail_rad)),
        "v_slip_rad_max_last_window": float(np.max(tail_rad)),
    }
    if summary["classified_outcome"] != case.expected_outcome:
        raise ValueError(f"{case.case_id}: physical outcome changed")
    if first_exit is not None and summary["first_exit_reason"] != "radial":
        raise ValueError(f"{case.case_id}: first exit is no longer radial")
    if not _close(summary["final_radial_disp"], case.final_radial_displacement):
        raise ValueError(f"{case.case_id}: final radial displacement changed")
    expected_tangential = case.final_tangential_arc
    if expected_tangential is None:
        if summary["final_tangential_arc"] is not None:
            raise ValueError(f"{case.case_id}: final tangential arc must be absent")
    elif not _close(summary["final_tangential_arc"], expected_tangential):
        raise ValueError(f"{case.case_id}: final tangential arc changed")
    return summary


def validate_result(
    result: dict[str, Any],
    trajectory: dict[str, Any],
    case: CaseSpec,
) -> dict[str, Any]:
    fixed = {
        "solver": "fbf",
        "mu": case.mu,
        "omega": case.omega,
        "drop_height": 0.2,
        "dt": DT_SECONDS,
        "num_steps": TOTAL_STEPS,
        "ramp_time": 0.5,
        "steady_window": 1.0,
        "usd": None,
        "T_seconds": 6.0,
    }
    for field, expected in fixed.items():
        if not exact_json_equal(result[field], expected):
            raise ValueError(f"{case.case_id}: result {field} changed")
    ratio = case.omega / math.sqrt(case.mu * GRAVITY / CUBE_OFFSET_RADIUS)
    if not _close(result["omega_over_crit"], ratio):
        raise ValueError(f"{case.case_id}: omega reference ratio changed")
    metric_fields = (
        "v_slip_tan_mean_last_window",
        "v_slip_tan_max_last_window",
        "v_slip_rad_mean_last_window",
        "v_slip_rad_max_last_window",
        "final_radial_disp",
    )
    for field in metric_fields:
        if not _close(result[field], trajectory[field]):
            raise ValueError(
                f"{case.case_id}: result metric {field} does not match trajectory"
            )
    if trajectory["final_tangential_arc"] is None:
        if not math.isnan(result["final_tangential_arc"]):
            raise ValueError(f"{case.case_id}: result tangential arc must be NaN")
    elif not _close(result["final_tangential_arc"], trajectory["final_tangential_arc"]):
        raise ValueError(f"{case.case_id}: result tangential arc mismatch")
    step_times = result["step_times_ms"]
    if (
        not isinstance(step_times, list)
        or len(step_times) != TOTAL_STEPS
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0.0
            for value in step_times
        )
    ):
        raise ValueError(f"{case.case_id}: invalid step timing list")
    if not _close(result["avg_step_time_ms"], float(np.mean(step_times))):
        raise ValueError(f"{case.case_id}: average timing does not match samples")
    config = result["config"]
    if not isinstance(config, dict) or set(config) != CONFIG_KEYS:
        raise ValueError(f"{case.case_id}: result solver config fields changed")
    expected_config = dict(EXPECTED_CONFIG)
    expected_config["history_path"] = (
        f"/tmp/fbf-sca-2026-author/paper_examples/turntable/results/"
        f"{case.run_id}/{case.directory}/history.json"
    )
    if not exact_json_equal(config, expected_config):
        raise ValueError(f"{case.case_id}: result solver config changed")
    return result


def summarize_history(history: Any, case: CaseSpec) -> dict[str, Any]:
    if not isinstance(history, dict) or set(history) != {"meta", "steps"}:
        raise ValueError(f"{case.case_id}: history top-level schema changed")
    meta = history["meta"]
    if not isinstance(meta, dict) or set(meta) != {
        "solver",
        "inner_solver",
        "num_bodies",
        "config",
        "schema_version",
        "bench",
        "dt",
    }:
        raise ValueError(f"{case.case_id}: history metadata fields changed")
    expected_meta_fixed = {
        "solver": "FBF",
        "inner_solver": "block_gs",
        "num_bodies": 2,
        "schema_version": 2,
        "dt": DT_SECONDS,
        "bench": {
            "scene": "turntable",
            "scene_params": {
                "mu": case.mu,
                "omega": case.omega,
                "drop_height": 0.2,
            },
            "warmup_steps": 0,
            "total_steps": TOTAL_STEPS,
            "git_sha": SOURCE_SHORT_COMMIT,
        },
    }
    for field, expected in expected_meta_fixed.items():
        if not exact_json_equal(meta[field], expected):
            raise ValueError(f"{case.case_id}: history metadata {field} changed")
    config = meta["config"]
    if not isinstance(config, dict) or set(config) != CONFIG_KEYS:
        raise ValueError(f"{case.case_id}: history solver config fields changed")
    expected_config = dict(EXPECTED_CONFIG)
    expected_config["history_path"] = (
        f"paper_examples/turntable/results/{case.run_id}/"
        f"{case.directory}/history.json"
    )
    if not exact_json_equal(config, expected_config):
        raise ValueError(f"{case.case_id}: history solver config changed")

    steps = history["steps"]
    if not isinstance(steps, list) or len(steps) != TOTAL_STEPS:
        raise ValueError(f"{case.case_id}: history must contain 360 steps")
    contact_steps = 0
    nonconverged: list[int] = []
    cap_hits: list[int] = []
    max_outer = 0
    converged_coulomb: list[float] = []
    initial_converged = 0
    natural_over_tolerance_converged = 0
    for index, step in enumerate(steps):
        if not isinstance(step, dict) or set(step) != STEP_KEYS:
            raise ValueError(f"{case.case_id}: history step fields changed")
        if step["step_idx"] != index:
            raise ValueError(f"{case.case_id}: noncontiguous history step")
        if type(step["num_contacts"]) is not int or step["num_contacts"] < 0:
            raise ValueError(f"{case.case_id}: invalid contact count at step {index}")
        if type(step["converged"]) is not bool or type(step["warmup"]) is not bool:
            raise ValueError(f"{case.case_id}: invalid history booleans")
        if step["warmup"]:
            raise ValueError(f"{case.case_id}: unexpected warmup step {index}")
        outer = step["outer"]
        if (
            type(step["outer_iters"]) is not int
            or step["outer_iters"] < 0
            or not isinstance(outer, list)
            or len(outer) != step["outer_iters"]
        ):
            raise ValueError(f"{case.case_id}: inconsistent outer history")
        for outer_index, record in enumerate(outer):
            if not isinstance(record, dict) or set(record) != OUTER_KEYS:
                raise ValueError(f"{case.case_id}: outer iteration fields changed")
            if record["k"] != outer_index:
                raise ValueError(f"{case.case_id}: noncontiguous outer iteration")
        if outer and not _close(step["final_residual"], outer[-1]["residual"]):
            raise ValueError(
                f"{case.case_id}: terminal natural residual changed at step {index}"
            )
        max_outer = max(max_outer, step["outer_iters"])
        if step["outer_iters"] >= config["max_outer"]:
            cap_hits.append(index)
        if step["num_contacts"] == 0:
            if (
                not step["converged"]
                or step["outer_iters"] != 0
                or step["final_residual"] != 0.0
            ):
                raise ValueError(
                    f"{case.case_id}: contact-free step {index} contract changed"
                )
            continue
        contact_steps += 1
        if step["converged"]:
            if step["outer_iters"] == 0:
                if not step["initial_residual"] < TERMINATION_TOLERANCE:
                    raise ValueError(
                        f"{case.case_id}: invalid initial convergence at step {index}"
                    )
                initial_converged += 1
            else:
                terminal = outer[-1]["r_coulomb"]
                if (
                    isinstance(terminal, bool)
                    or not isinstance(terminal, (int, float))
                    or not math.isfinite(terminal)
                    or terminal < 0.0
                    or terminal >= TERMINATION_TOLERANCE
                ):
                    raise ValueError(
                        f"{case.case_id}: converged step {index} violates "
                        "coulomb_rel tolerance"
                    )
                converged_coulomb.append(float(terminal))
            if step["final_residual"] >= TERMINATION_TOLERANCE:
                natural_over_tolerance_converged += 1
        else:
            nonconverged.append(index)
            if step["outer_iters"] == 0:
                raise ValueError(
                    f"{case.case_id}: nonconverged step {index} has no iterations"
                )
            terminal = outer[-1]["r_coulomb"]
            if terminal >= 0.0 and terminal < TERMINATION_TOLERANCE:
                raise ValueError(
                    f"{case.case_id}: nonconverged step {index} satisfies tolerance"
                )
            if case.case_id == "mu_0_2_omega_2" and index == 130:
                if (
                    step["outer_iters"] != 40
                    or not _close(
                        step["final_residual"],
                        0.0009073968376424851,
                        tolerance=1.0e-15,
                    )
                    or not _close(
                        terminal,
                        0.00014589023205034332,
                        tolerance=1.0e-15,
                    )
                ):
                    raise ValueError(
                        f"{case.case_id}: expected plateau step 130 changed"
                    )
    if contact_steps != case.contact_steps:
        raise ValueError(f"{case.case_id}: contact-step count changed")
    if tuple(nonconverged) != case.nonconverged_steps:
        raise ValueError(f"{case.case_id}: nonconverged-step set changed")
    if cap_hits:
        raise ValueError(f"{case.case_id}: unexpected outer cap hit")
    if max_outer != case.max_outer_iterations:
        raise ValueError(f"{case.case_id}: maximum outer iteration count changed")
    if initial_converged != case.initial_converged_steps:
        raise ValueError(f"{case.case_id}: initial-convergence count changed")
    if (
        natural_over_tolerance_converged
        != case.natural_residual_over_tolerance_converged_steps
    ):
        raise ValueError(
            f"{case.case_id}: natural-residual convergence evidence changed"
        )
    maximum_coulomb = max(converged_coulomb)
    if not _close(
        maximum_coulomb,
        case.max_converged_coulomb_residual,
        tolerance=1.0e-15,
    ):
        raise ValueError(f"{case.case_id}: converged Coulomb residual changed")
    return {
        "contact_steps": contact_steps,
        "nonconverged_steps": nonconverged,
        "max_outer_iterations": max_outer,
        "outer_cap_hit_steps": cap_hits,
        "max_converged_coulomb_residual": maximum_coulomb,
        "initial_converged_steps": initial_converged,
        "natural_residual_over_tolerance_converged_steps": (
            natural_over_tolerance_converged
        ),
        "solver_contract_valid": not nonconverged,
    }


def validate_bundle(bundle: Path = DEFAULT_BUNDLE) -> dict[str, Any]:
    bundle = Path(os.path.abspath(bundle))
    reject_symlink_path_components(bundle)
    validate_membership(bundle)
    manifest = validate_manifest_contract(read_json(bundle / "manifest.json"))
    histories = validate_artifacts(bundle, manifest)

    metadata = {}
    for run_id, run in RUNS.items():
        metadata[run_id] = validate_metadata(
            read_json(bundle / run["metadata_path"]), run_id=run_id
        )

    case_reports = []
    for case in CASES:
        if metadata[case.run_id]["mu"] != case.mu:
            raise ValueError(f"{case.case_id}: metadata/run binding changed")
        arrays = load_trajectory(bundle / case.trajectory_path)
        trajectory = summarize_trajectory(arrays, case)
        result = read_result(bundle / case.result_path, case)
        validate_result(result, trajectory, case)
        history = parse_json_bytes(
            histories[case.history_path], label=case.history_path
        )
        history_summary = summarize_history(history, case)
        case_reports.append(
            {
                "case_id": case.case_id,
                "mu": case.mu,
                "omega": case.omega,
                "trajectory": trajectory,
                "solver": history_summary,
                "physical_outcome_valid": True,
                "solver_contract_valid": history_summary["solver_contract_valid"],
            }
        )

    computed_solver = {
        report["case_id"]: report["solver_contract_valid"] for report in case_reports
    }
    if computed_solver != EXPECTED_PREDICATES["solver_contract_by_case"]:
        raise ValueError("author turntable solver predicate matrix changed")
    if all(computed_solver.values()):
        raise ValueError("author turntable expected partial-negative was promoted")
    return {
        "schema_version": VALIDATION_SCHEMA_VERSION,
        "pass": True,
        "artifact_count": len(EXPECTED_ARTIFACTS),
        "stored_artifact_bytes": EXPECTED_STORED_BYTES,
        "source_commit": SOURCE_COMMIT,
        "cases": case_reports,
        "predicates": copy.deepcopy(EXPECTED_PREDICATES),
        "nonclaims": list(NONCLAIMS),
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
    if _git_output(source_repo, "rev-parse", "HEAD") != SOURCE_COMMIT:
        raise ValueError("author source checkout is not at the pinned commit")
    if _git_output(source_repo, "rev-parse", "HEAD^{tree}") != SOURCE_TREE:
        raise ValueError("author source tree identity changed")
    if _git_output(source_repo, "remote", "get-url", "origin") != SOURCE_REPOSITORY:
        raise ValueError("author source remote changed")
    if _git_output(source_repo, "status", "--porcelain=v1"):
        raise ValueError("author source checkout must be clean")
    if sha256(source_repo / RUNNER_PATH) != RUNNER_SHA256:
        raise ValueError("author turntable runner content changed")
    if sha256(source_repo / "requirements.txt") != REQUIREMENTS_SHA256:
        raise ValueError("author requirements lock changed")


def _gzip_n9(data: bytes) -> bytes:
    process = subprocess.run(
        ["gzip", "-n", "-9", "-c"],
        input=data,
        check=True,
        capture_output=True,
    )
    return process.stdout


def finalize_bundle(
    source_repo: Path = DEFAULT_SOURCE_REPO,
    bundle: Path = DEFAULT_BUNDLE,
) -> dict[str, Any]:
    reject_symlink_path_components(bundle)
    source_repo = source_repo.resolve(strict=True)
    _validate_source_repo(source_repo)
    if bundle.exists():
        raise ValueError(f"refusing to replace existing bundle: {bundle}")
    bundle.mkdir(parents=True)
    try:
        reject_symlink_path_components(bundle)
        for run_id, run in RUNS.items():
            source_run = source_repo / run["source_directory"]
            metadata_destination = bundle / run["metadata_path"]
            metadata_destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source_run / "metadata.json", metadata_destination)
            for case in (item for item in CASES if item.run_id == run_id):
                source_case = source_run / case.directory
                destination = bundle / case.prefix
                destination.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(
                    source_case / "result.json", destination / "result.json"
                )
                shutil.copyfile(
                    source_case / "trajectory.npz", destination / "trajectory.npz"
                )
                history = (source_case / "history.json").read_bytes()
                (destination / "history.json.gz").write_bytes(_gzip_n9(history))
        _validate_source_repo(source_repo)
        write_json(bundle / "manifest.json", expected_manifest())
        return validate_bundle(bundle)
    except Exception:
        _remove_bundle_root(bundle)
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Finalize or verify the pinned author FBF turntable reference"
    )
    parser.add_argument("--bundle", type=Path, default=DEFAULT_BUNDLE)
    parser.add_argument("--source-repo", type=Path, default=DEFAULT_SOURCE_REPO)
    parser.add_argument("--verify-only", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.verify_only:
            report = validate_bundle(args.bundle)
        else:
            report = finalize_bundle(args.source_repo, args.bundle)
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
