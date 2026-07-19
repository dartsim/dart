#!/usr/bin/env python3
# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

"""Produce a durable, explicitly non-parity MuJoCo evidence bundle.

The runner executes the seven reconstructed small FBF fixtures sequentially,
records every physical state and synchronous ``mujoco.mj_step`` wall time, and
writes raw CSV, aggregates, provenance/contract metadata, checksums, and a
human-readable report. It must be run from the optional ``fbf-baselines``
environment; MuJoCo is intentionally not a default DART dependency.
"""

import argparse
import csv
import datetime as dt
import hashlib
import importlib.metadata
import importlib.util
import json
import math
import os
import pathlib
import platform
import statistics
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
BASELINE_PATH = REPO_ROOT / "tests/benchmark/integration/fbf_paper_mujoco_baseline.py"

SCHEMA_VERSION = 1
THREAD_ENV_VARS = (
    "OMP_NUM_THREADS",
    "MKL_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
    "NUMEXPR_NUM_THREADS",
)
OWNED_ARTIFACTS = (
    "metadata.json",
    "raw.csv",
    "summary.csv",
    "summary.json",
    "REPORT.md",
)

PAPER_TABLE5 = {
    "backspin": {"fbf_ms": 6.0, "mujoco_relative": 0.3},
    "incline_mu_0_5": {"fbf_ms": 5.5, "mujoco_relative": 0.4},
    "incline_mu_0_4": {"fbf_ms": 5.3, "mujoco_relative": 0.6},
    "turntable_mu_0_5_omega_2": {"fbf_ms": 6.8, "mujoco_relative": 0.6},
    "turntable_mu_0_5_omega_5": {"fbf_ms": 3.1, "mujoco_relative": 1.2},
    "turntable_mu_0_2_omega_2": {"fbf_ms": 3.7, "mujoco_relative": 1.1},
    "turntable_mu_0_2_omega_5": {"fbf_ms": 3.1, "mujoco_relative": 1.2},
}

SCENE_CONTRACTS = {
    "backspin": {
        "published": {
            "shape": "uniform solid sphere",
            "radius_m": 0.25,
            "initial_linear_velocity_mps": [4.0, 0.0, 0.0],
            "initial_angular_velocity_rad_s": [0.0, -200.0, 0.0],
            "friction": 0.5,
            "timestep_s": 1.0 / 60.0,
            "figure_instants_s": [0.0, 10.0 / 60.0, 50.0 / 60.0, 2.0, 130.0 / 60.0],
        },
        "local_reconstruction": {
            "mass_kg": 1.0,
            "initial_penetration_m": 0.005,
            "trajectory_end_s": 130.0 / 60.0,
        },
        "unknown_or_unpublished": [
            "sphere mass",
            "initial contact penetration or gap",
            "MuJoCo contact solref and solimp",
        ],
    },
    "incline_mu_0_5": {
        "published": {
            "shape": "cube",
            "slope_tangent": 0.5,
            "friction": 0.5,
            "timestep_s": 1.0 / 60.0,
            "duration_s": 2.0,
        },
        "local_reconstruction": {
            "cube_size_m": [1.0, 1.0, 1.0],
            "mass_kg": 1.0,
            "initial_penetration_m": 0.01,
        },
        "unknown_or_unpublished": [
            "cube size and mass",
            "initial contact penetration or gap",
            "MuJoCo contact solref and solimp",
        ],
    },
    "incline_mu_0_4": {
        "published": {
            "shape": "cube",
            "slope_tangent": 0.5,
            "friction": 0.4,
            "timestep_s": 1.0 / 60.0,
            "duration_s": 2.0,
        },
        "local_reconstruction": {
            "cube_size_m": [1.0, 1.0, 1.0],
            "mass_kg": 1.0,
            "initial_penetration_m": 0.01,
        },
        "unknown_or_unpublished": [
            "cube size and mass",
            "initial contact penetration or gap",
            "MuJoCo contact solref and solimp",
        ],
    },
}


def _turntable_contract(mu, omega):
    return {
        "published": {
            "shape": "cube on rotating turntable",
            "friction": mu,
            "target_angular_velocity_rad_s": omega,
            "angular_velocity_profile": "ramped smoothly from rest",
            "timestep_s": 1.0 / 60.0,
        },
        "local_reconstruction": {
            "turntable_size_m": [4.0, 4.0, 0.1],
            "rider_size_m": [0.25, 0.25, 0.25],
            "rider_mass_kg": 1.0,
            "initial_radius_m": 1.0,
            "linear_ramp_duration_s": 1.0,
            "initial_penetration_m": 0.005,
            "trajectory_end_s": 4.0,
            "drive": "hinge qvel overwritten before each mj_step",
        },
        "unknown_or_unpublished": [
            "turntable and rider dimensions and mass",
            "initial radius and contact penetration or gap",
            "ramp function and duration",
            "simulation duration",
            "kinematic drive implementation",
            "MuJoCo contact solref and solimp",
        ],
    }


for _mu in (0.2, 0.5):
    for _omega in (2, 5):
        SCENE_CONTRACTS[f"turntable_mu_{str(_mu).replace('.', '_')}_omega_{_omega}"] = (
            _turntable_contract(_mu, float(_omega))
        )


RAW_FIELDS = (
    "run_id",
    "repetition",
    "step",
    "sim_time_s",
    "scenario",
    "solver",
    "timed_step",
    "step_wall_ns",
    "step_wall_ms",
    "body",
    "x_m",
    "y_m",
    "z_m",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "wx_rad_s",
    "wy_rad_s",
    "wz_rad_s",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
    "up_z",
    "speed_mps",
    "angular_speed_rad_s",
    "dx_m",
    "dy_m",
    "dz_m",
    "down_slope_displacement_m",
    "radial_distance_m",
    "azimuth_rad",
    "turntable_angle_rad",
    "turntable_relative_phase_rad",
    "turntable_command_omega_rad_s",
    "contacts",
    "contact_constraints",
    "min_contact_distance_m",
    "airborne",
    "solver_iterations_sum",
    "solver_iterations_max_island",
    "solver_nnz_sum",
)

SUMMARY_FIELDS = (
    "scenario",
    "repetitions",
    "steps_per_repetition",
    "timed_steps",
    "duration_s",
    "mean_step_ms",
    "median_step_ms",
    "p95_step_ms",
    "max_step_ms",
    "mean_solver_iterations",
    "p95_solver_iterations",
    "mean_contacts",
    "contact_step_fraction",
    "airborne_step_fraction",
    "max_z_m",
    "max_height_above_initial_m",
    "final_x_mean_m",
    "final_z_mean_m",
    "final_vx_mean_mps",
    "final_wy_mean_rad_s",
    "final_down_slope_mean_m",
    "analytic_down_slope_reference_m",
    "final_down_slope_abs_error_m",
    "backspin_analytic_vx_reference_mps",
    "backspin_final_vx_abs_error_mps",
    "backspin_analytic_wy_reference_rad_s",
    "backspin_final_wy_abs_error_rad_s",
    "max_radial_distance_m",
    "final_radial_distance_mean_m",
    "physical_outcome",
    "paper_fbf_ms_reference",
    "paper_mujoco_relative_reference",
    "paper_implied_mujoco_ms_reference",
    "paper_comparable",
)


def load_baseline_module():
    spec = importlib.util.spec_from_file_location(
        "fbf_paper_mujoco_baseline", BASELINE_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load baseline module from {BASELINE_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def percentile(values, quantile):
    if not values:
        return float("nan")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def parse_cpu_list(value):
    cpus = set()
    for part in value.split(","):
        part = part.strip()
        if not part:
            raise ValueError("empty CPU-list component")
        if "-" in part:
            first_text, last_text = part.split("-", 1)
            first = int(first_text)
            last = int(last_text)
            if first > last:
                raise ValueError(f"descending CPU range: {part}")
            cpus.update(range(first, last + 1))
        else:
            cpus.add(int(part))
    if not cpus or min(cpus) < 0:
        raise ValueError("CPU list must contain non-negative CPU IDs")
    return sorted(cpus)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_text(path):
    try:
        return pathlib.Path(path).read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _command_output(args):
    try:
        return subprocess.run(
            args,
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def git_metadata():
    status = _command_output(["git", "status", "--porcelain=v1"])
    return {
        "head": _command_output(["git", "rev-parse", "HEAD"]),
        "branch": _command_output(["git", "branch", "--show-current"]),
        "dirty": bool(status),
        "status_porcelain": status.splitlines() if status else [],
    }


def cpu_metadata(affinity):
    cpu_model = None
    cpuinfo = _read_text("/proc/cpuinfo")
    if cpuinfo:
        for line in cpuinfo.splitlines():
            if line.startswith("model name"):
                cpu_model = line.split(":", 1)[1].strip()
                break
    per_cpu = {}
    for cpu in affinity:
        base = pathlib.Path(f"/sys/devices/system/cpu/cpu{cpu}/cpufreq")
        per_cpu[str(cpu)] = {
            "governor": _read_text(base / "scaling_governor"),
            "current_khz": _read_text(base / "scaling_cur_freq"),
            "min_khz": _read_text(base / "cpuinfo_min_freq"),
            "max_khz": _read_text(base / "cpuinfo_max_freq"),
        }
    return {
        "model": cpu_model,
        "logical_cpu_count": os.cpu_count(),
        "process_affinity": affinity,
        "per_affinity_cpu": per_cpu,
    }


def _finite_mean(values):
    finite = [value for value in values if math.isfinite(value)]
    return statistics.fmean(finite) if finite else float("nan")


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def classify_outcome(scenario, run_rows):
    final = run_rows[-1]
    max_z = max(row["z_m"] for row in run_rows)
    if scenario == "backspin":
        initial_z = run_rows[0]["z_m"]
        if max_z > initial_z + 0.25:
            return "ejected_above_plane"
        if any(row["airborne"] for row in run_rows[1:]):
            return "lost_contact"
        return "sustained_contact"
    if scenario.startswith("incline_"):
        displacement = final["down_slope_displacement_m"]
        if any(row["airborne"] for row in run_rows[1:]):
            return "slid_with_contact_loss"
        if displacement > 0.01:
            return "slid_or_crept_down_slope"
        return "approximately_stuck"
    if scenario.startswith("turntable_"):
        max_radius = max(row["radial_distance_m"] for row in run_rows)
        if max_radius > 2.125 or final["airborne"]:
            return "ejected_or_off_support"
        return "retained_on_support"
    return "unclassified"


def summarize_scenario(scenario, rows):
    by_run = {}
    for row in rows:
        by_run.setdefault(row["run_id"], []).append(row)
    for run_rows in by_run.values():
        run_rows.sort(key=lambda row: row["step"])

    timed = [row for row in rows if row["timed_step"]]
    finals = [run_rows[-1] for run_rows in by_run.values()]
    step_ms = [row["step_wall_ms"] for row in timed]
    solver_iterations = [row["solver_iterations_sum"] for row in timed]
    outcomes = sorted({classify_outcome(scenario, value) for value in by_run.values()})
    paper = PAPER_TABLE5[scenario]
    first_run = next(iter(by_run.values()))
    analytic_down_slope = float("nan")
    if scenario.startswith("incline_"):
        mu = 0.5 if scenario.endswith("0_5") else 0.4
        theta = math.atan(0.5)
        acceleration = 9.81 * (math.sin(theta) - mu * math.cos(theta))
        analytic_down_slope = (
            0.5 * max(acceleration, 0.0) * (first_run[-1]["sim_time_s"] ** 2)
        )
    final_down_slope = _finite_mean(
        [row["down_slope_displacement_m"] for row in finals]
    )
    max_height_above_initial = max(
        max(row["z_m"] for row in run_rows) - run_rows[0]["z_m"]
        for run_rows in by_run.values()
    )
    backspin_vx_reference = (
        -11.428571428571429 if scenario == "backspin" else float("nan")
    )
    backspin_wy_reference = (
        -45.714285714285715 if scenario == "backspin" else float("nan")
    )
    final_vx = statistics.fmean(row["vx_mps"] for row in finals)
    final_wy = statistics.fmean(row["wy_rad_s"] for row in finals)
    return {
        "scenario": scenario,
        "repetitions": len(by_run),
        "steps_per_repetition": len(first_run) - 1,
        "timed_steps": len(timed),
        "duration_s": first_run[-1]["sim_time_s"],
        "mean_step_ms": statistics.fmean(step_ms),
        "median_step_ms": statistics.median(step_ms),
        "p95_step_ms": percentile(step_ms, 0.95),
        "max_step_ms": max(step_ms),
        "mean_solver_iterations": statistics.fmean(solver_iterations),
        "p95_solver_iterations": percentile(solver_iterations, 0.95),
        "mean_contacts": statistics.fmean(row["contacts"] for row in timed),
        "contact_step_fraction": statistics.fmean(
            int(row["contacts"] > 0) for row in timed
        ),
        "airborne_step_fraction": statistics.fmean(row["airborne"] for row in timed),
        "max_z_m": max(row["z_m"] for row in rows),
        "max_height_above_initial_m": max_height_above_initial,
        "final_x_mean_m": statistics.fmean(row["x_m"] for row in finals),
        "final_z_mean_m": statistics.fmean(row["z_m"] for row in finals),
        "final_vx_mean_mps": final_vx,
        "final_wy_mean_rad_s": final_wy,
        "final_down_slope_mean_m": final_down_slope,
        "analytic_down_slope_reference_m": analytic_down_slope,
        "final_down_slope_abs_error_m": (
            abs(final_down_slope - analytic_down_slope)
            if math.isfinite(analytic_down_slope)
            else float("nan")
        ),
        "backspin_analytic_vx_reference_mps": backspin_vx_reference,
        "backspin_final_vx_abs_error_mps": (
            abs(final_vx - backspin_vx_reference)
            if math.isfinite(backspin_vx_reference)
            else float("nan")
        ),
        "backspin_analytic_wy_reference_rad_s": backspin_wy_reference,
        "backspin_final_wy_abs_error_rad_s": (
            abs(final_wy - backspin_wy_reference)
            if math.isfinite(backspin_wy_reference)
            else float("nan")
        ),
        "max_radial_distance_m": max(
            (
                row["radial_distance_m"]
                for row in rows
                if math.isfinite(row["radial_distance_m"])
            ),
            default=float("nan"),
        ),
        "final_radial_distance_mean_m": _finite_mean(
            [row["radial_distance_m"] for row in finals]
        ),
        "physical_outcome": (
            outcomes[0] if len(outcomes) == 1 else "mixed:" + "+".join(outcomes)
        ),
        "paper_fbf_ms_reference": paper["fbf_ms"],
        "paper_mujoco_relative_reference": paper["mujoco_relative"],
        "paper_implied_mujoco_ms_reference": paper["fbf_ms"] * paper["mujoco_relative"],
        "paper_comparable": False,
    }


def validate_results(mujoco, baseline, scenarios, repetitions, results):
    errors = []
    expected_runs = repetitions * len(scenarios)
    if len(results) != expected_runs:
        errors.append(f"expected {expected_runs} runs, observed {len(results)}")

    expected_solver = int(mujoco.mjtSolver.mjSOL_NEWTON)
    expected_cone = int(mujoco.mjtCone.mjCONE_ELLIPTIC)
    expected_iterations = baseline.PAPER_MUJOCO_MAX_ITERATIONS
    for result in results:
        scenario = result["scenario"]
        options = result["model_options"]
        if options["solver_enum"] != expected_solver:
            errors.append(f"{scenario}: solver is not Newton")
        if options["cone_enum"] != expected_cone:
            errors.append(f"{scenario}: cone is not elliptic")
        if options["iterations"] != expected_iterations:
            errors.append(f"{scenario}: iterations is not {expected_iterations}")
        if options["numeric_dtype"] != "float64":
            errors.append(
                f"{scenario}: unexpected numeric dtype {options['numeric_dtype']}"
            )
        rows = result["rows"]
        expected_steps = result["steps"]
        if [row["step"] for row in rows] != list(range(expected_steps + 1)):
            errors.append(f"{scenario}: non-contiguous step sequence")
        if len(rows) != expected_steps + 1:
            errors.append(f"{scenario}: incomplete trajectory")
        if result["warnings"]:
            errors.append(f"{scenario}: MuJoCo warnings {result['warnings']}")
        for row in rows:
            required = (
                "sim_time_s",
                "x_m",
                "y_m",
                "z_m",
                "vx_mps",
                "vy_mps",
                "vz_mps",
                "quat_w",
                "quat_x",
                "quat_y",
                "quat_z",
            )
            if not all(math.isfinite(row[name]) for name in required):
                errors.append(f"{scenario}: non-finite state at step {row['step']}")
                break
            if row["step"] > 0 and row["step_wall_ns"] <= 0:
                errors.append(f"{scenario}: non-positive timed step at {row['step']}")
                break
            if row["solver_iterations_max_island"] > expected_iterations:
                errors.append(f"{scenario}: solver exceeded iteration cap")
                break
    return errors


def write_csv(path, fieldnames, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
        writer.writeheader()
        writer.writerows(rows)


def write_json(path, value):
    path.write_text(
        json.dumps(json_safe(value), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def report_text(metadata, summaries):
    lines = [
        "# MuJoCo reconstructed-fixture CPU evidence",
        "",
        "**Verdict: this bundle is not paper-parity or apples-to-apples evidence.**",
        "It is a reproducible same-machine run of the local standalone MuJoCo",
        "reconstructions, with the paper's published Newton/elliptic/500-iteration",
        "settings and explicit labels for everything that remains different or unknown.",
        "",
        "## Run contract",
        "",
        f"- MuJoCo: {metadata['software']['mujoco_version']} ({metadata['software']['numeric_precision']})",
        f"- CPU: {metadata['host']['cpu']['model']}",
        f"- Process affinity: {metadata['host']['cpu']['process_affinity']}",
        f"- Repetitions: {metadata['invocation']['repetitions']} complete trajectories, sequential, no warmup exclusion",
        "- Timed interval: synchronous `mujoco.mj_step` only; model compilation, state extraction, and control assignment excluded",
        "- Solver: Newton; cone: elliptic; maximum iterations: 500; tolerance: local native default recorded per model",
        "",
        "## Aggregates",
        "",
        "| Scenario | Mean ms/step | Median | p95 | Contacts/step | Outcome |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for row in summaries:
        lines.append(
            f"| `{row['scenario']}` | {row['mean_step_ms']:.6f} | "
            f"{row['median_step_ms']:.6f} | {row['p95_step_ms']:.6f} | "
            f"{row['mean_contacts']:.3f} | {row['physical_outcome']} |"
        )
    lines.extend(
        [
            "",
            "The paper reference columns are retained in `summary.csv` only as transcription",
            "anchors. No local/paper speed ratio is computed because the precision, CPU,",
            "backend, assets, and several scene/contact parameters do not match.",
            "",
            "## Non-comparability reasons",
            "",
        ]
    )
    for reason in metadata["paper_contract"]["non_comparability_reasons"]:
        lines.append(f"- {reason}")
    lines.extend(
        [
            "",
            "## Files",
            "",
            "- `raw.csv`: every initial state and completed step, including wall time, full tracked-body pose/velocity, contacts, and scenario metrics.",
            "- `summary.csv` / `summary.json`: trajectory timing and physical aggregates.",
            "- `metadata.json`: host, affinity, software, exact model options, scene contracts, validation, provenance, and SHA-256 checksums.",
            "",
        ]
    )
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True, type=pathlib.Path)
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument(
        "--scenarios",
        nargs="+",
        default=None,
        help="Subset of the seven small fixtures (default: all).",
    )
    parser.add_argument(
        "--cpu-list",
        default=None,
        help="Optional Linux process affinity, for example 4 or 4,6 or 4-7.",
    )
    parser.add_argument("--note", default=None)
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite this runner's five named artifacts, preserving unrelated files.",
    )
    args = parser.parse_args(argv)
    if args.repetitions <= 0:
        parser.error("--repetitions must be positive")
    return args


def prepare_output(output_dir, force):
    """Reserve a clean runner-owned artifact set without touching other files."""
    output_dir.mkdir(parents=True, exist_ok=True)
    existing = [
        output_dir / name for name in OWNED_ARTIFACTS if (output_dir / name).exists()
    ]
    if existing and not force:
        paths = ", ".join(str(path) for path in existing)
        raise SystemExit(
            f"refusing to overwrite existing evidence files: {paths}; pass --force"
        )
    if force:
        # Invalidate the whole prior bundle before simulation.  If this run is
        # interrupted, no stale metadata can authenticate old sibling files.
        for path in existing:
            path.unlink()


def main(argv=None):
    args = parse_args(argv)
    baseline = load_baseline_module()
    scenarios = args.scenarios or list(baseline.SMALL_FIXTURE_SCENARIOS)
    invalid = sorted(set(scenarios) - set(baseline.SMALL_FIXTURE_SCENARIOS))
    if invalid:
        raise SystemExit(
            "unsupported evidence scenario(s): "
            + ", ".join(invalid)
            + "; the Rigid-IPC arch remains opt-in in the baseline harness and is not a paper-contract fixture"
        )
    if len(scenarios) != len(set(scenarios)):
        raise SystemExit("--scenarios must not contain duplicates")

    output_dir = args.output_dir.resolve()
    prepare_output(output_dir, args.force)

    original_thread_env = {name: os.environ.get(name) for name in THREAD_ENV_VARS}
    for name in THREAD_ENV_VARS:
        os.environ[name] = "1"

    original_affinity = (
        sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else []
    )
    requested_affinity = None
    if args.cpu_list:
        try:
            requested_affinity = parse_cpu_list(args.cpu_list)
            os.sched_setaffinity(0, requested_affinity)
        except (AttributeError, OSError, ValueError) as error:
            raise SystemExit(
                f"cannot apply --cpu-list {args.cpu_list!r}: {error}"
            ) from error
    effective_affinity = (
        sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else []
    )

    import mujoco  # noqa: PLC0415 (optional environment, after thread limits)

    started = dt.datetime.now(dt.timezone.utc)
    load_start = list(os.getloadavg()) if hasattr(os, "getloadavg") else None
    git = git_metadata()
    cpu_start = cpu_metadata(effective_affinity)
    results = []
    model_metadata = {}
    for repetition in range(args.repetitions):
        for scenario in scenarios:
            run_id = f"{scenario}-r{repetition + 1:03d}"
            result = baseline.simulate_small_fixture(
                mujoco,
                scenario,
                dt=baseline.PAPER_DT,
                duration=baseline.SCENARIO_DEFAULT_DURATIONS_S[scenario],
                repetition=repetition,
                run_id=run_id,
            )
            results.append(result)
            if scenario not in model_metadata:
                model_metadata[scenario] = {
                    "xml_sha256": hashlib.sha256(result["xml"].encode()).hexdigest(),
                    "options": result["model_options"],
                    "warnings": result["warnings"],
                }

    finished = dt.datetime.now(dt.timezone.utc)
    load_end = list(os.getloadavg()) if hasattr(os, "getloadavg") else None
    cpu_end = cpu_metadata(effective_affinity)
    errors = validate_results(mujoco, baseline, scenarios, args.repetitions, results)

    all_rows = [row for result in results for row in result["rows"]]
    summaries = []
    for scenario in scenarios:
        summaries.append(
            summarize_scenario(
                scenario, [row for row in all_rows if row["scenario"] == scenario]
            )
        )

    raw_path = output_dir / "raw.csv"
    summary_csv_path = output_dir / "summary.csv"
    summary_json_path = output_dir / "summary.json"
    report_path = output_dir / "REPORT.md"
    metadata_path = output_dir / "metadata.json"
    write_csv(raw_path, RAW_FIELDS, all_rows)
    write_csv(summary_csv_path, SUMMARY_FIELDS, summaries)
    write_json(
        summary_json_path,
        {
            "schema_version": SCHEMA_VERSION,
            "paper_comparable": False,
            "rows": summaries,
        },
    )

    package_versions = {}
    for package in ("mujoco", "numpy"):
        try:
            package_versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            package_versions[package] = None

    metadata = {
        "schema_version": SCHEMA_VERSION,
        "created_at_utc": finished.isoformat(),
        "elapsed_wall_s": (finished - started).total_seconds(),
        "invocation": {
            "argv": sys.argv,
            "cwd": os.getcwd(),
            "output_dir": str(output_dir),
            "scenarios": scenarios,
            "repetitions": args.repetitions,
            "warmup_repetitions": 0,
            "execution_order": "repetition-major, scenarios sequential",
            "timing_scope": "synchronous mujoco.mj_step only",
            "model_compilation_timed": False,
            "state_extraction_timed": False,
            "turntable_control_assignment_timed": False,
            "note": args.note,
        },
        "repository": git,
        "host": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": sys.version,
            "load_average_start": load_start,
            "load_average_end": load_end,
            "cpu": cpu_start,
            "cpu_at_end": cpu_end,
            "original_process_affinity": original_affinity,
            "requested_process_affinity": requested_affinity,
        },
        "software": {
            "mujoco_version": getattr(mujoco, "__version__", None),
            "mujoco_runtime_version": mujoco.mj_versionString(),
            "mujoco_module": str(pathlib.Path(mujoco.__file__).resolve()),
            "numeric_precision": "float64",
            "package_versions": package_versions,
            "thread_environment_original": original_thread_env,
            "thread_environment_effective": {
                name: os.environ.get(name) for name in THREAD_ENV_VARS
            },
        },
        "paper_contract": {
            "paper_title": "A Splitting Architecture for Exact Reduced Coulomb Friction",
            "paper_url": "https://www.cs.ubc.ca/research/fbf-friction/paper.pdf",
            "paper_location": "Appendix B, Table 5 and accompanying performance contract",
            "published_solver": "MuJoCo Newton",
            "published_cone": "elliptic",
            "published_max_iterations": 500,
            "published_tolerance": "native tolerance; numeric value not stated",
            "published_timestep_s": 1.0 / 60.0,
            "published_cpu_execution": "single precision, sequential, one Apple-silicon Mac",
            "paper_comparable": False,
            "non_comparability_reasons": [
                "Local official MuJoCo pypi uses float64; the paper reports single-precision CPU runs.",
                "Local CPU is not the paper's Apple-silicon Mac.",
                "The author code and scene assets are unavailable, so adapter/backend equivalence cannot be established; this runner uses standalone MuJoCo pypi.",
                "The paper does not publish the numeric native tolerance, MuJoCo contact solref/solimp, or all integrator settings.",
                "Initial penetrations are DART reconstruction choices, not published paper parameters.",
                "Turntable dimensions, mass, radius, exact ramp, drive implementation, and duration are unpublished and reconstructed.",
                "Only mj_step is timed; the exact timing boundary used by the paper implementation is unavailable.",
            ],
        },
        "scene_contracts": {
            scenario: SCENE_CONTRACTS[scenario] for scenario in scenarios
        },
        "compiled_models": model_metadata,
        "validation": {
            "valid": not errors,
            "errors": errors,
            "raw_row_count": len(all_rows),
            "timed_step_count": sum(row["timed_step"] for row in all_rows),
            "complete_small_fixture_matrix": set(scenarios)
            == set(baseline.SMALL_FIXTURE_SCENARIOS),
        },
        "source_files": {
            str(BASELINE_PATH.relative_to(REPO_ROOT)): sha256_file(BASELINE_PATH),
            str(pathlib.Path(__file__).resolve().relative_to(REPO_ROOT)): sha256_file(
                pathlib.Path(__file__).resolve()
            ),
            "pixi.lock": sha256_file(REPO_ROOT / "pixi.lock"),
        },
    }
    report_path.write_text(report_text(metadata, summaries), encoding="utf-8")
    metadata["artifacts"] = {
        path.name: {"size_bytes": path.stat().st_size, "sha256": sha256_file(path)}
        for path in (raw_path, summary_csv_path, summary_json_path, report_path)
    }
    write_json(metadata_path, metadata)

    print(f"wrote {output_dir}")
    print(
        f"validation={'PASS' if not errors else 'FAIL'}; timed_steps={metadata['validation']['timed_step_count']}"
    )
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
