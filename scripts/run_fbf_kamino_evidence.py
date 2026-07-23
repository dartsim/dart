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

"""Run reconstructed FBF-paper fixtures with Newton's CUDA Kamino solver.

This is a benchmark/example-only external comparison lane.  It reconstructs
the DART backspin and incline fixtures with the public Newton 1.x builder API,
runs ``newton.solvers.SolverKamino`` on CUDA, and preserves synchronized
per-step timings and physical state in raw CSV.

It is deliberately not an apples-to-apples reproduction of the paper's
Kamino rows.  The author scene files, exact Kamino revision/configuration, and
matched hardware are unavailable.  The generated metadata and report retain
those differences rather than promoting a same-machine reconstructed-fixture
measurement into a paper-parity claim.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

PAPER_DT = 1.0 / 60.0
INCLINE_TAN = 0.5
INCLINE_INITIAL_PENETRATION = 0.01
BACKSPIN_RADIUS = 0.25
BACKSPIN_LINEAR_VELOCITY = 4.0
BACKSPIN_ANGULAR_VELOCITY = -200.0
BACKSPIN_FRICTION = 0.5
BACKSPIN_INITIAL_PENETRATION = 0.005
BACKSPIN_ANALYTIC_VX = -11.428571428571429
BACKSPIN_ANALYTIC_WY = -45.714285714285715

SCENARIO_ORDER = ("backspin", "incline_mu_0_5", "incline_mu_0_4")
DEFAULT_DURATIONS = {
    "backspin": 4.0,
    "incline_mu_0_5": 2.0,
    "incline_mu_0_4": 2.0,
}

RAW_FIELDS = (
    "scenario",
    "solver",
    "device",
    "precision",
    "step",
    "time_s",
    "dt_s",
    "elapsed_ns",
    "elapsed_ms",
    "realtime_factor",
    "contacts",
    "x_m",
    "y_m",
    "z_m",
    "qx",
    "qy",
    "qz",
    "qw",
    "vx_m_s",
    "vy_m_s",
    "vz_m_s",
    "wx_rad_s",
    "wy_rad_s",
    "wz_rad_s",
    "up_z",
    "downhill_displacement_m",
    "downhill_speed_m_s",
    "backspin_slip_m_s",
    "analytic_terminal_displacement_m",
    "analytic_terminal_vx_m_s",
    "analytic_terminal_wy_rad_s",
    "finite_state",
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact_binding(path: Path) -> dict[str, int | str]:
    return {"bytes": path.stat().st_size, "sha256": _sha256(path)}


def scenario_spec(name: str) -> dict:
    """Return the public, serializable contract for one fixture."""
    if name == "backspin":
        return {
            "name": name,
            "body": "backspin_sphere",
            "duration_s": DEFAULT_DURATIONS[name],
            "friction": BACKSPIN_FRICTION,
            "geometry": {"sphere_radius_m": BACKSPIN_RADIUS, "infinite_plane": True},
            "mass_kg": 1.0,
            "initial_penetration_m": BACKSPIN_INITIAL_PENETRATION,
            "initial_linear_velocity_m_s": [BACKSPIN_LINEAR_VELOCITY, 0.0, 0.0],
            "initial_angular_velocity_rad_s": [
                0.0,
                BACKSPIN_ANGULAR_VELOCITY,
                0.0,
            ],
            "analytic_terminal_vx_m_s": BACKSPIN_ANALYTIC_VX,
            "analytic_terminal_wy_rad_s": BACKSPIN_ANALYTIC_WY,
            "dart_source": "tests/benchmark/integration/fbf_paper_trace.cpp",
        }

    if name in ("incline_mu_0_5", "incline_mu_0_4"):
        mu = 0.5 if name.endswith("0_5") else 0.4
        theta = math.atan(INCLINE_TAN)
        acceleration = 9.81 * (math.sin(theta) - mu * math.cos(theta))
        analytic_displacement = max(
            0.0, 0.5 * acceleration * DEFAULT_DURATIONS[name] ** 2
        )
        return {
            "name": name,
            "body": "incline_cube",
            "duration_s": DEFAULT_DURATIONS[name],
            "friction": mu,
            "geometry": {
                "cube_size_m": [1.0, 1.0, 1.0],
                "incline_tan": INCLINE_TAN,
                "incline_angle_rad": theta,
                "infinite_plane": True,
            },
            "mass_kg": 1.0,
            "initial_penetration_m": INCLINE_INITIAL_PENETRATION,
            "analytic_terminal_displacement_m": analytic_displacement,
            "dart_source": "tests/benchmark/integration/fbf_paper_trace.cpp",
        }

    raise ValueError(f"unsupported scenario: {name}")


def linear_percentile(values: list[float], percentile: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def fixture_verdict(scenario: str, final: dict) -> tuple[bool, str]:
    contacts = int(final["contacts"])
    if not final["finite_state"]:
        return False, "non-finite terminal state"
    if contacts <= 0:
        return False, "no terminal contact"

    if scenario == "backspin":
        vx_error = abs(float(final["vx_m_s"]) - BACKSPIN_ANALYTIC_VX)
        wy_error = abs(float(final["wy_rad_s"]) - BACKSPIN_ANALYTIC_WY)
        slip = abs(float(final["backspin_slip_m_s"]))
        height_error = abs(float(final["z_m"]) - BACKSPIN_RADIUS)
        passed = (
            vx_error <= 0.5 and wy_error <= 2.0 and slip <= 0.5 and height_error <= 0.03
        )
        return (
            passed,
            "DART fixture tolerances: |dvx|<=0.5 m/s, |dwy|<=2 rad/s, "
            "|slip|<=0.5 m/s, |dz|<=0.03 m",
        )

    displacement = float(final["downhill_displacement_m"])
    if scenario == "incline_mu_0_5":
        return (
            abs(displacement) <= 0.02,
            "DART fixture tolerance: |displacement|<=0.02 m",
        )

    analytic = float(final["analytic_terminal_displacement_m"])
    passed = abs(displacement - analytic) <= 0.2 and displacement > 0.5
    return (
        passed,
        "DART fixture tolerances: |displacement-analytic|<=0.2 m and displacement>0.5 m",
    )


def summarize_rows(scenario: str, rows: list[dict]) -> dict:
    if not rows:
        raise ValueError(f"cannot summarize empty scenario: {scenario}")
    elapsed_ms = [float(row["elapsed_ms"]) for row in rows]
    final = rows[-1]
    simulated_s = float(final["time_s"])
    full_duration_reached = (
        simulated_s + 0.5 * float(final["dt_s"]) >= DEFAULT_DURATIONS[scenario]
    )
    if full_duration_reached:
        passed, criterion = fixture_verdict(scenario, final)
    else:
        passed = None
        criterion = "not evaluated: trajectory is shorter than the fixture duration"
    elapsed_s = sum(elapsed_ms) / 1000.0
    budget_ms = float(final["dt_s"]) * 1000.0
    return {
        "scenario": scenario,
        "steps": len(rows),
        "simulated_s": simulated_s,
        "timed_wall_s": elapsed_s,
        "mean_step_ms": statistics.fmean(elapsed_ms),
        "median_step_ms": statistics.median(elapsed_ms),
        "p95_step_ms": linear_percentile(elapsed_ms, 0.95),
        "max_step_ms": max(elapsed_ms),
        "trajectory_realtime_factor": (
            simulated_s / elapsed_s if elapsed_s > 0.0 else math.inf
        ),
        "realtime_step_fraction": sum(value <= budget_ms for value in elapsed_ms)
        / len(elapsed_ms),
        "all_steps_within_dt": all(value <= budget_ms for value in elapsed_ms),
        "min_contacts": min(int(row["contacts"]) for row in rows),
        "max_contacts": max(int(row["contacts"]) for row in rows),
        "full_duration_reached": full_duration_reached,
        "final": {
            key: final[key]
            for key in (
                "x_m",
                "y_m",
                "z_m",
                "vx_m_s",
                "vy_m_s",
                "vz_m_s",
                "wx_rad_s",
                "wy_rad_s",
                "wz_rad_s",
                "downhill_displacement_m",
                "downhill_speed_m_s",
                "backspin_slip_m_s",
                "contacts",
            )
        },
        "fixture_check_pass": passed,
        "fixture_check_criterion": criterion,
    }


def _run_text(command: list[str], cwd: Path | None = None) -> str | None:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip()


def _git_metadata(repo_root: Path) -> dict:
    revision = _run_text(["git", "rev-parse", "HEAD"], repo_root)
    status = _run_text(["git", "status", "--porcelain"], repo_root)
    return {
        "revision": revision or "unknown",
        "dirty": bool(status),
        "dirty_entry_count": len(status.splitlines()) if status else 0,
    }


def _nvidia_smi_metadata(device_name: str) -> dict:
    output = _run_text(
        [
            "nvidia-smi",
            "--query-gpu=name,uuid,driver_version,memory.total,power.limit",
            "--format=csv,noheader,nounits",
        ]
    )
    if not output:
        return {"available": False}
    rows = []
    for line in output.splitlines():
        values = [value.strip() for value in line.split(",")]
        if len(values) == 5:
            rows.append(
                {
                    "name": values[0],
                    "uuid": values[1],
                    "driver_version": values[2],
                    "memory_mib": values[3],
                    "power_limit_w": values[4],
                }
            )
    selected = next((row for row in rows if row["name"] == device_name), None)
    return {"available": True, "selected": selected, "all_gpus": rows}


def _mat33_diagonal(wp, value: float):
    return wp.mat33(value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value)


def _make_builder(wp, newton, scenario: str):
    spec = scenario_spec(scenario)
    builder = newton.ModelBuilder(gravity=-9.81)
    builder.rigid_gap = 0.0
    builder.default_shape_cfg.margin = 0.0
    builder.default_shape_cfg.gap = 0.0
    shape_cfg = newton.ModelBuilder.ShapeConfig(
        density=0.0,
        mu=spec["friction"],
        margin=0.0,
        gap=0.0,
    )

    if scenario == "backspin":
        builder.add_shape_plane(
            plane=wp.vec4(0.0, 0.0, 1.0, 0.0),
            body=-1,
            width=0.0,
            length=0.0,
            cfg=shape_cfg,
            label="floor",
        )
        inertia = 0.4 * BACKSPIN_RADIUS * BACKSPIN_RADIUS
        body = builder.add_body(
            xform=wp.transform(
                wp.vec3(0.0, 0.0, BACKSPIN_RADIUS - BACKSPIN_INITIAL_PENETRATION),
                wp.quat_identity(),
            ),
            mass=1.0,
            inertia=_mat33_diagonal(wp, inertia),
            lock_inertia=True,
            label=spec["body"],
        )
        builder.add_shape_sphere(
            body,
            radius=BACKSPIN_RADIUS,
            cfg=shape_cfg,
            label="backspin_sphere_shape",
        )
        return builder, body

    theta = math.atan(INCLINE_TAN)
    normal = (-math.sin(theta), 0.0, math.cos(theta))
    builder.add_shape_plane(
        plane=wp.vec4(normal[0], normal[1], normal[2], 0.0),
        body=-1,
        width=0.0,
        length=0.0,
        cfg=shape_cfg,
        label="incline_plane",
    )
    offset = 0.5 - INCLINE_INITIAL_PENETRATION
    position = wp.vec3(*(component * offset for component in normal))
    orientation = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), -theta)
    body = builder.add_body(
        xform=wp.transform(position, orientation),
        mass=1.0,
        inertia=_mat33_diagonal(wp, 1.0 / 6.0),
        lock_inertia=True,
        label=spec["body"],
    )
    builder.add_shape_box(
        body,
        hx=0.5,
        hy=0.5,
        hz=0.5,
        cfg=shape_cfg,
        label="incline_cube_shape",
    )
    return builder, body


def _make_simulation(
    wp, newton, scenario: str, device, tolerance: float, max_iterations: int
):
    builder, body = _make_builder(wp, newton, scenario)
    model = builder.finalize(device=device)
    state_0 = model.state()
    state_1 = model.state()
    control = model.control()

    if scenario == "backspin":
        joint_qd = state_0.joint_qd.numpy()
        joint_qd[:] = (
            BACKSPIN_LINEAR_VELOCITY,
            0.0,
            0.0,
            0.0,
            BACKSPIN_ANGULAR_VELOCITY,
            0.0,
        )
        state_0.joint_qd.assign(joint_qd)
    newton.eval_fk(model, state_0.joint_q, state_0.joint_qd, state_0)

    config = newton.solvers.SolverKamino.Config.from_model(model)
    config.use_collision_detector = True
    config.use_fk_solver = True
    config.collision_detector.pipeline = "unified"
    config.collision_detector.broadphase = "explicit"
    config.collision_detector.default_gap = 0.0
    config.collision_detector.max_contacts = 16
    config.collision_detector.max_contacts_per_pair = 4
    config.dynamics.preconditioning = True
    config.padmm.primal_tolerance = tolerance
    config.padmm.dual_tolerance = tolerance
    config.padmm.compl_tolerance = tolerance
    config.padmm.max_iterations = max_iterations
    config.padmm.warmstart_mode = "containers"
    config.padmm.contact_warmstart_method = "key_and_position"
    solver = newton.solvers.SolverKamino(model=model, config=config)
    contacts = model.contacts()
    initial_position = state_0.body_q.numpy()[body, :3].copy()
    return {
        "model": model,
        "solver": solver,
        "state_0": state_0,
        "state_1": state_1,
        "control": control,
        "contacts": contacts,
        "body": body,
        "initial_position": initial_position,
        "initial_joint_q": state_0.joint_q.numpy().copy(),
        "initial_joint_qd": state_0.joint_qd.numpy().copy(),
    }


def _step_simulation(wp, sim: dict, device, timestep: float) -> int:
    sim["state_0"].clear_forces()
    wp.synchronize_device(device)
    start = time.perf_counter_ns()
    sim["solver"].step(
        sim["state_0"],
        sim["state_1"],
        sim["control"],
        None,
        timestep,
    )
    wp.synchronize_device(device)
    elapsed_ns = time.perf_counter_ns() - start
    sim["state_0"], sim["state_1"] = sim["state_1"], sim["state_0"]
    return elapsed_ns


def _sample_row(
    wp, sim: dict, scenario: str, device, step: int, timestep: float, elapsed_ns: int
) -> dict:
    sim["solver"].update_contacts(sim["contacts"], sim["state_0"])
    wp.synchronize_device(device)
    pose = sim["state_0"].body_q.numpy()[sim["body"]]
    velocity = sim["state_0"].body_qd.numpy()[sim["body"]]
    contacts = int(sim["contacts"].rigid_contact_count.numpy()[0])
    position = pose[:3]
    quaternion = pose[3:7]
    initial = sim["initial_position"]

    downhill_displacement = math.nan
    downhill_speed = math.nan
    analytic_displacement = math.nan
    analytic_vx = math.nan
    analytic_wy = math.nan
    backspin_slip = math.nan
    if scenario == "backspin":
        backspin_slip = float(velocity[0] - BACKSPIN_RADIUS * velocity[4])
        analytic_vx = BACKSPIN_ANALYTIC_VX
        analytic_wy = BACKSPIN_ANALYTIC_WY
    else:
        theta = math.atan(INCLINE_TAN)
        downhill = (-math.cos(theta), 0.0, -math.sin(theta))
        downhill_displacement = sum(
            float(position[index] - initial[index]) * downhill[index]
            for index in range(3)
        )
        downhill_speed = sum(
            float(velocity[index]) * downhill[index] for index in range(3)
        )
        analytic_displacement = scenario_spec(scenario)[
            "analytic_terminal_displacement_m"
        ]

    values = [*position, *quaternion, *velocity]
    finite_state = all(math.isfinite(float(value)) for value in values)
    qx, qy, _qz, _qw = (float(value) for value in quaternion)
    up_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    elapsed_ms = elapsed_ns / 1.0e6
    return {
        "scenario": scenario,
        "solver": "newton.solvers.SolverKamino",
        "device": str(device),
        "precision": "float32",
        "step": step,
        "time_s": step * timestep,
        "dt_s": timestep,
        "elapsed_ns": elapsed_ns,
        "elapsed_ms": elapsed_ms,
        "realtime_factor": timestep / (elapsed_ns / 1.0e9),
        "contacts": contacts,
        "x_m": float(position[0]),
        "y_m": float(position[1]),
        "z_m": float(position[2]),
        "qx": float(quaternion[0]),
        "qy": float(quaternion[1]),
        "qz": float(quaternion[2]),
        "qw": float(quaternion[3]),
        "vx_m_s": float(velocity[0]),
        "vy_m_s": float(velocity[1]),
        "vz_m_s": float(velocity[2]),
        "wx_rad_s": float(velocity[3]),
        "wy_rad_s": float(velocity[4]),
        "wz_rad_s": float(velocity[5]),
        "up_z": up_z,
        "downhill_displacement_m": downhill_displacement,
        "downhill_speed_m_s": downhill_speed,
        "backspin_slip_m_s": backspin_slip,
        "analytic_terminal_displacement_m": analytic_displacement,
        "analytic_terminal_vx_m_s": analytic_vx,
        "analytic_terminal_wy_rad_s": analytic_wy,
        "finite_state": finite_state,
    }


def _run_scenario(
    wp,
    newton,
    scenario: str,
    device,
    timestep: float,
    steps: int,
    warmup_steps: int,
    tolerance: float,
    max_iterations: int,
) -> list[dict]:
    sim = _make_simulation(wp, newton, scenario, device, tolerance, max_iterations)
    # SolverKamino performs lazy, per-instance setup on its first step. Warm the
    # actual instance from the unchanged initial input state, then reset its
    # internal state exactly as Newton's official Kamino examples do. Building a
    # throwaway solver would leave that per-instance setup in the measured row.
    for _ in range(warmup_steps):
        sim["state_0"].clear_forces()
        sim["solver"].step(
            sim["state_0"],
            sim["state_1"],
            sim["control"],
            None,
            timestep,
        )
        wp.synchronize_device(device)
    if warmup_steps > 0:
        sim["solver"].reset(sim["state_0"])
        # reset() restores Newton's model defaults, whose free-joint velocity
        # is zero. Reapply the fixture's captured generalized state so the
        # timed trajectory still starts from the exact backspin contract.
        sim["state_0"].joint_q.assign(sim["initial_joint_q"])
        sim["state_0"].joint_qd.assign(sim["initial_joint_qd"])
        newton.eval_fk(
            sim["model"],
            sim["state_0"].joint_q,
            sim["state_0"].joint_qd,
            sim["state_0"],
        )
        wp.synchronize_device(device)
    rows = []
    for index in range(steps):
        elapsed_ns = _step_simulation(wp, sim, device, timestep)
        rows.append(
            _sample_row(
                wp,
                sim,
                scenario,
                device,
                index + 1,
                timestep,
                elapsed_ns,
            )
        )
    return rows


def _write_raw(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=RAW_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def _write_summary_csv(path: Path, summaries: list[dict]) -> None:
    fields = (
        "scenario",
        "steps",
        "simulated_s",
        "timed_wall_s",
        "mean_step_ms",
        "median_step_ms",
        "p95_step_ms",
        "max_step_ms",
        "trajectory_realtime_factor",
        "realtime_step_fraction",
        "all_steps_within_dt",
        "min_contacts",
        "max_contacts",
        "fixture_check_pass",
    )
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for summary in summaries:
            writer.writerow({field: summary[field] for field in fields})


def json_safe(value):
    """Replace non-finite floats with JSON null recursively."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def render_report(metadata: dict, summaries: list[dict]) -> str:
    lines = [
        "# Newton/Kamino CUDA reconstructed-fixture evidence",
        "",
        "This run is **not an apples-to-apples paper reproduction**.",
        (
            f"It uses public Newton {metadata['software']['newton_version']} / Warp "
            f"{metadata['software']['warp_version']} on `{metadata['device']['name']}` with "
            "DART-reconstructed fixtures."
        ),
        "It does not use the authors' scene assets, exact Kamino revision/configuration, or hardware.",
        "",
        "Timing encloses one synchronized `SolverKamino.step` call.",
        (
            "It includes internal collision detection, constrained dynamics, PADMM, integration, "
            "GPU work, and Python launch overhead."
        ),
        "It excludes construction, kernel compilation/warmup, state export, and CSV I/O.",
        "",
        "| Scenario | Mean ms | Median ms | P95 ms | Trajectory RTF | Steps <= dt | Contacts | Fixture check |",
        "|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for summary in summaries:
        lines.append(
            "| {scenario} | {mean_step_ms:.3f} | {median_step_ms:.3f} | "
            "{p95_step_ms:.3f} | {trajectory_realtime_factor:.3f} | "
            "{fraction:.1%} | {min_contacts}-{max_contacts} | {verdict} |".format(
                fraction=summary["realtime_step_fraction"],
                verdict=(
                    "not evaluated (truncated)"
                    if summary["fixture_check_pass"] is None
                    else ("pass" if summary["fixture_check_pass"] else "FAIL")
                ),
                **summary,
            )
        )
    lines.extend(
        [
            "",
            "The fixture check reuses the broad physical tolerances in DART's regression tests;",
            "it is not a solver-residual certificate or a paper-parity verdict.",
            "",
            "Files: `raw.csv` contains every measured step; `summary.csv` and `summary.json` contain aggregates.",
            "`metadata.json` records the exact scene, solver, software, device, timing, and comparability contracts.",
            "",
        ]
    )
    return "\n".join(lines)


def _prepare_output(output_dir: Path, force: bool) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    owned = ("metadata.json", "raw.csv", "summary.csv", "summary.json", "REPORT.md")
    existing = [output_dir / name for name in owned if (output_dir / name).exists()]
    if existing and not force:
        paths = ", ".join(str(path) for path in existing)
        raise FileExistsError(
            f"refusing to overwrite existing evidence files: {paths}; pass --force"
        )
    if force:
        # Remove the complete old runner-owned set up front.  A failed or
        # interrupted replacement run must not leave a stale manifest beside
        # partially refreshed measurements.
        for path in existing:
            path.unlink()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--scenario",
        action="append",
        choices=SCENARIO_ORDER,
        help="repeat to select scenarios; defaults to all reconstructed fixtures",
    )
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--dt", type=float, default=PAPER_DT)
    parser.add_argument(
        "--steps",
        type=int,
        help="override each scenario's full-duration step count (useful for smoke runs)",
    )
    parser.add_argument("--warmup-steps", type=int, default=2)
    parser.add_argument("--tolerance", type=float, default=1.0e-6)
    parser.add_argument("--padmm-max-iterations", type=int, default=200)
    parser.add_argument("--force", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not math.isfinite(args.dt) or args.dt <= 0.0:
        raise ValueError("--dt must be finite and positive")
    if args.steps is not None and args.steps <= 0:
        raise ValueError("--steps must be positive")
    if args.warmup_steps < 0:
        raise ValueError("--warmup-steps must be non-negative")
    if not math.isfinite(args.tolerance) or args.tolerance <= 0.0:
        raise ValueError("--tolerance must be finite and positive")
    if args.padmm_max_iterations <= 0:
        raise ValueError("--padmm-max-iterations must be positive")

    try:
        import newton  # noqa: PLC0415 - optional benchmark-only dependency
        import warp as wp  # noqa: PLC0415 - optional benchmark-only dependency
    except Exception as error:  # noqa: BLE001 - preserve optional dependency error
        print(
            "Newton/Warp are unavailable. Run through the optional environment: "
            "pixi run -e fbf-baselines fbf-kamino-evidence -- <args>",
            file=sys.stderr,
        )
        print(f"import error: {error}", file=sys.stderr)
        return 2

    wp.init()
    if not wp.is_cuda_available():
        print("Kamino evidence requires an available CUDA device", file=sys.stderr)
        return 2
    device = wp.get_device(args.device)
    if not device.is_cuda:
        print(f"Kamino evidence requires CUDA; got {device}", file=sys.stderr)
        return 2

    output_dir = args.output_dir.resolve()
    _prepare_output(output_dir, args.force)
    scenarios = args.scenario or list(SCENARIO_ORDER)
    repo_root = Path(__file__).resolve().parents[1]

    all_rows = []
    summaries = []
    for scenario in scenarios:
        steps = args.steps
        if steps is None:
            steps = int(round(DEFAULT_DURATIONS[scenario] / args.dt))
        print(
            f"running {scenario}: {steps} measured steps on {device}",
            file=sys.stderr,
            flush=True,
        )
        rows = _run_scenario(
            wp,
            newton,
            scenario,
            device,
            args.dt,
            steps,
            args.warmup_steps,
            args.tolerance,
            args.padmm_max_iterations,
        )
        all_rows.extend(rows)
        summaries.append(summarize_rows(scenario, rows))

    cuda_driver = wp.get_cuda_driver_version()
    cuda_toolkit = wp.get_cuda_toolkit_version()
    metadata = {
        "schema_version": 1,
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "command": [sys.executable, *sys.argv],
        "repo": _git_metadata(repo_root),
        "host": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "software": {
            "newton_version": getattr(newton, "__version__", "unknown"),
            "newton_path": os.path.realpath(newton.__file__),
            "warp_version": getattr(
                wp, "__version__", getattr(wp.config, "version", "unknown")
            ),
            "warp_path": os.path.realpath(wp.__file__),
        },
        "device": {
            "alias": str(device),
            "name": device.name,
            "uuid": device.uuid,
            "architecture": device.arch,
            "total_memory_bytes": device.total_memory,
            "cuda_driver_api": f"{cuda_driver[0]}.{cuda_driver[1]}",
            "cuda_toolkit": f"{cuda_toolkit[0]}.{cuda_toolkit[1]}",
            "nvidia_smi": _nvidia_smi_metadata(device.name),
        },
        "solver": {
            "class": "newton.solvers.SolverKamino",
            "precision": "float32 (Newton/Warp simulation arrays)",
            "integrator": "euler",
            "collision_detector": "internal unified / explicit broadphase",
            "contact_gap_m": 0.0,
            "padmm_tolerances": {
                "primal": args.tolerance,
                "dual": args.tolerance,
                "complementarity": args.tolerance,
            },
            "padmm_max_iterations": args.padmm_max_iterations,
            "preconditioning": True,
            "warmstart_mode": "containers",
            "contact_warmstart_method": "key_and_position",
        },
        "timing": {
            "dt_s": args.dt,
            "warmup_steps_per_scenario": args.warmup_steps,
            "clock": "time.perf_counter_ns",
            "synchronization": "wp.synchronize_device immediately before and after each step",
            "included": [
                "SolverKamino.step",
                "internal collision detection",
                "constrained dynamics and PADMM",
                "integration",
                "GPU work and Python launch overhead",
            ],
            "excluded": [
                "model and solver construction",
                "kernel compilation and warmup",
                "contact/state export",
                "CSV and JSON I/O",
            ],
        },
        "scenarios": [scenario_spec(name) for name in scenarios],
        "comparability": {
            "apples_to_apples_with_paper": False,
            "same_paper_hardware": False,
            "author_scene_assets_used": False,
            "author_kamino_revision_known": False,
            "author_kamino_configuration_known": False,
            "scope": "same-machine public Newton/Kamino run of DART-reconstructed small fixtures",
            "differences": [
                "public Newton/Warp packages rather than a pinned author environment",
                "DART procedural reconstructions rather than author scene assets",
                "RTX 5000 Ada laptop GPU rather than matched paper hardware",
                "current SolverKamino defaults plus explicitly recorded PADMM settings",
                "synchronized Python call timing rather than the unpublished author harness",
            ],
        },
    }

    _write_raw(output_dir / "raw.csv", all_rows)
    _write_summary_csv(output_dir / "summary.csv", summaries)
    summary_document = {
        "schema_version": 1,
        "comparability": metadata["comparability"],
        "scenarios": summaries,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(
            json_safe(summary_document), allow_nan=False, indent=2, sort_keys=True
        )
        + "\n",
        encoding="utf-8",
    )
    (output_dir / "REPORT.md").write_text(
        render_report(metadata, summaries),
        encoding="utf-8",
    )
    metadata["source_files"] = {
        str(Path(__file__).resolve().relative_to(repo_root)): {
            "sha256": _sha256(Path(__file__).resolve())
        }
    }
    metadata["artifacts"] = {
        name: _artifact_binding(output_dir / name)
        for name in ("raw.csv", "summary.csv", "summary.json", "REPORT.md")
    }
    metadata_path = output_dir / "metadata.json"
    metadata_temporary_path = output_dir / ".metadata.json.tmp"
    metadata_temporary_path.write_text(
        json.dumps(json_safe(metadata), allow_nan=False, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    metadata_temporary_path.replace(metadata_path)

    failed = [
        summary["scenario"]
        for summary in summaries
        if summary["fixture_check_pass"] is False
    ]
    if failed:
        print(f"physical fixture checks failed: {', '.join(failed)}", file=sys.stderr)
        return 1
    print(f"wrote Kamino evidence to {output_dir}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
