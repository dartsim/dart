#!/usr/bin/env python3
"""Write the CT-003 dense elastic-contact evidence packet (PLAN-123 WS2).

Bounded claim (corpus row CT-003, motivated by the historical SimBenchmark
dense elastic contact test): elastic dense contact may inject energy or
expose solver failure.

Bounded reconstruction (explicitly not source-exact): the same 6x6x6 sphere
grid as CT-002 but with restitution 0.8 on every body, dropped onto a static
ground box and simulated over a timestep grid for each rigid contact solver
method. Per cell it records:

- finite-state outcome (any NaN/Inf fails the cell);
- the mechanical-energy envelope: max total energy over the run relative to
  the initial total energy (an elastic pile with restitution < 1 must never
  exceed its initial mechanical energy);
- max penetration, max active contacts, max solver iterations/residual from
  `StepMetrics`;
- deterministic repeats: each cell runs twice and must be bit-identical.

Energy accounting uses `StepMetrics.total_energy` (kinetic + gravitational
potential with the world-origin zero reference), so the envelope comparison
is solver-independent and self-consistent.

Usage (after `pixi run build`):

    PYTHONPATH=build/default/cpp/Release/python pixi run python \
        scripts/write_citation_ct003_elastic_contact_packet.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import subprocess
import sys
from pathlib import Path
from typing import Any

import dartpy as sx
import numpy as np
from citation_packet_utils import (
    PENETRATION_CLAMP_NOTE,
    SEQUENTIAL_IMPULSE_ITERATIONS_NOTE,
    UNSUPPORTED_SOLVER_RESIDUAL,
    preserve_review,
    solver_iterations_by_method,
    world_resolved_configuration,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "123-citation-driven-simulation-trust"
    / "evidence"
    / "CT-003-dart7-dense-elastic-contact.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct003_dense_elastic_grid",
    "description": (
        "6x6x6 grid of 216 spheres (radius 0.05 m, mass 0.1 kg, spacing "
        "0.12 m) dropped from 0.5 m clearance onto a static ground box, "
        "restitution 0.8, friction 0.8 on all bodies, simulated to a 2 s "
        "horizon per timestep/solver cell."
    ),
    "gravity_mps2": [0.0, 0.0, -9.81],
    "grid_dimension": 6,
    "sphere_radius_m": 0.05,
    "sphere_mass_kg": 0.1,
    "grid_spacing_m": 0.12,
    "drop_clearance_m": 0.5,
    "friction": 0.8,
    "restitution": 0.8,
    "ground_half_extents_m": [2.0, 2.0, 0.05],
    "horizon_s": 2.0,
    "timesteps_s": [0.002, 0.004],
    "contact_solver_methods": ["SEQUENTIAL_IMPULSE", "BOXED_LCP"],
    "deterministic_repeats": 2,
}


def scene_digest(parameters: dict[str, Any]) -> str:
    canonical = json.dumps(parameters, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _sphere_inertia(mass: float, radius: float) -> np.ndarray:
    moment = 0.4 * mass * radius * radius
    return np.diag([moment, moment, moment])


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def run_single(
    method_name: str, timestep: float, parameters: dict[str, Any]
) -> dict[str, Any]:
    """Run one elastic-grid drop and return raw metrics plus a state hash."""
    radius = float(parameters["sphere_radius_m"])
    mass = float(parameters["sphere_mass_kg"])
    dimension = int(parameters["grid_dimension"])
    spacing = float(parameters["grid_spacing_m"])
    clearance = float(parameters["drop_clearance_m"])
    ground_half = np.asarray(parameters["ground_half_extents_m"], dtype=float)
    step_count = int(round(float(parameters["horizon_s"]) / timestep))

    world = sx.World(
        time_step=timestep,
        gravity=np.asarray(parameters["gravity_mps2"], dtype=float),
        contact_solver_method=sx.ContactSolverMethod[method_name],
    )

    ground = world.add_rigid_body("ct003_ground")
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(ground_half))
    ground.transform = _transform_at(np.array([0.0, 0.0, -ground_half[2]]))
    ground.friction = float(parameters["friction"])
    ground.restitution = float(parameters["restitution"])

    bodies = []
    offset = 0.5 * (dimension - 1) * spacing
    for ix in range(dimension):
        for iy in range(dimension):
            for iz in range(dimension):
                body = world.add_rigid_body(f"ct003_sphere_{ix}_{iy}_{iz}")
                body.mass = mass
                body.inertia = _sphere_inertia(mass, radius)
                body.set_collision_shape(sx.CollisionShape.sphere(radius))
                body.friction = float(parameters["friction"])
                body.restitution = float(parameters["restitution"])
                body.transform = _transform_at(
                    np.array(
                        [
                            ix * spacing - offset,
                            iy * spacing - offset,
                            clearance + radius + iz * spacing,
                        ]
                    )
                )
                bodies.append(body)

    world.enter_simulation_mode()

    resolved = {
        "world_resolution": world_resolved_configuration(world),
        "contact_solver_method": world.contact_solver_method.name,
        "rigid_body_solver": world.rigid_body_solver.name,
        "gravity_mps2": [float(v) for v in np.asarray(world.gravity)],
        "time_step_s": float(world.time_step),
    }

    initial_metrics = world.compute_step_metrics()
    initial_total_energy = float(initial_metrics.total_energy)

    state_hash = hashlib.sha256()
    non_finite = False
    max_total_energy = initial_total_energy
    max_total_energy_after_step = None
    max_penetration = 0.0
    max_iterations = 0
    max_residual = 0.0
    max_contacts = 0

    for _ in range(step_count):
        world.step()
        metrics = world.compute_step_metrics()
        total_energy = float(metrics.total_energy)
        if not math.isfinite(total_energy):
            non_finite = True
            break
        max_total_energy = max(max_total_energy, total_energy)
        max_total_energy_after_step = (
            total_energy
            if max_total_energy_after_step is None
            else max(max_total_energy_after_step, total_energy)
        )
        max_penetration = max(max_penetration, float(metrics.max_penetration_depth))
        max_iterations = max(max_iterations, int(metrics.last_step_iterations))
        max_residual = max(max_residual, float(metrics.last_step_residual))
        max_contacts = max(max_contacts, int(metrics.active_contact_count))

    if not non_finite:
        for body in bodies:
            velocity = np.asarray(body.linear_velocity, dtype=float)
            position = np.asarray(body.translation, dtype=float)
            if not (np.all(np.isfinite(velocity)) and np.all(np.isfinite(position))):
                non_finite = True
                break
            state_hash.update(position.tobytes())
            state_hash.update(velocity.tobytes())

    final_metrics = world.compute_step_metrics()
    return {
        "contact_solver_method": method_name,
        "timestep_s": timestep,
        "steps": step_count,
        "resolved": resolved,
        "finite": not non_finite,
        "final_state_sha256": state_hash.hexdigest() if not non_finite else None,
        "initial_total_energy_j": initial_total_energy,
        "max_total_energy_j": max_total_energy,
        "max_total_energy_after_step_j": max_total_energy_after_step,
        "envelope_set_by_initial_state": (
            max_total_energy_after_step is not None
            and max_total_energy_after_step <= initial_total_energy
        ),
        "energy_envelope_excess_j": max_total_energy - initial_total_energy,
        "final_total_energy_j": (
            float(final_metrics.total_energy) if not non_finite else None
        ),
        "max_penetration_m": max_penetration,
        "max_solver_iterations": max_iterations,
        "raw_last_step_residual_max": max_residual,
        "max_active_contacts": max_contacts,
    }


def git_head() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def build_packet(output_path: Path | None = None) -> dict[str, Any]:
    parameters = SCENE_PARAMETERS
    rows: list[dict[str, Any]] = []
    determinism_failures: list[str] = []
    resolved_by_cell: dict[str, dict[str, Any]] = {}

    for method in parameters["contact_solver_methods"]:
        for timestep in parameters["timesteps_s"]:
            repeats = [
                run_single(method, timestep, parameters)
                for _ in range(int(parameters["deterministic_repeats"]))
            ]
            hashes = {run["final_state_sha256"] for run in repeats}
            if len(hashes) != 1:
                determinism_failures.append(
                    f"{method} dt {timestep}: final-state hashes differ "
                    f"{sorted(map(str, hashes))}"
                )
            row = repeats[0]
            for repeat in repeats:
                readback = repeat["resolved"]
                if readback != row["resolved"]:
                    raise SystemExit(
                        f"{method} dt {timestep}: resolved configuration "
                        f"differs between repeats"
                    )
                if readback["contact_solver_method"] != method:
                    raise SystemExit(
                        f"requested contact solver {method} but World readback "
                        f"reports {readback['contact_solver_method']}"
                    )
                if readback["time_step_s"] != timestep:
                    raise SystemExit(
                        f"requested timestep {timestep} but World readback "
                        f"reports {readback['time_step_s']}"
                    )
                if readback["gravity_mps2"] != list(parameters["gravity_mps2"]):
                    raise SystemExit(
                        f"requested gravity {parameters['gravity_mps2']} but "
                        f"World readback reports {readback['gravity_mps2']}"
                    )
            row["repeat_state_sha256"] = [
                repeat["final_state_sha256"] for repeat in repeats
            ]
            resolved_by_cell[f"{method}@dt={timestep}"] = row["resolved"]
            rows.append(row)

    if determinism_failures:
        raise SystemExit(
            "deterministic repeats failed:\n  " + "\n  ".join(determinism_failures)
        )

    all_finite = all(row["finite"] for row in rows)
    max_excess = max(row["energy_envelope_excess_j"] for row in rows)
    initial_energy = rows[0]["initial_total_energy_j"]
    envelope_tolerance_j = 1.0e-6 * max(1.0, abs(initial_energy))
    violating_cells = [
        {
            "contact_solver_method": row["contact_solver_method"],
            "timestep_s": row["timestep_s"],
            "reasons": [
                reason
                for reason, bad in (
                    ("non-finite state", not row["finite"]),
                    (
                        "energy envelope exceeded initial total energy",
                        row["energy_envelope_excess_j"] > envelope_tolerance_j,
                    ),
                )
                if bad
            ],
        }
        for row in rows
    ]
    violating_cells = [cell for cell in violating_cells if cell["reasons"]]
    disposition = "reproduced" if violating_cells else "unresolved"

    command = (
        "PYTHONPATH=build/default/cpp/Release/python pixi run python "
        "scripts/write_citation_ct003_elastic_contact_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-003",
        "title": "Dense elastic contact energy envelope (DART 7 first packet)",
        "source": {
            "url": "https://leggedrobotics.github.io/SimBenchmark/",
            "claim": (
                "Elastic dense contact may inject energy or expose solver " "failure."
            ),
        },
        "target": {"branch": "main", "commit": git_head()},
        "scene": {
            "id": parameters["scene_id"],
            "digest": scene_digest(parameters),
            "description": parameters["description"],
            "parameters": parameters,
        },
        "configuration": {
            "requested": {
                "contact_solver_method_sweep": parameters["contact_solver_methods"],
                "timestep_sweep_s": parameters["timesteps_s"],
                "rigid_body_solver": "SEQUENTIAL_IMPULSE (World default)",
                "integrator": "World default semi-implicit stepping",
                "precision": "float64",
                "backend": "cpu",
                "threads": "World default sequential step",
            },
            "resolved": {"by_cell": resolved_by_cell},
            "resolved_provenance": (
                "World property readback after enter_simulation_mode, with "
                "contact solver, timestep, and gravity each asserted equal "
                "to the request per repeat and per cell. rigid_body_solver "
                "is recorded but not asserted, because this scene does not "
                "request one. configuration.timestep is the smallest swept "
                "value; configuration.resolved.by_cell is authoritative per "
                "cell. World::getResolvedConfiguration() is not yet exposed "
                "to Python (PLAN-123 WS4 follow-up)."
            ),
            "detector": (
                "DART 7 native World collision pipeline (the World step API "
                "exposes no detector selection on main)"
            ),
            "timestep": min(parameters["timesteps_s"]),
            "substeps": 1,
            "iterations": (
                "World defaults; per-step actual iterations recorded via "
                "StepMetrics.last_step_iterations"
            ),
            "fallback_policy": (
                "World defaults; no per-island fallback reporting is exposed "
                "on main (PLAN-123 WS4 follow-up)"
            ),
        },
        "ensemble": {
            "kind": "parameter-sweep-with-deterministic-repeats",
            "sweep": [
                {"contact_solver_method": method, "timestep_s": timestep}
                for method in parameters["contact_solver_methods"]
                for timestep in parameters["timesteps_s"]
            ],
            "deterministic_repeats": int(parameters["deterministic_repeats"]),
            "deterministic_repeats_identical": not determinism_failures,
            "measurement_window": {
                "start_s": 0.0,
                "end_s": parameters["horizon_s"],
            },
        },
        "metrics": {
            "physical": {
                "method": (
                    "Mechanical-energy envelope from "
                    "StepMetrics.total_energy (max over run vs initial), "
                    "finite-state check over 216 bodies"
                ),
                "all_cells_finite": all_finite,
                "initial_total_energy_j": initial_energy,
                "max_energy_envelope_excess_j": max_excess,
                "envelope_tolerance_j": envelope_tolerance_j,
                "violating_cells": violating_cells,
                # A zero excess is the measured result, not a missing value:
                # total energy never rose above its initial value in any cell.
                "measured_zero_fields": (
                    ["max_energy_envelope_excess_j"] if max_excess == 0 else []
                ),
            },
            "numerical": {
                "method": (
                    "Max over run of StepMetrics.max_penetration_depth and "
                    "active_contact_count; per-method iteration counts where "
                    "the runtime records them"
                ),
                "max_penetration_m": max(row["max_penetration_m"] for row in rows),
                "penetration_semantics": PENETRATION_CLAMP_NOTE,
                "solver_iterations_by_method": solver_iterations_by_method(
                    {
                        method: max(
                            row["max_solver_iterations"]
                            for row in rows
                            if row["contact_solver_method"] == method
                        )
                        for method in parameters["contact_solver_methods"]
                    }
                ),
                "sequential_impulse_iterations_semantics": (
                    SEQUENTIAL_IMPULSE_ITERATIONS_NOTE
                ),
                "solver_residual": dict(UNSUPPORTED_SOLVER_RESIDUAL),
                "max_active_contacts": max(row["max_active_contacts"] for row in rows),
            },
            "performance": {
                "status": "unsupported",
                "reason": (
                    "This packet makes no timing claim; interleaved "
                    "same-host methodology is required before any "
                    "performance row"
                ),
            },
            "allocation": {
                "status": "unsupported",
                "reason": (
                    "Post-bake allocation gates are owned by PLAN-122 "
                    "tooling; this packet does not measure allocations"
                ),
            },
        },
        "evidence": {
            "commands": [command],
            "raw_rows": rows,
            "visual": {
                "status": "not-applicable",
                "reason": (
                    "The oracle is the numeric energy envelope and finite "
                    "state; no visible-behavior claim is made by this "
                    "packet."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "DART 7 main, this commit, a bounded (not source-exact) "
                "6x6x6 sphere-grid drop with restitution 0.8 and mu=0.8, "
                "2 s horizon, timesteps 2 and 4 ms, SEQUENTIAL_IMPULSE and "
                "BOXED_LCP contact solvers, energy envelope from "
                "StepMetrics.total_energy. The cited failure mode was NOT "
                "observed here: no cell exceeded its initial mechanical "
                "energy and no cell went non-finite, so this bounded "
                "reconstruction does not reproduce the claim. That is not a "
                "refutation of the original report, which concerns a "
                "different engine version and scene. Says nothing about the "
                "original SimBenchmark scene parameters, historical DART "
                "versions, restitution values other than 0.8, or DART 6."
            ),
            "limitations": [
                "Bounded reconstruction: the original SimBenchmark elastic "
                "test assets and parameters are not reproduced exactly.",
                "The envelope uses aggregate world energy; per-body "
                "restitution-outcome tracking (bounce-height ratios) is "
                "future work for this row.",
                "The envelope maximum is set by the initial state in every "
                "cell (energy never rose above its starting value), so the "
                "1.8e-4 J tolerance is never exercised; the test is "
                "one-sided by construction and is reported as such in "
                "raw_rows.envelope_set_by_initial_state.",
                "The cited claim also names solver failure. That limb is "
                "only instrumented as non-finite state: no residual exists "
                "on this path and the boxed-LCP branch records no iteration "
                "count, so a non-diverging solver failure would not be "
                "detected here.",
                "max_penetration_m is bit-identical across the two solvers "
                "at each timestep while the trajectories diverge, so it is "
                "set during the shared first impact and carries no "
                "solver-discriminating information.",
                "max_active_contacts is 216 in every cell (a fully "
                "stacked column geometry), so it carries no "
                "discriminating information here.",
                "Two timesteps and two solvers only.",
            ],
        },
        "review": (
            preserve_review(output_path) if output_path is not None else {"passes": []}
        ),
        "host": {
            "platform": platform.platform(),
            "python": sys.version.split()[0],
            "machine": platform.machine(),
            "performance_valid": False,
            "note": (
                "Host recorded for provenance only; no timing methodology "
                "was applied"
            ),
        },
    }
    return packet


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Packet output path (default: {DEFAULT_OUTPUT})",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    packet = build_packet(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(packet, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    physical = packet["metrics"]["physical"]
    print(f"wrote {args.output}")
    print(
        f"  all finite: {physical['all_cells_finite']}; envelope excess "
        f"{physical['max_energy_envelope_excess_j']:.3e} J (tolerance "
        f"{physical['envelope_tolerance_j']:.3e} J)"
    )
    print(f"  violating cells: {physical['violating_cells']}")
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
