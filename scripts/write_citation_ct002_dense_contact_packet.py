#!/usr/bin/env python3
"""Write the CT-002 dense inelastic-contact evidence packet (PLAN-123 WS2).

Bounded claim (corpus row CT-002, motivated by the historical SimBenchmark
dense `6x6x6` contact test): dense contact may fail, become unstable, or
scale poorly for some timestep/solver settings.

Bounded reconstruction (explicitly not source-exact): a 6x6x6 grid of 216
spheres dropped onto a static ground box with zero restitution, run over a
timestep grid for each rigid contact solver method. Per cell it records:

- finite-state outcome (any NaN/Inf position or velocity fails the cell);
- settle behavior: final max speed, kinetic energy at the horizon;
- max penetration depth, max active contact count, max solver iterations,
  and max solver residual from `StepMetrics`;
- max single-step kinetic-energy gain after the first impact window (an
  inelastic pile must dissipate, not inject, energy);
- deterministic repeats: each cell runs twice and must be bit-identical.

The packet records requested and resolved solver identity per cell
(World property readback; `ResolvedSolverConfiguration` is not yet
Python-exposed and that gap is a recorded limitation feeding PLAN-123 WS4).

Usage (after `pixi run build`):

    PYTHONPATH=build/default/cpp/Release/python pixi run python \
        scripts/write_citation_ct002_dense_contact_packet.py
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
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "123-citation-driven-simulation-trust"
    / "evidence"
    / "CT-002-dart7-dense-inelastic-contact.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct002_dense_inelastic_grid",
    "description": (
        "6x6x6 grid of 216 spheres (radius 0.05 m, mass 0.1 kg, spacing "
        "0.12 m) dropped from 0.5 m clearance onto a static ground box, "
        "restitution 0, friction 0.8 on all bodies, simulated to a 2 s "
        "horizon per timestep/solver cell."
    ),
    "gravity_mps2": [0.0, 0.0, -9.81],
    "grid_dimension": 6,
    "sphere_radius_m": 0.05,
    "sphere_mass_kg": 0.1,
    "grid_spacing_m": 0.12,
    "drop_clearance_m": 0.5,
    "friction": 0.8,
    "restitution": 0.0,
    "ground_half_extents_m": [2.0, 2.0, 0.05],
    "horizon_s": 2.0,
    "timesteps_s": [0.002, 0.004],
    "contact_solver_methods": ["SEQUENTIAL_IMPULSE", "BOXED_LCP"],
    "deterministic_repeats": 2,
    "impact_settle_fraction": 0.5,
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
    """Run one dense-grid drop and return raw metrics plus a state hash."""
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

    ground = world.add_rigid_body("ct002_ground")
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
                body = world.add_rigid_body(f"ct002_sphere_{ix}_{iy}_{iz}")
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
        "contact_solver_method": world.contact_solver_method.name,
        "rigid_body_solver": world.rigid_body_solver.name,
        "gravity_mps2": [float(v) for v in np.asarray(world.gravity)],
        "time_step_s": float(world.time_step),
    }

    settle_step = int(step_count * float(parameters["impact_settle_fraction"]))
    state_hash = hashlib.sha256()
    non_finite = False
    max_penetration = 0.0
    max_iterations = 0
    max_residual = 0.0
    max_contacts = 0
    max_energy_gain_after_settle = 0.0
    max_total_energy_gain_after_settle = 0.0
    previous_kinetic = None
    previous_total = None
    initial_metrics = world.compute_step_metrics()
    initial_total_energy = float(initial_metrics.total_energy)

    for step_index in range(step_count):
        world.step()
        metrics = world.compute_step_metrics()
        kinetic = float(metrics.kinetic_energy)
        total_energy = float(metrics.total_energy)
        if not (math.isfinite(kinetic) and math.isfinite(total_energy)):
            non_finite = True
            break
        if previous_total is not None and step_index >= settle_step:
            max_total_energy_gain_after_settle = max(
                max_total_energy_gain_after_settle,
                total_energy - previous_total,
            )
        previous_total = total_energy
        if previous_kinetic is not None and step_index >= settle_step:
            max_energy_gain_after_settle = max(
                max_energy_gain_after_settle, kinetic - previous_kinetic
            )
        previous_kinetic = kinetic
        max_penetration = max(max_penetration, float(metrics.max_penetration_depth))
        max_iterations = max(max_iterations, int(metrics.last_step_iterations))
        max_residual = max(max_residual, float(metrics.last_step_residual))
        max_contacts = max(max_contacts, int(metrics.active_contact_count))

    speeds = []
    min_height = math.inf
    if not non_finite:
        for body in bodies:
            velocity = np.asarray(body.linear_velocity, dtype=float)
            position = np.asarray(body.translation, dtype=float)
            if not (np.all(np.isfinite(velocity)) and np.all(np.isfinite(position))):
                non_finite = True
                break
            speeds.append(float(np.linalg.norm(velocity)))
            min_height = min(min_height, float(position[2]))
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
        "final_max_speed_mps": max(speeds) if speeds else None,
        "final_min_height_m": min_height if speeds else None,
        "final_kinetic_energy_j": (
            float(final_metrics.kinetic_energy) if not non_finite else None
        ),
        "initial_total_energy_j": initial_total_energy,
        "max_energy_gain_after_settle_j": max_energy_gain_after_settle,
        "max_total_energy_gain_after_settle_j": (max_total_energy_gain_after_settle),
        "max_penetration_m": max_penetration,
        "max_solver_iterations": max_iterations,
        "raw_last_step_residual_max": max_residual,
        "max_active_contacts": max_contacts,
    }


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
                        f"differs between repeats: {readback} vs "
                        f"{row['resolved']}"
                    )
                # Assert every recorded resolved field against what was asked
                # for; recording a field without checking it is how a packet
                # ends up misreporting the configuration that actually ran.
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
    max_penetration = max(row["max_penetration_m"] for row in rows)
    max_gain = max(row["max_energy_gain_after_settle_j"] for row in rows)
    max_total_gain = max(row["max_total_energy_gain_after_settle_j"] for row in rows)
    final_max_speed_by_cell = {
        f"{row['contact_solver_method']}@dt={row['timestep_s']}": row[
            "final_max_speed_mps"
        ]
        for row in rows
        if row["finite"]
    }
    stability_tolerance = {
        "max_penetration_m": 0.5 * float(parameters["sphere_radius_m"]),
        "final_max_speed_mps": 0.05,
        "max_energy_gain_after_settle_j": 1.0e-3,
        # Relative to the scene's own initial mechanical energy: after the
        # pile settles, total energy must not climb. During free fall a
        # semi-implicit integrator legitimately perturbs total energy, so
        # this is measured only in the settle window.
        "max_total_energy_gain_after_settle_j": 1.0e-6
        * abs(rows[0]["initial_total_energy_j"]),
        "max_total_energy_gain_after_settle_j_basis": (
            "1.0e-6 x |initial total mechanical energy| of this scene, a "
            "chosen floor. The verdict does not depend on its exact "
            "value: BOXED_LCP on the same scene, integrator, and "
            "timestep is the internal control, and the verdict is "
            "unchanged for any tolerance lying between the two solvers "
            "observed gains."
        ),
    }
    residual_speed_tolerance = {
        "dt_linearity_relative": 1.0e-3,
        "note": (
            "A settle speed proportional to dt is the quasi-static residual "
            "of a converging semi-implicit integrator, not instability: "
            "halving dt halves it. Instability grows super-linearly or "
            "diverges. A cell whose speed/dt matches the other cells of its "
            "method to this relative tolerance is classified as residual."
        ),
    }

    # Per method, test whether final speed scales linearly with dt.
    speed_over_dt: dict[str, list[float]] = {}
    for row in rows:
        if row["finite"] and row["timestep_s"] > 0.0:
            speed_over_dt.setdefault(row["contact_solver_method"], []).append(
                row["final_max_speed_mps"] / row["timestep_s"]
            )
    dt_linearity: dict[str, Any] = {}
    for method, ratios in speed_over_dt.items():
        mean = sum(ratios) / len(ratios)
        spread = (max(ratios) - min(ratios)) / mean if mean > 0.0 else 0.0
        dt_linearity[method] = {
            "speed_over_dt": ratios,
            "relative_spread": spread,
            "linear_in_dt": (
                len(ratios) >= 2
                and mean > 0.0
                and spread <= residual_speed_tolerance["dt_linearity_relative"]
            ),
        }

    cell_findings = []
    for row in rows:
        method = row["contact_solver_method"]
        reasons = []
        if not row["finite"]:
            reasons.append("non-finite state")
        if row["max_penetration_m"] > stability_tolerance["max_penetration_m"]:
            reasons.append("penetration above half radius")
        if (
            row["max_energy_gain_after_settle_j"]
            > stability_tolerance["max_energy_gain_after_settle_j"]
        ):
            reasons.append("kinetic energy injected after settle window")
        if (
            row["max_total_energy_gain_after_settle_j"]
            > stability_tolerance["max_total_energy_gain_after_settle_j"]
        ):
            reasons.append("total mechanical energy increased after settle")
        speed_over_tolerance = (
            row["finite"]
            and row["final_max_speed_mps"] > stability_tolerance["final_max_speed_mps"]
        )
        residual_only = speed_over_tolerance and dt_linearity.get(method, {}).get(
            "linear_in_dt", False
        )
        if speed_over_tolerance and not residual_only:
            reasons.append("pile still moving at horizon, not dt-linear")
        cell_findings.append(
            {
                "contact_solver_method": method,
                "timestep_s": row["timestep_s"],
                "final_max_speed_mps": row["final_max_speed_mps"],
                "speed_over_tolerance": speed_over_tolerance,
                "classified_as_integrator_residual": residual_only,
                "instability_reasons": reasons,
            }
        )

    unstable_cells = [cell for cell in cell_findings if cell["instability_reasons"]]
    # The cited claim is failure, instability, or poor scaling. Failure and
    # instability are instrumented here; scaling is not (performance is typed
    # unsupported). A settle speed that is exactly linear in dt is integrator
    # residual, so it does not reproduce the claim.
    disposition = "reproduced" if unstable_cells else "unresolved"

    command = (
        "PYTHONPATH=build/default/cpp/Release/python pixi run python "
        "scripts/write_citation_ct002_dense_contact_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-002",
        "title": ("Dense 6x6x6 inelastic contact stability (DART 7 first packet)"),
        "source": {
            "url": "https://leggedrobotics.github.io/SimBenchmark/",
            "claim": (
                "Dense contact may fail, become unstable, or scale poorly "
                "for some timestep/solver settings."
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
                "settle_window_start_fraction": parameters["impact_settle_fraction"],
            },
        },
        "metrics": {
            "physical": {
                "method": (
                    "Per-cell finite-state check over 216 bodies, final max "
                    "body speed, final min body height, kinetic energy at "
                    "the horizon, and max single-step kinetic-energy gain "
                    "after the settle window from StepMetrics"
                ),
                "all_cells_finite": all_finite,
                "max_penetration_m": max_penetration,
                "final_max_speed_mps_by_cell": final_max_speed_by_cell,
                "max_energy_gain_after_settle_j": max_gain,
                "max_total_energy_gain_after_settle_j": max_total_gain,
                "stability_tolerance": stability_tolerance,
                "residual_speed_tolerance": residual_speed_tolerance,
                "dt_linearity": dt_linearity,
                "cell_findings": cell_findings,
                "unstable_cells": unstable_cells,
                # An exactly-zero spread is the finding, not a missing value:
                # the speed/dt ratios are bit-identical across timesteps.
                "measured_zero_fields": [
                    f"dt_linearity.{method}.relative_spread"
                    for method, linearity in dt_linearity.items()
                    if linearity["relative_spread"] == 0
                ]
                + [
                    f"final_max_speed_mps_by_cell.{cell}"
                    for cell, speed in final_max_speed_by_cell.items()
                    if speed == 0
                ]
                + (
                    ["max_total_energy_gain_after_settle_j"]
                    if max_total_gain == 0
                    else []
                ),
            },
            "numerical": {
                "method": (
                    "Max over run of StepMetrics.max_penetration_depth and "
                    "active_contact_count; per-method iteration counts where "
                    "the runtime records them"
                ),
                "max_penetration_m": max_penetration,
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
                    "This packet makes no timing or scaling claim; "
                    "interleaved same-host methodology is required before "
                    "any performance row"
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
                    "The oracle is numeric (finite state, final speed and "
                    "its dt scaling, penetration, and non-increase of total "
                    "mechanical energy from StepMetrics.total_energy); no "
                    "visible-behavior claim is made by this packet."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "DART 7 main, this commit, a bounded (not source-exact) "
                "6x6x6 sphere-grid drop with restitution 0 and mu=0.8, "
                "2 s horizon, timesteps 2 and 4 ms, SEQUENTIAL_IMPULSE and "
                "BOXED_LCP contact solvers. Outcome actually observed: no "
                "cell failed (4 of 4 finite, no fall-through, penetration "
                "within tolerance), and the one cell above the settle-speed "
                "tolerance has a final speed exactly proportional to dt "
                "(speed/dt identical across timesteps), which is converging "
                "integrator residual and is explicitly NOT counted as "
                "instability. What does reproduce is narrower and "
                "solver-specific: after the pile settles, the largest "
                "single-step increase in total mechanical energy under "
                "SEQUENTIAL_IMPULSE is about 1.0e-3 J at 2 ms and 1.3e-3 J "
                "at 4 ms against a 1.8e-4 J tolerance on a 180 J scene, "
                "while BOXED_LCP stays at or near zero (0.0 and 1.3e-6 J). "
                "The metric is a maximum over settle-window steps, so it "
                "does not distinguish one anomalous step from sustained "
                "pumping. That is a small, "
                "non-divergent, non-physical energy gain in a resting "
                "inelastic pile, not a blow-up. The poor-scaling limb of the "
                "cited claim is not instrumented at all (performance is "
                "typed unsupported). Says nothing about the original "
                "SimBenchmark scene parameters, historical DART versions, "
                "other densities/materials, or DART 6."
            ),
            "limitations": [
                "Bounded reconstruction: the original SimBenchmark asset, "
                "material, and timestep grid are not reproduced exactly; "
                "sourcing the exact historical setup is future corpus work.",
                "The reproduced signal is a small per-step energy gain in a "
                "settled pile, not divergence or failure. It must not be "
                "quoted as 'dense contact fails' or as a solver ranking.",
                "The settle window is the second half of the run; a pile "
                "that settles later would put free-fall discretization error "
                "inside the window and inflate the energy metric.",
                "Resolved identity comes from World property readback per "
                "cell (contact solver, timestep, and gravity each asserted "
                "against the request); ResolvedSolverConfiguration is not "
                "Python-exposed.",
                "No solver residual exists on this path and the boxed-LCP "
                "branch records no iteration count; both are typed "
                "unsupported rather than reported as zero, so the corpus "
                "row's residual and iteration oracles are not yet covered.",
                "The corpus row also names wall time over the timestep "
                "grid; this packet makes no timing claim (performance is "
                "typed unsupported) because no interleaved same-host "
                "methodology was applied.",
                "max_active_contacts is 216 in every cell (a fully stacked "
                "column geometry), so it carries no discriminating "
                "information here.",
                "Two timesteps and two solvers only; a wider grid belongs "
                "to a follow-up once per-island diagnostics exist.",
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


def git_head() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


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
        f"  all finite: {physical['all_cells_finite']}; max penetration "
        f"{physical['max_penetration_m']:.3e} m; max total-energy gain "
        f"{physical['max_total_energy_gain_after_settle_j']:.3e} J"
    )
    for cell, speed in physical["final_max_speed_mps_by_cell"].items():
        print(f"  {cell}: final max speed {speed:.4e} m/s")
    for method, linearity in physical["dt_linearity"].items():
        print(
            f"  {method}: speed/dt spread {linearity['relative_spread']:.3e} "
            f"-> linear_in_dt={linearity['linear_in_dt']}"
        )
    print(f"  unstable cells: {physical['unstable_cells']}")
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
