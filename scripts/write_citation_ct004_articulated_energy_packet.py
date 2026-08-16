#!/usr/bin/env python3
"""Write the CT-004 articulated energy/momentum evidence packet (PLAN-123 WS2).

Bounded claim (corpus row CT-004): articulated integration accuracy must be
comparable at matched cost, with energy/momentum drift measured against
timestep.

Bounded reconstruction (explicitly not source-exact): a passive planar
pendulum chain released from rest under gravity, with no contact and no
control, swept over a timestep grid for each multibody integration family
(`SEMI_IMPLICIT` and `VARIATIONAL`). A passive chain conserves total
mechanical energy exactly, so the oracle is unambiguous and solver-neutral:

- energy drift relative to the initial total energy, over the run;
- whether that drift shrinks with timestep, and at what observed order;
- angular-momentum behavior about the world origin (gravity exerts a torque
  about the origin, so this is reported as a trajectory, not a conservation
  claim -- the packet does not pretend it is conserved);
- deterministic repeats: each cell runs twice and must be bit-identical.

Cost is recorded as step count only. This packet makes NO timing claim: a
matched-cost comparison needs interleaved same-host methodology, which is not
applied here, so `metrics.performance` is typed unsupported. The corpus row's
"at matched cost" half is therefore explicitly not covered yet.

Usage (after `pixi run build`):

    PYTHONPATH=build/default/cpp/Release/python pixi run python \
        scripts/write_citation_ct004_articulated_energy_packet.py
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
    UNSUPPORTED_SOLVER_RESIDUAL,
    preserve_review,
    target_fetch_hint,
    world_resolved_configuration,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "123-citation-driven-simulation-trust"
    / "evidence"
    / "CT-004-dart7-articulated-energy-momentum.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct004_passive_pendulum_chain",
    "description": (
        "Passive 4-link planar pendulum chain (revolute hinges about y, link "
        "length 0.3 m, mass 1.0 kg) released from rest at alternating joint "
        "angles under gravity, no contact and no control, simulated to a 2 s "
        "horizon per timestep/integration-family cell."
    ),
    "gravity_mps2": [0.0, 0.0, -9.81],
    "link_count": 4,
    "link_length_m": 0.3,
    "link_mass_kg": 1.0,
    "link_radius_m": 0.05,
    "initial_joint_angle_rad": 0.35,
    "horizon_s": 2.0,
    "timesteps_s": [0.004, 0.002, 0.001, 0.0005],
    "integration_families": ["SEMI_IMPLICIT", "VARIATIONAL"],
    "deterministic_repeats": 2,
}


def scene_digest(parameters: dict[str, Any]) -> str:
    canonical = json.dumps(parameters, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def run_single(
    family_name: str, timestep: float, parameters: dict[str, Any]
) -> dict[str, Any]:
    """Run one passive-chain release and return raw metrics plus a hash."""
    link_count = int(parameters["link_count"])
    length = float(parameters["link_length_m"])
    mass = float(parameters["link_mass_kg"])
    radius = float(parameters["link_radius_m"])
    angle = float(parameters["initial_joint_angle_rad"])
    step_count = int(round(float(parameters["horizon_s"]) / timestep))

    world = sx.World(
        time_step=timestep,
        gravity=np.asarray(parameters["gravity_mps2"], dtype=float),
        multibody_options=sx.MultibodyOptions(
            integration_family=sx.MultibodyIntegrationFamily[family_name]
        ),
    )

    chain = world.add_multibody("ct004_chain")
    parent = chain.add_link("base")
    ixx = 0.5 * mass * radius * radius
    itrans = mass * length * length / 12.0
    for index in range(link_count):
        offset = 0.0 if index == 0 else length
        link = chain.add_link(
            f"link{index}",
            parent=parent,
            joint=sx.JointSpec(
                name=f"hinge{index}",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 1.0, 0.0),
                transform_from_parent=_translation(offset, 0.0, 0.0),
            ),
        )
        link.mass = mass
        link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
        link.parent_joint.position = [angle * (-1.0 if index % 2 else 1.0)]
        parent = link

    world.enter_simulation_mode()

    resolved = {
        "world_resolution": world_resolved_configuration(world),
        "integration_family": world.multibody_options.integration_family.name,
        "rigid_body_solver": world.rigid_body_solver.name,
        "gravity_mps2": [float(v) for v in np.asarray(world.gravity)],
        "time_step_s": float(world.time_step),
    }

    initial = world.compute_step_metrics()
    initial_total_energy = float(initial.total_energy)
    initial_angular_momentum = np.asarray(initial.angular_momentum, dtype=float)

    trajectory = hashlib.sha256()
    max_abs_energy_drift = 0.0
    max_abs_angular_momentum = float(np.linalg.norm(initial_angular_momentum))
    non_finite = False

    for _ in range(step_count):
        world.step()
        metrics = world.compute_step_metrics()
        total_energy = float(metrics.total_energy)
        angular_momentum = np.asarray(metrics.angular_momentum, dtype=float)
        # Total energy and angular momentum are aggregates and do not
        # uniquely identify a multi-link state, so the repeat hash covers
        # every joint's position and velocity at every step.
        articulated_state = np.asarray(
            [
                float(value)
                for joint in chain.joints
                for value in (
                    *np.atleast_1d(joint.position),
                    *np.atleast_1d(joint.velocity),
                )
            ],
            dtype=float,
        )
        if not (
            math.isfinite(total_energy)
            and np.all(np.isfinite(angular_momentum))
            and np.all(np.isfinite(articulated_state))
        ):
            non_finite = True
            break
        max_abs_energy_drift = max(
            max_abs_energy_drift, abs(total_energy - initial_total_energy)
        )
        max_abs_angular_momentum = max(
            max_abs_angular_momentum, float(np.linalg.norm(angular_momentum))
        )
        trajectory.update(articulated_state.tobytes())
        trajectory.update(np.array([total_energy]).tobytes())
        trajectory.update(angular_momentum.tobytes())

    final = world.compute_step_metrics()
    return {
        "integration_family": family_name,
        "timestep_s": timestep,
        "steps": step_count,
        "resolved": resolved,
        "finite": not non_finite,
        "trajectory_sha256": trajectory.hexdigest(),
        "initial_total_energy_j": initial_total_energy,
        "final_total_energy_j": float(final.total_energy) if not non_finite else None,
        "max_abs_energy_drift_j": max_abs_energy_drift,
        "max_abs_relative_energy_drift": (
            max_abs_energy_drift / abs(initial_total_energy)
            if initial_total_energy != 0.0
            else None
        ),
        "max_abs_angular_momentum": max_abs_angular_momentum,
    }


def git_head() -> str:
    """HEAD commit, refusing to attribute a dirty tree's results to it.

    A packet's target.commit claims the recorded results come from that
    commit's code; uncommitted modifications to tracked files would make
    that attribution false, so generation aborts instead. (The packet
    output file itself is checked before being overwritten, so an
    unmodified existing packet does not block regeneration.)
    """
    dirty = subprocess.run(
        [
            "git",
            "status",
            "--porcelain",
            "--untracked-files=no",
            "--",
            ".",
            ":(exclude)docs/plans/123-citation-driven-simulation-trust/evidence",
            ":(exclude)docs/design/dart6_citation_driven_contact_trust/evidence",
        ],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if dirty:
        raise SystemExit(
            "refusing to generate evidence from a dirty tree; commit or "
            "stash these tracked modifications first:\n" + dirty
        )
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def observed_order(drifts_by_timestep: dict[float, float]) -> Any:
    """Least-squares slope of log(drift) vs log(dt) over the sweep.

    A converging integrator shows a positive order: halving the timestep
    reduces the energy drift. The slope is the honest summary of that; the
    packet does not assert a nominal order the method is "supposed" to have.
    """
    points = [
        (math.log(dt), math.log(drift))
        for dt, drift in sorted(drifts_by_timestep.items())
        if dt > 0.0 and drift > 0.0
    ]
    if len(points) < 2:
        return {
            "status": "unsupported",
            "reason": (
                "Fewer than two cells have a strictly positive energy drift, "
                "so a log-log convergence slope is undefined rather than "
                "zero."
            ),
        }
    mean_x = sum(x for x, _ in points) / len(points)
    mean_y = sum(y for _, y in points) / len(points)
    denominator = sum((x - mean_x) ** 2 for x, _ in points)
    if denominator == 0.0:
        return {
            "status": "unsupported",
            "reason": "All sampled timesteps are equal; slope is undefined.",
        }
    numerator = sum((x - mean_x) * (y - mean_y) for x, y in points)
    return numerator / denominator


def build_packet(output_path: Path | None = None) -> dict[str, Any]:
    parameters = SCENE_PARAMETERS
    rows: list[dict[str, Any]] = []
    determinism_failures: list[str] = []
    resolved_by_cell: dict[str, dict[str, Any]] = {}

    for family in parameters["integration_families"]:
        for timestep in parameters["timesteps_s"]:
            repeats = [
                run_single(family, timestep, parameters)
                for _ in range(int(parameters["deterministic_repeats"]))
            ]
            hashes = {run["trajectory_sha256"] for run in repeats}
            if len(hashes) != 1:
                determinism_failures.append(
                    f"{family} dt {timestep}: trajectory hashes differ "
                    f"{sorted(hashes)}"
                )
            row = repeats[0]
            for repeat in repeats:
                readback = repeat["resolved"]
                if readback != row["resolved"]:
                    raise SystemExit(
                        f"{family} dt {timestep}: resolved configuration "
                        "differs between repeats"
                    )
                if readback["integration_family"] != family:
                    raise SystemExit(
                        f"requested integration family {family} but World "
                        f"readback reports {readback['integration_family']}"
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
            row["repeat_trajectory_sha256"] = [
                repeat["trajectory_sha256"] for repeat in repeats
            ]
            resolved_by_cell[f"{family}@dt={timestep}"] = row["resolved"]
            rows.append(row)

    if determinism_failures:
        raise SystemExit(
            "deterministic repeats failed:\n  " + "\n  ".join(determinism_failures)
        )

    family_summary: dict[str, Any] = {}
    for family in parameters["integration_families"]:
        family_rows = [row for row in rows if row["integration_family"] == family]
        drifts = {
            row["timestep_s"]: row["max_abs_relative_energy_drift"]
            for row in family_rows
            if row["max_abs_relative_energy_drift"] is not None
        }
        smallest_dt = min(drifts) if drifts else None
        largest_dt = max(drifts) if drifts else None
        family_summary[family] = {
            "cells": len(family_rows),
            "relative_energy_drift_by_timestep": {
                str(dt): value for dt, value in sorted(drifts.items())
            },
            "observed_convergence_order": observed_order(drifts),
            "drift_at_smallest_timestep": drifts[smallest_dt],
            "drift_at_largest_timestep": drifts[largest_dt],
            "drift_shrinks_with_timestep": drifts[smallest_dt] < drifts[largest_dt],
            "max_abs_angular_momentum": max(
                row["max_abs_angular_momentum"] for row in family_rows
            ),
        }

    all_finite = all(row["finite"] for row in rows)
    converging = sorted(
        family
        for family, stats in family_summary.items()
        if stats["drift_shrinks_with_timestep"]
    )
    # The corpus defines `reproduced` as "the claimed behavior is observed".
    # The cited claim is a methodological prescription -- accuracy compared at
    # matched cost -- and this packet measures no cost, so none of the claim's
    # content is observed here. Convergence with timestep is a real result the
    # source asserts nothing about, so it cannot promote the row. The row stays
    # `unresolved` until matched-cost timing lands, following the CT-003
    # precedent for "the cited behavior was not observed".
    disposition = "unresolved"
    if not all_finite or set(converging) != set(family_summary):
        # The claim boundary asserts both families stay finite with drift
        # shrinking as the timestep shrinks; abort rather than regenerate
        # that prose against contradicting measurements.
        raise SystemExit(
            "CT-004: finiteness or per-family convergence no longer matches "
            f"the claim boundary (all_finite={all_finite}, "
            f"converging={converging}); rewrite the boundary from the new "
            "findings."
        )
    convergence_finding = {
        "all_cells_finite": all_finite,
        "families_with_shrinking_drift": converging,
        "all_families_converge": (
            all_finite and len(converging) == len(parameters["integration_families"])
        ),
        "note": (
            "This is the packet's positive result. It does not promote the "
            "corpus row, whose claim concerns comparison at matched cost."
        ),
    }

    command = (
        "PYTHONPATH=build/default/cpp/Release/python pixi run python "
        "scripts/write_citation_ct004_articulated_energy_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-004",
        "title": (
            "Articulated energy drift versus timestep across integration "
            "families (DART 7 first packet)"
        ),
        "source": {
            "url": "https://leggedrobotics.github.io/SimBenchmark/",
            "claim": (
                "Articulated integration/contact accuracy must be compared "
                "at matched cost."
            ),
        },
        "target": {
            "branch": "main",
            "commit": (head_commit := git_head()),
            "fetch_hint": target_fetch_hint(head_commit),
            "commit_role": (
                "Source state measured: the library and fixture ran at this "
                "commit, which is HEAD at capture time. The packet and its "
                "writer land in a later commit."
            ),
        },
        "scene": {
            "id": parameters["scene_id"],
            "digest": scene_digest(parameters),
            "description": parameters["description"],
            "parameters": parameters,
        },
        "configuration": {
            "requested": {
                "integration_family_sweep": parameters["integration_families"],
                "timestep_sweep_s": parameters["timesteps_s"],
                "contact": "none (passive chain, no collision shapes)",
                "control": "none (released from rest)",
                "precision": "float64",
                "backend": "cpu",
                "threads": "World default sequential step",
            },
            "resolved": {"by_cell": resolved_by_cell},
            "resolved_provenance": (
                "World property readback after enter_simulation_mode, with "
                "integration family, timestep, and gravity each asserted "
                "equal to the request per repeat and per cell. "
                "rigid_body_solver is recorded but not asserted, because "
                "this scene requests none. configuration.timestep is the "
                "smallest swept value; configuration.resolved.by_cell is "
                "authoritative per cell. World::getResolvedConfiguration() is not yet "
                "exposed to Python (PLAN-123 WS4 follow-up)."
            ),
            "detector": (
                "not applicable: the scene has no collision shapes and runs "
                "no narrow phase"
            ),
            "timestep": min(parameters["timesteps_s"]),
            "substeps": 1,
            "iterations": (
                "World defaults; the variational family uses "
                "MultibodyOptions.variational_max_iterations/tolerance "
                "defaults, which this packet does not vary"
            ),
            "fallback_policy": (
                "World defaults; no per-island fallback reporting is exposed "
                "on main (PLAN-123 WS4 follow-up)"
            ),
        },
        "ensemble": {
            "kind": "parameter-sweep-with-deterministic-repeats",
            "sweep": [
                {"integration_family": family, "timestep_s": timestep}
                for family in parameters["integration_families"]
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
                    "Passive chain: total mechanical energy is conserved "
                    "analytically, so the oracle is max |total_energy - "
                    "initial| over the run from StepMetrics.total_energy, "
                    "normalized by the initial energy, swept over timestep. "
                    "The convergence order is the least-squares slope of "
                    "log(drift) versus log(dt)."
                ),
                "all_cells_finite": all_finite,
                "per_family_summary": family_summary,
                "families_with_shrinking_drift": converging,
                "convergence_finding": convergence_finding,
            },
            "numerical": {
                "method": (
                    "Angular momentum magnitude about the world origin from "
                    "StepMetrics.angular_momentum, reported as an observed "
                    "envelope"
                ),
                "max_abs_angular_momentum_over_cells": max(
                    row["max_abs_angular_momentum"] for row in rows
                ),
                "angular_momentum_semantics": (
                    "Gravity exerts a torque about the world origin, so "
                    "angular momentum is NOT conserved in this scene. The "
                    "envelope is recorded as an observed trajectory bound, "
                    "not as a conservation oracle."
                ),
                "solver_residual": dict(UNSUPPORTED_SOLVER_RESIDUAL),
            },
            "performance": {
                "status": "unsupported",
                "reason": (
                    "The corpus row asks for a matched-cost comparison, but "
                    "no interleaved same-host timing methodology was applied "
                    "here, so no cost is measured and no accuracy-per-cost "
                    "claim is made"
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
            "commands": ["pixi run build", command],
            "raw_rows": rows,
            "visual": {
                "status": "not-applicable",
                "reason": (
                    "The oracle is the numeric energy-drift trend against "
                    "timestep; no visible-behavior claim is made."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "DART 7 main, this commit, a passive 4-link planar pendulum "
                "chain with no contact and no control, 2 s horizon, "
                "timesteps 0.5-4 ms, SEMI_IMPLICIT and VARIATIONAL multibody "
                "integration families. What is established: both families "
                "stay finite and their relative energy drift shrinks as the "
                "timestep shrinks, with the observed log-log slopes recorded "
                "per family. What is NOT established: the 'at matched cost' "
                "half of the corpus row, because no timing was measured; "
                "any contact-accuracy claim, because the scene has no "
                "contact; and any ranking between the two families. Says "
                "nothing about historical DART versions, controlled or "
                "contacting articulated scenes, or DART 6."
            ),
            "limitations": [
                "The relative drift divides by the initial total energy, "
                "which contains a gravitational term measured from the "
                "world-origin datum. Moving the mount changes that "
                "divisor without changing the absolute drift, so the "
                "relative figures are gauge-dependent; the verdict and "
                "the log-log slope are not, because the divisor is "
                "constant within a family. Absolute drift is published "
                "per row as max_abs_energy_drift_j.",
                "No contact and no control: this isolates integration "
                "accuracy and therefore covers only part of the corpus row, "
                "whose contact and matched-cost halves need separate work.",
                "Cost is recorded as step count only; performance is typed "
                "unsupported, so nothing here supports an accuracy-per-cost "
                "comparison.",
                "Angular momentum is not conserved under gravity about the "
                "world origin, so it is reported as an envelope rather than "
                "an invariant.",
                "Variational iteration limits and tolerance are left at "
                "World defaults and not swept; a tighter tolerance would "
                "change the drift figures.",
                "Four timesteps over one decade; the observed slope is a "
                "trend summary, not a certified order of accuracy.",
            ],
        },
        "review": {"passes": []},
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
    if output_path is not None:
        # Rebind after assembly: only passes whose content_digest matches the
        # regenerated packet survive (see preserve_review).
        packet["review"] = preserve_review(output_path, packet)
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
    for family, stats in physical["per_family_summary"].items():
        order = stats["observed_convergence_order"]
        order_text = (
            f"{order:.2f}" if isinstance(order, (int, float)) else "unsupported"
        )
        print(
            f"  {family}: drift {stats['drift_at_largest_timestep']:.3e} -> "
            f"{stats['drift_at_smallest_timestep']:.3e} (relative), observed "
            f"order {order_text}"
        )
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
