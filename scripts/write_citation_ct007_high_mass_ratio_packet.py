#!/usr/bin/env python3
"""Write the CT-007 high-mass-ratio conditioning packet (PLAN-123 WS2/WS5).

Bounded claim (corpus row CT-007): exact Coulomb cones and adaptive proximal
methods may improve conditioning and remove friction-pyramid anisotropy.

That claim is comparative, and DART `main` has no exact-cone contact solver,
so one arm of the comparison does not exist and the row cannot be
`reproduced` here. What this packet does establish is the other arm: how the
solvers DART actually ships degrade as the mass ratio in a resting stack
grows. That baseline is the input the WS5 exact-cone GO/NO-GO decision needs
-- without it, "an exact cone would help" is untestable.

Bounded reconstruction: a two-box stack at rest, a light box supporting a
heavy one, swept over mass ratio for each rigid contact solver. Per cell it
records:

- steady-state penetration of the loaded contact, relative to box size, which
  is the direct conditioning signal: a well-conditioned solve holds the stack
  apart regardless of the ratio;
- whether the stack survives (finite state, no fall-through, boxes still
  stacked at the horizon);
- settling behavior and residual motion at the horizon;
- deterministic repeats: each cell runs twice and must be bit-identical.

Usage (after `pixi run build`):

    PYTHONPATH=build/default/cpp/Release/python pixi run python \
        scripts/write_citation_ct007_high_mass_ratio_packet.py
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
    UNSUPPORTED_SOLVER_RESIDUAL,
    preserve_review,
    solver_iterations_by_method,
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
    / "CT-007-dart7-high-mass-ratio.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct007_high_mass_ratio_stack",
    "description": (
        "Two-box stack at rest on a static ground: a 1 kg lower box supporting "
        "an upper box whose mass is swept over four decades, so the loaded "
        "contact sees mass ratios from 1:1 to 1000:1. Boxes are 0.2 m cubes, "
        "friction 0.8, restitution 0, simulated to a 2 s horizon per "
        "ratio/solver cell."
    ),
    "gravity_mps2": [0.0, 0.0, -9.81],
    "box_half_extent_m": 0.1,
    "lower_mass_kg": 1.0,
    "mass_ratios": [1.0, 10.0, 100.0, 1000.0],
    "friction": 0.8,
    "restitution": 0.0,
    "ground_half_extents_m": [1.0, 1.0, 0.05],
    "timestep_s": 0.002,
    "horizon_s": 2.0,
    "contact_solver_methods": ["SEQUENTIAL_IMPULSE", "BOXED_LCP"],
    "deterministic_repeats": 2,
}


def scene_digest(parameters: dict[str, Any]) -> str:
    canonical = json.dumps(parameters, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _box_inertia(mass: float, half: float) -> np.ndarray:
    full = 2.0 * half
    moment = mass * (full * full + full * full) / 12.0
    return np.diag([moment, moment, moment])


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def run_single(
    method_name: str, ratio: float, parameters: dict[str, Any]
) -> dict[str, Any]:
    """Run one stack at a given mass ratio and return raw metrics."""
    half = float(parameters["box_half_extent_m"])
    lower_mass = float(parameters["lower_mass_kg"])
    upper_mass = lower_mass * ratio
    dt = float(parameters["timestep_s"])
    ground_half = np.asarray(parameters["ground_half_extents_m"], dtype=float)
    step_count = int(round(float(parameters["horizon_s"]) / dt))

    world = sx.World(
        time_step=dt,
        gravity=np.asarray(parameters["gravity_mps2"], dtype=float),
        contact_solver_method=sx.ContactSolverMethod[method_name],
    )

    ground = world.add_rigid_body("ct007_ground")
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(ground_half))
    ground.transform = _transform_at(np.array([0.0, 0.0, -ground_half[2]]))
    ground.friction = float(parameters["friction"])
    ground.restitution = float(parameters["restitution"])

    boxes = []
    for index, mass in enumerate((lower_mass, upper_mass)):
        body = world.add_rigid_body(f"ct007_box{index}")
        body.mass = mass
        body.inertia = _box_inertia(mass, half)
        body.set_collision_shape(sx.CollisionShape.box(np.array([half, half, half])))
        body.friction = float(parameters["friction"])
        body.restitution = float(parameters["restitution"])
        # Rest each box exactly on the one below, so any observed overlap is
        # solver conditioning rather than an initial interpenetration.
        body.transform = _transform_at(np.array([0.0, 0.0, half + index * 2.0 * half]))
        boxes.append(body)

    world.enter_simulation_mode()

    resolved = {
        "world_resolution": world_resolved_configuration(world),
        "contact_solver_method": world.contact_solver_method.name,
        "gravity_mps2": [float(v) for v in np.asarray(world.gravity)],
        "time_step_s": float(world.time_step),
    }

    lower_rest_z = half
    upper_rest_z = 3.0 * half
    state_hash = hashlib.sha256()
    non_finite = False
    max_iterations = 0
    max_contacts = 0
    max_penetration = 0.0

    for _ in range(step_count):
        world.step()
        metrics = world.compute_step_metrics()
        max_penetration = max(max_penetration, float(metrics.max_penetration_depth))
        max_iterations = max(max_iterations, int(metrics.last_step_iterations))
        max_contacts = max(max_contacts, int(metrics.active_contact_count))
        for body in boxes:
            position = np.asarray(body.translation, dtype=float)
            if not np.all(np.isfinite(position)):
                non_finite = True
                break
        if non_finite:
            break

    if non_finite:
        return {
            "contact_solver_method": method_name,
            "mass_ratio": ratio,
            "resolved": resolved,
            "finite": False,
            "final_state_sha256": None,
            "lower_sink_m": None,
            "upper_sink_m": None,
            "gap_closure_m": None,
            "relative_gap_closure": None,
            "final_max_speed_mps": None,
            "max_penetration_m": max_penetration,
            "max_solver_iterations": max_iterations,
            "max_active_contacts": max_contacts,
        }

    speeds = []
    positions = []
    for body in boxes:
        position = np.asarray(body.translation, dtype=float)
        velocity = np.asarray(body.linear_velocity, dtype=float)
        positions.append(position)
        speeds.append(float(np.linalg.norm(velocity)))
        state_hash.update(position.tobytes())
        state_hash.update(velocity.tobytes())

    lower_sink = lower_rest_z - float(positions[0][2])
    upper_sink = upper_rest_z - float(positions[1][2])
    # How far the two box centers closed toward each other: the loaded
    # contact's steady-state overlap, independent of how far the pair sank
    # into the ground.
    gap_closure = 2.0 * half - float(positions[1][2] - positions[0][2])

    return {
        "contact_solver_method": method_name,
        "mass_ratio": ratio,
        "resolved": resolved,
        "finite": True,
        "final_state_sha256": state_hash.hexdigest(),
        "lower_sink_m": lower_sink,
        "upper_sink_m": upper_sink,
        "gap_closure_m": gap_closure,
        "relative_gap_closure": gap_closure / (2.0 * half),
        "final_max_speed_mps": max(speeds),
        "max_penetration_m": max_penetration,
        "max_solver_iterations": max_iterations,
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
        for ratio in parameters["mass_ratios"]:
            repeats = [
                run_single(method, ratio, parameters)
                for _ in range(int(parameters["deterministic_repeats"]))
            ]
            hashes = {run["final_state_sha256"] for run in repeats}
            if len(hashes) != 1:
                determinism_failures.append(
                    f"{method} ratio {ratio}: final-state hashes differ "
                    f"{sorted(map(str, hashes))}"
                )
            row = repeats[0]
            for repeat in repeats:
                readback = repeat["resolved"]
                if readback != row["resolved"]:
                    raise SystemExit(
                        f"{method} ratio {ratio}: resolved configuration "
                        "differs between repeats"
                    )
                if readback["contact_solver_method"] != method:
                    raise SystemExit(
                        f"requested contact solver {method} but World readback "
                        f"reports {readback['contact_solver_method']}"
                    )
                if readback["time_step_s"] != parameters["timestep_s"]:
                    raise SystemExit(
                        f"requested timestep {parameters['timestep_s']} but "
                        f"World readback reports {readback['time_step_s']}"
                    )
            row["repeat_state_sha256"] = [
                repeat["final_state_sha256"] for repeat in repeats
            ]
            resolved_by_cell[f"{method}@ratio={ratio}"] = row["resolved"]
            rows.append(row)

    if determinism_failures:
        raise SystemExit(
            "deterministic repeats failed:\n  " + "\n  ".join(determinism_failures)
        )

    method_summary: dict[str, Any] = {}
    for method in parameters["contact_solver_methods"]:
        method_rows = [row for row in rows if row["contact_solver_method"] == method]
        finite_rows = [row for row in method_rows if row["finite"]]
        closure_by_ratio = {
            row["mass_ratio"]: row["relative_gap_closure"] for row in finite_rows
        }
        method_summary[method] = {
            "cells": len(method_rows),
            "all_finite": len(finite_rows) == len(method_rows),
            "relative_gap_closure_by_ratio": {
                str(ratio): value for ratio, value in sorted(closure_by_ratio.items())
            },
            "max_relative_gap_closure": (
                max(closure_by_ratio.values()) if closure_by_ratio else None
            ),
            "closure_growth_factor": (
                (
                    closure_by_ratio[max(closure_by_ratio)]
                    / closure_by_ratio[min(closure_by_ratio)]
                )
                if closure_by_ratio and closure_by_ratio.get(min(closure_by_ratio))
                else {
                    "status": "unsupported",
                    "reason": (
                        "Closure at the lowest mass ratio is zero or absent, "
                        "so a growth factor against it is undefined rather "
                        "than zero."
                    ),
                }
            ),
            "max_final_speed_mps": max(
                (row["final_max_speed_mps"] for row in finite_rows),
                default=0.0,
            ),
        }

    all_finite = all(row["finite"] for row in rows)
    # A stack whose boxes close more than a tenth of a box height has not been
    # held apart in any useful sense.
    collapse_threshold = 0.1
    degraded_cells = [
        {
            "contact_solver_method": row["contact_solver_method"],
            "mass_ratio": row["mass_ratio"],
            "relative_gap_closure": row["relative_gap_closure"],
            "finite": row["finite"],
        }
        for row in rows
        if not row["finite"]
        or (row["relative_gap_closure"] or 0.0) > collapse_threshold
    ]

    # Characterize each method's failure onset: the smallest swept ratio at
    # which the stack stops being held apart.
    baseline_finding: dict[str, Any] = {}
    for method in parameters["contact_solver_methods"]:
        failing = sorted(
            row["mass_ratio"]
            for row in rows
            if row["contact_solver_method"] == method
            and (
                not row["finite"]
                or (row["relative_gap_closure"] or 0.0) > collapse_threshold
            )
        )
        baseline_finding[method] = {
            "fails_at_mass_ratios": failing,
            "failure_onset_ratio": (
                failing[0]
                if failing
                else {
                    "status": "unsupported",
                    "reason": (
                        "This method held the stack at every swept ratio, "
                        "so there is no onset within the swept range; the "
                        "onset is unmeasured, not zero or infinite."
                    ),
                }
            ),
            "holds_across_swept_range": not failing,
        }

    # The cited claim compares against exact-cone methods, which this branch
    # does not have. The baseline arm is what this packet establishes, so the
    # row stays unresolved and the measurement is published as WS5 input.
    disposition = "unresolved"

    # The claim boundary quotes this run's failure pattern and closures;
    # assert the pattern and derive the numbers so a regeneration under
    # different behavior cannot keep stale prose.
    si_finding = baseline_finding["SEQUENTIAL_IMPULSE"]
    lcp_finding = baseline_finding["BOXED_LCP"]
    si_closures = {
        float(ratio): value
        for ratio, value in method_summary["SEQUENTIAL_IMPULSE"][
            "relative_gap_closure_by_ratio"
        ].items()
    }
    lcp_closures = {
        float(ratio): value
        for ratio, value in method_summary["BOXED_LCP"][
            "relative_gap_closure_by_ratio"
        ].items()
    }
    si_fails = [float(r) for r in si_finding["fails_at_mass_ratios"]]
    si_holds = sorted(set(si_closures) - set(si_fails))
    if not si_fails or not si_holds or not lcp_finding["holds_across_swept_range"]:
        raise SystemExit(
            "CT-007: the failure pattern changed (sequential impulse "
            f"fails at {si_fails}, boxed-LCP holds="
            f"{lcp_finding['holds_across_swept_range']}); the claim boundary "
            "describes SI holding at low ratios, failing at high ones, and "
            "boxed-LCP holding throughout. Rewrite it from the new findings."
        )
    si_holds_text = " and ".join(f"{r:g}" for r in si_holds)
    si_fails_text = " and ".join(f"{r:g}" for r in si_fails)
    si_hold_closures = " and ".join(f"{si_closures[r]:.1e}" for r in si_holds)
    heavy_descent_m = (
        2.0
        * float(parameters["box_half_extent_m"])
        * max(si_closures[r] for r in si_fails)
    )
    lcp_values = sorted(lcp_closures.values())
    lcp_range_text = f"{lcp_values[0]:.1e} to {lcp_values[-1]:.1e}"

    command = (
        "PYTHONPATH=build/default/cpp/Release/python pixi run python "
        "scripts/write_citation_ct007_high_mass_ratio_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-007",
        "title": (
            "High-mass-ratio stack conditioning baseline for the exact-cone "
            "GO/NO-GO (DART 7)"
        ),
        "source": {
            "url": "https://arxiv.org/abs/2405.17020",
            "claim": (
                "Exact Coulomb cones and adaptive proximal methods may improve "
                "conditioning and remove friction-pyramid anisotropy."
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
                "contact_solver_method_sweep": parameters["contact_solver_methods"],
                "mass_ratio_sweep": parameters["mass_ratios"],
                "exact_cone_solver": (
                    "not available on this branch; the comparison arm the "
                    "cited claim needs does not exist yet"
                ),
                "precision": "float64",
                "backend": "cpu",
                "threads": "World default sequential step",
            },
            "resolved": {"by_cell": resolved_by_cell},
            "resolved_provenance": (
                "The resolved identity is the World's own bake-time "
                "resolution (World.resolved_configuration), recorded per "
                "domain with requested, resolved, reason, and a substitution "
                "flag. Requested contact solver and timestep are additionally "
                "asserted against readback per repeat and per cell."
            ),
            "detector": (
                "DART 7 native World collision pipeline (the World step API "
                "exposes no detector selection on main)"
            ),
            "timestep": parameters["timestep_s"],
            "substeps": 1,
            "iterations": (
                "World defaults; sequential impulse reports its configured "
                "sweep count and the boxed-LCP path records none"
            ),
            "fallback_policy": (
                "World defaults; no per-island fallback reporting is exposed " "on main"
            ),
        },
        "ensemble": {
            "kind": "parameter-sweep-with-deterministic-repeats",
            "sweep": [
                {"contact_solver_method": method, "mass_ratio": ratio}
                for method in parameters["contact_solver_methods"]
                for ratio in parameters["mass_ratios"]
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
                    "Steady-state closure of the loaded contact: how far the "
                    "two box centers approach each other by the horizon, "
                    "relative to a box height. Reported per mass ratio and "
                    "solver, with survival and residual speed."
                ),
                "all_cells_finite": all_finite,
                "per_method_summary": method_summary,
                "collapse_threshold_relative": collapse_threshold,
                "degraded_cells": degraded_cells,
                "baseline_finding": baseline_finding,
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
                "solver_residual": dict(UNSUPPORTED_SOLVER_RESIDUAL),
                "max_active_contacts": max(row["max_active_contacts"] for row in rows),
            },
            "performance": {
                "status": "unsupported",
                "reason": (
                    "This packet makes no timing claim; the matched-residual "
                    "timing a Pareto comparison needs is WS5 work and "
                    "requires the exact-cone arm to exist first"
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
                    "The oracle is the numeric steady-state closure of the "
                    "loaded contact; no visible-behavior claim is made."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "DART 7 main, this commit, a two-box stack with a 1 kg lower "
                "box, mass ratios 1 to 1000, 2 ms timestep, 2 s horizon, "
                "SEQUENTIAL_IMPULSE and BOXED_LCP. The cited claim is "
                "comparative and its exact-cone arm does not exist on this "
                "branch, so the row is NOT reproduced and nothing here says "
                "whether an exact cone would help. What this establishes is "
                "the baseline arm the WS5 GO/NO-GO needs, and it is not a "
                "null result: SEQUENTIAL_IMPULSE -- the World's default "
                f"contact solver -- holds the stack at mass ratios "
                f"{si_holds_text} (relative closure {si_hold_closures}) but "
                f"fails completely at {si_fails_text}, where the heavy box "
                f"descends a full box height ({heavy_descent_m:.5f} m) while "
                "the light box beneath it moves by "
                "microns, coming to rest fully interpenetrated at near-zero "
                f"velocity. BOXED_LCP holds across the swept range (closure "
                f"{lcp_range_text}). Both outcomes are bit-identical across "
                "repeats. Says nothing about deeper stacks, other shapes or "
                "materials, other timesteps, historical DART versions, or "
                "DART 6."
            ),
            "limitations": [
                "The sequential-impulse failure is a geometric "
                "observation at one timestep. This packet does not "
                "establish the mechanism; a fixed iteration budget is "
                "the obvious suspect, and confirming it needs the "
                "per-solve residual WS4 has not exposed yet.",
                "Two boxes only. Conditioning problems are usually worse in "
                "deeper stacks, so this is a floor on the effect, not a "
                "characterization of it.",
                "One timestep. Contact stiffness interacts with the step "
                "size, so a ratio sweep at a single dt cannot separate the "
                "two.",
                "No exact-cone arm exists, so the packet cannot test the "
                "cited improvement; it only records what the current solvers "
                "do.",
                "Closure is measured at the horizon, not tracked as a time "
                "series, so a stack that closes and recovers is not "
                "distinguished from one that never closed.",
                "No solver residual exists on this path and boxed LCP records "
                "no iteration count; both are typed unsupported rather than "
                "reported as zero, so the conditioning signal here is "
                "geometric rather than algebraic.",
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
    for method, stats in physical["per_method_summary"].items():
        closures = stats["relative_gap_closure_by_ratio"]
        rendered = ", ".join(
            f"{ratio}:{value:.3e}" for ratio, value in closures.items()
        )
        print(f"  {method}: relative closure by ratio -> {rendered}")
    print(f"  degraded cells: {physical['degraded_cells']}")
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
