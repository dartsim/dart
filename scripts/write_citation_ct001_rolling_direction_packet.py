#!/usr/bin/env python3
"""Write the CT-001 rolling-direction evidence packet (PLAN-123 WS2).

Bounded claim (corpus row CT-001): polyhedral friction can produce
direction-dependent rolling/sliding behavior. Under an isotropic Coulomb
friction model, a sphere launched sliding (no spin) on a horizontal plane
behaves identically for every launch direction; a friction pyramid aligned to
fixed tangent axes breaks that rotational symmetry.

The fixture launches one sphere per run at speed `v0` along a swept in-plane
angle, without initial spin, on a static ground box, and measures per angle:

- lateral drift from the launch ray and final-velocity heading error at the
  measurement horizon (symmetry-breaking signals);
- slide-to-roll transition time and post-transition speed;
- mechanical-energy trajectory (friction during sliding must dissipate, never
  inject, energy);
- `StepMetrics` numerics (max penetration, iterations, residual) and active
  contact counts.

Each (contact solver, angle) cell runs twice and must be bit-identical
(deterministic repeats). The packet records requested and resolved solver
identity from `World.resolved_configuration` (the bake-time
`ResolvedSolverConfiguration`, bound to Python in this branch) plus World
property readback and step-profile stage names.

Usage (after `pixi run build`):

    PYTHONPATH=build/default/cpp/Release/python pixi run python \
        scripts/write_citation_ct001_rolling_direction_packet.py
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
    UNSUPPORTED_ANTISYMMETRY_RATIO,
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
    / "CT-001-dart7-rolling-direction.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct001_rolling_direction_sweep",
    "description": (
        "One 1 kg sphere per run, radius 0.08 m, launched sliding (no spin) "
        "at 1.0 m/s along a swept in-plane angle on a static ground box, "
        "restitution 0, friction 0.35 on both bodies."
    ),
    "gravity_mps2": [0.0, 0.0, -9.81],
    "time_step_s": 0.002,
    "step_count": 500,
    "sphere_radius_m": 0.08,
    "sphere_mass_kg": 1.0,
    "launch_speed_mps": 1.0,
    "friction": 0.35,
    "restitution": 0.0,
    "ground_half_extents_m": [2.5, 2.5, 0.05],
    "launch_angles_deg": [0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0],
    "contact_solver_methods": ["SEQUENTIAL_IMPULSE", "BOXED_LCP"],
    "deterministic_repeats": 2,
    "slip_ratio_rolling_threshold": 0.02,
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
    method_name: str, angle_deg: float, parameters: dict[str, Any]
) -> dict[str, Any]:
    """Run one launch and return raw metrics plus a trajectory hash."""
    radius = float(parameters["sphere_radius_m"])
    mass = float(parameters["sphere_mass_kg"])
    speed = float(parameters["launch_speed_mps"])
    dt = float(parameters["time_step_s"])
    step_count = int(parameters["step_count"])
    ground_half = np.asarray(parameters["ground_half_extents_m"], dtype=float)
    angle = math.radians(angle_deg)
    direction = np.array([math.cos(angle), math.sin(angle), 0.0])

    world = sx.World(
        time_step=dt,
        gravity=np.asarray(parameters["gravity_mps2"], dtype=float),
        contact_solver_method=sx.ContactSolverMethod[method_name],
    )
    world.step_profiling_enabled = True

    ground = world.add_rigid_body("ct001_ground")
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(ground_half))
    ground.transform = _transform_at(np.array([0.0, 0.0, -ground_half[2]]))
    ground.friction = float(parameters["friction"])
    ground.restitution = float(parameters["restitution"])

    sphere = world.add_rigid_body("ct001_sphere")
    sphere.mass = mass
    sphere.inertia = _sphere_inertia(mass, radius)
    sphere.set_collision_shape(sx.CollisionShape.sphere(radius))
    sphere.friction = float(parameters["friction"])
    sphere.restitution = float(parameters["restitution"])
    start = np.array([0.0, 0.0, radius])
    sphere.transform = _transform_at(start)
    sphere.linear_velocity = speed * direction
    sphere.angular_velocity = (0.0, 0.0, 0.0)

    world.enter_simulation_mode()

    resolved = {
        "world_resolution": world_resolved_configuration(world),
        "contact_solver_method": world.contact_solver_method.name,
        "rigid_body_solver": world.rigid_body_solver.name,
        "gravity_mps2": [float(v) for v in np.asarray(world.gravity)],
        "time_step_s": float(world.time_step),
    }

    trajectory = hashlib.sha256()
    slide_end_time = None
    max_energy_gain = 0.0
    max_penetration = 0.0
    max_iterations = 0
    max_residual = 0.0
    contact_count_max = 0
    previous_energy = None
    stage_names: list[str] = []

    lateral_axis = np.array([-direction[1], direction[0], 0.0])
    for step_index in range(step_count):
        world.step()
        position = np.asarray(sphere.translation, dtype=float)
        velocity = np.asarray(sphere.linear_velocity, dtype=float)
        angular = np.asarray(sphere.angular_velocity, dtype=float)
        trajectory.update(position.tobytes())
        trajectory.update(velocity.tobytes())
        trajectory.update(angular.tobytes())

        metrics = world.compute_step_metrics()
        kinetic = float(metrics.kinetic_energy)
        if previous_energy is not None:
            max_energy_gain = max(max_energy_gain, kinetic - previous_energy)
        previous_energy = kinetic
        max_penetration = max(max_penetration, float(metrics.max_penetration_depth))
        max_iterations = max(max_iterations, int(metrics.last_step_iterations))
        max_residual = max(max_residual, float(metrics.last_step_residual))
        contact_count_max = max(contact_count_max, int(metrics.active_contact_count))

        if slide_end_time is None:
            # Contact-point slip as a planar vector, so a transient sign
            # cancellation in one component cannot be mistaken for rolling.
            # The contact point moves at v + omega x (-R z_hat), whose planar
            # part is (v_x - R*w_y, v_y + R*w_x).
            slip_vector = np.array(
                [
                    velocity[0] - radius * angular[1],
                    velocity[1] + radius * angular[0],
                ]
            )
            slip = float(np.linalg.norm(slip_vector))
            planar_speed = float(np.linalg.norm(velocity[:2]))
            surface_speed = radius * float(np.linalg.norm(angular[:2]))
            denom = planar_speed + surface_speed + 1.0e-9
            rolling = slip / denom < float(parameters["slip_ratio_rolling_threshold"])
            # A sphere at rest has no slip but is not rolling; require motion.
            if rolling and planar_speed > 1.0e-6:
                slide_end_time = (step_index + 1) * dt

        if step_index == step_count - 1:
            profile = world.last_step_profile
            if not profile.is_empty():
                stage_names = [stage.name for stage in profile.stages]

    final_position = np.asarray(sphere.translation, dtype=float)
    final_velocity = np.asarray(sphere.linear_velocity, dtype=float)
    displacement = final_position - start
    along = float(np.dot(displacement, direction))
    lateral = float(np.dot(displacement, lateral_axis))
    planar_speed = float(np.linalg.norm(final_velocity[:2]))
    if planar_speed > 1.0e-9:
        heading_error_deg = math.degrees(
            math.atan2(
                float(np.dot(final_velocity, lateral_axis)),
                float(np.dot(final_velocity, direction)),
            )
        )
    else:
        heading_error_deg = 0.0

    return {
        "angle_deg": angle_deg,
        "contact_solver_method": method_name,
        "resolved": resolved,
        "trajectory_sha256": trajectory.hexdigest(),
        "along_travel_m": along,
        "lateral_drift_m": lateral,
        "heading_error_deg": heading_error_deg,
        "final_planar_speed_mps": planar_speed,
        "slide_end_time_s": slide_end_time,
        "max_energy_gain_j": max_energy_gain,
        "max_penetration_m": max_penetration,
        "max_solver_iterations": max_iterations,
        "raw_last_step_residual_max": max_residual,
        "max_active_contacts": contact_count_max,
        "final_step_stage_names": stage_names,
    }


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Per-solver rotational-symmetry summary across the angle sweep."""
    summary: dict[str, Any] = {}
    for method in SCENE_PARAMETERS["contact_solver_methods"]:
        method_rows = [row for row in rows if row["contact_solver_method"] == method]
        drifts = [abs(row["lateral_drift_m"]) for row in method_rows]
        travels = [row["along_travel_m"] for row in method_rows]
        headings = [abs(row["heading_error_deg"]) for row in method_rows]
        travel_mean = sum(travels) / len(travels)
        travel_spread = max(travels) - min(travels)
        summary[method] = {
            "runs": len(method_rows),
            "max_abs_lateral_drift_m": max(drifts),
            "max_abs_heading_error_deg": max(headings),
            "travel_mean_m": travel_mean,
            "travel_spread_m": travel_spread,
            "travel_spread_relative": (
                travel_spread / travel_mean if travel_mean > 0.0 else 0.0
            ),
            "max_energy_gain_j": max(row["max_energy_gain_j"] for row in method_rows),
            "max_penetration_m": max(row["max_penetration_m"] for row in method_rows),
            "solver_residual": dict(UNSUPPORTED_SOLVER_RESIDUAL),
            "antisymmetry_residual_m": antisymmetry_residual(method_rows),
            "antisymmetry_residual_over_peak_drift": (
                antisymmetry_residual(method_rows) / max(drifts)
                if max(drifts) > 0.0
                else dict(UNSUPPORTED_ANTISYMMETRY_RATIO)
            ),
        }
    return summary


def antisymmetry_residual(rows: list[dict[str, Any]]) -> float:
    """Largest |d(theta) + d(90-theta)| over the sweep.

    A friction pyramid aligned to the tangent axes makes lateral drift
    antisymmetric about 45 degrees, so this residual is ~0 for a genuine
    pyramid signature and comparable to the peak drift for isotropic scatter.
    It is what separates orientation-dependent anisotropy from contact noise.
    """
    by_angle = {row["angle_deg"]: row["lateral_drift_m"] for row in rows}
    residual = 0.0
    for angle, drift in by_angle.items():
        mirror = by_angle.get(90.0 - angle)
        if mirror is not None:
            residual = max(residual, abs(drift + mirror))
    return residual


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
    resolved_by_method: dict[str, dict[str, Any]] = {}

    for method in parameters["contact_solver_methods"]:
        for angle in parameters["launch_angles_deg"]:
            repeats = [
                run_single(method, angle, parameters)
                for _ in range(int(parameters["deterministic_repeats"]))
            ]
            hashes = {run["trajectory_sha256"] for run in repeats}
            if len(hashes) != 1:
                determinism_failures.append(
                    f"{method} angle {angle}: trajectory hashes differ "
                    f"{sorted(hashes)}"
                )
            row = repeats[0]
            for repeat in repeats:
                readback = repeat["resolved"]
                if readback != row["resolved"]:
                    raise SystemExit(
                        f"{method} angle {angle}: resolved configuration "
                        f"differs between repeats: {readback} vs "
                        f"{row['resolved']}"
                    )
                if readback["contact_solver_method"] != method:
                    raise SystemExit(
                        f"requested contact solver {method} but World readback "
                        f"reports {readback['contact_solver_method']}"
                    )
                if readback["time_step_s"] != float(parameters["time_step_s"]):
                    raise SystemExit(
                        f"requested timestep {parameters['time_step_s']} but "
                        f"World readback reports {readback['time_step_s']}"
                    )
                if readback["gravity_mps2"] != list(parameters["gravity_mps2"]):
                    raise SystemExit(
                        f"requested gravity {parameters['gravity_mps2']} but "
                        f"World readback reports {readback['gravity_mps2']}"
                    )
            row["repeat_trajectory_sha256"] = [
                repeat["trajectory_sha256"] for repeat in repeats
            ]
            previous = resolved_by_method.setdefault(method, row["resolved"])
            if previous != row["resolved"]:
                raise SystemExit(
                    f"{method}: resolved configuration drifted across the "
                    f"sweep: {previous} vs {row['resolved']}"
                )
            rows.append(row)

    if determinism_failures:
        raise SystemExit(
            "deterministic repeats failed:\n  " + "\n  ".join(determinism_failures)
        )

    summary = summarize(rows)
    isotropy_tolerance = {
        "max_abs_lateral_drift_m": 1.0e-4,
        "max_abs_heading_error_deg": 0.1,
        "travel_spread_relative": 0.01,
    }
    # Record which criterion fired per method, and whether the drift carries
    # the antisymmetric pyramid signature rather than orientation-independent
    # scatter. Exceeding a tolerance is not by itself evidence of the cited
    # mechanism.
    anisotropy_findings: dict[str, Any] = {}
    for method, stats in summary.items():
        criteria = sorted(
            key
            for key, tolerance in isotropy_tolerance.items()
            if isinstance(stats.get(key), (int, float)) and stats[key] > tolerance
        )
        ratio = stats["antisymmetry_residual_over_peak_drift"]
        has_signature = isinstance(ratio, (int, float)) and ratio <= 0.05
        anisotropy_findings[method] = {
            "criteria_exceeded": criteria,
            "pyramid_signature": has_signature,
            "signature_test": (
                "lateral drift antisymmetric about 45 deg to within 5% of " "peak drift"
            ),
        }
    anisotropic_methods = sorted(
        method
        for method, finding in anisotropy_findings.items()
        if finding["criteria_exceeded"] and finding["pyramid_signature"]
    )

    # A degenerate run (tunnelling, blow-up) would also break symmetry, so the
    # disposition is gated on physical validity, not on deviation alone.
    validity_failures = [
        f"{row['contact_solver_method']} angle {row['angle_deg']}: {reason}"
        for row in rows
        for reason, bad in (
            (
                "final speed departs from the analytic rolling speed 5/7 v0",
                abs(
                    row["final_planar_speed_mps"]
                    - (5.0 / 7.0) * float(parameters["launch_speed_mps"])
                )
                > 1.0e-3,
            ),
            ("energy injected", row["max_energy_gain_j"] > 1.0e-6),
            ("never reached rolling", row["slide_end_time_s"] is None),
        )
        if bad
    ]
    disposition = (
        "reproduced" if anisotropic_methods and not validity_failures else "unresolved"
    )

    # The claim boundary quotes quantitative outcomes; format them from THIS
    # run's summary so a regeneration with different rolling behavior cannot
    # combine fresh raw rows with a stale conclusion.
    peak_drift = max(stats["max_abs_lateral_drift_m"] for stats in summary.values())
    ratios = [
        stats["antisymmetry_residual_over_peak_drift"]
        for stats in summary.values()
        if isinstance(stats["antisymmetry_residual_over_peak_drift"], (int, float))
    ]
    max_ratio = max(ratios) if ratios else None
    swept_angles = sorted({row["angle_deg"] for row in rows})
    null_angles = [
        angle
        for angle in swept_angles
        if all(
            row["lateral_drift_m"] == 0.0 for row in rows if row["angle_deg"] == angle
        )
    ]
    criteria_union = sorted(
        {
            criterion
            for finding in anisotropy_findings.values()
            for criterion in finding["criteria_exceeded"]
        }
    )
    solver_names = " and ".join(sorted(summary))
    outcome_sentence = (
        "Outcome actually observed: lateral drift reaches "
        f"{peak_drift:.1e} m against a "
        f"{isotropy_tolerance['max_abs_lateral_drift_m']:.0e} m isotropy "
        "tolerance"
        + (
            ", with exact nulls at "
            + ", ".join(f"{angle:g}" for angle in null_angles)
            + " deg"
            if null_angles
            else ""
        )
        + (
            f" and antisymmetry about 45 deg to within {max_ratio:.0e} of peak"
            if max_ratio is not None
            else ""
        )
        + f", in {solver_names}; criteria exceeded: "
        + (", ".join(criteria_union) if criteria_union else "none")
        + (
            ", and every cell passes the physical-validity gate."
            if not validity_failures
            else "; some cells FAIL the physical-validity gate."
        )
    )

    # Zeros this fixture asserts are genuine measurements, not missing data.
    # The validator rejects any other exact zero and any stale entry here, so
    # an unexpected zero in a future run fails the gate instead of passing as
    # a silent sentinel.
    measured_zero_physical = [
        f"per_solver_summary.{method}.max_penetration_m"
        for method in parameters["contact_solver_methods"]
    ]
    measured_zero_numerical = ["max_penetration_m"]

    command = (
        "PYTHONPATH=build/default/cpp/Release/python pixi run python "
        "scripts/write_citation_ct001_rolling_direction_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-001",
        "title": "Rolling-direction friction dependence (DART 7 first packet)",
        "source": {
            "url": "https://leggedrobotics.github.io/SimBenchmark/",
            "claim": (
                "Polyhedral friction can produce direction-dependent "
                "rolling/sliding behavior."
            ),
        },
        "target": {
            "branch": "main",
            "commit": (head_commit := git_head()),
            "fetch_hint": target_fetch_hint(head_commit),
            "commit_role": (
                "Source state measured: the library and fixture were run at "
                "this commit, which is HEAD at capture time. The packet and "
                "its writer land in a later commit, so re-running the "
                "recorded command requires the child commit that adds the "
                "writer; the measured behavior belongs to this one."
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
                "rigid_body_solver": "SEQUENTIAL_IMPULSE (World default)",
                "integrator": "World default semi-implicit stepping",
                "precision": "float64",
                "backend": "cpu",
                "threads": "World default sequential step",
            },
            "resolved": {
                "by_contact_solver_method": resolved_by_method,
            },
            "resolved_provenance": (
                "World.resolved_configuration (the bake-time "
                "ResolvedSolverConfiguration bound to Python in this "
                "branch), recorded per cell as world_resolution, plus World "
                "property readback (contact_solver_method, "
                "rigid_body_solver, gravity, time_step) after "
                "enter_simulation_mode, each asserted equal to the request "
                "and stable across every repeat and sweep point. The "
                "independent evidence that the selection changed behavior "
                "is that the per-angle trajectory hashes differ between the "
                "two contact solvers at every angle."
            ),
            "detector": (
                "DART 7 native World collision pipeline (the World step API "
                "exposes no detector selection on main)"
            ),
            "timestep": parameters["time_step_s"],
            "substeps": 1,
            "iterations": (
                "World defaults. Sequential impulse reports its configured "
                "sweep count through StepMetrics.last_step_iterations; the "
                "boxed-LCP path records no iteration count at all (see "
                "metrics.numerical.solver_iterations_by_method)."
            ),
            "fallback_policy": (
                "World defaults; no per-island fallback reporting is exposed "
                "on main (PLAN-123 WS4 follow-up)"
            ),
        },
        "ensemble": {
            "kind": "parameter-sweep-with-deterministic-repeats",
            "sweep": [
                {"contact_solver_method": method, "angle_deg": angle}
                for method in parameters["contact_solver_methods"]
                for angle in parameters["launch_angles_deg"]
            ],
            "deterministic_repeats": int(parameters["deterministic_repeats"]),
            "deterministic_repeats_identical": not determinism_failures,
            "measurement_window": {
                "start_s": 0.0,
                "end_s": parameters["time_step_s"] * parameters["step_count"],
                "steps": parameters["step_count"],
            },
        },
        "metrics": {
            "physical": {
                "method": (
                    "Rotational-symmetry sweep: per-angle lateral drift from "
                    "the launch ray, final-velocity heading error, along-ray "
                    "travel, slide-to-roll transition time, and max "
                    "single-step kinetic-energy gain from "
                    "StepMetrics.kinetic_energy"
                ),
                "per_solver_summary": summary,
                "isotropy_tolerance": isotropy_tolerance,
                "anisotropy_findings": anisotropy_findings,
                "anisotropic_methods": anisotropic_methods,
                "validity_failures": validity_failures,
                "measured_zero_fields": measured_zero_physical,
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
                "measured_zero_fields": measured_zero_numerical,
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
                    "The oracle is numeric rotational symmetry; per-angle "
                    "trajectories have no visual claim beyond the recorded "
                    "metrics. Visual capture joins when a corpus row makes "
                    "a visible-behavior claim."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "DART 7 main, this commit, one sphere sliding to rolling on "
                "a static ground box at v0=1 m/s, mu=0.35, dt=2 ms, 1 s "
                "horizon, SEQUENTIAL_IMPULSE and BOXED_LCP contact solvers, "
                "launch angles 0-90 deg in 15 deg steps. "
                + outcome_sentence
                + " Says nothing about other speeds, "
                "shapes, stacks, historical DART versions, or DART 6."
            ),
            "limitations": [
                "Resolved identity now comes from the World's own bake-time "
                "resolution, corroborated by per-solver trajectory-hash "
                "differences.",
                "No solver residual exists on this path: it is typed "
                "unsupported, not reported as zero. Friction-cone and "
                "complementarity violations are not exposed either.",
                "The boxed-LCP path records no iteration count; only "
                "sequential impulse reports one, and that is its configured "
                "sweep count rather than an observed convergence measure.",
                "Penetration is clamped at zero by the runtime, so 0.0 means "
                "no positive penetration was observed and does not "
                "distinguish resting-tangent from separated contacts.",
                "The corpus row also names stopping distance and cone "
                "violation as oracles; neither is measured here (the sphere "
                "rolls indefinitely at 5/7 v0, and no cone metric is "
                "exposed).",
                "The sweep covers 0-90 deg; pyramid orientation with a "
                "period other than 90 deg would need a wider sweep.",
                "No high-mass-ratio or stacked variant yet; those belong "
                "to CT-002/CT-007 fixtures.",
                "SEQUENTIAL_IMPULSE and BOXED_LCP produce metric summaries "
                "that agree to printed precision on this single-contact "
                "scene while their full trajectory hashes differ; this "
                "packet is not a solver comparison and must not be quoted "
                "as one.",
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
    summary = packet["metrics"]["physical"]["per_solver_summary"]
    print(f"wrote {args.output}")
    for method, stats in summary.items():
        print(
            f"  {method}: max |lateral drift| "
            f"{stats['max_abs_lateral_drift_m']:.3e} m, max |heading err| "
            f"{stats['max_abs_heading_error_deg']:.3e} deg, travel spread "
            f"{stats['travel_spread_relative']:.3e} rel"
        )
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
