#!/usr/bin/env python3
"""Write the CT-001 rolling-direction evidence packet for release-6.20.

Bounded claim (PLAN-123 corpus row CT-001, owned on `main`): polyhedral
friction can produce direction-dependent rolling/sliding behavior. Under an
isotropic Coulomb model a sphere launched sliding (no spin) on a horizontal
plane behaves identically for every in-plane launch direction; a friction
pyramid aligned to fixed tangent axes breaks that rotational symmetry.

DART 6 lane: reproduce with existing methods/detectors only. The fixture
launches one sphere per run across a swept launch angle on a static ground
box and repeats the sweep for every collision detector available in this
build (`fcl` default, `dart`, plus `bullet`/`ode` when compiled), keeping the
default boxed-LCP constraint solver. Per run it records lateral drift from
the launch ray, final-velocity heading error, along-ray travel,
slide-to-roll transition time, kinetic-energy gain bound, max penetration,
and contact counts; each cell runs twice and must be bit-identical.

Additive evidence tooling only: no library, API, ABI, or default change.

Usage (after `pixi run build` with dartpy):

    PYTHONPATH=build/default/cpp/Release/python/dartpy pixi run python \
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

import dartpy as dart
import numpy as np
from citation_packet_utils import (
    UNSUPPORTED_ANTISYMMETRY_RATIO,
    UNSUPPORTED_FALLBACK_EVENTS,
    UNSUPPORTED_SOLVER_ITERATIONS,
    UNSUPPORTED_SOLVER_RESIDUAL,
    preserve_review,
    target_fetch_hint,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "design"
    / "dart6_citation_driven_contact_trust"
    / "evidence"
    / "CT-001-dart6-rolling-direction.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct001_rolling_direction_sweep_dart6",
    "description": (
        "One 1 kg sphere per run, radius 0.08 m, launched sliding (no spin) "
        "at 1.0 m/s along a swept in-plane angle on a static ground box, "
        "restitution 0, friction 0.35 on both bodies, default boxed-LCP "
        "constraint solver, one sweep per available collision detector."
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
    "deterministic_repeats": 2,
    "slip_ratio_rolling_threshold": 0.02,
}

DETECTOR_FACTORIES = (
    ("fcl", "FCLCollisionDetector"),
    ("dart", "DARTCollisionDetector"),
    ("bullet", "BulletCollisionDetector"),
    ("ode", "OdeCollisionDetector"),
)


def available_detectors() -> list[str]:
    return [
        key
        for key, class_name in DETECTOR_FACTORIES
        if hasattr(dart.collision, class_name)
    ]


def make_detector(key: str):
    for candidate, class_name in DETECTOR_FACTORIES:
        if candidate == key:
            return getattr(dart.collision, class_name)()
    raise SystemExit(f"unknown collision detector key {key!r}")


def scene_digest(parameters: dict[str, Any]) -> str:
    canonical = json.dumps(parameters, sort_keys=True, separators=(",", ":"))
    return "sha256:" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _make_sphere(parameters: dict[str, Any]) -> tuple[Any, Any, Any]:
    radius = float(parameters["sphere_radius_m"])
    mass = float(parameters["sphere_mass_kg"])
    skel = dart.dynamics.Skeleton("ct001_sphere")
    joint, body = skel.createFreeJointAndBodyNodePair(None)
    body.setName("ct001_sphere_body")
    shape = dart.dynamics.SphereShape(radius)
    shape_node = body.createShapeNode(shape)
    shape_node.createCollisionAspect()
    dynamics_aspect = shape_node.createDynamicsAspect()
    dynamics_aspect.setFrictionCoeff(float(parameters["friction"]))
    dynamics_aspect.setRestitutionCoeff(float(parameters["restitution"]))
    body.setInertia(
        dart.dynamics.Inertia(mass, np.zeros(3), shape.computeInertia(mass))
    )
    start = np.array([0.0, 0.0, radius])
    joint.setTransform(dart.math.Isometry3(np.eye(3), start))
    return skel, joint, body


def _make_ground(parameters: dict[str, Any]) -> Any:
    half = np.asarray(parameters["ground_half_extents_m"], dtype=float)
    skel = dart.dynamics.Skeleton("ct001_ground")
    _, body = skel.createFreeJointAndBodyNodePair(None)
    body.setName("ct001_ground_body")
    shape = dart.dynamics.BoxShape(2.0 * half)
    shape_node = body.createShapeNode(shape)
    shape_node.createCollisionAspect()
    dynamics_aspect = shape_node.createDynamicsAspect()
    dynamics_aspect.setFrictionCoeff(float(parameters["friction"]))
    dynamics_aspect.setRestitutionCoeff(float(parameters["restitution"]))
    skel.getJoint(0).setTransform(
        dart.math.Isometry3(np.eye(3), np.array([0.0, 0.0, -half[2]]))
    )
    skel.setMobile(False)
    return skel


def run_single(
    detector_key: str, angle_deg: float, parameters: dict[str, Any]
) -> dict[str, Any]:
    """Run one launch and return raw metrics plus a trajectory hash."""
    radius = float(parameters["sphere_radius_m"])
    speed = float(parameters["launch_speed_mps"])
    dt = float(parameters["time_step_s"])
    step_count = int(parameters["step_count"])
    angle = math.radians(angle_deg)
    direction = np.array([math.cos(angle), math.sin(angle), 0.0])
    lateral_axis = np.array([-direction[1], direction[0], 0.0])

    world = dart.simulation.World("ct001")
    world.setGravity(np.asarray(parameters["gravity_mps2"], dtype=float))
    world.setTimeStep(dt)
    constraint_solver = world.getConstraintSolver()
    constraint_solver.setCollisionDetector(make_detector(detector_key))

    world.addSkeleton(_make_ground(parameters))
    sphere_skel, joint, body = _make_sphere(parameters)
    world.addSkeleton(sphere_skel)

    joint.setVelocities(np.concatenate([np.zeros(3), speed * direction]))
    launch_velocity = np.asarray(body.getLinearVelocity(), dtype=float)
    if not np.allclose(launch_velocity, speed * direction, atol=1.0e-12):
        raise SystemExit(
            f"free-joint launch velocity readback {launch_velocity} does not "
            f"match requested {speed * direction}"
        )

    resolved = {
        "collision_detector": constraint_solver.getCollisionDetector().getType(),
        "constraint_solver": type(constraint_solver).__name__,
        "gravity_mps2": [float(v) for v in np.asarray(world.getGravity())],
        "time_step_s": float(world.getTimeStep()),
    }

    trajectory = hashlib.sha256()
    start = np.array([0.0, 0.0, radius])
    slide_end_time = None
    max_energy_gain = 0.0
    max_penetration = 0.0
    contact_count_max = 0
    # Start the energy-gain gate from the launched (pre-step) state so the
    # first contact solve is covered; an initial injection that later steps
    # dissipate must not escape the physical-validity gate.
    previous_energy = float(sphere_skel.computeKineticEnergy())

    for step_index in range(step_count):
        world.step()
        position = np.asarray(body.getTransform().translation(), dtype=float)
        velocity = np.asarray(body.getLinearVelocity(), dtype=float)
        angular = np.asarray(body.getAngularVelocity(), dtype=float)
        trajectory.update(position.tobytes())
        trajectory.update(velocity.tobytes())
        trajectory.update(angular.tobytes())

        kinetic = float(sphere_skel.computeKineticEnergy())
        max_energy_gain = max(max_energy_gain, kinetic - previous_energy)
        previous_energy = kinetic

        collision_result = world.getLastCollisionResult()
        contacts = collision_result.getContacts()
        contact_count_max = max(contact_count_max, len(contacts))
        for contact in contacts:
            max_penetration = max(max_penetration, float(contact.penetrationDepth))

        if slide_end_time is None:
            planar_speed = float(np.linalg.norm(velocity[:2]))
            spin_axis = np.array([-direction[1], direction[0], 0.0])
            surface_speed = radius * float(np.dot(angular, spin_axis))
            slip = abs(planar_speed - surface_speed)
            denom = planar_speed + abs(surface_speed) + 1.0e-9
            if slip / denom < float(parameters["slip_ratio_rolling_threshold"]):
                slide_end_time = (step_index + 1) * dt

    final_position = np.asarray(body.getTransform().translation(), dtype=float)
    final_velocity = np.asarray(body.getLinearVelocity(), dtype=float)
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
        "collision_detector": detector_key,
        "resolved": resolved,
        "trajectory_sha256": trajectory.hexdigest(),
        "along_travel_m": along,
        "lateral_drift_m": lateral,
        "heading_error_deg": heading_error_deg,
        "final_planar_speed_mps": planar_speed,
        "slide_end_time_s": slide_end_time,
        "max_energy_gain_j": max_energy_gain,
        "max_penetration_m": max_penetration,
        "max_contact_count": contact_count_max,
        "final_height_m": float(final_position[2]),
    }


def antisymmetry_residual(rows: list[dict[str, Any]]) -> float:
    """Largest |d(theta) + d(90-theta)| over the sweep.

    A friction pyramid aligned to the tangent axes makes lateral drift
    antisymmetric about 45 degrees, so this residual is ~0 for a genuine
    pyramid signature and comparable to the peak drift for isotropic
    contact-generation scatter. Exceeding a drift tolerance is not by itself
    evidence of the cited mechanism; this statistic is what separates them.
    """
    by_angle = {row["angle_deg"]: row["lateral_drift_m"] for row in rows}
    residual = 0.0
    for angle, drift in by_angle.items():
        mirror = by_angle.get(90.0 - angle)
        if mirror is not None:
            residual = max(residual, abs(drift + mirror))
    return residual


def summarize(rows: list[dict[str, Any]], detectors: list[str]) -> dict[str, Any]:
    """Per-detector rotational-symmetry summary across the angle sweep."""
    summary: dict[str, Any] = {}
    for detector in detectors:
        detector_rows = [row for row in rows if row["collision_detector"] == detector]
        drifts = [abs(row["lateral_drift_m"]) for row in detector_rows]
        travels = [row["along_travel_m"] for row in detector_rows]
        headings = [abs(row["heading_error_deg"]) for row in detector_rows]
        travel_mean = sum(travels) / len(travels)
        travel_spread = max(travels) - min(travels)
        summary[detector] = {
            "runs": len(detector_rows),
            "max_abs_lateral_drift_m": max(drifts),
            "max_abs_heading_error_deg": max(headings),
            "travel_mean_m": travel_mean,
            "travel_spread_m": travel_spread,
            "travel_spread_relative": (
                travel_spread / travel_mean if travel_mean > 0.0 else 0.0
            ),
            "max_energy_gain_j": max(row["max_energy_gain_j"] for row in detector_rows),
            "max_penetration_m": max(row["max_penetration_m"] for row in detector_rows),
            "min_final_height_m": min(row["final_height_m"] for row in detector_rows),
            "max_contact_count": max(row["max_contact_count"] for row in detector_rows),
            "antisymmetry_residual_m": antisymmetry_residual(detector_rows),
            "antisymmetry_residual_over_peak_drift": (
                antisymmetry_residual(detector_rows) / max(drifts)
                if max(drifts) > 0.0
                else dict(UNSUPPORTED_ANTISYMMETRY_RATIO)
            ),
        }
    return summary


def git_head() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def build_packet(output_path: Path | None = None) -> dict[str, Any]:
    parameters = dict(SCENE_PARAMETERS)
    detectors = available_detectors()
    parameters["collision_detectors"] = detectors

    rows: list[dict[str, Any]] = []
    determinism_failures: list[str] = []
    resolved_by_detector: dict[str, dict[str, Any]] = {}

    for detector in detectors:
        for angle in parameters["launch_angles_deg"]:
            repeats = [
                run_single(detector, angle, parameters)
                for _ in range(int(parameters["deterministic_repeats"]))
            ]
            hashes = {run["trajectory_sha256"] for run in repeats}
            if len(hashes) != 1:
                determinism_failures.append(
                    f"{detector} angle {angle}: trajectory hashes differ "
                    f"{sorted(hashes)}"
                )
            row = repeats[0]
            for repeat in repeats:
                readback = repeat["resolved"]
                if readback["collision_detector"] != detector:
                    raise SystemExit(
                        f"requested detector {detector} but readback reports "
                        f"{readback['collision_detector']}"
                    )
                if readback["time_step_s"] != float(parameters["time_step_s"]):
                    raise SystemExit(
                        f"requested timestep {parameters['time_step_s']} but "
                        f"readback reports {readback['time_step_s']}"
                    )
                if readback["gravity_mps2"] != list(parameters["gravity_mps2"]):
                    raise SystemExit(
                        f"requested gravity {parameters['gravity_mps2']} but "
                        f"readback reports {readback['gravity_mps2']}"
                    )
                if readback != row["resolved"]:
                    raise SystemExit(
                        f"{detector} angle {angle}: resolved configuration "
                        f"differs between repeats: {readback} vs "
                        f"{row['resolved']}"
                    )
            row["repeat_trajectory_sha256"] = [
                repeat["trajectory_sha256"] for repeat in repeats
            ]
            previous = resolved_by_detector.setdefault(detector, row["resolved"])
            if previous != row["resolved"]:
                raise SystemExit(
                    f"{detector}: resolved configuration drifted across the "
                    f"sweep: {previous} vs {row['resolved']}"
                )
            rows.append(row)

    if determinism_failures:
        raise SystemExit(
            "deterministic repeats failed:\n  " + "\n  ".join(determinism_failures)
        )

    summary = summarize(rows, detectors)
    isotropy_tolerance = {
        "max_abs_lateral_drift_m": 1.0e-4,
        "max_abs_heading_error_deg": 0.1,
        "travel_spread_relative": 0.01,
    }
    # Record which criterion fired per detector and whether the drift carries
    # the antisymmetric pyramid signature. Exceeding a drift tolerance with no
    # angular structure is contact-generation scatter, not the cited
    # orientation-dependent mechanism, and must not be counted as a fourth
    # corroborating instance.
    anisotropy_findings: dict[str, Any] = {}
    for detector, stats in summary.items():
        criteria = sorted(
            key
            for key, tolerance in isotropy_tolerance.items()
            if isinstance(stats.get(key), (int, float)) and stats[key] > tolerance
        )
        ratio = stats["antisymmetry_residual_over_peak_drift"]
        has_signature = isinstance(ratio, (int, float)) and ratio <= 0.05
        anisotropy_findings[detector] = {
            "criteria_exceeded": criteria,
            "pyramid_signature": has_signature,
            "signature_test": (
                "lateral drift antisymmetric about 45 deg to within 5% of " "peak drift"
            ),
            "attribution": (
                "orientation-dependent friction-pyramid anisotropy"
                if has_signature and criteria
                else (
                    "scatter exceeding tolerance without angular structure; "
                    "not attributed to polyhedral friction by this packet"
                    if criteria
                    else "within isotropy tolerance"
                )
            ),
        }
    anisotropic_detectors = sorted(
        detector
        for detector, finding in anisotropy_findings.items()
        if finding["criteria_exceeded"] and finding["pyramid_signature"]
    )
    # Detectors that exceed tolerance WITHOUT the pyramid signature are
    # excluded from the reproducing set. The packet's conclusions are built
    # from these computed sets so a build without bullet, or a bullet that
    # starts conforming, regenerates consistent text instead of asserting
    # one historical run's results.
    nonconforming_detectors = sorted(
        detector
        for detector, finding in anisotropy_findings.items()
        if finding["criteria_exceeded"] and not finding["pyramid_signature"]
    )

    # Detectors whose trajectory hashes match exactly are one measurement, not
    # several; recording that keeps the sweep from reading as more independent
    # corroboration than it is.
    hash_groups: dict[tuple[str, ...], list[str]] = {}
    for detector in detectors:
        key = tuple(
            row["trajectory_sha256"]
            for row in rows
            if row["collision_detector"] == detector
        )
        hash_groups.setdefault(key, []).append(detector)
    identical_detector_groups = sorted(
        sorted(group) for group in hash_groups.values() if len(group) > 1
    )

    # A degenerate run (tunnelling, blow-up) would also break symmetry, so the
    # disposition is gated on physical validity, not on deviation alone.
    radius = float(parameters["sphere_radius_m"])
    validity_failures = [
        f"{row['collision_detector']} angle {row['angle_deg']}: {reason}"
        for row in rows
        for reason, bad in (
            (
                "final speed departs from the analytic rolling speed 5/7 v0",
                abs(
                    row["final_planar_speed_mps"]
                    - (5.0 / 7.0) * float(parameters["launch_speed_mps"])
                )
                > 1.0e-2,
            ),
            ("never reached rolling", row["slide_end_time_s"] is None),
            (
                "sphere sank below its own radius (fall-through)",
                row["final_height_m"] < 0.5 * radius,
            ),
            (
                "penetration exceeded a tenth of the radius",
                row["max_penetration_m"] > 0.1 * radius,
            ),
            (
                # A sliding sphere must dissipate; a per-step kinetic gain
                # above a thousandth of the launch energy is a blow-up
                # signature, which the gate must catch before any symmetry
                # verdict is trusted.
                "kinetic energy injected during the run",
                row["max_energy_gain_j"]
                > 1.0e-3
                * 0.5
                * float(parameters["sphere_mass_kg"])
                * float(parameters["launch_speed_mps"]) ** 2,
            ),
        )
        if bad
    ]
    disposition = (
        "reproduced"
        if anisotropic_detectors and not validity_failures
        else "unresolved"
    )

    command = (
        "PYTHONPATH=build/default/cpp/Release/python/dartpy pixi run python "
        "scripts/write_citation_ct001_rolling_direction_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-001",
        "title": (
            "Rolling-direction friction dependence " "(release-6.20 detector sweep)"
        ),
        "source": {
            "url": "https://leggedrobotics.github.io/SimBenchmark/",
            "claim": (
                "Polyhedral friction can produce direction-dependent "
                "rolling/sliding behavior."
            ),
        },
        "target": {
            "branch": "release-6.20",
            "commit": git_head(),
            "fetch_hint": target_fetch_hint(),
        },
        "scene": {
            "id": parameters["scene_id"],
            "digest": scene_digest(parameters),
            "description": parameters["description"],
            "parameters": parameters,
        },
        "configuration": {
            "requested": {
                "collision_detector_sweep": detectors,
                "constraint_solver": "World default (boxed LCP)",
                "integrator": "World::step semi-implicit default",
                "precision": "float64",
                "backend": "cpu",
                "threads": "single-threaded default",
            },
            "resolved": {
                "by_collision_detector": resolved_by_detector,
            },
            "resolved_provenance": (
                "ConstraintSolver.getCollisionDetector().getType() readback "
                "after setCollisionDetector plus constraint-solver type "
                "name; the writer aborts if readback differs from the "
                "request: detector type, timestep, and gravity are each "
                "asserted per run and across repeats. Boxed-LCP internal "
                "Dantzig/PGS selection is not exposed per solve on "
                "release-6.20."
            ),
            "detector": "swept: " + ", ".join(detectors),
            "timestep": parameters["time_step_s"],
            "substeps": 1,
            "iterations": (
                "Dantzig direct solve with PGS fallback; per-solve iteration "
                "counts are not exposed on release-6.20 and are typed "
                "unsupported in metrics.numerical.solver_iterations rather "
                "than reported as zero"
            ),
            "fallback_policy": (
                "BoxedLcpConstraintSolver secondary-solver fallback "
                "(default); per-solve fallback events are not exposed on "
                "release-6.20"
            ),
        },
        "ensemble": {
            "kind": "parameter-sweep-with-deterministic-repeats",
            "sweep": [
                {"collision_detector": detector, "angle_deg": angle}
                for detector in detectors
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
                    "Rotational-symmetry sweep: per-angle lateral drift "
                    "from the launch ray, final-velocity heading error, "
                    "along-ray travel, slide-to-roll transition time, and "
                    "max single-step kinetic-energy gain from "
                    "Skeleton.computeKineticEnergy"
                ),
                "per_detector_summary": summary,
                "isotropy_tolerance": isotropy_tolerance,
                "anisotropy_findings": anisotropy_findings,
                "anisotropic_detectors": anisotropic_detectors,
                "identical_detector_groups": identical_detector_groups,
                "validity_failures": validity_failures,
            },
            "numerical": {
                "method": (
                    "Max over run of contact penetrationDepth and contact "
                    "count from World.getLastCollisionResult; sphere final "
                    "height is checked against fall-through in the "
                    "disposition validity gate"
                ),
                "max_penetration_m": max(row["max_penetration_m"] for row in rows),
                "max_contact_count": max(row["max_contact_count"] for row in rows),
                "min_final_height_m": min(row["final_height_m"] for row in rows),
                "solver_iterations": dict(UNSUPPORTED_SOLVER_ITERATIONS),
                "solver_residual": dict(UNSUPPORTED_SOLVER_RESIDUAL),
                "solver_fallback_events": dict(UNSUPPORTED_FALLBACK_EVENTS),
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
                    "No allocation methodology on this row; DART 6 has no "
                    "PLAN-122-style allocation gates and this packet does "
                    "not measure allocations"
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
                    "trajectories carry no visible-behavior claim beyond "
                    "the recorded metrics."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "release-6.20, this commit, one sphere sliding to rolling "
                "on a static ground box at v0=1 m/s, mu=0.35, dt=2 ms, 1 s "
                "horizon, default boxed-LCP constraint solver, launch "
                "angles 0-90 deg in 15 deg steps, detectors "
                + ", ".join(detectors)
                + ". "
                + (
                    "The claim reproduces on "
                    + ", ".join(anisotropic_detectors)
                    + ", which show the antisymmetric friction-pyramid "
                    "signature"
                    if anisotropic_detectors
                    else (
                        "No swept detector shows the antisymmetric "
                        "friction-pyramid signature"
                    )
                )
                + (
                    "; excluded for exceeding the drift tolerance without "
                    "that angular structure: " + ", ".join(nonconforming_detectors)
                    if nonconforming_detectors
                    else ""
                )
                + ". Says nothing about other "
                "speeds, shapes, stacks, historical DART versions, or DART 7."
            ),
            "limitations": [
                "Per-solve LCP iteration counts, residuals, and "
                "Dantzig-vs-PGS fallback events are not exposed on "
                "release-6.20; they are typed unsupported in "
                "metrics.numerical rather than reported as zero, and solver "
                "identity is type-level readback.",
            ]
            + [
                f"{detector} exceeds the drift tolerance without the "
                "antisymmetric pyramid signature "
                f"({anisotropy_findings[detector]['attribution']}); its "
                "scatter is NOT attributed to polyhedral friction by this "
                "packet and it is excluded from the reproducing set. Its "
                "per-angle statistics are in metrics.physical and the raw "
                "rows."
                for detector in nonconforming_detectors
            ]
            + (
                [
                    "Detector groups "
                    + "; ".join(", ".join(group) for group in identical_detector_groups)
                    + " produce bit-identical trajectory hashes at every "
                    "angle, so the sweep contains fewer independent "
                    "measurements than detectors; see "
                    "metrics.physical.identical_detector_groups."
                ]
                if identical_detector_groups
                else []
            )
            + [
                "The sweep covers 0-90 deg; pyramid orientation with a "
                "period other than 90 deg would need a wider sweep.",
                "Detector availability depends on the build; the packet "
                "records the swept set explicitly.",
                "This packet is not a cross-detector accuracy ranking; it "
                "only measures rotational-symmetry breaking per detector.",
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
    summary = packet["metrics"]["physical"]["per_detector_summary"]
    print(f"wrote {args.output}")
    for detector, stats in summary.items():
        print(
            f"  {detector}: max |lateral drift| "
            f"{stats['max_abs_lateral_drift_m']:.3e} m, max |heading err| "
            f"{stats['max_abs_heading_error_deg']:.3e} deg, travel spread "
            f"{stats['travel_spread_relative']:.3e} rel, min height "
            f"{stats['min_final_height_m']:.4f} m"
        )
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
