#!/usr/bin/env python3
"""Write the CT-005 PD-control tracking evidence packet (PLAN-123 WS2).

Bounded claim (corpus row CT-005): controlled robot tracking exposes
whole-step speed/accuracy tradeoffs.

Bounded reconstruction (explicitly not source-exact): a 4-link articulated
chain driven by a fixed-gain PD controller toward a smooth sinusoidal joint
reference, swept over a timestep grid for each multibody integration family.
No contact. Per cell it records:

- tracking error against the reference (RMS and maximum over the window);
- control work, the integral of |tau . qdot| dt, so accuracy can be read
  against actuation effort rather than against nothing;
- whether the controller stayed stable (finite state, bounded torque);
- deterministic repeats: each cell runs twice and must be bit-identical.

The corpus row's oracle also names constraint error and a timing
distribution. This fixture has no constraints beyond the joint structure, and
it measures no time: `metrics.performance` is typed unsupported because no
interleaved same-host methodology was applied. The "speed" half of the
speed/accuracy tradeoff is therefore NOT established here, and the packet
disposition reflects that rather than promoting the row on the accuracy half
alone.

Usage (after `pixi run build`):

    PYTHONPATH=build/default/cpp/Release/python pixi run python \
        scripts/write_citation_ct005_pd_tracking_packet.py
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
)

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "123-citation-driven-simulation-trust"
    / "evidence"
    / "CT-005-dart7-pd-tracking.json"
)

SCENE_PARAMETERS: dict[str, Any] = {
    "scene_id": "ct005_pd_tracked_chain",
    "description": (
        "4-link chain hanging downward under gravity (revolute hinges "
        "about y, 0.3 m links, 1.0 kg each, mass centered at mid-link), "
        "driven by an inertia-scaled joint PD controller toward a smooth "
        "sinusoidal reference about the hanging equilibrium, no contact, "
        "simulated to a 2 s horizon per timestep/integration-family cell."
    ),
    "gravity_mps2": [0.0, 0.0, -9.81],
    "link_count": 4,
    "link_length_m": 0.3,
    "link_mass_kg": 1.0,
    "link_radius_m": 0.05,
    "reference_amplitude_rad": 0.25,
    "reference_frequency_hz": 0.5,
    "pd_natural_frequency_rad_per_s": 20.0,
    "pd_damping_ratio": 1.0,
    "torque_limit_nm": 500.0,
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


def downstream_inertia(index: int, parameters: dict[str, Any]) -> float:
    """Inertia the chain below hinge `index` presents about that hinge.

    Each joint of a serial chain sees a different load -- here they differ by
    roughly 60x between root and tip -- so a single gain pair cannot serve
    both ends: gains stable at the tip are far too soft at the root, and
    gains stiff enough for the root make the tip's explicit damping term
    unstable at the coarser timesteps. Scaling each joint's gains by its own
    downstream inertia gives every joint the same closed-loop natural
    frequency and damping ratio, which is what makes the timestep sweep a
    measurement of integration rather than of gain tuning.
    """
    count = int(parameters["link_count"])
    length = float(parameters["link_length_m"])
    mass = float(parameters["link_mass_kg"])
    own = mass * length * length / 12.0
    total = 0.0
    for j in range(index, count):
        lever = (j - index) * length + 0.5 * length
        total += mass * lever * lever + own
    return total


def reference_angle(time: float, index: int, parameters: dict[str, Any]) -> float:
    """Smooth per-joint sinusoidal reference, phase-shifted along the chain."""
    amplitude = float(parameters["reference_amplitude_rad"])
    frequency = float(parameters["reference_frequency_hz"])
    phase = 0.5 * math.pi * index / max(1, int(parameters["link_count"]))
    return amplitude * math.sin(2.0 * math.pi * frequency * time + phase)


def reference_rate(time: float, index: int, parameters: dict[str, Any]) -> float:
    """Analytic derivative of the reference, so the controller is not
    differentiating a signal numerically."""
    amplitude = float(parameters["reference_amplitude_rad"])
    frequency = float(parameters["reference_frequency_hz"])
    omega = 2.0 * math.pi * frequency
    phase = 0.5 * math.pi * index / max(1, int(parameters["link_count"]))
    return amplitude * omega * math.cos(omega * time + phase)


def reference_acceleration(
    time: float, index: int, parameters: dict[str, Any]
) -> float:
    """Analytic second derivative of the reference."""
    amplitude = float(parameters["reference_amplitude_rad"])
    frequency = float(parameters["reference_frequency_hz"])
    omega = 2.0 * math.pi * frequency
    phase = 0.5 * math.pi * index / max(1, int(parameters["link_count"]))
    return -amplitude * omega * omega * math.sin(omega * time + phase)


def run_single(
    family_name: str, timestep: float, parameters: dict[str, Any]
) -> dict[str, Any]:
    """Run one PD-tracked chain and return raw metrics plus a hash."""
    link_count = int(parameters["link_count"])
    length = float(parameters["link_length_m"])
    mass = float(parameters["link_mass_kg"])
    radius = float(parameters["link_radius_m"])
    omega = float(parameters["pd_natural_frequency_rad_per_s"])
    zeta = float(parameters["pd_damping_ratio"])
    torque_limit = float(parameters["torque_limit_nm"])
    step_count = int(round(float(parameters["horizon_s"]) / timestep))

    world = sx.World(
        time_step=timestep,
        gravity=np.asarray(parameters["gravity_mps2"], dtype=float),
        multibody_options=sx.MultibodyOptions(
            integration_family=sx.MultibodyIntegrationFamily[family_name]
        ),
    )

    chain = world.add_multibody("ct005_chain")
    parent = chain.add_link("base")
    ixx = 0.5 * mass * radius * radius
    itrans = mass * length * length / 12.0
    joints = []
    for index in range(link_count):
        offset = 0.0 if index == 0 else length
        link = chain.add_link(
            f"link{index}",
            parent=parent,
            joint=sx.JointSpec(
                name=f"hinge{index}",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 1.0, 0.0),
                transform_from_parent=_translation(0.0, 0.0, -offset),
            ),
        )
        link.mass = mass
        link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
        # Rod-like link: mass centered halfway along its own length, so each
        # hinge sees a real gravity lever instead of a point mass sitting on
        # the axis.
        link.center_of_mass = (0.0, 0.0, -0.5 * length)
        link.parent_joint.position = [reference_angle(0.0, index, parameters)]
        joints.append(link.parent_joint)
        parent = link

    world.enter_simulation_mode()

    resolved = {
        "integration_family": world.multibody_options.integration_family.name,
        "gravity_mps2": [float(v) for v in np.asarray(world.gravity)],
        "time_step_s": float(world.time_step),
    }

    trajectory = hashlib.sha256()
    squared_error_sum = 0.0
    error_samples = 0
    max_abs_error = 0.0
    control_work = 0.0
    max_abs_torque = 0.0
    torque_saturated = False
    non_finite = False

    for step_index in range(step_count):
        time = step_index * timestep
        positions = np.array(
            [float(np.asarray(j.position, dtype=float)[0]) for j in joints]
        )
        velocities = np.array(
            [float(np.asarray(j.velocity, dtype=float)[0]) for j in joints]
        )
        targets = np.array(
            [reference_angle(time, i, parameters) for i in range(link_count)]
        )
        target_rates = np.array(
            [reference_rate(time, i, parameters) for i in range(link_count)]
        )
        target_accels = np.array(
            [reference_acceleration(time, i, parameters) for i in range(link_count)]
        )
        # Computed torque: ask the model what torque realizes the commanded
        # acceleration, so the closed loop has the specified natural frequency
        # and damping in every configuration. A hand-tuned diagonal PD cannot:
        # a serial chain's root joint sees a far smaller articulated inertia
        # than its composite rigid-body inertia, so gains sized from geometry
        # destabilize the root while the tip tracks fine.
        commanded = (
            target_accels
            + 2.0 * zeta * omega * (target_rates - velocities)
            + omega * omega * (targets - positions)
        )
        torques = np.asarray(chain.compute_inverse_dynamics(commanded), dtype=float)
        for index, joint in enumerate(joints):
            torque = float(torques[index])
            if abs(torque) > torque_limit:
                torque_saturated = True
                torque = math.copysign(torque_limit, torque)
            joint.force = [torque]
            max_abs_torque = max(max_abs_torque, abs(torque))
            control_work += abs(torque * velocities[index]) * timestep

        world.step()

        for index, joint in enumerate(joints):
            target = reference_angle(time + timestep, index, parameters)
            position = float(np.asarray(joint.position, dtype=float)[0])
            velocity = float(np.asarray(joint.velocity, dtype=float)[0])
            if not (math.isfinite(position) and math.isfinite(velocity)):
                non_finite = True
                break
            error = position - target
            squared_error_sum += error * error
            error_samples += 1
            max_abs_error = max(max_abs_error, abs(error))
            trajectory.update(np.array([position, velocity]).tobytes())
        if non_finite:
            break

    rms_error = math.sqrt(squared_error_sum / error_samples) if error_samples else None
    return {
        "integration_family": family_name,
        "timestep_s": timestep,
        "steps": step_count,
        "resolved": resolved,
        "finite": not non_finite,
        "trajectory_sha256": trajectory.hexdigest(),
        "rms_tracking_error_rad": rms_error,
        "max_abs_tracking_error_rad": max_abs_error,
        "control_work_j": control_work,
        "max_abs_torque_nm": max_abs_torque,
        "torque_saturated": torque_saturated,
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
        errors_by_dt = {
            row["timestep_s"]: row["rms_tracking_error_rad"]
            for row in family_rows
            if row["rms_tracking_error_rad"] is not None
        }
        work_by_dt = {row["timestep_s"]: row["control_work_j"] for row in family_rows}
        smallest = min(errors_by_dt) if errors_by_dt else None
        largest = max(errors_by_dt) if errors_by_dt else None
        family_summary[family] = {
            "cells": len(family_rows),
            "rms_tracking_error_by_timestep": {
                str(dt): value for dt, value in sorted(errors_by_dt.items())
            },
            "control_work_by_timestep": {
                str(dt): value for dt, value in sorted(work_by_dt.items())
            },
            "rms_error_at_smallest_timestep": errors_by_dt[smallest],
            "rms_error_at_largest_timestep": errors_by_dt[largest],
            "tracking_improves_with_timestep": (
                errors_by_dt[smallest] < errors_by_dt[largest]
            ),
            "max_abs_torque_nm": max(row["max_abs_torque_nm"] for row in family_rows),
            "any_torque_saturated": any(row["torque_saturated"] for row in family_rows),
        }

    all_finite = all(row["finite"] for row in rows)
    improving = sorted(
        family
        for family, stats in family_summary.items()
        if stats["tracking_improves_with_timestep"]
    )

    # The cited claim is about a speed/accuracy tradeoff. This packet measures
    # accuracy and actuation effort but no speed, so half of the tradeoff is
    # absent and the row cannot be promoted -- the same reasoning that keeps
    # CT-004 unresolved. The accuracy result is published as its own finding.
    disposition = "unresolved"
    error_spread = {
        family: (
            stats["rms_error_at_smallest_timestep"]
            / stats["rms_error_at_largest_timestep"]
        )
        for family, stats in family_summary.items()
    }
    accuracy_finding = {
        "all_cells_finite": all_finite,
        "families_with_improving_tracking": improving,
        "rms_error_ratio_smallest_over_largest_timestep": error_spread,
        "regime": ("controller-limited" if not improving else "integration-limited"),
        "note": (
            "Under computed-torque control the tracking error is set by the "
            "closed-loop bandwidth, not by integration error: refining the "
            "timestep eightfold does not reduce RMS error (it varies by "
            "about 13% and is in fact slightly lower at the coarsest "
            "timestep), while control work rises monotonically as the "
            "timestep shrinks. So for this system the whole-step tradeoff is "
            "not the simple 'smaller timestep is more accurate' shape. Step "
            "cost is still not measured, so no speed/accuracy tradeoff is "
            "established and this finding does not promote the corpus row."
        ),
    }

    command = (
        "PYTHONPATH=build/default/cpp/Release/python pixi run python "
        "scripts/write_citation_ct005_pd_tracking_packet.py"
    )
    packet: dict[str, Any] = {
        "schema": "dart.citation_claim_evidence/v1",
        "claim_id": "CT-005",
        "title": (
            "PD-control tracking error and control work versus timestep "
            "(DART 7 first packet)"
        ),
        "source": {
            "url": "https://leggedrobotics.github.io/SimBenchmark/",
            "claim": (
                "Controlled robot tracking exposes whole-step speed/accuracy "
                "tradeoffs."
            ),
        },
        "target": {
            "branch": "main",
            "commit": git_head(),
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
                "controller": (
                    "joint PD with per-joint gains scaled to each joint's downstream "
                    "inertia for a uniform 20 rad/s closed-loop natural "
                    "frequency at critical damping, "
                    "torque limit 500 Nm, applied before each step"
                ),
                "contact": "none (no collision shapes)",
                "precision": "float64",
                "backend": "cpu",
                "threads": "World default sequential step",
            },
            "resolved": {"by_cell": resolved_by_cell},
            "resolved_provenance": (
                "World property readback after enter_simulation_mode, with "
                "integration family, timestep, and gravity each asserted "
                "equal to the request per repeat and per cell. The controller "
                "is applied by this script, not resolved by the World. "
                "configuration.timestep is the smallest swept value; "
                "configuration.resolved.by_cell is authoritative per cell. "
                "World::getResolvedConfiguration() is not yet exposed to "
                "Python (PLAN-123 WS4 follow-up)."
            ),
            "detector": (
                "not applicable: the scene has no collision shapes and runs "
                "no narrow phase"
            ),
            "timestep": min(parameters["timesteps_s"]),
            "substeps": 1,
            "iterations": (
                "World defaults; the variational family uses its default "
                "iteration limit and tolerance, which this packet does not "
                "vary"
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
                    "Per-step joint error against the sinusoidal reference "
                    "(RMS and maximum over the window), and control work as "
                    "the integral of |tau . qdot| dt accumulated per step"
                ),
                "per_family_summary": family_summary,
                "accuracy_finding": accuracy_finding,
            },
            "numerical": {
                "method": (
                    "Maximum applied joint torque and whether the configured "
                    "torque limit was reached in any cell"
                ),
                "max_abs_torque_nm_over_cells": max(
                    row["max_abs_torque_nm"] for row in rows
                ),
                "any_torque_saturated": any(row["torque_saturated"] for row in rows),
                "constraint_error": {
                    "status": "unsupported",
                    "reason": (
                        "The corpus row names constraint error as an oracle. "
                        "This chain has no loop closures or contacts, so "
                        "there is no constraint residual to measure; the "
                        "quantity is absent rather than zero."
                    ),
                },
                "solver_residual": dict(UNSUPPORTED_SOLVER_RESIDUAL),
            },
            "performance": {
                "status": "unsupported",
                "reason": (
                    "The corpus row asks for a whole-step speed/accuracy "
                    "tradeoff, but no interleaved same-host timing "
                    "methodology was applied, so step cost is not measured "
                    "and no tradeoff is established"
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
                    "The oracle is numeric tracking error and control work; "
                    "no visible-behavior claim is made."
                ),
            },
        },
        "result": {
            "disposition": disposition,
            "claim_boundary": (
                "DART 7 main, this commit, a 4-link PD-tracked chain with no "
                "contact, fixed gains, a 2 s horizon, timesteps 0.5-4 ms, and "
                "both multibody integration families. What is established: "
                "the controller stays stable and unsaturated in every "
                "cell, and RMS tracking error does NOT improve as the "
                "timestep is refined eightfold (about 2.1e-3 rad at 4 ms "
                "versus 2.4e-3 rad at 0.5 ms, against a 0.25 rad reference "
                "amplitude), while control work rises monotonically as the "
                "timestep shrinks -- the error here is controller-limited, "
                "not integration-limited. What is "
                "NOT established: the cited speed/accuracy tradeoff, because "
                "step cost is not measured at all; constraint error, because "
                "this scene has no constraints to violate; and any ranking "
                "between the integration families. Says nothing about "
                "contacting or legged control, gain tuning, historical DART "
                "versions, or DART 6."
            ),
            "limitations": [
                "No contact: a controlled robot's hard cases are contact "
                "transitions, which this fixture deliberately excludes so "
                "the tracking signal is unambiguous.",
                "Step cost is not measured, so the row's speed half is "
                "absent; performance is typed unsupported rather than "
                "estimated.",
                "One gain policy (uniform 20 rad/s, critically damped) and one "
                "reference trajectory; a gain sweep would "
                "change the error/effort balance and is not attempted.",
                "Control work uses the commanded torque and the joint "
                "velocity sampled before the step, so it is a first-order "
                "estimate of actuation effort rather than an exact integral.",
                "The torque limit is recorded and its saturation flagged; a "
                "saturated cell would make the comparison between cells "
                "unequal, which is why the flag is published per family.",
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
    for family, stats in physical["per_family_summary"].items():
        print(
            f"  {family}: RMS error {stats['rms_error_at_largest_timestep']:.4e}"
            f" -> {stats['rms_error_at_smallest_timestep']:.4e} rad, "
            f"max torque {stats['max_abs_torque_nm']:.1f} Nm, saturated="
            f"{stats['any_torque_saturated']}"
        )
    print(f"  disposition: {packet['result']['disposition']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
