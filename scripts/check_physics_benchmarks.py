#!/usr/bin/env python3
"""Check focused physics benchmark counters for accuracy/stability regressions."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class BenchmarkRun:
    target: str
    filter: str


BENCHMARK_RUNS = {
    "free_body": BenchmarkRun(
        target="bm_free_body_energy",
        filter=(
            "(WorldStep/10000/1000|"
            "PrincipalAxisWorldStep/10000/1000|"
            "GravityWorldStep/10000/1000|"
            "GravityRungeKutta4/10000/1000|"
            "GeneralizedForceWorldStep/10000/1000|"
            "AccelerationCommandWorldStep/10000/1000|"
            "DampedWorldStep/10000/1000|"
            "ForcedTorqueWorldStep/10000/1000)$"
        ),
    ),
    "free_joint_pose": BenchmarkRun(
        target="bm_free_joint_pose",
        filter="(WorldTangentIsometry/10000|BodyTwistIsometry/10000)$",
    ),
    "articulated": BenchmarkRun(
        target="bm_articulated_energy",
        filter=(
            "(RevoluteChainWorldStep/5/10000/1000|"
            "RevoluteChainSubsteppedWorldStep/5/10000/1000/5|"
            "RevoluteChainRungeKutta4/5/10000/1000|"
            "BallChainWorldStep/3/5000/1000|"
            "BallChainSubsteppedWorldStep/3/5000/1000/5|"
            "BallChainRungeKutta4/3/5000/1000|"
            "FloatingBaseChainWorldStep/3/5000/1000|"
            "FloatingBaseChainSubsteppedWorldStep/3/5000/1000/5|"
            "FloatingBaseChainRungeKutta4/3/5000/1000)$"
        ),
    ),
    "contact": BenchmarkRun(
        target="bm_contact_stability",
        filter=(
            "(ContactStackWorldStep/3/10000/1000|"
            "ContactStackSubsteppedWorldStep/3/10000/1000/5|"
            "FrictionHoldWorldStep/10000/1000|"
            "FrictionHoldSubsteppedWorldStep/10000/1000/5)$"
        ),
    ),
    "constraint": BenchmarkRun(
        target="bm_constraint_stability",
        filter=(
            "(BallConstraintWorldStep/10000/500|"
            "BallConstraintSubsteppedWorldStep/10000/500/5|"
            "JointLimitWorldStep/10000/500|"
            "JointLimitSubsteppedWorldStep/10000/500/5|"
            "ServoTrackingWorldStep/10000/1000|"
            "ServoTrackingSubsteppedWorldStep/10000/1000/5)$"
        ),
    ),
}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use (default: Release)",
    )
    parser.add_argument(
        "--min-time",
        default="0.01s",
        help="Google Benchmark minimum time per case (default: 0.01s)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(".benchmark_results") / "physics_check",
        help="Directory for benchmark JSON outputs",
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=None,
        help="Read existing benchmark JSON outputs instead of running them",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print passing checks as well as failures",
    )
    return parser.parse_args(argv)


def build_dir(build_type: str) -> Path:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    return Path("build") / env_name / "cpp" / build_type


def build_target(target: str, build_type: str) -> Path:
    directory = build_dir(build_type)
    if not directory.exists():
        raise SystemExit(
            f"Build directory {directory} not found. Run `pixi run config` first."
        )

    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(directory),
            "--target",
            target,
        ],
        check=True,
    )

    candidates = [
        directory / "bin" / target,
        directory / "tests" / "benchmark" / "dynamics" / target,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise SystemExit(f"Benchmark binary not found for target {target}")


def run_benchmark(
    name: str,
    run: BenchmarkRun,
    build_type: str,
    min_time: str,
    output_dir: Path,
) -> Path:
    binary = build_target(run.target, build_type)
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"{name}.json"

    subprocess.run(
        [
            str(binary),
            f"--benchmark_filter={run.filter}",
            f"--benchmark_min_time={min_time}",
            f"--benchmark_out={output}",
            "--benchmark_out_format=json",
        ],
        check=True,
    )
    return output


def load_entries(path: Path) -> dict[str, dict]:
    with open(path) as f:
        data = json.load(f)

    entries = {}
    for entry in data.get("benchmarks", []):
        if entry.get("run_type") == "aggregate":
            continue
        entries[entry["name"]] = entry
    return entries


def require_entry(entries: dict[str, dict], name: str) -> dict:
    if name not in entries:
        raise AssertionError(f"Missing benchmark entry: {name}")
    return entries[name]


def require_counter(entry: dict, counter: str) -> float:
    if counter not in entry:
        raise AssertionError(f"Missing counter {counter}: {entry['name']}")
    value = float(entry[counter])
    if not value == value:
        raise AssertionError(f"Counter is NaN: {entry['name']} {counter}")
    return value


def check_less_equal(
    failures: list[str],
    passes: list[str],
    label: str,
    value: float,
    threshold: float,
) -> None:
    message = f"{label}: {value:.6g} <= {threshold:.6g}"
    if value <= threshold:
        passes.append(message)
    else:
        failures.append(message)


def check_greater_equal(
    failures: list[str],
    passes: list[str],
    label: str,
    value: float,
    threshold: float,
) -> None:
    message = f"{label}: {value:.6g} >= {threshold:.6g}"
    if value >= threshold:
        passes.append(message)
    else:
        failures.append(message)


def check_ratio(
    failures: list[str],
    passes: list[str],
    label: str,
    numerator: float,
    denominator: float,
    threshold: float,
) -> None:
    if denominator <= 0.0:
        failures.append(f"{label}: denominator is not positive ({denominator})")
        return

    ratio = numerator / denominator
    message = f"{label}: ratio {ratio:.6g} <= {threshold:.6g}"
    if ratio <= threshold:
        passes.append(message)
    else:
        failures.append(message)


def check_free_body(entries: dict[str, dict]) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    passes: list[str] = []

    world = require_entry(entries, "BM_FreeBodyEnergy_WorldStep/10000/1000")
    principal = require_entry(
        entries, "BM_FreeBodyEnergy_PrincipalAxisWorldStep/10000/1000"
    )
    forced = require_entry(
        entries, "BM_FreeBodyEnergy_ForcedTorqueWorldStep/10000/1000"
    )
    gravity = require_entry(entries, "BM_FreeBodyEnergy_GravityWorldStep/10000/1000")
    gravity_runge_kutta = require_entry(
        entries, "BM_FreeBodyEnergy_GravityRungeKutta4/10000/1000"
    )
    generalized_force = require_entry(
        entries, "BM_FreeBodyEnergy_GeneralizedForceWorldStep/10000/1000"
    )
    acceleration_command = require_entry(
        entries, "BM_FreeBodyEnergy_AccelerationCommandWorldStep/10000/1000"
    )
    damped = require_entry(entries, "BM_FreeBodyEnergy_DampedWorldStep/10000/1000")

    check_less_equal(
        failures,
        passes,
        "free body max relative energy drift",
        require_counter(world, "max_rel_energy_drift"),
        1e-12,
    )
    check_less_equal(
        failures,
        passes,
        "free body world angular momentum vector drift",
        require_counter(world, "max_rel_world_ang_mom_vector_drift"),
        1e-12,
    )
    check_less_equal(
        failures,
        passes,
        "free body world linear momentum vector drift",
        require_counter(world, "max_rel_world_linear_mom_vector_drift"),
        1e-12,
    )
    check_less_equal(
        failures,
        passes,
        "principal-axis energy drift",
        require_counter(principal, "max_rel_energy_drift"),
        1e-14,
    )
    check_less_equal(
        failures,
        passes,
        "principal-axis angular momentum drift",
        require_counter(principal, "max_rel_ang_mom_drift"),
        1e-14,
    )
    check_greater_equal(
        failures,
        passes,
        "free body throughput",
        require_counter(world, "items_per_second"),
        1e4,
    )
    check_less_equal(
        failures,
        passes,
        "gravity free body velocity error",
        require_counter(gravity, "max_gravity_velocity_error"),
        1e-10,
    )
    check_less_equal(
        failures,
        passes,
        "gravity free body position error",
        require_counter(gravity, "max_gravity_position_error"),
        1e-8,
    )
    check_greater_equal(
        failures,
        passes,
        "gravity free body throughput",
        require_counter(gravity, "items_per_second"),
        1e4,
    )
    check_less_equal(
        failures,
        passes,
        "gravity Runge-Kutta free body velocity error",
        require_counter(gravity_runge_kutta, "max_gravity_velocity_error"),
        1e-10,
    )
    check_less_equal(
        failures,
        passes,
        "gravity Runge-Kutta free body position error",
        require_counter(gravity_runge_kutta, "max_gravity_position_error"),
        1e-10,
    )
    check_greater_equal(
        failures,
        passes,
        "gravity Runge-Kutta free body throughput",
        require_counter(gravity_runge_kutta, "items_per_second"),
        1e4,
    )
    check_less_equal(
        failures,
        passes,
        "generalized force free body velocity error",
        require_counter(generalized_force, "max_generalized_force_velocity_error"),
        1e-10,
    )
    check_less_equal(
        failures,
        passes,
        "generalized force free body position error",
        require_counter(generalized_force, "max_generalized_force_position_error"),
        1e-8,
    )
    check_greater_equal(
        failures,
        passes,
        "generalized force final energy gain",
        require_counter(generalized_force, "generalized_force_final_energy_gain"),
        1e-4,
    )
    check_greater_equal(
        failures,
        passes,
        "generalized force throughput",
        require_counter(generalized_force, "items_per_second"),
        1e4,
    )
    check_less_equal(
        failures,
        passes,
        "acceleration command free body velocity error",
        require_counter(
            acceleration_command, "max_acceleration_command_velocity_error"
        ),
        1e-10,
    )
    check_less_equal(
        failures,
        passes,
        "acceleration command free body position error",
        require_counter(
            acceleration_command, "max_acceleration_command_position_error"
        ),
        1e-8,
    )
    check_greater_equal(
        failures,
        passes,
        "acceleration command final energy gain",
        require_counter(acceleration_command, "acceleration_command_final_energy_gain"),
        1e-4,
    )
    check_greater_equal(
        failures,
        passes,
        "acceleration command throughput",
        require_counter(acceleration_command, "items_per_second"),
        1e4,
    )
    check_less_equal(
        failures,
        passes,
        "damped free body max energy increase",
        require_counter(damped, "max_energy_increase_per_step"),
        1e-12,
    )
    check_less_equal(
        failures,
        passes,
        "damped free body final energy ratio",
        require_counter(damped, "final_energy_ratio"),
        0.1,
    )
    check_greater_equal(
        failures,
        passes,
        "damped free body throughput",
        require_counter(damped, "items_per_second"),
        1e4,
    )
    check_less_equal(
        failures,
        passes,
        "forced torque angular impulse error",
        require_counter(forced, "max_rel_torque_impulse_error"),
        1e-10,
    )
    check_greater_equal(
        failures,
        passes,
        "forced torque per-step energy gain",
        require_counter(forced, "min_energy_gain_per_step"),
        1e-8,
    )
    check_greater_equal(
        failures,
        passes,
        "forced torque final energy gain",
        require_counter(forced, "final_energy_gain"),
        1e-4,
    )
    check_greater_equal(
        failures,
        passes,
        "forced torque throughput",
        require_counter(forced, "items_per_second"),
        1e4,
    )
    return failures, passes


def check_free_joint_pose(entries: dict[str, dict]) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    passes: list[str] = []

    world_tangent = require_entry(
        entries, "BM_FreeJointPose_WorldTangentIsometry/10000"
    )
    body_twist = require_entry(entries, "BM_FreeJointPose_BodyTwistIsometry/10000")

    check_less_equal(
        failures,
        passes,
        "world-tangent translation error",
        require_counter(world_tangent, "max_world_trans_err_m"),
        1e-14,
    )
    check_less_equal(
        failures,
        passes,
        "world-tangent rotation error",
        require_counter(world_tangent, "max_rot_err_rad"),
        1e-14,
    )
    check_less_equal(
        failures,
        passes,
        "body-twist translation error",
        require_counter(body_twist, "max_body_trans_err_m"),
        1e-14,
    )
    check_less_equal(
        failures,
        passes,
        "body-twist rotation error",
        require_counter(body_twist, "max_rot_err_rad"),
        1e-14,
    )
    check_greater_equal(
        failures,
        passes,
        "world-tangent pose throughput",
        require_counter(world_tangent, "items_per_second"),
        1e6,
    )
    check_greater_equal(
        failures,
        passes,
        "body-twist pose throughput",
        require_counter(body_twist, "items_per_second"),
        1e6,
    )
    return failures, passes


def check_articulated(entries: dict[str, dict]) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    passes: list[str] = []

    direct = require_entry(
        entries, "BM_ArticulatedEnergy_RevoluteChainWorldStep/5/10000/1000"
    )
    substepped = require_entry(
        entries,
        "BM_ArticulatedEnergy_RevoluteChainSubsteppedWorldStep/5/10000/1000/5",
    )
    runge_kutta = require_entry(
        entries, "BM_ArticulatedEnergy_RevoluteChainRungeKutta4/5/10000/1000"
    )
    ball_direct = require_entry(
        entries, "BM_ArticulatedEnergy_BallChainWorldStep/3/5000/1000"
    )
    ball_substepped = require_entry(
        entries, "BM_ArticulatedEnergy_BallChainSubsteppedWorldStep/3/5000/1000/5"
    )
    ball_runge_kutta = require_entry(
        entries, "BM_ArticulatedEnergy_BallChainRungeKutta4/3/5000/1000"
    )
    floating_direct = require_entry(
        entries, "BM_ArticulatedEnergy_FloatingBaseChainWorldStep/3/5000/1000"
    )
    floating_substepped = require_entry(
        entries,
        "BM_ArticulatedEnergy_FloatingBaseChainSubsteppedWorldStep/3/5000/1000/5",
    )
    floating_runge_kutta = require_entry(
        entries,
        "BM_ArticulatedEnergy_FloatingBaseChainRungeKutta4/3/5000/1000",
    )

    direct_drift = require_counter(direct, "max_rel_energy_drift")
    substep_drift = require_counter(substepped, "max_rel_energy_drift")
    runge_kutta_drift = require_counter(runge_kutta, "max_rel_energy_drift")
    ball_direct_drift = require_counter(ball_direct, "max_rel_energy_drift")
    ball_substep_drift = require_counter(ball_substepped, "max_rel_energy_drift")
    ball_runge_kutta_drift = require_counter(ball_runge_kutta, "max_rel_energy_drift")
    floating_direct_drift = require_counter(floating_direct, "max_rel_energy_drift")
    floating_substep_drift = require_counter(
        floating_substepped, "max_rel_energy_drift"
    )
    floating_runge_kutta_drift = require_counter(
        floating_runge_kutta, "max_rel_energy_drift"
    )
    check_ratio(
        failures,
        passes,
        "revolute-chain substep energy drift improvement",
        substep_drift,
        direct_drift,
        0.25,
    )
    check_less_equal(
        failures,
        passes,
        "revolute-chain substep energy drift",
        substep_drift,
        0.05,
    )
    check_greater_equal(
        failures,
        passes,
        "revolute-chain substep throughput",
        require_counter(substepped, "items_per_second"),
        5e4,
    )
    check_ratio(
        failures,
        passes,
        "revolute-chain Runge-Kutta energy drift improvement",
        runge_kutta_drift,
        substep_drift,
        0.01,
    )
    check_less_equal(
        failures,
        passes,
        "revolute-chain Runge-Kutta energy drift",
        runge_kutta_drift,
        2e-4,
    )
    check_greater_equal(
        failures,
        passes,
        "revolute-chain Runge-Kutta throughput",
        require_counter(runge_kutta, "items_per_second"),
        1e4,
    )
    check_greater_equal(
        failures,
        passes,
        "revolute-chain Runge-Kutta reference substeps",
        require_counter(runge_kutta, "reference_substeps"),
        10,
    )
    check_less_equal(
        failures,
        passes,
        "revolute-chain Runge-Kutta position RMS error vs reference",
        require_counter(runge_kutta, "position_rms_error_vs_ref"),
        1e-3,
    )
    check_less_equal(
        failures,
        passes,
        "revolute-chain Runge-Kutta velocity RMS error vs reference",
        require_counter(runge_kutta, "velocity_rms_error_vs_ref"),
        2e-2,
    )
    check_less_equal(
        failures,
        passes,
        "revolute-chain Runge-Kutta max position error vs reference",
        require_counter(runge_kutta, "max_abs_position_error_vs_ref"),
        2e-3,
    )
    check_less_equal(
        failures,
        passes,
        "revolute-chain Runge-Kutta max velocity error vs reference",
        require_counter(runge_kutta, "max_abs_velocity_error_vs_ref"),
        3e-2,
    )
    check_ratio(
        failures,
        passes,
        "ball-chain substep energy drift improvement",
        ball_substep_drift,
        ball_direct_drift,
        0.3,
    )
    check_less_equal(
        failures,
        passes,
        "ball-chain substep energy drift",
        ball_substep_drift,
        5e-4,
    )
    check_greater_equal(
        failures,
        passes,
        "ball-chain substep throughput",
        require_counter(ball_substepped, "items_per_second"),
        5e4,
    )
    check_ratio(
        failures,
        passes,
        "ball-chain Runge-Kutta energy drift improvement",
        ball_runge_kutta_drift,
        ball_substep_drift,
        1e-3,
    )
    check_less_equal(
        failures,
        passes,
        "ball-chain Runge-Kutta energy drift",
        ball_runge_kutta_drift,
        1e-7,
    )
    check_greater_equal(
        failures,
        passes,
        "ball-chain Runge-Kutta throughput",
        require_counter(ball_runge_kutta, "items_per_second"),
        1e4,
    )
    check_greater_equal(
        failures,
        passes,
        "ball-chain Runge-Kutta reference substeps",
        require_counter(ball_runge_kutta, "reference_substeps"),
        10,
    )
    check_less_equal(
        failures,
        passes,
        "ball-chain Runge-Kutta position RMS error vs reference",
        require_counter(ball_runge_kutta, "position_rms_error_vs_ref"),
        5e-6,
    )
    check_less_equal(
        failures,
        passes,
        "ball-chain Runge-Kutta velocity RMS error vs reference",
        require_counter(ball_runge_kutta, "velocity_rms_error_vs_ref"),
        5e-5,
    )
    check_less_equal(
        failures,
        passes,
        "ball-chain Runge-Kutta max position error vs reference",
        require_counter(ball_runge_kutta, "max_abs_position_error_vs_ref"),
        1e-5,
    )
    check_less_equal(
        failures,
        passes,
        "ball-chain Runge-Kutta max velocity error vs reference",
        require_counter(ball_runge_kutta, "max_abs_velocity_error_vs_ref"),
        1e-4,
    )
    check_ratio(
        failures,
        passes,
        "floating-base chain substep energy drift improvement",
        floating_substep_drift,
        floating_direct_drift,
        0.3,
    )
    check_less_equal(
        failures,
        passes,
        "floating-base chain substep energy drift",
        floating_substep_drift,
        0.01,
    )
    check_greater_equal(
        failures,
        passes,
        "floating-base chain substep throughput",
        require_counter(floating_substepped, "items_per_second"),
        5e4,
    )
    check_ratio(
        failures,
        passes,
        "floating-base chain Runge-Kutta energy drift improvement",
        floating_runge_kutta_drift,
        floating_substep_drift,
        1e-3,
    )
    check_less_equal(
        failures,
        passes,
        "floating-base chain Runge-Kutta energy drift",
        floating_runge_kutta_drift,
        1e-5,
    )
    check_greater_equal(
        failures,
        passes,
        "floating-base chain Runge-Kutta throughput",
        require_counter(floating_runge_kutta, "items_per_second"),
        1e4,
    )
    check_greater_equal(
        failures,
        passes,
        "floating-base chain Runge-Kutta reference substeps",
        require_counter(floating_runge_kutta, "reference_substeps"),
        10,
    )
    check_less_equal(
        failures,
        passes,
        "floating-base chain Runge-Kutta position RMS error vs reference",
        require_counter(floating_runge_kutta, "position_rms_error_vs_ref"),
        1e-5,
    )
    check_less_equal(
        failures,
        passes,
        "floating-base chain Runge-Kutta velocity RMS error vs reference",
        require_counter(floating_runge_kutta, "velocity_rms_error_vs_ref"),
        2e-5,
    )
    check_less_equal(
        failures,
        passes,
        "floating-base chain Runge-Kutta max position error vs reference",
        require_counter(floating_runge_kutta, "max_abs_position_error_vs_ref"),
        2e-5,
    )
    check_less_equal(
        failures,
        passes,
        "floating-base chain Runge-Kutta max velocity error vs reference",
        require_counter(floating_runge_kutta, "max_abs_velocity_error_vs_ref"),
        5e-5,
    )
    return failures, passes


def check_contact(entries: dict[str, dict]) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    passes: list[str] = []

    direct = require_entry(entries, "BM_ContactStackWorldStep/3/10000/1000")
    substepped = require_entry(
        entries, "BM_ContactStackSubsteppedWorldStep/3/10000/1000/5"
    )
    friction_direct = require_entry(entries, "BM_FrictionHoldWorldStep/10000/1000")
    friction_substepped = require_entry(
        entries, "BM_FrictionHoldSubsteppedWorldStep/10000/1000/5"
    )

    direct_penetration = require_counter(direct, "max_penetration_depth")
    substep_penetration = require_counter(substepped, "max_penetration_depth")
    direct_height_error = require_counter(direct, "max_top_height_error")
    substep_height_error = require_counter(substepped, "max_top_height_error")

    check_ratio(
        failures,
        passes,
        "contact substep penetration improvement",
        substep_penetration,
        direct_penetration,
        0.25,
    )
    check_ratio(
        failures,
        passes,
        "contact substep height-error improvement",
        substep_height_error,
        direct_height_error,
        0.25,
    )
    check_less_equal(
        failures,
        passes,
        "contact substep penetration",
        substep_penetration,
        2e-7,
    )
    check_less_equal(
        failures,
        passes,
        "contact substep height error",
        substep_height_error,
        5e-7,
    )
    check_greater_equal(
        failures,
        passes,
        "contact substep throughput",
        require_counter(substepped, "items_per_second"),
        2e3,
    )
    check_less_equal(
        failures,
        passes,
        "friction hold direct lateral drift",
        require_counter(friction_direct, "max_lateral_drift"),
        1e-5,
    )
    check_less_equal(
        failures,
        passes,
        "friction hold substep lateral drift",
        require_counter(friction_substepped, "max_lateral_drift"),
        1e-5,
    )
    check_ratio(
        failures,
        passes,
        "friction hold substep lateral-drift improvement",
        require_counter(friction_substepped, "max_lateral_drift"),
        require_counter(friction_direct, "max_lateral_drift"),
        0.5,
    )
    check_less_equal(
        failures,
        passes,
        "friction hold substep lateral speed",
        require_counter(friction_substepped, "max_lateral_speed"),
        1e-5,
    )
    check_less_equal(
        failures,
        passes,
        "friction hold substep penetration",
        require_counter(friction_substepped, "max_penetration_depth"),
        5e-6,
    )
    check_greater_equal(
        failures,
        passes,
        "friction hold substep throughput",
        require_counter(friction_substepped, "items_per_second"),
        2e3,
    )
    return failures, passes


def check_constraint(entries: dict[str, dict]) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    passes: list[str] = []

    direct = require_entry(entries, "BM_BallConstraintWorldStep/10000/500")
    substepped = require_entry(
        entries, "BM_BallConstraintSubsteppedWorldStep/10000/500/5"
    )
    joint_limit_direct = require_entry(entries, "BM_JointLimitWorldStep/10000/500")
    joint_limit_substepped = require_entry(
        entries, "BM_JointLimitSubsteppedWorldStep/10000/500/5"
    )
    servo_direct = require_entry(entries, "BM_ServoTrackingWorldStep/10000/1000")
    servo_substepped = require_entry(
        entries, "BM_ServoTrackingSubsteppedWorldStep/10000/1000/5"
    )

    direct_error = require_counter(direct, "final_anchor_error")
    substep_error = require_counter(substepped, "final_anchor_error")
    check_ratio(
        failures,
        passes,
        "ball constraint substep final-anchor improvement",
        substep_error,
        direct_error,
        0.01,
    )
    check_less_equal(
        failures,
        passes,
        "ball constraint substep final anchor error",
        substep_error,
        1e-5,
    )
    check_less_equal(
        failures,
        passes,
        "ball constraint substep max anchor error",
        require_counter(substepped, "max_anchor_error"),
        0.5,
    )
    check_greater_equal(
        failures,
        passes,
        "ball constraint substep throughput",
        require_counter(substepped, "items_per_second"),
        5e3,
    )
    check_greater_equal(
        failures,
        passes,
        "joint limit direct initial violation",
        require_counter(joint_limit_direct, "initial_limit_violation"),
        0.4,
    )
    check_less_equal(
        failures,
        passes,
        "joint limit direct final violation",
        require_counter(joint_limit_direct, "final_limit_violation"),
        1e-12,
    )
    check_less_equal(
        failures,
        passes,
        "joint limit substep final violation",
        require_counter(joint_limit_substepped, "final_limit_violation"),
        1e-12,
    )
    check_less_equal(
        failures,
        passes,
        "joint limit substep max velocity",
        require_counter(joint_limit_substepped, "max_abs_velocity"),
        10.1,
    )
    check_greater_equal(
        failures,
        passes,
        "joint limit substep throughput",
        require_counter(joint_limit_substepped, "items_per_second"),
        5e3,
    )
    check_greater_equal(
        failures,
        passes,
        "servo tracking direct command coverage",
        require_counter(servo_direct, "max_abs_command"),
        0.99,
    )
    check_less_equal(
        failures,
        passes,
        "servo tracking direct velocity error",
        require_counter(servo_direct, "max_servo_velocity_error"),
        1e-10,
    )
    check_less_equal(
        failures,
        passes,
        "servo tracking direct position error",
        require_counter(servo_direct, "max_servo_position_error"),
        1e-10,
    )
    check_less_equal(
        failures,
        passes,
        "servo tracking substep velocity error",
        require_counter(servo_substepped, "max_servo_velocity_error"),
        1e-10,
    )
    check_less_equal(
        failures,
        passes,
        "servo tracking substep position error",
        require_counter(servo_substepped, "max_servo_position_error"),
        1e-10,
    )
    check_greater_equal(
        failures,
        passes,
        "servo tracking substep throughput",
        require_counter(servo_substepped, "items_per_second"),
        5e3,
    )
    return failures, passes


CHECKS = {
    "free_body": check_free_body,
    "free_joint_pose": check_free_joint_pose,
    "articulated": check_articulated,
    "contact": check_contact,
    "constraint": check_constraint,
}


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    all_failures: list[str] = []
    all_passes: list[str] = []

    for name, run in BENCHMARK_RUNS.items():
        if args.input_dir:
            output = args.input_dir / f"{name}.json"
        else:
            output = run_benchmark(
                name, run, args.build_type, args.min_time, args.output_dir
            )

        entries = load_entries(output)
        failures, passes = CHECKS[name](entries)
        all_failures.extend(f"{name}: {failure}" for failure in failures)
        all_passes.extend(f"{name}: {passed}" for passed in passes)

    if args.verbose:
        for passed in all_passes:
            print(f"PASS {passed}")

    if all_failures:
        print("Physics benchmark checks failed:")
        for failure in all_failures:
            print(f"  FAIL {failure}")
        return 1

    print(f"All physics benchmark checks passed ({len(all_passes)} checks).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
