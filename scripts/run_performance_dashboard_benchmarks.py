#!/usr/bin/env python3
"""Run a bounded benchmark slice for the static performance dashboard."""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class BenchmarkSpec:
    surface: str
    target: str
    benchmark_filter: str
    output_name: str


# The dashboard is intentionally scoped to DART 7 World performance. Keep
# the published history focused on end-to-end DART 7 World update/step
# cases for the new DART 7 simulation APIs (the DART 7 World and its
# solver families) instead of mixing in unrelated SIMD or robot-loader surfaces.
#
# Each surface tracks the *end-to-end World step* of a DART 7 solver family so
# the headline charts stay comparable. Internal micro-kernels (distance, barrier,
# tangent-stencil, candidate-set, etc.) and CUDA/GPU-only or DART_BUILD_DIFF-gated
# rows are deliberately excluded: they either need hardware the GitHub-hosted
# runner lacks or a build flag the dashboard build does not set, and they would
# bury the World-step throughput signal. Filters are bounded so a run stays cheap.
BENCHMARK_SPECS = [
    # Core DART 7 World step & scaling (kinematics, sequential/parallel
    # World step, rigid-body step scaling, the contact-shaped/contact-island
    # scalable-compute proxies, and the Phase 5 CPU baseline row).
    BenchmarkSpec(
        surface="world",
        target="bm_compute_graph",
        benchmark_filter=(
            "BM_WorldUpdateKinematics/.*|"
            "BM_WorldStep(Sequential|Parallel)/.*|"
            "BM_RigidBodyStep(Sequential|Parallel)/.*|"
            "BM_ContactShaped(Sequential|Parallel)/.*|"
            "BM_ContactIslandShaped(Sequential|Parallel)/.*|"
            "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10"
        ),
        output_name="dashboard_world.json",
    ),
    # Rigid-body dynamics solver (PLAN-080 / PLAN-082): end-to-end World step
    # with the default sequential-impulse contact solve and the opt-in IPC
    # barrier solve, over a small stacked-box scene.
    BenchmarkSpec(
        surface="rigid-world",
        target="bm_rigid_ipc_solver",
        benchmark_filter="BM_RigidWorldStep_(SequentialImpulse|Ipc)/.*",
        output_name="dashboard_rigid_world.json",
    ),
    # Deformable Vertex Block Descent solver (PLAN-104): end-to-end World step
    # of a square deformable grid with the default gradient-descent solver and
    # the VBD solver.
    BenchmarkSpec(
        surface="vbd-world",
        target="bm_vbd_world_solver",
        benchmark_filter="BM_VbdWorldStep(Default|Vbd)/.*",
        output_name="dashboard_vbd_world.json",
    ),
    # Deformable FEM solver (PLAN-081): end-to-end World step of a neo-Hookean
    # FEM beam under gravity via the sparse projected-Newton solve.
    BenchmarkSpec(
        surface="deformable-world",
        target="bm_deformable_body",
        benchmark_filter="BM_DeformableFemBarStep/.*",
        output_name="dashboard_deformable_world.json",
    ),
    # Augmented VBD (PLAN-104): end-to-end World step of the empty and first
    # non-empty 2D source-demo baselines plus 2D/3D dynamic/static friction,
    # 3D ground, stacking, and breakable source rows and narrow public rigid
    # fixed/revolute/prismatic/breakable/spherical rows and articulated
    # revolute/prismatic/breakable motor, same-multibody/world-anchored one-DOF
    # breakable motors, and breakable rows routed through the AVBD projection
    # paths, with narrow and paper-scale high mass-ratio articulated-chain
    # smoke rows plus the paper-scale high-ratio iteration-count sweep.
    BenchmarkSpec(
        surface="avbd-world",
        target="bm_avbd_rigid_fixed_joint",
        benchmark_filter=(
            "BM_AvbdEmptyWorldStep$|"
            "BM_AvbdDemo2dMotorStep$|"
            "BM_AvbdDemo2dHangingRopeStep$|"
            "BM_AvbdDemo2dFractureStep$|"
            "BM_AvbdDemo2dGroundStep$|"
            "BM_AvbdDemo2dDynamicFrictionStep$|"
            "BM_AvbdDemo2dFrictionCoefficientSweep/.*|"
            "BM_AvbdDemo2dStaticFrictionStep$|"
            "BM_AvbdDemo2dPyramidStep$|"
            "BM_AvbdDemo2dCardsStep$|"
            "BM_AvbdDemo2dStackStep$|"
            "BM_AvbdDemo2dStackRatioStep$|"
            "BM_AvbdDemo2dRodStep$|"
            "BM_AvbdDemo2dSoftBodyStep$|"
            "BM_AvbdDemo2dJointGridStep$|"
            "BM_AvbdDemo2dRopeStep$|"
            "BM_AvbdDemo2dHeavyRopeStep$|"
            "BM_AvbdDemo2dSpringStep$|"
            "BM_AvbdDemo2dSpringRatioStep$|"
            "BM_AvbdDemo2dNetStep$|"
            "BM_AvbdDemo3dGroundStep$|"
            "BM_AvbdDemo3dDynamicFrictionStep$|"
            "BM_AvbdDemo3dStaticFrictionStep$|"
            "BM_AvbdDemo3dPyramidStep$|"
            "BM_AvbdDemo3dRopeStep$|"
            "BM_AvbdDemo3dHeavyRopeStep$|"
            "BM_AvbdDemo3dSpringStep$|"
            "BM_AvbdDemo3dSpringRatioStep$|"
            "BM_AvbdDemo3dStackStep$|"
            "BM_AvbdDemo3dStackRatioStep$|"
            "BM_AvbdDemo3dSoftBodyStep$|"
            "BM_AvbdDemo3dBridgeStep$|"
            "BM_AvbdDemo3dBreakableStep$|"
            "BM_AvbdArticulatedHighRatioChainStep$|"
            "BM_AvbdPaperScaleHighRatioChainStep$|"
            "BM_AvbdPaperScaleHighRatioChainIterationSweep/.*|"
            "BM_Avbd(Rigid(FixedJoint|RevoluteMotor|PrismaticMotor|BreakableJoint"
            "|SphericalBreakableJoint)"
            "|Articulated((Revolute|World(Revolute|Prismatic)Breakable"
            "|PrismaticBreakable|Prismatic|Breakable)Motor"
            "|BreakableJoint|WorldSphericalBreakableJoint"
            "|SphericalPairBreakableJoint))Step/.*"
        ),
        output_name="dashboard_avbd_world.json",
    ),
]


def parse_args(argv: list[str]) -> argparse.Namespace:
    surfaces = sorted({spec.surface for spec in BENCHMARK_SPECS})
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--surface",
        action="append",
        choices=["all", *surfaces],
        default=[],
        help="Benchmark surface to run. May be passed more than once.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(".benchmark_results"),
        help="Directory for Google Benchmark JSON outputs.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type passed to scripts/run_cpp_benchmark.py.",
    )
    parser.add_argument(
        "--benchmark-min-time",
        default="1ms",
        help="Google Benchmark minimum time for each selected row.",
    )
    parser.add_argument(
        "--benchmark-repetitions",
        default="3",
        help="Google Benchmark repetition count for the dashboard slice.",
    )
    parser.add_argument(
        "--continue-on-error",
        action="store_true",
        help="Try later surfaces even if an earlier benchmark fails.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print benchmark commands without executing them.",
    )
    return parser.parse_args(argv)


def _selected_specs(surfaces: list[str]) -> list[BenchmarkSpec]:
    selected = surfaces or ["all"]
    if "all" in selected:
        return BENCHMARK_SPECS

    wanted = set(selected)
    return [spec for spec in BENCHMARK_SPECS if spec.surface in wanted]


def _benchmark_command(
    spec: BenchmarkSpec,
    output_dir: Path,
    build_type: str,
    min_time: str,
    repetitions: str,
) -> list[str]:
    output = output_dir / spec.output_name
    return [
        sys.executable,
        "scripts/run_cpp_benchmark.py",
        "--target",
        spec.target,
        "--build-type",
        build_type,
        "--",
        f"--benchmark_filter={spec.benchmark_filter}",
        f"--benchmark_out={output.as_posix()}",
        "--benchmark_out_format=json",
        f"--benchmark_min_time={min_time}",
        f"--benchmark_repetitions={repetitions}",
    ]


def _format_command(command: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def _has_benchmark_rows(output: Path) -> bool:
    if not output.is_file() or output.stat().st_size == 0:
        return False

    try:
        data = json.loads(output.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return False

    return bool(data.get("benchmarks"))


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    specs = _selected_specs(args.surface)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    failures: list[tuple[BenchmarkSpec, str]] = []
    for spec in specs:
        output = args.output_dir / spec.output_name
        command = _benchmark_command(
            spec=spec,
            output_dir=args.output_dir,
            build_type=args.build_type,
            min_time=args.benchmark_min_time,
            repetitions=args.benchmark_repetitions,
        )
        if args.dry_run:
            print(_format_command(command))
            continue

        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as exc:
            failures.append((spec, f"exit code {exc.returncode}"))
            if not args.continue_on_error:
                raise
            continue

        if not _has_benchmark_rows(output):
            message = f"no benchmark rows in {output}"
            failures.append((spec, message))
            if not args.continue_on_error:
                raise SystemExit(
                    f"{spec.surface} benchmark target {spec.target} failed: {message}."
                )

    if failures:
        for spec, reason in failures:
            print(
                f"{spec.surface} benchmark target {spec.target} failed "
                f"with {reason}.",
                file=sys.stderr,
            )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
