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


BENCHMARK_SPECS = [
    BenchmarkSpec(
        surface="common",
        target="allocators",
        benchmark_filter="BM_SingleAlloc_(Std|Pool)/(8|128)/(1|64)(/.*)?$",
        output_name="dashboard_common_allocators.json",
    ),
    BenchmarkSpec(
        surface="dynamics",
        target="kinematics",
        benchmark_filter="BM_(Kinematics|Dynamics)/(1|10)$",
        output_name="dashboard_dynamics_kinematics.json",
    ),
    BenchmarkSpec(
        surface="dynamics",
        target="dynamics_cache_io",
        benchmark_filter=("BM_Robot_(KR5|Atlas)_Forward(Kinematics|Dynamics)(/.*)?$"),
        output_name="dashboard_dynamics_robot_models.json",
    ),
    BenchmarkSpec(
        surface="lcp",
        target="lcp_compare",
        benchmark_filter="BM_LCP_COMPARE_SMOKE$",
        output_name="dashboard_lcp_compare.json",
    ),
    BenchmarkSpec(
        surface="math",
        target="helpers",
        benchmark_filter="BM_isNan_(Baseline|Optimized)/(3|4)(/.*)?$",
        output_name="dashboard_math_helpers.json",
    ),
    BenchmarkSpec(
        surface="simd",
        target="simd",
        benchmark_filter="BM_Add_DART_f32(_Baseline)?/1024(/.*)?$",
        output_name="dashboard_simd_add.json",
    ),
    BenchmarkSpec(
        surface="simulation",
        target="bm_compute_graph",
        benchmark_filter="BM_WorldStep(Sequential|Taskflow)/32/8(/.*)?$",
        output_name="dashboard_simulation_world_step.json",
    ),
    BenchmarkSpec(
        surface="compute",
        target="bm_compute_graph",
        benchmark_filter="BM_ComputeGraph(Sequential|Taskflow)/1024/32(/.*)?$",
        output_name="dashboard_compute_graph.json",
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
