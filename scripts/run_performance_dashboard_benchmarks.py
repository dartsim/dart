#!/usr/bin/env python3
"""Run the bounded DART 6 benchmark slice for the static dashboard."""

from __future__ import annotations

import argparse
import json
import os
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
    optional: bool = False


BENCHMARK_SPECS = [
    BenchmarkSpec(
        surface="empty",
        target="BM_INTEGRATION_empty",
        benchmark_filter="BM_Empty$",
        output_name="dashboard_empty.json",
    ),
    BenchmarkSpec(
        surface="kinematics-dynamics",
        target="BM_INTEGRATION_kinematics",
        benchmark_filter="BM_(Kinematics|Dynamics)/(1|10|100)$",
        output_name="dashboard_kinematics_dynamics.json",
    ),
    BenchmarkSpec(
        surface="inverse-dynamics",
        target="BM_INTEGRATION_inverse_dynamics",
        benchmark_filter=(
            "BM_InverseDynamics/(10|20|40)$|"
            "BM_InverseDynamicsViaMassMatrix/(10|20|40)$|"
            "BM_ContactInverseDynamics/(2|4|8)$|"
            "BM_ContactInverseDynamicsBasis/(4|8|16)$"
        ),
        output_name="dashboard_inverse_dynamics.json",
    ),
    BenchmarkSpec(
        surface="boxes",
        target="BM_INTEGRATION_boxes",
        benchmark_filter="BM_RunBoxes/(2|4|8)$",
        output_name="dashboard_boxes.json",
    ),
    # PR #3209 adds this target. Treat it as optional so the dashboard can land
    # before that PR while automatically publishing the richer contact rows once
    # the benchmark exists on release-6.20.
    BenchmarkSpec(
        surface="contact-container",
        target="BM_INTEGRATION_contact_container",
        benchmark_filter="BM_ContactContainerActive/.*",
        output_name="dashboard_contact_container.json",
        optional=True,
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


def _build_dir(build_type: str) -> Path:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    return Path("build") / env_name / "cpp" / build_type


def _target_declared(target: str, build_type: str) -> bool:
    build_dir = _build_dir(build_type)
    if not build_dir.exists():
        return True

    try:
        result = subprocess.run(
            ["cmake", "--build", str(build_dir), "--target", "help"],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError:
        return True

    return target in result.stdout or target in result.stderr


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

    required_failures: list[tuple[BenchmarkSpec, str]] = []
    optional_failures: list[tuple[BenchmarkSpec, str]] = []
    for spec in specs:
        if (
            spec.optional
            and not args.dry_run
            and not _target_declared(spec.target, args.build_type)
        ):
            print(
                f"Skipping optional {spec.surface} benchmark target "
                f"{spec.target}: target is not configured."
            )
            continue

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
            failure = (spec, f"exit code {exc.returncode}")
            if spec.optional:
                optional_failures.append(failure)
                continue
            required_failures.append(failure)
            if not args.continue_on_error:
                raise
            continue

        if not _has_benchmark_rows(output):
            message = f"no benchmark rows in {output}"
            failure = (spec, message)
            if spec.optional:
                optional_failures.append(failure)
                continue
            required_failures.append(failure)
            if not args.continue_on_error:
                raise SystemExit(
                    f"{spec.surface} benchmark target {spec.target} failed: "
                    f"{message}."
                )

    for spec, reason in optional_failures:
        print(
            f"Skipping optional {spec.surface} benchmark target {spec.target}: "
            f"{reason}.",
            file=sys.stderr,
        )

    if required_failures:
        for spec, reason in required_failures:
            print(
                f"{spec.surface} benchmark target {spec.target} failed "
                f"with {reason}.",
                file=sys.stderr,
            )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
