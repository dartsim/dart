#!/usr/bin/env python3
"""Build and run a C++ Google Benchmark target with optional runtime args."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

CANONICAL_BENCHMARKS = {
    "empty": "BM_INTEGRATION_empty",
    "boxes": "BM_INTEGRATION_boxes",
    "kinematics": "BM_INTEGRATION_kinematics",
    "dynamics": "BM_INTEGRATION_kinematics",
    "inverse_dynamics": "BM_INTEGRATION_inverse_dynamics",
    "inverse-dynamics": "BM_INTEGRATION_inverse_dynamics",
    # Added by PR #3209. Keeping the alias here lets the dashboard pick it up
    # automatically once that target lands on the DART 6 branch.
    "contact_container": "BM_INTEGRATION_contact_container",
    "contact-container": "BM_INTEGRATION_contact_container",
}

ALIASES = {
    **CANONICAL_BENCHMARKS,
    "bm_empty": "BM_INTEGRATION_empty",
    "bm_boxes": "BM_INTEGRATION_boxes",
    "bm_kinematics": "BM_INTEGRATION_kinematics",
    "bm_inverse_dynamics": "BM_INTEGRATION_inverse_dynamics",
    "bm_contact_container": "BM_INTEGRATION_contact_container",
}


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser_argv, passthrough_args = _split_passthrough(argv)
    parser = argparse.ArgumentParser(
        description=__doc__,
        add_help=False,  # Let --help pass through to the benchmark binary.
    )
    parser.add_argument(
        "benchmark",
        nargs="?",
        default=None,
        help="Benchmark alias or CMake target, for example boxes.",
    )
    parser.add_argument(
        "--target",
        dest="target",
        help="Benchmark alias or CMake target name.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use.",
    )
    parser.add_argument(
        "--pixi-help",
        action="store_true",
        help="Show this wrapper help instead of forwarding --help.",
    )
    known, unknown = parser.parse_known_args(parser_argv)
    if known.target is not None:
        known.benchmark = known.target

    if known.pixi_help or known.benchmark is None:
        parser.print_help()
        _print_known_benchmarks()
        sys.exit(0)

    return known, [*unknown, *passthrough_args]


def _split_passthrough(argv: list[str]) -> tuple[list[str], list[str]]:
    if "--" not in argv:
        return argv, []
    index = argv.index("--")
    return argv[:index], argv[index + 1 :]


def _print_known_benchmarks() -> None:
    print("\nCommon benchmarks (alias -> CMake target):")
    for alias, target in sorted(CANONICAL_BENCHMARKS.items()):
        print(f"  {alias} -> {target}")
    print("  (pass --help through to the benchmark after `--`)")


def _normalize_key(name: str) -> str:
    return name.strip().lower().replace("-", "_")


def _resolve_target(benchmark: str) -> str:
    key = _normalize_key(benchmark)
    return ALIASES.get(key, benchmark)


def _build_dir(build_type: str) -> Path:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    return Path("build") / env_name / "cpp" / build_type


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return

    raise SystemExit(
        f"Build directory {build_dir} does not exist.\n"
        f"Run `BUILD_TYPE={build_type} pixi run config` first, then retry."
    )


def _find_binary(build_dir: Path, target: str) -> Path:
    candidates = [
        build_dir / "bin" / target,
        build_dir / "tests" / "benchmark" / "integration" / target,
        build_dir / "tests" / "benchmark" / target,
    ]

    for path in candidates:
        if path.exists():
            return path

    raise SystemExit(
        f"Binary not found for target '{target}'. "
        "Check the build output for the runtime directory."
    )


def run(benchmark: str, build_type: str, run_args: list[str]) -> int:
    build_dir = _build_dir(build_type)
    ensure_build_exists(build_dir, build_type)

    target = _resolve_target(benchmark)
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build_dir),
            "--target",
            target,
            "--parallel",
        ],
        check=True,
    )

    binary = _find_binary(build_dir, target)
    subprocess.run([str(binary), *run_args], check=True)
    return 0


def main(argv: list[str]) -> int:
    args, run_args = parse_args(argv)
    return run(args.benchmark, args.build_type, run_args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
