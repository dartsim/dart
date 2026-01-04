#!/usr/bin/env python3
"""Build and run a C++ example with optional runtime arguments."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

_RENAMED_EXAMPLES = {
    "add_delete_skels": "viz_add_delete_skels",
    "atlas_simbicon": "control_walking_humanoid",
    "biped_stand": "control_balance_biped",
    "box_stacking": "collision_box_stacking",
    "capsule_ground_contact": "collision_capsule_ground_contact",
    "drag_and_drop": "viz_drag_and_drop",
    "empty": "viz_empty",
    "fetch": "model_fetch",
    "heightmap": "collision_heightmap",
    "headless_simulation": "perf_headless_simulation",
    "hybrid_dynamics": "control_actuator_modes",
    "imgui": "viz_imgui",
    "mixed_chain": "hybrid_mixed_chain",
    "operational_space_control": "control_operational_space",
    "polyhedron_visual": "viz_polyhedron_visual",
    "rigid_shapes": "collision_rigid_shapes",
    "csv_logger": "tool_csv_logger",
    "unified_loading": "io_unified_loading",
    "vehicle": "control_vehicle",
    "wam_ikfast": "ik_analytic_wam",
}


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description=__doc__,
        add_help=False,  # Allow --help to pass through to the example.
    )
    parser.add_argument(
        "target",
        nargs="?",
        default=None,
        help="CMake target / example binary name (e.g., control_walking_humanoid)",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use (default: Release)",
    )
    parser.add_argument(
        "--pixi-help",
        action="store_true",
        help="Show this help for the pixi wrapper.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available examples and exit.",
    )
    known, unknown = parser.parse_known_args(argv)

    if known.list:
        _print_example_list()
        sys.exit(0)

    if known.pixi_help or known.target is None:
        parser.print_help()
        sys.exit(0)

    return known, unknown


def _normalize_target(target: str) -> str:
    renamed = _RENAMED_EXAMPLES.get(target)
    if renamed:
        print(
            f"NOTE: example `{target}` was renamed to `{renamed}`.",
            file=sys.stderr,
        )
        return renamed
    return target


def _list_examples(root: Path = Path("examples")) -> list[str]:
    if not root.is_dir():
        return []
    examples = []
    for entry in sorted(root.iterdir()):
        if not entry.is_dir():
            continue
        if (entry / "CMakeLists.txt").is_file():
            examples.append(entry.name)
    return examples


def _print_example_list() -> None:
    examples = _list_examples()
    if not examples:
        print("No examples found.")
        return
    print("\n".join(examples))


def _resolve_build_and_binary(target: str) -> tuple[str, str]:
    return target, target


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return

    msg = (
        f"Build directory {build_dir} does not exist.\n"
        f"Run `pixi run config --build_type {build_type}` first, "
        "then re-run this command."
    )
    raise SystemExit(msg)


def run(target: str, build_type: str, run_args: list[str]) -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type

    ensure_build_exists(build_dir, build_type)

    target = _normalize_target(target)
    build_target, binary_name = _resolve_build_and_binary(target)

    env = os.environ.copy()
    env["BUILD_TYPE"] = build_type
    env["CMAKE_BUILD_DIR"] = str(build_dir)

    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(build_dir),
            "--target",
            build_target,
        ],
        check=True,
        env=env,
    )

    binary = build_dir / "bin" / binary_name
    if not binary.exists():
        raise SystemExit(f"Binary not found: {binary}")

    cmd = [str(binary), *run_args]
    subprocess.run(cmd, check=True, env=env)
    return 0


def main(argv: list[str]) -> int:
    args, run_args = parse_args(argv)
    return run(args.target, args.build_type, run_args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
