#!/usr/bin/env python3
"""Build and run a C++ benchmark with optional runtime arguments."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

CANONICAL_BENCHMARKS = {
    "boxes": "bm_boxes",
    "kinematics": "bm_kinematics",
    "lcp_compare": "BM_LCP_COMPARE",
    "lcp_solver": "BM_LCPSOLVER",
    "lcp_solver_solvers": "BM_LCPSOLVER_SOLVERS",
    "row_swapping": "BM_ROW_SWAPPING",
    "dot_product": "BM_DOT_PRODUCT",
    "matrix_multiply": "BM_MATRIX_MULTIPLY",
    "simd": "bm_simd",
    "dynamics_cache": "bm_dynamics_cache",
    "dynamics_cache_io": "bm_dynamics_cache_io",
    "allocators": "bm_allocators",
    "allocators_comparative": "bm_allocators_comparative",
    "allocators-comparative": "bm_allocators_comparative",
    "helpers": "bm_helpers",
    "math_helpers": "bm_helpers",
    "spatial_algebra": "bm_spatial_algebra",
    "jacobian": "bm_jacobian",
}

ALIASES = {
    **CANONICAL_BENCHMARKS,
    "bm_boxes": "bm_boxes",
    "bm_kinematics": "bm_kinematics",
    "bm_lcp_compare": "BM_LCP_COMPARE",
    "bm_lcpsolver": "BM_LCPSOLVER",
    "bm_lcpsolver_solvers": "BM_LCPSOLVER_SOLVERS",
    "bm_row_swapping": "BM_ROW_SWAPPING",
    "bm_dot_product": "BM_DOT_PRODUCT",
    "bm_matrix_multiply": "BM_MATRIX_MULTIPLY",
    "lcpsolver": "BM_LCPSOLVER",
    "lcp_solvers": "BM_LCPSOLVER_SOLVERS",
    "bm_simd": "bm_simd",
    "bm_dynamics_cache": "bm_dynamics_cache",
    "bm_dynamics_cache_io": "bm_dynamics_cache_io",
    "bm_allocators": "bm_allocators",
    "bm_allocators_comparative": "bm_allocators_comparative",
    "bm_helpers": "bm_helpers",
    "bm_spatial_algebra": "bm_spatial_algebra",
    "bm_jacobian": "bm_jacobian",
}


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser_argv, passthrough_args = _split_passthrough(argv)
    parser = argparse.ArgumentParser(
        description=__doc__,
        add_help=False,  # Allow --help to pass through to the benchmark.
    )
    parser.add_argument(
        "benchmark",
        nargs="?",
        default=None,
        help="Benchmark alias or CMake target (e.g., lcp_compare or BM_LCP_COMPARE)",
    )
    parser.add_argument(
        "--target",
        dest="target",
        help="Alias for benchmark target name (legacy pixi usage).",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use (default: Release)",
    )
    parser.add_argument(
        "--cpu-affinity",
        default=None,
        help=(
            "Pin only the benchmark binary to one CPU. Pass a CPU index or "
            "'auto' to choose a nonzero allowed CPU when possible."
        ),
    )
    parser.add_argument(
        "--pixi-help",
        action="store_true",
        help="Show this help for the pixi wrapper.",
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


def _resolve_cpu_affinity(cpu_affinity: str | None) -> int | None:
    if cpu_affinity is None:
        return None

    if not hasattr(os, "sched_getaffinity"):
        raise SystemExit("--cpu-affinity requires os.sched_getaffinity support")

    allowed = sorted(os.sched_getaffinity(0))
    if not allowed:
        raise SystemExit("no CPUs are available for --cpu-affinity")

    if cpu_affinity == "auto":
        return _choose_auto_cpu(allowed)

    try:
        cpu = int(cpu_affinity)
    except ValueError as exc:
        raise SystemExit(
            "--cpu-affinity must be an integer CPU index or 'auto'"
        ) from exc

    if cpu not in allowed:
        allowed_text = ",".join(str(value) for value in allowed)
        raise SystemExit(f"CPU {cpu} is not in the allowed CPU set: {allowed_text}")

    return cpu


def _read_cpu_times() -> dict[int, tuple[int, int]]:
    try:
        lines = Path("/proc/stat").read_text(encoding="utf-8").splitlines()
    except OSError:
        return {}

    times = {}
    for line in lines:
        fields = line.split()
        name = fields[0] if fields else ""
        if not name.startswith("cpu") or not name[3:].isdigit():
            continue
        values = [int(value) for value in fields[1:]]
        if len(values) < 5:
            continue
        idle = values[3] + values[4]
        total = sum(values)
        times[int(name[3:])] = (idle, total)
    return times


def _sample_cpu_utilization(
    allowed: list[int], interval_seconds: float = 0.20
) -> dict[int, float]:
    before = _read_cpu_times()
    if not before:
        return {}

    time.sleep(interval_seconds)
    after = _read_cpu_times()
    utilizations = {}
    for cpu in allowed:
        if cpu not in before or cpu not in after:
            continue
        idle_before, total_before = before[cpu]
        idle_after, total_after = after[cpu]
        total_delta = total_after - total_before
        if total_delta <= 0:
            continue
        idle_delta = idle_after - idle_before
        busy = max(0, total_delta - idle_delta)
        utilizations[cpu] = busy / total_delta
    return utilizations


def _read_thread_siblings(cpu: int) -> set[int]:
    path = (
        Path("/sys/devices/system/cpu")
        / f"cpu{cpu}"
        / "topology"
        / ("thread_siblings_list")
    )
    try:
        text = path.read_text(encoding="utf-8").strip()
    except OSError:
        return {cpu}

    siblings: set[int] = set()
    for part in text.split(","):
        if "-" in part:
            start, end = part.split("-", 1)
            try:
                siblings.update(range(int(start), int(end) + 1))
            except ValueError:
                continue
        else:
            try:
                siblings.add(int(part))
            except ValueError:
                continue
    return siblings or {cpu}


def _read_cpu_max_frequency(cpu: int) -> int:
    base = Path("/sys/devices/system/cpu") / f"cpu{cpu}" / "cpufreq"
    for name in ("cpuinfo_max_freq", "scaling_max_freq"):
        try:
            return int((base / name).read_text(encoding="utf-8").strip())
        except OSError, ValueError:
            continue
    return 0


def _choose_auto_cpu(allowed: list[int]) -> int:
    candidates = [cpu for cpu in allowed if cpu != 0] or allowed
    utilizations = _sample_cpu_utilization(candidates)
    if not utilizations:
        return allowed[min(2, len(allowed) - 1)]

    frequencies = {cpu: _read_cpu_max_frequency(cpu) for cpu in candidates}
    allowed_set = set(allowed)
    sibling_utilizations = {
        cpu: max(
            utilizations.get(sibling, 1.0)
            for sibling in (_read_thread_siblings(cpu) & allowed_set) or {cpu}
        )
        for cpu in candidates
    }
    quiet_candidates = [
        cpu for cpu in candidates if sibling_utilizations.get(cpu, 1.0) <= 0.25
    ]
    if quiet_candidates:
        return min(
            quiet_candidates,
            key=lambda cpu: (
                sibling_utilizations[cpu],
                utilizations.get(cpu, 1.0),
                -frequencies.get(cpu, 0),
                cpu,
            ),
        )

    return min(
        candidates,
        key=lambda cpu: (
            sibling_utilizations.get(cpu, 1.0),
            utilizations.get(cpu, 1.0),
            -frequencies.get(cpu, 0),
            cpu,
        ),
    )


def _cpu_affinity_preexec(cpu: int | None):
    if cpu is None:
        return None

    def apply_affinity() -> None:
        os.sched_setaffinity(0, {cpu})

    return apply_affinity


def _prewarm_cpu_affinity(cpu: int | None, seconds: float = 1.5) -> int | None:
    if cpu is None or not hasattr(os, "sched_setaffinity"):
        return None

    original_affinity = os.sched_getaffinity(0)
    try:
        os.sched_setaffinity(0, {cpu})
        end_time = time.perf_counter() + seconds
        value = 0
        while time.perf_counter() < end_time:
            value = (value * 1664525 + 1013904223) & 0xFFFFFFFF
        return value
    finally:
        os.sched_setaffinity(0, original_affinity)


def ensure_build_exists(build_dir: Path, build_type: str) -> None:
    if build_dir.exists():
        return

    msg = (
        f"Build directory {build_dir} does not exist.\n"
        f"Run `pixi run config --build_type {build_type}` first, "
        "then re-run this command."
    )
    raise SystemExit(msg)


def _find_binary(build_dir: Path, target: str) -> Path:
    candidates = [
        build_dir / "bin" / target,
        build_dir / "tests" / "benchmark" / target,
        build_dir / "tests" / "benchmark" / "lcpsolver" / target,
        build_dir / "tests" / "benchmark" / "collision" / target,
        build_dir / "tests" / "benchmark" / "dynamics" / target,
        build_dir / "tests" / "benchmark" / "integration" / target,
        build_dir / "tests" / "benchmark" / "unit" / target,
        build_dir / "tests" / "benchmark" / "simd" / target,
        build_dir / "tests" / "benchmark" / "common" / target,
        build_dir / "tests" / "benchmark" / "math" / target,
        build_dir / "tests" / "benchmark" / "simulation" / "experimental" / target,
    ]

    for path in candidates:
        if path.exists():
            return path

    raise SystemExit(
        f"Binary not found for target '{target}'. "
        "Check the build output for the runtime directory."
    )


def run(
    benchmark: str,
    build_type: str,
    run_args: list[str],
    *,
    cpu_affinity: str | None = None,
) -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type

    ensure_build_exists(build_dir, build_type)

    target = _resolve_target(benchmark)

    if target.startswith("SPECIAL:"):
        task_name = target[8:]
        print(f"This benchmark requires special configuration.")
        print(f"Please run: pixi run {task_name}")
        return 1

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
            target,
        ],
        check=True,
        env=env,
    )

    binary = _find_binary(build_dir, target)
    cpu = _resolve_cpu_affinity(cpu_affinity)
    if cpu is not None:
        print(f"Pinning benchmark binary to CPU {cpu}")
        _prewarm_cpu_affinity(cpu)
    subprocess.run(
        [str(binary), *run_args],
        check=True,
        env=env,
        preexec_fn=_cpu_affinity_preexec(cpu),
    )
    return 0


def main(argv: list[str]) -> int:
    args, run_args = parse_args(argv)
    run_args = [arg for arg in run_args if arg != "--"]
    return run(
        args.benchmark,
        args.build_type,
        run_args,
        cpu_affinity=args.cpu_affinity,
    )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
