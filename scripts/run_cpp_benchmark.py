#!/usr/bin/env python3
"""Build and run a C++ benchmark with optional runtime arguments."""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
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
}


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
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
        dest="benchmark",
        help="Alias for benchmark target name (legacy pixi usage).",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type to use (default: Release)",
    )
    parser.add_argument(
        "--report",
        action="store_true",
        help="Write a JSON benchmark report under benchmarks/reports.",
    )
    parser.add_argument(
        "--report-dir",
        type=Path,
        default=Path("benchmarks") / "reports",
        help="Root directory for benchmark reports (default: benchmarks/reports).",
    )
    parser.add_argument(
        "--report-name",
        default=None,
        help="Override the report benchmark name (default: alias or target).",
    )
    parser.add_argument(
        "--runner-id",
        default=os.environ.get("DART_BENCH_RUNNER_ID", "local"),
        help="Runner identifier for reports (default: DART_BENCH_RUNNER_ID or 'local').",
    )
    parser.add_argument(
        "--pixi-help",
        action="store_true",
        help="Show this help for the pixi wrapper.",
    )
    known, unknown = parser.parse_known_args(argv)

    if known.pixi_help or known.benchmark is None:
        parser.print_help()
        _print_known_benchmarks()
        sys.exit(0)

    return known, unknown


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


def _report_slug(benchmark: str, target: str, override: str | None) -> str:
    if override:
        return _normalize_key(override)
    key = _normalize_key(benchmark)
    if key in CANONICAL_BENCHMARKS:
        return key
    slug = target.strip().lower().replace("/", "_")
    slug = re.sub(r"[^a-z0-9._-]+", "_", slug)
    return slug or "benchmark"


def _normalize_runner_id(value: str) -> str:
    if not value:
        return "local"
    safe = re.sub(r"[^a-zA-Z0-9._-]+", "_", value.strip())
    return safe or "local"


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
    ]

    for path in candidates:
        if path.exists():
            return path

    raise SystemExit(
        f"Binary not found for target '{target}'. "
        "Check the build output for the runtime directory."
    )


def _git_output(args: list[str], default: str = "unknown") -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError):
        return default
    value = result.stdout.strip()
    return value or default


def _report_path(root: Path, slug: str, runner_id: str, short_sha: str) -> Path:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%SZ")
    return root / slug / runner_id / f"{timestamp}-{short_sha}.json"


def _has_benchmark_out_args(run_args: list[str]) -> bool:
    for arg in run_args:
        if arg.startswith("--benchmark_out=") or arg.startswith("--benchmark_out_format="):
            return True
        if arg in {"--benchmark_out", "--benchmark_out_format"}:
            return True
    return False


def _annotate_report(path: Path, metadata: dict[str, str]) -> None:
    data = json.loads(path.read_text())
    context = data.setdefault("context", {})
    context.pop("host_name", None)
    context.pop("executable", None)
    context["dart_bench"] = metadata
    path.write_text(json.dumps(data, indent=2, sort_keys=True))


def run(
    benchmark: str,
    build_type: str,
    run_args: list[str],
    report: bool,
    report_dir: Path,
    report_name: str | None,
    runner_id: str,
) -> int:
    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_dir = Path("build") / env_name / "cpp" / build_type

    ensure_build_exists(build_dir, build_type)

    target = _resolve_target(benchmark)
    slug = _report_slug(benchmark, target, report_name)
    runner_id = _normalize_runner_id(runner_id)

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
    report_path = None
    if report:
        if _has_benchmark_out_args(run_args):
            raise SystemExit(
                "Report mode already manages --benchmark_out. "
                "Remove custom benchmark_out arguments."
            )
        report_dir = report_dir.resolve()
        short_sha = _git_output(["rev-parse", "--short", "HEAD"])
        report_path = _report_path(report_dir, slug, runner_id, short_sha)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        run_args = [
            *run_args,
            f"--benchmark_out={report_path}",
            "--benchmark_out_format=json",
        ]

    subprocess.run([str(binary), *run_args], check=True, env=env)
    if report_path:
        metadata = {
            "benchmark": slug,
            "target": target,
            "runner_id": runner_id,
            "git_sha": _git_output(["rev-parse", "HEAD"]),
            "git_branch": _git_output(["rev-parse", "--abbrev-ref", "HEAD"]),
            "build_type": build_type,
            "pixi_env": env_name,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        _annotate_report(report_path, metadata)
        print(f"Benchmark report: {report_path}")
    return 0


def main(argv: list[str]) -> int:
    args, run_args = parse_args(argv)
    return run(
        args.benchmark,
        args.build_type,
        run_args,
        report=args.report,
        report_dir=args.report_dir,
        report_name=args.report_name,
        runner_id=args.runner_id,
    )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
