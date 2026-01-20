#!/usr/bin/env python3
"""Build and run the collision pipeline breakdown benchmark, capture JSON, and append a summary."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import date
from pathlib import Path

TARGET = "bm_scenarios_pipeline_breakdown"
RESULTS_DIR = Path("docs/dev_tasks/experimental_collision/results")
SUMMARY_PATH = Path("docs/dev_tasks/experimental_collision/benchmark_results.md")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="CMake build directory (defaults to pixi build or ./build)",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="Build type when using pixi build dir (default: Release)",
    )
    parser.add_argument(
        "--no-doc-update",
        action="store_true",
        help="Do not append summary to benchmark_results.md",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Path to JSON output (defaults to docs/dev_tasks/experimental_collision/results/)",
    )
    parser.add_argument(
        "benchmark_args",
        nargs=argparse.REMAINDER,
        help="Arguments passed through to the benchmark (after --)",
    )
    return parser.parse_args(argv)


def resolve_build_dir(build_dir: Path | None, build_type: str) -> Path:
    if build_dir is not None:
        return build_dir

    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    pixi_build = Path("build") / env_name / "cpp" / build_type
    if pixi_build.exists():
        return pixi_build

    fallback = Path("build")
    if fallback.exists():
        return fallback

    raise SystemExit(
        "Build directory not found. Run `pixi run config` or pass --build-dir."
    )


def find_binary(build_dir: Path, target: str) -> Path:
    candidates = [
        build_dir / "tests" / "benchmark" / target,
        build_dir / "tests" / "benchmark" / "collision" / target,
        build_dir / "bin" / target,
    ]

    for path in candidates:
        if path.exists():
            return path

    raise SystemExit(f"Benchmark binary not found for target '{target}'.")


def build_target(build_dir: Path, target: str) -> None:
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
    )


def _extract_counter(entry: dict, key: str) -> float | None:
    counters = entry.get("counters", {})
    if key in counters:
        return counters[key]
    if key in entry:
        return entry[key]
    return None


def summarize(json_path: Path) -> list[str]:
    data = json.loads(json_path.read_text())
    lines = []

    for bench in data.get("benchmarks", []):
        name = bench.get("name", "")
        if "PipelineBreakdown" not in name:
            continue
        if bench.get("run_type") not in (None, "iteration"):
            continue

        aabb = _extract_counter(bench, "aabb_update_ns")
        broad = _extract_counter(bench, "broadphase_ns")
        narrow = _extract_counter(bench, "narrowphase_ns")
        merge = _extract_counter(bench, "merge_ns")
        pairs = _extract_counter(bench, "pairs")
        contacts = _extract_counter(bench, "contacts")

        line = (
            f"{name}: aabb_ns={aabb} broadphase_ns={broad} "
            f"narrowphase_ns={narrow} merge_ns={merge} "
            f"pairs={pairs} contacts={contacts}"
        )
        lines.append(line)

    return lines


def append_summary(
    summary_path: Path, output_path: Path, build_dir: Path, summary_lines: list[str]
) -> None:
    today = date.today().isoformat()
    section = [
        "",
        f"## Run {today} — Pipeline breakdown",
        f"Raw Output: {output_path.as_posix()}",
        f"Build dir: {build_dir.as_posix()}",
        "Summary:",
        "```",
    ]
    section.extend(summary_lines or ["(no matching benchmarks found)"])
    section.extend(["```", ""])

    summary_path.write_text(summary_path.read_text() + "\n".join(section))


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    build_dir = resolve_build_dir(args.build_dir, args.build_type)

    if args.out is None:
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        out_path = (
            RESULTS_DIR / f"bm_pipeline_breakdown_{date.today().isoformat()}.json"
        )
    else:
        out_path = args.out
        out_path.parent.mkdir(parents=True, exist_ok=True)

    build_target(build_dir, TARGET)
    binary = find_binary(build_dir, TARGET)

    cmd = [
        str(binary),
        f"--benchmark_out={out_path.as_posix()}",
        "--benchmark_out_format=json",
    ]
    cmd.extend(arg for arg in args.benchmark_args if arg != "--")
    subprocess.run(cmd, check=True)

    if not args.no_doc_update:
        lines = summarize(out_path)
        append_summary(SUMMARY_PATH, out_path, build_dir, lines)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
