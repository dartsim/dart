#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Iterable, Sequence

LCOV_RC = [
    "--rc",
    "geninfo_unexecuted_blocks=1",
    "--rc",
    "lcov_excl_line=LCOV_EXCL_LINE",
]

LCOV_REMOVE_PATTERNS = [
    "/usr/*",
    "*/.pixi/*",
    "*/.deps/*",
    "*/tests/*",
    "*/examples/*",
    "*/tutorials/*",
    "*/dart/gui/*",
]

_COMPUTE_PARALLEL_JOBS = None


def _load_compute_parallel_jobs():
    global _COMPUTE_PARALLEL_JOBS
    if _COMPUTE_PARALLEL_JOBS is not None:
        return _COMPUTE_PARALLEL_JOBS

    module_path = Path(__file__).resolve().with_name("parallel_jobs.py")
    spec = importlib.util.spec_from_file_location("parallel_jobs", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load parallel job policy: {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _COMPUTE_PARALLEL_JOBS = module.compute_parallel_jobs
    return _COMPUTE_PARALLEL_JOBS


def _env_positive_int(name: str) -> int | None:
    raw = os.environ.get(name)
    if not raw:
        return None
    try:
        value = int(raw.strip())
    except ValueError:
        return None
    return value if value > 0 else None


def default_build_dir(env: dict[str, str] | None = None) -> Path:
    env = os.environ if env is None else env
    pixi_environment = env.get("PIXI_ENVIRONMENT_NAME") or "default"
    build_type = env.get("BUILD_TYPE") or "Debug"
    return Path("build") / pixi_environment / "cpp" / build_type


def delete_excluded_coverage_files(build_dir: Path) -> None:
    for pattern in ("*.gcda", "*.gcno"):
        for path in build_dir.rglob(pattern):
            if "examples" in path.parts or "tutorials" in path.parts:
                path.unlink()


def prune_nested_directories(directories: Iterable[Path]) -> list[Path]:
    result: list[Path] = []
    for directory in sorted({path for path in directories}, key=lambda p: p.parts):
        if any(directory == kept or directory.is_relative_to(kept) for kept in result):
            continue
        result.append(directory)
    return result


def collect_gcda_directories(build_dir: Path) -> list[Path]:
    return prune_nested_directories(path.parent for path in build_dir.rglob("*.gcda"))


def round_robin_chunks(items: Sequence[Path], chunk_count: int) -> list[list[Path]]:
    chunk_count = max(1, chunk_count)
    return [list(items[index::chunk_count]) for index in range(chunk_count)]


def default_parallel_jobs() -> int:
    override = _env_positive_int("DART_PARALLEL_JOBS")
    if override:
        return override

    return max(1, _load_compute_parallel_jobs()())


def lcov_capture_command(directories: Sequence[Path], output_file: Path) -> list[str]:
    command = ["lcov", "--quiet", "--capture"]
    for directory in directories:
        command.extend(["--directory", str(directory)])
    command.extend(["--output-file", str(output_file)])
    command.extend(LCOV_RC)
    return command


def run_lcov_capture_chunks(
    directories: Sequence[Path], jobs: int, temp_dir: Path
) -> list[Path]:
    processes: list[tuple[Path, subprocess.Popen[bytes]]] = []

    for index, chunk in enumerate(round_robin_chunks(directories, jobs)):
        if not chunk:
            continue

        output_file = temp_dir / f"coverage.chunk.{index}.info"
        processes.append(
            (output_file, subprocess.Popen(lcov_capture_command(chunk, output_file)))
        )

    failed = False
    for _, process in processes:
        if process.wait() != 0:
            failed = True

    if failed:
        raise RuntimeError("one or more lcov capture processes failed")

    return [output_file for output_file, _ in processes]


def merge_tracefiles(tracefiles: Sequence[Path], output_file: Path) -> None:
    if not tracefiles:
        raise RuntimeError("no lcov tracefiles were produced")

    command: list[str] = ["lcov"]
    for tracefile in tracefiles:
        command.extend(["-a", str(tracefile)])
    command.extend(["--output-file", str(output_file)])
    subprocess.run(command, check=True)


def remove_excluded_sources(output_file: Path) -> None:
    subprocess.run(
        [
            "lcov",
            "--remove",
            str(output_file),
            *LCOV_REMOVE_PATTERNS,
            "--output-file",
            str(output_file),
        ],
        check=True,
    )


def list_coverage(output_file: Path) -> None:
    subprocess.run(["lcov", "--list", str(output_file)], check=True)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture DART lcov coverage from a configured build tree."
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=default_build_dir(),
        help="Coverage build directory. Defaults to build/$PIXI_ENVIRONMENT_NAME/cpp/$BUILD_TYPE.",
    )
    parser.add_argument(
        "--jobs",
        type=int,
        default=default_parallel_jobs(),
        help="Number of parallel lcov capture workers. Defaults to scripts/parallel_jobs.py.",
    )
    parser.add_argument(
        "--output-file",
        type=Path,
        default=Path("coverage.info"),
        help="Merged lcov output path.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    build_dir = args.build_dir

    if not build_dir.is_dir():
        print(f"coverage build directory does not exist: {build_dir}", file=sys.stderr)
        return 1

    delete_excluded_coverage_files(build_dir)
    directories = collect_gcda_directories(build_dir)
    if not directories:
        print(f"no .gcda coverage data found under: {build_dir}", file=sys.stderr)
        return 1

    with tempfile.TemporaryDirectory(prefix="coverage.", dir=".") as temp_dir_name:
        temp_dir = Path(temp_dir_name)
        tracefiles = run_lcov_capture_chunks(directories, args.jobs, temp_dir)
        merge_tracefiles(tracefiles, args.output_file)

    remove_excluded_sources(args.output_file)
    list_coverage(args.output_file)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
