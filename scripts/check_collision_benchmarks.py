#!/usr/bin/env python3
"""Check native collision benchmark results against reference engines."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}
_BACKEND_RE = re.compile(
    r"^(?P<prefix>.+)_(?P<backend>Native|FCL|Bullet|ODE)(?P<params>/.*)?$"
)
_REFERENCE_BACKENDS = {"FCL", "Bullet", "ODE"}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        action="append",
        default=[],
        help="Existing Google Benchmark JSON file. Can be passed more than once.",
    )
    parser.add_argument(
        "--target",
        default=None,
        help="Benchmark CMake target to build and run when --input is omitted.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(".benchmark_results/collision_check.json"),
        help="Output JSON path when running a target.",
    )
    parser.add_argument(
        "--build-dir",
        type=Path,
        default=None,
        help="CMake build directory. Defaults to build/$PIXI_ENVIRONMENT_NAME/cpp/Release.",
    )
    parser.add_argument(
        "--build-type",
        default="Release",
        help="CMake build type for the default build directory.",
    )
    parser.add_argument(
        "--metric",
        choices=["cpu_time", "real_time"],
        default="cpu_time",
        help="Benchmark metric to compare.",
    )
    parser.add_argument(
        "--aggregate",
        choices=["median", "mean"],
        default="median",
        help="Preferred aggregate row when repetitions exist.",
    )
    parser.add_argument(
        "--max-native-ratio",
        type=float,
        default=1.0,
        help="Maximum allowed native/reference time ratio. 1.0 means native must be no slower.",
    )
    parser.add_argument(
        "--benchmark-filter",
        default=None,
        help="Google Benchmark filter passed when running --target.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print passing comparisons too.",
    )
    parser.add_argument(
        "benchmark_args",
        nargs=argparse.REMAINDER,
        help="Extra benchmark arguments after --.",
    )
    return parser.parse_args(argv)


def _to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def _resolve_build_dir(build_dir: Path | None, build_type: str) -> Path:
    if build_dir is not None:
        return build_dir

    env_name = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    candidate = Path("build") / env_name / "cpp" / build_type
    if candidate.exists():
        return candidate

    raise SystemExit(f"Build directory not found: {candidate}")


def _find_binary(build_dir: Path, target: str) -> Path:
    candidates = [
        build_dir / "bin" / target,
        build_dir / "tests" / "benchmark" / target,
        build_dir / "tests" / "benchmark" / "collision" / target,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise SystemExit(f"Benchmark binary not found for target '{target}' in {build_dir}")


def _run_target(args: argparse.Namespace) -> Path:
    if args.target is None:
        raise SystemExit("Pass --input or --target.")

    build_dir = _resolve_build_dir(args.build_dir, args.build_type)
    subprocess.run(
        [
            sys.executable,
            "scripts/cmake_build.py",
            "--build-dir",
            str(build_dir),
            "--target",
            args.target,
        ],
        check=True,
    )

    binary = _find_binary(build_dir, args.target)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(binary),
        f"--benchmark_out={args.output.as_posix()}",
        "--benchmark_out_format=json",
    ]
    if args.benchmark_filter:
        cmd.append(f"--benchmark_filter={args.benchmark_filter}")
    cmd.extend(arg for arg in args.benchmark_args if arg != "--")
    subprocess.run(cmd, check=True)
    return args.output


def _row_name(row: dict) -> str:
    return row.get("run_name") or row.get("name") or ""


def _load_rows(path: Path, metric: str, aggregate: str) -> list[tuple[str, float]]:
    data = json.loads(path.read_text())
    rows = data.get("benchmarks", [])

    aggregate_rows = [
        row
        for row in rows
        if row.get("run_type") == "aggregate" and row.get("aggregate_name") == aggregate
    ]
    if aggregate_rows:
        rows = aggregate_rows
    else:
        rows = [row for row in rows if row.get("run_type") != "aggregate"]

    values: list[tuple[str, float]] = []
    for row in rows:
        name = _row_name(row)
        if not name or metric not in row:
            continue
        values.append((name, _to_ns(float(row[metric]), row.get("time_unit", "ns"))))
    return values


def _comparison_key(name: str) -> tuple[str, str] | None:
    match = _BACKEND_RE.match(name)
    if not match:
        return None
    params = match.group("params") or ""
    return match.group("prefix") + params, match.group("backend")


def _load_comparisons(
    paths: list[Path], metric: str, aggregate: str
) -> dict[str, dict[str, float]]:
    grouped: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for path in paths:
        for name, value_ns in _load_rows(path, metric, aggregate):
            key = _comparison_key(name)
            if key is None:
                continue
            family, backend = key
            grouped[family][backend].append(value_ns)

    return {
        family: {
            backend: sum(values) / len(values)
            for backend, values in by_backend.items()
            if values
        }
        for family, by_backend in grouped.items()
    }


def check_results(
    comparisons: dict[str, dict[str, float]],
    max_native_ratio: float,
    verbose: bool,
) -> int:
    failures: list[str] = []
    passes: list[str] = []
    skipped = 0

    for family in sorted(comparisons):
        by_backend = comparisons[family]
        native = by_backend.get("Native")
        references = {
            backend: value
            for backend, value in by_backend.items()
            if backend in _REFERENCE_BACKENDS
        }
        if native is None or not references:
            skipped += 1
            continue

        best_backend, best_reference = min(references.items(), key=lambda item: item[1])
        if best_reference <= 0:
            skipped += 1
            continue

        ratio = native / best_reference
        line = (
            f"{family}: native={native / 1000.0:.3f}us, "
            f"best_reference={best_backend}:{best_reference / 1000.0:.3f}us, "
            f"ratio={ratio:.3f}, limit={max_native_ratio:.3f}"
        )
        if ratio > max_native_ratio:
            failures.append(line)
        else:
            passes.append(line)

    for line in failures:
        print(f"FAIL {line}")
    if verbose:
        for line in passes:
            print(f"PASS {line}")

    print(
        "collision benchmark check: "
        f"{len(passes)} passed, {len(failures)} failed, {skipped} skipped"
    )
    return 1 if failures else 0


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    paths = args.input or [_run_target(args)]
    comparisons = _load_comparisons(paths, args.metric, args.aggregate)
    if not comparisons:
        print("No native/reference benchmark comparisons found.", file=sys.stderr)
        return 1
    return check_results(comparisons, args.max_native_ratio, args.verbose)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
