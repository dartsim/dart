#!/usr/bin/env python3
"""
Generate Dolan-More performance profiles for LCP solvers.

Usage:
    pixi run python scripts/lcp_performance_profile.py [--run] [--output OUTPUT_DIR]

Options:
    --run           Run benchmarks (otherwise use cached results)
    --output DIR    Output directory for results (default: docs/background/lcp/figures)
    --benchmark-filter REGEX
                    Google Benchmark regex (default: BM_LcpCompare/)
    --benchmark-min-time VALUE
                    Optional Google Benchmark minimum time for smoke runs
    --benchmark-timeout SECONDS
                    Benchmark subprocess timeout (default: 600)
    --allow-partial
                    Allow incomplete native solver coverage for smoke runs
"""

import argparse
import csv
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from check_lcp_solver_roster import (
    FORM_SUPPORT_COUNTER_BY_CATEGORY,
    FORM_SUPPORT_COUNTERS,
    PROBLEM_TYPE_COUNTER_BY_CATEGORY,
    PROBLEM_TYPE_COUNTERS,
    PROFILE_CATEGORIES,
    REQUIRED_EVIDENCE_COLUMNS,
    SOLVER_FAMILY_COUNTER_BY_FAMILY,
    SOLVER_FAMILY_COUNTERS,
    SOLVER_IDENTITY_COUNTERS,
    SOLVER_IDENTITY_SCHEMA_VERSION,
    parse_cpp_manifest,
)

try:
    import matplotlib.pyplot as plt

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

_MANIFEST_NAME_BY_LOWER: dict[str, str] | None = None
_MANIFEST_NAME_BY_ID: dict[int, str] | None = None
_MANIFEST_FAMILY_BY_NAME: dict[str, str] | None = None

DEFAULT_BENCHMARK_FILTER = "BM_LcpCompare/"
DEFAULT_BENCHMARK_TIMEOUT_SECONDS = 600
PROFILE_EVIDENCE_CSV_NAME = "performance_profile_evidence.csv"


def canonical_solver_name(name: str) -> str:
    global _MANIFEST_NAME_BY_LOWER
    if _MANIFEST_NAME_BY_LOWER is None:
        _MANIFEST_NAME_BY_LOWER = {
            entry.name.lower(): entry.name for entry in parse_cpp_manifest()
        }
    return _MANIFEST_NAME_BY_LOWER.get(name.lower(), name)


def manifest_name_by_id() -> dict[int, str]:
    global _MANIFEST_NAME_BY_ID
    if _MANIFEST_NAME_BY_ID is None:
        _MANIFEST_NAME_BY_ID = {
            index: entry.name
            for index, entry in enumerate(parse_cpp_manifest(), start=1)
        }
    return _MANIFEST_NAME_BY_ID


def manifest_family_by_name() -> dict[str, str]:
    global _MANIFEST_FAMILY_BY_NAME
    if _MANIFEST_FAMILY_BY_NAME is None:
        _MANIFEST_FAMILY_BY_NAME = {
            entry.name: entry.family for entry in parse_cpp_manifest()
        }
    return _MANIFEST_FAMILY_BY_NAME


def expected_solver_family_counters(solver_name: str) -> dict[str, float | None]:
    family = manifest_family_by_name().get(solver_name)
    return {
        counter: (
            (1.0 if counter == SOLVER_FAMILY_COUNTER_BY_FAMILY.get(family) else 0.0)
            if family is not None
            else None
        )
        for counter in SOLVER_FAMILY_COUNTERS
    }


def run_benchmarks(
    benchmark_exe: Path,
    filter_pattern: str = DEFAULT_BENCHMARK_FILTER,
    benchmark_min_time: str = "",
    benchmark_timeout: int = DEFAULT_BENCHMARK_TIMEOUT_SECONDS,
) -> dict:
    cmd = [str(benchmark_exe), "--benchmark_format=json"]
    if filter_pattern:
        cmd.append(f"--benchmark_filter={filter_pattern}")
    if benchmark_min_time:
        cmd.append(f"--benchmark_min_time={benchmark_min_time}")

    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=benchmark_timeout
        )
    except subprocess.TimeoutExpired:
        print(
            f"Benchmark timed out after {benchmark_timeout} seconds: {' '.join(cmd)}",
            file=sys.stderr,
        )
        print(
            "Use --benchmark-filter to narrow the run or --benchmark-timeout "
            "to allow a longer full-profile run.",
            file=sys.stderr,
        )
        sys.exit(1)
    if result.returncode != 0:
        print(f"Benchmark failed: {result.stderr}", file=sys.stderr)
        sys.exit(1)

    return json.loads(result.stdout)


def _parse_problem_size(token: str) -> int | None:
    try:
        return int(token)
    except ValueError:
        return None


def parse_benchmark_name(name: str) -> tuple[str, str, int] | None:
    parts = name.split("/")

    # Current benchmark schema from tests/benchmark/lcpsolver/bm_lcp_compare.cpp:
    # BM_LcpCompare/<problem-family>/<solver>/<problem-size>
    if len(parts) >= 4 and parts[0] == "BM_LcpCompare":
        category = parts[1]
        if category not in PROFILE_CATEGORIES:
            return None
        problem_size = _parse_problem_size(parts[3])
        if problem_size is None:
            return None
        return category, canonical_solver_name(parts[2]), problem_size

    # Historical cached profile schema kept for old benchmark JSON packets:
    # BM_LcpCompare_<solver>_<problem-family>/<problem-size>
    if len(parts) >= 2 and parts[0].startswith("BM_LcpCompare_"):
        solver_category = parts[0].removeprefix("BM_LcpCompare_")
        solver_parts = solver_category.rsplit("_", 1)
        if len(solver_parts) != 2:
            return None
        solver_name, category = solver_parts
        if category not in PROFILE_CATEGORIES:
            return None
        problem_size = _parse_problem_size(parts[1])
        if problem_size is None:
            return None
        return category, canonical_solver_name(solver_name), problem_size

    return None


def parse_benchmark_results(data: dict) -> dict:
    results = defaultdict(lambda: defaultdict(dict))
    observed_keys: set[tuple[str, str, int]] = set()

    for bm in data.get("benchmarks", []):
        if bm.get("run_type") == "aggregate":
            continue

        parsed = parse_benchmark_name(bm["name"])
        if parsed is None:
            continue
        category, solver_name, problem_size = parsed
        result_key = (category, solver_name, problem_size)
        if result_key in observed_keys:
            raise RuntimeError(
                "duplicate LCP performance profile benchmark row for "
                f"{category}/{solver_name}/{problem_size}"
            )
        observed_keys.add(result_key)

        time_ns = bm.get("cpu_time", bm.get("real_time", 0))
        contract_ok = bm.get("contract_ok", 0)
        family_counters = expected_solver_family_counters(solver_name)

        results[category][(solver_name, problem_size)] = {
            "lcp_dimension": bm.get("problem_size"),
            "contact_count": bm.get("contact_count"),
            "time_ns": time_ns,
            "contract_ok": contract_ok,
            "iterations": bm.get("iterations"),
            "residual": bm.get("residual", 0),
            "complementarity": bm.get("complementarity", 0),
            "bound_violation": bm.get("bound_violation"),
            "solver_identity_schema_version": bm.get("solver_identity_schema_version"),
            "solver_manifest_index": bm.get("solver_manifest_index"),
            **{
                counter: bm.get(counter, family_counters[counter])
                for counter in SOLVER_FAMILY_COUNTERS
            },
            "solver_supports_standard": bm.get("solver_supports_standard"),
            "solver_supports_boxed": bm.get("solver_supports_boxed"),
            "solver_supports_friction_index": bm.get("solver_supports_friction_index"),
            "solver_supports_problem": bm.get("solver_supports_problem"),
            "problem_type_standard": bm.get("problem_type_standard"),
            "problem_type_boxed": bm.get("problem_type_boxed"),
            "problem_type_friction_index": bm.get("problem_type_friction_index"),
            "problem_type_invalid": bm.get("problem_type_invalid"),
        }

    return results


def load_native_support_by_category() -> dict[str, set[str]]:
    manifest = parse_cpp_manifest()
    return {
        "Standard": {entry.name for entry in manifest if entry.standard},
        "Boxed": {entry.name for entry in manifest if entry.boxed},
        "FrictionIndex": {entry.name for entry in manifest if entry.findex},
    }


def _counter_to_int(value) -> int | None:
    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None

    rounded = int(round(numeric))
    if abs(numeric - rounded) > 1e-9:
        return None
    return rounded


def check_native_profile_coverage(
    results: dict,
    native_support_by_category: dict[str, set[str]],
    allow_partial: bool = False,
) -> None:
    coverage_errors = []
    evidence_errors = []
    manifest_names = set().union(*native_support_by_category.values())

    for category in PROFILE_CATEGORIES:
        observed = {solver for solver, _ in results.get(category, {})}
        expected = native_support_by_category[category]
        unknown = sorted(observed - manifest_names)
        missing = sorted(expected - observed)
        non_native_rows = sorted(
            f"{solver}/{problem_size}"
            for (solver, problem_size), _ in results.get(category, {}).items()
            if solver in manifest_names and solver not in expected
        )
        unsupported_rows = sorted(
            f"{solver}/{problem_size}"
            for (solver, problem_size), data in results.get(category, {}).items()
            if data.get("solver_supports_problem") is not None
            and data["solver_supports_problem"] <= 0.5
        )
        form_support_mismatches = sorted(
            f"{solver}/{problem_size}"
            for (solver, problem_size), data in results.get(category, {}).items()
            if any(data.get(key) is not None for key in FORM_SUPPORT_COUNTERS)
            and data.get(FORM_SUPPORT_COUNTER_BY_CATEGORY[category], 0.0) <= 0.5
        )
        problem_type_mismatches = sorted(
            f"{solver}/{problem_size}"
            for (solver, problem_size), data in results.get(category, {}).items()
            if any(data.get(key) is not None for key in PROBLEM_TYPE_COUNTERS)
            and (
                data.get(PROBLEM_TYPE_COUNTER_BY_CATEGORY[category], 0.0) <= 0.5
                or sum(1 for key in PROBLEM_TYPE_COUNTERS if data.get(key, 0.0) > 0.5)
                != 1
            )
        )
        problem_dimension_mismatches = []
        for (solver, problem_size), data in results.get(category, {}).items():
            lcp_dimension = _counter_to_int(data.get("lcp_dimension"))
            contact_count = _counter_to_int(data.get("contact_count"))
            if lcp_dimension is None:
                continue
            if category == "FrictionIndex":
                if contact_count != problem_size or lcp_dimension != 3 * problem_size:
                    problem_dimension_mismatches.append(f"{solver}/{problem_size}")
            elif lcp_dimension != problem_size:
                problem_dimension_mismatches.append(f"{solver}/{problem_size}")
        problem_dimension_mismatches = sorted(problem_dimension_mismatches)
        solver_identity_mismatches = sorted(
            f"{solver}/{problem_size}"
            for (solver, problem_size), data in results.get(category, {}).items()
            if any(data.get(key) is not None for key in SOLVER_IDENTITY_COUNTERS)
            and (
                _counter_to_int(data.get("solver_identity_schema_version"))
                != SOLVER_IDENTITY_SCHEMA_VERSION
                or manifest_name_by_id().get(
                    _counter_to_int(data.get("solver_manifest_index"))
                )
                != solver
            )
        )
        solver_family_mismatches = []
        for (solver, problem_size), data in results.get(category, {}).items():
            if not any(data.get(key) is not None for key in SOLVER_FAMILY_COUNTERS):
                continue
            expected_counters = expected_solver_family_counters(solver)
            actual_sum = 0
            mismatch = False
            for key in SOLVER_FAMILY_COUNTERS:
                actual = _counter_to_int(data.get(key))
                expected = _counter_to_int(expected_counters[key])
                actual_sum += 1 if actual == 1 else 0
                if actual != expected:
                    mismatch = True
            if mismatch or actual_sum != 1:
                solver_family_mismatches.append(f"{solver}/{problem_size}")
        solver_family_mismatches = sorted(solver_family_mismatches)

        if unknown:
            evidence_errors.append(
                f"{category}: benchmark JSON contains unknown solvers {unknown}"
            )
        if missing:
            coverage_errors.append(f"{category}: missing native solvers {missing}")
        if non_native_rows:
            evidence_errors.append(
                f"{category}: benchmark JSON contains non-native rows "
                f"{non_native_rows}"
            )
        if unsupported_rows:
            evidence_errors.append(
                f"{category}: current-schema rows report "
                f"solver_supports_problem=0 for {unsupported_rows}"
            )
        if form_support_mismatches:
            evidence_errors.append(
                f"{category}: current-schema rows report unsupported "
                f"form counters for {form_support_mismatches}"
            )
        if problem_type_mismatches:
            evidence_errors.append(
                f"{category}: current-schema rows disagree with problem_type "
                f"counters for {problem_type_mismatches}"
            )
        if problem_dimension_mismatches:
            evidence_errors.append(
                f"{category}: current-schema rows disagree with problem "
                f"dimension counters for {problem_dimension_mismatches}"
            )
        if solver_identity_mismatches:
            evidence_errors.append(
                f"{category}: current-schema rows disagree with solver identity "
                f"counters for {solver_identity_mismatches}"
            )
        if solver_family_mismatches:
            evidence_errors.append(
                f"{category}: current-schema rows disagree with solver family "
                f"counters for {solver_family_mismatches}"
            )

    if evidence_errors:
        message = "LCP performance profile evidence is invalid:\n  - " + "\n  - ".join(
            evidence_errors
        )
        raise RuntimeError(message)

    if not coverage_errors:
        return

    message = "LCP performance profile coverage is incomplete:\n  - " + "\n  - ".join(
        coverage_errors
    )
    if allow_partial:
        print(f"Warning: {message}", file=sys.stderr)
        return
    raise RuntimeError(message)


def compute_performance_ratios(
    results: dict,
    category: str,
    supported_solvers: set[str] | None = None,
) -> tuple[dict, list, list]:
    category_results = results.get(category, {})
    if not category_results:
        return {}, [], []

    problems = sorted(set(ps for _, ps in category_results.keys()))
    solvers = sorted(set(s for s, _ in category_results.keys()))
    if supported_solvers is not None:
        solvers = [solver for solver in solvers if solver in supported_solvers]

    ratios = defaultdict(list)

    for problem_size in problems:
        times = {}
        for solver in solvers:
            key = (solver, problem_size)
            if key in category_results:
                data = category_results[key]
                if data["contract_ok"] > 0.5:
                    times[solver] = data["time_ns"]

        if not times:
            continue

        best_time = min(times.values())

        for solver in solvers:
            if solver in times:
                ratio = times[solver] / best_time
                ratios[solver].append(ratio)
            else:
                ratios[solver].append(float("inf"))

    return ratios, solvers, problems


def compute_performance_profile(
    ratios: dict, solvers: list, tau_max: float = 10.0, num_points: int = 200
) -> tuple[np.ndarray, dict]:
    tau_values = np.linspace(1.0, tau_max, num_points)
    profiles = {}

    for solver in solvers:
        solver_ratios = ratios.get(solver, [])
        if not solver_ratios:
            profiles[solver] = np.zeros(num_points)
            continue

        n_problems = len(solver_ratios)
        profile = np.zeros(num_points)

        for i, tau in enumerate(tau_values):
            count = sum(1 for r in solver_ratios if r <= tau)
            profile[i] = count / n_problems

        profiles[solver] = profile

    return tau_values, profiles


def save_profile_csv(
    tau_values: np.ndarray,
    profiles: dict,
    output_path: Path,
):
    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        solvers = sorted(profiles.keys())
        writer.writerow(["tau"] + solvers)
        for i, tau in enumerate(tau_values):
            row = [f"{tau:.4f}"]
            for solver in solvers:
                row.append(f"{profiles[solver][i]:.4f}")
            writer.writerow(row)
    print(f"Saved CSV: {output_path}")


def _csv_value(value) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.17g}"
    return str(value)


def _missing_required_evidence_fields(category: str, data: dict) -> list[str]:
    missing = []
    for key in REQUIRED_EVIDENCE_COLUMNS[3:]:
        if key == "contact_count" and category != "FrictionIndex":
            continue
        if _csv_value(data.get(key)) == "":
            missing.append(key)
    return missing


def save_profile_evidence_csv(results: dict, output_path: Path) -> None:
    header = list(REQUIRED_EVIDENCE_COLUMNS)

    with open(output_path, "w", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(header)
        for category in PROFILE_CATEGORIES:
            for (solver, problem_size), data in sorted(
                results.get(category, {}).items()
            ):
                missing = _missing_required_evidence_fields(category, data)
                if missing:
                    raise RuntimeError(
                        "Cannot write LCP performance profile evidence with "
                        "missing current-schema fields for "
                        f"{category}/{solver}/{problem_size}: {missing}"
                    )
                writer.writerow(
                    [
                        category,
                        solver,
                        problem_size,
                        *(_csv_value(data.get(key)) for key in header[3:]),
                    ]
                )

    print(f"Saved evidence CSV: {output_path}")


def plot_performance_profile(
    tau_values: np.ndarray,
    profiles: dict,
    category: str,
    output_path: Path,
    title_suffix: str = "",
):
    if not HAS_MATPLOTLIB:
        csv_path = output_path.with_suffix(".csv")
        save_profile_csv(tau_values, profiles, csv_path)
        return

    fig, ax = plt.subplots(figsize=(10, 6))

    colors = plt.cm.tab20(np.linspace(0, 1, len(profiles)))

    sorted_solvers = sorted(
        profiles.keys(), key=lambda s: -profiles[s][-1] if len(profiles[s]) > 0 else 0
    )

    for i, solver in enumerate(sorted_solvers):
        profile = profiles[solver]
        ax.plot(tau_values, profile, label=solver, color=colors[i], linewidth=1.5)

    ax.set_xlabel(r"Performance ratio $\tau$", fontsize=12)
    ax.set_ylabel(r"Fraction of problems solved $P(r \leq \tau)$", fontsize=12)
    ax.set_title(f"Performance Profile: {category} LCP Problems{title_suffix}")
    ax.set_xlim(1, tau_values[-1])
    ax.set_ylim(0, 1.05)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=8, ncol=2)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()

    print(f"Saved plot: {output_path}")


def print_summary_table(ratios: dict, solvers: list, category: str):
    print(f"\n=== {category} Problems ===")
    print(f"{'Solver':<30} {'Wins':>6} {'Solved':>8} {'Avg Ratio':>10}")
    print("-" * 56)

    for solver in sorted(solvers):
        solver_ratios = ratios.get(solver, [])
        if not solver_ratios:
            continue

        finite_ratios = [r for r in solver_ratios if r != float("inf")]
        wins = sum(1 for r in solver_ratios if r == 1.0)
        solved = len(finite_ratios)
        avg_ratio = np.mean(finite_ratios) if finite_ratios else float("inf")

        print(f"{solver:<30} {wins:>6} {solved:>8} {avg_ratio:>10.2f}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate LCP solver performance profiles"
    )
    parser.add_argument("--run", action="store_true", help="Run benchmarks")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/background/lcp/figures"),
        help="Output directory",
    )
    parser.add_argument(
        "--cache",
        type=Path,
        default=Path("build/benchmark_results.json"),
        help="Cache file for benchmark results",
    )
    parser.add_argument(
        "--benchmark-filter",
        default=DEFAULT_BENCHMARK_FILTER,
        help=(
            "Google Benchmark regex passed to BM_LCP_COMPARE "
            f"(default: {DEFAULT_BENCHMARK_FILTER!r})"
        ),
    )
    parser.add_argument(
        "--benchmark-min-time",
        default="",
        help="Optional --benchmark_min_time value for benchmark smoke runs",
    )
    parser.add_argument(
        "--allow-partial",
        action="store_true",
        help="Allow incomplete native solver coverage in benchmark JSON",
    )
    parser.add_argument(
        "--benchmark-timeout",
        type=int,
        default=DEFAULT_BENCHMARK_TIMEOUT_SECONDS,
        help=(
            "Timeout in seconds for the BM_LCP_COMPARE subprocess "
            f"(default: {DEFAULT_BENCHMARK_TIMEOUT_SECONDS})"
        ),
    )
    args = parser.parse_args()

    project_root = Path(__file__).parent.parent
    benchmark_exe = project_root / "build/default/cpp/Release/bin/BM_LCP_COMPARE"

    if args.run or not args.cache.exists():
        if not benchmark_exe.exists():
            print(f"Benchmark not found: {benchmark_exe}")
            print(
                "Build with: cmake --build build/default/cpp/Release --target tests/benchmark/lcpsolver/all"
            )
            sys.exit(1)

        print("Running benchmarks (this may take a few minutes)...")
        data = run_benchmarks(
            benchmark_exe,
            args.benchmark_filter,
            args.benchmark_min_time,
            args.benchmark_timeout,
        )

        args.cache.parent.mkdir(parents=True, exist_ok=True)
        with open(args.cache, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Cached results to: {args.cache}")
    else:
        print(f"Loading cached results from: {args.cache}")
        with open(args.cache) as f:
            data = json.load(f)

    results = parse_benchmark_results(data)

    args.output.mkdir(parents=True, exist_ok=True)

    native_support_by_category = load_native_support_by_category()
    check_native_profile_coverage(
        results, native_support_by_category, allow_partial=args.allow_partial
    )
    save_profile_evidence_csv(results, args.output / PROFILE_EVIDENCE_CSV_NAME)

    for category in PROFILE_CATEGORIES:
        ratios, solvers, problems = compute_performance_ratios(
            results, category, native_support_by_category[category]
        )
        if not solvers:
            print(f"No results for category: {category}")
            continue

        print_summary_table(ratios, solvers, category)

        tau_values, profiles = compute_performance_profile(ratios, solvers)

        output_path = args.output / f"performance_profile_{category.lower()}.png"
        plot_performance_profile(tau_values, profiles, category, output_path)

    print("\nDone!")


if __name__ == "__main__":
    main()
