#!/usr/bin/env python3
"""
Generate Dolan-More performance profiles for LCP solvers.

Usage:
    pixi run python scripts/lcp_performance_profile.py [--run] [--output OUTPUT_DIR]

Options:
    --run           Run benchmarks (otherwise use cached results)
    --output DIR    Output directory for results (default: docs/background/lcp/figures)
"""

import argparse
import csv
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def run_benchmarks(benchmark_exe: Path, filter_pattern: str = "") -> dict:
    cmd = [str(benchmark_exe), "--benchmark_format=json"]
    if filter_pattern:
        cmd.append(f"--benchmark_filter={filter_pattern}")

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    if result.returncode != 0:
        print(f"Benchmark failed: {result.stderr}", file=sys.stderr)
        sys.exit(1)

    return json.loads(result.stdout)


def parse_benchmark_results(data: dict) -> dict:
    results = defaultdict(lambda: defaultdict(dict))

    for bm in data.get("benchmarks", []):
        name = bm["name"]
        parts = name.split("/")
        if len(parts) < 2:
            continue

        solver_category = parts[0].replace("BM_LcpCompare_", "")
        problem_size = int(parts[1])

        solver_parts = solver_category.rsplit("_", 1)
        if len(solver_parts) == 2:
            solver_name, category = solver_parts
        else:
            solver_name = solver_category
            category = "Unknown"

        time_ns = bm.get("cpu_time", bm.get("real_time", 0))
        contract_ok = bm.get("contract_ok", 0)

        results[category][(solver_name, problem_size)] = {
            "time_ns": time_ns,
            "contract_ok": contract_ok,
            "residual": bm.get("residual", 0),
            "complementarity": bm.get("complementarity", 0),
        }

    return results


def compute_performance_ratios(results: dict, category: str) -> tuple[dict, list, list]:
    category_results = results.get(category, {})
    if not category_results:
        return {}, [], []

    problems = sorted(set(ps for _, ps in category_results.keys()))
    solvers = sorted(set(s for s, _ in category_results.keys()))

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
        writer = csv.writer(f)
        solvers = sorted(profiles.keys())
        writer.writerow(["tau"] + solvers)
        for i, tau in enumerate(tau_values):
            row = [f"{tau:.4f}"]
            for solver in solvers:
                row.append(f"{profiles[solver][i]:.4f}")
            writer.writerow(row)
    print(f"Saved CSV: {output_path}")


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
        data = run_benchmarks(benchmark_exe)

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

    categories = ["Standard", "Boxed", "FrictionIndex"]

    for category in categories:
        ratios, solvers, problems = compute_performance_ratios(results, category)
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
