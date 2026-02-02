#!/usr/bin/env python3
"""Compare two Google Benchmark JSON files and produce a comparison table."""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
import sys
from collections import defaultdict
from pathlib import Path

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}
_NS_TO_US = 1e-3


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare two Google Benchmark JSON files.",
    )
    parser.add_argument(
        "baseline",
        type=Path,
        help="Baseline benchmark JSON file",
    )
    parser.add_argument(
        "optimized",
        type=Path,
        help="Optimized benchmark JSON file",
    )
    parser.add_argument(
        "--metric",
        choices=["cpu_time", "real_time"],
        default="cpu_time",
        help="Time metric to compare (default: cpu_time)",
    )
    parser.add_argument(
        "--aggregate",
        choices=["median", "mean"],
        default="median",
        help="Aggregate type to use when repetitions exist (default: median)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output file (default: stdout)",
    )
    parser.add_argument(
        "--format",
        choices=["markdown", "csv"],
        default="markdown",
        dest="fmt",
        help="Output format (default: markdown)",
    )
    return parser.parse_args(argv)


def _to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def load_benchmarks(
    filepath: Path,
    metric: str,
    aggregate: str,
) -> tuple[dict, dict[str, float]]:
    """Load benchmark JSON and return ``(context, {name: time_in_ns})``.

    Prefers aggregate entries (median/mean) when available.  Falls back to
    iteration entries, averaging when a benchmark has multiple repetitions.
    """
    with open(filepath) as f:
        data = json.load(f)

    context = data.get("context", {})
    benchmarks = data.get("benchmarks", [])

    if not benchmarks:
        print(
            "Error: no benchmarks in {}".format(filepath),
            file=sys.stderr,
        )
        sys.exit(1)

    agg_entries = [
        b
        for b in benchmarks
        if b.get("run_type") == "aggregate" and b.get("aggregate_name") == aggregate
    ]

    if agg_entries:
        result: dict[str, float] = {}
        for b in agg_entries:
            result[b["run_name"]] = _to_ns(b[metric], b.get("time_unit", "ns"))
        return context, result

    iter_entries = [b for b in benchmarks if b.get("run_type") == "iteration"]
    if not iter_entries:
        iter_entries = benchmarks

    groups: dict[str, list[float]] = defaultdict(list)
    for b in iter_entries:
        name = b.get("run_name", b.get("name", "unknown"))
        groups[name].append(_to_ns(b[metric], b.get("time_unit", "ns")))

    return context, {n: sum(t) / len(t) for n, t in groups.items()}


def _strip_aggregate_suffix(name: str) -> str:
    for suffix in ("_mean", "_median", "_stddev", "_cv"):
        if name.endswith(suffix):
            return name[: -len(suffix)]
    return name


def compute_comparison(
    baseline_times: dict[str, float],
    optimized_times: dict[str, float],
) -> list[dict]:
    base_norm = {_strip_aggregate_suffix(k): v for k, v in baseline_times.items()}
    opt_norm = {_strip_aggregate_suffix(k): v for k, v in optimized_times.items()}

    all_names = set(base_norm.keys()) | set(opt_norm.keys())

    rows: list[dict] = []
    for name in sorted(all_names):
        if name not in base_norm:
            print(
                "Warning: '{}' only in optimized, skipping".format(name),
                file=sys.stderr,
            )
            continue
        if name not in opt_norm:
            print(
                "Warning: '{}' only in baseline, skipping".format(name),
                file=sys.stderr,
            )
            continue

        base_ns = base_norm[name]
        opt_ns = opt_norm[name]

        if base_ns <= 0:
            print(
                "Warning: zero baseline for '{}', skipping".format(name),
                file=sys.stderr,
            )
            continue

        base_us = base_ns * _NS_TO_US
        opt_us = opt_ns * _NS_TO_US
        delta_us = opt_us - base_us
        change_pct = (opt_ns - base_ns) / base_ns * 100.0

        rows.append(
            {
                "name": name,
                "baseline_us": base_us,
                "optimized_us": opt_us,
                "delta_us": delta_us,
                "change_pct": change_pct,
            }
        )

    rows.sort(key=lambda r: r["change_pct"])
    return rows


def geometric_mean_speedup(rows: list[dict]) -> float:
    """``exp(mean(log(baseline_i / optimized_i)))`` for all valid pairs."""
    log_ratios: list[float] = []
    for r in rows:
        if r["baseline_us"] > 0 and r["optimized_us"] > 0:
            log_ratios.append(math.log(r["baseline_us"] / r["optimized_us"]))
    if not log_ratios:
        return 1.0
    return math.exp(sum(log_ratios) / len(log_ratios))


def _short_name(name: str) -> str:
    idx = name.find("/min_time:")
    if idx != -1:
        return name[:idx]
    return name


def _fmt_change(pct: float) -> str:
    if pct <= -1.0:
        return "**{:.1f}%**".format(pct)
    if pct >= 1.0:
        return "**+{:.1f}%**".format(pct)
    if pct >= 0:
        return "+{:.1f}%".format(pct)
    return "{:.1f}%".format(pct)


def format_markdown(
    rows: list[dict],
    base_ctx: dict,
    opt_ctx: dict,
    metric: str,
    aggregate: str,
) -> str:
    lines: list[str] = [
        "## Benchmark Comparison",
        "",
        "**Baseline**: {} ({})".format(
            base_ctx.get("date", "unknown"),
            base_ctx.get("executable", "unknown"),
        ),
        "**Optimized**: {} ({})".format(
            opt_ctx.get("date", "unknown"),
            opt_ctx.get("executable", "unknown"),
        ),
        "**CPU**: {} \u00d7 {} MHz".format(
            base_ctx.get("num_cpus", "?"),
            base_ctx.get("mhz_per_cpu", "?"),
        ),
        "**Metric**: {} ({})".format(metric, aggregate),
        "",
        "| Benchmark | Baseline (\u00b5s) | Optimized (\u00b5s) "
        "| \u0394 (\u00b5s) | Change |",
        "|-----------|--------------|----------------|--------|--------|",
    ]

    for r in rows:
        lines.append(
            "| {} | {:.1f} | {:.1f} | {:.1f} | {} |".format(
                _short_name(r["name"]),
                r["baseline_us"],
                r["optimized_us"],
                r["delta_us"],
                _fmt_change(r["change_pct"]),
            )
        )

    n_total = len(rows)
    n_improved = sum(1 for r in rows if r["change_pct"] < -1.0)
    n_regressed = sum(1 for r in rows if r["change_pct"] > 1.0)
    n_unchanged = n_total - n_improved - n_regressed
    geo = geometric_mean_speedup(rows)

    lines.extend(
        [
            "",
            "### Summary",
            "- **Compared**: {} benchmarks".format(n_total),
            "- **Improved** (>1%): {}".format(n_improved),
            "- **Regressed** (>1%): {}".format(n_regressed),
            "- **Unchanged**: {}".format(n_unchanged),
            "- **Geometric mean speedup**: {:.2f}x".format(geo),
            "",
        ]
    )

    return "\n".join(lines)


def format_csv(
    rows: list[dict],
    base_ctx: dict,
    opt_ctx: dict,
    metric: str,
    aggregate: str,
) -> str:
    _ = base_ctx, opt_ctx, metric, aggregate

    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(
        [
            "Benchmark",
            "Baseline (us)",
            "Optimized (us)",
            "Delta (us)",
            "Change (%)",
        ]
    )
    for r in rows:
        writer.writerow(
            [
                _short_name(r["name"]),
                "{:.1f}".format(r["baseline_us"]),
                "{:.1f}".format(r["optimized_us"]),
                "{:.1f}".format(r["delta_us"]),
                "{:.1f}".format(r["change_pct"]),
            ]
        )
    return buf.getvalue()


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if not args.baseline.exists():
        print(
            "Error: file not found: {}".format(args.baseline),
            file=sys.stderr,
        )
        return 1
    if not args.optimized.exists():
        print(
            "Error: file not found: {}".format(args.optimized),
            file=sys.stderr,
        )
        return 1

    base_ctx, base_times = load_benchmarks(args.baseline, args.metric, args.aggregate)
    opt_ctx, opt_times = load_benchmarks(args.optimized, args.metric, args.aggregate)

    rows = compute_comparison(base_times, opt_times)

    if not rows:
        print("Error: no matching benchmarks found", file=sys.stderr)
        return 1

    if args.fmt == "markdown":
        text = format_markdown(rows, base_ctx, opt_ctx, args.metric, args.aggregate)
    else:
        text = format_csv(rows, base_ctx, opt_ctx, args.metric, args.aggregate)

    if args.output:
        args.output.write_text(text)
        print("Output written to {}".format(args.output), file=sys.stderr)
    else:
        sys.stdout.write(text)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
