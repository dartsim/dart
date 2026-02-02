#!/usr/bin/env python3
"""Parse Google Benchmark JSON from allocator benchmarks and produce reports."""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}

_CATEGORY_PATTERNS = [
    ("Single Alloc/Dealloc", re.compile(r"BM_SingleAlloc_")),
    ("Frame Bulk Reset", re.compile(r"BM_FrameBulkReset")),
    ("Mixed Steady State", re.compile(r"BM_MixedSteadyState_")),
    ("Pool Boundary", re.compile(r"BM_PoolBoundary")),
    ("MM Dispatch", re.compile(r"BM_MMDispatch_|BM_DirectFreeList")),
    ("MM Construct/Destroy", re.compile(r"BM_MMConstruct_")),
    ("STL Container", re.compile(r"BM_StlContainer_")),
    ("Realistic Workload", re.compile(r"BM_RealisticWorkload_")),
]

_ALLOCATOR_RE = re.compile(r"BM_\w+?_(\w+?)(?:<|/|$)")

_CHART_WIDTH = 60
_CHART_HEIGHT = 15
_MARKERS = [".", "o", "x", "+", "*", "#", "@", "="]


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Google Benchmark JSON file")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory (default: same as input)",
    )
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        dest="json_out",
        help="JSON output file (default: <output-dir>/allocator_report.json)",
    )
    parser.add_argument(
        "--markdown",
        type=Path,
        default=None,
        dest="md_out",
        help="Markdown output file (default: <output-dir>/allocator_report.md)",
    )
    return parser.parse_args(argv)


def _to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def _classify_category(name: str) -> str:
    for cat_name, pattern in _CATEGORY_PATTERNS:
        if pattern.search(name):
            return cat_name
    return "Other"


def _extract_allocator(name: str) -> str:
    m = _ALLOCATOR_RE.match(name)
    if m:
        return m.group(1)
    return "unknown"


def load_benchmarks(filepath: Path) -> tuple[dict, list[dict]]:
    with open(filepath) as f:
        data = json.load(f)
    context = data.get("context", {})
    benchmarks = data.get("benchmarks", [])
    if not benchmarks:
        print("Error: no benchmarks in {}".format(filepath), file=sys.stderr)
        sys.exit(1)
    return context, benchmarks


def _prefer_aggregates(benchmarks: list[dict], agg: str = "median") -> list[dict]:
    agg_entries = [
        b
        for b in benchmarks
        if b.get("run_type") == "aggregate" and b.get("aggregate_name") == agg
    ]
    if agg_entries:
        return agg_entries

    iter_entries = [b for b in benchmarks if b.get("run_type") == "iteration"]
    if not iter_entries:
        iter_entries = benchmarks

    groups: dict[str, list[dict]] = defaultdict(list)
    for b in iter_entries:
        name = b.get("run_name", b.get("name", "unknown"))
        groups[name].append(b)

    result = []
    for name, entries in groups.items():
        avg = dict(entries[0])
        avg["cpu_time"] = sum(e["cpu_time"] for e in entries) / len(entries)
        avg["real_time"] = sum(e["real_time"] for e in entries) / len(entries)
        avg["run_name"] = name
        result.append(avg)
    return result


def build_structured_data(
    benchmarks: list[dict],
) -> dict[str, dict[str, list[dict]]]:
    entries = _prefer_aggregates(benchmarks)
    structured: dict[str, dict[str, list[dict]]] = defaultdict(
        lambda: defaultdict(list)
    )

    for b in entries:
        name = b.get("run_name", b.get("name", "unknown"))
        category = _classify_category(name)
        allocator = _extract_allocator(name)
        time_ns = _to_ns(b.get("cpu_time", 0), b.get("time_unit", "ns"))

        entry = {
            "name": name,
            "cpu_time_ns": time_ns,
            "real_time_ns": _to_ns(b.get("real_time", 0), b.get("time_unit", "ns")),
            "iterations": b.get("iterations", 0),
        }

        if "items_per_second" in b:
            entry["items_per_second"] = b["items_per_second"]
        if "bytes_per_second" in b:
            entry["bytes_per_second"] = b["bytes_per_second"]

        for key in b:
            if key not in (
                "name",
                "run_name",
                "run_type",
                "aggregate_name",
                "repetitions",
                "repetition_index",
                "threads",
                "cpu_time",
                "real_time",
                "time_unit",
                "iterations",
                "items_per_second",
                "bytes_per_second",
            ):
                entry[key] = b[key]

        structured[category][allocator].append(entry)

    return {k: dict(v) for k, v in structured.items()}


def _detect_unexpected_findings(
    data: dict[str, dict[str, list[dict]]],
) -> list[str]:
    findings: list[str] = []

    single = data.get("Single Alloc/Dealloc", {})
    std_times = {}
    frame_times = {}
    pool_times = {}
    freelist_times = {}

    for alloc_name, entries in single.items():
        for e in entries:
            key = e["name"]
            ns = e["cpu_time_ns"]
            if "Std" in alloc_name:
                std_times[key] = ns
            elif "Frame" in alloc_name:
                frame_times[key] = ns
            elif "Pool" in alloc_name:
                pool_times[key] = ns
            elif "FreeList" in alloc_name:
                freelist_times[key] = ns

    for name, frame_ns in frame_times.items():
        size_match = re.search(r"/(\d+)/", name)
        if not size_match:
            continue
        std_key = name.replace("Frame", "Std")
        if std_key in std_times and frame_ns > std_times[std_key] * 1.1:
            findings.append(
                "Frame SLOWER than malloc for {}: {:.0f} ns vs {:.0f} ns".format(
                    name, frame_ns, std_times[std_key]
                )
            )

    pool_boundary = data.get("Pool Boundary", {})
    for alloc_name, entries in pool_boundary.items():
        times_by_size: dict[int, float] = {}
        for e in entries:
            size_match = re.search(r"/(\d+)$", e["name"])
            if size_match:
                times_by_size[int(size_match.group(1))] = e["cpu_time_ns"]
        if 1024 in times_by_size and 1025 in times_by_size:
            ratio = times_by_size[1025] / times_by_size[1024]
            if ratio > 2.0:
                findings.append(
                    "Pool boundary cliff: 1025 bytes is {:.1f}x slower than 1024 bytes".format(
                        ratio
                    )
                )

    dispatch = data.get("MM Dispatch", {})
    for alloc_name, entries in dispatch.items():
        if "Typed" in alloc_name or "Direct" in alloc_name:
            continue
        for e in entries:
            typed_key = e["name"].replace(
                "BM_MMDispatch_" + alloc_name, "BM_MMDispatch_Typed"
            )
            direct_key = e["name"].replace(
                "BM_MMDispatch_" + alloc_name, "BM_DirectFreeList"
            )
            for candidate_alloc, candidate_entries in dispatch.items():
                for ce in candidate_entries:
                    if ce["name"] == direct_key:
                        overhead = (e["cpu_time_ns"] - ce["cpu_time_ns"]) / max(
                            ce["cpu_time_ns"], 1
                        )
                        if overhead > 0.1:
                            findings.append(
                                "MM dispatch overhead >{:.0f}% for {}".format(
                                    overhead * 100, e["name"]
                                )
                            )

    for alloc_name, entries in single.items():
        if "Pool" not in alloc_name:
            continue
        for e in entries:
            freelist_key = e["name"].replace("Pool", "FreeList")
            for fl_alloc, fl_entries in single.items():
                if "FreeList" not in fl_alloc:
                    continue
                for fe in fl_entries:
                    if fe["name"] == freelist_key:
                        if e["cpu_time_ns"] > fe["cpu_time_ns"] * 1.1:
                            findings.append(
                                "Pool slower than FreeList for {}: {:.0f} vs {:.0f} ns".format(
                                    e["name"],
                                    e["cpu_time_ns"],
                                    fe["cpu_time_ns"],
                                )
                            )

    return findings


def _render_ascii_chart(
    series: dict[str, list[tuple[float, float]]],
    x_label: str = "scale",
    y_label: str = "time (ns)",
) -> list[str]:
    if not series or all(len(pts) == 0 for pts in series.values()):
        return ["  (no data)"]

    all_x = []
    all_y = []
    for pts in series.values():
        for x, y in pts:
            all_x.append(x)
            all_y.append(y)

    if not all_x:
        return ["  (no data)"]

    x_min = min(all_x)
    x_max = max(all_x)
    y_min = min(all_y)
    y_max = max(all_y)

    if y_max <= y_min:
        y_max = y_min + 1

    use_log_x = (x_max / max(x_min, 1e-9)) > 100

    def x_to_col(x: float) -> int:
        if use_log_x:
            if x <= 0:
                return 0
            log_min = math.log10(max(x_min, 1e-9))
            log_max = math.log10(max(x_max, 1e-9))
            if log_max <= log_min:
                return 0
            frac = (math.log10(x) - log_min) / (log_max - log_min)
        else:
            if x_max <= x_min:
                return 0
            frac = (x - x_min) / (x_max - x_min)
        return max(0, min(_CHART_WIDTH - 1, int(frac * (_CHART_WIDTH - 1))))

    def y_to_row(y: float) -> int:
        frac = (y - y_min) / (y_max - y_min)
        return max(
            0,
            min(_CHART_HEIGHT - 1, _CHART_HEIGHT - 1 - int(frac * (_CHART_HEIGHT - 1))),
        )

    grid = [[" "] * _CHART_WIDTH for _ in range(_CHART_HEIGHT)]

    lines: list[str] = []
    legend_parts = []
    for idx, (name, pts) in enumerate(series.items()):
        marker = _MARKERS[idx % len(_MARKERS)]
        legend_parts.append("{} = {}".format(marker, name))
        for x, y in pts:
            col = x_to_col(x)
            row = y_to_row(y)
            grid[row][col] = marker

    y_fmt_max = "{:.0f}".format(y_max)
    y_fmt_min = "{:.0f}".format(y_min)
    y_pad = max(len(y_fmt_max), len(y_fmt_min))

    lines.append("  " + " " * y_pad + " " + y_label)
    for r in range(_CHART_HEIGHT):
        if r == 0:
            label = y_fmt_max.rjust(y_pad)
        elif r == _CHART_HEIGHT - 1:
            label = y_fmt_min.rjust(y_pad)
        else:
            label = " " * y_pad
        lines.append("  {} |{}".format(label, "".join(grid[r])))

    x_axis = "  " + " " * y_pad + " +" + "-" * _CHART_WIDTH
    lines.append(x_axis)

    x_fmt_min = "{:.0f}".format(x_min)
    x_fmt_max = "{:.0f}".format(x_max)
    scale_label = "log" if use_log_x else "linear"
    x_footer = (
        "  "
        + " " * y_pad
        + "  "
        + x_fmt_min
        + " " * max(1, _CHART_WIDTH - len(x_fmt_min) - len(x_fmt_max))
        + x_fmt_max
        + "  ({} {})".format(x_label, scale_label)
    )
    lines.append(x_footer)

    if legend_parts:
        lines.append("  Legend: " + "  ".join(legend_parts))

    return lines


def format_markdown(
    context: dict,
    data: dict[str, dict[str, list[dict]]],
    findings: list[str],
) -> str:
    lines: list[str] = [
        "## Allocator Benchmark Report",
        "",
        "**CPU**: {} x {} MHz".format(
            context.get("num_cpus", "?"),
            context.get("mhz_per_cpu", "?"),
        ),
        "**Date**: {}".format(context.get("date", "unknown")),
        "",
    ]

    total_benchmarks = sum(len(e) for cat in data.values() for e in cat.values())
    lines.append("**Total benchmarks**: {}".format(total_benchmarks))
    lines.append("")

    if findings:
        lines.append("### Unexpected Findings")
        lines.append("")
        for f in findings:
            lines.append("- {}".format(f))
        lines.append("")

    lines.append("### Summary by Category")
    lines.append("")
    lines.append("| Category | Allocators | Benchmarks | Fastest |")
    lines.append("|----------|-----------|------------|---------|")

    for cat_name in [c[0] for c in _CATEGORY_PATTERNS] + ["Other"]:
        if cat_name not in data:
            continue
        cat = data[cat_name]
        alloc_names = sorted(cat.keys())
        total = sum(len(e) for e in cat.values())

        best_alloc = ""
        best_time = float("inf")
        for aname, entries in cat.items():
            avg = sum(e["cpu_time_ns"] for e in entries) / max(len(entries), 1)
            if avg < best_time:
                best_time = avg
                best_alloc = aname

        lines.append(
            "| {} | {} | {} | {} |".format(
                cat_name,
                ", ".join(alloc_names),
                total,
                best_alloc,
            )
        )

    lines.append("")

    for cat_name in [c[0] for c in _CATEGORY_PATTERNS] + ["Other"]:
        if cat_name not in data:
            continue
        cat = data[cat_name]
        lines.append("### {}".format(cat_name))
        lines.append("")

        lines.append("| Benchmark | Allocator | CPU Time (ns) | Items/sec |")
        lines.append("|-----------|-----------|---------------|-----------|")

        all_entries = []
        for aname, entries in sorted(cat.items()):
            for e in entries:
                all_entries.append((aname, e))

        all_entries.sort(key=lambda x: x[1].get("cpu_time_ns", 0))

        for aname, e in all_entries:
            items_sec = e.get("items_per_second", 0)
            items_str = "{:.0f}".format(items_sec) if items_sec else "-"
            short_name = e["name"]
            if len(short_name) > 50:
                short_name = short_name[:47] + "..."
            lines.append(
                "| {} | {} | {:.1f} | {} |".format(
                    short_name,
                    aname,
                    e["cpu_time_ns"],
                    items_str,
                )
            )

        chart_series: dict[str, list[tuple[float, float]]] = defaultdict(list)
        for aname, entries in sorted(cat.items()):
            for e in entries:
                size_match = re.search(r"/(\d+)(?:/|$)", e["name"])
                if size_match:
                    x_val = float(size_match.group(1))
                    chart_series[aname].append((x_val, e["cpu_time_ns"]))

        if chart_series and any(len(v) > 1 for v in chart_series.values()):
            lines.append("")
            lines.append("```")
            for chart_line in _render_ascii_chart(chart_series):
                lines.append(chart_line)
            lines.append("```")

        lines.append("")

    return "\n".join(lines)


def main(argv: list[str]) -> int:
    args = parse_args(argv)

    if not args.input.exists():
        print(
            "Error: file not found: {}".format(args.input),
            file=sys.stderr,
        )
        return 1

    output_dir = args.output_dir or args.input.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    json_out = args.json_out or (output_dir / "allocator_report.json")
    md_out = args.md_out or (output_dir / "allocator_report.md")

    context, benchmarks = load_benchmarks(args.input)
    data = build_structured_data(benchmarks)
    findings = _detect_unexpected_findings(data)

    json_report = {
        "context": context,
        "categories": data,
        "findings": findings,
    }
    json_out.write_text(json.dumps(json_report, indent=2, default=str))
    print("JSON report: {}".format(json_out), file=sys.stderr)

    md_text = format_markdown(context, data, findings)
    md_out.write_text(md_text)
    print("Markdown report: {}".format(md_out), file=sys.stderr)

    if findings:
        print("\nUnexpected findings:", file=sys.stderr)
        for f in findings:
            print("  - {}".format(f), file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
