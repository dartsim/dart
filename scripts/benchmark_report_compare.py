#!/usr/bin/env python3
"""Compare committed benchmark reports against a base ref and emit Markdown."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

COMMENT_MARKER = "<!-- dart-benchmark-report -->"

UNIT_SCALE = {
    "ns": 1e-9,
    "us": 1e-6,
    "ms": 1e-3,
    "s": 1.0,
}


@dataclass(frozen=True)
class ReportInfo:
    path: Path
    slug: str
    runner_id: str


@dataclass(frozen=True)
class BenchmarkRow:
    name: str
    base_seconds: float
    new_seconds: float


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-ref", required=True, help="Git ref to compare against.")
    parser.add_argument("--report", action="append", default=[], help="Report file path.")
    parser.add_argument(
        "--report-list",
        type=Path,
        help="Optional file with newline-delimited report paths.",
    )
    parser.add_argument("--output", type=Path, help="Write Markdown output to file.")
    return parser.parse_args(argv)


def _git_show(ref: str, path: str) -> str:
    result = subprocess.run(
        ["git", "show", f"{ref}:{path}"],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout


def _git_ls_tree(ref: str, prefix: str) -> list[str]:
    result = subprocess.run(
        ["git", "ls-tree", "-r", "--name-only", ref, "--", prefix],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def _load_report(path: Path) -> dict:
    return json.loads(path.read_text())


def _load_report_from_git(ref: str, path: str) -> dict:
    return json.loads(_git_show(ref, path))


def _report_info(path: Path) -> ReportInfo | None:
    parts = path.parts
    try:
        idx = parts.index("benchmarks")
    except ValueError:
        return None
    if idx + 3 >= len(parts) or parts[idx + 1] != "reports":
        return None
    slug = parts[idx + 2]
    runner_id = parts[idx + 3]
    return ReportInfo(path=path, slug=slug, runner_id=runner_id)


def _time_seconds(entry: dict) -> float | None:
    value = entry.get("real_time")
    unit = entry.get("time_unit", "ns")
    if value is None:
        value = entry.get("cpu_time")
    if value is None:
        return None
    scale = UNIT_SCALE.get(unit)
    if scale is None:
        return None
    return float(value) * scale


def _format_seconds(value: float) -> str:
    if value < 0:
        return "n/a"
    for unit, scale in UNIT_SCALE.items():
        converted = value / scale
        if converted < 1000 or unit == "s":
            return f"{converted:.3g} {unit}"
    return f"{value:.3g} s"


def _bench_key(entry: dict) -> str | None:
    name = entry.get("name")
    if not name:
        return None
    aggregate = entry.get("aggregate_name")
    if aggregate:
        return f"{name}/{aggregate}"
    return name


def _extract_benchmarks(report: dict) -> dict[str, dict]:
    benchmarks: dict[str, dict] = {}
    for entry in report.get("benchmarks", []):
        if entry.get("error_occurred"):
            continue
        key = _bench_key(entry)
        if not key:
            continue
        benchmarks[key] = entry
    return benchmarks


def _compare(base: dict, new: dict) -> tuple[list[BenchmarkRow], list[str]]:
    base_bench = _extract_benchmarks(base)
    new_bench = _extract_benchmarks(new)
    rows: list[BenchmarkRow] = []
    missing: list[str] = []
    for name in sorted(new_bench):
        if name not in base_bench:
            missing.append(name)
            continue
        base_time = _time_seconds(base_bench[name])
        new_time = _time_seconds(new_bench[name])
        if base_time is None or new_time is None:
            missing.append(name)
            continue
        rows.append(BenchmarkRow(name=name, base_seconds=base_time, new_seconds=new_time))
    return rows, missing


def _delta_percent(base: float, new: float) -> float | None:
    if base <= 0:
        return None
    return (new - base) / base * 100.0


def _find_baseline_path(base_ref: str, info: ReportInfo) -> str | None:
    prefix = f"benchmarks/reports/{info.slug}/{info.runner_id}"
    candidates = _git_ls_tree(base_ref, prefix)
    if not candidates:
        return None
    return sorted(candidates)[-1]


def _load_report_paths(args: argparse.Namespace) -> list[Path]:
    paths = [Path(p) for p in args.report]
    if args.report_list and args.report_list.exists():
        for line in args.report_list.read_text().splitlines():
            line = line.strip()
            if line:
                paths.append(Path(line))
    return paths


def _build_markdown(base_ref: str, reports: Iterable[Path]) -> str:
    report_paths = list(dict.fromkeys(reports))
    if not report_paths:
        return ""

    lines = [COMMENT_MARKER, "## Benchmark report comparison", "", "Delta < 0 means faster.", ""]

    for path in report_paths:
        info = _report_info(path)
        if info is None:
            lines.append(f"### `{path}`")
            lines.append("- Skipped (unrecognized report path).")
            lines.append("")
            continue

        lines.append(f"### `{info.slug}` (runner: `{info.runner_id}`)")
        lines.append(f"- Report: `{path}`")

        baseline_path = _find_baseline_path(base_ref, info)
        if not baseline_path:
            lines.append(f"- No baseline found in `{base_ref}` for this runner.")
            lines.append("")
            continue

        lines.append(f"- Baseline: `{baseline_path}`")
        lines.append("")

        base_report = _load_report_from_git(base_ref, baseline_path)
        new_report = _load_report(path)
        rows, missing = _compare(base_report, new_report)

        if not rows:
            lines.append("No comparable benchmarks found.")
            lines.append("")
            continue

        lines.append("| Benchmark | Baseline | PR | Delta |")
        lines.append("| --- | --- | --- | --- |")
        for row in rows:
            delta = _delta_percent(row.base_seconds, row.new_seconds)
            delta_str = "n/a" if delta is None else f"{delta:+.2f}%"
            lines.append(
                "| "
                + " | ".join(
                    [
                        row.name,
                        _format_seconds(row.base_seconds),
                        _format_seconds(row.new_seconds),
                        delta_str,
                    ]
                )
                + " |"
            )

        if missing:
            lines.append("")
            lines.append(
                "Missing baseline entries: " + ", ".join(f"`{name}`" for name in missing)
            )
        lines.append("")

    return "\n".join(lines).rstrip() + "\n"


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    report_paths = _load_report_paths(args)
    markdown = _build_markdown(args.base_ref, report_paths)
    if args.output:
        args.output.write_text(markdown)
    else:
        print(markdown)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
