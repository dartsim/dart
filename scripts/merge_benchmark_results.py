#!/usr/bin/env python3
"""Merge Google Benchmark JSON files into a single dashboard report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from benchmark_display_names import humanize_name  # noqa: E402


def _iter_inputs(inputs: list[Path]) -> list[Path]:
    files: list[Path] = []
    for entry in inputs:
        if entry.is_dir():
            files.extend(
                sorted(p for p in entry.glob("*.json") if p.name != "combined.json")
            )
        else:
            files.append(entry)
    return files


def _select(benchmarks: list[dict], aggregate: str) -> list[dict]:
    if aggregate == "all":
        return benchmarks

    groups: dict[str, list[dict]] = {}
    for row in benchmarks:
        key = row.get("run_name") or row.get("name", "")
        groups.setdefault(key, []).append(row)

    selected: list[dict] = []
    for key, rows in groups.items():
        chosen = [row for row in rows if row.get("aggregate_name") == aggregate]
        if not chosen:
            chosen = [row for row in rows if row.get("run_type") != "aggregate"]
        for row in chosen:
            row = dict(row)
            row["name"] = key
            selected.append(row)
    return selected


def merge(
    inputs: list[Path], aggregate: str = "median", humanize: bool = False
) -> dict:
    context: dict | None = None
    benchmarks: list[dict] = []
    for path in _iter_inputs(inputs):
        data = json.loads(path.read_text(encoding="utf-8"))
        if context is None and isinstance(data.get("context"), dict):
            context = data["context"]
        benchmarks.extend(data.get("benchmarks", []))

    benchmarks = _select(benchmarks, aggregate)
    if not benchmarks:
        raise SystemExit("No benchmark rows found in inputs.")

    if humanize:
        for row in benchmarks:
            row["name"] = humanize_name(row.get("name", ""))

    return {"context": context or {}, "benchmarks": benchmarks}


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="Google Benchmark JSON files or directories containing them.",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        required=True,
        help="Path to write the merged Google Benchmark JSON.",
    )
    parser.add_argument(
        "--aggregate",
        choices=["median", "mean", "all"],
        default="median",
        help="Which repetition aggregate to keep per benchmark.",
    )
    parser.add_argument(
        "--humanize",
        action="store_true",
        help="Rewrite raw benchmark names into readable dashboard titles.",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    merged = merge(args.inputs, args.aggregate, humanize=args.humanize)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(merged, indent=2) + "\n", encoding="utf-8")
    print(f"Merged {len(merged['benchmarks'])} rows into {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
