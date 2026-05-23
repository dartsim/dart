#!/usr/bin/env python3
"""Merge Google Benchmark JSON files into a single report.

Each benchmark target emits its own ``--benchmark_out`` JSON file. The
performance dashboard action consumes one file, so this concatenates the
``benchmarks`` arrays from every input into a single Google Benchmark report.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _iter_inputs(inputs: list[Path]) -> list[Path]:
    files: list[Path] = []
    for entry in inputs:
        if entry.is_dir():
            files.extend(sorted(entry.glob("*.json")))
        else:
            files.append(entry)
    return files


def merge(inputs: list[Path]) -> dict:
    files = _iter_inputs(inputs)
    context: dict | None = None
    benchmarks: list[dict] = []
    for path in files:
        data = json.loads(path.read_text(encoding="utf-8"))
        if context is None and isinstance(data.get("context"), dict):
            context = data["context"]
        benchmarks.extend(data.get("benchmarks", []))
    if not benchmarks:
        raise SystemExit("No benchmark rows found in inputs.")
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
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    merged = merge(args.inputs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(merged, indent=2) + "\n", encoding="utf-8")
    print(f"Merged {len(merged['benchmarks'])} rows into {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
