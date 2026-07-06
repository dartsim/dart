#!/usr/bin/env python3
"""Fail when a soft-body comparison summary contains CPU regressions."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary_json", type=Path)
    parser.add_argument(
        "--max-regression-pct",
        type=float,
        default=0.0,
        help="Largest allowed positive CPU-time change in percent.",
    )
    parser.add_argument(
        "--comparisons",
        default="current_vs_base,current_vs_parent",
        help="Comma-separated comparison names to check.",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    summary = json.loads(args.summary_json.read_text(encoding="utf-8"))
    comparison_names = {
        item.strip() for item in args.comparisons.split(",") if item.strip()
    }

    failures = []
    for item in summary.get("comparisons", []):
        if item.get("comparison") not in comparison_names:
            continue
        if "missing" in item:
            failures.append(
                "{comparison} {detector}/{scene}/{threads}: missing {missing}".format(
                    **item
                )
            )
            continue

        change = float(item["cpu_change_pct"])
        if change > args.max_regression_pct:
            failures.append(
                "{comparison} {detector}/{scene}/{threads}: "
                "CPU {change:+.3f}% current={current:.6g} ms".format(
                    comparison=item["comparison"],
                    detector=item["detector"],
                    scene=item["scene"],
                    threads=item["threads"],
                    change=change,
                    current=float(item["current_cpu_ms"]),
                )
            )

    existing_failures = list(summary.get("failures", []))
    if existing_failures:
        failures.extend(f"summary failure: {failure}" for failure in existing_failures)

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1

    print(
        "PASS: no CPU regressions above "
        f"{args.max_regression_pct:.3f}% in {', '.join(sorted(comparison_names))}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
