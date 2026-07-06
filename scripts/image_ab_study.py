#!/usr/bin/env python3
"""Reduce blind-judge image A/B study rows into detection deltas."""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.image_ab_study/v1"
RESULT_SCHEMA_VERSION = "dart.image_ab_study_results/v1"
DEFECT_LABEL = "defect"
CONTROL_LABELS = {"clean", "control", "none"}
OBSERVED_LABELS = {"defect", "clean", "uncertain"}
EXPECTED_LABELS = CONTROL_LABELS | {DEFECT_LABEL}


def _load_manifest(path: Path) -> Mapping[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ValueError(f"{path}: expected a JSON object")
    schema = data.get("schema_version")
    if schema != SCHEMA_VERSION:
        raise ValueError(f"{path}: unsupported schema_version {schema!r}")
    rows = data.get("rows")
    if not isinstance(rows, list) or not rows:
        raise ValueError(f"{path}: expected a non-empty rows list")
    return data


def _expect_string(row: Mapping[str, Any], name: str) -> str:
    value = row.get(name)
    if not isinstance(value, str) or not value:
        raise ValueError(f"study row missing non-empty string field {name!r}: {row!r}")
    return value


def _is_control(expected: str) -> bool:
    return expected.lower() in CONTROL_LABELS


def reduce_study(
    manifest: Mapping[str, Any],
    *,
    baseline: str,
) -> dict[str, Any]:
    rows = manifest["rows"]
    variant_stats: dict[str, dict[str, Any]] = defaultdict(
        lambda: {
            "defect_trials": 0,
            "control_trials": 0,
            "detections": 0,
            "false_positives": 0,
            "uncertain": 0,
            "cases": set(),
            "judges": set(),
        }
    )

    for raw_row in rows:
        if not isinstance(raw_row, Mapping):
            raise ValueError(f"study rows must be objects, got {raw_row!r}")
        case_id = _expect_string(raw_row, "case")
        variant = _expect_string(raw_row, "variant")
        judge = _expect_string(raw_row, "judge")
        expected = _expect_string(raw_row, "expected").lower()
        observed = _expect_string(raw_row, "observed").lower()
        if observed not in OBSERVED_LABELS:
            raise ValueError(
                f"row {case_id}/{variant}/{judge}: observed must be one of "
                f"{sorted(OBSERVED_LABELS)}, got {observed!r}"
            )
        if expected not in EXPECTED_LABELS:
            raise ValueError(
                f"row {case_id}/{variant}/{judge}: expected must be one of "
                f"{sorted(EXPECTED_LABELS)}, got {expected!r}"
            )

        stats = variant_stats[variant]
        stats["cases"].add(case_id)
        stats["judges"].add(judge)
        if observed == "uncertain":
            stats["uncertain"] += 1
        if _is_control(expected):
            stats["control_trials"] += 1
            if observed == DEFECT_LABEL:
                stats["false_positives"] += 1
        else:
            stats["defect_trials"] += 1
            if observed == DEFECT_LABEL:
                stats["detections"] += 1

    if baseline not in variant_stats:
        raise ValueError(f"baseline variant {baseline!r} has no rows")

    baseline_detection = _rate(
        variant_stats[baseline]["detections"],
        variant_stats[baseline]["defect_trials"],
    )
    variants: dict[str, Any] = {}
    for variant, stats in sorted(variant_stats.items()):
        detection_rate = _rate(stats["detections"], stats["defect_trials"])
        false_positive_rate = _rate(stats["false_positives"], stats["control_trials"])
        variants[variant] = {
            "defect_trials": stats["defect_trials"],
            "control_trials": stats["control_trials"],
            "detections": stats["detections"],
            "false_positives": stats["false_positives"],
            "uncertain": stats["uncertain"],
            "case_count": len(stats["cases"]),
            "judge_count": len(stats["judges"]),
            "detection_rate": detection_rate,
            "false_positive_rate": false_positive_rate,
            "detection_delta_vs_baseline": (
                None
                if detection_rate is None or baseline_detection is None
                else detection_rate - baseline_detection
            ),
        }

    return {
        "schema_version": RESULT_SCHEMA_VERSION,
        "study_id": str(manifest.get("study_id", "")),
        "baseline": baseline,
        "variants": variants,
        "pass": True,
    }


def _rate(numerator: int, denominator: int) -> float | None:
    if denominator == 0:
        return None
    return numerator / denominator


def render_markdown(results: Mapping[str, Any]) -> str:
    lines = [
        f"# Image A/B Study Results: {results.get('study_id', '')}".rstrip(),
        "",
        f"Baseline: `{results['baseline']}`",
        "",
        "| Variant | Defect trials | Detection rate | Delta | Control trials | False positive rate | Uncertain |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for variant, stats in results["variants"].items():
        lines.append(
            "| "
            + " | ".join(
                [
                    f"`{variant}`",
                    str(stats["defect_trials"]),
                    _format_rate(stats["detection_rate"]),
                    _format_delta(stats["detection_delta_vs_baseline"]),
                    str(stats["control_trials"]),
                    _format_rate(stats["false_positive_rate"]),
                    str(stats["uncertain"]),
                ]
            )
            + " |"
        )
    lines.append("")
    return "\n".join(lines)


def _format_rate(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.1%}"


def _format_delta(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:+.1%}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--baseline", default="single")
    parser.add_argument("--out", type=Path, help="write JSON results here")
    parser.add_argument("--markdown", type=Path, help="write a Markdown table here")
    args = parser.parse_args(argv)

    try:
        results = reduce_study(_load_manifest(args.manifest), baseline=args.baseline)
    except (OSError, ValueError) as exc:
        print(f"image_ab_study.py: {exc}", file=sys.stderr)
        return 2

    output = json.dumps(results, indent=2, sort_keys=True) + "\n"
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(output, encoding="utf-8")
    else:
        print(output, end="")

    if args.markdown is not None:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(render_markdown(results), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
