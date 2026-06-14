#!/usr/bin/env python3
"""Validate PLAN-083's CPU scene corpus manifest."""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "083-unified-newton-barrier-multibody"
    / "cpu-scene-corpus.json"
)

VALID_STATUSES = {"planned", "in-progress", "manual", "not-applicable"}
VALID_TARGET_TYPES = {
    "benchmark-manifest",
    "benchmark-report",
    "comparison-benchmark",
    "not-applicable",
    "py-demo",
    "py-demo+benchmark",
    "scaling-benchmark",
}
REQUIRED_FIELDS = {
    "row_id",
    "source_ref",
    "dart_target_type",
    "priority",
    "status",
    "py_demo_category",
    "py_demo_scene_ids",
    "smoke_command",
    "long_horizon_visual_command",
    "visual_evidence_artifact",
    "benchmark_command",
    "benchmark_profile_artifact",
    "expected_invariant",
    "limitation_status",
    "notes_or_gap",
}
EXPECTED_ROW_IDS = {
    "abd-bullet-large",
    "abd-bullet-medium",
    "abd-bullet-small",
    "abd-chain-16",
    "abd-chain-8",
    "abd-chain-96",
    "abd-complex-geometry",
    "abd-demo-suite",
    "abd-fem-coupling",
    "abd-future-gpu",
    "abd-gears",
    "abd-vs-rigid-cards",
    "abd-vs-rigid-wreck",
    "unb-alg-barriers",
    "unb-fig-01",
    "unb-fig-02",
    "unb-fig-03",
    "unb-fig-04",
    "unb-fig-10",
    "unb-fig-11",
    "unb-fig-13",
    "unb-fig-20",
    "unb-fig-22",
    "unb-fig-23",
    "unb-fig-24",
    "unb-fig-25",
    "unb-table-02",
}
PY_DEMO_ROW_IDS = {
    "abd-complex-geometry",
    "abd-demo-suite",
    "abd-fem-coupling",
    "unb-fig-01",
    "unb-fig-02",
    "unb-fig-03",
    "unb-fig-04",
    "unb-fig-10",
    "unb-fig-11",
    "unb-fig-13",
    "unb-fig-20",
    "unb-fig-22",
    "unb-fig-23",
    "unb-fig-25",
}
BENCHMARK_ROW_IDS = {
    "abd-bullet-large",
    "abd-bullet-medium",
    "abd-bullet-small",
    "abd-chain-16",
    "abd-chain-8",
    "abd-chain-96",
    "abd-complex-geometry",
    "abd-fem-coupling",
    "abd-gears",
    "abd-vs-rigid-cards",
    "abd-vs-rigid-wreck",
    "unb-alg-barriers",
    "unb-fig-01",
    "unb-fig-02",
    "unb-fig-03",
    "unb-fig-04",
    "unb-fig-10",
    "unb-fig-11",
    "unb-fig-13",
    "unb-fig-20",
    "unb-fig-22",
    "unb-fig-23",
    "unb-fig-24",
    "unb-fig-25",
    "unb-table-02",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    return parser.parse_args()


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text())
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{path}: invalid JSON: {exc}") from exc


def _is_missing(value: object) -> bool:
    return value is None or value == "" or value == []


def validate_manifest(manifest: dict[str, Any]) -> list[str]:
    errors: list[str] = []

    if manifest.get("schema_version") != 1:
        errors.append("schema_version must be 1")

    source = manifest.get("source")
    if not isinstance(source, dict):
        errors.append("source must be an object")

    scenes = manifest.get("scenes")
    if not isinstance(scenes, list):
        errors.append("scenes must be a list")
        return errors

    row_ids: list[str] = []
    status_counts: Counter[str] = Counter()
    target_type_counts: Counter[str] = Counter()

    for index, row in enumerate(scenes):
        if not isinstance(row, dict):
            errors.append(f"scene {index}: row must be an object")
            continue

        missing = REQUIRED_FIELDS - row.keys()
        if missing:
            errors.append(f"scene {index}: missing fields {sorted(missing)}")
            continue

        row_id = row.get("row_id")
        if not isinstance(row_id, str) or not row_id:
            errors.append(f"scene {index}: row_id must be a non-empty string")
            continue
        row_ids.append(row_id)

        status = row.get("status")
        if status not in VALID_STATUSES:
            errors.append(f"{row_id}: invalid status {status!r}")
        else:
            status_counts[str(status)] += 1

        target_type = row.get("dart_target_type")
        if target_type not in VALID_TARGET_TYPES:
            errors.append(f"{row_id}: invalid dart_target_type {target_type!r}")
        else:
            target_type_counts[str(target_type)] += 1

        scene_ids = row.get("py_demo_scene_ids")
        if not isinstance(scene_ids, list) or not all(
            isinstance(scene_id, str) and scene_id for scene_id in scene_ids
        ):
            errors.append(f"{row_id}: py_demo_scene_ids must be a string list")

        for field in (
            "source_ref",
            "priority",
            "py_demo_category",
            "smoke_command",
            "long_horizon_visual_command",
            "visual_evidence_artifact",
            "benchmark_command",
            "benchmark_profile_artifact",
            "expected_invariant",
            "limitation_status",
            "notes_or_gap",
        ):
            if _is_missing(row.get(field)):
                errors.append(f"{row_id}: missing {field}")

        if row_id in PY_DEMO_ROW_IDS and status != "not-applicable":
            if not scene_ids:
                errors.append(f"{row_id}: py-demo row needs a scene id")
            if row.get("py_demo_category") == "not-required":
                errors.append(f"{row_id}: py-demo row needs a category")
            if not str(row.get("smoke_command", "")).startswith("pixi run py-demos"):
                errors.append(f"{row_id}: py-demo row needs a py-demos smoke")
            if not str(row.get("long_horizon_visual_command", "")).startswith(
                "pixi run py-demo-capture"
            ):
                errors.append(f"{row_id}: py-demo row needs a capture command")
            if row.get("visual_evidence_artifact") == "not-required":
                errors.append(f"{row_id}: py-demo row needs visual evidence path")

        if row_id in BENCHMARK_ROW_IDS and status != "not-applicable":
            if row.get("benchmark_command") == "not-required":
                errors.append(f"{row_id}: benchmark row needs a command")
            if row.get("benchmark_profile_artifact") == "not-required":
                errors.append(f"{row_id}: benchmark row needs a profile path")

        if status in {"manual", "not-applicable"} and not row.get("notes_or_gap"):
            errors.append(f"{row_id}: {status} row needs a rationale")

    duplicate_ids = sorted(
        row_id for row_id, count in Counter(row_ids).items() if count > 1
    )
    if duplicate_ids:
        errors.append(f"duplicate row ids: {duplicate_ids}")

    found_ids = set(row_ids)
    missing_ids = sorted(EXPECTED_ROW_IDS - found_ids)
    extra_ids = sorted(found_ids - EXPECTED_ROW_IDS)
    if missing_ids:
        errors.append(f"missing expected row ids: {missing_ids}")
    if extra_ids:
        errors.append(f"unexpected row ids: {extra_ids}")

    summary = manifest.get("summary")
    if isinstance(summary, dict):
        if summary.get("row_count") != len(scenes):
            errors.append(
                f"summary.row_count must be {len(scenes)}, "
                f"got {summary.get('row_count')!r}"
            )
        if summary.get("status_counts") != dict(sorted(status_counts.items())):
            errors.append("summary.status_counts does not match scene rows")
        if summary.get("target_type_counts") != dict(
            sorted(target_type_counts.items())
        ):
            errors.append("summary.target_type_counts does not match scene rows")
    else:
        errors.append("summary must be an object")

    return errors


def main() -> int:
    args = parse_args()
    manifest = load_manifest(args.manifest)
    errors = validate_manifest(manifest)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print(
        "PLAN-083 CPU scene corpus manifest OK: "
        f"{len(manifest.get('scenes', []))} rows"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
