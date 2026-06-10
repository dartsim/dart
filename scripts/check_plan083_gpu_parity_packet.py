#!/usr/bin/env python3
"""Validate PLAN-083's private GPU parity packet manifest."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from collections.abc import Mapping
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PACKET = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "083-unified-newton-barrier-multibody"
    / "gpu-parity-packet.json"
)

VALID_STATUSES = {"planned", "in-progress", "measured", "not-applicable"}
EXPECTED_ROW_IDS = {
    "assembly-linear-solve",
    "barrier-friction-local-kernels",
    "ccd-line-search",
    "contact-stencils-candidate-filtering",
    "psd-projection",
    "scene-parity-speedup",
}
REQUIRED_FIELDS = {
    "row_id",
    "roadmap_item",
    "owner",
    "status",
    "cpu_evidence_command",
    "gpu_evidence_command",
    "parity_evidence_artifact",
    "benchmark_profile_artifact",
    "same_scene_policy",
    "tolerance_policy",
    "timing_policy",
    "speedup_policy",
    "public_api_policy",
    "limitation_status",
    "notes_or_gap",
}
REQUIRED_TIMING_KEYS = {
    "setup",
    "host_to_device",
    "kernel",
    "solve",
    "device_to_host",
    "readback",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--packet", type=Path, default=DEFAULT_PACKET)
    return parser.parse_args()


def load_packet(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{path}: invalid JSON: {exc}") from exc


def _is_missing(value: object) -> bool:
    return value is None or value == "" or value == []


def _numeric(value: object) -> float | None:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _validate_measured_row(row: Mapping[str, Any], row_id: str) -> list[str]:
    errors: list[str] = []

    if row.get("same_scene_cpu_gpu") is not True:
        errors.append(f"{row_id}: measured rows require same_scene_cpu_gpu=true")

    max_error = _numeric(row.get("max_result_abs_error"))
    tolerance = _numeric(row.get("result_abs_error_tolerance"))
    if max_error is None:
        errors.append(f"{row_id}: measured rows need max_result_abs_error")
    if tolerance is None or tolerance <= 0.0:
        errors.append(
            f"{row_id}: measured rows need positive result_abs_error_tolerance"
        )
    if max_error is not None and tolerance is not None and max_error > tolerance:
        errors.append(
            f"{row_id}: result error {max_error:.3g} exceeds tolerance {tolerance:.3g}"
        )

    speedup = _numeric(row.get("speedup"))
    min_speedup = _numeric(row.get("min_speedup"))
    if speedup is None:
        errors.append(f"{row_id}: measured rows need speedup")
    if min_speedup is None or min_speedup <= 0.0:
        errors.append(f"{row_id}: measured rows need positive min_speedup")
    if speedup is not None and min_speedup is not None and speedup < min_speedup:
        errors.append(
            f"{row_id}: speedup {speedup:.3f} is below required {min_speedup:.3f}"
        )

    timing = row.get("timing_ns")
    if not isinstance(timing, Mapping):
        errors.append(f"{row_id}: measured rows need timing_ns")
    else:
        missing = sorted(REQUIRED_TIMING_KEYS - set(timing.keys()))
        if missing:
            errors.append(f"{row_id}: timing_ns missing keys {missing}")
        for key, value in sorted(timing.items(), key=lambda item: str(item[0])):
            if key not in REQUIRED_TIMING_KEYS:
                errors.append(f"{row_id}: timing_ns has unexpected key {key!r}")
                continue
            numeric = _numeric(value)
            if numeric is None or numeric < 0.0:
                errors.append(f"{row_id}: timing_ns.{key} must be non-negative")

    return errors


def validate_packet(packet: dict[str, Any]) -> list[str]:
    errors: list[str] = []

    if packet.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    if not isinstance(packet.get("source"), dict):
        errors.append("source must be an object")
    if not isinstance(packet.get("policy"), dict):
        errors.append("policy must be an object")

    rows = packet.get("rows")
    if not isinstance(rows, list):
        errors.append("rows must be a list")
        return errors

    row_ids: list[str] = []
    status_counts: Counter[str] = Counter()

    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            errors.append(f"row {index}: row must be an object")
            continue

        missing = REQUIRED_FIELDS - row.keys()
        if missing:
            errors.append(f"row {index}: missing fields {sorted(missing)}")
            continue

        row_id = row.get("row_id")
        if not isinstance(row_id, str) or not row_id:
            errors.append(f"row {index}: row_id must be a non-empty string")
            continue
        row_ids.append(row_id)

        status = row.get("status")
        if status not in VALID_STATUSES:
            errors.append(f"{row_id}: invalid status {status!r}")
        else:
            status_counts[str(status)] += 1

        for field in REQUIRED_FIELDS - {"row_id", "status"}:
            if _is_missing(row.get(field)):
                errors.append(f"{row_id}: missing {field}")

        public_api_policy = str(row.get("public_api_policy", "")).lower()
        if "no public" not in public_api_policy:
            errors.append(f"{row_id}: public_api_policy must state no public exposure")

        if status == "measured":
            errors.extend(_validate_measured_row(row, row_id))
        elif _is_missing(row.get("limitation_status")) or _is_missing(
            row.get("notes_or_gap")
        ):
            errors.append(f"{row_id}: non-measured rows need limitation rationale")

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

    summary = packet.get("summary")
    if isinstance(summary, dict):
        if summary.get("row_count") != len(rows):
            errors.append(
                f"summary.row_count must be {len(rows)}, "
                f"got {summary.get('row_count')!r}"
            )
        if summary.get("status_counts") != dict(sorted(status_counts.items())):
            errors.append("summary.status_counts does not match packet rows")
    else:
        errors.append("summary must be an object")

    return errors


def main() -> int:
    args = parse_args()
    packet = load_packet(args.packet)
    errors = validate_packet(packet)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print(f"PLAN-083 GPU parity packet manifest OK: {len(packet.get('rows', []))} rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
