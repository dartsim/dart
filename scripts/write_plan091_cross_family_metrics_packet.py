#!/usr/bin/env python3
"""Write a PLAN-091 cross-family metrics packet from benchmark JSON."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from benchmark_packet_utils import (  # noqa: E402
    benchmark_row_name,
    benchmark_timing_field_errors,
    canonical_benchmark_name,
)

DEFAULT_MANIFEST = Path(
    "docs/plans/091-architecture-hardening/cross-family-metrics-corpus.json"
)
DEFAULT_OUTPUT = Path(".benchmark_results/plan091/cross_family_metrics_packet.json")
PACKET_KEY = "plan091_cross_family_metrics_packet"

REQUIRED_METRIC_COUNTERS = (
    "step_count",
    "kinetic_energy",
    "potential_energy",
    "total_energy",
    "linear_momentum_x",
    "linear_momentum_y",
    "linear_momentum_z",
    "angular_momentum_x",
    "angular_momentum_y",
    "angular_momentum_z",
    "active_contact_count",
    "max_penetration_depth",
    "last_step_iterations",
    "last_step_residual",
)


class Plan091CrossFamilyPacketError(RuntimeError):
    pass


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        required=True,
        help="Google Benchmark JSON from bm_plan091_cross_family_corpus.",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=DEFAULT_MANIFEST,
        help=f"Expected corpus manifest (default: {DEFAULT_MANIFEST}).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"Output packet JSON path (default: {DEFAULT_OUTPUT}).",
    )
    return parser.parse_args(list(argv))


def _load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise Plan091CrossFamilyPacketError(f"{path} root must be a JSON object")
    return data


def load_manifest(path: Path) -> dict[str, Any]:
    manifest = _load_json(path)
    rows = manifest.get("rows")
    if not isinstance(rows, list) or not rows:
        raise Plan091CrossFamilyPacketError(f"{path} has no manifest rows")
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise Plan091CrossFamilyPacketError(
                f"{path}.rows[{index}] must be an object"
            )
        for field in (
            "row_id",
            "benchmark",
            "scene_id",
            "family_axis",
            "resolved_solver_identity",
            "invariant",
        ):
            if not isinstance(row.get(field), str) or not row[field]:
                raise Plan091CrossFamilyPacketError(
                    f"{path}.rows[{index}].{field} must be a non-empty string"
                )
    return manifest


def _finite_number(row: Mapping[str, Any], field: str) -> float | None:
    value = row.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _metric_counters(row: Mapping[str, Any], name: str) -> dict[str, float]:
    counters: dict[str, float] = {}
    missing: list[str] = []
    for field in REQUIRED_METRIC_COUNTERS:
        value = _finite_number(row, field)
        if value is None:
            missing.append(field)
        else:
            counters[field] = value
    if missing:
        raise Plan091CrossFamilyPacketError(
            f"{name} is missing finite StepMetrics counters: " + ", ".join(missing)
        )
    if counters["step_count"] <= 0:
        raise Plan091CrossFamilyPacketError(f"{name}.step_count must be positive")
    if counters["active_contact_count"] < 0:
        raise Plan091CrossFamilyPacketError(
            f"{name}.active_contact_count must be non-negative"
        )
    if counters["max_penetration_depth"] < 0.0:
        raise Plan091CrossFamilyPacketError(
            f"{name}.max_penetration_depth must be non-negative"
        )
    if counters["last_step_iterations"] < 0:
        raise Plan091CrossFamilyPacketError(
            f"{name}.last_step_iterations must be non-negative"
        )
    if counters["last_step_residual"] < 0.0:
        raise Plan091CrossFamilyPacketError(
            f"{name}.last_step_residual must be non-negative"
        )
    return counters


def _rows_by_canonical_name(rows: list[Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for row in rows:
        if not isinstance(row, dict):
            continue
        aggregate_name = row.get("aggregate_name")
        if aggregate_name not in {None, "median"}:
            continue
        name = canonical_benchmark_name(benchmark_row_name(row))
        if not name:
            continue
        if aggregate_name == "median" or name not in result:
            result[name] = row
    return result


def make_packet(
    benchmark_data: dict[str, Any], manifest: dict[str, Any]
) -> dict[str, Any]:
    rows = benchmark_data.get("benchmarks")
    if not isinstance(rows, list):
        raise Plan091CrossFamilyPacketError(
            "benchmark JSON is missing a benchmark row list"
        )

    by_name = _rows_by_canonical_name(rows)
    packet_rows: list[dict[str, Any]] = []
    errors: list[str] = []
    for manifest_row in manifest["rows"]:
        benchmark_name = manifest_row["benchmark"]
        benchmark_row = by_name.get(benchmark_name)
        if benchmark_row is None:
            errors.append(f"missing benchmark row: {benchmark_name}")
            continue
        errors.extend(benchmark_timing_field_errors(benchmark_row, benchmark_name))
        counters = _metric_counters(benchmark_row, benchmark_name)
        packet_rows.append(
            {
                "row_id": manifest_row["row_id"],
                "scene_id": manifest_row["scene_id"],
                "family_axis": manifest_row["family_axis"],
                "resolved_solver_identity": manifest_row["resolved_solver_identity"],
                "invariant": manifest_row["invariant"],
                "benchmark": benchmark_name,
                "metrics": counters,
            }
        )

    if errors:
        raise Plan091CrossFamilyPacketError("\n".join(errors))

    return {
        PACKET_KEY: {
            "schema_version": int(manifest.get("schema_version", 1)),
            "metric_source": "dart::simulation::World::computeStepMetrics",
            "row_count": len(packet_rows),
            "rows": packet_rows,
        },
        "benchmarks": rows,
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2) + "\n", encoding="utf-8")


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(
            _load_json(args.benchmark_json), load_manifest(args.manifest)
        )
    except Plan091CrossFamilyPacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    print(
        "Wrote PLAN-091 cross-family metrics packet: "
        f"{args.output} ({packet[PACKET_KEY]['row_count']} rows)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
