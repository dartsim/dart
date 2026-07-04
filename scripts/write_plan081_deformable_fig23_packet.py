#!/usr/bin/env python3
"""Write a PLAN-081 deformable Fig. 23-shaped statistics packet from benchmark JSON.

The packet distils the ``bm_deformable_body`` Google Benchmark JSON into the
per-scene statistics axes the IPC paper reports in Fig. 23 / Table 1 (per-step
Newton and CG effort, assembled sparse-Hessian footprint, per-step wall time,
and the active-contact statistics), for the DART-runnable scenes only. It is a
shape-parity scaffold, not paper parity: see ``paper_scale`` /
``limitation_status`` in the corpus manifest.
"""

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
    benchmark_timing_ns,
    canonical_benchmark_name,
)

DEFAULT_MANIFEST = Path(
    "docs/plans/081-deformable-implicit-barrier-solver/"
    "fig23_deformable_statistics_corpus.json"
)
DEFAULT_OUTPUT = Path(
    ".benchmark_results/plan081/deformable_fig23_statistics_packet.json"
)
PACKET_KEY = "plan081_deformable_fig23_statistics_packet"

KNOWN_FAMILIES = ("solver-scaling", "contact-statistics")

ROW_STRING_FIELDS = (
    "row_id",
    "benchmark",
    "scene_id",
    "family",
    "solver_variant",
    "invariant",
)


class Plan081DeformableFig23PacketError(RuntimeError):
    pass


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        required=True,
        help="Google Benchmark JSON emitted by bm_deformable_body "
        "(--benchmark_format=json).",
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
        raise Plan081DeformableFig23PacketError(f"{path} root must be a JSON object")
    return data


def load_manifest(path: Path) -> dict[str, Any]:
    manifest = _load_json(path)
    schema_version = manifest.get("schema_version")
    if (
        not isinstance(schema_version, int)
        or isinstance(schema_version, bool)
        or schema_version <= 0
    ):
        raise Plan081DeformableFig23PacketError(
            f"{path}.schema_version must be a positive integer"
        )
    if not isinstance(manifest.get("paper_scale"), bool):
        raise Plan081DeformableFig23PacketError(f"{path}.paper_scale must be a boolean")
    if manifest["paper_scale"]:
        raise Plan081DeformableFig23PacketError(
            f"{path}.paper_scale must be false: this packet is a shape-parity "
            "scaffold, not IPC paper parity"
        )
    for field in (
        "limitation_status",
        "invariant_semantics",
        "reference",
        "benchmark_binary",
    ):
        if not isinstance(manifest.get(field), str) or not manifest[field]:
            raise Plan081DeformableFig23PacketError(
                f"{path}.{field} must be a non-empty string"
            )
    rows = manifest.get("rows")
    if not isinstance(rows, list) or not rows:
        raise Plan081DeformableFig23PacketError(f"{path} has no manifest rows")
    seen_ids: set[str] = set()
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise Plan081DeformableFig23PacketError(
                f"{path}.rows[{index}] must be an object"
            )
        for field in ROW_STRING_FIELDS:
            if not isinstance(row.get(field), str) or not row[field]:
                raise Plan081DeformableFig23PacketError(
                    f"{path}.rows[{index}].{field} must be a non-empty string"
                )
        if row["family"] not in KNOWN_FAMILIES:
            raise Plan081DeformableFig23PacketError(
                f"{path}.rows[{index}].family must be one of "
                f"{', '.join(KNOWN_FAMILIES)}"
            )
        if row["row_id"] in seen_ids:
            raise Plan081DeformableFig23PacketError(
                f"{path} has a duplicate row_id: {row['row_id']}"
            )
        seen_ids.add(row["row_id"])
        fields = row.get("metric_fields")
        if not isinstance(fields, list) or not fields:
            raise Plan081DeformableFig23PacketError(
                f"{path}.rows[{index}].metric_fields must be a non-empty list"
            )
        for field in fields:
            if not isinstance(field, str) or not field:
                raise Plan081DeformableFig23PacketError(
                    f"{path}.rows[{index}].metric_fields entries must be "
                    "non-empty strings"
                )
        for optional in ("positive_fields", "zero_fields"):
            values = row.get(optional, [])
            if not isinstance(values, list):
                raise Plan081DeformableFig23PacketError(
                    f"{path}.rows[{index}].{optional} must be a list"
                )
            for field in values:
                if not isinstance(field, str) or not field:
                    raise Plan081DeformableFig23PacketError(
                        f"{path}.rows[{index}].{optional} entries must be "
                        "non-empty strings"
                    )
        # A strictly-positive gate only makes sense on a collected metric.
        for field in row.get("positive_fields", []):
            if field not in fields:
                raise Plan081DeformableFig23PacketError(
                    f"{path}.rows[{index}].positive_fields entry {field!r} "
                    "must also appear in metric_fields"
                )
    return manifest


def _finite_number(row: Mapping[str, Any], field: str) -> float | None:
    value = row.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _row_metrics(
    benchmark_row: Mapping[str, Any],
    name: str,
    fields: Sequence[str],
    positive_fields: Sequence[str],
    zero_fields: Sequence[str],
) -> dict[str, float]:
    metrics: dict[str, float] = {}
    errors: list[str] = []
    for field in fields:
        value = _finite_number(benchmark_row, field)
        if value is None:
            errors.append(f"missing/non-finite counter {field}")
        elif value < 0.0:
            errors.append(f"negative counter {field}")
        else:
            metrics[field] = value
    for field in positive_fields:
        # positive_fields is a validated subset of metric_fields, so the value is
        # absent here only when it already failed the missing/negative gate above.
        if field in metrics and metrics[field] <= 0.0:
            errors.append(f"non-positive counter {field} (must be > 0)")
    for field in zero_fields:
        value = _finite_number(benchmark_row, field)
        if value is None:
            errors.append(f"missing/non-finite zero-counter {field}")
        elif value != 0.0:
            errors.append(f"nonzero counter {field} (must be 0)")
        else:
            metrics[field] = value
    if errors:
        raise Plan081DeformableFig23PacketError(f"{name}: " + "; ".join(errors))
    return metrics


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
        raise Plan081DeformableFig23PacketError(
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
        timing_errors = benchmark_timing_field_errors(benchmark_row, benchmark_name)
        if timing_errors:
            errors.extend(timing_errors)
            continue
        try:
            metrics = _row_metrics(
                benchmark_row,
                benchmark_name,
                manifest_row["metric_fields"],
                manifest_row.get("positive_fields", []),
                manifest_row.get("zero_fields", []),
            )
        except Plan081DeformableFig23PacketError as exc:
            errors.append(str(exc))
            continue
        packet_rows.append(
            {
                "row_id": manifest_row["row_id"],
                "scene_id": manifest_row["scene_id"],
                "family": manifest_row["family"],
                "solver_variant": manifest_row["solver_variant"],
                "invariant": manifest_row["invariant"],
                "benchmark": benchmark_name,
                "per_step_time_ns": benchmark_timing_ns(benchmark_row),
                "metrics": metrics,
            }
        )

    if errors:
        raise Plan081DeformableFig23PacketError("\n".join(errors))

    return {
        PACKET_KEY: {
            "schema_version": manifest["schema_version"],
            "paper_scale": manifest["paper_scale"],
            "limitation_status": manifest["limitation_status"],
            "invariant_semantics": manifest["invariant_semantics"],
            "reference": manifest["reference"],
            "benchmark_binary": manifest["benchmark_binary"],
            "metric_source": "DeformableSolverDiagnostics "
            "(World::getLastDeformableSolverDiagnostics) and "
            "DeformableDynamicsStage::getLastStats, surfaced as bm_deformable_body "
            "benchmark counters; 'nodes' is the fixture node count",
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
    except Plan081DeformableFig23PacketError as exc:
        raise SystemExit(str(exc)) from exc

    write_packet(args.output, packet)
    print(
        "Wrote PLAN-081 deformable Fig-23 statistics packet: "
        f"{args.output} ({packet[PACKET_KEY]['row_count']} rows)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
