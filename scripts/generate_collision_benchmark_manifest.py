#!/usr/bin/env python3
"""Generate the native collision performance-wave benchmark manifest."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable

_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}
_NATIVE_BACKEND = "Native"
_DEFAULT_OUTPUT = Path("docs/plans/035-native-collision/benchmark-manifest.md")
_DEFAULT_INPUTS = [
    Path(".benchmark_results/collision_check_adapter.json"),
    Path(".benchmark_results/collision_check_distance.json"),
    Path(".benchmark_results/collision_check_mesh.json"),
    Path(".benchmark_results/collision_check_mixed.json"),
    Path(".benchmark_results/collision_check_narrow.json"),
    Path(".benchmark_results/collision_check_narrow_adapter.json"),
    Path(".benchmark_results/collision_check_narrow_raw_reference.json"),
    Path(".benchmark_results/collision_check_raycast.json"),
    Path(".benchmark_results/collision_check_raycast_batch.json"),
    Path(".benchmark_results/collision_check_stacked_scenes.json"),
]


@dataclass
class Measurement:
    median_ns: float | None = None
    mean_ns: float | None = None
    stddev_ns: float | None = None
    repetitions: int | None = None


@dataclass
class BenchmarkRow:
    key: str
    taxonomy_family: str
    source: Path
    measurements: dict[str, Measurement] = field(default_factory=dict)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        action="append",
        default=[],
        help="Google Benchmark JSON input. Defaults to the official collision-check packets.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=_DEFAULT_OUTPUT,
        help=f"Markdown manifest output path. Defaults to {_DEFAULT_OUTPUT}.",
    )
    return parser.parse_args()


def _to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def _row_name(row: dict) -> str:
    return str(row.get("run_name") or row.get("name") or "")


def _comparison_key(name: str) -> tuple[str, str] | None:
    head, separator, params = name.partition("/")
    prefix, backend_separator, backend = head.rpartition("_")
    if not backend_separator:
        return None
    key = prefix
    if separator:
        key += separator + params
    return key, backend


def _display_key(key: str) -> str:
    return key.replace("RawReference", "RawComparison")


def _taxonomy_family(key: str) -> str:
    if key.startswith("BM_Distance_"):
        return "Distance Queries"
    if key.startswith("BM_Raycast_") or key.startswith("BM_Scenario_RaycastBatch_"):
        return "Ray And Cast Queries"
    if key.startswith("BM_DartAdapter_") or key.startswith("BM_NarrowPhaseAdapter_"):
        return "Public Adapter And Package Path"
    if key.startswith("BM_Scenario_MeshHeavy"):
        return "Mesh, Convex, And Field Heavy Scenes"
    if key.startswith("BM_Scenario_MixedPrimitives_"):
        return "Broadphase And Pair Generation"
    if key.startswith(
        (
            "BM_BoxGridStack_",
            "BM_CompoundBoxStructures_",
            "BM_ConvexHullTerrainSnapshot_",
            "BM_ElongatedConvexHullStack_",
            "BM_HaltonConvexCellPack_",
            "BM_MixedPrimitiveStackWithTerrain_",
            "BM_ScaledPrimitiveStack_",
        )
    ):
        return "Stacked And Terrain Profiling"
    if key.startswith("BM_NarrowPhase_") or key.startswith(
        "BM_NarrowPhaseRawReference_"
    ):
        return "Pair Narrow Phase"
    return "Unclassified"


def _collect_inputs(inputs: list[Path]) -> list[Path]:
    if inputs:
        return sorted(inputs)
    official_inputs = [path for path in _DEFAULT_INPUTS if path.exists()]
    if official_inputs:
        return official_inputs
    return sorted(Path(".benchmark_results").glob("collision_check*.json"))


def _ensure_measurement(
    rows: dict[str, BenchmarkRow], key: str, backend: str, source: Path
) -> Measurement:
    if key not in rows:
        rows[key] = BenchmarkRow(
            key=key,
            taxonomy_family=_taxonomy_family(key),
            source=source,
        )
    return rows[key].measurements.setdefault(backend, Measurement())


def _collect_backend_groups(paths: Iterable[Path]) -> dict[str, set[str]]:
    groups: dict[str, set[str]] = {}
    for path in paths:
        data = json.loads(path.read_text(encoding="utf-8"))
        for row in data.get("benchmarks", []):
            run_type = row.get("run_type")
            if run_type == "iteration":
                continue
            aggregate = row.get("aggregate_name")
            if run_type == "aggregate" and aggregate not in {
                "mean",
                "median",
                "stddev",
            }:
                continue
            comparison = _comparison_key(_row_name(row))
            if comparison is None:
                continue
            key, backend = comparison
            groups.setdefault(key, set()).add(backend)
    return groups


def _is_backend_row(
    comparison: tuple[str, str] | None, backend_groups: dict[str, set[str]]
) -> bool:
    if comparison is None:
        return False
    key, backend = comparison
    if backend == _NATIVE_BACKEND:
        return True
    group = backend_groups.get(key, set())
    return _NATIVE_BACKEND in group and len(group) > 1


def _load_rows(paths: Iterable[Path]) -> tuple[dict[str, BenchmarkRow], list[dict]]:
    rows: dict[str, BenchmarkRow] = {}
    contexts: list[dict] = []
    input_paths = list(paths)
    backend_groups = _collect_backend_groups(input_paths)

    for path in input_paths:
        data = json.loads(path.read_text(encoding="utf-8"))
        contexts.append({"path": path, **data.get("context", {})})

        for row in data.get("benchmarks", []):
            run_type = row.get("run_type")
            if run_type == "iteration":
                continue

            aggregate = row.get("aggregate_name")
            if run_type == "aggregate" and aggregate not in {
                "mean",
                "median",
                "stddev",
            }:
                continue

            name = _row_name(row)
            if not name:
                continue

            comparison = _comparison_key(name)
            if not _is_backend_row(comparison, backend_groups):
                key = name
                backend = _NATIVE_BACKEND
            else:
                key, backend = comparison

            measurement = _ensure_measurement(rows, key, backend, path)
            value_ns = _to_ns(
                float(row.get("cpu_time", 0.0)), str(row.get("time_unit", "ns"))
            )
            if aggregate == "mean":
                measurement.mean_ns = value_ns
            elif aggregate == "median":
                measurement.median_ns = value_ns
            elif aggregate == "stddev":
                measurement.stddev_ns = value_ns
            else:
                measurement.median_ns = value_ns
            if row.get("repetitions") is not None:
                measurement.repetitions = int(row["repetitions"])

    return rows, contexts


def _format_time(ns: float | None) -> str:
    if ns is None:
        return "-"
    if ns < 1_000:
        return f"{ns:.3g} ns"
    if ns < 1_000_000:
        return f"{ns / 1_000:.3g} us"
    return f"{ns / 1_000_000:.3g} ms"


def _row_status(row: BenchmarkRow) -> tuple[str, float | None, Measurement | None]:
    native = row.measurements.get(_NATIVE_BACKEND)
    if native is None or native.median_ns is None:
        return "needs-rerun", None, None

    comparison_measurements = [
        measurement
        for backend, measurement in row.measurements.items()
        if backend != _NATIVE_BACKEND and measurement.median_ns is not None
    ]
    if not comparison_measurements:
        return "non-comparable", None, None

    strongest = min(
        comparison_measurements,
        key=lambda measurement: measurement.median_ns or float("inf"),
    )
    if strongest.median_ns is None or strongest.median_ns <= 0:
        return "needs-rerun", None, strongest

    ratio = native.median_ns / strongest.median_ns
    return ("lead" if ratio < 1.0 else "behind"), ratio, strongest


def _unique_context_values(contexts: list[dict], key: str) -> list[object]:
    values: list[object] = []
    for context in contexts:
        value = context.get(key)
        if value is None or value in values:
            continue
        values.append(value)
    return values


def _format_context_values(values: list[object], default: str = "unknown") -> str:
    if not values:
        return default
    if len(values) == 1:
        return str(values[0])
    return ", ".join(str(value) for value in values)


def _affinity_description() -> str:
    if not hasattr(os, "sched_getaffinity"):
        return "unknown"
    return ", ".join(str(cpu) for cpu in sorted(os.sched_getaffinity(0)))


def _context_lines(contexts: list[dict]) -> list[str]:
    source_paths = ", ".join(f"`{context['path'].as_posix()}`" for context in contexts)
    build_types = _format_context_values(
        _unique_context_values(contexts, "library_build_type")
    )
    cpu_descriptions = [
        f"{context.get('num_cpus', 'unknown')} x {context.get('mhz_per_cpu', 'unknown')} MHz"
        for context in contexts
    ]
    cpu_description = _format_context_values(list(dict.fromkeys(cpu_descriptions)))
    cpu_scaling = _format_context_values(
        _unique_context_values(contexts, "cpu_scaling_enabled")
    )
    aslr = _format_context_values(_unique_context_values(contexts, "aslr_enabled"))
    threads = "1"
    return [
        f"- Generated: `{datetime.now().astimezone().isoformat(timespec='seconds')}`",
        "- Generator: `scripts/generate_collision_benchmark_manifest.py`",
        f"- JSON sources: {source_paths}",
        "- Metric: `cpu_time` median, with mean/stddev retained for variance context.",
        f"- Build type: `{build_types}`",
        f"- Host CPU: `{cpu_description}`",
        f"- Benchmark worker threads: `{threads}` per Google Benchmark row.",
        f"- Process CPU affinity when manifest was generated: `{_affinity_description()}`",
        f"- Profiling instrumentation: `DART_BUILD_PROFILE={os.environ.get('DART_BUILD_PROFILE', 'unknown')}`",
        f"- CPU scaling enabled: `{cpu_scaling}`",
        f"- ASLR enabled: `{aslr}`",
        "- Status caveat: this manifest is generated from ignored local JSON. "
        "Rerun the collision benchmark guard before accepting new performance "
        "claims; the current host still reports CPU scaling enabled.",
    ]


def _write_manifest(
    rows: dict[str, BenchmarkRow], contexts: list[dict], output: Path
) -> None:
    def sort_key(row: BenchmarkRow) -> tuple[str, float, str]:
        status, ratio, _ = _row_status(row)
        return status, -(ratio or 0), row.key

    ordered_rows = sorted(rows.values(), key=sort_key)
    counts: dict[str, int] = {}
    for row in ordered_rows:
        status, _, _ = _row_status(row)
        counts[status] = counts.get(status, 0) + 1

    lines = [
        "# Native Collision Benchmark Manifest",
        "",
        "<!-- Generated by scripts/generate_collision_benchmark_manifest.py; do not edit by hand. -->",
        "",
        "This manifest is generated from Google Benchmark JSON outputs and groups rows",
        "by DART-owned feature/algorithm family. It intentionally reports only the",
        "strongest comparison timing, not the comparison implementation name.",
        "",
        "## Run Policy",
        "",
        *_context_lines(contexts),
        "",
        "## Summary",
        "",
        "| Status | Rows |",
        "| ------ | ---- |",
    ]
    for status in ("lead", "behind", "non-comparable", "needs-rerun"):
        lines.append(f"| `{status}` | {counts.get(status, 0)} |")

    lines.extend(
        [
            "",
            "## Comparable Rows",
            "",
            "| Status | Family | Benchmark row | Native median | Native stddev | Strongest comparison median | Ratio | Source |",
            "| ------ | ------ | ------------- | ------------- | ------------- | --------------------------- | ----- | ------ |",
        ]
    )

    for row in ordered_rows:
        status, ratio, strongest = _row_status(row)
        native = row.measurements.get(_NATIVE_BACKEND)
        if status == "non-comparable":
            continue
        ratio_text = "-" if ratio is None else f"{ratio:.3f}"
        lines.append(
            "| "
            + " | ".join(
                [
                    f"`{status}`",
                    row.taxonomy_family,
                    f"`{_display_key(row.key)}`",
                    _format_time(native.median_ns if native else None),
                    _format_time(native.stddev_ns if native else None),
                    _format_time(strongest.median_ns if strongest else None),
                    ratio_text,
                    f"`{row.source.as_posix()}`",
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Native-Only Rows",
            "",
            "| Status | Family | Benchmark row | Native median | Native stddev | Source |",
            "| ------ | ------ | ------------- | ------------- | ------------- | ------ |",
        ]
    )
    for row in ordered_rows:
        status, _, _ = _row_status(row)
        if status != "non-comparable":
            continue
        native = row.measurements.get(_NATIVE_BACKEND)
        lines.append(
            "| "
            + " | ".join(
                [
                    "`non-comparable`",
                    row.taxonomy_family,
                    f"`{_display_key(row.key)}`",
                    _format_time(native.median_ns if native else None),
                    _format_time(native.stddev_ns if native else None),
                    f"`{row.source.as_posix()}`",
                ]
            )
            + " |"
        )

    if not output.parent.exists():
        raise SystemExit(f"Output directory does not exist: {output.parent}")
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    inputs = _collect_inputs(args.input)
    if not inputs:
        raise SystemExit("No collision benchmark JSON inputs found.")
    rows, contexts = _load_rows(inputs)
    _write_manifest(rows, contexts, args.output)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
