"""Shared Google Benchmark packet helpers for DART evidence scripts."""

from __future__ import annotations

import math
import re
from collections.abc import Iterable, Mapping
from typing import Any

_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}
SOLVER_SUBPHASE_TIMINGS_KEY = "solver_subphase_timings_ns"


def canonical_benchmark_name(name: str) -> str:
    """Strip Google Benchmark repeat and aggregate suffixes from a row name."""
    return _AGGREGATE_SUFFIX_RE.sub("", _REPEATS_SUFFIX_RE.sub("", name))


def benchmark_row_name(row: Mapping[str, Any]) -> str:
    """Return the stable Google Benchmark row name, preferring run_name."""
    name = row.get("run_name", row.get("name", ""))
    return name if isinstance(name, str) else ""


def timing_to_ns(value: float, unit: str) -> float:
    return value * _UNIT_TO_NS.get(unit, 1.0)


def benchmark_timing_ns(row: Mapping[str, Any]) -> float:
    value = row.get("real_time", row.get("cpu_time"))
    if value is None:
        return math.nan
    try:
        return timing_to_ns(float(value), str(row.get("time_unit", "ns")))
    except TypeError, ValueError:
        return math.nan


def median_timing_by_name(rows: Iterable[Mapping[str, Any]]) -> dict[str, float]:
    timings: dict[str, float] = {}
    for row in rows:
        if row.get("aggregate_name") != "median":
            continue
        name = canonical_benchmark_name(benchmark_row_name(row))
        if not name:
            continue
        timing = benchmark_timing_ns(row)
        if math.isfinite(timing) and timing > 0.0:
            timings[name] = timing
    return timings


def benchmark_timing_field_errors(
    row: Mapping[str, Any], name: str, fields: Iterable[str] = ("real_time", "cpu_time")
) -> list[str]:
    errors: list[str] = []
    for field in fields:
        value = row.get(field)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            errors.append(f"{name} has non-finite {field}: {value!r}")
        elif not math.isfinite(value):
            errors.append(f"{name} has non-finite {field}: {value!r}")
        elif value <= 0:
            errors.append(f"{name} has non-positive {field}: {value!r}")
    if not isinstance(row.get("time_unit"), str):
        errors.append(f"{name} is missing time_unit")
    return errors


def benchmark_packet_timing_schema_errors(
    metadata: Mapping[str, Any],
    packet_name: str,
    *,
    required_subphases: Iterable[str] = (),
) -> list[str]:
    errors: list[str] = []
    step_count = metadata.get("step_count")
    if not isinstance(step_count, int) or isinstance(step_count, bool):
        errors.append(f"{packet_name}.step_count must be an integer")
    elif step_count <= 0:
        errors.append(f"{packet_name}.step_count must be positive")

    required = tuple(required_subphases)
    subphases = metadata.get(SOLVER_SUBPHASE_TIMINGS_KEY)
    if subphases is None:
        if required:
            errors.append(f"{packet_name}.{SOLVER_SUBPHASE_TIMINGS_KEY} is missing")
        return errors
    if not isinstance(subphases, Mapping):
        errors.append(f"{packet_name}.{SOLVER_SUBPHASE_TIMINGS_KEY} must be an object")
        return errors

    for subphase in required:
        if subphase not in subphases:
            errors.append(
                f"{packet_name}.{SOLVER_SUBPHASE_TIMINGS_KEY}.{subphase} is missing"
            )

    for subphase, value in sorted(subphases.items(), key=lambda item: str(item[0])):
        if not isinstance(subphase, str) or not subphase:
            errors.append(
                f"{packet_name}.{SOLVER_SUBPHASE_TIMINGS_KEY} has an invalid key"
            )
            continue
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            errors.append(
                f"{packet_name}.{SOLVER_SUBPHASE_TIMINGS_KEY}.{subphase} "
                f"must be numeric"
            )
            continue
        if not math.isfinite(float(value)) or float(value) < 0.0:
            errors.append(
                f"{packet_name}.{SOLVER_SUBPHASE_TIMINGS_KEY}.{subphase} "
                f"must be finite and non-negative"
            )
    return errors


def batched_benchmark_row_schema_errors(
    row: Mapping[str, Any], packet_name: str
) -> list[str]:
    """Return schema errors for a PLAN-091 batched benchmark packet row."""

    errors: list[str] = []
    backend = row.get("backend")
    if not isinstance(backend, str) or not backend:
        errors.append(f"{packet_name}.backend must be a non-empty string")

    precision = row.get("precision")
    if not isinstance(precision, str) or not precision:
        errors.append(f"{packet_name}.precision must be a non-empty string")

    includes_transfer_time = row.get("includes_transfer_time")
    if not isinstance(includes_transfer_time, bool):
        errors.append(f"{packet_name}.includes_transfer_time must be a boolean")

    lane_count = row.get("lane_count")
    if not isinstance(lane_count, int) or isinstance(lane_count, bool):
        errors.append(f"{packet_name}.lane_count must be an integer")
    elif lane_count <= 0:
        errors.append(f"{packet_name}.lane_count must be positive")

    execution_shape = row.get("resolved_execution_shape")
    if not isinstance(execution_shape, str) or not execution_shape:
        errors.append(
            f"{packet_name}.resolved_execution_shape must be a non-empty string"
        )

    step_count = row.get("step_count")
    if not isinstance(step_count, int) or isinstance(step_count, bool):
        errors.append(f"{packet_name}.step_count must be an integer")
    elif step_count <= 0:
        errors.append(f"{packet_name}.step_count must be positive")

    return errors
