"""Shared Google Benchmark packet helpers for DART evidence scripts."""

from __future__ import annotations

import math
import re
from collections.abc import Iterable, Mapping
from typing import Any

_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")
_UNIT_TO_NS = {"ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}


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
    except (TypeError, ValueError):
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
