#!/usr/bin/env python3
"""Compare two DART scene JSON dumps and emit a structural verdict."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "dart.scene_diff/v1"
DEFAULT_ABS_TOL = 1e-9


def _path_text(path: Sequence[str | int]) -> str:
    text = "$"
    for item in path:
        if isinstance(item, int):
            text += f"[{item}]"
        else:
            text += f".{item}"
    return text


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _ignored(path: Sequence[str | int], *, ignore_world_time: bool) -> bool:
    if not ignore_world_time:
        return False
    return tuple(path) in {("world", "time"), ("world", "frame")}


def _add_diff(
    diffs: list[dict[str, Any]],
    path: Sequence[str | int],
    reason: str,
    actual: Any = None,
    expected: Any = None,
) -> None:
    item: dict[str, Any] = {"path": _path_text(path), "reason": reason}
    if actual is not None:
        item["actual"] = actual
    if expected is not None:
        item["expected"] = expected
    diffs.append(item)


def _compare_value(
    actual: Any,
    expected: Any,
    *,
    path: tuple[str | int, ...],
    abs_tol: float,
    ignore_world_time: bool,
    diffs: list[dict[str, Any]],
    max_diffs: int,
) -> None:
    if len(diffs) >= max_diffs or _ignored(path, ignore_world_time=ignore_world_time):
        return

    if isinstance(actual, Mapping) and isinstance(expected, Mapping):
        actual_keys = set(actual)
        expected_keys = set(expected)
        for key in sorted(actual_keys - expected_keys, key=str):
            _add_diff(diffs, (*path, str(key)), "unexpected key", actual=actual[key])
            if len(diffs) >= max_diffs:
                return
        for key in sorted(expected_keys - actual_keys, key=str):
            _add_diff(diffs, (*path, str(key)), "missing key", expected=expected[key])
            if len(diffs) >= max_diffs:
                return
        for key in sorted(actual_keys & expected_keys, key=str):
            _compare_value(
                actual[key],
                expected[key],
                path=(*path, str(key)),
                abs_tol=abs_tol,
                ignore_world_time=ignore_world_time,
                diffs=diffs,
                max_diffs=max_diffs,
            )
            if len(diffs) >= max_diffs:
                return
        return

    if (
        isinstance(actual, Sequence)
        and isinstance(expected, Sequence)
        and not isinstance(actual, (str, bytes, bytearray))
        and not isinstance(expected, (str, bytes, bytearray))
    ):
        if len(actual) != len(expected):
            _add_diff(
                diffs,
                path,
                "length mismatch",
                actual=len(actual),
                expected=len(expected),
            )
            if len(diffs) >= max_diffs:
                return
        for index, (actual_item, expected_item) in enumerate(zip(actual, expected)):
            _compare_value(
                actual_item,
                expected_item,
                path=(*path, index),
                abs_tol=abs_tol,
                ignore_world_time=ignore_world_time,
                diffs=diffs,
                max_diffs=max_diffs,
            )
            if len(diffs) >= max_diffs:
                return
        return

    if _is_number(actual) and _is_number(expected):
        actual_float = float(actual)
        expected_float = float(expected)
        if not (
            math.isfinite(actual_float)
            and math.isfinite(expected_float)
            and math.isclose(actual_float, expected_float, rel_tol=0.0, abs_tol=abs_tol)
        ):
            _add_diff(
                diffs,
                path,
                "numeric mismatch",
                actual=actual,
                expected=expected,
            )
        return

    if actual != expected:
        _add_diff(diffs, path, "value mismatch", actual=actual, expected=expected)


def compare_scene_json(
    actual: Mapping[str, Any],
    expected: Mapping[str, Any],
    *,
    abs_tol: float = DEFAULT_ABS_TOL,
    ignore_world_time: bool = True,
    max_diffs: int = 50,
    metadata: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    if max_diffs < 1:
        raise ValueError("max_diffs must be at least 1")
    diffs: list[dict[str, Any]] = []
    _compare_value(
        actual,
        expected,
        path=(),
        abs_tol=abs_tol,
        ignore_world_time=ignore_world_time,
        diffs=diffs,
        max_diffs=max_diffs,
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "metadata": dict(metadata or {}),
        "thresholds_used": {
            "abs_tol": abs_tol,
            "ignore_world_time": ignore_world_time,
            "max_diffs": max_diffs,
        },
        "diff_count": len(diffs),
        "pass": not diffs,
        "diffs": diffs,
        "reasons": [f"{diff['path']}: {diff['reason']}" for diff in diffs],
    }


def _load_json(path: Path) -> Mapping[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(data, Mapping):
        raise ValueError(f"{path}: expected a JSON object")
    return data


def _metadata(values: Sequence[str]) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"metadata must be KEY=VALUE, got {value!r}")
        key, item = value.split("=", 1)
        if not key:
            raise ValueError("metadata keys must be non-empty")
        metadata[key] = item
    return metadata


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("actual", type=Path)
    parser.add_argument("expected", type=Path)
    parser.add_argument("--abs-tol", type=float, default=DEFAULT_ABS_TOL)
    parser.add_argument(
        "--include-world-time",
        action="store_true",
        help="include world.time and world.frame in the structural comparison",
    )
    parser.add_argument("--max-diffs", type=int, default=50)
    parser.add_argument("--metadata", action="append", default=[], metavar="KEY=VALUE")
    parser.add_argument("--out", type=Path, help="write the JSON verdict here")
    args = parser.parse_args(argv)

    try:
        verdict = compare_scene_json(
            _load_json(args.actual),
            _load_json(args.expected),
            abs_tol=args.abs_tol,
            ignore_world_time=not args.include_world_time,
            max_diffs=args.max_diffs,
            metadata=_metadata(args.metadata),
        )
    except (OSError, ValueError) as exc:
        print(f"scene_diff.py: {exc}", file=sys.stderr)
        return 2

    output = json.dumps(verdict, indent=2, sort_keys=True) + "\n"
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(output, encoding="utf-8")
    else:
        print(output, end="")
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
