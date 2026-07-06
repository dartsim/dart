#!/usr/bin/env python3
"""Compare DART trajectory TSV files and emit a JSON verdict."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

SCHEMA_VERSION = "dart.trajectory_compare/v0"
_CONTACT_ALIASES = {"contacts": "contact_count"}


@dataclass(frozen=True)
class Row:
    tokens: dict[str, str]
    values: dict[str, float]

    @property
    def frame(self) -> int | None:
        value = self.values.get("frame")
        return None if value is None else int(value)

    @property
    def body(self) -> str | None:
        return self.tokens.get("body")


@dataclass(frozen=True)
class Trajectory:
    path: str
    text: str
    columns: tuple[str, ...]
    rows: tuple[Row, ...]


def _normalize_column(name: str) -> str:
    return _CONTACT_ALIASES.get(name, name)


def _load_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def parse_trajectory_text(text: str, *, path: str = "<memory>") -> Trajectory:
    columns: list[str] | None = None
    rows: list[Row] = []
    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            body = line[1:].strip()
            if body.startswith("columns:"):
                columns = [
                    _normalize_column(part)
                    for part in body[len("columns:") :].strip().split()
                ]
            continue

        if columns is None:
            raise ValueError(f"{path}:{line_number}: missing '# columns:' header")
        parts = line.split()
        if len(parts) != len(columns):
            raise ValueError(
                f"{path}:{line_number}: expected {len(columns)} columns, got "
                f"{len(parts)}"
            )
        tokens = dict(zip(columns, parts, strict=True))
        values: dict[str, float] = {}
        for column, token in tokens.items():
            if column == "body":
                continue
            try:
                values[column] = float(token)
            except ValueError as exc:
                raise ValueError(
                    f"{path}:{line_number}: column {column!r} is not numeric: "
                    f"{token!r}"
                ) from exc
        rows.append(Row(tokens=tokens, values=values))

    if columns is None:
        raise ValueError(f"{path}: missing '# columns:' header")
    return Trajectory(
        path=path,
        text=text,
        columns=tuple(columns),
        rows=tuple(rows),
    )


def load_trajectory(path: Path) -> Trajectory:
    return parse_trajectory_text(_load_text(path), path=str(path))


def _json_value(value: str | float | int | None) -> str | float | int | None:
    if isinstance(value, str):
        try:
            number = float(value)
        except ValueError:
            return value
        if math.isfinite(number):
            return int(number) if number.is_integer() else number
        return value
    return value


def _divergence(
    row: Row | None,
    column: str,
    a_value: str | float | int | None,
    b_value: str | float | int | None,
) -> dict[str, Any]:
    return {
        "frame": None if row is None else row.frame,
        "body": None if row is None else row.body,
        "column": column,
        "a": _json_value(a_value),
        "b": _json_value(b_value),
    }


def _numeric_columns(columns: Iterable[str]) -> list[str]:
    return [column for column in columns if column != "body"]


def _initial_max_abs(columns: Iterable[str]) -> dict[str, float]:
    return {column: 0.0 for column in _numeric_columns(columns)}


def _update_max_abs(max_abs: dict[str, float], a: Row, b: Row) -> None:
    for column in set(a.values).intersection(b.values):
        max_abs[column] = max(
            max_abs.get(column, 0.0),
            abs(a.values[column] - b.values[column]),
        )


def _checksum(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _first_exact_divergence(a: Trajectory, b: Trajectory) -> dict[str, Any] | None:
    if a.columns != b.columns:
        return _divergence(None, "columns", " ".join(a.columns), " ".join(b.columns))
    for row_a, row_b in zip(a.rows, b.rows, strict=False):
        if row_a.frame != row_b.frame:
            return _divergence(
                row_a,
                "frame",
                row_a.tokens.get("frame"),
                row_b.tokens.get("frame"),
            )
        if row_a.body != row_b.body:
            return _divergence(row_a, "body", row_a.body, row_b.body)
        for column in a.columns:
            if row_a.tokens.get(column) != row_b.tokens.get(column):
                return _divergence(
                    row_a,
                    column,
                    row_a.tokens.get(column),
                    row_b.tokens.get(column),
                )
    if len(a.rows) != len(b.rows):
        row = a.rows[min(len(a.rows), len(b.rows)) - 1] if a.rows and b.rows else None
        return _divergence(row, "row_count", len(a.rows), len(b.rows))
    if a.text != b.text:
        return _divergence(None, "file_bytes", _checksum(a.text), _checksum(b.text))
    return None


def _tolerance_for(
    column: str,
    a_value: float,
    b_value: float,
    abs_tolerances: dict[str, float],
    rel_tolerances: dict[str, float],
) -> float:
    abs_tol = max(abs_tolerances.get("*", 0.0), abs_tolerances.get(column, 0.0))
    rel_tol = max(rel_tolerances.get("*", 0.0), rel_tolerances.get(column, 0.0))
    return max(abs_tol, rel_tol * max(abs(a_value), abs(b_value)))


def _first_tolerance_divergence(
    a: Trajectory,
    b: Trajectory,
    *,
    abs_tolerances: dict[str, float],
    rel_tolerances: dict[str, float],
) -> dict[str, Any] | None:
    if a.columns != b.columns:
        return _divergence(None, "columns", " ".join(a.columns), " ".join(b.columns))
    for row_a, row_b in zip(a.rows, b.rows, strict=False):
        if row_a.frame != row_b.frame:
            return _divergence(row_a, "frame", row_a.frame, row_b.frame)
        if row_a.body != row_b.body:
            return _divergence(row_a, "body", row_a.body, row_b.body)
        for column in _numeric_columns(a.columns):
            delta = abs(row_a.values[column] - row_b.values[column])
            tolerance = _tolerance_for(
                column,
                row_a.values[column],
                row_b.values[column],
                abs_tolerances,
                rel_tolerances,
            )
            if delta > tolerance:
                return _divergence(
                    row_a, column, row_a.values[column], row_b.values[column]
                )
    if len(a.rows) != len(b.rows):
        row = a.rows[min(len(a.rows), len(b.rows)) - 1] if a.rows and b.rows else None
        return _divergence(row, "row_count", len(a.rows), len(b.rows))
    return None


def _max_abs_deviation(a: Trajectory, b: Trajectory) -> dict[str, float]:
    max_abs = _initial_max_abs(a.columns)
    for row_a, row_b in zip(a.rows, b.rows, strict=False):
        _update_max_abs(max_abs, row_a, row_b)
    return {column: round(value, 17) for column, value in sorted(max_abs.items())}


def derive_empirical_abs_tolerances(
    calibration: list[Trajectory], *, factor: float
) -> dict[str, float]:
    if factor < 0.0:
        raise ValueError("--calibration-factor must be non-negative")
    if len(calibration) < 2:
        return {}
    reference = calibration[0]
    max_abs = _initial_max_abs(reference.columns)
    for candidate in calibration[1:]:
        if candidate.columns != reference.columns or len(candidate.rows) != len(
            reference.rows
        ):
            raise ValueError(
                "calibration trajectories must have matching schema and rows"
            )
        for row_a, row_b in zip(reference.rows, candidate.rows, strict=True):
            if row_a.frame != row_b.frame or row_a.body != row_b.body:
                raise ValueError(
                    "calibration trajectories must have matching frame/body rows"
                )
            _update_max_abs(max_abs, row_a, row_b)
    return {column: value * factor for column, value in max_abs.items()}


def _parse_column_tolerance(values: list[str]) -> dict[str, float]:
    parsed: dict[str, float] = {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"column tolerance must be COL=VALUE, got {value!r}")
        column, text = value.split("=", 1)
        column = _normalize_column(column.strip())
        if not column:
            raise ValueError(f"empty column name in {value!r}")
        parsed[column] = float(text)
    return parsed


def compare_trajectories(
    a: Trajectory,
    b: Trajectory,
    *,
    mode: str = "exact",
    abs_tolerances: dict[str, float] | None = None,
    rel_tolerances: dict[str, float] | None = None,
    calibration: list[Trajectory] | None = None,
    calibration_factor: float = 2.0,
) -> dict[str, Any]:
    if mode not in {"exact", "tolerance"}:
        raise ValueError(f"unknown compare mode {mode!r}")

    max_abs = _max_abs_deviation(a, b)
    abs_used = dict(abs_tolerances or {})
    rel_used = dict(rel_tolerances or {})
    empirical = derive_empirical_abs_tolerances(
        calibration or [], factor=calibration_factor
    )
    for column, value in empirical.items():
        abs_used[column] = max(abs_used.get(column, 0.0), value)

    if mode == "exact":
        first = None if a.text == b.text else _first_exact_divergence(a, b)
    else:
        first = _first_tolerance_divergence(
            a, b, abs_tolerances=abs_used, rel_tolerances=rel_used
        )

    calibration_count = len(calibration or [])
    if calibration_count >= 2:
        tolerance_policy = (
            "absolute drift bands were derived from same-scene DART trajectory "
            f"calibration runs, then multiplied by {calibration_factor:g}"
        )
    else:
        tolerance_policy = (
            "no empirical calibration set was supplied; tolerance mode uses only "
            "caller-provided absolute/relative tolerances"
        )

    passed = first is None
    return {
        "schema_version": SCHEMA_VERSION,
        "mode": mode,
        "matched": passed,
        "first_divergence": first,
        "max_abs_dev": max_abs,
        "checksums": {"a": _checksum(a.text), "b": _checksum(b.text)},
        "thresholds_used": {
            "abs": {key: abs_used[key] for key in sorted(abs_used)},
            "rel": {key: rel_used[key] for key in sorted(rel_used)},
            "calibration_files": [item.path for item in calibration or []],
            "calibration_factor": calibration_factor,
            "tolerance_policy": tolerance_policy,
        },
        "pass": passed,
    }


def compare_trajectory_texts(
    a_text: str,
    b_text: str,
    *,
    mode: str = "exact",
    abs_tolerances: dict[str, float] | None = None,
    rel_tolerances: dict[str, float] | None = None,
    calibration_texts: list[str] | None = None,
    calibration_factor: float = 2.0,
) -> dict[str, Any]:
    calibration = [
        parse_trajectory_text(text, path=f"<calibration:{index}>")
        for index, text in enumerate(calibration_texts or [])
    ]
    return compare_trajectories(
        parse_trajectory_text(a_text, path="<a>"),
        parse_trajectory_text(b_text, path="<b>"),
        mode=mode,
        abs_tolerances=abs_tolerances,
        rel_tolerances=rel_tolerances,
        calibration=calibration,
        calibration_factor=calibration_factor,
    )


def _write_json(path: Path | None, payload: dict[str, Any]) -> None:
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if path is None:
        sys.stdout.write(text)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compare two DART trajectory TSV files and emit a JSON verdict."
    )
    parser.add_argument("a", type=Path)
    parser.add_argument("b", type=Path)
    parser.add_argument("--mode", choices=("exact", "tolerance"), default="exact")
    parser.add_argument("--abs-tol", type=float, default=0.0)
    parser.add_argument("--rel-tol", type=float, default=0.0)
    parser.add_argument(
        "--abs-column",
        action="append",
        default=[],
        metavar="COL=VALUE",
        help="absolute tolerance override for one column",
    )
    parser.add_argument(
        "--rel-column",
        action="append",
        default=[],
        metavar="COL=VALUE",
        help="relative tolerance override for one column",
    )
    parser.add_argument(
        "--calibration",
        action="append",
        default=[],
        type=Path,
        help="same-scene DART trajectory run for empirical drift calibration",
    )
    parser.add_argument("--calibration-factor", type=float, default=2.0)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)

    try:
        abs_tolerances = {"*": args.abs_tol}
        abs_tolerances.update(_parse_column_tolerance(args.abs_column))
        rel_tolerances = {"*": args.rel_tol}
        rel_tolerances.update(_parse_column_tolerance(args.rel_column))
        calibration = [load_trajectory(path) for path in args.calibration]
        verdict = compare_trajectories(
            load_trajectory(args.a),
            load_trajectory(args.b),
            mode=args.mode,
            abs_tolerances=abs_tolerances,
            rel_tolerances=rel_tolerances,
            calibration=calibration,
            calibration_factor=args.calibration_factor,
        )
        _write_json(args.out, verdict)
    except (OSError, ValueError) as exc:
        print(f"trajectory_compare.py: {exc}", file=sys.stderr)
        return 2
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
