#!/usr/bin/env python3
"""Validate native collision upstream-superset status artifacts."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

_DEFAULT_INVENTORY = Path("docs/plans/035-native-collision/upstream-inventory.md")
_DEFAULT_CASE_MAP = Path("docs/plans/035-native-collision/upstream-case-map.md")
_DEFAULT_BENCHMARK_MANIFEST = Path(
    "docs/plans/035-native-collision/benchmark-manifest.md"
)
_UNRESOLVED_CASE_STATUSES = {
    "fixture-needed",
    "mapping-needed",
    "new-benchmark-needed",
    "new-test-needed",
}
_CASE_STATUSES = {"covered", "not-applicable"} | _UNRESOLVED_CASE_STATUSES
_BENCHMARK_STATUSES = {"lead", "behind", "non-comparable", "needs-rerun"}
_REQUIRED_NATIVE_SCOPES = {
    ("FCL", "FCL test tree"),
    ("Bullet", "Bullet collision unit test"),
    ("Bullet", "Bullet benchmark examples"),
    ("Bullet", "Bullet OpenCL collision kernels"),
    ("ODE", "ODE collision unit tests"),
    ("ODE", "ODE demos"),
    ("ODE", "ODE libccd tests"),
    ("ODE", "ODE libccd benchmarks"),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inventory", type=Path, default=_DEFAULT_INVENTORY)
    parser.add_argument("--case-map", type=Path, default=_DEFAULT_CASE_MAP)
    parser.add_argument(
        "--benchmark-manifest", type=Path, default=_DEFAULT_BENCHMARK_MANIFEST
    )
    parser.add_argument(
        "--max-behind",
        type=int,
        default=0,
        help="Maximum allowed strict benchmark rows behind the strongest reference.",
    )
    parser.add_argument(
        "--max-needs-rerun",
        type=int,
        default=0,
        help="Maximum allowed benchmark rows marked needs-rerun.",
    )
    return parser.parse_args()


def _table_cells(line: str) -> list[str] | None:
    stripped = line.strip()
    if not stripped.startswith("|") or not stripped.endswith("|"):
        return None
    cells = [cell.strip() for cell in stripped.strip("|").split("|")]
    if not cells or all(set(cell) <= {"-", ":"} for cell in cells if cell):
        return None
    return cells


def _unquote(value: str) -> str:
    value = value.strip()
    if value.startswith("`") and value.endswith("`"):
        return value[1:-1]
    return value


def _section_lines(text: str, heading: str, next_level: str = "## ") -> list[str]:
    lines = text.splitlines()
    start = None
    for index, line in enumerate(lines):
        if line.strip() == heading:
            start = index + 1
            break
    if start is None:
        raise ValueError(f"Missing section: {heading}")

    end = len(lines)
    for index in range(start, len(lines)):
        if lines[index].startswith(next_level):
            end = index
            break
    return lines[start:end]


def _parse_summary_counts(text: str) -> Counter[str]:
    counts: Counter[str] = Counter()
    for line in _section_lines(text, "## Summary"):
        cells = _table_cells(line)
        if cells is None or len(cells) < 2 or cells[0].lower() == "status":
            continue
        counts[_unquote(cells[0])] += int(cells[1])
    return counts


def _source_revisions(text: str) -> dict[str, str]:
    revisions: dict[str, str] = {}
    for line in _section_lines(text, "## Sources"):
        cells = _table_cells(line)
        if cells is None or len(cells) < 3 or cells[0].lower() == "project":
            continue
        revisions[cells[0]] = _unquote(cells[2])
    return revisions


def _inventory_scope_rows(text: str) -> list[tuple[str, str, str, int, int, int]]:
    rows = []
    for line in _section_lines(text, "## Summary"):
        cells = _table_cells(line)
        if cells is None or len(cells) < 6 or cells[0].lower() == "project":
            continue
        rows.append(
            (
                cells[0],
                cells[1],
                cells[2],
                int(cells[3]),
                int(cells[4]),
                int(cells[5]),
            )
        )
    return rows


def _case_rows(text: str) -> list[list[str]]:
    rows: list[list[str]] = []
    for line in _section_lines(text, "## Rows"):
        cells = _table_cells(line)
        if cells is None or len(cells) < 7 or cells[0].lower() == "status":
            continue
        rows.append(cells)
    return rows


def _manifest_rows(text: str) -> tuple[list[list[str]], list[list[str]]]:
    comparable = []
    native_only = []
    for line in _section_lines(text, "## Comparable Rows"):
        cells = _table_cells(line)
        if cells is None or len(cells) < 8 or cells[0].lower() == "status":
            continue
        comparable.append(cells)
    for line in _section_lines(text, "## Native-Only Rows"):
        cells = _table_cells(line)
        if cells is None or len(cells) < 6 or cells[0].lower() == "status":
            continue
        native_only.append(cells)
    return comparable, native_only


def _check_revisions(name: str, text: str, errors: list[str]) -> None:
    revisions = _source_revisions(text)
    missing = {"FCL", "Bullet", "ODE"} - set(revisions)
    if missing:
        errors.append(f"{name}: missing source rows for {', '.join(sorted(missing))}")
    for project, revision in revisions.items():
        if not revision or revision == "missing":
            errors.append(f"{name}: {project} revision is missing")


def check_inventory(path: Path, errors: list[str]) -> None:
    text = path.read_text()
    _check_revisions(path.as_posix(), text, errors)

    scope_rows = _inventory_scope_rows(text)
    present_scopes = {(project, scope) for project, scope, *_ in scope_rows}
    missing_scopes = _REQUIRED_NATIVE_SCOPES - present_scopes
    if missing_scopes:
        formatted = ", ".join(
            f"{project}/{scope}" for project, scope in sorted(missing_scopes)
        )
        errors.append(f"{path}: missing required upstream scopes: {formatted}")

    for project, scope, native_scope, source_files, cases, benchmarks in scope_rows:
        if (
            "native-collision" not in native_scope
            and "catalogued outside" not in native_scope
        ):
            errors.append(f"{path}: scope lacks classification: {project}/{scope}")
        if source_files <= 0:
            errors.append(f"{path}: no source files recorded for {project}/{scope}")
        if "correctness superset" in native_scope and cases <= 0:
            errors.append(
                f"{path}: no correctness cases recorded for {project}/{scope}"
            )
        if "benchmark superset" in native_scope and benchmarks <= 0:
            errors.append(f"{path}: no benchmark cases recorded for {project}/{scope}")


def check_case_map(path: Path, errors: list[str]) -> None:
    text = path.read_text()
    _check_revisions(path.as_posix(), text, errors)

    rows = _case_rows(text)
    row_counts: Counter[str] = Counter(_unquote(row[0]) for row in rows)
    summary_counts = _parse_summary_counts(text)
    if row_counts != summary_counts:
        errors.append(
            f"{path}: summary counts {summary_counts} != row counts {row_counts}"
        )

    unknown_statuses = set(row_counts) - _CASE_STATUSES
    if unknown_statuses:
        errors.append(f"{path}: unknown case statuses: {sorted(unknown_statuses)}")

    unresolved = {
        status: count
        for status, count in row_counts.items()
        if status in _UNRESOLVED_CASE_STATUSES and count
    }
    if unresolved:
        errors.append(f"{path}: unresolved upstream rows remain: {unresolved}")

    projects = {row[1] for row in rows}
    if {"FCL", "Bullet", "ODE"} - projects:
        errors.append(f"{path}: rows do not cover all FCL/Bullet/ODE projects")
    if row_counts["covered"] <= 0:
        errors.append(f"{path}: no covered rows recorded")
    if row_counts["not-applicable"] <= 0:
        errors.append(f"{path}: no non-applicability decisions recorded")


def check_benchmark_manifest(
    path: Path, max_behind: int, max_needs_rerun: int, errors: list[str]
) -> None:
    text = path.read_text()
    comparable, native_only = _manifest_rows(text)
    row_counts: Counter[str] = Counter(_unquote(row[0]) for row in comparable)
    row_counts.update(_unquote(row[0]) for row in native_only)
    summary_counts = _parse_summary_counts(text)
    if row_counts != summary_counts:
        errors.append(
            f"{path}: summary counts {summary_counts} != row counts {row_counts}"
        )

    unknown_statuses = set(row_counts) - _BENCHMARK_STATUSES
    if unknown_statuses:
        errors.append(f"{path}: unknown benchmark statuses: {sorted(unknown_statuses)}")

    if row_counts["needs-rerun"] > max_needs_rerun:
        errors.append(
            f"{path}: {row_counts['needs-rerun']} rows need rerun "
            f"(allowed {max_needs_rerun})"
        )
    if row_counts["behind"] > max_behind:
        errors.append(
            f"{path}: {row_counts['behind']} strict rows behind strongest reference "
            f"(allowed {max_behind})"
        )
    if row_counts["lead"] <= 0:
        errors.append(f"{path}: no strict native benchmark leads recorded")
    if row_counts["non-comparable"] <= 0:
        errors.append(f"{path}: no native-only benchmark rows recorded")


def main() -> int:
    args = parse_args()
    errors: list[str] = []
    check_inventory(args.inventory, errors)
    check_case_map(args.case_map, errors)
    check_benchmark_manifest(
        args.benchmark_manifest, args.max_behind, args.max_needs_rerun, errors
    )
    if errors:
        for error in errors:
            print(error)
        return 1

    print("Native collision upstream-superset status check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
