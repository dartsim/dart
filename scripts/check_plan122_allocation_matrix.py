#!/usr/bin/env python3
"""Validate PLAN-122 simulation-loop allocation coverage rows."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MATRIX = (
    REPO_ROOT
    / "docs"
    / "plans"
    / "122-simulation-loop-allocation-hardening"
    / "coverage-matrix.md"
)
DEFAULT_TEST_ROOT = REPO_ROOT / "tests"

VALID_STATUSES = {"Closed", "Steady-state", "Open", "Out of scope"}
ROW_ID_PATTERN = re.compile(r"^[A-Z]-\d{3}$")
TEST_PATTERN = re.compile(
    r"\b(?:TEST|TEST_F|TEST_P)\s*\(\s*([A-Za-z_]\w*)\s*,\s*([A-Za-z_]\w*)\s*\)",
    re.MULTILINE,
)
BACKTICK_PATTERN = re.compile(r"`([^`]+)`")


@dataclass(frozen=True)
class MatrixRow:
    line_number: int
    row_id: str
    status: str
    evidence: str


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, default=DEFAULT_MATRIX)
    parser.add_argument("--test-root", type=Path, default=DEFAULT_TEST_ROOT)
    return parser.parse_args(argv)


def parse_matrix_rows(text: str) -> list[MatrixRow]:
    rows: list[MatrixRow] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line.startswith("|"):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 6 or not ROW_ID_PATTERN.fullmatch(cells[0]):
            continue
        rows.append(
            MatrixRow(
                line_number=line_number,
                row_id=cells[0],
                status=cells[1],
                evidence=cells[3],
            )
        )
    return rows


def load_test_names(test_root: Path) -> set[str]:
    names: set[str] = set()
    for path in sorted(test_root.rglob("*")):
        if path.suffix not in {".cc", ".cpp", ".hpp"}:
            continue
        text = path.read_text(encoding="utf-8")
        for suite, test_name in TEST_PATTERN.findall(text):
            names.add(test_name)
            names.add(f"{suite}.{test_name}")
    return names


def _looks_like_test_token(token: str) -> bool:
    if "/" in token or "::" in token or token.endswith((".md", ".py", ".json")):
        return False
    if "." in token:
        suite, test_name = token.split(".", 1)
        return bool(suite) and bool(test_name) and suite[0].isupper()
    return bool(token) and token[0].isupper()


def validate_matrix(matrix: Path, test_root: Path) -> list[str]:
    if not matrix.exists():
        return [f"{matrix}: matrix file does not exist"]
    if not test_root.exists():
        return [f"{test_root}: test root does not exist"]

    rows = parse_matrix_rows(matrix.read_text(encoding="utf-8"))
    if not rows:
        return [f"{matrix}: no PLAN-122 matrix rows found"]

    errors: list[str] = []
    seen_row_ids: dict[str, int] = {}
    for row in rows:
        previous_line = seen_row_ids.get(row.row_id)
        if previous_line is not None:
            errors.append(
                f"{matrix}:{row.line_number}: duplicate row {row.row_id} "
                f"(first seen on line {previous_line})"
            )
        seen_row_ids[row.row_id] = row.line_number

        if row.status not in VALID_STATUSES:
            errors.append(
                f"{matrix}:{row.line_number}: row {row.row_id} has invalid "
                f"status {row.status!r}"
            )

    test_names = load_test_names(test_root)
    for row in rows:
        if row.status != "Closed":
            continue

        cited_tokens = BACKTICK_PATTERN.findall(row.evidence)
        cited_test_tokens = [
            token for token in cited_tokens if _looks_like_test_token(token)
        ]
        missing_tests = [
            token for token in cited_test_tokens if token not in test_names
        ]
        for token in missing_tests:
            errors.append(
                f"{matrix}:{row.line_number}: closed row {row.row_id} cites "
                f"missing test {token!r}"
            )
        if not cited_test_tokens:
            errors.append(
                f"{matrix}:{row.line_number}: closed row {row.row_id} cites "
                "no test in Current evidence"
            )

    return errors


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    errors = validate_matrix(args.matrix, args.test_root)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1

    rows = parse_matrix_rows(args.matrix.read_text(encoding="utf-8"))
    closed_rows = sum(1 for row in rows if row.status == "Closed")
    print(f"PLAN-122 allocation matrix OK: {len(rows)} rows, {closed_rows} closed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
