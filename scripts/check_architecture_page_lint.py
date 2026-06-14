#!/usr/bin/env python3
"""Validate that every available (✅) claim in architecture.md is CI-checkable.

PLAN-091 WP-091.3: each ✅-available row in
``docs/readthedocs/architecture.md`` must cite both
  (1) a public-header API symbol that resolves in the ``dart/`` header tree, and
  (2) a DART-owned test file under ``tests/`` that exercises the claim,
so an availability marker can never drift from the code/tests that back it.

A marked row that has no real backing test is a finding to report back, not a
marker to silently downgrade (see the packet non-goals).
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DOC = REPO_ROOT / "docs" / "readthedocs" / "architecture.md"
DEFAULT_HEADER_ROOT = REPO_ROOT / "dart"

# A backtick-quoted token that looks like a public C++ API symbol: a qualified
# name (contains `::`) or a CamelCase type. A trailing `(...)`/`()` is allowed.
_BACKTICK_RE = re.compile(r"`([^`]+)`")
_SYMBOL_RE = re.compile(
    r"^[A-Za-z_][A-Za-z0-9_]*(?:::[A-Za-z_][A-Za-z0-9_]*)*(?:\(\.\.\.\)|\(\))?$"
)
_CAMELCASE_RE = re.compile(r"^[A-Z][A-Za-z0-9]+$")
# A repo-relative test path, however it is wrapped (bare or inside a URL).
_TEST_PATH_RE = re.compile(r"tests/[A-Za-z0-9_./-]+\.(?:cpp|cc|hpp|py|cu)")


def is_table_row(line: str) -> bool:
    return line.lstrip().startswith("|")


def available_rows(text: str) -> list[tuple[int, str]]:
    """Table rows whose status cell marks the claim ✅ available."""
    rows: list[tuple[int, str]] = []
    for lineno, line in enumerate(text.splitlines(), start=1):
        if is_table_row(line) and "✅" in line and "available" in line:
            rows.append((lineno, line))
    return rows


def header_symbols(line: str) -> list[str]:
    symbols: list[str] = []
    for token in _BACKTICK_RE.findall(line):
        token = token.strip()
        if not _SYMBOL_RE.match(token):
            continue
        base = token.split("(", 1)[0]
        if "::" in base or _CAMELCASE_RE.match(base):
            symbols.append(base)
    return symbols


def test_paths(line: str) -> list[str]:
    # De-duplicate while preserving order.
    seen: dict[str, None] = {}
    for match in _TEST_PATH_RE.findall(line):
        seen.setdefault(match, None)
    return list(seen)


def load_header_text(header_root: Path) -> str:
    parts: list[str] = []
    for hpp in sorted(header_root.rglob("*.hpp")):
        try:
            parts.append(hpp.read_text(encoding="utf-8", errors="ignore"))
        except OSError:
            continue
    return "\n".join(parts)


def symbol_resolves(symbol: str, header_text: str) -> bool:
    tail = symbol.split("::")[-1]
    return re.search(r"\b" + re.escape(tail) + r"\b", header_text) is not None


def check(doc: Path, header_root: Path) -> list[str]:
    errors: list[str] = []
    if not doc.is_file():
        return [f"{doc}: architecture page not found"]
    text = doc.read_text(encoding="utf-8")
    rows = available_rows(text)
    if not rows:
        return [
            f"{doc}: found no ✅-available rows — the marker/table format may "
            "have changed; update this checker."
        ]
    header_text = load_header_text(header_root)
    for lineno, line in rows:
        label = f"{doc.name}:{lineno}"
        symbols = header_symbols(line)
        tests = test_paths(line)
        if not symbols:
            errors.append(
                f"{label}: ✅-available row cites no public-header symbol "
                "(expected a backtick-quoted API symbol)."
            )
        elif not any(symbol_resolves(s, header_text) for s in symbols):
            errors.append(
                f"{label}: none of the cited symbols {symbols} resolve in "
                f"{header_root.name}/**/*.hpp."
            )
        if not tests:
            errors.append(
                f"{label}: ✅-available row cites no test "
                "(expected a tests/... path or link)."
            )
        else:
            missing = [t for t in tests if not (REPO_ROOT / t).is_file()]
            if missing:
                errors.append(f"{label}: cited test file(s) do not exist: {missing}.")
    return errors


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--doc", type=Path, default=DEFAULT_DOC)
    parser.add_argument("--header-root", type=Path, default=DEFAULT_HEADER_ROOT)
    args = parser.parse_args(argv)

    errors = check(args.doc, args.header_root)
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    count = len(available_rows(args.doc.read_text(encoding="utf-8")))
    print(f"Validated {count} available claim(s) in {args.doc.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
