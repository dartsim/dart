#!/usr/bin/env python3
"""Check that DART headers use `#pragma once` instead of include guards.

DART standardized on `#pragma once`: it is supported by every compiler in the
support matrix, cannot collide, and cannot drift out of sync with a renamed
file. This gate flags any header that reintroduces an
`#ifndef`/`#define`/`#endif` include guard.

Headers that are deliberately not guarded at all are out of scope: `*-impl.hpp`
files are included textually at the end of their owning header, and the
EnTT/Taskflow poison headers under tests/ must `#error` on every inclusion.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

HEADER_GLOBS = ("*.hpp", "*.h", "*.hh", "*.hxx", "*.cuh")

# Deliberately unguarded: textually included at the end of their owning header.
SKIP_SUFFIXES = ("-impl.hpp",)
# Poison headers must #error on every inclusion, so they carry no guard.
SKIP_DIR_PARTS = {"poison"}

# Vendored third-party headers keep their upstream form so they stay diffable
# against the source they were imported from.
ALLOWLIST = {
    # IKFast output; kept byte-compatible with the generator's template.
    "dart/dynamics/ikfast.h",
}
# Vendored upstream trees (Open Dynamics Engine, Copyright Russell L. Smith).
ALLOWLIST_PREFIXES = (
    "dart/math/lcp/pivoting/dantzig/",
    "tests/baseline/odelcpsolver/",
)

IFNDEF = re.compile(r"^[ \t]*#[ \t]*ifndef[ \t]+([A-Za-z_]\w*)[ \t]*$")
DEFINE = re.compile(r"^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)[ \t]*$")
PRAGMA_ONCE = re.compile(r"^[ \t]*#[ \t]*pragma[ \t]+once[ \t]*$")


@dataclass(frozen=True)
class Violation:
    path: str
    macro: str


def tracked_headers(root: Path) -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", *HEADER_GLOBS],
        cwd=root,
        capture_output=True,
        text=True,
        check=True,
    )
    out = []
    for rel in result.stdout.split():
        if rel in ALLOWLIST or rel.endswith(SKIP_SUFFIXES):
            continue
        if rel.startswith(ALLOWLIST_PREFIXES):
            continue
        if SKIP_DIR_PARTS & set(Path(rel).parts):
            continue
        if (root / rel).is_file():
            out.append(rel)
    return sorted(out)


def guard_macro(text: str) -> str | None:
    """Return the include-guard macro name, or None if the header has none."""
    lines = text.split("\n")
    for index, line in enumerate(lines):
        if PRAGMA_ONCE.match(line):
            return None
        opener = IFNDEF.match(line)
        if not opener:
            continue
        following = index + 1
        while following < len(lines) and lines[following].strip() == "":
            following += 1
        if following >= len(lines):
            return None
        definition = DEFINE.match(lines[following])
        if definition and definition.group(1) == opener.group(1):
            return opener.group(1)
        # A conditional that is not a guard; the header is not guard-wrapped.
        return None
    return None


def find_violations(root: Path) -> list[Violation]:
    violations = []
    for rel in tracked_headers(root):
        text = (root / rel).read_text(encoding="utf-8", errors="replace")
        macro = guard_macro(text)
        if macro is not None:
            violations.append(Violation(rel, macro))
    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    violations = find_violations(REPO_ROOT)
    if violations:
        print("header guard check failed: use `#pragma once` instead of include guards")
        for violation in violations:
            print(f"  - {violation.path}: replace `#ifndef {violation.macro}` guard")
        print(f"\n{len(violations)} violation(s)")
        return 1

    print("header guard check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
