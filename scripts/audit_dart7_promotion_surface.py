#!/usr/bin/env python3
"""Audit the DART 7 simulation API promotion surface (PLAN-041).

Report-only inventory of how far the experimental simulation headers under
``dart/simulation/experimental/`` are from a promotable public surface. It
classifies each header as a promotion target or an internal header, then flags
ECS-storage leaks (EnTT includes, ``entt::`` usage, ``getRegistry``, and direct
``comps``/``ecs`` includes) in the promotion-target headers.

This operationalizes ``docs/design/dart7_promotion_readiness_audit.md`` (the
PLAN-041 Workstream 1 readiness audit) as a runnable check so the inventory
does not drift.

Keystone finding: ``dart/simulation/experimental/world.hpp`` declares an
``entt::registry`` data member and includes ``<entt/entt.hpp>`` directly, so any
consumer of the public ``world.hpp`` needs the full EnTT type. EnTT/Taskflow
therefore cannot become private package dependencies, and the internal headers
cannot be removed from the install set, until the opaque-ownership/pimpl
refactor (Workstream 5) removes that member. Run this audit to see the current
leak set that Workstream 5 must clear.

By default this script is informational and always exits 0. Pass ``--strict``
to exit nonzero when any promotion-target header still leaks ECS storage; that
mode is intended for a future Workstream 3 enforcement gate, once the handles
have been cleaned, not for the current tree.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

EXPERIMENTAL_ROOT = Path("dart/simulation/experimental")

# Directories whose headers are implementation-internal and are expected to be
# hidden from the promoted public surface (never installed, never included by a
# promoted header). See the readiness audit for the rationale.
INTERNAL_DIRS = (
    "comps",
    "compute",
    "common",
    "detail",
    "diff",
    "ecs",
    "io",
    "space",
)

# Top-level experimental headers that are part of the promoted public surface.
PROMOTE_TOPLEVEL = {
    "world.hpp",
    "world_options.hpp",
    "world_sync_stage.hpp",
    "entity.hpp",
    "fwd.hpp",
    "export.hpp",
    "version.hpp",
}

# Subdirectories whose headers carry promotion-target handles/value objects.
PROMOTE_DIRS = (
    "body",
    "multibody",
    "frame",
    "constraint",
)

# ECS-storage leak markers that must not appear in a promoted public header.
# Both includes and symbol usage count: handles expose ECS not only via
# includes but also via the EntityObject/EntityObjectWith template base and
# comps:: types named in the public class declaration.
LEAK_PATTERNS = {
    "entt-include": re.compile(r"#\s*include\s*<entt[/>]"),
    "entt-symbol": re.compile(r"\bentt::"),
    "getRegistry": re.compile(r"\bgetRegistry\b"),
    # Any include of an internal-only experimental directory is a leak in a
    # promoted header. Derived from INTERNAL_DIRS so every internal path is
    # covered (comps/ecs/detail/diff/compute/io/common/space), not just a
    # hardcoded subset: a promoted header must not require hidden internals.
    "internal-include": re.compile(
        r"#\s*include\s*[<\"]dart/simulation/experimental/("
        + "|".join(INTERNAL_DIRS)
        + r")/"
    ),
    "comps-symbol": re.compile(r"\bcomps::"),
    "entity-object-base": re.compile(r"\bEntityObject(With)?\b"),
}


def classify(relpath: Path) -> str:
    """Return 'promote' or 'internal' for an experimental header path."""
    parts = relpath.parts
    if len(parts) == 1:
        return "promote" if relpath.name in PROMOTE_TOPLEVEL else "internal"
    top = parts[0]
    if top in INTERNAL_DIRS:
        return "internal"
    if top in PROMOTE_DIRS:
        return "promote"
    return "internal"


def find_leaks(text: str) -> list[str]:
    """Return the sorted leak-marker names present in the header text."""
    return sorted(name for name, pat in LEAK_PATTERNS.items() if pat.search(text))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help=(
            "Exit nonzero if any promotion-target header leaks ECS storage "
            "(future Workstream 3 enforcement; the current tree is expected "
            "to leak)."
        ),
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    base = root / EXPERIMENTAL_ROOT
    if not base.is_dir():
        print(f"error: {EXPERIMENTAL_ROOT} not found (run from the repo root)")
        return 2

    promote_clean: list[Path] = []
    promote_leaking: list[tuple[Path, list[str]]] = []
    internal_count = 0

    for header in sorted(base.rglob("*.hpp")):
        rel = header.relative_to(base)
        kind = classify(rel)
        if kind == "internal":
            internal_count += 1
            continue
        leaks = find_leaks(header.read_text(encoding="utf-8"))
        if leaks:
            promote_leaking.append((rel, leaks))
        else:
            promote_clean.append(rel)

    total_promote = len(promote_clean) + len(promote_leaking)
    print("DART 7 promotion-surface audit (PLAN-041)")
    print("=" * 60)
    print(f"experimental headers scanned : {total_promote + internal_count}")
    print(f"  promotion-target headers   : {total_promote}")
    print(f"    clean (no ECS leak)      : {len(promote_clean)}")
    print(f"    leaking ECS storage      : {len(promote_leaking)}")
    print(f"  internal headers (hidden)  : {internal_count}")
    print()

    if promote_leaking:
        print("Promotion-target headers leaking ECS storage")
        print("(Workstream 5 must clear these before promotion):")
        for rel, leaks in promote_leaking:
            print(f"  - {rel}  [{', '.join(leaks)}]")
        print()

    if promote_clean:
        print("Promotion-target headers already clean:")
        for rel in promote_clean:
            print(f"  - {rel}")
        print()

    print(
        "Keystone blocker: world.hpp holds an entt::registry member, so EnTT/"
        "Taskflow stay public deps and internals stay installed until the "
        "Workstream 5 opaque-ownership refactor removes it."
    )

    if args.strict and promote_leaking:
        print()
        print(
            f"strict: {len(promote_leaking)} promotion-target header(s) still "
            "leak ECS storage"
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
