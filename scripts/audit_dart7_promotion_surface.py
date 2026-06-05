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

History: the original keystone blocker was that
``dart/simulation/experimental/world.hpp`` declared an ``entt::registry`` data
member and included ``<entt/entt.hpp>`` directly, so any consumer of the public
``world.hpp`` needed the full EnTT type. The Workstream 5 opaque-ownership
refactor removed that member (``world.hpp`` now holds an opaque
``std::unique_ptr<detail::WorldStorage>``), so the leak set is empty: EnTT and
Taskflow are now PRIVATE dependencies and only the promoted public headers are
installed (see ``dart/simulation/experimental/CMakeLists.txt``).

This audit is now an enforcement gate. By default it is informational and
always exits 0; pass ``--strict`` (used in CI / ``pixi run
check-experimental-public-headers``) to exit nonzero if any promotion-target
header reintroduces an EnTT/Taskflow leak, which would re-expose those private
backends through the public surface.

The strict gate also cross-checks the CMake install allowlist (the
``dart_experimental_public_headers*`` lists in
``dart/simulation/experimental/CMakeLists.txt``) against the set of headers this
audit classifies as promotion targets, and fails on any asymmetric difference.
That closes a silent drift path: a header auto-promoted by the audit's
directory/file rules but not added to the install list would be audited yet
shipped non-self-contained (or vice versa).
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

# Specific headers under an otherwise-internal directory that ARE part of the
# promoted public surface. Keep this list minimal and explicit; the rest of the
# directory stays internal. These headers are themselves audited as promotion
# targets (they must not leak ECS). Reasons:
#   * diff/rollout.hpp is the public differentiable-simulation entry point
#     (diff::rollout / RolloutTrajectory / RolloutGradient), demonstrated by the
#     experimental_differentiable_gui example and intended for standalone use.
#     It is audited unconditionally (leak scan + allowlist cross-check), but the
#     CMake install ships it only under DART_BUILD_DIFF, since its implementation
#     (diff/rollout.cpp) is compiled only then; this static audit is
#     config-agnostic by design.
#   * diff/step_derivatives.hpp / diff/step_gradient.hpp are pulled into the
#     public include closure by value (world.hpp / rollout.hpp), so consumers
#     including only those headers need the complete types.
#   * compute/compute_stage_metadata.hpp carries backend-neutral stage domain
#     metadata. It is intentionally public because World step profiles report a
#     stage domain without exposing concrete executor/backend details.
#   * compute/world_step_profile.hpp is the experimental World's public
#     text-first profiling value type, returned by World::getLastStepProfile().
#   * compute/execution_profile.hpp holds the per-stage execution-timing value
#     types embedded in world_step_profile.hpp's public profile, so it is public
#     by inclusion; it is backend- and ECS-agnostic (std types only).
PROMOTE_FILES = {
    "compute/compute_stage_metadata.hpp",
    "compute/world_step_profile.hpp",
    "compute/execution_profile.hpp",
    "diff/rollout.hpp",
    "diff/step_derivatives.hpp",
    "diff/step_gradient.hpp",
}

# ECS-storage leak markers that must not appear in a promoted public header.
# Both includes and symbol usage count: handles expose ECS not only via
# includes but also via the EntityObject/EntityObjectWith template base and
# comps:: types named in the public class declaration.
LEAK_PATTERNS = {
    "entt-include": re.compile(r"#\s*include\s*<entt[/>]"),
    "entt-symbol": re.compile(r"\bentt::"),
    "getRegistry": re.compile(r"\bgetRegistry\b"),
    "comps-symbol": re.compile(r"\bcomps::"),
    "entity-object-base": re.compile(r"\bEntityObject(With)?\b"),
    # Taskflow is the other PRIVATE backend dependency: the documented intent is
    # that a promoted public header leaks neither EnTT NOR Taskflow. Gate both
    # the include and the tf:: symbol so a Taskflow type named in a public
    # declaration is caught the same way an entt:: type is.
    "taskflow-include": re.compile(r"#\s*include\s*<taskflow[/>]"),
    "taskflow-symbol": re.compile(r"\btf::"),
}


def classify(relpath: Path) -> str:
    """Return 'promote' or 'internal' for an experimental header path."""
    parts = relpath.parts
    if relpath.as_posix() in PROMOTE_FILES:
        return "promote"
    if len(parts) == 1:
        return "promote" if relpath.name in PROMOTE_TOPLEVEL else "internal"
    top = parts[0]
    if top in INTERNAL_DIRS:
        return "internal"
    if top in PROMOTE_DIRS:
        return "promote"
    return "internal"


def find_leaks(text: str) -> list[str]:
    """Return the sorted direct leak-marker names present in the header text."""
    names = [name for name, pat in LEAK_PATTERNS.items() if pat.search(text)]
    # A promoted header must not include an experimental header that classifies
    # as internal. This is computed via classify() rather than a hardcoded
    # directory regex so that headers explicitly promoted out of an internal
    # directory (PROMOTE_FILES, e.g. the diff value types returned by public
    # World methods) are not falsely flagged when a promoted header includes
    # them; every other internal include is still caught.
    for match in EXPERIMENTAL_INCLUDE.finditer(text):
        if classify(Path(match.group(1))) == "internal":
            names.append("internal-include")
            break
    return sorted(set(names))


# Include of another experimental header. Leaks are followed transitively
# through these: a promoted header is only clean if neither it NOR any
# experimental header it (transitively) includes leaks ECS — otherwise
# including the "clean" header still forces consumers to pull ECS internals
# (e.g. rigid_body.hpp -> frame/frame.hpp -> ecs/comps/entt).
EXPERIMENTAL_INCLUDE = re.compile(
    r"#\s*include\s*[<\"]dart/simulation/experimental/([^>\"]+)[>\"]"
)


def parse_experimental_includes(text: str, known: set[str]) -> set[str]:
    """Return experimental header relpaths (posix) that `text` includes."""
    return {
        m.group(1) for m in EXPERIMENTAL_INCLUDE.finditer(text) if m.group(1) in known
    }


def transitive_leak_sources(
    rel: str,
    direct: dict[str, list[str]],
    includes: dict[str, set[str]],
) -> dict[str, list[str]]:
    """Headers in rel's include-closure (incl. self) that directly leak -> leaks."""
    seen: set[str] = set()
    stack = [rel]
    found: dict[str, list[str]] = {}
    while stack:
        cur = stack.pop()
        if cur in seen:
            continue
        seen.add(cur)
        if direct.get(cur):
            found[cur] = direct[cur]
        stack.extend(includes.get(cur, ()))
    return found


# Path (relative to the repo root) of the CMake file that owns the single source
# of truth for the installed public-header allowlist.
EXPERIMENTAL_CMAKE = EXPERIMENTAL_ROOT / "CMakeLists.txt"

# Matches a `set(dart_experimental_public_headers[_<suffix>] ... )` block. The
# aggregate lists (dart_experimental_public_headers, DART_EXPERIMENTAL_PUBLIC_
# HEADERS) only reference other variables via ${...} and carry no literal .hpp
# tokens, so they contribute nothing to the parsed set; only the per-group
# leaf lists (…_toplevel/_body/…) hold the actual relpaths. Case-insensitive so
# the exported CACHE variable name is tolerated too.
_CMAKE_SET_BLOCK = re.compile(
    r"set\s*\(\s*(dart_experimental_public_headers\w*)\b(.*?)\)",
    re.IGNORECASE | re.DOTALL,
)
# A bare .hpp relpath token (e.g. body/rigid_body.hpp or world.hpp). Excludes
# ${...} variable references and CMake keywords, which never contain ".hpp".
_CMAKE_HPP_TOKEN = re.compile(r"[A-Za-z0-9_./-]+\.hpp")
# Strip CMake line comments (# to end-of-line) before tokenizing so a commented
# header is not counted as installed.
_CMAKE_COMMENT = re.compile(r"#[^\n]*")


def parse_cmake_install_allowlist(cmake_text: str) -> set[str]:
    """Return the installed public-header relpaths declared in the CMake file.

    Parses every ``set(dart_experimental_public_headers* ...)`` block and
    collects the literal ``*.hpp`` relpath tokens it contains. Robust to line
    comments, arbitrary whitespace/newlines, and the per-directory grouping; the
    aggregate variable lists contribute nothing because they only reference
    other variables (no literal ``.hpp`` tokens).
    """
    found: set[str] = set()
    for block in _CMAKE_SET_BLOCK.finditer(cmake_text):
        body = _CMAKE_COMMENT.sub("", block.group(2))
        for token in _CMAKE_HPP_TOKEN.findall(body):
            found.add(Path(token).as_posix())
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help=(
            "Exit nonzero if any promotion-target header leaks ECS storage. "
            "This is the active enforcement gate: the current tree is clean "
            "(0 leakers), so any new leak fails the check."
        ),
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    base = root / EXPERIMENTAL_ROOT
    if not base.is_dir():
        print(f"error: {EXPERIMENTAL_ROOT} not found (run from the repo root)")
        return 2

    # Read every experimental header; record its direct leaks and the
    # experimental headers it includes, so leaks can be followed transitively.
    all_rel: set[str] = {h.relative_to(base).as_posix() for h in base.rglob("*.hpp")}
    direct: dict[str, list[str]] = {}
    includes: dict[str, set[str]] = {}
    for rel in all_rel:
        text = (base / rel).read_text(encoding="utf-8")
        direct[rel] = find_leaks(text)
        includes[rel] = parse_experimental_includes(text, all_rel)

    promote_clean: list[str] = []
    # (rel, own_direct_leaks, {included_header: its_direct_leaks})
    promote_leaking: list[tuple[str, list[str], dict[str, list[str]]]] = []
    internal_count = 0
    for rel in sorted(all_rel):
        if classify(Path(rel)) == "internal":
            internal_count += 1
            continue
        sources = transitive_leak_sources(rel, direct, includes)
        own = sources.pop(rel, [])
        if own or sources:
            promote_leaking.append((rel, own, sources))
        else:
            promote_clean.append(rel)

    total_promote = len(promote_clean) + len(promote_leaking)
    print("DART 7 promotion-surface audit (PLAN-041)")
    print("=" * 60)
    print(f"experimental headers scanned : {len(all_rel)}")
    print(f"  promotion-target headers   : {total_promote}")
    print(f"    clean (no ECS leak)      : {len(promote_clean)}")
    print(f"    leaking ECS storage      : {len(promote_leaking)}")
    print(f"  internal headers (hidden)  : {internal_count}")
    print("  (leak = direct, or transitive via an included experimental header)")
    print()

    if promote_leaking:
        print("Promotion-target headers leaking ECS storage")
        print("(Workstream 5 must clear these before promotion):")
        for rel, own, via in promote_leaking:
            parts = []
            if own:
                parts.append(f"direct: {', '.join(own)}")
            for inc in sorted(via):
                parts.append(f"via {inc}: {', '.join(via[inc])}")
            print(f"  - {rel}  [{'; '.join(parts)}]")
        print()

    if promote_clean:
        print("Promotion-target headers already clean:")
        for rel in promote_clean:
            print(f"  - {rel}")
        print()

    if any(rel == "world.hpp" for rel, _own, _via in promote_leaking):
        print(
            "Keystone blocker: world.hpp holds an entt::registry member, so EnTT/"
            "Taskflow stay public deps and internals stay installed until the "
            "Workstream 5 opaque-ownership refactor removes it."
        )
    elif not promote_leaking:
        print(
            "Keystone cleared: world.hpp no longer exposes the EnTT registry "
            "(Workstream 5 opaque-ownership refactor complete); EnTT/Taskflow can "
            "move to private deps and the internal headers can leave the install set."
        )

    # Cross-check: the CMake install allowlist must EXACTLY equal the set of
    # headers this audit classifies as promotion targets. The audit promotes by
    # directory/file rule, while CMake installs a hardcoded filename list; they
    # agree today, but a future header auto-promoted by the rules (e.g. a new
    # body/*.hpp) but not added to the install list would be audited yet shipped
    # non-self-contained. Asserting set-equality closes that silent drift path.
    audit_promoted: set[str] = set(promote_clean)
    audit_promoted.update(rel for rel, _own, _via in promote_leaking)

    cmake_path = root / EXPERIMENTAL_CMAKE
    cmake_mismatch = False
    if not cmake_path.is_file():
        print()
        print(f"error: {EXPERIMENTAL_CMAKE} not found (cannot cross-check install set)")
        cmake_mismatch = True
    else:
        installed = parse_cmake_install_allowlist(
            cmake_path.read_text(encoding="utf-8")
        )
        missing_from_install = sorted(audit_promoted - installed)
        extra_in_install = sorted(installed - audit_promoted)
        print("Install-allowlist cross-check (CMake vs audit promotion rules)")
        print(f"  promoted by audit rules    : {len(audit_promoted)}")
        print(f"  installed by CMake list    : {len(installed)}")
        if missing_from_install or extra_in_install:
            cmake_mismatch = True
            if missing_from_install:
                print(
                    "  MISSING from install (audit-promoted but not installed -> "
                    "package would be non-self-contained):"
                )
                for rel in missing_from_install:
                    print(f"    - {rel}")
            if extra_in_install:
                print(
                    "  EXTRA in install (installed but not an audit promotion "
                    "target -> stale/over-broad allowlist):"
                )
                for rel in extra_in_install:
                    print(f"    + {rel}")
        else:
            print("  MATCH: install allowlist == audit promotion set")
    print()

    exit_code = 0
    if args.strict and promote_leaking:
        print(
            f"strict: {len(promote_leaking)} promotion-target header(s) still "
            "leak ECS storage"
        )
        exit_code = 1
    if cmake_mismatch:
        # Always a hard failure under --strict (the active CI gate); under the
        # default informational run it is printed above but does not change the
        # exit code, matching the leak-reporting contract.
        if args.strict:
            print(
                "strict: install allowlist and audit promotion set disagree "
                "(see diff above)"
            )
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
