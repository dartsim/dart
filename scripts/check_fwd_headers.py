#!/usr/bin/env python3
"""Check that DART forward declarations live in the owning namespace's fwd.hpp.

Every DART namespace that needs forward declarations owns a single ``fwd.hpp``
(``dart/simulation/fwd.hpp``, ``dart/gui/detail/fwd.hpp``, ...). Headers that
only need a declaration include that file instead of hand-rolling their own
``class Foo;``, so a type is declared in exactly one place.

This gate flags namespace-scope forward declarations that should have come from
a ``fwd.hpp``. Two things are legitimately declared in place and are not
reported:

* a primary template whose specializations follow in the same file, and any
  other entity that the same file goes on to define -- that is ordinary in-file
  ordering, not a cross-header declaration;
* file-local tag types that are never defined anywhere (see ``ALLOWLIST``).

Namespace is not the only handle: redeclaring a name that some ``fwd.hpp`` at or
above the file already declares is reported whatever the namespace, so
third-party and global-namespace types are covered once a forward header owns
them. Beyond that, third-party namespaces are out of scope, except for the
render/UI backends used by ``dart::gui``, which have their own forward header
(``dart/gui/detail/backend_fwd.hpp``) and are enforced against it.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

SCAN_ROOTS = ("dart", "python", "examples", "tutorials", "tests")
HEADER_SUFFIXES = (".hpp", ".hh", ".hxx")
SKIPPED_DIRS = {
    ".git",
    ".pixi",
    ".omc",
    "__pycache__",
    "build",
    "node_modules",
}

# Files that *are* forward-declaration headers.
FWD_BASENAMES = {"fwd.hpp", "backend_fwd.hpp"}

# (namespace, name) pairs that are declared but never defined anywhere, and are
# used only by the single file that declares them. Keep this list short; a new
# entry needs a comment saying why the type has no definition.
ALLOWLIST = {
    # Tag type used purely to disambiguate inverseImpl<> specializations.
    ("dart::math::detail", "Range"),
    # Vestigial aspect name kept for the DART_BAKE_SPECIALIZED_ASPECT macros.
    ("dart::dynamics::detail", "SoftBodyAspect"),
    # tests/dart/test/reference_collision is a frozen copy of the legacy
    # collision backends; these names have no definition in the tree.
    ("dart::collision", "FCLCollisionObject"),
    ("dart::collision", "FCLCollisionObjectUserData"),
    ("dart::collision::detail", "OdeGeom"),
}

# Third-party namespaces that dart::gui declares through backend_fwd.hpp.
GUI_BACKEND_NAMESPACES = {"filament"}
GUI_BACKEND_GLOBALS = {"GLFWwindow", "ImDrawData", "ImGuiIO"}

FWD_RE = re.compile(
    r"^(?:template\s*<(?P<tparams>.*)>\s*)?"
    r"(?P<kind>class|struct|union|enum\s+class|enum\s+struct|enum)\s+"
    r"(?:DART_[A-Z0-9_]*API\s+)?"
    r"(?P<name>[A-Za-z_][A-Za-z0-9_]*)"
    r"(?:\s*:\s*[^;]*)?$"
)


@dataclass(frozen=True)
class Violation:
    path: str
    namespace: str
    name: str
    message: str


def _strip_noise(src: str) -> str:
    """Blank out comments, string/char literals and preprocessor directives.

    Newlines are preserved so line numbers stay meaningful. Preprocessor
    directives are dropped including backslash continuations, so declarations
    inside macro bodies are not mistaken for real ones.
    """
    out: list[str] = []
    i = 0
    n = len(src)
    at_line_start = True
    while i < n:
        char = src[i]
        if char == "\n":
            out.append("\n")
            i += 1
            at_line_start = True
            continue
        if at_line_start and char in " \t":
            out.append(char)
            i += 1
            continue
        if at_line_start and char == "#":
            # Consume the whole directive, following line continuations.
            while i < n:
                if src[i] == "\\" and i + 1 < n and src[i + 1] == "\n":
                    out.append(" \n")
                    i += 2
                    continue
                if src[i] == "\n":
                    break
                out.append(" ")
                i += 1
            continue
        at_line_start = False
        if char == "/" and i + 1 < n and src[i + 1] == "/":
            while i < n and src[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if char == "/" and i + 1 < n and src[i + 1] == "*":
            out.append("  ")
            i += 2
            while i < n and not (src[i] == "*" and i + 1 < n and src[i + 1] == "/"):
                out.append("\n" if src[i] == "\n" else " ")
                i += 1
            out.append("  ")
            i += 2
            continue
        if char == '"':
            out.append(" ")
            i += 1
            while i < n and src[i] != '"':
                if src[i] == "\\":
                    out.append(" ")
                    i += 1
                out.append("\n" if i < n and src[i] == "\n" else " ")
                i += 1
            out.append(" ")
            i += 1
            continue
        if char == "'":
            end = i + 1
            while end < n and src[end] not in ("'", "\n"):
                if src[end] == "\\":
                    end += 1
                end += 1
            if end < n and src[end] == "'" and end - i <= 5:
                out.append(" " * (end - i + 1))
                i = end + 1
                continue
        out.append(char)
        i += 1
    return "".join(out)


def _iter_files(root: Path, suffixes: tuple[str, ...]):
    for path in sorted(root.rglob("*")):
        if not path.is_file() or path.suffix not in suffixes:
            continue
        if any(part in SKIPPED_DIRS for part in path.parts):
            continue
        yield path


def _defined_names(clean: str) -> set[str]:
    """Names the file defines or specializes (so an in-file declaration is fine)."""
    names: set[str] = set()
    definition = re.compile(
        r"^[ \t]*(?:class|struct|union|enum(?:\s+class|\s+struct)?)\s+"
        r"(?:DART_[A-Z0-9_]*API\s+)?"
        r"([A-Za-z_][A-Za-z0-9_]*)"
        r"(?![ \t]*;)",
        re.MULTILINE,
    )
    for match in definition.finditer(clean):
        names.add(match.group(1))
    return names


def _namespace_scope_decls(clean: str):
    """Yield (namespace, kind, name) for declarations at namespace scope."""
    scopes: list[tuple[str, str | None]] = []
    pending = 0
    i = 0
    n = len(clean)
    while i < n:
        char = clean[i]
        if char == "{":
            frag = " ".join(clean[pending:i].split())
            named = re.search(r"\bnamespace\s+([A-Za-z_][A-Za-z0-9_:]*)\s*$", frag)
            anonymous = re.search(r"\bnamespace\s*$", frag)
            if named:
                scopes.append(("ns", named.group(1)))
            elif anonymous:
                scopes.append(("ns", "<anonymous>"))
            else:
                scopes.append(("other", None))
            pending = i + 1
        elif char == "}":
            if scopes:
                scopes.pop()
            pending = i + 1
        elif char == ";":
            frag = " ".join(clean[pending:i].split())
            if frag and all(kind == "ns" for kind, _ in scopes):
                match = FWD_RE.match(frag)
                if match:
                    namespace = "::".join(name for _, name in scopes if name)
                    yield namespace, re.sub(
                        r"\s+", " ", match.group("kind")
                    ), match.group("name")
            pending = i + 1
        i += 1


def _fwd_header_index(root: Path) -> dict[Path, dict[str, str]]:
    """Map each fwd header's directory to {declared name: header path}."""
    index: dict[Path, dict[str, str]] = {}
    for scan_root in SCAN_ROOTS:
        base = root / scan_root
        if not base.is_dir():
            continue
        for path in _iter_files(base, HEADER_SUFFIXES):
            if path.name not in FWD_BASENAMES:
                continue
            clean = _strip_noise(path.read_text(encoding="utf-8", errors="replace"))
            rel = path.relative_to(root).as_posix()
            bucket = index.setdefault(path.parent, {})
            for _namespace, _kind, name in _namespace_scope_decls(clean):
                bucket.setdefault(name, rel)
    return index


def _reachable_fwd_owner(
    path: Path, name: str, root: Path, index: dict[Path, dict[str, str]]
) -> str | None:
    """The nearest fwd header, at or above `path`, that already declares `name`."""
    directory = path.parent
    while True:
        owner = index.get(directory, {}).get(name)
        if owner is not None:
            return owner
        if directory == root or directory.parent == directory:
            return None
        directory = directory.parent


def find_violations(root: Path) -> list[Violation]:
    index = _fwd_header_index(root)
    violations: list[Violation] = []
    for scan_root in SCAN_ROOTS:
        base = root / scan_root
        if not base.is_dir():
            continue
        for path in _iter_files(base, HEADER_SUFFIXES):
            if path.name in FWD_BASENAMES:
                continue
            rel = path.relative_to(root).as_posix()
            clean = _strip_noise(path.read_text(encoding="utf-8", errors="replace"))
            defined = _defined_names(clean)
            for namespace, _kind, name in _namespace_scope_decls(clean):
                if name in defined:
                    continue  # defined or specialized in this same file
                if (namespace, name) in ALLOWLIST:
                    continue
                root_ns = namespace.split("::")[0]
                owner = _reachable_fwd_owner(path, name, root, index)
                if owner is not None:
                    # A fwd header at or above this file already declares it,
                    # whatever the namespace (covers third-party and global too).
                    violations.append(
                        Violation(
                            rel,
                            namespace or "(global)",
                            name,
                            f"{owner} already declares it; include that header instead",
                        )
                    )
                elif root_ns == "dart":
                    violations.append(
                        Violation(
                            rel,
                            namespace,
                            name,
                            f"declare it in {_fwd_header_for(namespace, root)} and "
                            "include that header instead",
                        )
                    )
                elif rel.startswith("dart/gui/") and (
                    root_ns in GUI_BACKEND_NAMESPACES
                    or (namespace == "" and name in GUI_BACKEND_GLOBALS)
                ):
                    violations.append(
                        Violation(
                            rel,
                            namespace or "(global)",
                            name,
                            "declare it in dart/gui/detail/backend_fwd.hpp and "
                            "include that header instead",
                        )
                    )
    return violations


def _fwd_header_for(namespace: str, root: Path) -> str:
    """Best-guess fwd.hpp path for a dart:: namespace, longest match first."""
    parts = namespace.split("::")
    while len(parts) > 1:
        candidate = Path(*parts) / "fwd.hpp"
        if (root / candidate).is_file():
            return candidate.as_posix()
        parts = parts[:-1]
    return "the owning namespace's fwd.hpp"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()

    violations = find_violations(REPO_ROOT)
    if violations:
        print("fwd.hpp check failed: forward declarations outside a fwd.hpp")
        for violation in violations:
            scope = violation.namespace or "(global)"
            print(
                f"  - {violation.path}: {scope}::{violation.name} — {violation.message}"
            )
        print(f"\n{len(violations)} violation(s)")
        return 1

    print("fwd.hpp check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
